// SPDX-License-Identifier: GPL-2.0-only
/* Q2 CST816T wheel firmware protocol, derived from stock hyn_slider.ko. */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>

struct q2_wheel {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *reset;
	ktime_t ready_at;
	bool irq_masked;
};

/* Stock hyn_wr_reg uses separate STOP-terminated transfers, not SMBus's
 * combined repeated-START. Hold the segment across the two calls so another
 * client cannot replace the register pointer. Retry the complete read once.
 */
static int q2_wheel_read_reg(struct q2_wheel *wheel, u8 reg)
{
	struct i2c_client *client = wheel->client;
	u8 value = 0;
	struct i2c_msg message = {
		.addr = client->addr,
		.len = 1,
	};
	int attempt, ret = -EIO;

	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	for (attempt = 0; attempt < 2; attempt++) {
		message.flags = 0;
		message.buf = &reg;
		ret = __i2c_transfer(client->adapter, &message, 1);
		if (ret == 1) {
			message.flags = I2C_M_RD;
			message.buf = &value;
			ret = __i2c_transfer(client->adapter, &message, 1);
			if (ret == 1) {
				ret = value;
				break;
			}
		}
		if (ret >= 0)
			ret = -EIO;
	}
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	return ret;
}

static int q2_wheel_normal_mode(struct q2_wheel *wheel)
{
	const u8 wake = 0x00;
	const u8 normal[] = { 0xff, 0x00 };
	int ret;

	/* Stock cst8xxT_set_workmode(0): 00, wait 1 ms, then ff 00.
	 * No boot address, firmware command or flash write is involved.
	 */
	ret = i2c_master_send(wheel->client, &wake, sizeof(wake));
	if (ret != (int)sizeof(wake))
		return ret < 0 ? ret : -EIO;
	usleep_range(1000, 2000);
	ret = i2c_master_send(wheel->client, normal, sizeof(normal));
	return ret == (int)sizeof(normal) ? 0 : ret < 0 ? ret : -EIO;
}

static void q2_wheel_release(struct q2_wheel *wheel)
{
	/* Cancel a gesture whose physical state is no longer readable. */
	input_report_abs(wheel->input, ABS_PRESSURE, 0);
	input_report_key(wheel->input, BTN_TOUCH, 0);
	input_sync(wheel->input);
}

static irqreturn_t q2_wheel_irq(int irq, void *data)
{
	struct q2_wheel *wheel = data;
	int pos;

	pos = q2_wheel_read_reg(wheel, 0x21);
	if (pos < 0) {
		dev_warn_ratelimited(&wheel->client->dev, "position read failed: %d\n", pos);
		q2_wheel_release(wheel);
		return IRQ_HANDLED;
	}
	if (pos > 200) {
		dev_warn_ratelimited(&wheel->client->dev, "invalid position: %d\n", pos);
		q2_wheel_release(wheel);
		return IRQ_HANDLED;
	}
	if (pos)
		input_report_abs(wheel->input, ABS_X, pos);
	input_report_abs(wheel->input, ABS_PRESSURE, !!pos);
	input_report_key(wheel->input, BTN_TOUCH, !!pos);
	input_sync(wheel->input);
	return IRQ_HANDLED;
}

static void q2_wheel_reset(void *data)
{
	struct q2_wheel *wheel = data;

	gpiod_set_value_cansleep(wheel->reset, 1);
}

static int q2_wheel_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct q2_wheel *wheel;
	int chip, project, ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "interrupt is required\n");
	wheel = devm_kzalloc(dev, sizeof(*wheel), GFP_KERNEL);
	if (!wheel)
		return -ENOMEM;
	wheel->client = client;
	wheel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(wheel->reset))
		return dev_err_probe(dev, PTR_ERR(wheel->reset), "reset GPIO unavailable\n");
	ret = devm_add_action_or_reset(dev, q2_wheel_reset, wheel);
	if (ret)
		return ret;

	/* Stock cst8xxT_rst asserts PB20 for 11 ms. Allow application startup;
	 * do not enter the boot address or attempt a firmware update.
	 */
	usleep_range(11000, 12000);
	gpiod_set_value_cansleep(wheel->reset, 0);
	msleep(50);
	chip = q2_wheel_read_reg(wheel, 0xa7);
	if (chip < 0)
		return dev_err_probe(dev, chip, "chip identity read failed\n");
	project = q2_wheel_read_reg(wheel, 0xa8);
	if (project < 0)
		return dev_err_probe(dev, project, "project identity read failed\n");
	/* These are application-firmware bytes, not a fixed silicon identity.
	 * Physical Q2 firmware returns 00/00 while reporting valid positions.
	 * The emulator's b6/02 values must not restrict this DT-bound device.
	 */
	dev_dbg(dev, "application identity %02x/%02x\n", chip, project);
	ret = q2_wheel_normal_mode(wheel);
	if (ret)
		return dev_err_probe(dev, ret, "normal work mode failed\n");

	wheel->input = devm_input_allocate_device(dev);
	if (!wheel->input)
		return -ENOMEM;
	/* Preserve the existing Q2 userspace discovery name. */
	wheel->input->name = "hyn_sliderbar";
	wheel->input->id.bustype = BUS_I2C;
	input_set_abs_params(wheel->input, ABS_X, 1, 200, 0, 0);
	input_set_abs_params(wheel->input, ABS_PRESSURE, 0, 1, 0, 0);
	input_set_capability(wheel->input, EV_KEY, BTN_TOUCH);
	ret = input_register_device(wheel->input);
	if (ret)
		return ret;
	ret = devm_request_threaded_irq(dev, client->irq, NULL, q2_wheel_irq,
					IRQF_ONESHOT, dev_name(dev), wheel);
	if (ret)
		return dev_err_probe(dev, ret, "interrupt request failed\n");
	i2c_set_clientdata(client, wheel);
	return 0;
}

static void q2_wheel_shutdown(struct i2c_client *client)
{
	disable_irq(client->irq);
	q2_wheel_reset(i2c_get_clientdata(client));
}

static int q2_wheel_suspend(struct device *dev)
{
	struct q2_wheel *wheel = dev_get_drvdata(dev);
	bool was_masked = wheel->irq_masked;
	int ret;

	wheel->ready_at = 0;
	if (!was_masked) {
		disable_irq(wheel->client->irq);
		wheel->irq_masked = true;
	}
	q2_wheel_release(wheel);

	ret = gpiod_set_value_cansleep(wheel->reset, 1);
	if (ret) {
		if (!was_masked) {
			wheel->irq_masked = false;
			enable_irq(wheel->client->irq);
		}
		return ret;
	}

	/* Complete the stock reset pulse minimum before the platform enters sleep. */
	usleep_range(11000, 12000);
	return 0;
}

static int q2_wheel_resume_early(struct device *dev)
{
	struct q2_wheel *wheel = dev_get_drvdata(dev);
	int ret;

	if (!wheel->irq_masked || wheel->ready_at)
		return 0;

	/* PB20 is on the SoC GPIO controller. Leave the I2C transfer to resume. */
	ret = gpiod_set_value_cansleep(wheel->reset, 0);
	if (ret)
		return ret;

	wheel->ready_at = ktime_add_ms(ktime_get(), 50);
	return 0;
}

static int q2_wheel_resume(struct device *dev)
{
	struct q2_wheel *wheel = dev_get_drvdata(dev);
	s64 remaining;
	int ret;

	if (!wheel->irq_masked)
		return 0;

	/* Abort paths can resume without ever reaching the early callback. */
	ret = q2_wheel_resume_early(dev);
	if (ret)
		return ret;

	remaining = ktime_sub(wheel->ready_at, ktime_get());
	if (remaining > 0) {
		unsigned long usecs = DIV_ROUND_UP_ULL(remaining, NSEC_PER_USEC);

		usleep_range(usecs, usecs + 1000);
	}

	ret = q2_wheel_normal_mode(wheel);
	if (ret)
		return ret;

	wheel->irq_masked = false;
	enable_irq(wheel->client->irq);
	return 0;
}

static const struct dev_pm_ops q2_wheel_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(q2_wheel_suspend, q2_wheel_resume)
	LATE_SYSTEM_SLEEP_PM_OPS(NULL, q2_wheel_resume_early)
};

static const struct of_device_id q2_wheel_of_match[] = {
	{ .compatible = "shanling,q2-wheel" },
	{ }
};
MODULE_DEVICE_TABLE(of, q2_wheel_of_match);

static struct i2c_driver q2_wheel_driver = {
	.probe = q2_wheel_probe,
	.shutdown = q2_wheel_shutdown,
	.driver = {
		.name = "shanling-q2-wheel",
		.pm = pm_sleep_ptr(&q2_wheel_pm_ops),
		.of_match_table = q2_wheel_of_match,
	},
};
module_i2c_driver(q2_wheel_driver);

MODULE_DESCRIPTION("Shanling Q2 CST816T wheel firmware input");
MODULE_LICENSE("GPL");
