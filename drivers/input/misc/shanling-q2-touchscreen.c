// SPDX-License-Identifier: GPL-2.0-only
/* Shanling Q2 panel CST8xxT application protocol. */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#define Q2_TOUCH_MAX_X 319
#define Q2_TOUCH_MAX_Y 374
#define Q2_TOUCH_REPORT_SIZE 15
#define Q2_TOUCH_CANCEL 1
#define Q2_TOUCH_ATTEMPTS 3

struct q2_touchscreen {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *reset;
	struct regulator *vdd;
	struct mutex mutex;
	bool opened;
	bool suspended;
	bool needs_release;
	bool ready;
	bool powered;
	bool idle;
	u8 identity[4];
	unsigned int epoch;
	int error;
	unsigned int startup_reports;
	atomic_t irq_reports, invalid_reports, read_errors;
};

static void q2_touch_contact(struct input_dev *input, bool down)
{
	input_report_abs(input, ABS_PRESSURE, down);
	input_report_key(input, BTN_TOUCH, down);
	input_sync(input);
}

/* MSC_RAW tells the evdev reader to discard any tap in progress. */
static void q2_touch_cancel(struct input_dev *input)
{
	input_event(input, EV_MSC, MSC_RAW, Q2_TOUCH_CANCEL);
	q2_touch_contact(input, false);
}

static int q2_touch_send(struct q2_touchscreen *touch, const u8 *data, size_t len)
{
	int ret = i2c_master_send(touch->client, data, len);

	return ret == len ? 0 : ret < 0 ? ret : -EIO;
}

static int q2_touch_read(struct q2_touchscreen *touch, u8 reg, u8 *data, size_t len)
{
	int ret;

	/* Stock uses a STOP between sending the register and reading it. */
	ret = q2_touch_send(touch, &reg, sizeof(reg));
	if (ret)
		return ret;
	ret = i2c_master_recv(touch->client, data, len);
	return ret == len ? 0 : ret < 0 ? ret : -EIO;
}

struct q2_touch_frame {
	unsigned int x, y;
	bool down;
};

static int q2_touch_frame(struct q2_touchscreen *touch, struct q2_touch_frame *frame)
{
	u8 report[Q2_TOUCH_REPORT_SIZE];
	unsigned int contacts;
	int ret;

	ret = q2_touch_read(touch, 0, report, sizeof(report));
	if (ret) {
		atomic_inc(&touch->read_errors);
		return ret;
	}

	contacts = report[2] & 0xf;
	if (contacts > 2)
		goto invalid;
	/* Stock treats bit 6 as the release marker. */
	frame->down = contacts && !(report[3] & BIT(6));
	if (!frame->down)
		return 0;

	frame->x = ((report[3] & 0xf) << 8) | report[4];
	frame->y = ((report[5] & 0xf) << 8) | report[6];
	/* The retail driver admits both inclusive panel dimensions. */
	if (frame->x > Q2_TOUCH_MAX_X + 1 || frame->y > Q2_TOUCH_MAX_Y + 1)
		goto invalid;
	frame->x = min_t(unsigned int, frame->x, Q2_TOUCH_MAX_X);
	frame->y = min_t(unsigned int, frame->y, Q2_TOUCH_MAX_Y);
	return 0;

invalid:
	atomic_inc(&touch->invalid_reports);
	return -EPROTO;
}

static irqreturn_t q2_touch_irq(int irq, void *data)
{
	struct q2_touchscreen *touch = data;
	struct q2_touch_frame frame;
	int ret = q2_touch_frame(touch, &frame);

	if (ret) {
		WRITE_ONCE(touch->idle, false);
		dev_warn_ratelimited(&touch->client->dev, "panel report failed: %d\n", ret);
		if (!touch->needs_release) {
			touch->needs_release = true;
			q2_touch_cancel(touch->input);
		}
		return IRQ_HANDLED;
	}

	atomic_inc(&touch->irq_reports);
	WRITE_ONCE(touch->idle, !frame.down);
	if (!frame.down) {
		touch->needs_release = false;
		q2_touch_contact(touch->input, false);
	} else if (!touch->needs_release) {
		input_report_abs(touch->input, ABS_X, frame.x);
		input_report_abs(touch->input, ABS_Y, frame.y);
		q2_touch_contact(touch->input, true);
	}
	return IRQ_HANDLED;
}

static int q2_touch_init_panel(struct q2_touchscreen *touch)
{
	const u8 normal_mode[] = { 0xe5, 0x00 };
	const u8 probe_reg = 0;
	struct q2_touch_frame frame;
	int ret;

	gpiod_set_value_cansleep(touch->reset, 1);
	msleep(11);
	gpiod_set_value_cansleep(touch->reset, 0);
	msleep(50);

	/* Stock normal-mode entry also runs after every resume/reset. */
	ret = q2_touch_send(touch, &probe_reg, sizeof(probe_reg));
	if (!ret)
		ret = q2_touch_send(touch, normal_mode, sizeof(normal_mode));
	if (!ret)
		ret = q2_touch_read(touch, 0xa7, touch->identity, sizeof(touch->identity));
	if (ret) {
		atomic_inc(&touch->read_errors);
		return ret;
	}
	/* Keep actual component information even if the report then fails. */
	if (!memchr_inv(touch->identity, 0, sizeof(touch->identity)) ||
	    !memchr_inv(touch->identity, 0xff, sizeof(touch->identity)))
		return -ENODEV;

	ret = q2_touch_frame(touch, &frame);
	if (ret)
		return ret;
	touch->startup_reports++;
	/* A contact already present at open is not a new tap. */
	touch->needs_release = frame.down;
	WRITE_ONCE(touch->idle, !frame.down);
	return 0;
}

static void q2_touch_power_off(struct q2_touchscreen *touch)
{
	int ret;

	gpiod_set_value_cansleep(touch->reset, 1);
	if (!touch->powered)
		return;

	ret = regulator_disable(touch->vdd);
	if (ret) {
		dev_warn(&touch->client->dev, "panel supply disable failed: %d\n", ret);
		return;
	}
	touch->powered = false;
}

static int q2_touch_start(struct q2_touchscreen *touch)
{
	int attempt;

	touch->epoch++;
	touch->startup_reports = 0;
	WRITE_ONCE(touch->idle, false);
	memset(touch->identity, 0, sizeof(touch->identity));
	atomic_set(&touch->irq_reports, 0);
	atomic_set(&touch->invalid_reports, 0);
	atomic_set(&touch->read_errors, 0);

	if (!touch->powered) {
		gpiod_set_value_cansleep(touch->reset, 1);
		touch->error = regulator_enable(touch->vdd);
		if (touch->error)
			return touch->error;
		touch->powered = true;
	}

	for (attempt = 0; attempt < Q2_TOUCH_ATTEMPTS; attempt++) {
		memset(touch->identity, 0, sizeof(touch->identity));
		touch->error = q2_touch_init_panel(touch);
		if (!touch->error) {
			touch->ready = true;
			enable_irq(touch->client->irq);
			return 0;
		}
	}
	q2_touch_power_off(touch);
	return touch->error;
}

static void q2_touch_stop(struct q2_touchscreen *touch)
{
	if (touch->ready)
		disable_irq(touch->client->irq);
	touch->ready = false;
	WRITE_ONCE(touch->idle, false);
	q2_touch_power_off(touch);
	q2_touch_cancel(touch->input);
}

static int q2_touch_open(struct input_dev *input)
{
	struct q2_touchscreen *touch = input_get_drvdata(input);
	int ret = 0;

	mutex_lock(&touch->mutex);
	if (!touch->suspended)
		ret = q2_touch_start(touch);
	touch->opened = !ret;
	mutex_unlock(&touch->mutex);
	return ret;
}

static void q2_touch_close(struct input_dev *input)
{
	struct q2_touchscreen *touch = input_get_drvdata(input);

	mutex_lock(&touch->mutex);
	if (!touch->suspended)
		q2_touch_stop(touch);
	touch->opened = false;
	mutex_unlock(&touch->mutex);
}

static int q2_touch_suspend(struct device *dev)
{
	struct q2_touchscreen *touch = dev_get_drvdata(dev);

	mutex_lock(&touch->mutex);
	if (touch->opened)
		q2_touch_stop(touch);
	touch->suspended = true;
	mutex_unlock(&touch->mutex);
	return 0;
}

static int q2_touch_resume(struct device *dev)
{
	struct q2_touchscreen *touch = dev_get_drvdata(dev);

	mutex_lock(&touch->mutex);
	touch->suspended = false;
	if (touch->opened && q2_touch_start(touch))
		dev_warn(dev, "panel resume failed: %d\n", touch->error);
	mutex_unlock(&touch->mutex);
	return 0;
}

static ssize_t panel_status_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct q2_touchscreen *touch = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&touch->mutex);
	len = sysfs_emit(buf,
			 "epoch=%u ready=%u powered=%u idle=%u error=%d identity=%4phN startup_reports=%u irq_reports=%d invalid_reports=%d read_errors=%d\n",
			 touch->epoch, touch->ready, touch->powered, READ_ONCE(touch->idle),
			 touch->error, touch->identity,
			 touch->startup_reports, atomic_read(&touch->irq_reports),
			 atomic_read(&touch->invalid_reports), atomic_read(&touch->read_errors));
	mutex_unlock(&touch->mutex);
	return len;
}
static DEVICE_ATTR_RO(panel_status);

static struct attribute *q2_touch_attrs[] = {
	&dev_attr_panel_status.attr,
	NULL,
};
ATTRIBUTE_GROUPS(q2_touch);

static DEFINE_SIMPLE_DEV_PM_OPS(q2_touch_pm, q2_touch_suspend, q2_touch_resume);

static int q2_touch_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct q2_touchscreen *touch;
	struct input_dev *input;
	int ret;

	touch = devm_kzalloc(dev, sizeof(*touch), GFP_KERNEL);
	if (!touch)
		return -ENOMEM;
	touch->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(touch->reset))
		return dev_err_probe(dev, PTR_ERR(touch->reset), "panel reset GPIO\n");

	touch->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(touch->vdd))
		return dev_err_probe(dev, PTR_ERR(touch->vdd), "panel supply\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	if (client->irq <= 0)
		return -EINVAL;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	touch->client = client;
	touch->input = input;
	mutex_init(&touch->mutex);
	i2c_set_clientdata(client, touch);
	input_set_drvdata(input, touch);

	input->name = "Q2 touchscreen";
	input->id.bustype = BUS_I2C;
	input->open = q2_touch_open;
	input->close = q2_touch_close;
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_capability(input, EV_MSC, MSC_RAW);
	input_set_abs_params(input, ABS_X, 0, Q2_TOUCH_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, Q2_TOUCH_MAX_Y, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 1, 0, 0);

	ret = devm_request_threaded_irq(dev, client->irq, NULL, q2_touch_irq,
				       IRQF_ONESHOT | IRQF_NO_AUTOEN,
				       dev_name(dev), touch);
	if (ret)
		return dev_err_probe(dev, ret, "panel IRQ\n");

	return input_register_device(input);
}

static const struct of_device_id q2_touch_of_match[] = {
	{ .compatible = "shanling,q2-touchscreen" },
	{ }
};
MODULE_DEVICE_TABLE(of, q2_touch_of_match);

static struct i2c_driver q2_touch_driver = {
	.driver = {
		.name = "shanling-q2-touchscreen",
		.of_match_table = q2_touch_of_match,
		.dev_groups = q2_touch_groups,
		.pm = pm_sleep_ptr(&q2_touch_pm),
	},
	.probe = q2_touch_probe,
};
module_i2c_driver(q2_touch_driver);

MODULE_DESCRIPTION("Shanling Q2 touchscreen");
MODULE_LICENSE("GPL");
