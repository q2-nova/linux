// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C driver for the X-Powers' Power Management ICs
 *
 * AXP20x typically comprises an adaptive USB-Compatible PWM charger, BUCK DC-DC
 * converters, LDOs, multiple 12-bit ADCs of voltage, current and temperature
 * as well as configurable GPIOs.
 *
 * This driver supports the I2C variants.
 *
 * Copyright (C) 2014 Carlo Caione
 *
 * Author: Carlo Caione <carlo@caione.org>
 */

#include <linux/acpi.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mfd/axp20x.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/pm.h>

#include "axp2101-rail-snapshot.h"

static bool q2_bulk_rail_audit;
module_param(q2_bulk_rail_audit, bool, 0644);
MODULE_PARM_DESC(q2_bulk_rail_audit, "Read the Q2 AXP2101 rail audit in native I2C bursts");

/* Inserted into the owning I2C driver; private allocation, no public ABI change. */
enum axp2101_rail_phase { RAIL_LIVE, RAIL_SUSPEND, RAIL_RESUME, RAIL_PHASES };

struct axp20x_i2c_state {
	struct axp20x_dev axp20x;
	struct mutex rail_lock;
	struct axp2101_rail_sample rail[RAIL_PHASES];
	unsigned int rail_count[RAIL_PHASES];
};

static int axp2101_rail_read(void *context, unsigned int reg, unsigned int *value)
{
	struct axp20x_dev *axp20x = context;

	/* A cached enable/voltage value is not a bus readback. */
	return regmap_read_bypassed(axp20x->regmap, reg, value);
}

static int axp2101_rail_read_block(void *context, u8 reg, u8 *values, size_t count)
{
	struct axp20x_dev *axp20x = context;
	struct i2c_client *client = to_i2c_client(axp20x->dev);
	struct i2c_msg messages[] = {
		{ .addr = client->addr, .len = 1, .buf = &reg },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = count, .buf = values },
	};
	int ret;

	/* Same addressed repeated-start frame as regmap-i2c. Never alter
	 * shared cache flags or publish these readbacks into the cache.
	 */
	ret = i2c_transfer(client->adapter, messages, ARRAY_SIZE(messages));
	if (ret < 0)
		return ret;
	return ret == ARRAY_SIZE(messages) ? 0 : -EIO;
}

static u64 axp2101_rail_clock(void *context)
{
	return ktime_get_boottime_ns();
}

static void axp20x_rail_record(struct axp20x_i2c_state *state, unsigned int phase)
{
	struct i2c_client *client = to_i2c_client(state->axp20x.dev);
	struct axp2101_rail_io io = {
		.read = axp2101_rail_read, .clock = axp2101_rail_clock,
		.context = &state->axp20x,
	};

	if (state->axp20x.variant != AXP2101_ID)
		return;
	if (READ_ONCE(q2_bulk_rail_audit) && of_machine_is_compatible("shanling,q2") &&
	    i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		io.read_block = axp2101_rail_read_block;
	mutex_lock(&state->rail_lock);
	axp2101_rail_capture(&state->rail[phase], &io);
	state->rail_count[phase]++;
	mutex_unlock(&state->rail_lock);
}

static struct axp20x_i2c_state *axp20x_i2c_state(struct device *dev)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(dev);

	return container_of(axp20x, struct axp20x_i2c_state, axp20x);
}

static ssize_t rail_snapshot_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	static const char * const names[] = { "live", "suspend", "resume" };
	struct axp20x_i2c_state *state = axp20x_i2c_state(dev);
	unsigned int i, j;
	int n;

	axp20x_rail_record(state, RAIL_LIVE);
	mutex_lock(&state->rail_lock);
	n = scnprintf(buf, PAGE_SIZE, "version=1\n");
	for (i = 0; i < RAIL_PHASES; i++) {
		const struct axp2101_rail_sample *s = &state->rail[i];

		n += scnprintf(buf + n, PAGE_SIZE - n,
			"stage=%s count=%u valid=%u error=%d begin_ns=%llu end_ns=%llu\n",
			names[i], state->rail_count[i], s->valid, s->error,
			(unsigned long long)s->begin_ns, (unsigned long long)s->end_ns);
		for (j = 0; j < AXP2101_RAIL_WORDS; j++) {
			if (j < s->valid)
				n += scnprintf(buf + n, PAGE_SIZE - n,
					"reg=%02x value=%02x\n",
					axp2101_rail_registers[j], s->values[j]);
			else
				n += scnprintf(buf + n, PAGE_SIZE - n,
					"reg=%02x value=--\n", axp2101_rail_registers[j]);
		}
	}
	mutex_unlock(&state->rail_lock);
	return n;
}
static DEVICE_ATTR_RO(rail_snapshot);

static struct attribute *axp2101_rail_attrs[] = {
	&dev_attr_rail_snapshot.attr, NULL
};
static const struct attribute_group axp2101_rail_group = { .attrs = axp2101_rail_attrs };

/* Child regulator class suspend runs before this parent callback. The GPIO
 * I2C adapter is our parent and is still available. Never do these reads in
 * late/noirq/syscore, after the parent adapter or its GPIO provider sleeps.
 * Failed observation is recorded; it does not change existing PM behavior.
 */
static int axp20x_i2c_suspend(struct device *dev)
{
	axp20x_rail_record(axp20x_i2c_state(dev), RAIL_SUSPEND);
	return 0;
}

static int axp20x_i2c_resume(struct device *dev)
{
	/* Parent-before-children resume witness, distinct from the later live view. */
	axp20x_rail_record(axp20x_i2c_state(dev), RAIL_RESUME);
	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(axp20x_i2c_pm_ops,
			      axp20x_i2c_suspend, axp20x_i2c_resume);


static int axp20x_i2c_probe(struct i2c_client *i2c)
{
	struct axp20x_dev *axp20x;
	struct axp20x_i2c_state *state;
	int ret;

	state = devm_kzalloc(&i2c->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;
	mutex_init(&state->rail_lock);
	axp20x = &state->axp20x;

	axp20x->dev = &i2c->dev;
	axp20x->irq = i2c->irq;
	dev_set_drvdata(axp20x->dev, axp20x);

	ret = axp20x_match_device(axp20x);
	if (ret)
		return ret;

	axp20x->regmap = devm_regmap_init_i2c(i2c, axp20x->regmap_cfg);
	if (IS_ERR(axp20x->regmap)) {
		ret = PTR_ERR(axp20x->regmap);
		dev_err(&i2c->dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	ret = axp20x_device_probe(axp20x);
	if (ret)
		return ret;
	if (axp20x->variant == AXP2101_ID) {
		ret = devm_device_add_group(&i2c->dev, &axp2101_rail_group);
		if (ret) {
			axp20x_device_remove(axp20x);
			return ret;
		}
	}
	return 0;
}

static void axp20x_i2c_remove(struct i2c_client *i2c)
{
	struct axp20x_dev *axp20x = i2c_get_clientdata(i2c);

	axp20x_device_remove(axp20x);
}

#ifdef CONFIG_OF
static const struct of_device_id axp20x_i2c_of_match[] = {
	{ .compatible = "x-powers,axp152", .data = (void *)AXP152_ID },
	{ .compatible = "x-powers,axp192", .data = (void *)AXP192_ID },
	{ .compatible = "x-powers,axp202", .data = (void *)AXP202_ID },
	{ .compatible = "x-powers,axp209", .data = (void *)AXP209_ID },
	{ .compatible = "x-powers,axp221", .data = (void *)AXP221_ID },
	{ .compatible = "x-powers,axp223", .data = (void *)AXP223_ID },
	{ .compatible = "x-powers,axp313a", .data = (void *)AXP313A_ID },
	{ .compatible = "x-powers,axp323", .data = (void *)AXP323_ID },
	{ .compatible = "x-powers,axp717", .data = (void *)AXP717_ID },
	{ .compatible = "x-powers,axp803", .data = (void *)AXP803_ID },
	{ .compatible = "x-powers,axp806", .data = (void *)AXP806_ID },
	{ .compatible = "x-powers,axp2101", .data = (void *)AXP2101_ID },
	{ .compatible = "x-powers,axp15060", .data = (void *)AXP15060_ID },
	{ },
};
MODULE_DEVICE_TABLE(of, axp20x_i2c_of_match);
#endif

static const struct i2c_device_id axp20x_i2c_id[] = {
	{ "axp152" },
	{ "axp192" },
	{ "axp202" },
	{ "axp209" },
	{ "axp221" },
	{ "axp223" },
	{ "axp313a" },
	{ "axp717" },
	{ "axp803" },
	{ "axp806" },
	{ "axp2101" },
	{ "axp15060" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, axp20x_i2c_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id axp20x_i2c_acpi_match[] = {
	{
		.id = "INT33F4",
		.driver_data = AXP288_ID,
	},
	{ },
};
MODULE_DEVICE_TABLE(acpi, axp20x_i2c_acpi_match);
#endif

static struct i2c_driver axp20x_i2c_driver = {
	.driver = {
		.name	= "axp20x-i2c",
		.pm = pm_sleep_ptr(&axp20x_i2c_pm_ops),
		.of_match_table	= of_match_ptr(axp20x_i2c_of_match),
		.acpi_match_table = ACPI_PTR(axp20x_i2c_acpi_match),
	},
	.probe		= axp20x_i2c_probe,
	.remove		= axp20x_i2c_remove,
	.id_table	= axp20x_i2c_id,
};

module_i2c_driver(axp20x_i2c_driver);

MODULE_DESCRIPTION("PMIC MFD I2C driver for AXP20X");
MODULE_AUTHOR("Carlo Caione <carlo@caione.org>");
MODULE_LICENSE("GPL");
