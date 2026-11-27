// SPDX-License-Identifier: GPL-2.0-only
/* Shanling Q2 combo-chip prerequisites, before MSC1 enumeration. */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "pwrseq.h"

/* GPIO enables and their stock delays remain owned by vmmc's supply chain. */
static const struct mmc_pwrseq_ops q2_pwrseq_ops = {};

static int q2_pwrseq_probe(struct platform_device *pdev)
{
	static const char * const supplies[] = { "aldo1", "bldo1", "bldo2" };
	struct device *dev = &pdev->dev;
	struct mmc_pwrseq *pwrseq;
	size_t i;
	int ret;

	/* Required properties prevent dummy supplies hiding an incomplete DT. */
	for (i = 0; i < ARRAY_SIZE(supplies); i++) {
		char property[16];

		snprintf(property, sizeof(property), "%s-supply", supplies[i]);
		if (!device_property_present(dev, property))
			return dev_err_probe(dev, -EINVAL, "missing %s\n", property);
	}

	pwrseq = devm_kzalloc(dev, sizeof(*pwrseq), GFP_KERNEL);
	if (!pwrseq)
		return -ENOMEM;

	/*
	 * These shared rails already have always-on board constraints. Acquire
	 * them before publishing the provider: a modular PMIC must defer MSC1
	 * probing, not let its non-removable scan run once with the chip unpowered.
	 * Managed bulk acquisition rolls back on errors and provider removal.
	 */
	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(supplies), supplies);
	if (ret)
		return dev_err_probe(dev, ret, "combo-chip supplies not ready\n");

	pwrseq->dev = dev;
	pwrseq->ops = &q2_pwrseq_ops;
	pwrseq->owner = THIS_MODULE;
	platform_set_drvdata(pdev, pwrseq);
	return mmc_pwrseq_register(pwrseq);
}

static void q2_pwrseq_remove(struct platform_device *pdev)
{
	mmc_pwrseq_unregister(platform_get_drvdata(pdev));
}

static const struct of_device_id q2_pwrseq_match[] = {
	{ .compatible = "shanling,q2-wifi-pwrseq" },
	{}
};
MODULE_DEVICE_TABLE(of, q2_pwrseq_match);

static struct platform_driver q2_pwrseq_driver = {
	.probe = q2_pwrseq_probe,
	.remove = q2_pwrseq_remove,
	.driver = {
		.name = "pwrseq-q2",
		.of_match_table = q2_pwrseq_match,
	},
};
module_platform_driver(q2_pwrseq_driver);
MODULE_DESCRIPTION("Shanling Q2 Wi-Fi power prerequisites");
MODULE_LICENSE("GPL");
