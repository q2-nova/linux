// SPDX-License-Identifier: GPL-2.0-only
/* Shared X2000 CPU/L2 divider policy for Q2's fixed 1200 MHz CPU parent. */
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct q2_cpufreq {
	struct clk *cpu;
	struct clk *l2;
	struct clk *parent;
};
static struct q2_cpufreq *q2;

/* Stock frequencies obtainable without changing a PLL, mux or supply. */
static struct cpufreq_frequency_table q2_rates[] = {
	{ .frequency = 200000 },
	{ .frequency = 300000 },
	{ .frequency = 400000 },
	{ .frequency = 600000 },
	{ .frequency = 1200000 },
	{ .frequency = CPUFREQ_TABLE_END },
};

static unsigned long q2_l2_rate(unsigned long cpu)
{
	return cpu > 750000000 ? cpu / 2 : cpu;
}

static bool q2_achieved(unsigned long rate)
{
	return clk_get_rate(q2->parent) == 1200000000 &&
		clk_get_rate(q2->cpu) == rate &&
		clk_get_rate(q2->l2) == q2_l2_rate(rate);
}

static int q2_target(struct cpufreq_policy *policy, unsigned int index)
{
	unsigned long rate = policy->freq_table[index].frequency * 1000UL;
	unsigned long old = clk_get_rate(q2->cpu);
	int ret;

	if (!q2_achieved(old))
		return -EIO;
	ret = clk_set_rate(q2->cpu, rate);
	/* CCF may discard the provider's error. Verify uncached CPU and L2. */
	if (!ret && q2_achieved(rate))
		return 0;
	clk_set_rate(q2->cpu, old);
	if (!q2_achieved(old))
		panic("Q2 cpufreq could not restore its previous clock state");
	return ret ? ret : -EIO;
}

static int q2_policy_init(struct cpufreq_policy *policy)
{
	if (!q2_achieved(clk_get_rate(q2->cpu)))
		return -EINVAL;
	policy->clk = q2->cpu;
	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	policy->dvfs_possible_from_any_cpu = true;
	/* Provisional scheduling bound; measure before choosing a dynamic default. */
	cpufreq_generic_init(policy, q2_rates, 1000000);
	return 0;
}

static struct cpufreq_driver q2_driver = {
	.name = "q2-cpufreq",
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.init = q2_policy_init,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = q2_target,
	.get = cpufreq_generic_get,
};

static int q2_cpufreq_probe(struct platform_device *pdev)
{
	struct q2_cpufreq *data;
	int ret;

	if (q2)
		return -EBUSY;
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->cpu = devm_clk_get(&pdev->dev, "cpu");
	if (IS_ERR(data->cpu))
		return PTR_ERR(data->cpu);
	data->l2 = devm_clk_get(&pdev->dev, "l2");
	if (IS_ERR(data->l2))
		return PTR_ERR(data->l2);
	data->parent = devm_clk_get(&pdev->dev, "parent");
	if (IS_ERR(data->parent))
		return PTR_ERR(data->parent);
	ret = clk_rate_exclusive_get(data->parent);
	if (ret)
		return ret;
	q2 = data;
	if (!q2_achieved(1200000000)) {
		ret = -EINVAL;
		goto fail;
	}
	ret = cpufreq_register_driver(&q2_driver);
	if (!ret)
		return 0;
fail:
	q2 = NULL;
	clk_rate_exclusive_put(data->parent);
	return ret;
}

static void q2_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&q2_driver);
	clk_rate_exclusive_put(q2->parent);
	q2 = NULL;
}

static const struct of_device_id q2_cpufreq_match[] = {
	{ .compatible = "shanling,q2-cpufreq" },
	{ }
};
MODULE_DEVICE_TABLE(of, q2_cpufreq_match);

static struct platform_driver q2_platform_driver = {
	.probe = q2_cpufreq_probe,
	.remove = q2_cpufreq_remove,
	.driver = {
		.name = "q2-cpufreq",
		.of_match_table = q2_cpufreq_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(q2_platform_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Q2 shared CPU/L2 divider frequency scaling");
