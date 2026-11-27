// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SMP support for the Ingenic X2000 (XBurst2).
 *
 * The core control unit (CCU) holds the secondary core in reset and gives
 * it a reset entry address, routes peripheral, OST and mailbox interrupts
 * to each core through per-core mask bits, and has one mailbox register
 * per core: a non-zero value raises CP0 IP3 on that core until the core
 * writes it back to zero.
 *
 * Based on the XBurst2 SMP support by Zhou Yanjie and SudoMaker in the
 * Ingenic-community kernel, and on the behaviour of Ingenic's vendor
 * kernel for the X2000.
 * Copyright (c) 2013 Paul Burton <paul.burton@imgtec.com>
 * Copyright (c) 2021 周琰杰 (Zhou Yanjie) <zhouyanjie@wanyeetech.com>
 */

#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kexec.h>
#include <linux/libfdt.h>
#include <linux/reboot.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/sched/hotplug.h>
#include <linux/sched/task_stack.h>
#include <linux/smp.h>
#include <linux/spinlock.h>

#include <asm/cpu-info.h>
#include <asm/mipsregs.h>
#include <asm/kexec.h>
#include <asm/smp-ops.h>
#include <asm/tlbflush.h>

#define CCU_CSSR		0x020	/* core sleep status */
#define CCU_CSRR		0x040	/* core software reset */
#define CCU_PIMR		0x120	/* peripheral interrupt mask, per core */
#define CCU_MIMR		0x160	/* mailbox interrupt mask, per core */
#define CCU_OIMR		0x1a0	/* OST interrupt mask, per core */
#define CCU_RER			0xf00	/* reset entry */
#define CCU_MBR(cpu)		(0x1000 + 4 * (cpu))

#define X2000_MAILBOX_IRQ	(MIPS_CPU_IRQ_BASE + 3)

void x2000_secondary_entry(void);

/* Stack and thread pointers for the core being started (smp-ingenic-entry.S). */
unsigned long x2000_secondary_sp;
unsigned long x2000_secondary_gp;

static void __iomem *ccu;
static DEFINE_RAW_SPINLOCK(ccu_lock);
static int __percpu *mbox_dev_id;

static void ccu_update(unsigned int reg, u32 clear, u32 set)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ccu_lock, flags);
	writel((readl(ccu + reg) & ~clear) | set, ccu + reg);
	raw_spin_unlock_irqrestore(&ccu_lock, flags);
}

static irqreturn_t x2000_mailbox_irq(int irq, void *dev_id)
{
	unsigned int cpu = smp_processor_id();
	u32 action;

	raw_spin_lock(&ccu_lock);
	action = readl(ccu + CCU_MBR(cpu));
	writel(0, ccu + CCU_MBR(cpu));
	raw_spin_unlock(&ccu_lock);

	if (action & SMP_RESCHEDULE_YOURSELF)
		scheduler_ipi();
	if (action & SMP_CALL_FUNCTION)
		generic_smp_call_function_interrupt();

	return IRQ_HANDLED;
}

static void x2000_send_ipi_single(int cpu, unsigned int action)
{
	ccu_update(CCU_MBR(cpu), 0, action);
}

static void x2000_send_ipi_mask(const struct cpumask *mask, unsigned int action)
{
	unsigned int cpu;

	for_each_cpu(cpu, mask)
		x2000_send_ipi_single(cpu, action);
}

static void __init x2000_smp_setup(void)
{
	struct device_node *np;
	int ncpus = 0;

	for_each_of_cpu_node(np) {
		__cpu_number_map[ncpus] = ncpus;
		__cpu_logical_map[ncpus] = ncpus;
		set_cpu_possible(ncpus, true);
		set_cpu_present(ncpus, true);
		ncpus++;
	}

	/* Only this core takes mailbox interrupts until the others are up. */
	writel(BIT(0), ccu + CCU_MIMR);
	for (int cpu = 0; cpu < ncpus; cpu++)
		writel(0, ccu + CCU_MBR(cpu));
	writel(KSEG1ADDR((unsigned long)x2000_secondary_entry), ccu + CCU_RER);
}

static void __init x2000_prepare_cpus(unsigned int max_cpus)
{
	u32 cores = GENMASK(num_possible_cpus() - 1, 0);
	int err;

	mbox_dev_id = alloc_percpu(int);
	if (!mbox_dev_id)
		panic("x2000-smp: cannot allocate mailbox data");
	err = irq_set_percpu_devid(X2000_MAILBOX_IRQ);
	if (err)
		panic("x2000-smp: cannot configure mailbox interrupt: %d", err);
	irq_set_handler(X2000_MAILBOX_IRQ, handle_percpu_devid_irq);
	err = request_percpu_irq(X2000_MAILBOX_IRQ, x2000_mailbox_irq,
				 "x2000-mailbox", mbox_dev_id);
	if (err) {
		panic("x2000-smp: mailbox interrupt: %d", err);
	}
	enable_percpu_irq(X2000_MAILBOX_IRQ, IRQ_TYPE_NONE);

	/* Peripheral and OST interrupts reach every core. */
	ccu_update(CCU_PIMR, 0, cores);
	ccu_update(CCU_OIMR, 0, cores);
}

static int x2000_boot_secondary(int cpu, struct task_struct *idle)
{
	ccu_update(CCU_CSRR, 0, BIT(cpu));

	x2000_secondary_sp = __KSTK_TOS(idle);
	x2000_secondary_gp = (unsigned long)task_thread_info(idle);
	/* The new core reads these uncached from the entry code. */
	wmb();
	__flush_cache_all();

	ccu_update(CCU_CSRR, BIT(cpu), 0);

	return 0;
}

static void x2000_init_secondary(void)
{
}

static void x2000_smp_finish(void)
{
	unsigned int cpu = smp_processor_id();

	ccu_update(CCU_PIMR, 0, BIT(cpu));
	ccu_update(CCU_OIMR, 0, BIT(cpu));
	ccu_update(CCU_MIMR, 0, BIT(cpu));
	enable_percpu_irq(X2000_MAILBOX_IRQ, IRQ_TYPE_NONE);
	change_c0_status(ST0_IM, STATUSF_IP2 | STATUSF_IP3 | STATUSF_IP4);
}

#ifdef CONFIG_HOTPLUG_CPU
static int x2000_cpu_disable(void)
{
	unsigned int cpu = smp_processor_id();

	if (!cpu)
		return -EBUSY;

	set_cpu_online(cpu, false);
	calculate_cpu_foreign_map();
	ccu_update(CCU_PIMR, BIT(cpu), 0);
	ccu_update(CCU_OIMR, BIT(cpu), 0);
	irq_migrate_all_off_this_cpu();
	clear_tasks_mm_cpumask(cpu);

	return 0;
}

static void x2000_cpu_die(unsigned int cpu)
{
	/* Wait for the core to reach WAIT in play_dead(), then reset it. */
	while (!(readl(ccu + CCU_CSSR) & BIT(cpu)))
		cpu_relax();
	ccu_update(CCU_CSRR, 0, BIT(cpu));
}

void __noreturn play_dead(void)
{
	unsigned int cpu = smp_processor_id();

	ccu_update(CCU_MIMR, BIT(cpu), 0);
	idle_task_exit();
	local_irq_disable();
	local_flush_tlb_all();
	__flush_cache_all();

	for (;;)
		__asm__ __volatile__(
		"	.set	push		\n"
		"	.set	mips32r2	\n"
		"	sync			\n"
		"	wait			\n"
		"	.set	pop		\n");
}
#endif

#ifdef CONFIG_KEXEC_CORE
static int x2000_kexec_prepare(struct kimage *image)
{
	unsigned int i;

	/* Normal handoff only. Crash entry and context-preserving jumps need
	 * their own secondary/cache protocols, not the normal hotplug path.
	 */
	if (image->type != KEXEC_TYPE_DEFAULT || image->preserve_context ||
	    !IS_ENABLED(CONFIG_HOTPLUG_CPU) || reboot_cpu != 0)
		return -EOPNOTSUPP;

	for (i = 0; i < image->nr_segments; i++) {
		struct fdt_header header __aligned(8);
		struct kexec_segment *seg = &image->segment[i];

		if (seg->bufsz < sizeof(header) ||
		    copy_from_user(&header, seg->buf, sizeof(header)) ||
		    fdt_check_header(&header))
			continue;
		if (fdt_totalsize(&header) > seg->bufsz ||
		    fdt_totalsize(&header) > seg->memsz)
			return -EINVAL;
		kexec_args[0] = -2;
		kexec_args[1] = CKSEG0ADDR(seg->mem);
		kexec_args[2] = kexec_args[3] = 0;
		return 0;
	}
	return -EINVAL;
}

static void x2000_kexec_shutdown(void)
{
	/* Hotplug flushes the secondary, waits for WAIT, then holds it in CCU
	 * reset. It must not run old kernel text while relocation overwrites it.
	 */
	if (smp_processor_id() != 0)
		panic("X2000 kexec must run on CPU0");
	smp_shutdown_nonboot_cpus(0);
	if (num_online_cpus() != 1)
		panic("X2000 kexec could not offline secondary");
	writel(0, ccu + CCU_MBR(1));
	ccu_update(CCU_PIMR, BIT(1), 0);
	ccu_update(CCU_MIMR, BIT(1), 0);
	ccu_update(CCU_OIMR, BIT(1), 0);
	ccu_update(CCU_CSRR, 0, BIT(1));
}

static void __noreturn x2000_kexec_nonboot_cpu(void)
{
	/* Defensive secondary path; normal shutdown has already offlined it. */
	local_irq_disable();
	__flush_cache_all();
	ccu_update(CCU_CSRR, 0, BIT(smp_processor_id()));
	for (;;)
		__asm__ __volatile__("wait");
}
#endif

static const struct plat_smp_ops x2000_smp_ops = {
	.send_ipi_single	= x2000_send_ipi_single,
	.send_ipi_mask		= x2000_send_ipi_mask,
	.smp_setup		= x2000_smp_setup,
	.prepare_cpus		= x2000_prepare_cpus,
	.boot_secondary		= x2000_boot_secondary,
	.init_secondary		= x2000_init_secondary,
	.smp_finish		= x2000_smp_finish,
#ifdef CONFIG_KEXEC_CORE
	.kexec_nonboot_cpu	= x2000_kexec_nonboot_cpu,
#endif
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable		= x2000_cpu_disable,
	.cpu_die		= x2000_cpu_die,
#endif
};

int __init register_ingenic_x2000_smp_ops(void)
{
	struct device_node *np;

	if ((read_c0_prid() & PRID_IMP_MASK) != PRID_IMP_XBURST2)
		return -ENODEV;

	np = of_find_compatible_node(NULL, NULL, "ingenic,x2000-ccu");
	if (!np)
		return -ENODEV;
	ccu = of_iomap(np, 0);
	of_node_put(np);
	if (!ccu)
		return -ENOMEM;

#ifdef CONFIG_KEXEC_CORE
	reboot_cpu = 0;
	_machine_kexec_prepare = x2000_kexec_prepare;
	_machine_kexec_shutdown = x2000_kexec_shutdown;
#endif
	register_smp_ops(&x2000_smp_ops);

	return 0;
}
