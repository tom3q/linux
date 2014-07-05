/*
 * Samsung's S3C64XX flattened device tree enabled machine
 *
 * Copyright (c) 2013 Tomasz Figa <tomasz.figa@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/of_platform.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/arm-vic.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>

#include <plat/cpu.h>
#include <plat/pm.h>
#include <plat/wakeup-mask.h>
#include <plat/watchdog-reset.h>

#include <mach/map.h>

#include "common.h"
#include "regs-sys.h"
#include "regs-syscon-power.h"

#define S3C64XX_PWR_CFG_WAKE_MASK	(0x3ff << 7)

/*
 * VIC wake-up support
 */

/**
 * struct s3c64xx_wkup_irq - Exynos GIC to PMU IRQ mapping
 * @hwirq: Hardware IRQ signal of the GIC
 * @mask: Mask in PMU wake-up mask register
 */
struct s3c64xx_wkup_irq {
	unsigned long vic_base;
	unsigned int hwirq;
	u32 mask;
};

static u32 s3c64xx_irqwake_intmask = 0xffffffff;

static const struct s3c64xx_wkup_irq s3c64xx_wkup_irq[] = {
	{ 0x71200000, 2, S3C64XX_PWRCFG_RTC_TICK_DISABLE }, /* RTC tick */
	{ 0x71200000, 14, S3C64XX_PWRCFG_BATF_DISABLE }, /* Batt fault */
	{ 0x71300000, 28, S3C64XX_PWRCFG_RTC_ALARM_DISABLE }, /* RTC alarm */
	{ /* sentinel */ },
};

static int s3c64xx_irq_set_wake(struct irq_data *data, unsigned int state)
{
	const struct s3c64xx_wkup_irq *wkup_irq = s3c64xx_wkup_irq;
	struct irq_domain *domain = data->domain;
	struct device_node *np = domain->of_node;
	struct resource addr;
	int ret;

	ret = of_address_to_resource(np, 0, &addr);
	if (ret)
		return ret;

	while (wkup_irq->mask) {
		if (wkup_irq->vic_base == addr.start
		    && wkup_irq->hwirq == data->hwirq) {
			if (!state)
				s3c64xx_irqwake_intmask |= wkup_irq->mask;
			else
				s3c64xx_irqwake_intmask &= ~wkup_irq->mask;
			return 0;
		}
		++wkup_irq;
	}

	return -ENOENT;
}

/*
 * PM sleep
 */

static struct sleep_save sys_save[] = {
	SAVE_ITEM(S3C64XX_SDMA_SEL),
	SAVE_ITEM(S3C64XX_NORMAL_CFG),
};

static int s3c64xx_pm_suspend(void)
{
	s3c_pm_do_save(sys_save, ARRAY_SIZE(sys_save));

	return 0;
}

static void s3c64xx_pm_resume(void)
{
	u32 reg;

	s3c_pm_do_restore_core(sys_save, ARRAY_SIZE(sys_save));

	__raw_writel(0, S3C64XX_EINT_MASK);

	/* Setup PWRCFG to enter idle mode */
	reg = __raw_readl(S3C64XX_PWR_CFG);
	reg &= ~S3C64XX_PWRCFG_CFG_WFI_MASK;
	reg |= S3C64XX_PWRCFG_CFG_WFI_IDLE;
	__raw_writel(reg, S3C64XX_PWR_CFG);
}

static struct syscore_ops s3c64xx_pm_syscore_ops = {
	.suspend	= s3c64xx_pm_suspend,
	.resume		= s3c64xx_pm_resume,
};

static void s3c64xx_pm_prepare(void)
{
	u32 reg;

	reg = __raw_readl(S3C64XX_PWR_CFG);
	reg &= ~S3C64XX_PWR_CFG_WAKE_MASK;
	reg |= s3c64xx_irqwake_intmask & S3C64XX_PWR_CFG_WAKE_MASK;
	__raw_writel(reg, S3C64XX_PWR_CFG);

	__raw_writel(s3c_irqwake_eintmask, S3C64XX_EINT_MASK);

	/* store address of resume. */
	__raw_writel(virt_to_phys(s3c_cpu_resume), S3C64XX_INFORM0);
}

static int s3c64xx_cpu_suspend(unsigned long arg)
{
	unsigned long tmp;

	/* set our standby method to sleep */
	tmp = __raw_readl(S3C64XX_PWR_CFG);
	tmp &= ~S3C64XX_PWRCFG_CFG_WFI_MASK;
	tmp |= S3C64XX_PWRCFG_CFG_WFI_SLEEP;
	__raw_writel(tmp, S3C64XX_PWR_CFG);

	/* clear wake-up stat */
	__raw_writel(__raw_readl(S3C64XX_WAKEUP_STAT), S3C64XX_WAKEUP_STAT);

	/* issue the standby signal into the pm unit. Note, we
	 * issue a write-buffer drain just in case */

	tmp = 0;

	asm("b 1f\n\t"
	    ".align 5\n\t"
	    "1:\n\t"
	    "mcr p15, 0, %0, c7, c10, 5\n\t"
	    "mcr p15, 0, %0, c7, c10, 4\n\t"
	    "mcr p15, 0, %0, c7, c0, 4" :: "r" (tmp));

	/* we should never get past here */

	pr_info("Failed to suspend the system\n");
	return 1; /* Aborting suspend */
}

static int s3c64xx_suspend_enter(suspend_state_t state)
{
	int ret;

	s3c_pm_debug_init();

	S3C_PMDBG("%s: suspending the system...\n", __func__);

	S3C_PMDBG("%s: wakeup masks: %08x,%08x\n", __func__,
			s3c64xx_irqwake_intmask, s3c64xx_get_eint_wake_mask());

	if (s3c64xx_irqwake_intmask == -1U
	    && s3c64xx_get_eint_wake_mask() == -1U) {
		pr_err("%s: No wake-up sources!\n", __func__);
		pr_err("%s: Aborting sleep\n", __func__);
		return -EINVAL;
	}

	s3c_pm_save_uarts();
	s3c64xx_pm_prepare();
	flush_cache_all();
	s3c_pm_check_store();

	ret = cpu_suspend(0, s3c64xx_cpu_suspend);
	if (ret)
		return ret;

	s3c_pm_restore_uarts();

	S3C_PMDBG("%s: wakeup stat: %08x\n", __func__,
			__raw_readl(S3C64XX_WAKEUP_STAT));

	s3c_pm_check_restore();

	S3C_PMDBG("%s: resuming the system...\n", __func__);

	return 0;
}

static int s3c64xx_suspend_prepare(void)
{
	s3c_pm_check_prepare();

	return 0;
}

static void s3c64xx_suspend_finish(void)
{
	s3c_pm_check_cleanup();
}

static const struct platform_suspend_ops s3c64xx_suspend_ops = {
	.enter		= s3c64xx_suspend_enter,
	.prepare	= s3c64xx_suspend_prepare,
	.finish		= s3c64xx_suspend_finish,
	.valid		= suspend_valid_only_mem,
};

/*
 * IO mapping for shared system controller IP.
 *
 * FIXME: Make remaining drivers use dynamic mapping.
 */
static struct map_desc s3c64xx_dt_iodesc[] __initdata = {
	{
		.virtual	= (unsigned long)S3C_VA_SYS,
		.pfn		= __phys_to_pfn(S3C64XX_PA_SYSCON),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
};

static void __init s3c64xx_dt_map_io(void)
{
	debug_ll_io_init();
	iotable_init(s3c64xx_dt_iodesc, ARRAY_SIZE(s3c64xx_dt_iodesc));

	s3c64xx_init_cpu();

	if (!soc_is_s3c64xx())
		panic("SoC is not S3C64xx!");
}

static void __init s3c64xx_dt_init_machine(void)
{
	samsung_wdt_reset_of_init();
	s3c64xx_pm_init();
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	register_syscore_ops(&s3c64xx_pm_syscore_ops);
	suspend_set_ops(&s3c64xx_suspend_ops);
	vic_arch_set_wake = s3c64xx_irq_set_wake;
}

static void __init s3c64xx_dt_init_late(void)
{
	s3c64xx_pm_late_initcall();
}

static void s3c64xx_dt_restart(enum reboot_mode mode, const char *cmd)
{
	if (mode != REBOOT_SOFT)
		samsung_wdt_reset();

	/* if all else fails, or mode was for soft, jump to 0 */
	soft_restart(0);
}

static char const *s3c64xx_dt_compat[] __initdata = {
	"samsung,s3c6400",
	"samsung,s3c6410",
	NULL
};

DT_MACHINE_START(S3C6400_DT, "Samsung S3C64xx (Flattened Device Tree)")
	/* Maintainer: Tomasz Figa <tomasz.figa@gmail.com> */
	.dt_compat	= s3c64xx_dt_compat,
	.map_io		= s3c64xx_dt_map_io,
	.init_machine	= s3c64xx_dt_init_machine,
	.init_late	= s3c64xx_dt_init_late,
	.restart        = s3c64xx_dt_restart,
MACHINE_END
