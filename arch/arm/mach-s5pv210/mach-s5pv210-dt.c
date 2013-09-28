
/*
 * Samsung's S5PV210/S5PC110 flattened device tree enabled machine
 *
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 * Author: Mateusz Krawczuk <m.krawczuk@partner.samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk-provider.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#include <plat/cpu.h>
#include <plat/watchdog-reset.h>

#include <mach/map.h>

#include "common.h"

/*
 * IO mapping for shared system controller IP.
 *
 * FIXME: Make remaining drivers use dynamic mapping.
 */
static struct map_desc s5pv210_dt_iodesc[] __initdata = {
	{
		.virtual = (unsigned long)S3C_VA_SYS,
		.pfn = __phys_to_pfn(S5PV210_PA_SYSCON),
		.length = SZ_64K,
		.type = MT_DEVICE,
	},
};

static void __init s5pv210_dt_map_io(void)
{
	debug_ll_io_init();
	iotable_init(s5pv210_dt_iodesc, ARRAY_SIZE(s5pv210_dt_iodesc));
}

static void __init s5pv210_dt_init_irq(void)
{
	void __iomem *chipid_base;
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "samsung,s5pv210-chipid");
	if (!np)
		panic("%s: Unable to find device node!", __func__);

	chipid_base = of_iomap(np, 0);
	if (!chipid_base)
		panic("Unable get a chipid!");

	s5p_init_cpu(chipid_base);
	iounmap(chipid_base);

	if (!soc_is_s5pv210())
		panic("SoC is not S5PV210/S5PC110!");

	of_clk_init(NULL);
	irqchip_init();
};

static void __init s5pv210_dt_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static char const *s5pv210_dt_compat[] __initconst = {
	"samsung,s5pc110",
	"samsung,s5pv210",
	NULL
};

DT_MACHINE_START(S3C6400_DT, "Samsung S5PV210/S5PC110 (Flattened Device Tree)")
	/* Maintainer: Mateusz Krawczuk <m.krawczuk@partner.samsung.com> */
	.dt_compat  = s5pv210_dt_compat,
	.map_io    = s5pv210_dt_map_io,
	.init_irq  = s5pv210_dt_init_irq,
	.init_machine  = s5pv210_dt_init_machine,
	.restart        = s5pv210_restart,
MACHINE_END
