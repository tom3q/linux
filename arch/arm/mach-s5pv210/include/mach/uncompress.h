/* linux/arch/arm/mach-s5pv210/include/mach/uncompress.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV210 - uncompress code
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <mach/map.h>
#include <plat/cpu.h>
#include <plat/uncompress.h>

static unsigned int __raw_readl(unsigned int ptr)
{
	return *((volatile unsigned int *)ptr);
}

static void arch_detect_cpu(void)
{
	u32 chip_id = __raw_readl(S5P_PA_CHIPID) & S5PV210_CPU_MASK;
	u32 uart_nr = CONFIG_S3C_LOWLEVEL_UART_PORT;
	u32 uart_pa;

	fifo_mask = S5PV210_UFSTAT_TXMASK;
	fifo_max = 63 << S5PV210_UFSTAT_TXSHIFT;

	if (chip_id == S5PV210_CPU_ID)
		uart_pa = S5PV210_PA_UART;
	else
		uart_pa = S5P6442_PA_UART;

	uart_base = (volatile u8 *)(uart_pa + uart_nr * S3C_UART_OFFSET);
}

#endif /* __ASM_ARCH_UNCOMPRESS_H */
