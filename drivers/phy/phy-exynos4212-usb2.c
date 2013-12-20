/*
 * Samsung SoC USB 1.1/2.0 PHY driver - Exynos 4212 support
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 * Author: Kamil Debski <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include "phy-samsung-usb2.h"

/* Exynos USB PHY registers */

/* PHY power control */
#define EXYNOS_4212_UPHYPWR			0x0

#define EXYNOS_4212_UPHYPWR_PHY0_SUSPEND	BIT(0)
#define EXYNOS_4212_UPHYPWR_PHY0_PWR		BIT(3)
#define EXYNOS_4212_UPHYPWR_PHY0_OTG_PWR	BIT(4)
#define EXYNOS_4212_UPHYPWR_PHY0_SLEEP		BIT(5)
#define EXYNOS_4212_UPHYPWR_PHY0 ( \
	EXYNOS_4212_UPHYPWR_PHY0_SUSPEND | \
	EXYNOS_4212_UPHYPWR_PHY0_PWR | \
	EXYNOS_4212_UPHYPWR_PHY0_OTG_PWR | \
	EXYNOS_4212_UPHYPWR_PHY0_SLEEP)

#define EXYNOS_4212_UPHYPWR_PHY1_SUSPEND	BIT(6)
#define EXYNOS_4212_UPHYPWR_PHY1_PWR		BIT(7)
#define EXYNOS_4212_UPHYPWR_PHY1_SLEEP		BIT(8)
#define EXYNOS_4212_UPHYPWR_PHY1 ( \
	EXYNOS_4212_UPHYPWR_PHY1_SUSPEND | \
	EXYNOS_4212_UPHYPWR_PHY1_PWR | \
	EXYNOS_4212_UPHYPWR_PHY1_SLEEP)

#define EXYNOS_4212_UPHYPWR_HSCI0_SUSPEND	BIT(9)
#define EXYNOS_4212_UPHYPWR_HSCI0_PWR		BIT(10)
#define EXYNOS_4212_UPHYPWR_HSCI0_SLEEP		BIT(11)
#define EXYNOS_4212_UPHYPWR_HSCI0 ( \
	EXYNOS_4212_UPHYPWR_HSCI0_SUSPEND | \
	EXYNOS_4212_UPHYPWR_HSCI0_PWR | \
	EXYNOS_4212_UPHYPWR_HSCI0_SLEEP)

#define EXYNOS_4212_UPHYPWR_HSCI1_SUSPEND	BIT(12)
#define EXYNOS_4212_UPHYPWR_HSCI1_PWR		BIT(13)
#define EXYNOS_4212_UPHYPWR_HSCI1_SLEEP		BIT(14)
#define EXYNOS_4212_UPHYPWR_HSCI1 ( \
	EXYNOS_4212_UPHYPWR_HSCI1_SUSPEND | \
	EXYNOS_4212_UPHYPWR_HSCI1_PWR | \
	EXYNOS_4212_UPHYPWR_HSCI1_SLEEP)

/* PHY clock control */
#define EXYNOS_4212_UPHYCLK			0x4

#define EXYNOS_4212_UPHYCLK_PHYFSEL_MASK	(0x7 << 0)
#define EXYNOS_4212_UPHYCLK_PHYFSEL_9MHZ6	(0x0 << 0)
#define EXYNOS_4212_UPHYCLK_PHYFSEL_10MHZ	(0x1 << 0)
#define EXYNOS_4212_UPHYCLK_PHYFSEL_12MHZ	(0x2 << 0)
#define EXYNOS_4212_UPHYCLK_PHYFSEL_19MHZ2	(0x3 << 0)
#define EXYNOS_4212_UPHYCLK_PHYFSEL_20MHZ	(0x4 << 0)
#define EXYNOS_4212_UPHYCLK_PHYFSEL_24MHZ	(0x5 << 0)
#define EXYNOS_4212_UPHYCLK_PHYFSEL_50MHZ	(0x7 << 0)

#define EXYNOS_4212_UPHYCLK_PHY0_ID_PULLUP	BIT(3)
#define EXYNOS_4212_UPHYCLK_PHY0_COMMON_ON	BIT(4)
#define EXYNOS_4212_UPHYCLK_PHY1_COMMON_ON	BIT(7)

#define EXYNOS_4212_UPHYCLK_HSIC_REFCLK_MASK	(0x7f << 10)
#define EXYNOS_4212_UPHYCLK_HSIC_REFCLK_12MHZ	(0x24 << 10)
#define EXYNOS_4212_UPHYCLK_HSIC_REFCLK_15MHZ	(0x1c << 10)
#define EXYNOS_4212_UPHYCLK_HSIC_REFCLK_16MHZ	(0x1a << 10)
#define EXYNOS_4212_UPHYCLK_HSIC_REFCLK_19MHZ2	(0x15 << 10)
#define EXYNOS_4212_UPHYCLK_HSIC_REFCLK_20MHZ	(0x14 << 10)

/* PHY reset control */
#define EXYNOS_4212_UPHYRST			0x8

#define EXYNOS_4212_URSTCON_PHY0		BIT(0)
#define EXYNOS_4212_URSTCON_OTG_HLINK		BIT(1)
#define EXYNOS_4212_URSTCON_OTG_PHYLINK		BIT(2)
#define EXYNOS_4212_URSTCON_HOST_PHY		BIT(3)
#define EXYNOS_4212_URSTCON_PHY1		BIT(4)
#define EXYNOS_4212_URSTCON_HSIC0		BIT(5)
#define EXYNOS_4212_URSTCON_HSIC1		BIT(6)
#define EXYNOS_4212_URSTCON_HOST_LINK_ALL	BIT(7)
#define EXYNOS_4212_URSTCON_HOST_LINK_P0	BIT(8)
#define EXYNOS_4212_URSTCON_HOST_LINK_P1	BIT(9)
#define EXYNOS_4212_URSTCON_HOST_LINK_P2	BIT(10)

/* Isolation, configured in the power management unit */
#define EXYNOS_4212_USB_ISOL_OFFSET		0x704
#define EXYNOS_4212_USB_ISOL_OTG		BIT(0)
#define EXYNOS_4212_USB_ISOL_HSIC0_OFFSET	0x708
#define EXYNOS_4212_USB_ISOL_HSIC0		BIT(0)
#define EXYNOS_4212_USB_ISOL_HSIC1_OFFSET	0x70c
#define EXYNOS_4212_USB_ISOL_HSIC1		BIT(0)

/* Mode switching SUB Device <-> Host */
#define EXYNOS_4212_MODE_SWITCH_OFFSET		0x21c
#define EXYNOS_4212_MODE_SWITCH_MASK		1
#define EXYNOS_4212_MODE_SWITCH_DEVICE		0
#define EXYNOS_4212_MODE_SWITCH_HOST		1

enum exynos4x12_phy_id {
	EXYNOS4212_DEVICE,
	EXYNOS4212_HOST,
	EXYNOS4212_HSIC0,
	EXYNOS4212_HSIC1,
	EXYNOS4212_NUM_PHYS,
};

/*
 * exynos4212_rate_to_clk() converts the supplied clock rate to the value that
 * can be written to the phy register.
 */
static int exynos4212_rate_to_clk(unsigned long rate, u32 *reg)
{
	/* EXYNOS_4212_UPHYCLK_PHYFSEL_MASK */

	switch (rate) {
	case 9600 * KHZ:
		*reg = EXYNOS_4212_UPHYCLK_PHYFSEL_9MHZ6;
		break;
	case 10 * MHZ:
		*reg = EXYNOS_4212_UPHYCLK_PHYFSEL_10MHZ;
		break;
	case 12 * MHZ:
		*reg = EXYNOS_4212_UPHYCLK_PHYFSEL_12MHZ;
		break;
	case 19200 * KHZ:
		*reg = EXYNOS_4212_UPHYCLK_PHYFSEL_19MHZ2;
		break;
	case 20 * MHZ:
		*reg = EXYNOS_4212_UPHYCLK_PHYFSEL_20MHZ;
		break;
	case 24 * MHZ:
		*reg = EXYNOS_4212_UPHYCLK_PHYFSEL_24MHZ;
		break;
	case 50 * MHZ:
		*reg = EXYNOS_4212_UPHYCLK_PHYFSEL_50MHZ;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void exynos4212_isol(struct samsung_usb2_phy_instance *inst, bool on)
{
	struct samsung_usb2_phy_driver *drv = inst->drv;
	u32 offset;
	u32 mask;

	switch (inst->cfg->id) {
	case EXYNOS4212_DEVICE:
	case EXYNOS4212_HOST:
		offset = EXYNOS_4212_USB_ISOL_OFFSET;
		mask = EXYNOS_4212_USB_ISOL_OTG;
		break;
	case EXYNOS4212_HSIC0:
		offset = EXYNOS_4212_USB_ISOL_HSIC0_OFFSET;
		mask = EXYNOS_4212_USB_ISOL_HSIC0;
		break;
	case EXYNOS4212_HSIC1:
		offset = EXYNOS_4212_USB_ISOL_HSIC1_OFFSET;
		mask = EXYNOS_4212_USB_ISOL_HSIC1;
		break;
	default:
		return;
	};

	regmap_update_bits(drv->reg_pmu, offset, mask, on ? 0 : mask);
}

static void exynos4212_phy_pwr(struct samsung_usb2_phy_instance *inst, bool on)
{
	struct samsung_usb2_phy_driver *drv = inst->drv;
	u32 rstbits = 0;
	u32 phypwr = 0;
	u32 rst;
	u32 pwr;

	switch (inst->cfg->id) {
	case EXYNOS4212_DEVICE:
		phypwr =	EXYNOS_4212_UPHYPWR_PHY0;
		rstbits =	EXYNOS_4212_URSTCON_PHY0;
		break;
	case EXYNOS4212_HOST:
		phypwr =	EXYNOS_4212_UPHYPWR_PHY1;
		rstbits =	EXYNOS_4212_URSTCON_HOST_PHY;
		break;
	case EXYNOS4212_HSIC0:
		phypwr =	EXYNOS_4212_UPHYPWR_HSCI0;
		rstbits =	EXYNOS_4212_URSTCON_HSIC1 |
				EXYNOS_4212_URSTCON_HOST_LINK_P0 |
				EXYNOS_4212_URSTCON_HOST_PHY;
		break;
	case EXYNOS4212_HSIC1:
		phypwr =	EXYNOS_4212_UPHYPWR_HSCI1;
		rstbits =	EXYNOS_4212_URSTCON_HSIC1 |
				EXYNOS_4212_URSTCON_HOST_LINK_P1;
		break;
	};

	if (on) {
		writel(inst->clk_reg_val, drv->reg_phy + EXYNOS_4212_UPHYCLK);

		pwr = readl(drv->reg_phy + EXYNOS_4212_UPHYPWR);
		pwr &= ~phypwr;
		writel(pwr, drv->reg_phy + EXYNOS_4212_UPHYPWR);

		rst = readl(drv->reg_phy + EXYNOS_4212_UPHYRST);
		rst |= rstbits;
		writel(rst, drv->reg_phy + EXYNOS_4212_UPHYRST);
		udelay(10);
		rst &= ~rstbits;
		writel(rst, drv->reg_phy + EXYNOS_4212_UPHYRST);
	} else {
		pwr = readl(drv->reg_phy + EXYNOS_4212_UPHYPWR);
		pwr |= phypwr;
		writel(pwr, drv->reg_phy + EXYNOS_4212_UPHYPWR);
	}
}

static int exynos4212_power_on(struct samsung_usb2_phy_instance *inst)
{
	struct samsung_usb2_phy_driver *drv = inst->drv;

	inst->enabled = 1;
	exynos4212_phy_pwr(inst, 1);
	exynos4212_isol(inst, 0);

	/* Power on the device, as it is necessary for HSIC to work */
	if (inst->cfg->id == EXYNOS4212_HSIC0) {
		struct samsung_usb2_phy_instance *device =
					&drv->instances[EXYNOS4212_DEVICE];
		exynos4212_phy_pwr(device, 1);
		exynos4212_isol(device, 0);
	}

	return 0;
}

static int exynos4212_power_off(struct samsung_usb2_phy_instance *inst)
{
	struct samsung_usb2_phy_driver *drv = inst->drv;
	struct samsung_usb2_phy_instance *device =
					&drv->instances[EXYNOS4212_DEVICE];

	inst->enabled = 0;
	exynos4212_isol(inst, 1);
	exynos4212_phy_pwr(inst, 0);

	if (inst->cfg->id == EXYNOS4212_HSIC0 && !device->enabled) {
		exynos4212_isol(device, 1);
		exynos4212_phy_pwr(device, 0);
	}

	return 0;
}


static const struct samsung_usb2_common_phy exynos4212_phys[] = {
	{
		.label		= "device",
		.id		= EXYNOS4212_DEVICE,
		.rate_to_clk	= exynos4212_rate_to_clk,
		.power_on	= exynos4212_power_on,
		.power_off	= exynos4212_power_off,
	},
	{
		.label		= "host",
		.id		= EXYNOS4212_HOST,
		.rate_to_clk	= exynos4212_rate_to_clk,
		.power_on	= exynos4212_power_on,
		.power_off	= exynos4212_power_off,
	},
	{
		.label		= "hsic0",
		.id		= EXYNOS4212_HSIC0,
		.rate_to_clk	= exynos4212_rate_to_clk,
		.power_on	= exynos4212_power_on,
		.power_off	= exynos4212_power_off,
	},
	{
		.label		= "hsic1",
		.id		= EXYNOS4212_HSIC1,
		.rate_to_clk	= exynos4212_rate_to_clk,
		.power_on	= exynos4212_power_on,
		.power_off	= exynos4212_power_off,
	},
	{},
};

const struct samsung_usb2_phy_config exynos4212_usb2_phy_config = {
	.num_phys		= EXYNOS4212_NUM_PHYS,
	.phys			= exynos4212_phys,
	.has_mode_switch	= 1,
};

