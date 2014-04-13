/*
 * Samsung SoC USB 1.1/2.0 PHY driver - Exynos 4210 support
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
#define S3C64XX_OPHYPWR				0x0

#define S3C6410_OPHYPWR_OTG_DISABLE		BIT(4)
#define S3C64XX_OPHYPWR_ANALOG_POWERDOWN	BIT(3)
#define S3C6400_OPHYPWR_XO_POWERDOWN		BIT(2)
#define S3C64XX_OPHYPWR_PLL_POWERDOWN		BIT(1)
#define S3C64XX_OPHYPWR_FORCE_SUSPEND		BIT(0)

/* PHY clock control */
#define S3C64XX_OPHYCLK				0x4

#define S3C64XX_OPHYCLK_SERIAL_MODE		BIT(6)
#define S3C64XX_OPHYCLK_XO_EXT_CLK_ENB		BIT(5)
#define S3C64XX_OPHYCLK_COMMON_ON_N		BIT(4)
#define S3C6400_OPHYCLK_XO_ON_N			BIT(3)
#define S3C64XX_OPHYCLK_ID_PULLUP		BIT(2)
#define S3C64XX_OPHYCLK_CLK_SEL_MASK		(0x3 << 0)
#define S3C64XX_OPHYCLK_CLK_SEL_48MHZ		(0x0 << 0)
#define S3C64XX_OPHYCLK_CLK_SEL_12MHZ		(0x2 << 0)
#define S3C64XX_OPHYCLK_CLK_SEL_24MHZ		(0x3 << 0)

/* PHY reset control */
#define S3C64XX_ORSTCON				0x8

#define S3C64XX_ORSTCON_PHYLNK_SW_RST		BIT(2)
#define S3C64XX_ORSTCON_LINK_SW_RST		BIT(1)
#define S3C64XX_ORSTCON_PHY_SW_RST		BIT(0)

/* Isolation, configured in the power management unit */
#define S3C64XX_OTHERS_OFFSET			0x900
#define S3C64XX_OTHERS_USB_SIG_MASK		BIT(16)

enum s3c64xx_phy_id {
	S3C64XX_DEVICE,
	S3C64XX_HOST0,
	S3C64XX_HOST1,
	S3C64XX_NUM_PHYS,
};

/*
 * s3c64xx_rate_to_clk() converts the supplied clock rate to the value that
 * can be written to the phy register.
 */
static int s3c64xx_rate_to_clk(unsigned long rate, u32 *reg)
{
	switch (rate) {
	case 12 * MHZ:
		*reg = S3C64XX_OPHYCLK_CLK_SEL_12MHZ;
		break;
	case 24 * MHZ:
		*reg = S3C64XX_OPHYCLK_CLK_SEL_24MHZ;
		break;
	case 48 * MHZ:
		*reg = S3C64XX_OPHYCLK_CLK_SEL_48MHZ;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void s3c64xx_sig_mask(struct samsung_usb2_phy_driver *drv, bool on)
{
	regmap_update_bits(drv->reg_pmu, S3C64XX_OTHERS_OFFSET,
				S3C64XX_OTHERS_USB_SIG_MASK,
				on ? S3C64XX_OTHERS_USB_SIG_MASK : 0);
}

static void s3c64xx_power_on_common(struct samsung_usb2_phy_driver *drv)
{
	u32 reg;

	s3c64xx_sig_mask(drv, 1);

	reg = readl(drv->reg_phy + S3C64XX_OPHYCLK);
	reg &= ~S3C64XX_OPHYCLK_CLK_SEL_MASK;
	reg |= drv->ref_reg_val;
	reg |= S3C64XX_OPHYCLK_COMMON_ON_N;
	writel(reg, drv->reg_phy + S3C64XX_OPHYCLK);

	reg = readl(drv->reg_phy + S3C64XX_OPHYPWR);
	reg &= ~(S3C6410_OPHYPWR_OTG_DISABLE |
		S3C64XX_OPHYPWR_ANALOG_POWERDOWN |
		S3C6400_OPHYPWR_XO_POWERDOWN |
		S3C64XX_OPHYPWR_PLL_POWERDOWN |
		S3C64XX_OPHYPWR_FORCE_SUSPEND);
	writel(reg, drv->reg_phy + S3C64XX_OPHYPWR);

	mdelay(1);

	reg = readl(drv->reg_phy + S3C64XX_ORSTCON);
	reg |= (S3C64XX_ORSTCON_PHYLNK_SW_RST |
		S3C64XX_ORSTCON_LINK_SW_RST |
		S3C64XX_ORSTCON_PHY_SW_RST);
	writel(reg, drv->reg_phy + S3C64XX_ORSTCON);

	udelay(20);

	reg &= ~(S3C64XX_ORSTCON_PHYLNK_SW_RST |
		S3C64XX_ORSTCON_LINK_SW_RST |
		S3C64XX_ORSTCON_PHY_SW_RST);
	writel(reg, drv->reg_phy + S3C64XX_ORSTCON);
}

static void s3c64xx_power_off_common(struct samsung_usb2_phy_driver *drv)
{
	u32 reg;

	reg = readl(drv->reg_phy + S3C64XX_OPHYPWR);
	reg |= (S3C6410_OPHYPWR_OTG_DISABLE |
		S3C64XX_OPHYPWR_ANALOG_POWERDOWN |
		S3C6400_OPHYPWR_XO_POWERDOWN |
		S3C64XX_OPHYPWR_PLL_POWERDOWN |
		S3C64XX_OPHYPWR_FORCE_SUSPEND);
	writel(reg, drv->reg_phy + S3C64XX_OPHYPWR);

	s3c64xx_sig_mask(drv, 0);
}

static int s3c64xx_power_on(struct samsung_usb2_phy_instance *inst)
{
	struct samsung_usb2_phy_driver *drv = inst->drv;
	struct samsung_usb2_phy_instance *other = NULL;
	bool set_device = false;
	bool set_host = false;

	switch (inst->cfg->id) {
	case S3C64XX_DEVICE:
		other = &drv->instances[S3C64XX_HOST0];
		set_device = true;
		break;
	case S3C64XX_HOST0:
		other = &drv->instances[S3C64XX_DEVICE];
		set_host = true;
		break;
	}

	/* Only one of the paths can be enabled at the same time */
	if (other && other->int_cnt)
		return -EBUSY;

	if (set_device || set_host) {
		u32 ophyclk;

		ophyclk = readl(drv->reg_phy + S3C64XX_OPHYCLK);
		if (set_device)
			ophyclk &= ~S3C64XX_OPHYCLK_SERIAL_MODE;
		if (set_host)
			ophyclk |= S3C64XX_OPHYCLK_SERIAL_MODE;
		writel(ophyclk, drv->reg_phy + S3C64XX_OPHYCLK);
	}

	inst->int_cnt = 1;

	if (drv->enable_count++)
		return 0;

	s3c64xx_power_on_common(drv);

	return 0;
}

static int s3c64xx_power_off(struct samsung_usb2_phy_instance *inst)
{
	struct samsung_usb2_phy_driver *drv = inst->drv;

	inst->int_cnt = 0;

	if (--drv->enable_count)
		return 0;

	s3c64xx_power_off_common(inst->drv);

	return 0;
}

static const struct samsung_usb2_common_phy s3c64xx_phys[] = {
	{
		.label		= "device",
		.id		= S3C64XX_DEVICE,
		.power_on	= s3c64xx_power_on,
		.power_off	= s3c64xx_power_off,
	},
	{
		.label		= "host0",
		.id		= S3C64XX_HOST0,
		.power_on	= s3c64xx_power_on,
		.power_off	= s3c64xx_power_off,
	},
	{
		.label		= "host1",
		.id		= S3C64XX_HOST1,
		.power_on	= s3c64xx_power_on,
		.power_off	= s3c64xx_power_off,
	},
};

const struct samsung_usb2_phy_config s3c64xx_usb_phy_config = {
	.has_mode_switch	= 0,
	.num_phys		= S3C64XX_NUM_PHYS,
	.phys			= s3c64xx_phys,
	.rate_to_clk		= s3c64xx_rate_to_clk,
};
