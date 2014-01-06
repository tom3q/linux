/*
 * S6D04D1 LCD panel driver.
 *
 * Copyright (c) 2013 Tomasz Figa <tomasz.figa@gmail.com>
 *
 * Derived from drivers/video/backlight/s6e63m0.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>

#define MAX_BRIGHTNESS		127

/* SPI command codes */
#define POWCTL			0xf3
#define VCMCTL			0xf4
#define SRCCTL			0xf5
#define SLPOUT			0x11
#define TEON			0x35
#define MADCTL			0x36
#define COLMOD			0x3a
#define DISCTL			0xf2
#define IFCTL			0xf6
#define GATECTL			0xfd
#define WRDISBV			0x51
#define WRCABCMB		0x5e
#define MIECTL1			0xca
#define BCMODE			0xcb
#define MIECTL2			0xcc
#define MIECTL3			0xcd
#define RPGAMCTL		0xf7
#define RNGAMCTL		0xf8
#define GPGAMCTL		0xf9
#define GNGAMCTL		0xfa
#define BPGAMCTL		0xfb
#define BNGAMCTL		0xfc
#define CASET			0x2a
#define PASET			0x2b
#define RAMWR			0x2c
#define WRCTRLD			0x53
#define WRCABC			0x55
#define DISPON			0x29
#define DISPOFF			0x28
#define SLPIN			0x10
#define RDDIDIF			0x04
#define RDID1			0xda
#define RDID2			0xdb
#define RDID3			0xdc

#define DATA(val)		(0x100 | (val))

struct s6d04d1_panel {
	u16 mtp_value[3];
	const u16 *power_config;
	unsigned int power_config_len;
	const u16 *display_config;
	unsigned int display_config_len;
};

struct s6d04d1 {
	struct device *dev;
	struct spi_device *spi;
	struct lcd_device *ld;
	struct backlight_device *bd;
	const struct s6d04d1_panel *panel;

	struct regulator_bulk_data supplies[2];
	int reset_gpio;

	unsigned int power;
	unsigned int current_brightness;
};

/*
 * Panel-specific power-on sequences
 */

/* SONY WQVGA 3.2" panel */
static const u16 power_config_sony[] = {
	POWCTL,		DATA(0x80), DATA(0x29), DATA(0x29), DATA(0x08),
			DATA(0x33), DATA(0x32), DATA(0x32), DATA(0x20),
	VCMCTL,		DATA(0x19), DATA(0x19), DATA(0x1E), DATA(0x1E),
			DATA(0x33),
	SRCCTL,		DATA(0x00), DATA(0x00), DATA(0x06), DATA(0xF0),
			DATA(0x00), DATA(0x1F),
	SLPOUT,
};

static const u16 display_config_sony[] = {
	TEON,		DATA(0x00),
	MADCTL,		DATA(0x98),
	COLMOD,		DATA(0x77),
	DISCTL,		DATA(0x16), DATA(0x16), DATA(0x0F), DATA(0x04),
			DATA(0x04), DATA(0x04), DATA(0x04), DATA(0x0A),
			DATA(0x00), DATA(0x16), DATA(0x16),
	IFCTL,		DATA(0x00), DATA(0x81), DATA(0x30), DATA(0x13),
	GATECTL,	DATA(0x44), DATA(0x01),
	RPGAMCTL,	DATA(0x80), DATA(0x00), DATA(0x00), DATA(0x00),
			DATA(0x02), DATA(0x1A), DATA(0x22), DATA(0x2C),
			DATA(0x13), DATA(0x10), DATA(0x1A), DATA(0x19),
			DATA(0x0D), DATA(0x84), DATA(0x21),
	RNGAMCTL,	DATA(0x80), DATA(0x00), DATA(0x00), DATA(0x00),
			DATA(0x02), DATA(0x1A), DATA(0x22), DATA(0x2C),
			DATA(0x13), DATA(0x10), DATA(0x1A), DATA(0x19),
			DATA(0x0D), DATA(0x84), DATA(0x21),
	CASET,		DATA(0x00), DATA(0x00), DATA(0x00), DATA(0xEF),
	PASET,		DATA(0x00), DATA(0x00), DATA(0x01), DATA(0x8F),
	RAMWR,
	MIECTL3,	DATA(0x7C), DATA(0x01),
	WRDISBV,	DATA(0x00),
	BCMODE,		DATA(0x01),
	WRCTRLD,	DATA(0x24),
	DISPON,
};

static const struct s6d04d1_panel sony_panel = {
	.mtp_value = {
		0x52, 0x2c, 0x11
	},
	.power_config = power_config_sony,
	.power_config_len = sizeof(power_config_sony),
	.display_config = display_config_sony,
	.display_config_len = sizeof(display_config_sony),
};

/* Samsung Mobile Display WQVGA 3.2" panel */
static const u16 power_config_smd[] = {
	POWCTL,		DATA(0x80), DATA(0x00), DATA(0x00), DATA(0x0B),
			DATA(0x33), DATA(0x7F), DATA(0x7F),
	VCMCTL,		DATA(0x6E), DATA(0x6E), DATA(0x7F), DATA(0x7F),
			DATA(0x33),
	SRCCTL,		DATA(0x12), DATA(0x00), DATA(0x03), DATA(0xF0),
			DATA(0x70),
	SLPOUT,
};

static const u16 display_config_smd[] = {
	MADCTL,		DATA(0x98),
	COLMOD,		DATA(0x77),
	DISCTL,		DATA(0x14), DATA(0x14), DATA(0x03), DATA(0x03),
			DATA(0x04), DATA(0x03), DATA(0x04), DATA(0x10),
			DATA(0x04), DATA(0x14), DATA(0x14),
	IFCTL,		DATA(0x00), DATA(0x81), DATA(0x30), DATA(0x10),
	GATECTL,	DATA(0x22), DATA(0x01),
	WRDISBV,	DATA(0x00),
	WRCABCMB,	DATA(0x00),
	MIECTL1,	DATA(0x80), DATA(0x80), DATA(0x20),
	BCMODE,		DATA(0x03),
	MIECTL2,	DATA(0x20), DATA(0x01), DATA(0x8F),
	MIECTL3,	DATA(0x7C), DATA(0x01),
	RPGAMCTL,	DATA(0x00), DATA(0x23), DATA(0x15), DATA(0x15),
			DATA(0x1C), DATA(0x19), DATA(0x18), DATA(0x1E),
			DATA(0x24), DATA(0x25), DATA(0x25), DATA(0x20),
			DATA(0x10), DATA(0x22), DATA(0x21),
	RNGAMCTL,	DATA(0x19), DATA(0x00), DATA(0x15), DATA(0x15),
			DATA(0x1C), DATA(0x1F), DATA(0x1E), DATA(0x24),
			DATA(0x1E), DATA(0x1F), DATA(0x25), DATA(0x20),
			DATA(0x10), DATA(0x22), DATA(0x21),
	GPGAMCTL,	DATA(0x06), DATA(0x23), DATA(0x14), DATA(0x14),
			DATA(0x1D), DATA(0x1A), DATA(0x19), DATA(0x1F),
			DATA(0x24), DATA(0x26), DATA(0x30), DATA(0x1E),
			DATA(0x1E), DATA(0x22), DATA(0x21),
	GNGAMCTL,	DATA(0x19), DATA(0x06), DATA(0x14), DATA(0x14),
			DATA(0x1D), DATA(0x20), DATA(0x1F), DATA(0x25),
			DATA(0x1E), DATA(0x20), DATA(0x30), DATA(0x1E),
			DATA(0x1E), DATA(0x22), DATA(0x21),
	BPGAMCTL,	DATA(0x2C), DATA(0x23), DATA(0x20), DATA(0x20),
			DATA(0x23), DATA(0x2F), DATA(0x30), DATA(0x39),
			DATA(0x09), DATA(0x09), DATA(0x18), DATA(0x13),
			DATA(0x13), DATA(0x22), DATA(0x21),
	BNGAMCTL,	DATA(0x19), DATA(0x2C), DATA(0x20), DATA(0x20),
			DATA(0x23), DATA(0x35), DATA(0x36), DATA(0x3F),
			DATA(0x03), DATA(0x03), DATA(0x18), DATA(0x13),
			DATA(0x13), DATA(0x22), DATA(0x21),
	CASET,		DATA(0x00), DATA(0x00), DATA(0x00), DATA(0xEF),
	PASET,		DATA(0x00), DATA(0x00), DATA(0x01), DATA(0x8F),
	RAMWR,		DATA(0x00),
	WRCTRLD,	DATA(0x2C),
	WRCABC,		DATA(0x00),
	DISPON,
};

static const struct s6d04d1_panel smd_panel = {
	.mtp_value = {
		0x52, 0x09, 0x11
	},
	.power_config = power_config_smd,
	.power_config_len = sizeof(power_config_smd),
	.display_config = display_config_smd,
	.display_config_len = sizeof(display_config_smd),
};

static const struct s6d04d1_panel smd_panel2 = {
	.mtp_value = {
		0x53, 0x09, 0x10
	},
	.power_config = power_config_smd,
	.power_config_len = sizeof(power_config_smd),
	.display_config = display_config_smd,
	.display_config_len = sizeof(display_config_smd),
};

static const struct s6d04d1_panel *panels[] = {
	&sony_panel,
	&smd_panel,
	&smd_panel2,
};

/*
 * Display power control
 */

static int s6d04d1_power_is_on(int power)
{
	return power == FB_BLANK_UNBLANK;
}

static int s6d04d1_detect_panel(struct s6d04d1 *lcd)
{
	struct spi_message message;
	struct spi_transfer x[3];
	static const u16 read_ldi[] = { RDDIDIF };
	u16 buf_16[3];
	u8 buf_8[2];
	int ret;
	int i;

	/*
	 * Quirky message for quirky chip...
	 *
	 * Command and first read word are 9-bit and remaining words
	 * are 8-bit...
	 */
	spi_message_init(&message);
	memset(x, 0, sizeof(x));
	x[0].tx_buf = read_ldi;
	x[0].len = sizeof(read_ldi);
	spi_message_add_tail(&x[0], &message);
	x[1].rx_buf = buf_16;
	x[1].len = 2;
	spi_message_add_tail(&x[1], &message);
	x[2].rx_buf = buf_8,
	x[2].len = 2;
	x[2].bits_per_word = 8;
	spi_message_add_tail(&x[2], &message);

	ret = spi_sync(lcd->spi, &message);
	if (ret)
		return ret;

	/* 9th bit of first read word must be ignored (dummy bit) */
	buf_16[0] &= 0xff;
	buf_16[1] = buf_8[0];
	buf_16[2] = buf_8[1];

	for (i = 0; i < ARRAY_SIZE(panels); ++i) {
		if (!memcmp(buf_16, panels[i]->mtp_value, sizeof(buf_16))) {
			lcd->panel = panels[i];
			return 0;
		}
	}

	dev_err(lcd->dev, "unsupported panel (%02x %02x %02x)\n",
		buf_16[0], buf_16[1], buf_16[2]);

	return -EINVAL;
}

static int s6d04d1_brightness_ctrl(struct s6d04d1 *lcd, unsigned int val)
{
	u16 brightness[] = { WRDISBV, DATA(2 * val) };

	return spi_write(lcd->spi, brightness, sizeof(brightness));
}

static int s6d04d1_power_on(struct s6d04d1 *lcd)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(lcd->supplies), lcd->supplies);
	if (ret)
		return ret;

	msleep(10);

	gpio_set_value(lcd->reset_gpio, 1);
	msleep(10);
	gpio_set_value(lcd->reset_gpio, 0);
	msleep(10);
	gpio_set_value(lcd->reset_gpio, 1);
	msleep(20);

	ret = s6d04d1_detect_panel(lcd);
	if (ret)
		return ret;

	ret = spi_write(lcd->spi, lcd->panel->power_config,
			lcd->panel->power_config_len);
	if (ret)
		return ret;

	msleep(120);

	ret = spi_write(lcd->spi, lcd->panel->display_config,
			lcd->panel->display_config_len);
	if (ret)
		return ret;

	msleep(50);

	ret = s6d04d1_brightness_ctrl(lcd, lcd->bd->props.brightness);
	if (ret)
		return ret;

	return 0;
}

static int s6d04d1_power_off(struct s6d04d1 *lcd)
{
	static const u16 display_off[] = {
		WRCTRLD,	DATA(0x00),
		DISPOFF,
	};
	static const u16 sleep_in[] = {
		SLPIN,
	};
	int ret;

	ret = spi_write(lcd->spi, display_off, sizeof(display_off));
	if (ret)
		return ret;

	msleep(120);

	ret = spi_write(lcd->spi, sleep_in, sizeof(sleep_in));
	if (ret)
		return ret;

	msleep(50);

	gpio_set_value(lcd->reset_gpio, 0);

	ret = regulator_bulk_disable(ARRAY_SIZE(lcd->supplies), lcd->supplies);
	if (ret)
		return ret;

	return 0;
}

static int s6d04d1_power(struct s6d04d1 *lcd, int power)
{
	int ret = 0;

	if (s6d04d1_power_is_on(power) && !s6d04d1_power_is_on(lcd->power))
		ret = s6d04d1_power_on(lcd);
	else if (!s6d04d1_power_is_on(power) && s6d04d1_power_is_on(lcd->power))
		ret = s6d04d1_power_off(lcd);

	if (!ret)
		lcd->power = power;

	return ret;
}

static int s6d04d1_set_power(struct lcd_device *ld, int power)
{
	struct s6d04d1 *lcd = lcd_get_data(ld);

	if (power != FB_BLANK_UNBLANK && power != FB_BLANK_POWERDOWN &&
		power != FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "power value should be 0, 1 or 4.\n");
		return -EINVAL;
	}

	return s6d04d1_power(lcd, power);
}

static int s6d04d1_get_power(struct lcd_device *ld)
{
	struct s6d04d1 *lcd = lcd_get_data(ld);

	return lcd->power;
}

static int s6d04d1_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int s6d04d1_set_brightness(struct backlight_device *bd)
{
	struct s6d04d1 *lcd = bl_get_data(bd);
	int ret;

	ret = s6d04d1_brightness_ctrl(lcd, bd->props.brightness);
	if (ret) {
		dev_err(&bd->dev, "lcd brightness setting failed.\n");
		return -EIO;
	}

	return ret;
}

static struct lcd_ops s6d04d1_lcd_ops = {
	.set_power = s6d04d1_set_power,
	.get_power = s6d04d1_get_power,
};

static const struct backlight_ops s6d04d1_backlight_ops  = {
	.get_brightness = s6d04d1_get_brightness,
	.update_status = s6d04d1_set_brightness,
};

static int s6d04d1_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct backlight_properties props;
	struct backlight_device *bd;
	struct lcd_device *ld;
	struct s6d04d1 *lcd;
	int ret;

	if (!np) {
		dev_err(&spi->dev, "device must be instantiated using DT\n");
		return -EINVAL;
	}

	lcd = devm_kzalloc(&spi->dev, sizeof(struct s6d04d1), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	spi->bits_per_word = 9;
	spi->mode = SPI_MODE_3;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "spi setup failed.\n");
		return ret;
	}

	lcd->spi = spi;
	lcd->dev = &spi->dev;
	lcd->supplies[0].supply = "vdd3";
	lcd->supplies[1].supply = "vci";

	ld = devm_lcd_device_register(&spi->dev, "s6d04d1", &spi->dev, lcd,
					&s6d04d1_lcd_ops);
	if (IS_ERR(ld))
		return PTR_ERR(ld);

	lcd->ld = ld;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS;

	bd = devm_backlight_device_register(&spi->dev, "s6d04d1bl-bl",
						&spi->dev, lcd,
						&s6d04d1_backlight_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	bd->props.brightness = MAX_BRIGHTNESS;
	lcd->bd = bd;

	if (s6d04d1_detect_panel(lcd) < 0)
		lcd->power = FB_BLANK_POWERDOWN;
	else
		lcd->power = FB_BLANK_UNBLANK;

	lcd->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (lcd->reset_gpio < 0)
		return lcd->reset_gpio;

	ret = devm_regulator_bulk_get(lcd->dev, ARRAY_SIZE(lcd->supplies),
					lcd->supplies);
	if (ret)
		return ret;

	ret = devm_gpio_request_one(lcd->dev, lcd->reset_gpio,
					GPIOF_OUT_INIT_HIGH, "s6d04d1-reset");
	if (ret) {
		dev_err(&spi->dev, "failed to request reset GPIO\n");
		return ret;
	}

	if (lcd->power == FB_BLANK_UNBLANK) {
		ret = regulator_bulk_enable(ARRAY_SIZE(lcd->supplies),
						lcd->supplies);
		if (ret)
			return ret;
	} else {
		gpio_set_value(lcd->reset_gpio, 0);
		s6d04d1_power(lcd, FB_BLANK_UNBLANK);
	}

	spi_set_drvdata(spi, lcd);

	dev_dbg(&spi->dev, "s6d04d1 panel driver has been probed.\n");

	return 0;
}

static int s6d04d1_remove(struct spi_device *spi)
{
	struct s6d04d1 *lcd = spi_get_drvdata(spi);

	s6d04d1_power(lcd, FB_BLANK_POWERDOWN);

	return 0;
}

/* Power down all displays on reboot, poweroff or halt. */
static void s6d04d1_shutdown(struct spi_device *spi)
{
	struct s6d04d1 *lcd = spi_get_drvdata(spi);

	s6d04d1_power(lcd, FB_BLANK_POWERDOWN);
}

#ifdef CONFIG_OF
static const struct of_device_id s6d04d1_of_match[] = {
	{ .compatible = "samsung,s6d04d1", },
	{ /* sentinel */ }
};
#endif

static struct spi_driver s6d04d1_driver = {
	.driver = {
		.name	= "s6d04d1",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(s6d04d1_of_match),
	},
	.probe		= s6d04d1_probe,
	.remove		= s6d04d1_remove,
	.shutdown	= s6d04d1_shutdown,
};
module_spi_driver(s6d04d1_driver);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("S6D04D1 LCD Driver");
MODULE_LICENSE("GPL v2");
