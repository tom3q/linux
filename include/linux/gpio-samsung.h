/*
 * Copyright 2008 Openmoko, Inc.
 * Copyright (c) 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * GPIO support for legacy platforms
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _GPIO_SAMSUNG_H
#define _GPIO_SAMSUNG_H

#include <linux/gpio.h>

/* A big number which should be enough for all supported platforms. */
#define S3C_GPIO_END	384

/*
 * S3C24xx specific definitions
 */

/*
 * GPIO sizes for various SoCs:
 *
 *   2410 2412 2440 2443 2416
 *             2442
 *   ---- ---- ---- ---- ----
 * A  23   22   25   16   27
 * B  11   11   11   11   11
 * C  16   16   16   16   16
 * D  16   16   16   16   16
 * E  16   16   16   16   16
 * F  8    8    8    8    8
 * G  16   16   16   16   8
 * H  11   11   11   15   15
 * J  --   --   13   16   --
 * K  --   --   --   --   16
 * L  --   --   --   15   14
 * M  --   --   --   2    2
 */

/* GPIO bank sizes */

#define S3C2410_GPIO_A_NR	(32)
#define S3C2410_GPIO_B_NR	(32)
#define S3C2410_GPIO_C_NR	(32)
#define S3C2410_GPIO_D_NR	(32)
#define S3C2410_GPIO_E_NR	(32)
#define S3C2410_GPIO_F_NR	(32)
#define S3C2410_GPIO_G_NR	(32)
#define S3C2410_GPIO_H_NR	(32)
#define S3C2410_GPIO_J_NR	(32)	/* technically 16. */
#define S3C2410_GPIO_K_NR	(32)	/* technically 16. */
#define S3C2410_GPIO_L_NR	(32)	/* technically 15. */
#define S3C2410_GPIO_M_NR	(32)	/* technically 2. */

#define S3C2410_GPIO_NEXT(__gpio) \
	((__gpio##_START) + (__gpio##_NR))

#ifndef __ASSEMBLY__

enum {
	S3C2410_GPIO_A_START = 0,
	S3C2410_GPIO_B_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_A),
	S3C2410_GPIO_C_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_B),
	S3C2410_GPIO_D_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_C),
	S3C2410_GPIO_E_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_D),
	S3C2410_GPIO_F_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_E),
	S3C2410_GPIO_G_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_F),
	S3C2410_GPIO_H_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_G),
	S3C2410_GPIO_J_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_H),
	S3C2410_GPIO_K_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_J),
	S3C2410_GPIO_L_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_K),
	S3C2410_GPIO_M_START = S3C2410_GPIO_NEXT(S3C2410_GPIO_L),
};

#endif /* __ASSEMBLY__ */

/* S3C2410 GPIO number definitions. */

#define S3C2410_GPA(_nr)	(S3C2410_GPIO_A_START + (_nr))
#define S3C2410_GPB(_nr)	(S3C2410_GPIO_B_START + (_nr))
#define S3C2410_GPC(_nr)	(S3C2410_GPIO_C_START + (_nr))
#define S3C2410_GPD(_nr)	(S3C2410_GPIO_D_START + (_nr))
#define S3C2410_GPE(_nr)	(S3C2410_GPIO_E_START + (_nr))
#define S3C2410_GPF(_nr)	(S3C2410_GPIO_F_START + (_nr))
#define S3C2410_GPG(_nr)	(S3C2410_GPIO_G_START + (_nr))
#define S3C2410_GPH(_nr)	(S3C2410_GPIO_H_START + (_nr))
#define S3C2410_GPJ(_nr)	(S3C2410_GPIO_J_START + (_nr))
#define S3C2410_GPK(_nr)	(S3C2410_GPIO_K_START + (_nr))
#define S3C2410_GPL(_nr)	(S3C2410_GPIO_L_START + (_nr))
#define S3C2410_GPM(_nr)	(S3C2410_GPIO_M_START + (_nr))

/*
 * S3C64xx specific definitions
 */

/* GPIO bank sizes */
#define S3C64XX_GPIO_A_NR	(8)
#define S3C64XX_GPIO_B_NR	(7)
#define S3C64XX_GPIO_C_NR	(8)
#define S3C64XX_GPIO_D_NR	(5)
#define S3C64XX_GPIO_E_NR	(5)
#define S3C64XX_GPIO_F_NR	(16)
#define S3C64XX_GPIO_G_NR	(7)
#define S3C64XX_GPIO_H_NR	(10)
#define S3C64XX_GPIO_I_NR	(16)
#define S3C64XX_GPIO_J_NR	(12)
#define S3C64XX_GPIO_K_NR	(16)
#define S3C64XX_GPIO_L_NR	(15)
#define S3C64XX_GPIO_M_NR	(6)
#define S3C64XX_GPIO_N_NR	(16)
#define S3C64XX_GPIO_O_NR	(16)
#define S3C64XX_GPIO_P_NR	(15)
#define S3C64XX_GPIO_Q_NR	(9)

/* GPIO bank numbes */

#define S3C64XX_GPIO_NEXT(__gpio) \
	((__gpio##_START) + (__gpio##_NR) + 1)

enum {
	S3C64XX_GPIO_A_START = 0,
	S3C64XX_GPIO_B_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_A),
	S3C64XX_GPIO_C_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_B),
	S3C64XX_GPIO_D_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_C),
	S3C64XX_GPIO_E_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_D),
	S3C64XX_GPIO_F_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_E),
	S3C64XX_GPIO_G_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_F),
	S3C64XX_GPIO_H_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_G),
	S3C64XX_GPIO_I_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_H),
	S3C64XX_GPIO_J_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_I),
	S3C64XX_GPIO_K_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_J),
	S3C64XX_GPIO_L_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_K),
	S3C64XX_GPIO_M_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_L),
	S3C64XX_GPIO_N_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_M),
	S3C64XX_GPIO_O_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_N),
	S3C64XX_GPIO_P_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_O),
	S3C64XX_GPIO_Q_START = S3C64XX_GPIO_NEXT(S3C64XX_GPIO_P),
};

/* S3C64XX GPIO number definitions. */

#define S3C64XX_GPA(_nr)	(S3C64XX_GPIO_A_START + (_nr))
#define S3C64XX_GPB(_nr)	(S3C64XX_GPIO_B_START + (_nr))
#define S3C64XX_GPC(_nr)	(S3C64XX_GPIO_C_START + (_nr))
#define S3C64XX_GPD(_nr)	(S3C64XX_GPIO_D_START + (_nr))
#define S3C64XX_GPE(_nr)	(S3C64XX_GPIO_E_START + (_nr))
#define S3C64XX_GPF(_nr)	(S3C64XX_GPIO_F_START + (_nr))
#define S3C64XX_GPG(_nr)	(S3C64XX_GPIO_G_START + (_nr))
#define S3C64XX_GPH(_nr)	(S3C64XX_GPIO_H_START + (_nr))
#define S3C64XX_GPI(_nr)	(S3C64XX_GPIO_I_START + (_nr))
#define S3C64XX_GPJ(_nr)	(S3C64XX_GPIO_J_START + (_nr))
#define S3C64XX_GPK(_nr)	(S3C64XX_GPIO_K_START + (_nr))
#define S3C64XX_GPL(_nr)	(S3C64XX_GPIO_L_START + (_nr))
#define S3C64XX_GPM(_nr)	(S3C64XX_GPIO_M_START + (_nr))
#define S3C64XX_GPN(_nr)	(S3C64XX_GPIO_N_START + (_nr))
#define S3C64XX_GPO(_nr)	(S3C64XX_GPIO_O_START + (_nr))
#define S3C64XX_GPP(_nr)	(S3C64XX_GPIO_P_START + (_nr))
#define S3C64XX_GPQ(_nr)	(S3C64XX_GPIO_Q_START + (_nr))

/*
 * GPIO core
 */

#define GPIOCON_OFF	(0x00)
#define GPIODAT_OFF	(0x04)

#define con_4bit_shift(__off) ((__off) * 4)

/* Define the core gpiolib support functions that the s3c platforms may
 * need to extend or change depending on the hardware and the s3c chip
 * selected at build or found at run time.
 *
 * These definitions are not intended for driver inclusion, there is
 * nothing here that should not live outside the platform and core
 * specific code.
*/

struct samsung_gpio_chip;

/**
 * struct samsung_gpio_pm - power management (suspend/resume) information
 * @save: Routine to save the state of the GPIO block
 * @resume: Routine to resume the GPIO block.
 */
struct samsung_gpio_pm {
	void (*save)(struct samsung_gpio_chip *chip);
	void (*resume)(struct samsung_gpio_chip *chip);
};

struct samsung_gpio_cfg;

/**
 * struct samsung_gpio_chip - wrapper for specific implementation of gpio
 * @chip: The chip structure to be exported via gpiolib.
 * @offset: The base offset to the gpio configuration registers.
 * @base: The base pointer to the gpio configuration registers.
 * @group: The group register number for gpio interrupt support.
 * @irq_base: The base irq number.
 * @config: special function and pull-resistor control information.
 * @lock: Lock for exclusive access to this gpio bank.
 * @pm_save: Save information for suspend/resume support.
 * @bitmap_gpio_int: Bitmap for representing GPIO interrupt or not.
 *
 * This wrapper provides the necessary information for the Samsung
 * specific gpios being registered with gpiolib.
 *
 * The lock protects each gpio bank from multiple access of the shared
 * configuration registers, or from reading of data whilst another thread
 * is writing to the register set.
 *
 * Each chip has its own lock to avoid any  contention between different
 * CPU cores trying to get one lock for different GPIO banks, where each
 * bank of GPIO has its own register space and configuration registers.
 */
struct samsung_gpio_chip {
	struct gpio_chip	chip;
	struct samsung_gpio_cfg	*config;
	struct samsung_gpio_pm	*pm;
	u32			offset;
	void __iomem		*base;
	int			irq_base;
	int			group;
	spinlock_t		 lock;
#ifdef CONFIG_PM
	u32			pm_save[4];
#endif
	u32			bitmap_gpio_int;
};

static inline struct samsung_gpio_chip *to_samsung_gpio(struct gpio_chip *gpc)
{
	return container_of(gpc, struct samsung_gpio_chip, chip);
}

/**
 * samsung_gpiolib_to_irq - convert gpio pin to irq number
 * @chip: The gpio chip that the pin belongs to.
 * @offset: The offset of the pin in the chip.
 *
 * This helper returns the irq number calculated from the chip->irq_base and
 * the provided offset.
 */
extern int samsung_gpiolib_to_irq(struct gpio_chip *chip, unsigned int offset);

/* exported for core SoC support to change */
extern struct samsung_gpio_cfg s3c24xx_gpiocfg_default;

#ifdef CONFIG_S3C_GPIO_TRACK
extern struct samsung_gpio_chip *s3c_gpios[S3C_GPIO_END];

static inline struct samsung_gpio_chip *samsung_gpiolib_getchip(unsigned int chip)
{
	return (chip < S3C_GPIO_END) ? s3c_gpios[chip] : NULL;
}
#else
/* machine specific code should provide samsung_gpiolib_getchip */

extern struct samsung_gpio_chip s3c24xx_gpios[];

static inline struct samsung_gpio_chip *samsung_gpiolib_getchip(unsigned int pin)
{
	struct samsung_gpio_chip *chip;

	if (pin > S3C_GPIO_END)
		return NULL;

	chip = &s3c24xx_gpios[pin/32];
	return ((pin - chip->chip.base) < chip->chip.ngpio) ? chip : NULL;
}

static inline void s3c_gpiolib_track(struct samsung_gpio_chip *chip) { }
#endif

#ifdef CONFIG_PM
extern struct samsung_gpio_pm samsung_gpio_pm_1bit;
extern struct samsung_gpio_pm samsung_gpio_pm_2bit;
extern struct samsung_gpio_pm samsung_gpio_pm_4bit;
#define __gpio_pm(x) x
#else
#define samsung_gpio_pm_1bit NULL
#define samsung_gpio_pm_2bit NULL
#define samsung_gpio_pm_4bit NULL
#define __gpio_pm(x) NULL

#endif /* CONFIG_PM */

/* locking wrappers to deal with multiple access to the same gpio bank */
#define samsung_gpio_lock(_oc, _fl) spin_lock_irqsave(&(_oc)->lock, _fl)
#define samsung_gpio_unlock(_oc, _fl) spin_unlock_irqrestore(&(_oc)->lock, _fl)

/*
 * GPIO cfg
 */

#include <linux/types.h>

typedef unsigned int __bitwise__ samsung_gpio_pull_t;

/* forward declaration if gpio-core.h hasn't been included */
struct samsung_gpio_chip;

/**
 * struct samsung_gpio_cfg GPIO configuration
 * @cfg_eint: Configuration setting when used for external interrupt source
 * @get_pull: Read the current pull configuration for the GPIO
 * @set_pull: Set the current pull configuraiton for the GPIO
 * @set_config: Set the current configuration for the GPIO
 * @get_config: Read the current configuration for the GPIO
 *
 * Each chip can have more than one type of GPIO bank available and some
 * have different capabilites even when they have the same control register
 * layouts. Provide an point to vector control routine and provide any
 * per-bank configuration information that other systems such as the
 * external interrupt code will need.
 *
 * @sa samsung_gpio_cfgpin
 * @sa s3c_gpio_getcfg
 * @sa s3c_gpio_setpull
 * @sa s3c_gpio_getpull
 */
struct samsung_gpio_cfg {
	unsigned int	cfg_eint;

	samsung_gpio_pull_t	(*get_pull)(struct samsung_gpio_chip *chip, unsigned offs);
	int		(*set_pull)(struct samsung_gpio_chip *chip, unsigned offs,
				    samsung_gpio_pull_t pull);

	unsigned (*get_config)(struct samsung_gpio_chip *chip, unsigned offs);
	int	 (*set_config)(struct samsung_gpio_chip *chip, unsigned offs,
			       unsigned config);
};

#define S3C_GPIO_SPECIAL_MARK	(0xfffffff0)
#define S3C_GPIO_SPECIAL(x)	(S3C_GPIO_SPECIAL_MARK | (x))

/* Defines for generic pin configurations */
#define S3C_GPIO_INPUT	(S3C_GPIO_SPECIAL(0))
#define S3C_GPIO_OUTPUT	(S3C_GPIO_SPECIAL(1))
#define S3C_GPIO_SFN(x)	(S3C_GPIO_SPECIAL(x))

#define samsung_gpio_is_cfg_special(_cfg) \
	(((_cfg) & S3C_GPIO_SPECIAL_MARK) == S3C_GPIO_SPECIAL_MARK)

/**
 * s3c_gpio_cfgpin() - Change the GPIO function of a pin.
 * @pin pin The pin number to configure.
 * @to to The configuration for the pin's function.
 *
 * Configure which function is actually connected to the external
 * pin, such as an gpio input, output or some form of special function
 * connected to an internal peripheral block.
 *
 * The @to parameter can be one of the generic S3C_GPIO_INPUT, S3C_GPIO_OUTPUT
 * or S3C_GPIO_SFN() to indicate one of the possible values that the helper
 * will then generate the correct bit mask and shift for the configuration.
 *
 * If a bank of GPIOs all needs to be set to special-function 2, then
 * the following code will work:
 *
 *	for (gpio = start; gpio < end; gpio++)
 *		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
 *
 * The @to parameter can also be a specific value already shifted to the
 * correct position in the control register, although these are discouraged
 * in newer kernels and are only being kept for compatibility.
 */
extern int s3c_gpio_cfgpin(unsigned int pin, unsigned int to);

/**
 * s3c_gpio_getcfg - Read the current function for a GPIO pin
 * @pin: The pin to read the configuration value for.
 *
 * Read the configuration state of the given @pin, returning a value that
 * could be passed back to s3c_gpio_cfgpin().
 *
 * @sa s3c_gpio_cfgpin
 */
extern unsigned s3c_gpio_getcfg(unsigned int pin);

/**
 * s3c_gpio_cfgpin_range() - Change the GPIO function for configuring pin range
 * @start: The pin number to start at
 * @nr: The number of pins to configure from @start.
 * @cfg: The configuration for the pin's function
 *
 * Call s3c_gpio_cfgpin() for the @nr pins starting at @start.
 *
 * @sa s3c_gpio_cfgpin.
 */
extern int s3c_gpio_cfgpin_range(unsigned int start, unsigned int nr,
				 unsigned int cfg);

/* Define values for the pull-{up,down} available for each gpio pin.
 *
 * These values control the state of the weak pull-{up,down} resistors
 * available on most pins on the S3C series. Not all chips support both
 * up or down settings, and it may be dependent on the chip that is being
 * used to whether the particular mode is available.
 */
#define S3C_GPIO_PULL_NONE	((__force samsung_gpio_pull_t)0x00)
#define S3C_GPIO_PULL_DOWN	((__force samsung_gpio_pull_t)0x01)
#define S3C_GPIO_PULL_UP	((__force samsung_gpio_pull_t)0x02)

/**
 * s3c_gpio_setpull() - set the state of a gpio pin pull resistor
 * @pin: The pin number to configure the pull resistor.
 * @pull: The configuration for the pull resistor.
 *
 * This function sets the state of the pull-{up,down} resistor for the
 * specified pin. It will return 0 if successful, or a negative error
 * code if the pin cannot support the requested pull setting.
 *
 * @pull is one of S3C_GPIO_PULL_NONE, S3C_GPIO_PULL_DOWN or S3C_GPIO_PULL_UP.
*/
extern int s3c_gpio_setpull(unsigned int pin, samsung_gpio_pull_t pull);

/**
 * s3c_gpio_getpull() - get the pull resistor state of a gpio pin
 * @pin: The pin number to get the settings for
 *
 * Read the pull resistor value for the specified pin.
*/
extern samsung_gpio_pull_t s3c_gpio_getpull(unsigned int pin);

/* configure `all` aspects of an gpio */

/**
 * s3c_gpio_cfgall_range() - configure range of gpio functtion and pull.
 * @start: The gpio number to start at.
 * @nr: The number of gpio to configure from @start.
 * @cfg: The configuration to use
 * @pull: The pull setting to use.
 *
 * Run s3c_gpio_cfgpin() and s3c_gpio_setpull() over the gpio range starting
 * @gpio and running for @size.
 *
 * @sa s3c_gpio_cfgpin
 * @sa s3c_gpio_setpull
 * @sa s3c_gpio_cfgpin_range
 */
extern int s3c_gpio_cfgall_range(unsigned int start, unsigned int nr,
				 unsigned int cfg, samsung_gpio_pull_t pull);

static inline int s3c_gpio_cfgrange_nopull(unsigned int pin, unsigned int size,
					   unsigned int cfg)
{
	return s3c_gpio_cfgall_range(pin, size, cfg, S3C_GPIO_PULL_NONE);
}

/*
 * GPIO cfg helpers
 */

/*
 * As a note, all gpio configuration functions are entered exclusively, either
 * with the relevant lock held or the system prevented from doing anything else
 * by disabling interrupts.
 */

static inline int samsung_gpio_do_setcfg(struct samsung_gpio_chip *chip,
					 unsigned int off, unsigned int config)
{
	return (chip->config->set_config)(chip, off, config);
}

static inline unsigned samsung_gpio_do_getcfg(struct samsung_gpio_chip *chip,
					      unsigned int off)
{
	return (chip->config->get_config)(chip, off);
}

static inline int samsung_gpio_do_setpull(struct samsung_gpio_chip *chip,
					  unsigned int off, samsung_gpio_pull_t pull)
{
	return (chip->config->set_pull)(chip, off, pull);
}

static inline samsung_gpio_pull_t samsung_gpio_do_getpull(struct samsung_gpio_chip *chip,
							  unsigned int off)
{
	return chip->config->get_pull(chip, off);
}

/* Pull-{up,down} resistor controls.
 *
 * S3C2410,S3C2440 = Pull-UP,
 * S3C2412,S3C2413 = Pull-Down
 * S3C6400,S3C6410 = Pull-Both [None,Down,Up,Undef]
 * S3C2443 = Pull-Both [not same as S3C6400]
 */

/**
 * s3c24xx_gpio_setpull_1up() - Pull configuration for choice of up or none.
 * @chip: The gpio chip that is being configured.
 * @off: The offset for the GPIO being configured.
 * @param: pull: The pull mode being requested.
 *
 * This is a helper function for the case where we have GPIOs with one
 * bit configuring the presence of a pull-up resistor.
 */
extern int s3c24xx_gpio_setpull_1up(struct samsung_gpio_chip *chip,
				    unsigned int off, samsung_gpio_pull_t pull);

/**
 * s3c24xx_gpio_setpull_1down() - Pull configuration for choice of down or none
 * @chip: The gpio chip that is being configured
 * @off: The offset for the GPIO being configured
 * @param: pull: The pull mode being requested
 *
 * This is a helper function for the case where we have GPIOs with one
 * bit configuring the presence of a pull-down resistor.
 */
extern int s3c24xx_gpio_setpull_1down(struct samsung_gpio_chip *chip,
				      unsigned int off, samsung_gpio_pull_t pull);

/**
 * samsung_gpio_setpull_upown() - Pull configuration for choice of up,
 * down or none
 *
 * @chip: The gpio chip that is being configured.
 * @off: The offset for the GPIO being configured.
 * @param: pull: The pull mode being requested.
 *
 * This is a helper function for the case where we have GPIOs with two
 * bits configuring the presence of a pull resistor, in the following
 * order:
 *	00 = No pull resistor connected
 *	01 = Pull-up resistor connected
 *	10 = Pull-down resistor connected
 */
extern int samsung_gpio_setpull_updown(struct samsung_gpio_chip *chip,
				       unsigned int off, samsung_gpio_pull_t pull);

/**
 * samsung_gpio_getpull_updown() - Get configuration for choice of up,
 * down or none
 *
 * @chip: The gpio chip that the GPIO pin belongs to
 * @off: The offset to the pin to get the configuration of.
 *
 * This helper function reads the state of the pull-{up,down} resistor
 * for the given GPIO in the same case as samsung_gpio_setpull_upown.
*/
extern samsung_gpio_pull_t samsung_gpio_getpull_updown(struct samsung_gpio_chip *chip,
						       unsigned int off);

/**
 * s3c24xx_gpio_getpull_1up() - Get configuration for choice of up or none
 * @chip: The gpio chip that the GPIO pin belongs to
 * @off: The offset to the pin to get the configuration of.
 *
 * This helper function reads the state of the pull-up resistor for the
 * given GPIO in the same case as s3c24xx_gpio_setpull_1up.
*/
extern samsung_gpio_pull_t s3c24xx_gpio_getpull_1up(struct samsung_gpio_chip *chip,
						    unsigned int off);

/**
 * s3c24xx_gpio_getpull_1down() - Get configuration for choice of down or none
 * @chip: The gpio chip that the GPIO pin belongs to
 * @off: The offset to the pin to get the configuration of.
 *
 * This helper function reads the state of the pull-down resistor for the
 * given GPIO in the same case as s3c24xx_gpio_setpull_1down.
*/
extern samsung_gpio_pull_t s3c24xx_gpio_getpull_1down(struct samsung_gpio_chip *chip,
						      unsigned int off);

/**
 * s3c2443_gpio_setpull() - Pull configuration for s3c2443.
 * @chip: The gpio chip that is being configured.
 * @off: The offset for the GPIO being configured.
 * @param: pull: The pull mode being requested.
 *
 * This is a helper function for the case where we have GPIOs with two
 * bits configuring the presence of a pull resistor, in the following
 * order:
 *	00 = Pull-up resistor connected
 *	10 = Pull-down resistor connected
 *	x1 = No pull up resistor
 */
extern int s3c2443_gpio_setpull(struct samsung_gpio_chip *chip,
				unsigned int off, samsung_gpio_pull_t pull);

/**
 * s3c2443_gpio_getpull() - Get configuration for s3c2443 pull resistors
 * @chip: The gpio chip that the GPIO pin belongs to.
 * @off: The offset to the pin to get the configuration of.
 *
 * This helper function reads the state of the pull-{up,down} resistor for the
 * given GPIO in the same case as samsung_gpio_setpull_upown.
*/
extern samsung_gpio_pull_t s3c2443_gpio_getpull(struct samsung_gpio_chip *chip,
						unsigned int off);

#endif
