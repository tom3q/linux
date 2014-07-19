/* linux/arch/arm/plat-s3c/include/plat/gpio-cfg.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C Platform - GPIO pin configuration
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/* This file contains the necessary definitions to get the basic gpio
 * pin configuration done such as setting a pin to input or output or
 * changing the pull-{up,down} configurations.
 */

/* Note, this interface is being added to the s3c64xx arch first and will
 * be added to the s3c24xx systems later.
 */

#ifndef __PLAT_GPIO_CFG_H
#define __PLAT_GPIO_CFG_H __FILE__

#include <linux/gpio-samsung.h>

/* FIXME: Fix remaining users of this header. */

#endif /* __PLAT_GPIO_CFG_H */
