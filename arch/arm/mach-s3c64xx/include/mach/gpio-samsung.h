/*
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C6400 - GPIO lib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef GPIO_SAMSUNG_S3C64XX_H
#define GPIO_SAMSUNG_S3C64XX_H

#include <linux/gpio-samsung.h>

/* FIXME: Fix remaining users of this header. */

/* the end of the S3C64XX specific gpios */
#define S3C64XX_GPIO_END	(S3C64XX_GPQ(S3C64XX_GPIO_Q_NR) + 1)

/* define the number of gpios we need to the one after the GPQ() range */
#define GPIO_BOARD_START (S3C64XX_GPQ(S3C64XX_GPIO_Q_NR) + 1)

#endif /* GPIO_SAMSUNG_S3C64XX_H */
