/*
 * Copyright (c) 2014 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Device Tree binding constants for Samsung S3C64xx power domains.
*/

#ifndef _DT_BINDINGS_ARM_S3C64XX_POWER_DOMAINS_H
#define _DT_BINDINGS_ARM_S3C64XX_POWER_DOMAINS_H

#define DOMAIN_V		0
#define DOMAIN_G		1
#define DOMAIN_I		2
#define DOMAIN_P		3
#define DOMAIN_F		4
#define DOMAIN_S		5
#define DOMAIN_ETM		6
#define DOMAIN_IROM		7

/* Total number of clocks. */
#define NR_DOMAINS		(DOMAIN_IROM + 1)

#endif /* _DT_BINDINGS_ARM_S3C64XX_POWER_DOMAINS_H */
