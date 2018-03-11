/*
 * drivers/media/platform/samsung/mfc5/s5p_mfc_intr.c
 *
 * C file for Samsung MFC (Multi Function Codec - FIMV) driver
 * This file contains functions used to wait for command completion.
 *
 * Kamil Debski, Copyright (C) 2011 Samsung Electronics Co., Ltd.
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include "s5p_mfc_common.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_opr.h"

int s5p_mfc_wait_for_done_dev(struct s5p_mfc_dev *dev, int command)
{
	int ret;

	ret = wait_event_interruptible_timeout(dev->queue,
		!s5p_mfc_hw_is_locked(dev),
		msecs_to_jiffies(MFC_INT_TIMEOUT));
	if (ret == 0) {
		mfc_err("Interrupt (dev->int_type:%d, command:%d) timed out\n",
							dev->int_type, command);
		return -ETIMEDOUT;
	} else if (ret == -ERESTARTSYS) {
		mfc_err("Interrupted by a signal\n");
		return -ERESTARTSYS;
	}
	mfc_debug(1, "Finished waiting (dev->int_type:%d, command: %d)\n",
							dev->int_type, command);
	if (dev->int_type == S5P_MFC_R2H_CMD_ERR_RET || dev->int_err != 0)
		return -EIO;
	if (dev->int_type != command)
		return -EINVAL;
	return 0;
}

/* Must be called with dev->mfc_mutex held. */
int s5p_mfc_wait_for_done_ctx(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	/*
	 * The mutex prevents new work from being queued for ctx
	 * and ctx->state from changing after the wait returns.
	 */
	WARN_ON(!mutex_is_locked(&dev->mfc_mutex));

	/* Timeouts handled by watchdog. */
	wait_event(dev->queue, test_bit(ctx->num, &dev->ctx_work_bits));

	if (s5p_mfc_ctx_has_error(ctx))
		return -EIO;

	return 0;
}

