/*
 * drivers/media/platform/s5p-mfc/s5p_mfc_opr.c
 *
 * Samsung MFC (Multi Function Codec - FIMV) driver
 * This file contains hw related functions.
 *
 * Kamil Debski, Copyright (c) 2012 Samsung Electronics Co., Ltd.
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "s5p_mfc_debug.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_opr.h"
#include "s5p_mfc_opr_v5.h"
#include "s5p_mfc_opr_v6.h"
#include "s5p_mfc_pm.h"

static struct s5p_mfc_hw_ops *s5p_mfc_ops;

void s5p_mfc_init_hw_ops(struct s5p_mfc_dev *dev)
{
	if (IS_MFCV6_PLUS(dev)) {
		s5p_mfc_ops = s5p_mfc_init_hw_ops_v6();
		dev->warn_start = S5P_FIMV_ERR_WARNINGS_START_V6;
	} else {
		s5p_mfc_ops = s5p_mfc_init_hw_ops_v5();
		dev->warn_start = S5P_FIMV_ERR_WARNINGS_START;
	}
	dev->mfc_ops = s5p_mfc_ops;
}

void s5p_mfc_init_regs(struct s5p_mfc_dev *dev)
{
	if (IS_MFCV6_PLUS(dev))
		dev->mfc_regs = s5p_mfc_init_regs_v6_plus(dev);
}

int s5p_mfc_alloc_priv_buf(struct s5p_mfc_dev *dev, unsigned int mem_ctx,
			   struct s5p_mfc_priv_buf *b)
{
	unsigned int bits = dev->mem_size >> PAGE_SHIFT;
	unsigned int count = b->size >> PAGE_SHIFT;
	unsigned int align = (SZ_64K >> PAGE_SHIFT) - 1;
	unsigned int start, offset;

	mfc_debug(3, "Allocating priv: %zu\n", b->size);

	if (dev->mem_virt) {
		start = bitmap_find_next_zero_area(dev->mem_bitmap, bits, 0, count, align);
		if (start > bits)
			goto no_mem;

		bitmap_set(dev->mem_bitmap, start, count);
		offset = start << PAGE_SHIFT;
		b->virt = dev->mem_virt + offset;
		b->dma = dev->mem_base + offset;
	} else {
		struct device *mem_dev = dev->mem_dev[mem_ctx];
		dma_addr_t base = dev->dma_base[mem_ctx];

		b->ctx = mem_ctx;
		b->virt = dma_alloc_coherent(mem_dev, b->size, &b->dma, GFP_KERNEL);
		if (!b->virt)
			goto no_mem;
		if (b->dma < base) {
			mfc_err("Invalid memory configuration - buffer (%pad) is below base memory address(%pad)\n",
				&b->dma, &base);
			dma_free_coherent(mem_dev, b->size, b->virt, b->dma);
			return -ENOMEM;
		}
	}

	mfc_debug(3, "Allocated addr %p %pad\n", b->virt, &b->dma);
	return 0;
no_mem:
	mfc_err("Allocating private buffer of size %zu failed\n", b->size);
	return -ENOMEM;
}

int s5p_mfc_alloc_generic_buf(struct s5p_mfc_dev *dev, unsigned int mem_ctx,
			   struct s5p_mfc_priv_buf *b)
{
	struct device *mem_dev = dev->mem_dev[mem_ctx];

	mfc_debug(3, "Allocating generic buf: %zu\n", b->size);

	b->ctx = mem_ctx;
	b->virt = dma_alloc_coherent(mem_dev, b->size, &b->dma, GFP_KERNEL);
	if (!b->virt)
		goto no_mem;

	mfc_debug(3, "Allocated addr %p %pad\n", b->virt, &b->dma);
	return 0;
no_mem:
	mfc_err("Allocating generic buffer of size %zu failed\n", b->size);
	return -ENOMEM;
}

void s5p_mfc_release_priv_buf(struct s5p_mfc_dev *dev,
			      struct s5p_mfc_priv_buf *b)
{
	if (dev->mem_virt) {
		unsigned int start = (b->dma - dev->mem_base) >> PAGE_SHIFT;
		unsigned int count = b->size >> PAGE_SHIFT;

		bitmap_clear(dev->mem_bitmap, start, count);
	} else {
		struct device *mem_dev = dev->mem_dev[b->ctx];

		dma_free_coherent(mem_dev, b->size, b->virt, b->dma);
	}
	b->virt = NULL;
	b->dma = 0;
	b->size = 0;
}

void s5p_mfc_release_generic_buf(struct s5p_mfc_dev *dev,
			      struct s5p_mfc_priv_buf *b)
{
	struct device *mem_dev = dev->mem_dev[b->ctx];
	dma_free_coherent(mem_dev, b->size, b->virt, b->dma);
	b->virt = NULL;
	b->dma = 0;
	b->size = 0;
}

/* Try running an operation on hardware */
void s5p_mfc_try_run(struct s5p_mfc_dev *dev)
{
	struct s5p_mfc_ctx *ctx;
	int new_ctx;
	unsigned int ret = 0;

	mfc_debug(1, "Try run dev: %p\n", dev);

	if (test_bit(0, &dev->enter_suspend)) {
		mfc_debug(1, "Entering suspend so do not schedule any jobs\n");
		return;
	}

	/* Check whether hardware is not running */
	if (test_and_set_bit(0, &dev->hw_lock) != 0) {
		/* This is perfectly ok, the scheduled ctx should wait */
		mfc_debug(1, "Couldn't lock HW.\n");
		return;
	}

	/* Choose the context to run */
	new_ctx = s5p_mfc_get_new_ctx(dev);
	if (new_ctx < 0) {
		/* No contexts to run */
		if (test_and_clear_bit(0, &dev->hw_lock) == 0) {
			mfc_err("Failed to unlock hardware.\n");
			return;
		}

		mfc_debug(1, "No ctx is scheduled to be run.\n");
		return;
	}

	mfc_debug(1, "New context: %d\n", new_ctx);
	ctx = dev->ctx[new_ctx];
	mfc_debug(1, "Setting new context to %p\n", ctx);
	/* Got context to run in ctx */
	mfc_debug(1, "ctx->dst_queue_cnt=%d ctx->dpb_count=%d ctx->src_queue_cnt=%d\n",
		ctx->dst_queue_cnt, ctx->pb_count, ctx->src_queue_cnt);
	mfc_debug(1, "ctx->state=%d\n", ctx->state);
	/* Last frame has already been sent to MFC
	 * Now obtaining frames from MFC buffer */

	s5p_mfc_clock_on();
	s5p_mfc_clean_ctx_int_flags(ctx);

	ret = s5p_mfc_hw_call(dev->mfc_ops, run_ctx, dev, ctx);
	if (ret) {
		/* Free hardware lock */
		if (test_and_clear_bit(0, &dev->hw_lock) == 0)
			mfc_err("Failed to unlock hardware.\n");

		/* This is in deed imporant, as no operation has been
		 * scheduled, reduce the clock count as no one will
		 * ever do this, because no interrupt related to this try_run
		 * will ever come from hardware. */
		s5p_mfc_clock_off();
	}
}
