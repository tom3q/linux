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

#include <linux/bitops.h>
#include "s5p_mfc_cmd.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_opr.h"
#include "s5p_mfc_opr_v5.h"
#include "s5p_mfc_opr_v6.h"
#include "s5p_mfc_pm.h"

static struct s5p_mfc_hw_ops *s5p_mfc_ops;

int s5p_mfc_hw_trylock(struct s5p_mfc_dev *dev)
{
	if (test_and_set_bit(0, &dev->hw_lock))
		return -EBUSY;
	return 0;
}

int __s5p_mfc_hw_unlock(struct s5p_mfc_dev *dev)
{
	return test_and_clear_bit(0, &dev->hw_lock);
}

void s5p_mfc_hw_unlock(struct s5p_mfc_dev *dev)
{
	WARN_ON(!__s5p_mfc_hw_unlock(dev));
}

bool s5p_mfc_hw_is_locked(struct s5p_mfc_dev *dev)
{
	return test_bit(0, &dev->hw_lock);
}

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

/* Can be called from hardware ops as a fallback for last frame processing. */
int s5p_mfc_run_dec_frame(struct s5p_mfc_ctx *ctx,
			  enum s5p_mfc_decode_arg last_frame)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *temp_vb;

	if (ctx->state == MFCINST_FINISHING) {
		last_frame = MFC_DEC_LAST_FRAME;
		s5p_mfc_hw_call(dev->mfc_ops, set_dec_stream_buffer, ctx,
				0, 0, 0);
		dev->curr_ctx = ctx->num;
		s5p_mfc_clean_ctx_int_flags(ctx);
		s5p_mfc_hw_call(dev->mfc_ops, decode_one_frame, ctx,
				last_frame);
		return 0;
	}

	/* Frames are being decoded */
	if (list_empty(&ctx->src_queue)) {
		mfc_debug(2, "No src buffers.\n");
		return -EAGAIN;
	}
	/* Get the next source buffer */
	temp_vb = list_entry(ctx->src_queue.next, struct s5p_mfc_buf, list);
	temp_vb->flags |= MFC_BUF_FLAG_USED;
	s5p_mfc_hw_call(dev->mfc_ops, set_dec_stream_buffer, ctx,
			vb2_dma_contig_plane_dma_addr(&temp_vb->b->vb2_buf, 0),
			ctx->consumed_stream,
			temp_vb->b->vb2_buf.planes[0].bytesused);

	dev->curr_ctx = ctx->num;
	if (temp_vb->b->vb2_buf.planes[0].bytesused == 0) {
		last_frame = MFC_DEC_LAST_FRAME;
		mfc_debug(2, "Setting ctx->state to FINISHING\n");
		ctx->state = MFCINST_FINISHING;
	}

	s5p_mfc_hw_call(dev->mfc_ops, decode_one_frame, ctx, last_frame);
	return 0;
}

static int s5p_mfc_run_enc_frame(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *dst_mb;
	struct s5p_mfc_buf *src_mb;
	unsigned long src_y_addr, src_c_addr, dst_addr;
	unsigned int dst_size;

	if (list_empty(&ctx->src_queue) && ctx->state != MFCINST_FINISHING) {
		mfc_debug(2, "no src buffers.\n");
		return -EAGAIN;
	}

	if (list_empty(&ctx->dst_queue)) {
		mfc_debug(2, "no dst buffers.\n");
		return -EAGAIN;
	}

	if (list_empty(&ctx->src_queue)) {
		/* send null frame */
		s5p_mfc_hw_call(dev->mfc_ops, set_enc_null_frame, ctx);
		src_mb = NULL;
	} else {
		src_mb = list_entry(ctx->src_queue.next, struct s5p_mfc_buf, list);
		src_mb->flags |= MFC_BUF_FLAG_USED;
		if (src_mb->b->vb2_buf.planes[0].bytesused == 0) {
			/* send null frame */
			s5p_mfc_hw_call(dev->mfc_ops, set_enc_null_frame, ctx);
			ctx->state = MFCINST_FINISHING;
		} else {
			src_y_addr = vb2_dma_contig_plane_dma_addr(
					&src_mb->b->vb2_buf, 0);
			src_c_addr = vb2_dma_contig_plane_dma_addr(
					&src_mb->b->vb2_buf, 1);

			mfc_debug(2, "enc src y addr: 0x%08lx\n", src_y_addr);
			mfc_debug(2, "enc src c addr: 0x%08lx\n", src_c_addr);

			s5p_mfc_hw_call(dev->mfc_ops, set_enc_frame_buffer, ctx,
					src_y_addr, src_c_addr);
			if (src_mb->flags & MFC_BUF_FLAG_EOS)
				ctx->state = MFCINST_FINISHING;
		}
	}

	dst_mb = list_entry(ctx->dst_queue.next, struct s5p_mfc_buf, list);
	dst_mb->flags |= MFC_BUF_FLAG_USED;
	dst_addr = vb2_dma_contig_plane_dma_addr(&dst_mb->b->vb2_buf, 0);
	dst_size = vb2_plane_size(&dst_mb->b->vb2_buf, 0);

	s5p_mfc_hw_call(dev->mfc_ops, set_enc_stream_buffer, ctx, dst_addr,
			dst_size);
	dev->curr_ctx = ctx->num;
	s5p_mfc_hw_call(dev->mfc_ops, encode_one_frame, ctx);

	return 0;
}

static void s5p_mfc_run_init_dec(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *temp_vb;

	/* Initializing decoding - parsing header */
	mfc_debug(2, "Preparing to init decoding.\n");
	temp_vb = list_entry(ctx->src_queue.next, struct s5p_mfc_buf, list);
	s5p_mfc_hw_call(dev->mfc_ops, set_dec_desc_buffer, ctx);
	mfc_debug(2, "Header size: %d\n", temp_vb->b->vb2_buf.planes[0].bytesused);
	s5p_mfc_hw_call(dev->mfc_ops, set_dec_stream_buffer, ctx,
			vb2_dma_contig_plane_dma_addr(&temp_vb->b->vb2_buf, 0),
			0, temp_vb->b->vb2_buf.planes[0].bytesused);
	dev->curr_ctx = ctx->num;
	s5p_mfc_hw_call(dev->mfc_ops, init_decode, ctx);
}

static void s5p_mfc_run_init_enc(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *dst_mb;
	unsigned long dst_addr;
	unsigned int dst_size;

	s5p_mfc_hw_call(dev->mfc_ops, set_enc_ref_buffer, ctx);
	dst_mb = list_entry(ctx->dst_queue.next, struct s5p_mfc_buf, list);
	dst_addr = vb2_dma_contig_plane_dma_addr(&dst_mb->b->vb2_buf, 0);
	dst_size = vb2_plane_size(&dst_mb->b->vb2_buf, 0);
	s5p_mfc_hw_call(dev->mfc_ops, set_enc_stream_buffer, ctx,
			dst_addr, dst_size);
	dev->curr_ctx = ctx->num;
	s5p_mfc_hw_call(dev->mfc_ops, init_encode, ctx);
}

static int s5p_mfc_run_init_dec_buffers(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *temp_vb;
	int ret;

	/*
	 * Header was parsed now starting processing
	 * First set the output frame buffers
	 */
	if (ctx->capture_state != QUEUE_BUFS_MMAPED) {
		mfc_err("It seems that not all destination buffers were mmaped\nMFC requires that all destination are mmaped before starting processing\n");
		return -EAGAIN;
	}

	if (list_empty(&ctx->src_queue)) {
		mfc_err("Header has been deallocated in the middle of initialization\n");
		return -EIO;
	}

	temp_vb = list_entry(ctx->src_queue.next, struct s5p_mfc_buf, list);
	mfc_debug(2, "Header size: %d\n",
			temp_vb->b->vb2_buf.planes[0].bytesused);
	s5p_mfc_hw_call(dev->mfc_ops, set_dec_stream_buffer, ctx,
			vb2_dma_contig_plane_dma_addr(&temp_vb->b->vb2_buf, 0),
			0, temp_vb->b->vb2_buf.planes[0].bytesused);

	dev->curr_ctx = ctx->num;

	ret = s5p_mfc_hw_call(dev->mfc_ops, set_dec_frame_buffer, ctx);
	if (ret) {
		mfc_err("Failed to alloc frame mem.\n");
		ctx->state = MFCINST_ERROR;
	}
	return ret;
}

static int s5p_mfc_run_ctx(struct s5p_mfc_dev *dev,
			      struct s5p_mfc_ctx *ctx)
{
	int ret = 0;

	if (ctx->type == MFCINST_DECODER) {
		switch (ctx->state) {
		case MFCINST_FINISHING:
			s5p_mfc_hw_call(dev->mfc_ops, run_dec_last_frames, ctx);
			break;
		case MFCINST_RUNNING:
			ret = s5p_mfc_run_dec_frame(ctx, MFC_DEC_FRAME);
			break;
		case MFCINST_INIT:
			ret = s5p_mfc_hw_call(dev->mfc_cmds, open_inst_cmd,
					ctx);
			break;
		case MFCINST_RETURN_INST:
			ret = s5p_mfc_hw_call(dev->mfc_cmds, close_inst_cmd,
					ctx);
			break;
		case MFCINST_GOT_INST:
			s5p_mfc_run_init_dec(ctx);
			break;
		case MFCINST_HEAD_PARSED:
			ret = s5p_mfc_run_init_dec_buffers(ctx);
			break;
		case MFCINST_FLUSH:
			s5p_mfc_hw_call(dev->mfc_ops, set_flush, ctx, ctx->dpb_flush_flag);
			break;
		case MFCINST_RES_CHANGE_INIT:
			s5p_mfc_hw_call(dev->mfc_ops, run_res_change, ctx);
			break;
		case MFCINST_RES_CHANGE_FLUSH:
			s5p_mfc_hw_call(dev->mfc_ops, run_dec_last_frames, ctx);
			break;
		case MFCINST_RES_CHANGE_END:
			mfc_debug(2, "Finished remaining frames after resolution change.\n");
			ctx->capture_state = QUEUE_FREE;
			mfc_debug(2, "Will re-init the codec`.\n");
			s5p_mfc_run_init_dec(ctx);
			break;
		default:
			ret = -EAGAIN;
		}
	} else if (ctx->type == MFCINST_ENCODER) {
		switch (ctx->state) {
		case MFCINST_FINISHING:
		case MFCINST_RUNNING:
			ret = s5p_mfc_run_enc_frame(ctx);
			break;
		case MFCINST_INIT:
			ret = s5p_mfc_hw_call(dev->mfc_cmds, open_inst_cmd,
					ctx);
			break;
		case MFCINST_RETURN_INST:
			ret = s5p_mfc_hw_call(dev->mfc_cmds, close_inst_cmd,
					ctx);
			break;
		case MFCINST_GOT_INST:
			s5p_mfc_run_init_enc(ctx);
			break;
		case MFCINST_HEAD_PRODUCED:
			if (dev->mfc_ops->run_init_enc_buffers)
				ret = dev->mfc_ops->run_init_enc_buffers(ctx);
			break;
		default:
			ret = -EAGAIN;
		}
	} else {
		mfc_err("invalid context type: %d\n", ctx->type);
		ret = -EAGAIN;
	}

	return ret;
}

static int s5p_mfc_get_new_ctx(struct s5p_mfc_dev *dev)
{
	unsigned long flags;
	int ctx;

	spin_lock_irqsave(&dev->condlock, flags);

	/* Start searching from context next to current. */
	ctx = dev->curr_ctx + 1;
	if (ctx < MFC_NUM_CONTEXTS)
		ctx = find_next_bit(&dev->ctx_work_bits, MFC_NUM_CONTEXTS, ctx);
	/* Search again from the beginning if not found. */
	if (ctx == MFC_NUM_CONTEXTS)
		ctx = find_first_bit(&dev->ctx_work_bits, MFC_NUM_CONTEXTS);

	spin_unlock_irqrestore(&dev->condlock, flags);

	return ctx == MFC_NUM_CONTEXTS ? -EAGAIN : ctx;
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
	if (s5p_mfc_hw_trylock(dev) < 0) {
		/* This is perfectly ok, the scheduled ctx should wait */
		mfc_debug(1, "Couldn't lock HW.\n");
		return;
	}

	/* Choose the context to run */
	new_ctx = s5p_mfc_get_new_ctx(dev);
	if (new_ctx < 0) {
		/* No contexts to run */
		s5p_mfc_hw_unlock(dev);
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

	schedule_delayed_work(&dev->watchdog_work,
			msecs_to_jiffies(MFC_WATCHDOG_TIMEOUT_MS));

	ret = s5p_mfc_run_ctx(dev, ctx);
	if (ret) {
		/* Cancel the watchdog. */
		cancel_delayed_work(&dev->watchdog_work);

		/* Free hardware lock */
		s5p_mfc_hw_unlock(dev);

		/* This is in deed imporant, as no operation has been
		 * scheduled, reduce the clock count as no one will
		 * ever do this, because no interrupt related to this try_run
		 * will ever come from hardware. */
		s5p_mfc_clock_off();
	}
}
