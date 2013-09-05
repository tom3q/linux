/*
 * Chips&Media CODA video encoder/decoder driver
 *
 * Copyright (C) 2013 Tomasz Figa <tomasz.figa@gmail.com>
 *
 * Heavily based on original Coda driver
 *
 * Copyright (C) 2012 Vista Silicon S.L.
 *    Javier Martin, <javier.martin@vista-silicon.com>
 *    Xavier Duret
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/of.h>
#include <linux/platform_data/coda.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "coda.h"
#include "coda-regs.h"

/*
 * Bitstream queue
 */
static inline int coda_get_bitstream_payload(struct coda_ctx *ctx)
{
	return kfifo_len(&ctx->bitstream_fifo);
}

static void coda_kfifo_sync_from_device(struct coda_ctx *ctx)
{
	struct __kfifo *kfifo = &ctx->bitstream_fifo.kfifo;
	struct coda_dev *dev = ctx->dev;
	u32 rd_ptr;

	rd_ptr = coda_read(dev, CODA_REG_BIT_RD_PTR(ctx->reg_idx));
	kfifo->out = (kfifo->in & ~kfifo->mask) |
		      (rd_ptr - ctx->bitstream.paddr);
	if (kfifo->out > kfifo->in)
		kfifo->out -= kfifo->mask + 1;
}

static void coda_kfifo_sync_to_device_full(struct coda_ctx *ctx)
{
	struct __kfifo *kfifo = &ctx->bitstream_fifo.kfifo;
	struct coda_dev *dev = ctx->dev;
	u32 rd_ptr, wr_ptr;

	rd_ptr = ctx->bitstream.paddr + (kfifo->out & kfifo->mask);
	coda_write(dev, rd_ptr, CODA_REG_BIT_RD_PTR(ctx->reg_idx));
	wr_ptr = ctx->bitstream.paddr + (kfifo->in & kfifo->mask);
	coda_write(dev, wr_ptr, CODA_REG_BIT_WR_PTR(ctx->reg_idx));
}

static void coda_kfifo_sync_to_device_write(struct coda_ctx *ctx)
{
	struct __kfifo *kfifo = &ctx->bitstream_fifo.kfifo;
	struct coda_dev *dev = ctx->dev;
	u32 wr_ptr;

	wr_ptr = ctx->bitstream.paddr + (kfifo->in & kfifo->mask);
	coda_write(dev, wr_ptr, CODA_REG_BIT_WR_PTR(ctx->reg_idx));
}

static int coda_bitstream_queue(struct coda_ctx *ctx,
				struct vb2_buffer *src_buf)
{
	u32 src_size = vb2_get_plane_payload(src_buf, 0);
	u32 n;

	n = kfifo_in(&ctx->bitstream_fifo,
			vb2_plane_vaddr(src_buf, 0), src_size);
	if (n < src_size)
		return -ENOSPC;

	wmb();

	ctx->qsequence++;

	return 0;
}

static bool coda_bitstream_try_queue(struct coda_ctx *ctx,
				     struct vb2_buffer *src_buf)
{
	int ret;

	if (coda_get_bitstream_payload(ctx) +
	    vb2_get_plane_payload(src_buf, 0) + 512 >= ctx->bitstream.size)
		return false;

	if (vb2_plane_vaddr(src_buf, 0) == NULL) {
		v4l2_err(&ctx->dev->v4l2_dev,
				"trying to queue empty buffer\n");
		return true;
	}

	ret = coda_bitstream_queue(ctx, src_buf);
	if (ret < 0) {
		v4l2_err(&ctx->dev->v4l2_dev, "bitstream buffer overflow\n");
		return false;
	}
	/* Sync read pointer to device */
	if (ctx == v4l2_m2m_get_curr_priv(ctx->dev->m2m_dev))
		coda_kfifo_sync_to_device_write(ctx);

	ctx->prescan_failed = false;

	return true;
}

static void coda_fill_bitstream(struct coda_ctx *ctx)
{
	struct vb2_buffer *src_buf;

	while (v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx) > 0) {
		src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);

		if (coda_bitstream_try_queue(ctx, src_buf)) {
			src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		} else {
			break;
		}
	}
}

/*
 * Queue operations
 */
static void coda_buf_queue(struct vb2_buffer *vb)
{
	struct coda_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct coda_dev *dev = ctx->dev;

	if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);

	/*
	 * In the decoder case, immediately try to copy the buffer into the
	 * bitstream ringbuffer and mark it as ready to be dequeued.
	 *
	 * For backwards compatiblity, queuing an empty buffer marks
	 * the stream end
	 */
	if (vb2_get_plane_payload(vb, 0) == 0)
		ctx->bit_stream_param |= BIT(dev->devtype->stream_end_bit);

	mutex_lock(&ctx->bitstream_mutex);
	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);
	coda_fill_bitstream(ctx);
	mutex_unlock(&ctx->bitstream_mutex);
}

static void coda_finish_decode(struct coda_ctx *ctx)
{
	struct coda_dev *dev = ctx->dev;
	struct coda_q_data *q_data_dst;
	struct vb2_buffer *dst_buf;
	int decoded_idx;
	int display_idx;
	int success;
	u32 val;

	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);

	/* Update kfifo out pointer from coda bitstream read pointer */
	coda_kfifo_sync_from_device(ctx);

	/*
	 * in stream-end mode, the read pointer can overshoot the write
	 * pointer by up to 512 bytes
	 */
	if (ctx->bit_stream_param & BIT(dev->devtype->stream_end_bit) &&
	    coda_get_bitstream_payload(ctx) >= CODA_MAX_FRAME_SIZE - 512)
		kfifo_init(&ctx->bitstream_fifo, ctx->bitstream.vaddr,
				ctx->bitstream.size);

	val = coda_read(dev, CODA_RET_DEC_PIC_SUCCESS);
	if (val != 1)
		v4l2_err(&dev->v4l2_dev, "DEC_PIC_SUCCESS = %d\n", val);

	success = val & 0x1;
	if (!success)
		v4l2_err(&dev->v4l2_dev, "decode failed\n");

	if (ctx->codec->fourcc == V4L2_PIX_FMT_H264) {
		if (val & (1 << 3))
			v4l2_err(&dev->v4l2_dev,
				"insufficient PS buffer space (%d bytes)\n",
				ctx->psbuf.size);
		if (val & (1 << 2))
			v4l2_err(&dev->v4l2_dev,
				"insufficient slice buffer space (%d bytes)\n",
				ctx->slicebuf.size);
	}

	q_data_dst = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	val = coda_read(dev, CODA_RET_DEC_PIC_TYPE);
	if ((val & 0x7) == 0) {
		dst_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_KEYFRAME;
		dst_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_PFRAME;
	} else {
		dst_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_PFRAME;
		dst_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_KEYFRAME;
	}

	val = coda_read(dev, CODA_RET_DEC_PIC_ERR_MB);
	if (val > 0)
		v4l2_err(&dev->v4l2_dev,
			 "errors in %d macroblocks\n", val);

	if (dev->devtype->product == CODA_7541) {
		val = coda_read(dev, CODA_RET_DEC_PIC_OPTION);
		if (val == 0) {
			/* not enough bitstream data */
			v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
				 "prescan failed: %d\n", val);
			ctx->prescan_failed = true;
			return;
		}
	}

	if (dev->devtype->has_frm_dis) {
		ctx->frm_dis_flg = coda_read(dev,
					CODA_REG_BIT_FRM_DIS_FLG(ctx->reg_idx));

		/*
		* The previous display frame was copied out by the rotator,
		* now it can be overwritten again
		*/
		if (ctx->display_idx >= 0 &&
		    ctx->display_idx < ctx->num_internal_frames) {
			ctx->frm_dis_flg &= ~(1 << ctx->display_idx);
			coda_write(dev, ctx->frm_dis_flg,
					CODA_REG_BIT_FRM_DIS_FLG(ctx->reg_idx));
		}
	}

	/*
	 * The index of the last decoded frame, not necessarily in
	 * display order, and the index of the next display frame.
	 * The latter could have been decoded in a previous run.
	 */
	decoded_idx = coda_read(dev, CODA_RET_DEC_PIC_CUR_IDX);
	display_idx = coda_read(dev, CODA_RET_DEC_PIC_FRAME_IDX);

	if (decoded_idx == -1) {
		/* no frame was decoded, but we might have a display frame */
		if (display_idx < 0 && ctx->display_idx < 0)
			ctx->prescan_failed = true;
	} else if (decoded_idx == -2) {
		/* no frame was decoded, we still return the remaining buffers */
	} else if (decoded_idx < 0 || decoded_idx >= ctx->num_internal_frames) {
		v4l2_err(&dev->v4l2_dev,
				"decoded frame index out of range: %d\n",
				decoded_idx);
	}

	if (display_idx == -1) {
		/*
		 * no more frames to be decoded, but there could still
		 * be rotator output to dequeue
		 */
		ctx->prescan_failed = true;
	} else if (display_idx == -3) {
		/*
		 * possibly prescan failure
		 * (in case of CODA MFC v1: frame was delayed)
		 */
	} else if (display_idx < 0 || display_idx >= ctx->num_internal_frames) {
		v4l2_err(&dev->v4l2_dev,
				"presentation frame index out of range: %d\n",
				display_idx);
	}

	/* If a frame was copied out, return it */
	if (ctx->display_idx >= 0 &&
	    ctx->display_idx < ctx->num_internal_frames) {
		int p;

		dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
		dst_buf->v4l2_buf.sequence = ctx->osequence++;

		for (p = 0; p < q_data_dst->num_planes; ++p)
			vb2_set_plane_payload(dst_buf, 0,
						q_data_dst->sizeimage[p]);

		v4l2_m2m_buf_done(dst_buf, success ? VB2_BUF_STATE_DONE :
						     VB2_BUF_STATE_ERROR);

		v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
			"job finished: decoding frame (%d) (%s)\n",
			dst_buf->v4l2_buf.sequence,
			(dst_buf->v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) ?
			"KEYFRAME" : "PFRAME");
	} else {
		v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
			"job finished: no frame decoded\n");
	}

	/* The rotator will copy the current display frame next time */
	ctx->display_idx = display_idx;
}

void coda_setup_iram(struct coda_ctx *ctx)
{
	struct coda_iram_info *iram_info = &ctx->iram_info;
	struct coda_q_data *q_data_dst;
	struct coda_dev *dev = ctx->dev;
	int ipacdc_size;
	int bitram_size;
	int dbk_size;
	int ovl_size;
	int mb_width;
	int mb_height;
	int size;

	memset(iram_info, 0, sizeof(*iram_info));
	size = dev->iram_size;

	q_data_dst = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	mb_width = DIV_ROUND_UP(q_data_dst->width, 16);
	mb_height = DIV_ROUND_UP(q_data_dst->height, 16);

	dbk_size = round_up(256 * mb_width, 1024);
	if (size >= dbk_size) {
		iram_info->axi_sram_use |= CODA7_USE_HOST_DBK_ENABLE;
		iram_info->buf_dbk_y_use = dev->iram_paddr;
		iram_info->buf_dbk_c_use = dev->iram_paddr +
						dbk_size / 2;
		size -= dbk_size;
	} else {
		goto out;
	}

	bitram_size = round_up(128 * mb_width, 1024);
	if (size >= bitram_size) {
		iram_info->axi_sram_use |= CODA7_USE_HOST_BIT_ENABLE;
		iram_info->buf_bit_use = iram_info->buf_dbk_c_use +
						dbk_size / 2;
		size -= bitram_size;
	} else {
		goto out;
	}

	ipacdc_size = round_up(128 * mb_width, 1024);
	if (size >= ipacdc_size) {
		iram_info->axi_sram_use |= CODA7_USE_HOST_IP_ENABLE;
		iram_info->buf_ip_ac_dc_use = iram_info->buf_bit_use +
						bitram_size;
		size -= ipacdc_size;
	} else {
		goto out;
	}

	ovl_size = round_up(80 * mb_width, 1024);

out:
	/* i.MX53 uses secondary AXI for IRAM access */
	if (iram_info->axi_sram_use & CODA7_USE_HOST_BIT_ENABLE)
		iram_info->axi_sram_use |= CODA7_USE_BIT_ENABLE;
	if (iram_info->axi_sram_use & CODA7_USE_HOST_IP_ENABLE)
		iram_info->axi_sram_use |= CODA7_USE_IP_ENABLE;
	if (iram_info->axi_sram_use & CODA7_USE_HOST_DBK_ENABLE)
		iram_info->axi_sram_use |= CODA7_USE_DBK_ENABLE;
	if (iram_info->axi_sram_use & CODA7_USE_HOST_OVL_ENABLE)
		iram_info->axi_sram_use |= CODA7_USE_OVL_ENABLE;
	if (iram_info->axi_sram_use & CODA7_USE_HOST_ME_ENABLE)
		iram_info->axi_sram_use |= CODA7_USE_ME_ENABLE;

	if (!(iram_info->axi_sram_use & CODA7_USE_HOST_IP_ENABLE))
		v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			 "IRAM smaller than needed\n");

	/* TODO - Enabling these causes picture errors on CODA7541 */
	/* fw 1.4.50 */
	iram_info->axi_sram_use &= ~(CODA7_USE_HOST_IP_ENABLE |
					CODA7_USE_IP_ENABLE);

}

static int coda_start_decoding(struct coda_ctx *ctx)
{
	struct coda_q_data *q_data_src, *q_data_dst;
	u32 bitstream_buf, bitstream_size;
	struct coda_dev *dev = ctx->dev;
	struct v4l2_format fmt;
	unsigned width, height;
	u32 val;
	int ret;

	/* Start decoding */
	q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	q_data_dst = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	bitstream_buf = ctx->bitstream.paddr;
	bitstream_size = ctx->bitstream.size;

	ret = coda_power_up(dev);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "failed to power up coda\n");
		return ret;
	}

	ret = coda_clk_enable(dev);
	if (ret)
		goto err_power_down;

	coda_write(dev, ctx->parabuf.paddr, CODA_REG_BIT_PARA_BUF_ADDR);

	/* Update coda bitstream read and write pointers from kfifo */
	coda_kfifo_sync_to_device_full(ctx);

	ctx->display_idx = -1;
	ctx->frm_dis_flg = 0;
	if (dev->devtype->has_frm_dis)
		coda_write(dev, 0, CODA_REG_BIT_FRM_DIS_FLG(ctx->reg_idx));

	ctx->bit_stream_param |= CODA_BIT_DEC_SEQ_INIT_ESCAPE;

	coda_write(dev, bitstream_buf, CODA_CMD_DEC_SEQ_BB_START);
	coda_write(dev, bitstream_size / 1024, CODA_CMD_DEC_SEQ_BB_SIZE);

	val = 0;
	if (dev->devtype->has_reorder)
		val |= CODA_REORDER_ENABLE;
	coda_write(dev, val, CODA_CMD_DEC_SEQ_OPTION);

	ctx->params.codec_mode = ctx->codec->mode;
	ctx->params.codec_mode_aux = 0;

	if (ctx->codec->fourcc == V4L2_PIX_FMT_H264 &&
	    dev->devtype->has_ps_buf) {
		coda_write(dev, ctx->psbuf.paddr,
				CODA_CMD_DEC_SEQ_PS_BB_START);
		coda_write(dev, (CODA7_PS_BUF_SIZE / 1024),
				CODA_CMD_DEC_SEQ_PS_BB_SIZE);
	}

	if (dev->devtype->product == CODA_MFC_V1)
		coda_write(dev, 0, CODAMFC_CMD_DEC_SEQ_START_BYTE);

	if (coda_command_sync(ctx, CODA_COMMAND_SEQ_INIT)) {
		v4l2_err(&dev->v4l2_dev, "CODA_COMMAND_SEQ_INIT timeout\n");
		ctx->bit_stream_param &= ~CODA_BIT_DEC_SEQ_INIT_ESCAPE;
		ret = -ETIMEDOUT;
		goto err_seq_end;
	}

	/* Update kfifo out pointer from coda bitstream read pointer */
	coda_kfifo_sync_from_device(ctx);

	ctx->bit_stream_param &= ~CODA_BIT_DEC_SEQ_INIT_ESCAPE;

	if (coda_read(dev, CODA_RET_DEC_SEQ_SUCCESS) == 0) {
		v4l2_err(&dev->v4l2_dev,
			"CODA_COMMAND_SEQ_INIT failed, error code = %d\n",
			coda_read(dev, CODA_RET_DEC_SEQ_ERR_REASON));
		ret = -EINVAL;
		goto err_seq_end;
	}

	val = coda_read(dev, CODA_RET_DEC_SEQ_SRC_SIZE);
	if (dev->devtype->product != CODA_7541) {
		width = (val >> CODADX6_PICWIDTH_OFFSET)
				& CODADX6_PICWIDTH_MASK;
		height = val & CODADX6_PICHEIGHT_MASK;
	} else {
		width = (val >> CODA7_PICWIDTH_OFFSET) & CODA7_PICWIDTH_MASK;
		height = val & CODA7_PICHEIGHT_MASK;
	}

	if (ctx->codec->fourcc == V4L2_PIX_FMT_H264) {
		width = round_up(width, 16);
		height = round_up(height, 16);
	} else {
		width = round_up(width, 8);
	}

	v4l2_dbg(1, coda_debug, &dev->v4l2_dev, "%s instance %d now: %dx%d\n",
			__func__, ctx->idx, width, height);

	ctx->num_internal_frames = 1 +
				coda_read(dev, CODA_RET_DEC_SEQ_FRAME_NEED);
	if (ctx->num_internal_frames > CODA_MAX_FRAMEBUFFERS) {
		v4l2_err(&dev->v4l2_dev,
			"not enough framebuffers to decode (%d < %d)\n",
			CODA_MAX_FRAMEBUFFERS, ctx->num_internal_frames);
		ret = -EINVAL;
		goto err_seq_end;
	}

	val = coda_read(dev, CODA_RET_DEC_SEQ_SRC_F_RATE);
	ctx->params.framerate_num = (val >> CODA_FRATE_RES_OFFSET)
					& CODA_FRATE_RES_MASK;
	ctx->params.framerate_denom = ((val >> CODA_FRATE_DIV_OFFSET)
					& CODA_FRATE_DIV_MASK) + 1;

	coda_clk_disable(dev);

	q_data_src->width = width;
	q_data_src->height = height;

	memset(&fmt, 0, sizeof(fmt));

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	coda_g_fmt(NULL, &ctx->fh, &fmt);
	if (ctx->params.rot_mode & CODA_ROT_90) {
		fmt.fmt.pix_mp.width = height;
		fmt.fmt.pix_mp.height = width;
	} else {
		fmt.fmt.pix_mp.width = width;
		fmt.fmt.pix_mp.height = height;
	}
	coda_try_fmt(&fmt);
	coda_s_fmt(ctx, &fmt, q_data_dst->fmt);

	return 0;

err_seq_end:
	if (coda_command_sync(ctx, CODA_COMMAND_SEQ_END))
		v4l2_err(&dev->v4l2_dev, "CODA_COMMAND_SEQ_END failed\n");
	coda_clk_disable(dev);
err_power_down:
	coda_power_down(dev);

	return ret;
}

static int coda_set_framebuffers(struct coda_ctx *ctx)
{
	struct coda_q_data *q_data_src;
	struct coda_dev *dev = ctx->dev;
	int ret;

	q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	ret = coda_alloc_framebuffers(ctx, q_data_src);
	if (ret)
		return ret;

	ret = coda_clk_enable(dev);
	if (ret)
		goto err_free_framebuffers;

	/* Tell the decoder how many frame buffers we allocated. */
	coda_write(dev, ctx->num_internal_frames, CODA_CMD_SET_FRAME_BUF_NUM);
	coda_write(dev, q_data_src->width, CODA_CMD_SET_FRAME_BUF_STRIDE);

	if (dev->devtype->product == CODA_7541) {
		/* Set secondary AXI IRAM */
		coda_setup_iram(ctx);

		coda_write(dev, ctx->iram_info.buf_bit_use,
				CODA7_CMD_SET_FRAME_AXI_BIT_ADDR);
		coda_write(dev, ctx->iram_info.buf_ip_ac_dc_use,
				CODA7_CMD_SET_FRAME_AXI_IPACDC_ADDR);
		coda_write(dev, ctx->iram_info.buf_dbk_y_use,
				CODA7_CMD_SET_FRAME_AXI_DBKY_ADDR);
		coda_write(dev, ctx->iram_info.buf_dbk_c_use,
				CODA7_CMD_SET_FRAME_AXI_DBKC_ADDR);
		coda_write(dev, ctx->iram_info.buf_ovl_use,
				CODA7_CMD_SET_FRAME_AXI_OVL_ADDR);
	}

	if (ctx->codec->fourcc == V4L2_PIX_FMT_H264 &&
	    dev->devtype->has_slice_buf) {
		coda_write(dev, ctx->slicebuf.paddr,
				CODA_CMD_SET_FRAME_SLICE_BB_START);
		coda_write(dev, ctx->slicebuf.size / 1024,
				CODA_CMD_SET_FRAME_SLICE_BB_SIZE);
	}

	if (dev->devtype->max_mb_x) {
		int max_mb_x = dev->devtype->max_mb_x;
		int max_mb_y = dev->devtype->max_mb_y;
		int max_mb_num = max_mb_x * max_mb_y;

		coda_write(dev, max_mb_num << 16 | max_mb_x << 8 | max_mb_y,
				CODA7_CMD_SET_FRAME_MAX_DEC_SIZE);
	}

	if (coda_command_sync(ctx, CODA_COMMAND_SET_FRAME_BUF)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "CODA_COMMAND_SET_FRAME_BUF timeout\n");
		ret = -ETIMEDOUT;
		goto err_clk_disable;
	}

	coda_clk_disable(dev);

	return 0;

err_clk_disable:
	coda_clk_disable(dev);
err_free_framebuffers:
	coda_free_framebuffers(ctx);

	return ret;
}

static int coda_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct coda_ctx *ctx = vb2_get_drv_priv(q);
	struct coda_dev *dev = ctx->dev;
	struct coda_q_data *q_data_src;
	int ret;

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (count < 1 || !ctx->streamon_out)
			return -EINVAL;

		mutex_lock(&dev->coda_mutex);
		ret = coda_set_framebuffers(ctx);
		mutex_unlock(&dev->coda_mutex);

		if (!ret)
			ctx->streamon_cap = 1;

		return ret;
	}

	if (coda_get_bitstream_payload(ctx) < 512)
		return -EINVAL;

	/* Allow device_run with no buffers queued and after streamoff */
	v4l2_m2m_set_src_buffered(ctx->m2m_ctx, true);

	/* Allocate per-instance buffers */
	q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	ret = coda_alloc_context_buffers(ctx, q_data_src);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->coda_mutex);
	ret = coda_start_decoding(ctx);
	mutex_unlock(&dev->coda_mutex);

	if (ret)
		goto err_free_ctx_bufs;

	ctx->streamon_out = 1;
	return 0;

err_free_ctx_bufs:
	coda_free_context_buffers(ctx);

	return ret;
}

static void coda_stop_decoding(struct coda_ctx *ctx)
{
	struct coda_dev *dev = ctx->dev;
	int ret;

	ret = coda_clk_enable(dev);
	if (ret)
		return;

	v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
		 "%s: sending CODA_COMMAND_SEQ_END\n", __func__);

	if (coda_command_sync(ctx, CODA_COMMAND_SEQ_END))
		v4l2_err(&dev->v4l2_dev,
			 "CODA_COMMAND_SEQ_END failed\n");

	coda_power_down(dev);

	coda_clk_disable(dev);
}

static int coda_stop_streaming(struct vb2_queue *q)
{
	struct coda_ctx *ctx = vb2_get_drv_priv(q);
	struct coda_dev *dev = ctx->dev;

	v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
			"%s: type=%d\n", __func__, q->type);

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ctx->streamon_out = 0;
		ctx->bit_stream_param |= BIT(dev->devtype->stream_end_bit);
		ctx->isequence = 0;
	} else {
		ctx->streamon_cap = 0;
		ctx->osequence = 0;
	}

	if (!ctx->streamon_out && !ctx->streamon_cap) {
		mutex_lock(&dev->coda_mutex);
		coda_stop_decoding(ctx);
		mutex_unlock(&dev->coda_mutex);

		kfifo_init(&ctx->bitstream_fifo,
			ctx->bitstream.vaddr, ctx->bitstream.size);
		ctx->runcounter = 0;

		coda_free_context_buffers(ctx);
		coda_free_framebuffers(ctx);
	}

	return 0;
}

static const struct vb2_ops coda_dec_vb2_ops = {
	.queue_setup = coda_queue_setup,
	.buf_prepare = coda_buf_prepare,
	.buf_queue = coda_buf_queue,
	.wait_prepare = coda_wait_prepare,
	.wait_finish = coda_wait_finish,
	.start_streaming = coda_start_streaming,
	.stop_streaming = coda_stop_streaming,
};

static void set_default_params(struct coda_ctx *ctx)
{
	int max_w;
	int max_h;

	ctx->codec = coda_find_codec(ctx, NULL);
	max_w = ctx->codec->max_w;
	max_h = ctx->codec->max_h;

	ctx->params.codec_mode = CODA_MODE_INVALID;
	ctx->colorspace = V4L2_COLORSPACE_REC709;
	ctx->params.framerate_num = 30;
	ctx->params.framerate_denom = 1;
	ctx->aborting = 0;

	/* Default formats for output and input queues */
	ctx->q_data[V4L2_M2M_SRC].fmt =
			coda_find_format(ctx->codec->fourcc);
	ctx->q_data[V4L2_M2M_SRC].sizeimage[0] = CODA_MAX_FRAME_SIZE / 2;
	ctx->q_data[V4L2_M2M_SRC].width = max_w;
	ctx->q_data[V4L2_M2M_SRC].height = max_h;
	ctx->q_data[V4L2_M2M_SRC].num_planes = 1;

	ctx->q_data[V4L2_M2M_DST].fmt =
			coda_find_format(V4L2_PIX_FMT_YUV420);
	ctx->q_data[V4L2_M2M_DST].sizeimage[0] = (max_w * max_h * 3) / 2;
	ctx->q_data[V4L2_M2M_DST].width = max_w;
	ctx->q_data[V4L2_M2M_DST].height = max_h;
	ctx->q_data[V4L2_M2M_DST].num_planes = 1;
}

/*
 * V4L2 control operations.
 */
static int coda_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct coda_ctx *ctx =
			container_of(ctrl->handler, struct coda_ctx, ctrls);

	v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
		 "s_ctrl: id = %d, val = %d\n", ctrl->id, ctrl->val);

	if (ctx->streamon_out)
		return -EBUSY;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			ctx->params.rot_mode |= CODA_MIR_HOR;
		else
			ctx->params.rot_mode &= ~CODA_MIR_HOR;
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			ctx->params.rot_mode |= CODA_MIR_VER;
		else
			ctx->params.rot_mode &= ~CODA_MIR_VER;
		break;
	case V4L2_CID_ROTATE:
		ctx->params.rot_mode &= ~CODA_ROT_MASK;
		switch (ctrl->val) {
		case 0:
			ctx->params.rot_mode |= CODA_ROT_0;
			break;
		case 90:
			ctx->params.rot_mode |= CODA_ROT_90;
			break;
		case 180:
			ctx->params.rot_mode |= CODA_ROT_180;
			break;
		case 270:
			ctx->params.rot_mode |= CODA_ROT_270;
			break;
		}
		break;
	default:
		v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			"Invalid control, id=%d, val=%d\n",
			ctrl->id, ctrl->val);
		return -EINVAL;
	}

	return 0;
}

static int coda_g_v_ctrl(struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ctrl->val = 1;
		break;
	}
	return 0;
}

static struct v4l2_ctrl_ops coda_ctrl_ops = {
	.g_volatile_ctrl = coda_g_v_ctrl,
	.s_ctrl = coda_s_ctrl,
};

static int coda_dec_ctrls_setup(struct coda_ctx *ctx)
{
	v4l2_ctrl_handler_init(&ctx->ctrls, 9);

	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
				V4L2_CID_ROTATE, 0, 270, 90, 0);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
				V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 1, 1, 1, 1);

	if (ctx->ctrls.error) {
		v4l2_err(&ctx->dev->v4l2_dev,
				"control initialization error (%d)",
				ctx->ctrls.error);
		return -EINVAL;
	}

	return v4l2_ctrl_handler_setup(&ctx->ctrls);
}

/*
 * V4L2 mem2mem operations.
 */
static int coda_prepare_decoding(struct coda_ctx *ctx)
{
	struct vb2_buffer *dst_buf;
	struct coda_dev *dev = ctx->dev;
	struct coda_q_data *q_data_dst;
	u32 stridey, height;
	u32 picture_y, picture_cb, picture_cr;

	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	q_data_dst = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	stridey = q_data_dst->width;
	height = q_data_dst->height;

	/* Try to copy source buffer contents into the bitstream ringbuffer */
	mutex_lock(&ctx->bitstream_mutex);
	coda_fill_bitstream(ctx);
	mutex_unlock(&ctx->bitstream_mutex);

	if (coda_get_bitstream_payload(ctx) < 512 &&
	    (!(ctx->bit_stream_param & BIT(dev->devtype->stream_end_bit)))) {
		v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
			 "bitstream payload: %d, skipping\n",
			 coda_get_bitstream_payload(ctx));
		return -EAGAIN;
	}

	/* Set rotator output */
	picture_y = vb2_dma_contig_plane_dma_addr(dst_buf, 0);
	if (coda_format_is_mplane(q_data_dst->fmt)) {
		if (coda_format_is_yvu(q_data_dst->fmt)) {
			/* Switch Cr and Cb for YVU420 format */
			picture_cr = vb2_dma_contig_plane_dma_addr(dst_buf, 1);
			picture_cb = vb2_dma_contig_plane_dma_addr(dst_buf, 2);
		} else {
			picture_cb = vb2_dma_contig_plane_dma_addr(dst_buf, 1);
			picture_cr = vb2_dma_contig_plane_dma_addr(dst_buf, 2);
		}
	} else {
		if (coda_format_is_yvu(q_data_dst->fmt)) {
			/* Switch Cr and Cb for YVU420 format */
			picture_cr = picture_y + stridey * height;
			picture_cb = picture_cr + stridey / 2 * height / 2;
		} else {
			picture_cb = picture_y + stridey * height;
			picture_cr = picture_cb + stridey / 2 * height / 2;
		}
	}

	coda_write(dev, picture_y, CODA_CMD_DEC_PIC_ROT_ADDR_Y);
	coda_write(dev, picture_cb, CODA_CMD_DEC_PIC_ROT_ADDR_CB);
	coda_write(dev, picture_cr, CODA_CMD_DEC_PIC_ROT_ADDR_CR);
	coda_write(dev, CODA_ROT_MIR_ENABLE | ctx->params.rot_mode,
			CODA_CMD_DEC_PIC_ROT_MODE);

	switch (dev->devtype->product) {
	case CODA_MFC_V1:
		coda_write(dev, stridey, CODAMFC_CMD_DEC_PIC_ROT_STRIDE);
		coda_write(dev, CODA_PRE_SCAN_EN, CODAMFC_CMD_DEC_PIC_OPTION);

		coda_write(dev, 0, CODAMFC_CMD_DEC_PIC_BB_START);
		coda_write(dev, 0, CODAMFC_CMD_DEC_PIC_START_BYTE);

		coda_write(dev, ctx->psbuf.paddr, CODAMFC_CMD_DEC_PIC_MV_ADDR);
		coda_write(dev, ctx->psbuf.paddr + CODAMFC_MV_BUF_SIZE,
				CODAMFC_CMD_DEC_PIC_MBTYPE_ADDR);
		break;
	case CODA_DX6:
		/* TBD */
	case CODA_7541:
		coda_write(dev, stridey, CODA_CMD_DEC_PIC_ROT_STRIDE);
		coda_write(dev, CODA_PRE_SCAN_EN, CODA_CMD_DEC_PIC_OPTION);

		coda_write(dev, 0, CODA_CMD_DEC_PIC_SKIP_NUM);

		coda_write(dev, 0, CODA_CMD_DEC_PIC_BB_START);
		coda_write(dev, 0, CODA_CMD_DEC_PIC_START_BYTE);
		break;
	}

	coda_kfifo_sync_to_device_full(ctx);

	return 0;
}

/*
 * V4L2 ioctl() operations.
 */
static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);
	struct coda_q_data *q_data_dst;

	q_data_dst = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	f->fmt.pix_mp.colorspace = ctx->colorspace;
	f->fmt.pix_mp.width = q_data_dst->width;
	f->fmt.pix_mp.height = q_data_dst->height;

	return coda_try_fmt(f);
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);
	const struct coda_fmt *fmt;
	struct coda_q_data *q_data_src;
	u32 sizeimage;

	q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

	f->fmt.pix_mp.width = q_data_src->width;
	f->fmt.pix_mp.height = q_data_src->height;

	fmt = coda_find_format(f->fmt.pix_mp.pixelformat);

	if (!f->fmt.pix_mp.colorspace)
		f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;

	sizeimage = min_t(u32, CODA_MAX_FRAME_SIZE / 2,
				f->fmt.pix_mp.plane_fmt[0].sizeimage);
	f->fmt.pix_mp.plane_fmt[0].sizeimage = round_down(sizeimage,
				CODA_MIN_BITSTREAM_CHUNK);

	return coda_try_fmt(f);
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);
	const struct coda_fmt *fmt;
	int ret;

	ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	fmt = coda_find_format(f->fmt.pix_mp.pixelformat);

	return coda_s_fmt(ctx, f, fmt);
}

static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);
	const struct coda_fmt *fmt;
	int ret;

	ret = vidioc_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	fmt = coda_find_format(f->fmt.pix_mp.pixelformat);

	ret = coda_s_fmt(ctx, f, fmt);
	if (ret)
		return ret;

	ctx->colorspace = f->fmt.pix_mp.colorspace;
	ctx->codec = coda_find_codec(ctx, fmt);

	return 0;
}

static int vidioc_decoder_cmd(struct file *file, void *fh,
			      struct v4l2_decoder_cmd *dc)
{
	struct coda_ctx *ctx = fh_to_ctx(fh);
	struct coda_dev *dev = ctx->dev;

	if (dc->cmd != V4L2_DEC_CMD_STOP)
		return -EINVAL;

	if ((dc->flags & V4L2_DEC_CMD_STOP_TO_BLACK) ||
	    (dc->flags & V4L2_DEC_CMD_STOP_IMMEDIATELY))
		return -EINVAL;

	if (dc->stop.pts != 0)
		return -EINVAL;

	/* Set the strem-end flag on this context */
	ctx->bit_stream_param |= BIT(dev->devtype->stream_end_bit);

	return 0;
}


static int coda_cropcap(struct file *file, void *fh, struct v4l2_cropcap *cr)
{
	struct coda_ctx *ctx = fh_to_ctx(fh);
	struct coda_q_data *q_data;

	q_data = get_q_data(ctx, cr->type);

	cr->bounds.left = 0;
	cr->bounds.top = 0;
	cr->bounds.width = q_data->width;
	cr->bounds.height = q_data->height;
	cr->defrect = cr->bounds;

	return 0;
}

static int coda_g_crop(struct file *file, void *fh, struct v4l2_crop *cr)
{
	struct coda_ctx *ctx = fh_to_ctx(fh);
	struct coda_q_data *q_data;

	q_data = get_q_data(ctx, cr->type);

	cr->c.left = 0;
	cr->c.top = 0;
	cr->c.width = q_data->width;
	cr->c.height = q_data->height;

	return 0;
}

static int coda_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct coda_ctx *ctx = fh_to_ctx(fh);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	a->parm.output.timeperframe.denominator = ctx->params.framerate_denom;
	a->parm.output.timeperframe.numerator = ctx->params.framerate_num;

	return 0;
}

static const struct v4l2_ioctl_ops coda_dec_ioctl_ops = {
	.vidioc_querycap = coda_querycap,

	.vidioc_enum_fmt_vid_cap_mplane = coda_enum_fmt_raw,
	.vidioc_g_fmt_vid_cap_mplane = coda_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out_mplane = coda_enum_fmt_bitstream,
	.vidioc_g_fmt_vid_out_mplane = coda_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt_vid_out,

	.vidioc_reqbufs = coda_reqbufs,
	.vidioc_querybuf = coda_querybuf,

	.vidioc_qbuf = coda_qbuf,
	.vidioc_expbuf = coda_expbuf,
	.vidioc_dqbuf = coda_dqbuf,
	.vidioc_create_bufs = coda_create_bufs,

	.vidioc_streamon = coda_streamon,
	.vidioc_streamoff = coda_streamoff,

	.vidioc_decoder_cmd = vidioc_decoder_cmd,

	.vidioc_subscribe_event = coda_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_g_crop = coda_g_crop,
	.vidioc_cropcap = coda_cropcap,

	.vidioc_g_parm = coda_g_parm,
};

static int coda_dec_open(struct coda_ctx *ctx)
{
	set_default_params(ctx);
	coda_dec_ctrls_setup(ctx);
	ctx->fh.ctrl_handler = &ctx->ctrls;

	return 0;
}

const struct coda_video_dev codec_dec_vdev = {
	.name = "decoder",
	.open = coda_dec_open,
	.prepare = coda_prepare_decoding,
	.finish = coda_finish_decode,
	.ioctl_ops = &coda_dec_ioctl_ops,
	.vb2_ops = &coda_dec_vb2_ops,
};
