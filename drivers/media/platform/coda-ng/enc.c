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

static const u8 coda_filler_nal[14] = { 0x00, 0x00, 0x00, 0x01, 0x0c, 0xff,
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80 };
static const u8 coda_filler_size[8] = { 0, 7, 14, 13, 12, 11, 10, 9 };

/*
 * Queue operations
 */
static void coda_buf_queue(struct vb2_buffer *vb)
{
	struct coda_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct coda_q_data *q_data;

	q_data = get_q_data(ctx, vb->vb2_queue->type);
	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);
}

static int coda_encode_header(struct coda_ctx *ctx, struct vb2_buffer *buf,
			      int header_code, u8 *header, int *size)
{
	struct coda_dev *dev = ctx->dev;
	int ret;

	coda_write(dev, vb2_dma_contig_plane_dma_addr(buf, 0),
		   CODA_CMD_ENC_HEADER_BB_START);
	coda_write(dev, vb2_plane_size(buf, 0), CODA_CMD_ENC_HEADER_BB_SIZE);
	coda_write(dev, header_code, CODA_CMD_ENC_HEADER_CODE);

	ret = coda_command_sync(ctx, CODA_COMMAND_ENCODE_HEADER);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev,
				"CODA_COMMAND_ENCODE_HEADER timeout\n");
		return ret;
	}

	*size = coda_read(dev, CODA_REG_BIT_WR_PTR(ctx->reg_idx)) -
		coda_read(dev, CODA_CMD_ENC_HEADER_BB_START);
	memcpy(header, vb2_plane_vaddr(buf, 0), *size);

	return 0;
}

static void coda_finish_encode(struct coda_ctx *ctx)
{
	struct vb2_buffer *src_buf, *dst_buf;
	struct coda_dev *dev = ctx->dev;
	u32 wr_ptr, start_ptr;

	src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);

	/* Get results from the coda */
	coda_read(dev, CODA_RET_ENC_PIC_TYPE);
	start_ptr = coda_read(dev, CODA_CMD_ENC_PIC_BB_START);
	wr_ptr = coda_read(dev, CODA_REG_BIT_WR_PTR(ctx->reg_idx));

	/* Calculate bytesused field */
	if (dst_buf->v4l2_buf.sequence == 0) {
		vb2_set_plane_payload(dst_buf, 0, wr_ptr - start_ptr +
					ctx->vpu_header_size[0] +
					ctx->vpu_header_size[1] +
					ctx->vpu_header_size[2]);
	} else {
		vb2_set_plane_payload(dst_buf, 0, wr_ptr - start_ptr);
	}

	v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev, "frame size = %u\n",
		 wr_ptr - start_ptr);

	coda_read(dev, CODA_RET_ENC_PIC_SLICE_NUM);
	coda_read(dev, CODA_RET_ENC_PIC_FLAG);

	if (src_buf->v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) {
		dst_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_KEYFRAME;
		dst_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_PFRAME;
	} else {
		dst_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_PFRAME;
		dst_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_KEYFRAME;
	}

	dst_buf->v4l2_buf.timestamp = src_buf->v4l2_buf.timestamp;
	dst_buf->v4l2_buf.timecode = src_buf->v4l2_buf.timecode;

	v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

	ctx->gopcounter--;
	if (ctx->gopcounter < 0)
		ctx->gopcounter = ctx->params.gop_size - 1;

	v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
		"job finished: encoding frame (%d) (%s)\n",
		dst_buf->v4l2_buf.sequence,
		(dst_buf->v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) ?
		"KEYFRAME" : "PFRAME");
}

static int coda_h264_padding(int size, char *p)
{
	int nal_size;
	int diff;

	diff = size - (size & ~0x7);
	if (diff == 0)
		return 0;

	nal_size = coda_filler_size[diff];
	memcpy(p, coda_filler_nal, nal_size);

	/* Add rbsp stop bit and trailing at the end */
	*(p + nal_size - 1) = 0x80;

	return nal_size;
}

static void coda_setup_iram(struct coda_ctx *ctx)
{
	struct coda_iram_info *iram_info = &ctx->iram_info;
	struct coda_dev *dev = ctx->dev;
	struct coda_q_data *q_data_src;
	int ipacdc_size;
	int bitram_size;
	int dbk_size;
	int mb_width;
	int me_size;
	int size;

	memset(iram_info, 0, sizeof(*iram_info));
	size = dev->iram_size;

	q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	mb_width = DIV_ROUND_UP(q_data_src->width, 16);

	/* Prioritize in case IRAM is too small for everything */
	me_size = round_up(round_up(q_data_src->width, 16) * 36 + 2048,
				1024);
	iram_info->search_ram_size = me_size;
	if (size >= iram_info->search_ram_size) {
		if (dev->devtype->product == CODA_7541)
			iram_info->axi_sram_use |= CODA7_USE_HOST_ME_ENABLE;
		iram_info->search_ram_paddr = dev->iram_paddr;
		size -= iram_info->search_ram_size;
	} else {
		pr_err("IRAM is smaller than the search ram size\n");
		goto out;
	}

	/* Only H.264BP and H.263P3 are considered */
	dbk_size = round_up(128 * mb_width, 1024);
	if (size >= dbk_size) {
		iram_info->axi_sram_use |= CODA7_USE_HOST_DBK_ENABLE;
		iram_info->buf_dbk_y_use = dev->iram_paddr +
						iram_info->search_ram_size;
		iram_info->buf_dbk_c_use = iram_info->buf_dbk_y_use +
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
	}

	/* OVL and BTP disabled for encoder */

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
	/* fw 13.4.29 */
	iram_info->axi_sram_use &= ~(CODA7_USE_HOST_IP_ENABLE |
					CODA7_USE_HOST_DBK_ENABLE |
					CODA7_USE_IP_ENABLE |
					CODA7_USE_DBK_ENABLE);
}

static int coda_start_encoding(struct coda_ctx *ctx)
{
	struct v4l2_device *v4l2_dev = &ctx->dev->v4l2_dev;
	struct coda_q_data *q_data_src, *q_data_dst;
	u32 bitstream_buf, bitstream_size;
	struct coda_dev *dev = ctx->dev;
	struct vb2_buffer *buf;
	u32 dst_fourcc;
	u32 value;
	int ret;

	ret = coda_power_up(dev);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "failed to power up coda\n");
		return ret;
	}

	ret = coda_clk_enable(dev);
	if (ret)
		goto err_power_down;

	q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	q_data_dst = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	bitstream_buf = vb2_dma_contig_plane_dma_addr(buf, 0);
	bitstream_size = q_data_dst->sizeimage[0];
	dst_fourcc = q_data_dst->fmt->fourcc;

	coda_write(dev, ctx->parabuf.paddr, CODA_REG_BIT_PARA_BUF_ADDR);
	coda_write(dev, bitstream_buf, CODA_REG_BIT_RD_PTR(ctx->reg_idx));
	coda_write(dev, bitstream_buf, CODA_REG_BIT_WR_PTR(ctx->reg_idx));

	coda_write(dev, BIT(dev->devtype->stream_buf_dynalloc_bit) |
			BIT(dev->devtype->stream_pic_reset_bit),
			CODA_REG_BIT_STREAM_CTRL);

	if (dev->devtype->product == CODA_DX6)
		coda_write(dev, dev->iram_paddr,
				CODADX6_REG_BIT_SEARCH_RAM_BASE_ADDR);

	/* Could set rotation here if needed */
	switch (dev->devtype->product) {
	case CODA_DX6:
	case CODA_MFC_V1:
		value = (q_data_src->width & CODADX6_PICWIDTH_MASK)
				<< CODADX6_PICWIDTH_OFFSET;
		value |= (q_data_src->height & CODADX6_PICHEIGHT_MASK)
				<< CODA_PICHEIGHT_OFFSET;
		break;
	default:
		value = (q_data_src->width & CODA7_PICWIDTH_MASK)
				<< CODA7_PICWIDTH_OFFSET;
		value |= (q_data_src->height & CODA7_PICHEIGHT_MASK)
				<< CODA_PICHEIGHT_OFFSET;
	}

	coda_write(dev, value, CODA_CMD_ENC_SEQ_SRC_SIZE);
	value = (ctx->params.framerate_denom - 1) << CODA_FRATE_DIV_OFFSET;
	value |= ctx->params.framerate_num << CODA_FRATE_RES_OFFSET;
	coda_write(dev, value, CODA_CMD_ENC_SEQ_SRC_F_RATE);

	ctx->params.codec_mode = ctx->codec->mode;

	switch (dst_fourcc) {
	case V4L2_PIX_FMT_MPEG4:
		coda_write(dev, CODA_STD_MPEG4, CODA_CMD_ENC_SEQ_COD_STD);
		coda_write(dev, 0, CODA_CMD_ENC_SEQ_MP4_PARA);
		break;
	case V4L2_PIX_FMT_H264:
		coda_write(dev, CODA_STD_H264, CODA_CMD_ENC_SEQ_COD_STD);
		coda_write(dev, 0, CODA_CMD_ENC_SEQ_264_PARA);
		break;
	default:
		v4l2_err(v4l2_dev,
			 "dst format (0x%08x) invalid.\n", dst_fourcc);
		ret = -EINVAL;
		goto err_clk_disable;
	}

	switch (ctx->params.slice_mode) {
	case V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE:
		value = 0;
		break;
	case V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB:
		value  = (ctx->params.slice_max_mb & CODA_SLICING_SIZE_MASK)
				<< CODA_SLICING_SIZE_OFFSET;
		value |= (1 & CODA_SLICING_UNIT_MASK)
				<< CODA_SLICING_UNIT_OFFSET;
		value |=  1 & CODA_SLICING_MODE_MASK;
		break;
	case V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES:
		value  = (ctx->params.slice_max_bits & CODA_SLICING_SIZE_MASK)
				<< CODA_SLICING_SIZE_OFFSET;
		value |= (0 & CODA_SLICING_UNIT_MASK)
				<< CODA_SLICING_UNIT_OFFSET;
		value |=  1 & CODA_SLICING_MODE_MASK;
		break;
	}
	coda_write(dev, value, CODA_CMD_ENC_SEQ_SLICE_MODE);

	value = ctx->params.gop_size & CODA_GOP_SIZE_MASK;
	coda_write(dev, value, CODA_CMD_ENC_SEQ_GOP_SIZE);

	if (ctx->params.bitrate) {
		/* Rate control enabled */
		value = (ctx->params.bitrate & CODA_RATECONTROL_BITRATE_MASK)
				<< CODA_RATECONTROL_BITRATE_OFFSET;
		value |=  1 & CODA_RATECONTROL_ENABLE_MASK;
	} else {
		value = 0;
	}
	coda_write(dev, value, CODA_CMD_ENC_SEQ_RC_PARA);

	coda_write(dev, 0, CODA_CMD_ENC_SEQ_RC_BUF_SIZE);
	coda_write(dev, 0, CODA_CMD_ENC_SEQ_INTRA_REFRESH);
	coda_write(dev, bitstream_buf, CODA_CMD_ENC_SEQ_BB_START);
	coda_write(dev, bitstream_size / 1024, CODA_CMD_ENC_SEQ_BB_SIZE);

	value = 0;
	if (dev->devtype->has_gamma_ctl) {
		u32 gamma;

		/* set default gamma */
		gamma = (CODA_DEFAULT_GAMMA & CODA_GAMMA_MASK)
				<< CODA_GAMMA_OFFSET;
		coda_write(dev, gamma, CODA_CMD_ENC_SEQ_RC_GAMMA);

		if (gamma)
			value |= BIT(dev->devtype->option_gamma_bit);
	}
	coda_write(dev, value, CODA_CMD_ENC_SEQ_OPTION);

	coda_setup_iram(ctx);

	if (dst_fourcc == V4L2_PIX_FMT_H264) {
		value  = (FMO_SLICE_SAVE_BUF_SIZE << 7);
		value |= (0 & CODA_FMOPARAM_TYPE_MASK)
				<< CODA_FMOPARAM_TYPE_OFFSET;
		value |=  0 & CODA_FMOPARAM_SLICENUM_MASK;

		if (dev->devtype->product != CODA_7541) {
			coda_write(dev, value, CODADX6_CMD_ENC_SEQ_FMO);
		} else {
			coda_write(dev, ctx->iram_info.search_ram_paddr,
					CODA7_CMD_ENC_SEQ_SEARCH_BASE);
			coda_write(dev, ctx->iram_info.search_ram_size,
					CODA7_CMD_ENC_SEQ_SEARCH_SIZE);
		}
	}

	ret = coda_command_sync(ctx, CODA_COMMAND_SEQ_INIT);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "CODA_COMMAND_SEQ_INIT timeout\n");
		goto err_seq_end;
	}

	if (coda_read(dev, CODA_RET_ENC_SEQ_SUCCESS) == 0) {
		v4l2_err(v4l2_dev, "CODA_COMMAND_SEQ_INIT failed\n");
		ret = -EFAULT;
		goto err_seq_end;
	}

	ctx->num_internal_frames = 2;
	ret = coda_alloc_framebuffers(ctx, q_data_src);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "failed to allocate framebuffers\n");
		goto err_seq_end;
	}

	coda_write(dev, ctx->num_internal_frames, CODA_CMD_SET_FRAME_BUF_NUM);
	coda_write(dev, round_up(q_data_src->width, 8),
			CODA_CMD_SET_FRAME_BUF_STRIDE);

	if (dev->devtype->product == CODA_7541) {
		coda_write(dev, round_up(q_data_src->width, 8),
				CODA7_CMD_SET_FRAME_SOURCE_BUF_STRIDE);
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

	ret = coda_command_sync(ctx, CODA_COMMAND_SET_FRAME_BUF);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "CODA_COMMAND_SET_FRAME_BUF timeout\n");
		goto err_seq_end;
	}

	/* Save stream headers */
	buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	switch (dst_fourcc) {
	case V4L2_PIX_FMT_H264:
		/*
		 * Get SPS in the first frame and copy it to an
		 * intermediate buffer.
		 */
		ret = coda_encode_header(ctx, buf, CODA_HEADER_H264_SPS,
					 &ctx->vpu_header[0][0],
					 &ctx->vpu_header_size[0]);
		if (ret < 0)
			goto err_seq_end;

		/*
		 * Get PPS in the first frame and copy it to an
		 * intermediate buffer.
		 */
		ret = coda_encode_header(ctx, buf, CODA_HEADER_H264_PPS,
					 &ctx->vpu_header[1][0],
					 &ctx->vpu_header_size[1]);
		if (ret < 0)
			goto err_seq_end;

		/*
		 * Length of H.264 headers is variable and thus it might not
		 * be aligned for the coda to append the encoded frame. In
		 * this case a filler NAL must be added to header 2.
		 */
		ctx->vpu_header_size[2] = coda_h264_padding(
					(ctx->vpu_header_size[0] +
					 ctx->vpu_header_size[1]),
					 ctx->vpu_header[2]);
		break;
	case V4L2_PIX_FMT_MPEG4:
		/*
		 * Get VOS in the first frame and copy it to an
		 * intermediate buffer
		 */
		ret = coda_encode_header(ctx, buf, CODA_HEADER_MP4V_VOS,
					 &ctx->vpu_header[0][0],
					 &ctx->vpu_header_size[0]);
		if (ret < 0)
			goto err_seq_end;

		ret = coda_encode_header(ctx, buf, CODA_HEADER_MP4V_VIS,
					 &ctx->vpu_header[1][0],
					 &ctx->vpu_header_size[1]);
		if (ret < 0)
			goto err_seq_end;

		ret = coda_encode_header(ctx, buf, CODA_HEADER_MP4V_VOL,
					 &ctx->vpu_header[2][0],
					 &ctx->vpu_header_size[2]);
		if (ret < 0)
			goto err_seq_end;
		break;
	default:
		/* No more formats need to save headers at the moment */
		break;
	}

	return 0;

err_seq_end:
	if (coda_command_sync(ctx, CODA_COMMAND_SEQ_END))
		v4l2_err(&dev->v4l2_dev, "CODA_COMMAND_SEQ_END failed\n");
err_clk_disable:
	coda_clk_disable(dev);
err_power_down:
	coda_power_down(dev);

	return ret;
}

static int coda_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct coda_ctx *ctx = vb2_get_drv_priv(q);
	struct coda_dev *dev = ctx->dev;
	struct coda_q_data *q_data_src;
	int ret;

	q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (count < 1)
			return -EINVAL;

		ctx->streamon_out = 1;
	} else {
		if (count < 1)
			return -EINVAL;

		ctx->streamon_cap = 1;
	}

	/* Don't start encoding unless both queues are on */
	if (!(ctx->streamon_out & ctx->streamon_cap))
		return 0;

	ctx->gopcounter = ctx->params.gop_size - 1;

	/* Allocate per-instance buffers */
	ret = coda_alloc_context_buffers(ctx, q_data_src);
	if (ret < 0)
		return ret;

	mutex_lock(&dev->coda_mutex);
	ret = coda_start_encoding(ctx);
	mutex_unlock(&dev->coda_mutex);

	return ret;
}

static void coda_stop_encoding(struct coda_ctx *ctx)
{
	struct coda_dev *dev = ctx->dev;

	v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
		 "%s: sending CODA_COMMAND_SEQ_END\n", __func__);

	if (coda_command_sync(ctx, CODA_COMMAND_SEQ_END))
		v4l2_err(&dev->v4l2_dev,
			 "CODA_COMMAND_SEQ_END failed\n");

	coda_power_down(dev);
}

static int coda_stop_streaming(struct vb2_queue *q)
{
	struct coda_ctx *ctx = vb2_get_drv_priv(q);
	struct coda_dev *dev = ctx->dev;

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
		coda_stop_encoding(ctx);
		mutex_unlock(&dev->coda_mutex);

		kfifo_init(&ctx->bitstream_fifo,
			ctx->bitstream.vaddr, ctx->bitstream.size);
		ctx->runcounter = 0;
	}

	return 0;
}

static struct vb2_ops coda_enc_vb2_ops = {
	.queue_setup		= coda_queue_setup,
	.buf_prepare		= coda_buf_prepare,
	.buf_queue		= coda_buf_queue,
	.wait_prepare		= coda_wait_prepare,
	.wait_finish		= coda_wait_finish,
	.start_streaming	= coda_start_streaming,
	.stop_streaming		= coda_stop_streaming,
};

static void set_default_params(struct coda_ctx *ctx)
{
	int max_w;
	int max_h;

	ctx->codec = &ctx->dev->devtype->codecs[0];
	max_w = ctx->codec->max_w;
	max_h = ctx->codec->max_h;

	ctx->params.codec_mode = CODA_MODE_INVALID;
	ctx->colorspace = V4L2_COLORSPACE_REC709;
	ctx->params.framerate_num = 30;
	ctx->params.framerate_denom = 1;
	ctx->aborting = 0;

	/* Default formats for output and input queues */
	ctx->q_data[V4L2_M2M_SRC].fmt =
			coda_find_format(V4L2_PIX_FMT_YUV420);
	ctx->q_data[V4L2_M2M_SRC].sizeimage[0] = (max_w * max_h * 3) / 2;
	ctx->q_data[V4L2_M2M_DST].fmt =
			coda_find_format(ctx->codec->fourcc);
	ctx->q_data[V4L2_M2M_DST].sizeimage[0] = CODA_MAX_FRAME_SIZE;

	ctx->q_data[V4L2_M2M_SRC].width = max_w;
	ctx->q_data[V4L2_M2M_SRC].height = max_h;
	ctx->q_data[V4L2_M2M_SRC].num_planes = 1;
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
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctx->params.bitrate = ctrl->val / 1000;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctx->params.gop_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		ctx->params.h264_intra_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		ctx->params.h264_inter_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP:
		ctx->params.mpeg4_intra_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP:
		ctx->params.mpeg4_inter_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		ctx->params.slice_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		ctx->params.slice_max_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES:
		ctx->params.slice_max_bits = ctrl->val * 8;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		break;
	default:
		v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			"Invalid control, id=%d, val=%d\n",
			ctrl->id, ctrl->val);
		return -EINVAL;
	}

	return 0;
}

static struct v4l2_ctrl_ops coda_ctrl_ops = {
	.s_ctrl = coda_s_ctrl,
};

static int coda_enc_ctrls_setup(struct coda_ctx *ctx)
{
	v4l2_ctrl_handler_init(&ctx->ctrls, 9);

	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_ROTATE, 0, 270, 90, 0);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_BITRATE, 0, 32767000, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_GOP_SIZE, 1, 60, 1, 16);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, 1, 51, 1, 25);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP, 1, 51, 1, 25);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP, 1, 31, 1, 2);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP, 1, 31, 1, 2);
	v4l2_ctrl_new_std_menu(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
		V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES, 0x0,
		V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB, 1, 0x3fffffff, 1, 1);
	v4l2_ctrl_new_std(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES,
		1, 0x3fffffff, 1, 500);
	v4l2_ctrl_new_std_menu(&ctx->ctrls, &coda_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_HEADER_MODE,
		V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
		(1 << V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE),
		V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);

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
static int coda_prepare_encode(struct coda_ctx *ctx)
{
	struct coda_q_data *q_data_src, *q_data_dst;
	struct vb2_buffer *src_buf, *dst_buf;
	struct coda_dev *dev = ctx->dev;
	int force_ipicture;
	int quant_param = 0;
	u32 picture_y, picture_cb, picture_cr;
	u32 pic_stream_buffer_addr, pic_stream_buffer_size;
	u32 dst_fourcc;

	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	q_data_src = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	q_data_dst = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	dst_fourcc = q_data_dst->fmt->fourcc;

	src_buf->v4l2_buf.sequence = ctx->osequence;
	dst_buf->v4l2_buf.sequence = ctx->osequence;
	ctx->osequence++;

	/*
	 * Workaround coda firmware BUG that only marks the first
	 * frame as IDR. This is a problem for some decoders that can't
	 * recover when a frame is lost.
	 */
	if (src_buf->v4l2_buf.sequence % ctx->params.gop_size) {
		src_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_PFRAME;
		src_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_KEYFRAME;
	} else {
		src_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_KEYFRAME;
		src_buf->v4l2_buf.flags &= ~V4L2_BUF_FLAG_PFRAME;
	}

	/*
	 * Copy headers at the beginning of the first frame for H.264 only.
	 * In MPEG4 they are already copied by the coda.
	 */
	if (src_buf->v4l2_buf.sequence == 0) {
		pic_stream_buffer_addr =
			vb2_dma_contig_plane_dma_addr(dst_buf, 0) +
			ctx->vpu_header_size[0] +
			ctx->vpu_header_size[1] +
			ctx->vpu_header_size[2];
		pic_stream_buffer_size = CODA_MAX_FRAME_SIZE -
			ctx->vpu_header_size[0] -
			ctx->vpu_header_size[1] -
			ctx->vpu_header_size[2];
		memcpy(vb2_plane_vaddr(dst_buf, 0),
		       &ctx->vpu_header[0][0], ctx->vpu_header_size[0]);
		memcpy(vb2_plane_vaddr(dst_buf, 0) + ctx->vpu_header_size[0],
		       &ctx->vpu_header[1][0], ctx->vpu_header_size[1]);
		memcpy(vb2_plane_vaddr(dst_buf, 0) + ctx->vpu_header_size[0] +
			ctx->vpu_header_size[1], &ctx->vpu_header[2][0],
			ctx->vpu_header_size[2]);
	} else {
		pic_stream_buffer_addr =
			vb2_dma_contig_plane_dma_addr(dst_buf, 0);
		pic_stream_buffer_size = CODA_MAX_FRAME_SIZE;
	}

	if (src_buf->v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) {
		force_ipicture = 1;
		switch (dst_fourcc) {
		case V4L2_PIX_FMT_H264:
			quant_param = ctx->params.h264_intra_qp;
			break;
		case V4L2_PIX_FMT_MPEG4:
			quant_param = ctx->params.mpeg4_intra_qp;
			break;
		default:
			v4l2_warn(&ctx->dev->v4l2_dev,
				"cannot set intra qp, fmt not supported\n");
			break;
		}
	} else {
		force_ipicture = 0;
		switch (dst_fourcc) {
		case V4L2_PIX_FMT_H264:
			quant_param = ctx->params.h264_inter_qp;
			break;
		case V4L2_PIX_FMT_MPEG4:
			quant_param = ctx->params.mpeg4_inter_qp;
			break;
		default:
			v4l2_warn(&ctx->dev->v4l2_dev,
				"cannot set inter qp, fmt not supported\n");
			break;
		}
	}

	/* submit */
	coda_write(dev, CODA_ROT_MIR_ENABLE | ctx->params.rot_mode,
			CODA_CMD_ENC_PIC_ROT_MODE);
	coda_write(dev, quant_param, CODA_CMD_ENC_PIC_QS);

	picture_y = vb2_dma_contig_plane_dma_addr(src_buf, 0);
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
		if (coda_format_is_yvu(q_data_src->fmt)) {
			/* Switch Cb and Cr for YVU420 format */
			picture_cr = picture_y + q_data_src->width *
					q_data_src->height;
			picture_cb = picture_cr + q_data_src->width / 2 *
					q_data_src->height / 2;
		} else {
			picture_cb = picture_y + q_data_src->width *
					q_data_src->height;
			picture_cr = picture_cb + q_data_src->width / 2 *
					q_data_src->height / 2;
		}
	}

	coda_write(dev, picture_y, CODA_CMD_ENC_PIC_SRC_ADDR_Y);
	coda_write(dev, picture_cb, CODA_CMD_ENC_PIC_SRC_ADDR_CB);
	coda_write(dev, picture_cr, CODA_CMD_ENC_PIC_SRC_ADDR_CR);
	coda_write(dev, force_ipicture << 1 & 0x2,
		   CODA_CMD_ENC_PIC_OPTION);

	coda_write(dev, pic_stream_buffer_addr, CODA_CMD_ENC_PIC_BB_START);
	coda_write(dev, pic_stream_buffer_size / 1024,
		   CODA_CMD_ENC_PIC_BB_SIZE);

	if (dev->devtype->product == CODA_7541)
		coda_write(dev, ctx->iram_info.axi_sram_use,
				CODA7_REG_BIT_AXI_SRAM_USE);

	return 0;
}

/*
 * V4L2 ioctl() operations.
 */
static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);
	const struct coda_fmt *fmt;
	u32 sizeimage;

	fmt = coda_find_format(f->fmt.pix_mp.pixelformat);

	f->fmt.pix_mp.colorspace = ctx->colorspace;

	sizeimage = max_t(u32, CODA_MAX_FRAME_SIZE,
				f->fmt.pix_mp.plane_fmt[0].sizeimage);
	f->fmt.pix_mp.plane_fmt[0].sizeimage = round_up(sizeimage,
				CODA_MIN_BITSTREAM_CHUNK);

	return coda_try_fmt(f);
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	if (!f->fmt.pix_mp.colorspace)
		f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;

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

	ret = coda_s_fmt(ctx, f, fmt);
	if (ret)
		return ret;

	ctx->codec = coda_find_codec(ctx, fmt);
	return 0;
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
		ctx->colorspace = f->fmt.pix_mp.colorspace;

	return ret;
}

static int coda_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct coda_ctx *ctx = fh_to_ctx(fh);
	u64 num;
	u32 denom;

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	denom = a->parm.output.timeperframe.denominator;
	num = a->parm.output.timeperframe.numerator;

	if (denom > CODA_FRATE_DIV_MASK + 1) {
		num *= denom;
		do_div(num, CODA_FRATE_DIV_MASK + 1);
		denom = CODA_FRATE_DIV_MASK + 1;
	}

	ctx->params.framerate_denom = denom;
	ctx->params.framerate_num = num;

	return 0;
}

static int coda_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct coda_ctx *ctx = fh_to_ctx(fh);

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	a->parm.output.timeperframe.denominator = ctx->params.framerate_denom;
	a->parm.output.timeperframe.numerator = ctx->params.framerate_num;

	return 0;
}

static const struct v4l2_ioctl_ops coda_enc_ioctl_ops = {
	.vidioc_querycap = coda_querycap,

	.vidioc_enum_fmt_vid_cap_mplane = coda_enum_fmt_bitstream,
	.vidioc_g_fmt_vid_cap_mplane = coda_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out_mplane = coda_enum_fmt_raw,
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

	.vidioc_subscribe_event = coda_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_g_parm = coda_g_parm,
	.vidioc_s_parm = coda_s_parm,
};

static int coda_enc_open(struct coda_ctx *ctx)
{
	set_default_params(ctx);
	coda_enc_ctrls_setup(ctx);
	ctx->fh.ctrl_handler = &ctx->ctrls;

	return 0;
}

const struct coda_video_dev codec_enc_vdev = {
	.name = "encoder",
	.open = coda_enc_open,
	.prepare = coda_prepare_encode,
	.finish = coda_finish_encode,
	.ioctl_ops = &coda_enc_ioctl_ops,
	.vb2_ops = &coda_enc_vb2_ops,
};
