/*
 * linux/drivers/media/platform/s5p-mfc/s5p_mfc_dec.c
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 * Kamil Debski, <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include "s5p_mfc_common.h"
#include "s5p_mfc_ctrl.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_dec.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_opr.h"
#include "s5p_mfc_pm.h"

static struct s5p_mfc_fmt formats[] = {
	{
		.name		= "4:2:0 2 Planes 16x16 Tiles",
		.fourcc		= V4L2_PIX_FMT_NV12MT_16X16,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V6_BIT | MFC_V7_BIT,
	},
	{
		.name		= "4:2:0 2 Planes 64x32 Tiles",
		.fourcc		= V4L2_PIX_FMT_NV12MT,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V5_BIT,
	},
	{
		.name		= "4:2:0 2 Planes Y/CbCr",
		.fourcc		= V4L2_PIX_FMT_NV12M,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V6_BIT | MFC_V7_BIT | MFC_V8_BIT,
	},
	{
		.name		= "4:2:0 2 Planes Y/CrCb",
		.fourcc		= V4L2_PIX_FMT_NV21M,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V6_BIT | MFC_V7_BIT | MFC_V8_BIT,
	},
	{
		.name		= "H264 Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_H264,
		.codec_mode	= S5P_MFC_CODEC_H264_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5_BIT | MFC_V6_BIT | MFC_V7_BIT |
								MFC_V8_BIT,
	},
	{
		.name		= "H264/MVC Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_H264_MVC,
		.codec_mode	= S5P_MFC_CODEC_H264_MVC_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V6_BIT | MFC_V7_BIT | MFC_V8_BIT,
	},
	{
		.name		= "H263 Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_H263,
		.codec_mode	= S5P_MFC_CODEC_H263_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5_BIT | MFC_V6_BIT | MFC_V7_BIT |
								MFC_V8_BIT,
	},
	{
		.name		= "MPEG1 Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_MPEG1,
		.codec_mode	= S5P_MFC_CODEC_MPEG2_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5_BIT | MFC_V6_BIT | MFC_V7_BIT |
								MFC_V8_BIT,
	},
	{
		.name		= "MPEG2 Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_MPEG2,
		.codec_mode	= S5P_MFC_CODEC_MPEG2_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5_BIT | MFC_V6_BIT | MFC_V7_BIT |
								MFC_V8_BIT,
	},
	{
		.name		= "MPEG4 Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_MPEG4,
		.codec_mode	= S5P_MFC_CODEC_MPEG4_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5_BIT | MFC_V6_BIT | MFC_V7_BIT |
								MFC_V8_BIT,
	},
	{
		.name		= "XviD Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_XVID,
		.codec_mode	= S5P_MFC_CODEC_MPEG4_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5_BIT | MFC_V6_BIT | MFC_V7_BIT |
								MFC_V8_BIT,
	},
	{
		.name		= "VC1 Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_VC1_ANNEX_G,
		.codec_mode	= S5P_MFC_CODEC_VC1_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5_BIT | MFC_V6_BIT | MFC_V7_BIT |
								MFC_V8_BIT,
	},
	{
		.name		= "VC1 RCV Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_VC1_ANNEX_L,
		.codec_mode	= S5P_MFC_CODEC_VC1RCV_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5_BIT | MFC_V6_BIT | MFC_V7_BIT |
								MFC_V8_BIT,
	},
	{
		.name		= "VP8 Encoded Stream",
		.fourcc		= V4L2_PIX_FMT_VP8,
		.codec_mode	= S5P_MFC_CODEC_VP8_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V6_BIT | MFC_V7_BIT | MFC_V8_BIT,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

/* Find selected format description */
static struct s5p_mfc_fmt *find_format(struct v4l2_format *f, unsigned int t)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == f->fmt.pix_mp.pixelformat &&
		    formats[i].type == t)
			return &formats[i];
	}
	return NULL;
}

static struct mfc_control controls[] = {
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "H264 Display Delay",
		.minimum = 0,
		.maximum = 16383,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "H264 Display Delay Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Mpeg4 Loop Filter Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Slice Interface Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Minimum number of cap bufs",
		.minimum = 1,
		.maximum = 32,
		.step = 1,
		.default_value = 1,
		.is_volatile = 1,
	},
};

#define NUM_CTRLS ARRAY_SIZE(controls)

/* Check whether a context should be run on hardware */
static bool s5p_mfc_ctx_ready(struct s5p_mfc_ctx *ctx)
{
	bool ready;

	switch (ctx->state) {
	/* Context is not initialized. */
	case MFCINST_FREE:
	/* Context is stopped. */
	case MFCINST_FINISHED:
	case MFCINST_ERROR:
	case MFCINST_ABORT:
		ready = false;
		break;
	/* Context is to get a hardware instance. */
	case MFCINST_INIT:
	/* Context is to discard DPB. (MFCv6+) */
	case MFCINST_FLUSH:
	/* Context is to return its hardware instance. */
	case MFCINST_RETURN_INST:
		ready = true;
		break;
	/* Context is to parse header */
	case MFCINST_GOT_INST:
	case MFCINST_RES_CHANGE_END:
		ready = ctx->src_queue_cnt >= 1;
		break;
	/* Context is to set CAPTURE buffers */
	case MFCINST_HEAD_PARSED:
		ready = ctx->src_queue_cnt >= 1 &&
			ctx->capture_state == QUEUE_BUFS_MMAPED;
		break;
	/* Context is to decode a frame */
	case MFCINST_RUNNING:
		ready = ctx->src_queue_cnt >= 1 &&
			ctx->dst_queue_cnt >= ctx->pb_count;
		break;
	/* Context is to flush remaining decoded frames */
	case MFCINST_RES_CHANGE_INIT:
	case MFCINST_RES_CHANGE_FLUSH:
	case MFCINST_FINISHING:
		ready = ctx->dst_queue_cnt >= ctx->pb_count;
		break;
	default:
		mfc_err("Context %d in invalid state %d\n", ctx->num,
			ctx->state);
		return false;
	}

	mfc_debug(2, "ctx %d in state %d is %sready\n", ctx->num, ctx->state,
		  ready ? "" : "ready ");
	return ready;
}

/* Header parsing interrupt handling */
static void s5p_mfc_handle_seq_done(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	ctx->img_width = s5p_mfc_hw_call(dev->mfc_ops, get_img_width,
			dev);
	ctx->img_height = s5p_mfc_hw_call(dev->mfc_ops, get_img_height,
			dev);

	s5p_mfc_hw_call(dev->mfc_ops, dec_calc_dpb_size, ctx);

	ctx->pb_count = s5p_mfc_hw_call(dev->mfc_ops, get_dpb_count,
			dev);
	ctx->mv_count = s5p_mfc_hw_call(dev->mfc_ops, get_mv_count,
			dev);
	if (ctx->img_width == 0 || ctx->img_height == 0)
		ctx->state = MFCINST_ERROR;
	else
		ctx->state = MFCINST_HEAD_PARSED;

	if ((ctx->codec_mode == S5P_MFC_CODEC_H264_DEC ||
		ctx->codec_mode == S5P_MFC_CODEC_H264_MVC_DEC) &&
			!list_empty(&ctx->src_queue)) {
		struct s5p_mfc_buf *src_buf;
		src_buf = list_entry(ctx->src_queue.next,
				struct s5p_mfc_buf, list);
		if (s5p_mfc_hw_call(dev->mfc_ops, get_consumed_stream,
					dev) <
				src_buf->b->vb2_buf.planes[0].bytesused)
			ctx->head_processed = 0;
		else
			ctx->head_processed = 1;
	} else {
		ctx->head_processed = 1;
	}
}

static void s5p_mfc_handle_frame_all_extracted(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_buf *dst_buf;
	struct s5p_mfc_dev *dev = ctx->dev;

	ctx->state = MFCINST_FINISHED;
	ctx->sequence++;
	while (!list_empty(&ctx->dst_queue)) {
		dst_buf = list_entry(ctx->dst_queue.next,
				     struct s5p_mfc_buf, list);
		mfc_debug(2, "Cleaning up buffer: %d\n",
					  dst_buf->b->vb2_buf.index);
		vb2_set_plane_payload(&dst_buf->b->vb2_buf, 0, 0);
		vb2_set_plane_payload(&dst_buf->b->vb2_buf, 1, 0);
		list_del(&dst_buf->list);
		dst_buf->flags |= MFC_BUF_FLAG_EOS;
		ctx->dst_queue_cnt--;
		dst_buf->b->sequence = (ctx->sequence++);

		if (s5p_mfc_hw_call(dev->mfc_ops, get_pic_type_top, ctx) ==
			s5p_mfc_hw_call(dev->mfc_ops, get_pic_type_bot, ctx))
			dst_buf->b->field = V4L2_FIELD_NONE;
		else
			dst_buf->b->field = V4L2_FIELD_INTERLACED;
		dst_buf->b->flags |= V4L2_BUF_FLAG_LAST;

		ctx->dec_dst_flag &= ~(1 << dst_buf->b->vb2_buf.index);
		vb2_buffer_done(&dst_buf->b->vb2_buf, VB2_BUF_STATE_DONE);
	}
}

static void s5p_mfc_handle_frame_copy_time(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf  *dst_buf, *src_buf;
	size_t dec_y_addr;
	unsigned int frame_type;

	/* Make sure we actually have a new frame before continuing. */
	frame_type = s5p_mfc_hw_call(dev->mfc_ops, get_dec_frame_type, dev);
	if (frame_type == S5P_FIMV_DECODE_FRAME_SKIPPED)
		return;
	dec_y_addr = s5p_mfc_hw_call(dev->mfc_ops, get_dec_y_adr, dev);

	/* Copy timestamp / timecode from decoded src to dst and set
	   appropriate flags. */
	src_buf = list_entry(ctx->src_queue.next, struct s5p_mfc_buf, list);
	list_for_each_entry(dst_buf, &ctx->dst_queue, list) {
		if (vb2_dma_contig_plane_dma_addr(&dst_buf->b->vb2_buf, 0)
				== dec_y_addr) {
			dst_buf->b->timecode =
						src_buf->b->timecode;
			dst_buf->b->vb2_buf.timestamp =
						src_buf->b->vb2_buf.timestamp;
			dst_buf->b->flags &=
				~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
			dst_buf->b->flags |=
				src_buf->b->flags
				& V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
			switch (frame_type) {
			case S5P_FIMV_DECODE_FRAME_I_FRAME:
				dst_buf->b->flags |=
						V4L2_BUF_FLAG_KEYFRAME;
				break;
			case S5P_FIMV_DECODE_FRAME_P_FRAME:
				dst_buf->b->flags |=
						V4L2_BUF_FLAG_PFRAME;
				break;
			case S5P_FIMV_DECODE_FRAME_B_FRAME:
				dst_buf->b->flags |=
						V4L2_BUF_FLAG_BFRAME;
				break;
			default:
				/* Don't know how to handle
				   S5P_FIMV_DECODE_FRAME_OTHER_FRAME. */
				mfc_debug(2, "Unexpected frame type: %d\n",
						frame_type);
			}
			break;
		}
	}
}

static void s5p_mfc_handle_frame_new(struct s5p_mfc_ctx *ctx, unsigned int err)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf  *dst_buf;
	size_t dspl_y_addr;
	unsigned int frame_type;

	dspl_y_addr = s5p_mfc_hw_call(dev->mfc_ops, get_dspl_y_adr, dev);
	if (IS_MFCV6_PLUS(dev))
		frame_type = s5p_mfc_hw_call(dev->mfc_ops,
			get_disp_frame_type, ctx);
	else
		frame_type = s5p_mfc_hw_call(dev->mfc_ops,
			get_dec_frame_type, dev);

	/* If frame is same as previous then skip and do not dequeue */
	if (frame_type == S5P_FIMV_DECODE_FRAME_SKIPPED) {
		if (!ctx->after_packed_pb)
			ctx->sequence++;
		ctx->after_packed_pb = 0;
		return;
	}
	ctx->sequence++;
	/* The MFC returns address of the buffer, now we have to
	 * check which videobuf does it correspond to */
	list_for_each_entry(dst_buf, &ctx->dst_queue, list) {
		/* Check if this is the buffer we're looking for */
		if (vb2_dma_contig_plane_dma_addr(&dst_buf->b->vb2_buf, 0)
				== dspl_y_addr) {
			list_del(&dst_buf->list);
			ctx->dst_queue_cnt--;
			dst_buf->b->sequence = ctx->sequence;
			if (s5p_mfc_hw_call(dev->mfc_ops,
					get_pic_type_top, ctx) ==
				s5p_mfc_hw_call(dev->mfc_ops,
					get_pic_type_bot, ctx))
				dst_buf->b->field = V4L2_FIELD_NONE;
			else
				dst_buf->b->field =
							V4L2_FIELD_INTERLACED;
			vb2_set_plane_payload(&dst_buf->b->vb2_buf, 0,
						ctx->luma_size);
			vb2_set_plane_payload(&dst_buf->b->vb2_buf, 1,
						ctx->chroma_size);
			clear_bit(dst_buf->b->vb2_buf.index,
							&ctx->dec_dst_flag);

			vb2_buffer_done(&dst_buf->b->vb2_buf, err ?
				VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);

			break;
		}
	}
}

/* Handle frame decoding interrupt */
static void s5p_mfc_handle_frame(struct s5p_mfc_ctx *ctx, unsigned int err)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned int dst_frame_status;
	unsigned int dec_frame_status;
	struct s5p_mfc_buf *src_buf;
	unsigned int res_change;

	dst_frame_status = s5p_mfc_hw_call(dev->mfc_ops, get_dspl_status, dev)
				& S5P_FIMV_DEC_STATUS_DECODING_STATUS_MASK;
	dec_frame_status = s5p_mfc_hw_call(dev->mfc_ops, get_dec_status, dev)
				& S5P_FIMV_DEC_STATUS_DECODING_STATUS_MASK;
	res_change = (s5p_mfc_hw_call(dev->mfc_ops, get_dspl_status, dev)
				& S5P_FIMV_DEC_STATUS_RESOLUTION_MASK)
				>> S5P_FIMV_DEC_STATUS_RESOLUTION_SHIFT;
	mfc_debug(2, "Frame Status: %x\n", dst_frame_status);
	if (ctx->state == MFCINST_RES_CHANGE_INIT)
		ctx->state = MFCINST_RES_CHANGE_FLUSH;
	if (res_change == S5P_FIMV_RES_INCREASE ||
		res_change == S5P_FIMV_RES_DECREASE) {
		ctx->state = MFCINST_RES_CHANGE_INIT;
		return;
	}
	if (ctx->dpb_flush_flag)
		ctx->dpb_flush_flag = 0;

	/* All frames remaining in the buffer have been extracted  */
	if (dst_frame_status == S5P_FIMV_DEC_STATUS_DECODING_EMPTY) {
		if (ctx->state == MFCINST_RES_CHANGE_FLUSH) {
			static const struct v4l2_event ev_src_ch = {
				.type = V4L2_EVENT_SOURCE_CHANGE,
				.u.src_change.changes =
					V4L2_EVENT_SRC_CH_RESOLUTION,
			};

			s5p_mfc_handle_frame_all_extracted(ctx);
			ctx->state = MFCINST_RES_CHANGE_END;
			v4l2_event_queue_fh(&ctx->fh, &ev_src_ch);

			return;
		} else {
			s5p_mfc_handle_frame_all_extracted(ctx);
		}
	}

	if (dec_frame_status == S5P_FIMV_DEC_STATUS_DECODING_DISPLAY)
		s5p_mfc_handle_frame_copy_time(ctx);

	/* A frame has been decoded and is in the buffer  */
	if (dst_frame_status == S5P_FIMV_DEC_STATUS_DISPLAY_ONLY ||
	    dst_frame_status == S5P_FIMV_DEC_STATUS_DECODING_DISPLAY) {
		s5p_mfc_handle_frame_new(ctx, err);
	} else {
		mfc_debug(2, "No frame decode\n");
	}
	/* Mark source buffer as complete */
	if (dst_frame_status != S5P_FIMV_DEC_STATUS_DISPLAY_ONLY
		&& !list_empty(&ctx->src_queue)) {
		src_buf = list_entry(ctx->src_queue.next, struct s5p_mfc_buf,
								list);
		ctx->consumed_stream += s5p_mfc_hw_call(dev->mfc_ops,
						get_consumed_stream, dev);
		if (ctx->codec_mode != S5P_MFC_CODEC_H264_DEC &&
			ctx->codec_mode != S5P_MFC_CODEC_VP8_DEC &&
			ctx->consumed_stream + STUFF_BYTE <
			src_buf->b->vb2_buf.planes[0].bytesused) {
			/* Run MFC again on the same buffer */
			mfc_debug(2, "Running again the same buffer\n");
			ctx->after_packed_pb = 1;
		} else {
			mfc_debug(2, "MFC needs next buffer\n");
			ctx->consumed_stream = 0;
			if (src_buf->flags & MFC_BUF_FLAG_EOS)
				ctx->state = MFCINST_FINISHING;
			list_del(&src_buf->list);
			ctx->src_queue_cnt--;
			if (s5p_mfc_hw_call(dev->mfc_ops, err_dec, err) > 0)
				vb2_buffer_done(&src_buf->b->vb2_buf,
						VB2_BUF_STATE_ERROR);
			else
				vb2_buffer_done(&src_buf->b->vb2_buf,
						VB2_BUF_STATE_DONE);
		}
	}
}

static const struct s5p_mfc_codec_ops decoder_codec_ops = {
	.ctx_ready		= s5p_mfc_ctx_ready,
	.post_seq_start		= s5p_mfc_handle_seq_done,
	.post_frame_start	= s5p_mfc_handle_frame,
};

/* Query capabilities of the device */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);

	strncpy(cap->driver, S5P_MFC_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, dev->vfd_dec->name, sizeof(cap->card) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(&dev->plat_dev->dev));
	/*
	 * This is only a mem-to-mem video device. The capture and output
	 * device capability flags are left only for backward compatibility
	 * and are scheduled for removal.
	 */
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

/* Enumerate format */
static int vidioc_enum_fmt(struct file *file, struct v4l2_fmtdesc *f,
							bool out)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_fmt *fmt;
	int i, j = 0;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (out && formats[i].type != MFC_FMT_DEC)
			continue;
		else if (!out && formats[i].type != MFC_FMT_RAW)
			continue;
		else if ((dev->variant->version_bit & formats[i].versions) == 0)
			continue;

		if (j == f->index)
			break;
		++j;
	}
	if (i == ARRAY_SIZE(formats))
		return -EINVAL;
	fmt = &formats[i];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vidioc_enum_fmt_vid_cap_mplane(struct file *file, void *pirv,
							struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(file, f, false);
}

static int vidioc_enum_fmt_vid_out_mplane(struct file *file, void *priv,
							struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(file, f, true);
}

/* Get format */
static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);
	struct v4l2_pix_format_mplane *pix_mp;

	mfc_debug_enter();
	pix_mp = &f->fmt.pix_mp;
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    (ctx->state == MFCINST_GOT_INST || ctx->state ==
						MFCINST_RES_CHANGE_END)) {
		/* If the MFC is parsing the header,
		 * so wait until it is finished */
		int ret = s5p_mfc_wait_for_done_ctx(ctx);
		if (ret) {
			mfc_err("Waiting for context %d timed out\n",
				ctx->num);
			return ret;
		}
	}
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    ctx->state >= MFCINST_HEAD_PARSED &&
	    ctx->state < MFCINST_ABORT) {
		/* This is run on CAPTURE (decode output) */
		/* Width and height are set to the dimensions
		   of the movie, the buffer is bigger and
		   further processing stages should crop to this
		   rectangle. */
		pix_mp->width = ctx->buf_width;
		pix_mp->height = ctx->buf_height;
		pix_mp->field = V4L2_FIELD_NONE;
		pix_mp->num_planes = 2;
		/* Set pixelformat to the format in which MFC
		   outputs the decoded frame */
		pix_mp->pixelformat = ctx->dst_fmt->fourcc;
		pix_mp->plane_fmt[0].bytesperline = ctx->buf_width;
		pix_mp->plane_fmt[0].sizeimage = ctx->luma_size;
		pix_mp->plane_fmt[1].bytesperline = ctx->buf_width;
		pix_mp->plane_fmt[1].sizeimage = ctx->chroma_size;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* This is run on OUTPUT
		   The buffer contains compressed image
		   so width and height have no meaning */
		pix_mp->width = 0;
		pix_mp->height = 0;
		pix_mp->field = V4L2_FIELD_NONE;
		pix_mp->plane_fmt[0].bytesperline = ctx->dec_src_buf_size;
		pix_mp->plane_fmt[0].sizeimage = ctx->dec_src_buf_size;
		pix_mp->pixelformat = ctx->src_fmt->fourcc;
		pix_mp->num_planes = ctx->src_fmt->num_planes;
	} else {
		mfc_err("Format could not be read\n");
		mfc_debug(2, "%s-- with error\n", __func__);
		return -EINVAL;
	}
	mfc_debug_leave();
	return 0;
}

/* Try format */
static int vidioc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_fmt *fmt;

	mfc_debug(2, "Type is %d\n", f->type);
	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fmt = find_format(f, MFC_FMT_DEC);
		if (!fmt) {
			mfc_err("Unsupported format for source.\n");
			return -EINVAL;
		}
		if (fmt->codec_mode == S5P_FIMV_CODEC_NONE) {
			mfc_err("Unknown codec\n");
			return -EINVAL;
		}
		if ((dev->variant->version_bit & fmt->versions) == 0) {
			mfc_err("Unsupported format by this MFC version.\n");
			return -EINVAL;
		}
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt = find_format(f, MFC_FMT_RAW);
		if (!fmt) {
			mfc_err("Unsupported format for destination.\n");
			return -EINVAL;
		}
		if ((dev->variant->version_bit & fmt->versions) == 0) {
			mfc_err("Unsupported format by this MFC version.\n");
			return -EINVAL;
		}
	}

	return 0;
}

/* Set format */
static int vidioc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);
	int ret = 0;
	struct v4l2_pix_format_mplane *pix_mp;
	struct s5p_mfc_buf_size *buf_size = dev->variant->buf_size;

	mfc_debug_enter();
	ret = vidioc_try_fmt(file, priv, f);
	pix_mp = &f->fmt.pix_mp;
	if (ret)
		return ret;
	if (vb2_is_streaming(&ctx->vq_src) || vb2_is_streaming(&ctx->vq_dst)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		ret = -EBUSY;
		goto out;
	}
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* dst_fmt is validated by call to vidioc_try_fmt */
		ctx->dst_fmt = find_format(f, MFC_FMT_RAW);
		ret = 0;
		goto out;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* src_fmt is validated by call to vidioc_try_fmt */
		ctx->src_fmt = find_format(f, MFC_FMT_DEC);
		ctx->codec_mode = ctx->src_fmt->codec_mode;
		mfc_debug(2, "The codec number is: %d\n", ctx->codec_mode);
		pix_mp->height = 0;
		pix_mp->width = 0;
		if (pix_mp->plane_fmt[0].sizeimage == 0)
			pix_mp->plane_fmt[0].sizeimage = ctx->dec_src_buf_size =
								DEF_CPB_SIZE;
		else if (pix_mp->plane_fmt[0].sizeimage > buf_size->cpb)
			ctx->dec_src_buf_size = buf_size->cpb;
		else
			ctx->dec_src_buf_size = pix_mp->plane_fmt[0].sizeimage;
		pix_mp->plane_fmt[0].bytesperline = 0;
		ctx->state = MFCINST_INIT;
		ret = 0;
		goto out;
	} else {
		mfc_err("Wrong type error for S_FMT : %d", f->type);
		ret = -EINVAL;
		goto out;
	}

out:
	mfc_debug_leave();
	return ret;
}

static int reqbufs_output(struct s5p_mfc_dev *dev, struct s5p_mfc_ctx *ctx,
				struct v4l2_requestbuffers *reqbufs)
{
	int ret = 0;

	s5p_mfc_clock_on();

	if (reqbufs->count == 0) {
		mfc_debug(2, "Freeing buffers\n");
		ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
		if (ret)
			goto out;
		ctx->src_bufs_cnt = 0;
		ctx->output_state = QUEUE_FREE;
	} else if (ctx->output_state == QUEUE_FREE) {
		/* Can only request buffers when we have a valid format set. */
		WARN_ON(ctx->src_bufs_cnt != 0);
		if (ctx->state != MFCINST_INIT) {
			mfc_err("Reqbufs called in an invalid state\n");
			ret = -EINVAL;
			goto out;
		}

		mfc_debug(2, "Allocating %d buffers for OUTPUT queue\n",
				reqbufs->count);
		ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
		if (ret)
			goto out;

		ret = s5p_mfc_open_mfc_inst(dev, ctx);
		if (ret) {
			reqbufs->count = 0;
			vb2_reqbufs(&ctx->vq_src, reqbufs);
			goto out;
		}

		ctx->output_state = QUEUE_BUFS_REQUESTED;
	} else {
		mfc_err("Buffers have already been requested\n");
		ret = -EINVAL;
	}
out:
	s5p_mfc_clock_off();
	if (ret)
		mfc_err("Failed allocating buffers for OUTPUT queue\n");
	return ret;
}

static int reqbufs_capture(struct s5p_mfc_dev *dev, struct s5p_mfc_ctx *ctx,
				struct v4l2_requestbuffers *reqbufs)
{
	int ret = 0;

	s5p_mfc_clock_on();

	if (reqbufs->count == 0) {
		mfc_debug(2, "Freeing buffers\n");
		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret)
			goto out;
		s5p_mfc_hw_call(dev->mfc_ops, release_codec_buffers, ctx);
		ctx->dst_bufs_cnt = 0;
	} else if (ctx->capture_state == QUEUE_FREE) {
		WARN_ON(ctx->dst_bufs_cnt != 0);
		mfc_debug(2, "Allocating %d buffers for CAPTURE queue\n",
				reqbufs->count);
		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret)
			goto out;

		ctx->capture_state = QUEUE_BUFS_REQUESTED;
		ctx->total_dpb_count = reqbufs->count;

		ret = s5p_mfc_hw_call(dev->mfc_ops, alloc_codec_buffers, ctx);
		if (ret) {
			mfc_err("Failed to allocate decoding buffers\n");
			reqbufs->count = 0;
			vb2_reqbufs(&ctx->vq_dst, reqbufs);
			ret = -ENOMEM;
			ctx->capture_state = QUEUE_FREE;
			goto out;
		}

		WARN_ON(ctx->dst_bufs_cnt != ctx->total_dpb_count);
		ctx->capture_state = QUEUE_BUFS_MMAPED;

		s5p_mfc_try_ctx(ctx);
		ret = s5p_mfc_wait_for_done_ctx(ctx);
		if (ret)
			mfc_err("Waiting for context %d timed out\n",
				ctx->num);
	} else {
		mfc_err("Buffers have already been requested\n");
		ret = -EINVAL;
	}
out:
	s5p_mfc_clock_off();
	if (ret)
		mfc_err("Failed allocating buffers for CAPTURE queue\n");
	return ret;
}

/* Request buffers */
static int vidioc_reqbufs(struct file *file, void *priv,
					  struct v4l2_requestbuffers *reqbufs)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);

	if (reqbufs->memory != V4L2_MEMORY_MMAP) {
		mfc_debug(2, "Only V4L2_MEMORY_MMAP is supported\n");
		return -EINVAL;
	}

	if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		return reqbufs_output(dev, ctx, reqbufs);
	} else if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		return reqbufs_capture(dev, ctx, reqbufs);
	} else {
		mfc_err("Invalid type requested\n");
		return -EINVAL;
	}
}

/* Query buffer */
static int vidioc_querybuf(struct file *file, void *priv,
						   struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);
	int ret;
	int i;

	if (buf->memory != V4L2_MEMORY_MMAP) {
		mfc_err("Only mmaped buffers can be used\n");
		return -EINVAL;
	}
	mfc_debug(2, "State: %d, buf->type: %d\n", ctx->state, buf->type);
	if (ctx->state == MFCINST_GOT_INST &&
			buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_src, buf);
	} else if (ctx->state == MFCINST_RUNNING &&
			buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_dst, buf);
		for (i = 0; i < buf->length; i++)
			buf->m.planes[i].m.mem_offset += DST_QUEUE_OFF_BASE;
	} else {
		mfc_err("vidioc_querybuf called in an inappropriate state\n");
		ret = -EINVAL;
	}
	mfc_debug_leave();
	return ret;
}

/* Queue a buffer */
static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);

	if (ctx->state == MFCINST_ERROR) {
		mfc_err("Call on QBUF after unrecoverable error\n");
		return -EIO;
	}
	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_qbuf(&ctx->vq_src, buf);
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_qbuf(&ctx->vq_dst, buf);
	return -EINVAL;
}

/* Dequeue a buffer */
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	const struct v4l2_event ev = {
		.type = V4L2_EVENT_EOS
	};
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);
	int ret;

	if (ctx->state == MFCINST_ERROR) {
		mfc_err_limited("Call on DQBUF after unrecoverable error\n");
		return -EIO;
	}

	switch (buf->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return vb2_dqbuf(&ctx->vq_src, buf, file->f_flags & O_NONBLOCK);
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		ret = vb2_dqbuf(&ctx->vq_dst, buf, file->f_flags & O_NONBLOCK);
		if (ret)
			return ret;

		if (ctx->state == MFCINST_FINISHED &&
		    (ctx->dst_bufs[buf->index].flags & MFC_BUF_FLAG_EOS))
			v4l2_event_queue_fh(&ctx->fh, &ev);
		return 0;
	default:
		return -EINVAL;
	}
}

/* Export DMA buffer */
static int vidioc_expbuf(struct file *file, void *priv,
	struct v4l2_exportbuffer *eb)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);

	if (eb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_expbuf(&ctx->vq_src, eb);
	if (eb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_expbuf(&ctx->vq_dst, eb);
	return -EINVAL;
}

/* Stream on */
static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);
	int ret = -EINVAL;

	mfc_debug_enter();
	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ret = vb2_streamon(&ctx->vq_src, type);
	else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ret = vb2_streamon(&ctx->vq_dst, type);
	mfc_debug_leave();
	return ret;
}

/* Stream off, which equals to a pause */
static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_streamoff(&ctx->vq_src, type);
	else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_streamoff(&ctx->vq_dst, type);
	return -EINVAL;
}

/* Set controls - v4l2 control framework */
static int s5p_mfc_dec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5p_mfc_ctx *ctx = ctrl_to_ctx(ctrl);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY:
		ctx->display_delay = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY_ENABLE:
		ctx->display_delay_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER:
		ctx->loop_filter_mpeg4 = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE:
		ctx->slice_interface = ctrl->val;
		break;
	default:
		mfc_err("Invalid control 0x%08x\n", ctrl->id);
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_dec_g_v_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5p_mfc_ctx *ctx = ctrl_to_ctx(ctrl);
	struct s5p_mfc_dev *dev = ctx->dev;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		if (ctx->state >= MFCINST_HEAD_PARSED &&
		    ctx->state < MFCINST_ABORT) {
			ctrl->val = ctx->pb_count;
			break;
		} else if (ctx->state != MFCINST_INIT &&
				ctx->state != MFCINST_RES_CHANGE_END) {
			v4l2_err(&dev->v4l2_dev, "Decoding not initialised\n");
			return -EINVAL;
		}
		/* Should wait for the header to be parsed */
		ret = s5p_mfc_wait_for_done_ctx(ctx);
		if (ret) {
			mfc_err("Waiting for context %d timed out\n",
				ctx->num);
			return ret;
		}
		if (ctx->state >= MFCINST_HEAD_PARSED &&
		    ctx->state < MFCINST_ABORT) {
			ctrl->val = ctx->pb_count;
		} else {
			v4l2_err(&dev->v4l2_dev, "Decoding not initialised\n");
			return -EINVAL;
		}
		break;
	}
	return 0;
}


static const struct v4l2_ctrl_ops s5p_mfc_dec_ctrl_ops = {
	.s_ctrl = s5p_mfc_dec_s_ctrl,
	.g_volatile_ctrl = s5p_mfc_dec_g_v_ctrl,
};

/* Get cropping information */
static int vidioc_g_crop(struct file *file, void *priv,
		struct v4l2_crop *cr)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);
	struct s5p_mfc_dev *dev = ctx->dev;
	u32 left, right, top, bottom;

	if (ctx->state != MFCINST_HEAD_PARSED &&
	    ctx->state != MFCINST_RUNNING &&
	    ctx->state != MFCINST_FINISHING &&
	    ctx->state != MFCINST_FINISHED) {
		mfc_err("Can not get crop information\n");
		return -EINVAL;
	}
	if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_H264) {
		left = s5p_mfc_hw_call(dev->mfc_ops, get_crop_info_h, ctx);
		right = left >> S5P_FIMV_SHARED_CROP_RIGHT_SHIFT;
		left = left & S5P_FIMV_SHARED_CROP_LEFT_MASK;
		top = s5p_mfc_hw_call(dev->mfc_ops, get_crop_info_v, ctx);
		bottom = top >> S5P_FIMV_SHARED_CROP_BOTTOM_SHIFT;
		top = top & S5P_FIMV_SHARED_CROP_TOP_MASK;
		cr->c.left = left;
		cr->c.top = top;
		cr->c.width = ctx->img_width - left - right;
		cr->c.height = ctx->img_height - top - bottom;
		mfc_debug(2, "Cropping info [h264]: l=%d t=%d w=%d h=%d (r=%d b=%d fw=%d fh=%d\n",
			  left, top, cr->c.width, cr->c.height, right, bottom,
			  ctx->buf_width, ctx->buf_height);
	} else {
		cr->c.left = 0;
		cr->c.top = 0;
		cr->c.width = ctx->img_width;
		cr->c.height = ctx->img_height;
		mfc_debug(2, "Cropping info: w=%d h=%d fw=%d fh=%d\n",
			  cr->c.width,	cr->c.height, ctx->buf_width,
			  ctx->buf_height);
	}
	return 0;
}

static int vidioc_decoder_cmd(struct file *file, void *priv,
			      struct v4l2_decoder_cmd *cmd)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(priv);
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *buf;
	unsigned long flags;

	switch (cmd->cmd) {
	case V4L2_DEC_CMD_STOP:
		if (cmd->flags != 0)
			return -EINVAL;

		if (!vb2_is_streaming(&ctx->vq_src))
			return -EINVAL;

		spin_lock_irqsave(&dev->irqlock, flags);
		if (list_empty(&ctx->src_queue)) {
			mfc_err("EOS: empty src queue, entering finishing state");
			ctx->state = MFCINST_FINISHING;
			spin_unlock_irqrestore(&dev->irqlock, flags);
			s5p_mfc_try_ctx(ctx);
		} else {
			mfc_err("EOS: marking last buffer of stream");
			buf = list_entry(ctx->src_queue.prev,
						struct s5p_mfc_buf, list);
			if (buf->flags & MFC_BUF_FLAG_USED)
				ctx->state = MFCINST_FINISHING;
			else
				buf->flags |= MFC_BUF_FLAG_EOS;
			spin_unlock_irqrestore(&dev->irqlock, flags);
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				const struct  v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}


/* v4l2_ioctl_ops */
static const struct v4l2_ioctl_ops s5p_mfc_dec_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = vidioc_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out_mplane = vidioc_enum_fmt_vid_out_mplane,
	.vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt,
	.vidioc_reqbufs = vidioc_reqbufs,
	.vidioc_querybuf = vidioc_querybuf,
	.vidioc_qbuf = vidioc_qbuf,
	.vidioc_dqbuf = vidioc_dqbuf,
	.vidioc_expbuf = vidioc_expbuf,
	.vidioc_streamon = vidioc_streamon,
	.vidioc_streamoff = vidioc_streamoff,
	.vidioc_g_crop = vidioc_g_crop,
	.vidioc_decoder_cmd = vidioc_decoder_cmd,
	.vidioc_subscribe_event = vidioc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int s5p_mfc_queue_setup(struct vb2_queue *vq,
			unsigned int *buf_count,
			unsigned int *plane_count, unsigned int psize[],
			struct device *alloc_devs[])
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(vq->drv_priv);
	struct s5p_mfc_dev *dev = ctx->dev;

	/* Video output for decoding (source)
	 * this can be set after getting an instance */
	if (ctx->state == MFCINST_INIT &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* A single plane is required for input */
		*plane_count = 1;
		if (*buf_count < 1)
			*buf_count = 1;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
	/* Video capture for decoding (destination)
	 * this can be set after the header was parsed */
	} else if (ctx->state == MFCINST_HEAD_PARSED &&
		   vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* Output plane count is 2 - one for Y and one for CbCr */
		*plane_count = 2;
		/* Setup buffer count */
		if (*buf_count < ctx->pb_count)
			*buf_count = ctx->pb_count;
		if (*buf_count > ctx->pb_count + MFC_MAX_EXTRA_DPB)
			*buf_count = ctx->pb_count + MFC_MAX_EXTRA_DPB;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
	} else {
		mfc_err("State seems invalid. State = %d, vq->type = %d\n",
							ctx->state, vq->type);
		return -EINVAL;
	}
	mfc_debug(2, "Buffer count=%d, plane count=%d\n",
						*buf_count, *plane_count);
	if (ctx->state == MFCINST_HEAD_PARSED &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		psize[0] = ctx->luma_size;
		psize[1] = ctx->chroma_size;

		if (IS_MFCV6_PLUS(dev))
			alloc_devs[0] = ctx->dev->mem_dev[BANK_L_CTX];
		else
			alloc_devs[0] = ctx->dev->mem_dev[BANK_R_CTX];
		alloc_devs[1] = ctx->dev->mem_dev[BANK_L_CTX];
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
		   ctx->state == MFCINST_INIT) {
		psize[0] = ctx->dec_src_buf_size;
		alloc_devs[0] = ctx->dev->mem_dev[BANK_L_CTX];
	} else {
		mfc_err("This video node is dedicated to decoding. Decoding not initialized\n");
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = fh_to_ctx(vq->drv_priv);
	unsigned int i;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (ctx->capture_state == QUEUE_BUFS_MMAPED)
			return 0;
		for (i = 0; i < ctx->dst_fmt->num_planes; i++) {
			if (IS_ERR_OR_NULL(ERR_PTR(
					vb2_dma_contig_plane_dma_addr(vb, i)))) {
				mfc_err("Plane mem not allocated\n");
				return -EINVAL;
			}
		}
		if (vb2_plane_size(vb, 0) < ctx->luma_size ||
			vb2_plane_size(vb, 1) < ctx->chroma_size) {
			mfc_err("Plane buffer (CAPTURE) is too small\n");
			return -EINVAL;
		}
		i = vb->index;
		ctx->dst_bufs[i].b = vbuf;
		ctx->dst_bufs[i].cookie.raw.luma =
					vb2_dma_contig_plane_dma_addr(vb, 0);
		ctx->dst_bufs[i].cookie.raw.chroma =
					vb2_dma_contig_plane_dma_addr(vb, 1);
		ctx->dst_bufs_cnt++;
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (IS_ERR_OR_NULL(ERR_PTR(
					vb2_dma_contig_plane_dma_addr(vb, 0)))) {
			mfc_err("Plane memory not allocated\n");
			return -EINVAL;
		}
		if (vb2_plane_size(vb, 0) < ctx->dec_src_buf_size) {
			mfc_err("Plane buffer (OUTPUT) is too small\n");
			return -EINVAL;
		}

		i = vb->index;
		ctx->src_bufs[i].b = vbuf;
		ctx->src_bufs[i].cookie.stream =
					vb2_dma_contig_plane_dma_addr(vb, 0);
		ctx->src_bufs_cnt++;
	} else {
		mfc_err("s5p_mfc_buf_init: unknown queue type\n");
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct s5p_mfc_ctx *ctx = fh_to_ctx(q->drv_priv);

	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	if (ctx->state == MFCINST_FINISHING ||
		ctx->state == MFCINST_FINISHED)
		ctx->state = MFCINST_RUNNING;
	s5p_mfc_try_ctx(ctx);
	return 0;
}

static void s5p_mfc_stop_streaming(struct vb2_queue *q)
{
	unsigned long flags;
	struct s5p_mfc_ctx *ctx = fh_to_ctx(q->drv_priv);
	struct s5p_mfc_dev *dev = ctx->dev;
	int aborted = 0;

	spin_lock_irqsave(&dev->irqlock, flags);
	if ((ctx->state == MFCINST_FINISHING ||
		ctx->state ==  MFCINST_RUNNING) &&
		dev->curr_ctx == ctx->num && s5p_mfc_hw_is_locked(dev)) {
		ctx->state = MFCINST_ABORT;
		spin_unlock_irqrestore(&dev->irqlock, flags);
		if (s5p_mfc_wait_for_done_ctx(ctx))
			mfc_err("Waiting for context %d timed out\n",
				ctx->num);
		else
			aborted = 1;
		spin_lock_irqsave(&dev->irqlock, flags);
	}
	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		s5p_mfc_cleanup_queue(&ctx->dst_queue, &ctx->vq_dst);
		INIT_LIST_HEAD(&ctx->dst_queue);
		ctx->dst_queue_cnt = 0;
		ctx->dpb_flush_flag = 1;
		ctx->dec_dst_flag = 0;
		if (IS_MFCV6_PLUS(dev) && (ctx->state == MFCINST_RUNNING)) {
			ctx->state = MFCINST_FLUSH;
			s5p_mfc_try_ctx(ctx);
			spin_unlock_irqrestore(&dev->irqlock, flags);
			if (s5p_mfc_wait_for_done_ctx(ctx))
				mfc_err("Err flushing buffers\n");
			spin_lock_irqsave(&dev->irqlock, flags);
		}
	} else if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		s5p_mfc_cleanup_queue(&ctx->src_queue, &ctx->vq_src);
		INIT_LIST_HEAD(&ctx->src_queue);
		ctx->src_queue_cnt = 0;
	}
	if (aborted)
		ctx->state = MFCINST_RUNNING;
	spin_unlock_irqrestore(&dev->irqlock, flags);
}


static void s5p_mfc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = fh_to_ctx(vq->drv_priv);
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long flags;
	struct s5p_mfc_buf *mfc_buf;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_buf = &ctx->src_bufs[vb->index];
		mfc_buf->flags &= ~MFC_BUF_FLAG_USED;
		spin_lock_irqsave(&dev->irqlock, flags);
		list_add_tail(&mfc_buf->list, &ctx->src_queue);
		ctx->src_queue_cnt++;
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_buf = &ctx->dst_bufs[vb->index];
		mfc_buf->flags &= ~MFC_BUF_FLAG_USED;
		/* Mark destination as available for use by MFC */
		spin_lock_irqsave(&dev->irqlock, flags);
		set_bit(vb->index, &ctx->dec_dst_flag);
		list_add_tail(&mfc_buf->list, &ctx->dst_queue);
		ctx->dst_queue_cnt++;
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else {
		mfc_err("Unsupported buffer type (%d)\n", vq->type);
	}
	s5p_mfc_try_ctx(ctx);
}

static struct vb2_ops s5p_mfc_dec_qops = {
	.queue_setup		= s5p_mfc_queue_setup,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_init		= s5p_mfc_buf_init,
	.start_streaming	= s5p_mfc_start_streaming,
	.stop_streaming		= s5p_mfc_stop_streaming,
	.buf_queue		= s5p_mfc_buf_queue,
};

const struct s5p_mfc_codec_ops *get_dec_codec_ops(void)
{
	return &decoder_codec_ops;
}

struct vb2_ops *get_dec_queue_ops(void)
{
	return &s5p_mfc_dec_qops;
}

const struct v4l2_ioctl_ops *get_dec_v4l2_ioctl_ops(void)
{
	return &s5p_mfc_dec_ioctl_ops;
}

#define IS_MFC51_PRIV(x) ((V4L2_CTRL_ID2WHICH(x) == V4L2_CTRL_CLASS_MPEG) \
						&& V4L2_CTRL_DRIVER_PRIV(x))

int s5p_mfc_dec_ctrls_setup(struct s5p_mfc_ctx *ctx)
{
	struct v4l2_ctrl_config cfg;
	int i;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, NUM_CTRLS);
	if (ctx->ctrl_handler.error) {
		mfc_err("v4l2_ctrl_handler_init failed\n");
		return ctx->ctrl_handler.error;
	}

	for (i = 0; i < NUM_CTRLS; i++) {
		if (IS_MFC51_PRIV(controls[i].id)) {
			memset(&cfg, 0, sizeof(struct v4l2_ctrl_config));
			cfg.ops = &s5p_mfc_dec_ctrl_ops;
			cfg.id = controls[i].id;
			cfg.min = controls[i].minimum;
			cfg.max = controls[i].maximum;
			cfg.def = controls[i].default_value;
			cfg.name = controls[i].name;
			cfg.type = controls[i].type;

			cfg.step = controls[i].step;
			cfg.menu_skip_mask = 0;

			ctx->ctrls[i] = v4l2_ctrl_new_custom(&ctx->ctrl_handler,
					&cfg, NULL);
		} else {
			ctx->ctrls[i] = v4l2_ctrl_new_std(&ctx->ctrl_handler,
					&s5p_mfc_dec_ctrl_ops,
					controls[i].id, controls[i].minimum,
					controls[i].maximum, controls[i].step,
					controls[i].default_value);
		}
		if (ctx->ctrl_handler.error) {
			mfc_err("Adding control (%d) failed\n", i);
			return ctx->ctrl_handler.error;
		}
		if (controls[i].is_volatile && ctx->ctrls[i])
			ctx->ctrls[i]->flags |= V4L2_CTRL_FLAG_VOLATILE;
	}
	return 0;
}

void s5p_mfc_dec_ctrls_delete(struct s5p_mfc_ctx *ctx)
{
	int i;

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	for (i = 0; i < NUM_CTRLS; i++)
		ctx->ctrls[i] = NULL;
}

void s5p_mfc_dec_init(struct s5p_mfc_ctx *ctx)
{
	struct v4l2_format f;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	ctx->src_fmt = find_format(&f, MFC_FMT_DEC);
	if (IS_MFCV8(ctx->dev))
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
	else if (IS_MFCV6_PLUS(ctx->dev))
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12MT_16X16;
	else
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12MT;
	ctx->dst_fmt = find_format(&f, MFC_FMT_RAW);
	mfc_debug(2, "Default src_fmt is %p, dest_fmt is %p\n",
			ctx->src_fmt, ctx->dst_fmt);
}

