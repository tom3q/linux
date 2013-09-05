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
#include <linux/pm_runtime.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "coda.h"
#include "coda-regs.h"

int coda_debug;
module_param(coda_debug, int, 0644);
MODULE_PARM_DESC(coda_debug, "Debug level (0-1)");

static inline int coda_get_bitstream_payload(struct coda_ctx *ctx)
{
	return kfifo_len(&ctx->bitstream_fifo);
}

static int coda_wait_timeout(struct coda_dev *dev)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);

	while (coda_isbusy(dev)) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	}
	return 0;
}

void coda_command_async(struct coda_ctx *ctx, int cmd)
{
	struct coda_dev *dev = ctx->dev;

	/* Restore context related registers to CODA */
	if (dev->devtype->has_frm_dis)
		coda_write(dev, ctx->frm_dis_flg,
				CODA_REG_BIT_FRM_DIS_FLG(ctx->reg_idx));
	if (dev->devtype->ctxbuf_size)
		coda_write(dev, ctx->workbuf.paddr, CODA_REG_BIT_WORK_BUF_ADDR);
	if (dev->devtype->has_aux_std)
		coda_write(dev, ctx->params.codec_mode_aux,
				CODA7_REG_BIT_RUN_AUX_STD);
	coda_write(dev, ctx->bit_stream_param,
				CODA_REG_BIT_BIT_STREAM_PARAM);
	coda_write(dev, ctx->idx, CODA_REG_BIT_RUN_INDEX);
	coda_write(dev, ctx->params.codec_mode, CODA_REG_BIT_RUN_COD_STD);

	coda_write(dev, CODA_REG_BIT_BUSY_FLAG, CODA_REG_BIT_BUSY);
	coda_write(dev, cmd, CODA_REG_BIT_RUN_COMMAND);
}

int coda_command_sync(struct coda_ctx *ctx, int cmd)
{
	struct coda_dev *dev = ctx->dev;

	coda_command_async(ctx, cmd);
	return coda_wait_timeout(dev);
}

/*
 * Format management.
 */
static struct coda_fmt coda_formats[] = {
	{
		.name = "YUV 4:2:0 Planar, YCbCr",
		.fourcc = V4L2_PIX_FMT_YUV420,
		.flags = 0,
	},
	{
		.name = "YUV 4:2:0 Planar, YCrCb",
		.fourcc = V4L2_PIX_FMT_YVU420,
		.flags = CODA_FMT_YVU,
	},
	{
		.name = "YUV 4:2:0 Multiplanar, YCbCr",
		.fourcc = V4L2_PIX_FMT_YUV420M,
		.flags = CODA_FMT_MPLANE,
	},
	{
		.name = "YUV 4:2:0 Multiplanar, YCrCb",
		.fourcc = V4L2_PIX_FMT_YVU420M,
		.flags = CODA_FMT_YVU | CODA_FMT_MPLANE,
	},
	{
		.name = "H264 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_H264,
		.flags = CODA_FMT_BITSTREAM,
	},
	{
		.name = "MPEG4 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_MPEG4,
		.flags = CODA_FMT_BITSTREAM,
	},
	{
		.name = "H263 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_H263,
		.flags = CODA_FMT_BITSTREAM,
	},
};

#define CODA_DECODER(mode, fourcc, max_w, max_h) \
	{ mode, V4L2_PIX_FMT_ ## fourcc, CODA_INST_DECODER, max_w, max_h }

#define CODA_ENCODER(mode, fourcc, max_w, max_h) \
	{ mode, V4L2_PIX_FMT_ ## fourcc, CODA_INST_ENCODER, max_w, max_h }

/*
 * Arrays of codecs supported by each given version of Coda:
 * S3C64xx -> codamfcv1
 *  i.MX27 -> codadx6
 *  i.MX5x -> coda7
 *  i.MX6  -> coda960
 */
static struct coda_codec codadx6_codecs[] = {
	CODA_ENCODER(CODADX6_MODE_ENCODE_H264, H264, 720, 576),
	CODA_ENCODER(CODADX6_MODE_ENCODE_MP4, MPEG4, 720, 576),
};

static struct coda_codec coda7_codecs[] = {
	CODA_ENCODER(CODA7_MODE_ENCODE_H264, H264, 1280, 720),
	CODA_ENCODER(CODA7_MODE_ENCODE_MP4, MPEG4, 1280, 720),
	CODA_DECODER(CODA7_MODE_DECODE_H264, H264, 1920, 1080),
	CODA_DECODER(CODA7_MODE_DECODE_MP4, MPEG4, 1920, 1080),
};

static struct coda_codec mfcv1_codecs[] = {
	CODA_ENCODER(CODADX6_MODE_ENCODE_H264, H264, 864, 608),
	CODA_ENCODER(CODADX6_MODE_ENCODE_MP4, MPEG4, 720, 576),
	CODA_DECODER(CODADX6_MODE_DECODE_H264, H264, 864, 608),
	CODA_DECODER(CODADX6_MODE_DECODE_MP4, MPEG4, 720, 576),
	CODA_DECODER(CODADX6_MODE_DECODE_MP4, H263, 720, 480),
};

const struct coda_fmt *coda_find_format(u32 fourcc)
{
	int i;

	/* Try to match the FourCC first... */
	for (i = 0; i < ARRAY_SIZE(coda_formats); ++i)
		if (coda_formats[i].fourcc == fourcc)
			return &coda_formats[i];

	/* Return default format otherwise. */
	return &coda_formats[0];
}

const struct coda_codec *coda_find_codec(struct coda_ctx *ctx,
					 const struct coda_fmt *fmt)
{
	struct coda_dev *dev = ctx->dev;
	struct coda_codec *codecs = dev->devtype->codecs;
	int num_codecs = dev->devtype->num_codecs;
	int k;

	if (!fmt)
		goto no_fmt;

	/* Try to find the requested format. */
	for (k = 0; k < num_codecs; k++)
		if (codecs[k].type == ctx->inst_type &&
		    codecs[k].fourcc == fmt->fourcc)
			return &codecs[k];

no_fmt:
	/* Fall back to any codec supported by this instance type. */
	for (k = 0; k < num_codecs; k++)
		if (codecs[k].type == ctx->inst_type)
			return &codecs[k];

	/* This should not happen. */
	return NULL;
}

/*
 * Utility buffer helpers.
 */
static void coda_parabuf_write(struct coda_ctx *ctx, int index, u32 value)
{
	struct coda_dev *dev = ctx->dev;
	u32 *p = ctx->parabuf.vaddr;

	if (dev->devtype->has_swapped_parabuf)
		p[index] = value;
	else
		p[index ^ 1] = value;
}

static int coda_alloc_aux_buf(struct coda_dev *dev,
			      struct coda_aux_buf *buf, size_t size)
{
	buf->vaddr = dma_alloc_coherent(dev->dev, size, &buf->paddr,
					GFP_KERNEL);
	if (!buf->vaddr)
		return -ENOMEM;

	buf->size = size;

	return 0;
}

static void coda_free_aux_buf(struct coda_dev *dev,
			      struct coda_aux_buf *buf)
{
	if (buf->vaddr) {
		dma_free_coherent(dev->dev, buf->size,
				  buf->vaddr, buf->paddr);
		buf->vaddr = NULL;
		buf->size = 0;
	}
}

void coda_free_framebuffers(struct coda_ctx *ctx)
{
	int i;

	for (i = 0; i < CODA_MAX_FRAMEBUFFERS; i++)
		coda_free_aux_buf(ctx->dev, &ctx->internal_frames[i]);
}

int coda_alloc_framebuffers(struct coda_ctx *ctx, struct coda_q_data *q_data)
{
	struct coda_dev *dev = ctx->dev;
	unsigned int ysize;
	dma_addr_t paddr;
	size_t size;
	int i, ret;

	ysize = q_data->width * q_data->height;
	size = 3 * ysize / 2;

	if (ctx->codec->type == CODA_INST_DECODER &&
	    ctx->codec->fourcc == V4L2_PIX_FMT_H264 &&
	    dev->devtype->has_mvcol_buf)
		size += ysize / 4;

	/* Allocate frame buffers */
	for (i = 0; i < ctx->num_internal_frames; i++) {
		ret = coda_alloc_aux_buf(ctx->dev,
					&ctx->internal_frames[i], size);
		if (ret < 0) {
			coda_free_framebuffers(ctx);
			return ret;
		}
	}

	/* Register frame buffers in the parameter buffer */
	for (i = 0; i < ctx->num_internal_frames; i++) {
		paddr = ctx->internal_frames[i].paddr;
		coda_parabuf_write(ctx, i * 3 + 0, paddr); /* Y */
		coda_parabuf_write(ctx, i * 3 + 1, paddr + ysize); /* Cb */
		coda_parabuf_write(ctx, i * 3 + 2,
					paddr + ysize + ysize / 4); /* Cr */

		/* mvcol buffer for h.264 */
		if (ctx->codec->type == CODA_INST_DECODER &&
		    ctx->codec->fourcc == V4L2_PIX_FMT_H264 &&
		    dev->devtype->has_mvcol_buf)
			coda_parabuf_write(ctx, 96 + i,
						ctx->internal_frames[i].paddr +
						ysize + ysize / 4 + ysize / 4);
	}

	/* mvcol buffer for mpeg4 */
	if (dev->devtype->has_mvcol_buf &&
	    ctx->codec->type == CODA_INST_DECODER &&
	    ctx->codec->fourcc == V4L2_PIX_FMT_MPEG4)
		coda_parabuf_write(ctx, 97, ctx->internal_frames[i].paddr +
						ysize + ysize / 4 + ysize / 4);

	return 0;
}

void coda_free_context_buffers(struct coda_ctx *ctx)
{
	struct coda_dev *dev = ctx->dev;

	coda_free_aux_buf(dev, &ctx->slicebuf);
	coda_free_aux_buf(dev, &ctx->psbuf);
	if (dev->devtype->ctxbuf_size)
		coda_free_aux_buf(dev, &ctx->workbuf);
}

int coda_alloc_context_buffers(struct coda_ctx *ctx,
				      struct coda_q_data *q_data)
{
	struct coda_dev *dev = ctx->dev;
	int ret;

	if (dev->devtype->has_slice_buf &&
	    q_data->fmt->fourcc == V4L2_PIX_FMT_H264) {
		u32 size;

		if (ctx->slicebuf.vaddr) {
			v4l2_err(&dev->v4l2_dev, "slicebuf still allocated\n");
			return -EBUSY;
		}

		/* worst case slice size */
		size = (DIV_ROUND_UP(q_data->width, 16) *
			DIV_ROUND_UP(q_data->height, 16)) * 3200 / 8 + 512;

		ret = coda_alloc_aux_buf(dev, &ctx->slicebuf, size);
		if (ret < 0) {
			v4l2_err(&dev->v4l2_dev,
				"failed to allocate %d byte slice buffer",
				ctx->slicebuf.size);
			return ret;
		}
	}

	if (dev->devtype->psbuf_size) {
		if (ctx->psbuf.vaddr) {
			v4l2_err(&dev->v4l2_dev, "psmembuf still allocated\n");
			return -EBUSY;
		}

		ret = coda_alloc_aux_buf(dev, &ctx->psbuf,
						dev->devtype->psbuf_size);
		if (ret < 0) {
			v4l2_err(&dev->v4l2_dev,
					"failed to allocate psmem buffer");
			goto err;
		}
	}

	if (dev->devtype->ctxbuf_size) {
		if (ctx->workbuf.vaddr) {
			v4l2_err(&dev->v4l2_dev,
					"context buffer still allocated\n");
			return -EBUSY;
		}

		ret = coda_alloc_aux_buf(dev, &ctx->workbuf,
						dev->devtype->ctxbuf_size);
		if (ret < 0) {
			v4l2_err(&dev->v4l2_dev,
				"failed to allocate %d byte context buffer",
				ctx->workbuf.size);
			goto err;
		}
	}

	return 0;

err:
	coda_free_context_buffers(ctx);

	return ret;
}

/*
 * Common V4L2 IOCTL helpers.
 */
int coda_enum_fmt_bitstream(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct coda_ctx *ctx = fh_to_ctx(fh);
	struct coda_codec *codecs = ctx->dev->devtype->codecs;
	int num_codecs = ctx->dev->devtype->num_codecs;
	int i, k, num = 0;

	for (i = 0; i < ARRAY_SIZE(coda_formats); i++) {
		if (coda_format_is_yuv(&coda_formats[i]))
			continue;

		/* Compressed formats may be supported, check the codec list */
		for (k = 0; k < num_codecs; k++) {
			if (codecs[k].type != ctx->inst_type)
				continue;
			if (codecs[k].fourcc != coda_formats[i].fourcc)
				continue;
			if (num++ == f->index) {
				strlcpy(f->description, coda_formats[i].name,
					sizeof(f->description));
				f->pixelformat = coda_formats[i].fourcc;
				return 0;
			}
		}
	}

	/* Format not found. */
	return -EINVAL;
}

int coda_enum_fmt_raw(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	int i, num = 0;

	for (i = 0; i < ARRAY_SIZE(coda_formats); i++) {
		if (!coda_format_is_yuv(&coda_formats[i]))
			continue;
		if (num++ == f->index) {
			strlcpy(f->description, coda_formats[i].name,
				sizeof(f->description));
			f->pixelformat = coda_formats[i].fourcc;
			return 0;
		}
	}

	/* Format not found. */
	return -EINVAL;
}

int coda_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct coda_ctx *ctx = fh_to_ctx(fh);
	struct coda_q_data *q_data;
	int i;

	q_data = get_q_data(ctx, f->type);

	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.pixelformat = q_data->fmt->fourcc;
	f->fmt.pix_mp.width = q_data->width;
	f->fmt.pix_mp.height = q_data->height;
	f->fmt.pix_mp.num_planes = q_data->num_planes;
	f->fmt.pix_mp.colorspace	= ctx->colorspace;

	/* Bitstream data: Always single plane and zerp bytesperline. */
	if (!coda_format_is_yuv(q_data->fmt)) {
		f->fmt.pix_mp.plane_fmt[0].bytesperline = 0;
		f->fmt.pix_mp.plane_fmt[0].sizeimage = q_data->sizeimage[0];
		return 0;
	}

	/* Raw video data: Can be multiplanar, has valid bytesperline. */
	f->fmt.pix_mp.plane_fmt[0].bytesperline =
				round_up(f->fmt.pix_mp.width, 2);
	f->fmt.pix_mp.plane_fmt[0].sizeimage = q_data->sizeimage[0];

	for (i = 1; i < f->fmt.pix_mp.num_planes; ++i) {
		f->fmt.pix_mp.plane_fmt[i].bytesperline =
				round_up(f->fmt.pix_mp.width / 2, 2);
		f->fmt.pix_mp.plane_fmt[i].sizeimage =
				q_data->sizeimage[i];
	}

	return 0;
}

int coda_try_fmt(struct v4l2_format *f)
{
	const struct coda_fmt *fmt;
	enum v4l2_field field;
	int i;

	fmt = coda_find_format(f->fmt.pix_mp.pixelformat);

	field = f->fmt.pix_mp.field;
	if (field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (field != V4L2_FIELD_NONE)
		return -EINVAL;

	f->fmt.pix_mp.field = field;

	/* Frame stride must be multiple of 8 */
	f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width;

	if (!coda_format_is_yuv(fmt)) {
		f->fmt.pix_mp.num_planes = 1;
		f->fmt.pix_mp.plane_fmt[0].bytesperline = 0;
		return 0;
	}

	if (!coda_format_is_mplane(fmt)) {
		f->fmt.pix_mp.num_planes = 1;
		f->fmt.pix_mp.plane_fmt[0].sizeimage =
				f->fmt.pix_mp.plane_fmt[0].bytesperline *
				f->fmt.pix_mp.height * 3 / 2;
		return 0;
	}

	f->fmt.pix_mp.num_planes = CODA_NUM_PLANES;
	f->fmt.pix_mp.plane_fmt[0].sizeimage =
				f->fmt.pix_mp.plane_fmt[0].bytesperline *
				f->fmt.pix_mp.height;

	for (i = 1; i < f->fmt.pix_mp.num_planes; ++i) {
		f->fmt.pix_mp.plane_fmt[i].bytesperline =
			round_up(f->fmt.pix_mp.width / 2, 4);
		f->fmt.pix_mp.plane_fmt[i].sizeimage =
			f->fmt.pix_mp.plane_fmt[i].bytesperline *
			f->fmt.pix_mp.height / 2;
	}

	return 0;
}

int coda_s_fmt(struct coda_ctx *ctx, struct v4l2_format *f,
			const struct coda_fmt *fmt)
{
	struct coda_q_data *q_data;
	struct vb2_queue *vq;
	int i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&ctx->dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	q_data->fmt = fmt;
	q_data->width = f->fmt.pix_mp.width;
	q_data->height = f->fmt.pix_mp.height;
	q_data->num_planes = f->fmt.pix_mp.num_planes;

	for (i = 0; i < f->fmt.pix_mp.num_planes; ++i)
		q_data->sizeimage[i] = f->fmt.pix_mp.plane_fmt[i].sizeimage;

	v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
		"Setting format for type %d, wxh: %dx%d, fmt: %d\n",
		f->type, q_data->width, q_data->height, fmt->fourcc);

	return 0;
}

int coda_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, CODA_NAME, sizeof(cap->driver));
	strlcpy(cap->card, CODA_NAME, sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:" CODA_NAME, sizeof(cap->bus_info));
	/*
	 * This is only a mem-to-mem video device. The capture and output
	 * device capability flags are left only for backward compatibility
	 * and are scheduled for removal.
	 */
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
				V4L2_CAP_VIDEO_OUTPUT_MPLANE |
				V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

int coda_reqbufs(struct file *file, void *priv,
		 struct v4l2_requestbuffers *reqbufs)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

int coda_querybuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);
}

int coda_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

int coda_expbuf(struct file *file, void *priv, struct v4l2_exportbuffer *eb)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_expbuf(file, ctx->m2m_ctx, eb);
}

static bool coda_buf_is_end_of_stream(struct coda_ctx *ctx,
				      struct v4l2_buffer *buf)
{
	struct coda_dev *dev = ctx->dev;
	struct vb2_queue *src_vq;

	src_vq = v4l2_m2m_get_vq(ctx->m2m_ctx,
					V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

	return ((ctx->bit_stream_param & BIT(dev->devtype->stream_end_bit)) &&
		(buf->sequence == (ctx->qsequence - 1)));
}

int coda_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);
	int ret;

	ret = v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);

	/* If this is the last capture buffer, emit an end-of-stream event */
	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    coda_buf_is_end_of_stream(ctx, buf)) {
		const struct v4l2_event eos_event = {
			.type = V4L2_EVENT_EOS
		};

		v4l2_event_queue_fh(&ctx->fh, &eos_event);
	}

	return ret;
}

int coda_create_bufs(struct file *file, void *priv,
		     struct v4l2_create_buffers *create)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_create_bufs(file, ctx->m2m_ctx, create);
}

int coda_streamon(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

int coda_streamoff(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct coda_ctx *ctx = fh_to_ctx(priv);
	int ret;

	/*
	 * This indirectly calls __vb2_queue_cancel, which dequeues all
	 * buffers. We therefore have to lock it against running hardware
	 * in this context, which still needs the buffers.
	 */
	mutex_lock(&ctx->buffer_mutex);
	ret = v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
	mutex_unlock(&ctx->buffer_mutex);

	return ret;
}

int coda_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

/*
 * Common V4L2 VB2 operations.
 */
void coda_wait_prepare(struct vb2_queue *q)
{
	struct coda_ctx *ctx = vb2_get_drv_priv(q);

	mutex_unlock(&ctx->dev->dev_mutex);
}

void coda_wait_finish(struct vb2_queue *q)
{
	struct coda_ctx *ctx = vb2_get_drv_priv(q);

	mutex_lock(&ctx->dev->dev_mutex);
}

int coda_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct coda_ctx *ctx = vb2_get_drv_priv(vq);
	struct coda_q_data *q_data;
	unsigned int size = 0;
	int i;

	q_data = get_q_data(ctx, vq->type);

	for (i = 0; i < q_data->num_planes; ++i) {
		size += q_data->sizeimage[i];
		sizes[i] = q_data->sizeimage[i];
		alloc_ctxs[i] = ctx->dev->alloc_ctx;
	}

	*nplanes = q_data->num_planes;

	v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
		 "get %d buffer(s) of size %d each.\n", *nbuffers, size);

	return 0;
}

int coda_buf_prepare(struct vb2_buffer *vb)
{
	struct coda_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct coda_q_data *q_data;
	int i;

	q_data = get_q_data(ctx, vb->vb2_queue->type);

	for (i = 0; i < q_data->num_planes; ++i) {
		if (vb2_plane_size(vb, i) < q_data->sizeimage[i]) {
			v4l2_warn(&ctx->dev->v4l2_dev,
				"%s data will not fit into plane (%lu < %lu)\n",
				__func__, vb2_plane_size(vb, 0),
				(long)q_data->sizeimage);
			return -EINVAL;
		}
	}

	return 0;
}

static int coda_queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct coda_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_DMABUF | VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = ctx->vdev->vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_DMABUF | VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = ctx->vdev->vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	return vb2_queue_init(dst_vq);
}

/*
 * Common V4L2 mem2mem operations.
 */
static void coda_skip_run(struct work_struct *work)
{
	struct coda_ctx *ctx = container_of(work, struct coda_ctx, skip_run);

	v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->m2m_ctx);
}

static void coda_device_run(void *m2m_priv)
{
	struct coda_ctx *ctx = m2m_priv;
	struct coda_dev *dev = ctx->dev;
	int ret;

	mutex_lock(&ctx->buffer_mutex);

	/*
	 * If streamoff dequeued all buffers before we could get the lock,
	 * just bail out immediately.
	 */
	if ((!v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx) &&
	    ctx->inst_type != CODA_INST_DECODER) ||
		!v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx)) {
		v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
			"%d: device_run without buffers\n", ctx->idx);
		mutex_unlock(&ctx->buffer_mutex);
		schedule_work(&ctx->skip_run);
		return;
	}

	mutex_lock(&dev->coda_mutex);

	ret = coda_clk_enable(dev);
	if (ret) {
		mutex_unlock(&dev->coda_mutex);
		mutex_unlock(&ctx->buffer_mutex);
		schedule_work(&ctx->skip_run);
		return;
	}

	ret = ctx->vdev->prepare(ctx);
	if (ret < 0) {
		mutex_unlock(&dev->coda_mutex);
		mutex_unlock(&ctx->buffer_mutex);
		coda_clk_disable(dev);
		schedule_work(&ctx->skip_run);
		return;
	}

	dev->curr_ctx = ctx;

	/* 1 second timeout in case CODA locks up */
	schedule_delayed_work(&dev->timeout, HZ);

	coda_command_async(ctx, CODA_COMMAND_PIC_RUN);
}

static int coda_job_ready(void *m2m_priv)
{
	struct coda_ctx *ctx = m2m_priv;
	struct coda_dev *dev = ctx->dev;

	/*
	 * For both 'P' and 'key' frame cases 1 picture
	 * and 1 frame are needed. In the decoder case,
	 * the compressed frame can be in the bitstream.
	 */
	if (!v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx) &&
	    ctx->inst_type == CODA_INST_ENCODER) {
		v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			 "not ready: not enough video buffers.\n");
		return 0;
	}

	if (!v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx)) {
		v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			 "not ready: not enough video capture buffers.\n");
		return 0;
	}

	if (ctx->prescan_failed ||
	    ((ctx->inst_type == CODA_INST_DECODER) &&
	     (coda_get_bitstream_payload(ctx) < 512) &&
	     !(ctx->bit_stream_param & BIT(dev->devtype->stream_end_bit)))) {
		v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			 "%d: not ready: not enough bitstream data.\n",
			 ctx->idx);
		return 0;
	}

	if (ctx->aborting) {
		v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			 "not ready: aborting\n");
		return 0;
	}

	v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			"job ready\n");
	return 1;
}

static void coda_job_abort(void *priv)
{
	struct coda_ctx *ctx = priv;

	ctx->aborting = 1;

	v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
		 "Aborting task\n");
}

static void coda_lock(void *priv)
{
	struct coda_ctx *ctx = priv;
	struct coda_dev *dev = ctx->dev;

	mutex_lock(&dev->coda_mutex);
}

static void coda_unlock(void *priv)
{
	struct coda_ctx *ctx = priv;
	struct coda_dev *dev = ctx->dev;

	mutex_unlock(&dev->coda_mutex);
}

static struct v4l2_m2m_ops coda_m2m_ops = {
	.device_run	= coda_device_run,
	.job_ready	= coda_job_ready,
	.job_abort	= coda_job_abort,
	.lock		= coda_lock,
	.unlock		= coda_unlock,
};

/*
 * Instance allocation helpers.
 * NOTE: Must be called with dev_mutex held.
 */
static int coda_alloc_instance(struct coda_dev *dev)
{
	int inst;

	inst = ffz(dev->instance_mask);
	if (inst < CODA_MAX_INSTANCES)
		set_bit(inst, &dev->instance_mask);
	else
		inst = -EBUSY;

	return inst;
}

static void coda_free_instance(struct coda_dev *dev, struct coda_ctx *ctx)
{
	clear_bit(ctx->idx, &dev->instance_mask);
}

static void coda_register_instance(struct coda_dev *dev, struct coda_ctx *ctx)
{
	list_add(&ctx->list, &dev->instances);
}

static void coda_unregister_instance(struct coda_dev *dev, struct coda_ctx *ctx)
{
	list_del(&ctx->list);
}

/*
 * Video devices to register.
 */
static const struct coda_video_dev *coda_video_devs[CODA_NUM_INST] = {
	[CODA_INST_DECODER] = &codec_dec_vdev,
	[CODA_INST_ENCODER] = &codec_enc_vdev,
};

/*
 * V4L2 file operations.
 */
static int coda_open(struct file *file)
{
	struct video_device *video_dev = video_devdata(file);
	struct coda_dev *dev = video_drvdata(file);
	const struct coda_video_dev *vdev;
	struct coda_ctx *ctx = NULL;
	const struct firmware *fw = NULL;
	int ret;

	if (WARN_ON_ONCE(!video_dev))
		return -ENODEV;

	if (WARN_ON_ONCE(video_dev->index >= ARRAY_SIZE(coda_video_devs)))
		return -ENODEV;

	vdev = coda_video_devs[video_dev->index];
	if (WARN_ON_ONCE(!vdev))
		return -ENODEV;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;

	if (!dev->codebuf.vaddr) {
		ret = request_firmware(&fw, dev->devtype->firmware, dev->dev);
		if (ret < 0) {
			v4l2_err(&dev->v4l2_dev, "firmware request failed\n");
			goto err_unlock;
		}

		/*
		 * BIT processor firmware must be located in DMA compatible
		 * memory, while firmware class allocates firmware buffers
		 * using vmalloc.
		 */
		ret = coda_alloc_aux_buf(dev, &dev->codebuf, fw->size);
		if (ret < 0) {
			v4l2_err(&dev->v4l2_dev,
					"firmware buffer allocation failed\n");
			release_firmware(fw);
			goto err_unlock;
		}

		memcpy(dev->codebuf.vaddr, fw->data, fw->size);
		release_firmware(fw);
	}

	ctx = kzalloc(sizeof *ctx, GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto err_unlock;
	}

	ctx->idx = coda_alloc_instance(dev);
	if (ctx->idx < 0) {
		ret = ctx->idx;
		goto err_free_ctx;
	}

	v4l2_fh_init(&ctx->fh, video_dev);
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->dev = dev;
	ctx->reg_idx = dev->devtype->has_reg_idx ? ctx->idx : 0;
	ctx->vdev = vdev;
	ctx->inst_type = video_dev->index;

	mutex_init(&ctx->bitstream_mutex);
	mutex_init(&ctx->buffer_mutex);
	INIT_WORK(&ctx->skip_run, coda_skip_run);

	ret = coda_alloc_aux_buf(ctx->dev, &ctx->parabuf, CODA_PARA_BUF_SIZE);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed to allocate parabuf");
		goto err_free_fh;
	}

	ctx->bitstream.size = CODA_MAX_FRAME_SIZE;
	ctx->bitstream.vaddr = dma_alloc_writecombine(dev->dev,
			ctx->bitstream.size, &ctx->bitstream.paddr, GFP_KERNEL);
	if (!ctx->bitstream.vaddr) {
		v4l2_err(&dev->v4l2_dev,
			"failed to allocate bitstream ringbuffer");
		ret = -ENOMEM;
		goto err_free_ctx_buf;
	}

	kfifo_init(&ctx->bitstream_fifo,
		ctx->bitstream.vaddr, ctx->bitstream.size);

	if (vdev->open) {
		ret = vdev->open(ctx);
		if (ret)
			goto err_free_bitstream;
	}

	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, coda_queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		v4l2_err(&dev->v4l2_dev, "%s return error (%d)\n",
			__func__, ret);
		goto err_free_bitstream;
	}

	coda_register_instance(dev, ctx);

	v4l2_dbg(1, coda_debug, &dev->v4l2_dev, "Created instance %d (%p)\n",
		 ctx->idx, ctx);

	mutex_unlock(&dev->dev_mutex);
	return 0;

err_free_bitstream:
	dma_free_writecombine(dev->dev, ctx->bitstream.size,
				ctx->bitstream.vaddr, ctx->bitstream.paddr);
err_free_ctx_buf:
	coda_free_context_buffers(ctx);
err_free_fh:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	coda_free_instance(dev, ctx);
err_free_ctx:
	kfree(ctx);
err_unlock:
	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static int coda_release(struct file *file)
{
	struct coda_dev *dev = video_drvdata(file);
	struct coda_ctx *ctx = fh_to_ctx(file->private_data);

	mutex_lock(&dev->dev_mutex);

	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	coda_free_framebuffers(ctx);
	coda_unregister_instance(dev, ctx);
	dma_free_writecombine(dev->dev, ctx->bitstream.size,
				ctx->bitstream.vaddr, ctx->bitstream.paddr);
	coda_free_context_buffers(ctx);
	if (dev->devtype->ctxbuf_size)
		coda_free_aux_buf(dev, &ctx->workbuf);
	coda_free_aux_buf(dev, &ctx->parabuf);
	v4l2_ctrl_handler_free(&ctx->ctrls);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	coda_free_instance(dev, ctx);
	kfree(ctx);

	v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
			"Released instance %d\n", ctx->idx);

	mutex_unlock(&dev->dev_mutex);
	return 0;
}

static unsigned int coda_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct coda_ctx *ctx = fh_to_ctx(file->private_data);
	struct coda_dev *dev = ctx->dev;
	int ret;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;

	ret = v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static int coda_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct coda_ctx *ctx = fh_to_ctx(file->private_data);
	struct coda_dev *dev = ctx->dev;
	int ret;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;

	ret = v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static const struct v4l2_file_operations coda_fops = {
	.owner		= THIS_MODULE,
	.open		= coda_open,
	.release	= coda_release,
	.poll		= coda_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= coda_mmap,
};

/*
 * Hardware operations
 */
static irqreturn_t coda_irq_handler(int irq, void *data)
{
	struct coda_dev *dev = data;
	struct coda_ctx *ctx;
	u32 stat;

	/* read status register to attend the IRQ */
	stat = coda_read(dev, CODA_REG_BIT_INT_STATUS);
	if (!stat)
		return IRQ_NONE;

	coda_write(dev, CODA_REG_BIT_INT_CLEAR_SET, CODA_REG_BIT_INT_CLEAR);

	cancel_delayed_work(&dev->timeout);

	ctx = dev->curr_ctx;
	if (ctx == NULL) {
		v4l2_err(&dev->v4l2_dev,
			"Instance released before the end of transaction\n");
		mutex_unlock(&dev->coda_mutex);
		return IRQ_HANDLED;
	}

	if (ctx->aborting) {
		v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			"task has been aborted\n");
		goto out;
	}

	if (coda_isbusy(ctx->dev)) {
		v4l2_dbg(1, coda_debug, &ctx->dev->v4l2_dev,
			"coda is still busy!!!!\n");
		return IRQ_HANDLED;
	}

	ctx->vdev->finish(ctx);

out:
	/* TODO: Implement aborting handling properly! */

	if (ctx->aborting || (!ctx->streamon_cap && !ctx->streamon_out)) {
		v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
			"%s: sending CODA_COMMAND_SEQ_END\n", __func__);

		if (coda_command_sync(ctx, CODA_COMMAND_SEQ_END))
			v4l2_err(&dev->v4l2_dev,
				"CODA_COMMAND_SEQ_END failed\n");

		kfifo_init(&ctx->bitstream_fifo,
			ctx->bitstream.vaddr, ctx->bitstream.size);

		coda_free_framebuffers(ctx);
		coda_free_context_buffers(ctx);
	}

	coda_clk_disable(dev);

	mutex_unlock(&dev->coda_mutex);
	mutex_unlock(&ctx->buffer_mutex);

	v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->m2m_ctx);

	return IRQ_HANDLED;
}

static void coda_timeout(struct work_struct *work)
{
	struct coda_ctx *ctx;
	struct coda_dev *dev = container_of(to_delayed_work(work),
					    struct coda_dev, timeout);

	/* TODO: Implement timeout handling properly! */

	dev_err(dev->dev, "CODA PIC_RUN timeout, stopping all streams\n");

	mutex_lock(&dev->dev_mutex);
	list_for_each_entry(ctx, &dev->instances, list) {
		if (mutex_is_locked(&ctx->buffer_mutex))
			mutex_unlock(&ctx->buffer_mutex);
		v4l2_m2m_streamoff(NULL, ctx->m2m_ctx,
					V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		v4l2_m2m_streamoff(NULL, ctx->m2m_ctx,
					V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	}
	mutex_unlock(&dev->dev_mutex);

	mutex_unlock(&dev->coda_mutex);
	ctx = dev->curr_ctx;
	if (ctx)
		v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->m2m_ctx);
}

static u32 coda_supported_firmwares[] = {
	CODA_FIRMWARE_VERNUM(CODA_DX6, 2, 2, 5),
	CODA_FIRMWARE_VERNUM(CODA_7541, 1, 4, 50),
	CODA_FIRMWARE_VERNUM(CODA_MFC_V1, 1, 3, 242),
};

static bool coda_firmware_supported(u32 vernum)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coda_supported_firmwares); i++)
		if (vernum == coda_supported_firmwares[i])
			return true;
	return false;
}

static char *coda_product_name(int product)
{
	static char buf[9];

	switch (product) {
	case CODA_MFC_V1:
		return "CodaMFCv1";
	case CODA_DX6:
		return "CodaDx6";
	case CODA_7541:
		return "CODA7541";
	default:
		snprintf(buf, sizeof(buf), "(0x%04x)", product);
		return buf;
	}
}

int coda_clk_enable(struct coda_dev *dev)
{
	int ret;

	ret = clk_prepare_enable(dev->clk_per);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "failed to enable peripheral clock\n");
		return ret;
	}

	ret = clk_prepare_enable(dev->clk_ahb);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "failed to enable bus clock\n");
		goto err_clk_per_disable;
	}

	if (!IS_ERR(dev->clk_spec)) {
		ret = clk_prepare_enable(dev->clk_spec);
		if (ret) {
			v4l2_err(&dev->v4l2_dev,
				"failed to enable special clock\n");
			goto err_clk_ahb_disable;
		}
	}

	return 0;

err_clk_ahb_disable:
	clk_disable_unprepare(dev->clk_ahb);
err_clk_per_disable:
	clk_disable_unprepare(dev->clk_per);

	return ret;
}

void coda_clk_disable(struct coda_dev *dev)
{
	if (!IS_ERR(dev->clk_spec))
		clk_disable_unprepare(dev->clk_spec);
	clk_disable_unprepare(dev->clk_ahb);
	clk_disable_unprepare(dev->clk_per);
}

static int coda_hw_init(struct coda_dev *dev)
{
	u16 product, major, minor, release;
	int old_coda_debug;
	u32 data;
	u16 *p;
	int i, ret;

	ret = coda_clk_enable(dev);
	if (ret)
		return ret;

	/* Disable coda_debug for firmware upload to reduce the noise. */
	old_coda_debug = coda_debug;
	coda_debug = 0;

	/*
	 * Copy the first CODA_ISRAM_SIZE in the internal SRAM.
	 * The 16-bit chars in the code buffer are in memory access
	 * order, re-sort them to CODA order for register download.
	 * Data in this SRAM survives a reboot.
	 */
	p = (u16 *)dev->codebuf.vaddr;
	if (!dev->devtype->has_swapped_codebuf) {
		for (i = 0; i < (CODA_ISRAM_SIZE / 2); i++)  {
			data = CODA_DOWN_ADDRESS_SET(i) |
				CODA_DOWN_DATA_SET(p[i ^ 1]);
			coda_write(dev, data, CODA_REG_BIT_CODE_DOWN);
		}
	} else {
		for (i = 0; i < (CODA_ISRAM_SIZE / 2); i++) {
			data = CODA_DOWN_ADDRESS_SET(i) |
				CODA_DOWN_DATA_SET(p[round_down(i, 4) +
							3 - (i % 4)]);
			coda_write(dev, data, CODA_REG_BIT_CODE_DOWN);
		}
	}

	/* Clear registers */
	for (i = 0; i < 64; i++)
		coda_write(dev, 0, CODA_REG_BIT_CODE_BUF_ADDR + i * 4);

	/* Restore coda_debug. */
	coda_debug = old_coda_debug;

	/* Tell the BIT where to find everything it needs */
	if (dev->devtype->tempbuf_size) {
		coda_write(dev, dev->tempbuf.paddr,
				CODA_REG_BIT_TEMP_BUF_ADDR);
		coda_write(dev, 0, CODA_REG_BIT_BIT_STREAM_PARAM);
	}

	if (dev->devtype->workbuf_size)
		coda_write(dev, dev->workbuf.paddr,
			      CODA_REG_BIT_WORK_BUF_ADDR);

	if (dev->devtype->product == CODA_MFC_V1)
		coda_write(dev, 0, CODAMFC_REG_BIT_WORK_BUF_CONFIG);

	coda_write(dev, dev->codebuf.paddr, CODA_REG_BIT_CODE_BUF_ADDR);
	coda_write(dev, 0, CODA_REG_BIT_CODE_RUN);

	/* Set default values */
	coda_write(dev, BIT(dev->devtype->stream_pic_flush_bit),
			CODA_REG_BIT_STREAM_CTRL);
	coda_write(dev, 0, CODA_REG_BIT_FRAME_MEM_CTRL);

	if (dev->devtype->product == CODA_7541)
		coda_write(dev, 0, CODA7_REG_BIT_AXI_SRAM_USE);

	/* TODO: Proper interrupt support */
	coda_write(dev, CODA_INT_INTERRUPT_ENABLE, CODA_REG_BIT_INT_ENABLE);

	/* Reset VPU and start processor */
	coda_write(dev, CODA_REG_RESET_ENABLE, CODA_REG_BIT_CODE_RESET);
	udelay(10);
	coda_write(dev, 0, CODA_REG_BIT_CODE_RESET);
	coda_write(dev, CODA_REG_RUN_ENABLE, CODA_REG_BIT_CODE_RUN);

	/* Load firmware */
	coda_write(dev, 0, CODA_CMD_FIRMWARE_VERNUM);
	coda_write(dev, CODA_REG_BIT_BUSY_FLAG, CODA_REG_BIT_BUSY);
	coda_write(dev, 0, CODA_REG_BIT_RUN_INDEX);
	coda_write(dev, 0, CODA_REG_BIT_RUN_COD_STD);
	coda_write(dev, CODA_COMMAND_FIRMWARE_GET, CODA_REG_BIT_RUN_COMMAND);

	if (coda_wait_timeout(dev)) {
		v4l2_err(&dev->v4l2_dev, "firmware get command error\n");
		coda_clk_disable(dev);
		return -EIO;
	}

	/* Check we are compatible with the loaded firmware */
	data = coda_read(dev, CODA_CMD_FIRMWARE_VERNUM);
	product = CODA_FIRMWARE_PRODUCT(data);
	major = CODA_FIRMWARE_MAJOR(data);
	minor = CODA_FIRMWARE_MINOR(data);
	release = CODA_FIRMWARE_RELEASE(data);

	if (product != dev->devtype->product) {
		v4l2_err(&dev->v4l2_dev,
			"Wrong firmware. HW product: %s, FW product: %s, Version: %u.%u.%u\n",
			coda_product_name(dev->devtype->product),
			coda_product_name(product), major, minor, release);
		coda_clk_disable(dev);
		return -EINVAL;
	}

	v4l2_info(&dev->v4l2_dev, "Initialized %s.\n",
			coda_product_name(product));

	if (coda_firmware_supported(data))
		v4l2_info(&dev->v4l2_dev, "Firmware version: %u.%u.%u\n",
				major, minor, release);
	else
		v4l2_warn(&dev->v4l2_dev,
				"Unsupported firmware version: %u.%u.%u\n",
				major, minor, release);

	coda_clk_disable(dev);
	return 0;
}

int coda_power_up(struct coda_dev *dev)
{
	int ret;

	ret = pm_runtime_get_sync(dev->dev);
	if (ret < 0)
		return ret;

	if (ret > 0)
		return 0;

	ret = coda_hw_init(dev);
	if (ret) {
		pm_runtime_put(dev->dev);
		return ret;
	}

	return 0;
}

void coda_power_down(struct coda_dev *dev)
{
	pm_runtime_put(dev->dev);
}

/*
 * Driver initialization
 */
static int coda_register_vdev(struct coda_dev *dev, int i)
{
	const struct coda_video_dev *vdev = coda_video_devs[i];
	struct video_device *vfd = &dev->vfd[i];
	int ret;

	vfd->fops = &coda_fops,
	vfd->ioctl_ops = vdev->ioctl_ops;
	vfd->release = video_device_release_empty,
	vfd->lock = &dev->dev_mutex;
	vfd->v4l2_dev = &dev->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	snprintf(vfd->name, sizeof(vfd->name), "%s.%s",
			CODA_NAME, vdev->name);
	video_set_drvdata(vfd, dev);

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev,
			"Failed to register video device for %s\n",
			vdev->name);
		return ret;
	}

	v4l2_info(&dev->v4l2_dev, "%s registered as /dev/video%d\n",
			vdev->name, vfd->num);

	return 0;
}

enum coda_platform {
	CODA_IMX27,
	CODA_IMX53,
	CODA_S3C64XX,
};

#define CODA7_MAX_MB_X		(1920 / 16)
#define CODA7_MAX_MB_Y		(1088 / 16)

static const struct coda_devtype coda_devdata[] = {
	[CODA_IMX27] = {
		.firmware = "v4l-codadx6-imx27.bin",
		.product = CODA_DX6,
		.codecs = codadx6_codecs,
		.num_codecs = ARRAY_SIZE(codadx6_codecs),
		.workbuf_size = CODADX6_WORK_BUF_SIZE,
		.ext_iram_size = CODADX6_IRAM_SIZE,
		.stream_end_bit = CODA_BIT_STREAM_END_OFFSET,
		.stream_pic_flush_bit = CODADX6_STREAM_BUF_PIC_FLUSH_OFFSET,
		.stream_pic_reset_bit = CODADX6_STREAM_BUF_PIC_RESET_OFFSET,
		.stream_buf_dynalloc_bit = CODADX6_STREAM_BUF_DYNALLOC_EN_OFFSET,
		.option_gamma_bit = CODADX6_OPTION_GAMMA_OFFSET,
		.has_reg_idx = true,
		.has_slice_buf = true,
		.has_gamma_ctl = true,
	},
	[CODA_IMX53] = {
		.firmware = "v4l-coda7541-imx53.bin",
		.product = CODA_7541,
		.codecs = coda7_codecs,
		.num_codecs = ARRAY_SIZE(coda7_codecs),
		.tempbuf_size = CODA7_TEMP_BUF_SIZE,
		.ext_iram_size = CODA7_IRAM_SIZE,
		.psbuf_size = CODA7_PS_BUF_SIZE,
		.max_mb_x = CODA7_MAX_MB_X,
		.max_mb_y = CODA7_MAX_MB_Y,
		.stream_end_bit = CODA_BIT_STREAM_END_OFFSET,
		.stream_pic_flush_bit = CODA7_STREAM_BUF_PIC_FLUSH_OFFSET,
		.stream_pic_reset_bit = CODA7_STREAM_BUF_PIC_RESET_OFFSET,
		.stream_buf_dynalloc_bit = CODA7_STREAM_BUF_DYNALLOC_EN_OFFSET,
		.option_gamma_bit = CODA7_OPTION_GAMMA_OFFSET,
		.has_slice_buf = true,
		.has_ps_buf = true,
		.has_frm_dis = true,
		.has_aux_std = true,
		.has_swapped_parabuf = true,
		.has_mvcol_buf = true,
		.has_gamma_ctl = true,
		.has_reorder = true,
	},
	[CODA_S3C64XX] = {
		.firmware = "v4l-mfc-s3c64xx.bin",
		.product = CODA_MFC_V1,
		.codecs = mfcv1_codecs,
		.num_codecs = ARRAY_SIZE(mfcv1_codecs),
		.workbuf_size = CODAMFC_WORK_BUF_SIZE,
		.psbuf_size = CODAMFC_PS_BUF_SIZE,
		.stream_end_bit = CODAMFC_BIT_STREAM_END_OFFSET,
		.stream_pic_flush_bit = CODADX6_STREAM_BUF_PIC_FLUSH_OFFSET,
		.stream_pic_reset_bit = CODADX6_STREAM_BUF_PIC_RESET_OFFSET,
		.stream_buf_dynalloc_bit = CODADX6_STREAM_BUF_DYNALLOC_EN_OFFSET,
		.has_reg_idx = true,
		.has_swapped_parabuf = true,
		.has_reorder = true,
	},
};

static struct platform_device_id coda_platform_ids[] = {
	{ .name = "coda-imx27", .driver_data = CODA_IMX27 },
	{ .name = "coda-imx53", .driver_data = CODA_IMX53 },
	{ .name = "coda-s3c6400", .driver_data = CODA_S3C64XX },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, coda_platform_ids);

#ifdef CONFIG_OF
static const struct of_device_id coda_dt_ids[] = {
	{ .compatible = "fsl,imx27-vpu", .data = &coda_devdata[CODA_IMX27] },
	{ .compatible = "fsl,imx53-vpu", .data = &coda_devdata[CODA_IMX53] },
	{ .compatible = "samsung,s3c6400-mfc",
					.data = &coda_devdata[CODA_S3C64XX] },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, coda_dt_ids);
#endif

static int coda_alloc_memory(struct coda_dev *dev)
{
	struct coda_platform_data *pdata = dev->dev->platform_data;
	struct device_node *np = dev->dev->of_node;
	int ret;

	/* allocate auxiliary per-device buffers for the BIT processor */
	if (dev->devtype->workbuf_size) {
		ret = coda_alloc_aux_buf(dev, &dev->workbuf,
						dev->devtype->workbuf_size);
		if (ret < 0) {
			dev_err(dev->dev, "failed to allocate work buffer\n");
			return ret;
		}
	}

	if (dev->devtype->tempbuf_size) {
		ret = coda_alloc_aux_buf(dev, &dev->tempbuf,
						dev->devtype->tempbuf_size);
		if (ret < 0) {
			dev_err(dev->dev, "failed to allocate temp buffer\n");
			goto err_free_workbuf;
		}
	}

	if (dev->devtype->ext_iram_size) {
		struct gen_pool *pool;

		/* Get IRAM pool from device tree or platform data */
		pool = of_get_named_gen_pool(np, "iram", 0);
		if (!pool && pdata)
			pool = dev_get_gen_pool(pdata->iram_dev);
		if (!pool) {
			dev_err(dev->dev, "iram pool not available\n");
			ret = -ENOMEM;
			goto err_free_tempbuf;
		}
		dev->iram_pool = pool;

		dev->iram_vaddr = gen_pool_alloc(dev->iram_pool,
						dev->devtype->ext_iram_size);
		if (!dev->iram_vaddr) {
			dev_err(dev->dev, "unable to alloc iram\n");
			ret = -ENOMEM;
			goto err_free_tempbuf;
		}
		dev->iram_paddr = gen_pool_virt_to_phys(dev->iram_pool,
							dev->iram_vaddr);
	}

	return 0;

err_free_tempbuf:
	coda_free_aux_buf(dev, &dev->tempbuf);
err_free_workbuf:
	coda_free_aux_buf(dev, &dev->workbuf);

	return ret;
}

static int coda_v4l2_register(struct coda_dev *dev)
{
	int i, ret;

	ret = v4l2_device_register(dev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(dev->dev, "failed to register V4L2 device\n");
		return ret;
	}

	dev->alloc_ctx = vb2_dma_contig_init_ctx(dev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		v4l2_err(&dev->v4l2_dev, "failed to alloc vb2 context\n");
		goto err_v4l2_unreg;
	}

	dev->m2m_dev = v4l2_m2m_init(&coda_m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		v4l2_err(&dev->v4l2_dev, "failed to init mem2mem device\n");
		ret = PTR_ERR(dev->m2m_dev);
		goto err_free_dma_contig;
	}

	for (i = 0; i < ARRAY_SIZE(coda_video_devs); ++i) {
		ret = coda_register_vdev(dev, i);
		if (ret)
			goto err_unreg_vdev;
	}

	return 0;

err_unreg_vdev:
	for (--i; i >= 0; --i)
		video_unregister_device(&dev->vfd[i]);
err_free_dma_contig:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
err_v4l2_unreg:
	v4l2_device_unregister(&dev->v4l2_dev);

	return ret;
}

static int coda_probe(struct platform_device *pdev)
{
	const struct platform_device_id *pdev_id;
	struct device_node *np = pdev->dev.of_node;
	struct coda_dev *dev;
	struct resource *res;
	int ret, irq;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	pdev_id = platform_get_device_id(pdev);
	if (pdev_id)
		dev->devtype = &coda_devdata[pdev_id->driver_data];
	if (!dev->devtype && IS_ENABLED(CONFIG_OF) && np) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(coda_dt_ids), &pdev->dev);
		dev->devtype = match->data;
	}
	if (!dev->devtype)
		return -ENODEV;

	dev->dev = &pdev->dev;

	spin_lock_init(&dev->irqlock);
	INIT_LIST_HEAD(&dev->instances);
	INIT_DELAYED_WORK(&dev->timeout, coda_timeout);
	mutex_init(&dev->dev_mutex);
	mutex_init(&dev->coda_mutex);

	dev->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(dev->clk_per)) {
		dev_err(&pdev->dev, "Could not get per clock\n");
		return PTR_ERR(dev->clk_per);
	}

	dev->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(dev->clk_ahb)) {
		dev_err(&pdev->dev, "Could not get ahb clock\n");
		return PTR_ERR(dev->clk_ahb);
	}

	dev->clk_spec = devm_clk_get(&pdev->dev, "special");
	if (!IS_ERR(dev->clk_spec))
		clk_set_rate(dev->clk_spec, 133000000);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->regs_base))
		return PTR_ERR(dev->regs_base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get irq resource\n");
		return -ENOENT;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
					coda_irq_handler, IRQF_ONESHOT,
					pdev->name, dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request irq\n");
		return -ENOENT;
	}

	ret = coda_alloc_memory(dev);
	if (ret)
		return ret;

	ret = coda_v4l2_register(dev);
	if (ret)
		goto err_free_mem;

	platform_set_drvdata(pdev, dev);

	pm_runtime_enable(&pdev->dev);

	return 0;

err_free_mem:
	if (dev->iram_vaddr)
		gen_pool_free(dev->iram_pool, dev->iram_vaddr, dev->iram_size);
	coda_free_aux_buf(dev, &dev->tempbuf);
	coda_free_aux_buf(dev, &dev->workbuf);

	return ret;
}

static int coda_remove(struct platform_device *pdev)
{
	struct coda_dev *dev = platform_get_drvdata(pdev);
	int i;

	pm_runtime_disable(&pdev->dev);

	for (i = 0; i < ARRAY_SIZE(coda_video_devs); ++i)
		video_unregister_device(&dev->vfd[i]);

	v4l2_m2m_release(dev->m2m_dev);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	v4l2_device_unregister(&dev->v4l2_dev);

	if (dev->iram_vaddr)
		gen_pool_free(dev->iram_pool, dev->iram_vaddr, dev->iram_size);

	coda_free_aux_buf(dev, &dev->codebuf);
	coda_free_aux_buf(dev, &dev->tempbuf);
	coda_free_aux_buf(dev, &dev->workbuf);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int coda_suspend(struct device *dev)
{
	return -EINVAL;
}

static int coda_resume(struct device *dev)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int coda_runtime_suspend(struct device *dev)
{
	return 0;
}

static int coda_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops coda_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(coda_suspend, coda_resume)
	SET_RUNTIME_PM_OPS(coda_runtime_suspend, coda_runtime_resume, NULL)
};

static struct platform_driver coda_driver = {
	.probe	= coda_probe,
	.remove	= coda_remove,
	.driver	= {
		.name = CODA_NAME,
		.owner = THIS_MODULE,
		.pm = &coda_pm_ops,
		.of_match_table = of_match_ptr(coda_dt_ids),
	},
	.id_table = coda_platform_ids,
};
module_platform_driver(coda_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("CODA multi-standard codec V4L2 driver (coda-ng)");
