/*
 * Samsung S3C PP - Video Postprocessor Driver
 *
 * Copyright (c) 2013 Tomasz Figa <tomasz.figa@gmail.com>
 *
 * Based on: Samsung S5P G2D - 2D Graphics Accelerator Driver
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Kamil Debski, <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#define fh2ctx(__fh) container_of(__fh, struct pp_ctx, fh)

#define PP_NAME "s3c-pp"

/* Special function registers */
#define MODE_REG			0x00
#define POSTENVID_REG			0x9c
#define MODE_2_REG			0xa0

/* Scaling control registers */
#define PRESCALE_RATIO_REG		0x04
#define PRESCALE_IMG_SIZE_REG		0x08
#define SRC_IMG_SIZE_REG		0x0c
#define MAIN_SCALE_H_RATIO_REG		0x10
#define MAIN_SCALE_V_RATIO_REG		0x14
#define DST_IMG_SIZE_REG		0x18
#define PRESCALE_SH_FACTOR_REG		0x1c

/* Buffer configuration registers */
#define ADDR_START_Y_REG		0x20
#define ADDR_START_CB_REG		0x24
#define ADDR_START_CR_REG		0x28
#define ADDR_START_RGB_REG		0x2c
#define ADDR_END_Y_REG			0x30
#define ADDR_END_CB_REG			0x34
#define ADDR_END_CR_REG			0x38
#define ADDR_END_RGB_REG		0x3c
#define OFFSET_Y_REG			0x40
#define OFFSET_CB_REG			0x44
#define OFFSET_CR_REG			0x48
#define OFFSET_RGB_REG			0x4c
#define NXT_ADDR_START_Y_REG		0x54
#define NXT_ADDR_START_CB_REG		0x58
#define NXT_ADDR_START_CR_REG		0x5c
#define NXT_ADDR_START_RGB_REG		0x60
#define NXT_ADDR_END_Y_REG		0x64
#define NXT_ADDR_END_CB_REG		0x68
#define NXT_ADDR_END_CR_REG		0x6c
#define NXT_ADDR_END_RGB_REG		0x70
#define ADDR_START_O_CB_REG		0x74
#define ADDR_START_O_CR_REG		0x78
#define ADDR_END_O_CB_REG		0x7c
#define ADDR_END_O_CR_REG		0x80
#define OFFSET_O_CB_REG			0x84
#define OFFSET_O_CR_REG			0x88
#define NXT_ADDR_START_O_CB_REG		0x8c
#define NXT_ADDR_START_O_CR_REG		0x90
#define NXT_ADDR_END_O_CB_REG		0x94
#define NXT_ADDR_END_O_CR_REG		0x98

/* MODE_REG bitfields */
#define MODE_IN_FORMAT_YCYC		(1 << 0)
#define MODE_IN_RGB_FORMAT		(1 << 1)
#define MODE_INTERLEAVE			(1 << 2)
#define MODE_IN_RGB			(1 << 3)
#define MODE_OUT_RGB_FORMAT		(1 << 4)
#define MODE_IRQ_LEVEL			(1 << 5)
#define MODE_POSTINT			(1 << 6)
#define MODE_INTEN			(1 << 7)
#define MODE_SRC420			(1 << 8)
#define MODE_COLOR_SPACE_MASK		(3 << 10)
#define MODE_COLOR_SPACE_NARROW		(1 << 10)
#define MODE_COLOR_SPACE_WIDE		(2 << 10)
#define MODE_INTERLACE			(1 << 12)
#define MODE_LCD_PATH_ENABLE		(1 << 13)
#define MODE_AUTO_LOAD_ENABLE		(1 << 14)
#define MODE_IN_FORMAT_CBCR		(1 << 15)
#define MODE_R2Y_SEL			(1 << 16)
#define MODE_DST420			(1 << 17)
#define MODE_OUT_RGB			(1 << 18)
#define MODE_OUT_YCBCR_FORMAT_MASK	(3 << 19)
#define MODE_OUT_YCBCR_FORMAT_CRYCBY	(0 << 19)
#define MODE_OUT_YCBCR_FORMAT_YCRYCB	(1 << 19)
#define MODE_OUT_YCBCR_FORMAT_CBYCRY	(2 << 19)
#define MODE_OUT_YCBCR_FORMAT_YCBYCR	(3 << 19)
#define MODE_CLKSEL_F_MASK		(3 << 21)
#define MODE_CLKSEL_F_VAL(x)		((x) << 21)
#define MODE_CLKDIR			(1 << 23)
#define MODE_CLKVAL_F_MASK		(0x3f << 24)
#define MODE_CLKVAL_F_VAL(x)		((x) << 24)
#define MODE_CLKVALUP			(1 << 30)

#define MODE_SRC_FORMAT_MASK		(\
		MODE_IN_FORMAT_YCYC | MODE_IN_RGB_FORMAT | MODE_INTERLEAVE | \
		MODE_IN_RGB | MODE_SRC420 | MODE_IN_FORMAT_CBCR)
#define MODE_DST_FORMAT_MASK		(\
		MODE_OUT_RGB_FORMAT | MODE_DST420 | MODE_OUT_RGB | \
		MODE_OUT_YCBCR_FORMAT_MASK)

/* Scaler registers */
#define PRESCALE_RATIO_H_MASK		(0x7f << 0)
#define PRESCALE_RATIO_H_VAL(x)		((x) << 0)
#define PRESCALE_RATIO_V_MASK		(0x7f << 7)
#define PRESCALE_RATIO_V_VAL(x)		((x) << 7)
#define IMG_SIZE_WIDTH_MASK		(0xfff << 0)
#define IMG_SIZE_WIDTH_VAL(x)		((x) << 0)
#define IMG_SIZE_HEIGHT_MASK		(0xfff << 12)
#define IMG_SIZE_HEIGHT_VAL(x)		((x) << 12)

/* POSTENVID_REG bitfields */
#define POSTENVID_EN			(1 << 31)

/* MODE_2_REG bitfields */
#define MODE_2_BC_SEL			(1 << 3)
#define MODE_2_ADDR_CH_DIS		(1 << 4)

/* Hardware limits */
#define MAX_SRC_WIDTH		4096
#define MAX_SRC_HEIGHT		4096
#define MAX_DST_WIDTH		2048
#define MAX_DST_HEIGHT		2048

#define PP_TIMEOUT		500

#define DEFAULT_WIDTH		720
#define DEFAULT_HEIGHT		576

#define PP_H_SHIFT_MAX		5
#define PP_V_SHIFT_MAX		5

#define PP_MAX_PLANES		3

#define PP_MAX_CLOCKS		4

struct pp_dev {
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device vfd;
	struct vb2_alloc_ctx *alloc_ctx;

	struct device *dev;
	void __iomem *regs;
	struct clk *vid_clks[PP_MAX_CLOCKS];
	struct clk *clk;
	struct clk *gate;
	int irq;

	wait_queue_head_t irq_queue;
	struct mutex mutex;
	struct pp_ctx *curr;
	u32 mode_val;
};

struct pp_frame {
	/* Original dimensions */
	u32 width;
	u32 height;
	/* Crop size */
	u32 c_width;
	u32 c_height;
	/* Offset */
	u32 left;
	u32 top;
	/* Memplane size */
	u32 sizeimage[PP_MAX_PLANES];
	/* Image format */
	struct pp_fmt *fmt;
	enum v4l2_colorspace color_space;
	/* Variables that can calculated once and reused */
	u32 bytes_per_line[PP_MAX_PLANES];
	u32 offset[PP_MAX_PLANES];
	u32 skip[PP_MAX_PLANES];
	u32 end[PP_MAX_PLANES];
	u32 cplane_offset[PP_MAX_PLANES];
};

struct pp_scaler {
	u16 prescaled_dst_w;
	u16 prescaled_dst_h;
	u16 v_ratio;
	u16 h_ratio;
	u8 prescale_v;
	u8 prescale_h;
	u8 sh_factor;
};

struct pp_ctx {
	struct v4l2_fh		fh;
	struct pp_dev		*dev;
	struct v4l2_m2m_ctx	*m2m_ctx;
	struct pp_frame		in;
	struct pp_frame		out;
	struct pp_scaler	scaler;
};

struct pp_fmt {
	const char *name;
	u32 fourcc;
	u32 hw_src;
	u32 hw_dst;
	u16 memplanes;
	u16 colplanes;
	u8 depth[PP_MAX_PLANES];
	u8 cplane_wshift[PP_MAX_PLANES];
	u8 cplane_hshift[PP_MAX_PLANES];
	u8 ybpp_shift;
};

/*
 * Supported formats
 */
static struct pp_fmt formats[] = {
	{
		.name		= "YUV 4:2:0 planar, YCbCr",
		.fourcc		= V4L2_PIX_FMT_YUV420,
		.hw_src		= MODE_SRC420 | MODE_IN_RGB_FORMAT,
		.hw_dst		= MODE_DST420,
		.depth		= { 12 },
		.cplane_wshift	= { 0, 1, 1 },
		.cplane_hshift	= { 0, 1, 1 },
		.ybpp_shift	= 0,
		.memplanes	= 1,
		.colplanes	= 3,
	}, {
		.name		= "YUV 4:2:0 non-contig. 3p, Y/Cb/Cr",
		.fourcc		= V4L2_PIX_FMT_YUV420M,
		.hw_src		= MODE_SRC420 | MODE_IN_RGB_FORMAT,
		.hw_dst		= MODE_DST420,
		.depth		= { 8, 2, 2 },
		.cplane_wshift	= { 0, 1, 1 },
		.cplane_hshift	= { 0, 1, 1 },
		.ybpp_shift	= 0,
		.memplanes	= 3,
		.colplanes	= 3,
	}, {
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.hw_src		= MODE_INTERLEAVE | MODE_IN_RGB_FORMAT |
					MODE_IN_FORMAT_YCYC |
					MODE_IN_FORMAT_CBCR,
		.hw_dst		= MODE_OUT_YCBCR_FORMAT_YCBYCR,
		.depth		= { 16 },
		.ybpp_shift	= 1,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "YUV 4:2:2 packed, CbYCrY",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.hw_src		= MODE_INTERLEAVE | MODE_IN_RGB_FORMAT |
					MODE_IN_FORMAT_CBCR,
		.hw_dst		= MODE_OUT_YCBCR_FORMAT_CBYCRY,
		.depth		= { 16 },
		.ybpp_shift	= 1,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "YUV 4:2:2 packed, CrYCbY",
		.fourcc		= V4L2_PIX_FMT_VYUY,
		.hw_src		= MODE_INTERLEAVE | MODE_IN_RGB_FORMAT,
		.hw_dst		= MODE_OUT_YCBCR_FORMAT_CRYCBY,
		.depth		= { 16 },
		.ybpp_shift	= 1,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "YUV 4:2:2 packed, YCrYCb",
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.hw_src		= MODE_INTERLEAVE | MODE_IN_RGB_FORMAT |
					MODE_IN_FORMAT_YCYC,
		.hw_dst		= MODE_OUT_YCBCR_FORMAT_YCRYCB,
		.depth		= { 16 },
		.ybpp_shift	= 1,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "ARGB8888, 32 bpp",
		.fourcc		= V4L2_PIX_FMT_RGB32,
		.hw_src		= MODE_IN_RGB | MODE_INTERLEAVE |
					MODE_IN_RGB_FORMAT,
		.hw_dst		= MODE_OUT_RGB | MODE_OUT_RGB_FORMAT,
		.depth		= { 32 },
		.ybpp_shift	= 2,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "RGB565",
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.hw_src		= MODE_IN_RGB | MODE_INTERLEAVE,
		.hw_dst		= MODE_OUT_RGB,
		.depth		= { 16 },
		.ybpp_shift	= 1,
		.memplanes	= 1,
		.colplanes	= 1,
	}
};
#define NUM_FORMATS ARRAY_SIZE(formats)

static struct pp_frame def_frame = {
	.width		= DEFAULT_WIDTH,
	.height		= DEFAULT_HEIGHT,
	.c_width	= DEFAULT_WIDTH,
	.c_height	= DEFAULT_HEIGHT,
	.left		= 0,
	.top		= 0,
	.fmt		= &formats[0],
};

/*
 * Hardware configuration
 */
static inline void pp_write(struct pp_dev *d, u32 val, u32 reg)
{
	writel(val, d->regs + reg);
}

static inline u32 pp_read(struct pp_dev *d, u32 reg)
{
	return readl(d->regs + reg);
}

static void pp_set_src(struct pp_dev *d,
		       struct pp_frame *f, struct vb2_buffer *vb)
{
	u32 reg, start, end;

	d->mode_val &= ~MODE_SRC_FORMAT_MASK;
	d->mode_val |= f->fmt->hw_src;

	switch (f->color_space) {
	case V4L2_COLORSPACE_JPEG:
	case V4L2_COLORSPACE_SRGB:
		d->mode_val |= MODE_R2Y_SEL;
		break;
	default:
		d->mode_val &= ~MODE_R2Y_SEL;
	}

	reg = IMG_SIZE_WIDTH_VAL(f->c_width) |
		IMG_SIZE_HEIGHT_VAL(f->c_height);
	pp_write(d, reg, SRC_IMG_SIZE_REG);

	switch (f->fmt->colplanes) {
	case 3:
		if (f->fmt->memplanes >= 3)
			start = vb2_dma_contig_plane_dma_addr(vb, 2);
		else
			start = vb2_dma_contig_plane_dma_addr(vb, 0) +
					f->cplane_offset[2];

		end = start + f->end[2];
		start = start + f->offset[2];

		pp_write(d, start, ADDR_START_CR_REG);
		pp_write(d, end, ADDR_END_CR_REG);
		pp_write(d, f->skip[2], OFFSET_CR_REG);

		/* Fall through */
	case 2:
		if (f->fmt->memplanes >= 2)
			start = vb2_dma_contig_plane_dma_addr(vb, 1);
		else
			start = vb2_dma_contig_plane_dma_addr(vb, 0) +
					f->cplane_offset[1];

		end = start + f->end[1];
		start = start + f->offset[1];

		pp_write(d, start, ADDR_START_CB_REG);
		pp_write(d, end, ADDR_END_CB_REG);
		pp_write(d, f->skip[1], OFFSET_CB_REG);

		/* Fall through */
	case 1:
		start = vb2_dma_contig_plane_dma_addr(vb, 0);

		end = start + f->end[0];
		start = start + f->offset[0];

		pp_write(d, start, ADDR_START_Y_REG);
		pp_write(d, end, ADDR_END_Y_REG);
		pp_write(d, f->skip[0], OFFSET_Y_REG);
	}
}

static void pp_set_dst(struct pp_dev *d,
		       struct pp_frame *f, struct vb2_buffer *vb)
{
	u32 reg, start, end;

	d->mode_val &= ~MODE_DST_FORMAT_MASK;
	d->mode_val |= f->fmt->hw_dst;

	d->mode_val &= ~MODE_COLOR_SPACE_MASK;
	switch (f->color_space) {
	case V4L2_COLORSPACE_JPEG:
	case V4L2_COLORSPACE_SRGB:
		d->mode_val |= MODE_COLOR_SPACE_WIDE;
		break;
	default:
		d->mode_val |= MODE_COLOR_SPACE_NARROW;
	}

	reg = IMG_SIZE_WIDTH_VAL(f->c_width) |
		IMG_SIZE_HEIGHT_VAL(f->c_height);
	pp_write(d, reg, DST_IMG_SIZE_REG);

	switch (f->fmt->colplanes) {
	case 3:
		if (f->fmt->memplanes >= 3)
			start = vb2_dma_contig_plane_dma_addr(vb, 2);
		else
			start = vb2_dma_contig_plane_dma_addr(vb, 0) +
					f->cplane_offset[2];

		end = start + f->end[2];
		start = start + f->offset[2];

		pp_write(d, start, ADDR_START_O_CR_REG);
		pp_write(d, end, ADDR_END_O_CR_REG);
		pp_write(d, f->skip[2], OFFSET_O_CR_REG);

		/* Fall through */
	case 2:
		if (f->fmt->memplanes >= 2)
			start = vb2_dma_contig_plane_dma_addr(vb, 1);
		else
			start = vb2_dma_contig_plane_dma_addr(vb, 0) +
					f->cplane_offset[1];

		end = start + f->end[1];
		start = start + f->offset[1];

		pp_write(d, start, ADDR_START_O_CB_REG);
		pp_write(d, end, ADDR_END_O_CB_REG);
		pp_write(d, f->skip[1], OFFSET_O_CB_REG);

		/* Fall through */
	case 1:
		start = vb2_dma_contig_plane_dma_addr(vb, 0);

		end = start + f->end[0];
		start = start + f->offset[0];

		pp_write(d, start, ADDR_START_RGB_REG);
		pp_write(d, end, ADDR_END_RGB_REG);
		pp_write(d, f->skip[0], OFFSET_RGB_REG);
	}
}

static void pp_set_scaler(struct pp_dev *d, struct pp_scaler *s)
{
	u32 reg;

	reg = PRESCALE_RATIO_V_VAL(s->prescale_v) |
		PRESCALE_RATIO_H_VAL(s->prescale_h);
	pp_write(d, reg, PRESCALE_RATIO_REG);

	reg = IMG_SIZE_WIDTH_VAL(s->prescaled_dst_w) |
		IMG_SIZE_HEIGHT_VAL(s->prescaled_dst_h);
	pp_write(d, reg, PRESCALE_IMG_SIZE_REG);

	pp_write(d, s->v_ratio, MAIN_SCALE_V_RATIO_REG);
	pp_write(d, s->h_ratio, MAIN_SCALE_H_RATIO_REG);
	pp_write(d, s->sh_factor, PRESCALE_SH_FACTOR_REG);
}

static void pp_start(struct pp_dev *d)
{
	pp_write(d, d->mode_val, MODE_REG);
	pp_write(d, POSTENVID_EN, POSTENVID_REG);
}

static void pp_clear_int(struct pp_dev *d)
{
	u32 reg;

	reg = pp_read(d, MODE_REG);
	reg &= ~MODE_POSTINT;
	pp_write(d, reg, MODE_REG);
}

static irqreturn_t pp_isr(int irq, void *prv)
{
	struct pp_dev *dev = prv;
	struct pp_ctx *ctx = dev->curr;
	struct vb2_buffer *src, *dst;

	pp_clear_int(dev);
	pm_runtime_put(dev->dev);

	BUG_ON(ctx == NULL);

	src = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);

	BUG_ON(src == NULL);
	BUG_ON(dst == NULL);

	dst->v4l2_buf.timecode = src->v4l2_buf.timecode;
	dst->v4l2_buf.timestamp = src->v4l2_buf.timestamp;

	v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	v4l2_m2m_job_finish(dev->m2m_dev, ctx->m2m_ctx);

	dev->curr = NULL;
	wake_up(&dev->irq_queue);

	return IRQ_HANDLED;
}

/*
 * Helpers
 */
static struct pp_fmt *find_fmt(struct v4l2_format *f)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == f->fmt.pix.pixelformat)
			return &formats[i];
	}

	return NULL;
}

static struct pp_frame *get_frame(struct pp_ctx *ctx, enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return &ctx->in;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		return &ctx->out;
	default:
		return ERR_PTR(-EINVAL);
	}
}

/*
 * Videobuf2
 */
static int pp_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], void *alloc_ctxs[])
{
	struct pp_ctx *ctx = vb2_get_drv_priv(vq);
	struct pp_frame *f = get_frame(ctx, vq->type);
	u32 offset;
	int i;

	if (IS_ERR(f))
		return PTR_ERR(f);

	/*
	 * Return number of non-contigous planes (plane buffers)
	 * depending on the configured color format.
	 */
	if (!f->fmt)
		return -EINVAL;

	*nplanes = f->fmt->memplanes;
	for (i = 0; i < f->fmt->memplanes; i++) {
		sizes[i] = (f->width * f->height * f->fmt->depth[i]) / 8;
		alloc_ctxs[i] = ctx->dev->alloc_ctx;
		dev_dbg(ctx->dev->dev, "plane %d: size = %u\n", i, sizes[i]);
	}

	if (f->fmt->colplanes == f->fmt->memplanes)
		return 0;

	for (offset = 0, i = 0; i < f->fmt->colplanes; ++i) {
		u32 w, h;

		w = f->width >> f->fmt->cplane_wshift[i];
		h = f->height >> f->fmt->cplane_hshift[i];

		f->cplane_offset[i] = offset;
		offset += (w * h) << f->fmt->ybpp_shift;

		dev_dbg(ctx->dev->dev, "plane %d: cplane_offset = %u\n",
			i, f->cplane_offset[i]);
	}

	return 0;
}

static int pp_buf_prepare(struct vb2_buffer *vb)
{
	struct pp_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct pp_frame *f = get_frame(ctx, vb->vb2_queue->type);
	int i;

	if (IS_ERR(f))
		return PTR_ERR(f);

	for (i = 0; i < f->fmt->memplanes; i++)
		vb2_set_plane_payload(vb, i, f->sizeimage[i]);

	return 0;
}

static void pp_buf_queue(struct vb2_buffer *vb)
{
	struct pp_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);
}

static struct vb2_ops pp_qops = {
	.queue_setup	= pp_queue_setup,
	.buf_prepare	= pp_buf_prepare,
	.buf_queue	= pp_buf_queue,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
						struct vb2_queue *dst_vq)
{
	struct pp_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &pp_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &pp_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	return vb2_queue_init(dst_vq);
}

/*
 * V4L2 file operations
 */
static int pp_open(struct file *file)
{
	struct pp_dev *dev = video_drvdata(file);
	struct pp_ctx *ctx = NULL;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	/* Set default formats */
	ctx->in		= def_frame;
	ctx->out	= def_frame;

	if (mutex_lock_interruptible(&dev->mutex)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}

	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		mutex_unlock(&dev->mutex);
		kfree(ctx);
		return ret;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	mutex_unlock(&dev->mutex);

	v4l2_info(&dev->v4l2_dev, "instance opened\n");

	return 0;
}

static int pp_release(struct file *file)
{
	struct pp_dev *dev = video_drvdata(file);
	struct pp_ctx *ctx = fh2ctx(file->private_data);

	mutex_lock(&dev->mutex);

	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	mutex_unlock(&dev->mutex);

	v4l2_info(&dev->v4l2_dev, "instance closed\n");

	return 0;
}

static unsigned int pp_poll(struct file *file, struct poll_table_struct *wait)
{
	struct pp_ctx *ctx = fh2ctx(file->private_data);
	struct pp_dev *dev = ctx->dev;
	unsigned int res;

	mutex_lock(&dev->mutex);
	res = v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
	mutex_unlock(&dev->mutex);

	return res;
}

static int pp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct pp_ctx *ctx = fh2ctx(file->private_data);
	struct pp_dev *dev = ctx->dev;
	int ret;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	ret = v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);

	mutex_unlock(&dev->mutex);

	return ret;
}

static const struct v4l2_file_operations pp_fops = {
	.owner		= THIS_MODULE,
	.open		= pp_open,
	.release	= pp_release,
	.poll		= pp_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= pp_mmap,
};

/*
 * V4L2 IOCTLs
 */
static int pp_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strncpy(cap->driver, PP_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, PP_NAME, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(1, 0, 0);

	/*
	 * This is only a mem-to-mem video device. The capture and output
	 * device capability flags are left only for backward compatibility
	 * and are scheduled for removal.
	 */
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE |
		V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE;

	return 0;
}

static int pp_enum_fmt_mplane(struct file *file, void *prv,
			      struct v4l2_fmtdesc *f)
{
	struct pp_fmt *fmt;

	if (f->index >= NUM_FORMATS)
		return -EINVAL;

	fmt = &formats[f->index];
	f->pixelformat = fmt->fourcc;
	strncpy(f->description, fmt->name, sizeof(f->description) - 1);

	return 0;
}

static int pp_g_fmt_mplane(struct file *file, void *prv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixm = &f->fmt.pix_mp;
	struct pp_ctx *ctx = prv;
	struct pp_frame *frame;
	int i;

	frame = get_frame(ctx, f->type);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

	pixm->width = frame->width;
	pixm->height = frame->height;
	pixm->field = V4L2_FIELD_NONE;
	pixm->pixelformat = frame->fmt->fourcc;
	pixm->colorspace = frame->color_space;
	pixm->num_planes = frame->fmt->memplanes;

	for (i = 0; i < pixm->num_planes; ++i) {
		pixm->plane_fmt[i].bytesperline = frame->bytes_per_line[i];
		pixm->plane_fmt[i].sizeimage = frame->sizeimage[i];
	}

	return 0;
}

static void pp_bound_align_image(struct pp_fmt *fmt, s32 *left, s32 *top,
				 s32 *width, s32 *height)
{
	u32 w_align = 1, h_align_sh = 0, w_align_sh;
	s32 orig_left = *left, orig_top = *top;
	int i;

	/* The hardware requires lines to be word-aligned. */
	for (i = 0; i < fmt->colplanes; ++i) {
		u32 pix_in_word;

		pix_in_word = (4 >> fmt->ybpp_shift) << fmt->cplane_wshift[i];
		if (pix_in_word > w_align)
			w_align = pix_in_word;

		if (fmt->cplane_hshift[i] > h_align_sh)
			h_align_sh = fmt->cplane_hshift[i];
	}

	w_align_sh = fls(w_align) - 1;

	v4l_bound_align_image(left, *left, *left + *width, w_align_sh,
				top, *top, *top + *height, h_align_sh, 0);

	*width -= (*left - orig_left);
	*height -= (*top - orig_top);

	v4l_bound_align_image(width, 1, *width, w_align_sh,
				height, 1, *height, h_align_sh, 0);
}

static int pp_try_fmt_mplane(struct file *file, void *prv,
			     struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct pp_fmt *fmt;
	int i;
	s32 top = 0, left = 0;

	fmt = find_fmt(f);
	if (!fmt)
		return -EINVAL;

	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (pix->width > MAX_SRC_WIDTH)
			pix->width = MAX_SRC_WIDTH;
		if (pix->height > MAX_SRC_HEIGHT)
			pix->height = MAX_SRC_HEIGHT;
	} else {
		if (pix->width > MAX_DST_WIDTH)
			pix->width = MAX_DST_WIDTH;
		if (pix->height > MAX_DST_HEIGHT)
			pix->height = MAX_DST_HEIGHT;
	}

	if (pix->width < 1)
		pix->width = 1;
	if (pix->height < 1)
		pix->height = 1;

	pp_bound_align_image(fmt, &left, &top, &pix->width, &pix->height);

	pix->num_planes = fmt->memplanes;
	for (i = 0; i < pix->num_planes; ++i) {
		struct v4l2_plane_pix_format *p_fmt = &pix->plane_fmt[i];
		u32 bpl = p_fmt->bytesperline;
		u32 w = pix->width >> fmt->cplane_wshift[i];
		u32 h = pix->height >> fmt->cplane_hshift[i];

		if (!bpl || (w << fmt->ybpp_shift) > bpl)
			bpl = w << fmt->ybpp_shift;

		p_fmt->bytesperline = bpl;
		p_fmt->sizeimage = max(p_fmt->sizeimage,
					(w * h * fmt->depth[i]) / 8);
	}

	return 0;
}

static int pp_s_fmt_mplane(struct file *file, void *prv, struct v4l2_format *f)
{
	struct pp_ctx *ctx = prv;
	struct pp_dev *dev = ctx->dev;
	u32 bytes[PP_MAX_PLANES];
	struct vb2_queue *vq;
	struct pp_frame *frm;
	struct pp_fmt *fmt;
	int ret;
	int i;

	/*
	 * Adjust all values accordingly to the hardware capabilities
	 * and chosen format.
	 */
	ret = pp_try_fmt_mplane(file, prv, f);
	if (ret)
		return ret;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (vb2_is_busy(vq)) {
		v4l2_err(&dev->v4l2_dev, "queue (%d) bust\n", f->type);
		return -EBUSY;
	}

	frm = get_frame(ctx, f->type);
	if (IS_ERR(frm))
		return PTR_ERR(frm);

	fmt = find_fmt(f);
	if (!fmt)
		return -EINVAL;

	frm->width = f->fmt.pix_mp.width;
	frm->height = f->fmt.pix_mp.height;
	frm->color_space = f->fmt.pix_mp.colorspace;
	frm->fmt = fmt;

	/* Reset crop settings */
	frm->left = 0;
	frm->top = 0;
	frm->c_width = frm->width;
	frm->c_height = frm->height;

	for (i = 0; i < fmt->memplanes; ++i) {
		struct v4l2_plane_pix_format *p_fmt;
		u32 c_h;

		p_fmt = &f->fmt.pix_mp.plane_fmt[i];
		c_h = frm->c_height >> fmt->cplane_hshift[i];

		frm->bytes_per_line[i] = p_fmt->bytesperline;
		frm->sizeimage[i] = p_fmt->sizeimage;
		bytes[i] = frm->bytes_per_line[i] * c_h;
	}

	for (; i < fmt->colplanes; ++i) {
		u32 c_h;

		c_h = frm->c_height >> fmt->cplane_hshift[i];

		frm->bytes_per_line[i] = frm->bytes_per_line[0]
						>> frm->fmt->cplane_wshift[i];
		bytes[i] = frm->bytes_per_line[i] * c_h;
	}

	for (i = 0; i < fmt->colplanes; ++i) {
		u32 c_w;

		c_w = frm->c_width >> fmt->cplane_wshift[i];

		frm->offset[i] = 0;
		frm->skip[i] = frm->bytes_per_line[i] -
				(c_w << fmt->ybpp_shift);
		frm->end[i] = bytes[i] - frm->skip[i];

		dev_dbg(ctx->dev->dev, "plane %d: bytesperline = %u, bytes = %u\n",
			i, frm->bytes_per_line[i], bytes[i]);
		dev_dbg(ctx->dev->dev, "offset = %u, skip = %u, end = %u\n",
			frm->offset[i], frm->skip[i], frm->end[i]);
	}

	return 0;
}

static int pp_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *reqbufs)
{
	struct pp_ctx *ctx = priv;
	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

static int pp_querybuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct pp_ctx *ctx = priv;
	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);
}

static int pp_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct pp_ctx *ctx = priv;
	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int pp_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct pp_ctx *ctx = priv;
	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int pp_prepare_scaling_factors(struct pp_dev *d, struct pp_ctx *ctx)
{
	struct pp_frame *src = &ctx->in, *dst = &ctx->out;
	struct pp_scaler *sc = &ctx->scaler;
	u8 h_shift, v_shift;

	for (h_shift = 0; h_shift <= PP_H_SHIFT_MAX; ++h_shift)
		if (src->c_width < (dst->c_width << (h_shift + 1)))
			break;

	if (h_shift > PP_H_SHIFT_MAX)
		return -EINVAL;

	for (v_shift = 0; v_shift <= PP_V_SHIFT_MAX; ++v_shift)
		if (src->c_height < (dst->c_height << (v_shift + 1)))
			break;

	if (v_shift > PP_V_SHIFT_MAX)
		return -EINVAL;

	sc->prescale_h = (1 << h_shift);
	sc->prescale_v = (1 << v_shift);
	sc->sh_factor = 10 - h_shift - v_shift;
	sc->prescaled_dst_w = src->c_width >> h_shift;
	sc->prescaled_dst_h = src->c_height >> v_shift;
	sc->h_ratio = (src->c_width << 8) / (dst->c_width << h_shift);
	sc->v_ratio = (src->c_height << 8) / (dst->c_height << v_shift);

	return 0;
}

static int pp_streamon(struct file *file, void *priv,
					enum v4l2_buf_type type)
{
	struct pp_ctx *ctx = priv;
	int ret;

	ret = pp_prepare_scaling_factors(ctx->dev, ctx);
	if (ret)
		return ret;

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int pp_streamoff(struct file *file, void *priv,
					enum v4l2_buf_type type)
{
	struct pp_ctx *ctx = priv;
	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static int pp_cropcap(struct file *file, void *priv,
					struct v4l2_cropcap *cr)
{
	struct pp_ctx *ctx = priv;
	struct pp_frame *f;

	f = get_frame(ctx, cr->type);
	if (IS_ERR(f))
		return PTR_ERR(f);

	cr->bounds.left		= 0;
	cr->bounds.top		= 0;
	cr->bounds.width	= f->width;
	cr->bounds.height	= f->height;
	cr->defrect		= cr->bounds;

	return 0;
}

static int pp_g_crop(struct file *file, void *prv, struct v4l2_crop *cr)
{
	struct pp_ctx *ctx = prv;
	struct pp_frame *f;

	f = get_frame(ctx, cr->type);
	if (IS_ERR(f))
		return PTR_ERR(f);

	cr->c.left	= f->left;
	cr->c.top	= f->top;
	cr->c.width	= f->c_width;
	cr->c.height	= f->c_height;

	return 0;
}

static int pp_try_crop(struct pp_ctx *ctx, struct pp_frame *f,
		       struct v4l2_crop *cr)
{
	struct pp_dev *dev = ctx->dev;

	if (cr->c.top < 0 || cr->c.left < 0) {
		v4l2_err(&dev->v4l2_dev,
			"doesn't support negative values for top & left\n");
		return -EINVAL;
	}

	pp_bound_align_image(f->fmt, &cr->c.left, &cr->c.top,
				&cr->c.width, &cr->c.height);

	return 0;
}

static int pp_s_crop(struct file *file, void *prv, const struct v4l2_crop *crop)
{
	struct v4l2_crop cr = *crop;
	struct pp_ctx *ctx = prv;
	struct pp_frame *f;
	struct pp_fmt *fmt;
	int ret;
	int i;

	f = get_frame(ctx, cr.type);
	if (IS_ERR(f))
		return PTR_ERR(f);

	ret = pp_try_crop(ctx, f, &cr);
	if (ret)
		return ret;

	f->c_width = cr.c.width;
	f->c_height = cr.c.height;
	f->left = cr.c.left;
	f->top = cr.c.top;

	fmt = f->fmt;
	for (i = 0; i < f->fmt->colplanes; ++i) {
		u32 w, c_w, c_h, l, t;

		w = f->width >> fmt->cplane_wshift[i];
		c_w = f->c_width >> fmt->cplane_wshift[i];
		c_h = f->c_height >> fmt->cplane_hshift[i];
		l = f->left >> fmt->cplane_wshift[i];
		t = f->top >> fmt->cplane_hshift[i];

		f->skip[i] = f->bytes_per_line[i] - (c_w << fmt->ybpp_shift);
		f->offset[i] = f->bytes_per_line[i] * t +
				(l << fmt->ybpp_shift);
		f->end[i] = f->offset[i] + f->bytes_per_line[i] * c_h -
				f->skip[i];

		dev_dbg(ctx->dev->dev, "plane %d: offset = %u, skip = %u, end = %u\n",
			i, f->offset[i], f->skip[i], f->end[i]);
	}

	return 0;
}

static const struct v4l2_ioctl_ops pp_ioctl_ops = {
	.vidioc_querycap = pp_querycap,

	.vidioc_enum_fmt_vid_cap_mplane = pp_enum_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane = pp_g_fmt_mplane,
	.vidioc_try_fmt_vid_cap_mplane = pp_try_fmt_mplane,
	.vidioc_s_fmt_vid_cap_mplane = pp_s_fmt_mplane,

	.vidioc_enum_fmt_vid_out_mplane = pp_enum_fmt_mplane,
	.vidioc_g_fmt_vid_out_mplane = pp_g_fmt_mplane,
	.vidioc_try_fmt_vid_out_mplane = pp_try_fmt_mplane,
	.vidioc_s_fmt_vid_out_mplane = pp_s_fmt_mplane,

	.vidioc_reqbufs = pp_reqbufs,
	.vidioc_querybuf = pp_querybuf,

	.vidioc_qbuf = pp_qbuf,
	.vidioc_dqbuf = pp_dqbuf,

	.vidioc_streamon = pp_streamon,
	.vidioc_streamoff = pp_streamoff,

	.vidioc_g_crop = pp_g_crop,
	.vidioc_s_crop = pp_s_crop,
	.vidioc_cropcap = pp_cropcap,
};

/*
 * Mem-to-mem ops
 */
static void pp_job_abort(void *prv)
{
	struct pp_ctx *ctx = prv;
	struct pp_dev *dev = ctx->dev;
	int ret;

	if (dev->curr == NULL) /* No job currently running */
		return;

	ret = wait_event_timeout(dev->irq_queue,
		dev->curr == NULL, msecs_to_jiffies(PP_TIMEOUT));
	if (!ret) {
		struct vb2_buffer *src, *dst;

		pp_write(dev, 0, POSTENVID_REG);

		pm_runtime_put(dev->dev);

		src = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		dst = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);

		BUG_ON(src == NULL);
		BUG_ON(dst == NULL);

		dst->v4l2_buf.timecode = src->v4l2_buf.timecode;
		dst->v4l2_buf.timestamp = src->v4l2_buf.timestamp;

		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(dev->m2m_dev, ctx->m2m_ctx);

		dev->curr = NULL;
		wake_up(&dev->irq_queue);
	}
}

static void pp_device_run(void *prv)
{
	struct pp_ctx *ctx = prv;
	struct pp_dev *dev = ctx->dev;
	struct vb2_buffer *src, *dst;

	dev->curr = ctx;

	src = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);

	pm_runtime_get_sync(dev->dev);

	pp_set_scaler(dev, &ctx->scaler);
	pp_set_src(dev, &ctx->in, src);
	pp_set_dst(dev, &ctx->out, dst);
	pp_start(dev);
}

static const struct v4l2_m2m_ops pp_m2m_ops = {
	.device_run	= pp_device_run,
	.job_abort	= pp_job_abort,
};

/*
 * V4L2 video device
 */
static const struct video_device pp_videodev = {
	.fops		= &pp_fops,
	.ioctl_ops	= &pp_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release_empty,
	.vfl_dir	= VFL_DIR_M2M,
};

/*
 * Platform driver
 */
#ifdef CONFIG_OF
static const struct of_device_id pp_match[] = {
	{ .compatible = "samsung,s3c6400-pp", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pp_match);
#endif

static int pp_probe(struct platform_device *pdev)
{
	char clk_name[] = "vid_clkX";
	bool vid_clk_found = false;
	struct video_device *vfd;
	struct resource *res;
	struct pp_dev *dev;
	int ret;
	int i;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->mutex);
	init_waitqueue_head(&dev->irq_queue);
	dev->mode_val = MODE_INTEN | MODE_IRQ_LEVEL |
					MODE_CLKDIR | MODE_CLKVAL_F_VAL(1);
	dev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->regs))
		return PTR_ERR(dev->regs);

	for (i = 0; i < PP_MAX_CLOCKS; ++i) {
		snprintf(clk_name, sizeof(clk_name), "vid_clk%d", i);
		dev->vid_clks[i] = devm_clk_get(&pdev->dev, clk_name);
		if (!IS_ERR(dev->vid_clks[i]))
			vid_clk_found = true;
	}

	if (!vid_clk_found) {
		dev_err(&pdev->dev, "failed to get any video clock\n");
		return -EINVAL;
	}

	/* TODO: Select clock with best rate. */
	for (i = 0; i < PP_MAX_CLOCKS; ++i) {
		if (IS_ERR(dev->vid_clks[i]))
			continue;

		if (clk_get_rate(dev->vid_clks[i]) > 0) {
			dev->clk = dev->vid_clks[i];
			dev->mode_val |= MODE_CLKSEL_F_VAL(i);
			break;
		}
	}

	if (!dev->clk) {
		dev_err(&pdev->dev, "no valid video clocks found\n");
		return -EINVAL;
	}

	dev->gate = devm_clk_get(&pdev->dev, "bus_clk");
	if (IS_ERR(dev->gate)) {
		dev_err(&pdev->dev, "failed to get bus clock\n");
		return PTR_ERR(dev->gate);
	}

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "failed to find IRQ\n");
		return dev->irq;
	}

	ret = devm_request_irq(&pdev->dev, dev->irq, pp_isr,
						0, pdev->name, dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to install IRQ\n");
		return ret;
	}

	ret = clk_prepare_enable(dev->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare pp clock\n");
		return ret;
	}

	ret = clk_prepare_enable(dev->gate);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare pp clock gate\n");
		goto unprep_clk;
	}

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto unprep_clk_gate;


	dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		ret = PTR_ERR(dev->alloc_ctx);
		goto unreg_v4l2_dev;
	}

	vfd = &dev->vfd;
	memcpy(vfd, &pp_videodev, sizeof(*vfd));
	vfd->lock = &dev->mutex;
	vfd->v4l2_dev = &dev->v4l2_dev;
	snprintf(vfd->name, sizeof(vfd->name), "pp.%d", pdev->id);
	video_set_drvdata(vfd, dev);

	dev->m2m_dev = v4l2_m2m_init(&pp_m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		v4l2_err(&dev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(dev->m2m_dev);
		goto alloc_ctx_cleanup;
	}

	platform_set_drvdata(pdev, dev);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	pm_runtime_get_sync(&pdev->dev);

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		goto rel_m2m;
	}

	pm_runtime_put(&pdev->dev);

	v4l2_info(&dev->v4l2_dev, "device registered as /dev/video%d\n",
								vfd->num);

	return 0;

rel_m2m:
	v4l2_m2m_release(dev->m2m_dev);
alloc_ctx_cleanup:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
unreg_v4l2_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
unprep_clk_gate:
	clk_disable_unprepare(dev->gate);
unprep_clk:
	clk_disable_unprepare(dev->clk);

	return ret;
}

static int pp_remove(struct platform_device *pdev)
{
	struct pp_dev *dev = platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " PP_NAME);

	video_unregister_device(&dev->vfd);
	v4l2_m2m_release(dev->m2m_dev);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	v4l2_device_unregister(&dev->v4l2_dev);

	pm_runtime_disable(&pdev->dev);

	if (!pm_runtime_status_suspended(&pdev->dev)) {
		clk_disable_unprepare(dev->gate);
		clk_disable_unprepare(dev->clk);
	}

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int pp_runtime_suspend(struct device *dev)
{
	struct pp_dev *pp = dev_get_drvdata(dev);

	clk_disable_unprepare(pp->clk);
	clk_disable_unprepare(pp->gate);

	return 0;
}

static int pp_runtime_resume(struct device *dev)
{
	struct pp_dev *pp = dev_get_drvdata(dev);

	clk_prepare_enable(pp->gate);
	clk_prepare_enable(pp->clk);

	return 0;
}
#endif

static const struct dev_pm_ops pp_pm_ops = {
	SET_RUNTIME_PM_OPS(pp_runtime_suspend, pp_runtime_resume, NULL)
};

static struct platform_driver pp_pdrv = {
	.probe		= pp_probe,
	.remove		= pp_remove,
	.driver		= {
		.name = PP_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pp_match),
		.pm = &pp_pm_ops,
	},
};
module_platform_driver(pp_pdrv);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("Samsung S3C6400/S3C6410 video postprocessor driver");
MODULE_LICENSE("GPLv2");
