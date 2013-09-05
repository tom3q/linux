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

#ifndef _CODA_H_
#define _CODA_H_

#include "coda-regs.h"

#define CODA_NAME			"coda-ng"

#define CODA_MAX_INSTANCES		4

#define CODA_FMO_BUF_SIZE		32
#define CODADX6_WORK_BUF_SIZE		\
	(288 * 1024 + CODA_FMO_BUF_SIZE * 8 * 1024)
#define CODAMFC_WORK_BUF_SIZE		(1024 * 1024)
#define CODA7_WORK_BUF_SIZE		(128 * 1024)
#define CODA7_TEMP_BUF_SIZE		(304 * 1024)
#define CODA_PARA_BUF_SIZE		(10 * 1024)
#define CODA_ISRAM_SIZE			(2048 * 2)
#define CODADX6_IRAM_SIZE		0xb000
#define CODA7_IRAM_SIZE			0x14000
#define CODA7_PS_BUF_SIZE		0x28000
#define CODAMFC_PS_BUF_SIZE		60000
#define CODAMFC_MV_BUF_SIZE		25920

#define CODA_MAX_FRAMEBUFFERS		19

#define MAX_W				8192
#define MAX_H				8192
#define CODA_MAX_FRAME_SIZE		0x100000
#define CODA_MIN_BITSTREAM_CHUNK	512
#define FMO_SLICE_SAVE_BUF_SIZE         (32)
#define CODA_DEFAULT_GAMMA		4096

#define MIN_W				176
#define MIN_H				144

#define S_ALIGN				1 /* multiple of 2 */
#define W_ALIGN				1 /* multiple of 2 */
#define H_ALIGN				1 /* multiple of 2 */

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

enum coda_inst_type {
	CODA_INST_DECODER,
	CODA_INST_ENCODER,
	CODA_NUM_INST,
};

enum coda_product {
	CODA_DX6 = 0xf001,
	CODA_7541 = 0xf012,
	CODA_MFC_V1 = 0xf213,
};

#define CODA_FMT_BITSTREAM	(1 << 0)
#define CODA_FMT_YVU		(1 << 1)
#define CODA_FMT_MPLANE		(1 << 2)

struct coda_ctx;

struct coda_fmt {
	char *name;
	u32 fourcc;
	u32 flags;
};

struct coda_codec {
	u32 mode;
	u32 fourcc;
	enum coda_inst_type type;
	u32 max_w;
	u32 max_h;
};

struct coda_devtype {
	char *firmware;
	enum coda_product product;
	struct coda_codec *codecs;
	unsigned int num_codecs;

	size_t workbuf_size;
	size_t tempbuf_size;
	size_t ctxbuf_size;
	size_t ext_iram_size;
	size_t psbuf_size;

	int max_mb_x;
	int max_mb_y;

	u8 stream_end_bit;
	u8 stream_pic_flush_bit;
	u8 stream_pic_reset_bit;
	u8 stream_buf_dynalloc_bit;
	u8 option_gamma_bit;

	unsigned has_reg_idx:1;
	unsigned has_slice_buf:1;
	unsigned has_ps_buf:1;
	unsigned has_frm_dis:1;
	unsigned has_aux_std:1;
	unsigned has_swapped_parabuf:1;
	unsigned has_swapped_codebuf:1;
	unsigned has_mvcol_buf:1;
	unsigned has_gamma_ctl:1;
	unsigned has_reorder:1;
};

#define CODA_NUM_PLANES		3

/* Per-queue, driver-specific private data */
struct coda_q_data {
	unsigned int width;
	unsigned int height;
	unsigned int sizeimage[CODA_NUM_PLANES];
	const struct coda_fmt *fmt;
	u8 num_planes;
};

struct coda_aux_buf {
	void *vaddr;
	dma_addr_t paddr;
	u32 size;
};

#define CODA_NUM_VDEVS		2

struct coda_dev {
	struct v4l2_device v4l2_dev;
	struct video_device vfd[CODA_NUM_VDEVS];
	struct device *dev;
	const struct coda_devtype *devtype;

	void __iomem *regs_base;
	struct clk *clk_per;
	struct clk *clk_ahb;
	struct clk *clk_spec;

	struct coda_aux_buf codebuf;
	struct coda_aux_buf tempbuf;
	struct coda_aux_buf workbuf;
	struct gen_pool *iram_pool;
	long unsigned int iram_vaddr;
	long unsigned int iram_paddr;
	unsigned long iram_size;

	spinlock_t irqlock;
	struct mutex dev_mutex;
	struct mutex coda_mutex;
	struct v4l2_m2m_dev *m2m_dev;
	struct vb2_alloc_ctx *alloc_ctx;
	struct list_head instances;
	unsigned long instance_mask;
	struct delayed_work timeout;

	struct coda_ctx *curr_ctx;
};

struct coda_params {
	u8 rot_mode;
	u8 h264_intra_qp;
	u8 h264_inter_qp;
	u8 mpeg4_intra_qp;
	u8 mpeg4_inter_qp;
	u8 gop_size;
	int codec_mode;
	int codec_mode_aux;
	enum v4l2_mpeg_video_multi_slice_mode slice_mode;
	u16 framerate_num;
	u16 framerate_denom;
	u16 bitrate;
	u32 slice_max_bits;
	u32 slice_max_mb;
};

struct coda_iram_info {
	u32 axi_sram_use;
	phys_addr_t buf_bit_use;
	phys_addr_t buf_ip_ac_dc_use;
	phys_addr_t buf_dbk_y_use;
	phys_addr_t buf_dbk_c_use;
	phys_addr_t buf_ovl_use;
	phys_addr_t buf_btp_use;
	phys_addr_t search_ram_paddr;
	int search_ram_size;
};

struct coda_ctx {
	struct coda_dev *dev;
	const struct coda_video_dev *vdev;
	enum coda_inst_type inst_type;
	struct video_device *video_device;
	struct mutex buffer_mutex;
	struct mutex bitstream_mutex;
	struct list_head list;
	struct work_struct skip_run;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_fh fh;
	int reg_idx;
	int idx;

	struct coda_q_data q_data[2];
	const struct coda_codec *codec;
	enum v4l2_colorspace colorspace;
	struct coda_params params;
	struct coda_iram_info iram_info;

	struct kfifo bitstream_fifo;

	struct coda_aux_buf bitstream;
	struct coda_aux_buf parabuf;
	struct coda_aux_buf psbuf;
	struct coda_aux_buf slicebuf;
	struct coda_aux_buf internal_frames[CODA_MAX_FRAMEBUFFERS];
	struct coda_aux_buf workbuf;

	u32 isequence;
	u32 qsequence;
	u32 osequence;
	u32 gopcounter;
	u32 runcounter;

	char vpu_header[3][64];
	int vpu_header_size[3];

	u32 num_internal_frames;
	u32 bit_stream_param;
	u32 frm_dis_flg;
	int display_idx;

	unsigned aborting:1;
	unsigned streamon_out:1;
	unsigned streamon_cap:1;
	unsigned prescan_failed:1;
};

static inline struct coda_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct coda_ctx, fh);
}

struct coda_video_dev {
	const char *name;

	int (*open)(struct coda_ctx *);
	int (*ctrls_setup)(struct coda_ctx *);
	int (*prepare)(struct coda_ctx *);
	void (*finish)(struct coda_ctx *);

	const struct v4l2_ioctl_ops *ioctl_ops;
	const struct vb2_ops *vb2_ops;
};

extern const struct coda_video_dev codec_dec_vdev;
extern const struct coda_video_dev codec_enc_vdev;

extern int coda_debug;

static inline void coda_write(struct coda_dev *dev, u32 data, u32 reg)
{
	v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
		 "%s: data=0x%x, reg=0x%x\n", __func__, data, reg);
	writel(data, dev->regs_base + reg);
}

static inline unsigned int coda_read(struct coda_dev *dev, u32 reg)
{
	u32 data;
	data = readl(dev->regs_base + reg);
	v4l2_dbg(1, coda_debug, &dev->v4l2_dev,
		 "%s: data=0x%x, reg=0x%x\n", __func__, data, reg);
	return data;
}

static inline unsigned long coda_isbusy(struct coda_dev *dev)
{
	return coda_read(dev, CODA_REG_BIT_BUSY);
}

static inline int coda_is_initialized(struct coda_dev *dev)
{
	return (coda_read(dev, CODA_REG_BIT_CUR_PC) != 0);
}

static inline struct coda_q_data *get_q_data(struct coda_ctx *ctx,
					 enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return &(ctx->q_data[V4L2_M2M_SRC]);
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		return &(ctx->q_data[V4L2_M2M_DST]);
	default:
		BUG();
	}
	return NULL;
}

static inline bool coda_format_is_yuv(const struct coda_fmt *fmt)
{
	return !(fmt->flags & CODA_FMT_BITSTREAM);
}

static inline bool coda_format_is_yvu(const struct coda_fmt *fmt)
{
	return !!(fmt->flags & CODA_FMT_YVU);
}

static inline bool coda_format_is_mplane(const struct coda_fmt *fmt)
{
	return !!(fmt->flags & CODA_FMT_MPLANE);
}

extern int coda_power_up(struct coda_dev *dev);
extern void coda_power_down(struct coda_dev *dev);
extern int coda_clk_enable(struct coda_dev *dev);
extern void coda_clk_disable(struct coda_dev *dev);

extern u32 coda_bit_stream_end_flag(struct coda_dev *dev);

extern int coda_command_sync(struct coda_ctx *ctx, int cmd);
extern void coda_command_async(struct coda_ctx *ctx, int cmd);

extern int coda_alloc_framebuffers(struct coda_ctx *ctx,
				   struct coda_q_data *q_data);
extern void coda_free_framebuffers(struct coda_ctx *ctx);
extern int coda_alloc_context_buffers(struct coda_ctx *ctx,
				      struct coda_q_data *q_data);
extern void coda_free_context_buffers(struct coda_ctx *ctx);

extern const struct coda_fmt *coda_find_format(u32 fourcc);
extern const struct coda_codec *coda_find_codec(struct coda_ctx *ctx,
						const struct coda_fmt *fmt);

extern int coda_enum_fmt_bitstream(struct file *file, void *fh,
				   struct v4l2_fmtdesc *f);
extern int coda_enum_fmt_raw(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f);
extern int coda_g_fmt(struct file *file, void *fh, struct v4l2_format *f);
extern int coda_try_fmt(struct v4l2_format *f);
extern int coda_s_fmt(struct coda_ctx *ctx, struct v4l2_format *f,
		      const struct coda_fmt *fmt);
extern int coda_querycap(struct file *file, void *priv,
			 struct v4l2_capability *cap);
extern int coda_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *reqbufs);
extern int coda_querybuf(struct file *file, void *priv,
			 struct v4l2_buffer *buf);
extern int coda_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf);
extern int coda_expbuf(struct file *file, void *priv,
		       struct v4l2_exportbuffer *eb);
extern int coda_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf);
extern int coda_create_bufs(struct file *file, void *priv,
		     struct v4l2_create_buffers *create);
extern int coda_streamon(struct file *file, void *priv,
			 enum v4l2_buf_type type);
extern int coda_streamoff(struct file *file, void *priv,
			  enum v4l2_buf_type type);
extern int coda_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub);

extern int coda_queue_setup(struct vb2_queue *vq,
			    const struct v4l2_format *fmt,
			    unsigned int *nbuffers, unsigned int *nplanes,
			    unsigned int sizes[], void *alloc_ctxs[]);
extern int coda_buf_prepare(struct vb2_buffer *vb);
extern void coda_wait_prepare(struct vb2_queue *q);
extern void coda_wait_finish(struct vb2_queue *q);

#endif
