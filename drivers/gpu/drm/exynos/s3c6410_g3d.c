/*
 * DRM driver for Samsung FIMG-3DSE GPU
 *
 * Copyright (C) 2013-2014 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * TODO (most important first)
 *
 * - framebuffer setup request validation
 * - texture setup request validation
 */

#ifdef CONFIG_DRM_EXYNOS_S3C6410_G3D_DEBUG
#define DEBUG
#endif

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <drm/drmP.h>
#include <drm/exynos_drm.h>
#include "exynos_drm_drv.h"
#include "exynos_drm_gem.h"

#ifndef __CHECKER__
#define CREATE_TRACE_POINTS
#include "s3c6410_g3d_trace.h"
#endif

/*
 * Various definitions
 */

enum g3d_shader_type {
	G3D_SHADER_VERTEX,
	G3D_SHADER_PIXEL,

	G3D_NUM_SHADERS
};

enum g3d_shader_data_type {
	G3D_SHADER_DATA_FLOAT,
	G3D_SHADER_DATA_INT,
	G3D_SHADER_DATA_BOOL,

	G3D_NUM_SHADER_DATA_TYPES
};

#define G3D_AUTOSUSPEND_DELAY		100
#define G3D_FLUSH_TIMEOUT		msecs_to_jiffies(1000)
#define G3D_NUM_SHADER_INSTR		512
#define G3D_NUM_TEXTURES		8
#define G3D_NUM_VTX_TEXTURES		4

#define G3D_STATE_BUFFER_MAX_SIZE	\
	(G3D_NUM_REGISTERS * NUM_G3D_REG_WRITE_WORDS)
#define G3D_DRAW_BUFFER_MAX_SIZE	4096
#define G3D_INDEX_BUFFER_MAX_SIZE	(31 * sizeof(uint32_t))
#define G3D_SHADER_PROGRAM_MAX_SIZE	\
	(G3D_NUM_SHADER_INSTR * sizeof(uint32_t) * 4)

#define G3D_NUM_CONST_FLOAT		1024
#define G3D_NUM_CONST_INT		16
#define G3D_NUM_CONST_BOOL		1

#define G3D_CONST_FLOAT_BASE		0
#define G3D_CONST_INT_BASE		\
	G3D_CONST_FLOAT_BASE + G3D_NUM_CONST_FLOAT
#define G3D_CONST_BOOL_BASE		\
	G3D_CONST_INT_BASE + G3D_NUM_CONST_INT
#define G3D_CONST_REG_WORDS		\
	G3D_CONST_BOOL_BASE + G3D_NUM_CONST_BOOL
#define G3D_SHADER_DATA_MAX_SIZE	\
	(sizeof(uint32_t) * G3D_CONST_REG_WORDS)

#define G3D_VERTEX_BUFFER_SIZE		SZ_4K

#define G3D_NUM_DMA_LLI_ENTRIES		128
#define G3D_DMA_LLI_WORDS		8
#define G3D_DMA_LLI_SIZE		(G3D_DMA_LLI_WORDS * sizeof(uint32_t))

#define G3D_DMA_LLI_SRC_WORD		0
#define G3D_DMA_LLI_DST_WORD		1
#define G3D_DMA_LLI_NEXT_WORD		2
#define G3D_DMA_LLI_CTRL0_WORD		3
#define G3D_DMA_LLI_CTRL1_WORD		4
#define G3D_DMA_LLI_CONST_WORD		5

#define G3D_DMA_INTCLR_REG		0x008
#define G3D_DMA_CFG_REG			0x030
#define G3D_DMA_SRC_REG			0x100
#define G3D_DMA_DST_REG			0x104
#define G3D_DMA_NEXT_REG		0x108
#define G3D_DMA_CTRL0_REG		0x10c
#define G3D_DMA_CTRL1_REG		0x110
#define G3D_DMA_CCFG_REG		0x114

/*
 * Registers
 */

enum g3d_register {
	/*
	 * Protected registers
	 */

	FGPF_FRONTST,
	FGPF_DEPTHT,
	FGPF_DBMSK,
	FGPF_FBCTL,
	FGPF_DBADDR,
	FGPF_CBADDR,
	FGPF_FBW,

	G3D_NUM_PROT_REGISTERS,

	/*
	 * Public registers
	 */

	/* Host interface */
	FGHI_ATTRIB0 = G3D_NUM_PROT_REGISTERS,
	FGHI_ATTRIB1,
	FGHI_ATTRIB2,
	FGHI_ATTRIB3,
	FGHI_ATTRIB4,
	FGHI_ATTRIB5,
	FGHI_ATTRIB6,
	FGHI_ATTRIB7,
	FGHI_ATTRIB8,
	FGHI_ATTRIB9,
	FGHI_ATTRIB_VBCTRL0,
	FGHI_ATTRIB_VBCTRL1,
	FGHI_ATTRIB_VBCTRL2,
	FGHI_ATTRIB_VBCTRL3,
	FGHI_ATTRIB_VBCTRL4,
	FGHI_ATTRIB_VBCTRL5,
	FGHI_ATTRIB_VBCTRL6,
	FGHI_ATTRIB_VBCTRL7,
	FGHI_ATTRIB_VBCTRL8,
	FGHI_ATTRIB_VBCTRL9,
	FGHI_ATTRIB_VBBASE0,
	FGHI_ATTRIB_VBBASE1,
	FGHI_ATTRIB_VBBASE2,
	FGHI_ATTRIB_VBBASE3,
	FGHI_ATTRIB_VBBASE4,
	FGHI_ATTRIB_VBBASE5,
	FGHI_ATTRIB_VBBASE6,
	FGHI_ATTRIB_VBBASE7,
	FGHI_ATTRIB_VBBASE8,
	FGHI_ATTRIB_VBBASE9,

	/* Vertex shader */
	FGVS_ATTRIBUTE_NUM,
	FGVS_IN_ATTR_INDEX0,
	FGVS_IN_ATTR_INDEX1,
	FGVS_IN_ATTR_INDEX2,
	FGVS_OUT_ATTR_INDEX0,
	FGVS_OUT_ATTR_INDEX1,
	FGVS_OUT_ATTR_INDEX2,

	/* Primitive engine */
	FGPE_VERTEX_CONTEXT,
	FGPE_VIEWPORT_OX,
	FGPE_VIEWPORT_OY,
	FGPE_VIEWPORT_HALF_PX,
	FGPE_VIEWPORT_HALF_PY,
	FGPE_DEPTHRANGE_HALF_F_SUB_N,
	FGPE_DEPTHRANGE_HALF_F_ADD_N,

	/* Raster engine */
	FGRA_PIX_SAMP,
	FGRA_D_OFF_EN,
	FGRA_D_OFF_FACTOR,
	FGRA_D_OFF_UNITS,
	FGRA_BFCULL,
	FGRA_YCLIP,
	FGRA_LODCTL,
	FGRA_XCLIP,
	FGRA_PWIDTH,
	FGRA_PSIZE_MIN,
	FGRA_PSIZE_MAX,
	FGRA_COORDREPLACE,
	FGRA_LWIDTH,

	/* Per-fragment unit */
	FGPF_ALPHAT,
	FGPF_BACKST,
	FGPF_CCLR,
	FGPF_BLEND,
	FGPF_LOGOP,
	FGPF_CBMSK,

	G3D_NUM_REGISTERS
};

#define G3D_FGGB_PIPESTAT_REG		0x00
#define G3D_FGGB_PIPESTAT_MSK		0x0005171f

#define G3D_FGGB_CACHECTL_REG		0x04
#define G3D_FGGB_FLUSH_MSK		0x00000033
#define G3D_FGGB_INVAL_MSK		0x00001300

#define G3D_FGGB_RESET_REG		0x08
#define G3D_FGGB_VERSION		0x10
#define G3D_FGGB_INTPENDING_REG		0x40
#define G3D_FGGB_INTMASK_REG		0x44
#define G3D_FGGB_PIPEMASK_REG		0x48
#define G3D_FGGB_PIPETGTSTATE_REG	0x4c

#define G3D_FGWP_DEBUG_ENABLE_REG	0x200

#define G3D_FGHI_CONTROL_REG		0x8008
#define G3D_FGHI_IDXOFFSET_REG		0x800c
#define G3D_FGHI_VBADDR_REG		0x8010
#define G3D_FGHI_FIFO_ENTRY_REG		0xc000
#define G3D_FGHI_VB_ENTRY_REG		0xe000

#define G3D_FGVS_CONFIG_REG		0x1c800
#define G3D_FGVS_PC_RANGE_REG		0x20000
#define G3D_FGVS_ATTRIBUTE_NUM_REG	0x20004

#define G3D_FGPS_EXE_MODE_REG		0x4c800
#define G3D_FGPS_PC_START_REG		0x4c804
#define G3D_FGPS_PC_END_REG		0x4c808
#define G3D_FGPS_PC_COPY_REG		0x4c80c
#define G3D_FGPS_ATTRIBUTE_NUM_REG	0x4c810
#define G3D_FGPS_IBSTATUS_REG		0x4c814

#define G3D_TEXTURE_BASE(unit)		(0x60000 + (unit) * 0x50)
#define G3D_VTX_TEXTURE_BASE(unit)	(0x602c0 + (unit) * 0x8)

/*
 * Requests
 */

enum g3d_request_id {
	G3D_REQUEST_REGISTER_WRITE,

#define G3D_REG_WRITE_REG		0
#define G3D_REG_WRITE_VAL		1

#define NUM_G3D_REG_WRITE_WORDS		2

	G3D_REQUEST_SHADER_PROGRAM,

#define G3D_SHADER_PROG_UNIT_NATTRIB	0
#define G3D_SHADER_PROG_DCOUNT0		1
#define G3D_SHADER_PROG_DCOUNT1		2
#define G3D_SHADER_PROG_HANDLE		3
#define G3D_SHADER_PROG_OFFSET		4
#define G3D_SHADER_PROG_LENGTH		5

#define NUM_G3D_SHADER_PROG_WORDS	6

	G3D_REQUEST_SHADER_DATA,

#define G3D_SHADER_DATA_UNIT_TYPE_OFFS	0
#define G3D_SHADER_DATA_WORDS_BASE	1

	G3D_REQUEST_TEXTURE,

#define G3D_TEXTURE_CONTROL		0
#define G3D_TEXTURE_WIDTH		1
#define G3D_TEXTURE_HEIGHT		2
#define G3D_TEXTURE_DEPTH		3
#define G3D_TEXTURE_OFFSET_BASE		4
#define G3D_TEXTURE_OFFSET(lvl)		(G3D_TEXTURE_OFFSET_BASE + (lvl))
#define		G3D_NUM_MIPMAPS		11
#define G3D_TEXTURE_MIN_LEVEL		15
#define G3D_TEXTURE_MAX_LEVEL		16
#define G3D_TEXTURE_BASE_OFFSET		17
#define G3D_TEXTURE_HANDLE		18
#define G3D_TEXTURE_FLAGS		19
#define		G3D_TEXTURE_DIRTY	(1 << 0)
#define		G3D_TEXTURE_DETACH	(1 << 1)

#define NUM_G3D_TEXTURE_WORDS		20

	G3D_REQUEST_COLORBUFFER,

#define G3D_COLORBUFFER_FBCTL		0
#define G3D_COLORBUFFER_OFFSET		1
#define G3D_COLORBUFFER_WIDTH		2
#define G3D_COLORBUFFER_HANDLE		3
#define G3D_COLORBUFFER_FLAGS		4
#define		G3D_COLORBUFFER_DIRTY	(1 << 0)
#define		G3D_COLORBUFFER_DETACH	(1 << 1)

#define NUM_G3D_COLORBUFFER_WORDS	5

	G3D_REQUEST_DEPTHBUFFER,

#define G3D_DEPTHBUFFER_OFFSET		0
#define G3D_DEPTHBUFFER_HANDLE		1
#define G3D_DEPTHBUFFER_FLAGS		2
#define		G3D_DEPTHBUFFER_DIRTY	(1 << 0)
#define		G3D_DEPTHBUFFER_DETACH	(1 << 1)

#define NUM_G3D_DEPTHBUFFER_WORDS	3

	G3D_REQUEST_DRAW,

#define G3D_DRAW_VERTICES		0
#define G3D_DRAW_IB_HANDLE		1
#define G3D_DRAW_IB_OFFSET		2
#define G3D_DRAW_CONTROL		3
#define		G3D_DRAW_INDEXED	(1 << 31)

#define NUM_G3D_DRAW_WORDS		4

	G3D_REQUEST_VERTEX_BUFFER,

#define G3D_VERTEX_BUF_LENGTH		0
#define G3D_VERTEX_BUF_HANDLE		1
#define G3D_VERTEX_BUF_OFFSET		2
#define G3D_VERTEX_BUF_DST_OFFSET	3

#define NUM_G3D_VERTEX_BUF_WORDS	4

	G3D_REQUEST_VTX_TEXTURE,

#define G3D_VTX_TEXTURE_CONTROL		0
#define G3D_VTX_TEXTURE_BASE_OFFSET	1
#define G3D_VTX_TEXTURE_HANDLE		2
#define G3D_VTX_TEXTURE_FLAGS		3
#define		G3D_VTX_TEXTURE_DIRTY	(1 << 0)
#define		G3D_VTX_TEXTURE_DETACH	(1 << 1)

#define NUM_G3D_VTX_TEXTURE_WORDS	4

	G3D_NUM_REQUESTS
};

#define G3D_REQUEST_STRICT_SIZE_CHECK	(1 << 3)

static inline uint32_t req_length(const uint32_t *req)
{
	return *req & 0xffffff;
}

static inline uint8_t req_type(const uint32_t *req)
{
	return *req >> 24;
}

static inline uint32_t *req_data_wr(uint32_t *req)
{
	return req + 1;
}

static inline const uint32_t *req_data(const uint32_t *req)
{
	return req + 1;
}

/*
 * Private data types
 */
struct g3d_context;

enum g3d_state_bits {
	G3D_STATE_IDLE,
};

enum g3d_flush_level {
	G3D_FLUSH_HOST_FIFO = 0,
	G3D_FLUSH_HOST_INTERFACE,
	G3D_FLUSH_VERTEX_FIFO,
	G3D_FLUSH_VERTEX_CACHE,
	G3D_FLUSH_VERTEX_SHADER,
	G3D_FLUSH_PRIMITIVE = 8,
	G3D_FLUSH_TRIANGLE_SETUP,
	G3D_FLUSH_RASTER,
	G3D_FLUSH_PIXEL_SHADER = 12,
	G3D_FLUSH_PER_FRAGMENT = 16,
	G3D_FLUSH_COLOR_CACHE = 18,
};

/* Common driver data */
struct g3d_submit;

struct g3d_drvdata {
	struct exynos_drm_subdrv subdrv;
	void __iomem *base;
	int irq;
	struct resource *mem;
	struct clk *clock;
	struct device *dev;
	struct task_struct *thread;
	uint32_t version;

	spinlock_t ready_lock;
	struct list_head submit_list;
	wait_queue_head_t ready_wq;
	wait_queue_head_t flush_wq;
	wait_queue_head_t idle_wq;
	struct completion completion;
	struct timer_list watchdog_timer;

	atomic_t next_fence;
	uint32_t submitted_fence;
	uint32_t completed_fence;
	wait_queue_head_t fence_wq;
	uint32_t num_draws;

	struct g3d_context *last_ctx;

	unsigned long state;
	unsigned int tex_timestamp;
	unsigned int fb_timestamp;

	struct g3d_submit *submit;

	dma_addr_t base_phys;
	void __iomem *dma_base;
	struct clk *dma_clock;
	uint32_t dma_flush_mask;
	bool dma_enabled;
	bool dma_dirty;
	dma_addr_t dma_lli_phys;
	uint32_t *dma_lli;
	unsigned int lli_used;
};

enum g3d_context_state_bits {
	G3D_CONTEXT_STOP_PIXEL_SHADER,
	G3D_CONTEXT_ABORTED,
};

/* Per-file private data */
struct g3d_priv {
	struct g3d_drvdata *g3d;
	struct idr pipes_idr;
	struct rw_semaphore pipes_idr_sem;
};

/* Per-pipe context data */
struct g3d_shader_program {
	struct exynos_drm_gem_obj *obj;
	uint32_t data[G3D_CONST_REG_WORDS];
	uint32_t req[NUM_G3D_SHADER_PROG_WORDS];
};

struct g3d_texture {
	struct exynos_drm_gem_obj *obj;
	struct exynos_drm_gem_obj *prev_obj;
	uint32_t req[NUM_G3D_TEXTURE_WORDS];
};

struct g3d_vtx_texture {
	struct exynos_drm_gem_obj *obj;
	struct exynos_drm_gem_obj *prev_obj;
	uint32_t req[NUM_G3D_VTX_TEXTURE_WORDS];
};

struct g3d_colorbuffer {
	struct exynos_drm_gem_obj *obj;
	struct exynos_drm_gem_obj *prev_obj;
};

struct g3d_depthbuffer {
	struct exynos_drm_gem_obj *obj;
	struct exynos_drm_gem_obj *prev_obj;
};

struct g3d_context {
	struct g3d_drvdata *g3d;
	struct drm_device *drm_dev;
	struct drm_file *file;
	struct kref ref;

	struct mutex submit_mutex;
	uint32_t last_fence;

	uint32_t registers[G3D_NUM_REGISTERS];
	uint32_t register_masks[G3D_NUM_PROT_REGISTERS];

	struct g3d_shader_program shader_program[G3D_NUM_SHADERS];
	struct g3d_texture texture[G3D_NUM_TEXTURES];
	struct g3d_vtx_texture vtx_texture[G3D_NUM_VTX_TEXTURES];
	struct g3d_colorbuffer colorbuffer;
	struct g3d_depthbuffer depthbuffer;

	unsigned long state;
	unsigned int dirty_level;
};

struct g3d_submit {
	struct g3d_context *ctx;
	struct idr read_gem_idr;
	struct idr write_gem_idr;
	uint32_t fence;
	struct list_head list;

	const uint32_t *state_update_start;
	const uint32_t *state_update_end;
	const uint32_t *apply_start;
	const uint32_t *apply_end;
	const uint32_t *cur;
	const uint32_t *end;

	uint32_t num_words;
	uint32_t words[];
};

struct exynos_drm_gem_g3d_priv {
	unsigned int tex_timestamp;
	unsigned int fb_timestamp;
	uint32_t read_fence;
	uint32_t write_fence;
	bool dirty;
};

struct g3d_request_info {
	int (*validate)(struct g3d_submit *, uint32_t *);
	int (*handle)(struct g3d_submit *, const uint32_t *);
	void (*update_state)(struct g3d_submit *, const uint32_t *);
	void (*apply)(struct g3d_submit *, const uint32_t *);
	size_t length;
	unsigned int flags;
};

static const uint32_t g3d_shader_data_reg_base[G3D_NUM_SHADER_DATA_TYPES] = {
	[G3D_SHADER_DATA_FLOAT] = 0x4000,
	[G3D_SHADER_DATA_INT] = 0x8000,
	[G3D_SHADER_DATA_BOOL] = 0x8400,
};

static const uint32_t g3d_shader_data_count[G3D_NUM_SHADER_DATA_TYPES] = {
	[G3D_SHADER_DATA_FLOAT] = G3D_NUM_CONST_FLOAT,
	[G3D_SHADER_DATA_INT] = G3D_NUM_CONST_INT,
	[G3D_SHADER_DATA_BOOL] = G3D_NUM_CONST_BOOL,
};

static const uint32_t g3d_shader_type_offset[G3D_NUM_SHADER_DATA_TYPES] = {
	[G3D_SHADER_DATA_FLOAT] = G3D_CONST_FLOAT_BASE,
	[G3D_SHADER_DATA_INT] = G3D_CONST_INT_BASE,
	[G3D_SHADER_DATA_BOOL] = G3D_CONST_BOOL_BASE,
};

static const uint32_t g3d_shader_base[G3D_NUM_SHADERS] = {
	[G3D_SHADER_VERTEX] = 0x10000,
	[G3D_SHADER_PIXEL] = 0x40000,
};

static const uint32_t g3d_register_masks_def[G3D_NUM_PROT_REGISTERS] = {
	[FGPF_FRONTST] = 0,
	[FGPF_DEPTHT] = 0,
	[FGPF_DBMSK] = 0,
	[FGPF_FBCTL] = 0xfffffff8,
	[FGPF_DBADDR] = 0,
	[FGPF_CBADDR] = 0,
	[FGPF_FBW] = 0,
};

static const uint32_t g3d_registers[G3D_NUM_REGISTERS] = {
	/*
	 * Protected registers
	 */

	[FGPF_FRONTST] = 0x7000c,
	[FGPF_DEPTHT] = 0x70014,
	[FGPF_DBMSK] = 0x70028,
	[FGPF_FBCTL] = 0x7002c,
	[FGPF_DBADDR] = 0x70030,
	[FGPF_CBADDR] = 0x70034,
	[FGPF_FBW] = 0x70038,

	/*
	 * Public registers
	 */

	/* Host interface */
	[FGHI_ATTRIB0] = 0x8040,
	[FGHI_ATTRIB1] = 0x8044,
	[FGHI_ATTRIB2] = 0x8048,
	[FGHI_ATTRIB3] = 0x804c,
	[FGHI_ATTRIB4] = 0x8050,
	[FGHI_ATTRIB5] = 0x8054,
	[FGHI_ATTRIB6] = 0x8058,
	[FGHI_ATTRIB7] = 0x805c,
	[FGHI_ATTRIB8] = 0x8060,
	[FGHI_ATTRIB9] = 0x8064,
	[FGHI_ATTRIB_VBCTRL0] = 0x8080,
	[FGHI_ATTRIB_VBCTRL1] = 0x8084,
	[FGHI_ATTRIB_VBCTRL2] = 0x8088,
	[FGHI_ATTRIB_VBCTRL3] = 0x808c,
	[FGHI_ATTRIB_VBCTRL4] = 0x8090,
	[FGHI_ATTRIB_VBCTRL5] = 0x8094,
	[FGHI_ATTRIB_VBCTRL6] = 0x8098,
	[FGHI_ATTRIB_VBCTRL7] = 0x809c,
	[FGHI_ATTRIB_VBCTRL8] = 0x80a0,
	[FGHI_ATTRIB_VBCTRL9] = 0x80a4,
	[FGHI_ATTRIB_VBBASE0] = 0x80c0,
	[FGHI_ATTRIB_VBBASE1] = 0x80c4,
	[FGHI_ATTRIB_VBBASE2] = 0x80c8,
	[FGHI_ATTRIB_VBBASE3] = 0x80cc,
	[FGHI_ATTRIB_VBBASE4] = 0x80d0,
	[FGHI_ATTRIB_VBBASE5] = 0x80d4,
	[FGHI_ATTRIB_VBBASE6] = 0x80d8,
	[FGHI_ATTRIB_VBBASE7] = 0x80dc,
	[FGHI_ATTRIB_VBBASE8] = 0x80e0,
	[FGHI_ATTRIB_VBBASE9] = 0x80e4,

	/* Vertex shader */
	[FGVS_ATTRIBUTE_NUM] = 0x20004,
	[FGVS_IN_ATTR_INDEX0] = 0x20008,
	[FGVS_IN_ATTR_INDEX1] = 0x2000c,
	[FGVS_IN_ATTR_INDEX2] = 0x20010,
	[FGVS_OUT_ATTR_INDEX0] = 0x20014,
	[FGVS_OUT_ATTR_INDEX1] = 0x20018,
	[FGVS_OUT_ATTR_INDEX2] = 0x2001c,

	/* Primitive engine */
	[FGPE_VERTEX_CONTEXT] = 0x30000,
	[FGPE_VIEWPORT_OX] = 0x30004,
	[FGPE_VIEWPORT_OY] = 0x30008,
	[FGPE_VIEWPORT_HALF_PX] = 0x3000c,
	[FGPE_VIEWPORT_HALF_PY] = 0x30010,
	[FGPE_DEPTHRANGE_HALF_F_SUB_N] = 0x30014,
	[FGPE_DEPTHRANGE_HALF_F_ADD_N] = 0x30018,

	/* Raster engine */
	[FGRA_PIX_SAMP] = 0x38000,
	[FGRA_D_OFF_EN] = 0x38004,
	[FGRA_D_OFF_FACTOR] = 0x38008,
	[FGRA_D_OFF_UNITS] = 0x3800c,
	[FGRA_BFCULL] = 0x38014,
	[FGRA_YCLIP] = 0x38018,
	[FGRA_LODCTL] = 0x3c000,
	[FGRA_XCLIP] = 0x3c004,
	[FGRA_PWIDTH] = 0x3801c,
	[FGRA_PSIZE_MIN] = 0x38020,
	[FGRA_PSIZE_MAX] = 0x38024,
	[FGRA_COORDREPLACE] = 0x38028,
	[FGRA_LWIDTH] = 0x3802c,

	/* Per-fragment unit */
	[FGPF_ALPHAT] = 0x70008,
	[FGPF_BACKST] = 0x70010,
	[FGPF_CCLR] = 0x70018,
	[FGPF_BLEND] = 0x7001c,
	[FGPF_LOGOP] = 0x70020,
	[FGPF_CBMSK] = 0x70024,
};

static const struct g3d_request_info g3d_requests[];

/*
 * Debugging helpers
 */

#define DEBUG_TRACE		(1 << 0)
#define DEBUG_EVENTS		(1 << 1)
#define DEBUG_PIPE		(1 << 2)
#define DEBUG_FOPS		(1 << 3)
#define DEBUG_CTX		(1 << 4)
#define DEBUG_REQS		(1 << 27)
#define DEBUG_REQ_DATA		(1 << 28)
#define DEBUG_DRAW_DATA		(1 << 29)
#define DEBUG_IO		(1 << 30)
#define DEBUG_HW_WATCHPOINTS	(1 << 31)

#ifdef DEBUG

#define dev_dbg_mask(mask, ...)				\
	do {						\
		if (s3c6410_g3d_debug_mask & (mask))	\
			dev_dbg(__VA_ARGS__);		\
	} while (0)

static unsigned int s3c6410_g3d_debug_mask;
module_param(s3c6410_g3d_debug_mask, uint, 0644);

static inline void g3d_assert_enabled(struct g3d_drvdata *g3d)
{
	BUG_ON(test_bit(G3D_STATE_IDLE, &g3d->state));
}

static void g3d_dump_draw_buffer(const void *buf, size_t len)
{
	if (s3c6410_g3d_debug_mask & DEBUG_DRAW_DATA)
		print_hex_dump(KERN_DEBUG, "    ", DUMP_PREFIX_OFFSET,
				32, 4, buf, len, false);
}

static void g3d_dump_req_data(const void *buf, size_t len)
{
	if (s3c6410_g3d_debug_mask & DEBUG_REQ_DATA)
		print_hex_dump(KERN_DEBUG, "    ", DUMP_PREFIX_OFFSET,
				32, 4, buf, len * 4, false);
}
#else /* !DEBUG */

#define dev_dbg_mask(mask, ...)				\
	do {						\
		if (0)	\
			dev_dbg(__VA_ARGS__);		\
	} while (0)

static inline void g3d_assert_enabled(struct g3d_drvdata *g3d) {}
static inline void g3d_dump_draw_buffer(const void *buf, size_t len) {}
static inline void g3d_dump_req_data(const void *buf, size_t len) {}

#endif /* DEBUG */

#define dev_dbg_trace(...)	dev_dbg_mask(DEBUG_TRACE, __VA_ARGS__)
#define dev_dbg_events(...)	dev_dbg_mask(DEBUG_EVENTS, __VA_ARGS__)
#define dev_dbg_pipe(...)	dev_dbg_mask(DEBUG_PIPE, __VA_ARGS__)
#define dev_dbg_fops(...)	dev_dbg_mask(DEBUG_FOPS, __VA_ARGS__)
#define dev_dbg_ctx(...)	dev_dbg_mask(DEBUG_CTX, __VA_ARGS__)
#define dev_dbg_reqs(...)	dev_dbg_mask(DEBUG_REQS, __VA_ARGS__)
#define dev_dbg_io(...)		dev_dbg_mask(DEBUG_IO, __VA_ARGS__)

/*
 * Register accessors
 */
static inline void g3d_write_relaxed(struct g3d_drvdata *g3d,
				     uint32_t val, uint32_t reg)
{
	g3d_assert_enabled(g3d);
	dev_dbg_io(g3d->dev, "%s(%08x, %08x)\n", __func__, val, reg);
	writel_relaxed(val, g3d->base + reg);
}

static inline void g3d_write(struct g3d_drvdata *g3d,
			     uint32_t val, uint32_t reg)
{
	g3d_assert_enabled(g3d);
	dev_dbg_io(g3d->dev, "%s(%08x, %08x)\n", __func__, val, reg);
	writel(val, g3d->base + reg);
}

static inline uint32_t g3d_read(struct g3d_drvdata *g3d, uint32_t reg)
{
	uint32_t val;

	g3d_assert_enabled(g3d);
	val = readl(g3d->base + reg);
	dev_dbg_io(g3d->dev, "%s(%08x) = %08x\n", __func__, reg, val);

	return val;
}

static inline void g3d_write_burst(struct g3d_drvdata *g3d, uint32_t reg,
				   const uint32_t *buf, uint32_t len)
{
	g3d_assert_enabled(g3d);

	memcpy(g3d->base + reg, buf, len);

	/* Make sure register writes are not reordered across this point. */
	wmb();
}

#ifdef DEBUG
static void g3d_enable_debug(struct g3d_drvdata *g3d)
{
	if (s3c6410_g3d_debug_mask & DEBUG_HW_WATCHPOINTS)
		g3d_write(g3d, 1, G3D_FGWP_DEBUG_ENABLE_REG);
	else
		g3d_write(g3d, 0, G3D_FGWP_DEBUG_ENABLE_REG);
}
#else
static void g3d_enable_debug(struct g3d_drvdata *g3d)
{
	g3d_write(g3d, 0, G3D_FGWP_DEBUG_ENABLE_REG);
}
#endif

static inline bool g3d_pipeline_idle(struct g3d_drvdata *g3d, uint32_t mask)
{
	uint32_t stat = g3d_read(g3d, G3D_FGGB_PIPESTAT_REG);

	dev_dbg_pipe(g3d->dev, "%s: stat = %08x\n", __func__, stat);
	return !(stat & mask);
}

/*
 * DMA support (Experimental, PL080S-only)
 */

static inline unsigned int g3d_dma_free_lli(struct g3d_drvdata *g3d)
{
	return G3D_NUM_DMA_LLI_ENTRIES - g3d->lli_used;
}

static void g3d_dma_flush(struct g3d_drvdata *g3d)
{
	if (!g3d->lli_used || g3d->dma_dirty)
		return;

	g3d->dma_dirty = true;
}

static inline bool g3d_dma_idle(struct g3d_drvdata *g3d)
{
	return !(readl(g3d->dma_base + G3D_DMA_CCFG_REG) & 0x1);
}

static inline bool g3d_idle_condition_dma(struct g3d_drvdata *g3d,
					  uint32_t mask)
{
	return (g3d_dma_idle(g3d) && g3d_pipeline_idle(g3d, mask))
		|| !list_empty(&g3d->submit_list);
}

static void g3d_dma_set(struct g3d_drvdata *g3d, uint32_t reg,
			uint32_t val, uint32_t num_words);

static int g3d_dma_wait(struct g3d_drvdata *g3d, uint32_t mask, bool idle)
{
	dev_dbg_trace(g3d->dev, "%s\n", __func__);

	if (!g3d->dma_dirty)
		return 0;

	dev_dbg_trace(g3d->dev, "%s\n", __func__);

	g3d_dma_set(g3d, G3D_FGGB_PIPEMASK_REG, mask, 1);
	g3d_dma_set(g3d, G3D_FGGB_INTMASK_REG, 1, 1);

	writel(0x1, g3d->dma_base + G3D_DMA_CFG_REG);

	writel_relaxed(g3d->dma_lli[G3D_DMA_LLI_SRC_WORD],
			g3d->dma_base + G3D_DMA_SRC_REG);
	writel_relaxed(g3d->dma_lli[G3D_DMA_LLI_DST_WORD],
			g3d->dma_base + G3D_DMA_DST_REG);
	writel_relaxed(g3d->dma_lli[G3D_DMA_LLI_NEXT_WORD],
			g3d->dma_base + G3D_DMA_NEXT_REG);
	writel_relaxed(g3d->dma_lli[G3D_DMA_LLI_CTRL0_WORD],
			g3d->dma_base + G3D_DMA_CTRL0_REG);
	writel_relaxed(g3d->dma_lli[G3D_DMA_LLI_CTRL1_WORD],
			g3d->dma_base + G3D_DMA_CTRL1_REG);

	g3d->lli_used = 0;
	g3d->dma_dirty = false;
	g3d->dma_flush_mask = mask;

	if (in_interrupt()) {
		mod_timer(&g3d->watchdog_timer, jiffies + G3D_FLUSH_TIMEOUT);
		writel(0x8001, g3d->dma_base + G3D_DMA_CCFG_REG);
		return -EAGAIN;
	}

	init_completion(&g3d->completion);
	mod_timer(&g3d->watchdog_timer, jiffies + G3D_FLUSH_TIMEOUT);
	writel(0x8001, g3d->dma_base + G3D_DMA_CCFG_REG);

	if (idle)
		wait_event(g3d->idle_wq, g3d_idle_condition_dma(g3d, mask));
	else
		wait_for_completion(&g3d->completion);

	dev_dbg_events(g3d->dev, "%s:%d idle/flush_wq wake-up\n",
			__func__, __LINE__);

	return -EAGAIN;
}

static void g3d_dma_set(struct g3d_drvdata *g3d, uint32_t reg,
			uint32_t val, uint32_t num_words)
{
	dma_addr_t lli_phys;
	uint32_t *lli;

	if (!g3d_dma_free_lli(g3d)) {
		BUG_ON(in_interrupt());

		g3d_dma_flush(g3d);
		g3d_dma_wait(g3d, 0, false);
	}

	lli = &g3d->dma_lli[g3d->lli_used * G3D_DMA_LLI_WORDS];
	lli_phys = g3d->dma_lli_phys + g3d->lli_used * G3D_DMA_LLI_SIZE;

	lli[G3D_DMA_LLI_SRC_WORD] = lli_phys
			+ ((void *)&lli[G3D_DMA_LLI_CONST_WORD] - (void *)lli);
	lli[G3D_DMA_LLI_DST_WORD] = g3d->base_phys + reg;
	lli[G3D_DMA_LLI_NEXT_WORD] = 0;
	lli[G3D_DMA_LLI_CTRL0_WORD] = 0x7A480000;
	lli[G3D_DMA_LLI_CTRL1_WORD] = num_words;
	lli[G3D_DMA_LLI_CONST_WORD] = val;

	if (g3d->lli_used) {
		uint32_t *lli_prev = lli - G3D_DMA_LLI_WORDS;

		lli_prev[G3D_DMA_LLI_NEXT_WORD] = lli_phys;
	}

	++g3d->lli_used;
}

static void g3d_dma_copy(struct g3d_drvdata *g3d, uint32_t reg,
			 dma_addr_t src, uint32_t num_words)
{
	dma_addr_t lli_phys;
	uint32_t *lli;

	if (!g3d_dma_free_lli(g3d)) {
		g3d_dma_flush(g3d);
		g3d_dma_wait(g3d, 0, false);
	}

	lli = &g3d->dma_lli[g3d->lli_used * G3D_DMA_LLI_WORDS];
	lli_phys = g3d->dma_lli_phys + g3d->lli_used * G3D_DMA_LLI_SIZE;

	lli[G3D_DMA_LLI_SRC_WORD] = src;
	lli[G3D_DMA_LLI_DST_WORD] = g3d->base_phys + reg;
	lli[G3D_DMA_LLI_NEXT_WORD] = 0;
	lli[G3D_DMA_LLI_CTRL0_WORD] = 0x76489000;
	lli[G3D_DMA_LLI_CTRL1_WORD] = num_words;

	if (g3d->lli_used) {
		uint32_t *lli_prev = lli - G3D_DMA_LLI_WORDS;

		lli_prev[G3D_DMA_LLI_NEXT_WORD] = lli_phys;
	}

	++g3d->lli_used;
}

/*
 * Hardware operations
 */
static void g3d_soft_reset(struct g3d_drvdata *g3d)
{
	g3d_write(g3d, 1, G3D_FGGB_RESET_REG);
	udelay(10);
	g3d_write(g3d, 0, G3D_FGGB_RESET_REG);
	udelay(10);

	dev_dbg_trace(g3d->dev, "%s\n", __func__);
}

static void g3d_init_regs(struct g3d_drvdata *g3d)
{
	g3d_write_relaxed(g3d, 0x80010009, G3D_FGHI_CONTROL_REG);
	g3d_write(g3d, 0x00000001, G3D_FGHI_IDXOFFSET_REG);
}

static void g3d_idle_irq_enable(struct g3d_drvdata *g3d, uint32_t mask)
{
	g3d_write(g3d, 0, G3D_FGGB_PIPEMASK_REG);
	g3d_write(g3d, 0, G3D_FGGB_PIPETGTSTATE_REG);
	g3d_write(g3d, mask, G3D_FGGB_PIPEMASK_REG);
	g3d_write(g3d, 1, G3D_FGGB_INTMASK_REG);
}

static void g3d_idle_irq_ack_and_disable(struct g3d_drvdata *g3d)
{
	g3d_write(g3d, 0, G3D_FGGB_INTMASK_REG);
	g3d_write(g3d, 0, G3D_FGGB_INTPENDING_REG);
}

static void g3d_initialize(struct g3d_drvdata *g3d)
{
	g3d_soft_reset(g3d);
	g3d_idle_irq_ack_and_disable(g3d);
	g3d_init_regs(g3d);
	g3d_enable_debug(g3d);
}

static int g3d_flush_caches(struct g3d_drvdata *g3d)
{
	int timeout = 100000000;

	dev_dbg_trace(g3d->dev, "%s\n", __func__);

	++g3d->fb_timestamp;
	g3d_write(g3d, G3D_FGGB_FLUSH_MSK, G3D_FGGB_CACHECTL_REG);

	do {
		if (!g3d_read(g3d, G3D_FGGB_CACHECTL_REG))
			return 0;
		cpu_relax();
	} while (--timeout);

	dev_err(g3d->dev, "cache flush timed out\n");

	return -EFAULT;
}

static int g3d_invalidate_caches(struct g3d_drvdata *g3d)
{
	int timeout = 100000000;

	dev_dbg_trace(g3d->dev, "%s\n", __func__);

	++g3d->tex_timestamp;
	g3d_write(g3d, G3D_FGGB_INVAL_MSK, G3D_FGGB_CACHECTL_REG);

	do {
		if (!g3d_read(g3d, G3D_FGGB_CACHECTL_REG))
			return 0;
		cpu_relax();
	} while (--timeout);

	dev_err(g3d->dev, "cache invalidation timed out\n");

	return -EFAULT;
}

static inline bool g3d_idle_condition(struct g3d_drvdata *g3d, uint32_t mask)
{
	return g3d_pipeline_idle(g3d, mask) || !list_empty(&g3d->submit_list);
}

static int g3d_wait_for_flush(struct g3d_drvdata *g3d, uint32_t mask, bool idle)
{
	dev_dbg_trace(g3d->dev, "%s\n", __func__);

	if (in_interrupt()) {
		mod_timer(&g3d->watchdog_timer, jiffies + G3D_FLUSH_TIMEOUT);
		g3d_idle_irq_enable(g3d, mask);
		return -EAGAIN;
	}

	init_completion(&g3d->completion);
	mod_timer(&g3d->watchdog_timer, jiffies + G3D_FLUSH_TIMEOUT);
	g3d_idle_irq_enable(g3d, mask);

	if (idle)
		wait_event(g3d->idle_wq, g3d_idle_condition(g3d, mask));
	else
		wait_for_completion(&g3d->completion);

	dev_dbg_events(g3d->dev, "%s:%d idle/flush_wq wake-up\n",
			__func__, __LINE__);

	return 0;
}

static int g3d_flush_pipeline(struct g3d_drvdata *g3d,
			      enum g3d_flush_level level, bool idle)
{
	uint32_t mask;
	int ret;

	mask = ((1 << (level + 1)) - 1) & G3D_FGGB_PIPESTAT_MSK;

	dev_dbg_trace(g3d->dev, "%s (mask = %08x)\n", __func__, mask);

	if (g3d->dma_enabled) {
		ret = g3d_dma_wait(g3d, mask, idle);
		if (ret)
			return ret;
	}

	if (g3d_pipeline_idle(g3d, mask)) {
		trace_g3d_draw_complete(g3d->num_draws);
		return 0;
	}

	ret = g3d_wait_for_flush(g3d, mask, idle);
	if (ret < 0)
		return ret;

	return -EAGAIN;
}

static void g3d_stop_pixel_shader(struct g3d_drvdata *g3d)
{
	g3d_write(g3d, 0, G3D_FGPS_EXE_MODE_REG);
}

static void g3d_start_pixel_shader(struct g3d_drvdata *g3d)
{
	int timeout = 1000;

	while (g3d_read(g3d, G3D_FGPS_IBSTATUS_REG) & 1) {
		if (!timeout--) {
			dev_err(g3d->dev, "timed out waiting for pixel shader to get ready\n");
			break;
		}
		cpu_relax();
	}

	g3d_write(g3d, 1, G3D_FGPS_EXE_MODE_REG);
}

/*
 * Register cache
 */

static inline void g3d_reg_cache_write_masked(struct g3d_context *ctx,
					      uint32_t value, uint32_t reg)
{
	uint32_t mask = ctx->register_masks[reg];
	uint32_t val = ctx->registers[reg] & mask;

	val |= value & ~mask;
	ctx->registers[reg] = val;
}

static inline void g3d_reg_cache_write(struct g3d_context *ctx,
				       uint32_t value, uint32_t reg)
{
	ctx->registers[reg] = value;
}

static inline void g3d_reg_cache_sync(struct g3d_drvdata *g3d,
				      struct g3d_context *ctx, uint32_t reg)
{
	g3d_write_relaxed(g3d, ctx->registers[reg], g3d_registers[reg]);
}

static void g3d_reg_cache_sync_all(struct g3d_drvdata *g3d,
				   struct g3d_context *ctx)
{
	unsigned int reg;

	for (reg = 0; reg < G3D_NUM_REGISTERS; ++reg)
		g3d_reg_cache_sync(g3d, ctx, reg);

	/* Make sure register writes are not reordered across this point. */
	wmb();
}

/*
 * GEM helpers
 */

static struct exynos_drm_gem_obj *exynos_drm_gem_lookup(struct drm_device *dev,
							struct drm_file *file,
							uint32_t handle)
{
	struct drm_gem_object *base;

	base = drm_gem_object_lookup(dev, file, handle);
	if (!base)
		return NULL;

	return to_exynos_gem_obj(base);
}

static int g3d_prepare_gem_object(struct g3d_drvdata *g3d,
				  struct exynos_drm_gem_obj *obj)
{
	if (obj->g3d_priv)
		return 0;

	obj->g3d_priv = kzalloc(sizeof(*obj->g3d_priv), GFP_KERNEL);
	if (!obj->g3d_priv)
		return -ENOMEM;

	obj->g3d_priv->tex_timestamp = g3d->tex_timestamp;
	obj->g3d_priv->fb_timestamp = g3d->fb_timestamp;

	return 0;
}

/*
 * Requests
 */

/* Request callback wrappers */
static inline int g3d_request_handle(struct g3d_submit *submit,
				      const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	const struct g3d_request_info *req_info;
	int ret = 0;

	req_info = &g3d_requests[req_type(req)];

	dev_dbg_reqs(g3d->dev, "%s: %d\n", __func__, req_type(req));

	if (req_info->handle)
		ret = req_info->handle(submit, req);

	return ret;
}

static inline void g3d_request_update_state(struct g3d_submit *submit,
					    const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	const struct g3d_request_info *req_info;

	req_info = &g3d_requests[req_type(req)];

	dev_dbg_reqs(g3d->dev, "%s: %d\n", __func__, req_type(req));

	if (req_info->update_state)
		req_info->update_state(submit, req);
}

static inline void g3d_request_apply(struct g3d_submit *submit,
				     const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	const struct g3d_request_info *req_info;

	req_info = &g3d_requests[req_type(req)];

	dev_dbg_reqs(g3d->dev, "%s: %d\n", __func__, req_type(req));

	if (req_info->apply)
		req_info->apply(submit, req);
}

static void g3d_restore_shader_program(struct g3d_context *ctx,
				       struct g3d_shader_program *sp,
				       uint8_t unit);
static void g3d_restore_texture(struct g3d_context *ctx,
				struct g3d_texture *tex, uint8_t unit);
static void g3d_restore_vtx_texture(struct g3d_context *ctx,
				struct g3d_vtx_texture *tex, uint8_t unit);

static void g3d_restore_context(struct g3d_context *ctx)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	int i;

	g3d_reg_cache_sync_all(g3d, ctx);

	for (i = 0; i < G3D_NUM_SHADERS; ++i)
		g3d_restore_shader_program(ctx, &ctx->shader_program[i], i);

	for (i = 0; i < G3D_NUM_TEXTURES; ++i)
		g3d_restore_texture(ctx, &ctx->texture[i], i);

	for (i = 0; i < G3D_NUM_VTX_TEXTURES; ++i)
		g3d_restore_vtx_texture(ctx, &ctx->vtx_texture[i], i);
}

/* State update */
static inline void g3d_state_mark_dirty(struct g3d_context *ctx,
					unsigned int level)
{
	if (level > ctx->dirty_level)
		ctx->dirty_level = level;
}

static void g3d_state_update(struct g3d_context *ctx, struct g3d_submit *submit)
{
	const uint32_t *req = submit->state_update_start;

	while (req < submit->state_update_end) {
		g3d_request_update_state(submit, req);
		req = req_data(req) + req_length(req);
	}
}

static void g3d_state_apply(struct g3d_context *ctx, struct g3d_submit *submit)
{
	const uint32_t *req = submit->apply_start;

	while (req < submit->apply_end) {
		g3d_request_apply(submit, req);
		req = req_data(req) + req_length(req);
	}
}

static int g3d_prepare_draw(struct g3d_context *ctx, struct g3d_submit *submit,
			     const uint32_t *req)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	bool stop_pixel_shader = false;
	bool was_powered_down = false;
	bool full_restore = false;
	unsigned int flush_level;
	int ret;

	submit->state_update_end = req;

	g3d_state_update(ctx, submit);

	submit->state_update_start = req_data(req) + req_length(req);

	if (test_and_clear_bit(G3D_STATE_IDLE, &g3d->state)) {
		/* The hardware must be operational if generated interrupt */
		BUG_ON(in_interrupt());

		ret = pm_runtime_get_sync(g3d->dev);
		if (!ret)
			was_powered_down = true;
		clk_prepare_enable(g3d->clock);
	}

	if (was_powered_down || g3d->last_ctx != ctx) {
		flush_level = G3D_FLUSH_PER_FRAGMENT;
		stop_pixel_shader = true;
		full_restore = true;
	} else {
		flush_level = ctx->dirty_level;
	}

	if (in_interrupt() && flush_level >= G3D_FLUSH_VERTEX_SHADER)
		return -EINVAL;

	ret = g3d_flush_pipeline(g3d, flush_level, false);
	if (ret == -EAGAIN)
		return ret;

	if (test_and_clear_bit(G3D_CONTEXT_STOP_PIXEL_SHADER, &ctx->state))
		stop_pixel_shader = true;

	if (stop_pixel_shader)
		g3d_stop_pixel_shader(g3d);

	submit->apply_end = req;

	if (full_restore)
		g3d_restore_context(ctx);
	else
		g3d_state_apply(ctx, submit);

	if (stop_pixel_shader)
		g3d_start_pixel_shader(g3d);

	ctx->dirty_level = G3D_FLUSH_HOST_FIFO;
	submit->apply_start = req_data(req) + req_length(req);
	g3d->last_ctx = ctx;

	return 0;
}

/* Register state update request */
static int g3d_validate_state_buffer(struct g3d_submit *submit, uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	unsigned int count = req_length(req) / NUM_G3D_REG_WRITE_WORDS;

	req = req_data_wr(req);

	while (count--) {
		uint32_t reg = req[G3D_REG_WRITE_REG];

		if (unlikely(reg >= G3D_NUM_REGISTERS)) {
			dev_err(g3d->dev, "register index out of range\n");
			return -EINVAL;
		}

		req += NUM_G3D_REG_WRITE_WORDS;
	}

	return 0;
}

static void g3d_process_state_buffer(struct g3d_submit *submit,
				     const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	unsigned int count = req_length(req) / NUM_G3D_REG_WRITE_WORDS;
	unsigned int dirty_level;
	uint32_t max_reg = 0;

	req = req_data(req);

	while (count--) {
		uint32_t reg = req[G3D_REG_WRITE_REG];
		uint32_t val = req[G3D_REG_WRITE_VAL];

		if (reg < G3D_NUM_PROT_REGISTERS) {
			uint32_t mask = ctx->register_masks[reg];
			val &= mask;
			val |= ctx->registers[reg] & ~mask;
			max_reg = 0x80000000;
		}

		ctx->registers[reg] = val;
		max_reg = max(max_reg, reg);
		req += NUM_G3D_REG_WRITE_WORDS;
	}

	if (max_reg < FGVS_ATTRIBUTE_NUM)
		dirty_level = G3D_FLUSH_VERTEX_FIFO;
	else if (max_reg < FGPE_VERTEX_CONTEXT)
		dirty_level = G3D_FLUSH_VERTEX_SHADER;
	else if (max_reg < FGRA_PIX_SAMP)
		dirty_level = G3D_FLUSH_PRIMITIVE;
	else if (max_reg < FGPF_ALPHAT)
		dirty_level = G3D_FLUSH_RASTER;
	else
		dirty_level = G3D_FLUSH_PER_FRAGMENT;

	g3d_state_mark_dirty(ctx, dirty_level);
}

static void g3d_apply_state_buffer(struct g3d_submit *submit,
				   const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	unsigned int count = req_length(req) / NUM_G3D_REG_WRITE_WORDS;

	req = req_data(req);

	while (count--) {
		uint32_t reg = req[G3D_REG_WRITE_REG];

		g3d_write_relaxed(g3d, ctx->registers[reg], g3d_registers[reg]);
		req += NUM_G3D_REG_WRITE_WORDS;
	}

	/* Make sure register writes are not reordered across this point. */
	wmb();
}

static struct exynos_drm_gem_obj *g3d_acquire_gem(struct g3d_submit *submit,
						  uint32_t *handle, bool write)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	struct idr *idr;
	int ret;

	obj = exynos_drm_gem_lookup(g3d->subdrv.drm_dev, ctx->file, *handle);
	if (!obj)
		return NULL;

	ret = g3d_prepare_gem_object(g3d, obj);
	if (ret)
		goto err_obj;

	idr = write ? &submit->write_gem_idr : &submit->read_gem_idr;
	ret = idr_alloc(idr, obj, 1, 0, GFP_KERNEL);
	if (ret < 0) {
		dev_err(g3d->dev,
			"failed to allocate GEM submission ID (%d)\n", ret);
		goto err_obj;
	}

	*handle = ret;
	return obj;

err_obj:
	drm_gem_object_unreference_unlocked(&obj->base);

	return NULL;
}

static void g3d_release_gem(struct g3d_submit *submit, uint32_t sid,
			    bool write)
{
	struct exynos_drm_gem_obj *obj;
	struct idr *idr;

	idr = write ? &submit->write_gem_idr : &submit->read_gem_idr;
	obj = idr_find(idr, sid);
	if (WARN_ON(!obj))
		return;

	idr_remove(idr, sid);
	drm_gem_object_unreference_unlocked(&obj->base);
}

static struct exynos_drm_gem_obj *g3d_lookup_gem(struct g3d_submit *submit,
						 uint32_t sid, bool write)
{
	struct exynos_drm_gem_obj *obj;
	struct idr *idr;

	idr = write ? &submit->write_gem_idr : &submit->read_gem_idr;
	obj = idr_find(idr, sid);
	BUG_ON(!obj);

	drm_gem_object_reference(&obj->base);

	return obj;
}

static struct exynos_drm_gem_obj *
g3d_lookup_gem_noref(struct g3d_submit *submit, uint32_t sid, bool write)
{
	struct exynos_drm_gem_obj *obj;
	struct idr *idr;

	idr = write ? &submit->write_gem_idr : &submit->read_gem_idr;
	obj = idr_find(idr, sid);
	BUG_ON(!obj);

	return obj;
}

/* Draw request */
static int g3d_validate_draw_buffer(struct g3d_submit *submit, uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	uint32_t offset, length;
	int ret;

	req = req_data_wr(req);

	if (!(req[G3D_DRAW_CONTROL] & G3D_DRAW_INDEXED))
		return 0;

	offset = req[G3D_DRAW_IB_OFFSET];
	length = req[G3D_DRAW_VERTICES];

	obj = g3d_acquire_gem(submit, &req[G3D_DRAW_IB_HANDLE], false);
	if (!obj) {
		dev_err(g3d->dev, "failed to lookup index BO\n");
		return -EINVAL;
	}

	if (unlikely(!obj->buffer->kvaddr)) {
		dev_err(g3d->dev, "index BO must have kernel mapping\n");
		ret = -EINVAL;
		goto err_obj;
	}

	if (unlikely(offset + length > obj->size)) {
		dev_err(g3d->dev, "index data bigger than BO\n");
		ret = -EINVAL;
		goto err_obj;
	}

	return 0;

err_obj:
	g3d_release_gem(submit, req[G3D_DRAW_IB_HANDLE], false);

	return ret;
}

static int g3d_handle_draw_buffer(struct g3d_submit *submit,
				   const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	unsigned int num_vertices;
	uint32_t handle, offset;
	const uint32_t *data;
	int ret;

	g3d_state_mark_dirty(ctx, G3D_FLUSH_VERTEX_CACHE);
	ret = g3d_prepare_draw(ctx, submit, req);
	if (ret)
		return ret;

	if (in_interrupt() && g3d->dma_enabled && g3d_dma_free_lli(g3d) < 4)
		return -EINVAL;

	req = req_data(req);

	num_vertices = req[G3D_DRAW_VERTICES];

	dev_dbg_reqs(g3d->dev, "%s (cnt = %u)\n", __func__, num_vertices);

	trace_g3d_draw_request(++g3d->num_draws);

	if (!(req[G3D_DRAW_CONTROL] & G3D_DRAW_INDEXED)) {
		if (g3d->dma_enabled) {
			g3d_dma_set(g3d, G3D_FGHI_CONTROL_REG, 0x80010009, 1);
			g3d_dma_set(g3d, G3D_FGHI_IDXOFFSET_REG, 0x00000001, 1);
			g3d_dma_set(g3d, G3D_FGHI_FIFO_ENTRY_REG,
					num_vertices, 1);
			g3d_dma_set(g3d, G3D_FGHI_FIFO_ENTRY_REG, 0, 1);
			g3d_dma_flush(g3d);

			return 0;
		}

		g3d_write_relaxed(g3d, 0x80010009, G3D_FGHI_CONTROL_REG);
		g3d_write(g3d, 0x00000001, G3D_FGHI_IDXOFFSET_REG);
		g3d_write(g3d, num_vertices, G3D_FGHI_FIFO_ENTRY_REG);
		g3d_write(g3d, 0, G3D_FGHI_FIFO_ENTRY_REG);

		return 0;
	}

	handle = req[G3D_DRAW_IB_HANDLE];
	offset = req[G3D_DRAW_IB_OFFSET];

	obj = g3d_lookup_gem_noref(submit, handle, false);
	data = obj->buffer->kvaddr + offset;

	if (g3d->dma_enabled) {
		dma_addr_t data_phys = obj->buffer->dma_addr + offset;

		g3d_dma_set(g3d, G3D_FGHI_CONTROL_REG, 0x83000019, 1);
		g3d_dma_set(g3d, G3D_FGHI_IDXOFFSET_REG, 0x00000000, 1);
		g3d_dma_set(g3d, G3D_FGHI_FIFO_ENTRY_REG, num_vertices, 1);

		num_vertices = ALIGN(num_vertices, 4);

		g3d_dma_copy(g3d, G3D_FGHI_FIFO_ENTRY_REG, data_phys,
				num_vertices / 4);
		g3d_dma_flush(g3d);

		return 0;
	}

	g3d_write_relaxed(g3d, 0x83000019, G3D_FGHI_CONTROL_REG);
	g3d_write(g3d, 0x00000000, G3D_FGHI_IDXOFFSET_REG);
	g3d_write(g3d, num_vertices, G3D_FGHI_FIFO_ENTRY_REG);

	num_vertices = ALIGN(num_vertices, 4);
	g3d_dump_draw_buffer(data, num_vertices);
	g3d_write_burst(g3d, G3D_FGHI_FIFO_ENTRY_REG, data, num_vertices);

	return 0;
}

/* Vertex buffer upload request */
static int g3d_validate_vertex_buffer(struct g3d_submit *submit, uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	uint32_t offset, dst_offset, length;
	int ret;

	req = req_data_wr(req);

	offset = req[G3D_VERTEX_BUF_OFFSET];
	dst_offset = req[G3D_VERTEX_BUF_DST_OFFSET];
	length = req[G3D_VERTEX_BUF_LENGTH];

	if (unlikely(dst_offset % 4 || length % 4)) {
		dev_err(g3d->dev, "unaligned vertex buffer access\n");
		return -EINVAL;
	}

	obj = g3d_acquire_gem(submit, &req[G3D_VERTEX_BUF_HANDLE], false);
	if (!obj) {
		dev_err(g3d->dev, "failed to lookup vertex BO\n");
		return -EINVAL;
	}

	if (unlikely(!obj->buffer->kvaddr)) {
		dev_err(g3d->dev, "vertex BO must have kernel mapping\n");
		ret = -EINVAL;
		goto err_obj;
	}

	if (unlikely(offset + length > obj->size)) {
		dev_err(g3d->dev, "vertex data bigger than BO\n");
		ret = -EINVAL;
		goto err_obj;
	}

	if (unlikely(dst_offset + length > G3D_VERTEX_BUFFER_SIZE)) {
		dev_err(g3d->dev, "vertex data exceeds vertex buffer\n");
		ret = -EINVAL;
		goto err_obj;
	}

	return 0;

err_obj:
	g3d_release_gem(submit, req[G3D_VERTEX_BUF_HANDLE], false);

	return ret;
}

static int g3d_handle_vertex_buffer(struct g3d_submit *submit,
				     const uint32_t *req)
{
	uint32_t handle, offset, dst_offset, length;
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	unsigned int leading, trailing;
	const uint32_t *data;
	int ret;

	g3d_state_mark_dirty(ctx, G3D_FLUSH_VERTEX_FIFO);

	ret = g3d_prepare_draw(ctx, submit, req);
	if (ret)
		return ret;

	if (in_interrupt() && g3d->dma_enabled && g3d_dma_free_lli(g3d) < 4)
		return -EINVAL;

	req = req_data(req);

	handle = req[G3D_VERTEX_BUF_HANDLE];
	offset = req[G3D_VERTEX_BUF_OFFSET];
	dst_offset = req[G3D_VERTEX_BUF_DST_OFFSET];
	length = ALIGN(req[G3D_VERTEX_BUF_LENGTH], 4);

	obj = g3d_lookup_gem_noref(submit, handle, false);
	data = obj->buffer->kvaddr + offset;

	dev_dbg_reqs(g3d->dev, "%s (len = %u)\n", __func__, length);
	g3d_dump_draw_buffer(data, length);

	leading = (dst_offset % 16) / 4;
	trailing = (4 - ((length + dst_offset) % 16) / 4) % 4;

	if (g3d->dma_enabled) {
		dma_addr_t data_phys = obj->buffer->dma_addr + offset;

		g3d_dma_set(g3d, G3D_FGHI_VBADDR_REG,
					dst_offset & ~0xf, 1);

		if (leading)
			g3d_dma_set(g3d, G3D_FGHI_VB_ENTRY_REG, 0, leading);

		g3d_dma_copy(g3d, G3D_FGHI_VB_ENTRY_REG, data_phys, length / 4);

		if (trailing)
			g3d_dma_set(g3d, G3D_FGHI_VB_ENTRY_REG, 0, trailing);

		return 0;
	}

	g3d_write(g3d, dst_offset & ~0xf, G3D_FGHI_VBADDR_REG);

	while (leading--)
		g3d_write_relaxed(g3d, 0, G3D_FGHI_VB_ENTRY_REG);
	g3d_write_burst(g3d, G3D_FGHI_VB_ENTRY_REG, data, length);
	while (trailing--)
		g3d_write_relaxed(g3d, 0, G3D_FGHI_VB_ENTRY_REG);

	return 0;
}

/* Shader program update request */
static inline uint8_t req_rsp_unit(const uint32_t *req)
{
	return (req[G3D_SHADER_PROG_UNIT_NATTRIB] >> 8) & 0xff;
}

static inline uint8_t req_rsp_nattrib(const uint32_t *req)
{
	return req[G3D_SHADER_PROG_UNIT_NATTRIB] & 0xff;
}

static inline uint16_t req_rsp_dcount(const uint32_t *req, unsigned int type)
{
	uint32_t word = req[G3D_SHADER_PROG_DCOUNT0 + type / 2];

	if (type % 2)
		word >>= 16;

	return word & 0xffff;
}

static int g3d_validate_shader_program(struct g3d_submit *submit, uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	uint32_t offset, length;
	unsigned int type;
	uint8_t unit;
	int ret;

	req = req_data_wr(req);

	unit = req_rsp_unit(req);
	offset = req[G3D_SHADER_PROG_OFFSET];
	length = req[G3D_SHADER_PROG_LENGTH];

	if (unlikely(unit >= G3D_NUM_SHADERS)) {
		dev_err(g3d->dev, "invalid shader index\n");
		return -EINVAL;
	}

	if (!length) {
		/* Shader detach request */
		return 0;
	}

	if (unlikely(length > G3D_SHADER_PROGRAM_MAX_SIZE)) {
		dev_err(g3d->dev, "shader program size above maximum\n");
		return -EINVAL;
	}

	obj = g3d_acquire_gem(submit, &req[G3D_SHADER_PROG_HANDLE], false);
	if (!obj) {
		dev_err(g3d->dev, "failed to lookup shader program BO\n");
		return -EINVAL;
	}

	if (unlikely(!obj->buffer->kvaddr)) {
		dev_err(g3d->dev,
			"shader program BO must have kernel mapping\n");
		ret = -EINVAL;
		goto err_obj;
	}

	if (unlikely(offset + length > obj->size)) {
		dev_err(g3d->dev, "shader program bigger than BO\n");
		ret = -EINVAL;
		goto err_obj;
	}

	for (type = 0; type < G3D_NUM_SHADER_DATA_TYPES; ++type) {
		uint16_t dcount = req_rsp_dcount(req, type);

		if (unlikely(dcount > g3d_shader_data_count[type])) {
			dev_err(g3d->dev, "requested shader data too big\n");
			ret = -EINVAL;
			goto err_obj;
		}
	}

	return 0;

err_obj:
	g3d_release_gem(submit, req[G3D_SHADER_PROG_HANDLE], false);

	return ret;
}

static void g3d_process_shader_program(struct g3d_submit *submit,
				       const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_shader_program *sp;
	uint32_t handle, length;
	uint8_t unit;

	req = req_data(req);

	unit = req_rsp_unit(req);
	handle = req[G3D_SHADER_PROG_HANDLE];
	length = req[G3D_SHADER_PROG_LENGTH];

	sp = &ctx->shader_program[unit];
	if (sp->obj) {
		drm_gem_object_unreference_unlocked(&sp->obj->base);
		sp->obj = NULL;
	}

	memcpy(sp->req, req, sizeof(sp->req));

	if (length)
		sp->obj = g3d_lookup_gem(submit, handle, false);

	if (unit == G3D_SHADER_PIXEL) {
		set_bit(G3D_CONTEXT_STOP_PIXEL_SHADER, &ctx->state);
		g3d_state_mark_dirty(ctx, G3D_FLUSH_PIXEL_SHADER);
	} else {
		g3d_state_mark_dirty(ctx, G3D_FLUSH_VERTEX_SHADER);
	}
}

#define VS_PC_RANGE(len)	((((len) / 16) - 1) << 16)
#define PS_PC_RANGE(len)	(((len) / 16) - 1)

static const uint32_t dummy_shader[] = {
	0x00000000, 0x00000000, 0x00000000, 0x00000000, /* NOP */
};

static void g3d_restore_shader_program(struct g3d_context *ctx,
				       struct g3d_shader_program *sp,
				       uint8_t unit)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	uint32_t offset, length;
	const uint32_t *data;
	const uint32_t *req;
	unsigned int words;
	unsigned int type;
	uint32_t reg;

	req = sp->req;

	offset = req[G3D_SHADER_PROG_OFFSET];
	length = req[G3D_SHADER_PROG_LENGTH];

	if (length) {
		data = sp->obj->buffer->kvaddr + offset;
	} else {
		data = dummy_shader;
		length = sizeof(dummy_shader);
	}

	reg = g3d_shader_base[unit];
	words = length / 4;

	while (words--) {
		g3d_write_relaxed(g3d, *(data++), reg);
		reg += 4;
	}

	for (type = 0; type < G3D_NUM_SHADER_DATA_TYPES; ++type) {
		data = sp->data + g3d_shader_type_offset[type];
		words = req_rsp_dcount(req, type);
		reg = g3d_shader_base[unit] + g3d_shader_data_reg_base[type];

		while (words--) {
			g3d_write_relaxed(g3d, *(data++), reg);
			reg += 4;
		}
	}

	switch (unit) {
	case G3D_SHADER_VERTEX:
		g3d_write(g3d, VS_PC_RANGE(length), G3D_FGVS_PC_RANGE_REG);
		g3d_write(g3d, 3, G3D_FGVS_CONFIG_REG);
		break;
	case G3D_SHADER_PIXEL:
		g3d_write(g3d, 0, G3D_FGPS_PC_START_REG);
		g3d_write(g3d, PS_PC_RANGE(length), G3D_FGPS_PC_END_REG);
		g3d_write(g3d, 1, G3D_FGPS_PC_COPY_REG);
		/* Hardcoded to 8 due to hardware bug. */
		g3d_write(g3d, 8, G3D_FGPS_ATTRIBUTE_NUM_REG);
		break;
	default:
		dev_warn(g3d->dev, "invalid shader type\n");
	}
}

static void g3d_apply_shader_program(struct g3d_submit *submit,
				     const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_shader_program *sp;
	uint8_t unit;

	req = req_data(req);

	unit = req_rsp_unit(req);
	sp = &ctx->shader_program[unit];

	g3d_restore_shader_program(ctx, sp, unit);
}

static void g3d_free_shader_program(struct g3d_shader_program *sp)
{
	if (sp->obj)
		drm_gem_object_unreference_unlocked(&sp->obj->base);
}

/* Shader data update request */
static inline uint8_t req_rsd_unit(const uint32_t *req)
{
	return (req[G3D_SHADER_DATA_UNIT_TYPE_OFFS] >> 24) & 0xff;
}

static inline uint8_t req_rsd_type(const uint32_t *req)
{
	return (req[G3D_SHADER_DATA_UNIT_TYPE_OFFS] >> 16) & 0xff;
}

static inline uint16_t req_rsd_offset(const uint32_t *req)
{
	return req[G3D_SHADER_DATA_UNIT_TYPE_OFFS] & 0xff;
}

static const uint32_t *req_rsd_data(const uint32_t *req)
{
	return req + G3D_SHADER_DATA_WORDS_BASE;
}

static int g3d_validate_shader_data(struct g3d_submit *submit, uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	uint8_t unit, type;
	uint16_t offset;
	uint32_t length;

	length = req_length(req);
	if (unlikely(length < G3D_SHADER_DATA_WORDS_BASE)) {
		dev_err(g3d->dev, "malformed shader data request\n");
		return -EINVAL;
	}

	req = req_data_wr(req);

	unit = req_rsd_unit(req);
	type = req_rsd_type(req);
	offset = req_rsd_offset(req);
	length -= G3D_SHADER_DATA_WORDS_BASE;

	if (unlikely(unit >= G3D_NUM_SHADERS)) {
		dev_err(g3d->dev, "invalid shader index\n");
		return -EINVAL;
	}

	if (unlikely(type >= G3D_NUM_SHADER_DATA_TYPES)) {
		dev_err(g3d->dev, "invalid shader data type\n");
		return -EINVAL;
	}

	if (unlikely(offset + length > g3d_shader_data_count[type])) {
		dev_err(g3d->dev, "shader data access out of range\n");
		return -EINVAL;
	}

	return 0;
}

static void g3d_process_shader_data(struct g3d_submit *submit,
				    const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_shader_program *sp;
	uint8_t unit, type;
	uint16_t offset;
	uint32_t length;

	length = req_length(req);

	req = req_data(req);

	unit = req_rsd_unit(req);
	type = req_rsd_type(req);
	offset = req_rsd_offset(req);
	length -= G3D_SHADER_DATA_WORDS_BASE;

	sp = &ctx->shader_program[unit];

	memcpy(sp->data + g3d_shader_type_offset[type] + offset,
		req_rsd_data(req), length * sizeof(uint32_t));

	if (unit == G3D_SHADER_PIXEL) {
		g3d_state_mark_dirty(ctx, G3D_FLUSH_PIXEL_SHADER);
		set_bit(G3D_CONTEXT_STOP_PIXEL_SHADER, &ctx->state);
	} else {
		g3d_state_mark_dirty(ctx, G3D_FLUSH_VERTEX_SHADER);
	}
}

static void g3d_apply_shader_data(struct g3d_submit *submit,
				  const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	const uint32_t *data;
	uint8_t unit, type;
	uint32_t length;
	uint16_t offset;
	uint32_t reg;

	length = req_length(req);

	req = req_data(req);

	unit = req_rsd_unit(req);
	type = req_rsd_type(req);
	offset = req_rsd_offset(req);
	length -= G3D_SHADER_DATA_WORDS_BASE;

	data = req_rsd_data(req);
	reg = g3d_shader_base[unit] + g3d_shader_data_reg_base[type];
	reg += offset * 4;

	while (length--) {
		g3d_write_relaxed(g3d, *(data++), reg);
		reg += 4;
	}

	/* Make sure register writes are not reordered across this point. */
	wmb();
}

/* Texture setup request */
static const u8 g3d_texture_bpp[32] = {
	16, /* 1555 */
	16, /* 565 */
	16, /* 4444 */
	16, /* depth 16 */
	16, /* 88 */
	 8, /* 8 */
	32, /* 8888 */
	 1, /* 1 bpp */
	 2, /* 2 bpp */
	 4, /* 4 bpp */
	 8, /* 8 bpp */
	 4, /* S3TC (DXT1) */
	12, /* YUV422 (Y1VY0U) */
	12, /* YUV422 (VY1UY0) */
	12, /* YUV422 (Y1UY0V) */
	12, /* YUV422 (UY1VY0) */
	/* remaining format codes are reserved */
};

static inline uint8_t req_rts_unit(const uint32_t *req)
{
	return (req[G3D_TEXTURE_FLAGS] >> 24) & 0xff;
}

static int g3d_validate_texture(struct g3d_submit *submit, uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	uint32_t flags;
	uint8_t unit;

	req = req_data_wr(req);

	unit = req_rts_unit(req);
	flags = req[G3D_TEXTURE_FLAGS];

	if (unlikely(unit > G3D_NUM_TEXTURES)) {
		dev_err(g3d->dev, "invalid texture unit\n");
		return -EINVAL;
	}

	if (flags & G3D_TEXTURE_DETACH)
		return 0;

	obj = g3d_acquire_gem(submit, &req[G3D_TEXTURE_HANDLE], false);
	if (!obj) {
		dev_err(g3d->dev, "failed to lookup texture BO\n");
		return -EINVAL;
	}

	/* TODO: do some checks */
	req[G3D_TEXTURE_BASE_OFFSET] += obj->buffer->dma_addr;

	if (flags & G3D_TEXTURE_DIRTY)
		obj->g3d_priv->tex_timestamp = g3d->tex_timestamp;

	return 0;
}

static void g3d_process_texture(struct g3d_submit *submit, const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_texture *tex;
	uint32_t handle, flags;
	uint8_t unit;

	req = req_data(req);

	handle = req[G3D_TEXTURE_HANDLE];
	flags = req[G3D_TEXTURE_FLAGS];
	unit = req_rts_unit(req);

	tex = &ctx->texture[unit];
	if (tex->obj) {
		if (tex->prev_obj)
			drm_gem_object_unreference_unlocked(&tex->obj->base);
		else
			tex->prev_obj = tex->obj;
		tex->obj = NULL;
	}

	memcpy(tex->req, req, sizeof(tex->req));

	if (flags & G3D_TEXTURE_DETACH) {
		/* Disable texture request */
		return;
	}

	tex->obj = g3d_lookup_gem(submit, handle, false);

	g3d_state_mark_dirty(ctx, G3D_FLUSH_PIXEL_SHADER);
}

static void g3d_flush_texture(struct g3d_drvdata *g3d,
			      struct exynos_drm_gem_obj *obj)
{
	struct exynos_drm_gem_g3d_priv *priv = obj->g3d_priv;

	if (g3d->tex_timestamp <= priv->tex_timestamp) {
		priv->tex_timestamp = g3d->tex_timestamp;
		g3d_invalidate_caches(g3d);
	}
}

static void g3d_restore_texture(struct g3d_context *ctx,
				struct g3d_texture *tex, uint8_t unit)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	const uint32_t *req = tex->req;
	unsigned int words;
	uint32_t reg;

	if (tex->prev_obj) {
		drm_gem_object_unreference_unlocked(&tex->prev_obj->base);
		tex->prev_obj = NULL;
	}

	reg = G3D_TEXTURE_BASE(unit);

	if (!tex->obj) {
		/* Texture detach request */
		g3d_write(g3d, 0, reg + G3D_TEXTURE_BASE_OFFSET);
		return;
	}

	g3d_flush_texture(g3d, tex->obj);

	words = G3D_TEXTURE_HANDLE - G3D_TEXTURE_CONTROL;

	while (words--) {
		g3d_write_relaxed(g3d, *(req++), reg);
		reg += 4;
	}

	/* Make sure register writes are not reordered across this point. */
	wmb();
}

static void g3d_apply_texture(struct g3d_submit *submit, const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_texture *tex;
	uint8_t unit;

	req = req_data(req);

	unit = req_rts_unit(req);
	tex = &ctx->texture[unit];

	g3d_restore_texture(ctx, tex, unit);
}

static void g3d_free_texture(struct g3d_texture *tex)
{
	if (tex->obj)
		drm_gem_object_unreference_unlocked(&tex->obj->base);
	if (tex->prev_obj)
		drm_gem_object_unreference_unlocked(&tex->prev_obj->base);
}

/* Vertex texture setup request */
static inline uint8_t req_rvt_unit(const uint32_t *req)
{
	return (req[G3D_VTX_TEXTURE_FLAGS] >> 24) & 0xff;
}

static int g3d_validate_vtx_texture(struct g3d_submit *submit, uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	uint32_t flags;
	uint8_t unit;

	req = req_data_wr(req);

	unit = req_rvt_unit(req);
	flags = req[G3D_VTX_TEXTURE_FLAGS];

	if (unlikely(unit > G3D_NUM_VTX_TEXTURES)) {
		dev_err(g3d->dev, "invalid vtx texture unit\n");
		return -EINVAL;
	}

	if (flags & G3D_VTX_TEXTURE_DETACH)
		return 0;

	obj = g3d_acquire_gem(submit, &req[G3D_VTX_TEXTURE_HANDLE], false);
	if (!obj) {
		dev_err(g3d->dev, "failed to lookup vtx texture BO\n");
		return -EINVAL;
	}

	/* TODO: do some checks */
	req[G3D_VTX_TEXTURE_BASE_OFFSET] += obj->buffer->dma_addr;

	if (flags & G3D_VTX_TEXTURE_DIRTY)
		obj->g3d_priv->tex_timestamp = g3d->tex_timestamp;

	return 0;
}

static void g3d_process_vtx_texture(struct g3d_submit *submit,
				    const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_vtx_texture *tex;
	uint32_t handle, flags;
	uint8_t unit;

	req = req_data(req);

	handle = req[G3D_VTX_TEXTURE_HANDLE];
	flags = req[G3D_VTX_TEXTURE_FLAGS];
	unit = req_rvt_unit(req);

	tex = &ctx->vtx_texture[unit];
	if (tex->obj) {
		if (tex->prev_obj)
			drm_gem_object_unreference_unlocked(&tex->obj->base);
		else
			tex->prev_obj = tex->obj;
		tex->obj = NULL;
	}

	memcpy(tex->req, req, sizeof(tex->req));

	if (flags & G3D_VTX_TEXTURE_DETACH) {
		/* Disable vtx_texture request */
		return;
	}

	tex->obj = g3d_lookup_gem(submit, handle, false);

	g3d_state_mark_dirty(ctx, G3D_FLUSH_VERTEX_SHADER);
}

static void g3d_restore_vtx_texture(struct g3d_context *ctx,
				struct g3d_vtx_texture *tex, uint8_t unit)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	const uint32_t *req = tex->req;
	unsigned int words;
	uint32_t reg;

	if (tex->prev_obj) {
		drm_gem_object_unreference_unlocked(&tex->prev_obj->base);
		tex->prev_obj = NULL;
	}

	reg = G3D_VTX_TEXTURE_BASE(unit);

	if (!tex->obj) {
		/* Texture detach request */
		g3d_write(g3d, 0, reg + G3D_VTX_TEXTURE_BASE_OFFSET);
		return;
	}

	g3d_flush_texture(g3d, tex->obj);

	words = G3D_VTX_TEXTURE_HANDLE - G3D_VTX_TEXTURE_CONTROL;

	while (words--) {
		g3d_write_relaxed(g3d, *(req++), reg);
		reg += 4;
	}

	/* Make sure register writes are not reordered across this point. */
	wmb();
}

static void g3d_apply_vtx_texture(struct g3d_submit *submit,
				  const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_vtx_texture *tex;
	uint8_t unit;

	req = req_data(req);

	unit = req_rvt_unit(req);
	tex = &ctx->vtx_texture[unit];

	g3d_restore_vtx_texture(ctx, tex, unit);
}

static void g3d_free_vtx_texture(struct g3d_vtx_texture *tex)
{
	if (tex->obj)
		drm_gem_object_unreference_unlocked(&tex->obj->base);
	if (tex->prev_obj)
		drm_gem_object_unreference_unlocked(&tex->prev_obj->base);
}

/* Color buffer setup request */
static int g3d_validate_colorbuffer(struct g3d_submit *submit, uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	uint32_t flags;

	req = req_data_wr(req);

	flags = req[G3D_COLORBUFFER_FLAGS];

	if (flags & G3D_COLORBUFFER_DETACH)
		return 0;

	obj = g3d_acquire_gem(submit, &req[G3D_COLORBUFFER_HANDLE], true);
	if (!obj) {
		dev_err(g3d->dev, "failed to lookup cbuffer BO\n");
		return -EINVAL;
	}

	if (flags & G3D_COLORBUFFER_DIRTY)
		obj->g3d_priv->fb_timestamp = g3d->fb_timestamp;

	/* TODO: do some checks */
	req[G3D_COLORBUFFER_OFFSET] += obj->buffer->dma_addr;

	return 0;
}

static void g3d_flush_colorbuffer(struct g3d_drvdata *g3d,
				  struct exynos_drm_gem_obj *obj)
{
	struct exynos_drm_gem_g3d_priv *priv = obj->g3d_priv;

	if (g3d->fb_timestamp <= priv->fb_timestamp) {
		priv->fb_timestamp = g3d->fb_timestamp;
		g3d_flush_caches(g3d);
	}
}

static void g3d_process_colorbuffer(struct g3d_submit *submit,
				    const uint32_t *req)
{
	uint32_t handle, fbctl, offset, width, flags;
	struct g3d_context *ctx = submit->ctx;
	struct g3d_colorbuffer *cb;

	req = req_data(req);

	handle = req[G3D_COLORBUFFER_HANDLE];
	fbctl = req[G3D_COLORBUFFER_FBCTL];
	offset = req[G3D_COLORBUFFER_OFFSET];
	width = req[G3D_COLORBUFFER_WIDTH];
	flags = req[G3D_COLORBUFFER_FLAGS];

	cb = &ctx->colorbuffer;
	if (cb->obj) {
		if (cb->prev_obj)
			drm_gem_object_unreference_unlocked(&cb->obj->base);
		else
			cb->prev_obj = cb->obj;
		cb->obj = NULL;
	}

	if (flags & G3D_COLORBUFFER_DETACH) {
		/* Framebuffer disable request */
		g3d_reg_cache_write_masked(ctx, 0, FGPF_FBCTL);
		g3d_reg_cache_write_masked(ctx, 0, FGPF_CBADDR);
		g3d_reg_cache_write_masked(ctx, width, FGPF_FBW);
		return;
	}

	g3d_reg_cache_write_masked(ctx, fbctl, FGPF_FBCTL);
	g3d_reg_cache_write_masked(ctx, offset, FGPF_CBADDR);
	g3d_reg_cache_write_masked(ctx, width, FGPF_FBW);

	cb->obj = g3d_lookup_gem(submit, handle, true);
	cb->obj->g3d_priv->tex_timestamp = ctx->g3d->tex_timestamp;

	g3d_state_mark_dirty(ctx, G3D_FLUSH_PER_FRAGMENT);
}

static void g3d_apply_colorbuffer(struct g3d_submit *submit,
				  const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct g3d_colorbuffer *cb = &ctx->colorbuffer;

	if (cb->prev_obj) {
		g3d_flush_colorbuffer(g3d, cb->prev_obj);
		drm_gem_object_unreference_unlocked(&cb->prev_obj->base);
		cb->prev_obj = NULL;
	}

	g3d_reg_cache_sync(g3d, ctx, FGPF_FBCTL);
	g3d_reg_cache_sync(g3d, ctx, FGPF_CBADDR);
	g3d_reg_cache_sync(g3d, ctx, FGPF_FBW);

	/* Make sure register writes are not reordered across this point. */
	wmb();
}

static void g3d_free_colorbuffer(struct g3d_colorbuffer *cb)
{
	if (cb->obj)
		drm_gem_object_unreference_unlocked(&cb->obj->base);
	if (cb->prev_obj)
		drm_gem_object_unreference_unlocked(&cb->prev_obj->base);
}

/* Depth buffer setup request */
static int g3d_validate_depthbuffer(struct g3d_submit *submit, uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *obj;
	uint32_t flags;

	req = req_data_wr(req);

	flags = req[G3D_DEPTHBUFFER_FLAGS];

	if (flags & G3D_DEPTHBUFFER_DETACH)
		return 0;

	obj = g3d_acquire_gem(submit, &req[G3D_DEPTHBUFFER_HANDLE], true);
	if (!obj) {
		dev_err(g3d->dev, "failed to lookup dbuffer BO\n");
		return -EINVAL;
	}

	if (flags & G3D_DEPTHBUFFER_DIRTY)
		obj->g3d_priv->fb_timestamp = g3d->fb_timestamp;

	/* TODO: do some checks */
	req[G3D_DEPTHBUFFER_OFFSET] += obj->buffer->dma_addr;

	return 0;
}

static void g3d_flush_depthbuffer(struct g3d_drvdata *g3d,
				  struct exynos_drm_gem_obj *obj)
{
	struct exynos_drm_gem_g3d_priv *priv = obj->g3d_priv;

	if (g3d->fb_timestamp <= priv->fb_timestamp) {
		priv->fb_timestamp = g3d->fb_timestamp;
		g3d_flush_caches(g3d);
	}
}

static void g3d_process_depthbuffer(struct g3d_submit *submit,
				    const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_depthbuffer *db;
	uint32_t handle, offset, flags;

	req = req_data(req);

	handle = req[G3D_DEPTHBUFFER_HANDLE];
	offset = req[G3D_DEPTHBUFFER_OFFSET];
	flags = req[G3D_DEPTHBUFFER_FLAGS];

	db = &ctx->depthbuffer;
	if (db->obj) {
		if (db->prev_obj)
			drm_gem_object_unreference_unlocked(&db->obj->base);
		else
			db->prev_obj = db->obj;
		db->obj = NULL;
	}

	if (flags & G3D_DEPTHBUFFER_DETACH) {
		/* Depth buffer disable request */
		g3d_reg_cache_write(ctx, 0, FGPF_DBADDR);

		ctx->register_masks[FGPF_FRONTST] = 0;
		ctx->register_masks[FGPF_DEPTHT] = 0;
		ctx->register_masks[FGPF_DBMSK] = 0;

		g3d_reg_cache_write(ctx, 0, FGPF_FRONTST);
		g3d_reg_cache_write(ctx, 0, FGPF_DEPTHT);
		g3d_reg_cache_write(ctx, 0xffff0001, FGPF_DBMSK);

		return;
	}

	g3d_reg_cache_write(ctx, offset, FGPF_DBADDR);

	ctx->register_masks[FGPF_FRONTST] = 0xffffffff;
	ctx->register_masks[FGPF_DEPTHT] = 0xffffffff;
	ctx->register_masks[FGPF_DBMSK] = 0xffffffff;

	db->obj = g3d_lookup_gem(submit, handle, true);
	db->obj->g3d_priv->tex_timestamp = ctx->g3d->tex_timestamp;

	g3d_state_mark_dirty(ctx, G3D_FLUSH_PER_FRAGMENT);
}

static void g3d_apply_depthbuffer(struct g3d_submit *submit,
				  const uint32_t *req)
{
	struct g3d_context *ctx = submit->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct g3d_depthbuffer *db = &ctx->depthbuffer;

	if (db->prev_obj) {
		g3d_flush_depthbuffer(g3d, db->prev_obj);
		drm_gem_object_unreference_unlocked(&db->prev_obj->base);
		db->prev_obj = NULL;
	}

	g3d_reg_cache_sync(g3d, ctx, FGPF_DBADDR);
	g3d_reg_cache_sync(g3d, ctx, FGPF_FRONTST);
	g3d_reg_cache_sync(g3d, ctx, FGPF_DEPTHT);
	g3d_reg_cache_sync(g3d, ctx, FGPF_DBMSK);

	/* Make sure register writes are not reordered across this point. */
	wmb();
}

static void g3d_free_depthbuffer(struct g3d_depthbuffer *db)
{
	if (db->obj)
		drm_gem_object_unreference_unlocked(&db->obj->base);
	if (db->prev_obj)
		drm_gem_object_unreference_unlocked(&db->prev_obj->base);
}

/* Supported request descriptors */
static const struct g3d_request_info g3d_requests[G3D_NUM_REQUESTS] = {
	[G3D_REQUEST_REGISTER_WRITE] = {
		.validate = g3d_validate_state_buffer,
		.update_state = g3d_process_state_buffer,
		.apply = g3d_apply_state_buffer,
		.length = G3D_STATE_BUFFER_MAX_SIZE,
	},
	[G3D_REQUEST_SHADER_PROGRAM] = {
		.validate = g3d_validate_shader_program,
		.update_state = g3d_process_shader_program,
		.apply = g3d_apply_shader_program,
		.flags = G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = NUM_G3D_SHADER_PROG_WORDS,
	},
	[G3D_REQUEST_SHADER_DATA] = {
		.validate = g3d_validate_shader_data,
		.update_state = g3d_process_shader_data,
		.apply = g3d_apply_shader_data,
		.length = G3D_SHADER_DATA_WORDS_BASE + G3D_CONST_REG_WORDS,
	},
	[G3D_REQUEST_TEXTURE] = {
		.validate = g3d_validate_texture,
		.update_state = g3d_process_texture,
		.apply = g3d_apply_texture,
		.flags = G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = NUM_G3D_TEXTURE_WORDS,
	},
	[G3D_REQUEST_COLORBUFFER] = {
		.validate = g3d_validate_colorbuffer,
		.update_state = g3d_process_colorbuffer,
		.apply = g3d_apply_colorbuffer,
		.flags = G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = NUM_G3D_COLORBUFFER_WORDS,
	},
	[G3D_REQUEST_DEPTHBUFFER] = {
		.validate = g3d_validate_depthbuffer,
		.update_state = g3d_process_depthbuffer,
		.apply = g3d_apply_depthbuffer,
		.flags = G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = NUM_G3D_DEPTHBUFFER_WORDS,
	},
	[G3D_REQUEST_DRAW] = {
		.validate = g3d_validate_draw_buffer,
		.handle = g3d_handle_draw_buffer,
		.flags = G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = NUM_G3D_DRAW_WORDS,
	},
	[G3D_REQUEST_VERTEX_BUFFER] = {
		.validate = g3d_validate_vertex_buffer,
		.handle = g3d_handle_vertex_buffer,
		.flags = G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = NUM_G3D_VERTEX_BUF_WORDS,
	},
	[G3D_REQUEST_VTX_TEXTURE] = {
		.validate = g3d_validate_vtx_texture,
		.update_state = g3d_process_vtx_texture,
		.apply = g3d_apply_vtx_texture,
		.flags = G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = NUM_G3D_VTX_TEXTURE_WORDS,
	}
};

/*
 * Command thread
 */

static void g3d_do_idle(struct g3d_drvdata *g3d)
{
	if (test_bit(G3D_STATE_IDLE, &g3d->state)) {
		dev_dbg_events(g3d->dev, "%s (already disabled)\n", __func__);
		return;
	}

	g3d_flush_pipeline(g3d, G3D_FLUSH_COLOR_CACHE, true);

	if (!list_empty(&g3d->submit_list)) {
		dev_dbg_events(g3d->dev, "%s (commands ready)\n", __func__);
		return;
	}

	clk_disable_unprepare(g3d->clock);

	pm_runtime_mark_last_busy(g3d->dev);
	pm_runtime_put_autosuspend(g3d->dev);

	set_bit(G3D_STATE_IDLE, &g3d->state);

	dev_dbg_events(g3d->dev, "%s (idle)\n", __func__);
}

static inline bool fence_completed(struct g3d_drvdata *g3d, uint32_t fence)
{
	return g3d->completed_fence >= fence;
}

static int g3d_wait_interruptible_timeout(struct g3d_drvdata *g3d,
					  uint32_t fence,
					  struct timespec *timeout)
{
	unsigned long remaining_jiffies;
	unsigned long timeout_jiffies;
	unsigned long start_jiffies;
	int ret;

	if (fence > g3d->submitted_fence) {
		dev_err(g3d->dev, "waiting on invalid fence: %u (of %u)\n",
			fence, g3d->submitted_fence);
		return -EINVAL;
	}

	if (!timeout) {
		/* no-wait: */
		if (fence_completed(g3d, fence))
			return 0;

		return -EBUSY;
	}

	timeout_jiffies = timespec_to_jiffies(timeout);
	start_jiffies = jiffies;

	trace_g3d_fence_wait_request(fence);

	if (!timeout_jiffies) {
		ret = wait_event_interruptible(g3d->fence_wq,
						fence_completed(g3d, fence));
	} else {
		if (time_after(start_jiffies, timeout_jiffies))
			remaining_jiffies = 0;
		else
			remaining_jiffies = timeout_jiffies - start_jiffies;

		ret = wait_event_interruptible_timeout(g3d->fence_wq,
						fence_completed(g3d, fence),
						remaining_jiffies);
		if (ret == 0) {
			dev_dbg_events(g3d->dev,
				"timeout waiting for fence: %u (completed: %u)",
				fence, g3d->completed_fence);
			ret = -ETIMEDOUT;
		} else if (ret > 0) {
			ret = 0;
		}
	}

	trace_g3d_fence_wait_complete(fence, ret);

	return ret;
}

/* Fence request */
static int g3d_signal_fence(struct g3d_drvdata *g3d, uint32_t fence)
{
	g3d->completed_fence = fence;

	wake_up_all(&g3d->fence_wq);

	return 0;
}

static int g3d_gem_idr_cleanup(int id, void *p, void *data)
{
	struct exynos_drm_gem_obj *obj = p;

	drm_gem_object_unreference_unlocked(&obj->base);

	return 0;
}

static void g3d_release_submit(struct g3d_submit *submit)
{
	idr_for_each(&submit->read_gem_idr, g3d_gem_idr_cleanup, NULL);
	idr_for_each(&submit->write_gem_idr, g3d_gem_idr_cleanup, NULL);
	idr_destroy(&submit->read_gem_idr);
	idr_destroy(&submit->write_gem_idr);
	vfree(submit);
}

static void g3d_release_context(struct kref *kref);

static void g3d_process_submit(struct g3d_drvdata *g3d,
			       struct g3d_submit *submit)
{
	struct g3d_context *ctx = submit->ctx;
	int ret;

	g3d->submit = submit;
	submit->cur = submit->words;
	submit->end = submit->words + submit->num_words;

	submit->state_update_start = submit->cur;
	submit->apply_start = submit->cur;

	trace_g3d_gpu_request(submit->fence);

	while (submit->cur < submit->end) {
		if (test_bit(G3D_CONTEXT_ABORTED, &ctx->state))
			break;

		ret = g3d_request_handle(submit, submit->cur);
		if (ret != -EAGAIN)
			submit->cur = req_data(submit->cur)
						+ req_length(submit->cur);
	}

	g3d->submit = NULL;

	if (!test_bit(G3D_STATE_IDLE, &g3d->state)) {
		g3d_flush_pipeline(g3d, G3D_FLUSH_COLOR_CACHE, false);
		g3d_flush_caches(g3d);
	}

	trace_g3d_gpu_complete(submit->fence);

	g3d_signal_fence(g3d, submit->fence);

	g3d_release_submit(submit);

	kref_put(&ctx->ref, g3d_release_context);
}

/* Main loop of virtual request processor */
static int g3d_command_thread(void *data)
{
	struct g3d_drvdata *g3d = (struct g3d_drvdata *)data;
	struct g3d_submit *submit;

	int ret;

	allow_signal(SIGTERM);
	set_freezable();

	while (!kthread_should_stop()) {
		spin_lock(&g3d->ready_lock);

		while (list_empty(&g3d->submit_list)) {
			spin_unlock(&g3d->ready_lock);

			g3d_do_idle(g3d);

			ret = wait_event_freezable(g3d->ready_wq,
					!list_empty(&g3d->submit_list)
					|| kthread_should_stop());
			if (ret && kthread_should_stop())
				goto finish;

			dev_dbg_events(g3d->dev, "%s:%d ready_wq wake-up\n",
					__func__, __LINE__);

			spin_lock(&g3d->ready_lock);
		}

		submit = list_first_entry(&g3d->submit_list,
						struct g3d_submit, list);
		list_del(&submit->list);

		spin_unlock(&g3d->ready_lock);

		g3d_process_submit(g3d, submit);
	}

finish:
	return 0;
}

/*
 * IRQ handler
 */
static void g3d_watchdog_timer(unsigned long data)
{
	struct g3d_drvdata *g3d = (struct g3d_drvdata *)data;

	if (!g3d_dma_idle(g3d)) {
		dev_err(g3d->dev, "DMA transfer timed out\n");

		writel(0, g3d->dma_base + G3D_DMA_CCFG_REG);
	}

	g3d_idle_irq_ack_and_disable(g3d);

	dev_err(g3d->dev, "pipeline flush timed out (stat = %08x)\n",
					g3d_read(g3d, G3D_FGGB_PIPESTAT_REG));
	g3d_initialize(g3d);

	complete(&g3d->completion);
	wake_up_interruptible(&g3d->idle_wq);
}

static int g3d_schedule_from_irq(struct g3d_drvdata *g3d)
{
	struct g3d_submit *submit = g3d->submit;
	struct g3d_context *ctx;
	int ret;

	if (!submit)
		return 0;

	ctx = submit->ctx;

	while (submit->cur < submit->end) {
		if (test_bit(G3D_CONTEXT_ABORTED, &ctx->state))
			break;

		ret = g3d_request_handle(submit, submit->cur);
		if (ret)
			return ret;

		submit->cur = req_data(submit->cur)
					+ req_length(submit->cur);
	}

	return 0;
}

static irqreturn_t g3d_handle_irq(int irq, void *dev_id)
{
	struct g3d_drvdata *g3d = (struct g3d_drvdata *)dev_id;
	int ret;

	g3d_idle_irq_ack_and_disable(g3d);

	del_timer(&g3d->watchdog_timer);

	dev_dbg_events(g3d->dev, "%s:%d\n", __func__, __LINE__);

	ret = g3d_schedule_from_irq(g3d);
	if (ret == -EAGAIN)
		return IRQ_HANDLED;

	dev_dbg_events(g3d->dev, "%s:%d waking up flush/idle_wq\n",
			__func__, __LINE__);
	complete(&g3d->completion);
	wake_up_interruptible(&g3d->idle_wq);

	return IRQ_HANDLED;
}

/*
 * Request submission
 */
static int g3d_validate_request(struct g3d_drvdata *g3d,
				struct g3d_submit *submit, uint32_t *req)
{
	const struct g3d_request_info *info;
	uint32_t length = req_length(req);
	uint8_t type = req_type(req);
	int ret;

	dev_dbg_reqs(g3d->dev, "%s: type = %02x, length = %u\n",
			__func__, type, length);
	g3d_dump_req_data(req_data(req), length);

	if (type >= G3D_NUM_REQUESTS) {
		dev_err(g3d->dev, "invalid request type %d\n", type);
		return -EINVAL;
	}

	info = &g3d_requests[type];

	if (info->length) {
		if (info->flags & G3D_REQUEST_STRICT_SIZE_CHECK
		    && unlikely(length != info->length)) {
			dev_err(g3d->dev, "invalid request data size (%u != %u, type = %u)\n",
				length, info->length, type);
			return -EINVAL;
		}

		if (unlikely(length > info->length)) {
			dev_err(g3d->dev, "request data size too big (%u > %u, type = %u)\n",
				length, info->length, type);
			return -EINVAL;
		}
	}

	if (info->validate && (ret = info->validate(submit, req))) {
		dev_err(g3d->dev,
			"failed to validate request data (type = %d)\n", type);
		return ret;
	}

	return 0;
}

struct g3d_idr_set_fence {
	uint32_t fence;
	bool write;
};

static int g3d_gem_idr_set_fence(int id, void *p, void *data)
{
	struct g3d_idr_set_fence *sf = data;
	struct exynos_drm_gem_obj *obj = p;

	if (sf->write)
		obj->g3d_priv->write_fence = sf->fence;
	else
		obj->g3d_priv->read_fence = sf->fence;

	return 0;
}

static uint32_t g3d_post_requests(struct g3d_drvdata *g3d,
				  struct g3d_submit *submit)
{
	struct g3d_idr_set_fence sf;
	bool wake;
	uint32_t fence;

	spin_lock(&g3d->ready_lock);

	fence = atomic_inc_return(&g3d->next_fence);
	submit->fence = fence;
	g3d->submitted_fence = fence;
	submit->ctx->last_fence = fence;

	sf.fence = fence;
	sf.write = false;
	idr_for_each(&submit->read_gem_idr, g3d_gem_idr_set_fence, &sf);
	sf.write = true;
	idr_for_each(&submit->write_gem_idr, g3d_gem_idr_set_fence, &sf);

	wake = list_empty(&g3d->submit_list);
	list_add_tail(&submit->list, &g3d->submit_list);

	if (wake) {
		dev_dbg_events(g3d->dev, "%s:%d waking up ready/idle_wq\n",
			__func__, __LINE__);
		wake_up_interruptible_sync(&g3d->ready_wq);
		wake_up_interruptible_sync(&g3d->idle_wq);
	}

	spin_unlock(&g3d->ready_lock);

	return fence;
}

static struct g3d_context *g3d_get_context(struct g3d_priv *priv, int id)
{
	struct g3d_drvdata *g3d = priv->g3d;
	struct g3d_context *ctx;

	ctx = idr_find(&priv->pipes_idr, id);
	if (!ctx) {
		dev_err(g3d->dev, "invalid pipe ID %u\n", id);
		ctx = ERR_PTR(-ENOENT);
	}

	return ctx;
}

int s3c6410_g3d_submit(struct drm_device *dev, void *data,
		       struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_priv *priv = file_priv->g3d_priv;
	struct drm_exynos_g3d_submit *drm_submit = data;
	struct g3d_drvdata *g3d = priv->g3d;
	struct exynos_drm_gem_obj *gem;
	struct g3d_submit *submit;
	struct g3d_context *ctx;
	uint32_t offset, length;
	uint32_t *req, *end;
	int ret;

	offset = drm_submit->offset;
	length = drm_submit->length;

	dev_dbg_fops(g3d->dev,
			"%s: handle = %08x, offset = %08x, length = %08x\n",
			__func__, drm_submit->handle, offset,
			length);

	down_read(&priv->pipes_idr_sem);

	ctx = g3d_get_context(priv, drm_submit->pipe);
	if (IS_ERR(ctx)) {
		ret = PTR_ERR(ctx);
		goto err_unlock_idr;
	}

	mutex_lock(&ctx->submit_mutex);

	gem = exynos_drm_gem_lookup(dev, file, drm_submit->handle);
	if (!gem) {
		dev_err(g3d->dev, "failed to lookup submit buffer\n");
		ret = -EINVAL;
		goto err_unlock_ctx;
	}

	if (unlikely(!gem->buffer->kvaddr)) {
		dev_err(g3d->dev, "submit buffers need to have kernel mapping\n");
		ret = -EINVAL;
		goto err_gem_unref;
	}

	if (unlikely(offset + length > gem->size)) {
		dev_err(g3d->dev, "submit data bigger than backing buffer\n");
		ret = -EINVAL;
		goto err_gem_unref;
	}

	if (unlikely(offset % 4 || length % 4)) {
		dev_err(g3d->dev, "submit data incorrectly aligned\n");
		ret = -EINVAL;
		goto err_gem_unref;
	}

	submit = vmalloc(sizeof(*submit) + length);
	if (!submit) {
		ret = -ENOMEM;
		goto err_gem_unref;
	}

	memcpy(submit->words, gem->buffer->kvaddr + offset, length);

	submit->ctx = ctx;
	submit->num_words = length / 4;
	idr_init(&submit->read_gem_idr);
	idr_init(&submit->write_gem_idr);

	req = submit->words;
	end = submit->words + submit->num_words;

	while (req < end) {
		if (unlikely((req_data(req) + req_length(req)) > end)) {
			dev_err(g3d->dev, "submit data truncated at 0x%x\n",
				req - submit->words);
			ret = -EINVAL;
			goto err_free;
		}

		ret = g3d_validate_request(g3d, submit, req);
		if (ret < 0)
			goto err_free;

		req = req_data_wr(req) + req_length(req);
	}

	kref_get(&ctx->ref);
	drm_submit->fence = g3d_post_requests(g3d, submit);
	drm_gem_object_unreference_unlocked(&gem->base);

	mutex_unlock(&ctx->submit_mutex);
	up_read(&priv->pipes_idr_sem);

	return 0;

err_free:
	g3d_release_submit(submit);
err_gem_unref:
	drm_gem_object_unreference_unlocked(&gem->base);
err_unlock_ctx:
	mutex_unlock(&ctx->submit_mutex);
err_unlock_idr:
	up_read(&priv->pipes_idr_sem);

	return ret;
}

int s3c6410_g3d_wait(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_priv *priv = file_priv->g3d_priv;
	struct g3d_drvdata *g3d = priv->g3d;
	struct drm_exynos_g3d_wait *wait = data;

	if (wait->timeout.tv_sec >= 0) {
		struct timespec ts;

		ts.tv_sec = wait->timeout.tv_sec;
		ts.tv_nsec = wait->timeout.tv_nsec;

		return g3d_wait_interruptible_timeout(g3d, wait->fence, &ts);
	}

	return g3d_wait_interruptible_timeout(g3d, wait->fence, NULL);
}

int s3c6410_g3d_cpu_prep(struct drm_device *dev, void *data,
			 struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct drm_exynos_g3d_cpu_prep *prep = data;
	struct g3d_priv *priv = file_priv->g3d_priv;
	struct g3d_drvdata *g3d = priv->g3d;
	struct exynos_drm_gem_obj *obj;
	uint32_t fence = 0;
	int ret;

	obj = exynos_drm_gem_lookup(g3d->subdrv.drm_dev, file, prep->handle);
	if (!obj)
		return -ENOENT;

	if (!obj->g3d_priv) {
		ret = 0;
		goto done;
	}

	if (prep->op & DRM_EXYNOS_G3D_PREP_READ)
		fence = obj->g3d_priv->write_fence;
	if (prep->op & DRM_EXYNOS_G3D_PREP_WRITE)
		fence = max(fence, obj->g3d_priv->read_fence);

	if (!(prep->op & DRM_EXYNOS_G3D_PREP_NOSYNC)) {
		struct timespec ts;

		ts.tv_sec = prep->timeout.tv_sec;
		ts.tv_nsec = prep->timeout.tv_nsec;

		ret = g3d_wait_interruptible_timeout(g3d, fence, &ts);
	} else {
		ret = g3d_wait_interruptible_timeout(g3d, fence, NULL);
	}

	if (!ret && (prep->op & DRM_EXYNOS_G3D_PREP_WRITE))
		obj->g3d_priv->dirty = true;

done:
	drm_gem_object_unreference_unlocked(&obj->base);

	return ret;
}

int s3c6410_g3d_cpu_fini(struct drm_device *dev, void *data,
			 struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct drm_exynos_g3d_cpu_prep *prep = data;
	struct g3d_priv *priv = file_priv->g3d_priv;
	struct g3d_drvdata *g3d = priv->g3d;
	struct exynos_drm_gem_obj *obj;

	obj = exynos_drm_gem_lookup(g3d->subdrv.drm_dev, file, prep->handle);
	if (!obj)
		return -ENOENT;

	if (!obj->g3d_priv)
		goto done;

	if (obj->g3d_priv->dirty) {
		obj->g3d_priv->tex_timestamp = g3d->tex_timestamp;
		obj->g3d_priv->fb_timestamp = g3d->fb_timestamp;
		obj->g3d_priv->dirty = false;
	}

done:
	drm_gem_object_unreference_unlocked(&obj->base);

	return 0;
}

int s3c6410_g3d_create_pipe(struct drm_device *drm_dev, void *data,
			    struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_priv *priv = file_priv->g3d_priv;
	struct drm_exynos_g3d_pipe *pipe = data;
	struct g3d_drvdata *g3d = priv->g3d;
	struct g3d_context *ctx;
	int ret;

	down_write(&priv->pipes_idr_sem);

	ctx = kzalloc(sizeof(struct g3d_context), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto err_unlock;
	}

	dev_dbg_ctx(g3d->dev, "%s(ctx = %p)\n", __func__, ctx);

	ctx->drm_dev = drm_dev;
	ctx->g3d = g3d;
	ctx->file = file;

	kref_init(&ctx->ref);
	mutex_init(&ctx->submit_mutex);

	memcpy(ctx->register_masks, g3d_register_masks_def,
						sizeof(ctx->register_masks));

	ret = idr_alloc(&priv->pipes_idr, ctx, 1, 0, GFP_KERNEL);
	if (ret < 0) {
		dev_err(g3d->dev, "failed to allocate pipe ID (%d)\n", ret);
		goto err_free;
	}

	pipe->pipe = ret;

	up_write(&priv->pipes_idr_sem);

	return 0;

err_free:
	kfree(ctx);
err_unlock:
	up_write(&priv->pipes_idr_sem);

	return ret;
}

static void g3d_release_context(struct kref *kref)
{
	struct g3d_context *ctx = container_of(kref, struct g3d_context, ref);
	int i;

	dev_dbg_ctx(ctx->g3d->dev, "%s(ctx = %p)\n", __func__, ctx);

	for (i = 0; i < G3D_NUM_SHADERS; ++i)
		g3d_free_shader_program(&ctx->shader_program[i]);

	for (i = 0; i < G3D_NUM_TEXTURES; ++i)
		g3d_free_texture(&ctx->texture[i]);

	for (i = 0; i < G3D_NUM_VTX_TEXTURES; ++i)
		g3d_free_vtx_texture(&ctx->vtx_texture[i]);

	g3d_free_colorbuffer(&ctx->colorbuffer);
	g3d_free_depthbuffer(&ctx->depthbuffer);

	kfree(ctx);
}

static void g3d_destroy_context(struct g3d_context *ctx)
{
	dev_dbg_ctx(ctx->g3d->dev, "%s(ctx = %p)\n", __func__, ctx);

	set_bit(G3D_CONTEXT_ABORTED, &ctx->state);
	kref_put(&ctx->ref, g3d_release_context);
}

int s3c6410_g3d_destroy_pipe(struct drm_device *drm_dev, void *data,
			     struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_priv *priv = file_priv->g3d_priv;
	struct drm_exynos_g3d_pipe *pipe = data;
	struct g3d_context *ctx;

	down_write(&priv->pipes_idr_sem);

	ctx = g3d_get_context(priv, pipe->pipe);
	if (!IS_ERR(ctx)) {
		idr_remove(&priv->pipes_idr, pipe->pipe);
		g3d_destroy_context(ctx);
	}

	up_write(&priv->pipes_idr_sem);

	return 0;
}

/*
 * DRM subdriver
 */
static int g3d_open(struct drm_device *drm_dev, struct device *dev,
		    struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);
	struct g3d_priv *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	idr_init(&priv->pipes_idr);
	priv->g3d = g3d;
	init_rwsem(&priv->pipes_idr_sem);

	/* Set private data */
	file_priv->g3d_priv = priv;

	dev_dbg_fops(dev, "device opened\n");

	return 0;
}

static int g3d_pipes_idr_cleanup(int id, void *p, void *data)
{
	struct g3d_context *ctx = p;

	g3d_destroy_context(ctx);

	return 0;
}

static void g3d_close(struct drm_device *drm_dev, struct device *dev,
		      struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_priv *priv = file_priv->g3d_priv;
	struct g3d_drvdata *g3d = priv->g3d;

	idr_for_each(&priv->pipes_idr, g3d_pipes_idr_cleanup, NULL);
	idr_destroy(&priv->pipes_idr);
	kfree(priv);

	dev_dbg_fops(g3d->dev, "device released\n");
}

/*
 * Platform driver
 */
static int g3d_probe(struct platform_device *pdev)
{
	struct exynos_drm_subdrv *subdrv;
	struct g3d_drvdata *g3d;
	struct resource *res;
	int ret;

	g3d = devm_kzalloc(&pdev->dev, sizeof(*g3d), GFP_KERNEL);
	if (!g3d)
		return -ENOMEM;

	/* get device clock */
	g3d->clock = devm_clk_get(&pdev->dev, "bus-clk");
	if (IS_ERR(g3d->clock)) {
		dev_err(&pdev->dev, "failed to find g3d clock source\n");
		return PTR_ERR(g3d->clock);
	}

	/* map mem resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	g3d->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(g3d->base))
		return PTR_ERR(g3d->base);
	g3d->base_phys = res->start;

	/* get the IRQ */
	g3d->irq = platform_get_irq(pdev, 0);
	if (g3d->irq < 0) {
		dev_err(&pdev->dev,
			"failed to get irq resource (%d).\n", g3d->irq);
		return g3d->irq;
	}

	/* request the IRQ */
	ret = devm_request_irq(&pdev->dev, g3d->irq,
					g3d_handle_irq, 0, pdev->name, g3d);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed (%d).\n", ret);
		return ret;
	}

	/* try to map DMA registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	g3d->dma_base = devm_ioremap_resource(&pdev->dev, res);

	/* try to get DMA clock */
	g3d->dma_clock = devm_clk_get(&pdev->dev, "dma-clk");

	if (!IS_ERR(g3d->dma_base) && !IS_ERR(g3d->dma_clock)) {
		clk_prepare_enable(g3d->dma_clock);

		dev_info(&pdev->dev,
				"found DMA resources, using external DMA\n");
		g3d->dma_enabled = true;
	}

	if (g3d->dma_enabled) {
		g3d->dma_lli = dma_alloc_coherent(&pdev->dev,
				G3D_NUM_DMA_LLI_ENTRIES * G3D_DMA_LLI_SIZE,
				&g3d->dma_lli_phys, GFP_KERNEL);
		if (!g3d->dma_lli) {
			dev_err(&pdev->dev,
				"failed to allocate DMA LLI buffer, disabling DMA\n");
			g3d->dma_enabled = false;
		}
	}

	g3d->dev = &pdev->dev;
	spin_lock_init(&g3d->ready_lock);
	INIT_LIST_HEAD(&g3d->submit_list);
	init_waitqueue_head(&g3d->ready_wq);
	init_waitqueue_head(&g3d->flush_wq);
	init_waitqueue_head(&g3d->idle_wq);
	init_waitqueue_head(&g3d->fence_wq);
	atomic_set(&g3d->next_fence, 0);
	setup_timer(&g3d->watchdog_timer, g3d_watchdog_timer,
			(unsigned long)g3d);

	platform_set_drvdata(pdev, g3d);

	pm_runtime_set_autosuspend_delay(&pdev->dev, G3D_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

#ifdef CONFIG_DRM_EXYNOS_S3C6410_G3D_ALWAYS_ON
	pm_runtime_get_sync(&pdev->dev);
	clk_prepare_enable(g3d->clock);
#endif

	pm_runtime_get_sync(&pdev->dev);

	clk_prepare_enable(g3d->clock);
	g3d->version = g3d_read(g3d, G3D_FGGB_VERSION);
	clk_disable_unprepare(g3d->clock);

	dev_info(&pdev->dev, "detected FIMG-3DSE version %d.%d.%d\n",
			g3d->version >> 24, (g3d->version >> 16) & 0xff,
			(g3d->version >> 8) & 0xff);

	pm_runtime_put_sync(&pdev->dev);
	set_bit(G3D_STATE_IDLE, &g3d->state);

	g3d->thread = kthread_run(g3d_command_thread, g3d, "kfimgd");
	if (IS_ERR(g3d->thread)) {
		ret = PTR_ERR(g3d->thread);
		dev_err(&pdev->dev,
				"failed to create command thread (%d)\n", ret);
		goto err_thread;
	}

	subdrv = &g3d->subdrv;
	subdrv->dev = &pdev->dev;
	subdrv->open = g3d_open;
	subdrv->close = g3d_close;

	ret = exynos_drm_subdrv_register(subdrv);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register drm g3d device\n");
		goto err_subdrv_register;
	}

	return 0;

err_subdrv_register:
	kthread_stop(g3d->thread);
err_thread:
	pm_runtime_disable(&pdev->dev);
	if (g3d->dma_enabled)
		dma_free_coherent(&pdev->dev,
				G3D_NUM_DMA_LLI_ENTRIES * G3D_DMA_LLI_SIZE,
				g3d->dma_lli, g3d->dma_lli_phys);

	return ret;
}

static int g3d_remove(struct platform_device *pdev)
{
	struct g3d_drvdata *g3d = platform_get_drvdata(pdev);

	exynos_drm_subdrv_unregister(&g3d->subdrv);

	kthread_stop(g3d->thread);

#ifdef CONFIG_DRM_EXYNOS_S3C6410_G3D_ALWAYS_ON
	clk_disable_unprepare(g3d->clock);
	pm_runtime_put_sync(&pdev->dev);
#endif

	pm_runtime_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	if (g3d->dma_enabled) {
		dma_free_coherent(&pdev->dev,
				G3D_NUM_DMA_LLI_ENTRIES * G3D_DMA_LLI_SIZE,
				g3d->dma_lli, g3d->dma_lli_phys);

		clk_disable_unprepare(g3d->dma_clock);
	}

	return 0;
}

#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM_RUNTIME)
static int g3d_runtime_suspend(struct device *dev)
{
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);
	bool idle;
	int ret;

	clk_prepare_enable(g3d->clock);
	idle = test_and_clear_bit(G3D_STATE_IDLE, &g3d->state);

	ret = g3d_flush_pipeline(g3d, G3D_FLUSH_COLOR_CACHE, false);
	if (ret)
		return ret;

	ret = g3d_flush_caches(g3d);
	if (ret)
		return ret;

	if (idle)
		set_bit(G3D_STATE_IDLE, &g3d->state);
	clk_disable_unprepare(g3d->clock);

	return 0;
}

static int g3d_runtime_resume(struct device *dev)
{
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);
	bool idle;

	clk_prepare_enable(g3d->clock);
	idle = test_and_clear_bit(G3D_STATE_IDLE, &g3d->state);

	g3d_initialize(g3d);

	if (idle)
		set_bit(G3D_STATE_IDLE, &g3d->state);
	clk_disable_unprepare(g3d->clock);

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int g3d_suspend(struct device *dev)
{
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);
	int ret;

	if (!pm_runtime_suspended(dev)) {
		ret = g3d_runtime_suspend(dev);
		if (ret)
			return ret;
	}

	if (!test_bit(G3D_STATE_IDLE, &g3d->state))
		clk_disable_unprepare(g3d->clock);

	return 0;
}

static int g3d_resume(struct device *dev)
{
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);

	if (!test_bit(G3D_STATE_IDLE, &g3d->state))
		clk_prepare_enable(g3d->clock);

	if (!pm_runtime_suspended(dev))
		return g3d_runtime_resume(dev);

	return 0;
}
#endif

static const struct dev_pm_ops g3d_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(g3d_suspend, g3d_resume)
	SET_RUNTIME_PM_OPS(g3d_runtime_suspend, g3d_runtime_resume, NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id g3d_of_matches[] = {
	{ .compatible = "samsung,s3c6410-g3d", },
	{ /* Sentinel */ }
};
#endif

struct platform_driver s3c6410_g3d_driver = {
	.probe	= g3d_probe,
	.remove	= g3d_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "s3c6410-g3d",
		.of_match_table = of_match_ptr(g3d_of_matches),
		.pm	= &g3d_pm_ops,
	},
};
