/*
 * DRM driver for Samsung FIMG-3DSE GPU
 *
 * Copyright 2013 Tomasz Figa <tomasz.figa at gmail.com>
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
 * - advanced context scheduling
 */

#ifdef CONFIG_DRM_EXYNOS_S3C6410_G3D_DEBUG
#define DEBUG
#endif

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
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
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <drm/drmP.h>
#include <drm/exynos_drm.h>
#include "exynos_drm_drv.h"
#include "exynos_drm_gem.h"

/*
 * Various definitions
 */
#define G3D_AUTOSUSPEND_DELAY		100
#define G3D_FLUSH_TIMEOUT		1000
#define G3D_MAX_REQUESTS		128
#define G3D_NUM_SHADER_INSTR		512
#define G3D_NUM_CONST_FLOAT		1024
#define G3D_NUM_CONST_INT		16
#define G3D_NUM_CONST_BOOL		1
#define G3D_NUM_TEXTURES		8

#define G3D_STATE_BUFFER_MAX_SIZE	\
	(G3D_NUM_REGISTERS * sizeof(struct g3d_state_entry))
#define G3D_DRAW_BUFFER_MAX_SIZE	4096
#define G3D_SHADER_PROGRAM_MAX_SIZE	\
	(G3D_NUM_SHADER_INSTR * sizeof(uint32_t))
#define G3D_SHADER_DATA_MAX_SIZE	\
	((G3D_NUM_CONST_FLOAT + G3D_NUM_CONST_INT + G3D_NUM_CONST_BOOL) \
	* sizeof(uint32_t))

/*
 * Registers
 */
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

/*
 * Request flags
 */
#define G3D_REQUEST_MARK_PREEMPTION	(1 << 0)
#define G3D_REQUEST_PREEMPTION_POINT	(1 << 1)
#define G3D_REQUEST_UPDATE_STATE	(1 << 2)
#define G3D_REQUEST_STRICT_SIZE_CHECK	(1 << 3)

#define REQ_DATA(req)			((req) + 1)
#define REQ_LENGTH(req)			(*(req) & 0xffffff)
#define REQ_TYPE(req)			(*(req) >> 24)
#define REQ_HDR(type, len)		(((type) << 24) | ((len) & 0xffffff))

#define TEX_UNIT(flags)			((flags) >> 24)

/*
 * Private data types
 */
struct g3d_context;

enum g3d_state_bits {
	G3D_STATE_IDLE,
	G3D_STATE_PREEMPTED,
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

struct g3d_drvdata {
	struct exynos_drm_subdrv subdrv;
	void __iomem *base;
	int irq;
	struct resource *mem;
	struct clk *clock;
	struct device *dev;
	struct task_struct *thread;
	struct mutex stall_mutex;

	spinlock_t ready_lock;
	struct list_head ready_list;
	wait_queue_head_t ready_wq;

	struct g3d_context *last_ctx;

	struct semaphore request_sem;

	unsigned long state;
	unsigned int tex_timestamp;
	unsigned int fb_timestamp;
};

enum g3d_context_state_bits {
	G3D_CONTEXT_FULL_RESTORE,
	G3D_CONTEXT_FLUSHED,
	G3D_CONTEXT_STOP_PIXEL_SHADER,
};

struct g3d_validation_data {
	struct g3d_request *shader_program[G3D_NUM_SHADERS];
};

struct g3d_context {
	struct g3d_drvdata *g3d;
	struct drm_device *drm_dev;
	struct drm_file *file;
	struct list_head list;

	uint32_t submitted_fence;
	uint32_t completed_fence;
	wait_queue_head_t fence_wq;

	struct list_head state_update_list;
	struct list_head state_restore_list;

	struct list_head request_list;

	uint32_t registers[G3D_NUM_REGISTERS];
	uint32_t register_masks[G3D_NUM_PROT_REGISTERS];

	struct g3d_request *shader_program[G3D_NUM_SHADERS];
	struct g3d_request *texture[G3D_NUM_TEXTURES];
	struct g3d_request *colorbuffer;
	struct g3d_request *depthbuffer;

	struct g3d_validation_data validation;

	unsigned long state;
};

struct g3d_request {
	struct g3d_context *ctx;
	struct list_head list;
	struct kref kref;
	uint32_t header;
	uint8_t data[];
};

struct exynos_drm_gem_g3d_priv {
	unsigned int tex_timestamp;
	unsigned int fb_timestamp;
};

struct g3d_request_info {
	int (*validate)(struct g3d_drvdata *g3d, struct g3d_request *);
	void (*free)(struct g3d_drvdata *g3d, struct g3d_request *);
	int (*handle)(struct g3d_drvdata *g3d, struct g3d_request *);
	int (*update_state)(struct g3d_drvdata *g3d, struct g3d_request *);
	int (*apply)(struct g3d_drvdata *g3d, struct g3d_request *);
	size_t length;
	unsigned int flags;
	size_t priv_length;
};

struct g3d_colorbuffer {
	struct exynos_drm_gem_obj *obj;
	struct g3d_req_colorbuffer_setup req[];
};

struct g3d_depthbuffer {
	struct exynos_drm_gem_obj *obj;
	struct g3d_req_depthbuffer_setup req[];
};

struct g3d_texture {
	struct exynos_drm_gem_obj *obj;
	struct g3d_req_texture_setup req[];
};

struct g3d_draw_buffer {
	struct exynos_drm_gem_obj *obj;
	struct g3d_req_draw_buffer req[];
};

struct g3d_shader_program {
	struct exynos_drm_gem_obj *obj;
	uint32_t *data;
	uint16_t type_offset[G3D_NUM_SHADER_DATA_TYPES];
	struct g3d_req_shader_program req[];
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
 * Register accessors
 */
#ifdef DEBUG

#define DEBUG_TRACE		(1 << 0)
#define DEBUG_EVENTS		(1 << 1)
#define DEBUG_PIPE		(1 << 2)
#define DEBUG_FOPS		(1 << 3)
#define DEBUG_REQS		(1 << 4)
#define DEBUG_REQ_DATA		(1 << 28)
#define DEBUG_DRAW_DATA		(1 << 29)
#define DEBUG_IO		(1 << 30)
#define DEBUG_HW_WATCHPOINTS	(1 << 31)

static unsigned int s3c6410_g3d_debug_mask;
module_param(s3c6410_g3d_debug_mask, uint, 0644);

#define dev_dbg_mask(mask, ...)				\
	do {						\
		if (s3c6410_g3d_debug_mask & (mask))	\
			dev_dbg(__VA_ARGS__);		\
	} while (0)

#define dev_dbg_trace(...)	dev_dbg_mask(DEBUG_TRACE, __VA_ARGS__)
#define dev_dbg_events(...)	dev_dbg_mask(DEBUG_EVENTS, __VA_ARGS__)
#define dev_dbg_pipe(...)	dev_dbg_mask(DEBUG_PIPE, __VA_ARGS__)
#define dev_dbg_fops(...)	dev_dbg_mask(DEBUG_FOPS, __VA_ARGS__)
#define dev_dbg_reqs(...)	dev_dbg_mask(DEBUG_REQS, __VA_ARGS__)
#define dev_dbg_io(...)		dev_dbg_mask(DEBUG_IO, __VA_ARGS__)

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
				32, 4, buf, len, false);
}
#else /* !DEBUG */

#define dev_dbg_trace(...)
#define dev_dbg_events(...)
#define dev_dbg_pipe(...)
#define dev_dbg_fops(...)
#define dev_dbg_reqs(...)
#define dev_dbg_io(...)

static inline void g3d_assert_enabled(struct g3d_drvdata *g3d) {}
static inline void g3d_dump_draw_buffer(const void *buf, size_t len) {}
static inline void g3d_dump_req_data(const void *buf, size_t len) {}

#endif /* DEBUG */

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

static inline bool g3d_pipeline_idle(struct g3d_drvdata *g3d, uint32_t mask)
{
	uint32_t stat = g3d_read(g3d, G3D_FGGB_PIPESTAT_REG);

	dev_dbg_pipe(g3d->dev, "%s: stat = %08x\n", __func__, stat);
	return !(stat & mask);
}

static int g3d_wait_for_flush(struct g3d_drvdata *g3d, uint32_t mask, bool idle)
{
	int ret;

	dev_dbg_trace(g3d->dev, "%s\n", __func__);

	g3d_idle_irq_enable(g3d, mask);

	ret = wait_event_interruptible_timeout(g3d->ready_wq,
				g3d_pipeline_idle(g3d, mask)
				|| (idle && !list_empty(&g3d->ready_list)),
				msecs_to_jiffies(G3D_FLUSH_TIMEOUT));

	g3d_idle_irq_ack_and_disable(g3d);

	if (ret < 0)
		return ret;
	if (!ret) {
		dev_err(g3d->dev, "pipeline flush timed out (stat = %08x)\n",
					g3d_read(g3d, G3D_FGGB_PIPESTAT_REG));
		g3d_initialize(g3d);
		return ret;
	}

	dev_dbg_events(g3d->dev, "%s:%d ready_wq wake-up\n",
			__func__, __LINE__);

	return 0;
}

static int g3d_flush_pipeline(struct g3d_drvdata *g3d,
			      enum g3d_flush_level level, bool idle)
{
	uint32_t mask;

	mask = ((1 << (level + 1)) - 1) & G3D_FGGB_PIPESTAT_MSK;

	dev_dbg_trace(g3d->dev, "%s (mask = %08x)\n", __func__, mask);

	if (g3d_pipeline_idle(g3d, mask))
		return 0;

	return g3d_wait_for_flush(g3d, mask, idle);
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

static void exynos_drm_gem_sync_for_device(struct drm_device *drm_dev,
					   struct exynos_drm_gem_obj *obj,
					   enum dma_data_direction dir)
{
	dma_sync_single_for_device(drm_dev->dev,
					obj->buffer->dma_addr, obj->size, dir);
}

static void exynos_drm_gem_sync_for_cpu(struct drm_device *drm_dev,
					struct exynos_drm_gem_obj *obj,
					enum dma_data_direction dir)
{
	dma_sync_single_for_cpu(drm_dev->dev,
					obj->buffer->dma_addr, obj->size, dir);
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

static inline void *req_data(struct g3d_request *req)
{
	return (void *)req->data;
}

static inline uint32_t req_length(struct g3d_request *req)
{
	return REQ_LENGTH(&req->header);
}

/* Request callback wrappers */
static inline int g3d_request_handle(struct g3d_drvdata *g3d,
						struct g3d_request *req)
{
	dev_dbg_reqs(g3d->dev, "%s: %d\n", __func__, REQ_TYPE(&req->header));
	return g3d_requests[REQ_TYPE(&req->header)].handle(g3d, req);
}

static inline void g3d_request_update_state(struct g3d_drvdata *g3d,
						struct g3d_request *req)
{
	dev_dbg_reqs(g3d->dev, "%s: %d\n", __func__, REQ_TYPE(&req->header));
	g3d_requests[REQ_TYPE(&req->header)].update_state(g3d, req);
}

static inline void g3d_request_apply(struct g3d_drvdata *g3d,
						struct g3d_request *req)
{
	dev_dbg_reqs(g3d->dev, "%s: %d\n", __func__, REQ_TYPE(&req->header));
	g3d_requests[REQ_TYPE(&req->header)].apply(g3d, req);
}

static void g3d_restore_context(struct g3d_drvdata *g3d,
				struct g3d_context *ctx)
{
	struct g3d_request *req;

	g3d_reg_cache_sync_all(g3d, ctx);

	list_for_each_entry(req, &ctx->state_restore_list, list)
		g3d_request_apply(g3d, req);
}

/* Helper for state update requests */
static int g3d_handle_state_update(struct g3d_drvdata *g3d,
				   struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;

	list_add_tail(&req->list, &ctx->state_update_list);
	return 0;
}

/* Request management helpers */
static void g3d_request_release(struct kref *kref)
{
	struct g3d_request *req = container_of(kref, struct g3d_request, kref);
	struct g3d_drvdata *g3d = req->ctx->g3d;

	if (g3d_requests[REQ_TYPE(&req->header)].free)
		g3d_requests[REQ_TYPE(&req->header)].free(g3d, req);

	kfree(req);
	up(&g3d->request_sem);
}

static inline void g3d_request_put(struct g3d_request *req)
{
	kref_put(&req->kref, g3d_request_release);
}

static inline void g3d_request_get(struct g3d_request *req)
{
	kref_get(&req->kref);
}

/* State update */
static void g3d_state_update_and_restore(struct g3d_drvdata *g3d,
					 struct g3d_context *ctx)
{
	struct g3d_request *req, *p;

	list_for_each_entry_safe(req, p, &ctx->state_update_list, list) {
		g3d_request_update_state(g3d, req);
		g3d_request_put(req);
	}

	g3d_restore_context(g3d, ctx);
}

static void g3d_state_update_and_apply(struct g3d_drvdata *g3d,
				       struct g3d_context *ctx)
{
	struct g3d_request *req, *p;

	list_for_each_entry_safe(req, p, &ctx->state_update_list, list) {
		g3d_request_update_state(g3d, req);
		g3d_request_apply(g3d, req);
		g3d_request_put(req);
	}
}

static void g3d_update_state(struct g3d_drvdata *g3d, struct g3d_context *ctx)
{
	int ret;
	bool was_powered_down = false;

	if (test_and_clear_bit(G3D_STATE_IDLE, &g3d->state)) {
		ret = pm_runtime_get_sync(g3d->dev);
		if (!ret)
			was_powered_down = true;
		clk_prepare_enable(g3d->clock);
	}

	if (was_powered_down) {
		set_bit(G3D_CONTEXT_FULL_RESTORE, &ctx->state);
		set_bit(G3D_CONTEXT_STOP_PIXEL_SHADER, &ctx->state);
	} else if (g3d->last_ctx != ctx) {
		g3d_flush_pipeline(g3d, G3D_FLUSH_PER_FRAGMENT, false);
		if (g3d->last_ctx)
			set_bit(G3D_CONTEXT_FLUSHED, &g3d->last_ctx->state);
		set_bit(G3D_CONTEXT_FULL_RESTORE, &ctx->state);
		set_bit(G3D_CONTEXT_STOP_PIXEL_SHADER, &ctx->state);
	} else if (!list_empty(&ctx->state_update_list)) {
		g3d_flush_pipeline(g3d, G3D_FLUSH_PER_FRAGMENT, false);
	} else {
		g3d_flush_pipeline(g3d, G3D_FLUSH_VERTEX_FIFO, false);
	}

	if (test_bit(G3D_CONTEXT_STOP_PIXEL_SHADER, &ctx->state))
		g3d_stop_pixel_shader(g3d);

	if (test_and_clear_bit(G3D_CONTEXT_FULL_RESTORE, &ctx->state))
		g3d_state_update_and_restore(g3d, ctx);
	else
		g3d_state_update_and_apply(g3d, ctx);

	if (test_and_clear_bit(G3D_CONTEXT_STOP_PIXEL_SHADER, &ctx->state))
		g3d_start_pixel_shader(g3d);

	INIT_LIST_HEAD(&ctx->state_update_list);
	g3d->last_ctx = ctx;
}

/* Register state update request */
static int g3d_validate_state_buffer(struct g3d_drvdata *g3d,
				     struct g3d_request *req)
{
	struct g3d_state_entry *reg = req_data(req);
	int count = req_length(req) / sizeof(*reg);

	while (count--) {
		if (unlikely(reg->reg >= G3D_NUM_REGISTERS)) {
			dev_err(g3d->dev, "register index out of range\n");
			return -EINVAL;
		}
		++reg;
	}

	return 0;
}

static int g3d_process_state_buffer(struct g3d_drvdata *g3d,
				    struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	const struct g3d_state_entry *reg = req_data(req);
	unsigned int count = req_length(req) / sizeof(*reg);

	while (count--) {
		uint32_t val = reg->val;
		if (reg->reg < G3D_NUM_PROT_REGISTERS) {
			uint32_t mask = ctx->register_masks[reg->reg];
			val &= mask;
			val |= ctx->registers[reg->reg] & ~mask;
		}
		ctx->registers[reg->reg] = val;
		++reg;
	}

	return 0;
}

static int g3d_apply_state_buffer(struct g3d_drvdata *g3d,
				  struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	const struct g3d_state_entry *reg = req_data(req);
	unsigned int count = req_length(req) / sizeof(*reg);

	while (count--) {
		g3d_write_relaxed(g3d, ctx->registers[reg->reg],
						g3d_registers[reg->reg]);
		++reg;
	}

	/* Make sure register writes are not reordered across this point. */
	wmb();

	return 0;
}

/* Draw request */
static int g3d_validate_draw_buffer(struct g3d_drvdata *g3d,
				    struct g3d_request *req)
{
	struct g3d_draw_buffer *db = req_data(req);
	struct g3d_req_draw_buffer *rdb = db->req;
	int ret;

	db->obj = exynos_drm_gem_lookup(g3d->subdrv.drm_dev,
						req->ctx->file, rdb->handle);
	if (!db->obj) {
		dev_err(g3d->dev, "failed to lookup draw BO\n");
		return -EINVAL;
	}

	if (unlikely(!db->obj->buffer->kvaddr)) {
		dev_err(g3d->dev, "draw BO must have kernel mapping\n");
		ret = -EINVAL;
		goto err_obj;
	}

	if (unlikely(rdb->offset + rdb->length > db->obj->size)) {
		dev_err(g3d->dev, "draw data bigger than BO\n");
		ret = -EINVAL;
		goto err_obj;
	}

	if (unlikely(rdb->length > G3D_DRAW_BUFFER_MAX_SIZE)) {
		dev_err(g3d->dev, "draw data size above maximum\n");
		ret = -EINVAL;
		goto err_obj;
	}

	return 0;

err_obj:
	drm_gem_object_unreference_unlocked(&db->obj->base);

	return ret;
}

static void g3d_free_draw_buffer(struct g3d_drvdata *g3d,
				 struct g3d_request *req)
{
	struct g3d_draw_buffer *db = req_data(req);

	drm_gem_object_unreference_unlocked(&db->obj->base);
}

static int g3d_handle_draw_buffer(struct g3d_drvdata *g3d,
				  struct g3d_request *req)
{
	struct g3d_draw_buffer *db = req_data(req);
	struct g3d_req_draw_buffer *rdb = db->req;
	size_t vertex_len = rdb->length & ~31;
	const void *vertex_buf;

	vertex_buf = db->obj->buffer->kvaddr + rdb->offset;

	dev_dbg_reqs(g3d->dev, "%s (cnt = %u, len = %u)\n",
					__func__, rdb->vertices, vertex_len);
	g3d_dump_draw_buffer(vertex_buf, vertex_len);

	g3d_write(g3d, 0, G3D_FGHI_VBADDR_REG);
	g3d_write_burst(g3d, G3D_FGHI_VB_ENTRY_REG, vertex_buf, vertex_len);

	g3d_write(g3d, rdb->vertices, G3D_FGHI_FIFO_ENTRY_REG);
	g3d_write(g3d, 0, G3D_FGHI_FIFO_ENTRY_REG);

	clear_bit(G3D_CONTEXT_FLUSHED, &req->ctx->state);

	g3d_request_put(req);

	return 0;
}

/* Fence request */
static int g3d_handle_fence(struct g3d_drvdata *g3d, struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;

	++ctx->completed_fence;
	wake_up_interruptible_all(&ctx->fence_wq);

	g3d_request_put(req);

	return 0;
}

/* Shader program update request */
static inline uint8_t RSP_UNIT(const struct g3d_req_shader_program *rsp)
{
	return rsp->unit_nattrib >> 8;
}

static inline uint8_t RSP_NATTRIB(const struct g3d_req_shader_program *rsp)
{
	return rsp->unit_nattrib;
}

static inline uint16_t RSP_DCOUNT(const struct g3d_req_shader_program *rsp,
				  unsigned int type)
{
	return rsp->dcount[type / 2] >> (16 * (type % 2));
}

static int g3d_validate_shader_program(struct g3d_drvdata *g3d,
				    struct g3d_request *req)
{
	struct g3d_shader_program *sp = req_data(req);
	struct g3d_req_shader_program *rsp = sp->req;
	struct g3d_context *ctx = req->ctx;
	unsigned int count;
	unsigned int type;
	int ret;

	if (unlikely(RSP_UNIT(rsp) >= G3D_NUM_SHADERS)) {
		dev_err(g3d->dev, "invalid shader index\n");
		return -EINVAL;
	}

	if (!rsp->length) {
		/* Shader detach request */
		sp->data = NULL;
		ctx->validation.shader_program[RSP_UNIT(rsp)] = NULL;
		return 0;
	}

	sp->obj = exynos_drm_gem_lookup(g3d->subdrv.drm_dev,
						req->ctx->file, rsp->handle);
	if (!sp->obj) {
		dev_err(g3d->dev, "failed to lookup shader program BO\n");
		return -EINVAL;
	}

	if (unlikely(!sp->obj->buffer->kvaddr)) {
		dev_err(g3d->dev,
			"shader program BO must have kernel mapping\n");
		ret = -EINVAL;
		goto err_obj;
	}

	if (unlikely(rsp->offset + rsp->length > sp->obj->size)) {
		dev_err(g3d->dev, "shader program bigger than BO\n");
		ret = -EINVAL;
		goto err_obj;
	}

	if (unlikely(rsp->length > G3D_SHADER_PROGRAM_MAX_SIZE)) {
		dev_err(g3d->dev, "shader program size above maximum\n");
		ret = -EINVAL;
		goto err_obj;
	}

	count = 0;
	for (type = 0; type < G3D_NUM_SHADER_DATA_TYPES; ++type) {
		if (unlikely(RSP_DCOUNT(rsp, type)
		    > g3d_shader_data_count[type])) {
			dev_err(g3d->dev, "requested shader data too big\n");
			ret = -EINVAL;
			goto err_obj;
		}

		sp->type_offset[type] = count;
		count += RSP_DCOUNT(rsp, type);
	}

	sp->data = kcalloc(count, sizeof(uint32_t), GFP_KERNEL);
	if (!sp->data) {
		dev_err(g3d->dev,
			"failed to allocate memory for shader data\n");
		ret = -ENOMEM;
		goto err_obj;
	}

	ctx->validation.shader_program[RSP_UNIT(rsp)] = req;

	return 0;

err_obj:
	drm_gem_object_unreference_unlocked(&sp->obj->base);

	return ret;
}

static void g3d_free_shader_program(struct g3d_drvdata *g3d,
				 struct g3d_request *req)
{
	struct g3d_shader_program *sp = req_data(req);

	kfree(sp->data);
	drm_gem_object_unreference_unlocked(&sp->obj->base);
}

static int g3d_handle_shader_program(struct g3d_drvdata *g3d,
				     struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	struct g3d_shader_program *sp = req_data(req);
	struct g3d_req_shader_program *rsp = sp->req;

	if (RSP_UNIT(rsp) == G3D_SHADER_PIXEL)
		set_bit(G3D_CONTEXT_STOP_PIXEL_SHADER, &ctx->state);

	return g3d_handle_state_update(g3d, req);
}

static int g3d_process_shader_program(struct g3d_drvdata *g3d,
				      struct g3d_request *req)
{
	struct g3d_shader_program *sp = req_data(req);
	struct g3d_req_shader_program *rsp = sp->req;
	struct g3d_context *ctx = req->ctx;
	struct g3d_request *prev_req;

	prev_req = ctx->shader_program[RSP_UNIT(rsp)];
	if (prev_req) {
		list_del(&prev_req->list);
		g3d_request_put(prev_req);
	}

	if (!sp->obj) {
		/* Shader detach request */
		ctx->shader_program[RSP_UNIT(rsp)] = NULL;
		return 0;
	}

	g3d_request_get(req);
	ctx->shader_program[RSP_UNIT(rsp)] = req;
	list_add_tail(&req->list, &ctx->state_restore_list);

	return 0;
}

static int g3d_apply_shader_program(struct g3d_drvdata *g3d,
				    struct g3d_request *req)
{
	struct g3d_shader_program *sp = req_data(req);
	struct g3d_req_shader_program *rsp = sp->req;
	const uint32_t *buf;
	uint32_t offset;
	uint32_t len;

	if (!sp->obj)
		return 0;

	buf = sp->obj->buffer->kvaddr + rsp->offset;
	len = rsp->length / 4;
	offset = g3d_shader_base[RSP_UNIT(rsp)];

	while (len--) {
		g3d_write_relaxed(g3d, *(buf++), offset);
		offset += 4;
	}

	switch (RSP_UNIT(rsp)) {
	case G3D_SHADER_VERTEX:
		g3d_write(g3d, ((rsp->length / 16) - 1) << 16,
						G3D_FGVS_PC_RANGE_REG);
		g3d_write(g3d, 3, G3D_FGVS_CONFIG_REG);
		g3d_write(g3d, RSP_NATTRIB(rsp), G3D_FGVS_ATTRIBUTE_NUM_REG);
		break;
	case G3D_SHADER_PIXEL:
		g3d_write(g3d, 0, G3D_FGPS_PC_START_REG);
		g3d_write(g3d, (rsp->length / 16) - 1,
						G3D_FGPS_PC_END_REG);
		g3d_write(g3d, 1, G3D_FGPS_PC_COPY_REG);
		g3d_write(g3d, RSP_NATTRIB(rsp), G3D_FGPS_ATTRIBUTE_NUM_REG);
		break;
	default:
		dev_warn(g3d->dev, "invalid shader type\n");
	}

	return 0;
}

/* Shader data update request */
static inline uint8_t RSD_UNIT(const struct g3d_req_shader_data *rsd)
{
	return rsd->unit_type_offs >> 24;
}

static inline uint8_t RSD_TYPE(const struct g3d_req_shader_data *rsd)
{
	return rsd->unit_type_offs >> 16;
}

static inline uint16_t RSD_OFFSET(const struct g3d_req_shader_data *rsd)
{
	return rsd->unit_type_offs;
}

static int g3d_validate_shader_data(struct g3d_drvdata *g3d,
				    struct g3d_request *req)
{
	struct g3d_req_shader_data *rsd = req_data(req);
	struct g3d_context *ctx = req->ctx;
	struct g3d_shader_program *sp;
	struct g3d_req_shader_program *rsp;
	int count = (req_length(req) - sizeof(*rsd)) / 4;

	if (unlikely(RSD_UNIT(rsd) >= G3D_NUM_SHADERS)) {
		dev_err(g3d->dev, "invalid shader index\n");
		return -EINVAL;
	}

	if (unlikely(RSD_TYPE(rsd) >= G3D_NUM_SHADER_DATA_TYPES)) {
		dev_err(g3d->dev, "invalid shader data type\n");
		return -EINVAL;
	}

	if (unlikely(count <= 0)) {
		dev_err(g3d->dev, "invalid shader data access length\n");
		return -EINVAL;
	}

	if (unlikely(!ctx->validation.shader_program[RSD_UNIT(rsd)])) {
		dev_err(g3d->dev,
			"shader data access with no shader program\n");
		return -EACCES;
	}

	sp = req_data(ctx->validation.shader_program[RSD_UNIT(rsd)]);
	rsp = sp->req;

	if (unlikely(RSD_OFFSET(rsd) + count
	    > RSP_DCOUNT(rsp, RSD_TYPE(rsd)))) {
		dev_err(g3d->dev, "shader data access out of range\n");
		return -ENOSPC;
	}

	return 0;
}

static int g3d_handle_shader_data(struct g3d_drvdata *g3d,
				  struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	struct g3d_req_shader_data *rsd = req_data(req);

	if (RSD_UNIT(rsd) == G3D_SHADER_PIXEL)
		set_bit(G3D_CONTEXT_STOP_PIXEL_SHADER, &ctx->state);

	return g3d_handle_state_update(g3d, req);
}

static int g3d_process_shader_data(struct g3d_drvdata *g3d,
				   struct g3d_request *req)
{
	struct g3d_req_shader_data *rsd = req_data(req);
	struct g3d_context *ctx = req->ctx;
	struct g3d_shader_program *sp;
	uint32_t count = (req_length(req) - sizeof(*rsd)) / 4;

	sp = req_data(ctx->validation.shader_program[RSD_UNIT(rsd)]);

	memcpy(sp->data + sp->type_offset[RSD_TYPE(rsd)] + RSD_OFFSET(rsd),
		rsd->data, count * 4);

	return 0;
}

static int g3d_apply_shader_data(struct g3d_drvdata *g3d,
				 struct g3d_request *req)
{
	struct g3d_req_shader_data *rsd = req_data(req);
	const uint32_t *buf = rsd->data;
	uint32_t count = (req_length(req) - sizeof(*rsd)) / 4;
	uint32_t type_base = g3d_shader_data_reg_base[RSD_TYPE(rsd)];
	uint32_t reg;

	reg = g3d_shader_base[RSD_UNIT(rsd)] + type_base + 4 * RSD_OFFSET(rsd);

	while (count--) {
		g3d_write_relaxed(g3d, *(buf++), reg);
		reg += 4;
	}

	/* Make sure register writes are not reordered across this point. */
	wmb();

	return 0;
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

static const struct g3d_req_texture_setup g3d_texture_detached = {
	.control = 0x8000000,
};

static int g3d_validate_texture(struct g3d_drvdata *g3d,
				struct g3d_request *req)
{
	struct g3d_texture *tex = req_data(req);
	struct g3d_req_texture_setup *rts = tex->req;
	int ret;

	if (rts->flags & G3D_TEXTURE_DETACH) {
		tex->obj = NULL;
		memcpy(rts, &g3d_texture_detached, sizeof(*rts));
		return 0;
	}

	tex->obj = exynos_drm_gem_lookup(g3d->subdrv.drm_dev,
						req->ctx->file, rts->handle);
	if (!tex->obj) {
		dev_err(g3d->dev, "failed to lookup texture BO\n");
		return -EINVAL;
	}

	ret = g3d_prepare_gem_object(g3d, tex->obj);
	if (ret)
		goto err_obj;

	/* TODO: do some checks */
	rts->base_offset += tex->obj->buffer->dma_addr;

	if (rts->flags & G3D_TEXTURE_DIRTY) {
		exynos_drm_gem_sync_for_device(req->ctx->drm_dev,
						tex->obj, DMA_TO_DEVICE);

		tex->obj->g3d_priv->tex_timestamp = g3d->tex_timestamp;
	}

	return 0;

err_obj:
	drm_gem_object_unreference_unlocked(&tex->obj->base);

	return ret;
}

static void g3d_free_texture(struct g3d_drvdata *g3d,
			     struct g3d_request *req)
{
	struct g3d_texture *tex = req_data(req);

	if (tex->obj)
		drm_gem_object_unreference_unlocked(&tex->obj->base);
}

static int g3d_process_texture(struct g3d_drvdata *g3d,
			       struct g3d_request *req)
{
	struct g3d_texture *tex = req_data(req);
	struct g3d_req_texture_setup *rts = tex->req;
	struct g3d_context *ctx = req->ctx;
	struct g3d_request *prev_req;

	prev_req = ctx->texture[TEX_UNIT(rts->flags)];
	if (prev_req) {
		list_del(&prev_req->list);
		g3d_request_put(prev_req);
	}

	if (!tex->obj) {
		/* Disable texture request */
		ctx->texture[TEX_UNIT(rts->flags)] = NULL;
		return 0;
	}

	g3d_request_get(req);
	ctx->texture[TEX_UNIT(rts->flags)] = req;
	list_add_tail(&req->list, &ctx->state_restore_list);

	return 0;
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

static int g3d_apply_texture(struct g3d_drvdata *g3d, struct g3d_request *req)
{
	struct g3d_texture *tex = req_data(req);
	struct g3d_req_texture_setup *rts = tex->req;
	const uint32_t *data;
	uint32_t offset;
	uint32_t len;

	if (tex->obj)
		g3d_flush_texture(g3d, tex->obj);

	data = (const uint32_t *)&rts->control;

	offset = G3D_TEXTURE_BASE(TEX_UNIT(rts->flags));
	len = &rts->handle - &rts->control;

	while (len--) {
		g3d_write_relaxed(g3d, *(data++), offset);
		offset += 4;
	}

	/* Make sure register writes are not reordered across this point. */
	wmb();

	return 0;
}

/* Color buffer setup request */
static const struct g3d_req_colorbuffer_setup g3d_cbuffer_detached;

static int g3d_validate_colorbuffer(struct g3d_drvdata *g3d,
				    struct g3d_request *req)
{
	struct g3d_colorbuffer *cb = req_data(req);
	struct g3d_req_colorbuffer_setup *rcs = cb->req;
	int ret;

	if (rcs->flags & G3D_CBUFFER_DETACH) {
		cb->obj = NULL;
		memcpy(rcs, &g3d_cbuffer_detached, sizeof(*rcs));
		return 0;
	}

	cb->obj = exynos_drm_gem_lookup(g3d->subdrv.drm_dev,
						req->ctx->file, rcs->handle);
	if (!cb->obj) {
		dev_err(g3d->dev, "failed to lookup cbuffer BO\n");
		return -EINVAL;
	}

	ret = g3d_prepare_gem_object(g3d, cb->obj);
	if (ret)
		goto err_cobj;

	if (rcs->flags & G3D_CBUFFER_DIRTY) {
		exynos_drm_gem_sync_for_device(req->ctx->drm_dev,
						cb->obj, DMA_BIDIRECTIONAL);

		cb->obj->g3d_priv->fb_timestamp = g3d->fb_timestamp;
	}

	/* TODO: do some checks */
	rcs->offset += cb->obj->buffer->dma_addr;

	return 0;

err_cobj:
	drm_gem_object_unreference_unlocked(&cb->obj->base);

	return ret;
}

static void g3d_free_colorbuffer(struct g3d_drvdata *g3d,
				 struct g3d_request *req)
{
	struct g3d_colorbuffer *cb = req_data(req);

	if (cb->obj) {
		exynos_drm_gem_sync_for_cpu(req->ctx->drm_dev,
						cb->obj, DMA_BIDIRECTIONAL);
		drm_gem_object_unreference_unlocked(&cb->obj->base);
	}
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

static int g3d_process_colorbuffer(struct g3d_drvdata *g3d,
				   struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	struct g3d_colorbuffer *cb = req_data(req);
	struct g3d_req_colorbuffer_setup *rcs = cb->req;

	if (ctx->colorbuffer)
		g3d_request_put(ctx->colorbuffer);

	g3d_reg_cache_write_masked(ctx, rcs->fbctl, FGPF_FBCTL);
	g3d_reg_cache_write_masked(ctx, rcs->offset, FGPF_CBADDR);
	g3d_reg_cache_write_masked(ctx, rcs->width, FGPF_FBW);

	if (!cb->obj) {
		/* Framebuffer disable request */
		ctx->colorbuffer = 0;
		return 0;
	}

	g3d_request_get(req);
	ctx->colorbuffer = req;

	return 0;
}

static int g3d_apply_colorbuffer(struct g3d_drvdata *g3d,
				 struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	struct g3d_colorbuffer *cb = req_data(req);

	if (cb->obj)
		g3d_flush_colorbuffer(g3d, cb->obj);

	g3d_reg_cache_sync(g3d, ctx, FGPF_FBCTL);
	g3d_reg_cache_sync(g3d, ctx, FGPF_CBADDR);
	g3d_reg_cache_sync(g3d, ctx, FGPF_FBW);

	/* Make sure register writes are not reordered across this point. */
	wmb();

	return 0;
}

/* Depth buffer setup request */
static const struct g3d_req_depthbuffer_setup g3d_dbuffer_detached;

static int g3d_validate_depthbuffer(struct g3d_drvdata *g3d,
				    struct g3d_request *req)
{
	struct g3d_depthbuffer *db = req_data(req);
	struct g3d_req_depthbuffer_setup *rdb = db->req;
	int ret;

	if (rdb->flags & G3D_DBUFFER_DETACH) {
		db->obj = NULL;
		memcpy(rdb, &g3d_dbuffer_detached, sizeof(*rdb));
		return 0;
	}

	db->obj = exynos_drm_gem_lookup(g3d->subdrv.drm_dev,
						req->ctx->file, rdb->handle);
	if (!db->obj) {
		dev_err(g3d->dev, "failed to lookup dbuffer BO\n");
		return -EINVAL;
	}

	ret = g3d_prepare_gem_object(g3d, db->obj);
	if (ret)
		goto err_cobj;

	if (rdb->flags & G3D_DBUFFER_DIRTY) {
		exynos_drm_gem_sync_for_device(req->ctx->drm_dev,
						db->obj, DMA_BIDIRECTIONAL);

		db->obj->g3d_priv->fb_timestamp = g3d->fb_timestamp;
	}

	/* TODO: do some checks */
	rdb->offset += db->obj->buffer->dma_addr;

	return 0;

err_cobj:
	drm_gem_object_unreference_unlocked(&db->obj->base);

	return ret;
}

static void g3d_free_depthbuffer(struct g3d_drvdata *g3d,
				 struct g3d_request *req)
{
	struct g3d_depthbuffer *db = req_data(req);

	if (db->obj) {
		exynos_drm_gem_sync_for_cpu(req->ctx->drm_dev,
						db->obj, DMA_BIDIRECTIONAL);
		drm_gem_object_unreference_unlocked(&db->obj->base);
	}
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

static int g3d_process_depthbuffer(struct g3d_drvdata *g3d,
				   struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	struct g3d_depthbuffer *db = req_data(req);
	struct g3d_req_depthbuffer_setup *rdb = db->req;

	if (ctx->depthbuffer)
		g3d_request_put(ctx->depthbuffer);

	g3d_reg_cache_write(ctx, rdb->offset, FGPF_DBADDR);

	if (!db->obj) {
		/* Depth buffer disable request */
		ctx->depthbuffer = 0;

		ctx->register_masks[FGPF_FRONTST] = 0;
		ctx->register_masks[FGPF_DEPTHT] = 0;
		ctx->register_masks[FGPF_DBMSK] = 0;

		g3d_reg_cache_write(ctx, 0, FGPF_FRONTST);
		g3d_reg_cache_write(ctx, 0, FGPF_DEPTHT);
		g3d_reg_cache_write(ctx, 0xffff0001, FGPF_DBMSK);

		return 0;
	}

	ctx->register_masks[FGPF_FRONTST] = 0xffffffff;
	ctx->register_masks[FGPF_DEPTHT] = 0xffffffff;
	ctx->register_masks[FGPF_DBMSK] = 0xffffffff;

	g3d_request_get(req);
	ctx->depthbuffer = req;

	return 0;
}

static int g3d_apply_depthbuffer(struct g3d_drvdata *g3d,
				 struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	struct g3d_depthbuffer *db = req_data(req);

	if (db->obj)
		g3d_flush_depthbuffer(g3d, db->obj);

	g3d_reg_cache_sync(g3d, ctx, FGPF_DBADDR);
	g3d_reg_cache_sync(g3d, ctx, FGPF_FRONTST);
	g3d_reg_cache_sync(g3d, ctx, FGPF_DEPTHT);
	g3d_reg_cache_sync(g3d, ctx, FGPF_DBMSK);

	/* Make sure register writes are not reordered across this point. */
	wmb();

	return 0;
}

enum g3d_priv_request_id {
	G3D_REQUEST_FENCE = G3D_NUM_REQUESTS,

	G3D_NUM_ALL_REQUESTS
};

/* Supported request descriptors */
static const struct g3d_request_info g3d_requests[G3D_NUM_ALL_REQUESTS] = {
	[G3D_REQUEST_REGISTER_WRITE] = {
		.validate = g3d_validate_state_buffer,
		.handle = g3d_handle_state_update,
		.update_state = g3d_process_state_buffer,
		.apply = g3d_apply_state_buffer,
		.flags = G3D_REQUEST_PREEMPTION_POINT,
		.length = G3D_STATE_BUFFER_MAX_SIZE,
	},
	[G3D_REQUEST_SHADER_PROGRAM] = {
		.validate = g3d_validate_shader_program,
		.free = g3d_free_shader_program,
		.handle = g3d_handle_shader_program,
		.update_state = g3d_process_shader_program,
		.apply = g3d_apply_shader_program,
		.flags = G3D_REQUEST_PREEMPTION_POINT |
				G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = sizeof(struct g3d_req_shader_program),
		.priv_length = sizeof(struct g3d_shader_program),
	},
	[G3D_REQUEST_SHADER_DATA] = {
		.validate = g3d_validate_shader_data,
		.handle = g3d_handle_shader_data,
		.update_state = g3d_process_shader_data,
		.apply = g3d_apply_shader_data,
		.flags = G3D_REQUEST_PREEMPTION_POINT,
		.length = G3D_SHADER_DATA_MAX_SIZE,
	},
	[G3D_REQUEST_TEXTURE] = {
		.validate = g3d_validate_texture,
		.free = g3d_free_texture,
		.handle = g3d_handle_state_update,
		.update_state = g3d_process_texture,
		.apply = g3d_apply_texture,
		.flags = G3D_REQUEST_PREEMPTION_POINT |
				G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = sizeof(struct g3d_req_texture_setup),
		.priv_length = sizeof(struct g3d_texture),
	},
	[G3D_REQUEST_COLORBUFFER] = {
		.validate = g3d_validate_colorbuffer,
		.free = g3d_free_colorbuffer,
		.handle = g3d_handle_state_update,
		.update_state = g3d_process_colorbuffer,
		.apply = g3d_apply_colorbuffer,
		.flags = G3D_REQUEST_PREEMPTION_POINT |
				G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = sizeof(struct g3d_req_colorbuffer_setup),
		.priv_length = sizeof(struct g3d_colorbuffer),
	},
	[G3D_REQUEST_DEPTHBUFFER] = {
		.validate = g3d_validate_depthbuffer,
		.free = g3d_free_depthbuffer,
		.handle = g3d_handle_state_update,
		.update_state = g3d_process_depthbuffer,
		.apply = g3d_apply_depthbuffer,
		.flags = G3D_REQUEST_PREEMPTION_POINT |
				G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = sizeof(struct g3d_req_depthbuffer_setup),
		.priv_length = sizeof(struct g3d_depthbuffer),
	},
	[G3D_REQUEST_DRAW] = {
		.validate = g3d_validate_draw_buffer,
		.free = g3d_free_draw_buffer,
		.handle = g3d_handle_draw_buffer,
		.flags = G3D_REQUEST_MARK_PREEMPTION |
				G3D_REQUEST_UPDATE_STATE |
				G3D_REQUEST_STRICT_SIZE_CHECK,
		.length = sizeof(struct g3d_req_draw_buffer),
		.priv_length = sizeof(struct g3d_draw_buffer),
	},
	[G3D_REQUEST_FENCE] = {
		.handle = g3d_handle_fence,
		.flags = G3D_REQUEST_PREEMPTION_POINT |
				G3D_REQUEST_STRICT_SIZE_CHECK,
	},
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

	if (!list_empty(&g3d->ready_list)) {
		dev_dbg_events(g3d->dev, "%s (commands ready)\n", __func__);
		return;
	}

	clk_disable_unprepare(g3d->clock);

	pm_runtime_mark_last_busy(g3d->dev);
	pm_runtime_put_autosuspend(g3d->dev);

	set_bit(G3D_STATE_IDLE, &g3d->state);

	dev_dbg_events(g3d->dev, "%s (idle)\n", __func__);
}

/* Primitive request scheduler */
static struct g3d_request *g3d_get_next_request(struct g3d_drvdata *g3d)
{
	const struct g3d_request_info *req_info;
	struct g3d_request *req;
	struct g3d_context *ctx;

	ctx = list_first_entry(&g3d->ready_list, struct g3d_context, list);

	req = list_first_entry(&ctx->request_list, struct g3d_request, list);
	req_info = &g3d_requests[REQ_TYPE(&req->header)];

	if (req_info->flags & G3D_REQUEST_PREEMPTION_POINT
	    && test_and_clear_bit(G3D_STATE_PREEMPTED, &g3d->state)) {
		list_del(&ctx->list);
		list_add_tail(&ctx->list, &g3d->ready_list);

		ctx = list_first_entry(&g3d->ready_list,
						struct g3d_context, list);

		req = list_first_entry(&ctx->request_list,
						struct g3d_request, list);
		req_info = &g3d_requests[REQ_TYPE(&req->header)];
	}

	list_del(&req->list);

	if (list_empty(&ctx->request_list))
		list_del_init(&ctx->list);

	if (req_info->flags & G3D_REQUEST_MARK_PREEMPTION)
		set_bit(G3D_STATE_PREEMPTED, &g3d->state);

	return req;
}

/* Main loop of virtual request processor */
static int g3d_command_thread(void *data)
{
	struct g3d_drvdata *g3d = (struct g3d_drvdata *)data;
	const struct g3d_request_info *req_info;
	struct g3d_context *ctx;
	struct g3d_request *req;
	int ret;

	allow_signal(SIGTERM);
	set_freezable();

	while (!kthread_should_stop()) {
		mutex_lock(&g3d->stall_mutex);

		spin_lock(&g3d->ready_lock);

		while (list_empty(&g3d->ready_list)) {
			spin_unlock(&g3d->ready_lock);

			mutex_unlock(&g3d->stall_mutex);

			g3d_do_idle(g3d);

			ret = wait_event_freezable(g3d->ready_wq,
					!list_empty(&g3d->ready_list)
					|| kthread_should_stop());
			if (ret && kthread_should_stop())
				goto finish;

			dev_dbg_events(g3d->dev, "%s:%d ready_wq wake-up\n",
					__func__, __LINE__);

			mutex_lock(&g3d->stall_mutex);

			spin_lock(&g3d->ready_lock);
		}

		req = g3d_get_next_request(g3d);
		req_info = &g3d_requests[REQ_TYPE(&req->header)];
		ctx = req->ctx;

		spin_unlock(&g3d->ready_lock);

		if (req_info->flags & G3D_REQUEST_UPDATE_STATE)
			g3d_update_state(g3d, ctx);

		ret = g3d_request_handle(g3d, req);
		if (ret)
			dev_err(g3d->dev, "request %p (type %d) failed\n",
				req, REQ_TYPE(&req->header));

		mutex_unlock(&g3d->stall_mutex);
	}

finish:
	return 0;
}

/*
 * IRQ handler
 */
static irqreturn_t g3d_handle_irq(int irq, void *dev_id)
{
	struct g3d_drvdata *g3d = (struct g3d_drvdata *)dev_id;

	g3d_idle_irq_ack_and_disable(g3d);

	dev_dbg_events(g3d->dev, "%s:%d waking up ready_wq\n",
			__func__, __LINE__);
	wake_up_interruptible(&g3d->ready_wq);

	return IRQ_HANDLED;
}

/*
 * Request submission
 */
static struct g3d_request *g3d_allocate_fence(struct g3d_drvdata *g3d,
					      struct g3d_context *ctx,
					      uint32_t *fence)
{
	struct g3d_request *req;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req) {
		dev_err(g3d->dev, "failed to allocate g3d request\n");
		return ERR_PTR(-ENOMEM);
	}

	kref_init(&req->kref);
	req->ctx = ctx;
	req->header = REQ_HDR(G3D_REQUEST_FENCE, 0);

	*fence = ++ctx->submitted_fence;

	return req;
}

static struct g3d_request *g3d_allocate_request(struct g3d_drvdata *g3d,
	struct g3d_context *ctx, const uint32_t *req_data)
{
	const struct g3d_request_info *info;
	struct g3d_request *req;
	int ret;
	uint32_t length = REQ_LENGTH(req_data);
	uint8_t type = REQ_TYPE(req_data);

	dev_dbg_reqs(g3d->dev, "%s: type = %02x, length = %u\n",
			__func__, type, length);
	g3d_dump_req_data(REQ_DATA(req_data), length);

	if (type >= G3D_NUM_REQUESTS || g3d_requests[type].handle == NULL) {
		dev_err(g3d->dev, "invalid request type %d, ignoring\n", type);
		return ERR_PTR(-EINVAL);
	}

	info = &g3d_requests[type];

	if (info->length) {
		if (info->flags & G3D_REQUEST_STRICT_SIZE_CHECK
		    && unlikely(length != info->length)) {
			dev_err(g3d->dev, "invalid request data size (%u != %u, type = %u)\n",
				length, info->length, type);
			return ERR_PTR(-EINVAL);
		}

		if (unlikely(length > info->length)) {
			dev_err(g3d->dev, "request data size too big (%u > %u, type = %u)\n",
				length, info->length, type);
			return ERR_PTR(-EINVAL);
		}
	}

	ret = down_interruptible(&g3d->request_sem);
	if (ret)
		return ERR_PTR(ret);

	req = kmalloc(sizeof(*req) + info->priv_length + length, GFP_KERNEL);
	if (!req) {
		dev_err(g3d->dev, "failed to allocate g3d request\n");
		ret = -ENOMEM;
		goto err_up;
	}
	kref_init(&req->kref);
	req->ctx = ctx;
	req->header = req_data[0];

	memcpy(req->data + info->priv_length, REQ_DATA(req_data), length);

	if (!g3d_requests[type].validate)
		return req;

	ret = g3d_requests[type].validate(g3d, req);
	if (ret) {
		dev_err(g3d->dev,
			"failed to validate request data (type = %d)\n", type);
		goto err_free;
	}

	return req;

err_free:
	kfree(req);
err_up:
	up(&g3d->request_sem);

	return ERR_PTR(ret);
}

static void g3d_post_requests(struct g3d_drvdata *g3d, struct g3d_context *ctx,
			      struct list_head *list)
{
	bool wake;

	spin_lock(&g3d->ready_lock);

	list_splice_tail(list, &ctx->request_list);

	wake = list_empty(&g3d->ready_list);

	if (list_empty(&ctx->list))
		list_add_tail(&ctx->list, &g3d->ready_list);

	if (wake) {
		dev_dbg_events(g3d->dev, "%s:%d waking up ready_wq\n",
			__func__, __LINE__);
		wake_up_interruptible_sync(&g3d->ready_wq);
	}

	spin_unlock(&g3d->ready_lock);
}

int s3c6410_g3d_submit(struct drm_device *dev, void *data,
		       struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_context *ctx = file_priv->g3d_priv;
	struct drm_exynos_g3d_submit *submit = data;
	struct g3d_validation_data validation_copy;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct exynos_drm_gem_obj *gem;
	struct g3d_request *req, *n;
	LIST_HEAD(submit_list);
	const uint32_t *sdata;
	uint32_t length;
	uint32_t fence;
	unsigned int i;
	int ret = 0;

	dev_dbg_fops(g3d->dev,
			"%s: handle = %08x, offset = %08x, length = %08x\n",
			__func__, submit->handle, submit->offset,
			submit->length);

	gem = exynos_drm_gem_lookup(dev, file, submit->handle);
	if (!gem) {
		dev_err(g3d->dev, "failed to lookup submit buffer\n");
		return -EINVAL;
	}

	if (unlikely(!gem->buffer->kvaddr)) {
		dev_err(g3d->dev, "submit buffers need to have kernel mapping\n");
		ret = -EINVAL;
		goto err_gem_unref;
	}

	if (unlikely(submit->offset + submit->length > gem->size)) {
		dev_err(g3d->dev, "submit data bigger than backing buffer\n");
		ret = -EINVAL;
		goto err_gem_unref;
	}

	if (unlikely(submit->offset % 4)) {
		dev_err(g3d->dev, "submit data incorrectly aligned\n");
		ret = -EINVAL;
		goto err_gem_unref;
	}

	sdata = gem->buffer->kvaddr + submit->offset;
	length = submit->length;
	memcpy(&validation_copy, &ctx->validation, sizeof(validation_copy));

	for (i = 0; length; ++i) {
		uint32_t req_length = REQ_LENGTH(sdata) + sizeof(*sdata);

		if (unlikely(req_length > length)) {
			dev_err(g3d->dev,
				"submit data truncated at request %u\n", i);
			ret = -EINVAL;
			goto err_free;
		}

		if (unlikely(req_length % 4)) {
			dev_err(g3d->dev,
				"submit data not aligned at request %u\n", i);
			ret = -EINVAL;
			goto err_free;
		}

		req = g3d_allocate_request(g3d, ctx, sdata);
		if (IS_ERR(req)) {
			ret = PTR_ERR(req);
			goto err_free;
		}

		list_add_tail(&req->list, &submit_list);

		sdata += req_length / sizeof(*sdata);
		length -= req_length;
	}

	if (!i) {
		ret = -EINVAL;
		goto err_gem_unref;
	}

	req = g3d_allocate_fence(g3d, ctx, &fence);
	if (IS_ERR(req)) {
		ret = PTR_ERR(req);
		goto err_free;
	}
	list_add_tail(&req->list, &submit_list);

	g3d_post_requests(g3d, ctx, &submit_list);
	drm_gem_object_unreference_unlocked(&gem->base);

	submit->fence = fence;

	return 0;

err_free:
	list_for_each_entry_safe(req, n, &submit_list, list)
		g3d_request_put(req);
	memcpy(&ctx->validation, &validation_copy, sizeof(validation_copy));
err_gem_unref:
	drm_gem_object_unreference_unlocked(&gem->base);

	return ret;
}

static inline bool fence_completed(struct g3d_context *ctx, uint32_t fence)
{
	return ctx->completed_fence >= fence;
}

static int g3d_wait_interruptible(struct g3d_context *ctx, uint32_t fence,
				  struct timespec *timeout)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	unsigned long remaining_jiffies;
	unsigned long timeout_jiffies;
	unsigned long start_jiffies;
	int ret;

	if (fence > ctx->submitted_fence) {
		dev_err(g3d->dev, "waiting on invalid fence: %u (of %u)\n",
			fence, ctx->submitted_fence);
		return -EINVAL;
	}

	if (!timeout) {
		/* no-wait: */
		if (fence_completed(ctx, fence))
			return 0;

		return -EBUSY;
	}

	timeout_jiffies = timespec_to_jiffies(timeout);
	start_jiffies = jiffies;

	if (time_after(start_jiffies, timeout_jiffies))
		remaining_jiffies = 0;
	else
		remaining_jiffies = timeout_jiffies - start_jiffies;

	ret = wait_event_interruptible_timeout(ctx->fence_wq,
			fence_completed(ctx, fence),
			remaining_jiffies);
	if (ret == 0) {
		dev_dbg_events(g3d->dev,
			"timeout waiting for fence: %u (completed: %u)",
			fence, ctx->completed_fence);
		ret = -ETIMEDOUT;
	} else if (ret > 0) {
		ret = 0;
	}

	return ret;
}

int s3c6410_g3d_wait(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_context *ctx = file_priv->g3d_priv;
	struct drm_exynos_g3d_wait *wait = data;
	struct timespec ts;

	ts.tv_sec = wait->timeout.tv_sec;
	ts.tv_nsec = wait->timeout.tv_nsec;

	return g3d_wait_interruptible(ctx, wait->fence, &ts);
}

/*
 * DRM subdriver
 */
static int g3d_open(struct drm_device *drm_dev, struct device *dev,
		    struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);
	struct g3d_context *ctx;

	ctx = kzalloc(sizeof(struct g3d_context), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->drm_dev = drm_dev;
	ctx->g3d = g3d;
	ctx->file = file;

	INIT_LIST_HEAD(&ctx->list);
	init_waitqueue_head(&ctx->fence_wq);
	INIT_LIST_HEAD(&ctx->state_update_list);
	INIT_LIST_HEAD(&ctx->state_restore_list);
	INIT_LIST_HEAD(&ctx->request_list);

	memcpy(ctx->register_masks, g3d_register_masks_def,
						sizeof(ctx->register_masks));

	/* Set private data */
	file_priv->g3d_priv = ctx;

	dev_dbg_fops(dev, "device opened\n");

	return 0;
}

static void g3d_close(struct drm_device *drm_dev, struct device *dev,
		      struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g3d_context *ctx = file_priv->g3d_priv;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct g3d_request *req, *n;

	mutex_lock(&g3d->stall_mutex);

	spin_lock(&g3d->ready_lock);

	if (!list_empty(&ctx->list))
		list_del(&ctx->list);

	spin_unlock(&g3d->ready_lock);

	mutex_unlock(&g3d->stall_mutex);

	list_for_each_entry_safe(req, n, &ctx->request_list, list) {
		list_del(&req->list);
		g3d_request_release(&req->kref);
	}

	list_for_each_entry_safe(req, n, &ctx->state_update_list, list) {
		list_del(&req->list);
		g3d_request_release(&req->kref);
	}

	list_for_each_entry_safe(req, n, &ctx->state_restore_list, list) {
		list_del(&req->list);
		g3d_request_release(&req->kref);
	}

	if (ctx->colorbuffer)
		g3d_request_put(ctx->colorbuffer);
	if (ctx->depthbuffer)
		g3d_request_put(ctx->colorbuffer);

	kfree(ctx);

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
	uint32_t version;
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

	g3d->dev = &pdev->dev;
	spin_lock_init(&g3d->ready_lock);
	INIT_LIST_HEAD(&g3d->ready_list);
	init_waitqueue_head(&g3d->ready_wq);
	mutex_init(&g3d->stall_mutex);
	sema_init(&g3d->request_sem, G3D_MAX_REQUESTS);

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
	version = g3d_read(g3d, G3D_FGGB_VERSION);
	clk_disable_unprepare(g3d->clock);

	dev_info(&pdev->dev, "detected FIMG-3DSE version %d.%d.%d\n",
		version >> 24, (version >> 16) & 0xff, (version >> 8) & 0xff);

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
