/*
 * DRM driver for Samsung FIMG-2D graphics accelerator
 *
 * Copyright (C) 2014 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifdef CONFIG_DRM_EXYNOS_S3C6410_G2D_DEBUG
#define DEBUG
#endif

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kref.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <drm/drmP.h>
#include <drm/exynos_drm.h>
#include "exynos_drm_drv.h"
#include "exynos_drm_gem.h"

/*
 * Various definitions
 */

#define G2D_AUTOSUSPEND_DELAY		50
#define G2D_WATCHDOG_TIMEOUT		msecs_to_jiffies(250)
#define G2D_RESET_TIMEOUT_US		10
#define G2D_REG_WRITE_MAX_SIZE		\
	(G2D_NUM_REGISTERS * NUM_G2D_REG_WRITE_WORDS)
#define G2D_NUM_COMMANDS		6
#define G2D_FIFO_SIZE			32

/*
 * Registers
 */

enum g2d_register {
	/* Protected registers */
	SRC_RES_REG,
	DST_RES_REG,
	SRC_COLOR_MODE_REG,
	DST_COLOR_MODE_REG,
	SRC_BASE_ADDR_REG,
	DST_BASE_ADDR_REG,

	G2D_NUM_PROT_REGISTERS,

	/* Clipping window */
	CW_LT_REG = G2D_NUM_PROT_REGISTERS,
	CW_RB_REG,

	/* Coordinates */
	COORD0_REG,
	COORD1_REG,
	COORD2_REG,
	COORD3_REG,

	/* Rotation */
	ROT_OC_REG,
	ROTATE_REG,

	/* Endianess mode */
	ENDIAN_REG,

	/* X, Y increment setting */
	X_INCR_REG,
	Y_INCR_REG,

	/* ROP and alpha setting */
	ROP_REG,
	ALPHA_REG,

	/* Color */
	FG_COLOR_REG,
	BG_COLOR_REG,
	BS_COLOR_REG,

	/* Color keying */
	COLORKEY_CNTL_REG,
	COLORKEY_DR_MIN_REG,
	COLORKEY_DR_MAX_REG,

	G2D_NUM_REGISTERS
};

#define G2D_CONTROL_REG				0x0000
#define		G2D_CONTROL_R			BIT(0)
#define G2D_INTEN_REG				0x0004
#define		G2D_INTEN_ACF			BIT(9)
#define G2D_FIFO_INTC_REG			0x0008
#define G2D_INTC_PEND_REG			0x000c
#define		G2D_INTC_PEND_CLRSEL		BIT(31)
#define G2D_FIFO_STAT_REG			0x0010
#define		G2D_FIFO_STAT_USED_SHIFT	1
#define		G2D_FIFO_STAT_USED_MASK		0x3f
#define G2D_CMD_REG(x)				(0x0100 + 4 * (x))
#define G2D_PATTERN_REG(x)			(0x0600 + 4 * (x))

/*
 * Requests
 */

#define G2D_REQUEST_REG_WRITE		0
#define		G2D_REG_WRITE_REG	0
#define		G2D_REG_WRITE_VAL	1
#define		NUM_G2D_REG_WRITE_WORDS	2
#define G2D_REQUEST_SET_SRC		1
#define G2D_REQUEST_SET_DST		2
#define		G2D_SURFACE_RES		0
#define		G2D_SURFACE_COLOR_MODE	1
#define		G2D_SURFACE_OFFSET	2
#define		G2D_SURFACE_HANDLE	3
#define		NUM_G2D_SURFACE_WORDS	4
#define G2D_REQUEST_CMD			3
#define		G2D_CMD_NUM		0
#define		G2D_CMD_ARG		1
#define		NUM_G2D_CMD_WORDS	2

#define G2D_NUM_REQUESTS		4

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

static inline const uint32_t *req_next(const uint32_t *req)
{
	return req_data(req) + req_length(req);
}

/*
 * Private data types
 */
struct g2d_context;

/* Common driver data */
struct g2d_submit;

struct g2d_drvdata {
	struct exynos_drm_subdrv subdrv;
	void __iomem *base;
	int irq;
	struct resource *mem;
	struct clk *clock;
	struct device *dev;

	spinlock_t ready_lock;
	struct list_head submit_list;
	struct timer_list watchdog_timer;
	struct list_head retired_submits;
	struct work_struct retire_work;

	atomic_t next_fence;
	uint32_t submitted_fence;
	uint32_t completed_fence;
	wait_queue_head_t fence_wq;

	struct g2d_context *last_ctx;
};

/* Per-file private data */
struct g2d_priv {
	struct g2d_drvdata *g2d;
	struct idr pipes_idr;
	struct rw_semaphore pipes_idr_sem;
};

/* Per-pipe context data */
struct g2d_surface {
	struct exynos_drm_gem_obj *obj;
};

enum {
	G2D_SURFACE_SRC,
	G2D_SURFACE_DST,

	G2D_NUM_SURFACES
};

struct g2d_context {
	struct g2d_drvdata *g2d;
	struct drm_device *drm_dev;
	struct drm_file *file;
	struct kref ref;

	struct mutex submit_mutex;

	uint32_t registers[G2D_NUM_REGISTERS];
	uint32_t register_masks[G2D_NUM_PROT_REGISTERS];
	unsigned long register_dirty[BITS_TO_LONGS(G2D_NUM_REGISTERS)];
	unsigned int num_register_dirty;

	struct g2d_surface surface[G2D_NUM_SURFACES];

	bool aborted:1;
};

struct g2d_submit {
	struct g2d_context *ctx;
	struct idr gem_idr;
	uint32_t fence;
	struct list_head list;

	uint32_t ptr;

	uint32_t num_words;
	uint32_t words[];
};

struct g2d_request_info {
	int (*validate)(struct g2d_submit *, uint32_t *);
	int (*handle)(struct g2d_submit *, const uint32_t *);
	size_t length;
	bool strict_size_check:1;
};

static const uint32_t g2d_register_masks_def[G2D_NUM_PROT_REGISTERS] = {
	[SRC_RES_REG] = 0,
	[DST_RES_REG] = 0,
	[SRC_COLOR_MODE_REG] = 0,
	[DST_COLOR_MODE_REG] = 0,
	[SRC_BASE_ADDR_REG] = 0,
	[DST_BASE_ADDR_REG] = 0,
};

static const uint32_t g2d_registers[G2D_NUM_REGISTERS] = {
	/*
	 * Protected registers
	 */

	[SRC_RES_REG] = 0x200,
	[DST_RES_REG] = 0x210,
	[SRC_COLOR_MODE_REG] = 0x510,
	[DST_COLOR_MODE_REG] = 0x514,
	[SRC_BASE_ADDR_REG] = 0x730,
	[DST_BASE_ADDR_REG] = 0x734,

	/*
	 * Public registers
	 */

	/* Clipping window */
	[CW_LT_REG] = 0x220,
	[CW_RB_REG] = 0x230,

	/* Coordinates */
	[COORD0_REG] = 0x300,
	[COORD1_REG] = 0x310,
	[COORD2_REG] = 0x320,
	[COORD3_REG] = 0x330,

	/* Rotation */
	[ROT_OC_REG] = 0x340,
	[ROTATE_REG] = 0x34c,

	/* Endianess setting */
	[ENDIAN_REG] = 0x350,

	/* X, Y increment setting */
	[X_INCR_REG] = 0x400,
	[Y_INCR_REG] = 0x404,

	/* ROP and alpha setting */
	[ROP_REG] = 0x410,
	[ALPHA_REG] = 0x420,

	/* Color */
	[FG_COLOR_REG] = 0x500,
	[BG_COLOR_REG] = 0x504,
	[BS_COLOR_REG] = 0x508,

	/* Color keying */
	[COLORKEY_CNTL_REG] = 0x720,
	[COLORKEY_DR_MIN_REG] = 0x724,
	[COLORKEY_DR_MAX_REG] = 0x728,
};

static const struct g2d_request_info g2d_requests[];

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
#define DEBUG_IO		(1 << 30)

#ifdef DEBUG

#define dev_dbg_mask(mask, ...)				\
	do {						\
		if (s3c6410_g2d_debug_mask & (mask))	\
			dev_dbg(__VA_ARGS__);		\
	} while (0)

static unsigned int s3c6410_g2d_debug_mask;
module_param(s3c6410_g2d_debug_mask, uint, 0644);

static void g2d_dump_req_data(const void *buf, size_t len)
{
	if (s3c6410_g2d_debug_mask & DEBUG_REQ_DATA)
		print_hex_dump(KERN_DEBUG, "    ", DUMP_PREFIX_OFFSET,
				32, 4, buf, len * 4, false);
}
#else /* !DEBUG */

#define dev_dbg_mask(mask, ...)				\
	do {						\
		if (0)	\
			dev_dbg(__VA_ARGS__);		\
	} while (0)

static inline void g2d_dump_req_data(const void *buf, size_t len) {}

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
static inline void g2d_write_relaxed(struct g2d_drvdata *g2d,
				     uint32_t val, uint32_t reg)
{
	dev_dbg_io(g2d->dev, "%s(%08x, %08x)\n", __func__, val, reg);
	writel_relaxed(val, g2d->base + reg);
}

static inline void g2d_write(struct g2d_drvdata *g2d,
			     uint32_t val, uint32_t reg)
{
	dev_dbg_io(g2d->dev, "%s(%08x, %08x)\n", __func__, val, reg);
	writel(val, g2d->base + reg);
}

static inline uint32_t g2d_read(struct g2d_drvdata *g2d, uint32_t reg)
{
	uint32_t val;

	val = readl(g2d->base + reg);
	dev_dbg_io(g2d->dev, "%s(%08x) = %08x\n", __func__, reg, val);

	return val;
}

/*
 * Hardware operations
 */
static void g2d_soft_reset(struct g2d_drvdata *g2d)
{
	ktime_t start;

	dev_dbg_trace(g2d->dev, "%s\n", __func__);

	g2d_write(g2d, G2D_CONTROL_R, G2D_CONTROL_REG);

	start = ktime_get();
	while (ktime_us_delta(ktime_get(), start) < G2D_RESET_TIMEOUT_US)
		if (!(g2d_read(g2d, G2D_CONTROL_REG) & G2D_CONTROL_R))
			break;

	if (g2d_read(g2d, G2D_CONTROL_REG) & G2D_CONTROL_R)
		dev_err(g2d->dev, "G2D reset timed out\n");
}

static void g2d_idle_irq_ack_and_disable(struct g2d_drvdata *g2d)
{
	g2d_write(g2d, 0, G2D_INTEN_REG);
	g2d_write(g2d, 0, G2D_INTC_PEND_REG);
}

static void g2d_idle_irq_enable(struct g2d_drvdata *g2d)
{
	g2d_write(g2d, G2D_INTEN_ACF, G2D_INTEN_REG);
}

static void g2d_initialize(struct g2d_drvdata *g2d)
{
	g2d_soft_reset(g2d);
	g2d_idle_irq_ack_and_disable(g2d);
	g2d->last_ctx = NULL;
}

/*
 * Register cache
 */

static inline uint32_t update_bits(uint32_t old, uint32_t new, uint32_t mask)
{
	return (old & ~mask) | (new & mask);
}

static void g2d_reg_cache_write(struct g2d_context *ctx,
				       uint32_t value, uint32_t reg)
{
	ctx->registers[reg] = value;

	if (!test_and_set_bit(reg, ctx->register_dirty))
		++ctx->num_register_dirty;
}

static void g2d_reg_cache_sync_all(struct g2d_drvdata *g2d,
				   struct g2d_context *ctx)
{
	unsigned int reg;

	for_each_set_bit(reg, ctx->register_dirty, G2D_NUM_REGISTERS)
		g2d_write_relaxed(g2d, ctx->registers[reg], g2d_registers[reg]);

	/* Make sure register writes are not reordered across this point. */
	wmb();

	bitmap_zero(ctx->register_dirty, G2D_NUM_REGISTERS);
	ctx->num_register_dirty = 0;
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

static struct exynos_drm_gem_obj *g2d_lookup_gem(struct g2d_submit *submit,
						struct exynos_drm_gem_obj *obj,
						uint32_t sid)
{
	return idr_replace(&submit->gem_idr, obj, sid);
}

static int g2d_register_gem(struct g2d_submit *submit,
			    struct exynos_drm_gem_obj *obj)
{
	struct g2d_context *ctx = submit->ctx;
	struct g2d_drvdata *g2d = ctx->g2d;
	int ret;

	ret = idr_alloc(&submit->gem_idr, obj, 1, 0, GFP_KERNEL);
	if (ret < 0) {
		dev_err(g2d->dev,
			"failed to allocate GEM submission ID (%d)\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Requests
 */

/* Request callback wrappers */
static int g2d_request_handle(struct g2d_submit *submit,
				      const uint32_t *req)
{
	struct g2d_context *ctx = submit->ctx;
	struct g2d_drvdata *g2d = ctx->g2d;
	const struct g2d_request_info *req_info;

	req_info = &g2d_requests[req_type(req)];

	dev_dbg_reqs(g2d->dev, "%s: %d\n", __func__, req_type(req));

	return req_info->handle(submit, req);
}

/* Register state update request */
static int g2d_validate_reg_write(struct g2d_submit *submit, uint32_t *req)
{
	struct g2d_context *ctx = submit->ctx;
	struct g2d_drvdata *g2d = ctx->g2d;
	unsigned int count = req_length(req) / NUM_G2D_REG_WRITE_WORDS;

	req = req_data_wr(req);

	while (count--) {
		uint32_t reg = req[G2D_REG_WRITE_REG];

		if (unlikely(reg >= G2D_NUM_REGISTERS)) {
			dev_err(g2d->dev, "register index out of range\n");
			return -EINVAL;
		}

		req += NUM_G2D_REG_WRITE_WORDS;
	}

	return 0;
}

static int g2d_handle_reg_write(struct g2d_submit *submit, const uint32_t *req)
{
	struct g2d_context *ctx = submit->ctx;
	unsigned int count = req_length(req) / NUM_G2D_REG_WRITE_WORDS;

	req = req_data(req);

	while (count--) {
		g2d_reg_cache_write(ctx, req[G2D_REG_WRITE_VAL],
					req[G2D_REG_WRITE_REG]);
		req += NUM_G2D_REG_WRITE_WORDS;
	}

	return 0;
}

/* Surface setup requests */
static int g2d_validate_surface(struct g2d_submit *submit, uint32_t *req)
{
	struct g2d_context *ctx = submit->ctx;
	struct g2d_drvdata *g2d = ctx->g2d;
	struct exynos_drm_gem_obj *obj = NULL;
	int ret;

	req = req_data_wr(req);

	if (!req[G2D_SURFACE_RES]) {
		obj = exynos_drm_gem_lookup(g2d->subdrv.drm_dev, ctx->file,
						req[G2D_SURFACE_HANDLE]);
		if (!obj) {
			dev_err(g2d->dev, "failed to lookup surface BO\n");
			return -EINVAL;
		}

		/* TODO: check size */
		req[G2D_SURFACE_OFFSET] += obj->buffer->dma_addr;
	}

	ret = g2d_register_gem(submit, obj);
	if (!ret) {
		dev_err(g2d->dev, "failed to lookup surface BO\n");
		goto err_obj;
	}

	req[G2D_SURFACE_HANDLE] = ret;
	return 0;

err_obj:
	if (obj)
		drm_gem_object_unreference_unlocked(&obj->base);

	return ret;
}

static int g2d_exchange_surfaces(struct g2d_submit *submit, unsigned surface,
				 uint32_t handle)
{
	struct g2d_context *ctx = submit->ctx;
	struct exynos_drm_gem_obj *obj, *prev_obj;

	prev_obj = ctx->surface[surface].obj;
	obj = g2d_lookup_gem(submit, prev_obj, handle);
	ctx->surface[surface].obj = obj;

	return obj != 0;
}

static int g2d_handle_set_src(struct g2d_submit *submit, const uint32_t *req)
{
	struct g2d_context *ctx = submit->ctx;
	int ret;

	req = req_data(req);

	ret = g2d_exchange_surfaces(submit, G2D_SURFACE_SRC,
					req[G2D_SURFACE_HANDLE]);
	if (!ret) {
		g2d_reg_cache_write(ctx, 0, SRC_RES_REG);
		g2d_reg_cache_write(ctx, 0, SRC_COLOR_MODE_REG);
		g2d_reg_cache_write(ctx, 0, SRC_BASE_ADDR_REG);

		return 0;
	}

	g2d_reg_cache_write(ctx, req[G2D_SURFACE_RES], SRC_RES_REG);
	g2d_reg_cache_write(ctx, req[G2D_SURFACE_COLOR_MODE],
				SRC_COLOR_MODE_REG);
	g2d_reg_cache_write(ctx, req[G2D_SURFACE_OFFSET],
				SRC_BASE_ADDR_REG);

	return 0;
}

static int g2d_handle_set_dst(struct g2d_submit *submit, const uint32_t *req)
{
	struct g2d_context *ctx = submit->ctx;
	int ret;

	req = req_data(req);

	ret = g2d_exchange_surfaces(submit, G2D_SURFACE_DST,
					req[G2D_SURFACE_HANDLE]);
	if (!ret) {
		g2d_reg_cache_write(ctx, 0, DST_RES_REG);
		g2d_reg_cache_write(ctx, 0, DST_COLOR_MODE_REG);
		g2d_reg_cache_write(ctx, 0, DST_BASE_ADDR_REG);

		return 0;
	}

	g2d_reg_cache_write(ctx, req[G2D_SURFACE_RES], DST_RES_REG);
	g2d_reg_cache_write(ctx, req[G2D_SURFACE_COLOR_MODE],
				DST_COLOR_MODE_REG);
	g2d_reg_cache_write(ctx, req[G2D_SURFACE_OFFSET],
				DST_BASE_ADDR_REG);

	return 0;
}

/* Command request */
static int g2d_validate_cmd(struct g2d_submit *submit, uint32_t *req)
{
	struct g2d_context *ctx = submit->ctx;
	struct g2d_drvdata *g2d = ctx->g2d;

	req = req_data_wr(req);

	if (req[G2D_CMD_NUM] >= G2D_NUM_COMMANDS) {
		dev_err(g2d->dev, "invalid command request\n");
		return -EINVAL;
	}

	return 0;
}

static int g2d_handle_cmd(struct g2d_submit *submit, const uint32_t *req)
{
	struct g2d_context *ctx = submit->ctx;
	struct g2d_drvdata *g2d = ctx->g2d;

	if (g2d->last_ctx != ctx)
		bitmap_fill(ctx->register_dirty, G2D_NUM_REGISTERS);

	g2d_reg_cache_sync_all(g2d, ctx);

	g2d->last_ctx = ctx;

	req = req_data(req);
	g2d_write(g2d, req[G2D_CMD_ARG], G2D_CMD_REG(req[G2D_CMD_NUM]));

	return 1;
}

/* Supported request descriptors */
static const struct g2d_request_info g2d_requests[G2D_NUM_REQUESTS] = {
	[G2D_REQUEST_REG_WRITE] = {
		.validate = g2d_validate_reg_write,
		.handle = g2d_handle_reg_write,
		.length = G2D_REG_WRITE_MAX_SIZE,
	},
	[G2D_REQUEST_SET_SRC] = {
		.validate = g2d_validate_surface,
		.handle = g2d_handle_set_src,
		.length = NUM_G2D_SURFACE_WORDS,
		.strict_size_check = true,
	},
	[G2D_REQUEST_SET_DST] = {
		.validate = g2d_validate_surface,
		.handle = g2d_handle_set_dst,
		.length = NUM_G2D_SURFACE_WORDS,
		.strict_size_check = true,
	},
	[G2D_REQUEST_CMD] = {
		.validate = g2d_validate_cmd,
		.handle = g2d_handle_cmd,
		.length = NUM_G2D_CMD_WORDS,
		.strict_size_check = true,
	},
};

static inline bool fence_completed(struct g2d_drvdata *g2d, uint32_t fence)
{
	return g2d->completed_fence >= fence;
}

static int g2d_wait_interruptible_timeout(struct g2d_drvdata *g2d,
					  uint32_t fence,
					  struct timespec *timeout)
{
	unsigned long remaining_jiffies;
	unsigned long timeout_jiffies;
	unsigned long start_jiffies;
	int ret;

	if (fence > g2d->submitted_fence) {
		dev_err(g2d->dev, "waiting on invalid fence: %u (of %u)\n",
			fence, g2d->submitted_fence);
		return -EINVAL;
	}

	if (!timeout) {
		/* no-wait: */
		if (fence_completed(g2d, fence))
			return 0;

		return -EBUSY;
	}

	timeout_jiffies = timespec_to_jiffies(timeout);
	start_jiffies = jiffies;

	if (!timeout_jiffies) {
		ret = wait_event_interruptible(g2d->fence_wq,
						fence_completed(g2d, fence));
	} else {
		if (time_after(start_jiffies, timeout_jiffies))
			remaining_jiffies = 0;
		else
			remaining_jiffies = timeout_jiffies - start_jiffies;

		ret = wait_event_interruptible_timeout(g2d->fence_wq,
						fence_completed(g2d, fence),
						remaining_jiffies);
		if (ret == 0) {
			dev_dbg_events(g2d->dev,
				"timeout waiting for fence: %u (completed: %u)",
				fence, g2d->completed_fence);
			ret = -ETIMEDOUT;
		} else if (ret > 0) {
			ret = 0;
		}
	}

	return ret;
}

/* Fence request */
static int g2d_signal_fence(struct g2d_drvdata *g2d, uint32_t fence)
{
	g2d->completed_fence = fence;

	wake_up_all(&g2d->fence_wq);

	return 0;
}

static int g2d_gem_idr_cleanup(int id, void *p, void *data)
{
	struct exynos_drm_gem_obj *obj = p;

	if (obj)
		drm_gem_object_unreference_unlocked(&obj->base);

	return 0;
}

static void g2d_release_context(struct kref *kref);

static void g2d_release_submit(struct g2d_submit *submit)
{
	struct g2d_context *ctx = submit->ctx;
	struct g2d_drvdata *g2d = ctx->g2d;

	list_del(&submit->list);
	g2d_signal_fence(g2d, submit->fence);
	kref_put(&ctx->ref, g2d_release_context);
	idr_for_each(&submit->gem_idr, g2d_gem_idr_cleanup, NULL);
	idr_destroy(&submit->gem_idr);
	vfree(submit);
}

static void g2d_retire_work_func(struct work_struct *work)
{
	struct g2d_drvdata *g2d = container_of(work, struct g2d_drvdata,
						retire_work);
	LIST_HEAD(retired_submits);
	unsigned long flags;

	spin_lock_irqsave(&g2d->ready_lock, flags);

	list_splice(&g2d->retired_submits, &retired_submits);

	spin_unlock_irqrestore(&g2d->ready_lock, flags);

	while (!list_empty(&retired_submits)) {
		struct g2d_submit *submit = list_first_entry(&retired_submits,
						struct g2d_submit, list);

		g2d_release_submit(submit);
	}
}

static void g2d_retire_submit(struct g2d_drvdata *g2d,
			      struct g2d_submit *submit)
{
	unsigned long flags;

	spin_lock_irqsave(&g2d->ready_lock, flags);

	list_del(&submit->list);
	list_add_tail(&submit->list, &g2d->retired_submits);

	spin_unlock_irqrestore(&g2d->ready_lock, flags);

	schedule_work(&g2d->retire_work);
}

static int g2d_process_one_submit(struct g2d_drvdata *g2d,
				  struct g2d_submit *submit)
{
	struct g2d_context *ctx = submit->ctx;
	const uint32_t *cur, *end;
	int ret = 0;

	cur = submit->words + submit->ptr;
	end = submit->words + submit->num_words;

	if (cur >= end || ctx->aborted) {
		g2d_retire_submit(g2d, submit);
		return 0;
	}

	while (cur < end) {
		ret = g2d_request_handle(submit, cur);
		if (ret)
			break;

		cur = req_next(cur);
	}

	submit->ptr = cur - submit->words;
	return ret;
}

static int g2d_process_submits(struct g2d_drvdata *g2d)
{
	struct g2d_submit *submit;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&g2d->ready_lock, flags);

	while (!list_empty(&g2d->submit_list)) {
		submit = list_first_entry(&g2d->submit_list, struct g2d_submit,
					  list);

		spin_unlock_irqrestore(&g2d->ready_lock, flags);

		ret = g2d_process_one_submit(g2d, submit);
		if (ret)
			return ret;

		spin_lock_irqsave(&g2d->ready_lock, flags);
	}

	spin_unlock_irqrestore(&g2d->ready_lock, flags);

	return 0;
}

/*
 * IRQ handler
 */
static void g2d_handle_event(struct g2d_drvdata *g2d)
{
	int ret;

	ret = g2d_process_submits(g2d);
	if (ret) {
		mod_timer(&g2d->watchdog_timer, G2D_WATCHDOG_TIMEOUT);
		g2d_idle_irq_enable(g2d);
		return;
	}

	clk_disable(g2d->clock);
	pm_runtime_mark_last_busy(g2d->dev);
	pm_runtime_put_autosuspend(g2d->dev);
}

static irqreturn_t g2d_handle_irq(int irq, void *dev_id)
{
	struct g2d_drvdata *g2d = (struct g2d_drvdata *)dev_id;

	g2d_idle_irq_ack_and_disable(g2d);
	del_timer(&g2d->watchdog_timer);

	dev_dbg_events(g2d->dev, "%s:%d\n", __func__, __LINE__);

	g2d_handle_event(g2d);

	return IRQ_HANDLED;
}

static void g2d_watchdog_timer(unsigned long data)
{
	struct g2d_drvdata *g2d = (struct g2d_drvdata *)data;

	dev_err(g2d->dev, "G2D request timed out, resetting\n");

	g2d_initialize(g2d);
	g2d_handle_event(g2d);
}

/*
 * Request submission
 */
static int g2d_validate_request(struct g2d_drvdata *g2d,
				struct g2d_submit *submit, uint32_t *req)
{
	const struct g2d_request_info *info;
	uint32_t length = req_length(req);
	uint8_t type = req_type(req);
	int ret;

	dev_dbg_reqs(g2d->dev, "%s: type = %02x, length = %u\n",
			__func__, type, length);
	g2d_dump_req_data(req_data(req), length);

	if (type >= G2D_NUM_REQUESTS) {
		dev_err(g2d->dev, "invalid request type %d\n", type);
		return -EINVAL;
	}

	info = &g2d_requests[type];

	if (info->length) {
		if (info->strict_size_check
		    && unlikely(length != info->length)) {
			dev_err(g2d->dev, "invalid request data size (%u != %u, type = %u)\n",
				length, info->length, type);
			return -EINVAL;
		}

		if (unlikely(length > info->length)) {
			dev_err(g2d->dev, "request data size too big (%u > %u, type = %u)\n",
				length, info->length, type);
			return -EINVAL;
		}
	}

	if (!info->validate)
		return 0;

	ret = info->validate(submit, req);
	if (ret) {
		dev_err(g2d->dev,
			"failed to validate request data (type = %d)\n", type);
		return ret;
	}

	return 0;
}

static uint32_t g2d_post_requests(struct g2d_drvdata *g2d,
				  struct g2d_submit *submit)
{
	unsigned long flags;
	uint32_t fence;
	bool active;

	spin_lock_irqsave(&g2d->ready_lock, flags);

	fence = atomic_inc_return(&g2d->next_fence);
	submit->fence = fence;
	g2d->submitted_fence = fence;
	active = !list_empty(&g2d->submit_list);
	list_add_tail(&submit->list, &g2d->submit_list);

	spin_unlock_irqrestore(&g2d->ready_lock, flags);

	if (active)
		return fence;

	dev_dbg_events(g2d->dev, "%s:%d waking up ready/idle_wq\n",
			__func__, __LINE__);

	pm_runtime_get_sync(g2d->dev);
	clk_enable(g2d->clock);
	g2d_handle_event(g2d);

	return fence;
}

static struct g2d_context *g2d_get_context(struct g2d_priv *priv, int id)
{
	struct g2d_drvdata *g2d = priv->g2d;
	struct g2d_context *ctx;

	ctx = idr_find(&priv->pipes_idr, id);
	if (!ctx) {
		dev_err(g2d->dev, "invalid pipe ID %u\n", id);
		ctx = ERR_PTR(-ENOENT);
	}

	return ctx;
}

int s3c6410_g2d_submit(struct drm_device *dev, void *data,
		       struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g2d_priv *priv = file_priv->s3c6410_g2d_priv;
	struct drm_exynos_g3d_submit *drm_submit = data;
	struct g2d_drvdata *g2d = priv->g2d;
	struct exynos_drm_gem_obj *gem;
	struct g2d_submit *submit;
	struct g2d_context *ctx;
	uint32_t offset, length;
	uint32_t *req, *end;
	int ret;

	offset = drm_submit->offset;
	length = drm_submit->length;

	dev_dbg_fops(g2d->dev,
			"%s: handle = %08x, offset = %08x, length = %08x\n",
			__func__, drm_submit->handle, offset,
			length);

	down_read(&priv->pipes_idr_sem);

	ctx = g2d_get_context(priv, drm_submit->pipe);
	if (IS_ERR(ctx)) {
		ret = PTR_ERR(ctx);
		goto err_unlock_idr;
	}

	mutex_lock(&ctx->submit_mutex);

	gem = exynos_drm_gem_lookup(dev, file, drm_submit->handle);
	if (!gem) {
		dev_err(g2d->dev, "failed to lookup submit buffer\n");
		ret = -EINVAL;
		goto err_unlock_ctx;
	}

	if (unlikely(!gem->buffer->kvaddr)) {
		dev_err(g2d->dev, "submit buffers need to have kernel mapping\n");
		ret = -EINVAL;
		goto err_gem_unref;
	}

	if (unlikely(offset + length > gem->size)) {
		dev_err(g2d->dev, "submit data bigger than backing buffer\n");
		ret = -EINVAL;
		goto err_gem_unref;
	}

	if (unlikely(offset % 4 || length % 4)) {
		dev_err(g2d->dev, "submit data incorrectly aligned\n");
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
	idr_init(&submit->gem_idr);

	req = submit->words;
	end = submit->words + submit->num_words;

	while (req < end) {
		if (unlikely((req_data(req) + req_length(req)) > end)) {
			dev_err(g2d->dev, "submit data truncated at 0x%x\n",
				req - submit->words);
			ret = -EINVAL;
			goto err_free;
		}

		ret = g2d_validate_request(g2d, submit, req);
		if (ret < 0)
			goto err_free;

		req = req_data_wr(req) + req_length(req);
	}

	kref_get(&ctx->ref);
	drm_submit->fence = g2d_post_requests(g2d, submit);
	drm_gem_object_unreference_unlocked(&gem->base);

	mutex_unlock(&ctx->submit_mutex);
	up_read(&priv->pipes_idr_sem);

	return 0;

err_free:
	g2d_release_submit(submit);
err_gem_unref:
	drm_gem_object_unreference_unlocked(&gem->base);
err_unlock_ctx:
	mutex_unlock(&ctx->submit_mutex);
err_unlock_idr:
	up_read(&priv->pipes_idr_sem);

	return ret;
}

int s3c6410_g2d_wait(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g2d_priv *priv = file_priv->s3c6410_g2d_priv;
	struct g2d_drvdata *g2d = priv->g2d;
	struct drm_exynos_g3d_wait *wait = data;

	if (wait->timeout.tv_sec >= 0) {
		struct timespec ts;

		ts.tv_sec = wait->timeout.tv_sec;
		ts.tv_nsec = wait->timeout.tv_nsec;

		return g2d_wait_interruptible_timeout(g2d, wait->fence, &ts);
	}

	return g2d_wait_interruptible_timeout(g2d, wait->fence, NULL);
}

int s3c6410_g2d_create_pipe(struct drm_device *drm_dev, void *data,
			    struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g2d_priv *priv = file_priv->s3c6410_g2d_priv;
	struct drm_exynos_g3d_pipe *pipe = data;
	struct g2d_drvdata *g2d = priv->g2d;
	struct g2d_context *ctx;
	int ret;

	down_write(&priv->pipes_idr_sem);

	ctx = kzalloc(sizeof(struct g2d_context), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto err_unlock;
	}

	dev_dbg_ctx(g2d->dev, "%s(ctx = %p)\n", __func__, ctx);

	ctx->drm_dev = drm_dev;
	ctx->g2d = g2d;
	ctx->file = file;

	kref_init(&ctx->ref);
	mutex_init(&ctx->submit_mutex);

	memcpy(ctx->register_masks, g2d_register_masks_def,
						sizeof(ctx->register_masks));

	ret = idr_alloc(&priv->pipes_idr, ctx, 1, 0, GFP_KERNEL);
	if (ret < 0) {
		dev_err(g2d->dev, "failed to allocate pipe ID (%d)\n", ret);
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

static void g2d_release_context(struct kref *kref)
{
	struct g2d_context *ctx = container_of(kref, struct g2d_context, ref);
	int i;

	dev_dbg_ctx(ctx->g2d->dev, "%s(ctx = %p)\n", __func__, ctx);

	for (i = 0; i < G2D_NUM_SURFACES; ++i)
		if (ctx->surface[i].obj)
			drm_gem_object_unreference_unlocked(
						&ctx->surface[i].obj->base);

	kfree(ctx);
}

static void g2d_destroy_context(struct g2d_context *ctx)
{
	dev_dbg_ctx(ctx->g2d->dev, "%s(ctx = %p)\n", __func__, ctx);

	ctx->aborted = true;
	kref_put(&ctx->ref, g2d_release_context);
}

int s3c6410_g2d_destroy_pipe(struct drm_device *drm_dev, void *data,
			     struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g2d_priv *priv = file_priv->s3c6410_g2d_priv;
	struct drm_exynos_g3d_pipe *pipe = data;
	struct g2d_context *ctx;

	down_write(&priv->pipes_idr_sem);

	ctx = g2d_get_context(priv, pipe->pipe);
	if (!IS_ERR(ctx)) {
		idr_remove(&priv->pipes_idr, pipe->pipe);
		g2d_destroy_context(ctx);
	}

	up_write(&priv->pipes_idr_sem);

	return 0;
}

/*
 * DRM subdriver
 */
static int g2d_open(struct drm_device *drm_dev, struct device *dev,
		    struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g2d_drvdata *g2d = dev_get_drvdata(dev);
	struct g2d_priv *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	idr_init(&priv->pipes_idr);
	priv->g2d = g2d;
	init_rwsem(&priv->pipes_idr_sem);

	/* Set private data */
	file_priv->s3c6410_g2d_priv = priv;

	dev_dbg_fops(dev, "device opened\n");

	return 0;
}

static int g2d_pipes_idr_cleanup(int id, void *p, void *data)
{
	struct g2d_context *ctx = p;

	g2d_destroy_context(ctx);

	return 0;
}

static void g2d_close(struct drm_device *drm_dev, struct device *dev,
		      struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct g2d_priv *priv = file_priv->s3c6410_g2d_priv;
	struct g2d_drvdata *g2d = priv->g2d;

	idr_for_each(&priv->pipes_idr, g2d_pipes_idr_cleanup, NULL);
	idr_destroy(&priv->pipes_idr);
	kfree(priv);

	dev_dbg_fops(g2d->dev, "device released\n");
}

/*
 * Platform driver
 */
static int g2d_probe(struct platform_device *pdev)
{
	struct exynos_drm_subdrv *subdrv;
	struct g2d_drvdata *g2d;
	struct resource *res;
	int ret;

	g2d = devm_kzalloc(&pdev->dev, sizeof(*g2d), GFP_KERNEL);
	if (!g2d)
		return -ENOMEM;

	/* get device clock */
	g2d->clock = devm_clk_get(&pdev->dev, "bus-clk");
	if (IS_ERR(g2d->clock)) {
		dev_err(&pdev->dev, "failed to find g2d clock source\n");
		return PTR_ERR(g2d->clock);
	}
	clk_prepare(g2d->clock);

	/* map mem resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	g2d->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(g2d->base))
		return PTR_ERR(g2d->base);

	/* get the IRQ */
	g2d->irq = platform_get_irq(pdev, 0);
	if (g2d->irq < 0) {
		dev_err(&pdev->dev,
			"failed to get irq resource (%d).\n", g2d->irq);
		return g2d->irq;
	}

	/* request the IRQ */
	ret = devm_request_irq(&pdev->dev, g2d->irq,
					g2d_handle_irq, 0, pdev->name, g2d);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed (%d).\n", ret);
		return ret;
	}

	g2d->dev = &pdev->dev;
	spin_lock_init(&g2d->ready_lock);
	INIT_LIST_HEAD(&g2d->submit_list);
	init_waitqueue_head(&g2d->fence_wq);
	atomic_set(&g2d->next_fence, 0);
	setup_timer(&g2d->watchdog_timer, g2d_watchdog_timer,
			(unsigned long)g2d);
	INIT_WORK(&g2d->retire_work, g2d_retire_work_func);

	platform_set_drvdata(pdev, g2d);

	pm_runtime_set_autosuspend_delay(&pdev->dev, G2D_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

#ifdef CONFIG_DRM_EXYNOS_S3C6410_G2D_ALWAYS_ON
	pm_runtime_get_sync(&pdev->dev);
	clk_prepare_enable(g2d->clock);
#endif

	subdrv = &g2d->subdrv;
	subdrv->dev = &pdev->dev;
	subdrv->open = g2d_open;
	subdrv->close = g2d_close;

	ret = exynos_drm_subdrv_register(subdrv);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register drm g2d device\n");
		goto err_subdrv_register;
	}

	return 0;

err_subdrv_register:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int g2d_remove(struct platform_device *pdev)
{
	struct g2d_drvdata *g2d = platform_get_drvdata(pdev);

	exynos_drm_subdrv_unregister(&g2d->subdrv);

#ifdef CONFIG_DRM_EXYNOS_S3C6410_G2D_ALWAYS_ON
	clk_disable_unprepare(g2d->clock);
	pm_runtime_put_sync(&pdev->dev);
#endif

	pm_runtime_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	clk_unprepare(g2d->clock);

	return 0;
}

#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM_RUNTIME)
static int g2d_runtime_resume(struct device *dev)
{
	struct g2d_drvdata *g2d = dev_get_drvdata(dev);

	clk_enable(g2d->clock);
	g2d_initialize(g2d);
	clk_disable(g2d->clock);

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int g2d_resume(struct device *dev)
{
	if (!pm_runtime_suspended(dev))
		return g2d_runtime_resume(dev);

	return 0;
}
#endif

static const struct dev_pm_ops g2d_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(NULL, g2d_resume)
	SET_RUNTIME_PM_OPS(NULL, g2d_runtime_resume, NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id g2d_of_matches[] = {
	{ .compatible = "samsung,s3c6410-g2d", },
	{ /* Sentinel */ }
};
#endif

struct platform_driver s3c6410_g2d_driver = {
	.probe	= g2d_probe,
	.remove	= g2d_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "s3c6410-g2d",
		.of_match_table = of_match_ptr(g2d_of_matches),
		.pm	= &g2d_pm_ops,
	},
};
