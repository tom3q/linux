/* exynos_drm_buf.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Author: Inki Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <drm/drmP.h>
#include <drm/exynos_drm.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_gem.h"
#include "exynos_drm_buf.h"

static int lowlevel_buffer_allocate(struct drm_device *dev,
		unsigned int flags, struct exynos_drm_gem_buf *buf)
{
	int ret = 0;
	enum dma_attr attr;

	if (buf->dma_addr) {
		DRM_DEBUG_KMS("already allocated.\n");
		return 0;
	}

	init_dma_attrs(&buf->dma_attrs);

	/*
	 * if EXYNOS_BO_CONTIG, fully physically contiguous memory
	 * region will be allocated else physically contiguous
	 * as possible.
	 */
	if (!(flags & EXYNOS_BO_NONCONTIG))
		dma_set_attr(DMA_ATTR_FORCE_CONTIGUOUS, &buf->dma_attrs);

	/*
	 * if EXYNOS_BO_WC or EXYNOS_BO_NONCACHABLE, writecombine mapping
	 * else cachable mapping.
	 */
	if (flags & EXYNOS_BO_WC || !(flags & EXYNOS_BO_CACHABLE))
		attr = DMA_ATTR_WRITE_COMBINE;
	else
		attr = DMA_ATTR_NON_CONSISTENT;

	dma_set_attr(attr, &buf->dma_attrs);

	if (flags & EXYNOS_BO_UNMAPPED)
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &buf->dma_attrs);

	buf->cookie = dma_alloc_attrs(dev->dev, buf->size,
				&buf->dma_addr, GFP_KERNEL,
				&buf->dma_attrs);
	if (!buf->cookie) {
		DRM_ERROR("failed to allocate buffer.\n");
		return -ENOMEM;
	}
	if (dma_get_attr(DMA_ATTR_NO_KERNEL_MAPPING, &buf->dma_attrs))
		buf->kvaddr = NULL;
	else
		buf->kvaddr = buf->cookie;

	buf->sgt = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (IS_ERR(buf->sgt)) {
		DRM_ERROR("failed to allocate sg table.\n");
		ret = PTR_ERR(buf->sgt);
		goto err_free_attrs;
	}

	ret = dma_get_sgtable(dev->dev, buf->sgt, buf->cookie,
						buf->dma_addr, buf->size);
	if (ret) {
		DRM_ERROR("failed to get sg table.\n");
		goto err_free_sgt;
	}

	DRM_DEBUG_KMS("vaddr(0x%lx), dma_addr(0x%lx), size(0x%lx)\n",
			(unsigned long)buf->kvaddr,
			(unsigned long)buf->dma_addr,
			buf->size);

	return ret;

err_free_sgt:
	kfree(buf->sgt);
	buf->sgt = NULL;
err_free_attrs:
	dma_free_attrs(dev->dev, buf->size, buf->cookie,
			(dma_addr_t)buf->dma_addr, &buf->dma_attrs);
	buf->dma_addr = (dma_addr_t)NULL;

	return ret;
}

static void lowlevel_buffer_deallocate(struct drm_device *dev,
		unsigned int flags, struct exynos_drm_gem_buf *buf)
{
	if (!buf->dma_addr) {
		DRM_DEBUG_KMS("dma_addr is invalid.\n");
		return;
	}

	DRM_DEBUG_KMS("vaddr(0x%lx), dma_addr(0x%lx), size(0x%lx)\n",
			(unsigned long)buf->kvaddr,
			(unsigned long)buf->dma_addr,
			buf->size);

	sg_free_table(buf->sgt);

	kfree(buf->sgt);
	buf->sgt = NULL;

	dma_free_attrs(dev->dev, buf->size, buf->cookie,
				(dma_addr_t)buf->dma_addr, &buf->dma_attrs);

	buf->dma_addr = (dma_addr_t)NULL;
}

struct exynos_drm_gem_buf *exynos_drm_init_buf(struct drm_device *dev,
						unsigned int size)
{
	struct exynos_drm_gem_buf *buffer;

	DRM_DEBUG_KMS("desired size = 0x%x\n", size);

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return NULL;

	buffer->size = size;
	return buffer;
}

void exynos_drm_fini_buf(struct drm_device *dev,
				struct exynos_drm_gem_buf *buffer)
{
	kfree(buffer);
	buffer = NULL;
}

int exynos_drm_alloc_buf(struct drm_device *dev,
		struct exynos_drm_gem_buf *buf, unsigned int flags)
{

	/*
	 * allocate memory region and set the memory information
	 * to vaddr and dma_addr of a buffer object.
	 */
	if (lowlevel_buffer_allocate(dev, flags, buf) < 0)
		return -ENOMEM;

	return 0;
}

void exynos_drm_free_buf(struct drm_device *dev,
		unsigned int flags, struct exynos_drm_gem_buf *buffer)
{

	lowlevel_buffer_deallocate(dev, flags, buffer);
}
