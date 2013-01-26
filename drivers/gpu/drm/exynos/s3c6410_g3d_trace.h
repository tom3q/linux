/*
 * OpenFIMG DRM driver tracepoints
 *
 * Copyright 2014 Tomasz Figa <tomasz.figa@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#if !defined(__OPENFIMG_G3D_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define __OPENFIMG_G3D_TRACE_H_

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM g3d
#define TRACE_SYSTEM_STRING __stringify(TRACE_SYSTEM)
#define TRACE_INCLUDE_FILE s3c6410_g3d_trace


TRACE_EVENT(g3d_draw_request,
		TP_PROTO(uint32_t draw),

		TP_ARGS(draw),

		TP_STRUCT__entry(
				__field(uint32_t, draw)
		),

		TP_fast_assign(
				__entry->draw = draw;
		),

		TP_printk("draw=%u", __entry->draw)
);

TRACE_EVENT(g3d_draw_complete,
		TP_PROTO(uint32_t draw),

		TP_ARGS(draw),

		TP_STRUCT__entry(
				__field(uint32_t, draw)
		),

		TP_fast_assign(
				__entry->draw = draw;
		),

		TP_printk("draw=%u", __entry->draw)
);

TRACE_EVENT(g3d_fence_wait_request,
		TP_PROTO(uint32_t fence),

		TP_ARGS(fence),

		TP_STRUCT__entry(
				__field(uint32_t, fence)
		),

		TP_fast_assign(
				__entry->fence = fence;
		),

		TP_printk("fence=%u", __entry->fence)
);

TRACE_EVENT(g3d_fence_wait_complete,
		TP_PROTO(uint32_t fence, int ret),

		TP_ARGS(fence, ret),

		TP_STRUCT__entry(
				__field(uint32_t, fence)
				__field(int, ret)
		),

		TP_fast_assign(
				__entry->fence = fence;
				__entry->ret = ret;
		),

		TP_printk("fence=%u, ret=%d", __entry->fence, __entry->ret)
);

TRACE_EVENT(g3d_gpu_request,
		TP_PROTO(uint32_t fence),

		TP_ARGS(fence),

		TP_STRUCT__entry(
				__field(uint32_t, fence)
		),

		TP_fast_assign(
				__entry->fence = fence;
		),

		TP_printk("fence=%u", __entry->fence)
);

TRACE_EVENT(g3d_gpu_complete,
		TP_PROTO(uint32_t fence),

		TP_ARGS(fence),

		TP_STRUCT__entry(
				__field(uint32_t, fence)
		),

		TP_fast_assign(
				__entry->fence = fence;
		),

		TP_printk("fence=%u", __entry->fence)
);

#endif

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#include <trace/define_trace.h>
