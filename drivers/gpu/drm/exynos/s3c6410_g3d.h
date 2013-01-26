/*
 * OpenFIMG DRM driver
 *
 * Copyright 2013 Tomasz Figa <tomasz.figa@gmail.com>
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

#ifndef __OPENFIMG_G3D_H_
#define __OPENFIMG_G3D_H_

extern int s3c6410_g3d_submit(struct drm_device *dev, void *data,
				    struct drm_file *file_priv);
extern int s3c6410_g3d_wait(struct drm_device *dev, void *data,
					struct drm_file *file_priv);

#endif
