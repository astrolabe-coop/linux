/*
 * Copyright (c) 2016 Intel Corporation
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#ifndef __DRM_BRIDGE_H__
#define __DRM_BRIDGE_H__

// #include <linux/ctype.h>
// #include <linux/list.h>
// #include <linux/mutex.h>

// #include <drm/drm_atomic.h>
// #include <drm/drm_encoder.h>
// #include <drm/drm_mode_object.h>
// #include <drm/drm_modes.h>

/**
 * enum drm_bridge_attach_flags - Flags for &drm_bridge_funcs.attach
 */
enum drm_bridge_attach_flags {
	/**
	 * @DRM_BRIDGE_ATTACH_NO_CONNECTOR: When this flag is set the bridge
	 * shall not create a drm_connector.
	 */
	DRM_BRIDGE_ATTACH_NO_CONNECTOR = BIT(0),
};
#endif
