/*******************************************************************************
 * utest-vin.h
 *
 * ADAS unit-test. VIN LVDS cameras backend
 *
 * Copyright (c) 2016 Cogent Embedded Inc. ALL RIGHTS RESERVED.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#ifndef __UTEST_VIN_H
#define __UTEST_VIN_H

/*******************************************************************************
 * Types definitions
 ******************************************************************************/

typedef struct vin_data     vin_data_t;

/*******************************************************************************
 * VIN meta data
 ******************************************************************************/

/* ...metadata structure */
typedef struct vin_meta
{
    GstMeta             meta;

    /* ...camera identifier */
    int                 camera_id;

    /* ...buffer index in the camera pool */
    int                 index;

    /* ...buffer dimensions */
    int                 width, height;

    /* ...buffer format (V4L2) */
    u32                 format;

    /* ...private data pointer */
    void               *priv;

    /* ...buffer DMA file-descriptors (per-plane) */
    int                 dmafd[3];

    /* ...DMA offsets (per-plane) */
    u32                 offset[3];

    /* ...buffers strides (per-plane) */
    u32                 stride[3];

    /* ...buffer length */
    int                 length;

}   vin_meta_t;

/* ...metadata API type accessor */
extern GType vin_meta_api_get_type(void);
#define VIN_META_API_TYPE               (vin_meta_api_get_type())

/* ...metadata information handle accessor */
extern const GstMetaInfo *vin_meta_get_info(void);
#define VIN_META_INFO                   (vin_meta_get_info())

/* ...get access to the buffer metadata */
#define gst_buffer_get_vin_meta(b)      \
    ((vin_meta_t *)gst_buffer_get_meta((b), VIN_META_API_TYPE))

/* ...attach metadata to the buffer */
#define gst_buffer_add_vin_meta(b)    \
    ((vin_meta_t *)gst_buffer_add_meta((b), VIN_META_INFO, NULL))

/*******************************************************************************
 * Public API
 ******************************************************************************/

extern vin_data_t * vin_init(char **devname, int num, camera_callback_t *cb, void *cdata);

extern int vin_device_configure(vin_data_t *vin, int i, int w, int h, u32 fmt, int size);
extern int vin_device_init(vin_data_t *vin, int i, int w, int h, u32 fmt, int size);
extern int vin_device_snapshot(vin_data_t *vin, int i, void *data);

extern int vin_start(vin_data_t *vin);

#endif  /* __UTEST_VIN_H */

