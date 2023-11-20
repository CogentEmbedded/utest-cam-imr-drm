/*******************************************************************************
 * utest-app.c
 *
 * IMR unit test application
 *
 * Copyright (c) 2017-2018 Cogent Embedded Inc. ALL RIGHTS RESERVED.
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

#define MODULE_TAG                      APP

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "utest-common.h"
#include "utest-app.h"
#include "utest-vsink.h"
#include "utest-vin.h"
#include "utest-imr.h"
#include "utest-mesh-generators.h"
#include "utest-common-math.h"
#include <linux/videodev2.h>
#include "imr-v4l2-api.h"

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 1);

/*******************************************************************************
 * Local constants definitions
 ******************************************************************************/

int cameras_number = CAMERAS_NUMBER;
static int split_camera_frames = 0;
static int imr_number = IMR_NUMBER;

/* ...maximal number of triangles to have for a single camera */
#define IMR_CFG_CAMERA_SIZE             10000

/* ...output buffer pool size */
#define VSP_POOL_SIZE                   3

#define MAX_IMR_SUPPORT_RESOLUTION_WIDTH 2048
#define BLOCK_SIZE 16

/*******************************************************************************
 * Local types definitions
 ******************************************************************************/

typedef struct
{
    int32_t x_start;
    int32_t y_start;
    int32_t width;
    int32_t height;
} layout_config_t;

static layout_config_t layout_config_list[CAMERAS_NUMBER];

struct app_data
{
    /* ...main window handle */
    window_data_t      *window;

    /* ...input stream dimensions */
    int                 width, height;

    /* ...miscellaneous control flags */
    u32                 flags;

    /* ...input (camera) buffers readiness flag */
    u32                 input_ready;

    /* ...pending input buffers (waiting for IMR processing start) */
    GQueue              input[CAMERAS_NUMBER];

    /* ...output (IMR) buffers readiness flag */
    u32                 output_ready;

    /* ...ready output buffers (waiting for rendering) */
    GQueue              output[CAMERAS_NUMBER];

    /* ...output buffers */
    GstBuffer          *buf[2][CAMERAS_NUMBER];

    /* ...current bank */
    int                 bank;

    /* ...data access lock */
    pthread_mutex_t     lock;

    /* ...VIN handle */
    vin_data_t         *vin;

    /* ...IMR engine handle */
    imr_data_t         *imr;

    /* ...raw memory buffers */
    vsp_mem_t         **raw_plane;

    /* ...buffers memory */
    vsp_mem_t          *camera_plane[CAMERAS_NUMBER][VSP_POOL_SIZE];

    /* divided input buffer memory */
    vsp_mem_t              **divided_input;
    GstBuffer              **divided_buffer;
    uint32_t index;

    /* ...IMR configuration */
    imr_cfg_t          *cfg[CAMERAS_NUMBER];

    /* ...frame number */
    u32                 frame_num;

    int                 dump_png;

};

/*******************************************************************************
 * Operation control flags
 ******************************************************************************/

/* ...output debugging info */
#define APP_FLAG_DEBUG                  (1 << 0)

/* ...renderer flushing condition */
#define APP_FLAG_EOS                    (1 << 1)

/*******************************************************************************
 * Interface exposed to the camera backend
 ******************************************************************************/

/* ...deallocate texture data */
static void __destroy_vin_buffer(gpointer data, GstMiniObject *obj)
{
    GstBuffer      *buffer = (GstBuffer *)obj;
    vin_meta_t     *meta = gst_buffer_get_vin_meta(buffer);

    TRACE(DEBUG, _b("destroy texture referenced by meta: %p:%p"), meta, meta->priv);

    /* ...unexport buffer? - tbd */
}

/* ...input buffer allocation */
static int app_input_alloc(void *data, int i, GstBuffer *buffer)
{
    app_data_t     *app = data;
    vin_meta_t     *meta = gst_buffer_get_vin_meta(buffer);
    int             w = meta->width, h = meta->height;
    int             j = meta->index;

    /* ...make sure camera index is valid */
    CHK_ERR((u32)i < (u32)cameras_number, -EINVAL);

    /* ...make sure camera buffer is valid */
    CHK_ERR((u32)j < (u32)__vin_buffers_num, -EINVAL);

    /* ...assign memory buffer */
    meta->priv = app->raw_plane[cameras_number * j + i];

    /* ...create DMA buffers for a memory chunk */
    CHK_API(vsp_buffer_export(meta->priv, CHECK_MULTIPLICITY(w, 255), h, meta->format, meta->dmafd, meta->offset, meta->stride));

    TRACE(DEBUG, _b("dmafd: %d/%d/%d, offset: %d/%d/%d, stride: %u/%u/%u"),
          meta->dmafd[0], meta->dmafd[1], meta->dmafd[2],
          meta->offset[0], meta->offset[1], meta->offset[2],
          meta->stride[0], meta->stride[1], meta->stride[2]);

    /* ...add custom destructor to the buffer */
    gst_mini_object_weak_ref(GST_MINI_OBJECT(buffer), __destroy_vin_buffer, app);

    /* ...do not need to do anything with the buffer allocation? */
    TRACE(INFO, _b("camera-%d: input buffer-%d allocated (%p)"), i, j, buffer);

    return 0;
}

static inline int store_uyvy(const char *fname, int width, int height, int format, void *data)
{
    FILE   *f;

    CHK_ERR(f = fopen(fname, "wb"), -errno);

    if (fwrite(data, width * height * 2, 1, f) != 1)
    {
        TRACE(ERROR, _x("failed to write image: %m"));
    }

    fclose(f);

    return 0;
}


/* ...input buffer constructor */
static GstBuffer * __input_buffer_create(app_data_t *app, int w, int h, int stride, u32 fmt, vsp_mem_t *mem, int32_t offset)
{
    GstBuffer      *buffer;
    vin_meta_t   *vmeta;

    /* ...allocate single input buffer */
    CHK_ERR(buffer = gst_buffer_new(), NULL);

    /* ...add vsink meta-data (needed by IMR engine) */
    CHK_ERR(vmeta = gst_buffer_add_vin_meta(buffer), NULL);
    vmeta->priv = vsp_mem_ptr(mem);
    vmeta->format = fmt;
    vmeta->width = w;
    vmeta->height = h;
    vmeta->offset[0] = offset;
    vmeta->stride[0] = stride;
    GST_META_FLAG_SET(vmeta, GST_META_FLAG_POOLED);
    vsp_dmabuf_t * t = vsp_dmabuf_export(mem, offset, stride*h * 2);
    vmeta->dmafd[0] = vsp_dmabuf_fd(t);

    /* ...set buffer custom data */
    buffer->pool = (void *)app;

    return buffer;
}

/* ...process new input buffer submitted from camera */
static int app_input_process(void *data, int i, GstBuffer *buffer)
{
    app_data_t     *app = data;

    TRACE(DEBUG, _b("camera-%d: input buffer received"), i);

    /* ...make sure camera index is valid */
    CHK_ERR((u32)i < (u32)cameras_number, -EINVAL);

    /* ...lock access to the internal queue */
    pthread_mutex_lock(&app->lock);

    /* ...collect buffers in a pending input queue */
    g_queue_push_tail(&app->input[i], gst_buffer_ref(buffer));

    /* ...submit a job only when all buffers have been collected */
    if ((app->input_ready &= ~(1 << i)) == 0)
    {
        /* ...collect buffers from input queue (it is a common function) */
        for (i = 0; i < cameras_number; i++)
        {
            GstBuffer  *buf;

            /* ...get buffer from the head of input queue */
            buf = g_queue_pop_head(&app->input[i]);

            if (app->dump_png && --app->dump_png == 0)
            {
                vin_device_snapshot(app->vin, i, buf);
            }

            /* ...submit to corresponding IMR engine */
            if (split_camera_frames == 1)
            {
                vin_meta_t *meta = gst_buffer_get_vin_meta(buf);
                app->index = i *__vin_buffers_num + meta->index;
                CHK_API(imr_engine_push_buffer(app->imr, (i * 2) + 0, app->divided_buffer[app->index * 2 + 0]));
                CHK_API(imr_engine_push_buffer(app->imr, (i * 2) + 1, app->divided_buffer[app->index * 2 + 1]));
                vin_meta_t *meta_div = gst_buffer_get_vin_meta(app->divided_buffer[app->index * 2 + 0]);
                meta_div->priv = buf;
                meta_div = gst_buffer_get_vin_meta(app->divided_buffer[app->index * 2 + 1]);
                meta_div->priv = buf;
            }
            else
            {
                CHK_API(imr_engine_push_buffer(app->imr, i, buf));
                /* ...drop the reference to input buffer */
                gst_buffer_unref(buf);
            }


            /* ...update readiness flag */
            (g_queue_is_empty(&app->input[i]) ? app->input_ready |= (1 << i) : 0);
        }
    }

    /* ...release application lock */
    pthread_mutex_unlock(&app->lock);

    return 0;
}

/* ...callbacks for camera back-end */
static camera_callback_t camera_cb = {
    .allocate = app_input_alloc,
    .process = app_input_process,
};

/*******************************************************************************
 * Distortion correction engine interface
 ******************************************************************************/

/* ...deallocate texture data */
static void __destroy_imr_buffer(gpointer data, GstMiniObject *obj)
{
    GstBuffer      *buffer = (GstBuffer *)obj;
    imr_meta_t     *meta = gst_buffer_get_imr_meta(buffer);

    /* ...destroy texture data */
    (meta->priv2 ? texture_destroy(meta->priv2) : 0);

    TRACE(DEBUG, _b("destroy IMR buffer <%d:%d>"), meta->id, meta->index);
}

/* ...buffer allocation callback */
static int imr_buffer_allocate(void *cdata, int i, GstBuffer *buffer)
{
    app_data_t     *app = cdata;
    imr_meta_t     *meta = gst_buffer_get_imr_meta(buffer);
    int             w = meta->width, h = meta->height, format = meta->format;
    int             j = meta->index;
    int             dmafd[GST_VIDEO_MAX_PLANES];
    u32             offset[GST_VIDEO_MAX_PLANES];
    u32             stride[GST_VIDEO_MAX_PLANES];

    /* ...verify engine id */
    CHK_ERR((u32)i < (u32)imr_number, -(errno = EBADFD));

    /* ...verify individual buffer index */
    CHK_ERR((u32)j < (u32)VSP_POOL_SIZE, -(errno = EBADFD));


    if (split_camera_frames == 0)
    {
        /* ...save pointer to the memory buffer */
        meta->priv = app->camera_plane[i][j];
        /* ...create DMA buffers for a memory chunk */
        CHK_API(vsp_buffer_export(meta->priv, w, h, __pixfmt_gst_to_v4l2(format), dmafd, offset, stride));
    }
    else
    {
        /* ...save pointer to the memory buffer */
        meta->priv = app->camera_plane[0][j];

        CHK_API(vsp_buffer_export(meta->priv, w, h, __pixfmt_gst_to_v4l2(format), dmafd, offset, stride));
    }
    /* ...create a texture for visualization */
    CHK_ERR(meta->priv2 = texture_create(w, h, format, dmafd, offset, stride), -1);

    /* ...assign camera buffer DMA-fd (tbd - single plane only?)  */
    memcpy(meta->buf->dmafd, dmafd, sizeof(dmafd));

    /* ...add custom buffer destructor */
    gst_mini_object_weak_ref(GST_MINI_OBJECT(buffer), __destroy_imr_buffer, app);

    TRACE(INFO, _b("imr-buffer <%d:%d> allocated: %u*%u (format=%u, dmafd=%d)"), i, j, w, h, meta->format, meta->buf->dmafd[0]);

    return 0;
}

/* ...buffer preparation callback */
static int imr_buffer_prepare(void *cdata, int i, GstBuffer *buffer)
{
    imr_meta_t     *meta = gst_buffer_get_imr_meta(buffer);
    int             j = meta->index;

    /* ...sanity check */
    CHK_ERR((u32)i < (u32)imr_number, -(errno = EBADFD));
    CHK_ERR((u32)j < (u32)VSP_POOL_SIZE, -(errno = EBADFD));

    /* ...no special preparation */
    return 0;
}

/* ...output buffer callback */
static int imr_buffer_process(void *cdata, int i, GstBuffer *buffer)
{
    app_data_t     *app = cdata;
    imr_meta_t     *meta = gst_buffer_get_imr_meta(buffer);
    int             j = meta->index;

    TRACE(DEBUG, _b("imr-buffer <%d:%d> ready: %p (refcount=%d)"), i, j, buffer, GST_MINI_OBJECT_REFCOUNT(buffer));

    /* ...lock data access */
    pthread_mutex_lock(&app->lock);

    if (app->dump_png) {

        imr_meta_t *meta = gst_buffer_get_imr_meta(buffer);

        store_uyvy("output.nv16", meta->width, meta->height, meta->format, vsp_mem_ptr(meta->priv));

        app->dump_png = 0;
    }

    /* ...submit buffer to output queue */
    g_queue_push_tail(&app->output[i], gst_buffer_ref(buffer));

    /* ...notify renderer only when all buffers have been collected */
    if ((app->output_ready &= ~(1 << i)) == 0)
    {
        if (split_camera_frames == 1)
        {
            vin_meta_t *meta_div = gst_buffer_get_vin_meta(app->divided_buffer[j * 2]);
            gst_buffer_unref(meta_div->priv);
        }

        window_schedule_redraw(app->window);
    }

    /* ...release data access lock */
    pthread_mutex_unlock(&app->lock);

    return 0;
}

/* ...engine callback structure */
static camera_callback_t   imr_cb = {
    .allocate = imr_buffer_allocate,
    .prepare = imr_buffer_prepare,
    .process = imr_buffer_process,
};

/*******************************************************************************
 * Application window callbacks
 ******************************************************************************/

/* ...redraw main application window */
static void app_redraw(display_data_t *display, void *data)
{
    app_data_t     *app = data;
    window_data_t  *window = app->window;
    int             W = window_get_width(window), H = window_get_height(window);

    /* ...lock internal data */
    pthread_mutex_lock(&app->lock);

    /* ...retrieve pending buffers from rendering queue */
    while (app->output_ready == 0)
    {
        float           fps = window_frame_rate_update(window);
        int             i;
        GstBuffer     **buf = app->buf[app->bank];

        /* ...get buffers from output queue */
        for (i = 0; i < imr_number; i++)
        {
            buf[i] = g_queue_pop_head(&app->output[i]);

            /* ...clear readiness flag if queue gets empty */
            (g_queue_is_empty(&app->output[i]) ? app->output_ready |= (1 << i) : 0);
        }

        /* ...release the data access lock */
        pthread_mutex_unlock(&app->lock);

        TRACE(INFO, _b("redraw frame: %u (fps: %.2f)"), app->frame_num++, fps);

        /* ...set up the planes */
        for (i = 0; i < cameras_number; i++)
        {
            texture_data_t     *t = gst_buffer_get_imr_meta(buf[i])->priv2;
            texture_view_t      v;
            texture_crop_t      c;

            /* ...set view-port for a buffer */
            (cameras_number == 1 ?
                        texture_set_view(&v, 0 ,0, W, H) :
                        texture_set_view(&v, (i & 1 ? W / 2 : 0), (i & 2 ? H / 2 : 0), (i & 1 ? W : W / 2), (i & 2 ? H : H / 2)));

            /* ...set cropping parameters */
            (cameras_number == 1 ?
                        texture_set_crop(&c, 0, 0, W / 1, H / 1) :
                        texture_set_crop(&c, 0, 0, W / 2, H / 2));

            /* ...setup rendering plane */
            plane_setup(window, i, t, NULL, 0xFF, &v, &c, NULL, 0);
        }

        /* ...submit data to renderer */
        window_draw(window);

        /* ...check if we have something new */
        pthread_mutex_lock(&app->lock);

        /* ...switch to new bank */
        if ((buf = app->buf[app->bank ^= 1])[0] != NULL)
        {
            /* ...release old buffers (we need to keep last buffer) */
            for (i = 0; i < imr_number; i++)
            {
                imr_meta_t *meta = gst_buffer_get_imr_meta(buf[i]);

                if (0 && meta)
                {
                    vsp_mem_t  *mem = meta->priv;
                    memset(vsp_mem_ptr(mem), 0, vsp_mem_size(mem));
                }

                /* ...release reference to a buffer */
                gst_buffer_unref(buf[i]);
            }
        }
    }

    /* ...release processing lock */
    pthread_mutex_unlock(&app->lock);

    TRACE(DEBUG, _b("drawing complete.."));
}

/* ...initialize application processing context */
static int app_context_init(widget_data_t *widget, void *data)
{
    app_data_t     *app = data;
    window_data_t  *window = (window_data_t *)widget;
    int             w = window_get_width(window);
    int             h = window_get_height(window);
    int             fmt = __pixfmt_v4l2_to_gst(__vin_format);
    int             W = w / 2, H = h / 2;
    int             i;
    u32             c_type;

    (cameras_number == 1 ? (W = w / 1, H = h / 1) : 0);
    split_camera_frames = (__vin_width > MAX_IMR_SUPPORT_RESOLUTION_WIDTH) ? 1 : 0;

    /* ...allocate buffers */
    CHK_ERR(app->raw_plane = calloc(cameras_number, __vin_buffers_num * sizeof(vsp_mem_t *)), -1);

    /* ...allocate cacheable buffers for VIN */
    CHK_API(vsp_allocate_buffers(CHECK_MULTIPLICITY(__vin_width, 255), __vin_height + 1, __vin_format, app->raw_plane, cameras_number * __vin_buffers_num, 1*1));

    int camera_buffer_shift = 0;


    if (split_camera_frames == 1)
    {
        if (prepare_rectified_mesh == true)
        {
            camera_buffer_shift = 256;
        }

        CHK_ERR(app->divided_buffer = calloc(cameras_number, 2 * __vin_buffers_num * sizeof(GstBuffer *)), -1);

        for (i = 0; i < cameras_number * __vin_buffers_num; i++)
        {
            CHK_ERR(app->divided_buffer[i * 2 + 0] = __input_buffer_create(app, (__vin_width / 2) + camera_buffer_shift, __vin_height, __vin_width, __vin_format, app->raw_plane[i], 0), -1);
            CHK_ERR(app->divided_buffer[i * 2 + 1] = __input_buffer_create(app, (__vin_width / 2) + camera_buffer_shift, __vin_height, __vin_width, __vin_format, app->raw_plane[i], (__vin_width / 2) - camera_buffer_shift), -1);
        }
    }

    /* ...create VIN engine */
    CHK_ERR(app->vin = vin_init(vin_dev_name, cameras_number, &camera_cb, app), -1);

    /* ...create IMR engine */
    if (split_camera_frames == 1)
    {
        imr_number = cameras_number * 2;
        CHK_ERR(app->imr = imr_init(imr_dev_name, imr_number, &imr_cb, app), -1);
    }
    else
    {
        imr_number = cameras_number;
        CHK_ERR(app->imr = imr_init(imr_dev_name, imr_number, &imr_cb, app), -1);
    }

    /* ...allocate output buffers for IMR (non-cacheable) */
    CHK_API(vsp_allocate_buffers(W, H, __vin_format, &app->camera_plane[0][0], cameras_number * VSP_POOL_SIZE, 0));

    /* ...configure VINs */
    for (i = 0; i < cameras_number; i++)
    {
        /* ...configure VIN device */
        CHK_API(vin_device_configure(app->vin, i, __vin_width, __vin_height, __vin_format, __vin_buffers_num));
    }

    /* ...prepare IMR configuration descriptor for camera-planes */
    c_type = IMR_MAP_TME | IMR_MAP_BFE | 0*IMR_MAP_TCM;

    if (prepare_rectified_mesh == false)
    {
        c_type |= IMR_MAP_AUTODG;
    }

    c_type |= IMR_MAP_UVDPOR(DL_SRC_SUBSAMPLE) | (DL_DST_SUBSAMPLE ? IMR_MAP_DDP : 0);

    /* ...setup VINs */
    for (i = 0; i < cameras_number; i++)
    {
        /* ...setup VIN device */
        CHK_API(vin_device_init(app->vin, i, __vin_width, __vin_height, __vin_format, __vin_buffers_num));
    }

    /* ...setup VINs */
    if (split_camera_frames == 0)
    {
        /* ...setup VINs */
        for (i = 0; i < cameras_number; i++)
        {
            /* ...setup IMR engine to do a simple scaling */
            CHK_API(imr_setup(app->imr, i, __vin_width, __vin_height, W, H, __vin_width, fmt, fmt, VSP_POOL_SIZE));

            /* ...allocate engine configuration */
            CHK_ERR(app->cfg[i] = imr_cfg_create(c_type, IMR_CFG_CAMERA_SIZE * sizeof(dl_abs_t)), -1);

            /* ...setup camera configuration */
            if (prepare_rectified_mesh == false)
            {
                CHK_API(__undistorted_cfg_setup(app, camera_intrinsic_list, i, __vin_width, __vin_height, 0, 0, W, H, app->cfg[i]));
            }
            else
            {
                float hfov = 90.0f;
                float bottom_vertical_angle = ((__vin_height - camera_intrinsic_list[i].cy) / camera_intrinsic_list[i].fy)* 180 / M_PI;
                float upper_vertical_angle = (camera_intrinsic_list[i].cy / camera_intrinsic_list[i].fy)* 180 / M_PI;

                if ((upper_vertical_angle + bottom_vertical_angle) > (hfov / 2.0f))
                {
                    float aspect = (float)(layout_config_list[i].y_start + layout_config_list[i].height) / (float)(layout_config_list[i].x_start + layout_config_list[i].width);
                    bottom_vertical_angle = ((hfov * aspect) / 2.0f);
                    upper_vertical_angle = bottom_vertical_angle;
                }

                CHK_API(__distorted_cfg_setup(camera_intrinsic_list,
                                            i,
                                            __vin_width, __vin_height, 0, camera_buffer_shift,
                                            layout_config_list[i].x_start,
                                            layout_config_list[i].y_start,
                                            (layout_config_list[i].x_start + layout_config_list[i].width),
                                            (layout_config_list[i].y_start + layout_config_list[i].height),
                                            hfov, bottom_vertical_angle, upper_vertical_angle, BLOCK_SIZE,
                                            app->cfg[i],
                                            app->cfg[i])
                        );

            }

            /* ...apply configuration instantly */
            CHK_API(imr_cfg_apply(app->imr, i, app->cfg[i]));
        }
    }
    else
    {
        /* ...setup IMR engine to do a simple scaling */
        for (i = 0; i < imr_number; i++)
        {
            CHK_API(imr_setup(app->imr, i, (__vin_width / 2) + camera_buffer_shift, __vin_height, W, H, __vin_width, fmt, fmt, VSP_POOL_SIZE));
            /* ...allocate engine configuration */
            CHK_ERR(app->cfg[i] = imr_cfg_create(c_type, IMR_CFG_CAMERA_SIZE * sizeof(dl_abs_t)), -1);
        }

        if (prepare_rectified_mesh == false)
        {
            for (i = 0; i < cameras_number; i++)
            {
                /* ...setup camera configuration */
                CHK_API(__undistorted_cfg_setup(app,
                                            camera_intrinsic_list,
                                            i,
                                            __vin_width / 2, __vin_height,
                                            layout_config_list[i].x_start,
                                            layout_config_list[i].y_start,
                                            (layout_config_list[i].x_start + layout_config_list[i].width) / 2,
                                            (layout_config_list[i].y_start + layout_config_list[i].height),
                                            app->cfg[(i * 2) + 0])
                        );
                CHK_API(__undistorted_cfg_setup(app,
                                            camera_intrinsic_list,
                                            i,
                                            __vin_width / 2, __vin_height,
                                            layout_config_list[i].x_start + (layout_config_list[i].x_start + layout_config_list[i].width) / 2,
                                            layout_config_list[i].y_start,
                                            (layout_config_list[i].x_start + layout_config_list[i].width),
                                            (layout_config_list[i].y_start + layout_config_list[i].height),
                                            app->cfg[(i * 2) + 1])
                        );
            }
        }
        else
        {
            for (i = 0; i < cameras_number; i++)
            {
                float hfov = 90.0f;
                float bottom_vertical_angle = ((__vin_height - camera_intrinsic_list[i].cy) / camera_intrinsic_list[i].fy)* 180 / M_PI;
                float upper_vertical_angle = (camera_intrinsic_list[i].cy / camera_intrinsic_list[i].fy)* 180 / M_PI;
                /* ...setup camera configuration */
                CHK_API(__distorted_cfg_setup(camera_intrinsic_list,
                                            i,
                                            __vin_width, __vin_height, (__vin_width / 2), camera_buffer_shift,
                                            layout_config_list[i].x_start,
                                            layout_config_list[i].y_start,
                                            (layout_config_list[i].x_start + layout_config_list[i].width),
                                            (layout_config_list[i].y_start + layout_config_list[i].height),
                                            hfov, bottom_vertical_angle, upper_vertical_angle, BLOCK_SIZE,
                                            app->cfg[(i * 2) + 0],
                                            app->cfg[(i * 2) + 1])
                        );
            }
        }

        for (i = 0; i < cameras_number; i++)
        {
            /* ...apply configuration instantly */
            CHK_API(imr_cfg_apply(app->imr, (i * 2) + 0, app->cfg[(i * 2) + 0]));
            CHK_API(imr_cfg_apply(app->imr, (i * 2) + 1, app->cfg[(i * 2) + 1]));
        }
    }

    /* ...buffers not ready */
    app->input_ready = (1 << cameras_number) - 1;

    /* ...output buffers are not ready */
    app->output_ready = (1 << imr_number) - 1;

    /* ...start IMR operation */
    CHK_API(imr_start(app->imr));

    TRACE(INFO, _b("run-time initialized: VIN: %d*%d@%c%c%c%c, VSP: %d*%d, DISP: %d*%d"), __vin_width, __vin_height, __v4l2_fmt(__vin_format), __vsp_width, __vsp_height, w, h);

    return 0;
}

/*******************************************************************************
 * Application thread
 ******************************************************************************/

void * app_thread(void *arg)
{
    /* ...nothing to do */
    while (1)   sleep(10);

    return NULL;
}
static inline widget_data_t * app_kbd_event(app_data_t *app, widget_data_t *widget, widget_key_event_t *event)
{
    if (event->type == WIDGET_EVENT_KEY_PRESS && event->state)
    {
        switch (event->code)
        {
        case KEY_ESC:
        case KEY_SPACE:
        case KEY_Z:
            {
                pthread_mutex_lock(&app->lock);
                app->dump_png = 1;
                pthread_mutex_unlock(&app->lock);
                TRACE(1, _b("take camera snapshot"));
            }
            break;

        default:
            /* ...pass to IMR engine */
            break;
        }
    }

    /* ...always keep focus */
    return widget;
}

/* ...event-processing function */
static widget_data_t * app_input_event(widget_data_t *widget, void *cdata, widget_event_t *event)
{
    app_data_t     *app = cdata;
    /* ...pass event to GUI layer first */

    switch (WIDGET_EVENT_TYPE(event->type))
    {
    case WIDGET_EVENT_KEY:
        return app_kbd_event(app, widget, &event->key);
    default:
        return NULL;
    }
}

/*******************************************************************************
 * Window parameters
 ******************************************************************************/

/* ...processing window parameters */
static window_info_t app_main_info = {
    .fullscreen = 1,
    .redraw = app_redraw,
};

/* ...main window widget parameters (input-interface + GUI?) */
static widget_info_t app_main_info2 = {
    .init = app_context_init,
    .event = app_input_event,
};

/*******************************************************************************
 * Layout parameters
 ******************************************************************************/

static inline void init_layout(int32_t camera_number, int32_t out_width, int32_t out_height)
{
    switch(camera_number)
    {
        case 1:
            layout_config_list[0].x_start = 0;
            layout_config_list[0].y_start = 0;
            layout_config_list[0].width = out_width;
            layout_config_list[0].height = out_height;
            break;
        case 2:
            layout_config_list[0].x_start = 0;
            layout_config_list[0].y_start = 0;
            layout_config_list[0].width = out_width / 2;
            layout_config_list[0].height = out_height;
            layout_config_list[1].x_start = out_width / 2;
            layout_config_list[1].y_start = 0;
            layout_config_list[1].width = out_width / 2;
            layout_config_list[1].height = out_height;
            break;
        case 3:
            layout_config_list[0].x_start = 0;
            layout_config_list[0].y_start = 0;
            layout_config_list[0].width = out_width / 2;
            layout_config_list[0].height = out_height / 2;

            layout_config_list[1].x_start = 0;
            layout_config_list[1].y_start = out_height / 2;
            layout_config_list[1].width = out_width / 2;
            layout_config_list[1].height = out_height;

            layout_config_list[2].x_start = out_width / 2;
            layout_config_list[2].y_start = 0;
            layout_config_list[2].width = out_width / 2;
            layout_config_list[2].height = out_height;
            break;
        case 4:
            layout_config_list[0].x_start = 0;
            layout_config_list[0].y_start = 0;
            layout_config_list[0].width = out_width / 2;
            layout_config_list[0].height = out_height / 2;

            layout_config_list[1].x_start = 0;
            layout_config_list[1].y_start = out_height / 2;
            layout_config_list[1].width = out_width / 2;
            layout_config_list[1].height = out_height;

            layout_config_list[2].x_start = out_width / 2;
            layout_config_list[2].y_start = 0;
            layout_config_list[2].width = out_width / 2;
            layout_config_list[2].height = out_height / 2;

            layout_config_list[3].x_start = out_width / 2;
            layout_config_list[3].y_start = out_height / 2;
            layout_config_list[3].width = out_width / 2;
            layout_config_list[3].height = out_height / 2;
            break;
        default:
            TRACE(ERROR, _x("Unsupported camera number: %d"), camera_number);
            break;
    }
}

/*******************************************************************************
 * Entry point
 ******************************************************************************/

/* ...module initialization function */
app_data_t * app_init(display_data_t *display)
{
    app_data_t            *app;
    pthread_mutexattr_t    attr;

    /* ...create local data handle */
    CHK_ERR(app = calloc(1, sizeof(*app)), (errno = ENOMEM, NULL));

    /* ...set output device dimensions */
    app_main_info.width = __vsp_width;
    app_main_info.height = __vsp_height;
    app->dump_png = 2;
    init_layout(cameras_number, __vsp_width, __vsp_height);

    /* ...initialize data access lock */
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&app->lock, &attr);
    pthread_mutexattr_destroy(&attr);

    /* ...create full-screen window for processing results visualization */
    if ((app->window = window_create(display, &app_main_info, &app_main_info2, app)) == NULL)
    {
        TRACE(ERROR, _x("failed to create main window: %m"));
        goto error;
    }

    /* ...start VIN interface */
    if (vin_start(app->vin) < 0)
    {
        TRACE(ERROR, _x("failed to start VIN: %m"));
        goto error;
    }

    TRACE(INIT, _b("application initialized"));

    return app;

error:
    /* ...destroy main application window */
    (app->window ? window_destroy(app->window) : 0);

    /* ...destroy data handle */
    free(app);

    return NULL;
}
