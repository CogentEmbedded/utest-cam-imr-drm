/*******************************************************************************
 * utest-app.c
 *
 * IMR unit test application
 *
 * Copyright (c) 2017 Cogent Embedded Inc. ALL RIGHTS RESERVED.
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
#include <linux/videodev2.h>
#include "imr-v4l2-api.h"

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 0);

/*******************************************************************************
 * Local constants definitions
 ******************************************************************************/

/* ...number of cameras */
#define CAMERAS_NUMBER                  4
int cameras_number = CAMERAS_NUMBER;

/* ...maximal number of triangles to have for a single camera */
#define IMR_CFG_CAMERA_SIZE             10000

/* ...output buffer pool size */
#define VSP_POOL_SIZE                   3

/*******************************************************************************
 * Local types definitions
 ******************************************************************************/

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
    
    /* ...data access lock */
    pthread_mutex_t     lock;

    /* ...VIN handle */
    vin_data_t         *vin;

    /* ...IMR engine handle */
    imr_data_t         *imr;

    /* ...buffers memory */
    vsp_mem_t          *camera_plane[CAMERAS_NUMBER][VSP_POOL_SIZE];
    
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
static void __destroy_vsink_texture(gpointer data, GstMiniObject *obj)
{
    GstBuffer      *buffer = (GstBuffer *)obj;
    vsink_meta_t   *meta = gst_buffer_get_vsink_meta(buffer);

    TRACE(DEBUG, _b("destroy texture referenced by meta: %p:%p"), meta, meta->priv);

    /* ...destroy texture */
    texture_destroy(meta->priv);
}

/* ...input buffer allocation */
static int app_input_alloc(void *data, int i, GstBuffer *buffer)
{
    app_data_t     *app = data;
    vsink_meta_t   *vmeta = gst_buffer_get_vsink_meta(buffer);
    int             w = vmeta->width, h = vmeta->height;

    /* ...make sure camera index is valid */
    CHK_ERR((u32)i < (u32)cameras_number, -EINVAL);

    if (app->width)
    {
        /* ...verify buffer dimensions are valid */
        CHK_ERR(w == app->width && h == app->height, -EINVAL);
    }
    else
    {
        /* ...check dimensions are valid */
        CHK_ERR(w && h, -EINVAL);
        
        /* ...set buffer dimensions */
        app->width = w, app->height = h;
    }

    TRACE(DEBUG, _b("dmafd: %d/%d/%d, offset: %d/%d/%d, stride: %u/%u/%u"),
          vmeta->dmafd[0], vmeta->dmafd[1], vmeta->dmafd[2],
          vmeta->offset[0], vmeta->offset[1], vmeta->offset[2],
          vmeta->stride[0], vmeta->stride[1], vmeta->stride[2]);

    /* ...allocate texture to wrap the buffer (maybe NULL) */
    vmeta->priv = texture_create(w, h, vmeta->format, vmeta->dmafd, vmeta->offset, vmeta->stride);

    /* ...add custom destructor to the buffer */
    gst_mini_object_weak_ref(GST_MINI_OBJECT(buffer), __destroy_vsink_texture, app);

    /* ...do not need to do anything with the buffer allocation? */
    TRACE(INFO, _b("camera-%d: input buffer allocated (%p)"), i, buffer);

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

/* ...process new input buffer submitted from camera */
static int app_input_process(void *data, int i, GstBuffer *buffer)
{
    app_data_t     *app = data;
    vsink_meta_t   *vmeta = gst_buffer_get_vsink_meta(buffer);
    char fname[256];

    TRACE(DEBUG, _b("camera-%d: input buffer received"), i);

    /* ...make sure camera index is valid */
    CHK_ERR((u32)i < (u32)cameras_number, -EINVAL);

    /* ...make sure buffer dimensions are valid */
    CHK_ERR(vmeta && vmeta->width == app->width && vmeta->height == app->height, -EINVAL);

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

            //vin_get_otp_id
            if (app->dump_png) {
                vin_device_snapshot(app->vin, i, buf);
            }
#if 0
            sprintf(fname, "/tmp/camera-%04d.uyvy", i);
            vmeta = gst_buffer_get_vsink_meta(buf);
            store_uyvy(fname, vmeta->width, vmeta->height, __pixfmt_gst_to_v4l2(vmeta->format), vmeta->plane[0]);
            TRACE(INFO, _b("dump %dx%d"), vmeta->width, vmeta->height);
#endif
            /* ...submit to corresponding IMR engine */
            CHK_API(imr_engine_push_buffer(app->imr, i, buf));

            /* ...drop the reference to input buffer */
            gst_buffer_unref(buf);

            /* ...update readiness flag */
            (g_queue_is_empty(&app->input[i]) ? app->input_ready |= (1 << i) : 0);
        }
        app->dump_png = 0;
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
    int             w = meta->width, h = meta->height, s = meta->stride;
    int             format = meta->format;
    int             j = meta->index;
    int             dmafd[GST_VIDEO_MAX_PLANES];
    u32             offset[GST_VIDEO_MAX_PLANES];
    u32             stride[GST_VIDEO_MAX_PLANES];

    /* ...verify engine id */
    CHK_ERR((u32)i < (u32)cameras_number, -(errno = EBADFD));

    /* ...verify individula buffer index */
    CHK_ERR((u32)j < (u32)VSP_POOL_SIZE, -(errno = EBADFD));

    /* ...save pointer to the memory buffer */
    meta->priv = app->camera_plane[i][j];

    if (format == GST_VIDEO_FORMAT_GRAY8)
    {
        /* output as NV16 with UV fixed to 0x80 (blank) */
        memset(vsp_mem_ptr(meta->priv), 0x80, vsp_mem_size(meta->priv));
    }
    
    /* ...create DMA buffers for a memory chunk */
    CHK_API(vsp_buffer_export(meta->priv, w, h, s, __pixfmt_gst_to_v4l2(format), dmafd, offset, stride));

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
    app_data_t     *app = cdata;
    imr_meta_t     *meta = gst_buffer_get_imr_meta(buffer);
    vsp_mem_t      *mem = meta->priv;
    int             j = meta->index;

    /* ...sanity check */
    CHK_ERR((u32)i < (u32)cameras_number, -(errno = EBADFD));
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

    /* ...submit buffer to output queue */
    g_queue_push_tail(&app->output[i], gst_buffer_ref(buffer));

    /* ...notify renderer only when all buffers have been collected */
    if ((app->output_ready &= ~(1 << i)) == 0)
    {
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
        GstBuffer      *buf[CAMERAS_NUMBER];
        int             i;
        static char     fname[256];
        imr_meta_t     *meta;
        
        /* ...get buffers from output queue */
        for (i = 0; i < cameras_number; i++)
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

#if 0
            sprintf(fname, "/tmp/camera-imr-%04d.uyvy", i);
            meta = gst_buffer_get_imr_meta(buf[i]);
            store_uyvy(fname, meta->width, meta->height, meta->format, vsp_mem_ptr(meta->priv));
            TRACE(INFO, _b("dump %dx%d"), meta->width, meta->height);
#endif

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

        /* ...release old buffers (we need to keep last buffer) */
        for (i = 0; i < cameras_number; i++)
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

    /* ...release processing lock */
    pthread_mutex_unlock(&app->lock);    

    TRACE(DEBUG, _b("drawing complete.."));
}

/* ...setup IMR configuration for a particular camera */
static int __camera_cfg_setup(app_data_t *app, int id, int w, int h, int W, int H, imr_cfg_t *cfg)
{
    display_list_t     *dl = imr_cfg_dl(cfg);
    int                 columns, rows;
    int                 x, y, dx, dy;
    int                 i, j;

    /* ...calculate lattice size (in destination space) */
    columns = ((W << DL_DST_SUBSAMPLE)+ DL_DST_THRESHOLD - 1) / DL_DST_THRESHOLD;
    rows = ((H << DL_DST_SUBSAMPLE) + DL_DST_THRESHOLD - 1) / DL_DST_THRESHOLD;

    /* ...bail out if we have empty lattice */
    if (!columns || !rows)  return 0;

    /* ...put lattice configuration */
    dx = (W >= DL_DST_THRESHOLD ? DL_DST_THRESHOLD : W);
    dy = (H >= DL_DST_THRESHOLD ? DL_DST_THRESHOLD : H);
    CHK_API(dl_autocg_set_dxdy(dl, dx, dy));

    /* ...set origin in destination space at <0,0> */
    CHK_API(dl_autocg_set_x(dl, 0));

    float   x_ratio = (float)w / W;
    float   y_ratio = (float)h / H;
    int     n = (columns + 1) * 2;
    
    /* ...process the strips */
    for (i = 0, y = 0; i < rows; i++, y += DL_DST_THRESHOLD)
    {
        dl_strip_abs_t   *s;

        /* ...adjust strip source vertical coordinate */
        CHK_API(dl_autocg_set_y(dl, y));

        /* ...get next DL-record */
        CHK_ERR(s = dl_abs_strip_create(dl, n), -1);
        
        /* ...generate strip of length 2 * columns */
        for (j = 0, x = 0; j <= columns; j++, x += DL_DST_THRESHOLD, s += 2)
        {
            float X = (float)x / (W << DL_DST_SUBSAMPLE);
            float Y = (float)y / (H << DL_DST_SUBSAMPLE);
            
            /* ...{x,y} is a point in destination space; get associated point in source space */
            u16     u = (u16)(X * ((w << DL_SRC_SUBSAMPLE) - 1));
            u16     v = (u16)(Y * (h << DL_SRC_SUBSAMPLE));

            TRACE(0, _b("x,y=%u,%u, X,Y=%f,%f, u,v=%u,%u"), x, y, X, Y, u, v);

            s[0].uX = u;
            s[0].vY = v;

            Y = (float)(y + DL_DST_THRESHOLD) / (H << DL_DST_SUBSAMPLE);
            v = (u16)(Y * (h << DL_SRC_SUBSAMPLE));
            s[1].uX = u;
            s[1].vY = v;

            TRACE(0, _b("x,y=%u,%u, u,v=%u,%u"), x, y, u, v);
        }
    }

    TRACE(INIT, _b("camera-%d: display-list created (%d*%d, %u bytes)"), id, columns, rows, dl->n);

    BUG(0, _x("break"));
    
    return 0;
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

    /* ...create VIN engine */
    CHK_ERR(app->vin = vin_init(vin_dev_name, cameras_number, &camera_cb, app), -1);

    /* ...create IMR engine */
    CHK_ERR(app->imr = imr_init(imr_dev_name, cameras_number, &imr_cb, app), -1);
    
    /* ...allocate output buffers for IMR */
    CHK_API(vsp_allocate_buffers(W, H, 0, __vin_format, &app->camera_plane[0][0], cameras_number * VSP_POOL_SIZE));

    /* ...prepare IMR configuration descriptor for camera-planes */
    c_type = IMR_MAP_TME | IMR_MAP_BFE | IMR_MAP_AUTODG | 0*IMR_MAP_TCM;
    c_type |= IMR_MAP_UVDPOR(DL_SRC_SUBSAMPLE) | (DL_DST_SUBSAMPLE ? IMR_MAP_DDP : 0);

    /* ...configure VINs */
    for (i = 0; i < cameras_number; i++)
    {
        /* ...configure VIN device */
        CHK_API(vin_device_configure(app->vin, i, __vin_width, __vin_height, __vin_stride, __vin_format, __vin_buffers_num));
    }

    /* ...setup VINs */
    for (i = 0; i < cameras_number; i++)
    {
        /* ...setup VIN device */
        CHK_API(vin_device_init(app->vin, i, __vin_width, __vin_height, __vin_stride, __vin_format, __vin_buffers_num));

        /* ...setup IMR engine to do a simple scaling */
        CHK_API(imr_setup(app->imr, i, __vin_width, __vin_height, __vin_stride, W, H, 0, fmt, fmt, VSP_POOL_SIZE));

        /* ...allocate engine configuration */
        CHK_ERR(app->cfg[i] = imr_cfg_create(c_type, IMR_CFG_CAMERA_SIZE * sizeof(dl_abs_t)), -1);

        /* ...setup camera configuration */
        CHK_API(__camera_cfg_setup(app, i, __vin_width, __vin_height, W, H, app->cfg[i]));

        /* ...apply configuration instantly */
        CHK_API(imr_cfg_apply(app->imr, i, app->cfg[i]));
    }

    /* ...buffers not ready */
    app->input_ready = (1 << cameras_number) - 1;

    /* ...output buffers are not ready */
    app->output_ready = (1 << cameras_number) - 1;

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
    app->dump_png = 0;

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
