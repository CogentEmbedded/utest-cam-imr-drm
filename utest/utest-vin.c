/*******************************************************************************
 * utest-vin.c
 *
 * ADAS unit-test. VIN LVDS cameras backend
 *
 * Copyright (c) 2015 Cogent Embedded Inc. ALL RIGHTS RESERVED.
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

#define MODULE_TAG                      VIN

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "utest-common.h"
#include "utest-camera.h"
#include "utest-vsink.h"
#include <ccamera.h>
#include <ccamera/ccamera-memory-dma.h>

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 0);

/*******************************************************************************
 * Local constants definitions
 ******************************************************************************/

/* ...individual camera buffer pool size */
#define VIN_BUFFER_POOL_SIZE            5
#define OTP_ID_NAME_SIZE        32
/*******************************************************************************
 * Local types definitions
 ******************************************************************************/

/* ...decoder data structure */
typedef struct camera_data
{
    char *name;

    int camera_fd;

    int camera_id;

    /* ...decoder activity state */
    int                         active;

    /* ...queue access lock */
    pthread_mutex_t             lock;

    pthread_t                   thread;

    /* ...decoding thread conditional variable */
    pthread_cond_t              wait;

    /* ...application-provided callback */
    const camera_callback_t    *cb;

    /* ...application callback data */
    void                       *cdata;

}   camera_data_t;

/*******************************************************************************
 * Custom buffer metadata implementation
 ******************************************************************************/

/* ...metadata structure */
typedef struct vin_meta
{
    GstMeta             meta;

    /* ...camera identifier */
    int                 camera_id;

    camera_buf_t        *buffer;

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

/* ...metadata type registration */
GType vin_meta_api_get_type(void)
{
    static volatile GType type;
    static const gchar *tags[] = { GST_META_TAG_VIDEO_STR, GST_META_TAG_MEMORY_STR, NULL };

    if (g_once_init_enter(&type))
    {
        GType _type = gst_meta_api_type_register("VinDecMetaAPI", tags);
        g_once_init_leave(&type, _type);
    }

    return type;
}

/* ...low-level interface */
static gboolean vin_meta_init(GstMeta *meta, gpointer params, GstBuffer *buffer)
{
    vin_meta_t     *_meta = (vin_meta_t *) meta;

    /* ...reset fields */
    memset(&_meta->meta + 1, 0, sizeof(vin_meta_t) - sizeof(GstMeta));

    return TRUE;
}

/* ...metadata transformation */
static gboolean vin_meta_transform(GstBuffer *transbuf, GstMeta *meta,
        GstBuffer *buffer, GQuark type, gpointer data)
{
    vin_meta_t     *_meta = (vin_meta_t *) meta, *_tmeta;

    /* ...add JPU metadata for a transformed buffer */
    _tmeta = gst_buffer_add_vin_meta(transbuf);

    /* ...just copy data regardless of transform type? */
    memcpy(&_tmeta->meta + 1, &_meta->meta + 1, sizeof(vin_meta_t) - sizeof(GstMeta));

    return TRUE;
}

/* ...metadata release */
static void vin_meta_free(GstMeta *meta, GstBuffer *buffer)
{
    vin_meta_t     *_meta = (vin_meta_t *) meta;

    /* ...anything to destroy? - tbd */
    TRACE(DEBUG, _b("free metadata %p"), _meta);
}

/* ...register metadata implementation */
const GstMetaInfo * vin_meta_get_info(void)
{
    static const GstMetaInfo *meta_info = NULL;

    if (g_once_init_enter(&meta_info))
    {
        const GstMetaInfo *mi = gst_meta_register(
            VIN_META_API_TYPE,
            "VinDecMeta",
            sizeof(vin_meta_t),
            vin_meta_init,
            vin_meta_free,
            vin_meta_transform);

        g_once_init_leave (&meta_info, mi);
    }

    return meta_info;
}

/* ...output buffer dispose function (called in response to "gst_buffer_unref") */
static gboolean __output_buffer_dispose(GstMiniObject *obj)
{
    GstBuffer      *buffer = GST_BUFFER(obj);
    camera_data_t  *camera_data = (camera_data_t *)buffer->pool;
    vin_meta_t     *meta = gst_buffer_get_vin_meta(buffer);
    int             i = meta->camera_id;
    camera_buf_t *camera_buffer = meta->buffer;
    gboolean        destroy;

    /* ...lock internal data access */
    pthread_mutex_lock(&camera_data->lock);

    camera_release_buffer(camera_buffer);

    /* ...release data access lock */
    pthread_mutex_unlock(&camera_data->lock);

    destroy = TRUE;

    return destroy;
}

/* ...buffer processing function */
static inline int __process_buffer(camera_data_t *camera_data, camera_buf_t *cam_buf)
{
    GstBuffer      *buffer;
    int             j;
    u64             ts;
    u32             seq;
    vin_meta_t     *meta;
    vsink_meta_t   *vmeta;

    CHK_ERR(buffer = gst_buffer_new(), -ENOMEM);

    CHK_ERR(meta = gst_buffer_add_vin_meta(buffer), -ENOMEM);
    meta->camera_id = camera_data->camera_id;
    meta->buffer = cam_buf;
    GST_META_FLAG_SET(meta, GST_META_FLAG_POOLED);

    /* ...add vsink metadata */
    CHK_ERR(vmeta = gst_buffer_add_vsink_meta(buffer), -ENOMEM);
    vmeta->width = cam_buf->width;
    vmeta->height = cam_buf->height;
    vmeta->format = GST_VIDEO_FORMAT_UYVY;

    vmeta->dmafd[0] = camera_buffer_get_dmafd(cam_buf, 0);
    vmeta->plane[0] = (void *)cam_buf->planes[0].data;
    ts = cam_buf->timestamp;
    TRACE(1, _b("%lu"), ts);

    /* TODO multiplanar */

    GST_META_FLAG_SET(vmeta, GST_META_FLAG_POOLED);

    /* ...modify buffer release callback */
    GST_MINI_OBJECT(buffer)->dispose = __output_buffer_dispose;

    /* ...use "pool" pointer as a custom data */
    buffer->pool = (void *)camera_data;

    /* ...set decoding/presentation timestamp (in nanoseconds) */
    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer) = ts * 1000;

    /* ...release lock before passing buffer to the application */
    pthread_mutex_unlock(&camera_data->lock);

    /* ...pass output buffer to application */
    CHK_API(camera_data->cb->process(camera_data->cdata, camera_data->camera_id, buffer));

    /* ...drop the reference (buffer is now owned by application) */
    gst_buffer_unref(buffer);

    /* ...reacquire data access lock */
    pthread_mutex_lock(&camera_data->lock);

    return 0;
}

/* ...decoding thread */
static void * camera_thread(void *arg)
{
    camera_data_t         *camera_data = arg;

    /* ...lock internal data access */
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
    pthread_mutex_lock(&camera_data->lock);

    /* ...start processing loop */
    while (1)
    {
        int     r, k;
        camera_buf_t *buffer = NULL;
        /* ...release the lock before going to waiting state */
        pthread_mutex_unlock(&camera_data->lock);
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

        TRACE(DEBUG, _b("start waiting..."));

        /* ...wait for event (infinite timeout) */
        buffer = camera_get_buffer(camera_data->camera_fd, 0);

        TRACE(DEBUG, _b("done waiting: %p"), buffer);

        /* ...reacquire the lock */
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
        pthread_mutex_lock(&camera_data->lock);

        if (!buffer)
        {
            TRACE(1, _b("No buffer received, continue..."));
            continue;
        }

        if (__process_buffer(camera_data, buffer) < 0)
        {
            TRACE(ERROR, _x("processing failed: %m"));
            goto out;
        }
    }

out:
    /* ...release access lock */
    pthread_mutex_unlock(&camera_data->lock);
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    TRACE(INIT, _b("thread exits: %m"));

    return (void *)(intptr_t)-errno;
}


/* ...start module operation */
int camera_start(camera_data_t *camera_data)
{
    pthread_attr_t  attr;
    int             r;

    /* ...initialize thread attributes (joinable, 128KB stack) */
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setstacksize(&attr, 128 << 10);

    /* ...mark module is active */
    camera_data->active = 1;

    /* ...create V4L2 thread to asynchronously process input buffers */
    r = pthread_create(&camera_data->thread, &attr, camera_thread, camera_data);
    pthread_attr_destroy(&attr);

    return CHK_API(r);
}

/*******************************************************************************
 * Buffer pool handling
 ******************************************************************************/

/* ...module initialization function */
camera_data_t **camera_init(char **devname, int num, int w, int h, u32 fmt, camera_callback_t *cb, void *cdata)
{
    camera_data_t             *camera_data;
    camera_data_t             **retval;
    pthread_mutexattr_t     attr;
    int                     i;


    retval = calloc(num, sizeof(camera_data_t *));

    /* ...allocate engine-specific data */
    if ((camera_data = calloc(num, sizeof(camera_data_t))) == NULL)
    {
        TRACE(ERROR, _x("failed to allocate %zu bytes"), num * sizeof(camera_data_t));
        goto error;
    }

    CHK_ERR(camera_data = calloc(num, sizeof(*camera_data)), (errno = ENOMEM, NULL));

    for (i = 0; i < num; i++)
    {
        retval[i] = &camera_data[i];

        camera_data[i].cb = cb, camera_data[i].cdata = cdata;
        camera_data[i].name = devname[i];
        camera_data[i].camera_id = i;
        camera_data[i].camera_fd = camera_open_with_args(camera_data[i].name,
                                                         "memory=dma",
                                                         0,
                                                         "UYVY", /* TODO */
                                                         w,
                                                         h,
                                                         30);

        /* ...initialize internal queue access lock */
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        pthread_mutex_init(&camera_data->lock, &attr);
        pthread_mutexattr_destroy(&attr);
    }


    TRACE(INIT, _b("CAMERA module initialized"));

    return retval;

error:

    return NULL;
}
