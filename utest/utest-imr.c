/*******************************************************************************
 * utest-imr.c
 *
 * ADAS unit-test. IMR module interface
 *
 * Copyright (c) 2015-2017 Cogent Embedded Inc. ALL RIGHTS RESERVED.
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

#define MODULE_TAG                      IMR

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "utest-imr.h"
#include "utest-vin.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/epoll.h>
#include <poll.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include "imr-v4l2-api.h"
#include "utest-common-math.h"
#include <math.h>

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 1);

/*******************************************************************************
 * Local types definitions
 ******************************************************************************/

/* ...IMR device data */
typedef struct imr_device
{
    /* ...V4L2 file decriptor */
    int                     vfd;

    /* ...input/output buffers pool length */
    int                     size;

    /* ...input/output buffers dimensions */
    int                     w, h, W, H;

    /* ...output buffers pool */
    imr_buffer_t           *pool;

    /* ...pending input buffers */
    GQueue                  input;

    /* ...streaming status */
    int                     active;

    /* ...number of submitted/busy buffer-pairs */
    int                     submitted, busy;

    /* ...index of next buffer-pair to submit to device */
    int                     index;

    /* ...frame sequence number */
    u32                     sequence;

    /* ...length of input/output buffers */
    u32                     input_length, output_length;

    /* ...processing time estimation */
    u32                     ts_acc;

}   imr_device_t;

/* ...distortion correction engine data */
typedef struct imr_data
{
    /* ...number of devices */
    int                     num;

    /* ...device-specific data */
    imr_device_t           *dev;

    /* ...poll file descriptors */
    struct pollfd          *pfd;

    /* ...engine activity flag */
    int                     active;

    /* ...module status */
    u32                     flags;

    /* ...internal data access lock */
    pthread_mutex_t         lock;

    /* ...processing thread */
    pthread_t               thread;

    /* ...user-provided callbacks */
    camera_callback_t      *cb;

    /* ...application callback data */
    void                   *cdata;

}   imr_data_t;

/*******************************************************************************
 * Custom buffer metadata implementation
 ******************************************************************************/

/* ...metadata type registration */
GType imr_meta_api_get_type(void)
{
    static volatile GType type;
    static const gchar *tags[] = { GST_META_TAG_MEMORY_STR, NULL };

    if (g_once_init_enter(&type))
    {
        GType _type = gst_meta_api_type_register("IMRMetaAPI", tags);
        g_once_init_leave(&type, _type);
    }

    return type;
}

/* ...low-level interface */
static gboolean imr_meta_init(GstMeta *meta, gpointer params, GstBuffer *buffer)
{
    imr_meta_t     *_meta = (imr_meta_t *) meta;

    /* ...reset fields */
    memset(&_meta->meta + 1, 0, sizeof(imr_meta_t) - sizeof(GstMeta));

    return TRUE;
}

/* ...metadata transformation */
static gboolean imr_meta_transform(GstBuffer *transbuf, GstMeta *meta,
        GstBuffer *buffer, GQuark type, gpointer data)
{
    imr_meta_t     *_meta = (imr_meta_t *) meta, *_tmeta;

    /* ...add JPU metadata for a transformed buffer */
    _tmeta = gst_buffer_add_imr_meta(transbuf);

    /* ...just copy data regardless of transform type? */
    memcpy(&_tmeta->meta + 1, &_meta->meta + 1, sizeof(imr_meta_t) - sizeof(GstMeta));

    return TRUE;
}

/* ...metadata release */
static void imr_meta_free(GstMeta *meta, GstBuffer *buffer)
{
    imr_meta_t     *_meta = (imr_meta_t *) meta;

    /* ...anything to destroy? - tbd */
    TRACE(DEBUG, _b("free metadata %p"), _meta);
}

/* ...register metadata implementation */
const GstMetaInfo * imr_meta_get_info(void)
{
    static const GstMetaInfo *meta_info = NULL;

    if (g_once_init_enter(&meta_info))
    {
        const GstMetaInfo *mi = gst_meta_register(
            IMR_META_API_TYPE,
            "IMRMeta",
            sizeof(imr_meta_t),
            imr_meta_init,
            imr_meta_free,
            imr_meta_transform);

        g_once_init_leave (&meta_info, mi);
    }

    return meta_info;
}

/*******************************************************************************
 * Processing time estimation
 ******************************************************************************/

/* ...reset FPS calculator */
static inline void imr_avg_time_reset(imr_device_t *dev)
{
    /* ...reset processing time accumulator */
    dev->ts_acc = 0;
}

/* ...update FPS calculator */
static inline u32 imr_avg_time_update(imr_device_t *dev, u32 delta)
{
    s32     acc;

    BUG(0 && (s32)delta < 0, _x("invalid delta: %d"), (s32)delta);

    /* ...check if accumulator is initialized */
    if ((acc = dev->ts_acc) == 0)
    {
        /* ...initialize accumulator on first invocation */
        acc = delta << 4;
    }
    else
    {
        /* ...accumulator is setup already; do exponential averaging */
        acc += delta - ((acc + 8) >> 4);
        ((s32)acc < 0 ? acc = 0 : 0);
    }

    /* ...calculate current frame-rate */
    TRACE(DEBUG, _b("delta: %u, acc: %u, fps: %f"), delta, acc, (acc ? 1e+6 / ((acc + 8) >> 4) : 0));

    /* ...update timestamp and accumulator values */
    dev->ts_acc = acc;

    return (acc + 8) >> 4;
}

/* ...get current average processing time */
static inline u32 imr_avg_time(imr_device_t *dev)
{
    return (dev->ts_acc + 8) >> 4;
}

/*******************************************************************************
 * V4L2 IMR interface helpers
 ******************************************************************************/

/* ...check video device capabilities */
static inline int __imr_check_caps(int vfd)
{
	struct v4l2_capability  cap;
    u32                     caps;

    /* ...query device capabilities */
    CHK_API(ioctl(vfd, VIDIOC_QUERYCAP, &cap));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    caps = cap.device_caps;
#else
    caps = cap.capabilities;
#endif

    /* ...check for a required capabilities */
    if (!(caps & V4L2_CAP_VIDEO_OUTPUT))
    {
        TRACE(ERROR, _x("single-planar input expected: %X"), caps);
        return -1;
    }
    else if (!(caps & V4L2_CAP_VIDEO_CAPTURE))
    {
        TRACE(ERROR, _x("single-planar output expected: %X"), caps);
        return -1;
    }
    else if (!(caps & V4L2_CAP_STREAMING))
    {
        TRACE(ERROR, _x("streaming I/O is expected: %X"), caps);
        return -1;
    }

    return 0;
}

/* ...prepare IMR module for operation */
static inline int imr_set_formats(int vfd, u32 w, u32 h, u32 W, u32 H, int stride, u32 ifmt, u32 ofmt)
{
	struct v4l2_format  fmt;

    /* ...set input format */
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	fmt.fmt.pix.pixelformat = ifmt;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;
    fmt.fmt.pix.width = w;
    fmt.fmt.pix.height = h;
    u32 bytesperline = CHECK_MULTIPLICITY(stride, 255);
    fmt.fmt.pix.bytesperline = bytesperline;
    CHK_API(ioctl(vfd, VIDIOC_S_FMT, &fmt));

    TRACE(INFO, _b("requested format: %u * %u, adjusted: %u * %u"), w, h, fmt.fmt.pix.width, fmt.fmt.pix.height);

    /* ...verify actual width/height haven't been changed */
    CHK_ERR(!w || !h || (fmt.fmt.pix.width == w && fmt.fmt.pix.height == h), -(errno = ERANGE));

    /* ...set output format */
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.pixelformat = ofmt;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;
    fmt.fmt.pix.width = W;
    fmt.fmt.pix.height = H;
    bytesperline = CHECK_MULTIPLICITY(W, 63);
    fmt.fmt.pix.bytesperline = bytesperline;
    CHK_API(ioctl(vfd, VIDIOC_S_FMT, &fmt));

    TRACE(INFO, _b("requested output format: %u * %u, adjusted: %u * %u"), W, H, fmt.fmt.pix.width, fmt.fmt.pix.height);

    /* ...verify actual width/height haven't been changed */
    CHK_ERR(fmt.fmt.pix.width == W && fmt.fmt.pix.height == H, -(errno = ERANGE));

    return 0;
}

/* ...start/stop streaming on specific V4L2 device */
static inline int imr_streaming_enable(int vfd, int enable)
{
    int     opcode = (enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF);
    int     type;

    CHK_API(ioctl(vfd, opcode, (type = V4L2_BUF_TYPE_VIDEO_OUTPUT, &type)));
    CHK_API(ioctl(vfd, opcode, (type = V4L2_BUF_TYPE_VIDEO_CAPTURE, &type)));

    return 0;
}

/* ...allocate buffer pool */
static inline int imr_allocate_buffers(int vfd, int num)
{
    struct v4l2_requestbuffers  reqbuf;

    /* ...allocate input buffers (user-provided memory) */
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    reqbuf.memory = V4L2_MEMORY_DMABUF;
    reqbuf.count = num;
    CHK_API(ioctl(vfd, VIDIOC_REQBUFS, &reqbuf));
    CHK_ERR(reqbuf.count == (u32)num, -(errno = ENOMEM));

    /* ...allocate output buffers */
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_DMABUF;
    reqbuf.count = num;
    CHK_API(ioctl(vfd, VIDIOC_REQBUFS, &reqbuf));
    CHK_ERR(reqbuf.count == (u32)num, -(errno = ENOMEM));

    TRACE(INFO, _b("buffer-pool allocated (%u buffers)"), num);

    /* ...enable streaming as soon as we are done */
    //CHK_API(imr_streaming_enable(vfd, 1));

    return 0;
}

/* ...destroy output/capture buffer pool */
static inline int imr_destroy_buffers(int vfd)
{
    struct v4l2_requestbuffers  reqbuf;

    /* ...disable streaming before releasing buffers */
    CHK_API(imr_streaming_enable(vfd, 0));

    /* ...release kernel-allocated input buffers */
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    reqbuf.memory = V4L2_MEMORY_DMABUF;
    reqbuf.count = 0;
    CHK_API(ioctl(vfd, VIDIOC_REQBUFS, &reqbuf));

    /* ...release kernel-allocated output buffers */
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_DMABUF;
    reqbuf.count = 0;
    CHK_API(ioctl(vfd, VIDIOC_REQBUFS, &reqbuf));

    TRACE(INFO, _b("buffer-pool destroyed"));

    return 0;
}

/* ...submit intput/output buffer pair */
static inline int imr_buffers_enqueue(int vfd, int j, int ifd, u32 ilen, int ofd, u32 olen)
{
    struct v4l2_buffer  buf;

    TRACE(DEBUG, _b("enqueue: j=%d, ifd=%d, ofd=%d"), j, ifd, ofd);

    /* ...prepare input buffer */
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = j;
    buf.m.fd = ifd;
    buf.length = buf.bytesused = ilen;
    TRACE(DEBUG, _b("input length: %u"), ilen);
    CHK_API(ioctl(vfd, VIDIOC_QBUF, &buf));

    /* ...set buffer parameters */
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = j;
    buf.m.fd = ofd;
    buf.length = olen;
    CHK_API(ioctl(vfd, VIDIOC_QBUF, &buf));

    return 0;
}

/* ...dequeue buffer pair */
static inline int imr_buffers_dequeue(int vfd, int *error, u32 *duration)
{
    struct v4l2_buffer  buf;
    int                 j, k;
    u64                 t0, t1;
    u32                 seq;

    /* ...dequeue input buffer */
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_DMABUF;
    CHK_API(ioctl(vfd, VIDIOC_DQBUF, &buf));
    j = buf.index;
    t0 = buf.timestamp.tv_sec * 1000000ULL + buf.timestamp.tv_usec;
    seq = buf.sequence;

    /* ...dequeue output buffer */
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    CHK_API(ioctl(vfd, VIDIOC_DQBUF, &buf));
    k = buf.index;
    t1 = buf.timestamp.tv_sec * 1000000ULL + buf.timestamp.tv_usec;

    /* ...check buffer-pair is correctly dequeued */
    CHK_ERR(j == k, -(errno = EBADFD));

    /* ...check sequences */
    BUG(buf.sequence != seq, _x("invalid sequence numbers: %u vs %u"), seq, buf.sequence);

    /* ...put buffer procesing status */
    (error ? *error = !!(buf.flags & V4L2_BUF_FLAG_ERROR) : 0);

    /* ...save processing duration as needed */
    (duration ? *duration = (u32)(t1 - t0) : 0);

    return k;
}

/*******************************************************************************
 * V4L2 decoder thread
 ******************************************************************************/

/* ...add device to the poll sources */
static inline int __register_poll(imr_data_t *imr, int i, int active)
{
    imr_device_t       *dev = &imr->dev[i];

    BUG(!dev->active || (active && !dev->submitted), _x("invalid poll op: active=%d, streaming=%d, submitted=%d"), active, dev->active, dev->submitted);

    /* ...add/remove source */
    imr->pfd[i].fd = (active ? dev->vfd : -1);

    TRACE(DEBUG, _b("#%d: poll source %s (%d)"), i, (active ? "added" : "removed"), dev->vfd);

    /* ...notify servicing thread on poll-descriptor set change */
    pthread_kill(imr->thread, SIGUSR1);

    return 0;
}

/* ...submit buffer to the device (called with a decoder lock held) */
static inline int __submit_buffer(imr_data_t *imr, int i)
{
    imr_device_t   *dev = &imr->dev[i];
    GstBuffer      *buffer;
    imr_buffer_t   *buf;
    vin_meta_t     *meta;
    int             j;

    TRACE(DEBUG, _b("#%d: input: %d, submitted: %d, busy: %d"), i, g_queue_get_length(&dev->input), dev->submitted, dev->busy);

    /* ...check if we have a pending input buffer */
    if (g_queue_is_empty(&dev->input))              return 0;

    /* ...check if we have free buffer-pair */
    if (dev->submitted + dev->busy == dev->size)    return 0;

    /* ...get head of the queue */
    buffer = g_queue_pop_head(&dev->input);

    /* ...take vin meta-data */
    meta = gst_buffer_get_vin_meta(buffer);

    /* ...get free buffer-pair index */
    buf = &dev->pool[j = dev->index];

    /* ...save associated input buffer (takes buffer ownership) */
    buf->input = buffer;

    TRACE(DEBUG, _b("enqueue buffer #<%d,%d>"), i, j);

    /* ...prepare output buffer if needed */
    (imr->cb->prepare ? imr->cb->prepare(imr->cdata, i, buf->output) : 0);

    /* ...submit buffer-pair to the V4L2 */
    CHK_API(imr_buffers_enqueue(dev->vfd, j, meta->dmafd[0], dev->input_length, buf->dmafd[0], dev->output_length));

    /* ...advance writing index */
    dev->index = (++j == dev->size ? 0 : j);

    /* ...advance buffer sequence number */
    dev->sequence++;

    /* ...add poll source as required */
    CHK_API(dev->submitted++ == 0 && dev->active ? __register_poll(imr, i, 1) : 0);

    return 0;
}

/* ...buffer processing function (called with a lock held) */
static inline int __process_buffer(imr_data_t *imr, int i)
{
    imr_device_t   *dev = &imr->dev[i];
    GstBuffer      *buffer;
    imr_buffer_t   *buf;
    int             error;
    u32             duration;
    int             j;

    /* ...if streaming is disabled already, bail out */
    if (!dev->active || !dev->submitted)        return 0;

    /* ...get buffer from a device */
    CHK_API(j = imr_buffers_dequeue(dev->vfd, &error, &duration));

    /* ...remove poll-source if last buffer is dequeued */
    (--dev->submitted == 0 ? __register_poll(imr, i, 0) : 0);

    /* ...estimate buffer processing time */
    imr_avg_time_update(dev, duration);

    TRACE(DEBUG, _b("dequeued buffer-pair #<%d,%d>, result: %d, duration: %u, submitted: %d"), i, j, error, duration, dev->submitted);

    /* ...get buffer descriptor */
    buf = &dev->pool[j];

    /* ...return input buffer to caller */
    gst_buffer_unref(buf->input);

    /* ...get output buffer handle */
    buffer = buf->output;

    /* ...advance number of busy buffers */
    dev->busy++;

    /* ...release lock before passing buffer to the application */
    pthread_mutex_unlock(&imr->lock);

    /* ...pass output buffer to application */
    if (imr->cb->process(imr->cdata, i, buffer) != 0)
    {
        TRACE(ERROR, _x("failed to submit buffer to the application: %m"));
    }

    /* ...drop the reference (buffer is now owned by application) */
    gst_buffer_unref(buffer);

    /* ...reaqcuire data access lock */
    pthread_mutex_lock(&imr->lock);

    return 0;
}

/* ...purge buffers */
static inline int __purge_buffer(imr_data_t *imr, int i)
{
    imr_device_t   *dev = &imr->dev[i];
    int             n = dev->submitted;
    int             N = dev->size;
    int             j = dev->index;
    imr_buffer_t   *buf;

    /* ...do nothing if no submitted buffers */
    if (!n)         return 0;

    /* ...get index if oldest submitted buffer */
    dev->index = ((j -= n) < 0 ? j += N : j);

    /* ...purge output queue */
    while (n--)
    {
        TRACE(DEBUG, _b("buffer <%d:%d> purge (index=%d)"), i, j, dev->index);

        /* ...get buffer descriptor */
        buf = &dev->pool[j];

        /* ...release input buffers only (output buffers still belong to the pool) */
        gst_buffer_unref(buf->input);

        /* ...advance pool position */
        (++j == N ? j = 0 : 0);
    }

    /* ...mark we have no submitted buffers anymore */
    dev->submitted = 0;

    return 0;
}

static void __imr_signal(int signo)
{
    TRACE(0, _b("signal received: %d"), signo);
}

/* ...V4L2 processing thread */
static void * imr_thread(void *arg)
{
    imr_data_t         *imr = arg;
    struct sigaction    sa = { .sa_handler = __imr_signal };

    /* ...use USR1 signal as a trigger to rearm poll descriptors */
    CHK_ERR(sigaction(SIGUSR1, &sa, 0) >= 0, NULL);

    pthread_setname_np(pthread_self(), "imr");

    /* ...lock internal data access */
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
    pthread_mutex_lock(&imr->lock);

    /* ...start processing loop */
    while (1)
    {
        int         r, i;
        sigset_t    sigmask;

        /* ...release the lock before going to waiting state */
        pthread_mutex_unlock(&imr->lock);
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

        TRACE(0, _b("start waiting..."));

        /* ...use USR1 signal to rearm the thread */
        sigemptyset(&sigmask);
        sigaddset(&sigmask, SIGINT);

        /* ...wait for event (infinite timeout) */
        r = ppoll(imr->pfd, imr->num, NULL, &sigmask);
        if (r < 0 && errno == EINTR)
        {
            /* ...poll-descriptor set has been adjusted likely */
            r = 0;
        }

        TRACE(0, _b("waiting complete: %d"), r);

        /* ...reacquire the lock */
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
        pthread_mutex_lock(&imr->lock);

        /* ...check operation result */
        if (r < 0)
        {
            TRACE(ERROR, _x("poll failed: %m"));
            goto out;
        }

        /* ...process all signalled descriptors */
        for (i = 0; i < imr->num; i++)
        {
            /* ...process output buffers */
            if (imr->pfd[i].revents & POLLIN)
            {
                if (__process_buffer(imr, i) < 0)
                {
                    TRACE(ERROR, _x("processing failed: %m"));
                    goto out;
                }
            }
        }
    }

out:
    /* ...release access lock */
    pthread_mutex_unlock(&imr->lock);
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    TRACE(INIT, _b("thread exits: %m"));

    return (void *)(intptr_t)-errno;
}

/*******************************************************************************
 * Output buffer pool handling
 ******************************************************************************/

/* ...output buffer dispose function */
static gboolean __imr_buffer_dispose(GstMiniObject *obj)
{
    GstBuffer      *buffer = GST_BUFFER(obj);
    imr_meta_t     *meta = gst_buffer_get_imr_meta(buffer);
    imr_data_t     *imr = (imr_data_t *)buffer->pool;
    int             i = meta->id, j = meta->index;
    imr_device_t   *dev = &imr->dev[i];
    imr_buffer_t   *buf = meta->buf;
    gboolean        destroy;

    /* ...check buffer validity */
    BUG((u32)i >= (u32)imr->num || (u32)j >= (u32)dev->size, _x("invalid buffer: <%d,%d>"), i, j);

    /* ...lock internal data access */
    pthread_mutex_lock(&imr->lock);

    /* ...decrement number of busy buffers */
    dev->busy--;

    TRACE(DEBUG, _b("output buffer #<%d:%d> (%p) returned to pool (busy: %d)"), i, j, buffer, dev->busy);

    /* ...check if buffer needs to be requeued into the pool */
    if (imr->active)
    {
        /* ...increment buffer reference */
        gst_buffer_ref(buffer);

        /* ...force processing if we have a pending input queue */
        if (dev->active)
        {
            __submit_buffer(imr, i);
        }
        else
        {
            /* ...buffer returned while streaming is off; decrement index of the buffer */
            (--dev->index < 0 ? dev->index += dev->size : 0);
            TRACE(DEBUG, _b("revert index: %d"), dev->index);
        }

        /* ...indicate the miniobject should not be freed */
        destroy = FALSE;
    }
    else
    {
        TRACE(DEBUG, _b("output buffer #<%d:%d> (%p) is freed"), i, j, buffer);

        /* ...mark the buffer is freed */
        buf->output = NULL;

        /* ...force destruction of the buffer miniobject */
        destroy = TRUE;
    }

    /* ...release decoder access lock */
    pthread_mutex_unlock(&imr->lock);

    return destroy;
}

/*******************************************************************************
 * Engine setup
 ******************************************************************************/

/* ...resume/suspend streaming */
int imr_enable(imr_data_t *imr, int enable)
{
    imr_device_t   *dev;
    int             i;

    /* ...make sure engine is active */
    CHK_ERR(imr->active, -EINVAL);

    /* ...aqcuire the engine lock */
    pthread_mutex_lock(&imr->lock);

    if (enable)
    {
        /* ...enable streaming */
        for (i = 0; i < imr->num; i++)
        {
            dev = &imr->dev[i];

            if (dev->active)    continue;

            /* ...enable input/output buffers streaming */
            CHK_API(imr_streaming_enable(dev->vfd, dev->active = 1));

            /* ...register poll-source as required */
            CHK_API(dev->submitted ? __register_poll(imr, i, 1) : 0);

            /* ...submit pending input buffers as required */
            CHK_API(__submit_buffer(imr, i));
        }
    }
    else
    {
        /* ...disable streaming */
        for (i = 0; i < imr->num; i++)
        {
            dev = &imr->dev[i];

            if (!dev->active)    continue;

            /* ...unregister poll-source as required */
            CHK_API(dev->submitted ? __register_poll(imr, i, 0) : 0);

            /* ...disable input/output buffers streaming */
            CHK_API(imr_streaming_enable(dev->vfd, dev->active = 0));

            /* ...purge all submitted buffers */
            CHK_API(__purge_buffer(imr, i));
        }
    }

    /* ...release engine lock */
    pthread_mutex_unlock(&imr->lock);

    TRACE(INFO, _b("streaming is %sabled"), (enable ? "en" : "dis"));

    return 0;
}

/* ...start module operation */
int imr_start(imr_data_t *imr)
{
    pthread_attr_t  attr;

    /* ...set decoder active flag */
    imr->active = 1;

    /* ...initialize thread attributes (joinable, 128KB stack) */
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setstacksize(&attr, 128 << 10);

    /* ...create V4L2 decoding thread to asynchronously process buffers */
    CHK_API(pthread_create(&imr->thread, &attr, imr_thread, imr));
    pthread_attr_destroy(&attr);

    /* ...enable streaming */
    CHK_API(imr_enable(imr, 1));

    return 0;
}

/*******************************************************************************
 * Module initialization
 ******************************************************************************/

/* ...IMR engine initialization */
imr_data_t * imr_init(char **devname, int num, camera_callback_t *cb, void *cdata)
{
    imr_data_t             *imr;
    pthread_mutexattr_t     attr;
    int                     i;

    /* ...allocate IMR processor data */
    CHK_ERR(imr = calloc(1, sizeof(*imr)), (errno = ENOMEM, NULL));

    /* ...save application callback data */
    imr->cb = cb, imr->cdata = cdata;

    /* ...allocate engine-specific data */
    if ((imr->dev = calloc(imr->num = num, sizeof(imr_device_t))) == NULL)
    {
        TRACE(ERROR, _x("failed to allocate %zu bytes"), num * sizeof(imr_device_t));
        goto error;
    }

    /* ...create poll file-descriptors */
    if ((imr->pfd = malloc(num * sizeof(*imr->pfd))) == NULL)
    {
        TRACE(ERROR, _x("failed to allocate %zu bytes"), num * sizeof(*imr->pfd));
        goto error;
    }

    /* ...open V4L2 image renderer devices */
    for (i = 0; i < num; i++)
    {
        imr_device_t   *dev = &imr->dev[i];

        /* ...open separate instance for an input camera */
        if ((dev->vfd = open(devname[i], O_RDWR | O_NONBLOCK)) < 0)
        {
            TRACE(ERROR, _x("failed to open device '%s': %m"), devname[i]);
            goto error_dev;
        }

        /* ...check device capabilities */
        if (__imr_check_caps(dev->vfd))
        {
            TRACE(ERROR, _x("capabilities check failed"));
            errno = EINVAL;
            goto error_dev;
        }

        /* ...put negative file-descriptor into poll structure (waiting disabled) */
        imr->pfd[i].fd = -1;
        imr->pfd[i].events = POLLIN;

        TRACE(DEBUG, _b("V4L2 IMR engine #%d initialized (%s)"), i, devname[i]);
    }

    /* ...initialize internal access lock */
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&imr->lock, &attr);
    pthread_mutexattr_destroy(&attr);

    TRACE(INIT, _b("distortion correction module initialized"));

    return imr;

error_dev:
    /* ...close all devices */
    do
    {
        (imr->dev[i].vfd >= 0 ? close(imr->dev[i].vfd) : 0);
    }
    while (i--);

    /* ...destroy devices memory */
    free(imr->dev);

error:
    /* ...destroy poll structure as needed */
    (imr->pfd ? free(imr->pfd) : 0);

    /* ...release module memory */
    free(imr);

    return NULL;
}

/* ...distortion correction engine runtime initialization */
int imr_setup(imr_data_t *imr, int i, int w, int h, int W, int H, int stride, int ifmt, int ofmt, int size)
{
    imr_device_t   *dev = &imr->dev[i];
    int             j;

    /* ...set input buffer length */
    CHK_ERR(dev->input_length = __pixfmt_image_size(CHECK_MULTIPLICITY(stride, 255), h, ifmt), -(errno = EINVAL));

    /* ...output buffer length must be positive (tbd - should I care about that?) */
    CHK_ERR(dev->output_length = __pixfmt_image_size(W, H, ofmt), -(errno = EINVAL));

    /* ...set buffers dimensions */
    dev->w = w, dev->h = h, dev->W = W, dev->H = H;

    /* ...set IMR format */
    CHK_API(imr_set_formats(dev->vfd, w, h, W, H, stride, __pixfmt_gst_to_v4l2(ifmt), __pixfmt_gst_to_v4l2(ofmt)));

    /* ...allocate buffers pool */
    CHK_ERR(dev->pool = calloc(dev->size = size, sizeof(imr_buffer_t)), -(errno = ENOMEM));

    /* ...allocate V4L2 buffers */
    CHK_API(imr_allocate_buffers(dev->vfd, size));

    /* ...create output buffers */
    for (j = 0; j < size; j++)
    {
        imr_buffer_t   *buf = &dev->pool[j];
        GstBuffer      *buffer;
        imr_meta_t     *meta;

        /* ...create output buffer (add some memory? - tbd) */
        CHK_ERR(buf->output = buffer = gst_buffer_new(), -(errno = ENOMEM));

        /* ...allocate metadata */
        CHK_ERR(meta = gst_buffer_add_imr_meta(buffer), -(errno = ENOMEM));
        meta->id = i;
        meta->index = j;
        meta->buf = buf;
        meta->format = ofmt;
        meta->width = W;
        meta->height = H;
        GST_META_FLAG_SET(meta, GST_META_FLAG_POOLED);

        /* ...modify buffer release callback */
        GST_MINI_OBJECT_CAST(buffer)->dispose = __imr_buffer_dispose;

        /* ...use "pool" pointer as a custom data */
        buffer->pool = (void *)imr;

        /* ...notify user about buffer allocation */
        CHK_API(imr->cb->allocate(imr->cdata, i, buffer));
    }

    TRACE(INIT, _b("IMR-#%d: buffer pool initialized"), i);

    return 0;
}

/*******************************************************************************
 * IMR configuration setup
 ******************************************************************************/

/* ...mesh configuration data */
typedef struct imr_cfg
{
    /* ...mesh descriptor */
    struct imr_map_desc     desc;

    /* ...generic display list data */
    display_list_t          dl;

    /* ...memory handle (page-aligned) */
    vsp_mem_t              *mem;

    /* ...any extra data? - tbd */

}   imr_cfg_t;

/* ...tbd - not too good */
#define IMR_OP_RET                      ((0x8D << 24))

/* ...create empty configuration for selected type */
imr_cfg_t * imr_cfg_create(u32 type, size_t size)
{
    imr_cfg_t  *cfg;
    vsp_mem_t  *mem;

    /* ...allocate configuration data structure */
    CHK_ERR(cfg = xcalloc(1, sizeof(*cfg)), NULL);

    /* ...allocate DMA-contiguous memory for a display list */
    if ((cfg->mem = mem = vsp_mem_alloc(size, 0)) == NULL)
    {
        free(cfg);
        return NULL;
    }

    /* ...initialize dynamic array (reserve last word for procedure termination) */
    __dl_create(&cfg->dl, vsp_mem_ptr(mem), vsp_mem_size(mem) - sizeof(u32));

    /* ...save descriptor type */
    cfg->desc.type = type;
    cfg->desc.data = (void *)(uintptr_t)vsp_mem_paddr(mem);

    return cfg;
}

/* ...dump VBO into file (debugging) */
int imr_cfg_dump(imr_cfg_t *cfg, const char *name, int i)
{
    struct imr_map_desc    *desc = &cfg->desc;
    char                    fname[256];
    FILE                   *f;

    /* ...prepare file name */
    snprintf(fname, sizeof(fname), name, i);

    /* ...create file */
    CHK_ERR(f = fopen(fname, "wb"), -errno);

    /* ...dump VBO (ignore errors) */
    if (fwrite(vsp_mem_ptr(cfg->mem), 1, desc->size, f) != desc->size)
    {
        TRACE(ERROR, _b("failed to write file: %m"));
    }

    fclose(f);

    TRACE(1, _b("engine-%d config dumped (%s, %d bytes)"), i, fname, desc->size);

    return 0;
}

/* ...retrieve display list associated with configuration */
display_list_t * imr_cfg_dl(imr_cfg_t *cfg)
{
    return &cfg->dl;
}

/* ...destroy mesh configuration structure */
void imr_cfg_destroy(imr_cfg_t *cfg)
{
    free(cfg);
}

/* ...set mesh confguration */
int imr_cfg_apply(imr_data_t *imr, int i, imr_cfg_t *cfg)
{
    imr_device_t           *dev = &imr->dev[i];
    struct imr_map_desc    *desc = &cfg->desc;
    u32                     size = cfg->dl.n;

    /* ...drop the cameras/alphas for a moment */
    (i < 0 ? size = 0 : 0);

    /* ...put display-list terminator */
    *(u32 *)(cfg->dl.buffer + size) = IMR_OP_RET;

    /* ...update configuration descriptor */
    desc->size = size + 4;

    TRACE(DEBUG, _b("imr-%d: set configuration: type=%08X, paddr=%08X, size=%u"), i, desc->type, (u32)(uintptr_t)desc->data, desc->size);

    /* ...apply mesh configuration; no extra locking is required */
    CHK_API(ioctl(dev->vfd, VIDIOC_IMR_MESH_RAW, desc));

    /* ...reset average processing time calculator */
    //imr_avg_time_reset(dev);

    return 0;
}

/* ...set mesh confguration */
int imr_cfg_set_color(imr_data_t *imr, int i, u32 color)
{
    imr_device_t   *dev = &imr->dev[i];

    /* ...apply mesh configuration; no extra locking is required */
    CHK_API(ioctl(dev->vfd, VIDIOC_IMR_COLOR, &color));

    return 0;
}

/* ...buffer submission */
int imr_engine_push_buffer(imr_data_t *imr, int i, GstBuffer *buffer)
{
    imr_device_t   *dev = &imr->dev[i];
    int             r;

    BUG((u32)i >= (u32)imr->num, _x("invalid transaction: %d"), i);

    /* ...make sure buffer has vsink metadata */
    CHK_ERR(gst_buffer_get_vin_meta(buffer), -(errno = EINVAL));

    /* ...lock internal data access */
    pthread_mutex_lock(&imr->lock);

    /* ...place buffer into pending input queue */
    g_queue_push_tail(&dev->input, gst_buffer_ref(buffer));

    /* ...try to submit buffers if possible */
    r = __submit_buffer(imr, i);

    /* ...release internal access lock */
    pthread_mutex_unlock(&imr->lock);

    /* ...drop the buffer in case of error */
    (r < 0 ? gst_buffer_unref(buffer) : 0);

    return CHK_API(r);
}

/* ...module closing */
void imr_engine_close(imr_data_t *imr)
{
    int     i, j;

    /* ...force thread termination */
    TRACE(DEBUG, _b("signal thread termination"));
    pthread_cancel(imr->thread);
    pthread_join(imr->thread, NULL);
    TRACE(DEBUG, _b("thread joined"));

    /* ...mark engine is disabled */
    imr->active = 0;

    /* ...deallocate all buffers */
    for (i = 0; i < imr->num; i++)
    {
        imr_device_t   *dev = &imr->dev[i];

        /* ...drop all pending input buffers */
        while (!g_queue_is_empty(&dev->input))
        {
            gst_buffer_unref(g_queue_pop_head(&dev->input));
        }

        /* ...deallocate V4L2 buffers */
        imr_destroy_buffers(dev->vfd);

        /* ...clean-up all buffers that haven't been freed */
        for (j = 0; j < dev->size; j++)
        {
            GstBuffer  *buffer = dev->pool[j].output;

            (buffer ? gst_buffer_unref(buffer) : 0);
        }

        /* ...close IMR V4L2 device handle */
        close(dev->vfd);
    }

    /* ...destroy engines data */
    free(imr->dev);

    /* ...destroy poll descriptors */
    free(imr->pfd);

    /* ...destroy module structure */
    free(imr);

    TRACE(INIT, _b("IMR engine destroyed"));
}

/* ...return average processing time in microseconds */
u32 imr_engine_avg_time(imr_data_t *imr, int i)
{
    return imr_avg_time(&imr->dev[i]);
}
