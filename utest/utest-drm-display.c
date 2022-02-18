/*******************************************************************************
 * utest-drm-display.c
 *
 * Display support for unit-test application (generic DRM)
 *
 * Copyright (c) 2014-2017 Cogent Embedded Inc. ALL RIGHTS RESERVED.
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

#define MODULE_TAG                      DISPLAY

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "utest-common.h"
#include "utest-drm-display.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <sys/epoll.h>
#include <gst/video/video-format.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <libudev.h>
#include <rcar_du_drm.h>
#include <libinput.h>

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(EVENT, 1);
TRACE_TAG(DEBUG, 1);

/*******************************************************************************
 * Local typedefs
 ******************************************************************************/

typedef struct __list   __list_t;

struct __list
{
    struct __list  *next, *prev;
};

static inline void __list_init(__list_t *head)
{
    head->next = head->prev = head;
}

static inline int __list_is_empty(__list_t *head)
{
    return (head->next == head);
}

static inline void __list_push_tail(__list_t *head, __list_t *item)
{
    head->prev->next = item, item->prev = head->prev;
    item->next = head, head->prev = item;
}

static inline void __list_push_head(__list_t *head, __list_t *item)
{
    head->next->prev = item, item->next = head->next;
    item->prev = head, head->next = item;
}

static inline void __list_delete(__list_t *item)
{
    item->prev->next = item->next, item->next->prev = item->prev;
    item->prev = item->next = NULL;
}

static inline __list_t * __list_pop_head(__list_t *head)
{
    __list_t   *item = head->next;
    
    if (item != head)
    {
        __list_delete(item);
        return item;
    }
    else
    {
        return NULL;
    }
}

static inline __list_t * __list_pop_tail(__list_t *head)
{
    __list_t   *item = head->prev;
    
    if (item != head)
    {
        __list_delete(item);
        return item;
    }
    else
    {
        return NULL;
    }
}

static inline __list_t * list_null(__list_t *head)
{
    return head;
}

static inline __list_t * list_first(__list_t *head)
{
    return head->next;
}

static inline __list_t * list_last(__list_t *head)
{
    return head->prev;
}

static inline __list_t * list_next(__list_t *head, __list_t *item)
{
    return item->next;
}

static inline __list_t * list_prev(__list_t *head, __list_t *item)
{
    return item->prev;
}

/*******************************************************************************
 * Window data
 ******************************************************************************/

/* ...output device data */
typedef struct output_data
{
    /* ...link item in the outputs list */
    __list_t                    link;

    /* ...output device id */
    int                         id;

    /* ...scanout source id */
    uint32_t                    crtc;

    /* ...DRM connector id */
    uint32_t                    connector_id;

    /* ...current output device width / height */
    u32                         width, height;

    /* ...original DRM mode */
    drmModeModeInfo             mode, *current_mode;

    /* ...supported modes */
    drmModeModeInfo            *modes;

    /* ...number of modes */
    int                         modes_num;

}   output_data_t;

/* ...input device data */
typedef struct input_data
{
    /* ...link item in the input list */
    __list_t                    link;

    /* ...anything with focus? - tbd */

}   input_data_t;

/* ...DRM properties */
enum display_prop
{
    PROP_FB_ID,
    PROP_CRTC_ID,
    PROP_CRTC_X,
    PROP_CRTC_Y,
    PROP_CRTC_W,
    PROP_CRTC_H,
    PROP_SRC_X,
    PROP_SRC_Y,
    PROP_SRC_W,
    PROP_SRC_H,
    PROP_ZPOS,
    PROP_ALPHA,
    PROP_ALPHAPLANE,
    PROP_BLEND,
    PROP_CKEY,
    PROP_CKEY_SET0,
    PROP_CKEY_SET1,
    PROP_NUMBER
};
    
/* ...dispatch loop source */
typedef struct display_source_cb
{
    /* ...processing function */
    int           (*hook)(display_data_t *, struct display_source_cb *, u32 events);
    
}   display_source_cb_t;

/* ...display data */
struct display_data
{
    /* ...udev handle */
	struct udev                *udev;

    /* ...udev monitor handle */
    struct udev_monitor        *udev_monitor;

    /* ...DRM device file handle */
    int                         fd;

    /* ...list of outputs */
    __list_t                    outputs;

    /* ...list of inputs */
    __list_t                    inputs;

    /* ...list of attached windows */
    __list_t                    windows;

    /* ...plane identifiers (tbd) */
    uint32_t                    plane_id[16];

    /* ...plane properties */
    uint32_t                    prop[PROP_NUMBER];
    
    /* ...cairo device associated with display */
    cairo_device_t             *cairo;

    /* ...libinput context */
    struct libinput            *libinput;

    /* ...dispatch loops epoll descriptors */
    int                         o_efd, i_efd;

    /* ...pending display event status */
    int                         pending;

    /* ...input/output dispatch threads handles */
    pthread_t                   i_thread, o_thread;

    /* ...display lock (need that really? - tbd) */
    pthread_mutex_t             lock;
};

/* ...widget data structure */
struct widget_data
{
    /* ...reference to owning window */
    window_data_t              *window;

    /* ...reference to parent widget (not used yet - tbd) */
    widget_data_t              *parent;

    /* ...pointer to the user-provided widget info */
    widget_info_t              *info;

    /* ...widget client data */
    void                       *cdata;

    /* ...actual widget dimensions */
    int                         left, top, width, height;

    /* ...surface update request */
    int                         dirty;
};

/* ...output window data */
struct window_data
{
    /* ...root widget data (must be first) */
    widget_data_t               widget;

    /* ...reference to a display data */
    display_data_t             *display;

    /* ...output device handle */
    output_data_t              *output;

    /* ...modeset initialization flag - tbd */
    int                         modeset;
    
    /* ...frame-buffer identifiers (should be two) */
    uint32_t                    fb_id;

    /* ...list node in display windows list */
    __list_t                    link;

    /* ...cairo device associated with current window context */
    cairo_device_t             *cairo;

    /* ...window information */
    const window_info_t        *info;

    /* ...client data for a callback */
    void                       *cdata;

    /* ...atomic request pointer */
    drmModeAtomicReqPtr         atomic_req;
    
    /* ...internal data access lock */
    pthread_mutex_t             lock;

    /* ...conditional variable for rendering thread */
    pthread_cond_t              wait;

    /* ...window rendering thread */
    pthread_t                   thread;

    /* ...processing flags */
    u32                         flags;

    /* ...frame-rate calculation */
    u32                         fps_ts, fps_acc;
};

/*******************************************************************************
 * Window processing flags
 ******************************************************************************/

/* ...redraw command pending */
#define WINDOW_FLAG_REDRAW              (1 << 0)

/* ...buffer busyness flag */
#define WINDOW_FLAG_BUSY                (1 << 1)

/* ...pending redraw request */
#define WINDOW_FLAG_MODESET             (1 << 2)

/* ...termination command pending */
#define WINDOW_FLAG_TERMINATE           (1 << 3)


/*******************************************************************************
 * Local variables
 ******************************************************************************/

/* ...this should be singleton for now - tbd */
static display_data_t   __display;

/* ...thread key to store current window in TLS */
static pthread_key_t    __key_window;

/*******************************************************************************
 * DRM support
 ******************************************************************************/

static int init_drm(struct udev_device *device)
{
    display_data_t             *display = &__display;
	const char                 *filename, *sysnum;
    int                         id;
    int                         fd;
    drmModePlaneRes            *plane_res;
    drmModeObjectProperties    *props;
    unsigned                    i;

    /* ...get device number */
	CHK_ERR(sysnum = udev_device_get_sysnum(device), -errno);

    /* ...save device identifier (need that? - tbd) */
    id = atoi(sysnum);

    /* ...get device filename */
	CHK_ERR(filename = udev_device_get_devnode(device), -errno);

    /* ...open file descriptor */
    if ((fd = open(filename, O_RDWR)) < 0)
    {
        TRACE(ERROR, _x("failed to open '%s': %m"), filename);
        return -errno;
    }
    else if (drmSetClientCap(fd, /*DRM_CLIENT_CAP_UNIVERSAL_PLANES*/DRM_CLIENT_CAP_ATOMIC, 1) < 0)
    {
        TRACE(ERROR, _x("failed to set caps: %m"));
        close(fd);
        return -errno;
    }
    else if ((plane_res = drmModeGetPlaneResources(fd)) == NULL)
    {
        TRACE(ERROR, _x("failed to get planes resources: %m"));
        close(fd);
        return -errno;
    }

    /* ...go through all planes */
    for (i = 0; i < plane_res->count_planes; i++)
    {
        drmModePlane    *plane = drmModeGetPlane(fd, plane_res->planes[i]);

        /* ...just dump the planes for a moment? - tbd */
        TRACE(INFO, _b("plane-id: %d"), plane->plane_id);

		//sprite->possible_crtcs = plane->possible_crtcs;
		//sprite->plane_id = plane->plane_id;
		//sprite->current = NULL;
		//sprite->next = NULL;
		//sprite->backend = b;
		//sprite->count_formats = plane->count_formats;
		//memcpy(sprite->formats, plane->formats, plane->count_formats * sizeof(plane->formats[0]));

        display->plane_id[i] = plane->plane_id;

        /* ...release plane handle */
		drmModeFreePlane(plane);
	}

    /* ...release plane resources */
	drmModeFreePlaneResources(plane_res);

    /* ...query plane properties */
    if ((props = drmModeObjectGetProperties(fd, display->plane_id[0], DRM_MODE_OBJECT_PLANE)) == NULL)
    {
        TRACE(ERROR, _x("failed to get plane properties: %m"));
        close(fd);
        return -errno;
    }

    for (i = 0; i < props->count_props; i++)
    {
        uint32_t            prop_id = props->props[i];
        drmModePropertyPtr  prop;
        
        if ((prop = drmModeGetProperty(fd, prop_id)) == NULL)
        {
            TRACE(ERROR, _x("failed to get property #%u (of %u): %m"), i, props->count_props);
            continue;
        }

        TRACE(INFO, _b("property #%u: '%s' (id=%u) = %llu"), i, prop->name, prop_id, (unsigned long long)props->prop_values[i]);

        if (strcmp(prop->name, "FB_ID") == 0)
        {
            display->prop[PROP_FB_ID] = prop_id;
        }
        else if (strcmp(prop->name, "CRTC_ID") == 0)
        {
            display->prop[PROP_CRTC_ID] = prop_id;
        }
        else if (strcmp(prop->name, "CRTC_X") == 0)
        {
            display->prop[PROP_CRTC_X] = prop_id;
        }
        else if (strcmp(prop->name, "CRTC_Y") == 0)
        {
            display->prop[PROP_CRTC_Y] = prop_id;
        }
        else if (strcmp(prop->name, "CRTC_W") == 0)
        {
            display->prop[PROP_CRTC_W] = prop_id;
        }
        else if (strcmp(prop->name, "CRTC_H") == 0)
        {
            display->prop[PROP_CRTC_H] = prop_id;
        }
        else if (strcmp(prop->name, "SRC_X") == 0)
        {
            display->prop[PROP_SRC_X] = prop_id;
        }
        else if (strcmp(prop->name, "SRC_Y") == 0)
        {
            display->prop[PROP_SRC_Y] = prop_id;
        }
        else if (strcmp(prop->name, "SRC_W") == 0)
        {
            display->prop[PROP_SRC_W] = prop_id;
        }
        else if (strcmp(prop->name, "SRC_H") == 0)
        {
            display->prop[PROP_SRC_H] = prop_id;
        }
        else if (strcmp(prop->name, "zpos") == 0)
        {
            display->prop[PROP_ZPOS] = prop_id;
        }
        else if (strcmp(prop->name, "alpha") == 0)
        {
            display->prop[PROP_ALPHA] = prop_id;
        }
        else if (strcmp(prop->name, "alphaplane") == 0)
        {
            display->prop[PROP_ALPHAPLANE] = prop_id;
        }
        else if (strcmp(prop->name, "blend") == 0)
        {
            display->prop[PROP_BLEND] = prop_id;
        }
        else if (strcmp(prop->name, "ckey") == 0)
        {
            display->prop[PROP_CKEY] = prop_id;
        }
        else if (strcmp(prop->name, "ckey_set0") == 0)
        {
            display->prop[PROP_CKEY_SET0] = prop_id;
        }
        else if (strcmp(prop->name, "ckey_set1") == 0)
        {
            display->prop[PROP_CKEY_SET1] = prop_id;
        }

        drmModeFreeProperty(prop);
    }

	drmModeFreeObjectProperties(props);

    TRACE(INFO, _b("opened DRM device '%s', card-id: %d"), filename, id);

    if (0 && drmSetMaster(fd) != 0)
    {
        TRACE(ERROR, _x("failed to become master: %m"));
    }

    if (1)
    {
        drm_magic_t magic;
        
        if (drmGetMagic(fd, &magic) == 0)
        {
            TRACE(0, _b("master: %d"), drmAuthMagic(fd, magic));
        }
        else
        {
            TRACE(ERROR, _b("failed to get magic: %m"));
        }
    }

    return fd;
}

static int find_drm(struct udev * udev, const char *seat_id)
{
	struct udev_enumerate  *e;
	struct udev_list_entry *entry;
	int fd = -1;

    /* ...enumerate all DRM devices */
	CHK_ERR(e = udev_enumerate_new(udev), fd);
	udev_enumerate_add_match_subsystem(e, "drm");
	udev_enumerate_add_match_sysname(e, "card[0-9]*");
	if (udev_enumerate_scan_devices(e) < 0) {
		TRACE(ERROR, _x("cannot find default DRM device: %m"));
		goto out;
	}

    /* ...find device that has required SEAT id */
	udev_list_entry_foreach(entry, udev_enumerate_get_list_entry(e))
    {
        const char         *path = udev_list_entry_get_name(entry);
        struct udev_device *device;

        /* ...get device handle */
		if ((device = udev_device_new_from_syspath(udev, path)) == NULL)
        {
            TRACE(ERROR, _x("failed to create device: %m"));
            goto out;
        }

        /* ...check if seat id is specified */
        if (seat_id)
        {
            const char *id = udev_device_get_property_value(device, "ID_SEAT");

            if (!id || strcmp(seat_id, id) != 0)
            {
                TRACE(INFO, _b("skip device '%s': seat-id = %s (expected %s)"), path, id, seat_id);
                udev_device_unref(device);
                continue;
            }
        }

        TRACE(INFO, _b("found DRM device: '%s'"), path);
        fd = init_drm(device);
        if (fd < 0) {
            TRACE(ERROR, _x("failed to initialize DRM device: '%s': %m"), path);
            udev_device_unref(device);
            continue;
        }

        goto out;
    }

    TRACE(ERROR, _b("no DRM found (seat-id = %s)"), seat_id ? seat_id : "(null)");

out:
    /* ...deallocate enumeration structure */
    udev_enumerate_unref(e);

    /* ...return device handle */
    return fd;
}

static int find_crtc(int fd, drmModeRes *resources, drmModeConnector *c)
{
	int             i, j;

    /* ...go through all available encoders */
    for (i = 0; i < c->count_encoders; i++)
    {
        drmModeEncoder *encoder;
        uint32_t        possible_crtcs;

        /* ...get encoder handle */
        CHK_ERR(encoder = drmModeGetEncoder(fd, c->encoders[i]), -errno);

        /* ...get mask of possible scanout-sources */
	possible_crtcs = encoder->possible_crtcs;

        /* ...release encoder handle */
	drmModeFreeEncoder(encoder);

        /* ...go through all scanout-sources (thrash; needs to be revised) */
	for (j = 0; j < resources->count_crtcs; j++)
        {
	    if (possible_crtcs & (1 << j))
            {
                TRACE(INFO, _b("found suitable crtc #%d: %d"), j, resources->crtcs[j]);
                return j;
            }
	}
    }

    return -1;
}

/* ...select output device mode */
static int output_mode_supported(output_data_t *output, int width, int height)
{
    int                 i;
    drmModeModeInfo    *best = NULL;
    
    for (i = 0; i < output->modes_num; i++)
    {
        drmModeModeInfo    *mode = &output->modes[i];

        if (mode->hdisplay == width && mode->vdisplay == height)
        {
            TRACE(INFO, _b("found mode %u*%u, refresh = %u, type=%X, flags=%X"),
                  mode->hdisplay, mode->vdisplay, mode->vrefresh, mode->flags, mode->type);

            //if (mode->vrefresh == 50)   return output->current_mode = mode, 0;
            
            /* ...set mode pointer */
            output->current_mode = mode;

            return 0;
        }
    }

    return -1;
}

/* ...configure output device */
static output_data_t * create_output(display_data_t *display, drmModeRes *resources, drmModeConnector *c, int id)
{
    output_data_t  *output;
    drmModeCrtc    *crtc;
    int             i;

    /* ...find encoder index */
    if ((i = find_crtc(display->fd, resources, c)) < 0)
    {
        TRACE(ERROR, _b("no encoder is found for connector"));
        return (errno = EINVAL, NULL);
    }

    /* ...create output device */
    CHK_ERR(output = calloc(1, sizeof(*output)), (errno = ENOMEM, NULL));

    /* ...set identifier */
    output->id = i;
    output->crtc = resources->crtcs[i];
    output->connector_id = c->connector_id;
    
    /* ...get current connector mode */
    if ((crtc = drmModeGetCrtc(display->fd, output->crtc)) == NULL)
    {
        TRACE(ERROR, _x("failed to get current mode: %m"));
        free(output);
        return NULL;
    }
    else if (!crtc->mode_valid)
    {
        TRACE(ERROR, _x("connector mode invalid"));
    }
    else
    {
        output->width = crtc->mode.hdisplay;
        output->height = crtc->mode.vdisplay;
        memcpy(&output->mode, &crtc->mode, sizeof(output->mode));
        output->current_mode = &output->mode;
    }

    /* ...save supported modes */
    if ((output->modes = malloc(sizeof(drmModeModeInfo) * c->count_modes)) == NULL)
    {
        TRACE(ERROR, _x("failed to allocate memory for %u modes"), c->count_modes);
        free(output);
        return NULL;
    }
    else
    {
        /* ...just make a copy of the modes */
        memcpy(output->modes, c->modes, sizeof(drmModeModeInfo) * c->count_modes);
        output->modes_num = c->count_modes;
    }
    
    /* ...get list of modes */
    for (i = 0; i < (int)c->count_modes; i++)
    {
        drmModeModeInfo    *mode = &c->modes[i];

        TRACE(INFO, _b("mode[%d]: %u*%u@%u"), i, mode->hdisplay, mode->vdisplay, mode->vrefresh);
    }
    
    /* ...put output into list */
    __list_push_tail(&display->outputs, &output->link);

    TRACE(INFO, _b("connector #%d: type=%d, id=%d, subpixel=%d, mode: %d*%d, cid=%u"), output->id, c->connector_type, c->connector_type_id, c->subpixel, output->width, output->height, output->connector_id);

    return output;
}

/* ...create output devices (hmm) */
static int create_outputs(display_data_t *display)
{
	drmModeConnector   *connector;
	drmModeRes         *resources;
    int                 i;
    int                 retval;

    CHK_ERR(resources = drmModeGetResources(display->fd), -errno);

    /* ...process all available connectors */
    for (i = 0; i < resources->count_connectors; i++)
    {
        if ((connector = drmModeGetConnector(display->fd, resources->connectors[i])) == NULL)
        {
            TRACE(ERROR, _x("failed to obtain connector #%d handle: %m"), i);
            retval = -errno;
            goto out;
        }

        TRACE(INFO, _b("connector #%d: id=%d, connection=%d"), i, connector->connector_id, connector->connection);

        /* ...put connector-id into the list of outputs? - tbd */
        create_output(display, resources, connector, i);

        /* ...release connector data */
        drmModeFreeConnector(connector);
    }

    retval = 0;

out:
    drmModeFreeResources(resources);
    return retval;
}

/* ...dumb framebuffer creation */
static inline uint32_t drm_fb_create_dumb(display_data_t *display, unsigned w, unsigned h, void **buf)
{
	struct drm_mode_create_dumb     create_arg;
	//struct drm_mode_destroy_dumb    destroy_arg;
    uint32_t                        fb_id;

	memset(&create_arg, 0, sizeof(create_arg));
	create_arg.bpp = 32;
	create_arg.width = w;
	create_arg.height = h;

	if (drmIoctl(display->fd, DRM_IOCTL_MODE_CREATE_DUMB, &create_arg) < 0)
    {
        TRACE(ERROR, _x("failed to create dumb buffer: %d*%d : %m"), w, h);
        return (uint32_t)-1;
    }

    /* ...shall we save this stuff? - tbd */
	//fb->handle = create_arg.handle;
	//fb->stride = create_arg.pitch;
	//fb->size = create_arg.size;
	//fb->fd = b->drm.fd;

    /* ...create frame-buffer object */
	if (drmModeAddFB(display->fd, w, h, 24, 32, create_arg.pitch, create_arg.handle, &fb_id) != 0)
    {
        TRACE(ERROR, _x("failed to create FB: %m"));
        return (uint32_t)-1;
    }

    /* ...map buffer if requested */
    if (buf)
    {
        struct drm_mode_map_dumb        map_arg;
        
        /* ...map the buffer created (need that? don't know yet) */
        memset(&map_arg, 0, sizeof(map_arg));
        map_arg.handle = create_arg.handle;
        if (drmIoctl(display->fd, DRM_IOCTL_MODE_MAP_DUMB, &map_arg) != 0)
        {
            TRACE(ERROR, _x("failed to map buffer: %m"));
            return (uint32_t)-1;
        }
    
        *buf = mmap(0, create_arg.size, PROT_READ | PROT_WRITE, MAP_SHARED, display->fd, map_arg.offset);
        if (*buf == MAP_FAILED)
        {
            TRACE(ERROR, _x("failed to map framebuffer: %m"));
            return (uint32_t)-1;
        }
    }

    TRACE(INFO, _b("dumb buffer created: %u"), fb_id);
    
	return fb_id;

#if 0
err_export_handle:
	munmap(fb->map, fb->size);
err_add_fb:
	drmModeRmFB(b->drm.fd, fb->fb_id);
err_bo:
	memset(&destroy_arg, 0, sizeof(destroy_arg));
	destroy_arg.handle = create_arg.handle;
	drmIoctl(b->drm.fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy_arg);
err_fb:
	free(fb);
	return NULL;
#endif
}

/* ...vblank handler (when requested and who needs that? - tbd) */
static void vblank_handler(int fd, unsigned int frame, unsigned int sec, unsigned int usec, void *data)
{
	window_data_t  *window = data;

    /* ...signal to the window completion of requested page-flip */
    pthread_mutex_lock(&window->lock);
    window->flags &= ~WINDOW_FLAG_BUSY;
    pthread_cond_signal(&window->wait);
    pthread_mutex_unlock(&window->lock);

#if 0
	struct drm_sprite *s = (struct drm_sprite *)data;
	struct drm_output *output = s->output;
	struct timespec ts;
	uint32_t flags = PRESENTATION_FEEDBACK_KIND_HW_COMPLETION |
			 PRESENTATION_FEEDBACK_KIND_HW_CLOCK;

	drm_output_update_msc(output, frame);
	output->vblank_pending = 0;

	drm_output_release_fb(output, s->current);
	s->current = s->next;
	s->next = NULL;

	if (!output->page_flip_pending) {
		ts.tv_sec = sec;
		ts.tv_nsec = usec * 1000;
		weston_output_finish_frame(&output->base, &ts, flags);
	}
#endif

    TRACE(1, _b("vblank called with data: %p"), data);    
}

/* ...page-flip handler */
static void page_flip_handler(int fd, unsigned int frame, unsigned int sec, unsigned int usec, void *data)
{
	window_data_t  *window = data;

    TRACE(1, _b("page-flip handled for %p"), window);

#if 1
    /* ...signal to the window completion of requested page-flip */
    pthread_mutex_lock(&window->lock);
    window->flags &= ~WINDOW_FLAG_BUSY;
    pthread_cond_signal(&window->wait);
    pthread_mutex_unlock(&window->lock);
#endif
}

/* ...DRM poll-source */
static int drm_event(display_data_t *display, display_source_cb_t *cb, u32 events)
{
	drmEventContext evctx;

    /* ...drop event if no reading flag set */
    if ((events & EPOLLIN) == 0)        return 0;
 
    TRACE(DEBUG, _b("DRM input event"));

	memset(&evctx, 0, sizeof(evctx));
	evctx.version = DRM_EVENT_CONTEXT_VERSION;
	evctx.page_flip_handler = page_flip_handler;
	evctx.vblank_handler = vblank_handler;
	drmHandleEvent(display->fd, &evctx);

    return 0;
}

static display_source_cb_t drm_source = {
    .hook = drm_event,
};

/*******************************************************************************
 * Udev monitor support
 ******************************************************************************/

static int udev_monitor_event(display_data_t *display, display_source_cb_t *cb, u32 events)
{
	struct udev_device *event;

    /* ...drop event if no reading flag set */
    if ((events & EPOLLIN) == 0)        return 0;
    
    /* ...read next event */
    CHK_ERR(event = udev_monitor_receive_device(display->udev_monitor), -errno);

    /* ...check for a hot-plug? - later - tbd */
    TRACE(INFO, _b("new event found"));

    udev_device_unref(event);

    return 0;
}

static display_source_cb_t udev_source = {
    .hook = udev_monitor_event,
};

/*******************************************************************************
 * Internal helpers
 ******************************************************************************/

/*******************************************************************************
 * Display dispatch thread
 ******************************************************************************/

/* ...number of output events expected */
#define DISPLAY_OUTPUT_EVENTS_NUM       2

/* ...number of input events expected */
#define DISPLAY_INPUT_EVENTS_NUM        3

/* ...add handle to a display polling structure */
static inline int __add_poll_source(int efd, int fd, display_source_cb_t *cb)
{
    struct epoll_event  event;
    
    event.events = EPOLLIN;
    event.data.ptr = cb;
    return epoll_ctl(efd, EPOLL_CTL_ADD, fd, &event);
}

/* ...remove handle from a display polling structure */
static inline int __remove_poll_source(int efd, int fd)
{
    return epoll_ctl(efd, EPOLL_CTL_DEL, fd, NULL);
}

/*******************************************************************************
 * Display dispatch threads
 ******************************************************************************/

/* ...display dispatch-output thread */
static void * __output_thread(void *arg)
{
    display_data_t     *display = arg;
    struct epoll_event  event[DISPLAY_OUTPUT_EVENTS_NUM];

    /* ...start waiting loop */
    while (1)
    {
        int     i, r;

        /* ...wait for an event */
        if ((r = epoll_wait(display->o_efd, event, DISPLAY_OUTPUT_EVENTS_NUM, -1)) < 0)
        {
            /* ...ignore soft interruptions */
            if (errno != EINTR)
            {
                TRACE(ERROR, _x("epoll failed: %m"));
                goto error;
            }
        }

        /* ...process all signalled events */
        for (i = 0; i < r; i++)
        {
            display_source_cb_t *dispatch = event[i].data.ptr;

            /* ...invoke event-processing function (ignore result code) */
            if (dispatch)
            {
                dispatch->hook(display, dispatch, event[i].events);
            }
            }
	}

    TRACE(INIT, _b("display dispatch-output thread terminated"));
    return NULL;

error:
    return (void *)(intptr_t)-errno;
}

/* ...display input dispatch thread */
static void * __input_thread(void *arg)
{
    display_data_t     *display = arg;
    struct epoll_event  event[DISPLAY_INPUT_EVENTS_NUM];

    /* ...start waiting loop */
    while (1)
    {
        int     i, r;

        /* ...wait for an event */
        if ((r = epoll_wait(display->i_efd, event, DISPLAY_INPUT_EVENTS_NUM, -1)) < 0)
        {
            /* ...ignore soft interruptions */
            if (errno != EINTR)
            {
                TRACE(ERROR, _x("epoll failed: %m"));
                goto error;
            }
        }

        /* ...process all signalled events */
        for (i = 0; i < r; i++)
        {
            display_source_cb_t *dispatch = event[i].data.ptr;

            /* ...invoke event-processing function (ignore result code) */
            dispatch->hook(display, dispatch, event[i].events);
        }
    }

    TRACE(INIT, _b("display dispatch-input thread terminated"));
    return NULL;

error:
    return (void *)(intptr_t)-errno;
}

/* ...get output device by number */
static output_data_t *display_get_output(display_data_t *display, int n)
{
    __list_t       *list = &display->outputs, *item;

    /* ...traverse available outputs list */
    for (item = list_first(list); item != list_null(list); item = list_next(list, item))
    {
        output_data_t  *output = container_of(item, output_data_t, link);

        /* ...check for identifier */
        if (output->id == n)    return  output;
    }
    
    /* ...not found */
    return NULL;
}

/*******************************************************************************
 * Spacenav 3D-joystick support
 ******************************************************************************/

/* ...spacenav input event processing */
static int input_spacenav_event(display_data_t *display, display_source_cb_t *cb, u32 events)
{
    __list_t       *list = &display->windows, *item;
    widget_event_t  event;
    spnav_event     e;

    /* ...drop event if no reading flag set */
    if ((events & EPOLLIN) == 0)        return 0;
    
    /* ...retrieve poll event */
    if (CHK_API(spnav_poll_event(&e)) == 0)     return 0;

    /* ...preare widget event */
    event.type = WIDGET_EVENT_SPNAV;
    event.spnav.e = &e;

    /* ...pass to all windows */
    for (item = list_first(list); item != list_null(list); item = list_next(list, item))
    {
        window_data_t  *window = container_of(item, window_data_t, link);
        widget_data_t  *widget = &window->widget;
        widget_info_t  *info = widget->info;

        /* ...ignore window if no input event is registered */
        if (!info || !info->event)      continue;

        /* ...pass event to root widget (only one consumer?) */
        if (info->event(widget, window->cdata, &event) != NULL)   break;
    }

    return 0;
}

static display_source_cb_t spacenav_source = {
    .hook = input_spacenav_event,
};

/* ...spacenav event initializer */
static inline int input_spacenav_init(display_data_t *display)
{
    int     fd;
    
    /* ...open spacenav device (do not die if not found) */
    if (spnav_open() < 0)
    {
        TRACE(INIT, _b("spacenavd daemon is not running"));
        return 0;
    }
    
    if ((fd = spnav_fd()) < 0)
    {
        TRACE(ERROR, _x("failed to open spacenav connection: %m"));
        goto error;
    }
    
    /* ...add file-descriptor as display poll source */
    if (__add_poll_source(display->i_efd, fd, &spacenav_source) < 0)
    {
        TRACE(ERROR, _x("failed to add poll source: %m"));
        goto error;
    }
    
    TRACE(INIT, _b("spacenav input added"));
    
    return 0;

error:
    /* ...destroy connection to a server */
    spnav_close();

    return -errno;
}

/*******************************************************************************
 * Libinput interface
 ******************************************************************************/

static int __open_restricted(const char *path, int flags, void *data)
{
	int fd;
	fd = open(path, flags);
	return fd < 0 ? -errno : fd;
}

static void __close_restricted(int fd, void *data)
{
	close(fd);
}

const struct libinput_interface libinput_interface = {
	.open_restricted = __open_restricted,
	.close_restricted = __close_restricted,
};



/* ...touch event processing */
static void libinput_keyboard_event(display_data_t *display, u32 state, u32 key)
{
    widget_event_t  event;
    __list_t       *list = &display->windows, *item;

    TRACE(DEBUG, _x("Keyboard state = %d key = %d"), state, key);

    event.type = WIDGET_EVENT_KEY;
    event.key.type = WIDGET_EVENT_KEY_PRESS;
    event.key.code = key;
    event.key.state = state;
   
    /* ...go through all windows (respect SEAT somehow probably? - tbd) */
	for (item = list_first(list); item != list_null(list); item = list_next(list, item))
	{
		window_data_t  *window = container_of(item, window_data_t, link);
		widget_data_t  *widget = &window->widget;
		widget_info_t  *info = widget->info;

		/* ...ignore window if no input event is registered */
		if (!info || !info->event)      continue;

		/* ...pass event to root widget (only one consumer?) */
		if (info->event(widget, window->cdata, &event) != NULL)   break;
	}
}

/* ...libinput input processing */
static int libinput_input_event(display_data_t *display, display_source_cb_t *cb, u32 events)
{
    struct libinput         *ctx = display->libinput;
    struct libinput_event   *ev;

    /* ...dispatch all collected events */
    CHK_ERR(libinput_dispatch(ctx) == 0, -1);

    /* ...process all events */
    while ((ev = libinput_get_event(ctx)) != NULL)
    {
        enum libinput_event_type type = libinput_event_get_type(ev);
        struct libinput_device *dev = libinput_event_get_device(ev);

        switch (type)
        {
        case LIBINPUT_EVENT_DEVICE_ADDED:
            TRACE(INIT, _b("new device added"));
            break;

        case LIBINPUT_EVENT_DEVICE_REMOVED:
            TRACE(INIT, _b("device removed"));
            break;
        case LIBINPUT_EVENT_KEYBOARD_KEY:
            {
                struct libinput_event_keyboard *keyboard = libinput_event_get_keyboard_event(ev);
                uint32_t key =  libinput_event_keyboard_get_key(keyboard);
                uint32_t state = libinput_event_keyboard_get_key_state(keyboard);
                libinput_keyboard_event(display, state, key);
                break;
            }
            break;
        case LIBINPUT_EVENT_TOUCH_DOWN:
        case LIBINPUT_EVENT_TOUCH_MOTION:
        case LIBINPUT_EVENT_TOUCH_UP:
        {
            break;
        }
        default:
            /* ...unknown event; ignore */
            break;
        }

        libinput_event_destroy(ev);
    }

    return 0;
}

/* ...display source callback structure */
static display_source_cb_t libinput_source = {
    .hook = libinput_input_event,
};

/* ...libinput input devices initialization */
static int input_libinput_init(display_data_t *display)
{
    struct libinput    *ctx;
    int                 fd;

    /* ...create libinput context */
	CHK_ERR(ctx = libinput_udev_create_context(&libinput_interface, display, display->udev), -1);

    /* ...enable context - add default seat? - tbd */
    CHK_API(libinput_udev_assign_seat(ctx, "seat0"));

    /* ...get device node descriptor */
    fd = libinput_get_fd(ctx);

    /* ...add input to the poll-source */
	if (__add_poll_source(display->i_efd, fd, &libinput_source) < 0)
	{
		TRACE(ERROR, _x("failed to add poll source: %m"));
		goto err;
	}

    /* ...save input-device client data */
	display->libinput = ctx;

    TRACE(1, _b("libinput interface initialized: %d"), fd);

    return 0;

err:
    /* ...destroy libinput context */
    libinput_unref(ctx);

    return -1;
}

/*******************************************************************************
 * stdin interface
 ******************************************************************************/

/* ...stdin event processing */
static void stdin_keyboard_event(display_data_t *display, char in)
{
    widget_event_t event;
    __list_t *list = &display->windows, *item;

    /* enter only */
    if (in != '\n') {
        return;
    }

    TRACE(DEBUG, _x("stdin creating event"));
    event.type = WIDGET_EVENT_KEY;
    event.key.type = WIDGET_EVENT_KEY_PRESS;
    event.key.code = KEY_Z;
    event.key.state = LIBINPUT_KEY_STATE_PRESSED;

    /* ...go through all windows */
    for (item = list_first(list); item != list_null(list); item = list_next(list, item))
    {
        window_data_t  *window = container_of(item, window_data_t, link);
        widget_data_t  *widget = &window->widget;
        widget_info_t  *info = widget->info;

        /* ...ignore window if no input event is registered */
        if (!info || !info->event)      continue;

        /* ...pass event to root widget (only one consumer?) */
        if (info->event(widget, window->cdata, &event) != NULL)   break;
    }
}

/* ...stdin event processing */
static int stdin_input_event(display_data_t *display, display_source_cb_t *cb, u32 events)
{
    int in;

    /* ...drop event if no reading flag set */
    if ((events & EPOLLIN) == 0) {
        return 0;
    }

    if ((in = getchar()) == EOF) {
        TRACE(ERROR, _x("getchar failed: %d"), errno);
    } else {
        TRACE(DEBUG, _x("stdin char: %c"), in);
        stdin_keyboard_event(display, in);
    }

    return 0;
}

/* ...display source callback structure */
static display_source_cb_t stdin_source = {
    .hook = stdin_input_event,
};

/* ...libinput input devices initialization */
static int stdin_input_init(display_data_t *display)
{
    /* ...add input to the poll-source */
    if (__add_poll_source(display->i_efd, STDIN_FILENO, &stdin_source) < 0)
    {
        TRACE(ERROR, _x("failed to add poll source: %m"));
        goto err;
    }

    TRACE(1, _b("stdin interface initialized"));

    return 0;

err:
    return -1;
}

/*******************************************************************************
 * Window support
 ******************************************************************************/

/* ...window rendering thread */
static void * window_thread(void *arg)
{
    window_data_t      *window = arg;
    display_data_t     *display = window->display;

    /* ...register current window inside TLS */
    pthread_setspecific(__key_window, window);

    /* ...enter executive loop */
    while (1)
    {
        /* ...serialize access to window state */
        pthread_mutex_lock(&window->lock);

        /* ...wait for a drawing command from an application */
        while (!(window->flags & (WINDOW_FLAG_REDRAW | WINDOW_FLAG_TERMINATE)))
        {
            TRACE(0, _b("window[%p] wait"), window);
            pthread_cond_wait(&window->wait, &window->lock);
        }

        TRACE(0, _b("window[%p] redraw (flags=%X)"), window, window->flags);

        /* ...break processing thread if requested to do that */
        if (window->flags & WINDOW_FLAG_TERMINATE)
        {
            pthread_mutex_unlock(&window->lock);
            break;
        }

        /* ...clear window drawing schedule flag */
        window->flags &= ~WINDOW_FLAG_REDRAW;

        /* ...release window access lock */
        pthread_mutex_unlock(&window->lock);

        /* ...invoke user-supplied hook */
        window->info->redraw(display, window->cdata);
    }

    TRACE(INIT, _b("window[%p] thread terminated"), window);

    return NULL;
}

/*******************************************************************************
 * Basic widgets support
 ******************************************************************************/

/* ...internal widget initialization function */
static int __widget_init(widget_data_t *widget, window_data_t *window, int W, int H, widget_info_t *info, void *cdata)
{
    int                 w, h;

    /* ...set user-supplied data */
    widget->info = info, widget->cdata = cdata;

    /* ...set pointer to the owning window */
    widget->window = window;

    /* ...if width/height are not specified, take them from window */
    widget->width = w = (info && info->width ? info->width : W);
    widget->height = h = (info && info->height ? info->height : H);
    widget->top = (info ? info->top : 0);
    widget->left = (info ? info->left : 0);

    /* ...initialize widget controls as needed */
    if (info && info->init)
    {
        if (info->init(widget, cdata) < 0)
        {
            TRACE(ERROR, _x("widget initialization failed: %m"));
            goto error;
        }
     
        /* ...mark widget is dirty */
        widget->dirty = 1;
    }
    else
    {
        /* ...clear dirty flag */
        widget->dirty = 0;
    }

    TRACE(INIT, _b("widget [%p] initialized"), widget);

    return 0;

error:
    return -1;    
}

/* ...return current widget width */
int widget_get_width(widget_data_t *widget)
{
    return widget->width;
}

/* ...return current widget height */
int widget_get_height(widget_data_t *widget)
{
    return widget->height;
}

/* ...return left point */
int widget_get_left(widget_data_t *widget)
{
    return widget->left;
}

/* ...return top point */
int widget_get_top(widget_data_t *widget)
{
    return widget->top;
}

/* ...get parent window root widget */
widget_data_t * widget_get_parent(widget_data_t *widget)
{
    return &widget->window->widget;
}

/*******************************************************************************
 * Entry points
 ******************************************************************************/

/* ...create native window */
window_data_t * window_create(display_data_t *display, window_info_t *info, widget_info_t *info2, void *cdata)
{
    int                 width = info->width;
    int                 height = info->height;
    output_data_t      *output;
    window_data_t      *window;
    pthread_attr_t      attr;
    int                 r;

    /* ...make sure we have a valid output device */
    if ((output = display_get_output(display, info->output)) == NULL)
    {
        TRACE(ERROR, _b("invalid output device number: %u"), info->output);
        errno = EINVAL;
        return NULL;
    }

    /* ...if width/height are not specified, use output device dimensions */
    (!width ? width = output->width : 0), (!height ? height = output->height : 0);

    /* ...check if the output mode needs to be changed */
    if ((uint32_t)width != output->width || (uint32_t)height != output->width)
    {
        if (output_mode_supported(output, width, height) < 0)
        {
            TRACE(ERROR, _b("output mode %d*%d not supported"), width, height);
            errno = EINVAL;
            return NULL;
        }
    }

    /* ...allocate a window data */
    if ((window = calloc(1, sizeof(*window))) == NULL)
    {
        TRACE(ERROR, _x("failed to allocate memory"));
        errno = ENOMEM;
        return NULL;
    }

    /* ...save output device handle */
    window->output = output;

    /* ...initialize window data access lock */
    pthread_mutex_init(&window->lock, NULL);

    /* ...initialize conditional variable for communication with rendering thread */
    pthread_cond_init(&window->wait, NULL);

    /* ...save display handle */
    window->display = display;

    /* ...save window info data */
    window->info = info, window->cdata = cdata;

    /* ...clear window flags */
    window->flags = 0;

    /* ...reset frame-rate calculator */
    window_frame_rate_reset(window);

    /* ...set window mode */
    if (1 && (window->flags & WINDOW_FLAG_MODESET) == 0)
    {
        drmModeCrtcPtr  crtc;
        
        if (drmModeSetCrtc(display->fd, output->crtc, -1, 0, 0, &output->connector_id, 1, output->current_mode) < 0)
        {
            TRACE(ERROR, _x("mode-set failed: %m"));
        }

        if ((crtc = drmModeGetCrtc(display->fd, output->crtc)) != NULL)
        {
            TRACE(INIT, _b("crtc: %u * %u, mode-valid: %d (%u*%u@%u)"), crtc->width, crtc->height, crtc->mode_valid, crtc->mode.hdisplay, crtc->mode.vdisplay, crtc->mode.vrefresh);
        }

        /* ...mark mode is set */
        window->flags |= WINDOW_FLAG_MODESET;
    }

#if 0    
    /* ...create dumb framebuffer (need that? don't know) */
    void *buf = NULL;

    window->fb_id = drm_fb_create_dumb(display, width, height, &buf);

    memset(buf, 0xAA, width * height * 2);
#endif

    /* ...allocate atomic request structure for compositing */
    if ((window->atomic_req = drmModeAtomicAlloc()) == NULL)
    {
        TRACE(ERROR, _x("failed to allocate atomic request: %m"));
        goto error;
    }

    /* ...initialize root widget data */
    if (__widget_init(&window->widget, window, width, height, info2, cdata) < 0)
    {
        TRACE(INIT, _b("widget initialization failed: %m"));
        goto error;
    }

    /* ...initialize thread attributes (joinable, default stack size) */
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    /* ...create rendering thread */
    r = pthread_create(&window->thread, &attr, window_thread, window);
    pthread_attr_destroy(&attr);
    if (r != 0)
    {
        TRACE(ERROR, _x("thread creation failed: %m"));
        goto error;
    }

    /* ...add window to global display list */
    __list_push_tail(&display->windows, &window->link);

    TRACE(INFO, _b("window created: %p, %u*%u, output: %u"), window, width, height, info->output);

    return window;

error:
    /* ...destroy window memory */
    free(window);
    return NULL;
}

/* ...destroy a window */
void window_destroy(window_data_t *window)
{
    //display_data_t         *display = window->display;
    const window_info_t    *info = window->info;
    const widget_info_t    *info2 = window->widget.info;

    /* ...terminate window rendering thread */
    pthread_mutex_lock(&window->lock);
    window->flags |= WINDOW_FLAG_TERMINATE;
    pthread_cond_signal(&window->wait);
    pthread_mutex_unlock(&window->lock);

    /* ...wait until thread completes */
    pthread_join(window->thread, NULL);

    TRACE(DEBUG, _b("window[%p] thread joined"), window);

    /* ...remove window from global display list */
    __list_delete(&window->link);

    /* ...invoke custom widget destructor function as needed */
    (info2 && info2->destroy ? info2->destroy(&window->widget, window->cdata) : 0);

    /* ...invoke custom window destructor function as needed */
    (info && info->destroy ? info->destroy(window, window->cdata) : 0);

    /* ...destroy window lock */
    pthread_mutex_destroy(&window->lock);

    /* ...destroy rendering thread conditional variable */
    pthread_cond_destroy(&window->wait);

    /* ...destroy object */
    free(window);

    TRACE(INFO, _b("window[%p] destroyed"), window);
}

/* ...return current window width */
int window_get_width(window_data_t *window)
{
    return window->widget.width;
}

/* ...return current window height */
int window_get_height(window_data_t *window)
{
    return window->widget.height;
}

/* ...schedule redrawal of the window */
void window_schedule_redraw(window_data_t *window)
{
    /* ...acquire window lock */
    pthread_mutex_lock(&window->lock);

    TRACE(0, _b("schedule window[%p] redraw: %X"), window, window->flags);

    /* ...check if we don't have a flag already */
    if ((window->flags & (WINDOW_FLAG_REDRAW | WINDOW_FLAG_BUSY)) == 0)
    {
        /* ...schedule drawing operation */
        window->flags ^= WINDOW_FLAG_REDRAW;

        /* ...kick waiting thread */
        pthread_cond_signal(&window->wait);
    }
    else
    {
        /* ...just put sticky redraw bit */
        window->flags |= WINDOW_FLAG_REDRAW;
    }

    /* ...release window access lock */
    pthread_mutex_unlock(&window->lock);
}

/* ...submit window to a renderer */
void window_draw(window_data_t *window)
{
    display_data_t         *display = window->display;
    output_data_t          *output = window->output;
    u32                     t0, t1;

    t0 = __get_time_usec();

    /* ...mark window is busy */
    pthread_mutex_lock(&window->lock);

    /* ...submit atomic request (modeset allow? - tbd) */
    if (drmModeAtomicCommit(display->fd, window->atomic_req, 0*DRM_MODE_PAGE_FLIP_EVENT | 0*DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_ATOMIC_ALLOW_MODESET, window) < 0)
    {
        TRACE(ERROR, _x("commit failed: %m, %d"), display->fd);
    }

    /* ...reset cursor once drawing operation has been submitted */
    drmModeAtomicSetCursor(window->atomic_req, 0);

    pthread_mutex_unlock(&window->lock);

    t1 = __get_time_usec();

    TRACE(0, _b("swap[%p]: %u"), window, t1 - t0);
}

/*******************************************************************************
 * Display module initialization
 ******************************************************************************/

/* ...create display data */
display_data_t * display_create(const int support_stdin)
{
    display_data_t     *display = &__display;
    pthread_attr_t      attr;
    int                 r;
    
    /* ...reset display data */
    memset(display, 0, sizeof(*display));

    /* ...create udev handle */
    if ((display->udev = udev_new()) == NULL)
    {
        TRACE(ERROR, _x("failed to connect to udev: %m"));
        goto error;
    }

    /* ...basic initialization of DRM */
    if ((display->fd = find_drm(display->udev, NULL)) < 0)
    {
        TRACE(ERROR, _x("failed to initialize DRM: %m"));
        goto error;
    }

    /* ...initialize inputs/outputs/windows lists */
    __list_init(&display->outputs);
    __list_init(&display->inputs);
    __list_init(&display->windows);

    /* ...create a display command/response lock */
    pthread_mutex_init(&display->lock, NULL);

    /* ...create polling structures for input/output dispatch threads */
    if ((display->i_efd = epoll_create(DISPLAY_INPUT_EVENTS_NUM)) < 0)
    {
        TRACE(ERROR, _x("failed to create epoll: %m"));
        goto error;
    }

    if ((display->o_efd = epoll_create(DISPLAY_OUTPUT_EVENTS_NUM)) < 0)
    {
        TRACE(ERROR, _x("failed to create epoll: %m"));
        goto error;
    }

    /* ...add DRM input events poll-source */
    if (__add_poll_source(display->o_efd, display->fd, &drm_source) < 0)
    {
        TRACE(ERROR, _x("failed to add DRM poll-source: %m"));
        goto error;
    }

    /* ...create udev monitor */
    if ((display->udev_monitor = udev_monitor_new_from_netlink(display->udev, "udev")) == NULL)
    {
        TRACE(ERROR, _x("failed to create udev monitor: %m"));
        goto error;
    }

    /* ...add search for an output */
    udev_monitor_filter_add_match_subsystem_devtype(display->udev_monitor, "drm", NULL);

    /* ...add search for an input device */
    udev_monitor_filter_add_match_subsystem_devtype(display->udev_monitor, "input", NULL);

    /* ...add poll source for udev monitor */
    if (__add_poll_source(display->i_efd, udev_monitor_get_fd(display->udev_monitor), &udev_source) < 0)
    {
        TRACE(ERROR, _x("failed to add udev monitor poll-source: %m"));
        goto error;
    }
    else if (udev_monitor_enable_receiving(display->udev_monitor) < 0)
    {
        TRACE(ERROR, _x("failed to enable udev-monitor: %m"));
        errno = -EBADFD;
        goto error;
    }

    /* ...create global TLS key for storing current window */
    if (pthread_key_create(&__key_window, NULL) < 0)
    {
        TRACE(ERROR, _x("failed to create TLS key: %m"));
        goto error;
    }

    if (create_outputs(display) < 0)
    {
        TRACE(ERROR, _x("failed to enumerate outputs"));
        goto error;
    }

    /* ...initialize extra input devices */
    input_spacenav_init(display);

    /* ...initialize libinit input devices */
    input_libinput_init(display);

    /* ...initialuze stdin input */
    if (support_stdin)
    {
        stdin_input_init(display);
    }
    /* ...create dispatch threads (joinable, default stack size) */
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    if (pthread_create(&display->i_thread, &attr, __input_thread, display) != 0)
    {
        TRACE(ERROR, _x("failed to create input thread: %m"));
        goto error;
    }

    if (pthread_create(&display->o_thread, &attr, __output_thread, display) != 0)
    {
        TRACE(ERROR, _x("failed to create output thread: %m"));
        goto error;
    }

    pthread_attr_destroy(&attr);

    /* ...wait until display thread starts? */
    TRACE(INIT, _b("DRM display interface initialized"));

    /* ...not sure if it is good idea to start threads here - tbd */
    return display;

error:
    return NULL;
}

/*******************************************************************************
 * Textures handling
 ******************************************************************************/

/* ...calculate cropping and viewport parameters for a texture */
void texture_set_view(texture_view_t *vcoord, int x0, int y0, int x1, int y1)
{
    (*vcoord)[0] = x0;
    (*vcoord)[1] = y0;
    (*vcoord)[2] = x1;
    (*vcoord)[3] = y1;
}

/* ...set texture cropping data */
void texture_set_crop(texture_crop_t *tcoord, int x0, int y0, int x1, int y1)
{
    (*tcoord)[0] = x0;
    (*tcoord)[1] = y0;
    (*tcoord)[2] = x1;
    (*tcoord)[3] = y1;
}

static inline int texture_view_x0(texture_view_t *v)
{
    return (*v)[0];    
}

static inline int texture_view_y0(texture_view_t *v)
{
    return (*v)[1];
}

static inline int texture_view_x1(texture_view_t *v)
{
    return (*v)[2];
}

static inline int texture_view_y1(texture_view_t *v)
{
    return (*v)[3];
}

static inline float texture_view_width(texture_view_t *v)
{
    return ((*v)[2] - (*v)[0]);    
}

static inline float texture_view_height(texture_view_t *v)
{
    return ((*v)[3] - (*v)[1]);
}

/* ...translate V4L2 pixel-format into DRM-format */
static inline uint32_t __pixfmt_gst_to_drm(int format, int *n)
{
    switch (format)
    {
    case GST_VIDEO_FORMAT_ARGB:     return *n = 1, DRM_FORMAT_ARGB8888;
    case GST_VIDEO_FORMAT_RGB16:    return *n = 1, DRM_FORMAT_RGB565;
    case GST_VIDEO_FORMAT_NV16:     return *n = 1, DRM_FORMAT_NV16;
    case GST_VIDEO_FORMAT_NV12:     return *n = 2, DRM_FORMAT_NV12;
    case GST_VIDEO_FORMAT_UYVY:     return *n = 1, DRM_FORMAT_UYVY;
    case GST_VIDEO_FORMAT_YUY2:     return *n = 1, DRM_FORMAT_YUYV;
    case GST_VIDEO_FORMAT_YVYU:     return *n = 1, DRM_FORMAT_YVYU;
    case GST_VIDEO_FORMAT_GRAY8:    return *n = 1, DRM_FORMAT_NV16; /* output as NV16 with UV fixed to 0x80 (blank) */
    default:                        return TRACE(ERROR, _x("unsupported format: %d"), format), 0;
    }
}

/* ...texture creation for given set of DMA-file-descriptors */
texture_data_t * texture_create(int w, int h, int format, int *dmafd, unsigned *offset, unsigned *stride)
{
    display_data_t                 *display = &__display;
    uint32_t                        handle[4];
    texture_data_t                 *texture;
    int                             n, i;
    uint32_t                        fmt;

    /* ...map format to the internal value */
    CHK_ERR((fmt = __pixfmt_gst_to_drm(format, &n)) > 0, (errno = EINVAL, NULL));

    /* ...allocate texture data */
    CHK_ERR(texture = malloc(sizeof(*texture)), (errno = ENOMEM, NULL));

	if (fmt == DRM_FORMAT_NV16)
	{
		dmafd[1] = dmafd[0];
		offset[1] = offset[0] + w * h;
		stride[0] = w;
		stride[1] = w;
		n = 2;
	}

    /* ...map buffer planes */
    for (i = 0; i < n; i++)
    {
        if (drmPrimeFDToHandle(display->fd, dmafd[i], &handle[i]) != 0)
        {
            TRACE(ERROR, _x("failed to create buffer handle for dma-fd %d: %m"), dmafd[i]);
            goto error;
        }
    }

    if (drmModeAddFB2(display->fd, w, h, fmt, handle, stride, offset, &texture->fb_id, 0) != 0)
    {
        TRACE(ERROR, _x("failed to create FB2: %m"));
        goto error;
    }

    TRACE(INFO, _b("buffer allocated: %d*%d@%d [fd=%d/%d/%d, %u]"), w, h, format, dmafd[0], dmafd[1], dmafd[2], texture->fb_id);
    goto out;

error:
    /* ...buffer creation failed; destroy texture object */
    free(texture), texture = NULL;

    /* ...pass through */
out:
    /* ...close handles */
    while (--i)
    {
        struct drm_gem_close gem_close = {.handle = handle[i]};

        drmIoctl(display->fd, DRM_IOCTL_GEM_CLOSE, &gem_close);
	}

    return texture;
}

/* ...destroy texture data */
void texture_destroy(texture_data_t *texture)
{
    display_data_t *display = &__display;

    /* ...destroy framebuffer object */
    (texture->fb_id ? drmModeRmFB(display->fd, texture->fb_id) : 0);
    
    /* ...destroy texture structure */
    free(texture);
}

/* ...make a screenshot to a texture */
int window_screenshot(window_data_t *window, texture_data_t *texture)
{
    display_data_t      *display = &__display;
    output_data_t       *output = (window ? window->output : display_get_output(display, 0));
    
    struct rcar_du_screen_shot  ss = {
        .buff = texture->fb_id,
        .crtc_id = output->crtc,
    };

    TRACE(INIT, _b("make a screenshot"));
 
    CHK_API(drmIoctl(display->fd, DRM_IOCTL_RCAR_DU_SCRSHOT, &ss));

    TRACE(INIT, _b("screenshoot created"));
    
    return 0;
}

/*******************************************************************************
 * Planes support
 ******************************************************************************/

/* ...setup DRM overlay plane */
int plane_setup(window_data_t *window, int i, texture_data_t *texture, texture_data_t *alpha, u8 tr, texture_view_t *view, texture_view_t *crop, u32 *ckey, u32 blend)
{
    display_data_t         *display = window->display;
    output_data_t          *output = window->output;
    int                     fd = display->fd;
    uint32_t                plane_id;
    uint32_t                x0, y0, w, h;
    uint32_t                X0, Y0, W, H;
    drmModeAtomicReqPtr     req;
    u32                     t0, t1, t2, t3, t4;
    
    t0 = __get_time_usec();

    /* ...view-port setup */
    if (!view)
    {
        x0 = 0, y0 = 0, w = window_get_width(window), h = window_get_height(window);
    }
    else
    {
        int        *p = *view;
        int         i;

        x0 = texture_view_x0(view), y0 = texture_view_y0(view);
        w = texture_view_width(view), h = texture_view_height(view);

        TRACE(0, _b("viewport: %u/%u/%u/%u"), x0, y0, w, h);
    }

    /* ...crop-region setup */
    if (!crop)
    {
        X0 = x0 << 16, Y0 = y0 << 16, W = w << 16, H = h << 16;
    }
    else
    {
        int        *p = *crop;

        X0 = p[0] << 16, Y0 = p[1] << 16, W = (p[2] - p[0]) << 16, H = (p[3] - p[1]) << 16;

        TRACE(0, _b("crop: %u/%u/%u/%u"), X0, Y0, W, H);
    }

    /* ...get plane identifier */
    CHK_ERR(plane_id = display->plane_id[i], -(errno = ENODEV));

    t1 = __get_time_usec();

    /* ...get atomic request structure */
    req = window->atomic_req;

    t2 = __get_time_usec();

    if (texture)
    {
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CRTC_ID], output->crtc));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_FB_ID], texture->fb_id));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CRTC_X], x0));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CRTC_Y], y0));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CRTC_W], w));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CRTC_H], h));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_SRC_X], X0));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_SRC_Y], Y0));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_SRC_W], W));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_SRC_H], H));
    }
    else
    {
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CRTC_ID], 0));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_FB_ID], 0));
    }

    /* ...set apha-plane property */
    CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_ALPHAPLANE], (alpha ? alpha->fb_id : 0)));
    
    /* ...set global alpha value */
    CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_ALPHA], tr));
    
    /* ...set blending property */
    CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_BLEND], blend));

    /* ...set color key if needed */
    if (ckey)
    {
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CKEY], ckey[0]));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CKEY_SET0], ckey[1]));
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CKEY_SET1], ckey[2]));
    }
    else
    {
        CHK_API(drmModeAtomicAddProperty(req, plane_id, display->prop[PROP_CKEY], 0));
    }

    t3 = __get_time_usec();

    TRACE(0, _b("plane #%d setup: %u/%u/%u (total=%u)"), i, (u32)(t1 - t0), (u32)(t2 - t1),
          (u32)(t3 - t2), (u32)(t3 - t0));

    return 0;
}

/*******************************************************************************
 * Auxiliary frame-rate calculation functions
 ******************************************************************************/

/* ...reset FPS calculator */
void window_frame_rate_reset(window_data_t *window)
{
    /* ...reset accumulator and timestamp */
    window->fps_acc = 0, window->fps_ts = 0;
}

/* ...update FPS calculator */
float window_frame_rate_update(window_data_t *window)
{
    u32     ts_0, ts_1, delta, acc;
    float   fps;

    /* ...get current timestamp for a window frame-rate calculation */
    delta = (ts_1 = __get_time_usec()) - (ts_0 = window->fps_ts);

    /* ...check if accumulator is initialized */
    if ((acc = window->fps_acc) == 0)
    {
        if (ts_0 != 0)
        {
            /* ...initialize accumulator */
            acc = delta << 4;
        }
    }
    else
    {
        /* ...accumulator is setup already; do exponential averaging */
        acc += delta - ((acc + 8) >> 4);
    }

    /* ...calculate current frame-rate */
    if ((fps = (acc ? 1e+06 / ((acc + 8) >> 4) : 0)) != 0)
    {
        TRACE(DEBUG, _b("delta: %u, acc: %u, fps: %f"), delta, acc, fps);
    }

    /* ...update timestamp and accumulator values */
    window->fps_acc = acc, window->fps_ts = ts_1;

    return fps;
}
