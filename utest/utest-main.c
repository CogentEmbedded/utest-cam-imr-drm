/*******************************************************************************
 * utest-main.c
 *
 * Surround-view unit-test main function
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

#define MODULE_TAG                      MAIN

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "utest-common.h"
#include "utest-app.h"
#include <getopt.h>
#include <linux/videodev2.h>

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 0);

/*******************************************************************************
 * Global variables definitions
 ******************************************************************************/

/* ...output device for main */
int     __output_main = 1;

/* ...log level (looks ugly) */
int     LOG_LEVEL = 1;

/* ...V4L2 device name */
char   *imr_dev_name[] = {
    "/dev/video4",
    "/dev/video5",
    "/dev/video6",
    "/dev/video7",

};

/* ...default joystick device name  */
char   *joystick_dev_name = "/dev/input/js0";

int support_stdin = 0;
/* ...input (VIN) format */
u32     __vin_format = V4L2_PIX_FMT_UYVY;
int     __vin_width = 1280, __vin_height = 800, __vin_stride = (1280 + 255) & ~255;
int     __vin_buffers_num = 6;

/* ...VSP dimensions */
int     __vsp_width = 1280, __vsp_height = 720;

/*******************************************************************************
 * Live capturing from VIN cameras
 ******************************************************************************/

/* ...default V4L2 device names */
char * vin_dev_name[4] = {
    "/dev/video0",
    "/dev/video1",
    "/dev/video2",
    "/dev/video3",
};

/*******************************************************************************
 * Parameters parsing
 ******************************************************************************/
/* ...parse VIN device names */
static inline int parse_vin_devices(char *str, char **name, int n)
{
    char   *s;

    for (s = strtok(str, ","); n > 0 && s; n--, s = strtok(NULL, ","))
    {
        /* ...just copy a pointer (string is persistent) */
        *name++ = s;
    }

    /* ...make sure we have parsed all addresses */
    CHK_ERR(n == 0, -EINVAL);

    return 0;
}

/* ...parse camera format */
static inline u32 parse_format(char *str)
{
    if (strcasecmp(str, "uyvy") == 0)
    {
        return V4L2_PIX_FMT_UYVY;
    }
    else if (strcasecmp(str, "yuyv") == 0)
    {
        return V4L2_PIX_FMT_YUYV;
    }
    else if (strcasecmp(str, "nv16") == 0)
    {
        return V4L2_PIX_FMT_NV16;
    }
    else if (strcasecmp(str, "nv12") == 0)
    {
        return V4L2_PIX_FMT_NV12;
    }
    else if (strcasecmp(str, "i420") == 0)
    {
        return V4L2_PIX_FMT_YUV420;
    }
    else if (strcasecmp(str, "gray8") == 0)
    {
        return V4L2_PIX_FMT_GREY;
    }
    else
    {
        return 0;
    }
}

/* ...command-line options */
static const struct option    options[] = {
    {   "debug",    required_argument,  NULL,   'd' },
    {   "vin",      required_argument,  NULL,   'v' },
    {   "imr",      required_argument,  NULL,   'r' },
    {   "format",   required_argument,  NULL,   'f' },
    {   "width",    required_argument,  NULL,   'w' },
    {   "height",   required_argument,  NULL,   'h' },
    {   "Width",    required_argument,  NULL,   'W' },
    {   "Height",   required_argument,  NULL,   'H' },
    {   "buffers",  required_argument,  NULL,   'n' },
    {   "cameras",  required_argument,  NULL,   'N' },
    {   "stdin",    no_argument,	NULL,	'i' },
    {   NULL,       0,                  NULL,   0   },
};

/* ...option parsing */
static int parse_cmdline(int argc, char **argv)
{
    int     index = 0;
    int     opt;

    /* ...process command-line parameters */
    while ((opt = getopt_long(argc, argv, "d:v:r:f:w:h:W:H:n:N:i", options, &index)) >= 0)
    {
        switch (opt)
        {
        case 'd':
            /* ...debug level */
            TRACE(INIT, _b("debug level: '%s'"), optarg);
            LOG_LEVEL = atoi(optarg);
            break;

        case 'v':
            /* ...VIN device names */
            TRACE(INIT, _b("VIN devices: '%s'"), optarg);
            CHK_API(parse_vin_devices(optarg, vin_dev_name, 4));
            break;

        case 'r':
            /* ...set default IMR device name */
            TRACE(INIT, _b("IMR device: '%s'"), optarg);
            CHK_API(parse_vin_devices(optarg, imr_dev_name, 4));
            break;

        case 'f':
            /* ...parse camera format */
            TRACE(INIT, _b("Format: '%s'"), optarg);
            CHK_ERR(__vin_format = parse_format(optarg), -(errno = EINVAL));
            break;

        case 'w':
            /* ...parse resolution */
            TRACE(INIT, _b("Width: '%s'"), optarg);
            CHK_ERR((u32)(__vin_width = atoi(optarg)) < 4096, -(errno = EINVAL));
            break;

        case 'h':
            /* ...parse resolution */
            TRACE(INIT, _b("Height: '%s'"), optarg);
            CHK_ERR((u32)(__vin_height = atoi(optarg)) < 4096, -(errno = EINVAL));
            break;

        case 'W':
            /* ...parse resolution */
            TRACE(INIT, _b("VSP width: '%s'"), optarg);
            CHK_ERR((u32)(__vsp_width = atoi(optarg)) < 4096, -(errno = EINVAL));
            break;

        case 'H':
            /* ...parse resolution */
            TRACE(INIT, _b("VSP height: '%s'"), optarg);
            CHK_ERR((u32)(__vsp_height = atoi(optarg)) < 4096, -(errno = EINVAL));
            break;

        case 'n':
            /* ...parse number of buffers for VIN */
            TRACE(INIT, _b("Number of buffers: '%s'"), optarg);
            CHK_ERR((u32)(__vin_buffers_num = atoi(optarg)) < 64, -(errno = EINVAL));
            break;

        case 'N':
            /* ...parse number of camera */
            TRACE(INIT, _b("Number of cameras: '%s'"), optarg);
            CHK_ERR((u32)(cameras_number = atoi(optarg)) < 5, -(errno = EINVAL));
            break;

        case 'i':
            TRACE(INIT, _b("Use stdin to capture camera images"));
            support_stdin = 1;
            break;
        default:
            return -EINVAL;
        }
    }

    switch (__vin_format)
    {
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_YUYV:
        __vin_stride = ((2 * __vin_width) + 255) & ~255;
        break;
    default:
        __vin_stride = (__vin_width + 255) & ~255;
        break;
    }

    return 0;
}

/*******************************************************************************
 * Entry point
 ******************************************************************************/

int main(int argc, char **argv)
{
    display_data_t  *display;
    app_data_t      *app;

    /* ...initialize tracer facility */
    TRACE_INIT("Smart-camera demo");

    /* ...initialize GStreamer */
    gst_init(&argc, &argv);

    /* ...parse application specific parameters */
    CHK_API(parse_cmdline(argc, argv));

    /* ...initialize display subsystem */
    CHK_ERR(display = display_create(support_stdin), -errno);

    /* ...initialize unit-test application */
    CHK_ERR(app = app_init(display), -errno);

    /* ...execute mainloop thread */
    app_thread(app);

    TRACE(INIT, _b("application terminated"));

    return 0;
}

