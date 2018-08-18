
/*******************************************************************************
 * utest-png.c
 *
 * ADAS unit-test. Interface to save camera pictures in pngs.
 *
 * Copyright (c) 2018 Cogent Embedded Inc. ALL RIGHTS RESERVED.
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

#define MODULE_TAG                      UTEST_PNG

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <png.h>

#include "utest-common.h"
#include "utest-png.h"

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 1);

/* ...camera format */
extern u32  __vin_format;

/*******************************************************************************
 * PNG storing
 ******************************************************************************/

#include <math.h>

typedef struct
{
    uint8_t u;
    uint8_t y1;
    uint8_t v;
    uint8_t y2;
} uyvy_pixel_t;

typedef struct
{
    uint8_t y1;
    uint8_t u;
    uint8_t y2;
    uint8_t v;
} yuyv_pixel_t;

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_pixel_t;


static inline void yuv444_to_rgb888(uint8_t yValue, uint8_t uValue, uint8_t vValue,
        uint8_t *r, uint8_t *g, uint8_t *b)
{
    int rTmp = yValue + (1.370705f * (vValue-128));
    int gTmp = yValue - (0.698001f * (vValue-128)) - (0.337633f * (uValue-128));
    int bTmp = yValue + (1.732446f * (uValue-128));
    *r = CLAMP(rTmp, 0, 255);
    *g = CLAMP(gTmp, 0, 255);
    *b = CLAMP(bTmp, 0, 255);
}

void uyvy_to_rgb(uint8_t* yuv_data, uint8_t* rgb_data, ssize_t width, ssize_t height)
{
    uyvy_pixel_t* yuv_pixel = (uyvy_pixel_t*) yuv_data;
    rgb_pixel_t* rgb_pixel = (rgb_pixel_t*) rgb_data;
    int i = 0;
    for (i = 0; i < (width * height / 2); i++)
    {
        yuv444_to_rgb888(yuv_pixel->y1, yuv_pixel->v, yuv_pixel->u, 
                         &(rgb_pixel->r), &(rgb_pixel->g), &(rgb_pixel->b));
        rgb_pixel++;
        yuv444_to_rgb888(yuv_pixel->y2, yuv_pixel->v, yuv_pixel->u, 
                         &(rgb_pixel->r), &(rgb_pixel->g), &(rgb_pixel->b));
        rgb_pixel++;
        yuv_pixel++;
    }
}
void yuyv_to_rgb(guint8* yuv_data, guint8* rgb_data, ssize_t width, ssize_t height)
{
    yuyv_pixel_t* yuv_pixel = (yuyv_pixel_t*) yuv_data;
    rgb_pixel_t* rgb_pixel = (rgb_pixel_t*) rgb_data;
    int i = 0;
    for (i = 0; i < (width * height / 2); i++)
    {
        yuv444_to_rgb888(yuv_pixel->y1, yuv_pixel->u, yuv_pixel->v, 
                         &(rgb_pixel->r), &(rgb_pixel->g), &(rgb_pixel->b));
        rgb_pixel++;
        yuv444_to_rgb888(yuv_pixel->y2, yuv_pixel->u, yuv_pixel->v, 
                         &(rgb_pixel->r), &(rgb_pixel->g), &(rgb_pixel->b));
        rgb_pixel++;
        yuv_pixel++;
    }
}
void nv16_to_rgb(guint8* nv16_data, guint8* rgb_data, ssize_t width, ssize_t height)
{
    guint8* y_pixel = nv16_data;
    guint8* uv_pixel = nv16_data + (width * height);
    rgb_pixel_t* rgb_pixel = (rgb_pixel_t*) rgb_data;
    int i = 0;
    for (i = 0; i < (width * height / 2); i++)
    {
        yuv444_to_rgb888(y_pixel[0], uv_pixel[1], uv_pixel[0],
                         &(rgb_pixel->r), &(rgb_pixel->g), &(rgb_pixel->b));
        rgb_pixel++;
        yuv444_to_rgb888(y_pixel[1], uv_pixel[1], uv_pixel[0],
                         &(rgb_pixel->r), &(rgb_pixel->g), &(rgb_pixel->b));
        rgb_pixel++;
        y_pixel += 2;
        uv_pixel += 2;
    }
}
static void
user_write_data(png_structp png_ptr,
        png_bytep data, png_size_t length)
{
    FILE *fp = (FILE *)png_get_io_ptr(png_ptr);
    size_t result;

    result = fwrite(data, 1, length, fp);

    if (result != length)
    {
        TRACE(1, _b("result %lu"), result);
    }
}

static void
user_flush_data(png_structp png_ptr)
{
    FILE *fp = (FILE *)png_get_io_ptr(png_ptr);

    if (fflush(fp))
        png_error(png_ptr, "Failed to flush png file!");
}

/* ...writer error handling routine */
static __attribute__((noreturn))  void __write_error(png_structp png_ptr, png_const_charp msg)
{
    jmp_buf    *jbp;

    TRACE(ERROR, _b("writepng libpng error: %s"), msg);
  
    if ((jbp = png_get_error_ptr(png_ptr)) == NULL)
    {
        BUG(1, _x("non-recoverable error"));
    }

    /* ...get back to the writer */
    longjmp(*jbp, EBADF);
}

/* ...write PNG file */
int store_png(const char *otp_id, int index ,int width, int height, int format, void *data)
{
	FILE                   *fp;
	int                     y, stride = 0;
	png_byte                color_type;
	png_byte                bit_depth= 24;
	png_structp             png_ptr;
	png_infop               info_ptr;
	jmp_buf                 jb;
    int                     retval = 0;
    uint8_t                 *rows[height];
    int i;

    uint8_t* pixbuf         = NULL;
    char* filename = NULL;

    if ((__vin_format != V4L2_PIX_FMT_NV16) &&
        (__vin_format != V4L2_PIX_FMT_UYVY) &&
        (__vin_format != V4L2_PIX_FMT_YUYV)) {
        TRACE(ERROR, _x("unsupported format %x: %m"), __vin_format);
        return -EINVAL;
    }

    asprintf(&filename, "frame-%s-%03d.png", otp_id, index);

    /* ...sanity check - data buffer pointer must be provided */
    CHK_ERR(width > 0 && height >= 0 && format > 0 && data, -EINVAL);

    /* ...prepare file for writing */
    if ((fp = fopen(filename, "wb")) == NULL)
    {
        TRACE(ERROR, _x("failed to open file '%s': %m"), filename);
        return -errno;
    }

    /* ...create write structure */
    if (!(png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, &jb, __write_error, NULL)))
    {
        TRACE(ERROR, _x("failed to create write struct: %m"));
        errno = ENOMEM;
        goto error;
    }
  
    if (!(info_ptr = png_create_info_struct(png_ptr)))
    {
        TRACE(ERROR, _x("failed to create info struct: %m"));
        errno = ENOMEM;
        goto error_png;
    }

    /* ...prepare emergency return point */
    if ((errno = setjmp(jb)) != 0)
    {
        TRACE(ERROR, _x("operation failed"));
        goto error_png;
    }

    /* ...initialize I/O */
    png_init_io(png_ptr, fp);

    /* ...set compression level (zlib 0 to 9) */
    //png_set_compression_level(png_ptr, 9);

    stride = width * 3;
    color_type = PNG_COLOR_TYPE_RGB;
    pixbuf = malloc(stride * height* 3);

    if (__vin_format == V4L2_PIX_FMT_NV16)
        nv16_to_rgb(data, pixbuf, width, height);
    else if (__vin_format == V4L2_PIX_FMT_UYVY)
        uyvy_to_rgb(data, pixbuf, width, height);
    else if (__vin_format == V4L2_PIX_FMT_YUYV)
        yuyv_to_rgb(data, pixbuf, width, height);

    for (i = 0; i < height; i++)
        rows[i] = pixbuf + i * stride;

    TRACE(DEBUG, _b("prepare writing: w=%d, h=%d, d=%d, c=%d format=%d "), width, height, bit_depth, color_type, format);

    /* ...native format that we use is BGRA - hmm, tbd */
    (color_type & PNG_COLOR_MASK_COLOR ? png_set_bgr(png_ptr) : 0);

    png_set_write_fn(png_ptr, fp, user_write_data, user_flush_data);
    /* ...write header */
    png_set_IHDR(png_ptr, info_ptr, width, height, 8, color_type, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);


    /* ...write info structure */
    png_write_info(png_ptr, info_ptr);
    /* ...write all image rows */
    png_set_rows(png_ptr, info_ptr, rows);
    png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);
    /* ...mark completion of the image */
    png_write_end(png_ptr, NULL);
    png_write_flush(png_ptr);

    TRACE(1, _b("written file '%s"), filename);

    /* ...cleanup after ourselves */
    png_destroy_info_struct(png_ptr, &info_ptr);
    png_destroy_write_struct(&png_ptr, NULL);

    free(pixbuf);
    free(filename);
    fclose(fp);

    return 0;

error_png:
    /* ...destroy write structure */
    png_destroy_info_struct(png_ptr, &info_ptr);
    png_destroy_write_struct(&png_ptr, NULL);
    free(pixbuf);

error:
    /* ...close file handle */
    free(filename);
    fclose(fp);
    return -1;
}
