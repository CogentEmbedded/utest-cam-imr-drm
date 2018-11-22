/*******************************************************************************
 * mesh-generators.h
 *
 * IMR unit-test application mesh generators function
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

#define MODULE_TAG                      MESH_GENERATORS

#include "utest-common.h"
#include "utest-mesh-generators.h"
#include "utest-camera-math.h"

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 1);

typedef struct
{
    float vertices[2];
    float texture[2];
} point_t;

int __undistorted_cfg_setup(app_data_t *app, camera_intrinsic_t *intrinsic, int id, int w, int h, int x_start, int y_start, int x_end, int y_end, imr_cfg_t *cfg)
{
    display_list_t     *dl = imr_cfg_dl(cfg);
    int                 columns, rows;
    int                 x, y, dx, dy;
    int                 i, j;
    int32_t dst_X_start = 0;
    int32_t dst_Y_start = 0;
    int32_t dst_X_end = 0;
    int32_t dst_Y_end = 0;
    int32_t dst_width = 0;
    int32_t dst_height = 0;

    dst_X_start = (x_start << DL_DST_SUBSAMPLE);
    dst_Y_start = (y_start << DL_DST_SUBSAMPLE);
    dst_X_end = (x_end << DL_DST_SUBSAMPLE);
    dst_Y_end = (y_end << DL_DST_SUBSAMPLE);
    dst_width = (dst_X_end - dst_X_start);
    dst_height = (dst_Y_end - dst_Y_start);

    /* ...calculate lattice size (in destination space) */
    columns = (dst_width + DL_DST_THRESHOLD - 1) / DL_DST_THRESHOLD;
    rows =  (dst_height + DL_DST_THRESHOLD - 1) / DL_DST_THRESHOLD;

    /* ...bail out if we have empty lattice */
    if ((columns == 0) ||
        (rows==0))
    {
        return 0;
    }

    /* ...put lattice configuration */
    dx = (dst_width >= DL_DST_THRESHOLD ? DL_DST_THRESHOLD : dst_width);
    dy = (dst_height >= DL_DST_THRESHOLD ? DL_DST_THRESHOLD : dst_height);
    CHK_API(dl_autocg_set_dxdy(dl, dx, dy));

    /* ...set origin in destination space at <0,0> */
    CHK_API(dl_autocg_set_x(dl, dst_X_start));

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
            float X = (float)x / dst_width;
            float Y = (float)y / dst_height;
            /* ...{x,y} is a point in destination space; get associated point in source space */
            u16     u = (u16)(X * (w << DL_SRC_SUBSAMPLE));
            u16     v = (u16)(Y * (h << DL_SRC_SUBSAMPLE));

            TRACE(0, _b("x,y=%u,%u, X,Y=%f,%f, u,v=%u,%u"), x, y, X, Y, u, v);

            s[0].uX = u;
            s[0].vY = v;


            Y = (float)(y + DL_DST_THRESHOLD) / dst_height;
            v = (u16)(Y * (h << DL_SRC_SUBSAMPLE));
            s[1].uX = u;
            s[1].vY = v;

            TRACE(0, _b("x,y=%u,%u, u,v=%u,%u"), x, y, u, v);
        }
    }

    TRACE(INIT, _b("camera-%d: display-list created (%d*%d, %u bytes)"), id, columns, rows, dl->n);

    return 0;
}

static inline point_t calculate_point(float x_cur, float y_cur, int dst_x_start, int dst_y_start,
                                      camera_intrinsic_t *intrinsic)
{
    point_t point;

    /* ...{dst_x_start,dst_y_start} is a point in destination space; get associated point in source space */
    float u = 0.0f;
    float v = 0.0f;
    project_world_to_camera(intrinsic, x_cur, y_cur, 1, &u, &v);

    point.vertices[0] = dst_x_start;
    point.vertices[1] = dst_y_start;
    point.texture[0] = u;
    point.texture[1] = v;
    return point;
}

static inline void push_quad_to_dl(display_list_t *dl_left,
                                   display_list_t *dl_right,
                                   point_t quad[4],
                                   int camera_width,
                                   int camera_height,
                                   int camera_threshold,
                                   int camera_buffer_shift)
{
    dl_abs_quad_t quad_abs = {0};
    quad_abs.opcode = set_opcode(4U);
    dl_vertex_abs_t vertice = {0, 0, 0, 0};
    display_list_t *cur_dl = NULL;
    int x_camera_shift = 0;

    if (quad[0].texture[0] > camera_threshold)
    {
        cur_dl = dl_right;
        x_camera_shift = camera_threshold - camera_buffer_shift;
    }
    else
    {
        cur_dl = dl_left;
    }

    uint32_t i;

    for (i = 0; i < 4U; i++)
    {
        int16_t dx = (int16_t)(quad[i].vertices[0] * (1 << DL_DST_SUBSAMPLE));
        int16_t dy = (int16_t)(quad[i].vertices[1] * (1 << DL_DST_SUBSAMPLE));
        int32_t xs = (quad[i].texture[0]);
        int32_t ys = (quad[i].texture[1]);

        if (xs > camera_threshold)
        {
            xs -= x_camera_shift;
        }

        uint16_t xsc, ysc;

        xsc = xs * (1 << DL_SRC_SUBSAMPLE);
        ysc = ys * (1 << DL_SRC_SUBSAMPLE);
        xsc = (xsc > 0) ? xsc : 0;
        ysc = (ysc > 0) ? ysc : 0;
        quad_abs.v[i] = vertice;
        quad_abs.v[i].X = dx;
        quad_abs.v[i].Y = dy;
        quad_abs.v[i].u = xsc;
        quad_abs.v[i].v = ysc;
    }

    dl_abs_quad_t *data;
    __dl_add(cur_dl, data, sizeof(dl_abs_quad_t));

    if (data != NULL)
    {
        *data = quad_abs;
    }
}

static inline void prepare_first_distorted_line(float x_start, float y_start, float x_step,
                                                int dst_x_start, int dst_y_start, int dst_x_step,
                                                int columns,
                                                camera_intrinsic_t *intrinsic,
                                                point_t *point_line)
{
    float x_cur = x_start;
    float dst_x_cur = dst_x_start;

    for (int xb = 0; xb <= columns; xb++)
    {
        point_line[xb] = calculate_point(x_cur, y_start, dst_x_cur, dst_y_start,
                                         intrinsic);
        x_cur += x_step;
        dst_x_cur += dst_x_step;
    }
}

static inline void prepare_distorted_line(display_list_t *dl_left,
                                          display_list_t *dl_right,
                                          camera_intrinsic_t *intrinsic,
                                          float x_start, float y_cur, float x_step,
                                          int dst_x_start, int dst_y_cur, int dst_x_step,
                                          int columns,
                                          point_t *points,
                                          int camera_width,
                                          int camera_height,
                                          int camera_threshold,
                                          int camera_buffer_shift)
{
    float x_cur = x_start;
    float dst_x_cur = dst_x_start;
    point_t point_prev;
    int xb = 0;

    {
        point_prev = calculate_point(x_cur, y_cur, dst_x_start, dst_y_cur,
                                     intrinsic);
        x_cur += x_step;
        dst_x_cur += dst_x_step;
    }

    xb++;

    point_t quad[4] = {0};

    for (; xb <= columns; xb++)
    {
        point_t point_cur = calculate_point(x_cur, y_cur, dst_x_cur, dst_y_cur,
                                            intrinsic);
        quad[0] = points[xb - 1U];
        quad[1] = point_prev;
        quad[2] = points[xb];
        quad[3] = point_cur;

        push_quad_to_dl(dl_left, dl_right, quad, camera_width, camera_height, camera_threshold, camera_buffer_shift);

        points[xb - 1U] = point_prev;
        point_prev = point_cur;
        x_cur += x_step;
        dst_x_cur += dst_x_step;
    }

    points[columns] = point_prev;
}

int __distorted_cfg_setup(camera_intrinsic_t *intrinsic,
                          int id,
                          int w, int h,
                          int camera_threshold, int camera_buffer_shift,
                          int x_start, int y_start,
                          int x_end, int y_end,
                          int hfov, int vfov_bottom_part,
                          int vfov_upper_part,
                          int block_size,
                          imr_cfg_t *cfg_left,
                          imr_cfg_t *cfg_right)
{
    display_list_t *dl_left = imr_cfg_dl(cfg_left);
    display_list_t *dl_right = imr_cfg_dl(cfg_right);
    int columns, rows;
    int x, y, dx, dy;
    int i, j;
    int32_t dst_X_start, dst_Y_start, dst_X_end, dst_Y_end;
    dst_X_start = (x_start);
    dst_Y_start = (y_start);
    dst_X_end = (x_end);
    dst_Y_end = (y_end);
    int32_t dst_width = (dst_X_end - dst_X_start);
    int32_t dst_height = (dst_Y_end - dst_Y_start);

    /* ...calculate lattice size (in destination space) */
    columns = (dst_width + block_size - 1) / block_size;
    rows = (dst_height + block_size - 1) / block_size;

    /* ...bail out if we have empty lattice */
    if ((columns == 0) ||
        (rows==0))
    {
        return 0;
    }

    int n = (columns + 1) * 2;

    float tan_x_start = -tanf((hfov / 2.0) * M_PI / 180.0f);
    float tan_x_end = tanf((hfov / 2.0) * M_PI / 180.0f);
    float tan_x_step = (block_size * (tan_x_end - tan_x_start)) / (dst_width);

    float tan_y_start = -tanf((vfov_upper_part)*M_PI / 180.0f);
    float tan_y_end = tanf((vfov_bottom_part)*M_PI / 180.0f);
    float tan_y_step = (block_size * (tan_y_end - tan_y_start)) / (dst_height);

    int points_number = (((dst_width + (block_size - 1)) / block_size) + 1);
    point_t point_line[points_number];

    prepare_first_distorted_line(tan_x_start, tan_y_start,
                                 tan_x_step, dst_X_start, dst_Y_start, block_size,
                                 columns,
                                 &intrinsic[id],
                                 point_line);

    float tan_y_cur = tan_y_start + tan_y_step;

    y = dst_Y_start + block_size;

    for (i = 1; i <= rows; i++)
    {
        prepare_distorted_line(dl_left,
                               dl_right,
                               &intrinsic[id],
                               tan_x_start, tan_y_cur, tan_x_step,
                               dst_X_start, y, block_size,
                               columns,
                               point_line,
                               w, h, camera_threshold, camera_buffer_shift);
        y += block_size;
        tan_y_cur += tan_y_step;
    }

    return 0;
}