/*******************************************************************************
 * dl.c
 *
 * Display list for rendering
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

#define MODULE_TAG                      DL

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "utest-common.h"
#include "dl.h"

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 0);

/*******************************************************************************
 * Triangle drawing opcode
 ******************************************************************************/

#define IMR_OP_TRI(n)                   ((0x8A << 24) | ((n) & 0xFFFF))
#define IMR_OP_WTS(add, data)           ((0x82 << 24) | (((add) / 4) << 16) | ((data) & 0xFFFF))
#define IMR_OP_WTL(add, n)              ((0x81 << 24) | (((add) / 4) << 16) | ((n) & 0xFFFF))
#define IMR_OP_RET                      ((0x8D << 24))

/* ...cropping registers */
#define IMR_XMINR                       0x80
#define IMR_YMINR                       0x84
#define IMR_XMAXR                       0x88
#define IMR_YMAXR                       0x8C

/* ...automatic coordinates generation registers */
#define IMR_AMXSR                       0x90
#define IMR_AMYSR                       0x94
#define IMR_AMXOR                       0x98
#define IMR_AMYOR                       0x9C

/*******************************************************************************
 * Fill triangle with automatically generated destoination coordinates
 ******************************************************************************/

int dl_autocg_set(display_list_t *dl, int x0, int y0, int dx, int dy)
{
    u32     *c;

    /* ...add DL-records */
    CHK_ERR(__dl_add(dl, c, sizeof(*c) * 4), -1);

    /* ...fill automatic coordinates generation */
    *c++ = IMR_OP_WTS(IMR_AMXSR, dx);
    *c++ = IMR_OP_WTS(IMR_AMYSR, dy);
    *c++ = IMR_OP_WTS(IMR_AMXOR, x0);
    *c++ = IMR_OP_WTS(IMR_AMYOR, y0);

    return 0;
}

int dl_autocg_set_y(display_list_t *dl, int y0)
{
    u32     *c;

    /* ...add DL-records */
    CHK_ERR(__dl_add(dl, c, sizeof(*c)), -1);

    /* ...fill automatic coordinates generation */
    *c++ = IMR_OP_WTS(IMR_AMYOR, y0);

    return 0;
}

int dl_autocg_set_x(display_list_t *dl, int x0)
{
    u32     *c;

    /* ...add DL-records */
    CHK_ERR(__dl_add(dl, c, sizeof(*c)), -1);

    /* ...fill automatic coordinates generation */
    *c++ = IMR_OP_WTS(IMR_AMXOR, x0);

    return 0;
}

int dl_autocg_set_xy(display_list_t *dl, int x0, int y0)
{
    u32     *c;

    /* ...add DL-records */
    CHK_ERR(__dl_add(dl, c, sizeof(*c) * 2), -1);

    /* ...fill automatic coordinates generation */
    *c++ = IMR_OP_WTS(IMR_AMXOR, x0);
    *c++ = IMR_OP_WTS(IMR_AMYOR, y0);

    return 0;
}

int dl_autocg_set_dxdy(display_list_t *dl, int dx, int dy)
{
    u32     *c;

    /* ...add DL-records */
    CHK_ERR(__dl_add(dl, c, sizeof(*c) * 2), -1);

    /* ...fill automatic coordinates generation */
    *c++ = IMR_OP_WTS(IMR_AMXSR, dx);
    *c++ = IMR_OP_WTS(IMR_AMYSR, dy);

    return 0;
}

/* ...create a triangle strip */
dl_strip_abs_t * dl_abs_strip_create(display_list_t *dl, int n)
{
    dl_autodg_abs_t  *c;

    /* ...create new record */
    CHK_ERR(__dl_add(dl, c, sizeof(*c) + n * sizeof(c->v[0])), NULL);

    /* ...fill opcode parameter */
    c->opcode = IMR_OP_TRI(n);

    /* ...return pointer to a strip data */
    return &c->v[0];
}

/* ...specify crop-region */
int dl_set_crop(display_list_t *dl, int x0, int y0, int x1, int y1)
{
    u32     *c;

    /* ...add DL-records */
    CHK_ERR(__dl_add(dl, c, sizeof(*c) * 4), -1);

    /* ...fill automatic coordinates generation */
    *c++ = IMR_OP_WTS(IMR_XMINR, x0);
    *c++ = IMR_OP_WTS(IMR_YMINR, y0);
    *c++ = IMR_OP_WTS(IMR_XMAXR, x1);
    *c++ = IMR_OP_WTS(IMR_YMAXR, y1);

    return 0;
}
