/*******************************************************************************
 * dl.h
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

#ifndef __DL_H
#define __DL_H

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "dynarray.h"

/*******************************************************************************
 * Types declarations
 ******************************************************************************/

/* ...absolute coordinates with luminance correction */
typedef struct dl_vertex_abs
{
    u16         v, u;
    s16         Y, X;

}    __attribute__((packed))    dl_vertex_abs_t;

/* ...display list item for absolute coordinates mesh */
typedef struct dl_abs
{
    /* ...opcode */
    u32                     opcode;

    /* ...vertices buffer */
    dl_vertex_abs_t         v[3];
    
}   __attribute__((packed))     dl_abs_t;

/* ...absolute coordinates with luminance correction */
typedef struct dl_vertex_luce
{
    u16         v, u;
    s16         Y, X;
    s8          lofs;
    u8          lscal;
    u16         pad;

}    __attribute__((packed))    dl_vertex_luce_t;

/* ...display list item for Luma correction only */
typedef struct dl_luce
{
    /* ...opcode */
    u32                     opcode;

    /* ...vertices buffer */
    dl_vertex_luce_t        v[3];
    
}   __attribute__((packed))     dl_luce_t;

/* ...absolute coordinates with full correction */
typedef struct dl_vertex_luce_clce
{
    u16         v, u;
    s16         Y, X;
    s8          lofs;
    u8          lscal;
    u16         pad;
    s8          vrofs;
    u8          vrscal;
    s8          ubofs;
    u8          ubscal;

}    __attribute__((packed))    dl_vertex_luce_clce_t;

/* ...display list for Luma/Chroma correction */
typedef struct dl_luce_clce
{
    /* ...opcode */
    u32                     opcode;

    /* ...vertices buffer */
    dl_vertex_luce_clce_t   v[3];
    
}   __attribute__((packed))     dl_luce_clce_t;

typedef struct dl_strip_abs
{
    /* ...<u,v> or <X,Y> coordinates of a strip vertex */
    u16         vY, uX;
    
}   __attribute__((packed))     dl_strip_abs_t;

typedef struct dl_strip_luce_clce
{
    /* ...<u,v> or <X,Y> coordinates of a strip vertex */
    u16         vY, uX;

    /* ...luminance/chrominance correction coefficients */
    s8          lofs;
    u8          lscal;
    u16         pad;
    s8          vrofs;
    u8          vrscal;
    s8          ubofs;
    u8          ubscal;
    
}   __attribute__((packed))     dl_strip_luce_clce_t;

/* ...vertex with auto-generated source/destination coordinates only */
typedef struct dl_autodg_abs
{
    /* ...opcode */
    u32                     opcode;

    /* ...vertex stripe */
    dl_strip_abs_t          v[0];

}   __attribute__((packed))     dl_autodg_abs_t;


/* ...vertex with auto-generated source/destination coordinates and Luma/Chroma correction */
typedef struct dl_autodg_luce_clce
{
    /* ...opcode */
    u32                     opcode;

    /* ...vertex stripe */
    dl_strip_luce_clce_t    v[0];

}   __attribute__((packed))     dl_autodg_luce_clce_t;

/* ...display list */
typedef struct display_list
{
    /* ...display list buffer */
    void                           *buffer;

    /* ...current buffer size */
    u32                             n;
    
    /* ...total buffer length */
    u32                             a;

    /* ...number of triangles (informative) */
    int                             N;

}   display_list_t;

/* ...add entry to display list */
#define __dl_add(dl, p, size)                                   \
({                                                              \
    u32     __n = (dl)->n, __a = (dl)->a, __m = __n + (size);   \
    void   *__p;                                                \
                                                                \
    if (__m > __a)                                              \
    {                                                           \
        __p = NULL;                                             \
    }                                                           \
    else                                                        \
    {                                                           \
        __p = (dl)->buffer + __n, (dl)->n = __m;                \
    }                                                           \
                                                                \
    (p) = __p;                                                  \
})

#define __dl_clear(dl)                                          \
    ((dl)->n = 0, (dl)->N = 0)

#define __dl_size(dl, type)                                     \
    ((const u32)((dl)->n / sizeof(type)))

#define __dl_create(dl, buf, size)                              \
    ((dl)->buffer = (buf), (dl)->a = (size), (dl)->n = 0)

/*******************************************************************************
 * Global constants definitions (tbd; should be configurable?)
 ******************************************************************************/

#define DL_SRC_SUBSAMPLE            5
#define DL_DST_SUBSAMPLE            2

#define DL_DST_THRESHOLD            (64 << DL_DST_SUBSAMPLE)
#define DL_SRC_THRESHOLD            (32 << DL_SRC_SUBSAMPLE)

/*******************************************************************************
 * Helpers
 ******************************************************************************/

/* ...automatic coordinates generation */
extern int dl_autocg_set(display_list_t *dl, int x0, int y0, int dx, int dy);
extern int dl_autocg_set_x(display_list_t *dl, int x0);
extern int dl_autocg_set_y(display_list_t *dl, int y0);
extern int dl_autocg_set_dxdy(display_list_t *dl, int dx, int dy);
extern int dl_autocg_set_xy(display_list_t *dl, int x0, int y0);

extern dl_strip_abs_t * dl_abs_strip_create(display_list_t *dl, int n);

#endif  /* __DL_H */
