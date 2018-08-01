/*******************************************************************************
 * dynarray.h
 *
 * Dynamic arrays handling
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

#ifndef __DYNARRAY_H
#define __DYNARRAY_H

/*******************************************************************************
 * Local types definitions
 ******************************************************************************/

/* ...dynamic array definition */
#define __dynarray(type, id)        struct { type *_p; int _n, _a; } id

/* ...array length */
#define __dynarray_n(id)            \
    (id._n)

/* ...data buffer */
#define __dynarray_p(id)            \
    (id._p)

/* ...size of dynamic array item */
#define __dynarray_size(id)         \
    sizeof(*id._p)

/* ...dynamic array initialization */
#define __dynarray_create(id, a, n) \
    (id._p = xrealloc(id._p, (id._n = (n), id._a = (a)) * sizeof(*id._p)))

/* ...dynamic array initialization for fixed size */
#define __dynarray_create_size(id, a, n, size)   \
    (id._p = xrealloc(id._p, (id._n = (n), id._a = (a)) * (size)))

/* ...dynamic array initialization with preallocated storage */
#define __dynarray_create_mem(id, a, n, p)      \
    (id._n = n, id._a = a, id._p = p)

/* ...dynamic array destruction */
#define __dynarray_delete(id)       \
    (id._p ? free(id._p) : 0)

/* ...add entry into dynamic array */
#define __dynarray_add(id, p)                                                                       \
({                                                                                                  \
    int     __a = id._a;                                                                            \
                                                                                                    \
    if (id._n < __a || (id._p = xrealloc(id._p, (id._a = __a + ((__a >> 2) ? : 1)) * sizeof(*id._p)))) \
    {                                                                                               \
        p = &id._p[id._n++];                                                                        \
    }                                                                                               \
    else                                                                                            \
    {                                                                                               \
        p = NULL;                                                                                   \
    }                                                                                               \
    p;                                                                                              \
})          

/* ...reserve entry in the dynamic array */
#define __dynarray_add_ext(id, k, p)                                                                \
({                                                                                                  \
    int     __a = id._a, __n = id._n, __m = __n + (k);                                              \
                                                                                                    \
    if (__m <= __a || (id._p = xrealloc(id._p, (id._a = __m + (__m >> 2 ? : 1)) * sizeof(*id._p)))) \
    {                                                                                               \
        p = &id._p[__n], id._n = __m;                                                               \
    }                                                                                               \
    else                                                                                            \
    {                                                                                               \
        p = NULL;                                                                                   \
    }                                                                                               \
    p;                                                                                              \
})    

/* ...add entry into dynamic array */
#define __dynarray_add_nogrow(id, p)            \
({                                              \
    int     __a = id._a;                        \
                                                \
    if (id._n < __a)                            \
    {                                           \
        p = &id._p[id._n++];                    \
    }                                           \
    else                                        \
    {                                           \
        p = NULL;                               \
    }                                           \
    p;                                          \
})          

/* ...pop last item from the dynamic array */
#define __dynarray_pop(id, p)                   \
    (p = (id._n ? &id._p[--id._n] : NULL))

/* ...clear dynamic array */
#define __dynarray_clear(id, n)     \
    (id._n = (n))

/*******************************************************************************
 * Serialization support
 ******************************************************************************/

#define __m_save(id, n, f)                                                      \
({                                                                              \
    int     __err;                                                              \
                                                                                \
    if ((__err = (fwrite(id, sizeof(*id), (n), (f)) != (size_t)(n))) != 0)      \
    {                                                                           \
        TRACE(ERROR, _x("failed to save %zu bytes: %m"), sizeof(*id) * (n));    \
    }                                                                           \
    __err;                                                                      \
})

#define __m_restore(id, n, f)                                                   \
({                                                                              \
    int     __err;                                                              \
                                                                                \
    if ((__err = (fread(id, sizeof(*id), (n), (f)) != (size_t)(n))) != 0)       \
    {                                                                           \
        TRACE(ERROR, _x("failed to restore %zu bytes: %m"), sizeof(*id) * (n)); \
    }                                                                           \
    __err;                                                                      \
})

/* ...serialize dynamic array */
#define __dynarray_save(id, f)                                  \
    ({ int __n = id._n; (__m_save(&__n, 1, (f)) ? : __m_save(id._p, __n, (f))); })        
    
#define __dynarray_restore(id, f)                               \
    ({ int __n; __m_restore(&__n, 1, (f)) ? : (__dynarray_create(id, __n, __n) ? __m_restore(id._p, __n, (f)) : -1); })

/*******************************************************************************
 * Miscellaneous helpers
 ******************************************************************************/

#define __m_cmp(a, b)           \
    (memcmp(a, b, sizeof(*a)))

/* ...compare two dynamic arrays */
#define __dynarray_cmp(a, b)    \
    (a._n != b._n || memcmp(a._p, b._p, sizeof(*a._p) * a._n))

#endif  /* __DYNARRAY_H */
