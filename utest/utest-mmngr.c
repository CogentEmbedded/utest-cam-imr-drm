/*******************************************************************************
 * utest-mmngr.c
 *
 * Memory manager for IMR-based surround-view application
 *
 * Copyright (c) 2016 Cogent Embedded Inc. ALL RIGHTS RESERVED.
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

#define MODULE_TAG                      VSP

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "utest-common.h"
#include "utest-mmngr.h"
#include <mmngr_user_public.h>
#include <mmngr_buf_user_public.h>
#include <linux/videodev2.h>

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 1);

/*******************************************************************************
 * Types definitions
 ******************************************************************************/

/* ...DMA buffer descriptor */
struct vsp_dmabuf
{
    /* ...exported buffer identifier */
    int                 id;

    /* ...DMA file-descriptor */
    int                 fd;
};

/* ...memory descriptor */
struct vsp_mem
{
    /* ...identifier of the memory buffer */
    MMNGR_ID            id;

    /* ...user-accessible pointer */
    unsigned long       user_virt_addr;
    
    /* ...physical address(es?) */
    unsigned long       phy_addr, hard_addr;

    /* ...size of a chunk */
    unsigned long       size;

    /* ...exported DMA buffers */
    vsp_dmabuf_t      **dmabuf;

    /* ...number of planes */
    int                 planes;
    
    /* ...number of planes */
    u32                 offset[3];
};
    
/*******************************************************************************
 * Memory allocation
 ******************************************************************************/

/* ...allocate contiguous block */
vsp_mem_t * vsp_mem_alloc(u32 size)
{
    vsp_mem_t      *mem;
    int             err;

    /* ...allocate contiguous memory descriptor */
    CHK_ERR(mem = calloc(1, sizeof(*mem)), (errno = ENOMEM, NULL));

    /* ...all chunks must be page-size aligned */
    size = (size + 4095) & ~4095;

    /* ...allocate physically contiguous memory */
    switch (err = mmngr_alloc_in_user(&mem->id, size, &mem->phy_addr, &mem->hard_addr, &mem->user_virt_addr, MMNGR_VA_SUPPORT))
    {
    case R_MM_OK:
        /* ...memory allocated successfully */
        TRACE(DEBUG, _b("allocated %p[%u] block[%X] (pa=%08lx)"), (void *)(uintptr_t)mem->user_virt_addr, size, mem->id, mem->hard_addr);
        mem->size = size;
        return mem;

    case R_MM_NOMEM:
        /* ...insufficient memory */
        TRACE(ERROR, _x("failed to allocated contiguous memory block (%u bytes)"), size);
        errno = ENOMEM;
        break;
        
    default:
        /* ...internal allocation error */
        TRACE(ERROR, _x("memory allocation error (%u bytes), err=%d"), size, err);
        errno = EBADF;
    }

    free(mem);
    return NULL;
}

/* ...destroy contiguous memory block */
void vsp_mem_free(vsp_mem_t *mem)
{
    /* ...free allocated memory */
    mmngr_free_in_user(mem->id);
    
    TRACE(DEBUG, _b("destroyed block #%X (va=%p)"), mem->id, (void *)(uintptr_t)mem->user_virt_addr);

    /* ...destroy memory descriptor */
    free(mem);
}

/* ...memory buffer accessor */
void * vsp_mem_ptr(vsp_mem_t *mem)
{
    return (void *)(uintptr_t)mem->user_virt_addr;
}

/* ...memory buffer physical address */
u32 vsp_mem_paddr(vsp_mem_t *mem)
{
    return mem->hard_addr;
}

/* ...memory size */
u32 vsp_mem_size(vsp_mem_t *mem)
{
    return mem->size;
}

/* ...export DMA file-descriptor representing contiguous block */
vsp_dmabuf_t * vsp_dmabuf_export(vsp_mem_t *mem, u32 offset, u32 size)
{
    vsp_dmabuf_t   *dmabuf;
    unsigned long   hard_addr;
    int             err;

    /* ...sanity check */
    CHK_ERR(offset + size <= mem->size, (errno = EINVAL, NULL));

    /* ...allocate DMA-buffer descriptor */
    CHK_ERR(dmabuf = malloc(sizeof(*dmabuf)), (errno = ENOMEM, NULL));

    /* ...get physical address of the mapped buffer (should we align that by page-size? - tbd) */
    hard_addr = mem->hard_addr + offset;

    /* ...exported chunks must be page-size aligned (tbd) */
    size = (size + 4095) & ~4095;

    /* ...export memory as file-descriptor */
#ifdef __VSPM_GEN3
    switch (err = mmngr_export_start_in_user_ext(&dmabuf->id, size, hard_addr, &dmabuf->fd, NULL))
#else
    switch (err = mmngr_export_start_in_user(&dmabuf->id, size, hard_addr, &dmabuf->fd))
#endif
    {
    case R_MM_OK:
        TRACE(DEBUG, _b("exported block[%X] (fd=%d): pa=0x%08lx, size=%u"), dmabuf->id, dmabuf->fd, hard_addr, size);
        return dmabuf;
        
    default:
        TRACE(ERROR, _x("failed to export DMA-fd: %d"), err);
        errno = EBADF;
    }

    free(dmabuf);
    return NULL;
}

/* ...DMA-buffer descriptor accessor */
int vsp_dmabuf_fd(vsp_dmabuf_t *dmabuf)
{
    return dmabuf->fd;
}

/* ...close DMA file-descriptor */
void vsp_dmabuf_unexport(vsp_dmabuf_t *dmabuf)
{
    /* ...release DMA-descriptor */
#ifdef __VSPM_GEN3
    mmngr_export_end_in_user_ext(dmabuf->id);
#else
    mmngr_export_end_in_user(dmabuf->id);
#endif

    /* ...destroy buffer handle */
    free(dmabuf);
}


static inline u32 __vsp_pixfmt_size(int w, int h, int s, u32 fmt)
{
    switch(fmt)
    {
    case V4L2_PIX_FMT_GREY:     return (s ? : w) * h;
    case V4L2_PIX_FMT_UYVY:     return (s ? : 2 * w) * h;
    case V4L2_PIX_FMT_YUYV:     return (s ? : 2 * w) * h;
    case V4L2_PIX_FMT_NV16:     return (s ? : w) * h * 2;
    case V4L2_PIX_FMT_ARGB32:   return (s ? : 4 * w) * h;
    case V4L2_PIX_FMT_YUV420:   return (s ? : w) * h * 3 / 2;
    default:                    return 0;
    }
}

/* ...determine planes parameters for a given format */
static inline int __vsp_pixfmt_planes(int w, int h, int s, u32 fmt, u32 *size, u32 *stride)
{
    switch(fmt)
    {
    case V4L2_PIX_FMT_GREY:
        return size[0] = (stride[0] = (s ? : w)) * h, 1;
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_YUYV:
        return size[0] = (stride[0] = (s ? : 2 * w)) * h, 1;
    case V4L2_PIX_FMT_NV12:
    case V4L2_PIX_FMT_NV21:
    case V4L2_PIX_FMT_YUV420:
        return size[0] = (stride[0] = (s ? : w)) * h * 3 / 2, 1;
    case V4L2_PIX_FMT_NV16:
    case V4L2_PIX_FMT_NV61:
        return size[0] = (stride[0] = (s ? : w)) * h * 2, 1;
    case V4L2_PIX_FMT_ARGB32:
    case V4L2_PIX_FMT_XRGB32:
        return size[0] = (stride[0] = (s ? : w * 4)) * h, 1;
    default:
        return TRACE(ERROR, _b("unrecognized format: %X: %c%c%c%c"), fmt, __v4l2_fmt(fmt)), 0;
    }
}

/* ...allocate contiguous memory buffer pool */
int vsp_allocate_buffers(int w, int h, int s, u32 fmt, vsp_mem_t **output, int num)
{
    u32     size;
    int     i, n;
    u32     psize[3], stride[3], offset[3];

    /* ...calculate size of the buffer */
    if ((size = __vsp_pixfmt_size(w, h, s, fmt)) == 0)
    {
        TRACE(ERROR, _x("unsupported format '%c%c%c%c'"), __v4l2_fmt(fmt));
        return -(errno = EINVAL);
    }

    /* ...calculate buffer properties */
    if ((n = __vsp_pixfmt_planes(w, h, s, fmt, psize, stride)) == 0)
    {
        TRACE(ERROR, _x("invalid format '%c%c%c%c'"), __v4l2_fmt(fmt));
        return -(errno = EINVAL);
    }

    /* ...set offsets of the planes */
    for (offset[0] = 0, i = 1; i < n; i++)
    {
        offset[i] = offset[i - 1] + psize[i - 1];
    }
    
    /* ...allocate memory descriptors */
    for (i = 0; i < num; i++)
    {
        if ((output[i] = vsp_mem_alloc(size)) == NULL)
        {
            TRACE(ERROR, _x("failed to allocate buffer pool"));
            goto error;
        }

        /* ...set planes offsets */
        memcpy(output[i]->offset, offset, sizeof(u32) * n);
        
        TRACE(DEBUG, _b("allocate buffer: fmt=%c%c%c%c, size=%u, addr=0x%08lX"), __v4l2_fmt(fmt), size, output[i]->hard_addr);
    }
    
    return 0;

error:
    /* ...destroy buffers allocated */
    while (i--)
    {
        vsp_mem_free(output[i]), output[i] = NULL;
    }

    return -(errno = ENOMEM);
}

/* ...allocate contiguous memory buffer pool */
int vsp_buffer_export(vsp_mem_t *mem, int w, int h, int s, u32 fmt, int *dmafd, u32 *offset, u32 *stride)
{
    u32     size[GST_VIDEO_MAX_PLANES], o;
    int     n;
    int     i;

    /* ...verify format */
    CHK_ERR((n = __vsp_pixfmt_planes(w, h, s, fmt, size, stride)) > 0, -(errno = EINVAL));

    /* ...check if buffer is mapped already */
    if (mem->dmabuf)
    {
        for (i = 0; i < n; i++)
        {
            dmafd[i] = mem->dmabuf[i]->fd;
            offset[i] = 0;
        }

        return 0;
    }
    
    //CHK_ERR(!mem->dmabuf, -(errno = EBUSY));    

    /* ...allocate dma-buffers array */
    CHK_ERR(mem->dmabuf = calloc(n, sizeof(vsp_dmabuf_t *)), -(errno = ENOMEM));

    for (i = 0; i < n; i++)
    {
        TRACE(INFO, _b("plane-%d: fmt=%c%c%c%c, size=%u, stride=%u"), i, __v4l2_fmt(fmt), size[i], stride[i]);
    }
    
    /* ...allocate required amount of planes */
    for (i = 0, o = 0; i < n; i++)
    {
        /* ...export single plane (may fail if not page-size aligned - tbd) */
        if ((mem->dmabuf[i] = vsp_dmabuf_export(mem, o, size[i])) == NULL)
        {
            TRACE(ERROR, _x("failed to export DMA buffer: %m"));
            goto error;
        }
        else
        {
            dmafd[i] = mem->dmabuf[i]->fd;
            offset[i] = 0;
            o += size[i];
            TRACE(DEBUG, _b("plane-%d: fd=%d, offset=%X, size=%u, stride=%u"), i, dmafd[i], offset[i], size[i], stride[i]);
        }
    }

    TRACE(INFO, _b("exported memory (format=%c%c%c%c, %d planes)"), __v4l2_fmt(fmt), n);

    return 0;

error:    
    /* ...destroy all buffers exported thus far */
    while (i--)
    {
        vsp_dmabuf_unexport(mem->dmabuf[i]);
    }

    /* ...destroy DMA buffers descriptors */
    free(mem->dmabuf);
    
    return -errno;
}
