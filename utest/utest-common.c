/*******************************************************************************
 * utest-common.c
 *
 * Common helpers for a unit-test application
 *
 * Copyright (c) 2014 Cogent Embedded Inc. ALL RIGHTS RESERVED.
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

#define MODULE_TAG                      COMMON

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "utest-common.h"

#include <fcntl.h>
#include <sys/timerfd.h>

#include <sys/syscall.h>
#include <sys/types.h>

#define gettid()    syscall(SYS_gettid)

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(DEBUG, 0);

/*******************************************************************************
 * Tracing facility
 ******************************************************************************/

#if INTERN_TRACE
/*******************************************************************************
 * Trace function definition
 ******************************************************************************/

/* ...tracing lock */
static pthread_mutex_t  intern_trace_mutex;
static int              intern_trace_fd;

/* ...tracing to communication processor */
int intern_trace(const char *format, ...)
{
    va_list             args;
    struct timespec     ts;
    static char         buffer[4096];
    char               *p = buffer;
    int                 n = sizeof(buffer), k;
    
    /* ...retrieve value of monotonic clock */
    clock_gettime(CLOCK_MONOTONIC, &ts);

    /* ...get global tracing lock */
    pthread_mutex_lock(&intern_trace_mutex);

    /* ...output timestamp */
    //k = snprintf(p, n, "[%02u.%06u]:%d: ", (u32)ts.tv_sec, (u32)ts.tv_nsec / 1000, sched_getcpu());
    k = snprintf(p, n, "[%02u.%06u]: ", (u32)ts.tv_sec, (u32)ts.tv_nsec / 1000);
    p += k, n -= k;
    
    /* ...output format string */
    va_start(args, format);
    k = vsnprintf(p, n, format, args);
    p += k, n -= k;
    va_end(args);

    /* ...output string terminator */
    (n > 0 ? *p++ = '\n' : 0);

    write(intern_trace_fd, buffer, p - buffer);
    
    /* ...release tracing lock */
    pthread_mutex_unlock(&intern_trace_mutex);

    return 0;
}

/* ...tracing facility initialization */
void intern_trace_init(const char *banner)
{
    /* ...initialize tracing lock */
    pthread_mutex_init(&intern_trace_mutex, NULL);

    if (!getenv("IMR_LOG_KMSG") || (intern_trace_fd = open("/dev/kmsg", O_WRONLY)) < 0)
    {
        intern_trace_fd = STDOUT_FILENO;
    }
    
    /* ...output banner */
    intern_trace("%s", banner);
}

#endif  /* INTERN_TRACE */
