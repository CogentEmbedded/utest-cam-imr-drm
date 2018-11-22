/*******************************************************************************
 * utest-camera-math.c
 *
 * IMR unit-test application camera math function
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

#define MODULE_TAG                      CAMERA_MATH

#include "utest-common.h"
#include "utest-camera-math.h"

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 1);

void project_world_to_camera(camera_intrinsic_t *intrinsic, float x, float y, float z, float* x_result, float* y_result)
{
    float l = sqrtf((x*x) + (y*y));
    float t = 0.0f;

    if (l != 0.0f)
    {
        t = atan2f(l, z);
        float il = 1.0f / l;
        x *= il;
        y *= il;
    }


    float t2 = t * t;
    float t3 = t * t2;
    float t5 = t3 * t2;
    float t7 = t5 * t2;
    float t9 = t7 * t2;

    float r = t + (intrinsic->d[0] * t3) + (intrinsic->d[1] * t5) + (intrinsic->d[2] * t7) + (intrinsic->d[3] * t9);

    float x_out = r * x;
    float y_out = r * y;

    *x_result = (intrinsic->fx * x_out) + intrinsic->cx;
    *y_result = (intrinsic->fy * y_out) + intrinsic->cy;
}