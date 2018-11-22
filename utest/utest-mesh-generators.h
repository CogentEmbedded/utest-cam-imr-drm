/*******************************************************************************
 * utest-mesh-generators.h
 *
 * IMR unit-test application mesh generators function definition
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

#ifndef MESH_GENERATORS
#define MESH_GENERATORS

#include "utest-app.h"
#include "utest-imr.h"

int __distorted_cfg_setup(camera_intrinsic_t *intrinsic,
                            int id,
                            int w, int h,
                            int camera_threshold,
                            int camera_buffer_shift,
                            int x_start, int y_start,
                            int x_end, int y_end,
                            int hfov, int vfov_bottom_part,
                            int vfov_upper_part,
                            int block_size,
                            imr_cfg_t *cfg_left,
                            imr_cfg_t *cfg_right);

int __undistorted_cfg_setup(app_data_t *app, camera_intrinsic_t *intrinsic, int id, int w, int h, int x_start, int y_start, int x_end, int y_end, imr_cfg_t *cfg);
#endif //MESH_GENERATORS