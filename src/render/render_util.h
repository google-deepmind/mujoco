// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THIRD_PARTY_MUJOCO_SRC_RENDER_RENDER_UTIL_
#define THIRD_PARTY_MUJOCO_SRC_RENDER_RENDER_UTIL_

#include <mujoco/mjexport.h>
#include <mujoco/mjrender.h>

#if defined(__cplusplus)
extern "C" {
#endif

// compute normal vector to given triangle, return length
void mjr_makeNormal(float* normal, const float* p1, const float* p2, const float* p3);

// set 4 floats
void mjr_setf4(float* vec, float f0, float f1, float f2, float f3);

// set 3 floats
void mjr_setf3(float* vec, float f0, float f1, float f2);

// multiply 4-by-4 matrices, column-major
void mjr_mulMat44(float* res, const float* A, const float* B);

// get row from 4-by-4 matrix, column-major
void mjr_getrow4(float* res, const float* A, int r);

// compute 3D vector cross-product a = b x c
void mjr_crossVec(float* a, const float* b, const float* c);

// normalize 3D vector
void mjr_normalizeVec(float* v);

// construct orthogonal vector (for up-direction)
void mjr_orthoVec(float* res, const float* v);

// compute dot-product
float mjr_dotVec(const float* a, const float* b);

// multiply 4x4 matrix by vector; column-major (OpenGL format)
void mjr_multiply4(float* res, const float* mat, const float* vec);

// gluLookAt replacement, with forward instead of center
void mjr_lookAt(const float* eye, const float* forward, const float* up);

// gluPerspective replacement
void mjr_perspective(float fovy, float aspect, float znear, float zfar);

// set reflection matrix
void mjr_reflect(const float* pos, const float* mat);

// set transformation matrix
void mjr_transform(const float* translate, const float* rotate, float scale);

// Find first rectangle containing mouse, -1: not found.
MJAPI int mjr_findRect(int x, int y, int nrect, const mjrRect* rect);

#if defined(__cplusplus)
}
#endif
#endif  // THIRD_PARTY_MUJOCO_SRC_RENDER_RENDER_UTIL_
