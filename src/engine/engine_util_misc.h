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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_MISC_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_MISC_H_

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------ tendons and actuators ---------------------------------------------

// wrap tendons around spheres and cylinders
mjtNum mju_wrap(mjtNum* wpnt, const mjtNum* x0, const mjtNum* x1,
                const mjtNum* xpos, const mjtNum* xmat, const mjtNum* size,
                int type, const mjtNum* side);

// muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)
MJAPI mjtNum mju_muscleGain(mjtNum len, mjtNum vel, const mjtNum lengthrange[2],
                            mjtNum acc0, const mjtNum prm[9]);

// muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)
MJAPI mjtNum mju_muscleBias(mjtNum len, const mjtNum lengthrange[2],
                            mjtNum acc0, const mjtNum prm[9]);

// muscle activation dynamics, prm = (tau_act, tau_deact)
MJAPI mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, const mjtNum prm[2]);

// all 3 semi-axes of a geom
MJAPI void mju_geomSemiAxes(const mjModel* m, int geom_id, mjtNum semiaxes[3]);

//------------------------------ misclellaneous ----------------------------------------------------

// convert contact force to pyramid representation
MJAPI void mju_encodePyramid(mjtNum* pyramid, const mjtNum* force,
                             const mjtNum* mu, int dim);

// convert pyramid representation to contact force
MJAPI void mju_decodePyramid(mjtNum* force, const mjtNum* pyramid,
                             const mjtNum* mu, int dim);

// integrate spring-damper analytically, return pos(dt)
MJAPI mjtNum mju_springDamper(mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt);

// print matrix
MJAPI void mju_printMat(const mjtNum* mat, int nr, int nc);

// print sparse matrix to screen
MJAPI void mju_printMatSparse(const mjtNum* mat, int nr,
                              const int* rownnz, const int* rowadr,
                              const int* colind);

// min function, single evaluation of a and b
MJAPI mjtNum mju_min(mjtNum a, mjtNum b);

// max function, single evaluation of a and b
MJAPI mjtNum mju_max(mjtNum a, mjtNum b);

// clip x to the range [min, max]
MJAPI mjtNum mju_clip(mjtNum x, mjtNum min, mjtNum max);

// sign function
MJAPI mjtNum mju_sign(mjtNum x);

// round to nearest integer
MJAPI int mju_round(mjtNum x);

// convert type id (mjtObj) to type name
MJAPI const char* mju_type2Str(int type);

// convert type name to type id (mjtObj)
MJAPI int mju_str2Type(const char* str);

// warning text
MJAPI const char* mju_warningText(int warning, int info);

// return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise
MJAPI int mju_isBad(mjtNum x);

// return 1 if all elements are 0
MJAPI int mju_isZero(mjtNum* vec, int n);

// standard normal random number generator (optional second number)
MJAPI mjtNum mju_standardNormal(mjtNum* num2);

// convert from float to mjtNum
MJAPI void mju_f2n(mjtNum* res, const float* vec, int n);

// convert from mjtNum to float
MJAPI void mju_n2f(float* res, const mjtNum* vec, int n);

// convert from double to mjtNum
MJAPI void mju_d2n(mjtNum* res, const double* vec, int n);

// convert from mjtNum to double
MJAPI void mju_n2d(double* res, const mjtNum* vec, int n);

// insertion sort, increasing order
MJAPI void mju_insertionSort(mjtNum* list, int n);

// integer insertion sort, increasing order
MJAPI void mju_insertionSortInt(int* list, int n);

// Halton sequence
MJAPI mjtNum mju_Halton(int index, int base);

// Call strncpy, then set dst[n-1] = 0.
MJAPI char* mju_strncpy(char *dst, const char *src, int n);

// Sigmoid function over 0<=x<=1 constructed from half-quadratics.
MJAPI mjtNum mju_sigmoid(mjtNum x);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_MISC_H_
