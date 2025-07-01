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
#include <mujoco/mjtnum.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

//------------------------------ tendons and actuators ---------------------------------------------

// wrap tendons around spheres and cylinders
mjtNum mju_wrap(mjtNum wpnt[6], const mjtNum x0[3], const mjtNum x1[3], const mjtNum xpos[3],
                const mjtNum xmat[9], mjtNum radius, int type, const mjtNum side[3]);

// normalized muscle length-gain curve
MJAPI mjtNum mju_muscleGainLength(mjtNum length, mjtNum lmin, mjtNum lmax);

// muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)
MJAPI mjtNum mju_muscleGain(mjtNum len, mjtNum vel, const mjtNum lengthrange[2],
                            mjtNum acc0, const mjtNum prm[9]);

// muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)
MJAPI mjtNum mju_muscleBias(mjtNum len, const mjtNum lengthrange[2],
                            mjtNum acc0, const mjtNum prm[9]);

// muscle time constant with optional smoothing
MJAPI mjtNum mju_muscleDynamicsTimescale(mjtNum dctrl, mjtNum tau_act, mjtNum tau_deact,
                                         mjtNum smoothing_width);

// muscle activation dynamics, prm = (tau_act, tau_deact, smoothing_width)
MJAPI mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, const mjtNum prm[3]);

// all 3 semi-axes of a geom
MJAPI void mju_geomSemiAxes(const mjModel* m, int geom_id, mjtNum semiaxes[3]);

// ----------------------------- Flex interpolation ------------------------------------------------

// evaluate the deformation gradient at p using the nodal dof values
MJAPI void mju_defGradient(mjtNum res[9], const mjtNum p[3], const mjtNum* dof, int order);

// ----------------------------- Base64 -----------------------------------------------------------

// encode data as Base64 into buf (including padding and null char)
// returns number of chars written in buf: 4 * [(ndata + 2) / 3] + 1
MJAPI size_t mju_encodeBase64(char* buf, const uint8_t* data, size_t ndata);

// return size in decoded bytes if s is a valid Base64 encoding
// return 0 if s is empty or invalid Base64 encoding
MJAPI size_t mju_isValidBase64(const char* s);

// decode valid Base64 in string s into buf, undefined behavior if s is not valid Base64
// returns number of bytes decoded (upper limit of 3 * (strlen(s) / 4))
MJAPI size_t mju_decodeBase64(uint8_t* buf, const char* s);

//------------------------------ miscellaneous ----------------------------------------------------

// convert contact force to pyramid representation
MJAPI void mju_encodePyramid(mjtNum* pyramid, const mjtNum* force,
                             const mjtNum* mu, int dim);

// convert pyramid representation to contact force
MJAPI void mju_decodePyramid(mjtNum* force, const mjtNum* pyramid,
                             const mjtNum* mu, int dim);

// integrate spring-damper analytically, return pos(dt)
MJAPI mjtNum mju_springDamper(mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt);

// return 1 if point is outside box given by pos, mat, size * inflate
// return -1 if point is inside box given by pos, mat, size / inflate
// return 0 if point is between the inflated and deflated boxes
MJAPI int mju_outsideBox(const mjtNum point[3], const mjtNum pos[3], const mjtNum mat[9],
                         const mjtNum size[3], mjtNum inflate);

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

// return human readable number of bytes using standard letter suffix
MJAPI const char* mju_writeNumBytes(size_t nbytes);

// warning text
MJAPI const char* mju_warningText(int warning, size_t info);

// return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise
MJAPI int mju_isBad(mjtNum x);

// return 1 if all elements are 0
MJAPI int mju_isZero(mjtNum* vec, int n);

// set integer vector to 0
MJAPI void mju_zeroInt(int* res, int n);

// set size_t vector to 0
MJAPI void mju_zeroSizeT(size_t* res, size_t n);

// copy int vector vec into res
MJAPI void mju_copyInt(int* res, const int* vec, int n);

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

// gather mjtNums
MJAPI void mju_gather(mjtNum* res, const mjtNum* vec, const int* ind, int n);

// scatter mjtNums
MJAPI void mju_scatter(mjtNum* res, const mjtNum* vec, const int* ind, int n);

// gather integers
MJAPI void mju_gatherInt(int* res, const int* vec, const int* ind, int n);

// scatter integers
MJAPI void mju_scatterInt(int* res, const int* vec, const int* ind, int n);

// insertion sort, increasing order
MJAPI void mju_insertionSort(mjtNum* list, int n);

// integer insertion sort, increasing order
MJAPI void mju_insertionSortInt(int* list, int n);

// Halton sequence
MJAPI mjtNum mju_Halton(int index, int base);

// call strncpy, then set dst[n-1] = 0
MJAPI char* mju_strncpy(char *dst, const char *src, int n);

// sigmoid function over 0<=x<=1 using quintic polynomial
MJAPI mjtNum mju_sigmoid(mjtNum x);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_MISC_H_
