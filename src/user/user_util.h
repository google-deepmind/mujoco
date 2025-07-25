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

#ifndef MUJOCO_SRC_USER_USER_UTIL_H_
#define MUJOCO_SRC_USER_USER_UTIL_H_

#include <optional>
#include <string>
#include <string_view>


const double mjEPS = 1E-14;                     // minimum value in various calculations
const double mjMINMASS = 1E-6;                  // minimum mass allowed

// check if numeric variable is defined:  !_isnan(num)
bool mjuu_defined(const double num);

// compute linear address of M[g1][g2] where M is triangular n-by-n
// return -1 if inputs are invalid
int mjuu_matadr(int g1, int g2, const int n);

// set 4D vector
void mjuu_setvec(double* dest, const double x, const double y, const double z, const double w);
void mjuu_setvec(float* dest, const double x, const double y, const double z, const double w);

// set 3D vector
void mjuu_setvec(double* dest, const double x, const double y, const double z);
void mjuu_setvec(float* dest, const double x, const double y, const double z);

// set 2D vector
void mjuu_setvec(double* dest, const double x, const double y);

// copy double array
void mjuu_copyvec(double* dest, const double* src, const int n);

// copy float array
void mjuu_copyvec(float* dest, const float* src, const int n);

// zero array
void mjuu_zerovec(double* dest, const int n);

// dot-product in 3D
double mjuu_dot3(const double* a, const double* b);

// distance beween 3D points
double mjuu_dist3(const double* a, const double* b);

// L1 norm between vectors
double mjuu_L1(const double* a, const double* b, int n);

// normalize vector to unit length, return previous length
//  if norm(vec)<mjEPS, return 0 and do not change vector
double mjuu_normvec(double* vec, const int n);

// convert quaternion to rotation matrix
void mjuu_quat2mat(double* res, const double* quat);

// multiply two unit quaternions
void mjuu_mulquat(double* res, const double* qa, const double* qb);

// multiply vector by matrix, 3-by-3
void mjuu_mulvecmat(double* res, const double* vec, const double* mat);

// compute res = R * M * R'
void mjuu_mulRMRT(double* res, const double* R, const double* M);

// compute res = A * B, all 3-by-3
void mjuu_mulmat(double* res, const double* A, const double* B);

// transpose 3-by-3 matrix
void mjuu_transposemat(double* res, const double* mat);

// convert global to local axis relative to given frame
void mjuu_localaxis(double* al, const double* ag, const double* quat);

// convert global to local position relative to given frame
void mjuu_localpos(double* pl, const double* pg, const double* pos, const double* quat);

// compute quaternion rotation from parent to child
void mjuu_localquat(double* local, const double* child, const double* parent);

// compute vector cross-product a = b x c
void mjuu_crossvec(double* a, const double* b, const double* c);

// compute normal vector to given triangle (uses float for OpenGL)
double mjuu_makenormal(double* normal, const float* a, const float* b, const float* c);

// compute quaternion corresponding to minimal rotation from [0;0;1] to vec
void mjuu_z2quat(double* quat, const double* vec);

// compute quaternion corresponding to given rotation matrix (i.e. frame)
void mjuu_frame2quat(double* quat, const double* x, const double* y, const double* z);

// invert frame transformation
void mjuu_frameinvert(double newpos[3], double newquat[4],
                      const double oldpos[3], const double oldquat[4]);

// accumulate frame transformation into parent frame
void mjuu_frameaccum(double pos[3], double quat[4],
                     const double childpos[3], const double childquat[4]);

// accumulate frame transformation into child frame
void mjuu_frameaccumChild(const double pos[3], const double quat[4],
                          double childpos[3], double childquat[4]);

// invert frame accumulation
void mjuu_frameaccuminv(double pos[3], double quat[4],
                        const double childpos[3], const double childquat[4]);

// convert local_inertia[3] to global_inertia[6]
void mjuu_globalinertia(double* global, const double* local, const double* quat);

// compute off-center correction to inertia matrix
void mjuu_offcenter(double* res, const double mass, const double* vec);

// compute viscosity coefficients from mass and inertia
void mjuu_visccoef(double* visccoef, double mass, const double* inertia, double scl=1);

// update moving frame along a discrete curve or initialize it, returns edge length
//   inputs:
//     normal    - normal vector computed by a previous call to the function
//     edge      - edge vector (non-unit tangent vector)
//     tprv      - unit tangent vector of previous body
//     tnxt      - unit tangent vector of next body
//     first     - 1 if the frame requires initialization
//   outputs:
//     quat      - frame orientation
//     normal    - unit normal vector
double mju_updateFrame(double quat[4], double normal[3], const double edge[3],
                       const double tprv[3], const double tnxt[3], int first);

// strip path from filename
std::string mjuu_strippath(std::string filename);

// strip extension from filename
std::string mjuu_stripext(std::string filename);

// get the extension of a filename
std::string mjuu_getext(std::string_view filename);

// check if path is absolute
bool mjuu_isabspath(std::string path);

// assemble full filename
std::string mjuu_makefullname(std::string filedir, std::string meshdir, std::string filename);

// return type from content_type format {type}/{subtype}[;{parameter}={value}]
std::optional<std::string_view> mjuu_parseContentTypeAttrType(std::string_view text);

// return subtype from content_type format {type}/{subtype}[;{parameter}={value}]
std::optional<std::string_view> mjuu_parseContentTypeAttrSubtype(std::string_view text);

// convert filename extension to content type; return empty string if not found
std::string mjuu_extToContentType(std::string_view filename);

#endif  // MUJOCO_SRC_USER_USER_UTIL_H_
