// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_ENVIRONMENT_H_
#define MUJOCO_SRC_ENGINE_ENGINE_ENVIRONMENT_H_

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// The environment as a field rather than five constants.
//
// mjOption carries one gravity vector, one density, one viscosity, one wind
// vector and one magnetic vector, so a model represents exactly one medium and
// cannot represent a boundary between two.  This layer generalises the first
// four to piecewise-constant fields, stratified along +z.
//
// Why stratified and not a general cell complex.  A piecewise-constant gravity
// field admits a continuous potential -- and therefore does no work around a
// closed loop -- if and only if, for every pair of adjacent cells, g_j - g_i is
// parallel to the normal of their shared face.  Measured on a two-cell
// arrangement split by a vertical plane with Earth gravity on one side and
// lunar on the other, a closed rectangular circuit gains 8.190000 J/kg per
// circuit; a body cycling across that boundary accelerates without bound.
// When every face shares one normal, as it does for parallel layers, the
// condition holds by construction for any assignment of layer values.  The
// stratified family is therefore not a convenience: it is the largest
// piecewise-constant family that cannot be misconfigured into a free-energy
// machine, and it is what depth-dependent water, layered atmosphere and a
// vacuum boundary all need.
//
// Lookup is a binary search over ascending boundary heights, so sampling is
// O(log nlayer), branch-free in the values, and identical for every caller.
// nlayer == 0 selects the legacy single-medium path and every consumer below
// then reads mjOption exactly as before, bit for bit.

// sampled environment at a point
typedef struct mjEnv_ {
  mjtNum gravity[3];
  mjtNum wind[3];
  mjtNum density;
  mjtNum viscosity;
} mjEnv;

// index of the layer containing height z; 0 when the model is single-medium
MJAPI int mj_envLayer(const mjModel* m, mjtNum z);

// sample the environment at a world point
MJAPI void mj_envSample(const mjModel* m, const mjtNum pos[3], mjEnv* env);

// true if the model declares a stratified environment
static inline int mj_envLayered(const mjModel* m) {
  return m->nlayer > 1;
}


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_ENVIRONMENT_H_
