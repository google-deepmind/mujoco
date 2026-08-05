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

#include "engine/engine_environment.h"


#include <mujoco/mjmodel.h>
#include "engine/engine_util_blas.h"

// index of the layer containing height z.
//
// layer i spans [layer_height[i-1], layer_height[i]), with layer 0 unbounded
// below and layer nlayer-1 unbounded above, so a height exactly on a boundary
// belongs to the upper layer. the search is over the nlayer-1 interior
// boundaries; layer_height[nlayer-1] is the unbounded top and is never read.
//
// determinism: this function performs no floating-point arithmetic. it reads
// heights, compares, and shifts integers. there is no add or multiply on
// mjtNum, so there is nothing for fma contraction, x87 excess precision or
// vectorized reassociation to change, and the result is bit-identical across
// compilers, platforms and optimization levels. the result depends only on
// (layer_height, z): no global state, no warm start, no data-dependent
// iteration count -- the loop runs exactly ceil(log2(nlayer-1)) times.
//
// total on every input. z = +inf selects the top layer, z = -inf and z = NaN
// select layer 0, because every comparison against NaN is false and the search
// then always takes the upper branch. NaN is not rejected here: the compiler
// rejects non-finite layer heights, and a NaN query height is a caller bug
// that the surrounding physics will surface as a NaN force.
int mj_envLayer(const mjModel* m, mjtNum z) {
  // search the nlayer-1 interior boundaries. the index arithmetic is done in
  // mjtSize, the type of nlayer, so nothing is narrowed and nothing can
  // overflow: lo and hi are bounded by nlayer, which mj_makeModel already
  // bounded by what the layer arrays could be allocated for
  mjtSize lo = 0, hi = m->nlayer - 1;
  if (hi <= 0) {
    return 0;                         // also covers the nlayer == 0 sentinel
  }

  while (lo < hi) {
    mjtSize mid = lo + ((hi - lo) >> 1);
    if (m->layer_height[mid] <= z) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return (int) lo;
}



// sample the environment at a world point
void mj_envSample(const mjModel* m, const mjtNum pos[3], mjEnv* env) {
  // single medium: copy mjOption verbatim, so every consumer sees bit-identical
  // values to the pre-stratification engine
  if (m->nlayer <= 1) {
    mju_copy3(env->gravity, m->opt.gravity);
    mju_copy3(env->wind, m->opt.wind);
    env->density = m->opt.density;
    env->viscosity = m->opt.viscosity;
    return;
  }

  int i = mj_envLayer(m, pos[2]);
  mju_copy3(env->gravity, m->layer_gravity + 3*i);
  mju_copy3(env->wind, m->layer_wind + 3*i);
  env->density = m->layer_density[i];
  env->viscosity = m->layer_viscosity[i];
}
