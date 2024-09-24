// Copyright 2022 DeepMind Technologies Limited
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

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <utility>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "sdf.h"
#include "nut.h"

namespace mujoco::plugin::sdf {
namespace {

static mjtNum distance(const mjtNum p[3], const mjtNum attributes[1]) {
  // see https://www.shadertoy.com/view/XtffzX
  mjtNum screw = 12;
  mjtNum radius2 = mju_sqrt(p[0]*p[0]+p[1]*p[1]) - attributes[0];
  mjtNum sqrt12 = mju_sqrt(2.)/2.;

  // a triangle wave spun around Oy, offset by the angle between x and z
  mjtNum azimuth = mju_atan2(p[1], p[0]);
  mjtNum triangle = abs(Fract(p[2] * screw - azimuth / mjPI / 2.) - .5);
  mjtNum thread2 = (radius2 - triangle / screw) * sqrt12;

  // clip the top
  mjtNum cone2 = (p[2] - radius2) * sqrt12;

  // the hole is the same thing, but substracted from the whole thing
  mjtNum hole = Subtraction(thread2, cone2 + .5 * sqrt12);
  hole = Union(hole, -cone2 - .05 * sqrt12);

  // create the hexagonal geometry for the head
  mjtNum point2D[2] = {p[0], p[1]};
  mjtNum res[2];
  mjtNum k = 6. / mjPI / 2.;
  mjtNum angle = -floor((mju_atan2(point2D[1], point2D[0])) * k + .5) / k;
  mjtNum s[2] = {mju_sin(angle), mju_sin(angle + mjPI * .5)};
  mjtNum mat[4] = {s[1], -s[0], s[0], s[1]};
  mju_mulMatVec(res, mat, point2D, 2, 2);
  mjtNum point3D[3] = {res[0], res[1], p[2]};
  mjtNum head = point3D[0] - .5;

  // the top is also rounded down with a cone
  head = Intersection(head, abs(point3D[2] + .25) - .25);
  head = Intersection(head, (point3D[2] + radius2 - .22) * sqrt12);
  return Subtraction(head, hole);
}

}  // namespace

// factory function
std::optional<Nut> Nut::Create(
  const mjModel* m, mjData* d, int instance) {
  if (CheckAttr("radius", m, instance)) {
    return Nut(m, d, instance);
  } else {
    mju_warning("Invalid parameter specification in Nut plugin");
    return std::nullopt;
  }
}

// plugin constructor
Nut::Nut(const mjModel* m, mjData* d, int instance) {
  SdfDefault<NutAttribute> defattribute;

  for (int i=0; i < NutAttribute::nattribute; i++) {
    attribute[i] = defattribute.GetDefault(
        NutAttribute::names[i],
        mj_getPluginConfig(m, instance, NutAttribute::names[i]));
  }
}

// plugin computation
void Nut::Compute(const mjModel* m, mjData* d, int instance) {
  visualizer_.Next();
}

// plugin reset
void Nut::Reset() {
  visualizer_.Reset();
}

// plugin visualization
void Nut::Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                    mjvScene* scn, int instance) {
  visualizer_.Visualize(m, d, opt, scn, instance);
}

// sdf
mjtNum Nut::Distance(const mjtNum point[3]) const {
  return distance(point, attribute);
}

// gradient of sdf
void Nut::Gradient(mjtNum grad[3], const mjtNum point[3]) const {
  mjtNum eps = 1e-8;
  mjtNum dist0 = distance(point, attribute);
  mjtNum pointX[3] = {point[0]+eps, point[1], point[2]};
  mjtNum distX = distance(pointX, attribute);
  mjtNum pointY[3] = {point[0], point[1]+eps, point[2]};
  mjtNum distY = distance(pointY, attribute);
  mjtNum pointZ[3] = {point[0], point[1], point[2]+eps};
  mjtNum distZ = distance(pointZ, attribute);

  grad[0] = (distX - dist0) / eps;
  grad[1] = (distY - dist0) / eps;
  grad[2] = (distZ - dist0) / eps;
}

// plugin registration
void Nut::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sdf.nut";
  plugin.capabilityflags |= mjPLUGIN_SDF;

  plugin.nattribute = NutAttribute::nattribute;
  plugin.attributes = NutAttribute::names;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto sdf_or_null = Nut::Create(m, d, instance);
    if (!sdf_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new Nut(std::move(*sdf_or_null)));
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Nut*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data,
                     int instance) {
    auto sdf = reinterpret_cast<Nut*>(plugin_data);
    sdf->Reset();
  };
  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
                         mjvScene* scn, int instance) {
    auto* sdf = reinterpret_cast<Nut*>(d->plugin_data[instance]);
    sdf->Visualize(m, d, opt, scn, instance);
  };
  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* sdf = reinterpret_cast<Nut*>(d->plugin_data[instance]);
        sdf->Compute(m, d, instance);
      };
  plugin.sdf_distance =
      +[](const mjtNum point[3], const mjData* d, int instance) {
        auto* sdf = reinterpret_cast<Nut*>(d->plugin_data[instance]);
        return sdf->Distance(point);
      };
  plugin.sdf_gradient = +[](mjtNum gradient[3], const mjtNum point[3],
                        const mjData* d, int instance) {
    auto* sdf = reinterpret_cast<Nut*>(d->plugin_data[instance]);
    sdf->visualizer_.AddPoint(point);
    sdf->Gradient(gradient, point);
  };
  plugin.sdf_staticdistance =
      +[](const mjtNum point[3], const mjtNum* attributes) {
        return distance(point, attributes);
      };
  plugin.sdf_aabb =
      +[](mjtNum aabb[6], const mjtNum* attributes) {
        aabb[0] = aabb[1] = aabb[2] = 0;
        aabb[3] = aabb[4] = .6;
        aabb[5] = 1;
      };
  plugin.sdf_attribute =
      +[](mjtNum attribute[], const char* name[], const char* value[]) {
        SdfDefault<NutAttribute> defattribute;
        defattribute.GetDefaults(attribute, name, value);
      };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::sdf
