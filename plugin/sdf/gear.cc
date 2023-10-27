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

#include <cstdio>
#include <sstream>
#include <optional>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "gear.h"

namespace mujoco::plugin::sdf {
namespace {

static mjtNum circle(mjtNum rho, mjtNum r) {
  return rho - r;
}

static mjtNum smoothUnion(mjtNum a, mjtNum b, mjtNum k) {
    mjtNum h = mju_clip(0.5 + 0.5*(b - a) / k, 0.0, 1.0);
    return b * (1. - h) + a * h - k * h * (1. - h);
}

static mjtNum smoothIntersection(mjtNum a, mjtNum b, mjtNum k) {
    return Subtraction(
        Intersection(a, b),
        smoothUnion(Subtraction(a, b), Subtraction(b, a), k));
}

static mjtNum extrusion(const mjtNum p[3], mjtNum sdf_2d, mjtNum h) {
    mjtNum w[2] = { sdf_2d, abs(p[2]) - h };
    mjtNum w_abs[2] = { mju_max(w[0], 0), mju_max(w[1], 0) };
    return mju_min(mju_max(w[0], w[1]), 0.) + mju_norm(w_abs, 2);
}

static mjtNum mod(mjtNum x, mjtNum y) {
  return x - y * floor(x/y);
}

static mjtNum distance2D(const mjtNum p[3], const mjtNum attributes[3]) {
  // see https://www.shadertoy.com/view/3lG3WR
  mjtNum D = 2.8;  // should be an attribute
  mjtNum N = 25;  // should be an attribute
  mjtNum psi = 3.096e-5 * N * N -6.557e-3 * N + 0.551;  // pressure angle
  mjtNum alpha = attributes[0];

  mjtNum R = D / 2.0;
  /* The Pitch Circle Diameter is the diameter of a circle which by a pure
   * rolling action would transmit the same motion as the actual gear wheel. It
   * should be noted that in the case of wheels which connect non-parallel
   * shafts, the pitch circle diameter is different for each cross section of
   * the wheel normal to the axis of rotation.
   */

  mjtNum rho = mju_norm(p, 2);
  mjtNum Pd = N / D;  // Diametral Pitch: teeth per unit length of diameter
  mjtNum P =
      mjPI / Pd;  // Circular Pitch: the length of arc round the pitch circle
                  // between corresponding points on adjacent teeth.
  mjtNum a = 1.0 / Pd;  // Addendum: radial length of a tooth from the pitch
                        // circle to the tip of the tooth.

  mjtNum Do = D + 2.0 * a;  // Outside Diameter
  mjtNum Ro = Do / 2.0;

  mjtNum h = 2.2 / Pd;

  mjtNum innerR = Ro - h - 0.4;

  // Early exit
  if (innerR - rho > 0.0)
      return innerR - rho;

  // Early exit
  if (Ro - rho < -0.2)
      return rho - Ro;

  mjtNum Db = D * mju_cos(psi);  // Base Diameter
  mjtNum Rb = Db / 2.0;

  mjtNum fi = mju_atan2(p[1], p[0]) + alpha;
  mjtNum alphaStride = P / R;

  mjtNum invAlpha = mju_acos(Rb / R);
  mjtNum invPhi = mju_tan(invAlpha) - invAlpha;

  mjtNum shift = alphaStride / 2.0 - 2.0 * invPhi;

  mjtNum fia = mod(fi + shift / 2.0, alphaStride) - shift / 2.0;
  mjtNum fib = mod(-fi - shift + shift / 2.0, alphaStride) - shift / 2.0;

  mjtNum dista = -1.0e6;
  mjtNum distb = -1.0e6;

  if (Rb < rho) {
      mjtNum acos_rbRho = mju_acos(Rb/rho);

      mjtNum thetaa = fia + acos_rbRho;
      mjtNum thetab = fib + acos_rbRho;

      mjtNum ta = mju_sqrt(rho * rho - Rb * Rb);

      // https://math.stackexchange.com/questions/1266689/distance-from-a-point-to-the-involute-of-a-circle
      dista = ta - Rb * thetaa;
      distb = ta - Rb * thetab;
  }

  mjtNum gearOuter = circle(rho, Ro);
  mjtNum gearLowBase = circle(rho, Ro - h);
  mjtNum crownBase = circle(rho, innerR);
  mjtNum cogs = Intersection(dista, distb);
  mjtNum baseWalls = Intersection(fia - (alphaStride - shift),
                                   fib - (alphaStride - shift));

  cogs = Intersection(baseWalls, cogs);
  cogs = smoothIntersection(gearOuter, cogs, 0.01);
  cogs = smoothUnion(gearLowBase, cogs, Rb - Ro + h);
  cogs = Subtraction(cogs, crownBase);

  return extrusion(p, cogs, .1);
}

static mjtNum distance(const mjtNum p[3], const mjtNum attributes[3]) {
  return extrusion(p, distance2D(p, attributes), .1) - .005;
}

}  // namespace

// factory function
std::optional<Gear> Gear::Create(
  const mjModel* m, mjData* d, int instance) {
  if (CheckAttr("alpha", m, instance)) {
    return Gear(m, d, instance);
  } else {
    mju_warning("Invalid parameter specification in Gear plugin");
    return std::nullopt;
  }
}

// plugin constructor
Gear::Gear(const mjModel* m, mjData* d, int instance) {
  alpha = strtod(mj_getPluginConfig(m, instance, "alpha"), nullptr);
}

// plugin computation
void Gear::Compute(const mjModel* m, mjData* d, int instance) {
  visualizer_.Next();
}

// plugin reset
void Gear::Reset() {
  visualizer_.Reset();
}

// plugin visualization
void Gear::Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                     mjvScene* scn, int instance) {
  visualizer_.Visualize(m, d, opt, scn, instance);
}

// sdf
mjtNum Gear::Distance(const mjtNum point[3]) const {
  return distance(point, &alpha);
}

// gradient of sdf
void Gear::Gradient(mjtNum grad[3], const mjtNum point[3]) const {
  mjtNum attributes[1]= {alpha};
  mjtNum eps = 1e-8;
  mjtNum dist0 = distance(point, attributes);

  mjtNum pointX[3] = {point[0]+eps, point[1], point[2]};
  mjtNum distX = distance(pointX, attributes);
  mjtNum pointY[3] = {point[0], point[1]+eps, point[2]};
  mjtNum distY = distance(pointY, attributes);
  mjtNum pointZ[3] = {point[0], point[1], point[2]+eps};
  mjtNum distZ = distance(pointZ, attributes);

  grad[0] = (distX - dist0) / eps;
  grad[1] = (distY - dist0) / eps;
  grad[2] = (distZ - dist0) / eps;
}

// plugin registration
void Gear::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sdf.gear";
  plugin.capabilityflags |= mjPLUGIN_SDF;

  const char* attributes[] = {"alpha"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto sdf_or_null = Gear::Create(m, d, instance);
    if (!sdf_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new Gear(std::move(*sdf_or_null)));
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Gear*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.reset = +[](const mjModel* m, double* plugin_state, void* plugin_data,
                     int instance) {
    auto sdf = reinterpret_cast<Gear*>(plugin_data);
    sdf->Reset();
  };
  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
                         mjvScene* scn, int instance) {
    auto* sdf = reinterpret_cast<Gear*>(d->plugin_data[instance]);
    sdf->Visualize(m, d, opt, scn, instance);
  };
  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* sdf = reinterpret_cast<Gear*>(d->plugin_data[instance]);
        sdf->Compute(m, d, instance);
      };
  plugin.sdf_distance =
      +[](const mjtNum point[3], const mjData* d, int instance) {
        auto* sdf = reinterpret_cast<Gear*>(d->plugin_data[instance]);
        sdf->visualizer_.AddPoint(point);
        return sdf->Distance(point);
      };
  plugin.sdf_gradient = +[](mjtNum gradient[3], const mjtNum point[3],
                        const mjData* d, int instance) {
    auto* sdf = reinterpret_cast<Gear*>(d->plugin_data[instance]);
    sdf->Gradient(gradient, point);
  };
  plugin.sdf_staticdistance =
      +[](const mjtNum point[3], const mjtNum* attributes) {
        return distance(point, attributes);
      };
  plugin.sdf_aabb =
      +[](mjtNum aabb[6], const mjtNum* attributes) {
        aabb[0] = aabb[1] = aabb[2] = 0;
        aabb[3] = aabb[4] = 1.7;
        aabb[5] = .11;
      };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::sdf
