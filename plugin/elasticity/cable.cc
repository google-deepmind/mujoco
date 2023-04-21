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

#include <algorithm>
#include <cstddef>
#include <sstream>
#include <optional>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "cable.h"


namespace mujoco::plugin::elasticity {
namespace {

// Jet color palette
void scalar2rgba(float rgba[4], mjtNum stress[3], mjtNum vmin, mjtNum vmax) {
  // L2 norm of the stress
  mjtNum v = mju_norm3(stress);
  v = v < vmin ? vmin : v;
  v = v > vmax ? vmax : v;
  mjtNum dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)) {
    rgba[0] = 0;
    rgba[1] = 4 * (v - vmin) / dv;
    rgba[2] = 1;
  } else if (v < (vmin + 0.5 * dv)) {
    rgba[0] = 0;
    rgba[1] = 1;
    rgba[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    rgba[0] = 4 * (v - vmin - 0.5 * dv) / dv;
    rgba[1] = 1;
    rgba[2] = 0;
  } else {
    rgba[0] = 1;
    rgba[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    rgba[2] = 0;
  }
}

// compute quaternion difference between two frames in joint coordinates
void QuatDiff(mjtNum* quat, const mjtNum body_quat[4],
              const mjtNum joint_quat[4], bool pullback = false) {
  if (pullback == 0) {
    // contribution in local coordinates
    mju_mulQuat(quat, body_quat, joint_quat);
  } else {
    // contribution pulled-back in local coordinates of the other body
    mjtNum invquat[4];
    mju_mulQuat(invquat, body_quat, joint_quat);
    mju_negQuat(quat, invquat);
  }
}

// compute local stress given material properties, orientation,
// and reference curvature
//   inputs:
//     stiffness   - material parameters
//     quat        - orientation of the body in local coordinates
//     omega0      - initial curvature
//     xquat       - Cartesian orientation of the body (optional)
//     scl         - scaling of the force
//   outputs:
//     stress      - local stress contribution
void LocalStress(mjtNum stress[3],
                const mjtNum stiffness[4],
                const mjtNum quat[4], const mjtNum omega0[3],
                bool pullback = false) {
  mjtNum omega[3];

  // compute curvature
  mju_quat2Vel(omega, quat, 1.0);

  // subtract omega0 in reference configuration
  mjtNum tmp[] = {
      - stiffness[0]*(omega[0] - omega0[0]) / stiffness[3],
      - stiffness[1]*(omega[1] - omega0[1]) / stiffness[3],
      - stiffness[2]*(omega[2] - omega0[2]) / stiffness[3],
  };


  // pull-back into the other body frame
  if (pullback) {
    mjtNum invquat[4];
    mju_negQuat(invquat, quat);
    mju_rotVecQuat(stress, tmp, invquat);
  } else {
    mju_copy3(stress, tmp);
  }
}

// reads numeric attributes
bool CheckAttr(const char* name, const mjModel* m, int instance) {
  char *end;
  std::string value = mj_getPluginConfig(m, instance, name);
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

}  // namespace


// factory function
std::optional<Cable> Cable::Create(
  const mjModel* m, mjData* d, int instance) {
  if (CheckAttr("twist", m, instance) && CheckAttr("bend", m, instance)) {
    return Cable(m, d, instance);
  } else {
    mju_warning("Invalid parameter specification in cable plugin");
    return std::nullopt;
  }
}

// plugin constructor
Cable::Cable(const mjModel* m, mjData* d, int instance) {
  // parameters were validated by the factor function
  std::string flat = mj_getPluginConfig(m, instance, "flat");
  mjtNum G = strtod(mj_getPluginConfig(m, instance, "twist"), nullptr);
  mjtNum E = strtod(mj_getPluginConfig(m, instance, "bend"), nullptr);
  vmax = strtod(mj_getPluginConfig(m, instance, "vmax"), nullptr);
  // count plugin bodies
  n = 0;
  for (int i = 1; i < m->nbody; i++) {
    if (m->body_plugin[i] == instance) {
      if (!n++) {
        i0 = i;
      }
    }
  }

  // allocate arrays
  prev.assign(n, 0);         // index of previous body
  next.assign(n, 0);         // index of next body
  omega0.assign(3*n, 0);     // reference curvature
  stress.assign(3*n, 0);     // mechanical stress
  stiffness.assign(4*n, 0);  // material parameters

  // run forward kinematics to populate xquat (mjData not yet initialized)
  mju_zero(d->mocap_quat, 4*m->nmocap);
  mju_copy(d->qpos, m->qpos0, m->nq);
  mj_kinematics(m, d);

  // compute initial curvature
  for (int b = 0; b < n; b++) {
    int i = i0 + b;
    if (m->body_plugin[i] != instance) {
      mju_error("This body does not have the requested plugin instance");
    }
    bool first = (b == 0), last = (b == n-1);
    prev[b] = first ? 0 : -1;
    next[b] =  last ? 0 : +1;

    // compute omega0: curvature at equilibrium
    if (prev[b] && flat != "true") {
      int qadr = m->jnt_qposadr[m->body_jntadr[i]] + m->body_dofnum[i]-3;
      mju_subQuat(omega0.data()+3*b, m->body_quat+4*i, d->qpos+qadr);
    } else {
      mju_zero3(omega0.data()+3*b);
    }

    // compute physical parameters
    int geom_i = m->body_geomadr[i];
    mjtNum J = 0, Iy = 0, Iz = 0;
    if (m->geom_type[geom_i] == mjGEOM_CYLINDER ||
        m->geom_type[geom_i] == mjGEOM_CAPSULE) {
      // https://en.wikipedia.org/wiki/Torsion_constant#Circle
      // https://en.wikipedia.org/wiki/List_of_second_moments_of_area
      J = mjPI * pow(m->geom_size[3*geom_i+0], 4) / 2;
      Iy = Iz = mjPI * pow(m->geom_size[3*geom_i+0], 4) / 4.;
    } else if (m->geom_type[geom_i] == mjGEOM_BOX) {
      // https://en.wikipedia.org/wiki/Torsion_constant#Rectangle
      // https://en.wikipedia.org/wiki/List_of_second_moments_of_area
      mjtNum h = m->geom_size[3*geom_i+1];
      mjtNum w = m->geom_size[3*geom_i+2];
      mjtNum a = std::max(h, w);
      mjtNum b = std::min(h, w);
      J = a*pow(b, 3)*(16./3.-3.36*b/a*(1-pow(b, 4)/pow(a, 4)/12));
      Iy = pow(2 * w, 3) * 2 * h / 12.;
      Iz = pow(2 * h, 3) * 2 * w / 12.;
    }
    stiffness[4*b+0] = J * G;
    stiffness[4*b+1] = Iy * E;
    stiffness[4*b+2] = Iz * E;
    stiffness[4*b+3] =
      prev[b] ? mju_dist3(d->xpos+3*i, d->xpos+3*(i+prev[b])) : 0;
  }
}

void Cable::Compute(const mjModel* m, mjData* d, int instance) {
  for (int b = 0; b < n; b++)  {
    // index into body array
    int i = i0 + b;
    if (m->body_plugin[i] != instance) {
      mju_error(
        "This body is not associated with the requested plugin instance");
    }

    // if no stiffness, skip body
    if (!stiffness[b*4+0] && !stiffness[b*4+1] && !stiffness[b*4+2]) {
      continue;
    }

    // elastic forces
    mjtNum quat[4] = {0};
    mjtNum lfrc[3] = {0};

    // local orientation
    if (prev[b]) {
      int qadr = m->jnt_qposadr[m->body_jntadr[i]] + m->body_dofnum[i]-3;
      QuatDiff(quat, m->body_quat+4*i, d->qpos+qadr);

      // contribution of orientation i-1 to xfrc i
      LocalStress(stress.data() + 3 * b, stiffness.data() + 4 * b, quat,
                  omega0.data() + 3 * b, true);
      mju_addToScl3(lfrc, stress.data() + 3 * b, 1.0);
    }

    if (next[b]) {
      int bn = b + next[b];
      int in = i + next[b];

      // local orientation
      int qadr = m->jnt_qposadr[m->body_jntadr[in]] + m->body_dofnum[in]-3;
      QuatDiff(quat, m->body_quat+4*in, d->qpos+qadr);

      // contribution of orientation i+1 to xfrc i
      LocalStress(stress.data() + 3 * bn, stiffness.data() + 4 * bn, quat,
                  omega0.data() + 3 * bn);
      mju_addToScl3(lfrc, stress.data() + 3 * bn, -1.0);
    }

    // convert from global coordinates and apply torque to com
    mjtNum xfrc[3] = {0};
    mju_rotVecQuat(xfrc, lfrc, d->xquat+4*i);
    mj_applyFT(m, d, 0, xfrc, d->xpos+3*i, i, d->qfrc_passive);
  }
}

void Cable::Visualize(const mjModel* m, mjData* d, mjvScene* scn,
                      int instance) {
  if (!vmax) {
    return;
  }

  for (int b = 0; b < n; b++)  {
    int i = i0 + b;
    int bn = b + next[b];

    // set geometry color based on stress norm
    mjtNum stress_m[3] = {0};
    mjtNum *stress_l = prev[b] ? stress.data()+3*b : stress.data()+3*bn;
    mjtNum *stress_r = next[b] ? stress.data()+3*bn : stress.data()+3*b;
    mju_add3(stress_m, stress_l, stress_r);
    mju_scl3(stress_m, stress_m, 0.5);
    scalar2rgba(m->geom_rgba + 4*m->body_geomadr[i], stress_m, 0, vmax);
  }
}

void Cable::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.cable";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"twist", "bend", "flat", "vmax"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto elasticity_or_null = Cable::Create(m, d, instance);
    if (!elasticity_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new Cable(std::move(*elasticity_or_null)));
return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Cable*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* elasticity = reinterpret_cast<Cable*>(d->plugin_data[instance]);
        elasticity->Compute(m, d, instance);
      };
  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn,
                         int instance) {
    auto* elasticity = reinterpret_cast<Cable*>(d->plugin_data[instance]);
    elasticity->Visualize(m, d, scn, instance);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::elasticity
