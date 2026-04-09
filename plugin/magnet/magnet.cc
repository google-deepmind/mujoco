// Copyright 2023 DeepMind Technologies Limited
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

#include <iostream>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <fstream>
#include <unordered_map>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

#include "magnet.h"

namespace mujoco::plugin::passive {

namespace {

void mjv_addLine(mjvScene* scn, const mjtNum from[3], const mjtNum to[3], const std::string& color) {
  mjvGeom line_geom;
  mjv_initGeom(&line_geom, mjGEOM_LINE, nullptr, nullptr, nullptr, nullptr);

  line_geom.type = mjGEOM_LINE;

  if (color == "red") {
    line_geom.rgba[0] = 1.0f;
    line_geom.rgba[1] = 0.0f;
    line_geom.rgba[2] = 0.0f;
  } else if (color == "green") {
    line_geom.rgba[0] = 0.0f;
    line_geom.rgba[1] = 1.0f;
    line_geom.rgba[2] = 0.0f;
  } else if (color == "blue") {
    line_geom.rgba[0] = 0.0f;
    line_geom.rgba[1] = 0.0f;
    line_geom.rgba[2] = 1.0f;
  } else if (color == "yellow") {
    line_geom.rgba[0] = 1.0f;
    line_geom.rgba[1] = 1.0f;
    line_geom.rgba[2] = 0.0f;
  } else if (color == "purple") {
    line_geom.rgba[0] = 0.5f;
    line_geom.rgba[1] = 0.0f;
    line_geom.rgba[2] = 0.5f;
  } else if (color == "cyan") {
    line_geom.rgba[0] = 0.0f;
    line_geom.rgba[1] = 1.0f;
    line_geom.rgba[2] = 1.0f;
  }
  line_geom.rgba[3] = 1.0f;

  constexpr mjtNum line_width_pixels = 20;
  mjv_connector(&line_geom, mjGEOM_LINE, line_width_pixels, from, to);
  scn->geoms[scn->ngeom++] = line_geom;
}

std::optional<mjtNum> ReadDoubleAttr(const mjModel* m, const int instance, const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;
  }
  return std::strtod(value, nullptr);
}

std::optional<std::array<mjtNum, 3>> ReadDoubleArrayAttr(const mjModel* m, const int instance, const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;
  }
  std::array<mjtNum, 3> vec;
  char* end;
  for (int i = 0; i < 3; i++) {
    vec[i] = std::strtod(value, &end);
    if (value == end) {
      // Not enough numbers
      return std::nullopt;
    }
    value = end;
  }
  return vec;
}

}  // namespace

MagneticFieldConfig MagneticFieldConfig::FromModel(const mjModel* m, int instance) {
  MagneticFieldConfig config;

  // Magnetic field at the origin is an optional field (defaults to zero)
  std::optional<std::array<mjtNum, 3>> field_opt = ReadDoubleArrayAttr(m, instance, "B0");
  if (field_opt) {
    config.B0[0] = field_opt.value()[0];
    config.B0[1] = field_opt.value()[1];
    config.B0[2] = field_opt.value()[2];
  }

  // Magnetic field gradient is an optional field (defaults to zero)
  std::optional<std::array<mjtNum, 3>> gradient_opt = ReadDoubleArrayAttr(m, instance, "dB");
  if (gradient_opt) {
    config.dB[0] = gradient_opt.value()[0];
    config.dB[1] = gradient_opt.value()[1];
    config.dB[2] = gradient_opt.value()[2];
  }

  return config;
}

InducedMagnetConfig InducedMagnetConfig::FromModel(const mjModel* m, int instance) {
  InducedMagnetConfig config;

  // Relative permeability is a mandatory field
  std::optional<mjtNum> mu_r_opt = ReadDoubleAttr(m, instance, "mu_r");
  if (!mu_r_opt) {
    throw std::runtime_error("Missing relative permeability attribute (mu_r) in magnet plugin for induced magnet.");
  }
  config.mu_r = mu_r_opt.value();

  // Volume of the magnet is a mandatory field
  // TODO: This could potentially be obtained from the selected primitive or mesh at some point
  std::optional<mjtNum> V_opt = ReadDoubleAttr(m, instance, "V");
  if (!V_opt) {
    throw std::runtime_error("Missing volume attribute (V) in magnet plugin for induced magnet.");
  }
  config.V = V_opt.value();

  // Demagnetizing factors are an optional field (defaults to 1/3, isotropic)
  std::optional<std::array<mjtNum, 3>> N_opt = ReadDoubleArrayAttr(m, instance, "N");
  if (N_opt) {
    config.N[0] = N_opt.value()[0];
    config.N[1] = N_opt.value()[1];
    config.N[2] = N_opt.value()[2];
  }

  return config;
}

MagneticFieldConfig InducedMagnet::field_config_{};

std::optional<InducedMagnet> InducedMagnet::Create(const mjModel* m,  mjData* d, const int instance) {
  if (instance == 0) {
    // The first instance has to be always the magnetic field definition
    field_config_ = MagneticFieldConfig::FromModel(m, instance);
    return std::nullopt;
  }

  // The next instances only have the definition of each magnet
  const InducedMagnetConfig config = InducedMagnetConfig::FromModel(m, instance);

  // Store a map of the instance and each body ID to be able to match them later
  std::unordered_map<int, int> instance_to_body_ids;
  for (int i = 1; i < m->nbody; i++) {
    if (m->body_plugin[i] == instance) instance_to_body_ids[instance] = i;
  }

  return InducedMagnet(config, instance_to_body_ids, instance);
}

InducedMagnet::InducedMagnet(
  const InducedMagnetConfig& config, const std::unordered_map<int, int>& body_ids, int /*instance*/) :
  config_(config),
  instance_to_body_ids_(body_ids) {
}

void InducedMagnet::Compute(const mjModel* m, mjData* d, int instance) {
  const int body_id = instance_to_body_ids_[instance];

  // Get magnet pose
  const mjtNum* I_r_IF = d->xpos + 3*body_id;      // (x,y,z)
  const mjtNum* I_R_F = d->xmat + 9*body_id;       // 3x3 rotation, body->world

  // Get field parameters
  const mjtNum I_B0[3] = {
    field_config_.B0[0],
    field_config_.B0[1],
    field_config_.B0[2]
  };
  const mjtNum dB[9] = {
    field_config_.dB[0], 0, 0,
    0, field_config_.dB[1], 0,
    0, 0, field_config_.dB[2]
  };

  // Induced magnet parameters
  const mjtNum chi = config_.mu_r - 1;
  const mjtNum V = config_.V;
  const mjtNum B_Nx = config_.N[0];
  const mjtNum B_Ny = config_.N[1];
  const mjtNum B_Nz = config_.N[2];

  // Evaluate global magnetic field at magnet position in inertial frame (I_Bm)
  mjtNum aux[3];
  mju_mulMatVec3(aux, dB, I_r_IF);

  mjtNum I_Bm[3];
  mju_add3(I_Bm, I_B0, aux);

  // Compute the apparent chi of the induced magnet (Body frame)
  const mjtNum B_chi_app[9] = {
    chi / (1 + chi*B_Nx), 0, 0,
    0, chi / (1 + chi*B_Ny), 0,
    0, 0, chi / (1 + chi*B_Nz)
  };

  // Compute the body frame alpha
  mjtNum B_alpha[9];
  mju_scl(B_alpha, B_chi_app, (V/mu_0_), 9);

  // Rotate body frame alpha into the inertial frame (Tensor rotation -> R * alpha * R.T)
  mjtNum aux2[9];
  mju_mulMatMat(aux2, I_R_F, B_alpha, 3, 3, 3);

  mjtNum I_alpha[9];
  mju_mulMatMatT(I_alpha, aux2, I_R_F, 3, 3, 3);

  // Compute magnetic moment (m = alpha * B)
  mjtNum mag[3];
  mju_mulMatVec3(mag, I_alpha, I_Bm);

  // Compute force and torque in the inertial frame
  mjtNum F[3];
  mju_mulMatVec3(F, dB, mag);

  mjtNum tau[3];
  mju_cross(tau, mag, I_Bm);

  // Apply wrench to body
  mj_applyFT(m, d, F, tau, d->xpos+3*body_id, body_id, d->qfrc_passive);

  // Visual
  forces_[instance] = {F[0], F[1], F[2]};
  torques_[instance] = {tau[0], tau[1], tau[2]};
}

void InducedMagnet::Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance) {
  const int body_id = instance_to_body_ids_[instance];

  const mjtNum scaling_factor = 1e-2;  // 1 N is 1 cm in the visual
  const mjtNum* pos = d->xpos + 3*body_id;

  if (forces_.contains(instance)) {
    mjtNum F[3];
    F[0] = forces_[instance][0];
    F[1] = forces_[instance][1];
    F[2] = forces_[instance][2];

    mjtNum force_scaled[3];
    mju_scl3(force_scaled, F, scaling_factor);

    mjtNum force_projected[3];
    mju_add3(force_projected, pos, force_scaled);

    mjv_addLine(scn, pos, force_projected, "red");
  }

  if (torques_.contains(instance)) {
    mjtNum tau[3];
    tau[0] = torques_[instance][0];
    tau[1] = torques_[instance][1];
    tau[2] = torques_[instance][2];

    mjtNum torque_scaled[3];
    mju_scl3(torque_scaled, tau, scaling_factor);

    mjtNum torque_projected[3];
    mju_add3(torque_projected, pos, torque_scaled);

    mjv_addLine(scn, pos, torque_projected, "green");
  }
}

void InducedMagnet::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.magnet";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"B0", "dB", "mu_r", "V", "N"};
  plugin.nattribute = std::size(attributes);
  plugin.attributes = attributes;
  plugin.nstate = +[](const mjModel* m, int instance) {
    return 0;
  };
  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    std::optional<InducedMagnet> magnet = InducedMagnet::Create(m, d, instance);
    // The first instance is always the magnetic field definition, so we don't need to create an instance
    if (instance == 0) return 0;
    if (!magnet) return -1;
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(new InducedMagnet(std::move(*magnet)));
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    if (instance == 0) return;
    delete reinterpret_cast<InducedMagnet*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
    if (instance == 0) return;
    auto* magnet = reinterpret_cast<InducedMagnet*>(d->plugin_data[instance]);
    magnet->Compute(m, d, instance);
  };
  plugin.visualize =+[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
    if (instance == 0) return;
    auto* magnet = reinterpret_cast<InducedMagnet*>(d->plugin_data[instance]);
    magnet->Visualize(m, d, scn, instance);
  };
  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::passive
