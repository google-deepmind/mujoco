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
#include <optional>
#include <utility>
#include <vector>
#include <fstream>
#include <unordered_map>
#include <variant>
#include <string>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

#include "magnet.h" // Ensure this matches your header file name

namespace mujoco::plugin::passive {

namespace {

// "type" must be included here so MuJoCo's XML parser recognizes it as a valid attribute
const char* attributes[] = {"type", "B0", "dB", "mu_r", "V", "N"};

void mjv_addLine(mjvScene* scn, const mjtNum from[3], const mjtNum to[3], const std::string& color) {
  mjvGeom line_geom;
  mjv_initGeom(&line_geom, mjGEOM_LINE, nullptr, nullptr, nullptr, nullptr);

  line_geom.type = mjGEOM_LINE;

  if (color == "red") {
    line_geom.rgba[0] = 1.0f; line_geom.rgba[1] = 0.0f; line_geom.rgba[2] = 0.0f;
  } else if (color == "green") {
    line_geom.rgba[0] = 0.0f; line_geom.rgba[1] = 1.0f; line_geom.rgba[2] = 0.0f;
  } else if (color == "blue") {
    line_geom.rgba[0] = 0.0f; line_geom.rgba[1] = 0.0f; line_geom.rgba[2] = 1.0f;
  } else if (color == "yellow") {
    line_geom.rgba[0] = 1.0f; line_geom.rgba[1] = 1.0f; line_geom.rgba[2] = 0.0f;
  } else if (color == "purple") {
    line_geom.rgba[0] = 0.5f; line_geom.rgba[1] = 0.0f; line_geom.rgba[2] = 0.5f;
  } else if (color == "cyan") {
    line_geom.rgba[0] = 0.0f; line_geom.rgba[1] = 1.0f; line_geom.rgba[2] = 1.0f;
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
      return std::nullopt;
    }
    value = end;
  }
  return vec;
}

}  // namespace

// ---------------------------------------------------------------------------
// Configuration Structs
// ---------------------------------------------------------------------------

MagneticFieldConfig MagneticFieldConfig::FromModel(const mjModel* m, int instance) {
  MagneticFieldConfig config;

  std::optional<std::array<mjtNum, 3>> field_opt = ReadDoubleArrayAttr(m, instance, "B0");
  if (field_opt) {
    config.B0[0] = field_opt.value()[0];
    config.B0[1] = field_opt.value()[1];
    config.B0[2] = field_opt.value()[2];
  }

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

  std::optional<mjtNum> mu_r_opt = ReadDoubleAttr(m, instance, "mu_r");
  if (!mu_r_opt) {
    throw std::runtime_error("Missing relative permeability attribute (mu_r) in magnet plugin.");
  }
  config.mu_r = mu_r_opt.value();

  std::optional<mjtNum> V_opt = ReadDoubleAttr(m, instance, "V");
  if (!V_opt) {
    throw std::runtime_error("Missing volume attribute (V) in magnet plugin.");
  }
  config.V = V_opt.value();

  std::optional<std::array<mjtNum, 3>> N_opt = ReadDoubleArrayAttr(m, instance, "N");
  if (N_opt) {
    config.N[0] = N_opt.value()[0];
    config.N[1] = N_opt.value()[1];
    config.N[2] = N_opt.value()[2];
  }

  return config;
}

MagneticFieldConfig MagneticPlugin::field_config_{};

// ---------------------------------------------------------------------------
// Sub-Class Constructors
// ---------------------------------------------------------------------------

MagneticPlugin::Field::Field(const MagneticFieldConfig& config) {}

MagneticPlugin::InducedMagnet::InducedMagnet(
  const InducedMagnetConfig& config, const std::unordered_map<int, int>& body_ids, int instance) :
  config_(config),
  instance_to_body_ids_(body_ids) {
  // Optional logger path configuration
  // file_ = std::ofstream("/path/to/log/magnet_log_" + std::to_string(instance) + ".csv");
  // file_ << "time,body_id,fx,fy,fz,tx,ty,tz" << std::endl;
}

MagneticPlugin::Magnetometer::Magnetometer(int instance) {}

// ---------------------------------------------------------------------------
// Main Plugin Class Factory Method
// ---------------------------------------------------------------------------

std::optional<MagneticPlugin> MagneticPlugin::Create(const mjModel* m, mjData* d, const int instance) {
  const char* type_str = mj_getPluginConfig(m, instance, "type");
  std::string type = type_str ? type_str : "";

  // 1. Core Field Definition Routing
  if (type == "linear_field") {
    field_config_ = MagneticFieldConfig::FromModel(m, instance);

    d->B0_magnetic[0] = field_config_.B0[0];
    d->B0_magnetic[1] = field_config_.B0[1];
    d->B0_magnetic[2] = field_config_.B0[2];
    d->dB_magnetic[0] = field_config_.dB[0];
    d->dB_magnetic[1] = field_config_.dB[1];
    d->dB_magnetic[2] = field_config_.dB[2];

    return MagneticPlugin(Field(field_config_));
  }

  // 2. Sensor Routing
  if (type == "magnetometer") {
    return MagneticPlugin(Magnetometer(instance));
  }

  // 3. Actuator/Passive Body Object Routing
  if (type == "induced_magnet") {
    const InducedMagnetConfig config = InducedMagnetConfig::FromModel(m, instance);
    std::unordered_map<int, int> instance_to_body_ids;
    for (int i = 1; i < m->nbody; i++) {
      if (m->body_plugin[i] == instance) {
        instance_to_body_ids[instance] = i;
      }
    }
    return MagneticPlugin(InducedMagnet(config, instance_to_body_ids, instance));
  }

  return std::nullopt;
}

// ---------------------------------------------------------------------------
// Sub-Class Functional Logic
// ---------------------------------------------------------------------------

void MagneticPlugin::InducedMagnet::Compute(const mjModel* m, mjData* d, int instance) {
  const int body_id = instance_to_body_ids_[instance];

  const mjtNum* I_r_IF = d->xpos + 3 * body_id;
  const mjtNum* I_R_F = d->xmat + 9 * body_id;

  const mjtNum I_B0[3] = { d->B0_magnetic[0], d->B0_magnetic[1], d->B0_magnetic[2] };
  const mjtNum dB[9] = {
    d->dB_magnetic[0], 0, 0,
    0, d->dB_magnetic[1], 0,
    0, 0, d->dB_magnetic[2]
  };

  const mjtNum chi = config_.mu_r - 1;
  const mjtNum V = config_.V;
  const mjtNum B_Nx = config_.N[0];
  const mjtNum B_Ny = config_.N[1];
  const mjtNum B_Nz = config_.N[2];

  mjtNum aux[3];
  mju_mulMatVec3(aux, dB, I_r_IF);

  mjtNum I_Bm[3];
  mju_add3(I_Bm, I_B0, aux);

  const mjtNum B_chi_app[9] = {
    chi / (1 + chi*B_Nx), 0, 0,
    0, chi / (1 + chi*B_Ny), 0,
    0, 0, chi / (1 + chi*B_Nz)
  };

  mjtNum B_alpha[9];
  mju_scl(B_alpha, B_chi_app, (V / MagneticPlugin::mu_0_), 9);

  mjtNum aux2[9];
  mju_mulMatMat(aux2, I_R_F, B_alpha, 3, 3, 3);

  mjtNum I_alpha[9];
  mju_mulMatMatT(I_alpha, aux2, I_R_F, 3, 3, 3);

  mjtNum mag[3];
  mju_mulMatVec3(mag, I_alpha, I_Bm);

  mjtNum F[3];
  mju_mulMatVec3(F, dB, mag);

  mjtNum tau[3];
  mju_cross(tau, mag, I_Bm);

  mj_applyFT(m, d, F, tau, d->xpos + 3 * body_id, body_id, d->qfrc_passive);

  forces_[instance] = {F[0], F[1], F[2]};
  torques_[instance] = {tau[0], tau[1], tau[2]};
}

void MagneticPlugin::Magnetometer::ComputeSensor(const mjModel* m, mjData* d, int instance) {
  int sensor_id = -1;
  for (int i = 0; i < m->nsensor; i++) {
    if (m->sensor_plugin[i] == instance) {
      sensor_id = i;
      break;
    }
  }

  if (sensor_id == -1 || m->sensor_objtype[sensor_id] != mjOBJ_SITE) return;
  const int site_id = m->sensor_objid[sensor_id];

  const mjtNum* I_r_IS = d->site_xpos + 3 * site_id;
  const mjtNum* I_R_S  = d->site_xmat + 9 * site_id;

  const mjtNum I_B0[3] = { d->B0_magnetic[0], d->B0_magnetic[1], d->B0_magnetic[2] };
  const mjtNum dB[9] = {
    d->dB_magnetic[0], 0, 0,
    0, d->dB_magnetic[1], 0,
    0, 0, d->dB_magnetic[2]
  };

  mjtNum aux[3];
  mju_mulMatVec3(aux, dB, I_r_IS);

  mjtNum I_Bm[3];
  mju_add3(I_Bm, I_B0, aux);

  mjtNum S_Bm[3];
  mju_mulMatTVec3(S_Bm, I_R_S, I_Bm);

  std::cout << "S_Bm: " << S_Bm[0] << ", " << S_Bm[1] << ", " << S_Bm[2] << std::endl;

  // Directly targets the active sensor array address index instead of the global instance
  int sensor_data_offset = m->sensor_adr[sensor_id];
  d->sensordata[sensor_data_offset + 0] = S_Bm[0];
  d->sensordata[sensor_data_offset + 1] = S_Bm[1];
  d->sensordata[sensor_data_offset + 2] = S_Bm[2];
}

void MagneticPlugin::InducedMagnet::Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance) {
  const int body_id = instance_to_body_ids_[instance];
  constexpr mjtNum scaling_factor = 1e-2;
  const mjtNum* pos = d->xpos + 3 * body_id;

  if (forces_.contains(instance)) {
    mjtNum F[3] = {forces_[instance][0], forces_[instance][1], forces_[instance][2]};
    mjtNum force_scaled[3];
    mju_scl3(force_scaled, F, scaling_factor);

    mjtNum force_projected[3];
    mju_add3(force_projected, pos, force_scaled);
    mjv_addLine(scn, pos, force_projected, "red");
  }

  if (torques_.contains(instance)) {
    mjtNum tau[3] = {torques_[instance][0], torques_[instance][1], torques_[instance][2]};
    mjtNum torque_scaled[3];
    mju_scl3(torque_scaled, tau, scaling_factor);

    mjtNum torque_projected[3];
    mju_add3(torque_projected, pos, torque_scaled);
    mjv_addLine(scn, pos, torque_projected, "green");
  }
}

// ---------------------------------------------------------------------------
// Variant Interface Dispatchers
// ---------------------------------------------------------------------------

void MagneticPlugin::Compute(const mjModel* m, mjData* d, int instance) {
  if (auto* magnet = std::get_if<InducedMagnet>(&impl_)) {
    magnet->Compute(m, d, instance);
  }
}

void MagneticPlugin::ComputeSensor(const mjModel* m, mjData* d, int instance) {
  if (auto* sensor = std::get_if<Magnetometer>(&impl_)) {
    sensor->ComputeSensor(m, d, instance);
  }
}

void MagneticPlugin::Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance) {
  if (auto* magnet = std::get_if<InducedMagnet>(&impl_)) {
    magnet->Visualize(m, d, scn, instance);
  }
}

// ---------------------------------------------------------------------------
// Registration Block
// ---------------------------------------------------------------------------

void MagneticPlugin::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.magnet";
  plugin.capabilityflags |= (mjPLUGIN_PASSIVE | mjPLUGIN_SENSOR);
  plugin.nattribute = std::size(attributes);
  plugin.attributes = attributes;
  plugin.nsensordata = [](const mjModel* m, int instance, int sensor_id) {
    return 3;  // Outputs Bx, By, Bz in sensor frame
  };
  plugin.nstate = +[](const mjModel* m, int instance) {
    return 0;
  };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    std::optional<MagneticPlugin> magnet = MagneticPlugin::Create(m, d, instance);
    if (!magnet) return -1;
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(new MagneticPlugin(std::move(*magnet)));
    return 0;
  };

  plugin.destroy = +[](mjData* d, int instance) {
    auto* plugin_ptr = reinterpret_cast<MagneticPlugin*>(d->plugin_data[instance]);
    if (plugin_ptr) {
      delete plugin_ptr;
      d->plugin_data[instance] = 0;
    }
  };

  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
    auto* plugin_ptr = reinterpret_cast<MagneticPlugin*>(d->plugin_data[instance]);
    if (!plugin_ptr) return;

    if (capability_bit & mjPLUGIN_PASSIVE) {
      plugin_ptr->Compute(m, d, instance);
    }
    if (capability_bit & mjPLUGIN_SENSOR) {
      plugin_ptr->ComputeSensor(m, d, instance);
    }
  };

  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
    auto* plugin_ptr = reinterpret_cast<MagneticPlugin*>(d->plugin_data[instance]);
    if (plugin_ptr) {
      plugin_ptr->Visualize(m, d, scn, instance);
    }
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::passive