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

#include "magnet.h"

#include <cstdint>
#include <iostream>
#include <cstdlib>
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

namespace mujoco::plugin::passive {

namespace {

std::vector<std::array<double, 3>> parse_matrix(std::string path) {
  std::ifstream file;

  file.open(path);
  if (!file.is_open()) {
    throw std::runtime_error("Error opening file " + path);
  }

  double x = 0;
  double y = 0;
  double z = 0;

  double B_x = 0;
  double B_y = 0;
  double B_z = 0;

  double dx = 0.05;
  double dy = 0.05;
  double dz = 1.90986;

  double ux = 15;
  double uy = 25;
  double uz = 177.617;

  double lx = -15;
  double ly = -25;
  double lz = 0.0;

  double nx = (ux - lx) / dx;
  double ny = (uy - ly) / dy;
  double nz = (uz - lz) / dz;

  std::vector<std::array<double, 3>> data =
    std::vector<std::array<double, 3>>(
      nx * ny * nz, {0.0, 0.0, 0.0});

  int i = 0;
  while (file >> x >> y >> z >> B_x >> B_y >> B_z) {
    //*** std::cout  << "Field at (" << x << ", " << y << ", " << z << "): (" << B_x << ", " << B_y << ", " << B_z << ")" << std::endl;
    data[i] = {B_x, B_y, B_z};
    i++;
  }

  //*** std::cout  << "Loaded " << i << " field points from file." << std::endl;
  file.close();

  return data;
}

void mjv_addLine(mjvScene* scn, const mjtNum from[3], const mjtNum to[3], std::string color) {
  mjvGeom line_geom;
  mjv_initGeom(&line_geom, mjGEOM_LINE, NULL, NULL, NULL, NULL);

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

  const mjtNum line_width_pixels = 20;
  mjv_connector(&line_geom, mjGEOM_LINE, line_width_pixels, from, to);
  scn->geoms[scn->ngeom++] = line_geom;
}

std::optional<mjtNum> ReadDoubleAttr(const mjModel* m, int instance, const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;
  }
  return std::strtod(value, nullptr);
}

std::optional<std::array<mjtNum, 3>> ReadDoubleArrayAttr(const mjModel* m, int instance, const char* attr) {
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

std::optional<std::string> ReadStringAttr(const mjModel* m, int instance, const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;
  }
  return std::string(value);
}

}  // namespace

// Configs

MagnetConfig MagnetConfig::FromModel(const mjModel* m, int instance) {
  //*** std::cout  << "Reading magnet plugin config for instance: " << instance << std::endl;
  MagnetConfig config;

  // Type of magnet is mandatory
  std::optional<std::string> type_opt = ReadStringAttr(m, instance, "type");
  if (!type_opt) {
    throw std::runtime_error("Missing type attribute in magnet plugin. Possible values are 'permanent' or 'induced'.");
  }
  config.type = type_opt.value();

  if (config.type == "permanent") {
    // If permanent magnet the magnetic moment is mandatory
    std::optional<std::array<mjtNum, 3>> m_opt = ReadDoubleArrayAttr(m, instance, "m");
    if (!m_opt) {
      throw std::runtime_error("Missing magnetic moment attribute (m) in magnet plugin for permanent magnet.");
    }
    config.m[0] = m_opt.value()[0];
    config.m[1] = m_opt.value()[1];
    config.m[2] = m_opt.value()[2];
  } else if (config.type == "induced") {
    // If induced magnet the relative permeability and the volume are mandatory
    std::optional<mjtNum> mu_r_opt = ReadDoubleAttr(m, instance, "mu_r");
    if (!mu_r_opt) {
      throw std::runtime_error("Missing relative permeability attribute (mu_r) in magnet plugin for induced magnet.");
    }
    config.mu_r = mu_r_opt.value();

    std::optional<mjtNum> V_opt = ReadDoubleAttr(m, instance, "V");
    if (!V_opt) {
      throw std::runtime_error("Missing volume attribute (V) in magnet plugin for induced magnet.");
    }
    config.V = V_opt.value();

    // Demagnetizing factors are optional, default to 1/3
    std::array<mjtNum, 3> N = ReadDoubleArrayAttr(m, instance, "N").value_or(std::array<mjtNum, 3>({1/3, 1/3, 1/3}));
    config.N[0] = N[0];
    config.N[1] = N[1];
    config.N[2] = N[2];
  }

  // Field and gradient are optional, default to zero
  std::array<mjtNum, 3> B = ReadDoubleArrayAttr(m, instance, "B").value_or(std::array<mjtNum, 3>({0.0, 0.0, 0.0}));
  config.B[0] = B[0];
  config.B[1] = B[1];
  config.B[2] = B[2];

  std::array<mjtNum, 3> dB = ReadDoubleArrayAttr(m, instance, "dB").value_or(std::array<mjtNum, 3>({0.0, 0.0, 0.0}));
  config.dB[0] = dB[0];
  config.dB[1] = dB[1];
  config.dB[2] = dB[2];

  // You can also provide a path to a file with the field
  std::optional<std::string> field_path_opt = ReadStringAttr(m, instance, "field_path");
  if (field_path_opt) {
    std::string field_path = field_path_opt.value();
    //*** std::cout  << "Loading magnetic field from file: " << field_path << std::endl;
    std::vector<std::array<double, 3>> field = parse_matrix(field_path);
  }

  return config;
}

// Magnet class

std::optional<Magnet> Magnet::Create(const mjModel* m,  mjData* d, int instance) {
  MagnetConfig config = MagnetConfig::FromModel(m, instance);

  std::map<int, int> body_ids;
  for (int i = 1; i < m->nbody; i++) {
    if (m->body_plugin[i] == instance) {
      //*** std::cout  << "Found body " << i << " for plugin instance " << instance << std::endl;
      body_ids[instance] = i;
    }
  }

  return Magnet(config, body_ids, instance);
}

// plugin constructor
Magnet::Magnet(MagnetConfig config, std::map<int, int> body_ids, int instance) :
  config_(std::move(config)),
  body_ids_(std::move(body_ids)) {
    log_.open("/home/jplayang/Projects/Python/quadruped_nmpc/data/magnet_log_" + std::to_string(instance) + ".csv");
    log_ << "time,body_id,fx,fy,fz,tx,ty,tz" << std::endl;
  }

void Magnet::Compute(const mjModel* m, mjData* d, int instance) {
  // //*** std::cout  << "Computing magnet plugin for instance: " << instance << std::endl;

  int i = body_ids_[instance];

  // Get magnet pose
  // const mjtNum* I_r_IF = d->xpos + 3*i;         // (x,y,z)
  const mjtNum* I_R_F = d->xmat + 9*i;       // 3x3 rotation, body->world

  // std::cout << "Mujoco I_R_F:" << std::endl;
  // std::cout <<  I_R_F[0] << ", " << I_R_F[1] << ", " << I_R_F[2] << std::endl;
  // std::cout <<  I_R_F[3] << ", " << I_R_F[4] << ", " << I_R_F[5] << std::endl;
  // std::cout <<  I_R_F[6] << ", " << I_R_F[7] << ", " << I_R_F[8] << std::endl;

  // Parameters
  const mjtNum chi = config_.mu_r - 1;
  const mjtNum V = config_.V;
  const mjtNum N_x = config_.N[0];
  const mjtNum N_y = config_.N[1];
  const mjtNum N_z = config_.N[2];

  const mjtNum I_B_F[3] = {
    config_.B[0],
    config_.B[1],
    config_.B[2]
  };
  const mjtNum dB[9] = {
    config_.dB[0], 0, 0,
    0, config_.dB[1], 0,
    0, 0, config_.dB[2]
  };

  //*** std::cout  << "dB: (" << dB[0] << ", " << dB[4] << ", " << dB[8] << ")" << std::endl;


  // Evaluate global magnetic field at pos in inertial frame

  // mjtNum aux[3];
  // mju_mulMatVec3(aux, dB, I_r_IF);

  // mjtNum I_B_F[3];
  // mju_add3(I_B_F, I_B0, aux);

  // Compute field generated by permamanent magnets in pos and add to I_B_F

  // for (const auto& [perm_instance, perm_body_id] : permanent_magnet_ids_) {
  //   const mjtNum* I_r_IB = d->xpos + 3*perm_body_id;         // (x,y,z)
  //   const mjtNum* I_R_B = d->xmat + 9*perm_body_id;       // 3x3 rotation, body->world

  //   const mjtNum* B_m = config_.m;

  //   // Get magnetic moment in inertial frame
  //   mjtNum I_m[3];
  //   mju_mulMatVec3(I_m, I_R_B, B_m);

  //   // Vector from source to field point
  //   mjtNum r[3];
  //   mju_sub3(r, I_r_IF, I_r_IB);
  //   mjtNum dist = mju_norm3(r);
  //   if (dist < 1e-6) {
  //     // Magnets in contact, skip field computation
  //     continue;
  //   }
  //   mjtNum r_hat[3];
  //   mju_scl3(r_hat, r, 1/dist);

  //   // Magnetic field from dipole
  //   mjtNum coeff = mu_0_/(4*mjPI*dist*dist*dist);
  //   mjtNum dot = mju_dot3(I_m, r_hat);

  //   mjtNum B_dipole[3];
  //   mju_scl3(B_dipole, r_hat, 3*dot);
  //   mju_subFrom3(B_dipole, I_m);
  //   mju_scl3(B_dipole, B_dipole, coeff);

  //   // Add to total field
  //   mju_add3(I_B_F, I_B_F, B_dipole);
  // }


  // If we are in a permanent magnet take the magnetic moment directly


  // If we are in an induced magnet compute the magnetic moment with the total field

  mjtNum F_chi_app[9] = {
    chi / (1 + chi*N_x), 0, 0,
    0, chi / (1 + chi*N_y), 0,
    0, 0, chi / (1 + chi*N_z)
  };

  mjtNum F_alpha[9];
  mju_scl(F_alpha, F_chi_app, V/mu_0_, 9);

  // std::cout  << "F_alpha: [" << F_alpha[0] << ", " << F_alpha[4] << ", " << F_alpha[8] << "]" << std::endl;

  mjtNum aux2[9];
  mju_mulMatMat(aux2, I_R_F, F_alpha, 3, 3, 3);

  mjtNum I_alpha[9];
  mju_mulMatMatT(I_alpha, aux2, I_R_F, 3, 3, 3);

  // std::cout << "Mujoco I_alpha:" << std::endl;
  // std::cout <<  I_alpha[0] << ", " << I_alpha[1] << ", " << I_alpha[2] << std::endl;
  // std::cout <<  I_alpha[3] << ", " << I_alpha[4] << ", " << I_alpha[5] << std::endl;
  // std::cout <<  I_alpha[6] << ", " << I_alpha[7] << ", " << I_alpha[8] << std::endl;

  mjtNum mag[3];
  mju_mulMatVec3(mag, I_alpha, I_B_F);

  //*** std::cout  << "Magnetic moment: (" << mag[0] << ", " << mag[1] << ", " << mag[2] << ")" << std::endl;


  // Compute force and torque in inertial frame and apply to body

  mjtNum F[3];
  mju_mulMatVec3(F, dB, mag);

  mjtNum tau[3];
  mju_cross(tau, mag, I_B_F);

  mj_applyFT(m, d, F, tau, d->xpos+3*i, i, d->qfrc_passive);

  // Visual
  forces_[instance] = {F[0], F[1], F[2]};
  torques_[instance] = {tau[0], tau[1], tau[2]};

  log_ << d->time << "," << i << "," << F[0] << "," << F[1] << "," << F[2] << "," << tau[0] << "," << tau[1] << "," << tau[2] << std::endl;

  std::cout  << "Magnet plugin applied force: (" << F[0] << ", " << F[1] << ", " << F[2] << ") and torque: (" << tau[0] << ", " << tau[1] << ", " << tau[2] << ")" << std::endl;
}

void Magnet::Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance) {
  int i = body_ids_[instance];

  const mjtNum scaling_factor = 1e-2;  // 1 N is 1 cm in the visual
  const mjtNum* pos = d->xpos + 3*i;

  if (forces_.find(instance) != forces_.end()) {
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

  if (torques_.find(instance) != torques_.end()) {
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

void Magnet::RegisterPlugin() {
  //*** std::cout  << "Registering magnet plugin" << std::endl;
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.magnet";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"B", "dB", "type", "m", "mu_r", "V", "N", "field_path"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate =
  +[](const mjModel* m, int instance) {
    return 0;
  };
  plugin.init =
  +[](const mjModel* m, mjData* d, int instance) {
    auto magnet = Magnet::Create(m, d, instance);
    if (!magnet) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(new Magnet(std::move(*magnet)));
    return 0;
  };
  plugin.destroy =
  +[](mjData* d, int instance) {
    delete reinterpret_cast<Magnet*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.compute =
  +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
    auto* magnet = reinterpret_cast<Magnet*>(d->plugin_data[instance]);
    magnet->Compute(m, d, instance);
  };
  plugin.visualize =
  +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
    auto* magnet = reinterpret_cast<Magnet*>(d->plugin_data[instance]);
    magnet->Visualize(m, d, scn, instance);
  };
  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::passive
