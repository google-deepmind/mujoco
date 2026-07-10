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
#include <array>
#include <cstdlib>
#include <optional>
#include <utility>
#include <vector>
#include <variant>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <cmath>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::passive {

namespace {

const char* attributes[] = {"type", "B0", "dB", "mu_r", "V", "N", "center", "Bmax", "radius", "vis_scale"};
constexpr double MU_0 = 4.0 * M_PI * 1e-7;

struct GlobalMagneticField {
  enum class Type { kNone, kLinear, kSpherical };
  Type type = Type::kNone;

  mjtNum B0[3] = {0.0, 0.0, 0.0};
  mjtNum dB_diag[3] = {0.0, 0.0, 0.0};

  mjtNum center[3] = {0.0, 0.0, 0.0};
  mjtNum Bmax[3] = {0.0, 0.0, 0.0};
  mjtNum radius = 0.0;

  void getField(mjtNum res[3], const mjtNum pos[3]) const {
    if (type == Type::kLinear) {
      res[0] = B0[0] + dB_diag[0] * pos[0];
      res[1] = B0[1] + dB_diag[1] * pos[1];
      res[2] = B0[2] + dB_diag[2] * pos[2];
    } else if (type == Type::kSpherical) {
      mjtNum r[3];
      mju_sub3(r, pos, center);
      mjtNum dist_sq = r[0]*r[0] + r[1]*r[1] + r[2]*r[2];
      mjtNum radius_sq = radius * radius;

      if (dist_sq >= radius_sq) {
        mju_zero(res, 3);
        return;
      }

      mjtNum pct = dist_sq / radius_sq;
      mjtNum factor = (1.0 - pct) * (1.0 - pct);
      mju_scl3(res, Bmax, factor);
    } else {
      mju_zero(res, 3);
    }
  }

  void getGradient(mjtNum res[9], const mjtNum pos[3]) const {
    mju_zero(res, 9);
    if (type == Type::kLinear) {
      res[0] = dB_diag[0];
      res[4] = dB_diag[1];
      res[8] = dB_diag[2];
    } else if (type == Type::kSpherical) {
      mjtNum r[3];
      mju_sub3(r, pos, center);
      mjtNum dist_sq = r[0]*r[0] + r[1]*r[1] + r[2]*r[2];
      mjtNum radius_sq = radius * radius;

      if (dist_sq >= radius_sq) return;

      mjtNum pct = dist_sq / radius_sq;
      mjtNum factor = (1.0 - pct);
      mjtNum scale = -4.0 * factor / radius_sq;

      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          res[i * 3 + j] = scale * Bmax[i] * r[j];
        }
      }
    }
  }
};

GlobalMagneticField global_field_;

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
  } else {
    line_geom.rgba[0] = 1.0f; line_geom.rgba[1] = 1.0f; line_geom.rgba[2] = 1.0f;
  }
  line_geom.rgba[3] = 1.0f;

  constexpr mjtNum line_width_pixels = 20;
  mjv_connector(&line_geom, mjGEOM_LINE, line_width_pixels, from, to);
  scn->geoms[scn->ngeom++] = line_geom;
}

std::optional<mjtNum> ReadDoubleAttr(const mjModel* m, const int instance, const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') return std::nullopt;
  return std::strtod(value, nullptr);
}

std::optional<std::array<mjtNum, 3>> ReadDoubleArrayAttr(const mjModel* m, const int instance, const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') return std::nullopt;
  std::array<mjtNum, 3> vec;
  char* end;
  for (int i = 0; i < 3; i++) {
    vec[i] = std::strtod(value, &end);
    if (value == end) return std::nullopt;
    value = end;
  }
  return vec;
}

}  // namespace

struct InducedMagnetConfig {
  mjtNum mu_r = 1.0;
  mjtNum V = 0.0;
  mjtNum N[3] = {0.0, 0.0, 0.0};
  mjtNum vis_scale = 1e-2;

  static InducedMagnetConfig FromModel(const mjModel* m, int instance) {
    InducedMagnetConfig config;
    std::optional<mjtNum> mu_r_opt = ReadDoubleAttr(m, instance, "mu_r");
    if (!mu_r_opt) throw std::runtime_error("Missing mu_r attribute.");
    config.mu_r = *mu_r_opt;

    std::optional<mjtNum> V_opt = ReadDoubleAttr(m, instance, "V");
    if (!V_opt) throw std::runtime_error("Missing V attribute.");
    config.V = *V_opt;

    std::optional<std::array<mjtNum, 3>> N_opt = ReadDoubleArrayAttr(m, instance, "N");
    if (N_opt) std::copy(N_opt->begin(), N_opt->end(), config.N);

    std::optional<mjtNum> scale_opt = ReadDoubleAttr(m, instance, "vis_scale");
    if (scale_opt) config.vis_scale = *scale_opt;

    return config;
  }
};

class MagneticPlugin {
 public:
  struct Field {};

  struct InducedMagnet {
    InducedMagnetConfig config;
    int body_id;

    void ComputeCore(const mjModel* m, mjData* d, mjtNum F_out[3], mjtNum tau_out[3]) const {
      const mjtNum* I_r_IF = d->xpos + 3 * body_id;
      const mjtNum* I_R_F = d->xmat + 9 * body_id;

      mjtNum I_Bm[3];
      global_field_.getField(I_Bm, I_r_IF);

      mjtNum dB[9];
      global_field_.getGradient(dB, I_r_IF);

      const mjtNum chi = config.mu_r - 1.0;
      const mjtNum V = config.V;
      const mjtNum B_chi_app[9] = {
        chi / (1 + chi * config.N[0]), 0, 0,
        0, chi / (1 + chi * config.N[1]), 0,
        0, 0, chi / (1 + chi * config.N[2])
      };

      mjtNum B_alpha[9];
      mju_scl(B_alpha, B_chi_app, (V / MU_0), 9);

      mjtNum aux2[9];
      mju_mulMatMat(aux2, I_R_F, B_alpha, 3, 3, 3);

      mjtNum I_alpha[9];
      mju_mulMatMatT(I_alpha, aux2, I_R_F, 3, 3, 3);

      mjtNum mag[3];
      mju_mulMatVec3(mag, I_alpha, I_Bm);

      mju_mulMatVec3(F_out, dB, mag);
      mju_cross(tau_out, mag, I_Bm);
    }

    void Compute(const mjModel* m, mjData* d) const {
      mjtNum F[3], tau[3];
      ComputeCore(m, d, F, tau);
      mj_applyFT(m, d, F, tau, d->xpos + 3 * body_id, body_id, d->qfrc_passive);
    }

    void Visualize(const mjModel* m, mjData* d, mjvScene* scn) const {
      mjtNum F[3], tau[3];
      ComputeCore(m, d, F, tau);
      const mjtNum* pos = d->xpos + 3 * body_id;

      mjtNum force_scaled[3], force_projected[3];
      mju_scl3(force_scaled, F, config.vis_scale);
      mju_add3(force_projected, pos, force_scaled);
      mjv_addLine(scn, pos, force_projected, "red");

      mjtNum torque_scaled[3], torque_projected[3];
      mju_scl3(torque_scaled, tau, config.vis_scale);
      mju_add3(torque_projected, pos, torque_scaled);
      mjv_addLine(scn, pos, torque_projected, "green");
    }
  };

  struct Magnetometer {
    int sensor_id;
    int site_id;

    void ComputeSensor(const mjModel* m, mjData* d) const {
      const mjtNum* I_r_IS = d->site_xpos + 3 * site_id;
      const mjtNum* I_R_S  = d->site_xmat + 9 * site_id;

      mjtNum I_Bm[3];
      global_field_.getField(I_Bm, I_r_IS);

      mjtNum S_Bm[3];
      mju_mulMatTVec3(S_Bm, I_R_S, I_Bm);

      int sensor_data_offset = m->sensor_adr[sensor_id];
      d->sensordata[sensor_data_offset + 0] = S_Bm[0];
      d->sensordata[sensor_data_offset + 1] = S_Bm[1];
      d->sensordata[sensor_data_offset + 2] = S_Bm[2];
    }
  };

  explicit MagneticPlugin(std::variant<Field, InducedMagnet, Magnetometer> impl)
      : impl_(std::move(impl)) {}

  static std::optional<MagneticPlugin> Create(const mjModel* m, mjData* d, const int instance);

  void Compute(const mjModel* m, mjData* d);
  void ComputeSensor(const mjModel* m, mjData* d);
  void Visualize(const mjModel* m, mjData* d, mjvScene* scn);

  // Declared static method inside the class namespace
  static void RegisterPlugin();

 private:
  std::variant<Field, InducedMagnet, Magnetometer> impl_;
};

std::optional<MagneticPlugin> MagneticPlugin::Create(const mjModel* m, mjData* d, const int instance) {
  const char* type_str = mj_getPluginConfig(m, instance, "type");
  std::string type = type_str ? type_str : "";

  if (type == "linear_field") {
    global_field_.type = GlobalMagneticField::Type::kLinear;

    auto b0_opt = ReadDoubleArrayAttr(m, instance, "B0");
    if (b0_opt) std::copy(b0_opt->begin(), b0_opt->end(), global_field_.B0);

    auto db_opt = ReadDoubleArrayAttr(m, instance, "dB");
    if (db_opt) std::copy(db_opt->begin(), db_opt->end(), global_field_.dB_diag);

    return MagneticPlugin(Field{});
  }

  if (type == "spherical_field") {
    global_field_.type = GlobalMagneticField::Type::kSpherical;

    auto center_opt = ReadDoubleArrayAttr(m, instance, "center");
    if (center_opt) std::copy(center_opt->begin(), center_opt->end(), global_field_.center);

    auto bmax_opt = ReadDoubleArrayAttr(m, instance, "Bmax");
    if (bmax_opt) std::copy(bmax_opt->begin(), bmax_opt->end(), global_field_.Bmax);

    auto radius_opt = ReadDoubleAttr(m, instance, "radius");
    if (radius_opt) global_field_.radius = *radius_opt;

    return MagneticPlugin(Field{});
  }

  if (type == "magnetometer") {
    int sensor_id = -1;
    for (int i = 0; i < m->nsensor; i++) {
      if (m->sensor_plugin[i] == instance) {
        sensor_id = i;
        break;
      }
    }
    if (sensor_id == -1 || m->sensor_objtype[sensor_id] != mjOBJ_SITE) {
      throw std::runtime_error("Magnetometer must be attached to a site.");
    }
    return MagneticPlugin(Magnetometer{sensor_id, m->sensor_objid[sensor_id]});
  }

  if (type == "induced_magnet") {
    int body_id = -1;
    for (int i = 1; i < m->nbody; i++) {
      if (m->body_plugin[i] == instance) {
        body_id = i;
        break;
      }
    }
    if (body_id == -1) throw std::runtime_error("Induced magnet must be attached to a valid body.");

    return MagneticPlugin(InducedMagnet{InducedMagnetConfig::FromModel(m, instance), body_id});
  }

  return std::nullopt;
}

void MagneticPlugin::Compute(const mjModel* m, mjData* d) {
  if (auto* magnet = std::get_if<InducedMagnet>(&impl_)) magnet->Compute(m, d);
}

void MagneticPlugin::ComputeSensor(const mjModel* m, mjData* d) {
  if (auto* sensor = std::get_if<Magnetometer>(&impl_)) sensor->ComputeSensor(m, d);
}

void MagneticPlugin::Visualize(const mjModel* m, mjData* d, mjvScene* scn) {
  if (auto* magnet = std::get_if<InducedMagnet>(&impl_)) magnet->Visualize(m, d, scn);
}

// Fixed: Correctly scoped to the MagneticPlugin class
void MagneticPlugin::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.magnet";
  plugin.capabilityflags |= (mjPLUGIN_PASSIVE | mjPLUGIN_SENSOR);
  plugin.nattribute = std::size(attributes);
  plugin.attributes = attributes;
  plugin.nsensordata = +[](const mjModel* m, int instance, int sensor_id) { return 3; };
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

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
    if (capability_bit & mjPLUGIN_PASSIVE) plugin_ptr->Compute(m, d);
    if (capability_bit & mjPLUGIN_SENSOR) plugin_ptr->ComputeSensor(m, d);
  };

  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
    auto* plugin_ptr = reinterpret_cast<MagneticPlugin*>(d->plugin_data[instance]);
    if (plugin_ptr) plugin_ptr->Visualize(m, d, scn);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::passive