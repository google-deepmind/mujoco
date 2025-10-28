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

#ifndef MUJOCO_PLUGIN_PASSIVE_MAGNET_H_
#define MUJOCO_PLUGIN_PASSIVE_MAGNET_H_

#include <memory>
#include <optional>
#include <array>
#include <map>
#include <string>
#include <fstream>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::passive {

struct MagnetConfig {
  // Inertial frame magnetic field
  mjtNum B[3] = {0.0, 0.0, 0.0};

  // Inertial frame magnetic field gradient
  mjtNum dB[3] = {0.0, 0.0, 0.0};

  // Type of magnetic interaction
  std::string type = "";

  // Body frame magnetic moment
  mjtNum m[3] = {0.0, 0.0, 0.0};

  // Body frame demagnetizing factors
  mjtNum N[3] = {0.0, 0.0, 0.0};

  // Relative permeability of the magnetic material
  double mu_r = 0.0;

  // Volume of the magnet
  double V = 0.0;

  // Reads plugin attributes to construct PID configuration.
  static MagnetConfig FromModel(const mjModel* m, int instance);
};

class Magnet {
 public:
  ~Magnet() {
    log_.close();
  }
  Magnet(Magnet&&) = default;

  static std::optional<Magnet> Create(const mjModel* m, mjData* d, int instance);

  void Compute(const mjModel* m, mjData* d, int instance);
  void Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance);

  static void RegisterPlugin();

 protected:
  Magnet(MagnetConfig config, std::map<int, int> body_ids, int instance);

 private:
  MagnetConfig config_;
  std::map<int, int> body_ids_;
  std::map<int, int> permanent_magnet_ids_;

  std::map<int, std::array<mjtNum, 3>> forces_;
  std::map<int, std::array<mjtNum, 3>> torques_;

  std::ofstream log_;

  const mjtNum mu_0_ = 4 * mjPI * 1e-7;
};

}  // namespace mujoco::plugin::passive

#endif  // MUJOCO_PLUGIN_PASSIVE_MAGNET_H_
