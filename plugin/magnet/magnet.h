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

#include <optional>
#include <array>
#include <unordered_map>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::passive {

/**
 * @brief Class to store the configuration of the magnetic field. To define it, we provide the magnetic filed at the
 * origin (B0) and a constant gradient (dB).
 */
struct MagneticFieldConfig {
  /**
   * @brief Magnetic field at the origin of the inertial frame
   */
  mjtNum B0[3] = {0.0, 0.0, 0.0};

  /**
   * @brief Gradient of the magnetic field at the origin of the inertial frame
   */
  mjtNum dB[3] = {0.0, 0.0, 0.0};

  /**
   * @brief Reads the plugin's attributes to build the magnetic field
   * @param m MuJoCo model to read from
   * @param instance Instance of the plugin
   * @return An instance of the MagneticFieldConfig class with the fields read from the model
   */
  static MagneticFieldConfig FromModel(const mjModel* m, int instance);
};

/**
 * @brief Class to store the configuration of an induced magnet.
 */
struct InducedMagnetConfig {
  /**
   * @brief Demagnetizing factors (diagonal of the matrix) expressed in the magnet's body frame
   */
  mjtNum N[3] = {1.0/3.0, 1.0/3.0, 1.0/3.0};

  /**
   * @brief Relative permeability of the magnetic material
   */
  double mu_r = 0.0;

  /**
   * @brief Total volume of the magnet
   */
  double V = 0.0;

  /**
   * @brief Reads the plugin's attributes to build the induced magnet
   * @param m MuJoCo model to read from
   * @param instance Instance of the plugin
   * @return An instance of the InducedMagnetConfig class with the fields read from the model
   */
  static InducedMagnetConfig FromModel(const mjModel* m, int instance);
};

/**
 * @Class Class to register a plugin in MuJoCo and apply the magnetic interaction inside the simulation.
 */
class InducedMagnet {
 public:
  InducedMagnet(InducedMagnet&&) = default;
  ~InducedMagnet() = default;

  /**
   * @brief Create an instance of the induced magnet class
   * @param m MuJoCo model to read from
   * @param d MuJoCo data to read from
   * @param instance Instance of the plugin
   * @return An instance of the InducedMagnet class with the fields read from the model
   */
  static std::optional<InducedMagnet> Create(const mjModel* m, mjData* d, int instance);

  /**
   * @brief Compute the magnetic interaction inside the simulation
   * @param m MuJoCo model to read from
   * @param d MuJoCo data to read from
   * @param instance Instance of the plugin
   */
  void Compute(const mjModel* m, mjData* d, int instance);

  /**
   * @brief Display in the scene elements to visualize the interactions inside the simulation
   * @param m MuJoCo model to read from
   * @param d MuJoCo data to read from
   * @param scn Scene in where to visualize
   * @param instance Instance of the plugin
   */
  void Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance);

  /**
   * @brief Register the plugin within the MuJoCo environment
   */
  static void RegisterPlugin();

 protected:
  InducedMagnet(const InducedMagnetConfig& config, const std::unordered_map<int, int>& body_ids, int instance);

 private:
  // This config is shared between all instances
  static MagneticFieldConfig field_config_;

  // Per magnet config
  InducedMagnetConfig config_;
  std::unordered_map<int, int> instance_to_body_ids_;

  // Logging to visualize later
  std::unordered_map<int, std::array<mjtNum, 3>> forces_;
  std::unordered_map<int, std::array<mjtNum, 3>> torques_;

  // Permeability of the vacuum of space
  const mjtNum mu_0_ = 4 * mjPI * 1e-7;
};

}  // namespace mujoco::plugin::passive

#endif  // MUJOCO_PLUGIN_PASSIVE_MAGNET_H_
