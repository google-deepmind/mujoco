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
#include <fstream>
#include <variant>

#include <mujoco/mujoco.h>

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

class LinearMagneticField {
public:
  LinearMagneticField(const mjtNum B0[3], const mjtNum dB[9]) {
    mju_copy3(B0_, B0);
    mju_copy(dB_, dB, 9);
  }

  void getField(mjtNum res[3], const mjtNum pos[3]) const {
    mju_mulMatVec3(res, dB_, pos);
    mju_add3(res, B0_, res);
  }

  void getGradient(mjtNum res[9]) const {
    mju_copy(res, dB_, 9);
  }

private:
  mjtNum B0_[3];
  mjtNum dB_[9];
};

class SphericalMagneticField {
public:
  SphericalMagneticField(const mjtNum center[3], const mjtNum Bmax[3], mjtNum radius)
      : radius_(radius) {
    mju_copy3(center_, center);
    mju_copy3(Bmax_, Bmax);
  }

  void getField(mjtNum res[3], const mjtNum pos[3]) const {
    mjtNum r[3];
    mju_sub3(r, pos, center_);

    mjtNum dist = mju_norm3(r);
    if (dist >= radius_) {
      mju_zero3(res);
      return;
    }

    mjtNum pct = dist / radius_;
    mjtNum factor = 1.0 - (pct * pct);

    mju_scl3(res, Bmax_, factor);
  }


  void getGradient(mjtNum res[9], const mjtNum pos[3]) const {
    mjtNum r[3];
    mju_sub3(r, pos, center_);

    mjtNum dist = mju_norm3(r);
    if (dist >= radius_) {
      mju_zero(res, 9);
      return;
    }

    mjtNum scale = -2.0 / (radius_ * radius_);

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        res[i * 3 + j] = scale * Bmax_[i] * r[j];
      }
    }
  }

private:
  mjtNum center_[3];
  mjtNum Bmax_[3];
  mjtNum radius_;
};

/**
 * @Class Class to register a plugin in MuJoCo and apply the magnetic interaction inside the simulation.
 */
class MagneticPlugin {
 public:
  // ---------------------------------------------------------------------------
  // Internal Sub-components
  // ---------------------------------------------------------------------------

  class Field {
   public:
    explicit Field(const MagneticFieldConfig& config);
  };

  class InducedMagnet {
   public:
    InducedMagnet(const InducedMagnetConfig& config, const std::unordered_map<int, int>& body_ids, int instance);

    void Compute(const mjModel* m, mjData* d, int instance);
    void Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance);

   private:
    InducedMagnetConfig config_;
    std::unordered_map<int, int> instance_to_body_ids_;

    // Logging to visualize later
    std::unordered_map<int, std::array<mjtNum, 3>> forces_;
    std::unordered_map<int, std::array<mjtNum, 3>> torques_;
  };

  class Magnetometer {
   public:
    explicit Magnetometer(int instance);

    void ComputeSensor(const mjModel* m, mjData* d, int instance);

   private:
    std::ofstream file_;
  };

  // ---------------------------------------------------------------------------
  // Main Plugin Interface
  // ---------------------------------------------------------------------------

  MagneticPlugin(MagneticPlugin&&) = default;
  ~MagneticPlugin() = default;

  /**
   * @brief Create an instance of the specific sub-plugin class
   * @param m MuJoCo model to read from
   * @param d MuJoCo data to read from
   * @param instance Instance of the plugin
   * @return An instance of the MagneticPlugin class wrapping the chosen internal type
   */
  static std::optional<MagneticPlugin> Create(const mjModel* m, mjData* d, int instance);

  /**
   * @brief Compute the magnetic interaction inside the simulation (delegates to inner class)
   */
  void Compute(const mjModel* m, mjData* d, int instance);

  /**
   * @brief Computes the value for the magnetometer sensor (delegates to inner class)
   */
  void ComputeSensor(const mjModel* m, mjData* d, int instance);

  /**
   * @brief Display in the scene elements to visualize the interactions (delegates to inner class)
   */
  void Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance);

  /**
   * @brief Register the plugin within the MuJoCo environment
   */
  static void RegisterPlugin();

  /**
   * @brief Modify the field and gradient at the origin
   */
  static void setGlobalField(const mjtNum B0[3], const mjtNum dB[3]);

 private:
  // Private template constructor used by `Create` to wrap the specific implementation
  template <typename T>
  explicit MagneticPlugin(T&& impl) : impl_(std::forward<T>(impl)) {}

  // Holds exactly one of the three internal classes per plugin instance
  std::variant<Field, InducedMagnet, Magnetometer> impl_;

  // This config is shared between all instances
  static MagneticFieldConfig field_config_;

  // Permeability of the vacuum of space
  static constexpr mjtNum mu_0_ = 4 * mjPI * 1e-7;
};

}  // namespace mujoco::plugin::passive

#endif  // MUJOCO_PLUGIN_PASSIVE_MAGNET_H_