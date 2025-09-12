#include "thermal.h"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco::plugin::thermal {

constexpr char kAttrThermalCapacitance[] = "C";
constexpr char kAttrThermalResistance[] = "Rth";
constexpr char kAttrElectricalNominalResistance[] = "RNorm";
constexpr char kAttrTemperatureCoefficientOfResistance[] = "TempCoeff";
constexpr char kAttrTorqueConstant25[] = "Kt25";
constexpr char kAttrTorqueConstant130[] = "Kt130";
constexpr char kAttrGearRatio[] = "G";

std::optional<mjtNum> ReadOptionalDoubleAttr(const mjModel* m, int instance,
                                             const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;
  }
  return std::strtod(value, nullptr);
}

ThermalConfig ThermalConfig::FromModel(const mjModel* m, int instance) {
  ThermalConfig config;
  config.thermal_capacitance = 
      ReadOptionalDoubleAttr(m, instance, kAttrThermalCapacitance).value_or(0);
  config.thermal_resistance = 
      ReadOptionalDoubleAttr(m, instance, kAttrThermalResistance).value_or(0);
  config.electrical_nominal_resistance = 
      ReadOptionalDoubleAttr(m, instance, kAttrElectricalNominalResistance)
          .value_or(0);
  config.temperature_coefficient_of_resistance = 
      ReadOptionalDoubleAttr(m, instance, kAttrTemperatureCoefficientOfResistance)
          .value_or(0);
  config.torque_constant_25 = 
      ReadOptionalDoubleAttr(m, instance, kAttrTorqueConstant25).value_or(0);
  config.torque_constant_130 = 
      ReadOptionalDoubleAttr(m, instance, kAttrTorqueConstant130).value_or(0);
  config.gear_ratio = 
      ReadOptionalDoubleAttr(m, instance, kAttrGearRatio).value_or(0);

  return config;
}

void Thermal::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "mujoco.sensor.thermal";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  std::vector<const char*> attributes = {
      kAttrThermalCapacitance,
      kAttrThermalResistance,
      kAttrElectricalNominalResistance,
      kAttrTemperatureCoefficientOfResistance,
      kAttrTorqueConstant25,
      kAttrTorqueConstant130,
      kAttrGearRatio};
  plugin.nattribute = attributes.size();
  plugin.attributes = attributes.data();

  // Can only run after forces have been computed.
  plugin.needstage = mjSTAGE_ACC;

  // We are allocating plugin_state array to record the true current
  // temperature of the actuator. The sensor output is "observed temperature".
  // In the future we can add a sensor noise model to this.
  plugin.nstate = +[](const mjModel* m, int instance) {
    int nstate = 0;
    for (int i = 0; i < m->nsensor; ++i) {
      if (m->sensor_plugin[i] == instance) {
        nstate++;
      }
    }
    return nstate;
  };

  // The sensor output is the observed temperature.
  plugin.nsensordata = +[](const mjModel* m, int instance, int sensor_id) {
    return 1;
  };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    std::unique_ptr<Thermal> thermal = Thermal::Create(m, d, instance);
    if (thermal == nullptr) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(thermal.release());
    return 0;
  };

  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Thermal*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data,
                     int instance) {
    auto* thermal = reinterpret_cast<Thermal*>(plugin_data);
    thermal->Reset(m, plugin_state, instance);
  };

  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* thermal = reinterpret_cast<Thermal*>(d->plugin_data[instance]);
        thermal->Compute(m, d, instance);
      };
  mjp_registerPlugin(&plugin);
}

Thermal::Thermal(ThermalConfig config, std::vector<int> actuators)
    : model_(std::move(config)), sensors_(std::move(actuators)) {}

std::unique_ptr<Thermal> Thermal::Create(const mjModel* m, mjData* d,
                                          int instance) {
  ThermalConfig config = ThermalConfig::FromModel(m, instance);

  if (config.thermal_capacitance <= 0) {
    mju_warning(
        "Invalid thermal_capacitance (%.6f) for thermal plugin instance %d: "
        "must be > 0",
        config.thermal_capacitance, instance);
    return nullptr;
  }
  if (config.thermal_resistance <= 0) {
    mju_warning(
        "Invalid thermal_resistance (%.6f) for thermal plugin instance %d: "
        "must be > 0",
        config.thermal_resistance, instance);
    return nullptr;
  }
  if (config.electrical_nominal_resistance <= 0) {
    mju_warning(
        "Invalid electrical_nominal_resistance (%.6f) for thermal plugin "
        "instance %d: must be > 0",
        config.electrical_nominal_resistance, instance);
    return nullptr;
  }
  if (config.temperature_coefficient_of_resistance < 0) {
    mju_warning(
        "Invalid temperature_coefficient_of_resistance (%.6f) for thermal "
        "plugin instance %d: must be >= 0",
        config.temperature_coefficient_of_resistance, instance);
    return nullptr;
  }
  if (config.torque_constant_25 <= 0) {
    mju_warning(
        "Invalid torque_constant_25 (%.6f) for thermal plugin instance %d: "
        "must be > 0",
        config.torque_constant_25, instance);
    return nullptr;
  }
  if (config.torque_constant_130 <= 0) {
    mju_warning(
        "Invalid torque_constant_130 (%.6f) for thermal plugin instance %d: "
        "must be > 0",
        config.torque_constant_130, instance);
    return nullptr;
  }
  if (config.gear_ratio <= 0) {
    mju_warning(
        "Invalid gear_ratio (%.6f) for thermal plugin instance %d: "
        "must be > 0",
        config.gear_ratio, instance);
    return nullptr;
  }

  std::vector<int> sensors;
  for (int i = 0; i < m->nsensor; ++i) {
    if (m->sensor_plugin[i] == instance) {
      sensors.push_back(i);
    }
  }

  if (sensors.empty()) {
    mju_warning("No sensors found for this plugin instance %d", instance);
    return nullptr;
  }

  for (int sensor : sensors) {
    if (m->sensor_objtype[sensor] != mjOBJ_ACTUATOR && 
        m->sensor_objid[sensor] == -1) {
      mju_warning(
          "Sensor %d is not associated with an actuator for plugin instance %d",
          sensor, instance);
      return nullptr;
    }
  }

  // TODO(b/123456): Ask for a better way to handle this.
  int ambient_temp_id = 
      mj_name2id(m, mjOBJ_NUMERIC, Thermal::kAmbientTemperature);
  if (ambient_temp_id < 0) {
    mju_warning("Ambient temperature value not found");
    return nullptr;
  }

  double ambient_temp = m->numeric_data[m->numeric_adr[ambient_temp_id]];
  if (ambient_temp < 273.15) {
    mju_warning("Ambient temperature below absolute zero");
    return nullptr;
  }

  // Initialize the plugin state to the ambient temperature.
  for (int i = 0; i < m->plugin_statenum[instance]; ++i) {
    d->plugin_state[m->plugin_stateadr[instance] + i] = ambient_temp;
  }

  return std::unique_ptr<Thermal>(new Thermal(config, std::move(sensors)));
}

void Thermal::Reset(const mjModel* m, mjtNum* plugin_state, int instance) {
  int ambient_temp_id = 
      mj_name2id(m, mjOBJ_NUMERIC, Thermal::kAmbientTemperature);
  double ambient_temp = m->numeric_data[m->numeric_adr[ambient_temp_id]];

  // Initialize the plugin state to the ambient temperature.
  for (int i = 0; i < m->plugin_statenum[instance]; ++i) {
    plugin_state[m->plugin_stateadr[instance] + i] = ambient_temp;
  }
}

void Thermal::Compute(const mjModel* m, mjData* d, int instance) {
  int ambient_temp_id = 
      mj_name2id(m, mjOBJ_NUMERIC, Thermal::kAmbientTemperature);
  double ambient_temp = m->numeric_data[m->numeric_adr[ambient_temp_id]];

  for (size_t i = 0; i < sensors_.size(); ++i) {
    double current_temp = d->plugin_state[m->plugin_stateadr[instance] + i];
    
    // Compute torque constant with temperature compensation.
    constexpr double kTempRef25 = 298.15;   // 25°C in Kelvin
    constexpr double kTempRef130 = 403.15;  // 130°C in Kelvin
    double kt = model_.torque_constant_25 + 
        ((model_.torque_constant_130 - model_.torque_constant_25) / 
         (kTempRef130 - kTempRef25)) * (current_temp - kTempRef25);
    
    // Compute motor current from actuator force.
    double current = d->actuator_force[m->sensor_objid[sensors_[i]]] / 
                     (kt * model_.gear_ratio);
    
    // Compute temperature-dependent resistance.
    double resistance = model_.electrical_nominal_resistance * 
        (1 + model_.temperature_coefficient_of_resistance * 
         (current_temp - kTempRef25));
    
    // Compute power dissipation and temperature change.
    double power_in = current * current * resistance;
    double thermal_resistance = model_.thermal_resistance;
    double thermal_capacitance = model_.thermal_capacitance;
    double power_out = (current_temp - ambient_temp) / thermal_resistance;
    double dt_dt = (power_in - power_out) / thermal_capacitance;
    
    // Update temperature using Euler integration.
    current_temp += dt_dt * m->opt.timestep;
    d->plugin_state[m->plugin_stateadr[instance] + i] = current_temp;
    d->sensordata[m->sensor_adr[sensors_[i]]] = current_temp;
  }
}

}  // namespace mujoco::plugin::thermal