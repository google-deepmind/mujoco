#include "thermal.h"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco::plugin::thermal {

constexpr char kattrThermalCapacitance[] = "C";
constexpr char kattrThermalResistance[] = "Rth";
constexpr char kAttrElectricNominalResistance[] = "RNorm";
constexpr char kTemperatureCoefficientofResisteance[] = "TempCoeff";
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
  config.thermal_capacitance = ReadOptionalDoubleAttr(m, instance, kattrThermalCapacitance).value_or(0);
  config.thermal_resistance = ReadOptionalDoubleAttr(m, instance, kattrThermalResistance).value_or(0);
  config.electrical_nominal_resistance = ReadOptionalDoubleAttr(m, instance, kAttrElectricNominalResistance).value_or(0);
  config.temperature_coefficient_of_resistance = ReadOptionalDoubleAttr(m, instance, kTemperatureCoefficientofResisteance).value_or(0);
  config.torque_constant_25 = ReadOptionalDoubleAttr(m, instance, kAttrTorqueConstant25).value_or(0);
  config.torque_constant_130 = ReadOptionalDoubleAttr(m, instance, kAttrTorqueConstant130).value_or(0);
  config.gear_ratio = ReadOptionalDoubleAttr(m, instance, kAttrGearRatio).value_or(0);

  return config;
}

void Thermal::RegisterPlugin() 
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "mujoco.thermal";
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;
  plugin.needstage = mjSTAGE_ACC;


  std::vector<const char *> attributes = {kattrThermalCapacitance,
                                          kattrThermalResistance,
                                          kAttrElectricNominalResistance,
                                          kTemperatureCoefficientofResisteance,
                                          kAttrTorqueConstant25,
                                          kAttrTorqueConstant130,
                                          kAttrGearRatio};
  plugin.nattribute = attributes.size();
  plugin.attributes = attributes.data();

  // Can only run after forces have been computed.
  plugin.needstage = mjSTAGE_ACC;

  plugin.nstate = +[](const mjModel *m, int instance) {
    int nstate = 0;
    for(int i = 0; i < m->nsensor; i++) {
      if (m->sensor_plugin[i] == instance) {
        nstate++;
      }
    }
    return nstate;
  }; // We are allocating plugin_state array to record the true current temperature of the actuator. And
     // the sensor output as "observed temperature". In the future we can add a
     // sensor noise model to this.
  plugin.nsensordata = +[](const mjModel *m, int instance, int sensor_id) {
    return 1; // The sensor output is the observed temperature.
  };

  plugin.init = +[](const mjModel *m, mjData *d, int instance) {
    std::unique_ptr<Thermal> thermal = Thermal::Create(m, d, instance);
    if (thermal == nullptr) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(thermal.release());
    return 0;
  };

  plugin.destroy = +[](mjData *d, int instance) {
    delete reinterpret_cast<Thermal *>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.reset = +[](const mjModel *m, mjtNum *plugin_state, void *plugin_data,
                     int instance) {
    auto *thermal = reinterpret_cast<Thermal *>(plugin_data);
    thermal->Reset(m, plugin_state, instance);
  };

  plugin.compute =
      +[](const mjModel *m, mjData *d, int instance, int capability_bit) {
        auto *thermal = reinterpret_cast<Thermal *>(d->plugin_data[instance]);
        thermal->Compute(m, d, instance);
      };
  mjp_registerPlugin(&plugin);
}

Thermal::Thermal(ThermalConfig config, std::vector<int> actuators)
    : _model(std::move(config)), _sensors(std::move(actuators)) {}

std::unique_ptr<Thermal> Thermal::Create(const mjModel *m, mjData* d, int instance) 
{
    ThermalConfig config = ThermalConfig::FromModel(m, instance);
    
    if (config.thermal_capacitance <= 0) {
        mju_warning("Invalid thermal_capacitance (%.6f) for thermal plugin instance %d: must be > 0", config.thermal_capacitance, instance);
        return nullptr;
    }
    if (config.thermal_resistance <= 0) {
        mju_warning("Invalid thermal_resistance (%.6f) for thermal plugin instance %d: must be > 0", config.thermal_resistance, instance);
        return nullptr;
    }
    if (config.electrical_nominal_resistance <= 0) {
        mju_warning("Invalid electrical_nominal_resistance (%.6f) for thermal plugin instance %d: must be > 0", config.electrical_nominal_resistance, instance);
        return nullptr;
    }
    if (config.temperature_coefficient_of_resistance < 0) {
        mju_warning("Invalid temperature_coefficient_of_resistance (%.6f) for thermal plugin instance %d: must be >= 0", config.temperature_coefficient_of_resistance, instance);
        return nullptr;
    }
    if (config.torque_constant_25 <= 0) {
        mju_warning("Invalid torque_constant_25 (%.6f) for thermal plugin instance %d: must be > 0", config.torque_constant_25, instance);
        return nullptr;
    }
    if (config.torque_constant_130 <= 0) {
        mju_warning("Invalid torque_constant_130 (%.6f) for thermal plugin instance %d: must be > 0", config.torque_constant_130, instance);
        return nullptr;
    }
    if (config.gear_ratio <= 0) {
        mju_warning("Invalid gear_ratio (%.6f) for thermal plugin instance %d: must be > 0", config.gear_ratio, instance);
        return nullptr;
    }

    std::vector<int> sensors; 
    for(int i = 0; i < m->nsensor; i++){
        if (m->sensor_plugin[i] == instance) {
            sensors.push_back(i);
        }
    }

    if (sensors.empty()) {
        mju_warning("No sensors found for this plugin instance %d", instance);
        return nullptr;
    }

    for(int sensor: sensors) {
       if( m->sensor_objtype[sensor] != mjOBJ_ACTUATOR && m->sensor_objid[sensor] == -1)
         {
                mju_warning("Sensor %d is not associated with an actuator for plugin instance %d", sensor, instance);
                return nullptr;
        }
    }

    //TODO: Ask for a better way to handle this.
    int ambient_temp_id = mj_name2id(m, mjOBJ_NUMERIC, Thermal::kAmbientTemperature);
    if(ambient_temp_id < 0)
    {
        mju_warning("Ambient temperature value not found");
        return nullptr;
    }

    double ambient_temp = m->numeric_data[m->numeric_adr[ambient_temp_id]];
    if(ambient_temp < 273.15)
    {
        mju_warning("Ambient temperature below absolute zero");
        return nullptr;
    }

    for(int i = 0; i < m->plugin_statenum[instance]; i++) {
      // Initialize the plugin state to the ambient temperature.
      d->plugin_state[m->plugin_stateadr[instance] + i] = ambient_temp;
    }


    return std::unique_ptr<Thermal>(new Thermal(config,  std::move(sensors)));
}

void Thermal::Reset(const mjModel* m, mjtNum* plugin_state, int instance) {
    int ambient_temp_id = mj_name2id(m, mjOBJ_NUMERIC, Thermal::kAmbientTemperature);
  double ambient_temp = m->numeric_data[m->numeric_adr[ambient_temp_id]];

    for(int i = 0; i < m->plugin_statenum[instance]; i++) {
      // Initialize the plugin state to the ambient temperature.
      plugin_state[m->plugin_stateadr[instance] + i] = ambient_temp;
    }
}

void Thermal::Compute(const mjModel* m, mjData* d, int instance)
{
    int ambient_temp_id = mj_name2id(m, mjOBJ_NUMERIC, Thermal::kAmbientTemperature);
    double ambient_temp = m->numeric_data[m->numeric_adr[ambient_temp_id]];

    for(int i = 0 ; i < _sensors.size(); i++){
        double current_temp = d->plugin_state[m->plugin_stateadr[instance] + i];
        double Kt = _model.torque_constant_25 + ((_model.torque_constant_130 - _model.torque_constant_25) / (403.15 - 298.15)) * ( current_temp- 298.15);
        double current = d->actuator_force[m->sensor_objid[_sensors[i]]] / Kt * _model.gear_ratio;
        double resistance = _model.electrical_nominal_resistance * (1 + _model.temperature_coefficient_of_resistance * (current_temp - 298.15));
        double power_in = current * current * resistance;
        double thermal_resistance = _model.thermal_resistance;
        double thermal_capacitance = _model.thermal_capacitance;
        double power_out = (current_temp - ambient_temp) / thermal_resistance;
        double dTdt = (power_in - power_out) / thermal_capacitance;
        current_temp += dTdt * m->opt.timestep; // Update the temperature based on
        d->plugin_state[m->plugin_stateadr[instance] + i] = current_temp;
        d->sensordata[m->sensor_adr[_sensors[i]]] = current_temp;
    }
}

} // namespace mujoco::plugin::thermal