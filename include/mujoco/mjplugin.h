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

#ifndef MUJOCO_INCLUDE_MJPLUGIN_H_
#define MUJOCO_INCLUDE_MJPLUGIN_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

typedef enum mjtPluginTypeBit_ {
  mjPLUGIN_ACTUATOR = 1<<0,
  mjPLUGIN_SENSOR   = 1<<1
} mjtPluginTypeBit;

struct mjpPlugin_ {
  const char* name;    // globally unique name identifying the plugin

  int nattribute;                 // number of configuration attributes
  const char* const* attributes;  // name of configuration attributes

  int type;            // bitfield of mjtPluginTypeBits specifying the plugin type
  int needstage;       // an mjtStage enum value specifying the sensor computation stage

  // number of mjtNums needed to store the state of a plugin instance (required)
  int (*nstate)(const mjModel* m, int instance);

  // dimension of the specified sensor's output (required only for sensor plugins)
  int (*nsensordata)(const mjModel* m, int instance, int sensor_id);

  // called when a new mjData is being created (required)
  void (*init)(const mjModel* m, mjData* d, int instance);

  // called when an mjData is being freed (optional)
  void (*destroy)(mjData* d, int instance);

  // called when an mjData is being copied (optional)
  void (*copy)(mjData* dest, const mjModel* m, const mjData* src, int instance);

  // called when an mjData is being reset (required)
  void (*reset)(const mjModel* m, mjData* d, int instance);

  // called when the plugin needs to update its outputs (required)
  void (*compute)(const mjModel* m, mjData* d, int instance, int type);

  // called when time integration occurs (optional)
  void (*advance)(const mjModel* m, mjData* d, int instance);
};
typedef struct mjpPlugin_ mjpPlugin;

#endif  // MUJOCO_INCLUDE_MJPLUGIN_H_
