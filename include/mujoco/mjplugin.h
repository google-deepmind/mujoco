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
#include <mujoco/mjvisualize.h>


//---------------------------------- Resource Provider ---------------------------------------------

#define mjVFS_PREFIX    "vfs://"  // prefix for VFS providers

// callback for opeing a resource, returns zero on failure
typedef int (*mjfOpenResource)(mjResource* resource);

// callback for reading a resource
// return number of bytes stored in buffer, return -1 if error
typedef int (*mjfReadResource)(mjResource* resource, const void** buffer);

// callback for closing a resource (responsible for freeing any allocated memory)
typedef void (*mjfCloseResource)(mjResource* resource);

// struct describing a single resource provider
struct mjpResourceProvider_ {
  const char* prefix;             // prefix for match against a resource name
  mjfOpenResource open;           // opening callback
  mjfReadResource read;           // reading callback
  mjfCloseResource close;         // closing callback
  void* data;                     // opaque data pointer (resource invariant)
};
typedef struct mjpResourceProvider_ mjpResourceProvider;


//---------------------------------- Plugins -------------------------------------------------------

typedef enum mjtPluginCapabilityBit_ {
  mjPLUGIN_ACTUATOR = 1<<0,       // actuator forces
  mjPLUGIN_SENSOR   = 1<<1,       // sensor measurements
  mjPLUGIN_PASSIVE  = 1<<2,       // passive forces
} mjtPluginCapabilityBit;

struct mjpPlugin_ {
  const char* name;               // globally unique name identifying the plugin

  int nattribute;                 // number of configuration attributes
  const char* const* attributes;  // name of configuration attributes

  int capabilityflags;            // plugin capabilities: bitfield of mjtPluginCapabilityBit
  int needstage;                  // sensor computation stage (mjtStage)

  // number of mjtNums needed to store the state of a plugin instance (required)
  int (*nstate)(const mjModel* m, int instance);

  // dimension of the specified sensor's output (required only for sensor plugins)
  int (*nsensordata)(const mjModel* m, int instance, int sensor_id);

  // called when a new mjData is being created (required), returns 0 on success or -1 on failure
  int (*init)(const mjModel* m, mjData* d, int instance);

  // called when an mjData is being freed (optional)
  void (*destroy)(mjData* d, int instance);

  // called when an mjData is being copied (optional)
  void (*copy)(mjData* dest, const mjModel* m, const mjData* src, int instance);

  // called when an mjData is being reset (required)
  void (*reset)(const mjModel* m, double* plugin_state, void* plugin_data, int instance);

  // called when the plugin needs to update its outputs (required)
  void (*compute)(const mjModel* m, mjData* d, int instance, int capability_bit);

  // called when time integration occurs (optional)
  void (*advance)(const mjModel* m, mjData* d, int instance);

  // called by mjv_updateScene (optional)
  void (*visualize)(const mjModel*m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance);
};
typedef struct mjpPlugin_ mjpPlugin;

#if defined(__has_attribute)

  #if __has_attribute(constructor)
    #define mjPLUGIN_LIB_INIT __attribute__((constructor)) static void _mjplugin_init(void)
  #endif  // __has_attribute(constructor)

#elif defined(_MSC_VER)

  #ifndef mjDLLMAIN
    #define mjDLLMAIN DllMain
  #endif

  #if !defined(mjEXTERNC)
    #if defined(__cplusplus)
      #define mjEXTERNC extern "C"
    #else
      #define mjEXTERNC
    #endif  // defined(__cplusplus)
  #endif  // !defined(mjEXTERNC)

  // NOLINTBEGIN(runtime/int)
  #define mjPLUGIN_LIB_INIT                                                                 \
    static void _mjplugin_dllmain(void);                                                    \
    mjEXTERNC int __stdcall mjDLLMAIN(void* hinst, unsigned long reason, void* reserved) {  \
      if (reason == 1) {                                                                    \
        _mjplugin_dllmain();                                                                \
      }                                                                                     \
      return 1;                                                                             \
    }                                                                                       \
    static void _mjplugin_dllmain(void)
  // NOLINTEND(runtime/int)

#endif  // defined(_MSC_VER)

// function pointer type for mj_loadAllPluginLibraries callback
typedef void (*mjfPluginLibraryLoadCallback)(const char* filename, int first, int count);

#endif  // MUJOCO_INCLUDE_MJPLUGIN_H_
