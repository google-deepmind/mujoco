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
#include <mujoco/mjspec.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>


//---------------------------------- Resource Provider ---------------------------------------------

struct mjResource_ {
  char* name;                                   // name of resource (filename, etc)
  void* data;                                   // opaque data pointer
  mjVFS* vfs;                                   // pointer to the VFS
  char timestamp[512];                          // timestamp of the resource
  const struct mjpResourceProvider* provider;   // pointer to the provider
};
typedef struct mjResource_ mjResource;

// callback for opening a resource, returns zero on failure
typedef int (*mjfOpenResource)(mjResource* resource);

// callback for reading a resource
// return number of bytes stored in buffer, return -1 if error
typedef int (*mjfReadResource)(mjResource* resource, const void** buffer);

// callback for closing a resource (responsible for freeing any allocated memory)
typedef void (*mjfCloseResource)(mjResource* resource);

// callback for mounting a resource (provider), returns zero on failure
typedef int (*mjfMountResource)(mjResource* resource);

// callback for unmounting a resource (provider), returns zero on failure
typedef int (*mjfUnmountResource)(mjResource* resource);

// callback for checking if the current resource was modified from the time
// specified by the timestamp
// returns 0 if the resource's timestamp matches the provided timestamp
// returns > 0 if the resource is younger than the given timestamp
// returns < 0 if the resource is older than the given timestamp
typedef int (*mjfResourceModified)(const mjResource* resource, const char* timestamp);

// struct describing a single resource provider
struct mjpResourceProvider {
  const char* prefix;               // prefix for match against a resource name
  mjfOpenResource open;             // opening callback
  mjfReadResource read;             // reading callback
  mjfCloseResource close;           // closing callback
  mjfMountResource mount;           // mounting callback (optional)
  mjfUnmountResource unmount;       // unmounting callback (optional)
  mjfResourceModified modified;     // resource modified callback (optional)
  void* data;                       // opaque data pointer (resource invariant)
};
typedef struct mjpResourceProvider mjpResourceProvider;

//---------------------------------- Decoder -------------------------------------------------------

// function pointer types
// return an mjSpec representing the decoded resource.
typedef mjSpec* (*mjfDecode)(mjResource* resource, const mjVFS* vfs);
// return true if the given resource can be decoded.
typedef int (*mjfCanDecode)(const mjResource* resource);

// the struct defining the decoder plugin's interface
struct mjpDecoder {
  const char* content_type;
  const char* extension;
  // user-facing functions
  mjfCanDecode can_decode;  // quickly check if this decoder can handle the resource
  mjfDecode decode;         // main decoding function
  // the caller takes ownership of the spec returned by decode and is responsible
  // for cleaning it up
};
typedef struct mjpDecoder mjpDecoder;

//---------------------------------- Encoder -------------------------------------------------------

typedef int (*mjfEncode)(const mjSpec* s, const mjModel* m, const mjVFS* vfs,
                         mjResource* resource);

struct mjpEncoder {
  const char* content_type;
  const char* extension;
  mjfEncode encode;  //  Function to encode an mjSpec and mjModel to a mjResource.
  mjfCloseResource close_resource;  // Function to close/free the resource.
};
typedef struct mjpEncoder mjpEncoder;

//---------------------------------- Plugins -------------------------------------------------------

typedef enum mjtPluginCapabilityBit_ {
  mjPLUGIN_ACTUATOR = 1<<0,       // actuator forces
  mjPLUGIN_SENSOR   = 1<<1,       // sensor measurements
  mjPLUGIN_PASSIVE  = 1<<2,       // passive forces
  mjPLUGIN_SDF      = 1<<3,       // signed distance fields
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
  void (*reset)(const mjModel* m, mjtNum* plugin_state, void* plugin_data, int instance);

  // called when the plugin needs to update its outputs (required)
  void (*compute)(const mjModel* m, mjData* d, int instance, int capability_bit);

  // called when time integration occurs (optional)
  void (*advance)(const mjModel* m, mjData* d, int instance);

  // called by mjv_updateScene (optional)
  void (*visualize)(const mjModel*m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance);

  // methods specific to actuators (optional)

  // updates the actuator plugin's entries in act_dot
  // called after native act_dot is computed and before the compute callback
  void (*actuator_act_dot)(const mjModel* m, mjData* d, int instance);

  // methods specific to signed distance fields (optional)

  // signed distance from the surface
  mjtNum (*sdf_distance)(const mjtNum point[3], const mjData* d, int instance);

  // gradient of distance with respect to local coordinates
  void (*sdf_gradient)(mjtNum gradient[3], const mjtNum point[3], const mjData* d, int instance);

  // called during compilation for marching cubes
  mjtNum (*sdf_staticdistance)(const mjtNum point[3], const mjtNum* attributes);

  // convert attributes and provide defaults if not present
  void (*sdf_attribute)(mjtNum attribute[], const char* name[], const char* value[]);

  // bounding box of implicit surface
  void (*sdf_aabb)(mjtNum aabb[6], const mjtNum* attributes);
};
typedef struct mjpPlugin_ mjpPlugin;

struct mjSDF_ {
  const mjpPlugin** plugin;
  int* id;
  mjtSDFType type;
  mjtNum* relpos;
  mjtNum* relmat;
  mjtGeom* geomtype;
};
typedef struct mjSDF_ mjSDF;

//------------------------------------ Initialization ----------------------------------------------

#if defined(__has_attribute)
  #if __has_attribute(constructor)
    #define mjPLUGIN_LIB_INIT(n)                                     \
      static void _mj_init_##n(void) __attribute__((constructor));   \
      static void _mj_init_##n(void)
  #endif
#elif defined(_MSC_VER)
    // on x86, symbols are decorated with a leading underscore
    #ifdef _M_IX86
      #define LINKER_NAME "__mj_ptr_"
    #else
      #define LINKER_NAME "_mj_ptr_"
    #endif

    #pragma section(".CRT$XCU", read)

    #if !defined(mjEXTERNC)
      #if defined(__cplusplus)
        #define mjEXTERNC extern "C"
      #else
        #define mjEXTERNC
      #endif  // defined(__cplusplus)
    #endif  // !defined(mjEXTERNC)

    #define mjPLUGIN_LIB_INIT(n)                                                          \
      static void __cdecl _mj_init_##n(void);                                             \
      /* use mjEXTERNC to prevent C++ name mangling */                                    \
      /* allocate the function pointer to the .CRT$XCU section of the executable */       \
      /* functions in this section are executed on startup before calling main() */       \
      mjEXTERNC __declspec(allocate(".CRT$XCU"))                                          \
      void (__cdecl * _mj_ptr_##n)(void) = _mj_init_##n;                                  \
      /* Force the linker to include the pointer symbol */                                \
      __pragma(comment(linker, "/include:" LINKER_NAME #n))                               \
      static void __cdecl _mj_init_##n(void)

#else
    #error "Unknown compiler: Plugin registration not supported."
#endif

// function pointer type for mj_loadAllPluginLibraries callback
typedef void (*mjfPluginLibraryLoadCallback)(const char* filename, int first, int count);

#endif  // MUJOCO_INCLUDE_MJPLUGIN_H_
