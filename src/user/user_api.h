// Copyright 2024 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_USER_USER_API_H_
#define MUJOCO_SRC_USER_USER_API_H_

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

typedef struct _mjString* mjString;
typedef struct _mjDouble* mjDouble;
typedef struct _mjElement* mjElement;


// this is a C-API
#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------- Public structs ------------------------------------------------

typedef struct _mjmOrientation {
  double axisangle[4];            // rotation axis and angle
  double xyaxes[6];               // x and y axes
  double zaxis[3];                // z axis (use minimal rotation)
  double euler[3];                // euler rotations
} mjmOrientation;


typedef struct _mjmPlugin {
  bool active;
  mjString name;
  mjString instance_name;
  mjElement instance;
} mjmPlugin;


typedef struct _mjmBody {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  double pos[3];                  // frame position
  double quat[4];                 // frame orientation
  mjtByte mocap;                  // is this a mocap body
  mjmOrientation alt;             // frame alternative orientation
  double gravcomp;                // gravity compensation
  mjDouble userdata;              // user data
  double ipos[3];                 // inertial frame position
  double iquat[4];                // inertial frame orientation
  double mass;                    // mass
  double inertia[3];              // diagonal inertia (in i-frame)
  mjmOrientation ialt;            // inertial frame alternative orientation
  double fullinertia[6];          // non-axis-aligned inertia matrix
  mjtByte explicitinertial;       // whether to save the body with an explicit inertial clause
  mjmPlugin plugin;               // passive force plugin
  mjString info;                  // message appended to errors
} mjmBody;


typedef struct _mjmSite {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  mjtGeom type;                   // geom type for rendering
  int group;                      // group id, used for visualization
  double pos[3];                  // position
  double quat[4];                 // orientation
  mjString material;              // name of material for rendering
  double size[3];                 // geom size for rendering
  double fromto[6];               // alternative for capsule, cylinder, box, ellipsoid
  mjmOrientation alt;             // alternative orientation specification
  float rgba[4];                  // rgba when material is omitted
  mjDouble userdata;              // user data
  mjString info;                  // message appended to errors
} mjmSite;


//---------------------------------- Public API ----------------------------------------------------

// Create model.
MJAPI void* mjm_createModel();

// Delete model.
MJAPI void mjm_deleteModel(void* modelspec);

// Add child body to body, return child spec.
MJAPI mjmBody* mjm_addBody(mjmBody* body, void* defspec);

// Add site to body, return site spec.
MJAPI mjmSite* mjm_addSite(mjmBody* body, void* defspec);

// Add joint to body.
MJAPI void* mjm_addJoint(mjmBody* body, void* defspec);

// Add freejoint to body.
MJAPI void* mjm_addFreeJoint(mjmBody* body);

// Add geom to body.
MJAPI void* mjm_addGeom(mjmBody* body, void* defspec);

// Add camera to body.
MJAPI void* mjm_addCamera(mjmBody* body, void* defspec);

// Add light to body.
MJAPI void* mjm_addLight(mjmBody* body, void* defspec);

// Add frame to body.
MJAPI void* mjm_addFrame(mjmBody* body, void* parentframe);

// Add plugin to model.
MJAPI mjElement mjm_addPlugin(void* model);

// Get model from body.
MJAPI void* mjm_getModel(mjmBody* body);

// Get default corresponding to an mjElement.
MJAPI void* mjm_getDefault(mjElement element);

// Finding body with given name in model.
MJAPI mjmBody* mjm_findBody(void* modelspec, const char* name);

// Finding body with given name in body.
MJAPI mjmBody* mjm_findChild(mjmBody* body, const char* name);

// Get element id.
MJAPI int mjm_getId(mjElement element);

// Copy input text to destination string.
MJAPI void mjm_setString(mjString dest, const char* text);

// Copy input array to destination vector.
MJAPI void mjm_setDouble(mjDouble dest, const double* array, int size);

// Get string contents.
MJAPI const char* mjm_getString(mjString source);

// Get double array contents and optionally its size.
MJAPI const double* mjm_getDouble(mjDouble source, int* size);

// Set default.
MJAPI void mjm_setDefault(mjElement element, void* defspec);

// Set frame.
MJAPI void mjm_setFrame(mjElement dest, void* frame);

// Compute quat and inertia from body->fullinertia.
MJAPI const char* mjm_setFullInertia(mjmBody* body, double quat[4], double inertia[3]);


//---------------------------------- Initialization functions --------------------------------------

// Default body attributes.
MJAPI void mjm_defaultBody(mjmBody& body);

// Default site attributes.
MJAPI void mjm_defaultSite(mjmSite& site);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_USER_USER_API_H_
