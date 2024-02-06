// Copyright 2021 DeepMind Technologies Limited
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

typedef struct _mjString* mjString;
typedef struct _mjDouble* mjDouble;
typedef struct _mjElement* mjElement;



//---------------------------------- Public structs ------------------------------------------------
typedef struct _mjmOrientation {
  double axisangle[4];            // rotation axis and angle
  double xyaxes[6];               // x and y axes
  double zaxis[3];                // z axis (use minimal rotation)
  double euler[3];                // euler rotations
  double fullinertia[6];          // non-axis-aligned inertia matrix
} mjmOrientation;

typedef struct _mjmSite {
  mjElement element;              // only used internally, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  mjString info;                  // message appended to errors
  mjtGeom type;                   // geom type for rendering
  int group;                      // group id, used for visualization
  double size[3];                 // geom size for rendering
  double pos[3];                  // position
  double quat[4];                 // orientation
  mjString material;              // name of material for rendering
  mjDouble userdata;              // user data
  float rgba[4];                  // rgba when material is omitted
  double fromto[6];               // alternative for capsule, cylinder, box, ellipsoid
  mjmOrientation alt;             // alternative orientation specification
} mjmSite;

//---------------------------------- Public API ----------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

// Create model.
MJAPI void* mjm_createModel();

// Delete model.
MJAPI void mjm_deleteModel(void* model);

// Add body to body.
MJAPI void* mjm_addBody(void* body, void* def);

// Add site to body.
MJAPI mjmSite* mjm_addSite(void* body, void* def);

// Find object of given type.
MJAPI void* mjm_findObject(void* model, mjtObj type, const char* name);

// Copy input text to destination string.
MJAPI void mjm_setString(mjString dest, const char* text);

// Copy input array to destination vector.
MJAPI void mjm_setDouble(mjDouble dest, const double* array, int size);

// Get const pointer to mjString data.
MJAPI const char* mjm_getString(mjString source);

// Get const pointer to mjDouble data and its size.
MJAPI const double* mjm_getDouble(mjDouble source, int* size);

// Set frame.
MJAPI void mjm_setFrame(void* dest, void* frame);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_USER_USER_API_H_
