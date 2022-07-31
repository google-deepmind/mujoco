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

#ifndef MUJOCO_SRC_XML_XML_BASE_H_
#define MUJOCO_SRC_XML_XML_BASE_H_

#include <string>

#include "tinyxml2.h"
#include <mujoco/mjmodel.h>
#include "user/user_model.h"
#include "user/user_objects.h"
#include "xml/xml_util.h"


// keyword maps (defined in implementation files)
extern const int joint_sz;
extern const int camlight_sz;
extern const int light_sz;
extern const int integrator_sz;
extern const int collision_sz;
extern const int cone_sz;
extern const int jac_sz;
extern const int solver_sz;
extern const int equality_sz;
extern const int texture_sz;
extern const int builtin_sz;
extern const int mark_sz;
extern const int dyn_sz;
extern const int gain_sz;
extern const int bias_sz;
extern const int stage_sz;
extern const int datatype_sz;
extern const mjMap coordinate_map[];
extern const mjMap angle_map[];
extern const mjMap enable_map[];
extern const mjMap bool_map[];
extern const mjMap fluid_map[];
extern const mjMap TFAuto_map[];
extern const mjMap joint_map[];
extern const mjMap geom_map[];
extern const mjMap camlight_map[];
extern const mjMap light_map[];
extern const mjMap integrator_map[];
extern const mjMap collision_map[];
extern const mjMap impedance_map[];
extern const mjMap reference_map[];
extern const mjMap cone_map[];
extern const mjMap jac_map[];
extern const mjMap solver_map[];
extern const mjMap equality_map[];
extern const mjMap texture_map[];
extern const mjMap builtin_map[];
extern const mjMap mark_map[];
extern const mjMap dyn_map[];
extern const mjMap gain_map[];
extern const mjMap bias_map[];
extern const mjMap stage_map[];
extern const mjMap datatype_map[];
extern const mjMap meshtype_map[];


//---------------------------------- Base XML class ------------------------------------------------

class mjXBase : public mjXUtil {
 public:
  mjXBase();
  virtual ~mjXBase() = default;

  // parse: implemented in derived parser classes
  virtual void Parse(tinyxml2::XMLElement* root) {};

  // write: implemented in derived writer class
  virtual void Write(FILE* fp) {};

  // set the model allocated externally
  void SetModel(mjCModel*);

  // read alternative orientation specification
  static void ReadAlternative(tinyxml2::XMLElement* elem, mjCAlternative& alt);

 protected:
  mjCModel* model;                    // internally-allocated mjCModel object
};

#endif  // MUJOCO_SRC_XML_XML_BASE_H_
