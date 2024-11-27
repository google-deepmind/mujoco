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

#ifndef MUJOCO_SRC_USER_USER_COMPOSITE_H_
#define MUJOCO_SRC_USER_USER_COMPOSITE_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include "user/user_model.h"
#include "user/user_objects.h"

typedef enum _mjtCompType {
  mjCOMPTYPE_PARTICLE = 0,
  mjCOMPTYPE_GRID,
  mjCOMPTYPE_CABLE,
  mjCOMPTYPE_ROPE,
  mjCOMPTYPE_LOOP,
  mjCOMPTYPE_CLOTH,

  mjNCOMPTYPES
} mjtCompType;


typedef enum _mjtCompKind {
  mjCOMPKIND_JOINT = 0,
  mjCOMPKIND_TWIST,
  mjCOMPKIND_STRETCH,
  mjCOMPKIND_TENDON,
  mjCOMPKIND_SHEAR,
  mjCOMPKIND_PARTICLE,

  mjNCOMPKINDS
} mjtCompKind;


typedef enum _mjtCompShape {
  mjCOMPSHAPE_LINE = 0,
  mjCOMPSHAPE_COS,
  mjCOMPSHAPE_SIN,
  mjCOMPSHAPE_ZERO,

  mjNCOMPSHAPES
} mjtCompShape;


class mjCComposite {
 public:
  mjCComposite(void);

  void SetDefault(void);
  bool AddDefaultJoint(char* error = NULL, int error_sz = 0);
  void AdjustSoft(mjtNum* solref, mjtNum* solimp, int level);

  bool Make(mjSpec* spec, mjsBody* body, char* error, int error_sz);

  bool MakeParticle(mjCModel* model, mjsBody* body, char* error, int error_sz);
  bool MakeGrid(mjCModel* model, mjsBody* body, char* error, int error_sz);
  bool MakeCable(mjCModel* model, mjsBody* body, char* error, int error_sz);
  void MakeShear(mjCModel* model);

  void MakeSkin2(mjCModel* model, mjtNum inflate);
  void MakeSkin2Subgrid(mjCModel* model, mjtNum inflate);
  void MakeClothBones(mjCModel* model, mjsSkin* skin);
  void MakeClothBonesSubgrid(mjCModel* model, mjsSkin* skin);
  void MakeCableBones(mjCModel* model, mjsSkin* skin);
  void MakeCableBonesSubgrid(mjCModel* model, mjsSkin* skin);

  // common properties
  std::string prefix;             // name prefix
  mjtCompType type;               // composite type
  int count[3];                   // geom count in each dimension
  double spacing;                 // spacing between elements
  double offset[3];               // position offset for particle and grid
  std::vector<int> pin;           // pin elements of grid (do not create main joint)
  double flatinertia;             // flatten ineria of cloth elements; 0: disable
  mjtNum solrefsmooth[mjNREF];    // solref for smoothing equality
  mjtNum solimpsmooth[mjNIMP];    // solimp for smoothing equality

  // currently used only for cable
  std::string initial;            // root boundary type
  std::vector<float> uservert;    // user-specified vertex positions
  double size[3];                 // rope size (meaning depends on the shape)
  mjtCompShape curve[3];          // geometric shape

  // body names used in the skin
  std::vector<std::string> username;

  // plugin support
  std::string plugin_name;
  std::string plugin_instance_name;
  mjsPlugin plugin;

  // skin
  bool skin;                      // generate skin
  bool skintexcoord;              // generate texture coordinates
  std::string skinmaterial;       // skin material
  float skinrgba[4];              // skin rgba
  float skininflate;              // inflate skin
  int skinsubgrid;                // number of skin subgrid points; 0: none (2D only)
  int skingroup;                  // skin group of the composite object

  // element options
  bool add[mjNCOMPKINDS];                                          // add element
  mjCDef def[mjNCOMPKINDS];                                        // default geom, site, tendon
  std::unordered_map<mjtCompKind, std::vector<mjCDef> > defjoint;  // default joints

  // computed internally
  int dim;                        // dimensionality

 private:
  mjsBody* AddCableBody(mjCModel* model, mjsBody* body, int ix, double normal[3], double prev_quat[4]);

  // temporary skin vectors
  void CopyIntoSkin(mjsSkin* skin);
  std::vector<int> face;
  std::vector<float> vert;
  std::vector<float> bindpos;
  std::vector<float> bindquat;
  std::vector<float> texcoord;
  std::vector<std::vector<int>> vertid;
  std::vector<std::vector<float>> vertweight;
};

#endif  // MUJOCO_SRC_USER_USER_COMPOSITE_H_
