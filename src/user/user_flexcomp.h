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

#ifndef MUJOCO_SRC_USER_USER_FLEXCOMP_H_
#define MUJOCO_SRC_USER_USER_FLEXCOMP_H_

#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include "user/user_model.h"
#include "user/user_objects.h"


typedef enum _mjtFcompType {
  mjFCOMPTYPE_GRID = 0,
  mjFCOMPTYPE_BOX,
  mjFCOMPTYPE_CYLINDER,
  mjFCOMPTYPE_ELLIPSOID,
  mjFCOMPTYPE_SQUARE,
  mjFCOMPTYPE_DISC,
  mjFCOMPTYPE_CIRCLE,
  mjFCOMPTYPE_MESH,
  mjFCOMPTYPE_GMSH,
  mjFCOMPTYPE_DIRECT,

  mjNFCOMPTYPES
} mjtFcompType;


typedef enum _mjtDof {
  mjFCOMPDOF_FULL = 0,
  mjFCOMPDOF_RADIAL,
  mjFCOMPDOF_TRILINEAR,

  mjNFCOMPDOFS
} mjtDof;


class mjCFlexcomp {
 public:
  mjCFlexcomp(void);
  bool Make(mjsBody* body, char* error, int error_sz);

  bool MakeGrid(char* error, int error_sz);
  bool MakeBox(char* error, int error_sz);
  bool MakeSquare(char* error, int error_sz);
  bool MakeMesh(mjCModel* model, char* error, int error_sz);
  bool MakeGMSH(mjCModel* model, char* error, int error_sz);
  void LoadGMSH(mjCModel* model, mjResource* resource);
  void LoadGMSH41(char* buffer, int binary, int nodeend, int nodebegin,
                  int elemend, int elembegin);
  void LoadGMSH22(char* buffer, int binary, int nodeend, int nodebegin,
                  int elemend, int elembegin);


  int GridID(int ix, int iy);
  int GridID(int ix, int iy, int iz);
  int BoxID(int ix, int iy, int iz);
  void BoxProject(double* pos, int ix, int iy, int iz);

  // common properties set by user
  std::string name;               // flex name
  mjtFcompType type;              // flexcomp type
  int count[3];                   // grid count in each dimension
  double spacing[3];              // spacing between grid elements
  double scale[3];                // scaling for mesh and direct
  double origin[3];               // origin for generating a 3D mesh from a convex 2D mesh
  double mass;                    // total mass of auto-generated bodies
  double inertiabox;              // size of inertia box for each body
  bool equality;                  // create edge equality constraint
  std::string file;               // mesh/gmsh file name
  mjtDof doftype;                 // dof type, all vertices or trilinear interpolation

  // pin specifications
  std::vector<int> pinid;         // ids of points to pin
  std::vector<int> pinrange;      // range of ids to pin
  std::vector<int> pingrid;       // grid coordinates to pin
  std::vector<int> pingridrange;  // range of grid coordinates to pin

  // all other properties
  mjCDef def;                     // local copy, parsed parameters stored here

  // pose transform relative to parent body
  double pos[3];                  // position
  double quat[4];                 // orientation
  mjsOrientation alt;             // alternative orientation

  // set by user or computed internally
  bool rigid;                     // all vertices are in parent body (all pinned)
  bool centered;                  // all vertex coordinates are (0,0,0) (nothing pinned)
  std::vector<double> point;      // flex bodies/vertices
  std::vector<bool> pinned;       // is point pinned (true: no new body)
  std::vector<bool> used;         // is point used by any element (false: skip)
  std::vector<int> element;       // flex elements
  std::vector<float> texcoord;    // vertex texture coordinates
  std::vector<int> elemtexcoord;  // face texture coordinates (OBJ only)

  // plugin support
  std::string plugin_name;
  std::string plugin_instance_name;
  mjsPlugin plugin;
};

#endif  // MUJOCO_SRC_USER_USER_FLEXCOMP_H_
