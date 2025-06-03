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

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjplugin.h>
#include "cc/array_safety.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_util_errmem.h"
#include "user/user_flexcomp.h"
#include <mujoco/mjspec.h>
#include "user/user_api.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_resource.h"
#include "user/user_util.h"

namespace {
namespace mju = ::mujoco::util;
using std::vector;
using std::stringstream;
}  // namespace

// strncpy with 0, return false
static bool comperr(char* error, const char* msg, int error_sz) {
  mju_strncpy(error, msg, error_sz);
  return false;
}

// Read data of type T from a potentially unaligned buffer pointer.
template <typename T>
static void ReadFromBuffer(T* dst, const char* src) {
  std::memcpy(dst, src, sizeof(T));
}


static void ReadStrFromBuffer(char* dest, const char* src, int maxlen) {
  std::strncpy(dest, src, maxlen);
}

bool IsValidElementOrNodeHeader22(const std::string& line) {
  // making sure characters are numbers
  for (char c : line) {
    if (!std::isdigit(c)) {
      return false;
    }
  }
  return true;
}


// constructor: set defaults outside mjCDef
mjCFlexcomp::mjCFlexcomp(void) {
  type = mjFCOMPTYPE_GRID;
  count[0] = count[1] = count[2] = 10;
  mjuu_setvec(spacing, 0.02, 0.02, 0.02);
  mjuu_setvec(scale, 1, 1, 1);
  mass = 1;
  inertiabox = 0.005;
  equality = false;
  mjuu_setvec(pos, 0, 0, 0);
  mjuu_setvec(quat, 1, 0, 0, 0);
  rigid = false;
  centered = false;
  doftype = mjFCOMPDOF_FULL;

  mjs_defaultPlugin(&plugin);
  mjs_defaultOrientation(&alt);
  plugin_name = "";
  plugin_instance_name = "";
  plugin.plugin_name = (mjString*)&plugin_name;
  plugin.name = (mjString*)&plugin_instance_name;
}



// make flexcomp object
bool mjCFlexcomp::Make(mjsBody* body, char* error, int error_sz) {
  mjCModel* model = static_cast<mjCBody*>(body->element)->model;
  mjsCompiler* compiler = static_cast<mjCBody*>(body->element)->compiler;
  mjsFlex* dflex = def.spec.flex;
  bool direct = (type == mjFCOMPTYPE_DIRECT ||
                 type == mjFCOMPTYPE_MESH ||
                 type == mjFCOMPTYPE_GMSH);

  // check parent body name
  if (std::string(mjs_getString(body->name)).empty()) {
    return comperr(error, "Parent body must have name", error_sz);
  }

  // check dim
  if (dflex->dim < 1 || dflex->dim > 3) {
    return comperr(error, "Invalid dim, must be between 1 and 3", error_sz);
  }

  // check counts
  for (int i=0; i < 3; i++) {
    if (count[i] < 1 || ((doftype == mjFCOMPDOF_RADIAL && count[i] < 2) && dflex->dim == 3)) {
      return comperr(error, "Count too small", error_sz);
    }
  }

  // check spacing
  double minspace = 2*dflex->radius + dflex->margin;
  if (!direct) {
    if (spacing[0] < minspace ||
        spacing[1] < minspace ||
        spacing[2] < minspace) {
      return comperr(error, "Spacing must be larger than geometry size", error_sz);
    }
  }

  // check scale
  if (scale[0] < mjMINVAL || scale[1] < mjMINVAL || scale[2] < mjMINVAL) {
    return comperr(error, "Scale must be larger than mjMINVAL", error_sz);
  }

  // check mass and inertia
  if (mass < mjMINVAL || inertiabox < mjMINVAL) {
    return comperr(error, "Mass and inertiabox must be larger than mjMINVAL", error_sz);
  }

  // compute orientation
  const char* alterr = mjs_resolveOrientation(quat, compiler->degree, compiler->eulerseq, &alt);
  if (alterr) {
    return comperr(error, alterr, error_sz);
  }

  // type-specific constructor: populate point and element, possibly set dim
  bool res;
  switch (type) {
    case mjFCOMPTYPE_GRID:
    case mjFCOMPTYPE_CIRCLE:
      res = MakeGrid(error, error_sz);
      break;

    case mjFCOMPTYPE_BOX:
    case mjFCOMPTYPE_CYLINDER:
    case mjFCOMPTYPE_ELLIPSOID:
      res = MakeBox(error, error_sz);
      break;

    case mjFCOMPTYPE_SQUARE:
    case mjFCOMPTYPE_DISC:
      res = MakeSquare(error, error_sz);
      break;

    case mjFCOMPTYPE_MESH:
      res = MakeMesh(model, error, error_sz);
      break;

    case mjFCOMPTYPE_GMSH:
      res = MakeGMSH(model, error, error_sz);
      break;

    case mjFCOMPTYPE_DIRECT:
      res = true;
      break;

    default:
      return comperr(error, "Unknown flexcomp type", error_sz);
  }
  if (!res) {
    return false;
  }

  // force flatskin shading for box, cylinder and 3D grid
  if (type == mjFCOMPTYPE_BOX || type == mjFCOMPTYPE_CYLINDER ||
      (type == mjFCOMPTYPE_GRID && dflex->dim == 3)) {
    dflex->flatskin = true;
  }

  // check pin sizes
  if (pinrange.size()%2) {
    return comperr(error, "Pin range number must be multiple of 2", error_sz);
  }
  if (pingrid.size()%dflex->dim) {
    return comperr(error, "Pin grid number must be multiple of dim", error_sz);
  }
  if (pingridrange.size()%(2*dflex->dim)) {
    return comperr(error, "Pin grid range number of must be multiple of 2*dim", error_sz);
  }
  if (type != mjFCOMPTYPE_GRID && !(pingrid.empty() && pingridrange.empty())) {
    return comperr(error, "Pin grid(range) can only be used with grid type", error_sz);
  }
  if (dflex->dim == 1 && !(pingrid.empty() && pingridrange.empty())) {
    return comperr(error, "Pin grid(range) cannot be used with dim=1", error_sz);
  }

  // require element and point
  if (point.empty() || element.empty()) {
    return comperr(error, "Point and element required", error_sz);
  }

  // check point size
  if (point.size()%3) {
    return comperr(error, "Point size must be a multiple of 3", error_sz);
  }

  // check element size
  if (element.size()%(dflex->dim+1)) {
    return comperr(error, "Element size must be a multiple of dim+1", error_sz);
  }

  // get number of points
  int npnt = point.size()/3;

  // check elem vertex ids
  for (int i=0; i < (int)element.size(); i++) {
    if (element[i] < 0 || element[i] >= npnt) {
      char msg[100];
      snprintf(msg, sizeof(msg), "element %d has point id %d, number of points is %d", i,
               element[i], npnt);
      return comperr(error, msg, error_sz);
    }
  }

  // apply scaling for direct types
  if (direct && (scale[0] != 1 || scale[1] != 1 || scale[2] != 1)) {
    for (int i=0; i < npnt; i++) {
      point[3*i] *= scale[0];
      point[3*i+1] *= scale[1];
      point[3*i+2] *= scale[2];
    }
  }

  // apply pose transform to points
  for (int i=0; i < npnt; i++) {
    double newp[3], oldp[3] = {point[3*i], point[3*i+1], point[3*i+2]};
    mjuu_trnVecPose(newp, pos, quat, oldp);
    point[3*i] = newp[0];
    point[3*i+1] = newp[1];
    point[3*i+2] = newp[2];
  }

  // compute bounding box of points
  double minmax[6] = {mjMAXVAL, mjMAXVAL, mjMAXVAL, -mjMAXVAL, -mjMAXVAL, -mjMAXVAL};
  for (int i=0; i < npnt; i++) {
    for (int j=0; j < 3; j++) {
      minmax[j+0] = std::min(minmax[j+0], point[3*i+j]);
      minmax[j+3] = std::max(minmax[j+3], point[3*i+j]);
    }
  }

  // construct pinned array
  pinned = vector<bool>(npnt, rigid);

  // handle pins if user did not specify rigid
  if (!rigid) {
    // process pinid
    for (int i=0; i < (int)pinid.size(); i++) {
      // check range
      if (pinid[i] < 0 || pinid[i] >= npnt) {
        return comperr(error, "pinid out of range", error_sz);
      }

      // set
      pinned[pinid[i]] = true;
    }

    // process pinrange
    for (int i=0; i < (int)pinrange.size(); i+=2) {
      // check range
      if (pinrange[i] < 0 || pinrange[i] >= npnt ||
          pinrange[i+1] < 0 || pinrange[i+1] >= npnt) {
        return comperr(error, "pinrange out of range", error_sz);
      }

      // set
      for (int k=pinrange[i]; k <= pinrange[i+1]; k++) {
        pinned[k] = true;
      }
    }

    // process pingrid
    for (int i=0; i < (int)pingrid.size(); i+=dflex->dim) {
      // check range
      for (int k=0; k < dflex->dim; k++) {
        if (pingrid[i+k] < 0 || pingrid[i+k] >= count[k]) {
          return comperr(error, "pingrid out of range", error_sz);
        }
      }

      // set
      if (dflex->dim == 2) {
        pinned[GridID(pingrid[i], pingrid[i+1])] = true;
      }
      else if (dflex->dim == 3) {
        pinned[GridID(pingrid[i], pingrid[i+1], pingrid[i+2])] = true;
      }
    }

    // process pingridrange
    for (int i=0; i < (int)pingridrange.size(); i+=2*dflex->dim) {
      // check range
      for (int k=0; k < 2*dflex->dim; k++) {
        if (pingridrange[i+k] < 0 || pingridrange[i+k] >= count[k%dflex->dim]) {
          return comperr(error, "pingridrange out of range", error_sz);
        }
      }

      // set
      if (dflex->dim == 2) {
        for (int ix=pingridrange[i]; ix <= pingridrange[i+2]; ix++) {
          for (int iy=pingridrange[i+1]; iy <= pingridrange[i+3]; iy++) {
            pinned[GridID(ix, iy)] = true;
          }
        }
      }
      else if (dflex->dim == 3) {
        for (int ix=pingridrange[i]; ix <= pingridrange[i+3]; ix++) {
          for (int iy=pingridrange[i+1]; iy <= pingridrange[i+4]; iy++) {
            for (int iz=pingridrange[i+2]; iz <= pingridrange[i+5]; iz++) {
              pinned[GridID(ix, iy, iz)] = true;
            }
          }
        }
      }
    }

    // center of radial body is always pinned
    if (doftype == mjFCOMPDOF_RADIAL) {
      pinned[0] = true;
    }

    // check if all or none are pinned
    bool allpin = true, nopin = true;
    for (int i=0; i < npnt; i++) {
      if (pinned[i]) {
        nopin = false;
      }
      else {
        allpin = false;
      }
    }

    // adjust rigid and centered
    if (allpin) {
      rigid = true;
    }
    else if (nopin) {
      centered = true;
    }
  }

  // remove unreferenced for direct, mesh, gmsh
  if (direct) {
    // find used
    used = std::vector<bool> (npnt, false);
    for (int i=0; i < (int)element.size(); i++) {
      used[element[i]] = true;
    }

    // construct reindex
    bool hasunused = false;
    std::vector<int> reindex (npnt, 0);
    for (int i=0; i < npnt; i++) {
      if (!used[i]) {
        hasunused = true;
        for (int k=i+1; k < npnt; k++) {
          reindex[k]--;
        }
      }
    }

    // reindex elements if unused present
    if (hasunused) {
      for (int i=0; i < (int)element.size(); i++) {
        element[i] += reindex[element[i]];
      }
    }
  }

  // nothing to remove for auto-generated types
  else {
    used = std::vector<bool> (npnt, true);
  }

  // create flex, copy parameters
  mjCFlex* flex = model->AddFlex();
  mjsFlex* pf = &flex->spec;
  int id = flex->id;

  *flex = def.Flex();
  flex->PointToLocal();

  flex->model = model;
  flex->id = id;
  mjs_setString(pf->name, name.c_str());
  mjs_setInt(pf->elem, element.data(), element.size());
  mjs_setFloat(pf->texcoord, texcoord.data(), texcoord.size());
  mjs_setInt(pf->elemtexcoord, elemtexcoord.data(), elemtexcoord.size());
  if (!centered) {
    mjs_setDouble(pf->vert, point.data(), point.size());
  }

  // rigid: set parent name, nothing else to do
  if (rigid) {
    mjs_appendString(pf->vertbody, mjs_getString(body->name));
    return true;
  }

  // compute body mass and inertia matching specs
  double bodymass = mass/npnt;
  double bodyinertia = bodymass*(2.0*inertiabox*inertiabox)/3.0;

  // overwrite plugin name
  if (plugin.active && plugin_instance_name.empty()) {
    plugin_instance_name = "flexcomp_" + name;
    static_cast<mjCPlugin*>(plugin.element)->name = plugin_instance_name;
  }

  // create bodies, construct flex vert and vertbody
  for (int i=0; i < npnt; i++) {
    // not used: skip
    if (!used[i]) {
      continue;
    }

    // pinned or trilinear: parent body
    if (pinned[i] || doftype == mjFCOMPDOF_TRILINEAR) {
      mjs_appendString(pf->vertbody, mjs_getString(body->name));

      // add plugin
      if (plugin.active) {
        mjsPlugin* pplugin = &body->plugin;
        pplugin->active = true;
        pplugin->element = static_cast<mjsElement*>(plugin.element);
        mjs_setString(pplugin->plugin_name, mjs_getString(plugin.plugin_name));
        mjs_setString(pplugin->name, plugin_instance_name.c_str());
      }
    }

    // not pinned and not trilinear: new body
    else {
      // add new body at vertex coordinates
      mjsBody* pb = mjs_addBody(body, 0);

      // add geom if vertcollide
      if (dflex->vertcollide) {
        mjsGeom* geom = mjs_addGeom(pb, 0);
        geom->type = mjGEOM_SPHERE;
        geom->size[0] = dflex->radius;
        geom->group = 4;
      }

      // set frame and inertial
      pb->pos[0] = point[3*i];
      pb->pos[1] = point[3*i+1];
      pb->pos[2] = point[3*i+2];
      mjuu_zerovec(pb->ipos, 3);
      pb->mass = bodymass;
      pb->inertia[0] = bodyinertia;
      pb->inertia[1] = bodyinertia;
      pb->inertia[2] = bodyinertia;
      pb->explicitinertial = true;

      // add radial slider
      if (doftype == mjFCOMPDOF_RADIAL) {
        mjsJoint* jnt = mjs_addJoint(pb, 0);

        // set properties
        jnt->type = mjJNT_SLIDE;
        mjuu_setvec(jnt->pos, 0, 0, 0);
        mjuu_copyvec(jnt->axis, pb->pos, 3);
        mjuu_normvec(jnt->axis, 3);
      }

      // add three orthogonal sliders
      else if (doftype == mjFCOMPDOF_FULL) {
        for (int j=0; j < 3; j++) {
          // add joint to body
          mjsJoint* jnt = mjs_addJoint(pb, 0);

          // set properties
          jnt->type = mjJNT_SLIDE;
          mjuu_setvec(jnt->pos, 0, 0, 0);
          mjuu_setvec(jnt->axis, 0, 0, 0);
          jnt->axis[j] = 1;
        }
      }

      // construct body name, add to vertbody
      char txt[100];
      mju::sprintf_arr(txt, "%s_%d", name.c_str(), i);
      mjs_setString(pb->name, txt);
      mjs_appendString(pf->vertbody, mjs_getString(pb->name));

      // clear flex vertex coordinates if allocated
      if (!centered) {
        point[3*i] = 0;
        point[3*i+1] = 0;
        point[3*i+2] = 0;
      }

      // add plugin
      if (plugin.active) {
        mjsPlugin* pplugin = &pb->plugin;
        pplugin->active = true;
        pplugin->element = static_cast<mjsElement*>(plugin.element);
        mjs_setString(pplugin->plugin_name, mjs_getString(plugin.plugin_name));
        mjs_setString(pplugin->name, plugin_instance_name.c_str());
      }
    }
  }

  // create nodal mesh for trilinear interpolation
  if (doftype == mjFCOMPDOF_TRILINEAR) {
    std::vector<double> node(24, 0);
    for (int i=0; i < 2; i++) {
      for (int j=0; j < 2; j++) {
        for (int k=0; k < 2; k++) {
          if (pinned[i*4+j*2+k]) {
            node[3*(i*4+j*2+k)+0] = i == 0 ? minmax[0] : minmax[3];
            node[3*(i*4+j*2+k)+1] = j == 0 ? minmax[1] : minmax[4];
            node[3*(i*4+j*2+k)+2] = k == 0 ? minmax[2] : minmax[5];
            mjs_appendString(pf->nodebody, mjs_getString(body->name));
            continue;
          }

          mjsBody* pb = mjs_addBody(body, 0);
          pb->pos[0] = i == 0 ? minmax[0] : minmax[3];
          pb->pos[1] = j == 0 ? minmax[1] : minmax[4];
          pb->pos[2] = k == 0 ? minmax[2] : minmax[5];
          mjuu_zerovec(pb->ipos, 3);
          pb->mass = mass / 8;
          pb->inertia[0] = pb->mass*(2.0*inertiabox*inertiabox)/3.0;
          pb->inertia[1] = pb->mass*(2.0*inertiabox*inertiabox)/3.0;
          pb->inertia[2] = pb->mass*(2.0*inertiabox*inertiabox)/3.0;
          pb->explicitinertial = true;

          // add geom if vertcollide
          if (dflex->vertcollide) {
            mjsGeom* geom = mjs_addGeom(pb, 0);
            geom->type = mjGEOM_SPHERE;
            geom->size[0] = dflex->radius;
          }

          for (int d=0; d < 3; d++) {
            mjsJoint* jnt = mjs_addJoint(pb, 0);
            jnt->type = mjJNT_SLIDE;
            mjuu_setvec(jnt->pos, 0, 0, 0);
            mjuu_setvec(jnt->axis, 0, 0, 0);
            jnt->axis[d] = 1;
          }

          // construct node name, add to nodebody
          char txt[100];
          mju::sprintf_arr(txt, "%s_%d_%d_%d", name.c_str(), i, j, k);
          mjs_setString(pb->name, txt);
          mjs_appendString(pf->nodebody, mjs_getString(pb->name));
        }
      }
    }

    if (!centered) {
      mjs_setDouble(pf->node, node.data(), node.size());
    }
  }

  if (!centered || doftype == mjFCOMPDOF_TRILINEAR) {
    mjs_setDouble(pf->vert, point.data(), point.size());
  }

  // create edge equality constraint
  if (equality) {
    mjsEquality* pe = mjs_addEquality(&model->spec, &def.spec);
    mjs_setDefault(pe->element, &model->Default()->spec);
    pe->type = mjEQ_FLEX;
    pe->active = true;
    mjs_setString(pe->name1, name.c_str());
  }

  return true;
}



// get point id from grid coordinates
int mjCFlexcomp::GridID(int ix, int iy) {
  return ix*count[1] + iy;
}
int mjCFlexcomp::GridID(int ix, int iy, int iz) {
  return ix*count[1]*count[2] + iy*count[2] + iz;
}



// make grid
bool mjCFlexcomp::MakeGrid(char* error, int error_sz) {
  int dim = def.Flex().spec.dim;
  bool needtex = texcoord.empty() && mjs_getString(def.spec.flex->material)[0];

  // 1D
  if (dim == 1) {
    for (int ix=0; ix < count[0]; ix++) {
      if (type == mjFCOMPTYPE_CIRCLE) {
        if (ix >= count[0]-1) {
          continue;
        }

        // add point
        double theta = 2*mjPI/(count[0]-1);
        double radius = spacing[0]/std::sin(theta/2)/2;
        point.push_back(radius*std::cos(theta*ix));
        point.push_back(radius*std::sin(theta*ix));
        point.push_back(0);

        // add element
        element.push_back(ix);
        element.push_back(ix == count[0]-2 ? 0 : ix+1);
      } else {
        // add point
        point.push_back(spacing[0]*(ix - 0.5*(count[0]-1)));
        point.push_back(0);
        point.push_back(0);

        // add element
        if (ix < count[0]-1) {
          element.push_back(ix);
          element.push_back(ix+1);
        }
      }
    }
  }

  // 2D
  else if (dim == 2) {
    for (int ix=0; ix < count[0]; ix++) {
      for (int iy=0; iy < count[1]; iy++) {
        int quad2tri[2][3] = {{0, 1, 2}, {0, 2, 3}};

        // add point
        double pos[2] = {spacing[0]*(ix - 0.5*(count[0]-1)),
                         spacing[1]*(iy - 0.5*(count[1]-1))};
        point.push_back(pos[0]);
        point.push_back(pos[1]);
        point.push_back(0);

        // add texture coordinates, if not specified explicitly
        if (needtex) {
          texcoord.push_back(ix/(double)std::max(count[0]-1, 1));
          texcoord.push_back(iy/(double)std::max(count[1]-1, 1));
        }

        // flip triangles if radial projection is requested
        if (((pos[0] < -mjEPS && pos[1] > -mjEPS) ||
             (pos[0] > -mjEPS && pos[1] < -mjEPS)) &&
            type == mjFCOMPTYPE_DISC) {
          quad2tri[0][2] = 3;
          quad2tri[1][0] = 1;
        }

        // add elements
        if (ix < count[0]-1 && iy < count[1]-1) {
          int vert[4] = {
            count[2]*count[1]*(ix+0) + count[2]*(iy+0),
            count[2]*count[1]*(ix+1) + count[2]*(iy+0),
            count[2]*count[1]*(ix+1) + count[2]*(iy+1),
            count[2]*count[1]*(ix+0) + count[2]*(iy+1),
          };
          for (int s =0; s < 2; s++) {
            for (int v=0; v < 3; v++) {
              element.push_back(vert[quad2tri[s][v]]);
            }
          }
        }
      }
    }
  }

  // 3D
  else {
    int cube2tets[6][4] = {{0, 3, 1, 7}, {0, 1, 4, 7},
                           {1, 3, 2, 7}, {1, 2, 6, 7},
                           {1, 5, 4, 7}, {1, 6, 5, 7}};
    for (int ix=0; ix < count[0]; ix++) {
      for (int iy=0; iy < count[1]; iy++) {
        for (int iz=0; iz < count[2]; iz++) {
          // add point
          point.push_back(spacing[0]*(ix - 0.5*(count[0]-1)));
          point.push_back(spacing[1]*(iy - 0.5*(count[1]-1)));
          point.push_back(spacing[2]*(iz - 0.5*(count[2]-1)));

          // add texture coordinates, if not specified explicitly
          if (needtex) {
            texcoord.push_back(ix/(float)std::max(count[0]-1, 1));
            texcoord.push_back(iy/(float)std::max(count[1]-1, 1));
          }

          // add elements
          if (ix < count[0]-1 && iy < count[1]-1 && iz < count[2]-1) {
            int vert[8] = {
              count[2]*count[1]*(ix+0) + count[2]*(iy+0) + iz+0,
              count[2]*count[1]*(ix+1) + count[2]*(iy+0) + iz+0,
              count[2]*count[1]*(ix+1) + count[2]*(iy+1) + iz+0,
              count[2]*count[1]*(ix+0) + count[2]*(iy+1) + iz+0,
              count[2]*count[1]*(ix+0) + count[2]*(iy+0) + iz+1,
              count[2]*count[1]*(ix+1) + count[2]*(iy+0) + iz+1,
              count[2]*count[1]*(ix+1) + count[2]*(iy+1) + iz+1,
              count[2]*count[1]*(ix+0) + count[2]*(iy+1) + iz+1,
            };
            for (int s=0; s < 6; s++) {
              for (int v=0; v < 4; v++) {
                element.push_back(vert[cube2tets[s][v]]);
              }
            }
          }
        }
      }
    }
  }

  // check elements
  if (element.empty()) {
    return comperr(error, "No elements were created in grid", error_sz);
  }

  return true;
}



// get point id from box coordinates and side
int mjCFlexcomp::BoxID(int ix, int iy, int iz) {
  // side iz=0
  if (iz == 0) {
    return ix*count[1] + iy + 1;
  }

  // side iz=max
  else if (iz == count[2]-1) {
    return count[0]*count[1] + ix*count[1] + iy + 1;
  }

  // side iy=0
  else if (iy == 0) {
    return 2*count[0]*count[1] + ix*(count[2]-2) + iz - 1 + 1;
  }

  // side iy=max
  else if (iy == count[1]-1) {
    return 2*count[0]*count[1] + count[0]*(count[2]-2) + ix*(count[2]-2) + iz - 1 + 1;
  }

  // side ix=0
  else if (ix == 0) {
    return 2*count[0]*count[1] + 2*count[0]*(count[2]-2) + (iy-1)*(count[2]-2) + iz - 1 + 1;
  }

  // side ix=max
  else {
    return 2*count[0]*count[1] + 2*count[0]*(count[2]-2) + (count[1]-2)*(count[2]-2) +
           (iy-1)*(count[2]-2) + iz - 1 + 1;
  }
}



// project from box to other shape
void mjCFlexcomp::BoxProject(double* pos, int ix, int iy, int iz) {
  // init point
  pos[0] = 2.0*ix/(count[0]-1) - 1;
  pos[1] = 2.0*iy/(count[1]-1) - 1;
  pos[2] = 2.0*iz/(count[2]-1) - 1;

  // determine sizes
  double size[3] = {
    0.5*spacing[0]*(count[0]-1),
    0.5*spacing[1]*(count[1]-1),
    0.5*spacing[2]*(count[2]-1)
  };

  // box
  if (type == mjFCOMPTYPE_BOX) {
    pos[0] *= size[0];
    pos[1] *= size[1];
    pos[2] *= size[2];
  }

  // cylinder
  else if (type == mjFCOMPTYPE_CYLINDER) {
    double L0 = std::max(std::abs(pos[0]), std::abs(pos[1]));
    mjuu_normvec(pos, 2);
    pos[0] *= size[0]*L0;
    pos[1] *= size[1]*L0;
    pos[2] *= size[2];
  }

  // ellipsoid
  else if (type == mjFCOMPTYPE_ELLIPSOID) {
    mjuu_normvec(pos, 3);
    pos[0] *= size[0];
    pos[1] *= size[1];
    pos[2] *= size[2];
  }
}



// make 2d square or disc
bool mjCFlexcomp::MakeSquare(char* error, int error_sz) {
  // set 2D
  def.spec.flex->dim = 2;

  // create square
  if (!MakeGrid(error, error_sz)) {
    return false;
  }

  // do projection
  if (type == mjFCOMPTYPE_DISC) {
    double size[2] = {
      0.5*spacing[0]*(count[0]-1),
      0.5*spacing[1]*(count[1]-1),
    };

    for (int i=0; i < point.size()/3; i++) {
      double* pos = point.data() + i*3;
      double L0 = std::max(std::abs(pos[0]), std::abs(pos[1]));
      mjuu_normvec(pos, 2);
      pos[0] *= size[0]*L0;
      pos[1] *= size[1]*L0;
    }
  }

  return true;
}



// make 3d box, ellipsoid or cylinder
bool mjCFlexcomp::MakeBox(char* error, int error_sz) {
  double pos[3];
  bool needtex = texcoord.empty() && mjs_getString(def.spec.flex->material)[0];

  // set 3D
  def.spec.flex->dim = 3;

  // add center point
  point.push_back(0);
  point.push_back(0);
  point.push_back(0);

  // add texture coordinates, if not specified explicitly
  if (needtex) {
    texcoord.push_back(0);
    texcoord.push_back(0);
  }

  // iz=0/max
  for (int iz=0; iz < count[2]; iz+=count[2]-1) {
    for (int ix=0; ix < count[0]; ix++) {
      for (int iy=0; iy < count[1]; iy++) {
        // add point
        BoxProject(pos, ix, iy, iz);
        point.push_back(pos[0]);
        point.push_back(pos[1]);
        point.push_back(pos[2]);

        // add texture coordinates, if not specified explicitly
        if (needtex) {
          texcoord.push_back(ix/(float)std::max(count[0]-1, 1));
          texcoord.push_back(iy/(float)std::max(count[1]-1, 1));
        }

        // add elements
        if (ix < count[0]-1 && iy < count[1]-1) {
          element.push_back(0);
          element.push_back(BoxID(ix, iy, iz));
          element.push_back(BoxID(ix+1, iy, iz));
          element.push_back(BoxID(ix+1, iy+1, iz));

          element.push_back(0);
          element.push_back(BoxID(ix, iy, iz));
          element.push_back(BoxID(ix, iy+1, iz));
          element.push_back(BoxID(ix+1, iy+1, iz));
        }
      }
    }
  }

  // iy=0/max
  for (int iy=0; iy < count[1]; iy+=count[1]-1) {
    for (int ix=0; ix < count[0]; ix++) {
      for (int iz=0; iz < count[2]; iz++) {
        // add point
        if (iz > 0 && iz < count[2]-1) {
          BoxProject(pos, ix, iy, iz);
          point.push_back(pos[0]);
          point.push_back(pos[1]);
          point.push_back(pos[2]);

          // add texture coordinates
          if (needtex) {
            texcoord.push_back(ix/(float)std::max(count[0]-1, 1));
            texcoord.push_back(iz/(float)std::max(count[2]-1, 1));
          }
        }

        // add elements
        if (ix < count[0]-1 && iz < count[2]-1) {
          element.push_back(0);
          element.push_back(BoxID(ix, iy, iz));
          element.push_back(BoxID(ix+1, iy, iz));
          element.push_back(BoxID(ix+1, iy, iz+1));

          element.push_back(0);
          element.push_back(BoxID(ix, iy, iz));
          element.push_back(BoxID(ix, iy, iz+1));
          element.push_back(BoxID(ix+1, iy, iz+1));
        }
      }
    }
  }

  // ix=0/max
  for (int ix=0; ix < count[0]; ix+=count[0]-1) {
    for (int iy=0; iy < count[1]; iy++) {
      for (int iz=0; iz < count[2]; iz++) {
        // add point
        if (iz > 0 && iz < count[2]-1 && iy > 0 && iy < count[1]-1) {
          BoxProject(pos, ix, iy, iz);
          point.push_back(pos[0]);
          point.push_back(pos[1]);
          point.push_back(pos[2]);

          // add texture coordinates
          if (needtex) {
            texcoord.push_back(iy/(float)std::max(count[1]-1, 1));
            texcoord.push_back(iz/(float)std::max(count[2]-1, 1));
          }
        }

        // add elements
        if (iy < count[1]-1 && iz < count[2]-1) {
          element.push_back(0);
          element.push_back(BoxID(ix, iy, iz));
          element.push_back(BoxID(ix, iy+1, iz));
          element.push_back(BoxID(ix, iy+1, iz+1));

          element.push_back(0);
          element.push_back(BoxID(ix, iy, iz));
          element.push_back(BoxID(ix, iy, iz+1));
          element.push_back(BoxID(ix, iy+1, iz+1));
        }
      }
    }
  }

  return true;
}



// copied from user_mesh.cc
template <typename T> static T* VecToArray(std::vector<T>& vector, bool clear = true){
  if (vector.empty())
    return nullptr;
  else {
    int n = (int)vector.size();
    T* cvec = (T*) mju_malloc(n*sizeof(T));
    memcpy(cvec, vector.data(), n*sizeof(T));
    if (clear) {
      vector.clear();
    }
    return cvec;
  }
}



// make mesh
bool mjCFlexcomp::MakeMesh(mjCModel* model, char* error, int error_sz) {
  // strip path
  if (!file.empty() && model->spec.strippath) {
    file = mjuu_strippath(file);
  }

  // file is required
  if (file.empty()) {
    return comperr(error, "File is required", error_sz);
  }

  // check dim
  if (def.spec.flex->dim < 2) {
    return comperr(error, "Flex dim must be at least 2 for mesh", error_sz);
  }

  // load resource
  std::string filename = mjuu_combinePaths(mjs_getString(model->spec.meshdir), file);
  mjResource* resource = nullptr;


  if (mjCMesh::IsMSH(filename)) {
    return comperr(error, "legacy MSH files are not supported in flexcomp", error_sz);
  }

  try {
    resource = mjCBase::LoadResource(mjs_getString(model->spec.modelfiledir),
                                     filename, 0);
  } catch (mjCError err) {
    return comperr(error, err.message, error_sz);
  }


  // load mesh
  mjCMesh mesh;
  try {
    mesh.LoadFromResource(resource, true);
    mju_closeResource(resource);
  } catch (mjCError err) {
    mju_closeResource(resource);
    return comperr(error, err.message, error_sz);
  }

  // check sizes
  if (mesh.Vert().empty() || mesh.Face().empty()) {
    return comperr(error, "Vertex and face data required", error_sz);
  }

  // copy vertices
  point = mesh.Vert();

  if (mesh.HasTexcoord()) {
    texcoord = mesh.Texcoord();
    elemtexcoord = mesh.FaceTexcoord();
  }

  // copy faces or create 3D mesh
  if (def.spec.flex->dim == 2) {
    element = mesh.Face();
  } else {
    point.insert(point.begin() + 0, origin[0]);
    point.insert(point.begin() + 1, origin[1]);
    point.insert(point.begin() + 2, origin[2]);
    for (int i=0; i < mesh.Face().size(); i+=3) {
      // only add tetrahedra with positive volume
      int tet[3] = {mesh.Face()[i+0]+1,
                    mesh.Face()[i+1]+1,
                    mesh.Face()[i+2]+1};
      double edge1[3], edge2[3], edge3[3];
      for (int i=0; i < 3; i++) {
        edge1[i] = point[3*tet[0]+i] - origin[i];
        edge2[i] = point[3*tet[1]+i] - origin[i];
        edge3[i] = point[3*tet[2]+i] - origin[i];
      }
      double normal[3];
      mjuu_crossvec(normal, edge1, edge2);
      if (mjuu_dot3(normal, edge3) < mjMINVAL) {
        continue;
      }
      element.push_back(0);
      element.push_back(tet[0]);
      element.push_back(tet[1]);
      element.push_back(tet[2]);
    }
  }

  return true;
}



// find string in buffer, return position or -1 if not found
static int findstring(const char* buffer, int buffer_sz, const char* str) {
  int len = (int)strlen(str);

  // scan buffer
  for (int i=0; i < buffer_sz-len; i++) {
    // check for string at position i
    bool found = true;
    for (int k=0; k < len; k++) {
      if (buffer[i+k] != str[k]) {
        found = false;
        break;
      }
    }

    // string found
    if (found) {
      return i;
    }
  }

  // not found
  return -1;
}



// load points and elements from GMSH file
bool mjCFlexcomp::MakeGMSH(mjCModel* model, char* error, int error_sz) {
  // strip path
  if (!file.empty() && model->spec.strippath) {
    file = mjuu_strippath(file);
  }

  // file is required
  if (file.empty()) {
    return comperr(error, "File is required", error_sz);
  }

  // open resource
  mjResource* resource = nullptr;
  try {
    std::string filename = mjuu_combinePaths(mjs_getString(model->spec.meshdir), file);
    resource = mjCBase::LoadResource(mjs_getString(model->spec.modelfiledir),
                                     filename, 0);
  } catch (mjCError err) {
    return comperr(error, err.message, error_sz);
  }

  // try to load, close resource properly
  try {
    LoadGMSH(model, resource);
    mju_closeResource(resource);
  } catch (mjCError err) {
    mju_closeResource(resource);
    return comperr(error, err.message, error_sz);
  } catch (...) {
    mju_closeResource(resource);
    return comperr(error, "exception while reading GMSH file", error_sz);
  }

  return true;
}



// load GMSH format 4.1
void mjCFlexcomp::LoadGMSH41(char* buffer, int binary, int nodeend,
                             int nodebegin, int elemend, int elembegin){
  // header size
  constexpr int kGmsh41HeaderSize = 52;
  // base for node tags, to be subtracted from element data
  size_t minNodeTag, numEntityBlocks, numNodes, maxNodeTag, numNodesInBlock, tag;
  int entityDim, entityTag, parametric;

  // ascii nodes
  if (binary == 0) {
    // convert node char buffer to stringstream
    stringstream ss(std::string(buffer + nodebegin, nodeend - nodebegin));

    // read header
    ss >> numEntityBlocks >> numNodes >> minNodeTag >> maxNodeTag;
    ss >> entityDim >> entityTag >> parametric >> numNodesInBlock;
    if (!ss.good()) {
      throw mjCError(NULL, "Error reading Nodes header");
    }

    // check number of nodes is a positive number
    if (numNodes < 0) {
      throw mjCError(NULL, "Invalid number of nodes");
    }

    // require single block
    if (numEntityBlocks != 1 || numNodes != numNodesInBlock) {
      throw mjCError(NULL, "All nodes must be in single block");
    }

    // require maximum number of nodes be equal to maximum number of nodes in a block
    if (maxNodeTag != numNodesInBlock){
      throw mjCError(NULL, "Maximum number of nodes must be equal to number of nodes in a block");
    }

    // check dimensionality and save
    if (entityDim < 1 || entityDim > 3) {
      throw mjCError(NULL, "Entity must be 1D, 2D or 3D");
    }
    def.spec.flex->dim = entityDim;

    // read and discard node tags; require range from minNodeTag to maxNodeTag
    for (size_t i=0; i < numNodes; i++) {
      size_t tag;
      ss >> tag;
      if (!ss.good()) {
        throw mjCError(NULL, "Error reading node tags");
      }
      if (tag != i+minNodeTag) {
        throw mjCError(NULL, "Node tags must be sequential");
      }
    }

    // read points
    if (numNodes < 0 || numNodes >= INT_MAX / 3) {
      throw mjCError(NULL, "Invalid number of nodes.");
    }
    point.reserve(3*numNodes);
    for (size_t i=0; i < 3*numNodes; i++) {
      double x;
      ss >> x;
      if (!ss.good()) {
        throw mjCError(NULL, "Error reading node coordinates");
      }
      point.push_back(x);
    }
  }
  // binary nodes
  else {
    // check header size
    if (nodeend-nodebegin < kGmsh41HeaderSize) {
      throw mjCError(NULL, "Invalid nodes header");
    }

    // read header
    ReadFromBuffer(&numEntityBlocks, buffer + nodebegin);
    ReadFromBuffer(&numNodes, buffer + nodebegin + 8);
    ReadFromBuffer(&minNodeTag, buffer + nodebegin + 16);
    ReadFromBuffer(&maxNodeTag, buffer + nodebegin + 24);
    ReadFromBuffer(&entityDim, buffer + nodebegin + 32);
    ReadFromBuffer(&entityTag, buffer + nodebegin + 36);
    ReadFromBuffer(&parametric, buffer + nodebegin + 40);
    ReadFromBuffer(&numNodesInBlock, buffer + nodebegin + 44);

    // require single block
    if (numEntityBlocks != 1 || numNodes != numNodesInBlock) {
      throw mjCError(NULL, "All nodes must be in single block");
    }

    // check number of nodes is a positive number
    if (numNodes < 0) {
      throw mjCError(NULL, "Invalid number of nodes");
    }

    // check dimensionality and save
    if (entityDim < 1 || entityDim > 3) {
      throw mjCError(NULL, "Entity must be 1D, 2D or 3D");
    }
    def.spec.flex->dim = entityDim;

    // nodeData: node tag and 3 nodes
    constexpr int numNodeComponents = 4;
    constexpr int componentSize = 8;
    int nodeDataSize = numNodeComponents*componentSize;

    // check section byte size
    if (nodeend-nodebegin < kGmsh41HeaderSize + numNodes*nodeDataSize) {
      throw mjCError(NULL, "Insufficient byte size of Nodes");
    }

    // check node tags: must range from minNodeTag to maxNodeTag
    const char* tagbuffer = buffer + nodebegin + kGmsh41HeaderSize;
    for (size_t i=0; i < numNodes; i++) {
      ReadFromBuffer(&tag, tagbuffer + i*componentSize);
      if (tag != i+minNodeTag) {
        throw mjCError(NULL, "Node tags must be sequential");
      }
    }

    // read points
    if (numNodes < 0 || numNodes >= INT_MAX / 3) {
      throw mjCError(NULL, "Invalid number of nodes.");
    }
    point.reserve(3*numNodes);
    const char* pointbuffer = buffer + nodebegin + kGmsh41HeaderSize + componentSize*numNodes;
    for (size_t i=0; i < 3*numNodes; i++) {
      double x;
      ReadFromBuffer(&x, pointbuffer + i*componentSize);
      point.push_back(x);
    }
  }

  size_t numElements, minElementTag, maxElementTag, numElementsInBlock;
  int elementType;

  // ascii elements
  if (binary == 0) {
    // convert element char buffer to stringstream
    buffer[elemend] = 0;
    stringstream ss(std::string(buffer + elembegin, elemend - elembegin));

    // read header
    ss >> numEntityBlocks >> numElements >> minElementTag >> maxElementTag;
    ss >> entityDim >> entityTag >> elementType >> numElementsInBlock;
    if (!ss.good()) {
      throw mjCError(NULL, "Error reading Elements header");
    }

    // require single block
    if (numEntityBlocks != 1 || numElements != numElementsInBlock) {
      throw mjCError(NULL, "All elements must be in single block");
    }

    // check number of elements is a positive number
    if (numElements < 0) {
      throw mjCError(NULL, "Invalid number of elements");
    }

    // dimensionality must be same as nodes
    if (entityDim != def.spec.flex->dim) {
      throw mjCError(NULL, "Inconsistent dimensionality in Elements");
    }

    if (numElements < 0 || numElements >= INT_MAX / 4) {
      throw mjCError(NULL, "Invalid numElements.");
    }

    // type must be consistent with dimensionality
    if ((entityDim == 1 && elementType != 1) ||
        (entityDim == 2 && elementType != 2) ||
        (entityDim == 3 && elementType != 4)) {
      throw mjCError(NULL, "Element type inconsistent with dimensionality");
    }

    // read elements, discard tags
    element.reserve((entityDim+1)*numElements);
    for (size_t i=0; i < numElements; i++) {
      size_t tag, nodeid;
      ss >> tag;
      for (int k=0; k <= entityDim; k++) {
        ss >> nodeid;
        if (!ss.good()) {
          throw mjCError(NULL, "Error reading Elements");
        }
        element.push_back((int)(nodeid-minNodeTag));
      }
    }
  }

  // binary elements
  else {
    // check header size
    if (elemend-elembegin < kGmsh41HeaderSize) {
      throw mjCError(NULL, "Invalid elements header");
    }

    // read header
    ReadFromBuffer(&numEntityBlocks, buffer + elembegin);
    ReadFromBuffer(&numElements, buffer + elembegin + 8);
    ReadFromBuffer(&minElementTag, buffer + elembegin + 16);
    ReadFromBuffer(&maxElementTag, buffer + elembegin + 24);
    ReadFromBuffer(&entityDim, buffer + elembegin + 32);
    ReadFromBuffer(&entityTag, buffer + elembegin + 36);
    ReadFromBuffer(&elementType, buffer + elembegin + 40);
    ReadFromBuffer(&numElementsInBlock, buffer + elembegin + 44);

    // require single block
    if (numEntityBlocks != 1 || numElements != numElementsInBlock) {
      throw mjCError(NULL, "All elements must be in single block");
    }

    // check number of elements is a positive number
    if (numElements < 0) {
      throw mjCError(NULL, "Invalid number of elements");
    }

    // dimensionality must be same as nodes
    if (entityDim != def.spec.flex->dim) {
      throw mjCError(NULL, "Inconsistent dimensionality in Elements");
    }

    // type must be consistent with dimensionality
    if ((entityDim == 1 && elementType != 1) ||
        (entityDim == 2 && elementType != 2) ||
        (entityDim == 3 && elementType != 4)) {
      throw mjCError(NULL, "Element type inconsistent with dimensionality");
    }

    if (numElements < 0 || numElements >= INT_MAX / 4) {
      throw mjCError(NULL, "Invalid numElements.");
    }

    // elementData: element tag and n node tags
    int numElementComponents = (entityDim+2);
    constexpr int componentSize = 8;
    int elementDataSize = numElementComponents*componentSize;

    // check section byte size
    if (elemend - elembegin < kGmsh41HeaderSize + numElements*elementDataSize) {
      throw mjCError(NULL, "Insufficient byte size of Elements");
    }

    // read elements, discard tags
    element.reserve((entityDim+1)*numElements);
    const char* elembuffer = buffer + elembegin + kGmsh41HeaderSize;
    for (size_t i=0; i < numElements; i++) {
      // skip element tag
      elembuffer += componentSize;

      // read vertex ids
      size_t elemid;
      for (int k=0; k <= entityDim; k++) {
        ReadFromBuffer(&elemid, elembuffer);
        int elementid = elemid - minNodeTag;
        element.push_back(elementid);
        elembuffer += componentSize;
      }
    }
  }
}



// load GMSH format 2.2
void mjCFlexcomp::LoadGMSH22(char* buffer, int binary, int nodeend,
                             int nodebegin, int elemend, int elembegin) {
  // number of nodes
  size_t numNodes = 0;

  // ascii nodes
  if (binary == 0) {
    // convert node char buffer to stringstream
    stringstream ss(std::string(buffer + nodebegin, nodeend - nodebegin));
    std::string line;

    // checking header template
    std::getline(ss, line);
    if (!IsValidElementOrNodeHeader22(line)) {
      throw mjCError(NULL, "Invalid node header");
    }
    ss.seekg(-(line.size()+1), std::ios::cur);

    // read header
    size_t maxNodeTag = 0;
    ss >> maxNodeTag;
    if (!ss.good()) {
      throw mjCError(NULL, "Error reading Nodes header");
    }
    numNodes = maxNodeTag;

    if (numNodes < 0 || numNodes >= INT_MAX / 3) {
      throw mjCError(NULL, "Invalid number of nodes.");
    }

    // read points, discard tag
    point.reserve(3*numNodes);
    for (size_t i=0; i < numNodes; i++) {
      size_t tag;
      double x;
      ss >> tag;
      if (!ss.good()) {
        throw mjCError(NULL, "Error reading node tags");
      }
      // reading nodes
      for (int k=0; k < 3; k++) {
        ss >> x;
        if (!ss.good()) {
          throw mjCError(NULL, "Error reading node coordinates");
        }
        point.push_back(x);
      }
    }
  }

  // binary nodes
  else {
    // header size for gmshApp
    constexpr int nodeHeaderSizeGmshApp = 5;
    // header size compatible with both gmshApp and Ftetwild
    constexpr int nodeHeaderSize = nodeHeaderSizeGmshApp - 1;
    // check header size
    if (nodeend-nodebegin < nodeHeaderSize) {
      throw mjCError(NULL, "Invalid nodes header");
    }

    // parse maxNodeTag and then cast it to int
    char maxNodeTagChar[11] = {0};
    ReadStrFromBuffer(maxNodeTagChar, buffer + nodebegin, std::min(10, nodeend - nodebegin));
    size_t measuredHeaderSize = strnlen(maxNodeTagChar, 10) - 1;
    size_t maxNodeTag;
    try {
      maxNodeTag = std::stoi(maxNodeTagChar);
    } catch (const std::out_of_range& e) {
      throw mjCError(NULL, "Invalid number of nodes");
    }
    numNodes = maxNodeTag;

    // check number of nodes is a positive number
    if (numNodes < 0) {
      throw mjCError(NULL, "Invalid number of nodes");
    }

    // node data: node tag and 3 nodes
    int nodeSize = sizeof(double);
    int indexSize = sizeof(int);
    int nodeDataSize = indexSize + 3*nodeSize;

    // check section byte size
    if (nodeend - nodebegin < nodeHeaderSize + numNodes*nodeDataSize) {
      throw mjCError(NULL, "Insufficient byte size of Nodes");
    }

    // read point, discard tag
    if (numNodes < 0 || numNodes >= INT_MAX / 3) {
      throw mjCError(NULL, "Invalid number of nodes.");
    }
    point.reserve(3*numNodes);
    // beginning of buffer containing node info
    const char* tagBuffer = buffer + nodebegin + measuredHeaderSize;
    for (int i=0; i < numNodes; i++) {
      int tag;
      int offset = i*(sizeof(int) + sizeof(double)*3);
      ReadFromBuffer(&tag, tagBuffer + offset);
      for (int k=0; k < 3; k++) {
        double x;
        const char* nodeBuffer = tagBuffer + sizeof(int) + sizeof(double)*k;
        ReadFromBuffer(&x, nodeBuffer + offset);
        point.push_back(x);
      }
    }
  }


  // ascii elements
  if (binary == 0) {
    // convert element char buffer to stringstream
    buffer[elemend] = 0;
    stringstream ss(std::string(buffer + elembegin, elemend - elembegin));
    std::string line;

    // checking header template
    std::getline(ss, line);
    if (!IsValidElementOrNodeHeader22(line)) {
      throw mjCError(NULL, "Invalid elements header");
    }
    ss.seekg(-(line.size()+1), std::ios::cur);
    // read header
    size_t maxElementTag = 0;
    ss >> maxElementTag;
    if (!ss.good()) {
      throw mjCError(NULL, "Error reading Elements header");
    }
    size_t numElements = maxElementTag;

    if (numElements < 0 || numElements >= INT_MAX / 4) {
      throw mjCError(NULL, "Invalid number of elements.");
    }


    // check number of elements is a positive number
    if (numElements < 0) {
      throw mjCError(NULL, "Invalid number of elements");
    }

    // reading first element's type
    int tag = 0, elementType = 0, numTags = 0;
    ss >> tag >> elementType >> numTags;
    if (!ss.good()) {
      throw mjCError(NULL, "Error reading Elements");
    }

    size_t entityDim = 0;
    int numNodeTags = 0;
    // surface
    if (elementType == 2) {
      entityDim = 2;
      numNodeTags = 3;
    }
    // tetrahedral
    else if (elementType == 4) {
      entityDim = 3;
      numNodeTags = 4;
    }

    if (numNodeTags < 1 || numNodeTags > 4) {
      throw mjCError(NULL, "Invalid number of node tags");
    }

    // setting entityDim
    def.spec.flex->dim = entityDim;

    // read elements, discard all tags
    element.reserve(numNodeTags*numElements);
    for (size_t i=0; i < numElements; i++) {
      int nodeTag = 0, physicalEntityTag = 0, elementModelEntityTag = 0;
      if (i != 0) {
        ss >> tag >> elementType >> numTags;
        if (!ss.good()) {
          throw mjCError(NULL, "Error reading Elements");
        }
      }
      if (numTags > 0) {
        ss >> physicalEntityTag >> elementModelEntityTag;
        if (!ss.good()) {
          throw mjCError(NULL, "Error reading Elements");
        }
      }
      for (int k=0; k < numNodeTags; k++) {
        ss >> nodeTag;
        if (!ss.good()) {
          throw mjCError(NULL, "Error reading Elements");
        }
        if (nodeTag > numNodes || nodeTag < 1) {
          throw mjCError(NULL, "Invalid node tag");
        }
        element.push_back((int)(nodeTag-1));
      }
    }
  }
  // binary elements
  else {
    // header size for gmshApp
    constexpr int elementHeaderSizeGmshApp = 4;
    // header size for Ftetwild
    constexpr int elementHeaderSizeFtetwild = 17;
    // check header size
    if (elemend - elembegin < elementHeaderSizeGmshApp) {
      throw mjCError(NULL, "Invalid elements header");
    }

    // reading elements
    char maxElementTagChar[11] = {0};
    ReadStrFromBuffer(maxElementTagChar, buffer + elembegin, std::min(10, elemend - elembegin));
    int measuredHeaderSize = strnlen(maxElementTagChar, 10) - 1;
    int maxElementTag;
    try {
      maxElementTag = std::stoi(maxElementTagChar);
    } catch (const std::out_of_range& e) {
      throw mjCError(NULL, "Invalid number of elements");
    }
    int numElements = maxElementTag;
    int tag, numTags;
    int nodeTag;
    int elementType;

    // check number of elements is a positive number
    if (numElements < 0) {
      throw mjCError(NULL, "Invalid number of elements");
    }

    // size of single component in element data
    int componentSize = sizeof(int);
    // element buffer
    const char* elementsBuffer = buffer + elembegin + measuredHeaderSize;
    ReadFromBuffer(&elementType, elementsBuffer);
    ReadFromBuffer(&numTags, elementsBuffer + componentSize*2);
    ReadFromBuffer(&tag, elementsBuffer + componentSize*3);

    // tetrahedral has 4 node tags and surface has 3
    int numNodeTags = 0;
    size_t entityDim = 0;
    // surface
    if (elementType == 2) {
      entityDim = 2;
      numNodeTags = 3;
    }
    // tetrahedral
    else if (elementType == 4) {
      entityDim = 3;
      numNodeTags = 4;
    }

    if (numNodeTags < 1 || numNodeTags > 4) {
      throw mjCError(NULL, "Invalid number of node tags");
    }

    def.spec.flex->dim = entityDim;

    // element data(Ftetwild): tag and 4 nodeTag
    constexpr int numComponentsFtetwild = 5;
    // element data(gmshApp): 4 Info components, 2 entity tag and entityDim+1 nodeTags
    constexpr int numInfoComponents = 4;
    constexpr int numEntityTagComponents = 2;

    int numComponentsGmshApp = numInfoComponents + numEntityTagComponents + numNodeTags;

    // single element data size
    int elementDataSizeFtetwild = numComponentsFtetwild*componentSize;
    int elementDataSizeGmshApp = numComponentsGmshApp*componentSize;

    // elements section buffer size
    int elementsBufferSizeFtetwild = elementHeaderSizeFtetwild +
                                     numElements*elementDataSizeFtetwild;
    int elementsBufferSizeGmshApp = elementHeaderSizeGmshApp +
                                    numElements*elementDataSizeGmshApp;

    // check section byte size for ftetwild
    if (elemend - elembegin < elementsBufferSizeFtetwild) {
      throw mjCError(NULL, "Insufficient byte size of Elements");
    }

    // Handling elements produced by gmsh
    if (numTags > 0) {
      // check section byte size for gmsh
      if (elemend - elembegin < elementsBufferSizeGmshApp) {
        throw mjCError(NULL, "Insufficient byte size of Elements");
      }

      // read first element
      for (int k =0; k < numNodeTags; k++) {
        ReadFromBuffer(&nodeTag, elementsBuffer + componentSize*(6+k));
        if (nodeTag > numNodes || nodeTag < 1) {
          throw mjCError(NULL, "Invalid node tag");
        }
        element.push_back(nodeTag-1);
      }

      // read every other element
      for (int i=1; i < numElements; i++) {
        const char* numTagsBuffer = elementsBuffer + componentSize*2;
        const char* tagBuffer = elementsBuffer + componentSize*3;
        int offset = i*elementDataSizeGmshApp;
        ReadFromBuffer(&numTags, numTagsBuffer + offset);
        ReadFromBuffer(&tag, tagBuffer+offset);
        for (int k =0; k < numNodeTags; k++) {
          const char* nodeTagBuffer = elementsBuffer + componentSize*(6+k);
          ReadFromBuffer(&nodeTag, nodeTagBuffer + offset);
          if (nodeTag > numElements || nodeTag < 1) {
            throw mjCError(NULL, "Invalid node tag");
          }
          element.push_back(nodeTag-1);
        }
      }
    }

    // Handling elements produced by ftetwild
    else {
      // read first element
      for (int k = 0; k < numNodeTags; k++) {
        const char* nodeTagBuffer = elementsBuffer + componentSize*(4+k);
        ReadFromBuffer(&nodeTag, nodeTagBuffer);
        if (nodeTag > numNodes || nodeTag < 1) {
          throw mjCError(NULL, "Invalid node tag");
        }
        element.push_back(nodeTag-1);
      }

      // read every other element
      for (int i=0; i < numElements-1; i++) {
        int offset = componentSize*(4+2) + i*elementDataSizeFtetwild;
        const char* tagBuffer = elementsBuffer + componentSize*2;
        ReadFromBuffer(&tag, tagBuffer + offset);
        for (int k=0; k < numNodeTags; k++) {
          const char* nodeTagBuffer = elementsBuffer + componentSize*(3+k);
          ReadFromBuffer(&nodeTag, nodeTagBuffer + offset);
          if (nodeTag > numElements || nodeTag < 1) {
            throw mjCError(NULL, "Invalid node tag");
          }
          element.push_back(nodeTag-1);
        }
      }
    }
  }
}



// load GMSH file from resource
void mjCFlexcomp::LoadGMSH(mjCModel* model, mjResource* resource) {
  // get buffer from resource
  char* buffer = 0;
  int buffer_sz = mju_readResource(resource, (const void**) &buffer);

  // check buffer
  if (buffer_sz < 0) {
    throw mjCError(NULL, "Could not read GMSH file");
  } else if (buffer_sz == 0) {
    throw mjCError(NULL, "Empty GMSH file");
  } else if (buffer_sz < 11 || strncmp(buffer, "$MeshFormat", 11)) {
    throw mjCError(NULL, "GMSH file must begin with $MeshFormat");
  }

  // check version, determine ascii or binary
  double version;
  int binary;
  if (sscanf(buffer + 11, "%lf %d", &version, &binary) != 2) {
    throw mjCError(NULL, "Could not read GMSH file header");
  }
  if (mju_round(100*version) != 220 && mju_round(100*version) != 410) {
    throw mjCError(NULL, "Only GMSH file format versions 4.1 and 2.2 are supported");
  }

  // find section begin/end
  int nodebegin = findstring(buffer, buffer_sz, "$Nodes");
  int nodeend =   findstring(buffer, buffer_sz, "$EndNodes");
  int elembegin = findstring(buffer, buffer_sz, "$Elements");
  int elemend =   findstring(buffer, buffer_sz, "$EndElements");


  // correct begin for string size, +1 for LF in binary (CRLF in Win ascii works)
  nodebegin += (int)strlen("$Nodes") + 1;
  elembegin += (int)strlen("$Elements") + 1;

  // check sections
  if (nodebegin < 0) {
    throw mjCError(NULL, "GMSH file missing $Nodes");
  }
  if (nodeend < nodebegin) {
    throw mjCError(NULL, "GMSH file missing $EndNodes after $Nodes");
  }
  if (elembegin < 0) {
    throw mjCError(NULL, "GMSH file missing $Elements");
  }
  if (elemend < elembegin) {
    throw mjCError(NULL, "GMSH file missing $EndElements after $Elements");
  }

  // Support for 4.1
  if (mju_round(100*version) == 410) {
    LoadGMSH41(buffer, binary, nodeend, nodebegin, elemend, elembegin);
  }

  // Support for 2.2
  else if (mju_round(100*version) == 220) {
    LoadGMSH22(buffer, binary, nodeend, nodebegin, elemend, elembegin);
  } else {
    throw mjCError(NULL, "Unsupported GMSH file format version");
  }
}
