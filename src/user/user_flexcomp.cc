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

#include "user/user_api.h"
#include "user/user_flexcomp.h"
#include <stdio.h>

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <sstream>

#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjplugin.h>
#include "cc/array_safety.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_resource.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_util.h"

namespace {
namespace mju = ::mujoco::util;
using std::vector;
using std::string;
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

  mjm_defaultPlugin(plugin);
  plugin_name = "";
  plugin_instance_name = "";
  plugin.name = (mjString)&plugin_name;
  plugin.instance_name = (mjString)&plugin_instance_name;
}



// make flexcomp object
bool mjCFlexcomp::Make(mjCModel* model, mjmBody* body, char* error, int error_sz) {
  mjmFlex* dflex = def.spec.flex;
  bool radial = (type==mjFCOMPTYPE_BOX ||
                 type==mjFCOMPTYPE_CYLINDER ||
                 type==mjFCOMPTYPE_ELLIPSOID);
  bool direct = (type==mjFCOMPTYPE_DIRECT ||
                 type==mjFCOMPTYPE_MESH ||
                 type==mjFCOMPTYPE_GMSH);

  // check parent body name
  if (std::string(mjm_getString(body->name)).empty()) {
    return comperr(error, "Parent body must have name", error_sz);
  }

  // check counts
  for (int i=0; i<3; i++) {
    if (count[i]<1 || (radial && count[i]<2)) {
      return comperr(error, "Count too small", error_sz);
    }
  }

  // check spacing
  double minspace = 2*dflex->radius + dflex->margin;
  if (!direct) {
    if (spacing[0]<minspace ||
        spacing[1]<minspace ||
        spacing[2]<minspace) {
      return comperr(error, "Spacing must be larger than geometry size", error_sz);
    }
  }

  // check scale
  if (scale[0]<mjMINVAL || scale[1]<mjMINVAL || scale[2]<mjMINVAL) {
    return comperr(error, "Scale must be larger than mjMINVAL", error_sz);
  }

  // check mass and inertia
  if (mass<mjMINVAL || inertiabox<mjMINVAL) {
    return comperr(error, "Mass and inertiabox must be larger than mjMINVAL", error_sz);
  }

  // compute orientation
  const char* alterr = alt.Set(quat, model->spec.degree, model->spec.euler);
  if (alterr) {
    return comperr(error, alterr, error_sz);
  }

  // type-specific constructor: populate point and element, possibly set dim
  bool res;
  switch (type) {
  case mjFCOMPTYPE_GRID:
    res = MakeGrid(error, error_sz);
    break;

  case mjFCOMPTYPE_BOX:
  case mjFCOMPTYPE_CYLINDER:
  case mjFCOMPTYPE_ELLIPSOID:
    res = MakeBox(error, error_sz);
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
    return comperr(error, "Uknown flexcomp type", error_sz);
  }
  if (!res) {
    return false;
  }

  // get dim and check
  int dim = dflex->dim;
  if (dim<1 || dim>3) {
    return comperr(error, "Invalid dim, must be between 1 and 3", error_sz);
  }

  // force flatskin shading for box, cylinder and 3D grid
  if (type==mjFCOMPTYPE_BOX || type==mjFCOMPTYPE_CYLINDER ||
      (type==mjFCOMPTYPE_GRID && dim==3)) {
    dflex->flatskin = true;
  }

  // check pin sizes
  if (pinrange.size()%2) {
    return comperr(error, "Pin range number must be multiple of 2", error_sz);
  }
  if (pingrid.size()%dim) {
    return comperr(error, "Pin grid number must be multiple of dim", error_sz);
  }
  if (pingridrange.size()%(2*dim)) {
    return comperr(error, "Pin grid range number of must be multiple of 2*dim", error_sz);
  }
  if (type!=mjFCOMPTYPE_GRID && !(pingrid.empty() && pingridrange.empty())) {
    return comperr(error, "Pin grid(range) can only be used with grid type", error_sz);
  }
  if (dim==1 && !(pingrid.empty() && pingridrange.empty())) {
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
  if (element.size()%(dim+1)) {
    return comperr(error, "Element size must be a multiple of dim+1", error_sz);
  }

  // get number of points
  int npnt = point.size()/3;

  // check elem vertex ids
  for (int i=0; i<(int)element.size(); i++) {
    if (element[i]<0 || element[i]>=npnt) {
      char msg[100];
      snprintf(msg, sizeof(msg), "element %d has point id %d, number of points is %d", i,
               element[i], npnt);
      return comperr(error, msg, error_sz);
    }
  }

  // apply scaling for direct types
  if (direct && (scale[0]!=1 || scale[1]!=1 || scale[2]!=1)) {
    for (int i=0; i<npnt; i++) {
      point[3*i] *= scale[0];
      point[3*i+1] *= scale[1];
      point[3*i+2] *= scale[2];
    }
  }

  // apply pose transform to points
  mjtNum posn[3], quatn[4];
  mju_d2n(posn, pos, 3);
  mju_d2n(quatn, quat, 4);
  for (int i=0; i<npnt; i++) {
    mjtNum newp[3], oldp[3] = {point[3*i], point[3*i+1], point[3*i+2]};
    mju_trnVecPose(newp, posn, quatn, oldp);
    point[3*i] = newp[0];
    point[3*i+1] = newp[1];
    point[3*i+2] = newp[2];
  }

  // construct pinned array
  pinned = vector<bool>(npnt, rigid);

  // handle pins if user did not specify rigid
  if (!rigid) {
    // process pinid
    for (int i=0; i<(int)pinid.size(); i++) {
      // check range
      if (pinid[i]<0 || pinid[i]>=npnt) {
        return comperr(error, "pinid out of range", error_sz);
      }

      // set
      pinned[pinid[i]] = true;
    }

    // process pinrange
    for (int i=0; i<(int)pinrange.size(); i+=2) {
      // check range
      if (pinrange[i]<0 || pinrange[i]>=npnt ||
          pinrange[i+1]<0 || pinrange[i+1]>=npnt) {
        return comperr(error, "pinrange out of range", error_sz);
      }

      // set
      for (int k=pinrange[i]; k<=pinrange[i+1]; k++) {
        pinned[k] = true;
      }
    }

    // process pingrid
    for (int i=0; i<(int)pingrid.size(); i+=dim) {
      // check range
      for (int k=0; k<dim; k++) {
        if (pingrid[i+k]<0 || pingrid[i+k]>=count[k]) {
          return comperr(error, "pingrid out of range", error_sz);
        }
      }

      // set
      if (dim==2) {
        pinned[GridID(pingrid[i], pingrid[i+1])] = true;
      }
      else if (dim==3) {
        pinned[GridID(pingrid[i], pingrid[i+1], pingrid[i+2])] = true;
      }
    }

    // process pingridrange
    for (int i=0; i<(int)pingridrange.size(); i+=2*dim) {
      // check range
      for (int k=0; k<2*dim; k++) {
        if (pingridrange[i+k]<0 || pingridrange[i+k]>=count[k%dim]) {
          return comperr(error, "pingridrange out of range", error_sz);
        }
      }

      // set
      if (dim==2) {
        for (int ix=pingridrange[i]; ix<=pingridrange[i+2]; ix++) {
          for (int iy=pingridrange[i+1]; iy<=pingridrange[i+3]; iy++) {
            pinned[GridID(ix, iy)] = true;
          }
        }
      }
      else if (dim==3) {
        for (int ix=pingridrange[i]; ix<=pingridrange[i+3]; ix++) {
          for (int iy=pingridrange[i+1]; iy<=pingridrange[i+4]; iy++) {
            for (int iz=pingridrange[i+2]; iz<=pingridrange[i+5]; iz++) {
              pinned[GridID(ix, iy, iz)] = true;
            }
          }
        }
      }
    }

    // center of radial body is always pinned
    if (radial) {
      pinned[0] = true;
    }

    // check if all or none are pinned
    bool allpin = true, nopin = true;
    for (int i=0; i<npnt; i++) {
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
    for (int i=0; i<(int)element.size(); i++) {
      used[element[i]] = true;
    }

    // construct reindex
    bool hasunused = false;
    std::vector<int> reindex (npnt, 0);
    for (int i=0; i<npnt; i++) {
      if (!used[i]) {
        hasunused = true;
        for (int k=i+1; k<npnt; k++) {
          reindex[k]--;
        }
      }
    }

    // reindex elements if unused present
    if (hasunused) {
      for (int i=0; i<(int)element.size(); i++) {
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
  mjmFlex* pf = &flex->spec;
  int id = flex->id;

  *flex = def.flex;
  flex->PointToLocal();

  flex->model = model;
  flex->id = id;
  mjm_setString(pf->name, name.c_str());
  mjm_setInt(pf->elem, element.data(), element.size());
  mjm_setFloat(pf->texcoord, texcoord.data(), texcoord.size());

  // rigid: set parent name, nothing else to do
  if (rigid) {
    mjm_appendString(pf->vertbody, mjm_getString(body->name));
    return true;
  }

  // compute body mass and inertia matching specs
  double bodymass = mass/npnt;
  double bodyinertia = bodymass*(2.0*inertiabox*inertiabox)/3.0;

  // overwrite plugin name
  if (plugin.active && plugin_instance_name.empty()) {
    plugin_instance_name = "flexcomp_" + name;
    ((mjCPlugin*)plugin.instance)->name = plugin_instance_name;
  }

  // create bodies, construct flex vert and vertbody
  for (int i=0; i<npnt; i++) {
    // not used: skip
    if (!used[i]) {
      continue;
    }

    // pinned: parent body
    if (pinned[i]) {
      mjm_appendString(pf->vertbody, mjm_getString(body->name));

      // add plugin
      if (plugin.active) {
        mjmPlugin* pplugin = &body->plugin;
        pplugin->active = true;
        pplugin->instance = (mjElement)plugin.instance;
        mjm_setString(pplugin->name, mjm_getString(plugin.name));
        mjm_setString(pplugin->instance_name, plugin_instance_name.c_str());
      }
    }

    // not pinned: new body
    else {
      // add new body at vertex coordinates
      mjmBody* pb = mjm_addBody(body, 0);

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
      if (radial) {
        mjmJoint* jnt = mjm_addJoint(pb, 0);

        // set properties
        jnt->type = mjJNT_SLIDE;
        mjuu_setvec(jnt->pos, 0, 0, 0);
        mjuu_copyvec(jnt->axis, pb->pos, 3);
        mjuu_normvec(jnt->axis, 3);
      }

      // add three orthogonal sliders
      else {
        for (int j=0; j<3; j++) {
          // add joint to body
          mjmJoint* jnt = mjm_addJoint(pb, 0);

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
      mjm_setString(pb->name, txt);
      mjm_appendString(pf->vertbody, mjm_getString(pb->name));

      // clear flex vertex coordinates if allocated
      if (!centered) {
        point[3*i] = 0;
        point[3*i+1] = 0;
        point[3*i+2] = 0;
      }

      // add plugin
      if (plugin.active) {
        mjmPlugin* pplugin = &pb->plugin;
        pplugin->active = true;
        pplugin->instance = (mjElement)plugin.instance;
        mjm_setString(pplugin->name, mjm_getString(plugin.name));
        mjm_setString(pplugin->instance_name, plugin_instance_name.c_str());
      }
    }
  }

  if (!centered) {
    mjm_setDouble(pf->vert, point.data(), point.size());
  }

  // create edge equality constraint
  if (equality) {
    mjmEquality* pe = mjm_addEquality(&model->spec, &def.spec);
    mjm_setDefault(pe->element, &model->defaults[0]->spec);
    pe->type = mjEQ_FLEX;
    pe->active = true;
    mjm_setString(pe->name1, name.c_str());
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
  int dim = def.flex.spec.dim;
  bool hastex = texcoord.empty();

  // 1D
  if (dim==1) {
    for (int ix=0; ix<count[0]; ix++) {
      // add point
      point.push_back(spacing[0]*(ix - 0.5*(count[0]-1)));
      point.push_back(0);
      point.push_back(0);

      // add element
      if (ix<count[0]-1) {
        element.push_back(ix);
        element.push_back(ix+1);
      }
    }
  }

  // 2D
  else if (dim==2) {
    int quad2tri[2][3] = {{0, 1, 2}, {0, 2, 3}};
    for (int ix=0; ix<count[0]; ix++) {
      for (int iy=0; iy<count[1]; iy++) {
        // add point
        point.push_back(spacing[0]*(ix - 0.5*(count[0]-1)));
        point.push_back(spacing[1]*(iy - 0.5*(count[1]-1)));
        point.push_back(0);

        // add texture coordinates, if not specified explicitly
        if (!hastex) {
          texcoord.push_back(ix/(mjtNum)mjMAX(count[0]-1, 1));
          texcoord.push_back(iy/(mjtNum)mjMAX(count[1]-1, 1));
        }

        // add elements
        if (ix<count[0]-1 && iy<count[1]-1) {
          int vert[4] = {
            count[2]*count[1]*(ix+0) + count[2]*(iy+0),
            count[2]*count[1]*(ix+1) + count[2]*(iy+0),
            count[2]*count[1]*(ix+1) + count[2]*(iy+1),
            count[2]*count[1]*(ix+0) + count[2]*(iy+1),
          };
          for (int s = 0; s < 2; s++) {
            for (int v = 0; v < 3; v++) {
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
    for (int ix=0; ix<count[0]; ix++) {
      for (int iy=0; iy<count[1]; iy++) {
        for (int iz=0; iz<count[2]; iz++) {
          // add point
          point.push_back(spacing[0]*(ix - 0.5*(count[0]-1)));
          point.push_back(spacing[1]*(iy - 0.5*(count[1]-1)));
          point.push_back(spacing[2]*(iz - 0.5*(count[2]-1)));

          // add elements
          if (ix<count[0]-1 && iy<count[1]-1 && iz<count[2]-1) {
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
            for (int s = 0; s < 6; s++) {
              for (int v = 0; v < 4; v++) {
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
  if (iz==0) {
    return ix*count[1] + iy + 1;
  }

  // side iz=max
  else if (iz==count[2]-1) {
    return count[0]*count[1] + ix*count[1] + iy + 1;
  }

  // side iy=0
  else if (iy==0) {
    return 2*count[0]*count[1] + ix*(count[2]-2) + iz-1 + 1;
  }

  // side iy=max
  else if (iy==count[1]-1) {
    return 2*count[0]*count[1] + count[0]*(count[2]-2) + ix*(count[2]-2) + iz-1 + 1;
  }

  // side ix=0
  else if (ix==0) {
    return 2*count[0]*count[1] + 2*count[0]*(count[2]-2) + (iy-1)*(count[2]-2) + iz-1 + 1;
  }

  // side ix=max
  else {
    return 2*count[0]*count[1] + 2*count[0]*(count[2]-2) + (count[1]-2)*(count[2]-2) +
           (iy-1)*(count[2]-2) + iz-1 + 1;
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
  if (type==mjFCOMPTYPE_BOX) {
    pos[0] *= size[0];
    pos[1] *= size[1];
    pos[2] *= size[2];
  }

  // cylinder
  else if (type==mjFCOMPTYPE_CYLINDER) {
    double L0 = mjMAX(mju_abs(pos[0]), mju_abs(pos[1]));
    mjuu_normvec(pos, 2);
    pos[0] *= size[0]*L0;
    pos[1] *= size[1]*L0;
    pos[2] *= size[2];
  }

  // ellipsoid
  else if (type==mjFCOMPTYPE_ELLIPSOID) {
    mjuu_normvec(pos, 3);
    pos[0] *= size[0];
    pos[1] *= size[1];
    pos[2] *= size[2];
  }
}



// make 3d box, ellipsoid or cylinder
bool mjCFlexcomp::MakeBox(char* error, int error_sz) {
  double pos[3];

  // set 3D
  def.spec.flex->dim = 3;

  // add center point
  point.push_back(0);
  point.push_back(0);
  point.push_back(0);

  // iz=0/max
  for (int iz=0; iz<count[2]; iz+=count[2]-1) {
    for (int ix=0; ix<count[0]; ix++) {
      for (int iy=0; iy<count[1]; iy++) {
        // add point
        BoxProject(pos, ix, iy, iz);
        point.push_back(pos[0]);
        point.push_back(pos[1]);
        point.push_back(pos[2]);

        // add elements
        if (ix<count[0]-1 && iy<count[1]-1) {
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
  for (int iy=0; iy<count[1]; iy+=count[1]-1) {
    for (int ix=0; ix<count[0]; ix++) {
      for (int iz=0; iz<count[2]; iz++) {
        // add point
        if (iz>0 && iz<count[2]-1) {
          BoxProject(pos, ix, iy, iz);
          point.push_back(pos[0]);
          point.push_back(pos[1]);
          point.push_back(pos[2]);
        }

        // add elements
        if (ix<count[0]-1 && iz<count[2]-1) {
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
  for (int ix=0; ix<count[0]; ix+=count[0]-1) {
    for (int iy=0; iy<count[1]; iy++) {
      for (int iz=0; iz<count[2]; iz++) {
        // add point
        if (iz>0 && iz<count[2]-1 && iy>0 && iy<count[1]-1) {
          BoxProject(pos, ix, iy, iz);
          point.push_back(pos[0]);
          point.push_back(pos[1]);
          point.push_back(pos[2]);
        }

        // add elements
        if (iy<count[1]-1 && iz<count[2]-1) {
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

  // get extension and check; must be STL, OBJ or MSH
  string ext = mjuu_getext(file);
  if (strcasecmp(ext.c_str(), ".stl") &&
      strcasecmp(ext.c_str(), ".obj") &&
      strcasecmp(ext.c_str(), ".msh")) {
    return comperr(error, "Mesh file extension must be stl, obj or msh", error_sz);
  }

  // check dim
  if (def.spec.flex->dim!=2) {
    return comperr(error, "Flex dim must be 2 in for mesh", error_sz);
  }

  // load resource
  string filename = mjuu_makefullname(model->modelfiledir, mjm_getString(model->spec.meshdir), file);
  mjResource* resource = nullptr;

  try {
    resource = mjCBase::LoadResource(filename, 0);
  } catch (mjCError err) {
    return comperr(error, err.message, error_sz);
  }

  // load mesh
  mjCMesh mesh;
  bool isobj = false;
  try {
    if (!strcasecmp(ext.c_str(), ".stl")) {
      mesh.LoadSTL(resource);
    } else if (!strcasecmp(ext.c_str(), ".obj")) {
      isobj = true;
      mesh.LoadOBJ(resource);
    } else {
      mesh.LoadMSH(resource);
    }
    mju_closeResource(resource);
  } catch (mjCError err) {
    mju_closeResource(resource);
    return comperr(error, err.message, error_sz);
  }

  // LoadOBJ uses userXXX, extra processing needed
  if (isobj) {
    // check sizes
    if (mesh.get_uservert().empty() || mesh.get_userface().empty()) {
      return comperr(error, "Vertex and face data required", error_sz);
    }
    if (mesh.get_uservert().size()%3) {
      return comperr(error, "Vertex data must be multiple of 3", error_sz);
    }
    if (mesh.get_userface().size()%3) {
      return comperr(error, "Face data must be multiple of 3", error_sz);
    }

    // copy vectors and clear
    mesh.nvert_ = mesh.uservert_.size()/3;
    mesh.nface_ = mesh.userface_.size()/3;
    mesh.vert_ = VecToArray(mesh.uservert_, true);
    mesh.face_ = VecToArray(mesh.userface_, true);

    // remove repeated vertices (not called in LoadOBJ)
    mesh.RemoveRepeated();
  }

  // copy faces
  element = vector<int> (mesh.nface()*3);
  memcpy(element.data(), mesh.face_, mesh.nface_*3*sizeof(int));

  // copy vertices, convert from float to mjtNum
  point = vector<mjtNum> (mesh.nvert()*3);
  for (int i=0; i<mesh.nvert()*3; i++) {
    point[i] = (mjtNum) mesh.vert_[i];
  }

  return true;
}



// find string in buffer, return position or -1 if not found
static int findstring(const char* buffer, int buffer_sz, const char* str) {
  int len = (int)strlen(str);

  // scan buffer
  for (int i=0; i<buffer_sz-len; i++) {

    // check for string at position i
    bool found = true;
    for (int k=0; k<len; k++) {
      if (buffer[i+k]!=str[k]) {
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
  string filename = mjuu_makefullname(model->modelfiledir, mjm_getString(model->spec.meshdir), file);
  mjResource* resource = nullptr;

  try {
    resource = mjCBase::LoadResource(filename, 0);
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



// load GMSH file from resource
void mjCFlexcomp::LoadGMSH(mjCModel* model, mjResource* resource) {
  // get buffer from resource
  char* buffer = 0;
  int buffer_sz = mju_readResource(resource, (const void**) &buffer);

  // check buffer
  if (buffer_sz<0) {
    throw mjCError(NULL, "Could not read GMSH file");
  } else if (buffer_sz==0) {
    throw mjCError(NULL, "Empty GMSH file");
  } else if (buffer_sz<11 || strncmp(buffer, "$MeshFormat", 11)) {
    throw mjCError(NULL, "GMSH file must begin with $MeshFormat");
  }

  // check version, determine ascii or binary
  double version;
  int binary;
  if (sscanf(buffer+11, "%lf %d", &version, &binary) != 2) {
    throw mjCError(NULL, "Could not read GMSH file header");
  }
  if (mju_round(100*version)!=410) {
    throw mjCError(NULL, "Only GMSH file format 4.1 supported");
  }

  // find section begin/end
  int nodebegin = findstring(buffer, buffer_sz, "$Nodes");
  int nodeend =   findstring(buffer, buffer_sz, "$EndNodes");
  int elembegin = findstring(buffer, buffer_sz, "$Elements");
  int elemend =   findstring(buffer, buffer_sz, "$EndElements");

  // check sections
  if (nodebegin<0) {
    throw mjCError(NULL, "GMSH file missing $Nodes");
  }
  if (nodeend<nodebegin) {
    throw mjCError(NULL, "GMSH file missing $EndNodes after $Nodes");
  }
  if (elembegin<0) {
    throw mjCError(NULL, "GMSH file missing $Elements");
  }
  if (elemend<elembegin) {
    throw mjCError(NULL, "GMSH file missing $EndElements after $Elements");
  }

  // correct begin for string size, +1 for LF in binary (CRLF in Win ascii works)
  nodebegin += (int)strlen("$Nodes") + 1;
  elembegin += (int)strlen("$Elements") + 1;

  // base for node tags, to be subtracted from element data
  size_t minNodeTag, numEntityBlocks, numNodes, maxNodeTag, numNodesInBlock, tag;
  int entityDim, entityTag, parametric;

  // ascii nodes
  if (binary==0) {
    // convert node char buffer to stringstream
    buffer[nodeend] = 0;
    stringstream ss(buffer+nodebegin);

    // read header
    ss >> numEntityBlocks >> numNodes >> minNodeTag >> maxNodeTag;
    ss >> entityDim >> entityTag >> parametric >> numNodesInBlock;
    if (!ss.good()) {
      throw mjCError(NULL, "Error reading Nodes header");
    }

    // require single block
    if (numEntityBlocks!=1 || numNodes!=numNodesInBlock) {
      throw mjCError(NULL, "All nodes must be in single block");
    }

    // check dimensionality and save
    if (entityDim<1 || entityDim>3) {
      throw mjCError(NULL, "Entity must be 1D, 2D or 3D");
    }
    def.spec.flex->dim = entityDim;

    // read and discard node tags; require range from minNodeTag to maxNodeTag
    for (size_t i=0; i<numNodes; i++) {
      size_t tag;
      ss >> tag;
      if (!ss.good()) {
        throw mjCError(NULL, "Error reading node tags");
      }
      if (tag!=i+minNodeTag) {
        throw mjCError(NULL, "Node tags must be sequential");
      }
    }

    // read points
    point.reserve(3*numNodes);
    for (size_t i=0; i<3*numNodes; i++) {
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
    // check header size: 5 size_t, 3 int
    if (nodeend-nodebegin < 52) {
      throw mjCError(NULL, "Invalid nodes header");
    }

    // read header
    ReadFromBuffer(&numEntityBlocks, buffer+nodebegin);
    ReadFromBuffer(&numNodes, buffer+nodebegin+8);
    ReadFromBuffer(&minNodeTag, buffer+nodebegin+16);
    ReadFromBuffer(&maxNodeTag, buffer+nodebegin+24);
    ReadFromBuffer(&entityDim, buffer+nodebegin+32);
    ReadFromBuffer(&entityTag, buffer+nodebegin+36);
    ReadFromBuffer(&parametric, buffer+nodebegin+40);
    ReadFromBuffer(&numNodesInBlock, buffer+nodebegin+44);

    // require single block
    if (numEntityBlocks!=1 || numNodes!=numNodesInBlock) {
      throw mjCError(NULL, "All nodes must be in single block");
    }

    // check dimensionality and save
    if (entityDim<1 || entityDim>3) {
      throw mjCError(NULL, "Entity must be 1D, 2D or 3D");
    }
    def.spec.flex->dim = entityDim;

    // check section byte size
    if (nodeend-nodebegin < 52+numNodes*4*8) {
      throw mjCError(NULL, "Insufficient byte size of Nodes");
    }

    // check node tags: must range from minNodeTag to maxNodeTag
    const char* tagbuffer = buffer + nodebegin + 52;
    for (size_t i=0; i<numNodes; i++) {
      ReadFromBuffer(&tag, tagbuffer+i*8);
      if (tag!=i+minNodeTag) {
        throw mjCError(NULL, "Node tags must be sequential");
      }
    }

    // read points
    double x;
    point.reserve(3*numNodes);
    const char* pointbuffer = buffer + nodebegin + 52 + 8*numNodes;
    for (size_t i=0; i<3*numNodes; i++) {
      ReadFromBuffer(&x, pointbuffer+i*8);
      point.push_back(x);
    }
  }

  size_t numElements, minElementTag, maxElementTag, numElementsInBlock;
  int elementType;

  // ascii elements
  if (binary==0) {
    // convert element char buffer to stringstream
    buffer[elemend] = 0;
    stringstream ss(buffer+elembegin);

    // read header
    ss >> numEntityBlocks >> numElements >> minElementTag >> maxElementTag;
    ss >> entityDim >> entityTag >> elementType >> numElementsInBlock;
    if (!ss.good()) {
      throw mjCError(NULL, "Error reading Elements header");
    }

    // require single block
    if (numEntityBlocks!=1 || numElements!=numElementsInBlock) {
      throw mjCError(NULL, "All elements must be in single block");
    }

    // dimensionality must be same as nodes
    if (entityDim!=def.spec.flex->dim) {
      throw mjCError(NULL, "Inconsistent dimensionality in Elements");
    }

    // type must be consistent with dimensionality
    if ((entityDim==1 && elementType!=1) ||
        (entityDim==2 && elementType!=2) ||
        (entityDim==3 && elementType!=4)) {
      throw mjCError(NULL, "Element type inconsistent with dimensionality");
    }

    // read elements, discard tags
    element.reserve((entityDim+1)*numElements);
    for (size_t i=0; i<numElements; i++) {
      size_t tag, nodeid;
      ss >> tag;
      for (int k=0; k<=entityDim; k++) {
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
    // check header size: 5 size_t, 3 int
    if (elemend-elembegin < 52) {
      throw mjCError(NULL, "Invalid elements header");
    }

    // read header
    ReadFromBuffer(&numEntityBlocks, buffer+elembegin);
    ReadFromBuffer(&numElements, buffer+elembegin+8);
    ReadFromBuffer(&minElementTag, buffer+elembegin+16);
    ReadFromBuffer(&maxElementTag, buffer+elembegin+24);
    ReadFromBuffer(&entityDim, buffer+elembegin+32);
    ReadFromBuffer(&entityTag, buffer+elembegin+36);
    ReadFromBuffer(&elementType, buffer+elembegin+40);
    ReadFromBuffer(&numElementsInBlock, buffer+elembegin+44);

    // require single block
    if (numEntityBlocks!=1 || numElements!=numElementsInBlock) {
      throw mjCError(NULL, "All elements must be in single block");
    }

    // dimensionality must be same as nodes
    if (entityDim!=def.spec.flex->dim) {
      throw mjCError(NULL, "Inconsistent dimensionality in Elements");
    }

    // type must be consistent with dimensionality
    if ((entityDim==1 && elementType!=1) ||
        (entityDim==2 && elementType!=2) ||
        (entityDim==3 && elementType!=4)) {
      throw mjCError(NULL, "Element type inconsistent with dimensionality");
    }

    // check section byte size
    if (elemend-elembegin < 52+numElements*(entityDim+2)*8) {
      throw mjCError(NULL, "Insufficient byte size of Elements");
    }

    // read elements, discard tags
    element.reserve((entityDim+1)*numElements);
    const char* elembuffer = buffer + elembegin + 52;
    for (size_t i=0; i<numElements; i++) {
      // skip element tag
      elembuffer += 8;

      // read vertex ids
      size_t elemid;
      for (int k=0; k<=entityDim; k++) {
        ReadFromBuffer(&elemid, elembuffer);
        int elementid = elemid - minNodeTag;
        element.push_back(elementid);
        elembuffer += 8;
      }
    }
  }
}
