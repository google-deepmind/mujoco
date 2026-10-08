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
#include <mujoco/mjtype.h>
#include <mujoco/mjplugin.h>
#include "cc/array_safety.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_plugin.h"
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
using std::stringstream;
using std::vector;
}  // namespace

// strncpy with 0, return false
static bool comperr(char* error, const char* msg, int error_sz) {
  mju_strncpy(error, msg, error_sz);
  return false;
}


// constructor: set defaults outside mjCDef
mjCFlexcomp::mjCFlexcomp(void) {
  type     = mjFCOMPTYPE_GRID;
  count[0] = count[1] = count[2] = 10;
  cellcount[0] = cellcount[1] = cellcount[2] = -1;
  mjuu_setvec(spacing, 0.02, 0.02, 0.02);
  mjuu_setvec(scale, 1, 1, 1);
  mass       = 1;
  inertiabox = 0.005;
  equality   = 0;
  mjuu_setvec(pos, 0, 0, 0);
  mjuu_setvec(quat, 1, 0, 0, 0);
  rigid    = false;
  centered = false;
  doftype  = mjFCOMPDOF_FULL;
  has_dim  = false;

  mjs_defaultPlugin(&plugin);
  mjs_defaultOrientation(&alt);
  plugin_name          = "";
  plugin_instance_name = "";
  plugin.plugin_name   = (mjString*)&plugin_name;
  plugin.name          = (mjString*)&plugin_instance_name;
}


// identify empty cells and pin nodes exclusively in empty cells
void mjCFlexcomp::MarkEmptyCells(
    mjCFlex* flex, const double* points, const double minmax[6], int nx, int ny, int nz) {
  int cx    = flex->spec.cellcount[0];
  int cy    = flex->spec.cellcount[1];
  int cz    = flex->spec.cellcount[2];
  int order = flex->spec.order;

  // delegate cell_empty computation to mjCFlex
  int nelem = element.size() / (flex->spec.dim + 1);
  flex->ComputeCellEmpty(points, element.data(), nelem, flex->spec.dim, minmax);

  // pin nodes that belong exclusively to empty cells
  for (int gi = 0; gi < nx; gi++) {
    for (int gj = 0; gj < ny; gj++) {
      for (int gk = 0; gk < nz; gk++) {
        // find all cells that reference this node
        bool all_empty = true;
        int  ci_min    = std::max(0, gi == 0 ? 0 : (gi - 1) / order);
        int  ci_max    = std::min(cx - 1, gi / order);
        int  cj_min    = std::max(0, gj == 0 ? 0 : (gj - 1) / order);
        int  cj_max    = std::min(cy - 1, gj / order);
        int  ck_min    = std::max(0, gk == 0 ? 0 : (gk - 1) / order);
        int  ck_max    = std::min(cz - 1, gk / order);

        for (int ci = ci_min; ci <= ci_max && all_empty; ci++) {
          for (int cj = cj_min; cj <= cj_max && all_empty; cj++) {
            for (int ck = ck_min; ck <= ck_max && all_empty; ck++) {
              if (!flex->cell_empty[ci * cy * cz + cj * cz + ck]) { all_empty = false; }
            }
          }
        }

        if (all_empty) {
          int idx     = gi * ny * nz + gj * nz + gk;
          pinned[idx] = true;
        }
      }
    }
  }
}


// make flexcomp object
bool mjCFlexcomp::Make(mjsBody* body, char* error, int error_sz, const mjVFS* vfs) {
  mjCModel*    model    = static_cast<mjCBody*>(body->element)->model;
  mjsCompiler* compiler = static_cast<mjCBody*>(body->element)->compiler;
  mjsFlex*     dflex    = def.spec.flex;
  bool         direct =
      (type == mjFCOMPTYPE_DIRECT || type == mjFCOMPTYPE_MESH || type == mjFCOMPTYPE_GMSH);

  // check parent body name
  if (mjs_getName(body->element)->empty()) {
    return comperr(error, "Parent body must have name", error_sz);
  }

  // check dim
  if (dflex->dim < 1 || dflex->dim > 3) {
    return comperr(error, "Invalid dim, must be between 1 and 3", error_sz);
  }

  // check counts
  for (int i = 0; i < 3; i++) {
    if (count[i] < 1 || ((doftype == mjFCOMPDOF_RADIAL && count[i] < 2) && dflex->dim == 3)) {
      return comperr(error, "Count too small", error_sz);
    }
  }

  // check spacing
  double minspace = 2 * dflex->radius + dflex->margin;
  if (!direct) {
    if (spacing[0] < minspace || spacing[1] < minspace || spacing[2] < minspace) {
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
  if (alterr) { return comperr(error, alterr, error_sz); }

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
      res = MakeBox(error, error_sz, dflex->dim);
      break;

    case mjFCOMPTYPE_SQUARE:
    case mjFCOMPTYPE_DISC:
      res = MakeSquare(error, error_sz);
      break;

    case mjFCOMPTYPE_MESH:
      res = MakeMesh(model, compiler, error, error_sz, vfs);
      break;

    case mjFCOMPTYPE_GMSH:
      res = MakeGMSH(model, compiler, error, error_sz, vfs);
      break;

    case mjFCOMPTYPE_DIRECT:
      res = true;
      break;

    default:
      return comperr(error, "Unknown flexcomp type", error_sz);
  }
  if (!res) { return false; }

  // force flatskin shading for box, cylinder and 3D grid
  if (type == mjFCOMPTYPE_BOX ||
      type == mjFCOMPTYPE_CYLINDER ||
      (type == mjFCOMPTYPE_GRID && dflex->dim == 3)) {
    dflex->flatskin = true;
  }

  // check pin sizes
  if (pinrange.size() % 2) {
    return comperr(error, "Pin range number must be multiple of 2", error_sz);
  }
  if (pingrid.size() % dflex->dim) {
    return comperr(error, "Pin grid number must be multiple of dim", error_sz);
  }
  if (pingridrange.size() % (2 * dflex->dim)) {
    return comperr(error, "Pin grid range number of must be multiple of 2*dim", error_sz);
  }
  if (type != mjFCOMPTYPE_GRID &&
      !(pingrid.empty() && pingridrange.empty()) &&
      doftype != mjFCOMPDOF_TRILINEAR &&
      doftype != mjFCOMPDOF_QUADRATIC) {
    return comperr(error, "Pin grid(range) can only be used with grid or interpolated", error_sz);
  }
  if (dflex->dim == 1 && !(pingrid.empty() && pingridrange.empty())) {
    return comperr(error, "Pin grid(range) cannot be used with dim=1", error_sz);
  }

  // require element and point
  if (point.empty() || element.empty()) {
    return comperr(error, "Point and element required", error_sz);
  }

  // check point size
  if (point.size() % 3) { return comperr(error, "Point size must be a multiple of 3", error_sz); }

  // check element size
  if (element.size() % (dflex->dim + 1)) {
    return comperr(error, "Element size must be a multiple of dim+1", error_sz);
  }

  // get number of points
  int npnt = point.size() / 3;

  // check elem vertex ids
  for (int i = 0; i < (int)element.size(); i++) {
    if (element[i] < 0 || element[i] >= npnt) {
      char msg[100];
      snprintf(msg,
               sizeof(msg),
               "element %d has point id %d, number of points is %d",
               i,
               element[i],
               npnt);
      return comperr(error, msg, error_sz);
    }
  }

  // apply scaling for direct types
  if (direct && (scale[0] != 1 || scale[1] != 1 || scale[2] != 1)) {
    for (int i = 0; i < npnt; i++) {
      point[3 * i]     *= scale[0];
      point[3 * i + 1] *= scale[1];
      point[3 * i + 2] *= scale[2];
    }
  }

  // apply pose transform to points
  for (int i = 0; i < npnt; i++) {
    double newp[3], oldp[3] = {point[3 * i], point[3 * i + 1], point[3 * i + 2]};
    mjuu_trnVecPose(newp, pos, quat, oldp);
    point[3 * i]     = newp[0];
    point[3 * i + 1] = newp[1];
    point[3 * i + 2] = newp[2];
  }

  // compute bounding box of points
  double minmax[6] = {mjMAXVAL, mjMAXVAL, mjMAXVAL, -mjMAXVAL, -mjMAXVAL, -mjMAXVAL};
  for (int i = 0; i < npnt; i++) {
    for (int j = 0; j < 3; j++) {
      minmax[j + 0] = std::min(minmax[j + 0], point[3 * i + j]);
      minmax[j + 3] = std::max(minmax[j + 3], point[3 * i + j]);
    }
  }

  // construct pinned array
  int nnode = 0;
  if (doftype == mjFCOMPDOF_TRILINEAR || doftype == mjFCOMPDOF_QUADRATIC) {
    int order = doftype == mjFCOMPDOF_TRILINEAR ? 1 : 2;
    // multi-cell count for mesh/direct/gmsh, else single cell
    int cx = 1, cy = 1, cz = 1;
    if (type == mjFCOMPTYPE_MESH || type == mjFCOMPTYPE_DIRECT || type == mjFCOMPTYPE_GMSH) {
      if (cellcount[0] >= 0) {
        cx = cellcount[0];
        cy = cellcount[1];
        cz = cellcount[2];
      }
    }
    nnode = (cx * order + 1) * (cy * order + 1) * (cz * order + 1);
  }
  pinned = vector<bool>(std::max(npnt, nnode), rigid);

  // handle pins if user did not specify rigid
  if (!rigid) {
    // process pinid
    for (int i = 0; i < (int)pinid.size(); i++) {
      // check range
      if (pinid[i] < 0 || pinid[i] >= npnt) {
        return comperr(error, "pinid out of range", error_sz);
      }

      // set
      pinned[pinid[i]] = true;
    }

    // process pinrange
    for (int i = 0; i < (int)pinrange.size(); i += 2) {
      // check range
      if (pinrange[i] < 0 ||
          pinrange[i] >= npnt ||
          pinrange[i + 1] < 0 ||
          pinrange[i + 1] >= npnt) {
        return comperr(error, "pinrange out of range", error_sz);
      }

      // set
      for (int k = pinrange[i]; k <= pinrange[i + 1]; k++) { pinned[k] = true; }
    }

    // process pingrid
    for (int i = 0; i < (int)pingrid.size(); i += dflex->dim) {
      // check range
      int count_check[3] = {count[0], count[1], count[2]};
      if (type != mjFCOMPTYPE_GRID &&
          (doftype == mjFCOMPDOF_TRILINEAR || doftype == mjFCOMPDOF_QUADRATIC)) {
        int dim        = (doftype == mjFCOMPDOF_TRILINEAR) ? 2 : 3;
        count_check[0] = count_check[1] = count_check[2] = dim;
      }
      for (int k = 0; k < dflex->dim; k++) {
        if (pingrid[i + k] < 0 || pingrid[i + k] >= count_check[k]) {
          return comperr(error, "pingrid out of range", error_sz);
        }
      }

      // set
      if (dflex->dim == 2) {
        pinned[GridID(pingrid[i], pingrid[i + 1])] = true;
      } else if (dflex->dim == 3) {
        pinned[GridID(pingrid[i], pingrid[i + 1], pingrid[i + 2])] = true;
      }
    }

    // process pingridrange
    for (int i = 0; i < (int)pingridrange.size(); i += 2 * dflex->dim) {
      // check range
      for (int k = 0; k < 2 * dflex->dim; k++) {
        if (pingridrange[i + k] < 0 || pingridrange[i + k] >= count[k % dflex->dim]) {
          return comperr(error, "pingridrange out of range", error_sz);
        }
      }

      // set
      if (dflex->dim == 2) {
        for (int ix = pingridrange[i]; ix <= pingridrange[i + 2]; ix++) {
          for (int iy = pingridrange[i + 1]; iy <= pingridrange[i + 3]; iy++) {
            pinned[GridID(ix, iy)] = true;
          }
        }
      } else if (dflex->dim == 3) {
        for (int ix = pingridrange[i]; ix <= pingridrange[i + 3]; ix++) {
          for (int iy = pingridrange[i + 1]; iy <= pingridrange[i + 4]; iy++) {
            for (int iz = pingridrange[i + 2]; iz <= pingridrange[i + 5]; iz++) {
              pinned[GridID(ix, iy, iz)] = true;
            }
          }
        }
      }
    }

    // center of radial body is always pinned
    if (doftype == mjFCOMPDOF_RADIAL) { pinned[0] = true; }

    // check if all or none are pinned
    bool allpin = true, nopin = true;
    for (int i = 0; i < npnt; i++) {
      if (pinned[i]) {
        nopin = false;
      } else {
        allpin = false;
      }
    }

    // adjust rigid and centered
    if (allpin) {
      rigid = true;
    } else if (nopin) {
      centered = true;
    }
  }

  // remove unreferenced for direct, mesh, gmsh
  if (direct) {
    // find used
    used = std::vector<bool>(npnt, false);
    for (int i = 0; i < (int)element.size(); i++) { used[element[i]] = true; }

    // construct reindex
    bool             hasunused = false;
    std::vector<int> reindex(npnt, 0);
    for (int i = 0; i < npnt; i++) {
      if (!used[i]) {
        hasunused = true;
        for (int k = i + 1; k < npnt; k++) { reindex[k]--; }
      }
    }

    // reindex elements if unused present
    if (hasunused) {
      for (int i = 0; i < (int)element.size(); i++) { element[i] += reindex[element[i]]; }

      // compact point, texcoord, pinned arrays
      int new_npnt = 0;
      for (int i = 0; i < npnt; i++) {
        if (used[i]) {
          point[3 * new_npnt + 0] = point[3 * i + 0];
          point[3 * new_npnt + 1] = point[3 * i + 1];
          point[3 * new_npnt + 2] = point[3 * i + 2];

          if (!texcoord.empty()) {
            texcoord[2 * new_npnt + 0] = texcoord[2 * i + 0];
            texcoord[2 * new_npnt + 1] = texcoord[2 * i + 1];
          }

          pinned[new_npnt] = pinned[i];
          new_npnt++;
        }
      }

      // resize arrays
      point.resize(3 * new_npnt);
      if (!texcoord.empty()) { texcoord.resize(2 * new_npnt); }
      pinned.resize(std::max(new_npnt, nnode));
      used.assign(new_npnt, true);

      // update count
      npnt = new_npnt;
    }
  }

  // nothing to remove for auto-generated types
  else {
    used = std::vector<bool>(npnt, true);
  }

  // create flex, copy parameters
  mjCFlex* flex = model->AddFlex();
  mjsFlex* pf   = &flex->spec;
  int      id   = flex->id;

  *flex = def.Flex();
  flex->PointToLocal();

  flex->model = model;
  flex->id    = id;
  mjs_setName(pf->element, name.c_str());
  mjs_setInt(pf->elem, element.data(), element.size());
  mjs_setFloat(pf->texcoord, texcoord.data(), texcoord.size());
  mjs_setInt(pf->elemtexcoord, elemtexcoord.data(), elemtexcoord.size());
  if (!centered) { mjs_setDouble(pf->vert, point.data(), point.size()); }

  // rigid: set parent name, nothing else to do
  if (rigid) {
    mjs_appendString(pf->vertbody, mjs_getName(body->element)->c_str());
    return true;
  }

  // compute body mass and inertia matching specs
  double bodymass    = mass / npnt;
  double bodyinertia = bodymass * (2.0 * inertiabox * inertiabox) / 3.0;

  // overwrite plugin name
  if (plugin.active && plugin_instance_name.empty()) {
    plugin_instance_name                          = "flexcomp_" + name;
    static_cast<mjCPlugin*>(plugin.element)->name = plugin_instance_name;
  }

  // create bodies, construct flex vert and vertbody
  for (int i = 0; i < npnt; i++) {
    // not used: skip
    if (!used[i]) { continue; }

    // pinned or trilinear or quadratic: parent body
    if (pinned[i] || doftype == mjFCOMPDOF_TRILINEAR || doftype == mjFCOMPDOF_QUADRATIC) {
      mjs_appendString(pf->vertbody, mjs_getName(body->element)->c_str());

      // add plugin
      if (plugin.active) {
        mjsPlugin* pplugin = &body->plugin;
        pplugin->active    = true;
        pplugin->element   = static_cast<mjsElement*>(plugin.element);
        mjs_setString(pplugin->plugin_name, mjs_getString(plugin.plugin_name));
        mjs_setString(pplugin->name, plugin_instance_name.c_str());
      }
    }

    // not pinned and not trilinear: new body
    else {
      // add new body at vertex coordinates
      mjsBody* pb = mjs_addBody(body, 0);

      // set frame and inertial
      pb->pos[0] = point[3 * i];
      pb->pos[1] = point[3 * i + 1];
      pb->pos[2] = point[3 * i + 2];
      mjuu_zerovec(pb->ipos, 3);
      pb->mass             = bodymass;
      pb->inertia[0]       = bodyinertia;
      pb->inertia[1]       = bodyinertia;
      pb->inertia[2]       = bodyinertia;
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
        for (int j = 0; j < 3; j++) {
          // add joint to body
          mjsJoint* jnt = mjs_addJoint(pb, 0);

          // set properties
          jnt->type = mjJNT_SLIDE;
          mjuu_setvec(jnt->pos, 0, 0, 0);
          mjuu_setvec(jnt->axis, 0, 0, 0);
          jnt->axis[j] = 1;
        }
      }

      // add two orthogonal sliders (x and y only)
      else if (doftype == mjFCOMPDOF_2D) {
        for (int j = 0; j < 2; j++) {
          mjsJoint* jnt = mjs_addJoint(pb, 0);
          jnt->type     = mjJNT_SLIDE;
          mjuu_setvec(jnt->pos, 0, 0, 0);
          mjuu_setvec(jnt->axis, 0, 0, 0);
          jnt->axis[j] = 1;
        }
      }

      // construct body name, add to vertbody
      char txt[100];
      mju::sprintf_arr(txt, "%s_%d", name.c_str(), i);
      mjs_setName(pb->element, txt);
      mjs_appendString(pf->vertbody, mjs_getName(pb->element)->c_str());

      // clear flex vertex coordinates if allocated
      if (!centered) {
        point[3 * i]     = 0;
        point[3 * i + 1] = 0;
        point[3 * i + 2] = 0;
      }

      // add plugin
      if (plugin.active) {
        mjsPlugin* pplugin = &pb->plugin;
        pplugin->active    = true;
        pplugin->element   = static_cast<mjsElement*>(plugin.element);
        mjs_setString(pplugin->plugin_name, mjs_getString(plugin.plugin_name));
        mjs_setString(pplugin->name, plugin_instance_name.c_str());
      }
    }
  }

  // create nodal mesh for trilinear/quadratic interpolation
  if (doftype == mjFCOMPDOF_TRILINEAR || doftype == mjFCOMPDOF_QUADRATIC) {
    flex->spec.order = doftype == mjFCOMPDOF_TRILINEAR ? 1 : 2;

    if (cellcount[0] >= 0) {
      flex->spec.cellcount[0] = cellcount[0];
      flex->spec.cellcount[1] = cellcount[1];
      flex->spec.cellcount[2] = cellcount[2];
    }

    // total number of nodes with shared boundaries
    int nx    = flex->spec.cellcount[0] * flex->spec.order + 1;
    int ny    = flex->spec.cellcount[1] * flex->spec.order + 1;
    int nz    = flex->spec.cellcount[2] * flex->spec.order + 1;
    int nnode = nx * ny * nz;

    // mark empty cells and pin nodes exclusively in empty cells (volume mode only)
    if (!dflex->elastic2d) { MarkEmptyCells(flex, point.data(), minmax, nx, ny, nz); }

    // shell mode: pin all interior (non-boundary) nodes
    if (dflex->elastic2d) {
      for (int gi = 0; gi < nx; gi++) {
        for (int gj = 0; gj < ny; gj++) {
          for (int gk = 0; gk < nz; gk++) {
            bool is_boundary =
                (gi == 0 || gi == nx - 1 || gj == 0 || gj == ny - 1 || gk == 0 || gk == nz - 1);
            if (!is_boundary) { pinned[gi * ny * nz + gj * nz + gk] = true; }
          }
        }
      }
    }

    // if MarkEmptyCells pinned any nodes, force centered=false
    // so that pf->node (local positions) is saved to the model
    if (centered) {
      for (int i = 0; i < nnode; i++) {
        if (pinned[i]) {
          centered = false;
          break;
        }
      }
    }

    std::vector<double> node(3 * nnode, 0);
    int                 idx = 0;

    // Simpson's rule weights for quadratic mass distribution
    double massP2[3] = {1. / 6., 2. / 3., 1. / 6.};


    // collect created bodies for mass normalization
    std::vector<mjsBody*> node_bodies;

    for (int gi = 0; gi < nx; gi++) {
      for (int gj = 0; gj < ny; gj++) {
        for (int gk = 0; gk < nz; gk++) {
          // parametric position in [0, 1]^3
          double s = (double)gi / (flex->spec.cellcount[0] * flex->spec.order);
          double t = (double)gj / (flex->spec.cellcount[1] * flex->spec.order);
          double u = (double)gk / (flex->spec.cellcount[2] * flex->spec.order);

          // physical position
          double px = minmax[0] + s * (minmax[3] - minmax[0]);
          double py = minmax[1] + t * (minmax[4] - minmax[1]);
          double pz = minmax[2] + u * (minmax[5] - minmax[2]);

          if (pinned[idx]) {
            node[3 * idx + 0] = px;
            node[3 * idx + 1] = py;
            node[3 * idx + 2] = pz;
            mjs_appendString(pf->nodebody, mjs_getName(body->element)->c_str());
            idx++;
            continue;
          }

          mjsBody* pb = mjs_addBody(body, 0);
          pb->pos[0]  = px;
          pb->pos[1]  = py;
          pb->pos[2]  = pz;
          mjuu_zerovec(pb->ipos, 3);

          // mass distribution
          if (doftype == mjFCOMPDOF_TRILINEAR) {
            pb->mass = 1.0;
          } else {
            // local index within the cell for mass computation
            int li = gi % flex->spec.order;
            int lj = gj % flex->spec.order;
            int lk = gk % flex->spec.order;
            // boundary nodes: average mass contribution
            int ncells_i = (gi > 0 && gi < nx - 1 && li == 0) ? 2 : 1;
            int ncells_j = (gj > 0 && gj < ny - 1 && lj == 0) ? 2 : 1;
            int ncells_k = (gk > 0 && gk < nz - 1 && lk == 0) ? 2 : 1;
            // use Simpson weights
            double wi = massP2[li == 0 ? 0 : li];
            double wj = massP2[lj == 0 ? 0 : lj];
            double wk = massP2[lk == 0 ? 0 : lk];
            pb->mass  = wi * wj * wk * ncells_i * ncells_j * ncells_k;
          }

          node_bodies.push_back(pb);

          pb->inertia[0]       = pb->mass * (2.0 * inertiabox * inertiabox) / 3.0;
          pb->inertia[1]       = pb->mass * (2.0 * inertiabox * inertiabox) / 3.0;
          pb->inertia[2]       = pb->mass * (2.0 * inertiabox * inertiabox) / 3.0;
          pb->explicitinertial = true;

          for (int d = 0; d < 3; d++) {
            mjsJoint* jnt = mjs_addJoint(pb, 0);
            jnt->type     = mjJNT_SLIDE;
            mjuu_setvec(jnt->pos, 0, 0, 0);
            mjuu_setvec(jnt->axis, 0, 0, 0);
            jnt->axis[d] = 1;
          }

          // construct node name, add to nodebody
          char txt[100];
          mju::sprintf_arr(txt, "%s_%d_%d_%d", name.c_str(), gi, gj, gk);
          mjs_setName(pb->element, txt);
          mjs_appendString(pf->nodebody, mjs_getName(pb->element)->c_str());

          idx++;
        }
      }
    }

    // normalize masses so total equals prescribed mass
    double total_mass = 0;
    for (mjsBody* pb : node_bodies) { total_mass += pb->mass; }
    if (total_mass > 0) {
      double scale = mass / total_mass;
      for (mjsBody* pb : node_bodies) {
        pb->mass       *= scale;
        pb->inertia[0] *= scale;
        pb->inertia[1] *= scale;
        pb->inertia[2] *= scale;
      }
    }

    if (!centered) { mjs_setDouble(pf->node, node.data(), node.size()); }
  }

  if (!centered || doftype == mjFCOMPDOF_TRILINEAR || doftype == mjFCOMPDOF_QUADRATIC) {
    mjs_setDouble(pf->vert, point.data(), point.size());
  }

  // create equality constraints
  if (equality) {
    // equality 1=edge(mjEQ_FLEX), 2=vert(mjEQ_FLEXVERT), 3=strain(mjEQ_FLEXSTRAIN)
    if (equality == 1 || equality == 2) {
      mjsEquality* pe = mjs_addEquality(&model->spec, &def.spec);
      mjs_setDefault(pe->element, &model->Default()->spec);
      pe->type   = (equality == 1) ? mjEQ_FLEX : mjEQ_FLEXVERT;
      pe->active = true;
      mjs_setString(pe->name1, name.c_str());
    } else if (equality == 3) {
      // create one strain constraint per finite element, storing element index
      flex->has_strain_eq = true;
      int  cell_cx        = flex->spec.cellcount[0];
      int  cell_cy        = flex->spec.cellcount[1];
      int  cell_cz        = flex->spec.cellcount[2];
      bool shell          = (doftype == mjFCOMPDOF_TRILINEAR || doftype == mjFCOMPDOF_QUADRATIC) &&
                            flex->spec.elastic2d;

      if (shell) {
        // shell mode: one constraint per boundary face element
        int nelem_fe = 2 * (cell_cy * cell_cz + cell_cx * cell_cz + cell_cx * cell_cy);
        for (int fe = 0; fe < nelem_fe; fe++) {
          mjsEquality* pe = mjs_addEquality(&model->spec, &def.spec);
          mjs_setDefault(pe->element, &model->Default()->spec);
          pe->type   = mjEQ_FLEXSTRAIN;
          pe->active = true;
          mjs_setString(pe->name1, name.c_str());
          pe->data[0] = fe;
          pe->data[1] = -1;  // sentinel: shell mode
          pe->data[2] = -1;
        }
      } else {
        // volume mode: one constraint per 3D cell
        for (int ci = 0; ci < cell_cx; ci++) {
          for (int cj = 0; cj < cell_cy; cj++) {
            for (int ck = 0; ck < cell_cz; ck++) {
              // skip empty cells
              if (!flex->cell_empty.empty() &&
                  flex->cell_empty[ci * cell_cy * cell_cz + cj * cell_cz + ck]) {
                continue;
              }
              mjsEquality* pe = mjs_addEquality(&model->spec, &def.spec);
              mjs_setDefault(pe->element, &model->Default()->spec);
              pe->type   = mjEQ_FLEXSTRAIN;
              pe->active = true;
              mjs_setString(pe->name1, name.c_str());
              pe->data[0] = ci;
              pe->data[1] = cj;
              pe->data[2] = ck;
            }
          }
        }
      }
    }
  }

  return true;
}


// get point id from grid coordinates
int mjCFlexcomp::GridID(int ix, int iy) {
  return ix * count[1] + iy;
}
int mjCFlexcomp::GridID(int ix, int iy, int iz) {
  return ix * count[1] * count[2] + iy * count[2] + iz;
}


// append points, elements and texcoords of a procedural mesh spec; 1D meshes
// only have nodes, consecutive nodes are connected and closed loops wrap around
static void AppendMeshSpec(const mjsMesh&       mesh,
                           int                  dim,
                           bool                 closed,
                           std::vector<double>& point,
                           std::vector<int>&    element,
                           std::vector<float>&  texcoord) {
  point.insert(point.end(), mesh.usernode->begin(), mesh.usernode->end());
  texcoord.insert(texcoord.end(), mesh.usertexcoord->begin(), mesh.usertexcoord->end());
  if (dim == 1) {
    int n     = mesh.usernode->size() / 3;
    int nedge = closed ? n : n - 1;
    for (int i = 0; i < nedge; i++) {
      element.push_back(i);
      element.push_back((i + 1) % n);
    }
  } else {
    const std::vector<int>& elem = dim == 3 ? *mesh.usertet : *mesh.userface;
    element.insert(element.end(), elem.begin(), elem.end());
  }
}


// make grid
bool mjCFlexcomp::MakeGrid(char* error, int error_sz) {
  int  dim     = def.Flex().spec.dim;
  bool needtex = texcoord.empty() && mjs_getString(def.spec.flex->material)[0];
  bool circle  = dim == 1 && type == mjFCOMPTYPE_CIRCLE;

  mjCMesh mesh;
  if (circle) {
    mesh.MakeCircle(count, spacing);
  } else {
    mesh.MakeGrid(count, spacing, dim, needtex);
  }
  AppendMeshSpec(mesh.spec, dim, circle, point, element, texcoord);

  // check elements
  if (element.empty()) { return comperr(error, "No elements were created in grid", error_sz); }

  return true;
}


// make 2d square or disc
bool mjCFlexcomp::MakeSquare(char* error, int error_sz) {
  // set 2D
  def.spec.flex->dim = 2;
  bool needtex       = texcoord.empty() && mjs_getString(def.spec.flex->material)[0];

  mjCMesh mesh;
  if (type == mjFCOMPTYPE_DISC) {
    mesh.MakeDisc(count, spacing, needtex);
  } else {
    mesh.MakeGrid(count, spacing, 2, needtex);
  }
  AppendMeshSpec(mesh.spec, 2, /*closed=*/false, point, element, texcoord);

  // check elements
  if (element.empty()) { return comperr(error, "No elements were created in grid", error_sz); }

  return true;
}


// make 3d box, ellipsoid or cylinder
bool mjCFlexcomp::MakeBox(char* error, int error_sz, int dim, bool open) {
  bool needtex = texcoord.empty() && mjs_getString(def.spec.flex->material)[0];

  // set dimension
  def.spec.flex->dim = dim;

  mjCMesh mesh;
  if (type == mjFCOMPTYPE_CYLINDER) {
    mesh.MakeCylinder(count, spacing, dim, needtex, open);
  } else if (type == mjFCOMPTYPE_ELLIPSOID) {
    mesh.MakeEllipsoid(count, spacing, dim, needtex, open);
  } else {
    mesh.MakeBox(count, spacing, dim, needtex, open);
  }
  AppendMeshSpec(mesh.spec, dim, /*closed=*/false, point, element, texcoord);

  // check elements
  if (element.empty()) { return comperr(error, "No elements were created in box", error_sz); }

  return true;
}


// copied from user_mesh.cc
template <typename T>
static T* VecToArray(std::vector<T>& vector, bool clear = true) {
  if (vector.empty())
    return nullptr;
  else {
    int n    = (int)vector.size();
    T*  cvec = (T*)mju_malloc(n * sizeof(T));
    memcpy(cvec, vector.data(), n * sizeof(T));
    if (clear) { vector.clear(); }
    return cvec;
  }
}


// make mesh
bool mjCFlexcomp::MakeMesh(
    mjCModel* model, mjsCompiler* compiler, char* error, int error_sz, const mjVFS* vfs) {
  // strip path
  if (!file.empty() && model->spec.strippath) { file = mjuu_strippath(file); }

  // file is required
  if (file.empty()) { return comperr(error, "File is required", error_sz); }

  // check dim
  if (def.spec.flex->dim < 1) {
    return comperr(error, "Flex dim must be at least 1 for mesh", error_sz);
  }

  // load resource
  std::string filename = mjuu_combinePaths(mjs_getString(compiler->meshdir), file);
  mjResource* resource = nullptr;


  if (mjCMesh::IsMSH(filename)) {
    return comperr(error, "legacy MSH files are not supported in flexcomp", error_sz);
  }

  try {
    resource = mjCBase::LoadResource(mjs_getString(model->spec.modelfiledir), filename, vfs);
  } catch (mjCError err) { return comperr(error, err.message, error_sz); }


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
  point.assign(mesh.Vert().begin(), mesh.Vert().end());

  if (mesh.HasTexcoord()) {
    texcoord     = mesh.Texcoord();
    elemtexcoord = mesh.FaceTexcoord();
  }

  // copy faces or create 3D mesh
  if (def.spec.flex->dim == 2) {
    element = mesh.Face();
  } else if (def.spec.flex->dim == 1) {
    // extract edge pairs from degenerate triangles (i1, i2, i2)
    const std::vector<int>& face = mesh.Face();
    element.clear();
    element.reserve(face.size() * 2 / 3);
    for (size_t i = 0; i < face.size(); i += 3) {
      element.push_back(face[i]);
      element.push_back(face[i + 1]);
    }
  } else {
    point.insert(point.begin() + 0, origin[0]);
    point.insert(point.begin() + 1, origin[1]);
    point.insert(point.begin() + 2, origin[2]);
    for (int i = 0; i < mesh.Face().size(); i += 3) {
      // only add tetrahedra with positive volume
      int    tet[3] = {mesh.Face()[i + 0] + 1, mesh.Face()[i + 1] + 1, mesh.Face()[i + 2] + 1};
      double edge1[3], edge2[3], edge3[3];
      for (int i = 0; i < 3; i++) {
        edge1[i] = point[3 * tet[0] + i] - origin[i];
        edge2[i] = point[3 * tet[1] + i] - origin[i];
        edge3[i] = point[3 * tet[2] + i] - origin[i];
      }
      double normal[3];
      mjuu_crossvec(normal, edge1, edge2);
      if (mjuu_dot3(normal, edge3) < mjMINVAL) { continue; }
      element.push_back(0);
      element.push_back(tet[0]);
      element.push_back(tet[1]);
      element.push_back(tet[2]);
    }
  }

  return true;
}


// load points and elements from GMSH file
bool mjCFlexcomp::MakeGMSH(
    mjCModel* model, mjsCompiler* compiler, char* error, int error_sz, const mjVFS* vfs) {
  // strip path
  if (!file.empty() && model->spec.strippath) { file = mjuu_strippath(file); }

  // file is required
  if (file.empty()) { return comperr(error, "File is required", error_sz); }

  // open resource
  mjResource* resource = nullptr;
  try {
    std::string filename = mjuu_combinePaths(mjs_getString(compiler->meshdir), file);
    resource = mjCBase::LoadResource(mjs_getString(model->spec.modelfiledir), filename, vfs);
  } catch (mjCError err) { return comperr(error, err.message, error_sz); }

  const mjpDecoder* decoder = mjp_findDecoder(resource, "model/vnd.gmsh");
  if (!decoder) { decoder = mjp_findDecoder(resource, ""); }
  if (!decoder) {
    mju_closeResource(resource);
    return comperr(error, "no decoder found for GMSH file", error_sz);
  }

  mjSpec* spec              = nullptr;
  char    decode_error[500] = "";
  try {
    spec = decoder->decode(resource, vfs, decode_error, sizeof(decode_error));
    mju_closeResource(resource);
  } catch (...) {
    mju_closeResource(resource);
    return comperr(error, "exception while reading GMSH file", error_sz);
  }

  if (!spec) {
    return comperr(error, decode_error[0] ? decode_error : "failed to decode GMSH file", error_sz);
  }

  mjsElement* mesh_elem = mjs_firstElement(spec, mjOBJ_MESH);
  mjsMesh*    mesh      = mjs_asMesh(mesh_elem);
  if (!mesh) {
    mj_deleteSpec(spec);
    return comperr(error, "invalid GMSH spec decoded", error_sz);
  }

  int mesh_dim = 0;
  if (mesh->usertet && !mesh->usertet->empty()) {
    mesh_dim = 3;
  } else if (mesh->userface && !mesh->userface->empty()) {
    mesh_dim = 2;
  }

  if (mesh_dim == 0) {
    mj_deleteSpec(spec);
    return comperr(error, "unsupported GMSH mesh dimensionality", error_sz);
  }

  if (has_dim && def.spec.flex->dim != mesh_dim) {
    mj_deleteSpec(spec);
    return comperr(error, "flexcomp dim does not match GMSH mesh dimensionality", error_sz);
  }
  def.spec.flex->dim = mesh_dim;

  const mjDoubleVec* nodes = mesh->usernode;
  if (!nodes || nodes->empty()) {
    mj_deleteSpec(spec);
    return comperr(error, "GMSH mesh has no nodes", error_sz);
  }
  point.assign(nodes->begin(), nodes->end());

  if (mesh_dim == 3) {
    element.assign(mesh->usertet->begin(), mesh->usertet->end());
  } else {
    element.assign(mesh->userface->begin(), mesh->userface->end());
  }

  mj_deleteSpec(spec);

  return true;
}
