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

#include "user/user_composite.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mjmodel.h>
#include "cc/array_safety.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_util.h"

namespace {
namespace mju = ::mujoco::util;
using std::vector;
using std::string;
}  // namespace

// strncpy with 0, return false
static bool comperr(char* error, const char* msg, int error_sz) {
  mju_strncpy(error, msg, error_sz);
  return false;
}



// constructor
mjCComposite::mjCComposite(void) {
  // common properties
  prefix.clear();
  type = mjCOMPTYPE_PARTICLE;
  count[0] = count[1] = count[2] = 1;
  spacing = 0;
  mjuu_setvec(offset, 0, 0, 0);
  pin.clear();
  flatinertia = 0;
  mj_defaultSolRefImp(solrefsmooth, solimpsmooth);

  // skin
  skin = false;
  skintexcoord = false;
  skinmaterial.clear();
  mjuu_setvec(skinrgba, 1, 1, 1, 1);
  skininflate = 0;
  skinsubgrid = 0;
  skingroup = 0;

  // clear add flags
  for (int i=0; i<mjNCOMPKINDS; i++) {
    add[i] = false;
  }

  // clear internal
  dim = 0;
}



// adjust constraint softness
void mjCComposite::AdjustSoft(mjtNum* solref, mjtNum* solimp, int level) {
  switch (level) {
  case 0:
    solref[0] = 0.01;
    solimp[0] = solimp[1] = 0.99;
    break;

  case 1:
    solref[0] = 0.02;
    solimp[0] = solimp[1] = 0.9;
    break;
  }
}



// set defaults, after reading top-level info and skin
void mjCComposite::SetDefault(void) {
  int i;

  // determine dimensionality
  int tmpdim = 0;
  for (i=0; i<3; i++) {
    if (count[i]>1) {
      tmpdim++;
    }
  }

  // set all deafult groups to 3
  for (int i=0; i<mjNCOMPKINDS; i++) {
    def[i].geom.group = 3;
    def[i].site.group = 3;
    def[i].joint.group = 3;
    def[i].tendon.group = 3;
  }

  // set default geom and tendon group to 0 if needed to be visible
  if (!skin ||
      type==mjCOMPTYPE_PARTICLE   ||
      type==mjCOMPTYPE_ROPE       ||
      type==mjCOMPTYPE_LOOP       ||
      (type==mjCOMPTYPE_GRID && tmpdim==1)) {
    for (i=0; i<mjNCOMPKINDS; i++) {
      def[i].geom.group = 0;
      def[i].tendon.group = 0;
    }
  }

  // other type-specific adjustments
  switch (type) {
  case mjCOMPTYPE_PARTICLE:       // particle

    // no friction with anything
    def[0].geom.condim = 1;
    def[0].geom.priority = 1;
    break;

  case mjCOMPTYPE_GRID:           // grid

    // hard main tendon fix
    AdjustSoft(def[mjCOMPKIND_TENDON].equality.solref,
               def[mjCOMPKIND_TENDON].equality.solimp, 0);

    break;

  case mjCOMPTYPE_ROPE:           // rope
    break;

  case mjCOMPTYPE_LOOP:           // loop

    // hard smoothing
    AdjustSoft(solrefsmooth, solimpsmooth, 0);
    break;

  case mjCOMPTYPE_CLOTH:          // cloth
    break;

  case mjCOMPTYPE_BOX:            // 3D
  case mjCOMPTYPE_CYLINDER:
  case mjCOMPTYPE_ELLIPSOID:

    // no self-collisions
    def[0].geom.contype = 0;

    // soft smoothing
    AdjustSoft(solrefsmooth, solimpsmooth, 1);

    // soft fix everywhere
    for (i=0; i<mjNCOMPKINDS; i++) {
      AdjustSoft(def[i].equality.solref, def[i].equality.solimp, 1);
    }

    // hard main tendon fix
    AdjustSoft(def[mjCOMPKIND_TENDON].equality.solref,
               def[mjCOMPKIND_TENDON].equality.solimp, 0);
    break;
  default:
    // SHOULD NOT OCCUR
    mju_error_i("Invalid composite type: %d", type);
    break;
  }
}



// make composite object
bool mjCComposite::Make(mjCModel* model, mjCBody* body, char* error, int error_sz) {
  // require local coordinates
  if (model->global) {
    return comperr(error, "Composite requires local coordinates", error_sz);
  }

  // check geom type
  if (def[0].geom.type!=mjGEOM_SPHERE &&
      def[0].geom.type!=mjGEOM_CAPSULE &&
      def[0].geom.type!=mjGEOM_ELLIPSOID) {
    return comperr(error, "Composite geom type must be sphere, capsule or ellipsoid", error_sz);
  }

  // check pin coord number
  if (pin.size()%2) {
    return comperr(error, "Pin coordinate number of must be multiple of 2", error_sz);
  }

  // check counts
  for (int i=0; i<3; i++) {
    if (count[i]<1) {
      return comperr(error, "Positive counts expected in composite", error_sz);
    }
  }

  // check spacing
  if (spacing<mjMINVAL) {
    return comperr(error, "Positive spacing expected in composite", error_sz);
  }

  // determine dimensionality, check singleton order
  bool first = false;
  for (int i=0; i<3; i++) {
    if (count[i]==1) {
      first = true;
    } else {
      dim++;
      if (first) {
        return comperr(error, "Singleton counts must come last", error_sz);
      }
    }
  }

  // require 3x3 for subgrid
  if (skin && skinsubgrid>0) {
    if (count[0]<3 || count[1]<3) {
      return comperr(error, "At least 3x3 required for skin subgrid", error_sz);
    }
  }

  // dispatch
  switch (type) {
  case mjCOMPTYPE_PARTICLE:
    return MakeParticle(model, body, error, error_sz);

  case mjCOMPTYPE_GRID:
    return MakeGrid(model, body, error, error_sz);

  case mjCOMPTYPE_ROPE:
  case mjCOMPTYPE_LOOP:
    return MakeRope(model, body, error, error_sz);

  case mjCOMPTYPE_CLOTH:
    return MakeCloth(model, body, error, error_sz);

  case mjCOMPTYPE_BOX:
  case mjCOMPTYPE_CYLINDER:
  case mjCOMPTYPE_ELLIPSOID:
    return MakeBox(model, body, error, error_sz);

  default:
    return comperr(error, "Uknown shape in composite", error_sz);
  }
}



// make particles
bool mjCComposite::MakeParticle(mjCModel* model, mjCBody* body, char* error, int error_sz) {
  // create bodies and geoms
  for (int ix=0; ix<count[0]; ix++) {
    for (int iy=0; iy<count[1]; iy++) {
      for (int iz=0; iz<count[2]; iz++) {
        // create body
        mjCBody* b = body->AddBody(NULL);

        // set body position
        b->pos[0] = offset[0] + spacing*(ix - 0.5*count[0]);
        b->pos[1] = offset[1] + spacing*(iy - 0.5*count[1]);
        b->pos[2] = offset[2] + spacing*(iz - 0.5*count[2]);

        // add slider joints
        for (int i=0; i<3; i++) {
          mjCJoint* jnt = b->AddJoint(def + mjCOMPKIND_JOINT, false);
          jnt->def = body->def;
          jnt->type = mjJNT_SLIDE;
          mjuu_setvec(jnt->pos, 0, 0, 0);
          mjuu_setvec(jnt->axis, 0, 0, 0);
          jnt->axis[i] = 1;
        }

        // add geom
        mjCGeom* g = b->AddGeom(def);
        g->def = body->def;
        g->type = mjGEOM_SPHERE;
      }
    }
  }

  return true;
}



// make grid connected with tendons
bool mjCComposite::MakeGrid(mjCModel* model, mjCBody* body, char* error, int error_sz) {
  char txt[100], txt1[100], txt2[100];

  // check dimensionality
  if (dim>2) {
    return comperr(error, "Grid can only be 1D or 2D", error_sz);
  }

  // check shear dimensionality
  if (add[mjCOMPKIND_SHEAR] && dim!=2) {
    return comperr(error, "Shear requires 2D grid", error_sz);
  }

  // check skin dimensionality
  if (skin && dim!=2) {
    return comperr(error, "Skin requires 2D grid", error_sz);
  }

  // create bodies, joints, geoms, sites
  for (int ix=0; ix<count[0]; ix++) {
    for (int iy=0; iy<count[1]; iy++) {
      // create body
      mjCBody* b = body->AddBody(NULL);
      mju::sprintf_arr(txt, "%sB%d_%d", prefix.c_str(), ix, iy);
      b->name = txt;

      // set body position
      b->pos[0] = offset[0] + spacing*(ix - 0.5*count[0]);
      b->pos[1] = offset[1] + spacing*(iy - 0.5*count[1]);
      b->pos[2] = offset[2];

      // add geom
      mjCGeom* g = b->AddGeom(def);
      g->def = body->def;
      g->type = mjGEOM_SPHERE;
      mju::sprintf_arr(txt, "%sG%d_%d", prefix.c_str(), ix, iy);
      g->name = txt;

      // add site
      mjCSite* s = b->AddSite(def);
      s->def = body->def;
      s->type = mjGEOM_SPHERE;
      mju::sprintf_arr(txt, "%sS%d_%d", prefix.c_str(), ix, iy);
      s->name = txt;

      // skip pinned elements
      bool skip = false;
      for (int ip=0; ip<pin.size(); ip+=2) {
        if (pin[ip]==ix && pin[ip+1]==iy) {
          skip = true;
          break;
        }
      }
      if (skip) {
        continue;
      }

      // add slider joint
      mjCJoint* jnt[3];
      for (int i=0; i<3; i++) {
        jnt[i] = b->AddJoint(def + mjCOMPKIND_JOINT);
        jnt[i]->def = body->def;
        mju::sprintf_arr(txt, "%sJ%d_%d_%d", prefix.c_str(), i, ix, iy);
        jnt[i]->name = txt;
        jnt[i]->type = mjJNT_SLIDE;
        mjuu_setvec(jnt[i]->pos, 0, 0, 0);
        mjuu_setvec(jnt[i]->axis, 0, 0, 0);
        jnt[i]->axis[i] = 1;
      }
    }
  }

  // create tendons and equality constraints
  for (int i=0; i<2; i++) {
    for (int ix=0; ix<count[0]-(i==0); ix++) {
      for (int iy=0; iy<count[1]-(i==1); iy++) {
        // recover site names
        mju::sprintf_arr(txt1, "%sS%d_%d", prefix.c_str(), ix, iy);
        mju::sprintf_arr(txt2, "%sS%d_%d", prefix.c_str(), ix+(i==0), iy+(i==1));

        // create tendon
        mjCTendon* ten = model->AddTendon(def + mjCOMPKIND_TENDON);
        ten->def = model->defaults[0];
        mju::sprintf_arr(txt, "%sT%d_%d_%d", prefix.c_str(), i, ix, iy);
        ten->name = txt;
        ten->WrapSite(txt1);
        ten->WrapSite(txt2);

        // add equality constraint
        mjCEquality* eq = model->AddEquality(def + mjCOMPKIND_TENDON);
        eq->def = model->defaults[0];
        eq->type = mjEQ_TENDON;
        eq->name1 = ten->name;
      }
    }
  }

  // shear for 2D
  if (add[mjCOMPKIND_SHEAR]) {
    MakeShear(model);
  }

  // skin
  if (skin) {
    if (skinsubgrid>0) {
      MakeSkin2Subgrid(model);
    } else {
      MakeSkin2(model);
    }
  }

  return true;
}



// make rope
bool mjCComposite::MakeRope(mjCModel* model, mjCBody* body, char* error, int error_sz) {
  // check dim
  if (dim!=1) {
    return comperr(error, "Rope must be one-dimensional", error_sz);
  }

  // check root body name prefix
  char txt[200];
  mju::sprintf_arr(txt, "%sB", prefix.c_str());
  if (std::strncmp(txt, body->name.substr(0, strlen(txt)).c_str(), mju::sizeof_arr(txt))) {
    mju::strcat_arr(txt, " must be the beginning of root body name");
    return comperr(error, txt, error_sz);
  }

  // read origin coordinate from root body
  mju::strcpy_arr(txt, body->name.substr(strlen(txt)).c_str());
  int ox = -1;
  if (sscanf(txt, "%d", &ox)!=1) {
    return comperr(error, "Root body name must contain X coordinate", error_sz);
  }
  if (ox<0 || ox>=count[0]) {
    return comperr(error, "Root body coordinate out of range", error_sz);
  }

  // add origin
  AddRopeBody(model, body, ox, ox);

  // add elements: right
  mjCBody* pbody = body;
  for (int ix=ox; ix<count[0]-1; ix++) {
    pbody = AddRopeBody(model, pbody, ix, ix+1);
  }

  // add elements: left
  pbody = body;
  for (int ix=ox; ix>0; ix--) {
    pbody = AddRopeBody(model, pbody, ix, ix-1);
  }

  // close loop
  if (type==mjCOMPTYPE_LOOP) {
    char txt2[200];

    // add equality constraint
    mjCEquality* eq = model->AddEquality();
    eq->type = mjEQ_CONNECT;
    mju::sprintf_arr(txt, "%sB0", prefix.c_str());
    mju::sprintf_arr(txt2, "%sB%d", prefix.c_str(), count[0]-1);
    eq->name1 = txt;
    eq->name2 = txt2;
    mjuu_setvec(eq->data, -0.5*spacing, 0, 0);
    mju_copy(eq->solref, solrefsmooth, mjNREF);
    mju_copy(eq->solimp, solimpsmooth, mjNIMP);

    // remove contact between connected bodies
    mjCBodyPair* pair = model->AddExclude();
    pair->bodyname1 = txt;
    pair->bodyname2 = txt2;
  }

  return true;
}



// add child body for cloth
mjCBody* mjCComposite::AddRopeBody(mjCModel* model, mjCBody* body, int ix, int ix1) {
  char txt[100];
  bool isroot = (ix==ix1);
  double dx = spacing*(ix1-ix);

  // add child if not root
  if (!isroot) {
    body = body->AddBody();
    mju::sprintf_arr(txt, "%sB%d", prefix.c_str(), ix1);
    body->name = txt;

    // loop
    if (type==mjCOMPTYPE_LOOP) {
      double alpha = 2*mjPI/count[0];
      double R = 0.5*spacing*sin(mjPI-alpha)/sin(0.5*alpha);

      if (ix1>ix) {
        mjuu_setvec(body->pos, R*cos(0.5*alpha), R*sin(0.5*alpha), 0);
        mjuu_setvec(body->quat, cos(0.5*alpha), 0, 0, sin(0.5*alpha));
      } else {
        mjuu_setvec(body->pos, -R*cos(0.5*alpha), R*sin(0.5*alpha), 0);
        mjuu_setvec(body->quat, cos(-0.5*alpha), 0, 0, sin(-0.5*alpha));
      }
    }

    // no loop
    else {
      mjuu_setvec(body->pos, dx, 0, 0);
    }
  }

  // add geom
  mjCGeom* geom = body->AddGeom(def);
  geom->def = body->def;
  mju::sprintf_arr(txt, "%sG%d", prefix.c_str(), ix1);
  geom->name = txt;
  mjuu_setvec(geom->pos, 0, 0, 0);
  mjuu_setvec(geom->quat, sqrt(0.5), 0, sqrt(0.5), 0);

  // root: no joints
  if (isroot) {
    return body;
  }

  // add main joint
  for (int i=0; i<2; i++) {
    // add joint
    mjCJoint* jnt = body->AddJoint(def + mjCOMPKIND_JOINT);
    jnt->def = body->def;
    mju::sprintf_arr(txt, "%sJ%d_%d", prefix.c_str(), i, ix1);
    jnt->name = txt;
    jnt->type = mjJNT_HINGE;
    mjuu_setvec(jnt->pos, -0.5*dx, 0, 0);
    mjuu_setvec(jnt->axis, 0, 0, 0);
    jnt->axis[i+1] = 1;
  }

  // add twist joint
  if (add[mjCOMPKIND_TWIST]) {
    // add joint
    mjCJoint* jnt = body->AddJoint(def + mjCOMPKIND_TWIST);
    jnt->def = body->def;
    mju::sprintf_arr(txt, "%sJT%d", prefix.c_str(), ix1);
    jnt->name = txt;
    jnt->type = mjJNT_HINGE;
    mjuu_setvec(jnt->pos, -0.5*dx, 0, 0);
    mjuu_setvec(jnt->axis, 1, 0, 0);

    // add constraint
    mjCEquality* eq = model->AddEquality(def + mjCOMPKIND_TWIST);
    eq->def = model->defaults[0];
    eq->type = mjEQ_JOINT;
    eq->name1 = jnt->name;
  }

  // add stretch joint
  if (add[mjCOMPKIND_STRETCH]) {
    // add joint
    mjCJoint* jnt = body->AddJoint(def + mjCOMPKIND_STRETCH);
    jnt->def = body->def;
    mju::sprintf_arr(txt, "%sJS%d", prefix.c_str(), ix1);
    jnt->name = txt;
    jnt->type = mjJNT_SLIDE;
    mjuu_setvec(jnt->pos, -0.5*dx, 0, 0);
    mjuu_setvec(jnt->axis, 1, 0, 0);

    // add constraint
    mjCEquality* eq = model->AddEquality(def + mjCOMPKIND_STRETCH);
    eq->def = model->defaults[0];
    eq->type = mjEQ_JOINT;
    eq->name1 = jnt->name;
  }

  return body;
}



// make 2d cloth
bool mjCComposite::MakeCloth(mjCModel* model, mjCBody* body, char* error, int error_sz) {
  // check dim
  if (dim!=2) {
    return comperr(error, "Cloth must be two-dimensional", error_sz);
  }

  // check root body name prefix
  char txt[200];
  mju::sprintf_arr(txt, "%sB", prefix.c_str());
  if (std::strncmp(txt, body->name.substr(0, strlen(txt)).c_str(), mju::sizeof_arr(txt))) {
    mju::strcat_arr(txt, " must be the beginning of root body name");
    return comperr(error, txt, error_sz);
  }

  // read origin coordinate from root body
  mju::strcpy_arr(txt, body->name.substr(strlen(txt)).c_str());
  int ox = -1, oy = -1;
  char c;
  if (sscanf(txt, "%d%c%d", &ox, &c, &oy)!=3 || c!='_') {
    return comperr(error, "Root body name must contain X_Y coordinates", error_sz);
  }
  if (ox<0 || ox>=count[0] || oy<0 || oy>=count[1]) {
    return comperr(error, "Root body coordinates out of range", error_sz);
  }

  // add origin
  AddClothBody(model, body, ox, oy, ox, oy);

  // add row: both directions
  for (int i=-2; i<=2; i+=2) {
    // init at origin
    int ix = ox;

    // process row to end
    mjCBody* pbodyx = body;
    while (1) {
      // candidate neighbor
      int ix1 = ix;
      if (i==0) {
        ix1 = mjMIN(count[0]-1, ix+1);
      } else if (i==2) {
        ix1 = mjMAX(0, ix-1);
      }

      // regular column
      if (i!=-2) {
        // stop if same
        if (ix1==ix) {
          break;
        }

        // add body, assign counter
        pbodyx = AddClothBody(model, pbodyx, ix, oy, ix1, oy);
        ix = ix1;
      }

      // add column: regular or center
      for (int j=1; j<=3; j+=2) {
        // init at row
        int iy = oy;

        // process column to end
        mjCBody* pbodyy = pbodyx;
        while (1) {
          // candidate neighbor
          int iy1;
          if (j==1) {
            iy1 = mjMIN(count[1]-1, iy+1);
          } else {
            iy1 = mjMAX(0, iy-1);
          }

          // stop if same
          if (iy1==iy) {
            break;
          }

          // add body, assign counter
          pbodyy = AddClothBody(model, pbodyy, ix, iy, ix, iy1);
          iy = iy1;
        }
      }

      // center column added
      if (i==-2) {
        break;
      }
    }
  }

  // add main tendons
  for (int iy=0; iy<count[1]; iy++) {
    if (iy!=oy) {
      for (int ix=0; ix<count[0]-1; ix++) {
        char txt1[100], txt2[100];

        // recover site names
        mju::sprintf_arr(txt1, "%sS%d_%d", prefix.c_str(), ix, iy);
        mju::sprintf_arr(txt2, "%sS%d_%d", prefix.c_str(), ix+1, iy);

        // create tendon
        mjCTendon* ten = model->AddTendon(def + mjCOMPKIND_TENDON);
        ten->def = model->defaults[0];
        mju::sprintf_arr(txt, "%sT%d_%d", prefix.c_str(), ix, iy);
        ten->name = txt;
        ten->WrapSite(txt1);
        ten->WrapSite(txt2);

        // add equality constraint
        mjCEquality* eq = model->AddEquality(def + mjCOMPKIND_TENDON);
        eq->def = model->defaults[0];
        eq->type = mjEQ_TENDON;
        eq->name1 = ten->name;
      }
    }
  }

  // shear for 2D
  if (add[mjCOMPKIND_SHEAR]) {
    MakeShear(model);
  }

  // skin
  if (skin) {
    if (skinsubgrid>0) {
      MakeSkin2Subgrid(model);
    } else {
      MakeSkin2(model);
    }
  }

  return true;
}



// add child body for cloth
mjCBody* mjCComposite::AddClothBody(mjCModel* model, mjCBody* body,
                                    int ix, int iy, int ix1, int iy1) {
  char txt[100];
  bool isroot = (ix==ix1 && iy==iy1);
  double dx = spacing*(ix1-ix);
  double dy = spacing*(iy1-iy);

  // add child if not root
  if (!isroot) {
    body = body->AddBody();
    mju::sprintf_arr(txt, "%sB%d_%d", prefix.c_str(), ix1, iy1);
    body->name = txt;
    mjuu_setvec(body->pos, dx, dy, 0);
  }

  // add geom, alternating rotations
  mjCGeom* geom = body->AddGeom(def);
  geom->def = body->def;
  mju::sprintf_arr(txt, "%sG%d_%d", prefix.c_str(), ix1, iy1);
  geom->name = txt;
  mjuu_setvec(geom->pos, 0, 0, 0);
  if ((ix1+iy1)%2) {
    mjuu_setvec(geom->quat, sqrt(0.5), sqrt(0.5), 0, 0);
  } else {
    mjuu_setvec(geom->quat, 0.5, 0.5, 0.5, 0.5);
  }

  // make inertia flat
  if (flatinertia>0) {
    // set body mass = geom mass
    body->mass = mjuu_defined(geom->_mass) ? geom->_mass :
                 geom->density * geom->GetVolume();

    // set body i-frame
    mjuu_setvec(body->ipos, 0, 0, 0);
    mjuu_setvec(body->iquat, 1, 0, 0, 0);

    // desired equivalent inertia box
    double size[3] = {0.5*spacing, 0.5*spacing, 0.5*spacing*flatinertia};

    // convert to inertia
    body->inertia[0] = body->mass*(size[1]*size[1]+size[2]*size[2])/3;
    body->inertia[1] = body->mass*(size[0]*size[0]+size[2]*size[2])/3;
    body->inertia[2] = body->mass*(size[0]*size[0]+size[1]*size[1])/3;

    body->MakeInertialExplicit();
  }

  // add site
  mjCSite* site = body->AddSite(def);
  site->def = body->def;
  site->type = mjGEOM_SPHERE;
  mju::sprintf_arr(txt, "%sS%d_%d", prefix.c_str(), ix1, iy1);
  site->name = txt;
  mjuu_setvec(site->pos, 0, 0, 0);
  mjuu_setvec(site->quat, 1, 0, 0, 0);

  // root: no joints
  if (isroot) {
    return body;
  }

  // add main joint
  for (int i=0; i<2; i++) {
    // add joint
    mjCJoint* jnt = body->AddJoint(def + mjCOMPKIND_JOINT);
    jnt->def = body->def;
    mju::sprintf_arr(txt, "%sJ%d_%d_%d", prefix.c_str(), i, ix1, iy1);
    jnt->name = txt;
    jnt->type = mjJNT_HINGE;
    mjuu_setvec(jnt->pos, -dx, -dy, 0);

    // set universal axis depending on parent direction
    mjuu_setvec(jnt->axis, 0, 0, 0);
    if (ix!=ix1) {
      jnt->axis[i+1] = 1;
    } else {
      jnt->axis[2*i] = 1;
    }
  }

  // add twist joint
  if (add[mjCOMPKIND_TWIST]) {
    mjCJoint* jnt = body->AddJoint(def + mjCOMPKIND_TWIST);
    jnt->def = body->def;
    mju::sprintf_arr(txt, "%sJT%d_%d", prefix.c_str(), ix1, iy1);
    jnt->name = txt;
    jnt->type = mjJNT_HINGE;
    mjuu_setvec(jnt->pos, -dx, -dy, 0);
    if (ix!=ix1) {
      mjuu_setvec(jnt->axis, 1, 0, 0);
    } else {
      mjuu_setvec(jnt->axis, 0, 1, 0);
    }

    // add constraint
    mjCEquality* eq = model->AddEquality(def + mjCOMPKIND_TWIST);
    eq->def = model->defaults[0];
    eq->type = mjEQ_JOINT;
    eq->name1 = jnt->name;
  }

  // add stretch joint
  if (add[mjCOMPKIND_STRETCH]) {
    // add joint
    mjCJoint* jnt = body->AddJoint(def + mjCOMPKIND_STRETCH);
    jnt->def = body->def;
    mju::sprintf_arr(txt, "%sJS%d_%d", prefix.c_str(), ix1, iy1);
    jnt->name = txt;
    jnt->type = mjJNT_SLIDE;
    mjuu_setvec(jnt->pos, -dx, -dy, 0);
    if (ix!=ix1) {
      mjuu_setvec(jnt->axis, 1, 0, 0);
    } else {
      mjuu_setvec(jnt->axis, 0, 1, 0);
    }

    // add constraint
    mjCEquality* eq = model->AddEquality(def + mjCOMPKIND_STRETCH);
    eq->def = model->defaults[0];
    eq->type = mjEQ_JOINT;
    eq->name1 = jnt->name;
  }

  return body;
}



// project from box to other shape
void mjCComposite::BoxProject(double* pos) {
  // determine sizes
  double size[3] = {
    0.5*spacing*(count[0]-1),
    0.5*spacing*(count[1]-1),
    0.5*spacing*(count[2]-1)
  };

  // box
  if (type==mjCOMPTYPE_BOX) {
    pos[0] *= size[0];
    pos[1] *= size[1];
    pos[2] *= size[2];
  }

  // cylinder
  else if (type==mjCOMPTYPE_CYLINDER) {
    double L0 = mjMAX(mju_abs(pos[0]), mju_abs(pos[1]));
    mjuu_normvec(pos, 2);
    pos[0] *= size[0]*L0;
    pos[1] *= size[1]*L0;
    pos[2] *= size[2];
  }

  // ellipsoid
  else if (type==mjCOMPTYPE_ELLIPSOID) {
    mjuu_normvec(pos, 3);
    pos[0] *= size[0];
    pos[1] *= size[1];
    pos[2] *= size[2];
  }
}



// make 3d box, ellipsoid or cylinder
bool mjCComposite::MakeBox(mjCModel* model, mjCBody* body, char* error, int error_sz) {
  int ix, iy, iz;
  char txt[100];

  // check dim
  if (dim!=3) {
    return comperr(error, "Box and ellipsoid must be three-dimensional", error_sz);
  }

  // center geom: two times bigger
  mjCGeom* geom = body->AddGeom(def);
  geom->def = body->def;
  geom->type = mjGEOM_SPHERE;
  mju::sprintf_arr(txt, "%sGcenter", prefix.c_str());
  geom->name = txt;
  mjuu_setvec(geom->pos, 0, 0, 0);
  geom->size[0] *= 2;
  geom->size[1] = 0;
  geom->size[2] = 0;

  // fixed tendon for all joints
  mjCTendon* ten = model->AddTendon(def + mjCOMPKIND_TENDON);
  ten->def = model->defaults[0];
  mju::sprintf_arr(txt, "%sT", prefix.c_str());
  ten->name = txt;

  // create bodies, geoms and joints: outside shell only
  for (ix=0; ix<count[0]; ix++) {
    for (iy=0; iy<count[1]; iy++) {
      for (iz=0; iz<count[2]; iz++) {
        if (ix==0 || ix==count[0]-1 ||
            iy==0 || iy==count[1]-1 ||
            iz==0 || iz==count[2]-1) {
          // create body
          mjCBody* b = body->AddBody(NULL);
          mju::sprintf_arr(txt, "%sB%d_%d_%d", prefix.c_str(), ix, iy, iz);
          b->name = txt;

          // set body position (+/- 1)
          b->pos[0] = 2.0*ix/(count[0]-1) - 1;
          b->pos[1] = 2.0*iy/(count[1]-1) - 1;
          b->pos[2] = 2.0*iz/(count[2]-1) - 1;

          // reshape
          BoxProject(b->pos);

          // reorient body
          mjuu_copyvec(b->alt.zaxis, b->pos, 3);
          mjuu_normvec(b->alt.zaxis, 3);

          // add geom
          mjCGeom* g = b->AddGeom(def);
          g->def = body->def;
          mju::sprintf_arr(txt, "%sG%d_%d_%d", prefix.c_str(), ix, iy, iz);
          g->name = txt;

          // offset inwards, enforce sphere or capsule
          if (g->type==mjGEOM_CAPSULE) {
            g->pos[2] = -(g->size[0] + g->size[1]);
          } else {
            g->type = mjGEOM_SPHERE;
            g->pos[2] = -g->size[0];
          }

          // add slider joint
          mjCJoint* jnt = b->AddJoint(def + mjCOMPKIND_JOINT);
          jnt->def = body->def;
          mju::sprintf_arr(txt, "%sJ%d_%d_%d", prefix.c_str(), ix, iy, iz);
          jnt->name = txt;
          jnt->type = mjJNT_SLIDE;
          mjuu_setvec(jnt->pos, 0, 0, 0);
          mjuu_setvec(jnt->axis, 0, 0, 1);

          // add fix constraint
          mjCEquality* eq = model->AddEquality(def + mjCOMPKIND_JOINT);
          eq->def = model->defaults[0];
          eq->type = mjEQ_JOINT;
          eq->name1 = jnt->name;

          // add joint to tendon
          ten->WrapJoint(jnt->name, 1);

          // add neighbor constraints
          for (int i=0; i<3; i++) {
            int ix1 = mjMIN(ix+(i==0), count[0]-1);
            int iy1 = mjMIN(iy+(i==1), count[1]-1);
            int iz1 = mjMIN(iz+(i==2), count[2]-1);
            if ((ix1==0 || ix1==count[0]-1 ||
                 iy1==0 || iy1==count[1]-1 ||
                 iz1==0 || iz1==count[2]-1) &&
                (ix!=ix1 || iy!=iy1 || iz!=iz1)) {
              char txt2[200];
              mju::sprintf_arr(txt2,
                               "%sJ%d_%d_%d", prefix.c_str(), ix1, iy1, iz1);
              mjCEquality* eqn = model->AddEquality();
              mju_copy(eqn->solref, solrefsmooth, mjNREF);
              mju_copy(eqn->solimp, solimpsmooth, mjNIMP);
              eqn->type = mjEQ_JOINT;
              eqn->name1 = txt;
              eqn->name2 = txt2;
            }
          }
        }
      }
    }
  }

  // finalize fixed tendon
  mjCEquality* eqt = model->AddEquality(def + mjCOMPKIND_TENDON);
  eqt->def = model->defaults[0];
  eqt->type = mjEQ_TENDON;
  eqt->name1 = ten->name;

  // skin
  if (skin) {
    MakeSkin3(model);
  }

  return true;
}



// add shear tendons to 2D
void mjCComposite::MakeShear(mjCModel* model) {
  int ix, iy;
  char txt[100], txt1[100], txt2[100];

  for (ix=0; ix<count[0]-1; ix++) {
    for (iy=0; iy<count[1]-1; iy++) {
      // recover site names
      mju::sprintf_arr(txt1, "%sS%d_%d", prefix.c_str(), ix, iy);
      mju::sprintf_arr(txt2, "%sS%d_%d", prefix.c_str(), ix+1, iy+1);

      // create tendon
      mjCTendon* ten = model->AddTendon(def + mjCOMPKIND_SHEAR);
      ten->def = model->defaults[0];
      ten->WrapSite(txt1);
      ten->WrapSite(txt2);

      // name tendon
      mju::sprintf_arr(txt, "%sTS%d_%d", prefix.c_str(), ix, iy);
      ten->name = txt;

      // equality constraint
      mjCEquality* eq = model->AddEquality(def + mjCOMPKIND_SHEAR);
      eq->def = model->defaults[0];
      eq->type = mjEQ_TENDON;
      eq->name1 = txt;
    }
  }
}



// add skin to 2D
void mjCComposite::MakeSkin2(mjCModel* model) {
  char txt[100];
  int N = count[0]*count[1];

  // add skin, set name and material
  mjCSkin* skin = model->AddSkin();
  mju::sprintf_arr(txt, "%sSkin", prefix.c_str());
  skin->name = txt;
  skin->material = skinmaterial;
  mjuu_copyvec(skin->rgba, skinrgba, 4);
  skin->inflate = skininflate;
  skin->group = skingroup;

  // populate mesh: two sides
  for (int i=0; i<2; i++) {
    for (int ix=0; ix<count[0]; ix++) {
      for (int iy=0; iy<count[1]; iy++) {
        // vertex
        skin->vert.push_back(0);
        skin->vert.push_back(0);
        skin->vert.push_back(0);

        // texture coordinate
        if (skintexcoord) {
          skin->texcoord.push_back(ix/(float)(count[0]-1));
          skin->texcoord.push_back(iy/(float)(count[1]-1));
        }

        // face
        if (ix<count[0]-1 && iy<count[1]-1) {
          skin->face.push_back(i*N + ix*count[1]+iy);
          skin->face.push_back(i*N + (ix+1)*count[1]+iy+(i==1));
          skin->face.push_back(i*N + (ix+1)*count[1]+iy+(i==0));

          skin->face.push_back(i*N + ix*count[1]+iy);
          skin->face.push_back(i*N + (ix+(i==0))*count[1]+iy+1);
          skin->face.push_back(i*N + (ix+(i==1))*count[1]+iy+1);
        }
      }
    }
  }

  // add thin triangles: X direction, iy = 0
  for (int ix=0; ix<count[0]-1; ix++) {
    skin->face.push_back(ix*count[1]);
    skin->face.push_back(N + (ix+1)*count[1]);
    skin->face.push_back((ix+1)*count[1]);

    skin->face.push_back(ix*count[1]);
    skin->face.push_back(N + ix*count[1]);
    skin->face.push_back(N + (ix+1)*count[1]);
  }

  // add thin triangles: X direction, iy = count[1]-1
  for (int ix=0; ix<count[0]-1; ix++) {
    skin->face.push_back(ix*count[1] + count[1]-1);
    skin->face.push_back((ix+1)*count[1] + count[1]-1);
    skin->face.push_back(N + (ix+1)*count[1] + count[1]-1);

    skin->face.push_back(ix*count[1] + count[1]-1);
    skin->face.push_back(N + (ix+1)*count[1] + count[1]-1);
    skin->face.push_back(N + ix*count[1] + count[1]-1);
  }

  // add thin triangles: Y direction, ix = 0
  for (int iy=0; iy<count[1]-1; iy++) {
    skin->face.push_back(iy);
    skin->face.push_back(iy+1);
    skin->face.push_back(N + iy+1);

    skin->face.push_back(iy);
    skin->face.push_back(N + iy+1);
    skin->face.push_back(N + iy);
  }

  // add thin triangles: Y direction, ix = count[0]-1
  for (int iy=0; iy<count[1]-1; iy++) {
    skin->face.push_back(iy + (count[0]-1)*count[1]);
    skin->face.push_back(N + iy+1 + (count[0]-1)*count[1]);
    skin->face.push_back(iy+1 + (count[0]-1)*count[1]);

    skin->face.push_back(iy + (count[0]-1)*count[1]);
    skin->face.push_back(N + iy + (count[0]-1)*count[1]);
    skin->face.push_back(N + iy+1 + (count[0]-1)*count[1]);
  }

  // populate bones
  for (int ix=0; ix<count[0]; ix++) {
    for (int iy=0; iy<count[1]; iy++) {
      // body name
      mju::sprintf_arr(txt, "%sB%d_%d", prefix.c_str(), ix, iy);

      // bind pose
      skin->bodyname.push_back(txt);
      skin->bindpos.push_back(0);
      skin->bindpos.push_back(0);
      skin->bindpos.push_back(0);
      skin->bindquat.push_back(1);
      skin->bindquat.push_back(0);
      skin->bindquat.push_back(0);
      skin->bindquat.push_back(0);

      // create vertid and vertweight
      vector<int> vertid;
      vector<float> vertweight;
      vertid.push_back(ix*count[1]+iy);
      vertid.push_back(N + ix*count[1]+iy);
      vertweight.push_back(1);
      vertweight.push_back(1);
      skin->vertid.push_back(vertid);
      skin->vertweight.push_back(vertweight);
    }
  }
}


//------------------------------------- subgrid matrices

// C = W * [f; f_x; f_y; f_xy]
static const mjtNum subW[16*16] = {
  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
 -3,  0,  0,  3,  0,  0,  0,  0, -2,  0,  0, -1,  0,  0,  0,  0,
  2,  0,  0, -2,  0,  0,  0,  0,  1,  0,  0,  1,  0,  0,  0,  0,
  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,
  0,  0,  0,  0, -3,  0,  0,  3,  0,  0,  0,  0, -2,  0,  0, -1,
  0,  0,  0,  0,  2,  0,  0, -2,  0,  0,  0,  0,  1,  0,  0,  1,
 -3,  3,  0,  0, -2, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0, -3,  3,  0,  0, -2, -1,  0,  0,
  9, -9,  9, -9,  6,  3, -3, -6,  6, -6, -3,  3,  4,  2,  1,  2,
 -6,  6, -6,  6, -4, -2,  2,  4, -3,  3,  3, -3, -2, -1, -1, -2,
  2, -2,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  2, -2,  0,  0,  1,  1,  0,  0,
 -6,  6, -6,  6, -3, -3,  3,  3, -4,  4,  2, -2, -2, -2, -1, -1,
  4, -4,  4, -4,  2,  2, -2, -2,  2, -2, -2,  2,  1,  1,  1,  1
};

// left-bottom
static const mjtNum subD00[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  5, -1,   9,  1,   -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  6, -1,   10, 1,   -1,

  5, -1,   6,  1,   -1,       // f_y
  9, -1,   10, 1,   -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  9,  -1,    6, -1,    5, 1,    10, 1,    -1,     // f_xy
  13, -0.5,  6, -0.5,  5, 0.5,  14, 0.5,  -1,
  13, -0.25, 7, -0.25, 5, 0.25, 15, 0.25, -1,
  9,  -0.5,  7, -0.5,  5, 0.5,  11, 0.5,  -1
};

// center-bottom
static const mjtNum subD10[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  2, -0.5, 10, 0.5, -1,

  5, -1,   6,  1,   -1,       // f_y
  9, -1,   10, 1,   -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  9,  -0.5,  2, -0.5,  1, 0.5,  10, 0.5,  -1,     // f_xy
  13, -0.5,  6, -0.5,  5, 0.5,  14, 0.5,  -1,
  13, -0.25, 7, -0.25, 5, 0.25, 15, 0.25, -1,
  9,  -0.25, 3, -0.25, 1, 0.25, 11, 0.25, -1
};

// right-bottom
static const mjtNum subD20[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -1,   9,  1,   -1,
  6, -1,   10, 1,   -1,
  2, -0.5, 10, 0.5, -1,

  5, -1,   6,  1,   -1,       // f_y
  9, -1,   10, 1,   -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  9, -0.5,  2, -0.5,  1, 0.5,  10, 0.5,  -1,      // f_xy
  9, -1,    6, -1,    5, 1,    10, 1,    -1,
  9, -0.5,  7, -0.5,  5, 0.5,  11, 0.5,  -1,
  9, -0.25, 3, -0.25, 1, 0.25, 11, 0.25, -1
};

// left-center
static const mjtNum subD01[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  5, -1,   9,  1,   -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  6, -1,   10, 1,   -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  8,  -0.5,  6, -0.5,  4, 0.5,  10, 0.5,  -1,     // f_xy
  12, -0.25, 6, -0.25, 4, 0.25, 14, 0.25, -1,
  13, -0.25, 7, -0.25, 5, 0.25, 15, 0.25, -1,
  9,  -0.5,  7, -0.5,  5, 0.5,  11, 0.5,  -1
};

// center-center
static const mjtNum subD11[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  2, -0.5, 10, 0.5, -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  8,  -0.25, 2, -0.25, 0, 0.25, 10, 0.25, -1,     // f_xy
  12, -0.25, 6, -0.25, 4, 0.25, 14, 0.25, -1,
  13, -0.25, 7, -0.25, 5, 0.25, 15, 0.25, -1,
  9,  -0.25, 3, -0.25, 1, 0.25, 11, 0.25, -1
};

// right-center
static const mjtNum subD21[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -1,   9,  1,   -1,
  6, -1,   10, 1,   -1,
  2, -0.5, 10, 0.5, -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  8, -0.25, 2, -0.25, 0, 0.25, 10, 0.25, -1,      // f_xy
  8, -0.5,  6, -0.5,  4, 0.5,  10, 0.5,  -1,
  9, -0.5,  7, -0.5,  5, 0.5,  11, 0.5,  -1,
  9, -0.25, 3, -0.25, 1, 0.25, 11, 0.25, -1
};

// left-top
static const mjtNum subD02[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  5, -1,   9,  1,   -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  6, -1,   10, 1,   -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -1,   10, 1,   -1,
  5, -1,   6,  1,   -1,

  8,  -0.5,  6, -0.5,  4, 0.5,  10, 0.5,  -1,     // f_xy
  12, -0.25, 6, -0.25, 4, 0.25, 14, 0.25, -1,
  13, -0.5,  6, -0.5,  5, 0.5,  14, 0.5,  -1,
  9,  -1,    6, -1,    5, 1,    10, 1,    -1
};

// center-top
static const mjtNum subD12[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  2, -0.5, 10, 0.5, -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -1,   10, 1,   -1,
  5, -1,   6,  1,   -1,

  8,  -0.25, 2, -0.25, 0, 0.25, 10, 0.25, -1,     // f_xy
  12, -0.25, 6, -0.25, 4, 0.25, 14, 0.25, -1,
  13, -0.5,  6, -0.5,  5, 0.5,  14, 0.5,  -1,
  9,  -0.5,  2, -0.5,  1, 0.5,  10, 0.5,  -1
};

// right-top
static const mjtNum subD22[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -1,   9,  1,   -1,
  6, -1,   10, 1,   -1,
  2, -0.5, 10, 0.5, -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -1,   10, 1,   -1,
  5, -1,   6,  1,   -1,

  8, -0.25, 2, -0.25, 0, 0.25, 10, 0.25, -1,      // f_xy
  8, -0.5,  6, -0.5,  4, 0.5,  10, 0.5,  -1,
  9, -1,    6, -1,    5, 1,    10, 1,    -1,
  9, -0.5,  2, -0.5,  1, 0.5,  10, 0.5,  -1
};


// add skin to 2D, with subgrid
void mjCComposite::MakeSkin2Subgrid(mjCModel* model) {
  // assemble pointers to Dxx matrices
  const mjtNum* Dp[3][3] = {
    {subD00, subD01, subD02},
    {subD10, subD11, subD12},
    {subD20, subD21, subD22}
  };

  // allocate
  const int N = (2+skinsubgrid)*(2+skinsubgrid);
  mjtNum* XY = (mjtNum*) mju_malloc(N*16*sizeof(mjtNum));
  mjtNum* XY_W = (mjtNum*) mju_malloc(N*16*sizeof(mjtNum));
  mjtNum* Weight = (mjtNum*) mju_malloc(9*N*16*sizeof(mjtNum));
  mjtNum* D = (mjtNum*) mju_malloc(16*16*sizeof(mjtNum));

  // XY matrix
  const mjtNum step = 1.0/(1+skinsubgrid);
  int rxy = 0;
  for (int sx=0; sx<=1+skinsubgrid; sx++) {
    for (int sy=0; sy<=1+skinsubgrid; sy++) {
      // compute x, y
      mjtNum x = sx*step;
      mjtNum y = sy*step;

      // make XY-row
      XY[16*rxy + 0] =  1;
      XY[16*rxy + 1] =  y;
      XY[16*rxy + 2] =  y*y;
      XY[16*rxy + 3] =  y*y*y;

      XY[16*rxy + 4] =  x*1;
      XY[16*rxy + 5] =  x*y;
      XY[16*rxy + 6] =  x*y*y;
      XY[16*rxy + 7] =  x*y*y*y;

      XY[16*rxy + 8] =  x*x*1;
      XY[16*rxy + 9] =  x*x*y;
      XY[16*rxy + 10] = x*x*y*y;
      XY[16*rxy + 11] = x*x*y*y*y;

      XY[16*rxy + 12] = x*x*x*1;
      XY[16*rxy + 13] = x*x*x*y;
      XY[16*rxy + 14] = x*x*x*y*y;
      XY[16*rxy + 15] = x*x*x*y*y*y;

      // advance row
      rxy++;
    }
  }

  // XY_W = XY * W
  mju_mulMatMat(XY_W, XY, subW, N, 16, 16);

  // Weight matrices
  for (int dx=0; dx<3; dx++) {
    for (int dy=0; dy<3; dy++) {
      // make dense D
      mju_zero(D, 16*16);
      int cnt = 0;
      int r = 0, c;
      while (r<16) {
        // scan row
        while ((c = mju_round(Dp[dx][dy][cnt]))!=-1) {
          D[r*16+c] = Dp[dx][dy][cnt+1];
          cnt +=2;
        }

        // advance
        r++;
        cnt++;
      }

      // Weight(d) = XY * W * D(d)
      mju_mulMatMat(Weight + (dx*3+dy)*N*16, XY_W, D, N, 16, 16);
    }
  }

  // add skin, set name and material
  char txt[100];
  mjCSkin* skin = model->AddSkin();
  mju::sprintf_arr(txt, "%sSkin", prefix.c_str());
  skin->name = txt;
  skin->material = skinmaterial;
  mjuu_copyvec(skin->rgba, skinrgba, 4);
  skin->inflate = skininflate;
  skin->group = skingroup;

  // populate mesh: two sides
  mjtNum S = spacing/(1+skinsubgrid);
  int C0 = count[0] + (count[0]-1)*skinsubgrid;
  int C1 = count[1] + (count[1]-1)*skinsubgrid;
  int NN = C0*C1;
  for (int i=0; i<2; i++) {
    for (int ix=0; ix<C0; ix++) {
      for (int iy=0; iy<C1; iy++) {
        // vertex
        skin->vert.push_back(ix*S);
        skin->vert.push_back(iy*S);
        skin->vert.push_back(0);

        // texture coordinate
        if (skintexcoord) {
          skin->texcoord.push_back(ix/(float)(C0-1));
          skin->texcoord.push_back(iy/(float)(C1-1));
        }

        // face
        if (ix<C0-1 && iy<C1-1) {
          skin->face.push_back(i*NN + ix*C1+iy);
          skin->face.push_back(i*NN + (ix+1)*C1+iy+(i==1));
          skin->face.push_back(i*NN + (ix+1)*C1+iy+(i==0));

          skin->face.push_back(i*NN + ix*C1+iy);
          skin->face.push_back(i*NN + (ix+(i==0))*C1+iy+1);
          skin->face.push_back(i*NN + (ix+(i==1))*C1+iy+1);
        }
      }
    }
  }

  // add thin triangles: X direction, iy = 0
  for (int ix=0; ix<C0-1; ix++) {
    skin->face.push_back(ix*C1);
    skin->face.push_back(NN + (ix+1)*C1);
    skin->face.push_back((ix+1)*C1);

    skin->face.push_back(ix*C1);
    skin->face.push_back(NN + ix*C1);
    skin->face.push_back(NN + (ix+1)*C1);
  }

  // add thin triangles: X direction, iy = C1-1
  for (int ix=0; ix<C0-1; ix++) {
    skin->face.push_back(ix*C1 + C1-1);
    skin->face.push_back((ix+1)*C1 + C1-1);
    skin->face.push_back(NN + (ix+1)*C1 + C1-1);

    skin->face.push_back(ix*C1 + C1-1);
    skin->face.push_back(NN + (ix+1)*C1 + C1-1);
    skin->face.push_back(NN + ix*C1 + C1-1);
  }

  // add thin triangles: Y direction, ix = 0
  for (int iy=0; iy<C1-1; iy++) {
    skin->face.push_back(iy);
    skin->face.push_back(iy+1);
    skin->face.push_back(NN + iy+1);

    skin->face.push_back(iy);
    skin->face.push_back(NN + iy+1);
    skin->face.push_back(NN + iy);
  }

  // add thin triangles: Y direction, ix = C0-1
  for (int iy=0; iy<C1-1; iy++) {
    skin->face.push_back(iy + (C0-1)*C1);
    skin->face.push_back(NN + iy+1 + (C0-1)*C1);
    skin->face.push_back(iy+1 + (C0-1)*C1);

    skin->face.push_back(iy + (C0-1)*C1);
    skin->face.push_back(NN + iy + (C0-1)*C1);
    skin->face.push_back(NN + iy+1 + (C0-1)*C1);
  }

  // populate bones
  for (int ix=0; ix<count[0]; ix++) {
    for (int iy=0; iy<count[1]; iy++) {
      // body name
      mju::sprintf_arr(txt, "%sB%d_%d", prefix.c_str(), ix, iy);

      // bind pose
      skin->bodyname.push_back(txt);
      skin->bindpos.push_back(ix*spacing);
      skin->bindpos.push_back(iy*spacing);
      skin->bindpos.push_back(0);
      skin->bindquat.push_back(1);
      skin->bindquat.push_back(0);
      skin->bindquat.push_back(0);
      skin->bindquat.push_back(0);

      // empty vertid and vertweight
      vector<int> vertid;
      vector<float> vertweight;
      skin->vertid.push_back(vertid);
      skin->vertweight.push_back(vertweight);
    }
  }

  // bind vertices to bones: one big square at a time
  for (int ix=0; ix<count[0]-1; ix++) {
    for (int iy=0; iy<count[1]-1; iy++) {
      // determine d for Weight indexing
      int d = 3 * (ix==0 ? 0 : (ix==count[0]-2 ? 2 : 1)) +
              (iy==0 ? 0 : (iy==count[1]-2 ? 2 : 1));

      // precompute 16 bone indices for big square
      int boneid[16];
      int cnt = 0;
      for (int dx=-1; dx<3; dx++) {
        for (int dy=-1; dy<3; dy++) {
          boneid[cnt++] = (ix+dx)*count[1] + (iy+dy);
        }
      }

      // process subgrid, top-rigth owns last index
      for (int dx=0; dx<1+skinsubgrid+(ix==count[0]-2); dx++) {
        for (int dy=0; dy<1+skinsubgrid+(iy==count[1]-2); dy++) {
          // recover vertex id
          int vid = (ix*(1+skinsubgrid)+dx)*C1 + iy*(1+skinsubgrid)+dy;

          // determine row in Weight
          int n = dx*(2+skinsubgrid) + dy;

          // add vertex to 16 bones
          for (int bi=0; bi<16; bi++) {
            mjtNum w = Weight[d*N*16 + n*16 + bi];
            if (w) {
              skin->vertid[boneid[bi]].push_back(vid);
              skin->vertid[boneid[bi]].push_back(vid+NN);
              skin->vertweight[boneid[bi]].push_back((float)w);
              skin->vertweight[boneid[bi]].push_back((float)w);
            }
          }
        }
      }
    }
  }

  // free allocations
  mju_free(XY);
  mju_free(XY_W);
  mju_free(Weight);
  mju_free(D);
}



// add skin to 3D
void mjCComposite::MakeSkin3(mjCModel* model) {
  int vcnt = 0;
  std::map<string, int> vmap;
  char txt[100], cnt0[10], cnt1[10], cnt2[10];
  string fmt;

  // string counts
  mju::sprintf_arr(cnt0, "%d", count[0]-1);
  mju::sprintf_arr(cnt1, "%d", count[1]-1);
  mju::sprintf_arr(cnt2, "%d", count[2]-1);

  // add skin, set name and material
  mjCSkin* skin = model->AddSkin();
  mju::sprintf_arr(txt, "%sSkin", prefix.c_str());
  skin->name = txt;
  skin->material = skinmaterial;
  mjuu_copyvec(skin->rgba, skinrgba, 4);
  skin->inflate = skininflate;
  skin->group = skingroup;

  // box
  if (type==mjCOMPTYPE_BOX) {
    // z-faces
    MakeSkin3Box(skin, count[0], count[1], 1, vcnt, "%sB%d_%d_0");
    fmt = "%sB%d_%d_" + string(cnt2);
    MakeSkin3Box(skin, count[0], count[1], 0, vcnt, fmt.c_str());

    // y-faces
    MakeSkin3Box(skin, count[0], count[2], 0, vcnt, "%sB%d_0_%d");
    fmt = "%sB%d_" + string(cnt1) + "_%d";
    MakeSkin3Box(skin, count[0], count[2], 1, vcnt, fmt.c_str());

    // x-faces
    MakeSkin3Box(skin, count[1], count[2], 1, vcnt, "%sB0_%d_%d");
    fmt = "%sB" + string(cnt0) + "_%d_%d";
    MakeSkin3Box(skin, count[1], count[2], 0, vcnt, fmt.c_str());
  }

  // cylinder
  else if (type==mjCOMPTYPE_CYLINDER) {
    // generate vertices in map
    for (int ix=0; ix<count[0]; ix++) {
      for (int iy=0; iy<count[1]; iy++) {
        for (int iz=0; iz<count[2]; iz++) {
          bool xedge = (ix==0 || ix==count[0]-1);
          bool yedge = (iy==0 || iy==count[1]-1);
          if (xedge || yedge) {
            // body name
            mju::sprintf_arr(txt, "%sB%d_%d_%d", prefix.c_str(), ix, iy, iz);

            // add vertex
            skin->vert.push_back(0);
            skin->vert.push_back(0);
            skin->vert.push_back(0);

            // texture coordinate
            if (skintexcoord) {
              float X = 0, Y = 0;
              if (xedge) {
                X = iy/(float)(count[1]-1);
                Y = iz/(float)(count[2]-1);
              } else {
                X = ix/(float)(count[0]-1);
                Y = iz/(float)(count[2]-1);
              }

              skin->texcoord.push_back(X);
              skin->texcoord.push_back(Y);
            }

            // save vertex id in map
            vmap[txt] = vcnt;
            vcnt++;
          }
        }
      }
    }

    // y-faces
    MakeSkin3Smooth(skin, count[0], count[2], 0, vmap, "%sB%d_0_%d");
    fmt = "%sB%d_" + string(cnt1) + "_%d";
    MakeSkin3Smooth(skin, count[0], count[2], 1, vmap, fmt.c_str());

    // x-faces
    MakeSkin3Smooth(skin, count[1], count[2], 1, vmap, "%sB0_%d_%d");
    fmt = "%sB" + string(cnt0) + "_%d_%d";
    MakeSkin3Smooth(skin, count[1], count[2], 0, vmap, fmt.c_str());

    // z-faces, boxy-type
    MakeSkin3Box(skin, count[0], count[1], 1, vcnt, "%sB%d_%d_0");
    fmt = "%sB%d_%d_" + string(cnt2);
    MakeSkin3Box(skin, count[0], count[1], 0, vcnt, fmt.c_str());
  }

  // smooth
  else {
    // generate vertices in map
    for (int ix=0; ix<count[0]; ix++) {
      for (int iy=0; iy<count[1]; iy++) {
        for (int iz=0; iz<count[2]; iz++) {
          bool xedge = (ix==0 || ix==count[0]-1);
          bool yedge = (iy==0 || iy==count[1]-1);
          bool zedge = (iz==0 || iz==count[2]-1);
          if (xedge || yedge || zedge) {
            // body name
            mju::sprintf_arr(txt, "%sB%d_%d_%d", prefix.c_str(), ix, iy, iz);

            // add vertex
            skin->vert.push_back(0);
            skin->vert.push_back(0);
            skin->vert.push_back(0);

            // texture coordinate
            if (skintexcoord) {
              float X = 0, Y = 0;
              if (xedge) {
                X = iy/(float)(count[1]-1);
                Y = iz/(float)(count[2]-1);
              } else if (yedge) {
                X = ix/(float)(count[0]-1);
                Y = iz/(float)(count[2]-1);
              } else {
                X = ix/(float)(count[0]-1);
                Y = iy/(float)(count[1]-1);
              }

              skin->texcoord.push_back(X);
              skin->texcoord.push_back(Y);
            }

            // save vertex id in map
            vmap[txt] = vcnt;
            vcnt++;
          }
        }
      }
    }

    // z-faces
    MakeSkin3Smooth(skin, count[0], count[1], 1, vmap, "%sB%d_%d_0");
    fmt = "%sB%d_%d_" + string(cnt2);
    MakeSkin3Smooth(skin, count[0], count[1], 0, vmap, fmt.c_str());

    // y-faces
    MakeSkin3Smooth(skin, count[0], count[2], 0, vmap, "%sB%d_0_%d");
    fmt = "%sB%d_" + string(cnt1) + "_%d";
    MakeSkin3Smooth(skin, count[0], count[2], 1, vmap, fmt.c_str());

    // x-faces
    MakeSkin3Smooth(skin, count[1], count[2], 1, vmap, "%sB0_%d_%d");
    fmt = "%sB" + string(cnt0) + "_%d_%d";
    MakeSkin3Smooth(skin, count[1], count[2], 0, vmap, fmt.c_str());
  }
}



// make one face of 3D skin, box
void mjCComposite::MakeSkin3Box(mjCSkin* skin, int c0, int c1, int side,
                                int& vcnt, const char* format) {
  int i0, i1;
  char txt[100];

  // loop over bodies/vertices of specified face
  for (i0=0; i0<c0; i0++) {
    for (i1=0; i1<c1; i1++) {
      // vertex
      skin->vert.push_back(0);
      skin->vert.push_back(0);
      skin->vert.push_back(0);

      // texture coordinate
      if (skintexcoord) {
        skin->texcoord.push_back(i0/(float)(c0-1));
        skin->texcoord.push_back(i1/(float)(c1-1));
      }

      // face
      if (i0<c0-1 && i1<c1-1) {
        skin->face.push_back(vcnt + i0*c1+i1);
        skin->face.push_back(vcnt + (i0+1)*c1+i1+(side==1));
        skin->face.push_back(vcnt + (i0+1)*c1+i1+(side==0));

        skin->face.push_back(vcnt + i0*c1+i1);
        skin->face.push_back(vcnt + (i0+(side==0))*c1+i1+1);
        skin->face.push_back(vcnt + (i0+(side==1))*c1+i1+1);
      }

      // body name
      mju::sprintf_arr(txt, format, prefix.c_str(), i0, i1);

      // bind pose: origin
      skin->bodyname.push_back(txt);
      skin->bindpos.push_back(0);
      skin->bindpos.push_back(0);
      skin->bindpos.push_back(0);
      skin->bindquat.push_back(1);
      skin->bindquat.push_back(0);
      skin->bindquat.push_back(0);
      skin->bindquat.push_back(0);

      // vertid and vertweight
      vector<int> vertid;
      vector<float> vertweight;
      vertid.push_back(vcnt + i0*c1+i1);
      vertweight.push_back(1);
      skin->vertid.push_back(vertid);
      skin->vertweight.push_back(vertweight);
    }
  }

  // update vertex count
  vcnt += c0*c1;
}



// make one face of 3D skin, smooth
void mjCComposite::MakeSkin3Smooth(mjCSkin* skin, int c0, int c1, int side,
                                   const std::map<string, int>& vmap, const char* format) {
  int i0, i1;
  char txt00[100], txt01[100], txt10[100], txt11[100];

  // loop over bodies/vertices of specified face
  for (i0=0; i0<c0; i0++) {
    for (i1=0; i1<c1; i1++) {
      // body names
      mju::sprintf_arr(txt00, format, prefix.c_str(), i0, i1);
      mju::sprintf_arr(txt01, format, prefix.c_str(), i0, i1+1);
      mju::sprintf_arr(txt10, format, prefix.c_str(), i0+1, i1);
      mju::sprintf_arr(txt11, format, prefix.c_str(), i0+1, i1+1);

      // face
      if (i0<c0-1 && i1<c1-1) {
        if (side==0) {
          skin->face.push_back(vmap.find(txt00)->second);
          skin->face.push_back(vmap.find(txt10)->second);
          skin->face.push_back(vmap.find(txt11)->second);

          skin->face.push_back(vmap.find(txt00)->second);
          skin->face.push_back(vmap.find(txt11)->second);
          skin->face.push_back(vmap.find(txt01)->second);
        } else {
          skin->face.push_back(vmap.find(txt00)->second);
          skin->face.push_back(vmap.find(txt01)->second);
          skin->face.push_back(vmap.find(txt11)->second);

          skin->face.push_back(vmap.find(txt00)->second);
          skin->face.push_back(vmap.find(txt11)->second);
          skin->face.push_back(vmap.find(txt10)->second);
        }
      }

      // bind pose: origin
      skin->bodyname.push_back(txt00);
      skin->bindpos.push_back(0);
      skin->bindpos.push_back(0);
      skin->bindpos.push_back(0);
      skin->bindquat.push_back(1);
      skin->bindquat.push_back(0);
      skin->bindquat.push_back(0);
      skin->bindquat.push_back(0);

      // vertid and vertweight
      vector<int> vertid;
      vector<float> vertweight;
      vertid.push_back(vmap.find(txt00)->second);
      vertweight.push_back(1);
      skin->vertid.push_back(vertid);
      skin->vertweight.push_back(vertweight);
    }
  }
}
