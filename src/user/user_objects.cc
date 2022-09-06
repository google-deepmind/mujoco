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

#include "user/user_objects.h"

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "lodepng.h"
#include <mujoco/mjmodel.h>
#include "cc/array_safety.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_file.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_vfs.h"
#include "user/user_model.h"
#include "user/user_util.h"

namespace {
namespace mju = ::mujoco::util;
using std::string;
using std::vector;
}  // namespace

// utiility function for checking size parameters
static void checksize(double* size, mjtGeom type, mjCBase* object, const char* name, int id) {
  // plane: handle infinite
  if (type==mjGEOM_PLANE) {
    if (size[2]<=0) {
      throw mjCError(object, "plane size(3) must be positive in object '%s' (id = %d)", name, id);
    }
  }

  // regular geom
  else {
    for (int i=0; i<mjGEOMINFO[type]; i++) {
      if (size[i]<=0) {
        throw mjCError(object, "sizes must be positive in object '%s' (id = %d)", name, id);
      }
    }
  }
}

// error message for missing "limited" attribute
static void checklimited(
    const mjCBase* obj,
    bool autolimits, const char* entity, const char* attr, int limited, bool hasrange) {
  if (!autolimits && limited == 2 && hasrange) {
    std::stringstream ss;
    ss << entity << " has `" << attr << "range` but not `" << attr << "limited`. "
       << "set the autolimits=\"true\" compiler option, specify `" << attr << "limited` "
       << "explicitly (\"true\" or \"false\"), or remove the `" << attr << "range` attribute.";
    throw mjCError(obj, "%s", ss.str().c_str());
  }
}


//------------------------- class mjCError implementation ------------------------------------------

// constructor
mjCError::mjCError(const mjCBase* obj, const char* msg, const char* str, int pos1, int pos2) {
  char temp[300];

  // init
  warning = false;
  if (obj || msg) {
    mju::sprintf_arr(message, "Error");
  } else {
    message[0] = 0;
  }

  // construct error message
  if (msg) {
    if (str) {
      mju::sprintf_arr(temp, msg, str, pos1, pos2);
    } else {
      mju::sprintf_arr(temp, msg, pos1, pos2);
    }

    mju::strcat_arr(message, ": ");
    mju::strcat_arr(message, temp);
  }

  // append info from mjCBase object
  if (obj) {
    // with or without xml position
    if (obj->xmlpos[0]>= 0) {
      mju::sprintf_arr(temp, "Object name = %s, id = %d, line = %d, column = %d",
                       obj->name.c_str(), obj->id, obj->xmlpos[0], obj->xmlpos[1]);
    } else {
      mju::sprintf_arr(temp, "Object name = %s, id = %d", obj->name.c_str(), obj->id);
    }

    // append to message
    mju::strcat_arr(message, "\n");
    mju::strcat_arr(message, temp);
  }
}



//------------------ class mjCAlternative implementation -------------------------------------------

// constructor
mjCAlternative::mjCAlternative() {
  axisangle[0] = xyaxes[0] = zaxis[0] = euler[0] = fullinertia[0] = mjNAN;
};


// compute frame orientation given alternative specifications
// used for geom, site, body and camera frames
const char* mjCAlternative::Set(double* quat, double* inertia,
                                bool degree, const char* sequence) {
  // set quat using axisangle
  if (mjuu_defined(axisangle[0])) {
    // convert to radians if necessary, normalize axis
    if (degree) {
      axisangle[3] = axisangle[3] / 180.0 * mjPI;
    }
    if (mjuu_normvec(axisangle, 3)<mjEPS) {
      return "axisangle too small";
    }

    // construct quaternion
    double ang2 = axisangle[3]/2;
    quat[0] = cos(ang2);
    quat[1] = sin(ang2)*axisangle[0];
    quat[2] = sin(ang2)*axisangle[1];
    quat[3] = sin(ang2)*axisangle[2];
  }

  // set quat using xyaxes
  if (mjuu_defined(xyaxes[0])) {
    // normalize x axis
    if (mjuu_normvec(xyaxes, 3)<mjEPS) {
      return "xaxis too small";
    }

    // make y axis orthogonal to x axis, normalize
    double d = mjuu_dot3(xyaxes, xyaxes+3);
    xyaxes[3] -= xyaxes[0]*d;
    xyaxes[4] -= xyaxes[1]*d;
    xyaxes[5] -= xyaxes[2]*d;
    if (mjuu_normvec(xyaxes+3, 3)<mjEPS) {
      return "yaxis too small";
    }

    // compute and normalize z axis
    double z[3];
    mjuu_crossvec(z, xyaxes, xyaxes+3);
    if (mjuu_normvec(z, 3)<mjEPS) {
      return "cross(xaxis, yaxis) too small";
    }

    // convert frame into quaternion
    mjuu_frame2quat(quat, xyaxes, xyaxes+3, z);
  }

  // set quat using zaxis
  if (mjuu_defined(zaxis[0])) {
    if (mjuu_normvec(zaxis, 3)<mjEPS) {
      return "zaxis too small";
    }
    mjuu_z2quat(quat, zaxis);
  }

  // handle fullinertia
  if (mjuu_defined(fullinertia[0])) {
    mjtNum eigval[3], eigvec[9], quattmp[4];
    mjtNum full[9] = {
      fullinertia[0], fullinertia[3], fullinertia[4],
      fullinertia[3], fullinertia[1], fullinertia[5],
      fullinertia[4], fullinertia[5], fullinertia[2]
    };

    mju_eig3(eigval, eigvec, quattmp, full);

    // copy
    for (int i=0; i<4; i++) {
      quat[i] = quattmp[i];
    }
    if (inertia) {
      for (int i=0; i<3; i++) {
        inertia[i] = eigval[i];
      }
    }

    // check mimimal eigenvalue
    if (eigval[2]<mjEPS) {
      return "inertia must have positive eigenvalues";
    }
  }

  // handle euler
  if (mjuu_defined(euler[0])) {
    // convert to radians if necessary
    if (degree) {
      for (int i=0; i<3; i++) {
        euler[i] = euler[i] / 180.0 * mjPI;
      }
    }

    // init
    mjuu_setvec(quat, 1, 0, 0, 0);

    // loop over euler angles, accumulate rotations
    for (int i=0; i<3; i++) {
      double tmp[4], qrot[4] = {cos(euler[i]/2), 0, 0, 0};
      double sa = sin(euler[i]/2);

      // construct quaternion rotation
      if (sequence[i]=='x' || sequence[i]=='X') {
        qrot[1] = sa;
      } else if (sequence[i]=='y' || sequence[i]=='Y') {
        qrot[2] = sa;
      } else if (sequence[i]=='z' || sequence[i]=='Z') {
        qrot[3] = sa;
      } else {
        return "euler sequence can only contain x, y, z, X, Y, Z";
      }

      // accumulate rotation
      if (sequence[i]=='x' || sequence[i]=='y' || sequence[i]=='z') {
        mjuu_mulquat(tmp, quat, qrot);  // moving axes: post-multiply
      } else {
        mjuu_mulquat(tmp, qrot, quat);  // fixed axes: pre-multiply
      }
      mjuu_copyvec(quat, tmp, 4);
    }

    // normalize, just in case
    mjuu_normvec(quat, 4);
  }

  return 0;
}



//------------------------- class mjCDef implementation --------------------------------------------

// constructor
mjCDef::mjCDef(void) {
  name.clear();
  parentid = -1;
  childid.clear();
}



// compiler
void mjCDef::Compile(const mjCModel* model) {
  // enforce length of all default userdata arrays
  joint.userdata.resize(model->nuser_jnt);
  geom.userdata.resize(model->nuser_geom);
  site.userdata.resize(model->nuser_site);
  camera.userdata.resize(model->nuser_cam);
  tendon.userdata.resize(model->nuser_tendon);
  actuator.userdata.resize(model->nuser_actuator);
}



//------------------------- class mjCBase implementation -------------------------------------------

// constructor
mjCBase::mjCBase() {
  name.clear();
  classname.clear();
  id = -1;
  xmlpos[0] = xmlpos[1] = -1;
  model = 0;
  def = 0;
}



//------------------ class mjCBody implementation --------------------------------------------------

// constructor
mjCBody::mjCBody(mjCModel* _model) {
  // set model pointer
  model = _model;

  // missing information, must be supplied later
  pos[0] = ipos[0] = mjNAN;

  // clear variables
  explicitinertial = false;
  mocap = false;
  mjuu_setvec(quat, 1, 0, 0, 0);
  mjuu_setvec(iquat, 1, 0, 0, 0);
  mjuu_setvec(locquat, 1, 0, 0, 0);
  mjuu_setvec(lociquat, 1, 0, 0, 0);
  mjuu_zerovec(pos+1, 2);
  mjuu_zerovec(ipos+1, 2);
  mjuu_zerovec(locpos, 3);
  mjuu_zerovec(locipos, 3);
  mass = 0;
  mjuu_setvec(inertia, 0, 0, 0);
  parentid = -1;
  weldid = -1;
  dofnum = 0;
  lastdof = -1;
  userdata.clear();

  // clear object lists
  bodies.clear();
  geoms.clear();
  joints.clear();
  sites.clear();
  cameras.clear();
  lights.clear();
}



// destructor
mjCBody::~mjCBody() {
  unsigned int i;

  // delete objects allocated here
  for (i=0; i<bodies.size(); i++) delete bodies[i];
  for (i=0; i<geoms.size(); i++) delete geoms[i];
  for (i=0; i<joints.size(); i++) delete joints[i];
  for (i=0; i<sites.size(); i++) delete sites[i];
  for (i=0; i<cameras.size(); i++) delete cameras[i];
  for (i=0; i<lights.size(); i++) delete lights[i];

  bodies.clear();
  geoms.clear();
  joints.clear();
  sites.clear();
  cameras.clear();
  lights.clear();
}



// create child body and add it to body
mjCBody* mjCBody::AddBody(mjCDef* _def) {
  // create body
  mjCBody* obj = new mjCBody(model);

  // handle def recursion (i.e. childclass)
  obj->def = _def ? _def : def;

  bodies.push_back(obj);
  return obj;
}



// create new joint and add it to body
//  _def==NULL means no defaults, unlike all others which inherit from body
mjCJoint* mjCBody::AddJoint(mjCDef* _def, bool isfree) {
  // create joint
  mjCJoint* obj = new mjCJoint(model, _def ? _def : (isfree ? NULL : def));

  // set free type if specified
  if (isfree) {
    obj->type = mjJNT_FREE;
  }

  // set body pointer, add
  obj->body = this;

  joints.push_back(obj);
  return obj;
}



// create new geom and add it to body
mjCGeom* mjCBody::AddGeom(mjCDef* _def) {
  // create geom
  mjCGeom* obj = new mjCGeom(model, _def ? _def : def);

  //  set body pointer, add
  obj->body = this;

  geoms.push_back(obj);
  return obj;
}



// create new site and add it to body
mjCSite* mjCBody::AddSite(mjCDef* _def) {
  // create site
  mjCSite* obj = new mjCSite(model, _def ? _def : def);

  // set body pointer, add
  obj->body = this;

  sites.push_back(obj);
  return obj;
}



// create new camera and add it to body
mjCCamera* mjCBody::AddCamera(mjCDef* _def) {
  // create camera
  mjCCamera* obj = new mjCCamera(model, _def ? _def : def);

  // set body pointer, add
  obj->body = this;

  cameras.push_back(obj);
  return obj;
}



// create new light and add it to body
mjCLight* mjCBody::AddLight(mjCDef* _def) {
  // create light
  mjCLight* obj = new mjCLight(model, _def ? _def : def);

  // set body pointer, add
  obj->body = this;

  lights.push_back(obj);
  return obj;
}



// get number of objects of specified type
int mjCBody::NumObjects(mjtObj type) {
  switch (type) {
  case mjOBJ_BODY:
  case mjOBJ_XBODY:
    return (int)bodies.size();
  case mjOBJ_JOINT:
    return (int)joints.size();
  case mjOBJ_GEOM:
    return (int)geoms.size();
  case mjOBJ_SITE:
    return (int)sites.size();
  case mjOBJ_CAMERA:
    return (int)cameras.size();
  case mjOBJ_LIGHT:
    return (int)lights.size();
  default:
    return 0;
  }
}



// get poiner to specified object
mjCBase* mjCBody::GetObject(mjtObj type, int id) {
  if (id>=0 && id<NumObjects(type)) {
    switch (type) {
    case mjOBJ_BODY:
    case mjOBJ_XBODY:
      return bodies[id];
    case mjOBJ_JOINT:
      return joints[id];
    case mjOBJ_GEOM:
      return geoms[id];
    case mjOBJ_SITE:
      return sites[id];
    case mjOBJ_CAMERA:
      return cameras[id];
    case mjOBJ_LIGHT:
      return lights[id];
    default:
      return 0;
    }
  }

  return 0;
}



// find object by name in given list
template <class T>
static T* findobject(string name, vector<T*>& list) {
  for (unsigned int i=0; i<list.size(); i++) {
    if (list[i]->name == name) {
      return list[i];
    }
  }

  return 0;
}



// recursive find by name
mjCBase* mjCBody::FindObject(mjtObj type, string _name, bool recursive) {
  mjCBase* res = 0;

  // check self: just in case
  if (name == _name) {
    return this;
  }

  // search elements of this body
  if (type==mjOBJ_BODY || type==mjOBJ_XBODY) {
    res = findobject(_name, bodies);
  } else if (type==mjOBJ_JOINT) {
    res = findobject(_name, joints);
  } else if (type==mjOBJ_GEOM) {
    res = findobject(_name, geoms);
  } else if (type==mjOBJ_SITE) {
    res = findobject(_name, sites);
  } else if (type==mjOBJ_CAMERA) {
    res = findobject(_name, cameras);
  } else if (type==mjOBJ_LIGHT) {
    res = findobject(_name, lights);
  }

  // found
  if (res) {
    return res;
  }

  // search children
  if (recursive) {
    for (int i=0; i<(int)bodies.size(); i++) {
      if ((res = bodies[i]->FindObject(type, _name, true))) {
        return res;
      }
    }
  }

  // not found
  return res;
}



// compute geom inertial frame: ipos, iquat, mass, inertia
void mjCBody::GeomFrame(void) {
  int i, sz;
  double com[3] = {0, 0, 0};
  double toti[6] = {0, 0, 0, 0, 0, 0};
  vector<mjCGeom*> sel;

  // select geoms based on group
  sel.clear();
  for (i=0; i<geoms.size(); i++) {
    if (geoms[i]->group>=model->inertiagrouprange[0] &&
        geoms[i]->group<=model->inertiagrouprange[1]) {
      sel.push_back(geoms[i]);
    }
  }
  sz = sel.size();

  // single geom: copy
  if (sz==1) {
    mjuu_copyvec(ipos, sel[0]->pos, 3);
    mjuu_copyvec(iquat, sel[0]->quat, 4);
    mass = sel[0]->mass;
    mjuu_copyvec(inertia, sel[0]->inertia, 3);
  }

  // multiple geoms
  else if (sz>1) {
    // compute total mass and center of mass
    mass = 0;
    for (i=0; i<sz; i++) {
      mass += sel[i]->mass;
      com[0] += sel[i]->mass * sel[i]->pos[0];
      com[1] += sel[i]->mass * sel[i]->pos[1];
      com[2] += sel[i]->mass * sel[i]->pos[2];
    }

    // check for small mass
    if (mass<mjMINVAL) {
      throw mjCError(this, "body mass is too small, cannot compute center of mass");
    }

    // ipos = geom com
    ipos[0] = com[0]/mass;
    ipos[1] = com[1]/mass;
    ipos[2] = com[2]/mass;

    // add geom inertias
    for (i=0; i<sz; i++) {
      double inert0[6], inert1[6];
      double dpos[3] = {
        sel[i]->pos[0] - ipos[0],
        sel[i]->pos[1] - ipos[1],
        sel[i]->pos[2] - ipos[2]
      };

      mjuu_globalinertia(inert0, sel[i]->inertia, sel[i]->quat);
      mjuu_offcenter(inert1, sel[i]->mass, dpos);
      for (int j=0; j<6; j++) {
        toti[j] = toti[j] + inert0[j] + inert1[j];
      }
    }

    // compute principal axes of inertia
    mjCAlternative alt;
    mjuu_copyvec(alt.fullinertia, toti, 6);
    const char* err1 = alt.Set(iquat, inertia, model->degree, model->euler);
    if (err1) {
      throw mjCError(this, "error '%s' in alternative for principal axes", err1);
    }
  }
}



// setup child local frame: pos
void mjCBody::MakeLocal(double* _locpos, double* _locquat,
                        const double* _pos, const double* _quat) {
  // global: transform to local
  if (model->global) {
    mjuu_localpos(_locpos, _pos, pos, quat);
    mjuu_localquat(_locquat, _quat, quat);
  }

  // local: copy
  else {
    mjuu_copyvec(_locpos, _pos, 3);
    mjuu_copyvec(_locquat, _quat, 4);
  }
}

// set explicitinertial to true
void mjCBody::MakeInertialExplicit() {
  explicitinertial = true;
}


// compiler
void mjCBody::Compile(void) {
  unsigned int i;

  // resize userdata
  if (userdata.size() > model->nuser_body) {
    throw mjCError(this, "user has more values than nuser_body in body '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata.resize(model->nuser_body);

  // pos defaults to (0,0,0) in local coordinates
  if (!mjuu_defined(pos[0]) && !model->global) {
    mjuu_setvec(pos, 0, 0, 0);
  }

  // normalize user-defined quaternions
  mjuu_normvec(quat, 4);
  mjuu_normvec(iquat, 4);

  // set parentid and weldid of children
  for (i=0; i<bodies.size(); i++) {
    bodies[i]->parentid = id;
    bodies[i]->weldid = (!bodies[i]->joints.empty() ? bodies[i]->id : weldid);
  }

  // check and process orientation alternatives for body
  const char* err = alt.Set(quat, inertia, model->degree, model->euler);
  if (err) {
    throw mjCError(this, "error '%s' in frame alternative", err);
  }

  // check and process orientation alternatives for inertia
  const char* ierr = ialt.Set(iquat, inertia, model->degree, model->euler);
  if (ierr) {
    throw mjCError(this, "error '%s' in inertia alternative", ierr);
  }

  // compile all geoms, phase 1
  for (i=0; i<geoms.size(); i++) {
    geoms[i]->inferinertia = id>0 && (!explicitinertial ||
                                      model->inertiafromgeom==mjINERTIAFROMGEOM_TRUE);
    geoms[i]->Compile();
  }

  // set inertial frame from geoms if necessary
  if (id>0 && (model->inertiafromgeom==mjINERTIAFROMGEOM_TRUE ||
               (!mjuu_defined(ipos[0]) && model->inertiafromgeom==mjINERTIAFROMGEOM_AUTO))) {
    GeomFrame();
  }

  // both pos and ipos undefiend: error
  if (!mjuu_defined(ipos[0]) && !mjuu_defined(pos[0])) {
    throw mjCError(this, "body pos and ipos are both undefined");
  }

  // ipos undefined: copy body frame into inertial
  else if (!mjuu_defined(ipos[0])) {
    mjuu_copyvec(ipos, pos, 3);
    mjuu_copyvec(iquat, quat, 4);
  }

  // pos undefined: copy inertial frame into body frame
  else if (!mjuu_defined(pos[0])) {
    mjuu_copyvec(pos, ipos, 3);
    mjuu_copyvec(quat, iquat, 4);
  }

  // check and correct mass and inertia
  if (id>0) {
    // fix minimum
    mass = mjMAX(mass, model->boundmass);
    inertia[0] = mjMAX(inertia[0], model->boundinertia);
    inertia[1] = mjMAX(inertia[1], model->boundinertia);
    inertia[2] = mjMAX(inertia[2], model->boundinertia);

    // check for negative values
    if (mass<0 || inertia[0]<0 || inertia[1]<0 ||inertia[2]<0) {
      throw mjCError(this, "mass and inertia cannot be negative");
    }

    // check for non-physical inertia
    if (inertia[0] + inertia[1] < inertia[2] ||
        inertia[0] + inertia[2] < inertia[1] ||
        inertia[1] + inertia[2] < inertia[0]) {
      if (model->balanceinertia) {
        inertia[0] = inertia[1] = inertia[2] = (inertia[0] + inertia[1] + inertia[2])/3.0;
      } else {
        throw mjCError(this, "inertia must satisfy A + B >= C; use 'balanceinertia' to fix");
      }
    }
  }

  // compute local frame rel. to parent body
  if (id>0) {
    model->bodies[parentid]->MakeLocal(locpos, locquat, pos, quat);
  }

  // make local inertial frame relative to this body
  if (id>0) {
    MakeLocal(locipos, lociquat, ipos, iquat);
  }

  // make local frames of geoms
  for (i=0; i<geoms.size(); i++) {
    MakeLocal(geoms[i]->locpos, geoms[i]->locquat, geoms[i]->pos, geoms[i]->quat);
  }

  // compile all joints, count dofs
  dofnum = 0;
  for (i=0; i<joints.size(); i++) {
    dofnum += joints[i]->Compile();
  }

  // check for excessive number of dofs
  if (dofnum>6) {
    throw mjCError(this, "more than 6 dofs in body '%s'", name.c_str());
  }

  // check for rotation dof after ball joint
  bool hasball = false;
  for (i=0; i<joints.size(); i++) {
    if ((joints[i]->type==mjJNT_BALL || joints[i]->type==mjJNT_HINGE) && hasball) {
      throw mjCError(this, "ball followed by rotation in body '%s'", name.c_str());
    }
    if (joints[i]->type==mjJNT_BALL) {
      hasball = true;
    }
  }

  // make sure mocap body is fixed child of world
  if (mocap)
    if (dofnum || parentid) {
      throw mjCError(this, "mocap body '%s' is not a fixed child of world", name.c_str());
    }

  // compile all sites
  for (i=0; i<sites.size(); i++) sites[i]->Compile();

  // compile all cameras
  for (i=0; i<cameras.size(); i++) cameras[i]->Compile();

  // compile all lights
  for (i=0; i<lights.size(); i++) lights[i]->Compile();
}



//------------------ class mjCJoint implementation -------------------------------------------------

// initialize default joint
mjCJoint::mjCJoint(mjCModel* _model, mjCDef* _def) {
  // joint defaults
  type = mjJNT_HINGE;
  group = 0;
  mjuu_setvec(pos, 0, 0, 0);
  mjuu_setvec(axis, 0, 0, 1);
  limited = 2;
  stiffness = 0;
  range[0] = 0;
  range[1] = 0;
  springdamper[0] = 0;
  springdamper[1] = 0;
  mj_defaultSolRefImp(solref_limit, solimp_limit);
  mj_defaultSolRefImp(solref_friction, solimp_friction);
  margin = 0;
  ref = 0;
  springref = 0;
  userdata.clear();

  // dof defaults
  armature = 0;
  frictionloss = 0;
  damping = 0;

  // clear internal variables
  body = 0;
  mjuu_setvec(locpos, 0, 0, 0);
  mjuu_setvec(locaxis, 0, 0, 1);
  urdfeffort = -1;

  // reset to default if given
  if (_def) {
    *this = _def->joint;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



// compiler
int mjCJoint::Compile(void) {
  // resize userdata
  if (userdata.size() > model->nuser_jnt) {
    throw mjCError(this, "user has more values than nuser_jnt in joint '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata.resize(model->nuser_jnt);

  // check springdamper
  if (springdamper[0] || springdamper[1]) {
    if (springdamper[0]<=0 || springdamper[1]<=0) {
      throw mjCError(this,
                     "when defined, springdamper values must be positive in joint '%s' (id = %d)",
                     name.c_str(), id);
    }
  }

  // free joints cannot be limited
  if (type==mjJNT_FREE) {
    limited = 0;
  }
  // otherwise if limited is auto, set according to whether range is specified
  else if (limited==2) {
    bool hasrange = !(range[0]==0 && range[1]==0);
    checklimited(this, model->autolimits, "joint", "", limited, hasrange);
    limited = hasrange ? 1 : 0;
  }

  // resolve limits
  if (limited) {
    // check data
    if (range[0]>=range[1] && type!=mjJNT_BALL) {
      throw mjCError(this,
                     "range[0] should be smaller than range[1] in joint '%s' (id = %d)",
                     name.c_str(), id);
    }
    if (range[0] && type==mjJNT_BALL) {
      throw mjCError(this, "range[0] should be 0 in ball joint '%s' (id = %d)", name.c_str(), id);
    }

    // convert limits to radians
    if (model->degree && (type==mjJNT_HINGE || type==mjJNT_BALL)) {
      if (range[0]) {
        range[0] *= mjPI/180.0;
      }
      if (range[1]) {
        range[1] *= mjPI/180.0;
      }
    }
  }

  // FREE or BALL: set axis to (0,0,1)
  if (type==mjJNT_FREE || type==mjJNT_BALL) {
    axis[0] = axis[1] = 0;
    axis[2] = 1;
  }

  // FREE: set pos to (0,0,0)
  if (type==mjJNT_FREE) {
    mjuu_zerovec(pos, 3);
  }

  // normalize axis, check norm
  if (mjuu_normvec(axis, 3)<mjEPS) {
    throw mjCError(this, "axis too small in joint '%s' (id = %d)", name.c_str(), id);
  }

  // check data
  if (type==mjJNT_FREE && limited) {
    throw mjCError(this,
                   "limits should not be defined in free joint '%s' (id = %d)", name.c_str(), id);
  }

  // compute local position
  if (type!=mjJNT_FREE) {
    double qunit[4] = {1, 0, 0, 0};
    double qloc[4];
    body->MakeLocal(locpos, qloc, pos, qunit);
  } else {
    mjuu_zerovec(locpos, 3);
  }

  // compute local axis relative to specified body
  if (model->global) {
    mjuu_localaxis(locaxis, axis, body->quat);
  } else {
    mjuu_copyvec(locaxis, axis, 3);
  }

  // convert reference angles to radians for hinge joints
  if (type==mjJNT_HINGE && model->degree) {
    ref *= mjPI/180.0;
    springref *= mjPI/180.0;
  }

  // return dofnum
  if (type==mjJNT_FREE) {
    return 6;
  } else if (type==mjJNT_BALL) {
    return 3;
  } else {
    return 1;
  }
}



//------------------ class mjCGeom implementation --------------------------------------------------

// initialize default geom
mjCGeom::mjCGeom(mjCModel* _model, mjCDef* _def) {
  // clear alternatives
  fromto[0] = mjNAN;
  _mass = mjNAN;

  // set defaults
  type = mjGEOM_SPHERE;
  mjuu_setvec(size, 0, 0, 0);
  contype = 1;
  conaffinity = 1;
  condim = 3;
  group = 0;
  priority = 0;
  mjuu_setvec(friction, 1, 0.005, 0.0001);
  solmix = 1.0;
  mj_defaultSolRefImp(solref, solimp);
  margin = 0;
  gap = 0;
  fluid_switch = 0.0;
  // user-tunable ellipsoid-fluid interaction coefs sorted from most to least likely to need tuning
  // defaults are tuned for slender bodies in a Re 50-1000 environment
  fluid_coefs[0] = 0.5;       // blunt_drag_coef
  fluid_coefs[1] = 0.25;      // slender_drag_coef
  fluid_coefs[2] = 1.5;       // ang_drag_coef
  fluid_coefs[3] = 1.0;       // kutta_lift_coef
  fluid_coefs[4] = 1.0;       // magnus_lift_coef
  for (int i = 0; i < mjNFLUID; i++){
    fluid[i] = 0;
  }
  density = 1000;             // water density (1000 kg / m^3)
  mesh.clear();
  fitscale = 1;
  material.clear();
  rgba[0] = rgba[1] = rgba[2] = 0.5f;
  rgba[3] = 1.0f;
  userdata.clear();
  typeinertia = mjVOLUME_MESH;
  inferinertia = true;

  // clear internal variables
  mjuu_setvec(quat, 1, 0, 0, 0);
  mjuu_setvec(pos, 0, 0, 0);
  mjuu_setvec(locpos, 0, 0, 0);
  mjuu_setvec(locquat, 1, 0, 0, 0);
  mass = 0;
  mjuu_setvec(inertia, 0, 0, 0);
  body = 0;
  matid = -1;
  meshid = -1;
  hfieldid = -1;

  // reset to default if given
  if (_def) {
    *this = _def->geom;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



// compute geom volume
double mjCGeom::GetVolume(void) {
  double height;

  // get from mesh
  if (type==mjGEOM_MESH) {
    if (meshid<0 || meshid>=(int)model->meshes.size()) {
      throw mjCError(this, "invalid meshid in mesh geom '%s' (id = %d)", name.c_str(), id);
    }

    mjCMesh* pmesh = model->meshes[meshid];
    if (model->exactmeshinertia) {
      return pmesh->GetVolumeRef(typeinertia);
    } else {
      return pmesh->boxsz_volume[0]*pmesh->boxsz_volume[1]*pmesh->boxsz_volume[2]*8;
    }
  }

  // compute from geom shape
  else {
    switch (type) {
    case mjGEOM_SPHERE:
      return 4*mjPI*size[0]*size[0]*size[0]/3;

    case mjGEOM_CAPSULE:
      height = 2*size[1];
      return mjPI*(size[0]*size[0]*height + 4*size[0]*size[0]*size[0]/3);

    case mjGEOM_CYLINDER:
      height = 2*size[1];
      return mjPI*size[0]*size[0]*height;

    case mjGEOM_ELLIPSOID:
      return 4*mjPI*size[0]*size[1]*size[2]/3;

    case mjGEOM_BOX:
      return size[0]*size[1]*size[2]*8;

    default:
      return 0;
    }
  }
}



// set geom diagonal inertia given density
void mjCGeom::SetInertia(void) {
  double height;

  // get from mesh
  if (type==mjGEOM_MESH) {
    if (meshid<0 || meshid>=(int)model->meshes.size()) {
      throw mjCError(this, "invalid meshid in mesh geom '%s' (id = %d)", name.c_str(), id);
    }

    mjCMesh* pmesh = model->meshes[meshid];
    double* boxsz = pmesh->GetInertiaBoxPtr(typeinertia);
    inertia[0] = mass*(boxsz[1]*boxsz[1] + boxsz[2]*boxsz[2]) / 3;
    inertia[1] = mass*(boxsz[0]*boxsz[0] + boxsz[2]*boxsz[2]) / 3;
    inertia[2] = mass*(boxsz[0]*boxsz[0] + boxsz[1]*boxsz[1]) / 3;
  }

  // compute from geom shape
  else {
    if (typeinertia)
      throw mjCError(this, "typeinertia currently only available for meshes'%s' (id = %d)",
                     name.c_str(), id);
    switch (type) {
    case mjGEOM_SPHERE:
      inertia[0] = inertia[1] = inertia[2] = 2*mass*size[0]*size[0]/5;
      return;

    case mjGEOM_CAPSULE: {
      height = 2*size[1];
      double radius = size[0];
      double sphere_mass = mass*4*radius/(4*radius + 3*height);  // mass*(sphere_vol/total_vol)
      double cylinder_mass = mass - sphere_mass;
      // cylinder part
      inertia[0] = inertia[1] = cylinder_mass*(3*radius*radius + height*height)/12;
      inertia[2] = cylinder_mass*radius*radius/2;
      // add two hemispheres, displace along third axis
      double sphere_inertia = 2*sphere_mass*radius*radius/5;
      inertia[0] += sphere_inertia + sphere_mass*height*(3*radius + 2*height)/8;
      inertia[1] += sphere_inertia + sphere_mass*height*(3*radius + 2*height)/8;
      inertia[2] += sphere_inertia;
      return;
    }

    case mjGEOM_CYLINDER:
      height = 2*size[1];
      inertia[0] = inertia[1] = mass*(3*size[0]*size[0]+height*height)/12;
      inertia[2] = mass*size[0]*size[0]/2;
      return;

    case mjGEOM_ELLIPSOID:
      inertia[0] = mass*(size[1]*size[1]+size[2]*size[2])/5;
      inertia[1] = mass*(size[0]*size[0]+size[2]*size[2])/5;
      inertia[2] = mass*(size[0]*size[0]+size[1]*size[1])/5;
      return;

    case mjGEOM_BOX:
      inertia[0] = mass*(size[1]*size[1]+size[2]*size[2])/3;
      inertia[1] = mass*(size[0]*size[0]+size[2]*size[2])/3;
      inertia[2] = mass*(size[0]*size[0]+size[1]*size[1])/3;
      return;

    default:
      inertia[0] = inertia[1] = inertia[2] = 0;
      return;
    }
  }
}



// compute radius of bounding sphere
double mjCGeom::GetRBound(void) {
  double* aabb;

  switch (type) {
  case mjGEOM_SPHERE:
    return size[0];

  case mjGEOM_CAPSULE:
    return size[0]+size[1];

  case mjGEOM_CYLINDER:
    return sqrt(size[0]*size[0]+size[1]*size[1]);

  case mjGEOM_ELLIPSOID:
    return mjMAX(mjMAX(size[0], size[1]), size[2]);

  case mjGEOM_BOX:
    return sqrt(size[0]*size[0]+size[1]*size[1]+size[2]*size[2]);

  case mjGEOM_MESH:
    aabb = model->meshes[meshid]->aabb;
    return sqrt(aabb[0]*aabb[0]+aabb[1]*aabb[1]+aabb[2]*aabb[2]);

  default:
    return 0;
  }
}



// Compute the coefficients of the added inertia due to the surrounding fluid.
double mjCGeom::GetAddedMassKappa(double dx, double dy, double dz) {
  // Integration by Gauss–Kronrod quadrature on interval l in [0, infinity] of
  // f(l) = dx*dy*dz / np.sqrt((dx*dx+ l)**3 * (dy*dy+ l) * (dz*dz+ l))
  // 15-point Gauss–Kronrod quadrature (K15) points x in [0, 1].

  // static constexpr mjtNum kronrod_x[15] = [     // unused, left in comment for completeness
  //   0.00427231, 0.02544604, 0.06756779, 0.12923441, 0.20695638,
  //   0.29707742, 0.39610752, 0.50000000, 0.60389248, 0.70292258,
  //   0.79304362, 0.87076559, 0.93243221, 0.97455396, 0.99572769];
  // 15-point Gauss–Kronrod quadrature (K15) weights.
  static constexpr double kronrod_w[15] = {
    0.01146766, 0.03154605, 0.05239501, 0.07032663, 0.08450236,
    0.09517529, 0.10221647, 0.10474107, 0.10221647, 0.09517529,
    0.08450236, 0.07032663, 0.05239501, 0.03154605, 0.01146766};
  // Integrate from 0 to inf by change of variables:
  // l = x^3 / (1-x)^2. Exponents 3 and 2 found to minimize error.
  static constexpr double kronrod_l[15] = {
    7.865151709349917e-08, 1.7347976913907274e-05, 0.0003548008144506193,
    0.002846636252924549, 0.014094260903596077, 0.053063261727396636,
    0.17041978741317773, 0.5, 1.4036301548686991, 3.9353484827022642,
    11.644841677041734, 39.53187807410903, 177.5711362220801,
    1429.4772912937397, 54087.416549217705};
  // dl = dl/dx dx. The following are dl/dx(x).
  static constexpr double kronrod_d[15] = {
    5.538677720489877e-05, 0.002080868285293228, 0.016514126520723166,
    0.07261900344370877, 0.23985243401862602, 0.6868318249020725,
    1.8551129519182894, 5.0, 14.060031152313941, 43.28941239611009,
    156.58546376397112, 747.9826085305024, 5827.4042950027115,
    116754.0197944512, 25482945.327264845};

  const double invdx2 = 1.0 / (dx * dx);
  const double invdy2 = 1.0 / (dy * dy);
  const double invdz2 = 1.0 / (dz * dz);

  // for added numerical stability we non-dimensionalize x by scale
  // because 1 + l/d^2 in denom, l should be scaled by d^2
  const double scale = std::pow(dx*dx*dx * dy * dz, 0.4);  // ** (2/5)
  double kappa = 0.0;
  for (int i = 0; i < 15; ++i) {
    const double lambda = scale * kronrod_l[i];
    const double denom = (1 + lambda*invdx2) * std::sqrt(
      (1 + lambda*invdx2) * (1 + lambda*invdy2) * (1 + lambda*invdz2));
    kappa += scale * kronrod_d[i] / denom * kronrod_w[i];
  }
  return kappa * invdx2;
}



// Compute the kappa coefs of the added inertia due to the surrounding fluid.
void mjCGeom::SetFluidCoefs(void) {
  double dx, dy, dz;

  // get semiaxes
  switch (type) {

    case mjGEOM_SPHERE:
      dx = size[0];
      dy = size[0];
      dz = size[0];
      break;

    case mjGEOM_CAPSULE:
      dx = size[0];
      dy = size[0];
      dz = size[1] + size[0];
      break;

    case mjGEOM_CYLINDER:
      dx = size[0];
      dy = size[0];
      dz = size[1];
      break;

    default:
      dx = size[0];
      dy = size[1];
      dz = size[2];
  }

  // volume of equivalent ellipsoid
  const double volume = 4.0 / 3.0 * mjPI * dx * dy * dz;

  // GetAddedMassKappa is invariant to permutation of last two arguments
  const double kx = GetAddedMassKappa(dx, dy, dz);
  const double ky = GetAddedMassKappa(dy, dz, dx);
  const double kz = GetAddedMassKappa(dz, dx, dy);

  // coefficients of virtual moment of inertia. Note: if (kz-ky) in numerator
  // is negative, also the denom is negative. Abs both and clip to MINVAL
  const auto pow2 = [](const double val) { return val * val; };
  const double Ixfac = pow2(dy*dy - dz*dz) * std::fabs(kz - ky) / std::max(
    mjMINVAL, std::fabs(2*(dy*dy - dz*dz) + (dy*dy + dz*dz)*(ky - kz)));
  const double Iyfac = pow2(dz*dz - dx*dx) * std::fabs(kx - kz) / std::max(
    mjMINVAL, std::fabs(2*(dz*dz - dx*dx) + (dz*dz + dx*dx)*(kz - kx)));
  const double Izfac = pow2(dx*dx - dy*dy) * std::fabs(ky - kx) / std::max(
    mjMINVAL, std::fabs(2*(dx*dx - dy*dy) + (dx*dx + dy*dy)*(kx - ky)));

  const mjtNum virtual_mass[3] = {
      volume * kx / std::max(mjMINVAL, 2-kx),
      volume * ky / std::max(mjMINVAL, 2-ky),
      volume * kz / std::max(mjMINVAL, 2-kz)};
  const mjtNum virtual_inertia[3] = {volume*Ixfac/5, volume*Iyfac/5, volume*Izfac/5};

  writeFluidGeomInteraction(fluid, &fluid_switch, &fluid_coefs[0],
                            &fluid_coefs[1], &fluid_coefs[2],
                            &fluid_coefs[3], &fluid_coefs[4],
                            virtual_mass, virtual_inertia);
}



// compiler
void mjCGeom::Compile(void) {
  // resize userdata
  if (userdata.size() > model->nuser_geom) {
    throw mjCError(this, "user has more values than nuser_geom in geom '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata.resize(model->nuser_geom);

  // check type
  if (type<0 || type>=mjNGEOMTYPES) {
    throw mjCError(this, "invalid type in geom '%s' (id = %d)", name.c_str(), id);
  }

  // check condim
  if (condim!=1 && condim!=3 && condim!=4 && condim!=6) {
    throw mjCError(this, "invalid condim in geom '%s' (id = %d)", name.c_str(), id);
  }

  // check mesh
  if (type==mjGEOM_MESH && meshid<0) {
    throw mjCError(this, "mesh geom '%s' (id = %d) must have valid meshid", name.c_str(), id);
  }

  // check hfield
  if ((type==mjGEOM_HFIELD && hfieldid<0) || (type!=mjGEOM_HFIELD && hfieldid>=0)) {
    throw mjCError(this, "hfield geom '%s' (id = %d) must have valid hfieldid", name.c_str(), id);
  }

  // plane and hfield only allowed in static bodies
  if ((type==mjGEOM_PLANE || type==mjGEOM_HFIELD) && body->weldid!=0) {
    throw mjCError(this, "plane and hfield only allowed in static bodies: geom '%s' (id = %d)",
                   name.c_str(), id);
  }

  // normalize quaternion
  mjuu_normvec(quat, 4);

  // 'fromto': compute pos, quat, size
  if (mjuu_defined(fromto[0])) {
    // check type
    if (type!=mjGEOM_CAPSULE &&
        type!=mjGEOM_CYLINDER &&
        type!=mjGEOM_ELLIPSOID &&
        type!=mjGEOM_BOX) {
      throw mjCError(this,
                     "fromto requires capsule, cylinder, box or ellipsoid in geom '%s' (id = %d)",
                     name.c_str(), id);
    }

    // make sure pos is not defined; cannot use mjuu_defined because default is (0,0,0)
    if (pos[0] || pos[1] || pos[2]) {
      throw mjCError(this,
                     "both pos and fromto defined in geom '%s' (id = %d)",
                     name.c_str(), id);
    }

    // size[1] = length (for capsule and cylinder)
    double vec[3] = {
      fromto[0]-fromto[3],
      fromto[1]-fromto[4],
      fromto[2]-fromto[5]
    };
    size[1] = mjuu_normvec(vec, 3)/2;
    if (size[1]<mjEPS) {
      throw mjCError(this, "fromto points too close in geom '%s' (id = %d)", name.c_str(), id);
    }

    // adjust size for ellipsoid and box
    if (type==mjGEOM_ELLIPSOID || type==mjGEOM_BOX) {
      size[2] = size[1];
      size[1] = size[0];
    }

    // compute position
    pos[0] = (fromto[0]+fromto[3])/2;
    pos[1] = (fromto[1]+fromto[4])/2;
    pos[2] = (fromto[2]+fromto[5])/2;

    // compute orientation
    mjuu_z2quat(quat, vec);
  }

  // not 'fromto': try alternative
  else {
    const char* err = alt.Set(quat, inertia, model->degree, model->euler);
    if (err) {
      throw mjCError(this, "alternative specification error '%s' in geom %d", err, id);
    }
  }

  // mesh: accumulate frame, fit geom if needed
  if (meshid!=-1) {
    // check for inapplicable fromto
    if (mjuu_defined(fromto[0])) {
      throw mjCError(this, "fromto cannot be used with mesh geom '%s' (id = %d)", name.c_str(), id);
    }

    // get associated mesh
    mjCMesh* pmesh = model->meshes[meshid];

    // fit geom if type is not mjGEOM_MESH
    double meshpos[3];
    if (type!=mjGEOM_MESH) {
      pmesh->FitGeom(this, meshpos);

      // remove reference to mesh
      mesh.clear();
      meshid = -1;
    } else {
      mjuu_copyvec(meshpos, pmesh->GetPosPtr(typeinertia), 3);
    }

    // apply geom pos/quat as offset
    mjuu_frameaccum(pos, quat, meshpos, pmesh->GetQuatPtr(typeinertia));
  }

  // check size parameters
  checksize(size, type, this, name.c_str(), id);

  // set hfield sizes in geom.size
  if (type==mjGEOM_HFIELD) {
    size[0] = model->hfields[hfieldid]->size[0];
    size[1] = model->hfields[hfieldid]->size[1];
    size[2] = 0.5*(model->hfields[hfieldid]->size[2]+model->hfields[hfieldid]->size[3]);
  } else if (type==mjGEOM_MESH) {
    size[0] = model->meshes[meshid]->aabb[0];
    size[1] = model->meshes[meshid]->aabb[1];
    size[2] = model->meshes[meshid]->aabb[2];
  }

  // compute geom mass and inertia
  if (inferinertia) {
    if (mjuu_defined(_mass)) {
      if (_mass==0) {
        mass = 0;
        density = 0;
      } else if (GetVolume()>mjMINVAL) {
        mass = _mass;
        density = _mass / GetVolume();
        SetInertia();
      }
    } else {
      mass = density * GetVolume();
      SetInertia();
    }

    // check for negative values
    if (mass<0 || inertia[0]<0 || inertia[1]<0 || inertia[2]<0 || density<0)
      throw mjCError(this, "mass, inertia or density are negative in geom '%s' (id = %d)",
                    name.c_str(), id);
  }

  // fluid-interaction coefficients, requires computed inertia and mass
  if (fluid_switch > 0) {
    SetFluidCoefs();
  }
}



//------------------ class mjCSite implementation --------------------------------------------------

// initialize default site
mjCSite::mjCSite(mjCModel* _model, mjCDef* _def) {
  // set defaults
  type = mjGEOM_SPHERE;
  mjuu_setvec(size, 0.005, 0.005, 0.005);
  group = 0;
  mjuu_setvec(quat, 1, 0, 0, 0);
  mjuu_setvec(pos, 0, 0, 0);
  material.clear();
  rgba[0] = rgba[1] = rgba[2] = 0.5f;
  rgba[3] = 1.0f;
  fromto[0] = mjNAN;
  userdata.clear();

  // clear internal variables
  material.clear();
  body = 0;
  mjuu_setvec(locpos, 0, 0, 0);
  mjuu_setvec(locquat, 1, 0, 0, 0);
  matid = -1;

  // reset to default if given
  if (_def) {
    *this = _def->site;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



// compiler
void mjCSite::Compile(void) {
  // resize userdata
  if (userdata.size() > model->nuser_site) {
    throw mjCError(this, "user has more values than nuser_site in site '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata.resize(model->nuser_site);

  // check type
  if (type<0 || type>=mjNGEOMTYPES) {
    throw mjCError(this, "invalid type in site '%s' (id = %d)", name.c_str(), id);
  }

  // do not allow meshes, hfields and planes
  if (type==mjGEOM_MESH || type==mjGEOM_HFIELD || type==mjGEOM_PLANE) {
    throw mjCError(this, "meshes, hfields and planes not allowed in site '%s' (id = %d)",
                   name.c_str(), id);
  }

  // 'fromto': compute pos, quat, size
  if (mjuu_defined(fromto[0])) {
    // check type
    if (type!=mjGEOM_CAPSULE &&
        type!=mjGEOM_CYLINDER &&
        type!=mjGEOM_ELLIPSOID &&
        type!=mjGEOM_BOX) {
      throw mjCError(this,
                     "fromto requires capsule, cylinder, box or ellipsoid in geom '%s' (id = %d)",
                     name.c_str(), id);
    }

    // make sure pos is not defined; cannot use mjuu_defined because default is (0,0,0)
    if (pos[0] || pos[1] || pos[2]) {
      throw mjCError(this, "both pos and fromto defined in geom '%s' (id = %d)", name.c_str(), id);
    }

    // size[1] = length (for capsule and cylinder)
    double vec[3] = {
      fromto[0]-fromto[3],
      fromto[1]-fromto[4],
      fromto[2]-fromto[5]
    };
    size[1] = mjuu_normvec(vec, 3)/2;
    if (size[1]<mjEPS) {
      throw mjCError(this, "fromto points too close in geom '%s' (id = %d)", name.c_str(), id);
    }

    // adjust size for ellipsoid and box
    if (type==mjGEOM_ELLIPSOID || type==mjGEOM_BOX) {
      size[2] = size[1];
      size[1] = size[0];
    }

    // compute position
    pos[0] = (fromto[0]+fromto[3])/2;
    pos[1] = (fromto[1]+fromto[4])/2;
    pos[2] = (fromto[2]+fromto[5])/2;

    // compute orientation
    mjuu_z2quat(quat, vec);
  }

  // alternative orientation
  else {
    const char* err = alt.Set(quat, 0, model->degree, model->euler);
    if (err) {
      throw mjCError(this, "alternative specification error '%s' in site %d", err, id);
    }
  }

  // normalize quaternion
  mjuu_normvec(quat, 4);

  // check size parameters
  checksize(size, type, this, name.c_str(), id);

  // ask parent body to compute our local pos and quat relative to itself
  body->MakeLocal(locpos, locquat, pos, quat);
}



//------------------ class mjCCamera implementation ------------------------------------------------

// initialize defaults
mjCCamera::mjCCamera(mjCModel* _model, mjCDef* _def) {
  // set defaults
  mode = mjCAMLIGHT_FIXED;
  targetbody.clear();
  mjuu_setvec(pos, 0, 0, 0);
  mjuu_setvec(quat, 1, 0, 0, 0);
  fovy = 45;
  ipd = 0.068;
  userdata.clear();

  // clear private variables
  body = 0;
  mjuu_setvec(locpos, 0, 0, 0);
  mjuu_setvec(locquat, 1, 0, 0, 0);
  targetbodyid = -1;

  // reset to default if given
  if (_def) {
    *this = _def->camera;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



// compiler
void mjCCamera::Compile(void) {
  // resize userdata
  if (userdata.size() > model->nuser_cam) {
    throw mjCError(this, "user has more values than nuser_cam in camera '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata.resize(model->nuser_cam);

  // process orientation specifications
  const char* err = alt.Set(quat, 0, model->degree, model->euler);
  if (err) {
    throw mjCError(this, "alternative specification error '%s' in site %d", err, id);
  }

  // normalize quaternion
  mjuu_normvec(quat, 4);

  // ask parent body to compute our local pos and quat relative to itself
  body->MakeLocal(locpos, locquat, pos, quat);

  // get targetbodyid
  if (!targetbody.empty()) {
    mjCBody* tb = (mjCBody*)model->FindObject(mjOBJ_BODY, targetbody);
    if (tb) {
      targetbodyid = tb->id;
    } else {
      throw mjCError(this, "unknown target body in camera '%s' (id = %d)", name.c_str(), id);
    }
  }

  // make sure it is not targeting parent body
  if (targetbodyid==body->id) {
    throw mjCError(this, "parent-targeting in camera '%s' (id = %d)", name.c_str(), id);
  }
}



//------------------ class mjCLight implementation -------------------------------------------------

// initialize defaults
mjCLight::mjCLight(mjCModel* _model, mjCDef* _def) {
  // set defaults
  mode = mjCAMLIGHT_FIXED;
  targetbody.clear();
  directional = false;
  castshadow = true;
  active = true;
  mjuu_setvec(pos, 0, 0, 0);
  mjuu_setvec(dir, 0, 0, -1);
  mjuu_setvec(attenuation, 1, 0, 0);
  cutoff = 45;
  exponent = 10;
  ambient[0] = ambient[1] = ambient[2] = 0;
  diffuse[0] = diffuse[1] = diffuse[2] = 0.7;
  specular[0] = specular[1] = specular[2] = 0.3;

  // clear private variables
  body = 0;
  mjuu_setvec(locpos, 0, 0, 0);
  mjuu_setvec(locdir, 0, 0, 0);
  targetbodyid = -1;

  // reset to default if given
  if (_def) {
    *this = _def->light;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



// compiler
void mjCLight::Compile(void) {
  double locquat[4], quat[4]= {1, 0, 0, 0};

  // normalize direction, make sure it is not zero
  if (mjuu_normvec(dir, 3)<mjMINVAL) {
    throw mjCError(this, "zero direction in light '%s' (id = %d)", name.c_str(), id);
  }

  // ask parent body to compute our local pos and quat relative to itself
  body->MakeLocal(locpos, locquat, pos, quat);

  // copy/convert dir to local frame
  if (model->global) {
    double mat[9], q[4] = {locquat[0], -locquat[1], -locquat[2], -locquat[3]};
    mjuu_quat2mat(mat, q);
    mjuu_mulvecmat(locdir, dir, mat);
  } else {
    mjuu_copyvec(locdir, dir, 3);
  }

  // get targetbodyid
  if (!targetbody.empty()) {
    mjCBody* tb = (mjCBody*)model->FindObject(mjOBJ_BODY, targetbody);
    if (tb) {
      targetbodyid = tb->id;
    } else {
      throw mjCError(this, "unknown target body in light '%s' (id = %d)", name.c_str(), id);
    }
  }

  // make sure it is not self-targeting
  if (targetbodyid==body->id) {
    throw mjCError(this, "parent-targeting in light '%s' (id = %d)", name.c_str(), id);
  }
}



//------------------------- class mjCHField --------------------------------------------------------

// constructor
mjCHField::mjCHField(mjCModel* _model) {
  // set model pointer
  model = _model;

  // clear variables
  mjuu_setvec(size, 0, 0, 0, 0);
  file.clear();
  nrow = 0;
  ncol = 0;
  data = 0;
}



// destructor
mjCHField::~mjCHField() {
  if (data) {
    mju_free(data);
  }
}



// load elevation data from custom format
void mjCHField::LoadCustom(string filename, const mjVFS* vfs) {
  // get file data in buffer
  void* buffer = 0;
  int buffer_sz = 0, flag_existing = 0;
  if (vfs) {
    int id = mj_findFileVFS(vfs, filename.c_str());
    if (id>=0) {
      buffer = vfs->filedata[id];
      buffer_sz = vfs->filesize[id];
      flag_existing = 1;
    }
  }

  // if not found in vfs, read from file
  if (!buffer) {
    buffer = mju_fileToMemory(filename.c_str(), &buffer_sz);
  }

  // still not found
  if (!buffer || !buffer_sz) {
    throw mjCError(this, "could not open hfield file '%s'", filename.c_str());
  }

  if (buffer_sz < 2*sizeof(int)) {
    if (!flag_existing) {
      mju_free(buffer);
    }
    throw mjCError(this, "hfield missing header '%s'", filename.c_str());
  }

  // read dimensions
  int* pint = (int*)buffer;
  nrow = pint[0];
  ncol = pint[1];

  // check dimensions
  if (nrow<1 || ncol<1) {
    if (!flag_existing) {
      mju_free(buffer);
    }

    throw mjCError(this, "non-positive hfield dimensions in file '%s'", filename.c_str());
  }

  // check buffer size
  if (buffer_sz != nrow*ncol*sizeof(float)+8) {
    if (!flag_existing) {
      mju_free(buffer);
    }

    throw mjCError(this, "unexpected file size in file '%s'", filename.c_str());
  }

  // allocate
  data = (float*) mju_malloc(nrow*ncol*sizeof(float));
  if (!data) {
    if (!flag_existing) {
      mju_free(buffer);
    }

    throw mjCError(this, "could not allocate buffers in hfield");
  }

  // copy data
  memcpy(data, (void*)(pint+2), nrow*ncol*sizeof(float));

  // free buffer if allocated here
  if (!flag_existing) {
    mju_free(buffer);
  }
}



// load elevation data from PNG format
void mjCHField::LoadPNG(string filename, const mjVFS* vfs) {
  // determine data source
  const unsigned char* inbuffer = 0;
  size_t inbuffer_sz = 0;
  if (vfs) {
    int id = mj_findFileVFS(vfs, filename.c_str());
    if (id>=0) {
      inbuffer = (const unsigned char*)vfs->filedata[id];
      inbuffer_sz = (size_t)vfs->filesize[id];
    }
  }

  // load PNG from file or memory
  unsigned int w, h, err;
  std::vector<unsigned char> image;
  if (inbuffer_sz) {
    err = lodepng::decode(image, w, h, inbuffer, inbuffer_sz, LCT_GREY, 8);
  } else {
    err = lodepng::decode(image, w, h, filename, LCT_GREY, 8);
  }

  // check
  if (err) {
    throw mjCError(this, "PNG load error '%s' in hfield id = %d", lodepng_error_text(err), id);
  }
  if (!w || !h) {
    throw mjCError(this, "Zero dimension in PNG hfield '%s' (id = %d)", name.c_str(), id);
  }

  // allocate
  data = (float*) mju_malloc(w*h*sizeof(float));
  if (!data) {
    throw mjCError(this, "could not allocate buffers in hfield");
  }

  // assign and copy
  ncol = w;
  nrow = h;
  for (int c=0; c<ncol; c++)
    for (int r=0; r<nrow; r++) {
      data[c+(nrow-1-r)*ncol] = (float)image[c+r*ncol];
    }
  image.clear();
}



// compiler
void mjCHField::Compile(const mjVFS* vfs) {
  // check size parameters
  for (int i=0; i<4; i++)
    if (size[i]<=0)
      throw mjCError(this,
                     "size parameter is not positive in hfield '%s' (id = %d)", name.c_str(), id);

  // remove path from file if necessary
  if (model->strippath) {
    file = mjuu_strippath(file);
  }

  // load from file if specified
  if (!file.empty()) {
    // make sure hfield was not already specified manually
    if (nrow || ncol || data) {
      throw mjCError(this,
                     "hfield '%s' (id = %d) specified from file and manually", name.c_str(), id);
    }

    // make filename
    string filename = mjuu_makefullname(model->modelfiledir, model->meshdir, file);

    // load depending on format
    if (!strcasecmp(filename.substr(filename.length()-4, 5).c_str(), ".png")) {
      LoadPNG(filename, vfs);
    } else {
      LoadCustom(filename, vfs);
    }
  }

  // make sure hfield was specified (from file or manually)
  if (nrow<1 || ncol<1 || data==0) {
    throw mjCError(this, "hfield '%s' (id = %d) not specified", name.c_str(), id);
  }

  // set elevation data to [0-1] range
  float emin = 1E+10, emax = -1E+10;
  for (int i = 0; i<nrow*ncol; i++) {
    emin = mjMIN(emin, data[i]);
    emax = mjMAX(emax, data[i]);
  }
  if (emin>emax) {
    throw mjCError(this, "invalid data range in hfield '%s'", file.c_str());
  }
  for (int i=0; i<nrow*ncol; i++) {
    data[i] -= emin;
    if (emax-emin>mjMINVAL) {
      data[i] /= (emax - emin);
    }
  }
}



//------------------ class mjCTexture implementation -----------------------------------------------

// initialize defaults
mjCTexture::mjCTexture(mjCModel* _model) {
  // set model pointer
  model = _model;

  // clear user settings: builtin
  type = mjTEXTURE_CUBE;
  builtin = mjBUILTIN_NONE;
  mark = mjMARK_NONE;
  mjuu_setvec(rgb1, 0.8, 0.8, 0.8);
  mjuu_setvec(rgb2, 0.5, 0.5, 0.5);
  mjuu_setvec(markrgb, 0, 0, 0);
  random = 0.01;
  height = width = 0;

  // clear user settings: single file
  file.clear();
  gridsize[0] = gridsize[1] = 1;
  mju::strcpy_arr(gridlayout, "............");

  // clear user settings: separate file
  for (int i=0; i<6; i++) {
    cubefiles[i].clear();
  }

  // clear flip options
  hflip = false;
  vflip = false;

  // clear internal variables
  rgb = 0;
}



// free data storage allocated by lodepng
mjCTexture::~mjCTexture() {
  if (rgb) {
    mju_free(rgb);
    rgb = 0;
  }
}



// insert random dots
static void randomdot(unsigned char* rgb, const double* markrgb,
                      int width, int height, double probability) {
  int r, c, j;
  for (r=0; r<height; r++) {
    for (c=0; c<width; c++) {
      if (rand()<probability*RAND_MAX) {
        for (j=0; j<3; j++) {
          rgb[3*(r*width+c)+j] = (mjtByte)(255*markrgb[j]);
        }
      }
    }
  }
}



// interpolate between colors based on value in (-1, +1)
static void interp(unsigned char* rgb, const double* rgb1, const double* rgb2, double pos) {
  const double correction = 1.0/sqrt(2);
  double alpha = 0.5*(1 + pos/sqrt(1+pos*pos)/correction);
  if (alpha<0) {
    alpha = 0;
  } else if (alpha>1) {
    alpha = 1;
  }

  for (int j=0; j<3; j++) {
    rgb[j] = (mjtByte)(255*(alpha*rgb1[j] + (1-alpha)*rgb2[j]));
  }
}



// make checker pattern for one side
static void checker(unsigned char* rgb, const unsigned char* RGB1, const unsigned char* RGB2,
                    int width, int height) {
  int r, c;

  for (r=0; r<height/2; r++) {
    for (c=0; c<width/2; c++) {
      memcpy(rgb+3*(r*width+c), RGB1, 3);
    }
  }
  for (r=height/2; r<height; r++) {
    for (c=width/2; c<width; c++) {
      memcpy(rgb+3*(r*width+c), RGB1, 3);
    }
  }
  for (r=0; r<height/2; r++) {
    for (c=width/2; c<width; c++) {
      memcpy(rgb+3*(r*width+c), RGB2, 3);
    }
  }
  for (r=height/2; r<height; r++) {
    for (c=0; c<width/2; c++) {
      memcpy(rgb+3*(r*width+c), RGB2, 3);
    }
  }
}



// make builtin: 2D
void mjCTexture::Builtin2D(void) {
  unsigned char RGB1[3], RGB2[3], RGBm[3];
  int r, c, j;

  // convert fixed colors
  for (j=0; j<3; j++) {
    RGB1[j] = (mjtByte)(255*rgb1[j]);
    RGB2[j] = (mjtByte)(255*rgb2[j]);
    RGBm[j] = (mjtByte)(255*markrgb[j]);
  }

  //------------------ face

  // gradient
  if (builtin==mjBUILTIN_GRADIENT) {
    for (r=0; r<height; r++) {
      for (c=0; c<width; c++) {
        // compute normalized coordinates and radius
        double x = 2*c/((double)(width-1)) - 1;
        double y = 1 - 2*r/((double)(height-1));
        double pos = 2*sqrt(x*x+y*y) - 1;

        // interpolate through sigmoid
        interp(rgb + 3*(r*width+c), rgb2, rgb1, pos);
      }
    }
  }

  // checker
  else if (builtin==mjBUILTIN_CHECKER) {
    checker(rgb, RGB1, RGB2, width, height);
  }

  // flat
  else if (builtin==mjBUILTIN_FLAT) {
    for (r=0; r<height; r++) {
      for (c=0; c<width; c++) {
        memcpy(rgb+3*(r*width+c), RGB1, 3);
      }
    }
  }

  //------------------ marks

  // edge
  if (mark==mjMARK_EDGE) {
    for (r=0; r<height; r++) {
      memcpy(rgb+3*(r*width+0), RGBm, 3);
      memcpy(rgb+3*(r*width+width-1), RGBm, 3);
    }
    for (c=0; c<width; c++) {
      memcpy(rgb+3*(0*width+c), RGBm, 3);
      memcpy(rgb+3*((height-1)*width+c), RGBm, 3);
    }
  }

  // cross
  else if (mark==mjMARK_CROSS) {
    for (r=0; r<height; r++) {
      memcpy(rgb+3*(r*width+width/2), RGBm, 3);
    }
    for (c=0; c<width; c++) {
      memcpy(rgb+3*(height/2*width+c), RGBm, 3);
    }
  }

  // random dots
  else if (mark==mjMARK_RANDOM && random>0) {
    randomdot(rgb, markrgb, width, height, random);
  }
}



// make builtin: Cube
void mjCTexture::BuiltinCube(void) {
  unsigned char RGB1[3], RGB2[3], RGBm[3], RGBi[3];
  int r, c, j;

  // convert fixed colors
  for (j=0; j<3; j++) {
    RGB1[j] = (mjtByte)(255*rgb1[j]);
    RGB2[j] = (mjtByte)(255*rgb2[j]);
    RGBm[j] = (mjtByte)(255*markrgb[j]);
  }

  //------------------ faces

  // gradient
  if (builtin==mjBUILTIN_GRADIENT) {
    for (r=0; r<width; r++) {
      for (c=0; c<width; c++) {
        // compute normalized pixel coordinates
        double x = 2*c/((double)(width-1)) - 1;
        double y = 1 - 2*r/((double)(width-1));

        // compute normalized elevation for sides and up/down
        double elside = asin(y/sqrt(1+x*x+y*y)) / (0.5*mjPI);
        double elup = 1 - acos(1.0/sqrt(1+x*x+y*y)) / (0.5*mjPI);

        // set sides
        interp(RGBi, rgb1, rgb2, elside);
        memcpy(rgb+0*3*width*width+3*(r*width+c), RGBi, 3);     // 0: right
        memcpy(rgb+1*3*width*width+3*(r*width+c), RGBi, 3);     // 1: left
        memcpy(rgb+4*3*width*width+3*(r*width+c), RGBi, 3);     // 4: front
        memcpy(rgb+5*3*width*width+3*(r*width+c), RGBi, 3);     // 5: back

        // set up and down
        interp(rgb+2*3*width*width+3*(r*width+c), rgb1, rgb2, elup);    // 2: up
        interp(rgb+3*3*width*width+3*(r*width+c), rgb1, rgb2, -elup);   // 3: down
      }
    }
  }

  // checker
  else if (builtin==mjBUILTIN_CHECKER) {
    checker(rgb+0*3*width*width, RGB1, RGB2, width, width);
    checker(rgb+1*3*width*width, RGB1, RGB2, width, width);
    checker(rgb+2*3*width*width, RGB1, RGB2, width, width);
    checker(rgb+3*3*width*width, RGB1, RGB2, width, width);
    checker(rgb+4*3*width*width, RGB2, RGB1, width, width);
    checker(rgb+5*3*width*width, RGB2, RGB1, width, width);
  }

  // flat
  else if (builtin==mjBUILTIN_FLAT) {
    for (r=0; r<width; r++) {
      for (c=0; c<width; c++) {
        // set sides and up
        memcpy(rgb+0*3*width*width+3*(r*width+c), RGB1, 3);
        memcpy(rgb+1*3*width*width+3*(r*width+c), RGB1, 3);
        memcpy(rgb+2*3*width*width+3*(r*width+c), RGB1, 3);
        memcpy(rgb+4*3*width*width+3*(r*width+c), RGB1, 3);
        memcpy(rgb+5*3*width*width+3*(r*width+c), RGB1, 3);

        // set down
        memcpy(rgb+3*3*width*width+3*(r*width+c), RGB2, 3);
      }
    }
  }

  //------------------ marks

  // edge
  if (mark==mjMARK_EDGE) {
    for (j=0; j<6; j++) {
      for (r=0; r<width; r++) {
        memcpy(rgb+j*3*width*width+3*(r*width+0), RGBm, 3);
        memcpy(rgb+j*3*width*width+3*(r*width+width-1), RGBm, 3);
      }
      for (c=0; c<width; c++) {
        memcpy(rgb+j*3*width*width+3*(0*width+c), RGBm, 3);
        memcpy(rgb+j*3*width*width+3*((width-1)*width+c), RGBm, 3);
      }
    }
  }

  // cross
  else if (mark==mjMARK_CROSS) {
    for (j=0; j<6; j++) {
      for (r=0; r<width; r++) {
        memcpy(rgb+j*3*width*width+3*(r*width+width/2), RGBm, 3);
      }
      for (c=0; c<width; c++) {
        memcpy(rgb+j*3*width*width+3*(width/2*width+c), RGBm, 3);
      }
    }
  }

  // random dots
  else if (mark==mjMARK_RANDOM && random>0) {
    randomdot(rgb, markrgb, width, height, random);
  }
}



// load PNG file
void mjCTexture::LoadPNG(string filename, const mjVFS* vfs,
                         std::vector<unsigned char>& image,
                         unsigned int& w, unsigned int& h) {
  // determine data source
  const unsigned char* inbuffer = 0;
  size_t inbuffer_sz = 0;
  if (vfs) {
    int id = mj_findFileVFS(vfs, filename.c_str());
    if (id>=0) {
      inbuffer = (const unsigned char*)vfs->filedata[id];
      inbuffer_sz = (size_t)vfs->filesize[id];
    }
  }

  // load PNG from file or memory
  unsigned int err;
  if (inbuffer_sz) {
    err = lodepng::decode(image, w, h, inbuffer, inbuffer_sz, LCT_RGB, 8);
  } else {
    err = lodepng::decode(image, w, h, filename, LCT_RGB, 8);
  }

  // check
  if (err) {
    throw mjCError(this,
                   "PNG file load error '%s' in texture id = %d", lodepng_error_text(err), id);
  }
  if (w<1 || h<1) {
    throw mjCError(this, "Empty PNG file in texture '%s' (id %d)", (const char*)file.c_str(), id);
  }
}



// load custom file
void mjCTexture::LoadCustom(string filename, const mjVFS* vfs,
                            std::vector<unsigned char>& image,
                            unsigned int& w, unsigned int& h) {
  // get file data in buffer
  void* buffer = 0;
  int buffer_sz = 0, flag_existing = 0;
  if (vfs) {
    int id = mj_findFileVFS(vfs, filename.c_str());
    if (id>=0) {
      buffer = vfs->filedata[id];
      buffer_sz = vfs->filesize[id];
      flag_existing = 1;
    }
  }

  // if not found in vfs, read from file
  if (!buffer) {
    buffer = mju_fileToMemory(filename.c_str(), &buffer_sz);
  }

  // still not found
  if (!buffer || !buffer_sz) {
    throw mjCError(this, "could not open texture file '%s'", filename.c_str());
  }

  // read dimensions
  int* pint = (int*)buffer;
  w = pint[0];
  h = pint[1];

  // check dimensions
  if (w<1 || h<1) {
    if (!flag_existing) {
      mju_free(buffer);
    }

    throw mjCError(this, "Non-PNG texture, assuming custom binary file format,\n"
                         "non-positive texture dimensions in file '%s'", filename.c_str());
  }

  // check buffer size
  if (buffer_sz != 2*sizeof(int) + w*h*3*sizeof(char)) {
    if (!flag_existing) {
      mju_free(buffer);
    }

    throw mjCError(this, "Non-PNG texture, assuming custom binary file format,\n"
                         "unexpected file size in file '%s'", filename.c_str());
  }

  // allocate and copy
  image.resize(w*h*3);
  memcpy(image.data(), (void*)(pint+2), w*h*3*sizeof(char));

  // free buffer if allocated here
  if (!flag_existing) {
    mju_free(buffer);
  }
}



// load from PNG or custom file, flip if specified
void mjCTexture::LoadFlip(string filename, const mjVFS* vfs,
                          std::vector<unsigned char>& image,
                          unsigned int& w, unsigned int& h) {
  // dispatch to PNG or Custom loaded
  if (!strcasecmp(filename.substr(filename.length()-4, 5).c_str(), ".png")) {
    LoadPNG(filename, vfs, image, w, h);
  } else {
    LoadCustom(filename, vfs, image, w, h);
  }

  // horizontal flip
  if (hflip) {
    for (int r=0; r<h; r++) {
      for (int c=0; c<w/2; c++) {
        int c1 = w-1-c;
        unsigned char tmp[3] = {
          image[3*(r*w+c)],
          image[3*(r*w+c)+1],
          image[3*(r*w+c)+2]
        };

        image[3*(r*w+c)]   = image[3*(r*w+c1)];
        image[3*(r*w+c)+1] = image[3*(r*w+c1)+1];
        image[3*(r*w+c)+2] = image[3*(r*w+c1)+2];

        image[3*(r*w+c1)]   = tmp[0];
        image[3*(r*w+c1)+1] = tmp[1];
        image[3*(r*w+c1)+2] = tmp[2];
      }
    }
  }

  // vertical flip
  if (vflip) {
    for (int r=0; r<h/2; r++) {
      for (int c=0; c<w; c++) {
        int r1 = h-1-r;
        unsigned char tmp[3] = {
          image[3*(r*w+c)],
          image[3*(r*w+c)+1],
          image[3*(r*w+c)+2]
        };

        image[3*(r*w+c)]   = image[3*(r1*w+c)];
        image[3*(r*w+c)+1] = image[3*(r1*w+c)+1];
        image[3*(r*w+c)+2] = image[3*(r1*w+c)+2];

        image[3*(r1*w+c)]   = tmp[0];
        image[3*(r1*w+c)+1] = tmp[1];
        image[3*(r1*w+c)+2] = tmp[2];
      }
    }
  }
}



// load 2D
void mjCTexture::Load2D(string filename, const mjVFS* vfs) {
  // load PNG or custom
  unsigned int w, h;
  std::vector<unsigned char> image;
  LoadFlip(filename, vfs, image, w, h);

  // assign size
  width = w;
  height = h;

  // allocate and copy data
  rgb = (mjtByte*) mju_malloc(3*width*height);
  if (!rgb) {
    throw mjCError(this, "Could not allocate memory for texture '%s' (id %d)",
                   (const char*)file.c_str(), id);
  }
  memcpy(rgb, image.data(), 3*width*height);
  image.clear();
}



// load cube or skybox from single file (repeated or grid)
void mjCTexture::LoadCubeSingle(string filename, const mjVFS* vfs) {
  int i, j, k, s;

  // check gridsize
  if (gridsize[0]<1 || gridsize[1]<1 || gridsize[0]*gridsize[1]>12) {
    throw mjCError(this,
                   "gridsize must be non-zero and no more than 12 squares in texture '%s' (id %d)",
                   (const char*)name.c_str(), id);
  }

  // load PNG or custom
  unsigned int w, h;
  std::vector<unsigned char> image;
  LoadFlip(filename, vfs, image, w, h);

  // check gridsize for compatibility
  if (w/gridsize[1]!=h/gridsize[0] || (w%gridsize[1]) || (h%gridsize[0])) {
    throw mjCError(this,
                   "PNG size must be integer multiple of gridsize in texture '%s' (id %d)",
                   (const char*)file.c_str(), id);
  }

  // assign size: repeated or full
  if (gridsize[0]==1 && gridsize[1]==1) {
    width = height = w;
  } else {
    width = w/gridsize[1];
    height = 6*width;
  }

  // allocate data
  rgb = (mjtByte*) mju_malloc(3*width*height);
  if (!rgb) {
    throw mjCError(this,
                   "Could not allocate memory for texture '%s' (id %d)",
                   (const char*)file.c_str(), id);
  }

  // copy: repeated
  if (gridsize[0]==1 && gridsize[1]==1) {
    memcpy(rgb, image.data(), 3*width*width);
  }

  // copy: grid
  else {
    // keep track of which faces were defined
    int loaded[6] = {0, 0, 0, 0, 0, 0};

    // process grid
    for (k=0; k<gridsize[0]*gridsize[1]; k++) {
      // decode face symbol
      i = -1;
      if (gridlayout[k]=='R') {
        i = 0;
      } else if (gridlayout[k]=='L') {
        i = 1;
      } else if (gridlayout[k]=='U') {
        i = 2;
      } else if (gridlayout[k]=='D') {
        i = 3;
      } else if (gridlayout[k]=='F') {
        i = 4;
      } else if (gridlayout[k]=='B') {
        i = 5;
      } else if (gridlayout[k]!='.')
        throw mjCError(this, "gridlayout symbol is not among '.RLUDFB' in texture '%s' (id %d)",
                       (const char*)file.c_str(), id);

      // load if specified
      if (i>=0) {
        // extract sub-image
        int rstart = width*(k/gridsize[1]);
        int cstart = width*(k%gridsize[1]);
        for (j=0; j<width; j++) {
          memcpy(rgb+i*3*width*width+j*3*width, image.data()+(j+rstart)*3*w+3*cstart, 3*width);
        }

        // mark as defined
        loaded[i] = 1;
      }
    }

    // set undefined faces to rgb1
    for (i=0; i<6; i++) {
      if (!loaded[i]) {
        for (k=0; k<width; k++) {
          for (s=0; s<width; s++) {
            for (j=0; j<3; j++) {
              rgb[i*3*width*width + 3*(k*width+s) + j] = (mjtByte)(255*rgb1[j]);
            }
          }
        }
      }
    }
  }

  image.clear();
}



// load cube or skybox from separate file
void mjCTexture::LoadCubeSeparate(const mjVFS* vfs) {
  int i, j, k, s;

  // keep track of which faces were defined
  int loaded[6] = {0, 0, 0, 0, 0, 0};

  // process nonempty files
  for (i=0; i<6; i++) {
    if (!cubefiles[i].empty()) {
      // remove path from file if necessary
      if (model->strippath) {
        cubefiles[i] = mjuu_strippath(cubefiles[i]);
      }

      // make filename
      string filename = mjuu_makefullname(model->modelfiledir, model->texturedir, cubefiles[i]);

      // load PNG or custom
      unsigned int w, h;
      std::vector<unsigned char> image;
      LoadFlip(filename, vfs, image, w, h);

      // PNG must be square
      if (w!=h) {
        throw mjCError(this,
                       "Non-square PNG file '%s' in cube or skybox id %d",
                       (const char*)cubefiles[i].c_str(), id);
      }

      // first file: set size and allocate data
      if (!rgb) {
        width = w;
        height = 6*width;
        rgb = (mjtByte*) mju_malloc(3*width*height);
        if (!rgb) {
          throw mjCError(this,
                         "Could not allocate memory for texture '%s' (id %d)",
                         (const char*)name.c_str(), id);
        }
      }

      // otherwise check size
      else if (width!=w) {
        throw mjCError(this,
                       "PNG file '%s' has incompatible size in texture id %d",
                       (const char*)cubefiles[i].c_str(), id);
      }

      // copy data
      memcpy(rgb+i*3*width*width, image.data(), 3*width*width);
      image.clear();

      // mark as defined
      loaded[i] = 1;
    }
  }

  // set undefined faces to rgb1
  for (i=0; i<6; i++) {
    if (!loaded[i]) {
      for (k=0; k<width; k++) {
        for (s=0; s<width; s++) {
          for (j=0; j<3; j++) {
            rgb[i*3*width*width + 3*(k*width+s) + j] = (mjtByte)(255*rgb1[j]);
          }
        }
      }
    }
  }
}



// compiler
void mjCTexture::Compile(const mjVFS* vfs) {
  // builtin
  if (builtin!=mjBUILTIN_NONE) {
    // check size
    if (width<1 || height<1) {
      throw mjCError(this,
                     "Invalid width or height of builtin texture '%s' (id %d)",
                     (const char*)name.c_str(), id);
    }

    // adjust height of cube texture
    if (type!=mjTEXTURE_2D) {
      height = 6*width;
    }

    // allocate data
    rgb = (mjtByte*) mju_malloc(3*width*height);
    if (!rgb) {
      throw mjCError(this,
                     "Could not allocate memory for texture '%s' (id %d)",
                     (const char*)name.c_str(), id);
    }

    // dispatch
    if (type==mjTEXTURE_2D) {
      Builtin2D();
    } else {
      BuiltinCube();
    }
  }

  // single file
  else if (!file.empty()) {
    // remove path from file if necessary
    if (model->strippath) {
      file = mjuu_strippath(file);
    }

    // make filename
    string filename = mjuu_makefullname(model->modelfiledir, model->texturedir, file);

    // dispatch
    if (type==mjTEXTURE_2D) {
      Load2D(filename, vfs);
    } else {
      LoadCubeSingle(filename, vfs);
    }
  }

  // separate files
  else {
    // 2D not allowed
    if (type==mjTEXTURE_2D) {
      throw mjCError(this,
                     "Cannot load 2D texture from separate files, texture '%s' (id %d)",
                     (const char*)name.c_str(), id);
    }

    // at least one cubefile must be defined
    bool defined = false;
    for (int i=0; i<6; i++) {
      if (!cubefiles[i].empty()) {
        defined = true;
        break;
      }
    }
    if (!defined) {
      throw mjCError(this,
                     "No cubefiles defined in cube or skybox texture '%s' (id %d)",
                     (const char*)name.c_str(), id);
    }

    // only cube and skybox
    LoadCubeSeparate(vfs);
  }

  // make sure someone allocated data; SHOULD NOT OCCUR
  if (!rgb) {
    throw mjCError(this,
                   "texture '%s' (id %d) was not specified", (const char*)name.c_str(), id);
  }
}



//------------------ class mjCMaterial implementation ----------------------------------------------

// initialize defaults
mjCMaterial::mjCMaterial(mjCModel* _model, mjCDef* _def) {
  // set defaults
  texture.clear();
  texid = -1;
  texuniform = false;
  texrepeat[0] = texrepeat[1] = 1;
  emission = 0;
  specular = 0.5;
  shininess = 0.5;
  reflectance = 0;
  rgba[0] = rgba[1] = rgba[2] = rgba[3] = 1;

  // reset to default if given
  if (_def) {
    *this = _def->material;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



// compiler
void mjCMaterial::Compile(void) {
  // nothing to do for now
}



//------------------ class mjCPair implementation --------------------------------------------------

// constructor
mjCPair::mjCPair(mjCModel* _model, mjCDef* _def) {
  // set defaults
  geomname1.clear();
  geomname2.clear();

  condim = 3;
  mj_defaultSolRefImp(solref, solimp);
  margin = 0;
  gap = 0;
  friction[0] = 1;
  friction[1] = 1;
  friction[2] = 0.005;
  friction[3] = 0.0001;
  friction[4] = 0.0001;

  // clear internal variables
  geom1 = geom2 = signature = -1;

  // reset to default if given
  if (_def) {
    *this = _def->pair;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



// compiler
void mjCPair::Compile(void) {
  // check condim
  if (condim!=1 && condim!=3 && condim!=4 && condim!=6) {
    throw mjCError(this, "invalid condim in collision %d", "", id);
  }

  // find geom 1
  mjCGeom* pg1 = (mjCGeom*)model->FindObject(mjOBJ_GEOM, geomname1);
  if (!pg1) {
    throw mjCError(this, "geom '%s' not found in collision %d", geomname1.c_str(), id);
  }

  // find geom 2
  mjCGeom* pg2 = (mjCGeom*)model->FindObject(mjOBJ_GEOM, geomname2);
  if (!pg2) {
    throw mjCError(this, "geom '%s' not found in collision %d", geomname2.c_str(), id);
  }

  // swap if body1 > body2
  if (pg1->body->id > pg2->body->id) {
    string nametmp = geomname1;
    geomname1 = geomname2;
    geomname2 = nametmp;

    mjCGeom* geomtmp = pg1;
    pg1 = pg2;
    pg2 = geomtmp;
  }

  // get geom ids and body signature
  geom1 = pg1->id;
  geom2 = pg2->id;
  signature = ((pg1->body->id+1)<<16) + pg2->body->id+1;

  // set undefined margin: max
  if (!mjuu_defined(margin)) {
    margin = mjMAX(pg1->margin, pg2->margin);
  }

  // set undefined gap: max
  if (!mjuu_defined(gap)) {
    gap = mjMAX(pg1->gap, pg2->gap);
  }

  // set undefined condim, friction, solref, solimp: different priority
  if (pg1->priority!=pg2->priority) {
    mjCGeom* pgh = (pg1->priority>pg2->priority ? pg1 : pg2);

    // condim
    if (condim<0) {
      condim = pgh->condim;
    }

    // friction
    if (!mjuu_defined(friction[0])) {
      friction[0] = friction[1] = pgh->friction[0];
      friction[2] =               pgh->friction[1];
      friction[3] = friction[4] = pgh->friction[2];
    }

    // reference
    if (!mjuu_defined(solref[0])) {
      for (int i=0; i<mjNREF; i++) {
        solref[i] = pgh->solref[i];
      }
    }

    // impedance
    if (!mjuu_defined(solimp[0])) {
      for (int i=0; i<mjNIMP; i++) {
        solimp[i] = pgh->solimp[i];
      }
    }
  }

  // set undefined condim, friction, solref, solimp: same priority
  else {
    // condim: max
    if (condim<0) {
      condim = mjMAX(pg1->condim, pg2->condim);
    }

    // friction: max
    if (!mjuu_defined(friction[0])) {
      friction[0] = friction[1] = mjMAX(pg1->friction[0], pg2->friction[0]);
      friction[2] =               mjMAX(pg1->friction[1], pg2->friction[1]);
      friction[3] = friction[4] = mjMAX(pg1->friction[2], pg2->friction[2]);
    }

    // solver mix factor
    double mix;
    if (pg1->solmix>=mjMINVAL && pg2->solmix>=mjMINVAL) {
      mix = pg1->solmix / (pg1->solmix + pg2->solmix);
    } else if (pg1->solmix<mjMINVAL && pg2->solmix<mjMINVAL) {
      mix = 0.5;
    } else if (pg1->solmix<mjMINVAL) {
      mix = 0.0;
    } else {
      mix = 1.0;
    }

    // reference
    if (!mjuu_defined(solref[0])) {
      // standard: mix
      if (solref[0]>0) {
        for (int i=0; i<mjNREF; i++) {
          solref[i] = mix*pg1->solref[i] + (1-mix)*pg2->solref[i];
        }
      }

      // direct: min
      else {
        for (int i=0; i<mjNREF; i++) {
          solref[i] = mju_min(pg1->solref[i], pg2->solref[i]);
        }
      }
    }

    // impedance
    if (!mjuu_defined(solimp[0])) {
      for (int i=0; i<mjNIMP; i++) {
        solimp[i] = mix*pg1->solimp[i] + (1-mix)*pg2->solimp[i];
      }
    }
  }
}



//------------------ class mjCBodyPair implementation ----------------------------------------------

// constructor
mjCBodyPair::mjCBodyPair(mjCModel* _model) {
  // set model pointer
  model = _model;

  // set defaults
  bodyname1.clear();
  bodyname2.clear();

  // clear internal variables
  body1 = body2 = signature = -1;
}



// compiler
void mjCBodyPair::Compile(void) {
  // find body 1
  mjCBody* pb1 = (mjCBody*)model->FindObject(mjOBJ_BODY, bodyname1);
  if (!pb1) {
    throw mjCError(this, "body '%s' not found in bodypair %d", bodyname1.c_str(), id);
  }

  // find body 2
  mjCBody* pb2 = (mjCBody*)model->FindObject(mjOBJ_BODY, bodyname2);
  if (!pb2) {
    throw mjCError(this, "body '%s' not found in bodypair %d", bodyname2.c_str(), id);
  }

  // swap if body1 > body2
  if (pb1->id > pb2->id) {
    string nametmp = bodyname1;
    bodyname1 = bodyname2;
    bodyname2 = nametmp;

    mjCBody* bodytmp = pb1;
    pb1 = pb2;
    pb2 = bodytmp;
  }

  // get body ids and body signature
  body1 = pb1->id;
  body2 = pb2->id;
  signature = ((body1+1)<<16) + body2+1;
}



//------------------ class mjCEquality implementation ----------------------------------------------

// initialize default constraint
mjCEquality::mjCEquality(mjCModel* _model, mjCDef* _def) {
  // set defaults
  type = mjEQ_CONNECT;
  name1.clear();
  name2.clear();
  active = true;
  mj_defaultSolRefImp(solref, solimp);

  mjuu_zerovec(data, mjNEQDATA);
  data[1] = 1;
  data[10] = 1;  // torque:force ratio

  // clear internal variables
  obj1id = obj2id = -1;

  // reset to default if given
  if (_def) {
    *this = _def->equality;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}


// compiler
void mjCEquality::Compile(void) {
  mjtObj objtype;
  mjCBase *px1, *px2;
  mjtJoint jt1, jt2;
  double anchor[3], qdummy[4], qunit[4] = {1, 0, 0, 0};

  // determine object type
  if (type==mjEQ_CONNECT || type==mjEQ_WELD) {
    objtype = mjOBJ_BODY;
  } else if (type==mjEQ_JOINT) {
    objtype = mjOBJ_JOINT;
  } else if (type==mjEQ_TENDON) {
    objtype = mjOBJ_TENDON;
  } else if (type==mjEQ_DISTANCE) {
    objtype = mjOBJ_GEOM;
  } else {
    throw mjCError(this, "invalid type in equality constraint '%s' (id = %d)'", name.c_str(), id);
  }

  // find object 1, get id
  px1 = model->FindObject(objtype, name1);
  if (!px1) {
    throw mjCError(this, "unknown element '%s' in equality constraint %d", name1.c_str(), id);
  }
  obj1id = px1->id;

  // find object 2, get id
  if (!name2.empty()) {
    px2 = model->FindObject(objtype, name2);
    if (!px2) {
      throw mjCError(this, "unknown element '%s' in equality constraint %d", name2.c_str(), id);
    }
    obj2id = px2->id;
  }

  // object 2 unspecified: set to -1, except for distance
  else {
    if (objtype==mjOBJ_GEOM) {
      throw mjCError(this, "both geom are required in equality constraint '%s' (id = %d)",
                     name.c_str(), id);
    } else {
      obj2id = -1;
      px2 = 0;
    }
  }

  // set missing body = world
  if (objtype==mjOBJ_BODY && obj2id==-1) {
    obj2id = 0;
  }

  // make sure the two objects are different
  if (obj1id==obj2id) {
    throw mjCError(this, "element '%s' is repeated in equality constraint %d", name1.c_str(), id);
  }

  // make sure joints are scalar
  if (type==mjEQ_JOINT) {
    jt1 = ((mjCJoint*)px1)->type;
    jt2 = (px2 ? ((mjCJoint*)px2)->type : mjJNT_HINGE);
    if ((jt1!=mjJNT_HINGE && jt1!=mjJNT_SLIDE) ||
        (jt2!=mjJNT_HINGE && jt2!=mjJNT_SLIDE)) {
      throw mjCError(this, "only HINGE and SLIDE joint allowed in constraint '%s' (id = %d)",
                     name.c_str(), id);
    }
  }

  // connect: convert anchor to body1 local coordinates
  if (type==mjEQ_CONNECT) {
    ((mjCBody*)px1)->MakeLocal(anchor, qdummy, data, qunit);
    mjuu_copyvec(data, anchor, 3);
  } else if (type==mjEQ_WELD) {
    if (px2) {
      ((mjCBody*)px2)->MakeLocal(anchor, qdummy, data, qunit);
      mjuu_copyvec(data, anchor, 3);
    }
  }
}



//------------------ class mjCTendon implementation ------------------------------------------------

// constructor
mjCTendon::mjCTendon(mjCModel* _model, mjCDef* _def) {
  // tendon defaults
  group = 0;
  material.clear();
  width = 0.003;
  limited = 2;
  range[0] = 0;
  range[1] = 0;
  mj_defaultSolRefImp(solref_limit, solimp_limit);
  mj_defaultSolRefImp(solref_friction, solimp_friction);
  margin = 0;
  stiffness = 0;
  damping = 0;
  frictionloss = 0;
  springlength = -1;
  rgba[0] = rgba[1] = rgba[2] = 0.5f;
  rgba[3] = 1.0f;
  userdata.clear();

  // clear internal variables
  path.clear();
  matid = -1;

  // reset to default if given
  if (_def) {
    *this = _def->tendon;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



// desctructor
mjCTendon::~mjCTendon() {
  // delete objects allocated here
  for (unsigned int i=0; i<path.size(); i++) {
    delete path[i];
  }

  path.clear();
}



// add site as wrap object
void mjCTendon::WrapSite(string name, int row, int col) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->xmlpos[0] = row;
  wrap->xmlpos[1] = col;

  // set parameters, add to path
  wrap->type = mjWRAP_SITE;
  wrap->name = name;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add geom (with side site) as wrap object
void mjCTendon::WrapGeom(string name, string sidesite, int row, int col) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->xmlpos[0] = row;
  wrap->xmlpos[1] = col;

  // set parameters, add to path
  wrap->type = mjWRAP_SPHERE;         // replace with cylinder later if needed
  wrap->name = name;
  wrap->sidesite = sidesite;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add joint as wrap object
void mjCTendon::WrapJoint(string name, double coef, int row, int col) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->xmlpos[0] = row;
  wrap->xmlpos[1] = col;

  // set parameters, add to path
  wrap->type = mjWRAP_JOINT;
  wrap->name = name;
  wrap->prm = coef;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add pulley
void mjCTendon::WrapPulley(double divisor, int row, int col) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->xmlpos[0] = row;
  wrap->xmlpos[1] = col;

  // set parameters, add to path
  wrap->type = mjWRAP_PULLEY;
  wrap->prm = divisor;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// get number of wraps
int mjCTendon::NumWraps(void) {
  return (int)path.size();
}



// get pointer to specified wrap
mjCWrap* mjCTendon::GetWrap(int id) {
  if (id>=0 && id<(int)path.size()) {
    return path[id];
  } else {
    return 0;
  }
}



// compiler
void mjCTendon::Compile(void) {
  // resize userdata
  if (userdata.size() > model->nuser_tendon) {
    throw mjCError(this, "user has more values than nuser_tendon in tendon '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata.resize(model->nuser_tendon);

  // check for empty path
  int sz = (int)path.size();
  if (!sz) {
    throw mjCError(this,
                   "tendon '%s' (id = %d): path cannot be empty",
                   name.c_str(), id);
  }

  // determine type
  bool spatial = (path[0]->type!=mjWRAP_JOINT);

  // require at least two objects in spatial path
  if (spatial && sz<2) {
    throw mjCError(this, "tendon '%s' (id = %d): spatial path must contain at least two objects",
                   name.c_str(), id);
  }

  // require positive width
  if (spatial && width<=0) {
    throw mjCError(this, "tendon '%s' (id = %d) must have positive width", name.c_str(), id);
  }

  // compile objects in path
  for (int i=0; i<sz; i++) {
    path[i]->Compile();
  }

  // check path
  for (int i=0; i<sz; i++) {
    // fixed
    if (!spatial) {
      // make sure all objects are joints
      if (path[i]->type!=mjWRAP_JOINT) {
        throw mjCError(this, "tendon '%s' (id = %d): spatial object found in fixed path at pos %d",
                       name.c_str(), id, i);
      }
    }

    // spatial path
    else {
      switch (path[i]->type) {
      case mjWRAP_PULLEY:
        // pulley should not follow other pulley
        if (i>0 && path[i-1]->type==mjWRAP_PULLEY) {
          throw mjCError(this, "tendon '%s' (id = %d): consequtive pulleys (pos %d)",
                         name.c_str(), id, i);
        }

        // pulley should not be last
        if (i==sz-1) {
          throw mjCError(this, "tendon '%s' (id = %d): path ends with pulley", name.c_str(), id);
        }
        break;

      case mjWRAP_SITE:
        // site needs a neighbor that is not a pulley
        if ((i==0 || path[i-1]->type==mjWRAP_PULLEY) &&
            (i==sz-1 || path[i+1]->type==mjWRAP_PULLEY)) {
          throw mjCError(this,
                         "tendon '%s' (id = %d): site %d needs a neighbor that is not a pulley",
                         name.c_str(), id, i);
        }

        // site cannot be repeated
        if (i<sz-1 && path[i+1]->type==mjWRAP_SITE && path[i]->objid==path[i+1]->objid) {
          throw mjCError(this,
                         "tendon '%s' (id = %d): site %d is repeated",
                         name.c_str(), id, i);
        }

        break;

      case mjWRAP_SPHERE:
      case mjWRAP_CYLINDER:
        // geom must be bracketed by sites
        if (i==0 || i==sz-1 || path[i-1]->type!=mjWRAP_SITE || path[i+1]->type!=mjWRAP_SITE) {
          throw mjCError(this,
                         "tendon '%s' (id = %d): geom at pos %d not bracketed by sites",
                         name.c_str(), id, i);
        }
        break;

      case mjWRAP_JOINT:
        throw mjCError(this,
                       "tendon '%s (id = %d)': joint wrap found in spatial path at pos %d",
                       name.c_str(), id, i);

      default:
        throw mjCError(this,
                       "tendon '%s (id = %d)': invalid wrap object at pos %d",
                       name.c_str(), id, i);
      }
    }
  }

  // if limited is auto, set to 1 if range is specified, otherwise unlimited
  if (limited==2) {
    bool hasrange = !(range[0]==0 && range[1]==0);
    checklimited(this, model->autolimits, "tendon", "", limited, hasrange);
    limited = hasrange ? 1 : 0;
  }

  // check limits
  if (range[0]>=range[1] && limited) {
    throw mjCError(this, "invalid limits in tendon '%s (id = %d)'", name.c_str(), id);
  }
}



//------------------ class mjCWrap implementation --------------------------------------------------

// constructor
mjCWrap::mjCWrap(mjCModel* _model, mjCTendon* _tendon) {
  // set model and tendon pointer
  model = _model;
  tendon = _tendon;

  // clear variables
  type = mjWRAP_NONE;
  objid = -1;
  sideid = -1;
  prm = 0;
  sidesite.clear();
}



// compiler
void mjCWrap::Compile(void) {
  mjCBase *ptr = 0, *pside;

  // handle wrap object types
  switch (type) {
  case mjWRAP_JOINT:                          // joint
    // find joint by name
    ptr = model->FindObject(mjOBJ_JOINT, name);
    if (!ptr) {
      throw mjCError(this,
                     "joint '%s' not found in tendon %d, wrap %d",
                     name.c_str(), tendon->id, id);
    }

    break;

  case mjWRAP_SPHERE:                         // geom (cylinder type set here)
    // find geom by name
    ptr = model->FindObject(mjOBJ_GEOM, name);
    if (!ptr) {
      throw mjCError(this,
                     "geom '%s' not found in tendon %d, wrap %d",
                     name.c_str(), tendon->id, id);
    }

    // set/check geom type
    if (((mjCGeom*)ptr)->type == mjGEOM_CYLINDER) {
      type = mjWRAP_CYLINDER;
    } else if (((mjCGeom*)ptr)->type != mjGEOM_SPHERE) {
      throw mjCError(this,
                     "geom '%s' in tendon %d, wrap %d is not sphere or cylinder",
                     name.c_str(), tendon->id, id);
    }

    // process side site
    if (!sidesite.empty()) {
      // find site by name
      pside = model->FindObject(mjOBJ_SITE, sidesite);
      if (!pside) {
        throw mjCError(this,
                       "side site '%s' not found in tendon %d, wrap %d",
                       sidesite.c_str(), tendon->id, id);
      }

      // save side site id
      sideid = pside->id;
    }
    break;

  case mjWRAP_PULLEY:                         // pulley
    // make sure divisor is non-negative
    if (prm<0) {
      throw mjCError(this,
                     "pulley has negative divisor in tendon %d, wrap %d",
                     0, tendon->id, id);
    }

    break;

  case mjWRAP_SITE:                           // site
    // find site by name
    ptr = model->FindObject(mjOBJ_SITE, name);
    if (!ptr) {
      throw mjCError(this, "site '%s' not found in wrap %d", name.c_str(), id);
    }
    break;

  default:                                    // SHOULD NOT OCCUR
    throw mjCError(this, "unknown wrap type in tendon %d, wrap %d", 0, tendon->id, id);
  }

  // set object id
  if (ptr) {
    objid = ptr->id;
  }
}



//------------------ class mjCActuator implementation ----------------------------------------------

// initialize defaults
mjCActuator::mjCActuator(mjCModel* _model, mjCDef* _def) {
  // actuator defaults
  group = 0;
  ctrllimited = 2;
  forcelimited = 2;
  actlimited = 2;
  trntype = mjTRN_UNDEFINED;
  dyntype = mjDYN_NONE;
  gaintype = mjGAIN_FIXED;
  biastype = mjBIAS_NONE;
  mjuu_zerovec(dynprm, mjNDYN);
  mjuu_zerovec(gainprm, mjNGAIN);
  mjuu_zerovec(biasprm, mjNBIAS);
  mjuu_zerovec(ctrlrange, 2);
  mjuu_zerovec(forcerange, 2);
  mjuu_zerovec(actrange, 2);
  mjuu_zerovec(lengthrange, 2);
  mjuu_zerovec(gear, 6);
  gear[0] = 1;
  dynprm[0] = 1;
  gainprm[0] = 1;
  cranklength = 0;
  target.clear();
  slidersite.clear();
  refsite.clear();
  userdata.clear();

  // clear private variables
  trnid[0] = trnid[1] = -1;

  // reset to default if given
  if (_def) {
    *this = _def->actuator;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



// compiler
void mjCActuator::Compile(void) {
  mjCJoint* pjnt;

  // resize userdata
  if (userdata.size() > model->nuser_actuator) {
    throw mjCError(this, "user has more values than nuser_actuator in actuator '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata.resize(model->nuser_actuator);

  // if limited is auto, set to 1 if range is specified, otherwise unlimited
  if (forcelimited==2) {
    bool hasrange = !(forcerange[0]==0 && forcerange[1]==0);
    checklimited(this, model->autolimits, "actuator", "force", forcelimited, hasrange);
    forcelimited = hasrange ? 1 : 0;
  }
  if (ctrllimited==2) {
    bool hasrange = !(ctrlrange[0]==0 && ctrlrange[1]==0);
    checklimited(this, model->autolimits, "actuator", "ctrl", ctrllimited, hasrange);
    ctrllimited = hasrange ? 1 : 0;
  }
  if (actlimited==2) {
    bool hasrange = !(actrange[0]==0 && actrange[1]==0);
    checklimited(this, model->autolimits, "actuator", "act", actlimited, hasrange);
    actlimited = hasrange ? 1 : 0;
  }

  // check limits
  if (forcerange[0]>=forcerange[1] && forcelimited) {
    throw mjCError(this, "invalid force range for actuator '%s' (id = %d)", name.c_str(), id);
  }
  if (ctrlrange[0]>=ctrlrange[1] && ctrllimited) {
    throw mjCError(this, "invalid control range for actuator '%s' (id = %d)", name.c_str(), id);
  }
  if (actrange[0]>=actrange[1] && actlimited) {
    throw mjCError(this, "invalid activation range for actuator '%s' (id = %d)", name.c_str(), id);
  }
  if (actlimited && dyntype == mjDYN_NONE) {
    throw mjCError(this, "actrange specified but dyntype is 'none' in actuator '%s' (id = %d)",
                   name.c_str(), id);
  }

  // check muscle parameters
  for (int i=0; i<2; i++) {
    // select gain or bias
    double* prm = NULL;
    if (i==0 && gaintype==mjGAIN_MUSCLE) {
      prm = gainprm;
    } else if (i==1 && biastype==mjBIAS_MUSCLE) {
      prm = biasprm;
    }

    // nothing to check
    if (!prm) {
      continue;
    }

    // range
    if (prm[0]>=prm[1]) {
      throw mjCError(this, "range[0]<range[1] required in muscle '%s' (id = %d)", name.c_str(), id);
    }

    // lmin<1<lmax
    if (prm[4]>=1 || prm[5]<=1) {
      throw mjCError(this, "lmin<1<lmax required in muscle '%s' (id = %d)", name.c_str(), id);
    }

    // scale, vmax, fpmax, fvmax>0
    if (prm[3]<=0 || prm[6]<=0 || prm[7]<=0 || prm[8]<=0) {
      throw mjCError(this,
                     "positive scale, vmax, fpmax, fvmax required in muscle '%s' (id = %d)",
                     name.c_str(), id);
    }
  }

  // check for missing target name
  if (target.empty()) {
    throw mjCError(this,
                   "missing transmission target for actuator '%s' (id = %d)", name.c_str(), id);
  }

  // find transmission target in object arrays
  mjCBase* ptarget = 0;
  switch (trntype) {
  case mjTRN_JOINT:
  case mjTRN_JOINTINPARENT:
    // get joint
    ptarget = model->FindObject(mjOBJ_JOINT, target);
    if (!ptarget) {
      throw mjCError(this,
                     "unknown transmission target '%s' for actuator id = %d", target.c_str(), id);
    }
    pjnt = (mjCJoint*) ptarget;

    // apply urdfeffort
    if (pjnt->urdfeffort>0) {
      forcerange[0] = -pjnt->urdfeffort;
      forcerange[1] = pjnt->urdfeffort;
      forcelimited = 1;
    }
    break;

  case mjTRN_SLIDERCRANK:
    // get slidersite, copy in trnid[1]
    if (slidersite.empty()) {
      throw mjCError(this, "missing base site for slider-crank '%s' (id = %d)", name.c_str(), id);
    }
    ptarget = model->FindObject(mjOBJ_SITE, slidersite);
    if (!ptarget) {
      throw mjCError(this, "base site '%s' not found for actuator %d", slidersite.c_str(), id);
    }
    trnid[1] = ptarget->id;

    // check cranklength
    if (cranklength<=0) {
      throw mjCError(this,
                     "crank length must be positive in actuator '%s' (id = %d)", name.c_str(), id);
    }

    // proceed with regular target
    ptarget = model->FindObject(mjOBJ_SITE, target);
    break;

  case mjTRN_TENDON:
    // get tendon
    ptarget = model->FindObject(mjOBJ_TENDON, target);
    break;

  case mjTRN_SITE:
    // get refsite, copy into trnid[1]
    if (!refsite.empty()) {
      ptarget = model->FindObject(mjOBJ_SITE, refsite);
      if (!ptarget) {
        throw mjCError(this, "reference site '%s' not found for actuator %d", refsite.c_str(), id);
      }
      trnid[1] = ptarget->id;
    }

    // proceed with regular site target
    ptarget = model->FindObject(mjOBJ_SITE, target);
    break;

  case mjTRN_BODY:
    // get body
    ptarget = model->FindObject(mjOBJ_BODY, target);
    break;

  default:
    throw mjCError(this, "invalid transmission type in actuator '%s' (id = %d)", name.c_str(), id);
  }

  // assign and check
  if (!ptarget) {
    throw mjCError(this, "transmission target '%s' not found in actuator %d", target.c_str(), id);
  } else {
    trnid[0] = ptarget->id;
  }
}



//------------------ class mjCSensor implementation ------------------------------------------------

// initialize defaults
mjCSensor::mjCSensor(mjCModel* _model) {
  // set model
  model = _model;

  // set sensor defaults (somewhat arbitrary)
  type = mjSENS_TOUCH;
  datatype = mjDATATYPE_REAL;
  needstage = mjSTAGE_ACC;
  objtype = mjOBJ_UNKNOWN;
  objname.clear();
  reftype = mjOBJ_UNKNOWN;
  refname.clear();
  cutoff = 0;
  noise = 0;
  userdata.clear();
  dim = 0;

  // clear private variables
  objid = -1;
  refid = -1;
}



// compiler
void mjCSensor::Compile(void) {
  const mjCBase* pobj;

  // resize userdata
  if (userdata.size() > model->nuser_sensor) {
    throw mjCError(this, "user has more values than nuser_sensor in sensor '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata.resize(model->nuser_sensor);

  // require non-negative noise
  if (noise<0) {
    throw mjCError(this, "negative noise in sensor '%s' (id = %d)", name.c_str(), id);
  }

  // require non-negative cutoff
  if (cutoff<0) {
    throw mjCError(this, "negative cutoff in sensor '%s' (id = %d)", name.c_str(), id);
  }

  // get objid from objtype and objname
  if (objtype!=mjOBJ_UNKNOWN) {
    // check for missing object name
    if (objname.empty()) {
      throw mjCError(this,
                     "missing name of sensorized object in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }

    // find name
    pobj = model->FindObject(objtype, objname);
    if (!pobj) {
      throw mjCError(this,
                     "unrecognized name of sensorized object in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }

    // get sensorized object id
    objid = pobj->id;
  } else if (type != mjSENS_CLOCK) {
    throw mjCError(this, "invalid type in sensor '%s' (id = %d)", name.c_str(), id);
  }

  // get refid from reftype and refname
  if (reftype!=mjOBJ_UNKNOWN) {
    // check for missing object name
    if (refname.empty()) {
      throw mjCError(this,
                     "missing name of reference frame object in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }

    // find name
    mjCBase* pref = model->FindObject(reftype, refname);
    if (!pref) {
      throw mjCError(this,
                     "unrecognized name of reference frame object in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }

    // must be attached to object with spatial frame
    if (reftype!=mjOBJ_BODY && reftype!=mjOBJ_XBODY &&
        reftype!=mjOBJ_GEOM && reftype!=mjOBJ_SITE && reftype!=mjOBJ_CAMERA) {
      throw mjCError(this,
                     "reference frame object must be (x)body, geom, site or camera:"
                     " sensor '%s' (id = %d)", name.c_str(), id);
    }

    // get sensorized object id
    refid = pref->id;
  }

  // process according to sensor type
  switch (type) {
  case mjSENS_TOUCH:
  case mjSENS_ACCELEROMETER:
  case mjSENS_VELOCIMETER:
  case mjSENS_GYRO:
  case mjSENS_FORCE:
  case mjSENS_TORQUE:
  case mjSENS_MAGNETOMETER:
  case mjSENS_RANGEFINDER:
    // must be attached to site
    if (objtype!=mjOBJ_SITE) {
      throw mjCError(this,
                     "sensor must be attached to site: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set dim and datatype
    if (type==mjSENS_TOUCH || type==mjSENS_RANGEFINDER) {
      dim = 1;
      datatype = mjDATATYPE_POSITIVE;
    } else {
      dim = 3;
      datatype = mjDATATYPE_REAL;
    }

    // set stage
    if (type==mjSENS_MAGNETOMETER || type==mjSENS_RANGEFINDER) {
      needstage = mjSTAGE_POS;
    } else if (type==mjSENS_GYRO || type==mjSENS_VELOCIMETER) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_ACC;
    }
    break;

  case mjSENS_JOINTPOS:
  case mjSENS_JOINTVEL:
    // must be attached to joint
    if (objtype!=mjOBJ_JOINT) {
      throw mjCError(this,
                     "sensor must be attached to joint: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure joint is slide or hinge
    if (((mjCJoint*)pobj)->type!=mjJNT_SLIDE && ((mjCJoint*)pobj)->type!=mjJNT_HINGE) {
      throw mjCError(this,
                     "joint must be slide or hinge in sensor '%s' (id = %d)", name.c_str(), id);
    }

    //set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_JOINTPOS) {
      needstage = mjSTAGE_POS;
    } else {
      needstage = mjSTAGE_VEL;
    }
    break;

  case mjSENS_TENDONPOS:
  case mjSENS_TENDONVEL:
    // must be attached to tendon
    if (objtype!=mjOBJ_TENDON) {
      throw mjCError(this,
                     "sensor must be attached to tendon: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_TENDONPOS) {
      needstage = mjSTAGE_POS;
    } else {
      needstage = mjSTAGE_VEL;
    }
    break;

  case mjSENS_ACTUATORPOS:
  case mjSENS_ACTUATORVEL:
  case mjSENS_ACTUATORFRC:
    // must be attached to actuator
    if (objtype!=mjOBJ_ACTUATOR) {
      throw mjCError(this,
                     "sensor must be attached to actuator: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_ACTUATORPOS) {
      needstage = mjSTAGE_POS;
    } else if (type==mjSENS_ACTUATORVEL) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_ACC;
    }
    break;

  case mjSENS_BALLQUAT:
  case mjSENS_BALLANGVEL:
    // must be attached to joint
    if (objtype!=mjOBJ_JOINT) {
      throw mjCError(this,
                     "sensor must be attached to joint: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure joint is ball
    if (((mjCJoint*)pobj)->type!=mjJNT_BALL) {
      throw mjCError(this,
                     "joint must be ball in sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    if (type==mjSENS_BALLQUAT) {
      dim = 4;
      datatype = mjDATATYPE_QUATERNION;
      needstage = mjSTAGE_POS;
    } else {
      dim = 3;
      datatype = mjDATATYPE_REAL;
      needstage = mjSTAGE_VEL;
    }
    break;

  case mjSENS_JOINTLIMITPOS:
  case mjSENS_JOINTLIMITVEL:
  case mjSENS_JOINTLIMITFRC:
    // must be attached to joint
    if (objtype!=mjOBJ_JOINT) {
      throw mjCError(this,
                     "sensor must be attached to joint: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure joint has limit
    if (!((mjCJoint*)pobj)->limited) {
      throw mjCError(this, "joint must be limited in sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_JOINTLIMITPOS) {
      needstage = mjSTAGE_POS;
    } else if (type==mjSENS_JOINTLIMITVEL) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_ACC;
    }
    break;

  case mjSENS_TENDONLIMITPOS:
  case mjSENS_TENDONLIMITVEL:
  case mjSENS_TENDONLIMITFRC:
    // must be attached to tendon
    if (objtype!=mjOBJ_TENDON) {
      throw mjCError(this,
                     "sensor must be attached to tendon: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure tendon has limit
    if (!((mjCTendon*)pobj)->limited) {
      throw mjCError(this, "tendon must be limited in sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_TENDONLIMITPOS) {
      needstage = mjSTAGE_POS;
    } else if (type==mjSENS_TENDONLIMITVEL) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_ACC;
    }
    break;

  case mjSENS_FRAMEPOS:
  case mjSENS_FRAMEQUAT:
  case mjSENS_FRAMEXAXIS:
  case mjSENS_FRAMEYAXIS:
  case mjSENS_FRAMEZAXIS:
  case mjSENS_FRAMELINVEL:
  case mjSENS_FRAMEANGVEL:
  case mjSENS_FRAMELINACC:
  case mjSENS_FRAMEANGACC:
    // must be attached to object with spatial frame
    if (objtype!=mjOBJ_BODY && objtype!=mjOBJ_XBODY &&
        objtype!=mjOBJ_GEOM && objtype!=mjOBJ_SITE && objtype!=mjOBJ_CAMERA) {
      throw mjCError(this,
                     "sensor must be attached to (x)body, geom, site or camera:"
                     " sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set dim
    if (type==mjSENS_FRAMEQUAT) {
      dim = 4;
    } else {
      dim = 3;
    }

    // set datatype
    if (type==mjSENS_FRAMEQUAT) {
      datatype = mjDATATYPE_QUATERNION;
    } else if (type==mjSENS_FRAMEXAXIS || type==mjSENS_FRAMEYAXIS || type==mjSENS_FRAMEZAXIS) {
      datatype = mjDATATYPE_AXIS;
    } else {
      datatype = mjDATATYPE_REAL;
    }

    // set needstage
    if (type==mjSENS_FRAMELINACC || type==mjSENS_FRAMEANGACC) {
      needstage = mjSTAGE_ACC;
    } else if (type==mjSENS_FRAMELINVEL || type==mjSENS_FRAMEANGVEL) {
      needstage = mjSTAGE_VEL;
    } else {
      needstage = mjSTAGE_POS;
    }
    break;

  case mjSENS_SUBTREECOM:
  case mjSENS_SUBTREELINVEL:
  case mjSENS_SUBTREEANGMOM:
    // must be attached to body
    if (objtype!=mjOBJ_BODY) {
      throw mjCError(this,
                     "sensor must be attached to body: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // set
    dim = 3;
    datatype = mjDATATYPE_REAL;
    if (type==mjSENS_SUBTREECOM) {
      needstage = mjSTAGE_POS;
    } else {
      needstage = mjSTAGE_VEL;
    }
    break;

  case mjSENS_CLOCK:
    dim = 1;
    needstage = mjSTAGE_POS;
    datatype = mjDATATYPE_REAL;
    break;

  case mjSENS_USER:
    // check for negative dim
    if (dim<0) {
      throw mjCError(this, "sensor dim must be positive: sensor '%s' (id = %d)", name.c_str(), id);
    }

    // make sure dim is consistent with datatype
    if (datatype==mjDATATYPE_AXIS && dim!=3) {
      throw mjCError(this,
                     "datatype AXIS requires dim=3 in sensor '%s' (id = %d)", name.c_str(), id);
    }
    if (datatype==mjDATATYPE_QUATERNION && dim!=4) {
      throw mjCError(this,
                     "datatype QUATERNION requires dim=4 in sensor '%s' (id = %d)",
                     name.c_str(), id);
    }
    break;

  default:
    throw mjCError(this, "invalid type in sensor '%s' (id = %d)", name.c_str(), id);
  }

  // check cutoff for incompatible data types
  if (cutoff>0 && (datatype==mjDATATYPE_AXIS || datatype==mjDATATYPE_QUATERNION)) {
    throw mjCError(this,
                   "cutoff applied to axis or quaternion datatype in sensor '%s' (id = %d)",
                   name.c_str(), id);
  }
}



//------------------ class mjCNumeric implementation -----------------------------------------------

// constructor
mjCNumeric::mjCNumeric(mjCModel* _model) {
  // set model pointer
  model = _model;

  // clear variables
  data.clear();
  size = 0;
}



// destructor
mjCNumeric::~mjCNumeric() {
  data.clear();
}



// compiler
void mjCNumeric::Compile(void) {
  // check for size conflict
  if (size && !data.empty() && size<(int)data.size()) {
    throw mjCError(this,
                   "numeric '%s' (id = %d): specified size smaller than initialization array",
                   name.c_str(), id);
  }

  // set size if left unspecified
  if (!size) {
    size = (int)data.size();
  }

  // size cannot be zero
  if (!size) {
    throw mjCError(this, "numeric '%s' (id = %d): size cannot be zero", name.c_str(), id);
  }
}



//------------------ class mjCText implementation --------------------------------------------------

// constructor
mjCText::mjCText(mjCModel* _model) {
  // set model pointer
  model = _model;

  // clear variables
  data.clear();
}



// destructor
mjCText::~mjCText() {
  data.clear();
}



// compiler
void mjCText::Compile(void) {
  // size cannot be zero
  if (data.empty()) {
    throw mjCError(this, "text '%s' (id = %d): size cannot be zero", name.c_str(), id);
  }
}



//------------------ class mjCTuple implementation -------------------------------------------------

// constructor
mjCTuple::mjCTuple(mjCModel* _model) {
  // set model pointer
  model = _model;

  // clear variables
  objtype.clear();
  objname.clear();
  objprm.clear();
  objid.clear();
}



// destructor
mjCTuple::~mjCTuple() {
  objtype.clear();
  objname.clear();
  objprm.clear();
  objid.clear();
}



// compiler
void mjCTuple::Compile(void) {
  // check for empty tuple
  if (objtype.empty()) {
    throw mjCError(this, "tuple '%s' (id = %d) is empty", name.c_str(), id);
  }

  // check for size conflict
  if (objtype.size()!=objname.size() || objtype.size()!=objprm.size()) {
    throw mjCError(this,
                   "tuple '%s' (id = %d) has object arrays with different sizes", name.c_str(), id);
  }

  // resize objid to correct size
  objid.resize(objtype.size());

  // find objects, fill in ids
  for (int i=0; i<objtype.size(); i++) {
    // find object by type and name
    mjCBase* res = model->FindObject(objtype[i], objname[i]);
    if (!res) {
      throw mjCError(this, "unrecognized object '%s' in tuple %d", objname[i].c_str(), id);
    }

    // assign id
    objid[i] = res->id;
  }
}



//------------------ class mjCKey implementation ---------------------------------------------------

// constructor
mjCKey::mjCKey(mjCModel* _model) {
  // set model pointer
  model = _model;

  // clear variables
  time = 0;
  qpos.clear();
  qvel.clear();
  act.clear();
  mpos.clear();
  mquat.clear();
  ctrl.clear();
}



// destructor
mjCKey::~mjCKey() {
  qpos.clear();
  qvel.clear();
  act.clear();
  mpos.clear();
  mquat.clear();
  ctrl.clear();
}



// compiler
void mjCKey::Compile(const mjModel* m) {
  int i;

  // qpos: allocate or check size
  if (qpos.empty()) {
    qpos.resize(m->nq);
    for (i=0; i<m->nq; i++) {
      qpos[i] = (double)m->qpos0[i];
    }
  } else if (qpos.size()!=m->nq) {
    throw mjCError(this, "key %d: invalid qpos size, expected length %d", nullptr, id, m->nq);
  }

  // qvel: allocate or check size
  if (qvel.empty()) {
    qvel.resize(m->nv);
    for (i=0; i<m->nv; i++) {
      qvel[i] = 0;
    }
  } else if (qvel.size()!=m->nv) {
    throw mjCError(this, "key %d: invalid qvel size, expected length %d", nullptr, id, m->nv);
  }

  // act: allocate or check size
  if (act.empty()) {
    act.resize(m->na);
    for (i=0; i<m->na; i++) {
      act[i] = 0;
    }
  } else if (act.size()!=m->na) {
    throw mjCError(this, "key %d: invalid act size, expected length %d", nullptr, id, m->na);
  }

  // mpos: allocate or check size
  if (mpos.empty()) {
    mpos.resize(3*m->nmocap);
    if (m->nmocap) {
      for (i=0; i<m->nbody; i++) {
        if (m->body_mocapid[i]>=0) {
          int mocapid = m->body_mocapid[i];
          mpos[3*mocapid]   = m->body_pos[3*i];
          mpos[3*mocapid+1] = m->body_pos[3*i+1];
          mpos[3*mocapid+2] = m->body_pos[3*i+2];
        }
      }
    }
  } else if (mpos.size()!=3*m->nmocap) {
    throw mjCError(this, "key %d: invalid mpos size, expected length %d", nullptr, id, 3*m->nmocap);
  }

  // mquat: allocate or check size
  if (mquat.empty()) {
    mquat.resize(4*m->nmocap);
    if (m->nmocap) {
      for (i=0; i<m->nbody; i++) {
        if (m->body_mocapid[i]>=0) {
          int mocapid = m->body_mocapid[i];
          mquat[4*mocapid]   = m->body_quat[4*i];
          mquat[4*mocapid+1] = m->body_quat[4*i+1];
          mquat[4*mocapid+2] = m->body_quat[4*i+2];
          mquat[4*mocapid+3] = m->body_quat[4*i+3];
        }
      }
    }
  } else if (mquat.size()!=4*m->nmocap) {
    throw mjCError(this, "key %d: invalid mquat size, expected length %d", nullptr, id, 4*m->nmocap);
  }

  // ctrl: allocate or check size
  if (ctrl.empty()) {
    ctrl.resize(m->nu);
    for (i=0; i<m->nu; i++) {
      ctrl[i] = 0;
    }
  } else if (ctrl.size()!=m->nu) {
    throw mjCError(this, "key %d: invalid ctrl size, expected length %d", nullptr, id, m->nu);
  }

}
