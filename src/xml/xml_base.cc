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

#include "xml/xml_base.h"

#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "user/user_objects.h"
#include "xml/xml_util.h"
#include "tinyxml2.h"

namespace {

using std::string;
using tinyxml2::XMLElement;

}  // namespace


//--------------------------------- Base class, helper functions -----------------------------------

// base constructor
mjXBase::mjXBase() {
  spec = NULL;
}


// set model field
void mjXBase::SetModel(mjSpec* _model, const mjModel* m) {
  spec = _model;
}


// read alternative orientation specification
int mjXBase::ReadAlternative(XMLElement* elem, mjsOrientation& alt) {
  string text;
  int    numspec = (int)(elem->Attribute("quat") != 0);

  // a quaternion replaces an alternative which the element has from its default class
  if (numspec) { alt.type = mjORIENTATION_QUAT; }
  if (ReadAttr(elem, "axisangle", 4, alt.axisangle, text)) {
    numspec++;
    alt.type = mjORIENTATION_AXISANGLE;
  }
  if (ReadAttr(elem, "xyaxes", 6, alt.xyaxes, text)) {
    numspec++;
    alt.type = mjORIENTATION_XYAXES;
  }
  if (ReadAttr(elem, "zaxis", 3, alt.zaxis, text)) {
    numspec++;
    alt.type = mjORIENTATION_ZAXIS;
  }
  if (ReadAttr(elem, "euler", 3, alt.euler, text)) {
    numspec++;
    alt.type = mjORIENTATION_EULER;
  }
  if (numspec > 1) { throw mjXError(elem, "multiple orientation specifiers are not allowed"); }
  return numspec;
}


//--------------------------------- Actuator shortcuts ---------------------------------------------

// the documented defaults of a shortcut's parameters
void mjXShortcutDefaults(mjXShortcut* s, mjtActuator type) {
  *s          = mjXShortcut();
  s->diameter = -1;
  switch (type) {
    case mjACTUATOR_POSITION:
    case mjACTUATOR_INTVELOCITY:
    case mjACTUATOR_ORIENTATION:
    case mjACTUATOR_PID:
      s->kp = 1;
      break;
    case mjACTUATOR_VELOCITY:
      s->kv = 1;
      break;
    case mjACTUATOR_CYLINDER:
      s->timeconst[0] = 1;
      s->area         = 1;
      break;
    case mjACTUATOR_MUSCLE:
      s->timeconst[0] = 0.01;
      s->timeconst[1] = 0.04;
      s->range[0]     = 0.75;
      s->range[1]     = 1.05;
      s->force        = -1;
      s->scale        = 200;
      s->lmin         = 0.5;
      s->lmax         = 1.6;
      s->vmax         = 1.5;
      s->fpmax        = 1.3;
      s->fvmax        = 1.2;
      break;
    case mjACTUATOR_ADHESION:
      s->gain = 1;
      break;
    default:
      break;
  }
}


// whether a shortcut pre-reads its parameters from a class slot: one written with the same
// shortcut, or with general and the shortcut's gain type
static bool ShortcutInherits(const mjsActuator* slot, mjtActuator type) {
  if (slot->type == type) { return true; }
  if (slot->type != mjACTUATOR_GENERAL) { return false; }
  switch (type) {
    case mjACTUATOR_DAMPER:
      return slot->gaintype == mjGAIN_AFFINE;
    case mjACTUATOR_MUSCLE:
      return slot->gaintype == mjGAIN_MUSCLE;
    case mjACTUATOR_DCMOTOR:
      return slot->gaintype == mjGAIN_DCMOTOR;
    case mjACTUATOR_PID:
      return slot->gaintype == mjGAIN_PID;
    case mjACTUATOR_ORIENTATION:
      return slot->gaintype == mjGAIN_SO3;
    default:
      return slot->gaintype == mjGAIN_FIXED;
  }
}


// the class slot a shortcut pre-reads: the class's own, else the nearest ancestor class which the
// shortcut inherits from; null for the documented defaults
const mjsActuator* mjXShortcutSource(const mjsActuator* slot, const mjCDef* def, mjtActuator type) {
  if (ShortcutInherits(slot, type)) { return slot; }
  for (const mjCDef* d = def ? def->parent : nullptr; d; d = d->parent) {
    if (ShortcutInherits(d->spec.actuator, type)) { return d->spec.actuator; }
  }
  return nullptr;
}


void mjXShortcutFromActuator(mjXShortcut* s, const mjsActuator* a, mjtActuator type) {
  mjXShortcutDefaults(s, type);
  const double* g = a->gainprm;
  const double* b = a->biasprm;
  const double* d = a->dynprm;
  s->ctrlspec     = a->ctrlspec;

  // kv or dampratio: the sign of biasprm[2] (negative: kv)
  auto damping = [&]() {
    s->has_kv        = b[2] < 0;
    s->has_dampratio = b[2] > 0;
    s->kv            = s->has_kv ? -b[2] : 0;
    s->dampratio     = s->has_dampratio ? b[2] : 0;
  };

  switch (type) {
    case mjACTUATOR_POSITION:
    case mjACTUATOR_INTVELOCITY:
      s->kp = g[0];
      damping();
      s->has_timeconst = a->dyntype == mjDYN_FILTEREXACT;
      s->timeconst[0]  = s->has_timeconst ? d[0] : 0;
      s->inheritrange  = a->inheritrange;
      break;
    case mjACTUATOR_ORIENTATION:
      s->kp = g[0];
      damping();
      break;
    case mjACTUATOR_PID:
      s->kp = -b[1];
      damping();
      s->ki           = g[0];
      s->imax         = d[0];
      s->slewmax      = d[1];
      s->inheritrange = a->inheritrange;
      break;
    case mjACTUATOR_VELOCITY:
      s->kv = g[0];
      break;
    case mjACTUATOR_DAMPER:
      s->kv = -g[2];
      break;
    case mjACTUATOR_CYLINDER:
      s->timeconst[0] = d[0];
      s->area         = g[0];
      std::copy(b, b + 3, s->bias);
      break;
    case mjACTUATOR_MUSCLE:
      std::copy(d, d + 2, s->timeconst);
      s->tausmooth = d[2];
      std::copy(g, g + 2, s->range);
      s->force = g[2];
      s->scale = g[3];
      s->lmin  = g[4];
      s->lmax  = g[5];
      s->vmax  = g[6];
      s->fpmax = g[7];
      s->fvmax = g[8];
      break;
    case mjACTUATOR_ADHESION:
      s->gain = g[0];
      break;
    case mjACTUATOR_DCMOTOR: {
      s->motorconst[0] = g[1];
      s->resistance    = g[0];
      bool symmetric   = a->forcerange[1] > 0 && a->forcerange[0] == -a->forcerange[1];
      if (a->forcelimited == mjLIMITED_TRUE && symmetric) { s->saturation[0] = a->forcerange[1]; }
      s->saturation[2] = d[1];
      s->inductance[1] = d[0];
      std::copy(b, b + 3, s->cogging);
      double controller[6] = {g[4], g[5], g[6], d[7], d[8], g[7]};
      double thermal[6]    = {d[2], d[3], 0, g[2], g[3], d[4]};
      double lugre[5]      = {d[5], d[6], b[3], b[4], b[5]};
      std::copy(controller, controller + 6, s->controller);
      std::copy(thermal, thermal + 6, s->thermal);
      std::copy(lugre, lugre + 5, s->lugre);
      break;
    }
    default:
      break;
  }
}


// set an actuator's control model from a shortcut
const char* mjXSetToShortcut(mjsActuator* a, mjtActuator type, const mjXShortcut& s) {
  mjXShortcut c         = s;  // the setters take mutable arrays
  double*     kv        = s.has_kv ? &c.kv : nullptr;
  double*     dampratio = s.has_dampratio ? &c.dampratio : nullptr;
  double*     timeconst = s.has_timeconst ? c.timeconst : nullptr;
  switch (type) {
    case mjACTUATOR_MOTOR:
      return mjs_setToMotor(a);
    case mjACTUATOR_POSITION:
      return mjs_setToPosition(a, s.kp, kv, dampratio, timeconst, s.inheritrange);
    case mjACTUATOR_INTVELOCITY:
      return mjs_setToIntVelocity(a, s.kp, kv, dampratio, timeconst, s.inheritrange);
    case mjACTUATOR_ORIENTATION:
      return mjs_setToOrientation(a, s.kp, kv, dampratio, s.ctrlspec);
    case mjACTUATOR_PID:
      return mjs_setToPID(a,
                          s.kp,
                          kv,
                          dampratio,
                          &c.ki,
                          &c.imax,
                          &c.slewmax,
                          s.inheritrange,
                          s.ctrlspec);
    case mjACTUATOR_VELOCITY:
      return mjs_setToVelocity(a, s.kv);
    case mjACTUATOR_DAMPER:
      return mjs_setToDamper(a, s.kv);
    case mjACTUATOR_CYLINDER: {
      const char* err = mjs_setToCylinder(a, s.timeconst[0], s.bias[0], s.area, s.diameter);
      a->biasprm[1]   = s.bias[1];
      a->biasprm[2]   = s.bias[2];
      return err;
    }
    case mjACTUATOR_MUSCLE:
      return mjs_setToMuscle(a,
                             c.timeconst,
                             s.tausmooth,
                             c.range,
                             s.force,
                             s.scale,
                             s.lmin,
                             s.lmax,
                             s.vmax,
                             s.fpmax,
                             s.fvmax);
    case mjACTUATOR_ADHESION:
      return mjs_setToAdhesion(a, s.gain);
    case mjACTUATOR_DCMOTOR:
      return mjs_setToDCMotor(a,
                              c.motorconst,
                              s.resistance,
                              c.nominal,
                              c.saturation,
                              c.inductance,
                              c.cogging,
                              c.controller,
                              c.thermal,
                              c.lugre,
                              s.ctrlspec);
    default:
      return "not an actuator shortcut";
  }
}
