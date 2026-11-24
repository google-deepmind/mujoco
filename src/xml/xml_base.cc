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
    case mjACTUATOR_MILLARD2012: {
      // OpenSim Millard2012EquilibriumMuscle; a stiffness of 0 takes OpenSim's derived default
      const double active[4]   = {0.4441, 0.73, 1.8123, 0.8616};
      const double passive[5]  = {0, 0.7, 0.2, 0, 0.75};
      const double tendon[4]   = {0.049, 0, 2.0/3.0, 0.5};
      const double velocity[6] = {1.4, 0.25, 5, 0.15, 0.6, 0.9};
      s->timeconst[0]  = 0.01;
      s->timeconst[1]  = 0.04;
      s->minactivation = 0.01;
      s->force         = 1000;
      s->optlength     = 0.1;
      s->slacklength   = 0.2;
      s->vmax          = 10;
      s->fiberdamping  = 0.1;
      std::copy(active, active + 4, s->activecurve);
      std::copy(passive, passive + 5, s->passivecurve);
      std::copy(tendon, tendon + 4, s->tendoncurve);
      std::copy(velocity, velocity + 6, s->velocitycurve);
      break;
    }
    case mjACTUATOR_GEYER2010:
      // Geyer and Herr (2010); a parallel element strain of 0 takes the paper's, the width
      s->timeconst[0]     = 0.01;
      s->timeconst[1]     = 0.04;
      s->force            = 1000;
      s->optlength        = 0.1;
      s->slacklength      = 0.2;
      s->vmax             = 12;
      s->activecurve[0]   = 0.56;
      s->activecurve[1]   = 0.05;
      s->passivecurve[0]  = 1;
      s->tendoncurve[0]   = 0.04;
      s->velocitycurve[0] = 5;
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
    case mjACTUATOR_MILLARD2012:
      return slot->gaintype == mjGAIN_MILLARD2012;
    case mjACTUATOR_GEYER2010:
      return slot->gaintype == mjGAIN_GEYER2010;
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
    case mjACTUATOR_MILLARD2012:
      std::copy(d, d + 2, s->timeconst);
      s->minactivation = d[2];
      s->fiberdamping  = d[3];
      std::copy(d + 4, d + 10, s->velocitycurve);
      s->force       = g[0];
      s->optlength   = g[1];
      s->slacklength = g[2];
      s->vmax        = g[3];
      s->pennation   = g[4];
      std::copy(g + 5, g + 9, s->activecurve);
      std::copy(b, b + 5, s->passivecurve);
      std::copy(b + 5, b + 9, s->tendoncurve);
      s->rigidtendon = b[9] != 0;

      // derived stiffnesses read back as derived
      if (s->passivecurve[3] == 2 / (s->passivecurve[1] - s->passivecurve[0])) {
        s->passivecurve[3] = 0;
      }
      if (s->tendoncurve[1] == 1.375 / s->tendoncurve[0]) { s->tendoncurve[1] = 0; }
      break;
    case mjACTUATOR_GEYER2010:
      std::copy(d, d + 2, s->timeconst);
      s->tausmooth        = d[2];
      s->fiberdamping     = d[3];
      s->velocitycurve[0] = d[4];
      s->force            = g[0];
      s->optlength        = g[1];
      s->slacklength      = g[2];
      s->vmax             = g[3];
      std::copy(g + 4, g + 6, s->activecurve);
      std::copy(b, b + 2, s->passivecurve);
      s->tendoncurve[0] = b[2];
      s->rigidtendon    = b[3] != 0;

      // the derived parallel element strain reads back as derived
      if (s->passivecurve[1] == s->activecurve[0]) { s->passivecurve[1] = 0; }
      break;
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
    case mjACTUATOR_MILLARD2012:
      return mjs_setToMillard2012(a,
                                  c.timeconst,
                                  s.minactivation,
                                  s.force,
                                  s.optlength,
                                  s.slacklength,
                                  s.vmax,
                                  s.pennation,
                                  s.fiberdamping,
                                  s.rigidtendon,
                                  c.activecurve,
                                  c.passivecurve,
                                  c.tendoncurve,
                                  c.velocitycurve);
    case mjACTUATOR_GEYER2010:
      return mjs_setToGeyer2010(a,
                                c.timeconst,
                                s.tausmooth,
                                s.force,
                                s.optlength,
                                s.slacklength,
                                s.vmax,
                                s.fiberdamping,
                                s.rigidtendon,
                                c.activecurve,
                                c.passivecurve,
                                s.tendoncurve[0],
                                s.velocitycurve[0]);
    default:
      return "not an actuator shortcut";
  }
}
