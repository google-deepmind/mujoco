// Copyright 2024 DeepMind Technologies Limited
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

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iterator>
#include <map>
#include <new>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_cache.h"
#include "user/user_util.h"

namespace {

using mujoco::user::StringToVector;

}  // namespace

// global cache size in bytes (default 500MB)
static constexpr std::size_t kGlobalCacheSize = 500 * (1 << 20);


// create model
mjSpec* mj_makeSpec() {
  mjCModel* modelC = new mjCModel;
  return &modelC->spec;
}



// copy model
mjSpec* mj_copySpec(const mjSpec* s) {
  mjCModel* modelC = nullptr;
  try {
    modelC = new mjCModel(*static_cast<mjCModel*>(s->element));
  } catch (mjCError& e) {
    static_cast<mjCModel*>(s->element)->SetError(e);
    return nullptr;
  }
  return &modelC->spec;
}



// compile model
mjModel* mj_compile(mjSpec* s, const mjVFS* vfs) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  return modelC->Compile(vfs);
}



// recompile spec to model, preserving the state, return 0 on success
[[nodiscard]] int mj_recompile(mjSpec* s, const mjVFS* vfs, mjModel* m, mjData* d) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  std::string state_name = "state";
  mjtNum time = 0;
  try {
    if (d) {
      time = d->time;
      modelC->SaveState(state_name, d->qpos, d->qvel, d->act, d->ctrl, d->mocap_pos, d->mocap_quat);
    }
    if (!modelC->Compile(vfs, &m)) {
      if (d) {
        mj_deleteData(d);
      }
      return -1;
    };
    if (d) {
      modelC->MakeData(m, &d);
      modelC->RestoreState(state_name, m->qpos0, m->body_pos, m->body_quat, d->qpos, d->qvel,
                           d->act, d->ctrl, d->mocap_pos, d->mocap_quat);
      d->time = time;
    }
  } catch (mjCError& e) {
    modelC->SetError(e);
    return -1;
  }
  return 0;
}


// set frame for all elements of a body
static void SetFrame(mjsBody* body, mjtObj objtype, mjsFrame* frame) {
  mjsElement* el = mjs_firstChild(body, objtype, 0);
  while (el) {
    if (frame->element != el && mjs_getFrame(el) == nullptr) {
      mjs_setFrame(el, frame);
    }
    el = mjs_nextChild(body, el, 0);
  }
}



// attach body to a frame of the parent
static mjsElement* attachBody(mjCFrame* parent, const mjCBody* child,
                              const char* prefix, const char* suffix) {
  mjCBody* mutable_child = const_cast<mjCBody*>(child);
  mutable_child->prefix = prefix;
  mutable_child->suffix = suffix;
  try {
    *parent += *mutable_child;
  } catch (mjCError& e) {
    parent->model->SetError(e);
    return nullptr;
  }
  mjsBody* attached_body = parent->last_attached;
  parent->last_attached = nullptr;
  return attached_body->element;
}



// attach frame to a parent body
static mjsElement* attachFrame(mjCBody* parent, const mjCFrame* child,
                               const char* prefix, const char* suffix) {
  mjCFrame* mutable_child = const_cast<mjCFrame*>(child);
  mutable_child->prefix = prefix;
  mutable_child->suffix = suffix;
  try {
    *parent += *mutable_child;
  } catch (mjCError& e) {
    parent->model->SetError(e);
    return nullptr;
  }
  mjsFrame* attached_frame = parent->last_attached;
  parent->last_attached = nullptr;
  return attached_frame->element;
}



// attach child body to a parent site
static mjsElement* attachToSite(mjCSite* parent, const mjCBody* child,
                                const char* prefix, const char* suffix) {
  mjSpec* spec = mjs_getSpec(parent->spec.element);
  mjCBody* body = parent->Body();
  mjCFrame* frame = body->AddFrame(parent->frame);
  frame->SetParent(body);
  frame->spec.pos[0] = parent->spec.pos[0];
  frame->spec.pos[1] = parent->spec.pos[1];
  frame->spec.pos[2] = parent->spec.pos[2];
  frame->spec.quat[0] = parent->spec.quat[0];
  frame->spec.quat[1] = parent->spec.quat[1];
  frame->spec.quat[2] = parent->spec.quat[2];
  frame->spec.quat[3] = parent->spec.quat[3];
  mjs_resolveOrientation(frame->spec.quat, spec->compiler.degree,
                         spec->compiler.eulerseq, &parent->spec.alt);
  return attachBody(frame, child, prefix, suffix);
}



// attach child frame to a parent site
static mjsElement* attachFrameToSite(mjCSite* parent, const mjCFrame* child,
                                     const char* prefix, const char* suffix) {
  mjSpec* spec = mjs_getSpec(parent->spec.element);
  mjCBody* body = parent->Body();
  mjCFrame* frame = body->AddFrame(parent->frame);
  frame->SetParent(body);
  frame->spec.pos[0] = parent->spec.pos[0];
  frame->spec.pos[1] = parent->spec.pos[1];
  frame->spec.pos[2] = parent->spec.pos[2];
  frame->spec.quat[0] = parent->spec.quat[0];
  frame->spec.quat[1] = parent->spec.quat[1];
  frame->spec.quat[2] = parent->spec.quat[2];
  frame->spec.quat[3] = parent->spec.quat[3];
  mjs_resolveOrientation(frame->spec.quat, spec->compiler.degree,
                         spec->compiler.eulerseq, &parent->spec.alt);

  mjsElement* attached_frame = attachFrame(body, child, prefix, suffix);
  mjs_setFrame(attached_frame, &frame->spec);
  return attached_frame;
}


mjsElement* mjs_attach(mjsElement* parent, const mjsElement* child,
                       const char* prefix, const char* suffix) {
  if (!parent) {
    mju_error("parent element is null");
    return nullptr;
  }
  if (!child) {
    mju_error("child element is null");
    return nullptr;
  }
  mjCModel* model = static_cast<mjCModel*>(mjs_getSpec(parent)->element);
  if (child->elemtype == mjOBJ_MODEL) {
    mjCModel* child_model = static_cast<mjCModel*>((mjsElement*)child);
    mjsBody* worldbody = mjs_findBody(&child_model->spec, "world");
    if (!worldbody) {
      model->SetError(mjCError(0, "Child does not have a world body."));
      return nullptr;
    }
    mjsFrame* worldframe = mjs_addFrame(worldbody, nullptr);
    SetFrame(worldbody, mjOBJ_BODY, worldframe);
    SetFrame(worldbody, mjOBJ_SITE, worldframe);
    SetFrame(worldbody, mjOBJ_FRAME, worldframe);
    SetFrame(worldbody, mjOBJ_JOINT, worldframe);
    SetFrame(worldbody, mjOBJ_GEOM, worldframe);
    SetFrame(worldbody, mjOBJ_LIGHT, worldframe);
    SetFrame(worldbody, mjOBJ_CAMERA, worldframe);
    child = worldframe->element;
  }
  switch (parent->elemtype) {
    case mjOBJ_FRAME:
      if (child->elemtype == mjOBJ_BODY) {
        return attachBody(static_cast<mjCFrame*>(parent),
                          static_cast<const mjCBody*>(child), prefix, suffix);
      } else if (child->elemtype == mjOBJ_FRAME) {
        mjsBody* parent_body = mjs_getParent(parent);
        if (!parent_body) {
          model->SetError(mjCError(0, "Frame does not have a parent body."));
          return nullptr;
        }
        mjCFrame* frame = static_cast<mjCFrame*>(parent);
        mjsElement* attached_frame =
            attachFrame(static_cast<mjCBody*>(parent_body->element),
                        static_cast<const mjCFrame*>(child), prefix, suffix);
        if (mjs_setFrame(attached_frame, &frame->spec)) {
          return nullptr;
        }
        return attached_frame;
      } else {
        model->SetError(mjCError(0, "child element is not a body or frame"));
        return nullptr;
      }
    case mjOBJ_BODY:
      if (child->elemtype == mjOBJ_FRAME) {
        return attachFrame(static_cast<mjCBody*>(parent),
                           static_cast<const mjCFrame*>(child), prefix, suffix);
      } else {
        model->SetError(mjCError(0, "child element is not a frame"));
        return nullptr;
      }
    case mjOBJ_SITE:
      if (child->elemtype == mjOBJ_BODY) {
        return attachToSite(static_cast<mjCSite*>(parent),
                            static_cast<const mjCBody*>(child), prefix, suffix);
      } else if (child->elemtype == mjOBJ_FRAME) {
        return attachFrameToSite(static_cast<mjCSite*>(parent),
                                 static_cast<const mjCFrame*>(child), prefix, suffix);
      } else {
        model->SetError(mjCError(0, "child element is not a body or frame"));
        return nullptr;
      }
    default:
      model->SetError(mjCError(0, "parent element is not a frame, body or site"));
      return nullptr;
  }
  return nullptr;
}



// get error message from model
const char* mjs_getError(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  return modelC->GetError().message;
}



// detach body from mjSpec, return 0 on success
int mjs_detachBody(mjSpec* s, mjsBody* b) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  mjCBody* body = static_cast<mjCBody*>(b->element);
  try {
    *model -= *body;
  } catch (mjCError& e) {
    model->SetError(e);
    return -1;
  }
  model->Detach(body);
  return 0;
}

// detach default from mjSpec, return 0 on success
int mjs_detachDefault(mjSpec* s, mjsDefault* def) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  if (!def) {
    modelC->SetError(mjCError(0, "Cannot detach, default is null"));
    return -1;
  }
  mjCDef* defC = static_cast<mjCDef*>(def->element);
  try {
    *modelC -= *defC;
  } catch (mjCError& e) {
    modelC->SetError(e);
    return -1;
  }
  return 0;
}

// check if model has warnings
int mjs_isWarning(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  return modelC->GetError().warning;
}



// delete model
void mj_deleteSpec(mjSpec* s) {
  if (s) {
    mjCModel* model = static_cast<mjCModel*>(s->element);
    delete model;
  }
}



// add spec (model asset) to spec
void mjs_addSpec(mjSpec* s, mjSpec* child) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  model->AppendSpec(child);
}



// activate plugin
int mjs_activatePlugin(mjSpec* s, const char* name) {
  int plugin_slot = -1;
  const mjpPlugin* plugin = mjp_getPlugin(name, &plugin_slot);
  if (!plugin) {
    return -1;
  }
  mjCModel* model = static_cast<mjCModel*>(s->element);
  model->ActivatePlugin(plugin, plugin_slot);
  return 0;
}



// set deep copy flag
int mjs_setDeepCopy(mjSpec* s, int deepcopy) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  model->SetDeepCopy(deepcopy);
  return 0;
}



// copy real-valued arrays from model to spec, returns 1 on success
int mj_copyBack(mjSpec* s, const mjModel* m) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  return model->CopyBack(m);
}



// delete object, return 0 on success
int mjs_delete(mjsElement* element) {
  mjCModel* model;
  if (element->elemtype == mjOBJ_DEFAULT)
    model = static_cast<mjCDef*>(element)->model;
  else
    model = static_cast<mjCBase*>(element)->model;
  try {
    // it will call the appropriate destructor since ~mjCBase is virtual
    model->DeleteElement(element);
    return 0;
  } catch (mjCError& e) {
    model->SetError(e);
    return -1;
  }
}



// add child body to body, return child spec
mjsBody* mjs_addBody(mjsBody* bodyspec, const mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element)->AddBody(def);
  return &body->spec;
}



// add site to body, return site spec
mjsSite* mjs_addSite(mjsBody* bodyspec, const mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCSite* site = body->AddSite(def);
  return &site->spec;
}



// add joint to body
mjsJoint* mjs_addJoint(mjsBody* bodyspec, const mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCJoint* joint = body->AddJoint(def);
  return &joint->spec;
}



// add free joint to body
mjsJoint* mjs_addFreeJoint(mjsBody* bodyspec) {
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCJoint* joint = body->AddFreeJoint();
  return &joint->spec;
}



// add geom to body
mjsGeom* mjs_addGeom(mjsBody* bodyspec, const mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCGeom* geom = body->AddGeom(def);
  return &geom->spec;
}



// add camera to body
mjsCamera* mjs_addCamera(mjsBody* bodyspec, const mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCCamera* camera = body->AddCamera(def);
  return &camera->spec;
}



// add light to body
mjsLight* mjs_addLight(mjsBody* bodyspec, const mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCLight* light = body->AddLight(def);
  return &light->spec;
}



// add flex to model
mjsFlex* mjs_addFlex(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCFlex* flex = modelC->AddFlex();
  return &flex->spec;
}



// add frame to body
mjsFrame* mjs_addFrame(mjsBody* bodyspec, mjsFrame* parentframe) {
  mjCFrame* parentframeC = 0;
  if (parentframe) {
    parentframeC = static_cast<mjCFrame*>(parentframe->element);
  }
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCFrame* frameC = body->AddFrame(parentframeC);
  frameC->SetParent(body);
  return &frameC->spec;
}



// add mesh to model
mjsMesh* mjs_addMesh(mjSpec* s, const mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCMesh* mesh = modelC->AddMesh(def);
  return &mesh->spec;
}



// add height field to model
mjsHField* mjs_addHField(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCHField* heightField = modelC->AddHField();
  return &heightField->spec;
}



// add skin to model
mjsSkin* mjs_addSkin(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCSkin* skin = modelC->AddSkin();
  return &skin->spec;
}



// add texture to model
mjsTexture* mjs_addTexture(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCTexture* texture = modelC->AddTexture();
  return &texture->spec;
}



// add material to model
mjsMaterial* mjs_addMaterial(mjSpec* s, const mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCMaterial* material = modelC->AddMaterial(def);
  return &material->spec;
}



// add pair to model
mjsPair* mjs_addPair(mjSpec* s, const mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCPair* pair = modelC->AddPair(def);
  return &pair->spec;
}



// add pair exclusion to model
mjsExclude* mjs_addExclude(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCBodyPair* bodypair = modelC->AddExclude();
  return &bodypair->spec;
}



// add equality to model
mjsEquality* mjs_addEquality(mjSpec* s, const mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCEquality* equality = modelC->AddEquality(def);
  return &equality->spec;
}



// add tendon to model
mjsTendon* mjs_addTendon(mjSpec* s, const mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCTendon* tendon = modelC->AddTendon(def);
  return &tendon->spec;
}



// wrap site using tendon
mjsWrap* mjs_wrapSite(mjsTendon* tendonspec, const char* name) {
  mjCTendon* tendon = static_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapSite(name);
  return &tendon->path.back()->spec;
}



// wrap geom using tendon
mjsWrap* mjs_wrapGeom(mjsTendon* tendonspec, const char* name, const char* sidesite) {
  mjCTendon* tendon = static_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapGeom(name, sidesite);
  return &tendon->path.back()->spec;
}



// wrap joint using tendon
mjsWrap* mjs_wrapJoint(mjsTendon* tendonspec, const char* name, double coef) {
  mjCTendon* tendon = static_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapJoint(name, coef);
  return &tendon->path.back()->spec;
}



// wrap pulley using tendon
mjsWrap* mjs_wrapPulley(mjsTendon* tendonspec, double divisor) {
  mjCTendon* tendon = static_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapPulley(divisor);
  return &tendon->path.back()->spec;
}



// add actuator to model
mjsActuator* mjs_addActuator(mjSpec* s, const mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCActuator* actuator = modelC->AddActuator(def);
  return &actuator->spec;
}



// add sensor to model
mjsSensor* mjs_addSensor(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCSensor* sensor = modelC->AddSensor();
  return &sensor->spec;
}



// add numeric to model
mjsNumeric* mjs_addNumeric(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCNumeric* numeric = modelC->AddNumeric();
  return &numeric->spec;
}



// add text to model
mjsText* mjs_addText(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCText* text = modelC->AddText();
  return &text->spec;
}



// add tuple to model
mjsTuple* mjs_addTuple(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCTuple* tuple = modelC->AddTuple();
  return &tuple->spec;
}



// add keyframe to model
mjsKey* mjs_addKey(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCKey* key = modelC->AddKey();
  return &key->spec;
}



// add plugin to model
mjsPlugin* mjs_addPlugin(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCPlugin* plugin = modelC->AddPlugin();
  plugin->spec.element = static_cast<mjsElement*>(plugin);
  return &plugin->spec;
}



// add default to model
mjsDefault* mjs_addDefault(mjSpec* s, const char* classname, const mjsDefault* parent) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* parentC = parent ? static_cast<mjCDef*>(parent->element) :
                             static_cast<mjCModel*>(s->element)->Default();
  mjCDef* def = modelC->AddDefault(classname, parentC);
  if (def) {
    return &def->spec;
  } else {
    return nullptr;
  }
}



// set actuator to motor
const char* mjs_setToMotor(mjsActuator* actuator) {
  // unit gain
  actuator->gainprm[0] = 1;

  // implied parameters
  actuator->dyntype = mjDYN_NONE;
  actuator->gaintype = mjGAIN_FIXED;
  actuator->biastype = mjBIAS_NONE;
  return "";
}



// set to position actuator
const char* mjs_setToPosition(mjsActuator* actuator, double kp, double kv[1],
                              double dampratio[1], double timeconst[1], double inheritrange) {
  actuator->gainprm[0] = kp;
  actuator->biasprm[1] = -kp;

  // set biasprm[2]; negative: regular damping, positive: dampratio
  if (dampratio && kv) {
    return "kv and dampratio cannot both be defined";
  }

  if (kv) {
    if (*kv < 0) return "kv cannot be negative";
    actuator->biasprm[2] = -(*kv);
  }
  if (dampratio) {
    if (*dampratio < 0) return "dampratio cannot be negative";
    actuator->biasprm[2] = *dampratio;
  }
  if (timeconst) {
    if (*timeconst < 0) return "timeconst cannot be negative";
    actuator->dynprm[0] = *timeconst;
    actuator->dyntype = *timeconst == 0 ? mjDYN_NONE : mjDYN_FILTEREXACT;
  }
  actuator->inheritrange = inheritrange;

  if (inheritrange > 0) {
    if (actuator->ctrlrange[0] || actuator->ctrlrange[1]) {
      return "ctrlrange and inheritrange cannot both be defined";
    }
  }

  actuator->gaintype = mjGAIN_FIXED;
  actuator->biastype = mjBIAS_AFFINE;
  return "";
}



// Set to integrated velocity actuator.
const char* mjs_setToIntVelocity(mjsActuator* actuator, double kp, double kv[1],
                                 double dampratio[1], double timeconst[1], double inheritrange) {
  mjs_setToPosition(actuator, kp, kv, dampratio, timeconst, inheritrange);
  actuator->dyntype = mjDYN_INTEGRATOR;
  actuator->actlimited = 1;

  if (inheritrange > 0) {
    if (actuator->actrange[0] || actuator->actrange[1]) {
      return "actrange and inheritrange cannot both be defined";
    }
  }
  return "";
}



// Set to velocity actuator.
const char* mjs_setToVelocity(mjsActuator* actuator, double kv) {
  mjuu_zerovec(actuator->biasprm, mjNBIAS);
  actuator->gainprm[0] = kv;
  actuator->biasprm[2] = -kv;
  actuator->dyntype = mjDYN_NONE;
  actuator->gaintype = mjGAIN_FIXED;
  actuator->biastype = mjBIAS_AFFINE;
  return "";
}



// Set to damper actuator.
const char* mjs_setToDamper(mjsActuator* actuator, double kv) {
  mjuu_zerovec(actuator->gainprm, mjNGAIN);
  actuator->gainprm[2] = -kv;
  actuator->ctrllimited = 1;
  actuator->dyntype = mjDYN_NONE;
  actuator->gaintype = mjGAIN_AFFINE;
  actuator->biastype = mjBIAS_NONE;

  if (kv < 0) {
    return "damping coefficient cannot be negative";
  }
  if (actuator->ctrlrange[0] < 0 || actuator->ctrlrange[1] < 0) {
    return "damper control range cannot be negative";
  }
  return "";
}



// Set to cylinder actuator.
const char* mjs_setToCylinder(mjsActuator* actuator, double timeconst, double bias,
                       double area, double diameter) {
  actuator->dynprm[0] = timeconst;
  actuator->biasprm[0] = bias;
  actuator->gainprm[0] = area;
  if (diameter >= 0) {
    actuator->gainprm[0] = mjPI / 4 * diameter*diameter;
  }
  actuator->dyntype = mjDYN_FILTER;
  actuator->gaintype = mjGAIN_FIXED;
  actuator->biastype = mjBIAS_AFFINE;
  return "";
}



// Set to muscle actuator.
const char* mjs_setToMuscle(mjsActuator* actuator, double timeconst[2], double tausmooth,
                            double range[2], double force, double scale, double lmin,
                            double lmax, double vmax, double fpmax, double fvmax) {
  // set muscle defaults if same as global defaults
  if (actuator->dynprm[0] == 1) actuator->dynprm[0] = 0.01;    // tau act
  if (actuator->dynprm[1] == 0) actuator->dynprm[1] = 0.04;    // tau deact
  if (actuator->gainprm[0] == 1) actuator->gainprm[0] = 0.75;  // range[0]
  if (actuator->gainprm[1] == 0) actuator->gainprm[1] = 1.05;  // range[1]
  if (actuator->gainprm[2] == 0) actuator->gainprm[2] = -1;    // force
  if (actuator->gainprm[3] == 0) actuator->gainprm[3] = 200;   // scale
  if (actuator->gainprm[4] == 0) actuator->gainprm[4] = 0.5;   // lmin
  if (actuator->gainprm[5] == 0) actuator->gainprm[5] = 1.6;   // lmax
  if (actuator->gainprm[6] == 0) actuator->gainprm[6] = 1.5;   // vmax
  if (actuator->gainprm[7] == 0) actuator->gainprm[7] = 1.3;   // fpmax
  if (actuator->gainprm[8] == 0) actuator->gainprm[8] = 1.2;   // fvmax

  if (tausmooth < 0)
    return "muscle tausmooth cannot be negative";

  actuator->dynprm[2] = tausmooth;
  if (timeconst[0] >= 0) actuator->dynprm[0] = timeconst[0];
  if (timeconst[1] >= 0) actuator->dynprm[1] = timeconst[1];
  if (range[0] >= 0) actuator->gainprm[0] = range[0];
  if (range[1] >= 0) actuator->gainprm[1] = range[1];
  if (force >= 0) actuator->gainprm[2] = force;
  if (scale >= 0) actuator->gainprm[3] = scale;
  if (lmin >= 0) actuator->gainprm[4] = lmin;
  if (lmax >= 0) actuator->gainprm[5] = lmax;
  if (vmax >= 0) actuator->gainprm[6] = vmax;
  if (fpmax >= 0) actuator->gainprm[7] = fpmax;
  if (fvmax >= 0) actuator->gainprm[8] = fvmax;

  // biasprm = gainprm
  for (int n=0; n < 9; n++) {
    actuator->biasprm[n] = actuator->gainprm[n];
  }

  actuator->dyntype = mjDYN_MUSCLE;
  actuator->gaintype = mjGAIN_MUSCLE;
  actuator->biastype = mjBIAS_MUSCLE;
  return "";
}



// Set to adhesion actuator.
const char* mjs_setToAdhesion(mjsActuator* actuator, double gain) {
  actuator->gainprm[0] = gain;
  actuator->ctrllimited = 1;
  actuator->gaintype = mjGAIN_FIXED;
  actuator->biastype = mjBIAS_NONE;

  if (gain < 0)
    return "adhesion gain cannot be negative";
  if (actuator->ctrlrange[0] < 0 || actuator->ctrlrange[1] < 0)
    return "adhesion control range cannot be negative";
  return "";
}



// get spec from body
mjSpec* mjs_getSpec(mjsElement* element) {
  return &(static_cast<mjCBase*>(element)->model->spec);
}



// find spec (model asset) by name
mjSpec* mjs_findSpec(mjSpec* s, const char* name) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  return model->FindSpec(name);
}



// get default
mjsDefault* mjs_getDefault(mjsElement* element) {
  mjCModel* model = static_cast<mjCBase*>(element)->model;
  std::string classname = static_cast<mjCBase*>(element)->classname;
  return &(model->def_map[classname]->spec);
}



// Find default with given name in model.
mjsDefault* mjs_findDefault(mjSpec* s, const char* classname) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* cdef = modelC->FindDefault(classname);
  if (!cdef) {
    return nullptr;
  }
  return &cdef->spec;
}



// get default[0] from model
mjsDefault* mjs_getSpecDefault(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = modelC->Default();
  if (!def) {
    return nullptr;
  }
  return &def->spec;
}



// find body in model by name
mjsBody* mjs_findBody(mjSpec* s, const char* name) {
  mjsElement* body = mjs_findElement(s, mjOBJ_BODY, name);
  return body ? &(static_cast<mjCBody*>(body)->spec) : nullptr;
}



// find element in spec by name
mjsElement* mjs_findElement(mjSpec* s, mjtObj type, const char* name) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  if (model->IsCompiled() && type != mjOBJ_FRAME) {
    return model->FindObject(type, std::string(name));  // fast lookup
  }
  switch (type) {
    case mjOBJ_BODY:
    case mjOBJ_SITE:
    case mjOBJ_GEOM:
    case mjOBJ_JOINT:
    case mjOBJ_CAMERA:
    case mjOBJ_LIGHT:
    case mjOBJ_FRAME:
      return model->FindTree(model->GetWorld(), type, std::string(name));  // recursive search
    default:
      return model->FindObject(type, std::string(name));  // always available
  }
}



// find child of a body by name
mjsBody* mjs_findChild(mjsBody* bodyspec, const char* name) {
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCBase* child = body->FindObject(mjOBJ_BODY, std::string(name));
  return child ? &(static_cast<mjCBody*>(child)->spec) : nullptr;
}



// get parent body
mjsBody* mjs_getParent(mjsElement* element) {
  switch (element->elemtype) {
    case mjOBJ_BODY:
      return &(static_cast<mjCBody*>(element)->GetParent()->spec);
    case mjOBJ_FRAME:
      return &(static_cast<mjCFrame*>(element)->GetParent()->spec);
    case mjOBJ_JOINT:
      return &(static_cast<mjCJoint*>(element)->GetParent()->spec);
    case mjOBJ_GEOM:
      return &(static_cast<mjCGeom*>(element)->GetParent()->spec);
    case mjOBJ_SITE:
      return &(static_cast<mjCSite*>(element)->GetParent()->spec);
    case mjOBJ_CAMERA:
      return &(static_cast<mjCCamera*>(element)->GetParent()->spec);
    case mjOBJ_LIGHT:
      return &(static_cast<mjCLight*>(element)->GetParent()->spec);
    default:
      return nullptr;
  }
}



// get parent frame
mjsFrame* mjs_getFrame(mjsElement* element) {
  mjCBase* base = static_cast<mjCBase*>(element);
  switch (element->elemtype) {
    case mjOBJ_BODY:
    case mjOBJ_FRAME:
    case mjOBJ_JOINT:
    case mjOBJ_GEOM:
    case mjOBJ_SITE:
    case mjOBJ_CAMERA:
    case mjOBJ_LIGHT:
      return base->frame ? &(base->frame->spec) : nullptr;
    default:
      return nullptr;
  }
}



// find frame by name
mjsFrame* mjs_findFrame(mjSpec* s, const char* name) {
  mjsElement* frame = mjs_findElement(s, mjOBJ_FRAME, name);
  return frame ? &(static_cast<mjCFrame*>(frame)->spec) : nullptr;
}



// set frame
int mjs_setFrame(mjsElement* dest, mjsFrame* frame) {
  if (!frame) {
    return -1;
  }
  mjCFrame* frameC = static_cast<mjCFrame*>(frame->element);
  mjCBase* baseC = static_cast<mjCBase*>(dest);
  try {
    baseC->SetFrame(frameC);
    return 0;
  } catch (mjCError& e) {
    baseC->model->SetError(e);
    return -1;
  }
}



// Resolve alternative orientations.
const char* mjs_resolveOrientation(double quat[4], mjtByte degree, const char* sequence,
                                   const mjsOrientation* orientation) {
  return ResolveOrientation(quat, degree, sequence, *orientation);
}



// Transform body into a frame.
mjsFrame* mjs_bodyToFrame(mjsBody** body) {
  mjCBody* bodyC = static_cast<mjCBody*>((*body)->element);
  mjCFrame* frameC = bodyC->ToFrame();
  bodyC->model->Detach(bodyC);
  *body = nullptr;
  return &frameC->spec;
}

void mjs_setUserValue(mjsElement* element, const char* key, const void* data) {
  mjs_setUserValueWithCleanup(element, key, data, nullptr);
}

// set user payload
void mjs_setUserValueWithCleanup(mjsElement* element, const char* key,
                                 const void* data,
                                 void (*cleanup)(const void*)) {
  mjCBase* baseC = static_cast<mjCBase*>(element);
  baseC->SetUserValue(key, data, cleanup);
}

// return user payload or NULL if none found
const void* mjs_getUserValue(mjsElement* element, const char* key) {
  mjCBase* baseC = static_cast<mjCBase*>(element);
  return baseC->GetUserValue(key);
}



// delete user payload
void mjs_deleteUserValue(mjsElement* element, const char* key) {
  mjCBase* baseC = static_cast<mjCBase*>(element);
  baseC->DeleteUserValue(key);
}



// get id
int mjs_getId(mjsElement* element) {
  if (!element) {
    return -1;
  }
  return static_cast<mjCBase*>(element)->id;
}



// set default
void mjs_setDefault(mjsElement* element, const mjsDefault* defspec) {
  mjCBase* baseC = static_cast<mjCBase*>(element);
  baseC->classname = static_cast<mjCDef*>(defspec->element)->name;
}



// return first child of selected type
mjsElement* mjs_firstChild(mjsBody* body, mjtObj type, int recurse) {
  mjCBody* bodyC = static_cast<mjCBody*>(body->element);
  try {
    return bodyC->NextChild(NULL, type, recurse);
  } catch (mjCError& e) {
    bodyC->model->SetError(e);
    return nullptr;
  }
}



// return body's next child; return NULL if child is last
mjsElement* mjs_nextChild(mjsBody* body, mjsElement* child, int recurse) {
  mjCBody* bodyC = static_cast<mjCBody*>(body->element);
  try {
    return bodyC->NextChild(child, child->elemtype, recurse);
  } catch(mjCError& e) {
    bodyC->model->SetError(e);
    return nullptr;
  }
}



// return spec's first element of selected type
mjsElement* mjs_firstElement(mjSpec* s, mjtObj type) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  return modelC->NextObject(NULL, type);
}



// return spec's next element; return NULL if element is last
mjsElement* mjs_nextElement(mjSpec* s, mjsElement* element) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  return modelC->NextObject(element);
}



// return body given mjsElement
mjsBody* mjs_asBody(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_BODY) {
    return &(static_cast<mjCBody*>(element)->spec);
  }
  return nullptr;
}



// return geom given mjsElement
mjsGeom* mjs_asGeom(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_GEOM) {
    return &(static_cast<mjCGeom*>(element)->spec);
  }
  return nullptr;
}



// return joint given mjsElement
mjsJoint* mjs_asJoint(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_JOINT) {
    return &(static_cast<mjCJoint*>(element)->spec);
  }
  return nullptr;
}



// Return site given mjsElement
mjsSite* mjs_asSite(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_SITE) {
    return &(static_cast<mjCSite*>(element)->spec);
  }
  return nullptr;
}



// return camera given mjsElement
mjsCamera* mjs_asCamera(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_CAMERA) {
    return &(static_cast<mjCCamera*>(element)->spec);
  }
  return nullptr;
}



// return light given mjsElement
mjsLight* mjs_asLight(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_LIGHT) {
    return &(static_cast<mjCLight*>(element)->spec);
  }
  return nullptr;
}



// return frame given mjsElement
mjsFrame* mjs_asFrame(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_FRAME) {
    return &(static_cast<mjCFrame*>(element)->spec);
  }
  return nullptr;
}



// return actuator given mjsElement
mjsActuator* mjs_asActuator(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_ACTUATOR) {
    return &(static_cast<mjCActuator*>(element)->spec);
  }
  return nullptr;
}



// return sensor given mjsElement
mjsSensor* mjs_asSensor(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_SENSOR) {
    return &(static_cast<mjCSensor*>(element)->spec);
  }
  return nullptr;
}



// return flex given mjsElement
mjsFlex* mjs_asFlex(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_FLEX) {
    return &(static_cast<mjCFlex*>(element)->spec);
  }
  return nullptr;
}



// return pair given mjsElement
mjsPair* mjs_asPair(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_PAIR) {
    return &(static_cast<mjCPair*>(element)->spec);
  }
  return nullptr;
}



// return equality given mjsElement
mjsEquality* mjs_asEquality(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_EQUALITY) {
    return &(static_cast<mjCEquality*>(element)->spec);
  }
  return nullptr;
}



// return exclude given mjsElement
mjsExclude* mjs_asExclude(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_EXCLUDE) {
    return &(static_cast<mjCBodyPair*>(element)->spec);
  }
  return nullptr;
}



// return tendon given mjsElement
mjsTendon* mjs_asTendon(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_TENDON) {
    return &(static_cast<mjCTendon*>(element)->spec);
  }
  return nullptr;
}



// return numeric given mjsElement
mjsNumeric* mjs_asNumeric(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_NUMERIC) {
    return &(static_cast<mjCNumeric*>(element)->spec);
  }
  return nullptr;
}



// return text given mjsElement
mjsText* mjs_asText(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_TEXT) {
    return &(static_cast<mjCText*>(element)->spec);
  }
  return nullptr;
}



// return tuple given mjsElement
mjsTuple* mjs_asTuple(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_TUPLE) {
    return &(static_cast<mjCTuple*>(element)->spec);
  }
  return nullptr;
}



// return key given mjsElement
mjsKey* mjs_asKey(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_KEY) {
    return &(static_cast<mjCKey*>(element)->spec);
  }
  return nullptr;
}



// return mesh given mjsElement
mjsMesh* mjs_asMesh(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_MESH) {
    return &(static_cast<mjCMesh*>(element)->spec);
  }
  return nullptr;
}



// return hfield given mjsElement
mjsHField* mjs_asHField(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_HFIELD) {
    return &(static_cast<mjCHField*>(element)->spec);
  }
  return nullptr;
}



// return skin given mjsElement
mjsSkin* mjs_asSkin(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_SKIN) {
    return &(static_cast<mjCSkin*>(element)->spec);
  }
  return nullptr;
}



// return texture given mjsElement
mjsTexture* mjs_asTexture(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_TEXTURE) {
    return &(static_cast<mjCTexture*>(element)->spec);
  }
  return nullptr;
}



// return material given mjsElement
mjsMaterial* mjs_asMaterial(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_MATERIAL) {
    return &(static_cast<mjCMaterial*>(element)->spec);
  }
  return nullptr;
}



// return plugin given mjsElement
mjsPlugin* mjs_asPlugin(mjsElement* element) {
  if (element && element->elemtype == mjOBJ_PLUGIN) {
    return &(static_cast<mjCPlugin*>(element)->spec);
  }
  return nullptr;
}



// copy buffer to destination buffer
void mjs_setBuffer(mjByteVec* dest, const void* array, int size) {
  const std::byte* buffer = static_cast<const std::byte*>(array);
  dest->clear();
  dest->reserve(size);
  std::copy_n(buffer, size, std::back_inserter(*dest));
}



// set string
void mjs_setString(mjString* dest, const char* text) {
  std::string* str = static_cast<std::string*>(dest);
  *str = std::string(text);
}



// Set specific entry in destination string vector.
mjtByte mjs_setInStringVec(mjStringVec* dest, int i, const char* text) {
  if (dest->size() <= i) {
    mju_error("Requested index in mjs_setInStringVec is out of bounds");
    return 0;
  }
  dest->at(i) = std::string(text);
  return 1;
}



// split text and copy into string array
void mjs_setStringVec(mjStringVec* dest, const char* text) {
  std::vector<std::string>* v = static_cast<std::vector<std::string>*>(dest);
  *v = StringToVector<std::string>(text);
}



// add text entry to destination string vector
void mjs_appendString(mjStringVec* dest, const char* text) {
  dest->push_back(std::string(text));
}


// copy int array to vector
void mjs_setInt(mjIntVec* dest, const int* array, int size) {
  dest->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*dest)[i] = array[i];
  }
}



// append int array to vector of arrays
void mjs_appendIntVec(mjIntVecVec* dest, const int* array, int size) {
  dest->push_back(std::vector<int>(array, array + size));
}



// copy float array to vector
void mjs_setFloat(mjFloatVec* dest, const float* array, int size) {
  dest->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*dest)[i] = array[i];
  }
}




// append float array to vector of arrays
void mjs_appendFloatVec(mjFloatVecVec* dest, const float* array, int size) {
  dest->push_back(std::vector<float>(array, array + size));
}



// copy double array to vector
void mjs_setDouble(mjDoubleVec* dest, const double* array, int size) {
  dest->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*dest)[i] = array[i];
  }
}



// get string
const char* mjs_getString(const mjString* source) {
  return source->c_str();
}



// get double array
const double* mjs_getDouble(const mjDoubleVec* source, int* size) {
  if (size) {
    *size = source->size();
  }
  return source->data();
}



// set plugin attributes
void mjs_setPluginAttributes(mjsPlugin* plugin, void* attributes) {
  mjCPlugin* pluginC = static_cast<mjCPlugin*>(plugin->element);
  std::map<std::string, std::string, std::less<> >* config_attribs =
    reinterpret_cast<std::map<std::string, std::string, std::less<> >*>(attributes);
  pluginC->config_attribs = std::move(*config_attribs);
}



// get plugin attributes
const void* mjs_getPluginAttributes(const mjsPlugin* plugin) {
  mjCPlugin* pluginC = static_cast<mjCPlugin*>(plugin->element);
  return &pluginC->config_attribs;
}



// -------------------------- GLOBAL ASSET CACHE -------------------------------

void mj_setCacheSize(mjCache cache, std::size_t size) {
  mjCCache* ccache = reinterpret_cast<mjCCache*>(cache);
  if (ccache) {
    ccache->SetMaxSize(size);
  }
}



mjCache mj_globalCache() {
  // mjCCache is not trivially destructible and so the global cache needs to
  // allocated on the heap
  if constexpr (kGlobalCacheSize != 0) {
    static mjCCache* cache = new(std::nothrow) mjCCache(kGlobalCacheSize);
    return (mjCache) cache;
  } else {
    return NULL;
  }
}
