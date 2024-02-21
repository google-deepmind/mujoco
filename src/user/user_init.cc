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

#include <cstring>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "cc/array_safety.h"
#include "user/user_api.h"
#include "user/user_util.h"



// default model attributes
void mjm_defaultModel(mjmModel& model) {
  memset(&model, 0, sizeof(mjmModel));

  // default statistics
  model.stat.meaninertia = mjNAN;
  model.stat.meanmass = mjNAN;
  model.stat.meansize = mjNAN;
  model.stat.extent = mjNAN;
  model.stat.center[0] = mjNAN;

  // compiler settings
  model.autolimits = true;
  model.boundmass = 0;
  model.boundinertia = 0;
  model.settotalmass = -1;
  model.balanceinertia = false;
  model.strippath = false;
  model.fitaabb = false;
  model.degree = true;
  model.euler[0] = 'x';
  model.euler[1] = 'y';
  model.euler[2] = 'z';
  model.discardvisual = false;
  model.convexhull = true;
  model.usethread = true;
  model.fusestatic = false;
  model.inertiafromgeom = mjINERTIAFROMGEOM_AUTO;
  model.inertiagrouprange[0] = 0;
  model.inertiagrouprange[1] = mjNGROUP-1;
  model.exactmeshinertia = false;
  mj_defaultLROpt(&model.LRopt);

  // engine data
  mj_defaultOption(&model.option);
  mj_defaultVisual(&model.visual);
  model.memory = -1;
  model.nemax = 0;
  model.njmax = -1;
  model.nconmax = -1;
  model.nstack = -1;
  model.nuserdata = 0;
  model.nuser_body = -1;
  model.nuser_jnt = -1;
  model.nuser_geom = -1;
  model.nuser_site = -1;
  model.nuser_cam = -1;
  model.nuser_tendon = -1;
  model.nuser_actuator = -1;
  model.nuser_sensor = -1;
}



// default body attributes
void mjm_defaultBody(mjmBody& body) {
  memset(&body, 0, sizeof(mjmBody));

  // body frame
  body.pos[0] = mjNAN;
  body.quat[0] = 1;
  body.alt.axisangle[0] = body.alt.xyaxes[0] = body.alt.zaxis[0] = body.alt.euler[0] = mjNAN;

  // inertial frame
  body.ipos[0] = mjNAN;
  body.iquat[0] = 1;
  body.ialt.axisangle[0] = body.ialt.xyaxes[0] = body.ialt.zaxis[0] = body.ialt.euler[0] = mjNAN;
  body.fullinertia[0] = mjNAN;
}



// default frame attributes
void mjm_defaultFrame(mjmFrame& frame) {
  memset(&frame, 0, sizeof(mjmFrame));
  mju_zero3(frame.pos);
  mjuu_setvec(frame.quat, 1, 0, 0, 0);
  frame.alt.axisangle[0] = frame.alt.xyaxes[0] = frame.alt.zaxis[0] = frame.alt.euler[0] = mjNAN;
}



// default joint attributes
void mjm_defaultJoint(mjmJoint& joint) {
  memset(&joint, 0, sizeof(mjmJoint));

  joint.type = mjJNT_HINGE;
  joint.axis[2] = 1;
  joint.limited = mjLIMITED_AUTO;
  joint.actfrclimited = mjLIMITED_AUTO;
  mj_defaultSolRefImp(joint.solref_limit, joint.solimp_limit);
  mj_defaultSolRefImp(joint.solref_friction, joint.solimp_friction);
  joint.urdfeffort = -1;
}



// default geom attributes
void mjm_defaultGeom(mjmGeom& geom) {
  memset(&geom, 0, sizeof(mjmGeom));

  // type
  geom.type = mjGEOM_SPHERE;

  // frame
  geom.quat[0] = 1;
  geom.fromto[0] = mjNAN;
  geom.alt.axisangle[0] = geom.alt.xyaxes[0] = geom.alt.zaxis[0] = geom.alt.euler[0] = mjNAN;

  // contact-related
  geom.contype = 1;
  geom.conaffinity = 1;
  geom.condim = 3;
  geom.friction[0] = 1;
  geom.friction[1] = 0.005;
  geom.friction[2] = 0.0001;
  geom.solmix = 1.0;
  mj_defaultSolRefImp(geom.solref, geom.solimp);

  // inertia-related
  geom.mass = mjNAN;
  geom.density = 1000;  // water density (1000 Kg / m^3)
  geom.typeinertia = mjINERTIA_VOLUME;

  // color
  geom.rgba[0] = geom.rgba[1] = geom.rgba[2] = 0.5f;
  geom.rgba[3] = 1.0f;

  // fluid forces
  geom.fluid_coefs[0] = 0.5;   // blunt drag
  geom.fluid_coefs[1] = 0.25;  // slender drag
  geom.fluid_coefs[2] = 1.5;   // angular drag
  geom.fluid_coefs[3] = 1.0;   // kutta lift
  geom.fluid_coefs[4] = 1.0;   // magnus lift

  // other
  geom.fitscale = 1;
}



// default site attributes
void mjm_defaultSite(mjmSite& site) {
  // set everything to 0 / false / NULL
  memset(&site, 0, sizeof(mjmSite));

  // type
  site.type = mjGEOM_SPHERE;

  // frame
  site.quat[0] = 1;
  site.size[0] = site.size[1] = site.size[2] = 0.005;
  site.fromto[0] = mjNAN;
  site.alt.axisangle[0] = site.alt.xyaxes[0] = site.alt.zaxis[0] = site.alt.euler[0] = mjNAN;

  // color
  site.rgba[0] = site.rgba[1] = site.rgba[2] = 0.5f;
  site.rgba[3] = 1.0f;
}



// default cam attributes
void mjm_defaultCamera(mjmCamera& cam) {
  memset(&cam, 0, sizeof(mjmCamera));

  // type
  cam.mode = mjCAMLIGHT_FIXED;

  // extrinsics
  cam.quat[0] = 1;
  cam.alt.axisangle[0] = cam.alt.xyaxes[0] = cam.alt.zaxis[0] = cam.alt.euler[0] = mjNAN;

  // intrinsics
  cam.fovy = 45;
  cam.ipd = 0.068;
  cam.resolution[0] = cam.resolution[1] = 1;
}



// default light attributes
void mjm_defaultLight(mjmLight& light) {
  memset(&light, 0, sizeof(mjmLight));

  // mode
  light.mode = mjCAMLIGHT_FIXED;

  // intrinsics
  light.castshadow = 1;
  light.active = 1;
  light.dir[2] = -1;
  light.attenuation[0] = 1;
  light.cutoff = 45;
  light.exponent = 10;
  light.diffuse[0] = light.diffuse[1] = light.diffuse[2] = 0.7;
  light.specular[0] = light.specular[1] = light.specular[2] = 0.3;
}



// default flex attributes
void mjm_defaultFlex(mjmFlex& flex) {
  memset(&flex, 0, sizeof(mjmFlex));

  // set contact defaults
  flex.contype = 1;
  flex.conaffinity = 1;
  flex.condim = 3;
  mjuu_setvec(flex.friction, 1, 0.005, 0.0001);
  flex.solmix = 1.0;
  mj_defaultSolRefImp(flex.solref, flex.solimp);

  // set other defaults
  flex.dim = 2;
  flex.radius = 0.005;
  flex.internal = true;
  flex.selfcollide = mjFLEXSELF_AUTO;
  flex.activelayers = 1;
  flex.rgba[0] = flex.rgba[1] = flex.rgba[2] = 0.5f;
  flex.rgba[3] = 1.0f;
}



// default mesh attributes
void mjm_defaultMesh(mjmMesh& mesh) {
  memset(&mesh, 0, sizeof(mjmMesh));
  mjuu_setvec(mesh.refpos, 0, 0, 0);
  mjuu_setvec(mesh.refquat, 1, 0, 0, 0);
  mjuu_setvec(mesh.scale, 1, 1, 1);
  mesh.smoothnormal = false;
}



// default height field attributes
void mjm_defaultHField(mjmHField& hfield) {
  memset(&hfield, 0, sizeof(mjmHField));
}



// default skin attributes
void mjm_defaultSkin(mjmSkin& skin) {
  memset(&skin, 0, sizeof(mjmSkin));
  skin.rgba[0] = skin.rgba[1] = skin.rgba[2] = 0.5f;
  skin.rgba[3] = 1.0f;
  skin.inflate = 0;
  skin.group = 0;
}



// default texture attributes
void mjm_defaultTexture(mjmTexture& texture) {
  memset(&texture, 0, sizeof(mjmTexture));
  texture.type = mjTEXTURE_CUBE;
  mjuu_setvec(texture.rgb1, 0.8, 0.8, 0.8);
  mjuu_setvec(texture.rgb2, 0.5, 0.5, 0.5);
  mjuu_setvec(texture.markrgb, 0, 0, 0);
  texture.random = 0.01;
  texture.gridsize[0] = texture.gridsize[1] = 1;
  mujoco::util::strcpy_arr(texture.gridlayout, "............");
}



// default material attributes
void mjm_defaultMaterial(mjmMaterial& material) {
  memset(&material, 0, sizeof(mjmMaterial));
  material.texuniform = false;
  material.texrepeat[0] = material.texrepeat[1] = 1;
  material.emission = 0;
  material.specular = 0.5;
  material.shininess = 0.5;
  material.reflectance = 0;
  material.rgba[0] = material.rgba[1] = material.rgba[2] = material.rgba[3] = 1;
}



// default pair attributes
void mjm_defaultPair(mjmPair& pair) {
  memset(&pair, 0, sizeof(mjmPair));
  pair.condim = 3;
  mj_defaultSolRefImp(pair.solref, pair.solimp);
  pair.friction[0] = 1;
  pair.friction[1] = 1;
  pair.friction[2] = 0.005;
  pair.friction[3] = 0.0001;
  pair.friction[4] = 0.0001;
}



// default equality attributes
void mjm_defaultEquality(mjmEquality& equality) {
  memset(&equality, 0, sizeof(mjmEquality));
  equality.type = mjEQ_CONNECT;
  equality.active = 1;
  mj_defaultSolRefImp(equality.solref, equality.solimp);
  equality.data[1] = 1;
  equality.data[10] = 1;  // torque:force ratio
}



// default tendon attributes
void mjm_defaultTendon(mjmTendon& tendon) {
  memset(&tendon, 0, sizeof(mjmTendon));
  tendon.limited = mjLIMITED_AUTO;
  tendon.springlength[0] = tendon.springlength[1] = -1;
  mj_defaultSolRefImp(tendon.solref_limit, tendon.solimp_limit);
  mj_defaultSolRefImp(tendon.solref_friction, tendon.solimp_friction);
  tendon.width = 0.003;
  tendon.rgba[0] = tendon.rgba[1] = tendon.rgba[2] = 0.5f;
  tendon.rgba[3] = 1.0f;
}



// default actuator attributes
void mjm_defaultActuator(mjmActuator& actuator) {
  memset(&actuator, 0, sizeof(mjmActuator));

  // gain, bias
  actuator.gaintype = mjGAIN_FIXED;
  actuator.gainprm[0] = 1;
  actuator.biastype = mjBIAS_NONE;

  // activation state
  actuator.dyntype = mjDYN_NONE;
  actuator.dynprm[0] = 1;
  actuator.actdim = -1;

  // transmission
  actuator.trntype = mjTRN_UNDEFINED;
  actuator.gear[0] = 1;

  // input/output clamping
  actuator.ctrllimited = mjLIMITED_AUTO;
  actuator.forcelimited = mjLIMITED_AUTO;
  actuator.actlimited = mjLIMITED_AUTO;
}



// default sensor attributes
void mjm_defaultSensor(mjmSensor& sensor) {
  memset(&sensor, 0, sizeof(mjmSensor));
  sensor.type = mjSENS_TOUCH;
  sensor.datatype = mjDATATYPE_REAL;
  sensor.needstage = mjSTAGE_ACC;
}



// Default numeric attributes.
void mjm_defaultNumeric(mjmNumeric& numeric) {
  memset(&numeric, 0, sizeof(mjmNumeric));
}



// Default text attributes.
void mjm_defaultText(mjmText& text) {
  memset(&text, 0, sizeof(mjmText));
}



// Default tuple attributes.
void mjm_defaultTuple(mjmTuple& tuple) {
  memset(&tuple, 0, sizeof(mjmTuple));
}



// Default keyframe attributes.
void mjm_defaultKey(mjmKey& key) {
  memset(&key, 0, sizeof(mjmKey));
}



// default plugin attributes
void mjm_defaultPlugin(mjmPlugin& plugin) {
  memset(&plugin, 0, sizeof(mjmPlugin));
  plugin.plugin_slot = -1;
}

