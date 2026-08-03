// Copyright 2026 DeepMind Technologies Limited
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

// GENERATED FILE, DO NOT EDIT. Generated from src/xml/mjcf.schema by
// doc/generate/generate_default_table.py; test/doc/doc_test.py checks
// freshness.
//
// One row per schema attribute with a declared default, binding the declared
// values to the bound field. SchemaDefaultsTest compares every row against a
// freshly-constructed spec: the schema's defaults must agree with the C
// default-constructors. Rows are {attr, offset, kind, len, ndecl, values};
// values beyond ndecl are expected to be zero; kind: 0=double 1=float 2=int
// 3=byte 4=mjtNum.

// clang-format off
struct mjXDefaultEntry {
  const char* attr;
  int offset;
  int kind;
  int len;
  int ndecl;
  double value[8];
};

struct mjXDefaultTable {
  const char* structname;
  const mjXDefaultEntry* entries;
  int n;
};


static const mjXDefaultEntry kDefaults_mjOption[] = {
  {"timestep", (int)offsetof(mjOption, timestep), 4, 1, 1, {0.002}},
  {"impratio", (int)offsetof(mjOption, impratio), 4, 1, 1, {1.0}},
  {"tolerance", (int)offsetof(mjOption, tolerance), 4, 1, 1, {1e-08}},
  {"ls_tolerance", (int)offsetof(mjOption, ls_tolerance), 4, 1, 1, {0.01}},
  {"noslip_tolerance", (int)offsetof(mjOption, noslip_tolerance), 4, 1, 1, {1e-06}},
  {"ccd_tolerance", (int)offsetof(mjOption, ccd_tolerance), 4, 1, 1, {1e-06}},
  {"sleep_tolerance", (int)offsetof(mjOption, sleep_tolerance), 4, 1, 1, {0.001}},
  {"gravity", (int)offsetof(mjOption, gravity), 4, 3, 3, {0.0, 0.0, -9.81}},
  {"wind", (int)offsetof(mjOption, wind), 4, 3, 3, {0.0, 0.0, 0.0}},
  {"magnetic", (int)offsetof(mjOption, magnetic), 4, 3, 3, {0.0, -0.5, 0.0}},
  {"o_friction", (int)offsetof(mjOption, o_friction), 4, 5, 5, {1.0, 1.0, 0.005, 0.0001, 0.0001}},
  {"integrator", (int)offsetof(mjOption, integrator), 2, 1, 1, {(double)mjINT_EULER}},
  {"cone", (int)offsetof(mjOption, cone), 2, 1, 1, {(double)mjCONE_PYRAMIDAL}},
  {"jacobian", (int)offsetof(mjOption, jacobian), 2, 1, 1, {(double)mjJAC_AUTO}},
  {"solver", (int)offsetof(mjOption, solver), 2, 1, 1, {(double)mjSOL_NEWTON}},
  {"iterations", (int)offsetof(mjOption, iterations), 2, 1, 1, {100.0}},
  {"ls_iterations", (int)offsetof(mjOption, ls_iterations), 2, 1, 1, {50.0}},
  {"ccd_iterations", (int)offsetof(mjOption, ccd_iterations), 2, 1, 1, {35.0}},
  {"sdf_iterations", (int)offsetof(mjOption, sdf_iterations), 2, 1, 1, {10.0}},
  {"sdf_initpoints", (int)offsetof(mjOption, sdf_initpoints), 2, 1, 1, {40.0}},
};

static const mjXDefaultEntry kDefaults_mjVisual_global[] = {
  {"cameraid", (int)offsetof(mjVisual, global.cameraid), 2, 1, 1, {-1.0}},
  {"orthographic", (int)offsetof(mjVisual, global.orthographic), 2, 1, 1, {0}},
  {"fovy", (int)offsetof(mjVisual, global.fovy), 1, 1, 1, {45.0}},
  {"ipd", (int)offsetof(mjVisual, global.ipd), 1, 1, 1, {0.068}},
  {"azimuth", (int)offsetof(mjVisual, global.azimuth), 1, 1, 1, {90.0}},
  {"elevation", (int)offsetof(mjVisual, global.elevation), 1, 1, 1, {-45.0}},
  {"linewidth", (int)offsetof(mjVisual, global.linewidth), 1, 1, 1, {1.0}},
  {"glow", (int)offsetof(mjVisual, global.glow), 1, 1, 1, {0.3}},
  {"offwidth", (int)offsetof(mjVisual, global.offwidth), 2, 1, 1, {640.0}},
  {"offheight", (int)offsetof(mjVisual, global.offheight), 2, 1, 1, {480.0}},
  {"realtime", (int)offsetof(mjVisual, global.realtime), 1, 1, 1, {1.0}},
  {"ellipsoidinertia", (int)offsetof(mjVisual, global.ellipsoidinertia), 2, 1, 1, {0}},
  {"bvactive", (int)offsetof(mjVisual, global.bvactive), 2, 1, 1, {1}},
};

static const mjXDefaultEntry kDefaults_mjVisual_headlight[] = {
  {"ambient", (int)offsetof(mjVisual, headlight.ambient), 1, 3, 3, {0.1, 0.1, 0.1}},
  {"diffuse", (int)offsetof(mjVisual, headlight.diffuse), 1, 3, 3, {0.4, 0.4, 0.4}},
  {"specular", (int)offsetof(mjVisual, headlight.specular), 1, 3, 3, {0.5, 0.5, 0.5}},
  {"active", (int)offsetof(mjVisual, headlight.active), 2, 1, 1, {1.0}},
};

static const mjXDefaultEntry kDefaults_mjVisual_map[] = {
  {"stiffness", (int)offsetof(mjVisual, map.stiffness), 1, 1, 1, {100.0}},
  {"stiffnessrot", (int)offsetof(mjVisual, map.stiffnessrot), 1, 1, 1, {500.0}},
  {"force", (int)offsetof(mjVisual, map.force), 1, 1, 1, {0.005}},
  {"torque", (int)offsetof(mjVisual, map.torque), 1, 1, 1, {0.1}},
  {"alpha", (int)offsetof(mjVisual, map.alpha), 1, 1, 1, {0.3}},
  {"fogstart", (int)offsetof(mjVisual, map.fogstart), 1, 1, 1, {3.0}},
  {"fogend", (int)offsetof(mjVisual, map.fogend), 1, 1, 1, {10.0}},
  {"znear", (int)offsetof(mjVisual, map.znear), 1, 1, 1, {0.01}},
  {"zfar", (int)offsetof(mjVisual, map.zfar), 1, 1, 1, {50.0}},
  {"haze", (int)offsetof(mjVisual, map.haze), 1, 1, 1, {0.3}},
  {"shadowclip", (int)offsetof(mjVisual, map.shadowclip), 1, 1, 1, {1.0}},
  {"shadowscale", (int)offsetof(mjVisual, map.shadowscale), 1, 1, 1, {0.6}},
  {"actuatortendon", (int)offsetof(mjVisual, map.actuatortendon), 1, 1, 1, {2.0}},
};

static const mjXDefaultEntry kDefaults_mjVisual_quality[] = {
  {"shadowsize", (int)offsetof(mjVisual, quality.shadowsize), 2, 1, 1, {4096.0}},
  {"offsamples", (int)offsetof(mjVisual, quality.offsamples), 2, 1, 1, {4.0}},
  {"numslices", (int)offsetof(mjVisual, quality.numslices), 2, 1, 1, {28.0}},
  {"numstacks", (int)offsetof(mjVisual, quality.numstacks), 2, 1, 1, {16.0}},
  {"numquads", (int)offsetof(mjVisual, quality.numquads), 2, 1, 1, {4.0}},
};

static const mjXDefaultEntry kDefaults_mjVisual_scale[] = {
  {"forcewidth", (int)offsetof(mjVisual, scale.forcewidth), 1, 1, 1, {0.1}},
  {"contactwidth", (int)offsetof(mjVisual, scale.contactwidth), 1, 1, 1, {0.3}},
  {"contactheight", (int)offsetof(mjVisual, scale.contactheight), 1, 1, 1, {0.1}},
  {"connect", (int)offsetof(mjVisual, scale.connect), 1, 1, 1, {0.2}},
  {"com", (int)offsetof(mjVisual, scale.com), 1, 1, 1, {0.4}},
  {"camera", (int)offsetof(mjVisual, scale.camera), 1, 1, 1, {0.3}},
  {"light", (int)offsetof(mjVisual, scale.light), 1, 1, 1, {0.3}},
  {"selectpoint", (int)offsetof(mjVisual, scale.selectpoint), 1, 1, 1, {0.2}},
  {"jointlength", (int)offsetof(mjVisual, scale.jointlength), 1, 1, 1, {1.0}},
  {"jointwidth", (int)offsetof(mjVisual, scale.jointwidth), 1, 1, 1, {0.1}},
  {"actuatorlength", (int)offsetof(mjVisual, scale.actuatorlength), 1, 1, 1, {0.7}},
  {"actuatorwidth", (int)offsetof(mjVisual, scale.actuatorwidth), 1, 1, 1, {0.2}},
  {"framelength", (int)offsetof(mjVisual, scale.framelength), 1, 1, 1, {1.0}},
  {"framewidth", (int)offsetof(mjVisual, scale.framewidth), 1, 1, 1, {0.1}},
  {"constraint", (int)offsetof(mjVisual, scale.constraint), 1, 1, 1, {0.1}},
  {"slidercrank", (int)offsetof(mjVisual, scale.slidercrank), 1, 1, 1, {0.2}},
  {"frustum", (int)offsetof(mjVisual, scale.frustum), 1, 1, 1, {10.0}},
};

static const mjXDefaultEntry kDefaults_mjsActuator[] = {
  {"gear", (int)offsetof(mjsActuator, gear), 0, 6, 6, {1.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
  {"actdim", (int)offsetof(mjsActuator, actdim), 2, 1, 1, {-1.0}},
  {"dyntype", (int)offsetof(mjsActuator, dyntype), 2, 1, 1, {(double)mjDYN_NONE}},
  {"gaintype", (int)offsetof(mjsActuator, gaintype), 2, 1, 1, {(double)mjGAIN_FIXED}},
  {"biastype", (int)offsetof(mjsActuator, biastype), 2, 1, 1, {(double)mjBIAS_NONE}},
  {"dynprm", (int)offsetof(mjsActuator, dynprm), 0, mjNDYN, 1, {1.0}},
  {"gainprm", (int)offsetof(mjsActuator, gainprm), 0, mjNGAIN, 1, {1.0}},
};

static const mjXDefaultEntry kDefaults_mjsBody[] = {
  {"pos", (int)offsetof(mjsBody, pos), 0, 3, 3, {0.0, 0.0, 0.0}},
  {"quat", (int)offsetof(mjsBody, quat), 0, 4, 4, {1.0, 0.0, 0.0, 0.0}},
  {"simple", (int)offsetof(mjsBody, simple), 3, 1, 1, {(double)1}},
};

static const mjXDefaultEntry kDefaults_mjsCamera[] = {
  {"fovy", (int)offsetof(mjsCamera, fovy), 0, 1, 1, {45.0}},
  {"ipd", (int)offsetof(mjsCamera, ipd), 0, 1, 1, {0.068}},
  {"resolution", (int)offsetof(mjsCamera, resolution), 2, 2, 2, {1.0, 1.0}},
  {"pos", (int)offsetof(mjsCamera, pos), 0, 3, 3, {0.0, 0.0, 0.0}},
  {"quat", (int)offsetof(mjsCamera, quat), 0, 4, 4, {1.0, 0.0, 0.0, 0.0}},
  {"mode", (int)offsetof(mjsCamera, mode), 2, 1, 1, {(double)mjCAMLIGHT_FIXED}},
};

static const mjXDefaultEntry kDefaults_mjsEquality[] = {
  {"active", (int)offsetof(mjsEquality, active), 3, 1, 1, {1}},
  {"solref", (int)offsetof(mjsEquality, solref), 4, mjNREF, 2, {0.02, 1.0}},
  {"solimp", (int)offsetof(mjsEquality, solimp), 4, mjNIMP, 5, {0.9, 0.95, 0.001, 0.5, 2.0}},
};

static const mjXDefaultEntry kDefaults_mjsFrame[] = {
  {"quat", (int)offsetof(mjsFrame, quat), 0, 4, 4, {1.0, 0.0, 0.0, 0.0}},
};

static const mjXDefaultEntry kDefaults_mjsGeom[] = {
  {"type", (int)offsetof(mjsGeom, type), 2, 1, 1, {(double)mjGEOM_SPHERE}},
  {"contype", (int)offsetof(mjsGeom, contype), 2, 1, 1, {1.0}},
  {"conaffinity", (int)offsetof(mjsGeom, conaffinity), 2, 1, 1, {1.0}},
  {"condim", (int)offsetof(mjsGeom, condim), 2, 1, 1, {3.0}},
  {"friction", (int)offsetof(mjsGeom, friction), 0, 3, 3, {1.0, 0.005, 0.0001}},
  {"density", (int)offsetof(mjsGeom, density), 0, 1, 1, {1000.0}},
  {"solmix", (int)offsetof(mjsGeom, solmix), 0, 1, 1, {1.0}},
  {"solref", (int)offsetof(mjsGeom, solref), 4, mjNREF, 2, {0.02, 1.0}},
  {"solimp", (int)offsetof(mjsGeom, solimp), 4, mjNIMP, 5, {0.9, 0.95, 0.001, 0.5, 2.0}},
  {"pos", (int)offsetof(mjsGeom, pos), 0, 3, 3, {0.0, 0.0, 0.0}},
  {"quat", (int)offsetof(mjsGeom, quat), 0, 4, 4, {1.0, 0.0, 0.0, 0.0}},
  {"fitscale", (int)offsetof(mjsGeom, fitscale), 0, 1, 1, {1.0}},
  {"rgba", (int)offsetof(mjsGeom, rgba), 1, 4, 4, {0.5, 0.5, 0.5, 1.0}},
};

static const mjXDefaultEntry kDefaults_mjsJoint[] = {
  {"type", (int)offsetof(mjsJoint, type), 2, 1, 1, {(double)mjJNT_HINGE}},
  {"axis", (int)offsetof(mjsJoint, axis), 0, 3, 3, {0.0, 0.0, 1.0}},
};

static const mjXDefaultEntry kDefaults_mjsLight[] = {
  {"castshadow", (int)offsetof(mjsLight, castshadow), 3, 1, 1, {1}},
  {"active", (int)offsetof(mjsLight, active), 3, 1, 1, {1}},
  {"pos", (int)offsetof(mjsLight, pos), 0, 3, 3, {0.0, 0.0, 0.0}},
  {"dir", (int)offsetof(mjsLight, dir), 0, 3, 3, {0.0, 0.0, -1.0}},
  {"bulbradius", (int)offsetof(mjsLight, bulbradius), 1, 1, 1, {0.02}},
  {"range", (int)offsetof(mjsLight, range), 1, 1, 1, {10.0}},
  {"attenuation", (int)offsetof(mjsLight, attenuation), 1, 3, 3, {1.0, 0.0, 0.0}},
  {"cutoff", (int)offsetof(mjsLight, cutoff), 1, 1, 1, {45.0}},
  {"exponent", (int)offsetof(mjsLight, exponent), 1, 1, 1, {10.0}},
  {"diffuse", (int)offsetof(mjsLight, diffuse), 1, 3, 3, {0.7, 0.7, 0.7}},
  {"specular", (int)offsetof(mjsLight, specular), 1, 3, 3, {0.3, 0.3, 0.3}},
  {"mode", (int)offsetof(mjsLight, mode), 2, 1, 1, {(double)mjCAMLIGHT_FIXED}},
};

static const mjXDefaultEntry kDefaults_mjsMaterial[] = {
  {"texrepeat", (int)offsetof(mjsMaterial, texrepeat), 1, 2, 2, {1.0, 1.0}},
  {"specular", (int)offsetof(mjsMaterial, specular), 1, 1, 1, {0.5}},
  {"shininess", (int)offsetof(mjsMaterial, shininess), 1, 1, 1, {0.5}},
  {"metallic", (int)offsetof(mjsMaterial, metallic), 1, 1, 1, {-1.0}},
  {"roughness", (int)offsetof(mjsMaterial, roughness), 1, 1, 1, {-1.0}},
  {"rgba", (int)offsetof(mjsMaterial, rgba), 1, 4, 4, {1.0, 1.0, 1.0, 1.0}},
};

static const mjXDefaultEntry kDefaults_mjsMesh[] = {
  {"refquat", (int)offsetof(mjsMesh, refquat), 0, 4, 4, {1.0, 0.0, 0.0, 0.0}},
  {"scale", (int)offsetof(mjsMesh, scale), 0, 3, 3, {1.0, 1.0, 1.0}},
  {"maxhullvert", (int)offsetof(mjsMesh, maxhullvert), 2, 1, 1, {-1.0}},
  {"inertia", (int)offsetof(mjsMesh, inertia), 2, 1, 1, {(double)mjMESH_INERTIA_LEGACY}},
};

static const mjXDefaultEntry kDefaults_mjsPair[] = {
  {"condim", (int)offsetof(mjsPair, condim), 2, 1, 1, {3.0}},
  {"friction", (int)offsetof(mjsPair, friction), 0, 5, 5, {1.0, 1.0, 0.005, 0.0001, 0.0001}},
  {"solref", (int)offsetof(mjsPair, solref), 4, mjNREF, 2, {0.02, 1.0}},
  {"solimp", (int)offsetof(mjsPair, solimp), 4, mjNIMP, 5, {0.9, 0.95, 0.001, 0.5, 2.0}},
};

static const mjXDefaultEntry kDefaults_mjsSensor[] = {
  {"datatype", (int)offsetof(mjsSensor, datatype), 2, 1, 1, {(double)mjDATATYPE_REAL}},
  {"needstage", (int)offsetof(mjsSensor, needstage), 2, 1, 1, {(double)mjSTAGE_ACC}},
};

static const mjXDefaultEntry kDefaults_mjsSite[] = {
  {"type", (int)offsetof(mjsSite, type), 2, 1, 1, {(double)mjGEOM_SPHERE}},
  {"pos", (int)offsetof(mjsSite, pos), 0, 3, 3, {0.0, 0.0, 0.0}},
  {"quat", (int)offsetof(mjsSite, quat), 0, 4, 4, {1.0, 0.0, 0.0, 0.0}},
  {"size", (int)offsetof(mjsSite, size), 0, 3, 3, {0.005, 0.005, 0.005}},
  {"rgba", (int)offsetof(mjsSite, rgba), 1, 4, 4, {0.5, 0.5, 0.5, 1.0}},
};

static const mjXDefaultEntry kDefaults_mjsSkin[] = {
  {"rgba", (int)offsetof(mjsSkin, rgba), 1, 4, 4, {0.5, 0.5, 0.5, 1.0}},
};

static const mjXDefaultEntry kDefaults_mjsTendon[] = {
  {"springlength", (int)offsetof(mjsTendon, springlength), 0, 2, 2, {-1.0, -1.0}},
  {"width", (int)offsetof(mjsTendon, width), 0, 1, 1, {0.003}},
  {"rgba", (int)offsetof(mjsTendon, rgba), 1, 4, 4, {0.5, 0.5, 0.5, 1.0}},
};

static const mjXDefaultEntry kDefaults_mjsTexture[] = {
  {"type", (int)offsetof(mjsTexture, type), 2, 1, 1, {(double)mjTEXTURE_CUBE}},
  {"colorspace", (int)offsetof(mjsTexture, colorspace), 2, 1, 1, {(double)mjCOLORSPACE_AUTO}},
  {"gridsize", (int)offsetof(mjsTexture, gridsize), 2, 2, 2, {1.0, 1.0}},
  {"rgb1", (int)offsetof(mjsTexture, rgb1), 0, 3, 3, {0.8, 0.8, 0.8}},
  {"rgb2", (int)offsetof(mjsTexture, rgb2), 0, 3, 3, {0.5, 0.5, 0.5}},
  {"random", (int)offsetof(mjsTexture, random), 0, 1, 1, {0.01}},
  {"nchannel", (int)offsetof(mjsTexture, nchannel), 2, 1, 1, {3.0}},
};

static const mjXDefaultTable kDefaultTables[] = {
  {"mjOption", kDefaults_mjOption, (int)(sizeof(kDefaults_mjOption) / sizeof(kDefaults_mjOption[0]))},
  {"mjVisual", kDefaults_mjVisual_global, (int)(sizeof(kDefaults_mjVisual_global) / sizeof(kDefaults_mjVisual_global[0]))},
  {"mjVisual", kDefaults_mjVisual_headlight, (int)(sizeof(kDefaults_mjVisual_headlight) / sizeof(kDefaults_mjVisual_headlight[0]))},
  {"mjVisual", kDefaults_mjVisual_map, (int)(sizeof(kDefaults_mjVisual_map) / sizeof(kDefaults_mjVisual_map[0]))},
  {"mjVisual", kDefaults_mjVisual_quality, (int)(sizeof(kDefaults_mjVisual_quality) / sizeof(kDefaults_mjVisual_quality[0]))},
  {"mjVisual", kDefaults_mjVisual_scale, (int)(sizeof(kDefaults_mjVisual_scale) / sizeof(kDefaults_mjVisual_scale[0]))},
  {"mjsActuator", kDefaults_mjsActuator, (int)(sizeof(kDefaults_mjsActuator) / sizeof(kDefaults_mjsActuator[0]))},
  {"mjsBody", kDefaults_mjsBody, (int)(sizeof(kDefaults_mjsBody) / sizeof(kDefaults_mjsBody[0]))},
  {"mjsCamera", kDefaults_mjsCamera, (int)(sizeof(kDefaults_mjsCamera) / sizeof(kDefaults_mjsCamera[0]))},
  {"mjsEquality", kDefaults_mjsEquality, (int)(sizeof(kDefaults_mjsEquality) / sizeof(kDefaults_mjsEquality[0]))},
  {"mjsFrame", kDefaults_mjsFrame, (int)(sizeof(kDefaults_mjsFrame) / sizeof(kDefaults_mjsFrame[0]))},
  {"mjsGeom", kDefaults_mjsGeom, (int)(sizeof(kDefaults_mjsGeom) / sizeof(kDefaults_mjsGeom[0]))},
  {"mjsJoint", kDefaults_mjsJoint, (int)(sizeof(kDefaults_mjsJoint) / sizeof(kDefaults_mjsJoint[0]))},
  {"mjsLight", kDefaults_mjsLight, (int)(sizeof(kDefaults_mjsLight) / sizeof(kDefaults_mjsLight[0]))},
  {"mjsMaterial", kDefaults_mjsMaterial, (int)(sizeof(kDefaults_mjsMaterial) / sizeof(kDefaults_mjsMaterial[0]))},
  {"mjsMesh", kDefaults_mjsMesh, (int)(sizeof(kDefaults_mjsMesh) / sizeof(kDefaults_mjsMesh[0]))},
  {"mjsPair", kDefaults_mjsPair, (int)(sizeof(kDefaults_mjsPair) / sizeof(kDefaults_mjsPair[0]))},
  {"mjsSensor", kDefaults_mjsSensor, (int)(sizeof(kDefaults_mjsSensor) / sizeof(kDefaults_mjsSensor[0]))},
  {"mjsSite", kDefaults_mjsSite, (int)(sizeof(kDefaults_mjsSite) / sizeof(kDefaults_mjsSite[0]))},
  {"mjsSkin", kDefaults_mjsSkin, (int)(sizeof(kDefaults_mjsSkin) / sizeof(kDefaults_mjsSkin[0]))},
  {"mjsTendon", kDefaults_mjsTendon, (int)(sizeof(kDefaults_mjsTendon) / sizeof(kDefaults_mjsTendon[0]))},
  {"mjsTexture", kDefaults_mjsTexture, (int)(sizeof(kDefaults_mjsTexture) / sizeof(kDefaults_mjsTexture[0]))},
};
static const int kDefaultTablesN = (int)(sizeof(kDefaultTables) / sizeof(kDefaultTables[0]));
// clang-format on
