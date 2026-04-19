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

// XSD generator for MJCF.
//
// The MJCF language is described in three places inside this directory:
//   1. MJCF[]   (xml_native_reader.cc) -- element tree + attribute names.
//   2. *_map    (xml_native_reader.cc) -- enum value sets.
//   3. OneX()   (xml_native_reader.cc) -- the ReadAttr*/MapValue calls that
//                                        assign types and enum maps to attrs.
//
// (1) and (2) are already declarative and reused directly. (3) is not, so
// this file adds a single flat table `kMjXAttrTable` mirroring the type
// decisions made in the OneX() parsers.
//
// ---------------------------------------------------------------------------
// MAINTENANCE: if you add/rename an attribute in the parser, you also need
// to touch this file. Concretely, when you edit `OneX()` in
// xml_native_reader.cc (or add an entry to MJCF[]):
//
//   * add a row to `kMjXAttrTable` below (one per (element, attr) pair)
//     matching the ReadAttr*/MapValue call you wrote -- this is the only
//     per-attribute maintenance burden.
//   * if you added a new enum map, declare it `extern` at the top of the
//     reader (most already are), and forward-declare it at the top of this
//     file alongside the existing ones.
//   * document the attribute under `.. _<element>-<attr>:` in
//     doc/XMLreference.rst (default value + prose) -- the enrichment pass in
//     doc/mjcf_schema_enrich.py picks this up and injects it as
//     <xs:annotation> on the generated xs:attribute.
//
// Release pipeline (regenerating doc/mjcf.xsd):
//   $ cmake --build build --target xmlschema
//   $ ./build/bin/xmlschema /tmp/raw.xsd
//   $ python3 doc/mjcf_schema_enrich.py --in /tmp/raw.xsd \
//         --rst doc/XMLreference.rst --out doc/mjcf.xsd --strict --report
//
// `--strict` exits non-zero if the enricher finds drift between the C table,
// the RST, and the schema -- wire this into CI to catch stale docs.

#include "xml/xml_native_schema.h"

#include <sstream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "xml/xml_native_reader.h"  // exports MJCF[] and nMJCF
#include "xml/xml_util.h"           // exports mjMap

// Enum-map symbols and their sizes live at global scope in xml_native_reader.cc
// (inside a nameless namespace that closes before they are defined). Redeclare
// them here so the linker resolves them.
extern const mjMap coordinate_map[2];
extern const mjMap angle_map[2];
extern const mjMap bool_map[2];
extern const mjMap fluid_map[2];
extern const mjMap TFAuto_map[3];
extern const mjMap enable_map[2];
extern const mjMap bodysleep_map[];
extern const int   bodysleep_sz;
extern const mjMap joint_map[];
extern const int   joint_sz;
extern const mjMap geom_map[];
extern const mjMap projection_map[];
extern const int   projection_sz;
extern const mjMap camlight_map[];
extern const int   camlight_sz;
extern const mjMap lighttype_map[];
extern const int   lighttype_sz;
extern const mjMap texrole_map[];
extern const int   texrole_sz;
extern const mjMap integrator_map[];
extern const int   integrator_sz;
extern const mjMap cone_map[];
extern const int   cone_sz;
extern const mjMap jac_map[];
extern const int   jac_sz;
extern const mjMap solver_map[];
extern const int   solver_sz;
extern const mjMap equality_map[];
extern const int   equality_sz;
extern const mjMap texture_map[];
extern const int   texture_sz;
extern const mjMap colorspace_map[];
extern const int   colorspace_sz;
extern const mjMap builtin_map[];
extern const int   builtin_sz;
extern const mjMap mark_map[];
extern const int   mark_sz;
extern const mjMap dyn_map[];
extern const int   dyn_sz;
extern const mjMap dcmotorinput_map[];
extern const int   dcmotorinput_sz;
extern const mjMap gain_map[];
extern const int   gain_sz;
extern const mjMap bias_map[];
extern const int   bias_sz;
extern const mjMap interp_map[];
extern const int   interp_sz;
extern const mjMap stage_map[];
extern const int   stage_sz;
extern const mjMap datatype_map[];
extern const int   datatype_sz;
extern const mjMap condata_map[];
extern const mjMap raydata_map[];
extern const mjMap camout_map[];
extern const int   camout_sz;
extern const mjMap reduce_map[];
extern const int   reduce_sz;
extern const mjMap lrmode_map[];
extern const int   lrmode_sz;
extern const mjMap comp_map[];
extern const mjMap jkind_map[1];
extern const mjMap shape_map[];
extern const mjMap meshtype_map[2];
extern const mjMap meshinertia_map[4];
extern const mjMap meshbuiltin_map[];
extern const int   meshbuiltin_sz;
extern const mjMap fcomp_map[];
extern const mjMap fdof_map[];
extern const mjMap flexself_map[5];
extern const mjMap elastic2d_map[5];
extern const mjMap flexeq_map[4];

// Literal sizes for the few remaining maps that are sized via mjN* enum
// constants; kept local rather than pulling in extra headers.
namespace {
constexpr int kGeomTypesSz  = 9;   // mjNGEOMTYPES
constexpr int kCompTypesSz  = 6;   // mjNCOMPTYPES
constexpr int kFcompTypesSz = 10;  // mjNFCOMPTYPES
constexpr int kFcompDofsSz  = 4;   // mjNFCOMPDOFS
constexpr int kNDYN         = 10;  // mjNDYN
constexpr int kNGAIN        = 10;  // mjNGAIN
constexpr int kNBIAS        = 10;  // mjNBIAS
}  // namespace

namespace mujoco {


//---------------------------------- Attribute type table ------------------------------------------

// clang-format off
const mjXAttr kMjXAttrTable[] = {

  //-------------------------------- <mujoco> ------------------------------------------------------
  {"mujoco", "model", kMjXString, 0, nullptr, 0},

  //-------------------------------- <compiler> ----------------------------------------------------
  {"compiler", "autolimits",        kMjXEnum,   0, bool_map,        2},
  {"compiler", "boundmass",         kMjXReal,   0, nullptr,         0},
  {"compiler", "boundinertia",      kMjXReal,   0, nullptr,         0},
  {"compiler", "settotalmass",      kMjXReal,   0, nullptr,         0},
  {"compiler", "balanceinertia",    kMjXEnum,   0, bool_map,        2},
  {"compiler", "strippath",         kMjXEnum,   0, bool_map,        2},
  {"compiler", "coordinate",        kMjXEnum,   0, coordinate_map,  2},
  {"compiler", "angle",             kMjXEnum,   0, angle_map,       2},
  {"compiler", "fitaabb",           kMjXEnum,   0, bool_map,        2},
  {"compiler", "eulerseq",          kMjXString, 0, nullptr,         0},
  {"compiler", "meshdir",           kMjXString, 0, nullptr,         0},
  {"compiler", "texturedir",        kMjXString, 0, nullptr,         0},
  {"compiler", "discardvisual",     kMjXEnum,   0, bool_map,        2},
  {"compiler", "usethread",         kMjXEnum,   0, bool_map,        2},
  {"compiler", "fusestatic",        kMjXEnum,   0, bool_map,        2},
  {"compiler", "inertiafromgeom",   kMjXEnum,   0, TFAuto_map,      3},
  {"compiler", "inertiagrouprange", kMjXIntN,   2, nullptr,         0},
  {"compiler", "saveinertial",      kMjXEnum,   0, bool_map,        2},
  {"compiler", "assetdir",          kMjXString, 0, nullptr,         0},
  {"compiler", "alignfree",         kMjXEnum,   0, bool_map,        2},

  //-------------------------------- <compiler><lengthrange> ---------------------------------------
  {"lengthrange", "mode",        kMjXEnum, 0, lrmode_map, 4},
  {"lengthrange", "useexisting", kMjXEnum, 0, bool_map,   2},
  {"lengthrange", "uselimit",    kMjXEnum, 0, bool_map,   2},
  {"lengthrange", "accel",       kMjXReal, 0, nullptr,    0},
  {"lengthrange", "maxforce",    kMjXReal, 0, nullptr,    0},
  {"lengthrange", "timeconst",   kMjXReal, 0, nullptr,    0},
  {"lengthrange", "timestep",    kMjXReal, 0, nullptr,    0},
  {"lengthrange", "inttotal",    kMjXReal, 0, nullptr,    0},
  {"lengthrange", "interval",    kMjXReal, 0, nullptr,    0},
  {"lengthrange", "tolrange",    kMjXReal, 0, nullptr,    0},

  //-------------------------------- <option> ------------------------------------------------------
  {"option", "timestep",             kMjXReal,      0, nullptr,        0},
  {"option", "impratio",             kMjXReal,      0, nullptr,        0},
  {"option", "tolerance",            kMjXReal,      0, nullptr,        0},
  {"option", "ls_tolerance",         kMjXReal,      0, nullptr,        0},
  {"option", "noslip_tolerance",     kMjXReal,      0, nullptr,        0},
  {"option", "ccd_tolerance",        kMjXReal,      0, nullptr,        0},
  {"option", "sleep_tolerance",      kMjXReal,      0, nullptr,        0},
  {"option", "gravity",              kMjXRealN,     3, nullptr,        0},
  {"option", "wind",                 kMjXRealN,     3, nullptr,        0},
  {"option", "magnetic",             kMjXRealN,     3, nullptr,        0},
  {"option", "density",              kMjXReal,      0, nullptr,        0},
  {"option", "viscosity",            kMjXReal,      0, nullptr,        0},
  {"option", "o_margin",             kMjXReal,      0, nullptr,        0},
  {"option", "o_solref",             kMjXRealUpToN, 2, nullptr,        0},
  {"option", "o_solimp",             kMjXRealUpToN, 5, nullptr,        0},
  {"option", "o_friction",           kMjXRealUpToN, 5, nullptr,        0},
  {"option", "integrator",           kMjXEnum,      0, integrator_map, 4},
  {"option", "cone",                 kMjXEnum,      0, cone_map,       2},
  {"option", "jacobian",             kMjXEnum,      0, jac_map,        3},
  {"option", "solver",               kMjXEnum,      0, solver_map,     3},
  {"option", "iterations",           kMjXInt,       0, nullptr,        0},
  {"option", "ls_iterations",        kMjXInt,       0, nullptr,        0},
  {"option", "noslip_iterations",    kMjXInt,       0, nullptr,        0},
  {"option", "ccd_iterations",       kMjXInt,       0, nullptr,        0},
  {"option", "sdf_iterations",       kMjXInt,       0, nullptr,        0},
  {"option", "sdf_initpoints",       kMjXInt,       0, nullptr,        0},
  {"option", "actuatorgroupdisable", kMjXIntVec,    0, nullptr,        0},

  //-------------------------------- <option><flag> ------------------------------------------------
  {"flag", "constraint",   kMjXEnum, 0, enable_map, 2},
  {"flag", "equality",     kMjXEnum, 0, enable_map, 2},
  {"flag", "frictionloss", kMjXEnum, 0, enable_map, 2},
  {"flag", "limit",        kMjXEnum, 0, enable_map, 2},
  {"flag", "contact",      kMjXEnum, 0, enable_map, 2},
  {"flag", "spring",       kMjXEnum, 0, enable_map, 2},
  {"flag", "damper",       kMjXEnum, 0, enable_map, 2},
  {"flag", "gravity",      kMjXEnum, 0, enable_map, 2},
  {"flag", "clampctrl",    kMjXEnum, 0, enable_map, 2},
  {"flag", "warmstart",    kMjXEnum, 0, enable_map, 2},
  {"flag", "filterparent", kMjXEnum, 0, enable_map, 2},
  {"flag", "actuation",    kMjXEnum, 0, enable_map, 2},
  {"flag", "refsafe",      kMjXEnum, 0, enable_map, 2},
  {"flag", "sensor",       kMjXEnum, 0, enable_map, 2},
  {"flag", "midphase",     kMjXEnum, 0, enable_map, 2},
  {"flag", "eulerdamp",    kMjXEnum, 0, enable_map, 2},
  {"flag", "autoreset",    kMjXEnum, 0, enable_map, 2},
  {"flag", "nativeccd",    kMjXEnum, 0, enable_map, 2},
  {"flag", "island",       kMjXEnum, 0, enable_map, 2},
  {"flag", "override",     kMjXEnum, 0, enable_map, 2},
  {"flag", "energy",       kMjXEnum, 0, enable_map, 2},
  {"flag", "fwdinv",       kMjXEnum, 0, enable_map, 2},
  {"flag", "invdiscrete",  kMjXEnum, 0, enable_map, 2},
  {"flag", "multiccd",     kMjXEnum, 0, enable_map, 2},
  {"flag", "sleep",        kMjXEnum, 0, enable_map, 2},

  //-------------------------------- <size> --------------------------------------------------------
  {"size", "memory",         kMjXString, 0, nullptr, 0},
  {"size", "njmax",          kMjXInt,    0, nullptr, 0},
  {"size", "nconmax",        kMjXInt,    0, nullptr, 0},
  {"size", "nstack",         kMjXInt,    0, nullptr, 0},
  {"size", "nuserdata",      kMjXInt,    0, nullptr, 0},
  {"size", "nkey",           kMjXInt,    0, nullptr, 0},
  {"size", "nuser_body",     kMjXInt,    0, nullptr, 0},
  {"size", "nuser_jnt",      kMjXInt,    0, nullptr, 0},
  {"size", "nuser_geom",     kMjXInt,    0, nullptr, 0},
  {"size", "nuser_site",     kMjXInt,    0, nullptr, 0},
  {"size", "nuser_cam",      kMjXInt,    0, nullptr, 0},
  {"size", "nuser_tendon",   kMjXInt,    0, nullptr, 0},
  {"size", "nuser_actuator", kMjXInt,    0, nullptr, 0},
  {"size", "nuser_sensor",   kMjXInt,    0, nullptr, 0},

  //-------------------------------- <statistic> ---------------------------------------------------
  {"statistic", "meaninertia", kMjXReal,  0, nullptr, 0},
  {"statistic", "meanmass",    kMjXReal,  0, nullptr, 0},
  {"statistic", "meansize",    kMjXReal,  0, nullptr, 0},
  {"statistic", "extent",      kMjXReal,  0, nullptr, 0},
  {"statistic", "center",      kMjXRealN, 3, nullptr, 0},

  //-------------------------------- <visual> subelements ------------------------------------------
  {"global", "cameraid",         kMjXInt,  0, nullptr,  0},
  {"global", "orthographic",     kMjXEnum, 0, bool_map, 2},
  {"global", "fovy",             kMjXReal, 0, nullptr,  0},
  {"global", "ipd",              kMjXReal, 0, nullptr,  0},
  {"global", "azimuth",          kMjXReal, 0, nullptr,  0},
  {"global", "elevation",        kMjXReal, 0, nullptr,  0},
  {"global", "linewidth",        kMjXReal, 0, nullptr,  0},
  {"global", "glow",             kMjXReal, 0, nullptr,  0},
  {"global", "offwidth",         kMjXInt,  0, nullptr,  0},
  {"global", "offheight",        kMjXInt,  0, nullptr,  0},
  {"global", "realtime",         kMjXReal, 0, nullptr,  0},
  {"global", "ellipsoidinertia", kMjXEnum, 0, bool_map, 2},
  {"global", "bvactive",         kMjXEnum, 0, bool_map, 2},

  {"quality", "shadowsize", kMjXInt, 0, nullptr, 0},
  {"quality", "offsamples", kMjXInt, 0, nullptr, 0},
  {"quality", "numslices",  kMjXInt, 0, nullptr, 0},
  {"quality", "numstacks",  kMjXInt, 0, nullptr, 0},
  {"quality", "numquads",   kMjXInt, 0, nullptr, 0},

  {"headlight", "ambient",  kMjXRealN, 3, nullptr, 0},
  {"headlight", "diffuse",  kMjXRealN, 3, nullptr, 0},
  {"headlight", "specular", kMjXRealN, 3, nullptr, 0},
  {"headlight", "active",   kMjXInt,   0, nullptr, 0},

  {"map", "stiffness",      kMjXReal, 0, nullptr, 0},
  {"map", "stiffnessrot",   kMjXReal, 0, nullptr, 0},
  {"map", "force",          kMjXReal, 0, nullptr, 0},
  {"map", "torque",         kMjXReal, 0, nullptr, 0},
  {"map", "alpha",          kMjXReal, 0, nullptr, 0},
  {"map", "fogstart",       kMjXReal, 0, nullptr, 0},
  {"map", "fogend",         kMjXReal, 0, nullptr, 0},
  {"map", "znear",          kMjXReal, 0, nullptr, 0},
  {"map", "zfar",           kMjXReal, 0, nullptr, 0},
  {"map", "haze",           kMjXReal, 0, nullptr, 0},
  {"map", "shadowclip",     kMjXReal, 0, nullptr, 0},
  {"map", "shadowscale",    kMjXReal, 0, nullptr, 0},
  {"map", "actuatortendon", kMjXReal, 0, nullptr, 0},

  {"scale", "forcewidth",     kMjXReal, 0, nullptr, 0},
  {"scale", "contactwidth",   kMjXReal, 0, nullptr, 0},
  {"scale", "contactheight",  kMjXReal, 0, nullptr, 0},
  {"scale", "connect",        kMjXReal, 0, nullptr, 0},
  {"scale", "com",            kMjXReal, 0, nullptr, 0},
  {"scale", "camera",         kMjXReal, 0, nullptr, 0},
  {"scale", "light",          kMjXReal, 0, nullptr, 0},
  {"scale", "selectpoint",    kMjXReal, 0, nullptr, 0},
  {"scale", "jointlength",    kMjXReal, 0, nullptr, 0},
  {"scale", "jointwidth",     kMjXReal, 0, nullptr, 0},
  {"scale", "actuatorlength", kMjXReal, 0, nullptr, 0},
  {"scale", "actuatorwidth",  kMjXReal, 0, nullptr, 0},
  {"scale", "framelength",    kMjXReal, 0, nullptr, 0},
  {"scale", "framewidth",     kMjXReal, 0, nullptr, 0},
  {"scale", "constraint",     kMjXReal, 0, nullptr, 0},
  {"scale", "slidercrank",    kMjXReal, 0, nullptr, 0},
  {"scale", "frustum",        kMjXReal, 0, nullptr, 0},

  {"rgba", "fog",              kMjXRealN, 4, nullptr, 0},
  {"rgba", "haze",             kMjXRealN, 4, nullptr, 0},
  {"rgba", "force",            kMjXRealN, 4, nullptr, 0},
  {"rgba", "inertia",          kMjXRealN, 4, nullptr, 0},
  {"rgba", "joint",            kMjXRealN, 4, nullptr, 0},
  {"rgba", "actuator",         kMjXRealN, 4, nullptr, 0},
  {"rgba", "actuatornegative", kMjXRealN, 4, nullptr, 0},
  {"rgba", "actuatorpositive", kMjXRealN, 4, nullptr, 0},
  {"rgba", "com",              kMjXRealN, 4, nullptr, 0},
  {"rgba", "camera",           kMjXRealN, 4, nullptr, 0},
  {"rgba", "light",            kMjXRealN, 4, nullptr, 0},
  {"rgba", "selectpoint",      kMjXRealN, 4, nullptr, 0},
  {"rgba", "connect",          kMjXRealN, 4, nullptr, 0},
  {"rgba", "contactpoint",     kMjXRealN, 4, nullptr, 0},
  {"rgba", "contactforce",     kMjXRealN, 4, nullptr, 0},
  {"rgba", "contactfriction",  kMjXRealN, 4, nullptr, 0},
  {"rgba", "contacttorque",    kMjXRealN, 4, nullptr, 0},
  {"rgba", "contactgap",       kMjXRealN, 4, nullptr, 0},
  {"rgba", "rangefinder",      kMjXRealN, 4, nullptr, 0},
  {"rgba", "constraint",       kMjXRealN, 4, nullptr, 0},
  {"rgba", "slidercrank",      kMjXRealN, 4, nullptr, 0},
  {"rgba", "crankbroken",      kMjXRealN, 4, nullptr, 0},
  {"rgba", "frustum",          kMjXRealN, 4, nullptr, 0},
  {"rgba", "bv",               kMjXRealN, 4, nullptr, 0},
  {"rgba", "bvactive",         kMjXRealN, 4, nullptr, 0},

  //-------------------------------- <default> -----------------------------------------------------
  {"default", "class", kMjXString, 0, nullptr, 0},

  //-------------------------------- <mesh> (asset + default) -------------------------------------
  {"mesh", "name",         kMjXString,    0, nullptr,          0},
  {"mesh", "class",        kMjXString,    0, nullptr,          0},
  {"mesh", "content_type", kMjXString,    0, nullptr,          0},
  {"mesh", "file",         kMjXString,    0, nullptr,          0},
  {"mesh", "vertex",       kMjXRealVec,   0, nullptr,          0},
  {"mesh", "normal",       kMjXRealVec,   0, nullptr,          0},
  {"mesh", "texcoord",     kMjXRealVec,   0, nullptr,          0},
  {"mesh", "face",         kMjXIntVec,    0, nullptr,          0},
  {"mesh", "refpos",       kMjXRealN,     3, nullptr,          0},
  {"mesh", "refquat",      kMjXRealN,     4, nullptr,          0},
  {"mesh", "scale",        kMjXRealN,     3, nullptr,          0},
  {"mesh", "smoothnormal", kMjXEnum,      0, bool_map,         2},
  {"mesh", "maxhullvert",  kMjXInt,       0, nullptr,          0},
  {"mesh", "inertia",      kMjXEnum,      0, meshinertia_map,  4},
  {"mesh", "builtin",      kMjXEnum,      0, meshbuiltin_map,  8},
  {"mesh", "params",       kMjXRealVec,   0, nullptr,          0},
  {"mesh", "material",     kMjXString,    0, nullptr,          0},

  //-------------------------------- <material> (asset + default) ----------------------------------
  {"material", "name",        kMjXString, 0, nullptr,  0},
  {"material", "class",       kMjXString, 0, nullptr,  0},
  {"material", "texture",     kMjXString, 0, nullptr,  0},
  {"material", "texrepeat",   kMjXRealN,  2, nullptr,  0},
  {"material", "texuniform",  kMjXEnum,   0, bool_map, 2},
  {"material", "emission",    kMjXReal,   0, nullptr,  0},
  {"material", "specular",    kMjXReal,   0, nullptr,  0},
  {"material", "shininess",   kMjXReal,   0, nullptr,  0},
  {"material", "reflectance", kMjXReal,   0, nullptr,  0},
  {"material", "metallic",    kMjXReal,   0, nullptr,  0},
  {"material", "roughness",   kMjXReal,   0, nullptr,  0},
  {"material", "rgba",        kMjXRealN,  4, nullptr,  0},

  //-------------------------------- <material><layer> ---------------------------------------------
  {"layer", "texture", kMjXString, 0, nullptr,     0},
  {"layer", "role",    kMjXEnum,   0, texrole_map, 9},

  //-------------------------------- <joint> (body + default) --------------------------------------
  {"joint", "name",              kMjXString,    0, nullptr,    0},
  {"joint", "class",             kMjXString,    0, nullptr,    0},
  {"joint", "type",              kMjXEnum,      0, joint_map,  4},
  {"joint", "group",             kMjXInt,       0, nullptr,    0},
  {"joint", "pos",               kMjXRealN,     3, nullptr,    0},
  {"joint", "axis",              kMjXRealN,     3, nullptr,    0},
  {"joint", "springdamper",      kMjXRealN,     2, nullptr,    0},
  {"joint", "limited",           kMjXEnum,      0, TFAuto_map, 3},
  {"joint", "actuatorfrclimited",kMjXEnum,      0, TFAuto_map, 3},
  {"joint", "solreflimit",       kMjXRealUpToN, 2, nullptr,    0},
  {"joint", "solimplimit",       kMjXRealUpToN, 5, nullptr,    0},
  {"joint", "solreffriction",    kMjXRealUpToN, 2, nullptr,    0},
  {"joint", "solimpfriction",    kMjXRealUpToN, 5, nullptr,    0},
  {"joint", "stiffness",         kMjXRealUpToN, 3, nullptr,    0},
  {"joint", "range",             kMjXRealN,     2, nullptr,    0},
  {"joint", "actuatorfrcrange",  kMjXRealN,     2, nullptr,    0},
  {"joint", "actuatorgravcomp",  kMjXEnum,      0, bool_map,   2},
  {"joint", "margin",            kMjXReal,      0, nullptr,    0},
  {"joint", "ref",               kMjXReal,      0, nullptr,    0},
  {"joint", "springref",         kMjXReal,      0, nullptr,    0},
  {"joint", "armature",          kMjXReal,      0, nullptr,    0},
  {"joint", "damping",           kMjXRealUpToN, 3, nullptr,    0},
  {"joint", "frictionloss",      kMjXReal,      0, nullptr,    0},
  {"joint", "user",              kMjXRealVec,   0, nullptr,    0},

  //-------------------------------- <freejoint> (body) --------------------------------------------
  {"freejoint", "name",  kMjXString, 0, nullptr,    0},
  {"freejoint", "group", kMjXInt,    0, nullptr,    0},
  {"freejoint", "align", kMjXEnum,   0, TFAuto_map, 3},

  //-------------------------------- <geom> (body + default) ---------------------------------------
  {"geom", "name",         kMjXString,    0, nullptr,   0},
  {"geom", "class",        kMjXString,    0, nullptr,   0},
  {"geom", "type",         kMjXEnum,      0, geom_map,  kGeomTypesSz},
  {"geom", "pos",          kMjXRealN,     3, nullptr,   0},
  {"geom", "quat",         kMjXRealN,     4, nullptr,   0},
  {"geom", "axisangle",    kMjXRealN,     4, nullptr,   0},
  {"geom", "xyaxes",       kMjXRealN,     6, nullptr,   0},
  {"geom", "zaxis",        kMjXRealN,     3, nullptr,   0},
  {"geom", "euler",        kMjXRealN,     3, nullptr,   0},
  {"geom", "contype",      kMjXInt,       0, nullptr,   0},
  {"geom", "conaffinity",  kMjXInt,       0, nullptr,   0},
  {"geom", "condim",       kMjXInt,       0, nullptr,   0},
  {"geom", "group",        kMjXInt,       0, nullptr,   0},
  {"geom", "priority",     kMjXInt,       0, nullptr,   0},
  {"geom", "size",         kMjXRealUpToN, 3, nullptr,   0},
  {"geom", "material",     kMjXString,    0, nullptr,   0},
  {"geom", "friction",     kMjXRealUpToN, 3, nullptr,   0},
  {"geom", "mass",         kMjXReal,      0, nullptr,   0},
  {"geom", "density",      kMjXReal,      0, nullptr,   0},
  {"geom", "shellinertia", kMjXEnum,      0, meshtype_map, 2},
  {"geom", "solmix",       kMjXReal,      0, nullptr,   0},
  {"geom", "solref",       kMjXRealUpToN, 2, nullptr,   0},
  {"geom", "solimp",       kMjXRealUpToN, 5, nullptr,   0},
  {"geom", "margin",       kMjXReal,      0, nullptr,   0},
  {"geom", "gap",          kMjXReal,      0, nullptr,   0},
  {"geom", "fromto",       kMjXRealN,     6, nullptr,   0},
  {"geom", "hfield",       kMjXString,    0, nullptr,   0},
  {"geom", "mesh",         kMjXString,    0, nullptr,   0},
  {"geom", "fitscale",     kMjXReal,      0, nullptr,   0},
  {"geom", "rgba",         kMjXRealN,     4, nullptr,   0},
  {"geom", "fluidshape",   kMjXEnum,      0, fluid_map, 2},
  {"geom", "fluidcoef",    kMjXRealUpToN, 5, nullptr,   0},
  {"geom", "user",         kMjXRealVec,   0, nullptr,   0},

  //-------------------------------- <site> (body + default) ---------------------------------------
  {"site", "name",      kMjXString,    0, nullptr,  0},
  {"site", "class",     kMjXString,    0, nullptr,  0},
  {"site", "type",      kMjXEnum,      0, geom_map, kGeomTypesSz},
  {"site", "group",     kMjXInt,       0, nullptr,  0},
  {"site", "pos",       kMjXRealN,     3, nullptr,  0},
  {"site", "quat",      kMjXRealN,     4, nullptr,  0},
  {"site", "material",  kMjXString,    0, nullptr,  0},
  {"site", "size",      kMjXRealUpToN, 3, nullptr,  0},
  {"site", "fromto",    kMjXRealN,     6, nullptr,  0},
  {"site", "axisangle", kMjXRealN,     4, nullptr,  0},
  {"site", "xyaxes",    kMjXRealN,     6, nullptr,  0},
  {"site", "zaxis",     kMjXRealN,     3, nullptr,  0},
  {"site", "euler",     kMjXRealN,     3, nullptr,  0},
  {"site", "rgba",      kMjXRealN,     4, nullptr,  0},
  {"site", "user",      kMjXRealVec,   0, nullptr,  0},

  //-------------------------------- <camera> (body + default) -------------------------------------
  {"camera", "name",           kMjXString,  0, nullptr,        0},
  {"camera", "class",          kMjXString,  0, nullptr,        0},
  {"camera", "projection",     kMjXEnum,    0, projection_map, 2},
  {"camera", "fovy",           kMjXReal,    0, nullptr,        0},
  {"camera", "ipd",            kMjXReal,    0, nullptr,        0},
  {"camera", "resolution",     kMjXIntN,    2, nullptr,        0},
  {"camera", "output",         kMjXString,  0, nullptr,        0},  // space-separated bitflags
  {"camera", "pos",            kMjXRealN,   3, nullptr,        0},
  {"camera", "quat",           kMjXRealN,   4, nullptr,        0},
  {"camera", "axisangle",      kMjXRealN,   4, nullptr,        0},
  {"camera", "xyaxes",         kMjXRealN,   6, nullptr,        0},
  {"camera", "zaxis",          kMjXRealN,   3, nullptr,        0},
  {"camera", "euler",          kMjXRealN,   3, nullptr,        0},
  {"camera", "mode",           kMjXEnum,    0, camlight_map,   5},
  {"camera", "target",         kMjXString,  0, nullptr,        0},
  {"camera", "focal",          kMjXRealN,   2, nullptr,        0},
  {"camera", "focalpixel",     kMjXRealN,   2, nullptr,        0},
  {"camera", "principal",      kMjXRealN,   2, nullptr,        0},
  {"camera", "principalpixel", kMjXRealN,   2, nullptr,        0},
  {"camera", "sensorsize",     kMjXRealN,   2, nullptr,        0},
  {"camera", "user",           kMjXRealVec, 0, nullptr,        0},

  //-------------------------------- <light> (body + default) --------------------------------------
  {"light", "name",        kMjXString, 0, nullptr,       0},
  {"light", "class",       kMjXString, 0, nullptr,       0},
  {"light", "pos",         kMjXRealN,  3, nullptr,       0},
  {"light", "dir",         kMjXRealN,  3, nullptr,       0},
  {"light", "bulbradius",  kMjXReal,   0, nullptr,       0},
  {"light", "intensity",   kMjXReal,   0, nullptr,       0},
  {"light", "range",       kMjXReal,   0, nullptr,       0},
  {"light", "directional", kMjXEnum,   0, bool_map,      2},
  {"light", "type",        kMjXEnum,   0, lighttype_map, 4},
  {"light", "castshadow",  kMjXEnum,   0, bool_map,      2},
  {"light", "active",      kMjXEnum,   0, bool_map,      2},
  {"light", "attenuation", kMjXRealN,  3, nullptr,       0},
  {"light", "cutoff",      kMjXReal,   0, nullptr,       0},
  {"light", "exponent",    kMjXReal,   0, nullptr,       0},
  {"light", "ambient",     kMjXRealN,  3, nullptr,       0},
  {"light", "diffuse",     kMjXRealN,  3, nullptr,       0},
  {"light", "specular",    kMjXRealN,  3, nullptr,       0},
  {"light", "mode",        kMjXEnum,   0, camlight_map,  5},
  {"light", "target",      kMjXString, 0, nullptr,       0},
  {"light", "texture",     kMjXString, 0, nullptr,       0},

  //-------------------------------- <pair> (default + contact) -----------------------------------
  {"pair", "name",           kMjXString,    0, nullptr, 0},
  {"pair", "class",          kMjXString,    0, nullptr, 0},
  {"pair", "geom1",          kMjXString,    0, nullptr, 0},
  {"pair", "geom2",          kMjXString,    0, nullptr, 0},
  {"pair", "condim",         kMjXInt,       0, nullptr, 0},
  {"pair", "friction",       kMjXRealUpToN, 5, nullptr, 0},
  {"pair", "solref",         kMjXRealUpToN, 2, nullptr, 0},
  {"pair", "solreffriction", kMjXRealUpToN, 2, nullptr, 0},
  {"pair", "solimp",         kMjXRealUpToN, 5, nullptr, 0},
  {"pair", "margin",         kMjXReal,      0, nullptr, 0},
  {"pair", "gap",            kMjXReal,      0, nullptr, 0},

  //-------------------------------- <equality> in <default> --------------------------------------
  {"equality", "active", kMjXEnum,      0, bool_map, 2},
  {"equality", "solref", kMjXRealUpToN, 2, nullptr,  0},
  {"equality", "solimp", kMjXRealUpToN, 5, nullptr,  0},

  //-------------------------------- <tendon> (default + top-level tendon is a complex section) ---
  // The <tendon> row in MJCF[] under <default> holds only tendon defaults.
  // These attrs are shared by <spatial> and <fixed> when emitted from <tendon>.
  {"tendon", "group",              kMjXInt,       0, nullptr,    0},
  {"tendon", "limited",            kMjXEnum,      0, TFAuto_map, 3},
  {"tendon", "range",              kMjXRealN,     2, nullptr,    0},
  {"tendon", "solreflimit",        kMjXRealUpToN, 2, nullptr,    0},
  {"tendon", "solimplimit",        kMjXRealUpToN, 5, nullptr,    0},
  {"tendon", "solreffriction",     kMjXRealUpToN, 2, nullptr,    0},
  {"tendon", "solimpfriction",     kMjXRealUpToN, 5, nullptr,    0},
  {"tendon", "frictionloss",       kMjXReal,      0, nullptr,    0},
  {"tendon", "springlength",       kMjXRealUpToN, 2, nullptr,    0},
  {"tendon", "width",              kMjXReal,      0, nullptr,    0},
  {"tendon", "material",           kMjXString,    0, nullptr,    0},
  {"tendon", "margin",             kMjXReal,      0, nullptr,    0},
  {"tendon", "stiffness",          kMjXRealUpToN, 3, nullptr,    0},
  {"tendon", "damping",            kMjXRealUpToN, 3, nullptr,    0},
  {"tendon", "rgba",               kMjXRealN,     4, nullptr,    0},
  {"tendon", "user",               kMjXRealVec,   0, nullptr,    0},

  //-------------------------------- <spatial> (tendon section) -----------------------------------
  {"spatial", "name",              kMjXString,    0, nullptr,    0},
  {"spatial", "class",             kMjXString,    0, nullptr,    0},
  {"spatial", "group",             kMjXInt,       0, nullptr,    0},
  {"spatial", "limited",           kMjXEnum,      0, TFAuto_map, 3},
  {"spatial", "actuatorfrclimited",kMjXEnum,      0, TFAuto_map, 3},
  {"spatial", "range",             kMjXRealN,     2, nullptr,    0},
  {"spatial", "actuatorfrcrange",  kMjXRealN,     2, nullptr,    0},
  {"spatial", "solreflimit",       kMjXRealUpToN, 2, nullptr,    0},
  {"spatial", "solimplimit",       kMjXRealUpToN, 5, nullptr,    0},
  {"spatial", "solreffriction",    kMjXRealUpToN, 2, nullptr,    0},
  {"spatial", "solimpfriction",    kMjXRealUpToN, 5, nullptr,    0},
  {"spatial", "frictionloss",      kMjXReal,      0, nullptr,    0},
  {"spatial", "springlength",      kMjXRealUpToN, 2, nullptr,    0},
  {"spatial", "width",             kMjXReal,      0, nullptr,    0},
  {"spatial", "material",          kMjXString,    0, nullptr,    0},
  {"spatial", "margin",            kMjXReal,      0, nullptr,    0},
  {"spatial", "stiffness",         kMjXRealUpToN, 3, nullptr,    0},
  {"spatial", "damping",           kMjXRealUpToN, 3, nullptr,    0},
  {"spatial", "armature",          kMjXReal,      0, nullptr,    0},
  {"spatial", "rgba",              kMjXRealN,     4, nullptr,    0},
  {"spatial", "user",              kMjXRealVec,   0, nullptr,    0},

  //-------------------------------- <fixed> (tendon section) -------------------------------------
  {"fixed", "name",              kMjXString,    0, nullptr,    0},
  {"fixed", "class",             kMjXString,    0, nullptr,    0},
  {"fixed", "group",             kMjXInt,       0, nullptr,    0},
  {"fixed", "limited",           kMjXEnum,      0, TFAuto_map, 3},
  {"fixed", "actuatorfrclimited",kMjXEnum,      0, TFAuto_map, 3},
  {"fixed", "range",             kMjXRealN,     2, nullptr,    0},
  {"fixed", "actuatorfrcrange",  kMjXRealN,     2, nullptr,    0},
  {"fixed", "solreflimit",       kMjXRealUpToN, 2, nullptr,    0},
  {"fixed", "solimplimit",       kMjXRealUpToN, 5, nullptr,    0},
  {"fixed", "solreffriction",    kMjXRealUpToN, 2, nullptr,    0},
  {"fixed", "solimpfriction",    kMjXRealUpToN, 5, nullptr,    0},
  {"fixed", "frictionloss",      kMjXReal,      0, nullptr,    0},
  {"fixed", "springlength",      kMjXRealUpToN, 2, nullptr,    0},
  {"fixed", "margin",            kMjXReal,      0, nullptr,    0},
  {"fixed", "stiffness",         kMjXRealUpToN, 3, nullptr,    0},
  {"fixed", "damping",           kMjXRealUpToN, 3, nullptr,    0},
  {"fixed", "armature",          kMjXReal,      0, nullptr,    0},
  {"fixed", "user",              kMjXRealVec,   0, nullptr,    0},

  //-------------------------------- <spatial> children: site/geom/pulley -------------------------
  // Inside <spatial>, children are <site site=...>/<geom geom=... sidesite=...>/<pulley divisor=...>
  // However the child element name "site" collides with top-level <site>, and our lookup is by
  // (element, attr). We accept the collision: "site" attribute "site" is unique to the wrap,
  // but "geom" attribute "geom" is also unique.
  {"pulley", "divisor", kMjXReal,   0, nullptr, 0},

  //-------------------------------- <fixed><joint> (tendon wrap) ---------------------------------
  // The <joint> child of <fixed> has attrs {joint, coef}. Our top-level <joint> already uses
  // "name"/"class"/... so the string refs "joint" and real "coef" are distinct here; map them.
  {"joint", "coef",    kMjXReal,   0, nullptr, 0},

  //-------------------------------- actuator (shared attrs) --------------------------------------
  // Attributes common to all actuator shortcut types; listed per element so per-(element,attr)
  // lookups work.
#define ACT_COMMON(elem) \
  {elem, "name",          kMjXString,    0, nullptr,    0}, \
  {elem, "class",         kMjXString,    0, nullptr,    0}, \
  {elem, "group",         kMjXInt,       0, nullptr,    0}, \
  {elem, "nsample",       kMjXInt,       0, nullptr,    0}, \
  {elem, "interp",        kMjXEnum,      0, interp_map, 3}, \
  {elem, "delay",         kMjXReal,      0, nullptr,    0}, \
  {elem, "ctrllimited",   kMjXEnum,      0, TFAuto_map, 3}, \
  {elem, "forcelimited",  kMjXEnum,      0, TFAuto_map, 3}, \
  {elem, "ctrlrange",     kMjXRealN,     2, nullptr,    0}, \
  {elem, "forcerange",    kMjXRealN,     2, nullptr,    0}, \
  {elem, "lengthrange",   kMjXRealN,     2, nullptr,    0}, \
  {elem, "gear",          kMjXRealUpToN, 6, nullptr,    0}, \
  {elem, "damping",       kMjXRealUpToN, 3, nullptr,    0}, \
  {elem, "armature",      kMjXReal,      0, nullptr,    0}, \
  {elem, "cranklength",   kMjXReal,      0, nullptr,    0}, \
  {elem, "user",          kMjXRealVec,   0, nullptr,    0}, \
  {elem, "joint",         kMjXString,    0, nullptr,    0}, \
  {elem, "jointinparent", kMjXString,    0, nullptr,    0}, \
  {elem, "tendon",        kMjXString,    0, nullptr,    0}, \
  {elem, "slidersite",    kMjXString,    0, nullptr,    0}, \
  {elem, "cranksite",     kMjXString,    0, nullptr,    0}, \
  {elem, "site",          kMjXString,    0, nullptr,    0}, \
  {elem, "refsite",       kMjXString,    0, nullptr,    0}, \
  {elem, "body",          kMjXString,    0, nullptr,    0}

  ACT_COMMON("general"),
  {"general", "actlimited", kMjXEnum,      0, TFAuto_map, 3},
  {"general", "actrange",   kMjXRealN,     2, nullptr,    0},
  {"general", "actdim",     kMjXInt,       0, nullptr,    0},
  {"general", "dyntype",    kMjXEnum,      0, dyn_map,    7},
  {"general", "gaintype",   kMjXEnum,      0, gain_map,   5},
  {"general", "biastype",   kMjXEnum,      0, bias_map,   5},
  {"general", "dynprm",     kMjXRealUpToN, kNDYN,  nullptr, 0},
  {"general", "gainprm",    kMjXRealUpToN, kNGAIN, nullptr, 0},
  {"general", "biasprm",    kMjXRealUpToN, kNBIAS, nullptr, 0},
  {"general", "actearly",   kMjXEnum,      0, bool_map, 2},

  ACT_COMMON("motor"),

  ACT_COMMON("position"),
  {"position", "kp",           kMjXReal, 0, nullptr, 0},
  {"position", "kv",           kMjXReal, 0, nullptr, 0},
  {"position", "dampratio",    kMjXReal, 0, nullptr, 0},
  {"position", "timeconst",    kMjXReal, 0, nullptr, 0},
  {"position", "inheritrange", kMjXReal, 0, nullptr, 0},

  ACT_COMMON("velocity"),
  {"velocity", "kv", kMjXReal, 0, nullptr, 0},

  ACT_COMMON("intvelocity"),
  {"intvelocity", "kp",           kMjXReal,  0, nullptr, 0},
  {"intvelocity", "kv",           kMjXReal,  0, nullptr, 0},
  {"intvelocity", "dampratio",    kMjXReal,  0, nullptr, 0},
  {"intvelocity", "actrange",     kMjXRealN, 2, nullptr, 0},
  {"intvelocity", "inheritrange", kMjXReal,  0, nullptr, 0},

  ACT_COMMON("damper"),
  {"damper", "kv", kMjXReal, 0, nullptr, 0},

  ACT_COMMON("cylinder"),
  {"cylinder", "timeconst", kMjXReal,  0, nullptr, 0},
  {"cylinder", "area",      kMjXReal,  0, nullptr, 0},
  {"cylinder", "diameter",  kMjXReal,  0, nullptr, 0},
  {"cylinder", "bias",      kMjXRealN, 3, nullptr, 0},

  ACT_COMMON("muscle"),
  {"muscle", "timeconst", kMjXRealN, 2, nullptr, 0},
  {"muscle", "tausmooth", kMjXReal,  0, nullptr, 0},
  {"muscle", "range",     kMjXRealN, 2, nullptr, 0},
  {"muscle", "force",     kMjXReal,  0, nullptr, 0},
  {"muscle", "scale",     kMjXReal,  0, nullptr, 0},
  {"muscle", "lmin",      kMjXReal,  0, nullptr, 0},
  {"muscle", "lmax",      kMjXReal,  0, nullptr, 0},
  {"muscle", "vmax",      kMjXReal,  0, nullptr, 0},
  {"muscle", "fpmax",     kMjXReal,  0, nullptr, 0},
  {"muscle", "fvmax",     kMjXReal,  0, nullptr, 0},

  // adhesion doesn't use full ACT_COMMON (no joint/tendon/etc), so list manually.
  {"adhesion", "name",         kMjXString,    0, nullptr,    0},
  {"adhesion", "class",        kMjXString,    0, nullptr,    0},
  {"adhesion", "group",        kMjXInt,       0, nullptr,    0},
  {"adhesion", "nsample",      kMjXInt,       0, nullptr,    0},
  {"adhesion", "interp",       kMjXEnum,      0, interp_map, 3},
  {"adhesion", "delay",        kMjXReal,      0, nullptr,    0},
  {"adhesion", "forcelimited", kMjXEnum,      0, TFAuto_map, 3},
  {"adhesion", "ctrlrange",    kMjXRealN,     2, nullptr,    0},
  {"adhesion", "forcerange",   kMjXRealN,     2, nullptr,    0},
  {"adhesion", "user",         kMjXRealVec,   0, nullptr,    0},
  {"adhesion", "body",         kMjXString,    0, nullptr,    0},
  {"adhesion", "gain",         kMjXReal,      0, nullptr,    0},

  ACT_COMMON("dcmotor"),
  {"dcmotor", "motorconst", kMjXRealUpToN, 2, nullptr,         0},
  {"dcmotor", "resistance", kMjXReal,      0, nullptr,         0},
  {"dcmotor", "nominal",    kMjXRealUpToN, 3, nullptr,         0},
  {"dcmotor", "saturation", kMjXRealUpToN, 3, nullptr,         0},
  {"dcmotor", "inductance", kMjXRealUpToN, 2, nullptr,         0},
  {"dcmotor", "cogging",    kMjXRealUpToN, 3, nullptr,         0},
  {"dcmotor", "controller", kMjXRealUpToN, 6, nullptr,         0},
  {"dcmotor", "thermal",    kMjXRealUpToN, 6, nullptr,         0},
  {"dcmotor", "lugre",      kMjXRealUpToN, 5, nullptr,         0},
  {"dcmotor", "input",      kMjXEnum,      0, dcmotorinput_map, 3},

#undef ACT_COMMON

  //-------------------------------- <extension> ---------------------------------------------------
  {"plugin",   "plugin",   kMjXString, 0, nullptr, 0},
  {"plugin",   "instance", kMjXString, 0, nullptr, 0},
  {"plugin",   "name",     kMjXString, 0, nullptr, 0},
  {"plugin",   "class",    kMjXString, 0, nullptr, 0},
  {"instance", "name",     kMjXString, 0, nullptr, 0},
  {"config",   "key",      kMjXString, 0, nullptr, 0},
  {"config",   "value",    kMjXString, 0, nullptr, 0},

  //-------------------------------- <custom> ------------------------------------------------------
  {"numeric", "name", kMjXString,  0, nullptr, 0},
  {"numeric", "size", kMjXInt,     0, nullptr, 0},
  {"numeric", "data", kMjXRealVec, 0, nullptr, 0},

  {"text",    "name", kMjXString,  0, nullptr, 0},
  {"text",    "data", kMjXString,  0, nullptr, 0},

  {"tuple",   "name",    kMjXString, 0, nullptr, 0},
  {"element", "objtype", kMjXString, 0, nullptr, 0},
  {"element", "objname", kMjXString, 0, nullptr, 0},
  {"element", "prm",     kMjXReal,   0, nullptr, 0},

  //-------------------------------- <asset><hfield> -----------------------------------------------
  {"hfield", "name",         kMjXString,  0, nullptr, 0},
  {"hfield", "content_type", kMjXString,  0, nullptr, 0},
  {"hfield", "file",         kMjXString,  0, nullptr, 0},
  {"hfield", "nrow",         kMjXInt,     0, nullptr, 0},
  {"hfield", "ncol",         kMjXInt,     0, nullptr, 0},
  {"hfield", "size",         kMjXRealN,   4, nullptr, 0},
  {"hfield", "elevation",    kMjXRealVec, 0, nullptr, 0},

  //-------------------------------- <asset><skin> / <deformable><skin> ---------------------------
  {"skin", "name",     kMjXString,  0, nullptr,  0},
  {"skin", "file",     kMjXString,  0, nullptr,  0},
  {"skin", "material", kMjXString,  0, nullptr,  0},
  {"skin", "rgba",     kMjXRealN,   4, nullptr,  0},
  {"skin", "inflate",  kMjXReal,    0, nullptr,  0},
  {"skin", "vertex",   kMjXRealVec, 0, nullptr,  0},
  {"skin", "texcoord", kMjXRealVec, 0, nullptr,  0},
  {"skin", "face",     kMjXIntVec,  0, nullptr,  0},
  {"skin", "group",    kMjXInt,     0, nullptr,  0},
  // composite's <skin> sub-element
  {"skin", "texcoord_composite", kMjXEnum, 0, bool_map, 2},  // unused placeholder
  {"skin", "subgrid",  kMjXInt,     0, nullptr,  0},

  {"bone", "body",       kMjXString,  0, nullptr, 0},
  {"bone", "bindpos",    kMjXRealN,   3, nullptr, 0},
  {"bone", "bindquat",   kMjXRealN,   4, nullptr, 0},
  {"bone", "vertid",     kMjXIntVec,  0, nullptr, 0},
  {"bone", "vertweight", kMjXRealVec, 0, nullptr, 0},

  //-------------------------------- <asset><texture> ----------------------------------------------
  {"texture", "name",         kMjXString,  0, nullptr,         0},
  {"texture", "type",         kMjXEnum,    0, texture_map,     3},
  {"texture", "colorspace",   kMjXEnum,    0, colorspace_map,  3},
  {"texture", "content_type", kMjXString,  0, nullptr,         0},
  {"texture", "file",         kMjXString,  0, nullptr,         0},
  {"texture", "gridsize",     kMjXIntN,    2, nullptr,         0},
  {"texture", "gridlayout",   kMjXString,  0, nullptr,         0},
  {"texture", "fileright",    kMjXString,  0, nullptr,         0},
  {"texture", "fileleft",     kMjXString,  0, nullptr,         0},
  {"texture", "fileup",       kMjXString,  0, nullptr,         0},
  {"texture", "filedown",     kMjXString,  0, nullptr,         0},
  {"texture", "filefront",    kMjXString,  0, nullptr,         0},
  {"texture", "fileback",     kMjXString,  0, nullptr,         0},
  {"texture", "builtin",      kMjXEnum,    0, builtin_map,     4},
  {"texture", "rgb1",         kMjXRealN,   3, nullptr,         0},
  {"texture", "rgb2",         kMjXRealN,   3, nullptr,         0},
  {"texture", "mark",         kMjXEnum,    0, mark_map,        4},
  {"texture", "markrgb",      kMjXRealN,   3, nullptr,         0},
  {"texture", "random",       kMjXReal,    0, nullptr,         0},
  {"texture", "width",        kMjXInt,     0, nullptr,         0},
  {"texture", "height",       kMjXInt,     0, nullptr,         0},
  {"texture", "hflip",        kMjXEnum,    0, bool_map,        2},
  {"texture", "vflip",        kMjXEnum,    0, bool_map,        2},
  {"texture", "nchannel",     kMjXInt,     0, nullptr,         0},

  //-------------------------------- <asset><model> ------------------------------------------------
  {"model", "name",         kMjXString, 0, nullptr, 0},
  {"model", "file",         kMjXString, 0, nullptr, 0},
  {"model", "content_type", kMjXString, 0, nullptr, 0},

  //-------------------------------- <body> / <worldbody> -----------------------------------------
  {"body", "name",       kMjXString,  0, nullptr,       0},
  {"body", "childclass", kMjXString,  0, nullptr,       0},
  {"body", "pos",        kMjXRealN,   3, nullptr,       0},
  {"body", "quat",       kMjXRealN,   4, nullptr,       0},
  {"body", "mocap",      kMjXEnum,    0, bool_map,      2},
  {"body", "axisangle",  kMjXRealN,   4, nullptr,       0},
  {"body", "xyaxes",     kMjXRealN,   6, nullptr,       0},
  {"body", "zaxis",      kMjXRealN,   3, nullptr,       0},
  {"body", "euler",      kMjXRealN,   3, nullptr,       0},
  {"body", "gravcomp",   kMjXReal,    0, nullptr,       0},
  {"body", "sleep",      kMjXEnum,    0, bodysleep_map, 4},
  {"body", "user",       kMjXRealVec, 0, nullptr,       0},

  //-------------------------------- <body><inertial> ---------------------------------------------
  {"inertial", "pos",          kMjXRealN, 3, nullptr, 0},
  {"inertial", "quat",         kMjXRealN, 4, nullptr, 0},
  {"inertial", "mass",         kMjXReal,  0, nullptr, 0},
  {"inertial", "diaginertia",  kMjXRealN, 3, nullptr, 0},
  {"inertial", "axisangle",    kMjXRealN, 4, nullptr, 0},
  {"inertial", "xyaxes",       kMjXRealN, 6, nullptr, 0},
  {"inertial", "zaxis",        kMjXRealN, 3, nullptr, 0},
  {"inertial", "euler",        kMjXRealN, 3, nullptr, 0},
  {"inertial", "fullinertia",  kMjXRealN, 6, nullptr, 0},

  //-------------------------------- <body><frame> ------------------------------------------------
  {"frame", "name",       kMjXString, 0, nullptr, 0},
  {"frame", "childclass", kMjXString, 0, nullptr, 0},
  {"frame", "pos",        kMjXRealN,  3, nullptr, 0},
  {"frame", "quat",       kMjXRealN,  4, nullptr, 0},
  {"frame", "axisangle",  kMjXRealN,  4, nullptr, 0},
  {"frame", "xyaxes",     kMjXRealN,  6, nullptr, 0},
  {"frame", "zaxis",      kMjXRealN,  3, nullptr, 0},
  {"frame", "euler",      kMjXRealN,  3, nullptr, 0},

  //-------------------------------- <body><replicate> --------------------------------------------
  {"replicate", "count",      kMjXInt,    0, nullptr, 0},
  {"replicate", "offset",     kMjXRealN,  3, nullptr, 0},
  {"replicate", "euler",      kMjXRealN,  3, nullptr, 0},
  {"replicate", "sep",        kMjXString, 0, nullptr, 0},
  {"replicate", "childclass", kMjXString, 0, nullptr, 0},

  //-------------------------------- <body><attach> -----------------------------------------------
  {"attach", "model",  kMjXString, 0, nullptr, 0},
  {"attach", "body",   kMjXString, 0, nullptr, 0},
  {"attach", "prefix", kMjXString, 0, nullptr, 0},

  //-------------------------------- <composite> --------------------------------------------------
  {"composite", "prefix",  kMjXString,  0, nullptr,  0},
  {"composite", "type",    kMjXEnum,    0, comp_map, kCompTypesSz},
  {"composite", "count",   kMjXIntVec,  0, nullptr,  0},
  {"composite", "offset",  kMjXRealN,   3, nullptr,  0},
  {"composite", "vertex",  kMjXRealVec, 0, nullptr,  0},
  {"composite", "initial", kMjXString,  0, nullptr,  0},
  {"composite", "curve",   kMjXString,  0, nullptr,  0},
  {"composite", "size",    kMjXRealN,   3, nullptr,  0},
  {"composite", "quat",    kMjXRealN,   4, nullptr,  0},

  //-------------------------------- <flexcomp> ---------------------------------------------------
  {"flexcomp", "name",       kMjXString,  0, nullptr,   0},
  {"flexcomp", "type",       kMjXEnum,    0, fcomp_map, kFcompTypesSz},
  {"flexcomp", "group",      kMjXInt,     0, nullptr,   0},
  {"flexcomp", "dim",        kMjXInt,     0, nullptr,   0},
  {"flexcomp", "dof",        kMjXEnum,    0, fdof_map,  kFcompDofsSz},
  {"flexcomp", "count",      kMjXIntN,    3, nullptr,   0},
  {"flexcomp", "cellcount",  kMjXIntN,    3, nullptr,   0},
  {"flexcomp", "spacing",    kMjXRealN,   3, nullptr,   0},
  {"flexcomp", "radius",     kMjXReal,    0, nullptr,   0},
  {"flexcomp", "rigid",      kMjXEnum,    0, bool_map,  2},
  {"flexcomp", "mass",       kMjXReal,    0, nullptr,   0},
  {"flexcomp", "inertiabox", kMjXReal,    0, nullptr,   0},
  {"flexcomp", "scale",      kMjXRealN,   3, nullptr,   0},
  {"flexcomp", "file",       kMjXString,  0, nullptr,   0},
  {"flexcomp", "point",      kMjXRealVec, 0, nullptr,   0},
  {"flexcomp", "element",    kMjXIntVec,  0, nullptr,   0},
  {"flexcomp", "texcoord",   kMjXRealVec, 0, nullptr,   0},
  {"flexcomp", "material",   kMjXString,  0, nullptr,   0},
  {"flexcomp", "rgba",       kMjXRealN,   4, nullptr,   0},
  {"flexcomp", "flatskin",   kMjXEnum,    0, bool_map,  2},
  {"flexcomp", "pos",        kMjXRealN,   3, nullptr,   0},
  {"flexcomp", "quat",       kMjXRealN,   4, nullptr,   0},
  {"flexcomp", "axisangle",  kMjXRealN,   4, nullptr,   0},
  {"flexcomp", "xyaxes",     kMjXRealN,   6, nullptr,   0},
  {"flexcomp", "zaxis",      kMjXRealN,   3, nullptr,   0},
  {"flexcomp", "euler",      kMjXRealN,   3, nullptr,   0},
  {"flexcomp", "origin",     kMjXRealN,   3, nullptr,   0},

  //-------------------------------- <flexcomp>/<flex> children -----------------------------------
  // <edge> under <flex> has {stiffness, damping}; under <flexcomp> adds {equality, solref, solimp}.
  // We include the union.
  {"edge", "equality",  kMjXEnum,      0, flexeq_map, 4},
  {"edge", "solref",    kMjXRealUpToN, 2, nullptr,  0},
  {"edge", "solimp",    kMjXRealUpToN, 5, nullptr,  0},
  {"edge", "stiffness", kMjXReal,      0, nullptr,  0},
  {"edge", "damping",   kMjXReal,      0, nullptr,  0},

  {"elasticity", "young",     kMjXReal,      0, nullptr,         0},
  {"elasticity", "poisson",   kMjXReal,      0, nullptr,         0},
  {"elasticity", "damping",   kMjXReal,      0, nullptr,         0},
  {"elasticity", "thickness", kMjXReal,      0, nullptr,         0},
  {"elasticity", "elastic2d", kMjXEnum,      0, elastic2d_map,   5},

  {"contact", "contype",      kMjXInt,       0, nullptr,      0},
  {"contact", "conaffinity",  kMjXInt,       0, nullptr,      0},
  {"contact", "condim",       kMjXInt,       0, nullptr,      0},
  {"contact", "priority",     kMjXInt,       0, nullptr,      0},
  {"contact", "friction",     kMjXRealUpToN, 3, nullptr,      0},
  {"contact", "solmix",       kMjXReal,      0, nullptr,      0},
  {"contact", "solref",       kMjXRealUpToN, 2, nullptr,      0},
  {"contact", "solimp",       kMjXRealUpToN, 5, nullptr,      0},
  {"contact", "margin",       kMjXReal,      0, nullptr,      0},
  {"contact", "gap",          kMjXReal,      0, nullptr,      0},
  {"contact", "internal",     kMjXEnum,      0, bool_map,     2},
  {"contact", "selfcollide",  kMjXEnum,      0, flexself_map, 5},
  {"contact", "activelayers", kMjXInt,       0, nullptr,      0},
  {"contact", "passive",      kMjXEnum,      0, bool_map,     2},

  {"pin", "id",        kMjXIntVec,   0, nullptr, 0},
  {"pin", "range",     kMjXRealN,    2, nullptr, 0},
  {"pin", "grid",      kMjXIntVec,   0, nullptr, 0},
  {"pin", "gridrange", kMjXIntVec,   0, nullptr, 0},

  //-------------------------------- <composite> children -----------------------------------------
  // Most of composite's children re-use element names that already have entries.
  // "kind" is specific to composite's <joint>; treat as a composite-joint attr via shared key.

  //-------------------------------- <deformable><flex> -------------------------------------------
  {"flex", "name",         kMjXString,   0, nullptr,   0},
  {"flex", "group",        kMjXInt,      0, nullptr,   0},
  {"flex", "dim",          kMjXInt,      0, nullptr,   0},
  {"flex", "radius",       kMjXReal,     0, nullptr,   0},
  {"flex", "material",     kMjXString,   0, nullptr,   0},
  {"flex", "rgba",         kMjXRealN,    4, nullptr,   0},
  {"flex", "flatskin",     kMjXEnum,     0, bool_map,  2},
  {"flex", "body",         kMjXString,   0, nullptr,   0},
  {"flex", "vertex",       kMjXRealVec,  0, nullptr,   0},
  {"flex", "element",      kMjXIntVec,   0, nullptr,   0},
  {"flex", "texcoord",     kMjXRealVec,  0, nullptr,   0},
  {"flex", "elemtexcoord", kMjXIntVec,   0, nullptr,   0},
  {"flex", "node",         kMjXString,   0, nullptr,   0},
  {"flex", "cellcount",    kMjXIntN,     3, nullptr,   0},
  {"flex", "dof",          kMjXEnum,     0, fdof_map,  kFcompDofsSz},
  // equality-flex refs (used in <equality><flex>)
  {"flex", "class",        kMjXString,   0, nullptr,   0},
  {"flex", "flex",         kMjXString,   0, nullptr,   0},
  {"flex", "active",       kMjXEnum,     0, bool_map,  2},
  {"flex", "solref",       kMjXRealUpToN,2, nullptr,   0},
  {"flex", "solimp",       kMjXRealUpToN,5, nullptr,   0},

  //-------------------------------- <contact> ----------------------------------------------------
  {"exclude", "name",  kMjXString, 0, nullptr, 0},
  {"exclude", "body1", kMjXString, 0, nullptr, 0},
  {"exclude", "body2", kMjXString, 0, nullptr, 0},

  //-------------------------------- <equality> children ------------------------------------------
  {"connect", "name",     kMjXString,    0, nullptr,  0},
  {"connect", "class",    kMjXString,    0, nullptr,  0},
  {"connect", "body1",    kMjXString,    0, nullptr,  0},
  {"connect", "body2",    kMjXString,    0, nullptr,  0},
  {"connect", "anchor",   kMjXRealN,     3, nullptr,  0},
  {"connect", "site1",    kMjXString,    0, nullptr,  0},
  {"connect", "site2",    kMjXString,    0, nullptr,  0},
  {"connect", "active",   kMjXEnum,      0, bool_map, 2},
  {"connect", "solref",   kMjXRealUpToN, 2, nullptr,  0},
  {"connect", "solimp",   kMjXRealUpToN, 5, nullptr,  0},

  {"weld", "name",        kMjXString,    0, nullptr,  0},
  {"weld", "class",       kMjXString,    0, nullptr,  0},
  {"weld", "body1",       kMjXString,    0, nullptr,  0},
  {"weld", "body2",       kMjXString,    0, nullptr,  0},
  {"weld", "relpose",     kMjXRealN,     7, nullptr,  0},
  {"weld", "anchor",      kMjXRealN,     3, nullptr,  0},
  {"weld", "site1",       kMjXString,    0, nullptr,  0},
  {"weld", "site2",       kMjXString,    0, nullptr,  0},
  {"weld", "active",      kMjXEnum,      0, bool_map, 2},
  {"weld", "solref",      kMjXRealUpToN, 2, nullptr,  0},
  {"weld", "solimp",      kMjXRealUpToN, 5, nullptr,  0},
  {"weld", "torquescale", kMjXReal,      0, nullptr,  0},

  // <equality><joint> has a distinct set of attrs from top-level <joint>.
  // Since our lookup key is (element, attr), we add the ones not already covered
  // by top-level <joint> (joint1, joint2, polycoef).
  {"joint", "joint1",   kMjXString,    0, nullptr,  0},
  {"joint", "joint2",   kMjXString,    0, nullptr,  0},
  {"joint", "polycoef", kMjXRealUpToN, 5, nullptr,  0},
  {"joint", "active",   kMjXEnum,      0, bool_map, 2},
  {"joint", "solref",   kMjXRealUpToN, 2, nullptr,  0},
  {"joint", "solimp",   kMjXRealUpToN, 5, nullptr,  0},
  {"joint", "kind",     kMjXString,    0, nullptr,  0},  // composite joint kind
  {"joint", "joint",    kMjXString,    0, nullptr,  0},  // tendon wrap

  // <equality><tendon>
  {"tendon", "name",     kMjXString,    0, nullptr,  0},
  {"tendon", "class",    kMjXString,    0, nullptr,  0},
  {"tendon", "tendon1",  kMjXString,    0, nullptr,  0},
  {"tendon", "tendon2",  kMjXString,    0, nullptr,  0},
  {"tendon", "polycoef", kMjXRealUpToN, 5, nullptr,  0},

  // flexvert / flexstrain equality variants
  {"flexvert", "name",   kMjXString,    0, nullptr,  0},
  {"flexvert", "class",  kMjXString,    0, nullptr,  0},
  {"flexvert", "flex",   kMjXString,    0, nullptr,  0},
  {"flexvert", "active", kMjXEnum,      0, bool_map, 2},
  {"flexvert", "solref", kMjXRealUpToN, 2, nullptr,  0},
  {"flexvert", "solimp", kMjXRealUpToN, 5, nullptr,  0},

  {"flexstrain", "name",   kMjXString,    0, nullptr,  0},
  {"flexstrain", "class",  kMjXString,    0, nullptr,  0},
  {"flexstrain", "flex",   kMjXString,    0, nullptr,  0},
  {"flexstrain", "active", kMjXEnum,      0, bool_map, 2},
  {"flexstrain", "solref", kMjXRealUpToN, 2, nullptr,  0},
  {"flexstrain", "solimp", kMjXRealUpToN, 5, nullptr,  0},

  //-------------------------------- <tendon> wrap subelements ------------------------------------
  // <site site="..."/>, <geom geom="..." sidesite="..."/>
  {"site", "site",     kMjXString, 0, nullptr, 0},
  {"geom", "geom",     kMjXString, 0, nullptr, 0},
  {"geom", "sidesite", kMjXString, 0, nullptr, 0},

  //-------------------------------- <sensor> ------------------------------------------------------
  // shared sensor attrs
#define SENS_COMMON(elem) \
  {elem, "name",     kMjXString,    0, nullptr,    0}, \
  {elem, "nsample",  kMjXInt,       0, nullptr,    0}, \
  {elem, "interp",   kMjXEnum,      0, interp_map, 3}, \
  {elem, "delay",    kMjXReal,      0, nullptr,    0}, \
  {elem, "interval", kMjXRealUpToN, 2, nullptr,    0}, \
  {elem, "cutoff",   kMjXReal,      0, nullptr,    0}, \
  {elem, "noise",    kMjXReal,      0, nullptr,    0}, \
  {elem, "user",     kMjXRealVec,   0, nullptr,    0}

  SENS_COMMON("touch"),         {"touch",         "site",     kMjXString, 0, nullptr, 0},
  SENS_COMMON("accelerometer"), {"accelerometer", "site",     kMjXString, 0, nullptr, 0},
  SENS_COMMON("velocimeter"),   {"velocimeter",   "site",     kMjXString, 0, nullptr, 0},
  SENS_COMMON("gyro"),          {"gyro",          "site",     kMjXString, 0, nullptr, 0},
  SENS_COMMON("force"),         {"force",         "site",     kMjXString, 0, nullptr, 0},
  SENS_COMMON("torque"),        {"torque",        "site",     kMjXString, 0, nullptr, 0},
  SENS_COMMON("magnetometer"),  {"magnetometer",  "site",     kMjXString, 0, nullptr, 0},
  SENS_COMMON("camprojection"), {"camprojection", "site",     kMjXString, 0, nullptr, 0},
                                {"camprojection", "camera",   kMjXString, 0, nullptr, 0},
  SENS_COMMON("rangefinder"),   {"rangefinder",   "site",     kMjXString, 0, nullptr, 0},
                                {"rangefinder",   "camera",   kMjXString, 0, nullptr, 0},
                                {"rangefinder",   "data",     kMjXString, 0, nullptr, 0},
  SENS_COMMON("jointpos"),      {"jointpos",      "joint",    kMjXString, 0, nullptr, 0},
  SENS_COMMON("jointvel"),      {"jointvel",      "joint",    kMjXString, 0, nullptr, 0},
  SENS_COMMON("tendonpos"),     {"tendonpos",     "tendon",   kMjXString, 0, nullptr, 0},
  SENS_COMMON("tendonvel"),     {"tendonvel",     "tendon",   kMjXString, 0, nullptr, 0},
  SENS_COMMON("actuatorpos"),   {"actuatorpos",   "actuator", kMjXString, 0, nullptr, 0},
  SENS_COMMON("actuatorvel"),   {"actuatorvel",   "actuator", kMjXString, 0, nullptr, 0},
  SENS_COMMON("actuatorfrc"),   {"actuatorfrc",   "actuator", kMjXString, 0, nullptr, 0},
  SENS_COMMON("jointactuatorfrc"),  {"jointactuatorfrc",  "joint",  kMjXString, 0, nullptr, 0},
  SENS_COMMON("tendonactuatorfrc"), {"tendonactuatorfrc", "tendon", kMjXString, 0, nullptr, 0},
  SENS_COMMON("ballquat"),          {"ballquat",          "joint",  kMjXString, 0, nullptr, 0},
  SENS_COMMON("ballangvel"),        {"ballangvel",        "joint",  kMjXString, 0, nullptr, 0},
  SENS_COMMON("jointlimitpos"),     {"jointlimitpos",     "joint",  kMjXString, 0, nullptr, 0},
  SENS_COMMON("jointlimitvel"),     {"jointlimitvel",     "joint",  kMjXString, 0, nullptr, 0},
  SENS_COMMON("jointlimitfrc"),     {"jointlimitfrc",     "joint",  kMjXString, 0, nullptr, 0},
  SENS_COMMON("tendonlimitpos"),    {"tendonlimitpos",    "tendon", kMjXString, 0, nullptr, 0},
  SENS_COMMON("tendonlimitvel"),    {"tendonlimitvel",    "tendon", kMjXString, 0, nullptr, 0},
  SENS_COMMON("tendonlimitfrc"),    {"tendonlimitfrc",    "tendon", kMjXString, 0, nullptr, 0},

#define SENS_FRAME(elem) \
  SENS_COMMON(elem), \
  {elem, "objtype", kMjXString, 0, nullptr, 0}, \
  {elem, "objname", kMjXString, 0, nullptr, 0}, \
  {elem, "reftype", kMjXString, 0, nullptr, 0}, \
  {elem, "refname", kMjXString, 0, nullptr, 0}

  SENS_FRAME("framepos"),
  SENS_FRAME("framequat"),
  SENS_FRAME("framexaxis"),
  SENS_FRAME("frameyaxis"),
  SENS_FRAME("framezaxis"),
  SENS_FRAME("framelinvel"),
  SENS_FRAME("frameangvel"),
  SENS_COMMON("framelinacc"), {"framelinacc", "objtype", kMjXString, 0, nullptr, 0},
                              {"framelinacc", "objname", kMjXString, 0, nullptr, 0},
  SENS_COMMON("frameangacc"), {"frameangacc", "objtype", kMjXString, 0, nullptr, 0},
                              {"frameangacc", "objname", kMjXString, 0, nullptr, 0},

  SENS_COMMON("subtreecom"),    {"subtreecom",    "body", kMjXString, 0, nullptr, 0},
  SENS_COMMON("subtreelinvel"), {"subtreelinvel", "body", kMjXString, 0, nullptr, 0},
  SENS_COMMON("subtreeangmom"), {"subtreeangmom", "body", kMjXString, 0, nullptr, 0},

  SENS_COMMON("insidesite"),
  {"insidesite", "site",    kMjXString, 0, nullptr, 0},
  {"insidesite", "objtype", kMjXString, 0, nullptr, 0},
  {"insidesite", "objname", kMjXString, 0, nullptr, 0},

  // distance/normal/fromto share the same attrs
  SENS_COMMON("distance"),
  {"distance", "geom1", kMjXString, 0, nullptr, 0},
  {"distance", "geom2", kMjXString, 0, nullptr, 0},
  {"distance", "body1", kMjXString, 0, nullptr, 0},
  {"distance", "body2", kMjXString, 0, nullptr, 0},
  SENS_COMMON("normal"),
  {"normal", "geom1", kMjXString, 0, nullptr, 0},
  {"normal", "geom2", kMjXString, 0, nullptr, 0},
  {"normal", "body1", kMjXString, 0, nullptr, 0},
  {"normal", "body2", kMjXString, 0, nullptr, 0},
  SENS_COMMON("fromto"),
  {"fromto", "geom1", kMjXString, 0, nullptr, 0},
  {"fromto", "geom2", kMjXString, 0, nullptr, 0},
  {"fromto", "body1", kMjXString, 0, nullptr, 0},
  {"fromto", "body2", kMjXString, 0, nullptr, 0},

  // contact sensor
  {"contact", "name",     kMjXString, 0, nullptr,    0},
  {"contact", "geom1",    kMjXString, 0, nullptr,    0},
  {"contact", "geom2",    kMjXString, 0, nullptr,    0},
  {"contact", "body1",    kMjXString, 0, nullptr,    0},
  {"contact", "body2",    kMjXString, 0, nullptr,    0},
  {"contact", "subtree1", kMjXString, 0, nullptr,    0},
  {"contact", "subtree2", kMjXString, 0, nullptr,    0},
  {"contact", "site",     kMjXString, 0, nullptr,    0},
  {"contact", "num",      kMjXInt,    0, nullptr,    0},
  {"contact", "data",     kMjXString, 0, nullptr,    0},  // space-separated enum bitflags
  {"contact", "reduce",   kMjXEnum,   0, reduce_map, 4},
  {"contact", "nsample",  kMjXInt,    0, nullptr,    0},
  {"contact", "interp",   kMjXEnum,   0, interp_map, 3},
  {"contact", "delay",    kMjXReal,   0, nullptr,    0},
  {"contact", "interval", kMjXRealUpToN, 2, nullptr, 0},
  {"contact", "cutoff",   kMjXReal,   0, nullptr,    0},
  {"contact", "noise",    kMjXReal,   0, nullptr,    0},
  {"contact", "user",     kMjXRealVec,0, nullptr,    0},

  SENS_COMMON("e_potential"),
  SENS_COMMON("e_kinetic"),
  SENS_COMMON("clock"),

  {"tactile", "name",    kMjXString,  0, nullptr,    0},
  {"tactile", "geom",    kMjXString,  0, nullptr,    0},
  {"tactile", "mesh",    kMjXString,  0, nullptr,    0},
  {"tactile", "nsample", kMjXInt,     0, nullptr,    0},
  {"tactile", "interp",  kMjXEnum,    0, interp_map, 3},
  {"tactile", "delay",   kMjXReal,    0, nullptr,    0},
  {"tactile", "interval",kMjXRealUpToN, 2, nullptr,  0},
  {"tactile", "user",    kMjXRealVec, 0, nullptr,    0},

  {"user", "name",      kMjXString,  0, nullptr,       0},
  {"user", "objtype",   kMjXString,  0, nullptr,       0},
  {"user", "objname",   kMjXString,  0, nullptr,       0},
  {"user", "datatype",  kMjXEnum,    0, datatype_map,  4},
  {"user", "needstage", kMjXEnum,    0, stage_map,     4},
  {"user", "dim",       kMjXInt,     0, nullptr,       0},
  {"user", "cutoff",    kMjXReal,    0, nullptr,       0},
  {"user", "noise",     kMjXReal,    0, nullptr,       0},
  {"user", "user",      kMjXRealVec, 0, nullptr,       0},

#undef SENS_COMMON
#undef SENS_FRAME

  //-------------------------------- <keyframe> ---------------------------------------------------
  {"key", "name",  kMjXString,  0, nullptr, 0},
  {"key", "time",  kMjXReal,    0, nullptr, 0},
  {"key", "qpos",  kMjXRealVec, 0, nullptr, 0},
  {"key", "qvel",  kMjXRealVec, 0, nullptr, 0},
  {"key", "act",   kMjXRealVec, 0, nullptr, 0},
  {"key", "mpos",  kMjXRealVec, 0, nullptr, 0},
  {"key", "mquat", kMjXRealVec, 0, nullptr, 0},
  {"key", "ctrl",  kMjXRealVec, 0, nullptr, 0},
};
// clang-format on

const int kMjXAttrTableSize =
    sizeof(kMjXAttrTable) / sizeof(kMjXAttrTable[0]);


//---------------------------------- MJCF tree parser ----------------------------------------------

namespace {

struct Node {
  const char* name = nullptr;
  char type = '?';
  std::vector<const char*> attrs;
  std::vector<Node> children;
};

int ParseNode(int start, int nrow, Node& out) {
  out.name = MJCF[start][0];
  out.type = MJCF[start][1][0];
  const int nattr = static_cast<int>(MJCF[start].size()) - 2;
  for (int i = 0; i < nattr; i++) {
    out.attrs.push_back(MJCF[start][2 + i]);
  }

  int next = start + 1;
  if (next < nrow && MJCF[next][0][0] == '<') {
    next++;  // skip '<'
    while (next < nrow && MJCF[next][0][0] != '>') {
      Node child;
      next = ParseNode(next, nrow, child);
      out.children.push_back(std::move(child));
    }
    next++;  // skip '>'
  }
  return next;
}


//---------------------------------- XSD emit helpers ----------------------------------------------

void Indent(std::stringstream& out, int level) {
  for (int i = 0; i < level; i++) out << "  ";
}

const mjXAttr* Lookup(const char* element, const char* attr) {
  for (int i = 0; i < kMjXAttrTableSize; i++) {
    const mjXAttr& e = kMjXAttrTable[i];
    if (std::string(e.element) == element && std::string(e.attr) == attr) {
      return &e;
    }
  }
  return nullptr;
}

void CardinalityToOccurs(char type, const char** min_occurs,
                         const char** max_occurs) {
  switch (type) {
    case '!': *min_occurs = "1"; *max_occurs = "1";         break;
    case '?': *min_occurs = "0"; *max_occurs = "1";         break;
    case '*': *min_occurs = "0"; *max_occurs = "unbounded"; break;
    case 'R': *min_occurs = "0"; *max_occurs = "unbounded"; break;
    default:  *min_occurs = "0"; *max_occurs = "1";         break;
  }
}

std::string EnumTypeName(const char* element, const char* attr) {
  return std::string(element) + "_" + attr + "_enum";
}

std::string ListTypeName(mjXAttrKind kind, int size) {
  switch (kind) {
    case kMjXIntN:      return "list_int_" + std::to_string(size);
    case kMjXIntVec:    return "vec_int";
    case kMjXRealN:     return "list_real_" + std::to_string(size);
    case kMjXRealUpToN: return "uptolist_real_" + std::to_string(size);
    case kMjXRealVec:   return "vec_real";
    default:            return "unknown";
  }
}


//---------------------------------- Emit: enum / list types ---------------------------------------

void CollectTypes(
    const Node& node,
    std::set<std::string>& list_types,
    std::vector<std::pair<std::string, const mjXAttr*>>& enum_types,
    std::set<std::string>& enum_seen) {
  for (const char* a : node.attrs) {
    const mjXAttr* info = Lookup(node.name, a);
    if (!info) continue;

    if (info->kind == kMjXEnum) {
      std::string n = EnumTypeName(node.name, a);
      if (enum_seen.insert(n).second) {
        enum_types.emplace_back(n, info);
      }
    } else if (info->kind == kMjXIntN || info->kind == kMjXRealN ||
               info->kind == kMjXRealUpToN || info->kind == kMjXIntVec ||
               info->kind == kMjXRealVec) {
      list_types.insert(ListTypeName(info->kind, info->size));
    }
  }
  for (const Node& child : node.children) {
    CollectTypes(child, list_types, enum_types, enum_seen);
  }
}

void EmitListType(std::stringstream& out, const std::string& name) {
  const bool is_real = name.find("_real") != std::string::npos;
  const bool is_upto = name.rfind("uptolist_", 0) == 0;
  const bool is_vec  = name.rfind("vec_", 0) == 0;
  int size = 0;
  if (!is_vec) {
    auto pos = name.find_last_of('_');
    if (pos != std::string::npos) {
      try {
        size = std::stoi(name.substr(pos + 1));
      } catch (...) { size = 0; }
    }
  }

  Indent(out, 1);
  out << "<xs:simpleType name=\"" << name << "\">\n";
  Indent(out, 2);
  out << "<xs:restriction>\n";
  Indent(out, 3);
  out << "<xs:simpleType>\n";
  Indent(out, 4);
  out << "<xs:list itemType=\"" << (is_real ? "xs:double" : "xs:int") << "\"/>\n";
  Indent(out, 3);
  out << "</xs:simpleType>\n";
  if (is_vec) {
    // unconstrained length
  } else if (is_upto) {
    Indent(out, 3);
    out << "<xs:minLength value=\"1\"/>\n";
    Indent(out, 3);
    out << "<xs:maxLength value=\"" << size << "\"/>\n";
  } else {
    Indent(out, 3);
    out << "<xs:length value=\"" << size << "\"/>\n";
  }
  Indent(out, 2);
  out << "</xs:restriction>\n";
  Indent(out, 1);
  out << "</xs:simpleType>\n";
}

void EmitEnumType(std::stringstream& out, const std::string& name,
                  const mjXAttr& info) {
  Indent(out, 1);
  out << "<xs:simpleType name=\"" << name << "\">\n";
  Indent(out, 2);
  out << "<xs:restriction base=\"xs:string\">\n";
  for (int i = 0; i < info.map_size; i++) {
    // Some map arrays in xml_native_reader.cc are sized larger than the
    // number of initialized entries (e.g. elastic2d_map[5] with 4 entries);
    // skip nullptr sentinels defensively.
    if (!info.map[i].key) break;
    Indent(out, 3);
    out << "<xs:enumeration value=\"" << info.map[i].key << "\"/>\n";
  }
  Indent(out, 2);
  out << "</xs:restriction>\n";
  Indent(out, 1);
  out << "</xs:simpleType>\n";
}


//---------------------------------- Emit: elements ------------------------------------------------

void EmitAttribute(std::stringstream& out, int level, const char* element,
                   const char* attr) {
  const mjXAttr* info = Lookup(element, attr);
  Indent(out, level);
  out << "<xs:attribute name=\"" << attr << "\"";
  if (!info) {
    out << " type=\"xs:string\"/>  <!-- No type found -->\n";
    return;
  }
  switch (info->kind) {
    case kMjXString: out << " type=\"xs:string\"/>\n"; break;
    case kMjXInt:    out << " type=\"xs:int\"/>\n";    break;
    case kMjXReal:   out << " type=\"xs:double\"/>\n"; break;
    case kMjXEnum:
      out << " type=\"" << EnumTypeName(element, attr) << "\"/>\n";
      break;
    case kMjXIntN:
    case kMjXRealN:
    case kMjXRealUpToN:
    case kMjXIntVec:
    case kMjXRealVec:
      out << " type=\"" << ListTypeName(info->kind, info->size) << "\"/>\n";
      break;
  }
}

// Is this node recursive? ('R' cardinality)
bool IsRecursive(const Node& n) { return n.type == 'R'; }

// Emit the body of a complexType (choice + attributes) for a given element.
// Separated from EmitElement so it can be reused for named types that are
// referenced from multiple places.
void EmitComplexTypeBody(std::stringstream& out, const Node& node, int level,
                         const char* effective_name) {
  if (!node.children.empty() || IsRecursive(node) ||
      std::string(effective_name) == "body") {
    Indent(out, level);
    out << "<xs:choice minOccurs=\"0\" maxOccurs=\"unbounded\">\n";
    for (const Node& child : node.children) {
      extern void EmitElementInternal(std::stringstream&, const Node&, int,
                                       bool, const char*);
      EmitElementInternal(out, child, level + 1, /*top_level=*/false, nullptr);
    }
    // Recursive self-reference for 'R' elements.
    if (IsRecursive(node)) {
      Indent(out, level + 1);
      out << "<xs:element name=\"" << effective_name << "\" type=\""
          << effective_name << "_type\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
    }
    // Body special-cases: <body> can also contain <body>/<frame>/<replicate>
    // (per mjXSchema::NameMatch). Emit them when effective_name is body-family.
    // Each alias points at its own named complexType so its own attributes
    // (declared in kMjXAttrTable) are honored -- body_type would drop them.
    if (std::string(effective_name) == "body" ||
        std::string(effective_name) == "worldbody") {
      for (const char* alias : {"body", "frame", "replicate"}) {
        Indent(out, level + 1);
        out << "<xs:element name=\"" << alias << "\" type=\"" << alias << "_type\""
            << " minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
      }
    }
    // <include> is handled by the XML preprocessor (xml.cc::IncludeXML) and
    // can appear as a child of any element. Always allow it.
    Indent(out, level + 1);
    out << "<xs:element ref=\"include\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
    Indent(out, level);
    out << "</xs:choice>\n";
  }

  for (const char* a : node.attrs) {
    EmitAttribute(out, level, node.name, a);
  }
}

// Internal implementation (exposed via the `extern` inside EmitComplexTypeBody).
void EmitElementInternal(std::stringstream& out, const Node& node, int level,
                         bool top_level, const char* name_override) {
  const char* min_occurs = "0";
  const char* max_occurs = "1";
  CardinalityToOccurs(node.type, &min_occurs, &max_occurs);

  const char* element_name = name_override ? name_override : node.name;

  // Recursive elements use a named complexType so they can be self-referenced.
  const bool use_named_type = IsRecursive(node) ||
                              std::string(node.name) == "body";

  Indent(out, level);
  out << "<xs:element name=\"" << element_name << "\"";
  if (use_named_type) {
    out << " type=\"" << node.name << "_type\"";
  }
  if (!top_level) {
    if (std::string(min_occurs) != "1") out << " minOccurs=\"" << min_occurs << "\"";
    if (std::string(max_occurs) != "1") out << " maxOccurs=\"" << max_occurs << "\"";
  }
  if (use_named_type) {
    out << "/>\n";
    return;
  }
  out << ">\n";

  Indent(out, level + 1);
  out << "<xs:complexType>\n";
  EmitComplexTypeBody(out, node, level + 2, element_name);
  Indent(out, level + 1);
  out << "</xs:complexType>\n";
  Indent(out, level);
  out << "</xs:element>\n";
}

// Collect recursive (or body-family) nodes for named-type emission.
void CollectNamedTypes(const Node& node, std::vector<const Node*>& out) {
  if (IsRecursive(node) || std::string(node.name) == "body") {
    out.push_back(&node);
  }
  for (const Node& child : node.children) {
    CollectNamedTypes(child, out);
  }
}

void EmitNamedType(std::stringstream& out, const Node& node) {
  Indent(out, 1);
  out << "<xs:complexType name=\"" << node.name << "_type\">\n";
  EmitComplexTypeBody(out, node, 2, node.name);
  Indent(out, 1);
  out << "</xs:complexType>\n";
}

// Emit a named complexType that shares the <body> content model but has its
// own attributes (from kMjXAttrTable). Used for <frame> and <replicate>,
// which are aliases of <body> at the tree level but carry distinct attrs.
void EmitAliasType(std::stringstream& out, const Node& body_node,
                   const char* alias_name) {
  Indent(out, 1);
  out << "<xs:complexType name=\"" << alias_name << "_type\">\n";
  // Choice block mirrors body_type: same children + body-family aliases.
  Indent(out, 2);
  out << "<xs:choice minOccurs=\"0\" maxOccurs=\"unbounded\">\n";
  for (const Node& child : body_node.children) {
    EmitElementInternal(out, child, 3, /*top_level=*/false, nullptr);
  }
  // Body-family recursion: allow body/frame/replicate nested inside.
  for (const char* sub : {"body", "frame", "replicate"}) {
    Indent(out, 3);
    out << "<xs:element name=\"" << sub << "\" type=\"" << sub << "_type\""
        << " minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
  }
  Indent(out, 3);
  out << "<xs:element ref=\"include\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
  Indent(out, 2);
  out << "</xs:choice>\n";
  // Attributes come from the flat kMjXAttrTable entries for this element.
  for (int i = 0; i < kMjXAttrTableSize; i++) {
    const mjXAttr& row = kMjXAttrTable[i];
    if (std::string(row.element) == alias_name) {
      EmitAttribute(out, 2, alias_name, row.attr);
    }
  }
  Indent(out, 1);
  out << "</xs:complexType>\n";
}

}  // namespace


//---------------------------------- Entry point ---------------------------------------------------

void PrintSchemaXSD(std::stringstream& out) {
  Node root;
  ParseNode(0, nMJCF, root);

  std::set<std::string> list_types;
  std::vector<std::pair<std::string, const mjXAttr*>> enum_types;
  std::set<std::string> enum_seen;
  CollectTypes(root, list_types, enum_types, enum_seen);

  // Find recursive / body-family nodes so we can emit them as named types.
  std::vector<const Node*> named_types;
  CollectNamedTypes(root, named_types);

  out << "<?xml version=\"1.0\"?>\n";
  out << "<!-- Auto-generated from MuJoCo MJCF schema. DO NOT EDIT. -->\n";
  out << "<xs:schema xmlns:xs=\"http://www.w3.org/2001/XMLSchema\""
         " elementFormDefault=\"qualified\">\n";

  if (!list_types.empty()) {
    out << "\n  <!-- List types -->\n";
    for (const std::string& n : list_types) {
      EmitListType(out, n);
    }
  }

  if (!enum_types.empty()) {
    out << "\n  <!-- Enum types -->\n";
    for (const auto& [n, info] : enum_types) {
      EmitEnumType(out, n, *info);
    }
  }

  // Named complex types (for recursive elements like <body> and <default>).
  if (!named_types.empty()) {
    out << "\n  <!-- Recursive complex types -->\n";
    const Node* body_node = nullptr;
    for (const Node* n : named_types) {
      EmitNamedType(out, *n);
      if (std::string(n->name) == "body") body_node = n;
    }
    // <frame> and <replicate> are body-family aliases handled outside MJCF[]
    // (see mjXSchema::NameMatch). They share body's content model but each
    // has its own attribute set in kMjXAttrTable; emit dedicated types so
    // those attrs are actually applied when validating.
    if (body_node) {
      for (const char* alias : {"frame", "replicate"}) {
        EmitAliasType(out, *body_node, alias);
      }
    }
  }

  // Global <include> element (preprocessor directive, allowed as a child of
  // any element in the tree).
  out << "\n  <!-- Preprocessor include -->\n";
  Indent(out, 1);
  out << "<xs:element name=\"include\">\n";
  Indent(out, 2);
  out << "<xs:complexType>\n";
  Indent(out, 3);
  out << "<xs:attribute name=\"file\" type=\"xs:string\" use=\"required\"/>\n";
  Indent(out, 3);
  out << "<xs:attribute name=\"prefix\" type=\"xs:string\"/>\n";
  Indent(out, 2);
  out << "</xs:complexType>\n";
  Indent(out, 1);
  out << "</xs:element>\n";

  out << "\n  <!-- Root element -->\n";

  // Emit the <mujoco> root manually so we can rename top-level <body> →
  // <worldbody> (mjXSchema::NameMatch handles this at level==1).
  Indent(out, 1);
  out << "<xs:element name=\"" << root.name << "\">\n";
  Indent(out, 2);
  out << "<xs:complexType>\n";
  Indent(out, 3);
  out << "<xs:choice minOccurs=\"0\" maxOccurs=\"unbounded\">\n";
  for (const Node& child : root.children) {
    const char* override_name =
        (std::string(child.name) == "body") ? "worldbody" : nullptr;
    EmitElementInternal(out, child, 4, /*top_level=*/false, override_name);
  }
  Indent(out, 4);
  out << "<xs:element ref=\"include\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
  Indent(out, 3);
  out << "</xs:choice>\n";
  for (const char* a : root.attrs) {
    EmitAttribute(out, 3, root.name, a);
  }
  Indent(out, 2);
  out << "</xs:complexType>\n";
  Indent(out, 1);
  out << "</xs:element>\n";

  out << "</xs:schema>\n";
}

}  // namespace mujoco
