// Copyright 2025 DeepMind Technologies Limited
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

// NOLINTBEGIN(whitespace/line_length)
// NOLINTBEGIN(whitespace/semicolon)

#include "wasm/codegen/generated/bindings.h"

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>  // NOLINT

#include <array>
#include <cstdint>
#include <memory>
#include <optional>  // NOLINT
#include <string>    // NOLINT
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "engine/engine_util_errmem.h"
#include "wasm/unpack.h"

namespace mujoco::wasm {

using emscripten::enum_;
using emscripten::class_;
using emscripten::function;
using emscripten::val;
using emscripten::constant;
using emscripten::register_optional;
using emscripten::register_type;
using emscripten::register_vector;
using emscripten::return_value_policy::reference;
using emscripten::return_value_policy::take_ownership;

// ERROR HANDLER
void ThrowMujocoErrorToJS(const char* msg) {
  // Get a handle to the JS global Error constructor function, create a new
  // object instance and then throw the object as an exception using the
  // val::throw_() helper function.
  val(val::global("Error").new_(val("MuJoCo Error: " + std::string(msg))))
      .throw_();
}
__attribute__((constructor)) void InitMuJoCoErrorHandler() {
  mju_user_error = ThrowMujocoErrorToJS;
}

// CONSTANTS
template <size_t N>
val MakeValArray(const char* (&strings)[N]) {
  val result = val::array();
  for (int i = 0; i < N; i++) {
    result.call<void>("push", val(strings[i]));
  }
  return result;
}

template <size_t N, size_t M>
val MakeValArray3(const char* (&strings)[N][M]) {
  val result = val::array();
  for (int i = 0; i < N; i++) {
    val inner = val::array();
    for (int j = 0; j < M; j++) {
      inner.call<void>("push", val(strings[i][j]));
    }
    result.call<void>("push", inner);
  }
  return result;
}

val get_mjDISABLESTRING() { return MakeValArray(mjDISABLESTRING); }
val get_mjENABLESTRING() { return MakeValArray(mjENABLESTRING); }
val get_mjTIMERSTRING() { return MakeValArray(mjTIMERSTRING); }
val get_mjLABELSTRING() { return MakeValArray(mjLABELSTRING); }
val get_mjFRAMESTRING() { return MakeValArray(mjFRAMESTRING); }
val get_mjVISSTRING() { return MakeValArray3(mjVISSTRING); }
val get_mjRNDSTRING() { return MakeValArray3(mjRNDSTRING); }

EMSCRIPTEN_BINDINGS(constants) {
  // from mjmodel.h
  constant("mjPI", mjPI);
  constant("mjMAXVAL", mjMAXVAL);
  constant("mjMINMU", mjMINMU);
  constant("mjMINIMP", mjMINIMP);
  constant("mjMAXIMP", mjMAXIMP);
  constant("mjMAXCONPAIR", mjMAXCONPAIR);
  constant("mjNEQDATA", mjNEQDATA);
  constant("mjNDYN", mjNDYN);
  constant("mjNGAIN", mjNGAIN);
  constant("mjNBIAS", mjNBIAS);
  constant("mjNREF", mjNREF);
  constant("mjNIMP", mjNIMP);
  constant("mjNSOLVER", mjNSOLVER);

  // from mjvisualize.h
  constant("mjNGROUP", mjNGROUP);
  constant("mjMAXLIGHT", mjMAXLIGHT);
  constant("mjMAXOVERLAY", mjMAXOVERLAY);
  constant("mjMAXLINE", mjMAXLINE);
  constant("mjMAXLINEPNT", mjMAXLINEPNT);
  constant("mjMAXPLANEGRID", mjMAXPLANEGRID);

  // from mujoco.h
  constant("mjVERSION_HEADER", mjVERSION_HEADER);

  // from mjtnum.h
  constant("mjMINVAL", mjMINVAL);

  // emscripten::constant() is designed for simple, compile-time literal values
  // (like numbers or a single string literal), complex values need to be
  // bound as functions.
  emscripten::function("get_mjDISABLESTRING", &get_mjDISABLESTRING);
  emscripten::function("get_mjENABLESTRING", &get_mjENABLESTRING);
  emscripten::function("get_mjTIMERSTRING", &get_mjTIMERSTRING);
  emscripten::function("get_mjLABELSTRING", &get_mjLABELSTRING);
  emscripten::function("get_mjFRAMESTRING", &get_mjFRAMESTRING);
  emscripten::function("get_mjVISSTRING", &get_mjVISSTRING);
  emscripten::function("get_mjRNDSTRING", &get_mjRNDSTRING);
}

EMSCRIPTEN_BINDINGS(mujoco_enums) {
  enum_<mjtDisableBit>("mjtDisableBit")
    .value("mjDSBL_CONSTRAINT", mjDSBL_CONSTRAINT)
    .value("mjDSBL_EQUALITY", mjDSBL_EQUALITY)
    .value("mjDSBL_FRICTIONLOSS", mjDSBL_FRICTIONLOSS)
    .value("mjDSBL_LIMIT", mjDSBL_LIMIT)
    .value("mjDSBL_CONTACT", mjDSBL_CONTACT)
    .value("mjDSBL_SPRING", mjDSBL_SPRING)
    .value("mjDSBL_DAMPER", mjDSBL_DAMPER)
    .value("mjDSBL_GRAVITY", mjDSBL_GRAVITY)
    .value("mjDSBL_CLAMPCTRL", mjDSBL_CLAMPCTRL)
    .value("mjDSBL_WARMSTART", mjDSBL_WARMSTART)
    .value("mjDSBL_FILTERPARENT", mjDSBL_FILTERPARENT)
    .value("mjDSBL_ACTUATION", mjDSBL_ACTUATION)
    .value("mjDSBL_REFSAFE", mjDSBL_REFSAFE)
    .value("mjDSBL_SENSOR", mjDSBL_SENSOR)
    .value("mjDSBL_MIDPHASE", mjDSBL_MIDPHASE)
    .value("mjDSBL_EULERDAMP", mjDSBL_EULERDAMP)
    .value("mjDSBL_AUTORESET", mjDSBL_AUTORESET)
    .value("mjDSBL_NATIVECCD", mjDSBL_NATIVECCD)
    .value("mjDSBL_ISLAND", mjDSBL_ISLAND)
    .value("mjNDISABLE", mjNDISABLE);

  enum_<mjtEnableBit>("mjtEnableBit")
    .value("mjENBL_OVERRIDE", mjENBL_OVERRIDE)
    .value("mjENBL_ENERGY", mjENBL_ENERGY)
    .value("mjENBL_FWDINV", mjENBL_FWDINV)
    .value("mjENBL_INVDISCRETE", mjENBL_INVDISCRETE)
    .value("mjENBL_MULTICCD", mjENBL_MULTICCD)
    .value("mjNENABLE", mjNENABLE);

  enum_<mjtJoint>("mjtJoint")
    .value("mjJNT_FREE", mjJNT_FREE)
    .value("mjJNT_BALL", mjJNT_BALL)
    .value("mjJNT_SLIDE", mjJNT_SLIDE)
    .value("mjJNT_HINGE", mjJNT_HINGE);

  enum_<mjtGeom>("mjtGeom")
    .value("mjGEOM_PLANE", mjGEOM_PLANE)
    .value("mjGEOM_HFIELD", mjGEOM_HFIELD)
    .value("mjGEOM_SPHERE", mjGEOM_SPHERE)
    .value("mjGEOM_CAPSULE", mjGEOM_CAPSULE)
    .value("mjGEOM_ELLIPSOID", mjGEOM_ELLIPSOID)
    .value("mjGEOM_CYLINDER", mjGEOM_CYLINDER)
    .value("mjGEOM_BOX", mjGEOM_BOX)
    .value("mjGEOM_MESH", mjGEOM_MESH)
    .value("mjGEOM_SDF", mjGEOM_SDF)
    .value("mjNGEOMTYPES", mjNGEOMTYPES)
    .value("mjGEOM_ARROW", mjGEOM_ARROW)
    .value("mjGEOM_ARROW1", mjGEOM_ARROW1)
    .value("mjGEOM_ARROW2", mjGEOM_ARROW2)
    .value("mjGEOM_LINE", mjGEOM_LINE)
    .value("mjGEOM_LINEBOX", mjGEOM_LINEBOX)
    .value("mjGEOM_FLEX", mjGEOM_FLEX)
    .value("mjGEOM_SKIN", mjGEOM_SKIN)
    .value("mjGEOM_LABEL", mjGEOM_LABEL)
    .value("mjGEOM_TRIANGLE", mjGEOM_TRIANGLE)
    .value("mjGEOM_NONE", mjGEOM_NONE);

  enum_<mjtCamLight>("mjtCamLight")
    .value("mjCAMLIGHT_FIXED", mjCAMLIGHT_FIXED)
    .value("mjCAMLIGHT_TRACK", mjCAMLIGHT_TRACK)
    .value("mjCAMLIGHT_TRACKCOM", mjCAMLIGHT_TRACKCOM)
    .value("mjCAMLIGHT_TARGETBODY", mjCAMLIGHT_TARGETBODY)
    .value("mjCAMLIGHT_TARGETBODYCOM", mjCAMLIGHT_TARGETBODYCOM);

  enum_<mjtLightType>("mjtLightType")
    .value("mjLIGHT_SPOT", mjLIGHT_SPOT)
    .value("mjLIGHT_DIRECTIONAL", mjLIGHT_DIRECTIONAL)
    .value("mjLIGHT_POINT", mjLIGHT_POINT)
    .value("mjLIGHT_IMAGE", mjLIGHT_IMAGE);

  enum_<mjtTexture>("mjtTexture")
    .value("mjTEXTURE_2D", mjTEXTURE_2D)
    .value("mjTEXTURE_CUBE", mjTEXTURE_CUBE)
    .value("mjTEXTURE_SKYBOX", mjTEXTURE_SKYBOX);

  enum_<mjtTextureRole>("mjtTextureRole")
    .value("mjTEXROLE_USER", mjTEXROLE_USER)
    .value("mjTEXROLE_RGB", mjTEXROLE_RGB)
    .value("mjTEXROLE_OCCLUSION", mjTEXROLE_OCCLUSION)
    .value("mjTEXROLE_ROUGHNESS", mjTEXROLE_ROUGHNESS)
    .value("mjTEXROLE_METALLIC", mjTEXROLE_METALLIC)
    .value("mjTEXROLE_NORMAL", mjTEXROLE_NORMAL)
    .value("mjTEXROLE_OPACITY", mjTEXROLE_OPACITY)
    .value("mjTEXROLE_EMISSIVE", mjTEXROLE_EMISSIVE)
    .value("mjTEXROLE_RGBA", mjTEXROLE_RGBA)
    .value("mjTEXROLE_ORM", mjTEXROLE_ORM)
    .value("mjNTEXROLE", mjNTEXROLE);

  enum_<mjtColorSpace>("mjtColorSpace")
    .value("mjCOLORSPACE_AUTO", mjCOLORSPACE_AUTO)
    .value("mjCOLORSPACE_LINEAR", mjCOLORSPACE_LINEAR)
    .value("mjCOLORSPACE_SRGB", mjCOLORSPACE_SRGB);

  enum_<mjtIntegrator>("mjtIntegrator")
    .value("mjINT_EULER", mjINT_EULER)
    .value("mjINT_RK4", mjINT_RK4)
    .value("mjINT_IMPLICIT", mjINT_IMPLICIT)
    .value("mjINT_IMPLICITFAST", mjINT_IMPLICITFAST);

  enum_<mjtCone>("mjtCone")
    .value("mjCONE_PYRAMIDAL", mjCONE_PYRAMIDAL)
    .value("mjCONE_ELLIPTIC", mjCONE_ELLIPTIC);

  enum_<mjtJacobian>("mjtJacobian")
    .value("mjJAC_DENSE", mjJAC_DENSE)
    .value("mjJAC_SPARSE", mjJAC_SPARSE)
    .value("mjJAC_AUTO", mjJAC_AUTO);

  enum_<mjtSolver>("mjtSolver")
    .value("mjSOL_PGS", mjSOL_PGS)
    .value("mjSOL_CG", mjSOL_CG)
    .value("mjSOL_NEWTON", mjSOL_NEWTON);

  enum_<mjtEq>("mjtEq")
    .value("mjEQ_CONNECT", mjEQ_CONNECT)
    .value("mjEQ_WELD", mjEQ_WELD)
    .value("mjEQ_JOINT", mjEQ_JOINT)
    .value("mjEQ_TENDON", mjEQ_TENDON)
    .value("mjEQ_FLEX", mjEQ_FLEX)
    .value("mjEQ_DISTANCE", mjEQ_DISTANCE);

  enum_<mjtWrap>("mjtWrap")
    .value("mjWRAP_NONE", mjWRAP_NONE)
    .value("mjWRAP_JOINT", mjWRAP_JOINT)
    .value("mjWRAP_PULLEY", mjWRAP_PULLEY)
    .value("mjWRAP_SITE", mjWRAP_SITE)
    .value("mjWRAP_SPHERE", mjWRAP_SPHERE)
    .value("mjWRAP_CYLINDER", mjWRAP_CYLINDER);

  enum_<mjtTrn>("mjtTrn")
    .value("mjTRN_JOINT", mjTRN_JOINT)
    .value("mjTRN_JOINTINPARENT", mjTRN_JOINTINPARENT)
    .value("mjTRN_SLIDERCRANK", mjTRN_SLIDERCRANK)
    .value("mjTRN_TENDON", mjTRN_TENDON)
    .value("mjTRN_SITE", mjTRN_SITE)
    .value("mjTRN_BODY", mjTRN_BODY)
    .value("mjTRN_UNDEFINED", mjTRN_UNDEFINED);

  enum_<mjtDyn>("mjtDyn")
    .value("mjDYN_NONE", mjDYN_NONE)
    .value("mjDYN_INTEGRATOR", mjDYN_INTEGRATOR)
    .value("mjDYN_FILTER", mjDYN_FILTER)
    .value("mjDYN_FILTEREXACT", mjDYN_FILTEREXACT)
    .value("mjDYN_MUSCLE", mjDYN_MUSCLE)
    .value("mjDYN_USER", mjDYN_USER);

  enum_<mjtGain>("mjtGain")
    .value("mjGAIN_FIXED", mjGAIN_FIXED)
    .value("mjGAIN_AFFINE", mjGAIN_AFFINE)
    .value("mjGAIN_MUSCLE", mjGAIN_MUSCLE)
    .value("mjGAIN_USER", mjGAIN_USER);

  enum_<mjtBias>("mjtBias")
    .value("mjBIAS_NONE", mjBIAS_NONE)
    .value("mjBIAS_AFFINE", mjBIAS_AFFINE)
    .value("mjBIAS_MUSCLE", mjBIAS_MUSCLE)
    .value("mjBIAS_USER", mjBIAS_USER);

  enum_<mjtObj>("mjtObj")
    .value("mjOBJ_UNKNOWN", mjOBJ_UNKNOWN)
    .value("mjOBJ_BODY", mjOBJ_BODY)
    .value("mjOBJ_XBODY", mjOBJ_XBODY)
    .value("mjOBJ_JOINT", mjOBJ_JOINT)
    .value("mjOBJ_DOF", mjOBJ_DOF)
    .value("mjOBJ_GEOM", mjOBJ_GEOM)
    .value("mjOBJ_SITE", mjOBJ_SITE)
    .value("mjOBJ_CAMERA", mjOBJ_CAMERA)
    .value("mjOBJ_LIGHT", mjOBJ_LIGHT)
    .value("mjOBJ_FLEX", mjOBJ_FLEX)
    .value("mjOBJ_MESH", mjOBJ_MESH)
    .value("mjOBJ_SKIN", mjOBJ_SKIN)
    .value("mjOBJ_HFIELD", mjOBJ_HFIELD)
    .value("mjOBJ_TEXTURE", mjOBJ_TEXTURE)
    .value("mjOBJ_MATERIAL", mjOBJ_MATERIAL)
    .value("mjOBJ_PAIR", mjOBJ_PAIR)
    .value("mjOBJ_EXCLUDE", mjOBJ_EXCLUDE)
    .value("mjOBJ_EQUALITY", mjOBJ_EQUALITY)
    .value("mjOBJ_TENDON", mjOBJ_TENDON)
    .value("mjOBJ_ACTUATOR", mjOBJ_ACTUATOR)
    .value("mjOBJ_SENSOR", mjOBJ_SENSOR)
    .value("mjOBJ_NUMERIC", mjOBJ_NUMERIC)
    .value("mjOBJ_TEXT", mjOBJ_TEXT)
    .value("mjOBJ_TUPLE", mjOBJ_TUPLE)
    .value("mjOBJ_KEY", mjOBJ_KEY)
    .value("mjOBJ_PLUGIN", mjOBJ_PLUGIN)
    .value("mjNOBJECT", mjNOBJECT)
    .value("mjOBJ_FRAME", mjOBJ_FRAME)
    .value("mjOBJ_DEFAULT", mjOBJ_DEFAULT)
    .value("mjOBJ_MODEL", mjOBJ_MODEL);

  enum_<mjtSensor>("mjtSensor")
    .value("mjSENS_TOUCH", mjSENS_TOUCH)
    .value("mjSENS_ACCELEROMETER", mjSENS_ACCELEROMETER)
    .value("mjSENS_VELOCIMETER", mjSENS_VELOCIMETER)
    .value("mjSENS_GYRO", mjSENS_GYRO)
    .value("mjSENS_FORCE", mjSENS_FORCE)
    .value("mjSENS_TORQUE", mjSENS_TORQUE)
    .value("mjSENS_MAGNETOMETER", mjSENS_MAGNETOMETER)
    .value("mjSENS_RANGEFINDER", mjSENS_RANGEFINDER)
    .value("mjSENS_CAMPROJECTION", mjSENS_CAMPROJECTION)
    .value("mjSENS_JOINTPOS", mjSENS_JOINTPOS)
    .value("mjSENS_JOINTVEL", mjSENS_JOINTVEL)
    .value("mjSENS_TENDONPOS", mjSENS_TENDONPOS)
    .value("mjSENS_TENDONVEL", mjSENS_TENDONVEL)
    .value("mjSENS_ACTUATORPOS", mjSENS_ACTUATORPOS)
    .value("mjSENS_ACTUATORVEL", mjSENS_ACTUATORVEL)
    .value("mjSENS_ACTUATORFRC", mjSENS_ACTUATORFRC)
    .value("mjSENS_JOINTACTFRC", mjSENS_JOINTACTFRC)
    .value("mjSENS_TENDONACTFRC", mjSENS_TENDONACTFRC)
    .value("mjSENS_BALLQUAT", mjSENS_BALLQUAT)
    .value("mjSENS_BALLANGVEL", mjSENS_BALLANGVEL)
    .value("mjSENS_JOINTLIMITPOS", mjSENS_JOINTLIMITPOS)
    .value("mjSENS_JOINTLIMITVEL", mjSENS_JOINTLIMITVEL)
    .value("mjSENS_JOINTLIMITFRC", mjSENS_JOINTLIMITFRC)
    .value("mjSENS_TENDONLIMITPOS", mjSENS_TENDONLIMITPOS)
    .value("mjSENS_TENDONLIMITVEL", mjSENS_TENDONLIMITVEL)
    .value("mjSENS_TENDONLIMITFRC", mjSENS_TENDONLIMITFRC)
    .value("mjSENS_FRAMEPOS", mjSENS_FRAMEPOS)
    .value("mjSENS_FRAMEQUAT", mjSENS_FRAMEQUAT)
    .value("mjSENS_FRAMEXAXIS", mjSENS_FRAMEXAXIS)
    .value("mjSENS_FRAMEYAXIS", mjSENS_FRAMEYAXIS)
    .value("mjSENS_FRAMEZAXIS", mjSENS_FRAMEZAXIS)
    .value("mjSENS_FRAMELINVEL", mjSENS_FRAMELINVEL)
    .value("mjSENS_FRAMEANGVEL", mjSENS_FRAMEANGVEL)
    .value("mjSENS_FRAMELINACC", mjSENS_FRAMELINACC)
    .value("mjSENS_FRAMEANGACC", mjSENS_FRAMEANGACC)
    .value("mjSENS_SUBTREECOM", mjSENS_SUBTREECOM)
    .value("mjSENS_SUBTREELINVEL", mjSENS_SUBTREELINVEL)
    .value("mjSENS_SUBTREEANGMOM", mjSENS_SUBTREEANGMOM)
    .value("mjSENS_INSIDESITE", mjSENS_INSIDESITE)
    .value("mjSENS_GEOMDIST", mjSENS_GEOMDIST)
    .value("mjSENS_GEOMNORMAL", mjSENS_GEOMNORMAL)
    .value("mjSENS_GEOMFROMTO", mjSENS_GEOMFROMTO)
    .value("mjSENS_CONTACT", mjSENS_CONTACT)
    .value("mjSENS_E_POTENTIAL", mjSENS_E_POTENTIAL)
    .value("mjSENS_E_KINETIC", mjSENS_E_KINETIC)
    .value("mjSENS_CLOCK", mjSENS_CLOCK)
    .value("mjSENS_TACTILE", mjSENS_TACTILE)
    .value("mjSENS_PLUGIN", mjSENS_PLUGIN)
    .value("mjSENS_USER", mjSENS_USER);

  enum_<mjtStage>("mjtStage")
    .value("mjSTAGE_NONE", mjSTAGE_NONE)
    .value("mjSTAGE_POS", mjSTAGE_POS)
    .value("mjSTAGE_VEL", mjSTAGE_VEL)
    .value("mjSTAGE_ACC", mjSTAGE_ACC);

  enum_<mjtDataType>("mjtDataType")
    .value("mjDATATYPE_REAL", mjDATATYPE_REAL)
    .value("mjDATATYPE_POSITIVE", mjDATATYPE_POSITIVE)
    .value("mjDATATYPE_AXIS", mjDATATYPE_AXIS)
    .value("mjDATATYPE_QUATERNION", mjDATATYPE_QUATERNION);

  enum_<mjtConDataField>("mjtConDataField")
    .value("mjCONDATA_FOUND", mjCONDATA_FOUND)
    .value("mjCONDATA_FORCE", mjCONDATA_FORCE)
    .value("mjCONDATA_TORQUE", mjCONDATA_TORQUE)
    .value("mjCONDATA_DIST", mjCONDATA_DIST)
    .value("mjCONDATA_POS", mjCONDATA_POS)
    .value("mjCONDATA_NORMAL", mjCONDATA_NORMAL)
    .value("mjCONDATA_TANGENT", mjCONDATA_TANGENT)
    .value("mjNCONDATA", mjNCONDATA);

  enum_<mjtSameFrame>("mjtSameFrame")
    .value("mjSAMEFRAME_NONE", mjSAMEFRAME_NONE)
    .value("mjSAMEFRAME_BODY", mjSAMEFRAME_BODY)
    .value("mjSAMEFRAME_INERTIA", mjSAMEFRAME_INERTIA)
    .value("mjSAMEFRAME_BODYROT", mjSAMEFRAME_BODYROT)
    .value("mjSAMEFRAME_INERTIAROT", mjSAMEFRAME_INERTIAROT);

  enum_<mjtLRMode>("mjtLRMode")
    .value("mjLRMODE_NONE", mjLRMODE_NONE)
    .value("mjLRMODE_MUSCLE", mjLRMODE_MUSCLE)
    .value("mjLRMODE_MUSCLEUSER", mjLRMODE_MUSCLEUSER)
    .value("mjLRMODE_ALL", mjLRMODE_ALL);

  enum_<mjtFlexSelf>("mjtFlexSelf")
    .value("mjFLEXSELF_NONE", mjFLEXSELF_NONE)
    .value("mjFLEXSELF_NARROW", mjFLEXSELF_NARROW)
    .value("mjFLEXSELF_BVH", mjFLEXSELF_BVH)
    .value("mjFLEXSELF_SAP", mjFLEXSELF_SAP)
    .value("mjFLEXSELF_AUTO", mjFLEXSELF_AUTO);

  enum_<mjtSDFType>("mjtSDFType")
    .value("mjSDFTYPE_SINGLE", mjSDFTYPE_SINGLE)
    .value("mjSDFTYPE_INTERSECTION", mjSDFTYPE_INTERSECTION)
    .value("mjSDFTYPE_MIDSURFACE", mjSDFTYPE_MIDSURFACE)
    .value("mjSDFTYPE_COLLISION", mjSDFTYPE_COLLISION);

  enum_<mjtTaskStatus>("mjtTaskStatus")
    .value("mjTASK_NEW", mjTASK_NEW)
    .value("mjTASK_QUEUED", mjTASK_QUEUED)
    .value("mjTASK_COMPLETED", mjTASK_COMPLETED);

  enum_<mjtState>("mjtState")
    .value("mjSTATE_TIME", mjSTATE_TIME)
    .value("mjSTATE_QPOS", mjSTATE_QPOS)
    .value("mjSTATE_QVEL", mjSTATE_QVEL)
    .value("mjSTATE_ACT", mjSTATE_ACT)
    .value("mjSTATE_WARMSTART", mjSTATE_WARMSTART)
    .value("mjSTATE_CTRL", mjSTATE_CTRL)
    .value("mjSTATE_QFRC_APPLIED", mjSTATE_QFRC_APPLIED)
    .value("mjSTATE_XFRC_APPLIED", mjSTATE_XFRC_APPLIED)
    .value("mjSTATE_EQ_ACTIVE", mjSTATE_EQ_ACTIVE)
    .value("mjSTATE_MOCAP_POS", mjSTATE_MOCAP_POS)
    .value("mjSTATE_MOCAP_QUAT", mjSTATE_MOCAP_QUAT)
    .value("mjSTATE_USERDATA", mjSTATE_USERDATA)
    .value("mjSTATE_PLUGIN", mjSTATE_PLUGIN)
    .value("mjNSTATE", mjNSTATE)
    .value("mjSTATE_PHYSICS", mjSTATE_PHYSICS)
    .value("mjSTATE_FULLPHYSICS", mjSTATE_FULLPHYSICS)
    .value("mjSTATE_USER", mjSTATE_USER)
    .value("mjSTATE_INTEGRATION", mjSTATE_INTEGRATION);

  enum_<mjtConstraint>("mjtConstraint")
    .value("mjCNSTR_EQUALITY", mjCNSTR_EQUALITY)
    .value("mjCNSTR_FRICTION_DOF", mjCNSTR_FRICTION_DOF)
    .value("mjCNSTR_FRICTION_TENDON", mjCNSTR_FRICTION_TENDON)
    .value("mjCNSTR_LIMIT_JOINT", mjCNSTR_LIMIT_JOINT)
    .value("mjCNSTR_LIMIT_TENDON", mjCNSTR_LIMIT_TENDON)
    .value("mjCNSTR_CONTACT_FRICTIONLESS", mjCNSTR_CONTACT_FRICTIONLESS)
    .value("mjCNSTR_CONTACT_PYRAMIDAL", mjCNSTR_CONTACT_PYRAMIDAL)
    .value("mjCNSTR_CONTACT_ELLIPTIC", mjCNSTR_CONTACT_ELLIPTIC);

  enum_<mjtConstraintState>("mjtConstraintState")
    .value("mjCNSTRSTATE_SATISFIED", mjCNSTRSTATE_SATISFIED)
    .value("mjCNSTRSTATE_QUADRATIC", mjCNSTRSTATE_QUADRATIC)
    .value("mjCNSTRSTATE_LINEARNEG", mjCNSTRSTATE_LINEARNEG)
    .value("mjCNSTRSTATE_LINEARPOS", mjCNSTRSTATE_LINEARPOS)
    .value("mjCNSTRSTATE_CONE", mjCNSTRSTATE_CONE);

  enum_<mjtWarning>("mjtWarning")
    .value("mjWARN_INERTIA", mjWARN_INERTIA)
    .value("mjWARN_CONTACTFULL", mjWARN_CONTACTFULL)
    .value("mjWARN_CNSTRFULL", mjWARN_CNSTRFULL)
    .value("mjWARN_VGEOMFULL", mjWARN_VGEOMFULL)
    .value("mjWARN_BADQPOS", mjWARN_BADQPOS)
    .value("mjWARN_BADQVEL", mjWARN_BADQVEL)
    .value("mjWARN_BADQACC", mjWARN_BADQACC)
    .value("mjWARN_BADCTRL", mjWARN_BADCTRL)
    .value("mjNWARNING", mjNWARNING);

  enum_<mjtTimer>("mjtTimer")
    .value("mjTIMER_STEP", mjTIMER_STEP)
    .value("mjTIMER_FORWARD", mjTIMER_FORWARD)
    .value("mjTIMER_INVERSE", mjTIMER_INVERSE)
    .value("mjTIMER_POSITION", mjTIMER_POSITION)
    .value("mjTIMER_VELOCITY", mjTIMER_VELOCITY)
    .value("mjTIMER_ACTUATION", mjTIMER_ACTUATION)
    .value("mjTIMER_CONSTRAINT", mjTIMER_CONSTRAINT)
    .value("mjTIMER_ADVANCE", mjTIMER_ADVANCE)
    .value("mjTIMER_POS_KINEMATICS", mjTIMER_POS_KINEMATICS)
    .value("mjTIMER_POS_INERTIA", mjTIMER_POS_INERTIA)
    .value("mjTIMER_POS_COLLISION", mjTIMER_POS_COLLISION)
    .value("mjTIMER_POS_MAKE", mjTIMER_POS_MAKE)
    .value("mjTIMER_POS_PROJECT", mjTIMER_POS_PROJECT)
    .value("mjTIMER_COL_BROAD", mjTIMER_COL_BROAD)
    .value("mjTIMER_COL_NARROW", mjTIMER_COL_NARROW)
    .value("mjNTIMER", mjNTIMER);

  enum_<mjtGeomInertia>("mjtGeomInertia")
    .value("mjINERTIA_VOLUME", mjINERTIA_VOLUME)
    .value("mjINERTIA_SHELL", mjINERTIA_SHELL);

  enum_<mjtMeshInertia>("mjtMeshInertia")
    .value("mjMESH_INERTIA_CONVEX", mjMESH_INERTIA_CONVEX)
    .value("mjMESH_INERTIA_EXACT", mjMESH_INERTIA_EXACT)
    .value("mjMESH_INERTIA_LEGACY", mjMESH_INERTIA_LEGACY)
    .value("mjMESH_INERTIA_SHELL", mjMESH_INERTIA_SHELL);

  enum_<mjtMeshBuiltin>("mjtMeshBuiltin")
    .value("mjMESH_BUILTIN_NONE", mjMESH_BUILTIN_NONE)
    .value("mjMESH_BUILTIN_SPHERE", mjMESH_BUILTIN_SPHERE)
    .value("mjMESH_BUILTIN_HEMISPHERE", mjMESH_BUILTIN_HEMISPHERE)
    .value("mjMESH_BUILTIN_CONE", mjMESH_BUILTIN_CONE)
    .value("mjMESH_BUILTIN_SUPERSPHERE", mjMESH_BUILTIN_SUPERSPHERE)
    .value("mjMESH_BUILTIN_SUPERTORUS", mjMESH_BUILTIN_SUPERTORUS)
    .value("mjMESH_BUILTIN_WEDGE", mjMESH_BUILTIN_WEDGE)
    .value("mjMESH_BUILTIN_PLATE", mjMESH_BUILTIN_PLATE);

  enum_<mjtBuiltin>("mjtBuiltin")
    .value("mjBUILTIN_NONE", mjBUILTIN_NONE)
    .value("mjBUILTIN_GRADIENT", mjBUILTIN_GRADIENT)
    .value("mjBUILTIN_CHECKER", mjBUILTIN_CHECKER)
    .value("mjBUILTIN_FLAT", mjBUILTIN_FLAT);

  enum_<mjtMark>("mjtMark")
    .value("mjMARK_NONE", mjMARK_NONE)
    .value("mjMARK_EDGE", mjMARK_EDGE)
    .value("mjMARK_CROSS", mjMARK_CROSS)
    .value("mjMARK_RANDOM", mjMARK_RANDOM);

  enum_<mjtLimited>("mjtLimited")
    .value("mjLIMITED_FALSE", mjLIMITED_FALSE)
    .value("mjLIMITED_TRUE", mjLIMITED_TRUE)
    .value("mjLIMITED_AUTO", mjLIMITED_AUTO);

  enum_<mjtAlignFree>("mjtAlignFree")
    .value("mjALIGNFREE_FALSE", mjALIGNFREE_FALSE)
    .value("mjALIGNFREE_TRUE", mjALIGNFREE_TRUE)
    .value("mjALIGNFREE_AUTO", mjALIGNFREE_AUTO);

  enum_<mjtInertiaFromGeom>("mjtInertiaFromGeom")
    .value("mjINERTIAFROMGEOM_FALSE", mjINERTIAFROMGEOM_FALSE)
    .value("mjINERTIAFROMGEOM_TRUE", mjINERTIAFROMGEOM_TRUE)
    .value("mjINERTIAFROMGEOM_AUTO", mjINERTIAFROMGEOM_AUTO);

  enum_<mjtOrientation>("mjtOrientation")
    .value("mjORIENTATION_QUAT", mjORIENTATION_QUAT)
    .value("mjORIENTATION_AXISANGLE", mjORIENTATION_AXISANGLE)
    .value("mjORIENTATION_XYAXES", mjORIENTATION_XYAXES)
    .value("mjORIENTATION_ZAXIS", mjORIENTATION_ZAXIS)
    .value("mjORIENTATION_EULER", mjORIENTATION_EULER);

  enum_<mjtCatBit>("mjtCatBit")
    .value("mjCAT_STATIC", mjCAT_STATIC)
    .value("mjCAT_DYNAMIC", mjCAT_DYNAMIC)
    .value("mjCAT_DECOR", mjCAT_DECOR)
    .value("mjCAT_ALL", mjCAT_ALL);

  enum_<mjtMouse>("mjtMouse")
    .value("mjMOUSE_NONE", mjMOUSE_NONE)
    .value("mjMOUSE_ROTATE_V", mjMOUSE_ROTATE_V)
    .value("mjMOUSE_ROTATE_H", mjMOUSE_ROTATE_H)
    .value("mjMOUSE_MOVE_V", mjMOUSE_MOVE_V)
    .value("mjMOUSE_MOVE_H", mjMOUSE_MOVE_H)
    .value("mjMOUSE_ZOOM", mjMOUSE_ZOOM)
    .value("mjMOUSE_MOVE_V_REL", mjMOUSE_MOVE_V_REL)
    .value("mjMOUSE_MOVE_H_REL", mjMOUSE_MOVE_H_REL);

  enum_<mjtPertBit>("mjtPertBit")
    .value("mjPERT_TRANSLATE", mjPERT_TRANSLATE)
    .value("mjPERT_ROTATE", mjPERT_ROTATE);

  enum_<mjtCamera>("mjtCamera")
    .value("mjCAMERA_FREE", mjCAMERA_FREE)
    .value("mjCAMERA_TRACKING", mjCAMERA_TRACKING)
    .value("mjCAMERA_FIXED", mjCAMERA_FIXED)
    .value("mjCAMERA_USER", mjCAMERA_USER);

  enum_<mjtLabel>("mjtLabel")
    .value("mjLABEL_NONE", mjLABEL_NONE)
    .value("mjLABEL_BODY", mjLABEL_BODY)
    .value("mjLABEL_JOINT", mjLABEL_JOINT)
    .value("mjLABEL_GEOM", mjLABEL_GEOM)
    .value("mjLABEL_SITE", mjLABEL_SITE)
    .value("mjLABEL_CAMERA", mjLABEL_CAMERA)
    .value("mjLABEL_LIGHT", mjLABEL_LIGHT)
    .value("mjLABEL_TENDON", mjLABEL_TENDON)
    .value("mjLABEL_ACTUATOR", mjLABEL_ACTUATOR)
    .value("mjLABEL_CONSTRAINT", mjLABEL_CONSTRAINT)
    .value("mjLABEL_FLEX", mjLABEL_FLEX)
    .value("mjLABEL_SKIN", mjLABEL_SKIN)
    .value("mjLABEL_SELECTION", mjLABEL_SELECTION)
    .value("mjLABEL_SELPNT", mjLABEL_SELPNT)
    .value("mjLABEL_CONTACTPOINT", mjLABEL_CONTACTPOINT)
    .value("mjLABEL_CONTACTFORCE", mjLABEL_CONTACTFORCE)
    .value("mjLABEL_ISLAND", mjLABEL_ISLAND)
    .value("mjNLABEL", mjNLABEL);

  enum_<mjtFrame>("mjtFrame")
    .value("mjFRAME_NONE", mjFRAME_NONE)
    .value("mjFRAME_BODY", mjFRAME_BODY)
    .value("mjFRAME_GEOM", mjFRAME_GEOM)
    .value("mjFRAME_SITE", mjFRAME_SITE)
    .value("mjFRAME_CAMERA", mjFRAME_CAMERA)
    .value("mjFRAME_LIGHT", mjFRAME_LIGHT)
    .value("mjFRAME_CONTACT", mjFRAME_CONTACT)
    .value("mjFRAME_WORLD", mjFRAME_WORLD)
    .value("mjNFRAME", mjNFRAME);

  enum_<mjtVisFlag>("mjtVisFlag")
    .value("mjVIS_CONVEXHULL", mjVIS_CONVEXHULL)
    .value("mjVIS_TEXTURE", mjVIS_TEXTURE)
    .value("mjVIS_JOINT", mjVIS_JOINT)
    .value("mjVIS_CAMERA", mjVIS_CAMERA)
    .value("mjVIS_ACTUATOR", mjVIS_ACTUATOR)
    .value("mjVIS_ACTIVATION", mjVIS_ACTIVATION)
    .value("mjVIS_LIGHT", mjVIS_LIGHT)
    .value("mjVIS_TENDON", mjVIS_TENDON)
    .value("mjVIS_RANGEFINDER", mjVIS_RANGEFINDER)
    .value("mjVIS_CONSTRAINT", mjVIS_CONSTRAINT)
    .value("mjVIS_INERTIA", mjVIS_INERTIA)
    .value("mjVIS_SCLINERTIA", mjVIS_SCLINERTIA)
    .value("mjVIS_PERTFORCE", mjVIS_PERTFORCE)
    .value("mjVIS_PERTOBJ", mjVIS_PERTOBJ)
    .value("mjVIS_CONTACTPOINT", mjVIS_CONTACTPOINT)
    .value("mjVIS_ISLAND", mjVIS_ISLAND)
    .value("mjVIS_CONTACTFORCE", mjVIS_CONTACTFORCE)
    .value("mjVIS_CONTACTSPLIT", mjVIS_CONTACTSPLIT)
    .value("mjVIS_TRANSPARENT", mjVIS_TRANSPARENT)
    .value("mjVIS_AUTOCONNECT", mjVIS_AUTOCONNECT)
    .value("mjVIS_COM", mjVIS_COM)
    .value("mjVIS_SELECT", mjVIS_SELECT)
    .value("mjVIS_STATIC", mjVIS_STATIC)
    .value("mjVIS_SKIN", mjVIS_SKIN)
    .value("mjVIS_FLEXVERT", mjVIS_FLEXVERT)
    .value("mjVIS_FLEXEDGE", mjVIS_FLEXEDGE)
    .value("mjVIS_FLEXFACE", mjVIS_FLEXFACE)
    .value("mjVIS_FLEXSKIN", mjVIS_FLEXSKIN)
    .value("mjVIS_BODYBVH", mjVIS_BODYBVH)
    .value("mjVIS_MESHBVH", mjVIS_MESHBVH)
    .value("mjVIS_SDFITER", mjVIS_SDFITER)
    .value("mjNVISFLAG", mjNVISFLAG);

  enum_<mjtRndFlag>("mjtRndFlag")
    .value("mjRND_SHADOW", mjRND_SHADOW)
    .value("mjRND_WIREFRAME", mjRND_WIREFRAME)
    .value("mjRND_REFLECTION", mjRND_REFLECTION)
    .value("mjRND_ADDITIVE", mjRND_ADDITIVE)
    .value("mjRND_SKYBOX", mjRND_SKYBOX)
    .value("mjRND_FOG", mjRND_FOG)
    .value("mjRND_HAZE", mjRND_HAZE)
    .value("mjRND_SEGMENT", mjRND_SEGMENT)
    .value("mjRND_IDCOLOR", mjRND_IDCOLOR)
    .value("mjRND_CULL_FACE", mjRND_CULL_FACE)
    .value("mjNRNDFLAG", mjNRNDFLAG);

  enum_<mjtStereo>("mjtStereo")
    .value("mjSTEREO_NONE", mjSTEREO_NONE)
    .value("mjSTEREO_QUADBUFFERED", mjSTEREO_QUADBUFFERED)
    .value("mjSTEREO_SIDEBYSIDE", mjSTEREO_SIDEBYSIDE);

  enum_<mjtPluginCapabilityBit>("mjtPluginCapabilityBit")
    .value("mjPLUGIN_ACTUATOR", mjPLUGIN_ACTUATOR)
    .value("mjPLUGIN_SENSOR", mjPLUGIN_SENSOR)
    .value("mjPLUGIN_PASSIVE", mjPLUGIN_PASSIVE)
    .value("mjPLUGIN_SDF", mjPLUGIN_SDF);

  enum_<mjtGridPos>("mjtGridPos")
    .value("mjGRID_TOPLEFT", mjGRID_TOPLEFT)
    .value("mjGRID_TOPRIGHT", mjGRID_TOPRIGHT)
    .value("mjGRID_BOTTOMLEFT", mjGRID_BOTTOMLEFT)
    .value("mjGRID_BOTTOMRIGHT", mjGRID_BOTTOMRIGHT)
    .value("mjGRID_TOP", mjGRID_TOP)
    .value("mjGRID_BOTTOM", mjGRID_BOTTOM)
    .value("mjGRID_LEFT", mjGRID_LEFT)
    .value("mjGRID_RIGHT", mjGRID_RIGHT);

  enum_<mjtFramebuffer>("mjtFramebuffer")
    .value("mjFB_WINDOW", mjFB_WINDOW)
    .value("mjFB_OFFSCREEN", mjFB_OFFSCREEN);

  enum_<mjtDepthMap>("mjtDepthMap")
    .value("mjDEPTH_ZERONEAR", mjDEPTH_ZERONEAR)
    .value("mjDEPTH_ZEROFAR", mjDEPTH_ZEROFAR);

  enum_<mjtFontScale>("mjtFontScale")
    .value("mjFONTSCALE_50", mjFONTSCALE_50)
    .value("mjFONTSCALE_100", mjFONTSCALE_100)
    .value("mjFONTSCALE_150", mjFONTSCALE_150)
    .value("mjFONTSCALE_200", mjFONTSCALE_200)
    .value("mjFONTSCALE_250", mjFONTSCALE_250)
    .value("mjFONTSCALE_300", mjFONTSCALE_300);

  enum_<mjtFont>("mjtFont")
    .value("mjFONT_NORMAL", mjFONT_NORMAL)
    .value("mjFONT_SHADOW", mjFONT_SHADOW)
    .value("mjFONT_BIG", mjFONT_BIG);

  enum_<mjtButton>("mjtButton")
    .value("mjBUTTON_NONE", mjBUTTON_NONE)
    .value("mjBUTTON_LEFT", mjBUTTON_LEFT)
    .value("mjBUTTON_RIGHT", mjBUTTON_RIGHT)
    .value("mjBUTTON_MIDDLE", mjBUTTON_MIDDLE);

  enum_<mjtEvent>("mjtEvent")
    .value("mjEVENT_NONE", mjEVENT_NONE)
    .value("mjEVENT_MOVE", mjEVENT_MOVE)
    .value("mjEVENT_PRESS", mjEVENT_PRESS)
    .value("mjEVENT_RELEASE", mjEVENT_RELEASE)
    .value("mjEVENT_SCROLL", mjEVENT_SCROLL)
    .value("mjEVENT_KEY", mjEVENT_KEY)
    .value("mjEVENT_RESIZE", mjEVENT_RESIZE)
    .value("mjEVENT_REDRAW", mjEVENT_REDRAW)
    .value("mjEVENT_FILESDROP", mjEVENT_FILESDROP);

  enum_<mjtItem>("mjtItem")
    .value("mjITEM_END", mjITEM_END)
    .value("mjITEM_SECTION", mjITEM_SECTION)
    .value("mjITEM_SEPARATOR", mjITEM_SEPARATOR)
    .value("mjITEM_STATIC", mjITEM_STATIC)
    .value("mjITEM_BUTTON", mjITEM_BUTTON)
    .value("mjITEM_CHECKINT", mjITEM_CHECKINT)
    .value("mjITEM_CHECKBYTE", mjITEM_CHECKBYTE)
    .value("mjITEM_RADIO", mjITEM_RADIO)
    .value("mjITEM_RADIOLINE", mjITEM_RADIOLINE)
    .value("mjITEM_SELECT", mjITEM_SELECT)
    .value("mjITEM_SLIDERINT", mjITEM_SLIDERINT)
    .value("mjITEM_SLIDERNUM", mjITEM_SLIDERNUM)
    .value("mjITEM_EDITINT", mjITEM_EDITINT)
    .value("mjITEM_EDITNUM", mjITEM_EDITNUM)
    .value("mjITEM_EDITFLOAT", mjITEM_EDITFLOAT)
    .value("mjITEM_EDITTXT", mjITEM_EDITTXT)
    .value("mjNITEM", mjNITEM);

  enum_<mjtSection>("mjtSection")
    .value("mjSECT_CLOSED", mjSECT_CLOSED)
    .value("mjSECT_OPEN", mjSECT_OPEN)
    .value("mjSECT_FIXED", mjSECT_FIXED);
}

// STRUCTS
// =============== MjLROpt =============== //
MjLROpt::MjLROpt(mjLROpt *ptr) : ptr_(ptr) {}
MjLROpt::MjLROpt() : ptr_(new mjLROpt) {
  owned_ = true;
  mj_defaultLROpt(ptr_);
}
MjLROpt::MjLROpt(const MjLROpt &other) : MjLROpt() {
  *ptr_ = *other.get();
}
MjLROpt& MjLROpt::operator=(const MjLROpt &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjLROpt::~MjLROpt() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjLROpt> MjLROpt::copy() {
  return std::make_unique<MjLROpt>(*this);
}

// =============== MjOption =============== //
MjOption::MjOption(mjOption *ptr) : ptr_(ptr) {}
MjOption::MjOption() : ptr_(new mjOption) {
  owned_ = true;
  mj_defaultOption(ptr_);
}
MjOption::MjOption(const MjOption &other) : MjOption() {
  *ptr_ = *other.get();
}
MjOption& MjOption::operator=(const MjOption &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjOption::~MjOption() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjOption> MjOption::copy() {
  return std::make_unique<MjOption>(*this);
}

// =============== MjStatistic =============== //
MjStatistic::MjStatistic(mjStatistic *ptr) : ptr_(ptr) {}
MjStatistic::MjStatistic() : ptr_(new mjStatistic) {
  owned_ = true;
}
MjStatistic::MjStatistic(const MjStatistic &other) : MjStatistic() {
  *ptr_ = *other.get();
}
MjStatistic& MjStatistic::operator=(const MjStatistic &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjStatistic::~MjStatistic() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjStatistic> MjStatistic::copy() {
  return std::make_unique<MjStatistic>(*this);
}

// =============== MjVisual... =============== //
MjVisualGlobal::MjVisualGlobal(mjVisualGlobal *ptr) : ptr_(ptr) {}
MjVisualGlobal::MjVisualGlobal() : ptr_(new mjVisualGlobal) {
  owned_ = true;
}
MjVisualGlobal::MjVisualGlobal(const MjVisualGlobal &other) : MjVisualGlobal() {
  *ptr_ = *other.get();
}
MjVisualGlobal& MjVisualGlobal::operator=(const MjVisualGlobal &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjVisualGlobal::~MjVisualGlobal() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjVisualGlobal> MjVisualGlobal::copy() {
  return std::make_unique<MjVisualGlobal>(*this);
}

MjVisualQuality::MjVisualQuality(mjVisualQuality *ptr) : ptr_(ptr) {}
MjVisualQuality::MjVisualQuality() : ptr_(new mjVisualQuality) {
  owned_ = true;
}
MjVisualQuality::MjVisualQuality(const MjVisualQuality &other) : MjVisualQuality() {
  *ptr_ = *other.get();
}
MjVisualQuality& MjVisualQuality::operator=(const MjVisualQuality &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjVisualQuality::~MjVisualQuality() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjVisualQuality> MjVisualQuality::copy() {
  return std::make_unique<MjVisualQuality>(*this);
}

MjVisualHeadlight::MjVisualHeadlight(mjVisualHeadlight *ptr) : ptr_(ptr) {}
MjVisualHeadlight::MjVisualHeadlight() : ptr_(new mjVisualHeadlight) {
  owned_ = true;
}
MjVisualHeadlight::MjVisualHeadlight(const MjVisualHeadlight &other) : MjVisualHeadlight() {
  *ptr_ = *other.get();
}
MjVisualHeadlight& MjVisualHeadlight::operator=(const MjVisualHeadlight &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjVisualHeadlight::~MjVisualHeadlight() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjVisualHeadlight> MjVisualHeadlight::copy() {
  return std::make_unique<MjVisualHeadlight>(*this);
}

MjVisualMap::MjVisualMap(mjVisualMap *ptr) : ptr_(ptr) {}
MjVisualMap::MjVisualMap() : ptr_(new mjVisualMap) {
  owned_ = true;
}
MjVisualMap::MjVisualMap(const MjVisualMap &other) : MjVisualMap() {
  *ptr_ = *other.get();
}
MjVisualMap& MjVisualMap::operator=(const MjVisualMap &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjVisualMap::~MjVisualMap() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjVisualMap> MjVisualMap::copy() {
  return std::make_unique<MjVisualMap>(*this);
}

MjVisualScale::MjVisualScale(mjVisualScale *ptr) : ptr_(ptr) {}
MjVisualScale::MjVisualScale() : ptr_(new mjVisualScale) {
  owned_ = true;
}
MjVisualScale::MjVisualScale(const MjVisualScale &other) : MjVisualScale() {
  *ptr_ = *other.get();
}
MjVisualScale& MjVisualScale::operator=(const MjVisualScale &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjVisualScale::~MjVisualScale() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjVisualScale> MjVisualScale::copy() {
  return std::make_unique<MjVisualScale>(*this);
}

MjVisualRgba::MjVisualRgba(mjVisualRgba *ptr) : ptr_(ptr) {}
MjVisualRgba::MjVisualRgba() : ptr_(new mjVisualRgba) {
  owned_ = true;
}
MjVisualRgba::MjVisualRgba(const MjVisualRgba &other) : MjVisualRgba() {
  *ptr_ = *other.get();
}
MjVisualRgba& MjVisualRgba::operator=(const MjVisualRgba &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjVisualRgba::~MjVisualRgba() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjVisualRgba> MjVisualRgba::copy() {
  return std::make_unique<MjVisualRgba>(*this);
}

MjVisual::MjVisual(mjVisual *ptr) : ptr_(ptr), global(&ptr_->global), quality(&ptr_->quality), headlight(&ptr_->headlight), map(&ptr_->map), scale(&ptr_->scale), rgba(&ptr_->rgba) {}
MjVisual::MjVisual() : ptr_(new mjVisual), global(&ptr_->global), quality(&ptr_->quality), headlight(&ptr_->headlight), map(&ptr_->map), scale(&ptr_->scale), rgba(&ptr_->rgba) {
  owned_ = true;
  mj_defaultVisual(ptr_);
}
MjVisual::MjVisual(const MjVisual &other) : MjVisual() {
  *ptr_ = *other.get();
  global.set(&ptr_->global);
  quality.set(&ptr_->quality);
  headlight.set(&ptr_->headlight);
  map.set(&ptr_->map);
  scale.set(&ptr_->scale);
  rgba.set(&ptr_->rgba);
}
MjVisual& MjVisual::operator=(const MjVisual &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  global.set(&ptr_->global);
  quality.set(&ptr_->quality);
  headlight.set(&ptr_->headlight);
  map.set(&ptr_->map);
  scale.set(&ptr_->scale);
  rgba.set(&ptr_->rgba);
  return *this;
}
MjVisual::~MjVisual() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjVisual> MjVisual::copy() {
  return std::make_unique<MjVisual>(*this);
}

// =============== MjSolverStat =============== //
MjSolverStat::MjSolverStat(mjSolverStat *ptr) : ptr_(ptr) {}
MjSolverStat::MjSolverStat() : ptr_(new mjSolverStat) {
  owned_ = true;
}
MjSolverStat::MjSolverStat(const MjSolverStat &other) : MjSolverStat() {
  *ptr_ = *other.get();
}
MjSolverStat& MjSolverStat::operator=(const MjSolverStat &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjSolverStat::~MjSolverStat() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjSolverStat> MjSolverStat::copy() {
  return std::make_unique<MjSolverStat>(*this);
}

// =============== MjTimerStat =============== //
MjTimerStat::MjTimerStat(mjTimerStat *ptr) : ptr_(ptr) {}
MjTimerStat::MjTimerStat() : ptr_(new mjTimerStat) {
  owned_ = true;
}
MjTimerStat::MjTimerStat(const MjTimerStat &other) : MjTimerStat() {
  *ptr_ = *other.get();
}
MjTimerStat& MjTimerStat::operator=(const MjTimerStat &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjTimerStat::~MjTimerStat() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjTimerStat> MjTimerStat::copy() {
  return std::make_unique<MjTimerStat>(*this);
}

// =============== MjWarningStat =============== //
MjWarningStat::MjWarningStat(mjWarningStat *ptr) : ptr_(ptr) {}
MjWarningStat::MjWarningStat() : ptr_(new mjWarningStat) {
  owned_ = true;
}
MjWarningStat::MjWarningStat(const MjWarningStat &other) : MjWarningStat() {
  *ptr_ = *other.get();
}
MjWarningStat& MjWarningStat::operator=(const MjWarningStat &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjWarningStat::~MjWarningStat() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjWarningStat> MjWarningStat::copy() {
  return std::make_unique<MjWarningStat>(*this);
}

// =============== MjContact =============== //
MjContact::MjContact(mjContact *ptr) : ptr_(ptr) {}
MjContact::MjContact() : ptr_(new mjContact) {
  owned_ = true;
}
MjContact::MjContact(const MjContact &other) : MjContact() {
  *ptr_ = *other.get();
}
MjContact& MjContact::operator=(const MjContact &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjContact::~MjContact() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjContact> MjContact::copy() {
  return std::make_unique<MjContact>(*this);
}

// =============== MjModel =============== //
MjModel::MjModel(mjModel *m)
    : ptr_(m), opt(&m->opt), stat(&m->stat), vis(&m->vis) {}
MjModel::MjModel(const MjModel &other)
    : ptr_(mj_copyModel(nullptr, other.get())),
      opt(&ptr_->opt),
      stat(&ptr_->stat),
      vis(&ptr_->vis) {}
MjModel::~MjModel() {
  if (ptr_) {
    mj_deleteModel(ptr_);
  }
}

// TODO(manevi): Consider passing `const MjModel& m` here, mj_makeData uses a const model.
// =============== MjData =============== //
MjData::MjData(MjModel *m) {
  model = m->get();
  ptr_ = mj_makeData(model);
  if (ptr_) {
    solver = InitSolverArray();
    timer = InitTimerArray();
    warning = InitWarningArray();
  }
}
MjData::MjData(const MjModel &model, const MjData &other)
    : ptr_(mj_copyData(nullptr, model.get(), other.get())), model(model.get()) {
  if (ptr_) {
    solver = InitSolverArray();
    timer = InitTimerArray();
    warning = InitWarningArray();
  }
}
MjData::~MjData() {
  if (ptr_) {
    mj_deleteData(ptr_);
  }
}
std::vector<MjSolverStat> MjData::InitSolverArray() {
  std::vector<MjSolverStat> arr;
  arr.reserve(mjNSOLVER * mjNISLAND);
  for (int i = 0; i < mjNSOLVER * mjNISLAND; i++) {
    arr.emplace_back(&get()->solver[i]);
  }
  return arr;
}
std::vector<MjTimerStat> MjData::InitTimerArray() {
  std::vector<MjTimerStat> arr;
  arr.reserve(mjNTIMER);
  for (int i = 0; i < mjNTIMER; i++) {
    arr.emplace_back(&get()->timer[i]);
  }
  return arr;
}
std::vector<MjWarningStat>
MjData::InitWarningArray() {
  std::vector<MjWarningStat> arr;
  arr.reserve(mjNWARNING);
  for (int i = 0; i < mjNWARNING; i++) {
    arr.emplace_back(&get()->warning[i]);
  }
  return arr;
}
std::vector<MjContact> MjData::contact() const {
  std::vector<MjContact> contacts;
  contacts.reserve(get()->ncon);
  for (int i = 0; i < get()->ncon; ++i) {
    contacts.emplace_back(&get()->contact[i]);
  }
  return contacts;
}
// =============== MjvPerturb =============== //
MjvPerturb::MjvPerturb(mjvPerturb *ptr) : ptr_(ptr) {}
MjvPerturb::MjvPerturb() : ptr_(new mjvPerturb) {
  owned_ = true;
  mjv_defaultPerturb(ptr_);
}
MjvPerturb::MjvPerturb(const MjvPerturb &other) : MjvPerturb() {
  *ptr_ = *other.get();
}
MjvPerturb& MjvPerturb::operator=(const MjvPerturb &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjvPerturb::~MjvPerturb() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjvPerturb> MjvPerturb::copy() {
  return std::make_unique<MjvPerturb>(*this);
}

// =============== MjvCamera =============== //
MjvCamera::MjvCamera(mjvCamera *ptr) : ptr_(ptr) {}
MjvCamera::MjvCamera() : ptr_(new mjvCamera) {
  owned_ = true;
  mjv_defaultCamera(ptr_);
}
MjvCamera::MjvCamera(const MjvCamera &other) : MjvCamera() {
  *ptr_ = *other.get();
}
MjvCamera& MjvCamera::operator=(const MjvCamera &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjvCamera::~MjvCamera() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjvCamera> MjvCamera::copy() {
  return std::make_unique<MjvCamera>(*this);
}

// =============== MjvGLCamera =============== //
MjvGLCamera::MjvGLCamera(mjvGLCamera *ptr) : ptr_(ptr) {}
MjvGLCamera::MjvGLCamera() : ptr_(new mjvGLCamera) {
  owned_ = true;
}
MjvGLCamera::MjvGLCamera(const MjvGLCamera &other) : MjvGLCamera() {
  *ptr_ = *other.get();
}
MjvGLCamera& MjvGLCamera::operator=(const MjvGLCamera &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjvGLCamera::~MjvGLCamera() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjvGLCamera> MjvGLCamera::copy() {
  return std::make_unique<MjvGLCamera>(*this);
}

// =============== MjvGeom =============== //
MjvGeom::MjvGeom(mjvGeom *ptr) { ptr_ = ptr; };
MjvGeom::MjvGeom() : ptr_(new mjvGeom) {
  owned_ = true;
  mjv_initGeom(ptr_, mjGEOM_NONE, nullptr, nullptr, nullptr, nullptr);
};
MjvGeom::MjvGeom(const MjvGeom &other) : MjvGeom() {
  *ptr_ = *other.get();
}
MjvGeom &MjvGeom::operator=(
    const MjvGeom &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjvGeom::~MjvGeom() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjvGeom> MjvGeom::copy() {
  return std::make_unique<MjvGeom>(*this);
}

// =============== MjvLight =============== //
MjvLight::MjvLight(mjvLight *ptr) : ptr_(ptr) {}
MjvLight::MjvLight() : ptr_(new mjvLight) {
  owned_ = true;
}
MjvLight::MjvLight(const MjvLight &other) : MjvLight() {
  *ptr_ = *other.get();
}
MjvLight& MjvLight::operator=(const MjvLight &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjvLight::~MjvLight() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjvLight> MjvLight::copy() {
  return std::make_unique<MjvLight>(*this);
}

// =============== MjvOption =============== //
MjvOption::MjvOption(mjvOption *ptr) : ptr_(ptr) {}
MjvOption::MjvOption() : ptr_(new mjvOption) {
  owned_ = true;
  mjv_defaultOption(ptr_);
}
MjvOption::MjvOption(const MjvOption &other) : MjvOption() {
  *ptr_ = *other.get();
}
MjvOption& MjvOption::operator=(const MjvOption &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjvOption::~MjvOption() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjvOption> MjvOption::copy() {
  return std::make_unique<MjvOption>(*this);
}

// =============== MjvScene =============== //
MjvScene::MjvScene() {
  owned_ = true;
  ptr_ = new mjvScene;
  mjv_defaultScene(ptr_);
  mjv_makeScene(nullptr, ptr_, 0);
  lights = InitLightsArray();
  camera = InitCameraArray();
};

MjvScene::MjvScene(MjModel *m, int maxgeom) {
  owned_ = true;
  model = m->get();
  ptr_ = new mjvScene;
  mjv_defaultScene(ptr_);
  mjv_makeScene(model, ptr_, maxgeom);
  lights = InitLightsArray();
  camera = InitCameraArray();
};
MjvScene::~MjvScene() {
  if (owned_ && ptr_) {
    mjv_freeScene(ptr_);
    delete ptr_;
  }
}

// Taken from the python mujoco bindings code for MjvScene Wrapper
int MjvScene::GetSumFlexFaces() const {
  int nflexface = 0;
  int flexfacenum = 0;
  for (int f = 0; f < model->nflex; f++) {
    if (model->flex_dim[f] == 0) {
      // 1D : 0
      flexfacenum = 0;
    } else if (model->flex_dim[f] == 2) {
      // 2D: 2*fragments + 2*elements
      flexfacenum = 2 * model->flex_shellnum[f] + 2 * model->flex_elemnum[f];
    } else {
      // 3D: max(fragments, 4*maxlayer)
      // find number of elements in biggest layer
      int maxlayer = 0, layer = 0, nlayer = 1;
      while (nlayer) {
        nlayer = 0;
        for (int e = 0; e < model->flex_elemnum[f]; e++) {
          if (model->flex_elemlayer[model->flex_elemadr[f] + e] == layer) {
            nlayer++;
          }
        }
        maxlayer = mjMAX(maxlayer, nlayer);
        layer++;
      }
      flexfacenum = mjMAX(model->flex_shellnum[f], 4 * maxlayer);
    }

    // accumulate over flexes
    nflexface += flexfacenum;
  }
  return nflexface;
}

std::vector<MjvLight> MjvScene::InitLightsArray() {
  std::vector<MjvLight> arr;
  arr.reserve(mjMAXLIGHT);
  for (int i = 0; i < mjMAXLIGHT; i++) {
    arr.emplace_back(&ptr_->lights[i]);
  }
  return arr;
}

std::vector<MjvGLCamera> MjvScene::InitCameraArray() {
  std::vector<MjvGLCamera> arr;
  arr.reserve(2);
  for (int i = 0; i < 2; i++) {
    arr.emplace_back(&ptr_->camera[i]);
  }
  return arr;
}

std::vector<MjvGeom> MjvScene::geoms() const {
  std::vector<MjvGeom> geoms;
  geoms.reserve(ptr_->ngeom);
  for (int i = 0; i < ptr_->ngeom; ++i) {
    geoms.emplace_back(&ptr_->geoms[i]);
  }
  return geoms;
}

// =============== MjvFigure =============== //
MjvFigure::MjvFigure(mjvFigure *ptr) : ptr_(ptr) {}
MjvFigure::MjvFigure() : ptr_(new mjvFigure) {
  owned_ = true;
  mjv_defaultFigure(ptr_);
}
MjvFigure::MjvFigure(const MjvFigure &other) : MjvFigure() {
  *ptr_ = *other.get();
}
MjvFigure& MjvFigure::operator=(const MjvFigure &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
MjvFigure::~MjvFigure() {
  if (owned_ && ptr_) delete ptr_;
}
std::unique_ptr<MjvFigure> MjvFigure::copy() {
  return std::make_unique<MjvFigure>(*this);
}

// =============== MjsElement =============== //
MjsElement::MjsElement(mjsElement *ptr) : ptr_(ptr) {}
MjsElement::~MjsElement() {}
std::unique_ptr<MjsElement> MjsElement::copy() {
  return std::make_unique<MjsElement>(*this);
}

// =============== MjsCompiler =============== //
MjsCompiler::MjsCompiler(mjsCompiler *ptr) : ptr_(ptr), LRopt(&ptr_->LRopt) {}
MjsCompiler::~MjsCompiler() {}

// =============== MjSpec =============== //
MjSpec::MjSpec()
    : ptr_(mj_makeSpec()),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat),
      compiler(&ptr_->compiler),
      element(ptr_->element) {
  owned_ = true;
  mjs_defaultSpec(ptr_);
};

MjSpec::MjSpec(mjSpec *ptr)
    : ptr_(ptr),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat),
      compiler(&ptr_->compiler),
      element(ptr_->element) {}

MjSpec::MjSpec(const MjSpec &other)
    : ptr_(mj_copySpec(other.get())),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat),
      compiler(&ptr_->compiler),
      element(ptr_->element) {
  owned_ = true;
}

MjSpec& MjSpec::operator=(const MjSpec &other) {
  if (this == &other) {
    return *this;
  }
  if (owned_ && ptr_) {
    mj_deleteSpec(ptr_);
  }
  ptr_ = mj_copySpec(other.get());
  owned_ = true;
  option.set(&ptr_->option);
  visual.set(&ptr_->visual);
  stat.set(&ptr_->stat);
  compiler.set(&ptr_->compiler);
  element.set(ptr_->element);
  return *this;
}

MjSpec::~MjSpec() {
  if (ptr_ && owned_) {
    mj_deleteSpec(ptr_);
  }
}

// =============== MjsOrientation =============== //
MjsOrientation::MjsOrientation(mjsOrientation *ptr) : ptr_(ptr) {}
MjsOrientation::~MjsOrientation() {}
std::unique_ptr<MjsOrientation> MjsOrientation::copy() {
  return std::make_unique<MjsOrientation>(*this);
}

// =============== MjsBody =============== //
MjsBody::MjsBody(mjsBody *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt), ialt(&ptr_->ialt), plugin(&ptr_->plugin) {}
MjsBody::~MjsBody() {}

// =============== MjsGeom =============== //
MjsGeom::MjsGeom(mjsGeom *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt), plugin(&ptr_->plugin) {}
MjsGeom::~MjsGeom() {}

// =============== MjsFrame =============== //
MjsFrame::MjsFrame(mjsFrame *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt) {}
MjsFrame::~MjsFrame() {}

// =============== MjsJoint =============== //
MjsJoint::MjsJoint(mjsJoint *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsJoint::~MjsJoint() {}

// =============== MjsSite =============== //
MjsSite::MjsSite(mjsSite *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt) {}
MjsSite::~MjsSite() {}

// =============== MjsCamera =============== //
MjsCamera::MjsCamera(mjsCamera *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt) {}
MjsCamera::~MjsCamera() {}

// =============== MjsLight =============== //
MjsLight::MjsLight(mjsLight *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsLight::~MjsLight() {}

// =============== MjsFlex =============== //
MjsFlex::MjsFlex(mjsFlex *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsFlex::~MjsFlex() {}

// =============== MjsMesh =============== //
MjsMesh::MjsMesh(mjsMesh *ptr) : ptr_(ptr), element(ptr_->element), plugin(&ptr_->plugin) {}
MjsMesh::~MjsMesh() {}

// =============== MjsHField =============== //
MjsHField::MjsHField(mjsHField *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsHField::~MjsHField() {}

// =============== MjsSkin =============== //
MjsSkin::MjsSkin(mjsSkin *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsSkin::~MjsSkin() {}

// =============== MjsTexture =============== //
MjsTexture::MjsTexture(mjsTexture *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsTexture::~MjsTexture() {}

// =============== MjsMaterial =============== //
MjsMaterial::MjsMaterial(mjsMaterial *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsMaterial::~MjsMaterial() {}

// =============== MjsPair =============== //
MjsPair::MjsPair(mjsPair *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsPair::~MjsPair() {}

// =============== MjsExclude =============== //
MjsExclude::MjsExclude(mjsExclude *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsExclude::~MjsExclude() {}

// =============== MjsEquality =============== //
MjsEquality::MjsEquality(mjsEquality *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsEquality::~MjsEquality() {}

// =============== MjsTendon =============== //
MjsTendon::MjsTendon(mjsTendon *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsTendon::~MjsTendon() {}

// =============== MjsWrap =============== //
MjsWrap::MjsWrap(mjsWrap *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsWrap::~MjsWrap() {}

// =============== MjsActuator =============== //
MjsActuator::MjsActuator(mjsActuator *ptr) : ptr_(ptr), element(ptr_->element), plugin(&ptr_->plugin) {}
MjsActuator::~MjsActuator() {}

// =============== MjsSensor =============== //
MjsSensor::MjsSensor(mjsSensor *ptr) : ptr_(ptr), element(ptr_->element), plugin(&ptr_->plugin) {}
MjsSensor::~MjsSensor() {}

// =============== MjsNumeric =============== //
MjsNumeric::MjsNumeric(mjsNumeric *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsNumeric::~MjsNumeric() {}

// =============== MjsText =============== //
MjsText::MjsText(mjsText *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsText::~MjsText() {}

// =============== MjsTuple =============== //
MjsTuple::MjsTuple(mjsTuple *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsTuple::~MjsTuple() {}

// =============== MjsKey =============== //
MjsKey::MjsKey(mjsKey *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsKey::~MjsKey() {}

// =============== MjsDefault =============== //
MjsDefault::MjsDefault(mjsDefault *ptr) : ptr_(ptr), element(ptr_->element), joint(ptr_->joint), geom(ptr_->geom), site(ptr_->site), camera(ptr_->camera), light(ptr_->light), flex(ptr_->flex), mesh(ptr_->mesh), material(ptr_->material), pair(ptr_->pair), equality(ptr_->equality), tendon(ptr_->tendon), actuator(ptr_->actuator) {}
MjsDefault::~MjsDefault() {}

// =============== MjsPlugin =============== //
MjsPlugin::MjsPlugin(mjsPlugin *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsPlugin::~MjsPlugin() {}

// =============== MjVFS =============== //
MjVFS::MjVFS(mjVFS *ptr) : ptr_(ptr) {}
MjVFS::MjVFS() : ptr_(new mjVFS) {
  owned_ = true;
  mj_defaultVFS(ptr_);
}
MjVFS::~MjVFS() {
  if (owned_ && ptr_) {
    mj_deleteVFS(ptr_);
  }
}

// ======= FACTORY AND HELPER FUNCTIONS ========= //
std::unique_ptr<MjModel> loadFromXML(std::string filename) {
  char error[1000];
  mjModel *model = mj_loadXML(filename.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    printf("Loading error: %s\n", error);
    return nullptr;
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjSpec> parseXMLString(const std::string &xml) {
  char error[1000];
  mjSpec *ptr = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
  if (!ptr) {
    printf("Could not create Spec from XML string: %s\n", error);
    return nullptr;
  }
  return std::unique_ptr<MjSpec>(new MjSpec(ptr));
}

EMSCRIPTEN_BINDINGS(mujoco_bindings) {
  function("parseXMLString", &parseXMLString, take_ownership());

  emscripten::class_<MjLROpt>("MjLROpt")
      .constructor<>()
      .function("copy", &MjLROpt::copy, take_ownership())
      .property("mode", &MjLROpt::mode, &MjLROpt::set_mode, reference())
      .property("useexisting", &MjLROpt::useexisting, &MjLROpt::set_useexisting, reference())
      .property("uselimit", &MjLROpt::uselimit, &MjLROpt::set_uselimit, reference())
      .property("accel", &MjLROpt::accel, &MjLROpt::set_accel, reference())
      .property("maxforce", &MjLROpt::maxforce, &MjLROpt::set_maxforce, reference())
      .property("timeconst", &MjLROpt::timeconst, &MjLROpt::set_timeconst, reference())
      .property("timestep", &MjLROpt::timestep, &MjLROpt::set_timestep, reference())
      .property("inttotal", &MjLROpt::inttotal, &MjLROpt::set_inttotal, reference())
      .property("interval", &MjLROpt::interval, &MjLROpt::set_interval, reference())
      .property("tolrange", &MjLROpt::tolrange, &MjLROpt::set_tolrange, reference())
      ;
  emscripten::class_<MjModel>("MjModel")
      .class_function("loadFromXML", &loadFromXML, take_ownership())
      .constructor<const MjModel &>()
      .property("nq", &MjModel::nq, &MjModel::set_nq, reference())
      .property("nv", &MjModel::nv, &MjModel::set_nv, reference())
      .property("nu", &MjModel::nu, &MjModel::set_nu, reference())
      .property("na", &MjModel::na, &MjModel::set_na, reference())
      .property("nbody", &MjModel::nbody, &MjModel::set_nbody, reference())
      .property("nbvh", &MjModel::nbvh, &MjModel::set_nbvh, reference())
      .property("nbvhstatic", &MjModel::nbvhstatic, &MjModel::set_nbvhstatic, reference())
      .property("nbvhdynamic", &MjModel::nbvhdynamic, &MjModel::set_nbvhdynamic, reference())
      .property("noct", &MjModel::noct, &MjModel::set_noct, reference())
      .property("njnt", &MjModel::njnt, &MjModel::set_njnt, reference())
      .property("ntree", &MjModel::ntree, &MjModel::set_ntree, reference())
      .property("nM", &MjModel::nM, &MjModel::set_nM, reference())
      .property("nB", &MjModel::nB, &MjModel::set_nB, reference())
      .property("nC", &MjModel::nC, &MjModel::set_nC, reference())
      .property("nD", &MjModel::nD, &MjModel::set_nD, reference())
      .property("ngeom", &MjModel::ngeom, &MjModel::set_ngeom, reference())
      .property("nsite", &MjModel::nsite, &MjModel::set_nsite, reference())
      .property("ncam", &MjModel::ncam, &MjModel::set_ncam, reference())
      .property("nlight", &MjModel::nlight, &MjModel::set_nlight, reference())
      .property("nflex", &MjModel::nflex, &MjModel::set_nflex, reference())
      .property("nflexnode", &MjModel::nflexnode, &MjModel::set_nflexnode, reference())
      .property("nflexvert", &MjModel::nflexvert, &MjModel::set_nflexvert, reference())
      .property("nflexedge", &MjModel::nflexedge, &MjModel::set_nflexedge, reference())
      .property("nflexelem", &MjModel::nflexelem, &MjModel::set_nflexelem, reference())
      .property("nflexelemdata", &MjModel::nflexelemdata, &MjModel::set_nflexelemdata, reference())
      .property("nflexelemedge", &MjModel::nflexelemedge, &MjModel::set_nflexelemedge, reference())
      .property("nflexshelldata", &MjModel::nflexshelldata, &MjModel::set_nflexshelldata, reference())
      .property("nflexevpair", &MjModel::nflexevpair, &MjModel::set_nflexevpair, reference())
      .property("nflextexcoord", &MjModel::nflextexcoord, &MjModel::set_nflextexcoord, reference())
      .property("nmesh", &MjModel::nmesh, &MjModel::set_nmesh, reference())
      .property("nmeshvert", &MjModel::nmeshvert, &MjModel::set_nmeshvert, reference())
      .property("nmeshnormal", &MjModel::nmeshnormal, &MjModel::set_nmeshnormal, reference())
      .property("nmeshtexcoord", &MjModel::nmeshtexcoord, &MjModel::set_nmeshtexcoord, reference())
      .property("nmeshface", &MjModel::nmeshface, &MjModel::set_nmeshface, reference())
      .property("nmeshgraph", &MjModel::nmeshgraph, &MjModel::set_nmeshgraph, reference())
      .property("nmeshpoly", &MjModel::nmeshpoly, &MjModel::set_nmeshpoly, reference())
      .property("nmeshpolyvert", &MjModel::nmeshpolyvert, &MjModel::set_nmeshpolyvert, reference())
      .property("nmeshpolymap", &MjModel::nmeshpolymap, &MjModel::set_nmeshpolymap, reference())
      .property("nskin", &MjModel::nskin, &MjModel::set_nskin, reference())
      .property("nskinvert", &MjModel::nskinvert, &MjModel::set_nskinvert, reference())
      .property("nskintexvert", &MjModel::nskintexvert, &MjModel::set_nskintexvert, reference())
      .property("nskinface", &MjModel::nskinface, &MjModel::set_nskinface, reference())
      .property("nskinbone", &MjModel::nskinbone, &MjModel::set_nskinbone, reference())
      .property("nskinbonevert", &MjModel::nskinbonevert, &MjModel::set_nskinbonevert, reference())
      .property("nhfield", &MjModel::nhfield, &MjModel::set_nhfield, reference())
      .property("nhfielddata", &MjModel::nhfielddata, &MjModel::set_nhfielddata, reference())
      .property("ntex", &MjModel::ntex, &MjModel::set_ntex, reference())
      .property("ntexdata", &MjModel::ntexdata, &MjModel::set_ntexdata, reference())
      .property("nmat", &MjModel::nmat, &MjModel::set_nmat, reference())
      .property("npair", &MjModel::npair, &MjModel::set_npair, reference())
      .property("nexclude", &MjModel::nexclude, &MjModel::set_nexclude, reference())
      .property("neq", &MjModel::neq, &MjModel::set_neq, reference())
      .property("ntendon", &MjModel::ntendon, &MjModel::set_ntendon, reference())
      .property("nwrap", &MjModel::nwrap, &MjModel::set_nwrap, reference())
      .property("nsensor", &MjModel::nsensor, &MjModel::set_nsensor, reference())
      .property("nnumeric", &MjModel::nnumeric, &MjModel::set_nnumeric, reference())
      .property("nnumericdata", &MjModel::nnumericdata, &MjModel::set_nnumericdata, reference())
      .property("ntext", &MjModel::ntext, &MjModel::set_ntext, reference())
      .property("ntextdata", &MjModel::ntextdata, &MjModel::set_ntextdata, reference())
      .property("ntuple", &MjModel::ntuple, &MjModel::set_ntuple, reference())
      .property("ntupledata", &MjModel::ntupledata, &MjModel::set_ntupledata, reference())
      .property("nkey", &MjModel::nkey, &MjModel::set_nkey, reference())
      .property("nmocap", &MjModel::nmocap, &MjModel::set_nmocap, reference())
      .property("nplugin", &MjModel::nplugin, &MjModel::set_nplugin, reference())
      .property("npluginattr", &MjModel::npluginattr, &MjModel::set_npluginattr, reference())
      .property("nuser_body", &MjModel::nuser_body, &MjModel::set_nuser_body, reference())
      .property("nuser_jnt", &MjModel::nuser_jnt, &MjModel::set_nuser_jnt, reference())
      .property("nuser_geom", &MjModel::nuser_geom, &MjModel::set_nuser_geom, reference())
      .property("nuser_site", &MjModel::nuser_site, &MjModel::set_nuser_site, reference())
      .property("nuser_cam", &MjModel::nuser_cam, &MjModel::set_nuser_cam, reference())
      .property("nuser_tendon", &MjModel::nuser_tendon, &MjModel::set_nuser_tendon, reference())
      .property("nuser_actuator", &MjModel::nuser_actuator, &MjModel::set_nuser_actuator, reference())
      .property("nuser_sensor", &MjModel::nuser_sensor, &MjModel::set_nuser_sensor, reference())
      .property("nnames", &MjModel::nnames, &MjModel::set_nnames, reference())
      .property("npaths", &MjModel::npaths, &MjModel::set_npaths, reference())
      .property("nnames_map", &MjModel::nnames_map, &MjModel::set_nnames_map, reference())
      .property("nJmom", &MjModel::nJmom, &MjModel::set_nJmom, reference())
      .property("ngravcomp", &MjModel::ngravcomp, &MjModel::set_ngravcomp, reference())
      .property("nemax", &MjModel::nemax, &MjModel::set_nemax, reference())
      .property("njmax", &MjModel::njmax, &MjModel::set_njmax, reference())
      .property("nconmax", &MjModel::nconmax, &MjModel::set_nconmax, reference())
      .property("nuserdata", &MjModel::nuserdata, &MjModel::set_nuserdata, reference())
      .property("nsensordata", &MjModel::nsensordata, &MjModel::set_nsensordata, reference())
      .property("npluginstate", &MjModel::npluginstate, &MjModel::set_npluginstate, reference())
      .property("narena", &MjModel::narena, &MjModel::set_narena, reference())
      .property("nbuffer", &MjModel::nbuffer, &MjModel::set_nbuffer, reference())
      .property("opt", &MjModel::opt, reference())
      .property("vis", &MjModel::vis, reference())
      .property("stat", &MjModel::stat, reference())
      .property("buffer", &MjModel::buffer)
      .property("qpos0", &MjModel::qpos0)
      .property("qpos_spring", &MjModel::qpos_spring)
      .property("body_parentid", &MjModel::body_parentid)
      .property("body_rootid", &MjModel::body_rootid)
      .property("body_weldid", &MjModel::body_weldid)
      .property("body_mocapid", &MjModel::body_mocapid)
      .property("body_jntnum", &MjModel::body_jntnum)
      .property("body_jntadr", &MjModel::body_jntadr)
      .property("body_dofnum", &MjModel::body_dofnum)
      .property("body_dofadr", &MjModel::body_dofadr)
      .property("body_treeid", &MjModel::body_treeid)
      .property("body_geomnum", &MjModel::body_geomnum)
      .property("body_geomadr", &MjModel::body_geomadr)
      .property("body_simple", &MjModel::body_simple)
      .property("body_sameframe", &MjModel::body_sameframe)
      .property("body_pos", &MjModel::body_pos)
      .property("body_quat", &MjModel::body_quat)
      .property("body_ipos", &MjModel::body_ipos)
      .property("body_iquat", &MjModel::body_iquat)
      .property("body_mass", &MjModel::body_mass)
      .property("body_subtreemass", &MjModel::body_subtreemass)
      .property("body_inertia", &MjModel::body_inertia)
      .property("body_invweight0", &MjModel::body_invweight0)
      .property("body_gravcomp", &MjModel::body_gravcomp)
      .property("body_margin", &MjModel::body_margin)
      .property("body_user", &MjModel::body_user)
      .property("body_plugin", &MjModel::body_plugin)
      .property("body_contype", &MjModel::body_contype)
      .property("body_conaffinity", &MjModel::body_conaffinity)
      .property("body_bvhadr", &MjModel::body_bvhadr)
      .property("body_bvhnum", &MjModel::body_bvhnum)
      .property("bvh_depth", &MjModel::bvh_depth)
      .property("bvh_child", &MjModel::bvh_child)
      .property("bvh_nodeid", &MjModel::bvh_nodeid)
      .property("bvh_aabb", &MjModel::bvh_aabb)
      .property("oct_depth", &MjModel::oct_depth)
      .property("oct_child", &MjModel::oct_child)
      .property("oct_aabb", &MjModel::oct_aabb)
      .property("oct_coeff", &MjModel::oct_coeff)
      .property("jnt_type", &MjModel::jnt_type)
      .property("jnt_qposadr", &MjModel::jnt_qposadr)
      .property("jnt_dofadr", &MjModel::jnt_dofadr)
      .property("jnt_bodyid", &MjModel::jnt_bodyid)
      .property("jnt_group", &MjModel::jnt_group)
      .property("jnt_limited", &MjModel::jnt_limited)
      .property("jnt_actfrclimited", &MjModel::jnt_actfrclimited)
      .property("jnt_actgravcomp", &MjModel::jnt_actgravcomp)
      .property("jnt_solref", &MjModel::jnt_solref)
      .property("jnt_solimp", &MjModel::jnt_solimp)
      .property("jnt_pos", &MjModel::jnt_pos)
      .property("jnt_axis", &MjModel::jnt_axis)
      .property("jnt_stiffness", &MjModel::jnt_stiffness)
      .property("jnt_range", &MjModel::jnt_range)
      .property("jnt_actfrcrange", &MjModel::jnt_actfrcrange)
      .property("jnt_margin", &MjModel::jnt_margin)
      .property("jnt_user", &MjModel::jnt_user)
      .property("dof_bodyid", &MjModel::dof_bodyid)
      .property("dof_jntid", &MjModel::dof_jntid)
      .property("dof_parentid", &MjModel::dof_parentid)
      .property("dof_treeid", &MjModel::dof_treeid)
      .property("dof_Madr", &MjModel::dof_Madr)
      .property("dof_simplenum", &MjModel::dof_simplenum)
      .property("dof_solref", &MjModel::dof_solref)
      .property("dof_solimp", &MjModel::dof_solimp)
      .property("dof_frictionloss", &MjModel::dof_frictionloss)
      .property("dof_armature", &MjModel::dof_armature)
      .property("dof_damping", &MjModel::dof_damping)
      .property("dof_invweight0", &MjModel::dof_invweight0)
      .property("dof_M0", &MjModel::dof_M0)
      .property("geom_type", &MjModel::geom_type)
      .property("geom_contype", &MjModel::geom_contype)
      .property("geom_conaffinity", &MjModel::geom_conaffinity)
      .property("geom_condim", &MjModel::geom_condim)
      .property("geom_bodyid", &MjModel::geom_bodyid)
      .property("geom_dataid", &MjModel::geom_dataid)
      .property("geom_matid", &MjModel::geom_matid)
      .property("geom_group", &MjModel::geom_group)
      .property("geom_priority", &MjModel::geom_priority)
      .property("geom_plugin", &MjModel::geom_plugin)
      .property("geom_sameframe", &MjModel::geom_sameframe)
      .property("geom_solmix", &MjModel::geom_solmix)
      .property("geom_solref", &MjModel::geom_solref)
      .property("geom_solimp", &MjModel::geom_solimp)
      .property("geom_size", &MjModel::geom_size)
      .property("geom_aabb", &MjModel::geom_aabb)
      .property("geom_rbound", &MjModel::geom_rbound)
      .property("geom_pos", &MjModel::geom_pos)
      .property("geom_quat", &MjModel::geom_quat)
      .property("geom_friction", &MjModel::geom_friction)
      .property("geom_margin", &MjModel::geom_margin)
      .property("geom_gap", &MjModel::geom_gap)
      .property("geom_fluid", &MjModel::geom_fluid)
      .property("geom_user", &MjModel::geom_user)
      .property("geom_rgba", &MjModel::geom_rgba)
      .property("site_type", &MjModel::site_type)
      .property("site_bodyid", &MjModel::site_bodyid)
      .property("site_matid", &MjModel::site_matid)
      .property("site_group", &MjModel::site_group)
      .property("site_sameframe", &MjModel::site_sameframe)
      .property("site_size", &MjModel::site_size)
      .property("site_pos", &MjModel::site_pos)
      .property("site_quat", &MjModel::site_quat)
      .property("site_user", &MjModel::site_user)
      .property("site_rgba", &MjModel::site_rgba)
      .property("cam_mode", &MjModel::cam_mode)
      .property("cam_bodyid", &MjModel::cam_bodyid)
      .property("cam_targetbodyid", &MjModel::cam_targetbodyid)
      .property("cam_pos", &MjModel::cam_pos)
      .property("cam_quat", &MjModel::cam_quat)
      .property("cam_poscom0", &MjModel::cam_poscom0)
      .property("cam_pos0", &MjModel::cam_pos0)
      .property("cam_mat0", &MjModel::cam_mat0)
      .property("cam_orthographic", &MjModel::cam_orthographic)
      .property("cam_fovy", &MjModel::cam_fovy)
      .property("cam_ipd", &MjModel::cam_ipd)
      .property("cam_resolution", &MjModel::cam_resolution)
      .property("cam_sensorsize", &MjModel::cam_sensorsize)
      .property("cam_intrinsic", &MjModel::cam_intrinsic)
      .property("cam_user", &MjModel::cam_user)
      .property("light_mode", &MjModel::light_mode)
      .property("light_bodyid", &MjModel::light_bodyid)
      .property("light_targetbodyid", &MjModel::light_targetbodyid)
      .property("light_type", &MjModel::light_type)
      .property("light_texid", &MjModel::light_texid)
      .property("light_castshadow", &MjModel::light_castshadow)
      .property("light_bulbradius", &MjModel::light_bulbradius)
      .property("light_intensity", &MjModel::light_intensity)
      .property("light_range", &MjModel::light_range)
      .property("light_active", &MjModel::light_active)
      .property("light_pos", &MjModel::light_pos)
      .property("light_dir", &MjModel::light_dir)
      .property("light_poscom0", &MjModel::light_poscom0)
      .property("light_pos0", &MjModel::light_pos0)
      .property("light_dir0", &MjModel::light_dir0)
      .property("light_attenuation", &MjModel::light_attenuation)
      .property("light_cutoff", &MjModel::light_cutoff)
      .property("light_exponent", &MjModel::light_exponent)
      .property("light_ambient", &MjModel::light_ambient)
      .property("light_diffuse", &MjModel::light_diffuse)
      .property("light_specular", &MjModel::light_specular)
      .property("flex_contype", &MjModel::flex_contype)
      .property("flex_conaffinity", &MjModel::flex_conaffinity)
      .property("flex_condim", &MjModel::flex_condim)
      .property("flex_priority", &MjModel::flex_priority)
      .property("flex_solmix", &MjModel::flex_solmix)
      .property("flex_solref", &MjModel::flex_solref)
      .property("flex_solimp", &MjModel::flex_solimp)
      .property("flex_friction", &MjModel::flex_friction)
      .property("flex_margin", &MjModel::flex_margin)
      .property("flex_gap", &MjModel::flex_gap)
      .property("flex_internal", &MjModel::flex_internal)
      .property("flex_selfcollide", &MjModel::flex_selfcollide)
      .property("flex_activelayers", &MjModel::flex_activelayers)
      .property("flex_passive", &MjModel::flex_passive)
      .property("flex_dim", &MjModel::flex_dim)
      .property("flex_matid", &MjModel::flex_matid)
      .property("flex_group", &MjModel::flex_group)
      .property("flex_interp", &MjModel::flex_interp)
      .property("flex_nodeadr", &MjModel::flex_nodeadr)
      .property("flex_nodenum", &MjModel::flex_nodenum)
      .property("flex_vertadr", &MjModel::flex_vertadr)
      .property("flex_vertnum", &MjModel::flex_vertnum)
      .property("flex_edgeadr", &MjModel::flex_edgeadr)
      .property("flex_edgenum", &MjModel::flex_edgenum)
      .property("flex_elemadr", &MjModel::flex_elemadr)
      .property("flex_elemnum", &MjModel::flex_elemnum)
      .property("flex_elemdataadr", &MjModel::flex_elemdataadr)
      .property("flex_elemedgeadr", &MjModel::flex_elemedgeadr)
      .property("flex_shellnum", &MjModel::flex_shellnum)
      .property("flex_shelldataadr", &MjModel::flex_shelldataadr)
      .property("flex_evpairadr", &MjModel::flex_evpairadr)
      .property("flex_evpairnum", &MjModel::flex_evpairnum)
      .property("flex_texcoordadr", &MjModel::flex_texcoordadr)
      .property("flex_nodebodyid", &MjModel::flex_nodebodyid)
      .property("flex_vertbodyid", &MjModel::flex_vertbodyid)
      .property("flex_edge", &MjModel::flex_edge)
      .property("flex_edgeflap", &MjModel::flex_edgeflap)
      .property("flex_elem", &MjModel::flex_elem)
      .property("flex_elemtexcoord", &MjModel::flex_elemtexcoord)
      .property("flex_elemedge", &MjModel::flex_elemedge)
      .property("flex_elemlayer", &MjModel::flex_elemlayer)
      .property("flex_shell", &MjModel::flex_shell)
      .property("flex_evpair", &MjModel::flex_evpair)
      .property("flex_vert", &MjModel::flex_vert)
      .property("flex_vert0", &MjModel::flex_vert0)
      .property("flex_node", &MjModel::flex_node)
      .property("flex_node0", &MjModel::flex_node0)
      .property("flexedge_length0", &MjModel::flexedge_length0)
      .property("flexedge_invweight0", &MjModel::flexedge_invweight0)
      .property("flex_radius", &MjModel::flex_radius)
      .property("flex_stiffness", &MjModel::flex_stiffness)
      .property("flex_bending", &MjModel::flex_bending)
      .property("flex_damping", &MjModel::flex_damping)
      .property("flex_edgestiffness", &MjModel::flex_edgestiffness)
      .property("flex_edgedamping", &MjModel::flex_edgedamping)
      .property("flex_edgeequality", &MjModel::flex_edgeequality)
      .property("flex_rigid", &MjModel::flex_rigid)
      .property("flexedge_rigid", &MjModel::flexedge_rigid)
      .property("flex_centered", &MjModel::flex_centered)
      .property("flex_flatskin", &MjModel::flex_flatskin)
      .property("flex_bvhadr", &MjModel::flex_bvhadr)
      .property("flex_bvhnum", &MjModel::flex_bvhnum)
      .property("flex_rgba", &MjModel::flex_rgba)
      .property("flex_texcoord", &MjModel::flex_texcoord)
      .property("mesh_vertadr", &MjModel::mesh_vertadr)
      .property("mesh_vertnum", &MjModel::mesh_vertnum)
      .property("mesh_faceadr", &MjModel::mesh_faceadr)
      .property("mesh_facenum", &MjModel::mesh_facenum)
      .property("mesh_bvhadr", &MjModel::mesh_bvhadr)
      .property("mesh_bvhnum", &MjModel::mesh_bvhnum)
      .property("mesh_octadr", &MjModel::mesh_octadr)
      .property("mesh_octnum", &MjModel::mesh_octnum)
      .property("mesh_normaladr", &MjModel::mesh_normaladr)
      .property("mesh_normalnum", &MjModel::mesh_normalnum)
      .property("mesh_texcoordadr", &MjModel::mesh_texcoordadr)
      .property("mesh_texcoordnum", &MjModel::mesh_texcoordnum)
      .property("mesh_graphadr", &MjModel::mesh_graphadr)
      .property("mesh_vert", &MjModel::mesh_vert)
      .property("mesh_normal", &MjModel::mesh_normal)
      .property("mesh_texcoord", &MjModel::mesh_texcoord)
      .property("mesh_face", &MjModel::mesh_face)
      .property("mesh_facenormal", &MjModel::mesh_facenormal)
      .property("mesh_facetexcoord", &MjModel::mesh_facetexcoord)
      .property("mesh_graph", &MjModel::mesh_graph)
      .property("mesh_scale", &MjModel::mesh_scale)
      .property("mesh_pos", &MjModel::mesh_pos)
      .property("mesh_quat", &MjModel::mesh_quat)
      .property("mesh_pathadr", &MjModel::mesh_pathadr)
      .property("mesh_polynum", &MjModel::mesh_polynum)
      .property("mesh_polyadr", &MjModel::mesh_polyadr)
      .property("mesh_polynormal", &MjModel::mesh_polynormal)
      .property("mesh_polyvertadr", &MjModel::mesh_polyvertadr)
      .property("mesh_polyvertnum", &MjModel::mesh_polyvertnum)
      .property("mesh_polyvert", &MjModel::mesh_polyvert)
      .property("mesh_polymapadr", &MjModel::mesh_polymapadr)
      .property("mesh_polymapnum", &MjModel::mesh_polymapnum)
      .property("mesh_polymap", &MjModel::mesh_polymap)
      .property("skin_matid", &MjModel::skin_matid)
      .property("skin_group", &MjModel::skin_group)
      .property("skin_rgba", &MjModel::skin_rgba)
      .property("skin_inflate", &MjModel::skin_inflate)
      .property("skin_vertadr", &MjModel::skin_vertadr)
      .property("skin_vertnum", &MjModel::skin_vertnum)
      .property("skin_texcoordadr", &MjModel::skin_texcoordadr)
      .property("skin_faceadr", &MjModel::skin_faceadr)
      .property("skin_facenum", &MjModel::skin_facenum)
      .property("skin_boneadr", &MjModel::skin_boneadr)
      .property("skin_bonenum", &MjModel::skin_bonenum)
      .property("skin_vert", &MjModel::skin_vert)
      .property("skin_texcoord", &MjModel::skin_texcoord)
      .property("skin_face", &MjModel::skin_face)
      .property("skin_bonevertadr", &MjModel::skin_bonevertadr)
      .property("skin_bonevertnum", &MjModel::skin_bonevertnum)
      .property("skin_bonebindpos", &MjModel::skin_bonebindpos)
      .property("skin_bonebindquat", &MjModel::skin_bonebindquat)
      .property("skin_bonebodyid", &MjModel::skin_bonebodyid)
      .property("skin_bonevertid", &MjModel::skin_bonevertid)
      .property("skin_bonevertweight", &MjModel::skin_bonevertweight)
      .property("skin_pathadr", &MjModel::skin_pathadr)
      .property("hfield_size", &MjModel::hfield_size)
      .property("hfield_nrow", &MjModel::hfield_nrow)
      .property("hfield_ncol", &MjModel::hfield_ncol)
      .property("hfield_adr", &MjModel::hfield_adr)
      .property("hfield_data", &MjModel::hfield_data)
      .property("hfield_pathadr", &MjModel::hfield_pathadr)
      .property("tex_type", &MjModel::tex_type)
      .property("tex_colorspace", &MjModel::tex_colorspace)
      .property("tex_height", &MjModel::tex_height)
      .property("tex_width", &MjModel::tex_width)
      .property("tex_nchannel", &MjModel::tex_nchannel)
      .property("tex_adr", &MjModel::tex_adr)
      .property("tex_data", &MjModel::tex_data)
      .property("tex_pathadr", &MjModel::tex_pathadr)
      .property("mat_texid", &MjModel::mat_texid)
      .property("mat_texuniform", &MjModel::mat_texuniform)
      .property("mat_texrepeat", &MjModel::mat_texrepeat)
      .property("mat_emission", &MjModel::mat_emission)
      .property("mat_specular", &MjModel::mat_specular)
      .property("mat_shininess", &MjModel::mat_shininess)
      .property("mat_reflectance", &MjModel::mat_reflectance)
      .property("mat_metallic", &MjModel::mat_metallic)
      .property("mat_roughness", &MjModel::mat_roughness)
      .property("mat_rgba", &MjModel::mat_rgba)
      .property("pair_dim", &MjModel::pair_dim)
      .property("pair_geom1", &MjModel::pair_geom1)
      .property("pair_geom2", &MjModel::pair_geom2)
      .property("pair_signature", &MjModel::pair_signature)
      .property("pair_solref", &MjModel::pair_solref)
      .property("pair_solreffriction", &MjModel::pair_solreffriction)
      .property("pair_solimp", &MjModel::pair_solimp)
      .property("pair_margin", &MjModel::pair_margin)
      .property("pair_gap", &MjModel::pair_gap)
      .property("pair_friction", &MjModel::pair_friction)
      .property("exclude_signature", &MjModel::exclude_signature)
      .property("eq_type", &MjModel::eq_type)
      .property("eq_obj1id", &MjModel::eq_obj1id)
      .property("eq_obj2id", &MjModel::eq_obj2id)
      .property("eq_objtype", &MjModel::eq_objtype)
      .property("eq_active0", &MjModel::eq_active0)
      .property("eq_solref", &MjModel::eq_solref)
      .property("eq_solimp", &MjModel::eq_solimp)
      .property("eq_data", &MjModel::eq_data)
      .property("tendon_adr", &MjModel::tendon_adr)
      .property("tendon_num", &MjModel::tendon_num)
      .property("tendon_matid", &MjModel::tendon_matid)
      .property("tendon_group", &MjModel::tendon_group)
      .property("tendon_limited", &MjModel::tendon_limited)
      .property("tendon_actfrclimited", &MjModel::tendon_actfrclimited)
      .property("tendon_width", &MjModel::tendon_width)
      .property("tendon_solref_lim", &MjModel::tendon_solref_lim)
      .property("tendon_solimp_lim", &MjModel::tendon_solimp_lim)
      .property("tendon_solref_fri", &MjModel::tendon_solref_fri)
      .property("tendon_solimp_fri", &MjModel::tendon_solimp_fri)
      .property("tendon_range", &MjModel::tendon_range)
      .property("tendon_actfrcrange", &MjModel::tendon_actfrcrange)
      .property("tendon_margin", &MjModel::tendon_margin)
      .property("tendon_stiffness", &MjModel::tendon_stiffness)
      .property("tendon_damping", &MjModel::tendon_damping)
      .property("tendon_armature", &MjModel::tendon_armature)
      .property("tendon_frictionloss", &MjModel::tendon_frictionloss)
      .property("tendon_lengthspring", &MjModel::tendon_lengthspring)
      .property("tendon_length0", &MjModel::tendon_length0)
      .property("tendon_invweight0", &MjModel::tendon_invweight0)
      .property("tendon_user", &MjModel::tendon_user)
      .property("tendon_rgba", &MjModel::tendon_rgba)
      .property("wrap_type", &MjModel::wrap_type)
      .property("wrap_objid", &MjModel::wrap_objid)
      .property("wrap_prm", &MjModel::wrap_prm)
      .property("actuator_trntype", &MjModel::actuator_trntype)
      .property("actuator_dyntype", &MjModel::actuator_dyntype)
      .property("actuator_gaintype", &MjModel::actuator_gaintype)
      .property("actuator_biastype", &MjModel::actuator_biastype)
      .property("actuator_trnid", &MjModel::actuator_trnid)
      .property("actuator_actadr", &MjModel::actuator_actadr)
      .property("actuator_actnum", &MjModel::actuator_actnum)
      .property("actuator_group", &MjModel::actuator_group)
      .property("actuator_ctrllimited", &MjModel::actuator_ctrllimited)
      .property("actuator_forcelimited", &MjModel::actuator_forcelimited)
      .property("actuator_actlimited", &MjModel::actuator_actlimited)
      .property("actuator_dynprm", &MjModel::actuator_dynprm)
      .property("actuator_gainprm", &MjModel::actuator_gainprm)
      .property("actuator_biasprm", &MjModel::actuator_biasprm)
      .property("actuator_actearly", &MjModel::actuator_actearly)
      .property("actuator_ctrlrange", &MjModel::actuator_ctrlrange)
      .property("actuator_forcerange", &MjModel::actuator_forcerange)
      .property("actuator_actrange", &MjModel::actuator_actrange)
      .property("actuator_gear", &MjModel::actuator_gear)
      .property("actuator_cranklength", &MjModel::actuator_cranklength)
      .property("actuator_acc0", &MjModel::actuator_acc0)
      .property("actuator_length0", &MjModel::actuator_length0)
      .property("actuator_lengthrange", &MjModel::actuator_lengthrange)
      .property("actuator_user", &MjModel::actuator_user)
      .property("actuator_plugin", &MjModel::actuator_plugin)
      .property("sensor_type", &MjModel::sensor_type)
      .property("sensor_datatype", &MjModel::sensor_datatype)
      .property("sensor_needstage", &MjModel::sensor_needstage)
      .property("sensor_objtype", &MjModel::sensor_objtype)
      .property("sensor_objid", &MjModel::sensor_objid)
      .property("sensor_reftype", &MjModel::sensor_reftype)
      .property("sensor_refid", &MjModel::sensor_refid)
      .property("sensor_intprm", &MjModel::sensor_intprm)
      .property("sensor_dim", &MjModel::sensor_dim)
      .property("sensor_adr", &MjModel::sensor_adr)
      .property("sensor_cutoff", &MjModel::sensor_cutoff)
      .property("sensor_noise", &MjModel::sensor_noise)
      .property("sensor_user", &MjModel::sensor_user)
      .property("sensor_plugin", &MjModel::sensor_plugin)
      .property("plugin", &MjModel::plugin)
      .property("plugin_stateadr", &MjModel::plugin_stateadr)
      .property("plugin_statenum", &MjModel::plugin_statenum)
      .property("plugin_attr", &MjModel::plugin_attr)
      .property("plugin_attradr", &MjModel::plugin_attradr)
      .property("numeric_adr", &MjModel::numeric_adr)
      .property("numeric_size", &MjModel::numeric_size)
      .property("numeric_data", &MjModel::numeric_data)
      .property("text_adr", &MjModel::text_adr)
      .property("text_size", &MjModel::text_size)
      .property("text_data", &MjModel::text_data)
      .property("tuple_adr", &MjModel::tuple_adr)
      .property("tuple_size", &MjModel::tuple_size)
      .property("tuple_objtype", &MjModel::tuple_objtype)
      .property("tuple_objid", &MjModel::tuple_objid)
      .property("tuple_objprm", &MjModel::tuple_objprm)
      .property("key_time", &MjModel::key_time)
      .property("key_qpos", &MjModel::key_qpos)
      .property("key_qvel", &MjModel::key_qvel)
      .property("key_act", &MjModel::key_act)
      .property("key_mpos", &MjModel::key_mpos)
      .property("key_mquat", &MjModel::key_mquat)
      .property("key_ctrl", &MjModel::key_ctrl)
      .property("name_bodyadr", &MjModel::name_bodyadr)
      .property("name_jntadr", &MjModel::name_jntadr)
      .property("name_geomadr", &MjModel::name_geomadr)
      .property("name_siteadr", &MjModel::name_siteadr)
      .property("name_camadr", &MjModel::name_camadr)
      .property("name_lightadr", &MjModel::name_lightadr)
      .property("name_flexadr", &MjModel::name_flexadr)
      .property("name_meshadr", &MjModel::name_meshadr)
      .property("name_skinadr", &MjModel::name_skinadr)
      .property("name_hfieldadr", &MjModel::name_hfieldadr)
      .property("name_texadr", &MjModel::name_texadr)
      .property("name_matadr", &MjModel::name_matadr)
      .property("name_pairadr", &MjModel::name_pairadr)
      .property("name_excludeadr", &MjModel::name_excludeadr)
      .property("name_eqadr", &MjModel::name_eqadr)
      .property("name_tendonadr", &MjModel::name_tendonadr)
      .property("name_actuatoradr", &MjModel::name_actuatoradr)
      .property("name_sensoradr", &MjModel::name_sensoradr)
      .property("name_numericadr", &MjModel::name_numericadr)
      .property("name_textadr", &MjModel::name_textadr)
      .property("name_tupleadr", &MjModel::name_tupleadr)
      .property("name_keyadr", &MjModel::name_keyadr)
      .property("name_pluginadr", &MjModel::name_pluginadr)
      .property("names", &MjModel::names)
      .property("names_map", &MjModel::names_map)
      .property("paths", &MjModel::paths)
      .property("B_rownnz", &MjModel::B_rownnz)
      .property("B_rowadr", &MjModel::B_rowadr)
      .property("B_colind", &MjModel::B_colind)
      .property("M_rownnz", &MjModel::M_rownnz)
      .property("M_rowadr", &MjModel::M_rowadr)
      .property("M_colind", &MjModel::M_colind)
      .property("mapM2M", &MjModel::mapM2M)
      .property("D_rownnz", &MjModel::D_rownnz)
      .property("D_rowadr", &MjModel::D_rowadr)
      .property("D_diag", &MjModel::D_diag)
      .property("D_colind", &MjModel::D_colind)
      .property("mapM2D", &MjModel::mapM2D)
      .property("mapD2M", &MjModel::mapD2M)
      .property("signature", &MjModel::signature, &MjModel::set_signature, reference())
      ;
  emscripten::class_<MjData>("MjData")
      .constructor<MjModel *>()
      .constructor<const MjModel &, const MjData &>()
      .property("narena", &MjData::narena, &MjData::set_narena, reference())
      .property("nbuffer", &MjData::nbuffer, &MjData::set_nbuffer, reference())
      .property("nplugin", &MjData::nplugin, &MjData::set_nplugin, reference())
      .property("pstack", &MjData::pstack, &MjData::set_pstack, reference())
      .property("pbase", &MjData::pbase, &MjData::set_pbase, reference())
      .property("parena", &MjData::parena, &MjData::set_parena, reference())
      .property("maxuse_stack", &MjData::maxuse_stack, &MjData::set_maxuse_stack, reference())
      .property("maxuse_threadstack", &MjData::maxuse_threadstack)
      .property("maxuse_arena", &MjData::maxuse_arena, &MjData::set_maxuse_arena, reference())
      .property("maxuse_con", &MjData::maxuse_con, &MjData::set_maxuse_con, reference())
      .property("maxuse_efc", &MjData::maxuse_efc, &MjData::set_maxuse_efc, reference())
      .property("solver", &MjData::solver)
      .property("solver_niter", &MjData::solver_niter)
      .property("solver_nnz", &MjData::solver_nnz)
      .property("solver_fwdinv", &MjData::solver_fwdinv)
      .property("warning", &MjData::warning)
      .property("timer", &MjData::timer)
      .property("ncon", &MjData::ncon, &MjData::set_ncon, reference())
      .property("ne", &MjData::ne, &MjData::set_ne, reference())
      .property("nf", &MjData::nf, &MjData::set_nf, reference())
      .property("nl", &MjData::nl, &MjData::set_nl, reference())
      .property("nefc", &MjData::nefc, &MjData::set_nefc, reference())
      .property("nJ", &MjData::nJ, &MjData::set_nJ, reference())
      .property("nA", &MjData::nA, &MjData::set_nA, reference())
      .property("nisland", &MjData::nisland, &MjData::set_nisland, reference())
      .property("nidof", &MjData::nidof, &MjData::set_nidof, reference())
      .property("time", &MjData::time, &MjData::set_time, reference())
      .property("energy", &MjData::energy)
      .property("buffer", &MjData::buffer)
      .property("arena", &MjData::arena)
      .property("qpos", &MjData::qpos)
      .property("qvel", &MjData::qvel)
      .property("act", &MjData::act)
      .property("qacc_warmstart", &MjData::qacc_warmstart)
      .property("plugin_state", &MjData::plugin_state)
      .property("ctrl", &MjData::ctrl)
      .property("qfrc_applied", &MjData::qfrc_applied)
      .property("xfrc_applied", &MjData::xfrc_applied)
      .property("eq_active", &MjData::eq_active)
      .property("mocap_pos", &MjData::mocap_pos)
      .property("mocap_quat", &MjData::mocap_quat)
      .property("qacc", &MjData::qacc)
      .property("act_dot", &MjData::act_dot)
      .property("userdata", &MjData::userdata)
      .property("sensordata", &MjData::sensordata)
      .property("plugin", &MjData::plugin)
      .property("plugin_data", &MjData::plugin_data)
      .property("xpos", &MjData::xpos)
      .property("xquat", &MjData::xquat)
      .property("xmat", &MjData::xmat)
      .property("xipos", &MjData::xipos)
      .property("ximat", &MjData::ximat)
      .property("xanchor", &MjData::xanchor)
      .property("xaxis", &MjData::xaxis)
      .property("geom_xpos", &MjData::geom_xpos)
      .property("geom_xmat", &MjData::geom_xmat)
      .property("site_xpos", &MjData::site_xpos)
      .property("site_xmat", &MjData::site_xmat)
      .property("cam_xpos", &MjData::cam_xpos)
      .property("cam_xmat", &MjData::cam_xmat)
      .property("light_xpos", &MjData::light_xpos)
      .property("light_xdir", &MjData::light_xdir)
      .property("subtree_com", &MjData::subtree_com)
      .property("cdof", &MjData::cdof)
      .property("cinert", &MjData::cinert)
      .property("flexvert_xpos", &MjData::flexvert_xpos)
      .property("flexelem_aabb", &MjData::flexelem_aabb)
      .property("flexedge_J_rownnz", &MjData::flexedge_J_rownnz)
      .property("flexedge_J_rowadr", &MjData::flexedge_J_rowadr)
      .property("flexedge_J_colind", &MjData::flexedge_J_colind)
      .property("flexedge_J", &MjData::flexedge_J)
      .property("flexedge_length", &MjData::flexedge_length)
      .property("bvh_aabb_dyn", &MjData::bvh_aabb_dyn)
      .property("ten_wrapadr", &MjData::ten_wrapadr)
      .property("ten_wrapnum", &MjData::ten_wrapnum)
      .property("ten_J_rownnz", &MjData::ten_J_rownnz)
      .property("ten_J_rowadr", &MjData::ten_J_rowadr)
      .property("ten_J_colind", &MjData::ten_J_colind)
      .property("ten_J", &MjData::ten_J)
      .property("ten_length", &MjData::ten_length)
      .property("wrap_obj", &MjData::wrap_obj)
      .property("wrap_xpos", &MjData::wrap_xpos)
      .property("actuator_length", &MjData::actuator_length)
      .property("moment_rownnz", &MjData::moment_rownnz)
      .property("moment_rowadr", &MjData::moment_rowadr)
      .property("moment_colind", &MjData::moment_colind)
      .property("actuator_moment", &MjData::actuator_moment)
      .property("crb", &MjData::crb)
      .property("qM", &MjData::qM)
      .property("M", &MjData::M)
      .property("qLD", &MjData::qLD)
      .property("qLDiagInv", &MjData::qLDiagInv)
      .property("bvh_active", &MjData::bvh_active)
      .property("flexedge_velocity", &MjData::flexedge_velocity)
      .property("ten_velocity", &MjData::ten_velocity)
      .property("actuator_velocity", &MjData::actuator_velocity)
      .property("cvel", &MjData::cvel)
      .property("cdof_dot", &MjData::cdof_dot)
      .property("qfrc_bias", &MjData::qfrc_bias)
      .property("qfrc_spring", &MjData::qfrc_spring)
      .property("qfrc_damper", &MjData::qfrc_damper)
      .property("qfrc_gravcomp", &MjData::qfrc_gravcomp)
      .property("qfrc_fluid", &MjData::qfrc_fluid)
      .property("qfrc_passive", &MjData::qfrc_passive)
      .property("subtree_linvel", &MjData::subtree_linvel)
      .property("subtree_angmom", &MjData::subtree_angmom)
      .property("qH", &MjData::qH)
      .property("qHDiagInv", &MjData::qHDiagInv)
      .property("qDeriv", &MjData::qDeriv)
      .property("qLU", &MjData::qLU)
      .property("actuator_force", &MjData::actuator_force)
      .property("qfrc_actuator", &MjData::qfrc_actuator)
      .property("qfrc_smooth", &MjData::qfrc_smooth)
      .property("qacc_smooth", &MjData::qacc_smooth)
      .property("qfrc_constraint", &MjData::qfrc_constraint)
      .property("qfrc_inverse", &MjData::qfrc_inverse)
      .property("cacc", &MjData::cacc)
      .property("cfrc_int", &MjData::cfrc_int)
      .property("cfrc_ext", &MjData::cfrc_ext)
      .property("contact", &MjData::contact)
      .property("efc_type", &MjData::efc_type)
      .property("efc_id", &MjData::efc_id)
      .property("efc_J_rownnz", &MjData::efc_J_rownnz)
      .property("efc_J_rowadr", &MjData::efc_J_rowadr)
      .property("efc_J_rowsuper", &MjData::efc_J_rowsuper)
      .property("efc_J_colind", &MjData::efc_J_colind)
      .property("efc_J", &MjData::efc_J)
      .property("efc_pos", &MjData::efc_pos)
      .property("efc_margin", &MjData::efc_margin)
      .property("efc_frictionloss", &MjData::efc_frictionloss)
      .property("efc_diagApprox", &MjData::efc_diagApprox)
      .property("efc_KBIP", &MjData::efc_KBIP)
      .property("efc_D", &MjData::efc_D)
      .property("efc_R", &MjData::efc_R)
      .property("tendon_efcadr", &MjData::tendon_efcadr)
      .property("dof_island", &MjData::dof_island)
      .property("island_nv", &MjData::island_nv)
      .property("island_idofadr", &MjData::island_idofadr)
      .property("island_dofadr", &MjData::island_dofadr)
      .property("map_dof2idof", &MjData::map_dof2idof)
      .property("map_idof2dof", &MjData::map_idof2dof)
      .property("ifrc_smooth", &MjData::ifrc_smooth)
      .property("iacc_smooth", &MjData::iacc_smooth)
      .property("iM_rownnz", &MjData::iM_rownnz)
      .property("iM_rowadr", &MjData::iM_rowadr)
      .property("iM_colind", &MjData::iM_colind)
      .property("iM", &MjData::iM)
      .property("iLD", &MjData::iLD)
      .property("iLDiagInv", &MjData::iLDiagInv)
      .property("iacc", &MjData::iacc)
      .property("efc_island", &MjData::efc_island)
      .property("island_ne", &MjData::island_ne)
      .property("island_nf", &MjData::island_nf)
      .property("island_nefc", &MjData::island_nefc)
      .property("island_iefcadr", &MjData::island_iefcadr)
      .property("map_efc2iefc", &MjData::map_efc2iefc)
      .property("map_iefc2efc", &MjData::map_iefc2efc)
      .property("iefc_type", &MjData::iefc_type)
      .property("iefc_id", &MjData::iefc_id)
      .property("iefc_J_rownnz", &MjData::iefc_J_rownnz)
      .property("iefc_J_rowadr", &MjData::iefc_J_rowadr)
      .property("iefc_J_rowsuper", &MjData::iefc_J_rowsuper)
      .property("iefc_J_colind", &MjData::iefc_J_colind)
      .property("iefc_J", &MjData::iefc_J)
      .property("iefc_frictionloss", &MjData::iefc_frictionloss)
      .property("iefc_D", &MjData::iefc_D)
      .property("iefc_R", &MjData::iefc_R)
      .property("efc_AR_rownnz", &MjData::efc_AR_rownnz)
      .property("efc_AR_rowadr", &MjData::efc_AR_rowadr)
      .property("efc_AR_colind", &MjData::efc_AR_colind)
      .property("efc_AR", &MjData::efc_AR)
      .property("efc_vel", &MjData::efc_vel)
      .property("efc_aref", &MjData::efc_aref)
      .property("efc_b", &MjData::efc_b)
      .property("iefc_aref", &MjData::iefc_aref)
      .property("iefc_state", &MjData::iefc_state)
      .property("iefc_force", &MjData::iefc_force)
      .property("efc_state", &MjData::efc_state)
      .property("efc_force", &MjData::efc_force)
      .property("ifrc_constraint", &MjData::ifrc_constraint)
      .property("threadpool", &MjData::threadpool, &MjData::set_threadpool, reference())
      .property("signature", &MjData::signature, &MjData::set_signature, reference())
      ;
  emscripten::class_<MjOption>("MjOption")
      .constructor<>()
      .function("copy", &MjOption::copy, take_ownership())
      .property("timestep", &MjOption::timestep, &MjOption::set_timestep, reference())
      .property("impratio", &MjOption::impratio, &MjOption::set_impratio, reference())
      .property("tolerance", &MjOption::tolerance, &MjOption::set_tolerance, reference())
      .property("ls_tolerance", &MjOption::ls_tolerance, &MjOption::set_ls_tolerance, reference())
      .property("noslip_tolerance", &MjOption::noslip_tolerance, &MjOption::set_noslip_tolerance, reference())
      .property("ccd_tolerance", &MjOption::ccd_tolerance, &MjOption::set_ccd_tolerance, reference())
      .property("gravity", &MjOption::gravity)
      .property("wind", &MjOption::wind)
      .property("magnetic", &MjOption::magnetic)
      .property("density", &MjOption::density, &MjOption::set_density, reference())
      .property("viscosity", &MjOption::viscosity, &MjOption::set_viscosity, reference())
      .property("o_margin", &MjOption::o_margin, &MjOption::set_o_margin, reference())
      .property("o_solref", &MjOption::o_solref)
      .property("o_solimp", &MjOption::o_solimp)
      .property("o_friction", &MjOption::o_friction)
      .property("integrator", &MjOption::integrator, &MjOption::set_integrator, reference())
      .property("cone", &MjOption::cone, &MjOption::set_cone, reference())
      .property("jacobian", &MjOption::jacobian, &MjOption::set_jacobian, reference())
      .property("solver", &MjOption::solver, &MjOption::set_solver, reference())
      .property("iterations", &MjOption::iterations, &MjOption::set_iterations, reference())
      .property("ls_iterations", &MjOption::ls_iterations, &MjOption::set_ls_iterations, reference())
      .property("noslip_iterations", &MjOption::noslip_iterations, &MjOption::set_noslip_iterations, reference())
      .property("ccd_iterations", &MjOption::ccd_iterations, &MjOption::set_ccd_iterations, reference())
      .property("disableflags", &MjOption::disableflags, &MjOption::set_disableflags, reference())
      .property("enableflags", &MjOption::enableflags, &MjOption::set_enableflags, reference())
      .property("disableactuator", &MjOption::disableactuator, &MjOption::set_disableactuator, reference())
      .property("sdf_initpoints", &MjOption::sdf_initpoints, &MjOption::set_sdf_initpoints, reference())
      .property("sdf_iterations", &MjOption::sdf_iterations, &MjOption::set_sdf_iterations, reference())
      ;
  emscripten::class_<MjStatistic>("MjStatistic")
      .constructor<>()
      .function("copy", &MjStatistic::copy, take_ownership())
      .property("meaninertia", &MjStatistic::meaninertia, &MjStatistic::set_meaninertia, reference())
      .property("meanmass", &MjStatistic::meanmass, &MjStatistic::set_meanmass, reference())
      .property("meansize", &MjStatistic::meansize, &MjStatistic::set_meansize, reference())
      .property("extent", &MjStatistic::extent, &MjStatistic::set_extent, reference())
      .property("center", &MjStatistic::center)
      ;
  emscripten::class_<MjVisualGlobal>("MjVisualGlobal")
      .constructor<>()
      .function("copy", &MjVisualGlobal::copy, take_ownership())
      .property("cameraid", &MjVisualGlobal::cameraid, &MjVisualGlobal::set_cameraid, reference())
      .property("orthographic", &MjVisualGlobal::orthographic, &MjVisualGlobal::set_orthographic, reference())
      .property("fovy", &MjVisualGlobal::fovy, &MjVisualGlobal::set_fovy, reference())
      .property("ipd", &MjVisualGlobal::ipd, &MjVisualGlobal::set_ipd, reference())
      .property("azimuth", &MjVisualGlobal::azimuth, &MjVisualGlobal::set_azimuth, reference())
      .property("elevation", &MjVisualGlobal::elevation, &MjVisualGlobal::set_elevation, reference())
      .property("linewidth", &MjVisualGlobal::linewidth, &MjVisualGlobal::set_linewidth, reference())
      .property("glow", &MjVisualGlobal::glow, &MjVisualGlobal::set_glow, reference())
      .property("realtime", &MjVisualGlobal::realtime, &MjVisualGlobal::set_realtime, reference())
      .property("offwidth", &MjVisualGlobal::offwidth, &MjVisualGlobal::set_offwidth, reference())
      .property("offheight", &MjVisualGlobal::offheight, &MjVisualGlobal::set_offheight, reference())
      .property("ellipsoidinertia", &MjVisualGlobal::ellipsoidinertia, &MjVisualGlobal::set_ellipsoidinertia, reference())
      .property("bvactive", &MjVisualGlobal::bvactive, &MjVisualGlobal::set_bvactive, reference())
      ;
  emscripten::class_<MjVisualQuality>("MjVisualQuality")
      .constructor<>()
      .function("copy", &MjVisualQuality::copy, take_ownership())
      .property("shadowsize", &MjVisualQuality::shadowsize, &MjVisualQuality::set_shadowsize, reference())
      .property("offsamples", &MjVisualQuality::offsamples, &MjVisualQuality::set_offsamples, reference())
      .property("numslices", &MjVisualQuality::numslices, &MjVisualQuality::set_numslices, reference())
      .property("numstacks", &MjVisualQuality::numstacks, &MjVisualQuality::set_numstacks, reference())
      .property("numquads", &MjVisualQuality::numquads, &MjVisualQuality::set_numquads, reference())
      ;
  emscripten::class_<MjVisualHeadlight>("MjVisualHeadlight")
      .constructor<>()
      .function("copy", &MjVisualHeadlight::copy, take_ownership())
      .property("ambient", &MjVisualHeadlight::ambient)
      .property("diffuse", &MjVisualHeadlight::diffuse)
      .property("specular", &MjVisualHeadlight::specular)
      .property("active", &MjVisualHeadlight::active, &MjVisualHeadlight::set_active, reference())
      ;
  emscripten::class_<MjVisualMap>("MjVisualMap")
      .constructor<>()
      .function("copy", &MjVisualMap::copy, take_ownership())
      .property("stiffness", &MjVisualMap::stiffness, &MjVisualMap::set_stiffness, reference())
      .property("stiffnessrot", &MjVisualMap::stiffnessrot, &MjVisualMap::set_stiffnessrot, reference())
      .property("force", &MjVisualMap::force, &MjVisualMap::set_force, reference())
      .property("torque", &MjVisualMap::torque, &MjVisualMap::set_torque, reference())
      .property("alpha", &MjVisualMap::alpha, &MjVisualMap::set_alpha, reference())
      .property("fogstart", &MjVisualMap::fogstart, &MjVisualMap::set_fogstart, reference())
      .property("fogend", &MjVisualMap::fogend, &MjVisualMap::set_fogend, reference())
      .property("znear", &MjVisualMap::znear, &MjVisualMap::set_znear, reference())
      .property("zfar", &MjVisualMap::zfar, &MjVisualMap::set_zfar, reference())
      .property("haze", &MjVisualMap::haze, &MjVisualMap::set_haze, reference())
      .property("shadowclip", &MjVisualMap::shadowclip, &MjVisualMap::set_shadowclip, reference())
      .property("shadowscale", &MjVisualMap::shadowscale, &MjVisualMap::set_shadowscale, reference())
      .property("actuatortendon", &MjVisualMap::actuatortendon, &MjVisualMap::set_actuatortendon, reference())
      ;
  emscripten::class_<MjVisualScale>("MjVisualScale")
      .constructor<>()
      .function("copy", &MjVisualScale::copy, take_ownership())
      .property("forcewidth", &MjVisualScale::forcewidth, &MjVisualScale::set_forcewidth, reference())
      .property("contactwidth", &MjVisualScale::contactwidth, &MjVisualScale::set_contactwidth, reference())
      .property("contactheight", &MjVisualScale::contactheight, &MjVisualScale::set_contactheight, reference())
      .property("connect", &MjVisualScale::connect, &MjVisualScale::set_connect, reference())
      .property("com", &MjVisualScale::com, &MjVisualScale::set_com, reference())
      .property("camera", &MjVisualScale::camera, &MjVisualScale::set_camera, reference())
      .property("light", &MjVisualScale::light, &MjVisualScale::set_light, reference())
      .property("selectpoint", &MjVisualScale::selectpoint, &MjVisualScale::set_selectpoint, reference())
      .property("jointlength", &MjVisualScale::jointlength, &MjVisualScale::set_jointlength, reference())
      .property("jointwidth", &MjVisualScale::jointwidth, &MjVisualScale::set_jointwidth, reference())
      .property("actuatorlength", &MjVisualScale::actuatorlength, &MjVisualScale::set_actuatorlength, reference())
      .property("actuatorwidth", &MjVisualScale::actuatorwidth, &MjVisualScale::set_actuatorwidth, reference())
      .property("framelength", &MjVisualScale::framelength, &MjVisualScale::set_framelength, reference())
      .property("framewidth", &MjVisualScale::framewidth, &MjVisualScale::set_framewidth, reference())
      .property("constraint", &MjVisualScale::constraint, &MjVisualScale::set_constraint, reference())
      .property("slidercrank", &MjVisualScale::slidercrank, &MjVisualScale::set_slidercrank, reference())
      .property("frustum", &MjVisualScale::frustum, &MjVisualScale::set_frustum, reference())
      ;
  emscripten::class_<MjVisualRgba>("MjVisualRgba")
      .constructor<>()
      .function("copy", &MjVisualRgba::copy, take_ownership())
      .property("fog", &MjVisualRgba::fog)
      .property("haze", &MjVisualRgba::haze)
      .property("force", &MjVisualRgba::force)
      .property("inertia", &MjVisualRgba::inertia)
      .property("joint", &MjVisualRgba::joint)
      .property("actuator", &MjVisualRgba::actuator)
      .property("actuatornegative", &MjVisualRgba::actuatornegative)
      .property("actuatorpositive", &MjVisualRgba::actuatorpositive)
      .property("com", &MjVisualRgba::com)
      .property("camera", &MjVisualRgba::camera)
      .property("light", &MjVisualRgba::light)
      .property("selectpoint", &MjVisualRgba::selectpoint)
      .property("connect", &MjVisualRgba::connect)
      .property("contactpoint", &MjVisualRgba::contactpoint)
      .property("contactforce", &MjVisualRgba::contactforce)
      .property("contactfriction", &MjVisualRgba::contactfriction)
      .property("contacttorque", &MjVisualRgba::contacttorque)
      .property("contactgap", &MjVisualRgba::contactgap)
      .property("rangefinder", &MjVisualRgba::rangefinder)
      .property("constraint", &MjVisualRgba::constraint)
      .property("slidercrank", &MjVisualRgba::slidercrank)
      .property("crankbroken", &MjVisualRgba::crankbroken)
      .property("frustum", &MjVisualRgba::frustum)
      .property("bv", &MjVisualRgba::bv)
      .property("bvactive", &MjVisualRgba::bvactive)
      ;
  emscripten::class_<MjVisual>("MjVisual")
      .constructor<>()
      .function("copy", &MjVisual::copy, take_ownership())
      .property("global", &MjVisual::global, reference())
      .property("quality", &MjVisual::quality, reference())
      .property("headlight", &MjVisual::headlight, reference())
      .property("map", &MjVisual::map, reference())
      .property("scale", &MjVisual::scale, reference())
      .property("rgba", &MjVisual::rgba, reference())
      ;
  emscripten::class_<MjSolverStat>("MjSolverStat")
      .constructor<>()
      .function("copy", &MjSolverStat::copy, take_ownership())
      .property("improvement", &MjSolverStat::improvement, &MjSolverStat::set_improvement, reference())
      .property("gradient", &MjSolverStat::gradient, &MjSolverStat::set_gradient, reference())
      .property("lineslope", &MjSolverStat::lineslope, &MjSolverStat::set_lineslope, reference())
      .property("nactive", &MjSolverStat::nactive, &MjSolverStat::set_nactive, reference())
      .property("nchange", &MjSolverStat::nchange, &MjSolverStat::set_nchange, reference())
      .property("neval", &MjSolverStat::neval, &MjSolverStat::set_neval, reference())
      .property("nupdate", &MjSolverStat::nupdate, &MjSolverStat::set_nupdate, reference())
      ;
  emscripten::class_<MjTimerStat>("MjTimerStat")
      .constructor<>()
      .function("copy", &MjTimerStat::copy, take_ownership())
      .property("duration", &MjTimerStat::duration, &MjTimerStat::set_duration, reference())
      .property("number", &MjTimerStat::number, &MjTimerStat::set_number, reference())
      ;
  emscripten::class_<MjWarningStat>("MjWarningStat")
      .constructor<>()
      .function("copy", &MjWarningStat::copy, take_ownership())
      .property("lastinfo", &MjWarningStat::lastinfo, &MjWarningStat::set_lastinfo, reference())
      .property("number", &MjWarningStat::number, &MjWarningStat::set_number, reference())
      ;
  emscripten::class_<MjContact>("MjContact")
      .constructor<>()
      .function("copy", &MjContact::copy, take_ownership())
      .property("dist", &MjContact::dist, &MjContact::set_dist, reference())
      .property("pos", &MjContact::pos)
      .property("frame", &MjContact::frame)
      .property("includemargin", &MjContact::includemargin, &MjContact::set_includemargin, reference())
      .property("friction", &MjContact::friction)
      .property("solref", &MjContact::solref)
      .property("solreffriction", &MjContact::solreffriction)
      .property("solimp", &MjContact::solimp)
      .property("mu", &MjContact::mu, &MjContact::set_mu, reference())
      .property("H", &MjContact::H)
      .property("dim", &MjContact::dim, &MjContact::set_dim, reference())
      .property("geom1", &MjContact::geom1, &MjContact::set_geom1, reference())
      .property("geom2", &MjContact::geom2, &MjContact::set_geom2, reference())
      .property("geom", &MjContact::geom)
      .property("flex", &MjContact::flex)
      .property("elem", &MjContact::elem)
      .property("vert", &MjContact::vert)
      .property("exclude", &MjContact::exclude, &MjContact::set_exclude, reference())
      .property("efc_address", &MjContact::efc_address, &MjContact::set_efc_address, reference())
      ;
  emscripten::class_<MjvPerturb>("MjvPerturb")
      .constructor<>()
      .function("copy", &MjvPerturb::copy, take_ownership())
      .property("select", &MjvPerturb::select, &MjvPerturb::set_select, reference())
      .property("flexselect", &MjvPerturb::flexselect, &MjvPerturb::set_flexselect, reference())
      .property("skinselect", &MjvPerturb::skinselect, &MjvPerturb::set_skinselect, reference())
      .property("active", &MjvPerturb::active, &MjvPerturb::set_active, reference())
      .property("active2", &MjvPerturb::active2, &MjvPerturb::set_active2, reference())
      .property("refpos", &MjvPerturb::refpos)
      .property("refquat", &MjvPerturb::refquat)
      .property("refselpos", &MjvPerturb::refselpos)
      .property("localpos", &MjvPerturb::localpos)
      .property("localmass", &MjvPerturb::localmass, &MjvPerturb::set_localmass, reference())
      .property("scale", &MjvPerturb::scale, &MjvPerturb::set_scale, reference())
      ;
  emscripten::class_<MjvCamera>("MjvCamera")
      .constructor<>()
      .function("copy", &MjvCamera::copy, take_ownership())
      .property("type", &MjvCamera::type, &MjvCamera::set_type, reference())
      .property("fixedcamid", &MjvCamera::fixedcamid, &MjvCamera::set_fixedcamid, reference())
      .property("trackbodyid", &MjvCamera::trackbodyid, &MjvCamera::set_trackbodyid, reference())
      .property("lookat", &MjvCamera::lookat)
      .property("distance", &MjvCamera::distance, &MjvCamera::set_distance, reference())
      .property("azimuth", &MjvCamera::azimuth, &MjvCamera::set_azimuth, reference())
      .property("elevation", &MjvCamera::elevation, &MjvCamera::set_elevation, reference())
      .property("orthographic", &MjvCamera::orthographic, &MjvCamera::set_orthographic, reference())
      ;
  emscripten::class_<MjvGLCamera>("MjvGLCamera")
      .constructor<>()
      .function("copy", &MjvGLCamera::copy, take_ownership())
      .property("pos", &MjvGLCamera::pos)
      .property("forward", &MjvGLCamera::forward)
      .property("up", &MjvGLCamera::up)
      .property("frustum_center", &MjvGLCamera::frustum_center, &MjvGLCamera::set_frustum_center, reference())
      .property("frustum_width", &MjvGLCamera::frustum_width, &MjvGLCamera::set_frustum_width, reference())
      .property("frustum_bottom", &MjvGLCamera::frustum_bottom, &MjvGLCamera::set_frustum_bottom, reference())
      .property("frustum_top", &MjvGLCamera::frustum_top, &MjvGLCamera::set_frustum_top, reference())
      .property("frustum_near", &MjvGLCamera::frustum_near, &MjvGLCamera::set_frustum_near, reference())
      .property("frustum_far", &MjvGLCamera::frustum_far, &MjvGLCamera::set_frustum_far, reference())
      .property("orthographic", &MjvGLCamera::orthographic, &MjvGLCamera::set_orthographic, reference())
      ;
  emscripten::class_<MjvGeom>("MjvGeom")
      .constructor<>()
      .function("copy", &MjvGLCamera::copy, take_ownership())
      .property("type", &MjvGeom::type, &MjvGeom::set_type, reference())
      .property("dataid", &MjvGeom::dataid, &MjvGeom::set_dataid, reference())
      .property("objtype", &MjvGeom::objtype, &MjvGeom::set_objtype, reference())
      .property("objid", &MjvGeom::objid, &MjvGeom::set_objid, reference())
      .property("category", &MjvGeom::category, &MjvGeom::set_category, reference())
      .property("matid", &MjvGeom::matid, &MjvGeom::set_matid, reference())
      .property("texcoord", &MjvGeom::texcoord, &MjvGeom::set_texcoord, reference())
      .property("segid", &MjvGeom::segid, &MjvGeom::set_segid, reference())
      .property("size", &MjvGeom::size)
      .property("pos", &MjvGeom::pos)
      .property("mat", &MjvGeom::mat)
      .property("rgba", &MjvGeom::rgba)
      .property("emission", &MjvGeom::emission, &MjvGeom::set_emission, reference())
      .property("specular", &MjvGeom::specular, &MjvGeom::set_specular, reference())
      .property("shininess", &MjvGeom::shininess, &MjvGeom::set_shininess, reference())
      .property("reflectance", &MjvGeom::reflectance, &MjvGeom::set_reflectance, reference())
      .property("label", &MjvGeom::label)
      .property("camdist", &MjvGeom::camdist, &MjvGeom::set_camdist, reference())
      .property("modelrbound", &MjvGeom::modelrbound, &MjvGeom::set_modelrbound, reference())
      .property("transparent", &MjvGeom::transparent, &MjvGeom::set_transparent, reference())
      ;
  emscripten::class_<MjvLight>("MjvLight")
      .constructor<>()
      .function("copy", &MjvLight::copy, take_ownership())
      .property("id", &MjvLight::id, &MjvLight::set_id, reference())
      .property("pos", &MjvLight::pos)
      .property("dir", &MjvLight::dir)
      .property("type", &MjvLight::type, &MjvLight::set_type, reference())
      .property("texid", &MjvLight::texid, &MjvLight::set_texid, reference())
      .property("attenuation", &MjvLight::attenuation)
      .property("cutoff", &MjvLight::cutoff, &MjvLight::set_cutoff, reference())
      .property("exponent", &MjvLight::exponent, &MjvLight::set_exponent, reference())
      .property("ambient", &MjvLight::ambient)
      .property("diffuse", &MjvLight::diffuse)
      .property("specular", &MjvLight::specular)
      .property("headlight", &MjvLight::headlight, &MjvLight::set_headlight, reference())
      .property("castshadow", &MjvLight::castshadow, &MjvLight::set_castshadow, reference())
      .property("bulbradius", &MjvLight::bulbradius, &MjvLight::set_bulbradius, reference())
      .property("intensity", &MjvLight::intensity, &MjvLight::set_intensity, reference())
      .property("range", &MjvLight::range, &MjvLight::set_range, reference())
      ;
  emscripten::class_<MjvOption>("MjvOption")
      .constructor<>()
      .function("copy", &MjvOption::copy, take_ownership())
      .property("label", &MjvOption::label, &MjvOption::set_label, reference())
      .property("frame", &MjvOption::frame, &MjvOption::set_frame, reference())
      .property("geomgroup", &MjvOption::geomgroup)
      .property("sitegroup", &MjvOption::sitegroup)
      .property("jointgroup", &MjvOption::jointgroup)
      .property("tendongroup", &MjvOption::tendongroup)
      .property("actuatorgroup", &MjvOption::actuatorgroup)
      .property("flexgroup", &MjvOption::flexgroup)
      .property("skingroup", &MjvOption::skingroup)
      .property("flags", &MjvOption::flags)
      .property("bvh_depth", &MjvOption::bvh_depth, &MjvOption::set_bvh_depth, reference())
      .property("flex_layer", &MjvOption::flex_layer, &MjvOption::set_flex_layer, reference())
      ;

  emscripten::class_<MjvScene>("MjvScene")
      .constructor<>()
      .constructor<MjModel *, int>()
      .property("maxgeom", &MjvScene::maxgeom, &MjvScene::set_maxgeom, reference())
      .property("ngeom", &MjvScene::ngeom, &MjvScene::set_ngeom, reference())
      .property("geoms", &MjvScene::geoms)
      .property("geomorder", &MjvScene::geomorder)
      .property("nflex", &MjvScene::nflex, &MjvScene::set_nflex, reference())
      .property("flexedgeadr", &MjvScene::flexedgeadr)
      .property("flexedgenum", &MjvScene::flexedgenum)
      .property("flexvertadr", &MjvScene::flexvertadr)
      .property("flexvertnum", &MjvScene::flexvertnum)
      .property("flexfaceadr", &MjvScene::flexfaceadr)
      .property("flexfacenum", &MjvScene::flexfacenum)
      .property("flexfaceused", &MjvScene::flexfaceused)
      .property("flexedge", &MjvScene::flexedge)
      .property("flexvert", &MjvScene::flexvert)
      .property("flexface", &MjvScene::flexface)
      .property("flexnormal", &MjvScene::flexnormal)
      .property("flextexcoord", &MjvScene::flextexcoord)
      .property("flexvertopt", &MjvScene::flexvertopt, &MjvScene::set_flexvertopt, reference())
      .property("flexedgeopt", &MjvScene::flexedgeopt, &MjvScene::set_flexedgeopt, reference())
      .property("flexfaceopt", &MjvScene::flexfaceopt, &MjvScene::set_flexfaceopt, reference())
      .property("flexskinopt", &MjvScene::flexskinopt, &MjvScene::set_flexskinopt, reference())
      .property("nskin", &MjvScene::nskin, &MjvScene::set_nskin, reference())
      .property("skinfacenum", &MjvScene::skinfacenum)
      .property("skinvertadr", &MjvScene::skinvertadr)
      .property("skinvertnum", &MjvScene::skinvertnum)
      .property("skinvert", &MjvScene::skinvert)
      .property("skinnormal", &MjvScene::skinnormal)
      .property("nlight", &MjvScene::nlight, &MjvScene::set_nlight, reference())
      .property("lights", &MjvScene::lights)
      .property("camera", &MjvScene::camera)
      .property("enabletransform", &MjvScene::enabletransform, &MjvScene::set_enabletransform, reference())
      .property("translate", &MjvScene::translate)
      .property("rotate", &MjvScene::rotate)
      .property("scale", &MjvScene::scale, &MjvScene::set_scale, reference())
      .property("stereo", &MjvScene::stereo, &MjvScene::set_stereo, reference())
      .property("flags", &MjvScene::flags)
      .property("framewidth", &MjvScene::framewidth, &MjvScene::set_framewidth, reference())
      .property("framergb", &MjvScene::framergb)
      .property("status", &MjvScene::status, &MjvScene::set_status, reference())
      ;

  emscripten::class_<MjvFigure>("MjvFigure")
      .constructor<>()
      .function("copy", &MjvFigure::copy, take_ownership())
      .property("flg_legend", &MjvFigure::flg_legend, &MjvFigure::set_flg_legend, reference())
      .property("flg_ticklabel", &MjvFigure::flg_ticklabel)
      .property("flg_extend", &MjvFigure::flg_extend, &MjvFigure::set_flg_extend, reference())
      .property("flg_barplot", &MjvFigure::flg_barplot, &MjvFigure::set_flg_barplot, reference())
      .property("flg_selection", &MjvFigure::flg_selection, &MjvFigure::set_flg_selection, reference())
      .property("flg_symmetric", &MjvFigure::flg_symmetric, &MjvFigure::set_flg_symmetric, reference())
      .property("linewidth", &MjvFigure::linewidth, &MjvFigure::set_linewidth, reference())
      .property("gridwidth", &MjvFigure::gridwidth, &MjvFigure::set_gridwidth, reference())
      .property("gridsize", &MjvFigure::gridsize)
      .property("gridrgb", &MjvFigure::gridrgb)
      .property("figurergba", &MjvFigure::figurergba)
      .property("panergba", &MjvFigure::panergba)
      .property("legendrgba", &MjvFigure::legendrgba)
      .property("textrgb", &MjvFigure::textrgb)
      .property("linergb", &MjvFigure::linergb)
      .property("range", &MjvFigure::range)
      .property("xformat", &MjvFigure::xformat)
      .property("yformat", &MjvFigure::yformat)
      .property("minwidth", &MjvFigure::minwidth)
      .property("title", &MjvFigure::title)
      .property("xlabel", &MjvFigure::xlabel)
      .property("linename", &MjvFigure::linename)
      .property("legendoffset", &MjvFigure::legendoffset, &MjvFigure::set_legendoffset, reference())
      .property("subplot", &MjvFigure::subplot, &MjvFigure::set_subplot, reference())
      .property("highlight", &MjvFigure::highlight)
      .property("highlightid", &MjvFigure::highlightid, &MjvFigure::set_highlightid, reference())
      .property("selection", &MjvFigure::selection, &MjvFigure::set_selection, reference())
      .property("linepnt", &MjvFigure::linepnt)
      .property("linedata", &MjvFigure::linedata)
      .property("xaxispixel", &MjvFigure::xaxispixel)
      .property("yaxispixel", &MjvFigure::yaxispixel)
      .property("xaxisdata", &MjvFigure::xaxisdata)
      .property("yaxisdata", &MjvFigure::yaxisdata)
      ;

  emscripten::class_<MjSpec>("MjSpec")
      .constructor<const MjSpec &>()
      .property("element", &MjSpec::element)
      .property("modelname", &MjSpec::modelname, &MjSpec::set_modelname, reference())
      .property("compiler", &MjSpec::compiler)
      .property("strippath", &MjSpec::strippath, &MjSpec::set_strippath, reference())
      .property("option", &MjSpec::option)
      .property("visual", &MjSpec::visual)
      .property("stat", &MjSpec::stat)
      .property("memory", &MjSpec::memory, &MjSpec::set_memory, reference())
      .property("nemax", &MjSpec::nemax, &MjSpec::set_nemax, reference())
      .property("nuserdata", &MjSpec::nuserdata, &MjSpec::set_nuserdata, reference())
      .property("nuser_body", &MjSpec::nuser_body, &MjSpec::set_nuser_body, reference())
      .property("nuser_jnt", &MjSpec::nuser_jnt, &MjSpec::set_nuser_jnt, reference())
      .property("nuser_geom", &MjSpec::nuser_geom, &MjSpec::set_nuser_geom, reference())
      .property("nuser_site", &MjSpec::nuser_site, &MjSpec::set_nuser_site, reference())
      .property("nuser_cam", &MjSpec::nuser_cam, &MjSpec::set_nuser_cam, reference())
      .property("nuser_tendon", &MjSpec::nuser_tendon, &MjSpec::set_nuser_tendon, reference())
      .property("nuser_actuator", &MjSpec::nuser_actuator, &MjSpec::set_nuser_actuator, reference())
      .property("nuser_sensor", &MjSpec::nuser_sensor, &MjSpec::set_nuser_sensor, reference())
      .property("nkey", &MjSpec::nkey, &MjSpec::set_nkey, reference())
      .property("njmax", &MjSpec::njmax, &MjSpec::set_njmax, reference())
      .property("nconmax", &MjSpec::nconmax, &MjSpec::set_nconmax, reference())
      .property("nstack", &MjSpec::nstack, &MjSpec::set_nstack, reference())
      .property("comment", &MjSpec::comment, &MjSpec::set_comment, reference())
      .property("modelfiledir", &MjSpec::modelfiledir, &MjSpec::set_modelfiledir, reference())
      .property("hasImplicitPluginElem", &MjSpec::hasImplicitPluginElem, &MjSpec::set_hasImplicitPluginElem, reference())
      ;

  emscripten::class_<MjsElement>("MjsElement")
      .property("elemtype", &MjsElement::elemtype, &MjsElement::set_elemtype, reference())
      .property("signature", &MjsElement::signature, &MjsElement::set_signature, reference())
      ;

  emscripten::class_<MjsCompiler>("MjsCompiler")
      .property("autolimits", &MjsCompiler::autolimits, &MjsCompiler::set_autolimits, reference())
      .property("boundmass", &MjsCompiler::boundmass, &MjsCompiler::set_boundmass, reference())
      .property("boundinertia", &MjsCompiler::boundinertia, &MjsCompiler::set_boundinertia, reference())
      .property("settotalmass", &MjsCompiler::settotalmass, &MjsCompiler::set_settotalmass, reference())
      .property("balanceinertia", &MjsCompiler::balanceinertia, &MjsCompiler::set_balanceinertia, reference())
      .property("fitaabb", &MjsCompiler::fitaabb, &MjsCompiler::set_fitaabb, reference())
      .property("degree", &MjsCompiler::degree, &MjsCompiler::set_degree, reference())
      .property("eulerseq", &MjsCompiler::eulerseq)
      .property("discardvisual", &MjsCompiler::discardvisual, &MjsCompiler::set_discardvisual, reference())
      .property("usethread", &MjsCompiler::usethread, &MjsCompiler::set_usethread, reference())
      .property("fusestatic", &MjsCompiler::fusestatic, &MjsCompiler::set_fusestatic, reference())
      .property("inertiafromgeom", &MjsCompiler::inertiafromgeom, &MjsCompiler::set_inertiafromgeom, reference())
      .property("inertiagrouprange", &MjsCompiler::inertiagrouprange)
      .property("saveinertial", &MjsCompiler::saveinertial, &MjsCompiler::set_saveinertial, reference())
      .property("alignfree", &MjsCompiler::alignfree, &MjsCompiler::set_alignfree, reference())
      .property("LRopt", &MjsCompiler::LRopt, reference())
      .property("meshdir", &MjsCompiler::meshdir, &MjsCompiler::set_meshdir, reference())
      .property("texturedir", &MjsCompiler::texturedir, &MjsCompiler::set_texturedir, reference())
      ;

  emscripten::class_<MjsOrientation>("MjsOrientation")
      .function("copy", &MjsOrientation::copy, take_ownership())
      .property("type", &MjsOrientation::type, &MjsOrientation::set_type, reference())
      .property("axisangle", &MjsOrientation::axisangle)
      .property("xyaxes", &MjsOrientation::xyaxes)
      .property("zaxis", &MjsOrientation::zaxis)
      .property("euler", &MjsOrientation::euler)
      ;

  emscripten::class_<MjsBody>("MjsBody")
      .property("element", &MjsBody::element, reference())
      .property("childclass", &MjsBody::childclass, &MjsBody::set_childclass, reference())
      .property("pos", &MjsBody::pos)
      .property("quat", &MjsBody::quat)
      .property("alt", &MjsBody::alt, reference())
      .property("mass", &MjsBody::mass, &MjsBody::set_mass, reference())
      .property("ipos", &MjsBody::ipos)
      .property("iquat", &MjsBody::iquat)
      .property("inertia", &MjsBody::inertia)
      .property("ialt", &MjsBody::ialt, reference())
      .property("fullinertia", &MjsBody::fullinertia)
      .property("mocap", &MjsBody::mocap, &MjsBody::set_mocap, reference())
      .property("gravcomp", &MjsBody::gravcomp, &MjsBody::set_gravcomp, reference())
      .property("userdata", &MjsBody::userdata, reference())
      .property("explicitinertial", &MjsBody::explicitinertial, &MjsBody::set_explicitinertial, reference())
      .property("plugin", &MjsBody::plugin, reference())
      .property("info", &MjsBody::info, &MjsBody::set_info, reference())
      ;

  emscripten::class_<MjsGeom>("MjsGeom")
      .property("element", &MjsGeom::element, reference())
      .property("type", &MjsGeom::type, &MjsGeom::set_type, reference())
      .property("pos", &MjsGeom::pos)
      .property("quat", &MjsGeom::quat)
      .property("alt", &MjsGeom::alt, reference())
      .property("fromto", &MjsGeom::fromto)
      .property("size", &MjsGeom::size)
      .property("contype", &MjsGeom::contype, &MjsGeom::set_contype, reference())
      .property("conaffinity", &MjsGeom::conaffinity, &MjsGeom::set_conaffinity, reference())
      .property("condim", &MjsGeom::condim, &MjsGeom::set_condim, reference())
      .property("priority", &MjsGeom::priority, &MjsGeom::set_priority, reference())
      .property("friction", &MjsGeom::friction)
      .property("solmix", &MjsGeom::solmix, &MjsGeom::set_solmix, reference())
      .property("solref", &MjsGeom::solref)
      .property("solimp", &MjsGeom::solimp)
      .property("margin", &MjsGeom::margin, &MjsGeom::set_margin, reference())
      .property("gap", &MjsGeom::gap, &MjsGeom::set_gap, reference())
      .property("mass", &MjsGeom::mass, &MjsGeom::set_mass, reference())
      .property("density", &MjsGeom::density, &MjsGeom::set_density, reference())
      .property("typeinertia", &MjsGeom::typeinertia, &MjsGeom::set_typeinertia, reference())
      .property("fluid_ellipsoid", &MjsGeom::fluid_ellipsoid, &MjsGeom::set_fluid_ellipsoid, reference())
      .property("fluid_coefs", &MjsGeom::fluid_coefs)
      .property("material", &MjsGeom::material, &MjsGeom::set_material, reference())
      .property("rgba", &MjsGeom::rgba)
      .property("group", &MjsGeom::group, &MjsGeom::set_group, reference())
      .property("hfieldname", &MjsGeom::hfieldname, &MjsGeom::set_hfieldname, reference())
      .property("meshname", &MjsGeom::meshname, &MjsGeom::set_meshname, reference())
      .property("fitscale", &MjsGeom::fitscale, &MjsGeom::set_fitscale, reference())
      .property("userdata", &MjsGeom::userdata, reference())
      .property("plugin", &MjsGeom::plugin, reference())
      .property("info", &MjsGeom::info, &MjsGeom::set_info, reference())
      ;

  emscripten::class_<MjsFrame>("MjsFrame")
      .property("element", &MjsFrame::element, reference())
      .property("childclass", &MjsFrame::childclass, &MjsFrame::set_childclass, reference())
      .property("pos", &MjsFrame::pos)
      .property("quat", &MjsFrame::quat)
      .property("alt", &MjsFrame::alt, reference())
      .property("info", &MjsFrame::info, &MjsFrame::set_info, reference())
      ;

  emscripten::class_<MjsJoint>("MjsJoint")
      .property("element", &MjsJoint::element, reference())
      .property("type", &MjsJoint::type, &MjsJoint::set_type, reference())
      .property("pos", &MjsJoint::pos)
      .property("axis", &MjsJoint::axis)
      .property("ref", &MjsJoint::ref, &MjsJoint::set_ref, reference())
      .property("align", &MjsJoint::align, &MjsJoint::set_align, reference())
      .property("stiffness", &MjsJoint::stiffness, &MjsJoint::set_stiffness, reference())
      .property("springref", &MjsJoint::springref, &MjsJoint::set_springref, reference())
      .property("springdamper", &MjsJoint::springdamper)
      .property("limited", &MjsJoint::limited, &MjsJoint::set_limited, reference())
      .property("range", &MjsJoint::range)
      .property("margin", &MjsJoint::margin, &MjsJoint::set_margin, reference())
      .property("solref_limit", &MjsJoint::solref_limit)
      .property("solimp_limit", &MjsJoint::solimp_limit)
      .property("actfrclimited", &MjsJoint::actfrclimited, &MjsJoint::set_actfrclimited, reference())
      .property("actfrcrange", &MjsJoint::actfrcrange)
      .property("armature", &MjsJoint::armature, &MjsJoint::set_armature, reference())
      .property("damping", &MjsJoint::damping, &MjsJoint::set_damping, reference())
      .property("frictionloss", &MjsJoint::frictionloss, &MjsJoint::set_frictionloss, reference())
      .property("solref_friction", &MjsJoint::solref_friction)
      .property("solimp_friction", &MjsJoint::solimp_friction)
      .property("group", &MjsJoint::group, &MjsJoint::set_group, reference())
      .property("actgravcomp", &MjsJoint::actgravcomp, &MjsJoint::set_actgravcomp, reference())
      .property("userdata", &MjsJoint::userdata, reference())
      .property("info", &MjsJoint::info, &MjsJoint::set_info, reference())
      ;

  emscripten::class_<MjsSite>("MjsSite")
      .property("element", &MjsSite::element, reference())
      .property("pos", &MjsSite::pos)
      .property("quat", &MjsSite::quat)
      .property("alt", &MjsSite::alt, reference())
      .property("fromto", &MjsSite::fromto)
      .property("size", &MjsSite::size)
      .property("type", &MjsSite::type, &MjsSite::set_type, reference())
      .property("material", &MjsSite::material, &MjsSite::set_material, reference())
      .property("group", &MjsSite::group, &MjsSite::set_group, reference())
      .property("rgba", &MjsSite::rgba)
      .property("userdata", &MjsSite::userdata, reference())
      .property("info", &MjsSite::info, &MjsSite::set_info, reference())
      ;

  emscripten::class_<MjsCamera>("MjsCamera")
      .property("element", &MjsCamera::element, reference())
      .property("pos", &MjsCamera::pos)
      .property("quat", &MjsCamera::quat)
      .property("alt", &MjsCamera::alt, reference())
      .property("mode", &MjsCamera::mode, &MjsCamera::set_mode, reference())
      .property("targetbody", &MjsCamera::targetbody, &MjsCamera::set_targetbody, reference())
      .property("orthographic", &MjsCamera::orthographic, &MjsCamera::set_orthographic, reference())
      .property("fovy", &MjsCamera::fovy, &MjsCamera::set_fovy, reference())
      .property("ipd", &MjsCamera::ipd, &MjsCamera::set_ipd, reference())
      .property("intrinsic", &MjsCamera::intrinsic)
      .property("sensor_size", &MjsCamera::sensor_size)
      .property("resolution", &MjsCamera::resolution)
      .property("focal_length", &MjsCamera::focal_length)
      .property("focal_pixel", &MjsCamera::focal_pixel)
      .property("principal_length", &MjsCamera::principal_length)
      .property("principal_pixel", &MjsCamera::principal_pixel)
      .property("userdata", &MjsCamera::userdata, reference())
      .property("info", &MjsCamera::info, &MjsCamera::set_info, reference())
      ;

  emscripten::class_<MjsLight>("MjsLight")
      .property("element", &MjsLight::element, reference())
      .property("pos", &MjsLight::pos)
      .property("dir", &MjsLight::dir)
      .property("mode", &MjsLight::mode, &MjsLight::set_mode, reference())
      .property("targetbody", &MjsLight::targetbody, &MjsLight::set_targetbody, reference())
      .property("active", &MjsLight::active, &MjsLight::set_active, reference())
      .property("type", &MjsLight::type, &MjsLight::set_type, reference())
      .property("texture", &MjsLight::texture, &MjsLight::set_texture, reference())
      .property("castshadow", &MjsLight::castshadow, &MjsLight::set_castshadow, reference())
      .property("bulbradius", &MjsLight::bulbradius, &MjsLight::set_bulbradius, reference())
      .property("intensity", &MjsLight::intensity, &MjsLight::set_intensity, reference())
      .property("range", &MjsLight::range, &MjsLight::set_range, reference())
      .property("attenuation", &MjsLight::attenuation)
      .property("cutoff", &MjsLight::cutoff, &MjsLight::set_cutoff, reference())
      .property("exponent", &MjsLight::exponent, &MjsLight::set_exponent, reference())
      .property("ambient", &MjsLight::ambient)
      .property("diffuse", &MjsLight::diffuse)
      .property("specular", &MjsLight::specular)
      .property("info", &MjsLight::info, &MjsLight::set_info, reference())
      ;

  emscripten::class_<MjsFlex>("MjsFlex")
      .property("element", &MjsFlex::element, reference())
      .property("contype", &MjsFlex::contype, &MjsFlex::set_contype, reference())
      .property("conaffinity", &MjsFlex::conaffinity, &MjsFlex::set_conaffinity, reference())
      .property("condim", &MjsFlex::condim, &MjsFlex::set_condim, reference())
      .property("priority", &MjsFlex::priority, &MjsFlex::set_priority, reference())
      .property("friction", &MjsFlex::friction)
      .property("solmix", &MjsFlex::solmix, &MjsFlex::set_solmix, reference())
      .property("solref", &MjsFlex::solref)
      .property("solimp", &MjsFlex::solimp)
      .property("margin", &MjsFlex::margin, &MjsFlex::set_margin, reference())
      .property("gap", &MjsFlex::gap, &MjsFlex::set_gap, reference())
      .property("dim", &MjsFlex::dim, &MjsFlex::set_dim, reference())
      .property("radius", &MjsFlex::radius, &MjsFlex::set_radius, reference())
      .property("internal", &MjsFlex::internal, &MjsFlex::set_internal, reference())
      .property("flatskin", &MjsFlex::flatskin, &MjsFlex::set_flatskin, reference())
      .property("selfcollide", &MjsFlex::selfcollide, &MjsFlex::set_selfcollide, reference())
      .property("vertcollide", &MjsFlex::vertcollide, &MjsFlex::set_vertcollide, reference())
      .property("passive", &MjsFlex::passive, &MjsFlex::set_passive, reference())
      .property("activelayers", &MjsFlex::activelayers, &MjsFlex::set_activelayers, reference())
      .property("group", &MjsFlex::group, &MjsFlex::set_group, reference())
      .property("edgestiffness", &MjsFlex::edgestiffness, &MjsFlex::set_edgestiffness, reference())
      .property("edgedamping", &MjsFlex::edgedamping, &MjsFlex::set_edgedamping, reference())
      .property("rgba", &MjsFlex::rgba)
      .property("material", &MjsFlex::material, &MjsFlex::set_material, reference())
      .property("young", &MjsFlex::young, &MjsFlex::set_young, reference())
      .property("poisson", &MjsFlex::poisson, &MjsFlex::set_poisson, reference())
      .property("damping", &MjsFlex::damping, &MjsFlex::set_damping, reference())
      .property("thickness", &MjsFlex::thickness, &MjsFlex::set_thickness, reference())
      .property("elastic2d", &MjsFlex::elastic2d, &MjsFlex::set_elastic2d, reference())
      .property("nodebody", &MjsFlex::nodebody, reference())
      .property("vertbody", &MjsFlex::vertbody, reference())
      .property("node", &MjsFlex::node, reference())
      .property("vert", &MjsFlex::vert, reference())
      .property("elem", &MjsFlex::elem, reference())
      .property("texcoord", &MjsFlex::texcoord, reference())
      .property("elemtexcoord", &MjsFlex::elemtexcoord, reference())
      .property("info", &MjsFlex::info, &MjsFlex::set_info, reference())
      ;

  emscripten::class_<MjsMesh>("MjsMesh")
      .property("element", &MjsMesh::element, reference())
      .property("content_type", &MjsMesh::content_type, &MjsMesh::set_content_type, reference())
      .property("file", &MjsMesh::file, &MjsMesh::set_file, reference())
      .property("refpos", &MjsMesh::refpos)
      .property("refquat", &MjsMesh::refquat)
      .property("scale", &MjsMesh::scale)
      .property("inertia", &MjsMesh::inertia, &MjsMesh::set_inertia, reference())
      .property("smoothnormal", &MjsMesh::smoothnormal, &MjsMesh::set_smoothnormal, reference())
      .property("needsdf", &MjsMesh::needsdf, &MjsMesh::set_needsdf, reference())
      .property("maxhullvert", &MjsMesh::maxhullvert, &MjsMesh::set_maxhullvert, reference())
      .property("uservert", &MjsMesh::uservert, reference())
      .property("usernormal", &MjsMesh::usernormal, reference())
      .property("usertexcoord", &MjsMesh::usertexcoord, reference())
      .property("userface", &MjsMesh::userface, reference())
      .property("userfacenormal", &MjsMesh::userfacenormal, reference())
      .property("userfacetexcoord", &MjsMesh::userfacetexcoord, reference())
      .property("plugin", &MjsMesh::plugin, reference())
      .property("material", &MjsMesh::material, &MjsMesh::set_material, reference())
      .property("info", &MjsMesh::info, &MjsMesh::set_info, reference())
      ;

  emscripten::class_<MjsHField>("MjsHField")
      .property("element", &MjsHField::element, reference())
      .property("content_type", &MjsHField::content_type, &MjsHField::set_content_type, reference())
      .property("file", &MjsHField::file, &MjsHField::set_file, reference())
      .property("size", &MjsHField::size)
      .property("nrow", &MjsHField::nrow, &MjsHField::set_nrow, reference())
      .property("ncol", &MjsHField::ncol, &MjsHField::set_ncol, reference())
      .property("userdata", &MjsHField::userdata, reference())
      .property("info", &MjsHField::info, &MjsHField::set_info, reference())
      ;

  emscripten::class_<MjsSkin>("MjsSkin")
      .property("element", &MjsSkin::element, reference())
      .property("file", &MjsSkin::file, &MjsSkin::set_file, reference())
      .property("material", &MjsSkin::material, &MjsSkin::set_material, reference())
      .property("rgba", &MjsSkin::rgba)
      .property("inflate", &MjsSkin::inflate, &MjsSkin::set_inflate, reference())
      .property("group", &MjsSkin::group, &MjsSkin::set_group, reference())
      .property("vert", &MjsSkin::vert, reference())
      .property("texcoord", &MjsSkin::texcoord, reference())
      .property("face", &MjsSkin::face, reference())
      .property("bodyname", &MjsSkin::bodyname, reference())
      .property("bindpos", &MjsSkin::bindpos, reference())
      .property("bindquat", &MjsSkin::bindquat, reference())
      .property("vertid", &MjsSkin::vertid, reference())
      .property("vertweight", &MjsSkin::vertweight, reference())
      .property("info", &MjsSkin::info, &MjsSkin::set_info, reference())
      ;

  emscripten::class_<MjsTexture>("MjsTexture")
      .property("element", &MjsTexture::element, reference())
      .property("type", &MjsTexture::type, &MjsTexture::set_type, reference())
      .property("colorspace", &MjsTexture::colorspace, &MjsTexture::set_colorspace, reference())
      .property("builtin", &MjsTexture::builtin, &MjsTexture::set_builtin, reference())
      .property("mark", &MjsTexture::mark, &MjsTexture::set_mark, reference())
      .property("rgb1", &MjsTexture::rgb1)
      .property("rgb2", &MjsTexture::rgb2)
      .property("markrgb", &MjsTexture::markrgb)
      .property("random", &MjsTexture::random, &MjsTexture::set_random, reference())
      .property("height", &MjsTexture::height, &MjsTexture::set_height, reference())
      .property("width", &MjsTexture::width, &MjsTexture::set_width, reference())
      .property("nchannel", &MjsTexture::nchannel, &MjsTexture::set_nchannel, reference())
      .property("content_type", &MjsTexture::content_type, &MjsTexture::set_content_type, reference())
      .property("file", &MjsTexture::file, &MjsTexture::set_file, reference())
      .property("gridsize", &MjsTexture::gridsize)
      .property("gridlayout", &MjsTexture::gridlayout)
      .property("cubefiles", &MjsTexture::cubefiles, reference())
      .property("data", &MjsTexture::data, reference())
      .property("hflip", &MjsTexture::hflip, &MjsTexture::set_hflip, reference())
      .property("vflip", &MjsTexture::vflip, &MjsTexture::set_vflip, reference())
      .property("info", &MjsTexture::info, &MjsTexture::set_info, reference())
      ;

  emscripten::class_<MjsMaterial>("MjsMaterial")
      .property("element", &MjsMaterial::element, reference())
      .property("textures", &MjsMaterial::textures, reference())
      .property("texuniform", &MjsMaterial::texuniform, &MjsMaterial::set_texuniform, reference())
      .property("texrepeat", &MjsMaterial::texrepeat)
      .property("emission", &MjsMaterial::emission, &MjsMaterial::set_emission, reference())
      .property("specular", &MjsMaterial::specular, &MjsMaterial::set_specular, reference())
      .property("shininess", &MjsMaterial::shininess, &MjsMaterial::set_shininess, reference())
      .property("reflectance", &MjsMaterial::reflectance, &MjsMaterial::set_reflectance, reference())
      .property("metallic", &MjsMaterial::metallic, &MjsMaterial::set_metallic, reference())
      .property("roughness", &MjsMaterial::roughness, &MjsMaterial::set_roughness, reference())
      .property("rgba", &MjsMaterial::rgba)
      .property("info", &MjsMaterial::info, &MjsMaterial::set_info, reference())
      ;

  emscripten::class_<MjsPair>("MjsPair")
      .property("element", &MjsPair::element, reference())
      .property("geomname1", &MjsPair::geomname1, &MjsPair::set_geomname1, reference())
      .property("geomname2", &MjsPair::geomname2, &MjsPair::set_geomname2, reference())
      .property("condim", &MjsPair::condim, &MjsPair::set_condim, reference())
      .property("solref", &MjsPair::solref)
      .property("solreffriction", &MjsPair::solreffriction)
      .property("solimp", &MjsPair::solimp)
      .property("margin", &MjsPair::margin, &MjsPair::set_margin, reference())
      .property("gap", &MjsPair::gap, &MjsPair::set_gap, reference())
      .property("friction", &MjsPair::friction)
      .property("info", &MjsPair::info, &MjsPair::set_info, reference())
      ;

  emscripten::class_<MjsExclude>("MjsExclude")
      .property("element", &MjsExclude::element, reference())
      .property("bodyname1", &MjsExclude::bodyname1, &MjsExclude::set_bodyname1, reference())
      .property("bodyname2", &MjsExclude::bodyname2, &MjsExclude::set_bodyname2, reference())
      .property("info", &MjsExclude::info, &MjsExclude::set_info, reference())
      ;

  emscripten::class_<MjsEquality>("MjsEquality")
      .property("element", &MjsEquality::element, reference())
      .property("type", &MjsEquality::type, &MjsEquality::set_type, reference())
      .property("data", &MjsEquality::data)
      .property("active", &MjsEquality::active, &MjsEquality::set_active, reference())
      .property("name1", &MjsEquality::name1, &MjsEquality::set_name1, reference())
      .property("name2", &MjsEquality::name2, &MjsEquality::set_name2, reference())
      .property("objtype", &MjsEquality::objtype, &MjsEquality::set_objtype, reference())
      .property("solref", &MjsEquality::solref)
      .property("solimp", &MjsEquality::solimp)
      .property("info", &MjsEquality::info, &MjsEquality::set_info, reference())
      ;

  emscripten::class_<MjsTendon>("MjsTendon")
      .property("element", &MjsTendon::element, reference())
      .property("stiffness", &MjsTendon::stiffness, &MjsTendon::set_stiffness, reference())
      .property("springlength", &MjsTendon::springlength)
      .property("damping", &MjsTendon::damping, &MjsTendon::set_damping, reference())
      .property("frictionloss", &MjsTendon::frictionloss, &MjsTendon::set_frictionloss, reference())
      .property("solref_friction", &MjsTendon::solref_friction)
      .property("solimp_friction", &MjsTendon::solimp_friction)
      .property("armature", &MjsTendon::armature, &MjsTendon::set_armature, reference())
      .property("limited", &MjsTendon::limited, &MjsTendon::set_limited, reference())
      .property("actfrclimited", &MjsTendon::actfrclimited, &MjsTendon::set_actfrclimited, reference())
      .property("range", &MjsTendon::range)
      .property("actfrcrange", &MjsTendon::actfrcrange)
      .property("margin", &MjsTendon::margin, &MjsTendon::set_margin, reference())
      .property("solref_limit", &MjsTendon::solref_limit)
      .property("solimp_limit", &MjsTendon::solimp_limit)
      .property("material", &MjsTendon::material, &MjsTendon::set_material, reference())
      .property("width", &MjsTendon::width, &MjsTendon::set_width, reference())
      .property("rgba", &MjsTendon::rgba)
      .property("group", &MjsTendon::group, &MjsTendon::set_group, reference())
      .property("userdata", &MjsTendon::userdata, reference())
      .property("info", &MjsTendon::info, &MjsTendon::set_info, reference())
      ;

  emscripten::class_<MjsWrap>("MjsWrap")
      .property("element", &MjsWrap::element, reference())
      .property("type", &MjsWrap::type, &MjsWrap::set_type, reference())
      .property("info", &MjsWrap::info, &MjsWrap::set_info, reference())
      ;

  emscripten::class_<MjsActuator>("MjsActuator")
      .property("element", &MjsActuator::element, reference())
      .property("gaintype", &MjsActuator::gaintype, &MjsActuator::set_gaintype, reference())
      .property("gainprm", &MjsActuator::gainprm)
      .property("biastype", &MjsActuator::biastype, &MjsActuator::set_biastype, reference())
      .property("biasprm", &MjsActuator::biasprm)
      .property("dyntype", &MjsActuator::dyntype, &MjsActuator::set_dyntype, reference())
      .property("dynprm", &MjsActuator::dynprm)
      .property("actdim", &MjsActuator::actdim, &MjsActuator::set_actdim, reference())
      .property("actearly", &MjsActuator::actearly, &MjsActuator::set_actearly, reference())
      .property("trntype", &MjsActuator::trntype, &MjsActuator::set_trntype, reference())
      .property("gear", &MjsActuator::gear)
      .property("target", &MjsActuator::target, &MjsActuator::set_target, reference())
      .property("refsite", &MjsActuator::refsite, &MjsActuator::set_refsite, reference())
      .property("slidersite", &MjsActuator::slidersite, &MjsActuator::set_slidersite, reference())
      .property("cranklength", &MjsActuator::cranklength, &MjsActuator::set_cranklength, reference())
      .property("lengthrange", &MjsActuator::lengthrange)
      .property("inheritrange", &MjsActuator::inheritrange, &MjsActuator::set_inheritrange, reference())
      .property("ctrllimited", &MjsActuator::ctrllimited, &MjsActuator::set_ctrllimited, reference())
      .property("ctrlrange", &MjsActuator::ctrlrange)
      .property("forcelimited", &MjsActuator::forcelimited, &MjsActuator::set_forcelimited, reference())
      .property("forcerange", &MjsActuator::forcerange)
      .property("actlimited", &MjsActuator::actlimited, &MjsActuator::set_actlimited, reference())
      .property("actrange", &MjsActuator::actrange)
      .property("group", &MjsActuator::group, &MjsActuator::set_group, reference())
      .property("userdata", &MjsActuator::userdata, reference())
      .property("plugin", &MjsActuator::plugin, reference())
      .property("info", &MjsActuator::info, &MjsActuator::set_info, reference())
      ;

  emscripten::class_<MjsSensor>("MjsSensor")
      .property("element", &MjsSensor::element, reference())
      .property("type", &MjsSensor::type, &MjsSensor::set_type, reference())
      .property("objtype", &MjsSensor::objtype, &MjsSensor::set_objtype, reference())
      .property("objname", &MjsSensor::objname, &MjsSensor::set_objname, reference())
      .property("reftype", &MjsSensor::reftype, &MjsSensor::set_reftype, reference())
      .property("refname", &MjsSensor::refname, &MjsSensor::set_refname, reference())
      .property("intprm", &MjsSensor::intprm)
      .property("datatype", &MjsSensor::datatype, &MjsSensor::set_datatype, reference())
      .property("needstage", &MjsSensor::needstage, &MjsSensor::set_needstage, reference())
      .property("dim", &MjsSensor::dim, &MjsSensor::set_dim, reference())
      .property("cutoff", &MjsSensor::cutoff, &MjsSensor::set_cutoff, reference())
      .property("noise", &MjsSensor::noise, &MjsSensor::set_noise, reference())
      .property("userdata", &MjsSensor::userdata, reference())
      .property("plugin", &MjsSensor::plugin, reference())
      .property("info", &MjsSensor::info, &MjsSensor::set_info, reference())
      ;

  emscripten::class_<MjsNumeric>("MjsNumeric")
      .property("element", &MjsNumeric::element, reference())
      .property("data", &MjsNumeric::data, reference())
      .property("size", &MjsNumeric::size, &MjsNumeric::set_size, reference())
      .property("info", &MjsNumeric::info, &MjsNumeric::set_info, reference())
      ;

  emscripten::class_<MjsText>("MjsText")
      .property("element", &MjsText::element, reference())
      .property("data", &MjsText::data, &MjsText::set_data, reference())
      .property("info", &MjsText::info, &MjsText::set_info, reference())
      ;

  emscripten::class_<MjsTuple>("MjsTuple")
      .property("element", &MjsTuple::element, reference())
      .property("objtype", &MjsTuple::objtype, reference())
      .property("objname", &MjsTuple::objname, reference())
      .property("objprm", &MjsTuple::objprm, reference())
      .property("info", &MjsTuple::info, &MjsTuple::set_info, reference())
      ;

  emscripten::class_<MjsKey>("MjsKey")
      .property("element", &MjsKey::element, reference())
      .property("time", &MjsKey::time, &MjsKey::set_time, reference())
      .property("qpos", &MjsKey::qpos, reference())
      .property("qvel", &MjsKey::qvel, reference())
      .property("act", &MjsKey::act, reference())
      .property("mpos", &MjsKey::mpos, reference())
      .property("mquat", &MjsKey::mquat, reference())
      .property("ctrl", &MjsKey::ctrl, reference())
      .property("info", &MjsKey::info, &MjsKey::set_info, reference())
      ;

  emscripten::class_<MjsDefault>("MjsDefault")
      .property("element", &MjsDefault::element, reference())
      .property("joint", &MjsDefault::joint, reference())
      .property("geom", &MjsDefault::geom, reference())
      .property("site", &MjsDefault::site, reference())
      .property("camera", &MjsDefault::camera, reference())
      .property("light", &MjsDefault::light, reference())
      .property("flex", &MjsDefault::flex, reference())
      .property("mesh", &MjsDefault::mesh, reference())
      .property("material", &MjsDefault::material, reference())
      .property("pair", &MjsDefault::pair, reference())
      .property("equality", &MjsDefault::equality, reference())
      .property("tendon", &MjsDefault::tendon, reference())
      .property("actuator", &MjsDefault::actuator, reference())
      ;

  emscripten::class_<MjsPlugin>("MjsPlugin")
      .property("element", &MjsPlugin::element, reference())
      .property("name", &MjsPlugin::name, &MjsPlugin::set_name, reference())
      .property("plugin_name", &MjsPlugin::plugin_name, &MjsPlugin::set_plugin_name, reference())
      .property("active", &MjsPlugin::active, &MjsPlugin::set_active, reference())
      .property("info", &MjsPlugin::info, &MjsPlugin::set_info, reference())
      ;

  emscripten::class_<MjVFS>("MjVFS").constructor<>()
      // TODO: .property("impl_", &MjVFS::impl_)
      ;

  // TODO: should be generated in future CLs -- //
  emscripten::register_vector<MjSolverStat>("MjSolverStatVec");
  emscripten::register_vector<MjTimerStat>("MjTimerStatVec");
  emscripten::register_vector<MjWarningStat>("MjWarningStatVec");
  emscripten::register_vector<MjContact>("MjContactVec");
  emscripten::register_vector<MjvLight>("MjvLightVec");
  emscripten::register_vector<MjvGLCamera>("MjvGLCameraVec");
  emscripten::register_vector<MjvGeom>("MjvGeomVec");
}

// FUNCTIONS
EMSCRIPTEN_DECLARE_VAL_TYPE(NumberArray);
EMSCRIPTEN_DECLARE_VAL_TYPE(String);

// Raises an error if the given val is null or undefined.
// A macro is used so that the error contains the name of the variable.
// TODO(matijak): Remove this when we can handle strings using UNPACK_STRING?
#define CHECK_VAL(val)                                    \
  if (val.isNull()) {                                     \
    mju_error("Invalid argument: %s is null", #val);      \
  } else if (val.isUndefined()) {                         \
    mju_error("Invalid argument: %s is undefined", #val); \
  }
void error_wrapper(const String& msg) { mju_error("%s\n", msg.as<const std::string>().data()); }

int mj_copyBack_wrapper(MjSpec& s, const MjModel& m)
{
  return mj_copyBack(s.get(), m.get());
}

void mj_step_wrapper(const MjModel& m, MjData& d)
{
  mj_step(m.get(), d.get());
}

void mj_step1_wrapper(const MjModel& m, MjData& d)
{
  mj_step1(m.get(), d.get());
}

void mj_step2_wrapper(const MjModel& m, MjData& d)
{
  mj_step2(m.get(), d.get());
}

void mj_forward_wrapper(const MjModel& m, MjData& d)
{
  mj_forward(m.get(), d.get());
}

void mj_inverse_wrapper(const MjModel& m, MjData& d)
{
  mj_inverse(m.get(), d.get());
}

void mj_forwardSkip_wrapper(const MjModel& m, MjData& d, int skipstage, int skipsensor)
{
  mj_forwardSkip(m.get(), d.get(), skipstage, skipsensor);
}

void mj_inverseSkip_wrapper(const MjModel& m, MjData& d, int skipstage, int skipsensor)
{
  mj_inverseSkip(m.get(), d.get(), skipstage, skipsensor);
}

void mj_defaultLROpt_wrapper(MjLROpt& opt)
{
  mj_defaultLROpt(opt.get());
}

void mj_defaultSolRefImp_wrapper(const val& solref, const val& solimp)
{
  UNPACK_VALUE(mjtNum, solref);
  UNPACK_VALUE(mjtNum, solimp);
  mj_defaultSolRefImp(solref_.data(), solimp_.data());
}

void mj_defaultOption_wrapper(MjOption& opt)
{
  mj_defaultOption(opt.get());
}

void mj_defaultVisual_wrapper(MjVisual& vis)
{
  mj_defaultVisual(vis.get());
}

int mj_sizeModel_wrapper(const MjModel& m)
{
  return mj_sizeModel(m.get());
}

void mj_resetData_wrapper(const MjModel& m, MjData& d)
{
  mj_resetData(m.get(), d.get());
}

void mj_resetDataDebug_wrapper(const MjModel& m, MjData& d, unsigned char debug_value)
{
  mj_resetDataDebug(m.get(), d.get(), debug_value);
}

void mj_resetDataKeyframe_wrapper(const MjModel& m, MjData& d, int key)
{
  mj_resetDataKeyframe(m.get(), d.get(), key);
}

void mj_setConst_wrapper(MjModel& m, MjData& d)
{
  mj_setConst(m.get(), d.get());
}

int mjs_activatePlugin_wrapper(MjSpec& s, const String& name)
{
  CHECK_VAL(name);
  return mjs_activatePlugin(s.get(), name.as<const std::string>().data());
}

int mjs_setDeepCopy_wrapper(MjSpec& s, int deepcopy)
{
  return mjs_setDeepCopy(s.get(), deepcopy);
}

void mj_printFormattedModel_wrapper(const MjModel& m, const String& filename, const String& float_format)
{
  CHECK_VAL(filename);
  CHECK_VAL(float_format);
  mj_printFormattedModel(m.get(), filename.as<const std::string>().data(), float_format.as<const std::string>().data());
}

void mj_printModel_wrapper(const MjModel& m, const String& filename)
{
  CHECK_VAL(filename);
  mj_printModel(m.get(), filename.as<const std::string>().data());
}

void mj_printFormattedData_wrapper(const MjModel& m, const MjData& d, const String& filename, const String& float_format)
{
  CHECK_VAL(filename);
  CHECK_VAL(float_format);
  mj_printFormattedData(m.get(), d.get(), filename.as<const std::string>().data(), float_format.as<const std::string>().data());
}

void mj_printData_wrapper(const MjModel& m, const MjData& d, const String& filename)
{
  CHECK_VAL(filename);
  mj_printData(m.get(), d.get(), filename.as<const std::string>().data());
}

void mju_printMat_wrapper(const NumberArray& mat, int nr, int nc)
{
  UNPACK_ARRAY(mjtNum, mat);
  mju_printMat(mat_.data(), nr, nc);
}

void mj_printScene_wrapper(const MjvScene& s, const String& filename)
{
  CHECK_VAL(filename);
  mj_printScene(s.get(), filename.as<const std::string>().data());
}

void mj_printFormattedScene_wrapper(const MjvScene& s, const String& filename, const String& float_format)
{
  CHECK_VAL(filename);
  CHECK_VAL(float_format);
  mj_printFormattedScene(s.get(), filename.as<const std::string>().data(), float_format.as<const std::string>().data());
}

void mj_fwdPosition_wrapper(const MjModel& m, MjData& d)
{
  mj_fwdPosition(m.get(), d.get());
}

void mj_fwdVelocity_wrapper(const MjModel& m, MjData& d)
{
  mj_fwdVelocity(m.get(), d.get());
}

void mj_fwdActuation_wrapper(const MjModel& m, MjData& d)
{
  mj_fwdActuation(m.get(), d.get());
}

void mj_fwdAcceleration_wrapper(const MjModel& m, MjData& d)
{
  mj_fwdAcceleration(m.get(), d.get());
}

void mj_fwdConstraint_wrapper(const MjModel& m, MjData& d)
{
  mj_fwdConstraint(m.get(), d.get());
}

void mj_Euler_wrapper(const MjModel& m, MjData& d)
{
  mj_Euler(m.get(), d.get());
}

void mj_RungeKutta_wrapper(const MjModel& m, MjData& d, int N)
{
  mj_RungeKutta(m.get(), d.get(), N);
}

void mj_implicit_wrapper(const MjModel& m, MjData& d)
{
  mj_implicit(m.get(), d.get());
}

void mj_invPosition_wrapper(const MjModel& m, MjData& d)
{
  mj_invPosition(m.get(), d.get());
}

void mj_invVelocity_wrapper(const MjModel& m, MjData& d)
{
  mj_invVelocity(m.get(), d.get());
}

void mj_invConstraint_wrapper(const MjModel& m, MjData& d)
{
  mj_invConstraint(m.get(), d.get());
}

void mj_compareFwdInv_wrapper(const MjModel& m, MjData& d)
{
  mj_compareFwdInv(m.get(), d.get());
}

void mj_sensorPos_wrapper(const MjModel& m, MjData& d)
{
  mj_sensorPos(m.get(), d.get());
}

void mj_sensorVel_wrapper(const MjModel& m, MjData& d)
{
  mj_sensorVel(m.get(), d.get());
}

void mj_sensorAcc_wrapper(const MjModel& m, MjData& d)
{
  mj_sensorAcc(m.get(), d.get());
}

void mj_energyPos_wrapper(const MjModel& m, MjData& d)
{
  mj_energyPos(m.get(), d.get());
}

void mj_energyVel_wrapper(const MjModel& m, MjData& d)
{
  mj_energyVel(m.get(), d.get());
}

void mj_checkPos_wrapper(const MjModel& m, MjData& d)
{
  mj_checkPos(m.get(), d.get());
}

void mj_checkVel_wrapper(const MjModel& m, MjData& d)
{
  mj_checkVel(m.get(), d.get());
}

void mj_checkAcc_wrapper(const MjModel& m, MjData& d)
{
  mj_checkAcc(m.get(), d.get());
}

void mj_kinematics_wrapper(const MjModel& m, MjData& d)
{
  mj_kinematics(m.get(), d.get());
}

void mj_comPos_wrapper(const MjModel& m, MjData& d)
{
  mj_comPos(m.get(), d.get());
}

void mj_camlight_wrapper(const MjModel& m, MjData& d)
{
  mj_camlight(m.get(), d.get());
}

void mj_flex_wrapper(const MjModel& m, MjData& d)
{
  mj_flex(m.get(), d.get());
}

void mj_tendon_wrapper(const MjModel& m, MjData& d)
{
  mj_tendon(m.get(), d.get());
}

void mj_transmission_wrapper(const MjModel& m, MjData& d)
{
  mj_transmission(m.get(), d.get());
}

void mj_crb_wrapper(const MjModel& m, MjData& d)
{
  mj_crb(m.get(), d.get());
}

void mj_makeM_wrapper(const MjModel& m, MjData& d)
{
  mj_makeM(m.get(), d.get());
}

void mj_factorM_wrapper(const MjModel& m, MjData& d)
{
  mj_factorM(m.get(), d.get());
}

void mj_comVel_wrapper(const MjModel& m, MjData& d)
{
  mj_comVel(m.get(), d.get());
}

void mj_passive_wrapper(const MjModel& m, MjData& d)
{
  mj_passive(m.get(), d.get());
}

void mj_subtreeVel_wrapper(const MjModel& m, MjData& d)
{
  mj_subtreeVel(m.get(), d.get());
}

void mj_rnePostConstraint_wrapper(const MjModel& m, MjData& d)
{
  mj_rnePostConstraint(m.get(), d.get());
}

void mj_collision_wrapper(const MjModel& m, MjData& d)
{
  mj_collision(m.get(), d.get());
}

void mj_makeConstraint_wrapper(const MjModel& m, MjData& d)
{
  mj_makeConstraint(m.get(), d.get());
}

void mj_island_wrapper(const MjModel& m, MjData& d)
{
  mj_island(m.get(), d.get());
}

void mj_projectConstraint_wrapper(const MjModel& m, MjData& d)
{
  mj_projectConstraint(m.get(), d.get());
}

void mj_referenceConstraint_wrapper(const MjModel& m, MjData& d)
{
  mj_referenceConstraint(m.get(), d.get());
}

int mj_stateSize_wrapper(const MjModel& m, unsigned int sig)
{
  return mj_stateSize(m.get(), sig);
}

void mj_extractState_wrapper(const MjModel& m, const NumberArray& src, unsigned int srcsig, const val& dst, unsigned int dstsig)
{
  UNPACK_ARRAY(mjtNum, src);
  UNPACK_VALUE(mjtNum, dst);
  mj_extractState(m.get(), src_.data(), srcsig, dst_.data(), dstsig);
}

void mj_setKeyframe_wrapper(MjModel& m, const MjData& d, int k)
{
  mj_setKeyframe(m.get(), d.get(), k);
}

int mj_addContact_wrapper(const MjModel& m, MjData& d, const MjContact& con)
{
  return mj_addContact(m.get(), d.get(), con.get());
}

int mj_isPyramidal_wrapper(const MjModel& m)
{
  return mj_isPyramidal(m.get());
}

int mj_isSparse_wrapper(const MjModel& m)
{
  return mj_isSparse(m.get());
}

int mj_isDual_wrapper(const MjModel& m)
{
  return mj_isDual(m.get());
}

int mj_name2id_wrapper(const MjModel& m, int type, const String& name)
{
  CHECK_VAL(name);
  return mj_name2id(m.get(), type, name.as<const std::string>().data());
}

std::string mj_id2name_wrapper(const MjModel& m, int type, int id)
{
  return std::string(mj_id2name(m.get(), type, id));
}

void mj_objectVelocity_wrapper(const MjModel& m, const MjData& d, int objtype, int objid, const val& res, int flg_local)
{
  UNPACK_VALUE(mjtNum, res);
  mj_objectVelocity(m.get(), d.get(), objtype, objid, res_.data(), flg_local);
}

void mj_objectAcceleration_wrapper(const MjModel& m, const MjData& d, int objtype, int objid, const val& res, int flg_local)
{
  UNPACK_VALUE(mjtNum, res);
  mj_objectAcceleration(m.get(), d.get(), objtype, objid, res_.data(), flg_local);
}

void mj_contactForce_wrapper(const MjModel& m, const MjData& d, int id, const val& result)
{
  UNPACK_VALUE(mjtNum, result);
  mj_contactForce(m.get(), d.get(), id, result_.data());
}

void mj_local2Global_wrapper(MjData& d, const val& xpos, const val& xmat, const NumberArray& pos, const NumberArray& quat, int body, mjtByte sameframe)
{
  UNPACK_VALUE(mjtNum, xpos);
  UNPACK_VALUE(mjtNum, xmat);
  UNPACK_ARRAY(mjtNum, pos);
  UNPACK_ARRAY(mjtNum, quat);
  mj_local2Global(d.get(), xpos_.data(), xmat_.data(), pos_.data(), quat_.data(), body, sameframe);
}

mjtNum mj_getTotalmass_wrapper(const MjModel& m)
{
  return mj_getTotalmass(m.get());
}

void mj_setTotalmass_wrapper(MjModel& m, mjtNum newmass)
{
  mj_setTotalmass(m.get(), newmass);
}

std::string mj_versionString_wrapper()
{
  return std::string(mj_versionString());
}

mjtNum mj_ray_wrapper(const MjModel& m, const MjData& d, const NumberArray& pnt, const NumberArray& vec, const NumberArray& geomgroup, mjtByte flg_static, int bodyexclude, const val& geomid)
{
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_ARRAY(mjtByte, geomgroup);
  UNPACK_VALUE(int, geomid);
  return mj_ray(m.get(), d.get(), pnt_.data(), vec_.data(), geomgroup_.data(), flg_static, bodyexclude, geomid_.data());
}

mjtNum mj_rayHfield_wrapper(const MjModel& m, const MjData& d, int geomid, const NumberArray& pnt, const NumberArray& vec)
{
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  return mj_rayHfield(m.get(), d.get(), geomid, pnt_.data(), vec_.data());
}

mjtNum mj_rayMesh_wrapper(const MjModel& m, const MjData& d, int geomid, const NumberArray& pnt, const NumberArray& vec)
{
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  return mj_rayMesh(m.get(), d.get(), geomid, pnt_.data(), vec_.data());
}

mjtNum mju_rayGeom_wrapper(const NumberArray& pos, const NumberArray& mat, const NumberArray& size, const NumberArray& pnt, const NumberArray& vec, int geomtype)
{
  UNPACK_ARRAY(mjtNum, pos);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, size);
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  return mju_rayGeom(pos_.data(), mat_.data(), size_.data(), pnt_.data(), vec_.data(), geomtype);
}

mjtNum mju_rayFlex_wrapper(const MjModel& m, const MjData& d, int flex_layer, mjtByte flg_vert, mjtByte flg_edge, mjtByte flg_face, mjtByte flg_skin, int flexid, const NumberArray& pnt, const NumberArray& vec, const val& vertid)
{
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_VALUE(int, vertid);
  return mju_rayFlex(m.get(), d.get(), flex_layer, flg_vert, flg_edge, flg_face, flg_skin, flexid, pnt_.data(), vec_.data(), vertid_.data());
}

mjtNum mju_raySkin_wrapper(int nface, int nvert, const NumberArray& face, const NumberArray& vert, const NumberArray& pnt, const NumberArray& vec, const val& vertid)
{
  UNPACK_ARRAY(int, face);
  UNPACK_ARRAY(float, vert);
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_VALUE(int, vertid);
  return mju_raySkin(nface, nvert, face_.data(), vert_.data(), pnt_.data(), vec_.data(), vertid_.data());
}

void mjv_defaultCamera_wrapper(MjvCamera& cam)
{
  mjv_defaultCamera(cam.get());
}

void mjv_defaultFreeCamera_wrapper(const MjModel& m, MjvCamera& cam)
{
  mjv_defaultFreeCamera(m.get(), cam.get());
}

void mjv_defaultPerturb_wrapper(MjvPerturb& pert)
{
  mjv_defaultPerturb(pert.get());
}

void mjv_room2model_wrapper(const val& modelpos, const val& modelquat, const NumberArray& roompos, const NumberArray& roomquat, const MjvScene& scn)
{
  UNPACK_VALUE(mjtNum, modelpos);
  UNPACK_VALUE(mjtNum, modelquat);
  UNPACK_ARRAY(mjtNum, roompos);
  UNPACK_ARRAY(mjtNum, roomquat);
  mjv_room2model(modelpos_.data(), modelquat_.data(), roompos_.data(), roomquat_.data(), scn.get());
}

void mjv_model2room_wrapper(const val& roompos, const val& roomquat, const NumberArray& modelpos, const NumberArray& modelquat, const MjvScene& scn)
{
  UNPACK_VALUE(mjtNum, roompos);
  UNPACK_VALUE(mjtNum, roomquat);
  UNPACK_ARRAY(mjtNum, modelpos);
  UNPACK_ARRAY(mjtNum, modelquat);
  mjv_model2room(roompos_.data(), roomquat_.data(), modelpos_.data(), modelquat_.data(), scn.get());
}

void mjv_cameraInModel_wrapper(const val& headpos, const val& forward, const val& up, const MjvScene& scn)
{
  UNPACK_VALUE(mjtNum, headpos);
  UNPACK_VALUE(mjtNum, forward);
  UNPACK_VALUE(mjtNum, up);
  mjv_cameraInModel(headpos_.data(), forward_.data(), up_.data(), scn.get());
}

void mjv_cameraInRoom_wrapper(const val& headpos, const val& forward, const val& up, const MjvScene& scn)
{
  UNPACK_VALUE(mjtNum, headpos);
  UNPACK_VALUE(mjtNum, forward);
  UNPACK_VALUE(mjtNum, up);
  mjv_cameraInRoom(headpos_.data(), forward_.data(), up_.data(), scn.get());
}

mjtNum mjv_frustumHeight_wrapper(const MjvScene& scn)
{
  return mjv_frustumHeight(scn.get());
}

void mjv_alignToCamera_wrapper(const val& res, const NumberArray& vec, const NumberArray& forward)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_ARRAY(mjtNum, forward);
  mjv_alignToCamera(res_.data(), vec_.data(), forward_.data());
}

void mjv_moveCamera_wrapper(const MjModel& m, int action, mjtNum reldx, mjtNum reldy, const MjvScene& scn, MjvCamera& cam)
{
  mjv_moveCamera(m.get(), action, reldx, reldy, scn.get(), cam.get());
}

void mjv_movePerturb_wrapper(const MjModel& m, const MjData& d, int action, mjtNum reldx, mjtNum reldy, const MjvScene& scn, MjvPerturb& pert)
{
  mjv_movePerturb(m.get(), d.get(), action, reldx, reldy, scn.get(), pert.get());
}

void mjv_moveModel_wrapper(const MjModel& m, int action, mjtNum reldx, mjtNum reldy, const NumberArray& roomup, MjvScene& scn)
{
  UNPACK_ARRAY(mjtNum, roomup);
  mjv_moveModel(m.get(), action, reldx, reldy, roomup_.data(), scn.get());
}

void mjv_initPerturb_wrapper(const MjModel& m, MjData& d, const MjvScene& scn, MjvPerturb& pert)
{
  mjv_initPerturb(m.get(), d.get(), scn.get(), pert.get());
}

void mjv_applyPerturbPose_wrapper(const MjModel& m, MjData& d, const MjvPerturb& pert, int flg_paused)
{
  mjv_applyPerturbPose(m.get(), d.get(), pert.get(), flg_paused);
}

void mjv_applyPerturbForce_wrapper(const MjModel& m, MjData& d, const MjvPerturb& pert)
{
  mjv_applyPerturbForce(m.get(), d.get(), pert.get());
}

int mjv_select_wrapper(const MjModel& m, const MjData& d, const MjvOption& vopt, mjtNum aspectratio, mjtNum relx, mjtNum rely, const MjvScene& scn, const val& selpnt, const val& geomid, const val& flexid, const val& skinid)
{
  UNPACK_VALUE(mjtNum, selpnt);
  UNPACK_VALUE(int, geomid);
  UNPACK_VALUE(int, flexid);
  UNPACK_VALUE(int, skinid);
  return mjv_select(m.get(), d.get(), vopt.get(), aspectratio, relx, rely, scn.get(), selpnt_.data(), geomid_.data(), flexid_.data(), skinid_.data());
}

void mjv_defaultOption_wrapper(MjvOption& opt)
{
  mjv_defaultOption(opt.get());
}

void mjv_defaultFigure_wrapper(MjvFigure& fig)
{
  mjv_defaultFigure(fig.get());
}

void mjv_initGeom_wrapper(MjvGeom& geom, int type, const NumberArray& size, const NumberArray& pos, const NumberArray& mat, const NumberArray& rgba)
{
  UNPACK_ARRAY(mjtNum, size);
  UNPACK_ARRAY(mjtNum, pos);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(float, rgba);
  mjv_initGeom(geom.get(), type, size_.data(), pos_.data(), mat_.data(), rgba_.data());
}

void mjv_connector_wrapper(MjvGeom& geom, int type, mjtNum width, const NumberArray& from, const NumberArray& to)
{
  UNPACK_ARRAY(mjtNum, from);
  UNPACK_ARRAY(mjtNum, to);
  mjv_connector(geom.get(), type, width, from_.data(), to_.data());
}

void mjv_updateScene_wrapper(const MjModel& m, MjData& d, const MjvOption& opt, const MjvPerturb& pert, MjvCamera& cam, int catmask, MjvScene& scn)
{
  mjv_updateScene(m.get(), d.get(), opt.get(), pert.get(), cam.get(), catmask, scn.get());
}

void mjv_addGeoms_wrapper(const MjModel& m, MjData& d, const MjvOption& opt, const MjvPerturb& pert, int catmask, MjvScene& scn)
{
  mjv_addGeoms(m.get(), d.get(), opt.get(), pert.get(), catmask, scn.get());
}

void mjv_makeLights_wrapper(const MjModel& m, const MjData& d, MjvScene& scn)
{
  mjv_makeLights(m.get(), d.get(), scn.get());
}

void mjv_updateCamera_wrapper(const MjModel& m, const MjData& d, MjvCamera& cam, MjvScene& scn)
{
  mjv_updateCamera(m.get(), d.get(), cam.get(), scn.get());
}

void mjv_updateSkin_wrapper(const MjModel& m, const MjData& d, MjvScene& scn)
{
  mjv_updateSkin(m.get(), d.get(), scn.get());
}

void mju_writeLog_wrapper(const String& type, const String& msg)
{
  CHECK_VAL(type);
  CHECK_VAL(msg);
  mju_writeLog(type.as<const std::string>().data(), msg.as<const std::string>().data());
}

std::string mjs_getError_wrapper(MjSpec& s)
{
  return std::string(mjs_getError(s.get()));
}

int mjs_isWarning_wrapper(MjSpec& s)
{
  return mjs_isWarning(s.get());
}

void mju_zero3_wrapper(const val& res)
{
  UNPACK_VALUE(mjtNum, res);
  mju_zero3(res_.data());
}

void mju_copy3_wrapper(const val& res, const NumberArray& data)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, data);
  mju_copy3(res_.data(), data_.data());
}

void mju_scl3_wrapper(const val& res, const NumberArray& vec, mjtNum scl)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  mju_scl3(res_.data(), vec_.data(), scl);
}

void mju_add3_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  mju_add3(res_.data(), vec1_.data(), vec2_.data());
}

void mju_sub3_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  mju_sub3(res_.data(), vec1_.data(), vec2_.data());
}

void mju_addTo3_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  mju_addTo3(res_.data(), vec_.data());
}

void mju_subFrom3_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  mju_subFrom3(res_.data(), vec_.data());
}

void mju_addToScl3_wrapper(const val& res, const NumberArray& vec, mjtNum scl)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  mju_addToScl3(res_.data(), vec_.data(), scl);
}

void mju_addScl3_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2, mjtNum scl)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  mju_addScl3(res_.data(), vec1_.data(), vec2_.data(), scl);
}

mjtNum mju_normalize3_wrapper(const val& vec)
{
  UNPACK_VALUE(mjtNum, vec);
  return mju_normalize3(vec_.data());
}

mjtNum mju_norm3_wrapper(const NumberArray& vec)
{
  UNPACK_ARRAY(mjtNum, vec);
  return mju_norm3(vec_.data());
}

mjtNum mju_dot3_wrapper(const NumberArray& vec1, const NumberArray& vec2)
{
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  return mju_dot3(vec1_.data(), vec2_.data());
}

mjtNum mju_dist3_wrapper(const NumberArray& pos1, const NumberArray& pos2)
{
  UNPACK_ARRAY(mjtNum, pos1);
  UNPACK_ARRAY(mjtNum, pos2);
  return mju_dist3(pos1_.data(), pos2_.data());
}

void mju_mulMatVec3_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  mju_mulMatVec3(res_.data(), mat_.data(), vec_.data());
}

void mju_mulMatTVec3_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  mju_mulMatTVec3(res_.data(), mat_.data(), vec_.data());
}

void mju_cross_wrapper(const val& res, const NumberArray& a, const NumberArray& b)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, a);
  UNPACK_ARRAY(mjtNum, b);
  mju_cross(res_.data(), a_.data(), b_.data());
}

void mju_zero4_wrapper(const val& res)
{
  UNPACK_VALUE(mjtNum, res);
  mju_zero4(res_.data());
}

void mju_unit4_wrapper(const val& res)
{
  UNPACK_VALUE(mjtNum, res);
  mju_unit4(res_.data());
}

void mju_copy4_wrapper(const val& res, const NumberArray& data)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, data);
  mju_copy4(res_.data(), data_.data());
}

mjtNum mju_normalize4_wrapper(const val& vec)
{
  UNPACK_VALUE(mjtNum, vec);
  return mju_normalize4(vec_.data());
}

void mju_transformSpatial_wrapper(const val& res, const NumberArray& vec, int flg_force, const NumberArray& newpos, const NumberArray& oldpos, const NumberArray& rotnew2old)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_ARRAY(mjtNum, newpos);
  UNPACK_ARRAY(mjtNum, oldpos);
  UNPACK_ARRAY(mjtNum, rotnew2old);
  mju_transformSpatial(res_.data(), vec_.data(), flg_force, newpos_.data(), oldpos_.data(), rotnew2old_.data());
}

void mju_rotVecQuat_wrapper(const val& res, const NumberArray& vec, const NumberArray& quat)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_ARRAY(mjtNum, quat);
  mju_rotVecQuat(res_.data(), vec_.data(), quat_.data());
}

void mju_negQuat_wrapper(const val& res, const NumberArray& quat)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  mju_negQuat(res_.data(), quat_.data());
}

void mju_mulQuat_wrapper(const val& res, const NumberArray& quat1, const NumberArray& quat2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat1);
  UNPACK_ARRAY(mjtNum, quat2);
  mju_mulQuat(res_.data(), quat1_.data(), quat2_.data());
}

void mju_mulQuatAxis_wrapper(const val& res, const NumberArray& quat, const NumberArray& axis)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, axis);
  mju_mulQuatAxis(res_.data(), quat_.data(), axis_.data());
}

void mju_axisAngle2Quat_wrapper(const val& res, const NumberArray& axis, mjtNum angle)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, axis);
  mju_axisAngle2Quat(res_.data(), axis_.data(), angle);
}

void mju_quat2Vel_wrapper(const val& res, const NumberArray& quat, mjtNum dt)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  mju_quat2Vel(res_.data(), quat_.data(), dt);
}

void mju_subQuat_wrapper(const val& res, const NumberArray& qa, const NumberArray& qb)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, qa);
  UNPACK_ARRAY(mjtNum, qb);
  mju_subQuat(res_.data(), qa_.data(), qb_.data());
}

void mju_quat2Mat_wrapper(const val& res, const NumberArray& quat)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  mju_quat2Mat(res_.data(), quat_.data());
}

void mju_mat2Quat_wrapper(const val& quat, const NumberArray& mat)
{
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, mat);
  mju_mat2Quat(quat_.data(), mat_.data());
}

void mju_derivQuat_wrapper(const val& res, const NumberArray& quat, const NumberArray& vel)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, vel);
  mju_derivQuat(res_.data(), quat_.data(), vel_.data());
}

void mju_quatIntegrate_wrapper(const val& quat, const NumberArray& vel, mjtNum scale)
{
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, vel);
  mju_quatIntegrate(quat_.data(), vel_.data(), scale);
}

void mju_quatZ2Vec_wrapper(const val& quat, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, vec);
  mju_quatZ2Vec(quat_.data(), vec_.data());
}

int mju_mat2Rot_wrapper(const val& quat, const NumberArray& mat)
{
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, mat);
  return mju_mat2Rot(quat_.data(), mat_.data());
}

void mju_euler2Quat_wrapper(const val& quat, const NumberArray& euler, const String& seq)
{
  CHECK_VAL(seq);
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, euler);
  mju_euler2Quat(quat_.data(), euler_.data(), seq.as<const std::string>().data());
}

void mju_mulPose_wrapper(const val& posres, const val& quatres, const NumberArray& pos1, const NumberArray& quat1, const NumberArray& pos2, const NumberArray& quat2)
{
  UNPACK_VALUE(mjtNum, posres);
  UNPACK_VALUE(mjtNum, quatres);
  UNPACK_ARRAY(mjtNum, pos1);
  UNPACK_ARRAY(mjtNum, quat1);
  UNPACK_ARRAY(mjtNum, pos2);
  UNPACK_ARRAY(mjtNum, quat2);
  mju_mulPose(posres_.data(), quatres_.data(), pos1_.data(), quat1_.data(), pos2_.data(), quat2_.data());
}

void mju_negPose_wrapper(const val& posres, const val& quatres, const NumberArray& pos, const NumberArray& quat)
{
  UNPACK_VALUE(mjtNum, posres);
  UNPACK_VALUE(mjtNum, quatres);
  UNPACK_ARRAY(mjtNum, pos);
  UNPACK_ARRAY(mjtNum, quat);
  mju_negPose(posres_.data(), quatres_.data(), pos_.data(), quat_.data());
}

void mju_trnVecPose_wrapper(const val& res, const NumberArray& pos, const NumberArray& quat, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, pos);
  UNPACK_ARRAY(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, vec);
  mju_trnVecPose(res_.data(), pos_.data(), quat_.data(), vec_.data());
}

int mju_eig3_wrapper(const val& eigval, const val& eigvec, const val& quat, const NumberArray& mat)
{
  UNPACK_VALUE(mjtNum, eigval);
  UNPACK_VALUE(mjtNum, eigvec);
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, mat);
  return mju_eig3(eigval_.data(), eigvec_.data(), quat_.data(), mat_.data());
}

mjtNum mju_muscleGain_wrapper(mjtNum len, mjtNum vel, const NumberArray& lengthrange, mjtNum acc0, const NumberArray& prm)
{
  UNPACK_ARRAY(mjtNum, lengthrange);
  UNPACK_ARRAY(mjtNum, prm);
  return mju_muscleGain(len, vel, lengthrange_.data(), acc0, prm_.data());
}

mjtNum mju_muscleBias_wrapper(mjtNum len, const NumberArray& lengthrange, mjtNum acc0, const NumberArray& prm)
{
  UNPACK_ARRAY(mjtNum, lengthrange);
  UNPACK_ARRAY(mjtNum, prm);
  return mju_muscleBias(len, lengthrange_.data(), acc0, prm_.data());
}

mjtNum mju_muscleDynamics_wrapper(mjtNum ctrl, mjtNum act, const NumberArray& prm)
{
  UNPACK_ARRAY(mjtNum, prm);
  return mju_muscleDynamics(ctrl, act, prm_.data());
}

std::string mju_type2Str_wrapper(int type)
{
  return std::string(mju_type2Str(type));
}

int mju_str2Type_wrapper(const String& str)
{
  CHECK_VAL(str);
  return mju_str2Type(str.as<const std::string>().data());
}

std::string mju_writeNumBytes_wrapper(size_t nbytes)
{
  return std::string(mju_writeNumBytes(nbytes));
}

std::string mju_warningText_wrapper(int warning, size_t info)
{
  return std::string(mju_warningText(warning, info));
}

mjtNum mju_standardNormal_wrapper(const val& num2)
{
  UNPACK_VALUE(mjtNum, num2);
  return mju_standardNormal(num2_.data());
}

void mjd_quatIntegrate_wrapper(const NumberArray& vel, mjtNum scale, const val& Dquat, const val& Dvel, const val& Dscale)
{
  UNPACK_ARRAY(mjtNum, vel);
  UNPACK_VALUE(mjtNum, Dquat);
  UNPACK_VALUE(mjtNum, Dvel);
  UNPACK_VALUE(mjtNum, Dscale);
  mjd_quatIntegrate(vel_.data(), scale, Dquat_.data(), Dvel_.data(), Dscale_.data());
}

std::optional<MjsElement> mjs_attach_wrapper(MjsElement& parent, const MjsElement& child, const String& prefix, const String& suffix)
{
  CHECK_VAL(prefix);
  CHECK_VAL(suffix);
  mjsElement* result = mjs_attach(parent.get(), child.get(), prefix.as<const std::string>().data(), suffix.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsBody> mjs_addBody_wrapper(MjsBody& body, const MjsDefault& def)
{
  mjsBody* result = mjs_addBody(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjsSite> mjs_addSite_wrapper(MjsBody& body, const MjsDefault& def)
{
  mjsSite* result = mjs_addSite(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSite(result);
}

std::optional<MjsJoint> mjs_addJoint_wrapper(MjsBody& body, const MjsDefault& def)
{
  mjsJoint* result = mjs_addJoint(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsJoint(result);
}

std::optional<MjsJoint> mjs_addFreeJoint_wrapper(MjsBody& body)
{
  mjsJoint* result = mjs_addFreeJoint(body.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsJoint(result);
}

std::optional<MjsGeom> mjs_addGeom_wrapper(MjsBody& body, const MjsDefault& def)
{
  mjsGeom* result = mjs_addGeom(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsGeom(result);
}

std::optional<MjsCamera> mjs_addCamera_wrapper(MjsBody& body, const MjsDefault& def)
{
  mjsCamera* result = mjs_addCamera(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsCamera(result);
}

std::optional<MjsLight> mjs_addLight_wrapper(MjsBody& body, const MjsDefault& def)
{
  mjsLight* result = mjs_addLight(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsLight(result);
}

std::optional<MjsFrame> mjs_addFrame_wrapper(MjsBody& body, MjsFrame& parentframe)
{
  mjsFrame* result = mjs_addFrame(body.get(), parentframe.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFrame(result);
}

int mjs_delete_wrapper(MjSpec& spec, MjsElement& element)
{
  return mjs_delete(spec.get(), element.get());
}

std::optional<MjsActuator> mjs_addActuator_wrapper(MjSpec& s, const MjsDefault& def)
{
  mjsActuator* result = mjs_addActuator(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsActuator(result);
}

std::optional<MjsSensor> mjs_addSensor_wrapper(MjSpec& s)
{
  mjsSensor* result = mjs_addSensor(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSensor(result);
}

std::optional<MjsFlex> mjs_addFlex_wrapper(MjSpec& s)
{
  mjsFlex* result = mjs_addFlex(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFlex(result);
}

std::optional<MjsPair> mjs_addPair_wrapper(MjSpec& s, const MjsDefault& def)
{
  mjsPair* result = mjs_addPair(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsPair(result);
}

std::optional<MjsExclude> mjs_addExclude_wrapper(MjSpec& s)
{
  mjsExclude* result = mjs_addExclude(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsExclude(result);
}

std::optional<MjsEquality> mjs_addEquality_wrapper(MjSpec& s, const MjsDefault& def)
{
  mjsEquality* result = mjs_addEquality(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsEquality(result);
}

std::optional<MjsTendon> mjs_addTendon_wrapper(MjSpec& s, const MjsDefault& def)
{
  mjsTendon* result = mjs_addTendon(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTendon(result);
}

std::optional<MjsWrap> mjs_wrapSite_wrapper(MjsTendon& tendon, const String& name)
{
  CHECK_VAL(name);
  mjsWrap* result = mjs_wrapSite(tendon.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

std::optional<MjsWrap> mjs_wrapGeom_wrapper(MjsTendon& tendon, const String& name, const String& sidesite)
{
  CHECK_VAL(name);
  CHECK_VAL(sidesite);
  mjsWrap* result = mjs_wrapGeom(tendon.get(), name.as<const std::string>().data(), sidesite.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

std::optional<MjsWrap> mjs_wrapJoint_wrapper(MjsTendon& tendon, const String& name, double coef)
{
  CHECK_VAL(name);
  mjsWrap* result = mjs_wrapJoint(tendon.get(), name.as<const std::string>().data(), coef);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

std::optional<MjsWrap> mjs_wrapPulley_wrapper(MjsTendon& tendon, double divisor)
{
  mjsWrap* result = mjs_wrapPulley(tendon.get(), divisor);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

std::optional<MjsNumeric> mjs_addNumeric_wrapper(MjSpec& s)
{
  mjsNumeric* result = mjs_addNumeric(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsNumeric(result);
}

std::optional<MjsText> mjs_addText_wrapper(MjSpec& s)
{
  mjsText* result = mjs_addText(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsText(result);
}

std::optional<MjsTuple> mjs_addTuple_wrapper(MjSpec& s)
{
  mjsTuple* result = mjs_addTuple(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTuple(result);
}

std::optional<MjsKey> mjs_addKey_wrapper(MjSpec& s)
{
  mjsKey* result = mjs_addKey(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsKey(result);
}

std::optional<MjsPlugin> mjs_addPlugin_wrapper(MjSpec& s)
{
  mjsPlugin* result = mjs_addPlugin(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsPlugin(result);
}

std::optional<MjsDefault> mjs_addDefault_wrapper(MjSpec& s, const String& classname, const MjsDefault& parent)
{
  CHECK_VAL(classname);
  mjsDefault* result = mjs_addDefault(s.get(), classname.as<const std::string>().data(), parent.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsDefault(result);
}

std::string mjs_setToMotor_wrapper(MjsActuator& actuator)
{
  return std::string(mjs_setToMotor(actuator.get()));
}

std::string mjs_setToPosition_wrapper(MjsActuator& actuator, double kp, const val& kv, const val& dampratio, const val& timeconst, double inheritrange)
{
  UNPACK_VALUE(double, kv);
  UNPACK_VALUE(double, dampratio);
  UNPACK_VALUE(double, timeconst);
  return std::string(mjs_setToPosition(actuator.get(), kp, kv_.data(), dampratio_.data(), timeconst_.data(), inheritrange));
}

std::string mjs_setToIntVelocity_wrapper(MjsActuator& actuator, double kp, const val& kv, const val& dampratio, const val& timeconst, double inheritrange)
{
  UNPACK_VALUE(double, kv);
  UNPACK_VALUE(double, dampratio);
  UNPACK_VALUE(double, timeconst);
  return std::string(mjs_setToIntVelocity(actuator.get(), kp, kv_.data(), dampratio_.data(), timeconst_.data(), inheritrange));
}

std::string mjs_setToVelocity_wrapper(MjsActuator& actuator, double kv)
{
  return std::string(mjs_setToVelocity(actuator.get(), kv));
}

std::string mjs_setToDamper_wrapper(MjsActuator& actuator, double kv)
{
  return std::string(mjs_setToDamper(actuator.get(), kv));
}

std::string mjs_setToCylinder_wrapper(MjsActuator& actuator, double timeconst, double bias, double area, double diameter)
{
  return std::string(mjs_setToCylinder(actuator.get(), timeconst, bias, area, diameter));
}

std::string mjs_setToMuscle_wrapper(MjsActuator& actuator, const val& timeconst, double tausmooth, const val& range, double force, double scale, double lmin, double lmax, double vmax, double fpmax, double fvmax)
{
  UNPACK_VALUE(double, timeconst);
  UNPACK_VALUE(double, range);
  return std::string(mjs_setToMuscle(actuator.get(), timeconst_.data(), tausmooth, range_.data(), force, scale, lmin, lmax, vmax, fpmax, fvmax));
}

std::string mjs_setToAdhesion_wrapper(MjsActuator& actuator, double gain)
{
  return std::string(mjs_setToAdhesion(actuator.get(), gain));
}

std::optional<MjsMesh> mjs_addMesh_wrapper(MjSpec& s, const MjsDefault& def)
{
  mjsMesh* result = mjs_addMesh(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsMesh(result);
}

std::optional<MjsHField> mjs_addHField_wrapper(MjSpec& s)
{
  mjsHField* result = mjs_addHField(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsHField(result);
}

std::optional<MjsSkin> mjs_addSkin_wrapper(MjSpec& s)
{
  mjsSkin* result = mjs_addSkin(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSkin(result);
}

std::optional<MjsTexture> mjs_addTexture_wrapper(MjSpec& s)
{
  mjsTexture* result = mjs_addTexture(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTexture(result);
}

std::optional<MjsMaterial> mjs_addMaterial_wrapper(MjSpec& s, const MjsDefault& def)
{
  mjsMaterial* result = mjs_addMaterial(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsMaterial(result);
}

int mjs_makeMesh_wrapper(MjsMesh& mesh, mjtMeshBuiltin builtin, const val& params, int nparams)
{
  UNPACK_VALUE(double, params);
  return mjs_makeMesh(mesh.get(), builtin, params_.data(), nparams);
}

std::optional<MjSpec> mjs_getSpec_wrapper(MjsElement& element)
{
  mjSpec* result = mjs_getSpec(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjSpec(result);
}

std::optional<MjSpec> mjs_findSpec_wrapper(MjSpec& spec, const String& name)
{
  CHECK_VAL(name);
  mjSpec* result = mjs_findSpec(spec.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjSpec(result);
}

std::optional<MjsBody> mjs_findBody_wrapper(MjSpec& s, const String& name)
{
  CHECK_VAL(name);
  mjsBody* result = mjs_findBody(s.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjsElement> mjs_findElement_wrapper(MjSpec& s, mjtObj type, const String& name)
{
  CHECK_VAL(name);
  mjsElement* result = mjs_findElement(s.get(), type, name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsBody> mjs_findChild_wrapper(MjsBody& body, const String& name)
{
  CHECK_VAL(name);
  mjsBody* result = mjs_findChild(body.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjsBody> mjs_getParent_wrapper(MjsElement& element)
{
  mjsBody* result = mjs_getParent(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjsFrame> mjs_getFrame_wrapper(MjsElement& element)
{
  mjsFrame* result = mjs_getFrame(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFrame(result);
}

std::optional<MjsFrame> mjs_findFrame_wrapper(MjSpec& s, const String& name)
{
  CHECK_VAL(name);
  mjsFrame* result = mjs_findFrame(s.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFrame(result);
}

std::optional<MjsDefault> mjs_getDefault_wrapper(MjsElement& element)
{
  mjsDefault* result = mjs_getDefault(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsDefault(result);
}

std::optional<MjsDefault> mjs_findDefault_wrapper(MjSpec& s, const String& classname)
{
  CHECK_VAL(classname);
  mjsDefault* result = mjs_findDefault(s.get(), classname.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsDefault(result);
}

std::optional<MjsDefault> mjs_getSpecDefault_wrapper(MjSpec& s)
{
  mjsDefault* result = mjs_getSpecDefault(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsDefault(result);
}

int mjs_getId_wrapper(MjsElement& element)
{
  return mjs_getId(element.get());
}

std::optional<MjsElement> mjs_firstChild_wrapper(MjsBody& body, mjtObj type, int recurse)
{
  mjsElement* result = mjs_firstChild(body.get(), type, recurse);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsElement> mjs_nextChild_wrapper(MjsBody& body, MjsElement& child, int recurse)
{
  mjsElement* result = mjs_nextChild(body.get(), child.get(), recurse);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsElement> mjs_firstElement_wrapper(MjSpec& s, mjtObj type)
{
  mjsElement* result = mjs_firstElement(s.get(), type);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsElement> mjs_nextElement_wrapper(MjSpec& s, MjsElement& element)
{
  mjsElement* result = mjs_nextElement(s.get(), element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsElement> mjs_getWrapTarget_wrapper(MjsWrap& wrap)
{
  mjsElement* result = mjs_getWrapTarget(wrap.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsSite> mjs_getWrapSideSite_wrapper(MjsWrap& wrap)
{
  mjsSite* result = mjs_getWrapSideSite(wrap.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSite(result);
}

double mjs_getWrapDivisor_wrapper(MjsWrap& wrap)
{
  return mjs_getWrapDivisor(wrap.get());
}

double mjs_getWrapCoef_wrapper(MjsWrap& wrap)
{
  return mjs_getWrapCoef(wrap.get());
}

int mjs_setName_wrapper(MjsElement& element, const String& name)
{
  CHECK_VAL(name);
  return mjs_setName(element.get(), name.as<const std::string>().data());
}

std::string mjs_getName_wrapper(MjsElement& element)
{
  return *mjs_getName(element.get());
}

int mjs_getWrapNum_wrapper(const MjsTendon& tendonspec)
{
  return mjs_getWrapNum(tendonspec.get());
}

std::optional<MjsWrap> mjs_getWrap_wrapper(const MjsTendon& tendonspec, int i)
{
  mjsWrap* result = mjs_getWrap(tendonspec.get(), i);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

void mjs_setDefault_wrapper(MjsElement& element, const MjsDefault& def)
{
  mjs_setDefault(element.get(), def.get());
}

int mjs_setFrame_wrapper(MjsElement& dest, MjsFrame& frame)
{
  return mjs_setFrame(dest.get(), frame.get());
}

std::string mjs_resolveOrientation_wrapper(const val& quat, mjtByte degree, const String& sequence, const MjsOrientation& orientation)
{
  CHECK_VAL(sequence);
  UNPACK_VALUE(double, quat);
  return std::string(mjs_resolveOrientation(quat_.data(), degree, sequence.as<const std::string>().data(), orientation.get()));
}

void mjs_deleteUserValue_wrapper(MjsElement& element, const String& key)
{
  CHECK_VAL(key);
  mjs_deleteUserValue(element.get(), key.as<const std::string>().data());
}

int mjs_sensorDim_wrapper(const MjsSensor& sensor)
{
  return mjs_sensorDim(sensor.get());
}

void mjs_defaultSpec_wrapper(MjSpec& spec)
{
  mjs_defaultSpec(spec.get());
}

void mjs_defaultOrientation_wrapper(MjsOrientation& orient)
{
  mjs_defaultOrientation(orient.get());
}

void mjs_defaultBody_wrapper(MjsBody& body)
{
  mjs_defaultBody(body.get());
}

void mjs_defaultFrame_wrapper(MjsFrame& frame)
{
  mjs_defaultFrame(frame.get());
}

void mjs_defaultJoint_wrapper(MjsJoint& joint)
{
  mjs_defaultJoint(joint.get());
}

void mjs_defaultGeom_wrapper(MjsGeom& geom)
{
  mjs_defaultGeom(geom.get());
}

void mjs_defaultSite_wrapper(MjsSite& site)
{
  mjs_defaultSite(site.get());
}

void mjs_defaultCamera_wrapper(MjsCamera& camera)
{
  mjs_defaultCamera(camera.get());
}

void mjs_defaultLight_wrapper(MjsLight& light)
{
  mjs_defaultLight(light.get());
}

void mjs_defaultFlex_wrapper(MjsFlex& flex)
{
  mjs_defaultFlex(flex.get());
}

void mjs_defaultMesh_wrapper(MjsMesh& mesh)
{
  mjs_defaultMesh(mesh.get());
}

void mjs_defaultHField_wrapper(MjsHField& hfield)
{
  mjs_defaultHField(hfield.get());
}

void mjs_defaultSkin_wrapper(MjsSkin& skin)
{
  mjs_defaultSkin(skin.get());
}

void mjs_defaultTexture_wrapper(MjsTexture& texture)
{
  mjs_defaultTexture(texture.get());
}

void mjs_defaultMaterial_wrapper(MjsMaterial& material)
{
  mjs_defaultMaterial(material.get());
}

void mjs_defaultPair_wrapper(MjsPair& pair)
{
  mjs_defaultPair(pair.get());
}

void mjs_defaultEquality_wrapper(MjsEquality& equality)
{
  mjs_defaultEquality(equality.get());
}

void mjs_defaultTendon_wrapper(MjsTendon& tendon)
{
  mjs_defaultTendon(tendon.get());
}

void mjs_defaultActuator_wrapper(MjsActuator& actuator)
{
  mjs_defaultActuator(actuator.get());
}

void mjs_defaultSensor_wrapper(MjsSensor& sensor)
{
  mjs_defaultSensor(sensor.get());
}

void mjs_defaultNumeric_wrapper(MjsNumeric& numeric)
{
  mjs_defaultNumeric(numeric.get());
}

void mjs_defaultText_wrapper(MjsText& text)
{
  mjs_defaultText(text.get());
}

void mjs_defaultTuple_wrapper(MjsTuple& tuple)
{
  mjs_defaultTuple(tuple.get());
}

void mjs_defaultKey_wrapper(MjsKey& key)
{
  mjs_defaultKey(key.get());
}

void mjs_defaultPlugin_wrapper(MjsPlugin& plugin)
{
  mjs_defaultPlugin(plugin.get());
}

std::optional<MjsBody> mjs_asBody_wrapper(MjsElement& element)
{
  mjsBody* result = mjs_asBody(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjsGeom> mjs_asGeom_wrapper(MjsElement& element)
{
  mjsGeom* result = mjs_asGeom(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsGeom(result);
}

std::optional<MjsJoint> mjs_asJoint_wrapper(MjsElement& element)
{
  mjsJoint* result = mjs_asJoint(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsJoint(result);
}

std::optional<MjsSite> mjs_asSite_wrapper(MjsElement& element)
{
  mjsSite* result = mjs_asSite(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSite(result);
}

std::optional<MjsCamera> mjs_asCamera_wrapper(MjsElement& element)
{
  mjsCamera* result = mjs_asCamera(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsCamera(result);
}

std::optional<MjsLight> mjs_asLight_wrapper(MjsElement& element)
{
  mjsLight* result = mjs_asLight(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsLight(result);
}

std::optional<MjsFrame> mjs_asFrame_wrapper(MjsElement& element)
{
  mjsFrame* result = mjs_asFrame(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFrame(result);
}

std::optional<MjsActuator> mjs_asActuator_wrapper(MjsElement& element)
{
  mjsActuator* result = mjs_asActuator(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsActuator(result);
}

std::optional<MjsSensor> mjs_asSensor_wrapper(MjsElement& element)
{
  mjsSensor* result = mjs_asSensor(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSensor(result);
}

std::optional<MjsFlex> mjs_asFlex_wrapper(MjsElement& element)
{
  mjsFlex* result = mjs_asFlex(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFlex(result);
}

std::optional<MjsPair> mjs_asPair_wrapper(MjsElement& element)
{
  mjsPair* result = mjs_asPair(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsPair(result);
}

std::optional<MjsEquality> mjs_asEquality_wrapper(MjsElement& element)
{
  mjsEquality* result = mjs_asEquality(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsEquality(result);
}

std::optional<MjsExclude> mjs_asExclude_wrapper(MjsElement& element)
{
  mjsExclude* result = mjs_asExclude(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsExclude(result);
}

std::optional<MjsTendon> mjs_asTendon_wrapper(MjsElement& element)
{
  mjsTendon* result = mjs_asTendon(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTendon(result);
}

std::optional<MjsNumeric> mjs_asNumeric_wrapper(MjsElement& element)
{
  mjsNumeric* result = mjs_asNumeric(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsNumeric(result);
}

std::optional<MjsText> mjs_asText_wrapper(MjsElement& element)
{
  mjsText* result = mjs_asText(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsText(result);
}

std::optional<MjsTuple> mjs_asTuple_wrapper(MjsElement& element)
{
  mjsTuple* result = mjs_asTuple(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTuple(result);
}

std::optional<MjsKey> mjs_asKey_wrapper(MjsElement& element)
{
  mjsKey* result = mjs_asKey(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsKey(result);
}

std::optional<MjsMesh> mjs_asMesh_wrapper(MjsElement& element)
{
  mjsMesh* result = mjs_asMesh(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsMesh(result);
}

std::optional<MjsHField> mjs_asHField_wrapper(MjsElement& element)
{
  mjsHField* result = mjs_asHField(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsHField(result);
}

std::optional<MjsSkin> mjs_asSkin_wrapper(MjsElement& element)
{
  mjsSkin* result = mjs_asSkin(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSkin(result);
}

std::optional<MjsTexture> mjs_asTexture_wrapper(MjsElement& element)
{
  mjsTexture* result = mjs_asTexture(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTexture(result);
}

std::optional<MjsMaterial> mjs_asMaterial_wrapper(MjsElement& element)
{
  mjsMaterial* result = mjs_asMaterial(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsMaterial(result);
}

std::optional<MjsPlugin> mjs_asPlugin_wrapper(MjsElement& element)
{
  mjsPlugin* result = mjs_asPlugin(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsPlugin(result);
}

void mju_printMatSparse_wrapper(const NumberArray& mat, const NumberArray& rownnz, const NumberArray& rowadr, const NumberArray& colind)
{
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(int, rownnz);
  UNPACK_ARRAY(int, rowadr);
  UNPACK_ARRAY(int, colind);
  CHECK_SIZES(rownnz, rowadr);
  mju_printMatSparse(mat_.data(), rowadr_.size(),
                     rownnz_.data(),
                     rowadr_.data(),
                     colind_.data());
}

void mj_solveM_wrapper(const MjModel& m, MjData& d, const val& x, const NumberArray& y)
{
  UNPACK_VALUE(mjtNum, x);
  UNPACK_ARRAY(mjtNum, y);
  CHECK_SIZES(x, y);
  CHECK_DIVISIBLE(x, m.nv());
  mj_solveM(m.get(), d.get(), x_.data(), y_.data(), x_div.quot);
}

void mj_solveM2_wrapper(const MjModel& m, MjData& d,
                        const val& x, const NumberArray& y,
                        const NumberArray& sqrtInvD) {
  UNPACK_VALUE(mjtNum, x);
  UNPACK_ARRAY(mjtNum, y);
  UNPACK_ARRAY(mjtNum, sqrtInvD);
  CHECK_SIZES(x, y);
  CHECK_SIZE(sqrtInvD, m.nv());
  CHECK_DIVISIBLE(x, m.nv());
  mj_solveM2(m.get(), d.get(), x_.data(), y_.data(), sqrtInvD_.data(), x_div.quot);
}

void mj_rne_wrapper(const MjModel& m, MjData& d, int flg_acc, const val& result)
{
  UNPACK_VALUE(mjtNum, result);
  CHECK_SIZE(result, m.nv());
  mj_rne(m.get(), d.get(), flg_acc, result_.data());
}

int mj_saveLastXML_wrapper(const String& filename, const MjModel& m) {
  CHECK_VAL(filename);
  std::array<char, 1024> error;
  int result = mj_saveLastXML(filename.as<const std::string>().data(), m.get(), error.data(), error.size());
  if (!result) {
    mju_error("%s", error.data());
  }
  return result;
}

int mj_setLengthRange_wrapper(const MjModel& m, const MjData& d, int index, const MjLROpt& opt) {
  std::array<char, 1024> error;
  int result = mj_setLengthRange(m.get(), d.get(), index, opt.get(), error.data(), error.size());
  if (!result) {
    mju_error("%s", error.data());
  }
  return result;
}

void mj_constraintUpdate_wrapper(const MjModel& m, MjData& d, const NumberArray& jar, const val& cost, int flg_coneHessian)
{
  UNPACK_ARRAY(mjtNum, jar);
  UNPACK_NULLABLE_VALUE(mjtNum, cost);
  CHECK_SIZE(cost, 1);
  CHECK_SIZE(jar, d.nefc());
  mj_constraintUpdate(m.get(), d.get(), jar_.data(), cost_.data(), flg_coneHessian);
}

void mj_getState_wrapper(const MjModel& m, const MjData& d, const val& state, unsigned int spec)
{
  UNPACK_VALUE(mjtNum, state);
  CHECK_SIZE(state, mj_stateSize(m.get(), spec));
  mj_getState(m.get(), d.get(), state_.data(), spec);
}

void mj_setState_wrapper(const MjModel& m, MjData& d, const NumberArray& state, unsigned int spec)
{
  UNPACK_ARRAY(mjtNum, state);
  CHECK_SIZE(state, mj_stateSize(m.get(), spec));
  mj_setState(m.get(), d.get(), state_.data(), spec);
}

void mj_mulJacVec_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, d.nefc());
  CHECK_SIZE(vec, m.nv());
  mj_mulJacVec(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_mulJacTVec_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, d.nefc());
  mj_mulJacTVec(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_jac_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, const NumberArray& point, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  UNPACK_ARRAY(mjtNum, point);
  CHECK_SIZE(point, 3);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jac(m.get(), d.get(), jacp_.data(), jacr_.data(), point_.data(), body);
}

void mj_jacBody_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacBody(m.get(), d.get(), jacp_.data(), jacr_.data(), body);
}

void mj_jacBodyCom_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacBodyCom(m.get(), d.get(), jacp_.data(), jacr_.data(), body);
}

void mj_jacSubtreeCom_wrapper(const MjModel& m, MjData& d, const val& jacp, int body)
{
  UNPACK_VALUE(mjtNum, jacp);
  CHECK_SIZE(jacp, m.nv() * 3);
  mj_jacSubtreeCom(m.get(), d.get(), jacp_.data(), body);
}

void mj_jacGeom_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int geom)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacGeom(m.get(), d.get(), jacp_.data(), jacr_.data(), geom);
}

void mj_jacSite_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int site)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacSite(m.get(), d.get(), jacp_.data(), jacr_.data(), site);
}

void mj_jacPointAxis_wrapper(const MjModel& m, MjData& d, const val& jacPoint, const val& jacAxis, const NumberArray& point, const NumberArray& axis, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacPoint);
  UNPACK_NULLABLE_VALUE(mjtNum, jacAxis);
  UNPACK_ARRAY(mjtNum, point);
  UNPACK_ARRAY(mjtNum, axis);
  CHECK_SIZE(point, 3);
  CHECK_SIZE(axis, 3);
  CHECK_SIZE(jacPoint, m.nv() * 3);
  CHECK_SIZE(jacAxis, m.nv() * 3);
  mj_jacPointAxis(m.get(), d.get(), jacPoint_.data(), jacAxis_.data(), point_.data(), axis_.data(), body);
}

void mj_jacDot_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, const NumberArray& point, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  UNPACK_ARRAY(mjtNum, point);
  CHECK_SIZE(point, 3);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacDot(m.get(), d.get(), jacp_.data(), jacr_.data(), point_.data(), body);
}

void mj_angmomMat_wrapper(const MjModel& m, MjData& d, const val& mat, int body)
{
  UNPACK_VALUE(mjtNum, mat);
  CHECK_SIZE(mat, m.nv() * 3);
  mj_angmomMat(m.get(), d.get(), mat_.data(), body);
}

void mj_fullM_wrapper(const MjModel& m, const val& dst, const NumberArray& M)
{
  UNPACK_VALUE(mjtNum, dst);
  UNPACK_ARRAY(mjtNum, M);
  CHECK_SIZE(M, m.nM());
  CHECK_SIZE(dst, m.nv() * m.nv());
  mj_fullM(m.get(), dst_.data(), M_.data());
}

void mj_mulM_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, m.nv());
  mj_mulM(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_mulM2_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, m.nv());
  mj_mulM2(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_addM_wrapper(const MjModel& m, MjData& d, const val& dst, const val& rownnz, const val& rowadr, const val& colind)
{
  UNPACK_VALUE(mjtNum, dst);
  UNPACK_NULLABLE_VALUE(int, rownnz);
  UNPACK_NULLABLE_VALUE(int, rowadr);
  UNPACK_NULLABLE_VALUE(int, colind);
  CHECK_SIZE(rownnz, m.nv());
  CHECK_SIZE(rowadr, m.nv());
  CHECK_SIZE(colind, m.nM());
  CHECK_SIZE(dst, m.nM());
  mj_addM(m.get(), d.get(), dst_.data(), rownnz_.data(), rowadr_.data(), colind_.data());
}

void mj_applyFT_wrapper(const MjModel& m, MjData& d, const NumberArray& force, const NumberArray& torque, const NumberArray& point, int body, const val& qfrc_target)
{
  UNPACK_NULLABLE_ARRAY(mjtNum, force);
  UNPACK_NULLABLE_ARRAY(mjtNum, torque);
  UNPACK_ARRAY(mjtNum, point);
  UNPACK_VALUE(mjtNum, qfrc_target);
  CHECK_SIZE(qfrc_target, m.nv());
  CHECK_SIZE(force, 3);
  CHECK_SIZE(torque, 3);
  CHECK_SIZE(point, 3);
  mj_applyFT(m.get(), d.get(), force_.data(), torque_.data(), point_.data(), body, qfrc_target_.data());
}

mjtNum mj_geomDistance_wrapper(const MjModel& m, const MjData& d, int geom1, int geom2, mjtNum distmax, const val& fromto)
{
  UNPACK_NULLABLE_VALUE(mjtNum, fromto);
  CHECK_SIZE(fromto, 6);
  return mj_geomDistance(m.get(), d.get(), geom1, geom2, distmax, fromto_.data());
}

void mj_differentiatePos_wrapper(const MjModel& m, const val& qvel, mjtNum dt, const NumberArray& qpos1, const NumberArray& qpos2)
{
  UNPACK_VALUE(mjtNum, qvel);
  UNPACK_ARRAY(mjtNum, qpos1);
  UNPACK_ARRAY(mjtNum, qpos2);
  CHECK_SIZE(qvel, m.nv());
  CHECK_SIZE(qpos1, m.nq());
  CHECK_SIZE(qpos2, m.nq());
  mj_differentiatePos(m.get(), qvel_.data(), dt, qpos1_.data(), qpos2_.data());
}

void mj_integratePos_wrapper(const MjModel& m, const val& qpos, const NumberArray& qvel, mjtNum dt)
{
  UNPACK_VALUE(mjtNum, qpos);
  UNPACK_ARRAY(mjtNum, qvel);
  CHECK_SIZE(qpos, m.nq());
  CHECK_SIZE(qvel, m.nv());
  mj_integratePos(m.get(), qpos_.data(), qvel_.data(), dt);
}

void mj_normalizeQuat_wrapper(const MjModel& m, const val& qpos)
{
  UNPACK_VALUE(mjtNum, qpos);
  CHECK_SIZE(qpos, m.nq());
  mj_normalizeQuat(m.get(), qpos_.data());
}

void mj_multiRay_wrapper(const MjModel& m, MjData& d, const NumberArray& pnt, const NumberArray& vec, const val& geomgroup, mjtByte flg_static, int bodyexclude, const val& geomid, const val& dist, int nray, mjtNum cutoff)
{
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_VALUE(mjtByte, geomgroup);
  UNPACK_VALUE(int, geomid);
  UNPACK_VALUE(mjtNum, dist);
  CHECK_SIZE(dist, nray);
  CHECK_SIZE(geomid, nray);
  CHECK_SIZE(vec, 3 * nray);
  mj_multiRay(m.get(), d.get(), pnt_.data(), vec_.data(), geomgroup_.data(), flg_static, bodyexclude, geomid_.data(), dist_.data(), nray, cutoff);
}

void mju_zero_wrapper(const val& res)
{
  UNPACK_VALUE(mjtNum, res);
  mju_zero(res_.data(), res_.size());
}

void mju_fill_wrapper(const val& res, mjtNum val)
{
  UNPACK_VALUE(mjtNum, res);
  mju_fill(res_.data(), val, res_.size());
}

void mju_copy_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_copy(res_.data(), vec_.data(), res_.size());
}

mjtNum mju_sum_wrapper(const NumberArray& vec)
{
  UNPACK_ARRAY(mjtNum, vec);
  return mju_sum(vec_.data(), vec_.size());
}

mjtNum mju_L1_wrapper(const NumberArray& vec)
{
  UNPACK_ARRAY(mjtNum, vec);
  return mju_L1(vec_.data(), vec_.size());
}

void mju_scl_wrapper(const val& res, const NumberArray& vec, mjtNum scl)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_scl(res_.data(), vec_.data(), scl, res_.size());
}

void mju_add_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  mju_add(res_.data(), vec1_.data(), vec2_.data(), res_.size());
}

void mju_sub_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  mju_sub(res_.data(), vec1_.data(), vec2_.data(), res_.size());
}

void mju_addTo_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_addTo(res_.data(), vec_.data(), res_.size());
}

void mju_subFrom_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_subFrom(res_.data(), vec_.data(), res_.size());
}

void mju_addToScl_wrapper(const val& res, const NumberArray& vec, mjtNum scl)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_addToScl(res_.data(), vec_.data(), scl, res_.size());
}

void mju_addScl_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2, mjtNum scl)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  mju_addScl(res_.data(), vec1_.data(), vec2_.data(), scl, res_.size());
}

mjtNum mju_normalize_wrapper(const val& res)
{
  UNPACK_VALUE(mjtNum, res);
  return mju_normalize(res_.data(), res_.size());
}

mjtNum mju_norm_wrapper(const NumberArray& res)
{
  UNPACK_ARRAY(mjtNum, res);
  return mju_norm(res_.data(), res_.size());
}

mjtNum mju_dot_wrapper(const NumberArray& vec1, const NumberArray& vec2)
{
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(vec1, vec2);
  return mju_dot(vec1_.data(), vec2_.data(), vec1_.size());
}

void mju_mulMatVec_wrapper(const val& res, const NumberArray& mat,
                           const NumberArray& vec, int nr, int nc) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nr);
  CHECK_SIZE(vec, nc);
  mju_mulMatVec(res_.data(), mat_.data(), vec_.data(), nr, nc);
}

void mju_mulMatTVec_wrapper(const val& res, const NumberArray& mat,
                            const NumberArray& vec, int nr, int nc) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nc);
  CHECK_SIZE(vec, nr);
  mju_mulMatTVec(res_.data(), mat_.data(), vec_.data(), nr, nc);
}

mjtNum mju_mulVecMatVec_wrapper(const NumberArray& vec1, const NumberArray& mat, const NumberArray& vec2)
{
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec2);
  int64_t vec1_times_vec2 = vec1_.size() * vec2_.size();
  CHECK_SIZES(vec1, vec2);
  CHECK_SIZE(mat, vec1_times_vec2);
  return mju_mulVecMatVec(vec1_.data(), mat_.data(), vec2_.data(), vec1_.size());
}

void mju_transpose_wrapper(const val& res, const NumberArray& mat, int nr, int nc)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nr * nc);
  mju_transpose(res_.data(), mat_.data(), nr, nc);
}

void mju_symmetrize_wrapper(const val& res, const NumberArray& mat, int n)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, n * n);
  CHECK_SIZE(res, n * n);
  mju_symmetrize(res_.data(), mat_.data(), n);
}

void mju_eye_wrapper(const val& mat)
{
  UNPACK_VALUE(mjtNum, mat);
  CHECK_PERFECT_SQUARE(mat);
  mju_eye(mat_.data(), mat_sqrt);
}

void mju_mulMatMat_wrapper(const val& res, const NumberArray& mat1, const NumberArray& mat2, int r1, int c1, int c2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat1);
  UNPACK_ARRAY(mjtNum, mat2);
  CHECK_SIZE(res, r1 * c2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, c1 * c2);
  mju_mulMatMat(res_.data(), mat1_.data(), mat2_.data(), r1, c1, c2);
}

void mju_mulMatMatT_wrapper(const val& res, const NumberArray& mat1, const NumberArray& mat2, int r1, int c1, int r2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat1);
  UNPACK_ARRAY(mjtNum, mat2);
  CHECK_SIZE(res, r1 * r2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, r2 * c1);
  mju_mulMatMatT(res_.data(), mat1_.data(), mat2_.data(), r1, c1, r2);
}

void mju_mulMatTMat_wrapper(const val& res, const NumberArray& mat1, const NumberArray& mat2, int r1, int c1, int c2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat1);
  UNPACK_ARRAY(mjtNum, mat2);
  CHECK_SIZE(res, c1 * c2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, r1 * c2);
  mju_mulMatTMat(res_.data(), mat1_.data(), mat2_.data(), r1, c1, c2);
}

void mju_sqrMatTD_wrapper(const val& res, const NumberArray& mat, const NumberArray& diag, int nr, int nc)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, diag);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nc * nc);
  CHECK_SIZE(diag, nr);
  mju_sqrMatTD(res_.data(), mat_.data(), diag_.data(), nr, nc);
}

int mju_dense2sparse_wrapper(const val& res, const NumberArray& mat, int nr, int nc, const val& rownnz, const val& rowadr, const val& colind)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_VALUE(int, rownnz);
  UNPACK_VALUE(int, rowadr);
  UNPACK_VALUE(int, colind);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(rownnz, nr);
  CHECK_SIZE(rowadr, nr);
  CHECK_SIZE(colind, res_.size());
  return mju_dense2sparse(res_.data(), mat_.data(), nr, nc, rownnz_.data(), rowadr_.data(), colind_.data(), res_.size());
}

void mju_sparse2dense_wrapper(const val& res, const NumberArray& mat, int nr, int nc, const NumberArray& rownnz, const NumberArray& rowadr, const NumberArray& colind)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(int, rownnz);
  UNPACK_ARRAY(int, rowadr);
  UNPACK_ARRAY(int, colind);
  CHECK_SIZE(res, nr * nc);
  CHECK_SIZE(rownnz, nr);
  CHECK_SIZE(rowadr, nr);
  mju_sparse2dense(res_.data(), mat_.data(), nr, nc, rownnz_.data(), rowadr_.data(), colind_.data());
}

int mju_cholFactor_wrapper(const val& mat, mjtNum mindiag)
{
  UNPACK_VALUE(mjtNum, mat);
  CHECK_PERFECT_SQUARE(mat);
  return mju_cholFactor(mat_.data(), mat_sqrt, mindiag);
}

void mju_cholSolve_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_PERFECT_SQUARE(mat);
  CHECK_SIZE(res, mat_sqrt);
  CHECK_SIZE(vec, mat_sqrt);
  mju_cholSolve(res_.data(), mat_.data(), vec_.data(), mat_sqrt);
}

int mju_cholUpdate_wrapper(const val& mat, const val& x, int flg_plus)
{
  UNPACK_VALUE(mjtNum, mat);
  UNPACK_VALUE(mjtNum, x);
  CHECK_PERFECT_SQUARE(mat);
  CHECK_SIZE(x, mat_sqrt);
  return mju_cholUpdate(mat_.data(), x_.data(), mat_sqrt, flg_plus);
}

mjtNum mju_cholFactorBand_wrapper(const val& mat, int ntotal, int nband, int ndense, mjtNum diagadd, mjtNum diagmul)
{
  UNPACK_VALUE(mjtNum, mat);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  return mju_cholFactorBand(mat_.data(), ntotal, nband, ndense, diagadd, diagmul);
}

void mju_cholSolveBand_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec, int ntotal, int nband, int ndense)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal);
  CHECK_SIZE(vec, ntotal);
  mju_cholSolveBand(res_.data(), mat_.data(), vec_.data(), ntotal, nband, ndense);
}

void mju_band2Dense_wrapper(const val& res, const NumberArray& mat, int ntotal, int nband, int ndense, mjtByte flg_sym)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal * ntotal);
  mju_band2Dense(res_.data(), mat_.data(), ntotal, nband, ndense, flg_sym);
}

void mju_dense2Band_wrapper(const val& res, const NumberArray& mat, int ntotal, int nband, int ndense)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, ntotal * ntotal);
  CHECK_SIZE(res, (ntotal - ndense) * nband + ndense * ntotal);
  mju_dense2Band(res_.data(), mat_.data(), ntotal, nband, ndense);
}

void mju_bandMulMatVec_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec, int ntotal, int nband, int ndense, int nvec, mjtByte flg_sym)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal * nvec);
  CHECK_SIZE(vec, ntotal * nvec);
  mju_bandMulMatVec(res_.data(), mat_.data(), vec_.data(), ntotal, nband, ndense, nvec, flg_sym);
}

int mju_boxQP_wrapper(const val& res, const val& R, const val& index, const NumberArray& H, const NumberArray& g, const NumberArray& lower, const NumberArray& upper)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_VALUE(mjtNum, R);
  UNPACK_NULLABLE_VALUE(int, index);
  UNPACK_ARRAY(mjtNum, H);
  UNPACK_ARRAY(mjtNum, g);
  UNPACK_NULLABLE_ARRAY(mjtNum, lower);
  UNPACK_NULLABLE_ARRAY(mjtNum, upper);
  CHECK_SIZES(lower, res);
  CHECK_SIZES(upper, res);
  CHECK_SIZES(index, res);
  CHECK_SIZE(R, res_.size() * (res_.size() + 7))
  CHECK_PERFECT_SQUARE(H);
  CHECK_SIZES(g, res);
  return mju_boxQP(res_.data(), R_.data(), index_.data(), H_.data(), g_.data(), res_.size(), lower_.data(), upper_.data());
}

void mju_encodePyramid_wrapper(const val& pyramid, const NumberArray& force, const NumberArray& mu)
{
  UNPACK_VALUE(mjtNum, pyramid);
  UNPACK_ARRAY(mjtNum, force);
  UNPACK_ARRAY(mjtNum, mu);
  CHECK_SIZE(pyramid, 2 * mu_.size());
  CHECK_SIZE(force, mu_.size() + 1);
  mju_encodePyramid(pyramid_.data(), force_.data(), mu_.data(), mu_.size());
}

void mju_decodePyramid_wrapper(const val& force, const NumberArray& pyramid, const NumberArray& mu)
{
  UNPACK_VALUE(mjtNum, force);
  UNPACK_ARRAY(mjtNum, pyramid);
  UNPACK_ARRAY(mjtNum, mu);
  CHECK_SIZE(pyramid, 2 * mu_.size());
  CHECK_SIZE(force, mu_.size() + 1);
  mju_decodePyramid(force_.data(), pyramid_.data(), mu_.data(), mu_.size());
}

int mju_isZero_wrapper(const val& vec)
{
  UNPACK_VALUE(mjtNum, vec);
  return mju_isZero(vec_.data(), vec_.size());
}

void mju_f2n_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(float, vec);
  CHECK_SIZES(res, vec);
  mju_f2n(res_.data(), vec_.data(), res_.size());
}

void mju_n2f_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(float, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_n2f(res_.data(), vec_.data(), res_.size());
}

void mju_d2n_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(double, vec);
  CHECK_SIZES(res, vec);
  mju_d2n(res_.data(), vec_.data(), res_.size());
}

void mju_n2d_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(double, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_n2d(res_.data(), vec_.data(), res_.size());
}

void mju_insertionSort_wrapper(const val& list)
{
  UNPACK_VALUE(mjtNum, list);
  mju_insertionSort(list_.data(), list_.size());
}

void mju_insertionSortInt_wrapper(const val& list)
{
  UNPACK_VALUE(int, list);
  mju_insertionSortInt(list_.data(), list_.size());
}

void mjd_transitionFD_wrapper(const MjModel& m, MjData& d, mjtNum eps, mjtByte flg_centered, const val& A, const val& B, const val& C, const val& D)
{
  UNPACK_NULLABLE_VALUE(mjtNum, A);
  UNPACK_NULLABLE_VALUE(mjtNum, B);
  UNPACK_NULLABLE_VALUE(mjtNum, C);
  UNPACK_NULLABLE_VALUE(mjtNum, D);
  CHECK_SIZE(A, (2 * m.nv() + m.na()) * (2 * m.nv() + m.na()));
  CHECK_SIZE(B, (2 * m.nv() + m.na()) * m.nu());
  CHECK_SIZE(C, m.nsensordata() * (2 * m.nv() + m.na()));
  CHECK_SIZE(D, m.nsensordata() * m.nu());
  mjd_transitionFD(m.get(), d.get(), eps, flg_centered, A_.data(), B_.data(), C_.data(), D_.data());
}

void mjd_inverseFD_wrapper(const MjModel& m, MjData& d, mjtNum eps, mjtByte flg_actuation, const val& DfDq, const val& DfDv, const val& DfDa, const val& DsDq, const val& DsDv, const val& DsDa, const val& DmDq)
{
  UNPACK_NULLABLE_VALUE(mjtNum, DfDq);
  UNPACK_NULLABLE_VALUE(mjtNum, DfDv);
  UNPACK_NULLABLE_VALUE(mjtNum, DfDa);
  UNPACK_NULLABLE_VALUE(mjtNum, DsDq);
  UNPACK_NULLABLE_VALUE(mjtNum, DsDv);
  UNPACK_NULLABLE_VALUE(mjtNum, DsDa);
  UNPACK_NULLABLE_VALUE(mjtNum, DmDq);
  CHECK_SIZE(DfDq, m.nv() * m.nv());
  CHECK_SIZE(DfDv, m.nv() * m.nv());
  CHECK_SIZE(DfDa, m.nv() * m.nv());
  CHECK_SIZE(DsDq, m.nv() * m.nsensordata());
  CHECK_SIZE(DsDv, m.nv() * m.nsensordata());
  CHECK_SIZE(DsDa, m.nv() * m.nsensordata());
  CHECK_SIZE(DmDq, m.nv() * m.nM());
  mjd_inverseFD(m.get(), d.get(), eps, flg_actuation, DfDq_.data(), DfDv_.data(), DfDa_.data(),
                DsDq_.data(), DsDv_.data(), DsDa_.data(), DmDq_.data());
}

void mjd_subQuat_wrapper(const NumberArray& qa, const NumberArray& qb, const val& Da, const val& Db)
{
  UNPACK_ARRAY(mjtNum, qa);
  UNPACK_ARRAY(mjtNum, qb);
  UNPACK_NULLABLE_VALUE(mjtNum, Da);
  UNPACK_NULLABLE_VALUE(mjtNum, Db);
  CHECK_SIZE(qa, 4);
  CHECK_SIZE(qb, 4);
  CHECK_SIZE(Da, 9);
  CHECK_SIZE(Db, 9);
  mjd_subQuat(qa_.data(), qb_.data(), Da_.data(), Db_.data());
}

EMSCRIPTEN_BINDINGS(mujoco_functions) {
  function("mj_resetCallbacks", &mj_resetCallbacks);
  function("mj_version", &mj_version);
  function("mju_bandDiag", &mju_bandDiag);
  function("mju_springDamper", &mju_springDamper);
  function("mju_min", &mju_min);
  function("mju_max", &mju_max);
  function("mju_clip", &mju_clip);
  function("mju_sign", &mju_sign);
  function("mju_round", &mju_round);
  function("mju_isBad", &mju_isBad);
  function("mju_Halton", &mju_Halton);
  function("mju_sigmoid", &mju_sigmoid);
  function("mj_copyBack", &mj_copyBack_wrapper);
  function("mj_step", &mj_step_wrapper);
  function("mj_step1", &mj_step1_wrapper);
  function("mj_step2", &mj_step2_wrapper);
  function("mj_forward", &mj_forward_wrapper);
  function("mj_inverse", &mj_inverse_wrapper);
  function("mj_forwardSkip", &mj_forwardSkip_wrapper);
  function("mj_inverseSkip", &mj_inverseSkip_wrapper);
  function("mj_defaultLROpt", &mj_defaultLROpt_wrapper);
  function("mj_defaultSolRefImp", &mj_defaultSolRefImp_wrapper);
  function("mj_defaultOption", &mj_defaultOption_wrapper);
  function("mj_defaultVisual", &mj_defaultVisual_wrapper);
  function("mj_sizeModel", &mj_sizeModel_wrapper);
  function("mj_resetData", &mj_resetData_wrapper);
  function("mj_resetDataDebug", &mj_resetDataDebug_wrapper);
  function("mj_resetDataKeyframe", &mj_resetDataKeyframe_wrapper);
  function("mj_setConst", &mj_setConst_wrapper);
  function("mjs_activatePlugin", &mjs_activatePlugin_wrapper);
  function("mjs_setDeepCopy", &mjs_setDeepCopy_wrapper);
  function("mj_printFormattedModel", &mj_printFormattedModel_wrapper);
  function("mj_printModel", &mj_printModel_wrapper);
  function("mj_printFormattedData", &mj_printFormattedData_wrapper);
  function("mj_printData", &mj_printData_wrapper);
  function("mju_printMat", &mju_printMat_wrapper);
  function("mj_printScene", &mj_printScene_wrapper);
  function("mj_printFormattedScene", &mj_printFormattedScene_wrapper);
  function("mj_fwdPosition", &mj_fwdPosition_wrapper);
  function("mj_fwdVelocity", &mj_fwdVelocity_wrapper);
  function("mj_fwdActuation", &mj_fwdActuation_wrapper);
  function("mj_fwdAcceleration", &mj_fwdAcceleration_wrapper);
  function("mj_fwdConstraint", &mj_fwdConstraint_wrapper);
  function("mj_Euler", &mj_Euler_wrapper);
  function("mj_RungeKutta", &mj_RungeKutta_wrapper);
  function("mj_implicit", &mj_implicit_wrapper);
  function("mj_invPosition", &mj_invPosition_wrapper);
  function("mj_invVelocity", &mj_invVelocity_wrapper);
  function("mj_invConstraint", &mj_invConstraint_wrapper);
  function("mj_compareFwdInv", &mj_compareFwdInv_wrapper);
  function("mj_sensorPos", &mj_sensorPos_wrapper);
  function("mj_sensorVel", &mj_sensorVel_wrapper);
  function("mj_sensorAcc", &mj_sensorAcc_wrapper);
  function("mj_energyPos", &mj_energyPos_wrapper);
  function("mj_energyVel", &mj_energyVel_wrapper);
  function("mj_checkPos", &mj_checkPos_wrapper);
  function("mj_checkVel", &mj_checkVel_wrapper);
  function("mj_checkAcc", &mj_checkAcc_wrapper);
  function("mj_kinematics", &mj_kinematics_wrapper);
  function("mj_comPos", &mj_comPos_wrapper);
  function("mj_camlight", &mj_camlight_wrapper);
  function("mj_flex", &mj_flex_wrapper);
  function("mj_tendon", &mj_tendon_wrapper);
  function("mj_transmission", &mj_transmission_wrapper);
  function("mj_crb", &mj_crb_wrapper);
  function("mj_makeM", &mj_makeM_wrapper);
  function("mj_factorM", &mj_factorM_wrapper);
  function("mj_comVel", &mj_comVel_wrapper);
  function("mj_passive", &mj_passive_wrapper);
  function("mj_subtreeVel", &mj_subtreeVel_wrapper);
  function("mj_rnePostConstraint", &mj_rnePostConstraint_wrapper);
  function("mj_collision", &mj_collision_wrapper);
  function("mj_makeConstraint", &mj_makeConstraint_wrapper);
  function("mj_island", &mj_island_wrapper);
  function("mj_projectConstraint", &mj_projectConstraint_wrapper);
  function("mj_referenceConstraint", &mj_referenceConstraint_wrapper);
  function("mj_stateSize", &mj_stateSize_wrapper);
  function("mj_extractState", &mj_extractState_wrapper);
  function("mj_setKeyframe", &mj_setKeyframe_wrapper);
  function("mj_addContact", &mj_addContact_wrapper);
  function("mj_isPyramidal", &mj_isPyramidal_wrapper);
  function("mj_isSparse", &mj_isSparse_wrapper);
  function("mj_isDual", &mj_isDual_wrapper);
  function("mj_name2id", &mj_name2id_wrapper);
  function("mj_id2name", &mj_id2name_wrapper);
  function("mj_objectVelocity", &mj_objectVelocity_wrapper);
  function("mj_objectAcceleration", &mj_objectAcceleration_wrapper);
  function("mj_contactForce", &mj_contactForce_wrapper);
  function("mj_local2Global", &mj_local2Global_wrapper);
  function("mj_getTotalmass", &mj_getTotalmass_wrapper);
  function("mj_setTotalmass", &mj_setTotalmass_wrapper);
  function("mj_versionString", &mj_versionString_wrapper);
  function("mj_ray", &mj_ray_wrapper);
  function("mj_rayHfield", &mj_rayHfield_wrapper);
  function("mj_rayMesh", &mj_rayMesh_wrapper);
  function("mju_rayGeom", &mju_rayGeom_wrapper);
  function("mju_rayFlex", &mju_rayFlex_wrapper);
  function("mju_raySkin", &mju_raySkin_wrapper);
  function("mjv_defaultCamera", &mjv_defaultCamera_wrapper);
  function("mjv_defaultFreeCamera", &mjv_defaultFreeCamera_wrapper);
  function("mjv_defaultPerturb", &mjv_defaultPerturb_wrapper);
  function("mjv_room2model", &mjv_room2model_wrapper);
  function("mjv_model2room", &mjv_model2room_wrapper);
  function("mjv_cameraInModel", &mjv_cameraInModel_wrapper);
  function("mjv_cameraInRoom", &mjv_cameraInRoom_wrapper);
  function("mjv_frustumHeight", &mjv_frustumHeight_wrapper);
  function("mjv_alignToCamera", &mjv_alignToCamera_wrapper);
  function("mjv_moveCamera", &mjv_moveCamera_wrapper);
  function("mjv_movePerturb", &mjv_movePerturb_wrapper);
  function("mjv_moveModel", &mjv_moveModel_wrapper);
  function("mjv_initPerturb", &mjv_initPerturb_wrapper);
  function("mjv_applyPerturbPose", &mjv_applyPerturbPose_wrapper);
  function("mjv_applyPerturbForce", &mjv_applyPerturbForce_wrapper);
  function("mjv_select", &mjv_select_wrapper);
  function("mjv_defaultOption", &mjv_defaultOption_wrapper);
  function("mjv_defaultFigure", &mjv_defaultFigure_wrapper);
  function("mjv_initGeom", &mjv_initGeom_wrapper);
  function("mjv_connector", &mjv_connector_wrapper);
  function("mjv_updateScene", &mjv_updateScene_wrapper);
  function("mjv_addGeoms", &mjv_addGeoms_wrapper);
  function("mjv_makeLights", &mjv_makeLights_wrapper);
  function("mjv_updateCamera", &mjv_updateCamera_wrapper);
  function("mjv_updateSkin", &mjv_updateSkin_wrapper);
  function("mju_writeLog", &mju_writeLog_wrapper);
  function("mjs_getError", &mjs_getError_wrapper);
  function("mjs_isWarning", &mjs_isWarning_wrapper);
  function("mju_zero3", &mju_zero3_wrapper);
  function("mju_copy3", &mju_copy3_wrapper);
  function("mju_scl3", &mju_scl3_wrapper);
  function("mju_add3", &mju_add3_wrapper);
  function("mju_sub3", &mju_sub3_wrapper);
  function("mju_addTo3", &mju_addTo3_wrapper);
  function("mju_subFrom3", &mju_subFrom3_wrapper);
  function("mju_addToScl3", &mju_addToScl3_wrapper);
  function("mju_addScl3", &mju_addScl3_wrapper);
  function("mju_normalize3", &mju_normalize3_wrapper);
  function("mju_norm3", &mju_norm3_wrapper);
  function("mju_dot3", &mju_dot3_wrapper);
  function("mju_dist3", &mju_dist3_wrapper);
  function("mju_mulMatVec3", &mju_mulMatVec3_wrapper);
  function("mju_mulMatTVec3", &mju_mulMatTVec3_wrapper);
  function("mju_cross", &mju_cross_wrapper);
  function("mju_zero4", &mju_zero4_wrapper);
  function("mju_unit4", &mju_unit4_wrapper);
  function("mju_copy4", &mju_copy4_wrapper);
  function("mju_normalize4", &mju_normalize4_wrapper);
  function("mju_transformSpatial", &mju_transformSpatial_wrapper);
  function("mju_rotVecQuat", &mju_rotVecQuat_wrapper);
  function("mju_negQuat", &mju_negQuat_wrapper);
  function("mju_mulQuat", &mju_mulQuat_wrapper);
  function("mju_mulQuatAxis", &mju_mulQuatAxis_wrapper);
  function("mju_axisAngle2Quat", &mju_axisAngle2Quat_wrapper);
  function("mju_quat2Vel", &mju_quat2Vel_wrapper);
  function("mju_subQuat", &mju_subQuat_wrapper);
  function("mju_quat2Mat", &mju_quat2Mat_wrapper);
  function("mju_mat2Quat", &mju_mat2Quat_wrapper);
  function("mju_derivQuat", &mju_derivQuat_wrapper);
  function("mju_quatIntegrate", &mju_quatIntegrate_wrapper);
  function("mju_quatZ2Vec", &mju_quatZ2Vec_wrapper);
  function("mju_mat2Rot", &mju_mat2Rot_wrapper);
  function("mju_euler2Quat", &mju_euler2Quat_wrapper);
  function("mju_mulPose", &mju_mulPose_wrapper);
  function("mju_negPose", &mju_negPose_wrapper);
  function("mju_trnVecPose", &mju_trnVecPose_wrapper);
  function("mju_eig3", &mju_eig3_wrapper);
  function("mju_muscleGain", &mju_muscleGain_wrapper);
  function("mju_muscleBias", &mju_muscleBias_wrapper);
  function("mju_muscleDynamics", &mju_muscleDynamics_wrapper);
  function("mju_type2Str", &mju_type2Str_wrapper);
  function("mju_str2Type", &mju_str2Type_wrapper);
  function("mju_writeNumBytes", &mju_writeNumBytes_wrapper);
  function("mju_warningText", &mju_warningText_wrapper);
  function("mju_standardNormal", &mju_standardNormal_wrapper);
  function("mjd_quatIntegrate", &mjd_quatIntegrate_wrapper);
  function("mjs_attach", &mjs_attach_wrapper);
  function("mjs_addBody", &mjs_addBody_wrapper);
  function("mjs_addSite", &mjs_addSite_wrapper);
  function("mjs_addJoint", &mjs_addJoint_wrapper);
  function("mjs_addFreeJoint", &mjs_addFreeJoint_wrapper);
  function("mjs_addGeom", &mjs_addGeom_wrapper);
  function("mjs_addCamera", &mjs_addCamera_wrapper);
  function("mjs_addLight", &mjs_addLight_wrapper);
  function("mjs_addFrame", &mjs_addFrame_wrapper);
  function("mjs_delete", &mjs_delete_wrapper);
  function("mjs_addActuator", &mjs_addActuator_wrapper);
  function("mjs_addSensor", &mjs_addSensor_wrapper);
  function("mjs_addFlex", &mjs_addFlex_wrapper);
  function("mjs_addPair", &mjs_addPair_wrapper);
  function("mjs_addExclude", &mjs_addExclude_wrapper);
  function("mjs_addEquality", &mjs_addEquality_wrapper);
  function("mjs_addTendon", &mjs_addTendon_wrapper);
  function("mjs_wrapSite", &mjs_wrapSite_wrapper);
  function("mjs_wrapGeom", &mjs_wrapGeom_wrapper);
  function("mjs_wrapJoint", &mjs_wrapJoint_wrapper);
  function("mjs_wrapPulley", &mjs_wrapPulley_wrapper);
  function("mjs_addNumeric", &mjs_addNumeric_wrapper);
  function("mjs_addText", &mjs_addText_wrapper);
  function("mjs_addTuple", &mjs_addTuple_wrapper);
  function("mjs_addKey", &mjs_addKey_wrapper);
  function("mjs_addPlugin", &mjs_addPlugin_wrapper);
  function("mjs_addDefault", &mjs_addDefault_wrapper);
  function("mjs_setToMotor", &mjs_setToMotor_wrapper);
  function("mjs_setToPosition", &mjs_setToPosition_wrapper);
  function("mjs_setToIntVelocity", &mjs_setToIntVelocity_wrapper);
  function("mjs_setToVelocity", &mjs_setToVelocity_wrapper);
  function("mjs_setToDamper", &mjs_setToDamper_wrapper);
  function("mjs_setToCylinder", &mjs_setToCylinder_wrapper);
  function("mjs_setToMuscle", &mjs_setToMuscle_wrapper);
  function("mjs_setToAdhesion", &mjs_setToAdhesion_wrapper);
  function("mjs_addMesh", &mjs_addMesh_wrapper);
  function("mjs_addHField", &mjs_addHField_wrapper);
  function("mjs_addSkin", &mjs_addSkin_wrapper);
  function("mjs_addTexture", &mjs_addTexture_wrapper);
  function("mjs_addMaterial", &mjs_addMaterial_wrapper);
  function("mjs_makeMesh", &mjs_makeMesh_wrapper);
  function("mjs_getSpec", &mjs_getSpec_wrapper);
  function("mjs_findSpec", &mjs_findSpec_wrapper);
  function("mjs_findBody", &mjs_findBody_wrapper);
  function("mjs_findElement", &mjs_findElement_wrapper);
  function("mjs_findChild", &mjs_findChild_wrapper);
  function("mjs_getParent", &mjs_getParent_wrapper);
  function("mjs_getFrame", &mjs_getFrame_wrapper);
  function("mjs_findFrame", &mjs_findFrame_wrapper);
  function("mjs_getDefault", &mjs_getDefault_wrapper);
  function("mjs_findDefault", &mjs_findDefault_wrapper);
  function("mjs_getSpecDefault", &mjs_getSpecDefault_wrapper);
  function("mjs_getId", &mjs_getId_wrapper);
  function("mjs_firstChild", &mjs_firstChild_wrapper);
  function("mjs_nextChild", &mjs_nextChild_wrapper);
  function("mjs_firstElement", &mjs_firstElement_wrapper);
  function("mjs_nextElement", &mjs_nextElement_wrapper);
  function("mjs_getWrapTarget", &mjs_getWrapTarget_wrapper);
  function("mjs_getWrapSideSite", &mjs_getWrapSideSite_wrapper);
  function("mjs_getWrapDivisor", &mjs_getWrapDivisor_wrapper);
  function("mjs_getWrapCoef", &mjs_getWrapCoef_wrapper);
  function("mjs_setName", &mjs_setName_wrapper);
  function("mjs_getName", &mjs_getName_wrapper);
  function("mjs_getWrapNum", &mjs_getWrapNum_wrapper);
  function("mjs_getWrap", &mjs_getWrap_wrapper);
  function("mjs_setDefault", &mjs_setDefault_wrapper);
  function("mjs_setFrame", &mjs_setFrame_wrapper);
  function("mjs_resolveOrientation", &mjs_resolveOrientation_wrapper);
  function("mjs_deleteUserValue", &mjs_deleteUserValue_wrapper);
  function("mjs_sensorDim", &mjs_sensorDim_wrapper);
  function("mjs_defaultSpec", &mjs_defaultSpec_wrapper);
  function("mjs_defaultOrientation", &mjs_defaultOrientation_wrapper);
  function("mjs_defaultBody", &mjs_defaultBody_wrapper);
  function("mjs_defaultFrame", &mjs_defaultFrame_wrapper);
  function("mjs_defaultJoint", &mjs_defaultJoint_wrapper);
  function("mjs_defaultGeom", &mjs_defaultGeom_wrapper);
  function("mjs_defaultSite", &mjs_defaultSite_wrapper);
  function("mjs_defaultCamera", &mjs_defaultCamera_wrapper);
  function("mjs_defaultLight", &mjs_defaultLight_wrapper);
  function("mjs_defaultFlex", &mjs_defaultFlex_wrapper);
  function("mjs_defaultMesh", &mjs_defaultMesh_wrapper);
  function("mjs_defaultHField", &mjs_defaultHField_wrapper);
  function("mjs_defaultSkin", &mjs_defaultSkin_wrapper);
  function("mjs_defaultTexture", &mjs_defaultTexture_wrapper);
  function("mjs_defaultMaterial", &mjs_defaultMaterial_wrapper);
  function("mjs_defaultPair", &mjs_defaultPair_wrapper);
  function("mjs_defaultEquality", &mjs_defaultEquality_wrapper);
  function("mjs_defaultTendon", &mjs_defaultTendon_wrapper);
  function("mjs_defaultActuator", &mjs_defaultActuator_wrapper);
  function("mjs_defaultSensor", &mjs_defaultSensor_wrapper);
  function("mjs_defaultNumeric", &mjs_defaultNumeric_wrapper);
  function("mjs_defaultText", &mjs_defaultText_wrapper);
  function("mjs_defaultTuple", &mjs_defaultTuple_wrapper);
  function("mjs_defaultKey", &mjs_defaultKey_wrapper);
  function("mjs_defaultPlugin", &mjs_defaultPlugin_wrapper);
  function("mjs_asBody", &mjs_asBody_wrapper);
  function("mjs_asGeom", &mjs_asGeom_wrapper);
  function("mjs_asJoint", &mjs_asJoint_wrapper);
  function("mjs_asSite", &mjs_asSite_wrapper);
  function("mjs_asCamera", &mjs_asCamera_wrapper);
  function("mjs_asLight", &mjs_asLight_wrapper);
  function("mjs_asFrame", &mjs_asFrame_wrapper);
  function("mjs_asActuator", &mjs_asActuator_wrapper);
  function("mjs_asSensor", &mjs_asSensor_wrapper);
  function("mjs_asFlex", &mjs_asFlex_wrapper);
  function("mjs_asPair", &mjs_asPair_wrapper);
  function("mjs_asEquality", &mjs_asEquality_wrapper);
  function("mjs_asExclude", &mjs_asExclude_wrapper);
  function("mjs_asTendon", &mjs_asTendon_wrapper);
  function("mjs_asNumeric", &mjs_asNumeric_wrapper);
  function("mjs_asText", &mjs_asText_wrapper);
  function("mjs_asTuple", &mjs_asTuple_wrapper);
  function("mjs_asKey", &mjs_asKey_wrapper);
  function("mjs_asMesh", &mjs_asMesh_wrapper);
  function("mjs_asHField", &mjs_asHField_wrapper);
  function("mjs_asSkin", &mjs_asSkin_wrapper);
  function("mjs_asTexture", &mjs_asTexture_wrapper);
  function("mjs_asMaterial", &mjs_asMaterial_wrapper);
  function("mjs_asPlugin", &mjs_asPlugin_wrapper);
  function("error", &error_wrapper);
  function("mju_printMatSparse", &mju_printMatSparse_wrapper);
  function("mj_solveM", &mj_solveM_wrapper);
  function("mj_solveM2", &mj_solveM2_wrapper);
  function("mj_rne", &mj_rne_wrapper);
  function("mj_saveLastXML", &mj_saveLastXML_wrapper);
  function("mj_setLengthRange", &mj_setLengthRange_wrapper);
  function("mj_constraintUpdate", &mj_constraintUpdate_wrapper);
  function("mj_getState", &mj_getState_wrapper);
  function("mj_setState", &mj_setState_wrapper);
  function("mj_mulJacVec", &mj_mulJacVec_wrapper);
  function("mj_mulJacTVec", &mj_mulJacTVec_wrapper);
  function("mj_jac", &mj_jac_wrapper);
  function("mj_jacBody", &mj_jacBody_wrapper);
  function("mj_jacBodyCom", &mj_jacBodyCom_wrapper);
  function("mj_jacSubtreeCom", &mj_jacSubtreeCom_wrapper);
  function("mj_jacGeom", &mj_jacGeom_wrapper);
  function("mj_jacSite", &mj_jacSite_wrapper);
  function("mj_jacPointAxis", &mj_jacPointAxis_wrapper);
  function("mj_jacDot", &mj_jacDot_wrapper);
  function("mj_angmomMat", &mj_angmomMat_wrapper);
  function("mj_fullM", &mj_fullM_wrapper);
  function("mj_mulM", &mj_mulM_wrapper);
  function("mj_mulM2", &mj_mulM2_wrapper);
  function("mj_addM", &mj_addM_wrapper);
  function("mj_applyFT", &mj_applyFT_wrapper);
  function("mj_geomDistance", &mj_geomDistance_wrapper);
  function("mj_differentiatePos", &mj_differentiatePos_wrapper);
  function("mj_integratePos", &mj_integratePos_wrapper);
  function("mj_normalizeQuat", &mj_normalizeQuat_wrapper);
  function("mj_multiRay", &mj_multiRay_wrapper);
  function("mju_zero", &mju_zero_wrapper);
  function("mju_fill", &mju_fill_wrapper);
  function("mju_copy", &mju_copy_wrapper);
  function("mju_sum", &mju_sum_wrapper);
  function("mju_L1", &mju_L1_wrapper);
  function("mju_scl", &mju_scl_wrapper);
  function("mju_add", &mju_add_wrapper);
  function("mju_sub", &mju_sub_wrapper);
  function("mju_addTo", &mju_addTo_wrapper);
  function("mju_subFrom", &mju_subFrom_wrapper);
  function("mju_addToScl", &mju_addToScl_wrapper);
  function("mju_addScl", &mju_addScl_wrapper);
  function("mju_normalize", &mju_normalize_wrapper);
  function("mju_norm", &mju_norm_wrapper);
  function("mju_dot", &mju_dot_wrapper);
  function("mju_mulMatVec", &mju_mulMatVec_wrapper);
  function("mju_mulMatTVec", &mju_mulMatTVec_wrapper);
  function("mju_mulVecMatVec", &mju_mulVecMatVec_wrapper);
  function("mju_transpose", &mju_transpose_wrapper);
  function("mju_symmetrize", &mju_symmetrize_wrapper);
  function("mju_eye", &mju_eye_wrapper);
  function("mju_mulMatMat", &mju_mulMatMat_wrapper);
  function("mju_mulMatMatT", &mju_mulMatMatT_wrapper);
  function("mju_mulMatTMat", &mju_mulMatTMat_wrapper);
  function("mju_sqrMatTD", &mju_sqrMatTD_wrapper);
  function("mju_dense2sparse", &mju_dense2sparse_wrapper);
  function("mju_sparse2dense", &mju_sparse2dense_wrapper);
  function("mju_cholFactor", &mju_cholFactor_wrapper);
  function("mju_cholSolve", &mju_cholSolve_wrapper);
  function("mju_cholUpdate", &mju_cholUpdate_wrapper);
  function("mju_cholFactorBand", &mju_cholFactorBand_wrapper);
  function("mju_cholSolveBand", &mju_cholSolveBand_wrapper);
  function("mju_band2Dense", &mju_band2Dense_wrapper);
  function("mju_dense2Band", &mju_dense2Band_wrapper);
  function("mju_bandMulMatVec", &mju_bandMulMatVec_wrapper);
  function("mju_boxQP", &mju_boxQP_wrapper);
  function("mju_encodePyramid", &mju_encodePyramid_wrapper);
  function("mju_decodePyramid", &mju_decodePyramid_wrapper);
  function("mju_isZero", &mju_isZero_wrapper);
  function("mju_f2n", &mju_f2n_wrapper);
  function("mju_n2f", &mju_n2f_wrapper);
  function("mju_d2n", &mju_d2n_wrapper);
  function("mju_n2d", &mju_n2d_wrapper);
  function("mju_insertionSort", &mju_insertionSort_wrapper);
  function("mju_insertionSortInt", &mju_insertionSortInt_wrapper);
  function("mjd_transitionFD", &mjd_transitionFD_wrapper);
  function("mjd_inverseFD", &mjd_inverseFD_wrapper);
  function("mjd_subQuat", &mjd_subQuat_wrapper);
  class_<WasmBuffer<float>>("FloatBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<float>::FromArray)
      .function("GetPointer", &WasmBuffer<float>::GetPointer)
      .function("GetElementCount", &WasmBuffer<float>::GetElementCount)
      .function("GetView", &WasmBuffer<float>::GetView);
  class_<WasmBuffer<double>>("DoubleBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<double>::FromArray)
      .function("GetPointer", &WasmBuffer<double>::GetPointer)
      .function("GetElementCount", &WasmBuffer<double>::GetElementCount)
      .function("GetView", &WasmBuffer<double>::GetView);
  class_<WasmBuffer<int>>("IntBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<int>::FromArray)
      .function("GetPointer", &WasmBuffer<int>::GetPointer)
      .function("GetElementCount", &WasmBuffer<int>::GetElementCount)
      .function("GetView", &WasmBuffer<int>::GetView);
  register_vector<std::string>("mjStringVec");
  register_vector<int>("mjIntVec");
  register_vector<mjIntVec>("mjIntVecVec");
  register_vector<float>("mjFloatVec");
  register_vector<mjFloatVec>("mjFloatVecVec");
  register_vector<double>("mjDoubleVec");
  // register_type gives better type information (val is mapped to any by default)
  register_type<NumberArray>("number[]");
  register_type<String>("string");
  register_vector<uint8_t>("mjByteVec");
  register_optional<MjsElement>();
  register_optional<MjsBody>();
  register_optional<MjsSite>();
  register_optional<MjsJoint>();
  register_optional<MjsGeom>();
  register_optional<MjsCamera>();
  register_optional<MjsLight>();
  register_optional<MjsFrame>();
  register_optional<MjsActuator>();
  register_optional<MjsSensor>();
  register_optional<MjsFlex>();
  register_optional<MjsPair>();
  register_optional<MjsExclude>();
  register_optional<MjsEquality>();
  register_optional<MjsTendon>();
  register_optional<MjsWrap>();
  register_optional<MjsNumeric>();
  register_optional<MjsText>();
  register_optional<MjsTuple>();
  register_optional<MjsKey>();
  register_optional<MjsPlugin>();
  register_optional<MjsDefault>();
  register_optional<MjsMesh>();
  register_optional<MjsHField>();
  register_optional<MjsSkin>();
  register_optional<MjsTexture>();
  register_optional<MjsMaterial>();
  register_optional<MjSpec>();
}

}  // namespace mujoco::wasm
// NOLINTEND(whitespace/semicolon)
// NOLINTEND(whitespace/line_length)
