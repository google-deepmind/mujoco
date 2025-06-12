// Copyright 2022 DeepMind Technologies Limited
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
// ==============================================================================
// THIS FILE IS AUTO-GENERATED

using System;
using System.Runtime.InteropServices;
using System.Text;
using System.Collections.Generic;
using SizeDict = System.Collections.Generic.Dictionary<string, string[]>;
using NestedDictionary = System.Collections.Generic.Dictionary<string, System.Collections.Generic.Dictionary<string, string[]>>;
namespace Mujoco {
public static class MujocoLib {

// ----------------------------------Constants----------------------------------

public const bool THIRD_PARTY_MUJOCO_MJDATA_H_ = true;
public const bool THIRD_PARTY_MUJOCO_MJEXPORT_H_ = true;
public const bool MUJOCO_HELPER_DLL_LOCAL = true;
public const bool MUJOCO_HELPER_DLL_IMPORT = true;
public const bool MUJOCO_HELPER_DLL_EXPORT = true;
public const bool MJAPI = true;
public const bool MJLOCAL = true;
public const bool THIRD_PARTY_MUJOCO_MJMACRO_H_ = true;
public const bool THIRD_PARTY_MUJOCO_MJMODEL_H_ = true;
public const double mjPI = 3.141592653589793;
public const double mjMAXVAL = 10000000000.0;
public const double mjMINMU = 1e-05;
public const double mjMINIMP = 0.0001;
public const double mjMAXIMP = 0.9999;
public const int mjMAXCONPAIR = 50;
public const int mjMAXTREEDEPTH = 50;
public const int mjMAXFLEXNODES = 27;
public const int mjNEQDATA = 11;
public const int mjNDYN = 10;
public const int mjNGAIN = 10;
public const int mjNBIAS = 10;
public const int mjNFLUID = 12;
public const int mjNREF = 2;
public const int mjNIMP = 5;
public const int mjNSOLVER = 200;
public const int mjNISLAND = 20;
public const bool THIRD_PARTY_MUJOCO_INCLUDE_MJPLUGIN_H_ = true;
public const bool mjEXTERNC = true;
public const bool THIRD_PARTY_MUJOCO_MJRENDER_H_ = true;
public const int mjNAUX = 10;
public const int mjMAXTEXTURE = 1000;
public const int mjMAXMATERIAL = 1000;
public const bool THIRD_PARTY_MUJOCO_INCLUDE_MJSAN_H_ = true;
public const bool THIRD_PARTY_MUJOCO_INCLUDE_MJSPEC_H_ = true;
public const bool THIRD_PARTY_MUJOCO_INCLUDE_MJTHREAD_H_ = true;
public const int mjMAXTHREAD = 128;
public const bool THIRD_PARTY_MUJOCO_INCLUDE_MJTNUM_H_ = true;
public const double mjMINVAL = 1e-15;
public const bool THIRD_PARTY_MUJOCO_MJUI_H_ = true;
public const int mjMAXUISECT = 10;
public const int mjMAXUIITEM = 200;
public const int mjMAXUITEXT = 300;
public const int mjMAXUINAME = 40;
public const int mjMAXUIMULTI = 35;
public const int mjMAXUIEDIT = 7;
public const int mjMAXUIRECT = 25;
public const int mjSEPCLOSED = 1000;
public const int mjPRESERVE = 2000;
public const int mjKEY_ESCAPE = 256;
public const int mjKEY_ENTER = 257;
public const int mjKEY_TAB = 258;
public const int mjKEY_BACKSPACE = 259;
public const int mjKEY_INSERT = 260;
public const int mjKEY_DELETE = 261;
public const int mjKEY_RIGHT = 262;
public const int mjKEY_LEFT = 263;
public const int mjKEY_DOWN = 264;
public const int mjKEY_UP = 265;
public const int mjKEY_PAGE_UP = 266;
public const int mjKEY_PAGE_DOWN = 267;
public const int mjKEY_HOME = 268;
public const int mjKEY_END = 269;
public const int mjKEY_F1 = 290;
public const int mjKEY_F2 = 291;
public const int mjKEY_F3 = 292;
public const int mjKEY_F4 = 293;
public const int mjKEY_F5 = 294;
public const int mjKEY_F6 = 295;
public const int mjKEY_F7 = 296;
public const int mjKEY_F8 = 297;
public const int mjKEY_F9 = 298;
public const int mjKEY_F10 = 299;
public const int mjKEY_F11 = 300;
public const int mjKEY_F12 = 301;
public const int mjKEY_NUMPAD_0 = 320;
public const int mjKEY_NUMPAD_9 = 329;
public const bool THIRD_PARTY_MUJOCO_MJVISUALIZE_H_ = true;
public const int mjNGROUP = 6;
public const int mjMAXLIGHT = 100;
public const int mjMAXOVERLAY = 500;
public const int mjMAXLINE = 100;
public const int mjMAXLINEPNT = 1000;
public const int mjMAXPLANEGRID = 200;
public const bool THIRD_PARTY_MUJOCO_MJXMACRO_H_ = true;
public const bool THIRD_PARTY_MUJOCO_MUJOCO_H_ = true;
public const int mjVERSION_HEADER = 334;


// ------------------------------------Enums------------------------------------
public enum mjtWarning : int{
  mjWARN_INERTIA = 0,
  mjWARN_CONTACTFULL = 1,
  mjWARN_CNSTRFULL = 2,
  mjWARN_VGEOMFULL = 3,
  mjWARN_BADQPOS = 4,
  mjWARN_BADQVEL = 5,
  mjWARN_BADQACC = 6,
  mjWARN_BADCTRL = 7,
  mjNWARNING = 8,
}
public enum mjtTimer : int{
  mjTIMER_STEP = 0,
  mjTIMER_FORWARD = 1,
  mjTIMER_INVERSE = 2,
  mjTIMER_POSITION = 3,
  mjTIMER_VELOCITY = 4,
  mjTIMER_ACTUATION = 5,
  mjTIMER_CONSTRAINT = 6,
  mjTIMER_ADVANCE = 7,
  mjTIMER_POS_KINEMATICS = 8,
  mjTIMER_POS_INERTIA = 9,
  mjTIMER_POS_COLLISION = 10,
  mjTIMER_POS_MAKE = 11,
  mjTIMER_POS_PROJECT = 12,
  mjTIMER_COL_BROAD = 13,
  mjTIMER_COL_NARROW = 14,
  mjNTIMER = 15,
}
public enum mjtDisableBit : int{
  mjDSBL_CONSTRAINT = 1,
  mjDSBL_EQUALITY = 2,
  mjDSBL_FRICTIONLOSS = 4,
  mjDSBL_LIMIT = 8,
  mjDSBL_CONTACT = 16,
  mjDSBL_PASSIVE = 32,
  mjDSBL_GRAVITY = 64,
  mjDSBL_CLAMPCTRL = 128,
  mjDSBL_WARMSTART = 256,
  mjDSBL_FILTERPARENT = 512,
  mjDSBL_ACTUATION = 1024,
  mjDSBL_REFSAFE = 2048,
  mjDSBL_SENSOR = 4096,
  mjDSBL_MIDPHASE = 8192,
  mjDSBL_EULERDAMP = 16384,
  mjDSBL_AUTORESET = 32768,
  mjDSBL_NATIVECCD = 65536,
  mjNDISABLE = 17,
}
public enum mjtEnableBit : int{
  mjENBL_OVERRIDE = 1,
  mjENBL_ENERGY = 2,
  mjENBL_FWDINV = 4,
  mjENBL_INVDISCRETE = 8,
  mjENBL_MULTICCD = 16,
  mjENBL_ISLAND = 32,
  mjNENABLE = 6,
}
public enum mjtJoint : int{
  mjJNT_FREE = 0,
  mjJNT_BALL = 1,
  mjJNT_SLIDE = 2,
  mjJNT_HINGE = 3,
}
public enum mjtGeom : int{
  mjGEOM_PLANE = 0,
  mjGEOM_HFIELD = 1,
  mjGEOM_SPHERE = 2,
  mjGEOM_CAPSULE = 3,
  mjGEOM_ELLIPSOID = 4,
  mjGEOM_CYLINDER = 5,
  mjGEOM_BOX = 6,
  mjGEOM_MESH = 7,
  mjGEOM_SDF = 8,
  mjNGEOMTYPES = 9,
  mjGEOM_ARROW = 100,
  mjGEOM_ARROW1 = 101,
  mjGEOM_ARROW2 = 102,
  mjGEOM_LINE = 103,
  mjGEOM_LINEBOX = 104,
  mjGEOM_FLEX = 105,
  mjGEOM_SKIN = 106,
  mjGEOM_LABEL = 107,
  mjGEOM_TRIANGLE = 108,
  mjGEOM_NONE = 1001,
}
public enum mjtCamLight : int{
  mjCAMLIGHT_FIXED = 0,
  mjCAMLIGHT_TRACK = 1,
  mjCAMLIGHT_TRACKCOM = 2,
  mjCAMLIGHT_TARGETBODY = 3,
  mjCAMLIGHT_TARGETBODYCOM = 4,
}
public enum mjtLightType : int{
  mjLIGHT_SPOT = 0,
  mjLIGHT_DIRECTIONAL = 1,
  mjLIGHT_POINT = 2,
  mjLIGHT_IMAGE = 3,
}
public enum mjtTexture : int{
  mjTEXTURE_2D = 0,
  mjTEXTURE_CUBE = 1,
  mjTEXTURE_SKYBOX = 2,
}
public enum mjtTextureRole : int{
  mjTEXROLE_USER = 0,
  mjTEXROLE_RGB = 1,
  mjTEXROLE_OCCLUSION = 2,
  mjTEXROLE_ROUGHNESS = 3,
  mjTEXROLE_METALLIC = 4,
  mjTEXROLE_NORMAL = 5,
  mjTEXROLE_OPACITY = 6,
  mjTEXROLE_EMISSIVE = 7,
  mjTEXROLE_RGBA = 8,
  mjTEXROLE_ORM = 9,
  mjNTEXROLE = 10,
}
public enum mjtColorSpace : int{
  mjCOLORSPACE_AUTO = 0,
  mjCOLORSPACE_LINEAR = 1,
  mjCOLORSPACE_SRGB = 2,
}
public enum mjtIntegrator : int{
  mjINT_EULER = 0,
  mjINT_RK4 = 1,
  mjINT_IMPLICIT = 2,
  mjINT_IMPLICITFAST = 3,
}
public enum mjtCone : int{
  mjCONE_PYRAMIDAL = 0,
  mjCONE_ELLIPTIC = 1,
}
public enum mjtJacobian : int{
  mjJAC_DENSE = 0,
  mjJAC_SPARSE = 1,
  mjJAC_AUTO = 2,
}
public enum mjtSolver : int{
  mjSOL_PGS = 0,
  mjSOL_CG = 1,
  mjSOL_NEWTON = 2,
}
public enum mjtEq : int{
  mjEQ_CONNECT = 0,
  mjEQ_WELD = 1,
  mjEQ_JOINT = 2,
  mjEQ_TENDON = 3,
  mjEQ_FLEX = 4,
  mjEQ_DISTANCE = 5,
}
public enum mjtWrap : int{
  mjWRAP_NONE = 0,
  mjWRAP_JOINT = 1,
  mjWRAP_PULLEY = 2,
  mjWRAP_SITE = 3,
  mjWRAP_SPHERE = 4,
  mjWRAP_CYLINDER = 5,
}
public enum mjtTrn : int{
  mjTRN_JOINT = 0,
  mjTRN_JOINTINPARENT = 1,
  mjTRN_SLIDERCRANK = 2,
  mjTRN_TENDON = 3,
  mjTRN_SITE = 4,
  mjTRN_BODY = 5,
  mjTRN_UNDEFINED = 1000,
}
public enum mjtDyn : int{
  mjDYN_NONE = 0,
  mjDYN_INTEGRATOR = 1,
  mjDYN_FILTER = 2,
  mjDYN_FILTEREXACT = 3,
  mjDYN_MUSCLE = 4,
  mjDYN_USER = 5,
}
public enum mjtGain : int{
  mjGAIN_FIXED = 0,
  mjGAIN_AFFINE = 1,
  mjGAIN_MUSCLE = 2,
  mjGAIN_USER = 3,
}
public enum mjtBias : int{
  mjBIAS_NONE = 0,
  mjBIAS_AFFINE = 1,
  mjBIAS_MUSCLE = 2,
  mjBIAS_USER = 3,
}
public enum mjtObj : int{
  mjOBJ_UNKNOWN = 0,
  mjOBJ_BODY = 1,
  mjOBJ_XBODY = 2,
  mjOBJ_JOINT = 3,
  mjOBJ_DOF = 4,
  mjOBJ_GEOM = 5,
  mjOBJ_SITE = 6,
  mjOBJ_CAMERA = 7,
  mjOBJ_LIGHT = 8,
  mjOBJ_FLEX = 9,
  mjOBJ_MESH = 10,
  mjOBJ_SKIN = 11,
  mjOBJ_HFIELD = 12,
  mjOBJ_TEXTURE = 13,
  mjOBJ_MATERIAL = 14,
  mjOBJ_PAIR = 15,
  mjOBJ_EXCLUDE = 16,
  mjOBJ_EQUALITY = 17,
  mjOBJ_TENDON = 18,
  mjOBJ_ACTUATOR = 19,
  mjOBJ_SENSOR = 20,
  mjOBJ_NUMERIC = 21,
  mjOBJ_TEXT = 22,
  mjOBJ_TUPLE = 23,
  mjOBJ_KEY = 24,
  mjOBJ_PLUGIN = 25,
  mjNOBJECT = 26,
  mjOBJ_FRAME = 100,
  mjOBJ_DEFAULT = 101,
  mjOBJ_MODEL = 102,
}
public enum mjtConstraint : int{
  mjCNSTR_EQUALITY = 0,
  mjCNSTR_FRICTION_DOF = 1,
  mjCNSTR_FRICTION_TENDON = 2,
  mjCNSTR_LIMIT_JOINT = 3,
  mjCNSTR_LIMIT_TENDON = 4,
  mjCNSTR_CONTACT_FRICTIONLESS = 5,
  mjCNSTR_CONTACT_PYRAMIDAL = 6,
  mjCNSTR_CONTACT_ELLIPTIC = 7,
}
public enum mjtConstraintState : int{
  mjCNSTRSTATE_SATISFIED = 0,
  mjCNSTRSTATE_QUADRATIC = 1,
  mjCNSTRSTATE_LINEARNEG = 2,
  mjCNSTRSTATE_LINEARPOS = 3,
  mjCNSTRSTATE_CONE = 4,
}
public enum mjtSensor : int{
  mjSENS_TOUCH = 0,
  mjSENS_ACCELEROMETER = 1,
  mjSENS_VELOCIMETER = 2,
  mjSENS_GYRO = 3,
  mjSENS_FORCE = 4,
  mjSENS_TORQUE = 5,
  mjSENS_MAGNETOMETER = 6,
  mjSENS_RANGEFINDER = 7,
  mjSENS_CAMPROJECTION = 8,
  mjSENS_JOINTPOS = 9,
  mjSENS_JOINTVEL = 10,
  mjSENS_TENDONPOS = 11,
  mjSENS_TENDONVEL = 12,
  mjSENS_ACTUATORPOS = 13,
  mjSENS_ACTUATORVEL = 14,
  mjSENS_ACTUATORFRC = 15,
  mjSENS_JOINTACTFRC = 16,
  mjSENS_TENDONACTFRC = 17,
  mjSENS_BALLQUAT = 18,
  mjSENS_BALLANGVEL = 19,
  mjSENS_JOINTLIMITPOS = 20,
  mjSENS_JOINTLIMITVEL = 21,
  mjSENS_JOINTLIMITFRC = 22,
  mjSENS_TENDONLIMITPOS = 23,
  mjSENS_TENDONLIMITVEL = 24,
  mjSENS_TENDONLIMITFRC = 25,
  mjSENS_FRAMEPOS = 26,
  mjSENS_FRAMEQUAT = 27,
  mjSENS_FRAMEXAXIS = 28,
  mjSENS_FRAMEYAXIS = 29,
  mjSENS_FRAMEZAXIS = 30,
  mjSENS_FRAMELINVEL = 31,
  mjSENS_FRAMEANGVEL = 32,
  mjSENS_FRAMELINACC = 33,
  mjSENS_FRAMEANGACC = 34,
  mjSENS_SUBTREECOM = 35,
  mjSENS_SUBTREELINVEL = 36,
  mjSENS_SUBTREEANGMOM = 37,
  mjSENS_GEOMDIST = 38,
  mjSENS_GEOMNORMAL = 39,
  mjSENS_GEOMFROMTO = 40,
  mjSENS_E_POTENTIAL = 41,
  mjSENS_E_KINETIC = 42,
  mjSENS_CLOCK = 43,
  mjSENS_PLUGIN = 44,
  mjSENS_USER = 45,
}
public enum mjtStage : int{
  mjSTAGE_NONE = 0,
  mjSTAGE_POS = 1,
  mjSTAGE_VEL = 2,
  mjSTAGE_ACC = 3,
}
public enum mjtDataType : int{
  mjDATATYPE_REAL = 0,
  mjDATATYPE_POSITIVE = 1,
  mjDATATYPE_AXIS = 2,
  mjDATATYPE_QUATERNION = 3,
}
public enum mjtSameFrame : int{
  mjSAMEFRAME_NONE = 0,
  mjSAMEFRAME_BODY = 1,
  mjSAMEFRAME_INERTIA = 2,
  mjSAMEFRAME_BODYROT = 3,
  mjSAMEFRAME_INERTIAROT = 4,
}
public enum mjtLRMode : int{
  mjLRMODE_NONE = 0,
  mjLRMODE_MUSCLE = 1,
  mjLRMODE_MUSCLEUSER = 2,
  mjLRMODE_ALL = 3,
}
public enum mjtFlexSelf : int{
  mjFLEXSELF_NONE = 0,
  mjFLEXSELF_NARROW = 1,
  mjFLEXSELF_BVH = 2,
  mjFLEXSELF_SAP = 3,
  mjFLEXSELF_AUTO = 4,
}
public enum mjtSDFType : int{
  mjSDFTYPE_SINGLE = 0,
  mjSDFTYPE_INTERSECTION = 1,
  mjSDFTYPE_MIDSURFACE = 2,
  mjSDFTYPE_COLLISION = 3,
}
public enum mjtPluginCapabilityBit : int{
  mjPLUGIN_ACTUATOR = 1,
  mjPLUGIN_SENSOR = 2,
  mjPLUGIN_PASSIVE = 4,
  mjPLUGIN_SDF = 8,
}
public enum mjtGridPos : int{
  mjGRID_TOPLEFT = 0,
  mjGRID_TOPRIGHT = 1,
  mjGRID_BOTTOMLEFT = 2,
  mjGRID_BOTTOMRIGHT = 3,
  mjGRID_TOP = 4,
  mjGRID_BOTTOM = 5,
  mjGRID_LEFT = 6,
  mjGRID_RIGHT = 7,
}
public enum mjtFramebuffer : int{
  mjFB_WINDOW = 0,
  mjFB_OFFSCREEN = 1,
}
public enum mjtDepthMap : int{
  mjDEPTH_ZERONEAR = 0,
  mjDEPTH_ZEROFAR = 1,
}
public enum mjtFontScale : int{
  mjFONTSCALE_50 = 50,
  mjFONTSCALE_100 = 100,
  mjFONTSCALE_150 = 150,
  mjFONTSCALE_200 = 200,
  mjFONTSCALE_250 = 250,
  mjFONTSCALE_300 = 300,
}
public enum mjtFont : int{
  mjFONT_NORMAL = 0,
  mjFONT_SHADOW = 1,
  mjFONT_BIG = 2,
}
public enum mjtGeomInertia : int{
  mjINERTIA_VOLUME = 0,
  mjINERTIA_SHELL = 1,
}
public enum mjtMeshInertia : int{
  mjMESH_INERTIA_CONVEX = 0,
  mjMESH_INERTIA_EXACT = 1,
  mjMESH_INERTIA_LEGACY = 2,
  mjMESH_INERTIA_SHELL = 3,
}
public enum mjtBuiltin : int{
  mjBUILTIN_NONE = 0,
  mjBUILTIN_GRADIENT = 1,
  mjBUILTIN_CHECKER = 2,
  mjBUILTIN_FLAT = 3,
}
public enum mjtMark : int{
  mjMARK_NONE = 0,
  mjMARK_EDGE = 1,
  mjMARK_CROSS = 2,
  mjMARK_RANDOM = 3,
}
public enum mjtLimited : int{
  mjLIMITED_FALSE = 0,
  mjLIMITED_TRUE = 1,
  mjLIMITED_AUTO = 2,
}
public enum mjtAlignFree : int{
  mjALIGNFREE_FALSE = 0,
  mjALIGNFREE_TRUE = 1,
  mjALIGNFREE_AUTO = 2,
}
public enum mjtInertiaFromGeom : int{
  mjINERTIAFROMGEOM_FALSE = 0,
  mjINERTIAFROMGEOM_TRUE = 1,
  mjINERTIAFROMGEOM_AUTO = 2,
}
public enum mjtOrientation : int{
  mjORIENTATION_QUAT = 0,
  mjORIENTATION_AXISANGLE = 1,
  mjORIENTATION_XYAXES = 2,
  mjORIENTATION_ZAXIS = 3,
  mjORIENTATION_EULER = 4,
}
public enum mjtTaskStatus : int{
  mjTASK_NEW = 0,
  mjTASK_QUEUED = 1,
  mjTASK_COMPLETED = 2,
}
public enum mjtButton : int{
  mjBUTTON_NONE = 0,
  mjBUTTON_LEFT = 1,
  mjBUTTON_RIGHT = 2,
  mjBUTTON_MIDDLE = 3,
}
public enum mjtEvent : int{
  mjEVENT_NONE = 0,
  mjEVENT_MOVE = 1,
  mjEVENT_PRESS = 2,
  mjEVENT_RELEASE = 3,
  mjEVENT_SCROLL = 4,
  mjEVENT_KEY = 5,
  mjEVENT_RESIZE = 6,
  mjEVENT_REDRAW = 7,
  mjEVENT_FILESDROP = 8,
}
public enum mjtSection : int{
  mjSECT_CLOSED = 0,
  mjSECT_OPEN = 1,
  mjSECT_FIXED = 2,
}
public enum mjtCatBit : int{
  mjCAT_STATIC = 1,
  mjCAT_DYNAMIC = 2,
  mjCAT_DECOR = 4,
  mjCAT_ALL = 7,
}
public enum mjtMouse : int{
  mjMOUSE_NONE = 0,
  mjMOUSE_ROTATE_V = 1,
  mjMOUSE_ROTATE_H = 2,
  mjMOUSE_MOVE_V = 3,
  mjMOUSE_MOVE_H = 4,
  mjMOUSE_ZOOM = 5,
  mjMOUSE_SELECT = 6,
}
public enum mjtPertBit : int{
  mjPERT_TRANSLATE = 1,
  mjPERT_ROTATE = 2,
}
public enum mjtCamera : int{
  mjCAMERA_FREE = 0,
  mjCAMERA_TRACKING = 1,
  mjCAMERA_FIXED = 2,
  mjCAMERA_USER = 3,
}
public enum mjtLabel : int{
  mjLABEL_NONE = 0,
  mjLABEL_BODY = 1,
  mjLABEL_JOINT = 2,
  mjLABEL_GEOM = 3,
  mjLABEL_SITE = 4,
  mjLABEL_CAMERA = 5,
  mjLABEL_LIGHT = 6,
  mjLABEL_TENDON = 7,
  mjLABEL_ACTUATOR = 8,
  mjLABEL_CONSTRAINT = 9,
  mjLABEL_FLEX = 10,
  mjLABEL_SKIN = 11,
  mjLABEL_SELECTION = 12,
  mjLABEL_SELPNT = 13,
  mjLABEL_CONTACTPOINT = 14,
  mjLABEL_CONTACTFORCE = 15,
  mjLABEL_ISLAND = 16,
  mjNLABEL = 17,
}
public enum mjtFrame : int{
  mjFRAME_NONE = 0,
  mjFRAME_BODY = 1,
  mjFRAME_GEOM = 2,
  mjFRAME_SITE = 3,
  mjFRAME_CAMERA = 4,
  mjFRAME_LIGHT = 5,
  mjFRAME_CONTACT = 6,
  mjFRAME_WORLD = 7,
  mjNFRAME = 8,
}
public enum mjtVisFlag : int{
  mjVIS_CONVEXHULL = 0,
  mjVIS_TEXTURE = 1,
  mjVIS_JOINT = 2,
  mjVIS_CAMERA = 3,
  mjVIS_ACTUATOR = 4,
  mjVIS_ACTIVATION = 5,
  mjVIS_LIGHT = 6,
  mjVIS_TENDON = 7,
  mjVIS_RANGEFINDER = 8,
  mjVIS_CONSTRAINT = 9,
  mjVIS_INERTIA = 10,
  mjVIS_SCLINERTIA = 11,
  mjVIS_PERTFORCE = 12,
  mjVIS_PERTOBJ = 13,
  mjVIS_CONTACTPOINT = 14,
  mjVIS_ISLAND = 15,
  mjVIS_CONTACTFORCE = 16,
  mjVIS_CONTACTSPLIT = 17,
  mjVIS_TRANSPARENT = 18,
  mjVIS_AUTOCONNECT = 19,
  mjVIS_COM = 20,
  mjVIS_SELECT = 21,
  mjVIS_STATIC = 22,
  mjVIS_SKIN = 23,
  mjVIS_FLEXVERT = 24,
  mjVIS_FLEXEDGE = 25,
  mjVIS_FLEXFACE = 26,
  mjVIS_FLEXSKIN = 27,
  mjVIS_BODYBVH = 28,
  mjVIS_FLEXBVH = 29,
  mjVIS_MESHBVH = 30,
  mjVIS_SDFITER = 31,
  mjNVISFLAG = 32,
}
public enum mjtRndFlag : int{
  mjRND_SHADOW = 0,
  mjRND_WIREFRAME = 1,
  mjRND_REFLECTION = 2,
  mjRND_ADDITIVE = 3,
  mjRND_SKYBOX = 4,
  mjRND_FOG = 5,
  mjRND_HAZE = 6,
  mjRND_SEGMENT = 7,
  mjRND_IDCOLOR = 8,
  mjRND_CULL_FACE = 9,
  mjNRNDFLAG = 10,
}
public enum mjtStereo : int{
  mjSTEREO_NONE = 0,
  mjSTEREO_QUADBUFFERED = 1,
  mjSTEREO_SIDEBYSIDE = 2,
}

// -----------------------------struct declarations-----------------------------


[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjContact_ {
  public double dist;
  public fixed double pos[3];
  public fixed double frame[9];
  public double includemargin;
  public fixed double friction[5];
  public fixed double solref[2];
  public fixed double solreffriction[2];
  public fixed double solimp[5];
  public double mu;
  public fixed double H[36];
  public int dim;
  public int geom1;
  public int geom2;
  public fixed int geom[2];
  public fixed int flex[2];
  public fixed int elem[2];
  public fixed int vert[2];
  public int exclude;
  public int efc_address;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjWarningStat_ {
  public int lastinfo;
  public int number;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjTimerStat_ {
  public double duration;
  public int number;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjSolverStat_ {
  public double improvement;
  public double gradient;
  public double lineslope;
  public int nactive;
  public int nchange;
  public int neval;
  public int nupdate;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjData_ {
  public UIntPtr narena;
  public UIntPtr nbuffer;
  public int nplugin;
  public UIntPtr pstack;
  public UIntPtr pbase;
  public UIntPtr parena;
  public UIntPtr maxuse_stack;
  public UIntPtr maxuse_threadstack0;
  public UIntPtr maxuse_threadstack1;
  public UIntPtr maxuse_threadstack2;
  public UIntPtr maxuse_threadstack3;
  public UIntPtr maxuse_threadstack4;
  public UIntPtr maxuse_threadstack5;
  public UIntPtr maxuse_threadstack6;
  public UIntPtr maxuse_threadstack7;
  public UIntPtr maxuse_threadstack8;
  public UIntPtr maxuse_threadstack9;
  public UIntPtr maxuse_threadstack10;
  public UIntPtr maxuse_threadstack11;
  public UIntPtr maxuse_threadstack12;
  public UIntPtr maxuse_threadstack13;
  public UIntPtr maxuse_threadstack14;
  public UIntPtr maxuse_threadstack15;
  public UIntPtr maxuse_threadstack16;
  public UIntPtr maxuse_threadstack17;
  public UIntPtr maxuse_threadstack18;
  public UIntPtr maxuse_threadstack19;
  public UIntPtr maxuse_threadstack20;
  public UIntPtr maxuse_threadstack21;
  public UIntPtr maxuse_threadstack22;
  public UIntPtr maxuse_threadstack23;
  public UIntPtr maxuse_threadstack24;
  public UIntPtr maxuse_threadstack25;
  public UIntPtr maxuse_threadstack26;
  public UIntPtr maxuse_threadstack27;
  public UIntPtr maxuse_threadstack28;
  public UIntPtr maxuse_threadstack29;
  public UIntPtr maxuse_threadstack30;
  public UIntPtr maxuse_threadstack31;
  public UIntPtr maxuse_threadstack32;
  public UIntPtr maxuse_threadstack33;
  public UIntPtr maxuse_threadstack34;
  public UIntPtr maxuse_threadstack35;
  public UIntPtr maxuse_threadstack36;
  public UIntPtr maxuse_threadstack37;
  public UIntPtr maxuse_threadstack38;
  public UIntPtr maxuse_threadstack39;
  public UIntPtr maxuse_threadstack40;
  public UIntPtr maxuse_threadstack41;
  public UIntPtr maxuse_threadstack42;
  public UIntPtr maxuse_threadstack43;
  public UIntPtr maxuse_threadstack44;
  public UIntPtr maxuse_threadstack45;
  public UIntPtr maxuse_threadstack46;
  public UIntPtr maxuse_threadstack47;
  public UIntPtr maxuse_threadstack48;
  public UIntPtr maxuse_threadstack49;
  public UIntPtr maxuse_threadstack50;
  public UIntPtr maxuse_threadstack51;
  public UIntPtr maxuse_threadstack52;
  public UIntPtr maxuse_threadstack53;
  public UIntPtr maxuse_threadstack54;
  public UIntPtr maxuse_threadstack55;
  public UIntPtr maxuse_threadstack56;
  public UIntPtr maxuse_threadstack57;
  public UIntPtr maxuse_threadstack58;
  public UIntPtr maxuse_threadstack59;
  public UIntPtr maxuse_threadstack60;
  public UIntPtr maxuse_threadstack61;
  public UIntPtr maxuse_threadstack62;
  public UIntPtr maxuse_threadstack63;
  public UIntPtr maxuse_threadstack64;
  public UIntPtr maxuse_threadstack65;
  public UIntPtr maxuse_threadstack66;
  public UIntPtr maxuse_threadstack67;
  public UIntPtr maxuse_threadstack68;
  public UIntPtr maxuse_threadstack69;
  public UIntPtr maxuse_threadstack70;
  public UIntPtr maxuse_threadstack71;
  public UIntPtr maxuse_threadstack72;
  public UIntPtr maxuse_threadstack73;
  public UIntPtr maxuse_threadstack74;
  public UIntPtr maxuse_threadstack75;
  public UIntPtr maxuse_threadstack76;
  public UIntPtr maxuse_threadstack77;
  public UIntPtr maxuse_threadstack78;
  public UIntPtr maxuse_threadstack79;
  public UIntPtr maxuse_threadstack80;
  public UIntPtr maxuse_threadstack81;
  public UIntPtr maxuse_threadstack82;
  public UIntPtr maxuse_threadstack83;
  public UIntPtr maxuse_threadstack84;
  public UIntPtr maxuse_threadstack85;
  public UIntPtr maxuse_threadstack86;
  public UIntPtr maxuse_threadstack87;
  public UIntPtr maxuse_threadstack88;
  public UIntPtr maxuse_threadstack89;
  public UIntPtr maxuse_threadstack90;
  public UIntPtr maxuse_threadstack91;
  public UIntPtr maxuse_threadstack92;
  public UIntPtr maxuse_threadstack93;
  public UIntPtr maxuse_threadstack94;
  public UIntPtr maxuse_threadstack95;
  public UIntPtr maxuse_threadstack96;
  public UIntPtr maxuse_threadstack97;
  public UIntPtr maxuse_threadstack98;
  public UIntPtr maxuse_threadstack99;
  public UIntPtr maxuse_threadstack100;
  public UIntPtr maxuse_threadstack101;
  public UIntPtr maxuse_threadstack102;
  public UIntPtr maxuse_threadstack103;
  public UIntPtr maxuse_threadstack104;
  public UIntPtr maxuse_threadstack105;
  public UIntPtr maxuse_threadstack106;
  public UIntPtr maxuse_threadstack107;
  public UIntPtr maxuse_threadstack108;
  public UIntPtr maxuse_threadstack109;
  public UIntPtr maxuse_threadstack110;
  public UIntPtr maxuse_threadstack111;
  public UIntPtr maxuse_threadstack112;
  public UIntPtr maxuse_threadstack113;
  public UIntPtr maxuse_threadstack114;
  public UIntPtr maxuse_threadstack115;
  public UIntPtr maxuse_threadstack116;
  public UIntPtr maxuse_threadstack117;
  public UIntPtr maxuse_threadstack118;
  public UIntPtr maxuse_threadstack119;
  public UIntPtr maxuse_threadstack120;
  public UIntPtr maxuse_threadstack121;
  public UIntPtr maxuse_threadstack122;
  public UIntPtr maxuse_threadstack123;
  public UIntPtr maxuse_threadstack124;
  public UIntPtr maxuse_threadstack125;
  public UIntPtr maxuse_threadstack126;
  public UIntPtr maxuse_threadstack127;
  public UIntPtr maxuse_arena;
  public int maxuse_con;
  public int maxuse_efc;
  public mjSolverStat_ solver0;
  public mjSolverStat_ solver1;
  public mjSolverStat_ solver2;
  public mjSolverStat_ solver3;
  public mjSolverStat_ solver4;
  public mjSolverStat_ solver5;
  public mjSolverStat_ solver6;
  public mjSolverStat_ solver7;
  public mjSolverStat_ solver8;
  public mjSolverStat_ solver9;
  public mjSolverStat_ solver10;
  public mjSolverStat_ solver11;
  public mjSolverStat_ solver12;
  public mjSolverStat_ solver13;
  public mjSolverStat_ solver14;
  public mjSolverStat_ solver15;
  public mjSolverStat_ solver16;
  public mjSolverStat_ solver17;
  public mjSolverStat_ solver18;
  public mjSolverStat_ solver19;
  public mjSolverStat_ solver20;
  public mjSolverStat_ solver21;
  public mjSolverStat_ solver22;
  public mjSolverStat_ solver23;
  public mjSolverStat_ solver24;
  public mjSolverStat_ solver25;
  public mjSolverStat_ solver26;
  public mjSolverStat_ solver27;
  public mjSolverStat_ solver28;
  public mjSolverStat_ solver29;
  public mjSolverStat_ solver30;
  public mjSolverStat_ solver31;
  public mjSolverStat_ solver32;
  public mjSolverStat_ solver33;
  public mjSolverStat_ solver34;
  public mjSolverStat_ solver35;
  public mjSolverStat_ solver36;
  public mjSolverStat_ solver37;
  public mjSolverStat_ solver38;
  public mjSolverStat_ solver39;
  public mjSolverStat_ solver40;
  public mjSolverStat_ solver41;
  public mjSolverStat_ solver42;
  public mjSolverStat_ solver43;
  public mjSolverStat_ solver44;
  public mjSolverStat_ solver45;
  public mjSolverStat_ solver46;
  public mjSolverStat_ solver47;
  public mjSolverStat_ solver48;
  public mjSolverStat_ solver49;
  public mjSolverStat_ solver50;
  public mjSolverStat_ solver51;
  public mjSolverStat_ solver52;
  public mjSolverStat_ solver53;
  public mjSolverStat_ solver54;
  public mjSolverStat_ solver55;
  public mjSolverStat_ solver56;
  public mjSolverStat_ solver57;
  public mjSolverStat_ solver58;
  public mjSolverStat_ solver59;
  public mjSolverStat_ solver60;
  public mjSolverStat_ solver61;
  public mjSolverStat_ solver62;
  public mjSolverStat_ solver63;
  public mjSolverStat_ solver64;
  public mjSolverStat_ solver65;
  public mjSolverStat_ solver66;
  public mjSolverStat_ solver67;
  public mjSolverStat_ solver68;
  public mjSolverStat_ solver69;
  public mjSolverStat_ solver70;
  public mjSolverStat_ solver71;
  public mjSolverStat_ solver72;
  public mjSolverStat_ solver73;
  public mjSolverStat_ solver74;
  public mjSolverStat_ solver75;
  public mjSolverStat_ solver76;
  public mjSolverStat_ solver77;
  public mjSolverStat_ solver78;
  public mjSolverStat_ solver79;
  public mjSolverStat_ solver80;
  public mjSolverStat_ solver81;
  public mjSolverStat_ solver82;
  public mjSolverStat_ solver83;
  public mjSolverStat_ solver84;
  public mjSolverStat_ solver85;
  public mjSolverStat_ solver86;
  public mjSolverStat_ solver87;
  public mjSolverStat_ solver88;
  public mjSolverStat_ solver89;
  public mjSolverStat_ solver90;
  public mjSolverStat_ solver91;
  public mjSolverStat_ solver92;
  public mjSolverStat_ solver93;
  public mjSolverStat_ solver94;
  public mjSolverStat_ solver95;
  public mjSolverStat_ solver96;
  public mjSolverStat_ solver97;
  public mjSolverStat_ solver98;
  public mjSolverStat_ solver99;
  public mjSolverStat_ solver100;
  public mjSolverStat_ solver101;
  public mjSolverStat_ solver102;
  public mjSolverStat_ solver103;
  public mjSolverStat_ solver104;
  public mjSolverStat_ solver105;
  public mjSolverStat_ solver106;
  public mjSolverStat_ solver107;
  public mjSolverStat_ solver108;
  public mjSolverStat_ solver109;
  public mjSolverStat_ solver110;
  public mjSolverStat_ solver111;
  public mjSolverStat_ solver112;
  public mjSolverStat_ solver113;
  public mjSolverStat_ solver114;
  public mjSolverStat_ solver115;
  public mjSolverStat_ solver116;
  public mjSolverStat_ solver117;
  public mjSolverStat_ solver118;
  public mjSolverStat_ solver119;
  public mjSolverStat_ solver120;
  public mjSolverStat_ solver121;
  public mjSolverStat_ solver122;
  public mjSolverStat_ solver123;
  public mjSolverStat_ solver124;
  public mjSolverStat_ solver125;
  public mjSolverStat_ solver126;
  public mjSolverStat_ solver127;
  public mjSolverStat_ solver128;
  public mjSolverStat_ solver129;
  public mjSolverStat_ solver130;
  public mjSolverStat_ solver131;
  public mjSolverStat_ solver132;
  public mjSolverStat_ solver133;
  public mjSolverStat_ solver134;
  public mjSolverStat_ solver135;
  public mjSolverStat_ solver136;
  public mjSolverStat_ solver137;
  public mjSolverStat_ solver138;
  public mjSolverStat_ solver139;
  public mjSolverStat_ solver140;
  public mjSolverStat_ solver141;
  public mjSolverStat_ solver142;
  public mjSolverStat_ solver143;
  public mjSolverStat_ solver144;
  public mjSolverStat_ solver145;
  public mjSolverStat_ solver146;
  public mjSolverStat_ solver147;
  public mjSolverStat_ solver148;
  public mjSolverStat_ solver149;
  public mjSolverStat_ solver150;
  public mjSolverStat_ solver151;
  public mjSolverStat_ solver152;
  public mjSolverStat_ solver153;
  public mjSolverStat_ solver154;
  public mjSolverStat_ solver155;
  public mjSolverStat_ solver156;
  public mjSolverStat_ solver157;
  public mjSolverStat_ solver158;
  public mjSolverStat_ solver159;
  public mjSolverStat_ solver160;
  public mjSolverStat_ solver161;
  public mjSolverStat_ solver162;
  public mjSolverStat_ solver163;
  public mjSolverStat_ solver164;
  public mjSolverStat_ solver165;
  public mjSolverStat_ solver166;
  public mjSolverStat_ solver167;
  public mjSolverStat_ solver168;
  public mjSolverStat_ solver169;
  public mjSolverStat_ solver170;
  public mjSolverStat_ solver171;
  public mjSolverStat_ solver172;
  public mjSolverStat_ solver173;
  public mjSolverStat_ solver174;
  public mjSolverStat_ solver175;
  public mjSolverStat_ solver176;
  public mjSolverStat_ solver177;
  public mjSolverStat_ solver178;
  public mjSolverStat_ solver179;
  public mjSolverStat_ solver180;
  public mjSolverStat_ solver181;
  public mjSolverStat_ solver182;
  public mjSolverStat_ solver183;
  public mjSolverStat_ solver184;
  public mjSolverStat_ solver185;
  public mjSolverStat_ solver186;
  public mjSolverStat_ solver187;
  public mjSolverStat_ solver188;
  public mjSolverStat_ solver189;
  public mjSolverStat_ solver190;
  public mjSolverStat_ solver191;
  public mjSolverStat_ solver192;
  public mjSolverStat_ solver193;
  public mjSolverStat_ solver194;
  public mjSolverStat_ solver195;
  public mjSolverStat_ solver196;
  public mjSolverStat_ solver197;
  public mjSolverStat_ solver198;
  public mjSolverStat_ solver199;
  public mjSolverStat_ solver200;
  public mjSolverStat_ solver201;
  public mjSolverStat_ solver202;
  public mjSolverStat_ solver203;
  public mjSolverStat_ solver204;
  public mjSolverStat_ solver205;
  public mjSolverStat_ solver206;
  public mjSolverStat_ solver207;
  public mjSolverStat_ solver208;
  public mjSolverStat_ solver209;
  public mjSolverStat_ solver210;
  public mjSolverStat_ solver211;
  public mjSolverStat_ solver212;
  public mjSolverStat_ solver213;
  public mjSolverStat_ solver214;
  public mjSolverStat_ solver215;
  public mjSolverStat_ solver216;
  public mjSolverStat_ solver217;
  public mjSolverStat_ solver218;
  public mjSolverStat_ solver219;
  public mjSolverStat_ solver220;
  public mjSolverStat_ solver221;
  public mjSolverStat_ solver222;
  public mjSolverStat_ solver223;
  public mjSolverStat_ solver224;
  public mjSolverStat_ solver225;
  public mjSolverStat_ solver226;
  public mjSolverStat_ solver227;
  public mjSolverStat_ solver228;
  public mjSolverStat_ solver229;
  public mjSolverStat_ solver230;
  public mjSolverStat_ solver231;
  public mjSolverStat_ solver232;
  public mjSolverStat_ solver233;
  public mjSolverStat_ solver234;
  public mjSolverStat_ solver235;
  public mjSolverStat_ solver236;
  public mjSolverStat_ solver237;
  public mjSolverStat_ solver238;
  public mjSolverStat_ solver239;
  public mjSolverStat_ solver240;
  public mjSolverStat_ solver241;
  public mjSolverStat_ solver242;
  public mjSolverStat_ solver243;
  public mjSolverStat_ solver244;
  public mjSolverStat_ solver245;
  public mjSolverStat_ solver246;
  public mjSolverStat_ solver247;
  public mjSolverStat_ solver248;
  public mjSolverStat_ solver249;
  public mjSolverStat_ solver250;
  public mjSolverStat_ solver251;
  public mjSolverStat_ solver252;
  public mjSolverStat_ solver253;
  public mjSolverStat_ solver254;
  public mjSolverStat_ solver255;
  public mjSolverStat_ solver256;
  public mjSolverStat_ solver257;
  public mjSolverStat_ solver258;
  public mjSolverStat_ solver259;
  public mjSolverStat_ solver260;
  public mjSolverStat_ solver261;
  public mjSolverStat_ solver262;
  public mjSolverStat_ solver263;
  public mjSolverStat_ solver264;
  public mjSolverStat_ solver265;
  public mjSolverStat_ solver266;
  public mjSolverStat_ solver267;
  public mjSolverStat_ solver268;
  public mjSolverStat_ solver269;
  public mjSolverStat_ solver270;
  public mjSolverStat_ solver271;
  public mjSolverStat_ solver272;
  public mjSolverStat_ solver273;
  public mjSolverStat_ solver274;
  public mjSolverStat_ solver275;
  public mjSolverStat_ solver276;
  public mjSolverStat_ solver277;
  public mjSolverStat_ solver278;
  public mjSolverStat_ solver279;
  public mjSolverStat_ solver280;
  public mjSolverStat_ solver281;
  public mjSolverStat_ solver282;
  public mjSolverStat_ solver283;
  public mjSolverStat_ solver284;
  public mjSolverStat_ solver285;
  public mjSolverStat_ solver286;
  public mjSolverStat_ solver287;
  public mjSolverStat_ solver288;
  public mjSolverStat_ solver289;
  public mjSolverStat_ solver290;
  public mjSolverStat_ solver291;
  public mjSolverStat_ solver292;
  public mjSolverStat_ solver293;
  public mjSolverStat_ solver294;
  public mjSolverStat_ solver295;
  public mjSolverStat_ solver296;
  public mjSolverStat_ solver297;
  public mjSolverStat_ solver298;
  public mjSolverStat_ solver299;
  public mjSolverStat_ solver300;
  public mjSolverStat_ solver301;
  public mjSolverStat_ solver302;
  public mjSolverStat_ solver303;
  public mjSolverStat_ solver304;
  public mjSolverStat_ solver305;
  public mjSolverStat_ solver306;
  public mjSolverStat_ solver307;
  public mjSolverStat_ solver308;
  public mjSolverStat_ solver309;
  public mjSolverStat_ solver310;
  public mjSolverStat_ solver311;
  public mjSolverStat_ solver312;
  public mjSolverStat_ solver313;
  public mjSolverStat_ solver314;
  public mjSolverStat_ solver315;
  public mjSolverStat_ solver316;
  public mjSolverStat_ solver317;
  public mjSolverStat_ solver318;
  public mjSolverStat_ solver319;
  public mjSolverStat_ solver320;
  public mjSolverStat_ solver321;
  public mjSolverStat_ solver322;
  public mjSolverStat_ solver323;
  public mjSolverStat_ solver324;
  public mjSolverStat_ solver325;
  public mjSolverStat_ solver326;
  public mjSolverStat_ solver327;
  public mjSolverStat_ solver328;
  public mjSolverStat_ solver329;
  public mjSolverStat_ solver330;
  public mjSolverStat_ solver331;
  public mjSolverStat_ solver332;
  public mjSolverStat_ solver333;
  public mjSolverStat_ solver334;
  public mjSolverStat_ solver335;
  public mjSolverStat_ solver336;
  public mjSolverStat_ solver337;
  public mjSolverStat_ solver338;
  public mjSolverStat_ solver339;
  public mjSolverStat_ solver340;
  public mjSolverStat_ solver341;
  public mjSolverStat_ solver342;
  public mjSolverStat_ solver343;
  public mjSolverStat_ solver344;
  public mjSolverStat_ solver345;
  public mjSolverStat_ solver346;
  public mjSolverStat_ solver347;
  public mjSolverStat_ solver348;
  public mjSolverStat_ solver349;
  public mjSolverStat_ solver350;
  public mjSolverStat_ solver351;
  public mjSolverStat_ solver352;
  public mjSolverStat_ solver353;
  public mjSolverStat_ solver354;
  public mjSolverStat_ solver355;
  public mjSolverStat_ solver356;
  public mjSolverStat_ solver357;
  public mjSolverStat_ solver358;
  public mjSolverStat_ solver359;
  public mjSolverStat_ solver360;
  public mjSolverStat_ solver361;
  public mjSolverStat_ solver362;
  public mjSolverStat_ solver363;
  public mjSolverStat_ solver364;
  public mjSolverStat_ solver365;
  public mjSolverStat_ solver366;
  public mjSolverStat_ solver367;
  public mjSolverStat_ solver368;
  public mjSolverStat_ solver369;
  public mjSolverStat_ solver370;
  public mjSolverStat_ solver371;
  public mjSolverStat_ solver372;
  public mjSolverStat_ solver373;
  public mjSolverStat_ solver374;
  public mjSolverStat_ solver375;
  public mjSolverStat_ solver376;
  public mjSolverStat_ solver377;
  public mjSolverStat_ solver378;
  public mjSolverStat_ solver379;
  public mjSolverStat_ solver380;
  public mjSolverStat_ solver381;
  public mjSolverStat_ solver382;
  public mjSolverStat_ solver383;
  public mjSolverStat_ solver384;
  public mjSolverStat_ solver385;
  public mjSolverStat_ solver386;
  public mjSolverStat_ solver387;
  public mjSolverStat_ solver388;
  public mjSolverStat_ solver389;
  public mjSolverStat_ solver390;
  public mjSolverStat_ solver391;
  public mjSolverStat_ solver392;
  public mjSolverStat_ solver393;
  public mjSolverStat_ solver394;
  public mjSolverStat_ solver395;
  public mjSolverStat_ solver396;
  public mjSolverStat_ solver397;
  public mjSolverStat_ solver398;
  public mjSolverStat_ solver399;
  public mjSolverStat_ solver400;
  public mjSolverStat_ solver401;
  public mjSolverStat_ solver402;
  public mjSolverStat_ solver403;
  public mjSolverStat_ solver404;
  public mjSolverStat_ solver405;
  public mjSolverStat_ solver406;
  public mjSolverStat_ solver407;
  public mjSolverStat_ solver408;
  public mjSolverStat_ solver409;
  public mjSolverStat_ solver410;
  public mjSolverStat_ solver411;
  public mjSolverStat_ solver412;
  public mjSolverStat_ solver413;
  public mjSolverStat_ solver414;
  public mjSolverStat_ solver415;
  public mjSolverStat_ solver416;
  public mjSolverStat_ solver417;
  public mjSolverStat_ solver418;
  public mjSolverStat_ solver419;
  public mjSolverStat_ solver420;
  public mjSolverStat_ solver421;
  public mjSolverStat_ solver422;
  public mjSolverStat_ solver423;
  public mjSolverStat_ solver424;
  public mjSolverStat_ solver425;
  public mjSolverStat_ solver426;
  public mjSolverStat_ solver427;
  public mjSolverStat_ solver428;
  public mjSolverStat_ solver429;
  public mjSolverStat_ solver430;
  public mjSolverStat_ solver431;
  public mjSolverStat_ solver432;
  public mjSolverStat_ solver433;
  public mjSolverStat_ solver434;
  public mjSolverStat_ solver435;
  public mjSolverStat_ solver436;
  public mjSolverStat_ solver437;
  public mjSolverStat_ solver438;
  public mjSolverStat_ solver439;
  public mjSolverStat_ solver440;
  public mjSolverStat_ solver441;
  public mjSolverStat_ solver442;
  public mjSolverStat_ solver443;
  public mjSolverStat_ solver444;
  public mjSolverStat_ solver445;
  public mjSolverStat_ solver446;
  public mjSolverStat_ solver447;
  public mjSolverStat_ solver448;
  public mjSolverStat_ solver449;
  public mjSolverStat_ solver450;
  public mjSolverStat_ solver451;
  public mjSolverStat_ solver452;
  public mjSolverStat_ solver453;
  public mjSolverStat_ solver454;
  public mjSolverStat_ solver455;
  public mjSolverStat_ solver456;
  public mjSolverStat_ solver457;
  public mjSolverStat_ solver458;
  public mjSolverStat_ solver459;
  public mjSolverStat_ solver460;
  public mjSolverStat_ solver461;
  public mjSolverStat_ solver462;
  public mjSolverStat_ solver463;
  public mjSolverStat_ solver464;
  public mjSolverStat_ solver465;
  public mjSolverStat_ solver466;
  public mjSolverStat_ solver467;
  public mjSolverStat_ solver468;
  public mjSolverStat_ solver469;
  public mjSolverStat_ solver470;
  public mjSolverStat_ solver471;
  public mjSolverStat_ solver472;
  public mjSolverStat_ solver473;
  public mjSolverStat_ solver474;
  public mjSolverStat_ solver475;
  public mjSolverStat_ solver476;
  public mjSolverStat_ solver477;
  public mjSolverStat_ solver478;
  public mjSolverStat_ solver479;
  public mjSolverStat_ solver480;
  public mjSolverStat_ solver481;
  public mjSolverStat_ solver482;
  public mjSolverStat_ solver483;
  public mjSolverStat_ solver484;
  public mjSolverStat_ solver485;
  public mjSolverStat_ solver486;
  public mjSolverStat_ solver487;
  public mjSolverStat_ solver488;
  public mjSolverStat_ solver489;
  public mjSolverStat_ solver490;
  public mjSolverStat_ solver491;
  public mjSolverStat_ solver492;
  public mjSolverStat_ solver493;
  public mjSolverStat_ solver494;
  public mjSolverStat_ solver495;
  public mjSolverStat_ solver496;
  public mjSolverStat_ solver497;
  public mjSolverStat_ solver498;
  public mjSolverStat_ solver499;
  public mjSolverStat_ solver500;
  public mjSolverStat_ solver501;
  public mjSolverStat_ solver502;
  public mjSolverStat_ solver503;
  public mjSolverStat_ solver504;
  public mjSolverStat_ solver505;
  public mjSolverStat_ solver506;
  public mjSolverStat_ solver507;
  public mjSolverStat_ solver508;
  public mjSolverStat_ solver509;
  public mjSolverStat_ solver510;
  public mjSolverStat_ solver511;
  public mjSolverStat_ solver512;
  public mjSolverStat_ solver513;
  public mjSolverStat_ solver514;
  public mjSolverStat_ solver515;
  public mjSolverStat_ solver516;
  public mjSolverStat_ solver517;
  public mjSolverStat_ solver518;
  public mjSolverStat_ solver519;
  public mjSolverStat_ solver520;
  public mjSolverStat_ solver521;
  public mjSolverStat_ solver522;
  public mjSolverStat_ solver523;
  public mjSolverStat_ solver524;
  public mjSolverStat_ solver525;
  public mjSolverStat_ solver526;
  public mjSolverStat_ solver527;
  public mjSolverStat_ solver528;
  public mjSolverStat_ solver529;
  public mjSolverStat_ solver530;
  public mjSolverStat_ solver531;
  public mjSolverStat_ solver532;
  public mjSolverStat_ solver533;
  public mjSolverStat_ solver534;
  public mjSolverStat_ solver535;
  public mjSolverStat_ solver536;
  public mjSolverStat_ solver537;
  public mjSolverStat_ solver538;
  public mjSolverStat_ solver539;
  public mjSolverStat_ solver540;
  public mjSolverStat_ solver541;
  public mjSolverStat_ solver542;
  public mjSolverStat_ solver543;
  public mjSolverStat_ solver544;
  public mjSolverStat_ solver545;
  public mjSolverStat_ solver546;
  public mjSolverStat_ solver547;
  public mjSolverStat_ solver548;
  public mjSolverStat_ solver549;
  public mjSolverStat_ solver550;
  public mjSolverStat_ solver551;
  public mjSolverStat_ solver552;
  public mjSolverStat_ solver553;
  public mjSolverStat_ solver554;
  public mjSolverStat_ solver555;
  public mjSolverStat_ solver556;
  public mjSolverStat_ solver557;
  public mjSolverStat_ solver558;
  public mjSolverStat_ solver559;
  public mjSolverStat_ solver560;
  public mjSolverStat_ solver561;
  public mjSolverStat_ solver562;
  public mjSolverStat_ solver563;
  public mjSolverStat_ solver564;
  public mjSolverStat_ solver565;
  public mjSolverStat_ solver566;
  public mjSolverStat_ solver567;
  public mjSolverStat_ solver568;
  public mjSolverStat_ solver569;
  public mjSolverStat_ solver570;
  public mjSolverStat_ solver571;
  public mjSolverStat_ solver572;
  public mjSolverStat_ solver573;
  public mjSolverStat_ solver574;
  public mjSolverStat_ solver575;
  public mjSolverStat_ solver576;
  public mjSolverStat_ solver577;
  public mjSolverStat_ solver578;
  public mjSolverStat_ solver579;
  public mjSolverStat_ solver580;
  public mjSolverStat_ solver581;
  public mjSolverStat_ solver582;
  public mjSolverStat_ solver583;
  public mjSolverStat_ solver584;
  public mjSolverStat_ solver585;
  public mjSolverStat_ solver586;
  public mjSolverStat_ solver587;
  public mjSolverStat_ solver588;
  public mjSolverStat_ solver589;
  public mjSolverStat_ solver590;
  public mjSolverStat_ solver591;
  public mjSolverStat_ solver592;
  public mjSolverStat_ solver593;
  public mjSolverStat_ solver594;
  public mjSolverStat_ solver595;
  public mjSolverStat_ solver596;
  public mjSolverStat_ solver597;
  public mjSolverStat_ solver598;
  public mjSolverStat_ solver599;
  public mjSolverStat_ solver600;
  public mjSolverStat_ solver601;
  public mjSolverStat_ solver602;
  public mjSolverStat_ solver603;
  public mjSolverStat_ solver604;
  public mjSolverStat_ solver605;
  public mjSolverStat_ solver606;
  public mjSolverStat_ solver607;
  public mjSolverStat_ solver608;
  public mjSolverStat_ solver609;
  public mjSolverStat_ solver610;
  public mjSolverStat_ solver611;
  public mjSolverStat_ solver612;
  public mjSolverStat_ solver613;
  public mjSolverStat_ solver614;
  public mjSolverStat_ solver615;
  public mjSolverStat_ solver616;
  public mjSolverStat_ solver617;
  public mjSolverStat_ solver618;
  public mjSolverStat_ solver619;
  public mjSolverStat_ solver620;
  public mjSolverStat_ solver621;
  public mjSolverStat_ solver622;
  public mjSolverStat_ solver623;
  public mjSolverStat_ solver624;
  public mjSolverStat_ solver625;
  public mjSolverStat_ solver626;
  public mjSolverStat_ solver627;
  public mjSolverStat_ solver628;
  public mjSolverStat_ solver629;
  public mjSolverStat_ solver630;
  public mjSolverStat_ solver631;
  public mjSolverStat_ solver632;
  public mjSolverStat_ solver633;
  public mjSolverStat_ solver634;
  public mjSolverStat_ solver635;
  public mjSolverStat_ solver636;
  public mjSolverStat_ solver637;
  public mjSolverStat_ solver638;
  public mjSolverStat_ solver639;
  public mjSolverStat_ solver640;
  public mjSolverStat_ solver641;
  public mjSolverStat_ solver642;
  public mjSolverStat_ solver643;
  public mjSolverStat_ solver644;
  public mjSolverStat_ solver645;
  public mjSolverStat_ solver646;
  public mjSolverStat_ solver647;
  public mjSolverStat_ solver648;
  public mjSolverStat_ solver649;
  public mjSolverStat_ solver650;
  public mjSolverStat_ solver651;
  public mjSolverStat_ solver652;
  public mjSolverStat_ solver653;
  public mjSolverStat_ solver654;
  public mjSolverStat_ solver655;
  public mjSolverStat_ solver656;
  public mjSolverStat_ solver657;
  public mjSolverStat_ solver658;
  public mjSolverStat_ solver659;
  public mjSolverStat_ solver660;
  public mjSolverStat_ solver661;
  public mjSolverStat_ solver662;
  public mjSolverStat_ solver663;
  public mjSolverStat_ solver664;
  public mjSolverStat_ solver665;
  public mjSolverStat_ solver666;
  public mjSolverStat_ solver667;
  public mjSolverStat_ solver668;
  public mjSolverStat_ solver669;
  public mjSolverStat_ solver670;
  public mjSolverStat_ solver671;
  public mjSolverStat_ solver672;
  public mjSolverStat_ solver673;
  public mjSolverStat_ solver674;
  public mjSolverStat_ solver675;
  public mjSolverStat_ solver676;
  public mjSolverStat_ solver677;
  public mjSolverStat_ solver678;
  public mjSolverStat_ solver679;
  public mjSolverStat_ solver680;
  public mjSolverStat_ solver681;
  public mjSolverStat_ solver682;
  public mjSolverStat_ solver683;
  public mjSolverStat_ solver684;
  public mjSolverStat_ solver685;
  public mjSolverStat_ solver686;
  public mjSolverStat_ solver687;
  public mjSolverStat_ solver688;
  public mjSolverStat_ solver689;
  public mjSolverStat_ solver690;
  public mjSolverStat_ solver691;
  public mjSolverStat_ solver692;
  public mjSolverStat_ solver693;
  public mjSolverStat_ solver694;
  public mjSolverStat_ solver695;
  public mjSolverStat_ solver696;
  public mjSolverStat_ solver697;
  public mjSolverStat_ solver698;
  public mjSolverStat_ solver699;
  public mjSolverStat_ solver700;
  public mjSolverStat_ solver701;
  public mjSolverStat_ solver702;
  public mjSolverStat_ solver703;
  public mjSolverStat_ solver704;
  public mjSolverStat_ solver705;
  public mjSolverStat_ solver706;
  public mjSolverStat_ solver707;
  public mjSolverStat_ solver708;
  public mjSolverStat_ solver709;
  public mjSolverStat_ solver710;
  public mjSolverStat_ solver711;
  public mjSolverStat_ solver712;
  public mjSolverStat_ solver713;
  public mjSolverStat_ solver714;
  public mjSolverStat_ solver715;
  public mjSolverStat_ solver716;
  public mjSolverStat_ solver717;
  public mjSolverStat_ solver718;
  public mjSolverStat_ solver719;
  public mjSolverStat_ solver720;
  public mjSolverStat_ solver721;
  public mjSolverStat_ solver722;
  public mjSolverStat_ solver723;
  public mjSolverStat_ solver724;
  public mjSolverStat_ solver725;
  public mjSolverStat_ solver726;
  public mjSolverStat_ solver727;
  public mjSolverStat_ solver728;
  public mjSolverStat_ solver729;
  public mjSolverStat_ solver730;
  public mjSolverStat_ solver731;
  public mjSolverStat_ solver732;
  public mjSolverStat_ solver733;
  public mjSolverStat_ solver734;
  public mjSolverStat_ solver735;
  public mjSolverStat_ solver736;
  public mjSolverStat_ solver737;
  public mjSolverStat_ solver738;
  public mjSolverStat_ solver739;
  public mjSolverStat_ solver740;
  public mjSolverStat_ solver741;
  public mjSolverStat_ solver742;
  public mjSolverStat_ solver743;
  public mjSolverStat_ solver744;
  public mjSolverStat_ solver745;
  public mjSolverStat_ solver746;
  public mjSolverStat_ solver747;
  public mjSolverStat_ solver748;
  public mjSolverStat_ solver749;
  public mjSolverStat_ solver750;
  public mjSolverStat_ solver751;
  public mjSolverStat_ solver752;
  public mjSolverStat_ solver753;
  public mjSolverStat_ solver754;
  public mjSolverStat_ solver755;
  public mjSolverStat_ solver756;
  public mjSolverStat_ solver757;
  public mjSolverStat_ solver758;
  public mjSolverStat_ solver759;
  public mjSolverStat_ solver760;
  public mjSolverStat_ solver761;
  public mjSolverStat_ solver762;
  public mjSolverStat_ solver763;
  public mjSolverStat_ solver764;
  public mjSolverStat_ solver765;
  public mjSolverStat_ solver766;
  public mjSolverStat_ solver767;
  public mjSolverStat_ solver768;
  public mjSolverStat_ solver769;
  public mjSolverStat_ solver770;
  public mjSolverStat_ solver771;
  public mjSolverStat_ solver772;
  public mjSolverStat_ solver773;
  public mjSolverStat_ solver774;
  public mjSolverStat_ solver775;
  public mjSolverStat_ solver776;
  public mjSolverStat_ solver777;
  public mjSolverStat_ solver778;
  public mjSolverStat_ solver779;
  public mjSolverStat_ solver780;
  public mjSolverStat_ solver781;
  public mjSolverStat_ solver782;
  public mjSolverStat_ solver783;
  public mjSolverStat_ solver784;
  public mjSolverStat_ solver785;
  public mjSolverStat_ solver786;
  public mjSolverStat_ solver787;
  public mjSolverStat_ solver788;
  public mjSolverStat_ solver789;
  public mjSolverStat_ solver790;
  public mjSolverStat_ solver791;
  public mjSolverStat_ solver792;
  public mjSolverStat_ solver793;
  public mjSolverStat_ solver794;
  public mjSolverStat_ solver795;
  public mjSolverStat_ solver796;
  public mjSolverStat_ solver797;
  public mjSolverStat_ solver798;
  public mjSolverStat_ solver799;
  public mjSolverStat_ solver800;
  public mjSolverStat_ solver801;
  public mjSolverStat_ solver802;
  public mjSolverStat_ solver803;
  public mjSolverStat_ solver804;
  public mjSolverStat_ solver805;
  public mjSolverStat_ solver806;
  public mjSolverStat_ solver807;
  public mjSolverStat_ solver808;
  public mjSolverStat_ solver809;
  public mjSolverStat_ solver810;
  public mjSolverStat_ solver811;
  public mjSolverStat_ solver812;
  public mjSolverStat_ solver813;
  public mjSolverStat_ solver814;
  public mjSolverStat_ solver815;
  public mjSolverStat_ solver816;
  public mjSolverStat_ solver817;
  public mjSolverStat_ solver818;
  public mjSolverStat_ solver819;
  public mjSolverStat_ solver820;
  public mjSolverStat_ solver821;
  public mjSolverStat_ solver822;
  public mjSolverStat_ solver823;
  public mjSolverStat_ solver824;
  public mjSolverStat_ solver825;
  public mjSolverStat_ solver826;
  public mjSolverStat_ solver827;
  public mjSolverStat_ solver828;
  public mjSolverStat_ solver829;
  public mjSolverStat_ solver830;
  public mjSolverStat_ solver831;
  public mjSolverStat_ solver832;
  public mjSolverStat_ solver833;
  public mjSolverStat_ solver834;
  public mjSolverStat_ solver835;
  public mjSolverStat_ solver836;
  public mjSolverStat_ solver837;
  public mjSolverStat_ solver838;
  public mjSolverStat_ solver839;
  public mjSolverStat_ solver840;
  public mjSolverStat_ solver841;
  public mjSolverStat_ solver842;
  public mjSolverStat_ solver843;
  public mjSolverStat_ solver844;
  public mjSolverStat_ solver845;
  public mjSolverStat_ solver846;
  public mjSolverStat_ solver847;
  public mjSolverStat_ solver848;
  public mjSolverStat_ solver849;
  public mjSolverStat_ solver850;
  public mjSolverStat_ solver851;
  public mjSolverStat_ solver852;
  public mjSolverStat_ solver853;
  public mjSolverStat_ solver854;
  public mjSolverStat_ solver855;
  public mjSolverStat_ solver856;
  public mjSolverStat_ solver857;
  public mjSolverStat_ solver858;
  public mjSolverStat_ solver859;
  public mjSolverStat_ solver860;
  public mjSolverStat_ solver861;
  public mjSolverStat_ solver862;
  public mjSolverStat_ solver863;
  public mjSolverStat_ solver864;
  public mjSolverStat_ solver865;
  public mjSolverStat_ solver866;
  public mjSolverStat_ solver867;
  public mjSolverStat_ solver868;
  public mjSolverStat_ solver869;
  public mjSolverStat_ solver870;
  public mjSolverStat_ solver871;
  public mjSolverStat_ solver872;
  public mjSolverStat_ solver873;
  public mjSolverStat_ solver874;
  public mjSolverStat_ solver875;
  public mjSolverStat_ solver876;
  public mjSolverStat_ solver877;
  public mjSolverStat_ solver878;
  public mjSolverStat_ solver879;
  public mjSolverStat_ solver880;
  public mjSolverStat_ solver881;
  public mjSolverStat_ solver882;
  public mjSolverStat_ solver883;
  public mjSolverStat_ solver884;
  public mjSolverStat_ solver885;
  public mjSolverStat_ solver886;
  public mjSolverStat_ solver887;
  public mjSolverStat_ solver888;
  public mjSolverStat_ solver889;
  public mjSolverStat_ solver890;
  public mjSolverStat_ solver891;
  public mjSolverStat_ solver892;
  public mjSolverStat_ solver893;
  public mjSolverStat_ solver894;
  public mjSolverStat_ solver895;
  public mjSolverStat_ solver896;
  public mjSolverStat_ solver897;
  public mjSolverStat_ solver898;
  public mjSolverStat_ solver899;
  public mjSolverStat_ solver900;
  public mjSolverStat_ solver901;
  public mjSolverStat_ solver902;
  public mjSolverStat_ solver903;
  public mjSolverStat_ solver904;
  public mjSolverStat_ solver905;
  public mjSolverStat_ solver906;
  public mjSolverStat_ solver907;
  public mjSolverStat_ solver908;
  public mjSolverStat_ solver909;
  public mjSolverStat_ solver910;
  public mjSolverStat_ solver911;
  public mjSolverStat_ solver912;
  public mjSolverStat_ solver913;
  public mjSolverStat_ solver914;
  public mjSolverStat_ solver915;
  public mjSolverStat_ solver916;
  public mjSolverStat_ solver917;
  public mjSolverStat_ solver918;
  public mjSolverStat_ solver919;
  public mjSolverStat_ solver920;
  public mjSolverStat_ solver921;
  public mjSolverStat_ solver922;
  public mjSolverStat_ solver923;
  public mjSolverStat_ solver924;
  public mjSolverStat_ solver925;
  public mjSolverStat_ solver926;
  public mjSolverStat_ solver927;
  public mjSolverStat_ solver928;
  public mjSolverStat_ solver929;
  public mjSolverStat_ solver930;
  public mjSolverStat_ solver931;
  public mjSolverStat_ solver932;
  public mjSolverStat_ solver933;
  public mjSolverStat_ solver934;
  public mjSolverStat_ solver935;
  public mjSolverStat_ solver936;
  public mjSolverStat_ solver937;
  public mjSolverStat_ solver938;
  public mjSolverStat_ solver939;
  public mjSolverStat_ solver940;
  public mjSolverStat_ solver941;
  public mjSolverStat_ solver942;
  public mjSolverStat_ solver943;
  public mjSolverStat_ solver944;
  public mjSolverStat_ solver945;
  public mjSolverStat_ solver946;
  public mjSolverStat_ solver947;
  public mjSolverStat_ solver948;
  public mjSolverStat_ solver949;
  public mjSolverStat_ solver950;
  public mjSolverStat_ solver951;
  public mjSolverStat_ solver952;
  public mjSolverStat_ solver953;
  public mjSolverStat_ solver954;
  public mjSolverStat_ solver955;
  public mjSolverStat_ solver956;
  public mjSolverStat_ solver957;
  public mjSolverStat_ solver958;
  public mjSolverStat_ solver959;
  public mjSolverStat_ solver960;
  public mjSolverStat_ solver961;
  public mjSolverStat_ solver962;
  public mjSolverStat_ solver963;
  public mjSolverStat_ solver964;
  public mjSolverStat_ solver965;
  public mjSolverStat_ solver966;
  public mjSolverStat_ solver967;
  public mjSolverStat_ solver968;
  public mjSolverStat_ solver969;
  public mjSolverStat_ solver970;
  public mjSolverStat_ solver971;
  public mjSolverStat_ solver972;
  public mjSolverStat_ solver973;
  public mjSolverStat_ solver974;
  public mjSolverStat_ solver975;
  public mjSolverStat_ solver976;
  public mjSolverStat_ solver977;
  public mjSolverStat_ solver978;
  public mjSolverStat_ solver979;
  public mjSolverStat_ solver980;
  public mjSolverStat_ solver981;
  public mjSolverStat_ solver982;
  public mjSolverStat_ solver983;
  public mjSolverStat_ solver984;
  public mjSolverStat_ solver985;
  public mjSolverStat_ solver986;
  public mjSolverStat_ solver987;
  public mjSolverStat_ solver988;
  public mjSolverStat_ solver989;
  public mjSolverStat_ solver990;
  public mjSolverStat_ solver991;
  public mjSolverStat_ solver992;
  public mjSolverStat_ solver993;
  public mjSolverStat_ solver994;
  public mjSolverStat_ solver995;
  public mjSolverStat_ solver996;
  public mjSolverStat_ solver997;
  public mjSolverStat_ solver998;
  public mjSolverStat_ solver999;
  public mjSolverStat_ solver1000;
  public mjSolverStat_ solver1001;
  public mjSolverStat_ solver1002;
  public mjSolverStat_ solver1003;
  public mjSolverStat_ solver1004;
  public mjSolverStat_ solver1005;
  public mjSolverStat_ solver1006;
  public mjSolverStat_ solver1007;
  public mjSolverStat_ solver1008;
  public mjSolverStat_ solver1009;
  public mjSolverStat_ solver1010;
  public mjSolverStat_ solver1011;
  public mjSolverStat_ solver1012;
  public mjSolverStat_ solver1013;
  public mjSolverStat_ solver1014;
  public mjSolverStat_ solver1015;
  public mjSolverStat_ solver1016;
  public mjSolverStat_ solver1017;
  public mjSolverStat_ solver1018;
  public mjSolverStat_ solver1019;
  public mjSolverStat_ solver1020;
  public mjSolverStat_ solver1021;
  public mjSolverStat_ solver1022;
  public mjSolverStat_ solver1023;
  public mjSolverStat_ solver1024;
  public mjSolverStat_ solver1025;
  public mjSolverStat_ solver1026;
  public mjSolverStat_ solver1027;
  public mjSolverStat_ solver1028;
  public mjSolverStat_ solver1029;
  public mjSolverStat_ solver1030;
  public mjSolverStat_ solver1031;
  public mjSolverStat_ solver1032;
  public mjSolverStat_ solver1033;
  public mjSolverStat_ solver1034;
  public mjSolverStat_ solver1035;
  public mjSolverStat_ solver1036;
  public mjSolverStat_ solver1037;
  public mjSolverStat_ solver1038;
  public mjSolverStat_ solver1039;
  public mjSolverStat_ solver1040;
  public mjSolverStat_ solver1041;
  public mjSolverStat_ solver1042;
  public mjSolverStat_ solver1043;
  public mjSolverStat_ solver1044;
  public mjSolverStat_ solver1045;
  public mjSolverStat_ solver1046;
  public mjSolverStat_ solver1047;
  public mjSolverStat_ solver1048;
  public mjSolverStat_ solver1049;
  public mjSolverStat_ solver1050;
  public mjSolverStat_ solver1051;
  public mjSolverStat_ solver1052;
  public mjSolverStat_ solver1053;
  public mjSolverStat_ solver1054;
  public mjSolverStat_ solver1055;
  public mjSolverStat_ solver1056;
  public mjSolverStat_ solver1057;
  public mjSolverStat_ solver1058;
  public mjSolverStat_ solver1059;
  public mjSolverStat_ solver1060;
  public mjSolverStat_ solver1061;
  public mjSolverStat_ solver1062;
  public mjSolverStat_ solver1063;
  public mjSolverStat_ solver1064;
  public mjSolverStat_ solver1065;
  public mjSolverStat_ solver1066;
  public mjSolverStat_ solver1067;
  public mjSolverStat_ solver1068;
  public mjSolverStat_ solver1069;
  public mjSolverStat_ solver1070;
  public mjSolverStat_ solver1071;
  public mjSolverStat_ solver1072;
  public mjSolverStat_ solver1073;
  public mjSolverStat_ solver1074;
  public mjSolverStat_ solver1075;
  public mjSolverStat_ solver1076;
  public mjSolverStat_ solver1077;
  public mjSolverStat_ solver1078;
  public mjSolverStat_ solver1079;
  public mjSolverStat_ solver1080;
  public mjSolverStat_ solver1081;
  public mjSolverStat_ solver1082;
  public mjSolverStat_ solver1083;
  public mjSolverStat_ solver1084;
  public mjSolverStat_ solver1085;
  public mjSolverStat_ solver1086;
  public mjSolverStat_ solver1087;
  public mjSolverStat_ solver1088;
  public mjSolverStat_ solver1089;
  public mjSolverStat_ solver1090;
  public mjSolverStat_ solver1091;
  public mjSolverStat_ solver1092;
  public mjSolverStat_ solver1093;
  public mjSolverStat_ solver1094;
  public mjSolverStat_ solver1095;
  public mjSolverStat_ solver1096;
  public mjSolverStat_ solver1097;
  public mjSolverStat_ solver1098;
  public mjSolverStat_ solver1099;
  public mjSolverStat_ solver1100;
  public mjSolverStat_ solver1101;
  public mjSolverStat_ solver1102;
  public mjSolverStat_ solver1103;
  public mjSolverStat_ solver1104;
  public mjSolverStat_ solver1105;
  public mjSolverStat_ solver1106;
  public mjSolverStat_ solver1107;
  public mjSolverStat_ solver1108;
  public mjSolverStat_ solver1109;
  public mjSolverStat_ solver1110;
  public mjSolverStat_ solver1111;
  public mjSolverStat_ solver1112;
  public mjSolverStat_ solver1113;
  public mjSolverStat_ solver1114;
  public mjSolverStat_ solver1115;
  public mjSolverStat_ solver1116;
  public mjSolverStat_ solver1117;
  public mjSolverStat_ solver1118;
  public mjSolverStat_ solver1119;
  public mjSolverStat_ solver1120;
  public mjSolverStat_ solver1121;
  public mjSolverStat_ solver1122;
  public mjSolverStat_ solver1123;
  public mjSolverStat_ solver1124;
  public mjSolverStat_ solver1125;
  public mjSolverStat_ solver1126;
  public mjSolverStat_ solver1127;
  public mjSolverStat_ solver1128;
  public mjSolverStat_ solver1129;
  public mjSolverStat_ solver1130;
  public mjSolverStat_ solver1131;
  public mjSolverStat_ solver1132;
  public mjSolverStat_ solver1133;
  public mjSolverStat_ solver1134;
  public mjSolverStat_ solver1135;
  public mjSolverStat_ solver1136;
  public mjSolverStat_ solver1137;
  public mjSolverStat_ solver1138;
  public mjSolverStat_ solver1139;
  public mjSolverStat_ solver1140;
  public mjSolverStat_ solver1141;
  public mjSolverStat_ solver1142;
  public mjSolverStat_ solver1143;
  public mjSolverStat_ solver1144;
  public mjSolverStat_ solver1145;
  public mjSolverStat_ solver1146;
  public mjSolverStat_ solver1147;
  public mjSolverStat_ solver1148;
  public mjSolverStat_ solver1149;
  public mjSolverStat_ solver1150;
  public mjSolverStat_ solver1151;
  public mjSolverStat_ solver1152;
  public mjSolverStat_ solver1153;
  public mjSolverStat_ solver1154;
  public mjSolverStat_ solver1155;
  public mjSolverStat_ solver1156;
  public mjSolverStat_ solver1157;
  public mjSolverStat_ solver1158;
  public mjSolverStat_ solver1159;
  public mjSolverStat_ solver1160;
  public mjSolverStat_ solver1161;
  public mjSolverStat_ solver1162;
  public mjSolverStat_ solver1163;
  public mjSolverStat_ solver1164;
  public mjSolverStat_ solver1165;
  public mjSolverStat_ solver1166;
  public mjSolverStat_ solver1167;
  public mjSolverStat_ solver1168;
  public mjSolverStat_ solver1169;
  public mjSolverStat_ solver1170;
  public mjSolverStat_ solver1171;
  public mjSolverStat_ solver1172;
  public mjSolverStat_ solver1173;
  public mjSolverStat_ solver1174;
  public mjSolverStat_ solver1175;
  public mjSolverStat_ solver1176;
  public mjSolverStat_ solver1177;
  public mjSolverStat_ solver1178;
  public mjSolverStat_ solver1179;
  public mjSolverStat_ solver1180;
  public mjSolverStat_ solver1181;
  public mjSolverStat_ solver1182;
  public mjSolverStat_ solver1183;
  public mjSolverStat_ solver1184;
  public mjSolverStat_ solver1185;
  public mjSolverStat_ solver1186;
  public mjSolverStat_ solver1187;
  public mjSolverStat_ solver1188;
  public mjSolverStat_ solver1189;
  public mjSolverStat_ solver1190;
  public mjSolverStat_ solver1191;
  public mjSolverStat_ solver1192;
  public mjSolverStat_ solver1193;
  public mjSolverStat_ solver1194;
  public mjSolverStat_ solver1195;
  public mjSolverStat_ solver1196;
  public mjSolverStat_ solver1197;
  public mjSolverStat_ solver1198;
  public mjSolverStat_ solver1199;
  public mjSolverStat_ solver1200;
  public mjSolverStat_ solver1201;
  public mjSolverStat_ solver1202;
  public mjSolverStat_ solver1203;
  public mjSolverStat_ solver1204;
  public mjSolverStat_ solver1205;
  public mjSolverStat_ solver1206;
  public mjSolverStat_ solver1207;
  public mjSolverStat_ solver1208;
  public mjSolverStat_ solver1209;
  public mjSolverStat_ solver1210;
  public mjSolverStat_ solver1211;
  public mjSolverStat_ solver1212;
  public mjSolverStat_ solver1213;
  public mjSolverStat_ solver1214;
  public mjSolverStat_ solver1215;
  public mjSolverStat_ solver1216;
  public mjSolverStat_ solver1217;
  public mjSolverStat_ solver1218;
  public mjSolverStat_ solver1219;
  public mjSolverStat_ solver1220;
  public mjSolverStat_ solver1221;
  public mjSolverStat_ solver1222;
  public mjSolverStat_ solver1223;
  public mjSolverStat_ solver1224;
  public mjSolverStat_ solver1225;
  public mjSolverStat_ solver1226;
  public mjSolverStat_ solver1227;
  public mjSolverStat_ solver1228;
  public mjSolverStat_ solver1229;
  public mjSolverStat_ solver1230;
  public mjSolverStat_ solver1231;
  public mjSolverStat_ solver1232;
  public mjSolverStat_ solver1233;
  public mjSolverStat_ solver1234;
  public mjSolverStat_ solver1235;
  public mjSolverStat_ solver1236;
  public mjSolverStat_ solver1237;
  public mjSolverStat_ solver1238;
  public mjSolverStat_ solver1239;
  public mjSolverStat_ solver1240;
  public mjSolverStat_ solver1241;
  public mjSolverStat_ solver1242;
  public mjSolverStat_ solver1243;
  public mjSolverStat_ solver1244;
  public mjSolverStat_ solver1245;
  public mjSolverStat_ solver1246;
  public mjSolverStat_ solver1247;
  public mjSolverStat_ solver1248;
  public mjSolverStat_ solver1249;
  public mjSolverStat_ solver1250;
  public mjSolverStat_ solver1251;
  public mjSolverStat_ solver1252;
  public mjSolverStat_ solver1253;
  public mjSolverStat_ solver1254;
  public mjSolverStat_ solver1255;
  public mjSolverStat_ solver1256;
  public mjSolverStat_ solver1257;
  public mjSolverStat_ solver1258;
  public mjSolverStat_ solver1259;
  public mjSolverStat_ solver1260;
  public mjSolverStat_ solver1261;
  public mjSolverStat_ solver1262;
  public mjSolverStat_ solver1263;
  public mjSolverStat_ solver1264;
  public mjSolverStat_ solver1265;
  public mjSolverStat_ solver1266;
  public mjSolverStat_ solver1267;
  public mjSolverStat_ solver1268;
  public mjSolverStat_ solver1269;
  public mjSolverStat_ solver1270;
  public mjSolverStat_ solver1271;
  public mjSolverStat_ solver1272;
  public mjSolverStat_ solver1273;
  public mjSolverStat_ solver1274;
  public mjSolverStat_ solver1275;
  public mjSolverStat_ solver1276;
  public mjSolverStat_ solver1277;
  public mjSolverStat_ solver1278;
  public mjSolverStat_ solver1279;
  public mjSolverStat_ solver1280;
  public mjSolverStat_ solver1281;
  public mjSolverStat_ solver1282;
  public mjSolverStat_ solver1283;
  public mjSolverStat_ solver1284;
  public mjSolverStat_ solver1285;
  public mjSolverStat_ solver1286;
  public mjSolverStat_ solver1287;
  public mjSolverStat_ solver1288;
  public mjSolverStat_ solver1289;
  public mjSolverStat_ solver1290;
  public mjSolverStat_ solver1291;
  public mjSolverStat_ solver1292;
  public mjSolverStat_ solver1293;
  public mjSolverStat_ solver1294;
  public mjSolverStat_ solver1295;
  public mjSolverStat_ solver1296;
  public mjSolverStat_ solver1297;
  public mjSolverStat_ solver1298;
  public mjSolverStat_ solver1299;
  public mjSolverStat_ solver1300;
  public mjSolverStat_ solver1301;
  public mjSolverStat_ solver1302;
  public mjSolverStat_ solver1303;
  public mjSolverStat_ solver1304;
  public mjSolverStat_ solver1305;
  public mjSolverStat_ solver1306;
  public mjSolverStat_ solver1307;
  public mjSolverStat_ solver1308;
  public mjSolverStat_ solver1309;
  public mjSolverStat_ solver1310;
  public mjSolverStat_ solver1311;
  public mjSolverStat_ solver1312;
  public mjSolverStat_ solver1313;
  public mjSolverStat_ solver1314;
  public mjSolverStat_ solver1315;
  public mjSolverStat_ solver1316;
  public mjSolverStat_ solver1317;
  public mjSolverStat_ solver1318;
  public mjSolverStat_ solver1319;
  public mjSolverStat_ solver1320;
  public mjSolverStat_ solver1321;
  public mjSolverStat_ solver1322;
  public mjSolverStat_ solver1323;
  public mjSolverStat_ solver1324;
  public mjSolverStat_ solver1325;
  public mjSolverStat_ solver1326;
  public mjSolverStat_ solver1327;
  public mjSolverStat_ solver1328;
  public mjSolverStat_ solver1329;
  public mjSolverStat_ solver1330;
  public mjSolverStat_ solver1331;
  public mjSolverStat_ solver1332;
  public mjSolverStat_ solver1333;
  public mjSolverStat_ solver1334;
  public mjSolverStat_ solver1335;
  public mjSolverStat_ solver1336;
  public mjSolverStat_ solver1337;
  public mjSolverStat_ solver1338;
  public mjSolverStat_ solver1339;
  public mjSolverStat_ solver1340;
  public mjSolverStat_ solver1341;
  public mjSolverStat_ solver1342;
  public mjSolverStat_ solver1343;
  public mjSolverStat_ solver1344;
  public mjSolverStat_ solver1345;
  public mjSolverStat_ solver1346;
  public mjSolverStat_ solver1347;
  public mjSolverStat_ solver1348;
  public mjSolverStat_ solver1349;
  public mjSolverStat_ solver1350;
  public mjSolverStat_ solver1351;
  public mjSolverStat_ solver1352;
  public mjSolverStat_ solver1353;
  public mjSolverStat_ solver1354;
  public mjSolverStat_ solver1355;
  public mjSolverStat_ solver1356;
  public mjSolverStat_ solver1357;
  public mjSolverStat_ solver1358;
  public mjSolverStat_ solver1359;
  public mjSolverStat_ solver1360;
  public mjSolverStat_ solver1361;
  public mjSolverStat_ solver1362;
  public mjSolverStat_ solver1363;
  public mjSolverStat_ solver1364;
  public mjSolverStat_ solver1365;
  public mjSolverStat_ solver1366;
  public mjSolverStat_ solver1367;
  public mjSolverStat_ solver1368;
  public mjSolverStat_ solver1369;
  public mjSolverStat_ solver1370;
  public mjSolverStat_ solver1371;
  public mjSolverStat_ solver1372;
  public mjSolverStat_ solver1373;
  public mjSolverStat_ solver1374;
  public mjSolverStat_ solver1375;
  public mjSolverStat_ solver1376;
  public mjSolverStat_ solver1377;
  public mjSolverStat_ solver1378;
  public mjSolverStat_ solver1379;
  public mjSolverStat_ solver1380;
  public mjSolverStat_ solver1381;
  public mjSolverStat_ solver1382;
  public mjSolverStat_ solver1383;
  public mjSolverStat_ solver1384;
  public mjSolverStat_ solver1385;
  public mjSolverStat_ solver1386;
  public mjSolverStat_ solver1387;
  public mjSolverStat_ solver1388;
  public mjSolverStat_ solver1389;
  public mjSolverStat_ solver1390;
  public mjSolverStat_ solver1391;
  public mjSolverStat_ solver1392;
  public mjSolverStat_ solver1393;
  public mjSolverStat_ solver1394;
  public mjSolverStat_ solver1395;
  public mjSolverStat_ solver1396;
  public mjSolverStat_ solver1397;
  public mjSolverStat_ solver1398;
  public mjSolverStat_ solver1399;
  public mjSolverStat_ solver1400;
  public mjSolverStat_ solver1401;
  public mjSolverStat_ solver1402;
  public mjSolverStat_ solver1403;
  public mjSolverStat_ solver1404;
  public mjSolverStat_ solver1405;
  public mjSolverStat_ solver1406;
  public mjSolverStat_ solver1407;
  public mjSolverStat_ solver1408;
  public mjSolverStat_ solver1409;
  public mjSolverStat_ solver1410;
  public mjSolverStat_ solver1411;
  public mjSolverStat_ solver1412;
  public mjSolverStat_ solver1413;
  public mjSolverStat_ solver1414;
  public mjSolverStat_ solver1415;
  public mjSolverStat_ solver1416;
  public mjSolverStat_ solver1417;
  public mjSolverStat_ solver1418;
  public mjSolverStat_ solver1419;
  public mjSolverStat_ solver1420;
  public mjSolverStat_ solver1421;
  public mjSolverStat_ solver1422;
  public mjSolverStat_ solver1423;
  public mjSolverStat_ solver1424;
  public mjSolverStat_ solver1425;
  public mjSolverStat_ solver1426;
  public mjSolverStat_ solver1427;
  public mjSolverStat_ solver1428;
  public mjSolverStat_ solver1429;
  public mjSolverStat_ solver1430;
  public mjSolverStat_ solver1431;
  public mjSolverStat_ solver1432;
  public mjSolverStat_ solver1433;
  public mjSolverStat_ solver1434;
  public mjSolverStat_ solver1435;
  public mjSolverStat_ solver1436;
  public mjSolverStat_ solver1437;
  public mjSolverStat_ solver1438;
  public mjSolverStat_ solver1439;
  public mjSolverStat_ solver1440;
  public mjSolverStat_ solver1441;
  public mjSolverStat_ solver1442;
  public mjSolverStat_ solver1443;
  public mjSolverStat_ solver1444;
  public mjSolverStat_ solver1445;
  public mjSolverStat_ solver1446;
  public mjSolverStat_ solver1447;
  public mjSolverStat_ solver1448;
  public mjSolverStat_ solver1449;
  public mjSolverStat_ solver1450;
  public mjSolverStat_ solver1451;
  public mjSolverStat_ solver1452;
  public mjSolverStat_ solver1453;
  public mjSolverStat_ solver1454;
  public mjSolverStat_ solver1455;
  public mjSolverStat_ solver1456;
  public mjSolverStat_ solver1457;
  public mjSolverStat_ solver1458;
  public mjSolverStat_ solver1459;
  public mjSolverStat_ solver1460;
  public mjSolverStat_ solver1461;
  public mjSolverStat_ solver1462;
  public mjSolverStat_ solver1463;
  public mjSolverStat_ solver1464;
  public mjSolverStat_ solver1465;
  public mjSolverStat_ solver1466;
  public mjSolverStat_ solver1467;
  public mjSolverStat_ solver1468;
  public mjSolverStat_ solver1469;
  public mjSolverStat_ solver1470;
  public mjSolverStat_ solver1471;
  public mjSolverStat_ solver1472;
  public mjSolverStat_ solver1473;
  public mjSolverStat_ solver1474;
  public mjSolverStat_ solver1475;
  public mjSolverStat_ solver1476;
  public mjSolverStat_ solver1477;
  public mjSolverStat_ solver1478;
  public mjSolverStat_ solver1479;
  public mjSolverStat_ solver1480;
  public mjSolverStat_ solver1481;
  public mjSolverStat_ solver1482;
  public mjSolverStat_ solver1483;
  public mjSolverStat_ solver1484;
  public mjSolverStat_ solver1485;
  public mjSolverStat_ solver1486;
  public mjSolverStat_ solver1487;
  public mjSolverStat_ solver1488;
  public mjSolverStat_ solver1489;
  public mjSolverStat_ solver1490;
  public mjSolverStat_ solver1491;
  public mjSolverStat_ solver1492;
  public mjSolverStat_ solver1493;
  public mjSolverStat_ solver1494;
  public mjSolverStat_ solver1495;
  public mjSolverStat_ solver1496;
  public mjSolverStat_ solver1497;
  public mjSolverStat_ solver1498;
  public mjSolverStat_ solver1499;
  public mjSolverStat_ solver1500;
  public mjSolverStat_ solver1501;
  public mjSolverStat_ solver1502;
  public mjSolverStat_ solver1503;
  public mjSolverStat_ solver1504;
  public mjSolverStat_ solver1505;
  public mjSolverStat_ solver1506;
  public mjSolverStat_ solver1507;
  public mjSolverStat_ solver1508;
  public mjSolverStat_ solver1509;
  public mjSolverStat_ solver1510;
  public mjSolverStat_ solver1511;
  public mjSolverStat_ solver1512;
  public mjSolverStat_ solver1513;
  public mjSolverStat_ solver1514;
  public mjSolverStat_ solver1515;
  public mjSolverStat_ solver1516;
  public mjSolverStat_ solver1517;
  public mjSolverStat_ solver1518;
  public mjSolverStat_ solver1519;
  public mjSolverStat_ solver1520;
  public mjSolverStat_ solver1521;
  public mjSolverStat_ solver1522;
  public mjSolverStat_ solver1523;
  public mjSolverStat_ solver1524;
  public mjSolverStat_ solver1525;
  public mjSolverStat_ solver1526;
  public mjSolverStat_ solver1527;
  public mjSolverStat_ solver1528;
  public mjSolverStat_ solver1529;
  public mjSolverStat_ solver1530;
  public mjSolverStat_ solver1531;
  public mjSolverStat_ solver1532;
  public mjSolverStat_ solver1533;
  public mjSolverStat_ solver1534;
  public mjSolverStat_ solver1535;
  public mjSolverStat_ solver1536;
  public mjSolverStat_ solver1537;
  public mjSolverStat_ solver1538;
  public mjSolverStat_ solver1539;
  public mjSolverStat_ solver1540;
  public mjSolverStat_ solver1541;
  public mjSolverStat_ solver1542;
  public mjSolverStat_ solver1543;
  public mjSolverStat_ solver1544;
  public mjSolverStat_ solver1545;
  public mjSolverStat_ solver1546;
  public mjSolverStat_ solver1547;
  public mjSolverStat_ solver1548;
  public mjSolverStat_ solver1549;
  public mjSolverStat_ solver1550;
  public mjSolverStat_ solver1551;
  public mjSolverStat_ solver1552;
  public mjSolverStat_ solver1553;
  public mjSolverStat_ solver1554;
  public mjSolverStat_ solver1555;
  public mjSolverStat_ solver1556;
  public mjSolverStat_ solver1557;
  public mjSolverStat_ solver1558;
  public mjSolverStat_ solver1559;
  public mjSolverStat_ solver1560;
  public mjSolverStat_ solver1561;
  public mjSolverStat_ solver1562;
  public mjSolverStat_ solver1563;
  public mjSolverStat_ solver1564;
  public mjSolverStat_ solver1565;
  public mjSolverStat_ solver1566;
  public mjSolverStat_ solver1567;
  public mjSolverStat_ solver1568;
  public mjSolverStat_ solver1569;
  public mjSolverStat_ solver1570;
  public mjSolverStat_ solver1571;
  public mjSolverStat_ solver1572;
  public mjSolverStat_ solver1573;
  public mjSolverStat_ solver1574;
  public mjSolverStat_ solver1575;
  public mjSolverStat_ solver1576;
  public mjSolverStat_ solver1577;
  public mjSolverStat_ solver1578;
  public mjSolverStat_ solver1579;
  public mjSolverStat_ solver1580;
  public mjSolverStat_ solver1581;
  public mjSolverStat_ solver1582;
  public mjSolverStat_ solver1583;
  public mjSolverStat_ solver1584;
  public mjSolverStat_ solver1585;
  public mjSolverStat_ solver1586;
  public mjSolverStat_ solver1587;
  public mjSolverStat_ solver1588;
  public mjSolverStat_ solver1589;
  public mjSolverStat_ solver1590;
  public mjSolverStat_ solver1591;
  public mjSolverStat_ solver1592;
  public mjSolverStat_ solver1593;
  public mjSolverStat_ solver1594;
  public mjSolverStat_ solver1595;
  public mjSolverStat_ solver1596;
  public mjSolverStat_ solver1597;
  public mjSolverStat_ solver1598;
  public mjSolverStat_ solver1599;
  public mjSolverStat_ solver1600;
  public mjSolverStat_ solver1601;
  public mjSolverStat_ solver1602;
  public mjSolverStat_ solver1603;
  public mjSolverStat_ solver1604;
  public mjSolverStat_ solver1605;
  public mjSolverStat_ solver1606;
  public mjSolverStat_ solver1607;
  public mjSolverStat_ solver1608;
  public mjSolverStat_ solver1609;
  public mjSolverStat_ solver1610;
  public mjSolverStat_ solver1611;
  public mjSolverStat_ solver1612;
  public mjSolverStat_ solver1613;
  public mjSolverStat_ solver1614;
  public mjSolverStat_ solver1615;
  public mjSolverStat_ solver1616;
  public mjSolverStat_ solver1617;
  public mjSolverStat_ solver1618;
  public mjSolverStat_ solver1619;
  public mjSolverStat_ solver1620;
  public mjSolverStat_ solver1621;
  public mjSolverStat_ solver1622;
  public mjSolverStat_ solver1623;
  public mjSolverStat_ solver1624;
  public mjSolverStat_ solver1625;
  public mjSolverStat_ solver1626;
  public mjSolverStat_ solver1627;
  public mjSolverStat_ solver1628;
  public mjSolverStat_ solver1629;
  public mjSolverStat_ solver1630;
  public mjSolverStat_ solver1631;
  public mjSolverStat_ solver1632;
  public mjSolverStat_ solver1633;
  public mjSolverStat_ solver1634;
  public mjSolverStat_ solver1635;
  public mjSolverStat_ solver1636;
  public mjSolverStat_ solver1637;
  public mjSolverStat_ solver1638;
  public mjSolverStat_ solver1639;
  public mjSolverStat_ solver1640;
  public mjSolverStat_ solver1641;
  public mjSolverStat_ solver1642;
  public mjSolverStat_ solver1643;
  public mjSolverStat_ solver1644;
  public mjSolverStat_ solver1645;
  public mjSolverStat_ solver1646;
  public mjSolverStat_ solver1647;
  public mjSolverStat_ solver1648;
  public mjSolverStat_ solver1649;
  public mjSolverStat_ solver1650;
  public mjSolverStat_ solver1651;
  public mjSolverStat_ solver1652;
  public mjSolverStat_ solver1653;
  public mjSolverStat_ solver1654;
  public mjSolverStat_ solver1655;
  public mjSolverStat_ solver1656;
  public mjSolverStat_ solver1657;
  public mjSolverStat_ solver1658;
  public mjSolverStat_ solver1659;
  public mjSolverStat_ solver1660;
  public mjSolverStat_ solver1661;
  public mjSolverStat_ solver1662;
  public mjSolverStat_ solver1663;
  public mjSolverStat_ solver1664;
  public mjSolverStat_ solver1665;
  public mjSolverStat_ solver1666;
  public mjSolverStat_ solver1667;
  public mjSolverStat_ solver1668;
  public mjSolverStat_ solver1669;
  public mjSolverStat_ solver1670;
  public mjSolverStat_ solver1671;
  public mjSolverStat_ solver1672;
  public mjSolverStat_ solver1673;
  public mjSolverStat_ solver1674;
  public mjSolverStat_ solver1675;
  public mjSolverStat_ solver1676;
  public mjSolverStat_ solver1677;
  public mjSolverStat_ solver1678;
  public mjSolverStat_ solver1679;
  public mjSolverStat_ solver1680;
  public mjSolverStat_ solver1681;
  public mjSolverStat_ solver1682;
  public mjSolverStat_ solver1683;
  public mjSolverStat_ solver1684;
  public mjSolverStat_ solver1685;
  public mjSolverStat_ solver1686;
  public mjSolverStat_ solver1687;
  public mjSolverStat_ solver1688;
  public mjSolverStat_ solver1689;
  public mjSolverStat_ solver1690;
  public mjSolverStat_ solver1691;
  public mjSolverStat_ solver1692;
  public mjSolverStat_ solver1693;
  public mjSolverStat_ solver1694;
  public mjSolverStat_ solver1695;
  public mjSolverStat_ solver1696;
  public mjSolverStat_ solver1697;
  public mjSolverStat_ solver1698;
  public mjSolverStat_ solver1699;
  public mjSolverStat_ solver1700;
  public mjSolverStat_ solver1701;
  public mjSolverStat_ solver1702;
  public mjSolverStat_ solver1703;
  public mjSolverStat_ solver1704;
  public mjSolverStat_ solver1705;
  public mjSolverStat_ solver1706;
  public mjSolverStat_ solver1707;
  public mjSolverStat_ solver1708;
  public mjSolverStat_ solver1709;
  public mjSolverStat_ solver1710;
  public mjSolverStat_ solver1711;
  public mjSolverStat_ solver1712;
  public mjSolverStat_ solver1713;
  public mjSolverStat_ solver1714;
  public mjSolverStat_ solver1715;
  public mjSolverStat_ solver1716;
  public mjSolverStat_ solver1717;
  public mjSolverStat_ solver1718;
  public mjSolverStat_ solver1719;
  public mjSolverStat_ solver1720;
  public mjSolverStat_ solver1721;
  public mjSolverStat_ solver1722;
  public mjSolverStat_ solver1723;
  public mjSolverStat_ solver1724;
  public mjSolverStat_ solver1725;
  public mjSolverStat_ solver1726;
  public mjSolverStat_ solver1727;
  public mjSolverStat_ solver1728;
  public mjSolverStat_ solver1729;
  public mjSolverStat_ solver1730;
  public mjSolverStat_ solver1731;
  public mjSolverStat_ solver1732;
  public mjSolverStat_ solver1733;
  public mjSolverStat_ solver1734;
  public mjSolverStat_ solver1735;
  public mjSolverStat_ solver1736;
  public mjSolverStat_ solver1737;
  public mjSolverStat_ solver1738;
  public mjSolverStat_ solver1739;
  public mjSolverStat_ solver1740;
  public mjSolverStat_ solver1741;
  public mjSolverStat_ solver1742;
  public mjSolverStat_ solver1743;
  public mjSolverStat_ solver1744;
  public mjSolverStat_ solver1745;
  public mjSolverStat_ solver1746;
  public mjSolverStat_ solver1747;
  public mjSolverStat_ solver1748;
  public mjSolverStat_ solver1749;
  public mjSolverStat_ solver1750;
  public mjSolverStat_ solver1751;
  public mjSolverStat_ solver1752;
  public mjSolverStat_ solver1753;
  public mjSolverStat_ solver1754;
  public mjSolverStat_ solver1755;
  public mjSolverStat_ solver1756;
  public mjSolverStat_ solver1757;
  public mjSolverStat_ solver1758;
  public mjSolverStat_ solver1759;
  public mjSolverStat_ solver1760;
  public mjSolverStat_ solver1761;
  public mjSolverStat_ solver1762;
  public mjSolverStat_ solver1763;
  public mjSolverStat_ solver1764;
  public mjSolverStat_ solver1765;
  public mjSolverStat_ solver1766;
  public mjSolverStat_ solver1767;
  public mjSolverStat_ solver1768;
  public mjSolverStat_ solver1769;
  public mjSolverStat_ solver1770;
  public mjSolverStat_ solver1771;
  public mjSolverStat_ solver1772;
  public mjSolverStat_ solver1773;
  public mjSolverStat_ solver1774;
  public mjSolverStat_ solver1775;
  public mjSolverStat_ solver1776;
  public mjSolverStat_ solver1777;
  public mjSolverStat_ solver1778;
  public mjSolverStat_ solver1779;
  public mjSolverStat_ solver1780;
  public mjSolverStat_ solver1781;
  public mjSolverStat_ solver1782;
  public mjSolverStat_ solver1783;
  public mjSolverStat_ solver1784;
  public mjSolverStat_ solver1785;
  public mjSolverStat_ solver1786;
  public mjSolverStat_ solver1787;
  public mjSolverStat_ solver1788;
  public mjSolverStat_ solver1789;
  public mjSolverStat_ solver1790;
  public mjSolverStat_ solver1791;
  public mjSolverStat_ solver1792;
  public mjSolverStat_ solver1793;
  public mjSolverStat_ solver1794;
  public mjSolverStat_ solver1795;
  public mjSolverStat_ solver1796;
  public mjSolverStat_ solver1797;
  public mjSolverStat_ solver1798;
  public mjSolverStat_ solver1799;
  public mjSolverStat_ solver1800;
  public mjSolverStat_ solver1801;
  public mjSolverStat_ solver1802;
  public mjSolverStat_ solver1803;
  public mjSolverStat_ solver1804;
  public mjSolverStat_ solver1805;
  public mjSolverStat_ solver1806;
  public mjSolverStat_ solver1807;
  public mjSolverStat_ solver1808;
  public mjSolverStat_ solver1809;
  public mjSolverStat_ solver1810;
  public mjSolverStat_ solver1811;
  public mjSolverStat_ solver1812;
  public mjSolverStat_ solver1813;
  public mjSolverStat_ solver1814;
  public mjSolverStat_ solver1815;
  public mjSolverStat_ solver1816;
  public mjSolverStat_ solver1817;
  public mjSolverStat_ solver1818;
  public mjSolverStat_ solver1819;
  public mjSolverStat_ solver1820;
  public mjSolverStat_ solver1821;
  public mjSolverStat_ solver1822;
  public mjSolverStat_ solver1823;
  public mjSolverStat_ solver1824;
  public mjSolverStat_ solver1825;
  public mjSolverStat_ solver1826;
  public mjSolverStat_ solver1827;
  public mjSolverStat_ solver1828;
  public mjSolverStat_ solver1829;
  public mjSolverStat_ solver1830;
  public mjSolverStat_ solver1831;
  public mjSolverStat_ solver1832;
  public mjSolverStat_ solver1833;
  public mjSolverStat_ solver1834;
  public mjSolverStat_ solver1835;
  public mjSolverStat_ solver1836;
  public mjSolverStat_ solver1837;
  public mjSolverStat_ solver1838;
  public mjSolverStat_ solver1839;
  public mjSolverStat_ solver1840;
  public mjSolverStat_ solver1841;
  public mjSolverStat_ solver1842;
  public mjSolverStat_ solver1843;
  public mjSolverStat_ solver1844;
  public mjSolverStat_ solver1845;
  public mjSolverStat_ solver1846;
  public mjSolverStat_ solver1847;
  public mjSolverStat_ solver1848;
  public mjSolverStat_ solver1849;
  public mjSolverStat_ solver1850;
  public mjSolverStat_ solver1851;
  public mjSolverStat_ solver1852;
  public mjSolverStat_ solver1853;
  public mjSolverStat_ solver1854;
  public mjSolverStat_ solver1855;
  public mjSolverStat_ solver1856;
  public mjSolverStat_ solver1857;
  public mjSolverStat_ solver1858;
  public mjSolverStat_ solver1859;
  public mjSolverStat_ solver1860;
  public mjSolverStat_ solver1861;
  public mjSolverStat_ solver1862;
  public mjSolverStat_ solver1863;
  public mjSolverStat_ solver1864;
  public mjSolverStat_ solver1865;
  public mjSolverStat_ solver1866;
  public mjSolverStat_ solver1867;
  public mjSolverStat_ solver1868;
  public mjSolverStat_ solver1869;
  public mjSolverStat_ solver1870;
  public mjSolverStat_ solver1871;
  public mjSolverStat_ solver1872;
  public mjSolverStat_ solver1873;
  public mjSolverStat_ solver1874;
  public mjSolverStat_ solver1875;
  public mjSolverStat_ solver1876;
  public mjSolverStat_ solver1877;
  public mjSolverStat_ solver1878;
  public mjSolverStat_ solver1879;
  public mjSolverStat_ solver1880;
  public mjSolverStat_ solver1881;
  public mjSolverStat_ solver1882;
  public mjSolverStat_ solver1883;
  public mjSolverStat_ solver1884;
  public mjSolverStat_ solver1885;
  public mjSolverStat_ solver1886;
  public mjSolverStat_ solver1887;
  public mjSolverStat_ solver1888;
  public mjSolverStat_ solver1889;
  public mjSolverStat_ solver1890;
  public mjSolverStat_ solver1891;
  public mjSolverStat_ solver1892;
  public mjSolverStat_ solver1893;
  public mjSolverStat_ solver1894;
  public mjSolverStat_ solver1895;
  public mjSolverStat_ solver1896;
  public mjSolverStat_ solver1897;
  public mjSolverStat_ solver1898;
  public mjSolverStat_ solver1899;
  public mjSolverStat_ solver1900;
  public mjSolverStat_ solver1901;
  public mjSolverStat_ solver1902;
  public mjSolverStat_ solver1903;
  public mjSolverStat_ solver1904;
  public mjSolverStat_ solver1905;
  public mjSolverStat_ solver1906;
  public mjSolverStat_ solver1907;
  public mjSolverStat_ solver1908;
  public mjSolverStat_ solver1909;
  public mjSolverStat_ solver1910;
  public mjSolverStat_ solver1911;
  public mjSolverStat_ solver1912;
  public mjSolverStat_ solver1913;
  public mjSolverStat_ solver1914;
  public mjSolverStat_ solver1915;
  public mjSolverStat_ solver1916;
  public mjSolverStat_ solver1917;
  public mjSolverStat_ solver1918;
  public mjSolverStat_ solver1919;
  public mjSolverStat_ solver1920;
  public mjSolverStat_ solver1921;
  public mjSolverStat_ solver1922;
  public mjSolverStat_ solver1923;
  public mjSolverStat_ solver1924;
  public mjSolverStat_ solver1925;
  public mjSolverStat_ solver1926;
  public mjSolverStat_ solver1927;
  public mjSolverStat_ solver1928;
  public mjSolverStat_ solver1929;
  public mjSolverStat_ solver1930;
  public mjSolverStat_ solver1931;
  public mjSolverStat_ solver1932;
  public mjSolverStat_ solver1933;
  public mjSolverStat_ solver1934;
  public mjSolverStat_ solver1935;
  public mjSolverStat_ solver1936;
  public mjSolverStat_ solver1937;
  public mjSolverStat_ solver1938;
  public mjSolverStat_ solver1939;
  public mjSolverStat_ solver1940;
  public mjSolverStat_ solver1941;
  public mjSolverStat_ solver1942;
  public mjSolverStat_ solver1943;
  public mjSolverStat_ solver1944;
  public mjSolverStat_ solver1945;
  public mjSolverStat_ solver1946;
  public mjSolverStat_ solver1947;
  public mjSolverStat_ solver1948;
  public mjSolverStat_ solver1949;
  public mjSolverStat_ solver1950;
  public mjSolverStat_ solver1951;
  public mjSolverStat_ solver1952;
  public mjSolverStat_ solver1953;
  public mjSolverStat_ solver1954;
  public mjSolverStat_ solver1955;
  public mjSolverStat_ solver1956;
  public mjSolverStat_ solver1957;
  public mjSolverStat_ solver1958;
  public mjSolverStat_ solver1959;
  public mjSolverStat_ solver1960;
  public mjSolverStat_ solver1961;
  public mjSolverStat_ solver1962;
  public mjSolverStat_ solver1963;
  public mjSolverStat_ solver1964;
  public mjSolverStat_ solver1965;
  public mjSolverStat_ solver1966;
  public mjSolverStat_ solver1967;
  public mjSolverStat_ solver1968;
  public mjSolverStat_ solver1969;
  public mjSolverStat_ solver1970;
  public mjSolverStat_ solver1971;
  public mjSolverStat_ solver1972;
  public mjSolverStat_ solver1973;
  public mjSolverStat_ solver1974;
  public mjSolverStat_ solver1975;
  public mjSolverStat_ solver1976;
  public mjSolverStat_ solver1977;
  public mjSolverStat_ solver1978;
  public mjSolverStat_ solver1979;
  public mjSolverStat_ solver1980;
  public mjSolverStat_ solver1981;
  public mjSolverStat_ solver1982;
  public mjSolverStat_ solver1983;
  public mjSolverStat_ solver1984;
  public mjSolverStat_ solver1985;
  public mjSolverStat_ solver1986;
  public mjSolverStat_ solver1987;
  public mjSolverStat_ solver1988;
  public mjSolverStat_ solver1989;
  public mjSolverStat_ solver1990;
  public mjSolverStat_ solver1991;
  public mjSolverStat_ solver1992;
  public mjSolverStat_ solver1993;
  public mjSolverStat_ solver1994;
  public mjSolverStat_ solver1995;
  public mjSolverStat_ solver1996;
  public mjSolverStat_ solver1997;
  public mjSolverStat_ solver1998;
  public mjSolverStat_ solver1999;
  public mjSolverStat_ solver2000;
  public mjSolverStat_ solver2001;
  public mjSolverStat_ solver2002;
  public mjSolverStat_ solver2003;
  public mjSolverStat_ solver2004;
  public mjSolverStat_ solver2005;
  public mjSolverStat_ solver2006;
  public mjSolverStat_ solver2007;
  public mjSolverStat_ solver2008;
  public mjSolverStat_ solver2009;
  public mjSolverStat_ solver2010;
  public mjSolverStat_ solver2011;
  public mjSolverStat_ solver2012;
  public mjSolverStat_ solver2013;
  public mjSolverStat_ solver2014;
  public mjSolverStat_ solver2015;
  public mjSolverStat_ solver2016;
  public mjSolverStat_ solver2017;
  public mjSolverStat_ solver2018;
  public mjSolverStat_ solver2019;
  public mjSolverStat_ solver2020;
  public mjSolverStat_ solver2021;
  public mjSolverStat_ solver2022;
  public mjSolverStat_ solver2023;
  public mjSolverStat_ solver2024;
  public mjSolverStat_ solver2025;
  public mjSolverStat_ solver2026;
  public mjSolverStat_ solver2027;
  public mjSolverStat_ solver2028;
  public mjSolverStat_ solver2029;
  public mjSolverStat_ solver2030;
  public mjSolverStat_ solver2031;
  public mjSolverStat_ solver2032;
  public mjSolverStat_ solver2033;
  public mjSolverStat_ solver2034;
  public mjSolverStat_ solver2035;
  public mjSolverStat_ solver2036;
  public mjSolverStat_ solver2037;
  public mjSolverStat_ solver2038;
  public mjSolverStat_ solver2039;
  public mjSolverStat_ solver2040;
  public mjSolverStat_ solver2041;
  public mjSolverStat_ solver2042;
  public mjSolverStat_ solver2043;
  public mjSolverStat_ solver2044;
  public mjSolverStat_ solver2045;
  public mjSolverStat_ solver2046;
  public mjSolverStat_ solver2047;
  public mjSolverStat_ solver2048;
  public mjSolverStat_ solver2049;
  public mjSolverStat_ solver2050;
  public mjSolverStat_ solver2051;
  public mjSolverStat_ solver2052;
  public mjSolverStat_ solver2053;
  public mjSolverStat_ solver2054;
  public mjSolverStat_ solver2055;
  public mjSolverStat_ solver2056;
  public mjSolverStat_ solver2057;
  public mjSolverStat_ solver2058;
  public mjSolverStat_ solver2059;
  public mjSolverStat_ solver2060;
  public mjSolverStat_ solver2061;
  public mjSolverStat_ solver2062;
  public mjSolverStat_ solver2063;
  public mjSolverStat_ solver2064;
  public mjSolverStat_ solver2065;
  public mjSolverStat_ solver2066;
  public mjSolverStat_ solver2067;
  public mjSolverStat_ solver2068;
  public mjSolverStat_ solver2069;
  public mjSolverStat_ solver2070;
  public mjSolverStat_ solver2071;
  public mjSolverStat_ solver2072;
  public mjSolverStat_ solver2073;
  public mjSolverStat_ solver2074;
  public mjSolverStat_ solver2075;
  public mjSolverStat_ solver2076;
  public mjSolverStat_ solver2077;
  public mjSolverStat_ solver2078;
  public mjSolverStat_ solver2079;
  public mjSolverStat_ solver2080;
  public mjSolverStat_ solver2081;
  public mjSolverStat_ solver2082;
  public mjSolverStat_ solver2083;
  public mjSolverStat_ solver2084;
  public mjSolverStat_ solver2085;
  public mjSolverStat_ solver2086;
  public mjSolverStat_ solver2087;
  public mjSolverStat_ solver2088;
  public mjSolverStat_ solver2089;
  public mjSolverStat_ solver2090;
  public mjSolverStat_ solver2091;
  public mjSolverStat_ solver2092;
  public mjSolverStat_ solver2093;
  public mjSolverStat_ solver2094;
  public mjSolverStat_ solver2095;
  public mjSolverStat_ solver2096;
  public mjSolverStat_ solver2097;
  public mjSolverStat_ solver2098;
  public mjSolverStat_ solver2099;
  public mjSolverStat_ solver2100;
  public mjSolverStat_ solver2101;
  public mjSolverStat_ solver2102;
  public mjSolverStat_ solver2103;
  public mjSolverStat_ solver2104;
  public mjSolverStat_ solver2105;
  public mjSolverStat_ solver2106;
  public mjSolverStat_ solver2107;
  public mjSolverStat_ solver2108;
  public mjSolverStat_ solver2109;
  public mjSolverStat_ solver2110;
  public mjSolverStat_ solver2111;
  public mjSolverStat_ solver2112;
  public mjSolverStat_ solver2113;
  public mjSolverStat_ solver2114;
  public mjSolverStat_ solver2115;
  public mjSolverStat_ solver2116;
  public mjSolverStat_ solver2117;
  public mjSolverStat_ solver2118;
  public mjSolverStat_ solver2119;
  public mjSolverStat_ solver2120;
  public mjSolverStat_ solver2121;
  public mjSolverStat_ solver2122;
  public mjSolverStat_ solver2123;
  public mjSolverStat_ solver2124;
  public mjSolverStat_ solver2125;
  public mjSolverStat_ solver2126;
  public mjSolverStat_ solver2127;
  public mjSolverStat_ solver2128;
  public mjSolverStat_ solver2129;
  public mjSolverStat_ solver2130;
  public mjSolverStat_ solver2131;
  public mjSolverStat_ solver2132;
  public mjSolverStat_ solver2133;
  public mjSolverStat_ solver2134;
  public mjSolverStat_ solver2135;
  public mjSolverStat_ solver2136;
  public mjSolverStat_ solver2137;
  public mjSolverStat_ solver2138;
  public mjSolverStat_ solver2139;
  public mjSolverStat_ solver2140;
  public mjSolverStat_ solver2141;
  public mjSolverStat_ solver2142;
  public mjSolverStat_ solver2143;
  public mjSolverStat_ solver2144;
  public mjSolverStat_ solver2145;
  public mjSolverStat_ solver2146;
  public mjSolverStat_ solver2147;
  public mjSolverStat_ solver2148;
  public mjSolverStat_ solver2149;
  public mjSolverStat_ solver2150;
  public mjSolverStat_ solver2151;
  public mjSolverStat_ solver2152;
  public mjSolverStat_ solver2153;
  public mjSolverStat_ solver2154;
  public mjSolverStat_ solver2155;
  public mjSolverStat_ solver2156;
  public mjSolverStat_ solver2157;
  public mjSolverStat_ solver2158;
  public mjSolverStat_ solver2159;
  public mjSolverStat_ solver2160;
  public mjSolverStat_ solver2161;
  public mjSolverStat_ solver2162;
  public mjSolverStat_ solver2163;
  public mjSolverStat_ solver2164;
  public mjSolverStat_ solver2165;
  public mjSolverStat_ solver2166;
  public mjSolverStat_ solver2167;
  public mjSolverStat_ solver2168;
  public mjSolverStat_ solver2169;
  public mjSolverStat_ solver2170;
  public mjSolverStat_ solver2171;
  public mjSolverStat_ solver2172;
  public mjSolverStat_ solver2173;
  public mjSolverStat_ solver2174;
  public mjSolverStat_ solver2175;
  public mjSolverStat_ solver2176;
  public mjSolverStat_ solver2177;
  public mjSolverStat_ solver2178;
  public mjSolverStat_ solver2179;
  public mjSolverStat_ solver2180;
  public mjSolverStat_ solver2181;
  public mjSolverStat_ solver2182;
  public mjSolverStat_ solver2183;
  public mjSolverStat_ solver2184;
  public mjSolverStat_ solver2185;
  public mjSolverStat_ solver2186;
  public mjSolverStat_ solver2187;
  public mjSolverStat_ solver2188;
  public mjSolverStat_ solver2189;
  public mjSolverStat_ solver2190;
  public mjSolverStat_ solver2191;
  public mjSolverStat_ solver2192;
  public mjSolverStat_ solver2193;
  public mjSolverStat_ solver2194;
  public mjSolverStat_ solver2195;
  public mjSolverStat_ solver2196;
  public mjSolverStat_ solver2197;
  public mjSolverStat_ solver2198;
  public mjSolverStat_ solver2199;
  public mjSolverStat_ solver2200;
  public mjSolverStat_ solver2201;
  public mjSolverStat_ solver2202;
  public mjSolverStat_ solver2203;
  public mjSolverStat_ solver2204;
  public mjSolverStat_ solver2205;
  public mjSolverStat_ solver2206;
  public mjSolverStat_ solver2207;
  public mjSolverStat_ solver2208;
  public mjSolverStat_ solver2209;
  public mjSolverStat_ solver2210;
  public mjSolverStat_ solver2211;
  public mjSolverStat_ solver2212;
  public mjSolverStat_ solver2213;
  public mjSolverStat_ solver2214;
  public mjSolverStat_ solver2215;
  public mjSolverStat_ solver2216;
  public mjSolverStat_ solver2217;
  public mjSolverStat_ solver2218;
  public mjSolverStat_ solver2219;
  public mjSolverStat_ solver2220;
  public mjSolverStat_ solver2221;
  public mjSolverStat_ solver2222;
  public mjSolverStat_ solver2223;
  public mjSolverStat_ solver2224;
  public mjSolverStat_ solver2225;
  public mjSolverStat_ solver2226;
  public mjSolverStat_ solver2227;
  public mjSolverStat_ solver2228;
  public mjSolverStat_ solver2229;
  public mjSolverStat_ solver2230;
  public mjSolverStat_ solver2231;
  public mjSolverStat_ solver2232;
  public mjSolverStat_ solver2233;
  public mjSolverStat_ solver2234;
  public mjSolverStat_ solver2235;
  public mjSolverStat_ solver2236;
  public mjSolverStat_ solver2237;
  public mjSolverStat_ solver2238;
  public mjSolverStat_ solver2239;
  public mjSolverStat_ solver2240;
  public mjSolverStat_ solver2241;
  public mjSolverStat_ solver2242;
  public mjSolverStat_ solver2243;
  public mjSolverStat_ solver2244;
  public mjSolverStat_ solver2245;
  public mjSolverStat_ solver2246;
  public mjSolverStat_ solver2247;
  public mjSolverStat_ solver2248;
  public mjSolverStat_ solver2249;
  public mjSolverStat_ solver2250;
  public mjSolverStat_ solver2251;
  public mjSolverStat_ solver2252;
  public mjSolverStat_ solver2253;
  public mjSolverStat_ solver2254;
  public mjSolverStat_ solver2255;
  public mjSolverStat_ solver2256;
  public mjSolverStat_ solver2257;
  public mjSolverStat_ solver2258;
  public mjSolverStat_ solver2259;
  public mjSolverStat_ solver2260;
  public mjSolverStat_ solver2261;
  public mjSolverStat_ solver2262;
  public mjSolverStat_ solver2263;
  public mjSolverStat_ solver2264;
  public mjSolverStat_ solver2265;
  public mjSolverStat_ solver2266;
  public mjSolverStat_ solver2267;
  public mjSolverStat_ solver2268;
  public mjSolverStat_ solver2269;
  public mjSolverStat_ solver2270;
  public mjSolverStat_ solver2271;
  public mjSolverStat_ solver2272;
  public mjSolverStat_ solver2273;
  public mjSolverStat_ solver2274;
  public mjSolverStat_ solver2275;
  public mjSolverStat_ solver2276;
  public mjSolverStat_ solver2277;
  public mjSolverStat_ solver2278;
  public mjSolverStat_ solver2279;
  public mjSolverStat_ solver2280;
  public mjSolverStat_ solver2281;
  public mjSolverStat_ solver2282;
  public mjSolverStat_ solver2283;
  public mjSolverStat_ solver2284;
  public mjSolverStat_ solver2285;
  public mjSolverStat_ solver2286;
  public mjSolverStat_ solver2287;
  public mjSolverStat_ solver2288;
  public mjSolverStat_ solver2289;
  public mjSolverStat_ solver2290;
  public mjSolverStat_ solver2291;
  public mjSolverStat_ solver2292;
  public mjSolverStat_ solver2293;
  public mjSolverStat_ solver2294;
  public mjSolverStat_ solver2295;
  public mjSolverStat_ solver2296;
  public mjSolverStat_ solver2297;
  public mjSolverStat_ solver2298;
  public mjSolverStat_ solver2299;
  public mjSolverStat_ solver2300;
  public mjSolverStat_ solver2301;
  public mjSolverStat_ solver2302;
  public mjSolverStat_ solver2303;
  public mjSolverStat_ solver2304;
  public mjSolverStat_ solver2305;
  public mjSolverStat_ solver2306;
  public mjSolverStat_ solver2307;
  public mjSolverStat_ solver2308;
  public mjSolverStat_ solver2309;
  public mjSolverStat_ solver2310;
  public mjSolverStat_ solver2311;
  public mjSolverStat_ solver2312;
  public mjSolverStat_ solver2313;
  public mjSolverStat_ solver2314;
  public mjSolverStat_ solver2315;
  public mjSolverStat_ solver2316;
  public mjSolverStat_ solver2317;
  public mjSolverStat_ solver2318;
  public mjSolverStat_ solver2319;
  public mjSolverStat_ solver2320;
  public mjSolverStat_ solver2321;
  public mjSolverStat_ solver2322;
  public mjSolverStat_ solver2323;
  public mjSolverStat_ solver2324;
  public mjSolverStat_ solver2325;
  public mjSolverStat_ solver2326;
  public mjSolverStat_ solver2327;
  public mjSolverStat_ solver2328;
  public mjSolverStat_ solver2329;
  public mjSolverStat_ solver2330;
  public mjSolverStat_ solver2331;
  public mjSolverStat_ solver2332;
  public mjSolverStat_ solver2333;
  public mjSolverStat_ solver2334;
  public mjSolverStat_ solver2335;
  public mjSolverStat_ solver2336;
  public mjSolverStat_ solver2337;
  public mjSolverStat_ solver2338;
  public mjSolverStat_ solver2339;
  public mjSolverStat_ solver2340;
  public mjSolverStat_ solver2341;
  public mjSolverStat_ solver2342;
  public mjSolverStat_ solver2343;
  public mjSolverStat_ solver2344;
  public mjSolverStat_ solver2345;
  public mjSolverStat_ solver2346;
  public mjSolverStat_ solver2347;
  public mjSolverStat_ solver2348;
  public mjSolverStat_ solver2349;
  public mjSolverStat_ solver2350;
  public mjSolverStat_ solver2351;
  public mjSolverStat_ solver2352;
  public mjSolverStat_ solver2353;
  public mjSolverStat_ solver2354;
  public mjSolverStat_ solver2355;
  public mjSolverStat_ solver2356;
  public mjSolverStat_ solver2357;
  public mjSolverStat_ solver2358;
  public mjSolverStat_ solver2359;
  public mjSolverStat_ solver2360;
  public mjSolverStat_ solver2361;
  public mjSolverStat_ solver2362;
  public mjSolverStat_ solver2363;
  public mjSolverStat_ solver2364;
  public mjSolverStat_ solver2365;
  public mjSolverStat_ solver2366;
  public mjSolverStat_ solver2367;
  public mjSolverStat_ solver2368;
  public mjSolverStat_ solver2369;
  public mjSolverStat_ solver2370;
  public mjSolverStat_ solver2371;
  public mjSolverStat_ solver2372;
  public mjSolverStat_ solver2373;
  public mjSolverStat_ solver2374;
  public mjSolverStat_ solver2375;
  public mjSolverStat_ solver2376;
  public mjSolverStat_ solver2377;
  public mjSolverStat_ solver2378;
  public mjSolverStat_ solver2379;
  public mjSolverStat_ solver2380;
  public mjSolverStat_ solver2381;
  public mjSolverStat_ solver2382;
  public mjSolverStat_ solver2383;
  public mjSolverStat_ solver2384;
  public mjSolverStat_ solver2385;
  public mjSolverStat_ solver2386;
  public mjSolverStat_ solver2387;
  public mjSolverStat_ solver2388;
  public mjSolverStat_ solver2389;
  public mjSolverStat_ solver2390;
  public mjSolverStat_ solver2391;
  public mjSolverStat_ solver2392;
  public mjSolverStat_ solver2393;
  public mjSolverStat_ solver2394;
  public mjSolverStat_ solver2395;
  public mjSolverStat_ solver2396;
  public mjSolverStat_ solver2397;
  public mjSolverStat_ solver2398;
  public mjSolverStat_ solver2399;
  public mjSolverStat_ solver2400;
  public mjSolverStat_ solver2401;
  public mjSolverStat_ solver2402;
  public mjSolverStat_ solver2403;
  public mjSolverStat_ solver2404;
  public mjSolverStat_ solver2405;
  public mjSolverStat_ solver2406;
  public mjSolverStat_ solver2407;
  public mjSolverStat_ solver2408;
  public mjSolverStat_ solver2409;
  public mjSolverStat_ solver2410;
  public mjSolverStat_ solver2411;
  public mjSolverStat_ solver2412;
  public mjSolverStat_ solver2413;
  public mjSolverStat_ solver2414;
  public mjSolverStat_ solver2415;
  public mjSolverStat_ solver2416;
  public mjSolverStat_ solver2417;
  public mjSolverStat_ solver2418;
  public mjSolverStat_ solver2419;
  public mjSolverStat_ solver2420;
  public mjSolverStat_ solver2421;
  public mjSolverStat_ solver2422;
  public mjSolverStat_ solver2423;
  public mjSolverStat_ solver2424;
  public mjSolverStat_ solver2425;
  public mjSolverStat_ solver2426;
  public mjSolverStat_ solver2427;
  public mjSolverStat_ solver2428;
  public mjSolverStat_ solver2429;
  public mjSolverStat_ solver2430;
  public mjSolverStat_ solver2431;
  public mjSolverStat_ solver2432;
  public mjSolverStat_ solver2433;
  public mjSolverStat_ solver2434;
  public mjSolverStat_ solver2435;
  public mjSolverStat_ solver2436;
  public mjSolverStat_ solver2437;
  public mjSolverStat_ solver2438;
  public mjSolverStat_ solver2439;
  public mjSolverStat_ solver2440;
  public mjSolverStat_ solver2441;
  public mjSolverStat_ solver2442;
  public mjSolverStat_ solver2443;
  public mjSolverStat_ solver2444;
  public mjSolverStat_ solver2445;
  public mjSolverStat_ solver2446;
  public mjSolverStat_ solver2447;
  public mjSolverStat_ solver2448;
  public mjSolverStat_ solver2449;
  public mjSolverStat_ solver2450;
  public mjSolverStat_ solver2451;
  public mjSolverStat_ solver2452;
  public mjSolverStat_ solver2453;
  public mjSolverStat_ solver2454;
  public mjSolverStat_ solver2455;
  public mjSolverStat_ solver2456;
  public mjSolverStat_ solver2457;
  public mjSolverStat_ solver2458;
  public mjSolverStat_ solver2459;
  public mjSolverStat_ solver2460;
  public mjSolverStat_ solver2461;
  public mjSolverStat_ solver2462;
  public mjSolverStat_ solver2463;
  public mjSolverStat_ solver2464;
  public mjSolverStat_ solver2465;
  public mjSolverStat_ solver2466;
  public mjSolverStat_ solver2467;
  public mjSolverStat_ solver2468;
  public mjSolverStat_ solver2469;
  public mjSolverStat_ solver2470;
  public mjSolverStat_ solver2471;
  public mjSolverStat_ solver2472;
  public mjSolverStat_ solver2473;
  public mjSolverStat_ solver2474;
  public mjSolverStat_ solver2475;
  public mjSolverStat_ solver2476;
  public mjSolverStat_ solver2477;
  public mjSolverStat_ solver2478;
  public mjSolverStat_ solver2479;
  public mjSolverStat_ solver2480;
  public mjSolverStat_ solver2481;
  public mjSolverStat_ solver2482;
  public mjSolverStat_ solver2483;
  public mjSolverStat_ solver2484;
  public mjSolverStat_ solver2485;
  public mjSolverStat_ solver2486;
  public mjSolverStat_ solver2487;
  public mjSolverStat_ solver2488;
  public mjSolverStat_ solver2489;
  public mjSolverStat_ solver2490;
  public mjSolverStat_ solver2491;
  public mjSolverStat_ solver2492;
  public mjSolverStat_ solver2493;
  public mjSolverStat_ solver2494;
  public mjSolverStat_ solver2495;
  public mjSolverStat_ solver2496;
  public mjSolverStat_ solver2497;
  public mjSolverStat_ solver2498;
  public mjSolverStat_ solver2499;
  public mjSolverStat_ solver2500;
  public mjSolverStat_ solver2501;
  public mjSolverStat_ solver2502;
  public mjSolverStat_ solver2503;
  public mjSolverStat_ solver2504;
  public mjSolverStat_ solver2505;
  public mjSolverStat_ solver2506;
  public mjSolverStat_ solver2507;
  public mjSolverStat_ solver2508;
  public mjSolverStat_ solver2509;
  public mjSolverStat_ solver2510;
  public mjSolverStat_ solver2511;
  public mjSolverStat_ solver2512;
  public mjSolverStat_ solver2513;
  public mjSolverStat_ solver2514;
  public mjSolverStat_ solver2515;
  public mjSolverStat_ solver2516;
  public mjSolverStat_ solver2517;
  public mjSolverStat_ solver2518;
  public mjSolverStat_ solver2519;
  public mjSolverStat_ solver2520;
  public mjSolverStat_ solver2521;
  public mjSolverStat_ solver2522;
  public mjSolverStat_ solver2523;
  public mjSolverStat_ solver2524;
  public mjSolverStat_ solver2525;
  public mjSolverStat_ solver2526;
  public mjSolverStat_ solver2527;
  public mjSolverStat_ solver2528;
  public mjSolverStat_ solver2529;
  public mjSolverStat_ solver2530;
  public mjSolverStat_ solver2531;
  public mjSolverStat_ solver2532;
  public mjSolverStat_ solver2533;
  public mjSolverStat_ solver2534;
  public mjSolverStat_ solver2535;
  public mjSolverStat_ solver2536;
  public mjSolverStat_ solver2537;
  public mjSolverStat_ solver2538;
  public mjSolverStat_ solver2539;
  public mjSolverStat_ solver2540;
  public mjSolverStat_ solver2541;
  public mjSolverStat_ solver2542;
  public mjSolverStat_ solver2543;
  public mjSolverStat_ solver2544;
  public mjSolverStat_ solver2545;
  public mjSolverStat_ solver2546;
  public mjSolverStat_ solver2547;
  public mjSolverStat_ solver2548;
  public mjSolverStat_ solver2549;
  public mjSolverStat_ solver2550;
  public mjSolverStat_ solver2551;
  public mjSolverStat_ solver2552;
  public mjSolverStat_ solver2553;
  public mjSolverStat_ solver2554;
  public mjSolverStat_ solver2555;
  public mjSolverStat_ solver2556;
  public mjSolverStat_ solver2557;
  public mjSolverStat_ solver2558;
  public mjSolverStat_ solver2559;
  public mjSolverStat_ solver2560;
  public mjSolverStat_ solver2561;
  public mjSolverStat_ solver2562;
  public mjSolverStat_ solver2563;
  public mjSolverStat_ solver2564;
  public mjSolverStat_ solver2565;
  public mjSolverStat_ solver2566;
  public mjSolverStat_ solver2567;
  public mjSolverStat_ solver2568;
  public mjSolverStat_ solver2569;
  public mjSolverStat_ solver2570;
  public mjSolverStat_ solver2571;
  public mjSolverStat_ solver2572;
  public mjSolverStat_ solver2573;
  public mjSolverStat_ solver2574;
  public mjSolverStat_ solver2575;
  public mjSolverStat_ solver2576;
  public mjSolverStat_ solver2577;
  public mjSolverStat_ solver2578;
  public mjSolverStat_ solver2579;
  public mjSolverStat_ solver2580;
  public mjSolverStat_ solver2581;
  public mjSolverStat_ solver2582;
  public mjSolverStat_ solver2583;
  public mjSolverStat_ solver2584;
  public mjSolverStat_ solver2585;
  public mjSolverStat_ solver2586;
  public mjSolverStat_ solver2587;
  public mjSolverStat_ solver2588;
  public mjSolverStat_ solver2589;
  public mjSolverStat_ solver2590;
  public mjSolverStat_ solver2591;
  public mjSolverStat_ solver2592;
  public mjSolverStat_ solver2593;
  public mjSolverStat_ solver2594;
  public mjSolverStat_ solver2595;
  public mjSolverStat_ solver2596;
  public mjSolverStat_ solver2597;
  public mjSolverStat_ solver2598;
  public mjSolverStat_ solver2599;
  public mjSolverStat_ solver2600;
  public mjSolverStat_ solver2601;
  public mjSolverStat_ solver2602;
  public mjSolverStat_ solver2603;
  public mjSolverStat_ solver2604;
  public mjSolverStat_ solver2605;
  public mjSolverStat_ solver2606;
  public mjSolverStat_ solver2607;
  public mjSolverStat_ solver2608;
  public mjSolverStat_ solver2609;
  public mjSolverStat_ solver2610;
  public mjSolverStat_ solver2611;
  public mjSolverStat_ solver2612;
  public mjSolverStat_ solver2613;
  public mjSolverStat_ solver2614;
  public mjSolverStat_ solver2615;
  public mjSolverStat_ solver2616;
  public mjSolverStat_ solver2617;
  public mjSolverStat_ solver2618;
  public mjSolverStat_ solver2619;
  public mjSolverStat_ solver2620;
  public mjSolverStat_ solver2621;
  public mjSolverStat_ solver2622;
  public mjSolverStat_ solver2623;
  public mjSolverStat_ solver2624;
  public mjSolverStat_ solver2625;
  public mjSolverStat_ solver2626;
  public mjSolverStat_ solver2627;
  public mjSolverStat_ solver2628;
  public mjSolverStat_ solver2629;
  public mjSolverStat_ solver2630;
  public mjSolverStat_ solver2631;
  public mjSolverStat_ solver2632;
  public mjSolverStat_ solver2633;
  public mjSolverStat_ solver2634;
  public mjSolverStat_ solver2635;
  public mjSolverStat_ solver2636;
  public mjSolverStat_ solver2637;
  public mjSolverStat_ solver2638;
  public mjSolverStat_ solver2639;
  public mjSolverStat_ solver2640;
  public mjSolverStat_ solver2641;
  public mjSolverStat_ solver2642;
  public mjSolverStat_ solver2643;
  public mjSolverStat_ solver2644;
  public mjSolverStat_ solver2645;
  public mjSolverStat_ solver2646;
  public mjSolverStat_ solver2647;
  public mjSolverStat_ solver2648;
  public mjSolverStat_ solver2649;
  public mjSolverStat_ solver2650;
  public mjSolverStat_ solver2651;
  public mjSolverStat_ solver2652;
  public mjSolverStat_ solver2653;
  public mjSolverStat_ solver2654;
  public mjSolverStat_ solver2655;
  public mjSolverStat_ solver2656;
  public mjSolverStat_ solver2657;
  public mjSolverStat_ solver2658;
  public mjSolverStat_ solver2659;
  public mjSolverStat_ solver2660;
  public mjSolverStat_ solver2661;
  public mjSolverStat_ solver2662;
  public mjSolverStat_ solver2663;
  public mjSolverStat_ solver2664;
  public mjSolverStat_ solver2665;
  public mjSolverStat_ solver2666;
  public mjSolverStat_ solver2667;
  public mjSolverStat_ solver2668;
  public mjSolverStat_ solver2669;
  public mjSolverStat_ solver2670;
  public mjSolverStat_ solver2671;
  public mjSolverStat_ solver2672;
  public mjSolverStat_ solver2673;
  public mjSolverStat_ solver2674;
  public mjSolverStat_ solver2675;
  public mjSolverStat_ solver2676;
  public mjSolverStat_ solver2677;
  public mjSolverStat_ solver2678;
  public mjSolverStat_ solver2679;
  public mjSolverStat_ solver2680;
  public mjSolverStat_ solver2681;
  public mjSolverStat_ solver2682;
  public mjSolverStat_ solver2683;
  public mjSolverStat_ solver2684;
  public mjSolverStat_ solver2685;
  public mjSolverStat_ solver2686;
  public mjSolverStat_ solver2687;
  public mjSolverStat_ solver2688;
  public mjSolverStat_ solver2689;
  public mjSolverStat_ solver2690;
  public mjSolverStat_ solver2691;
  public mjSolverStat_ solver2692;
  public mjSolverStat_ solver2693;
  public mjSolverStat_ solver2694;
  public mjSolverStat_ solver2695;
  public mjSolverStat_ solver2696;
  public mjSolverStat_ solver2697;
  public mjSolverStat_ solver2698;
  public mjSolverStat_ solver2699;
  public mjSolverStat_ solver2700;
  public mjSolverStat_ solver2701;
  public mjSolverStat_ solver2702;
  public mjSolverStat_ solver2703;
  public mjSolverStat_ solver2704;
  public mjSolverStat_ solver2705;
  public mjSolverStat_ solver2706;
  public mjSolverStat_ solver2707;
  public mjSolverStat_ solver2708;
  public mjSolverStat_ solver2709;
  public mjSolverStat_ solver2710;
  public mjSolverStat_ solver2711;
  public mjSolverStat_ solver2712;
  public mjSolverStat_ solver2713;
  public mjSolverStat_ solver2714;
  public mjSolverStat_ solver2715;
  public mjSolverStat_ solver2716;
  public mjSolverStat_ solver2717;
  public mjSolverStat_ solver2718;
  public mjSolverStat_ solver2719;
  public mjSolverStat_ solver2720;
  public mjSolverStat_ solver2721;
  public mjSolverStat_ solver2722;
  public mjSolverStat_ solver2723;
  public mjSolverStat_ solver2724;
  public mjSolverStat_ solver2725;
  public mjSolverStat_ solver2726;
  public mjSolverStat_ solver2727;
  public mjSolverStat_ solver2728;
  public mjSolverStat_ solver2729;
  public mjSolverStat_ solver2730;
  public mjSolverStat_ solver2731;
  public mjSolverStat_ solver2732;
  public mjSolverStat_ solver2733;
  public mjSolverStat_ solver2734;
  public mjSolverStat_ solver2735;
  public mjSolverStat_ solver2736;
  public mjSolverStat_ solver2737;
  public mjSolverStat_ solver2738;
  public mjSolverStat_ solver2739;
  public mjSolverStat_ solver2740;
  public mjSolverStat_ solver2741;
  public mjSolverStat_ solver2742;
  public mjSolverStat_ solver2743;
  public mjSolverStat_ solver2744;
  public mjSolverStat_ solver2745;
  public mjSolverStat_ solver2746;
  public mjSolverStat_ solver2747;
  public mjSolverStat_ solver2748;
  public mjSolverStat_ solver2749;
  public mjSolverStat_ solver2750;
  public mjSolverStat_ solver2751;
  public mjSolverStat_ solver2752;
  public mjSolverStat_ solver2753;
  public mjSolverStat_ solver2754;
  public mjSolverStat_ solver2755;
  public mjSolverStat_ solver2756;
  public mjSolverStat_ solver2757;
  public mjSolverStat_ solver2758;
  public mjSolverStat_ solver2759;
  public mjSolverStat_ solver2760;
  public mjSolverStat_ solver2761;
  public mjSolverStat_ solver2762;
  public mjSolverStat_ solver2763;
  public mjSolverStat_ solver2764;
  public mjSolverStat_ solver2765;
  public mjSolverStat_ solver2766;
  public mjSolverStat_ solver2767;
  public mjSolverStat_ solver2768;
  public mjSolverStat_ solver2769;
  public mjSolverStat_ solver2770;
  public mjSolverStat_ solver2771;
  public mjSolverStat_ solver2772;
  public mjSolverStat_ solver2773;
  public mjSolverStat_ solver2774;
  public mjSolverStat_ solver2775;
  public mjSolverStat_ solver2776;
  public mjSolverStat_ solver2777;
  public mjSolverStat_ solver2778;
  public mjSolverStat_ solver2779;
  public mjSolverStat_ solver2780;
  public mjSolverStat_ solver2781;
  public mjSolverStat_ solver2782;
  public mjSolverStat_ solver2783;
  public mjSolverStat_ solver2784;
  public mjSolverStat_ solver2785;
  public mjSolverStat_ solver2786;
  public mjSolverStat_ solver2787;
  public mjSolverStat_ solver2788;
  public mjSolverStat_ solver2789;
  public mjSolverStat_ solver2790;
  public mjSolverStat_ solver2791;
  public mjSolverStat_ solver2792;
  public mjSolverStat_ solver2793;
  public mjSolverStat_ solver2794;
  public mjSolverStat_ solver2795;
  public mjSolverStat_ solver2796;
  public mjSolverStat_ solver2797;
  public mjSolverStat_ solver2798;
  public mjSolverStat_ solver2799;
  public mjSolverStat_ solver2800;
  public mjSolverStat_ solver2801;
  public mjSolverStat_ solver2802;
  public mjSolverStat_ solver2803;
  public mjSolverStat_ solver2804;
  public mjSolverStat_ solver2805;
  public mjSolverStat_ solver2806;
  public mjSolverStat_ solver2807;
  public mjSolverStat_ solver2808;
  public mjSolverStat_ solver2809;
  public mjSolverStat_ solver2810;
  public mjSolverStat_ solver2811;
  public mjSolverStat_ solver2812;
  public mjSolverStat_ solver2813;
  public mjSolverStat_ solver2814;
  public mjSolverStat_ solver2815;
  public mjSolverStat_ solver2816;
  public mjSolverStat_ solver2817;
  public mjSolverStat_ solver2818;
  public mjSolverStat_ solver2819;
  public mjSolverStat_ solver2820;
  public mjSolverStat_ solver2821;
  public mjSolverStat_ solver2822;
  public mjSolverStat_ solver2823;
  public mjSolverStat_ solver2824;
  public mjSolverStat_ solver2825;
  public mjSolverStat_ solver2826;
  public mjSolverStat_ solver2827;
  public mjSolverStat_ solver2828;
  public mjSolverStat_ solver2829;
  public mjSolverStat_ solver2830;
  public mjSolverStat_ solver2831;
  public mjSolverStat_ solver2832;
  public mjSolverStat_ solver2833;
  public mjSolverStat_ solver2834;
  public mjSolverStat_ solver2835;
  public mjSolverStat_ solver2836;
  public mjSolverStat_ solver2837;
  public mjSolverStat_ solver2838;
  public mjSolverStat_ solver2839;
  public mjSolverStat_ solver2840;
  public mjSolverStat_ solver2841;
  public mjSolverStat_ solver2842;
  public mjSolverStat_ solver2843;
  public mjSolverStat_ solver2844;
  public mjSolverStat_ solver2845;
  public mjSolverStat_ solver2846;
  public mjSolverStat_ solver2847;
  public mjSolverStat_ solver2848;
  public mjSolverStat_ solver2849;
  public mjSolverStat_ solver2850;
  public mjSolverStat_ solver2851;
  public mjSolverStat_ solver2852;
  public mjSolverStat_ solver2853;
  public mjSolverStat_ solver2854;
  public mjSolverStat_ solver2855;
  public mjSolverStat_ solver2856;
  public mjSolverStat_ solver2857;
  public mjSolverStat_ solver2858;
  public mjSolverStat_ solver2859;
  public mjSolverStat_ solver2860;
  public mjSolverStat_ solver2861;
  public mjSolverStat_ solver2862;
  public mjSolverStat_ solver2863;
  public mjSolverStat_ solver2864;
  public mjSolverStat_ solver2865;
  public mjSolverStat_ solver2866;
  public mjSolverStat_ solver2867;
  public mjSolverStat_ solver2868;
  public mjSolverStat_ solver2869;
  public mjSolverStat_ solver2870;
  public mjSolverStat_ solver2871;
  public mjSolverStat_ solver2872;
  public mjSolverStat_ solver2873;
  public mjSolverStat_ solver2874;
  public mjSolverStat_ solver2875;
  public mjSolverStat_ solver2876;
  public mjSolverStat_ solver2877;
  public mjSolverStat_ solver2878;
  public mjSolverStat_ solver2879;
  public mjSolverStat_ solver2880;
  public mjSolverStat_ solver2881;
  public mjSolverStat_ solver2882;
  public mjSolverStat_ solver2883;
  public mjSolverStat_ solver2884;
  public mjSolverStat_ solver2885;
  public mjSolverStat_ solver2886;
  public mjSolverStat_ solver2887;
  public mjSolverStat_ solver2888;
  public mjSolverStat_ solver2889;
  public mjSolverStat_ solver2890;
  public mjSolverStat_ solver2891;
  public mjSolverStat_ solver2892;
  public mjSolverStat_ solver2893;
  public mjSolverStat_ solver2894;
  public mjSolverStat_ solver2895;
  public mjSolverStat_ solver2896;
  public mjSolverStat_ solver2897;
  public mjSolverStat_ solver2898;
  public mjSolverStat_ solver2899;
  public mjSolverStat_ solver2900;
  public mjSolverStat_ solver2901;
  public mjSolverStat_ solver2902;
  public mjSolverStat_ solver2903;
  public mjSolverStat_ solver2904;
  public mjSolverStat_ solver2905;
  public mjSolverStat_ solver2906;
  public mjSolverStat_ solver2907;
  public mjSolverStat_ solver2908;
  public mjSolverStat_ solver2909;
  public mjSolverStat_ solver2910;
  public mjSolverStat_ solver2911;
  public mjSolverStat_ solver2912;
  public mjSolverStat_ solver2913;
  public mjSolverStat_ solver2914;
  public mjSolverStat_ solver2915;
  public mjSolverStat_ solver2916;
  public mjSolverStat_ solver2917;
  public mjSolverStat_ solver2918;
  public mjSolverStat_ solver2919;
  public mjSolverStat_ solver2920;
  public mjSolverStat_ solver2921;
  public mjSolverStat_ solver2922;
  public mjSolverStat_ solver2923;
  public mjSolverStat_ solver2924;
  public mjSolverStat_ solver2925;
  public mjSolverStat_ solver2926;
  public mjSolverStat_ solver2927;
  public mjSolverStat_ solver2928;
  public mjSolverStat_ solver2929;
  public mjSolverStat_ solver2930;
  public mjSolverStat_ solver2931;
  public mjSolverStat_ solver2932;
  public mjSolverStat_ solver2933;
  public mjSolverStat_ solver2934;
  public mjSolverStat_ solver2935;
  public mjSolverStat_ solver2936;
  public mjSolverStat_ solver2937;
  public mjSolverStat_ solver2938;
  public mjSolverStat_ solver2939;
  public mjSolverStat_ solver2940;
  public mjSolverStat_ solver2941;
  public mjSolverStat_ solver2942;
  public mjSolverStat_ solver2943;
  public mjSolverStat_ solver2944;
  public mjSolverStat_ solver2945;
  public mjSolverStat_ solver2946;
  public mjSolverStat_ solver2947;
  public mjSolverStat_ solver2948;
  public mjSolverStat_ solver2949;
  public mjSolverStat_ solver2950;
  public mjSolverStat_ solver2951;
  public mjSolverStat_ solver2952;
  public mjSolverStat_ solver2953;
  public mjSolverStat_ solver2954;
  public mjSolverStat_ solver2955;
  public mjSolverStat_ solver2956;
  public mjSolverStat_ solver2957;
  public mjSolverStat_ solver2958;
  public mjSolverStat_ solver2959;
  public mjSolverStat_ solver2960;
  public mjSolverStat_ solver2961;
  public mjSolverStat_ solver2962;
  public mjSolverStat_ solver2963;
  public mjSolverStat_ solver2964;
  public mjSolverStat_ solver2965;
  public mjSolverStat_ solver2966;
  public mjSolverStat_ solver2967;
  public mjSolverStat_ solver2968;
  public mjSolverStat_ solver2969;
  public mjSolverStat_ solver2970;
  public mjSolverStat_ solver2971;
  public mjSolverStat_ solver2972;
  public mjSolverStat_ solver2973;
  public mjSolverStat_ solver2974;
  public mjSolverStat_ solver2975;
  public mjSolverStat_ solver2976;
  public mjSolverStat_ solver2977;
  public mjSolverStat_ solver2978;
  public mjSolverStat_ solver2979;
  public mjSolverStat_ solver2980;
  public mjSolverStat_ solver2981;
  public mjSolverStat_ solver2982;
  public mjSolverStat_ solver2983;
  public mjSolverStat_ solver2984;
  public mjSolverStat_ solver2985;
  public mjSolverStat_ solver2986;
  public mjSolverStat_ solver2987;
  public mjSolverStat_ solver2988;
  public mjSolverStat_ solver2989;
  public mjSolverStat_ solver2990;
  public mjSolverStat_ solver2991;
  public mjSolverStat_ solver2992;
  public mjSolverStat_ solver2993;
  public mjSolverStat_ solver2994;
  public mjSolverStat_ solver2995;
  public mjSolverStat_ solver2996;
  public mjSolverStat_ solver2997;
  public mjSolverStat_ solver2998;
  public mjSolverStat_ solver2999;
  public mjSolverStat_ solver3000;
  public mjSolverStat_ solver3001;
  public mjSolverStat_ solver3002;
  public mjSolverStat_ solver3003;
  public mjSolverStat_ solver3004;
  public mjSolverStat_ solver3005;
  public mjSolverStat_ solver3006;
  public mjSolverStat_ solver3007;
  public mjSolverStat_ solver3008;
  public mjSolverStat_ solver3009;
  public mjSolverStat_ solver3010;
  public mjSolverStat_ solver3011;
  public mjSolverStat_ solver3012;
  public mjSolverStat_ solver3013;
  public mjSolverStat_ solver3014;
  public mjSolverStat_ solver3015;
  public mjSolverStat_ solver3016;
  public mjSolverStat_ solver3017;
  public mjSolverStat_ solver3018;
  public mjSolverStat_ solver3019;
  public mjSolverStat_ solver3020;
  public mjSolverStat_ solver3021;
  public mjSolverStat_ solver3022;
  public mjSolverStat_ solver3023;
  public mjSolverStat_ solver3024;
  public mjSolverStat_ solver3025;
  public mjSolverStat_ solver3026;
  public mjSolverStat_ solver3027;
  public mjSolverStat_ solver3028;
  public mjSolverStat_ solver3029;
  public mjSolverStat_ solver3030;
  public mjSolverStat_ solver3031;
  public mjSolverStat_ solver3032;
  public mjSolverStat_ solver3033;
  public mjSolverStat_ solver3034;
  public mjSolverStat_ solver3035;
  public mjSolverStat_ solver3036;
  public mjSolverStat_ solver3037;
  public mjSolverStat_ solver3038;
  public mjSolverStat_ solver3039;
  public mjSolverStat_ solver3040;
  public mjSolverStat_ solver3041;
  public mjSolverStat_ solver3042;
  public mjSolverStat_ solver3043;
  public mjSolverStat_ solver3044;
  public mjSolverStat_ solver3045;
  public mjSolverStat_ solver3046;
  public mjSolverStat_ solver3047;
  public mjSolverStat_ solver3048;
  public mjSolverStat_ solver3049;
  public mjSolverStat_ solver3050;
  public mjSolverStat_ solver3051;
  public mjSolverStat_ solver3052;
  public mjSolverStat_ solver3053;
  public mjSolverStat_ solver3054;
  public mjSolverStat_ solver3055;
  public mjSolverStat_ solver3056;
  public mjSolverStat_ solver3057;
  public mjSolverStat_ solver3058;
  public mjSolverStat_ solver3059;
  public mjSolverStat_ solver3060;
  public mjSolverStat_ solver3061;
  public mjSolverStat_ solver3062;
  public mjSolverStat_ solver3063;
  public mjSolverStat_ solver3064;
  public mjSolverStat_ solver3065;
  public mjSolverStat_ solver3066;
  public mjSolverStat_ solver3067;
  public mjSolverStat_ solver3068;
  public mjSolverStat_ solver3069;
  public mjSolverStat_ solver3070;
  public mjSolverStat_ solver3071;
  public mjSolverStat_ solver3072;
  public mjSolverStat_ solver3073;
  public mjSolverStat_ solver3074;
  public mjSolverStat_ solver3075;
  public mjSolverStat_ solver3076;
  public mjSolverStat_ solver3077;
  public mjSolverStat_ solver3078;
  public mjSolverStat_ solver3079;
  public mjSolverStat_ solver3080;
  public mjSolverStat_ solver3081;
  public mjSolverStat_ solver3082;
  public mjSolverStat_ solver3083;
  public mjSolverStat_ solver3084;
  public mjSolverStat_ solver3085;
  public mjSolverStat_ solver3086;
  public mjSolverStat_ solver3087;
  public mjSolverStat_ solver3088;
  public mjSolverStat_ solver3089;
  public mjSolverStat_ solver3090;
  public mjSolverStat_ solver3091;
  public mjSolverStat_ solver3092;
  public mjSolverStat_ solver3093;
  public mjSolverStat_ solver3094;
  public mjSolverStat_ solver3095;
  public mjSolverStat_ solver3096;
  public mjSolverStat_ solver3097;
  public mjSolverStat_ solver3098;
  public mjSolverStat_ solver3099;
  public mjSolverStat_ solver3100;
  public mjSolverStat_ solver3101;
  public mjSolverStat_ solver3102;
  public mjSolverStat_ solver3103;
  public mjSolverStat_ solver3104;
  public mjSolverStat_ solver3105;
  public mjSolverStat_ solver3106;
  public mjSolverStat_ solver3107;
  public mjSolverStat_ solver3108;
  public mjSolverStat_ solver3109;
  public mjSolverStat_ solver3110;
  public mjSolverStat_ solver3111;
  public mjSolverStat_ solver3112;
  public mjSolverStat_ solver3113;
  public mjSolverStat_ solver3114;
  public mjSolverStat_ solver3115;
  public mjSolverStat_ solver3116;
  public mjSolverStat_ solver3117;
  public mjSolverStat_ solver3118;
  public mjSolverStat_ solver3119;
  public mjSolverStat_ solver3120;
  public mjSolverStat_ solver3121;
  public mjSolverStat_ solver3122;
  public mjSolverStat_ solver3123;
  public mjSolverStat_ solver3124;
  public mjSolverStat_ solver3125;
  public mjSolverStat_ solver3126;
  public mjSolverStat_ solver3127;
  public mjSolverStat_ solver3128;
  public mjSolverStat_ solver3129;
  public mjSolverStat_ solver3130;
  public mjSolverStat_ solver3131;
  public mjSolverStat_ solver3132;
  public mjSolverStat_ solver3133;
  public mjSolverStat_ solver3134;
  public mjSolverStat_ solver3135;
  public mjSolverStat_ solver3136;
  public mjSolverStat_ solver3137;
  public mjSolverStat_ solver3138;
  public mjSolverStat_ solver3139;
  public mjSolverStat_ solver3140;
  public mjSolverStat_ solver3141;
  public mjSolverStat_ solver3142;
  public mjSolverStat_ solver3143;
  public mjSolverStat_ solver3144;
  public mjSolverStat_ solver3145;
  public mjSolverStat_ solver3146;
  public mjSolverStat_ solver3147;
  public mjSolverStat_ solver3148;
  public mjSolverStat_ solver3149;
  public mjSolverStat_ solver3150;
  public mjSolverStat_ solver3151;
  public mjSolverStat_ solver3152;
  public mjSolverStat_ solver3153;
  public mjSolverStat_ solver3154;
  public mjSolverStat_ solver3155;
  public mjSolverStat_ solver3156;
  public mjSolverStat_ solver3157;
  public mjSolverStat_ solver3158;
  public mjSolverStat_ solver3159;
  public mjSolverStat_ solver3160;
  public mjSolverStat_ solver3161;
  public mjSolverStat_ solver3162;
  public mjSolverStat_ solver3163;
  public mjSolverStat_ solver3164;
  public mjSolverStat_ solver3165;
  public mjSolverStat_ solver3166;
  public mjSolverStat_ solver3167;
  public mjSolverStat_ solver3168;
  public mjSolverStat_ solver3169;
  public mjSolverStat_ solver3170;
  public mjSolverStat_ solver3171;
  public mjSolverStat_ solver3172;
  public mjSolverStat_ solver3173;
  public mjSolverStat_ solver3174;
  public mjSolverStat_ solver3175;
  public mjSolverStat_ solver3176;
  public mjSolverStat_ solver3177;
  public mjSolverStat_ solver3178;
  public mjSolverStat_ solver3179;
  public mjSolverStat_ solver3180;
  public mjSolverStat_ solver3181;
  public mjSolverStat_ solver3182;
  public mjSolverStat_ solver3183;
  public mjSolverStat_ solver3184;
  public mjSolverStat_ solver3185;
  public mjSolverStat_ solver3186;
  public mjSolverStat_ solver3187;
  public mjSolverStat_ solver3188;
  public mjSolverStat_ solver3189;
  public mjSolverStat_ solver3190;
  public mjSolverStat_ solver3191;
  public mjSolverStat_ solver3192;
  public mjSolverStat_ solver3193;
  public mjSolverStat_ solver3194;
  public mjSolverStat_ solver3195;
  public mjSolverStat_ solver3196;
  public mjSolverStat_ solver3197;
  public mjSolverStat_ solver3198;
  public mjSolverStat_ solver3199;
  public mjSolverStat_ solver3200;
  public mjSolverStat_ solver3201;
  public mjSolverStat_ solver3202;
  public mjSolverStat_ solver3203;
  public mjSolverStat_ solver3204;
  public mjSolverStat_ solver3205;
  public mjSolverStat_ solver3206;
  public mjSolverStat_ solver3207;
  public mjSolverStat_ solver3208;
  public mjSolverStat_ solver3209;
  public mjSolverStat_ solver3210;
  public mjSolverStat_ solver3211;
  public mjSolverStat_ solver3212;
  public mjSolverStat_ solver3213;
  public mjSolverStat_ solver3214;
  public mjSolverStat_ solver3215;
  public mjSolverStat_ solver3216;
  public mjSolverStat_ solver3217;
  public mjSolverStat_ solver3218;
  public mjSolverStat_ solver3219;
  public mjSolverStat_ solver3220;
  public mjSolverStat_ solver3221;
  public mjSolverStat_ solver3222;
  public mjSolverStat_ solver3223;
  public mjSolverStat_ solver3224;
  public mjSolverStat_ solver3225;
  public mjSolverStat_ solver3226;
  public mjSolverStat_ solver3227;
  public mjSolverStat_ solver3228;
  public mjSolverStat_ solver3229;
  public mjSolverStat_ solver3230;
  public mjSolverStat_ solver3231;
  public mjSolverStat_ solver3232;
  public mjSolverStat_ solver3233;
  public mjSolverStat_ solver3234;
  public mjSolverStat_ solver3235;
  public mjSolverStat_ solver3236;
  public mjSolverStat_ solver3237;
  public mjSolverStat_ solver3238;
  public mjSolverStat_ solver3239;
  public mjSolverStat_ solver3240;
  public mjSolverStat_ solver3241;
  public mjSolverStat_ solver3242;
  public mjSolverStat_ solver3243;
  public mjSolverStat_ solver3244;
  public mjSolverStat_ solver3245;
  public mjSolverStat_ solver3246;
  public mjSolverStat_ solver3247;
  public mjSolverStat_ solver3248;
  public mjSolverStat_ solver3249;
  public mjSolverStat_ solver3250;
  public mjSolverStat_ solver3251;
  public mjSolverStat_ solver3252;
  public mjSolverStat_ solver3253;
  public mjSolverStat_ solver3254;
  public mjSolverStat_ solver3255;
  public mjSolverStat_ solver3256;
  public mjSolverStat_ solver3257;
  public mjSolverStat_ solver3258;
  public mjSolverStat_ solver3259;
  public mjSolverStat_ solver3260;
  public mjSolverStat_ solver3261;
  public mjSolverStat_ solver3262;
  public mjSolverStat_ solver3263;
  public mjSolverStat_ solver3264;
  public mjSolverStat_ solver3265;
  public mjSolverStat_ solver3266;
  public mjSolverStat_ solver3267;
  public mjSolverStat_ solver3268;
  public mjSolverStat_ solver3269;
  public mjSolverStat_ solver3270;
  public mjSolverStat_ solver3271;
  public mjSolverStat_ solver3272;
  public mjSolverStat_ solver3273;
  public mjSolverStat_ solver3274;
  public mjSolverStat_ solver3275;
  public mjSolverStat_ solver3276;
  public mjSolverStat_ solver3277;
  public mjSolverStat_ solver3278;
  public mjSolverStat_ solver3279;
  public mjSolverStat_ solver3280;
  public mjSolverStat_ solver3281;
  public mjSolverStat_ solver3282;
  public mjSolverStat_ solver3283;
  public mjSolverStat_ solver3284;
  public mjSolverStat_ solver3285;
  public mjSolverStat_ solver3286;
  public mjSolverStat_ solver3287;
  public mjSolverStat_ solver3288;
  public mjSolverStat_ solver3289;
  public mjSolverStat_ solver3290;
  public mjSolverStat_ solver3291;
  public mjSolverStat_ solver3292;
  public mjSolverStat_ solver3293;
  public mjSolverStat_ solver3294;
  public mjSolverStat_ solver3295;
  public mjSolverStat_ solver3296;
  public mjSolverStat_ solver3297;
  public mjSolverStat_ solver3298;
  public mjSolverStat_ solver3299;
  public mjSolverStat_ solver3300;
  public mjSolverStat_ solver3301;
  public mjSolverStat_ solver3302;
  public mjSolverStat_ solver3303;
  public mjSolverStat_ solver3304;
  public mjSolverStat_ solver3305;
  public mjSolverStat_ solver3306;
  public mjSolverStat_ solver3307;
  public mjSolverStat_ solver3308;
  public mjSolverStat_ solver3309;
  public mjSolverStat_ solver3310;
  public mjSolverStat_ solver3311;
  public mjSolverStat_ solver3312;
  public mjSolverStat_ solver3313;
  public mjSolverStat_ solver3314;
  public mjSolverStat_ solver3315;
  public mjSolverStat_ solver3316;
  public mjSolverStat_ solver3317;
  public mjSolverStat_ solver3318;
  public mjSolverStat_ solver3319;
  public mjSolverStat_ solver3320;
  public mjSolverStat_ solver3321;
  public mjSolverStat_ solver3322;
  public mjSolverStat_ solver3323;
  public mjSolverStat_ solver3324;
  public mjSolverStat_ solver3325;
  public mjSolverStat_ solver3326;
  public mjSolverStat_ solver3327;
  public mjSolverStat_ solver3328;
  public mjSolverStat_ solver3329;
  public mjSolverStat_ solver3330;
  public mjSolverStat_ solver3331;
  public mjSolverStat_ solver3332;
  public mjSolverStat_ solver3333;
  public mjSolverStat_ solver3334;
  public mjSolverStat_ solver3335;
  public mjSolverStat_ solver3336;
  public mjSolverStat_ solver3337;
  public mjSolverStat_ solver3338;
  public mjSolverStat_ solver3339;
  public mjSolverStat_ solver3340;
  public mjSolverStat_ solver3341;
  public mjSolverStat_ solver3342;
  public mjSolverStat_ solver3343;
  public mjSolverStat_ solver3344;
  public mjSolverStat_ solver3345;
  public mjSolverStat_ solver3346;
  public mjSolverStat_ solver3347;
  public mjSolverStat_ solver3348;
  public mjSolverStat_ solver3349;
  public mjSolverStat_ solver3350;
  public mjSolverStat_ solver3351;
  public mjSolverStat_ solver3352;
  public mjSolverStat_ solver3353;
  public mjSolverStat_ solver3354;
  public mjSolverStat_ solver3355;
  public mjSolverStat_ solver3356;
  public mjSolverStat_ solver3357;
  public mjSolverStat_ solver3358;
  public mjSolverStat_ solver3359;
  public mjSolverStat_ solver3360;
  public mjSolverStat_ solver3361;
  public mjSolverStat_ solver3362;
  public mjSolverStat_ solver3363;
  public mjSolverStat_ solver3364;
  public mjSolverStat_ solver3365;
  public mjSolverStat_ solver3366;
  public mjSolverStat_ solver3367;
  public mjSolverStat_ solver3368;
  public mjSolverStat_ solver3369;
  public mjSolverStat_ solver3370;
  public mjSolverStat_ solver3371;
  public mjSolverStat_ solver3372;
  public mjSolverStat_ solver3373;
  public mjSolverStat_ solver3374;
  public mjSolverStat_ solver3375;
  public mjSolverStat_ solver3376;
  public mjSolverStat_ solver3377;
  public mjSolverStat_ solver3378;
  public mjSolverStat_ solver3379;
  public mjSolverStat_ solver3380;
  public mjSolverStat_ solver3381;
  public mjSolverStat_ solver3382;
  public mjSolverStat_ solver3383;
  public mjSolverStat_ solver3384;
  public mjSolverStat_ solver3385;
  public mjSolverStat_ solver3386;
  public mjSolverStat_ solver3387;
  public mjSolverStat_ solver3388;
  public mjSolverStat_ solver3389;
  public mjSolverStat_ solver3390;
  public mjSolverStat_ solver3391;
  public mjSolverStat_ solver3392;
  public mjSolverStat_ solver3393;
  public mjSolverStat_ solver3394;
  public mjSolverStat_ solver3395;
  public mjSolverStat_ solver3396;
  public mjSolverStat_ solver3397;
  public mjSolverStat_ solver3398;
  public mjSolverStat_ solver3399;
  public mjSolverStat_ solver3400;
  public mjSolverStat_ solver3401;
  public mjSolverStat_ solver3402;
  public mjSolverStat_ solver3403;
  public mjSolverStat_ solver3404;
  public mjSolverStat_ solver3405;
  public mjSolverStat_ solver3406;
  public mjSolverStat_ solver3407;
  public mjSolverStat_ solver3408;
  public mjSolverStat_ solver3409;
  public mjSolverStat_ solver3410;
  public mjSolverStat_ solver3411;
  public mjSolverStat_ solver3412;
  public mjSolverStat_ solver3413;
  public mjSolverStat_ solver3414;
  public mjSolverStat_ solver3415;
  public mjSolverStat_ solver3416;
  public mjSolverStat_ solver3417;
  public mjSolverStat_ solver3418;
  public mjSolverStat_ solver3419;
  public mjSolverStat_ solver3420;
  public mjSolverStat_ solver3421;
  public mjSolverStat_ solver3422;
  public mjSolverStat_ solver3423;
  public mjSolverStat_ solver3424;
  public mjSolverStat_ solver3425;
  public mjSolverStat_ solver3426;
  public mjSolverStat_ solver3427;
  public mjSolverStat_ solver3428;
  public mjSolverStat_ solver3429;
  public mjSolverStat_ solver3430;
  public mjSolverStat_ solver3431;
  public mjSolverStat_ solver3432;
  public mjSolverStat_ solver3433;
  public mjSolverStat_ solver3434;
  public mjSolverStat_ solver3435;
  public mjSolverStat_ solver3436;
  public mjSolverStat_ solver3437;
  public mjSolverStat_ solver3438;
  public mjSolverStat_ solver3439;
  public mjSolverStat_ solver3440;
  public mjSolverStat_ solver3441;
  public mjSolverStat_ solver3442;
  public mjSolverStat_ solver3443;
  public mjSolverStat_ solver3444;
  public mjSolverStat_ solver3445;
  public mjSolverStat_ solver3446;
  public mjSolverStat_ solver3447;
  public mjSolverStat_ solver3448;
  public mjSolverStat_ solver3449;
  public mjSolverStat_ solver3450;
  public mjSolverStat_ solver3451;
  public mjSolverStat_ solver3452;
  public mjSolverStat_ solver3453;
  public mjSolverStat_ solver3454;
  public mjSolverStat_ solver3455;
  public mjSolverStat_ solver3456;
  public mjSolverStat_ solver3457;
  public mjSolverStat_ solver3458;
  public mjSolverStat_ solver3459;
  public mjSolverStat_ solver3460;
  public mjSolverStat_ solver3461;
  public mjSolverStat_ solver3462;
  public mjSolverStat_ solver3463;
  public mjSolverStat_ solver3464;
  public mjSolverStat_ solver3465;
  public mjSolverStat_ solver3466;
  public mjSolverStat_ solver3467;
  public mjSolverStat_ solver3468;
  public mjSolverStat_ solver3469;
  public mjSolverStat_ solver3470;
  public mjSolverStat_ solver3471;
  public mjSolverStat_ solver3472;
  public mjSolverStat_ solver3473;
  public mjSolverStat_ solver3474;
  public mjSolverStat_ solver3475;
  public mjSolverStat_ solver3476;
  public mjSolverStat_ solver3477;
  public mjSolverStat_ solver3478;
  public mjSolverStat_ solver3479;
  public mjSolverStat_ solver3480;
  public mjSolverStat_ solver3481;
  public mjSolverStat_ solver3482;
  public mjSolverStat_ solver3483;
  public mjSolverStat_ solver3484;
  public mjSolverStat_ solver3485;
  public mjSolverStat_ solver3486;
  public mjSolverStat_ solver3487;
  public mjSolverStat_ solver3488;
  public mjSolverStat_ solver3489;
  public mjSolverStat_ solver3490;
  public mjSolverStat_ solver3491;
  public mjSolverStat_ solver3492;
  public mjSolverStat_ solver3493;
  public mjSolverStat_ solver3494;
  public mjSolverStat_ solver3495;
  public mjSolverStat_ solver3496;
  public mjSolverStat_ solver3497;
  public mjSolverStat_ solver3498;
  public mjSolverStat_ solver3499;
  public mjSolverStat_ solver3500;
  public mjSolverStat_ solver3501;
  public mjSolverStat_ solver3502;
  public mjSolverStat_ solver3503;
  public mjSolverStat_ solver3504;
  public mjSolverStat_ solver3505;
  public mjSolverStat_ solver3506;
  public mjSolverStat_ solver3507;
  public mjSolverStat_ solver3508;
  public mjSolverStat_ solver3509;
  public mjSolverStat_ solver3510;
  public mjSolverStat_ solver3511;
  public mjSolverStat_ solver3512;
  public mjSolverStat_ solver3513;
  public mjSolverStat_ solver3514;
  public mjSolverStat_ solver3515;
  public mjSolverStat_ solver3516;
  public mjSolverStat_ solver3517;
  public mjSolverStat_ solver3518;
  public mjSolverStat_ solver3519;
  public mjSolverStat_ solver3520;
  public mjSolverStat_ solver3521;
  public mjSolverStat_ solver3522;
  public mjSolverStat_ solver3523;
  public mjSolverStat_ solver3524;
  public mjSolverStat_ solver3525;
  public mjSolverStat_ solver3526;
  public mjSolverStat_ solver3527;
  public mjSolverStat_ solver3528;
  public mjSolverStat_ solver3529;
  public mjSolverStat_ solver3530;
  public mjSolverStat_ solver3531;
  public mjSolverStat_ solver3532;
  public mjSolverStat_ solver3533;
  public mjSolverStat_ solver3534;
  public mjSolverStat_ solver3535;
  public mjSolverStat_ solver3536;
  public mjSolverStat_ solver3537;
  public mjSolverStat_ solver3538;
  public mjSolverStat_ solver3539;
  public mjSolverStat_ solver3540;
  public mjSolverStat_ solver3541;
  public mjSolverStat_ solver3542;
  public mjSolverStat_ solver3543;
  public mjSolverStat_ solver3544;
  public mjSolverStat_ solver3545;
  public mjSolverStat_ solver3546;
  public mjSolverStat_ solver3547;
  public mjSolverStat_ solver3548;
  public mjSolverStat_ solver3549;
  public mjSolverStat_ solver3550;
  public mjSolverStat_ solver3551;
  public mjSolverStat_ solver3552;
  public mjSolverStat_ solver3553;
  public mjSolverStat_ solver3554;
  public mjSolverStat_ solver3555;
  public mjSolverStat_ solver3556;
  public mjSolverStat_ solver3557;
  public mjSolverStat_ solver3558;
  public mjSolverStat_ solver3559;
  public mjSolverStat_ solver3560;
  public mjSolverStat_ solver3561;
  public mjSolverStat_ solver3562;
  public mjSolverStat_ solver3563;
  public mjSolverStat_ solver3564;
  public mjSolverStat_ solver3565;
  public mjSolverStat_ solver3566;
  public mjSolverStat_ solver3567;
  public mjSolverStat_ solver3568;
  public mjSolverStat_ solver3569;
  public mjSolverStat_ solver3570;
  public mjSolverStat_ solver3571;
  public mjSolverStat_ solver3572;
  public mjSolverStat_ solver3573;
  public mjSolverStat_ solver3574;
  public mjSolverStat_ solver3575;
  public mjSolverStat_ solver3576;
  public mjSolverStat_ solver3577;
  public mjSolverStat_ solver3578;
  public mjSolverStat_ solver3579;
  public mjSolverStat_ solver3580;
  public mjSolverStat_ solver3581;
  public mjSolverStat_ solver3582;
  public mjSolverStat_ solver3583;
  public mjSolverStat_ solver3584;
  public mjSolverStat_ solver3585;
  public mjSolverStat_ solver3586;
  public mjSolverStat_ solver3587;
  public mjSolverStat_ solver3588;
  public mjSolverStat_ solver3589;
  public mjSolverStat_ solver3590;
  public mjSolverStat_ solver3591;
  public mjSolverStat_ solver3592;
  public mjSolverStat_ solver3593;
  public mjSolverStat_ solver3594;
  public mjSolverStat_ solver3595;
  public mjSolverStat_ solver3596;
  public mjSolverStat_ solver3597;
  public mjSolverStat_ solver3598;
  public mjSolverStat_ solver3599;
  public mjSolverStat_ solver3600;
  public mjSolverStat_ solver3601;
  public mjSolverStat_ solver3602;
  public mjSolverStat_ solver3603;
  public mjSolverStat_ solver3604;
  public mjSolverStat_ solver3605;
  public mjSolverStat_ solver3606;
  public mjSolverStat_ solver3607;
  public mjSolverStat_ solver3608;
  public mjSolverStat_ solver3609;
  public mjSolverStat_ solver3610;
  public mjSolverStat_ solver3611;
  public mjSolverStat_ solver3612;
  public mjSolverStat_ solver3613;
  public mjSolverStat_ solver3614;
  public mjSolverStat_ solver3615;
  public mjSolverStat_ solver3616;
  public mjSolverStat_ solver3617;
  public mjSolverStat_ solver3618;
  public mjSolverStat_ solver3619;
  public mjSolverStat_ solver3620;
  public mjSolverStat_ solver3621;
  public mjSolverStat_ solver3622;
  public mjSolverStat_ solver3623;
  public mjSolverStat_ solver3624;
  public mjSolverStat_ solver3625;
  public mjSolverStat_ solver3626;
  public mjSolverStat_ solver3627;
  public mjSolverStat_ solver3628;
  public mjSolverStat_ solver3629;
  public mjSolverStat_ solver3630;
  public mjSolverStat_ solver3631;
  public mjSolverStat_ solver3632;
  public mjSolverStat_ solver3633;
  public mjSolverStat_ solver3634;
  public mjSolverStat_ solver3635;
  public mjSolverStat_ solver3636;
  public mjSolverStat_ solver3637;
  public mjSolverStat_ solver3638;
  public mjSolverStat_ solver3639;
  public mjSolverStat_ solver3640;
  public mjSolverStat_ solver3641;
  public mjSolverStat_ solver3642;
  public mjSolverStat_ solver3643;
  public mjSolverStat_ solver3644;
  public mjSolverStat_ solver3645;
  public mjSolverStat_ solver3646;
  public mjSolverStat_ solver3647;
  public mjSolverStat_ solver3648;
  public mjSolverStat_ solver3649;
  public mjSolverStat_ solver3650;
  public mjSolverStat_ solver3651;
  public mjSolverStat_ solver3652;
  public mjSolverStat_ solver3653;
  public mjSolverStat_ solver3654;
  public mjSolverStat_ solver3655;
  public mjSolverStat_ solver3656;
  public mjSolverStat_ solver3657;
  public mjSolverStat_ solver3658;
  public mjSolverStat_ solver3659;
  public mjSolverStat_ solver3660;
  public mjSolverStat_ solver3661;
  public mjSolverStat_ solver3662;
  public mjSolverStat_ solver3663;
  public mjSolverStat_ solver3664;
  public mjSolverStat_ solver3665;
  public mjSolverStat_ solver3666;
  public mjSolverStat_ solver3667;
  public mjSolverStat_ solver3668;
  public mjSolverStat_ solver3669;
  public mjSolverStat_ solver3670;
  public mjSolverStat_ solver3671;
  public mjSolverStat_ solver3672;
  public mjSolverStat_ solver3673;
  public mjSolverStat_ solver3674;
  public mjSolverStat_ solver3675;
  public mjSolverStat_ solver3676;
  public mjSolverStat_ solver3677;
  public mjSolverStat_ solver3678;
  public mjSolverStat_ solver3679;
  public mjSolverStat_ solver3680;
  public mjSolverStat_ solver3681;
  public mjSolverStat_ solver3682;
  public mjSolverStat_ solver3683;
  public mjSolverStat_ solver3684;
  public mjSolverStat_ solver3685;
  public mjSolverStat_ solver3686;
  public mjSolverStat_ solver3687;
  public mjSolverStat_ solver3688;
  public mjSolverStat_ solver3689;
  public mjSolverStat_ solver3690;
  public mjSolverStat_ solver3691;
  public mjSolverStat_ solver3692;
  public mjSolverStat_ solver3693;
  public mjSolverStat_ solver3694;
  public mjSolverStat_ solver3695;
  public mjSolverStat_ solver3696;
  public mjSolverStat_ solver3697;
  public mjSolverStat_ solver3698;
  public mjSolverStat_ solver3699;
  public mjSolverStat_ solver3700;
  public mjSolverStat_ solver3701;
  public mjSolverStat_ solver3702;
  public mjSolverStat_ solver3703;
  public mjSolverStat_ solver3704;
  public mjSolverStat_ solver3705;
  public mjSolverStat_ solver3706;
  public mjSolverStat_ solver3707;
  public mjSolverStat_ solver3708;
  public mjSolverStat_ solver3709;
  public mjSolverStat_ solver3710;
  public mjSolverStat_ solver3711;
  public mjSolverStat_ solver3712;
  public mjSolverStat_ solver3713;
  public mjSolverStat_ solver3714;
  public mjSolverStat_ solver3715;
  public mjSolverStat_ solver3716;
  public mjSolverStat_ solver3717;
  public mjSolverStat_ solver3718;
  public mjSolverStat_ solver3719;
  public mjSolverStat_ solver3720;
  public mjSolverStat_ solver3721;
  public mjSolverStat_ solver3722;
  public mjSolverStat_ solver3723;
  public mjSolverStat_ solver3724;
  public mjSolverStat_ solver3725;
  public mjSolverStat_ solver3726;
  public mjSolverStat_ solver3727;
  public mjSolverStat_ solver3728;
  public mjSolverStat_ solver3729;
  public mjSolverStat_ solver3730;
  public mjSolverStat_ solver3731;
  public mjSolverStat_ solver3732;
  public mjSolverStat_ solver3733;
  public mjSolverStat_ solver3734;
  public mjSolverStat_ solver3735;
  public mjSolverStat_ solver3736;
  public mjSolverStat_ solver3737;
  public mjSolverStat_ solver3738;
  public mjSolverStat_ solver3739;
  public mjSolverStat_ solver3740;
  public mjSolverStat_ solver3741;
  public mjSolverStat_ solver3742;
  public mjSolverStat_ solver3743;
  public mjSolverStat_ solver3744;
  public mjSolverStat_ solver3745;
  public mjSolverStat_ solver3746;
  public mjSolverStat_ solver3747;
  public mjSolverStat_ solver3748;
  public mjSolverStat_ solver3749;
  public mjSolverStat_ solver3750;
  public mjSolverStat_ solver3751;
  public mjSolverStat_ solver3752;
  public mjSolverStat_ solver3753;
  public mjSolverStat_ solver3754;
  public mjSolverStat_ solver3755;
  public mjSolverStat_ solver3756;
  public mjSolverStat_ solver3757;
  public mjSolverStat_ solver3758;
  public mjSolverStat_ solver3759;
  public mjSolverStat_ solver3760;
  public mjSolverStat_ solver3761;
  public mjSolverStat_ solver3762;
  public mjSolverStat_ solver3763;
  public mjSolverStat_ solver3764;
  public mjSolverStat_ solver3765;
  public mjSolverStat_ solver3766;
  public mjSolverStat_ solver3767;
  public mjSolverStat_ solver3768;
  public mjSolverStat_ solver3769;
  public mjSolverStat_ solver3770;
  public mjSolverStat_ solver3771;
  public mjSolverStat_ solver3772;
  public mjSolverStat_ solver3773;
  public mjSolverStat_ solver3774;
  public mjSolverStat_ solver3775;
  public mjSolverStat_ solver3776;
  public mjSolverStat_ solver3777;
  public mjSolverStat_ solver3778;
  public mjSolverStat_ solver3779;
  public mjSolverStat_ solver3780;
  public mjSolverStat_ solver3781;
  public mjSolverStat_ solver3782;
  public mjSolverStat_ solver3783;
  public mjSolverStat_ solver3784;
  public mjSolverStat_ solver3785;
  public mjSolverStat_ solver3786;
  public mjSolverStat_ solver3787;
  public mjSolverStat_ solver3788;
  public mjSolverStat_ solver3789;
  public mjSolverStat_ solver3790;
  public mjSolverStat_ solver3791;
  public mjSolverStat_ solver3792;
  public mjSolverStat_ solver3793;
  public mjSolverStat_ solver3794;
  public mjSolverStat_ solver3795;
  public mjSolverStat_ solver3796;
  public mjSolverStat_ solver3797;
  public mjSolverStat_ solver3798;
  public mjSolverStat_ solver3799;
  public mjSolverStat_ solver3800;
  public mjSolverStat_ solver3801;
  public mjSolverStat_ solver3802;
  public mjSolverStat_ solver3803;
  public mjSolverStat_ solver3804;
  public mjSolverStat_ solver3805;
  public mjSolverStat_ solver3806;
  public mjSolverStat_ solver3807;
  public mjSolverStat_ solver3808;
  public mjSolverStat_ solver3809;
  public mjSolverStat_ solver3810;
  public mjSolverStat_ solver3811;
  public mjSolverStat_ solver3812;
  public mjSolverStat_ solver3813;
  public mjSolverStat_ solver3814;
  public mjSolverStat_ solver3815;
  public mjSolverStat_ solver3816;
  public mjSolverStat_ solver3817;
  public mjSolverStat_ solver3818;
  public mjSolverStat_ solver3819;
  public mjSolverStat_ solver3820;
  public mjSolverStat_ solver3821;
  public mjSolverStat_ solver3822;
  public mjSolverStat_ solver3823;
  public mjSolverStat_ solver3824;
  public mjSolverStat_ solver3825;
  public mjSolverStat_ solver3826;
  public mjSolverStat_ solver3827;
  public mjSolverStat_ solver3828;
  public mjSolverStat_ solver3829;
  public mjSolverStat_ solver3830;
  public mjSolverStat_ solver3831;
  public mjSolverStat_ solver3832;
  public mjSolverStat_ solver3833;
  public mjSolverStat_ solver3834;
  public mjSolverStat_ solver3835;
  public mjSolverStat_ solver3836;
  public mjSolverStat_ solver3837;
  public mjSolverStat_ solver3838;
  public mjSolverStat_ solver3839;
  public mjSolverStat_ solver3840;
  public mjSolverStat_ solver3841;
  public mjSolverStat_ solver3842;
  public mjSolverStat_ solver3843;
  public mjSolverStat_ solver3844;
  public mjSolverStat_ solver3845;
  public mjSolverStat_ solver3846;
  public mjSolverStat_ solver3847;
  public mjSolverStat_ solver3848;
  public mjSolverStat_ solver3849;
  public mjSolverStat_ solver3850;
  public mjSolverStat_ solver3851;
  public mjSolverStat_ solver3852;
  public mjSolverStat_ solver3853;
  public mjSolverStat_ solver3854;
  public mjSolverStat_ solver3855;
  public mjSolverStat_ solver3856;
  public mjSolverStat_ solver3857;
  public mjSolverStat_ solver3858;
  public mjSolverStat_ solver3859;
  public mjSolverStat_ solver3860;
  public mjSolverStat_ solver3861;
  public mjSolverStat_ solver3862;
  public mjSolverStat_ solver3863;
  public mjSolverStat_ solver3864;
  public mjSolverStat_ solver3865;
  public mjSolverStat_ solver3866;
  public mjSolverStat_ solver3867;
  public mjSolverStat_ solver3868;
  public mjSolverStat_ solver3869;
  public mjSolverStat_ solver3870;
  public mjSolverStat_ solver3871;
  public mjSolverStat_ solver3872;
  public mjSolverStat_ solver3873;
  public mjSolverStat_ solver3874;
  public mjSolverStat_ solver3875;
  public mjSolverStat_ solver3876;
  public mjSolverStat_ solver3877;
  public mjSolverStat_ solver3878;
  public mjSolverStat_ solver3879;
  public mjSolverStat_ solver3880;
  public mjSolverStat_ solver3881;
  public mjSolverStat_ solver3882;
  public mjSolverStat_ solver3883;
  public mjSolverStat_ solver3884;
  public mjSolverStat_ solver3885;
  public mjSolverStat_ solver3886;
  public mjSolverStat_ solver3887;
  public mjSolverStat_ solver3888;
  public mjSolverStat_ solver3889;
  public mjSolverStat_ solver3890;
  public mjSolverStat_ solver3891;
  public mjSolverStat_ solver3892;
  public mjSolverStat_ solver3893;
  public mjSolverStat_ solver3894;
  public mjSolverStat_ solver3895;
  public mjSolverStat_ solver3896;
  public mjSolverStat_ solver3897;
  public mjSolverStat_ solver3898;
  public mjSolverStat_ solver3899;
  public mjSolverStat_ solver3900;
  public mjSolverStat_ solver3901;
  public mjSolverStat_ solver3902;
  public mjSolverStat_ solver3903;
  public mjSolverStat_ solver3904;
  public mjSolverStat_ solver3905;
  public mjSolverStat_ solver3906;
  public mjSolverStat_ solver3907;
  public mjSolverStat_ solver3908;
  public mjSolverStat_ solver3909;
  public mjSolverStat_ solver3910;
  public mjSolverStat_ solver3911;
  public mjSolverStat_ solver3912;
  public mjSolverStat_ solver3913;
  public mjSolverStat_ solver3914;
  public mjSolverStat_ solver3915;
  public mjSolverStat_ solver3916;
  public mjSolverStat_ solver3917;
  public mjSolverStat_ solver3918;
  public mjSolverStat_ solver3919;
  public mjSolverStat_ solver3920;
  public mjSolverStat_ solver3921;
  public mjSolverStat_ solver3922;
  public mjSolverStat_ solver3923;
  public mjSolverStat_ solver3924;
  public mjSolverStat_ solver3925;
  public mjSolverStat_ solver3926;
  public mjSolverStat_ solver3927;
  public mjSolverStat_ solver3928;
  public mjSolverStat_ solver3929;
  public mjSolverStat_ solver3930;
  public mjSolverStat_ solver3931;
  public mjSolverStat_ solver3932;
  public mjSolverStat_ solver3933;
  public mjSolverStat_ solver3934;
  public mjSolverStat_ solver3935;
  public mjSolverStat_ solver3936;
  public mjSolverStat_ solver3937;
  public mjSolverStat_ solver3938;
  public mjSolverStat_ solver3939;
  public mjSolverStat_ solver3940;
  public mjSolverStat_ solver3941;
  public mjSolverStat_ solver3942;
  public mjSolverStat_ solver3943;
  public mjSolverStat_ solver3944;
  public mjSolverStat_ solver3945;
  public mjSolverStat_ solver3946;
  public mjSolverStat_ solver3947;
  public mjSolverStat_ solver3948;
  public mjSolverStat_ solver3949;
  public mjSolverStat_ solver3950;
  public mjSolverStat_ solver3951;
  public mjSolverStat_ solver3952;
  public mjSolverStat_ solver3953;
  public mjSolverStat_ solver3954;
  public mjSolverStat_ solver3955;
  public mjSolverStat_ solver3956;
  public mjSolverStat_ solver3957;
  public mjSolverStat_ solver3958;
  public mjSolverStat_ solver3959;
  public mjSolverStat_ solver3960;
  public mjSolverStat_ solver3961;
  public mjSolverStat_ solver3962;
  public mjSolverStat_ solver3963;
  public mjSolverStat_ solver3964;
  public mjSolverStat_ solver3965;
  public mjSolverStat_ solver3966;
  public mjSolverStat_ solver3967;
  public mjSolverStat_ solver3968;
  public mjSolverStat_ solver3969;
  public mjSolverStat_ solver3970;
  public mjSolverStat_ solver3971;
  public mjSolverStat_ solver3972;
  public mjSolverStat_ solver3973;
  public mjSolverStat_ solver3974;
  public mjSolverStat_ solver3975;
  public mjSolverStat_ solver3976;
  public mjSolverStat_ solver3977;
  public mjSolverStat_ solver3978;
  public mjSolverStat_ solver3979;
  public mjSolverStat_ solver3980;
  public mjSolverStat_ solver3981;
  public mjSolverStat_ solver3982;
  public mjSolverStat_ solver3983;
  public mjSolverStat_ solver3984;
  public mjSolverStat_ solver3985;
  public mjSolverStat_ solver3986;
  public mjSolverStat_ solver3987;
  public mjSolverStat_ solver3988;
  public mjSolverStat_ solver3989;
  public mjSolverStat_ solver3990;
  public mjSolverStat_ solver3991;
  public mjSolverStat_ solver3992;
  public mjSolverStat_ solver3993;
  public mjSolverStat_ solver3994;
  public mjSolverStat_ solver3995;
  public mjSolverStat_ solver3996;
  public mjSolverStat_ solver3997;
  public mjSolverStat_ solver3998;
  public mjSolverStat_ solver3999;
  public fixed int solver_niter[20];
  public fixed int solver_nnz[20];
  public fixed double solver_fwdinv[2];
  public mjWarningStat_ warning0;
  public mjWarningStat_ warning1;
  public mjWarningStat_ warning2;
  public mjWarningStat_ warning3;
  public mjWarningStat_ warning4;
  public mjWarningStat_ warning5;
  public mjWarningStat_ warning6;
  public mjWarningStat_ warning7;
  public mjTimerStat_ timer0;
  public mjTimerStat_ timer1;
  public mjTimerStat_ timer2;
  public mjTimerStat_ timer3;
  public mjTimerStat_ timer4;
  public mjTimerStat_ timer5;
  public mjTimerStat_ timer6;
  public mjTimerStat_ timer7;
  public mjTimerStat_ timer8;
  public mjTimerStat_ timer9;
  public mjTimerStat_ timer10;
  public mjTimerStat_ timer11;
  public mjTimerStat_ timer12;
  public mjTimerStat_ timer13;
  public mjTimerStat_ timer14;
  public int ncon;
  public int ne;
  public int nf;
  public int nl;
  public int nefc;
  public int nJ;
  public int nA;
  public int nisland;
  public int nidof;
  public double time;
  public fixed double energy[2];
  public void* buffer;
  public void* arena;
  public double* qpos;
  public double* qvel;
  public double* act;
  public double* qacc_warmstart;
  public double* plugin_state;
  public double* ctrl;
  public double* qfrc_applied;
  public double* xfrc_applied;
  public byte* eq_active;
  public double* mocap_pos;
  public double* mocap_quat;
  public double* qacc;
  public double* act_dot;
  public double* userdata;
  public double* sensordata;
  public int* plugin;
  public UIntPtr* plugin_data;
  public double* xpos;
  public double* xquat;
  public double* xmat;
  public double* xipos;
  public double* ximat;
  public double* xanchor;
  public double* xaxis;
  public double* geom_xpos;
  public double* geom_xmat;
  public double* site_xpos;
  public double* site_xmat;
  public double* cam_xpos;
  public double* cam_xmat;
  public double* light_xpos;
  public double* light_xdir;
  public double* subtree_com;
  public double* cdof;
  public double* cinert;
  public double* flexvert_xpos;
  public double* flexelem_aabb;
  public int* flexedge_J_rownnz;
  public int* flexedge_J_rowadr;
  public int* flexedge_J_colind;
  public double* flexedge_J;
  public double* flexedge_length;
  public int* ten_wrapadr;
  public int* ten_wrapnum;
  public int* ten_J_rownnz;
  public int* ten_J_rowadr;
  public int* ten_J_colind;
  public double* ten_J;
  public double* ten_length;
  public int* wrap_obj;
  public double* wrap_xpos;
  public double* actuator_length;
  public int* moment_rownnz;
  public int* moment_rowadr;
  public int* moment_colind;
  public double* actuator_moment;
  public double* crb;
  public double* qM;
  public double* M;
  public double* qLD;
  public double* qLDiagInv;
  public double* bvh_aabb_dyn;
  public byte* bvh_active;
  public double* flexedge_velocity;
  public double* ten_velocity;
  public double* actuator_velocity;
  public double* cvel;
  public double* cdof_dot;
  public double* qfrc_bias;
  public double* qfrc_spring;
  public double* qfrc_damper;
  public double* qfrc_gravcomp;
  public double* qfrc_fluid;
  public double* qfrc_passive;
  public double* subtree_linvel;
  public double* subtree_angmom;
  public double* qH;
  public double* qHDiagInv;
  public int* B_rownnz;
  public int* B_rowadr;
  public int* B_colind;
  public int* M_rownnz;
  public int* M_rowadr;
  public int* M_colind;
  public int* mapM2M;
  public int* D_rownnz;
  public int* D_rowadr;
  public int* D_diag;
  public int* D_colind;
  public int* mapM2D;
  public int* mapD2M;
  public double* qDeriv;
  public double* qLU;
  public double* actuator_force;
  public double* qfrc_actuator;
  public double* qfrc_smooth;
  public double* qacc_smooth;
  public double* qfrc_constraint;
  public double* qfrc_inverse;
  public double* cacc;
  public double* cfrc_int;
  public double* cfrc_ext;
  public mjContact_* contact;
  public int* efc_type;
  public int* efc_id;
  public int* efc_J_rownnz;
  public int* efc_J_rowadr;
  public int* efc_J_rowsuper;
  public int* efc_J_colind;
  public int* efc_JT_rownnz;
  public int* efc_JT_rowadr;
  public int* efc_JT_rowsuper;
  public int* efc_JT_colind;
  public double* efc_J;
  public double* efc_JT;
  public double* efc_pos;
  public double* efc_margin;
  public double* efc_frictionloss;
  public double* efc_diagApprox;
  public double* efc_KBIP;
  public double* efc_D;
  public double* efc_R;
  public int* tendon_efcadr;
  public int* dof_island;
  public int* island_nv;
  public int* island_idofadr;
  public int* island_dofadr;
  public int* map_dof2idof;
  public int* map_idof2dof;
  public double* ifrc_smooth;
  public double* iacc_smooth;
  public int* iM_rownnz;
  public int* iM_rowadr;
  public int* iM_colind;
  public double* iM;
  public double* iLD;
  public double* iLDiagInv;
  public double* iacc;
  public int* efc_island;
  public int* island_ne;
  public int* island_nf;
  public int* island_nefc;
  public int* island_iefcadr;
  public int* map_efc2iefc;
  public int* map_iefc2efc;
  public int* iefc_type;
  public int* iefc_id;
  public int* iefc_J_rownnz;
  public int* iefc_J_rowadr;
  public int* iefc_J_rowsuper;
  public int* iefc_J_colind;
  public int* iefc_JT_rownnz;
  public int* iefc_JT_rowadr;
  public int* iefc_JT_rowsuper;
  public int* iefc_JT_colind;
  public double* iefc_J;
  public double* iefc_JT;
  public double* iefc_frictionloss;
  public double* iefc_D;
  public double* iefc_R;
  public int* efc_AR_rownnz;
  public int* efc_AR_rowadr;
  public int* efc_AR_colind;
  public double* efc_AR;
  public double* efc_vel;
  public double* efc_aref;
  public double* efc_b;
  public double* iefc_aref;
  public int* iefc_state;
  public double* iefc_force;
  public int* efc_state;
  public double* efc_force;
  public double* ifrc_constraint;
  public UIntPtr threadpool;
  public UInt64 signature;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjLROpt_ {
  public int mode;
  public int useexisting;
  public int uselimit;
  public double accel;
  public double maxforce;
  public double timeconst;
  public double timestep;
  public double inttotal;
  public double interval;
  public double tolrange;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct _mjVFS
{
  public void* impl_;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjOption_ {
  public double timestep;
  public double apirate;
  public double impratio;
  public double tolerance;
  public double ls_tolerance;
  public double noslip_tolerance;
  public double ccd_tolerance;
  public fixed double gravity[3];
  public fixed double wind[3];
  public fixed double magnetic[3];
  public double density;
  public double viscosity;
  public double o_margin;
  public fixed double o_solref[2];
  public fixed double o_solimp[5];
  public fixed double o_friction[5];
  public int integrator;
  public int cone;
  public int jacobian;
  public int solver;
  public int iterations;
  public int ls_iterations;
  public int noslip_iterations;
  public int ccd_iterations;
  public int disableflags;
  public int enableflags;
  public int disableactuator;
  public int sdf_initpoints;
  public int sdf_iterations;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct global {
  public int orthographic;
  public float fovy;
  public float ipd;
  public float azimuth;
  public float elevation;
  public float linewidth;
  public float glow;
  public float realtime;
  public int offwidth;
  public int offheight;
  public int ellipsoidinertia;
  public int bvactive;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct quality {
  public int shadowsize;
  public int offsamples;
  public int numslices;
  public int numstacks;
  public int numquads;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct headlight {
  public fixed float ambient[3];
  public fixed float diffuse[3];
  public fixed float specular[3];
  public int active;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct map {
  public float stiffness;
  public float stiffnessrot;
  public float force;
  public float torque;
  public float alpha;
  public float fogstart;
  public float fogend;
  public float znear;
  public float zfar;
  public float haze;
  public float shadowclip;
  public float shadowscale;
  public float actuatortendon;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct scale {
  public float forcewidth;
  public float contactwidth;
  public float contactheight;
  public float connect;
  public float com;
  public float camera;
  public float light;
  public float selectpoint;
  public float jointlength;
  public float jointwidth;
  public float actuatorlength;
  public float actuatorwidth;
  public float framelength;
  public float framewidth;
  public float constraint;
  public float slidercrank;
  public float frustum;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct rgba {
  public fixed float fog[4];
  public fixed float haze[4];
  public fixed float force[4];
  public fixed float inertia[4];
  public fixed float joint[4];
  public fixed float actuator[4];
  public fixed float actuatornegative[4];
  public fixed float actuatorpositive[4];
  public fixed float com[4];
  public fixed float camera[4];
  public fixed float light[4];
  public fixed float selectpoint[4];
  public fixed float connect[4];
  public fixed float contactpoint[4];
  public fixed float contactforce[4];
  public fixed float contactfriction[4];
  public fixed float contacttorque[4];
  public fixed float contactgap[4];
  public fixed float rangefinder[4];
  public fixed float constraint[4];
  public fixed float slidercrank[4];
  public fixed float crankbroken[4];
  public fixed float frustum[4];
  public fixed float bv[4];
  public fixed float bvactive[4];
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjVisual_ {
  public global global;
  public quality quality;
  public headlight headlight;
  public map map;
  public scale scale;
  public rgba rgba;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjStatistic_ {
  public double meaninertia;
  public double meanmass;
  public double meansize;
  public double extent;
  public fixed double center[3];
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjModel_ {
  public int nq;
  public int nv;
  public int nu;
  public int na;
  public int nbody;
  public int nbvh;
  public int nbvhstatic;
  public int nbvhdynamic;
  public int njnt;
  public int ngeom;
  public int nsite;
  public int ncam;
  public int nlight;
  public int nflex;
  public int nflexnode;
  public int nflexvert;
  public int nflexedge;
  public int nflexelem;
  public int nflexelemdata;
  public int nflexelemedge;
  public int nflexshelldata;
  public int nflexevpair;
  public int nflextexcoord;
  public int nmesh;
  public int nmeshvert;
  public int nmeshnormal;
  public int nmeshtexcoord;
  public int nmeshface;
  public int nmeshgraph;
  public int nmeshpoly;
  public int nmeshpolyvert;
  public int nmeshpolymap;
  public int nskin;
  public int nskinvert;
  public int nskintexvert;
  public int nskinface;
  public int nskinbone;
  public int nskinbonevert;
  public int nhfield;
  public int nhfielddata;
  public int ntex;
  public int ntexdata;
  public int nmat;
  public int npair;
  public int nexclude;
  public int neq;
  public int ntendon;
  public int nwrap;
  public int nsensor;
  public int nnumeric;
  public int nnumericdata;
  public int ntext;
  public int ntextdata;
  public int ntuple;
  public int ntupledata;
  public int nkey;
  public int nmocap;
  public int nplugin;
  public int npluginattr;
  public int nuser_body;
  public int nuser_jnt;
  public int nuser_geom;
  public int nuser_site;
  public int nuser_cam;
  public int nuser_tendon;
  public int nuser_actuator;
  public int nuser_sensor;
  public int nnames;
  public int npaths;
  public int nnames_map;
  public int nM;
  public int nB;
  public int nC;
  public int nD;
  public int nJmom;
  public int ntree;
  public int ngravcomp;
  public int nemax;
  public int njmax;
  public int nconmax;
  public int nuserdata;
  public int nsensordata;
  public int npluginstate;
  public UIntPtr narena;
  public UIntPtr nbuffer;
  public mjOption_ opt;
  public mjVisual_ vis;
  public mjStatistic_ stat;
  public void* buffer;
  public double* qpos0;
  public double* qpos_spring;
  public int* body_parentid;
  public int* body_rootid;
  public int* body_weldid;
  public int* body_mocapid;
  public int* body_jntnum;
  public int* body_jntadr;
  public int* body_dofnum;
  public int* body_dofadr;
  public int* body_treeid;
  public int* body_geomnum;
  public int* body_geomadr;
  public byte* body_simple;
  public byte* body_sameframe;
  public double* body_pos;
  public double* body_quat;
  public double* body_ipos;
  public double* body_iquat;
  public double* body_mass;
  public double* body_subtreemass;
  public double* body_inertia;
  public double* body_invweight0;
  public double* body_gravcomp;
  public double* body_margin;
  public double* body_user;
  public int* body_plugin;
  public int* body_contype;
  public int* body_conaffinity;
  public int* body_bvhadr;
  public int* body_bvhnum;
  public int* bvh_depth;
  public int* bvh_child;
  public int* bvh_nodeid;
  public double* bvh_aabb;
  public int* jnt_type;
  public int* jnt_qposadr;
  public int* jnt_dofadr;
  public int* jnt_bodyid;
  public int* jnt_group;
  public byte* jnt_limited;
  public byte* jnt_actfrclimited;
  public byte* jnt_actgravcomp;
  public double* jnt_solref;
  public double* jnt_solimp;
  public double* jnt_pos;
  public double* jnt_axis;
  public double* jnt_stiffness;
  public double* jnt_range;
  public double* jnt_actfrcrange;
  public double* jnt_margin;
  public double* jnt_user;
  public int* dof_bodyid;
  public int* dof_jntid;
  public int* dof_parentid;
  public int* dof_treeid;
  public int* dof_Madr;
  public int* dof_simplenum;
  public double* dof_solref;
  public double* dof_solimp;
  public double* dof_frictionloss;
  public double* dof_armature;
  public double* dof_damping;
  public double* dof_invweight0;
  public double* dof_M0;
  public int* geom_type;
  public int* geom_contype;
  public int* geom_conaffinity;
  public int* geom_condim;
  public int* geom_bodyid;
  public int* geom_dataid;
  public int* geom_matid;
  public int* geom_group;
  public int* geom_priority;
  public int* geom_plugin;
  public byte* geom_sameframe;
  public double* geom_solmix;
  public double* geom_solref;
  public double* geom_solimp;
  public double* geom_size;
  public double* geom_aabb;
  public double* geom_rbound;
  public double* geom_pos;
  public double* geom_quat;
  public double* geom_friction;
  public double* geom_margin;
  public double* geom_gap;
  public double* geom_fluid;
  public double* geom_user;
  public float* geom_rgba;
  public int* site_type;
  public int* site_bodyid;
  public int* site_matid;
  public int* site_group;
  public byte* site_sameframe;
  public double* site_size;
  public double* site_pos;
  public double* site_quat;
  public double* site_user;
  public float* site_rgba;
  public int* cam_mode;
  public int* cam_bodyid;
  public int* cam_targetbodyid;
  public double* cam_pos;
  public double* cam_quat;
  public double* cam_poscom0;
  public double* cam_pos0;
  public double* cam_mat0;
  public int* cam_orthographic;
  public double* cam_fovy;
  public double* cam_ipd;
  public int* cam_resolution;
  public float* cam_sensorsize;
  public float* cam_intrinsic;
  public double* cam_user;
  public int* light_mode;
  public int* light_bodyid;
  public int* light_targetbodyid;
  public int* light_type;
  public int* light_texid;
  public byte* light_castshadow;
  public float* light_bulbradius;
  public float* light_intensity;
  public float* light_range;
  public byte* light_active;
  public double* light_pos;
  public double* light_dir;
  public double* light_poscom0;
  public double* light_pos0;
  public double* light_dir0;
  public float* light_attenuation;
  public float* light_cutoff;
  public float* light_exponent;
  public float* light_ambient;
  public float* light_diffuse;
  public float* light_specular;
  public int* flex_contype;
  public int* flex_conaffinity;
  public int* flex_condim;
  public int* flex_priority;
  public double* flex_solmix;
  public double* flex_solref;
  public double* flex_solimp;
  public double* flex_friction;
  public double* flex_margin;
  public double* flex_gap;
  public byte* flex_internal;
  public int* flex_selfcollide;
  public int* flex_activelayers;
  public int* flex_dim;
  public int* flex_matid;
  public int* flex_group;
  public int* flex_interp;
  public int* flex_nodeadr;
  public int* flex_nodenum;
  public int* flex_vertadr;
  public int* flex_vertnum;
  public int* flex_edgeadr;
  public int* flex_edgenum;
  public int* flex_elemadr;
  public int* flex_elemnum;
  public int* flex_elemdataadr;
  public int* flex_elemedgeadr;
  public int* flex_shellnum;
  public int* flex_shelldataadr;
  public int* flex_evpairadr;
  public int* flex_evpairnum;
  public int* flex_texcoordadr;
  public int* flex_nodebodyid;
  public int* flex_vertbodyid;
  public int* flex_edge;
  public int* flex_edgeflap;
  public int* flex_elem;
  public int* flex_elemtexcoord;
  public int* flex_elemedge;
  public int* flex_elemlayer;
  public int* flex_shell;
  public int* flex_evpair;
  public double* flex_vert;
  public double* flex_vert0;
  public double* flex_node;
  public double* flex_node0;
  public double* flexedge_length0;
  public double* flexedge_invweight0;
  public double* flex_radius;
  public double* flex_stiffness;
  public double* flex_bending;
  public double* flex_damping;
  public double* flex_edgestiffness;
  public double* flex_edgedamping;
  public byte* flex_edgeequality;
  public byte* flex_rigid;
  public byte* flexedge_rigid;
  public byte* flex_centered;
  public byte* flex_flatskin;
  public int* flex_bvhadr;
  public int* flex_bvhnum;
  public float* flex_rgba;
  public float* flex_texcoord;
  public int* mesh_vertadr;
  public int* mesh_vertnum;
  public int* mesh_faceadr;
  public int* mesh_facenum;
  public int* mesh_bvhadr;
  public int* mesh_bvhnum;
  public int* mesh_normaladr;
  public int* mesh_normalnum;
  public int* mesh_texcoordadr;
  public int* mesh_texcoordnum;
  public int* mesh_graphadr;
  public float* mesh_vert;
  public float* mesh_normal;
  public float* mesh_texcoord;
  public int* mesh_face;
  public int* mesh_facenormal;
  public int* mesh_facetexcoord;
  public int* mesh_graph;
  public double* mesh_scale;
  public double* mesh_pos;
  public double* mesh_quat;
  public int* mesh_pathadr;
  public int* mesh_polynum;
  public int* mesh_polyadr;
  public double* mesh_polynormal;
  public int* mesh_polyvertadr;
  public int* mesh_polyvertnum;
  public int* mesh_polyvert;
  public int* mesh_polymapadr;
  public int* mesh_polymapnum;
  public int* mesh_polymap;
  public int* skin_matid;
  public int* skin_group;
  public float* skin_rgba;
  public float* skin_inflate;
  public int* skin_vertadr;
  public int* skin_vertnum;
  public int* skin_texcoordadr;
  public int* skin_faceadr;
  public int* skin_facenum;
  public int* skin_boneadr;
  public int* skin_bonenum;
  public float* skin_vert;
  public float* skin_texcoord;
  public int* skin_face;
  public int* skin_bonevertadr;
  public int* skin_bonevertnum;
  public float* skin_bonebindpos;
  public float* skin_bonebindquat;
  public int* skin_bonebodyid;
  public int* skin_bonevertid;
  public float* skin_bonevertweight;
  public int* skin_pathadr;
  public double* hfield_size;
  public int* hfield_nrow;
  public int* hfield_ncol;
  public int* hfield_adr;
  public float* hfield_data;
  public int* hfield_pathadr;
  public int* tex_type;
  public int* tex_colorspace;
  public int* tex_height;
  public int* tex_width;
  public int* tex_nchannel;
  public int* tex_adr;
  public byte* tex_data;
  public int* tex_pathadr;
  public int* mat_texid;
  public byte* mat_texuniform;
  public float* mat_texrepeat;
  public float* mat_emission;
  public float* mat_specular;
  public float* mat_shininess;
  public float* mat_reflectance;
  public float* mat_metallic;
  public float* mat_roughness;
  public float* mat_rgba;
  public int* pair_dim;
  public int* pair_geom1;
  public int* pair_geom2;
  public int* pair_signature;
  public double* pair_solref;
  public double* pair_solreffriction;
  public double* pair_solimp;
  public double* pair_margin;
  public double* pair_gap;
  public double* pair_friction;
  public int* exclude_signature;
  public int* eq_type;
  public int* eq_obj1id;
  public int* eq_obj2id;
  public int* eq_objtype;
  public byte* eq_active0;
  public double* eq_solref;
  public double* eq_solimp;
  public double* eq_data;
  public int* tendon_adr;
  public int* tendon_num;
  public int* tendon_matid;
  public int* tendon_group;
  public byte* tendon_limited;
  public byte* tendon_actfrclimited;
  public double* tendon_width;
  public double* tendon_solref_lim;
  public double* tendon_solimp_lim;
  public double* tendon_solref_fri;
  public double* tendon_solimp_fri;
  public double* tendon_range;
  public double* tendon_actfrcrange;
  public double* tendon_margin;
  public double* tendon_stiffness;
  public double* tendon_damping;
  public double* tendon_armature;
  public double* tendon_frictionloss;
  public double* tendon_lengthspring;
  public double* tendon_length0;
  public double* tendon_invweight0;
  public double* tendon_user;
  public float* tendon_rgba;
  public int* wrap_type;
  public int* wrap_objid;
  public double* wrap_prm;
  public int* actuator_trntype;
  public int* actuator_dyntype;
  public int* actuator_gaintype;
  public int* actuator_biastype;
  public int* actuator_trnid;
  public int* actuator_actadr;
  public int* actuator_actnum;
  public int* actuator_group;
  public byte* actuator_ctrllimited;
  public byte* actuator_forcelimited;
  public byte* actuator_actlimited;
  public double* actuator_dynprm;
  public double* actuator_gainprm;
  public double* actuator_biasprm;
  public byte* actuator_actearly;
  public double* actuator_ctrlrange;
  public double* actuator_forcerange;
  public double* actuator_actrange;
  public double* actuator_gear;
  public double* actuator_cranklength;
  public double* actuator_acc0;
  public double* actuator_length0;
  public double* actuator_lengthrange;
  public double* actuator_user;
  public int* actuator_plugin;
  public int* sensor_type;
  public int* sensor_datatype;
  public int* sensor_needstage;
  public int* sensor_objtype;
  public int* sensor_objid;
  public int* sensor_reftype;
  public int* sensor_refid;
  public int* sensor_dim;
  public int* sensor_adr;
  public double* sensor_cutoff;
  public double* sensor_noise;
  public double* sensor_user;
  public int* sensor_plugin;
  public int* plugin;
  public int* plugin_stateadr;
  public int* plugin_statenum;
  public char* plugin_attr;
  public int* plugin_attradr;
  public int* numeric_adr;
  public int* numeric_size;
  public double* numeric_data;
  public int* text_adr;
  public int* text_size;
  public char* text_data;
  public int* tuple_adr;
  public int* tuple_size;
  public int* tuple_objtype;
  public int* tuple_objid;
  public double* tuple_objprm;
  public double* key_time;
  public double* key_qpos;
  public double* key_qvel;
  public double* key_act;
  public double* key_mpos;
  public double* key_mquat;
  public double* key_ctrl;
  public int* name_bodyadr;
  public int* name_jntadr;
  public int* name_geomadr;
  public int* name_siteadr;
  public int* name_camadr;
  public int* name_lightadr;
  public int* name_flexadr;
  public int* name_meshadr;
  public int* name_skinadr;
  public int* name_hfieldadr;
  public int* name_texadr;
  public int* name_matadr;
  public int* name_pairadr;
  public int* name_excludeadr;
  public int* name_eqadr;
  public int* name_tendonadr;
  public int* name_actuatoradr;
  public int* name_sensoradr;
  public int* name_numericadr;
  public int* name_textadr;
  public int* name_tupleadr;
  public int* name_keyadr;
  public int* name_pluginadr;
  public char* names;
  public int* names_map;
  public char* paths;
  public UInt64 signature;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjrRect_ {
  public int left;
  public int bottom;
  public int width;
  public int height;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjrContext_ {
  public float lineWidth;
  public float shadowClip;
  public float shadowScale;
  public float fogStart;
  public float fogEnd;
  public fixed float fogRGBA[4];
  public int shadowSize;
  public int offWidth;
  public int offHeight;
  public int offSamples;
  public int fontScale;
  public fixed int auxWidth[10];
  public fixed int auxHeight[10];
  public fixed int auxSamples[10];
  public uint offFBO;
  public uint offFBO_r;
  public uint offColor;
  public uint offColor_r;
  public uint offDepthStencil;
  public uint offDepthStencil_r;
  public uint shadowFBO;
  public uint shadowTex;
  public fixed uint auxFBO[10];
  public fixed uint auxFBO_r[10];
  public fixed uint auxColor[10];
  public fixed uint auxColor_r[10];
  public fixed int mat_texid[10000];
  public fixed int mat_texuniform[1000];
  public fixed float mat_texrepeat[2000];
  public int ntexture;
  public fixed int textureType[1000];
  public fixed uint texture[1000];
  public uint basePlane;
  public uint baseMesh;
  public uint baseHField;
  public uint baseBuiltin;
  public uint baseFontNormal;
  public uint baseFontShadow;
  public uint baseFontBig;
  public int rangePlane;
  public int rangeMesh;
  public int rangeHField;
  public int rangeBuiltin;
  public int rangeFont;
  public int nskin;
  public uint* skinvertVBO;
  public uint* skinnormalVBO;
  public uint* skintexcoordVBO;
  public uint* skinfaceVBO;
  public fixed int charWidth[127];
  public fixed int charWidthBig[127];
  public int charHeight;
  public int charHeightBig;
  public int glInitialized;
  public int windowAvailable;
  public int windowSamples;
  public int windowStereo;
  public int windowDoublebuffer;
  public int currentBuffer;
  public int readPixelFormat;
  public int readDepthMap;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjsCompiler_ {
  public byte autolimits;
  public double boundmass;
  public double boundinertia;
  public double settotalmass;
  public byte balanceinertia;
  public byte fitaabb;
  public byte degree;
  public fixed sbyte eulerseq[3];
  public byte discardvisual;
  public byte usethread;
  public byte fusestatic;
  public int inertiafromgeom;
  public fixed int inertiagrouprange[2];
  public byte saveinertial;
  public int alignfree;
  public mjLROpt_ LRopt;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiState_ {
  public int nrect;
  public mjrRect_ rect0;
  public mjrRect_ rect1;
  public mjrRect_ rect2;
  public mjrRect_ rect3;
  public mjrRect_ rect4;
  public mjrRect_ rect5;
  public mjrRect_ rect6;
  public mjrRect_ rect7;
  public mjrRect_ rect8;
  public mjrRect_ rect9;
  public mjrRect_ rect10;
  public mjrRect_ rect11;
  public mjrRect_ rect12;
  public mjrRect_ rect13;
  public mjrRect_ rect14;
  public mjrRect_ rect15;
  public mjrRect_ rect16;
  public mjrRect_ rect17;
  public mjrRect_ rect18;
  public mjrRect_ rect19;
  public mjrRect_ rect20;
  public mjrRect_ rect21;
  public mjrRect_ rect22;
  public mjrRect_ rect23;
  public mjrRect_ rect24;
  public void* userdata;
  public int type;
  public int left;
  public int right;
  public int middle;
  public int doubleclick;
  public int button;
  public double buttontime;
  public double x;
  public double y;
  public double dx;
  public double dy;
  public double sx;
  public double sy;
  public int control;
  public int shift;
  public int alt;
  public int key;
  public double keytime;
  public int mouserect;
  public int dragrect;
  public int dragbutton;
  public int dropcount;
  public char** droppaths;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiThemeSpacing_ {
  public int total;
  public int scroll;
  public int label;
  public int section;
  public int cornersect;
  public int cornersep;
  public int itemside;
  public int itemmid;
  public int itemver;
  public int texthor;
  public int textver;
  public int linescroll;
  public int samples;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiThemeColor_ {
  public fixed float master[3];
  public fixed float thumb[3];
  public fixed float secttitle[3];
  public fixed float secttitle2[3];
  public fixed float secttitleuncheck[3];
  public fixed float secttitleuncheck2[3];
  public fixed float secttitlecheck[3];
  public fixed float secttitlecheck2[3];
  public fixed float sectfont[3];
  public fixed float sectsymbol[3];
  public fixed float sectpane[3];
  public fixed float separator[3];
  public fixed float separator2[3];
  public fixed float shortcut[3];
  public fixed float fontactive[3];
  public fixed float fontinactive[3];
  public fixed float decorinactive[3];
  public fixed float decorinactive2[3];
  public fixed float button[3];
  public fixed float check[3];
  public fixed float radio[3];
  public fixed float select[3];
  public fixed float select2[3];
  public fixed float slider[3];
  public fixed float slider2[3];
  public fixed float edit[3];
  public fixed float edit2[3];
  public fixed float cursor[3];
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiItemSingle_ {
  public int modifier;
  public int shortcut;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiItemMulti_ {
  public int nelem;
  public fixed sbyte name[35 * 40];
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiItemSlider_ {
  public fixed double range[2];
  public double divisions;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiItemEdit_ {
  public int nelem;
  public fixed double range[7 * 2];
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiSection_ {
  public fixed sbyte name[40];
  public int state;
  public int modifier;
  public int shortcut;
  public int checkbox;
  public int nitem;
  public mjrRect_ rtitle;
  public mjrRect_ rcontent;
  public int lastclick;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjUI_ {
  public mjuiThemeSpacing_ spacing;
  public mjuiThemeColor_ color;
  public mjfItemEnable predicate;
  public void* userdata;
  public int rectid;
  public int auxid;
  public int radiocol;
  public int width;
  public int height;
  public int maxheight;
  public int scroll;
  public int mousesect;
  public int mouseitem;
  public int mousehelp;
  public int mouseclicks;
  public int mousesectcheck;
  public int editsect;
  public int edititem;
  public int editcursor;
  public int editscroll;
  public fixed sbyte edittext[300];
  public mjuiItem_* editchanged;
  public int nsect;
  public mjuiSection_ sect0;
  public mjuiSection_ sect1;
  public mjuiSection_ sect2;
  public mjuiSection_ sect3;
  public mjuiSection_ sect4;
  public mjuiSection_ sect5;
  public mjuiSection_ sect6;
  public mjuiSection_ sect7;
  public mjuiSection_ sect8;
  public mjuiSection_ sect9;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiDef_ {
  public int type;
  public fixed sbyte name[40];
  public int state;
  public void* pdata;
  public fixed sbyte other[300];
  public int otherint;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvPerturb_ {
  public int select;
  public int flexselect;
  public int skinselect;
  public int active;
  public int active2;
  public fixed double refpos[3];
  public fixed double refquat[4];
  public fixed double refselpos[3];
  public fixed double localpos[3];
  public double localmass;
  public double scale;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvCamera_ {
  public int type;
  public int fixedcamid;
  public int trackbodyid;
  public fixed double lookat[3];
  public double distance;
  public double azimuth;
  public double elevation;
  public int orthographic;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvGLCamera_ {
  public fixed float pos[3];
  public fixed float forward[3];
  public fixed float up[3];
  public float frustum_center;
  public float frustum_width;
  public float frustum_bottom;
  public float frustum_top;
  public float frustum_near;
  public float frustum_far;
  public int orthographic;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvGeom_ {
  public int type;
  public int dataid;
  public int objtype;
  public int objid;
  public int category;
  public int matid;
  public int texcoord;
  public int segid;
  public fixed float size[3];
  public fixed float pos[3];
  public fixed float mat[9];
  public fixed float rgba[4];
  public float emission;
  public float specular;
  public float shininess;
  public float reflectance;
  public fixed sbyte label[100];
  public float camdist;
  public float modelrbound;
  public byte transparent;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvLight_ {
  public fixed float pos[3];
  public fixed float dir[3];
  public int type;
  public int texid;
  public fixed float attenuation[3];
  public float cutoff;
  public float exponent;
  public fixed float ambient[3];
  public fixed float diffuse[3];
  public fixed float specular[3];
  public byte headlight;
  public byte castshadow;
  public float bulbradius;
  public float intensity;
  public float range;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvOption_ {
  public int label;
  public int frame;
  public fixed byte geomgroup[6];
  public fixed byte sitegroup[6];
  public fixed byte jointgroup[6];
  public fixed byte tendongroup[6];
  public fixed byte actuatorgroup[6];
  public fixed byte flexgroup[6];
  public fixed byte skingroup[6];
  public fixed byte flags[32];
  public int bvh_depth;
  public int flex_layer;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvScene_ {
  public int maxgeom;
  public int ngeom;
  public mjvGeom_* geoms;
  public int* geomorder;
  public int nflex;
  public int* flexedgeadr;
  public int* flexedgenum;
  public int* flexvertadr;
  public int* flexvertnum;
  public int* flexfaceadr;
  public int* flexfacenum;
  public int* flexfaceused;
  public int* flexedge;
  public float* flexvert;
  public float* flexface;
  public float* flexnormal;
  public float* flextexcoord;
  public byte flexvertopt;
  public byte flexedgeopt;
  public byte flexfaceopt;
  public byte flexskinopt;
  public int nskin;
  public int* skinfacenum;
  public int* skinvertadr;
  public int* skinvertnum;
  public float* skinvert;
  public float* skinnormal;
  public int nlight;
  public mjvLight_ lights0;
  public mjvLight_ lights1;
  public mjvLight_ lights2;
  public mjvLight_ lights3;
  public mjvLight_ lights4;
  public mjvLight_ lights5;
  public mjvLight_ lights6;
  public mjvLight_ lights7;
  public mjvLight_ lights8;
  public mjvLight_ lights9;
  public mjvLight_ lights10;
  public mjvLight_ lights11;
  public mjvLight_ lights12;
  public mjvLight_ lights13;
  public mjvLight_ lights14;
  public mjvLight_ lights15;
  public mjvLight_ lights16;
  public mjvLight_ lights17;
  public mjvLight_ lights18;
  public mjvLight_ lights19;
  public mjvLight_ lights20;
  public mjvLight_ lights21;
  public mjvLight_ lights22;
  public mjvLight_ lights23;
  public mjvLight_ lights24;
  public mjvLight_ lights25;
  public mjvLight_ lights26;
  public mjvLight_ lights27;
  public mjvLight_ lights28;
  public mjvLight_ lights29;
  public mjvLight_ lights30;
  public mjvLight_ lights31;
  public mjvLight_ lights32;
  public mjvLight_ lights33;
  public mjvLight_ lights34;
  public mjvLight_ lights35;
  public mjvLight_ lights36;
  public mjvLight_ lights37;
  public mjvLight_ lights38;
  public mjvLight_ lights39;
  public mjvLight_ lights40;
  public mjvLight_ lights41;
  public mjvLight_ lights42;
  public mjvLight_ lights43;
  public mjvLight_ lights44;
  public mjvLight_ lights45;
  public mjvLight_ lights46;
  public mjvLight_ lights47;
  public mjvLight_ lights48;
  public mjvLight_ lights49;
  public mjvLight_ lights50;
  public mjvLight_ lights51;
  public mjvLight_ lights52;
  public mjvLight_ lights53;
  public mjvLight_ lights54;
  public mjvLight_ lights55;
  public mjvLight_ lights56;
  public mjvLight_ lights57;
  public mjvLight_ lights58;
  public mjvLight_ lights59;
  public mjvLight_ lights60;
  public mjvLight_ lights61;
  public mjvLight_ lights62;
  public mjvLight_ lights63;
  public mjvLight_ lights64;
  public mjvLight_ lights65;
  public mjvLight_ lights66;
  public mjvLight_ lights67;
  public mjvLight_ lights68;
  public mjvLight_ lights69;
  public mjvLight_ lights70;
  public mjvLight_ lights71;
  public mjvLight_ lights72;
  public mjvLight_ lights73;
  public mjvLight_ lights74;
  public mjvLight_ lights75;
  public mjvLight_ lights76;
  public mjvLight_ lights77;
  public mjvLight_ lights78;
  public mjvLight_ lights79;
  public mjvLight_ lights80;
  public mjvLight_ lights81;
  public mjvLight_ lights82;
  public mjvLight_ lights83;
  public mjvLight_ lights84;
  public mjvLight_ lights85;
  public mjvLight_ lights86;
  public mjvLight_ lights87;
  public mjvLight_ lights88;
  public mjvLight_ lights89;
  public mjvLight_ lights90;
  public mjvLight_ lights91;
  public mjvLight_ lights92;
  public mjvLight_ lights93;
  public mjvLight_ lights94;
  public mjvLight_ lights95;
  public mjvLight_ lights96;
  public mjvLight_ lights97;
  public mjvLight_ lights98;
  public mjvLight_ lights99;
  public mjvGLCamera_ camera0;
  public mjvGLCamera_ camera1;
  public byte enabletransform;
  public fixed float translate[3];
  public fixed float rotate[4];
  public float scale;
  public int stereo;
  public fixed byte flags[10];
  public int framewidth;
  public fixed float framergb[3];
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvFigure_ {
  public int flg_legend;
  public fixed int flg_ticklabel[2];
  public int flg_extend;
  public int flg_barplot;
  public int flg_selection;
  public int flg_symmetric;
  public float linewidth;
  public float gridwidth;
  public fixed int gridsize[2];
  public fixed float gridrgb[3];
  public fixed float figurergba[4];
  public fixed float panergba[4];
  public fixed float legendrgba[4];
  public fixed float textrgb[3];
  public fixed float linergb[100 * 3];
  public fixed float range[2 * 2];
  public fixed sbyte xformat[20];
  public fixed sbyte yformat[20];
  public fixed sbyte minwidth[20];
  public fixed sbyte title[1000];
  public fixed sbyte xlabel[100];
  public fixed sbyte linename[100 * 100];
  public int legendoffset;
  public int subplot;
  public fixed int highlight[2];
  public int highlightid;
  public float selection;
  public fixed int linepnt[100];
  public fixed float linedata[100 * 2000];
  public fixed int xaxispixel[2];
  public fixed int yaxispixel[2];
  public fixed float xaxisdata[2];
  public fixed float yaxisdata[2];
}public struct mjuiItem_ {}public struct mjfItemEnable {}

// ----------------------------Function declarations----------------------------

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_defaultVFS(void* vfs);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_addFileVFS(void* vfs, [MarshalAs(UnmanagedType.LPStr)]string directory, [MarshalAs(UnmanagedType.LPStr)]string filename);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_addBufferVFS(void* vfs, [MarshalAs(UnmanagedType.LPStr)]string name, void* buffer, int nbuffer);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_deleteFileVFS(void* vfs, [MarshalAs(UnmanagedType.LPStr)]string filename);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_deleteVFS(void* vfs);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjModel_* mj_loadXML([MarshalAs(UnmanagedType.LPStr)]string filename, void* vfs, StringBuilder error, int error_sz);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_saveLastXML([MarshalAs(UnmanagedType.LPStr)]string filename, mjModel_* m, StringBuilder error, int error_sz);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_freeLastXML();

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_step(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_step1(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_step2(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_forward(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_inverse(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_forwardSkip(mjModel_* m, mjData_* d, int skipstage, int skipsensor);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_inverseSkip(mjModel_* m, mjData_* d, int skipstage, int skipsensor);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_defaultLROpt(mjLROpt_* opt);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_defaultSolRefImp(double* solref, double* solimp);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_defaultOption(mjOption_* opt);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_defaultVisual(mjVisual_* vis);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjModel_* mj_copyModel(mjModel_* dest, mjModel_* src);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_saveModel(mjModel_* m, [MarshalAs(UnmanagedType.LPStr)]string filename, void* buffer, int buffer_sz);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjModel_* mj_loadModel([MarshalAs(UnmanagedType.LPStr)]string filename, void* vfs);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_deleteModel(mjModel_* m);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_sizeModel(mjModel_* m);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjData_* mj_makeData(mjModel_* m);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjData_* mj_copyData(mjData_* dest, mjModel_* m, mjData_* src);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjData_* mjv_copyData(mjData_* dest, mjModel_* m, mjData_* src);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_resetData(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_resetDataDebug(mjModel_* m, mjData_* d, byte debug_value);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_resetDataKeyframe(mjModel_* m, mjData_* d, int key);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_markStack(mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_freeStack(mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void* mj_stackAllocByte(mjData_* d, UIntPtr bytes, UIntPtr alignment);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double* mj_stackAllocNum(mjData_* d, UIntPtr size);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int* mj_stackAllocInt(mjData_* d, UIntPtr size);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_deleteData(mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_resetCallbacks();

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_setConst(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_setLengthRange(mjModel_* m, mjData_* d, int index, mjLROpt_* opt, StringBuilder error, int error_sz);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_printFormattedModel(mjModel_* m, [MarshalAs(UnmanagedType.LPStr)]string filename, [MarshalAs(UnmanagedType.LPStr)]string float_format);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_printModel(mjModel_* m, [MarshalAs(UnmanagedType.LPStr)]string filename);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_printFormattedData(mjModel_* m, mjData_* d, [MarshalAs(UnmanagedType.LPStr)]string filename, [MarshalAs(UnmanagedType.LPStr)]string float_format);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_printData(mjModel_* m, mjData_* d, [MarshalAs(UnmanagedType.LPStr)]string filename);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_printMat(double* mat, int nr, int nc);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_printMatSparse(double* mat, int nr, int* rownnz, int* rowadr, int* colind);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_printSchema([MarshalAs(UnmanagedType.LPStr)]string filename, StringBuilder buffer, int buffer_sz, int flg_html, int flg_pad);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_fwdPosition(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_fwdVelocity(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_fwdActuation(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_fwdAcceleration(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_fwdConstraint(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_Euler(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_RungeKutta(mjModel_* m, mjData_* d, int N);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_implicit(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_invPosition(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_invVelocity(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_invConstraint(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_compareFwdInv(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_sensorPos(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_sensorVel(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_sensorAcc(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_energyPos(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_energyVel(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_checkPos(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_checkVel(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_checkAcc(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_kinematics(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_comPos(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_camlight(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_flex(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_tendon(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_transmission(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_crb(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_makeM(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_factorM(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_solveM(mjModel_* m, mjData_* d, double* x, double* y, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_solveM2(mjModel_* m, mjData_* d, double* x, double* y, double* sqrtInvD, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_comVel(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_passive(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_subtreeVel(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_rne(mjModel_* m, mjData_* d, int flg_acc, double* result);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_rnePostConstraint(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_collision(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_makeConstraint(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_island(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_projectConstraint(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_referenceConstraint(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_constraintUpdate(mjModel_* m, mjData_* d, double* jar, double* cost, int flg_coneHessian);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_stateSize(mjModel_* m, uint spec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_getState(mjModel_* m, mjData_* d, double* state, uint spec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_setState(mjModel_* m, mjData_* d, double* state, uint spec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_setKeyframe(mjModel_* m, mjData_* d, int k);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_addContact(mjModel_* m, mjData_* d, mjContact_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_isPyramidal(mjModel_* m);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_isSparse(mjModel_* m);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_isDual(mjModel_* m);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_mulJacVec(mjModel_* m, mjData_* d, double* res, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_mulJacTVec(mjModel_* m, mjData_* d, double* res, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_jac(mjModel_* m, mjData_* d, double* jacp, double* jacr, double* point, int body);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_jacBody(mjModel_* m, mjData_* d, double* jacp, double* jacr, int body);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_jacBodyCom(mjModel_* m, mjData_* d, double* jacp, double* jacr, int body);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_jacSubtreeCom(mjModel_* m, mjData_* d, double* jacp, int body);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_jacGeom(mjModel_* m, mjData_* d, double* jacp, double* jacr, int geom);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_jacSite(mjModel_* m, mjData_* d, double* jacp, double* jacr, int site);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_jacPointAxis(mjModel_* m, mjData_* d, double* jacPoint, double* jacAxis, double* point, double* axis, int body);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_jacDot(mjModel_* m, mjData_* d, double* jacp, double* jacr, double* point, int body);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_angmomMat(mjModel_* m, mjData_* d, double* mat, int body);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_name2id(mjModel_* m, int type, [MarshalAs(UnmanagedType.LPStr)]string name);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
[return: MarshalAs(UnmanagedType.LPStr)]
public static unsafe extern string mj_id2name(mjModel_* m, int type, int id);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_fullM(mjModel_* m, double* dst, double* M);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_mulM(mjModel_* m, mjData_* d, double* res, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_mulM2(mjModel_* m, mjData_* d, double* res, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_addM(mjModel_* m, mjData_* d, double* dst, int* rownnz, int* rowadr, int* colind);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_applyFT(mjModel_* m, mjData_* d, double* force, double* torque, double* point, int body, double* qfrc_target);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_objectVelocity(mjModel_* m, mjData_* d, int objtype, int objid, double* res, int flg_local);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_objectAcceleration(mjModel_* m, mjData_* d, int objtype, int objid, double* res, int flg_local);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mj_geomDistance(mjModel_* m, mjData_* d, int geom1, int geom2, double distmax, double* fromto);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_contactForce(mjModel_* m, mjData_* d, int id, double* result);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_differentiatePos(mjModel_* m, double* qvel, double dt, double* qpos1, double* qpos2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_integratePos(mjModel_* m, double* qpos, double* qvel, double dt);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_normalizeQuat(mjModel_* m, double* qpos);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_local2Global(mjData_* d, double* xpos, double* xmat, double* pos, double* quat, int body, byte sameframe);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mj_getTotalmass(mjModel_* m);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_setTotalmass(mjModel_* m, double newmass);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
[return: MarshalAs(UnmanagedType.LPStr)]
public static unsafe extern string mj_getPluginConfig(mjModel_* m, int plugin_id, [MarshalAs(UnmanagedType.LPStr)]string attrib);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_loadPluginLibrary([MarshalAs(UnmanagedType.LPStr)]string path);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_version();

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
[return: MarshalAs(UnmanagedType.LPStr)]
public static unsafe extern string mj_versionString();

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_multiRay(mjModel_* m, mjData_* d, double* pnt, double* vec, byte* geomgroup, byte flg_static, int bodyexclude, int* geomid, double* dist, int nray, double cutoff);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mj_ray(mjModel_* m, mjData_* d, double* pnt, double* vec, byte* geomgroup, byte flg_static, int bodyexclude, int* geomid);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mj_rayHfield(mjModel_* m, mjData_* d, int geomid, double* pnt, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mj_rayMesh(mjModel_* m, mjData_* d, int geomid, double* pnt, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_rayGeom(double* pos, double* mat, double* size, double* pnt, double* vec, int geomtype);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_rayFlex(mjModel_* m, mjData_* d, int flex_layer, byte flg_vert, byte flg_edge, byte flg_face, byte flg_skin, int flexid, double* pnt, double* vec, int* vertid);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_raySkin(int nface, int nvert, int* face, float* vert, double* pnt, double* vec, int* vertid);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_defaultCamera(mjvCamera_* cam);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_defaultFreeCamera(mjModel_* m, mjvCamera_* cam);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_defaultPerturb(mjvPerturb_* pert);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_room2model(double* modelpos, double* modelquat, double* roompos, double* roomquat, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_model2room(double* roompos, double* roomquat, double* modelpos, double* modelquat, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_cameraInModel(double* headpos, double* forward, double* up, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_cameraInRoom(double* headpos, double* forward, double* up, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mjv_frustumHeight(mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_alignToCamera(double* res, double* vec, double* forward);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_moveCamera(mjModel_* m, int action, double reldx, double reldy, mjvScene_* scn, mjvCamera_* cam);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_movePerturb(mjModel_* m, mjData_* d, int action, double reldx, double reldy, mjvScene_* scn, mjvPerturb_* pert);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_moveModel(mjModel_* m, int action, double reldx, double reldy, double* roomup, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_initPerturb(mjModel_* m, mjData_* d, mjvScene_* scn, mjvPerturb_* pert);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_applyPerturbPose(mjModel_* m, mjData_* d, mjvPerturb_* pert, int flg_paused);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_applyPerturbForce(mjModel_* m, mjData_* d, mjvPerturb_* pert);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjvGLCamera_* mjv_averageCamera(mjvGLCamera_* cam1, mjvGLCamera_* cam2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mjv_select(mjModel_* m, mjData_* d, mjvOption_* vopt, double aspectratio, double relx, double rely, mjvScene_* scn, double* selpnt, int* geomid, int* flexid, int* skinid);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_defaultOption(mjvOption_* opt);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_defaultFigure(mjvFigure_* fig);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_initGeom(mjvGeom_* geom, int type, double* size, double* pos, double* mat, float* rgba);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_connector(mjvGeom_* geom, int type, double width, double* from, double* to);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_defaultScene(mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_makeScene(mjModel_* m, mjvScene_* scn, int maxgeom);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_freeScene(mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_updateScene(mjModel_* m, mjData_* d, mjvOption_* opt, mjvPerturb_* pert, mjvCamera_* cam, int catmask, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_copyModel(mjModel_* dest, mjModel_* src);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_addGeoms(mjModel_* m, mjData_* d, mjvOption_* opt, mjvPerturb_* pert, int catmask, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_makeLights(mjModel_* m, mjData_* d, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_updateCamera(mjModel_* m, mjData_* d, mjvCamera_* cam, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_updateSkin(mjModel_* m, mjData_* d, mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_defaultContext(mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_makeContext(mjModel_* m, mjrContext_* con, int fontscale);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_changeFont(int fontscale, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_addAux(int index, int width, int height, int samples, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_freeContext(mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_resizeOffscreen(int width, int height, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_uploadTexture(mjModel_* m, mjrContext_* con, int texid);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_uploadMesh(mjModel_* m, mjrContext_* con, int meshid);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_uploadHField(mjModel_* m, mjrContext_* con, int hfieldid);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_restoreBuffer(mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_setBuffer(int framebuffer, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_readPixels(byte* rgb, float* depth, mjrRect_* viewport, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_drawPixels(byte* rgb, float* depth, mjrRect_* viewport, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_blitBuffer(mjrRect_* src, mjrRect_* dst, int flg_color, int flg_depth, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_setAux(int index, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_blitAux(int index, mjrRect_* src, int left, int bottom, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_text(int font, [MarshalAs(UnmanagedType.LPStr)]string txt, mjrContext_* con, float x, float y, float r, float g, float b);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_overlay(int font, int gridpos, mjrRect_* viewport, [MarshalAs(UnmanagedType.LPStr)]string overlay, [MarshalAs(UnmanagedType.LPStr)]string overlay2, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjrRect_* mjr_maxViewport(mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_rectangle(mjrRect_* viewport, float r, float g, float b, float a);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_label(mjrRect_* viewport, int font, [MarshalAs(UnmanagedType.LPStr)]string txt, float r, float g, float b, float a, float rt, float gt, float bt, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_figure(mjrRect_* viewport, mjvFigure_* fig, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_render(mjrRect_* viewport, mjvScene_* scn, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjr_finish();

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mjr_getError();

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mjr_findRect(int x, int y, int nrect, mjrRect_* rect);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjuiThemeSpacing_* mjui_themeSpacing(int ind);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjuiThemeColor_* mjui_themeColor(int ind);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjui_add(mjUI_* ui, mjuiDef_* def);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjui_addToSection(mjUI_* ui, int sect, mjuiDef_* def);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjui_resize(mjUI_* ui, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjui_update(int section, int item, mjUI_* ui, mjuiState_* state, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern mjuiItem_* mjui_event(mjUI_* ui, mjuiState_* state, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjui_render(mjUI_* ui, mjuiState_* state, mjrContext_* con);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_error_i([MarshalAs(UnmanagedType.LPStr)]string msg, int i);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_error_s([MarshalAs(UnmanagedType.LPStr)]string msg, [MarshalAs(UnmanagedType.LPStr)]string text);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_warning_i([MarshalAs(UnmanagedType.LPStr)]string msg, int i);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_warning_s([MarshalAs(UnmanagedType.LPStr)]string msg, [MarshalAs(UnmanagedType.LPStr)]string text);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_clearHandlers();

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void* mju_malloc(UIntPtr size);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_free(void* ptr);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_warning(mjData_* d, int warning, int info);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_writeLog([MarshalAs(UnmanagedType.LPStr)]string type, [MarshalAs(UnmanagedType.LPStr)]string msg);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_zero3(double* res);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_copy3(double* res, double* data);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_scl3(double* res, double* vec, double scl);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_add3(double* res, double* vec1, double* vec2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_sub3(double* res, double* vec1, double* vec2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_addTo3(double* res, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_subFrom3(double* res, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_addToScl3(double* res, double* vec, double scl);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_addScl3(double* res, double* vec1, double* vec2, double scl);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_normalize3(double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_norm3(double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_dot3(double* vec1, double* vec2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_dist3(double* pos1, double* pos2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulMatVec3(double* res, double* mat, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulMatTVec3(double* res, double* mat, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_cross(double* res, double* a, double* b);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_zero4(double* res);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_unit4(double* res);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_copy4(double* res, double* data);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_normalize4(double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_zero(double* res, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_fill(double* res, double val, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_copy(double* res, double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_sum(double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_L1(double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_scl(double* res, double* vec, double scl, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_add(double* res, double* vec1, double* vec2, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_sub(double* res, double* vec1, double* vec2, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_addTo(double* res, double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_subFrom(double* res, double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_addToScl(double* res, double* vec, double scl, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_addScl(double* res, double* vec1, double* vec2, double scl, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_normalize(double* res, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_norm(double* res, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_dot(double* vec1, double* vec2, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulMatVec(double* res, double* mat, double* vec, int nr, int nc);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulMatTVec(double* res, double* mat, double* vec, int nr, int nc);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_mulVecMatVec(double* vec1, double* mat, double* vec2, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_transpose(double* res, double* mat, int nr, int nc);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_symmetrize(double* res, double* mat, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_eye(double* mat, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulMatMat(double* res, double* mat1, double* mat2, int r1, int c1, int c2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulMatMatT(double* res, double* mat1, double* mat2, int r1, int c1, int r2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulMatTMat(double* res, double* mat1, double* mat2, int r1, int c1, int c2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_sqrMatTD(double* res, double* mat, double* diag, int nr, int nc);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_transformSpatial(double* res, double* vec, int flg_force, double* newpos, double* oldpos, double* rotnew2old);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_dense2sparse(double* res, double* mat, int nr, int nc, int* rownnz, int* rowadr, int* colind, int nnz);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_sparse2dense(double* res, double* mat, int nr, int nc, int* rownnz, int* rowadr, int* colind);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_rotVecQuat(double* res, double* vec, double* quat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_negQuat(double* res, double* quat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulQuat(double* res, double* quat1, double* quat2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulQuatAxis(double* res, double* quat, double* axis);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_axisAngle2Quat(double* res, double* axis, double angle);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_quat2Vel(double* res, double* quat, double dt);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_subQuat(double* res, double* qa, double* qb);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_quat2Mat(double* res, double* quat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mat2Quat(double* quat, double* mat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_derivQuat(double* res, double* quat, double* vel);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_quatIntegrate(double* quat, double* vel, double scale);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_quatZ2Vec(double* quat, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_mat2Rot(double* quat, double* mat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_euler2Quat(double* quat, double* euler, [MarshalAs(UnmanagedType.LPStr)]string seq);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_mulPose(double* posres, double* quatres, double* pos1, double* quat1, double* pos2, double* quat2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_negPose(double* posres, double* quatres, double* pos, double* quat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_trnVecPose(double* res, double* pos, double* quat, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_cholFactor(double* mat, int n, double mindiag);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_cholSolve(double* res, double* mat, double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_cholUpdate(double* mat, double* x, int n, int flg_plus);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_cholFactorBand(double* mat, int ntotal, int nband, int ndense, double diagadd, double diagmul);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_cholSolveBand(double* res, double* mat, double* vec, int ntotal, int nband, int ndense);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_band2Dense(double* res, double* mat, int ntotal, int nband, int ndense, byte flg_sym);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_dense2Band(double* res, double* mat, int ntotal, int nband, int ndense);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_bandMulMatVec(double* res, double* mat, double* vec, int ntotal, int nband, int ndense, int nvec, byte flg_sym);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_bandDiag(int i, int ntotal, int nband, int ndense);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_eig3(double* eigval, double* eigvec, double* quat, double* mat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_boxQP(double* res, double* R, int* index, double* H, double* g, int n, double* lower, double* upper);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_boxQPmalloc(double** res, double** R, int** index, double** H, double** g, int n, double** lower, double** upper);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_muscleGain(double len, double vel, double* lengthrange, double acc0, double* prm);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_muscleBias(double len, double* lengthrange, double acc0, double* prm);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_muscleDynamics(double ctrl, double act, double* prm);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_encodePyramid(double* pyramid, double* force, double* mu, int dim);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_decodePyramid(double* force, double* pyramid, double* mu, int dim);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_springDamper(double pos0, double vel0, double Kp, double Kv, double dt);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_min(double a, double b);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_max(double a, double b);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_clip(double x, double min, double max);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_sign(double x);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_round(double x);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
[return: MarshalAs(UnmanagedType.LPStr)]
public static unsafe extern string mju_type2Str(int type);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_str2Type([MarshalAs(UnmanagedType.LPStr)]string str);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
[return: MarshalAs(UnmanagedType.LPStr)]
public static unsafe extern string mju_writeNumBytes(UIntPtr nbytes);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
[return: MarshalAs(UnmanagedType.LPStr)]
public static unsafe extern string mju_warningText(int warning, UIntPtr info);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_isBad(double x);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_isZero(double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_standardNormal(double* num2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_f2n(double* res, float* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_n2f(float* res, double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_d2n(double* res, double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_n2d(double* res, double* vec, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_insertionSort(double* list, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_insertionSortInt(int* list, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_Halton(int index, int base_);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
[return: MarshalAs(UnmanagedType.LPStr)]
public static unsafe extern string mju_strncpy(StringBuilder dst, [MarshalAs(UnmanagedType.LPStr)]string src, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_sigmoid(double x);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjd_transitionFD(mjModel_* m, mjData_* d, double eps, byte flg_centered, double* A, double* B, double* C, double* D);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjd_inverseFD(mjModel_* m, mjData_* d, double eps, byte flg_actuation, double* DfDq, double* DfDv, double* DfDa, double* DsDq, double* DsDv, double* DsDa, double* DmDq);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjd_subQuat(double* qa, double* qb, double* Da, double* Db);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjd_quatIntegrate(double* vel, double scale, double* Dquat, double* Dvel, double* Dscale);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_bindThreadPool(mjData_* d, void* thread_pool);
}
}
