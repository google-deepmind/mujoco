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
public const bool THIRD_PARTY_MUJOCO_MJMODEL_H_ = true;
public const double mjPI = 3.141592653589793;
public const double mjMAXVAL = 10000000000.0;
public const double mjMINMU = 1e-05;
public const double mjMINIMP = 0.0001;
public const double mjMAXIMP = 0.9999;
public const int mjMAXCONPAIR = 50;
public const int mjMAXVFS = 2000;
public const int mjMAXVFSNAME = 1000;
public const int mjNEQDATA = 11;
public const int mjNDYN = 10;
public const int mjNGAIN = 10;
public const int mjNBIAS = 10;
public const int mjNFLUID = 12;
public const int mjNREF = 2;
public const int mjNIMP = 5;
public const int mjNSOLVER = 1000;
public const bool THIRD_PARTY_MUJOCO_MJRENDER_H_ = true;
public const int mjNAUX = 10;
public const int mjMAXTEXTURE = 1000;
public const bool THIRD_PARTY_MUJOCO_INCLUDE_MJTNUM_H_ = true;
public const bool mjUSEDOUBLE = true;
public const double mjMINVAL = 1e-15;
public const bool THIRD_PARTY_MUJOCO_MJUI_H_ = true;
public const int mjMAXUISECT = 10;
public const int mjMAXUIITEM = 80;
public const int mjMAXUITEXT = 300;
public const int mjMAXUINAME = 40;
public const int mjMAXUIMULTI = 35;
public const int mjMAXUIEDIT = 7;
public const int mjMAXUIRECT = 25;
public const int mjSEPCLOSED = 1000;
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
public const bool THIRD_PARTY_MUJOCO_MJVISUALIZE_H_ = true;
public const int mjNGROUP = 6;
public const int mjMAXLIGHT = 100;
public const int mjMAXOVERLAY = 500;
public const int mjMAXLINE = 100;
public const int mjMAXLINEPNT = 1000;
public const int mjMAXPLANEGRID = 200;
public const bool THIRD_PARTY_MUJOCO_MJXMACRO_H_ = true;
public const bool THIRD_PARTY_MUJOCO_MUJOCO_H_ = true;
public const int mjVERSION_HEADER = 222;


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
  mjTIMER_ACCELERATION = 6,
  mjTIMER_CONSTRAINT = 7,
  mjTIMER_POS_KINEMATICS = 8,
  mjTIMER_POS_INERTIA = 9,
  mjTIMER_POS_COLLISION = 10,
  mjTIMER_POS_MAKE = 11,
  mjTIMER_POS_PROJECT = 12,
  mjNTIMER = 13,
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
  mjNDISABLE = 13,
}
public enum mjtEnableBit : int{
  mjENBL_OVERRIDE = 1,
  mjENBL_ENERGY = 2,
  mjENBL_FWDINV = 4,
  mjENBL_SENSORNOISE = 8,
  mjENBL_MULTICCD = 16,
  mjNENABLE = 5,
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
  mjNGEOMTYPES = 8,
  mjGEOM_ARROW = 100,
  mjGEOM_ARROW1 = 101,
  mjGEOM_ARROW2 = 102,
  mjGEOM_LINE = 103,
  mjGEOM_SKIN = 104,
  mjGEOM_LABEL = 105,
  mjGEOM_NONE = 1001,
}
public enum mjtCamLight : int{
  mjCAMLIGHT_FIXED = 0,
  mjCAMLIGHT_TRACK = 1,
  mjCAMLIGHT_TRACKCOM = 2,
  mjCAMLIGHT_TARGETBODY = 3,
  mjCAMLIGHT_TARGETBODYCOM = 4,
}
public enum mjtTexture : int{
  mjTEXTURE_2D = 0,
  mjTEXTURE_CUBE = 1,
  mjTEXTURE_SKYBOX = 2,
}
public enum mjtIntegrator : int{
  mjINT_EULER = 0,
  mjINT_RK4 = 1,
  mjINT_IMPLICIT = 2,
}
public enum mjtCollision : int{
  mjCOL_ALL = 0,
  mjCOL_PAIR = 1,
  mjCOL_DYNAMIC = 2,
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
  mjEQ_DISTANCE = 4,
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
  mjDYN_MUSCLE = 3,
  mjDYN_USER = 4,
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
  mjOBJ_MESH = 9,
  mjOBJ_SKIN = 10,
  mjOBJ_HFIELD = 11,
  mjOBJ_TEXTURE = 12,
  mjOBJ_MATERIAL = 13,
  mjOBJ_PAIR = 14,
  mjOBJ_EXCLUDE = 15,
  mjOBJ_EQUALITY = 16,
  mjOBJ_TENDON = 17,
  mjOBJ_ACTUATOR = 18,
  mjOBJ_SENSOR = 19,
  mjOBJ_NUMERIC = 20,
  mjOBJ_TEXT = 21,
  mjOBJ_TUPLE = 22,
  mjOBJ_KEY = 23,
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
  mjSENS_JOINTPOS = 8,
  mjSENS_JOINTVEL = 9,
  mjSENS_TENDONPOS = 10,
  mjSENS_TENDONVEL = 11,
  mjSENS_ACTUATORPOS = 12,
  mjSENS_ACTUATORVEL = 13,
  mjSENS_ACTUATORFRC = 14,
  mjSENS_BALLQUAT = 15,
  mjSENS_BALLANGVEL = 16,
  mjSENS_JOINTLIMITPOS = 17,
  mjSENS_JOINTLIMITVEL = 18,
  mjSENS_JOINTLIMITFRC = 19,
  mjSENS_TENDONLIMITPOS = 20,
  mjSENS_TENDONLIMITVEL = 21,
  mjSENS_TENDONLIMITFRC = 22,
  mjSENS_FRAMEPOS = 23,
  mjSENS_FRAMEQUAT = 24,
  mjSENS_FRAMEXAXIS = 25,
  mjSENS_FRAMEYAXIS = 26,
  mjSENS_FRAMEZAXIS = 27,
  mjSENS_FRAMELINVEL = 28,
  mjSENS_FRAMEANGVEL = 29,
  mjSENS_FRAMELINACC = 30,
  mjSENS_FRAMEANGACC = 31,
  mjSENS_SUBTREECOM = 32,
  mjSENS_SUBTREELINVEL = 33,
  mjSENS_SUBTREEANGMOM = 34,
  mjSENS_CLOCK = 35,
  mjSENS_USER = 36,
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
public enum mjtLRMode : int{
  mjLRMODE_NONE = 0,
  mjLRMODE_MUSCLE = 1,
  mjLRMODE_MUSCLEUSER = 2,
  mjLRMODE_ALL = 3,
}
public enum mjtGridPos : int{
  mjGRID_TOPLEFT = 0,
  mjGRID_TOPRIGHT = 1,
  mjGRID_BOTTOMLEFT = 2,
  mjGRID_BOTTOMRIGHT = 3,
}
public enum mjtFramebuffer : int{
  mjFB_WINDOW = 0,
  mjFB_OFFSCREEN = 1,
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
  mjLABEL_SKIN = 10,
  mjLABEL_SELECTION = 11,
  mjLABEL_SELPNT = 12,
  mjLABEL_CONTACTFORCE = 13,
  mjNLABEL = 14,
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
  mjVIS_CONTACTFORCE = 15,
  mjVIS_CONTACTSPLIT = 16,
  mjVIS_TRANSPARENT = 17,
  mjVIS_AUTOCONNECT = 18,
  mjVIS_COM = 19,
  mjVIS_SELECT = 20,
  mjVIS_STATIC = 21,
  mjVIS_SKIN = 22,
  mjNVISFLAG = 23,
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
  public fixed double solimp[5];
  public double mu;
  public fixed double H[36];
  public int dim;
  public int geom1;
  public int geom2;
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
  public int nstack;
  public int nbuffer;
  public int pstack;
  public int maxuse_stack;
  public int maxuse_con;
  public int maxuse_efc;
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
  public int solver_iter;
  public int solver_nnz;
  public fixed double solver_fwdinv[2];
  public int ne;
  public int nf;
  public int nefc;
  public int ncon;
  public double time;
  public fixed double energy[2];
  public void* buffer;
  public double* stack;
  public double* qpos;
  public double* qvel;
  public double* act;
  public double* qacc_warmstart;
  public double* ctrl;
  public double* qfrc_applied;
  public double* xfrc_applied;
  public double* mocap_pos;
  public double* mocap_quat;
  public double* qacc;
  public double* act_dot;
  public double* userdata;
  public double* sensordata;
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
  public int* ten_wrapadr;
  public int* ten_wrapnum;
  public int* ten_J_rownnz;
  public int* ten_J_rowadr;
  public int* ten_J_colind;
  public double* ten_length;
  public double* ten_J;
  public int* wrap_obj;
  public double* wrap_xpos;
  public double* actuator_length;
  public double* actuator_moment;
  public double* crb;
  public double* qM;
  public double* qLD;
  public double* qLDiagInv;
  public double* qLDiagSqrtInv;
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
  public int* efc_AR_rownnz;
  public int* efc_AR_rowadr;
  public int* efc_AR_colind;
  public double* efc_AR;
  public double* ten_velocity;
  public double* actuator_velocity;
  public double* cvel;
  public double* cdof_dot;
  public double* qfrc_bias;
  public double* qfrc_passive;
  public double* efc_vel;
  public double* efc_aref;
  public double* subtree_linvel;
  public double* subtree_angmom;
  public double* qH;
  public double* qHDiagInv;
  public int* D_rownnz;
  public int* D_rowadr;
  public int* D_colind;
  public double* qDeriv;
  public double* qLU;
  public double* actuator_force;
  public double* qfrc_actuator;
  public double* qfrc_smooth;
  public double* qacc_smooth;
  public double* efc_b;
  public double* efc_force;
  public int* efc_state;
  public double* qfrc_constraint;
  public double* qfrc_inverse;
  public double* cacc;
  public double* cfrc_int;
  public double* cfrc_ext;
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
  public double inteval;
  public double tolrange;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct _mjVFS
{
  public int nfile;
  [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2000 * 1000)] public char[] filename;
  [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2000)] public int[] filesize;
  [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2000)] public IntPtr[] filedata;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjOption_ {
  public double timestep;
  public double apirate;
  public double impratio;
  public double tolerance;
  public double noslip_tolerance;
  public double mpr_tolerance;
  public fixed double gravity[3];
  public fixed double wind[3];
  public fixed double magnetic[3];
  public double density;
  public double viscosity;
  public double o_margin;
  public fixed double o_solref[2];
  public fixed double o_solimp[5];
  public int integrator;
  public int collision;
  public int cone;
  public int jacobian;
  public int solver;
  public int iterations;
  public int noslip_iterations;
  public int mpr_iterations;
  public int disableflags;
  public int enableflags;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct global {
  public float fovy;
  public float ipd;
  public float azimuth;
  public float elevation;
  public float linewidth;
  public float glow;
  public int offwidth;
  public int offheight;
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
  public int njnt;
  public int ngeom;
  public int nsite;
  public int ncam;
  public int nlight;
  public int nmesh;
  public int nmeshvert;
  public int nmeshtexvert;
  public int nmeshface;
  public int nmeshgraph;
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
  public int nuser_body;
  public int nuser_jnt;
  public int nuser_geom;
  public int nuser_site;
  public int nuser_cam;
  public int nuser_tendon;
  public int nuser_actuator;
  public int nuser_sensor;
  public int nnames;
  public int nM;
  public int nD;
  public int nemax;
  public int njmax;
  public int nconmax;
  public int nstack;
  public int nuserdata;
  public int nsensordata;
  public int nbuffer;
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
  public double* body_user;
  public int* jnt_type;
  public int* jnt_qposadr;
  public int* jnt_dofadr;
  public int* jnt_bodyid;
  public int* jnt_group;
  public byte* jnt_limited;
  public double* jnt_solref;
  public double* jnt_solimp;
  public double* jnt_pos;
  public double* jnt_axis;
  public double* jnt_stiffness;
  public double* jnt_range;
  public double* jnt_margin;
  public double* jnt_user;
  public int* dof_bodyid;
  public int* dof_jntid;
  public int* dof_parentid;
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
  public byte* geom_sameframe;
  public double* geom_solmix;
  public double* geom_solref;
  public double* geom_solimp;
  public double* geom_size;
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
  public double* cam_fovy;
  public double* cam_ipd;
  public double* cam_user;
  public int* light_mode;
  public int* light_bodyid;
  public int* light_targetbodyid;
  public byte* light_directional;
  public byte* light_castshadow;
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
  public int* mesh_vertadr;
  public int* mesh_vertnum;
  public int* mesh_texcoordadr;
  public int* mesh_faceadr;
  public int* mesh_facenum;
  public int* mesh_graphadr;
  public float* mesh_vert;
  public float* mesh_normal;
  public float* mesh_texcoord;
  public int* mesh_face;
  public int* mesh_graph;
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
  public double* hfield_size;
  public int* hfield_nrow;
  public int* hfield_ncol;
  public int* hfield_adr;
  public float* hfield_data;
  public int* tex_type;
  public int* tex_height;
  public int* tex_width;
  public int* tex_adr;
  public byte* tex_rgb;
  public int* mat_texid;
  public byte* mat_texuniform;
  public float* mat_texrepeat;
  public float* mat_emission;
  public float* mat_specular;
  public float* mat_shininess;
  public float* mat_reflectance;
  public float* mat_rgba;
  public int* pair_dim;
  public int* pair_geom1;
  public int* pair_geom2;
  public int* pair_signature;
  public double* pair_solref;
  public double* pair_solimp;
  public double* pair_margin;
  public double* pair_gap;
  public double* pair_friction;
  public int* exclude_signature;
  public int* eq_type;
  public int* eq_obj1id;
  public int* eq_obj2id;
  public byte* eq_active;
  public double* eq_solref;
  public double* eq_solimp;
  public double* eq_data;
  public int* tendon_adr;
  public int* tendon_num;
  public int* tendon_matid;
  public int* tendon_group;
  public byte* tendon_limited;
  public double* tendon_width;
  public double* tendon_solref_lim;
  public double* tendon_solimp_lim;
  public double* tendon_solref_fri;
  public double* tendon_solimp_fri;
  public double* tendon_range;
  public double* tendon_margin;
  public double* tendon_stiffness;
  public double* tendon_damping;
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
  public int* actuator_group;
  public byte* actuator_ctrllimited;
  public byte* actuator_forcelimited;
  public byte* actuator_actlimited;
  public double* actuator_dynprm;
  public double* actuator_gainprm;
  public double* actuator_biasprm;
  public double* actuator_ctrlrange;
  public double* actuator_forcerange;
  public double* actuator_actrange;
  public double* actuator_gear;
  public double* actuator_cranklength;
  public double* actuator_acc0;
  public double* actuator_length0;
  public double* actuator_lengthrange;
  public double* actuator_user;
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
  public char* names;
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
  public int ntexture;
  public fixed int textureType[100];
  public fixed uint texture[100];
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
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjuiThemeSpacing_ {
  public int total;
  public int scroll;
  public int label;
  public int section;
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
  public fixed float sectfont[3];
  public fixed float sectsymbol[3];
  public fixed float sectpane[3];
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
  public int nitem;
  public mjrRect_ rtitle;
  public mjrRect_ rcontent;
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
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvPerturb_ {
  public int select;
  public int skinselect;
  public int active;
  public int active2;
  public fixed double refpos[3];
  public fixed double refquat[4];
  public fixed double localpos[3];
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
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvGLCamera_ {
  public fixed float pos[3];
  public fixed float forward[3];
  public fixed float up[3];
  public float frustum_center;
  public float frustum_bottom;
  public float frustum_top;
  public float frustum_near;
  public float frustum_far;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvGeom_ {
  public int type;
  public int dataid;
  public int objtype;
  public int objid;
  public int category;
  public int texid;
  public int texuniform;
  public int texcoord;
  public int segid;
  public fixed float texrepeat[2];
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
  public fixed float attenuation[3];
  public float cutoff;
  public float exponent;
  public fixed float ambient[3];
  public fixed float diffuse[3];
  public fixed float specular[3];
  public byte headlight;
  public byte directional;
  public byte castshadow;
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
  public fixed byte skingroup[6];
  public fixed byte flags[23];
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct mjvScene_ {
  public int maxgeom;
  public int ngeom;
  public mjvGeom_* geoms;
  public int* geomorder;
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
public static unsafe extern int mj_makeEmptyFileVFS(void* vfs, [MarshalAs(UnmanagedType.LPStr)]string filename, int filesize);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_findFileVFS(void* vfs, [MarshalAs(UnmanagedType.LPStr)]string filename);

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
public static unsafe extern int mj_printSchema([MarshalAs(UnmanagedType.LPStr)]string filename, StringBuilder buffer, int buffer_sz, int flg_html, int flg_pad);

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
public static unsafe extern void mj_resetData(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_resetDataDebug(mjModel_* m, mjData_* d, byte debug_value);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_resetDataKeyframe(mjModel_* m, mjData_* d, int key);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double* mj_stackAlloc(mjData_* d, int size);

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
public static unsafe extern void mj_tendon(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_transmission(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_crb(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_factorM(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_solveM(mjModel_* m, mjData_* d, double* x, double* y, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_solveM2(mjModel_* m, mjData_* d, double* x, double* y, int n);

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
public static unsafe extern void mj_projectConstraint(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_referenceConstraint(mjModel_* m, mjData_* d);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_constraintUpdate(mjModel_* m, mjData_* d, double* jar, double* cost, int flg_coneHessian);

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
public static unsafe extern int mj_version();

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mj_ray(mjModel_* m, mjData_* d, double* pnt, double* vec, byte* geomgroup, byte flg_static, int bodyexclude, int* geomid);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mj_rayHfield(mjModel_* m, mjData_* d, int geomid, double* pnt, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mj_rayMesh(mjModel_* m, mjData_* d, int geomid, double* pnt, double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_rayGeom(double* pos, double* mat, double* size, double* pnt, double* vec, int geomtype);

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
public static unsafe extern int mjv_select(mjModel_* m, mjData_* d, mjvOption_* vopt, double aspectratio, double relx, double rely, mjvScene_* scn, double* selpnt, int* geomid, int* skinid);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_defaultOption(mjvOption_* opt);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_defaultFigure(mjvFigure_* fig);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_initGeom(mjvGeom_* geom, int type, double* size, double* pos, double* mat, float* rgba);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_makeConnector(mjvGeom_* geom, int type, double width, double a0, double a1, double a2, double b0, double b1, double b2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_defaultScene(mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_makeScene(mjModel_* m, mjvScene_* scn, int maxgeom);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_freeScene(mjvScene_* scn);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mjv_updateScene(mjModel_* m, mjData_* d, mjvOption_* opt, mjvPerturb_* pert, mjvCamera_* cam, int catmask, mjvScene_* scn);

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
public static unsafe extern void mju_error([MarshalAs(UnmanagedType.LPStr)]string msg);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_error_i([MarshalAs(UnmanagedType.LPStr)]string msg, int i);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_error_s([MarshalAs(UnmanagedType.LPStr)]string msg, [MarshalAs(UnmanagedType.LPStr)]string text);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_warning([MarshalAs(UnmanagedType.LPStr)]string msg);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_warning_i([MarshalAs(UnmanagedType.LPStr)]string msg, int i);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_warning_s([MarshalAs(UnmanagedType.LPStr)]string msg, [MarshalAs(UnmanagedType.LPStr)]string text);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_clearHandlers();

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void* mju_malloc(uint size);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_free(void* ptr);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_warning(mjData_* d, int warning, int info);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_writeLog([MarshalAs(UnmanagedType.LPStr)]string type, [MarshalAs(UnmanagedType.LPStr)]string msg);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mj_activate([MarshalAs(UnmanagedType.LPStr)]string filename);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mj_deactivate();

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
public static unsafe extern double mju_normalize3(double* res);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_norm3(double* vec);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_dot3(double* vec1, double* vec2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_dist3(double* pos1, double* pos2);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_rotVecMat(double* res, double* vec, double* mat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_rotVecMatT(double* res, double* vec, double* mat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_cross(double* res, double* a, double* b);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_zero4(double* res);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_unit4(double* res);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_copy4(double* res, double* data);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern double mju_normalize4(double* res);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_zero(double* res, int n);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern void mju_copy(double* res, double* data, int n);

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
public static unsafe extern int mju_eig3(double* eigval, double* eigvec, double* quat, double* mat);

[DllImport("mujoco", CallingConvention = CallingConvention.Cdecl)]
public static unsafe extern int mju_boxQP(double* res, double* R, int* index, double* H, double* g, int n, double* lower, double* upper);

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
public static unsafe extern string mju_warningText(int warning, int info);

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
public static unsafe extern void mjd_transitionFD(mjModel_* m, mjData_* d, double eps, byte centered, double* A, double* B, double* C, double* D);
}
}
