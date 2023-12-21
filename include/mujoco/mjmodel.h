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

#ifndef MUJOCO_MJMODEL_H_
#define MUJOCO_MJMODEL_H_

#include <stddef.h>


#include <mujoco/mjtnum.h>

// global constants
#define mjPI            3.14159265358979323846
#define mjMAXVAL        1E+10     // maximum value in qpos, qvel, qacc
#define mjMINMU         1E-5      // minimum friction coefficient
#define mjMINIMP        0.0001    // minimum constraint impedance
#define mjMAXIMP        0.9999    // maximum constraint impedance
#define mjMAXCONPAIR    50        // maximum number of contacts per geom pair
#define mjMAXTREEDEPTH  50        // maximum bounding volume hierarchy depth
#define mjMAXVFS        2000      // maximum number of files in virtual file system
#define mjMAXVFSNAME    1000      // maximum filename size in virtual file system


//---------------------------------- sizes ---------------------------------------------------------

#define mjNEQDATA       11        // number of eq_data fields
#define mjNDYN          10        // number of actuator dynamics parameters
#define mjNGAIN         10        // number of actuator gain parameters
#define mjNBIAS         10        // number of actuator bias parameters
#define mjNFLUID        12        // number of fluid interaction parameters
#define mjNREF          2         // number of solver reference parameters
#define mjNIMP          5         // number of solver impedance parameters
#define mjNSOLVER       200       // size of one mjData.solver array
#define mjNISLAND       20        // number of mjData.solver arrays


//---------------------------------- enum types (mjt) ----------------------------------------------

typedef enum mjtDisableBit_ {     // disable default feature bitflags
  mjDSBL_CONSTRAINT   = 1<<0,     // entire constraint solver
  mjDSBL_EQUALITY     = 1<<1,     // equality constraints
  mjDSBL_FRICTIONLOSS = 1<<2,     // joint and tendon frictionloss constraints
  mjDSBL_LIMIT        = 1<<3,     // joint and tendon limit constraints
  mjDSBL_CONTACT      = 1<<4,     // contact constraints
  mjDSBL_PASSIVE      = 1<<5,     // passive forces
  mjDSBL_GRAVITY      = 1<<6,     // gravitational forces
  mjDSBL_CLAMPCTRL    = 1<<7,     // clamp control to specified range
  mjDSBL_WARMSTART    = 1<<8,     // warmstart constraint solver
  mjDSBL_FILTERPARENT = 1<<9,     // remove collisions with parent body
  mjDSBL_ACTUATION    = 1<<10,    // apply actuation forces
  mjDSBL_REFSAFE      = 1<<11,    // integrator safety: make ref[0]>=2*timestep
  mjDSBL_SENSOR       = 1<<12,    // sensors
  mjDSBL_MIDPHASE     = 1<<13,    // mid-phase collision filtering
  mjDSBL_EULERDAMP    = 1<<14,    // implicit integration of joint damping in Euler integrator

  mjNDISABLE          = 15        // number of disable flags
} mjtDisableBit;


typedef enum mjtEnableBit_ {      // enable optional feature bitflags
  mjENBL_OVERRIDE     = 1<<0,     // override contact parameters
  mjENBL_ENERGY       = 1<<1,     // energy computation
  mjENBL_FWDINV       = 1<<2,     // record solver statistics
  mjENBL_INVDISCRETE  = 1<<3,     // discrete-time inverse dynamics
  mjENBL_SENSORNOISE  = 1<<4,     // add noise to sensor data
                                  // experimental features:
  mjENBL_MULTICCD     = 1<<5,     // multi-point convex collision detection
  mjENBL_ISLAND       = 1<<6,     // constraint island discovery

  mjNENABLE           = 7         // number of enable flags
} mjtEnableBit;


typedef enum mjtJoint_ {          // type of degree of freedom
  mjJNT_FREE          = 0,        // global position and orientation (quat)       (7)
  mjJNT_BALL,                     // orientation (quat) relative to parent        (4)
  mjJNT_SLIDE,                    // sliding distance along body-fixed axis       (1)
  mjJNT_HINGE                     // rotation angle (rad) around body-fixed axis  (1)
} mjtJoint;


typedef enum mjtGeom_ {           // type of geometric shape
  // regular geom types
  mjGEOM_PLANE        = 0,        // plane
  mjGEOM_HFIELD,                  // height field
  mjGEOM_SPHERE,                  // sphere
  mjGEOM_CAPSULE,                 // capsule
  mjGEOM_ELLIPSOID,               // ellipsoid
  mjGEOM_CYLINDER,                // cylinder
  mjGEOM_BOX,                     // box
  mjGEOM_MESH,                    // mesh
  mjGEOM_SDF,                     // signed distance field

  mjNGEOMTYPES,                   // number of regular geom types

  // rendering-only geom types: not used in mjModel, not counted in mjNGEOMTYPES
  mjGEOM_ARROW        = 100,      // arrow
  mjGEOM_ARROW1,                  // arrow without wedges
  mjGEOM_ARROW2,                  // arrow in both directions
  mjGEOM_LINE,                    // line
  mjGEOM_LINEBOX,                 // box with line edges
  mjGEOM_FLEX,                    // flex
  mjGEOM_SKIN,                    // skin
  mjGEOM_LABEL,                   // text label
  mjGEOM_TRIANGLE,                // triangle

  mjGEOM_NONE         = 1001      // missing geom type
} mjtGeom;


typedef enum mjtCamLight_ {       // tracking mode for camera and light
  mjCAMLIGHT_FIXED    = 0,        // pos and rot fixed in body
  mjCAMLIGHT_TRACK,               // pos tracks body, rot fixed in global
  mjCAMLIGHT_TRACKCOM,            // pos tracks subtree com, rot fixed in body
  mjCAMLIGHT_TARGETBODY,          // pos fixed in body, rot tracks target body
  mjCAMLIGHT_TARGETBODYCOM        // pos fixed in body, rot tracks target subtree com
} mjtCamLight;


typedef enum mjtTexture_ {        // type of texture
  mjTEXTURE_2D        = 0,        // 2d texture, suitable for planes and hfields
  mjTEXTURE_CUBE,                 // cube texture, suitable for all other geom types
  mjTEXTURE_SKYBOX                // cube texture used as skybox
} mjtTexture;


typedef enum mjtIntegrator_ {     // integrator mode
  mjINT_EULER         = 0,        // semi-implicit Euler
  mjINT_RK4,                      // 4th-order Runge Kutta
  mjINT_IMPLICIT,                 // implicit in velocity
  mjINT_IMPLICITFAST              // implicit in velocity, no rne derivative
} mjtIntegrator;


typedef enum mjtCone_ {           // type of friction cone
  mjCONE_PYRAMIDAL     = 0,       // pyramidal
  mjCONE_ELLIPTIC                 // elliptic
} mjtCone;


typedef enum mjtJacobian_ {       // type of constraint Jacobian
  mjJAC_DENSE          = 0,       // dense
  mjJAC_SPARSE,                   // sparse
  mjJAC_AUTO                      // dense if nv<60, sparse otherwise
} mjtJacobian;


typedef enum mjtSolver_ {         // constraint solver algorithm
  mjSOL_PGS            = 0,       // PGS    (dual)
  mjSOL_CG,                       // CG     (primal)
  mjSOL_NEWTON                    // Newton (primal)
} mjtSolver;


typedef enum mjtEq_ {             // type of equality constraint
  mjEQ_CONNECT        = 0,        // connect two bodies at a point (ball joint)
  mjEQ_WELD,                      // fix relative position and orientation of two bodies
  mjEQ_JOINT,                     // couple the values of two scalar joints with cubic
  mjEQ_TENDON,                    // couple the lengths of two tendons with cubic
  mjEQ_FLEX,                      // fix all edge lengths of a flex
  mjEQ_DISTANCE                   // unsupported, will cause an error if used
} mjtEq;


typedef enum mjtWrap_ {           // type of tendon wrap object
  mjWRAP_NONE         = 0,        // null object
  mjWRAP_JOINT,                   // constant moment arm
  mjWRAP_PULLEY,                  // pulley used to split tendon
  mjWRAP_SITE,                    // pass through site
  mjWRAP_SPHERE,                  // wrap around sphere
  mjWRAP_CYLINDER                 // wrap around (infinite) cylinder
} mjtWrap;


typedef enum mjtTrn_ {            // type of actuator transmission
  mjTRN_JOINT         = 0,        // force on joint
  mjTRN_JOINTINPARENT,            // force on joint, expressed in parent frame
  mjTRN_SLIDERCRANK,              // force via slider-crank linkage
  mjTRN_TENDON,                   // force on tendon
  mjTRN_SITE,                     // force on site
  mjTRN_BODY,                     // adhesion force on a body's geoms

  mjTRN_UNDEFINED     = 1000      // undefined transmission type
} mjtTrn;


typedef enum mjtDyn_ {            // type of actuator dynamics
  mjDYN_NONE          = 0,        // no internal dynamics; ctrl specifies force
  mjDYN_INTEGRATOR,               // integrator: da/dt = u
  mjDYN_FILTER,                   // linear filter: da/dt = (u-a) / tau
  mjDYN_FILTEREXACT,              // linear filter: da/dt = (u-a) / tau, with exact integration
  mjDYN_MUSCLE,                   // piece-wise linear filter with two time constants
  mjDYN_USER                      // user-defined dynamics type
} mjtDyn;


typedef enum mjtGain_ {           // type of actuator gain
  mjGAIN_FIXED        = 0,        // fixed gain
  mjGAIN_AFFINE,                  // const + kp*length + kv*velocity
  mjGAIN_MUSCLE,                  // muscle FLV curve computed by mju_muscleGain()
  mjGAIN_USER                     // user-defined gain type
} mjtGain;


typedef enum mjtBias_ {           // type of actuator bias
  mjBIAS_NONE         = 0,        // no bias
  mjBIAS_AFFINE,                  // const + kp*length + kv*velocity
  mjBIAS_MUSCLE,                  // muscle passive force computed by mju_muscleBias()
  mjBIAS_USER                     // user-defined bias type
} mjtBias;


typedef enum mjtObj_ {            // type of MujoCo object
  mjOBJ_UNKNOWN       = 0,        // unknown object type
  mjOBJ_BODY,                     // body
  mjOBJ_XBODY,                    // body, used to access regular frame instead of i-frame
  mjOBJ_JOINT,                    // joint
  mjOBJ_DOF,                      // dof
  mjOBJ_GEOM,                     // geom
  mjOBJ_SITE,                     // site
  mjOBJ_CAMERA,                   // camera
  mjOBJ_LIGHT,                    // light
  mjOBJ_FLEX,                     // flex
  mjOBJ_MESH,                     // mesh
  mjOBJ_SKIN,                     // skin
  mjOBJ_HFIELD,                   // heightfield
  mjOBJ_TEXTURE,                  // texture
  mjOBJ_MATERIAL,                 // material for rendering
  mjOBJ_PAIR,                     // geom pair to include
  mjOBJ_EXCLUDE,                  // body pair to exclude
  mjOBJ_EQUALITY,                 // equality constraint
  mjOBJ_TENDON,                   // tendon
  mjOBJ_ACTUATOR,                 // actuator
  mjOBJ_SENSOR,                   // sensor
  mjOBJ_NUMERIC,                  // numeric
  mjOBJ_TEXT,                     // text
  mjOBJ_TUPLE,                    // tuple
  mjOBJ_KEY,                      // keyframe
  mjOBJ_PLUGIN,                   // plugin instance

  mjNOBJECT                       // number of object types
} mjtObj;


typedef enum mjtConstraint_ {     // type of constraint
  mjCNSTR_EQUALITY    = 0,        // equality constraint
  mjCNSTR_FRICTION_DOF,           // dof friction
  mjCNSTR_FRICTION_TENDON,        // tendon friction
  mjCNSTR_LIMIT_JOINT,            // joint limit
  mjCNSTR_LIMIT_TENDON,           // tendon limit
  mjCNSTR_CONTACT_FRICTIONLESS,   // frictionless contact
  mjCNSTR_CONTACT_PYRAMIDAL,      // frictional contact, pyramidal friction cone
  mjCNSTR_CONTACT_ELLIPTIC        // frictional contact, elliptic friction cone
} mjtConstraint;


typedef enum mjtConstraintState_ {  // constraint state
  mjCNSTRSTATE_SATISFIED = 0,       // constraint satisfied, zero cost (limit, contact)
  mjCNSTRSTATE_QUADRATIC,           // quadratic cost (equality, friction, limit, contact)
  mjCNSTRSTATE_LINEARNEG,           // linear cost, negative side (friction)
  mjCNSTRSTATE_LINEARPOS,           // linear cost, positive side (friction)
  mjCNSTRSTATE_CONE                 // squared distance to cone cost (elliptic contact)
} mjtConstraintState;


typedef enum mjtSensor_ {         // type of sensor
  // common robotic sensors, attached to a site
  mjSENS_TOUCH        = 0,        // scalar contact normal forces summed over sensor zone
  mjSENS_ACCELEROMETER,           // 3D linear acceleration, in local frame
  mjSENS_VELOCIMETER,             // 3D linear velocity, in local frame
  mjSENS_GYRO,                    // 3D angular velocity, in local frame
  mjSENS_FORCE,                   // 3D force between site's body and its parent body
  mjSENS_TORQUE,                  // 3D torque between site's body and its parent body
  mjSENS_MAGNETOMETER,            // 3D magnetometer
  mjSENS_RANGEFINDER,             // scalar distance to nearest geom or site along z-axis
  mjSENS_CAMPROJECTION,           // pixel coordinates of a site in the camera image

  // sensors related to scalar joints, tendons, actuators
  mjSENS_JOINTPOS,                // scalar joint position (hinge and slide only)
  mjSENS_JOINTVEL,                // scalar joint velocity (hinge and slide only)
  mjSENS_TENDONPOS,               // scalar tendon position
  mjSENS_TENDONVEL,               // scalar tendon velocity
  mjSENS_ACTUATORPOS,             // scalar actuator position
  mjSENS_ACTUATORVEL,             // scalar actuator velocity
  mjSENS_ACTUATORFRC,             // scalar actuator force
  mjSENS_JOINTACTFRC,             // scalar actuator force, measured at the joint

  // sensors related to ball joints
  mjSENS_BALLQUAT,                // 4D ball joint quaternion
  mjSENS_BALLANGVEL,              // 3D ball joint angular velocity

  // joint and tendon limit sensors, in constraint space
  mjSENS_JOINTLIMITPOS,           // joint limit distance-margin
  mjSENS_JOINTLIMITVEL,           // joint limit velocity
  mjSENS_JOINTLIMITFRC,           // joint limit force
  mjSENS_TENDONLIMITPOS,          // tendon limit distance-margin
  mjSENS_TENDONLIMITVEL,          // tendon limit velocity
  mjSENS_TENDONLIMITFRC,          // tendon limit force

  // sensors attached to an object with spatial frame: (x)body, geom, site, camera
  mjSENS_FRAMEPOS,                // 3D position
  mjSENS_FRAMEQUAT,               // 4D unit quaternion orientation
  mjSENS_FRAMEXAXIS,              // 3D unit vector: x-axis of object's frame
  mjSENS_FRAMEYAXIS,              // 3D unit vector: y-axis of object's frame
  mjSENS_FRAMEZAXIS,              // 3D unit vector: z-axis of object's frame
  mjSENS_FRAMELINVEL,             // 3D linear velocity
  mjSENS_FRAMEANGVEL,             // 3D angular velocity
  mjSENS_FRAMELINACC,             // 3D linear acceleration
  mjSENS_FRAMEANGACC,             // 3D angular acceleration

  // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
  mjSENS_SUBTREECOM,              // 3D center of mass of subtree
  mjSENS_SUBTREELINVEL,           // 3D linear velocity of subtree
  mjSENS_SUBTREEANGMOM,           // 3D angular momentum of subtree

  // global sensors
  mjSENS_CLOCK,                   // simulation time

  // plugin-controlled sensors
  mjSENS_PLUGIN,                  // plugin-controlled

  // user-defined sensor
  mjSENS_USER                     // sensor data provided by mjcb_sensor callback
} mjtSensor;


typedef enum mjtStage_ {          // computation stage
  mjSTAGE_NONE        = 0,        // no computations
  mjSTAGE_POS,                    // position-dependent computations
  mjSTAGE_VEL,                    // velocity-dependent computations
  mjSTAGE_ACC                     // acceleration/force-dependent computations
} mjtStage;


typedef enum mjtDataType_ {       // data type for sensors
  mjDATATYPE_REAL     = 0,        // real values, no constraints
  mjDATATYPE_POSITIVE,            // positive values; 0 or negative: inactive
  mjDATATYPE_AXIS,                // 3D unit vector
  mjDATATYPE_QUATERNION           // unit quaternion
} mjtDataType;


typedef enum mjtLRMode_ {         // mode for actuator length range computation
  mjLRMODE_NONE   = 0,            // do not process any actuators
  mjLRMODE_MUSCLE,                // process muscle actuators
  mjLRMODE_MUSCLEUSER,            // process muscle and user actuators
  mjLRMODE_ALL                    // process all actuators
} mjtLRMode;


typedef enum mjtFlexSelf_ {       // mode for flex selfcollide
  mjFLEXSELF_NONE   = 0,          // no self-collisions
  mjFLEXSELF_NARROW,              // skip midphase, go directly to narrowphase
  mjFLEXSELF_BVH,                 // use BVH in midphase (if midphase enabled)
  mjFLEXSELF_SAP,                 // use SAP in midphase
  mjFLEXSELF_AUTO                 // choose between BVH and SAP automatically
} mjtFlexSelf;


//---------------------------------- mjLROpt -------------------------------------------------------

struct mjLROpt_ {                 // options for mj_setLengthRange()
  // flags
  int mode;                       // which actuators to process (mjtLRMode)
  int useexisting;                // use existing length range if available
  int uselimit;                   // use joint and tendon limits if available

  // algorithm parameters
  mjtNum accel;                   // target acceleration used to compute force
  mjtNum maxforce;                // maximum force; 0: no limit
  mjtNum timeconst;               // time constant for velocity reduction; min 0.01
  mjtNum timestep;                // simulation timestep; 0: use mjOption.timestep
  mjtNum inttotal;                // total simulation time interval
  mjtNum interval;                // evaluation time interval (at the end)
  mjtNum tolrange;                // convergence tolerance (relative to range)
};
typedef struct mjLROpt_ mjLROpt;


//---------------------------------- mjVFS ---------------------------------------------------------

struct mjVFS_ {                             // virtual file system for loading from memory
  int    nfile;                             // number of files present
  char   filename[mjMAXVFS][mjMAXVFSNAME];  // file name without path
  size_t filesize[mjMAXVFS];                // file size in bytes
  void*  filedata[mjMAXVFS];                // buffer with file data
};
typedef struct mjVFS_ mjVFS;

//---------------------------------- mjOption ------------------------------------------------------

struct mjOption_ {                // physics options
  // timing parameters
  mjtNum timestep;                // timestep
  mjtNum apirate;                 // update rate for remote API (Hz)

  // solver parameters
  mjtNum impratio;                // ratio of friction-to-normal contact impedance
  mjtNum tolerance;               // main solver tolerance
  mjtNum ls_tolerance;            // CG/Newton linesearch tolerance
  mjtNum noslip_tolerance;        // noslip solver tolerance
  mjtNum mpr_tolerance;           // MPR solver tolerance

  // physical constants
  mjtNum gravity[3];              // gravitational acceleration
  mjtNum wind[3];                 // wind (for lift, drag and viscosity)
  mjtNum magnetic[3];             // global magnetic flux
  mjtNum density;                 // density of medium
  mjtNum viscosity;               // viscosity of medium

  // override contact solver parameters (if enabled)
  mjtNum o_margin;                // margin
  mjtNum o_solref[mjNREF];        // solref
  mjtNum o_solimp[mjNIMP];        // solimp
  mjtNum o_friction[5];           // friction

  // discrete settings
  int integrator;                 // integration mode (mjtIntegrator)
  int cone;                       // type of friction cone (mjtCone)
  int jacobian;                   // type of Jacobian (mjtJacobian)
  int solver;                     // solver algorithm (mjtSolver)
  int iterations;                 // maximum number of main solver iterations
  int ls_iterations;              // maximum number of CG/Newton linesearch iterations
  int noslip_iterations;          // maximum number of noslip solver iterations
  int mpr_iterations;             // maximum number of MPR solver iterations
  int disableflags;               // bit flags for disabling standard features
  int enableflags;                // bit flags for enabling optional features
  int disableactuator;            // bit flags for disabling actuators by group id

  // sdf collision settings
  int sdf_initpoints;             // number of starting points for gradient descent
  int sdf_iterations;             // max number of iterations for gradient descent
};
typedef struct mjOption_ mjOption;


//---------------------------------- mjVisual ------------------------------------------------------

struct mjVisual_ {                // visualization options
  struct {                        // global parameters
    float fovy;                   // y-field of view for free camera (degrees)
    float ipd;                    // inter-pupilary distance for free camera
    float azimuth;                // initial azimuth of free camera (degrees)
    float elevation;              // initial elevation of free camera (degrees)
    float linewidth;              // line width for wireframe and ray rendering
    float glow;                   // glow coefficient for selected body
    float realtime;               // initial real-time factor (1: real time)
    int   offwidth;               // width of offscreen buffer
    int   offheight;              // height of offscreen buffer
    int   ellipsoidinertia;       // geom for inertia visualization (0: box, 1: ellipsoid)
  } global;

  struct {                        // rendering quality
    int   shadowsize;             // size of shadowmap texture
    int   offsamples;             // number of multisamples for offscreen rendering
    int   numslices;              // number of slices for builtin geom drawing
    int   numstacks;              // number of stacks for builtin geom drawing
    int   numquads;               // number of quads for box rendering
  } quality;

  struct {                        // head light
    float ambient[3];             // ambient rgb (alpha=1)
    float diffuse[3];             // diffuse rgb (alpha=1)
    float specular[3];            // specular rgb (alpha=1)
    int   active;                 // is headlight active
  } headlight;

  struct {                        // mapping
    float stiffness;              // mouse perturbation stiffness (space->force)
    float stiffnessrot;           // mouse perturbation stiffness (space->torque)
    float force;                  // from force units to space units
    float torque;                 // from torque units to space units
    float alpha;                  // scale geom alphas when transparency is enabled
    float fogstart;               // OpenGL fog starts at fogstart * mjModel.stat.extent
    float fogend;                 // OpenGL fog ends at fogend * mjModel.stat.extent
    float znear;                  // near clipping plane = znear * mjModel.stat.extent
    float zfar;                   // far clipping plane = zfar * mjModel.stat.extent
    float haze;                   // haze ratio
    float shadowclip;             // directional light: shadowclip * mjModel.stat.extent
    float shadowscale;            // spot light: shadowscale * light.cutoff
    float actuatortendon;         // scale tendon width
  } map;

  struct {                        // scale of decor elements relative to mean body size
    float forcewidth;             // width of force arrow
    float contactwidth;           // contact width
    float contactheight;          // contact height
    float connect;                // autoconnect capsule width
    float com;                    // com radius
    float camera;                 // camera object
    float light;                  // light object
    float selectpoint;            // selection point
    float jointlength;            // joint length
    float jointwidth;             // joint width
    float actuatorlength;         // actuator length
    float actuatorwidth;          // actuator width
    float framelength;            // bodyframe axis length
    float framewidth;             // bodyframe axis width
    float constraint;             // constraint width
    float slidercrank;            // slidercrank width
    float frustum;                // frustum zfar plane
  } scale;

  struct {                        // color of decor elements
    float fog[4];                 // fog
    float haze[4];                // haze
    float force[4];               // external force
    float inertia[4];             // inertia box
    float joint[4];               // joint
    float actuator[4];            // actuator, neutral
    float actuatornegative[4];    // actuator, negative limit
    float actuatorpositive[4];    // actuator, positive limit
    float com[4];                 // center of mass
    float camera[4];              // camera object
    float light[4];               // light object
    float selectpoint[4];         // selection point
    float connect[4];             // auto connect
    float contactpoint[4];        // contact point
    float contactforce[4];        // contact force
    float contactfriction[4];     // contact friction force
    float contacttorque[4];       // contact torque
    float contactgap[4];          // contact point in gap
    float rangefinder[4];         // rangefinder ray
    float constraint[4];          // constraint
    float slidercrank[4];         // slidercrank
    float crankbroken[4];         // used when crank must be stretched/broken
    float frustum[4];             // camera frustum
  } rgba;
};
typedef struct mjVisual_ mjVisual;


//---------------------------------- mjStatistic ---------------------------------------------------

struct mjStatistic_ {             // model statistics (in qpos0)
  mjtNum meaninertia;             // mean diagonal inertia
  mjtNum meanmass;                // mean body mass
  mjtNum meansize;                // mean body size
  mjtNum extent;                  // spatial extent
  mjtNum center[3];               // center of model
};
typedef struct mjStatistic_ mjStatistic;


//---------------------------------- mjModel -------------------------------------------------------

struct mjModel_ {
  // ------------------------------- sizes

  // sizes needed at mjModel construction
  int nq;                         // number of generalized coordinates = dim(qpos)
  int nv;                         // number of degrees of freedom = dim(qvel)
  int nu;                         // number of actuators/controls = dim(ctrl)
  int na;                         // number of activation states = dim(act)
  int nbody;                      // number of bodies
  int nbvh;                       // number of total bounding volumes in all bodies
  int nbvhstatic;                 // number of static bounding volumes (aabb stored in mjModel)
  int nbvhdynamic;                // number of dynamic bounding volumes (aabb stored in mjData)
  int njnt;                       // number of joints
  int ngeom;                      // number of geoms
  int nsite;                      // number of sites
  int ncam;                       // number of cameras
  int nlight;                     // number of lights
  int nflex;                      // number of flexes
  int nflexvert;                  // number of vertices in all flexes
  int nflexedge;                  // number of edges in all flexes
  int nflexelem;                  // number of elements in all flexes
  int nflexelemdata;              // number of element vertex ids in all flexes
  int nflexshelldata;             // number of shell fragment vertex ids in all flexes
  int nflexevpair;                // number of element-vertex pairs in all flexes
  int nflextexcoord;              // number of vertices with texture coordinates
  int nmesh;                      // number of meshes
  int nmeshvert;                  // number of vertices in all meshes
  int nmeshnormal;                // number of normals in all meshes
  int nmeshtexcoord;              // number of texcoords in all meshes
  int nmeshface;                  // number of triangular faces in all meshes
  int nmeshgraph;                 // number of ints in mesh auxiliary data
  int nskin;                      // number of skins
  int nskinvert;                  // number of vertices in all skins
  int nskintexvert;               // number of vertiex with texcoords in all skins
  int nskinface;                  // number of triangular faces in all skins
  int nskinbone;                  // number of bones in all skins
  int nskinbonevert;              // number of vertices in all skin bones
  int nhfield;                    // number of heightfields
  int nhfielddata;                // number of data points in all heightfields
  int ntex;                       // number of textures
  int ntexdata;                   // number of bytes in texture rgb data
  int nmat;                       // number of materials
  int npair;                      // number of predefined geom pairs
  int nexclude;                   // number of excluded geom pairs
  int neq;                        // number of equality constraints
  int ntendon;                    // number of tendons
  int nwrap;                      // number of wrap objects in all tendon paths
  int nsensor;                    // number of sensors
  int nnumeric;                   // number of numeric custom fields
  int nnumericdata;               // number of mjtNums in all numeric fields
  int ntext;                      // number of text custom fields
  int ntextdata;                  // number of mjtBytes in all text fields
  int ntuple;                     // number of tuple custom fields
  int ntupledata;                 // number of objects in all tuple fields
  int nkey;                       // number of keyframes
  int nmocap;                     // number of mocap bodies
  int nplugin;                    // number of plugin instances
  int npluginattr;                // number of chars in all plugin config attributes
  int nuser_body;                 // number of mjtNums in body_user
  int nuser_jnt;                  // number of mjtNums in jnt_user
  int nuser_geom;                 // number of mjtNums in geom_user
  int nuser_site;                 // number of mjtNums in site_user
  int nuser_cam;                  // number of mjtNums in cam_user
  int nuser_tendon;               // number of mjtNums in tendon_user
  int nuser_actuator;             // number of mjtNums in actuator_user
  int nuser_sensor;               // number of mjtNums in sensor_user
  int nnames;                     // number of chars in all names
  int nnames_map;                 // number of slots in the names hash map
  int npaths;                     // number of chars in all paths

  // sizes set after mjModel construction (only affect mjData)
  int nM;                         // number of non-zeros in sparse inertia matrix
  int nD;                         // number of non-zeros in sparse dof-dof matrix
  int nB;                         // number of non-zeros in sparse body-dof matrix
  int ntree;                      // number of kinematic trees under world body
  int nemax;                      // number of potential equality-constraint rows
  int njmax;                      // number of available rows in constraint Jacobian
  int nconmax;                    // number of potential contacts in contact list
  int nuserdata;                  // number of extra fields in mjData
  int nsensordata;                // number of fields in sensor data vector
  int npluginstate;               // number of fields in plugin state vector

  size_t narena;                  // number of bytes in the mjData arena (inclusive of stack)
  size_t nbuffer;                 // number of bytes in buffer

  // ------------------------------- options and statistics

  mjOption opt;                   // physics options
  mjVisual vis;                   // visualization options
  mjStatistic stat;               // model statistics

  // ------------------------------- buffers

  // main buffer
  void*     buffer;               // main buffer; all pointers point in it    (nbuffer)

  // default generalized coordinates
  mjtNum*   qpos0;                // qpos values at default pose              (nq x 1)
  mjtNum*   qpos_spring;          // reference pose for springs               (nq x 1)

  // bodies
  int*      body_parentid;        // id of body's parent                      (nbody x 1)
  int*      body_rootid;          // id of root above body                    (nbody x 1)
  int*      body_weldid;          // id of body that this body is welded to   (nbody x 1)
  int*      body_mocapid;         // id of mocap data; -1: none               (nbody x 1)
  int*      body_jntnum;          // number of joints for this body           (nbody x 1)
  int*      body_jntadr;          // start addr of joints; -1: no joints      (nbody x 1)
  int*      body_dofnum;          // number of motion degrees of freedom      (nbody x 1)
  int*      body_dofadr;          // start addr of dofs; -1: no dofs          (nbody x 1)
  int*      body_treeid;          // id of body's kinematic tree; -1: static  (nbody x 1)
  int*      body_geomnum;         // number of geoms                          (nbody x 1)
  int*      body_geomadr;         // start addr of geoms; -1: no geoms        (nbody x 1)
  mjtByte*  body_simple;          // 1: diag M; 2: diag M, sliders only       (nbody x 1)
  mjtByte*  body_sameframe;       // inertial frame is same as body frame     (nbody x 1)
  mjtNum*   body_pos;             // position offset rel. to parent body      (nbody x 3)
  mjtNum*   body_quat;            // orientation offset rel. to parent body   (nbody x 4)
  mjtNum*   body_ipos;            // local position of center of mass         (nbody x 3)
  mjtNum*   body_iquat;           // local orientation of inertia ellipsoid   (nbody x 4)
  mjtNum*   body_mass;            // mass                                     (nbody x 1)
  mjtNum*   body_subtreemass;     // mass of subtree starting at this body    (nbody x 1)
  mjtNum*   body_inertia;         // diagonal inertia in ipos/iquat frame     (nbody x 3)
  mjtNum*   body_invweight0;      // mean inv inert in qpos0 (trn, rot)       (nbody x 2)
  mjtNum*   body_gravcomp;        // antigravity force, units of body weight  (nbody x 1)
  mjtNum*   body_margin;          // MAX over all geom margins                (nbody x 1)
  mjtNum*   body_user;            // user data                                (nbody x nuser_body)
  int*      body_plugin;          // plugin instance id; -1: not in use       (nbody x 1)
  int*      body_contype;         // OR over all geom contypes                (nbody x 1)
  int*      body_conaffinity;     // OR over all geom conaffinities           (nbody x 1)
  int*      body_bvhadr;          // address of bvh root                      (nbody x 1)
  int*      body_bvhnum;          // number of bounding volumes               (nbody x 1)

  // bounding volume hierarchy
  int*      bvh_depth;            // depth in the bounding volume hierarchy   (nbvh x 1)
  int*      bvh_child;            // left and right children in tree          (nbvh x 2)
  int*      bvh_nodeid;           // geom or elem id of node; -1: non-leaf    (nbvh x 1)
  mjtNum*   bvh_aabb;             // local bounding box (center, size)        (nbvhstatic x 6)

  // joints
  int*      jnt_type;             // type of joint (mjtJoint)                 (njnt x 1)
  int*      jnt_qposadr;          // start addr in 'qpos' for joint's data    (njnt x 1)
  int*      jnt_dofadr;           // start addr in 'qvel' for joint's data    (njnt x 1)
  int*      jnt_bodyid;           // id of joint's body                       (njnt x 1)
  int*      jnt_group;            // group for visibility                     (njnt x 1)
  mjtByte*  jnt_limited;          // does joint have limits                   (njnt x 1)
  mjtByte*  jnt_actfrclimited;    // does joint have actuator force limits    (njnt x 1)
  mjtNum*   jnt_solref;           // constraint solver reference: limit       (njnt x mjNREF)
  mjtNum*   jnt_solimp;           // constraint solver impedance: limit       (njnt x mjNIMP)
  mjtNum*   jnt_pos;              // local anchor position                    (njnt x 3)
  mjtNum*   jnt_axis;             // local joint axis                         (njnt x 3)
  mjtNum*   jnt_stiffness;        // stiffness coefficient                    (njnt x 1)
  mjtNum*   jnt_range;            // joint limits                             (njnt x 2)
  mjtNum*   jnt_actfrcrange;      // range of total actuator force            (njnt x 2)
  mjtNum*   jnt_margin;           // min distance for limit detection         (njnt x 1)
  mjtNum*   jnt_user;             // user data                                (njnt x nuser_jnt)

  // dofs
  int*      dof_bodyid;           // id of dof's body                         (nv x 1)
  int*      dof_jntid;            // id of dof's joint                        (nv x 1)
  int*      dof_parentid;         // id of dof's parent; -1: none             (nv x 1)
  int*      dof_treeid;           // id of dof's kinematic tree               (nv x 1)
  int*      dof_Madr;             // dof address in M-diagonal                (nv x 1)
  int*      dof_simplenum;        // number of consecutive simple dofs        (nv x 1)
  mjtNum*   dof_solref;           // constraint solver reference:frictionloss (nv x mjNREF)
  mjtNum*   dof_solimp;           // constraint solver impedance:frictionloss (nv x mjNIMP)
  mjtNum*   dof_frictionloss;     // dof friction loss                        (nv x 1)
  mjtNum*   dof_armature;         // dof armature inertia/mass                (nv x 1)
  mjtNum*   dof_damping;          // damping coefficient                      (nv x 1)
  mjtNum*   dof_invweight0;       // diag. inverse inertia in qpos0           (nv x 1)
  mjtNum*   dof_M0;               // diag. inertia in qpos0                   (nv x 1)

  // geoms
  int*      geom_type;            // geometric type (mjtGeom)                 (ngeom x 1)
  int*      geom_contype;         // geom contact type                        (ngeom x 1)
  int*      geom_conaffinity;     // geom contact affinity                    (ngeom x 1)
  int*      geom_condim;          // contact dimensionality (1, 3, 4, 6)      (ngeom x 1)
  int*      geom_bodyid;          // id of geom's body                        (ngeom x 1)
  int*      geom_dataid;          // id of geom's mesh/hfield; -1: none       (ngeom x 1)
  int*      geom_matid;           // material id for rendering; -1: none      (ngeom x 1)
  int*      geom_group;           // group for visibility                     (ngeom x 1)
  int*      geom_priority;        // geom contact priority                    (ngeom x 1)
  int*      geom_plugin;          // plugin instance id; -1: not in use       (ngeom x 1)
  mjtByte*  geom_sameframe;       // same as body frame (1) or iframe (2)     (ngeom x 1)
  mjtNum*   geom_solmix;          // mixing coef for solref/imp in geom pair  (ngeom x 1)
  mjtNum*   geom_solref;          // constraint solver reference: contact     (ngeom x mjNREF)
  mjtNum*   geom_solimp;          // constraint solver impedance: contact     (ngeom x mjNIMP)
  mjtNum*   geom_size;            // geom-specific size parameters            (ngeom x 3)
  mjtNum*   geom_aabb;            // bounding box, (center, size)             (ngeom x 6)
  mjtNum*   geom_rbound;          // radius of bounding sphere                (ngeom x 1)
  mjtNum*   geom_pos;             // local position offset rel. to body       (ngeom x 3)
  mjtNum*   geom_quat;            // local orientation offset rel. to body    (ngeom x 4)
  mjtNum*   geom_friction;        // friction for (slide, spin, roll)         (ngeom x 3)
  mjtNum*   geom_margin;          // detect contact if dist<margin            (ngeom x 1)
  mjtNum*   geom_gap;             // include in solver if dist<margin-gap     (ngeom x 1)
  mjtNum*   geom_fluid;           // fluid interaction parameters             (ngeom x mjNFLUID)
  mjtNum*   geom_user;            // user data                                (ngeom x nuser_geom)
  float*    geom_rgba;            // rgba when material is omitted            (ngeom x 4)

  // sites
  int*      site_type;            // geom type for rendering (mjtGeom)        (nsite x 1)
  int*      site_bodyid;          // id of site's body                        (nsite x 1)
  int*      site_matid;           // material id for rendering; -1: none      (nsite x 1)
  int*      site_group;           // group for visibility                     (nsite x 1)
  mjtByte*  site_sameframe;       // same as body frame (1) or iframe (2)     (nsite x 1)
  mjtNum*   site_size;            // geom size for rendering                  (nsite x 3)
  mjtNum*   site_pos;             // local position offset rel. to body       (nsite x 3)
  mjtNum*   site_quat;            // local orientation offset rel. to body    (nsite x 4)
  mjtNum*   site_user;            // user data                                (nsite x nuser_site)
  float*    site_rgba;            // rgba when material is omitted            (nsite x 4)

  // cameras
  int*      cam_mode;             // camera tracking mode (mjtCamLight)       (ncam x 1)
  int*      cam_bodyid;           // id of camera's body                      (ncam x 1)
  int*      cam_targetbodyid;     // id of targeted body; -1: none            (ncam x 1)
  mjtNum*   cam_pos;              // position rel. to body frame              (ncam x 3)
  mjtNum*   cam_quat;             // orientation rel. to body frame           (ncam x 4)
  mjtNum*   cam_poscom0;          // global position rel. to sub-com in qpos0 (ncam x 3)
  mjtNum*   cam_pos0;             // global position rel. to body in qpos0    (ncam x 3)
  mjtNum*   cam_mat0;             // global orientation in qpos0              (ncam x 9)
  int*      cam_resolution;       // [width, height] in pixels                (ncam x 2)
  mjtNum*   cam_fovy;             // y-field of view (deg)                    (ncam x 1)
  float*    cam_intrinsic;        // [focal length; principal point]          (ncam x 4)
  float*    cam_sensorsize;       // sensor size                              (ncam x 2)
  mjtNum*   cam_ipd;              // inter-pupilary distance                  (ncam x 1)
  mjtNum*   cam_user;             // user data                                (ncam x nuser_cam)

  // lights
  int*      light_mode;           // light tracking mode (mjtCamLight)        (nlight x 1)
  int*      light_bodyid;         // id of light's body                       (nlight x 1)
  int*      light_targetbodyid;   // id of targeted body; -1: none            (nlight x 1)
  mjtByte*  light_directional;    // directional light                        (nlight x 1)
  mjtByte*  light_castshadow;     // does light cast shadows                  (nlight x 1)
  mjtByte*  light_active;         // is light on                              (nlight x 1)
  mjtNum*   light_pos;            // position rel. to body frame              (nlight x 3)
  mjtNum*   light_dir;            // direction rel. to body frame             (nlight x 3)
  mjtNum*   light_poscom0;        // global position rel. to sub-com in qpos0 (nlight x 3)
  mjtNum*   light_pos0;           // global position rel. to body in qpos0    (nlight x 3)
  mjtNum*   light_dir0;           // global direction in qpos0                (nlight x 3)
  float*    light_attenuation;    // OpenGL attenuation (quadratic model)     (nlight x 3)
  float*    light_cutoff;         // OpenGL cutoff                            (nlight x 1)
  float*    light_exponent;       // OpenGL exponent                          (nlight x 1)
  float*    light_ambient;        // ambient rgb (alpha=1)                    (nlight x 3)
  float*    light_diffuse;        // diffuse rgb (alpha=1)                    (nlight x 3)
  float*    light_specular;       // specular rgb (alpha=1)                   (nlight x 3)

  // flexes: contact properties
  int*      flex_contype;         // flex contact type                        (nflex x 1)
  int*      flex_conaffinity;     // flex contact affinity                    (nflex x 1)
  int*      flex_condim;          // contact dimensionality (1, 3, 4, 6)      (nflex x 1)
  int*      flex_priority;        // flex contact priority                    (nflex x 1)
  mjtNum*   flex_solmix;          // mix coef for solref/imp in contact pair  (nflex x 1)
  mjtNum*   flex_solref;          // constraint solver reference: contact     (nflex x mjNREF)
  mjtNum*   flex_solimp;          // constraint solver impedance: contact     (nflex x mjNIMP)
  mjtNum*   flex_friction;        // friction for (slide, spin, roll)         (nflex x 3)
  mjtNum*   flex_margin;          // detect contact if dist<margin            (nflex x 1)
  mjtNum*   flex_gap;             // include in solver if dist<margin-gap     (nflex x 1)
  mjtByte*  flex_internal;        // internal flex collision enabled          (nflex x 1)
  int*      flex_selfcollide;     // self collision mode (mjtFlexSelf)        (nflex x 1)
  int*      flex_activelayers;    // number of active element layers, 3D only (nflex x 1)

  // flexes: other properties
  int*      flex_dim;             // 1: lines, 2: triangles, 3: tetrahedra    (nflex x 1)
  int*      flex_matid;           // material id for rendering                (nflex x 1)
  int*      flex_group;           // group for visibility                     (nflex x 1)
  int*      flex_vertadr;         // first vertex address                     (nflex x 1)
  int*      flex_vertnum;         // number of vertices                       (nflex x 1)
  int*      flex_edgeadr;         // first edge address                       (nflex x 1)
  int*      flex_edgenum;         // number of edges                          (nflex x 1)
  int*      flex_elemadr;         // first element address                    (nflex x 1)
  int*      flex_elemnum;         // number of elements                       (nflex x 1)
  int*      flex_elemdataadr;     // first element vertex id address          (nflex x 1)
  int*      flex_shellnum;        // number of shells                         (nflex x 1)
  int*      flex_shelldataadr;    // first shell data address                 (nflex x 1)
  int*      flex_evpairadr;       // first evpair address                     (nflex x 1)
  int*      flex_evpairnum;       // number of evpairs                        (nflex x 1)
  int*      flex_texcoordadr;     // address in flex_texcoord; -1: none       (nflex x 1)
  int*      flex_vertbodyid;      // vertex body ids                          (nflexvert x 1)
  int*      flex_edge;            // edge vertex ids (2 per edge)             (nflexedge x 2)
  int*      flex_elem;            // element vertex ids (dim+1 per elem)      (nflexelemdata x 1)
  int*      flex_elemlayer;       // element distance from surface, 3D only   (nflexelem x 1)
  int*      flex_shell;           // shell fragment vertex ids (dim per frag) (nflexshelldata x 1)
  int*      flex_evpair;          // (element, vertex) collision pairs        (nflexevpair x 2)
  mjtNum*   flex_vert;            // vertex positions in local body frames    (nflexvert x 3)
  mjtNum*   flex_xvert0;          // Cartesian vertex positions in qpos0      (nflexvert x 3)
  mjtNum*   flexedge_length0;     // edge lengths in qpos0                    (nflexedge x 1)
  mjtNum*   flexedge_invweight0;  // edge inv. weight in qpos0                (nflexedge x 1)
  mjtNum*   flex_radius;          // radius around primitive element          (nflex x 1)
  mjtNum*   flex_edgestiffness;   // edge stiffness                           (nflex x 1)
  mjtNum*   flex_edgedamping;     // edge damping                             (nflex x 1)
  mjtByte*  flex_edgeequality;    // is edge equality constraint defined      (nflex x 1)
  mjtByte*  flex_rigid;           // are all verices in the same body         (nflex x 1)
  mjtByte*  flexedge_rigid;       // are both edge vertices in same body      (nflexedge x 1)
  mjtByte*  flex_centered;        // are all vertex coordinates (0,0,0)       (nflex x 1)
  mjtByte*  flex_flatskin;        // render flex skin with flat shading       (nflex x 1)
  int*      flex_bvhadr;          // address of bvh root; -1: no bvh          (nflex x 1)
  int*      flex_bvhnum;          // number of bounding volumes               (nflex x 1)
  float*    flex_rgba;            // rgba when material is omitted            (nflex x 4)
  float*    flex_texcoord;        // vertex texture coordinates               (nflextexcoord x 2)

  // meshes
  int*      mesh_vertadr;         // first vertex address                     (nmesh x 1)
  int*      mesh_vertnum;         // number of vertices                       (nmesh x 1)
  int*      mesh_faceadr;         // first face address                       (nmesh x 1)
  int*      mesh_facenum;         // number of faces                          (nmesh x 1)
  int*      mesh_bvhadr;          // address of bvh root                      (nmesh x 1)
  int*      mesh_bvhnum;          // number of bvh                            (nmesh x 1)
  int*      mesh_normaladr;       // first normal address                     (nmesh x 1)
  int*      mesh_normalnum;       // number of normals                        (nmesh x 1)
  int*      mesh_texcoordadr;     // texcoord data address; -1: no texcoord   (nmesh x 1)
  int*      mesh_texcoordnum;     // number of texcoord                       (nmesh x 1)
  int*      mesh_graphadr;        // graph data address; -1: no graph         (nmesh x 1)
  mjtNum*   mesh_pos;             // translation applied to asset vertices    (nmesh x 3)
  mjtNum*   mesh_quat;            // rotation applied to asset vertices       (nmesh x 4)
  float*    mesh_vert;            // vertex positions for all meshes          (nmeshvert x 3)
  float*    mesh_normal;          // normals for all meshes                   (nmeshnormal x 3)
  float*    mesh_texcoord;        // vertex texcoords for all meshes          (nmeshtexcoord x 2)
  int*      mesh_face;            // vertex face data                         (nmeshface x 3)
  int*      mesh_facenormal;      // normal face data                         (nmeshface x 3)
  int*      mesh_facetexcoord;    // texture face data                        (nmeshface x 3)
  int*      mesh_graph;           // convex graph data                        (nmeshgraph x 1)
  int*      mesh_pathadr;         // address of asset path for mesh; -1: none (nmesh x 1)

  // skins
  int*      skin_matid;           // skin material id; -1: none               (nskin x 1)
  int*      skin_group;           // group for visibility                     (nskin x 1)
  float*    skin_rgba;            // skin rgba                                (nskin x 4)
  float*    skin_inflate;         // inflate skin in normal direction         (nskin x 1)
  int*      skin_vertadr;         // first vertex address                     (nskin x 1)
  int*      skin_vertnum;         // number of vertices                       (nskin x 1)
  int*      skin_texcoordadr;     // texcoord data address; -1: no texcoord   (nskin x 1)
  int*      skin_faceadr;         // first face address                       (nskin x 1)
  int*      skin_facenum;         // number of faces                          (nskin x 1)
  int*      skin_boneadr;         // first bone in skin                       (nskin x 1)
  int*      skin_bonenum;         // number of bones in skin                  (nskin x 1)
  float*    skin_vert;            // vertex positions for all skin meshes     (nskinvert x 3)
  float*    skin_texcoord;        // vertex texcoords for all skin meshes     (nskintexvert x 2)
  int*      skin_face;            // triangle faces for all skin meshes       (nskinface x 3)
  int*      skin_bonevertadr;     // first vertex in each bone                (nskinbone x 1)
  int*      skin_bonevertnum;     // number of vertices in each bone          (nskinbone x 1)
  float*    skin_bonebindpos;     // bind pos of each bone                    (nskinbone x 3)
  float*    skin_bonebindquat;    // bind quat of each bone                   (nskinbone x 4)
  int*      skin_bonebodyid;      // body id of each bone                     (nskinbone x 1)
  int*      skin_bonevertid;      // mesh ids of vertices in each bone        (nskinbonevert x 1)
  float*    skin_bonevertweight;  // weights of vertices in each bone         (nskinbonevert x 1)
  int*      skin_pathadr;         // address of asset path for skin; -1: none (nskin x 1)

  // height fields
  mjtNum*   hfield_size;          // (x, y, z_top, z_bottom)                    (nhfield x 4)
  int*      hfield_nrow;          // number of rows in grid                     (nhfield x 1)
  int*      hfield_ncol;          // number of columns in grid                  (nhfield x 1)
  int*      hfield_adr;           // address in hfield_data                     (nhfield x 1)
  float*    hfield_data;          // elevation data                             (nhfielddata x 1)
  int*      hfield_pathadr;       // address of asset path for hfield; -1: none (nhfield x 1)

  // textures
  int*      tex_type;             // texture type (mjtTexture)                  (ntex x 1)
  int*      tex_height;           // number of rows in texture image            (ntex x 1)
  int*      tex_width;            // number of columns in texture image         (ntex x 1)
  int*      tex_adr;              // address in rgb                             (ntex x 1)
  mjtByte*  tex_rgb;              // rgb (alpha = 1)                            (ntexdata x 1)
  int*      tex_pathadr;         // address of asset path for texture; -1: none (ntex x 1)

  // materials
  int*      mat_texid;            // texture id; -1: none                     (nmat x 1)
  mjtByte*  mat_texuniform;       // make texture cube uniform                (nmat x 1)
  float*    mat_texrepeat;        // texture repetition for 2d mapping        (nmat x 2)
  float*    mat_emission;         // emission (x rgb)                         (nmat x 1)
  float*    mat_specular;         // specular (x white)                       (nmat x 1)
  float*    mat_shininess;        // shininess coef                           (nmat x 1)
  float*    mat_reflectance;      // reflectance (0: disable)                 (nmat x 1)
  float*    mat_rgba;             // rgba                                     (nmat x 4)

  // predefined geom pairs for collision detection; has precedence over exclude
  int*      pair_dim;             // contact dimensionality                   (npair x 1)
  int*      pair_geom1;           // id of geom1                              (npair x 1)
  int*      pair_geom2;           // id of geom2                              (npair x 1)
  int*      pair_signature;       // body1 << 16 + body2                      (npair x 1)
  mjtNum*   pair_solref;          // solver reference: contact normal         (npair x mjNREF)
  mjtNum*   pair_solreffriction;  // solver reference: contact friction       (npair x mjNREF)
  mjtNum*   pair_solimp;          // solver impedance: contact                (npair x mjNIMP)
  mjtNum*   pair_margin;          // detect contact if dist<margin            (npair x 1)
  mjtNum*   pair_gap;             // include in solver if dist<margin-gap     (npair x 1)
  mjtNum*   pair_friction;        // tangent1, 2, spin, roll1, 2              (npair x 5)

  // excluded body pairs for collision detection
  int*      exclude_signature;    // body1 << 16 + body2                      (nexclude x 1)

  // equality constraints
  int*      eq_type;              // constraint type (mjtEq)                  (neq x 1)
  int*      eq_obj1id;            // id of object 1                           (neq x 1)
  int*      eq_obj2id;            // id of object 2                           (neq x 1)
  mjtByte*  eq_active0;           // initial enable/disable constraint state  (neq x 1)
  mjtNum*   eq_solref;            // constraint solver reference              (neq x mjNREF)
  mjtNum*   eq_solimp;            // constraint solver impedance              (neq x mjNIMP)
  mjtNum*   eq_data;              // numeric data for constraint              (neq x mjNEQDATA)

  // tendons
  int*      tendon_adr;           // address of first object in tendon's path (ntendon x 1)
  int*      tendon_num;           // number of objects in tendon's path       (ntendon x 1)
  int*      tendon_matid;         // material id for rendering                (ntendon x 1)
  int*      tendon_group;         // group for visibility                     (ntendon x 1)
  mjtByte*  tendon_limited;       // does tendon have length limits           (ntendon x 1)
  mjtNum*   tendon_width;         // width for rendering                      (ntendon x 1)
  mjtNum*   tendon_solref_lim;    // constraint solver reference: limit       (ntendon x mjNREF)
  mjtNum*   tendon_solimp_lim;    // constraint solver impedance: limit       (ntendon x mjNIMP)
  mjtNum*   tendon_solref_fri;    // constraint solver reference: friction    (ntendon x mjNREF)
  mjtNum*   tendon_solimp_fri;    // constraint solver impedance: friction    (ntendon x mjNIMP)
  mjtNum*   tendon_range;         // tendon length limits                     (ntendon x 2)
  mjtNum*   tendon_margin;        // min distance for limit detection         (ntendon x 1)
  mjtNum*   tendon_stiffness;     // stiffness coefficient                    (ntendon x 1)
  mjtNum*   tendon_damping;       // damping coefficient                      (ntendon x 1)
  mjtNum*   tendon_frictionloss;  // loss due to friction                     (ntendon x 1)
  mjtNum*   tendon_lengthspring;  // spring resting length range              (ntendon x 2)
  mjtNum*   tendon_length0;       // tendon length in qpos0                   (ntendon x 1)
  mjtNum*   tendon_invweight0;    // inv. weight in qpos0                     (ntendon x 1)
  mjtNum*   tendon_user;          // user data                                (ntendon x nuser_tendon)
  float*    tendon_rgba;          // rgba when material is omitted            (ntendon x 4)

  // list of all wrap objects in tendon paths
  int*      wrap_type;            // wrap object type (mjtWrap)               (nwrap x 1)
  int*      wrap_objid;           // object id: geom, site, joint             (nwrap x 1)
  mjtNum*   wrap_prm;             // divisor, joint coef, or site id          (nwrap x 1)

  // actuators
  int*      actuator_trntype;     // transmission type (mjtTrn)               (nu x 1)
  int*      actuator_dyntype;     // dynamics type (mjtDyn)                   (nu x 1)
  int*      actuator_gaintype;    // gain type (mjtGain)                      (nu x 1)
  int*      actuator_biastype;    // bias type (mjtBias)                      (nu x 1)
  int*      actuator_trnid;       // transmission id: joint, tendon, site     (nu x 2)
  int*      actuator_actadr;      // first activation address; -1: stateless  (nu x 1)
  int*      actuator_actnum;      // number of activation variables           (nu x 1)
  int*      actuator_group;       // group for visibility                     (nu x 1)
  mjtByte*  actuator_ctrllimited; // is control limited                       (nu x 1)
  mjtByte*  actuator_forcelimited;// is force limited                         (nu x 1)
  mjtByte*  actuator_actlimited;  // is activation limited                    (nu x 1)
  mjtNum*   actuator_dynprm;      // dynamics parameters                      (nu x mjNDYN)
  mjtNum*   actuator_gainprm;     // gain parameters                          (nu x mjNGAIN)
  mjtNum*   actuator_biasprm;     // bias parameters                          (nu x mjNBIAS)
  mjtByte*  actuator_actearly;    // step activation before force             (nu x 1)
  mjtNum*   actuator_ctrlrange;   // range of controls                        (nu x 2)
  mjtNum*   actuator_forcerange;  // range of forces                          (nu x 2)
  mjtNum*   actuator_actrange;    // range of activations                     (nu x 2)
  mjtNum*   actuator_gear;        // scale length and transmitted force       (nu x 6)
  mjtNum*   actuator_cranklength; // crank length for slider-crank            (nu x 1)
  mjtNum*   actuator_acc0;        // acceleration from unit force in qpos0    (nu x 1)
  mjtNum*   actuator_length0;     // actuator length in qpos0                 (nu x 1)
  mjtNum*   actuator_lengthrange; // feasible actuator length range           (nu x 2)
  mjtNum*   actuator_user;        // user data                                (nu x nuser_actuator)
  int*      actuator_plugin;      // plugin instance id; -1: not a plugin     (nu x 1)

  // sensors
  int*      sensor_type;          // sensor type (mjtSensor)                  (nsensor x 1)
  int*      sensor_datatype;      // numeric data type (mjtDataType)          (nsensor x 1)
  int*      sensor_needstage;     // required compute stage (mjtStage)        (nsensor x 1)
  int*      sensor_objtype;       // type of sensorized object (mjtObj)       (nsensor x 1)
  int*      sensor_objid;         // id of sensorized object                  (nsensor x 1)
  int*      sensor_reftype;       // type of reference frame (mjtObj)         (nsensor x 1)
  int*      sensor_refid;         // id of reference frame; -1: global frame  (nsensor x 1)
  int*      sensor_dim;           // number of scalar outputs                 (nsensor x 1)
  int*      sensor_adr;           // address in sensor array                  (nsensor x 1)
  mjtNum*   sensor_cutoff;        // cutoff for real and positive; 0: ignore  (nsensor x 1)
  mjtNum*   sensor_noise;         // noise standard deviation                 (nsensor x 1)
  mjtNum*   sensor_user;          // user data                                (nsensor x nuser_sensor)
  int*      sensor_plugin;        // plugin instance id; -1: not a plugin     (nsensor x 1)

  // plugin instances
  int*      plugin;               // globally registered plugin slot number   (nplugin x 1)
  int*      plugin_stateadr;      // address in the plugin state array        (nplugin x 1)
  int*      plugin_statenum;      // number of states in the plugin instance  (nplugin x 1)
  char*     plugin_attr;          // config attributes of plugin instances    (npluginattr x 1)
  int*      plugin_attradr;       // address to each instance's config attrib (nplugin x 1)

  // custom numeric fields
  int*      numeric_adr;          // address of field in numeric_data         (nnumeric x 1)
  int*      numeric_size;         // size of numeric field                    (nnumeric x 1)
  mjtNum*   numeric_data;         // array of all numeric fields              (nnumericdata x 1)

  // custom text fields
  int*      text_adr;             // address of text in text_data             (ntext x 1)
  int*      text_size;            // size of text field (strlen+1)            (ntext x 1)
  char*     text_data;            // array of all text fields (0-terminated)  (ntextdata x 1)

  // custom tuple fields
  int*      tuple_adr;            // address of text in text_data             (ntuple x 1)
  int*      tuple_size;           // number of objects in tuple               (ntuple x 1)
  int*      tuple_objtype;        // array of object types in all tuples      (ntupledata x 1)
  int*      tuple_objid;          // array of object ids in all tuples        (ntupledata x 1)
  mjtNum*   tuple_objprm;         // array of object params in all tuples     (ntupledata x 1)

  // keyframes
  mjtNum*   key_time;             // key time                                 (nkey x 1)
  mjtNum*   key_qpos;             // key position                             (nkey x nq)
  mjtNum*   key_qvel;             // key velocity                             (nkey x nv)
  mjtNum*   key_act;              // key activation                           (nkey x na)
  mjtNum*   key_mpos;             // key mocap position                       (nkey x 3*nmocap)
  mjtNum*   key_mquat;            // key mocap quaternion                     (nkey x 4*nmocap)
  mjtNum*   key_ctrl;             // key control                              (nkey x nu)

  // names
  int*      name_bodyadr;         // body name pointers                       (nbody x 1)
  int*      name_jntadr;          // joint name pointers                      (njnt x 1)
  int*      name_geomadr;         // geom name pointers                       (ngeom x 1)
  int*      name_siteadr;         // site name pointers                       (nsite x 1)
  int*      name_camadr;          // camera name pointers                     (ncam x 1)
  int*      name_lightadr;        // light name pointers                      (nlight x 1)
  int*      name_flexadr;         // flex name pointers                       (nflex x 1)
  int*      name_meshadr;         // mesh name pointers                       (nmesh x 1)
  int*      name_skinadr;         // skin name pointers                       (nskin x 1)
  int*      name_hfieldadr;       // hfield name pointers                     (nhfield x 1)
  int*      name_texadr;          // texture name pointers                    (ntex x 1)
  int*      name_matadr;          // material name pointers                   (nmat x 1)
  int*      name_pairadr;         // geom pair name pointers                  (npair x 1)
  int*      name_excludeadr;      // exclude name pointers                    (nexclude x 1)
  int*      name_eqadr;           // equality constraint name pointers        (neq x 1)
  int*      name_tendonadr;       // tendon name pointers                     (ntendon x 1)
  int*      name_actuatoradr;     // actuator name pointers                   (nu x 1)
  int*      name_sensoradr;       // sensor name pointers                     (nsensor x 1)
  int*      name_numericadr;      // numeric name pointers                    (nnumeric x 1)
  int*      name_textadr;         // text name pointers                       (ntext x 1)
  int*      name_tupleadr;        // tuple name pointers                      (ntuple x 1)
  int*      name_keyadr;          // keyframe name pointers                   (nkey x 1)
  int*      name_pluginadr;       // plugin instance name pointers            (nplugin x 1)
  char*     names;                // names of all objects, 0-terminated       (nnames x 1)
  int*      names_map;            // internal hash map of names               (nnames_map x 1)

  // paths
  char*     paths;                // paths to assets, 0-terminated            (npaths x 1)
};
typedef struct mjModel_ mjModel;

#endif  // MUJOCO_MJMODEL_H_
