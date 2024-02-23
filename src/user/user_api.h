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

#ifndef MUJOCO_SRC_USER_USER_API_H_
#define MUJOCO_SRC_USER_USER_API_H_

#include <stddef.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>


// this is a C-API
#ifdef __cplusplus
extern "C" {
#endif


//---------------------------------- handles to internal objects -----------------------------------

typedef struct _mjElement* mjElement;
typedef struct _mjString* mjString;
typedef struct _mjStringVec* mjStringVec;
typedef struct _mjIntVec* mjIntVec;
typedef struct _mjIntVecVec* mjIntVecVec;
typedef struct _mjFloatVec* mjFloatVec;
typedef struct _mjFloatVecVec* mjFloatVecVec;
typedef struct _mjDoubleVec* mjDoubleVec;


//---------------------------------- enum types (mjt) ----------------------------------------------

typedef enum _mjtGeomInertia {    // type of inertia inference
  mjINERTIA_VOLUME,               // mass distributed in the volume
  mjINERTIA_SHELL,                // mass distributed on the surface
} mjtGeomInertia;


typedef enum _mjtBuiltin {        // type of built-in procedural texture
  mjBUILTIN_NONE = 0,             // no built-in texture
  mjBUILTIN_GRADIENT,             // gradient: rgb1->rgb2
  mjBUILTIN_CHECKER,              // checker pattern: rgb1, rgb2
  mjBUILTIN_FLAT                  // 2d: rgb1; cube: rgb1-up, rgb2-side, rgb3-down
} mjtBuiltin;


typedef enum _mjtMark {           // mark type for procedural textures
  mjMARK_NONE = 0,                // no mark
  mjMARK_EDGE,                    // edges
  mjMARK_CROSS,                   // cross
  mjMARK_RANDOM                   // random dots
} mjtMark;


typedef enum _mjtLimited {        // type of limit specification
  mjLIMITED_FALSE = 0,            // not limited
  mjLIMITED_TRUE,                 // limited
  mjLIMITED_AUTO,                 // limited inferred from presence of range
} mjtLimited;


typedef enum _mjtInertiaFromGeom {
  mjINERTIAFROMGEOM_FALSE = 0,    // do not use; inertial element required
  mjINERTIAFROMGEOM_TRUE,         // always use; overwrite inertial element
  mjINERTIAFROMGEOM_AUTO          // use only if inertial element is missing
} mjtInertiaFromGeom;


//---------------------------------- attribute structs (mjm) ---------------------------------------

typedef struct _mjmModel {         // model specification
  mjElement element;               // internal, do not modify
  mjStatistic stat;                // statistics override (if defined)

  // compiler settings
  mjtByte autolimits;              // infer "limited" attribute based on range
  double boundmass;                // enforce minimum body mass
  double boundinertia;             // enforce minimum body diagonal inertia
  double settotalmass;             // rescale masses and inertias; <=0: ignore
  mjtByte balanceinertia;          // automatically impose A + B >= C rule
  mjtByte strippath;               // automatically strip paths from mesh files
  mjtByte fitaabb;                 // meshfit to aabb instead of inertia box
  mjtByte degree;                  // angles in radians or degrees
  char euler[3];                   // sequence for euler rotations
  mjString meshdir;                // mesh and hfield directory
  mjString texturedir;             // texture directory
  mjtByte discardvisual;           // discard visual geoms in parser
  mjtByte convexhull;              // compute mesh convex hulls
  mjtByte usethread;               // use multiple threads to speed up compiler
  mjtByte fusestatic;              // fuse static bodies with parent
  int inertiafromgeom;             // use geom inertias (mjtInertiaFromGeom)
  int inertiagrouprange[2];        // range of geom groups used to compute inertia
  mjtByte exactmeshinertia;        // if false, use old formula
  mjLROpt LRopt;                   // options for lengthrange computation

  // engine data
  mjString modelname;             // model name
  mjOption option;                // options
  mjVisual visual;                // visual options
  size_t memory;                  // size of arena+stack memory in bytes
  int nemax;                      // max number of equality constraints
  int njmax;                      // max number of constraints (Jacobian rows)
  int nconmax;                    // max number of detected contacts (mjContact array size)
  size_t nstack;                  // (deprecated) number of fields in mjData stack
  int nuserdata;                  // number extra fields in mjData
  int nuser_body;                 // number of mjtNums in body_user
  int nuser_jnt;                  // number of mjtNums in jnt_user
  int nuser_geom;                 // number of mjtNums in geom_user
  int nuser_site;                 // number of mjtNums in site_user
  int nuser_cam;                  // number of mjtNums in cam_user
  int nuser_tendon;               // number of mjtNums in tendon_user
  int nuser_actuator;             // number of mjtNums in actuator_user
  int nuser_sensor;               // number of mjtNums in sensor_user

  // sizes
  int nkey;                       // number of keyframes
} mjmModel;


typedef struct _mjmOrientation {   // alternative orientation specifiers
  double axisangle[4];             // rotation axis and angle
  double xyaxes[6];                // x and y axes
  double zaxis[3];                 // z axis (use minimal rotation)
  double euler[3];                 // euler angles
} mjmOrientation;


typedef struct _mjmPlugin {        // plugin specification
  mjElement instance;              // internal, do not modify
  mjString name;                   // name
  mjString instance_name;          // instance name
  int plugin_slot;                 // global registered slot number of the plugin
  mjtByte active;                  // is the plugin active
  mjString info;                   // message appended to compiler errors
} mjmPlugin;


typedef struct _mjmBody {          // body specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // childclass name

  // body frame
  double pos[3];                   // frame position
  double quat[4];                  // frame orientation
  mjmOrientation alt;              // frame alternative orientation

  // inertial frame
  double mass;                     // mass
  double ipos[3];                  // inertial frame position
  double iquat[4];                 // inertial frame orientation
  double inertia[3];               // diagonal inertia (in i-frame)
  mjmOrientation ialt;             // inertial frame alternative orientation
  double fullinertia[6];           // non-axis-aligned inertia matrix

  // other
  mjtByte mocap;                   // is this a mocap body
  double gravcomp;                 // gravity compensation
  mjDoubleVec userdata;            // user data
  mjtByte explicitinertial;        // whether to save the body with explicit inertial clause
  mjmPlugin plugin;                // passive force plugin
  mjString info;                   // message appended to compiler errors
} mjmBody;


typedef struct _mjmFrame {         // frame specification
  mjElement element;               // internal, do not modify
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjmOrientation alt;              // alternative orientation
  mjString info;                   // message appended to compiler errors
} mjmFrame;


typedef struct _mjmJoint {         // joint specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name
  mjtJoint type;                   // joint type

  // kinematics
  double pos[3];                   // anchor position
  double axis[3];                  // joint axis
  double ref;                      // value at reference configuration: qpos0

  // stiffness
  double stiffness;                // stiffness coefficient
  double springref;                // spring reference value: qpos_spring
  double springdamper[2];          // timeconst, dampratio

  // limits
  int limited;                     // does joint have limits (mjtLimited)
  double range[2];                 // joint limits
  double margin;                   // margin value for joint limit detection
  mjtNum solref_limit[mjNREF];     // solver reference: joint limits
  mjtNum solimp_limit[mjNIMP];     // solver impedance: joint limits
  int actfrclimited;               // are actuator forces on joint limited (mjtLimited)
  double actfrcrange[2];           // actuator force limits

  // dof properties
  double armature;                 // armature inertia (mass for slider)
  double damping;                  // damping coefficient
  double frictionloss;             // friction loss
  mjtNum solref_friction[mjNREF];  // solver reference: dof friction
  mjtNum solimp_friction[mjNIMP];  // solver impedance: dof friction

  // other
  int group;                       // group
  double urdfeffort;               // effort (urdf)
  mjDoubleVec userdata;            // user data
  mjString info;                   // message appended to compiler errors
} mjmJoint;


typedef struct _mjmGeom {          // geom specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // classname
  mjtGeom type;                    // geom type

  // frame, size
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjmOrientation alt;              // alternative orientation
  double fromto[6];                // alternative for capsule, cylinder, box, ellipsoid
  double size[3];                  // type-specific size

  // contact related
  int contype;                     // contact type
  int conaffinity;                 // contact affinity
  int condim;                      // contact dimensionality
  int priority;                    // contact priority
  double friction[3];              // one-sided friction coefficients: slide, roll, spin
  double solmix;                   // solver mixing for contact pairs
  mjtNum solref[mjNREF];           // solver reference
  mjtNum solimp[mjNIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist < margin-gap

  // inertia inference
  double mass;                     // used to compute density
  double density;                  // used to compute mass and inertia from volume or surface
  mjtGeomInertia typeinertia;      // selects between surface and volume inertia

  // fluid forces
  mjtNum fluid_ellipsoid;          // whether ellipsoid-fluid model is active
  mjtNum fluid_coefs[5];           // ellipsoid-fluid interaction coefs

  // visual
  mjString material;               // name of material
  float rgba[4];                   // rgba when material is omitted
  int group;                       // group

  // other
  mjString hfieldname;             // heightfield attached to geom
  mjString meshname;               // mesh attached to geom
  double fitscale;                 // scale mesh uniformly
  mjDoubleVec userdata;            // user data
  mjmPlugin plugin;                // sdf plugin
  mjString info;                   // message appended to compiler errors
} mjmGeom;


typedef struct _mjmSite {          // site specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name

  // frame, size
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjmOrientation alt;              // alternative orientation
  double fromto[6];                // alternative for capsule, cylinder, box, ellipsoid
  double size[3];                  // geom size

  // visual
  mjtGeom type;                    // geom type
  mjString material;               // name of material
  int group;                       // group
  float rgba[4];                   // rgba when material is omitted

  // other
  mjDoubleVec userdata;            // user data
  mjString info;                   // message appended to compiler errors
} mjmSite;


typedef struct _mjmCamera {        // camera specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name

  // extrinsics
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjmOrientation alt;              // alternative orientation
  mjtCamLight mode;                // tracking mode
  mjString targetbody;             // target body for tracking/targeting

  // intrinsics
  double fovy;                     // y-field of view
  double ipd;                      // inter-pupilary distance
  float intrinsic[4];              // camera intrinsics (length)
  float sensor_size[2];            // sensor size (length)
  float resolution[2];             // resolution (pixel)
  float focal_length[2];           // focal length (length)
  float focal_pixel[2];            // focal length (pixel)
  float principal_length[2];       // principal point (length)
  float principal_pixel[2];        // principal point (pixel)

  // other
  mjDoubleVec userdata;            // user data
  mjString info;                   // message appended to compiler errors
} mjmCamera;


typedef struct _mjmLight {         // light specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name

  // frame
  double pos[3];                   // position
  double dir[3];                   // direction
  mjtCamLight mode;                // tracking mode
  mjString targetbody;             // target body for targeting

  // intrinsics
  mjtByte active;                  // is light active
  mjtByte directional;             // is light directional or spot
  mjtByte castshadow;              // does light cast shadows
  float attenuation[3];            // OpenGL attenuation (quadratic model)
  float cutoff;                    // OpenGL cutoff
  float exponent;                  // OpenGL exponent
  float ambient[3];                // ambient color
  float diffuse[3];                // diffuse color
  float specular[3];               // specular color

  // other
  mjString info;                   // message appended to compiler errors
} mjmLight;


typedef struct _mjmFlex {
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name

  // contact properties
  int contype;                     // contact type
  int conaffinity;                 // contact affinity
  int condim;                      // contact dimensionality
  int priority;                    // contact priority
  double friction[3];              // one-sided friction coefficients: slide, roll, spin
  double solmix;                   // solver mixing for contact pairs
  mjtNum solref[mjNREF];           // solver reference
  mjtNum solimp[mjNIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist<margin-gap

  // other properties
  int dim;                         // element dimensionality
  double radius;                   // radius around primitive element
  mjtByte internal;                // enable internal collisions
  mjtByte flatskin;                // render flex skin with flat shading
  int selfcollide;                 // mode for flex self colllision
  int activelayers;                // number of active element layers in 3D
  int group;                       // group for visualizatioh
  double edgestiffness;            // edge stiffness
  double edgedamping;              // edge damping
  float rgba[4];                   // rgba when material is omitted
  mjString material;               // name of material used for rendering

  // mesh properties
  mjStringVec vertbody;            // vertex body names
  mjDoubleVec vert;                // vertex positions
  mjIntVec elem;                   // element vertex ids
  mjFloatVec texcoord;             // vertex texture coordinates

  // other
  mjString info;                   // message appended to compiler errors
} mjmFlex;


typedef struct _mjmMesh {          // mesh specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name
  mjString content_type;           // content type of file
  mjString file;                   // mesh file
  double refpos[3];                // reference position
  double refquat[4];               // reference orientation
  double scale[3];                 // rescale mesh
  mjtByte smoothnormal;            // do not exclude large-angle faces from normals
  mjFloatVec uservert;             // user vertex data
  mjFloatVec usernormal;           // user normal data
  mjFloatVec usertexcoord;         // user texcoord data
  mjIntVec userface;               // user vertex indices
  mjIntVec userfacenormal;         // user normal indices
  mjIntVec userfacetexcoord;       // user texcoord indices
  mjmPlugin plugin;                // sdf plugin
  mjString info;                   // message appended to compiler errors
} mjmMesh;


typedef struct _mjmHField {        // height field specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString content_type;           // content type of file
  mjString file;                   // file: (nrow, ncol, [elevation data])
  double size[4];                  // hfield size (ignore referencing geom size)
  int nrow;                        // number of rows
  int ncol;                        // number of columns
  mjFloatVec userdata;             // user-provided elevation data
  mjString info;                   // message appended to compiler errors
} mjmHField;



typedef struct _mjmSkin {          // skin specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name
  mjString file;                   // skin file
  mjString material;               // name of material used for rendering
  float rgba[4];                   // rgba when material is omitted
  float inflate;                   // inflate in normal direction
  int group;                       // group for visualization

  // mesh
  mjFloatVec vert;                 // vertex positions
  mjFloatVec texcoord;             // texture coordinates
  mjIntVec face;                   // faces

  // skin
  mjStringVec bodyname;            // body names
  mjFloatVec bindpos;              // bind pos
  mjFloatVec bindquat;             // bind quat
  mjIntVecVec vertid;              // vertex ids
  mjFloatVecVec vertweight;        // vertex weights

  // other
  mjString info;                   // message appended to compiler errors
} mjmSkin;


typedef struct _mjmTexture {       // texture specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name
  mjtTexture type;                 // texture type

  // method 1: builtin
  int builtin;                     // builtin type (mjtBuiltin)
  int mark;                        // mark type (mjtMark)
  double rgb1[3];                  // first color for builtin
  double rgb2[3];                  // second color for builtin
  double markrgb[3];               // mark color
  double random;                   // probability of random dots
  int height;                      // height in pixels (square for cube and skybox)
  int width;                       // width in pixels

  // method 2: single file
  mjString content_type;           // content type of file
  mjString file;                   // png file to load; use for all sides of cube
  int gridsize[2];                 // size of grid for composite file; (1,1)-repeat
  char gridlayout[13];             // row-major: L,R,F,B,U,D for faces; . for unused

  // method 3: separate files
  mjStringVec cubefiles;           // different file for each side of the cube

  // flip options
  mjtByte hflip;                   // horizontal flip
  mjtByte vflip;                   // vertical flip

  // other
  mjString info;                   // message appended to compiler errors
} mjmTexture;


typedef struct _mjmMaterial {      // material specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name
  mjString texture;                // name of texture (empty: none)
  mjtByte texuniform;              // make texture cube uniform
  float texrepeat[2];              // texture repetition for 2D mapping
  float emission;                  // emission
  float specular;                  // specular
  float shininess;                 // shininess
  float reflectance;               // reflectance
  float rgba[4];                   // rgba
  mjString info;                   // message appended to compiler errors
} mjmMaterial;


typedef struct _mjmPair {
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name
  mjString geomname1;              // name of geom 1
  mjString geomname2;              // name of geom 2

  // optional parameters: computed from geoms if not set by user
  int condim;                      // contact dimensionality
  mjtNum solref[mjNREF];           // solver reference, normal direction
  mjtNum solreffriction[mjNREF];   // solver reference, frictional directions
  mjtNum solimp[mjNIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist<margin-gap
  double friction[5];              // full contact friction
  mjString info;                   // message appended to errors
} mjmPair;


typedef struct _mjmExclude {
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString bodyname1;              // name of geom 1
  mjString bodyname2;              // name of geom 2
  mjString info;                   // message appended to errors
} mjmExclude;


typedef struct _mjmEquality {      // equality specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name
  mjtEq type;                      // constraint type
  double data[mjNEQDATA];          // type-dependent data
  mjtByte active;                  // is equality initially active
  mjString name1;                  // name of object 1
  mjString name2;                  // name of object 2
  mjtNum solref[mjNREF];           // solver reference
  mjtNum solimp[mjNIMP];           // solver impedance
  mjString info;                   // message appended to errors
} mjmEquality;


typedef struct _mjmTendon {        // tendon specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name

  // stiffness, damping, friction
  double stiffness;                // stiffness coefficient
  double springlength[2];          // spring resting length; {-1, -1}: use qpos_spring
  double damping;                  // damping coefficient
  double frictionloss;             // friction loss
  mjtNum solref_friction[mjNREF];  // solver reference: tendon friction
  mjtNum solimp_friction[mjNIMP];  // solver impedance: tendon friction

  // length range
  int limited;                     // does tendon have limits (mjtLimited)
  double range[2];                 // length limits
  double margin;                   // margin value for tendon limit detection
  mjtNum solref_limit[mjNREF];     // solver reference: tendon limits
  mjtNum solimp_limit[mjNIMP];     // solver impedance: tendon limits

  // visual
  mjString material;               // name of material for rendering
  double width;                    // width for rendering
  float rgba[4];                   // rgba when material is omitted
  int group;                       // group

  // other
  mjDoubleVec userdata;            // user data
  mjString info;                   // message appended to errors
} mjmTendon;


typedef struct _mjmWrap {          // wrapping object specification
  mjElement element;               // internal, do not modify
  mjString info;                   // message appended to errors
} mjmWrap;


typedef struct _mjmActuator {      // actuator specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name

  // gain, bias
  mjtGain gaintype;                // gain type
  double gainprm[mjNGAIN];         // gain parameters
  mjtBias biastype;                // bias type
  double biasprm[mjNGAIN];         // bias parameters

  // activation state
  mjtDyn dyntype;                  // dynamics type
  double dynprm[mjNDYN];           // dynamics parameters
  int actdim;                      // number of activation variables
  int plugin_actdim;               // actuator state size for plugins
  mjtByte actearly;                // apply next activations to qfrc

  // transmission
  mjtTrn trntype;                  // transmission type
  double gear[6];                  // length and transmitted force scaling
  mjString target;                 // name of transmission target
  mjString refsite;                // reference site, for site transmission
  mjString slidersite;             // site defining cylinder, for slider-crank
  double cranklength;              // crank length, for slider-crank
  double lengthrange[2];           // transmission length range
  double inheritrange;             // automatic range setting for position and intvelocity

  // input/output clamping
  int ctrllimited;                 // are control limits defined (mjtLimited)
  double ctrlrange[2];             // control range
  int forcelimited;                // are force limits defined (mjtLimited)
  double forcerange[2];            // force range
  int actlimited;                  // are activation limits defined (mjtLimited)
  double actrange[2];              // activation range

  // other
  int group;                       // group
  mjDoubleVec userdata;            // user data
  mjmPlugin plugin;                // actuator plugin
  mjString info;                   // message appended to compiler errors
} mjmActuator;


typedef struct _mjmSensor {        // sensor specfication
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString classname;              // class name

  // sensor defintion
  mjtSensor type;                  // type of sensor
  mjtObj objtype;                  // type of sensorized object
  mjString objname;                // name of sensorized object
  mjtObj reftype;                  // type of referenced object
  mjString refname;                // name of referenced object

  // user-defined sensors
  mjtDataType datatype;            // data type for sensor measurement
  mjtStage needstage;              // compute stage needed to simulate sensor
  int dim;                         // number of scalar outputs

  // output post-processing
  double cutoff;                   // cutoff for real and positive datatypes
  double noise;                    // noise stdev

  // other
  mjDoubleVec userdata;            // user data
  mjmPlugin plugin;                // sensor plugin
  mjString info;                   // message appended to compiler errors
} mjmSensor;


typedef struct _mjmNumeric {       // custom numeric field specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjDoubleVec data;                // initialization data
  int size;                        // array size, can be bigger than data size
  mjString info;                   // message appended to compiler errors
} mjmNumeric;


typedef struct _mjmText {          // custom text specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjString data;                   // text string
  mjString info;                   // message appended to compiler errors
} mjmText;


typedef struct _mjmTuple {         // tuple specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  mjIntVec objtype;                // object types
  mjStringVec objname;             // object names
  mjDoubleVec objprm;              // object parameters
  mjString info;                   // message appended to compiler errors
} mjmTuple;


typedef struct _mjmKey {           // keyframe specification
  mjElement element;               // internal, do not modify
  mjString name;                   // name
  double time;                     // time
  mjDoubleVec qpos;                // qpos
  mjDoubleVec qvel;                // qvel
  mjDoubleVec act;                 // act
  mjDoubleVec mpos;                // mocap pos
  mjDoubleVec mquat;               // mocap quat
  mjDoubleVec ctrl;                // ctrl
  mjString info;                   // message appended to compiler errors
} mjmKey;


typedef struct _mjmDefault {       // default specification
  mjString name;                   // name
  mjElement element;               // internal, do not modify
  mjmJoint* joint;                 // joint defaults
  mjmGeom* geom;                   // geom defaults
  mjmSite* site;                   // site defaults
  mjmCamera* camera;               // camera defaults
  mjmLight* light;                 // light defaults
  mjmFlex* flex;                   // flex defaults
  mjmMesh* mesh;                   // mesh defaults
  mjmMaterial* material;           // material defaults
  mjmPair* pair;                   // pair defaults
  mjmEquality* equality;           // equality defaults
  mjmTendon* tendon;               // tendon defaults
  mjmActuator* actuator;           // actuator defaults
} mjmDefault;


//---------------------------------- API functions -------------------------------------------------

// Create model.
MJAPI mjmModel* mjm_createModel();

// Delete model.
MJAPI void mjm_deleteModel(mjmModel* modelspec);

// Add child body to body, return child spec.
MJAPI mjmBody* mjm_addBody(mjmBody* body, mjmDefault* def);

// Add site to body, return site spec.
MJAPI mjmSite* mjm_addSite(mjmBody* body, mjmDefault* def);

// Add joint to body.
MJAPI mjmJoint* mjm_addJoint(mjmBody* body, mjmDefault* def);

// Add freejoint to body.
MJAPI mjmJoint* mjm_addFreeJoint(mjmBody* body);

// Add geom to body.
MJAPI mjmGeom* mjm_addGeom(mjmBody* body, mjmDefault* def);

// Add camera to body.
MJAPI mjmCamera* mjm_addCamera(mjmBody* body, mjmDefault* def);

// Add light to body.
MJAPI mjmLight* mjm_addLight(mjmBody* body, mjmDefault* def);

// Add frame to body.
MJAPI mjmFrame* mjm_addFrame(mjmBody* body, mjmFrame* parentframe);

// Add flex to model.
MJAPI mjmFlex* mjm_addFlex(mjmModel* model);

// Add mesh to model.
MJAPI mjmMesh* mjm_addMesh(mjmModel* model, mjmDefault* def);

// Add height field to model.
MJAPI mjmHField* mjm_addHField(mjmModel* model);

// Add skin to model.
MJAPI mjmSkin* mjm_addSkin(mjmModel* model);

// Add texture to model.
MJAPI mjmTexture* mjm_addTexture(mjmModel* model);

// Add material to model.
MJAPI mjmMaterial* mjm_addMaterial(mjmModel* model, mjmDefault* def);

// Add pair to model.
MJAPI mjmPair* mjm_addPair(mjmModel* model, mjmDefault* def);

// Add excluded body pair to model.
MJAPI mjmExclude* mjm_addExclude(mjmModel *model);

// Add equality to model.
MJAPI mjmEquality* mjm_addEquality(mjmModel* model, mjmDefault* def);

// Add tendon to model.
MJAPI mjmTendon* mjm_addTendon(mjmModel* model, mjmDefault* def);

// Wrap site using tendon.
MJAPI mjmWrap* mjm_wrapSite(mjmTendon* tendon, const char* name);

// Wrap geom using tendon.
MJAPI mjmWrap* mjm_wrapGeom(mjmTendon* tendon, const char* name, const char* sidesite);

// Wrap joint using tendon.
MJAPI mjmWrap* mjm_wrapJoint(mjmTendon* tendon, const char* name, double coef);

// Wrap pulley using tendon.
MJAPI mjmWrap* mjm_wrapPulley(mjmTendon* tendon, double divisor);

// Add actuator to model.
MJAPI mjmActuator* mjm_addActuator(mjmModel* model, mjmDefault* def);

// Add sensor to model.
MJAPI mjmSensor* mjm_addSensor(mjmModel* model);

// Add numeric to model.
MJAPI mjmNumeric* mjm_addNumeric(mjmModel* model);

// Add text to model.
MJAPI mjmText* mjm_addText(mjmModel* model);

// Add tuple to model.
MJAPI mjmTuple* mjm_addTuple(mjmModel* model);

// Add keyframe to model.
MJAPI mjmKey* mjm_addKey(mjmModel* model);

// Add plugin to model.
MJAPI mjmPlugin* mjm_addPlugin(mjmModel* model);

// Add default to model.
MJAPI mjmDefault* mjm_addDefault(mjmModel* model, const char* classname, int parentid);

// Get model from body.
MJAPI mjmModel* mjm_getModel(mjmBody* body);

// Get default corresponding to an mjElement.
MJAPI mjmDefault* mjm_getDefault(mjElement element);

// Find body in model by name.
MJAPI mjmBody* mjm_findBody(mjmModel* model, const char* name);

// Find child body by name.
MJAPI mjmBody* mjm_findChild(mjmBody* body, const char* name);

// Find mesh by name.
MJAPI mjmMesh* mjm_findMesh(mjmModel* model, const char* name);

// Get element id.
MJAPI int mjm_getId(mjElement element);

// Copy text to string.
MJAPI void mjm_setString(mjString dest, const char* text);

// Split text to entries and copy to string vector.
MJAPI void mjm_setStringVec(mjStringVec dest, const char* text);

// Set entry in string vector.
MJAPI mjtByte mjm_setInStringVec(mjStringVec dest, int i, const char* text);

// Append text entry to string vector.
MJAPI void mjm_appendString(mjStringVec dest, const char* text);

// Copy int array to vector.
MJAPI void mjm_setInt(mjIntVec dest, const int* array, int size);

// Append int array to vector of arrays.
MJAPI void mjm_appendIntVec(mjIntVecVec dest, const int* array, int size);

// Copy float array to vector.
MJAPI void mjm_setFloat(mjFloatVec dest, const float* array, int size);

// Append float array to vector of arrays.
MJAPI void mjm_appendFloatVec(mjFloatVecVec dest, const float* array, int size);

// Copy double array to vector.
MJAPI void mjm_setDouble(mjDoubleVec dest, const double* array, int size);

// Get string contents.
MJAPI const char* mjm_getString(mjString source);

// Get double array contents and optionally its size.
MJAPI const double* mjm_getDouble(mjDoubleVec source, int* size);

// Set plugin attributes.
MJAPI void mjm_setPluginAttributes(mjmPlugin* plugin, void* attributes);

// Set default.
MJAPI void mjm_setDefault(mjElement element, mjmDefault* def);

// Set frame.
MJAPI void mjm_setFrame(mjElement dest, mjmFrame* frame);

// Compute quat and inertia from body->fullinertia.
MJAPI const char* mjm_setFullInertia(mjmBody* body, double quat[4], double inertia[3]);


//---------------------------------- Initialization functions --------------------------------------

// Default model attributes.
MJAPI void mjm_defaultModel(mjmModel& model);

// Default body attributes.
MJAPI void mjm_defaultBody(mjmBody& body);

// Default frame attributes.
MJAPI void mjm_defaultFrame(mjmFrame& frame);

// Default joint attributes.
MJAPI void mjm_defaultJoint(mjmJoint& joint);

// Default geom attributes.
MJAPI void mjm_defaultGeom(mjmGeom& geom);

// Default site attributes.
MJAPI void mjm_defaultSite(mjmSite& site);

// Default camera attributes.
MJAPI void mjm_defaultCamera(mjmCamera& camera);

// Default light attributes.
MJAPI void mjm_defaultLight(mjmLight& light);

// Default flex attributes.
MJAPI void mjm_defaultFlex(mjmFlex& flex);

// Default mesh attributes.
MJAPI void mjm_defaultMesh(mjmMesh& mesh);

// Default height field attributes.
MJAPI void mjm_defaultHField(mjmHField& hfield);

// Default skin attributes.
MJAPI void mjm_defaultSkin(mjmSkin& skin);

// Default texture attributes.
MJAPI void mjm_defaultTexture(mjmTexture& texture);

// Default material attributes.
MJAPI void mjm_defaultMaterial(mjmMaterial& material);

// Default pair attributes.
MJAPI void mjm_defaultPair(mjmPair& pair);

// Default equality attributes.
MJAPI void mjm_defaultEquality(mjmEquality& equality);

// Default tendon attributes.
MJAPI void mjm_defaultTendon(mjmTendon& tendon);

// Default actuator attributes.
MJAPI void mjm_defaultActuator(mjmActuator& actuator);

// Default sensor attributes.
MJAPI void mjm_defaultSensor(mjmSensor& sensor);

// Default numeric attributes.
MJAPI void mjm_defaultNumeric(mjmNumeric& numeric);

// Default text attributes.
MJAPI void mjm_defaultText(mjmText& text);

// Default tuple attributes.
MJAPI void mjm_defaultTuple(mjmTuple& tuple);

// Default keyframe attributes.
MJAPI void mjm_defaultKey(mjmKey& key);

// Default plugin attributes.
MJAPI void mjm_defaultPlugin(mjmPlugin& plugin);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_USER_USER_API_H_
