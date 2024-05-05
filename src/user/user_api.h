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

#include <math.h>
#include <stddef.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>


// this is a C-API
#ifdef __cplusplus
extern "C" {
#endif

#define mjNAN NAN                  // used to mark undefined fields


//---------------------------------- handles to internal objects -----------------------------------

typedef struct _mjString* mjString;
typedef struct _mjStringVec* mjStringVec;
typedef struct _mjIntVec* mjIntVec;
typedef struct _mjIntVecVec* mjIntVecVec;
typedef struct _mjFloatVec* mjFloatVec;
typedef struct _mjFloatVecVec* mjFloatVecVec;
typedef struct _mjDoubleVec* mjDoubleVec;


//---------------------------------- enum types (mjt) ----------------------------------------------

typedef enum _mjtGeomInertia {     // type of inertia inference
  mjINERTIA_VOLUME,                // mass distributed in the volume
  mjINERTIA_SHELL,                 // mass distributed on the surface
} mjtGeomInertia;


typedef enum _mjtBuiltin {         // type of built-in procedural texture
  mjBUILTIN_NONE = 0,              // no built-in texture
  mjBUILTIN_GRADIENT,              // gradient: rgb1->rgb2
  mjBUILTIN_CHECKER,               // checker pattern: rgb1, rgb2
  mjBUILTIN_FLAT                   // 2d: rgb1; cube: rgb1-up, rgb2-side, rgb3-down
} mjtBuiltin;


typedef enum _mjtMark {            // mark type for procedural textures
  mjMARK_NONE = 0,                 // no mark
  mjMARK_EDGE,                     // edges
  mjMARK_CROSS,                    // cross
  mjMARK_RANDOM                    // random dots
} mjtMark;


typedef enum _mjtLimited {         // type of limit specification
  mjLIMITED_FALSE = 0,             // not limited
  mjLIMITED_TRUE,                  // limited
  mjLIMITED_AUTO,                  // limited inferred from presence of range
} mjtLimited;


typedef enum _mjtInertiaFromGeom { // whether to infer body inertias from child geoms
  mjINERTIAFROMGEOM_FALSE = 0,     // do not use; inertial element required
  mjINERTIAFROMGEOM_TRUE,          // always use; overwrite inertial element
  mjINERTIAFROMGEOM_AUTO           // use only if inertial element is missing
} mjtInertiaFromGeom;


typedef enum _mjtOrientation {     // type of orientation specifier
  mjORIENTATION_QUAT = 0,          // quaternion
  mjORIENTATION_AXISANGLE,         // axis and angle
  mjORIENTATION_XYAXES,            // x and y axes
  mjORIENTATION_ZAXIS,             // z axis (minimal rotation)
  mjORIENTATION_EULER,             // Euler angles
} mjtOrientation;


//---------------------------------- attribute structs (mjs) ---------------------------------------

typedef struct _mjElement {        // element type, do not modify
  mjtObj elemtype;                 // element type
} mjElement;


typedef struct _mjSpec {           // model specification
  mjElement* element;              // element type
  mjString modelname;              // model name

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
  mjOption option;                 // physics options
  mjVisual visual;                 // visual options
  mjStatistic stat;                // statistics override (if defined)

  // sizes
  size_t memory;                   // number of bytes in arena+stack memory
  int nemax;                       // max number of equality constraints
  int nuserdata;                   // number of mjtNums in userdata
  int nuser_body;                  // number of mjtNums in body_user
  int nuser_jnt;                   // number of mjtNums in jnt_user
  int nuser_geom;                  // number of mjtNums in geom_user
  int nuser_site;                  // number of mjtNums in site_user
  int nuser_cam;                   // number of mjtNums in cam_user
  int nuser_tendon;                // number of mjtNums in tendon_user
  int nuser_actuator;              // number of mjtNums in actuator_user
  int nuser_sensor;                // number of mjtNums in sensor_user
  int nkey;                        // number of keyframes
  int njmax;                       // (deprecated) max number of constraints
  int nconmax;                     // (deprecated) max number of detected contacts
  size_t nstack;                   // (deprecated) number of mjtNums in mjData stack

  // global data
  mjString comment;                // comment at top of XML
  mjString modelfiledir;           // path to model file

  // other
  mjtByte hasImplicitPluginElem;   // already encountered an implicit plugin sensor/actuator
} mjSpec;


typedef struct _mjsOrientation {   // alternative orientation specifiers
  mjtOrientation type;             // active orientation specifier
  double axisangle[4];             // axis and angle
  double xyaxes[6];                // x and y axes
  double zaxis[3];                 // z axis (minimal rotation)
  double euler[3];                 // Euler angles
} mjsOrientation;


typedef struct _mjsPlugin {        // plugin specification
  mjElement* instance;             // element type
  mjString name;                   // name
  mjString instance_name;          // instance name
  int plugin_slot;                 // global registered slot number of the plugin
  mjtByte active;                  // is the plugin active
  mjString info;                   // message appended to compiler errors
} mjsPlugin;


typedef struct _mjsBody {          // body specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjString childclass;             // childclass name

  // body frame
  double pos[3];                   // frame position
  double quat[4];                  // frame orientation
  mjsOrientation alt;              // frame alternative orientation

  // inertial frame
  double mass;                     // mass
  double ipos[3];                  // inertial frame position
  double iquat[4];                 // inertial frame orientation
  double inertia[3];               // diagonal inertia (in i-frame)
  mjsOrientation ialt;             // inertial frame alternative orientation
  double fullinertia[6];           // non-axis-aligned inertia matrix

  // other
  mjtByte mocap;                   // is this a mocap body
  double gravcomp;                 // gravity compensation
  mjDoubleVec userdata;            // user data
  mjtByte explicitinertial;        // whether to save the body with explicit inertial clause
  mjsPlugin plugin;                // passive force plugin
  mjString info;                   // message appended to compiler errors
} mjsBody;


typedef struct _mjsFrame {         // frame specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjString childclass;             // childclass name
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
  mjString info;                   // message appended to compiler errors
} mjsFrame;


typedef struct _mjsJoint {         // joint specification
  mjElement* element;              // element type
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
  mjtByte actgravcomp;             // is gravcomp force applied via actuators
  mjDoubleVec userdata;            // user data
  mjString info;                   // message appended to compiler errors
} mjsJoint;


typedef struct _mjsGeom {          // geom specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjString classname;              // classname
  mjtGeom type;                    // geom type

  // frame, size
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
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
  mjsPlugin plugin;                // sdf plugin
  mjString info;                   // message appended to compiler errors
} mjsGeom;


typedef struct _mjsSite {          // site specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjString classname;              // class name

  // frame, size
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
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
} mjsSite;


typedef struct _mjsCamera {        // camera specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjString classname;              // class name

  // extrinsics
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
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
} mjsCamera;


typedef struct _mjsLight {         // light specification
  mjElement* element;              // element type
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
  double bulbradius;               // bulb radius, for soft shadows
  float attenuation[3];            // OpenGL attenuation (quadratic model)
  float cutoff;                    // OpenGL cutoff
  float exponent;                  // OpenGL exponent
  float ambient[3];                // ambient color
  float diffuse[3];                // diffuse color
  float specular[3];               // specular color

  // other
  mjString info;                   // message appended to compiler errors
} mjsLight;


typedef struct _mjsFlex {
  mjElement* element;              // element type
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
} mjsFlex;


typedef struct _mjsMesh {          // mesh specification
  mjElement* element;              // element type
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
  mjsPlugin plugin;                // sdf plugin
  mjString info;                   // message appended to compiler errors
} mjsMesh;


typedef struct _mjsHField {        // height field specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjString content_type;           // content type of file
  mjString file;                   // file: (nrow, ncol, [elevation data])
  double size[4];                  // hfield size (ignore referencing geom size)
  int nrow;                        // number of rows
  int ncol;                        // number of columns
  mjFloatVec userdata;             // user-provided elevation data
  mjString info;                   // message appended to compiler errors
} mjsHField;



typedef struct _mjsSkin {          // skin specification
  mjElement* element;              // element type
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
} mjsSkin;


typedef struct _mjsTexture {       // texture specification
  mjElement* element;              // element type
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
} mjsTexture;


typedef struct _mjsMaterial {      // material specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjString classname;              // class name
  mjString texture;                // name of texture (empty: none)
  mjtByte texuniform;              // make texture cube uniform
  float texrepeat[2];              // texture repetition for 2D mapping
  float emission;                  // emission
  float specular;                  // specular
  float shininess;                 // shininess
  float reflectance;               // reflectance
  float metallic;                  // metallic
  float roughness;                 // roughness
  float rgba[4];                   // rgba
  mjString info;                   // message appended to compiler errors
} mjsMaterial;


typedef struct _mjsPair {
  mjElement* element;              // element type
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
} mjsPair;


typedef struct _mjsExclude {
  mjElement* element;              // element type
  mjString name;                   // name
  mjString bodyname1;              // name of geom 1
  mjString bodyname2;              // name of geom 2
  mjString info;                   // message appended to errors
} mjsExclude;


typedef struct _mjsEquality {      // equality specification
  mjElement* element;              // element type
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
} mjsEquality;


typedef struct _mjsTendon {        // tendon specification
  mjElement* element;              // element type
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
} mjsTendon;


typedef struct _mjsWrap {          // wrapping object specification
  mjElement* element;              // element type
  mjString info;                   // message appended to errors
} mjsWrap;


typedef struct _mjsActuator {      // actuator specification
  mjElement* element;              // element type
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
  mjsPlugin plugin;                // actuator plugin
  mjString info;                   // message appended to compiler errors
} mjsActuator;


typedef struct _mjsSensor {        // sensor specification
  mjElement* element;              // element type
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
  mjsPlugin plugin;                // sensor plugin
  mjString info;                   // message appended to compiler errors
} mjsSensor;


typedef struct _mjsNumeric {       // custom numeric field specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjDoubleVec data;                // initialization data
  int size;                        // array size, can be bigger than data size
  mjString info;                   // message appended to compiler errors
} mjsNumeric;


typedef struct _mjsText {          // custom text specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjString data;                   // text string
  mjString info;                   // message appended to compiler errors
} mjsText;


typedef struct _mjsTuple {         // tuple specification
  mjElement* element;              // element type
  mjString name;                   // name
  mjIntVec objtype;                // object types
  mjStringVec objname;             // object names
  mjDoubleVec objprm;              // object parameters
  mjString info;                   // message appended to compiler errors
} mjsTuple;


typedef struct _mjsKey {           // keyframe specification
  mjElement* element;              // element type
  mjString name;                   // name
  double time;                     // time
  mjDoubleVec qpos;                // qpos
  mjDoubleVec qvel;                // qvel
  mjDoubleVec act;                 // act
  mjDoubleVec mpos;                // mocap pos
  mjDoubleVec mquat;               // mocap quat
  mjDoubleVec ctrl;                // ctrl
  mjString info;                   // message appended to compiler errors
} mjsKey;


typedef struct _mjsDefault {       // default specification
  mjElement* element;              // element type
  mjString name;                   // class name
  mjsJoint* joint;                 // joint defaults
  mjsGeom* geom;                   // geom defaults
  mjsSite* site;                   // site defaults
  mjsCamera* camera;               // camera defaults
  mjsLight* light;                 // light defaults
  mjsFlex* flex;                   // flex defaults
  mjsMesh* mesh;                   // mesh defaults
  mjsMaterial* material;           // material defaults
  mjsPair* pair;                   // pair defaults
  mjsEquality* equality;           // equality defaults
  mjsTendon* tendon;               // tendon defaults
  mjsActuator* actuator;           // actuator defaults
} mjsDefault;


//---------------------------------- Top-level spec manipulation -----------------------------------

// Create spec.
MJAPI mjSpec* mjs_createSpec(void);

// Compile spec to model.
MJAPI mjModel* mjs_compile(mjSpec* s, const mjVFS* vfs);

// Copy spec.
MJAPI mjSpec* mjs_copySpec(const mjSpec* s);

// Get compiler error message from spec.
MJAPI const char* mjs_getError(mjSpec* s);

// Return 1 if compiler error is a warning.
MJAPI int mjs_isWarning(mjSpec* s);

// Copy model fields back into spec.
MJAPI void mjs_copyBack(mjSpec* s, const mjModel* m);

// Delete spec.
MJAPI void mjs_deleteSpec(mjSpec* s);


//---------------------------------- Attachment ----------------------------------------------------

// Attach child body to a parent frame, return 0 on success.
MJAPI int mjs_attachBody(mjsFrame* parent, const mjsBody* child,
                         const char* prefix, const char* suffix);

// Attach child frame to a parent body, return 0 on success.
MJAPI int mjs_attachFrame(mjsBody* parent, const mjsFrame* child,
                          const char* prefix, const char* suffix);

// Detach body from mjSpec, remove all references and delete the body, return 0 on success.
MJAPI int mjs_detachBody(mjSpec* s, mjsBody* b);


//---------------------------------- Add tree elements ---------------------------------------------

// Add child body to body, return child.
MJAPI mjsBody* mjs_addBody(mjsBody* body, mjsDefault* def);

// Add site to body, return site spec.
MJAPI mjsSite* mjs_addSite(mjsBody* body, mjsDefault* def);

// Add joint to body.
MJAPI mjsJoint* mjs_addJoint(mjsBody* body, mjsDefault* def);

// Add freejoint to body.
MJAPI mjsJoint* mjs_addFreeJoint(mjsBody* body);

// Add geom to body.
MJAPI mjsGeom* mjs_addGeom(mjsBody* body, mjsDefault* def);

// Add camera to body.
MJAPI mjsCamera* mjs_addCamera(mjsBody* body, mjsDefault* def);

// Add light to body.
MJAPI mjsLight* mjs_addLight(mjsBody* body, mjsDefault* def);

// Add frame to body.
MJAPI mjsFrame* mjs_addFrame(mjsBody* body, mjsFrame* parentframe);

// Delete body. TODO: make this a general mjs_deleteElement function
MJAPI void mjs_deleteBody(mjsBody* b);


//---------------------------------- Add non-tree elements -----------------------------------------

// Add actuator.
MJAPI mjsActuator* mjs_addActuator(mjSpec* s, mjsDefault* def);

// Add sensor.
MJAPI mjsSensor* mjs_addSensor(mjSpec* s);

// Add flex.
MJAPI mjsFlex* mjs_addFlex(mjSpec* s);

// Add contact pair.
MJAPI mjsPair* mjs_addPair(mjSpec* s, mjsDefault* def);

// Add excluded body pair.
MJAPI mjsExclude* mjs_addExclude(mjSpec* s);

// Add equality.
MJAPI mjsEquality* mjs_addEquality(mjSpec* s, mjsDefault* def);

// Add tendon.
MJAPI mjsTendon* mjs_addTendon(mjSpec* s, mjsDefault* def);

// Wrap site using tendon.
MJAPI mjsWrap* mjs_wrapSite(mjsTendon* tendon, const char* name);

// Wrap geom using tendon.
MJAPI mjsWrap* mjs_wrapGeom(mjsTendon* tendon, const char* name, const char* sidesite);

// Wrap joint using tendon.
MJAPI mjsWrap* mjs_wrapJoint(mjsTendon* tendon, const char* name, double coef);

// Wrap pulley using tendon.
MJAPI mjsWrap* mjs_wrapPulley(mjsTendon* tendon, double divisor);

// Add numeric.
MJAPI mjsNumeric* mjs_addNumeric(mjSpec* s);

// Add text.
MJAPI mjsText* mjs_addText(mjSpec* s);

// Add tuple.
MJAPI mjsTuple* mjs_addTuple(mjSpec* s);

// Add keyframe.
MJAPI mjsKey* mjs_addKey(mjSpec* s);

// Add plugin.
MJAPI mjsPlugin* mjs_addPlugin(mjSpec* s);

// Add default.
MJAPI mjsDefault* mjs_addDefault(mjSpec* s, const char* classname, int parentid, int* id);


//---------------------------------- Add assets ----------------------------------------------------

// Add mesh.
MJAPI mjsMesh* mjs_addMesh(mjSpec* s, mjsDefault* def);

// Add height field.
MJAPI mjsHField* mjs_addHField(mjSpec* s);

// Add skin.
MJAPI mjsSkin* mjs_addSkin(mjSpec* s);

// Add texture.
MJAPI mjsTexture* mjs_addTexture(mjSpec* s);

// Add material.
MJAPI mjsMaterial* mjs_addMaterial(mjSpec* s, mjsDefault* def);


//---------------------------------- Find/get utilities --------------------------------------------

// Get spec from body.
MJAPI mjSpec* mjs_getSpec(mjsBody* body);

// Find body in model by name.
MJAPI mjsBody* mjs_findBody(mjSpec* s, const char* name);

// Find child body by name.
MJAPI mjsBody* mjs_findChild(mjsBody* body, const char* name);

// Find mesh by name.
MJAPI mjsMesh* mjs_findMesh(mjSpec* s, const char* name);

// Find frame by name.
MJAPI mjsFrame* mjs_findFrame(mjSpec* s, const char* name);

// Get default corresponding to an element.
MJAPI mjsDefault* mjs_getDefault(mjElement* element);

// Find default in model by class name.
MJAPI mjsDefault* mjs_findDefault(mjSpec* s, const char* classname);

// Get global default from model.
MJAPI mjsDefault* mjs_getSpecDefault(mjSpec* s);

// Get element id.
MJAPI int mjs_getId(mjElement* element);


//---------------------------------- Attribute setters ---------------------------------------------

// Copy text to string.
MJAPI void mjs_setString(mjString dest, const char* text);

// Split text to entries and copy to string vector.
MJAPI void mjs_setStringVec(mjStringVec dest, const char* text);

// Set entry in string vector.
MJAPI mjtByte mjs_setInStringVec(mjStringVec dest, int i, const char* text);

// Append text entry to string vector.
MJAPI void mjs_appendString(mjStringVec dest, const char* text);

// Copy int array to vector.
MJAPI void mjs_setInt(mjIntVec dest, const int* array, int size);

// Append int array to vector of arrays.
MJAPI void mjs_appendIntVec(mjIntVecVec dest, const int* array, int size);

// Copy float array to vector.
MJAPI void mjs_setFloat(mjFloatVec dest, const float* array, int size);

// Append float array to vector of arrays.
MJAPI void mjs_appendFloatVec(mjFloatVecVec dest, const float* array, int size);

// Copy double array to vector.
MJAPI void mjs_setDouble(mjDoubleVec dest, const double* array, int size);

// Set plugin attributes.
MJAPI void mjs_setPluginAttributes(mjsPlugin* plugin, void* attributes);


//---------------------------------- Attribute getters ---------------------------------------------

// Get string contents.
MJAPI const char* mjs_getString(mjString source);

// Get double array contents and optionally its size.
MJAPI const double* mjs_getDouble(mjDoubleVec source, int* size);


//---------------------------------- Other utilities -----------------------------------------------

// Set active plugins.
MJAPI void mjs_setActivePlugins(mjSpec* s, void* activeplugins);

// Set element's default.
MJAPI void mjs_setDefault(mjElement* element, mjsDefault* def);

// Set element's enlcosing frame.
MJAPI void mjs_setFrame(mjElement* dest, mjsFrame* frame);

// Resolve alternative orientations to quat, return error if any.
MJAPI const char* mjs_resolveOrientation(double quat[4], mjtByte degree, const char* sequence,
                                         const mjsOrientation* orientation);

// Compute quat and diag inertia from full inertia matrix, return error if any.
MJAPI const char* mjs_fullInertia(double quat[4], double inertia[3], const double fullinertia[6]);


//---------------------------------- Initialization  -----------------------------------------------

// Default spec attributes.
MJAPI void mjs_defaultSpec(mjSpec* spec);

// Default orientation attributes.
MJAPI void mjs_defaultOrientation(mjsOrientation* orient);

// Default body attributes.
MJAPI void mjs_defaultBody(mjsBody* body);

// Default frame attributes.
MJAPI void mjs_defaultFrame(mjsFrame* frame);

// Default joint attributes.
MJAPI void mjs_defaultJoint(mjsJoint* joint);

// Default geom attributes.
MJAPI void mjs_defaultGeom(mjsGeom* geom);

// Default site attributes.
MJAPI void mjs_defaultSite(mjsSite* site);

// Default camera attributes.
MJAPI void mjs_defaultCamera(mjsCamera* camera);

// Default light attributes.
MJAPI void mjs_defaultLight(mjsLight* light);

// Default flex attributes.
MJAPI void mjs_defaultFlex(mjsFlex* flex);

// Default mesh attributes.
MJAPI void mjs_defaultMesh(mjsMesh* mesh);

// Default height field attributes.
MJAPI void mjs_defaultHField(mjsHField* hfield);

// Default skin attributes.
MJAPI void mjs_defaultSkin(mjsSkin* skin);

// Default texture attributes.
MJAPI void mjs_defaultTexture(mjsTexture* texture);

// Default material attributes.
MJAPI void mjs_defaultMaterial(mjsMaterial* material);

// Default pair attributes.
MJAPI void mjs_defaultPair(mjsPair* pair);

// Default equality attributes.
MJAPI void mjs_defaultEquality(mjsEquality* equality);

// Default tendon attributes.
MJAPI void mjs_defaultTendon(mjsTendon* tendon);

// Default actuator attributes.
MJAPI void mjs_defaultActuator(mjsActuator* actuator);

// Default sensor attributes.
MJAPI void mjs_defaultSensor(mjsSensor* sensor);

// Default numeric attributes.
MJAPI void mjs_defaultNumeric(mjsNumeric* numeric);

// Default text attributes.
MJAPI void mjs_defaultText(mjsText* text);

// Default tuple attributes.
MJAPI void mjs_defaultTuple(mjsTuple* tuple);

// Default keyframe attributes.
MJAPI void mjs_defaultKey(mjsKey* key);

// Default plugin attributes.
MJAPI void mjs_defaultPlugin(mjsPlugin* plugin);


//---------------------------------- Compiler cache ------------------------------------------------

typedef struct _mjCache* mjCache;

// Set the size of the cache in bytes.
MJAPI void mj_setCacheSize(mjCache cache, size_t size);

// Get internal global cache context.
MJAPI mjCache mj_globalCache(void);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_USER_USER_API_H_
