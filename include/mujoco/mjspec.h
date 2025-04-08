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

#ifndef MUJOCO_INCLUDE_MJSPEC_H_
#define MUJOCO_INCLUDE_MJSPEC_H_

#include <stddef.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>


// this is a C-API
#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#endif

//-------------------------------- handles to strings and arrays -----------------------------------

#ifdef __cplusplus
  // C++: defined to be compatible with corresponding std types
  using mjString      = std::string;
  using mjStringVec   = std::vector<std::string>;
  using mjIntVec      = std::vector<int>;
  using mjIntVecVec   = std::vector<std::vector<int>>;
  using mjFloatVec    = std::vector<float>;
  using mjFloatVecVec = std::vector<std::vector<float>>;
  using mjDoubleVec   = std::vector<double>;
  using mjByteVec     = std::vector<std::byte>;
#else
  // C: opaque types
  typedef void mjString;
  typedef void mjStringVec;
  typedef void mjIntVec;
  typedef void mjIntVecVec;
  typedef void mjFloatVec;
  typedef void mjFloatVecVec;
  typedef void mjDoubleVec;
  typedef void mjByteVec;
#endif


//-------------------------------- enum types (mjt) ------------------------------------------------

typedef enum mjtGeomInertia_ {     // type of inertia inference
  mjINERTIA_VOLUME = 0,            // mass distributed in the volume
  mjINERTIA_SHELL,                 // mass distributed on the surface
} mjtGeomInertia;


typedef enum mjtMeshInertia_ {      // type of mesh inertia
  mjMESH_INERTIA_CONVEX = 0,        // convex mesh inertia
  mjMESH_INERTIA_EXACT,             // exact mesh inertia
  mjMESH_INERTIA_LEGACY,            // legacy mesh inertia
  mjMESH_INERTIA_SHELL              // shell mesh inertia
} mjtMeshInertia;


typedef enum mjtBuiltin_ {         // type of built-in procedural texture
  mjBUILTIN_NONE = 0,              // no built-in texture
  mjBUILTIN_GRADIENT,              // gradient: rgb1->rgb2
  mjBUILTIN_CHECKER,               // checker pattern: rgb1, rgb2
  mjBUILTIN_FLAT                   // 2d: rgb1; cube: rgb1-up, rgb2-side, rgb3-down
} mjtBuiltin;


typedef enum mjtMark_ {            // mark type for procedural textures
  mjMARK_NONE = 0,                 // no mark
  mjMARK_EDGE,                     // edges
  mjMARK_CROSS,                    // cross
  mjMARK_RANDOM                    // random dots
} mjtMark;


typedef enum mjtLimited_ {         // type of limit specification
  mjLIMITED_FALSE = 0,             // not limited
  mjLIMITED_TRUE,                  // limited
  mjLIMITED_AUTO,                  // limited inferred from presence of range
} mjtLimited;

typedef enum mjtAlignFree_ {       // whether to align free joints with the inertial frame
  mjALIGNFREE_FALSE = 0,           // don't align
  mjALIGNFREE_TRUE,                // align
  mjALIGNFREE_AUTO,                // respect the global compiler flag
} mjtAlignFree;


typedef enum mjtInertiaFromGeom_ { // whether to infer body inertias from child geoms
  mjINERTIAFROMGEOM_FALSE = 0,     // do not use; inertial element required
  mjINERTIAFROMGEOM_TRUE,          // always use; overwrite inertial element
  mjINERTIAFROMGEOM_AUTO           // use only if inertial element is missing
} mjtInertiaFromGeom;


typedef enum mjtOrientation_ {     // type of orientation specifier
  mjORIENTATION_QUAT = 0,          // quaternion
  mjORIENTATION_AXISANGLE,         // axis and angle
  mjORIENTATION_XYAXES,            // x and y axes
  mjORIENTATION_ZAXIS,             // z axis (minimal rotation)
  mjORIENTATION_EULER,             // Euler angles
} mjtOrientation;


//-------------------------------- attribute structs (mjs) -----------------------------------------

typedef struct mjsElement_ {       // element type, do not modify
  mjtObj elemtype;                 // element type
  uint64_t signature;              // compilation signature
} mjsElement;


typedef struct mjsCompiler_ {      // compiler options
  mjtByte autolimits;              // infer "limited" attribute based on range
  double boundmass;                // enforce minimum body mass
  double boundinertia;             // enforce minimum body diagonal inertia
  double settotalmass;             // rescale masses and inertias; <=0: ignore
  mjtByte balanceinertia;          // automatically impose A + B >= C rule
  mjtByte fitaabb;                 // meshfit to aabb instead of inertia box
  mjtByte degree;                  // angles in radians or degrees
  char eulerseq[3];                // sequence for euler rotations
  mjtByte discardvisual;           // discard visual geoms in parser
  mjtByte usethread;               // use multiple threads to speed up compiler
  mjtByte fusestatic;              // fuse static bodies with parent
  int inertiafromgeom;             // use geom inertias (mjtInertiaFromGeom)
  int inertiagrouprange[2];        // range of geom groups used to compute inertia
  mjtByte saveinertial;            // save explicit inertial clause for all bodies to XML
  int alignfree;                   // align free joints with inertial frame
  mjLROpt LRopt;                   // options for lengthrange computation
} mjsCompiler;


typedef struct mjSpec_ {           // model specification
  mjsElement* element;             // element type
  mjString* modelname;             // model name

  // compiler data
  mjsCompiler compiler;            // compiler options
  mjtByte strippath;               // automatically strip paths from mesh files
  mjString* meshdir;               // mesh and hfield directory
  mjString* texturedir;            // texture directory

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
  mjString* comment;               // comment at top of XML
  mjString* modelfiledir;          // path to model file

  // other
  mjtByte hasImplicitPluginElem;   // already encountered an implicit plugin sensor/actuator
} mjSpec;


typedef struct mjsOrientation_ {   // alternative orientation specifiers
  mjtOrientation type;             // active orientation specifier
  double axisangle[4];             // axis and angle
  double xyaxes[6];                // x and y axes
  double zaxis[3];                 // z axis (minimal rotation)
  double euler[3];                 // Euler angles
} mjsOrientation;


typedef struct mjsPlugin_ {        // plugin specification
  mjsElement* element;             // element type
  mjString* name;                  // instance name
  mjString* plugin_name;           // plugin name
  mjtByte active;                  // is the plugin active
  mjString* info;                  // message appended to compiler errors
} mjsPlugin;


typedef struct mjsBody_ {          // body specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* childclass;            // childclass name

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
  mjDoubleVec* userdata;           // user data
  mjtByte explicitinertial;        // whether to save the body with explicit inertial clause
  mjsPlugin plugin;                // passive force plugin
  mjString* info;                  // message appended to compiler errors
} mjsBody;


typedef struct mjsFrame_ {         // frame specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* childclass;            // childclass name
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
  mjString* info;                  // message appended to compiler errors
} mjsFrame;


typedef struct mjsJoint_ {         // joint specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjtJoint type;                   // joint type

  // kinematics
  double pos[3];                   // anchor position
  double axis[3];                  // joint axis
  double ref;                      // value at reference configuration: qpos0
  int align;                       // align free joint with body com (mjtAlignFree)

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
  mjDoubleVec* userdata;           // user data
  mjString* info;                  // message appended to compiler errors
} mjsJoint;


typedef struct mjsGeom_ {          // geom specification
  mjsElement* element;             // element type
  mjString* name;                  // name
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
  mjString* material;              // name of material
  float rgba[4];                   // rgba when material is omitted
  int group;                       // group

  // other
  mjString* hfieldname;            // heightfield attached to geom
  mjString* meshname;              // mesh attached to geom
  double fitscale;                 // scale mesh uniformly
  mjDoubleVec* userdata;           // user data
  mjsPlugin plugin;                // sdf plugin
  mjString* info;                  // message appended to compiler errors
} mjsGeom;


typedef struct mjsSite_ {          // site specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // frame, size
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
  double fromto[6];                // alternative for capsule, cylinder, box, ellipsoid
  double size[3];                  // geom size

  // visual
  mjtGeom type;                    // geom type
  mjString* material;              // name of material
  int group;                       // group
  float rgba[4];                   // rgba when material is omitted

  // other
  mjDoubleVec* userdata;           // user data
  mjString* info;                  // message appended to compiler errors
} mjsSite;


typedef struct mjsCamera_ {        // camera specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // extrinsics
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
  mjtCamLight mode;                // tracking mode
  mjString* targetbody;            // target body for tracking/targeting

  // intrinsics
  int orthographic;                // is camera orthographic
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
  mjDoubleVec* userdata;           // user data
  mjString* info;                  // message appended to compiler errors
} mjsCamera;


typedef struct mjsLight_ {         // light specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // frame
  double pos[3];                   // position
  double dir[3];                   // direction
  mjtCamLight mode;                // tracking mode
  mjString* targetbody;            // target body for targeting

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
  mjString* info;                  // message appended to compiler errorsx
} mjsLight;


typedef struct mjsFlex_ {          // flex specification
  mjsElement* element;             // element type
  mjString* name;                  // name

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
  mjString* material;              // name of material used for rendering
  double young;                    // Young's modulus
  double poisson;                  // Poisson's ratio
  double damping;                  // Rayleigh's damping
  double thickness;                // thickness (2D only)

  // mesh properties
  mjStringVec* nodebody;           // node body names
  mjStringVec* vertbody;           // vertex body names
  mjDoubleVec* node;               // node positions
  mjDoubleVec* vert;               // vertex positions
  mjIntVec* elem;                  // element vertex ids
  mjFloatVec* texcoord;            // vertex texture coordinates
  mjIntVec* elemtexcoord;          // element texture coordinates

  // other
  mjString* info;                  // message appended to compiler errors
} mjsFlex;


typedef struct mjsMesh_ {          // mesh specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* content_type;          // content type of file
  mjString* file;                  // mesh file
  double refpos[3];                // reference position
  double refquat[4];               // reference orientation
  double scale[3];                 // rescale mesh
  mjtMeshInertia inertia;          // inertia type (convex, legacy, exact, shell)
  mjtByte smoothnormal;            // do not exclude large-angle faces from normals
  int maxhullvert;                 // maximum vertex count for the convex hull
  mjFloatVec* uservert;            // user vertex data
  mjFloatVec* usernormal;          // user normal data
  mjFloatVec* usertexcoord;        // user texcoord data
  mjIntVec* userface;              // user vertex indices
  mjIntVec* userfacetexcoord;      // user texcoord indices
  mjsPlugin plugin;                // sdf plugin
  mjString* info;                  // message appended to compiler errors
} mjsMesh;


typedef struct mjsHField_ {        // height field specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* content_type;          // content type of file
  mjString* file;                  // file: (nrow, ncol, [elevation data])
  double size[4];                  // hfield size (ignore referencing geom size)
  int nrow;                        // number of rows
  int ncol;                        // number of columns
  mjFloatVec* userdata;            // user-provided elevation data
  mjString* info;                  // message appended to compiler errors
} mjsHField;



typedef struct mjsSkin_ {          // skin specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* file;                  // skin file
  mjString* material;              // name of material used for rendering
  float rgba[4];                   // rgba when material is omitted
  float inflate;                   // inflate in normal direction
  int group;                       // group for visualization

  // mesh
  mjFloatVec* vert;                // vertex positions
  mjFloatVec* texcoord;            // texture coordinates
  mjIntVec* face;                  // faces

  // skin
  mjStringVec* bodyname;           // body names
  mjFloatVec* bindpos;             // bind pos
  mjFloatVec* bindquat;            // bind quat
  mjIntVecVec* vertid;             // vertex ids
  mjFloatVecVec* vertweight;       // vertex weights

  // other
  mjString* info;                  // message appended to compiler errors
} mjsSkin;


typedef struct mjsTexture_ {       // texture specification
  mjsElement* element;             // element type
  mjString* name;                  // name
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
  int nchannel;                    // number of channels

  // method 2: single file
  mjString* content_type;          // content type of file
  mjString* file;                  // png file to load; use for all sides of cube
  int gridsize[2];                 // size of grid for composite file; (1,1)-repeat
  char gridlayout[13];             // row-major: L,R,F,B,U,D for faces; . for unused

  // method 3: separate files
  mjStringVec* cubefiles;          // different file for each side of the cube

  // method 4: from buffer read by user
  mjByteVec* data;                  // texture data

  // flip options
  mjtByte hflip;                   // horizontal flip
  mjtByte vflip;                   // vertical flip

  // other
  mjString* info;                  // message appended to compiler errors
} mjsTexture;


typedef struct mjsMaterial_ {      // material specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjStringVec* textures;           // names of textures (empty: none)
  mjtByte texuniform;              // make texture cube uniform
  float texrepeat[2];              // texture repetition for 2D mapping
  float emission;                  // emission
  float specular;                  // specular
  float shininess;                 // shininess
  float reflectance;               // reflectance
  float metallic;                  // metallic
  float roughness;                 // roughness
  float rgba[4];                   // rgba
  mjString* info;                  // message appended to compiler errors
} mjsMaterial;


typedef struct mjsPair_ {          // pair specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* geomname1;             // name of geom 1
  mjString* geomname2;             // name of geom 2

  // optional parameters: computed from geoms if not set by user
  int condim;                      // contact dimensionality
  mjtNum solref[mjNREF];           // solver reference, normal direction
  mjtNum solreffriction[mjNREF];   // solver reference, frictional directions
  mjtNum solimp[mjNIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist<margin-gap
  double friction[5];              // full contact friction
  mjString* info;                  // message appended to errors
} mjsPair;


typedef struct mjsExclude_ {       // exclude specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* bodyname1;             // name of geom 1
  mjString* bodyname2;             // name of geom 2
  mjString* info;                  // message appended to errors
} mjsExclude;


typedef struct mjsEquality_ {      // equality specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjtEq type;                      // constraint type
  double data[mjNEQDATA];          // type-dependent data
  mjtByte active;                  // is equality initially active
  mjString* name1;                 // name of object 1
  mjString* name2;                 // name of object 2
  mjtObj objtype;                  // type of both objects
  mjtNum solref[mjNREF];           // solver reference
  mjtNum solimp[mjNIMP];           // solver impedance
  mjString* info;                  // message appended to errors
} mjsEquality;


typedef struct mjsTendon_ {        // tendon specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // stiffness, damping, friction, armature
  double stiffness;                // stiffness coefficient
  double springlength[2];          // spring resting length; {-1, -1}: use qpos_spring
  double damping;                  // damping coefficient
  double frictionloss;             // friction loss
  mjtNum solref_friction[mjNREF];  // solver reference: tendon friction
  mjtNum solimp_friction[mjNIMP];  // solver impedance: tendon friction
  double armature;                 // inertia associated with tendon velocity

  // length range
  int limited;                     // does tendon have limits (mjtLimited)
  int actfrclimited;               // does tendon have actuator force limits
  double range[2];                 // length limits
  double actfrcrange[2];           // actuator force limits
  double margin;                   // margin value for tendon limit detection
  mjtNum solref_limit[mjNREF];     // solver reference: tendon limits
  mjtNum solimp_limit[mjNIMP];     // solver impedance: tendon limits

  // visual
  mjString* material;              // name of material for rendering
  double width;                    // width for rendering
  float rgba[4];                   // rgba when material is omitted
  int group;                       // group

  // other
  mjDoubleVec* userdata;           // user data
  mjString* info;                  // message appended to errors
} mjsTendon;


typedef struct mjsWrap_ {          // wrapping object specification
  mjsElement* element;             // element type
  mjString* info;                  // message appended to errors
} mjsWrap;


typedef struct mjsActuator_ {      // actuator specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // gain, bias
  mjtGain gaintype;                // gain type
  double gainprm[mjNGAIN];         // gain parameters
  mjtBias biastype;                // bias type
  double biasprm[mjNGAIN];         // bias parameters

  // activation state
  mjtDyn dyntype;                  // dynamics type
  double dynprm[mjNDYN];           // dynamics parameters
  int actdim;                      // number of activation variables
  mjtByte actearly;                // apply next activations to qfrc

  // transmission
  mjtTrn trntype;                  // transmission type
  double gear[6];                  // length and transmitted force scaling
  mjString* target;                // name of transmission target
  mjString* refsite;               // reference site, for site transmission
  mjString* slidersite;            // site defining cylinder, for slider-crank
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
  mjDoubleVec* userdata;           // user data
  mjsPlugin plugin;                // actuator plugin
  mjString* info;                  // message appended to compiler errors
} mjsActuator;


typedef struct mjsSensor_ {        // sensor specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // sensor definition
  mjtSensor type;                  // type of sensor
  mjtObj objtype;                  // type of sensorized object
  mjString* objname;               // name of sensorized object
  mjtObj reftype;                  // type of referenced object
  mjString* refname;               // name of referenced object

  // user-defined sensors
  mjtDataType datatype;            // data type for sensor measurement
  mjtStage needstage;              // compute stage needed to simulate sensor
  int dim;                         // number of scalar outputs

  // output post-processing
  double cutoff;                   // cutoff for real and positive datatypes
  double noise;                    // noise stdev

  // other
  mjDoubleVec* userdata;           // user data
  mjsPlugin plugin;                // sensor plugin
  mjString* info;                  // message appended to compiler errors
} mjsSensor;


typedef struct mjsNumeric_ {       // custom numeric field specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjDoubleVec* data;               // initialization data
  int size;                        // array size, can be bigger than data size
  mjString* info;                  // message appended to compiler errors
} mjsNumeric;


typedef struct mjsText_ {          // custom text specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* data;                  // text string
  mjString* info;                  // message appended to compiler errors
} mjsText;


typedef struct mjsTuple_ {         // tuple specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjIntVec* objtype;               // object types
  mjStringVec* objname;            // object names
  mjDoubleVec* objprm;             // object parameters
  mjString* info;                  // message appended to compiler errors
} mjsTuple;


typedef struct mjsKey_ {           // keyframe specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  double time;                     // time
  mjDoubleVec* qpos;               // qpos
  mjDoubleVec* qvel;               // qvel
  mjDoubleVec* act;                // act
  mjDoubleVec* mpos;               // mocap pos
  mjDoubleVec* mquat;              // mocap quat
  mjDoubleVec* ctrl;               // ctrl
  mjString* info;                  // message appended to compiler errors
} mjsKey;


typedef struct mjsDefault_ {       // default specification
  mjsElement* element;             // element type
  mjString* name;                  // class name
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

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_INCLUDE_MJSPEC_H_
