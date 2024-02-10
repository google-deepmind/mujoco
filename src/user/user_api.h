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

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

typedef struct _mjString* mjString;
typedef struct _mjDouble* mjDouble;
typedef struct _mjElement* mjElement;


// this is a C-API
#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------- Public structs ------------------------------------------------

// type of mesh
typedef enum _mjtGeomInertia {
  mjINERTIA_VOLUME,
  mjINERTIA_SHELL,
} mjtGeomInertia;


typedef struct _mjmOrientation {
  double axisangle[4];            // rotation axis and angle
  double xyaxes[6];               // x and y axes
  double zaxis[3];                // z axis (use minimal rotation)
  double euler[3];                // euler rotations
} mjmOrientation;


typedef struct _mjmPlugin {
  bool active;
  mjString name;
  mjString instance_name;
  mjElement instance;
} mjmPlugin;


typedef struct _mjmBody {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  double pos[3];                  // frame position
  double quat[4];                 // frame orientation
  mjtByte mocap;                  // is this a mocap body
  mjmOrientation alt;             // frame alternative orientation
  double gravcomp;                // gravity compensation
  mjDouble userdata;              // user data
  double ipos[3];                 // inertial frame position
  double iquat[4];                // inertial frame orientation
  double mass;                    // mass
  double inertia[3];              // diagonal inertia (in i-frame)
  mjmOrientation ialt;            // inertial frame alternative orientation
  double fullinertia[6];          // non-axis-aligned inertia matrix
  mjtByte explicitinertial;       // whether to save the body with an explicit inertial clause
  mjmPlugin plugin;               // passive force plugin
  mjString info;                  // message appended to errors
} mjmBody;


typedef struct _mjmJoint {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name

  // joint properties
  mjtJoint type;                   // type of Joint
  int group;                       // used for rendering
  int limited;                     // does joint have limits: 0 false, 1 true, 2 auto
  int actfrclimited;               // are actuator forces on joints limited: 0 false, 1 true, 2 auto
  double pos[3];                   // anchor position
  double axis[3];                  // joint axis
  double stiffness;                // stiffness coefficient
  double springdamper[2];          // timeconst, dampratio
  double range[2];                 // joint limits
  double actfrcrange[2];           // actuator force limits
  mjtNum solref_limit[mjNREF];     // solver reference: joint limits
  mjtNum solimp_limit[mjNIMP];     // solver impedance: joint limits
  mjtNum solref_friction[mjNREF];  // solver reference: dof friction
  mjtNum solimp_friction[mjNIMP];  // solver impedance: dof friction
  double margin;                   // margin value for joint limit detection
  double ref;                      // value at reference configuration: qpos0
  double springref;                // spring reference value: qpos_spring
  mjDouble userdata;               // user data

  // dof properties
  double armature;                 // armature inertia (mass for slider)
  double damping;                  // damping coefficient
  double frictionloss;             // friction loss

  // other attributes
  mjString info;                  // message appended to errors
  double urdfeffort;              // store effort field from urdf
} mjmJoint;


typedef struct _mjmGeom {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // classname
  mjtGeom type;                   // geom type
  double pos[3];                  // position
  double quat[4];                 // orientation
  mjmOrientation alt;             // alternative orientation specifications
  int contype;                    // contact type
  int conaffinity;                // contact affinity
  int condim;                     // contact dimensionality
  int group;                      // used for rendering
  int priority;                   // contact priority
  double size[3];                 // geom-specific size parameters
  double friction[3];             // one-sided friction coefficients: slide, roll, spin
  double solmix;                  // solver mixing for contact pairs
  mjtNum solref[mjNREF];          // solver reference
  mjtNum solimp[mjNIMP];          // solver impedance
  double mass;                    // used to compute density
  double density;                 // used to compute mass and inertia (from volume)
  double fromto[6];               // alternative for capsule, cylinder, box, ellipsoid
  double margin;                  // margin for contact detection
  double gap;                     // include in solver if dist<margin-gap
  mjtNum fluid_ellipsoid;         // whether ellipsoid-fluid model is active
  mjtNum fluid_coefs[5];          // ellipsoid-fluid interaction coefs
  mjString material;              // name of material used for rendering
  mjString hfieldname;            // hfield attached to geom
  mjString meshname;              // mesh attached to geom
  double fitscale;                // scale mesh uniformly
  mjDouble userdata;              // user data
  float rgba[4];                  // rgba when material is omitted
  mjtGeomInertia typeinertia;     // selects between surface and volume inertia
  mjmPlugin plugin;               // sdf plugin
  mjString info;
} mjmGeom;


typedef struct _mjmSite {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  mjtGeom type;                   // geom type for rendering
  int group;                      // group id, used for visualization
  double pos[3];                  // position
  double quat[4];                 // orientation
  mjString material;              // name of material for rendering
  double size[3];                 // geom size for rendering
  double fromto[6];               // alternative for capsule, cylinder, box, ellipsoid
  mjmOrientation alt;             // alternative orientation specification
  float rgba[4];                  // rgba when material is omitted
  mjDouble userdata;              // user data
  mjString info;                  // message appended to errors
} mjmSite;


typedef struct _mjmCamera {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  mjString info;                  // message appended to errors
  mjtCamLight mode;               // tracking mode
  mjString targetbody;            // target body for orientation
  double fovy;                    // y-field of view
  double ipd;                     // inter-pupilary distance
  double pos[3];                  // position
  double quat[4];                 // orientation
  float intrinsic[4];             // camera intrinsics [length]
  float sensor_size[2];           // sensor size [length]
  float resolution[2];            // resolution [pixel]
  float focal_length[2];          // focal length [length]
  float focal_pixel[2];           // focal length [pixel]
  float principal_length[2];      // principal point [length]
  float principal_pixel[2];       // principal point [pixel]
  mjDouble userdata;              // user data
  mjmOrientation alt;             // alternative orientation specification
} mjmCamera;


typedef struct _mjmLight {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  mjString info;                  // message appended to errors
  mjtCamLight mode;               // tracking mode
  mjString targetbody;            // target body for orientation
  mjtByte directional;            // directional light
  mjtByte castshadow;             // does light cast shadows
  mjtByte active;                 // is light active
  double pos[3];                  // position
  double dir[3];                  // direction
  float attenuation[3];           // OpenGL attenuation (quadratic model)
  float cutoff;                   // OpenGL cutoff
  float exponent;                 // OpenGL exponent
  float ambient[3];               // ambient color
  float diffuse[3];               // diffuse color
  float specular[3];              // specular color
} mjmLight;


typedef struct _mjmEquality {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  mjtEq type;                     // constraint type
  mjString name1;                 // name of object 1
  mjString name2;                 // name of object 2
  mjtByte active;                 // initial activation state
  mjtNum solref[mjNREF];          // solver reference
  mjtNum solimp[mjNIMP];          // solver impedance
  double data[mjNEQDATA];         // type-dependent data
  mjString info;                  // message appended to errors
} mjmEquality;


typedef struct _mjmTendon {
  mjElement element;               // compiler only, do not modify
  mjString name;                   // name
  mjString classname;              // class name
  int group;                       // group for visualization
  int limited;                     // does tendon have limits: 0 false, 1 true, 2 auto
  double width;                    // width for rendering
  mjtNum solref_limit[mjNREF];     // solver reference: tendon limits
  mjtNum solimp_limit[mjNIMP];     // solver impedance: tendon limits
  mjtNum solref_friction[mjNREF];  // solver reference: tendon friction
  mjtNum solimp_friction[mjNIMP];  // solver impedance: tendon friction
  double range[2];                 // length limits
  double margin;                   // margin value for tendon limit detection
  double stiffness;                // stiffness coefficient
  double damping;                  // damping coefficient
  double frictionloss;             // friction loss
  double springlength[2];          // spring resting length; {-1, -1}: use qpos_spring
  mjString material;               // name of material for rendering
  mjDouble userdata;               // user data
  float rgba[4];                   // rgba when material is omitted
  mjString info;                   // message appended to errors
} mjmTendon;


typedef struct _mjmWrap {
  mjElement element;               // compiler only, do not modify
  mjString name;                   // name
  mjString classname;              // class name
  mjString info;                   // message appended to errors
} mjmWrap;


typedef struct _mjmActuator {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  mjString info;                  // message appended to errors
  int group;                      // group for visualization
  int ctrllimited;                // are control limits defined: 0 false, 1 true, 2 auto
  int forcelimited;               // are force limits defined: 0 false, 1 true, 2 auto
  int actlimited;                 // are activation limits defined: 0 false, 1 true, 2 auto
  int actdim;                     // dimension of associated activations
  int plugin_actdim;              // actuator state size for plugins
  mjtDyn dyntype;                 // dynamics type
  mjtTrn trntype;                 // transmission type
  mjtGain gaintype;               // gain type
  mjtBias biastype;               // bias type
  double dynprm[mjNDYN];          // dynamics parameters
  double gainprm[mjNGAIN];        // gain parameters
  double biasprm[mjNGAIN];        // bias parameters
  mjtByte actearly;               // apply activations to qfrc instantly
  double ctrlrange[2];            // control range
  double forcerange[2];           // force range
  double actrange[2];             // activation range
  double lengthrange[2];          // length range
  double gear[6];                 // length and transmitted force scaling
  double cranklength;             // crank length, for slider-crank only
  mjDouble userdata;              // user data
  mjString target;                // transmission target name
  mjString slidersite;            // site defining cylinder, for slider-crank only
  mjString refsite;               // reference site, for site transmission only
  mjmPlugin plugin;               // actuator plugin
} mjmActuator;


typedef struct _mjmSensor {
  mjElement element;              // compiler only, do not modify
  mjString name;                  // name
  mjString classname;             // class name
  mjString info;                  // message appended to errors
  mjtSensor type;                 // type of sensor
  mjtDataType datatype;           // data type for sensor measurement
  mjtStage needstage;             // compute stage needed to simulate sensor
  mjtObj objtype;                 // type of sensorized object
  mjString objname;               // name of sensorized object
  mjtObj reftype;                 // type of referenced object
  mjString refname;               // name of referenced object
  int dim;                        // number of scalar outputs
  double cutoff;                  // cutoff for real and positive datatypes
  double noise;                   // noise stdev
  mjDouble userdata;              // user data
  mjmPlugin plugin;               // sensor plugin
} mjmSensor;


//---------------------------------- Public API ----------------------------------------------------

// Create model.
MJAPI void* mjm_createModel();

// Delete model.
MJAPI void mjm_deleteModel(void* modelspec);

// Copy spec into private attributes.
MJAPI void mjm_finalize(mjElement object);

// Add child body to body, return child spec.
MJAPI mjmBody* mjm_addBody(mjmBody* body, void* defspec);

// Add site to body, return site spec.
MJAPI mjmSite* mjm_addSite(mjmBody* body, void* defspec);

// Add joint to body.
MJAPI mjmJoint* mjm_addJoint(mjmBody* body, void* defspec);

// Add freejoint to body.
MJAPI mjmJoint* mjm_addFreeJoint(mjmBody* body);

// Add geom to body.
MJAPI mjmGeom* mjm_addGeom(mjmBody* body, void* defspec);

// Add camera to body.
MJAPI mjmCamera* mjm_addCamera(mjmBody* body, void* defspec);

// Add light to body.
MJAPI mjmLight* mjm_addLight(mjmBody* body, void* defspec);

// Add frame to body.
MJAPI void* mjm_addFrame(mjmBody* body, void* parentframe);

// Add equality to model.
MJAPI mjmEquality* mjm_addEquality(void* model, void* defspec);

// Add tendon to model.
MJAPI mjmTendon* mjm_addTendon(void* model, void* defspec);

// Wrap site using tendon.
MJAPI mjmWrap* mjm_wrapSite(mjmTendon* tendon, const char* name);

// Wrap geom using tendon.
MJAPI mjmWrap* mjm_wrapGeom(mjmTendon* tendon, const char* name, const char* sidesite);

// Wrap joint using tendon.
MJAPI mjmWrap* mjm_wrapJoint(mjmTendon* tendon, const char* name, double coef);

// Wrap pulley using tendon.
MJAPI mjmWrap* mjm_wrapPulley(mjmTendon* tendon, double divisor);

// Add actuator to model.
MJAPI mjmActuator* mjm_addActuator(void* model, void* defspec);

// Add sensor to model.
MJAPI mjmSensor* mjm_addSensor(void* model);

// Add plugin to model.
MJAPI mjElement mjm_addPlugin(void* model);

// Get model from body.
MJAPI void* mjm_getModel(mjmBody* body);

// Get default corresponding to an mjElement.
MJAPI void* mjm_getDefault(mjElement element);

// Finding body with given name in model.
MJAPI mjmBody* mjm_findBody(void* modelspec, const char* name);

// Finding body with given name in body.
MJAPI mjmBody* mjm_findChild(mjmBody* body, const char* name);

// Get element id.
MJAPI int mjm_getId(mjElement element);

// Copy input text to destination string.
MJAPI void mjm_setString(mjString dest, const char* text);

// Copy input array to destination vector.
MJAPI void mjm_setDouble(mjDouble dest, const double* array, int size);

// Get string contents.
MJAPI const char* mjm_getString(mjString source);

// Get double array contents and optionally its size.
MJAPI const double* mjm_getDouble(mjDouble source, int* size);

// Set default.
MJAPI void mjm_setDefault(mjElement element, void* defspec);

// Set frame.
MJAPI void mjm_setFrame(mjElement dest, void* frame);

// Compute quat and inertia from body->fullinertia.
MJAPI const char* mjm_setFullInertia(mjmBody* body, double quat[4], double inertia[3]);


//---------------------------------- Initialization functions --------------------------------------

// Default body attributes.
MJAPI void mjm_defaultBody(mjmBody& body);

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

// Default equality attributes.
MJAPI void mjm_defaultEquality(mjmEquality& equality);

// Default tendon attributes.
MJAPI void mjm_defaultTendon(mjmTendon& tendon);

// Default actuator attributes.
MJAPI void mjm_defaultActuator(mjmActuator& actuator);

// Default sensor attributes.
MJAPI void mjm_defaultSensor(mjmSensor& sensor);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_USER_USER_API_H_
