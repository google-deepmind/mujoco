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

#ifndef MUJOCO_SRC_USER_USER_MODEL_H_
#define MUJOCO_SRC_USER_USER_MODEL_H_

#include <string>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include "user/user_objects.h"

typedef enum _mjtInertiaFromGeom {
  mjINERTIAFROMGEOM_FALSE = 0,    // do not use; inertial element required
  mjINERTIAFROMGEOM_TRUE,         // always use; overwrite inertial element
  mjINERTIAFROMGEOM_AUTO          // use only if inertial element is missing
} mjtInertiaFromGeom;



//---------------------------------- class mjCModel ------------------------------------------------
// mjCModel contains everything needed to generate the low-level model.
// It can be constructed manually by calling 'Add' functions and setting
// the public fields of the various objects.  Alternatively it can constructed
// by loading an XML file via mjCXML.  Once an mjCModel object is
// constructed, 'Compile' can be called to generate the corresponding mjModel object
// (which is the low-level model).  The mjCModel object can then be deleted.

class mjCModel {
  friend class mjCBody;
  friend class mjCJoint;
  friend class mjCGeom;
  friend class mjCMesh;
  friend class mjCSkin;
  friend class mjCHField;
  friend class mjCPair;
  friend class mjCBodyPair;
  friend class mjCSite;
  friend class mjCEquality;
  friend class mjCTendon;
  friend class mjCWrap;
  friend class mjCActuator;
  friend class mjCSensor;
  friend class mjCNumeric;
  friend class mjCTuple;
  friend class mjCKey;
  friend class mjXReader;
  friend class mjXWriter;

 public:
  mjCModel();                                 // constructor
  ~mjCModel();                                // destructor

  mjModel*    Compile(const mjVFS* vfs = 0);  // COMPILER: construct mjModel
  bool        CopyBack(const mjModel*);       // DECOMPILER: copy numeric back
  void        FuseStatic(void);               // fuse static bodies with parent
  void        FuseReindex(mjCBody* body);     // reindex elements during fuse


  //------------------------ API for adding model elements
  mjCMesh*    AddMesh(mjCDef* def = 0);               // mesh
  mjCSkin*    AddSkin(void);                          // skin
  mjCHField*  AddHField(void);                        // heightfield
  mjCTexture* AddTexture(void);                       // texture
  mjCMaterial*AddMaterial(mjCDef* def = 0);           // material
  mjCPair*    AddPair(mjCDef* def = 0);               // geom pair for inclusion
  mjCBodyPair*AddExclude(void);                       // body pair for exclusion
  mjCEquality*AddEquality(mjCDef* def = 0);           // equality constraint
  mjCTendon*  AddTendon(mjCDef* def = 0);             // tendon
  mjCActuator*AddActuator(mjCDef* def = 0);           // actuator
  mjCSensor*  AddSensor(void);                        // sensor
  mjCNumeric* AddNumeric(void);                       // custom numeric
  mjCText*    AddText(void);                          // custom text
  mjCTuple*   AddTuple(void);                         // custom tuple
  mjCKey*     AddKey(void);                           // keyframe

  //------------------------ API for access to model elements (outside tree)
  int         NumObjects(mjtObj type);                // number of objects in specified list
  mjCBase*    GetObject(mjtObj type, int id);         // pointer to specified object

  //------------------------ API for access to other variables
  bool        IsCompiled(void);                       // is model already compiled
  int         GetFixed(void);                         // number of fixed massless bodies
  mjCError    GetError(void);                         // copy of error object
  mjCBody*    GetWorld(void);                         // pointer to world body
  mjCDef*     FindDef(std::string name);              // find default class name
  mjCDef*     AddDef(std::string name, int parentid); // add default class to array
  mjCBase*    FindObject(mjtObj type, std::string name);  // find object given type and name
  bool        IsNullPose(const mjtNum* pos, const mjtNum* quat); // detect null pose


  //------------------------ global data
  std::string comment;            // comment at top of XML
  std::string modelfiledir;       // path to model file
  std::vector<mjCDef*> defaults;  // settings for each defaults class

  //------------------------ compiler settings
  bool autolimits;                // infer "limited" attribute based on range
  double boundmass;               // enfore minimum body mass
  double boundinertia;            // enfore minimum body diagonal inertia
  double settotalmass;            // rescale masses and inertias; <=0: ignore
  bool balanceinertia;            // automatically impose A + B >= C rule
  bool strippath;                 // automatically strip paths from mesh files
  bool fitaabb;                   // meshfit to aabb instead of inertia box
  bool global;                    // local or global coordinates
  bool degree;                    // angles in radians or degrees
  char euler[3];                  // sequence for euler rotations
  std::string meshdir;            // mesh and hfield directory
  std::string texturedir;         // texture directory
  bool discardvisual;             // discard visual geoms in parser
  bool convexhull;                // compute mesh convex hulls
  bool usethread;                 // use multiple threads to speed up compiler
  bool fusestatic;                // fuse static bodies with parent
  int inertiafromgeom;            // use geom inertias (mjtInertiaFromGeom)
  int inertiagrouprange[2];       // range of geom groups used to compute inertia
  bool exactmeshinertia;          // if false, use old formula
  mjLROpt LRopt;                  // options for lengthrange computation

  //------------------------ statistics override (if defined)
  double meaninertia;             // mean diagonal inertia
  double meanmass;                // mean body mass
  double meansize;                // mean body size
  double extent;                  // spatial extent
  double center[3];               // center of model

  //------------------------ engine data
  std::string modelname;          // model name
  mjOption option;                // options
  mjVisual visual;                // visual options
  int nemax;                      // max number of equality constraints
  int njmax;                      // max number of constraints (Jacobian rows)
  int nconmax;                    // max number of detected contacts (mjContact array size)
  int nstack;                     // number of fields in mjData stack
  int nuserdata;                  // number extra fields in mjData
  int nuser_body;                 // number of mjtNums in body_user
  int nuser_jnt;                  // number of mjtNums in jnt_user
  int nuser_geom;                 // number of mjtNums in geom_user
  int nuser_site;                 // number of mjtNums in site_user
  int nuser_cam;                  // number of mjtNums in cam_user
  int nuser_tendon;               // number of mjtNums in tendon_user
  int nuser_actuator;             // number of mjtNums in actuator_user
  int nuser_sensor;               // number of mjtNums in sensor_user

 private:
  void TryCompile(mjModel*& m, mjData*& d, const mjVFS* vfs);

  void Clear(void);               // clear objects allocated by Compile

  template <class T>              // add object of any type
  T* AddObject(std::vector<T*>& list, std::string type);

  template <class T>              // add object of any type, with def parameter
  T* AddObjectDef(std::vector<T*>& list, std::string type, mjCDef* def);

  //------------------------ compile phases
  void MakeLists(mjCBody* body);  // make lists of bodies, geoms, joints, sites
  void IndexAssets(void);         // convert asset names into indices
  void SetDefaultNames(void);     // if mesh or hfield name is missing, set to filename
  void SetSizes(void);            // compute sizes
  void AutoSpringDamper(mjModel*);// automatic stiffness and damping computation
  void LengthRange(mjModel*, mjData*); // compute actuator lengthrange
  void CopyNames(mjModel*);       // copy names, compute name addresses
  void CopyObjects(mjModel*);     // copy objects outside kinematic tree
  void CopyTree(mjModel*);        // copy objects inside kinematic tree

  //------------------------ sizes
  // sizes set from object list lengths
  int nbody;                      // number of bodies
  int njnt;                       // number of joints
  int ngeom;                      // number of geoms
  int nsite;                      // number of sites
  int ncam;                       // number of cameras
  int nlight;                     // number of lights
  int nmesh;                      // number of meshes
  int nskin;                      // number of skins
  int nhfield;                    // number of height fields
  int ntex;                       // number of textures
  int nmat;                       // number of materials
  int npair;                      // number of geom pairs in pair array
  int nexclude;                   // number of excluded body pairs
  int neq;                        // number of equality constraints
  int ntendon;                    // number of tendons
  int nsensor;                    // number of sensors
  int nnumeric;                   // number of numeric fields
  int ntext;                      // number of text fields
  int ntuple;                     // number of tuple fields
  int nkey;                       // number of keyframes
  int nmocap;                     // number of mocap bodies

  // sizes computed by Compile
  int nq;                         // number of generalized coordinates = dim(qpos)
  int nv;                         // number of degrees of freedom = dim(qvel)
  int nu;                         // number of actuators/controls
  int na;                         // number of activation variables
  int nmeshvert;                  // number of vertices in all meshes
  int nmeshtexvert;               // number of texture coordinates in all meshes
  int nmeshface;                  // number of triangular faces in all meshes
  int nmeshgraph;                 // number of shorts in mesh auxiliary data
  int nskinvert;                  // number of vertices in all skins
  int nskintexvert;               // number of vertices with texcoord in all skins
  int nskinface;                  // number of faces in all skins
  int nskinbone;                  // number of bones in all skins
  int nskinbonevert;              // number of vertices in all skins
  int nhfielddata;                // number of data points in all hfields
  int ntexdata;                   // number of texture bytes
  int nwrap;                      // number of wrap objects in all tendon paths
  int nsensordata;                // number of mjtNums in sensor data vector
  int nnumericdata;               // number of mjtNums in all custom fields
  int ntextdata;                  // number of chars in all text fields, including 0
  int ntupledata;                 // number of objects in all tuple fields
  int nnames;                     // number of chars in all names
  int nM;                         // number of non-zeros in sparse inertia matrix
  int nD;                         // number of non-zeros in sparse derivative matrix

  //------------------------ object lists
  // objects created here
  std::vector<mjCMesh*>     meshes;      // list of meshes
  std::vector<mjCSkin*>     skins;       // list of skins
  std::vector<mjCHField*>   hfields;     // list of height fields
  std::vector<mjCTexture*>  textures;    // list of textures
  std::vector<mjCMaterial*> materials;   // list of materials
  std::vector<mjCPair*>     pairs;       // list of geom pairs to include
  std::vector<mjCBodyPair*> excludes;    // list of body pairs to exclude
  std::vector<mjCEquality*> equalities;  // list of equality constraints
  std::vector<mjCTendon*>   tendons;     // list of tendons
  std::vector<mjCActuator*> actuators;   // list of actuators
  std::vector<mjCSensor*>   sensors;     // list of sensors
  std::vector<mjCNumeric*>  numerics;    // list of numeric fields
  std::vector<mjCText*>     texts;       // list of text fields
  std::vector<mjCTuple*>    tuples;      // list of tuple fields
  std::vector<mjCKey*>      keys;        // list of keyframe fields

  // pointers to objects created inside kinematic tree
  std::vector<mjCBody*>   bodies;   // list of bodies
  std::vector<mjCJoint*>  joints;   // list of joints allowing motion relative to parent
  std::vector<mjCGeom*>   geoms;    // list of geoms attached to this body
  std::vector<mjCSite*>   sites;    // list of sites attached to this body
  std::vector<mjCCamera*> cameras;  // list of cameras
  std::vector<mjCLight*>  lights;   // list of lights

  //------------------------ internal variables
  bool compiled;                  // already compiled flag (cannot be compiled again)
  mjCError errInfo;               // last error info
  int fixCount;                   // how many bodies have been fixed
  std::vector<mjtNum> qpos0;      // save qpos0, to recognize changed key_qpos in write
};
#endif  // MUJOCO_SRC_USER_USER_MODEL_H_
