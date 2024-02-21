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

#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include "user/user_api.h"
#include "user/user_objects.h"

typedef std::map<std::string, int, std::less<> > mjKeyMap;
typedef std::array<mjKeyMap, mjNOBJECT> mjListKeyMap;



//---------------------------------- class mjCModel ------------------------------------------------
// mjCModel contains everything needed to generate the low-level model.
// It can be constructed manually by calling 'Add' functions and setting
// the public fields of the various objects.  Alternatively it can constructed
// by loading an XML file via mjCXML.  Once an mjCModel object is
// constructed, 'Compile' can be called to generate the corresponding mjModel object
// (which is the low-level model).  The mjCModel object can then be deleted.

class mjCModel : private mjmModel {
  friend class mjCBody;
  friend class mjCCamera;
  friend class mjCGeom;
  friend class mjCFlex;
  friend class mjCHField;
  friend class mjCFrame;
  friend class mjCJoint;
  friend class mjCEquality;
  friend class mjCMesh;
  friend class mjCSkin;
  friend class mjCSite;
  friend class mjCTendon;
  friend class mjCTexture;
  friend class mjCActuator;
  friend class mjCSensor;
  friend class mjCDef;
  friend class mjXReader;
  friend class mjXWriter;

 public:
  mjCModel();                                          // constructor
  ~mjCModel();                                         // destructor
  void CopyFromSpec();                                 // copy spec to private attributes
  void PointToLocal();

  mjmModel spec;

  mjModel*    Compile(const mjVFS* vfs = 0);           // COMPILER: construct mjModel
  bool        CopyBack(const mjModel*);                // DECOMPILER: copy numeric back
  void        FuseStatic(void);                        // fuse static bodies with parent
  void        FuseReindex(mjCBody* body);              // reindex elements during fuse


  //------------------------ API for adding model elements
  mjCFlex*     AddFlex(void);                          // flex
  mjCMesh*     AddMesh(mjCDef* def = 0);               // mesh
  mjCSkin*     AddSkin(void);                          // skin
  mjCHField*   AddHField(void);                        // heightfield
  mjCTexture*  AddTexture(void);                       // texture
  mjCMaterial* AddMaterial(mjCDef* def = 0);           // material
  mjCPair*     AddPair(mjCDef* def = 0);               // geom pair for inclusion
  mjCBodyPair* AddExclude(void);                       // body pair for exclusion
  mjCEquality* AddEquality(mjCDef* def = 0);           // equality constraint
  mjCTendon*   AddTendon(mjCDef* def = 0);             // tendon
  mjCActuator* AddActuator(mjCDef* def = 0);           // actuator
  mjCSensor*   AddSensor(void);                        // sensor
  mjCNumeric*  AddNumeric(void);                       // custom numeric
  mjCText*     AddText(void);                          // custom text
  mjCTuple*    AddTuple(void);                         // custom tuple
  mjCKey*      AddKey(void);                           // keyframe
  mjCPlugin*   AddPlugin(void);                        // plugin instance

  //------------------------ API for deleting model elements
  template <class T>
  void Delete(std::vector<T*>& elements,
              const std::vector<bool>& discard);       // delete elements marked as discard=true

  template <class T>
  void DeleteAll(std::vector<T*>& elements);           // delete all elements

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

  //------------------------ getters
  std::string get_meshdir(void) const { return meshdir_; }
  std::string get_texturedir(void) const { return texturedir_; }

  //------------------------ API for plugins
  void        ResolvePlugin(mjCBase* obj,     // resolve plugin instance, create a new one if needed
                            const std::string& plugin_name,
                            const std::string& plugin_instance_name,
                            mjCPlugin** plugin_instance);


  //------------------------ global data
  std::string comment;            // comment at top of XML
  std::string modelfiledir;       // path to model file
  std::vector<mjCDef*> defaults;  // settings for each defaults class

 private:
  void TryCompile(mjModel*& m, mjData*& d, const mjVFS* vfs);
  mjModel* _Compile(const mjVFS* vfs);

  void Clear(void);               // clear objects allocated by Compile

  template <class T>              // add object of any type
  T* AddObject(std::vector<T*>& list, std::string type);

  template <class T>              // add object of any type, with def parameter
  T* AddObjectDef(std::vector<T*>& list, std::string type, mjCDef* def);

  template<class T>              // if asset name is missing, set to filename
  void SetDefaultNames(std::vector<T*>& assets);

  template <class T>             // delete material from object
  void DeleteMaterial(std::vector<T*>& list, std::string_view name = "");

  //------------------------ compile phases
  void MakeLists(mjCBody* body);        // make lists of bodies, geoms, joints, sites
  void IndexAssets(bool discard);       // convert asset names into indices
  void CheckEmptyNames(void);           // check empty names
  void SetSizes(void);                  // compute sizes
  void AutoSpringDamper(mjModel*);      // automatic stiffness and damping computation
  void LengthRange(mjModel*, mjData*);  // compute actuator lengthrange
  void CopyNames(mjModel*);             // copy names, compute name addresses
  void CopyPaths(mjModel*);             // copy paths, compute path addresses
  void CopyObjects(mjModel*);           // copy objects outside kinematic tree
  void CopyTree(mjModel*);              // copy objects inside kinematic tree

  //------------------------ sizes
  // sizes set from object list lengths
  int nbody;                      // number of bodies
  int njnt;                       // number of joints
  int ngeom;                      // number of geoms
  int nsite;                      // number of sites
  int ncam;                       // number of cameras
  int nlight;                     // number of lights
  int nflex;                      // number of flexes
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
  int nmocap;                     // number of mocap bodies
  int nplugin;                    // number of plugin instances

  // sizes computed by Compile
  int nq;                         // number of generalized coordinates = dim(qpos)
  int nv;                         // number of degrees of freedom = dim(qvel)
  int nu;                         // number of actuators/controls
  int na;                         // number of activation variables
  int nbvh;                       // number of total boundary volume hierarchies
  int nbvhstatic;                 // number of static boundary volume hierarchies
  int nbvhdynamic;                // number of dynamic boundary volume hierarchies
  int nflexvert;                  // number of vertices in all flexes
  int nflexedge;                  // number of edges in all flexes
  int nflexelem;                  // number of elements in all flexes
  int nflexelemdata;              // number of element vertex ids in all flexes
  int nflexshelldata;             // number of shell fragment vertex ids in all flexes
  int nflexevpair;                // number of element-vertex pairs in all flexes
  int nflextexcoord;              // number of vertex texture coordinates in all flexes
  int nmeshvert;                  // number of vertices in all meshes
  int nmeshnormal;                // number of normals in all meshes
  int nmeshtexcoord;              // number of texture coordinates in all meshes
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
  int npluginattr;                // number of chars in all plugin config attributes
  int nnames;                     // number of chars in all names
  int npaths;                     // number of chars in all paths
  int nM;                         // number of non-zeros in sparse inertia matrix
  int nD;                         // number of non-zeros in sparse dof-dof matrix
  int nB;                         // number of non-zeros in sparse body-dof matrix

  //------------------------ object lists
  // objects created here
  std::vector<mjCFlex*>     flexes;      // list of flexes
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

  std::vector<std::pair<const mjpPlugin*, int>> active_plugins;  // list of active plugins
  std::vector<mjCPlugin*>   plugins;     // list of plugin instances

  // pointers to objects created inside kinematic tree
  std::vector<mjCBody*>   bodies;   // list of bodies
  std::vector<mjCJoint*>  joints;   // list of joints allowing motion relative to parent
  std::vector<mjCGeom*>   geoms;    // list of geoms attached to this body
  std::vector<mjCSite*>   sites;    // list of sites attached to this body
  std::vector<mjCCamera*> cameras;  // list of cameras
  std::vector<mjCLight*>  lights;   // list of lights

  //------------------------ internal variables

  // array of pointers to each object list (enumerated by type)
  std::array<std::vector<mjCBase*>*, mjNOBJECT> object_lists;

  // statistics, as computed by mj_setConst
  double meaninertia_auto;        // mean diagonal inertia, as computed by mj_setConst
  double meanmass_auto;           // mean body mass, as computed by mj_setConst
  double meansize_auto;           // mean body size, as computed by mj_setConst
  double extent_auto;             // spatial extent, as computed by mj_setConst
  double center_auto[3];          // center of model, as computed by mj_setConst

  // map from object names to ids
  mjListKeyMap ids;

  bool hasImplicitPluginElem;     // already encountered an implicit plugin sensor/actuator
  bool compiled;                  // already compiled flag (cannot be compiled again)
  mjCError errInfo;               // last error info
  int fixCount;                   // how many bodies have been fixed
  std::vector<mjtNum> qpos0;      // save qpos0, to recognize changed key_qpos in write

  // variable-size attributes
  std::string modelname_;
  std::string meshdir_;
  std::string texturedir_;
  std::string spec_modelname_;
  std::string spec_meshdir_;
  std::string spec_texturedir_;
};
#endif  // MUJOCO_SRC_USER_USER_MODEL_H_
