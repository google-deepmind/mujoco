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

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjspec.h>
#include "user/user_objects.h"

typedef std::map<std::string, int, std::less<> > mjKeyMap;
typedef std::array<mjKeyMap, mjNOBJECT> mjListKeyMap;

typedef struct mjKeyInfo_ {
  std::string name;
  double time;
  bool qpos;
  bool qvel;
  bool act;
  bool ctrl;
  bool mpos;
  bool mquat;
} mjKeyInfo;

class mjCModel_ : public mjsElement {
 public:
  // attach namespaces
  std::string prefix;
  std::string suffix;

 protected:
  bool compiled;      // already compiled flag

  // sizes set from object list lengths
  int nbody;     // number of bodies
  int njnt;      // number of joints
  int ngeom;     // number of geoms
  int nsite;     // number of sites
  int ncam;      // number of cameras
  int nlight;    // number of lights
  int nflex;     // number of flexes
  int nmesh;     // number of meshes
  int nskin;     // number of skins
  int nhfield;   // number of height fields
  int ntex;      // number of textures
  int nmat;      // number of materials
  int npair;     // number of geom pairs in pair array
  int nexclude;  // number of excluded body pairs
  int neq;       // number of equality constraints
  int ntendon;   // number of tendons
  int nsensor;   // number of sensors
  int nnumeric;  // number of numeric fields
  int ntext;     // number of text fields
  int ntuple;    // number of tuple fields
  int nmocap;    // number of mocap bodies
  int nplugin;   // number of plugin instances

  // sizes computed by Compile
  int nq;              // number of generalized coordinates = dim(qpos)
  int nv;              // number of degrees of freedom = dim(qvel)
  int nu;              // number of actuators/controls
  int na;              // number of activation variables
  int nbvh;            // number of total boundary volume hierarchies
  int nbvhstatic;      // number of static boundary volume hierarchies
  int nbvhdynamic;     // number of dynamic boundary volume hierarchies
  int nflexnode;       // number of nodes in all flexes
  int nflexvert;       // number of vertices in all flexes
  int nflexedge;       // number of edges in all flexes
  int nflexelem;       // number of elements in all flexes
  int nflexelemdata;   // number of element vertex ids in all flexes
  int nflexelemedge;   // number of element edges in all flexes
  int nflexshelldata;  // number of shell fragment vertex ids in all flexes
  int nflexevpair;     // number of element-vertex pairs in all flexes
  int nflextexcoord;   // number of vertex texture coordinates in all flexes
  int nmeshvert;       // number of vertices in all meshes
  int nmeshnormal;     // number of normals in all meshes
  int nmeshtexcoord;   // number of texture coordinates in all meshes
  int nmeshface;       // number of triangular faces in all meshes
  int nmeshpoly;       // number of polygon faces in all meshes
  int nmeshgraph;      // number of ints in mesh auxiliary data
  int nmeshpolyvert;   // number of vertices in all polygon faces
  int nmeshpolymap;    // number of polygons in vertex map
  int nskinvert;       // number of vertices in all skins
  int nskintexvert;    // number of vertices with texcoord in all skins
  int nskinface;       // number of faces in all skins
  int nskinbone;       // number of bones in all skins
  int nskinbonevert;   // number of vertices in all skins
  int nhfielddata;     // number of data points in all hfields
  int ntexdata;        // number of texture bytes
  int nwrap;           // number of wrap objects in all tendon paths
  int nsensordata;     // number of mjtNums in sensor data vector
  int nnumericdata;    // number of mjtNums in all custom fields
  int ntextdata;       // number of chars in all text fields, including 0
  int ntupledata;      // number of objects in all tuple fields
  int npluginattr;     // number of chars in all plugin config attributes
  int nnames;          // number of chars in all names
  int npaths;          // number of chars in all paths
  int nM;              // number of non-zeros in sparse inertia matrix
  int nB;              // number of non-zeros in sparse body-dof matrix
  int nC;              // number of non-zeros in reduced sparse dof-dof matrix
  int nD;              // number of non-zeros in sparse dof-dof matrix
  int nJmom;           // number of non-zeros in sparse actuator_moment matrix

  // statistics, as computed by mj_setConst
  double meaninertia_auto;  // mean diagonal inertia, as computed by mj_setConst
  double meanmass_auto;     // mean body mass, as computed by mj_setConst
  double meansize_auto;     // mean body size, as computed by mj_setConst
  double extent_auto;       // spatial extent, as computed by mj_setConst
  double center_auto[3];    // center of model, as computed by mj_setConst

  // save qpos0, to recognize changed key_qpos in write
  std::vector<mjtNum> qpos0;
  std::vector<mjtNum> body_pos0;
  std::vector<mjtNum> body_quat0;

  // variable-size attributes
  std::string comment_;           // comment at top of XML
  std::string modelfiledir_;      // path to model file
  std::string modelname_;
  std::string meshdir_;
  std::string texturedir_;
  std::string spec_comment_;
  std::string spec_modelfiledir_;
  std::string spec_modelname_;
  std::string spec_meshdir_;
  std::string spec_texturedir_;
};

// mjCModel contains everything needed to generate the low-level model.
// It can be constructed manually by calling 'Add' functions and setting
// the public fields of the various objects.  Alternatively it can constructed
// by loading an XML file via mjCXML.  Once an mjCModel object is
// constructed, 'Compile' can be called to generate the corresponding mjModel object
// (which is the low-level model).  The mjCModel object can then be deleted.
class mjCModel : public mjCModel_, private mjSpec {
  friend class mjCBase;
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
  mjCModel();
  mjCModel(const mjCModel& other);
  ~mjCModel();
  void CopyFromSpec();  // copy spec to private attributes
  void PointToLocal();

  mjCModel& operator=(const mjCModel& other);     // copy other into this, if they are not the same
  mjCModel& operator+=(const mjCModel& other);    // add other into this, even if they are the same
  mjCModel& operator-=(const mjCBody& subtree);   // remove subtree and all references from model
  mjCModel_& operator+=(mjCDef& subtree);         // add default tree to this model
  mjCModel& operator-=(const mjCDef& subtree);    // remove default tree from this model

  mjSpec spec;

  mjModel* Compile(const mjVFS* vfs = nullptr, mjModel** m = nullptr);  // construct mjModel
  bool CopyBack(const mjModel*);                 // DECOMPILER: copy numeric back
  void FuseStatic();                             // fuse static bodies with parent
  void FuseReindex(mjCBody* body);               // reindex elements during fuse

  // API for adding model elements
  mjCFlex* AddFlex();
  mjCMesh* AddMesh(mjCDef* def = nullptr);
  mjCSkin* AddSkin();
  mjCHField* AddHField();
  mjCTexture* AddTexture();
  mjCMaterial* AddMaterial(mjCDef* def = nullptr);
  mjCPair* AddPair(mjCDef* def = nullptr);          // geom pair for inclusion
  mjCBodyPair* AddExclude();                        // body pair for exclusion
  mjCEquality* AddEquality(mjCDef* def = nullptr);  // equality constraint
  mjCTendon* AddTendon(mjCDef* def = nullptr);
  mjCActuator* AddActuator(mjCDef* def = nullptr);
  mjCSensor* AddSensor();
  mjCNumeric* AddNumeric();
  mjCText* AddText();
  mjCTuple* AddTuple();
  mjCKey* AddKey();
  mjCPlugin* AddPlugin();

  // append spec to this model, optionally map compiler options to the appended spec
  void AppendSpec(mjSpec* spec, const mjsCompiler* compiler = nullptr);

  // delete elements marked as discard=true
  template <class T> void Delete(std::vector<T*>& elements,
                                 const std::vector<bool>& discard);

  // delete all elements
  template <class T> void DeleteAll(std::vector<T*>& elements);

  // delete object from the corresponding list
  void DeleteElement(mjsElement* el);

  // delete default and all descendants
  void RemoveDefault(mjCDef* def);

  // detach subtree from model
  void Detach(mjCBody* subtree);

  // API for access to model elements (outside tree)
  int NumObjects(mjtObj type);              // number of objects in specified list
  mjCBase* GetObject(mjtObj type, int id);  // pointer to specified object
  mjsElement* NextObject(mjsElement* object, mjtObj type = mjOBJ_UNKNOWN);  // next object of specified type

  // API for access to other variables
  bool IsCompiled() const;                                          // is model already compiled
  const mjCError& GetError() const;                                 // get reference of error object
  void SetError(const mjCError& error) { errInfo = error; }         // set value of error object
  mjCBody* GetWorld();                                              // pointer to world body
  mjCDef* FindDefault(std::string name);                            // find defaults class name
  mjCDef* AddDefault(std::string name, mjCDef* parent = nullptr);   // add defaults class to array
  mjCBase* FindObject(mjtObj type, std::string name) const;         // find object given type and name
  mjCBase* FindTree(mjCBody* body, mjtObj type, std::string name);  // find tree object given name
  mjSpec* FindSpec(std::string name) const;                         // find spec given name
  mjSpec* FindSpec(const mjsCompiler* compiler_);                   // find spec given mjsCompiler
  void ActivatePlugin(const mjpPlugin* plugin, int slot);           // activate plugin

  // accessors
  std::string get_meshdir() const { return meshdir_; }
  std::string get_texturedir() const { return texturedir_; }

  mjCDef* Default() const { return defaults_[0]; }
  int NumDefaults() const { return defaults_.size(); }

  const std::vector<std::pair<const mjpPlugin*, int>>& ActivePlugins() const {
    return active_plugins_;
  };

  const std::vector<mjCFlex*>& Flexes() const { return flexes_; }
  const std::vector<mjCMesh*>& Meshes() const {return meshes_; }
  const std::vector<mjCSkin*>& Skins() const { return skins_; }
  const std::vector<mjCHField*>& HFields() const { return hfields_; }
  const std::vector<mjCTexture*>& Textures() const { return textures_; }
  const std::vector<mjCMaterial*>& Materials() const { return materials_; }
  const std::vector<mjCPair*>& Pairs() const { return pairs_; }
  const std::vector<mjCBodyPair*>& Excludes() const { return excludes_; }
  const std::vector<mjCEquality*>& Equalities() const { return equalities_; }
  const std::vector<mjCTendon*>& Tendons() const { return tendons_; }
  const std::vector<mjCActuator*>& Actuators() const { return actuators_; }
  const std::vector<mjCSensor*>& Sensors() const { return sensors_; }
  const std::vector<mjCNumeric*>& Numerics() const { return numerics_; }
  const std::vector<mjCText*>& Texts() const { return texts_; }
  const std::vector<mjCTuple*>& Tuples() const { return tuples_; }
  const std::vector<mjCKey*>& Keys() const { return keys_; }
  const std::vector<mjCPlugin*>& Plugins() const { return plugins_; }
  const std::vector<mjCBody*>& Bodies() const { return bodies_; }
  const std::vector<mjCGeom*>& Geoms() const { return geoms_; }

  // resolve plugin instance, create a new one if needed
  void ResolvePlugin(mjCBase* obj, const std::string& plugin_name,
                     const std::string& plugin_instance_name,
                     mjCPlugin** plugin_instance);

  // clear objects allocated by Compile
  void Clear();

  // multi-threaded mesh compilation
  void CompileMeshes(const mjVFS* vfs);

  // delete material from object
  template <class T> void DeleteMaterial(std::vector<T*>& list,
                                         std::string_view name = "");

  // save the current state
  template <class T>
  void SaveState(const std::string& state_name, const T* qpos, const T* qvel, const T* act,
                 const T* ctrl, const T* mpos, const T* mquat);

  // restore the previously saved state
  template <class T>
  void RestoreState(const std::string& state_name, const mjtNum* pos0, const mjtNum* mpos0,
                    const mjtNum* mquat0, T* qpos, T* qvel, T* act, T* ctrl, T* mpos, T* mquat);

  // clear existing data
  void MakeData(const mjModel* m, mjData** dest);

  // resolve keyframe references
  void StoreKeyframes(mjCModel* dest);

  // map from default class name to default class pointer
  std::unordered_map<std::string, mjCDef*> def_map;

  // set deepcopy flag
  void SetDeepCopy(bool deepcopy) { deepcopy_ = deepcopy; }

  // set attached flag
  void SetAttached(bool deepcopy) { attached_ |= !deepcopy; }

 private:
  // settings for each defaults class
  std::vector<mjCDef*> defaults_;

  // list of active plugins
  std::vector<std::pair<const mjpPlugin*, int>> active_plugins_;

  // make lists of bodies and children
  void MakeTreeLists(mjCBody* body = nullptr);

  // compile phases
  void TryCompile(mjModel*& m, mjData*& d, const mjVFS* vfs);
  void SetNuser();                      // set nuser fields
  void IndexAssets(bool discard);       // convert asset names into indices
  void CheckEmptyNames();               // check empty names
  void SetSizes();                      // compute sizes
  void AutoSpringDamper(mjModel*);      // automatic stiffness and damping computation
  void LengthRange(mjModel*, mjData*);  // compute actuator lengthrange
  void CopyNames(mjModel*);             // copy names, compute name addresses
  void CopyPaths(mjModel*);             // copy paths, compute path addresses
  void CopyObjects(mjModel*);           // copy objects outside kinematic tree
  void CopyTree(mjModel*);              // copy objects inside kinematic tree
  void FinalizeSimple(mjModel* m);      // finalize simple bodies/dofs including tendon information
  void CopyPlugins(mjModel*);           // copy plugin data
  int CountNJmom(const mjModel* m);     // compute number of non-zeros in actuator_moment matrix

  // remove plugins that are not referenced by any object
  void RemovePlugins();

  // objects created here
  std::vector<mjCFlex*>     flexes_;      // list of flexes
  std::vector<mjCMesh*>     meshes_;      // list of meshes
  std::vector<mjCSkin*>     skins_;       // list of skins
  std::vector<mjCHField*>   hfields_;     // list of height fields
  std::vector<mjCTexture*>  textures_;    // list of textures
  std::vector<mjCMaterial*> materials_;   // list of materials
  std::vector<mjCPair*>     pairs_;       // list of geom pairs to include
  std::vector<mjCBodyPair*> excludes_;    // list of body pairs to exclude
  std::vector<mjCEquality*> equalities_;  // list of equality constraints
  std::vector<mjCTendon*>   tendons_;     // list of tendons
  std::vector<mjCActuator*> actuators_;   // list of actuators
  std::vector<mjCSensor*>   sensors_;     // list of sensors
  std::vector<mjCNumeric*>  numerics_;    // list of numeric fields
  std::vector<mjCText*>     texts_;       // list of text fields
  std::vector<mjCTuple*>    tuples_;      // list of tuple fields
  std::vector<mjCKey*>      keys_;        // list of keyframe fields
  std::vector<mjCPlugin*>   plugins_;     // list of plugin instances
  std::vector<mjSpec*>      specs_;       // list of attached specs

  // pointers to objects created inside kinematic tree
  std::vector<mjCBody*>   bodies_;   // list of bodies
  std::vector<mjCJoint*>  joints_;   // list of joints allowing motion relative to parent
  std::vector<mjCGeom*>   geoms_;    // list of geoms attached to this body
  std::vector<mjCSite*>   sites_;    // list of sites attached to this body
  std::vector<mjCCamera*> cameras_;  // list of cameras
  std::vector<mjCLight*>  lights_;   // list of lights
  std::vector<mjCFrame*>  frames_;   // list of frames

  // array of pointers to each object list (enumerated by type)
  std::array<std::vector<mjCBase*>*, mjNOBJECT> object_lists_;

  // add object of any type
  template <class T> T* AddObject(std::vector<T*>& list, std::string type);

  // add object of any type, with defaults parameter
  template <class T> T* AddObjectDefault(std::vector<T*>& list, std::string type,
                                         mjCDef* def);

  // copy vector of elements to this model
  template <class T> void CopyList(std::vector<T*>& dest,
                                   const std::vector<T*>& sources);

  // copy plugins that are explicitly instantiated by the argument object to this model
  template <class T> void CopyExplicitPlugin(T* obj);

  // copy vector of plugins to this model
  template <class T> void CopyPlugin(const std::vector<mjCPlugin*>& sources,
                                     const std::vector<T*>& list);

  // delete from list the elements that cause an error
  template <class T> void RemoveFromList(std::vector<T*>& list, const mjCModel& other);

  // create mjCBase lists from children lists
  void CreateObjectLists();

  // populate objects ids
  void ProcessLists(bool checkrepeat = true);

  // reset lists of kinematic tree
  void ResetTreeLists();

  // save dof offsets in joints and actuators
  void SaveDofOffsets(bool computesize = false);

  // convert pending keyframes info to actual keyframes
  void ResolveKeyframes(const mjModel* m);

  // resize a keyframe, filling in missing values
  void ResizeKeyframe(mjCKey* key, const mjtNum* qpos0_, const mjtNum* bpos, const mjtNum* bquat);

  // compute qpos0
  void ComputeReference();

  // return true if all bodies have valid mass and inertia
  bool CheckBodiesMassInertia(std::vector<mjCBody*> bodies);

  // return true if body has valid mass and inertia
  bool CheckBodyMassInertia(mjCBody* body);

  // Mark plugin instances mentioned in the list
  template <class T>
  void MarkPluginInstance(std::unordered_map<std::string, bool>& instances,
                          const std::vector<T*>& list);

  // print the tree of a body
  std::string PrintTree(const mjCBody* body, std::string indent = "");

  // generate a signature for the model
  uint64_t Signature();

  // reassign children of a body to a new parent
  template <class T>
  void ReassignChild(std::vector<T*>& dest, std::vector<T*>& list, mjCBody* parent, mjCBody* body);

  // resolve references in a list of objects
  template <class T>
  void ResolveReferences(std::vector<T*>& list, mjCBody* body = nullptr);

  mjListKeyMap ids;   // map from object names to ids
  mjCError errInfo;   // last error info
  std::vector<mjKeyInfo> key_pending_;  // attached keyframes
  bool deepcopy_;     // copy objects when attaching
  bool attached_ = false;  // true if model is attached to a parent model
  std::unordered_map<const mjsCompiler*, mjSpec*> compiler2spec_;  // map from compiler to spec
};
#endif  // MUJOCO_SRC_USER_USER_MODEL_H_
