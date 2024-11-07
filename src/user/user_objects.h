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

#ifndef MUJOCO_SRC_USER_USER_OBJECTS_H_
#define MUJOCO_SRC_USER_USER_OBJECTS_H_

#include <cstddef>
#include <array>
#include <cstdlib>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mjtnum.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjspec.h>
#include "user/user_cache.h"
#include "user/user_util.h"

// forward declarations of all mjC/X classes
class mjCError;
class mjCBase;
class mjCBody;
class mjCFrame;
class mjCJoint;
class mjCGeom;
class mjCSite;
class mjCCamera;
class mjCLight;
class mjCHField;
class mjCFlex;         // defined in user_mesh.h
class mjCMesh;         // defined in user_mesh.h
class mjCSkin;         // defined in user_mesh.h
class mjCTexture;
class mjCMaterial;
class mjCPair;
class mjCBodyPair;
class mjCEquality;
class mjCTendon;
class mjCWrap;
class mjCActuator;
class mjCSensor;
class mjCNumeric;
class mjCText;
class mjCTuple;
class mjCDef;
class mjCModel;        // defined in user_model.h
class mjXWriter;       // defined in xml_native.h
class mjXURDF;         // defined in xml_urdf.h


//------------------------- helper constants, classes and functions --------------------------------

// number of positive size parameters for each geom type
const int mjGEOMINFO[mjNGEOMTYPES] = {3, 0, 1, 2, 3, 2, 3, 0};

// error information
class [[nodiscard]] mjCError {
 public:
  mjCError(const mjCBase* obj = 0,
           const char* msg = 0,
           const char* str = 0,
           int pos1 = 0,
           int pos2 = 0);

  char message[500];              // error message
  bool warning;                   // is this a warning instead of error
};

// alternative specifications of frame orientation
const char* ResolveOrientation(double* quat,             // set frame quat
                               bool degree,              // angle format: degree/radian
                               const char* sequence,     // euler sequence format: "xyz"
                               const mjsOrientation& orient);


//------------------------- class mjCBoundingVolumeHierarchy ---------------------------------------

// bounding volume
class mjCBoundingVolume {
 public:
  mjCBoundingVolume() { id_ = nullptr; };

  int contype;                  // contact type
  int conaffinity;              // contact affinity
  const double* aabb;           // axis-aligned bounding box (center, size)
  const double* pos;            // position (set by user or Compile1)
  const double* quat;           // orientation (set by user or Compile1)

  const int* GetId() const { if (id_) return id_; else return &idval_; }
  void SetId(const int* id) { id_ = id; }
  void SetId(int val) { idval_ = val; }

 private:
  int idval_;      // local id copy for nodes not storing their id's (e.g. faces)
  const int* id_;  // pointer to object id
};


// bounding volume hierarchy
struct mjCBoundingVolumeHierarchy_ {
 protected:
  int nbvh_ = 0;
  std::vector<mjtNum> bvh_;           // bounding boxes                        (nbvh x 6)
  std::vector<int> child_;            // children of each node                 (nbvh x 2)
  std::vector<int*> nodeid_;          // id of elem contained by the node      (nbvh x 1)
  std::vector<int> level_;            // levels of each node                   (nbvh x 1)
  std::vector<mjCBoundingVolume> bvleaf_;
  std::string name_;
  double ipos_[3];
  double iquat_[4];
};

class mjCBoundingVolumeHierarchy : public mjCBoundingVolumeHierarchy_ {
 public:
  mjCBoundingVolumeHierarchy();

  // make bounding volume hierarchy
  void CreateBVH();
  void Set(double ipos_element[3], double iquat_element[4]);
  void AllocateBoundingVolumes(int nleaf);
  void RemoveInactiveVolumes(int nmax);
  mjCBoundingVolume* GetBoundingVolume(int id);

  // public accessors
  int Nbvh() const { return nbvh_; }
  const std::vector<mjtNum>& Bvh() const { return bvh_; }
  const std::vector<int>& Child() const { return child_; }
  const std::vector<int*>& Nodeid() const { return nodeid_; }
  const int* Nodeid(int id) const { return nodeid_[id]; }
  const std::vector<int>& Level() const { return level_; }

 private:
  // internal class used during BVH construction, for partial sorting of bounding volumes
  struct BVElement {
    const mjCBoundingVolume* e;
    // position of the element in the BVH axes
    double lpos[3];
  };

  struct BVElementCompare {
    int axis = 0;

    bool operator()(const BVElement& e1, const BVElement& e2) const {
      if (std::abs(e1.lpos[axis] - e2.lpos[axis]) > mjEPS) {
        return e1.lpos[axis] < e2.lpos[axis];
      }
      // comparing pointers gives a stable sort, because they both come from the same array
      return e1.e < e2.e;
    }
  };

  int MakeBVH(std::vector<BVElement>::iterator elements_begin,
              std::vector<BVElement>::iterator elements_end, int lev = 0);
};



//------------------------- class mjCBase ----------------------------------------------------------
// Generic functionality for all derived classes

class mjCBase_ : public mjsElement {
 public:
  int id;                 // object id
  std::string name;       // object name
  std::string classname;  // defaults class name
  std::string info;       // error message info set by the user
  std::string prefix;     // prefix for model operations
  std::string suffix;     // suffix for model operations
};

class mjCBase : public mjCBase_ {
  friend class mjCDef;

 public:
  // load resource if found (fallback to OS filesystem)
  static mjResource* LoadResource(const std::string& modelfiledir,
                                  const std::string& filename, const mjVFS* vfs);

  // Get and sanitize content type from raw_text if not empty, otherwise parse
  // content type from resource_name; throw on failure
  std::string GetAssetContentType(std::string_view resource_name, std::string_view raw_text);

  // Add frame transformation
  void SetFrame(mjCFrame* _frame);

  // Copy spec into private attributes
  virtual void CopyFromSpec() {}

  // Throws an error if any of the references is missing
  virtual void ResolveReferences(const mjCModel* m) {}

  // Appends prefix and suffix to reference
  virtual void NameSpace(const mjCModel* m);

  // Copy assignment
  mjCBase& operator=(const mjCBase& other);

  mjCFrame* frame;                // pointer to frame transformation
  mjCModel* model;                // pointer to model that owns object
  mjsCompiler* compiler;          // pointer to the compiler options

  virtual ~mjCBase() = default;   // destructor

  // reset keyframe references for allowing self-attach
  virtual void ForgetKeyframes() {}
  virtual void ForgetKeyframes() const {}

 protected:
  mjCBase();                                 // constructor
  mjCBase(const mjCBase& other);             // copy constructor
};



//------------------------- class mjCBody -----------------------------------------------
// Describes a rigid body

class mjCBody_ : public mjCBase {
 protected:
  // variables computed by 'Compile' and 'AddXXX'
  int parentid;                   // parent index in global array
  int weldid;                     // top index of body we are welded to
  int dofnum;                     // number of motion dofs for body
  int mocapid;                    // mocap id, -1: not mocap

  int contype;                    // OR over geom contypes
  int conaffinity;                // OR over geom conaffinities
  double margin;                  // MAX over geom margins
  double xpos0[3];                // global position in qpos0
  double xquat0[4];               // global orientation in qpos0

  // used internally by compiler
  int lastdof;                    // id of last dof
  int subtreedofs;                // number of dofs in subtree, including self

  mjCBoundingVolumeHierarchy tree;  // bounding volume hierarchy

  // variable-size data
  std::string plugin_name;
  std::string plugin_instance_name;
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;

  // variables used for temporarily storing the state of the mocap bodies
  std::map<std::string, std::array<mjtNum, 3>> mpos_;   // saved mocap_pos
  std::map<std::string, std::array<mjtNum, 4>> mquat_;  // saved mocap_quat
};

class mjCBody : public mjCBody_, private mjsBody {
  friend class mjCJoint;
  friend class mjCGeom;
  friend class mjCSite;
  friend class mjCCamera;
  friend class mjCComposite;
  friend class mjCFrame;
  friend class mjCLight;
  friend class mjCFlex;
  friend class mjCFlexcomp;
  friend class mjCEquality;
  friend class mjCPair;
  friend class mjCModel;
  friend class mjXReader;
  friend class mjXWriter;
  friend class mjXURDF;

 public:
  mjCBody(mjCModel*);  // constructor
  ~mjCBody();          // destructor

  // API for adding objects to body
  mjCBody*    AddBody(mjCDef* = 0);
  mjCFrame*   AddFrame(mjCFrame* = 0);
  mjCJoint*   AddJoint(mjCDef* = 0);
  mjCJoint*   AddFreeJoint();
  mjCGeom*    AddGeom(mjCDef* = 0);
  mjCSite*    AddSite(mjCDef* = 0);
  mjCCamera*  AddCamera(mjCDef* = 0);
  mjCLight*   AddLight(mjCDef* = 0);

  // API for adding/removing objects to body
  mjCBody& operator+=(const mjCBody& other);
  mjCBody& operator+=(const mjCFrame& other);
  mjCBody& operator-=(const mjCBody& subtree);

  // API for accessing objects
  int NumObjects(mjtObj type);
  mjCBase* GetObject(mjtObj type, int id);
  mjCBase* FindObject(mjtObj type, std::string name, bool recursive = true);

  // Propagate suffix and prefix to the whole tree
  void NameSpace(const mjCModel* m);

  // set explicitinertial to true
  void MakeInertialExplicit();

  // compute the bounding volume hierarchy of the body.
  void ComputeBVH();

  // variables set by user
  mjsBody spec;

  // inherited
  using mjCBase::name;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }

  // get next child of given type recursively; if `child` is found while traversing the tree,
  // then `found` is set to true and the next element encountered is returned;
  // returns nullptr if the next child is not found or if `child` is the last element, returns
  // the next child after the input `child` otherwise
  mjsElement* NextChild(const mjsElement* child, mjtObj type = mjOBJ_UNKNOWN,
                        bool recursive = false, bool* found = nullptr);

  // reset keyframe references for allowing self-attach
  void ForgetKeyframes() const;

  // create a frame and move all contents of this body into it
  mjCFrame* ToFrame();

  // get mocap position and quaternion
  mjtNum* mpos(const std::string& state_name);
  mjtNum* mquat(const std::string& state_name);

  mjsFrame* last_attached;  // last attached frame to this body

 private:
  mjCBody(const mjCBody& other, mjCModel* _model);  // copy constructor
  mjCBody& operator=(const mjCBody& other);         // copy assignment

  void Compile(void);             // compiler
  void InertiaFromGeom(void);     // get inertial info from geoms

  // objects allocated by Add functions
  std::vector<mjCBody*>    bodies;     // child bodies
  std::vector<mjCGeom*>    geoms;      // geoms attached to this body
  std::vector<mjCFrame*>   frames;     // frames attached to this body
  std::vector<mjCJoint*>   joints;     // joints allowing motion relative to parent
  std::vector<mjCSite*>    sites;      // sites attached to this body
  std::vector<mjCCamera*>  cameras;    // cameras attached to this body
  std::vector<mjCLight*>   lights;     // lights attached to this body

  void CopyFromSpec();                 // copy spec into attributes
  void PointToLocal(void);
  void NameSpace_(const mjCModel* m, bool propagate = true);

  // copy src list of elements into dst; set body, model and frame
  template <typename T>
  void CopyList(std::vector<T*>& dst, const std::vector<T*>& src,
                std::map<mjCFrame*, int>& fmap, const mjCFrame* pframe = nullptr);

  // gets next child of the same type in this body
  template <class T>
  mjsElement* GetNext(const std::vector<T*>& list, const mjsElement* child, bool* found);
};



//------------------------- class mjCFrame ---------------------------------------------------------
// Describes a coordinate transformation relative to its parent

class mjCFrame_ : public mjCBase {
 protected:
  bool compiled;                           // frame already compiled
};

class mjCFrame : public mjCFrame_, private mjsFrame {
  friend class mjCBase;
  friend class mjCBody;
  friend class mjCGeom;
  friend class mjCJoint;
  friend class mjCSite;
  friend class mjCCamera;
  friend class mjCLight;
  friend class mjCModel;

 public:
  mjCFrame(mjCModel* = 0, mjCFrame* = 0);
  mjCFrame(const mjCFrame& other);
  mjCFrame& operator=(const mjCFrame& other);

  mjsFrame spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void SetParent(mjCBody* _body);

  mjCFrame& operator+=(const mjCBody& other);

  bool IsAncestor(const mjCFrame* child) const;  // true if child is contained in this frame

  mjsBody* last_attached;  // last attached body to this frame

 private:
  void Compile(void);                          // compiler

  mjCBody* body;  // body that owns the frame
};



//------------------------- class mjCJoint ---------------------------------------------------------
// Describes a motion degree of freedom of a body relative to its parent

class mjCJoint_ : public mjCBase {
 protected:
  mjCBody* body;                   // joint's body

  // variable used for temporarily storing the state of the joint
  std::map<std::string, std::array<mjtNum, 7>> qpos_;  // qpos at the previous step
  std::map<std::string, std::array<mjtNum, 6>> qvel_;  // qvel at the previous step

  // variable-size data
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};

class mjCJoint : public mjCJoint_, private mjsJoint {
  friend class mjCDef;
  friend class mjCEquality;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjCSensor;
  friend class mjXWriter;
  friend class mjXURDF;

 public:
  mjCJoint(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCJoint(const mjCJoint& other);
  mjCJoint& operator=(const mjCJoint& other);

  mjsJoint spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec(void);

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() const { return userdata_; }
  const double* get_range() const { return range; }

  bool is_limited() const;
  bool is_actfrclimited() const;

  static int nq(mjtJoint joint_type);
  static int nv(mjtJoint joint_type);
  int nq() const { return nq(spec.type); }
  int nv() const { return nv(spec.type); }

  mjtNum* qpos(const std::string& state_name);
  mjtNum* qvel(const std::string& state_name);

 private:
  int Compile(void);               // compiler; return dofnum
  void PointToLocal(void);

  // variables that should not be copied during copy assignment
  int qposadr_;                                        // address of dof in data->qpos
  int dofadr_;                                         // address of dof in data->qvel
};



//------------------------- class mjCGeom ----------------------------------------------------------
// Describes a geometric shape belonging to a body

class mjCGeom_ : public mjCBase {
 public:
  bool inferinertia;           // true if inertia should be computed from geom

 protected:
  bool visual_;                       // true: geom does not collide and is unreferenced
  int matid;                          // id of geom's material
  mjCMesh* mesh;                      // geom's mesh
  mjCHField* hfield;                  // geom's hfield
  double mass_;                       // mass
  double inertia[3];                  // local diagonal inertia
  double aabb[6];                     // axis-aligned bounding box (center, size)
  mjCBody* body;                      // geom's body
  mjtNum fluid[mjNFLUID];             // compile-time fluid-interaction parameters

  // variable-size data
  std::string plugin_name;
  std::string plugin_instance_name;
  std::string hfieldname_;
  std::string meshname_;
  std::string material_;
  std::vector<double> userdata_;
  std::string spec_hfieldname_;
  std::string spec_meshname_;
  std::string spec_material_;
  std::vector<double> spec_userdata_;
};

class mjCGeom : public mjCGeom_, private mjsGeom {
  friend class mjCDef;
  friend class mjCMesh;
  friend class mjCPair;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjCWrap;
  friend class mjXWriter;
  friend class mjXURDF;

 public:
  mjCGeom(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCGeom(const mjCGeom& other);
  mjCGeom& operator=(const mjCGeom& other);
  ~mjCGeom();

  using mjCBase::name;
  mjsGeom spec;                       // variables set by user
  double GetVolume() const;           // compute geom volume
  void SetInertia(void);              // compute and set geom inertia
  bool IsVisual(void) const { return visual_; }
  void SetNotVisual(void) { visual_ = false; }
  mjtGeom Type() const { return type; }

  // Compute all coefs modeling the interaction with the surrounding fluid.
  void SetFluidCoefs(void);
  // Compute the kappa coefs of the added inertia due to the surrounding fluid.
  double GetAddedMassKappa(double dx, double dy, double dz);

  // sets properties of a bounding volume
  void SetBoundingVolume(mjCBoundingVolume* bv) const;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() const { return userdata_; }
  const std::string& get_hfieldname() const { return spec_hfieldname_; }
  const std::string& get_meshname() const { return spec_meshname_; }
  const std::string& get_material() const { return spec_material_; }
  void del_material() { spec_material_.clear(); }

 private:
  void Compile(void);                 // compiler
  double GetRBound(void);             // compute bounding sphere radius
  void ComputeAABB(void);             // compute axis-aligned bounding box
  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const mjCModel* m);

  // inherited
  using mjCBase::info;
};



//------------------------- class mjCSite ----------------------------------------------------------
// Describes a site on a body

class mjCSite_ : public mjCBase {
 protected:
  // variable-size data
  std::string material_;
  std::vector<double> userdata_;
  std::string spec_material_;
  std::vector<double> spec_userdata_;

  // variables computed by 'compile' and 'mjCBody::addSite'
  mjCBody* body;                  // site's body
  int matid;                      // material id for rendering
};

class mjCSite : public mjCSite_, private mjsSite {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjXWriter;
  friend class mjXURDF;

 public:
  mjCSite(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCSite(const mjCSite& other);
  mjCSite& operator=(const mjCSite& other);

  mjsSite spec;                   // variables set by user

  // site's body
  mjCBody* Body() const { return body; }

  // use strings from mjCBase rather than mjStrings from mjsSite
  using mjCBase::name;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() const { return userdata_; }
  const std::string& get_material() const { return material_; }
  void del_material() { material_.clear(); }

 private:
  void Compile(void);                     // compiler
  void CopyFromSpec();                    // copy spec into attributes
  void PointToLocal(void);
};



//------------------------- class mjCCamera --------------------------------------------------------
// Describes a camera, attached to a body

class mjCCamera_ : public mjCBase {
 protected:
  mjCBody* body;                  // camera's body
  int targetbodyid;               // id of target body; -1: none
  std::string targetbody_;
  std::string spec_targetbody_;
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};

class mjCCamera : public mjCCamera_, private mjsCamera {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjCSensor;
  friend class mjXWriter;

 public:
  mjCCamera(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCCamera(const mjCCamera& other);
  mjCCamera& operator=(const mjCCamera& other);

  mjsCamera spec;
  using mjCBase::name;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::string& get_targetbody() const { return targetbody_; }
  const std::vector<double>& get_userdata() const { return userdata_; }

 private:
  void Compile(void);                     // compiler
  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const mjCModel* m);
};



//------------------------- class mjCLight ---------------------------------------------------------
// Describes a light, attached to a body

class mjCLight_ : public mjCBase {
 protected:
  mjCBody* body;                  // light's body
  int targetbodyid;               // id of target body; -1: none
  std::string targetbody_;
  std::string spec_targetbody_;
};

class mjCLight : public mjCLight_, private mjsLight {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCLight(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCLight(const mjCLight& other);
  mjCLight& operator=(const mjCLight& other);

  mjsLight spec;
  using mjCBase::name;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::string& get_targetbody() const { return targetbody_; }

 private:
  void Compile(void);                     // compiler
  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const mjCModel* m);
};



//------------------------- class mjCFlex ----------------------------------------------------------
// Describes a flex

class mjCFlex_ : public mjCBase {
 protected:
  int nvert;                              // number of verices
  int nedge;                              // number of edges
  int nelem;                              // number of elements
  int matid;                              // material id
  bool rigid;                             // all vertices attached to the same body
  bool centered;                          // all vertices coordinates (0,0,0)
  std::vector<int> vertbodyid;            // vertex body ids
  std::vector<std::pair<int, int>> edge;  // edge vertex ids
  std::vector<int> shell;                 // shell fragment vertex ids (dim per fragment)
  std::vector<int> elemlayer;             // element layer (distance from border)
  std::vector<int> evpair;                // element-vertex pairs
  std::vector<double> vertxpos;           // global vertex positions
  mjCBoundingVolumeHierarchy tree;        // bounding volume hierarchy
  std::vector<double> elemaabb_;          // element bounding volume
  std::vector<int> edgeidx_;              // element edge ids
  std::vector<double> stiffness;          // elasticity stiffness matrix

  // variable-size data
  std::vector<std::string> vertbody_;     // vertex body names
  std::vector<double> vert_;              // vertex positions
  std::vector<int> elem_;                 // element vertex ids
  std::vector<float> texcoord_;           // vertex texture coordinates
  std::string material_;                  // name of material used for rendering

  std::string spec_material_;
  std::vector<std::string> spec_vertbody_;
  std::vector<double> spec_vert_;
  std::vector<int> spec_elem_;
  std::vector<float> spec_texcoord_;
};

class mjCFlex: public mjCFlex_, private mjsFlex {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjCFlexcomp;
  friend class mjCEquality;
  friend class mjXWriter;

 public:
  mjCFlex(mjCModel* = nullptr);
  mjCFlex(const mjCFlex& other);
  mjCFlex& operator=(const mjCFlex& other);

  mjsFlex spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

  // used by mjXWriter and mjCModel
  const std::string& get_material() const { return material_; }
  const std::vector<std::string>& get_vertbody() const { return vertbody_; }
  const std::vector<double>& get_vert() const { return vert_; }
  const std::vector<double>& get_elemaabb() const { return elemaabb_; }
  const std::vector<int>& get_elem() const { return elem_; }
  const std::vector<float>& get_texcoord() const { return texcoord_; }

  bool HasTexcoord() const;               // texcoord not null
  void DelTexcoord();                     // delete texcoord

  static constexpr int kNumEdges[3] = {1, 3, 6};  // number of edges per element indexed by dim

 private:
  void Compile(const mjVFS* vfs);         // compiler
  void CreateBVH(void);                   // create flex BVH
  void CreateShellPair(void);             // create shells and evpairs
};



//------------------------- class mjCMesh ----------------------------------------------------------
// Describes a mesh

class mjCMesh_ : public mjCBase {
 protected:
  // variable size attributes
  std::string plugin_name;
  std::string plugin_instance_name;

  std::string content_type_;                     // content type of file
  std::string file_;                             // mesh file
  std::vector<float> vert_;                      // vertex data
  std::vector<float> normal_;                    // normal data
  std::vector<float> texcoord_;                  // texcoord data
  std::vector<int> face_;                        // vertex indices
  std::vector<int> facenormal_;                  // normal indices
  std::vector<int> facetexcoord_;                // texcoord indices

  std::string spec_content_type_;
  std::string spec_file_;
  std::vector<float> spec_vert_;
  std::vector<float> spec_normal_;
  std::vector<float> spec_texcoord_;
  std::vector<int> spec_face_;
  std::vector<int> spec_facenormal_;
  std::vector<int> spec_facetexcoord_;

  // used by the compiler
  bool visual_;                                  // true: the mesh is only visual
  std::vector< std::pair<int, int> > halfedge_;  // half-edge data

  // mesh properties that indicate a well-formed mesh
  std::pair<int, int> invalidorientation_;    // indices of invalid edge; -1 if none
  bool validarea_;                            // false if the area is too small
  int validvolume_;                           // 0: volume is too small, -1: volume is negative
  bool valideigenvalue_;                      // false if inertia eigenvalue is too small
  bool validinequality_;                      // false if inertia inequality is not satisfied
  bool processed_;                            // false if the mesh has not been processed yet

  // mesh properties computed by Compile
  double pos_volume_[3];              // CoM position (volume)
  double pos_surface_[3];             // CoM position (surface)
  double quat_volume_[4];             // inertia orientation (volume)
  double quat_surface_[4];            // inertia orientation (surface)
  double pos_[3];                     // translation applied to asset vertices
  double quat_[4];                    // rotation applied to asset vertices
  double boxsz_volume_[3];            // half-sizes of equivalent inertia box (volume)
  double boxsz_surface_[3];           // half-sizes of equivalent inertia box (surface)
  double aamm_[6];                    // axis-aligned bounding box in (min, max) format
  double volume_;                     // volume of the mesh
  double surface_;                    // surface of the mesh

  // size of mesh data to be copied into mjModel
  int szgraph_;                       // size of graph data in ints
  bool needhull_;                     // needs convex hull for collisions
  int maxhullvert_;                   // max vertex count of convex hull

  mjCBoundingVolumeHierarchy tree_;   // bounding volume hierarchy
  std::vector<double> face_aabb_;     // bounding boxes of all faces

  // paths stored during model attachment
  mujoco::user::FilePath modelfiledir_;
  mujoco::user::FilePath meshdir_;
};

class mjCMesh: public mjCMesh_, private mjsMesh {
  friend class mjCModel;

 public:
  mjCMesh(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCMesh(const mjCMesh& other);
  mjCMesh& operator=(const mjCMesh& other);
  ~mjCMesh();

  mjsMesh spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const mjCModel* m);

  // accessors
  const mjsPlugin& Plugin() const { return plugin; }
  const std::string& ContentType() const { return content_type_; }
  const std::string& File() const { return file_; }
  const double* Refpos() const { return refpos; }
  const double* Refquat() const { return refquat; }
  const double* Scale() const { return scale; }
  bool SmoothNormal() const { return smoothnormal; }
  const std::vector<float>& Vert() const { return vert_; }
  float Vert(int i) const { return vert_[i]; }
  const std::vector<float>& UserVert() const { return spec_vert_; }
  const std::vector<float>& UserNormal() const { return spec_normal_; }
  const std::vector<float>& UserTexcoord() const { return spec_texcoord_; }
  const std::vector<int>& Face() const { return face_; }
  const std::vector<int>& UserFace() const { return spec_face_; }
  mjtMeshInertia Inertia() const { return spec.inertia; }

  // setters
  void SetNeedHull(bool needhull) { needhull_ = needhull; }

  // mesh properties computed by Compile
  const double* aamm() const { return aamm_; }

  // number of vertices, normals, texture coordinates, and faces
  int nvert() const { return vert_.size()/3; }
  int nnormal() const { return normal_.size()/3; }
  int ntexcoord() const { return texcoord_.size()/2; }
  int nface() const { return face_.size()/3; }

  // return size of graph data in ints
  int szgraph() const { return szgraph_; }

  // bounding volume hierarchy tree
  const mjCBoundingVolumeHierarchy& tree() { return tree_; }

  void Compile(const mjVFS* vfs);                   // compiler
  double* GetPosPtr(mjtGeomInertia type);           // get position
  double* GetQuatPtr(mjtGeomInertia type);          // get orientation
  double* GetOffsetPosPtr();                        // get position offset for geom
  double* GetOffsetQuatPtr();                       // get orientation offset for geom
  double* GetInertiaBoxPtr(mjtGeomInertia type);    // get inertia box
  double& GetVolumeRef(mjtGeomInertia type);        // get volume
  void FitGeom(mjCGeom* geom, double* meshpos);     // approximate mesh with simple geom
  bool HasTexcoord() const;                         // texcoord not null
  void DelTexcoord();                               // delete texcoord
  bool IsVisual(void) const { return visual_; }     // is geom visual
  void SetNotVisual(void) { visual_ = false; }      // mark mesh as not visual

  void CopyVert(float* arr) const;                  // copy vert data into array
  void CopyNormal(float* arr) const;                // copy normal data into array
  void CopyFace(int* arr) const;                    // copy face data into array
  void CopyFaceNormal(int* arr) const;              // copy face normal data into array
  void CopyFaceTexcoord(int* arr) const;            // copy face texcoord data into array
  void CopyTexcoord(float* arr) const;              // copy texcoord data into array
  void CopyGraph(int* arr) const;                   // copy graph data into array

  // sets properties of a bounding volume given a face id
  void SetBoundingVolume(int faceid);

  void LoadOBJ(mjResource* resource);  // load mesh in wavefront OBJ format
  void LoadSTL(mjResource* resource);  // load mesh in STL BIN format
  void LoadMSH(mjResource* resource);  // load mesh in MSH BIN format

  void RemoveRepeated();               // remove repeated vertices

 private:
  // load mesh from cache asset, return true on success (OBJ files are only supported)
  bool LoadCachedMesh(mjCCache *cache, const mjResource* resource);
  // store mesh into asset cache (OBJ files are only supported)
  void CacheMesh(mjCCache *cache, const mjResource* resource,
                 std::string_view asset_type);

  void LoadSDF();                             // generate mesh using marching cubes
  void MakeGraph();                           // make graph of convex hull
  void CopyGraph();                           // copy graph into face data
  void MakeNormal();                          // compute vertex normals
  void MakeCenter();                          // compute face circumcircle data
  void Process();                             // compute inertial properties
  void ApplyTransformations();                // apply user transformations
  void ComputeFaceCentroid(double[3]);        // compute centroid of all faces
  void CheckMesh(mjtGeomInertia type);        // check if the mesh is valid

  // mesh data to be copied into mjModel
  double* center_;                    // face circumcenter data (3*nface)
  int* graph_;                        // convex graph data

  // for caching purposes
  std::vector<int> vertex_index_;
  std::vector<int> normal_index_;
  std::vector<int> texcoord_index_;
  std::vector<unsigned char> num_face_vertices_;

  // compute the volume and center-of-mass of the mesh given the face center
  void ComputeVolume(double CoM[3], mjtGeomInertia gtype, const double facecen[3]);
};



//------------------------- class mjCSkin ----------------------------------------------------------
// Describes a skin

class mjCSkin_ : public mjCBase {
 protected:
  // variable size attributes
  std::string file_;
  std::string material_;
  std::vector<float> vert_;
  std::vector<float> texcoord_;
  std::vector<int> face_;
  std::vector<std::string> bodyname_;
  std::vector<float> bindpos_;
  std::vector<float> bindquat_;
  std::vector<std::vector<int>> vertid_;
  std::vector<std::vector<float>> vertweight_;

  std::string spec_file_;
  std::string spec_material_;
  std::vector<float> spec_vert_;
  std::vector<float> spec_texcoord_;
  std::vector<int> spec_face_;
  std::vector<std::string> spec_bodyname_;
  std::vector<float> spec_bindpos_;
  std::vector<float> spec_bindquat_;
  std::vector<std::vector<int>> spec_vertid_;
  std::vector<std::vector<float>> spec_vertweight_;

  int matid;                          // material id
  std::vector<int> bodyid;            // body ids

  // paths stored during model attachment
  mujoco::user::FilePath modelfiledir_;
  mujoco::user::FilePath meshdir_;
};

class mjCSkin: public mjCSkin_, private mjsSkin {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCSkin(mjCModel* = nullptr);
  mjCSkin(const mjCSkin& other);
  mjCSkin& operator=(const mjCSkin& other);
  ~mjCSkin();

  mjsSkin spec;
  using mjCBase::name;
  using mjCBase::info;

  const std::string& File() const { return file_; }
  const std::string& get_material() const { return material_; }
  const std::vector<float>& get_vert() const { return vert_; }
  const std::vector<float>& get_texcoord() const { return texcoord_; }
  const std::vector<int>& get_face() const { return face_; }
  const std::vector<std::string>& get_bodyname() const { return bodyname_; }
  const std::vector<float>& get_bindpos() const { return bindpos_; }
  const std::vector<float>& get_bindquat() const { return bindquat_; }
  const std::vector<std::vector<int>>& get_vertid() const { return vertid_; }
  const std::vector<std::vector<float>>& get_vertweight() const { return vertweight_; }
  void del_material() { material_.clear(); }

  void CopyFromSpec();
  void PointToLocal();

 private:
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);
  void Compile(const mjVFS* vfs);             // compiler
  void LoadSKN(mjResource* resource);         // load skin in SKN BIN format
};



//------------------------- class mjCHField --------------------------------------------------------
// Describes a height field

class mjCHField_ : public mjCBase {
 protected:
  std::vector<float> data;  // elevation data, row-major format

  std::string file_;
  std::string content_type_;
  std::vector<float> userdata_;
  std::string spec_file_;
  std::string spec_content_type_;
  std::vector<float> spec_userdata_;

  // paths stored during model attachment
  mujoco::user::FilePath modelfiledir_;
  mujoco::user::FilePath meshdir_;
};

class mjCHField : public mjCHField_, private mjsHField {
  friend class mjCGeom;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCHField(mjCModel* model);
  mjCHField(const mjCHField& other);
  mjCHField& operator=(const mjCHField& other);
  ~mjCHField();

  mjsHField spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const mjCModel* m);

  std::string File() const { return file_; }

  // getter for user data
  std::vector<float>& get_userdata() { return userdata_; }

 private:
  void Compile(const mjVFS* vfs);         // compiler

  void LoadCustom(mjResource* resource);  // load from custom format
  void LoadPNG(mjResource* resource);     // load from PNG format
};



//------------------------- class mjCTexture -------------------------------------------------------
// Describes a texture

class mjCTexture_ : public mjCBase {
 protected:
  std::vector<std::byte> data_;  // texture data (rgb, roughness, etc.)

  std::string file_;
  std::string content_type_;
  std::vector<std::string> cubefiles_;
  std::string spec_file_;
  std::string spec_content_type_;
  std::vector<std::string> spec_cubefiles_;

  // paths stored during model attachment
  mujoco::user::FilePath modelfiledir_;
  mujoco::user::FilePath texturedir_;
};

class mjCTexture : public mjCTexture_, private mjsTexture {
  friend class mjCModel;
  friend class mjXReader;
  friend class mjXWriter;

 public:
  mjCTexture(mjCModel*);
  mjCTexture(const mjCTexture& other);
  mjCTexture& operator=(const mjCTexture& other);
  ~mjCTexture();

  mjsTexture spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const mjCModel* m);

  std::string File() const { return file_; }
  std::string get_content_type() const { return content_type_; }
  std::vector<std::string> get_cubefiles() const { return cubefiles_; }

 private:
  void Compile(const mjVFS* vfs);         // compiler

  void Builtin2D(void);                   // make builtin 2D
  void BuiltinCube(void);                 // make builtin cube
  void Load2D(std::string filename, const mjVFS* vfs);          // load 2D from file
  void LoadCubeSingle(std::string filename, const mjVFS* vfs);  // load cube from single file
  void LoadCubeSeparate(const mjVFS* vfs);                      // load cube from separate files

  void LoadFlip(std::string filename, const mjVFS* vfs,         // load and flip
                std::vector<unsigned char>& image,
                unsigned int& w, unsigned int& h);

  void LoadPNG(mjResource* resource,
               std::vector<unsigned char>& image,
               unsigned int& w, unsigned int& h);
  void LoadCustom(mjResource* resource,
                  std::vector<unsigned char>& image,
                  unsigned int& w, unsigned int& h);

  bool clear_data_;  // if true, data_ is empty and should be filled by Compile
};



//------------------------- class mjCMaterial ------------------------------------------------------
// Describes a material for rendering

class mjCMaterial_ : public mjCBase {
 protected:
  int texid[mjNTEXROLE];                    // id of material's textures
  std::vector<std::string> textures_;
  std::vector<std::string> spec_textures_;
};

class mjCMaterial : public mjCMaterial_, private mjsMaterial {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCMaterial(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCMaterial(const mjCMaterial& other);
  mjCMaterial& operator=(const mjCMaterial& other);

  mjsMaterial spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec();
  void PointToLocal();
  void NameSpace(const mjCModel* m);

  const std::string& get_texture(int i) const { return textures_[i]; }
  void del_textures() { for (auto& t : textures_) t.clear(); }

 private:
  void Compile(void);                       // compiler
};



//------------------------- class mjCPair ----------------------------------------------------------
// Predefined geom pair for collision detection

class mjCPair_ : public mjCBase {
 protected:
  int signature;                  // body1<<16 + body2
  std::string geomname1_;
  std::string geomname2_;
  std::string spec_geomname1_;
  std::string spec_geomname2_;
};

class mjCPair : public mjCPair_, private mjsPair {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCPair(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCPair(const mjCPair& other);
  mjCPair& operator=(const mjCPair& other);

  mjsPair spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

  const std::string& get_geomname1() const { return geomname1_; }
  const std::string& get_geomname2() const { return geomname2_; }

  int GetSignature(void) {
    return signature;
  }

 private:
  void Compile(void);                   // compiler

  mjCGeom* geom1;                 // geom1
  mjCGeom* geom2;                 // geom2
};



//------------------------- class mjCBodyPair ------------------------------------------------------
// Body pair specification, use to exclude pairs

class mjCBodyPair_ : public mjCBase {
 protected:
  int body1;                       // id of body1
  int body2;                       // id of body2
  int signature;                   // body1<<16 + body2

  std::string bodyname1_;          // name of geom 1
  std::string bodyname2_;          // name of geom 2
  std::string spec_bodyname1_;
  std::string spec_bodyname2_;
};

class mjCBodyPair : public mjCBodyPair_, private mjsExclude {
  friend class mjCBody;
  friend class mjCModel;

 public:
  mjCBodyPair(mjCModel*);
  mjCBodyPair(const mjCBodyPair& other);
  mjCBodyPair& operator=(const mjCBodyPair& other);

  mjsExclude spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

  std::string get_bodyname1() const { return bodyname1_; }
  std::string get_bodyname2() const { return bodyname2_; }

  int GetSignature() {
    return signature;
  }

 private:
  void Compile();              // compiler
};



//------------------------- class mjCEquality ------------------------------------------------------
// Describes an equality constraint

class mjCEquality_ : public mjCBase {
 protected:
  int obj1id;
  int obj2id;
  std::string name1_;
  std::string name2_;
  std::string spec_name1_;
  std::string spec_name2_;
};

class mjCEquality : public mjCEquality_, private mjsEquality {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCEquality(mjCModel* = 0, mjCDef* = 0);
  mjCEquality(const mjCEquality& other);
  mjCEquality& operator=(const mjCEquality& other);

  mjsEquality spec;
  using mjCBase::name;
  using mjCBase::info;

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

 private:
  void Compile(void);                       // compiler
};



//------------------------- class mjCTendon --------------------------------------------------------
// Describes a tendon

class mjCTendon_ : public mjCBase {
 protected:
  int matid;  // material id for rendering

  // variable-size data
  std::string material_;
  std::string spec_material_;
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};

class mjCTendon : public mjCTendon_, private mjsTendon {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCTendon(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCTendon(const mjCTendon& other);
  mjCTendon& operator=(const mjCTendon& other);
  ~mjCTendon();

  mjsTendon spec;
  using mjCBase::name;
  using mjCBase::info;

  void set_material(std::string _material) { material_ = _material; }
  const std::string& get_material() const { return material_; }
  void del_material() { material_.clear(); }

  // API for adding wrapping objects
  void WrapSite(std::string name, std::string_view info = "");                    // site
  void WrapGeom(std::string name, std::string side, std::string_view info = "");  // geom
  void WrapJoint(std::string name, double coef, std::string_view info = "");      // joint
  void WrapPulley(double divisor, std::string_view info = "");                    // pulley

  // API for access to wrapping objects
  int NumWraps() const;                       // number of wraps
  const mjCWrap* GetWrap(int i) const;        // pointer to wrap
  std::vector<mjCWrap*> path;                 // wrapping objects

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() const { return userdata_; }
  const double* get_range() { return range; }

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);
  void SetModel(mjCModel* _model);

  bool is_limited() const;

 private:
  void Compile(void);                         // compiler
};



//------------------------- class mjCWrap ----------------------------------------------------------
// Describes a tendon wrap object

class mjCWrap_ : public mjCBase {
 public:
  mjtWrap type;                   // wrap object type
  int sideid;                     // side site id; -1 if not applicable
  double prm;                     // parameter: divisor, coefficient
  std::string sidesite;           // name of side site
};

class mjCWrap : public mjCWrap_, private mjsWrap {
  friend class mjCTendon;
  friend class mjCModel;

 public:
  mjsWrap spec;
  using mjCBase::info;

  void PointToLocal();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

  mjCBase* obj;                   // wrap object pointer

 private:
  mjCWrap(mjCModel*, mjCTendon*);            // constructor
  mjCWrap(const mjCWrap& other);             // copy constructor
  mjCWrap& operator=(const mjCWrap& other);  // copy assignment

  mjCTendon* tendon;              // tendon owning this wrap
};



//------------------------- class mjCPlugin --------------------------------------------------------
// Describes an instance of a plugin

class mjCPlugin_ : public mjCBase {
 public:
  int nstate;        // state size for the plugin instance
  std::map<std::string, std::string, std::less<>> config_attribs;  // raw config attributes from XML
  std::vector<char> flattened_attributes;  // config attributes flattened in plugin-declared order;

 protected:
  std::string plugin_name;
};

class mjCPlugin : public mjCPlugin_ {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCPlugin(mjCModel*);
  mjCPlugin(const mjCPlugin& other);
  mjCPlugin& operator=(const mjCPlugin& other);
  mjsPlugin spec;
  mjCBase* parent;  // parent object (only used when generating error message)
  int plugin_slot;  // global registered slot number of the plugin

 private:
  void Compile(void);              // compiler
};



//------------------------- class mjCActuator ------------------------------------------------------
// Describes an actuator

class mjCActuator_ : public mjCBase {
 protected:
  int trnid[2];                   // id of transmission target

  // variable used for temporarily storing the state of the actuator
  int actadr_;                                      // address of dof in data->act
  int actdim_;                                      // number of dofs in data->act
  std::map<std::string, std::vector<mjtNum>> act_;  // act at the previous step
  std::map<std::string, mjtNum> ctrl_;              // ctrl at the previous step

  // variable-size data
  std::string plugin_name;
  std::string plugin_instance_name;
  std::string target_;
  std::string slidersite_;
  std::string refsite_;
  std::vector<double> userdata_;
  std::string spec_target_;
  std::string spec_slidersite_;
  std::string spec_refsite_;
  std::vector<double> spec_userdata_;
};

class mjCActuator : public mjCActuator_, private mjsActuator {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCActuator(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCActuator(const mjCActuator& other);
  mjCActuator& operator=(const mjCActuator& other);
  ~mjCActuator();

  mjsActuator spec;
  using mjCBase::name;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() const { return userdata_; }
  const std::string& get_target() const { return spec_target_; }
  const std::string& get_slidersite() const { return spec_slidersite_; }
  const std::string& get_refsite() const { return spec_refsite_; }

  bool is_ctrllimited() const;
  bool is_forcelimited() const;
  bool is_actlimited() const;

  std::vector<mjtNum>& act(const std::string& state_name);
  mjtNum& ctrl(const std::string& state_name);

 private:
  void Compile(void);                       // compiler
  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

  // reset keyframe references for allowing self-attach
  void ForgetKeyframes();

  mjCBase* ptarget;  // transmission target
};



//------------------------- class mjCSensor --------------------------------------------------------
// Describes a sensor

class mjCSensor_ : public mjCBase {
 protected:
  int refid;                      // id of reference frame

  // variable-size data
  std::string plugin_name;
  std::string plugin_instance_name;
  std::string objname_;
  std::string refname_;
  std::vector<double> userdata_;
  std::string spec_objname_;
  std::string spec_refname_;
  std::vector<double> spec_userdata_;
};

class mjCSensor : public mjCSensor_, private mjsSensor {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCSensor(mjCModel*);
  mjCSensor(const mjCSensor& other);
  mjCSensor& operator=(const mjCSensor& other);
  ~mjCSensor();

  mjsSensor spec;
  using mjCBase::name;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }
  const std::string& get_objname() { return spec_objname_; }
  const std::string& get_refname() { return spec_refname_; }

 private:
  void Compile(void);             // compiler
  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

  mjCBase* obj;                   // sensorized object
  mjCBase* ref;                   // sensorized reference
};



//------------------------- class mjCNumeric -------------------------------------------------------
// Describes a custom data field

class mjCNumeric_ : public mjCBase {
 protected:
  std::vector<double> data_;
  std::vector<double> spec_data_;
};

class mjCNumeric : public mjCNumeric_, private mjsNumeric {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCNumeric(mjCModel*);
  mjCNumeric(const mjCNumeric& other);
  mjCNumeric& operator=(const mjCNumeric& other);
  ~mjCNumeric();

  mjsNumeric spec;
  using mjCBase::name;
  using mjCBase::info;

  void PointToLocal();
  void CopyFromSpec();

 private:
  void Compile(void);                 // compiler
};



//------------------------- class mjCText ----------------------------------------------------------
// Describes a custom text field

class mjCText_ : public mjCBase {
 protected:
  std::string data_;
  std::string spec_data_;
};

class mjCText : public mjCText_, private mjsText {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCText(mjCModel*);
  mjCText(const mjCText& other);
  mjCText& operator=(const mjCText& other);
  ~mjCText();

  mjsText spec;
  using mjCBase::name;
  using mjCBase::info;

  void PointToLocal();
  void CopyFromSpec();

 private:
  void Compile(void);                 // compiler
};



//------------------------- class mjCTuple ---------------------------------------------------------
// Describes a custom tuple field

class mjCTuple_ : public mjCBase {
 protected:
  std::vector<mjCBase*> obj;  // object pointers
  std::vector<mjtObj> objtype_;
  std::vector<std::string> objname_;
  std::vector<double> objprm_;
  std::vector<mjtObj> spec_objtype_;
  std::vector<std::string> spec_objname_;
  std::vector<double> spec_objprm_;
};

class mjCTuple : public mjCTuple_, private mjsTuple {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCTuple(mjCModel*);
  mjCTuple(const mjCTuple& other);
  mjCTuple& operator=(const mjCTuple& other);
  ~mjCTuple();

  mjsTuple spec;
  using mjCBase::name;
  using mjCBase::info;

  void PointToLocal();
  void CopyFromSpec();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

 private:
  void Compile(void);             // compiler
};



//------------------------- class mjCKey -----------------------------------------------------------
// Describes a keyframe

class mjCKey_ : public mjCBase {
 protected:
  std::vector<double> qpos_;
  std::vector<double> qvel_;
  std::vector<double> act_;
  std::vector<double> mpos_;
  std::vector<double> mquat_;
  std::vector<double> ctrl_;
  std::vector<double> spec_qpos_;
  std::vector<double> spec_qvel_;
  std::vector<double> spec_act_;
  std::vector<double> spec_mpos_;
  std::vector<double> spec_mquat_;
  std::vector<double> spec_ctrl_;
};

class mjCKey : public mjCKey_, private mjsKey {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCKey(mjCModel*);
  mjCKey(const mjCKey& other);
  mjCKey& operator=(const mjCKey& other);
  ~mjCKey();

  mjsKey spec;
  using mjCBase::name;
  using mjCBase::info;

  void PointToLocal();
  void CopyFromSpec();

 private:
  void Compile(const mjModel* m);  // compiler
};



//------------------------- class mjCDef -----------------------------------------------------------
// Describes one set of defaults

class mjCDef : public mjsElement {
  friend class mjXWriter;

 public:
  mjCDef();
  mjCDef(const mjCDef& other);
  mjCDef& operator=(const mjCDef& other);
  mjCDef& operator+=(const mjCDef& other);

  void CopyWithoutChildren(const mjCDef& other);
  void PointToLocal(void);
  void CopyFromSpec(void);
  void NameSpace(const mjCModel* m);

  void Compile(const mjCModel* model);

  // accessors
  mjCJoint& Joint() { return joint_; }
  mjCGeom& Geom() { return geom_; }
  mjCSite& Site() { return site_; }
  mjCCamera& Camera() { return camera_; }
  mjCLight& Light() { return light_; }
  mjCFlex& Flex() { return flex_; }
  mjCMesh& Mesh() { return mesh_; }
  mjCMaterial& Material() { return material_; }
  mjCPair& Pair() { return pair_; }
  mjCEquality& Equality() { return equality_; }
  mjCTendon& Tendon() { return tendon_; }
  mjCActuator& Actuator() { return actuator_; }

  // identifiers
  std::string name;               // class name
  int id;                         // id of this default
  mjCDef* parent;                 // id of parent class
  std::vector<mjCDef*> child;     // child classes

  mjsDefault spec;

 private:
  mjCJoint joint_;
  mjCGeom geom_;
  mjCSite site_;
  mjCCamera camera_;
  mjCLight light_;
  mjCFlex flex_;
  mjCMesh mesh_;
  mjCMaterial material_;
  mjCPair pair_;
  mjCEquality equality_;
  mjCTendon tendon_;
  mjCActuator actuator_;
};

#endif  // MUJOCO_SRC_USER_USER_OBJECTS_H_
