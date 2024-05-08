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
#include "user/user_api.h"
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

// compute frame quat and diagonal inertia from full inertia matrix, return error if any
const char* FullInertia(double quat[4], double inertia[3], const double fullinertia[6]);

//------------------------- class mjCBoundingVolumeHierarchy ---------------------------------------

// bounding volume
class mjCBoundingVolume {
 public:
  mjCBoundingVolume() { id_ = nullptr; };

  int contype;                  // contact type
  int conaffinity;              // contact affinity
  const mjtNum* aabb;           // axis-aligned bounding box (center, size)
  const mjtNum* pos;            // position (set by user or Compile1)
  const mjtNum* quat;           // orientation (set by user or Compile1)

  const int* GetId() const { if (id_) return id_; else return &idval_; }
  void SetId(const int* id) { id_ = id; }
  void SetId(int val) { idval_ = val; }

 private:
  int idval_;      // local id copy for nodes not storing their id's (e.g. faces)
  const int* id_;  // pointer to object id
};


// bounding volume hierarchy
struct mjCBoundingVolumeHierarchy_ {
 public:
  int nbvh;
  std::vector<mjtNum> bvh;            // bounding boxes                                (nbvh x 6)
  std::vector<int> child;             // children of each node                         (nbvh x 2)
  std::vector<int*> nodeid;           // geom of elem id contained by the node         (nbvh x 1)
  std::vector<int> level;             // levels of each node                           (nbvh x 1)

 protected:
  std::vector<mjCBoundingVolume> bvleaf_;
  std::string name_;
  double ipos_[3];
  double iquat_[4];
};

class mjCBoundingVolumeHierarchy : public mjCBoundingVolumeHierarchy_ {
 public:
  mjCBoundingVolumeHierarchy();

  // make bounding volume hierarchy
  void CreateBVH(void);
  void Set(mjtNum ipos_element[3], mjtNum iquat_element[4]);
  void AllocateBoundingVolumes(int nleaf);
  void RemoveInactiveVolumes(int nmax);
  mjCBoundingVolume* GetBoundingVolume(int id);

 private:
  // internal class used during BVH construction, for partial sorting of bounding volumes
  struct BVElement {
    const mjCBoundingVolume* e;
    // position of the element in the BVH axes
    mjtNum lpos[3];
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

class mjCBase_ : public mjElement {
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
  static mjResource* LoadResource(std::string filename, const mjVFS* vfs);

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

  mjCDef* def;                    // defaults class used to init this object
  mjCFrame* frame;                // pointer to frame transformation
  mjCModel* model;                // pointer to model that created object

 protected:
  mjCBase();                                 // constructor
  mjCBase(const mjCBase& other);             // copy constructor
  virtual ~mjCBase() = default;              // destructor
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
  mjtNum xpos0[3];                // global position in qpos0
  mjtNum xquat0[4];               // global orientation in qpos0

  // used internally by compiler
  int lastdof;                    // id of last dof
  int subtreedofs;                // number of dofs in subtree, including self

  mjCBoundingVolumeHierarchy tree;  // bounding volume hierarchy

  // variable-size data
  std::string plugin_name;
  std::string plugin_instance_name;
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
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
  using mjCBase::classname;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }

 private:
  mjCBody(const mjCBody& other, mjCModel* _model);  // copy constructor
  mjCBody& operator=(const mjCBody& other);         // copy assignment

  void Compile(void);             // compiler
  void GeomFrame(void);           // get inertial info from geoms

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
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void SetParent(mjCBody* _body);

  mjCFrame& operator+=(const mjCBody& other);

  bool IsAncestor(const mjCFrame* child) const;  // true if child is contained in this frame

 private:
  void Compile(void);                          // compiler

  mjCBody* body;  // body that owns the frame
};



//------------------------- class mjCJoint ---------------------------------------------------------
// Describes a motion degree of freedom of a body relative to its parent

class mjCJoint_ : public mjCBase {
 protected:
  mjCBody* body;                   // joint's body

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
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec(void);

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }
  const double* get_range() { return range; }

  bool is_limited() const;
  bool is_actfrclimited() const;


 private:
  int Compile(void);               // compiler; return dofnum
  void PointToLocal(void);
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

  using mjCBase::name;
  mjsGeom spec;                       // variables set by user
  double GetVolume(void);             // compute geom volume
  void SetInertia(void);              // compute and set geom inertia
  bool IsVisual(void) const { return visual_; }
  void SetNotVisual(void) { visual_ = false; }

  // Compute all coefs modeling the interaction with the surrounding fluid.
  void SetFluidCoefs(void);
  // Compute the kappa coefs of the added inertia due to the surrounding fluid.
  double GetAddedMassKappa(double dx, double dy, double dz);

  // sets properties of a bounding volume
  void SetBoundingVolume(mjCBoundingVolume* bv) const;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }
  const std::string& get_hfieldname() { return spec_hfieldname_; }
  const std::string& get_meshname() { return spec_meshname_; }
  const std::string& get_material() { return spec_material_; }
  void del_material() { spec_material_.clear(); }

 private:
  void Compile(void);                 // compiler
  double GetRBound(void);             // compute bounding sphere radius
  void ComputeAABB(void);             // compute axis-aligned bounding box
  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const mjCModel* m);

  // inherited
  using mjCBase::classname;
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

  // use strings from mjCBase rather than mjStrings from mjsSite
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }
  const std::string& get_material() { return material_; }
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
  using mjCBase::classname;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::string& get_targetbody() { return targetbody_; }
  const std::vector<double>& get_userdata() { return userdata_; }

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
  using mjCBase::classname;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::string& get_targetbody() { return targetbody_; }

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
  std::vector<mjtNum> vertxpos;           // global vertex positions
  mjCBoundingVolumeHierarchy tree;        // bounding volume hierarchy
  std::vector<mjtNum> elemaabb_;          // element bounding volume

  // variable-size data
  std::vector<std::string> vertbody_;     // vertex body names
  std::vector<mjtNum> vert_;              // vertex positions
  std::vector<int> elem_;                 // element vertex ids
  std::vector<float> texcoord_;           // vertex texture coordinates
  std::string material_;                  // name of material used for rendering

  std::string spec_material_;
  std::vector<std::string> spec_vertbody_;
  std::vector<mjtNum> spec_vert_;
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
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

  // used by mjXWriter and mjCModel
  const std::string& get_material() { return material_; }
  const std::vector<std::string>& get_vertbody() { return vertbody_; }
  const std::vector<double>& get_vert() { return vert_; }
  const std::vector<double>& get_elemaabb() { return elemaabb_; }
  const std::vector<int>& get_elem() { return elem_; }
  const std::vector<float>& get_texcoord() { return texcoord_; }

  bool HasTexcoord() const;               // texcoord not null
  void DelTexcoord();                     // delete texcoord

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

  mjCBoundingVolumeHierarchy tree_;   // bounding volume hierarchy
  std::vector<double> face_aabb_;     // bounding boxes of all faces
};

class mjCMesh: public mjCMesh_, private mjsMesh {
  friend class mjCModel;
  friend class mjCFlexcomp;
  friend class mjXWriter;
 public:
  mjCMesh(mjCModel* = nullptr, mjCDef* = nullptr);
  mjCMesh(const mjCMesh& other);
  mjCMesh& operator=(const mjCMesh& other);
  ~mjCMesh();

  mjsMesh spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);

  // public getters and setters
  const std::string& get_content_type() const { return content_type_; }
  const std::string& get_file() const { return file_; }
  const double* get_refpos() const { return refpos; }
  const double* get_refquat() const { return refquat; }
  const double* get_scale() const { return scale; }
  bool get_smoothnormal() const { return smoothnormal; }
  void set_needhull(bool needhull);

  // public getters for user data
  const std::vector<float>& get_uservert() const { return spec_vert_; }
  const std::vector<float>& get_usernormal() const { return spec_normal_; }
  const std::vector<float>& get_usertexcoord() const { return spec_texcoord_; }
  const std::vector<int>& get_userface() const { return spec_face_; }

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

 private:
  void LoadOBJ(mjResource* resource);         // load mesh in wavefront OBJ format
  bool LoadCachedOBJ(const mjCAsset& asset);  // load OBJ from cache asset, return true on success
  void LoadSTL(mjResource* resource);         // load mesh in STL BIN format
  void LoadMSH(mjResource* resource);         // load mesh in MSH BIN format
  void LoadSDF();                             // generate mesh using marching cubes
  void MakeGraph(void);                       // make graph of convex hull
  void CopyGraph(void);                       // copy graph into face data
  void MakeNormal(void);                      // compute vertex normals
  void MakeCenter(void);                      // compute face circumcircle data
  void Process();                             // compute inertial properties
  void ApplyTransformations();                // apply user transformations
  void ComputeFaceCentroid(double[3]);        // compute centroid of all faces
  void RemoveRepeated(void);                  // remove repeated vertices
  void CheckMesh(mjtGeomInertia type);        // check if the mesh is valid

  // mesh data to be copied into mjModel
  double* center_;                    // face circumcenter data (3*nface)
  int* graph_;                        // convex graph data

  // compute the volume and center-of-mass of the mesh given the face center
  void ComputeVolume(double CoM[3], mjtGeomInertia type, const double facecen[3],
                     bool exactmeshinertia);
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
  using mjCBase::classname;
  using mjCBase::info;

  std::string get_file() const { return file_; }
  std::string& get_material() { return material_; }
  std::vector<float>& get_vert() { return vert_; }
  std::vector<float>& get_texcoord() { return texcoord_; }
  std::vector<int>& get_face() { return face_; }
  std::vector<std::string>& get_bodyname() { return bodyname_; }
  std::vector<float>& get_bindpos() { return bindpos_; }
  std::vector<float>& get_bindquat() { return bindquat_; }
  std::vector<std::vector<int>>& get_vertid() { return vertid_; }
  std::vector<std::vector<float>>& get_vertweight() { return vertweight_; }
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

  std::string get_file() const { return file_; }

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
  std::vector<mjtByte> rgb;                   // rgb data

  std::string file_;
  std::string content_type_;
  std::vector<std::string> cubefiles_;
  std::string spec_file_;
  std::string spec_content_type_;
  std::vector<std::string> spec_cubefiles_;
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
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);

  std::string get_file() const { return file_; }
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
};



//------------------------- class mjCMaterial ------------------------------------------------------
// Describes a material for rendering

class mjCMaterial_ : public mjCBase {
 protected:
  int texid;                      // id of material
  std::string texture_;
  std::string spec_texture_;
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
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec();
  void PointToLocal();
  void NameSpace(const mjCModel* m);

  std::string get_texture() { return texture_; }
  void del_texture() { texture_.clear(); }

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
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

  std::string get_geomname1() { return geomname1_; }
  std::string get_geomname2() { return geomname2_; }

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

  int GetSignature(void) {
    return signature;
  }

 private:

  void Compile(void);              // compiler
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
  using mjCBase::classname;
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
  using mjCBase::classname;
  using mjCBase::info;

  void set_material(std::string _material) { material_ = _material; }
  std::string& get_material() { return material_; }
  void del_material() { material_.clear(); }

  // API for adding wrapping objects
  void WrapSite(std::string name, std::string_view info = "");                    // site
  void WrapGeom(std::string name, std::string side, std::string_view info = "");  // geom
  void WrapJoint(std::string name, double coef, std::string_view info = "");      // joint
  void WrapPulley(double divisor, std::string_view info = "");                    // pulley

  // API for access to wrapping objects
  int NumWraps(void);                         // number of wraps
  mjCWrap* GetWrap(int);                      // pointer to wrap
  std::vector<mjCWrap*> path;                 // wrapping objects

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }
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
  std::string instance_name;
};

class mjCPlugin : public mjCPlugin_ {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjCPlugin(mjCModel*);
  mjCPlugin(const mjCPlugin& other);
  mjCPlugin& operator=(const mjCPlugin& other);
  mjsPlugin spec;
  mjCBase* parent;   // parent object (only used when generating error message)

 private:
  void Compile(void);              // compiler
};



//------------------------- class mjCActuator ------------------------------------------------------
// Describes an actuator

class mjCActuator_ : public mjCBase {
 protected:
  int trnid[2];                   // id of transmission target

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

  mjsActuator spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }
  const std::string& get_target() { return spec_target_; }
  const std::string& get_slidersite() { return spec_slidersite_; }
  const std::string& get_refsite() { return spec_refsite_; }

  bool is_ctrllimited() const;
  bool is_forcelimited() const;
  bool is_actlimited() const;

 private:
  void Compile(void);                       // compiler
  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const mjCModel* m);
  void NameSpace(const mjCModel* m);

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

  mjsSensor spec;
  using mjCBase::name;
  using mjCBase::classname;
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

class mjCDef : public mjElement {
  friend class mjXWriter;

 public:
  mjCDef(void);                            // constructor
  mjCDef(const mjCDef& other);             // copy constructor
  void Compile(const mjCModel* model);     // compiler
  mjCDef& operator=(const mjCDef& other);  // copy assignment
  void PointToLocal(void);
  void CopyFromSpec(void);

  // identifiers
  std::string name;               // class name
  int parentid;                   // id of parent class
  std::vector<int> childid;       // ids of child classes

  mjsDefault spec;

  // default objects (TODO: they should become private)
  mjCJoint    joint;
  mjCGeom     geom;
  mjCSite     site;
  mjCCamera   camera;
  mjCLight    light;
  mjCFlex     flex;
  mjCMesh     mesh;
  mjCMaterial material;
  mjCPair     pair;
  mjCEquality equality;
  mjCTendon   tendon;
  mjCActuator actuator;
};

#endif  // MUJOCO_SRC_USER_USER_OBJECTS_H_
