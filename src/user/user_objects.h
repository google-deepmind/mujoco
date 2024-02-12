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

#include <array>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mjtnum.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include "user/user_api.h"

// forward declarations of all mjC/X classes
class mjCError;
class mjCAlternative;
class mjCBase;
class mjCBody;
class mjCFrame;
class mjCJoint;
class mjCGeom;
class mjCSite;
class mjCCamera;
class mjCLight;
class mjCHField;
class mjCFlex;                        // defined in user_mesh
class mjCMesh;                        // defined in user_mesh
class mjCSkin;                        // defined in user_mesh
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
class mjCModel;                     // defined in user_model
class mjXWriter;                    // defined in xml_native
class mjXURDF;                      // defined in xml_urdf


//------------------------- helper classes and constants -------------------------------------------

// number of positive size parameters for each geom type
const int mjGEOMINFO[mjNGEOMTYPES] = {3, 0, 1, 2, 3, 2, 3, 0};


// builtin type for procedural textures
typedef enum _mjtBuiltin {
  mjBUILTIN_NONE = 0,             // no builtin
  mjBUILTIN_GRADIENT,             // circular gradient: rgb1->rgb2->rgb3
  mjBUILTIN_CHECKER,              // checker pattern: rgb1, rgb2
  mjBUILTIN_FLAT                  // 2d: rgb1; cube: rgb1-up, rgb2-side, rgb3-down
} mjtBuiltin;


// mark type for procedural textures
typedef enum _mjtMark {
  mjMARK_NONE = 0,                // no mark
  mjMARK_EDGE,                    // paint edges
  mjMARK_CROSS,                   // paint cross
  mjMARK_RANDOM                   // paint random dots
} mjtMark;


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
class mjCAlternative : public mjmOrientation {
 public:
  mjCAlternative();                               // constuctor
  const char* Set(double* quat,                   // set frame quat
                  bool degree,                    // angle format: degree/radian
                  const char* sequence);          // euler sequence format: "xyz"
};



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
  int idval_;                   // local id copy for nodes not storing their id's (e.g. faces)
  const int* id_;               // pointer to object id
};


// bounding volume hierarchy
class mjCBoundingVolumeHierarchy {
 public:
  mjCBoundingVolumeHierarchy();

  int nbvh;
  std::vector<mjtNum> bvh;            // bounding boxes                                (nbvh x 6)
  std::vector<int> child;             // children of each node                         (nbvh x 2)
  std::vector<int*> nodeid;           // geom of elem id contained by the node         (nbvh x 1)
  std::vector<int> level;             // levels of each node                           (nbvh x 1)

  // make bounding volume hierarchy
  void CreateBVH(void);
  void Set(mjtNum ipos_element[3], mjtNum iquat_element[4]);
  void AllocateBoundingVolumes(int nbvh);
  void RemoveInactiveVolumes(int nmax);
  mjCBoundingVolume* GetBoundingVolume(int id);

 private:
  int MakeBVH(std::vector<const mjCBoundingVolume*>& elements, int lev = 0);

  std::vector<mjCBoundingVolume> bvh_;
  std::string name_;
  double ipos_[3];
  double iquat_[4];
};



//------------------------- class mjCBase ----------------------------------------------------------
// Generic functionality for all derived classes

class mjCBase {
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

  std::string name;               // object name
  std::string classname;          // defaults class name
  int id;                         // object id
  std::string info;               // error message info set by the user
  mjCDef* def;                    // defaults class used to init this object
  mjCModel* model;                // pointer to model that created object
  mjCFrame* frame;                // pointer to frame transformation

  // plugin support
  mjmPlugin plugin;
  std::string plugin_name;
  std::string plugin_instance_name;
 protected:
  mjCBase();                      // constructor
  virtual ~mjCBase() = default;   // destructor
};



//------------------------- class mjCBody -----------------------------------------------
// Describes a rigid body

class mjCBody : public mjCBase, private mjmBody {
  friend class mjCJoint;
  friend class mjCGeom;
  friend class mjCSite;
  friend class mjCCamera;
  friend class mjCComposite;
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
  // API for adding objects to body
  mjCBody*    AddBody(mjCDef* = 0);
  mjCFrame*   AddFrame(mjCFrame* = 0);
  mjCJoint*   AddJoint(mjCDef* = 0);
  mjCJoint*   AddFreeJoint();
  mjCGeom*    AddGeom(mjCDef* = 0);
  mjCSite*    AddSite(mjCDef* = 0);
  mjCCamera*  AddCamera(mjCDef* = 0);
  mjCLight*   AddLight(mjCDef* = 0);

  // API for accessing objects
  int NumObjects(mjtObj type);
  mjCBase* GetObject(mjtObj type, int id);
  mjCBase* FindObject(mjtObj type, std::string name, bool recursive = true);

  // set explicitinertial to true
  void MakeInertialExplicit();

  // compute quat and diag inertia from fullinertia
  // return nullptr on success, error string on failure
  const char* FullInertia(double quat[4], double inertia[3]);

  // variables set by user
  mjmBody spec;

  // inherited
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;
  using mjCBase::plugin;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }

  mjCAlternative alt_;
  mjCAlternative ialt_;

  // variables computed by 'Compile' and 'AddXXX'
 private:
  mjCBody(mjCModel*);             // constructor
  ~mjCBody();                     // destructor
  void Compile(void);             // compiler
  void GeomFrame(void);           // get inertial info from geoms

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

  // variable-size data
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};



//------------------------- class mjCFrame ---------------------------------------------------------
// Describes a coordinate transformation relative to its parent

class mjCFrame : public mjCBase, private mjmFrame {
  friend class mjCBase;
  friend class mjCBody;
  friend class mjCGeom;
  friend class mjCJoint;
  friend class mjCSite;
  friend class mjCCamera;
  friend class mjCLight;
  friend class mjCModel;

 public:
  mjmFrame spec;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);

 private:
  bool compiled;                           // frame already compiled
  mjCAlternative alt_;

  mjCFrame(mjCModel* = 0, mjCFrame* = 0);  // constructor
  void Compile(void);                      // compiler
};



//------------------------- class mjCJoint ---------------------------------------------------------
// Describes a motion degree of freedom of a body relative to its parent

class mjCJoint : public mjCBase, private mjmJoint {
  friend class mjCDef;
  friend class mjCEquality;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjCSensor;
  friend class mjXWriter;
  friend class mjXURDF;

 public:
  mjmJoint spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec(void);

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }

  // public getters
  bool is_limited() const { return limited_; }
  bool is_actfrclimited() const { return actfrclimited_; }


 private:
  mjCJoint(mjCModel* = 0, mjCDef* = 0);

  int Compile(void);               // compiler; return dofnum
  void PointToLocal(void);

  mjCBody* body;                   // joint's body
  bool limited_;                   // actual (inferred) value of limited
  bool actfrclimited_;             // actual (inferred) value of actfrclimited
  // variable-size data
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};



//------------------------- class mjCGeom ----------------------------------------------------------
// Describes a geometric shape belonging to a body

class mjCGeom : public mjCBase, private mjmGeom {
  friend class mjCDef;
  friend class mjCMesh;
  friend class mjCPair;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjCWrap;
  friend class mjXWriter;
  friend class mjXURDF;

 public:
  using mjCBase::name;
  mjmGeom spec;                       // variables set by user
  double GetVolume(void);             // compute geom volume
  void SetInertia(void);              // compute and set geom inertia
  bool IsVisual(void) const { return visual_; }
  void SetNotVisual(void) { visual_ = false; }

  bool inferinertia;           // true if inertia should be computed from geom

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
  mjCGeom(mjCModel* = 0, mjCDef* = 0);
  void Compile(void);                 // compiler
  double GetRBound(void);             // compute bounding sphere radius
  void ComputeAABB(void);             // compute axis-aligned bounding box
  void CopyFromSpec(void);
  void PointToLocal(void);

  mjCAlternative alt_;
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
  std::string hfieldname_;
  std::string meshname_;
  std::string material_;
  std::vector<double> userdata_;
  std::string spec_hfieldname_;
  std::string spec_meshname_;
  std::string spec_material_;
  std::vector<double> spec_userdata_;

  // inherited
  using mjCBase::classname;
  using mjCBase::info;
  using mjCBase::plugin;
};



//------------------------- class mjCSite ----------------------------------------------------------
// Describes a site on a body

class mjCSite : public mjCBase, private mjmSite {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjXWriter;
  friend class mjXURDF;

 public:
  mjmSite spec;                   // variables set by user

  // use strings from mjCBase rather than mjStrings from mjmSite
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }
  const std::string& get_material() { return material_; }
  void del_material() { material_.clear(); }

 private:
  mjCSite(mjCModel* = 0, mjCDef* = 0);    // constructor
  void Compile(void);                     // compiler
  void CopyFromSpec();                    // copy spec into attributes
  void PointToLocal(void);

  mjCAlternative alt_;

  // variable-size data
  std::string material_;
  std::vector<double> userdata_;
  std::string spec_material_;
  std::vector<double> spec_userdata_;

  // variables computed by 'compile' and 'mjCBody::addSite'
  mjCBody* body;                  // site's body
  int matid;                      // material id for rendering
};



//------------------------- class mjCCamera --------------------------------------------------------
// Describes a camera, attached to a body

class mjCCamera : public mjCBase, private mjmCamera {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjCSensor;
  friend class mjXWriter;

 public:
  mjmCamera spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::string& get_targetbody() { return targetbody_; }
  const std::vector<double>& get_userdata() { return userdata_; }

 private:
  mjCCamera(mjCModel* = 0, mjCDef* = 0);  // constructor
  void Compile(void);                     // compiler
  void CopyFromSpec(void);
  void PointToLocal(void);

  mjCBody* body;                  // camera's body
  int targetbodyid;               // id of target body; -1: none
  mjCAlternative alt_;
  std::string targetbody_;
  std::string spec_targetbody_;
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};



//------------------------- class mjCLight ---------------------------------------------------------
// Describes a light, attached to a body

class mjCLight : public mjCBase, private mjmLight {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjmLight spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  // used by mjXWriter and mjCModel
  const std::string& get_targetbody() { return targetbody_; }

 private:
  mjCLight(mjCModel* = 0, mjCDef* = 0);   // constructor
  void Compile(void);                     // compiler
  void CopyFromSpec(void);

  mjCBody* body;                  // light's body
  int targetbodyid;               // id of target body; -1: none
  std::string targetbody_;
  std::string spec_targetbody_;
};



//------------------------- class mjCFlex ----------------------------------------------------------
// Describes a flex

class mjCFlex: public mjCBase, private mjmFlex {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjCFlexcomp;
  friend class mjCEquality;
  friend class mjXWriter;

 public:
  mjmFlex spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec(void);
  void PointToLocal(void);

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
  mjCFlex(mjCModel* = 0);
  void Compile(const mjVFS* vfs);         // compiler
  void CreateBVH(void);                   // create flex BVH
  void CreateShellPair(void);             // create shells and evpairs

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



//------------------------- class mjCMesh ----------------------------------------------------------
// Describes a mesh

class mjCMesh: public mjCBase {
  friend class mjCFlexcomp;
 public:
  mjCMesh(mjCModel* = 0, mjCDef* = 0);
  ~mjCMesh();

  // public getters
  const std::string& content_type() const { return content_type_; }
  const std::string& file() const { return file_; }
  const std::string& get_file() const { return file_; }
  const double* refpos() const { return refpos_; }
  const double* refquat() const { return refquat_; }
  const double* scale() const { return scale_; }
  bool smoothnormal() const { return smoothnormal_; }

  // public getters for user data
  const std::vector<float>& uservert() const { return uservert_; }
  const std::vector<float>& usernormal() const { return usernormal_; }
  const std::vector<float>& usertexcoord() const { return usertexcoord_; }
  const std::vector<int>& userface() const { return userface_; }

  // mesh properties computed by Compile
  const double* aamm() const { return aamm_; }

  // number of vertices, normals, texture coordinates, and faces
  int nvert() const { return nvert_; }
  int nnormal() const { return nnormal_; }
  int ntexcoord() const { return ntexcoord_; }
  int nface() const { return nface_; }

  // return size of graph data in ints
  int szgraph() const { return szgraph_; }

  // bounding volume hierarchy tree
  const mjCBoundingVolumeHierarchy& tree() { return tree_; }

  // general setters
  void set_file(const std::string& file);
  void set_scale(std::array<double, 3> scale);
  void set_smoothnormal(bool smoothnormal);
  void set_needhull(bool needhull);

  // setters used in reading XML attributes (no-op if empty optional)
  void set_content_type(std::optional<std::string>&& content_type);
  void set_file(std::optional<std::string>&& file);
  void set_refpos(std::optional<std::array<double, 3>> refpos);
  void set_refquat(std::optional<std::array<double, 4>> refquat);
  void set_scale(std::optional<std::array<double, 3>> scale);

  void set_uservert(std::optional<std::vector<float>>&& uservert);
  void set_usernormal(std::optional<std::vector<float>>&& usernormal);
  void set_usertexcoord(std::optional<std::vector<float>>&& usertexcoord);
  void set_userface(std::optional<std::vector<int>>&& userface);

  void Compile(const mjVFS* vfs);                   // compiler
  double* GetPosPtr(mjtGeomInertia type);              // get position
  double* GetQuatPtr(mjtGeomInertia type);             // get orientation
  double* GetOffsetPosPtr();                        // get position offset for geom
  double* GetOffsetQuatPtr();                       // get orientation offset for geom
  double* GetInertiaBoxPtr(mjtGeomInertia type);       // get inertia box
  double& GetVolumeRef(mjtGeomInertia type);           // get volume
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
  bool visual_;                       // true: the mesh is only visual
  std::string content_type_;          // content type of file
  std::string file_;                  // mesh file
  double refpos_[3];                  // reference position (translate)
  double refquat_[4];                 // reference orientation (rotate)
  double scale_[3];                   // rescale mesh
  bool smoothnormal_;                 // do not exclude large-angle faces from normals

  std::vector<float> uservert_;                  // user vertex data
  std::vector<float> usernormal_;                // user normal data
  std::vector<float> usertexcoord_;              // user texcoord data
  std::vector<int> userface_;                    // user vertex indices
  std::vector<int> userfacenormal_;              // user normal indices
  std::vector<int> userfacetexcoord_;            // user texcoord indices
  std::vector< std::pair<int, int> > useredge_;  // user half-edge data

  void LoadOBJ(mjResource* resource);         // load mesh in wavefront OBJ format
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
  void CheckMesh(mjtGeomInertia type);           // check if the mesh is valid

  // compute the volume and center-of-mass of the mesh given the face center
  void ComputeVolume(double CoM[3], mjtGeomInertia type, const double facecen[3],
                     bool exactmeshinertia);

  // mesh properties that indicate a well-formed mesh
  std::pair<int, int> invalidorientation_;    // indices of invalid edge; -1 if none
  bool validarea_;                            // false if the area is too small
  int validvolume_;                           // 0: volume is too small, -1: volume is negative
  bool valideigenvalue_;                      // false if inertia eigenvalue is too small
  bool validinequality_;                      // false if inertia inequality is not satisfied
  bool processed_;                            // false if the mesh has not been processed yet

  // mesh properties computed by Compile
  double pos_volume_[3];              // CoM position
  double pos_surface_[3];             // CoM position
  double quat_volume_[4];             // inertia orientation
  double quat_surface_[4];            // inertia orientation
  double pos_[3];                     // translation applied to asset vertices
  double quat_[4];                    // rotation applied to asset vertices
  double boxsz_volume_[3];            // half-sizes of equivalent inertia box (volume)
  double boxsz_surface_[3];           // half-sizes of equivalent inertia box (surface)
  double aamm_[6];                    // axis-aligned bounding box in (min, max) format
  double volume_;                     // volume of the mesh
  double surface_;                    // surface of the mesh

  // mesh data to be copied into mjModel
  int nvert_;                         // number of vertices
  int nnormal_;                       // number of normals
  int ntexcoord_;                     // number of texcoords
  int nface_;                         // number of faces
  int szgraph_;                       // size of graph data in ints
  float* vert_;                       // vertex data (3*nvert), relative to (pos, quat)
  float* normal_;                     // vertex normal data (3*nnormal)
  double* center_;                    // face circumcenter data (3*nface)
  float* texcoord_;                   // vertex texcoord data (2*ntexcoord or NULL)
  int* face_;                         // face vertex indices (3*nface)
  int* facenormal_;                   // face normal indices (3*nface)
  int* facetexcoord_;                 // face texcoord indices (3*nface)
  int* graph_;                        // convex graph data

  bool needhull_;                     // needs convex hull for collisions

  mjCBoundingVolumeHierarchy tree_;   // bounding volume hierarchy
  std::vector<double> face_aabb_;     // bounding boxes of all faces
};



//------------------------- class mjCSkin ----------------------------------------------------------
// Describes a skin

class mjCSkin: public mjCBase {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  std::string get_file() const { return file; }
  void set_material(std::string _material) { material_ = _material; }
  std::string& get_material() { return material_; }
  void del_material() { material_.clear(); }

  std::string file;                   // skin file
  float rgba[4];                      // rgba when material is omitted
  float inflate;                      // inflate in normal direction
  int group;                          // group for visualization

  // mesh
  std::vector<float> vert;            // vertex positions
  std::vector<float> texcoord;        // texture coordinates
  std::vector<int> face;              // faces

  // skin
  std::vector<std::string> bodyname;  // body names
  std::vector<float> bindpos;         // bind pos
  std::vector<float> bindquat;        // bind quat
  std::vector<std::vector<int>> vertid;         // vertex ids
  std::vector<std::vector<float>> vertweight;   // vertex weights

 private:
  mjCSkin(mjCModel* = 0);                     // constructor
  ~mjCSkin();                                 // destructor
  void Compile(const mjVFS* vfs);             // compiler
  void LoadSKN(mjResource* resource);         // load skin in SKN BIN format

  std::string material_;              // name of material used for rendering
  int matid;                          // material id
  std::vector<int> bodyid;            // body ids
};



//------------------------- class mjCHField --------------------------------------------------------
// Describes a height field

class mjCHField : public mjCBase {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  std::string get_file() const { return file; }

  std::string content_type;       // content type of file
  std::string file;               // file: (nrow, ncol, [elevation data])
  double size[4];                 // hfield size (ignore referencing geom size)
  int nrow;                       // number of rows
  int ncol;                       // number of columns
  float* data;                    // elevation data, row-major format

  // getter for user data
  std::vector<float>& userdata() { return userdata_; }

  // setter for user data
  void set_userdata(std::optional<std::vector<float>>&& userdata);

 private:
  mjCHField(mjCModel* model);             // constructor
  ~mjCHField();                           // destructor
  std::vector<float> userdata_;           // user-provided elevation data
  void Compile(const mjVFS* vfs);         // compiler

  void LoadCustom(mjResource* resource);  // load from custom format
  void LoadPNG(mjResource* resource);     // load from PNG format
};



//------------------------- class mjCTexture -------------------------------------------------------
// Describes a texture

class mjCTexture : public mjCBase {
  friend class mjCModel;
  friend class mjXReader;
  friend class mjXWriter;

 public:
  ~mjCTexture();                  // destructor

  std::string get_file() const { return file; }

  mjtTexture type;                // texture type

  // method 1: builtin
  mjtBuiltin builtin;             // builtin type
  mjtMark mark;                   // mark type
  double rgb1[3];                 // first color for builtin
  double rgb2[3];                 // second color for builtin
  double markrgb[3];              // mark color
  double random;                  // probability of random dots
  int height;                     // height in pixels (square for cube and skybox)
  int width;                      // width in pixels

  // method 2: single file
  std::string content_type;       // content type of file
  std::string file;               // png file to load; use for all sides of cube
  int gridsize[2];                // size of grid for composite file; (1,1)-repeat
  char gridlayout[13];            // row-major: L,R,F,B,U,D for faces; . for unused

  // method 3: separate files
  std::string cubefiles[6];       // different file for each side of the cube

  // flip options
  bool hflip;                     // horizontal flip
  bool vflip;                     // vertical flip

 private:
  mjCTexture(mjCModel*);                  // constructor
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

  mjtByte* rgb;                   // rgb data
};



//------------------------- class mjCMaterial ------------------------------------------------------
// Describes a material for rendering

class mjCMaterial : public mjCBase, private mjmMaterial {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjmMaterial spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec();
  void PointToLocal();

  std::string get_texture() { return texture_; }
  void del_texture() { texture_.clear(); }

 private:
  mjCMaterial(mjCModel* = 0, mjCDef* = 0);  // constructor
  void Compile(void);                       // compiler

  int texid;                      // id of material
  std::string texture_;
  std::string spec_texture_;
};



//------------------------- class mjCPair ----------------------------------------------------------
// Predefined geom pair for collision detection

class mjCPair : public mjCBase {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;

 public:
  // parameters set by user
  std::string geomname1;          // name of geom 1
  std::string geomname2;          // name of geom 2

  // optional parameters: computed from geoms if not set by user
  int condim;                     // contact dimensionality
  mjtNum solref[mjNREF];          // solver reference, normal direction
  mjtNum solreffriction[mjNREF];  // solver reference, frictional directions
  mjtNum solimp[mjNIMP];          // solver impedance
  double margin;                  // margin for contact detection
  double gap;                     // include in solver if dist<margin-gap
  double friction[5];             // full contact friction

  int GetSignature(void) {
    return signature;
  }

 private:
  mjCPair(mjCModel* = 0, mjCDef* = 0);  // constructor
  void Compile(void);                   // compiler

  mjCGeom* geom1;                 // geom1
  mjCGeom* geom2;                 // geom2
  int signature;                  // body1<<16 + body2
};



//------------------------- class mjCBodyPair ------------------------------------------------------
// Body pair specification, use to exclude pairs

class mjCBodyPair : public mjCBase {
  friend class mjCBody;
  friend class mjCModel;

 public:
  // parameters set by user
  std::string bodyname1;          // name of geom 1
  std::string bodyname2;          // name of geom 2

  int GetSignature(void) {
    return signature;
  }

 private:
  mjCBodyPair(mjCModel*);             // constructor
  void Compile(void);                 // compiler

  int body1;                      // id of body1
  int body2;                      // id of body2
  int signature;                  // body1<<16 + body2
};



//------------------------- class mjCEquality ------------------------------------------------------
// Describes an equality constraint

class mjCEquality : public mjCBase, private mjmEquality {
  friend class mjCDef;
  friend class mjCBody;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjmEquality spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  void CopyFromSpec();
  void PointToLocal();

 private:
  mjCEquality(mjCModel* = 0, mjCDef* = 0);  // constructor
  void Compile(void);                       // compiler

  int obj1id;                     // id of object 1
  int obj2id;                     // id of object 2

  std::string name1_;
  std::string name2_;
  std::string spec_name1_;
  std::string spec_name2_;
};



//------------------------- class mjCTendon --------------------------------------------------------
// Describes a tendon

class mjCTendon : public mjCBase, private mjmTendon {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjmTendon spec;
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

  void CopyFromSpec();
  void PointToLocal();

  // public getters
  bool is_limited() const { return limited_; }

 private:
  mjCTendon(mjCModel* = 0, mjCDef* = 0);      // constructor
  ~mjCTendon();                               // destructor
  void Compile(void);                         // compiler

  int matid;                      // material id for rendering
  bool limited_;                  // actual (inferred) value of limited

  // variable-size data
  std::string material_;
  std::string spec_material_;
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};



//------------------------- class mjCWrap ----------------------------------------------------------
// Describes a tendon wrap object

class mjCWrap : public mjCBase, private mjmWrap {
  friend class mjCTendon;
  friend class mjCModel;

 public:
  mjmWrap spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;

  void PointToLocal();

  mjtWrap type;                   // wrap object type
  mjCBase* obj;                   // wrap object pointer
  int sideid;                     // side site id; -1 if not applicable
  double prm;                     // parameter: divisor, coefficient
  std::string sidesite;           // name of side site

 private:
  mjCWrap(mjCModel*, mjCTendon*);     // constructor
  void Compile(void);                 // compiler

  mjCTendon* tendon;              // tendon owning this wrap
};



//------------------------- class mjCPlugin --------------------------------------------------------
// Describes an instance of a plugin

class mjCPlugin : public mjCBase {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  int plugin_slot;   // global registered slot number of the plugin
  int nstate;        // state size for the plugin instance
  mjCBase* parent;   // parent object (only used when generating error message)
  std::map<std::string, std::string, std::less<>> config_attribs;  // raw config attributes from XML
  std::vector<char> flattened_attributes;  // config attributes flattened in plugin-declared order

 private:
  mjCPlugin(mjCModel*);            // constructor
  void Compile(void);              // compiler
};



//------------------------- class mjCActuator ------------------------------------------------------
// Describes an actuator

class mjCActuator : public mjCBase, private mjmActuator {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjmActuator spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;
  using mjCBase::plugin;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }
  const std::string& get_target() { return spec_target_; }
  const std::string& get_slidersite() { return spec_slidersite_; }
  const std::string& get_refsite() { return spec_refsite_; }

  // public getters
  bool is_ctrllimited() const { return ctrllimited_; }
  bool is_forcelimited() const { return forcelimited_; }
  bool is_actlimited() const { return actlimited_; }

 private:
  mjCActuator(mjCModel* = 0, mjCDef* = 0);  // constructor
  void Compile(void);                       // compiler
  void CopyFromSpec();
  void MakePointerLocal();

  int trnid[2];                   // id of transmission target
  bool ctrllimited_;              // actual (inferred) value of ctrllimited
  bool forcelimited_;             // actual (inferred) value of forcelimited
  bool actlimited_;               // actual (inferred) value of actlimited

  // variable-size data
  std::string target_;
  std::string slidersite_;
  std::string refsite_;
  std::vector<double> userdata_;
  std::string spec_target_;
  std::string spec_slidersite_;
  std::string spec_refsite_;
  std::vector<double> spec_userdata_;
};



//------------------------- class mjCSensor --------------------------------------------------------
// Describes a sensor

class mjCSensor : public mjCBase, private mjmSensor {
  friend class mjCDef;
  friend class mjCModel;
  friend class mjXWriter;

 public:
  mjmSensor spec;
  using mjCBase::name;
  using mjCBase::classname;
  using mjCBase::info;
  using mjCBase::plugin;

  // used by mjXWriter and mjCModel
  const std::vector<double>& get_userdata() { return userdata_; }
  const std::string& get_objname() { return spec_objname_; }
  const std::string& get_refname() { return spec_refname_; }

 private:
  mjCSensor(mjCModel*);           // constructor
  void Compile(void);             // compiler
  void CopyFromSpec();
  void MakePointerLocal();

  mjCBase* obj;                   // sensorized object
  int refid;                      // id of reference frame

  // variable-size data
  std::string objname_;
  std::string refname_;
  std::vector<double> userdata_;
  std::string spec_objname_;
  std::string spec_refname_;
  std::vector<double> spec_userdata_;
};



//------------------------- class mjCNumeric -------------------------------------------------------
// Describes a custom data field

class mjCNumeric : public mjCBase {
  friend class mjCModel;

 public:
  // variables set by user
  std::vector<double> data;       // initialization data
  int size;                       // array size, can be bigger than data.size()

 private:
  mjCNumeric(mjCModel*);              // constructor
  ~mjCNumeric();                      // destructor
  void Compile(void);                 // compiler
};



//------------------------- class mjCText ----------------------------------------------------------
// Describes a custom text field

class mjCText : public mjCBase {
  friend class mjCModel;

 public:
  // variables set by user
  std::string data;               // string

 private:
  mjCText(mjCModel*);                 // constructor
  ~mjCText();                         // destructor
  void Compile(void);                 // compiler
};



//------------------------- class mjCTuple ---------------------------------------------------------
// Describes a custom tuple field

class mjCTuple : public mjCBase {
  friend class mjCModel;

 public:
  // variables set by user
  std::vector<mjtObj> objtype;       // object types
  std::vector<std::string> objname;  // object names
  std::vector<double> objprm;        // object parameters

 private:
  mjCTuple(mjCModel*);            // constructor
  ~mjCTuple();                    // destructor
  void Compile(void);             // compiler

  std::vector<mjCBase*> obj;         // object pointers
};



//------------------------- class mjCKey -----------------------------------------------------------
// Describes a keyframe

class mjCKey : public mjCBase {
  friend class mjCModel;
  friend class mjXWriter;

 public:
  double time;                     // time
  std::vector<double> qpos;        // qpos
  std::vector<double> qvel;        // qvel
  std::vector<double> act;         // act
  std::vector<double> mpos;        // mocap pos
  std::vector<double> mquat;       // mocap quat
  std::vector<double> ctrl;        // ctrl

 private:
  mjCKey(mjCModel*);               // constructor
  ~mjCKey();                       // destructor
  void Compile(const mjModel* m);  // compiler
};



//------------------------- class mjCDef -----------------------------------------------------------
// Describes one set of defaults

class mjCDef {
 public:
  mjCDef(void);                           // constructor
  void Compile(const mjCModel* model);    // compiler
  void PointToLocal();

  // identifiers
  std::string name;               // class name
  int parentid;                   // id of parent class
  std::vector<int> childid;       // ids of child classes

  // default objects
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
