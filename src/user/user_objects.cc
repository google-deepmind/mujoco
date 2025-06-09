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

#include "user/user_objects.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "lodepng.h"
#include "cc/array_safety.h"
#include "engine/engine_passive.h"
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "user/user_api.h"
#include "user/user_cache.h"
#include "user/user_model.h"
#include "user/user_resource.h"
#include "user/user_util.h"

namespace {
namespace mju = ::mujoco::util;
using mujoco::user::FilePath;

class PNGImage {
 public:
  static PNGImage Load(const mjCBase* obj, mjResource* resource,
                       LodePNGColorType color_type);
  int Width() const { return width_; }
  int Height() const { return height_; }
  bool IsSRGB() const { return is_srgb_; }

  uint8_t operator[] (int i) const { return data_[i]; }
  std::vector<unsigned char>& MoveData() { return data_; }

 private:
  std::size_t Size() const {
    return data_.size() + (3 * sizeof(int));
  }

  int width_;
  int height_;
  bool is_srgb_;
  LodePNGColorType color_type_;
  std::vector<uint8_t> data_;
};

PNGImage PNGImage::Load(const mjCBase* obj, mjResource* resource,
                        LodePNGColorType color_type) {
  PNGImage image;
  image.color_type_ = color_type;
  mjCCache *cache = reinterpret_cast<mjCCache*>(mj_globalCache());

  // cache callback
  auto callback = [&image](const void* data) {
    const PNGImage *cached_image = static_cast<const PNGImage*>(data);
    if (cached_image->color_type_ == image.color_type_) {
      image = *cached_image;
      return true;
    }
    return false;
  };

  // try loading from cache
  if (cache && cache->PopulateData(resource, callback)) {
      return image;
  }

  // open PNG resource
  const unsigned char* buffer;
  int nbuffer = mju_readResource(resource, (const void**) &buffer);

  if (nbuffer < 0) {
    throw mjCError(obj, "could not read PNG file '%s'", resource->name);
  }

  if (!nbuffer) {
    throw mjCError(obj, "empty PNG file '%s'", resource->name);
  }

  // decode PNG from buffer
  unsigned int w, h;

  lodepng::State state;
  state.info_raw.colortype = image.color_type_;
  state.info_raw.bitdepth = 8;
  unsigned err = lodepng::decode(image.data_, w, h, state, buffer, nbuffer);

  // check for errors
  if (err) {
    std::stringstream ss;
    ss << "error decoding PNG file '" << resource->name << "': " << lodepng_error_text(err);
    throw mjCError(obj, "%s", ss.str().c_str());
  }

  image.width_ = w;
  image.height_ = h;
  image.is_srgb_ = (state.info_png.srgb_defined == 1);

  if (image.width_ <= 0 || image.height_ < 0) {
    std::stringstream ss;
    ss << "error decoding PNG file '" << resource->name << "': " << "dimensions are invalid";
    throw mjCError(obj, "%s", ss.str().c_str());
  }

  // insert raw image data into cache
  if (cache) {
    PNGImage *cached_image = new PNGImage(image);;
    std::size_t size = image.Size();
    std::shared_ptr<const void> cached_data(cached_image, +[] (const void* data) {
        delete static_cast<const PNGImage*>(data);
      });
    cache->Insert("", resource, cached_data, size);
  }

  return image;
}

// associate all child list elements with a frame and copy them to parent list, clear child list
template <typename T>
void MapFrame(std::vector<T*>& parent, std::vector<T*>& child,
              mjCFrame* frame, mjCBody* parent_body) {
  std::for_each(child.begin(), child.end(), [frame, parent_body](T* element) {
      element->SetFrame(frame);
      element->SetParent(parent_body);
    });
  parent.insert(parent.end(), child.begin(), child.end());
  child.clear();
}

}  // namespace


// utiility function for checking size parameters
static void checksize(double* size, mjtGeom type, mjCBase* object, const char* name, int id) {
  // plane: handle infinite
  if (type == mjGEOM_PLANE) {
    if (size[2] <= 0) {
      throw mjCError(object, "plane size(3) must be positive");
    }
  }

  // regular geom
  else {
    for (int i=0; i < mjGEOMINFO[type]; i++) {
      if (size[i] <= 0) {
        throw mjCError(object, "size %d must be positive in geom", nullptr, i);
      }
    }
  }
}

// error message for missing "limited" attribute
static void checklimited(
  const mjCBase* obj,
  bool autolimits, const char* entity, const char* attr, int limited, bool hasrange) {
  if (!autolimits && limited == 2 && hasrange) {
    std::stringstream ss;
    ss << entity << " has `" << attr << "range` but not `" << attr << "limited`. "
       << "set the autolimits=\"true\" compiler option, specify `" << attr << "limited` "
       << "explicitly (\"true\" or \"false\"), or remove the `" << attr << "range` attribute.";
    throw mjCError(obj, "%s", ss.str().c_str());
  }
}

// returns true if limits should be active
static bool islimited(int limited, const double range[2]) {
  if (limited == mjLIMITED_TRUE || (limited == mjLIMITED_AUTO && range[0] < range[1])) {
    return true;
  }
  return false;
}

//------------------------- class mjCError implementation ------------------------------------------

// constructor
mjCError::mjCError(const mjCBase* obj, const char* msg, const char* str, int pos1, int pos2) {
  char temp[600];

  // init
  warning = false;
  if (obj || msg) {
    mju::sprintf_arr(message, "Error");
  } else {
    message[0] = 0;
  }

  // construct error message
  if (msg) {
    if (str) {
      mju::sprintf_arr(temp, msg, str, pos1, pos2);
    } else {
      mju::sprintf_arr(temp, msg, pos1, pos2);
    }

    mju::strcat_arr(message, ": ");
    mju::strcat_arr(message, temp);
  }

  // append info from mjCBase element
  if (obj) {
    // with or without xml position
    if (!obj->info.empty()) {
      mju::sprintf_arr(temp, "Element name '%s', id %d, %s",
                       obj->name.c_str(), obj->id, obj->info.c_str());
    } else {
      mju::sprintf_arr(temp, "Element name '%s', id %d", obj->name.c_str(), obj->id);
    }

    // append to message
    mju::strcat_arr(message, "\n");
    mju::strcat_arr(message, temp);
  }
}



//------------------ alternative orientation implementation ----------------------------------------

// compute frame orientation given alternative specifications
// used for geom, site, body and camera frames
const char* ResolveOrientation(double* quat, bool degree, const char* sequence,
                               const mjsOrientation& orient) {
  double axisangle[4];
  double xyaxes[6];
  double zaxis[3];
  double euler[3];

  mjuu_copyvec(axisangle, orient.axisangle, 4);
  mjuu_copyvec(xyaxes, orient.xyaxes, 6);
  mjuu_copyvec(zaxis, orient.zaxis, 3);
  mjuu_copyvec(euler, orient.euler, 3);

  // set quat using axisangle
  if (orient.type == mjORIENTATION_AXISANGLE) {
    // convert to radians if necessary, normalize axis
    if (degree) {
      axisangle[3] = axisangle[3] / 180.0 * mjPI;
    }
    if (mjuu_normvec(axisangle, 3) < mjEPS) {
      return "axisangle too small";
    }

    // construct quaternion
    double ang2 = axisangle[3]/2;
    quat[0] = cos(ang2);
    quat[1] = sin(ang2)*axisangle[0];
    quat[2] = sin(ang2)*axisangle[1];
    quat[3] = sin(ang2)*axisangle[2];
  }

  // set quat using xyaxes
  if (orient.type == mjORIENTATION_XYAXES) {
    // normalize x axis
    if (mjuu_normvec(xyaxes, 3) < mjEPS) {
      return "xaxis too small";
    }

    // make y axis orthogonal to x axis, normalize
    double d = mjuu_dot3(xyaxes, xyaxes+3);
    xyaxes[3] -= xyaxes[0]*d;
    xyaxes[4] -= xyaxes[1]*d;
    xyaxes[5] -= xyaxes[2]*d;
    if (mjuu_normvec(xyaxes+3, 3) < mjEPS) {
      return "yaxis too small";
    }

    // compute and normalize z axis
    double z[3];
    mjuu_crossvec(z, xyaxes, xyaxes+3);
    if (mjuu_normvec(z, 3) < mjEPS) {
      return "cross(xaxis, yaxis) too small";
    }

    // convert frame into quaternion
    mjuu_frame2quat(quat, xyaxes, xyaxes+3, z);
  }

  // set quat using zaxis
  if (orient.type == mjORIENTATION_ZAXIS) {
    if (mjuu_normvec(zaxis, 3) < mjEPS) {
      return "zaxis too small";
    }
    mjuu_z2quat(quat, zaxis);
  }


  // handle euler
  if (orient.type == mjORIENTATION_EULER) {
    // convert to radians if necessary
    if (degree) {
      for (int i=0; i < 3; i++) {
        euler[i] = euler[i] / 180.0 * mjPI;
      }
    }

    // init
    mjuu_setvec(quat, 1, 0, 0, 0);

    // loop over euler angles, accumulate rotations
    for (int i=0; i < 3; i++) {
      double tmp[4], qrot[4] = {cos(euler[i]/2), 0, 0, 0};
      double sa = sin(euler[i]/2);

      // construct quaternion rotation
      if (sequence[i] == 'x' || sequence[i] == 'X') {
        qrot[1] = sa;
      } else if (sequence[i] == 'y' || sequence[i] == 'Y') {
        qrot[2] = sa;
      } else if (sequence[i] == 'z' || sequence[i] == 'Z') {
        qrot[3] = sa;
      } else {
        return "euler sequence can only contain x, y, z, X, Y, Z";
      }

      // accumulate rotation
      if (sequence[i] == 'x' || sequence[i] == 'y' || sequence[i] == 'z') {
        mjuu_mulquat(tmp, quat, qrot);  // moving axes: post-multiply
      } else {
        mjuu_mulquat(tmp, qrot, quat);  // fixed axes: pre-multiply
      }
      mjuu_copyvec(quat, tmp, 4);
    }

    // normalize, just in case
    mjuu_normvec(quat, 4);
  }

  return 0;
}



//------------------------- class mjCBoundingVolumeHierarchy implementation ------------------------


// assign position and orientation
void mjCBoundingVolumeHierarchy::Set(double ipos_element[3], double iquat_element[4]) {
  mjuu_copyvec(ipos_, ipos_element, 3);
  mjuu_copyvec(iquat_, iquat_element, 4);
}



void mjCBoundingVolumeHierarchy::AllocateBoundingVolumes(int nleaf) {
  nbvh_ = 0;
  bvh_.clear();
  child_.clear();
  nodeid_.clear();
  level_.clear();
  bvleaf_.clear();
  bvleaf_.reserve(nleaf);
}


void mjCBoundingVolumeHierarchy::RemoveInactiveVolumes(int nmax) {
  bvleaf_.erase(bvleaf_.begin() + nmax, bvleaf_.end());
}

const mjCBoundingVolume*
mjCBoundingVolumeHierarchy::AddBoundingVolume(int id, int contype, int conaffinity,
                                              const double* pos, const double* quat,
                                              const double* aabb) {
  bvleaf_.emplace_back(id, contype, conaffinity, pos, quat, aabb);
  return &bvleaf_.back();
}


const mjCBoundingVolume*
mjCBoundingVolumeHierarchy::AddBoundingVolume(const int* id, int contype, int conaffinity,
                                              const double* pos, const double* quat,
                                              const double* aabb) {
  bvleaf_.emplace_back(id, contype, conaffinity, pos, quat, aabb);
  return &bvleaf_.back();
}


// create bounding volume hierarchy
void mjCBoundingVolumeHierarchy::CreateBVH() {
  // precompute the positions of each element in the hierarchy's axes, and drop
  // visual-only elements.
  std::vector<BVElement> elements;
  elements.reserve(bvleaf_.size());
  double qinv[4] = {iquat_[0], -iquat_[1], -iquat_[2], -iquat_[3]};
  for (int i = 0; i < bvleaf_.size(); i++) {
    if (bvleaf_[i].Conaffinity() || bvleaf_[i].Contype()) {
      BVElement element;
      element.e = &bvleaf_[i];
      double vert[3] = {element.e->Pos(0) - ipos_[0],
                        element.e->Pos(1) - ipos_[1],
                        element.e->Pos(2) - ipos_[2]};
      mjuu_rotVecQuat(element.lpos, vert, qinv);
      elements.push_back(std::move(element));
    }
  }
  MakeBVH(elements.begin(), elements.end());
}

// compute bounding volume hierarchy
int mjCBoundingVolumeHierarchy::MakeBVH(
  std::vector<BVElement>::iterator elements_begin,
  std::vector<BVElement>::iterator elements_end, int lev) {
  int nelements = elements_end - elements_begin;
  if (nelements == 0) {
    return -1;
  }
  constexpr double kMaxVal = std::numeric_limits<double>::max();
  double AAMM[6] = {kMaxVal, kMaxVal, kMaxVal, -kMaxVal, -kMaxVal, -kMaxVal};

  // inverse transformation
  double qinv[4] = {iquat_[0], -iquat_[1], -iquat_[2], -iquat_[3]};

  // accumulate AAMM over elements
  for (auto element = elements_begin; element != elements_end; ++element) {
    // transform element aabb to aamm format
    double aamm[6] = {element->e->AABB(0) - element->e->AABB(3),
                      element->e->AABB(1) - element->e->AABB(4),
                      element->e->AABB(2) - element->e->AABB(5),
                      element->e->AABB(0) + element->e->AABB(3),
                      element->e->AABB(1) + element->e->AABB(4),
                      element->e->AABB(2) + element->e->AABB(5)};

    // update node AAMM
    for (int v=0; v < 8; v++) {
      double vert[3], box[3];
      vert[0] = (v&1 ? aamm[3] : aamm[0]);
      vert[1] = (v&2 ? aamm[4] : aamm[1]);
      vert[2] = (v&4 ? aamm[5] : aamm[2]);

      // rotate to the body inertial frame if specified
      if (element->e->Quat()) {
        mjuu_rotVecQuat(box, vert, element->e->Quat());
        box[0] += element->e->Pos(0) - ipos_[0];
        box[1] += element->e->Pos(1) - ipos_[1];
        box[2] += element->e->Pos(2) - ipos_[2];
        mjuu_rotVecQuat(vert, box, qinv);
      }

      AAMM[0] = std::min(AAMM[0], vert[0]);
      AAMM[1] = std::min(AAMM[1], vert[1]);
      AAMM[2] = std::min(AAMM[2], vert[2]);
      AAMM[3] = std::max(AAMM[3], vert[0]);
      AAMM[4] = std::max(AAMM[4], vert[1]);
      AAMM[5] = std::max(AAMM[5], vert[2]);
    }
  }

  // inflate flat AABBs
  for (int i = 0; i < 3; i++) {
    if (std::abs(AAMM[i] - AAMM[i+3]) < mjEPS) {
      AAMM[i + 0] -= mjEPS;
      AAMM[i + 3] += mjEPS;
    }
  }

  // store current index
  int index = nbvh_++;
  child_.push_back(-1);
  child_.push_back(-1);
  nodeid_.push_back(-1);
  nodeidptr_.push_back(nullptr);
  level_.push_back(lev);

  // store bounding box of the current node
  bvh_.push_back((AAMM[3] + AAMM[0]) / 2);
  bvh_.push_back((AAMM[4] + AAMM[1]) / 2);
  bvh_.push_back((AAMM[5] + AAMM[2]) / 2);
  bvh_.push_back((AAMM[3] - AAMM[0]) / 2);
  bvh_.push_back((AAMM[4] - AAMM[1]) / 2);
  bvh_.push_back((AAMM[5] - AAMM[2]) / 2);

  // leaf node, return
  if (nelements == 1) {
    child_[2*index + 0] = -1;
    child_[2*index + 1] = -1;
    nodeid_[index] = *elements_begin->e->Id();
    nodeidptr_[index] = (int*)elements_begin->e->Id();
    return index;
  }

  // find longest axis, by a margin of at least mjEPS, default to 0
  int axis = 0;
  double edges[3] = {AAMM[3] - AAMM[0], AAMM[4] - AAMM[1], AAMM[5] - AAMM[2]};
  if (edges[1] >= edges[0] + mjEPS) axis = 1;
  if (edges[2] >= edges[axis] + mjEPS) axis = 2;

  // find median along the axis
  auto compare = [&](const BVElement& e1, const BVElement& e2) {
    if (std::abs(e1.lpos[axis] - e2.lpos[axis]) > mjEPS) {
      return e1.lpos[axis] < e2.lpos[axis];
    }
    // comparing pointers gives a stable sort, because they both come from the same array
    return e1.e < e2.e;
  };

  // note: nth_element performs a partial sort of elements
  int m = nelements / 2;
  std::nth_element(elements_begin, elements_begin + m, elements_end, compare);

  // recursive calls
  if (m > 0) {
    child_[2*index + 0] = MakeBVH(elements_begin, elements_begin + m, lev + 1);
  }

  if (m != nelements) {
    child_[2*index + 1] = MakeBVH(elements_begin + m, elements_end, lev + 1);
  }

  // SHOULD NOT OCCUR
  if (child_[2*index + 0] == -1 && child_[2*index + 1] == -1) {
    mju_error("this should have been a leaf, body=%s nelements=%d",
              name_.c_str(), nelements);
  }

  if (lev > mjMAXTREEDEPTH) {
    mju_warning("max tree depth exceeded in body=%s", name_.c_str());
  }

  return index;
}

//------------------------- class mjCDef implementation --------------------------------------------



// constructor
mjCDef::mjCDef() {
  name.clear();
  id = 0;
  parent = nullptr;
  model = 0;
  child.clear();
  elemtype = mjOBJ_DEFAULT;
  mjs_defaultJoint(&joint_.spec);
  mjs_defaultGeom(&geom_.spec);
  mjs_defaultSite(&site_.spec);
  mjs_defaultCamera(&camera_.spec);
  mjs_defaultLight(&light_.spec);
  mjs_defaultFlex(&flex_.spec);
  mjs_defaultMesh(&mesh_.spec);
  mjs_defaultMaterial(&material_.spec);
  mjs_defaultPair(&pair_.spec);
  mjs_defaultEquality(&equality_.spec);
  mjs_defaultTendon(&tendon_.spec);
  mjs_defaultActuator(&actuator_.spec);

  // make sure all the pointers are local
  PointToLocal();
}



// constructor with model
mjCDef::mjCDef(mjCModel* _model) : mjCDef() {
  model = _model;
}



// copy constructor
mjCDef::mjCDef(const mjCDef& other) {
  *this = other;
}



// compiler
void mjCDef::Compile(const mjCModel* model) {
  CopyFromSpec();

  // enforce length of all default userdata arrays
  joint_.userdata_.resize(model->nuser_jnt);
  geom_.userdata_.resize(model->nuser_geom);
  site_.userdata_.resize(model->nuser_site);
  camera_.userdata_.resize(model->nuser_cam);
  tendon_.userdata_.resize(model->nuser_tendon);
  actuator_.userdata_.resize(model->nuser_actuator);
}



// assignment operator
mjCDef& mjCDef::operator=(const mjCDef& other) {
  if (this != &other) {
    CopyWithoutChildren(other);

    // copy the rest of the default tree
    *this += other;
  }
  return *this;
}



mjCDef& mjCDef::operator+=(const mjCDef& other) {
  for (unsigned int i=0; i < other.child.size(); i++) {
    child.push_back(new mjCDef(*other.child[i]));  // triggers recursive call
    child.back()->parent = this;
  }
  return *this;
}



void mjCDef::NameSpace(const mjCModel* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  for (auto c : child) {
    c->NameSpace(m);
  }
}



void mjCDef::CopyWithoutChildren(const mjCDef& other) {
  name = other.name;
  parent = nullptr;
  child.clear();
  joint_ = other.joint_;
  geom_ = other.geom_;
  site_ = other.site_;
  camera_ = other.camera_;
  light_ = other.light_;
  flex_ = other.flex_;
  mesh_ = other.mesh_;
  material_ = other.material_;
  pair_ = other.pair_;
  equality_ = other.equality_;
  tendon_ = other.tendon_;
  actuator_ = other.actuator_;
  PointToLocal();
}



void mjCDef::PointToLocal() {
  joint_.PointToLocal();
  geom_.PointToLocal();
  site_.PointToLocal();
  camera_.PointToLocal();
  light_.PointToLocal();
  flex_.PointToLocal();
  mesh_.PointToLocal();
  material_.PointToLocal();
  pair_.PointToLocal();
  equality_.PointToLocal();
  tendon_.PointToLocal();
  actuator_.PointToLocal();
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.joint = &joint_.spec;
  spec.geom = &geom_.spec;
  spec.site = &site_.spec;
  spec.camera = &camera_.spec;
  spec.light = &light_.spec;
  spec.flex = &flex_.spec;
  spec.mesh = &mesh_.spec;
  spec.material = &material_.spec;
  spec.pair = &pair_.spec;
  spec.equality = &equality_.spec;
  spec.tendon = &tendon_.spec;
  spec.actuator = &actuator_.spec;
}



void mjCDef::CopyFromSpec() {
  joint_.CopyFromSpec();
  geom_.CopyFromSpec();
  site_.CopyFromSpec();
  camera_.CopyFromSpec();
  light_.CopyFromSpec();
  flex_.CopyFromSpec();
  mesh_.CopyFromSpec();
  material_.CopyFromSpec();
  pair_.CopyFromSpec();
  equality_.CopyFromSpec();
  tendon_.CopyFromSpec();
  actuator_.CopyFromSpec();
}



//------------------------- class mjCBase implementation -------------------------------------------

// constructor
mjCBase::mjCBase() {
  name.clear();
  classname.clear();
  id = -1;
  info = "";
  model = 0;
  frame = nullptr;
}



mjCBase::mjCBase(const mjCBase& other) {
  *this = other;
}



mjCBase& mjCBase::operator=(const mjCBase& other) {
  if (this != &other) {
    *static_cast<mjCBase_*>(this) = static_cast<const mjCBase_&>(other);
  }
  return *this;
}



void mjCBase::NameSpace(const mjCModel* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  if (!classname.empty() && classname != "main" && m != model) {
    classname = m->prefix + classname + m->suffix;
  }
}



// load resource if found (fallback to OS filesystem)
mjResource* mjCBase::LoadResource(const std::string& modelfiledir,
                                  const std::string& filename,
                                  const mjVFS* vfs) {
  // try reading from provided VFS or fallback to OS filesystem
  std::array<char, 1024> error;
  mjResource* resource = mju_openResource(modelfiledir.c_str(), filename.c_str(), vfs,
                                          error.data(), error.size());
  if (!resource) {
    throw mjCError(nullptr, "%s", error.data());
  }
  return resource;
}


// Get and sanitize content type from raw_text if not empty, otherwise parse
// content type from resource_name; throw error on failure
std::string mjCBase::GetAssetContentType(std::string_view resource_name,
                                         std::string_view raw_text) {
  if (!raw_text.empty()) {
    auto type = mjuu_parseContentTypeAttrType(raw_text);
    auto subtype = mjuu_parseContentTypeAttrSubtype(raw_text);
    if (!type.has_value() || !subtype.has_value()) {
      return "";
    }
    return std::string(*type) + "/" + std::string(*subtype);
  } else {
    return mjuu_extToContentType(resource_name);
  }
}


void mjCBase::SetFrame(mjCFrame* _frame) {
  if (!_frame) {
    return;
  }
  if (_frame->body && GetParent() != _frame->body) {
    throw mjCError(this, "Frame and body '%s' have mismatched parents", name.c_str());
  }
  frame = _frame;
}

void mjCBase::SetUserValue(std::string_view key, const void* data,
                           void (*cleanup)(const void*)) {
  user_payload_[std::string(key)] = UserValue(data, cleanup);
}

const void* mjCBase::GetUserValue(std::string_view key) {
  auto found = user_payload_.find(std::string(key));
  return found != user_payload_.end() ? found->second.value : nullptr;
}


void mjCBase::DeleteUserValue(std::string_view key) {
  user_payload_.erase(std::string(key));
}


//------------------ class mjCBody implementation --------------------------------------------------

// constructor
mjCBody::mjCBody(mjCModel* _model) {
  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  refcount = 1;
  mjs_defaultBody(&spec);
  elemtype = mjOBJ_BODY;
  parent = nullptr;
  weldid = -1;
  dofnum = 0;
  lastdof = -1;
  subtreedofs = 0;
  contype = 0;
  conaffinity = 0;
  margin = 0;
  mjuu_zerovec(xpos0, 3);
  mjuu_setvec(xquat0, 1, 0, 0, 0);
  last_attached = nullptr;
  mocapid = -1;

  // clear object lists
  bodies.clear();
  geoms.clear();
  frames.clear();
  joints.clear();
  sites.clear();
  cameras.clear();
  lights.clear();
  spec_userdata_.clear();

  // in case this body is not compiled
  CopyFromSpec();

  // point to local (needs to be after defaults)
  PointToLocal();
}



mjCBody::mjCBody(const mjCBody& other, mjCModel* _model) {
  model = _model;
  mjSpec* origin = model->FindSpec(other.compiler);
  compiler = origin ? &origin->compiler : &model->spec.compiler;
  *this = other;
  CopyPlugin();
}



mjCBody& mjCBody::operator=(const mjCBody& other) {
  if (this != &other) {
    spec = other.spec;
    *static_cast<mjCBody_*>(this) = static_cast<const mjCBody_&>(other);
    *static_cast<mjsBody*>(this) = static_cast<const mjsBody&>(other);
    bodies.clear();
    frames.clear();
    geoms.clear();
    joints.clear();
    sites.clear();
    cameras.clear();
    lights.clear();
    id = -1;
    subtreedofs = 0;

    // add elements to lists
    *this += other;
  }
  PointToLocal();
  return *this;
}



// copy children of other body into body
mjCBody& mjCBody::operator+=(const mjCBody& other) {
  // map other frames to indices
  std::map<mjCFrame*, int> fmap;
  for (int i=0; i < other.frames.size(); i++) {
    fmap[other.frames[i]] = i + frames.size();
  }

  // copy frames, needs to happen first
  CopyList(frames, other.frames, fmap);

  // copy all children
  CopyList(geoms, other.geoms, fmap);
  CopyList(joints, other.joints, fmap);
  CopyList(sites, other.sites, fmap);
  CopyList(cameras, other.cameras, fmap);
  CopyList(lights, other.lights, fmap);

  for (int i=0; i < other.bodies.size(); i++) {
    bodies.push_back(new mjCBody(*other.bodies[i], model));  // triggers recursive call
    bodies.back()->parent = this;
    bodies.back()->frame = nullptr;
    if (other.bodies[i]->frame) {
      if (fmap.find(other.bodies[i]->frame) != fmap.end()) {
        bodies.back()->frame = frames[fmap[other.bodies[i]->frame]];
      } else {
        throw mjCError(this, "Frame '%s' not found in other body",
                       other.bodies[i]->frame->name.c_str());
      }
      if (bodies.back()->frame && bodies.back()->frame->body != this) {
        throw mjCError(this, "Frame and body '%s' have mismatched parents", name.c_str());
      }
    }
  }

  return *this;
}



// attach frame to body
mjCBody& mjCBody::operator+=(const mjCFrame& other) {
  // append a copy of the attached spec
  if (other.model != model && !model->FindSpec(&other.model->spec.compiler)) {
    model->AppendSpec(mj_copySpec(&other.model->spec), &other.model->spec.compiler);
  }

  // create a copy of the subtree that contains the frame
  mjCBody* subtree = other.body;
  other.model->prefix = other.prefix;
  other.model->suffix = other.suffix;
  other.model->StoreKeyframes(model);
  mjCModel* other_model = other.model;

  // attach defaults
  if (other_model != model) {
    mjCDef* subdef = new mjCDef(*other_model->Default());
    subdef->NameSpace(other_model);
    *model += *subdef;
  }

  // copy input frame
  mjSpec* origin = model->FindSpec(other.compiler);
  mjCFrame* newframe(model->deepcopy_ ? new mjCFrame(other) : (mjCFrame*)&other);
  frames.push_back(newframe);
  frames.back()->body = this;
  frames.back()->model = model;
  frames.back()->compiler = origin ? &origin->compiler : &model->spec.compiler;
  frames.back()->frame = other.frame;
  if (model->deepcopy_) {
    frames.back()->NameSpace(other_model);
  } else {
    frames.back()->AddRef();
  }
  int i = frames.size();
  last_attached = &frames.back()->spec;

  // map input frames to index in this->frames
  std::map<mjCFrame*, int> fmap;
  for (auto frame : subtree->frames) {
    if (frame == static_cast<const mjCFrame*>(&other)) {
      fmap[frame] = frames.size() - 1;
    } else if (other.IsAncestor(frame)) {
      fmap[frame] = i++;
    }
  }

  // copy children that are inside the input frame
  CopyList(frames, subtree->frames, fmap, &other);  // needs to be done first
  CopyList(geoms, subtree->geoms, fmap, &other);
  CopyList(joints, subtree->joints, fmap, &other);
  CopyList(sites, subtree->sites, fmap, &other);
  CopyList(cameras, subtree->cameras, fmap, &other);
  CopyList(lights, subtree->lights, fmap, &other);

  if (!model->deepcopy_) {
    std::string name = subtree->name;
    subtree->SetModel(model);
    subtree->NameSpace(other_model);
    subtree->name = name;
  }

  int nbodies = (int)subtree->bodies.size();
  for (int i=0; i < nbodies; i++) {
    if (!other.IsAncestor(subtree->bodies[i]->frame)) {
      continue;
    }
    if (model->deepcopy_) {
      mjCBody* newbody(new mjCBody(*subtree->bodies[i], model));  // triggers recursive call
      bodies.push_back(newbody);
      subtree->bodies[i]->ForgetKeyframes();
      bodies.back()->NameSpace_(other_model, /*propagate=*/ false);
    } else {
      bodies.push_back(subtree->bodies[i]);
      bodies.back()->SetModel(model);
      bodies.back()->ResetId();
      bodies.back()->AddRef();
    }
    bodies.back()->parent = this;
    bodies.back()->frame =
      subtree->bodies[i]->frame ? frames[fmap[subtree->bodies[i]->frame]] : nullptr;
  }

  // attach referencing elements
  other_model->SetAttached(model->deepcopy_);
  *model += *other_model;

  // leave the source model in a clean state
  if (other_model != model) {
    other_model->key_pending_.clear();
  }

  // clear namespace and return body
  other_model->prefix.clear();
  other_model->suffix.clear();
  return *this;
}



// copy src list of elements into dst; set body, model and frame
template <typename T>
void mjCBody::CopyList(std::vector<T*>& dst, const std::vector<T*>& src,
                       std::map<mjCFrame*, int>& fmap, const mjCFrame* pframe) {
  int nsrc = (int)src.size();
  int ndst = (int)dst.size();
  for (int i=0; i < nsrc; i++) {
    if (pframe && !pframe->IsAncestor(src[i]->frame)) {
      continue;  // skip if the element is not inside pframe
    }
    mjSpec* origin = model->FindSpec(src[i]->compiler);
    T* new_obj = model->deepcopy_ ? new T(*src[i]) : src[i];
    dst.push_back(new_obj);
    dst.back()->body = this;
    dst.back()->model = model;
    dst.back()->compiler = origin ? &origin->compiler : &model->spec.compiler;
    dst.back()->id = -1;
    dst.back()->CopyPlugin();
    dst.back()->classname = src[i]->classname;

    // increment refcount if shallow copy is made
    if (!model->deepcopy_) {
      dst.back()->AddRef();
    }

    // set namespace
    dst.back()->NameSpace(src[i]->model);
  }

  // assign dst frame to src frame
  // needs to be done after the copy in case T is an mjCFrame
  int j = 0;
  for (int i = 0; i < src.size(); i++) {
    if (pframe && !pframe->IsAncestor(src[i]->frame)) {
      continue;  // skip if the element is not inside pframe
    }
    dst[ndst + j++]->frame = src[i]->frame ? frames[fmap[src[i]->frame]] : nullptr;
  }
}



// find and remove subtree
mjCBody& mjCBody::operator-=(const mjCBody& subtree) {
  for (int i=0; i < bodies.size(); i++) {
    if (bodies[i] == &subtree) {
      bodies.erase(bodies.begin() + i);
      break;
    }
    *bodies[i] -= subtree;
  }

  return *this;
}



// set model of this body and its subtree
void mjCBody::SetModel(mjCModel* _model) {
  model = _model;
  mjSpec* origin = _model->FindSpec(compiler);
  compiler = origin ? &origin->compiler : &model->spec.compiler;

  for (auto& body : bodies) {
    body->SetModel(_model);
  }
  for (auto& frame : frames) {
    origin = _model->FindSpec(frame->compiler);
    frame->model = _model;
    frame->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& geom : geoms) {
    origin = _model->FindSpec(geom->compiler);
    geom->model = _model;
    geom->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& joint : joints) {
    origin = _model->FindSpec(joint->compiler);
    joint->model = _model;
    joint->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& site : sites) {
    origin = _model->FindSpec(site->compiler);
    site->model = _model;
    site->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& camera : cameras) {
    origin = _model->FindSpec(camera->compiler);
    camera->model = _model;
    camera->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& light : lights) {
    origin = _model->FindSpec(light->compiler);
    light->model = _model;
    light->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
}



// reset ids of all objects in this body
void mjCBody::ResetId() {
  id = -1;
  for (auto& body : bodies) {
    body->ResetId();
  }
  for (auto& frame : frames) {
    frame->id = -1;
  }
  for (auto& geom : geoms) {
    geom->id = -1;
  }
  for (auto& joint : joints) {
    joint->id = -1;
    joint->qposadr_ = -1;
    joint->dofadr_ = -1;
  }
  for (auto& site : sites) {
    site->id = -1;
  }
  for (auto& camera : cameras) {
    camera->id = -1;
  }
  for (auto& light : lights) {
    light->id = -1;
  }
}



void mjCBody::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.childclass = &classname;
  spec.userdata = &spec_userdata_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = (&plugin_instance_name);
  spec.info = &info;
  userdata = nullptr;
}


void mjCBody::CopyFromSpec() {
  *static_cast<mjsBody*>(this) = spec;
  userdata_ = spec_userdata_;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;
}



void mjCBody::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



// destructor
mjCBody::~mjCBody() {
  for (int i=0; i < bodies.size(); i++) bodies[i]->Release();
  for (int i=0; i < geoms.size(); i++) geoms[i]->Release();
  for (int i=0; i < frames.size(); i++) frames[i]->Release();
  for (int i=0; i < joints.size(); i++) joints[i]->Release();
  for (int i=0; i < sites.size(); i++) sites[i]->Release();
  for (int i=0; i < cameras.size(); i++) cameras[i]->Release();
  for (int i=0; i < lights.size(); i++) lights[i]->Release();
}



// apply prefix and suffix, propagate to children
void mjCBody::NameSpace(const mjCModel* m) {
  NameSpace_(m, true);
}



// apply prefix and suffix, propagate to all descendants or only to child bodies
void mjCBody::NameSpace_(const mjCModel* m, bool propagate) {
  mjCBase::NameSpace(m);
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }

  for (auto& body : bodies) {
    body->prefix = m->prefix;
    body->suffix = m->suffix;
    body->NameSpace_(m, propagate);
  }

  if (!propagate) {
    return;
  }

  for (auto& joint : joints) {
    joint->NameSpace(m);
  }

  for (auto& geom : geoms) {
    geom->NameSpace(m);
  }

  for (auto& site : sites) {
    site->NameSpace(m);
  }

  for (auto& camera : cameras) {
    camera->NameSpace(m);
  }

  for (auto& light : lights) {
    light->NameSpace(m);
  }

  for (auto& frame : frames) {
    frame->NameSpace(m);
  }
}



// create child body and add it to body
mjCBody* mjCBody::AddBody(mjCDef* _def) {
  // create body
  mjCBody* obj = new mjCBody(model);

  // handle def recursion (i.e. childclass)
  obj->classname = _def ? _def->name : classname;

  bodies.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();

  obj->parent = this;

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new frame and add it to body
mjCFrame* mjCBody::AddFrame(mjCFrame* _frame) {
  mjCFrame* obj = new mjCFrame(model, _frame ? _frame : NULL);
  frames.push_back(obj);
  model->ResetTreeLists();
  model->MakeTreeLists();

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new free joint (no default inheritance) and add it to body
mjCJoint* mjCBody::AddFreeJoint() {
  // create free joint, don't inherit from defaults
  mjCJoint* obj = new mjCJoint(model, NULL);
  obj->spec.type = mjJNT_FREE;

  // set body pointer, add
  obj->body = this;

  joints.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();


  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new joint and add it to body
mjCJoint* mjCBody::AddJoint(mjCDef* _def) {
  // create joint
  mjCJoint* obj = new mjCJoint(model, _def ? _def : model->def_map[classname]);

  // set body pointer, add
  obj->body = this;

  joints.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();


  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new geom and add it to body
mjCGeom* mjCBody::AddGeom(mjCDef* _def) {
  // create geom
  mjCGeom* obj = new mjCGeom(model, _def ? _def : model->def_map[classname]);

  //  set body pointer, add
  obj->body = this;

  geoms.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();


  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new site and add it to body
mjCSite* mjCBody::AddSite(mjCDef* _def) {
  // create site
  mjCSite* obj = new mjCSite(model, _def ? _def : model->def_map[classname]);

  // set body pointer, add
  obj->body = this;

  sites.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();


  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new camera and add it to body
mjCCamera* mjCBody::AddCamera(mjCDef* _def) {
  // create camera
  mjCCamera* obj = new mjCCamera(model, _def ? _def : model->def_map[classname]);

  // set body pointer, add
  obj->body = this;

  cameras.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();


  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new light and add it to body
mjCLight* mjCBody::AddLight(mjCDef* _def) {
  // create light
  mjCLight* obj = new mjCLight(model, _def ? _def : model->def_map[classname]);

  // set body pointer, add
  obj->body = this;

  lights.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();


  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create a frame in the parent body and move all contents of this body into it
mjCFrame* mjCBody::ToFrame() {
  mjCFrame* newframe = parent->AddFrame(frame);
  mjuu_copyvec(newframe->spec.pos, spec.pos, 3);
  mjuu_copyvec(newframe->spec.quat, spec.quat, 4);
  if (parent->name != "world" && mass >= mjMINVAL) {
    if (!parent->explicitinertial) {
      parent->MakeInertialExplicit();
      mjuu_zerovec(parent->spec.ipos, 3);
      mjuu_zerovec(parent->spec.iquat, 4);
      mjuu_zerovec(parent->spec.inertia, 3);
    }
    parent->AccumulateInertia(&this->spec, &parent->spec);
  }
  MapFrame(parent->bodies, bodies, newframe, parent);
  MapFrame(parent->geoms, geoms, newframe, parent);
  MapFrame(parent->joints, joints, newframe, parent);
  MapFrame(parent->sites, sites, newframe, parent);
  MapFrame(parent->cameras, cameras, newframe, parent);
  MapFrame(parent->lights, lights, newframe, parent);
  MapFrame(parent->frames, frames, newframe, parent);
  parent->bodies.erase(
      std::remove_if(parent->bodies.begin(), parent->bodies.end(),
                     [this](mjCBody* body) { return body == this; }),
      parent->bodies.end());
  model->ResetTreeLists();
  model->MakeTreeLists();
  model->spec.element->signature = model->Signature();
  return newframe;
}



// get number of objects of specified type
int mjCBody::NumObjects(mjtObj type) {
  switch (type) {
    case mjOBJ_BODY:
    case mjOBJ_XBODY:
      return (int)bodies.size();
    case mjOBJ_JOINT:
      return (int)joints.size();
    case mjOBJ_GEOM:
      return (int)geoms.size();
    case mjOBJ_SITE:
      return (int)sites.size();
    case mjOBJ_CAMERA:
      return (int)cameras.size();
    case mjOBJ_LIGHT:
      return (int)lights.size();
    default:
      return 0;
  }
}



// get poiner to specified object
mjCBase* mjCBody::GetObject(mjtObj type, int i) {
  if (i >= 0 && i < NumObjects(type)) {
    switch (type) {
      case mjOBJ_BODY:
      case mjOBJ_XBODY:
        return bodies[i];
      case mjOBJ_JOINT:
        return joints[i];
      case mjOBJ_GEOM:
        return geoms[i];
      case mjOBJ_SITE:
        return sites[i];
      case mjOBJ_CAMERA:
        return cameras[i];
      case mjOBJ_LIGHT:
        return lights[i];
      default:
        return 0;
    }
  }

  return 0;
}



// find object by name in given list
template <class T>
static T* findobject(std::string name, std::vector<T*>& list) {
  for (unsigned int i=0; i < list.size(); i++) {
    if (list[i]->name == name) {
      return list[i];
    }
  }

  return 0;
}



// recursive find by name
mjCBase* mjCBody::FindObject(mjtObj type, std::string _name, bool recursive) {
  mjCBase* res = 0;

  // check self: just in case
  if (name == _name) {
    return this;
  }

  // search elements of this body
  if (type == mjOBJ_BODY || type == mjOBJ_XBODY) {
    res = findobject(_name, bodies);
  } else if (type == mjOBJ_JOINT) {
    res = findobject(_name, joints);
  } else if (type == mjOBJ_GEOM) {
    res = findobject(_name, geoms);
  } else if (type == mjOBJ_SITE) {
    res = findobject(_name, sites);
  } else if (type == mjOBJ_CAMERA) {
    res = findobject(_name, cameras);
  } else if (type == mjOBJ_LIGHT) {
    res = findobject(_name, lights);
  }

  // found
  if (res) {
    return res;
  }

  // search children
  if (recursive) {
    for (int i=0; i < (int)bodies.size(); i++) {
      if ((res = bodies[i]->FindObject(type, _name, true))) {
        return res;
      }
    }
  }

  // not found
  return res;
}



template <class T>
mjsElement* mjCBody::GetNext(const std::vector<T*>& list, const mjsElement* child, bool* found) {
  if (list.empty()) {
    // no children
    return nullptr;
  }

  if (!child) {
    // first child
    return list[0]->spec.element;
  }

  for (unsigned int i = 0; i < list.size()-1; i++) {
    // next child is in this body
    if (list[i]->spec.element == child) {
      *found = true;
      return list[i+1]->spec.element;
    }
  }

  if (list.back()->spec.element == child) {
    // next child is in another body
    *found = true;
  }

  return nullptr;
}



// get next child of given type
mjsElement* mjCBody::NextChild(const mjsElement* child, mjtObj type, bool recursive, bool* found) {
  if (type == mjOBJ_UNKNOWN) {
    if (!child) {
      throw mjCError(this, "child type must be specified if no child element is given");
    } else {
      type = child->elemtype;
    }
  } else if (child && child->elemtype != type) {
    throw mjCError(this, "child element is not of requested type");
  }

  bool found_ = false;
  if (!found) {
    found = &found_;
  }

  mjsElement* candidate = nullptr;
  switch (type) {
    case mjOBJ_BODY:
    case mjOBJ_XBODY:
      candidate = GetNext(bodies, child, found);
      break;
    case mjOBJ_JOINT:
      candidate = GetNext(joints, child, found);
      break;
    case mjOBJ_GEOM:
      candidate = GetNext(geoms, child, found);
      break;
    case mjOBJ_SITE:
      candidate = GetNext(sites, child, found);
      break;
    case mjOBJ_CAMERA:
      candidate = GetNext(cameras, child, found);
      break;
    case mjOBJ_LIGHT:
      candidate = GetNext(lights, child, found);
      break;
    case mjOBJ_FRAME:
      candidate = GetNext(frames, child, found);
      break;
    default:
      throw mjCError(this,
                     "Body.NextChild supports the types: body, frame, geom, "
                     "site, light, camera");
      break;
  }

  if (!candidate && recursive) {
    for (int i=0; i < (int)bodies.size(); i++) {
      candidate = bodies[i]->NextChild(*found ? nullptr : child, type, recursive, found);
      if (candidate) {
        return candidate;
      }
    }
  }

  return candidate;
}



// compute geom inertial frame: ipos, iquat, mass, inertia
void mjCBody::InertiaFromGeom(void) {
  int sz;
  double com[3] = {0, 0, 0};
  double toti[6] = {0, 0, 0, 0, 0, 0};
  std::vector<mjCGeom*> sel;

  // select geoms based on group
  sel.clear();
  for (int i=0; i < geoms.size(); i++) {
    if (geoms[i]->group >= compiler->inertiagrouprange[0] &&
        geoms[i]->group <= compiler->inertiagrouprange[1]) {
      sel.push_back(geoms[i]);
    }
  }
  sz = sel.size();

  // single geom: copy
  if (sz == 1) {
    mjuu_copyvec(ipos, sel[0]->pos, 3);
    mjuu_copyvec(iquat, sel[0]->quat, 4);
    mass = sel[0]->mass_;
    mjuu_copyvec(inertia, sel[0]->inertia, 3);
  }

  // multiple geoms
  else if (sz > 1) {
    // compute total mass and center of mass
    mass = 0;
    for (int i=0; i < sz; i++) {
      mass += sel[i]->mass_;
      com[0] += sel[i]->mass_ * sel[i]->pos[0];
      com[1] += sel[i]->mass_ * sel[i]->pos[1];
      com[2] += sel[i]->mass_ * sel[i]->pos[2];
    }

    // check for small mass
    if (mass < mjEPS) {
      throw mjCError(this, "body mass is too small, cannot compute center of mass");
    }

    // ipos = geom com
    ipos[0] = com[0]/mass;
    ipos[1] = com[1]/mass;
    ipos[2] = com[2]/mass;

    // add geom inertias
    for (int i=0; i < sz; i++) {
      double inert0[6], inert1[6];
      double dpos[3] = {
        sel[i]->pos[0] - ipos[0],
        sel[i]->pos[1] - ipos[1],
        sel[i]->pos[2] - ipos[2]
      };

      mjuu_globalinertia(inert0, sel[i]->inertia, sel[i]->quat);
      mjuu_offcenter(inert1, sel[i]->mass_, dpos);
      for (int j=0; j < 6; j++) {
        toti[j] = toti[j] + inert0[j] + inert1[j];
      }
    }

    // compute principal axes of inertia
    mjuu_copyvec(fullinertia, toti, 6);
    const char* errq = mjuu_fullInertia(iquat, inertia, fullinertia);
    if (errq) {
      throw mjCError(this, "error '%s' in alternative for principal axes", errq);
    }
  }
}



// set explicitinertial to true
void mjCBody::MakeInertialExplicit() {
  spec.explicitinertial = true;
}



// accumulate inertia of another body into this body
void mjCBody::AccumulateInertia(const mjsBody* other, mjsBody* result) {
  if (!result) {
    result = this;  // use the private mjsBody
  }

  // body_ipose = body_pose * body_ipose
  double other_ipos[3];
  double other_iquat[4];
  mjuu_copyvec(other_ipos, other->ipos, 3);
  mjuu_copyvec(other_iquat, other->iquat, 4);
  mjuu_frameaccum(other_ipos, other_iquat, other->pos, other->quat);

  // organize data
  double mass[2] = {
    result->mass,
    other->mass
  };
  double inertia[2][3] = {
    {result->inertia[0], result->inertia[1], result->inertia[2]},
    {other->inertia[0], other->inertia[1], other->inertia[2]}
  };
  double ipos[2][3] = {
    {result->ipos[0], result->ipos[1], result->ipos[2]},
    {other_ipos[0], other_ipos[1], other_ipos[2]}
  };
  double iquat[2][4] = {
    {result->iquat[0], result->iquat[1], result->iquat[2], result->iquat[3]},
    {other->iquat[0], other->iquat[1], other->iquat[2], other->iquat[3]}
  };

  // compute total mass
  result->mass = 0;
  mjuu_setvec(result->ipos, 0, 0, 0);
  for (int j=0; j < 2; j++) {
    result->mass += mass[j];
    result->ipos[0] += mass[j]*ipos[j][0];
    result->ipos[1] += mass[j]*ipos[j][1];
    result->ipos[2] += mass[j]*ipos[j][2];
  }

  // small mass: allow for now, check for errors later
  if (result->mass < mjMINVAL) {
    result->mass = 0;
    mjuu_setvec(result->inertia, 0, 0, 0);
    mjuu_setvec(result->ipos, 0, 0, 0);
    mjuu_setvec(result->iquat, 1, 0, 0, 0);
  }

  // proceed with regular computation
  else {
    // locipos = center-of-mass
    result->ipos[0] /= result->mass;
    result->ipos[1] /= result->mass;
    result->ipos[2] /= result->mass;

    // add inertias
    double toti[6] = {0, 0, 0, 0, 0, 0};
    for (int j=0; j < 2; j++) {
      double inertA[6], inertB[6];
      double dpos[3] = {
        ipos[j][0] - result->ipos[0],
        ipos[j][1] - result->ipos[1],
        ipos[j][2] - result->ipos[2]
      };

      mjuu_globalinertia(inertA, inertia[j], iquat[j]);
      mjuu_offcenter(inertB, mass[j], dpos);
      for (int k=0; k < 6; k++) {
        toti[k] += inertA[k] + inertB[k];
      }
    }

    // compute principal axes of inertia
    mjuu_copyvec(result->fullinertia, toti, 6);
    const char* err1 = mjuu_fullInertia(result->iquat, result->inertia, result->fullinertia);
    if (err1) {
      throw mjCError(nullptr, "error '%s' in fusing static body inertias", err1);
    }
  }
}



// compute bounding volume hierarchy
void mjCBody::ComputeBVH() {
  if (geoms.empty()) {
    return;
  }

  tree.Set(ipos, iquat);
  tree.AllocateBoundingVolumes(geoms.size());
  for (const mjCGeom* geom : geoms) {
    tree.AddBoundingVolume(&geom->id, geom->contype, geom->conaffinity,
                           geom->pos, geom->quat, geom->aabb);
  }
  tree.CreateBVH();
}



// reset keyframe references for allowing self-attach
void mjCBody::ForgetKeyframes() const {
  for (auto joint : joints) {
    joint->qpos_.clear();
    joint->qvel_.clear();
  }
  ((mjCBody*)this)->mpos_.clear();
  ((mjCBody*)this)->mquat_.clear();
  for (auto body : bodies) {
    body->ForgetKeyframes();
  }
}



mjtNum* mjCBody::mpos(const std::string& state_name) {
  if (mpos_.find(state_name) == mpos_.end()) {
    mpos_[state_name] = {mjNAN, 0, 0};
  }
  return mpos_.at(state_name).data();
}



mjtNum* mjCBody::mquat(const std::string& state_name) {
  if (mquat_.find(state_name) == mquat_.end()) {
    mquat_[state_name] = {mjNAN, 0, 0, 0};
  }
  return mquat_.at(state_name).data();
}



// compiler
void mjCBody::Compile(void) {
  CopyFromSpec();

  // compile all frames
  for (int i=0; i < frames.size(); i++) {
    frames[i]->Compile();
  }

  // resize userdata
  if (userdata_.size() > model->nuser_body) {
    throw mjCError(this, "user has more values than nuser_body in body");
  }
  userdata_.resize(model->nuser_body);

  // normalize user-defined quaternions
  mjuu_normvec(quat, 4);
  mjuu_normvec(iquat, 4);

  // set parentid and weldid of children
  for (int i=0; i < bodies.size(); i++) {
    bodies[i]->weldid = (!bodies[i]->joints.empty() ? bodies[i]->id : weldid);
  }

  // check and process orientation alternatives for body
  if (alt.type != mjORIENTATION_QUAT) {
    const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
    if (err) {
      throw mjCError(this, "error '%s' in frame alternative", err);
    }
  }

  // check orientation alternatives for inertia
  if (mjuu_defined(fullinertia[0]) && ialt.type != mjORIENTATION_QUAT) {
    throw mjCError(this, "fullinertia and inertial orientation cannot both be specified");
  }
  if (mjuu_defined(fullinertia[0]) && (inertia[0] || inertia[1] || inertia[2])) {
    throw mjCError(this, "fullinertia and diagonal inertia cannot both be specified");
  }

  // process orientation alternatives for inertia
  if (mjuu_defined(fullinertia[0])) {
    const char* err = mjuu_fullInertia(iquat, inertia, this->fullinertia);
    if (err) {
      throw mjCError(this, "error '%s' in fullinertia", err);
    }
  }

  if (ialt.type != mjORIENTATION_QUAT) {
    const char* err = ResolveOrientation(iquat, compiler->degree, compiler->eulerseq, ialt);
    if (err) {
      throw mjCError(this, "error '%s' in inertia alternative", err);
    }
  }

  // compile all geoms
  for (int i=0; i < geoms.size(); i++) {
    geoms[i]->inferinertia = id > 0 &&
      (!explicitinertial || compiler->inertiafromgeom == mjINERTIAFROMGEOM_TRUE) &&
      geoms[i]->spec.group >= compiler->inertiagrouprange[0] &&
      geoms[i]->spec.group <= compiler->inertiagrouprange[1];
    geoms[i]->Compile();
  }

  // set inertial frame from geoms if necessary
  if (id > 0 && (compiler->inertiafromgeom == mjINERTIAFROMGEOM_TRUE ||
                 (!mjuu_defined(ipos[0]) && compiler->inertiafromgeom == mjINERTIAFROMGEOM_AUTO))) {
    InertiaFromGeom();
  }

  // ipos undefined: copy body frame into inertial
  if (!mjuu_defined(ipos[0])) {
    mjuu_copyvec(ipos, pos, 3);
    mjuu_copyvec(iquat, quat, 4);
  }

  // check and correct mass and inertia
  if (id > 0) {
    // fix minimum
    mass = std::max(mass, compiler->boundmass);
    inertia[0] = std::max(inertia[0], compiler->boundinertia);
    inertia[1] = std::max(inertia[1], compiler->boundinertia);
    inertia[2] = std::max(inertia[2], compiler->boundinertia);

    // check for negative values
    if (mass < 0 || inertia[0] < 0 || inertia[1] < 0 ||inertia[2] < 0) {
      throw mjCError(this, "mass and inertia cannot be negative");
    }

    // check for non-physical inertia
    if (inertia[0] + inertia[1] < inertia[2] ||
        inertia[0] + inertia[2] < inertia[1] ||
        inertia[1] + inertia[2] < inertia[0]) {
      if (compiler->balanceinertia) {
        inertia[0] = inertia[1] = inertia[2] = (inertia[0] + inertia[1] + inertia[2])/3.0;
      } else {
        throw mjCError(this, "inertia must satisfy A + B >= C; use 'balanceinertia' to fix");
      }
    }
  }

  // frame
  if (frame) {
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // accumulate rbound, contype, conaffinity over geoms
  contype = conaffinity = 0;
  margin = 0;
  for (int i=0; i < geoms.size(); i++) {
    contype |= geoms[i]->contype;
    conaffinity |= geoms[i]->conaffinity;
    margin = std::max(margin, geoms[i]->margin);
  }

  // check conditions for free-joint alignment
  bool align_free = (joints.size() == 1                  &&  // only one joint AND
                     joints[0]->spec.type == mjJNT_FREE  &&  // it is a free joint AND
                     bodies.empty()                      &&  // no child bodies AND
                     (joints[0]->spec.align == 1 ||          // either joint.align="true"
                      (joints[0]->spec.align == 2        &&  // or joint.align="auto"
                       compiler->alignfree)));                  //  and compiler->align="true"

  // free-joint alignment, phase 1 (this body + child geoms)
  double ipos_inverse[3], iquat_inverse[4];
  if (align_free) {
    // accumulate iframe transformation to body frame
    mjuu_frameaccum(pos, quat, ipos, iquat);

    // compute inverse iframe transformation
    mjuu_frameinvert(ipos_inverse, iquat_inverse, ipos, iquat);

    // save iframe, set it to null
    mjuu_setvec(ipos, 0, 0, 0);
    mjuu_setvec(iquat, 1, 0, 0, 0);

    // apply inverse iframe transformation to all child geoms
    for (int i=0; i < geoms.size(); i++) {
      mjuu_frameaccumChild(ipos_inverse, iquat_inverse, geoms[i]->pos, geoms[i]->quat);
    }
  }

  // compute bounding volume hierarchy
  ComputeBVH();

  // compile all joints, count dofs
  dofnum = 0;
  for (int i=0; i < joints.size(); i++) {
    dofnum += joints[i]->Compile();
  }

  // check for excessive number of dofs
  if (dofnum > 6) {
    throw mjCError(this, "more than 6 dofs in body '%s'", name.c_str());
  }

  // check for rotation dof after ball joint
  bool hasball = false;
  for (int i=0; i < joints.size(); i++) {
    if ((joints[i]->type == mjJNT_BALL || joints[i]->type == mjJNT_HINGE) && hasball) {
      throw mjCError(this, "ball followed by rotation in body '%s'", name.c_str());
    }
    if (joints[i]->type == mjJNT_BALL) {
      hasball = true;
    }
  }

  // make sure mocap body is fixed child of world
  if (mocap && (dofnum || (parent && parent->name != "world"))) {
    throw mjCError(this, "mocap body '%s' is not a fixed child of world", name.c_str());
  }

  // compute body global pose (no joint transformations in qpos0)
  if (id > 0) {
    mjuu_rotVecQuat(xpos0, pos, parent->xquat0);
    mjuu_addtovec(xpos0, parent->xpos0, 3);
    mjuu_mulquat(xquat0, parent->xquat0, quat);
  }

  // compile all sites
  for (int i=0; i < sites.size(); i++)sites[i]->Compile();

  // compile all cameras
  for (int i=0; i < cameras.size(); i++)cameras[i]->Compile();

  // compile all lights
  for (int i=0; i < lights.size(); i++)lights[i]->Compile();

  // plugin
  if (plugin.active) {
    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw mjCError(
              this, "neither 'plugin' nor 'instance' is specified for body '%s', (id = %d)",
              name.c_str(), id);
    }

    mjCPlugin* plugin_instance = static_cast<mjCPlugin*>(plugin.element);
    model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
    plugin.element = plugin_instance;
    const mjpPlugin* pplugin = mjp_getPluginAtSlot(plugin_instance->plugin_slot);
    if (!(pplugin->capabilityflags & mjPLUGIN_PASSIVE)) {
      throw mjCError(this, "plugin '%s' does not support passive forces", pplugin->name);
    }
  }

  // free joint alignment, phase 2 (transform sites, cameras and lights)
  if (align_free) {
    // frames have already been compiled and applied to children

    // sites
    for (int i=0; i < sites.size(); i++) {
      mjuu_frameaccumChild(ipos_inverse, iquat_inverse, sites[i]->pos, sites[i]->quat);
    }

    // cameras
    for (int i=0; i < cameras.size(); i++) {
      mjuu_frameaccumChild(ipos_inverse, iquat_inverse, cameras[i]->pos, cameras[i]->quat);
    }

    // lights
    for (int i=0; i < lights.size(); i++) {
      double qunit[4]= {1, 0, 0, 0};
      mjuu_frameaccumChild(ipos_inverse, iquat_inverse, lights[i]->pos, qunit);
      mjuu_rotVecQuat(lights[i]->dir, lights[i]->dir, iquat_inverse);
    }
  }
}



//------------------ class mjCFrame implementation -------------------------------------------------

// initialize frame
mjCFrame::mjCFrame(mjCModel* _model, mjCFrame* _frame) {
  mjs_defaultFrame(&spec);
  elemtype = mjOBJ_FRAME;
  compiled = false;
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  body = NULL;
  frame = _frame ? _frame : NULL;
  last_attached = nullptr;
  PointToLocal();
  CopyFromSpec();
}



mjCFrame::mjCFrame(const mjCFrame& other) {
  *this = other;
}



mjCFrame& mjCFrame::operator=(const mjCFrame& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCFrame_*>(this) = static_cast<const mjCFrame_&>(other);
    *static_cast<mjsFrame*>(this) = static_cast<const mjsFrame&>(other);
  }
  PointToLocal();
  return *this;
}



// attach body to frame
mjCFrame& mjCFrame::operator+=(const mjCBody& other) {
  // append a copy of the attached spec
  if (other.model != model && !model->FindSpec(&other.model->spec.compiler)) {
    model->AppendSpec(mj_copySpec(&other.model->spec), &other.model->spec.compiler);
  }

  // apply namespace and store keyframes in the source model
  other.model->prefix = other.prefix;
  other.model->suffix = other.suffix;
  other.model->StoreKeyframes(model);
  other.model->prefix = "";
  other.model->suffix = "";
  mjCModel* other_model = other.model;

  // attach or copy the subtree
  mjCBody* subtree = model->deepcopy_ ? new mjCBody(other, model) : (mjCBody*)&other;
  if (model->deepcopy_) {
    other.ForgetKeyframes();
  } else {
    subtree->SetModel(model);
    subtree->ResetId();
    subtree->AddRef();
  }
  other_model->prefix = subtree->prefix;
  other_model->suffix = subtree->suffix;
  subtree->SetParent(body);
  subtree->SetFrame(this);
  subtree->NameSpace(other_model);

  // attach defaults
  if (other_model != model) {
    mjCDef* subdef = new mjCDef(*other_model->Default());
    subdef->NameSpace(other_model);
    *model += *subdef;
  }

  // add to body children
  body->bodies.push_back(subtree);
  last_attached = &body->bodies.back()->spec;

  // attach referencing elements
  other_model->SetAttached(model->deepcopy_);
  *model += *other_model;

  // leave the source model in a clean state
  if (other_model != model) {
    other_model->key_pending_.clear();
  }

  // clear suffixes and return
  other_model->suffix.clear();
  other_model->prefix.clear();
  return *this;
}



// return true if child is descendent of this frame
bool mjCFrame::IsAncestor(const mjCFrame* child) const {
  if (!child) {
    return false;
  }

  if (child == this) {
    return true;
  }

  return IsAncestor(child->frame);
}



void mjCFrame::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.childclass = &classname;
  spec.info = &info;
}



void mjCFrame::CopyFromSpec() {
  *static_cast<mjsFrame*>(this) = spec;
  mjuu_copyvec(pos, spec.pos, 3);
  mjuu_copyvec(quat, spec.quat, 4);
}



void mjCFrame::Compile() {
  if (compiled) {
    return;
  }

  CopyFromSpec();
  const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
  if (err) {
    throw mjCError(this, "orientation specification error '%s' in site %d", err, id);
  }

  // compile parents and accumulate result
  if (frame) {
    frame->Compile();
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  mjuu_normvec(quat, 4);
  compiled = true;
}



//------------------ class mjCJoint implementation -------------------------------------------------

// initialize default joint
mjCJoint::mjCJoint(mjCModel* _model, mjCDef* _def) {
  mjs_defaultJoint(&spec);
  elemtype = mjOBJ_JOINT;

  // clear internal variables
  spec_userdata_.clear();
  body = 0;

  // reset to default if given
  if (_def) {
    *this = _def->Joint();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this joint is not compiled
  CopyFromSpec();

  // no previous state when a joint is created
  qposadr_ = -1;
  dofadr_ = -1;
}



mjCJoint::mjCJoint(const mjCJoint& other) {
  *this = other;
}



mjCJoint& mjCJoint::operator=(const mjCJoint& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCJoint_*>(this) = static_cast<const mjCJoint_&>(other);
    *static_cast<mjsJoint*>(this) = static_cast<const mjsJoint&>(other);
    qposadr_ = -1;
    dofadr_ = -1;
  }
  PointToLocal();
  return *this;
}



bool mjCJoint::is_limited() const {
  return islimited(limited, range);
}
bool mjCJoint::is_actfrclimited() const {
  return islimited(actfrclimited, actfrcrange);
}



int mjCJoint::nq(mjtJoint joint_type) {
  switch (joint_type) {
    case mjJNT_FREE:
      return 7;
    case mjJNT_BALL:
      return 4;
    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      return 1;
  }
  return 1;
}



int mjCJoint::nv(mjtJoint joint_type) {
  switch (joint_type) {
    case mjJNT_FREE:
      return 6;
    case mjJNT_BALL:
      return 3;
    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      return 1;
  }
  return 1;
}



mjtNum* mjCJoint::qpos(const std::string& state_name) {
  if (qpos_.find(state_name) == qpos_.end()) {
    qpos_[state_name] = {mjNAN, 0, 0, 0, 0, 0, 0};
  }
  return qpos_.at(state_name).data();
}



mjtNum* mjCJoint::qvel(const std::string& state_name) {
  if (qvel_.find(state_name) == qvel_.end()) {
    qvel_[state_name] = {mjNAN, 0, 0, 0, 0, 0};
  }
  return qvel_.at(state_name).data();
}



void mjCJoint::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.userdata = &spec_userdata_;
  spec.info = &info;
  userdata = nullptr;
}



void mjCJoint::CopyFromSpec() {
  *static_cast<mjsJoint*>(this) = spec;
  userdata_ = spec_userdata_;
}



// compiler
int mjCJoint::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_jnt) {
    throw mjCError(this, "user has more values than nuser_jnt in joint");
  }
  userdata_.resize(model->nuser_jnt);

  // check springdamper
  if (springdamper[0] || springdamper[1]) {
    if (springdamper[0] <= 0 || springdamper[1] <= 0) {
      throw mjCError(this, "when defined, springdamper values must be positive in joint");
    }
  }

  // free joints cannot be limited
  if (type == mjJNT_FREE) {
    limited = mjLIMITED_FALSE;
  }
  // otherwise if limited is auto, check consistency wrt auto-limits
  else if (limited == mjLIMITED_AUTO) {
    bool hasrange = !(range[0] == 0 && range[1] == 0);
    checklimited(this, compiler->autolimits, "joint", "", limited, hasrange);
  }

  // resolve limits
  if (is_limited()) {
    // check data
    if (range[0] >= range[1] && type != mjJNT_BALL) {
      throw mjCError(this, "range[0] should be smaller than range[1] in joint");
    }
    if (range[0] && type == mjJNT_BALL) {
      throw mjCError(this, "range[0] should be 0 in ball joint");
    }

    // convert limits to radians
    if (compiler->degree && (type == mjJNT_HINGE || type == mjJNT_BALL)) {
      if (range[0]) {
        range[0] *= mjPI/180.0;
      }
      if (range[1]) {
        range[1] *= mjPI/180.0;
      }
    }
  }

  // actuator force range: none for free or ball joints
  if (type == mjJNT_FREE || type == mjJNT_BALL) {
    actfrclimited = mjLIMITED_FALSE;
  }
  // otherwise if actfrclimited is auto, check consistency wrt auto-limits
  else if (actfrclimited == mjLIMITED_AUTO) {
    bool hasrange = !(actfrcrange[0] == 0 && actfrcrange[1] == 0);
    checklimited(this, compiler->autolimits, "joint", "", actfrclimited, hasrange);
  }

  // resolve actuator force range limits
  if (is_actfrclimited()) {
    // check data
    if (actfrcrange[0] >= actfrcrange[1]) {
      throw mjCError(this, "actfrcrange[0] should be smaller than actfrcrange[1] in joint");
    }
  }

  // axis: FREE or BALL are fixed to (0,0,1)
  if (type == mjJNT_FREE || type == mjJNT_BALL) {
    axis[0] = axis[1] = 0;
    axis[2] = 1;
  }

  // otherwise accumulate frame rotation
  else if (frame) {
    mjuu_rotVecQuat(axis, axis, frame->quat);
  }

  // normalize axis, check norm
  if (mjuu_normvec(axis, 3) < mjEPS) {
    throw mjCError(this, "axis too small in joint");
  }

  // check data
  if (type == mjJNT_FREE && limited == mjLIMITED_TRUE) {
    throw mjCError(this, "limits should not be defined in free joint");
  }

  // pos: FREE is fixed to (0,0,0)
  if (type == mjJNT_FREE) {
    mjuu_zerovec(pos, 3);
  }

  // otherwise accumulate frame translation
  else if (frame) {
    double qunit[4] = {1, 0, 0, 0};
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, qunit);
  }

  // convert reference angles to radians for hinge joints
  if (type == mjJNT_HINGE && compiler->degree) {
    ref *= mjPI/180.0;
    springref *= mjPI/180.0;
  }

  // return dofnum
  if (type == mjJNT_FREE) {
    return 6;
  } else if (type == mjJNT_BALL) {
    return 3;
  } else {
    return 1;
  }
}



//------------------ class mjCGeom implementation --------------------------------------------------

// initialize default geom
mjCGeom::mjCGeom(mjCModel* _model, mjCDef* _def) {
  mjs_defaultGeom(&spec);
  elemtype = mjOBJ_GEOM;

  mass_ = 0;
  body = 0;
  matid = -1;
  mesh = nullptr;
  hfield = nullptr;
  visual_ = false;
  mjuu_setvec(inertia, 0, 0, 0);
  inferinertia = true;
  spec_material_.clear();
  spec_userdata_.clear();
  spec_meshname_.clear();
  spec_hfieldname_.clear();
  spec_userdata_.clear();

  for (int i = 0; i < mjNFLUID; i++){
    fluid[i] = 0;
  }

  // reset to default if given
  if (_def) {
    *this = _def->Geom();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this geom is not compiled
  CopyFromSpec();
}



mjCGeom::mjCGeom(const mjCGeom& other) {
  *this = other;
}



mjCGeom& mjCGeom::operator=(const mjCGeom& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCGeom_*>(this) = static_cast<const mjCGeom_&>(other);
    *static_cast<mjsGeom*>(this) = static_cast<const mjsGeom&>(other);
  }
  PointToLocal();
  return *this;
}



// to be called after any default copy constructor
void mjCGeom::PointToLocal(void) {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.info = &info;
  spec.userdata = &spec_userdata_;
  spec.material = &spec_material_;
  spec.meshname = &spec_meshname_;
  spec.hfieldname = &spec_hfieldname_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = &plugin_instance_name;
  userdata = nullptr;
  hfieldname = nullptr;
  meshname = nullptr;
  material = nullptr;
}



void mjCGeom::CopyFromSpec() {
  *static_cast<mjsGeom*>(this) = spec;
  userdata_ = spec_userdata_;
  hfieldname_ = spec_hfieldname_;
  meshname_ = spec_meshname_;
  material_ = spec_material_;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;
}



void mjCGeom::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



void mjCGeom::NameSpace(const mjCModel* m) {
  mjCBase::NameSpace(m);
  if (!spec_material_.empty() && model != m) {
    spec_material_ = m->prefix + spec_material_ + m->suffix;
  }
  if (!spec_hfieldname_.empty() && model != m) {
    spec_hfieldname_ = m->prefix + spec_hfieldname_ + m->suffix;
  }
  if (!spec_meshname_.empty() && model != m) {
    spec_meshname_ = m->prefix + spec_meshname_ + m->suffix;
  }
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }
}



// compute geom volume / surface area
double mjCGeom::GetVolume() const {
  // get from mesh
  if (type == mjGEOM_MESH || type == mjGEOM_SDF) {
    if (mesh->id < 0 || !((std::size_t) mesh->id <= model->Meshes().size())) {
      throw mjCError(this, "invalid mesh id in mesh geom");
    }

    return mesh->GetVolumeRef();
  }

  // compute from geom shape (type) and inertia type (typeinertia)
  switch (type) {
    case mjGEOM_SPHERE: {
      double radius = size[0];
      switch (typeinertia) {
        case mjINERTIA_VOLUME:
          return 4 * mjPI * radius * radius * radius / 3;
        case mjINERTIA_SHELL:
          return 4 * mjPI * radius * radius;
      }
      break;
    }
    case mjGEOM_CAPSULE: {
      double height = 2 * size[1];
      double radius = size[0];
      switch (typeinertia) {
        case mjINERTIA_VOLUME:
          return mjPI * (radius * radius * height + 4 * radius * radius * radius / 3);
        case mjINERTIA_SHELL:
          return 4 * mjPI * radius * radius + 2 * mjPI * radius * height;
      }
      break;
    }
    case mjGEOM_CYLINDER: {
      double height = 2 * size[1];
      double radius = size[0];
      switch (typeinertia) {
        case mjINERTIA_VOLUME:
          return mjPI * radius * radius * height;
        case mjINERTIA_SHELL:
          return 2 * mjPI * radius * radius + 2 * mjPI * radius * height;
      }
      break;
    }
    case mjGEOM_ELLIPSOID: {
      switch (typeinertia) {
        case mjINERTIA_VOLUME:
          return 4 * mjPI * size[0] * size[1] * size[2] / 3;
        case mjINERTIA_SHELL: {
          // Thomsen approximation
          // https://www.numericana.com/answer/ellipsoid.htm#thomsen
          double p = 1.6075;
          double tmp = std::pow(size[0] * size[1], p) +
                       std::pow(size[1] * size[2], p) +
                       std::pow(size[2] * size[0], p);
          return 4 * mjPI * std::pow(tmp / 3, 1 / p);
        }
      }
      break;
    }
    case mjGEOM_HFIELD:
    case mjGEOM_BOX: {
      switch (typeinertia) {
        case mjINERTIA_VOLUME:
          return size[0] * size[1] * size[2] * 8;
        case mjINERTIA_SHELL:
          return 8 * (size[0] * size[1] + size[1] * size[2] + size[2] * size[0]);
      }
      break;
    }
    default:
      break;
  }
  return 0;
}



// set geom diagonal inertia given density
void mjCGeom::SetInertia(void) {
  // get from mesh
  if (type == mjGEOM_MESH || type == mjGEOM_SDF) {
    if (mesh->id < 0 || !((std::size_t)mesh->id <= model->Meshes().size())) {
      throw mjCError(this, "invalid mesh id in mesh geom");
    }

    double* boxsz = mesh->GetInertiaBoxPtr();
    inertia[0] = mass_ * (boxsz[1] * boxsz[1] + boxsz[2] * boxsz[2]) / 3;
    inertia[1] = mass_ * (boxsz[0] * boxsz[0] + boxsz[2] * boxsz[2]) / 3;
    inertia[2] = mass_ * (boxsz[0] * boxsz[0] + boxsz[1] * boxsz[1]) / 3;

    return;
  }

  // compute from geom shape (type) and inertia type (typeinertia)
  switch (type) {
    case mjGEOM_SPHERE: {
      switch (typeinertia) {
        case mjINERTIA_VOLUME:
          inertia[0] = inertia[1] = inertia[2] = 2 * mass_ * size[0] * size[0] / 5;
          return;
        case mjINERTIA_SHELL:
          inertia[0] = inertia[1] = inertia[2] = 2 * mass_ * size[0] * size[0] / 3;
          return;
      }
      break;
    }
    case mjGEOM_CAPSULE: {
      double halfheight = size[1];
      double height = 2 * size[1];
      double radius = size[0];
      switch (typeinertia) {
        case mjINERTIA_VOLUME: {
          double sphere_mass =
            mass_ * 4 * radius / (4 * radius + 3 * height);    // mass*(sphere_vol/total_vol)
          double cylinder_mass = mass_ - sphere_mass;

          // cylinder part
          inertia[0] = inertia[1] = cylinder_mass * (3 * radius * radius + height * height) / 12;
          inertia[2] = cylinder_mass * radius * radius / 2;

          // add two hemispheres, displace along third axis
          double sphere_inertia = 2 * sphere_mass * radius * radius / 5;
          inertia[0] += sphere_inertia + sphere_mass * height * (3 * radius + 2 * height) / 8;
          inertia[1] += sphere_inertia + sphere_mass * height * (3 * radius + 2 * height) / 8;
          inertia[2] += sphere_inertia;
          return;
        }
        case mjINERTIA_SHELL: {
          // surface area
          double Asphere = 4 * mjPI * radius * radius;
          double Acylinder = 2 * mjPI * radius * height;
          double Atotal = Asphere + Acylinder;

          // mass
          double sphere_mass = mass_ * Asphere / Atotal;  // mass*(sphere_area/total_area)
          double cylinder_mass = mass_ - sphere_mass;

          // cylinder part
          inertia[0] = inertia[1] = cylinder_mass * (6 * radius * radius + height * height) / 12;
          inertia[2] = cylinder_mass * radius * radius;

          // add two hemispheres, displace along third axis
          double sphere_inertia = 2 * sphere_mass * radius * radius / 3;
          double hs_com = radius / 2;           // hemisphere center of mass
          double hs_pos = halfheight + hs_com;  // hemisphere position
          inertia[0] += sphere_inertia + sphere_mass * (hs_pos * hs_pos - hs_com * hs_com);
          inertia[1] += sphere_inertia + sphere_mass * (hs_pos * hs_pos - hs_com * hs_com);
          inertia[2] += sphere_inertia;
          return;
        }
        break;
      }
      break;
    }
    case mjGEOM_CYLINDER: {
      double halfheight = size[1];
      double height = 2 * halfheight;
      double radius = size[0];
      switch (typeinertia) {
        case mjINERTIA_VOLUME:

          inertia[0] = inertia[1] = mass_ * (3 * radius * radius + height * height) / 12;
          inertia[2] = mass_ * radius * radius / 2;
          return;
        case mjINERTIA_SHELL: {
          // surface area
          double Adisk = mjPI * radius * radius;
          double Acylinder = 2 * mjPI * radius * height;
          double Atotal = 2 * Adisk + Acylinder;

          // mass
          double mass_disk = mass_ * Adisk / Atotal;
          double mass_cylinder = mass_ - 2 * mass_disk;

          // cylinder contribution
          inertia[0] = inertia[1] = mass_cylinder * (6 * radius * radius + height * height) / 12;
          inertia[2] = mass_cylinder * radius * radius;

          // disk inertia
          double inertia_disk_x = mass_disk * radius * radius / 4 +
                                  mass_disk * halfheight * halfheight;
          double inertia_disk_z = mass_disk * radius * radius / 2;

          // top and bottom disk contributions
          inertia[0] += 2 * inertia_disk_x;
          inertia[1] += 2 * inertia_disk_x;
          inertia[2] += 2 * inertia_disk_z;
          return;
        }
      }
      break;
    }
    case mjGEOM_ELLIPSOID: {
      double s00 = size[0] * size[0];
      double s11 = size[1] * size[1];
      double s22 = size[2] * size[2];
      switch (typeinertia) {
        case mjINERTIA_VOLUME: {
          inertia[0] = mass_ * (s11 + s22) / 5;
          inertia[1] = mass_ * (s00 + s22) / 5;
          inertia[2] = mass_ * (s00 + s11) / 5;
          return;
        }
        case mjINERTIA_SHELL: {
          // approximate shell inertia by subtracting ellipsoid from expanded ellipsoid
          double eps = 1e-6;

          // solid volume (a)
          double Va = 4 * mjPI * size[0] * size[1] * size[2] / 3;

          // expanded volume (b)
          double ae = size[0] + eps;
          double be = size[1] + eps;
          double ce = size[2] + eps;
          double Vb = 4 * mjPI * ae * be * ce / 3;

          // density
          double density = mass_ / (Vb - Va);

          // inertia
          double mass_a = Va * density;
          double inertia_a[3];
          inertia_a[0] = mass_a * (s11 + s22) / 5;
          inertia_a[1] = mass_a * (s00 + s22) / 5;
          inertia_a[2] = mass_a * (s00 + s11) / 5;

          double mass_b = Vb * density;
          double inertia_b[3];
          inertia_b[0] = mass_b * (be * be + ce * ce) / 5;
          inertia_b[1] = mass_b * (ae * ae + ce * ce) / 5;
          inertia_b[2] = mass_b * (ae * ae + be * be) / 5;

          // shell inertia
          inertia[0] = inertia_b[0] - inertia_a[0];
          inertia[1] = inertia_b[1] - inertia_a[1];
          inertia[2] = inertia_b[2] - inertia_a[2];
          return;
        }
      }
      break;
    }
    case mjGEOM_HFIELD:
    case mjGEOM_BOX: {
      double s00 = size[0] * size[0];
      double s11 = size[1] * size[1];
      double s22 = size[2] * size[2];
      switch (typeinertia) {
        case mjINERTIA_VOLUME: {
          inertia[0] = mass_ * (s11 + s22) / 3;
          inertia[1] = mass_ * (s00 + s22) / 3;
          inertia[2] = mass_ * (s00 + s11) / 3;
          return;
        }
        case mjINERTIA_SHELL: {
          // length
          double lx = 2 * size[0];  // side 0
          double ly = 2 * size[1];  // side 1
          double lz = 2 * size[2];  // side 2

          // surface area
          double A0 = lx * ly;  // side 0
          double A1 = ly * lz;  // side 1
          double A2 = lz * lx;  // side 2
          double Atotal = 2 * (A0 + A1 + A2);

          // side 0
          double mass0 = mass_ * A0 / Atotal;
          double Ix0 = mass0 * ly * ly / 12;
          double Iy0 = mass0 * lx * lx / 12;
          double Iz0 = mass0 * (lx * lx + ly * ly) / 12;

          // side 1
          double mass1 = mass_ * A1 / Atotal;
          double Ix1 = mass1 * (ly * ly + lz * lz) / 12;
          double Iy1 = mass1 * lz * lz / 12;
          double Iz1 = mass1 * ly * ly / 12;

          // side 3
          double mass2 = mass_ * A2 / Atotal;
          double Ix2 = mass2 * lz * lz / 12;
          double Iy2 = mass2 * (lx * lx + lz * lz) / 12;
          double Iz2 = mass2 * lx * lx / 12;

          // total inertia
          inertia[0] = 2 * (mass0 * s22 + mass2 * s11 + Ix0 + Ix1 + Ix2);
          inertia[1] = 2 * (mass0 * s22 + mass1 * s00 + Iy0 + Iy1 + Iy2);
          inertia[2] = 2 * (mass1 * s00 + mass2 * s11 + Iz0 + Iz1 + Iz2);
          return;
        }
        break;
      }
      break;
    }
    default:
      inertia[0] = inertia[1] = inertia[2] = 0;
      return;
  }
}



// compute radius of bounding sphere
double mjCGeom::GetRBound(void) {
  const double *aamm, *hsize;
  double haabb[3] = {0};

  switch (type) {
    case mjGEOM_HFIELD:
      hsize = hfield->size;
      return sqrt(hsize[0]*hsize[0] + hsize[1]*hsize[1] +
                  std::max(hsize[2]*hsize[2], hsize[3]*hsize[3]));

    case mjGEOM_SPHERE:
      return size[0];

    case mjGEOM_CAPSULE:
      return size[0]+size[1];

    case mjGEOM_CYLINDER:
      return sqrt(size[0]*size[0]+size[1]*size[1]);

    case mjGEOM_ELLIPSOID:
      return std::max(std::max(size[0], size[1]), size[2]);

    case mjGEOM_BOX:
      return sqrt(size[0]*size[0]+size[1]*size[1]+size[2]*size[2]);

    case mjGEOM_MESH:
    case mjGEOM_SDF:
      aamm = mesh->aamm();
      haabb[0] = std::max(std::abs(aamm[0]), std::abs(aamm[3]));
      haabb[1] = std::max(std::abs(aamm[1]), std::abs(aamm[4]));
      haabb[2] = std::max(std::abs(aamm[2]), std::abs(aamm[5]));
      return sqrt(haabb[0]*haabb[0] + haabb[1]*haabb[1] + haabb[2]*haabb[2]);

    default:
      return 0;
  }
}



// Compute the coefficients of the added inertia due to the surrounding fluid.
double mjCGeom::GetAddedMassKappa(double dx, double dy, double dz) {
  // Integration by Gauss–Kronrod quadrature on interval l in [0, infinity] of
  // f(l) = dx*dy*dz / np.sqrt((dx*dx+ l)**3 * (dy*dy+ l) * (dz*dz+ l))
  // 15-point Gauss–Kronrod quadrature (K15) points x in [0, 1].

  // static constexpr mjtNum kronrod_x[15] = [     // unused, left in comment for completeness
  //   0.00427231, 0.02544604, 0.06756779, 0.12923441, 0.20695638,
  //   0.29707742, 0.39610752, 0.50000000, 0.60389248, 0.70292258,
  //   0.79304362, 0.87076559, 0.93243221, 0.97455396, 0.99572769];
  // 15-point Gauss–Kronrod quadrature (K15) weights.
  static constexpr double kronrod_w[15] = {
    0.01146766, 0.03154605, 0.05239501, 0.07032663, 0.08450236,
    0.09517529, 0.10221647, 0.10474107, 0.10221647, 0.09517529,
    0.08450236, 0.07032663, 0.05239501, 0.03154605, 0.01146766};
  // Integrate from 0 to inf by change of variables:
  // l = x^3 / (1-x)^2. Exponents 3 and 2 found to minimize error.
  static constexpr double kronrod_l[15] = {
    7.865151709349917e-08, 1.7347976913907274e-05, 0.0003548008144506193,
    0.002846636252924549, 0.014094260903596077, 0.053063261727396636,
    0.17041978741317773, 0.5, 1.4036301548686991, 3.9353484827022642,
    11.644841677041734, 39.53187807410903, 177.5711362220801,
    1429.4772912937397, 54087.416549217705};
  // dl = dl/dx dx. The following are dl/dx(x).
  static constexpr double kronrod_d[15] = {
    5.538677720489877e-05, 0.002080868285293228, 0.016514126520723166,
    0.07261900344370877, 0.23985243401862602, 0.6868318249020725,
    1.8551129519182894, 5.0, 14.060031152313941, 43.28941239611009,
    156.58546376397112, 747.9826085305024, 5827.4042950027115,
    116754.0197944512, 25482945.327264845};

  const double invdx2 = 1.0 / (dx * dx);
  const double invdy2 = 1.0 / (dy * dy);
  const double invdz2 = 1.0 / (dz * dz);

  // for added numerical stability we non-dimensionalize x by scale
  // because 1 + l/d^2 in denom, l should be scaled by d^2
  const double scale = std::pow(dx*dx*dx * dy * dz, 0.4);  // ** (2/5)
  double kappa = 0.0;
  for (int i = 0; i < 15; ++i) {
    const double lambda = scale * kronrod_l[i];
    const double denom = (1 + lambda*invdx2) * std::sqrt(
      (1 + lambda*invdx2) * (1 + lambda*invdy2) * (1 + lambda*invdz2));
    kappa += scale * kronrod_d[i] / denom * kronrod_w[i];
  }
  return kappa * invdx2;
}



// Compute the kappa coefs of the added inertia due to the surrounding fluid.
void mjCGeom::SetFluidCoefs(void) {
  double dx, dy, dz;

  // get semiaxes
  switch (type) {
    case mjGEOM_SPHERE:
      dx = size[0];
      dy = size[0];
      dz = size[0];
      break;

    case mjGEOM_CAPSULE:
      dx = size[0];
      dy = size[0];
      dz = size[1] + size[0];
      break;

    case mjGEOM_CYLINDER:
      dx = size[0];
      dy = size[0];
      dz = size[1];
      break;

    default:
      dx = size[0];
      dy = size[1];
      dz = size[2];
  }

  // volume of equivalent ellipsoid
  const double volume = 4.0 / 3.0 * mjPI * dx * dy * dz;

  // GetAddedMassKappa is invariant to permutation of last two arguments
  const double kx = GetAddedMassKappa(dx, dy, dz);
  const double ky = GetAddedMassKappa(dy, dz, dx);
  const double kz = GetAddedMassKappa(dz, dx, dy);

  // coefficients of virtual moment of inertia. Note: if (kz-ky) in numerator
  // is negative, also the denom is negative. Abs both and clip to MINVAL
  const auto pow2 = [](const double val) { return val * val; };
  const double Ixfac = pow2(dy*dy - dz*dz) * std::abs(kz - ky) / std::max(
    mjEPS, std::abs(2*(dy*dy - dz*dz) + (dy*dy + dz*dz)*(ky - kz)));
  const double Iyfac = pow2(dz*dz - dx*dx) * std::abs(kx - kz) / std::max(
    mjEPS, std::abs(2*(dz*dz - dx*dx) + (dz*dz + dx*dx)*(kz - kx)));
  const double Izfac = pow2(dx*dx - dy*dy) * std::abs(ky - kx) / std::max(
    mjEPS, std::abs(2*(dx*dx - dy*dy) + (dx*dx + dy*dy)*(kx - ky)));

  mjtNum virtual_mass[3];
  virtual_mass[0] = volume * kx / std::max(mjEPS, 2-kx);
  virtual_mass[1] = volume * ky / std::max(mjEPS, 2-ky);
  virtual_mass[2] = volume * kz / std::max(mjEPS, 2-kz);
  mjtNum virtual_inertia[3];
  virtual_inertia[0] = volume*Ixfac/5;
  virtual_inertia[1] = volume*Iyfac/5;
  virtual_inertia[2] = volume*Izfac/5;

  writeFluidGeomInteraction(fluid, &fluid_ellipsoid, &fluid_coefs[0],
                            &fluid_coefs[1], &fluid_coefs[2],
                            &fluid_coefs[3], &fluid_coefs[4],
                            virtual_mass, virtual_inertia);
}


// compute bounding box
void mjCGeom::ComputeAABB(void) {
  double aamm[6]; // axis-aligned bounding box in (min, max) format
  switch (type) {
    case mjGEOM_HFIELD:
      aamm[0] = -hfield->size[0];
      aamm[1] = -hfield->size[1];
      aamm[2] = -hfield->size[3];
      aamm[3] = hfield->size[0];
      aamm[4] = hfield->size[1];
      aamm[5] = hfield->size[2];
      break;

    case mjGEOM_SPHERE:
      aamm[3] = aamm[4] = aamm[5] = size[0];
      mjuu_setvec(aamm, -aamm[3], -aamm[4], -aamm[5]);
      break;

    case mjGEOM_CAPSULE:
      aamm[3] = aamm[4] = size[0];
      aamm[5] = size[0] + size[1];
      mjuu_setvec(aamm, -aamm[3], -aamm[4], -aamm[5]);
      break;

    case mjGEOM_CYLINDER:
      aamm[3] = aamm[4] = size[0];
      aamm[5] = size[1];
      mjuu_setvec(aamm, -aamm[3], -aamm[4], -aamm[5]);
      break;

    case mjGEOM_MESH:
    case mjGEOM_SDF:
      mjuu_copyvec(aamm, mesh->aamm(), 6);
      break;

    case mjGEOM_PLANE:
      aamm[0] = aamm[1] = aamm[2] = -mjMAXVAL;
      aamm[3] = aamm[4] = mjMAXVAL;
      aamm[5] = 0;
      break;

    default:
      mjuu_copyvec(aamm+3, size, 3);
      mjuu_setvec(aamm, -size[0], -size[1], -size[2]);
      break;
  }

  // convert aamm to aabb (center, size) format
  double pos[] = {(aamm[3] + aamm[0]) / 2, (aamm[4] + aamm[1]) / 2,
                  (aamm[5] + aamm[2]) / 2};
  double size[] = {(aamm[3] - aamm[0]) / 2, (aamm[4] - aamm[1]) / 2,
                   (aamm[5] - aamm[2]) / 2};
  mjuu_copyvec(aabb, pos, 3);
  mjuu_copyvec(aabb+3, size, 3);
}



// compiler
void mjCGeom::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_geom) {
    throw mjCError(this, "user has more values than nuser_geom in geom '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_geom);

  // check type
  if (type < 0 || type >= mjNGEOMTYPES) {
    throw mjCError(this, "invalid type in geom");
  }

  // check condim
  if (condim != 1 && condim != 3 && condim != 4 && condim != 6) {
    throw mjCError(this, "invalid condim in geom");
  }

  // check mesh
  if ((type == mjGEOM_MESH || type == mjGEOM_SDF) && !mesh) {
    throw mjCError(this, "mesh geom '%s' (id = %d) must have valid meshid", name.c_str(), id);
  }

  // check hfield
  if ((type == mjGEOM_HFIELD && !hfield) || (type != mjGEOM_HFIELD && hfield)) {
    throw mjCError(this, "hfield geom '%s' (id = %d) must have valid hfieldid", name.c_str(), id);
  }

  // plane only allowed in static bodies
  if (type == mjGEOM_PLANE && body->weldid != 0) {
    throw mjCError(this, "plane only allowed in static bodies");
  }

  // check if can collide
  visual_ = !contype && !conaffinity;

  // normalize quaternion
  mjuu_normvec(quat, 4);

  // 'fromto': compute pos, quat, size
  if (mjuu_defined(fromto[0])) {
    // check type
    if (type != mjGEOM_CAPSULE &&
        type != mjGEOM_CYLINDER &&
        type != mjGEOM_ELLIPSOID &&
        type != mjGEOM_BOX) {
      throw mjCError(this, "fromto requires capsule, cylinder, box or ellipsoid in geom");
    }

    // make sure pos is not defined; cannot use mjuu_defined because default is (0,0,0)
    if (pos[0] || pos[1] || pos[2]) {
      throw mjCError(this, "both pos and fromto defined in geom");
    }

    // size[1] = length (for capsule and cylinder)
    double vec[3] = {
      fromto[0]-fromto[3],
      fromto[1]-fromto[4],
      fromto[2]-fromto[5]
    };
    size[1] = mjuu_normvec(vec, 3)/2;
    if (size[1] < mjEPS) {
      throw mjCError(this, "fromto points too close in geom");
    }

    // adjust size for ellipsoid and box
    if (type == mjGEOM_ELLIPSOID || type == mjGEOM_BOX) {
      size[2] = size[1];
      size[1] = size[0];
    }

    // compute position
    pos[0] = (fromto[0]+fromto[3])/2;
    pos[1] = (fromto[1]+fromto[4])/2;
    pos[2] = (fromto[2]+fromto[5])/2;

    // compute orientation
    mjuu_z2quat(quat, vec);
  }

  // not 'fromto': try alternative
  else {
    const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
    if (err) {
      throw mjCError(this, "orientation specification error '%s' in geom %d", err, id);
    }
  }

  // mesh: accumulate frame, fit geom if needed
  if (mesh) {
    // check for inapplicable fromto
    if (mjuu_defined(fromto[0])) {
      throw mjCError(this, "fromto cannot be used with mesh geom");
    }

    // save reference in case this is not an mjGEOM_MESH
    mjCMesh* pmesh = mesh;

    // fit geom if type is not mjGEOM_MESH
    if (type != mjGEOM_MESH && type != mjGEOM_SDF) {
      double meshpos[3];
      mesh->FitGeom(this, meshpos);

      // remove reference to mesh
      meshname_.clear();
      mesh = nullptr;
      mjuu_copyvec(pmesh->GetPosPtr(), meshpos, 3);
    } else if (typeinertia == mjINERTIA_SHELL) {
      throw mjCError(this, "for mesh geoms, inertia should be specified in the mesh asset");
    }

    // apply geom pos/quat as offset
    mjuu_frameaccum(pos, quat, pmesh->GetPosPtr(), pmesh->GetQuatPtr());
  }

  // check size parameters
  checksize(size, type, this, name.c_str(), id);

  // set hfield sizes in geom.size
  if (type == mjGEOM_HFIELD) {
    size[0] = hfield->size[0];
    size[1] = hfield->size[1];
    size[2] = 0.25 * hfield->size[2] + 0.5 * hfield->size[3];
  } else if (type == mjGEOM_MESH || type == mjGEOM_SDF) {
    const double* aamm = mesh->aamm();
    size[0] = std::max(std::abs(aamm[0]), std::abs(aamm[3]));
    size[1] = std::max(std::abs(aamm[1]), std::abs(aamm[4]));
    size[2] = std::max(std::abs(aamm[2]), std::abs(aamm[5]));
  }

  for (double s : size) {
    if (std::isnan(s)) {
      throw mjCError(this, "nan size in geom");
    }
  }
  // compute aabb
  ComputeAABB();

  // compute geom mass and inertia
  if (inferinertia) {
    // mass is defined
    if (mjuu_defined(mass)) {
      if (mass == 0) {
        mass_ = 0;
        density = 0;
      } else if (GetVolume() > mjEPS) {
        mass_ = mass;
        density = mass / GetVolume();
        SetInertia();
      }
    }

    // mass is not defined
    else {
      if (density == 0) {
        mass_ = 0;
      } else {
        mass_ = density * GetVolume();
        SetInertia();
      }
    }


    // check for negative values
    if (mass_ < 0 || inertia[0] < 0 || inertia[1] < 0 || inertia[2] < 0 || density < 0)
      throw mjCError(this, "mass, inertia or density are negative in geom");
  }

  // fluid-interaction coefficients, requires computed inertia and mass
  if (fluid_ellipsoid > 0) {
    SetFluidCoefs();
  }

  // plugin
  if (plugin.active) {
    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw mjCError(
              this, "neither 'plugin' nor 'instance' is specified for geom");
    }

    mjCPlugin* plugin_instance = static_cast<mjCPlugin*>(plugin.element);
    model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
    plugin.element = plugin_instance;
    const mjpPlugin* pplugin = mjp_getPluginAtSlot(plugin_instance->plugin_slot);
    if (!(pplugin->capabilityflags & mjPLUGIN_SDF)) {
      throw mjCError(this, "plugin '%s' does not support sign distance fields", pplugin->name);
    }
  }

  // frame
  if (frame) {
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }
}



//------------------ class mjCSite implementation --------------------------------------------------

// initialize default site
mjCSite::mjCSite(mjCModel* _model, mjCDef* _def) {
  mjs_defaultSite(&spec);
  elemtype = mjOBJ_SITE;

  // clear internal variables
  body = 0;
  matid = -1;
  spec_material_.clear();
  spec_userdata_.clear();

  // reset to default if given
  if (_def) {
    *this = _def->Site();
  }

  // point to local
  PointToLocal();

  // in case this site is not compiled
  CopyFromSpec();

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";
}



mjCSite::mjCSite(const mjCSite& other) {
  *this = other;
}



mjCSite& mjCSite::operator=(const mjCSite& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCSite_*>(this) = static_cast<const mjCSite_&>(other);
    *static_cast<mjsSite*>(this) = static_cast<const mjsSite&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCSite::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.info = &info;
  spec.material = &spec_material_;
  spec.userdata = &spec_userdata_;
  userdata = nullptr;
  material = nullptr;
}



void mjCSite::CopyFromSpec() {
  *static_cast<mjsSite*>(this) = spec;
  userdata_ = spec_userdata_;
  material_ = spec_material_;
}



void mjCSite::NameSpace(const mjCModel* m) {
  mjCBase::NameSpace(m);
  if (!spec_material_.empty() && model != m) {
    spec_material_ = m->prefix + spec_material_ + m->suffix;
  }
}



// compiler
void mjCSite::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_site) {
    throw mjCError(this, "user has more values than nuser_site in site");
  }
  userdata_.resize(model->nuser_site);

  // check type
  if (type < 0 || type >= mjNGEOMTYPES) {
    throw mjCError(this, "invalid type in site");
  }

  // do not allow meshes, hfields and planes
  if (type == mjGEOM_MESH || type == mjGEOM_HFIELD || type == mjGEOM_PLANE) {
    throw mjCError(this, "meshes, hfields and planes not allowed in site");
  }

  // 'fromto': compute pos, quat, size
  if (mjuu_defined(fromto[0])) {
    // check type
    if (type != mjGEOM_CAPSULE &&
        type != mjGEOM_CYLINDER &&
        type != mjGEOM_ELLIPSOID &&
        type != mjGEOM_BOX) {
      throw mjCError(this, "fromto requires capsule, cylinder, box or ellipsoid in geom");
    }

    // make sure pos is not defined; cannot use mjuu_defined because default is (0,0,0)
    if (pos[0] || pos[1] || pos[2]) {
      throw mjCError(this, "both pos and fromto defined in geom");
    }

    // size[1] = length (for capsule and cylinder)
    double vec[3] = {
      fromto[0]-fromto[3],
      fromto[1]-fromto[4],
      fromto[2]-fromto[5]
    };
    size[1] = mjuu_normvec(vec, 3)/2;
    if (size[1] < mjEPS) {
      throw mjCError(this, "fromto points too close in geom");
    }

    // adjust size for ellipsoid and box
    if (type == mjGEOM_ELLIPSOID || type == mjGEOM_BOX) {
      size[2] = size[1];
      size[1] = size[0];
    }

    // compute position
    pos[0] = (fromto[0]+fromto[3])/2;
    pos[1] = (fromto[1]+fromto[4])/2;
    pos[2] = (fromto[2]+fromto[5])/2;

    // compute orientation
    mjuu_z2quat(quat, vec);
  }

  // alternative orientation
  else {
    const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
    if (err) {
      throw mjCError(this, "orientation specification error '%s' in site %d", err, id);
    }
  }

  // frame
  if (frame) {
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // normalize quaternion
  mjuu_normvec(quat, 4);

  // check size parameters
  checksize(size, type, this, name.c_str(), id);
}



//------------------ class mjCCamera implementation ------------------------------------------------

// initialize defaults
mjCCamera::mjCCamera(mjCModel* _model, mjCDef* _def) {
  mjs_defaultCamera(&spec);
  elemtype = mjOBJ_CAMERA;

  // clear private variables
  body = 0;
  targetbodyid = -1;
  spec_targetbody_.clear();

  // reset to default if given
  if (_def) {
    *this = _def->Camera();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCCamera::mjCCamera(const mjCCamera& other) {
  *this = other;
}



mjCCamera& mjCCamera::operator=(const mjCCamera& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCCamera_*>(this) = static_cast<const mjCCamera_&>(other);
    *static_cast<mjsCamera*>(this) = static_cast<const mjsCamera&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCCamera::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.userdata = &spec_userdata_;
  spec.targetbody = &spec_targetbody_;
  spec.info = &info;
  userdata = nullptr;
  targetbody = nullptr;
}



void mjCCamera::NameSpace(const mjCModel* m) {
  mjCBase::NameSpace(m);
  if (!spec_targetbody_.empty()) {
    spec_targetbody_ = m->prefix + spec_targetbody_ + m->suffix;
  }
}



void mjCCamera::CopyFromSpec() {
  *static_cast<mjsCamera*>(this) = spec;
  userdata_ = spec_userdata_;
  targetbody_ = spec_targetbody_;
}



void mjCCamera::ResolveReferences(const mjCModel* m) {
  if (!targetbody_.empty()) {
    mjCBody* tb = (mjCBody*)m->FindObject(mjOBJ_BODY, targetbody_);
    if (tb) {
      targetbodyid = tb->id;
    } else {
      throw mjCError(this, "unknown target body in camera");
    }
  }
}



// compiler
void mjCCamera::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_cam) {
    throw mjCError(this, "user has more values than nuser_cam in camera");
  }
  userdata_.resize(model->nuser_cam);

  // process orientation specifications
  const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
  if (err) {
    throw mjCError(this, "orientation specification error '%s' in camera %d", err, id);
  }

  // frame
  if (frame) {
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // normalize quaternion
  mjuu_normvec(quat, 4);

  // get targetbodyid
  ResolveReferences(model);

  // make sure the image size is finite
  if (fovy >= 180) {
    throw mjCError(this, "fovy too large in camera '%s' (id = %d, value = %d)",
                   name.c_str(), id, fovy);
  }

  // check that specs are not duplicated
  if ((principal_length[0] && principal_pixel[0]) ||
      (principal_length[1] && principal_pixel[1])) {
    throw mjCError(this, "principal length duplicated in camera");
  }

  if ((focal_length[0] && focal_pixel[0]) ||
      (focal_length[1] && focal_pixel[1])) {
    throw mjCError(this, "focal length duplicated in camera");
  }

  // compute number of pixels per unit length
  if (sensor_size[0] > 0 && sensor_size[1] > 0) {
    float pixel_density[2] = {
      (float)resolution[0] / sensor_size[0],
      (float)resolution[1] / sensor_size[1],
    };

    // defaults are zero, so only one term in each sum is nonzero
    intrinsic[0] = focal_pixel[0] / pixel_density[0] + focal_length[0];
    intrinsic[1] = focal_pixel[1] / pixel_density[1] + focal_length[1];
    intrinsic[2] = principal_pixel[0] / pixel_density[0] + principal_length[0];
    intrinsic[3] = principal_pixel[1] / pixel_density[1] + principal_length[1];

    // fovy with principal point at (0, 0)
    fovy = std::atan2(sensor_size[1]/2, intrinsic[1]) * 360.0 / mjPI;
  } else {
    intrinsic[0] = model->visual.map.znear;
    intrinsic[1] = model->visual.map.znear;
  }
}



//------------------ class mjCLight implementation -------------------------------------------------

// initialize defaults
mjCLight::mjCLight(mjCModel* _model, mjCDef* _def) {
  mjs_defaultLight(&spec);
  elemtype = mjOBJ_LIGHT;

  // clear private variables
  body = 0;
  targetbodyid = -1;
  texid = -1;
  spec_targetbody_.clear();
  spec_texture_.clear();


  // reset to default if given
  if (_def) {
    *this = _def->Light();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  PointToLocal();
  CopyFromSpec();
}



mjCLight::mjCLight(const mjCLight& other) {
  *this = other;
}



mjCLight& mjCLight::operator=(const mjCLight& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCLight_*>(this) = static_cast<const mjCLight_&>(other);
    *static_cast<mjsLight*>(this) = static_cast<const mjsLight&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCLight::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.targetbody = &spec_targetbody_;
  spec.texture = &spec_texture_;
  spec.info = &info;
  targetbody = nullptr;
}



void mjCLight::NameSpace(const mjCModel* m) {
  mjCBase::NameSpace(m);
  if (!spec_targetbody_.empty()) {
    spec_targetbody_ = m->prefix + spec_targetbody_ + m->suffix;
  }
  if (!spec_texture_.empty()) {
    spec_texture_ = m->prefix + spec_texture_ + m->suffix;
  }
}



void mjCLight::CopyFromSpec() {
  *static_cast<mjsLight*>(this) = spec;
  targetbody_ = spec_targetbody_;
  texture_ = spec_texture_;
}



void mjCLight::ResolveReferences(const mjCModel* m) {
  if (!targetbody_.empty()) {
    mjCBody* tb = (mjCBody*)m->FindObject(mjOBJ_BODY, targetbody_);
    if (tb) {
      targetbodyid = tb->id;
    } else {
      throw mjCError(this, "unknown target body in light");
    }
  }
  if (!texture_.empty()) {
    mjCTexture* tex = (mjCTexture*)m->FindObject(mjOBJ_TEXTURE, texture_);
    if (tex) {
      texid = tex->id;
    } else {
      throw mjCError(this, "unknown texture in light");
    }
  }
}



// compiler
void mjCLight::Compile(void) {
  CopyFromSpec();

  // frame
  if (frame) {
    // apply frame transform to pos, qunit is unused
    double qunit[4]= {1, 0, 0, 0};
    mjuu_frameaccumChild(frame->pos, frame->quat, pos, qunit);

    // rotate dir
    mjuu_rotVecQuat(dir, dir, frame->quat);
  }

  // normalize direction, make sure it is not zero
  if (mjuu_normvec(dir, 3) < mjEPS) {
    throw mjCError(this, "zero direction in light");
  }

  // get targetbodyid and texid
  ResolveReferences(model);
}



//------------------------- class mjCHField --------------------------------------------------------

// constructor
mjCHField::mjCHField(mjCModel* _model) {
  mjs_defaultHField(&spec);
  elemtype = mjOBJ_HFIELD;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  data.clear();
  spec_file_.clear();
  spec_userdata_.clear();

  // point to local
  PointToLocal();

  // copy from spec
  CopyFromSpec();
}



mjCHField::mjCHField(const mjCHField& other) {
  *this = other;
}



mjCHField& mjCHField::operator=(const mjCHField& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCHField_*>(this) = static_cast<const mjCHField_&>(other);
    *static_cast<mjsHField*>(this) = static_cast<const mjsHField&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCHField::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.file = &spec_file_;
  spec.content_type = &spec_content_type_;
  spec.userdata = &spec_userdata_;
  spec.info = &info;
  file = nullptr;
  content_type = nullptr;
  userdata = nullptr;
}



void mjCHField::CopyFromSpec() {
  *static_cast<mjsHField*>(this) = spec;
  file_ = spec_file_;
  content_type_ = spec_content_type_;
  userdata_ = spec_userdata_;

  // clear precompiled asset. TODO: use asset cache
  data.clear();
  if (!file_.empty()) {
    nrow = 0;
    ncol = 0;
  }

  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = mjuu_strippath(file_);
    name = mjuu_stripext(stripped);
  }
}



void mjCHField::NameSpace(const mjCModel* m) {
  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = mjuu_strippath(spec_file_);
    name = mjuu_stripext(stripped);
  }
  mjCBase::NameSpace(m);
  if (modelfiledir_.empty()) {
    modelfiledir_ = FilePath(m->spec_modelfiledir_);
  }
  if (meshdir_.empty()) {
    meshdir_ = FilePath(m->spec_meshdir_);
  }
}



// destructor
mjCHField::~mjCHField() {
  data.clear();
  userdata_.clear();
  spec_userdata_.clear();
}



// load elevation data from custom format
void mjCHField::LoadCustom(mjResource* resource) {
  // get file data in buffer
  const void* buffer = 0;
  int buffer_sz = mju_readResource(resource, &buffer);

  if (buffer_sz < 1) {
    throw mjCError(this, "could not read hfield file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw mjCError(this, "empty hfield file '%s'", resource->name);
  }


  if (buffer_sz < 2*sizeof(int)) {
    throw mjCError(this, "hfield missing header '%s'", resource->name);
  }

  // read dimensions
  int* pint = (int*)buffer;
  nrow = pint[0];
  ncol = pint[1];

  // check dimensions
  if (nrow < 1 || ncol < 1) {
    throw mjCError(this, "non-positive hfield dimensions in file '%s'", resource->name);
  }

  // check buffer size
  if (buffer_sz != nrow*ncol*sizeof(float)+8) {
    throw mjCError(this, "unexpected file size in file '%s'", resource->name);
  }

  // allocate
  data.assign(nrow*ncol, 0);
  if (data.empty()) {
    throw mjCError(this, "could not allocate buffers in hfield");
  }

  // copy data
  memcpy(data.data(), (void*)(pint+2), nrow*ncol*sizeof(float));
}



// load elevation data from PNG format
void mjCHField::LoadPNG(mjResource* resource) {
  PNGImage image = PNGImage::Load(this, resource, LCT_GREY);

  ncol = image.Width();
  nrow = image.Height();

  // copy image data over with rows reversed
  data.reserve(nrow * ncol);
  for (int r = 0; r < nrow; r++) {
    for (int c = 0; c < ncol; c++) {
      data.push_back((float) image[c + (nrow - 1 - r)*ncol]);
    }
  }
}



// compiler
void mjCHField::Compile(const mjVFS* vfs) {
  CopyFromSpec();

  // copy userdata into data
  if (!userdata_.empty()) {
    if (nrow*ncol != userdata_.size()) {
      throw mjCError(this, "elevation data length must match nrow*ncol");
    }
    data.assign(nrow*ncol, 0);
    if (data.empty()) {
      throw mjCError(this, "could not allocate buffers in hfield");
    }
    memcpy(data.data(), userdata_.data(), nrow*ncol*sizeof(float));
  }

  // check size parameters
  for (int i=0; i < 4; i++)
    if (size[i] <= 0)
      throw mjCError(this, "size parameter is not positive in hfield");

  // remove path from file if necessary
  if (model->strippath) {
    file_ = mjuu_strippath(file_);
  }

  // load from file if specified
  if (!file_.empty()) {
    // make sure hfield was not already specified manually
    if (nrow || ncol || !data.empty()) {
      throw mjCError(this, "hfield specified from file and manually");
    }

    std::string asset_type = GetAssetContentType(file_, content_type_);

    // fallback to custom
    if (asset_type.empty()) {
      asset_type = "image/vnd.mujoco.hfield";
    }

    if (asset_type != "image/png" && asset_type != "image/vnd.mujoco.hfield") {
      throw mjCError(this, "unsupported content type: '%s'", asset_type.c_str());
    }

    // copy paths from model if not already defined
    if (modelfiledir_.empty()) {
      modelfiledir_ = FilePath(model->modelfiledir_);
    }
    if (meshdir_.empty()) {
      meshdir_ = FilePath(model->meshdir_);
    }

    FilePath filename = meshdir_ + FilePath(file_);
    mjResource* resource = LoadResource(modelfiledir_.Str(), filename.Str(), vfs);

    try {
      if (asset_type == "image/png") {
        LoadPNG(resource);
      } else {
        LoadCustom(resource);
      }
      mju_closeResource(resource);
    } catch(mjCError err) {
      mju_closeResource(resource);
      throw err;
    }
  }

  // make sure hfield was specified (from file or manually)
  if (nrow < 1 || ncol < 1 || data.empty()) {
    throw mjCError(this, "hfield not specified");
  }

  // set elevation data to [0-1] range
  float emin = 1E+10, emax = -1E+10;
  for (int i = 0; i < nrow*ncol; i++) {
    emin = std::min(emin, data[i]);
    emax = std::max(emax, data[i]);
  }
  if (emin > emax) {
    throw mjCError(this, "invalid data range in hfield '%s'", file_.c_str());
  }
  for (int i=0; i < nrow*ncol; i++) {
    data[i] -= emin;
    if (emax-emin > mjEPS) {
      data[i] /= (emax - emin);
    }
  }
}



//------------------ class mjCTexture implementation -----------------------------------------------

// initialize defaults
mjCTexture::mjCTexture(mjCModel* _model) {
  mjs_defaultTexture(&spec);
  elemtype = mjOBJ_TEXTURE;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear user settings: single file
  spec_file_.clear();
  spec_content_type_.clear();

  // clear user settings: separate file
  spec_cubefiles_.assign(6, "");

  // clear internal variables
  data_.clear();
  clear_data_ = false;

  // point to local
  PointToLocal();

  // in case this texture is not compiled
  CopyFromSpec();
}



mjCTexture::mjCTexture(const mjCTexture& other) {
  *this = other;
}



mjCTexture& mjCTexture::operator=(const mjCTexture& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCTexture_*>(this) = static_cast<const mjCTexture_&>(other);
    clear_data_ = other.clear_data_;
  }
  PointToLocal();
  return *this;
}



void mjCTexture::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.file = &spec_file_;
  spec.data = &data_;
  spec.content_type = &spec_content_type_;
  spec.cubefiles = &spec_cubefiles_;
  spec.info = &info;
  file = nullptr;
  content_type = nullptr;
  cubefiles = nullptr;
}



void mjCTexture::CopyFromSpec() {
  *static_cast<mjsTexture*>(this) = spec;
  file_ = spec_file_;
  content_type_ = spec_content_type_;
  cubefiles_ = spec_cubefiles_;

  if (clear_data_) {
    // clear precompiled asset. TODO: use asset cache
    data_.clear();
  }

  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = mjuu_strippath(file_);
    name = mjuu_stripext(stripped);
  }
}



void mjCTexture::NameSpace(const mjCModel* m) {
  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = mjuu_strippath(spec_file_);
    name = mjuu_stripext(stripped);
  }
  mjCBase::NameSpace(m);
  if (modelfiledir_.empty()) {
    modelfiledir_ = FilePath(m->spec_modelfiledir_);
  }
  if (texturedir_.empty()) {
    texturedir_ = FilePath(m->spec_texturedir_);
  }
}



// free data storage allocated by lodepng
mjCTexture::~mjCTexture() {
  data_.clear();
}



// insert random dots
static void randomdot(std::byte* rgb, const double* markrgb,
                      int width, int height, double probability) {
  // make distribution using fixed seed
  std::mt19937_64 rng;
  rng.seed(42);
  std::uniform_real_distribution<double> dist(0, 1);

  // sample
  for (int r=0; r < height; r++) {
    for (int c=0; c < width; c++) {
      if (dist(rng) < probability) {
        for (int j=0; j < 3; j++) {
          rgb[3*(r*width+c)+j] = (std::byte)(255*markrgb[j]);
        }
      }
    }
  }
}



// interpolate between colors based on value in (-1, +1)
static void interp(std::byte* rgb, const double* rgb1, const double* rgb2, double pos) {
  const double correction = 1.0/sqrt(2);
  double alpha = 0.5*(1 + pos/sqrt(1+pos*pos)/correction);
  if (alpha < 0) {
    alpha = 0;
  } else if (alpha > 1) {
    alpha = 1;
  }

  for (int j=0; j < 3; j++) {
    rgb[j] = (std::byte)(255*(alpha*rgb1[j] + (1-alpha)*rgb2[j]));
  }
}



// make checker pattern for one side
static void checker(std::byte* rgb, const std::byte* RGB1, const std::byte* RGB2,
                    int width, int height) {
  for (int r=0; r < height/2; r++) {
    for (int c=0; c < width/2; c++) {
      memcpy(rgb+3*(r*width+c), RGB1, 3);
    }
  }
  for (int r=height/2; r < height; r++) {
    for (int c=width/2; c < width; c++) {
      memcpy(rgb+3*(r*width+c), RGB1, 3);
    }
  }
  for (int r=0; r < height/2; r++) {
    for (int c=width/2; c < width; c++) {
      memcpy(rgb+3*(r*width+c), RGB2, 3);
    }
  }
  for (int r=height/2; r < height; r++) {
    for (int c=0; c < width/2; c++) {
      memcpy(rgb+3*(r*width+c), RGB2, 3);
    }
  }
}



// make builtin: 2D
void mjCTexture::Builtin2D(void) {
  std::byte RGB1[3], RGB2[3], RGBm[3];
  // convert fixed colors
  for (int j=0; j < 3; j++) {
    RGB1[j] = (std::byte)(255*rgb1[j]);
    RGB2[j] = (std::byte)(255*rgb2[j]);
    RGBm[j] = (std::byte)(255*markrgb[j]);
  }

  //------------------ face

  // gradient
  if (builtin == mjBUILTIN_GRADIENT) {
    for (int r=0; r < height; r++) {
      for (int c=0; c < width; c++) {
        // compute normalized coordinates and radius
        double x = 2*c/((double)(width-1)) - 1;
        double y = 1 - 2*r/((double)(height-1));
        double pos = 2*sqrt(x*x+y*y) - 1;

        // interpolate through sigmoid
        interp(data_.data() + 3*(r*width+c), rgb2, rgb1, pos);
      }
    }
  }

  // checker
  else if (builtin == mjBUILTIN_CHECKER) {
    checker(data_.data(), RGB1, RGB2, width, height);
  }

  // flat
  else if (builtin == mjBUILTIN_FLAT) {
    for (int r=0; r < height; r++) {
      for (int c=0; c < width; c++) {
        memcpy(data_.data()+3*(r*width+c), RGB1, 3);
      }
    }
  }

  //------------------ marks

  // edge
  if (mark == mjMARK_EDGE) {
    for (int r=0; r < height; r++) {
      memcpy(data_.data()+3*(r*width+0), RGBm, 3);
      memcpy(data_.data()+3*(r*width+width-1), RGBm, 3);
    }
    for (int c=0; c < width; c++) {
      memcpy(data_.data()+3*(0*width+c), RGBm, 3);
      memcpy(data_.data()+3*((height-1)*width+c), RGBm, 3);
    }
  }

  // cross
  else if (mark == mjMARK_CROSS) {
    for (int r=0; r < height; r++) {
      memcpy(data_.data()+3*(r*width+width/2), RGBm, 3);
    }
    for (int c=0; c < width; c++) {
      memcpy(data_.data()+3*(height/2*width+c), RGBm, 3);
    }
  }

  // random dots
  else if (mark == mjMARK_RANDOM && random > 0) {
    randomdot(data_.data(), markrgb, width, height, random);
  }
}



// make builtin: Cube
void mjCTexture::BuiltinCube(void) {
  std::byte RGB1[3], RGB2[3], RGBm[3], RGBi[3];
  int w = width;
  if (w > std::numeric_limits<int>::max() / w) {
    throw mjCError(this, "Cube texture width is too large.");
  }
  int ww = width*width;

  // convert fixed colors
  for (int j = 0; j < 3; j++) {
    RGB1[j] = (std::byte)(255 * rgb1[j]);
    RGB2[j] = (std::byte)(255 * rgb2[j]);
    RGBm[j] = (std::byte)(255 * markrgb[j]);
  }

  //------------------ faces

  // gradient
  if (builtin == mjBUILTIN_GRADIENT) {
    if (ww > std::numeric_limits<int>::max() / 18) {
      throw mjCError(this, "Gradient texture width is too large.");
    }
    for (int r = 0; r < w; r++) {
      for (int c = 0; c < w; c++) {
        // compute normalized pixel coordinates
        double x = 2 * c / ((double)(w - 1)) - 1;
        double y = 1 - 2 * r / ((double)(w - 1));

        // compute normalized elevation for sides and up/down
        double elside = asin(y / sqrt(1 + x * x + y * y)) / (0.5 * mjPI);
        double elup = 1 - acos(1.0 / sqrt(1 + x * x + y * y)) / (0.5 * mjPI);

        // set sides
        interp(RGBi, rgb1, rgb2, elside);
        memcpy(data_.data() + 0 * 3 * ww + 3 * (r * w + c), RGBi, 3);  // 0: right
        memcpy(data_.data() + 1 * 3 * ww + 3 * (r * w + c), RGBi, 3);  // 1: left
        memcpy(data_.data() + 4 * 3 * ww + 3 * (r * w + c), RGBi, 3);  // 4: front
        memcpy(data_.data() + 5 * 3 * ww + 3 * (r * w + c), RGBi, 3);  // 5: back

        // set up and down
        interp(data_.data() + 2 * 3 * ww + 3 * (r * w + c), rgb1, rgb2, elup);  // 2: up
        interp(data_.data() + 3 * 3 * ww + 3 * (r * w + c), rgb1, rgb2, -elup);  // 3: down
      }
    }
  }

  // checker
  else if (builtin == mjBUILTIN_CHECKER) {
    checker(data_.data() + 0 * 3 * ww, RGB1, RGB2, w, w);
    checker(data_.data() + 1 * 3 * ww, RGB1, RGB2, w, w);
    checker(data_.data() + 2 * 3 * ww, RGB1, RGB2, w, w);
    checker(data_.data() + 3 * 3 * ww, RGB1, RGB2, w, w);
    checker(data_.data() + 4 * 3 * ww, RGB2, RGB1, w, w);
    checker(data_.data() + 5 * 3 * ww, RGB2, RGB1, w, w);
  }

  // flat
  else if (builtin == mjBUILTIN_FLAT) {
    for (int r = 0; r < w; r++) {
      for (int c = 0; c < w; c++) {
        // set sides and up
        memcpy(data_.data() + 0 * 3 * ww + 3 * (r * w + c), RGB1, 3);
        memcpy(data_.data() + 1 * 3 * ww + 3 * (r * w + c), RGB1, 3);
        memcpy(data_.data() + 2 * 3 * ww + 3 * (r * w + c), RGB1, 3);
        memcpy(data_.data() + 4 * 3 * ww + 3 * (r * w + c), RGB1, 3);
        memcpy(data_.data() + 5 * 3 * ww + 3 * (r * w + c), RGB1, 3);

        // set down
        memcpy(data_.data() + 3 * 3 * ww + 3 * (r * w + c), RGB2, 3);
      }
    }
  }

  //------------------ marks

  // edge
  if (mark == mjMARK_EDGE) {
    for (int j = 0; j < 6; j++) {
      for (int r = 0; r < w; r++) {
        memcpy(data_.data() + j * 3 * ww + 3 * (r * w + 0), RGBm, 3);
        memcpy(data_.data() + j * 3 * ww + 3 * (r * w + w - 1), RGBm, 3);
      }
      for (int c = 0; c < w; c++) {
        memcpy(data_.data() + j * 3 * ww + 3 * (0 * w + c), RGBm, 3);
        memcpy(data_.data() + j * 3 * ww + 3 * ((w - 1) * w + c), RGBm, 3);
      }
    }
  }

  // cross
  else if (mark == mjMARK_CROSS) {
    for (int j = 0; j < 6; j++) {
      for (int r = 0; r < w; r++) {
        memcpy(data_.data() + j * 3 * ww + 3 * (r * w + w / 2), RGBm, 3);
      }
      for (int c = 0; c < w; c++) {
        memcpy(data_.data() + j * 3 * ww + 3 * (w / 2 * w + c), RGBm, 3);
      }
    }
  }

  // random dots
  else if (mark == mjMARK_RANDOM && random > 0) {
    randomdot(data_.data(), markrgb, w, height, random);
  }
}

// load PNG file
void mjCTexture::LoadPNG(mjResource* resource,
                         std::vector<unsigned char>& image,
                         unsigned int& w, unsigned int& h, bool& is_srgb) {
  LodePNGColorType color_type;
  if (nchannel == 4) {
    color_type = LCT_RGBA;
  } else if (nchannel == 3) {
    color_type = LCT_RGB;
  } else if (nchannel == 1) {
    color_type = LCT_GREY;
  } else {
    throw mjCError(this, "Unsupported number of channels: %s",
                   std::to_string(nchannel).c_str());
  }
  PNGImage png_image = PNGImage::Load(this, resource, color_type);
  w = png_image.Width();
  h = png_image.Height();
  is_srgb = png_image.IsSRGB();
  image = png_image.MoveData();
}

// load KTX file
void mjCTexture::LoadKTX(mjResource* resource,
                         std::vector<unsigned char>& image, unsigned int& w,
                        unsigned int& h, bool& is_srgb) {
  const void* buffer = 0;
  int buffer_sz = mju_readResource(resource, &buffer);

  // still not found
  if (buffer_sz < 0) {
    throw mjCError(this, "could not read texture file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw mjCError(this, "texture file is empty: '%s'", resource->name);
  }

  w = buffer_sz;
  h = 1;
  is_srgb = false;

  image.resize(buffer_sz);
  memcpy(image.data(), buffer, buffer_sz);
}

// load custom file
void mjCTexture::LoadCustom(mjResource* resource,
                            std::vector<unsigned char>& image,
                            unsigned int& w, unsigned int& h, bool& is_srgb) {
  const void* buffer = 0;
  int buffer_sz = mju_readResource(resource, &buffer);

  // still not found
  if (buffer_sz < 0) {
    throw mjCError(this, "could not read texture file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw mjCError(this, "texture file is empty: '%s'", resource->name);
  }


  // read dimensions
  int* pint = (int*)buffer;
  w = pint[0];
  h = pint[1];

  // assume linear color space
  is_srgb = false;

  // check dimensions
  if (w < 1 || h < 1) {
    throw mjCError(this, "Non-PNG texture, assuming custom binary file format,\n"
                         "non-positive texture dimensions in file '%s'", resource->name);
  }

  // check buffer size
  if (buffer_sz != 2*sizeof(int) + w*h*3*sizeof(char)) {
    throw mjCError(this, "Non-PNG texture, assuming custom binary file format,\n"
                         "unexpected file size in file '%s'", resource->name);
  }

  // allocate and copy
  image.resize(w*h*3);
  memcpy(image.data(), (void*)(pint+2), w*h*3*sizeof(char));
}



// load from PNG or custom file, flip if specified
void mjCTexture::LoadFlip(std::string filename, const mjVFS* vfs,
                          std::vector<unsigned char>& image,
                          unsigned int& w, unsigned int& h, bool& is_srgb) {
  std::string asset_type = GetAssetContentType(filename, content_type_);

  // fallback to custom
  if (asset_type.empty()) {
    asset_type = "image/vnd.mujoco.texture";
  }

  if (asset_type != "image/png" && asset_type != "image/ktx" && asset_type != "image/vnd.mujoco.texture") {
    throw mjCError(this, "unsupported content type: '%s'", asset_type.c_str());
  }

  mjResource* resource = LoadResource(modelfiledir_.Str(), filename, vfs);

  try {
    if (asset_type == "image/png") {
      LoadPNG(resource, image, w, h, is_srgb);
    } else if (asset_type == "image/ktx") {
      if (hflip || vflip) {
        throw mjCError(this, "cannot flip KTX textures");
      }
      LoadKTX(resource, image, w, h, is_srgb);
    } else {
      LoadCustom(resource, image, w, h, is_srgb);
    }
    mju_closeResource(resource);
  } catch(mjCError err) {
    mju_closeResource(resource);
    throw err;
  }

  // horizontal flip
  if (hflip) {
    if (nchannel != 3) {
      throw mjCError(
              this, "currently only 3-channel textures support horizontal flip");
    }
    for (int r=0; r < h; r++) {
      for (int c=0; c < w/2; c++) {
        int c1 = w-1-c;
        unsigned char tmp[3] = {
          image[3*(r*w+c)],
          image[3*(r*w+c)+1],
          image[3*(r*w+c)+2]
        };

        image[3*(r*w+c)]   = image[3*(r*w+c1)];
        image[3*(r*w+c)+1] = image[3*(r*w+c1)+1];
        image[3*(r*w+c)+2] = image[3*(r*w+c1)+2];

        image[3*(r*w+c1)]   = tmp[0];
        image[3*(r*w+c1)+1] = tmp[1];
        image[3*(r*w+c1)+2] = tmp[2];
      }
    }
  }

  // vertical flip
  if (vflip) {
    if (nchannel != 3) {
      throw mjCError(
              this, "currently only 3-channel textures support vertical flip");
    }
    for (int r=0; r < h/2; r++) {
      for (int c=0; c < w; c++) {
        int r1 = h-1-r;
        unsigned char tmp[3] = {
          image[3*(r*w+c)],
          image[3*(r*w+c)+1],
          image[3*(r*w+c)+2]
        };

        image[3*(r*w+c)]   = image[3*(r1*w+c)];
        image[3*(r*w+c)+1] = image[3*(r1*w+c)+1];
        image[3*(r*w+c)+2] = image[3*(r1*w+c)+2];

        image[3*(r1*w+c)]   = tmp[0];
        image[3*(r1*w+c)+1] = tmp[1];
        image[3*(r1*w+c)+2] = tmp[2];
      }
    }
  }
}



// load 2D
void mjCTexture::Load2D(std::string filename, const mjVFS* vfs) {
  // load PNG or custom
  unsigned int w, h;
  bool is_srgb;
  std::vector<unsigned char> image;
  LoadFlip(filename, vfs, image, w, h, is_srgb);

  // assign size
  width = w;
  height = h;
  if (colorspace == mjCOLORSPACE_AUTO) {
    colorspace = is_srgb ? mjCOLORSPACE_SRGB : mjCOLORSPACE_LINEAR;
  }

  // allocate and copy data
  std::int64_t size = static_cast<std::int64_t>(width)*height;
  if (size >= std::numeric_limits<int>::max() / nchannel || size <= 0) {
    throw mjCError(this, "Texture too large");
  }
  try {
    data_.assign(nchannel*size, std::byte(0));
  } catch (const std::bad_alloc& e) {
    throw mjCError(this, "Could not allocate memory for texture '%s' (id %d)",
                   (const char*)file_.c_str(), id);
  }
  memcpy(data_.data(), image.data(), nchannel*size);
  image.clear();
}



// load cube or skybox from single file (repeated or grid)
void mjCTexture::LoadCubeSingle(std::string filename, const mjVFS* vfs) {
  // check gridsize
  if (gridsize[0] < 1 || gridsize[1] < 1 || gridsize[0]*gridsize[1] > 12) {
    throw mjCError(this, "gridsize must be non-zero and no more than 12 squares in texture");
  }

  // load PNG or custom
  unsigned int w, h;
  bool is_srgb;
  std::vector<unsigned char> image;
  LoadFlip(filename, vfs, image, w, h, is_srgb);

  if (colorspace == mjCOLORSPACE_AUTO) {
    colorspace = is_srgb ? mjCOLORSPACE_SRGB : mjCOLORSPACE_LINEAR;
  }

  // check gridsize for compatibility
  if (w/gridsize[1] != h/gridsize[0] || (w%gridsize[1]) || (h%gridsize[0])) {
    throw mjCError(this,
                   "PNG size must be integer multiple of gridsize in texture '%s' (id %d)",
                   (const char*)file_.c_str(), id);
  }

  // assign size: repeated or full
  if (gridsize[0] == 1 && gridsize[1] == 1) {
    width = height = w;
  } else {
    width = w/gridsize[1];
    if (width >= std::numeric_limits<int>::max()/6) {
      throw mjCError(this, "Invalid width of cube texture");
    }
    height = 6*width;
  }

  // allocate data
  std::int64_t size = static_cast<std::int64_t>(width)*height;
  if (size >= std::numeric_limits<int>::max() / 3 || size <= 0) {
    throw mjCError(this, "Cube texture too large");
  }
  try {
    data_.assign(3*size, std::byte(0));
  } catch (const std::bad_alloc& e) {
    throw mjCError(this,
                   "Could not allocate memory for texture '%s' (id %d)",
                   (const char*)file_.c_str(), id);
  }

  // copy: repeated
  if (gridsize[0] == 1 && gridsize[1] == 1) {
    memcpy(data_.data(), image.data(), 3*width*width);
  }

  // copy: grid
  else {
    // keep track of which faces were defined
    int loaded[6] = {0, 0, 0, 0, 0, 0};

    // process grid
    for (int k=0; k < gridsize[0]*gridsize[1]; k++) {
      // decode face symbol
      int i = -1;
      if (gridlayout[k] == 'R') {
        i = 0;
      } else if (gridlayout[k] == 'L') {
        i = 1;
      } else if (gridlayout[k] == 'U') {
        i = 2;
      } else if (gridlayout[k] == 'D') {
        i = 3;
      } else if (gridlayout[k] == 'F') {
        i = 4;
      } else if (gridlayout[k] == 'B') {
        i = 5;
      } else if (gridlayout[k] != '.')
        throw mjCError(this, "gridlayout symbol is not among '.RLUDFB' in texture");

      // load if specified
      if (i >= 0) {
        // extract sub-image
        int rstart = width*(k/gridsize[1]);
        int cstart = width*(k%gridsize[1]);
        for (int j=0; j < width; j++) {
          memcpy(data_.data()+i*3*width*width+j*3*width, image.data()+(j+rstart)*3*w+3*cstart, 3*width);
        }

        // mark as defined
        loaded[i] = 1;
      }
    }

    // set undefined faces to rgb1
    for (int i=0; i < 6; i++) {
      if (!loaded[i]) {
        for (int k=0; k < width; k++) {
          for (int s=0; s < width; s++) {
            for (int j=0; j < 3; j++) {
              data_[i*3*width*width + 3*(k*width+s) + j] = (std::byte)(255*rgb1[j]);
            }
          }
        }
      }
    }
  }

  image.clear();
}



// load cube or skybox from separate file
void mjCTexture::LoadCubeSeparate(const mjVFS* vfs) {
  // keep track of which faces were defined
  int loaded[6] = {0, 0, 0, 0, 0, 0};

  // process nonempty files
  for (int i=0; i < 6; i++) {
    if (!cubefiles_[i].empty()) {
      // remove path from file if necessary
      if (model->strippath) {
        cubefiles_[i] = mjuu_strippath(cubefiles_[i]);
      }

      // make filename
      FilePath filename = texturedir_ + FilePath(cubefiles_[i]);

      // load PNG or custom
      unsigned int w, h;
      bool is_srgb;
      std::vector<unsigned char> image;
      LoadFlip(filename.Str(), vfs, image, w, h, is_srgb);

      // assume all faces have the same colorspace
      if (colorspace == mjCOLORSPACE_AUTO) {
        colorspace = is_srgb ? mjCOLORSPACE_SRGB : mjCOLORSPACE_LINEAR;
      }

      // PNG must be square
      if (w != h) {
        throw mjCError(this,
                       "Non-square PNG file '%s' in cube or skybox id %d",
                       (const char*)cubefiles_[i].c_str(), id);
      }

      // first file: set size and allocate data
      if (data_.empty()) {
        width = w;
        if (width >= std::numeric_limits<int>::max()/6) {
          throw mjCError(this, "Invalid width of builtin texture");
        }
        height = 6*width;
        std::int64_t size = static_cast<std::int64_t>(width)*height;
        if (size >= std::numeric_limits<int>::max() / 3 || size <= 0) {
          throw mjCError(this, "PNG texture too large");
        }
        try {
          data_.assign(3*size, std::byte(0));
        } catch (const std::bad_alloc& e) {
          throw mjCError(this, "Could not allocate memory for texture");
        }
      }

      // otherwise check size
      else if (width != w) {
        throw mjCError(this,
                       "PNG file '%s' has incompatible size in texture id %d",
                       (const char*)cubefiles_[i].c_str(), id);
      }

      // copy data
      memcpy(data_.data()+i*3*width*width, image.data(), 3*width*width);
      image.clear();

      // mark as defined
      loaded[i] = 1;
    }
  }

  // set undefined faces to rgb1
  for (int i=0; i < 6; i++) {
    if (!loaded[i]) {
      for (int k=0; k < width; k++) {
        for (int s=0; s < width; s++) {
          for (int j=0; j < 3; j++) {
            data_[i*3*width*width + 3*(k*width+s) + j] = (std::byte)(255*rgb1[j]);
          }
        }
      }
    }
  }
}



// compiler
void mjCTexture::Compile(const mjVFS* vfs) {
  CopyFromSpec();

  // copy paths from model if not already defined
  if (modelfiledir_.empty()) {
    modelfiledir_ = FilePath(model->modelfiledir_);
  }
  if (texturedir_.empty()) {
    texturedir_ = FilePath(model->texturedir_);
  }

  // buffer from user
  if (!data_.empty()) {
    if (data_.size() != nchannel*width*height) {
      throw mjCError(this, "Texture buffer has incorrect size, given %d expected %d", nullptr,
                     data_.size(), nchannel * width * height);
    }
    return;
  }

  // builtin
  else if (builtin != mjBUILTIN_NONE) {
    // check width
    if (width < 1) {
      throw mjCError(this, "Invalid width of builtin texture");
    }

    // adjust height of cube texture
    if (type != mjTEXTURE_2D) {
      if (width >= std::numeric_limits<int>::max()/6) {
        throw mjCError(this, "Invalid width of builtin texture");
      }
      height = 6*width;
    } else {
      if (height < 1) {
        throw mjCError(this, "Invalid height of builtin texture");
      }
    }

    std::int64_t size = static_cast<std::int64_t>(width)*height;
    if (size >= std::numeric_limits<int>::max() / nchannel || size <= 0) {
      throw mjCError(this, "Builtin texture too large");
    }
    // allocate data
    try {
      data_.assign(nchannel*size, std::byte(0));
    } catch (const std::bad_alloc& e) {
      throw mjCError(this, "Could not allocate memory for texture");
    }

    // dispatch
    if (type == mjTEXTURE_2D) {
      Builtin2D();
    } else {
      BuiltinCube();
    }
  }

  // single file
  else if (!file_.empty()) {
    // remove path from file if necessary
    if (model->strippath) {
      file_ = mjuu_strippath(file_);
    }

    // make filename
    FilePath filename = texturedir_ + FilePath(file_);

    // dispatch
    if (type == mjTEXTURE_2D) {
      Load2D(filename.Str(), vfs);
    } else {
      LoadCubeSingle(filename.Str(), vfs);
    }
  }

  // separate files
  else {
    // 2D not allowed
    if (type == mjTEXTURE_2D) {
      throw mjCError(this,
                     "Cannot load 2D texture from separate files, texture");
    }

    // at least one cubefile must be defined
    bool defined = false;
    for (int i=0; i < 6; i++) {
      if (!cubefiles_[i].empty()) {
        defined = true;
        break;
      }
    }
    if (!defined) {
      throw mjCError(this,
                     "No cubefiles_ defined in cube or skybox texture");
    }

    // only cube and skybox
    LoadCubeSeparate(vfs);
  }

  // make sure someone allocated data; SHOULD NOT OCCUR
  if (data_.empty()) {
    throw mjCError(this, "texture '%s' (id %d) was not specified", name.c_str(), id);
  }

  // if recompiled is called, clear data_ first
  clear_data_ = true;
}



//------------------ class mjCMaterial implementation ----------------------------------------------

// initialize defaults
mjCMaterial::mjCMaterial(mjCModel* _model, mjCDef* _def) {
  mjs_defaultMaterial(&spec);
  elemtype = mjOBJ_MATERIAL;
  textures_.assign(mjNTEXROLE, "");
  spec_textures_.assign(mjNTEXROLE, "");

  // clear internal
  for (int i=0; i < mjNTEXROLE; i++) {
    texid[i] = -1;
  }

  // reset to default if given
  if (_def) {
    *this = _def->Material();
  }

  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  PointToLocal();

  // in case this material is not compiled
  CopyFromSpec();
}



mjCMaterial::mjCMaterial(const mjCMaterial& other) {
  *this = other;
}



mjCMaterial& mjCMaterial::operator=(const mjCMaterial& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCMaterial_*>(this) = static_cast<const mjCMaterial_&>(other);
    *static_cast<mjsMaterial*>(this) = static_cast<const mjsMaterial&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCMaterial::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.textures = &spec_textures_;
  spec.info = &info;
  textures = nullptr;
}



void mjCMaterial::CopyFromSpec() {
  *static_cast<mjsMaterial*>(this) = spec;
  textures_ = spec_textures_;
}



void mjCMaterial::NameSpace(const mjCModel* m) {
  mjCBase::NameSpace(m);
  for (int i=0; i < mjNTEXROLE; i++) {
    if (!spec_textures_[i].empty()) {
      spec_textures_[i] = m->prefix + spec_textures_[i] + m->suffix;
    }
  }
}



// compiler
void mjCMaterial::Compile(void) {
  CopyFromSpec();
}



//------------------ class mjCPair implementation --------------------------------------------------

// constructor
mjCPair::mjCPair(mjCModel* _model, mjCDef* _def) {
  mjs_defaultPair(&spec);
  elemtype = mjOBJ_PAIR;

  // set defaults
  spec_geomname1_.clear();
  spec_geomname2_.clear();

  // clear internal variables
  geom1 = nullptr;
  geom2 = nullptr;
  signature = -1;

  // reset to default if given
  if (_def) {
    *this = _def->Pair();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCPair::mjCPair(const mjCPair& other) {
  *this = other;
}



mjCPair& mjCPair::operator=(const mjCPair& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCPair_*>(this) = static_cast<const mjCPair_&>(other);
    *static_cast<mjsPair*>(this) = static_cast<const mjsPair&>(other);
    this->geom1 = nullptr;
    this->geom2 = nullptr;
  }
  PointToLocal();
  return *this;
}



void mjCPair::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.geomname1 = &spec_geomname1_;
  spec.geomname2 = &spec_geomname2_;
  geomname1 = nullptr;
  geomname2 = nullptr;
  spec.info = &info;
}



void mjCPair::NameSpace(const mjCModel* m) {
  mjCBase::NameSpace(m);
  prefix = m->prefix;
  suffix = m->suffix;
}



void mjCPair::CopyFromSpec() {
  *static_cast<mjsPair*>(this) = spec;
  geomname1_ = spec_geomname1_;
  geomname2_ = spec_geomname2_;
}



void mjCPair::ResolveReferences(const mjCModel* m) {
  geomname1_ = prefix + geomname1_ + suffix;
  geomname2_ = prefix + geomname2_ + suffix;
  geom1 = (mjCGeom*)m->FindObject(mjOBJ_GEOM, geomname1_);
  geom2 = (mjCGeom*)m->FindObject(mjOBJ_GEOM, geomname2_);

  if (!geom1 && geom2) {
    geomname1_ = spec_geomname1_;
    geom1 = (mjCGeom*)m->FindObject(mjOBJ_GEOM, geomname1_);
  }
  if (geom1 && !geom2) {
    geomname2_ = spec_geomname2_;
    geom2 = (mjCGeom*)m->FindObject(mjOBJ_GEOM, geomname2_);
  }

  if (!geom1) {
    throw mjCError(this, "geom '%s' not found in collision %d", geomname1_.c_str(), id);
  }
  if (!geom2) {
    throw mjCError(this, "geom '%s' not found in collision %d", geomname2_.c_str(), id);
  }

  spec_geomname1_ = geomname1_;
  spec_geomname2_ = geomname2_;
  prefix.clear();
  suffix.clear();

  // swap if body1 > body2
  if (geom1->body->id > geom2->body->id) {
    std::string nametmp = geomname1_;
    geomname1_ = geomname2_;
    geomname2_ = nametmp;

    mjCGeom* geomtmp = geom1;
    geom1 = geom2;
    geom2 = geomtmp;
  }

  // get geom ids and body signature
  signature = ((geom1->body->id)<<16) + geom2->body->id;
}



// compiler
void mjCPair::Compile(void) {
  CopyFromSpec();

  // check condim
  if (condim != 1 && condim != 3 && condim != 4 && condim != 6) {
    throw mjCError(this, "invalid condim in contact pair");
  }

  // find geoms
  ResolveReferences(model);

  // mark geoms as not visual
  geom1->SetNotVisual();
  geom2->SetNotVisual();

  // set undefined margin: max
  if (!mjuu_defined(margin)) {
    margin = std::max(geom1->margin, geom2->margin);
  }

  // set undefined gap: max
  if (!mjuu_defined(gap)) {
    gap = std::max(geom1->gap, geom2->gap);
  }

  // set undefined condim, friction, solref, solimp: different priority
  if (geom1->priority != geom2->priority) {
    mjCGeom* pgh = (geom1->priority > geom2->priority ? geom1 : geom2);

    // condim
    if (condim < 0) {
      condim = pgh->condim;
    }

    // friction
    if (!mjuu_defined(friction[0])) {
      friction[0] = friction[1] = pgh->friction[0];
      friction[2] =               pgh->friction[1];
      friction[3] = friction[4] = pgh->friction[2];
    }

    // reference
    if (!mjuu_defined(solref[0])) {
      for (int i=0; i < mjNREF; i++) {
        solref[i] = pgh->solref[i];
      }
    }

    // impedance
    if (!mjuu_defined(solimp[0])) {
      for (int i=0; i < mjNIMP; i++) {
        solimp[i] = pgh->solimp[i];
      }
    }
  }

  // set undefined condim, friction, solref, solimp: same priority
  else {
    // condim: max
    if (condim < 0) {
      condim = std::max(geom1->condim, geom2->condim);
    }

    // friction: max
    if (!mjuu_defined(friction[0])) {
      friction[0] = friction[1] = std::max(geom1->friction[0], geom2->friction[0]);
      friction[2] =               std::max(geom1->friction[1], geom2->friction[1]);
      friction[3] = friction[4] = std::max(geom1->friction[2], geom2->friction[2]);
    }

    // solver mix factor
    double mix;
    if (geom1->solmix >= mjEPS && geom2->solmix >= mjEPS) {
      mix = geom1->solmix / (geom1->solmix + geom2->solmix);
    } else if (geom1->solmix < mjEPS && geom2->solmix < mjEPS) {
      mix = 0.5;
    } else if (geom1->solmix < mjEPS) {
      mix = 0.0;
    } else {
      mix = 1.0;
    }

    // reference
    if (!mjuu_defined(solref[0])) {
      // standard: mix
      if (solref[0] > 0) {
        for (int i=0; i < mjNREF; i++) {
          solref[i] = mix*geom1->solref[i] + (1-mix)*geom2->solref[i];
        }
      }

      // direct: min
      else {
        for (int i=0; i < mjNREF; i++) {
          solref[i] = std::min(geom1->solref[i], geom2->solref[i]);
        }
      }
    }

    // impedance
    if (!mjuu_defined(solimp[0])) {
      for (int i=0; i < mjNIMP; i++) {
        solimp[i] = mix*geom1->solimp[i] + (1-mix)*geom2->solimp[i];
      }
    }
  }
}



//------------------ class mjCBodyPair implementation ----------------------------------------------

// constructor
mjCBodyPair::mjCBodyPair(mjCModel* _model) {
  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  elemtype = mjOBJ_EXCLUDE;

  // set defaults
  spec_bodyname1_.clear();
  spec_bodyname2_.clear();

  // clear internal variables
  body1 = body2 = signature = -1;

  PointToLocal();
  CopyFromSpec();
}



mjCBodyPair::mjCBodyPair(const mjCBodyPair& other) {
  *this = other;
}



mjCBodyPair& mjCBodyPair::operator=(const mjCBodyPair& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCBodyPair_*>(this) = static_cast<const mjCBodyPair_&>(other);
    *static_cast<mjsExclude*>(this) = static_cast<const mjsExclude&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCBodyPair::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.bodyname1 = &spec_bodyname1_;
  spec.bodyname2 = &spec_bodyname2_;
  spec.info = &info;
  bodyname1 = nullptr;
  bodyname2 = nullptr;
}



void mjCBodyPair::NameSpace(const mjCModel* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  prefix = m->prefix;
  suffix = m->suffix;
}



void mjCBodyPair::CopyFromSpec() {
  *static_cast<mjsExclude*>(this) = spec;
  bodyname1_ = spec_bodyname1_;
  bodyname2_ = spec_bodyname2_;
}



void mjCBodyPair::ResolveReferences(const mjCModel* m) {
  bodyname1_ = prefix + bodyname1_ + suffix;
  bodyname2_ = prefix + bodyname2_ + suffix;
  mjCBody* pb1 = (mjCBody*)m->FindObject(mjOBJ_BODY, bodyname1_);
  mjCBody* pb2 = (mjCBody*)m->FindObject(mjOBJ_BODY, bodyname2_);

  if (!pb1 && pb2) {
    bodyname1_ = spec_bodyname1_;
    pb1 = (mjCBody*)m->FindObject(mjOBJ_BODY, bodyname1_);
  }
  if (pb1 && !pb2) {
    bodyname2_ = spec_bodyname2_;
    pb2 = (mjCBody*)m->FindObject(mjOBJ_BODY, bodyname2_);
  }

  if (!pb1) {
    throw mjCError(this, "body '%s' not found in bodypair %d", bodyname1_.c_str(), id);
  }
  if (!pb2) {
    throw mjCError(this, "body '%s' not found in bodypair %d", bodyname2_.c_str(), id);
  }

  spec_bodyname1_ = bodyname1_;
  spec_bodyname2_ = bodyname2_;
  prefix.clear();
  suffix.clear();

  // swap if body1 > body2
  if (pb1->id > pb2->id) {
    std::string nametmp = bodyname1_;
    bodyname1_ = bodyname2_;
    bodyname2_ = nametmp;

    mjCBody* bodytmp = pb1;
    pb1 = pb2;
    pb2 = bodytmp;
  }

  // get body ids and body signature
  body1 = pb1->id;
  body2 = pb2->id;
  signature = (body1<<16) + body2;
}



// compiler
void mjCBodyPair::Compile(void) {
  CopyFromSpec();

  // find bodies
  ResolveReferences(model);
}



//------------------ class mjCEquality implementation ----------------------------------------------

// initialize default constraint
mjCEquality::mjCEquality(mjCModel* _model, mjCDef* _def) {
  mjs_defaultEquality(&spec);
  elemtype = mjOBJ_EQUALITY;

  // clear internal variables
  spec_name1_.clear();
  spec_name2_.clear();
  obj1id = obj2id = -1;

  // reset to default if given
  if (_def) {
    *this = _def->Equality();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCEquality::mjCEquality(const mjCEquality& other) {
  *this = other;
}



mjCEquality& mjCEquality::operator=(const mjCEquality& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCEquality_*>(this) = static_cast<const mjCEquality_&>(other);
    *static_cast<mjsEquality*>(this) = static_cast<const mjsEquality&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCEquality::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.name1 = &spec_name1_;
  spec.name2 = &spec_name2_;
  spec.info = &info;
  name1 = nullptr;
  name2 = nullptr;
}



void mjCEquality::NameSpace(const mjCModel* m) {
  mjCBase::NameSpace(m);
  if (!spec_name1_.empty()) {
    spec_name1_ = m->prefix + spec_name1_ + m->suffix;
  }
  if (!spec_name2_.empty()) {
    spec_name2_ = m->prefix + spec_name2_ + m->suffix;
  }
}



void mjCEquality::CopyFromSpec() {
  *static_cast<mjsEquality*>(this) = spec;
  name1_ = spec_name1_;
  name2_ = spec_name2_;
}



void mjCEquality::ResolveReferences(const mjCModel* m) {
  mjtObj object_type;
  mjCBase *px1, *px2;
  mjtJoint jt1, jt2;

  // determine object type
  if (type == mjEQ_WELD) {
    if (objtype != mjOBJ_SITE && objtype != mjOBJ_BODY) {
      throw mjCError(this, "weld constraint supports only sites and bodies");
    }
    object_type = objtype;
  } else if (type == mjEQ_CONNECT) {
    if (objtype != mjOBJ_SITE && objtype != mjOBJ_BODY) {
      throw mjCError(this, "connect constraint supports only sites and bodies");
    }
    object_type = objtype;
  } else if (type == mjEQ_JOINT) {
    object_type = mjOBJ_JOINT;
  } else if (type == mjEQ_TENDON) {
    object_type = mjOBJ_TENDON;
  } else if (type == mjEQ_FLEX) {
    object_type = mjOBJ_FLEX;
  } else {
    throw mjCError(this, "invalid type in equality constraint");
  }

  // find object 1, get id
  px1 = m->FindObject(object_type, name1_);
  if (!px1) {
    throw mjCError(this, "unknown element '%s' in equality constraint", name1_.c_str());
  }
  obj1id = px1->id;

  // find object 2, get id
  if (!name2_.empty()) {
    px2 = m->FindObject(object_type, name2_);
    if (!px2) {
      throw mjCError(this, "unknown element '%s' in equality constraint %d", name2_.c_str(), id);
    }
    obj2id = px2->id;
  } else {
    // object 2 unspecified: set to -1
    obj2id = -1;
    px2 = nullptr;
  }

  // set missing body = world
  if (object_type == mjOBJ_BODY && obj2id == -1) {
    obj2id = 0;
  }

  // make sure the two objects are different
  if (obj1id == obj2id) {
    throw mjCError(this, "element '%s' is repeated in equality constraint %d", name1_.c_str(), id);
  }

  // make sure joints are scalar
  if (type == mjEQ_JOINT) {
    jt1 = ((mjCJoint*)px1)->type;
    jt2 = (px2 ? ((mjCJoint*)px2)->type : mjJNT_HINGE);
    if ((jt1 != mjJNT_HINGE && jt1 != mjJNT_SLIDE) ||
        (jt2 != mjJNT_HINGE && jt2 != mjJNT_SLIDE)) {
      throw mjCError(this, "only HINGE and SLIDE joint allowed in constraint");
    }
  }
}



// compiler
void mjCEquality::Compile(void) {
  CopyFromSpec();

  // find objects
  ResolveReferences(model);

  // make sure flex is not rigid
  if (type == mjEQ_FLEX && model->Flexes()[obj1id]->rigid) {
    throw mjCError(this, "rigid flex '%s' in equality constraint %d", name1_.c_str(), id);
  }
}



//------------------ class mjCTendon implementation ------------------------------------------------

// constructor
mjCTendon::mjCTendon(mjCModel* _model, mjCDef* _def) {
  mjs_defaultTendon(&spec);
  elemtype = mjOBJ_TENDON;

  // clear internal variables
  spec_material_.clear();
  spec_userdata_.clear();
  path.clear();
  matid = -1;

  // reset to default if given
  if (_def) {
    *this = _def->Tendon();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCTendon::mjCTendon(const mjCTendon& other) {
  *this = other;
}



mjCTendon& mjCTendon::operator=(const mjCTendon& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCTendon_*>(this) = static_cast<const mjCTendon_&>(other);
    *static_cast<mjsTendon*>(this) = static_cast<const mjsTendon&>(other);
    for (int i=0; i < other.path.size(); i++) {
      path.push_back(new mjCWrap(*other.path[i]));
      path.back()->tendon = this;
    }
  }
  PointToLocal();
  return *this;
}



bool mjCTendon::is_limited() const {
  return islimited(limited, range);
}
bool mjCTendon::is_actfrclimited() const {
  return islimited(actfrclimited, actfrcrange);
}

void mjCTendon::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.material = &spec_material_;
  spec.userdata = &spec_userdata_;
  spec.info = &info;
  material = nullptr;
  userdata = nullptr;
}



void mjCTendon::NameSpace(const mjCModel* m) {
  mjCBase::NameSpace(m);
  prefix = m->prefix;
  suffix = m->suffix;
}



void mjCTendon::CopyFromSpec() {
  *static_cast<mjsTendon*>(this) = spec;
  material_ = spec_material_;
  userdata_ = spec_userdata_;

  // clear precompiled
  for (int i=0; i < path.size(); i++) {
    if (path[i]->type == mjWRAP_CYLINDER) {
      path[i]->type = mjWRAP_SPHERE;
    }
  }
}



// desctructor
mjCTendon::~mjCTendon() {
  // delete objects allocated here
  for (unsigned int i=0; i < path.size(); i++) {
    delete path[i];
  }

  path.clear();
}



void mjCTendon::SetModel(mjCModel* _model) {
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  for (int i=0; i < path.size(); i++) {
    path[i]->model = _model;
  }
}



// add site as wrap object
void mjCTendon::WrapSite(std::string name, std::string_view info) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->info = info;

  // set parameters, add to path
  wrap->type = mjWRAP_SITE;
  wrap->name = name;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add geom (with side site) as wrap object
void mjCTendon::WrapGeom(std::string name, std::string sidesite, std::string_view info) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->info = info;

  // set parameters, add to path
  wrap->type = mjWRAP_SPHERE;         // replace with cylinder later if needed
  wrap->name = name;
  wrap->sidesite = sidesite;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add joint as wrap object
void mjCTendon::WrapJoint(std::string name, double coef, std::string_view info) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->info = info;

  // set parameters, add to path
  wrap->type = mjWRAP_JOINT;
  wrap->name = name;
  wrap->prm = coef;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add pulley
void mjCTendon::WrapPulley(double divisor, std::string_view info) {
  // create wrap object
  mjCWrap* wrap = new mjCWrap(model, this);
  wrap->info = info;

  // set parameters, add to path
  wrap->type = mjWRAP_PULLEY;
  wrap->prm = divisor;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// get number of wraps
int mjCTendon::NumWraps() const {
  return (int)path.size();
}



// get pointer to specified wrap
const mjCWrap* mjCTendon::GetWrap(int i) const {
  if (i >= 0 && i < (int)path.size()) {
    return path[i];
  }
  return nullptr;
}



void mjCTendon::ResolveReferences(const mjCModel* m) {
  int nfailure = 0;
  int npulley = 0;
  for (int i=0; i < path.size(); i++) {
    std::string pname = path[i]->name;
    std::string psidesite = path[i]->sidesite;
    if (path[i]->type == mjWRAP_PULLEY) {
      npulley++;
    }
    try {
      // look for wrapped element with namespace
      path[i]->name = prefix + pname + suffix;
      path[i]->sidesite = prefix + psidesite + suffix;
      path[i]->ResolveReferences(m);
    } catch(mjCError) {
      // remove namespace from wrap names
      path[i]->name = pname;
      path[i]->sidesite = psidesite;
      path[i]->ResolveReferences(m);
      nfailure++;
    }
  }
  if (nfailure == path.size()-npulley) {
    throw mjCError(this, "tendon '%s' (id = %d): no attached reference found", name.c_str(), id);
  }
  prefix.clear();
  suffix.clear();
}



// compiler
void mjCTendon::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_tendon) {
    throw mjCError(this, "user has more values than nuser_tendon in tendon");
  }
  userdata_.resize(model->nuser_tendon);

  // check for empty path
  int sz = (int)path.size();
  if (!sz) {
    throw mjCError(this,
                   "tendon '%s' (id = %d): path cannot be empty",
                   name.c_str(), id);
  }

  // determine type
  bool spatial = (path[0]->type != mjWRAP_JOINT);

  // require at least two objects in spatial path
  if (spatial && sz < 2) {
    throw mjCError(this, "tendon '%s' (id = %d): spatial path must contain at least two objects",
                   name.c_str(), id);
  }

  // require positive width
  if (spatial && width <= 0) {
    throw mjCError(this, "tendon '%s' (id = %d) must have positive width", name.c_str(), id);
  }

  // compile objects in path
  ResolveReferences(model);

  // check path
  for (int i=0; i < sz; i++) {
    // fixed
    if (!spatial) {
      // make sure all objects are joints
      if (path[i]->type != mjWRAP_JOINT) {
        throw mjCError(this, "tendon '%s' (id = %d): spatial object found in fixed path at pos %d",
                       name.c_str(), id, i);
      }
    }

    // spatial path
    else {
      if (armature < 0) {
        throw mjCError(this,
                       "tendon '%s' (id = %d): tendon armature cannot be negative",
                        name.c_str(), id);
      }

      switch (path[i]->type) {
        case mjWRAP_PULLEY:
          // pulley should not follow other pulley
          if (i > 0 && path[i-1]->type == mjWRAP_PULLEY) {
            throw mjCError(this, "tendon '%s' (id = %d): consecutive pulleys (pos %d)",
                           name.c_str(), id, i);
          }

          // pulley should not be last
          if (i == sz-1) {
            throw mjCError(this, "tendon '%s' (id = %d): path ends with pulley", name.c_str(), id);
          }
          break;

        case mjWRAP_SITE:
          // site needs a neighbor that is not a pulley
          if ((i == 0 || path[i-1]->type == mjWRAP_PULLEY) &&
              (i == sz-1 || path[i+1]->type == mjWRAP_PULLEY)) {
            throw mjCError(this,
                           "tendon '%s' (id = %d): site %d needs a neighbor that is not a pulley",
                           name.c_str(), id, i);
          }

          // site cannot be repeated
          if (i < sz-1 && path[i+1]->type == mjWRAP_SITE && path[i]->obj->id == path[i+1]->obj->id) {
            throw mjCError(this,
                           "tendon '%s' (id = %d): site %d is repeated",
                           name.c_str(), id, i);
          }

          break;

        case mjWRAP_SPHERE:
        case mjWRAP_CYLINDER:
          // geom must be bracketed by sites
          if (i == 0 || i == sz-1 || path[i-1]->type != mjWRAP_SITE || path[i+1]->type != mjWRAP_SITE) {
            throw mjCError(this,
                           "tendon '%s' (id = %d): geom at pos %d not bracketed by sites",
                           name.c_str(), id, i);
          }

          if (armature > 0) {
            throw mjCError(this,
                           "tendon '%s' (id = %d): geom wrapping not supported by tendon armature",
                           name.c_str(), id);
          }

          // mark geoms as non visual
          model->Geoms()[path[i]->obj->id]->SetNotVisual();
          break;

        case mjWRAP_JOINT:
          throw mjCError(this,
                         "tendon '%s (id = %d)': joint wrap found in spatial path at pos %d",
                         name.c_str(), id, i);

        default:
          throw mjCError(this,
                         "tendon '%s (id = %d)': invalid wrap object at pos %d",
                         name.c_str(), id, i);
      }
    }
  }

  // if limited is auto, set to 1 if range is specified, otherwise unlimited
  if (limited == mjLIMITED_AUTO) {
    bool hasrange = !(range[0] == 0 && range[1] == 0);
    checklimited(this, compiler->autolimits, "tendon", "", limited, hasrange);
  }

  // check limits
  if (range[0] >= range[1] && is_limited()) {
    throw mjCError(this, "invalid limits in tendon");
  }

  // if limited is auto, set to 1 if range is specified, otherwise unlimited
  if (actfrclimited == mjLIMITED_AUTO) {
    bool hasactfrcrange = !(actfrcrange[0] == 0 && actfrcrange[1] == 0);
    checklimited(this, compiler->autolimits, "tendon", "", actfrclimited,
                 hasactfrcrange);
  }

  // check actfrclimits
  if (actfrcrange[0] >= actfrcrange[1] && is_actfrclimited()) {
    throw mjCError(this, "invalid actuatorfrcrange in tendon");
  }
  if ((actfrcrange[0] > 0 || actfrcrange[1] < 0) && is_actfrclimited()) {
    throw mjCError(this, "invalid actuatorfrcrange in tendon");
  }

  // check springlength
  if (springlength[0] > springlength[1]) {
    throw mjCError(this, "invalid springlength in tendon");
  }
}



//------------------ class mjCWrap implementation --------------------------------------------------

// constructor
mjCWrap::mjCWrap(mjCModel* _model, mjCTendon* _tendon) {
  elemtype = mjOBJ_UNKNOWN;

  // set model and tendon pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  tendon = _tendon;

  // clear variables
  type = mjWRAP_NONE;
  obj = nullptr;
  sideid = -1;
  prm = 0;
  sidesite.clear();

  // point to local
  PointToLocal();
}



mjCWrap::mjCWrap(const mjCWrap& other) {
  *this = other;
}



mjCWrap& mjCWrap::operator=(const mjCWrap& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCWrap_*>(this) = static_cast<const mjCWrap_&>(other);
    *static_cast<mjsWrap*>(this) = static_cast<const mjsWrap&>(other);
    obj = nullptr;
  }
  PointToLocal();
  return *this;
}



void mjCWrap::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.info = &info;
}



void mjCWrap::NameSpace(const mjCModel* m) {
  name = m->prefix + name + m->suffix;
  if (!sidesite.empty()) {
    sidesite = m->prefix + sidesite + m->suffix;
  }
}



void mjCWrap::ResolveReferences(const mjCModel* m) {
  mjCBase *pside;

  // handle wrap object types
  switch (type) {
    case mjWRAP_JOINT:                        // joint
      // find joint by name
      obj = m->FindObject(mjOBJ_JOINT, name);
      if (!obj) {
        throw mjCError(this,
                       "joint '%s' not found in tendon %d, wrap %d",
                       name.c_str(), tendon->id, id);
      }

      break;

    case mjWRAP_SPHERE:                       // geom (cylinder type set here)
      // find geom by name
      obj = m->FindObject(mjOBJ_GEOM, name);
      if (!obj) {
        throw mjCError(this,
                       "geom '%s' not found in tendon %d, wrap %d",
                       name.c_str(), tendon->id, id);
      }

      // set/check geom type
      if (((mjCGeom*)obj)->type == mjGEOM_CYLINDER) {
        type = mjWRAP_CYLINDER;
      } else if (((mjCGeom*)obj)->type != mjGEOM_SPHERE) {
        throw mjCError(this,
                       "geom '%s' in tendon %d, wrap %d is not sphere or cylinder",
                       name.c_str(), tendon->id, id);
      }

      // process side site
      if (!sidesite.empty()) {
        // find site by name
        pside = m->FindObject(mjOBJ_SITE, sidesite);
        if (!pside) {
          throw mjCError(this,
                         "side site '%s' not found in tendon %d, wrap %d",
                         sidesite.c_str(), tendon->id, id);
        }

        // save side site id
        sideid = pside->id;
      }
      break;

    case mjWRAP_PULLEY:                       // pulley
      // make sure divisor is non-negative
      if (prm < 0) {
        throw mjCError(this,
                       "pulley has negative divisor in tendon %d, wrap %d",
                       0, tendon->id, id);
      }

      break;

    case mjWRAP_SITE:                         // site
      // find site by name
      obj = m->FindObject(mjOBJ_SITE, name);
      if (!obj) {
        throw mjCError(this, "site '%s' not found in wrap %d", name.c_str(), id);
      }
      break;

    default:                                  // SHOULD NOT OCCUR
      throw mjCError(this, "unknown wrap type in tendon %d, wrap %d", 0, tendon->id, id);
  }
}



//------------------ class mjCActuator implementation ----------------------------------------------

// initialize defaults
mjCActuator::mjCActuator(mjCModel* _model, mjCDef* _def) {
  mjs_defaultActuator(&spec);
  elemtype = mjOBJ_ACTUATOR;

  // clear private variables
  ptarget = nullptr;
  spec_target_.clear();
  spec_slidersite_.clear();
  spec_refsite_.clear();
  spec_userdata_.clear();
  trnid[0] = trnid[1] = -1;

  // reset to default if given
  if (_def) {
    *this = _def->Actuator();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // in case this actuator is not compiled
  CopyFromSpec();

  // point to local
  PointToLocal();

  // no previous state when an actuator is created
  actadr_ = -1;
  actdim_ = -1;
}



mjCActuator::mjCActuator(const mjCActuator& other) {
  *this = other;
}



mjCActuator& mjCActuator::operator=(const mjCActuator& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCActuator_*>(this) = static_cast<const mjCActuator_&>(other);
    *static_cast<mjsActuator*>(this) = static_cast<const mjsActuator&>(other);
    ptarget = nullptr;
  }
  PointToLocal();
  return *this;
}



void mjCActuator::ForgetKeyframes() {
  act_.clear();
  ctrl_.clear();
}



bool mjCActuator::is_ctrllimited() const {
  return islimited(ctrllimited, ctrlrange);
}
bool mjCActuator::is_forcelimited() const {
  return islimited(forcelimited, forcerange);
}
bool mjCActuator::is_actlimited() const {
  return islimited(actlimited, actrange);
}



std::vector<mjtNum>& mjCActuator::act(const std::string& state_name) {
  if (act_.find(state_name) == act_.end()) {
    act_[state_name] = std::vector<mjtNum>(model->nu, mjNAN);
  }
  return act_.at(state_name);
}



mjtNum& mjCActuator::ctrl(const std::string& state_name) {
  if (ctrl_.find(state_name) == ctrl_.end()) {
    ctrl_[state_name] = mjNAN;
  }
  return ctrl_.at(state_name);
}



void mjCActuator::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.userdata = &spec_userdata_;
  spec.target = &spec_target_;
  spec.refsite = &spec_refsite_;
  spec.slidersite = &spec_slidersite_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = &plugin_instance_name;
  spec.info = &info;
  userdata = nullptr;
  target = nullptr;
  refsite = nullptr;
  slidersite = nullptr;
}



void mjCActuator::NameSpace(const mjCModel* m) {
  mjCBase::NameSpace(m);
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }
  if (!spec_target_.empty()) {
    spec_target_ = m->prefix + spec_target_ + m->suffix;
  }
  if (!spec_refsite_.empty()) {
    spec_refsite_ = m->prefix + spec_refsite_ + m->suffix;
  }
  if (!spec_slidersite_.empty()) {
    spec_slidersite_ = m->prefix + spec_slidersite_ + m->suffix;
  }
}



void mjCActuator::CopyFromSpec() {
  *static_cast<mjsActuator*>(this) = spec;
  userdata_ = spec_userdata_;
  target_ = spec_target_;
  refsite_ = spec_refsite_;
  slidersite_ = spec_slidersite_;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;
}



void mjCActuator::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



void mjCActuator::ResolveReferences(const mjCModel* m) {
  switch (trntype) {
    case mjTRN_JOINT:
    case mjTRN_JOINTINPARENT:
      // get joint
      ptarget = m->FindObject(mjOBJ_JOINT, target_);
      if (!ptarget) {
        throw mjCError(this,
                       "unknown transmission target '%s' for actuator id = %d", target_.c_str(), id);
      }
      break;

    case mjTRN_SLIDERCRANK:
      // get slidersite, copy in trnid[1]
      if (slidersite_.empty()) {
        throw mjCError(this, "missing base site for slider-crank '%s' (id = %d)", name.c_str(), id);
      }
      ptarget = m->FindObject(mjOBJ_SITE, slidersite_);
      if (!ptarget) {
        throw mjCError(this, "base site '%s' not found for actuator %d", slidersite_.c_str(), id);
      }
      trnid[1] = ptarget->id;

      // check cranklength
      if (cranklength <= 0) {
        throw mjCError(this,
                       "crank length must be positive in actuator '%s' (id = %d)", name.c_str(), id);
      }

      // proceed with regular target
      ptarget = m->FindObject(mjOBJ_SITE, target_);
      break;

    case mjTRN_TENDON:
      // get tendon
      ptarget = m->FindObject(mjOBJ_TENDON, target_);
      break;

    case mjTRN_SITE:
      // get refsite, copy into trnid[1]
      if (!refsite_.empty()) {
        ptarget = m->FindObject(mjOBJ_SITE, refsite_);
        if (!ptarget) {
          throw mjCError(this, "reference site '%s' not found for actuator %d", refsite_.c_str(), id);
        }
        trnid[1] = ptarget->id;
      }

      // proceed with regular site target
      ptarget = m->FindObject(mjOBJ_SITE, target_);
      break;

    case mjTRN_BODY:
      // get body
      ptarget = m->FindObject(mjOBJ_BODY, target_);
      break;

    default:
      throw mjCError(this, "invalid transmission type in actuator");
  }

  // assign and check
  if (!ptarget) {
    throw mjCError(this, "transmission target '%s' not found in actuator %d", target_.c_str(), id);
  } else {
    trnid[0] = ptarget->id;
  }
}



// compiler
void mjCActuator::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_actuator) {
    throw mjCError(this, "user has more values than nuser_actuator in actuator '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_actuator);

  // check for missing target name
  if (target_.empty()) {
    throw mjCError(this,
                   "missing transmission target for actuator");
  }

  // find transmission target in object arrays
  ResolveReferences(model);

  // handle inheritrange
  if (gaintype == mjGAIN_FIXED && biastype == mjBIAS_AFFINE &&
      gainprm[0] == -biasprm[1] && inheritrange > 0) {
    // semantic of actuator is the same as transmission, inheritrange is applicable
    double* range;
    if (dyntype == mjDYN_NONE || dyntype == mjDYN_FILTEREXACT) {
      // position actuator
      range = ctrlrange;
    } else if (dyntype == mjDYN_INTEGRATOR) {
      // intvelocity actuator
      range = actrange;
    } else {
      throw mjCError(this, "inheritrange only available for position "
                     "and intvelocity actuators");
    }

    const double* target_range;
    if (trntype == mjTRN_JOINT) {
      mjCJoint* pjnt = (mjCJoint*) ptarget;
      if (pjnt->spec.type != mjJNT_HINGE && pjnt->spec.type != mjJNT_SLIDE) {
        throw mjCError(this, "inheritrange can only be used with hinge and slide joints, "
                       "actuator");
      }
      target_range = pjnt->get_range();
    } else if (trntype == mjTRN_TENDON) {
      mjCTendon* pten = (mjCTendon*) ptarget;
      target_range = pten->get_range();
    } else {
      throw mjCError(this, "inheritrange can only be used with joint and tendon transmission, "
                     "actuator");
    }

    if (target_range[0] == target_range[1]) {
      throw mjCError(this, "inheritrange used but target '%s' has no range defined in actuator %d",
                     target_.c_str(), id);
    }

    // set range automatically
    double mean   = 0.5*(target_range[1] + target_range[0]);
    double radius = 0.5*(target_range[1] - target_range[0]) * inheritrange;
    range[0] = mean - radius;
    range[1] = mean + radius;
  }

  // if limited is auto, check for inconsistency wrt to autolimits
  if (forcelimited == mjLIMITED_AUTO) {
    bool hasrange = !(forcerange[0] == 0 && forcerange[1] == 0);
    checklimited(this, compiler->autolimits, "actuator", "force", forcelimited, hasrange);
  }
  if (ctrllimited == mjLIMITED_AUTO) {
    bool hasrange = !(ctrlrange[0] == 0 && ctrlrange[1] == 0);
    checklimited(this, compiler->autolimits, "actuator", "ctrl", ctrllimited, hasrange);
  }
  if (actlimited == mjLIMITED_AUTO) {
    bool hasrange = !(actrange[0] == 0 && actrange[1] == 0);
    checklimited(this, compiler->autolimits, "actuator", "act", actlimited, hasrange);
  }

  // check limits
  if (forcerange[0] >= forcerange[1] && is_forcelimited()) {
    throw mjCError(this, "invalid force range for actuator");
  }
  if (ctrlrange[0] >= ctrlrange[1] && is_ctrllimited()) {
    throw mjCError(this, "invalid control range for actuator");
  }
  if (actrange[0] >= actrange[1] && is_actlimited()) {
    throw mjCError(this, "invalid actrange for actuator");
  }
  if (is_actlimited() && dyntype == mjDYN_NONE) {
    throw mjCError(this, "actrange specified but dyntype is 'none' in actuator");
  }

  // check and set actdim
  if (!plugin.active) {
    if (actdim > 1 && dyntype != mjDYN_USER) {
      throw mjCError(this, "actdim > 1 is only allowed for dyntype 'user' in actuator");
    }
    if (actdim == 1 && dyntype == mjDYN_NONE) {
      throw mjCError(this, "invalid actdim 1 in stateless actuator");
    }
    if (actdim == 0 && dyntype != mjDYN_NONE) {
      throw mjCError(this, "invalid actdim 0 in stateful actuator");
    }
  }

  // set actdim
  if (actdim < 0) {
    actdim = (dyntype != mjDYN_NONE);
  }

  // check muscle parameters
  for (int i=0; i < 2; i++) {
    // select gain or bias
    double* prm = NULL;
    if (i == 0 && gaintype == mjGAIN_MUSCLE) {
      prm = gainprm;
    } else if (i == 1 && biastype == mjBIAS_MUSCLE) {
      prm = biasprm;
    }

    // nothing to check
    if (!prm) {
      continue;
    }

    // range
    if (prm[0] >= prm[1]) {
      throw mjCError(this, "range[0]<range[1] required in muscle");
    }

    // lmin<1<lmax
    if (prm[4] >= 1 || prm[5] <= 1) {
      throw mjCError(this, "lmin<1<lmax required in muscle");
    }

    // scale, vmax, fpmax, fvmax>0
    if (prm[3] <= 0 || prm[6] <= 0 || prm[7] <= 0 || prm[8] <= 0) {
      throw mjCError(this,
                     "positive scale, vmax, fpmax, fvmax required in muscle '%s' (id = %d)",
                     name.c_str(), id);
    }
  }

  // plugin
  if (plugin.active) {
    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw mjCError(
              this, "neither 'plugin' nor 'instance' is specified for actuator '%s', (id = %d)",
              name.c_str(), id);
    }

    mjCPlugin* plugin_instance = static_cast<mjCPlugin*>(plugin.element);
    model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
    plugin.element = plugin_instance;
    const mjpPlugin* pplugin = mjp_getPluginAtSlot(plugin_instance->plugin_slot);
    if (!(pplugin->capabilityflags & mjPLUGIN_ACTUATOR)) {
      throw mjCError(this, "plugin '%s' does not support actuators", pplugin->name);
    }
  }
}



//------------------ class mjCSensor implementation ------------------------------------------------

// initialize defaults
mjCSensor::mjCSensor(mjCModel* _model) {
  mjs_defaultSensor(&spec);
  elemtype = mjOBJ_SENSOR;

  // set model
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear private variables
  spec_objname_.clear();
  spec_refname_.clear();
  spec_userdata_.clear();
  obj = nullptr;
  ref = nullptr;
  refid = -1;

  // in case this sensor is not compiled
  CopyFromSpec();

  // point to local
  PointToLocal();
}



mjCSensor::mjCSensor(const mjCSensor& other) {
  *this = other;
}



mjCSensor& mjCSensor::operator=(const mjCSensor& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCSensor_*>(this) = static_cast<const mjCSensor_&>(other);
    *static_cast<mjsSensor*>(this) = static_cast<const mjsSensor&>(other);
    obj = nullptr;
    ref = nullptr;
  }
  PointToLocal();
  return *this;
}



void mjCSensor::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.userdata = &spec_userdata_;
  spec.objname = &spec_objname_;
  spec.refname = &spec_refname_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = &plugin_instance_name;
  spec.info = &info;
  userdata = nullptr;
  objname = nullptr;
  refname = nullptr;
}



void mjCSensor::NameSpace(const mjCModel* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }
  prefix = m->prefix;
  suffix = m->suffix;
}



void mjCSensor::CopyFromSpec() {
  *static_cast<mjsSensor*>(this) = spec;
  userdata_ = spec_userdata_;
  objname_ = spec_objname_;
  refname_ = spec_refname_;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;
}



void mjCSensor::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



void mjCSensor::ResolveReferences(const mjCModel* m) {
  obj = nullptr;
  ref = nullptr;
  objname_ = prefix + objname_ + suffix;
  refname_ = prefix + refname_ + suffix;

  // get references using the namespace
  if (objtype != mjOBJ_UNKNOWN) {
    obj = m->FindObject(objtype, objname_);
  }
  if (reftype != mjOBJ_UNKNOWN) {
    ref = m->FindObject(reftype, refname_);
  }

  // if failure and both were requested, use namespace only on one
  if (objtype != mjOBJ_UNKNOWN && reftype != mjOBJ_UNKNOWN && !obj && ref) {
    objname_ = spec_objname_;
    obj = m->FindObject(objtype, objname_);
  }
  if (objtype != mjOBJ_UNKNOWN && reftype != mjOBJ_UNKNOWN && obj && !ref) {
    refname_ = spec_refname_;
    ref = m->FindObject(reftype, refname_);
  }

  // get objid from objtype and objname
  if (objtype != mjOBJ_UNKNOWN) {
    // check for missing object name
    if (objname_.empty()) {
      throw mjCError(this, "missing name of sensorized object in sensor");
    }

    // find name
    if (!obj) {
      throw mjCError(this, "unrecognized name '%s' of sensorized object", objname_.c_str());
    }

    // if geom mark it as non visual
    if (objtype == mjOBJ_GEOM) {
      ((mjCGeom*)obj)->SetNotVisual();
    }

  } else if (type != mjSENS_E_POTENTIAL &&
             type != mjSENS_E_KINETIC   &&
             type != mjSENS_CLOCK       &&
             type != mjSENS_PLUGIN      &&
             type != mjSENS_USER) {
    throw mjCError(this, "invalid type in sensor");
  }

  // get refid from reftype and refname
  if (reftype != mjOBJ_UNKNOWN) {
    // check for missing object name
    if (refname_.empty()) {
      throw mjCError(this, "missing name of reference frame object in sensor");
    }

    // find name
    if (!ref) {
      throw mjCError(this, "unrecognized name '%s' of object", refname_.c_str());
    }

    // must be attached to object with spatial frame
    if (reftype != mjOBJ_BODY && reftype != mjOBJ_XBODY &&
        reftype != mjOBJ_GEOM && reftype != mjOBJ_SITE && reftype != mjOBJ_CAMERA) {
      throw mjCError(this,
                     "reference frame object must be (x)body, geom, site or camera in sensor");
    }

    // get sensorized object id
    refid = ref->id;
  }

  spec_objname_ = objname_;
  spec_refname_ = refname_;
  prefix.clear();
  suffix.clear();
}



// compiler
void mjCSensor::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_sensor) {
    throw mjCError(this, "user has more values than nuser_sensor in sensor");
  }
  userdata_.resize(model->nuser_sensor);

  // require non-negative noise
  if (noise < 0) {
    throw mjCError(this, "negative noise in sensor");
  }

  // require non-negative cutoff
  if (cutoff < 0) {
    throw mjCError(this, "negative cutoff in sensor");
  }

  // Find referenced object
  ResolveReferences(model);

  // process according to sensor type
  switch (type) {
    case mjSENS_TOUCH:
    case mjSENS_ACCELEROMETER:
    case mjSENS_VELOCIMETER:
    case mjSENS_GYRO:
    case mjSENS_FORCE:
    case mjSENS_TORQUE:
    case mjSENS_MAGNETOMETER:
    case mjSENS_RANGEFINDER:
    case mjSENS_CAMPROJECTION:
      // must be attached to site
      if (objtype != mjOBJ_SITE) {
        throw mjCError(this, "sensor must be attached to site");
      }

      // set dim and datatype
      if (type == mjSENS_TOUCH || type == mjSENS_RANGEFINDER) {
        dim = 1;
        datatype = mjDATATYPE_POSITIVE;
      } else if (type == mjSENS_CAMPROJECTION) {
        dim = 2;
        datatype = mjDATATYPE_REAL;
      } else {
        dim = 3;
        datatype = mjDATATYPE_REAL;
      }

      // set stage
      if (type == mjSENS_MAGNETOMETER || type == mjSENS_RANGEFINDER || type == mjSENS_CAMPROJECTION) {
        needstage = mjSTAGE_POS;
      } else if (type == mjSENS_GYRO || type == mjSENS_VELOCIMETER) {
        needstage = mjSTAGE_VEL;
      } else {
        needstage = mjSTAGE_ACC;
      }

      // check for camera resolution for camera projection sensor
      if (type == mjSENS_CAMPROJECTION) {
        mjCCamera* camref = (mjCCamera*)ref;
        if (!camref->resolution[0] || !camref->resolution[1]) {
          throw mjCError(this, "camera projection sensor requires camera resolution");
        }
      }
      break;

    case mjSENS_JOINTPOS:
    case mjSENS_JOINTVEL:
    case mjSENS_JOINTACTFRC:
      // must be attached to joint
      if (objtype != mjOBJ_JOINT) {
        throw mjCError(this, "sensor must be attached to joint");
      }

      // make sure joint is slide or hinge
      if (((mjCJoint*)obj)->type != mjJNT_SLIDE && ((mjCJoint*)obj)->type != mjJNT_HINGE) {
        throw mjCError(this, "joint must be slide or hinge in sensor");
      }

      // set
      dim = 1;
      datatype = mjDATATYPE_REAL;
      if (type == mjSENS_JOINTPOS) {
        needstage = mjSTAGE_POS;
      } else if (type == mjSENS_JOINTVEL) {
        needstage = mjSTAGE_VEL;
      } else if (type == mjSENS_JOINTACTFRC) {
        needstage = mjSTAGE_ACC;
      }
      break;

  case mjSENS_TENDONACTFRC:
    // must be attached to tendon
    if (objtype != mjOBJ_TENDON) {
      throw mjCError(this, "sensor must be attached to tendon");
    }

    // set
    dim = 1;
    datatype = mjDATATYPE_REAL;
    needstage = mjSTAGE_ACC;
    break;

    case mjSENS_TENDONPOS:
    case mjSENS_TENDONVEL:
      // must be attached to tendon
      if (objtype != mjOBJ_TENDON) {
        throw mjCError(this, "sensor must be attached to tendon");
      }

      // set
      dim = 1;
      datatype = mjDATATYPE_REAL;
      if (type == mjSENS_TENDONPOS) {
        needstage = mjSTAGE_POS;
      } else {
        needstage = mjSTAGE_VEL;
      }
      break;

    case mjSENS_ACTUATORPOS:
    case mjSENS_ACTUATORVEL:
    case mjSENS_ACTUATORFRC:
      // must be attached to actuator
      if (objtype != mjOBJ_ACTUATOR) {
        throw mjCError(this, "sensor must be attached to actuator");
      }

      // set
      dim = 1;
      datatype = mjDATATYPE_REAL;
      if (type == mjSENS_ACTUATORPOS) {
        needstage = mjSTAGE_POS;
      } else if (type == mjSENS_ACTUATORVEL) {
        needstage = mjSTAGE_VEL;
      } else {
        needstage = mjSTAGE_ACC;
      }
      break;

    case mjSENS_BALLQUAT:
    case mjSENS_BALLANGVEL:
      // must be attached to joint
      if (objtype != mjOBJ_JOINT) {
        throw mjCError(this, "sensor must be attached to joint");
      }

      // make sure joint is ball
      if (((mjCJoint*)obj)->type != mjJNT_BALL) {
        throw mjCError(this, "joint must be ball in sensor");
      }

      // set
      if (type == mjSENS_BALLQUAT) {
        dim = 4;
        datatype = mjDATATYPE_QUATERNION;
        needstage = mjSTAGE_POS;
      } else {
        dim = 3;
        datatype = mjDATATYPE_REAL;
        needstage = mjSTAGE_VEL;
      }
      break;

    case mjSENS_JOINTLIMITPOS:
    case mjSENS_JOINTLIMITVEL:
    case mjSENS_JOINTLIMITFRC:
      // must be attached to joint
      if (objtype != mjOBJ_JOINT) {
        throw mjCError(this, "sensor must be attached to joint");
      }

      // make sure joint has limit
      if (!((mjCJoint*)obj)->is_limited()) {
        throw mjCError(this, "joint must be limited in sensor");
      }

      // set
      dim = 1;
      datatype = mjDATATYPE_REAL;
      if (type == mjSENS_JOINTLIMITPOS) {
        needstage = mjSTAGE_POS;
      } else if (type == mjSENS_JOINTLIMITVEL) {
        needstage = mjSTAGE_VEL;
      } else {
        needstage = mjSTAGE_ACC;
      }
      break;

    case mjSENS_TENDONLIMITPOS:
    case mjSENS_TENDONLIMITVEL:
    case mjSENS_TENDONLIMITFRC:
      // must be attached to tendon
      if (objtype != mjOBJ_TENDON) {
        throw mjCError(this, "sensor must be attached to tendon");
      }

      // make sure tendon has limit
      if (!((mjCTendon*)obj)->is_limited()) {
        throw mjCError(this, "tendon must be limited in sensor");
      }

      // set
      dim = 1;
      datatype = mjDATATYPE_REAL;
      if (type == mjSENS_TENDONLIMITPOS) {
        needstage = mjSTAGE_POS;
      } else if (type == mjSENS_TENDONLIMITVEL) {
        needstage = mjSTAGE_VEL;
      } else {
        needstage = mjSTAGE_ACC;
      }
      break;

    case mjSENS_FRAMEPOS:
    case mjSENS_FRAMEQUAT:
    case mjSENS_FRAMEXAXIS:
    case mjSENS_FRAMEYAXIS:
    case mjSENS_FRAMEZAXIS:
    case mjSENS_FRAMELINVEL:
    case mjSENS_FRAMEANGVEL:
    case mjSENS_FRAMELINACC:
    case mjSENS_FRAMEANGACC:
      // must be attached to object with spatial frame
      if (objtype != mjOBJ_BODY && objtype != mjOBJ_XBODY &&
          objtype != mjOBJ_GEOM && objtype != mjOBJ_SITE && objtype != mjOBJ_CAMERA) {
        throw mjCError(this, "sensor must be attached to (x)body, geom, site or camera");
      }

      // set dim
      if (type == mjSENS_FRAMEQUAT) {
        dim = 4;
      } else {
        dim = 3;
      }

      // set datatype
      if (type == mjSENS_FRAMEQUAT) {
        datatype = mjDATATYPE_QUATERNION;
      } else if (type == mjSENS_FRAMEXAXIS ||
                 type == mjSENS_FRAMEYAXIS ||
                 type == mjSENS_FRAMEZAXIS) {
        datatype = mjDATATYPE_AXIS;
      } else {
        datatype = mjDATATYPE_REAL;
      }

      // set needstage
      if (type == mjSENS_FRAMELINACC || type == mjSENS_FRAMEANGACC) {
        needstage = mjSTAGE_ACC;
      } else if (type == mjSENS_FRAMELINVEL || type == mjSENS_FRAMEANGVEL) {
        needstage = mjSTAGE_VEL;
      } else {
        needstage = mjSTAGE_POS;
      }
      break;

    case mjSENS_SUBTREECOM:
    case mjSENS_SUBTREELINVEL:
    case mjSENS_SUBTREEANGMOM:
      // must be attached to body
      if (objtype != mjOBJ_BODY) {
        throw mjCError(this, "sensor must be attached to body");
      }

      // set
      dim = 3;
      datatype = mjDATATYPE_REAL;
      if (type == mjSENS_SUBTREECOM) {
        needstage = mjSTAGE_POS;
      } else {
        needstage = mjSTAGE_VEL;
      }
      break;

    case mjSENS_GEOMDIST:
    case mjSENS_GEOMNORMAL:
    case mjSENS_GEOMFROMTO:
      // must be attached to body or geom
      if ((objtype != mjOBJ_BODY && objtype != mjOBJ_GEOM) ||
          (reftype != mjOBJ_BODY && reftype != mjOBJ_GEOM)) {
        throw mjCError(this, "sensor must be attached to body or geom");
      }

      // objects must be different
      if (objtype == reftype && obj == ref) {
        throw mjCError(this, "1st body/geom must be different from 2nd body/geom");
      }

      // height fields are not necessarily convex and are not yet supported
      if ((objtype == mjOBJ_GEOM && static_cast<mjCGeom*>(obj)->Type() == mjGEOM_HFIELD) ||
          (reftype == mjOBJ_GEOM && static_cast<mjCGeom*>(ref)->Type() == mjGEOM_HFIELD)) {
        throw mjCError(this, "height fields are not supported in geom distance sensors");
      }

      // set
      needstage = mjSTAGE_POS;
      if (type == mjSENS_GEOMDIST) {
        dim = 1;
        datatype = mjDATATYPE_POSITIVE;
      } else if (type == mjSENS_GEOMNORMAL) {
        dim = 3;
        datatype = mjDATATYPE_AXIS;
      } else {
        dim = 6;
        datatype = mjDATATYPE_REAL;
      }
      break;

    case mjSENS_E_POTENTIAL:
    case mjSENS_E_KINETIC:
    case mjSENS_CLOCK:
      dim = 1;
      needstage = mjSTAGE_POS;
      datatype = mjDATATYPE_REAL;
      break;

    case mjSENS_USER:
      // check for negative dim
      if (dim < 0) {
        throw mjCError(this, "sensor dim must be positive in sensor");
      }

      // make sure dim is consistent with datatype
      if (datatype == mjDATATYPE_AXIS && dim != 3) {
        throw mjCError(this,
                       "datatype AXIS requires dim=3 in sensor");
      }
      if (datatype == mjDATATYPE_QUATERNION && dim != 4) {
        throw mjCError(this, "datatype QUATERNION requires dim=4 in sensor");
      }
      break;

    case mjSENS_PLUGIN:
      dim = 0; // to be filled in by the plugin later
      datatype = mjDATATYPE_REAL; // no noise added to plugin sensors, this attribute is unused

      if (plugin_name.empty() && plugin_instance_name.empty()) {
        throw mjCError(this, "neither 'plugin' nor 'instance' is specified for sensor");
      }

      // resolve plugin instance, or create one if using the "plugin" attribute shortcut
      {
        mjCPlugin* plugin_instance = static_cast<mjCPlugin*>(plugin.element);
        model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
        plugin.element = plugin_instance;
        const mjpPlugin* pplugin = mjp_getPluginAtSlot(plugin_instance->plugin_slot);
        if (!(pplugin->capabilityflags & mjPLUGIN_SENSOR)) {
          throw mjCError(this, "plugin '%s' does not support sensors", pplugin->name);
        }
        needstage = static_cast<mjtStage>(pplugin->needstage);
      }

      break;

    default:
      throw mjCError(this, "invalid type in sensor '%s' (id = %d)", name.c_str(), id);
  }

  // check cutoff for incompatible data types
  if (cutoff > 0 && (datatype == mjDATATYPE_QUATERNION ||
                     (datatype == mjDATATYPE_AXIS && type != mjSENS_GEOMNORMAL))) {
    throw mjCError(this, "cutoff applied to axis or quaternion datatype in sensor");
  }
}



//------------------ class mjCNumeric implementation -----------------------------------------------

// constructor
mjCNumeric::mjCNumeric(mjCModel* _model) {
  mjs_defaultNumeric(&spec);
  elemtype = mjOBJ_NUMERIC;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  spec_data_.clear();

  // point to local
  PointToLocal();

  // in case this numeric is not compiled
  CopyFromSpec();
}



mjCNumeric::mjCNumeric(const mjCNumeric& other) {
  *this = other;
}



mjCNumeric& mjCNumeric::operator=(const mjCNumeric& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCNumeric_*>(this) = static_cast<const mjCNumeric_&>(other);
    *static_cast<mjsNumeric*>(this) = static_cast<const mjsNumeric&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCNumeric::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.data = &spec_data_;
  spec.info = &info;
  data = nullptr;
}



void mjCNumeric::CopyFromSpec() {
  *static_cast<mjsNumeric*>(this) = spec;
  data_ = spec_data_;
}



// destructor
mjCNumeric::~mjCNumeric() {
  spec_data_.clear();
  data_.clear();
}



// compiler
void mjCNumeric::Compile(void) {
  CopyFromSpec();

  // check for size conflict
  if (size && !data_.empty() && size < (int)data_.size()) {
    throw mjCError(this,
                   "numeric '%s' (id = %d): specified size smaller than initialization array",
                   name.c_str(), id);
  }

  // set size if left unspecified
  if (!size) {
    size = (int)data_.size();
  }

  // size cannot be zero
  if (!size) {
    throw mjCError(this, "numeric '%s' (id = %d): size cannot be zero", name.c_str(), id);
  }
}



//------------------ class mjCText implementation --------------------------------------------------

// constructor
mjCText::mjCText(mjCModel* _model) {
  mjs_defaultText(&spec);
  elemtype = mjOBJ_TEXT;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  spec_data_.clear();

  // point to local
  PointToLocal();

  // in case this text is not compiled
  CopyFromSpec();
}



mjCText::mjCText(const mjCText& other) {
  *this = other;
}



mjCText& mjCText::operator=(const mjCText& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCText_*>(this) = static_cast<const mjCText_&>(other);
    *static_cast<mjsText*>(this) = static_cast<const mjsText&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCText::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.data = &spec_data_;
  spec.info = &info;
  data = nullptr;
}



void mjCText::CopyFromSpec() {
  *static_cast<mjsText*>(this) = spec;
  data_ = spec_data_;
}



// destructor
mjCText::~mjCText() {
  data_.clear();
  spec_data_.clear();
}



// compiler
void mjCText::Compile(void) {
  CopyFromSpec();

  // size cannot be zero
  if (data_.empty()) {
    throw mjCError(this, "text '%s' (id = %d): size cannot be zero", name.c_str(), id);
  }
}



//------------------ class mjCTuple implementation -------------------------------------------------

// constructor
mjCTuple::mjCTuple(mjCModel* _model) {
  mjs_defaultTuple(&spec);
  elemtype = mjOBJ_TUPLE;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  spec_objtype_.clear();
  spec_objname_.clear();
  spec_objprm_.clear();
  obj.clear();

  // point to local
  PointToLocal();

  // in case this tuple is not compiled
  CopyFromSpec();
}



mjCTuple::mjCTuple(const mjCTuple& other) {
  *this = other;
}



mjCTuple& mjCTuple::operator=(const mjCTuple& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCTuple_*>(this) = static_cast<const mjCTuple_&>(other);
    *static_cast<mjsTuple*>(this) = static_cast<const mjsTuple&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCTuple::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.objtype = (mjIntVec*)&spec_objtype_;
  spec.objname = &spec_objname_;
  spec.objprm = &spec_objprm_;
  spec.info = &info;
  objname = nullptr;
  objprm = nullptr;
}



void mjCTuple::NameSpace(const mjCModel* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  for (int i=0; i < spec_objname_.size(); i++) {
    spec_objname_[i] = m->prefix + spec_objname_[i] + m->suffix;
  }
}



void mjCTuple::CopyFromSpec() {
  *static_cast<mjsTuple*>(this) = spec;
  objtype_ = spec_objtype_;
  objname_ = spec_objname_;
  objprm_ = spec_objprm_;
  objtype = (mjIntVec*)&objtype_;
}



// destructor
mjCTuple::~mjCTuple() {
  objtype_.clear();
  objname_.clear();
  objprm_.clear();
  spec_objtype_.clear();
  spec_objname_.clear();
  spec_objprm_.clear();
  obj.clear();
}



void mjCTuple::ResolveReferences(const mjCModel* m) {
  // check for empty tuple
  if (objtype_.empty()) {
    throw mjCError(this, "tuple '%s' (id = %d) is empty", name.c_str(), id);
  }

  // check for size conflict
  if (objtype_.size() != objname_.size() || objtype_.size() != objprm_.size()) {
    throw mjCError(this,
                   "tuple '%s' (id = %d) has object arrays with different sizes", name.c_str(), id);
  }

  // resize objid to correct size
  obj.resize(objtype_.size());

  // find objects, fill in ids
  for (int i=0; i < objtype_.size(); i++) {
    // find object by type and name
    mjCBase* res = m->FindObject(objtype_[i], objname_[i]);
    if (!res) {
      throw mjCError(this, "unrecognized object '%s' in tuple %d", objname_[i].c_str(), id);
    }

    // if geom mark it as non visual
    if (objtype_[i] == mjOBJ_GEOM) {
      ((mjCGeom*)res)->SetNotVisual();
    }

    // assign id
    obj[i] = res;
  }
}



// compiler
void mjCTuple::Compile(void) {
  CopyFromSpec();
  ResolveReferences(model);
}



//------------------ class mjCKey implementation ---------------------------------------------------

// constructor
mjCKey::mjCKey(mjCModel* _model) {
  mjs_defaultKey(&spec);
  elemtype = mjOBJ_KEY;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  spec_qpos_.clear();
  spec_qvel_.clear();
  spec_act_.clear();
  spec_mpos_.clear();
  spec_mquat_.clear();
  spec_ctrl_.clear();

  // point to local
  PointToLocal();

  // in case this keyframe is not compiled
  CopyFromSpec();
}



mjCKey::mjCKey(const mjCKey& other) {
  *this = other;
}



mjCKey& mjCKey::operator=(const mjCKey& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCKey_*>(this) = static_cast<const mjCKey_&>(other);
    *static_cast<mjsKey*>(this) = static_cast<const mjsKey&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCKey::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.qpos = &spec_qpos_;
  spec.qvel = &spec_qvel_;
  spec.act = &spec_act_;
  spec.mpos = &spec_mpos_;
  spec.mquat = &spec_mquat_;
  spec.ctrl = &spec_ctrl_;
  spec.info = &info;
  qpos = nullptr;
  qvel = nullptr;
  act = nullptr;
  mpos = nullptr;
  mquat = nullptr;
  ctrl = nullptr;
}



void mjCKey::CopyFromSpec() {
  *static_cast<mjsKey*>(this) = spec;
  qpos_ = spec_qpos_;
  qvel_ = spec_qvel_;
  act_ = spec_act_;
  mpos_ = spec_mpos_;
  mquat_ = spec_mquat_;
  ctrl_ = spec_ctrl_;
}



// destructor
mjCKey::~mjCKey() {
  qpos_.clear();
  qvel_.clear();
  act_.clear();
  mpos_.clear();
  mquat_.clear();
  ctrl_.clear();
  spec_qpos_.clear();
  spec_qvel_.clear();
  spec_act_.clear();
  spec_mpos_.clear();
  spec_mquat_.clear();
  spec_ctrl_.clear();
}



// compiler
void mjCKey::Compile(const mjModel* m) {
  CopyFromSpec();

  // qpos: allocate or check size
  if (qpos_.empty()) {
    qpos_.resize(m->nq);
    for (int i=0; i < m->nq; i++) {
      qpos_[i] = (double)m->qpos0[i];
    }
  } else if (qpos_.size() != m->nq) {
    throw mjCError(this, "keyframe %d: invalid qpos size, expected length %d", nullptr, id, m->nq);
  }

  // qvel: allocate or check size
  if (qvel_.empty()) {
    qvel_.resize(m->nv);
    for (int i=0; i < m->nv; i++) {
      qvel_[i] = 0;
    }
  } else if (qvel_.size() != m->nv) {
    throw mjCError(this, "keyframe %d: invalid qvel size, expected length %d", nullptr, id, m->nv);
  }

  // act: allocate or check size
  if (act_.empty()) {
    act_.resize(m->na);
    for (int i=0; i < m->na; i++) {
      act_[i] = 0;
    }
  } else if (act_.size() != m->na) {
    throw mjCError(this, "keyframe %d: invalid act size, expected length %d", nullptr, id, m->na);
  }

  // mpos: allocate or check size
  if (mpos_.empty()) {
    mpos_.resize(3*m->nmocap);
    if (m->nmocap) {
      for (int i=0; i < m->nbody; i++) {
        if (m->body_mocapid[i] >= 0) {
          int mocapid = m->body_mocapid[i];
          mpos_[3*mocapid]   = m->body_pos[3*i];
          mpos_[3*mocapid+1] = m->body_pos[3*i+1];
          mpos_[3*mocapid+2] = m->body_pos[3*i+2];
        }
      }
    }
  } else if (mpos_.size() != 3*m->nmocap) {
    throw mjCError(this, "keyframe %d: invalid mpos size, expected length %d", nullptr, id, 3*m->nmocap);
  }

  // mquat: allocate or check size
  if (mquat_.empty()) {
    mquat_.resize(4*m->nmocap);
    if (m->nmocap) {
      for (int i=0; i < m->nbody; i++) {
        if (m->body_mocapid[i] >= 0) {
          int mocapid = m->body_mocapid[i];
          mquat_[4*mocapid]   = m->body_quat[4*i];
          mquat_[4*mocapid+1] = m->body_quat[4*i+1];
          mquat_[4*mocapid+2] = m->body_quat[4*i+2];
          mquat_[4*mocapid+3] = m->body_quat[4*i+3];
        }
      }
    }
  } else if (mquat_.size() != 4*m->nmocap) {
    throw mjCError(this, "keyframe %d: invalid mquat size, expected length %d", nullptr, id, 4*m->nmocap);
  }

  // ctrl: allocate or check size
  if (ctrl_.empty()) {
    ctrl_.resize(m->nu);
    for (int i=0; i < m->nu; i++) {
      ctrl_[i] = 0;
    }
  } else if (ctrl_.size() != m->nu) {
    throw mjCError(this, "keyframe %d: invalid ctrl size, expected length %d", nullptr, id, m->nu);
  }
}


//------------------ class mjCPlugin implementation ------------------------------------------------

// initialize defaults
mjCPlugin::mjCPlugin(mjCModel* _model) {
  name = "";
  nstate = -1;
  plugin_slot = -1;
  parent = this;
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  name.clear();
  plugin_name.clear();

  // public interface
  mjs_defaultPlugin(&spec);
  elemtype = mjOBJ_PLUGIN;
  spec.plugin_name = &plugin_name;
  spec.name = &name;
  spec.info = &info;

  PointToLocal();
}



mjCPlugin::mjCPlugin(const mjCPlugin& other) {
  *this = other;
  id = -1;
}



mjCPlugin& mjCPlugin::operator=(const mjCPlugin& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCPlugin_*>(this) = static_cast<const mjCPlugin_&>(other);
    parent = this;
    plugin_slot = other.plugin_slot;
  }
  PointToLocal();
  return *this;
}



void mjCPlugin::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.info = &info;
}



// compiler
void mjCPlugin::Compile(void) {
  mjCPlugin* plugin_instance = this;
  model->ResolvePlugin(this, plugin_name, name, &plugin_instance);
  const mjpPlugin* plugin = mjp_getPluginAtSlot(plugin_slot);

  // clear precompiled
  flattened_attributes.clear();
  std::map<std::string, std::string, std::less<> > config_attribs_copy = config_attribs;

  // concatenate all of the plugin's attribute values (as null-terminated strings) into
  // flattened_attributes, in the order declared in the mjpPlugin
  // each valid attribute found is appended to flattened_attributes and removed from xml_attributes
  for (int i = 0; i < plugin->nattribute; ++i) {
    std::string_view attr(plugin->attributes[i]);
    auto it = config_attribs_copy.find(attr);
    if (it == config_attribs_copy.end()) {
      flattened_attributes.push_back('\0');
    } else {
      auto original_size = flattened_attributes.size();
      flattened_attributes.resize(original_size + it->second.size() + 1);
      std::memcpy(&flattened_attributes[original_size], it->second.c_str(),
                  it->second.size() + 1);
      config_attribs_copy.erase(it);
    }
  }

  // if there are no attributes, add a null terminator
  if (plugin->nattribute == 0) {
    flattened_attributes.push_back('\0');
  }

  // anything left in xml_attributes at this stage is not a valid attribute
  if (!config_attribs_copy.empty()) {
    std::string error =
      "unrecognized attribute 'plugin:" + config_attribs_copy.begin()->first +
      "' for plugin " + std::string(plugin->name) + "'";
    throw mjCError(parent, "%s", error.c_str());
  }
}
