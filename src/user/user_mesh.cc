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

#include <algorithm>
#include <cmath>
#include <csetjmp>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef MUJOCO_TINYOBJLOADER_IMPL
#define TINYOBJLOADER_IMPLEMENTATION
#endif

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-anonymous-struct"
#pragma clang diagnostic ignored "-Wnested-anon-types"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <MC.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjplugin.h>
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_plugin.h"
#include "engine/engine_resource.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_spatial.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_util.h"
#include "xml/xml_util.h"
#include <tiny_obj_loader.h>

extern "C" {
#include "qhull_ra.h"
}

using std::string;
using std::vector;

// compute triangle area, surface normal, center
static mjtNum _triangle(mjtNum* normal, mjtNum* center,
                        const float* v1, const float* v2, const float* v3) {
  // center
  if (center) {
    for (int i=0; i<3; i++) {
      center[i] = (v1[i] + v2[i] + v3[i])/3;
    }
  }

  // normal = (v2-v1) cross (v3-v1)
  double b[3] = { v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2] };
  double c[3] = { v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2] };
  mju_cross(normal, b, c);

  // get length
  double len = mju_norm3(normal);

  // ignore small faces
  if (len<mjMINVAL) {
    return 0;
  }

  // normalize
  normal[0] /= len;
  normal[1] /= len;
  normal[2] /= len;

  // return area
  return len/2;
}

template <typename T>
static T* VecToArray(std::vector<T>& vector,  bool clear = true){
  if (vector.empty())
    return nullptr;
  else {
    int n = (int)vector.size();
    T* cvec = (T*) mju_malloc(n*sizeof(T));
    memcpy(cvec, vector.data(), n*sizeof(T));
    if (clear) {
      vector.clear();
    }
    return cvec;
  }
}

// Read data of type T from a potentially unaligned buffer pointer.
template <typename T>
static void ReadFromBuffer(T* dst, const char* src) {
  std::memcpy(dst, src, sizeof(T));
}

//------------------ class mjCMesh implementation --------------------------------------------------

mjCMesh::mjCMesh(mjCModel* _model, mjCDef* _def) {
  // set defaults
  mjuu_setvec(refpos_, 0, 0, 0);
  mjuu_setvec(refquat_, 1, 0, 0, 0);
  mjuu_setvec(scale_, 1, 1, 1);
  smoothnormal_ = false;

  // clear internal variables
  mjuu_setvec(pos_surface_, 0, 0, 0);
  mjuu_setvec(pos_volume_, 0, 0, 0);
  mjuu_setvec(quat_surface_, 1, 0, 0, 0);
  mjuu_setvec(quat_volume_, 1, 0, 0, 0);
  mjuu_setvec(pos_, 0, 0, 0);
  mjuu_setvec(quat_, 1, 0, 0, 0);

  mjuu_setvec(boxsz_surface_, 0, 0, 0);
  mjuu_setvec(boxsz_volume_, 0, 0, 0);
  mjuu_setvec(aamm_, 1e10, 1e10, 1e10);
  mjuu_setvec(aamm_+3, -1e10, -1e10, -1e10);
  nvert_ = 0;
  nnormal_ = 0;
  ntexcoord_ = 0;
  nface_ = 0;
  szgraph_ = 0;
  vert_ = NULL;
  normal_ = NULL;
  center_ = NULL;
  texcoord_ = NULL;
  face_ = NULL;
  facenormal_ = NULL;
  facetexcoord_ = NULL;
  graph_ = NULL;
  needhull_ = false;
  invalidorientation_.first = -1;
  invalidorientation_.second = -1;
  validarea_ = true;
  validvolume_ = 1;
  valideigenvalue_ = true;
  validinequality_ = true;
  processed_ = false;

  // reset to default if given
  if (_def) {
    *this = _def->mesh;
  }

  // set model, def
  model = _model;
  def = (_def ? _def : (_model ? _model->defaults[0] : 0));
}



mjCMesh::~mjCMesh() {
  if (vert_) mju_free(vert_);
  if (normal_) mju_free(normal_);
  if (texcoord_) mju_free(texcoord_);
  if (center_) mju_free(center_);
  if (face_) mju_free(face_);
  if (facenormal_) mju_free(facenormal_);
  if (facetexcoord_) mju_free(facetexcoord_);
  if (graph_) mju_free(graph_);
}



void mjCMesh::set_content_type(std::optional<std::string>&& content_type) {
  if (content_type.has_value()) {
    content_type_ = std::move(content_type.value());
  }
}



void mjCMesh::set_file(std::optional<std::string>&& file) {
  if (file.has_value()) {
    file_ = std::move(file.value());
  }
}



void mjCMesh::set_refpos(std::optional<std::array<double, 3>> refpos) {
  if (refpos.has_value()) {
    std::copy(refpos.value().begin(), refpos.value().end(), refpos_);
  }
}



void mjCMesh::set_refquat(std::optional<std::array<double, 4>> refquat) {
  if (refquat.has_value()) {
    std::copy(refquat.value().begin(), refquat.value().end(), refquat_);
  }
}



void mjCMesh::set_scale(std::optional<std::array<double, 3>> scale) {
  if (scale.has_value()) {
    set_scale(scale.value());
  }
}



void mjCMesh::set_uservert(std::optional<std::vector<float>>&& uservert) {
  if (uservert.has_value()) {
    uservert_ = std::move(uservert.value());
  }
}



void mjCMesh::set_usernormal(std::optional<std::vector<float>>&& usernormal) {
  if (usernormal.has_value()) {
    usernormal_ = std::move(usernormal.value());
  }
}



void mjCMesh::set_usertexcoord(std::optional<std::vector<float>>&& usertexcoord) {
  if (usertexcoord.has_value()) {
    usertexcoord_ = std::move(usertexcoord.value());
  }
}



void mjCMesh::set_userface(std::optional<std::vector<int>>&& userface) {
  if (userface.has_value()) {
    userface_ = std::move(userface.value());
  }
}



void mjCMesh::set_file(const std::string& file) {
  file_ = file;
}



void mjCMesh::set_scale(std::array<double, 3> scale) {
  std::copy(scale.begin(), scale.end(), scale_);
}



void mjCMesh::set_smoothnormal(bool smoothnormal) {
  smoothnormal_ = smoothnormal;
}



void mjCMesh::set_needhull(bool needhull) {
  needhull_ = needhull;
}



// generate mesh using marching cubes
void mjCMesh::LoadSDF() {
  if (plugin_name.empty() && plugin_instance_name.empty()) {
    throw mjCError(
        this, "neither 'plugin' nor 'instance' is specified for mesh '%s', (id = %d)",
        name.c_str(), id);
  }

  if (scale_[0] != 1 || scale_[1] != 1 || scale_[2] != 1) {
    throw mjCError(this, "attribute scale is not compatible with SDFs in mesh '%s', (id = %d)",
                   name.c_str(), id);
  }

  model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
  const mjpPlugin* plugin = mjp_getPluginAtSlot(plugin_instance->plugin_slot);
  if (!(plugin->capabilityflags & mjPLUGIN_SDF)) {
    throw mjCError(this, "plugin '%s' does not support signed distance fields", plugin->name);
  }

  std::vector<mjtNum> attributes(plugin->nattribute, 0);
  std::vector<const char*> names(plugin->nattribute, 0);
  std::vector<const char*> values(plugin->nattribute, 0);
  for (int i=0; i < plugin->nattribute; i++) {
    names[i] = plugin->attributes[i];
    values[i] = plugin_instance->config_attribs[names[i]].c_str();
  }

  if (plugin->sdf_attribute) {
    plugin->sdf_attribute(attributes.data(), names.data(), values.data());
  }

  mjtNum aabb[6] = {0};
  plugin->sdf_aabb(aabb, attributes.data());
  mjtNum total = aabb[3] + aabb[4] + aabb[5];

  const mjtNum n = 300;
  int nx, ny, nz;
  nx = floor(n / total * aabb[3]) + 1;
  ny = floor(n / total * aabb[4]) + 1;
  nz = floor(n / total * aabb[5]) + 1;
  MC::MC_FLOAT* field = new MC::MC_FLOAT[nx * ny * nz];

  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
        mjtNum point[] = {aabb[0]-aabb[3] + 2 * aabb[3] * i / (nx-1),
                          aabb[1]-aabb[4] + 2 * aabb[4] * j / (ny-1),
                          aabb[2]-aabb[5] + 2 * aabb[5] * k / (nz-1)};
        field[(k * ny + j) * nx + i] =  plugin->sdf_staticdistance(point, attributes.data());
      }
    }
  }

  MC::mcMesh mesh;
  MC::marching_cube(field, nx, ny, nz, mesh);
  std::vector<float> uservert;
  std::vector<float> usernormal;
  std::vector<int> userface;

  for (size_t i = 0; i < mesh.vertices.size(); i++) {
    uservert.push_back(2*aabb[3]*mesh.vertices.at(i).x/(nx-1) + aabb[0]-aabb[3]);
    uservert.push_back(2*aabb[4]*mesh.vertices.at(i).y/(ny-1) + aabb[1]-aabb[4]);
    uservert.push_back(2*aabb[5]*mesh.vertices.at(i).z/(nz-1) + aabb[2]-aabb[5]);
  }

  for (size_t i = 0; i < mesh.normals.size(); i++) {
    usernormal.push_back(mesh.normals.at(i).x);
    usernormal.push_back(mesh.normals.at(i).y);
    usernormal.push_back(mesh.normals.at(i).z);
  }

  for (size_t i = 0; i < mesh.indices.size(); i++) {
    userface.push_back(mesh.indices.at(i));
  }

  set_uservert(uservert);
  set_usernormal(usernormal);
  set_userface(userface);
  delete[] field;
}



// compiler
void mjCMesh::Compile(const mjVFS* vfs) {
  // load file
  if (!file_.empty()) {
    // remove path from file if necessary
    if (model->strippath) {
      file_ = mjuu_strippath(file_);
    }

    std::string asset_type = GetAssetContentType(file_, content_type_);
    if (asset_type.empty()) {
      throw mjCError(this, "unknown mesh content type for file: '%s'", file_.c_str());
    }

    if (asset_type != "model/stl" && asset_type != "model/obj"
        && asset_type != "model/vnd.mujoco.msh") {
      throw mjCError(this, "unsupported content type: '%s'", asset_type.c_str());
    }

    string filename = mjuu_makefullname(model->modelfiledir, model->meshdir, file_);
    mjResource* resource = LoadResource(filename, vfs);

    try {
      if (asset_type == "model/stl") {
        LoadSTL(resource);
      } else if (asset_type == "model/obj") {
        LoadOBJ(resource);
      } else {
        LoadMSH(resource);
      }
      mju_closeResource(resource);
    } catch (mjCError err) {
      mju_closeResource(resource);
      throw err;
    }
  }

  // create using marching cubes
  else if (is_plugin) {
    LoadSDF();
  }

  // copy user vertex
  if (!uservert_.empty()) {
    // check repeated
    if (vert_) {
      throw mjCError(this, "repeated vertex specification");
    }

    // check size
    if (uservert_.size()<12) {
      throw mjCError(this, "at least 4 vertices required");
    }
    if (uservert_.size()%3) {
      throw mjCError(this, "vertex data must be a multiple of 3");
    }

    // copy from user
    nvert_ = (int)uservert_.size()/3;
    vert_ = VecToArray(uservert_, !file_.empty());
  }

  // copy user normal
  if (!usernormal_.empty()) {
    // check repeated
    if (normal_) {
      throw mjCError(this, "repeated normal specification");
    }

    // check size
    if (usernormal_.size()%3) {
      throw mjCError(this, "normal data must be a multiple of 3");
    }

    // copy from user
    nnormal_ = (int)usernormal_.size()/3;
    normal_ = VecToArray(usernormal_, !file_.empty());
  }

  // copy user texcoord
  if (!usertexcoord_.empty()) {
    // check repeated
    if (texcoord_) {
      throw mjCError(this, "repeated texcoord specification");
    }

    // check size
    if (usertexcoord_.size()%2) {
      throw mjCError(this, "texcoord must be a multiple of 2");
    }

    // copy from user
    ntexcoord_ = (int)usertexcoord_.size()/2;
    texcoord_ = VecToArray(usertexcoord_, !file_.empty());
  }

  // copy user face
  if (!userface_.empty()) {
    // check repeated
    if (face_) {
      throw mjCError(this, "repeated face specification");
    }

    // check size
    if (userface_.size()%3) {
      throw mjCError(this, "face data must be a multiple of 3");
    }

    // check vertices exist
    for (int i=0; i<userface_.size(); i++) {
      if (userface_[i] >= nvert_ || userface_[i] < 0) {
        throw mjCError(this, "index in face does not exist in vertex array");
      }
    }

    // create half-edge structure (if mesh was in XML)
    if (useredge_.empty()) {
      for (int i=0; i<userface_.size()/3; i++) {
        int v0 = userface_[3*i+0];
        int v1 = userface_[3*i+1];
        int v2 = userface_[3*i+2];
        mjtNum normal[3];
        if (_triangle(normal, nullptr, vert_+3*v0, vert_+3*v1, vert_+3*v2)>sqrt(mjMINVAL)) {
          useredge_.push_back(std::pair(v0, v1));
          useredge_.push_back(std::pair(v1, v2));
          useredge_.push_back(std::pair(v2, v0));
        } else {
          // TODO(b/255525326)
        }
      }
    }

    // copy from user
    nface_ = (int)userface_.size()/3;
    face_ = VecToArray(userface_, !file_.empty());

    // check vertices exist
    for (auto vertex_index : userface_) {
      if (vertex_index>=nvert_ || vertex_index < 0) {
        throw mjCError(this, "found index in userface that exceeds uservert size.");
      }
    }
  }

  // check for inconsistent face orientations
  if (!useredge_.empty()) {
    std::stable_sort(useredge_.begin(), useredge_.end());
    auto iterator = std::adjacent_find(useredge_.begin(), useredge_.end());
    if (iterator != useredge_.end()) {
      invalidorientation_.first = iterator->first+1;
      invalidorientation_.second = iterator->second+1;
    }
  }

  // require vertices
  if (!vert_) {
    throw mjCError(this, "no vertices");
  }

  // make graph describing convex hull
  if ((model->convexhull && needhull_) || !face_) {
    MakeGraph();
  }

  // no faces: copy from convex hull
  if (!face_) {
    CopyGraph();
  }

  // no normals: make
  if (!normal_) {
    MakeNormal();
  }

  // copy user normal indices
  if (!userfacenormal_.empty()) {
    // check repeated
    if (facenormal_) {
      throw mjCError(this, "repeated facenormal specification");
    }

    if (userfacenormal_.size()!=3*nface_) {
      throw mjCError(this, "face data must have the same size as face normal data");
    }

    facenormal_ = VecToArray(userfacenormal_, !file_.empty());
  }

  // copy user texcoord
  if (!userfacetexcoord_.empty()) {
    // check repeated
    if (facetexcoord_) {
      throw mjCError(this, "repeated facetexcoord specification");
    }

    facetexcoord_ = VecToArray(userfacetexcoord_, !file_.empty());
  }

  // facenormal might not exist if usernormal was specified
  if (!facenormal_) {
    facenormal_ = (int*) mju_malloc(3*nface_*sizeof(int));
    memcpy(facenormal_, face_, 3*nface_*sizeof(int));
  }

  // scale, center, orient, compute mass and inertia
  Process();
  processed_ = true;

  // no radii: make
  if (!center_) {
    MakeCenter();
  }

  // make bounding volume hierarchy
  if (tree_.bvh.empty()) {
    face_aabb_.assign(6*nface_, 0);
    for (int i=0; i<nface_; i++) {
      tree_.AddBoundingVolume(GetBoundingVolume(i));
    }
    tree_.CreateBVH();
  }
}



// get bounding volume
mjCBoundingVolume mjCMesh::GetBoundingVolume(int faceid) {
  mjCBoundingVolume node;
  node.id = faceid;
  node.conaffinity = 1;
  node.contype = 1;
  node.pos = center_ + 3*faceid;
  node.quat = NULL;
  mjtNum face_aamm[6] = {1E+10, 1E+10, 1E+10, -1E+10, -1E+10, -1E+10};
  for (int j=0; j<3; j++) {
    int vertid = face_[3*faceid+j];
    face_aamm[0] = mjMIN(face_aamm[0], vert_[3*vertid+0]);
    face_aamm[1] = mjMIN(face_aamm[1], vert_[3*vertid+1]);
    face_aamm[2] = mjMIN(face_aamm[2], vert_[3*vertid+2]);
    face_aamm[3] = mjMAX(face_aamm[3], vert_[3*vertid+0]);
    face_aamm[4] = mjMAX(face_aamm[4], vert_[3*vertid+1]);
    face_aamm[5] = mjMAX(face_aamm[5], vert_[3*vertid+2]);
  }
  face_aabb_[6*faceid+0] = .5 * (face_aamm[0] + face_aamm[3]);
  face_aabb_[6*faceid+1] = .5 * (face_aamm[1] + face_aamm[4]);
  face_aabb_[6*faceid+2] = .5 * (face_aamm[2] + face_aamm[5]);
  face_aabb_[6*faceid+3] = .5 * (face_aamm[3] - face_aamm[0]);
  face_aabb_[6*faceid+4] = .5 * (face_aamm[4] - face_aamm[1]);
  face_aabb_[6*faceid+5] = .5 * (face_aamm[5] - face_aamm[2]);
  node.aabb = face_aabb_.data() + 6*faceid;
  return node;
}



// get position
double* mjCMesh::GetPosPtr(mjtMeshType type) {
  if (type==mjSHELL_MESH) {
    return pos_surface_;
  } else {
    return pos_volume_;
  }
}



// get orientation
double* mjCMesh::GetQuatPtr(mjtMeshType type) {
  if (type==mjSHELL_MESH) {
    return quat_surface_;
  } else {
    return quat_volume_;
  }
}



double* mjCMesh::GetOffsetPosPtr() {
  return pos_;
}



double* mjCMesh::GetOffsetQuatPtr() {
  return quat_;
}



bool mjCMesh::HasTexcoord() const {
  return texcoord_ != nullptr;
}



void mjCMesh::CopyVert(float* arr) const {
  std::copy(vert_, vert_+3*nvert_, arr);
}



void mjCMesh::CopyNormal(float* arr) const {
  std::copy(normal_, normal_+3*nnormal_, arr);
}



void mjCMesh::CopyFace(int* arr) const {
  std::copy(face_, face_+3*nface_, arr);
}



void mjCMesh::CopyFaceTexcoord(int* arr) const {
  std::copy(facetexcoord_, facetexcoord_+3*nface_, arr);
}



void mjCMesh::CopyFaceNormal(int* arr) const {
  std::copy(facenormal_, facenormal_+3*nface_, arr);
}



void mjCMesh::CopyTexcoord(float* arr) const {
  std::copy(texcoord_, texcoord_+2*ntexcoord_, arr);
}



void mjCMesh::CopyGraph(int* arr) const {
  std::copy(graph_, graph_+szgraph_, arr);
}



// set geom size to match mesh
void mjCMesh::FitGeom(mjCGeom* geom, double* meshpos) {
  // copy mesh pos into meshpos
  mjuu_copyvec(meshpos, GetPosPtr(geom->typeinertia), 3);

  // use inertial box
  if (!model->fitaabb) {
    // get inertia box type (shell or volume)
    double* boxsz = GetInertiaBoxPtr(geom->typeinertia);
    switch (geom->type) {
    case mjGEOM_SPHERE:
      geom->size[0] = (boxsz[0] + boxsz[1] + boxsz[2])/3;
      break;

    case mjGEOM_CAPSULE:
      geom->size[0] = (boxsz[0] + boxsz[1])/2;
      geom->size[1] = mju_max(0, boxsz[2] - geom->size[0]/2);
      break;

    case mjGEOM_CYLINDER:
      geom->size[0] = (boxsz[0] + boxsz[1])/2;
      geom->size[1] = boxsz[2];
      break;

    case mjGEOM_ELLIPSOID:
    case mjGEOM_BOX:
      geom->size[0] = boxsz[0];
      geom->size[1] = boxsz[1];
      geom->size[2] = boxsz[2];
      break;

    default:
      throw mjCError(this, "invalid geom type in fitting mesh %s", name.c_str());
    }
  }

  // use aamm
  else {
    // find aabb box center
    double cen[3] = {(aamm_[0]+aamm_[3])/2, (aamm_[1]+aamm_[4])/2, (aamm_[2]+aamm_[5])/2};

    // add box center into meshpos
    meshpos[0] += cen[0];
    meshpos[1] += cen[1];
    meshpos[2] += cen[2];

    // compute depending on type
    switch (geom->type) {
    case mjGEOM_SPHERE:
      // find maximum distance
      geom->size[0] = 0;
      for (int i=0; i<nvert_; i++) {
        double v[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
        double dst = mjuu_dist3(v, cen);
        geom->size[0] = mju_max(geom->size[0], dst);
      }
      break;

    case mjGEOM_CAPSULE:
    case mjGEOM_CYLINDER:
      // find maximum distance in XY, separately in Z
      geom->size[0] = 0;
      geom->size[1] = 0;
      for (int i=0; i<nvert_; i++) {
        double v[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
        double dst = sqrt((v[0]-cen[0])*(v[0]-cen[0]) +
                          (v[1]-cen[1])*(v[1]-cen[1]));
        geom->size[0] = mju_max(geom->size[0], dst);

        // proceed with z: valid for cylinder
        double dst2 = fabs(v[2]-cen[2]);
        geom->size[1] = mju_max(geom->size[1], dst2);
      }

      // special handling of capsule: consider curved cap
      if (geom->type==mjGEOM_CAPSULE) {
        geom->size[1] = 0;
        for (int i=0; i<nvert_; i++) {
          // get distance in XY and Z
          double v[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
          double dst = sqrt((v[0]-cen[0])*(v[0]-cen[0]) +
                            (v[1]-cen[1])*(v[1]-cen[1]));
          double dst2 = fabs(v[2]-cen[2]);

          // get spherical elevation at horizontal distance dst
          double h = geom->size[0] * sin(acos(dst/geom->size[0]));
          geom->size[1] = mju_max(geom->size[1], dst2-h);
        }
      }
      break;

    case mjGEOM_ELLIPSOID:
    case mjGEOM_BOX:
      geom->size[0] = aamm_[3] - cen[0];
      geom->size[1] = aamm_[4] - cen[1];
      geom->size[2] = aamm_[5] - cen[2];
      break;

    default:
      throw mjCError(this, "invalid fittype in mesh %s", name.c_str());
    }
  }

  // rescale size
  geom->size[0] *= geom->fitscale;
  geom->size[1] *= geom->fitscale;
  geom->size[2] *= geom->fitscale;
}



// comparison function for vertex sorting
quicksortfunc(vertcompare, context, el1, el2) {
  float* vert = (float*) context;
  float x1 = vert[3*(*(int*)el1)] + 1e-2*vert[1+3*(*(int*)el1)] + 1e-4*vert[2+3*(*(int*)el1)];
  float x2 = vert[3*(*(int*)el2)] + 1e-2*vert[1+3*(*(int*)el2)] + 1e-4*vert[2+3*(*(int*)el2)];

  if (x1 < x2) {
    return -1;
  } else if (x1 == x2) {
    return 0;
  } else {
    return 1;
  }
}

// remove repeated vertices
void mjCMesh::RemoveRepeated() {
  int repeated = 0;

  // allocate sort and redirection indices, set to identity
  auto index = std::unique_ptr<int[]>(new int[nvert_]);
  auto redirect = std::unique_ptr<int[]>(new int[nvert_]);
  for (int i=0; i < nvert_; i++) {
    index[i] = redirect[i] = i;
  }

  // sort vertices
  mjQUICKSORT(index.get(), nvert_, sizeof(int), vertcompare, vert_);

  // find repeated vertices, set redirect
  for (int i=1; i < nvert_; i++) {
    if (vert_[3*index[i]] == vert_[3*index[i-1]] &&
        vert_[3*index[i]+1] == vert_[3*index[i-1]+1] &&
        vert_[3*index[i]+2] == vert_[3*index[i-1]+2]) {
      redirect[index[i]] = index[i-1];
      repeated++;
    }
  }

  // compress vertices, change face data
  if (repeated) {
    // track redirections until non-redirected vertex, set
    for (int i=0; i<nvert_; i++) {
      int j = i;
      while (redirect[j]!=j) {
        j = redirect[j];
      }
      redirect[i] = j;
    }

    // find good vertices, compress, reuse index to save compressed position
    int j = 0;
    for (int i=0; i<nvert_; i++) {
      if (redirect[i]==i) {
        index[i] = j;
        memcpy(vert_+3*j, vert_+3*i, 3*sizeof(float));
        j++;
      } else {
        index[i] = -1;
      }
    }

    // recompute face data to reflect compressed vertices
    for (int i=0; i<3*nface_; i++) {
      face_[i] = index[redirect[face_[i]]];

      // sanity check, SHOULD NOT OCCUR
      if (face_[i]<0 || face_[i]>=nvert_-repeated) {
        throw mjCError(
            this, "error removing vertices from mesh '%s'", name.c_str());
      }
    }
  }

  // correct vertex count
  nvert_ -= repeated;

  // resize vert if any vertices were removed
  if (repeated) {
    float* old = vert_;
    vert_ = (float*) mju_malloc(3*nvert_*sizeof(float));
    memcpy(vert_, old, 3*nvert_*sizeof(float));
    mju_free(old);
  }
}


// load OBJ mesh
void mjCMesh::LoadOBJ(mjResource* resource) {
  tinyobj::ObjReader objReader;
  const void* bytes = nullptr;
  int buffer_sz = mju_readResource(resource, &bytes);
  if (buffer_sz < 0) {
    throw mjCError(this, "could not read OBJ file '%s'", resource->name);
  }

  // TODO(etom): support .mtl files?
  const char* buffer = (const char*) bytes;
  objReader.ParseFromString(std::string(buffer, buffer_sz), std::string());

  if (!objReader.Valid()) {
    throw mjCError(this, "could not parse OBJ file '%s'", resource->name);
  }

  const auto& attrib = objReader.GetAttrib();
  uservert_ = attrib.vertices;  // copy from one std::vector to another
  usernormal_ = attrib.normals;
  usertexcoord_ = attrib.texcoords;

  if (!objReader.GetShapes().empty()) {
    const auto& mesh = objReader.GetShapes()[0].mesh;
    bool righthand = (scale_[0]*scale_[1]*scale_[2] > 0);

    // iterate over mesh faces
    std::vector<tinyobj::index_t> face_indices;
    for (int face = 0, idx = 0; idx < mesh.indices.size();) {
      int nfacevert = mesh.num_face_vertices[face];
      if (nfacevert < 3 || nfacevert > 4) {
        throw mjCError(
            this, "only tri or quad meshes are supported for OBJ (file '%s')",
            resource->name);
      }

      face_indices.push_back(mesh.indices[idx]);
      face_indices.push_back(mesh.indices[idx + (righthand==1 ? 1 : 2)]);
      face_indices.push_back(mesh.indices[idx + (righthand==1 ? 2 : 1)]);

      if (nfacevert == 4) {
        face_indices.push_back(mesh.indices[idx]);
        face_indices.push_back(mesh.indices[idx + (righthand==1 ? 2 : 3)]);
        face_indices.push_back(mesh.indices[idx + (righthand==1 ? 3 : 2)]);
      }
      idx += nfacevert;
      ++face;
    }

    // for each vertex, store index, normal, and texcoord
    for (const auto& mesh_index : face_indices) {
      userface_.push_back(mesh_index.vertex_index);

      if (!usernormal_.empty()) {
        userfacenormal_.push_back(mesh_index.normal_index);
      }

      if (!usertexcoord_.empty()) {
        userfacetexcoord_.push_back(mesh_index.texcoord_index);
      }
    }
  }

  // flip the second texcoord
  for (int i=0; i<usertexcoord_.size()/2; i++) {
    usertexcoord_[2*i+1] = 1-usertexcoord_[2*i+1];
  }
}


// load STL binary mesh
void mjCMesh::LoadSTL(mjResource* resource) {
  bool righthand = (scale_[0]*scale_[1]*scale_[2]>0);

  // get file data in buffer
  char* buffer = 0;
  int buffer_sz = mju_readResource(resource, (const void**)  &buffer);

  // still not found
  if (buffer_sz < 0) {
    throw mjCError(this, "could not read STL file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw mjCError(this, "STL file '%s' is empty", resource->name);
  }

  // make sure there is enough data for header
  if (buffer_sz<84) {
    throw mjCError(this, "invalid header in STL file '%s'", resource->name);
  }

  // get number of triangles, check bounds
  ReadFromBuffer(&nface_, buffer + 80);
  if (nface_<1 || nface_>200000) {
    throw mjCError(this,
                   "number of faces should be between 1 and 200000 in STL file '%s';"
                   " perhaps this is an ASCII file?", resource->name);
  }

  // check remaining buffer size
  if (nface_*50 != buffer_sz-84) {
    throw mjCError(this,
                   "STL file '%s' has wrong size; perhaps this is an ASCII file?",
                   resource->name);
  }

  // assign stl data pointer
  const char* stl = buffer + 84;

  // allocate face and vertex data
  face_ = (int*) mju_malloc(3*nface_*sizeof(int));
  vert_ = (float*) mju_malloc(9*nface_*sizeof(float));

  // add vertices and faces, including repeated for now
  for (int i=0; i<nface_; i++) {
    for (int j=0; j<3; j++) {
      // read vertex coordinates
      float v[3];
      ReadFromBuffer(&v, stl+50*i+12*(j+1));

      for (int k=0; k < 3; k++) {
        if (std::isnan(v[k]) || std::isinf(v[k])) {
          throw mjCError(this, "STL file '%s' contains invalid vertices.",
                         resource->name);
        }
        // check if vertex coordinates can be cast to an int safely
        if (fabs(v[k])>pow(2, 30)) {
          throw mjCError(this,
                        "vertex coordinates in STL file '%s' exceed maximum bounds",
                        resource->name);
        }
      }

      // add vertex address in face; change order if scale makes it lefthanded
      if (righthand || j==0) {
        face_[3*i+j] = nvert_;
      } else {
        face_[3*i+3-j] = nvert_;
      }

      // add vertex data
      memcpy(vert_+3*nvert_, v, 3*sizeof(float));
      nvert_++;
    }
  }

  RemoveRepeated();
}



// load MSH binary mesh
void mjCMesh::LoadMSH(mjResource* resource) {
  bool righthand = (scale_[0]*scale_[1]*scale_[2]>0);

  // get file data in buffer
  char* buffer = 0;
  int buffer_sz = mju_readResource(resource, (const void**)  &buffer);

  // still not found
  if (buffer_sz < 0) {
    throw mjCError(this, "could not read MSH file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw mjCError(this, "MSH file '%s' is empty", resource->name);
  }

  // make sure header is present
  if (buffer_sz<4*sizeof(int)) {
    throw mjCError(this, "missing header in MSH file '%s'", resource->name);
  }

  // get sizes from header
  ReadFromBuffer(&nvert_, buffer);
  ReadFromBuffer(&nnormal_, buffer + sizeof(int));
  ReadFromBuffer(&ntexcoord_, buffer + 2*sizeof(int));
  ReadFromBuffer(&nface_, buffer + 3*sizeof(int));

  // check sizes
  if (nvert_<4 || nface_<0 || nnormal_<0 || ntexcoord_<0 ||
      (nnormal_>0 && nnormal_!=nvert_) ||
      (ntexcoord_>0 && ntexcoord_!=nvert_)) {
    throw mjCError(this, "invalid sizes in MSH file '%s'", resource->name);
  }

  // check file size
  if (buffer_sz != 4*sizeof(int) + 3*nvert_*sizeof(float) + 3*nnormal_*sizeof(float) +
      2*ntexcoord_*sizeof(float) + 3*nface_*sizeof(int)) {
    throw mjCError(this, "unexpected file size in MSH file '%s'", resource->name);
  }

  // allocate and copy
  using UnalignedFloat = char[sizeof(float)];
  auto fdata = reinterpret_cast<UnalignedFloat*>(buffer + 4*sizeof(int));
  if (nvert_) {
    vert_ = (float*) mju_malloc(3*nvert_*sizeof(float));
    memcpy(vert_, fdata, 3*nvert_*sizeof(float));
    fdata += 3*nvert_;
  }
  if (nnormal_) {
    normal_ = (float*) mju_malloc(3*nvert_*sizeof(float));
    memcpy(normal_, fdata, 3*nvert_*sizeof(float));
    fdata += 3*nvert_;
  }
  if (ntexcoord_) {
    texcoord_ = (float*) mju_malloc(2*nvert_*sizeof(float));
    memcpy(texcoord_, fdata, 2*nvert_*sizeof(float));
    fdata += 2*nvert_;
  }
  if (nface_) {
    face_ = (int*) mju_malloc(3*nface_*sizeof(int));
    facenormal_ = (int*) mju_malloc(3*nface_*sizeof(int));
    memcpy(face_, fdata, 3*nface_*sizeof(int));
    memcpy(facenormal_, fdata, 3*nface_*sizeof(int));
  }
  if  (nface_ && texcoord_) {
    facetexcoord_= (int*) mju_malloc(3*nface_*sizeof(int));
    memcpy(facetexcoord_, fdata, 3*nface_*sizeof(int));
  }

  // rearrange face data if left-handed scaling
  if (nface_ && !righthand) {
    for (int i=0; i<nface_; i++) {
      int tmp = face_[3*i+1];
      face_[3*i+1] = face_[3*i+2];
      face_[3*i+2] = tmp;
    }
  }
}


void mjCMesh::ComputeVolume(double CoM[3], mjtMeshType type,
                              const double facecen[3], bool exactmeshinertia) {
  double nrm[3];
  double cen[3];
  GetVolumeRef(type) = 0;
  mjuu_zerovec(CoM, 3);
  for (int i=0; i<nface_; i++) {
    // get area, normal and center
    double a = _triangle(nrm, cen, vert_+3*face_[3*i], vert_+3*face_[3*i+1], vert_+3*face_[3*i+2]);

    // compute and add volume
    const double vec[3] = {cen[0]-facecen[0], cen[1]-facecen[1], cen[2]-facecen[2]};
    double vol = type==mjSHELL_MESH ? a : mjuu_dot3(vec, nrm) * a / 3;

    // if legacy computation requested, then always positive
    if (!exactmeshinertia && type==mjVOLUME_MESH) {
      vol = fabs(vol);
    }

    // add pyramid com
    GetVolumeRef(type) += vol;
    for (int j=0; j<3; j++) {
      CoM[j] += vol*(cen[j]*3.0/4.0 + facecen[j]/4.0);
    }
  }
}


// apply transformations
void mjCMesh::ApplyTransformations() {
  // translate
  if (refpos_[0]!=0 || refpos_[1]!=0 || refpos_[2]!=0) {
    // prepare translation
    float rp[3] = {(float)refpos_[0], (float)refpos_[1], (float)refpos_[2]};

    // process vertices
    for (int i=0; i<nvert_; i++) {
      vert_[3*i] -= rp[0];
      vert_[3*i+1] -= rp[1];
      vert_[3*i+2] -= rp[2];
    }
  }

  // rotate
  if (refquat_[0]!=1 || refquat_[1]!=0 || refquat_[2]!=0 || refquat_[3]!=0) {
    // prepare rotation
    mjtNum quat[4] = {refquat_[0], refquat_[1], refquat_[2], refquat_[3]};
    mjtNum mat[9];
    mju_normalize4(quat);
    mju_quat2Mat(mat, quat);

    // process vertices
    for (int i=0; i<nvert_; i++) {
      mjtNum p1[3], p0[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
      mju_rotVecMatT(p1, p0, mat);
      vert_[3*i] = (float) p1[0];
      vert_[3*i+1] = (float) p1[1];
      vert_[3*i+2] = (float) p1[2];
    }

    // process normals
    for (int i=0; i<nnormal_; i++) {
      mjtNum n1[3], n0[3] = {normal_[3*i], normal_[3*i+1], normal_[3*i+2]};
      mju_rotVecMatT(n1, n0, mat);
      normal_[3*i] = (float) n1[0];
      normal_[3*i+1] = (float) n1[1];
      normal_[3*i+2] = (float) n1[2];
    }
  }

  // scale
  if (scale_[0]!=1 || scale_[1]!=1 || scale_[2]!=1) {
    for (int i=0; i<nvert_; i++) {
      vert_[3*i] *= scale_[0];
      vert_[3*i+1] *= scale_[1];
      vert_[3*i+2] *= scale_[2];
    }

    for (int i=0; i<nnormal_; i++) {
      normal_[3*i] *= scale_[0];
      normal_[3*i+1] *= scale_[1];
      normal_[3*i+2] *= scale_[2];
    }
  }

  // normalize normals
  for (int i=0; i<nnormal_; i++) {
    // compute length
    float len = normal_[3*i]*normal_[3*i] + normal_[3*i+1]*normal_[3*i+1] + normal_[3*i+2]*normal_[3*i+2];

    // rescale
    if (len>mjMINVAL) {
      float scl = 1/sqrtf(len);
      normal_[3*i] *= scl;
      normal_[3*i+1] *= scl;
      normal_[3*i+2] *= scl;
    } else {
      normal_[3*i] = 0;
      normal_[3*i+1] = 0;
      normal_[3*i+2] = 1;
    }
  }
}


// find centroid of faces
void mjCMesh::ComputeFaceCentroid(double facecen[3]) {
  double area = 0;
  double nrm[3];
  double cen[3];

  for (int i=0; i<nface_; i++) {
    // check vertex indices
    for (int j=0; j<3; j++) {
      if (face_[3*i+j]<0 || face_[3*i+j]>=nvert_) {
        throw mjCError(this, "vertex index out of range in %s (index = %d)", name.c_str(), i);
      }
    }

    // get area and center
    double a = _triangle(nrm, cen, vert_+3*face_[3*i], vert_+3*face_[3*i+1], vert_+3*face_[3*i+2]);

    // accumulate
    for (int j=0; j<3; j++) {
      facecen[j] += a*cen[j];
    }
    area += a;
  }

  // require positive area
  if (area < mjMINVAL) {
    validarea_ = false;
    return;
  }

  // finalize centroid of faces
  for (int j=0; j<3; j++) {
    facecen[j] /= area;
  }
}


void mjCMesh::Process() {
  double facecen[3] = {0, 0, 0};
  double nrm[3];
  double cen[3];

  // user offset, rotation, scaling
  ApplyTransformations();

  // find centroid of faces
  ComputeFaceCentroid(facecen);

  // compute inertial properties for both inertia types
  for ( const auto type : { mjtMeshType::mjVOLUME_MESH, mjtMeshType::mjSHELL_MESH } ) {
    double CoM[3] = {0, 0, 0};
    double inert[6] = {0, 0, 0, 0, 0, 0};
    bool exactmeshinertia = model->exactmeshinertia;

    // compute CoM and volume from pyramid volumes
    ComputeVolume(CoM, type, facecen, model->exactmeshinertia);

    // perform computation with convex mesh if volume is negative
    if (GetVolumeRef(type) <= 0 && exactmeshinertia) {
      mju_warning("Malformed mesh %s, computing mesh inertia from convex hull", name.c_str());
      exactmeshinertia = false;
      ComputeVolume(CoM, type, facecen, exactmeshinertia);
    }

    // if volume is still invalid, skip the rest of the computations
    if (GetVolumeRef(type) < mjMINVAL) {
      validvolume_ = GetVolumeRef(type) < 0 ? -1 : 0;
      continue;
    }

    // finalize CoM, save as mesh center
    for (int j=0; j<3; j++) {
      CoM[j] /= GetVolumeRef(type);
    }
    mjuu_copyvec(GetPosPtr(type), CoM, 3);

    // re-center mesh at CoM
    if (type==mjVOLUME_MESH || validvolume_<=0) {
      for (int i=0; i<nvert_; i++) {
        for (int j=0; j<3; j++) {
          vert_[3*i+j] -= CoM[j];
        }
      }
    }

    // accumulate products of inertia, recompute volume
    const int k[6][2] = {{0, 0}, {1, 1}, {2, 2}, {0, 1}, {0, 2}, {1, 2}};
    double P[6] = {0, 0, 0, 0, 0, 0};
    GetVolumeRef(type) = 0;
    for (int i=0; i<nface_; i++) {
      float* D = vert_+3*face_[3*i];
      float* E = vert_+3*face_[3*i+1];
      float* F = vert_+3*face_[3*i+2];

      // get area, normal and center; update volume
      double a = _triangle(nrm, cen, D, E, F);
      double vol = type==mjSHELL_MESH ? a : mjuu_dot3(cen, nrm) * a / 3;

      // if legacy computation requested, then always positive
      if (!exactmeshinertia && type==mjVOLUME_MESH) {
        vol = fabs(vol);
      }

      // apply formula, accumulate
      GetVolumeRef(type) += vol;
      for (int j=0; j<6; j++) {
        P[j] += def->geom.density*vol /
                  (type==mjSHELL_MESH ? 12 : 20) * (
                  2*(D[k[j][0]] * D[k[j][1]] +
                    E[k[j][0]] * E[k[j][1]] +
                    F[k[j][0]] * F[k[j][1]]) +
                  D[k[j][0]] * E[k[j][1]]  +  D[k[j][1]] * E[k[j][0]] +
                  D[k[j][0]] * F[k[j][1]]  +  D[k[j][1]] * F[k[j][0]] +
                  E[k[j][0]] * F[k[j][1]]  +  E[k[j][1]] * F[k[j][0]]);
      }
    }

    // convert from products of inertia to moments of inertia
    inert[0] = P[1] + P[2];
    inert[1] = P[0] + P[2];
    inert[2] = P[0] + P[1];
    inert[3] = -P[3];
    inert[4] = -P[4];
    inert[5] = -P[5];

    // get quaternion and diagonal inertia
    mjtNum eigval[3], eigvec[9], quattmp[4];
    mjtNum full[9] = {
      inert[0], inert[3], inert[4],
      inert[3], inert[1], inert[5],
      inert[4], inert[5], inert[2]
    };
    mju_eig3(eigval, eigvec, quattmp, full);

    // check eigval - SHOULD NOT OCCUR
    if (eigval[2]<=0) {
      valideigenvalue_ = false;
      return;
    }
    if (eigval[0] + eigval[1] < eigval[2] ||
        eigval[0] + eigval[2] < eigval[1] ||
        eigval[1] + eigval[2] < eigval[0]) {
      validinequality_ = false;
      return;
    }

    // compute sizes of equivalent inertia box
    double mass = GetVolumeRef(type) * def->geom.density;
    double* boxsz = GetInertiaBoxPtr(type);
    boxsz[0] = sqrt(6*(eigval[1]+eigval[2]-eigval[0])/mass)/2;
    boxsz[1] = sqrt(6*(eigval[0]+eigval[2]-eigval[1])/mass)/2;
    boxsz[2] = sqrt(6*(eigval[0]+eigval[1]-eigval[2])/mass)/2;

    // if volume was valid, copy volume quat to shell and stop,
    // otherwise use shell quat for coordinate transformations
    if (type==mjSHELL_MESH && validvolume_>0) {
      mju_copy4(GetQuatPtr(type), GetQuatPtr(mjVOLUME_MESH));
      continue;
    }

    // rotate vertices and normals into axis-aligned frame
    mju_copy4(GetQuatPtr(type), quattmp);
    double neg[4] = {quattmp[0], -quattmp[1], -quattmp[2], -quattmp[3]};
    double mat[9];
    mjuu_quat2mat(mat, neg);
    for (int i=0; i<nvert_; i++) {
      // vertices
      const double vec[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
      double res[3];
      mjuu_mulvecmat(res, vec, mat);
      for (int j=0; j<3; j++) {
        vert_[3*i+j] = (float) res[j];

        // axis-aligned bounding box
        aamm_[j+0] = mju_min(aamm_[j+0], res[j]);
        aamm_[j+3] = mju_max(aamm_[j+3], res[j]);
      }
    }
    for (int i=0; i<nnormal_; i++) {
      // normals
      const double nrm[3] = {normal_[3*i], normal_[3*i+1], normal_[3*i+2]};
      double res[3];
      mjuu_mulvecmat(res, nrm, mat);
      for (int j=0; j<3; j++) {
        normal_[3*i+j] = (float) res[j];
      }
    }
  }
}


// check that the mesh is valid
void mjCMesh::CheckMesh(mjtMeshType type) {
  if (!processed_) {
    return;
  }
  if (invalidorientation_.first>=0 || invalidorientation_.second>=0)
    throw mjCError(this,
                   "faces of mesh '%s' have inconsistent orientation. Please check the "
                   "faces containing the vertices %d and %d.",
                   name.c_str(), invalidorientation_.first, invalidorientation_.second);
  if (!validarea_ && type==mjSHELL_MESH)
    throw mjCError(this, "mesh surface area is too small: %s", name.c_str());
  if (validvolume_<0 && type==mjVOLUME_MESH)
    throw mjCError(this, "mesh volume is negative (misoriented triangles): %s", name.c_str());
  if (!validvolume_ && type==mjVOLUME_MESH)
    throw mjCError(this, "mesh volume is too small: %s", name.c_str());
  if (!valideigenvalue_)
    throw mjCError(this, "eigenvalue of mesh inertia must be positive: %s", name.c_str());
  if (!validinequality_)
    throw mjCError(this, "eigenvalues of mesh inertia violate A + B >= C: %s", name.c_str());
}


// get inertia pointer
double* mjCMesh::GetInertiaBoxPtr(mjtMeshType type) {
  CheckMesh(type);
  return type==mjSHELL_MESH ? boxsz_surface_ : boxsz_volume_;
}


double& mjCMesh::GetVolumeRef(mjtMeshType type) {
  CheckMesh(type);
  return type==mjSHELL_MESH ? surface_ : volume_;
}


// make graph describing convex hull
void mjCMesh::MakeGraph(void) {
  int adr, ok, curlong, totlong, exitcode;
  double* data;
  facetT* facet, **facetp;
  vertexT* vertex, *vertex1, **vertex1p;
  char qhopt[10] = "qhull Qt";

  // graph not needed for small meshes
  if (nvert_ < 4) {
    return;
  }

  // convert mesh data to double
  data = (double*) mju_malloc(3*nvert_*sizeof(double));
  if (!data) {
    throw mjCError(this, "could not allocate data for qhull");
  }
  for (int i=0; i<3*nvert_; i++) {
    data[i] = (double)vert_[i];
  }

  qhT qh_qh;
  qhT* qh = &qh_qh;
  qh_zero(qh, stderr);

  // qhull basic init
  qh_init_A(qh, stdin, stdout, stderr, 0, NULL);

  // install longjmp error handler
  exitcode = setjmp(qh->errexit);
  qh->NOerrexit = false;
  if (!exitcode) {
    // actual init
    qh_initflags(qh, qhopt);
    qh_init_B(qh, data, nvert_, 3, False);

    // construct convex hull
    qh_qhull(qh);
    qh_triangulate(qh);
    qh_vertexneighbors(qh);

    // allocate graph:
    //  numvert, numface, vert_edgeadr[numvert], vert_globalid[numvert],
    //  edge_localid[numvert+3*numface], face_globalid[3*numface]
    int numvert = qh->num_vertices;
    int numface = qh->num_facets;
    szgraph_ = 2 + 3*numvert + 6*numface;
    graph_ = (int*) mju_malloc(szgraph_*sizeof(int));
    graph_[0] = numvert;
    graph_[1] = numface;

    // pointers for convenience
    int* vert_edgeadr = graph_ + 2;
    int* vert_globalid = graph_ + 2 + numvert;
    int* edge_localid = graph_ + 2 + 2*numvert;
    int* face_globalid = graph_ + 2 + 3*numvert + 3*numface;

    // fill in graph data
    int i = adr = 0;
    ok = 1;
    FORALLvertices {
      // point id of this vertex, check
      int pid = qh_pointid(qh, vertex->point);
      if (pid<0 || pid>=nvert_) {
        ok = 0;
        break;
      }

      // save edge address and global id of this vertex
      vert_edgeadr[i] = adr;
      vert_globalid[i] = pid;

      // process neighboring faces and their vertices
      int start = adr;
      FOREACHsetelement_(facetT, vertex->neighbors, facet) {
        int cnt = 0;
        FOREACHsetelement_(vertexT, facet->vertices, vertex1) {
          cnt++;

          // point id of face vertex, check
          int pid1 = qh_pointid(qh, vertex1->point);
          if (pid1<0 || pid1>=nvert_) {
            ok = 0;
            break;
          }

          // if different from vertex id, try to insert
          if (pid!=pid1) {
            // check for previous record
            int j;
            for (j=start; j<adr; j++)
              if (pid1==edge_localid[j]) {
                break;
              }

            // not found: insert
            if (j>=adr) {
              edge_localid[adr++] = pid1;
            }
          }
        }

        // make sure we have triangle: SHOULD NOT OCCUR
        if (cnt!=3) {
          mju_error("Qhull did not return triangle");
        }
      }

      // insert separator, advance to next vertex
      edge_localid[adr++] = -1;
      i++;
    }

    // size check: SHOULD NOT OCCUR
    if (adr!=numvert+3*numface) {
      mju_error("Wrong size in convex hull graph");
    }

    // add triangle data, reorient faces if flipped
    adr = 0;
    FORALLfacets {
      int ii = 0;
      int ind[3] = {0, 1, 2};
      if (facet->toporient) {
        ind[0] = 1;
        ind[1] = 0;
      }

      // copy triangle data
      FOREACHsetelement_(vertexT, facet->vertices, vertex1) {
        // make sure we have triangle: SHOULD NOT OCCUR
        if (ii>=3) {
          mju_error("Qhull did not return triangle");
        }

        face_globalid[adr + ind[ii++]] = qh_pointid(qh, vertex1->point);
      }

      // advance to next triangle
      adr += 3;
    }

    // free all
    qh_freeqhull(qh, !qh_ALL);
    qh_memfreeshort(qh, &curlong, &totlong);
    mju_free(data);

    // bad graph: delete
    if (!ok) {
      szgraph_ = 0;
      mju_free(graph_);
      graph_ = 0;
      mju_warning("Could not construct convex hull graph");
    }

    // replace global ids with local ids in edge data
    for (int i=0; i<numvert+3*numface; i++) {
      if (edge_localid[i]>=0) {
        // search vert_globalid for match
        int adr;
        for (adr=0; adr<numvert; adr++) {
          if (vert_globalid[adr]==edge_localid[i]) {
            edge_localid[i] = adr;
            break;
          }
        }

        // make sure we found a match: SHOULD NOT OCCUR
        if (adr>=numvert) {
          mju_error("Vertex id not found in convex hull");
        }
      }
    }
  }

  // longjmp error handler
  else {
    // free all
    qh_freeqhull(qh, !qh_ALL);
    qh_memfreeshort(qh, &curlong, &totlong);
    mju_free(data);
    if (graph_) {
      mju_free(graph_);
      szgraph_ = 0;
    }

    throw mjCError(this, "qhull error");
  }
}



// copy graph into face data
void mjCMesh::CopyGraph(void) {
  // only if face data is missing
  if (face_) {
    return;
  }

  // get info from graph, allocate
  int numvert = graph_[0];
  nface_ = graph_[1];
  face_ = (int*) mju_malloc(3*nface_*sizeof(int));

  // copy faces
  for (int i=0; i<nface_; i++) {
    // address in graph
    int j = 2 + 3*numvert + 3*nface_ + 3*i;

    // copy
    face_[3*i] = graph_[j];
    face_[3*i+1] = graph_[j+1];
    face_[3*i+2] = graph_[j+2];
  }
}



// compute vertex normals
void mjCMesh::MakeNormal(void) {
  // only if normal data is missing
  if (normal_) {
    return;
  }

  // allocate and clear normals
  nnormal_ = nvert_;
  normal_ = (float*) mju_malloc(3*nnormal_*sizeof(float));
  memset(normal_, 0, 3*nnormal_*sizeof(float));

  if (!facenormal_) {
    facenormal_ = (int*) mju_malloc(3*nface_*sizeof(int));
    memset(facenormal_, 0, 3*nface_*sizeof(int));
  }

  // loop over faces, accumulate vertex normals
  for (int i=0; i<nface_; i++) {
    // get vertex ids
    int vertid[3];
    for (int j=0; j<3; j++) {
      vertid[j] = face_[3*i+j];
    }

    // get triangle edges
    mjtNum vec01[3], vec02[3];
    for (int j=0; j<3; j++) {
      vec01[j] = vert_[3*vertid[1]+j] - vert_[3*vertid[0]+j];
      vec02[j] = vert_[3*vertid[2]+j] - vert_[3*vertid[0]+j];
    }

    // compute face normal
    mjtNum nrm[3];
    mju_cross(nrm, vec01, vec02);
    mjtNum area = mju_normalize3(nrm);

    // add normal to each vertex with weight = area
    for (int j=0; j<3; j++) {
      for (int k=0; k<3; k++) {
        normal_[3*vertid[j]+k] += nrm[k]*area;
      }
      facenormal_[3*i+j] = vertid[j];
    }
  }

  // remove large-angle faces
  if (!smoothnormal_) {
    // allocate removal and clear
    float* nremove = (float*) mju_malloc(3*nnormal_*sizeof(float));
    memset(nremove, 0, 3*nnormal_*sizeof(float));

    // remove contributions from faces at large angles with vertex normal
    for (int i=0; i<nface_; i++) {
      // get vertex ids
      int vertid[3];
      for (int j=0; j<3; j++) {
        vertid[j] = face_[3*i+j];
      }

      // get triangle edges
      mjtNum vec01[3], vec02[3];
      for (int j=0; j<3; j++) {
        vec01[j] = vert_[3*vertid[1]+j] - vert_[3*vertid[0]+j];
        vec02[j] = vert_[3*vertid[2]+j] - vert_[3*vertid[0]+j];
      }

      // compute face normal
      mjtNum nrm[3];
      mju_cross(nrm, vec01, vec02);
      mjtNum area = mju_normalize3(nrm);

      // compare to vertex normal, subtract contribution if dot product too small
      for (int j=0; j<3; j++) {
        // normalized vertex normal
        mjtNum vnrm[3] = {normal_[3*vertid[j]], normal_[3*vertid[j]+1], normal_[3*vertid[j]+2]};
        mju_normalize3(vnrm);

        // dot too small: remove
        if (mju_dot3(nrm, vnrm)<0.8) {
          for (int k=0; k<3; k++) {
            nremove[3*vertid[j]+k] += nrm[k]*area;
          }
        }
      }
    }

    // apply removal, free nremove
    for (int i=0; i<3*nnormal_; i++) {
      normal_[i] -= nremove[i];
    }
    mju_free(nremove);
  }

  // normalize normals
  for (int i=0; i<nnormal_; i++) {
    // compute length
    float len = sqrtf(normal_[3*i]*normal_[3*i] +
                      normal_[3*i+1]*normal_[3*i+1] +
                      normal_[3*i+2]*normal_[3*i+2]);

    // divide by length
    if (len>mjMINVAL)
      for (int j=0; j<3; j++) {
        normal_[3*i+j] /= len;
      } else {
        normal_[3*i] = normal_[3*i+1] = 0;
        normal_[3*i+2] = 1;
    }
  }
}



// compute face circumradii
void mjCMesh::MakeCenter(void) {
  if (center_) {
    return;
  }

  // allocate and clear
  center_ = (double*) mju_malloc(3*nface_*sizeof(double));
  memset(center_, 0, 3*nface_*sizeof(double));

  for (int i=0; i<nface_; i++) {
    // get vertex ids
    int* vertid = face_ + 3*i;

    // get triangle edges
    mjtNum a[3], b[3];
    for (int j=0; j<3; j++) {
      a[j] = vert_[3*vertid[0]+j] - vert_[3*vertid[2]+j];
      b[j] = vert_[3*vertid[1]+j] - vert_[3*vertid[2]+j];
    }

    // compute face normal
    mjtNum nrm[3];
    mju_cross(nrm, a, b);

    // compute circumradius
    mjtNum norm_a_2 = mju_dot3(a, a);
    mjtNum norm_b_2 = mju_dot3(b, b);
    mjtNum area = mju_norm3(nrm);

    // compute circumcenter
    mjtNum res[3], vec[3] = {
      norm_a_2 * b[0] - norm_b_2 * a[0],
      norm_a_2 * b[1] - norm_b_2 * a[1],
      norm_a_2 * b[2] - norm_b_2 * a[2]
    };
    mju_cross(res, vec, nrm);
    center_[3*i+0] = res[0]/(2*area*area) + vert_[3*vertid[2]+0];
    center_[3*i+1] = res[1]/(2*area*area) + vert_[3*vertid[2]+1];
    center_[3*i+2] = res[2]/(2*area*area) + vert_[3*vertid[2]+2];
  }
}



//------------------ class mjCSkin implementation --------------------------------------------------

// constructor
mjCSkin::mjCSkin(mjCModel* _model) {
  // set model pointer
  model = _model;

  // clear data
  file.clear();
  material.clear();
  rgba[0] = rgba[1] = rgba[2] = 0.5f;
  rgba[3] = 1.0f;
  inflate = 0;
  group = 0;

  vert.clear();
  texcoord.clear();
  face.clear();

  bodyname.clear();
  bindpos.clear();
  bindquat.clear();
  vertid.clear();
  vertweight.clear();
  bodyid.clear();

  matid = -1;
}



// destructor
mjCSkin::~mjCSkin() {
  file.clear();
  material.clear();
  vert.clear();
  texcoord.clear();
  face.clear();
  bodyname.clear();
  bindpos.clear();
  bindquat.clear();
  vertid.clear();
  vertweight.clear();
  bodyid.clear();
}



// compiler
void mjCSkin::Compile(const mjVFS* vfs) {
  // load file
  if (!file.empty()) {
    // make sure data is not present
    if (!vert.empty() ||
        !texcoord.empty() ||
        !face.empty() ||
        !bodyname.empty() ||
        !bindpos.empty() ||
        !bindquat.empty() ||
        !vertid.empty() ||
        !vertweight.empty() ||
        !bodyid.empty()) {
      throw mjCError(this, "Data already exists, trying to load from skin file: %s", file.c_str());
    }

    // remove path from file if necessary
    if (model->strippath) {
      file = mjuu_strippath(file);
    }

    // load SKN
    string ext = mjuu_getext(file);
    if (strcasecmp(ext.c_str(), ".skn")) {
      throw mjCError(this, "Unknown skin file type: %s", file.c_str());
    }

    string filename = mjuu_makefullname(model->modelfiledir, model->meshdir, file);
    mjResource* resource = LoadResource(filename, vfs);

    try {
      LoadSKN(resource);
      mju_closeResource(resource);
    } catch(mjCError err) {
      mju_closeResource(resource);
      throw err;
    }
  }

  // make sure all data is present
  if (vert.empty() ||
      face.empty() ||
      bodyname.empty() ||
      bindpos.empty() ||
      bindquat.empty() ||
      vertid.empty() ||
      vertweight.empty()) {
    throw mjCError(this, "Missing data in skin");
  }

  // check mesh sizes
  if (vert.size()%3) {
    throw mjCError(this, "Vertex data must be multiple of 3");
  }
  if (!texcoord.empty() && texcoord.size()!=2*vert.size()/3) {
    throw mjCError(this, "Vertex and texcoord data incompatible size");
  }
  if (face.size()%3) {
    throw mjCError(this, "Face data must be multiple of 3");
  }

  // check bone sizes
  size_t nbone = bodyname.size();
  if (bindpos.size()!=3*nbone) {
    throw mjCError(this, "Unexpected bindpos size in skin");
  }
  if (bindquat.size()!=4*nbone) {
    throw mjCError(this, "Unexpected bindquat size in skin");
  }
  if (vertid.size()!=nbone) {
    throw mjCError(this, "Unexpected vertid size in skin");
  }
  if (vertweight.size()!=nbone) {
    throw mjCError(this, "Unexpected vertweight size in skin");
  }

  // resolve body names
  bodyid.resize(nbone);
  for (int i=0; i<nbone; i++) {
    mjCBase* pbody = model->FindObject(mjOBJ_BODY, bodyname[i]);
    if (!pbody) {
      throw mjCError(this, "unknown body '%s' in skin", bodyname[i].c_str());
    }
    bodyid[i] = pbody->id;
  }

  // resolve material name
  mjCBase* pmat = model->FindObject(mjOBJ_MATERIAL, material);
  if (pmat) {
    matid = pmat->id;
  } else if (!material.empty()) {
      throw mjCError(this, "unkown material '%s' in skin", material.c_str());
  }

  // set total vertex weights to 0
  vector<float> vw;
  size_t nvert = vert.size()/3;
  vw.resize(nvert);
  fill(vw.begin(), vw.end(), 0.0f);

  // accumulate vertex weights from all bones
  for (int i=0; i<nbone; i++) {
    // make sure bone has vertices and sizes match
    size_t nbv = vertid[i].size();
    if (vertweight[i].size()!=nbv || nbv==0) {
      throw mjCError(this, "vertid and vertweight must have same non-zero size in skin");
    }

    // accumulate weights in global array
    for (int j=0; j<nbv; j++) {
      // get index and check range
      int jj = vertid[i][j];
      if (jj<0 || jj>=nvert) {
        throw mjCError(this, "vertid %d out of range in skin", NULL, jj);
      }

      // accumulate
      vw[jj] += vertweight[i][j];
    }
  }

  // check coverage
  for (int i=0; i<nvert; i++) {
    if (vw[i]<=mjMINVAL) {
      throw mjCError(this, "vertex %d must have positive total weight in skin", NULL, i);
    }
  }

  // normalize vertex weights
  for (int i=0; i<nbone; i++) {
    for (int j=0; j<vertid[i].size(); j++) {
      vertweight[i][j] /= vw[vertid[i][j]];
    }
  }

  // normalize bindquat
  for (int i=0; i<nbone; i++) {
    mjtNum quat[4] = {
      (mjtNum)bindquat[4*i],
      (mjtNum)bindquat[4*i+1],
      (mjtNum)bindquat[4*i+2],
      (mjtNum)bindquat[4*i+3]
    };
    mju_normalize4(quat);

    bindquat[4*i]   = (float) quat[0];
    bindquat[4*i+1] = (float) quat[1];
    bindquat[4*i+2] = (float) quat[2];
    bindquat[4*i+3] = (float) quat[3];
  }
}



// load skin in SKN BIN format
void mjCSkin::LoadSKN(mjResource* resource) {
  char* buffer = 0;
  int buffer_sz = mju_readResource(resource, (const void**)  &buffer);

  if (buffer_sz < 0) {
    throw mjCError(this, "could not read SKN file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw mjCError(this, "SKN file '%s' is empty", resource->name);
  }

  // make sure header is present
  if (buffer_sz<16) {
    throw mjCError(this, "missing header in SKN file '%s'", resource->name);
  }

  // get sizes from header
  int nvert = ((int*)buffer)[0];
  int ntexcoord = ((int*)buffer)[1];
  int nface = ((int*)buffer)[2];
  int nbone = ((int*)buffer)[3];

  // negative sizes not allowed
  if (nvert<0 || ntexcoord<0 || nface<0 || nbone<0) {
    throw mjCError(this, "negative size in header of SKN file '%s'", resource->name);
  }

  // make sure we have data for vert, texcoord, face
  if (buffer_sz < 16 + 12*nvert + 8*ntexcoord + 12*nface) {
    throw mjCError(this, "insufficient data in SKN file '%s'", resource->name);
  }

  // data pointer and counter
  float* pdata = (float*)(buffer+16);
  int cnt = 0;

  // copy vert
  if (nvert) {
    vert.resize(3*nvert);
    memcpy(vert.data(), pdata+cnt, 3*nvert*sizeof(float));
    cnt += 3*nvert;
  }

  // copy texcoord
  if (ntexcoord) {
    texcoord.resize(2*ntexcoord);
    memcpy(texcoord.data(), pdata+cnt, 2*ntexcoord*sizeof(float));
    cnt += 2*ntexcoord;
  }

  // copy face
  if (nface) {
    face.resize(3*nface);
    memcpy(face.data(), pdata+cnt, 3*nface*sizeof(int));
    cnt += 3*nface;
  }

  // allocate bone arrays
  bodyname.clear();
  bindpos.resize(3*nbone);
  bindquat.resize(4*nbone);
  vertid.resize(nbone);
  vertweight.resize(nbone);

  // read bones
  for (int i=0; i<nbone; i++) {
    // check size
    if (buffer_sz/4-4-cnt < 18) {
      throw mjCError(this, "insufficient data in SKN file '%s', bone %d", resource->name, i);
    }

    // read name
    char txt[40];
    strncpy(txt, (char*)(pdata+cnt), 39);
    txt[39] = '\0';
    cnt += 10;
    bodyname.push_back(txt);

    // read bindpos
    memcpy(bindpos.data()+3*i, pdata+cnt, 3*sizeof(float));
    cnt += 3;

    // read bind quat
    memcpy(bindquat.data()+4*i, pdata+cnt, 4*sizeof(float));
    cnt += 4;

    // read vertex count
    int vcount = *(int*)(pdata+cnt);
    cnt += 1;

    // check for negative
    if (vcount<1) {
      throw mjCError(this, "vertex count must be positive in SKN file '%s', bone %d",
                     resource->name, i);
    }

    // check size
    if (buffer_sz/4-4-cnt < 2*vcount) {
      throw mjCError(this, "insufficient vertex data in SKN file '%s', bone %d",
                     resource->name, i);
    }

    // read vertid
    vertid[i].resize(vcount);
    memcpy(vertid[i].data(), (int*)(pdata+cnt), vcount*sizeof(int));
    cnt += vcount;

    // read vertweight
    vertweight[i].resize(vcount);
    memcpy(vertweight[i].data(), (int*)(pdata+cnt), vcount*sizeof(int));
    cnt += vcount;
  }

  // check final size
  if (buffer_sz != 16+4*cnt) {
    throw mjCError(this, "unexpected buffer size in SKN file '%s'", resource->name);
  }
}



//------------------ class mjCFlex implementation --------------------------------------------------

// hash function for std::pair
struct PairHash
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

// simplex connectivity
constexpr int kNumEdges[3] = {1, 3, 6};
constexpr int eledge[3][6][2] = {{{ 0,  1}, {-1, -1}, {-1, -1},
                                  {-1, -1}, {-1, -1}, {-1, -1}},
                                 {{ 1,  2}, { 2,  0}, { 0,  1},
                                  {-1, -1}, {-1, -1}, {-1, -1}},
                                 {{ 0,  1}, { 1,  2}, { 2,  0},
                                  { 2,  3}, { 0,  3}, { 1,  3}}};

// constructor
mjCFlex::mjCFlex(mjCModel* _model) {
  // set model
  model = _model;

  // set contact defaults
  contype = 1;
  conaffinity = 1;
  condim = 3;
  priority = 0;
  mjuu_setvec(friction, 1, 0.005, 0.0001);
  solmix = 1.0;
  mj_defaultSolRefImp(solref, solimp);
  margin = 0;
  gap = 0;

  // set other defaults
  dim = 2;
  radius = 0.005;
  internal = true;
  flatskin = false;
  selfcollide = mjFLEXSELF_AUTO;
  activelayers = 1;
  group = 0;
  edgestiffness = 0;
  edgedamping = 0;
  material.clear();
  rgba[0] = rgba[1] = rgba[2] = 0.5f;
  rgba[3] = 1.0f;

  // clear internal variables
  nvert = 0;
  nedge = 0;
  nelem = 0;
  matid = -1;
  rigid = false;
  centered = false;
}



// compiler
void mjCFlex::Compile(const mjVFS* vfs) {
  // set nelem; check sizes
  if (dim<1 || dim>3) {
      throw mjCError(this, "dim must be 1, 2 or 3");
  }
  if (elem.empty()) {
      throw mjCError(this, "elem is empty");
  }
  if (elem.size() % (dim+1)) {
      throw mjCError(this, "elem size must be multiple of (dim+1)");
  }
  if (vertbody.empty()) {
      throw mjCError(this, "vertbody is empty");
  }
  if (vert.size() % 3) {
      throw mjCError(this, "vert size must be a multiple of 3");
  }
  if (edgestiffness>0 && dim>1) {
    throw mjCError(this, "edge stiffness only available for dim=1, please use elasticity plugins");
  }
  nelem = (int)elem.size()/(dim+1);

  // set nvert, rigid, centered; check size
  if (vert.empty()) {
    centered = true;
    nvert = (int)vertbody.size();
  }
  else {
    nvert = (int)vert.size()/3;
    if (vertbody.size()==1) {
      rigid = true;
    }
  }
  if (nvert<dim+1) {
    throw mjCError(this, "not enough vertices");
  }

  // check elem vertex ids
  for (int i=0; i<(int)elem.size(); i++) {
    if (elem[i]<0 || elem[i]>=nvert) {
      throw mjCError(this, "elem vertex id out of range");
    }
  }

  // check texcoord
  if (!texcoord.empty() && texcoord.size()!=2*nvert) {
    throw mjCError(this, "two texture coordinates per vertex expected");
  }

  // resolve material name
  mjCBase* pmat = model->FindObject(mjOBJ_MATERIAL, material);
  if (pmat) {
    matid = pmat->id;
  } else if (!material.empty()) {
      throw mjCError(this, "unkown material '%s' in flex", material.c_str());
  }

  // resolve body ids
  for (int i=0; i<(int)vertbody.size(); i++) {
    mjCBase* pbody = model->FindObject(mjOBJ_BODY, vertbody[i]);
    if (pbody) {
      vertbodyid.push_back(pbody->id);
    } else {
        throw mjCError(this, "unkown body '%s' in flex", vertbody[i].c_str());
    }
  }

  // process elements
  for (int e=0; e<(int)elem.size()/(dim+1); e++) {
    // make sorted copy of element
    vector<int> el;
    el.assign(elem.begin()+e*(dim+1), elem.begin()+(e+1)*(dim+1));
    std::sort(el.begin(), el.end());

    // check for repeated vertices
    for (int k=0; k<dim; k++) {
      if (el[k]==el[k+1]) {
        throw mjCError(this, "repeated vertex in element");
      }
    }
  }

  // determine rigid if not already set
  if (!rigid) {
    rigid = true;
    for (int i=1; i<(int)vertbodyid.size(); i++) {
      if (vertbodyid[i]!=vertbodyid[0]) {
        rigid = false;
        break;
      }
    }
  }

  // determine centered if not already set
  if (!centered) {
    centered = true;
    for (int i=0; i<(int)vert.size(); i++) {
      if (vert[i]!=0) {
        centered = false;
        break;
      }
    }
  }

  // compute global vertex positions
  vertxpos = vector<mjtNum> (3*nvert);
  for (int i=0; i<nvert; i++) {
    // get body id, set vertxpos = body.xpos0
    int b = rigid ? vertbodyid[0] : vertbodyid[i];
    mju_copy3(vertxpos.data()+3*i, model->bodies[b]->xpos0);

    // add vertex offset within body if not centered
    if (!centered) {
      mjtNum offset[3];
      mju_rotVecQuat(offset, vert.data()+3*i, model->bodies[b]->xquat0);
      mju_addTo3(vertxpos.data()+3*i, offset);
    }
  }

  // reorder tetrahedra so right-handed face orientation is outside
  // faces are (0,1,2); (0,2,3); (0,3,1); (1,3,2)
  if (dim==3) {
    for (int e=0; e<nelem; e++) {
      const int* edata = elem.data() + e*(dim+1);
      mjtNum* v0 = vertxpos.data() + 3*edata[0];
      mjtNum* v1 = vertxpos.data() + 3*edata[1];
      mjtNum* v2 = vertxpos.data() + 3*edata[2];
      mjtNum* v3 = vertxpos.data() + 3*edata[3];
      mjtNum v01[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
      mjtNum v02[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
      mjtNum v03[3] = {v3[0]-v0[0], v3[1]-v0[1], v3[2]-v0[2]};

      // detect wrong orientation
      mjtNum nrm[3];
      mju_cross(nrm, v01, v02);
      if (mju_dot3(nrm, v03)>0) {
        // flip orientation
        int tmp = elem[e*(dim+1)+1];
        elem[e*(dim+1)+1] = elem[e*(dim+1)+2];
        elem[e*(dim+1)+2] = tmp;
      }
    }
  }

  // create edges
  std::vector<int> edgeidx(elem.size()*kNumEdges[dim-1]);

  // map from edge vertices to their index in `edges` vector
  std::unordered_map<std::pair<int, int>, int, PairHash> edge_indices;

  // insert local edges into global vector
  for (int f = 0; f < (int)elem.size()/(dim+1); f++) {
    int* v = elem.data() + f*(dim+1);
    for (int e = 0; e < kNumEdges[dim-1]; e++) {
      auto pair = std::pair(
        std::min(v[eledge[dim-1][e][0]], v[eledge[dim-1][e][1]]),
        std::max(v[eledge[dim-1][e][0]], v[eledge[dim-1][e][1]])
      );

      // if edge is already present in the vector only store its index
      auto [it, inserted] = edge_indices.insert({pair, nedge});

      if (inserted) {
        edge.push_back(pair);
        edgeidx[f*kNumEdges[dim-1]+e] = nedge++;
      } else {
        edgeidx[f*kNumEdges[dim-1]+e] = it->second;
      }
    }
  }

  // set size
  nedge = (int)edge.size();

  // add plugins
  std::string userface, useredge;
  mjXUtil::Vector2String(userface, elem);
  mjXUtil::Vector2String(useredge, edgeidx);

  for (int i=0; i<(int)vertbodyid.size(); i++) {
    if (model->bodies[vertbodyid[i]]->plugin_instance) {
      model->bodies[vertbodyid[i]]->plugin_instance->config_attribs["face"] = userface;
      model->bodies[vertbodyid[i]]->plugin_instance->config_attribs["edge"] = useredge;
    }
  }

  // create shell fragments and element-vertex collision pairs
  CreateShellPair();

  // create bounding volume hierarchy
  CreateBVH();
}



// create flex BVH
void mjCFlex::CreateBVH(void) {
  // init bounding volume object
  mjCBoundingVolume bv;
  bv.contype = contype;
  bv.conaffinity = conaffinity;
  bv.quat = NULL;

  // allocate element bounding boxes
  vector<mjtNum> elemaabb(6*nelem);

  // construct element bounding boxes, add to hierarchy
  for (int e=0; e<nelem; e++) {
    const int* edata = elem.data() + e*(dim+1);

    // skip inactive in 3D
    if (dim==3 && elemlayer[e]>=activelayers) {
      continue;
    }

    // compute min and max along each global axis
    mjtNum xmin[3], xmax[3];
    mju_copy3(xmin, vertxpos.data() + 3*edata[0]);
    mju_copy3(xmax, vertxpos.data() + 3*edata[0]);
    for (int i=1; i<=dim; i++) {
      for (int j=0; j<3; j++) {
        xmin[j] = mjMIN(xmin[j], vertxpos[3*edata[i]+j]);
        xmax[j] = mjMAX(xmax[j], vertxpos[3*edata[i]+j]);
      }
    }

    // compute aabb (center, size)
    elemaabb[6*e+0] = 0.5*(xmax[0]+xmin[0]);
    elemaabb[6*e+1] = 0.5*(xmax[1]+xmin[1]);
    elemaabb[6*e+2] = 0.5*(xmax[2]+xmin[2]);
    elemaabb[6*e+3] = 0.5*(xmax[0]-xmin[0]) + radius;
    elemaabb[6*e+4] = 0.5*(xmax[1]-xmin[1]) + radius;
    elemaabb[6*e+5] = 0.5*(xmax[2]-xmin[2]) + radius;

    // add bounding volume for this element
    bv.id = e;
    bv.aabb = elemaabb.data() + 6*e;
    bv.pos = bv.aabb;
    tree.AddBoundingVolume(bv);
  }

  // create hierarchy
  tree.CreateBVH();
}



// create shells and element-vertex collision pairs
void mjCFlex::CreateShellPair(void) {
  vector<vector<int>> fragspec(nelem*(dim+1));   // [sorted frag vertices, elem, original frag vertices]
  vector<vector<int>> connectspec;               // [elem1, elem2, common sorted frag vertices]
  vector<bool> border(nelem, false);             // is element on the border
  vector<bool> borderfrag(nelem*(dim+1), false); // is fragment on the border

  // make fragspec
  for (int e=0; e<nelem; e++) {
    int n = e*(dim+1);

    // element vertices in original (unsorted) order
    vector<int> el;
    el.assign(elem.begin()+n, elem.begin()+n+dim+1);

    // line: 2 vertex fragments
    if (dim==1) {
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(e);
      fragspec[n].push_back(el[0]);

      fragspec[n+1].push_back(el[1]);
      fragspec[n+1].push_back(e);
      fragspec[n+1].push_back(el[1]);
    }

    // triangle: 3 edge fragments
    else if (dim==2) {
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(el[1]);
      fragspec[n].push_back(e);
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(el[1]);

      fragspec[n+2].push_back(el[1]);
      fragspec[n+2].push_back(el[2]);
      fragspec[n+2].push_back(e);
      fragspec[n+2].push_back(el[1]);
      fragspec[n+2].push_back(el[2]);

      fragspec[n+1].push_back(el[2]);
      fragspec[n+1].push_back(el[0]);
      fragspec[n+1].push_back(e);
      fragspec[n+1].push_back(el[2]);
      fragspec[n+1].push_back(el[0]);
    }

    // tetrahedron: 4 face fragments
    else {
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(el[1]);
      fragspec[n].push_back(el[2]);
      fragspec[n].push_back(e);
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(el[1]);
      fragspec[n].push_back(el[2]);

      fragspec[n+2].push_back(el[0]);
      fragspec[n+2].push_back(el[2]);
      fragspec[n+2].push_back(el[3]);
      fragspec[n+2].push_back(e);
      fragspec[n+2].push_back(el[0]);
      fragspec[n+2].push_back(el[2]);
      fragspec[n+2].push_back(el[3]);

      fragspec[n+1].push_back(el[0]);
      fragspec[n+1].push_back(el[3]);
      fragspec[n+1].push_back(el[1]);
      fragspec[n+1].push_back(e);
      fragspec[n+1].push_back(el[0]);
      fragspec[n+1].push_back(el[3]);
      fragspec[n+1].push_back(el[1]);

      fragspec[n+3].push_back(el[1]);
      fragspec[n+3].push_back(el[3]);
      fragspec[n+3].push_back(el[2]);
      fragspec[n+3].push_back(e);
      fragspec[n+3].push_back(el[1]);
      fragspec[n+3].push_back(el[3]);
      fragspec[n+3].push_back(el[2]);
    }
  }

  // sort first segment of each fragspec
  if (dim>1) {
    for (int n=0; n<nelem*(dim+1); n++) {
      std::sort(fragspec[n].begin(), fragspec[n].begin()+dim);
    }
  }

  // sort fragspec
  std::sort(fragspec.begin(), fragspec.end());

  // make border and connectspec, record borderfrag
  int cnt = 1;
  for (int n=1; n<nelem*(dim+1); n++) {
    // extract frag vertices, without elem
    vector<int> previous = {fragspec[n-1].begin(), fragspec[n-1].begin()+dim};
    vector<int> current = {fragspec[n].begin(), fragspec[n].begin()+dim};

    // same sequential fragments
    if (previous==current) {
      // found pair of elements connected by common fragment
      vector<int> connect;
      connect.insert(connect.end(), fragspec[n-1][dim]);
      connect.insert(connect.end(), fragspec[n][dim]);
      connect.insert(connect.end(), fragspec[n].begin(), fragspec[n].begin()+dim);
      connectspec.push_back(connect);

      // count same sequential fragments
      cnt++;
    }

    // different sequential fragments
    else {
      // found border fragment
      if (cnt==1) {
        border[fragspec[n-1][dim]] = true;
        borderfrag[n-1] = true;
      }

      // reset count
      cnt = 1;
    }
  }

  // last fragment is border
  if (cnt==1) {
    int n = nelem*(dim+1);
    border[fragspec[n-1][dim]] = true;
    borderfrag[n-1] = true;
  }

  // create shell
  for (int i=0; i<(int)borderfrag.size(); i++) {
    if (borderfrag[i]) {
      // add fragment vertices, in original order
      shell.insert(shell.end(), fragspec[i].begin()+dim+1, fragspec[i].end());
    }
  }

  // compute elemlayer (distance from border) via value iteration in 3D
  if (dim<3) {
    elemlayer = vector<int> (nelem, 0);
  }
  else {
    elemlayer = vector<int> (nelem, nelem+1);   // init with greater than max value
    for (int e=0; e<nelem; e++) {
      if (border[e]) {
        elemlayer[e] = 0;                       // set border elements to 0
      }
    }

    bool change = true;
    while (change) {                            // repeat while changes are happening
      change = false;

      // process edges of element connectivity graph
      for (int i=0; i<(int)connectspec.size(); i++) {
        int e1 = connectspec[i][0];             // get element pair for this edge
        int e2 = connectspec[i][1];
          if (elemlayer[e1]>elemlayer[e2]+1) {
            elemlayer[e1] = elemlayer[e2]+1;    // better value found for e1: update
          change = true;
        } else if (elemlayer[e2]>elemlayer[e1]+1) {
          elemlayer[e2] = elemlayer[e1]+1;      // better value found for e2: update
          change = true;
        }
      }
    }
  }

  // create evpairs in 1D and 2D
  if (dim<3) {
    // process connected element pairs containing a border element
    for (int n=0; n<(int)connectspec.size(); n++) {
      if (border[connectspec[n][0]] || border[connectspec[n][1]]) {
        // extract common fragment
        vector<int> frag = {connectspec[n].begin()+2, connectspec[n].end()};

        // process both elements
        for (int ei=0; ei<2; ei++) {
          const int* edata = elem.data() + connectspec[n][ei]*(dim+1);

          // find element vertex that is not in the common fragment
          for (int i=0; i<=dim; i++) {
            if (frag.end() == std::find(frag.begin(), frag.end(), edata[i])) {
              // add ev pair, involving the other element in connectspec
              evpair.push_back(connectspec[n][1-ei]);
              evpair.push_back(edata[i]);

              // one such vertex exists
              break;
            }
          }
        }
      }
    }
  }
}
