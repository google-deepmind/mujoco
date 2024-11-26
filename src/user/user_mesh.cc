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
#include <climits>
#include <cmath>
#include <csetjmp>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mujoco/mjspec.h>
#include "user/user_api.h"

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
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include "engine/engine_crossplatform.h"
#include "engine/engine_plugin.h"
#include "engine/engine_util_errmem.h"
#include "user/user_cache.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_resource.h"
#include "user/user_util.h"
#include <tiny_obj_loader.h>

extern "C" {
#include "qhull_ra.h"
}

namespace {
  using mujoco::user::VectorToString;
  using mujoco::user::FilePath;
  using std::max;
  using std::min;
}  // namespace

// compute triangle area, surface normal, center
static double _triangle(double* normal, double* center,
                        const float* v1, const float* v2, const float* v3) {
  // center
  if (center) {
    for (int i=0; i < 3; i++) {
      center[i] = (v1[i] + v2[i] + v3[i])/3;
    }
  }

  // normal = (v2-v1) cross (v3-v1)
  double b[3] = { v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2] };
  double c[3] = { v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2] };
  mjuu_crossvec(normal, b, c);

  // get length
  double len = sqrt(mjuu_dot3(normal, normal));

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

// Read data of type T from a potentially unaligned buffer pointer.
template <typename T>
static void ReadFromBuffer(T* dst, const char* src) {
  std::memcpy(dst, src, sizeof(T));
}

//------------------ class mjCMesh implementation --------------------------------------------------

mjCMesh::mjCMesh(mjCModel* _model, mjCDef* _def) {
  mjs_defaultMesh(&spec);
  elemtype = mjOBJ_MESH;

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
  szgraph_ = 0;
  center_ = NULL;
  graph_ = NULL;
  needhull_ = false;
  maxhullvert_ = -1;
  invalidorientation_.first = -1;
  invalidorientation_.second = -1;
  validarea_ = true;
  validvolume_ = 1;
  valideigenvalue_ = true;
  validinequality_ = true;
  processed_ = false;
  visual_ = true;

  // reset to default if given
  if (_def) {
    *this = _def->Mesh();
  }

  // set model, def
  model = _model;
  classname = (_def ? _def->name : (_model ? "main" : ""));

  // in case this body is not compiled
  CopyFromSpec();

  // point to local
  PointToLocal();
}



mjCMesh::mjCMesh(const mjCMesh& other) {
  *this = other;
}



mjCMesh& mjCMesh::operator=(const mjCMesh& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCMesh_*>(this) = static_cast<const mjCMesh_&>(other);
    *static_cast<mjsMesh*>(this) = static_cast<const mjsMesh&>(other);
    if (other.center_) {
      size_t ncenter = 3*other.nface()*sizeof(double);
      this->center_ = (double*)mju_malloc(ncenter);
      memcpy(this->center_, other.center_, ncenter);
    } else {
      this->center_ = NULL;
    }
    if (other.graph_) {
      size_t szgraph = szgraph_*sizeof(int);
      this->graph_ = (int*)mju_malloc(szgraph);
      memcpy(this->graph_, other.graph_, szgraph);
    } else {
      this->graph_ = NULL;
    }
  }
  PointToLocal();
  return *this;
}



void mjCMesh::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.file = &spec_file_;
  spec.content_type = &spec_content_type_;
  spec.uservert = &spec_vert_;
  spec.usernormal = &spec_normal_;
  spec.userface = &spec_face_;
  spec.usertexcoord = &spec_texcoord_;
  spec.userfacetexcoord = &spec_facetexcoord_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = &plugin_instance_name;
  spec.info = &info;
  file = nullptr;
  content_type = nullptr;
  uservert = nullptr;
  usernormal = nullptr;
  userface = nullptr;
  usertexcoord = nullptr;
  userfacetexcoord = nullptr;
}



void mjCMesh::NameSpace(const mjCModel* m) {
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
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }
}



void mjCMesh::CopyFromSpec() {
  *static_cast<mjsMesh*>(this) = spec;
  file_ = spec_file_;
  content_type_ = spec_content_type_;
  vert_ = spec_vert_;
  normal_ = spec_normal_;
  face_ = spec_face_;
  texcoord_ = spec_texcoord_;
  facetexcoord_ = spec_facetexcoord_;
  maxhullvert_ = spec.maxhullvert;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;

  // clear precompiled asset. TODO: use asset cache
  if (center_) mju_free(center_);
  if (graph_) mju_free(graph_);
  szgraph_ = 0;
  center_ = NULL;
  graph_ = NULL;

  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = mjuu_strippath(file_);
    name = mjuu_stripext(stripped);
  }
}



void mjCMesh::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



mjCMesh::~mjCMesh() {
  if (center_) mju_free(center_);
  if (graph_) mju_free(graph_);
  if (spec.plugin.active && spec.plugin.name->empty() && model) {
    model->DeleteElement(spec.plugin.element);
  }
}



// generate mesh using marching cubes
void mjCMesh::LoadSDF() {
  if (plugin_name.empty() && plugin_instance_name.empty()) {
    throw mjCError(
        this, "neither 'plugin' nor 'instance' is specified for mesh '%s', (id = %d)",
        name.c_str(), id);
  }

  if (scale[0] != 1 || scale[1] != 1 || scale[2] != 1) {
    throw mjCError(this, "attribute scale is not compatible with SDFs in mesh '%s', (id = %d)",
                   name.c_str(), id);
  }

  mjCPlugin* plugin_instance = static_cast<mjCPlugin*>(plugin.element);
  model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
  plugin.element = plugin_instance;
  const mjpPlugin* pplugin = mjp_getPluginAtSlot(plugin_instance->plugin_slot);
  if (!(pplugin->capabilityflags & mjPLUGIN_SDF)) {
    throw mjCError(this, "plugin '%s' does not support signed distance fields", pplugin->name);
  }

  std::vector<mjtNum> attributes(pplugin->nattribute, 0);
  std::vector<const char*> names(pplugin->nattribute, 0);
  std::vector<const char*> values(pplugin->nattribute, 0);
  for (int i=0; i < pplugin->nattribute; i++) {
    names[i] = pplugin->attributes[i];
    values[i] = plugin_instance->config_attribs[names[i]].c_str();
  }

  if (pplugin->sdf_attribute) {
    pplugin->sdf_attribute(attributes.data(), names.data(), values.data());
  }

  mjtNum aabb[6] = {0};
  pplugin->sdf_aabb(aabb, attributes.data());
  mjtNum total = aabb[3] + aabb[4] + aabb[5];

  const double n = 300;
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
        field[(k * ny + j) * nx + i] =  pplugin->sdf_staticdistance(point, attributes.data());
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

  vert_ = std::move(uservert);
  normal_ = std::move(usernormal);
  face_ = std::move(userface);
  delete[] field;
}

void mjCMesh::CacheMesh(mjCCache* cache, const mjResource* resource,
                        std::string_view asset_type) {
  if (cache == nullptr) return;
  if (asset_type != "model/obj") return;  // only OBJ files are cached

  // cache mesh data into new mesh object
  mjCMesh *mesh =  new mjCMesh();
  mesh->vert_ = vert_;
  mesh->normal_ = normal_;
  mesh->texcoord_ = texcoord_;
  mesh->face_ = face_;
  mesh->facetexcoord_ = facetexcoord_;
  mesh->facenormal_ = facenormal_;
  mesh->vertex_index_ = vertex_index_;
  mesh->normal_index_ = normal_index_;
  mesh->texcoord_index_ = texcoord_index_;
  mesh->num_face_vertices_ = num_face_vertices_;

  // calculate estimated size of mesh
  std::size_t size = sizeof(mjCMesh)
      + (sizeof(float) * vert_.size())
      + (sizeof(float) * normal_.size())
      + (sizeof(float) * texcoord_.size())
      + (sizeof(int) * face_.size())
      + (sizeof(int) * facetexcoord_.size())
      + (sizeof(int) * facenormal_.size())
      + (sizeof(int) * vertex_index_.size())
      + (sizeof(int) * normal_index_.size())
      + (sizeof(int) * texcoord_index_.size())
      + (sizeof(unsigned char) * num_face_vertices_.size());

  std::shared_ptr<const void> cached_data(mesh, +[](const void* data) {
    const mjCMesh* mesh = static_cast<const mjCMesh*>(data);
    delete mesh;
  });
  cache->Insert("", resource, cached_data, size);
}

// compiler
void mjCMesh::Compile(const mjVFS* vfs) {
  CopyFromSpec();
  visual_ = true;
  std::string asset_type = GetAssetContentType(file_, content_type_);
  mjResource* resource = nullptr;
  mjCCache *cache = reinterpret_cast<mjCCache*>(mj_globalCache());

  // load file
  if (!file_.empty()) {
    vert_.clear();
    face_.clear();
    normal_.clear();
    texcoord_.clear();
    facenormal_.clear();
    facetexcoord_.clear();

    // copy paths from model if not already defined
    if (modelfiledir_.empty()) {
      modelfiledir_ = FilePath(model->modelfiledir_);
    }
    if (meshdir_.empty()) {
      meshdir_ = FilePath(model->meshdir_);
    }

    // remove path from file if necessary
    if (model->strippath) {
      file_ = mjuu_strippath(file_);
    }

    if (asset_type.empty()) {
      throw mjCError(this, "unknown mesh content type for file: '%s'", file_.c_str());
    }

    if (asset_type != "model/stl" && asset_type != "model/obj"
        && asset_type != "model/vnd.mujoco.msh") {
      throw mjCError(this, "unsupported content type: '%s'", asset_type.c_str());
    }

    FilePath filename = meshdir_ + FilePath(file_);
    resource = LoadResource(modelfiledir_.Str(), filename.Str(), vfs);

    // try loading from cache
    if (cache != nullptr && LoadCachedMesh(cache, resource)) {
      mju_closeResource(resource);
      resource = nullptr;
    }

    if (resource != nullptr) {
      try {
        if (asset_type == "model/stl") {
          LoadSTL(resource);
        } else if (asset_type == "model/obj"){
          LoadOBJ(resource);
        } else {
          LoadMSH(resource);
        }
      } catch (mjCError err) {
        mju_closeResource(resource);
        throw err;
      }

      CacheMesh(cache, resource, asset_type);
      mju_closeResource(resource);
    }

    // check repeated mesh data
    if (!vert_.empty() && !spec_vert_.empty()) {
      throw mjCError(this, "repeated vertex specification");
    } else if (vert_.empty()) {
      vert_ = spec_vert_;
    }
    if (!normal_.empty() && !spec_normal_.empty()) {
      throw mjCError(this, "repeated normal specification");
    } else if (normal_.empty()) {
      normal_ = spec_normal_;
    }
    if (!texcoord_.empty() && !spec_texcoord_.empty()) {
      throw mjCError(this, "repeated texcoord specification");
    } else if (texcoord_.empty()) {
      texcoord_ = spec_texcoord_;
    }
    if (!face_.empty() && !spec_face_.empty()) {
      throw mjCError(this, "repeated face specification");
    } else if (face_.empty()) {
      face_ = spec_face_;
    }
    if (!facenormal_.empty() && !spec_normal_.empty()) {
      throw mjCError(this, "repeated facenormal specification");
    } else if (facenormal_.empty()) {
      facenormal_ = spec_facenormal_;
    }
    if (!facetexcoord_.empty() && !spec_facetexcoord_.empty()) {
      throw mjCError(this, "repeated facetexcoord specification");
    } else if (facetexcoord_.empty()) {
      facetexcoord_ = spec_facetexcoord_;
    }
  } else if (plugin.active) {
    LoadSDF();  // create using marching cubes
  }

  // check sizes
  if (vert_.size() < 12) throw mjCError(this, "at least 4 vertices required");
  if (vert_.size() % 3) throw mjCError(this, "vertex data must be a multiple of 3");
  if (normal_.size() % 3) throw mjCError(this, "normal data must be a multiple of 3");
  if (texcoord_.size() % 2) throw mjCError(this, "texcoord must be a multiple of 2");
  if (face_.size() % 3) throw mjCError(this, "face data must be a multiple of 3");

  // check texcoord size if no face texcoord indices are given
  if (!texcoord_.empty() && texcoord_.size() != 2 * nvert() &&
      facetexcoord_.empty() && asset_type != "model/obj") {
    throw mjCError(this,
        "texcoord must be 2*nv if face texcoord indices are not provided in an OBJ file");
  }

  // check vertices exist
  for (int i=0; i < face_.size(); i++) {
    if (face_[i] >= nvert() || face_[i] < 0) {
      throw mjCError(this, "in face %d, vertex index %d does not exist",
                      nullptr, i / 3, face_[i]);
    }
  }

  // create half-edge structure (if mesh was in XML)
  if (halfedge_.empty()) {
    for (int i=0; i < face_.size()/3; i++) {
      int v0 = face_[3*i+0];
      int v1 = face_[3*i+1];
      int v2 = face_[3*i+2];
      double normal[3];
      float* vtx = vert_.data();
      if (_triangle(normal, nullptr, vtx+3*v0, vtx+3*v1, vtx+3*v2)>sqrt(mjMINVAL)) {
        halfedge_.push_back(std::pair(v0, v1));
        halfedge_.push_back(std::pair(v1, v2));
        halfedge_.push_back(std::pair(v2, v0));
      } else {
        // TODO(b/255525326)
      }
    }
  }

  // check vertices exist
  for (auto vertex_index : face_) {
    if (vertex_index>=nvert() || vertex_index < 0) {
      throw mjCError(this, "found index in userface that exceeds uservert size.");
    }
  }

  // check for inconsistent face orientations
  if (!halfedge_.empty()) {
    std::stable_sort(halfedge_.begin(), halfedge_.end());
    auto iterator = std::adjacent_find(halfedge_.begin(), halfedge_.end());
    if (iterator != halfedge_.end()) {
      invalidorientation_.first = iterator->first+1;
      invalidorientation_.second = iterator->second+1;
    }
  }

  // require vertices
  if (vert_.empty()) {
    throw mjCError(this, "no vertices");
  }

  // make graph describing convex hull
  if (needhull_ || face_.empty()) {
    MakeGraph();
  }

  // no faces: copy from convex hull
  if (face_.empty()) {
    CopyGraph();
  }

  // no normals: make
  if (normal_.empty()) {
    MakeNormal();
  }

  // check facenormal size
  if (!facenormal_.empty() && facenormal_.size()!=3*nface()) {
    throw mjCError(this, "face data must have the same size as face normal data");
  }

  // no facetexcoord: copy from faces
  if (facetexcoord_.empty() && !texcoord_.empty()) {
    facetexcoord_.assign(3*nface(), 0);
    memcpy(facetexcoord_.data(), face_.data(), 3*nface()*sizeof(int));
  }

  // facenormal might not exist if usernormal was specified
  if (facenormal_.empty()) {
    facenormal_.assign(3*nface(), 0);
    memcpy(facenormal_.data(), face_.data(), 3*nface()*sizeof(int));
  }

  // scale, center, orient, compute mass and inertia
  Process();
  processed_ = true;

  // no radii: make
  if (!center_) {
    MakeCenter();
  }

  // make bounding volume hierarchy
  if (tree_.Bvh().empty()) {
    face_aabb_.assign(6*nface(), 0);
    tree_.AllocateBoundingVolumes(nface());
    for (int i=0; i < nface(); i++) {
      SetBoundingVolume(i);
    }
    tree_.CreateBVH();
  }
}



// get bounding volume
void mjCMesh::SetBoundingVolume(int faceid) {
  mjCBoundingVolume* node = tree_.GetBoundingVolume(faceid);
  node->SetId(faceid);
  node->conaffinity = 1;
  node->contype = 1;
  node->pos = center_ + 3*faceid;
  node->quat = NULL;
  double face_aamm[6] = {1E+10, 1E+10, 1E+10, -1E+10, -1E+10, -1E+10};
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
  node->aabb = face_aabb_.data() + 6*faceid;
}



// get position
double* mjCMesh::GetPosPtr(mjtGeomInertia type) {
  if (type==mjINERTIA_SHELL) {
    return pos_surface_;
  } else {
    return pos_volume_;
  }
}



// get orientation
double* mjCMesh::GetQuatPtr(mjtGeomInertia type) {
  if (type==mjINERTIA_SHELL) {
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
  return !texcoord_.empty();
}



void mjCMesh::CopyVert(float* arr) const {
  std::copy(vert_.begin(), vert_.end(), arr);
}



void mjCMesh::CopyNormal(float* arr) const {
  std::copy(normal_.begin(), normal_.end(), arr);
}



void mjCMesh::CopyFace(int* arr) const {
  std::copy(face_.begin(), face_.end(), arr);
}



void mjCMesh::CopyFaceTexcoord(int* arr) const {
  std::copy(facetexcoord_.begin(), facetexcoord_.end(), arr);
}



void mjCMesh::CopyFaceNormal(int* arr) const {
  std::copy(facenormal_.begin(), facenormal_.end(), arr);
}



void mjCMesh::CopyTexcoord(float* arr) const {
  std::copy(texcoord_.begin(), texcoord_.end(), arr);
}



void mjCMesh::CopyGraph(int* arr) const {
  std::copy(graph_, graph_+szgraph_, arr);
}



void mjCMesh::DelTexcoord() {
  texcoord_.clear();
}



// set geom size to match mesh
void mjCMesh::FitGeom(mjCGeom* geom, double* meshpos) {
  // copy mesh pos into meshpos
  mjuu_copyvec(meshpos, GetPosPtr(geom->typeinertia), 3);

  // use inertial box
  if (!model->compiler.fitaabb) {
    // get inertia box type (shell or volume)
    double* boxsz = GetInertiaBoxPtr(geom->typeinertia);
    switch (geom->type) {
    case mjGEOM_SPHERE:
      geom->size[0] = (boxsz[0] + boxsz[1] + boxsz[2])/3;
      break;

    case mjGEOM_CAPSULE:
      geom->size[0] = (boxsz[0] + boxsz[1])/2;
      geom->size[1] = max(0.0, boxsz[2] - geom->size[0]/2);
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
      for (int i=0; i < nvert(); i++) {
        double v[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
        double dst = mjuu_dist3(v, cen);
        geom->size[0] = max(geom->size[0], dst);
      }
      break;

    case mjGEOM_CAPSULE:
    case mjGEOM_CYLINDER:
      // find maximum distance in XY, separately in Z
      geom->size[0] = 0;
      geom->size[1] = 0;
      for (int i=0; i < nvert(); i++) {
        double v[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
        double dst = sqrt((v[0]-cen[0])*(v[0]-cen[0]) +
                          (v[1]-cen[1])*(v[1]-cen[1]));
        geom->size[0] = max(geom->size[0], dst);

        // proceed with z: valid for cylinder
        double dst2 = abs(v[2]-cen[2]);
        geom->size[1] = max(geom->size[1], dst2);
      }

      // special handling of capsule: consider curved cap
      if (geom->type==mjGEOM_CAPSULE) {
        geom->size[1] = 0;
        for (int i=0; i < nvert(); i++) {
          // get distance in XY and Z
          double v[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
          double dst = sqrt((v[0]-cen[0])*(v[0]-cen[0]) +
                            (v[1]-cen[1])*(v[1]-cen[1]));
          double dst2 = abs(v[2]-cen[2]);

          // get spherical elevation at horizontal distance dst
          double h = geom->size[0] * sin(acos(dst/geom->size[0]));
          geom->size[1] = max(geom->size[1], dst2-h);
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
bool vertcompare(int index1, int index2, const std::vector<float>& vert) {
  for (int i = 0; i < 3; i++) {
    if (vert[3*index1 + i] < vert[3*index2 + i]) {
      return true;
    }
    if (vert[3*index1 + i] > vert[3*index2 + i]) {
      return false;
    }
  }
  return false;
}

// remove repeated vertices
void mjCMesh::RemoveRepeated() {
  int repeated = 0;

  std::vector<int> index(nvert());
  std::vector<int> redirect(nvert());
  for (int i=0; i < nvert(); i++) {
    index[i] = redirect[i] = i;
  }

  std::stable_sort(index.begin(), index.end(), [&vert = vert_](int a, int b) {
    return vertcompare(a, b, vert);
  });

  // find repeated vertices, set redirect
  for (int i=1; i < nvert(); i++) {
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
    for (int i=0; i < nvert(); i++) {
      int j = i;
      while (redirect[j]!=j) {
        j = redirect[j];
      }
      redirect[i] = j;
    }

    // find good vertices, compress, reuse index to save compressed position
    int j = 0;
    for (int i=0; i < nvert(); i++) {
      if (redirect[i]==i) {
        index[i] = j;
        memcpy(vert_.data()+3*j, vert_.data()+3*i, 3*sizeof(float));
        j++;
      } else {
        index[i] = -1;
      }
    }

    // recompute face data to reflect compressed vertices
    for (int i=0; i < 3*nface(); i++) {
      face_[i] = index[redirect[face_[i]]];

      // sanity check, SHOULD NOT OCCUR
      if (face_[i]<0 || face_[i]>=nvert()-repeated) {
        throw mjCError(
            this, "error removing vertices from mesh '%s'", name.c_str());
      }
    }
  }

  // resize vert if any vertices were removed
  if (repeated) {
    std::vector<float> old = vert_;
    vert_.assign(3*(nvert()-repeated), 0);
    memcpy(vert_.data(), old.data(), 3*nvert()*sizeof(float));
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
  vert_ = attrib.vertices;  // copy from one std::vector to another
  normal_ = attrib.normals;
  texcoord_ = attrib.texcoords;
  facenormal_.clear();
  facetexcoord_.clear();

  if (!objReader.GetShapes().empty()) {
    const auto& mesh = objReader.GetShapes()[0].mesh;
    bool righthand = (scale[0]*scale[1]*scale[2] > 0);

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
      face_.push_back(mesh_index.vertex_index);

      if (!normal_.empty()) {
        facenormal_.push_back(mesh_index.normal_index);
      }

      if (!texcoord_.empty()) {
        facetexcoord_.push_back(mesh_index.texcoord_index);
      }
    }
  }

  // flip the second texcoord
  for (int i=0; i < texcoord_.size()/2; i++) {
    texcoord_[2*i+1] = 1-texcoord_[2*i+1];
  }

  // save some partial data for caching
  if (!objReader.GetShapes().empty()) {
    const auto& mesh = objReader.GetShapes()[0].mesh;
    num_face_vertices_ = mesh.num_face_vertices;

    vertex_index_.reserve(mesh.indices.size());
    normal_index_.reserve(mesh.indices.size());
    texcoord_index_.reserve(mesh.indices.size());

    for (tinyobj::index_t index : mesh.indices) {
      vertex_index_.push_back(index.vertex_index);
      normal_index_.push_back(index.normal_index);
      texcoord_index_.push_back(index.texcoord_index);
    }
  }
}



// load OBJ from cached asset, return true on success
bool mjCMesh::LoadCachedMesh(mjCCache *cache, const mjResource* resource) {
  // check that asset has all data
  if (!cache->PopulateData(resource, [&](const void* data) {
    const mjCMesh* mesh = static_cast<const mjCMesh*>(data);
    vert_ = mesh->vert_;
    normal_ = mesh->normal_;
    texcoord_ = mesh->texcoord_;
    vertex_index_ = mesh->vertex_index_;
    normal_index_ = mesh->normal_index_;
    texcoord_index_ = mesh->texcoord_index_;
    num_face_vertices_ = mesh->num_face_vertices_;
  })) {
    return false;
  }

  bool righthand = (scale[0] * scale[1] * scale[2]) > 0;


  for (int face = 0, i = 0; i < vertex_index_.size();) {
    int nfacevert = num_face_vertices_[face];
    if (nfacevert < 3 || nfacevert > 4) {
      throw mjCError(
          this, "only tri or quad meshes are supported for OBJ (file '%s')",
          resource->name);
    }

    face_.push_back(vertex_index_[i]);
    face_.push_back(vertex_index_[i + (righthand == 1 ? 1 : 2)]);
    face_.push_back(vertex_index_[i + (righthand == 1 ? 2 : 1)]);

    if (!normal_.empty()) {
      facenormal_.push_back(normal_index_[i]);
      facenormal_.push_back(normal_index_[i + (righthand == 1 ? 1 : 2)]);
      facenormal_.push_back(normal_index_[i + (righthand == 1 ? 2 : 1)]);
    }

    if (!texcoord_.empty()) {
      facetexcoord_.push_back(texcoord_index_[i]);
      facetexcoord_.push_back(texcoord_index_[i + (righthand == 1 ? 1 : 2)]);
      facetexcoord_.push_back(texcoord_index_[i + (righthand == 1 ? 2 : 1)]);
    }

    if (nfacevert == 4) {
      face_.push_back(vertex_index_[i]);
      face_.push_back(vertex_index_[i + (righthand == 1 ? 2 : 3)]);
      face_.push_back(vertex_index_[i + (righthand == 1 ? 3 : 2)]);

      if (!normal_.empty()) {
        facenormal_.push_back(normal_index_[i]);
        facenormal_.push_back(normal_index_[i + (righthand == 1 ? 1 : 2)]);
        facenormal_.push_back(normal_index_[i + (righthand == 1 ? 2 : 1)]);
      }

      if (!texcoord_.empty()) {
        facetexcoord_.push_back(texcoord_index_[i]);
        facetexcoord_.push_back(texcoord_index_[i + (righthand == 1 ? 1 : 2)]);
        facetexcoord_.push_back(texcoord_index_[i + (righthand == 1 ? 2 : 1)]);
      }
    }
    i += nfacevert;
    ++face;
  }
  return true;
}

// load STL binary mesh
void mjCMesh::LoadSTL(mjResource* resource) {
  bool righthand = (scale[0]*scale[1]*scale[2]>0);

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
  int nfaces = 0;
  ReadFromBuffer(&nfaces, buffer + 80);
  if (nfaces<1 || nfaces>200000) {
    throw mjCError(this,
                   "number of faces should be between 1 and 200000 in STL file '%s';"
                   " perhaps this is an ASCII file?", resource->name);
  }

  // check remaining buffer size
  if (nfaces*50 != buffer_sz-84) {
    throw mjCError(this,
                   "STL file '%s' has wrong size; perhaps this is an ASCII file?",
                   resource->name);
  }

  // assign stl data pointer
  const char* stl = buffer + 84;

  // allocate face and vertex data
  face_.assign(3*nfaces, 0);
  vert_.clear();

  // add vertices and faces, including repeated for now
  for (int i=0; i < nfaces; i++) {
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
        if (fabs(v[k]) > pow(2, 30)) {
          throw mjCError(this,
                        "vertex coordinates in STL file '%s' exceed maximum bounds",
                        resource->name);
        }
      }

      // add vertex address in face; change order if scale makes it lefthanded
      if (righthand || j==0) {
        face_[3*i+j] = nvert();
      } else {
        face_[3*i+3-j] = nvert();
      }

      // add vertex data
      vert_.push_back(v[0]);
      vert_.push_back(v[1]);
      vert_.push_back(v[2]);
    }
  }

  RemoveRepeated();
}



// load MSH binary mesh
void mjCMesh::LoadMSH(mjResource* resource) {
  bool righthand = (scale[0]*scale[1]*scale[2]>0);

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
  int nvbuf = 0, nfbuf = 0, nnbuf = 0, ntbuf = 0;
  ReadFromBuffer(&nvbuf, buffer);
  ReadFromBuffer(&nnbuf, buffer + sizeof(int));
  ReadFromBuffer(&ntbuf, buffer + 2*sizeof(int));
  ReadFromBuffer(&nfbuf, buffer + 3*sizeof(int));

  // check sizes
  if (nvbuf<4 || nfbuf<0 || nnbuf<0 || ntbuf<0 ||
      (nnbuf>0 && nnbuf!=nvbuf) ||
      (ntbuf>0 && ntbuf!=nvbuf)) {
    throw mjCError(this, "invalid sizes in MSH file '%s'", resource->name);
  }

  if (nvbuf >= INT_MAX / sizeof(float) / 3 ||
      nnbuf >= INT_MAX / sizeof(float) / 3 ||
      ntbuf >= INT_MAX / sizeof(float) / 2 ||
      nfbuf >= INT_MAX / sizeof(int) / 3) {
    throw mjCError(this, "too large sizes in MSH file '%s'.", resource->name);
  }
  // check file size
  if (buffer_sz != 4*sizeof(int) + 3*nvbuf*sizeof(float) + 3*nnbuf*sizeof(float) +
      2*ntbuf*sizeof(float) + 3*nfbuf*sizeof(int)) {
    throw mjCError(this, "unexpected file size in MSH file '%s'", resource->name);
  }

  // allocate and copy
  using UnalignedFloat = char[sizeof(float)];
  auto fdata = reinterpret_cast<UnalignedFloat*>(buffer + 4*sizeof(int));
  if (nvbuf) {
    vert_.assign(3*nvbuf, 0);
    memcpy(vert_.data(), fdata, 3*nvert()*sizeof(float));
    fdata += 3*nvert();
  }
  if (nnbuf) {
    normal_.assign(3*nvert(), 0);
    memcpy(normal_.data(), fdata, 3*nvert()*sizeof(float));
    fdata += 3*nvert();
  }
  if (ntbuf) {
    texcoord_.assign(2*nvert(), 0);
    memcpy(texcoord_.data(), fdata, 2*nvert()*sizeof(float));
    fdata += 2*nvert();
  }
  if (nfbuf) {
    face_.assign(3*nfbuf, 0);
    facenormal_.assign(3*nfbuf, 0);
    memcpy(face_.data(), fdata, 3*nfbuf*sizeof(int));
    memcpy(facenormal_.data(), fdata, 3*nfbuf*sizeof(int));
  }
  if  (nfbuf && !texcoord_.empty()) {
    facetexcoord_.assign(3*nfbuf, 0);
    memcpy(facetexcoord_.data(), fdata, 3*nfbuf*sizeof(int));
  }

  // rearrange face data if left-handed scaling
  if (nfbuf && !righthand) {
    for (int i=0; i < nfbuf; i++) {
      int tmp = face_[3*i+1];
      face_[3*i+1] = face_[3*i+2];
      face_[3*i+2] = tmp;
    }
  }
}


void mjCMesh::ComputeVolume(double CoM[3], mjtGeomInertia type,
                              const double facecen[3]) {
  double nrm[3];
  double cen[3];
  GetVolumeRef(type) = 0;
  mjuu_zerovec(CoM, 3);
  int nf = (inertia == mjINERTIA_CONVEX) ? graph_[1] : nface();
  int* f = (inertia == mjINERTIA_CONVEX) ? graph_ + 2 + 3*(graph_[0]+graph_[1]) : face_.data();
  float* vv = vert_.data();
  for (int i=0; i < nf; i++) {
    // get area, normal and center
    double a = _triangle(nrm, cen, vv+3*f[3*i], vv+3*f[3*i+1], vv+3*f[3*i+2]);

    // compute and add volume
    const double vec[3] = {cen[0]-facecen[0], cen[1]-facecen[1], cen[2]-facecen[2]};
    double vol = type==mjINERTIA_SHELL ? a : mjuu_dot3(vec, nrm) * a / 3;

    // if legacy computation requested, then always positive
    if (inertia == mjINERTIA_LEGACY) {
      vol = abs(vol);
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
  if (refpos[0]!=0 || refpos[1]!=0 || refpos[2]!=0) {
    // prepare translation
    float rp[3] = {(float)refpos[0], (float)refpos[1], (float)refpos[2]};

    // process vertices
    for (int i=0; i < nvert(); i++) {
      vert_[3*i] -= rp[0];
      vert_[3*i+1] -= rp[1];
      vert_[3*i+2] -= rp[2];
    }
  }

  // rotate
  if (refquat[0]!=1 || refquat[1]!=0 || refquat[2]!=0 || refquat[3]!=0) {
    // prepare rotation
    double quat[4] = {refquat[0], refquat[1], refquat[2], refquat[3]};
    double mat[9];
    mjuu_normvec(quat, 4);
    mjuu_quat2mat(mat, quat);

    // process vertices
    for (int i=0; i < nvert(); i++) {
      double p1[3], p0[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
      mjuu_mulvecmatT(p1, p0, mat);
      vert_[3*i] = (float) p1[0];
      vert_[3*i+1] = (float) p1[1];
      vert_[3*i+2] = (float) p1[2];
    }

    // process normals
    for (int i=0; i < nnormal(); i++) {
      double n1[3], n0[3] = {normal_[3*i], normal_[3*i+1], normal_[3*i+2]};
      mjuu_mulvecmatT(n1, n0, mat);
      normal_[3*i] = (float) n1[0];
      normal_[3*i+1] = (float) n1[1];
      normal_[3*i+2] = (float) n1[2];
    }
  }

  // scale
  if (scale[0]!=1 || scale[1]!=1 || scale[2]!=1) {
    for (int i=0; i < nvert(); i++) {
      vert_[3*i] *= scale[0];
      vert_[3*i+1] *= scale[1];
      vert_[3*i+2] *= scale[2];
    }

    for (int i=0; i < nnormal(); i++) {
      normal_[3*i] *= scale[0];
      normal_[3*i+1] *= scale[1];
      normal_[3*i+2] *= scale[2];
    }
  }

  // normalize normals
  for (int i=0; i < nnormal(); i++) {
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

  for (int i=0; i < nface(); i++) {
    // check vertex indices
    for (int j=0; j<3; j++) {
      if (face_[3*i+j]<0 || face_[3*i+j]>=nvert()) {
        throw mjCError(this, "vertex index out of range in %s (index = %d)", name.c_str(), i);
      }
    }

    // get area and center
    float* vv = vert_.data();
    double a = _triangle(nrm, cen, vv+3*face_[3*i], vv+3*face_[3*i+1], vv+3*face_[3*i+2]);

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

  double density = model->def_map[classname]->Geom().density;

  // compute inertial properties for both inertia types
  for ( const auto type : { mjtGeomInertia::mjINERTIA_VOLUME, mjtGeomInertia::mjINERTIA_SHELL } ) {
    double CoM[3] = {0, 0, 0};
    double inert[6] = {0, 0, 0, 0, 0, 0};

    // compute CoM and volume from pyramid volumes
    ComputeVolume(CoM, type, facecen);

    // if volume is invalid, skip the rest of the computations
    if (GetVolumeRef(type) < mjMINVAL) {
      if (type == mjINERTIA_SHELL) {
        validarea_ = 0;
      } else {
        validvolume_ = GetVolumeRef(type) < 0 ? -1 : 0;
      }
      continue;
    }

    // finalize CoM, save as mesh center
    for (int j=0; j<3; j++) {
      CoM[j] /= GetVolumeRef(type);
    }
    mjuu_copyvec(GetPosPtr(type), CoM, 3);

    // re-center mesh at CoM
    if (type==mjINERTIA_VOLUME || validvolume_<=0) {
      for (int i=0; i < nvert(); i++) {
        for (int j=0; j<3; j++) {
          vert_[3*i+j] -= CoM[j];
        }
      }
    }

    // accumulate products of inertia, recompute volume
    const int k[6][2] = {{0, 0}, {1, 1}, {2, 2}, {0, 1}, {0, 2}, {1, 2}};
    double P[6] = {0, 0, 0, 0, 0, 0};
    GetVolumeRef(type) = 0;
    int nf = (inertia == mjINERTIA_CONVEX) ? graph_[1] : nface();
    int* f = (inertia == mjINERTIA_CONVEX) ? graph_ + 2 + 3*(graph_[0]+graph_[1]) : face_.data();
    for (int i=0; i < nf; i++) {
      float* D = vert_.data()+3*f[3*i];
      float* E = vert_.data()+3*f[3*i+1];
      float* F = vert_.data()+3*f[3*i+2];

      // get area, normal and center; update volume
      double a = _triangle(nrm, cen, D, E, F);
      double vol = type==mjINERTIA_SHELL ? a : mjuu_dot3(cen, nrm) * a / 3;

      // if legacy computation requested, then always positive
      if (inertia == mjINERTIA_LEGACY) {
        vol = abs(vol);
      }

      // apply formula, accumulate
      GetVolumeRef(type) += vol;
      for (int j=0; j<6; j++) {
        P[j] += density*vol /
                  (type==mjINERTIA_SHELL ? 12 : 20) * (
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
    double eigval[3], eigvec[9], quattmp[4];
    double full[9] = {
      inert[0], inert[3], inert[4],
      inert[3], inert[1], inert[5],
      inert[4], inert[5], inert[2]
    };
    mjuu_eig3(eigval, eigvec, quattmp, full);

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
    double mass = GetVolumeRef(type) * density;
    double* boxsz = GetInertiaBoxPtr(type);
    boxsz[0] = sqrt(6*(eigval[1]+eigval[2]-eigval[0])/mass)/2;
    boxsz[1] = sqrt(6*(eigval[0]+eigval[2]-eigval[1])/mass)/2;
    boxsz[2] = sqrt(6*(eigval[0]+eigval[1]-eigval[2])/mass)/2;

    // if volume was valid, copy volume quat to shell and stop,
    // otherwise use shell quat for coordinate transformations
    if (type==mjINERTIA_SHELL && validvolume_>0) {
      mjuu_copyvec(GetQuatPtr(type), GetQuatPtr(mjINERTIA_VOLUME), 4);
      continue;
    }

    // rotate vertices and normals into axis-aligned frame
    mjuu_copyvec(GetQuatPtr(type), quattmp, 4);
    double neg[4] = {quattmp[0], -quattmp[1], -quattmp[2], -quattmp[3]};
    double mat[9];
    mjuu_quat2mat(mat, neg);
    for (int i=0; i < nvert(); i++) {
      // vertices
      const double vec[3] = {vert_[3*i], vert_[3*i+1], vert_[3*i+2]};
      double res[3];
      mjuu_mulvecmat(res, vec, mat);
      for (int j=0; j<3; j++) {
        vert_[3*i+j] = (float) res[j];

        // axis-aligned bounding box
        aamm_[j+0] = min(aamm_[j+0], res[j]);
        aamm_[j+3] = max(aamm_[j+3], res[j]);
      }
    }
    for (int i=0; i < nnormal(); i++) {
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
void mjCMesh::CheckMesh(mjtGeomInertia type) {
  if (!processed_) {
    return;
  }
  if ((invalidorientation_.first>=0 || invalidorientation_.second>=0) && inertia == mjINERTIA_EXACT)
    throw mjCError(this,
                   "faces of mesh '%s' have inconsistent orientation. Please check the "
                   "faces containing the vertices %d and %d.",
                   name.c_str(), invalidorientation_.first, invalidorientation_.second);
  if (!validarea_ && type==mjINERTIA_SHELL)
    throw mjCError(this, "mesh surface area is too small: %s", name.c_str());
  if (validvolume_<0 && type==mjINERTIA_VOLUME)
    throw mjCError(this, "mesh volume is negative (misoriented triangles): %s", name.c_str());
  if (!validvolume_ && type==mjINERTIA_VOLUME)
    throw mjCError(this, "mesh volume is too small: %s", name.c_str());
  if (!valideigenvalue_)
    throw mjCError(this, "eigenvalue of mesh inertia must be positive: %s", name.c_str());
  if (!validinequality_)
    throw mjCError(this, "eigenvalues of mesh inertia violate A + B >= C: %s", name.c_str());
}


// get inertia pointer
double* mjCMesh::GetInertiaBoxPtr(mjtGeomInertia type) {
  CheckMesh(type);
  return type==mjINERTIA_SHELL ? boxsz_surface_ : boxsz_volume_;
}


double& mjCMesh::GetVolumeRef(mjtGeomInertia type) {
  CheckMesh(type);
  return type==mjINERTIA_SHELL ? surface_ : volume_;
}


// make graph describing convex hull
void mjCMesh::MakeGraph() {
  int adr, ok, curlong, totlong, exitcode;
  double* data;
  facetT* facet, **facetp;
  vertexT* vertex, *vertex1, **vertex1p;

  std::string qhopt = "qhull Qt";
  if (maxhullvert_ > -1) {
    // qhull "TA" actually means "number of vertices added after the initial simplex"
    qhopt += " TA" + std::to_string(maxhullvert_ - 4);
  }

  // graph not needed for small meshes
  if (nvert() < 4) {
    return;
  }

  // convert mesh data to double
  data = (double*) mju_malloc(3*nvert()*sizeof(double));
  if (!data) {
    throw mjCError(this, "could not allocate data for qhull");
  }
  for (int i=0; i < 3*nvert(); i++) {
    if (!std::isfinite(vert_[i])) {
      mju_free(data);
      throw mjCError(this, "vertex coordinate %d is not finite", NULL, i);
    }
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
    qh_initflags(qh, const_cast<char*>(qhopt.c_str()));
    qh_init_B(qh, data, nvert(), 3, False);

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
      if (pid<0 || pid>=nvert()) {
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
          if (pid1<0 || pid1>=nvert()) {
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
    for (int i=0; i < numvert+3*numface; i++) {
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
  if (!face_.empty()) {
    return;
  }

  // get info from graph, allocate
  int numvert = graph_[0];
  face_.assign(3*graph_[1], 0);

  // copy faces
  for (int i=0; i < nface(); i++) {
    // address in graph
    int j = 2 + 3*numvert + 3*nface() + 3*i;

    // copy
    face_[3*i] = graph_[j];
    face_[3*i+1] = graph_[j+1];
    face_[3*i+2] = graph_[j+2];
  }
}



// compute vertex normals
void mjCMesh::MakeNormal(void) {
  // only if normal data is missing
  if (!normal_.empty()) {
    return;
  }

  // allocate and clear normals
  normal_.assign(3*nvert(), 0);

  if (facenormal_.empty()) {
    facenormal_.assign(3*nface(), 0);
  }

  // loop over faces, accumulate vertex normals
  for (int i=0; i < nface(); i++) {
    // get vertex ids
    int vertid[3];
    for (int j=0; j<3; j++) {
      vertid[j] = face_[3*i+j];
    }

    // get triangle edges
    double vec01[3], vec02[3];
    for (int j=0; j<3; j++) {
      vec01[j] = vert_[3*vertid[1]+j] - vert_[3*vertid[0]+j];
      vec02[j] = vert_[3*vertid[2]+j] - vert_[3*vertid[0]+j];
    }

    // compute face normal
    double nrm[3];
    mjuu_crossvec(nrm, vec01, vec02);
    double area = mjuu_normvec(nrm, 3);

    // add normal to each vertex with weight = area
    for (int j=0; j<3; j++) {
      for (int k=0; k<3; k++) {
        normal_[3*vertid[j]+k] += nrm[k]*area;
      }
      facenormal_[3*i+j] = vertid[j];
    }
  }

  // remove large-angle faces
  if (!smoothnormal) {
    // allocate removal and clear
    float* nremove = (float*) mju_malloc(3*nnormal()*sizeof(float));
    memset(nremove, 0, 3*nnormal()*sizeof(float));

    // remove contributions from faces at large angles with vertex normal
    for (int i=0; i < nface(); i++) {
      // get vertex ids
      int vertid[3];
      for (int j=0; j<3; j++) {
        vertid[j] = face_[3*i+j];
      }

      // get triangle edges
      double vec01[3], vec02[3];
      for (int j=0; j<3; j++) {
        vec01[j] = vert_[3*vertid[1]+j] - vert_[3*vertid[0]+j];
        vec02[j] = vert_[3*vertid[2]+j] - vert_[3*vertid[0]+j];
      }

      // compute face normal
      double nrm[3];
      mjuu_crossvec(nrm, vec01, vec02);
      double area = mjuu_normvec(nrm, 3);

      // compare to vertex normal, subtract contribution if dot product too small
      for (int j=0; j<3; j++) {
        // normalized vertex normal
        double vnrm[3] = {normal_[3*vertid[j]], normal_[3*vertid[j]+1], normal_[3*vertid[j]+2]};
        mjuu_normvec(vnrm, 3);

        // dot too small: remove
        if (mjuu_dot3(nrm, vnrm)<0.8) {
          for (int k=0; k<3; k++) {
            nremove[3*vertid[j]+k] += nrm[k]*area;
          }
        }
      }
    }

    // apply removal, free nremove
    for (int i=0; i < 3*nnormal(); i++) {
      normal_[i] -= nremove[i];
    }
    mju_free(nremove);
  }

  // normalize normals
  for (int i=0; i < nnormal(); i++) {
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
  center_ = (double*) mju_malloc(3*nface()*sizeof(double));
  memset(center_, 0, 3*nface()*sizeof(double));

  for (int i=0; i < nface(); i++) {
    // get vertex ids
    int* vertid = face_.data() + 3*i;

    // get triangle edges
    double a[3], b[3];
    for (int j=0; j<3; j++) {
      a[j] = vert_[3*vertid[0]+j] - vert_[3*vertid[2]+j];
      b[j] = vert_[3*vertid[1]+j] - vert_[3*vertid[2]+j];
    }

    // compute face normal
    double nrm[3];
    mjuu_crossvec(nrm, a, b);

    // compute circumradius
    double norm_a_2 = mjuu_dot3(a, a);
    double norm_b_2 = mjuu_dot3(b, b);
    double area = sqrt(mjuu_dot3(nrm, nrm));

    // compute circumcenter
    double res[3], vec[3] = {
      norm_a_2 * b[0] - norm_b_2 * a[0],
      norm_a_2 * b[1] - norm_b_2 * a[1],
      norm_a_2 * b[2] - norm_b_2 * a[2]
    };
    mjuu_crossvec(res, vec, nrm);
    center_[3*i+0] = res[0]/(2*area*area) + vert_[3*vertid[2]+0];
    center_[3*i+1] = res[1]/(2*area*area) + vert_[3*vertid[2]+1];
    center_[3*i+2] = res[2]/(2*area*area) + vert_[3*vertid[2]+2];
  }
}



//------------------ class mjCSkin implementation --------------------------------------------------

// constructor
mjCSkin::mjCSkin(mjCModel* _model) {
  mjs_defaultSkin(&spec);
  elemtype = mjOBJ_SKIN;

  // set model pointer
  model = _model;

  // clear data
  spec_file_.clear();
  spec_material_.clear();
  spec_vert_.clear();
  spec_texcoord_.clear();
  spec_face_.clear();
  spec_bodyname_.clear();
  spec_bindpos_.clear();
  spec_bindquat_.clear();
  spec_vertid_.clear();
  spec_vertweight_.clear();

  bodyid.clear();
  matid = -1;

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



mjCSkin::mjCSkin(const mjCSkin& other) {
  *this = other;
}



mjCSkin& mjCSkin::operator=(const mjCSkin& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCSkin_*>(this) = static_cast<const mjCSkin_&>(other);
    *static_cast<mjsSkin*>(this) = static_cast<const mjsSkin&>(other);
  }
  PointToLocal();
  return *this;
}



void mjCSkin::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.file = &spec_file_;
  spec.material = &spec_material_;
  spec.vert = &spec_vert_;
  spec.texcoord = &spec_texcoord_;
  spec.face = &spec_face_;
  spec.bodyname = &spec_bodyname_;
  spec.bindpos = &spec_bindpos_;
  spec.bindquat = &spec_bindquat_;
  spec.vertid = &spec_vertid_;
  spec.vertweight = &spec_vertweight_;
  spec.info = &info;
  file = nullptr;
  material = nullptr;
  vert = nullptr;
  texcoord = nullptr;
  face = nullptr;
  bodyname = nullptr;
  bindpos = nullptr;
  bindquat = nullptr;
  vertid = nullptr;
  vertweight = nullptr;
}



void mjCSkin::NameSpace(const mjCModel* m) {
  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = mjuu_strippath(spec_file_);
    name = mjuu_stripext(stripped);
  }
  for (auto& name : spec_bodyname_) {
    name = m->prefix + name + m->suffix;
  }
  if (modelfiledir_.empty()) {
    modelfiledir_ = FilePath(m->spec_modelfiledir_);
  }
  if (meshdir_.empty()) {
    meshdir_ = FilePath(m->spec_meshdir_);
  }
}



void mjCSkin::CopyFromSpec() {
  *static_cast<mjsSkin*>(this) = spec;
  file_ = spec_file_;
  material_ = spec_material_;
  vert_ = spec_vert_;
  texcoord_ = spec_texcoord_;
  face_ = spec_face_;
  bodyname_ = spec_bodyname_;
  bindpos_ = spec_bindpos_;
  bindquat_ = spec_bindquat_;
  vertid_ = spec_vertid_;
  vertweight_ = spec_vertweight_;

  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = mjuu_strippath(file_);
    name = mjuu_stripext(stripped);
  }
}



// destructor
mjCSkin::~mjCSkin() {
  spec_file_.clear();
  spec_material_.clear();
  spec_vert_.clear();
  spec_texcoord_.clear();
  spec_face_.clear();
  spec_bodyname_.clear();
  spec_bindpos_.clear();
  spec_bindquat_.clear();
  spec_vertid_.clear();
  spec_vertweight_.clear();
  bodyid.clear();
}



void mjCSkin::ResolveReferences(const mjCModel* m) {
  size_t nbone = bodyname_.size();
  bodyid.resize(nbone);
  for (int i=0; i < nbone; i++) {
    mjCBase* pbody = m->FindObject(mjOBJ_BODY, bodyname_[i]);
    if (!pbody) {
      throw mjCError(this, "unknown body '%s' in skin", bodyname_[i].c_str());
    }
    bodyid[i] = pbody->id;
  }
}



// compiler
void mjCSkin::Compile(const mjVFS* vfs) {
  CopyFromSpec();

  // load file
  if (!file_.empty()) {
    // make sure data is not present
    if (!vert_.empty() ||
        !texcoord_.empty() ||
        !face_.empty() ||
        !bodyname_.empty() ||
        !bindpos_.empty() ||
        !bindquat_.empty() ||
        !vertid_.empty() ||
        !vertweight_.empty() ||
        !bodyid.empty()) {
      throw mjCError(this, "Data already exists, trying to load from skin file: %s", file_.c_str());
    }

    // remove path from file if necessary
    if (model->strippath) {
      file_ = mjuu_strippath(file_);
    }

    // load SKN
    std::string ext = mjuu_getext(file_);
    if (strcasecmp(ext.c_str(), ".skn")) {
      throw mjCError(this, "Unknown skin file type: %s", file_.c_str());
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
      LoadSKN(resource);
      mju_closeResource(resource);
    } catch(mjCError err) {
      mju_closeResource(resource);
      throw err;
    }
  }

  // make sure all data is present
  if (vert_.empty() ||
      face_.empty() ||
      bodyname_.empty() ||
      bindpos_.empty() ||
      bindquat_.empty() ||
      vertid_.empty() ||
      vertweight_.empty()) {
    throw mjCError(this, "Missing data in skin");
  }

  // check mesh sizes
  if (vert_.size()%3) {
    throw mjCError(this, "Vertex data must be multiple of 3");
  }
  if (!texcoord_.empty() && texcoord_.size()!=2*vert_.size()/3) {
    throw mjCError(this, "Vertex and texcoord data incompatible size");
  }
  if (face_.size()%3) {
    throw mjCError(this, "Face data must be multiple of 3");
  }

  // check bone sizes
  size_t nbone = bodyname_.size();
  if (bindpos_.size()!=3*nbone) {
    throw mjCError(this, "Unexpected bindpos size in skin");
  }
  if (bindquat_.size()!=4*nbone) {
    throw mjCError(this, "Unexpected bindquat size in skin");
  }
  if (vertid_.size()!=nbone) {
    throw mjCError(this, "Unexpected vertid size in skin");
  }
  if (vertweight_.size()!=nbone) {
    throw mjCError(this, "Unexpected vertweight size in skin");
  }

  // resolve body names
  ResolveReferences(model);

  // resolve material name
  mjCBase* pmat = model->FindObject(mjOBJ_MATERIAL, material_);
  if (pmat) {
    matid = pmat->id;
  } else if (!material_.empty()) {
    throw mjCError(this, "unknown material '%s' in skin", material_.c_str());
  }

  // set total vertex weights to 0
  std::vector<float> vw;
  size_t nvert = vert_.size()/3;
  vw.resize(nvert);
  fill(vw.begin(), vw.end(), 0.0f);

  // accumulate vertex weights from all bones
  for (int i=0; i < nbone; i++) {
    // make sure bone has vertices and sizes match
    size_t nbv = vertid_[i].size();
    if (vertweight_[i].size()!=nbv || nbv==0) {
      throw mjCError(this, "vertid and vertweight must have same non-zero size in skin");
    }

    // accumulate weights in global array
    for (int j=0; j<nbv; j++) {
      // get index and check range
      int jj = vertid_[i][j];
      if (jj<0 || jj>=nvert) {
        throw mjCError(this, "vertid %d out of range in skin", NULL, jj);
      }

      // accumulate
      vw[jj] += vertweight_[i][j];
    }
  }

  // check coverage
  for (int i=0; i < nvert; i++) {
    if (vw[i]<=mjMINVAL) {
      throw mjCError(this, "vertex %d must have positive total weight in skin", NULL, i);
    }
  }

  // normalize vertex weights
  for (int i=0; i < nbone; i++) {
    for (int j=0; j<vertid_[i].size(); j++) {
      vertweight_[i][j] /= vw[vertid_[i][j]];
    }
  }

  // normalize bindquat
  for (int i=0; i < nbone; i++) {
    double quat[4] = {
      (double)bindquat_[4*i],
      (double)bindquat_[4*i+1],
      (double)bindquat_[4*i+2],
      (double)bindquat_[4*i+3]
    };
    mjuu_normvec(quat, 4);

    bindquat_[4*i]   = (float) quat[0];
    bindquat_[4*i+1] = (float) quat[1];
    bindquat_[4*i+2] = (float) quat[2];
    bindquat_[4*i+3] = (float) quat[3];
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
    vert_.resize(3*nvert);
    memcpy(vert_.data(), pdata+cnt, 3*nvert*sizeof(float));
    cnt += 3*nvert;
  }

  // copy texcoord
  if (ntexcoord) {
    texcoord_.resize(2*ntexcoord);
    memcpy(texcoord_.data(), pdata+cnt, 2*ntexcoord*sizeof(float));
    cnt += 2*ntexcoord;
  }

  // copy face
  if (nface) {
    face_.resize(3*nface);
    memcpy(face_.data(), pdata+cnt, 3*nface*sizeof(int));
    cnt += 3*nface;
  }

  // allocate bone arrays
  bodyname_.clear();
  bindpos_.resize(3*nbone);
  bindquat_.resize(4*nbone);
  vertid_.resize(nbone);
  vertweight_.resize(nbone);

  // read bones
  for (int i=0; i < nbone; i++) {
    // check size
    if (buffer_sz/4-4-cnt < 18) {
      throw mjCError(this, "insufficient data in SKN file '%s', bone %d", resource->name, i);
    }

    // read name
    char txt[40];
    strncpy(txt, (char*)(pdata+cnt), 39);
    txt[39] = '\0';
    cnt += 10;
    bodyname_.push_back(txt);

    // read bindpos
    memcpy(bindpos_.data()+3*i, pdata+cnt, 3*sizeof(float));
    cnt += 3;

    // read bind quat
    memcpy(bindquat_.data()+4*i, pdata+cnt, 4*sizeof(float));
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
    vertid_[i].resize(vcount);
    memcpy(vertid_[i].data(), (int*)(pdata+cnt), vcount*sizeof(int));
    cnt += vcount;

    // read vertweight
    vertweight_[i].resize(vcount);
    memcpy(vertweight_[i].data(), (int*)(pdata+cnt), vcount*sizeof(int));
    cnt += vcount;
  }

  // check final size
  if (buffer_sz != 16+4*cnt) {
    throw mjCError(this, "unexpected buffer size in SKN file '%s'", resource->name);
  }
}



//--------------------- elasticity implementation --------------------------------------------------

// hash function for std::pair
struct PairHash
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

// simplex connectivity
constexpr int eledge[3][6][2] = {{{ 0,  1}, {-1, -1}, {-1, -1},
                                  {-1, -1}, {-1, -1}, {-1, -1}},
                                 {{ 1,  2}, { 2,  0}, { 0,  1},
                                  {-1, -1}, {-1, -1}, {-1, -1}},
                                 {{ 0,  1}, { 1,  2}, { 2,  0},
                                  { 2,  3}, { 0,  3}, { 1,  3}}};

struct Stencil2D {
  static constexpr int kNumEdges = 3;
  static constexpr int kNumVerts = 3;
  static constexpr int kNumFaces = 2;
  static constexpr int edge[kNumEdges][2] = {{1, 2}, {2, 0}, {0, 1}};
  static constexpr int face[kNumVerts][2] = {{1, 2}, {2, 0}, {0, 1}};
  static constexpr int edge2face[kNumEdges][2] = {{1, 2}, {2, 0}, {0, 1}};
  int vertices[kNumVerts];
  int edges[kNumEdges];
};

struct Stencil3D {
  static constexpr int kNumEdges = 6;
  static constexpr int kNumVerts = 4;
  static constexpr int kNumFaces = 3;
  static constexpr int edge[kNumEdges][2] = {{0, 1}, {1, 2}, {2, 0},
                                             {2, 3}, {0, 3}, {1, 3}};
  static constexpr int face[kNumVerts][3] = {{2, 1, 0}, {0, 1, 3},
                                             {1, 2, 3}, {2, 0, 3}};
  static constexpr int edge2face[kNumEdges][2] = {{2, 3}, {1, 3}, {2, 1},
                                                  {1, 0}, {0, 2}, {0, 3}};
  int vertices[kNumVerts];
  int edges[kNumEdges];
};

template <typename T>
inline double ComputeVolume(const double* x, const int v[T::kNumVerts]);

template <>
inline double ComputeVolume<Stencil2D>(const double* x,
                                       const int v[Stencil2D::kNumVerts]) {
  double normal[3];
  const double* x0 = x + 3*v[0];
  const double* x1 = x + 3*v[1];
  const double* x2 = x + 3*v[2];
  double edge1[3] = {x1[0]-x0[0], x1[1]-x0[1], x1[2]-x0[2]};
  double edge2[3] = {x2[0]-x0[0], x2[1]-x0[1], x2[2]-x0[2]};
  mjuu_crossvec(normal, edge1, edge2);
  return mjuu_normvec(normal, 3) / 2;
}

template<>
inline double ComputeVolume<Stencil3D>(const double* x,
                                       const int v[Stencil3D::kNumVerts]) {
  double normal[3];
  const double* x0 = x + 3*v[0];
  const double* x1 = x + 3*v[1];
  const double* x2 = x + 3*v[2];
  const double* x3 = x + 3*v[3];
  double edge1[3] = {x1[0]-x0[0], x1[1]-x0[1], x1[2]-x0[2]};
  double edge2[3] = {x2[0]-x0[0], x2[1]-x0[1], x2[2]-x0[2]};
  double edge3[3] = {x3[0]-x0[0], x3[1]-x0[1], x3[2]-x0[2]};
  mjuu_crossvec(normal, edge1, edge2);
  return mjuu_dot3(normal, edge3) / 6;
}

// compute metric tensor of edge lengths inner product
template <typename T>
void inline MetricTensor(double* metric, int idx, double mu,
                         double la, const double basis[T::kNumEdges][9]) {
  double trE[T::kNumEdges] = {0};
  double trEE[T::kNumEdges*T::kNumEdges] = {0};
  double k[T::kNumEdges*T::kNumEdges];

  // compute first invariant i.e. trace(strain)
  for (int e = 0; e < T::kNumEdges; e++) {
    for (int i = 0; i < 3; i++) {
      trE[e] += basis[e][4*i];
    }
  }

  // compute second invariant i.e. trace(strain^2)
  for (int ed1 = 0; ed1 < T::kNumEdges; ed1++) {
    for (int ed2 = 0; ed2 < T::kNumEdges; ed2++) {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          trEE[T::kNumEdges*ed1+ed2] += basis[ed1][3*i+j] * basis[ed2][3*j+i];
        }
      }
    }
  }

  // assembly of strain metric tensor
  for (int ed1 = 0; ed1 < T::kNumEdges; ed1++) {
    for (int ed2 = 0; ed2 < T::kNumEdges; ed2++) {
      k[T::kNumEdges*ed1 + ed2] = mu * trEE[T::kNumEdges * ed1 + ed2] +
                                  la * trE[ed2] * trE[ed1];
    }
  }

  // copy to triangular representation
  int id = 0;
  for (int ed1 = 0; ed1 < T::kNumEdges; ed1++) {
    for (int ed2 = ed1; ed2 < T::kNumEdges; ed2++) {
      metric[21*idx + id++] = k[T::kNumEdges*ed1 + ed2];
    }
  }

  if (id != T::kNumEdges*(T::kNumEdges+1)/2) {
    mju_error("incorrect stiffness matrix size");
  }
}

// compute local basis
template <typename T>
void inline ComputeBasis(double basis[9], const double* x,
                         const int v[T::kNumVerts],
                         const int faceL[T::kNumFaces],
                         const int faceR[T::kNumFaces], double volume);

template <>
void inline ComputeBasis<Stencil2D>(double basis[9], const double* x,
                                    const int v[Stencil2D::kNumVerts],
                                    const int faceL[Stencil2D::kNumFaces],
                                    const int faceR[Stencil2D::kNumFaces],
                                    double volume) {
  double basisL[3], basisR[3];
  double normal[3];

  const double* xL0 = x + 3*v[faceL[0]];
  const double* xL1 = x + 3*v[faceL[1]];
  const double* xR0 = x + 3*v[faceR[0]];
  const double* xR1 = x + 3*v[faceR[1]];
  double edgesL[3] = {xL0[0]-xL1[0], xL0[1]-xL1[1], xL0[2]-xL1[2]};
  double edgesR[3] = {xR1[0]-xR0[0], xR1[1]-xR0[1], xR1[2]-xR0[2]};

  mjuu_crossvec(normal, edgesR, edgesL);
  mjuu_normvec(normal, 3);
  mjuu_crossvec(basisL, normal, edgesL);
  mjuu_crossvec(basisR, edgesR, normal);

  // we use as basis the symmetrized tensor products of the edge normals of the
  // other two edges; this is shown in Weischedel "A discrete geometric view on
  // shear-deformable shell models" in the remark at the end of section 4.1;
  // equivalent to linear finite elements but in a coordinate-free formulation.

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      basis[3*i+j] = ( basisL[i]*basisR[j] +
                       basisR[i]*basisL[j] ) / (8*volume*volume);
    }
  }
}

// compute local basis
template <>
void inline ComputeBasis<Stencil3D>(double basis[9], const double* x,
                                    const int v[Stencil3D::kNumVerts],
                                    const int faceL[Stencil3D::kNumFaces],
                                    const int faceR[Stencil3D::kNumFaces],
                                    double volume) {
  const double* xL0 = x + 3*v[faceL[0]];
  const double* xL1 = x + 3*v[faceL[1]];
  const double* xL2 = x + 3*v[faceL[2]];
  const double* xR0 = x + 3*v[faceR[0]];
  const double* xR1 = x + 3*v[faceR[1]];
  const double* xR2 = x + 3*v[faceR[2]];
  double edgesL[6] = {xL1[0] - xL0[0], xL1[1] - xL0[1], xL1[2] - xL0[2],
                      xL2[0] - xL0[0], xL2[1] - xL0[1], xL2[2] - xL0[2]};
  double edgesR[6] = {xR1[0] - xR0[0], xR1[1] - xR0[1], xR1[2] - xR0[2],
                      xR2[0] - xR0[0], xR2[1] - xR0[1], xR2[2] - xR0[2]};

  double normalL[3], normalR[3];
  mjuu_crossvec(normalL, edgesL, edgesL+3);
  mjuu_crossvec(normalR, edgesR, edgesR+3);

  // we use as basis the symmetrized tensor products of the area normals of the
  // two faces not adjacent to the edge; this is the 3D equivalent to the basis
  // proposed in Weischedel "A discrete geometric view on shear-deformable shell
  // models" in the remark at the end of section 4.1. This is also equivalent to
  // linear finite elements but in a coordinate-free formulation.

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      basis[3*i+j] = ( normalL[i]*normalR[j] +
                       normalR[i]*normalL[j] ) / (36*2*volume*volume);
    }
  }
}

// compute stiffness for a single element
template <typename T>
void inline ComputeStiffness(std::vector<double>& stiffness,
                             const std::vector<double>& body_pos,
                             const int* v, int t, double E,
                             double nu, double thickness = 4) {
  // triangles area
  double volume = ComputeVolume<T>(body_pos.data(), v);

  // material parameters
  double mu = E / (2*(1+nu)) * std::abs(volume) / 4 * thickness;
  double la = E*nu / ((1+nu)*(1-2*nu)) * std::abs(volume) / 4 * thickness;

  // local geometric quantities
  double basis[T::kNumEdges][9] = {{0}};

  // compute edge basis
  for (int e = 0; e < T::kNumEdges; e++) {
    ComputeBasis<T>(basis[e], body_pos.data(), v,
                    T::face[T::edge2face[e][0]],
                    T::face[T::edge2face[e][1]], volume);
  }

  // compute metric tensor
  MetricTensor<T>(stiffness.data(), t, mu, la, basis);
}

//------------------ class mjCFlex implementation --------------------------------------------------

// constructor
mjCFlex::mjCFlex(mjCModel* _model) {
  mjs_defaultFlex(&spec);
  elemtype = mjOBJ_FLEX;

  // set model
  model = _model;

  // clear internal variables
  nvert = 0;
  nedge = 0;
  nelem = 0;
  matid = -1;
  rigid = false;
  centered = false;

  PointToLocal();
  CopyFromSpec();
}


mjCFlex::mjCFlex(const mjCFlex& other) {
  *this = other;
}


mjCFlex& mjCFlex::operator=(const mjCFlex& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCFlex_*>(this) = static_cast<const mjCFlex_&>(other);
    *static_cast<mjsFlex*>(this) = static_cast<const mjsFlex&>(other);
  }
  PointToLocal();
  return *this;
}


void mjCFlex::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.name = &name;
  spec.material = &spec_material_;
  spec.vertbody = &spec_vertbody_;
  spec.vert = &spec_vert_;
  spec.texcoord = &spec_texcoord_;
  spec.elem = &spec_elem_;
  spec.info = &info;
  material = nullptr;
  vertbody = nullptr;
  vert = nullptr;
  texcoord = nullptr;
  elem = nullptr;
}



void mjCFlex::NameSpace(const mjCModel* m) {
  for (auto& name : spec_vertbody_) {
    name = m->prefix + name + m->suffix;
  }
}



void mjCFlex::CopyFromSpec() {
  *static_cast<mjsFlex*>(this) = spec;
  spec.info = &info;
  material_ = spec_material_;
  vertbody_ = spec_vertbody_;
  vert_ = spec_vert_;
  texcoord_ = spec_texcoord_;
  elem_ = spec_elem_;

  // clear precompiled asset. TODO: use asset cache
  nedge = 0;
  edge.clear();
  shell.clear();
  evpair.clear();
}


bool mjCFlex::HasTexcoord() const {
  return !texcoord_.empty();
}


void mjCFlex::DelTexcoord() {
  texcoord_.clear();
}


void mjCFlex::ResolveReferences(const mjCModel* m) {
  for (const auto& vertbody : vertbody_) {
    mjCBase* pbody = m->FindObject(mjOBJ_BODY, vertbody);
    if (pbody) {
      vertbodyid.push_back(pbody->id);
    } else {
      throw mjCError(this, "unknown body '%s' in flex", vertbody.c_str());
    }
  }
}


// compiler
void mjCFlex::Compile(const mjVFS* vfs) {
  CopyFromSpec();

  // set nelem; check sizes
  if (dim<1 || dim>3) {
      throw mjCError(this, "dim must be 1, 2 or 3");
  }
  if (elem_.empty()) {
      throw mjCError(this, "elem is empty");
  }
  if (elem_.size() % (dim+1)) {
      throw mjCError(this, "elem size must be multiple of (dim+1)");
  }
  if (vertbody_.empty()) {
      throw mjCError(this, "vertbody is empty");
  }
  if (vert_.size() % 3) {
      throw mjCError(this, "vert size must be a multiple of 3");
  }
  if (edgestiffness>0 && dim>1) {
    throw mjCError(this, "edge stiffness only available for dim=1, please use elasticity plugins");
  }
  nelem = (int)elem_.size()/(dim+1);

  // set nvert, rigid, centered; check size
  if (vert_.empty()) {
    centered = true;
    nvert = (int)vertbody_.size();
  }
  else {
    nvert = (int)vert_.size()/3;
    if (vertbody_.size()==1) {
      rigid = true;
    }
  }
  if (nvert<dim+1) {
    throw mjCError(this, "not enough vertices");
  }

  // check elem vertex ids
  for (const auto& elem : elem_) {
    if (elem<0 || elem>=nvert) {
      throw mjCError(this, "elem vertex id out of range");
    }
  }

  // check texcoord
  if (!texcoord_.empty() && texcoord_.size()!=2*nvert) {
    throw mjCError(this, "two texture coordinates per vertex expected");
  }

  // resolve material name
  mjCBase* pmat = model->FindObject(mjOBJ_MATERIAL, material_);
  if (pmat) {
    matid = pmat->id;
  } else if (!material_.empty()) {
    throw mjCError(this, "unknown material '%s' in flex", material_.c_str());
  }

  // resolve body ids
  ResolveReferences(model);

  // process elements
  for (int e=0; e<(int)elem_.size()/(dim+1); e++) {
    // make sorted copy of element
    std::vector<int> el;
    el.assign(elem_.begin()+e*(dim+1), elem_.begin()+(e+1)*(dim+1));
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
    for (unsigned i=1; i < vertbodyid.size(); i++) {
      if (vertbodyid[i]!=vertbodyid[0]) {
        rigid = false;
        break;
      }
    }
  }

  // determine centered if not already set
  if (!centered) {
    centered = true;
    for (const auto& vert : vert_) {
      if (vert!=0) {
        centered = false;
        break;
      }
    }
  }

  // compute global vertex positions
  vertxpos = std::vector<double> (3*nvert);
  for (int i=0; i < nvert; i++) {
    // get body id, set vertxpos = body.xpos0
    int b = rigid ? vertbodyid[0] : vertbodyid[i];
    mjuu_copyvec(vertxpos.data()+3*i, model->Bodies()[b]->xpos0, 3);

    // add vertex offset within body if not centered
    if (!centered) {
      double offset[3];
      mjuu_rotVecQuat(offset, vert_.data()+3*i, model->Bodies()[b]->xquat0);
      mjuu_addtovec(vertxpos.data()+3*i, offset, 3);
    }
  }

  // reorder tetrahedra so right-handed face orientation is outside
  // faces are (0,1,2); (0,2,3); (0,3,1); (1,3,2)
  if (dim==3) {
    for (int e=0; e<nelem; e++) {
      const int* edata = elem_.data() + e*(dim+1);
      double* v0 = vertxpos.data() + 3*edata[0];
      double* v1 = vertxpos.data() + 3*edata[1];
      double* v2 = vertxpos.data() + 3*edata[2];
      double* v3 = vertxpos.data() + 3*edata[3];
      double v01[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
      double v02[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
      double v03[3] = {v3[0]-v0[0], v3[1]-v0[1], v3[2]-v0[2]};

      // detect wrong orientation
      double nrm[3];
      mjuu_crossvec(nrm, v01, v02);
      if (mjuu_dot3(nrm, v03)>0) {
        // flip orientation
        int tmp = elem_[e*(dim+1)+1];
        elem_[e*(dim+1)+1] = elem_[e*(dim+1)+2];
        elem_[e*(dim+1)+2] = tmp;
      }
    }
  }

  // create edges
  edgeidx_.assign(elem_.size()*kNumEdges[dim-1]/(dim+1), 0);

  // map from edge vertices to their index in `edges` vector
  std::unordered_map<std::pair<int, int>, int, PairHash> edge_indices;

  // insert local edges into global vector
  for (unsigned f = 0; f < elem_.size()/(dim+1); f++) {
    int* v = elem_.data() + f*(dim+1);
    for (int e = 0; e < kNumEdges[dim-1]; e++) {
      auto pair = std::pair(
        min(v[eledge[dim-1][e][0]], v[eledge[dim-1][e][1]]),
        max(v[eledge[dim-1][e][0]], v[eledge[dim-1][e][1]])
      );

      // if edge is already present in the vector only store its index
      auto [it, inserted] = edge_indices.insert({pair, nedge});

      if (inserted) {
        edge.push_back(pair);
        edgeidx_[f*kNumEdges[dim-1]+e] = nedge++;
      } else {
        edgeidx_[f*kNumEdges[dim-1]+e] = it->second;
      }
    }
  }

  // set size
  nedge = (int)edge.size();

  // compute elasticity
  if (young > 0) {
    if (poisson < 0 || poisson >= 0.5) {
      throw mjCError(this, "Poisson ratio must be in [0, 0.5)");
    }
    stiffness.assign(21*nelem, 0);
    for (unsigned int t = 0; t < nelem; t++) {
      if (dim==2) {
        ComputeStiffness<Stencil2D>(stiffness, vertxpos,
                                    elem_.data() + (dim + 1) * t, t, young,
                                    poisson, thickness);
      } else if (dim==3) {
        ComputeStiffness<Stencil3D>(stiffness, vertxpos,
                                    elem_.data() + (dim + 1) * t, t, young,
                                    poisson);
      }
    }
  }

  // add plugins
  std::string userface, useredge;
  userface = VectorToString(elem_);
  useredge = VectorToString(edgeidx_);

  for (const auto& vbodyid : vertbodyid) {
    if (model->Bodies()[vbodyid]->plugin.element) {
      mjCPlugin* plugin_instance =
          static_cast<mjCPlugin*>(model->Bodies()[vbodyid]->plugin.element);
      if (damping > 0) {
        plugin_instance->config_attribs["damping"] = std::to_string(damping);
      }
      plugin_instance->config_attribs["face"] = userface;
      plugin_instance->config_attribs["edge"] = useredge;
    }
  }

  // create shell fragments and element-vertex collision pairs
  CreateShellPair();

  // create bounding volume hierarchy
  CreateBVH();

  // compute bounding box coordinates
  vert0_.assign(3*nvert, 0);
  const mjtNum* bvh = tree.Bvh().data();
  for (int j=0; j < nvert; j++) {
    for (int k=0; k < 3; k++) {
      double size = 2*(bvh[k+3] - radius);
      vert0_[3*j+k] = (vertxpos[3*j+k] - bvh[k]) / size + 0.5;
    }
  }
}



// create flex BVH
void mjCFlex::CreateBVH(void) {
  int nbvh = 0;

  // allocate element bounding boxes
  elemaabb_.resize(6*nelem);
  tree.AllocateBoundingVolumes(nelem);

  // construct element bounding boxes, add to hierarchy
  for (int e=0; e<nelem; e++) {
    const int* edata = elem_.data() + e*(dim+1);

    // skip inactive in 3D
    if (dim==3 && elemlayer[e]>=activelayers) {
      continue;
    }

    // compute min and max along each global axis
    double xmin[3], xmax[3];
    mjuu_copyvec(xmin, vertxpos.data() + 3*edata[0], 3);
    mjuu_copyvec(xmax, vertxpos.data() + 3*edata[0], 3);
    for (int i=1; i <= dim; i++) {
      for (int j=0; j<3; j++) {
        xmin[j] = mjMIN(xmin[j], vertxpos[3*edata[i]+j]);
        xmax[j] = mjMAX(xmax[j], vertxpos[3*edata[i]+j]);
      }
    }

    // compute aabb (center, size)
    elemaabb_[6*e+0] = 0.5*(xmax[0]+xmin[0]);
    elemaabb_[6*e+1] = 0.5*(xmax[1]+xmin[1]);
    elemaabb_[6*e+2] = 0.5*(xmax[2]+xmin[2]);
    elemaabb_[6*e+3] = 0.5*(xmax[0]-xmin[0]) + radius;
    elemaabb_[6*e+4] = 0.5*(xmax[1]-xmin[1]) + radius;
    elemaabb_[6*e+5] = 0.5*(xmax[2]-xmin[2]) + radius;

    // add bounding volume for this element
    mjCBoundingVolume* bv = tree.GetBoundingVolume(nbvh++);
    bv->contype = contype;
    bv->conaffinity = conaffinity;
    bv->quat = NULL;
    bv->SetId(e);
    bv->aabb = elemaabb_.data() + 6*e;
    bv->pos = bv->aabb;
  }

  // create hierarchy
  tree.RemoveInactiveVolumes(nbvh);
  tree.CreateBVH();
}



// create shells and element-vertex collision pairs
void mjCFlex::CreateShellPair(void) {
  std::vector<std::vector<int>> fragspec(nelem*(dim+1));   // [sorted frag vertices, elem, original frag vertices]
  std::vector<std::vector<int>> connectspec;               // [elem1, elem2, common sorted frag vertices]
  std::vector<bool> border(nelem, false);              // is element on the border
  std::vector<bool> borderfrag(nelem*(dim+1), false);  // is fragment on the border

  // make fragspec
  for (int e=0; e<nelem; e++) {
    int n = e*(dim+1);

    // element vertices in original (unsorted) order
    std::vector<int> el;
    el.assign(elem_.begin()+n, elem_.begin()+n+dim+1);

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
    std::vector<int> previous = {fragspec[n-1].begin(), fragspec[n-1].begin()+dim};
    std::vector<int> current = {fragspec[n].begin(), fragspec[n].begin()+dim};

    // same sequential fragments
    if (previous==current) {
      // found pair of elements connected by common fragment
      std::vector<int> connect;
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
  for (unsigned i=0; i < borderfrag.size(); i++) {
    if (borderfrag[i]) {
      // add fragment vertices, in original order
      shell.insert(shell.end(), fragspec[i].begin()+dim+1, fragspec[i].end());
    }
  }

  // compute elemlayer (distance from border) via value iteration in 3D
  if (dim<3) {
    elemlayer = std::vector<int> (nelem, 0);
  }
  else {
    elemlayer = std::vector<int> (nelem, nelem+1);   // init with greater than max value
    for (int e=0; e<nelem; e++) {
      if (border[e]) {
        elemlayer[e] = 0;                       // set border elements to 0
      }
    }

    bool change = true;
    while (change) {                            // repeat while changes are happening
      change = false;

      // process edges of element connectivity graph
      for (const auto& connect : connectspec) {
        int e1 = connect[0];             // get element pair for this edge
        int e2 = connect[1];
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
    for (const auto& connect : connectspec) {
      if (border[connect[0]] || border[connect[1]]) {
        // extract common fragment
        std::vector<int> frag = {connect.begin()+2, connect.end()};

        // process both elements
        for (int ei=0; ei < 2; ei++) {
          const int* edata = elem_.data() + connect[ei]*(dim+1);

          // find element vertex that is not in the common fragment
          for (int i=0; i <= dim; i++) {
            if (frag.end() == std::find(frag.begin(), frag.end(), edata[i])) {
              // add ev pair, involving the other element in connectspec
              evpair.push_back(connect[1-ei]);
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
