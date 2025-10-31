// Copyright 2025 DeepMind Technologies Limited
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

// NOLINTBEGIN(whitespace/line_length)
#ifndef MUJOCO_WASM_CODEGEN_GENERATED_BINDINGS_H_
#define MUJOCO_WASM_CODEGEN_GENERATED_BINDINGS_H_
#include <emscripten.h>
#include <emscripten/bind.h>

#include <cstddef>
#include <cstdint>
#include <memory>

#include <mujoco/mujoco.h>

namespace mujoco::wasm {

// Create the types for anonymous structs
using mjVisualGlobal = decltype(::mjVisual::global);
using mjVisualQuality = decltype(::mjVisual::quality);
using mjVisualHeadlight = decltype(::mjVisual::headlight);
using mjVisualMap = decltype(::mjVisual::map);
using mjVisualScale = decltype(::mjVisual::scale);
using mjVisualRgba = decltype(::mjVisual::rgba);

// {{ AUTOGENNED_STRUCT_DEFINITIONS }}
struct MjVisualGlobal {
  MjVisualGlobal();
  explicit MjVisualGlobal(mjVisualGlobal *ptr);
  MjVisualGlobal(const MjVisualGlobal &);
  MjVisualGlobal &operator=(const MjVisualGlobal &);
  ~MjVisualGlobal();
  std::unique_ptr<MjVisualGlobal> copy();
  // INSERT-GENERATED-MjVisualGlobal-DEFINITIONS
  mjVisualGlobal* get() const { return ptr_; }
  void set(mjVisualGlobal* ptr) { ptr_ = ptr; }

 private:
  mjVisualGlobal* ptr_;
  bool owned_ = false;
};

struct MjVisualQuality {
  MjVisualQuality();
  explicit MjVisualQuality(mjVisualQuality *ptr);
  MjVisualQuality(const MjVisualQuality &);
  MjVisualQuality &operator=(const MjVisualQuality &);
  ~MjVisualQuality();
  std::unique_ptr<MjVisualQuality> copy();
  // INSERT-GENERATED-MjVisualQuality-DEFINITIONS
  mjVisualQuality* get() const { return ptr_; }
  void set(mjVisualQuality* ptr) { ptr_ = ptr; }

 private:
  mjVisualQuality* ptr_;
  bool owned_ = false;
};

struct MjVisualHeadlight {
  MjVisualHeadlight();
  explicit MjVisualHeadlight(mjVisualHeadlight *ptr);
  MjVisualHeadlight(const MjVisualHeadlight &);
  MjVisualHeadlight &operator=(const MjVisualHeadlight &);
  ~MjVisualHeadlight();
  std::unique_ptr<MjVisualHeadlight> copy();
  // INSERT-GENERATED-MjVisualHeadlight-DEFINITIONS
  mjVisualHeadlight* get() const { return ptr_; }
  void set(mjVisualHeadlight* ptr) { ptr_ = ptr; }

 private:
  mjVisualHeadlight* ptr_;
  bool owned_ = false;
};

struct MjVisualMap {
  MjVisualMap();
  explicit MjVisualMap(mjVisualMap *ptr);
  MjVisualMap(const MjVisualMap &);
  MjVisualMap &operator=(const MjVisualMap &);
  ~MjVisualMap();
  std::unique_ptr<MjVisualMap> copy();
  // INSERT-GENERATED-MjVisualMap-DEFINITIONS
  mjVisualMap* get() const { return ptr_; }
  void set(mjVisualMap* ptr) { ptr_ = ptr; }

 private:
  mjVisualMap* ptr_;
  bool owned_ = false;
};

struct MjVisualScale {
  MjVisualScale();
  explicit MjVisualScale(mjVisualScale *ptr);
  MjVisualScale(const MjVisualScale &);
  MjVisualScale &operator=(const MjVisualScale &);
  ~MjVisualScale();
  std::unique_ptr<MjVisualScale> copy();
  // INSERT-GENERATED-MjVisualScale-DEFINITIONS
  mjVisualScale* get() const { return ptr_; }
  void set(mjVisualScale* ptr) { ptr_ = ptr; }

 private:
  mjVisualScale* ptr_;
  bool owned_ = false;
};

struct MjVisualRgba {
  MjVisualRgba();
  explicit MjVisualRgba(mjVisualRgba *ptr);
  MjVisualRgba(const MjVisualRgba &);
  MjVisualRgba &operator=(const MjVisualRgba &);
  ~MjVisualRgba();
  std::unique_ptr<MjVisualRgba> copy();
  // INSERT-GENERATED-MjVisualRgba-DEFINITIONS
  mjVisualRgba* get() const { return ptr_; }
  void set(mjVisualRgba* ptr) { ptr_ = ptr; }

 private:
  mjVisualRgba* ptr_;
  bool owned_ = false;
};

struct MjVisual {
  MjVisual();
  explicit MjVisual(mjVisual *ptr_);
  MjVisual(const MjVisual &);
  MjVisual &operator=(const MjVisual &);
  ~MjVisual();
  std::unique_ptr<MjVisual> copy();
  // INSERT-GENERATED-MjVisual-DEFINITIONS
  mjVisual* get() const { return ptr_; }
  void set(mjVisual* ptr) { ptr_ = ptr; }

 private:
  mjVisual* ptr_;
  bool owned_ = false;

 public:
  MjVisualGlobal global;
  MjVisualQuality quality;
  MjVisualHeadlight headlight;
  MjVisualMap map;
  MjVisualScale scale;
  MjVisualRgba rgba;
};

struct MjModel {
  explicit MjModel(mjModel *m);
  explicit MjModel(const MjModel &other);
  ~MjModel();
  std::unique_ptr<MjModel> copy();
  // INSERT-GENERATED-MjModel-DEFINITIONS
  mjModel* get() const { return ptr_; }
  void set(mjModel* ptr) { ptr_ = ptr; }

 private:
  mjModel* ptr_;

 public:
  MjOption opt;
  MjStatistic stat;
  MjVisual vis;
};

struct MjData {
  MjData(MjModel *m);
  explicit MjData(const MjModel &, const MjData &);
  ~MjData();
  std::vector<MjSolverStat> InitSolverArray();
  std::vector<MjTimerStat> InitTimerArray();
  std::vector<MjWarningStat> InitWarningArray();
  std::vector<MjContact> contact() const;
  std::unique_ptr<MjData> copy();
  // INSERT-GENERATED-MjData-DEFINITIONS
  mjData* get() const { return ptr_; }
  void set(mjData* ptr) { ptr_ = ptr; }

 private:
  mjData* ptr_;

 public:
  mjModel *model;
  std::vector<MjSolverStat> solver;
  std::vector<MjTimerStat> timer;
  std::vector<MjWarningStat> warning;
};

struct MjvScene {
  MjvScene();
  MjvScene(MjModel *m, int maxgeom);
  // MjvScene(const MjvScene &);
  ~MjvScene();
  std::unique_ptr<MjvScene> copy();
  int GetSumFlexFaces() const;
  std::vector<MjvLight> InitLightsArray();
  std::vector<MjvGLCamera> InitCameraArray();

  std::vector<MjvGeom> geoms() const;

  emscripten::val geomorder() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->ngeom, ptr_->geomorder));
  }
  emscripten::val flexedgeadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexedgeadr));
  }
  emscripten::val flexedgenum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexedgenum));
  }
  emscripten::val flexvertadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexvertadr));
  }
  emscripten::val flexvertnum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexvertnum));
  }
  emscripten::val flexfaceadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexfaceadr));
  }
  emscripten::val flexfacenum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexfacenum));
  }
  emscripten::val flexfaceused() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexfaceused));
  }
  emscripten::val flexedge() const {
    return emscripten::val(
        emscripten::typed_memory_view(2 * model->nflexedge, ptr_->flexedge));
  }
  emscripten::val flexvert() const {
    return emscripten::val(
        emscripten::typed_memory_view(3 * model->nflexvert, ptr_->flexvert));
  }
  emscripten::val skinfacenum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nskin, ptr_->skinfacenum));
  }
  emscripten::val skinvertadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nskin, ptr_->skinvertadr));
  }
  emscripten::val skinvertnum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nskin, ptr_->skinvertnum));
  }
  emscripten::val skinvert() const {
    return emscripten::val(
        emscripten::typed_memory_view(3 * model->nskinvert, ptr_->skinvert));
  }
  emscripten::val skinnormal() const {
    return emscripten::val(
        emscripten::typed_memory_view(3 * model->nskinvert, ptr_->skinnormal));
  }
  emscripten::val flexface() const {
    return emscripten::val(emscripten::typed_memory_view(
        9 * MjvScene::GetSumFlexFaces(), ptr_->flexface));
  }
  emscripten::val flexnormal() const {
    return emscripten::val(emscripten::typed_memory_view(
        9 * MjvScene::GetSumFlexFaces(), ptr_->flexnormal));
  }
  emscripten::val flextexcoord() const {
    return emscripten::val(emscripten::typed_memory_view(
        6 * MjvScene::GetSumFlexFaces(), ptr_->flextexcoord));
  }
  // INSERT-GENERATED-MjvScene-DEFINITIONS
  mjvScene* get() const { return ptr_; }
  void set(mjvScene* ptr) { ptr_ = ptr; }

 private:
  mjvScene* ptr_;
  bool owned_ = false;

 public:
  mjModel *model;
  std::vector<MjvLight> lights;
  std::vector<MjvGLCamera> camera;
};

struct MjSpec {
  MjSpec();
  explicit MjSpec(mjSpec *ptr);
  MjSpec(const MjSpec &);
  MjSpec &operator=(const MjSpec &);
  ~MjSpec();
  std::unique_ptr<MjSpec> copy();
  // INSERT-GENERATED-MjSpec-DEFINITIONS
  mjSpec* get() const { return ptr_; }
  void set(mjSpec* ptr) { ptr_ = ptr; }

 private:
  mjSpec* ptr_;
  bool owned_ = false;

 public:
  MjOption option;
  MjVisual visual;
  MjStatistic stat;
  MjsCompiler compiler;
  MjsElement element;
};

// TODO: Refactor, Structs Manually added so functions.cc compile -- //
struct MjpResourceProvider {
  MjpResourceProvider(mjpResourceProvider *ptr_) { ptr = ptr_; };
  ~MjpResourceProvider() {}
  mjpResourceProvider *get() const { return ptr; }
  mjpResourceProvider *ptr;
};

struct MjpPlugin {
  MjpPlugin(mjpPlugin *ptr_) { ptr = ptr_; };
  ~MjpPlugin() {}
  mjpPlugin *get() const { return ptr; }
  mjpPlugin *ptr;
};

// TODO: Factory and debug helper functions, some should be removed when
// functions are generated -- //
std::unique_ptr<MjModel> loadFromXML(std::string filename);
void step(MjModel *model, MjData *data);
void error(const std::string &msg);
void kinematics(MjModel *m, MjData *d);
std::unique_ptr<MjSpec> parseXMLString(const std::string &xml);
std::unique_ptr<MjsBody> findBody(MjSpec *spec, const std::string &name);
std::unique_ptr<MjsGeom> findGeom(MjSpec *spec, const std::string &name);

}  // namespace mujoco::wasm

#endif  // MUJOCO_WASM_CODEGEN_GENERATED_BINDINGS_H_
// NOLINTEND(whitespace/line_length)
