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
// NOLINTBEGIN(whitespace/semicolon)

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>  // NOLINT
#include <memory>
#include <optional>  // NOLINT
#include <string>    // NOLINT
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "engine/engine_util_errmem.h"
#include "wasm/unpack.h"

namespace mujoco::wasm {

// Create the types for anonymous structs
using mjVisualGlobal = decltype(::mjVisual::global);
using mjVisualQuality = decltype(::mjVisual::quality);
using mjVisualHeadlight = decltype(::mjVisual::headlight);
using mjVisualMap = decltype(::mjVisual::map);
using mjVisualScale = decltype(::mjVisual::scale);
using mjVisualRgba = decltype(::mjVisual::rgba);

// {{ AUTOGENNED_STRUCTS_HEADER }}

struct MjVisual {
  MjVisual();
  explicit MjVisual(mjVisual *ptr_);
  MjVisual(const MjVisual &);
  MjVisual &operator=(const MjVisual &);
  ~MjVisual();
  std::unique_ptr<MjVisual> copy();
  mjVisual* get() const;
  void set(mjVisual* ptr);
  // INSERT-GENERATED-MjVisual-DEFINITIONS

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
  mjModel* get() const;
  void set(mjModel* ptr);
  // INSERT-GENERATED-MjModel-DEFINITIONS

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
  std::vector<MjContact> contact() const;
  std::unique_ptr<MjData> copy();
  mjData* get() const;
  void set(mjData* ptr);
  // INSERT-GENERATED-MjData-DEFINITIONS

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

  mjvScene* get() const;
  void set(mjvScene* ptr);

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
  mjSpec* get() const;
  void set(mjSpec* ptr);
  // INSERT-GENERATED-MjSpec-DEFINITIONS

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

using emscripten::enum_;
using emscripten::class_;
using emscripten::function;
using emscripten::val;
using emscripten::constant;
using emscripten::register_optional;
using emscripten::register_type;
using emscripten::register_vector;
using emscripten::return_value_policy::reference;
using emscripten::return_value_policy::take_ownership;

// ERROR HANDLER
void ThrowMujocoErrorToJS(const char* msg) {
  // Get a handle to the JS global Error constructor function, create a new
  // object instance and then throw the object as an exception using the
  // val::throw_() helper function.
  val(val::global("Error").new_(val("MuJoCo Error: " + std::string(msg))))
      .throw_();
}
__attribute__((constructor)) void InitMuJoCoErrorHandler() {
  mju_user_error = ThrowMujocoErrorToJS;
}

// CONSTANTS
template <size_t N>
val MakeValArray(const char* (&strings)[N]) {
  val result = val::array();
  for (int i = 0; i < N; i++) {
    result.call<void>("push", val(strings[i]));
  }
  return result;
}

template <size_t N, size_t M>
val MakeValArray3(const char* (&strings)[N][M]) {
  val result = val::array();
  for (int i = 0; i < N; i++) {
    val inner = val::array();
    for (int j = 0; j < M; j++) {
      inner.call<void>("push", val(strings[i][j]));
    }
    result.call<void>("push", inner);
  }
  return result;
}

val get_mjDISABLESTRING() { return MakeValArray(mjDISABLESTRING); }
val get_mjENABLESTRING() { return MakeValArray(mjENABLESTRING); }
val get_mjTIMERSTRING() { return MakeValArray(mjTIMERSTRING); }
val get_mjLABELSTRING() { return MakeValArray(mjLABELSTRING); }
val get_mjFRAMESTRING() { return MakeValArray(mjFRAMESTRING); }
val get_mjVISSTRING() { return MakeValArray3(mjVISSTRING); }
val get_mjRNDSTRING() { return MakeValArray3(mjRNDSTRING); }

EMSCRIPTEN_BINDINGS(constants) {
  // from mjmodel.h
  constant("mjPI", mjPI);
  constant("mjMAXVAL", mjMAXVAL);
  constant("mjMINMU", mjMINMU);
  constant("mjMINIMP", mjMINIMP);
  constant("mjMAXIMP", mjMAXIMP);
  constant("mjMAXCONPAIR", mjMAXCONPAIR);
  constant("mjNEQDATA", mjNEQDATA);
  constant("mjNDYN", mjNDYN);
  constant("mjNGAIN", mjNGAIN);
  constant("mjNBIAS", mjNBIAS);
  constant("mjNREF", mjNREF);
  constant("mjNIMP", mjNIMP);
  constant("mjNSOLVER", mjNSOLVER);

  // from mjvisualize.h
  constant("mjNGROUP", mjNGROUP);
  constant("mjMAXLIGHT", mjMAXLIGHT);
  constant("mjMAXOVERLAY", mjMAXOVERLAY);
  constant("mjMAXLINE", mjMAXLINE);
  constant("mjMAXLINEPNT", mjMAXLINEPNT);
  constant("mjMAXPLANEGRID", mjMAXPLANEGRID);

  // from mujoco.h
  constant("mjVERSION_HEADER", mjVERSION_HEADER);

  // from mjtnum.h
  constant("mjMINVAL", mjMINVAL);

  // emscripten::constant() is designed for simple, compile-time literal values
  // (like numbers or a single string literal), complex values need to be
  // bound as functions.
  emscripten::function("get_mjDISABLESTRING", &get_mjDISABLESTRING);
  emscripten::function("get_mjENABLESTRING", &get_mjENABLESTRING);
  emscripten::function("get_mjTIMERSTRING", &get_mjTIMERSTRING);
  emscripten::function("get_mjLABELSTRING", &get_mjLABELSTRING);
  emscripten::function("get_mjFRAMESTRING", &get_mjFRAMESTRING);
  emscripten::function("get_mjVISSTRING", &get_mjVISSTRING);
  emscripten::function("get_mjRNDSTRING", &get_mjRNDSTRING);
}

EMSCRIPTEN_BINDINGS(mujoco_enums) {
// {{ ENUM_BINDINGS }}
}

// STRUCTS
// {{ AUTOGENNED_STRUCTS_SOURCE }}
template <typename WrapperType, typename ArrayType, typename SizeType>
std::vector<WrapperType> InitWrapperArray(ArrayType* array, SizeType size) {
  std::vector<WrapperType> result;
  result.reserve(size);
  for (int i = 0; i < size; ++i) {
    result.emplace_back(&array[i]);
  }
  return result;
}

// =============== MjModel =============== //
MjModel::MjModel(mjModel *m)
    : ptr_(m), opt(&m->opt), stat(&m->stat), vis(&m->vis) {}
MjModel::MjModel(const MjModel &other)
    : ptr_(mj_copyModel(nullptr, other.get())),
      opt(&ptr_->opt),
      stat(&ptr_->stat),
      vis(&ptr_->vis) {}
MjModel::~MjModel() {
  if (ptr_) {
    mj_deleteModel(ptr_);
  }
}
mjModel* MjModel::get() const { return ptr_; }
void MjModel::set(mjModel *ptr) { ptr_ = ptr; }

// TODO(manevi): Consider passing `const MjModel& m` here, mj_makeData uses a const model.
// =============== MjData =============== //
MjData::MjData(MjModel *m) {
  model = m->get();
  ptr_ = mj_makeData(model);
  if (ptr_) {
    solver = InitWrapperArray<MjSolverStat>(get()->solver, mjNSOLVER * mjNISLAND);
    timer = InitWrapperArray<MjTimerStat>(get()->timer, mjNTIMER);
    warning = InitWrapperArray<MjWarningStat>(get()->warning, mjNWARNING);
  }
}
MjData::MjData(const MjModel &model, const MjData &other)
    : ptr_(mj_copyData(nullptr, model.get(), other.get())), model(model.get()) {
  if (ptr_) {
    solver = InitWrapperArray<MjSolverStat>(get()->solver, mjNSOLVER * mjNISLAND);
    timer = InitWrapperArray<MjTimerStat>(get()->timer, mjNTIMER);
    warning = InitWrapperArray<MjWarningStat>(get()->warning, mjNWARNING);
  }
}
MjData::~MjData() {
  if (ptr_) {
    mj_deleteData(ptr_);
  }
}
mjData* MjData::get() const { return ptr_; }
void MjData::set(mjData *ptr) { ptr_ = ptr; }

std::vector<MjContact> MjData::contact() const {
  return InitWrapperArray<MjContact>(get()->contact, get()->ncon);
}

MjvScene::MjvScene() {
  owned_ = true;
  ptr_ = new mjvScene;
  mjv_defaultScene(ptr_);
  mjv_makeScene(nullptr, ptr_, 0);
  lights = InitWrapperArray<MjvLight>(ptr_->lights, mjMAXLIGHT);
  camera = InitWrapperArray<MjvGLCamera>(ptr_->camera, 2);
};

MjvScene::MjvScene(MjModel *m, int maxgeom) {
  owned_ = true;
  model = m->get();
  ptr_ = new mjvScene;
  mjv_defaultScene(ptr_);
  mjv_makeScene(model, ptr_, maxgeom);
  lights = InitWrapperArray<MjvLight>(ptr_->lights, mjMAXLIGHT);
  camera = InitWrapperArray<MjvGLCamera>(ptr_->camera, 2);
};
MjvScene::~MjvScene() {
  if (owned_ && ptr_) {
    mjv_freeScene(ptr_);
    delete ptr_;
  }
}

mjvScene* MjvScene::get() const { return ptr_; }
void MjvScene::set(mjvScene *ptr) { ptr_ = ptr; }

// Taken from the python mujoco bindings code for MjvScene Wrapper
int MjvScene::GetSumFlexFaces() const {
  int nflexface = 0;
  int flexfacenum = 0;
  for (int f = 0; f < model->nflex; f++) {
    if (model->flex_dim[f] == 0) {
      // 1D : 0
      flexfacenum = 0;
    } else if (model->flex_dim[f] == 2) {
      // 2D: 2*fragments + 2*elements
      flexfacenum = 2 * model->flex_shellnum[f] + 2 * model->flex_elemnum[f];
    } else {
      // 3D: max(fragments, 4*maxlayer)
      // find number of elements in biggest layer
      int maxlayer = 0, layer = 0, nlayer = 1;
      while (nlayer) {
        nlayer = 0;
        for (int e = 0; e < model->flex_elemnum[f]; e++) {
          if (model->flex_elemlayer[model->flex_elemadr[f] + e] == layer) {
            nlayer++;
          }
        }
        maxlayer = mjMAX(maxlayer, nlayer);
        layer++;
      }
      flexfacenum = mjMAX(model->flex_shellnum[f], 4 * maxlayer);
    }

    // accumulate over flexes
    nflexface += flexfacenum;
  }
  return nflexface;
}

std::vector<MjvGeom> MjvScene::geoms() const {
  return InitWrapperArray<MjvGeom>(ptr_->geoms, ptr_->ngeom);
}

MjSpec::MjSpec()
    : ptr_(mj_makeSpec()),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat),
      compiler(&ptr_->compiler),
      element(ptr_->element) {
  owned_ = true;
  mjs_defaultSpec(ptr_);
};

MjSpec::MjSpec(mjSpec *ptr)
    : ptr_(ptr),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat),
      compiler(&ptr_->compiler),
      element(ptr_->element) {}

MjSpec::MjSpec(const MjSpec &other)
    : ptr_(mj_copySpec(other.get())),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat),
      compiler(&ptr_->compiler),
      element(ptr_->element) {
  owned_ = true;
}

MjSpec& MjSpec::operator=(const MjSpec &other) {
  if (this == &other) {
    return *this;
  }
  if (owned_ && ptr_) {
    mj_deleteSpec(ptr_);
  }
  ptr_ = mj_copySpec(other.get());
  owned_ = true;
  option.set(&ptr_->option);
  visual.set(&ptr_->visual);
  stat.set(&ptr_->stat);
  compiler.set(&ptr_->compiler);
  element.set(ptr_->element);
  return *this;
}

MjSpec::~MjSpec() {
  if (ptr_ && owned_) {
    mj_deleteSpec(ptr_);
  }
}

mjSpec *MjSpec::get() const { return ptr_; }
void MjSpec::set(mjSpec *ptr) { ptr_ = ptr; }

// ======= FACTORY AND HELPER FUNCTIONS ========= //
std::unique_ptr<MjModel> loadFromXML(std::string filename) {
  char error[1000];
  mjModel *model = mj_loadXML(filename.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    printf("Loading error: %s\n", error);
    return nullptr;
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjSpec> parseXMLString(const std::string &xml) {
  char error[1000];
  mjSpec *ptr = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
  if (!ptr) {
    printf("Could not create Spec from XML string: %s\n", error);
    return nullptr;
  }
  return std::unique_ptr<MjSpec>(new MjSpec(ptr));
}

EMSCRIPTEN_BINDINGS(mujoco_bindings) {
  function("parseXMLString", &parseXMLString, take_ownership());
  // {{ AUTOGENNED_STRUCTS_BINDINGS }}

  // TODO: should be generated in future CLs -- //
  emscripten::register_vector<MjSolverStat>("MjSolverStatVec");
  emscripten::register_vector<MjTimerStat>("MjTimerStatVec");
  emscripten::register_vector<MjWarningStat>("MjWarningStatVec");
  emscripten::register_vector<MjContact>("MjContactVec");
  emscripten::register_vector<MjvLight>("MjvLightVec");
  emscripten::register_vector<MjvGLCamera>("MjvGLCameraVec");
  emscripten::register_vector<MjvGeom>("MjvGeomVec");
}

// FUNCTIONS
EMSCRIPTEN_DECLARE_VAL_TYPE(NumberArray);
EMSCRIPTEN_DECLARE_VAL_TYPE(String);

// Raises an error if the given val is null or undefined.
// A macro is used so that the error contains the name of the variable.
// TODO(matijak): Remove this when we can handle strings using UNPACK_STRING?
#define CHECK_VAL(val)                                    \
  if (val.isNull()) {                                     \
    mju_error("Invalid argument: %s is null", #val);      \
  } else if (val.isUndefined()) {                         \
    mju_error("Invalid argument: %s is undefined", #val); \
  }
void error_wrapper(const String& msg) { mju_error("%s\n", msg.as<const std::string>().data()); }

// {{ WRAPPER_FUNCTIONS }}


void mju_printMatSparse_wrapper(const NumberArray& mat, const NumberArray& rownnz, const NumberArray& rowadr, const NumberArray& colind)
{
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(int, rownnz);
  UNPACK_ARRAY(int, rowadr);
  UNPACK_ARRAY(int, colind);
  CHECK_SIZES(rownnz, rowadr);
  mju_printMatSparse(mat_.data(), rowadr_.size(),
                     rownnz_.data(),
                     rowadr_.data(),
                     colind_.data());
}

void mj_solveM_wrapper(const MjModel& m, MjData& d, const val& x, const NumberArray& y)
{
  UNPACK_VALUE(mjtNum, x);
  UNPACK_ARRAY(mjtNum, y);
  CHECK_SIZES(x, y);
  CHECK_DIVISIBLE(x, m.nv());
  mj_solveM(m.get(), d.get(), x_.data(), y_.data(), x_div.quot);
}

void mj_solveM2_wrapper(const MjModel& m, MjData& d,
                        const val& x, const NumberArray& y,
                        const NumberArray& sqrtInvD) {
  UNPACK_VALUE(mjtNum, x);
  UNPACK_ARRAY(mjtNum, y);
  UNPACK_ARRAY(mjtNum, sqrtInvD);
  CHECK_SIZES(x, y);
  CHECK_SIZE(sqrtInvD, m.nv());
  CHECK_DIVISIBLE(x, m.nv());
  mj_solveM2(m.get(), d.get(), x_.data(), y_.data(), sqrtInvD_.data(), x_div.quot);
}

void mj_rne_wrapper(const MjModel& m, MjData& d, int flg_acc, const val& result)
{
  UNPACK_VALUE(mjtNum, result);
  CHECK_SIZE(result, m.nv());
  mj_rne(m.get(), d.get(), flg_acc, result_.data());
}

int mj_saveLastXML_wrapper(const String& filename, const MjModel& m) {
  CHECK_VAL(filename);
  std::array<char, 1024> error;
  int result = mj_saveLastXML(filename.as<const std::string>().data(), m.get(), error.data(), error.size());
  if (!result) {
    mju_error("%s", error.data());
  }
  return result;
}

int mj_setLengthRange_wrapper(const MjModel& m, const MjData& d, int index, const MjLROpt& opt) {
  std::array<char, 1024> error;
  int result = mj_setLengthRange(m.get(), d.get(), index, opt.get(), error.data(), error.size());
  if (!result) {
    mju_error("%s", error.data());
  }
  return result;
}

void mj_constraintUpdate_wrapper(const MjModel& m, MjData& d, const NumberArray& jar, const val& cost, int flg_coneHessian)
{
  UNPACK_ARRAY(mjtNum, jar);
  UNPACK_NULLABLE_VALUE(mjtNum, cost);
  CHECK_SIZE(cost, 1);
  CHECK_SIZE(jar, d.nefc());
  mj_constraintUpdate(m.get(), d.get(), jar_.data(), cost_.data(), flg_coneHessian);
}

void mj_getState_wrapper(const MjModel& m, const MjData& d, const val& state, unsigned int spec)
{
  UNPACK_VALUE(mjtNum, state);
  CHECK_SIZE(state, mj_stateSize(m.get(), spec));
  mj_getState(m.get(), d.get(), state_.data(), spec);
}

void mj_setState_wrapper(const MjModel& m, MjData& d, const NumberArray& state, unsigned int spec)
{
  UNPACK_ARRAY(mjtNum, state);
  CHECK_SIZE(state, mj_stateSize(m.get(), spec));
  mj_setState(m.get(), d.get(), state_.data(), spec);
}

void mj_mulJacVec_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, d.nefc());
  CHECK_SIZE(vec, m.nv());
  mj_mulJacVec(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_mulJacTVec_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, d.nefc());
  mj_mulJacTVec(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_jac_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, const NumberArray& point, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  UNPACK_ARRAY(mjtNum, point);
  CHECK_SIZE(point, 3);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jac(m.get(), d.get(), jacp_.data(), jacr_.data(), point_.data(), body);
}

void mj_jacBody_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacBody(m.get(), d.get(), jacp_.data(), jacr_.data(), body);
}

void mj_jacBodyCom_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacBodyCom(m.get(), d.get(), jacp_.data(), jacr_.data(), body);
}

void mj_jacSubtreeCom_wrapper(const MjModel& m, MjData& d, const val& jacp, int body)
{
  UNPACK_VALUE(mjtNum, jacp);
  CHECK_SIZE(jacp, m.nv() * 3);
  mj_jacSubtreeCom(m.get(), d.get(), jacp_.data(), body);
}

void mj_jacGeom_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int geom)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacGeom(m.get(), d.get(), jacp_.data(), jacr_.data(), geom);
}

void mj_jacSite_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int site)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacSite(m.get(), d.get(), jacp_.data(), jacr_.data(), site);
}

void mj_jacPointAxis_wrapper(const MjModel& m, MjData& d, const val& jacPoint, const val& jacAxis, const NumberArray& point, const NumberArray& axis, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacPoint);
  UNPACK_NULLABLE_VALUE(mjtNum, jacAxis);
  UNPACK_ARRAY(mjtNum, point);
  UNPACK_ARRAY(mjtNum, axis);
  CHECK_SIZE(point, 3);
  CHECK_SIZE(axis, 3);
  CHECK_SIZE(jacPoint, m.nv() * 3);
  CHECK_SIZE(jacAxis, m.nv() * 3);
  mj_jacPointAxis(m.get(), d.get(), jacPoint_.data(), jacAxis_.data(), point_.data(), axis_.data(), body);
}

void mj_jacDot_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, const NumberArray& point, int body)
{
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  UNPACK_ARRAY(mjtNum, point);
  CHECK_SIZE(point, 3);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacDot(m.get(), d.get(), jacp_.data(), jacr_.data(), point_.data(), body);
}

void mj_angmomMat_wrapper(const MjModel& m, MjData& d, const val& mat, int body)
{
  UNPACK_VALUE(mjtNum, mat);
  CHECK_SIZE(mat, m.nv() * 3);
  mj_angmomMat(m.get(), d.get(), mat_.data(), body);
}

void mj_fullM_wrapper(const MjModel& m, const val& dst, const NumberArray& M)
{
  UNPACK_VALUE(mjtNum, dst);
  UNPACK_ARRAY(mjtNum, M);
  CHECK_SIZE(M, m.nM());
  CHECK_SIZE(dst, m.nv() * m.nv());
  mj_fullM(m.get(), dst_.data(), M_.data());
}

void mj_mulM_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, m.nv());
  mj_mulM(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_mulM2_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, m.nv());
  mj_mulM2(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_addM_wrapper(const MjModel& m, MjData& d, const val& dst, const val& rownnz, const val& rowadr, const val& colind)
{
  UNPACK_VALUE(mjtNum, dst);
  UNPACK_NULLABLE_VALUE(int, rownnz);
  UNPACK_NULLABLE_VALUE(int, rowadr);
  UNPACK_NULLABLE_VALUE(int, colind);
  CHECK_SIZE(rownnz, m.nv());
  CHECK_SIZE(rowadr, m.nv());
  CHECK_SIZE(colind, m.nM());
  CHECK_SIZE(dst, m.nM());
  mj_addM(m.get(), d.get(), dst_.data(), rownnz_.data(), rowadr_.data(), colind_.data());
}

void mj_applyFT_wrapper(const MjModel& m, MjData& d, const NumberArray& force, const NumberArray& torque, const NumberArray& point, int body, const val& qfrc_target)
{
  UNPACK_NULLABLE_ARRAY(mjtNum, force);
  UNPACK_NULLABLE_ARRAY(mjtNum, torque);
  UNPACK_ARRAY(mjtNum, point);
  UNPACK_VALUE(mjtNum, qfrc_target);
  CHECK_SIZE(qfrc_target, m.nv());
  CHECK_SIZE(force, 3);
  CHECK_SIZE(torque, 3);
  CHECK_SIZE(point, 3);
  mj_applyFT(m.get(), d.get(), force_.data(), torque_.data(), point_.data(), body, qfrc_target_.data());
}

mjtNum mj_geomDistance_wrapper(const MjModel& m, const MjData& d, int geom1, int geom2, mjtNum distmax, const val& fromto)
{
  UNPACK_NULLABLE_VALUE(mjtNum, fromto);
  CHECK_SIZE(fromto, 6);
  return mj_geomDistance(m.get(), d.get(), geom1, geom2, distmax, fromto_.data());
}

void mj_differentiatePos_wrapper(const MjModel& m, const val& qvel, mjtNum dt, const NumberArray& qpos1, const NumberArray& qpos2)
{
  UNPACK_VALUE(mjtNum, qvel);
  UNPACK_ARRAY(mjtNum, qpos1);
  UNPACK_ARRAY(mjtNum, qpos2);
  CHECK_SIZE(qvel, m.nv());
  CHECK_SIZE(qpos1, m.nq());
  CHECK_SIZE(qpos2, m.nq());
  mj_differentiatePos(m.get(), qvel_.data(), dt, qpos1_.data(), qpos2_.data());
}

void mj_integratePos_wrapper(const MjModel& m, const val& qpos, const NumberArray& qvel, mjtNum dt)
{
  UNPACK_VALUE(mjtNum, qpos);
  UNPACK_ARRAY(mjtNum, qvel);
  CHECK_SIZE(qpos, m.nq());
  CHECK_SIZE(qvel, m.nv());
  mj_integratePos(m.get(), qpos_.data(), qvel_.data(), dt);
}

void mj_normalizeQuat_wrapper(const MjModel& m, const val& qpos)
{
  UNPACK_VALUE(mjtNum, qpos);
  CHECK_SIZE(qpos, m.nq());
  mj_normalizeQuat(m.get(), qpos_.data());
}

void mj_multiRay_wrapper(const MjModel& m, MjData& d, const NumberArray& pnt, const NumberArray& vec, const val& geomgroup, mjtByte flg_static, int bodyexclude, const val& geomid, const val& dist, int nray, mjtNum cutoff)
{
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_VALUE(mjtByte, geomgroup);
  UNPACK_VALUE(int, geomid);
  UNPACK_VALUE(mjtNum, dist);
  CHECK_SIZE(dist, nray);
  CHECK_SIZE(geomid, nray);
  CHECK_SIZE(vec, 3 * nray);
  mj_multiRay(m.get(), d.get(), pnt_.data(), vec_.data(), geomgroup_.data(), flg_static, bodyexclude, geomid_.data(), dist_.data(), nray, cutoff);
}

void mju_zero_wrapper(const val& res)
{
  UNPACK_VALUE(mjtNum, res);
  mju_zero(res_.data(), res_.size());
}

void mju_fill_wrapper(const val& res, mjtNum val)
{
  UNPACK_VALUE(mjtNum, res);
  mju_fill(res_.data(), val, res_.size());
}

void mju_copy_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_copy(res_.data(), vec_.data(), res_.size());
}

mjtNum mju_sum_wrapper(const NumberArray& vec)
{
  UNPACK_ARRAY(mjtNum, vec);
  return mju_sum(vec_.data(), vec_.size());
}

mjtNum mju_L1_wrapper(const NumberArray& vec)
{
  UNPACK_ARRAY(mjtNum, vec);
  return mju_L1(vec_.data(), vec_.size());
}

void mju_scl_wrapper(const val& res, const NumberArray& vec, mjtNum scl)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_scl(res_.data(), vec_.data(), scl, res_.size());
}

void mju_add_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  mju_add(res_.data(), vec1_.data(), vec2_.data(), res_.size());
}

void mju_sub_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  mju_sub(res_.data(), vec1_.data(), vec2_.data(), res_.size());
}

void mju_addTo_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_addTo(res_.data(), vec_.data(), res_.size());
}

void mju_subFrom_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_subFrom(res_.data(), vec_.data(), res_.size());
}

void mju_addToScl_wrapper(const val& res, const NumberArray& vec, mjtNum scl)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_addToScl(res_.data(), vec_.data(), scl, res_.size());
}

void mju_addScl_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2, mjtNum scl)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  mju_addScl(res_.data(), vec1_.data(), vec2_.data(), scl, res_.size());
}

mjtNum mju_normalize_wrapper(const val& res)
{
  UNPACK_VALUE(mjtNum, res);
  return mju_normalize(res_.data(), res_.size());
}

mjtNum mju_norm_wrapper(const NumberArray& res)
{
  UNPACK_ARRAY(mjtNum, res);
  return mju_norm(res_.data(), res_.size());
}

mjtNum mju_dot_wrapper(const NumberArray& vec1, const NumberArray& vec2)
{
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(vec1, vec2);
  return mju_dot(vec1_.data(), vec2_.data(), vec1_.size());
}

void mju_mulMatVec_wrapper(const val& res, const NumberArray& mat,
                           const NumberArray& vec, int nr, int nc) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nr);
  CHECK_SIZE(vec, nc);
  mju_mulMatVec(res_.data(), mat_.data(), vec_.data(), nr, nc);
}

void mju_mulMatTVec_wrapper(const val& res, const NumberArray& mat,
                            const NumberArray& vec, int nr, int nc) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nc);
  CHECK_SIZE(vec, nr);
  mju_mulMatTVec(res_.data(), mat_.data(), vec_.data(), nr, nc);
}

mjtNum mju_mulVecMatVec_wrapper(const NumberArray& vec1, const NumberArray& mat, const NumberArray& vec2)
{
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec2);
  int64_t vec1_times_vec2 = vec1_.size() * vec2_.size();
  CHECK_SIZES(vec1, vec2);
  CHECK_SIZE(mat, vec1_times_vec2);
  return mju_mulVecMatVec(vec1_.data(), mat_.data(), vec2_.data(), vec1_.size());
}

void mju_transpose_wrapper(const val& res, const NumberArray& mat, int nr, int nc)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nr * nc);
  mju_transpose(res_.data(), mat_.data(), nr, nc);
}

void mju_symmetrize_wrapper(const val& res, const NumberArray& mat, int n)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, n * n);
  CHECK_SIZE(res, n * n);
  mju_symmetrize(res_.data(), mat_.data(), n);
}

void mju_eye_wrapper(const val& mat)
{
  UNPACK_VALUE(mjtNum, mat);
  CHECK_PERFECT_SQUARE(mat);
  mju_eye(mat_.data(), mat_sqrt);
}

void mju_mulMatMat_wrapper(const val& res, const NumberArray& mat1, const NumberArray& mat2, int r1, int c1, int c2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat1);
  UNPACK_ARRAY(mjtNum, mat2);
  CHECK_SIZE(res, r1 * c2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, c1 * c2);
  mju_mulMatMat(res_.data(), mat1_.data(), mat2_.data(), r1, c1, c2);
}

void mju_mulMatMatT_wrapper(const val& res, const NumberArray& mat1, const NumberArray& mat2, int r1, int c1, int r2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat1);
  UNPACK_ARRAY(mjtNum, mat2);
  CHECK_SIZE(res, r1 * r2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, r2 * c1);
  mju_mulMatMatT(res_.data(), mat1_.data(), mat2_.data(), r1, c1, r2);
}

void mju_mulMatTMat_wrapper(const val& res, const NumberArray& mat1, const NumberArray& mat2, int r1, int c1, int c2)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat1);
  UNPACK_ARRAY(mjtNum, mat2);
  CHECK_SIZE(res, c1 * c2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, r1 * c2);
  mju_mulMatTMat(res_.data(), mat1_.data(), mat2_.data(), r1, c1, c2);
}

void mju_sqrMatTD_wrapper(const val& res, const NumberArray& mat, const NumberArray& diag, int nr, int nc)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, diag);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nc * nc);
  CHECK_SIZE(diag, nr);
  mju_sqrMatTD(res_.data(), mat_.data(), diag_.data(), nr, nc);
}

int mju_dense2sparse_wrapper(const val& res, const NumberArray& mat, int nr, int nc, const val& rownnz, const val& rowadr, const val& colind)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_VALUE(int, rownnz);
  UNPACK_VALUE(int, rowadr);
  UNPACK_VALUE(int, colind);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(rownnz, nr);
  CHECK_SIZE(rowadr, nr);
  CHECK_SIZE(colind, res_.size());
  return mju_dense2sparse(res_.data(), mat_.data(), nr, nc, rownnz_.data(), rowadr_.data(), colind_.data(), res_.size());
}

void mju_sparse2dense_wrapper(const val& res, const NumberArray& mat, int nr, int nc, const NumberArray& rownnz, const NumberArray& rowadr, const NumberArray& colind)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(int, rownnz);
  UNPACK_ARRAY(int, rowadr);
  UNPACK_ARRAY(int, colind);
  CHECK_SIZE(res, nr * nc);
  CHECK_SIZE(rownnz, nr);
  CHECK_SIZE(rowadr, nr);
  mju_sparse2dense(res_.data(), mat_.data(), nr, nc, rownnz_.data(), rowadr_.data(), colind_.data());
}

int mju_cholFactor_wrapper(const val& mat, mjtNum mindiag)
{
  UNPACK_VALUE(mjtNum, mat);
  CHECK_PERFECT_SQUARE(mat);
  return mju_cholFactor(mat_.data(), mat_sqrt, mindiag);
}

void mju_cholSolve_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_PERFECT_SQUARE(mat);
  CHECK_SIZE(res, mat_sqrt);
  CHECK_SIZE(vec, mat_sqrt);
  mju_cholSolve(res_.data(), mat_.data(), vec_.data(), mat_sqrt);
}

int mju_cholUpdate_wrapper(const val& mat, const val& x, int flg_plus)
{
  UNPACK_VALUE(mjtNum, mat);
  UNPACK_VALUE(mjtNum, x);
  CHECK_PERFECT_SQUARE(mat);
  CHECK_SIZE(x, mat_sqrt);
  return mju_cholUpdate(mat_.data(), x_.data(), mat_sqrt, flg_plus);
}

mjtNum mju_cholFactorBand_wrapper(const val& mat, int ntotal, int nband, int ndense, mjtNum diagadd, mjtNum diagmul)
{
  UNPACK_VALUE(mjtNum, mat);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  return mju_cholFactorBand(mat_.data(), ntotal, nband, ndense, diagadd, diagmul);
}

void mju_cholSolveBand_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec, int ntotal, int nband, int ndense)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal);
  CHECK_SIZE(vec, ntotal);
  mju_cholSolveBand(res_.data(), mat_.data(), vec_.data(), ntotal, nband, ndense);
}

void mju_band2Dense_wrapper(const val& res, const NumberArray& mat, int ntotal, int nband, int ndense, mjtByte flg_sym)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal * ntotal);
  mju_band2Dense(res_.data(), mat_.data(), ntotal, nband, ndense, flg_sym);
}

void mju_dense2Band_wrapper(const val& res, const NumberArray& mat, int ntotal, int nband, int ndense)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, ntotal * ntotal);
  CHECK_SIZE(res, (ntotal - ndense) * nband + ndense * ntotal);
  mju_dense2Band(res_.data(), mat_.data(), ntotal, nband, ndense);
}

void mju_bandMulMatVec_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec, int ntotal, int nband, int ndense, int nvec, mjtByte flg_sym)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal * nvec);
  CHECK_SIZE(vec, ntotal * nvec);
  mju_bandMulMatVec(res_.data(), mat_.data(), vec_.data(), ntotal, nband, ndense, nvec, flg_sym);
}

int mju_boxQP_wrapper(const val& res, const val& R, const val& index, const NumberArray& H, const NumberArray& g, const NumberArray& lower, const NumberArray& upper)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_VALUE(mjtNum, R);
  UNPACK_NULLABLE_VALUE(int, index);
  UNPACK_ARRAY(mjtNum, H);
  UNPACK_ARRAY(mjtNum, g);
  UNPACK_NULLABLE_ARRAY(mjtNum, lower);
  UNPACK_NULLABLE_ARRAY(mjtNum, upper);
  CHECK_SIZES(lower, res);
  CHECK_SIZES(upper, res);
  CHECK_SIZES(index, res);
  CHECK_SIZE(R, res_.size() * (res_.size() + 7))
  CHECK_PERFECT_SQUARE(H);
  CHECK_SIZES(g, res);
  return mju_boxQP(res_.data(), R_.data(), index_.data(), H_.data(), g_.data(), res_.size(), lower_.data(), upper_.data());
}

void mju_encodePyramid_wrapper(const val& pyramid, const NumberArray& force, const NumberArray& mu)
{
  UNPACK_VALUE(mjtNum, pyramid);
  UNPACK_ARRAY(mjtNum, force);
  UNPACK_ARRAY(mjtNum, mu);
  CHECK_SIZE(pyramid, 2 * mu_.size());
  CHECK_SIZE(force, mu_.size() + 1);
  mju_encodePyramid(pyramid_.data(), force_.data(), mu_.data(), mu_.size());
}

void mju_decodePyramid_wrapper(const val& force, const NumberArray& pyramid, const NumberArray& mu)
{
  UNPACK_VALUE(mjtNum, force);
  UNPACK_ARRAY(mjtNum, pyramid);
  UNPACK_ARRAY(mjtNum, mu);
  CHECK_SIZE(pyramid, 2 * mu_.size());
  CHECK_SIZE(force, mu_.size() + 1);
  mju_decodePyramid(force_.data(), pyramid_.data(), mu_.data(), mu_.size());
}

int mju_isZero_wrapper(const val& vec)
{
  UNPACK_VALUE(mjtNum, vec);
  return mju_isZero(vec_.data(), vec_.size());
}

void mju_f2n_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(float, vec);
  CHECK_SIZES(res, vec);
  mju_f2n(res_.data(), vec_.data(), res_.size());
}

void mju_n2f_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(float, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_n2f(res_.data(), vec_.data(), res_.size());
}

void mju_d2n_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(double, vec);
  CHECK_SIZES(res, vec);
  mju_d2n(res_.data(), vec_.data(), res_.size());
}

void mju_n2d_wrapper(const val& res, const NumberArray& vec)
{
  UNPACK_VALUE(double, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  mju_n2d(res_.data(), vec_.data(), res_.size());
}

void mju_insertionSort_wrapper(const val& list)
{
  UNPACK_VALUE(mjtNum, list);
  mju_insertionSort(list_.data(), list_.size());
}

void mju_insertionSortInt_wrapper(const val& list)
{
  UNPACK_VALUE(int, list);
  mju_insertionSortInt(list_.data(), list_.size());
}

void mjd_transitionFD_wrapper(const MjModel& m, MjData& d, mjtNum eps, mjtByte flg_centered, const val& A, const val& B, const val& C, const val& D)
{
  UNPACK_NULLABLE_VALUE(mjtNum, A);
  UNPACK_NULLABLE_VALUE(mjtNum, B);
  UNPACK_NULLABLE_VALUE(mjtNum, C);
  UNPACK_NULLABLE_VALUE(mjtNum, D);
  CHECK_SIZE(A, (2 * m.nv() + m.na()) * (2 * m.nv() + m.na()));
  CHECK_SIZE(B, (2 * m.nv() + m.na()) * m.nu());
  CHECK_SIZE(C, m.nsensordata() * (2 * m.nv() + m.na()));
  CHECK_SIZE(D, m.nsensordata() * m.nu());
  mjd_transitionFD(m.get(), d.get(), eps, flg_centered, A_.data(), B_.data(), C_.data(), D_.data());
}

void mjd_inverseFD_wrapper(const MjModel& m, MjData& d, mjtNum eps, mjtByte flg_actuation, const val& DfDq, const val& DfDv, const val& DfDa, const val& DsDq, const val& DsDv, const val& DsDa, const val& DmDq)
{
  UNPACK_NULLABLE_VALUE(mjtNum, DfDq);
  UNPACK_NULLABLE_VALUE(mjtNum, DfDv);
  UNPACK_NULLABLE_VALUE(mjtNum, DfDa);
  UNPACK_NULLABLE_VALUE(mjtNum, DsDq);
  UNPACK_NULLABLE_VALUE(mjtNum, DsDv);
  UNPACK_NULLABLE_VALUE(mjtNum, DsDa);
  UNPACK_NULLABLE_VALUE(mjtNum, DmDq);
  CHECK_SIZE(DfDq, m.nv() * m.nv());
  CHECK_SIZE(DfDv, m.nv() * m.nv());
  CHECK_SIZE(DfDa, m.nv() * m.nv());
  CHECK_SIZE(DsDq, m.nv() * m.nsensordata());
  CHECK_SIZE(DsDv, m.nv() * m.nsensordata());
  CHECK_SIZE(DsDa, m.nv() * m.nsensordata());
  CHECK_SIZE(DmDq, m.nv() * m.nM());
  mjd_inverseFD(m.get(), d.get(), eps, flg_actuation, DfDq_.data(), DfDv_.data(), DfDa_.data(),
                DsDq_.data(), DsDv_.data(), DsDa_.data(), DmDq_.data());
}

void mjd_subQuat_wrapper(const NumberArray& qa, const NumberArray& qb, const val& Da, const val& Db)
{
  UNPACK_ARRAY(mjtNum, qa);
  UNPACK_ARRAY(mjtNum, qb);
  UNPACK_NULLABLE_VALUE(mjtNum, Da);
  UNPACK_NULLABLE_VALUE(mjtNum, Db);
  CHECK_SIZE(qa, 4);
  CHECK_SIZE(qb, 4);
  CHECK_SIZE(Da, 9);
  CHECK_SIZE(Db, 9);
  mjd_subQuat(qa_.data(), qb_.data(), Da_.data(), Db_.data());
}

EMSCRIPTEN_BINDINGS(mujoco_functions) {
// {{ FUNCTION_BINDINGS }}
  function("error", &error_wrapper);
  function("mju_printMatSparse", &mju_printMatSparse_wrapper);
  function("mj_solveM", &mj_solveM_wrapper);
  function("mj_solveM2", &mj_solveM2_wrapper);
  function("mj_rne", &mj_rne_wrapper);
  function("mj_saveLastXML", &mj_saveLastXML_wrapper);
  function("mj_setLengthRange", &mj_setLengthRange_wrapper);
  function("mj_constraintUpdate", &mj_constraintUpdate_wrapper);
  function("mj_getState", &mj_getState_wrapper);
  function("mj_setState", &mj_setState_wrapper);
  function("mj_mulJacVec", &mj_mulJacVec_wrapper);
  function("mj_mulJacTVec", &mj_mulJacTVec_wrapper);
  function("mj_jac", &mj_jac_wrapper);
  function("mj_jacBody", &mj_jacBody_wrapper);
  function("mj_jacBodyCom", &mj_jacBodyCom_wrapper);
  function("mj_jacSubtreeCom", &mj_jacSubtreeCom_wrapper);
  function("mj_jacGeom", &mj_jacGeom_wrapper);
  function("mj_jacSite", &mj_jacSite_wrapper);
  function("mj_jacPointAxis", &mj_jacPointAxis_wrapper);
  function("mj_jacDot", &mj_jacDot_wrapper);
  function("mj_angmomMat", &mj_angmomMat_wrapper);
  function("mj_fullM", &mj_fullM_wrapper);
  function("mj_mulM", &mj_mulM_wrapper);
  function("mj_mulM2", &mj_mulM2_wrapper);
  function("mj_addM", &mj_addM_wrapper);
  function("mj_applyFT", &mj_applyFT_wrapper);
  function("mj_geomDistance", &mj_geomDistance_wrapper);
  function("mj_differentiatePos", &mj_differentiatePos_wrapper);
  function("mj_integratePos", &mj_integratePos_wrapper);
  function("mj_normalizeQuat", &mj_normalizeQuat_wrapper);
  function("mj_multiRay", &mj_multiRay_wrapper);
  function("mju_zero", &mju_zero_wrapper);
  function("mju_fill", &mju_fill_wrapper);
  function("mju_copy", &mju_copy_wrapper);
  function("mju_sum", &mju_sum_wrapper);
  function("mju_L1", &mju_L1_wrapper);
  function("mju_scl", &mju_scl_wrapper);
  function("mju_add", &mju_add_wrapper);
  function("mju_sub", &mju_sub_wrapper);
  function("mju_addTo", &mju_addTo_wrapper);
  function("mju_subFrom", &mju_subFrom_wrapper);
  function("mju_addToScl", &mju_addToScl_wrapper);
  function("mju_addScl", &mju_addScl_wrapper);
  function("mju_normalize", &mju_normalize_wrapper);
  function("mju_norm", &mju_norm_wrapper);
  function("mju_dot", &mju_dot_wrapper);
  function("mju_mulMatVec", &mju_mulMatVec_wrapper);
  function("mju_mulMatTVec", &mju_mulMatTVec_wrapper);
  function("mju_mulVecMatVec", &mju_mulVecMatVec_wrapper);
  function("mju_transpose", &mju_transpose_wrapper);
  function("mju_symmetrize", &mju_symmetrize_wrapper);
  function("mju_eye", &mju_eye_wrapper);
  function("mju_mulMatMat", &mju_mulMatMat_wrapper);
  function("mju_mulMatMatT", &mju_mulMatMatT_wrapper);
  function("mju_mulMatTMat", &mju_mulMatTMat_wrapper);
  function("mju_sqrMatTD", &mju_sqrMatTD_wrapper);
  function("mju_dense2sparse", &mju_dense2sparse_wrapper);
  function("mju_sparse2dense", &mju_sparse2dense_wrapper);
  function("mju_cholFactor", &mju_cholFactor_wrapper);
  function("mju_cholSolve", &mju_cholSolve_wrapper);
  function("mju_cholUpdate", &mju_cholUpdate_wrapper);
  function("mju_cholFactorBand", &mju_cholFactorBand_wrapper);
  function("mju_cholSolveBand", &mju_cholSolveBand_wrapper);
  function("mju_band2Dense", &mju_band2Dense_wrapper);
  function("mju_dense2Band", &mju_dense2Band_wrapper);
  function("mju_bandMulMatVec", &mju_bandMulMatVec_wrapper);
  function("mju_boxQP", &mju_boxQP_wrapper);
  function("mju_encodePyramid", &mju_encodePyramid_wrapper);
  function("mju_decodePyramid", &mju_decodePyramid_wrapper);
  function("mju_isZero", &mju_isZero_wrapper);
  function("mju_f2n", &mju_f2n_wrapper);
  function("mju_n2f", &mju_n2f_wrapper);
  function("mju_d2n", &mju_d2n_wrapper);
  function("mju_n2d", &mju_n2d_wrapper);
  function("mju_insertionSort", &mju_insertionSort_wrapper);
  function("mju_insertionSortInt", &mju_insertionSortInt_wrapper);
  function("mjd_transitionFD", &mjd_transitionFD_wrapper);
  function("mjd_inverseFD", &mjd_inverseFD_wrapper);
  function("mjd_subQuat", &mjd_subQuat_wrapper);
  class_<WasmBuffer<float>>("FloatBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<float>::FromArray)
      .function("GetPointer", &WasmBuffer<float>::GetPointer)
      .function("GetElementCount", &WasmBuffer<float>::GetElementCount)
      .function("GetView", &WasmBuffer<float>::GetView);
  class_<WasmBuffer<double>>("DoubleBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<double>::FromArray)
      .function("GetPointer", &WasmBuffer<double>::GetPointer)
      .function("GetElementCount", &WasmBuffer<double>::GetElementCount)
      .function("GetView", &WasmBuffer<double>::GetView);
  class_<WasmBuffer<int>>("IntBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<int>::FromArray)
      .function("GetPointer", &WasmBuffer<int>::GetPointer)
      .function("GetElementCount", &WasmBuffer<int>::GetElementCount)
      .function("GetView", &WasmBuffer<int>::GetView);
  register_vector<std::string>("mjStringVec");
  register_vector<int>("mjIntVec");
  register_vector<mjIntVec>("mjIntVecVec");
  register_vector<float>("mjFloatVec");
  register_vector<mjFloatVec>("mjFloatVecVec");
  register_vector<double>("mjDoubleVec");
  // register_type gives better type information (val is mapped to any by default)
  register_type<NumberArray>("number[]");
  register_type<String>("string");
  register_vector<uint8_t>("mjByteVec");
  register_optional<MjsElement>();
  register_optional<MjsBody>();
  register_optional<MjsSite>();
  register_optional<MjsJoint>();
  register_optional<MjsGeom>();
  register_optional<MjsCamera>();
  register_optional<MjsLight>();
  register_optional<MjsFrame>();
  register_optional<MjsActuator>();
  register_optional<MjsSensor>();
  register_optional<MjsFlex>();
  register_optional<MjsPair>();
  register_optional<MjsExclude>();
  register_optional<MjsEquality>();
  register_optional<MjsTendon>();
  register_optional<MjsWrap>();
  register_optional<MjsNumeric>();
  register_optional<MjsText>();
  register_optional<MjsTuple>();
  register_optional<MjsKey>();
  register_optional<MjsPlugin>();
  register_optional<MjsDefault>();
  register_optional<MjsMesh>();
  register_optional<MjsHField>();
  register_optional<MjsSkin>();
  register_optional<MjsTexture>();
  register_optional<MjsMaterial>();
  register_optional<MjSpec>();
}

}  // namespace mujoco::wasm
// NOLINTEND(whitespace/semicolon)
// NOLINTEND(whitespace/line_length)
