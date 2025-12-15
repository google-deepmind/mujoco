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
#include <mujoco/mjspec.h>
#include "engine/engine_util_errmem.h"
#include "wasm/unpack.h"

namespace mujoco::wasm {

using emscripten::enum_;
using emscripten::function;
using emscripten::val;
using emscripten::return_value_policy::reference;
using emscripten::return_value_policy::take_ownership;

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

template <typename WrapperType, typename ArrayType, typename SizeType>
std::vector<WrapperType> InitWrapperArray(ArrayType* array, SizeType size) {
  std::vector<WrapperType> result;
  result.reserve(size);
  for (int i = 0; i < size; ++i) {
    result.emplace_back(&array[i]);
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


// {{ ANONYMOUS_STRUCT_TYPEDEFS }}

// {{ STRUCTS_HEADER }}

// {{ STRUCTS_SOURCE }}

struct MjvScene {
  MjvScene();
  MjvScene(MjModel *m, int maxgeom);
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
  // INSERT-GENERATED-MjvScene-DECLARATION

 private:
  mjvScene* ptr_;
  bool owned_ = false;

 public:
  mjModel *model;
  std::vector<MjvLight> lights;
  std::vector<MjvGLCamera> camera;
};

MjModel::MjModel(mjModel* ptr)
    : ptr_(ptr), opt(&ptr->opt), vis(&ptr->vis), stat(&ptr->stat) {}

MjModel::MjModel(const MjModel& other)
    : ptr_(mj_copyModel(nullptr, other.get())),
      opt(&ptr_->opt),
      vis(&ptr_->vis),
      stat(&ptr_->stat) {}

MjModel::~MjModel() {
  if (ptr_) {
    mj_deleteModel(ptr_);
  }
}

mjModel* MjModel::get() const { return ptr_; }

void MjModel::set(mjModel* ptr) { ptr_ = ptr; }

MjData::MjData(MjModel* m) {
  model = m->get();
  ptr_ = mj_makeData(model);
  if (ptr_) {
    solver =
        InitWrapperArray<MjSolverStat>(get()->solver, mjNSOLVER * mjNISLAND);
    timer = InitWrapperArray<MjTimerStat>(get()->timer, mjNTIMER);
    warning = InitWrapperArray<MjWarningStat>(get()->warning, mjNWARNING);
  }
}

MjData::MjData(const MjModel& model, const MjData& other)
    : ptr_(mj_copyData(nullptr, model.get(), other.get())), model(model.get()) {
  if (ptr_) {
    solver =
        InitWrapperArray<MjSolverStat>(get()->solver, mjNSOLVER * mjNISLAND);
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

void MjData::set(mjData* ptr) { ptr_ = ptr; }

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

MjvScene::MjvScene(MjModel* m, int maxgeom) {
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
void MjvScene::set(mjvScene* ptr) { ptr_ = ptr; }

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
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat) {
  owned_ = true;
  mjs_defaultSpec(ptr_);
};

MjSpec::MjSpec(mjSpec *ptr)
    : ptr_(ptr),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat) {}

MjSpec::MjSpec(const MjSpec &other)
    : ptr_(mj_copySpec(other.get())),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat) {
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

std::unique_ptr<MjModel> mj_loadXML_wrapper(std::string filename) {
  char error[1000];
  mjModel *model = mj_loadXML(filename.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    printf("Loading error: %s\n", error);
    return nullptr;
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjSpec> parseXMLString_wrapper(const std::string &xml) {
  char error[1000];
  mjSpec *ptr = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
  if (!ptr) {
    printf("Could not create Spec from XML string: %s\n", error);
    return nullptr;
  }
  return std::unique_ptr<MjSpec>(new MjSpec(ptr));
}

std::unique_ptr<MjModel> mj_compile_wrapper(const MjSpec& spec) {
  mjSpec* spec_ptr = spec.get();
  mjModel* model = mj_compile(spec_ptr, nullptr);
  if (!model || mjs_isWarning(spec_ptr)) {
    mju_error("%s", mjs_getError(spec_ptr));
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

void error_wrapper(const String& msg) { mju_error("%s\n", msg.as<const std::string>().data()); }

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

// {{ WRAPPER_FUNCTIONS }}

EMSCRIPTEN_BINDINGS(mujoco_bindings) {
  // {{ ENUM_BINDINGS }}

  // {{ STRUCTS_BINDINGS }}

  // {{ FUNCTION_BINDINGS }}

  function("parseXMLString", &parseXMLString_wrapper, take_ownership());
  function("error", &error_wrapper);

  emscripten::class_<WasmBuffer<float>>("FloatBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<float>::FromArray)
      .function("GetPointer", &WasmBuffer<float>::GetPointer)
      .function("GetElementCount", &WasmBuffer<float>::GetElementCount)
      .function("GetView", &WasmBuffer<float>::GetView);

  emscripten::class_<WasmBuffer<double>>("DoubleBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<double>::FromArray)
      .function("GetPointer", &WasmBuffer<double>::GetPointer)
      .function("GetElementCount", &WasmBuffer<double>::GetElementCount)
      .function("GetView", &WasmBuffer<double>::GetView);

  emscripten::class_<WasmBuffer<int>>("IntBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<int>::FromArray)
      .function("GetPointer", &WasmBuffer<int>::GetPointer)
      .function("GetElementCount", &WasmBuffer<int>::GetElementCount)
      .function("GetView", &WasmBuffer<int>::GetView);

  emscripten::register_vector<std::string>("mjStringVec");
  emscripten::register_vector<int>("mjIntVec");
  emscripten::register_vector<mjIntVec>("mjIntVecVec");
  emscripten::register_vector<float>("mjFloatVec");
  emscripten::register_vector<mjFloatVec>("mjFloatVecVec");
  emscripten::register_vector<double>("mjDoubleVec");
  emscripten::register_vector<uint8_t>("mjByteVec");
  emscripten::register_vector<MjSolverStat>("MjSolverStatVec");
  emscripten::register_vector<MjTimerStat>("MjTimerStatVec");
  emscripten::register_vector<MjWarningStat>("MjWarningStatVec");
  emscripten::register_vector<MjContact>("MjContactVec");
  emscripten::register_vector<MjvLight>("MjvLightVec");
  emscripten::register_vector<MjvGLCamera>("MjvGLCameraVec");
  emscripten::register_vector<MjvGeom>("MjvGeomVec");

  // register_type() improves type information (val is mapped to any by default)
  emscripten::register_type<NumberArray>("number[]");
  emscripten::register_type<String>("string");

  emscripten::constant("mjMAXCONPAIR", mjMAXCONPAIR);
  emscripten::constant("mjMAXIMP", mjMAXIMP);
  emscripten::constant("mjMAXLIGHT", mjMAXLIGHT);
  emscripten::constant("mjMAXLINE", mjMAXLINE);
  emscripten::constant("mjMAXLINEPNT", mjMAXLINEPNT);
  emscripten::constant("mjMAXOVERLAY", mjMAXOVERLAY);
  emscripten::constant("mjMAXPLANEGRID", mjMAXPLANEGRID);
  emscripten::constant("mjMAXVAL", mjMAXVAL);
  emscripten::constant("mjMINIMP", mjMINIMP);
  emscripten::constant("mjMINMU", mjMINMU);
  emscripten::constant("mjMINVAL", mjMINVAL);
  emscripten::constant("mjNBIAS", mjNBIAS);
  emscripten::constant("mjNDYN", mjNDYN);
  emscripten::constant("mjNEQDATA", mjNEQDATA);
  emscripten::constant("mjNGAIN", mjNGAIN);
  emscripten::constant("mjNGROUP", mjNGROUP);
  emscripten::constant("mjNIMP", mjNIMP);
  emscripten::constant("mjNREF", mjNREF);
  emscripten::constant("mjNSOLVER", mjNSOLVER);
  emscripten::constant("mjPI", mjPI);
  emscripten::constant("mjVERSION_HEADER", mjVERSION_HEADER);

  // These complex constants are bound using function() rather than constant()
  emscripten::function("get_mjDISABLESTRING", &get_mjDISABLESTRING);
  emscripten::function("get_mjENABLESTRING", &get_mjENABLESTRING);
  emscripten::function("get_mjFRAMESTRING", &get_mjFRAMESTRING);
  emscripten::function("get_mjLABELSTRING", &get_mjLABELSTRING);
  emscripten::function("get_mjRNDSTRING", &get_mjRNDSTRING);
  emscripten::function("get_mjTIMERSTRING", &get_mjTIMERSTRING);
  emscripten::function("get_mjVISSTRING", &get_mjVISSTRING);
}

}  // namespace mujoco::wasm
// NOLINTEND(whitespace/semicolon)
// NOLINTEND(whitespace/line_length)
