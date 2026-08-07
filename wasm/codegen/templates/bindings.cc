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

#include "wasm/codegen/generated/bindings.h"

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/em_asm.h>
#include <emscripten/val.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>  // NOLINT
#include <memory>
#include <optional>  // NOLINT
#include <sstream>
#include <string>  // NOLINT
#include <string_view>
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "engine/engine_util_errmem.h"
#include "wasm/unpack.h"
#include "python/mujoco/indexer_xmacro.h"

namespace mujoco::wasm {


void ThrowMujocoErrorToJS(const mjLogMessage* msg) {
  if (msg->level == mjLOG_ERROR) {
    std::string message = msg->subject;
    if (msg->func) {
      message = std::string(msg->func) + ": " + msg->subject;
    }
    // Get a handle to the JS global Error constructor function, create a new
    // object instance and then throw the object as an exception using the
    // val::throw_() helper function.
    val(val::global("Error").new_(val("MuJoCo Error: " + message))).throw_();
  }
}
__attribute__((constructor)) void InitMuJoCoErrorHandler() {
  mju_setLogHandler(ThrowMujocoErrorToJS);
}

// Generates a descriptive error message for when a key lookup fails.
// The message includes the invalid name and a list of valid names of the
// specified object type currently present in the model.
//
// Arguments:
//   model: Pointer to the mjModel.
//   objtype: The mjOBJ_* enum value representing the object type.
//   count: The number of objects of the given type in the model.
//   name: The invalid name that was looked up.
//
// Returns:
//   A string containing the error message.
std::string KeyErrorMessage(const mjModel* model, int objtype, int count,
                            std::string_view name, std::string_view accessor_name) {
  std::vector<std::string> valid_names;
  valid_names.reserve(count);
  for (int i = 0; i < count; ++i) {
    const char* n = mj_id2name(model, objtype, i);
    if (n) {
      valid_names.push_back(n);
    }
  }
  std::sort(valid_names.begin(), valid_names.end());

  std::ostringstream message;
  message << "Invalid name '" << name << "' for " << accessor_name
          << ". Valid names: [";
  for (size_t i = 0; i < valid_names.size(); ++i) {
    message << "'" << valid_names[i] << "'";
    if (i < valid_names.size() - 1) {
      message << ", ";
    }
  }
  message << "]";
  return message.str();
}

std::string IndexErrorMessage(int index, int count,
                              std::string_view accessor_name) {
  std::ostringstream message;
  message << "Invalid index " << index << " for " << accessor_name
          << ". Valid indices from 0 to " << count - 1;
  return message.str();
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

// {{ STRUCTS_SOURCE }}

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
      stat(&ptr_->stat),
      authored(&ptr_->authored) {
  owned_ = true;
  mjs_defaultSpec(ptr_);
};

MjSpec::MjSpec(mjSpec *ptr)
    : ptr_(ptr),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat),
      authored(&ptr_->authored) {}

MjSpec::MjSpec(const MjSpec &other)
    : ptr_(mj_copySpec(other.get())),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat),
      authored(&ptr_->authored) {
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

emscripten::val MjSpec::timer() const {
  return emscripten::val(emscripten::typed_memory_view(9, mjs_getTimer(ptr_)));
}

std::unique_ptr<MjModel> mj_loadXML_wrapper_1(std::string filename) {
  char error[1000];
  mjModel *model = mj_loadXML(filename.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    mju_error("Loading error: %s\n", error);
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjModel> mj_loadXML_wrapper_2(std::string filename, const MjVFS& vfs) {
  char error[1000];
  mjModel *model = mj_loadXML(filename.c_str(), vfs.get(), error, sizeof(error));
  if (!model) {
    mju_error("Loading error: %s\n", error);
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

void mj_saveModel_wrapper(const MjModel& m, const StringOrNull& filename, const val& buffer) {
  UNPACK_NULLABLE_STRING(filename);
  UNPACK_NULLABLE_VALUE(uint8_t, buffer);
  mj_saveModel(m.get(), filename_.data(), buffer_.data(), static_cast<int>(buffer_.size()));
}

std::unique_ptr<MjModel> mj_loadModel_wrapper(std::string filename, const MjVFS& vfs) {
  mjModel *model = mj_loadModel(filename.c_str(), vfs.get());
  if (!model) {
    mju_error("Failed to load from mjb");
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjModel> from_xml_string_wrapper_1(const std::string& xml) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  const char* filename = "model.xml";
  int add_result = mj_addBufferVFS(&vfs, filename, xml.c_str(), xml.length());
  if (add_result != 0) {
    mj_deleteVFS(&vfs);
    mju_error("Could not add XML string to VFS: %d", add_result);
  }
  char error[1000];
  mjModel* model = mj_loadXML(filename, &vfs, error, sizeof(error));
  mj_deleteVFS(&vfs);
  if (!model) {
    mju_error("Loading error: %s\n", error);
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjModel> from_xml_string_wrapper_2(const std::string& xml, const MjVFS& vfs) {
  std::string filename = "model.xml";
  int add_result = mj_addBufferVFS(vfs.get(), filename.c_str(), xml.c_str(), xml.length());
  if (add_result != 0) {
    mju_error("Could not add XML string to VFS: %d", add_result);
  }
  char error[1000];
  mjModel* model = mj_loadXML(filename.c_str(), vfs.get(), error, sizeof(error));
  mj_deleteFileVFS(vfs.get(), filename.c_str());
  if (!model) {
    mju_error("Loading error: %s\n", error);
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjSpec> parseXMLString_wrapper(const std::string &xml) {
  char error[1000];
  mjSpec *ptr = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
  if (!ptr) {
    mju_error("Could not create Spec from XML string: %s\n", error);
  }
  return std::unique_ptr<MjSpec>(new MjSpec(ptr));
}

std::unique_ptr<MjSpec> parseXMLString_wrapper_2(const std::string &xml, const MjVFS& vfs) {
  char error[1000];
  mjSpec *ptr = mj_parseXMLString(xml.c_str(), vfs.get(), error, sizeof(error));
  if (!ptr) {
    mju_error("Could not create Spec from XML string: %s\n", error);
  }
  return std::unique_ptr<MjSpec>(new MjSpec(ptr));
}

std::unique_ptr<MjModel> mj_compile_wrapper_1(const MjSpec& spec) {
  mjSpec* spec_ptr = spec.get();

  // suppress stderr playback: warnings are raised via console.warn() below
  mjfLogHandler prev = _mjPRIVATE_setTlsLogHandler([](const mjLogMessage*) {});
  mjModel* model = mj_compile(spec_ptr, nullptr);
  _mjPRIVATE_setTlsLogHandler(prev);
  if (!model) {
    mju_error("%s", mjs_getError(spec_ptr));
  }
  int num_warnings = mjs_numWarnings(spec_ptr);
  if (num_warnings > 0) {
    for (int i = 0; i < num_warnings; ++i) {
      val::global("console").call<void>(
          "warn",
          val("MuJoCo Warning: " + std::string(mjs_getWarning(spec_ptr, i))));
    }
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjModel> mj_compile_wrapper_2(const MjSpec& spec, const MjVFS& vfs) {
  mjSpec* spec_ptr = spec.get();
  mjVFS* vfs_ptr = vfs.get();

  // suppress stderr playback: warnings are raised via console.warn() below
  mjfLogHandler prev = _mjPRIVATE_setTlsLogHandler([](const mjLogMessage*) {});
  mjModel* model = mj_compile(spec_ptr, vfs_ptr);
  _mjPRIVATE_setTlsLogHandler(prev);
  if (!model) {
    mju_error("%s", mjs_getError(spec_ptr));
  }
  int num_warnings = mjs_numWarnings(spec_ptr);
  if (num_warnings > 0) {
    for (int i = 0; i < num_warnings; ++i) {
      val::global("console").call<void>(
          "warn",
          val("MuJoCo Warning: " + std::string(mjs_getWarning(spec_ptr, i))));
    }
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

void mju_info_wrapper(int topic, const String& msg) {
  CHECK_VAL(msg);
  mju_info(topic, "%s", msg.as<const std::string>().data());
}

// {{ WRAPPER_FUNCTIONS }}

EMSCRIPTEN_BINDINGS(mujoco_bindings) {
  // {{ ENUM_BINDINGS }}

  // Bindings for the MjModel accessor classes.
  #define X(type, prefix, var, dim0, dim1) .property(#var, &Accessor::get_##var, &Accessor::set_##var)
  #define X_ACCESSOR(NAME, Name, OBJTYPE, field, nfield)        \
    {                                                           \
      using Accessor = MjModel##Name##Accessor;                 \
      emscripten::class_<Accessor>("MjModel" #Name "Accessor")  \
          .property("id", &Accessor::id)                        \
          .property("name", &Accessor::name)                    \
          MJMODEL_##NAME;                                       \
    }
    MJMODEL_ACCESSORS
  #undef X
  #undef X_ACCESSOR

  // Bindings for the MjData accessor classes.
  #define X(type, prefix, var, dim0, dim1) .property(#var, &Accessor::get_##var, &Accessor::set_##var)
  #define X_ACCESSOR(NAME, Name, OBJTYPE, field, nfield)       \
    {                                                          \
      using Accessor = MjData##Name##Accessor;                 \
      emscripten::class_<Accessor>("MjData" #Name "Accessor")  \
          .property("id", &Accessor::id)                       \
          .property("name", &Accessor::name)                   \
          MJDATA_##NAME;                                       \
    }
    MJDATA_ACCESSORS
  #undef X
  #undef X_ACCESSOR

  // {{ STRUCTS_BINDINGS }}

  emscripten::class_<MjVFS>("MjVFS")
      .constructor<>()
      .function("addBuffer", &MjVFS::AddBuffer)
      .function("deleteFile", &MjVFS::DeleteFile);

  // {{ FUNCTION_BINDINGS }}
  // parseXMLString is bound using two overloads to handle the optional MjVFS argument.
  function("parseXMLString", emscripten::select_overload<std::unique_ptr<MjSpec>(const std::string&)>(&parseXMLString_wrapper), take_ownership());
  function("parseXMLString", emscripten::select_overload<std::unique_ptr<MjSpec>(const std::string&, const MjVFS&)>(&parseXMLString_wrapper_2), take_ownership());
  function("error", &error_wrapper);
  function("mj_saveModel", &mj_saveModel_wrapper);
  function("mj_saveLastXML", &mj_saveLastXML_wrapper);
  function("mj_setLengthRange", &mj_setLengthRange_wrapper);
  function("mju_info", &mju_info_wrapper);
  // mj_compile is bound using two overloads to handle the optional MjVFS argument,
  // as using std::optional<MjVFS> caused memory errors due to missing copy/move constructors.
  function("mj_compile", emscripten::select_overload<std::unique_ptr<MjModel>(const MjSpec&)>(&mj_compile_wrapper_1));
  function("mj_compile", emscripten::select_overload<std::unique_ptr<MjModel>(const MjSpec&, const MjVFS&)>(&mj_compile_wrapper_2));
  function("from_xml_string", emscripten::select_overload<std::unique_ptr<MjModel>(const std::string&)>(&from_xml_string_wrapper_1));
  function("from_xml_string", emscripten::select_overload<std::unique_ptr<MjModel>(const std::string&, const MjVFS&)>(&from_xml_string_wrapper_2));

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

  emscripten::class_<WasmBuffer<uint8_t>>("Uint8Buffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<uint8_t>::FromArray)
      .function("GetPointer", &WasmBuffer<uint8_t>::GetPointer)
      .function("GetElementCount", &WasmBuffer<uint8_t>::GetElementCount)
      .function("GetView", &WasmBuffer<uint8_t>::GetView);

  emscripten::register_vector<std::string>("mjStringVec");
  emscripten::register_vector<std::vector<std::string>>("mjStringVecVec");
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
  // NumberOrString is used in functions returning accessors, allowing users to
  // get an accessor by name (string) or id (number).
  emscripten::register_type<NumberOrString>("number|string");
  emscripten::register_type<NumberArray>("number[]");
  emscripten::register_type<String>("string");
  emscripten::register_type<StringOrNull>("string|null");

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

  emscripten::function("get_mjDISABLESTRING", &get_mjDISABLESTRING);
  emscripten::function("get_mjENABLESTRING", &get_mjENABLESTRING);
  emscripten::function("get_mjFRAMESTRING", &get_mjFRAMESTRING);
  emscripten::function("get_mjLABELSTRING", &get_mjLABELSTRING);
  emscripten::function("get_mjRNDSTRING", &get_mjRNDSTRING);
  emscripten::function("get_mjTIMERSTRING", &get_mjTIMERSTRING);
  emscripten::function("get_mjVISSTRING", &get_mjVISSTRING);
  // Bind these complex constants as properties on the module object.
  // We use emscripten::constant with emscripten::val::array() to type them
  // as `any` in TypeScript. At runtime, the EM_ASM block below overrides
  // these properties with getters that return native JavaScript arrays
  // (string[] or string[][]) via the get_ functions above, which is more
  // performant and idiomatic than vector wrappers.
  emscripten::constant("mjDISABLESTRING", emscripten::val::array());
  emscripten::constant("mjENABLESTRING", emscripten::val::array());
  emscripten::constant("mjFRAMESTRING", emscripten::val::array());
  emscripten::constant("mjLABELSTRING", emscripten::val::array());
  emscripten::constant("mjRNDSTRING", emscripten::val::array());
  emscripten::constant("mjTIMERSTRING", emscripten::val::array());
  emscripten::constant("mjVISSTRING", emscripten::val::array());
  EM_ASM({
    if (typeof Module !== "undefined") {
      "mjDISABLESTRING mjENABLESTRING mjFRAMESTRING mjLABELSTRING mjRNDSTRING mjTIMERSTRING mjVISSTRING".split(" ").forEach(function(name) {
        Object.defineProperty(Module, name, {
          get: function() { return Module["get_" + name](); },
          set: function(v) { },
          enumerable: true,
          configurable: true
        });
      });
    }
  });
}

}  // namespace mujoco::wasm
// NOLINTEND(whitespace/semicolon)
// NOLINTEND(whitespace/line_length)
