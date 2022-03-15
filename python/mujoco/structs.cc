// Copyright 2022 DeepMind Technologies Limited
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

#include "structs.h"

#include <Python.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ios>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <absl/container/flat_hash_map.h>
#include <mjxmacro.h>
#include <mujoco.h>
#include "errors.h"
#include "function_traits.h"
#include "indexers.h"
#include "raw.h"
#include "serialization.h"
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace mujoco::python::_impl {

namespace py = ::pybind11;

namespace {
// Returns the shape of a NumPy array given the dimensions from an X Macro.
// If dim1 is a _literal_ constant 1, the resulting array is 1-dimensional of
// length dim0, otherwise the resulting array is 2-dimensional of shape
// (dim0, dim1).
#define X_ARRAY_SHAPE(dim0, dim1) XArrayShapeImpl(#dim1)((dim0), (dim1))

std::vector<int> XArrayShapeImpl1D(int dim0, int dim1) {
  return {dim0};
}

std::vector<int> XArrayShapeImpl2D(int dim0, int dim1) {
  return {dim0, dim1};
}

constexpr auto XArrayShapeImpl(const std::string_view dim1_str) {
  if (dim1_str == "1") {
    return XArrayShapeImpl1D;
  } else {
    return XArrayShapeImpl2D;
  }
}

// An equals operator that works for comparing numpy arrays too.
// Unlike other objects, the equality operator for numpy arrays returns a
// numpy array.
bool FieldsEqual(py::handle lhs, py::handle rhs, py::handle array_equal) {
  // np.array_equal handles non-arrays.
  return PyObject_IsTrue(array_equal(lhs, rhs).ptr());
}

// Returns an iterable object for iterating over attributes of T.
template <typename T>
py::object Dir() {
  py::object type = py::type::of<T>();
  auto dir = py::reinterpret_steal<py::object>(PyObject_Dir(type.ptr()));
  if (PyErr_Occurred()) {
    throw py::error_already_set();
  }
  return dir;
}

// Returns true if all public fields in lhs and rhs are equal.
template <typename T>
bool StructsEqual(py::object lhs, py::object rhs) {
  // Equivalent to the following python code:
  // if type(lhs) != type(rhs):
  //   return False
  // for field in dir(lhs):
  //   if field.startswith("_"):
  //     continue
  //   # equal() handles equality of numpy arrays
  //   if not equal(getattr(lhs, field, None), getattr(rhs, field, None)):
  //     return False
  //
  // return True

  auto np = py::module::import("numpy");
  auto array_equal = np.attr("array_equal");

  const py::handle lhs_t = py::type::handle_of(lhs);
  const py::handle rhs_t = py::type::handle_of(rhs);
  if (!lhs_t.is(rhs_t)) {
    return false;
  }
  for (py::handle f : Dir<T>()) {
    auto name = f.cast<std::string_view>();

    if (name.empty() || name[0] == '_') {
      continue;
    }
    py::object l = py::getattr(lhs, f, py::none());
    py::object r = py::getattr(rhs, f, py::none());
    if (!FieldsEqual(l, r, array_equal)) {
      return false;
    }
  }
  return true;
}

// Returns a string representation of a struct like object.
template <typename T>
std::string StructRepr(py::object self) {
  std::ostringstream result;
  result << "<"
         << self.attr("__class__").attr("__name__").cast<std::string_view>();
  for (py::handle f : Dir<T>()) {
    auto name = f.cast<std::string_view>();
    if (name.empty() || name[0] == '_') {
      continue;
    }

    result << "\n  " << name << ": "
           << self.attr(f).attr("__repr__")().cast<std::string_view>();
  }
  result << "\n>";
  return result.str();
}

template <typename C, typename... O>
void DefineStructFunctions(py::class_<C, O...> c) {
  c.def("__eq__", StructsEqual<C>);
  c.def("__repr__", StructRepr<C>);
}
}  // namespace

// ==================== MJOPTION ===============================================
#define X(var, dim) , var(InitPyArray(std::array{dim}, ptr_->var, owner_))
MjOptionWrapper::MjWrapper()
    : WrapperBase([]() {
        raw::MjOption* const opt = new raw::MjOption;
        mj_defaultOption(opt);
        return opt;
      }())
      MJOPTION_VECTORS {}

MjOptionWrapper::MjWrapper(raw::MjOption* ptr, py::handle owner)
    : WrapperBase(ptr, owner)
      MJOPTION_VECTORS {}
#undef X

MjOptionWrapper::MjWrapper(const MjOptionWrapper& other)
    : MjOptionWrapper() {
  *this->ptr_ = *other.ptr_;
}

// ==================== MJVISUAL ===============================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjVisualHeadlightWrapper::MjWrapper()
    : WrapperBase(new raw::MjVisualHeadlight{}),
      X(ambient),
      X(diffuse),
      X(specular) {}

MjVisualHeadlightWrapper::MjWrapper(
    raw::MjVisualHeadlight* ptr, py::handle owner)
    : WrapperBase(ptr, owner),
      X(ambient),
      X(diffuse),
      X(specular) {}
#undef X

MjVisualHeadlightWrapper::MjWrapper(const MjVisualHeadlightWrapper& other)
    : MjVisualHeadlightWrapper() {
  *this->ptr_ = *other.ptr_;
}

#define X(var) var(InitPyArray(ptr_->var, owner_))
MjVisualRgbaWrapper::MjWrapper()
    : WrapperBase(new raw::MjVisualRgba{}),
      X(fog),
      X(haze),
      X(force),
      X(inertia),
      X(joint),
      X(actuator),
      X(actuatornegative),
      X(actuatorpositive),
      X(com),
      X(camera),
      X(light),
      X(selectpoint),
      X(connect),
      X(contactpoint),
      X(contactforce),
      X(contactfriction),
      X(contacttorque),
      X(contactgap),
      X(rangefinder),
      X(constraint),
      X(slidercrank),
      X(crankbroken) {}

MjVisualRgbaWrapper::MjWrapper(raw::MjVisualRgba* ptr, py::handle owner)
    : WrapperBase(ptr, owner),
      X(fog),
      X(haze),
      X(force),
      X(inertia),
      X(joint),
      X(actuator),
      X(actuatornegative),
      X(actuatorpositive),
      X(com),
      X(camera),
      X(light),
      X(selectpoint),
      X(connect),
      X(contactpoint),
      X(contactforce),
      X(contactfriction),
      X(contacttorque),
      X(contactgap),
      X(rangefinder),
      X(constraint),
      X(slidercrank),
      X(crankbroken) {}
#undef X

MjVisualRgbaWrapper::MjWrapper(const MjVisualRgbaWrapper& other)
    : MjVisualRgbaWrapper() {
  *this->ptr_ = *other.ptr_;
}

MjVisualWrapper::MjWrapper()
    : WrapperBase(new raw::MjVisual{}),
      headlight(&ptr_->headlight, owner_),
      rgba(&ptr_->rgba, owner_) {}

MjVisualWrapper::MjWrapper(raw::MjVisual* ptr, py::handle owner)
    : WrapperBase(ptr, owner),
      headlight(&ptr_->headlight, owner_),
      rgba(&ptr_->rgba, owner_) {}


MjVisualWrapper::MjWrapper(const MjVisualWrapper& other)
    : MjVisualWrapper() {
  *this->ptr_ = *other.ptr_;
}

// ==================== MJMODEL ================================================
static void MjModelCapsuleDestructor(PyObject* pyobj) {
  mj_deleteModel(
      static_cast<raw::MjModel*>(PyCapsule_GetPointer(pyobj, nullptr)));
}

static absl::flat_hash_map<raw::MjModel*, MjModelWrapper*>&
MjModelRawPointerMap() {
  static auto* hash_map =
      new absl::flat_hash_map<raw::MjModel*, MjModelWrapper*>();
  return *hash_map;
}

MjModelWrapper* MjModelWrapper::FromRawPointer(raw::MjModel* m) noexcept {
  try {
    auto& map = MjModelRawPointerMap();
    {
      py::gil_scoped_acquire gil;
      auto found = map.find(m);
      return found != map.end() ? found->second : nullptr;
    }
  } catch (...) {
    return nullptr;
  }
}

#undef MJ_M
#define MJ_M(x) ptr_->x
#define X(dtype, var, dim0, dim1) \
    , var (InitPyArray(X_ARRAY_SHAPE(ptr_->dim0, dim1), ptr_->var, owner_))
MjModelWrapper::MjWrapper(raw::MjModel* ptr)
    : WrapperBase(ptr, &MjModelCapsuleDestructor),
      opt(&ptr->opt, owner_),
      vis(&ptr->vis, owner_),
      stat(&ptr->stat, owner_)
      MJMODEL_POINTERS,
      text_data_bytes(ptr->text_data, ptr->ntextdata),
      names_bytes(ptr->names, ptr->nnames),
      indexer_(ptr, owner_) {
  bool is_newly_inserted = false;
  {
    py::gil_scoped_acquire gil;
    is_newly_inserted = MjModelRawPointerMap().insert({ptr_, this}).second;
  }
  if (!is_newly_inserted) {
    throw UnexpectedError(
        "MjModelWrapper(mjModel*): MjModelRawPointerMap already contains this "
        "raw mjModel*");
  }
}

MjModelWrapper::MjWrapper(MjModelWrapper&& other)
    : WrapperBase(other.ptr_, other.owner_),
      opt(&ptr_->opt, owner_),
      vis(&ptr_->vis, owner_),
      stat(&ptr_->stat, owner_)
      MJMODEL_POINTERS,
      text_data_bytes(ptr_->text_data, ptr_->ntextdata),
      names_bytes(ptr_->names, ptr_->nnames),
      indexer_(ptr_, owner_) {
  bool is_newly_inserted = false;
  {
    py::gil_scoped_acquire gil;
    is_newly_inserted =
        MjModelRawPointerMap().insert_or_assign(ptr_, this).second;
  }
  if (is_newly_inserted) {
    throw UnexpectedError(
        "MjModelRawPointerMap does not contains the moved-from mjModel*");
  }
  other.ptr_ = nullptr;
}
#undef X
#undef MJ_M
#define MJ_M(x) x

// Delegating to the MjModelWrapper::MjWrapper(raw::MjModel*) constructor,
// no need to modify MjModelRawPointerMap here.
MjModelWrapper::MjWrapper(const MjModelWrapper& other)
    : MjModelWrapper(InterceptMjErrors(mj_copyModel)(NULL, other.get())) {}

MjModelWrapper::~MjWrapper() {
  if (ptr_) {
    bool erased = false;
    {
      py::gil_scoped_acquire gil;
      erased = MjModelRawPointerMap().erase(ptr_);
    }
    if (!erased) {
      std::cerr << "MjModelRawPointerMap does not contain this raw mjModel*"
                << std::endl;
      std::terminate();
    }
  }
}

namespace {
struct VfsAsset {
  VfsAsset(const char* name, const void* content, std::size_t content_size)
      : name(name), content(content), content_size(content_size) {}
  const char* name;
  const void* content;
  std::size_t content_size;
};
}

// Helper function for both LoadXMLFile and LoadBinaryFile.
// Creates a temporary MJB from the assets dictionary if one is supplied.
template <typename LoadFunc>
static raw::MjModel* LoadModelFileImpl(
    const std::string& filename,
    const std::vector<VfsAsset>& assets,
    LoadFunc&& loadfunc) {
  std::unique_ptr<mjVFS, void(*)(mjVFS*)> vfs(nullptr, [](mjVFS*){});
  if (!assets.empty()) {
    // mjVFS should be allocated on the heap, because it's ~2MB
    vfs = decltype(vfs)(new mjVFS, [](mjVFS* vfs) {
      mj_deleteVFS(vfs);
      delete vfs;
    });
    mj_defaultVFS(vfs.get());
    for (const auto& asset : assets) {
      const int vfs_error = InterceptMjErrors(mj_makeEmptyFileVFS)(
          vfs.get(), asset.name, asset.content_size);
      if (vfs_error) {
        throw py::value_error("assets dict is too big");
      }
      std::memcpy(vfs->filedata[vfs->nfile - 1],
                  asset.content, asset.content_size);
    }
  }

  raw::MjModel* model = loadfunc(filename.c_str(), vfs.get());
  if (model && !model->buffer) {
    mj_deleteModel(model);
    model = nullptr;
  }
  return model;
}

// Converts a dict with py::bytes value to a vector of standard C++ types.
// This allows us to release the GIL early. Note that the vector consists only
// of pointers to existing data so no substantial data copies are being made.
static std::vector<VfsAsset>
ConvertAssetsDict(
    const std::optional<std::unordered_map<std::string, py::bytes>>& assets) {
  std::vector<VfsAsset> out;
  if (assets.has_value()) {
    for (const auto& [name, content] : *assets) {
      if (name.length() >= mjMAXVFSNAME) {
        std::ostringstream error;
        error << "Filename length " << name.length() << " exceeds "
              << mjMAXVFSNAME - 1 << " character limit: " << name;
        throw py::value_error(error.str());
      }
      out.emplace_back(name.c_str(), PYBIND11_BYTES_AS_STRING(content.ptr()),
                       py::len(content));
    }
  }
  return out;
}

MjModelWrapper MjModelWrapper::LoadXMLFile(
    const std::string& filename,
    const std::optional<std::unordered_map<std::string, py::bytes>>& assets) {
  const auto converted_assets = ConvertAssetsDict(assets);
  raw::MjModel* model;
  {
    py::gil_scoped_release no_gil;
    char error[1024];
    model = LoadModelFileImpl(
        filename, converted_assets,
        [&error](const char* filename, const mjVFS* vfs) {
          return InterceptMjErrors(mj_loadXML)(
              filename, vfs, error, sizeof(error));
        });
    if (!model) {
      throw py::value_error(error);
    }
  }
  return MjModelWrapper(model);
}

MjModelWrapper MjModelWrapper::LoadBinaryFile(
    const std::string& filename,
    const std::optional<std::unordered_map<std::string, py::bytes>>& assets) {
  const auto converted_assets = ConvertAssetsDict(assets);
  raw::MjModel* model;
  {
    py::gil_scoped_release no_gil;
    model = LoadModelFileImpl(
        filename, converted_assets, InterceptMjErrors(mj_loadModel));
    if (!model) {
      throw py::value_error("mj_loadModel: failed to load from mjb");
    }
  }
  return MjModelWrapper(model);
}

MjModelWrapper MjModelWrapper::LoadXML(
    const std::string& xml,
    const std::optional<std::unordered_map<std::string, py::bytes>>& assets) {
  auto converted_assets = ConvertAssetsDict(assets);
  raw::MjModel* model;
  {
    py::gil_scoped_release no_gil;
    std::string model_filename = "model_.xml";
    if (assets.has_value()) {
      while (assets->find(model_filename) != assets->end()) {
        model_filename =
            model_filename.substr(0, model_filename.size() - 4) + "_.xml";
      }
    }
    converted_assets.emplace_back(
        model_filename.c_str(), xml.c_str(), xml.length());
    char error[1024];
    model = LoadModelFileImpl(
        model_filename, converted_assets,
        [&error](const char* filename, const mjVFS* vfs) {
          return InterceptMjErrors(mj_loadXML)(
              filename, vfs, error, sizeof(error));
        });
    if (!model) {
      throw py::value_error(error);
    }
  }
  return MjModelWrapper(model);
}

namespace {
// A byte at the start of serialized mjModel structs, which can be incremented
//  when we change the serialization logic to reject pickles from an
// unsupported future version.
constexpr static char kSerializationVersion = 1;

void CheckInput(const std::istream& input, std::string class_name) {
  if (input.fail()) {
    throw py::value_error("Invalid serialized " + class_name + ".");
  }
}

}  // namespace

void MjModelWrapper::Serialize(std::ostream& output) const {
  WriteChar(output, kSerializationVersion);

  int model_size = mj_sizeModel(get());
  WriteInt(output, model_size);
  std::string buffer(model_size, 0);
  mj_saveModel(get(), nullptr, buffer.data(), model_size);
  WriteBytes(output, buffer.data(), model_size);
}

MjModelWrapper MjModelWrapper::Deserialize(std::istream& input) {
  CheckInput(input, "mjModel");

  char serializationVersion = ReadChar(input);
  CheckInput(input, "mjModel");

  if (serializationVersion != kSerializationVersion) {
    throw py::value_error("Incompatible serialization version.");
  }

  int model_size = ReadInt(input);
  CheckInput(input, "mjModel");
  if (model_size < 0) {
    throw py::value_error("Invalid serialized mjModel.");
  }
  std::string model_bytes(model_size, 0);
  ReadBytes(input, model_bytes.data(), model_size);
  CheckInput(input, "mjModel");

  raw::MjModel* model = LoadModelFileImpl(
      "model.mjb",
      {{"model.mjb", model_bytes.data(), static_cast<std::size_t>(model_size)}},
      InterceptMjErrors(mj_loadModel));
  if (!model) {
    throw py::value_error("Invalid serialized mjModel.");
  }
  return MjModelWrapper(model);
}

// ==================== MJCONTACT ==============================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjContactWrapper::MjWrapper()
    : WrapperBase(new raw::MjContact{}),
      X(pos),
      X(frame),
      X(friction),
      X(solref),
      X(solimp),
      X(H) {}

MjContactWrapper::MjWrapper(raw::MjContact* ptr, py::handle owner)
    : WrapperBase(ptr, owner),
      X(pos),
      X(frame),
      X(friction),
      X(solref),
      X(solimp),
      X(H) {}
#undef X

MjContactWrapper::MjWrapper(const MjContactWrapper& other)
    : MjContactWrapper() {
  *this->ptr_ = *other.ptr_;
}

#define X(type, var)                                                   \
  var(std::vector<int>{num}, std::vector<int>{sizeof(raw::MjContact)}, \
      &ptr->var, owner)
#define XN(type, var)                                                       \
  var(std::vector<int>{num, sizeof(raw::MjContact::var) / sizeof(type)},    \
      std::vector<int>{sizeof(raw::MjContact), sizeof(type)}, &ptr->var[0], \
      owner)
MjContactList::MjStructList(raw::MjContact* ptr, int num, py::handle owner)
    : StructListBase(ptr, num, owner),
      X(mjtNum, dist),
      XN(mjtNum, pos),
      XN(mjtNum, frame),
      X(mjtNum, includemargin),
      XN(mjtNum, friction),
      XN(mjtNum, solref),
      XN(mjtNum, solimp),
      X(mjtNum, mu),
      XN(mjtNum, H),
      X(int, dim),
      X(int, geom1),
      X(int, geom2),
      X(int, exclude),
      X(int, efc_address) {}
#undef X
#undef XN

// Slicing
#define X(type, var) var(other.var[slice])
MjContactList::MjStructList(MjContactList& other, py::slice slice)
    : StructListBase(other, slice),
      X(mjtNum, dist),
      X(mjtNum, pos),
      X(mjtNum, frame),
      X(mjtNum, includemargin),
      X(mjtNum, friction),
      X(mjtNum, solref),
      X(mjtNum, solimp),
      X(mjtNum, mu),
      X(mjtNum, H),
      X(int, dim),
      X(int, geom1),
      X(int, geom2),
      X(int, exclude),
      X(int, efc_address) {}
#undef X

// ==================== MJDATA =================================================
static void MjDataCapsuleDestructor(PyObject* pyobj) {
  mj_deleteData(
      static_cast<raw::MjData*>(PyCapsule_GetPointer(pyobj, nullptr)));
}

absl::flat_hash_map<raw::MjData*, MjDataWrapper*>&
MjDataRawPointerMap() {
  static auto* hash_map =
      new absl::flat_hash_map<raw::MjData*, MjDataWrapper*>();
  return *hash_map;
}

MjDataWrapper* MjDataWrapper::FromRawPointer(raw::MjData* m) noexcept {
  try {
    auto& map = MjDataRawPointerMap();
    {
      py::gil_scoped_acquire gil;
      auto found = map.find(m);
      return found != map.end() ? found->second : nullptr;
    }
  } catch (...) {
    return nullptr;
  }
}

MjDataWrapper::MjWrapper(const MjModelWrapper& model)
    : WrapperBase(InterceptMjErrors(mj_makeData)(model.get()),
                  &MjDataCapsuleDestructor),
#undef MJ_M
#define MJ_M(x) model.get()->x
#define X(dtype, var, dim0, dim1) \
  var(InitPyArray(X_ARRAY_SHAPE(model.get()->dim0, dim1), ptr_->var, owner_)),
      MJDATA_POINTERS
#undef MJ_M
#define MJ_M(x) x
#undef X

#define X(dtype, var, dim0, dim1) var(InitPyArray(ptr_->var, owner_)),
      MJDATA_VECTOR
#undef X
      metadata_(model.get()),
      indexer_(ptr_, &metadata_, owner_) {
  bool is_newly_inserted = false;
  {
    py::gil_scoped_acquire gil;
    is_newly_inserted = MjDataRawPointerMap().insert({ptr_, this}).second;
  }
  if (!is_newly_inserted) {
    throw UnexpectedError(
        "MjDataRawPointerMap already contains this raw mjData*");
  }
}

MjDataWrapper::MjWrapper(const MjDataWrapper& other)
    : WrapperBase(other.Copy(), &MjDataCapsuleDestructor),
#undef MJ_M
#define MJ_M(x) other.metadata_.x
#define X(dtype, var, dim0, dim1)                                       \
  var(InitPyArray(X_ARRAY_SHAPE(other.metadata_.dim0, dim1), ptr_->var, \
                  owner_)),
      MJDATA_POINTERS
#undef MJ_M
#define MJ_M(x) x
#undef X

#define X(dtype, var, dim0, dim1) var(InitPyArray(ptr_->var, owner_)),
      MJDATA_VECTOR
#undef X
      metadata_(other.metadata_),
      indexer_(ptr_, &metadata_, owner_) {
  bool is_newly_inserted = false;
  {
    py::gil_scoped_acquire gil;
    is_newly_inserted = MjDataRawPointerMap().insert({ptr_, this}).second;
  }
  if (!is_newly_inserted) {
    throw UnexpectedError(
        "MjDataRawPointerMap already contains this raw mjData*");
  }
}

MjDataWrapper::MjWrapper(MjDataWrapper&& other)
    : WrapperBase(other.ptr_, other.owner_),
#undef MJ_M
#define MJ_M(x) other.metadata_.x
#define X(dtype, var, dim0, dim1)                                       \
  var(InitPyArray(X_ARRAY_SHAPE(other.metadata_.dim0, dim1), ptr_->var, \
                  owner_)),
      MJDATA_POINTERS
#undef MJ_M
#define MJ_M(x) x
#undef X

#define X(dtype, var, dim0, dim1) var(InitPyArray(ptr_->var, owner_)),
      MJDATA_VECTOR
#undef X
      metadata_(other.metadata_),
      indexer_(ptr_, &metadata_, owner_) {
  bool is_newly_inserted = false;
  {
    py::gil_scoped_acquire gil;
    is_newly_inserted =
        MjDataRawPointerMap().insert_or_assign(ptr_, this).second;
  }
  if (is_newly_inserted) {
    throw UnexpectedError(
        "MjDataRawPointerMap does not contains the moved-from mjData*");
  }
  other.ptr_ = nullptr;
}

MjDataWrapper::MjWrapper(MjDataMetadata&& metadata, raw::MjData* d)
    : WrapperBase(d, &MjDataCapsuleDestructor),
#undef MJ_M
#define MJ_M(x) metadata.x
#define X(dtype, var, dim0, dim1) \
  var(InitPyArray(X_ARRAY_SHAPE(metadata.dim0, dim1), ptr_->var, owner_)),
      MJDATA_POINTERS
#undef MJ_M
#define MJ_M(x) x
#undef X

#define X(dtype, var, dim0, dim1) var(InitPyArray(ptr_->var, owner_)),
      MJDATA_VECTOR
#undef X
      metadata_(std::move(metadata)),
      indexer_(ptr_, &metadata_, owner_) {
  bool is_newly_inserted = false;
  {
    py::gil_scoped_acquire gil;
    is_newly_inserted = MjDataRawPointerMap().insert({ptr_, this}).second;
  }
  if (!is_newly_inserted) {
    throw UnexpectedError(
        "MjDataRawPointerMap already contains this raw mjData*");
  }
}

MjDataWrapper::~MjWrapper() {
  if (ptr_) {
    bool erased = false;
    {
      py::gil_scoped_acquire gil;
      erased = MjDataRawPointerMap().erase(ptr_);
    }
    if (!erased) {
      std::cerr << "MjDataRawPointerMap does not contain this raw mjData*"
                << std::endl;
      std::terminate();
    }
  }
}

void MjDataWrapper::Serialize(std::ostream& output) const {
  // TODO: Replace this custom serialization with a protobuf
  WriteChar(output, kSerializationVersion);

  // Write all size fields
#define X(var) WriteInt(output, this->metadata_.var);
  MJMODEL_INTS
#undef X

#define X(dtype, var, n)                        \
  WriteBytes(output, this->metadata_.var.get(), \
             this->metadata_.n * sizeof(dtype));

  MJDATA_METADATA
#undef X

  // Write struct and scalar fields
#define X(var) WriteBytes(output, &ptr_->var, sizeof(ptr_->var))
  X(solver);
  X(timer);
  X(warning);
  X(ncon);
  X(time);
  X(energy);
#undef X

  // Write buffer contents
  WriteBytes(output, ptr_->buffer, ptr_->nbuffer);
}

MjDataWrapper MjDataWrapper::Deserialize(std::istream& input) {
  char serializationVersion = ReadChar(input);
  CheckInput(input, "mjData");

  if (serializationVersion != kSerializationVersion) {
    throw py::value_error("Incompatible serialization version.");
  }

  // Read all size and address fields
  MjDataMetadata metadata;
  raw::MjModel m{0};

#define X(var)                   \
  metadata.var = ReadInt(input); \
  CheckInput(input, "mjData");   \
  m.var = metadata.var;

  MJMODEL_INTS
#undef X

#define X(dtype, var, n)                                            \
  metadata.var.reset(new dtype[metadata.n]);                        \
  ReadBytes(input, metadata.var.get(), metadata.n * sizeof(dtype)); \
  CheckInput(input, "mjData");                                      \
  m.var = metadata.var.get();

  MJDATA_METADATA
#undef X

  raw::MjData* d = mj_makeData(&m);
  if (!d) {
    throw py::value_error("Failed to create mjData.");
  }

  // Read structs and scalar fields
#define X(var)                                       \
  ReadBytes(input, (void*) &d->var, sizeof(d->var)); \
  CheckInput(input, "mjData");

  X(solver);
  X(timer);
  X(warning);
  X(ncon);
  X(time);
  X(energy);
#undef X

  // Read buffer contents
  ReadBytes(input, d->buffer, d->nbuffer);
  CheckInput(input, "mjData");

  // All bytes should have been used.
  input.ignore(1);
  if (!input.eof()) {
    throw py::value_error("Invalid serialized mjData.");
  }

  return MjDataWrapper(std::move(metadata), d);
}

raw::MjData* MjDataWrapper::Copy() const {
  raw::MjModel m{0};
#define X(var) m.var = this->metadata_.var;
  MJMODEL_INTS
#undef X
#define X(dtype, var, n) m.var = this->metadata_.var.get();
  MJDATA_METADATA
#undef X
  return InterceptMjErrors(mj_copyData)(NULL, &m, this->ptr_);
}

// ==================== MJSTATISTIC ============================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjStatisticWrapper::MjWrapper()
    : WrapperBase(new raw::MjStatistic{}),
      X(center) {}

MjStatisticWrapper::MjWrapper(raw::MjStatistic* ptr, py::handle owner)
    : WrapperBase(ptr, owner),
      X(center) {}
#undef X

MjStatisticWrapper::MjWrapper(const MjStatisticWrapper& other)
    : MjStatisticWrapper() {
  *this->ptr_ = *other.ptr_;
}

// ==================== MJWARNINGSTAT ==========================================
MjWarningStatWrapper::MjWrapper()
    : WrapperBase(new raw::MjWarningStat{}) {}

MjWarningStatWrapper::MjWrapper(raw::MjWarningStat* ptr, py::handle owner)
    : WrapperBase(ptr, owner) {}

MjWarningStatWrapper::MjWrapper(const MjWarningStatWrapper& other)
    : MjWarningStatWrapper() {
  *this->ptr_ = *other.ptr_;
}

#define X(type, var)                                                       \
  var(std::vector<int>{num}, std::vector<int>{sizeof(raw::MjWarningStat)}, \
      &ptr->var, owner)
MjWarningStatList::MjStructList(raw::MjWarningStat* ptr, int num,
                                py::handle owner)
    : StructListBase(ptr, num, owner), X(int, lastinfo), X(int, number) {}
#undef X

// Slicing
#define X(type, var) var(other.var[slice])
MjWarningStatList::MjStructList(MjWarningStatList& other, py::slice slice)
    : StructListBase(other, slice),
      X(int, lastinfo),
      X(int, number) {}
#undef X

// ==================== MJTIMERSTAT ============================================
MjTimerStatWrapper::MjWrapper()
    : WrapperBase(new raw::MjTimerStat{}) {}

MjTimerStatWrapper::MjWrapper(raw::MjTimerStat* ptr, py::handle owner)
    : WrapperBase(ptr, owner) {}

MjTimerStatWrapper::MjWrapper(const MjTimerStatWrapper& other)
    : MjTimerStatWrapper() {
  *this->ptr_ = *other.ptr_;
}

#define X(type, var)                                                     \
  var(std::vector<int>{num}, std::vector<int>{sizeof(raw::MjTimerStat)}, \
      &ptr->var, owner)
MjTimerStatList::MjStructList(raw::MjTimerStat* ptr, int num, py::handle owner)
    : StructListBase(ptr, num, owner),
      X(mjtNum, duration),
      X(int, number) {}
#undef X

// Slicing
#define X(type, var) var(other.var[slice])
MjTimerStatList::MjStructList(MjTimerStatList& other, py::slice slice)
    : StructListBase(other, slice),
      X(mjtNum, duration),
      X(int, number) {}
#undef X

// ==================== MJSOLVERSTAT ===========================================
MjSolverStatWrapper::MjWrapper()
    : WrapperBase(new raw::MjSolverStat{}) {}

MjSolverStatWrapper::MjWrapper(raw::MjSolverStat* ptr, py::handle owner)
    : WrapperBase(ptr, owner) {}

MjSolverStatWrapper::MjWrapper(const MjSolverStatWrapper& other)
    : MjSolverStatWrapper() {
  *this->ptr_ = *other.ptr_;
}

#define X(type, var)                                                      \
  var(std::vector<int>{num}, std::vector<int>{sizeof(raw::MjSolverStat)}, \
      &ptr->var, owner)
MjSolverStatList::MjStructList(raw::MjSolverStat* ptr, int num,
                               py::handle owner)
    : StructListBase(ptr, num, owner),
      X(mjtNum, improvement),
      X(mjtNum, gradient),
      X(mjtNum, lineslope),
      X(int, nactive),
      X(int, nchange),
      X(int, neval),
      X(int, nupdate) {}
#undef X
#undef XN

// Slicing
#define X(type, var) var(other.var[slice])
MjSolverStatList::MjStructList(MjSolverStatList& other, py::slice slice)
    : StructListBase(other, slice),
      X(mjtNum, improvement),
      X(mjtNum, gradient),
      X(mjtNum, lineslope),
      X(int, nactive),
      X(int, nchange),
      X(int, neval),
      X(int, nupdate) {}
#undef X

// ==================== MJVPERTURB =============================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjvPerturbWrapper::MjWrapper()
    : WrapperBase([]() {
        raw::MjvPerturb* const pert = new raw::MjvPerturb;
        mjv_defaultPerturb(pert);
        return pert;
      }()),
      X(refpos),
      X(refquat),
      X(localpos) {}
#undef X

MjvPerturbWrapper::MjWrapper(const MjvPerturbWrapper& other)
    : MjvPerturbWrapper() {
  *this->ptr_ = *other.ptr_;
}

// ==================== MJVCAMERA ==============================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjvCameraWrapper::MjWrapper()
    : WrapperBase([]() {
        raw::MjvCamera* const cam = new raw::MjvCamera;
        mjv_defaultCamera(cam);
        return cam;
      }()),
      X(lookat) {}
#undef X

MjvCameraWrapper::MjWrapper(const MjvCameraWrapper& other)
    : MjvCameraWrapper() {
  *this->ptr_ = *other.ptr_;
}

// ==================== MJVGLCAMERA ============================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjvGLCameraWrapper::MjWrapper()
    : WrapperBase(new raw::MjvGLCamera{}),
      X(pos),
      X(forward),
      X(up) {}

MjvGLCameraWrapper::MjWrapper(raw::MjvGLCamera* ptr, py::handle owner)
    : WrapperBase(ptr, owner),
      X(pos),
      X(forward),
      X(up) {}
#undef X

MjvGLCameraWrapper::MjWrapper(raw::MjvGLCamera&& other)
    : MjvGLCameraWrapper() {
  *this->ptr_ = other;
}

MjvGLCameraWrapper::MjWrapper(const MjvGLCameraWrapper& other)
    : MjvGLCameraWrapper() {
  *this->ptr_ = *other.ptr_;
}

// ==================== MJVGEOM ================================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjvGeomWrapper::MjWrapper()
    : WrapperBase(new raw::MjvGeom{}),
      X(texrepeat),
      X(size),
      X(pos),
      mat([this]() {
        static_assert(sizeof(ptr_->mat) == sizeof(ptr_->mat[0])*9);
        return InitPyArray(std::array{3, 3}, ptr_->mat, owner_);
      }()),
      X(rgba) {}

MjvGeomWrapper::MjWrapper(raw::MjvGeom* ptr, py::handle owner)
    : WrapperBase(ptr, owner),
      X(texrepeat),
      X(size),
      X(pos),
      mat([this]() {
        static_assert(sizeof(ptr_->mat) == sizeof(ptr_->mat[0])*9);
        return InitPyArray(std::array{3, 3}, ptr_->mat, owner_);
      }()),
      X(rgba) {}
#undef X

MjvGeomWrapper::MjWrapper(const MjvGeomWrapper& other)
    : MjvGeomWrapper() {
  *this->ptr_ = *other.ptr_;
}

// ==================== MJVLIGHT ===============================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjvLightWrapper::MjWrapper()
    : WrapperBase(new raw::MjvLight{}),
      X(pos),
      X(dir),
      X(attenuation),
      X(ambient),
      X(diffuse),
      X(specular) {}

MjvLightWrapper::MjWrapper(raw::MjvLight* ptr, py::handle owner)
    : WrapperBase(ptr, owner),
      X(pos),
      X(dir),
      X(attenuation),
      X(ambient),
      X(diffuse),
      X(specular) {}
#undef X

MjvLightWrapper::MjWrapper(const MjvLightWrapper& other)
    : MjvLightWrapper() {
  *this->ptr_ = *other.ptr_;
}

// ==================== MJVOPTION ==============================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjvOptionWrapper::MjWrapper()
    : WrapperBase([]() {
        raw::MjvOption* const opt = new raw::MjvOption;
        mjv_defaultOption(opt);
        return opt;
      }()),
      X(geomgroup),
      X(sitegroup),
      X(jointgroup),
      X(tendongroup),
      X(actuatorgroup),
      X(flags) {}
#undef X

MjvOptionWrapper::MjWrapper(const MjvOptionWrapper& other)
    : MjvOptionWrapper() {
  *this->ptr_ = *other.ptr_;
}

// ==================== MJVSCENE ===============================================
static void MjvSceneCapsuleDestructor(PyObject* pyobj) {
  py::gil_scoped_acquire gil;
  auto* scn = static_cast<raw::MjvScene*>(PyCapsule_GetPointer(pyobj, nullptr));
  if (scn) {
    mjv_freeScene(scn);
    delete scn;
  }
}

#define X(var) var(InitPyArray(ptr_->var, owner_))
#define XN(var, n) var(InitPyArray(std::array{n}, ptr_->var, owner_))
MjvSceneWrapper::MjWrapper()
    : WrapperBase([]() {
        raw::MjvScene *const scn = new raw::MjvScene;
        mjv_defaultScene(scn);
        InterceptMjErrors(mjv_makeScene)(nullptr, scn, 0);
        return scn;
      }(), &MjvSceneCapsuleDestructor),
      nskinvert(0),
      XN(geoms, 0),
      XN(geomorder, 0),
      XN(skinfacenum, 0),
      XN(skinvertadr, 0),
      XN(skinvertnum, 0),
      XN(skinvert, 0),
      XN(skinnormal, 0),
      X(lights),
      X(camera),
      X(translate),
      X(rotate),
      X(flags),
      X(framergb) {}

#define XN(var, n) var(InitPyArray(std::array{n}, ptr_->var, owner_))
MjvSceneWrapper::MjWrapper(const MjModelWrapper& model, int maxgeom)
    : WrapperBase(
        [maxgeom](const raw::MjModel* m) {
          raw::MjvScene *const scn = new raw::MjvScene;
          mjv_defaultScene(scn);
          InterceptMjErrors(mjv_makeScene)(m, scn, maxgeom);
          return scn;
        }(model.get()),
        &MjvSceneCapsuleDestructor),
      nskinvert([](const raw::MjModel* m) {
        int nskinvert = 0;
        for (int i = 0; i < m->nskin; ++i) {
          nskinvert += m->skin_vertnum[i];
        }
        return nskinvert;
      }(model.get())),
      XN(geoms, ptr_->maxgeom),
      XN(geomorder, ptr_->maxgeom),
      XN(skinfacenum, ptr_->nskin),
      XN(skinvertadr, ptr_->nskin),
      XN(skinvertnum, ptr_->nskin),
      XN(skinvert, 3*nskinvert),
      XN(skinnormal, 3*nskinvert),
      X(lights),
      X(camera),
      X(translate),
      X(rotate),
      X(flags),
      X(framergb) {}
#undef X
#undef XN

template <typename T>
static T* MallocAndCopy(const T* src, int count) {
  if (src) {
    T* out = static_cast<T*>(mju_malloc(count * sizeof(T)));
    std::memcpy(out, src, count * sizeof(T));
    return out;
  } else {
    return nullptr;
  }
}

MjvSceneWrapper::MjWrapper(const MjvSceneWrapper& other)
    : MjvSceneWrapper() {
  mjv_freeScene(ptr_);
  *ptr_ = *other.ptr_;

#define XN(var, n)                                     \
  ptr_->var = MallocAndCopy(other.ptr_->var, n);       \
  var = InitPyArray(std::array{n}, ptr_->var, owner_);

  XN(geoms, ptr_->ngeom);
  XN(geomorder, ptr_->ngeom);
  XN(skinfacenum, ptr_->nskin);
  XN(skinvertadr, ptr_->nskin);
  XN(skinvertnum, ptr_->nskin);
  XN(skinvert, 3*nskinvert);
  XN(skinnormal, 3*nskinvert);

#undef XN
}

// ==================== MJVFIGURE ==============================================
#define X(var) var(InitPyArray(ptr_->var, owner_))
MjvFigureWrapper::MjWrapper()
    : WrapperBase([]() {
        raw::MjvFigure* const fig = new raw::MjvFigure;
        mjv_defaultFigure(fig);
        return fig;
      }()),
      X(flg_ticklabel),
      X(gridsize),
      X(gridrgb),
      X(figurergba),
      X(panergba),
      X(legendrgba),
      X(textrgb),
      X(linergb),
      X(range),
      X(highlight),
      X(linepnt),
      X(linedata),
      X(xaxispixel),
      X(yaxispixel),
      X(xaxisdata),
      X(yaxisdata),
#undef X

      linename([](raw::MjvFigure* ptr, py::handle owner) {
// Use a macro to help us static_assert that the array extents here are kept
// in sync with mjVisualize.h.
#define MAKE_STR_ARRAY(N1, N2)                                           \
  static_assert(                                                         \
      std::is_same_v<decltype(raw::MjvFigure::linename), char[N1][N2]>); \
  return py::array(py::dtype("|S" #N2), N1, ptr->linename, owner);

      MAKE_STR_ARRAY(mjMAXLINE, 100);

#undef MAKE_STR_ARRAY
      }(ptr_, owner_)) {}

MjvFigureWrapper::MjWrapper(const MjvFigureWrapper& other)
    : MjvFigureWrapper() {
  *this->ptr_ = *other.ptr_;
}

PYBIND11_MODULE(_structs, m) {
  py::module_::import("mujoco._enums");

  // ==================== MJOPTION =============================================
  py::class_<MjOptionWrapper> mjOption(m, "MjOption");
  mjOption.def(py::init<>());
  mjOption.def("__copy__", [](const MjOptionWrapper& other) {
    return MjOptionWrapper(other);
  });
  mjOption.def("__deepcopy__", [](const MjOptionWrapper& other, py::dict) {
    return MjOptionWrapper(other);
  });
  DefineStructFunctions(mjOption);

#define X(type, var)                                               \
  mjOption.def_property(                                           \
      #var, [](const MjOptionWrapper& c) { return c.get()->var; }, \
      [](MjOptionWrapper& c, type rhs) { c.get()->var = rhs; });
  MJOPTION_SCALARS
#undef X

#define X(var, dim) DefinePyArray(mjOption, #var, &MjOptionWrapper::var);
  MJOPTION_VECTORS
#undef X

  // ==================== MJVISUAL =============================================
  py::class_<MjVisualWrapper> mjVisual(m, "MjVisual");
  mjVisual.def("__copy__", [](const MjVisualWrapper& other) {
    return MjVisualWrapper(other);
  });
  mjVisual.def("__deepcopy__", [](const MjVisualWrapper& other, py::dict) {
    return MjVisualWrapper(other);
  });
  DefineStructFunctions(mjVisual);

  py::class_<raw::MjVisualGlobal> mjVisualGlobal(mjVisual, "Global");
  mjVisualGlobal.def("__copy__", [](const raw::MjVisualGlobal& other) {
    return raw::MjVisualGlobal(other);
  });
  mjVisualGlobal.def(
      "__deepcopy__", [](const raw::MjVisualGlobal& other, py::dict) {
        return raw::MjVisualGlobal(other);
      });
  DefineStructFunctions(mjVisualGlobal);
#define X(var) mjVisualGlobal.def_readwrite(#var, &raw::MjVisualGlobal::var)
  X(fovy);
  X(ipd);
  X(linewidth);
  X(glow);
  X(offwidth);
  X(offheight);
#undef X

  py::class_<raw::MjVisualQuality> mjVisualQuality(mjVisual, "Quality");
  mjVisualQuality.def("__copy__", [](const raw::MjVisualQuality& other) {
    return raw::MjVisualQuality(other);
  });
  mjVisualQuality.def(
      "__deepcopy__", [](const raw::MjVisualQuality& other, py::dict) {
        return raw::MjVisualQuality(other);
      });
  DefineStructFunctions(mjVisualQuality);
#define X(var) mjVisualQuality.def_readwrite(#var, &raw::MjVisualQuality::var)
  X(shadowsize);
  X(offsamples);
  X(numslices);
  X(numstacks);
  X(numquads);
#undef X

  py::class_<MjVisualHeadlightWrapper> mjVisualHeadlight(mjVisual, "Headlight");
  mjVisualHeadlight.def(
      "__copy__", [](const MjVisualHeadlightWrapper& other) {
        return MjVisualHeadlightWrapper(other);
      });
  mjVisualHeadlight.def(
      "__deepcopy__", [](const MjVisualHeadlightWrapper& other, py::dict) {
        return MjVisualHeadlightWrapper(other);
      });
  DefineStructFunctions(mjVisualHeadlight);
  #define X(var) \
      DefinePyArray(mjVisualHeadlight, #var, &MjVisualHeadlightWrapper::var)
  X(ambient);
  X(diffuse);
  X(specular);
  #undef X
  mjVisualHeadlight.def_property(
      "active",
      [](const MjVisualHeadlightWrapper& c) { return c.get()->active; },
      [](MjVisualHeadlightWrapper& c, int rhs) {
        return c.get()->active = rhs;
      });

  py::class_<raw::MjVisualMap> mjVisualMap(mjVisual, "Map");
  mjVisualMap.def("__copy__", [](const raw::MjVisualMap& other) {
    return raw::MjVisualMap(other);
  });
  mjVisualMap.def(
      "__deepcopy__", [](const raw::MjVisualMap& other, py::dict) {
        return raw::MjVisualMap(other);
      });
  DefineStructFunctions(mjVisualMap);
#define X(var) mjVisualMap.def_readwrite(#var, &raw::MjVisualMap::var)
  X(stiffness);
  X(stiffnessrot);
  X(force);
  X(torque);
  X(alpha);
  X(fogstart);
  X(fogend);
  X(znear);
  X(zfar);
  X(haze);
  X(shadowclip);
  X(shadowscale);
  X(actuatortendon);
#undef X

  py::class_<raw::MjVisualScale> mjVisualScale(mjVisual, "Scale");
  mjVisualScale.def("__copy__", [](const raw::MjVisualScale& other) {
    return raw::MjVisualScale(other);
  });
  mjVisualScale.def(
      "__deepcopy__", [](const raw::MjVisualScale& other, py::dict) {
        return raw::MjVisualScale(other);
      });
  DefineStructFunctions(mjVisualScale);
#define X(var) mjVisualScale.def_readwrite(#var, &raw::MjVisualScale::var)
  X(forcewidth);
  X(contactwidth);
  X(contactheight);
  X(connect);
  X(com);
  X(camera);
  X(light);
  X(selectpoint);
  X(jointlength);
  X(jointwidth);
  X(actuatorlength);
  X(actuatorwidth);
  X(framelength);
  X(framewidth);
  X(constraint);
  X(slidercrank);
#undef X

  py::class_<MjVisualRgbaWrapper> mjVisualRgba(mjVisual, "Rgba");
  mjVisualRgba.def("__copy__", [](const MjVisualRgbaWrapper& other) {
    return MjVisualRgbaWrapper(other);
  });
  mjVisualRgba.def(
      "__deepcopy__", [](const MjVisualRgbaWrapper& other, py::dict) {
        return MjVisualRgbaWrapper(other);
      });
  DefineStructFunctions(mjVisualRgba);
#define X(var) DefinePyArray(mjVisualRgba, #var, &MjVisualRgbaWrapper::var)
  X(fog);
  X(haze);
  X(force);
  X(inertia);
  X(joint);
  X(actuator);
  X(actuatornegative);
  X(actuatorpositive);
  X(com);
  X(camera);
  X(light);
  X(selectpoint);
  X(connect);
  X(contactpoint);
  X(contactforce);
  X(contactfriction);
  X(contacttorque);
  X(contactgap);
  X(rangefinder);
  X(constraint);
  X(slidercrank);
  X(crankbroken);
#undef X

#define X(var)                    \
  mjVisual.def_property_readonly( \
      #var, [](const MjVisualWrapper& c) -> auto& { return c.get()->var; });
  // mjVisual.global is exposed as "global_" to avoid clash with the Python
  // keyword.
  mjVisual.def_property_readonly(
      "global_",
      [](const MjVisualWrapper& c) -> auto& { return c.get()->global; });
  X(quality);
  mjVisual.def_readonly("headlight", &MjVisualWrapper::headlight);
  X(map);
  X(scale);
  mjVisual.def_readonly("rgba", &MjVisualWrapper::rgba);
#undef X

  // ==================== MJMODEL ==============================================
  py::class_<MjModelWrapper> mjModel(m, "MjModel");
  mjModel.def_static(
      "from_xml_string", &MjModelWrapper::LoadXML,
      py::arg("xml"), py::arg_v("assets", py::none()),
      py::doc(
R"(Loads an MjModel from an XML string and an optional assets dictionary.)"));
  mjModel.def_static(
      "from_xml_path", &MjModelWrapper::LoadXMLFile,
      py::arg("filename"), py::arg_v("assets", py::none()),
      py::doc(
R"(Loads an MjModel from an XML file and an optional assets dictionary.

The filename for the XML can also refer to a key in the assets dictionary.
This is useful for example when the XML is not available as a file on disk.)"));
  mjModel.def_static(
      "from_binary_path", &MjModelWrapper::LoadBinaryFile,
      py::arg("filename"), py::arg_v("assets", py::none()),
      py::doc(
R"(Loads an MjModel from an MJB file and an optional assets dictionary.

The filename for the MJB can also refer to a key in the assets dictionary.
This is useful for example when the MJB is not available as a file on disk.)"));
  mjModel.def_property_readonly("_address", [](const MjModelWrapper& m) {
    return reinterpret_cast<std::uintptr_t>(m.get());
  });
  mjModel.def("__copy__", [](const MjModelWrapper& other) {
    return MjModelWrapper(other);
  });
  mjModel.def("__deepcopy__", [](const MjModelWrapper& other, py::dict) {
    return MjModelWrapper(other);
  });
  mjModel.def(py::pickle(
      [](const MjModelWrapper& m) {  // __getstate__
        std::ostringstream output(std::ios::out | std::ios::binary);
        m.Serialize(output);
        return py::bytes(output.str());
      },
      [](py::bytes b) {  // __setstate__
        std::istringstream input(b, std::ios::in | std::ios::binary);
        return MjModelWrapper::Deserialize(input);
      }));

  mjModel.def_readonly("opt", &MjModelWrapper::opt);
  mjModel.def_readonly("vis", &MjModelWrapper::vis);
  mjModel.def_readonly("stat", &MjModelWrapper::stat);

#define X(var)                   \
  mjModel.def_property_readonly( \
      #var, [](const MjModelWrapper& m) { return m.get()->var; });
  MJMODEL_INTS
#undef X

#define X(dtype, var, dim0, dim1)                        \
  if constexpr (std::string_view(#var) != "text_data" &&     \
                std::string_view(#var) != "names") { \
    DefinePyArray(mjModel, #var, &MjModelWrapper::var);  \
  }
  MJMODEL_POINTERS
#undef X

  mjModel.def_property_readonly(
      "text_data", [](const MjModelWrapper& m) -> const auto& {
        // Return the full bytes array of concatenated text data
        return m.text_data_bytes;
      });
  mjModel.def_property_readonly(
      "names", [](const MjModelWrapper& m) -> const auto& {
        // Return the full bytes array of concatenated names
        return m.names_bytes;
      });

#define XGROUP(MjModelGroupedViews, field, nfield, FIELD_XMACROS)             \
  mjModel.def(                                                                \
      #field,                                                                 \
      [](MjModelWrapper& m, int i) -> auto& { return m.indexer().field(i); }, \
      py::return_value_policy::reference_internal);                           \
  mjModel.def(                                                                \
      #field, [](MjModelWrapper& m, std::string_view name) -> auto& {         \
        return m.indexer().field##_by_name(name);                             \
      },                                                                      \
      py::return_value_policy::reference_internal);

  MJMODEL_VIEW_GROUPS
#undef XGROUP

#define XGROUP(field, altname, FIELD_XMACROS)                                 \
  mjModel.def(                                                                \
      #altname,                                                               \
      [](MjModelWrapper& m, int i) -> auto& { return m.indexer().field(i); }, \
      py::return_value_policy::reference_internal);                           \
  mjModel.def(                                                                \
      #altname, [](MjModelWrapper& m, std::string_view name) -> auto& {       \
        return m.indexer().field##_by_name(name);                             \
      },                                                                      \
      py::return_value_policy::reference_internal);

  MJMODEL_VIEW_GROUPS_ALTNAMES
#undef XGROUP

#define XGROUP(MjModelGroupedViews, field, nfield, FIELD_XMACROS)           \
  {                                                                         \
    using GroupedViews = MjModelGroupedViews;                               \
    py::class_<MjModelGroupedViews> groupedViews(m, "_" #MjModelGroupedViews); \
    FIELD_XMACROS                                                           \
  }
#define X(type, prefix, var, dim0, dim1)                                    \
  groupedViews.def_property(                                                \
      #var, &GroupedViews::var, [](GroupedViews& views, py::handle rhs) {   \
        (views.var())[py::slice(py::none(), py::none(), py::none())] = rhs; \
      });

  MJMODEL_VIEW_GROUPS
#undef X
#undef XGROUP

  {
    py::handle builtins(PyEval_GetBuiltins());
    builtins[MjModelWrapper::kFromRawPointer] =
        reinterpret_cast<std::uintptr_t>(reinterpret_cast<void*>(
        &MjModelWrapper::FromRawPointer));
  }

  // ==================== MJWARNINGSTAT ========================================
  py::class_<MjWarningStatWrapper> mjWarningStat(m, "MjWarningStat");
  mjWarningStat.def(py::init<>());
  mjWarningStat.def("__copy__", [](const MjWarningStatWrapper& other) {
    return MjWarningStatWrapper(other);
  });
  mjWarningStat.def(
      "__deepcopy__", [](const MjWarningStatWrapper& other, py::dict) {
        return MjWarningStatWrapper(other);
      });
  DefineStructFunctions(mjWarningStat);
#define X(var)                                                             \
  mjWarningStat.def_property(                                              \
      #var, [](const MjWarningStatWrapper& d) { return d.get()->var; },    \
      [](MjWarningStatWrapper& d, decltype(raw::MjWarningStat::var) rhs) { \
        d.get()->var = rhs;                                                \
      });
  X(lastinfo);
  X(number);
#undef X

  py::class_<MjWarningStatList> mjWarningStatList(m, "_MjWarningStatList");
  mjWarningStatList.def(
      "__getitem__",
      &MjWarningStatList::operator[],
      py::return_value_policy::reference);
  mjWarningStatList.def(
      "__getitem__",
      [](MjWarningStatList& list, ::mjtWarning idx) { return list[idx]; },
      py::return_value_policy::reference);
  mjWarningStatList.def("__getitem__", &MjWarningStatList::Slice);
  mjWarningStatList.def("__len__", &MjWarningStatList::size);
  DefineStructFunctions(mjWarningStatList);

#define X(type, var) \
  mjWarningStatList.def_readonly(#var, &MjWarningStatList::var)
  X(int, lastinfo);
  X(int, number);
#undef X

  // ==================== MJTIMERSTAT ==========================================
  py::class_<MjTimerStatWrapper> mjTimerStat(m, "MjTimerStat");
  mjTimerStat.def(py::init<>());
  mjTimerStat.def("__copy__", [](const MjTimerStatWrapper& other) {
    return MjTimerStatWrapper(other);
  });
  mjTimerStat.def(
      "__deepcopy__", [](const MjTimerStatWrapper& other, py::dict) {
        return MjTimerStatWrapper(other);
      });
  DefineStructFunctions(mjTimerStat);
#define X(var)                                                         \
  mjTimerStat.def_property(                                            \
      #var, [](const MjTimerStatWrapper& d) { return d.get()->var; },  \
      [](MjTimerStatWrapper& d, decltype(raw::MjTimerStat::var) rhs) { \
        d.get()->var = rhs;                                            \
      });
  X(duration);
  X(number);
#undef X

  py::class_<MjTimerStatList> mjTimerStatList(m, "_MjTimerStatList");
  mjTimerStatList.def(
      "__getitem__",
      &MjTimerStatList::operator[],
      py::return_value_policy::reference);
  mjTimerStatList.def(
      "__getitem__",
      [](MjTimerStatList& list, ::mjtTimer idx) { return list[idx]; },
      py::return_value_policy::reference);
  mjTimerStatList.def("__getitem__", &MjTimerStatList::Slice);
  mjTimerStatList.def("__len__", &MjTimerStatList::size);
  DefineStructFunctions(mjTimerStatList);

#define X(type, var) \
  mjTimerStatList.def_readonly(#var, &MjTimerStatList::var)
  X(mjtNum, duration);
  X(int, number);
#undef X

  // ==================== MJSOLVERSTAT =========================================
  py::class_<MjSolverStatWrapper> mjSolverStat(m, "MjSolverStat");
  mjSolverStat.def(py::init<>());
  mjSolverStat.def("__copy__", [](const MjSolverStatWrapper& other) {
    return MjSolverStatWrapper(other);
  });
  mjSolverStat.def(
      "__deepcopy__", [](const MjSolverStatWrapper& other, py::dict) {
        return MjSolverStatWrapper(other);
      });
  DefineStructFunctions(mjSolverStat);
#define X(var)                                                           \
  mjSolverStat.def_property(                                             \
      #var, [](const MjSolverStatWrapper& d) { return d.get()->var; },   \
      [](MjSolverStatWrapper& d, decltype(raw::MjSolverStat::var) rhs) { \
        d.get()->var = rhs;                                              \
      });
  X(improvement);
  X(gradient);
  X(lineslope);
  X(nactive);
  X(nchange);
  X(neval);
  X(nupdate);
#undef X

  py::class_<MjSolverStatList> mjSolverStatList(m, "_MjSolverStatList");
  mjSolverStatList.def("__getitem__", &MjSolverStatList::operator[],
                    py::return_value_policy::reference);
  mjSolverStatList.def("__getitem__", &MjSolverStatList::Slice);
  mjSolverStatList.def("__len__", &MjSolverStatList::size);
  DefineStructFunctions(mjSolverStatList);

#define X(type, var) \
  mjSolverStatList.def_readonly(#var, &MjSolverStatList::var)
  X(mjtNum, improvement);
  X(mjtNum, gradient);
  X(mjtNum, lineslope);
  X(int, nactive);
  X(int, nchange);
  X(int, neval);
  X(int, nupdate);
#undef X

  // ==================== MJCONTACT ============================================
  py::class_<MjContactWrapper> mjContact(m, "MjContact");
  mjContact.def(py::init<>());
  mjContact.def("__copy__", [](const MjContactWrapper& self) {
    return MjContactWrapper(self);
  });
  mjContact.def("__deepcopy__", [](const MjContactWrapper& self, py::dict) {
    return MjContactWrapper(self);
  });
  DefineStructFunctions(mjContact);

#define X(var)                                                      \
  mjContact.def_property(                                           \
      #var, [](const MjContactWrapper& c) { return c.get()->var; }, \
      [](MjContactWrapper& c, decltype(raw::MjContact::var) rhs) {  \
        c.get()->var = rhs;                                         \
      })
  X(dist);
  X(includemargin);
  X(mu);
  X(dim);
  X(geom1);
  X(geom2);
  X(exclude);
  X(efc_address);
#undef X

#define X(var) DefinePyArray(mjContact, #var, &MjContactWrapper::var)
  X(pos);
  X(frame);
  X(friction);
  X(solref);
  X(solimp);
  X(H);
#undef X

  py::class_<MjContactList> mjContactList(m, "_MjContactList");
  mjContactList.def("__getitem__", &MjContactList::operator[],
                    py::return_value_policy::reference);
  mjContactList.def("__getitem__", &MjContactList::Slice);
  mjContactList.def("__len__", &MjContactList::size);
  DefineStructFunctions(mjContactList);

#define X(type, var) mjContactList.def_readonly(#var, &MjContactList::var)
  X(mjtNum, dist);
  X(mjtNum, pos);
  X(mjtNum, frame);
  X(mjtNum, includemargin);
  X(mjtNum, friction);
  X(mjtNum, solref);
  X(mjtNum, solimp);
  X(mjtNum, mu);
  X(mjtNum, H);
  X(int, dim);
  X(int, geom1);
  X(int, geom2);
  X(int, exclude);
  X(int, efc_address);
#undef X

  // ==================== MJDATA ===============================================
  py::class_<MjDataWrapper> mjData(m, "MjData");
  mjData.def(py::init<const MjModelWrapper&>());
  mjData.def_property_readonly("_address", [](const MjDataWrapper& d) {
    return reinterpret_cast<std::uintptr_t>(d.get());
  });
  mjData.def("__copy__", [](const MjDataWrapper& other) {
    return MjDataWrapper(other);
  });
  mjData.def("__deepcopy__", [](const MjDataWrapper& other, py::dict) {
    return MjDataWrapper(other);
  });
  mjData.def(py::pickle(
      [](const MjDataWrapper& d) {  // __getstate__
        std::ostringstream output(std::ios::out | std::ios::binary);
        d.Serialize(output);
        return py::bytes(output.str());
      },
      [](py::bytes b) {  // __setstate__
        std::istringstream input(b, std::ios::in | std::ios::binary);
        return MjDataWrapper::Deserialize(input);
      }));

#define X(type, var)                                             \
  mjData.def_property(                                           \
      #var, [](const MjDataWrapper& d) { return d.get()->var; }, \
      [](MjDataWrapper& d, decltype(raw::MjData::var) rhs) {     \
        d.get()->var = rhs;                                      \
      });
  MJDATA_SCALAR
#undef X

#define X(dtype, var, dim0, dim1) \
  DefinePyArray(mjData, #var, &MjDataWrapper::var);
  MJDATA_POINTERS
  MJDATA_VECTOR
#undef X

#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS)             \
  mjData.def(                                                                \
      #field,                                                                \
      [](MjDataWrapper& d, int i) -> auto& { return d.indexer().field(i); }, \
      py::return_value_policy::reference_internal);                          \
  mjData.def(                                                                \
      #field, [](MjDataWrapper& d, std::string_view name) -> auto& {         \
        return d.indexer().field##_by_name(name);                            \
      },                                                                     \
      py::return_value_policy::reference_internal);

  MJDATA_VIEW_GROUPS
#undef XGROUP

#define XGROUP(field, altname, FIELD_XMACROS)                                \
  mjData.def(                                                                \
      #altname,                                                              \
      [](MjDataWrapper& d, int i) -> auto& { return d.indexer().field(i); }, \
      py::return_value_policy::reference_internal);                          \
  mjData.def(                                                                \
      #altname, [](MjDataWrapper& d, std::string_view name) -> auto& {       \
        return d.indexer().field##_by_name(name);                            \
      },                                                                     \
      py::return_value_policy::reference_internal);

  MJDATA_VIEW_GROUPS_ALTNAMES
#undef XGROUP

#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS)             \
  {                                                                          \
    using GroupedViews = MjDataGroupedViews;                                 \
    py::class_<MjDataGroupedViews> groupedViews(m, "_" #MjDataGroupedViews); \
    FIELD_XMACROS                                                            \
  }
#define X(type, prefix, var, dim0, dim1)                                    \
  groupedViews.def_property(                                                \
      #var, &GroupedViews::var, [](GroupedViews& views, py::handle rhs) {   \
        (views.var())[py::slice(py::none(), py::none(), py::none())] = rhs; \
      });

  MJDATA_VIEW_GROUPS
#undef X
#undef XGROUP

  {
    py::handle builtins(PyEval_GetBuiltins());
    builtins[MjDataWrapper::kFromRawPointer] =
        reinterpret_cast<std::uintptr_t>(reinterpret_cast<void*>(
        &MjDataWrapper::FromRawPointer));
  }

  // ==================== MJSTATISTIC ==========================================
  py::class_<MjStatisticWrapper> mjStatistic(m, "MjStatistic");
  mjStatistic.def(py::init<>());
  mjStatistic.def("__copy__", [](const MjStatisticWrapper& other) {
    return MjStatisticWrapper(other);
  });
  mjStatistic.def(
      "__deepcopy__", [](const MjStatisticWrapper& other, py::dict) {
        return MjStatisticWrapper(other);
      });
  DefineStructFunctions(mjStatistic);

#define X(var)                                                         \
  mjStatistic.def_property(                                            \
      #var, [](const MjStatisticWrapper& c) { return c.get()->var; },  \
      [](MjStatisticWrapper& c, decltype(raw::MjStatistic::var) rhs) { \
        c.get()->var = rhs;                                            \
      })
  X(meaninertia);
  X(meanmass);
  X(meansize);
  X(extent);
#undef X

#define X(var) DefinePyArray(mjStatistic, #var, &MjStatisticWrapper::var)
  X(center);
#undef X

  // ==================== MJLROPT ==============================================
  py::class_<raw::MjLROpt> mjLROpt(m, "MjLROpt");
  mjLROpt.def(py::init<>());
  mjLROpt.def("__copy__", [](const raw::MjLROpt& other) {
    return raw::MjLROpt(other);
  });
  mjLROpt.def("__deepcopy__", [](const raw::MjLROpt& other, py::dict) {
    return raw::MjLROpt(other);
  });
  DefineStructFunctions(mjLROpt);
#define X(var) mjLROpt.def_readwrite(#var, &raw::MjLROpt::var)
  X(mode);
  X(useexisting);
  X(uselimit);
  X(accel);
  X(maxforce);
  X(timeconst);
  X(timestep);
  X(inttotal);
  X(inteval);
  X(tolrange);
#undef X

  // ==================== MJVPERTURB ===========================================
  py::class_<MjvPerturbWrapper> mjvPerturb(m, "MjvPerturb");
  mjvPerturb.def(py::init<>());
  mjvPerturb.def("__copy__", [](const MjvPerturbWrapper& other) {
    return MjvPerturbWrapper(other);
  });
  mjvPerturb.def("__deepcopy__", [](const MjvPerturbWrapper& other, py::dict) {
    return MjvPerturbWrapper(other);
  });
  DefineStructFunctions(mjvPerturb);
#define X(var)                                                       \
  mjvPerturb.def_property(                                           \
      #var, [](const MjvPerturbWrapper& c) { return c.get()->var; }, \
      [](MjvPerturbWrapper& c, decltype(raw::MjvPerturb::var) rhs) { \
        c.get()->var = rhs;                                          \
      })
  X(select);
  X(skinselect);
  X(active);
  X(active2);
#undef X

#define X(var) DefinePyArray(mjvPerturb, #var, &MjvPerturbWrapper::var)
  X(refpos);
  X(refquat);
  X(localpos);
#undef X

  // ==================== MJVCAMERA ============================================
  py::class_<MjvCameraWrapper> mjvCamera(m, "MjvCamera");
  mjvCamera.def(py::init<>());
  mjvCamera.def("__copy__", [](const MjvCameraWrapper& other) {
    return MjvCameraWrapper(other);
  });
  mjvCamera.def("__deepcopy__", [](const MjvCameraWrapper& other, py::dict) {
    return MjvCameraWrapper(other);
  });
  DefineStructFunctions(mjvCamera);
#define X(var)                                                      \
  mjvCamera.def_property(                                           \
      #var, [](const MjvCameraWrapper& c) { return c.get()->var; }, \
      [](MjvCameraWrapper& c, decltype(raw::MjvCamera::var) rhs) {  \
        c.get()->var = rhs;                                         \
      })
  X(type);
  X(fixedcamid);
  X(trackbodyid);
  X(distance);
  X(azimuth);
  X(elevation);
#undef X

#define X(var) DefinePyArray(mjvCamera, #var, &MjvCameraWrapper::var)
  X(lookat);
#undef X

  // ==================== MJVGLCAMERA ==========================================
  py::class_<MjvGLCameraWrapper> mjvGLCamera(m, "MjvGLCamera");
  mjvGLCamera.def(py::init<>());
  mjvGLCamera.def("__copy__", [](const MjvGLCameraWrapper& other) {
    return MjvGLCameraWrapper(other);
  });
  mjvGLCamera.def(
      "__deepcopy__",
      [](const MjvGLCameraWrapper& other, py::dict) {
        return MjvGLCameraWrapper(other);
      });
  DefineStructFunctions(mjvGLCamera);
#define X(var)                                                         \
  mjvGLCamera.def_property(                                            \
      #var, [](const MjvGLCameraWrapper& c) { return c.get()->var; },  \
      [](MjvGLCameraWrapper& c, decltype(raw::MjvGLCamera::var) rhs) { \
        c.get()->var = rhs;                                            \
      })
  X(frustum_center);
  X(frustum_bottom);
  X(frustum_top);
  X(frustum_near);
  X(frustum_far);
#undef X

#define X(var) DefinePyArray(mjvGLCamera, #var, &MjvGLCameraWrapper::var)
  X(pos);
  X(forward);
  X(up);
#undef X

  // ==================== MJVGEOM ==============================================
  py::class_<MjvGeomWrapper> mjvGeom(m, "MjvGeom");
  mjvGeom.def(py::init<>());
  mjvGeom.def("__copy__", [](const MjvGeomWrapper& other) {
    return MjvGeomWrapper(other);
  });
  mjvGeom.def("__deepcopy__", [](const MjvGeomWrapper& other, py::dict) {
    return MjvGeomWrapper(other);
  });
  DefineStructFunctions(mjvGeom);
#define X(var)                                                    \
  mjvGeom.def_property(                                           \
      #var, [](const MjvGeomWrapper& c) { return c.get()->var; }, \
      [](MjvGeomWrapper& c, decltype(raw::MjvGeom::var) rhs) {    \
        c.get()->var = rhs;                                       \
      })
  X(type);
  X(dataid);
  X(objtype);
  X(objid);
  X(category);
  X(texid);
  X(texuniform);
  X(texcoord);
  X(segid);
  X(emission);
  X(specular);
  X(shininess);
  X(reflectance);
  X(camdist);
  X(modelrbound);
  X(transparent);
#undef X

#define X(var) DefinePyArray(mjvGeom, #var, &MjvGeomWrapper::var)
  X(texrepeat);
  X(size);
  X(pos);
  X(mat);
  X(rgba);
#undef X

  DefinePyStr(mjvGeom, "label", &raw::MjvGeom::label);

  // ==================== MJVLIGHT =============================================
  py::class_<MjvLightWrapper> mjvLight(m, "MjvLight");
  mjvLight.def(py::init<>());
  mjvLight.def("__copy__", [](const MjvLightWrapper& other) {
    return MjvLightWrapper(other);
  });
  mjvLight.def("__deepcopy__", [](const MjvLightWrapper& other, py::dict) {
    return MjvLightWrapper(other);
  });
  DefineStructFunctions(mjvLight);
#define X(var)                                                     \
  mjvLight.def_property(                                           \
      #var, [](const MjvLightWrapper& c) { return c.get()->var; }, \
      [](MjvLightWrapper& c, decltype(raw::MjvLight::var) rhs) {   \
        c.get()->var = rhs;                                        \
      })
  X(cutoff);
  X(exponent);
  X(headlight);
  X(directional);
  X(castshadow);
#undef X

#define X(var) DefinePyArray(mjvLight, #var, &MjvLightWrapper::var)
  X(pos);
  X(dir);
  X(attenuation);
  X(ambient);
  X(diffuse);
  X(specular);
#undef X

  // ==================== MJVOPTION ============================================
  py::class_<MjvOptionWrapper> mjvOption(m, "MjvOption");
  mjvOption.def(py::init<>());
  mjvOption.def("__copy__", [](const MjvOptionWrapper& other) {
    return MjvOptionWrapper(other);
  });
  mjvOption.def("__deepcopy__", [](const MjvOptionWrapper& other, py::dict) {
    return MjvOptionWrapper(other);
  });
  DefineStructFunctions(mjvOption);
#define X(var)                                                      \
  mjvOption.def_property(                                           \
      #var, [](const MjvOptionWrapper& c) { return c.get()->var; }, \
      [](MjvOptionWrapper& c, decltype(raw::MjvOption::var) rhs) { \
        c.get()->var = rhs;                                         \
      })
  X(label);
  X(frame);
#undef X

#define X(var) DefinePyArray(mjvOption, #var, &MjvOptionWrapper::var)
  X(geomgroup);
  X(sitegroup);
  X(jointgroup);
  X(tendongroup);
  X(actuatorgroup);
  X(flags);
#undef X

  // ==================== MJVSCENE =============================================
  py::class_<MjvSceneWrapper> mjvScene(m, "MjvScene");
  mjvScene.def(py::init<>());
  mjvScene.def(py::init<const MjModelWrapper&, int>(),
               py::arg("model"), py::arg("maxgeom"));
  mjvScene.def("__copy__", [](const MjvSceneWrapper& other) {
    return MjvSceneWrapper(other);
  });
  mjvScene.def("__deepcopy__", [](const MjvSceneWrapper& other, py::dict) {
    return MjvSceneWrapper(other);
  });
#define X(var)                                                     \
  mjvScene.def_property(                                           \
      #var, [](const MjvSceneWrapper& c) { return c.get()->var; }, \
      [](MjvSceneWrapper& c, decltype(raw::MjvScene::var) rhs) {   \
        c.get()->var = rhs;                                        \
      })
  X(maxgeom);
  X(ngeom);
  X(nlight);
  X(enabletransform);
  X(scale);
  X(stereo);
  X(framewidth);
#undef X

#define X(var) DefinePyArray(mjvScene, #var, &MjvSceneWrapper::var)
  X(geoms);
  X(geomorder);
  X(skinfacenum);
  X(skinvertadr);
  X(skinvertnum);
  X(skinvert);
  X(skinnormal);
  X(lights);
  X(camera);
  X(translate);
  X(rotate);
  X(flags);
  X(framergb);
#undef X

  // ==================== MJVFIGURE ============================================
  py::class_<MjvFigureWrapper> mjvFigure(m, "MjvFigure");
  mjvFigure.def(py::init<>());
  mjvFigure.def("__copy__", [](const MjvFigureWrapper& other) {
    return MjvFigureWrapper(other);
  });
  mjvFigure.def("__deepcopy__", [](const MjvFigureWrapper& other, py::dict) {
    return MjvFigureWrapper(other);
  });
#define X(var)                                                      \
  mjvFigure.def_property(                                           \
      #var, [](const MjvFigureWrapper& c) { return c.get()->var; }, \
      [](MjvFigureWrapper& c, decltype(raw::MjvFigure::var) rhs) {  \
        c.get()->var = rhs;                                         \
      })
  X(flg_legend);
  X(flg_extend);
  X(flg_barplot);
  X(flg_selection);
  X(flg_symmetric);
  X(linewidth);
  X(gridwidth);
  X(legendoffset);
  X(subplot);
  X(highlightid);
  X(selection);
#undef X

#define X(var) DefinePyArray(mjvFigure, #var, &MjvFigureWrapper::var)
  X(flg_ticklabel);
  X(gridsize);
  X(gridrgb);
  X(figurergba);
  X(panergba);
  X(legendrgba);
  X(textrgb);
  X(linergb);
  X(range);
  X(linepnt);
  X(linedata);
  X(xaxispixel);
  X(yaxispixel);
  X(xaxisdata);
  X(yaxisdata);
#undef X

#define X(var) DefinePyStr(mjvFigure, #var, &raw::MjvFigure::var);
  X(xformat);
  X(yformat);
  X(minwidth);
  X(title);
  X(xlabel);
#undef X

  mjvFigure.def_readonly("linename", &MjvFigureWrapper::linename);

  // mjv_averageCamera returns an mjvGLCamera and we need to call the wrapper's
  // constructor on the return value. Defining the binding for this function
  // in this file to avoid symbol dependency across modules.
  m.def(
      "mjv_averageCamera",
      [](const MjvGLCameraWrapper& cam1, const MjvGLCameraWrapper& cam2) {
        return MjvGLCameraWrapper([&cam1, &cam2]() {
          py::gil_scoped_release no_gil;
          return InterceptMjErrors(mjv_averageCamera)(cam1.get(), cam2.get());
        }());
      },
      py::arg("cam1"), py::arg("cam2"),
      py::doc(python_traits::mjv_averageCamera::doc));
}  // PYBIND11_MODULE NOLINT(readability/fn_size)
}  // namespace mujoco::python::_impl
