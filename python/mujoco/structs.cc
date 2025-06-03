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

#include <cstdint>
#include <ios>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "errors.h"
#include "function_traits.h"
#include "indexer_xmacro.h"
#include "indexers.h"
#include "raw.h"
#include <pybind11/cast.h>
#include <pybind11/detail/common.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace mujoco::python::_impl {

namespace py = ::pybind11;

namespace {
#define PTRDIFF(x, y) \
  reinterpret_cast<const char*>(x) - reinterpret_cast<const char*>(y)

// Returns the shape of a NumPy array given the dimensions from an X Macro.
// If dim1 is a _literal_ constant 1, the resulting array is 1-dimensional of
// length dim0, otherwise the resulting array is 2-dimensional of shape
// (dim0, dim1).
#define X_ARRAY_SHAPE(dim0, dim1) XArrayShapeImpl(#dim1)((dim0), (dim1))

std::vector<int> XArrayShapeImpl1D(int dim0, int dim1) { return {dim0}; }

std::vector<int> XArrayShapeImpl2D(int dim0, int dim1) { return {dim0, dim1}; }

constexpr auto XArrayShapeImpl(const std::string_view dim1_str) {
  if (dim1_str == "1") {
    return XArrayShapeImpl1D;
  } else {
    return XArrayShapeImpl2D;
  }
}

py::tuple RecompileSpec(raw::MjSpec* spec, const MjModelWrapper& old_m,
                        const MjDataWrapper& old_d) {
  raw::MjModel* m = static_cast<raw::MjModel*>(mju_malloc(sizeof(mjModel)));
  m->buffer = nullptr;
  raw::MjData* d = mj_copyData(nullptr, old_m.get(), old_d.get());
  if (mj_recompile(spec, nullptr, m, d)) {
    throw py::value_error(mjs_getError(spec));
  }

  py::object m_pyobj = py::cast((MjModelWrapper(m)));
  py::object d_pyobj =
      py::cast((MjDataWrapper(py::cast<MjModelWrapper*>(m_pyobj), d)));
  return py::make_tuple(m_pyobj, d_pyobj);
}

}  // namespace

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

  mjOption.def_property_readonly_static("_float_fields", [](py::object) {
    std::vector<std::string> field_names;
#define X(type, var) field_names.push_back(#var);
    MJOPTION_FLOATS
#undef X
    return py::tuple(py::cast(field_names));
  });

  mjOption.def_property_readonly_static("_int_fields", [](py::object) {
    std::vector<std::string> field_names;
#define X(type, var) field_names.push_back(#var);
    MJOPTION_INTS
#undef X
    return py::tuple(py::cast(field_names));
  });

  mjOption.def_property_readonly_static("_floatarray_fields", [](py::object) {
    std::vector<std::string> field_names;
#define X(var, sz) field_names.push_back(#var);
    MJOPTION_VECTORS
#undef X
    return py::tuple(py::cast(field_names));
  });

  // ==================== MJVISUAL =============================================
  py::class_<MjVisualWrapper> mjVisual(m, "MjVisual");
  mjVisual.def("__copy__", [](const MjVisualWrapper& other) {
    return MjVisualWrapper(other);
  });
  mjVisual.def("__deepcopy__", [](const MjVisualWrapper& other, py::dict) {
    return MjVisualWrapper(other);
  });
  mjVisual.def("__eq__", StructsEqual<MjVisualWrapper>);
  // Special __repr__ implementation for MjVisual, since:
  // 1. Types under MjVisual confuse StructRepr;
  // 2. StructRepr does not handle the indentation of nested structs well.
  mjVisual.def("__repr__", [](py::object self) {
    std::ostringstream result;
    result << "<"
           << self.attr("__class__").attr("__name__").cast<std::string_view>();

#define X(type, var)          \
  result << "\n  " #var ": "; \
  StructReprImpl<type>(self.attr(#var), result, 2);

    X(raw::MjVisualGlobal, global_)
    X(raw::MjVisualQuality, quality)
    X(MjVisualHeadlightWrapper, headlight)
    X(raw::MjVisualMap, map)
    X(raw::MjVisualScale, scale)
    X(MjVisualRgbaWrapper, rgba)
#undef X
    result << "\n>";
    return result.str();
  });

  py::class_<raw::MjVisualGlobal> mjVisualGlobal(mjVisual, "Global");
  mjVisualGlobal.def("__copy__", [](const raw::MjVisualGlobal& other) {
    return raw::MjVisualGlobal(other);
  });
  mjVisualGlobal.def("__deepcopy__",
                     [](const raw::MjVisualGlobal& other, py::dict) {
                       return raw::MjVisualGlobal(other);
                     });
  DefineStructFunctions(mjVisualGlobal);
#define X(var) mjVisualGlobal.def_readwrite(#var, &raw::MjVisualGlobal::var)
  X(orthographic);
  X(fovy);
  X(ipd);
  X(azimuth);
  X(elevation);
  X(linewidth);
  X(glow);
  X(realtime);
  X(offwidth);
  X(offheight);
  X(ellipsoidinertia);
  X(bvactive);
#undef X

  py::class_<raw::MjVisualQuality> mjVisualQuality(mjVisual, "Quality");
  mjVisualQuality.def("__copy__", [](const raw::MjVisualQuality& other) {
    return raw::MjVisualQuality(other);
  });
  mjVisualQuality.def("__deepcopy__",
                      [](const raw::MjVisualQuality& other, py::dict) {
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
  mjVisualHeadlight.def("__copy__", [](const MjVisualHeadlightWrapper& other) {
    return MjVisualHeadlightWrapper(other);
  });
  mjVisualHeadlight.def("__deepcopy__",
                        [](const MjVisualHeadlightWrapper& other, py::dict) {
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
  mjVisualMap.def("__deepcopy__", [](const raw::MjVisualMap& other, py::dict) {
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
  mjVisualScale.def("__deepcopy__",
                    [](const raw::MjVisualScale& other, py::dict) {
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
  X(frustum);
#undef X

  py::class_<MjVisualRgbaWrapper> mjVisualRgba(mjVisual, "Rgba");
  mjVisualRgba.def("__copy__", [](const MjVisualRgbaWrapper& other) {
    return MjVisualRgbaWrapper(other);
  });
  mjVisualRgba.def("__deepcopy__",
                   [](const MjVisualRgbaWrapper& other, py::dict) {
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
  X(frustum);
  X(bv);
  X(bvactive);
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
      "from_xml_string", &MjModelWrapper::LoadXML, py::arg("xml"),
      py::arg_v("assets", py::none()),
      py::doc(
          R"(Loads an MjModel from an XML string and an optional assets dictionary.)"));
  mjModel.def_static("_from_model_ptr", [](uintptr_t addr) {
    return MjModelWrapper::WrapRawModel(reinterpret_cast<raw::MjModel*>(addr));
  });
  mjModel.def_static(
      "from_xml_path", &MjModelWrapper::LoadXMLFile, py::arg("filename"),
      py::arg_v("assets", py::none()),
      py::doc(
          R"(Loads an MjModel from an XML file and an optional assets dictionary.

The filename for the XML can also refer to a key in the assets dictionary.
This is useful for example when the XML is not available as a file on disk.)"));
  mjModel.def_static(
      "from_binary_path", &MjModelWrapper::LoadBinaryFile, py::arg("filename"),
      py::arg_v("assets", py::none()),
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

  mjModel.def_property_readonly("_sizes", [](const MjModelWrapper& m) {
    int nint = 0;
#define X(var) ++nint;
    MJMODEL_INTS
#undef X
    py::array_t<std::int64_t> sizes(nint);
    {
      int i = 0;
      auto data = sizes.mutable_unchecked();
#define X(var) data[i++] = m.get()->var;
      MJMODEL_INTS
#undef X
    }
    py::detail::array_proxy(sizes.ptr())->flags &=
        ~py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
    return sizes;
  });

  mjModel.def_property_readonly_static("_size_fields", [](py::object) {
    std::vector<std::string> fields;
#define X(var) fields.push_back(#var);
    MJMODEL_INTS
#undef X
    return py::tuple(py::cast(fields));
  });

#define X(dtype, var, dim0, dim1)                        \
  if constexpr (std::string_view(#var) != "text_data" && \
                std::string_view(#var) != "names" &&     \
                std::string_view(#var) != "paths") {     \
    DefinePyArray(mjModel, #var, &MjModelWrapper::var);  \
  }
  MJMODEL_POINTERS
#undef X

  mjModel.def_property_readonly("text_data",
                                [](const MjModelWrapper& m) -> const auto& {
                                  // Return the full bytes array of concatenated
                                  // text data
                                  return m.text_data_bytes;
                                });
  mjModel.def_property_readonly("names",
                                [](const MjModelWrapper& m) -> const auto& {
                                  // Return the full bytes array of concatenated
                                  // names
                                  return m.names_bytes;
                                });
  mjModel.def_property_readonly("paths",
                                [](const MjModelWrapper& m) -> const auto& {
                                  // Return the full bytes array of concatenated
                                  // paths
                                  return m.paths_bytes;
                                });
  mjModel.def_property_readonly("signature",
                                [](const MjModelWrapper& m) -> const uint64_t& {
                                  return m.get()->signature;
                                });

#define XGROUP(MjModelGroupedViews, field, nfield, FIELD_XMACROS)             \
  mjModel.def(                                                                \
      #field,                                                                 \
      [](MjModelWrapper& m, int i) -> auto& { return m.indexer().field(i); }, \
      py::return_value_policy::reference_internal);                           \
  mjModel.def(                                                                \
      #field,                                                                 \
      [](MjModelWrapper& m, std::string_view name) -> auto& {                 \
        return m.indexer().field##_by_name(name);                             \
      },                                                                      \
      py::return_value_policy::reference_internal, py::arg_v("name", ""));

  MJMODEL_VIEW_GROUPS
#undef XGROUP

#define XGROUP(spectype, field)                                      \
  mjModel.def(                                                       \
      "bind_scalar",                                                 \
      [](MjModelWrapper& m, spectype& spec) -> auto& {               \
        if (mjs_getSpec(spec.element)->element->signature !=         \
            m.get()->signature) {                                    \
          throw py::value_error(                                     \
              "The mjSpec does not match mjModel. Please recompile " \
              "the mjSpec.");                                        \
        }                                                            \
        return m.indexer().field(mjs_getId(spec.element));           \
      },                                                             \
      py::return_value_policy::reference_internal,                   \
      py::arg_v("spec", py::none()));

  MJMODEL_BIND_GROUPS
#undef XGROUP

#define XGROUP(field, altname, FIELD_XMACROS)                                 \
  mjModel.def(                                                                \
      #altname,                                                               \
      [](MjModelWrapper& m, int i) -> auto& { return m.indexer().field(i); }, \
      py::return_value_policy::reference_internal);                           \
  mjModel.def(                                                                \
      #altname,                                                               \
      [](MjModelWrapper& m, std::string_view name) -> auto& {                 \
        return m.indexer().field##_by_name(name);                             \
      },                                                                      \
      py::return_value_policy::reference_internal, py::arg_v("name", ""));

  MJMODEL_VIEW_GROUPS_ALTNAMES
#undef XGROUP

#define XGROUP(MjModelGroupedViews, field, nfield, FIELD_XMACROS)              \
  {                                                                            \
    using GroupedViews = MjModelGroupedViews;                                  \
    py::class_<MjModelGroupedViews> groupedViews(m, "_" #MjModelGroupedViews); \
    FIELD_XMACROS                                                              \
    groupedViews.def("__repr__", MjModelStructRepr<GroupedViews>);             \
    groupedViews.def_property_readonly(                                        \
        "id", [](GroupedViews& views) { return views.index(); });              \
    groupedViews.def_property_readonly(                                        \
        "name", [](GroupedViews& views) { return views.name(); });             \
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
        reinterpret_cast<std::uintptr_t>(
            reinterpret_cast<void*>(&MjModelWrapper::FromRawPointer));
  }

  // ==================== MJWARNINGSTAT ========================================
  py::class_<MjWarningStatWrapper> mjWarningStat(m, "MjWarningStat");
  mjWarningStat.def(py::init<>());
  mjWarningStat.def("__copy__", [](const MjWarningStatWrapper& other) {
    return MjWarningStatWrapper(other);
  });
  mjWarningStat.def("__deepcopy__",
                    [](const MjWarningStatWrapper& other, py::dict) {
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
  mjWarningStatList.def("__getitem__", &MjWarningStatList::operator[],
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
  mjTimerStat.def("__deepcopy__",
                  [](const MjTimerStatWrapper& other, py::dict) {
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
  mjTimerStatList.def("__getitem__", &MjTimerStatList::operator[],
                      py::return_value_policy::reference);
  mjTimerStatList.def(
      "__getitem__",
      [](MjTimerStatList& list, ::mjtTimer idx) { return list[idx]; },
      py::return_value_policy::reference);
  mjTimerStatList.def("__getitem__", &MjTimerStatList::Slice);
  mjTimerStatList.def("__len__", &MjTimerStatList::size);
  DefineStructFunctions(mjTimerStatList);

#define X(type, var) mjTimerStatList.def_readonly(#var, &MjTimerStatList::var)
  X(mjtNum, duration);
  X(int, number);
#undef X

  // ==================== MJSOLVERSTAT =========================================
  py::class_<MjSolverStatWrapper> mjSolverStat(m, "MjSolverStat");
  mjSolverStat.def(py::init<>());
  mjSolverStat.def("__copy__", [](const MjSolverStatWrapper& other) {
    return MjSolverStatWrapper(other);
  });
  mjSolverStat.def("__deepcopy__",
                   [](const MjSolverStatWrapper& other, py::dict) {
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

#define X(type, var) mjSolverStatList.def_readonly(#var, &MjSolverStatList::var)
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
  X(solreffriction);
  X(solimp);
  X(H);
  X(geom);
  X(flex);
  X(elem);
  X(vert);
#undef X

  py::class_<MjContactList> mjContactList(m, "_MjContactList");
  mjContactList.def("__getitem__", &MjContactList::operator[],
                    py::return_value_policy::reference);
  mjContactList.def("__getitem__", &MjContactList::Slice);
  mjContactList.def("__len__", &MjContactList::size);
  DefineStructFunctions(mjContactList);

#define X(type, var)                                                     \
  mjContactList.def_property_readonly(#var, [](const MjContactList& c) { \
    return py::array_t<type>(std::vector<int>{c.size()},                 \
                             std::vector<int>{sizeof(raw::MjContact)},   \
                             &c.get()->var, c.owner());                  \
  });
#define XN(type, var)                                                    \
  mjContactList.def_property_readonly(#var, [](const MjContactList& c) { \
    return py::array_t<type>(                                            \
        std::vector<int>{c.size(),                                       \
                         sizeof(raw::MjContact::var) / sizeof(type)},    \
        std::vector<int>{sizeof(raw::MjContact), sizeof(type)},          \
        &c.get()->var[0], c.owner());                                    \
  });
  X(mjtNum, dist);
  XN(mjtNum, pos);
  XN(mjtNum, frame);
  X(mjtNum, includemargin);
  XN(mjtNum, friction);
  XN(mjtNum, solref);
  XN(mjtNum, solreffriction);
  XN(mjtNum, solimp);
  X(mjtNum, mu);
  XN(mjtNum, H);
  X(int, dim);
  X(int, geom1);
  X(int, geom2);
  X(int, exclude);
  X(int, efc_address);
  XN(int, geom);
  XN(int, flex);
  XN(int, elem);
  XN(int, vert);
#undef X
#undef XN

  // ==================== MJDATA ===============================================
  py::class_<MjDataWrapper> mjData(m, "MjData");
  mjData.def(py::init<MjModelWrapper*>());
  mjData.def_property_readonly("_address", [](const MjDataWrapper& d) {
    return reinterpret_cast<std::uintptr_t>(d.get());
  });
  mjData.def_property_readonly(
      "model", [](const MjDataWrapper& d) { return &d.model(); });
  mjData.def("__copy__",
             [](const MjDataWrapper& other) { return MjDataWrapper(other); });
  mjData.def("__deepcopy__", [](const MjDataWrapper& other, py::dict memo) {
    // Use copy.deepcopy(model) to make a model that Python is aware of.
    py::object new_model_py =
        py::cast(other.model()).attr("__deepcopy__")(memo);
    return MjDataWrapper(other, new_model_py.cast<MjModelWrapper*>());
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
  mjData.def_property_readonly(
      "signature",
      [](const MjDataWrapper& d) -> uint64_t { return d.get()->signature; });

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
  MJDATA_ARENA_POINTERS_CONTACT
#undef X

#undef MJ_M
#define MJ_M(x) d.model().get()->x
#undef MJ_D
#define MJ_D(x) d.get()->x
#define X(dtype, var, dim0, dim1)                                           \
  mjData.def_property_readonly(#var, [](const MjDataWrapper& d) {           \
    return InitPyArray(X_ARRAY_SHAPE(dim0, dim1), d.get()->var, d.owner()); \
  });

  MJDATA_ARENA_POINTERS_SOLVER
  MJDATA_ARENA_POINTERS_DUAL
  MJDATA_ARENA_POINTERS_ISLAND

#undef MJ_M
#define MJ_M(x) (x)
#undef MJ_D
#define MJ_D(x) (x)
#undef X

#define X(dtype, var, dim0, dim1) \
  DefinePyArray(mjData, #var, &MjDataWrapper::var);
  MJDATA_VECTOR
#undef X

#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS)             \
  mjData.def(                                                                \
      #field,                                                                \
      [](MjDataWrapper& d, int i) -> auto& { return d.indexer().field(i); }, \
      py::return_value_policy::reference_internal);                          \
  mjData.def(                                                                \
      #field,                                                                \
      [](MjDataWrapper& d, std::string_view name) -> auto& {                 \
        return d.indexer().field##_by_name(name);                            \
      },                                                                     \
      py::return_value_policy::reference_internal, py::arg_v("name", ""));

  MJDATA_VIEW_GROUPS
#undef XGROUP

#define XGROUP(spectype, field)                                     \
  mjData.def(                                                       \
      "bind_scalar",                                                \
      [](MjDataWrapper& d, spectype& spec) -> auto& {               \
        if (mjs_getSpec(spec.element)->element->signature !=        \
            d.get()->signature) {                                   \
          throw py::value_error(                                    \
              "The mjSpec does not match mjData. Please recompile " \
              "the mjSpec.");                                       \
        }                                                           \
        return d.indexer().field(mjs_getId(spec.element));          \
      },                                                            \
      py::return_value_policy::reference_internal,                  \
      py::arg_v("spec", py::none()));

  MJDATA_BIND_GROUPS
#undef XGROUP

#define XGROUP(field, altname, FIELD_XMACROS)                                \
  mjData.def(                                                                \
      #altname,                                                              \
      [](MjDataWrapper& d, int i) -> auto& { return d.indexer().field(i); }, \
      py::return_value_policy::reference_internal);                          \
  mjData.def(                                                                \
      #altname,                                                              \
      [](MjDataWrapper& d, std::string_view name) -> auto& {                 \
        return d.indexer().field##_by_name(name);                            \
      },                                                                     \
      py::return_value_policy::reference_internal, py::arg_v("name", ""));

  MJDATA_VIEW_GROUPS_ALTNAMES
#undef XGROUP

#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS)             \
  {                                                                          \
    using GroupedViews = MjDataGroupedViews;                                 \
    py::class_<MjDataGroupedViews> groupedViews(m, "_" #MjDataGroupedViews); \
    FIELD_XMACROS                                                            \
    groupedViews.def("__repr__", MjDataStructRepr<GroupedViews>);            \
    groupedViews.def_property_readonly(                                      \
        "id", [](GroupedViews& views) { return views.index(); });            \
    groupedViews.def_property_readonly(                                      \
        "name", [](GroupedViews& views) { return views.name(); });           \
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
    builtins[MjDataWrapper::kFromRawPointer] = reinterpret_cast<std::uintptr_t>(
        reinterpret_cast<void*>(&MjDataWrapper::FromRawPointer));
  }

  // ==================== MJSTATISTIC ==========================================
  py::class_<MjStatisticWrapper> mjStatistic(m, "MjStatistic");
  mjStatistic.def(py::init<>());
  mjStatistic.def("__copy__", [](const MjStatisticWrapper& other) {
    return MjStatisticWrapper(other);
  });
  mjStatistic.def("__deepcopy__",
                  [](const MjStatisticWrapper& other, py::dict) {
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
  mjLROpt.def("__copy__",
              [](const raw::MjLROpt& other) { return raw::MjLROpt(other); });
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
  X(interval);
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
  X(flexselect);
  X(skinselect);
  X(active);
  X(active2);
  X(localmass);
  X(scale);
#undef X

#define X(var) DefinePyArray(mjvPerturb, #var, &MjvPerturbWrapper::var)
  X(refpos);
  X(refquat);
  X(refselpos);
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
  X(orthographic);
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
  mjvGLCamera.def("__deepcopy__",
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
  X(frustum_width);
  X(frustum_bottom);
  X(frustum_top);
  X(frustum_near);
  X(frustum_far);
  X(orthographic);
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
  X(matid);
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
  X(type);
  X(texid);
  X(castshadow);
  X(bulbradius);
  X(intensity);
  X(range);
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
      [](MjvOptionWrapper& c, decltype(raw::MjvOption::var) rhs) {  \
        c.get()->var = rhs;                                         \
      })
  X(label);
  X(frame);
  X(bvh_depth);
  X(flex_layer);
#undef X

#define X(var) DefinePyArray(mjvOption, #var, &MjvOptionWrapper::var)
  X(geomgroup);
  X(sitegroup);
  X(jointgroup);
  X(tendongroup);
  X(actuatorgroup);
  X(flexgroup);
  X(skingroup);
  X(flags);
#undef X

  // ==================== MJVSCENE =============================================
  py::class_<MjvSceneWrapper> mjvScene(m, "MjvScene");
  mjvScene.def(py::init<>());
  mjvScene.def(py::init<const MjModelWrapper&, int>(), py::arg("model"),
               py::arg("maxgeom"));
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
  X(nflex);
  X(nskin);
  X(nlight);
  X(flexvertopt);
  X(flexedgeopt);
  X(flexfaceopt);
  X(flexskinopt);
  X(enabletransform);
  X(scale);
  X(stereo);
  X(framewidth);
#undef X

#define X(var) DefinePyArray(mjvScene, #var, &MjvSceneWrapper::var)
  X(geoms);
  X(geomorder);
  X(flexedgeadr);
  X(flexedgenum);
  X(flexvertadr);
  X(flexvertnum);
  X(flexfaceadr);
  X(flexfacenum);
  X(flexfaceused);
  X(flexedge);
  X(flexvert);
  X(flexface);
  X(flexnormal);
  X(flextexcoord);
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
  X(highlight);
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

  m.def("_recompile_spec_addr", [](uintptr_t spec_addr, const MjModelWrapper& m,
                                   const MjDataWrapper& d) {
    return RecompileSpec(reinterpret_cast<raw::MjSpec*>(spec_addr), m, d);
  });
}  // PYBIND11_MODULE NOLINT(readability/fn_size)
}  // namespace mujoco::python::_impl
