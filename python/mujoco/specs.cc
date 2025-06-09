// Copyright 2024 DeepMind Technologies Limited
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

#include <array>
#include <cstddef>  // IWYU pragma: keep
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <string_view>  // IWYU pragma: keep
#include <unordered_map>
#include <utility>
#include <vector>       // IWYU pragma: keep

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <mujoco/mjspec.h>  // IWYU pragma: keep
#include <mujoco/mujoco.h>
#include "errors.h"
#include "indexers.h"  // IWYU pragma: keep
#include "specs_wrapper.h"  // IWYU pragma: keep
#include "raw.h"
#include "structs.h"  // IWYU pragma: keep
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/eigen/matrix.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace py = ::pybind11;

namespace mujoco::python {
using MjInt2 = Eigen::Map<Eigen::Vector2i>;
using MjInt3 = Eigen::Map<Eigen::Vector3i>;
using MjFloat2 = Eigen::Map<Eigen::Vector2f>;
using MjFloat3 = Eigen::Map<Eigen::Vector3f>;
using MjFloat4 = Eigen::Map<Eigen::Vector4f>;
using MjDouble2 = Eigen::Map<Eigen::Vector2d>;
using MjDouble3 = Eigen::Map<Eigen::Vector3d>;
using MjDouble4 = Eigen::Map<Eigen::Vector4d>;
using MjDouble5 = Eigen::Map<Eigen::Matrix<double, 5, 1>>;
using MjDouble6 = Eigen::Map<Eigen::Matrix<double, 6, 1>>;
using MjDouble10 = Eigen::Map<Eigen::Matrix<double, 10, 1>>;
using MjDouble11 = Eigen::Map<Eigen::Matrix<double, 11, 1>>;
using MjDoubleVec = Eigen::Map<Eigen::VectorXd>;

using MjIntRef2 = Eigen::Ref<const Eigen::Vector2i>;
using MjIntRef3 = Eigen::Ref<const Eigen::Vector3i>;
using MjFloatRef2 = Eigen::Ref<const Eigen::Vector2f>;
using MjFloatRef3 = Eigen::Ref<const Eigen::Vector3f>;
using MjFloatRef4 = Eigen::Ref<const Eigen::Vector4f>;
using MjDoubleRef2 = Eigen::Ref<const Eigen::Vector2d>;
using MjDoubleRef3 = Eigen::Ref<const Eigen::Vector3d>;
using MjDoubleRef4 = Eigen::Ref<const Eigen::Vector4d>;
using MjDoubleRef5 = Eigen::Ref<const Eigen::Matrix<double, 5, 1>>;
using MjDoubleRef6 = Eigen::Ref<const Eigen::Matrix<double, 6, 1>>;
using MjDoubleRef10 = Eigen::Ref<const Eigen::Matrix<double, 10, 1>>;
using MjDoubleRef11 = Eigen::Ref<const Eigen::Matrix<double, 11, 1>>;
using MjDoubleRefVec = Eigen::Ref<const Eigen::VectorXd>;

template <typename LoadFunc>
static raw::MjSpec* LoadSpecFileImpl(
    const std::string& filename,
    const std::vector<_impl::VfsAsset>& assets,
    LoadFunc&& loadfunc) {
  mjVFS vfs;
  mjVFS* vfs_ptr = nullptr;
  if (!assets.empty()) {
    mj_defaultVFS(&vfs);
    vfs_ptr = &vfs;
    for (const auto& asset : assets) {
      std::string buffer_name = _impl::StripPath(asset.name);
      const int vfs_error = InterceptMjErrors(mj_addBufferVFS)(
          vfs_ptr, buffer_name.c_str(), asset.content, asset.content_size);
      if (vfs_error) {
        mj_deleteVFS(vfs_ptr);
        if (vfs_error == 2) {
          throw py::value_error("Repeated file name in assets dict: " +
                                buffer_name);
        } else {
          throw py::value_error("Asset failed to load: " + buffer_name);
        }
      }
    }
  }

  raw::MjSpec* spec = loadfunc(filename.c_str(), vfs_ptr);
  mj_deleteVFS(vfs_ptr);
  return spec;
}

template <typename T>
struct MjTypeVec {
  MjTypeVec(T* data, int size) : ptr(data), size(size) {}
  T* ptr;
  int size;
};

template <typename T>
void DefineArray(py::module& m, const std::string& typestr) {
  using Class = MjTypeVec<T>;
  py::class_<Class>(m, typestr.c_str())
      .def(py::init([](T* data, int size) { return Class(data, size); }))
      .def("__getitem__",
           [](Class& v, int i) {
             if (i < 0 || i >= v.size) {
               throw py::index_error("Index out of range.");
             }
             return v.ptr[i];
           })
      .def("__setitem__",
           [](Class& v, int i, T c) {
             if (i < 0 || i >= v.size) {
               throw py::index_error("Index out of range.");
             }
             v.ptr[i] = std::move(c);
           })
      .def("__len__", [](Class& v) { return v.size; })
      .def("__iter__", [](Class& v) {
        return py::make_iterator(v.ptr, v.ptr + v.size);
      }, py::keep_alive<0, 1>(), py::return_value_policy::reference_internal);
};

py::list FindAllImpl(raw::MjsBody& body, mjtObj objtype, bool recursive) {
  py::list list;
  raw::MjsElement* el = mjs_firstChild(&body, objtype, recursive);
  std::string error = mjs_getError(mjs_getSpec(body.element));
  if (!el && !error.empty()) {
    throw pybind11::value_error(error);
  }
  while (el) {
    switch (objtype) {
      case mjOBJ_BODY:
        list.append(mjs_asBody(el));
        break;
      case mjOBJ_CAMERA:
        list.append(mjs_asCamera(el));
        break;
      case mjOBJ_FRAME:
        list.append(mjs_asFrame(el));
        break;
      case mjOBJ_GEOM:
        list.append(mjs_asGeom(el));
        break;
      case mjOBJ_JOINT:
        list.append(mjs_asJoint(el));
        break;
      case mjOBJ_LIGHT:
        list.append(mjs_asLight(el));
        break;
      case mjOBJ_SITE:
        list.append(mjs_asSite(el));
        break;
      default:
        // this should never happen
        throw pybind11::value_error(
            "body.find_all supports the types: body, frame, geom, site, "
            "joint, light, camera.");
        break;
    }
    el = mjs_nextChild(&body, el, recursive);
  }
  return list;  // list of pointers, so they can be copied
}

PYBIND11_MODULE(_specs, m) {
  auto structs_m = py::module::import("mujoco._structs");
  py::function mjmodel_from_raw_ptr =
      structs_m.attr("MjModel").attr("_from_model_ptr");
  py::function mjmodel_mjdata_from_spec_ptr =
      structs_m.attr("_recompile_spec_addr");

  py::class_<MjSpec> mjSpec(m, "MjSpec");
  py::class_<raw::MjsElement> mjsElement(m, "MjsElement");
  py::class_<raw::MjsDefault> mjsDefault(m, "MjsDefault");
  py::class_<raw::MjsBody> mjsBody(m, "MjsBody");
  py::class_<raw::MjsFrame> mjsFrame(m, "MjsFrame");
  py::class_<raw::MjsGeom> mjsGeom(m, "MjsGeom");
  py::class_<raw::MjsJoint> mjsJoint(m, "MjsJoint");
  py::class_<raw::MjsLight> mjsLight(m, "MjsLight");
  py::class_<raw::MjsMaterial> mjsMaterial(m, "MjsMaterial");
  py::class_<raw::MjsSite> mjsSite(m, "MjsSite");
  py::class_<raw::MjsMesh> mjsMesh(m, "MjsMesh");
  py::class_<raw::MjsSkin> mjsSkin(m, "MjsSkin");
  py::class_<raw::MjsTexture> mjsTexture(m, "MjsTexture");
  py::class_<raw::MjsText> mjsText(m, "MjsText");
  py::class_<raw::MjsTuple> mjsTuple(m, "MjsTuple");
  py::class_<raw::MjsCamera> mjsCamera(m, "MjsCamera");
  py::class_<raw::MjsFlex> mjsFlex(m, "MjsFlex");
  py::class_<raw::MjsHField> mjsHField(m, "MjsHField");
  py::class_<raw::MjsKey> mjsKey(m, "MjsKey");
  py::class_<raw::MjsNumeric> mjsNumeric(m, "MjsNumeric");
  py::class_<raw::MjsPair> mjsPair(m, "MjsPair");
  py::class_<raw::MjsExclude> mjsExclude(m, "MjsExclude");
  py::class_<raw::MjsEquality> mjsEquality(m, "MjsEquality");
  py::class_<raw::MjsTendon> mjsTendon(m, "MjsTendon");
  py::class_<raw::MjsSensor> mjsSensor(m, "MjsSensor");
  py::class_<raw::MjsActuator> mjsActuator(m, "MjsActuator");
  py::class_<raw::MjsPlugin> mjsPlugin(m, "MjsPlugin");
  py::class_<raw::MjsOrientation> mjsOrientation(m, "MjsOrientation");
  py::class_<raw::MjsWrap> mjsWrap(m, "MjsWrap");
  py::class_<raw::MjOption> mjOption(m, "MjOption");
  py::class_<raw::MjStatistic> mjStatistic(m, "MjStatistic");
  py::class_<raw::MjVisual> mjVisual(m, "MjVisual");
  py::class_<raw::MjVisualHeadlight> mjVisualHeadlight(m, "MjVisualHeadlight");
  py::class_<raw::MjVisualRgba> mjVisualRgba(m, "MjVisualRgba");
  py::class_<raw::MjsCompiler> mjsCompiler(m, "MjsCompiler");
  DefineArray<char>(m, "MjCharVec");
  DefineArray<std::string>(m, "MjStringVec");
  DefineArray<std::byte>(m, "MjByteVec");

  // ============================= MJSPEC =====================================
  mjSpec.def(py::init<>());
  mjSpec.def_property_readonly(
      "parent", [](MjSpec& self) -> MjSpec* { return self.parent; });
  mjSpec.def_static(
      "from_file",
      [](std::string& filename,
         std::optional<std::unordered_map<std::string, py::bytes>>& include,
         std::optional<py::dict>& assets) -> MjSpec {
        const auto files = _impl::ConvertAssetsDict(include);
        raw::MjSpec* spec;
        {
          py::gil_scoped_release no_gil;
          char error[1024];
          spec = LoadSpecFileImpl(
              filename, files,
              [&error](const char* filename, const mjVFS* vfs) {
                return InterceptMjErrors(mj_parseXML)(
                    filename, vfs, error, sizeof(error));
              });
          if (!spec) {
            throw py::value_error(error);
          }
        }
        if (assets.has_value()) {
          return MjSpec(spec, assets.value());
        }
        return MjSpec(spec);
      },
      py::arg("filename"), py::arg("include") = py::none(),
      py::arg("assets") = py::none(), R"mydelimiter(
    Creates a spec from an XML file.

    Parameters
    ----------
    filename : str
        Path to the XML file.
    include : dict, optional
        A dictionary of xml files included by the model. The keys are file names
        and the values are file contents.
    assets : dict, optional
        A dictionary of assets to be used by the spec. The keys are asset names
        and the values are asset contents.
  )mydelimiter",
      py::return_value_policy::move);
  mjSpec.def_static(
      "from_string",
      [](std::string& xml,
         std::optional<std::unordered_map<std::string, py::bytes>>& include,
         std::optional<py::dict>& assets) -> MjSpec {
        auto files = _impl::ConvertAssetsDict(include);
        raw::MjSpec* spec;
        {
          py::gil_scoped_release no_gil;
          std::string model_filename = "model_.xml";
          if (include.has_value()) {
            while (include->find(model_filename) !=
                   include->end()) {
              model_filename =
                  model_filename.substr(0, model_filename.size() - 4) + "_.xml";
            }
          }
          files.emplace_back(
              model_filename.c_str(), xml.c_str(), xml.length());
          char error[1024];
          spec = LoadSpecFileImpl(
              model_filename, files,
              [&error](const char* filename, const mjVFS* vfs) {
                return InterceptMjErrors(mj_parseXML)(
                    filename, vfs, error, sizeof(error));
              });
          if (!spec) {
            throw py::value_error(error);
          }
        }
        if (assets.has_value()) {
          return MjSpec(spec, assets.value());
        }
        return MjSpec(spec);
      },
      py::arg("xml"), py::arg("include") = py::none(),
      py::arg("assets") = py::none(), R"mydelimiter(
    Creates a spec from an XML string.

    Parameters
    ----------
    xml : str
        XML string.
    include : dict, optional
        A dictionary of xml files included by the model. The keys are file names
        and the values are file contents.
    assets : dict, optional
        A dictionary of assets to be used by the spec. The keys are asset names
        and the values are asset contents.
  )mydelimiter",
      py::return_value_policy::move);
  mjSpec.def("recompile", [mjmodel_mjdata_from_spec_ptr](
                              const MjSpec& self, py::object m, py::object d) {
    return mjmodel_mjdata_from_spec_ptr(reinterpret_cast<uintptr_t>(self.ptr),
                                        m, d);
  });
  mjSpec.def("copy", [](const MjSpec& self) -> MjSpec {
    return MjSpec(self);
  });
  mjSpec.def_property_readonly("_address", [](const MjSpec& self) {
    return reinterpret_cast<uintptr_t>(self.ptr);
  });
  mjSpec.def_property(
      "copy_during_attach",
      [](MjSpec& self) {
        throw pybind11::value_error("copy_during_attach can only be set.");
      },
      [](MjSpec& self, bool deepcopy) {
        return mjs_setDeepCopy(self.ptr, deepcopy);
      });
  mjSpec.def_property_readonly(
      "worldbody",
      [](MjSpec& self) -> raw::MjsBody* {
        return mjs_findBody(self.ptr, "world");
      },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "find_default",
      [](MjSpec& self, std::string& classname) -> raw::MjsDefault* {
        return mjs_findDefault(self.ptr, classname.c_str());
      },
      py::return_value_policy::reference_internal);
  mjSpec.def("compile", [mjmodel_from_raw_ptr](MjSpec& self) -> py::object {
    return mjmodel_from_raw_ptr(reinterpret_cast<uintptr_t>(self.Compile()));
  });
  mjSpec.def_property(
      "assets",
      [](MjSpec& self) -> py::dict {
        return self.assets;
      },
      [](MjSpec& self, py::dict& assets) {
        for (auto item : assets) {
          self.assets[item.first] = item.second;
        };
  }, py::return_value_policy::reference_internal);
  mjSpec.def_property(
      "override_assets",
      [](MjSpec& self) -> bool {
        return self.override_assets;
      },
      [](MjSpec& self, bool override_assets) {
        self.override_assets = override_assets;
      });
  mjSpec.def("to_xml", [](MjSpec& self) -> std::string {
    mj_deleteModel(self.Compile());
    std::array<char, 1024> err;
    int size = mj_saveXMLString(self.ptr, nullptr, 0, nullptr, 0);
    std::unique_ptr<char[]> buf(new char[size + 1]);
    buf[0] = '\0';
    err[0] = '\0';
    mj_saveXMLString(self.ptr, buf.get(), size + 1, err.data(), err.size());
    if (err[0] != '\0') {
      throw FatalError(std::string(err.data()));
    }
    return std::string(buf.get());
  });
  mjSpec.def("to_file", [](MjSpec& self, std::string& file) {
    std::array<char, 1024> err;
    if (mj_saveXML(self.ptr, file.c_str(), err.data(), err.size()) < 0) {
      throw FatalError(std::string(err.data()));
    }
  });
  mjSpec.def(
      "add_default",
      [](MjSpec* spec, std::string& classname,
         raw::MjsDefault* parent) -> raw::MjsDefault* {
        return mjs_addDefault(spec->ptr, classname.c_str(), parent);
      },
      py::return_value_policy::reference_internal);
  mjSpec.def("detach_default", [](MjSpec& self, raw::MjsDefault& def) {
    if (mjs_detachDefault(self.ptr, &def) != 0) {
      throw pybind11::value_error(mjs_getError(self.ptr));
    }
  });
  mjSpec.def_property_readonly(
      "default",
      [](MjSpec& self) -> raw::MjsDefault* {
        return mjs_getSpecDefault(self.ptr);
      },
      py::return_value_policy::reference_internal);
  mjSpec.def("detach_body", [](MjSpec& self, raw::MjsBody& body) {
    mjs_detachBody(self.ptr, &body);
  });
  mjSpec.def(
      "attach",
      [](MjSpec& self, MjSpec& child, std::optional<std::string>& prefix,
         std::optional<std::string>& suffix, std::optional<py::object>& site,
         std::optional<py::object>& frame) -> raw::MjsFrame* {
        if (!frame.has_value() && !site.has_value()) {
          throw pybind11::value_error(
              "One of frame or site must be specified.");
        }
        if (frame.has_value() && site.has_value()) {
          throw pybind11::value_error(
              "Only one of frame or site can be specified.");
        }
        const char* p = prefix.has_value() ? prefix.value().c_str() : "";
        const char* s = suffix.has_value() ? suffix.value().c_str() : "";
        raw::MjsElement* attached_frame = nullptr;
        if (frame.has_value()) {
          raw::MjsFrame* frame_ptr = nullptr;
          try {
            frame_ptr = frame->cast<raw::MjsFrame*>();
          } catch (const py::cast_error& e) {
            frame_ptr =
                mjs_findFrame(self.ptr, frame->cast<std::string>().c_str());
          }
          if (!frame_ptr) {
            throw pybind11::value_error("Frame not found.");
          }
          if (mjs_getSpec(frame_ptr->element) != self.ptr) {
            throw pybind11::value_error(
                "Frame spec does not match parent spec.");
          }
          attached_frame =
              mjs_attach(frame_ptr->element, child.ptr->element, p, s);
          if (!attached_frame) {
            throw pybind11::value_error(mjs_getError(self.ptr));
          }
        }
        if (site.has_value()) {
          raw::MjsSite* site_ptr = nullptr;
          try {
            site_ptr = site->cast<raw::MjsSite*>();
          } catch (const py::cast_error& e) {
            site_ptr = mjs_asSite(mjs_findElement(
                self.ptr, mjOBJ_SITE, site->cast<std::string>().c_str()));
          }
          if (!site_ptr) {
            throw pybind11::value_error("Site not found.");
          }
          if (mjs_getSpec(site_ptr->element) != self.ptr) {
            throw pybind11::value_error(
                "Site spec does not match parent spec.");
          }
          attached_frame =
              mjs_attach(site_ptr->element, child.ptr->element, p, s);
          if (!attached_frame) {
            throw pybind11::value_error(mjs_getError(self.ptr));
          }
        }
        for (const auto& asset : child.assets) {
          if (self.assets.contains(asset.first) && !self.override_assets) {
            throw pybind11::value_error("Asset " +
                                        asset.first.cast<std::string>() +
                                        " already exists in parent spec.");
          }
          self.assets[asset.first] = asset.second;
        }
        child.parent = &self;
        return mjs_asFrame(attached_frame);
      },
      py::arg("child"), py::arg("prefix") = py::none(),
      py::arg("suffix") = py::none(), py::arg("site") = py::none(),
      py::arg("frame") = py::none(),
      py::return_value_policy::reference_internal);
  mjSpec.def(
    "activate_plugin",
      [](MjSpec& self, std::string& name) {
        mjs_activatePlugin(self.ptr, name.c_str());
      },
      py::arg("name"),
      py::return_value_policy::reference_internal);
  mjSpec.def_static(
      "resolve_orientation",
      [](bool degree, const MjTypeVec<char>& sequence,
         const raw::MjsOrientation* orientation) -> std::array<double, 4> {
        std::array<double, 4> quat = {0, 0, 0, 0};
        const char* err = mjs_resolveOrientation(quat.data(), degree,
                                                 sequence.ptr, orientation);
        if (err) {
          throw pybind11::value_error(err);
        }
        return quat;
      },
      py::arg("degree"), py::arg("sequence") = py::none(),
      py::arg("orientation"), py::return_value_policy::copy);

  // ============================= MJSBODY =====================================
  mjsBody.def(
      "add_freejoint",
      [](raw::MjsBody& self, py::kwargs kwargs) -> raw::MjsJoint* {
        auto out = mjs_addFreeJoint(&self);
        py::dict kwarg_dict = kwargs;
        for (auto item : kwarg_dict) {
          std::string key = py::str(item.first);
          if (key == "align") {
            try {
              out->align = kwargs["align"].cast<int>();
            } catch (const py::cast_error& e) {
              throw pybind11::value_error("align is the wrong type.");
            }
          } else if (key == "name") {
            try {
              *out->name = kwargs["name"].cast<std::string>();
            } catch (const py::cast_error& e) {
              throw pybind11::value_error("name is the wrong type.");
            }
          } else if (key == "group") {
            try {
              out->group = kwargs["group"].cast<int>();
            } catch (const py::cast_error& e) {
              throw pybind11::value_error("group is the wrong type.");
            }
          } else {
            throw pybind11::type_error(
                "Invalid " + key +
                " keyword argument. Valid options are: align, group, name.");
          }
        }
        return out;
      },
      py::return_value_policy::reference_internal);
  mjsBody.def("set_frame", [](raw::MjsBody& self, raw::MjsFrame& frame) {
    if (mjs_setFrame(self.element, &frame) != 0) {
      throw pybind11::value_error(mjs_getError(mjs_getSpec(self.element)));
    }
  });
  mjsBody.def_property(
      "classname",
      [](raw::MjsBody& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsBody& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });
  mjsBody.def(
      "find_all",
      [](raw::MjsBody& self, mjtObj objtype) -> py::list {
        return FindAllImpl(self, objtype, true);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "find_all",
      [](raw::MjsBody& self, std::string& name) -> py::list {
        mjtObj objtype = mjOBJ_UNKNOWN;
        if (name == "body") {
          objtype = mjOBJ_BODY;
        } else if (name == "frame") {
          objtype = mjOBJ_FRAME;
        } else if (name == "geom") {
          objtype = mjOBJ_GEOM;
        } else if (name == "site") {
          objtype = mjOBJ_SITE;
        } else if (name == "joint") {
          objtype = mjOBJ_JOINT;
        } else if (name == "light") {
          objtype = mjOBJ_LIGHT;
        } else if (name == "camera") {
          objtype = mjOBJ_CAMERA;
        } else {
          throw pybind11::value_error(
              "body.find_all supports the types: body, frame, geom, site, "
              "joint, light, camera.");
        }
        return FindAllImpl(self, objtype, true);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "find_child",
      [](raw::MjsBody& self, std::string& name) -> raw::MjsBody* {
        return mjs_findChild(&self, name.c_str());
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_body",
      [](raw::MjsBody& self) -> raw::MjsBody* {
        return mjs_asBody(mjs_firstChild(&self, mjOBJ_BODY, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_body",
      [](raw::MjsBody& self, raw::MjsBody& child) -> raw::MjsBody* {
        return mjs_asBody(mjs_nextChild(&self, child.element, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def_property_readonly(
      "bodies",
      [](raw::MjsBody& self) -> py::list {
        return FindAllImpl(self, mjOBJ_BODY, false);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_camera",
      [](raw::MjsBody& self) -> raw::MjsCamera* {
        return mjs_asCamera(mjs_firstChild(&self, mjOBJ_CAMERA, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_camera",
      [](raw::MjsBody& self, raw::MjsCamera& child) -> raw::MjsCamera* {
        return mjs_asCamera(mjs_nextChild(&self, child.element, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def_property_readonly(
      "cameras",
      [](raw::MjsBody& self) -> py::list {
        return FindAllImpl(self, mjOBJ_CAMERA, false);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_light",
      [](raw::MjsBody& self) -> raw::MjsLight* {
        return mjs_asLight(mjs_firstChild(&self, mjOBJ_LIGHT, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_light",
      [](raw::MjsBody& self, raw::MjsLight& child) -> raw::MjsLight* {
        return mjs_asLight(mjs_nextChild(&self, child.element, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def_property_readonly(
      "lights",
      [](raw::MjsBody& self) -> py::list {
        return FindAllImpl(self, mjOBJ_LIGHT, false);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_joint",
      [](raw::MjsBody& self) -> raw::MjsJoint* {
        return mjs_asJoint(mjs_firstChild(&self, mjOBJ_JOINT, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_joint",
      [](raw::MjsBody& self, raw::MjsJoint& child) -> raw::MjsJoint* {
        return mjs_asJoint(mjs_nextChild(&self, child.element, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def_property_readonly(
      "joints",
      [](raw::MjsBody& self) -> py::list {
        return FindAllImpl(self, mjOBJ_JOINT, false);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_geom",
      [](raw::MjsBody& self) -> raw::MjsGeom* {
        return mjs_asGeom(mjs_firstChild(&self, mjOBJ_GEOM, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_geom",
      [](raw::MjsBody& self, raw::MjsGeom& child) -> raw::MjsGeom* {
        return mjs_asGeom(mjs_nextChild(&self, child.element, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def_property_readonly(
      "geoms",
      [](raw::MjsBody& self) -> py::list {
        return FindAllImpl(self, mjOBJ_GEOM, false);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_site",
      [](raw::MjsBody& self) -> raw::MjsSite* {
        return mjs_asSite(mjs_firstChild(&self, mjOBJ_SITE, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_site",
      [](raw::MjsBody& self, raw::MjsSite& child) -> raw::MjsSite* {
        return mjs_asSite(mjs_nextChild(&self, child.element, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def_property_readonly(
      "sites",
      [](raw::MjsBody& self) -> py::list {
        return FindAllImpl(self, mjOBJ_SITE, false);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_frame",
      [](raw::MjsBody& self) -> raw::MjsFrame* {
        return mjs_asFrame(mjs_firstChild(&self, mjOBJ_FRAME, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_frame",
      [](raw::MjsBody& self, raw::MjsFrame& child) -> raw::MjsFrame* {
        return mjs_asFrame(mjs_nextChild(&self, child.element, false));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def_property_readonly(
      "frames",
      [](raw::MjsBody& self) -> py::list {
        return FindAllImpl(self, mjOBJ_FRAME, false);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def_property_readonly(
      "parent",
      [](raw::MjsBody& self) -> raw::MjsBody* {
        return mjs_getParent(self.element);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "attach_frame",
      [](raw::MjsBody& self, raw::MjsFrame& frame,
         std::optional<std::string>& prefix,
         std::optional<std::string>& suffix) -> raw::MjsFrame* {
        const char* p = prefix.has_value() ? prefix.value().c_str() : "";
        const char* s = suffix.has_value() ? suffix.value().c_str() : "";
        auto new_frame = mjs_attach(self.element, frame.element, p, s);
        if (!new_frame) {
          throw pybind11::value_error(mjs_getError(mjs_getSpec(self.element)));
        }
        return mjs_asFrame(new_frame);
      },
      py::arg("frame"), py::arg("prefix") = py::none(),
      py::arg("suffix") = py::none(),
      py::return_value_policy::reference_internal);
  mjsBody.def(
    "to_frame",
      [](raw::MjsBody* self) -> raw::MjsFrame* {
        raw::MjsFrame* frame = mjs_bodyToFrame(&self);
        if (!frame) {
          throw pybind11::value_error(mjs_getError(mjs_getSpec(self->element)));
        }
        // TODO: set the body to py::none
        return frame;
      },
      py::return_value_policy::reference_internal);
  mjsBody.def_property_readonly(
    "frame",
      [](raw::MjsBody* self) -> raw::MjsFrame* {
        return mjs_getFrame(self->element);
      },
      py::return_value_policy::reference_internal);


  // ============================= MJSFRAME ====================================
  mjsFrame.def("delete", [](raw::MjsFrame& self) { mjs_delete(self.element); });
  mjsFrame.def("set_frame", [](raw::MjsFrame& self, raw::MjsFrame& frame) {
    if (mjs_setFrame(self.element, &frame) != 0) {
      throw pybind11::value_error(mjs_getError(mjs_getSpec(self.element)));
    }
  });
  mjsFrame.def_property_readonly(
      "parent",
      [](raw::MjsFrame& self) -> raw::MjsBody* {
        return mjs_getParent(self.element);
      },
      py::return_value_policy::reference_internal);
  mjsFrame.def(
      "attach_body",
      [](raw::MjsFrame& self, raw::MjsBody& body,
         std::optional<std::string>& prefix,
         std::optional<std::string>& suffix) -> raw::MjsBody* {
        const char* p = prefix.has_value() ? prefix.value().c_str() : "";
        const char* s = suffix.has_value() ? suffix.value().c_str() : "";
        auto new_body = mjs_attach(self.element, body.element, p, s);
        if (!new_body) {
          throw pybind11::value_error(
              mjs_getError(mjs_getSpec(self.element)));
        }
        return mjs_asBody(new_body);
      },
      py::arg("body"), py::arg("prefix") = py::none(),
      py::arg("suffix") = py::none(),
      py::return_value_policy::reference_internal);
  mjsFrame.def_property_readonly(
      "frame",
      [](raw::MjsFrame& self) -> raw::MjsFrame* {
        return mjs_getFrame(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSGEOM =====================================
  mjsGeom.def("delete", [](raw::MjsGeom& self) { mjs_delete(self.element); });
  mjsGeom.def("set_frame", [](raw::MjsGeom& self, raw::MjsFrame& frame) {
    if (mjs_setFrame(self.element, &frame) != 0) {
      throw pybind11::value_error(mjs_getError(mjs_getSpec(self.element)));
    }
  });
  mjsGeom.def_property_readonly(
      "parent",
      [](raw::MjsGeom& self) -> raw::MjsBody* {
        return mjs_getParent(self.element);
      },
      py::return_value_policy::reference_internal);
  mjsGeom.def_property(
      "classname",
      [](raw::MjsGeom& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsGeom& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });
  mjsGeom.def_property_readonly(
      "frame",
      [](raw::MjsGeom& self) -> raw::MjsFrame* {
        return mjs_getFrame(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSJOINT ====================================
  mjsJoint.def("delete", [](raw::MjsJoint& self) { mjs_delete(self.element); });
  mjsJoint.def("set_frame", [](raw::MjsJoint& self, raw::MjsFrame& frame) {
    if (mjs_setFrame(self.element, &frame) != 0) {
      throw pybind11::value_error(mjs_getError(mjs_getSpec(self.element)));
    }
  });
  mjsJoint.def_property_readonly(
      "parent",
      [](raw::MjsJoint& self) -> raw::MjsBody* {
        return mjs_getParent(self.element);
      },
      py::return_value_policy::reference_internal);
  mjsJoint.def_property(
      "classname",
      [](raw::MjsJoint& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsJoint& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });
  mjsJoint.def_property_readonly(
      "frame",
      [](raw::MjsJoint& self) -> raw::MjsFrame* {
        return mjs_getFrame(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSSITE =====================================
  mjsSite.def("delete", [](raw::MjsSite& self) { mjs_delete(self.element); });
  mjsSite.def("set_frame", [](raw::MjsSite& self, raw::MjsFrame& frame) {
    if (mjs_setFrame(self.element, &frame) != 0) {
      throw pybind11::value_error(mjs_getError(mjs_getSpec(self.element)));
    }
  });
  mjsSite.def_property_readonly(
      "parent",
      [](raw::MjsSite& self) -> raw::MjsBody* {
        return mjs_getParent(self.element);
      },
      py::return_value_policy::reference_internal);
  mjsSite.def_property(
      "classname",
      [](raw::MjsSite& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsSite& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });
  mjsSite.def(
      "attach_body",
      [](raw::MjsSite& self, raw::MjsBody& body,
         std::optional<std::string>& prefix,
         std::optional<std::string>& suffix) -> raw::MjsBody* {
        const char* p = prefix.has_value() ? prefix.value().c_str() : "";
        const char* s = suffix.has_value() ? suffix.value().c_str() : "";
        auto new_body = mjs_attach(self.element, body.element, p, s);
        if (!new_body) {
          throw pybind11::value_error(
              mjs_getError(mjs_getSpec(self.element)));
        }
        return mjs_asBody(new_body);
      },
      py::arg("body"), py::arg("prefix") = py::none(),
      py::arg("suffix") = py::none(),
      py::return_value_policy::reference_internal);
  mjsSite.def_property_readonly(
      "frame",
      [](raw::MjsSite& self) -> raw::MjsFrame* {
        return mjs_getFrame(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSCAMERA ===================================
  mjsCamera.def("delete",
                [](raw::MjsCamera& self) { mjs_delete(self.element); });
  mjsCamera.def("set_frame", [](raw::MjsCamera& self, raw::MjsFrame& frame) {
    if (mjs_setFrame(self.element, &frame) != 0) {
      throw pybind11::value_error(mjs_getError(mjs_getSpec(self.element)));
    }
  });
  mjsCamera.def_property_readonly(
      "parent",
      [](raw::MjsCamera& self) -> raw::MjsBody* {
        return mjs_getParent(self.element);
      },
      py::return_value_policy::reference_internal);
  mjsCamera.def_property(
      "classname",
      [](raw::MjsCamera& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsCamera& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });
  mjsCamera.def_property_readonly(
      "frame",
      [](raw::MjsCamera& self) -> raw::MjsFrame* {
        return mjs_getFrame(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSLIGHT ====================================
  mjsLight.def("delete", [](raw::MjsLight& self) { mjs_delete(self.element); });
  mjsLight.def("set_frame", [](raw::MjsLight& self, raw::MjsFrame& frame) {
    if (mjs_setFrame(self.element, &frame) != 0) {
      throw pybind11::value_error(mjs_getError(mjs_getSpec(self.element)));
    }
  });
  mjsLight.def_property_readonly(
      "parent",
      [](raw::MjsLight& self) -> raw::MjsBody* {
        return mjs_getParent(self.element);
      },
      py::return_value_policy::reference_internal);
  mjsLight.def_property(
      "classname",
      [](raw::MjsLight& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsLight& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });
  mjsLight.def_property_readonly(
      "frame",
      [](raw::MjsLight& self) -> raw::MjsFrame* {
        return mjs_getFrame(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSMATERIAL =================================
  mjsMaterial.def("delete",
                  [](raw::MjsMaterial& self) { mjs_delete(self.element); });
  mjsMaterial.def_property(
      "classname",
      [](raw::MjsMaterial& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsMaterial& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });

  // ============================= MJSMESH =====================================
  mjsMesh.def("delete", [](raw::MjsMesh& self) { mjs_delete(self.element); });
  mjsMesh.def_property(
      "classname",
      [](raw::MjsMesh& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsMesh& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });

  // ============================= MJSPAIR =====================================
  mjsPair.def("delete", [](raw::MjsPair& self) { mjs_delete(self.element); });
  mjsPair.def_property(
      "classname",
      [](raw::MjsPair& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsPair& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });

  // ============================= MJSEQUAL ====================================
  mjsEquality.def("delete",
                  [](raw::MjsEquality& self) { mjs_delete(self.element); });
  mjsEquality.def_property(
      "classname",
      [](raw::MjsEquality& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsEquality& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });

  // ============================= MJSACTUATOR =================================
  mjsActuator.def("delete",
                  [](raw::MjsActuator& self) { mjs_delete(self.element); });
  mjsActuator.def_property(
      "classname",
      [](raw::MjsActuator& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      [](raw::MjsActuator& self, raw::MjsDefault& default_) -> void {
        mjs_setDefault(self.element, &default_);
      });
  mjsActuator.def("set_to_motor", [](raw::MjsActuator* self) {
    std::string err = mjs_setToMotor(self);
    if (!err.empty()) {
      throw pybind11::value_error(err);
    }
  });
  mjsActuator.def(
      "set_to_position",
      [](raw::MjsActuator* self, double kp, double kv, double dampratio,
         double timeconst, bool inheritrange) {
        std::string err = mjs_setToPosition(
            self, kp, kv == -1 ? nullptr : &kv,
            dampratio == -1 ? nullptr : &dampratio,
            timeconst == -1 ? nullptr : &timeconst, inheritrange);
        if (!err.empty()) {
          throw pybind11::value_error(err);
        }
      },
      py::arg("kp"), py::arg("kv") = -1, py::arg("dampratio") = -1,
      py::arg("timeconst") = -1, py::arg("inheritrange") = false);
  mjsActuator.def(
      "set_to_intvelocity",
      [](raw::MjsActuator* self, double kp, double kv, double dampratio,
         double timeconst, bool inheritrange) {
        std::string err = mjs_setToIntVelocity(
            self, kp, kv == -1 ? nullptr : &kv,
            dampratio == -1 ? nullptr : &dampratio,
            timeconst == -1 ? nullptr : &timeconst, inheritrange);
        if (!err.empty()) {
          throw pybind11::value_error(err);
        }
      },
      py::arg("kp"), py::arg("kv") = -1, py::arg("dampratio") = -1,
      py::arg("timeconst") = -1, py::arg("inheritrange") = false);
  mjsActuator.def(
      "set_to_velocity",
      [](raw::MjsActuator* self, double kv) {
        std::string err = mjs_setToVelocity(self, kv);
        if (!err.empty()) {
          throw pybind11::value_error(err);
        }
      },
      py::arg("kv"));
  mjsActuator.def(
      "set_to_damper",
      [](raw::MjsActuator* self, double kv) {
        std::string err = mjs_setToDamper(self, kv);
        if (!err.empty()) {
          throw pybind11::value_error(err);
        }
      },
      py::arg("kv"));
  mjsActuator.def(
      "set_to_cylinder",
      [](raw::MjsActuator* self, double timeconst, double bias, double area,
         double diameter) {
        std::string err =
            mjs_setToCylinder(self, timeconst, bias, area, diameter);
        if (!err.empty()) {
          throw pybind11::value_error(err);
        }
      },
      py::arg("timeconst"), py::arg("bias"), py::arg("area"),
      py::arg("diameter") = -1);
  mjsActuator.def(
      "set_to_muscle",
      [](raw::MjsActuator* self, double timeconst[2], double tausmooth,
         double range[2], double force, double scale, double lmin, double lmax,
         double vmax, double fpmax, double fvmax) {
        std::string err =
            mjs_setToMuscle(self, timeconst, tausmooth, range, force, scale,
                            lmin, lmax, vmax, fpmax, fvmax);
        if (!err.empty()) {
          throw pybind11::value_error(err);
        }
      },
      py::arg("timeconst") = -1, py::arg("tausmooth"),
      py::arg("range") = std::array<double, 2>{-1, -1}, py::arg("force") = -1,
      py::arg("scale") = -1, py::arg("lmin") = -1, py::arg("lmax") = -1,
      py::arg("vmax") = -1, py::arg("fpmax") = -1, py::arg("fvmax") = -1);
  mjsActuator.def(
      "set_to_adhesion",
      [](raw::MjsActuator* self, double gain) {
        std::string err = mjs_setToAdhesion(self, gain);
        if (!err.empty()) {
          throw pybind11::value_error(err);
        }
      },
      py::arg("gain"));

  // ============================= MJSTENDON ===================================
  mjsTendon.def("delete",
                [](raw::MjsTendon& self) { mjs_delete(self.element); });
  mjsTendon.def(
      "default",
      [](raw::MjsTendon& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);
  mjsTendon.def(
      "wrap_site",
      [](raw::MjsTendon& self, std::string& name) {
        return mjs_wrapSite(&self, name.c_str());
      },
      py::return_value_policy::reference_internal);
  mjsTendon.def(
      "wrap_geom",
      [](raw::MjsTendon& self, std::string& name, std::string& sidesite) {
        return mjs_wrapGeom(&self, name.c_str(), sidesite.c_str());
      },
      py::return_value_policy::reference_internal);
  mjsTendon.def(
      "wrap_joint",
      [](raw::MjsTendon& self, std::string& name, double coef) {
        return mjs_wrapJoint(&self, name.c_str(), coef);
      },
      py::return_value_policy::reference_internal);
  mjsTendon.def(
      "wrap_pulley",
      [](raw::MjsTendon& self, double divisor) {
        return mjs_wrapPulley(&self, divisor);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSSENSOR ===================================
  mjsSensor.def("delete",
                [](raw::MjsSensor& self) { mjs_delete(self.element); });

  // ============================= MJSFLEX =====================================
  mjsFlex.def("delete", [](raw::MjsFlex& self) { mjs_delete(self.element); });

  // ============================= MJSHFIELD ===================================
  mjsHField.def("delete",
                [](raw::MjsHField& self) { mjs_delete(self.element); });

  // ============================= MJSSKIN =====================================
  mjsSkin.def("delete", [](raw::MjsSkin& self) { mjs_delete(self.element); });

  // ============================= MJSTEXTURE ==================================
  mjsTexture.def("delete",
                 [](raw::MjsTexture& self) { mjs_delete(self.element); });

  // ============================= MJSKEY ======================================
  mjsKey.def("delete", [](raw::MjsKey& self) { mjs_delete(self.element); });

  // ============================= MJSTEXT =====================================
  mjsText.def("delete", [](raw::MjsText& self) { mjs_delete(self.element); });

  // ============================= MJSNUMERIC ==================================
  mjsNumeric.def("delete",
                 [](raw::MjsNumeric& self) { mjs_delete(self.element); });

  // ============================= MJSEXCLUDE ==================================
  mjsExclude.def("delete",
                 [](raw::MjsExclude& self) { mjs_delete(self.element); });

  // ============================= MJSTUPLE ====================================
  mjsTuple.def("delete", [](raw::MjsTuple& self) { mjs_delete(self.element); });

  // ============================= MJSPLUGIN ===================================
  mjsPlugin.def_property(
      "id",
      [](raw::MjsPlugin& self) -> int { return mjs_getId(self.element); },
      [](raw::MjsPlugin& self, raw::MjsPlugin* other) {
        self.element = other->element;
      });
  mjsPlugin.def("delete",
                [](raw::MjsPlugin& self) { mjs_delete(self.element); });
  mjsPlugin.def_property(
      "config",
      [](raw::MjsPlugin& self) -> py::dict {
        const std::map<std::string, std::string, std::less<>>* config_attribs =
            static_cast<const std::map<std::string, std::string, std::less<>>*>(
                mjs_getPluginAttributes(&self));
        py::dict config;
        for (const auto& [key, value] : *config_attribs) {
          config[py::str(key)] = value;
        }
        return config;
      },
      [](raw::MjsPlugin& self, py::dict& config) {
        std::map<std::string, std::string, std::less<>> config_attribs;
        for (const auto& [key, value] : config) {
          std::string key_str = key.cast<std::string>();
          if (config_attribs.find(key_str) != config_attribs.end()) {
            throw pybind11::value_error("Duplicate config key: " + key_str);
          }
          config_attribs[key_str] = value.cast<std::string>();
        }
        mjs_setPluginAttributes(&self, &config_attribs);
      },
      py::return_value_policy::reference_internal);
  // ============================= MJVISUAL ====================================
  mjVisual.def_property(
      "global_",
      [](raw::MjVisual& self) -> raw::MjVisualGlobal& { return self.global; },
      [](raw::MjVisual& self, raw::MjVisualGlobal& value) {
        self.global = value;
      });

#include "specs.cc.inc"
}  // PYBIND11_MODULE // NOLINT
}  // namespace mujoco::python
