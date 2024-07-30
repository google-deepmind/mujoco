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
#include <memory>
#include <optional>
#include <string>
#include <string_view>  // IWYU pragma: keep
#include <vector>       // IWYU pragma: keep

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <mujoco/mjspec.h>  // IWYU pragma: keep
#include <mujoco/mujoco.h>
#include "errors.h"
#include "indexers.h"  // IWYU pragma: keep
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

struct MjSpec {
  MjSpec() : ptr(mj_makeSpec()) {}
  ~MjSpec() { mj_deleteSpec(ptr); }
  raw::MjSpec* ptr;
};

PYBIND11_MODULE(_specs, m) {
  auto structs_m = py::module::import("mujoco._structs");
  py::function mjmodel_from_spec_ptr =
      structs_m.attr("MjModel").attr("_from_spec_ptr");
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

  // ============================= MJSPEC =====================================
  mjSpec.def(py::init<>());
  mjSpec.def("recompile", [mjmodel_mjdata_from_spec_ptr](
                              const MjSpec& self, py::object m, py::object d) {
    return mjmodel_mjdata_from_spec_ptr(reinterpret_cast<uintptr_t>(self.ptr),
                                        m, d);
  });
  mjSpec.def(
      "copy",
      [](const MjSpec& self) -> raw::MjSpec* { return mj_copySpec(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "worldbody",
      [](MjSpec& self) -> raw::MjsBody* {
        return mjs_findBody(self.ptr, "world");
      },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "find_body",
      [](MjSpec& self, std::string& name) -> raw::MjsBody* {
        return mjs_findBody(self.ptr, name.c_str());
      },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "find_mesh",
      [](MjSpec& self, std::string& name) -> raw::MjsMesh* {
        return mjs_findMesh(self.ptr, name.c_str());
      },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "find_frame",
      [](MjSpec& self, std::string& name) -> raw::MjsFrame* {
        return mjs_findFrame(self.ptr, name.c_str());
      },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "find_keyframe",
      [](MjSpec& self, std::string& name) -> raw::MjsKey* {
        return mjs_findKeyframe(self.ptr, name.c_str());
      },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "find_default",
      [](MjSpec& self, std::string& classname) -> raw::MjsDefault* {
        return mjs_findDefault(self.ptr, classname.c_str());
      },
      py::return_value_policy::reference_internal);
  mjSpec.def("compile", [mjmodel_from_spec_ptr](MjSpec& self) {
    return mjmodel_from_spec_ptr(reinterpret_cast<uintptr_t>(self.ptr));
  });
  mjSpec.def(
      "compile",
      [mjmodel_from_spec_ptr](MjSpec& self, py::dict& assets) -> py::object {
        mjVFS vfs;
        mj_defaultVFS(&vfs);
        for (auto item : assets) {
          std::string buffer = py::cast<std::string>(item.second);
          mj_addBufferVFS(&vfs, py::cast<std::string>(item.first).c_str(),
                          buffer.c_str(), buffer.size());
        };
        auto model =
            mjmodel_from_spec_ptr(reinterpret_cast<uintptr_t>(self.ptr),
                                  reinterpret_cast<uintptr_t>(&vfs));
        mj_deleteVFS(&vfs);
        return model;
      }, R"mydelimiter(
    Compiles the spec and returns the compiled model.

    Parameters
    ----------
    assets : dict, optional
        A dictionary of assets to be used by the spec. The keys are asset names
        and the values are asset contents.
  )mydelimiter");
  mjSpec.def(
      "copy_back",
      [](MjSpec& self, raw::MjModel& model) {
        return mj_copyBack(self.ptr, &model);
      },
      py::return_value_policy::reference_internal);
  mjSpec.def("to_xml", [](MjSpec& self) -> std::string {
    int size = mj_saveXMLString(self.ptr, nullptr, 0, nullptr, 0);
    std::unique_ptr<char[]> buf(new char[size + 1]);
    std::array<char, 1024> err;
    buf[0] = '\0';
    err[0] = '\0';
    mj_saveXMLString(self.ptr, buf.get(), size + 1, err.data(), err.size());
    if (err[0] != '\0') {
      throw FatalError(std::string(err.data()));
    }
    return std::string(buf.get());
  });
  mjSpec.def(
      "from_file",
      [](MjSpec& self, std::string& filename,
         std::optional<py::dict>& assets) -> void {
        mjVFS vfs;
        mj_defaultVFS(&vfs);
        if (assets.has_value()) {
          for (auto item : assets.value()) {
            std::string buffer = py::cast<std::string>(item.second);
            mj_addBufferVFS(&vfs, py::cast<std::string>(item.first).c_str(),
                            buffer.c_str(), buffer.size());
          };
        }
        std::array<char, 1024> err;
        err[0] = '\0';
        mj_deleteSpec(self.ptr);
        self.ptr = mj_parseXML(filename.c_str(), &vfs, err.data(), err.size());
        mj_deleteVFS(&vfs);
        if (!self.ptr) {
          throw FatalError(std::string(err.data()));
        }
      },
      py::arg("filename"),
      py::arg("assets") = py::none(), R"mydelimiter(
    Creates a spec from an XML file.

    Parameters
    ----------
    filename : str
        Path to the XML file.
    assets : dict, optional
        A dictionary of assets to be used by the spec. The keys are asset names
        and the values are asset contents.
  )mydelimiter");
  mjSpec.def(
      "from_string",
      [](MjSpec& self, std::string& xml,
         std::optional<py::dict>& assets) -> void {
        mjVFS vfs;
        mj_defaultVFS(&vfs);
        if (assets.has_value()) {
          for (auto item : assets.value()) {
            std::string buffer = py::cast<std::string>(item.second);
            mj_addBufferVFS(&vfs, py::cast<std::string>(item.first).c_str(),
                            buffer.c_str(), buffer.size());
          };
        }
        std::array<char, 1024> err;
        err[0] = '\0';
        mj_deleteSpec(self.ptr);
        self.ptr = mj_parseXMLString(xml.c_str(), &vfs, err.data(), err.size());
        mj_deleteVFS(&vfs);
        if (!self.ptr) {
          throw FatalError(std::string(err.data()));
        }
      },
      py::arg("xml"),
      py::arg("assets") = py::none(), R"mydelimiter(
    Creates a spec from an XML string.

    Parameters
    ----------
    xml : str
        XML string.
    assets : dict, optional
        A dictionary of assets to be used by the spec. The keys are asset names
        and the values are asset contents.
  )mydelimiter");
  mjSpec.def(
      "add_default",
      [](MjSpec* spec, std::string& classname,
         raw::MjsDefault* parent) -> raw::MjsDefault* {
        return mjs_addDefault(spec->ptr, classname.c_str(), parent);
      },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "default",
      [](MjSpec& self) -> raw::MjsDefault* {
        return mjs_getSpecDefault(self.ptr);
      },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_material",
      [](MjSpec& self, raw::MjsDefault* default_) -> raw::MjsMaterial* {
        return mjs_addMaterial(self.ptr, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_mesh",
      [](MjSpec& self, raw::MjsDefault* default_) -> raw::MjsMesh* {
        return mjs_addMesh(self.ptr, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_skin",
      [](MjSpec& self) -> raw::MjsSkin* { return mjs_addSkin(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_texture",
      [](MjSpec& self) -> raw::MjsTexture* { return mjs_addTexture(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_text",
      [](MjSpec& self) -> raw::MjsText* { return mjs_addText(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_tuple",
      [](MjSpec& self) -> raw::MjsTuple* { return mjs_addTuple(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_flex",
      [](MjSpec& self) -> raw::MjsFlex* { return mjs_addFlex(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_hfield",
      [](MjSpec& self) -> raw::MjsHField* { return mjs_addHField(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_key",
      [](MjSpec& self) -> raw::MjsKey* { return mjs_addKey(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_numeric",
      [](MjSpec& self) -> raw::MjsNumeric* { return mjs_addNumeric(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_pair",
      [](MjSpec& self, raw::MjsDefault* default_) -> raw::MjsPair* {
        return mjs_addPair(self.ptr, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_exclude",
      [](MjSpec& self) -> raw::MjsExclude* { return mjs_addExclude(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_equality",
      [](MjSpec& self, raw::MjsDefault* default_) -> raw::MjsEquality* {
        return mjs_addEquality(self.ptr, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_tendon",
      [](MjSpec& self, raw::MjsDefault* default_) -> raw::MjsTendon* {
        return mjs_addTendon(self.ptr, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_sensor",
      [](MjSpec& self) -> raw::MjsSensor* { return mjs_addSensor(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_actuator",
      [](MjSpec& self, raw::MjsDefault* default_) -> raw::MjsActuator* {
        return mjs_addActuator(self.ptr, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjSpec.def(
      "add_plugin",
      [](MjSpec& self) -> raw::MjsPlugin* { return mjs_addPlugin(self.ptr); },
      py::return_value_policy::reference_internal);
  mjSpec.def("detach_body", [](MjSpec& self, raw::MjsBody& body) {
    mjs_detachBody(self.ptr, &body);
  });
  mjSpec.def_property_readonly(
      "actuators",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_ACTUATOR);
        while (el) {
          list.append(mjs_asActuator(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "sensors",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_SENSOR);
        while (el) {
          list.append(mjs_asSensor(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "flexes",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_FLEX);
        while (el) {
          list.append(mjs_asFlex(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "pairs",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_PAIR);
        while (el) {
          list.append(mjs_asPair(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "equality",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_EQUALITY);
        while (el) {
          list.append(mjs_asEquality(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "excludes",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_EXCLUDE);
        while (el) {
          list.append(mjs_asExclude(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "tendons",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_TENDON);
        while (el) {
          list.append(mjs_asTendon(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "numeric",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_NUMERIC);
        while (el) {
          list.append(mjs_asNumeric(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "text",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_TEXT);
        while (el) {
          list.append(mjs_asText(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "tuple",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_TUPLE);
        while (el) {
          list.append(mjs_asTuple(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "key",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_KEY);
        while (el) {
          list.append(mjs_asKey(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "mesh",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_MESH);
        while (el) {
          list.append(mjs_asMesh(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "hfield",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_HFIELD);
        while (el) {
          list.append(mjs_asHField(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "skin",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_SKIN);
        while (el) {
          list.append(mjs_asSkin(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "texture",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_TEXTURE);
        while (el) {
          list.append(mjs_asTexture(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);
  mjSpec.def_property_readonly(
      "material",
      [](MjSpec& self) -> py::list {
        py::list list;
        raw::MjsElement* el = mjs_firstElement(self.ptr, mjOBJ_MATERIAL);
        while (el) {
          list.append(mjs_asMaterial(el));
          el = mjs_nextElement(self.ptr, el);
        }
        return list;
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSBODY =====================================
  mjsBody.def_property_readonly(
      "id", [](raw::MjsBody& self) -> int { return mjs_getId(self.element); });
  mjsBody.def(
      "add_body",
      [](raw::MjsBody& self, raw::MjsDefault* default_) -> raw::MjsBody* {
        return mjs_addBody(&self, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "add_frame",
      [](raw::MjsBody& self, raw::MjsFrame* parentframe_) -> raw::MjsFrame* {
        return mjs_addFrame(&self, parentframe_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "add_geom",
      [](raw::MjsBody& self, raw::MjsDefault* default_) -> raw::MjsGeom* {
        return mjs_addGeom(&self, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "add_joint",
      [](raw::MjsBody& self, raw::MjsDefault* default_) -> raw::MjsJoint* {
        return mjs_addJoint(&self, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "add_freejoint",
      [](raw::MjsBody& self) -> raw::MjsJoint* {
        return mjs_addFreeJoint(&self);
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "add_light",
      [](raw::MjsBody& self, raw::MjsDefault* default_) -> raw::MjsLight* {
        return mjs_addLight(&self, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "add_site",
      [](raw::MjsBody& self, raw::MjsDefault* default_) -> raw::MjsSite* {
        return mjs_addSite(&self, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "add_camera",
      [](raw::MjsBody& self, raw::MjsDefault* default_) -> raw::MjsCamera* {
        return mjs_addCamera(&self, default_);
      },
      py::arg_v("default", nullptr),
      py::return_value_policy::reference_internal);
  mjsBody.def("set_frame",
              [](raw::MjsBody& self, raw::MjsFrame& frame) -> void {
                mjs_setFrame(self.element, &frame);
              });
  mjsBody.def("set_default",
              [](raw::MjsBody& self, raw::MjsDefault& default_) -> void {
                mjs_setDefault(self.element, &default_);
              });
  mjsBody.def(
      "default",
      [](raw::MjsBody& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
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
        return mjs_asBody(mjs_firstChild(&self, mjOBJ_BODY));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_body",
      [](raw::MjsBody& self, raw::MjsBody& child) -> raw::MjsBody* {
        return mjs_asBody(mjs_nextChild(&self, child.element));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_camera",
      [](raw::MjsBody& self) -> raw::MjsCamera* {
        return mjs_asCamera(mjs_firstChild(&self, mjOBJ_CAMERA));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_camera",
      [](raw::MjsBody& self, raw::MjsCamera& child) -> raw::MjsCamera* {
        return mjs_asCamera(mjs_nextChild(&self, child.element));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_light",
      [](raw::MjsBody& self) -> raw::MjsLight* {
        return mjs_asLight(mjs_firstChild(&self, mjOBJ_LIGHT));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_light",
      [](raw::MjsBody& self, raw::MjsLight& child) -> raw::MjsLight* {
        return mjs_asLight(mjs_nextChild(&self, child.element));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_joint",
      [](raw::MjsBody& self) -> raw::MjsJoint* {
        return mjs_asJoint(mjs_firstChild(&self, mjOBJ_JOINT));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_joint",
      [](raw::MjsBody& self, raw::MjsJoint& child) -> raw::MjsJoint* {
        return mjs_asJoint(mjs_nextChild(&self, child.element));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_geom",
      [](raw::MjsBody& self) -> raw::MjsGeom* {
        return mjs_asGeom(mjs_firstChild(&self, mjOBJ_GEOM));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_geom",
      [](raw::MjsBody& self, raw::MjsGeom& child) -> raw::MjsGeom* {
        return mjs_asGeom(mjs_nextChild(&self, child.element));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_site",
      [](raw::MjsBody& self) -> raw::MjsSite* {
        return mjs_asSite(mjs_firstChild(&self, mjOBJ_SITE));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_site",
      [](raw::MjsBody& self, raw::MjsSite& child) -> raw::MjsSite* {
        return mjs_asSite(mjs_nextChild(&self, child.element));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "first_frame",
      [](raw::MjsBody& self) -> raw::MjsFrame* {
        return mjs_asFrame(mjs_firstChild(&self, mjOBJ_FRAME));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "next_frame",
      [](raw::MjsBody& self, raw::MjsFrame& child) -> raw::MjsFrame* {
        return mjs_asFrame(mjs_nextChild(&self, child.element));
      },
      py::return_value_policy::reference_internal);
  mjsBody.def(
      "spec",
      [](raw::MjsBody& self) -> raw::MjSpec* { return mjs_getSpec(&self); },
      py::return_value_policy::reference_internal);
  mjsBody.def("attach_frame",
              [](raw::MjsBody& self, raw::MjsFrame& frame, std::string& prefix,
                 std::string& suffix) -> void {
                mjs_attachFrame(&self, &frame, prefix.c_str(), suffix.c_str());
              });

  // ============================= MJSFRAME ====================================
  mjsFrame.def_property_readonly(
      "id", [](raw::MjsFrame& self) -> int { return mjs_getId(self.element); });
  mjsFrame.def("delete", [](raw::MjsFrame& self) { mjs_delete(self.element); });
  mjsFrame.def("set_frame", [](raw::MjsFrame& self, raw::MjsFrame& frame) {
    mjs_setFrame(self.element, &frame);
  });
  mjsFrame.def("attach_body", [](raw::MjsFrame& self, raw::MjsBody& body,
                                 std::string& prefix, std::string& suffix) {
    mjs_attachBody(&self, &body, prefix.c_str(), suffix.c_str());
  });

  // ============================= MJSGEOM =====================================
  mjsGeom.def_property_readonly(
      "id", [](raw::MjsGeom& self) -> int { return mjs_getId(self.element); });
  mjsGeom.def("delete", [](raw::MjsGeom& self) { mjs_delete(self.element); });
  mjsGeom.def("set_frame", [](raw::MjsGeom& self, raw::MjsFrame& frame) {
    mjs_setFrame(self.element, &frame);
  });
  mjsGeom.def("set_default", [](raw::MjsGeom& self, raw::MjsDefault& def) {
    mjs_setDefault(self.element, &def);
  });
  mjsGeom.def(
      "default",
      [](raw::MjsGeom& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSJOINT ====================================
  mjsJoint.def_property_readonly(
      "id", [](raw::MjsJoint& self) -> int { return mjs_getId(self.element); });
  mjsJoint.def("delete", [](raw::MjsJoint& self) { mjs_delete(self.element); });
  mjsJoint.def("set_frame", [](raw::MjsJoint& self, raw::MjsFrame& frame) {
    mjs_setFrame(self.element, &frame);
  });
  mjsJoint.def("set_default", [](raw::MjsJoint& self, raw::MjsDefault& def) {
    mjs_setDefault(self.element, &def);
  });
  mjsJoint.def(
      "default",
      [](raw::MjsJoint& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSSITE =====================================
  mjsSite.def_property_readonly(
      "id", [](raw::MjsSite& self) -> int { return mjs_getId(self.element); });
  mjsSite.def("delete", [](raw::MjsSite& self) { mjs_delete(self.element); });
  mjsSite.def("set_frame", [](raw::MjsSite& self, raw::MjsFrame& frame) {
    mjs_setFrame(self.element, &frame);
  });
  mjsSite.def("set_default", [](raw::MjsSite& self, raw::MjsDefault& def) {
    mjs_setDefault(self.element, &def);
  });
  mjsSite.def(
      "default",
      [](raw::MjsSite& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSCAMERA ===================================
  mjsCamera.def_property_readonly("id", [](raw::MjsCamera& self) -> int {
    return mjs_getId(self.element);
  });
  mjsCamera.def("delete",
                [](raw::MjsCamera& self) { mjs_delete(self.element); });
  mjsCamera.def("set_frame", [](raw::MjsCamera& self, raw::MjsFrame& frame) {
    mjs_setFrame(self.element, &frame);
  });
  mjsCamera.def("set_default", [](raw::MjsCamera& self, raw::MjsDefault& def) {
    mjs_setDefault(self.element, &def);
  });
  mjsCamera.def(
      "default",
      [](raw::MjsCamera& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSLIGHT ====================================
  mjsLight.def_property_readonly(
      "id", [](raw::MjsLight& self) -> int { return mjs_getId(self.element); });
  mjsLight.def("delete", [](raw::MjsLight& self) { mjs_delete(self.element); });
  mjsLight.def("set_frame", [](raw::MjsLight& self, raw::MjsFrame& frame) {
    mjs_setFrame(self.element, &frame);
  });
  mjsLight.def("set_default", [](raw::MjsLight& self, raw::MjsDefault& def) {
    mjs_setDefault(self.element, &def);
  });
  mjsLight.def(
      "default",
      [](raw::MjsLight& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSMATERIAL =================================
  mjsMaterial.def_property_readonly("id", [](raw::MjsMaterial& self) -> int {
    return mjs_getId(self.element);
  });
  mjsMaterial.def("delete",
                  [](raw::MjsMaterial& self) { mjs_delete(self.element); });
  mjsMaterial.def("set_default",
                  [](raw::MjsMaterial& self, raw::MjsDefault& def) {
                    mjs_setDefault(self.element, &def);
                  });
  mjsMaterial.def(
      "default",
      [](raw::MjsMaterial& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSMESH =====================================
  mjsMesh.def_property_readonly(
      "id", [](raw::MjsMesh& self) -> int { return mjs_getId(self.element); });
  mjsMesh.def("delete", [](raw::MjsMesh& self) { mjs_delete(self.element); });
  mjsMesh.def("set_default", [](raw::MjsMesh& self, raw::MjsDefault& def) {
    mjs_setDefault(self.element, &def);
  });
  mjsMesh.def(
      "default",
      [](raw::MjsMesh& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSPAIR =====================================
  mjsPair.def_property_readonly(
      "id", [](raw::MjsPair& self) -> int { return mjs_getId(self.element); });
  mjsPair.def("delete", [](raw::MjsPair& self) { mjs_delete(self.element); });
  mjsPair.def("set_default", [](raw::MjsPair& self, raw::MjsDefault& def) {
    mjs_setDefault(self.element, &def);
  });
  mjsPair.def(
      "default",
      [](raw::MjsPair& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSEQUAL ====================================
  mjsEquality.def_property_readonly("id", [](raw::MjsEquality& self) -> int {
    return mjs_getId(self.element);
  });
  mjsEquality.def("delete",
                  [](raw::MjsEquality& self) { mjs_delete(self.element); });
  mjsEquality.def("set_default",
                  [](raw::MjsEquality& self, raw::MjsDefault& def) {
                    mjs_setDefault(self.element, &def);
                  });
  mjsEquality.def(
      "default",
      [](raw::MjsEquality& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSACTUATOR =================================
  mjsActuator.def_property_readonly("id", [](raw::MjsActuator& self) -> int {
    return mjs_getId(self.element);
  });
  mjsActuator.def("delete",
                  [](raw::MjsActuator& self) { mjs_delete(self.element); });
  mjsActuator.def("set_default",
                  [](raw::MjsActuator& self, raw::MjsDefault& def) {
                    mjs_setDefault(self.element, &def);
                  });
  mjsActuator.def(
      "default",
      [](raw::MjsActuator& self) -> raw::MjsDefault* {
        return mjs_getDefault(self.element);
      },
      py::return_value_policy::reference_internal);

  // ============================= MJSTENDON ===================================
  mjsTendon.def_property_readonly("id", [](raw::MjsTendon& self) -> int {
    return mjs_getId(self.element);
  });
  mjsTendon.def("delete",
                [](raw::MjsTendon& self) { mjs_delete(self.element); });
  mjsTendon.def("set_default", [](raw::MjsTendon& self, raw::MjsDefault& def) {
    mjs_setDefault(self.element, &def);
  });
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
  mjsSensor.def_property_readonly("id", [](raw::MjsSensor& self) -> int {
    return mjs_getId(self.element);
  });
  mjsSensor.def("delete",
                [](raw::MjsSensor& self) { mjs_delete(self.element); });

  // ============================= MJSFLEX =====================================
  mjsFlex.def_property_readonly(
      "id", [](raw::MjsFlex& self) -> int { return mjs_getId(self.element); });
  mjsFlex.def("delete", [](raw::MjsFlex& self) { mjs_delete(self.element); });

  // ============================= MJSHFIELD ===================================
  mjsHField.def_property_readonly("id", [](raw::MjsHField& self) -> int {
    return mjs_getId(self.element);
  });
  mjsHField.def("delete",
                [](raw::MjsHField& self) { mjs_delete(self.element); });

  // ============================= MJSSKIN =====================================
  mjsSkin.def_property_readonly(
      "id", [](raw::MjsSkin& self) -> int { return mjs_getId(self.element); },
      py::return_value_policy::reference_internal);
  mjsSkin.def("delete", [](raw::MjsSkin& self) { mjs_delete(self.element); });

  // ============================= MJSTEXTURE ==================================
  mjsTexture.def_property_readonly("id", [](raw::MjsTexture& self) -> int {
    return mjs_getId(self.element);
  });
  mjsTexture.def("delete",
                 [](raw::MjsTexture& self) { mjs_delete(self.element); });

  // ============================= MJSKEY ======================================
  mjsKey.def_property_readonly(
      "id", [](raw::MjsKey& self) -> int { return mjs_getId(self.element); });
  mjsKey.def("delete", [](raw::MjsKey& self) { mjs_delete(self.element); });

  // ============================= MJSTEXT =====================================
  mjsText.def_property_readonly(
      "id", [](raw::MjsText& self) -> int { return mjs_getId(self.element); });
  mjsText.def("delete", [](raw::MjsText& self) { mjs_delete(self.element); });

  // ============================= MJSNUMERIC ==================================
  mjsNumeric.def_property_readonly("id", [](raw::MjsNumeric& self) -> int {
    return mjs_getId(self.element);
  });
  mjsNumeric.def("delete",
                 [](raw::MjsNumeric& self) { mjs_delete(self.element); });

  // ============================= MJSEXCLUDE ==================================
  mjsExclude.def_property_readonly("id", [](raw::MjsExclude& self) -> int {
    return mjs_getId(self.element);
  });
  mjsExclude.def("delete",
                 [](raw::MjsExclude& self) { mjs_delete(self.element); });

  // ============================= MJSTUPLE ====================================
  mjsTuple.def_property_readonly(
      "id", [](raw::MjsTuple& self) -> int { return mjs_getId(self.element); });
  mjsTuple.def("delete", [](raw::MjsTuple& self) { mjs_delete(self.element); });

  // ============================= MJSPLUGIN ===================================
  mjsPlugin.def_property_readonly("id", [](raw::MjsPlugin& self) -> int {
    return mjs_getId(self.instance);
  });
  mjsPlugin.def("delete",
                [](raw::MjsPlugin& self) { mjs_delete(self.instance); });

#include "specs.cc.inc"
}  // PYBIND11_MODULE // NOLINT
}  // namespace mujoco::python
