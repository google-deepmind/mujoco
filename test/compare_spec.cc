// Copyright 2026 DeepMind Technologies Limited
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

#include "test/compare_spec.h"

#include <cmath>
#include <cstddef>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <mujoco/mjspec.h>
#include <mujoco/mjspecmacro.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>

namespace mujoco {
namespace {

// equality of authored values; a field which is not set is NaN, and two such
// fields are the same
template <typename T>
bool Same(const T& a, const T& b) {
  if constexpr (std::is_floating_point_v<T>) {
    return a == b || (std::isnan(a) && std::isnan(b));
  } else {
    return a == b;
  }
}

template <typename T>
bool Same(const std::vector<T>& a, const std::vector<T>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); i++) {
    if (!Same(a[i], b[i])) return false;
  }
  return true;
}

// value as text, for the report
template <typename T>
std::string Str(const T& value) {
  std::ostringstream out;
  out.precision(17);
  if constexpr (std::is_enum_v<T>) {
    out << static_cast<long long>(value);
  } else if constexpr (std::is_same_v<T, std::byte>) {
    out << std::to_integer<int>(value);
  } else if constexpr (sizeof(T) == 1) {
    out << static_cast<int>(value);
  } else {
    out << value;
  }
  return out.str();
}

std::string Str(const std::string& value) { return "'" + value + "'"; }

template <typename T>
std::string Str(const std::vector<T>& value) {
  std::string out = "[";
  for (size_t i = 0; i < value.size() && i < 8; i++) {
    out += (i ? " " : "") + Str(value[i]);
  }
  if (value.size() > 8) out += " ...";
  return out + "]";
}

class Comparer {
 public:
  Comparer(const mjSpec* s1, const mjSpec* s2) : s1_(s1), s2_(s2) {}

  std::string Run() {
    where_ = "spec";
    Fields(*s1_, *s2_);
    Field("authored", s1_->authored, s2_->authored);

    // parents and frames are compared by their position in the spec
    Index(mjOBJ_BODY, mjs_asBody);
    Index(mjOBJ_FRAME, mjs_asFrame);

    Elements("body", mjOBJ_BODY, mjs_asBody);
    Elements("frame", mjOBJ_FRAME, mjs_asFrame);
    Elements("joint", mjOBJ_JOINT, mjs_asJoint);
    Elements("geom", mjOBJ_GEOM, mjs_asGeom);
    Elements("site", mjOBJ_SITE, mjs_asSite);
    Elements("camera", mjOBJ_CAMERA, mjs_asCamera);
    Elements("light", mjOBJ_LIGHT, mjs_asLight);
    Elements("flex", mjOBJ_FLEX, mjs_asFlex);
    Elements("mesh", mjOBJ_MESH, mjs_asMesh);
    Elements("skin", mjOBJ_SKIN, mjs_asSkin);
    Elements("hfield", mjOBJ_HFIELD, mjs_asHField);
    Elements("texture", mjOBJ_TEXTURE, mjs_asTexture);
    Elements("material", mjOBJ_MATERIAL, mjs_asMaterial);
    Elements("pair", mjOBJ_PAIR, mjs_asPair);
    Elements("exclude", mjOBJ_EXCLUDE, mjs_asExclude);
    Elements("equality", mjOBJ_EQUALITY, mjs_asEquality);
    Elements("tendon", mjOBJ_TENDON, mjs_asTendon);
    Elements("actuator", mjOBJ_ACTUATOR, mjs_asActuator);
    Elements("sensor", mjOBJ_SENSOR, mjs_asSensor);
    Elements("numeric", mjOBJ_NUMERIC, mjs_asNumeric);
    Elements("text", mjOBJ_TEXT, mjs_asText);
    Elements("tuple", mjOBJ_TUPLE, mjs_asTuple);
    Elements("key", mjOBJ_KEY, mjs_asKey);
    Elements("plugin", mjOBJ_PLUGIN, mjs_asPlugin);

    Defaults();

    if (count_ > kMaxReported) {
      out_ << "and " << count_ - kMaxReported << " more differences\n";
    }
    return out_.str();
  }

 private:
  static constexpr int kMaxReported = 20;

  using PluginAttributes = std::map<std::string, std::string, std::less<>>;

  void Report(const std::string& field, const std::string& v1,
              const std::string& v2) {
    if (++count_ > kMaxReported) return;
    out_ << where_ << ": " << prefix_ << field << ": " << v1 << " != " << v2
         << '\n';
  }

  //---------------------------- one field -------------------------------------

  // number or enum
  template <
      typename T,
      std::enable_if_t<std::is_arithmetic_v<T> || std::is_enum_v<T>, int> = 0>
  void Field(const std::string& name, const T& a, const T& b) {
    if (!Same(a, b)) Report(name, Str(a), Str(b));
  }

  // array of fixed size
  template <typename T, size_t N>
  void Field(const std::string& name, const T (&a)[N], const T (&b)[N]) {
    for (size_t i = 0; i < N; i++) {
      if (!Same(a[i], b[i])) {
        Report(name + "[" + std::to_string(i) + "]", Str(a[i]), Str(b[i]));
      }
    }
  }

  // whether both pointers are set; some structs leave unused strings null
  bool Set(const std::string& name, const void* a, const void* b) {
    if (!a != !b) Report(name, a ? "set" : "null", b ? "set" : "null");
    return a && b;
  }

  // string
  void Field(const std::string& name, const std::string* a,
             const std::string* b) {
    if (!Set(name, a, b)) return;
    if (*a != *b) Report(name, Str(*a), Str(*b));
  }

  // vector
  template <typename T>
  void Field(const std::string& name, const std::vector<T>* a,
             const std::vector<T>* b) {
    if (!Set(name, a, b)) return;
    if (a->size() != b->size()) {
      Report(name + " size", std::to_string(a->size()),
             std::to_string(b->size()));
      return;
    }
    for (size_t i = 0; i < a->size(); i++) {
      if (!Same((*a)[i], (*b)[i])) {
        Report(name + "[" + std::to_string(i) + "]", Str((*a)[i]),
               Str((*b)[i]));
        return;
      }
    }
  }

  // the element that a struct belongs to is not authored content
  void Field(const std::string& name, const mjsElement* a,
             const mjsElement* b) {}

  // struct within a struct
  template <typename T, std::enable_if_t<std::is_class_v<T>, int> = 0>
  void Field(const std::string& name, const T& a, const T& b) {
    std::string prefix = prefix_;
    prefix_ += name + ".";
    Fields(a, b);
    prefix_ = prefix;
  }

  // struct held by pointer: the elements of a default class
  template <typename T>
  void Pointer(const std::string& name, const T* a, const T* b) {
    if (Set(name, a, b)) Field(name, *a, *b);
  }
  void Pointer(const std::string& name, const mjsElement* a,
               const mjsElement* b) {}

  //---------------------------- all fields of a struct ------------------------

#define X(type, name, dim) Field(#name, a.name, b.name);
#define XVEC(type, name, dim) Field(#name, a.name, b.name);
  void Fields(const mjSpec& a, const mjSpec& b) { MJSPEC_FIELDS }
  void Fields(const mjsCompiler& a, const mjsCompiler& b) { MJSCOMPILER_FIELDS }
  void Fields(const mjsOrientation& a, const mjsOrientation& b) {
    MJSORIENTATION_FIELDS
  }
  void Fields(const mjsPlugin& a, const mjsPlugin& b) { MJSPLUGIN_FIELDS }
  void Fields(const mjsBody& a, const mjsBody& b) { MJSBODY_FIELDS }
  void Fields(const mjsFrame& a, const mjsFrame& b) { MJSFRAME_FIELDS }
  void Fields(const mjsJoint& a, const mjsJoint& b) { MJSJOINT_FIELDS }
  void Fields(const mjsGeom& a, const mjsGeom& b) { MJSGEOM_FIELDS }
  void Fields(const mjsSite& a, const mjsSite& b) { MJSSITE_FIELDS }
  void Fields(const mjsCamera& a, const mjsCamera& b) { MJSCAMERA_FIELDS }
  void Fields(const mjsLight& a, const mjsLight& b) { MJSLIGHT_FIELDS }
  void Fields(const mjsFlex& a, const mjsFlex& b) { MJSFLEX_FIELDS }
  void Fields(const mjsMesh& a, const mjsMesh& b) { MJSMESH_FIELDS }
  void Fields(const mjsHField& a, const mjsHField& b) { MJSHFIELD_FIELDS }
  void Fields(const mjsSkin& a, const mjsSkin& b) { MJSSKIN_FIELDS }
  void Fields(const mjsTexture& a, const mjsTexture& b) { MJSTEXTURE_FIELDS }
  void Fields(const mjsMaterial& a, const mjsMaterial& b) { MJSMATERIAL_FIELDS }
  void Fields(const mjsPair& a, const mjsPair& b) { MJSPAIR_FIELDS }
  void Fields(const mjsExclude& a, const mjsExclude& b) { MJSEXCLUDE_FIELDS }
  void Fields(const mjsEquality& a, const mjsEquality& b) { MJSEQUALITY_FIELDS }
  void Fields(const mjsTendon& a, const mjsTendon& b) { MJSTENDON_FIELDS }
  void Fields(const mjsWrap& a, const mjsWrap& b) { MJSWRAP_FIELDS }
  void Fields(const mjsActuator& a, const mjsActuator& b) { MJSACTUATOR_FIELDS }
  void Fields(const mjsSensor& a, const mjsSensor& b) { MJSSENSOR_FIELDS }
  void Fields(const mjsNumeric& a, const mjsNumeric& b) { MJSNUMERIC_FIELDS }
  void Fields(const mjsText& a, const mjsText& b) { MJSTEXT_FIELDS }
  void Fields(const mjsTuple& a, const mjsTuple& b) { MJSTUPLE_FIELDS }
  void Fields(const mjsKey& a, const mjsKey& b) { MJSKEY_FIELDS }
  void Fields(const mjOption& a, const mjOption& b) { MJOPTION_FIELDS }
  void Fields(const decltype(mjVisual::global)& a,
              const decltype(mjVisual::global)& b) {
    MJVISUAL_GLOBAL_FIELDS
  }
  void Fields(const decltype(mjVisual::quality)& a,
              const decltype(mjVisual::quality)& b) {
    MJVISUAL_QUALITY_FIELDS
  }
  void Fields(const decltype(mjVisual::headlight)& a,
              const decltype(mjVisual::headlight)& b) {
    MJVISUAL_HEADLIGHT_FIELDS
  }
  void Fields(const decltype(mjVisual::map)& a,
              const decltype(mjVisual::map)& b) {
    MJVISUAL_MAP_FIELDS
  }
  void Fields(const decltype(mjVisual::scale)& a,
              const decltype(mjVisual::scale)& b) {
    MJVISUAL_SCALE_FIELDS
  }
  void Fields(const decltype(mjVisual::rgba)& a,
              const decltype(mjVisual::rgba)& b){MJVISUAL_RGBA_FIELDS}
#undef XVEC
#undef X

#define X(name, dim) Field(#name, a.name, b.name);
#define XVEC(name, dim) Field(#name, a.name, b.name);
  void Fields(const mjStatistic& a, const mjStatistic& b){MJSTATISTIC_FIELDS}
#undef XVEC
#undef X

#define X(type, name, dim) Pointer(#name, a.name, b.name);
  void Fields(const mjsDefault& a, const mjsDefault& b) {
    MJSDEFAULT_FIELDS
  }
#undef X

  void Fields(const mjVisual& a, const mjVisual& b) {
    Field("global", a.global, b.global);
    Field("quality", a.quality, b.quality);
    Field("headlight", a.headlight, b.headlight);
    Field("map", a.map, b.map);
    Field("scale", a.scale, b.scale);
    Field("rgba", a.rgba, b.rgba);
  }

  void Fields(const mjLROpt& a, const mjLROpt& b) {
    Field("mode", a.mode, b.mode);
    Field("useexisting", a.useexisting, b.useexisting);
    Field("uselimit", a.uselimit, b.uselimit);
    Field("accel", a.accel, b.accel);
    Field("maxforce", a.maxforce, b.maxforce);
    Field("timeconst", a.timeconst, b.timeconst);
    Field("timestep", a.timestep, b.timestep);
    Field("inttotal", a.inttotal, b.inttotal);
    Field("interval", a.interval, b.interval);
    Field("tolrange", a.tolrange, b.tolrange);
  }

  void Fields(const mjsAuthored& a, const mjsAuthored& b) {
    Field("option", a.option, b.option);
    Field("disableflags", a.disableflags, b.disableflags);
    Field("enableflags", a.enableflags, b.enableflags);
    Field("disableactuator", a.disableactuator, b.disableactuator);
    Field("visual_global", a.visual_global, b.visual_global);
    Field("visual_quality", a.visual_quality, b.visual_quality);
    Field("visual_headlight", a.visual_headlight, b.visual_headlight);
    Field("visual_map", a.visual_map, b.visual_map);
    Field("visual_scale", a.visual_scale, b.visual_scale);
    Field("visual_rgba", a.visual_rgba, b.visual_rgba);
  }

  //---------------------------- content that is not in the struct -------------

  template <typename T>
  void Extra(const T& a, const T& b) {}

  // the path of a tendon
  void Extra(const mjsTendon& a, const mjsTendon& b) {
    int n1 = mjs_getWrapNum(&a);
    int n2 = mjs_getWrapNum(&b);
    if (n1 != n2) {
      Report("number of wraps", std::to_string(n1), std::to_string(n2));
      return;
    }
    std::string prefix = prefix_;
    for (int i = 0; i < n1; i++) {
      const mjsWrap* w1 = mjs_getWrap(&a, i);
      const mjsWrap* w2 = mjs_getWrap(&b, i);
      prefix_ = prefix + "wrap[" + std::to_string(i) + "].";
      Fields(*w1, *w2);
      if (w1->type != w2->type) continue;
      std::string target1 = Name(mjs_getWrapTarget(w1));
      std::string target2 = Name(mjs_getWrapTarget(w2));
      Field("target", &target1, &target2);
      if (w1->type == mjWRAP_SPHERE || w1->type == mjWRAP_CYLINDER) {
        mjsSite* side1 = mjs_getWrapSideSite(w1);
        mjsSite* side2 = mjs_getWrapSideSite(w2);
        std::string sidesite1 = side1 ? Name(side1->element) : "";
        std::string sidesite2 = side2 ? Name(side2->element) : "";
        Field("sidesite", &sidesite1, &sidesite2);
      } else if (w1->type == mjWRAP_PULLEY) {
        Field("divisor", mjs_getWrapDivisor(w1), mjs_getWrapDivisor(w2));
      } else if (w1->type == mjWRAP_JOINT) {
        Field("coef", mjs_getWrapCoef(w1), mjs_getWrapCoef(w2));
      }
    }
    prefix_ = prefix;
  }

  // the configuration of a plugin instance
  void Extra(const mjsPlugin& a, const mjsPlugin& b) {
    const auto& config1 =
        *static_cast<const PluginAttributes*>(mjs_getPluginAttributes(&a));
    const auto& config2 =
        *static_cast<const PluginAttributes*>(mjs_getPluginAttributes(&b));
    if (config1 != config2) {
      Report("config", std::to_string(config1.size()) + " attributes",
             std::to_string(config2.size()) + " attributes");
    }
  }

  //---------------------------- elements --------------------------------------

  static std::string Name(mjsElement* element) {
    return element ? *mjs_getName(element) : "";
  }

  static std::vector<mjsElement*> List(const mjSpec* s, mjtObj type) {
    std::vector<mjsElement*> list;
    for (mjsElement* element = mjs_firstElement(s, type); element;
         element = mjs_nextElement(s, element)) {
      list.push_back(element);
    }
    return list;
  }

  // record the position of every body or frame in its spec
  template <typename T>
  void Index(mjtObj type, T* (*as)(mjsElement*)) {
    int i = 0;
    for (mjsElement* element : List(s1_, type)) index_[as(element)] = i++;
    i = 0;
    for (mjsElement* element : List(s2_, type)) index_[as(element)] = i++;
  }

  int Position(const void* body_or_frame) const {
    auto it = index_.find(body_or_frame);
    return it == index_.end() ? -1 : it->second;
  }

  // the name of the default class of an element, which is noted for Defaults()
  std::string Class(const mjsElement* element) {
    const mjsDefault* def = mjs_getDefault(element);
    std::string name = def ? Name(def->element) : "";
    classes_.insert(name);
    return name;
  }

  template <typename T>
  void Elements(const std::string& type_name, mjtObj type,
                T* (*as)(mjsElement*)) {
    std::vector<mjsElement*> list1 = List(s1_, type);
    std::vector<mjsElement*> list2 = List(s2_, type);
    if (list1.size() != list2.size()) {
      where_ = type_name;
      Report("count", std::to_string(list1.size()),
             std::to_string(list2.size()));
      return;
    }
    for (size_t i = 0; i < list1.size(); i++) {
      mjsElement* e1 = list1[i];
      mjsElement* e2 = list2[i];
      where_ =
          type_name + "[" + std::to_string(i) + "] " + Str(*mjs_getName(e1));
      Field("name", mjs_getName(e1), mjs_getName(e2));
      std::string class1 = Class(e1);
      std::string class2 = Class(e2);
      Field("class", &class1, &class2);
      Field("parent", Position(mjs_getParent(e1)), Position(mjs_getParent(e2)));
      Field("frame", Position(mjs_getFrame(e1)), Position(mjs_getFrame(e2)));
      const T& a = *as(e1);
      const T& b = *as(e2);
      Childclass(a, b);
      Fields(a, b);
      Extra(a, b);
    }
  }

  // note the class which a body or frame gives to its children
  template <typename T>
  void Childclass(const T& a, const T& b) {}
  void Childclass(const mjsBody& a, const mjsBody& b) {
    classes_.insert(*a.childclass);
  }
  void Childclass(const mjsFrame& a, const mjsFrame& b) {
    classes_.insert(*a.childclass);
  }

  void Defaults() {
    where_ = "default";
    const mjsDefault* main1 = mjs_getSpecDefault(s1_);
    const mjsDefault* main2 = mjs_getSpecDefault(s2_);
    if (main1 && main2) Fields(*main1, *main2);
    for (const std::string& name : classes_) {
      const mjsDefault* def1 = mjs_findDefault(s1_, name.c_str());
      const mjsDefault* def2 = mjs_findDefault(s2_, name.c_str());
      where_ = "default " + Str(name);
      if (!def1 || !def2) {
        if (def1 || def2)
          Report("class", def1 ? "" : "none", def2 ? "" : "none");
        continue;
      }
      Fields(*def1, *def2);
    }
  }

  const mjSpec* s1_;
  const mjSpec* s2_;
  std::map<const void*, int> index_;
  std::set<std::string> classes_;
  std::string where_;
  std::string prefix_;
  std::ostringstream out_;
  int count_ = 0;
};

}  // namespace

std::string CompareSpec(const mjSpec* s1, const mjSpec* s2) {
  return Comparer(s1, s2).Run();
}

}  // namespace mujoco
