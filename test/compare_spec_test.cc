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

// Tests for test/compare_spec.h.

#include "test/compare_spec.h"

#include <array>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::IsEmpty;
using ::testing::NotNull;

using CompareSpecTest = MujocoTest;

static constexpr char kXml[] = R"(
<mujoco>
  <compiler angle="degree"/>
  <default>
    <default class="big">
      <geom size=".2"/>
    </default>
  </default>
  <asset>
    <material name="red" rgba="1 0 0 1"/>
  </asset>
  <worldbody>
    <frame name="frame" euler="0 0 30">
      <body name="a" pos="0 0 1">
        <joint name="hinge" axis="0 1 0"/>
        <geom name="ball" class="big" material="red" user="1 2"/>
        <site name="tip" pos="0 0 .2"/>
        <site name="end" pos="0 0 .4"/>
      </body>
    </frame>
    <body name="b" pos="1 0 1">
      <joint name="slide" type="slide"/>
      <geom name="box" type="box" size=".1 .1 .1"/>
      <site name="top" pos="0 0 .1"/>
    </body>
  </worldbody>
  <contact>
    <pair name="first" geom1="ball" geom2="box"/>
    <pair name="second" geom1="box" geom2="ball" condim="1"/>
  </contact>
  <tendon>
    <spatial name="tendon">
      <site site="tip"/>
      <site site="top"/>
    </spatial>
  </tendon>
  <keyframe>
    <key name="home" qpos="10 .5"/>
  </keyframe>
</mujoco>
)";

mjSpec* Parse(const std::string& xml) {
  std::array<char, 1000> error;
  mjSpec* spec = mj_parseXMLString(xml.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << error.data();
  return spec;
}

std::string Replace(std::string text, const std::string& from,
                    const std::string& to) {
  text.replace(text.find(from), from.size(), to);
  return text;
}

TEST_F(CompareSpecTest, SameSpecs) {
  mjSpec* spec = Parse(kXml);
  mjSpec* parsed_again = Parse(kXml);
  mjSpec* copy = mj_copySpec(spec);
  EXPECT_THAT(CompareSpec(spec, parsed_again), IsEmpty());
  EXPECT_THAT(CompareSpec(spec, copy), IsEmpty());
  mj_deleteSpec(copy);
  mj_deleteSpec(parsed_again);
  mj_deleteSpec(spec);
}

// every kind of authored content is compared
TEST_F(CompareSpecTest, EditedSpec) {
  using Edit = std::function<void(mjSpec*)>;
  auto geom = [](mjSpec* s, const char* name) {
    return mjs_asGeom(mjs_findElement(s, mjOBJ_GEOM, name));
  };
  std::vector<std::pair<Edit, std::string>> edits = {
      {[&](mjSpec* s) { geom(s, "ball")->size[1] = 3; },
       "geom[0] 'ball': size[1]: 0 != 3"},
      {[&](mjSpec* s) { mjs_setString(geom(s, "ball")->material, ""); },
       "geom[0] 'ball': material: 'red' != ''"},
      {[&](mjSpec* s) { geom(s, "ball")->userdata->push_back(3); },
       "geom[0] 'ball': userdata size: 2 != 3"},
      {[&](mjSpec* s) { mjs_setName(geom(s, "ball")->element, "sphere"); },
       "geom[0] 'ball': name: 'ball' != 'sphere'"},
      {[&](mjSpec* s) {
         mjs_setDefault(geom(s, "ball")->element, mjs_getSpecDefault(s));
       },
       "geom[0] 'ball': class: 'big' != 'main'"},
      {[&](mjSpec* s) {
         mjsFrame* frame = mjs_addFrame(mjs_findBody(s, "b"), nullptr);
         mjs_setFrame(geom(s, "box")->element, frame);
       },
       "geom[1] 'box': frame: -1 != 1"},
      {[&](mjSpec* s) { mjs_addGeom(mjs_findBody(s, "b"), nullptr); },
       "geom: count: 2 != 3"},
      {[](mjSpec* s) { mjs_findFrame(s, "frame")->alt.euler[2] = 45; },
       "frame[0] 'frame': alt.euler[2]: 30 != 45"},
      {[](mjSpec* s) { mjs_findDefault(s, "big")->geom->size[0] = 1; },
       "default 'big': geom.size[0]"},
      {[](mjSpec* s) {
         mjs_asKey(mjs_findElement(s, mjOBJ_KEY, "home"))->qpos->at(1) = 1;
       },
       "key[0] 'home': qpos[1]: 0.5 != 1"},
      {[](mjSpec* s) { s->compiler.degree = 0; },
       "spec: compiler.degree: 1 != 0"},
      {[](mjSpec* s) { s->option.gravity[2] = -10; },
       "spec: option.gravity[2]"},
      {[](mjSpec* s) { s->visual.global.fovy = 60; },
       "spec: visual.global.fovy"},
  };
  for (const auto& [edit, expected] : edits) {
    mjSpec* spec = Parse(kXml);
    mjSpec* edited = Parse(kXml);
    edit(edited);
    EXPECT_THAT(CompareSpec(spec, edited), HasSubstr(expected));
    mj_deleteSpec(edited);
    mj_deleteSpec(spec);
  }
}

// the structure of the spec is compared
TEST_F(CompareSpecTest, RestructuredSpec) {
  const std::string first =
      "<pair name=\"first\" geom1=\"ball\" geom2=\"box\"/>";
  const std::string second =
      "<pair name=\"second\" geom1=\"box\" geom2=\"ball\" condim=\"1\"/>";
  std::vector<std::pair<std::string, std::string>> edits = {
      // the pairs in the other order
      {Replace(Replace(Replace(kXml, first, "<first/>"), second, first),
               "<first/>", second),
       "pair[0] 'first': name: 'first' != 'second'"},
      // a site in the other body
      {Replace(Replace(kXml, "<site name=\"end\" pos=\"0 0 .4\"/>", ""),
               "<site name=\"top\"",
               "<site name=\"end\" pos=\"0 0 .4\"/><site name=\"top\""),
       "site[1] 'end': parent: 1 != 2"},
      // another site in the path of the tendon
      {Replace(kXml, "<site site=\"top\"/>", "<site site=\"end\"/>"),
       "tendon[0] 'tendon': wrap[1].target: 'top' != 'end'"},
  };
  for (const auto& [xml, expected] : edits) {
    mjSpec* spec = Parse(kXml);
    mjSpec* edited = Parse(xml);
    EXPECT_THAT(CompareSpec(spec, edited), HasSubstr(expected));
    mj_deleteSpec(edited);
    mj_deleteSpec(spec);
  }
}

}  // namespace
}  // namespace mujoco
