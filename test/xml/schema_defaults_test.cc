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

// Checks that the attribute defaults declared in mjcf.schema agree with the
// C default-constructors: every generated row is compared against a
// freshly-constructed spec element.

#include <map>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include "test/fixture.h"

#include "src/xml/generated/mjcf_default_table.inc"

namespace mujoco {
namespace {

using SchemaDefaultsTest = MujocoTest;

TEST_F(SchemaDefaultsTest, DeclaredDefaultsMatchConstructors) {
  mjSpec* spec = mj_makeSpec();
  mjsBody* world = mjs_findBody(spec, "world");
  ASSERT_NE(world, nullptr);
  mjsBody* body = mjs_addBody(world, nullptr);

  std::map<std::string, const void*> objects = {
      {"mjOption", &spec->option},
      {"mjVisual", &spec->visual},
      {"mjsBody", body},
      {"mjsFrame", mjs_addFrame(body, nullptr)},
      {"mjsJoint", mjs_addJoint(body, nullptr)},
      {"mjsGeom", mjs_addGeom(body, nullptr)},
      {"mjsSite", mjs_addSite(body, nullptr)},
      {"mjsCamera", mjs_addCamera(body, nullptr)},
      {"mjsLight", mjs_addLight(body, nullptr)},
      {"mjsPair", mjs_addPair(spec, nullptr)},
      {"mjsEquality", mjs_addEquality(spec, nullptr)},
      {"mjsTendon", mjs_addTendon(spec, nullptr)},
      {"mjsActuator", mjs_addActuator(spec, nullptr)},
      {"mjsSensor", mjs_addSensor(spec)},
      {"mjsMesh", mjs_addMesh(spec, nullptr)},
      {"mjsSkin", mjs_addSkin(spec)},
      {"mjsMaterial", mjs_addMaterial(spec, nullptr)},
      {"mjsTexture", mjs_addTexture(spec)},
  };

  for (int t = 0; t < kDefaultTablesN; t++) {
    const mjXDefaultTable& table = kDefaultTables[t];
    auto it = objects.find(table.structname);
    ASSERT_NE(it, objects.end()) << "no factory for " << table.structname;
    const char* base = static_cast<const char*>(it->second);
    for (int i = 0; i < table.n; i++) {
      const mjXDefaultEntry& entry = table.entries[i];
      const char* field = base + entry.offset;
      for (int j = 0; j < entry.len; j++) {
        double expected = j < entry.ndecl ? entry.value[j] : 0;
        double actual = 0;
        switch (entry.kind) {
          case 0:  // double
            actual = reinterpret_cast<const double*>(field)[j];
            break;
          case 1:  // float: compare at float precision
            actual = reinterpret_cast<const float*>(field)[j];
            expected = static_cast<float>(expected);
            break;
          case 2:  // int-sized, including enums
            actual = reinterpret_cast<const int*>(field)[j];
            break;
          case 3:  // byte
            actual = reinterpret_cast<const unsigned char*>(field)[j];
            break;
          case 4:  // mjtNum: compare at mjtNum precision
            actual = reinterpret_cast<const mjtNum*>(field)[j];
            expected = static_cast<mjtNum>(expected);
            break;
        }
        EXPECT_EQ(actual, expected)
            << table.structname << "." << entry.attr << "[" << j << "]";
      }
    }
  }
  mj_deleteSpec(spec);
}

}  // namespace
}  // namespace mujoco
