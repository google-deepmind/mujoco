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

#include "src/engine/engine_print.h"

#include <array>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

namespace mujoco {
namespace {

#ifdef MEMORY_SANITIZER
using ::testing::Eq;
using ::testing::Not;
#endif
using ::testing::NotNull;

using EnginePrintTest = MujocoTest;


constexpr const char* NullFile() {
#ifdef _WIN32
  return "NUL";
#else
  return "/dev/null";
#endif
}

TEST_F(EnginePrintTest, PrintDataWorksWithMsan) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());

  // Mark qacc_smooth[0] as initialized.
  data->qacc_smooth[0] = 0;

  // This will read "uninitialized" values from mjData, but shouldn't fail.
  mj_printData(model, data, NullFile());

  // After mj_printData, poisoned status should be restored correctly, so
  // qacc_smooth[0] should be marked initialzed.
  EXPECT_EQ(data->qacc_smooth[0], 0);

#ifdef MEMORY_SANITIZER
  EXPECT_THAT(__msan_test_shadow(data->buffer, data->nbuffer), Not(Eq(-1))) <<
      "Expecting some of data->buffer to be marked uninitialized";
#endif
  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
