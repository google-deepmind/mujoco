// Copyright 2021 DeepMind Technologies Limited
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

// Tests for user/user_api.cc.


#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "src/user/user_api.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;


// ----------------------------- test set/get  --------------------------------

TEST_F(MujocoTest, ReadWriteData) {
  mjmModel* model = mjm_createModel();
  mjmBody* world = mjm_findBody(model, "world");
  mjmBody* body = mjm_addBody(world, 0);
  mjmSite* site = mjm_addSite(body, 0);

  {
    double vec[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    const char* str = "sitename";

    mjm_setString(site->name, str);
    mjm_setDouble(site->userdata, vec, 10);
  }

  EXPECT_THAT(mjm_getString(site->name), HasSubstr("sitename"));

  int nsize;
  const double* vec = mjm_getDouble(site->userdata, &nsize);
  for (int i = 0; i < nsize; ++i) {
    EXPECT_EQ(vec[i], i);
  }

  mjm_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
