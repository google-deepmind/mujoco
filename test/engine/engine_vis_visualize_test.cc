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

#include <cstring>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using MjvSceneTest = MujocoTest;

constexpr int kMaxGeom = 10000;

static const char* const kModelPath = "testdata/model.xml";

TEST_F(MjvSceneTest, UpdateScene) {
  for (const char* path : {kModelPath}) {
    const std::string xml_path = GetTestDataFilePath(path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
    ASSERT_THAT(model, NotNull()) << "Failed to load model from " << path;
    mjData* data = mj_makeData(model);

    while (data->time < .2) {
      mj_step(model, data);
    }

    mjvScene scn;
    mjv_defaultScene(&scn);
    mjv_makeScene(model, &scn, kMaxGeom);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    mjvCamera cam;
    mjv_defaultFreeCamera(model, &cam);

    // Enable all flags to exercise all code paths
    for (int i = 0; i < mjNVISFLAG; ++i) {
      opt.flags[i] = 1;
    }

    mjv_updateScene(model, data, &opt, &pert, &cam, mjCAT_ALL, &scn);
    EXPECT_GT(scn.ngeom, 0);
    if (model->nskin) EXPECT_GT(scn.nskin, 0);
    EXPECT_GT(scn.nlight, 0);

    mjv_updateScene(model, data, &opt, &pert, &cam, mjCAT_ALL, &scn);

    // call mj_copyData to expose any memory leaks mjv_updateScene.
    mjData* data_copy = mj_copyData(nullptr, model, data);

    mjv_freeScene(&scn);
    mj_deleteData(data_copy);
    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

}  // namespace
}  // namespace mujoco
