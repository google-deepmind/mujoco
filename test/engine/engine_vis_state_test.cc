// Copyright 2023 DeepMind Technologies Limited
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
using MjvSceneStateTest = MujocoTest;

constexpr int kMaxGeom = 10000;

static const char* const kHammockPath =
    "engine/testdata/hammock/hammock.xml";
static const char* const kTendonPath =
    "engine/testdata/island/tendon_wrap.xml";
static const char* const kFrustumPath =
    "engine/testdata/vis_visualize/frustum.xml";
static const char* const kFlex = "testdata/flex.xml";
static const char* const kModelPath = "testdata/model.xml";

#define EXPECT_ZERO(exp) EXPECT_EQ(0, exp);

TEST_F(MjvSceneStateTest, CanUpdateFromState) {
  for (const char* path :
       {kHammockPath, kTendonPath, kModelPath, kFrustumPath, kFlex}) {
    const std::string xml_path = GetTestDataFilePath(path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
    ASSERT_THAT(model, NotNull()) << "Failed to load model from " << path;
    mjData* data = mj_makeData(model);

    while (data->time < 2) {
      mj_step(model, data);
    }

    mjvScene scn1;
    mjv_defaultScene(&scn1);
    mjv_makeScene(model, &scn1, kMaxGeom);

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

    mjv_updateScene(model, data, &opt, &pert, &cam, mjCAT_ALL, &scn1);
    EXPECT_GT(scn1.ngeom, 0);
    if (model->nskin) EXPECT_GT(scn1.nskin, 0);
    EXPECT_GT(scn1.nlight, 0);

    mjvSceneState scnstate;
    mjv_defaultSceneState(&scnstate);
    mjv_makeSceneState(model, data, &scnstate, kMaxGeom);
    mjv_updateSceneState(model, data, &opt, &scnstate);

    mjvScene scn2;
    mjv_defaultScene(&scn2);
    mjv_makeScene(model, &scn2, kMaxGeom);
    mjv_updateSceneFromState(&scnstate, &opt, &pert, &cam, mjCAT_ALL, &scn2);

    EXPECT_EQ(scn1.ngeom, scn2.ngeom);
    for (int i = 0; i < scn1.ngeom; ++i) {
      EXPECT_ZERO(std::memcmp(&scn1.geoms[i], &scn2.geoms[i], sizeof(mjvGeom)));
    }
    // NB: scn->geomorder is a scratch space for use by mjr_render, so we don't
    // need to compare them here.

    EXPECT_LE(scn1.nskin, scn2.nskin);
    EXPECT_ZERO(std::memcmp(scn1.skinfacenum, scn2.skinfacenum,
                          sizeof(*scn2.skinfacenum) * scn2.nskin));
    EXPECT_ZERO(std::memcmp(scn1.skinvertadr, scn2.skinvertadr,
                          sizeof(*scn2.skinvertadr) * scn2.nskin));
    EXPECT_ZERO(std::memcmp(scn1.skinvertnum, scn2.skinvertnum,
                          sizeof(*scn2.skinvertnum) * scn2.nskin));
    EXPECT_ZERO(std::memcmp(scn1.skinvert, scn2.skinvert,
                          sizeof(*scn2.skinvert) * scn2.nskin));
    EXPECT_ZERO(std::memcmp(scn1.skinnormal, scn2.skinnormal,
                          sizeof(*scn2.skinnormal) * scn2.nskin));

    auto scn1_cmp_begin = reinterpret_cast<char*>(&scn1.nlight);
    auto scn2_cmp_begin = reinterpret_cast<char*>(&scn2.nlight);
    auto cmp_bytes =
        sizeof(mjvScene) - (scn2_cmp_begin - reinterpret_cast<char*>(&scn2));
    EXPECT_ZERO(std::memcmp(scn1_cmp_begin, scn2_cmp_begin, cmp_bytes));

    mjv_freeScene(&scn1);
    mjv_freeScene(&scn2);
    mjv_freeSceneState(&scnstate);

    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

}  // namespace
}  // namespace mujoco
