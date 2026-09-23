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

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

static const char* const kModelPath = "testdata/model.xml";

class MjvSceneTest : public MujocoTest {
 protected:
  static constexpr int kMaxGeom = 10000;

  void InitSceneObjects(mjModel* model, int maxgeom = kMaxGeom) {
    mjv_defaultScene(&scn_);
    mjv_makeScene(model, &scn_, maxgeom);
    mjv_defaultOption(&opt_);
    mjv_defaultPerturb(&pert_);
    mjv_defaultFreeCamera(model, &cam_);

    // enable flags to exercise additional code paths
    for (int i = 0; i < mjNVISFLAG; ++i) {
      opt_.flags[i] = 1;
    }
  }

  void FreeSceneObjects() { mjv_freeScene(&scn_); }

  mjvScene scn_;
  mjvOption opt_;
  mjvPerturb pert_;
  mjvCamera cam_;
};

TEST_F(MjvSceneTest, UpdateScene) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull())
      << "Failed to load model from " << kModelPath << ": " << error;

  InitSceneObjects(model);

  mjData* data = mj_makeData(model);
  while (data->time < .2) {
    mj_step(model, data);
  }

  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
  EXPECT_EQ(scn_.status, 0);
  EXPECT_GT(scn_.ngeom, 0);
  EXPECT_GT(scn_.nlight, 0);
  if (model->nskin) EXPECT_GT(scn_.nskin, 0);
  if (model->nflex) EXPECT_GT(scn_.nflex, 0);

  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);

  // call mj_copyData to expose any memory leaks in mjv_updateScene
  mjData* data_copy = mj_copyData(nullptr, model, data);

  mj_deleteData(data_copy);
  mj_deleteData(data);
  FreeSceneObjects();
  mj_deleteModel(model);
}

TEST_F(MjvSceneTest, UpdateSceneGeomsExhausted) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull())
      << "Failed to load model from " << kModelPath << ": " << error;

  const int maxgeoms = 1;
  InitSceneObjects(model, maxgeoms);

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // clear handlers to avoid test failure; we are explicitly expecting a warning
  mju_clearHandlers();
  mjv_updateScene(model, data, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
  EXPECT_EQ(scn_.status, 1);
  EXPECT_EQ(scn_.ngeom, maxgeoms);
  mj_deleteData(data);
  FreeSceneObjects();
  mj_deleteModel(model);
}

TEST_F(MjvSceneTest, PrincipalPointFrustumSign) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <camera name="cam" pos="0 0 1" zaxis="0 0 1"
              sensorsize="0.01 0.01" focal="0.01 0.01"
              principal="0 0.002"/>
    </worldbody>
  </mujoco>
  )";

  MjModelPtr model = LoadModelFromString(xml);
  ASSERT_THAT(model.get(), NotNull());
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());

  InitSceneObjects(model.get());

  // point camera at the fixed cam
  cam_.type = mjCAMERA_FIXED;
  cam_.fixedcamid = 0;
  mjv_updateCamera(model.get(), data.get(), &cam_, &scn_);

  float top = scn_.camera[0].frustum_top;
  float bottom = scn_.camera[0].frustum_bottom;

  // with cy > 0 the principal point is above center, so the frustum should
  // extend further downward than upward: |bottom| > top
  EXPECT_GT(-bottom, top);

  // verify exact values against the pinhole model
  float znear = model->vis.map.znear * model->stat.extent;
  float cy = model->cam_intrinsic[3];
  float fy = model->cam_intrinsic[1];
  float sh = model->cam_sensorsize[1];
  float half = znear / fy * (sh / 2);
  float offset = znear / fy * cy;

  EXPECT_FLOAT_EQ(top, half - offset);
  EXPECT_FLOAT_EQ(bottom, -(half + offset));

  FreeSceneObjects();
}

TEST_F(MjvSceneTest, InvalidFixedCamId) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <camera name="cam" pos="0 0 1"/>
    </worldbody>
  </mujoco>
  )";

  MjModelPtr model = LoadModelFromString(xml);
  ASSERT_THAT(model.get(), NotNull());
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());

  InitSceneObjects(model.get());
  cam_.type = mjCAMERA_FIXED;

  cam_.fixedcamid = -1;
  EXPECT_THAT(MjuErrorMessageFrom(mjv_updateCamera)(model.get(), data.get(),
                                                    &cam_, &scn_),
              ::testing::HasSubstr("fixed camera id is outside valid range"));
  EXPECT_THAT(MjuErrorMessageFrom(mjv_cameraFrame)(nullptr, nullptr, nullptr,
                                                   nullptr, data.get(), &cam_),
              ::testing::HasSubstr("fixed camera id is outside valid range"));

  cam_.fixedcamid = model->ncam;
  EXPECT_THAT(MjuErrorMessageFrom(mjv_updateCamera)(model.get(), data.get(),
                                                    &cam_, &scn_),
              ::testing::HasSubstr("fixed camera id is outside valid range"));

  FreeSceneObjects();
}

static constexpr char kMeshSiteXml[] = R"(
<mujoco>
  <asset>
    <mesh name="offset_box"
          vertex="1 -.5 0  3 -.5 0  3 .5 0  1 .5 0
                  1 -.5 4  3 -.5 4  3 .5 4  1 .5 4"/>
  </asset>
  <worldbody>
    <body pos=".4 .5 .6" euler="10 20 30">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="1 2 3"/>
      <frame pos=".3 -.2 .1" euler="30 10 20">
        <site name="plain" pos=".1 .2 .3" euler="20 30 10" size=".01"/>
        <site name="mesh" type="mesh" mesh="offset_box"
              pos=".1 .2 .3" euler="20 30 10"/>
        <geom name="reference" type="mesh" mesh="offset_box"
              pos=".1 .2 .3" euler="20 30 10" contype="0" conaffinity="0"/>
      </frame>
    </body>
  </worldbody>
  <sensor>
    <framepos objtype="site" objname="plain"/>
    <framepos objtype="site" objname="mesh"/>
    <framequat objtype="site" objname="plain"/>
    <framequat objtype="site" objname="mesh"/>
    <gyro site="plain"/>
    <gyro site="mesh"/>
    <accelerometer site="plain"/>
    <accelerometer site="mesh"/>
  </sensor>
</mujoco>
)";

TEST_F(MjvSceneTest, MeshSitePreservesAuthoredFrame) {
  MjModelPtr model = LoadModelFromString(kMeshSiteXml);
  ASSERT_THAT(model.get(), NotNull());
  MjDataPtr data = MakeData(model);
  int plain = mj_name2id(model.get(), mjOBJ_SITE, "plain");
  int mesh = mj_name2id(model.get(), mjOBJ_SITE, "mesh");
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(model->site_pos[3 * plain + j], model->site_pos[3 * mesh + j]);
  }
  for (int j = 0; j < 4; ++j) {
    EXPECT_EQ(model->site_quat[4 * plain + j], model->site_quat[4 * mesh + j]);
  }
  for (int i = 0; i < model->nv; ++i) {
    data->qvel[i] = 0.1 * (i + 1);
    data->qfrc_applied[i] = 0.2 * (i + 1);
  }
  mj_forward(model.get(), data.get());
  for (int sensor = 0; sensor < model->nsensor; sensor += 2) {
    int adr1 = model->sensor_adr[sensor];
    int adr2 = model->sensor_adr[sensor + 1];
    for (int j = 0; j < model->sensor_dim[sensor]; ++j) {
      EXPECT_NEAR(data->sensordata[adr1 + j], data->sensordata[adr2 + j], 1e-6);
    }
  }
}

TEST_F(MjvSceneTest, MeshSitePreservesVisualPoseAndVolume) {
  MjModelPtr model = LoadModelFromString(kMeshSiteXml);
  ASSERT_THAT(model.get(), NotNull());
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());
  int site = mj_name2id(model.get(), mjOBJ_SITE, "mesh");
  int geom = mj_name2id(model.get(), mjOBJ_GEOM, "reference");
  InitSceneObjects(model.get());
  mjv_updateScene(model.get(), data.get(), &opt_, &pert_, &cam_, mjCAT_ALL,
                  &scn_);
  const mjvGeom* mesh_site = nullptr;
  const mjvGeom* mesh_geom = nullptr;
  for (int i = 0; i < scn_.ngeom; ++i) {
    const mjvGeom* item = scn_.geoms + i;
    if (item->objtype == mjOBJ_SITE && item->objid == site) mesh_site = item;
    if (item->objtype == mjOBJ_GEOM && item->objid == geom) mesh_geom = item;
  }
  ASSERT_THAT(mesh_site, NotNull());
  ASSERT_THAT(mesh_geom, NotNull());
  for (int j = 0; j < 3; ++j) {
    EXPECT_NEAR(mesh_site->pos[j], mesh_geom->pos[j], 1e-6);
  }
  for (int j = 0; j < 9; ++j) {
    EXPECT_NEAR(mesh_site->mat[j], mesh_geom->mat[j], 1e-6);
  }
  EXPECT_EQ(
      mj_insideSite(model.get(), data.get(), site, data->geom_xpos + 3 * geom),
      1);
  mjtNum outside[3] = {100, 100, 100};
  EXPECT_EQ(mj_insideSite(model.get(), data.get(), site, outside), 0);
  FreeSceneObjects();
}

TEST_F(MjvSceneTest, TendonWrapGeodesicLength) {
  constexpr char kWrapXml[] = R"(
  <mujoco>
    <worldbody>
      <site name="0a" pos="0 0 1"/>
      <geom name="0g" type="sphere" size=".1" pos="0 0 0.5"/>
      <site name="0b" pos="0 .05 .15"/>

      <site name="1a" pos=".4 0 1"/>
      <geom name="1g" type="sphere" size=".1" pos=".4 0 0.5"/>
      <site name="1s" pos=".52 0 .5"/>
      <site name="1b" pos=".25 0 .15"/>

      <site name="2a" pos="-.4 0 1"/>
      <geom name="2g" type="sphere" size=".1" pos="-.4 0 0.5"/>
      <site name="2s" pos="-.49 0 .5"/>
      <site name="2b" pos="-.4 0 .15"/>

      <site name="3a" pos="0 1 1"/>
      <geom name="3g" type="cylinder" size=".05 .2" zaxis="0 1 .2" pos="0 1 0.5"/>
      <site name="3s" pos=".12 1 .5"/>
      <site name="3b" pos="-.15 1.1 .15"/>
    </worldbody>
    <tendon>
      <spatial>
        <site site="0a"/>
        <geom geom="0g"/>
        <site site="0b"/>
      </spatial>
      <spatial>
        <site site="1a"/>
        <geom geom="1g" sidesite="1s"/>
        <site site="1b"/>
      </spatial>
      <spatial>
        <site site="2a"/>
        <geom geom="2g" sidesite="2s"/>
        <site site="2b"/>
      </spatial>
      <spatial>
        <site site="3a"/>
        <geom geom="3g" sidesite="3s"/>
        <site site="3b"/>
      </spatial>
    </tendon>
  </mujoco>
  )";

  MjModelPtr model = LoadModelFromString(kWrapXml);
  ASSERT_THAT(model.get(), NotNull());
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());

  InitSceneObjects(model.get());
  mjv_updateScene(model.get(), data.get(), &opt_, &pert_, &cam_, mjCAT_ALL,
                  &scn_);

  for (int i = 0; i < model->ntendon; ++i) {
    mjtNum vis_length = 0;
    for (int g = 0; g < scn_.ngeom; ++g) {
      const mjvGeom* geom = scn_.geoms + g;
      if (geom->objtype == mjOBJ_TENDON && geom->objid == i) {
        vis_length += 2 * geom->size[2];
      }
    }
    EXPECT_NEAR(vis_length, data->ten_length[i], 1e-3) << "tendon=" << i;
  }

  FreeSceneObjects();
}

}  // namespace
}  // namespace mujoco
