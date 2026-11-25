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

#include "studio/sim/batched_model.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "testing/base/public/gunit.h"
#include <mujoco/mujoco.h>
#include "studio/sim/model_holder.h"

namespace mujoco::studio {
namespace {

constexpr const char* kTestXml = R"xml(
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" pos="0 0 -1" size="0 0 1" rgba="0 0 1 1"/>
    <body name="box_body" pos="0 0 0">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <geom name="box" type="box" size="0.2 0.2 0.2" rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
)xml";

std::unique_ptr<ModelHolder> LoadTestModel() {
  char error[256] = "";
  mjSpec* spec = mj_parseXMLString(kTestXml, nullptr, error, sizeof(error));
  EXPECT_NE(spec, nullptr) << error;
  return ModelHolder::FromSpec(spec);
}

TEST(BatchedModelFieldTest, IsBatchedModelFieldMatchesAllowlist) {
  EXPECT_TRUE(IsBatchedModelField("geom_rgba"));
  EXPECT_TRUE(IsBatchedModelField("geom_size"));
  EXPECT_TRUE(IsBatchedModelField("body_pos"));
  EXPECT_TRUE(IsBatchedModelField("qpos0"));
  EXPECT_TRUE(IsBatchedModelField("jnt_pos"));
  EXPECT_TRUE(IsBatchedModelField("jnt_axis"));
  EXPECT_TRUE(IsBatchedModelField("site_group"));
  EXPECT_TRUE(IsBatchedModelField("site_matid"));
  EXPECT_TRUE(IsBatchedModelField("cam_pos"));
  EXPECT_TRUE(IsBatchedModelField("cam_fovy"));
  EXPECT_TRUE(IsBatchedModelField("light_pos"));
  EXPECT_TRUE(IsBatchedModelField("light_diffuse"));
  EXPECT_TRUE(IsBatchedModelField("light_active"));
  EXPECT_TRUE(IsBatchedModelField("mat_rgba"));
  EXPECT_TRUE(IsBatchedModelField("mat_texid"));
  EXPECT_TRUE(IsBatchedModelField("mat_texuniform"));
  EXPECT_TRUE(IsBatchedModelField("mat_reflectance"));
  EXPECT_TRUE(IsBatchedModelField("mat_metallic"));
  EXPECT_TRUE(IsBatchedModelField("mat_roughness"));
  EXPECT_TRUE(IsBatchedModelField("tendon_matid"));
  EXPECT_TRUE(IsBatchedModelField("tendon_group"));
  EXPECT_TRUE(IsBatchedModelField("tendon_width"));
  EXPECT_TRUE(IsBatchedModelField("tendon_rgba"));
  EXPECT_TRUE(IsBatchedModelField("geom_group"));
  EXPECT_FALSE(IsBatchedModelField("ngeom"));
  EXPECT_FALSE(IsBatchedModelField("mesh_vert"));
  EXPECT_FALSE(IsBatchedModelField("body_parentid"));
  EXPECT_FALSE(IsBatchedModelField("nonexistent_field"));
}

TEST(BatchedModelFieldTest, ApplyTracksShapeChangesAndClearsSameFrame) {
  std::unique_ptr<ModelHolder> holder = LoadTestModel();
  ASSERT_TRUE(holder->ok()) << holder->error();
  mjModel* m = holder->model();
  const int box_id = mj_name2id(m, mjOBJ_GEOM, "box");
  ASSERT_GE(box_id, 0);
  ASSERT_NE(m->geom_sameframe[box_id], mjSAMEFRAME_NONE);

  std::vector<mjtNum> geom_pos(2 * 3 * m->ngeom, 0.0);
  std::vector<mjtNum> geom_size(2 * 3 * m->ngeom, 0.0);
  for (int sim = 0; sim < 2; ++sim) {
    std::copy(m->geom_pos, m->geom_pos + 3 * m->ngeom, geom_pos.data() + sim * 3 * m->ngeom);
    std::copy(m->geom_size, m->geom_size + 3 * m->ngeom, geom_size.data() + sim * 3 * m->ngeom);
  }
  geom_pos[1 * 3 * m->ngeom + 3 * box_id + 0] = 0.75;
  geom_size[1 * 3 * m->ngeom + 3 * box_id + 0] = 0.45;

  const BatchedModelField fields[2] = {
      {"geom_pos", geom_pos.data(), static_cast<size_t>(3 * m->ngeom) * sizeof(mjtNum)},
      {"geom_size", geom_size.data(), static_cast<size_t>(3 * m->ngeom) * sizeof(mjtNum)},
  };

  std::vector<std::byte> saved;
  SaveBatchedModelFields(m, fields, &saved);
  const mjtNum orig_rbound = m->geom_rbound[box_id];

  std::vector<std::byte> shape_row;
  EXPECT_TRUE(ApplyBatchedModelFields(m, fields, 0, &shape_row));
  EXPECT_FALSE(ApplyBatchedModelFields(m, fields, 0, &shape_row));
  EXPECT_EQ(m->geom_sameframe[box_id], mjSAMEFRAME_NONE);

  EXPECT_TRUE(ApplyBatchedModelFields(m, fields, 1, &shape_row));
  EXPECT_NEAR(m->geom_pos[3 * box_id + 0], 0.75, 1e-6);
  EXPECT_NEAR(m->geom_size[3 * box_id + 0], 0.45, 1e-6);
  EXPECT_NEAR(m->geom_rbound[box_id], std::sqrt(0.45 * 0.45 + 0.2 * 0.2 + 0.2 * 0.2), 1e-5);

  EXPECT_TRUE(RestoreBatchedModelFields(m, fields, saved));
  EXPECT_NEAR(m->geom_size[3 * box_id + 0], 0.2, 1e-6);
  EXPECT_NEAR(m->geom_rbound[box_id], orig_rbound, 1e-5);
}

TEST(BatchedModelFieldTest, RestoreReturnsFalseOnSizeMismatch) {
  std::unique_ptr<ModelHolder> holder = LoadTestModel();
  ASSERT_TRUE(holder->ok()) << holder->error();
  mjModel* m = holder->model();

  const std::vector<float> dummy(4 * m->ngeom, 0.0f);
  const BatchedModelField fields[1] = {
      {"geom_rgba", dummy.data(), static_cast<size_t>(4 * m->ngeom) * sizeof(float)},
  };

  std::vector<std::byte> saved;
  SaveBatchedModelFields(m, fields, &saved);
  ASSERT_EQ(saved.size(), static_cast<size_t>(4 * m->ngeom) * sizeof(float));

  const float original = m->geom_rgba[0];
  m->geom_rgba[0] = 0.25f;
  EXPECT_TRUE(RestoreBatchedModelFields(m, fields, saved));
  EXPECT_FLOAT_EQ(m->geom_rgba[0], original);

  saved.pop_back();
  m->geom_rgba[0] = 0.25f;
  EXPECT_FALSE(RestoreBatchedModelFields(m, fields, saved));
  EXPECT_FLOAT_EQ(m->geom_rgba[0], 0.25f);
}

}  // namespace
}  // namespace mujoco::studio
