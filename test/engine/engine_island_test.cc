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

// Tests for engine/engine_island.c.

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_island.h"
#include "src/engine/engine_util_sparse.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ElementsAre;
using IslandTest = MujocoTest;

std::vector<int> AsVector(const int* array, int n) {
  return std::vector<int>(array, array + n);
}

TEST_F(IslandTest, FloodFillSingleton) {
  // adjacency matrix for the graph  0   1   2
  //                                 U       U
  // (3 singletons, 0 and 2 have self-edges)
  mjtNum mat[9] = {
    1, 0, 0,
    0, 0, 0,
    0, 0, 1
  };
  constexpr int nr = 3;
  constexpr int nnz = 2;
  int rownnz[nr];
  int rowadr[nr];
  int colind[nnz];
  mjtNum res[nnz];  // unused
  mju_dense2sparse(res, mat, nr, nr, rownnz, rowadr, colind, nnz);

  // outputs / scratch
  int island[nr];
  int scratch[2*nr];

  // flood fill
  int nisland = mj_floodFill(island, nr, rownnz, rowadr, colind, scratch);

  EXPECT_EQ(nisland, 2);
  EXPECT_THAT(island, ElementsAre(0, -1, 1));
}


TEST_F(IslandTest, FloodFill1) {
  // adjacency matrix for the graph  0 - 1 - 2
  mjtNum mat[9] = {
    0, 1, 0,
    1, 0, 1,
    0, 1, 0
  };
  constexpr int nr = 3;
  constexpr int nnz = 4;
  int rownnz[nr];
  int rowadr[nr];
  int colind[nnz];
  mjtNum res[nnz];  // unused
  mju_dense2sparse(res, mat, nr, nr, rownnz, rowadr, colind, nnz);

  // outputs / stack
  int island[nr];
  int stack[nnz];

  int nisland = mj_floodFill(island, nr, rownnz, rowadr, colind, stack);

  EXPECT_EQ(nisland, 1);
  EXPECT_THAT(island, ElementsAre(0, 0, 0));
}


TEST_F(IslandTest, FloodFill2) {
  //  adjacency matrix for the graph   6 – 1 – 4   0 – 3 – 5 – 2
  mjtNum mat[49] = {
    0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 1,
    0, 0, 0, 0, 0, 1, 0,
    1, 0, 0, 0, 0, 1, 0,
    0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 1, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0,
  };
  constexpr int nr = 7;
  constexpr int nnz = 10;
  int rownnz[nr];
  int rowadr[nr];
  int colind[nnz];
  mjtNum res[nnz];  // unused
  mju_dense2sparse(res, mat, nr, nr, rownnz, rowadr, colind, nnz);

  // outputs / stack
  int island[nr];
  int stack[nnz];

  int nisland = mj_floodFill(island, nr, rownnz, rowadr, colind, stack);

  EXPECT_EQ(nisland, 2);
  EXPECT_THAT(island, ElementsAre(0, 1, 0, 0, 1, 0, 1));
}


TEST_F(IslandTest, FloodFill3a) {
  //  adjacency matrix for the graph   0   2   1 – 3
  //                                       U
  mjtNum mat[16] = {
    0, 0, 0, 0,
    0, 0, 0, 1,
    0, 0, 1, 0,
    0, 1, 0, 0,
  };
  constexpr int nr = 4;
  constexpr int nnz = 3;
  int rownnz[nr];
  int rowadr[nr];
  int colind[nnz];
  mjtNum res[nnz];  // unused
  mju_dense2sparse(res, mat, nr, nr, rownnz, rowadr, colind, nnz);

  // outputs / stack
  int island[nr];
  int stack[nnz];

  int nisland = mj_floodFill(island, nr, rownnz, rowadr, colind, stack);

  EXPECT_EQ(nisland, 2);
  EXPECT_THAT(island, ElementsAre(-1, 0, 1, 0));
}


TEST_F(IslandTest, FloodFill3b) {
  /*
    adjacency matrix for the graph   1 – 2   3   4 – 5
                                     U           | \ |
                                                 0 – 6
  */
  mjtNum mat[49] = {
    0, 0, 0, 0, 1, 0, 1,
    0, 1, 1, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 1, 1,
    0, 0, 0, 0, 1, 0, 1,
    1, 0, 0, 0, 1, 1, 0,
  };
  constexpr int nr = 7;
  constexpr int nnz = 13;
  int rownnz[nr];
  int rowadr[nr];
  int colind[nnz];
  mjtNum res[nnz];  // unused
  mju_dense2sparse(res, mat, nr, nr, rownnz, rowadr, colind, nnz);

  // outputs / stack
  int island[nr];
  int stack[nnz];

  int nisland = mj_floodFill(island, nr, rownnz, rowadr, colind, stack);

  EXPECT_EQ(nisland, 2);
  EXPECT_THAT(island, ElementsAre(0, 1, 1, -1, 0, 0, 0));
}

static const char* const kAbacusPath =
    "engine/testdata/island/abacus.xml";

TEST_F(IslandTest, Abacus) {
  const std::string xml_path = GetTestDataFilePath(kAbacusPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);

  // disable gravity
  model->opt.disableflags |= mjDSBL_GRAVITY;

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // no islands at qpos0
  EXPECT_EQ(data->nisland, 0);

  // push bead 0 to the left and bead 2 to the right until there are 3 contacts
  data->qfrc_applied[0] = -1;
  data->qfrc_applied[2] = 1;
  while (data->ncon != 3) {
    mj_step(model, data);
  }

  // sizes
  int nv      = model->nv;
  int nefc    = data->nefc;
  int nisland = data->nisland;

  // 4 dofs, 12 constraints, 2 islands
  EXPECT_EQ(nv, 4);
  EXPECT_EQ(nefc, 12);  // 3 pyramidal contacts
  EXPECT_EQ(nisland, 2);

  // the islands begin at dofs 0 and 1
  EXPECT_THAT(AsVector(data->island_dofadr, nisland), ElementsAre(0, 1));

  // number of dofs in the 2 islands
  EXPECT_THAT(AsVector(data->island_dofnum, nisland), ElementsAre(1, 2));

  // dof 0 in    island 0
  // dof 1 in no island
  // dofs 2,3 in island 1
  EXPECT_THAT(AsVector(data->dof_island, nv), ElementsAre(0, -1, 1, 1));

  // dof 0 constitutes first island
  // dofs 2, 3 are the second island
  // last index is unassigned since dof 1 is unconstrained
  EXPECT_THAT(AsVector(data->island_dofind, nv), ElementsAre(0, 2, 3, -1));

  // dof 0 constitutes first island
  // dofs 1 is unassigned
  // dofs 2, 3 are second island
  EXPECT_THAT(AsVector(data->dof_islandind, nv), ElementsAre(0, -1, 0, 1));

  // island 0 starts at constraint 0
  // island 1 starts at constraint 4
  EXPECT_THAT(AsVector(data->island_efcadr, nisland), ElementsAre(0, 4));

  // number of constraints in the 2 islands
  EXPECT_THAT(AsVector(data->island_efcnum, nisland), ElementsAre(4, 8));

  // first contact (4 constraints) is in island 0
  // second contact (8 constraints) is in island 1
  EXPECT_THAT(AsVector(data->efc_island, nefc),
              ElementsAre(0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1));

  // index lists for islands 0 and 1
  EXPECT_THAT(AsVector(data->island_efcind, nefc),
              ElementsAre(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11));

  // reset, push 0 to the left, 3 to the right, 1,2 to the middle
  mj_resetData(model, data);
  data->qfrc_applied[0] = -1;
  data->qfrc_applied[1] = 1;
  data->qfrc_applied[2] = -1;
  data->qfrc_applied[3] = 1;

  // simulate until there are 3 contacts
  while (data->ncon != 3) {
    mj_step(model, data);
  }

  // local variables
  nefc           = data->nefc;
  nisland        = data->nisland;

  EXPECT_EQ(nisland, 3);
  EXPECT_THAT(AsVector(data->island_dofadr, nisland), ElementsAre(0, 1, 3));
  EXPECT_THAT(AsVector(data->island_dofnum, nisland), ElementsAre(1, 2, 1));
  EXPECT_THAT(AsVector(data->dof_island, nv), ElementsAre(0, 1, 1, 2));
  EXPECT_THAT(AsVector(data->island_dofind, nv), ElementsAre(0, 1, 2, 3));
  EXPECT_THAT(AsVector(data->dof_islandind, nv), ElementsAre(0, 0, 1, 0));
  EXPECT_THAT(AsVector(data->island_efcadr, nisland), ElementsAre(0, 4, 8));
  EXPECT_THAT(AsVector(data->island_efcnum, nisland), ElementsAre(4, 4, 4));
  EXPECT_THAT(AsVector(data->efc_island, nefc),
              ElementsAre(0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2));
  EXPECT_THAT(AsVector(data->island_efcind, nefc),
              ElementsAre(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11));

  mj_deleteData(data);
  mj_deleteModel(model);
}

static const char* const kTendonWrapPath =
    "engine/testdata/island/tendon_wrap.xml";

TEST_F(IslandTest, DenseSparse) {
  const std::string xml_path = GetTestDataFilePath(kTendonWrapPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data1 = mj_makeData(model);
  mjData* data2 = mj_makeData(model);

  // dense
  model->opt.jacobian = mjJAC_DENSE;
  while (!data1->nefc) {
    mj_step(model, data1);
  }

  // sparse
  model->opt.jacobian = mjJAC_SPARSE;
  while (!data2->nefc) {
    mj_step(model, data2);
  }

  // sizes
  int nv      = model->nv;
  int nefc    = data1->nefc;
  int nisland = data1->nisland;

  // expect sparse and dense to be identical
  EXPECT_EQ(data1->nefc, data2->nefc);
  EXPECT_EQ(data1->nisland, data2->nisland);
  EXPECT_EQ(data1->nefc, data2->nefc);
  EXPECT_EQ(AsVector(data1->island_dofadr, nisland),
            AsVector(data2->island_dofadr, nisland));
  EXPECT_EQ(AsVector(data1->island_dofnum, nisland),
            AsVector(data2->island_dofnum, nisland));
  EXPECT_EQ(AsVector(data1->dof_island, nv),
            AsVector(data2->dof_island, nv));
  EXPECT_EQ(AsVector(data1->island_dofind, nv),
            AsVector(data2->island_dofind, nv));
  EXPECT_EQ(AsVector(data1->dof_islandind, nv),
            AsVector(data2->dof_islandind, nv));
  EXPECT_EQ(AsVector(data1->island_efcadr, nisland),
            AsVector(data2->island_efcadr, nisland));
  EXPECT_EQ(AsVector(data1->island_efcnum, nisland),
            AsVector(data2->island_efcnum, nisland));
  EXPECT_EQ(AsVector(data1->efc_island, nefc),
            AsVector(data2->efc_island, nefc));
  EXPECT_EQ(AsVector(data1->island_efcind, nefc),
            AsVector(data2->island_efcind, nefc));

  mj_deleteData(data2);
  mj_deleteData(data1);
  mj_deleteModel(model);
}

static const char* const kIlslandEfcPath =
    "engine/testdata/island/island_efc.xml";

TEST_F(IslandTest, IslandEfc) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  while (data->time < 0.2) {
    mj_step(model, data);
  }

  // expect island structure to correspond to comment at top of xml
  EXPECT_EQ(data->nisland, 4);
  EXPECT_EQ(data->ne, 7);
  EXPECT_EQ(data->nf, 2);
  EXPECT_EQ(data->nl, 1);
  EXPECT_EQ(data->nefc, 30);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(IslandTest, IslandEfcElliptic) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  model->opt.cone = mjCONE_ELLIPTIC;
  while (data->time < 0.2) {
    mj_step(model, data);
  }
  mj_forward(model, data);

  EXPECT_EQ(data->nisland, 4);
  EXPECT_EQ(data->ne, 7);
  EXPECT_EQ(data->nf, 2);
  EXPECT_EQ(data->nl, 1);
  EXPECT_EQ(data->nefc, 25);

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
