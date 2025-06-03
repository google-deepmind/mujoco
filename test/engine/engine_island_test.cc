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
  int nidof   = data->nidof;

  // 4 dofs, 12 constraints, 2 islands
  EXPECT_EQ(nv, 4);
  EXPECT_EQ(nidof, 3);
  EXPECT_EQ(nefc, 12);  // 3 pyramidal contacts
  EXPECT_EQ(nisland, 2);

  // the islands begin at dofs 0 and 1
  EXPECT_THAT(AsVector(data->island_idofadr, nisland), ElementsAre(0, 1));

  // number of dofs in the 2 islands
  EXPECT_THAT(AsVector(data->island_nv, nisland), ElementsAre(1, 2));

  // dof 0 in    island 0
  // dof 1 in no island
  // dofs 2,3 in island 1
  EXPECT_THAT(AsVector(data->dof_island, nv), ElementsAre(0, -1, 1, 1));

  // dof 0 constitutes first island
  // dofs 2, 3 are the second island
  // last index is unassigned since dof 1 is unconstrained
  EXPECT_THAT(AsVector(data->map_idof2dof, nv), ElementsAre(0, 2, 3, 1));

  // dof 0 constitutes first island
  // dofs 1 is unassigned
  // dofs 2, 3 are second island
  EXPECT_THAT(AsVector(data->map_dof2idof, nv), ElementsAre(0, 3, 1, 2));

  // island 0 starts at constraint 0
  // island 1 starts at constraint 4
  EXPECT_THAT(AsVector(data->island_iefcadr, nisland), ElementsAre(0, 4));

  // number of constraints in the 2 islands
  EXPECT_THAT(AsVector(data->island_nefc, nisland), ElementsAre(4, 8));

  // first contact (4 constraints) is in island 0
  // second contact (8 constraints) is in island 1
  EXPECT_THAT(AsVector(data->efc_island, nefc),
              ElementsAre(0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1));

  // index lists for islands 0 and 1
  EXPECT_THAT(AsVector(data->map_iefc2efc, nefc),
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
  nidof          = data->nidof;

  EXPECT_EQ(nisland, 3);
  EXPECT_EQ(nidof, 4);
  EXPECT_THAT(AsVector(data->island_idofadr, nisland), ElementsAre(0, 1, 3));
  EXPECT_THAT(AsVector(data->island_nv, nisland), ElementsAre(1, 2, 1));
  EXPECT_THAT(AsVector(data->dof_island, nv), ElementsAre(0, 1, 1, 2));
  EXPECT_THAT(AsVector(data->map_idof2dof, nv), ElementsAre(0, 1, 2, 3));
  EXPECT_THAT(AsVector(data->map_dof2idof, nv), ElementsAre(0, 1, 2, 3));
  EXPECT_THAT(AsVector(data->island_iefcadr, nisland), ElementsAre(0, 4, 8));
  EXPECT_THAT(AsVector(data->island_nefc, nisland), ElementsAre(4, 4, 4));
  EXPECT_THAT(AsVector(data->efc_island, nefc),
              ElementsAre(0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2));
  EXPECT_THAT(AsVector(data->map_iefc2efc, nefc),
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
  EXPECT_EQ(data1->nidof, data2->nidof);
  EXPECT_EQ(data1->nefc, data2->nefc);
  EXPECT_EQ(data1->nisland, data2->nisland);
  EXPECT_EQ(data1->nefc, data2->nefc);
  EXPECT_EQ(AsVector(data1->island_idofadr, nisland),
            AsVector(data2->island_idofadr, nisland));
  EXPECT_EQ(AsVector(data1->island_nv, nisland),
            AsVector(data2->island_nv, nisland));
  EXPECT_EQ(AsVector(data1->dof_island, nv),
            AsVector(data2->dof_island, nv));
  EXPECT_EQ(AsVector(data1->map_idof2dof, nv),
            AsVector(data2->map_idof2dof, nv));
  EXPECT_EQ(AsVector(data1->map_dof2idof, nv),
            AsVector(data2->map_dof2idof, nv));
  EXPECT_EQ(AsVector(data1->island_iefcadr, nisland),
            AsVector(data2->island_iefcadr, nisland));
  EXPECT_EQ(AsVector(data1->island_nefc, nisland),
            AsVector(data2->island_nefc, nisland));
  EXPECT_EQ(AsVector(data1->efc_island, nefc),
            AsVector(data2->efc_island, nefc));
  EXPECT_EQ(AsVector(data1->map_iefc2efc, nefc),
            AsVector(data2->map_iefc2efc, nefc));
  EXPECT_EQ(AsVector(data1->map_efc2iefc, nefc),
            AsVector(data2->map_efc2iefc, nefc));

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

static const char* const k2H100Path = "engine/testdata/island/2humanoid100.xml";

TEST_F(IslandTest, IslandJacobian) {
  for (const char* local_path : {kIlslandEfcPath, k2H100Path}) {
    const std::string xml_path = GetTestDataFilePath(local_path);
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
    int jac0 = m->opt.jacobian;
    mjData* d = mj_makeData(m);

    for (mjtNum t_stop : {0.0, 0.2, 2.0}) {
      while (d->time < t_stop) {
        mj_step(m, d);
      }

      for (mjtJacobian jac : {mjJAC_DENSE, mjJAC_SPARSE}) {
        m->opt.jacobian = jac;
        mj_forward(m, d);

        int nv = m->nv;
        int nefc = d->nefc;
        int nisland = d->nisland;
        int nidof = d->nidof;

        mjtNum* J = (mjtNum*)mju_malloc(sizeof(mjtNum) * nefc * nv);
        mjtNum* iJ = (mjtNum*)mju_malloc(sizeof(mjtNum) * nefc * nidof);

        // get local dense Jacobian
        if (jac == mjJAC_DENSE) {
          mju_copy(J, d->efc_J, nefc * nv);
          mju_copy(iJ, d->iefc_J, nefc * nidof);
        } else {
          mju_sparse2dense(J, d->efc_J, nefc, nv, d->efc_J_rownnz,
                           d->efc_J_rowadr, d->efc_J_colind);
        }

        // compare random access in efc_J to contiguous memory in iefc_J
        for (int island=0; island < nisland; island++) {
          int idof = d->island_idofadr[island];
          int iefc = d->island_iefcadr[island];
          int nefc_island = d->island_nefc[island];
          int nv_island = d->island_nv[island];

          // === test J

          // get pointer to J_island, dense (nefc_island x nv_island) submatrix
          mjtNum* J_island;
          if (jac == mjJAC_DENSE) {
            // point to starting address of island in efc_J
            J_island = iJ + iefc * nidof;
          } else {
            // dense copy of island in iJ (here used as scratch)
            mju_sparse2dense(iJ, d->iefc_J, nefc_island, nv_island,
                             d->iefc_J_rownnz + iefc,
                             d->iefc_J_rowadr + iefc,
                             d->iefc_J_colind);
            J_island = iJ;
          }

          // sequential memory in J_island equals random access memory in J
          for (int i=0; i < nefc_island; i++) {
            for (int j=0; j < nv_island; j++) {
              int efc = d->map_iefc2efc[iefc + i];
              int dof = d->map_idof2dof[idof + j];
              EXPECT_EQ(J_island[i * nv_island + j], J[efc * nv + dof]);
            }
          }

          // === test JT (if sparse)

          // get pointer to J_island, dense (nefc_island x nv_island) submatrix
          if (jac == mjJAC_SPARSE) {
            // dense copy of island in iJ (here used as scratch)
            mju_sparse2dense(iJ, d->iefc_JT, nv_island, nefc_island,
                             d->iefc_JT_rownnz + idof,
                             d->iefc_JT_rowadr + idof,
                             d->iefc_JT_colind);
            J_island = iJ;

            // sequential memory in J_island equals random access memory in J
            for (int i=0; i < nv_island; i++) {
              for (int j=0; j < nefc_island; j++) {
                int dof = d->map_idof2dof[idof + i];
                int efc = d->map_iefc2efc[iefc + j];
                EXPECT_EQ(J_island[i * nefc_island + j], J[efc * nv + dof]);
              }
            }
          }
        }

        mju_free(iJ);
        mju_free(J);
      }

      // reset opt.jacobian to initial value
      m->opt.jacobian = jac0;
    }

    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

TEST_F(IslandTest, IslandInertia) {
  for (const char* local_path : {kIlslandEfcPath, k2H100Path}) {
    const std::string xml_path = GetTestDataFilePath(local_path);
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
    int nv = m->nv;
    mjData* d = mj_makeData(m);
    mjtNum* M = (mjtNum*)mju_malloc(sizeof(mjtNum) * nv * nv);

    for (mjtNum t_stop : {0.0, 0.2, 2.0}) {
      while (d->time < t_stop) {
        mj_step(m, d);
      }
      mj_forward(m, d);

      int nisland = d->nisland;

      // get dense inertia (lower only)
      mj_fullM(m, M, d->qM);

      // compare iM sub-matrix to full M
      for (int island=0; island < nisland; island++) {
        int nvi = d->island_nv[island];
        mjtNum* Mi = (mjtNum*)mju_malloc(sizeof(mjtNum) * nvi * nvi);

        int adr = d->island_idofadr[island];
        mju_sparse2dense(Mi, d->iM, nvi, nvi,
                         d->iM_rownnz + adr,
                         d->iM_rowadr + adr,
                         d->iM_colind);

        // compare Mi to M (lower triangle only)
        for (int i=0; i < nvi; i++) {
          for (int j=0; j <= i; j++) {
            int dofi = d->map_idof2dof[adr + j];
            int dofj = d->map_idof2dof[adr + i];
            EXPECT_EQ(Mi[i * nvi + j], M[dofi * nv + dofj]);
          }
        }
        mju_free(Mi);
      }
    }

    mju_free(M);
    mj_deleteData(d);
    mj_deleteModel(m);
  }
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
