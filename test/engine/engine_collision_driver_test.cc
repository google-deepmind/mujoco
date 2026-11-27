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

// Tests for engine/engine_collision_driver.c.

#include "src/engine/engine_collision_driver.h"
#include "src/engine/engine_collision_nonconvex.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using MjCollisionTest = MujocoTest;
using GeomPair = std::pair<std::string, std::string>;
using ::testing::ElementsAre;
using ::testing::IsEmpty;
using ::testing::NotNull;

// Returns a sorted list of pairs of colliding geom names, where each pair of
// geom names is sorted.
static std::vector<GeomPair> colliding_pairs(const mjModel* model,
                                             const mjData* data) {
  std::vector<GeomPair> result;
  for (int i = 0; i < data->ncon; i++) {
    std::string geom1 = mj_id2name(model, mjOBJ_GEOM, data->contact[i].geom[0]);
    std::string geom2 = mj_id2name(model, mjOBJ_GEOM, data->contact[i].geom[1]);
    result.push_back(GeomPair(std::min(geom1, geom2), std::max(geom1, geom2)));
  }
  std::sort(result.begin(), result.end());
  return result;
}

TEST_F(MjCollisionTest, AllCollisions) {
  static const char* const kModelFilePath = "engine/testdata/collisions.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // mjCOL_ALL is the default
  mj_fwdPosition(model, data);
  EXPECT_THAT(colliding_pairs(model, data),
              ElementsAre(GeomPair("box", "sphere_collides"),
                          GeomPair("box", "sphere_predefined")));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionTest, EmptyModel) {
  char error[1024];
  MjModelPtr model = LoadModelFromString("<mujoco/>", error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  mj_fwdPosition(model.get(), data.get());
  EXPECT_THAT(colliding_pairs(model.get(), data.get()), IsEmpty());
}

TEST_F(MjCollisionTest, ContactCount) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="plane" size="5 5 .01"/>
      </body>
      <body pos="0 0 0.9">
        <freejoint/>
        <geom type="sphere" size="1" pos="-1 -1 0"/>
        <geom type="sphere" size="1" pos="-1  1 0"/>
        <geom type="sphere" size="1" pos=" 1 -1 0"/>
        <geom type="sphere" size="1" pos=" 1  1 0"/>
        <geom type="sphere" size="1" pos="-2 -2 0"/>
        <geom type="sphere" size="1" pos="-2  2 0"/>
        <geom type="sphere" size="1" pos=" 2 -2 0"/>
        <geom type="sphere" size="1" pos=" 2  2 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // there are 8 spheres, all touching the floor
  EXPECT_EQ(d->ncon, 8);
}

TEST_F(MjCollisionTest, InGapContactsMultiGeomBody) {
  // In-gap contacts survive mid-phase BVH pruning. Both bodies
  // have a "far" geom that pulls the BVH root box away, so detecting the A-B
  // pair requires the descent filter to account for gap (body_margin).
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 1">
        <freejoint/>
        <geom name="A" type="box" size=".005 .01 .3" pos="-.015 0 0" gap=".004"/>
        <geom name="far1" type="box" size=".005 .01 .3" pos="-.2 0 0"/>
      </body>
      <body pos="0 0 1">
        <freejoint/>
        <geom name="B" type="box" size=".005 .01 .3" pos="-.0022 0 0" gap=".004"/>
        <geom name="far2" type="box" size=".005 .01 .3" pos=".1 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // geoms A and B are separated by 2.8mm, within the 8mm combined gap:
  // inactive contacts are expected
  EXPECT_GT(d->ncon, 0);
  for (int i = 0; i < d->ncon; i++) {
    EXPECT_GT(d->contact[i].dist, 0);
    EXPECT_LT(d->contact[i].dist, 0.008);
    EXPECT_EQ(d->contact[i].efc_address, -1);
  }
}

TEST_F(MjCollisionTest, MidphaseDescendsLargerTree) {
  // The small body sits inside the large body's root box but touches neither
  // leaf. Descending the larger tree first prunes immediately, so the small
  // body's children are never tested. The inertial offset makes BVH center ==
  // size, which zeroes the area if (center, size) is mistaken for (min, max).
  static constexpr char xml[] = R"(
  <mujoco>
    <visual>
      <global bvactive="true"/>
    </visual>
    <worldbody>
      <body>
        <freejoint/>
        <inertial pos="-1.1 -.1 -.1" mass="1" diaginertia="1 1 1"/>
        <geom type="box" size=".1 .1 .1" pos="-1 0 0"/>
        <geom type="box" size=".1 .1 .1" pos="1 0 0"/>
      </body>
      <body>
        <freejoint/>
        <geom type="box" size=".02 .02 .02" pos="-.05 0 0"/>
        <geom type="box" size=".02 .02 .02" pos=".05 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  mj_forward(m.get(), d.get());

  // only the two root nodes are active
  int bvhadr = m->body_bvhadr[2];
  EXPECT_TRUE(d->bvh_active[m->body_bvhadr[1]]);
  EXPECT_TRUE(d->bvh_active[bvhadr]);
  for (int i = 1; i < m->body_bvhnum[2]; i++) {
    EXPECT_FALSE(d->bvh_active[bvhadr + i]);
  }
}

TEST_F(MjCollisionTest, FilterParent) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 0">
        <freejoint/>
        <geom name="colliding1" size="1" pos="0 0 100"/>
        <body>
          <geom size="1"/>
          <body>
            <joint axis="1 0 0"/>
            <geom size="1" pos="0 0 50"/>
            <body>
              <geom name="colliding2" size="1" pos="0 0 99.5"/>
            </body>
          </body>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_fwdPosition(m.get(), d.get());

  // there should be zero contacts, because colliding1 and colliding2 are in
  // bodies that have a parent-child relationship, through welds
  EXPECT_EQ(d->ncon, 0);

  // when this filtering is disabled, the geoms should collide
  m->opt.disableflags |= mjDSBL_FILTERPARENT;
  mj_fwdPosition(m.get(), d.get());

  EXPECT_THAT(colliding_pairs(m.get(), d.get()),
              ElementsAre(GeomPair("colliding1", "colliding2")));
}

TEST_F(MjCollisionTest, FilterParentDoesntAffectWorldBody) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="colliding1" size="1" pos="0 0 100"/>
      <body pos="0 0 0">
        <joint axis="1 0 0"/>
        <geom name="colliding2" size="1" pos="0 0 99.5"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_fwdPosition(m.get(), d.get());

  // even though colliding1 and colliding2 are have a parent-child relationship,
  // they collide because colliding1 is in <worldbody>
  EXPECT_THAT(colliding_pairs(m.get(), d.get()),
              ElementsAre(GeomPair("colliding1", "colliding2")));
}

TEST_F(MjCollisionTest, FilterStaticFlex) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="1 1 .1"/>
      <body name="static">
        <geom name="static" size=".1"/>
      </body>
      <body name="rigid">
        <flexcomp name="rigid" type="grid" count="3 3 1" spacing=".1 .1 .1" dim="2" rigid="true"/>
      </body>
      <body name="mocap" mocap="true" pos=".05 .05 .001">
        <flexcomp name="mocap" type="grid" count="3 3 1" spacing=".1 .1 .1" dim="2" rigid="true"/>
      </body>
      <body name="ball" pos="0 0 .09">
        <freejoint/>
        <geom name="ball" size=".1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // the floor, the static geom and the two flexes overlap but have no dofs:
  // they collide only with the ball
  int ball = mj_name2id(m.get(), mjOBJ_GEOM, "ball");
  int nflexcon[2] = {0, 0};
  for (int i = 0; i < d->ncon; i++) {
    const mjContact& con = d->contact[i];
    EXPECT_TRUE(con.geom[0] == ball || con.geom[1] == ball);
    if (con.flex[1] >= 0) {
      nflexcon[con.flex[1]]++;
    }
  }
  EXPECT_GT(nflexcon[0], 0);
  EXPECT_GT(nflexcon[1], 0);
}

TEST_F(MjCollisionTest, FilterStaticFlexSelfCollision) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="a" pos="-1 0 0"/>
      <body name="b" pos="1 0 0"/>
      <body name="c" pos="0 -1 .01"/>
      <body name="d" pos="0 1 .01"/>
      <body pos="0 0 1">
        <freejoint/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <deformable>
      <flex name="rope" dim="1" radius=".01" body="a b c d" element="0 1 2 3">
        <edge stiffness="1"/>
      </flex>
    </deformable>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // the two elements of the flex cross, but its vertices are in static bodies
  EXPECT_EQ(d->ncon, 0);
}

TEST_F(MjCollisionTest, TestOBB) {
  mjtNum bvh1[6] = {-1, -1, -1, 1, 1, 1};
  mjtNum bvh2[6] = {-1, -1, -1, 1, 1, 1};
  mjtNum pos1[3] = {0, 0, 0};
  mjtNum mat1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  mjtNum pos2[3] = {1.71, 1.71, 0};  // just a little more than 1+sqrt(2)/2
  mjtNum mat2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  EXPECT_THAT(
      mj_collideOBB(bvh1, bvh2, pos1, mat1, pos2, mat2, 0, NULL, NULL, 0),
      true);

  // rotate by 45 degrees
  mat2[0] = 1. / mju_sqrt(2.);
  mat2[1] = -1. / mju_sqrt(2.);
  mat2[3] = 1. / mju_sqrt(2.);
  mat2[4] = 1. / mju_sqrt(2.);

  EXPECT_THAT(
      mj_collideOBB(bvh1, bvh2, pos1, mat1, pos2, mat2, 0, NULL, NULL, 0),
      false);
}

TEST_F(MjCollisionTest, TestOBBMatchesReference) {
  // Reference 6-axis SAT implementation matching the original Gottschalk logic.
  // Returns -1 if close to the separating boundary (ill-conditioned).
  auto ref_obb = [](const mjtNum aabb1[6], const mjtNum aabb2[6],
                    const mjtNum xpos1[3], const mjtNum xmat1[9],
                    const mjtNum xpos2[3], const mjtNum xmat2[9], mjtNum margin,
                    mjtNum eps) -> int {
    bool inf1[3] = {aabb1[3] >= mjMAXVAL, aabb1[4] >= mjMAXVAL,
                    aabb1[5] >= mjMAXVAL};
    bool inf2[3] = {aabb2[3] >= mjMAXVAL, aabb2[4] >= mjMAXVAL,
                    aabb2[5] >= mjMAXVAL};
    if ((inf1[0] && inf1[1] && inf1[2]) || (inf2[0] && inf2[1] && inf2[2])) {
      return 1;
    }
    const mjtNum* aabb[2] = {aabb1, aabb2};
    const mjtNum* xmat[2] = {xmat1, xmat2};
    const mjtNum* xpos[2] = {xpos1, xpos2};
    bool infinite[2] = {inf1[0] || inf1[1] || inf1[2],
                        inf2[0] || inf2[1] || inf2[2]};
    double xc[2][3], n[2][3][3];
    for (int i = 0; i < 2; ++i) {
      for (int r = 0; r < 3; ++r) {
        double v = xmat[i]
                       ? (static_cast<double>(xmat[i][3 * r + 0]) * aabb[i][0] +
                          static_cast<double>(xmat[i][3 * r + 1]) * aabb[i][1] +
                          static_cast<double>(xmat[i][3 * r + 2]) * aabb[i][2])
                       : static_cast<double>(aabb[i][r]);
        xc[i][r] = v + (xpos[i] ? static_cast<double>(xpos[i][r]) : 0.0);
        for (int c = 0; c < 3; ++c) {
          n[i][r][c] = xmat[i] ? static_cast<double>(xmat[i][3 * c + r])
                               : (r == c ? 1.0 : 0.0);
        }
      }
    }
    auto dot = [](const double a[3], const double b[3]) {
      return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
    bool near_boundary = false;
    for (int j = 0; j < 2; ++j) {
      if (infinite[1 - j]) continue;
      for (int k = 0; k < 3; ++k) {
        double proj[2], rad[2];
        for (int i = 0; i < 2; ++i) {
          proj[i] = dot(xc[i], n[j][k]);
          rad[i] = std::abs(aabb[i][3] * dot(n[i][0], n[j][k])) +
                   std::abs(aabb[i][4] * dot(n[i][1], n[j][k])) +
                   std::abs(aabb[i][5] * dot(n[i][2], n[j][k]));
        }
        double diff = std::abs(proj[1] - proj[0]) - (rad[0] + rad[1] + margin);
        if (std::abs(diff) < eps) {
          near_boundary = true;
        }
        if (diff > eps) {
          return 0;
        }
      }
    }
    return near_boundary ? -1 : 1;
  };

  std::uint64_t state = 0x123456789abcdefULL;
  auto unif = [&state]() -> mjtNum {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    double u = static_cast<double>(state >> 11) / 9007199254740992.0;
    return static_cast<mjtNum>(2.0 * u - 1.0);
  };
  auto rand_mat = [&unif](mjtNum mat[9]) {
    mjtNum q[4] = {unif(), unif(), unif(), unif()};
    mju_normalize4(q);
    mju_quat2Mat(mat, q);
  };

  const mjtNum eps = MjEps(1e-10, 1e-4);
  int n_overlap = 0, n_sep = 0;
  for (int trial = 0; trial < 200; ++trial) {
    mjtNum pos1[3] = {unif(), unif(), unif()};
    mjtNum pos2[3] = {unif(), unif(), unif()};
    mjtNum mat1[9], mat2[9];
    rand_mat(mat1);
    rand_mat(mat2);

    // exercise all 4 NULL-frame combinations
    const mjtNum* p1 = (trial % 4 == 2 || trial % 4 == 3) ? nullptr : pos1;
    const mjtNum* m1 = (trial % 4 == 2 || trial % 4 == 3) ? nullptr : mat1;
    const mjtNum* p2 = (trial % 4 == 1 || trial % 4 == 3) ? nullptr : pos2;
    const mjtNum* m2 = (trial % 4 == 1 || trial % 4 == 3) ? nullptr : mat2;

    mjtNum product[36], offset[12];
    mjtBool init = true;
    for (int sub = 0; sub < 4; ++sub) {
      mjtNum aabb1[6] = {static_cast<mjtNum>(0.5) * unif(),
                         static_cast<mjtNum>(0.5) * unif(),
                         static_cast<mjtNum>(0.5) * unif(),
                         static_cast<mjtNum>(0.05 + 0.5 * std::abs(unif())),
                         static_cast<mjtNum>(0.05 + 0.5 * std::abs(unif())),
                         static_cast<mjtNum>(0.05 + 0.5 * std::abs(unif()))};
      mjtNum aabb2[6] = {static_cast<mjtNum>(0.5) * unif(),
                         static_cast<mjtNum>(0.5) * unif(),
                         static_cast<mjtNum>(0.5) * unif(),
                         static_cast<mjtNum>(0.05 + 0.5 * std::abs(unif())),
                         static_cast<mjtNum>(0.05 + 0.5 * std::abs(unif())),
                         static_cast<mjtNum>(0.05 + 0.5 * std::abs(unif()))};
      mjtNum margin = static_cast<mjtNum>(0.1) * unif();
      int expected = ref_obb(aabb1, aabb2, p1, m1, p2, m2, margin, eps);
      int got_cached = mj_collideOBB(aabb1, aabb2, p1, m1, p2, m2, margin,
                                     product, offset, &init);
      int got_uncached = mj_collideOBB(aabb1, aabb2, p1, m1, p2, m2, margin,
                                       nullptr, nullptr, nullptr);
      EXPECT_FALSE(init);
      if (expected >= 0) {
        EXPECT_EQ(got_cached, expected) << "trial " << trial << " sub " << sub;
        EXPECT_EQ(got_uncached, expected)
            << "trial " << trial << " sub " << sub;
        if (expected) {
          n_overlap++;
        } else {
          n_sep++;
        }
      }
    }
  }
  EXPECT_GT(n_overlap, 100);
  EXPECT_GT(n_sep, 100);

  // Infinite plane cases: plane vs finite box (above/below), rotated plane,
  // plane vs plane, and all-3-infinite box.
  mjtNum plane[6] = {0,        0,        -static_cast<mjtNum>(0.5) * mjMAXVAL,
                     mjMAXVAL, mjMAXVAL, static_cast<mjtNum>(0.5) * mjMAXVAL};
  mjtNum box[6] = {0,
                   0,
                   0,
                   static_cast<mjtNum>(0.1),
                   static_cast<mjtNum>(0.1),
                   static_cast<mjtNum>(0.1)};
  mjtNum all_inf[6] = {0, 0, 0, mjMAXVAL, mjMAXVAL, mjMAXVAL};
  mjtNum p_origin[3] = {0, 0, 0};
  // In single precision, 0.5 * mjMAXVAL (5e9f) has ULP 512, so z must exceed
  // 512 to be distinguishable from the plane's center offset.
  mjtNum p_above[3] = {0, 0, MjEps(0.5, 2000.0)};
  mjtNum p_touch[3] = {0, 0, static_cast<mjtNum>(0.05)};
  mjtNum eye[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  mjtNum rot[9];
  rand_mat(rot);
  mjtNum product[36], offset[12];
  mjtBool init = true;
  EXPECT_EQ(mj_collideOBB(plane, box, p_origin, eye, p_above, rot, 0, product,
                          offset, &init),
            0);
  EXPECT_EQ(mj_collideOBB(plane, box, p_origin, eye, p_above, rot, 0, nullptr,
                          nullptr, nullptr),
            0);
  init = true;
  EXPECT_EQ(mj_collideOBB(plane, box, p_origin, eye, p_touch, rot, 0, product,
                          offset, &init),
            1);
  EXPECT_EQ(mj_collideOBB(box, plane, p_above, rot, p_origin, eye, 0, nullptr,
                          nullptr, nullptr),
            0);
  EXPECT_EQ(mj_collideOBB(box, plane, p_touch, rot, p_origin, eye, 0, nullptr,
                          nullptr, nullptr),
            1);
  EXPECT_EQ(mj_collideOBB(plane, plane, p_origin, eye, p_above, rot, 0, nullptr,
                          nullptr, nullptr),
            1);
  EXPECT_EQ(mj_collideOBB(all_inf, box, p_origin, eye, p_above, rot, 0, nullptr,
                          nullptr, nullptr),
            1);
}

TEST_F(MjCollisionTest, MidphaseMatchesAllToAllMultiGeom) {
  // Multi-geom bodies mixing fullinertia, diaginertia, rotated geoms, spheres,
  // boxes, capsules, planes, and margins: midphase must match all-to-all.
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="5 5 0.1"/>
      <geom name="wall" type="plane" pos="0.4 0 0" zaxis="-1 0 0"
            size="5 5 0.1"/>
      <body name="a" pos="0 0 0.15" euler="15 25 35">
        <freejoint/>
        <inertial pos="0.01 -0.02 0.01" mass="1"
                  fullinertia="0.02 0.03 0.04 0.005 -0.003 0.004"/>
        <geom name="a_box0" type="box" size="0.08 0.04 0.03" pos="-0.06 0 0"
              margin="0.005"/>
        <geom name="a_box1" type="box" size="0.04 0.07 0.02" pos="0.07 0.02 0"
              euler="30 20 10"/>
        <geom name="a_cap" type="capsule" size="0.03 0.06" pos="0 -0.06 0.02"
              euler="45 0 30"/>
        <geom name="a_sph" type="sphere" size="0.04" pos="0 0 -0.07"/>
      </body>
      <body name="b" pos="0.12 0.03 0.16" euler="-20 40 10">
        <freejoint/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.02 0.02 0.02"/>
        <geom name="b_box0" type="box" size="0.06 0.05 0.04" pos="-0.04 0 0"/>
        <geom name="b_box1" type="box" size="0.03 0.03 0.05" pos="0.06 -0.03 0"/>
        <geom name="b_cyl" type="cylinder" size="0.03 0.05" pos="0 0.05 0.01"
              euler="10 50 20" gap="0.01"/>
        <geom name="b_sph" type="sphere" size="0.04" pos="-0.02 -0.04 -0.05"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;

  MjDataPtr d_mid = MakeData(m);
  MjDataPtr d_all = MakeData(m);

  for (int step = 0; step < 20; ++step) {
    m->opt.disableflags = 0;
    mj_fwdPosition(m.get(), d_mid.get());
    m->opt.disableflags = mjDSBL_MIDPHASE;
    mju_copy(d_all->qpos, d_mid->qpos, m->nq);
    mju_copy(d_all->qvel, d_mid->qvel, m->nv);
    mj_fwdPosition(m.get(), d_all.get());
    ASSERT_EQ(d_mid->ncon, d_all->ncon) << "step " << step;
    EXPECT_EQ(colliding_pairs(m.get(), d_mid.get()),
              colliding_pairs(m.get(), d_all.get()))
        << "step " << step;
    m->opt.disableflags = 0;
    mj_step(m.get(), d_mid.get());
  }
}

TEST_F(MjCollisionTest, PlaneInBody) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom pos="0 0 0" type="plane" size="1 1 .01"/>
      </body>
      <body pos="0 0 .0499">
        <joint type="slide" axis="0 0 1"/>
        <geom size=".05"/>
      </body>
    </worldbody>
    </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());
  mj_step(m.get(), d.get());
}

TEST_F(MjCollisionTest, PinchingSucceeds) {
  constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.002" gravity="0 0 -9.81"/>
    <worldbody>
      <geom name="floor" type="plane" size="0 0 1"/>

      <body name="gripper" pos="0 0 0.5">
        <joint name="lift" type="slide" axis="0 0 1" damping="50"/>
        <geom type="box" size="0.2 0.05 0.02" rgba="0.5 0.5 0.5 1"/> <!-- base -->

        <body name="left_finger" pos="-0.1 0 -0.1">
          <joint name="left_slide" type="slide" axis="1 0 0" damping="10"/>
          <geom type="box" size="0.02 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
        </body>

        <body name="right_finger" pos="0.1 0 -0.1">
          <joint name="right_slide" type="slide" axis="-1 0 0" damping="10"/>
          <geom type="box" size="0.02 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
        </body>
      </body>

      <flexcomp name="cloth" type="grid" dim="2" count="9 9 1" spacing="0.05 0.05 0.05"
                pos="0 0 0.1" radius="0.01">
        <edge equality="true"/>
      </flexcomp>
    </worldbody>

    <equality>
      <joint joint1="right_slide" joint2="left_slide"/>
    </equality>

    <tendon>
      <fixed name="grasp">
        <joint joint="right_slide" coef="1"/>
        <joint joint="left_slide" coef="1"/>
      </fixed>
    </tendon>

    <actuator>
      <position name="lift" joint="lift" kp="600" dampratio="1" ctrlrange="-1 1"/>
      <position name="grasp" tendon="grasp" kp="200" dampratio="1" ctrlrange="0 1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  int lift_id = mj_name2id(m.get(), mjOBJ_ACTUATOR, "lift");
  int grasp_id = mj_name2id(m.get(), mjOBJ_ACTUATOR, "grasp");

  // Phase 1: Lower gripper.
  // The gripper base starts at z=0.5. The finger has length 0.2 (size 0.1),
  // extending from z=0.4 to z=0.2 relative to base (center at -0.1).
  // The cloth is at z=0.1. We need to lower the gripper so the fingertips
  // reach the cloth. A lift value of -0.35 places the fingertips near z=0.05.

  for (int i = 0; i < 500; ++i) {
    d->ctrl[lift_id] = -0.35;  // Lower
    d->ctrl[grasp_id] = 0;     // Open
    mj_step(m.get(), d.get());
  }

  // Phase 2: Pinch
  for (int i = 0; i < 100; ++i) {
    d->ctrl[lift_id] = -0.35;  // Hold height
    d->ctrl[grasp_id] = 0.8;   // Close (max 1)
    mj_step(m.get(), d.get());
  }

  // Phase 3: Lift
  for (int i = 0; i < 1000; ++i) {
    d->ctrl[lift_id] = 0.5;   // Lift up
    d->ctrl[grasp_id] = 0.8;  // Keep closed
    mj_step(m.get(), d.get());
  }

  // Check if cloth is lifted
  // flex verts are in d->flexvert_xpos
  // original z is ~0.1 (falling to floor ~0.0)
  // gripper lifted to > 0.5 probably

  // Find average Z of cloth
  double avg_z = 0;
  int nvert = m->flex_vertnum[0];
  for (int i = 0; i < nvert; ++i) {
    avg_z += d->flexvert_xpos[3 * i + 2];
  }
  avg_z /= nvert;

  // If lifted, avg_z should be significantly > 0.1
  // If failed (slipped), avg_z should be near 0 (floor)

  // Specialized primitives (mjraw_BoxTriangle, mjraw_CapsuleTriangle) should
  // enable stable pinching, so we expect the cloth to be lifted.
  EXPECT_GT(avg_z, 0.2) << "Cloth slipped out of gripper!";
}

TEST_F(MjCollisionTest, MarginSumming) {
  // Two spheres with size 0.1, placed 0.21 apart (distance of 0.01).
  // With margin summing, margin1 + margin2 = 0.00999 + 0.00999 = 0.01998 > 0.01
  // so a contact should be generated.
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom name="sphere1" type="sphere" size=".1" margin="0.00999"/>
        <joint type="slide" axis="1 0 0"/>
      </body>
      <body pos=".21 0 0">
        <geom name="sphere2" type="sphere" size=".1" margin="0.00999"/>
        <joint type="slide" axis="1 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_fwdPosition(m.get(), d.get());

  // With margin summing, we expect 1 contact
  EXPECT_EQ(d->ncon, 1);
}

TEST_F(MjCollisionTest, MaxContact) {
  constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag multiccd="enable"/>
    </option>
    <asset>
      <mesh name="smallbox"
        vertex="-1 -1 -1  1 -1 -1   1  1 -1
                 1  1  1  1 -1  1  -1  1 -1
                -1  1  1 -1 -1  1"/>
    </asset>
    <worldbody>
      <geom name="mesh" type="mesh" mesh="smallbox"/>
      <geom name="box" type="box" size="1 1 1"/>
      <geom name="plane" type="plane" size="1 1 1"/>
      <geom name="sphere" type="sphere" size="1"/>
      <geom name="capsule" type="capsule" size="1 1"/>
      <geom name="ellipsoid" type="ellipsoid" size="1 1 1"/>
      <geom name="cylinder" type="cylinder" size="1 1"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  int mesh = mj_name2id(m.get(), mjOBJ_GEOM, "mesh");
  int box = mj_name2id(m.get(), mjOBJ_GEOM, "box");
  int plane = mj_name2id(m.get(), mjOBJ_GEOM, "plane");
  int sphere = mj_name2id(m.get(), mjOBJ_GEOM, "sphere");
  int capsule = mj_name2id(m.get(), mjOBJ_GEOM, "capsule");
  int ellipsoid = mj_name2id(m.get(), mjOBJ_GEOM, "ellipsoid");
  int cylinder = mj_name2id(m.get(), mjOBJ_GEOM, "cylinder");

  EXPECT_EQ(mj_maxContact(m.get(), mesh, box, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), mesh, plane, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), box, plane, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), mesh, mesh, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), box, box, -1), 8);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, capsule, -1), 2);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, box, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, plane, -1), 2);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, plane, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, sphere, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, capsule, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, box, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, mesh, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, plane, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, cylinder, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, ellipsoid, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, box, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, mesh, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, plane, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, cylinder, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, capsule, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, cylinder, -1), 2);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, mesh, -1), 2);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, cylinder, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, box, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, mesh, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), mesh, box, 1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), mesh, mesh, 1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, box, 1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, mesh, 1), 2);
}

// flex lying on a plane and on 16 spheres of the world body
std::string FlexOnGeoms(const std::string& flex) {
  return R"(
  <mujoco>
    <worldbody>
      <geom name="plane" type="plane" size="0 0 1" pos="0 0 .05"/>
      <replicate count="4" offset=".2 0 0">
        <replicate count="4" offset="0 .2 0">
          <geom size=".05" pos="-.3 -.3 0"/>
        </replicate>
      </replicate>
      )" +
         flex + R"(
    </worldbody>
  </mujoco>
  )";
}

// contacts with the plane, all contacts, maximum contacts with one sphere
struct FlexContacts {
  int nplane = 0;
  int ncon = 0;
  int maxsphere = 0;
};

FlexContacts CountFlexContacts(const mjModel* m, mjData* d) {
  mj_fwdPosition(m, d);
  int plane = mj_name2id(m, mjOBJ_GEOM, "plane");
  std::vector<int> ngeom(m->ngeom, 0);
  FlexContacts count;
  count.ncon = d->ncon;
  for (int i = 0; i < d->ncon; i++) {
    int g = d->contact[i].geom[0] >= 0 ? d->contact[i].geom[0]
                                       : d->contact[i].geom[1];
    ngeom[g]++;
  }
  for (int g = 0; g < m->ngeom; g++) {
    if (g == plane) {
      count.nplane = ngeom[g];
    } else if (m->geom_type[g] == mjGEOM_SPHERE) {
      count.maxsphere = std::max(count.maxsphere, ngeom[g]);
    }
  }
  return count;
}

TEST_F(MjCollisionTest, VertexFlexContactsAreNotLimited) {
  char error[1024];
  MjModelPtr m = LoadModelFromString(FlexOnGeoms(R"(
      <flexcomp name="cloth" type="grid" count="15 15 1" spacing=".05 .05 .05" dim="2"
                pos="0 0 .053" radius=".005">
        <edge equality="true"/>
        <contact selfcollide="none"/>
      </flexcomp>)"),
                                     error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  ASSERT_EQ(m->flex_rigid[0], 0);
  ASSERT_EQ(m->flex_interp[0], 0);

  MjDataPtr d_bvh = MakeData(m);
  FlexContacts bvh = CountFlexContacts(m.get(), d_bvh.get());

  // every vertex touches the plane, all contacts are kept
  EXPECT_EQ(bvh.nplane, m->flex_vertnum[0]);
  EXPECT_GT(bvh.ncon - bvh.nplane, 0);

  // same contacts without midphase
  m->opt.disableflags |= mjDSBL_MIDPHASE;
  MjDataPtr d_all = MakeData(m);
  FlexContacts all = CountFlexContacts(m.get(), d_all.get());
  EXPECT_EQ(all.nplane, bvh.nplane);
  EXPECT_EQ(all.ncon, bvh.ncon);
}

TEST_F(MjCollisionTest, ReducibleFlexContactLimitIsPerGeom) {
  // rigid cloth in a free body, trilinear soft box with its bottom layer on the
  // geoms
  const char* rigid = R"(
      <body name="cloth" pos="0 0 .053">
        <freejoint/>
        <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
        <flexcomp name="cloth" type="grid" count="15 15 1" spacing=".05 .05 .05" dim="2"
                  radius=".005" rigid="true">
          <contact selfcollide="none"/>
        </flexcomp>
      </body>)";
  const char* trilinear = R"(
      <flexcomp name="box" type="grid" count="15 15 2" spacing=".05 .05 .05" dim="3"
                pos="0 0 .078" radius=".005" mass="1" dof="trilinear">
        <contact selfcollide="none"/>
      </flexcomp>)";
  for (const char* attrib : {rigid, trilinear}) {
    char error[1024];
    MjModelPtr m =
        LoadModelFromString(FlexOnGeoms(attrib), error, sizeof(error));
    ASSERT_THAT(m.get(), NotNull()) << attrib << ": " << error;
    ASSERT_TRUE(m->flex_rigid[0] || m->flex_interp[0]) << attrib;

    MjDataPtr d_bvh = MakeData(m);
    FlexContacts bvh = CountFlexContacts(m.get(), d_bvh.get());

    // every vertex touches the plane, mjMAXCONPAIR contacts are kept
    EXPECT_EQ(bvh.nplane, mjMAXCONPAIR) << attrib;

    // the spheres have more contacts, each sphere at most mjMAXCONPAIR
    EXPECT_GT(bvh.ncon - bvh.nplane, mjMAXCONPAIR) << attrib;
    EXPECT_LE(bvh.maxsphere, mjMAXCONPAIR) << attrib;

    // same contacts without midphase
    m->opt.disableflags |= mjDSBL_MIDPHASE;
    MjDataPtr d_all = MakeData(m);
    FlexContacts all = CountFlexContacts(m.get(), d_all.get());
    EXPECT_EQ(all.nplane, bvh.nplane) << attrib;
    EXPECT_EQ(all.ncon, bvh.ncon) << attrib;
  }
}

TEST_F(MjCollisionTest, Flex3DActiveLayersMidphaseDisabled) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="box" type="box" size="0.15 0.15 0.05" pos="0 0 0"/>
      <flexcomp name="vol1" type="grid" dim="3" count="4 4 4"
                spacing="0.06 0.06 0.06" pos="0 0 0.09" radius="0.005" mass="1">
        <contact selfcollide="none" activelayers="1"/>
        <edge equality="true"/>
      </flexcomp>
      <flexcomp name="vol2" type="grid" dim="3" count="4 4 4"
                spacing="0.06 0.06 0.06" pos="0 0 0.22" radius="0.005" mass="1">
        <contact selfcollide="none" activelayers="1"/>
        <edge equality="true"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;

  MjDataPtr d_bvh = MakeData(m);
  mj_forward(m.get(), d_bvh.get());
  EXPECT_GT(d_bvh->ncon, 0);

  m->opt.disableflags |= mjDSBL_MIDPHASE;
  MjDataPtr d_all = MakeData(m);
  mj_forward(m.get(), d_all.get());
  EXPECT_EQ(d_all->ncon, d_bvh->ncon);

  for (int i = 0; i < d_all->ncon; ++i) {
    for (int k = 0; k < 2; ++k) {
      int f = d_all->contact[i].flex[k];
      int e = d_all->contact[i].elem[k];
      if (f >= 0 && e >= 0) {
        int layer = m->flex_elemlayer[m->flex_elemadr[f] + e];
        EXPECT_LT(layer, m->flex_activelayers[f]);
      }
    }
  }
}

// GJK/EPA options with a tight tolerance: EPA normals of curved shapes converge
// as the square root of the distance tolerance
std::string CcdOption() {
  return sizeof(mjtNum) == sizeof(float)
             ? R"(ccd_tolerance="1e-6" ccd_iterations="200")"
             : R"(ccd_tolerance="1e-12" ccd_iterations="200")";
}

// model with the given options, a probe geom on a free body, and the given
// bodies
std::string ProbeModel(const std::string& probe, const std::string& bodies,
                       const std::string& options = "") {
  return R"(
  <mujoco>
    )" + options +
         R"(
    <asset>
      <mesh name="tet" vertex="0 0 0  .1 0 0  0 .1 0  0 0 .1"/>
    </asset>
    <worldbody>
      <body>
        <freejoint/>
        )" +
         probe + R"(
      </body>
      )" +
         bodies + R"(
    </worldbody>
  </mujoco>)";
}

// compares the probe contacts of two models over random probe poses around the
// tetrahedron with vertices (0, 0, 0), (.1, 0, 0), (0, .1, 0), (0, 0, .1);
// returns the number of poses with a contact
int CompareProbeContacts(const std::string& xml1, const std::string& xml2) {
  char error[1024];
  MjModelPtr m1 = LoadModelFromString(xml1.c_str(), error, sizeof(error));
  EXPECT_THAT(m1.get(), NotNull()) << error;
  MjModelPtr m2 = LoadModelFromString(xml2.c_str(), error, sizeof(error));
  EXPECT_THAT(m2.get(), NotNull()) << error;
  if (!m1 || !m2) return 0;
  MjDataPtr d1 = MakeData(m1);
  MjDataPtr d2 = MakeData(m2);

  // pseudo-random numbers in [-1, 1), identical on all platforms (64-bit LCG)
  std::uint64_t state = 1;
  auto unif = [&state]() {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    double u = static_cast<double>(state >> 11) / 9007199254740992.0;  // 2^53
    return static_cast<mjtNum>(2 * u - 1);
  };
  // in single precision, GJK/EPA occasionally misses a contact
  const bool single = sizeof(mjtNum) == sizeof(float);
  int ncontact = 0, nmismatch = 0;
  std::string mismatched;
  for (int i = 0; i < 300; i++) {
    // position within .07 of the tetrahedron centroid, random orientation
    mjtNum qpos[7];
    for (int k = 0; k < 7; k++) {
      qpos[k] = unif();
    }
    for (int k = 0; k < 3; k++) {
      qpos[k] = .025 + .07 * qpos[k];
    }
    mju_normalize4(qpos + 3);
    mju_copy(d1->qpos, qpos, 7);
    mju_copy(d2->qpos, qpos, 7);
    mj_fwdPosition(m1.get(), d1.get());
    mj_fwdPosition(m2.get(), d2.get());

    // skip poses at the contact boundary, where existence is ill-conditioned
    if (d1->ncon + d2->ncon == 0) continue;
    if (d1->ncon != d2->ncon) {
      const mjContact* c = d1->ncon ? d1->contact : d2->contact;
      if (std::abs(c->dist - c->includemargin) < MjTol(1e-6, 1e-4)) continue;
    }

    SCOPED_TRACE("pose " + std::to_string(i));
    if (d1->ncon != 1 || d2->ncon != 1) {
      const mjContact* c = d1->ncon ? d1->contact : d2->contact;
      if (!single) {
        ADD_FAILURE() << "ncon " << d1->ncon << " vs " << d2->ncon
                      << ", dist - includemargin "
                      << c->dist - c->includemargin;
      }
      nmismatch++;
      mismatched += " " + std::to_string(i);
      continue;
    }
    const mjContact& con1 = d1->contact[0];
    const mjContact& con2 = d2->contact[0];
    // in single precision, GJK/EPA normals of curved shapes are limited by the
    // achievable tolerance: they converge as the square root of the distance
    // tolerance, and the frame tolerance (about 17 degrees) reflects the float
    // GJK/EPA reference, not the analytic contacts
    EXPECT_NEAR(con1.dist, con2.dist, MjTol(2e-8, 3e-5));
    for (int k = 0; k < 3; k++) {
      EXPECT_NEAR(con1.frame[k], con2.frame[k], MjTol(3e-4, 3e-1));
      EXPECT_NEAR(con1.pos[k], con2.pos[k], MjTol(2e-6, 1e-3));
    }
    ncontact++;
  }
  EXPECT_LE(nmismatch, single ? 3 : 0) << "mismatched poses:" << mismatched;
  return ncontact;
}

TEST_F(MjCollisionTest, Flex3DSphereCapsuleMatchMesh) {
  // flex tetrahedron with zero radius, and the same tetrahedron as a mesh
  const std::string flex = R"(
      <flexcomp name="tet" type="direct" dim="3" radius="0" mass="1"
                point="0 0 0  .1 0 0  0 .1 0  0 0 .1" element="0 1 2 3">
        <contact selfcollide="none"/>
      </flexcomp>)";
  const std::string mesh = R"(
      <body>
        <freejoint/>
        <geom type="mesh" mesh="tet"/>
      </body>)";
  const std::string nomulti = "<option " + CcdOption() + R"(>
      <flag multiccd="disable"/>
    </option>)";

  // analytic sphere and capsule : tetrahedron match GJK/EPA with the mesh
  for (const char* probe :
       {R"(<geom size=".02"/>)", R"(<geom type="capsule" size=".015 .03"/>)"}) {
    SCOPED_TRACE(probe);
    EXPECT_GT(CompareProbeContacts(ProbeModel(probe, flex),
                                   ProbeModel(probe, mesh, nomulti)),
              50);
  }
}

TEST_F(MjCollisionTest, Flex3DSphereRadiusMarginMatchEllipsoid) {
  // with flex radius and margin, analytic sphere matches GJK/EPA with an
  // ellipsoid of equal radii, which uses the general geom : elem path
  const std::string flex = R"(
      <flexcomp name="tet" type="direct" dim="3" radius=".005" mass="1"
                point="0 0 0  .1 0 0  0 .1 0  0 0 .1" element="0 1 2 3">
        <contact selfcollide="none" margin=".01"/>
      </flexcomp>)";
  const std::string ccd = "<option " + CcdOption() + "/>";
  EXPECT_GT(CompareProbeContacts(
                ProbeModel(R"(<geom size=".02"/>)", flex),
                ProbeModel(R"(<geom type="ellipsoid" size=".02 .02 .02"/>)",
                           flex, ccd)),
            50);
}

TEST_F(MjCollisionTest, Flex3DSphereFlatElementMatchesConvex) {
  // nearly flat tetrahedron, which is too thin for the analytic path: sphere
  // matches GJK/EPA with an ellipsoid of equal radii (the analytic path would
  // also match in double precision; the Warp test_degenerate_tetrahedron checks
  // that it rejects the element)
  const std::string flex = R"(
      <flexcomp name="tet" type="direct" dim="3" radius=".005" mass="1"
                point="0 0 0  .1 0 0  0 .1 0  .03 .03 1e-8" element="0 1 2 3">
        <contact selfcollide="none"/>
      </flexcomp>)";
  const std::string ccd = "<option " + CcdOption() + "/>";
  EXPECT_GT(CompareProbeContacts(
                ProbeModel(R"(<geom size=".02"/>)", flex, ccd),
                ProbeModel(R"(<geom type="ellipsoid" size=".02 .02 .02"/>)",
                           flex, ccd)),
            20);
}

// expects a single contact between the probe at the free joint pose qpos and
// the unit tetrahedron with vertices pos, pos + (1, 0, 0), pos + (0, 1, 0),
// pos + (0, 0, 1), with the given distance and normal (from the probe to the
// tetrahedron)
void ExpectUnitTetrahedronContact(const char* probe, const char* pos,
                                  const mjtNum qpos[7], mjtNum dist,
                                  const mjtNum normal[3], mjtNum tol) {
  const std::string flex = std::string(R"(
      <flexcomp name="tet" type="direct" dim="3" radius="0" mass="1" pos=")") +
                           pos + R"("
                point="0 0 0  1 0 0  0 1 0  0 0 1" element="0 1 2 3">
        <contact selfcollide="none"/>
      </flexcomp>)";
  char error[1024];
  MjModelPtr m = LoadModelFromString(ProbeModel(probe, flex).c_str(), error,
                                     sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mju_copy(d->qpos, qpos, 7);
  mj_fwdPosition(m.get(), d.get());
  ASSERT_EQ(d->ncon, 1);
  EXPECT_NEAR(d->contact[0].dist, dist, tol);
  for (int k = 0; k < 3; k++) {
    EXPECT_NEAR(d->contact[0].frame[k], normal[k], tol);
  }
}

TEST_F(MjCollisionTest, Flex3DSphereCenterOnFace) {
  // sphere centers on the face x + y + z = 1, on either side of it within
  // rounding, also far from the origin: the normal is the face normal and the
  // depth is the radius
  const mjtNum s3 = 1 / mju_sqrt(3.0);
  const mjtNum normal[3] = {-s3, -s3, -s3};
  const mjtNum w[5][3] = {{.2, .3, .5},
                          {.6, .2, .2},
                          {.25, .5, .25},
                          {s3 * s3, s3 * s3, s3 * s3},
                          {.45, .1, .45}};
  struct {
    const char* str;
    mjtNum val[3];
  } offsets[2] = {{"0 0 0", {0, 0, 0}}, {"100 -50 30", {100, -50, 30}}};
  for (const auto& offset : offsets) {
    for (const auto& wi : w) {
      SCOPED_TRACE(offset.str);
      mjtNum qpos[7] = {0, 0, 0, 1, 0, 0, 0};
      for (int k = 0; k < 3; k++) {
        qpos[k] = offset.val[k] + wi[k];
      }
      ExpectUnitTetrahedronContact(R"(<geom size=".1"/>)", offset.str, qpos,
                                   -.1, normal, MjTol(1e-10, 1e-4));
    }
  }
}

TEST_F(MjCollisionTest, Flex3DSphereCapsuleNearTouchingNormals) {
  // probes whose center or axis passes just outside the unit tetrahedron: the
  // normals are exact, for a closest point at a vertex, inside an edge and
  // inside a face
  const mjtNum s2 = 1 / mju_sqrt(2.0), s3 = 1 / mju_sqrt(3.0);
  const mjtNum tol = MjTol(1e-10, 1e-5);

  // sphere near vertex (0, 0, 0)
  {
    const mjtNum qpos[7] = {-5e-6, -5e-6, -5e-6, 1, 0, 0, 0};
    const mjtNum normal[3] = {s3, s3, s3};
    ExpectUnitTetrahedronContact(R"(<geom size=".01"/>)", "0 0 0", qpos,
                                 5e-6 * mju_sqrt(3.0) - .01, normal, tol);
  }

  // capsule along (0, 1, -1) crossing just outside edge x, rotated by -135
  // degrees about x
  {
    const mjtNum qpos[7] = {
        .5, -5e-6, -5e-6, mju_cos(-3 * mjPI / 8), mju_sin(-3 * mjPI / 8), 0, 0};
    const mjtNum normal[3] = {0, s2, s2};
    ExpectUnitTetrahedronContact(R"(<geom type="capsule" size=".05 .14"/>)",
                                 "0 0 0", qpos, 5e-6 * mju_sqrt(2.0) - .05,
                                 normal, tol);
  }

  // capsule along x just below face z = 0, overhanging the tetrahedron
  {
    const mjtNum qpos[7] = {0, .5, -1e-7, s2, 0, s2, 0};
    const mjtNum normal[3] = {0, 0, 1};
    ExpectUnitTetrahedronContact(R"(<geom type="capsule" size=".01 5"/>)",
                                 "0 0 0", qpos, 1e-7 - .01, normal, tol);
  }
}

// closed box surface with n[i] points along edge i and half-sizes size[i], as
// point and element strings of a flexcomp of type direct
void BoxSurface(const int n[3], const mjtNum size[3], std::string* point,
                std::string* element) {
  // index surface points
  std::vector<int> id(n[0] * n[1] * n[2], -1);
  int npoint = 0;
  for (int i = 0; i < n[0]; i++) {
    for (int j = 0; j < n[1]; j++) {
      for (int k = 0; k < n[2]; k++) {
        if (i == 0 || i == n[0] - 1 || j == 0 || j == n[1] - 1 || k == 0 ||
            k == n[2] - 1) {
          int ijk[3] = {i, j, k};
          id[(i * n[1] + j) * n[2] + k] = npoint++;
          for (int a = 0; a < 3; a++) {
            *point += std::to_string(size[a] * (2.0 * ijk[a] / (n[a] - 1) - 1));
            *point += ' ';
          }
        }
      }
    }
  }

  // two triangles per quad on each face
  for (int a = 0; a < 3; a++) {
    int b = (a + 1) % 3, c = (a + 2) % 3;
    for (int s : {0, n[a] - 1}) {
      for (int u = 0; u < n[b] - 1; u++) {
        for (int v = 0; v < n[c] - 1; v++) {
          int quad[4];
          int corner[4][2] = {{u, v}, {u + 1, v}, {u + 1, v + 1}, {u, v + 1}};
          for (int q = 0; q < 4; q++) {
            int ijk[3];
            ijk[a] = s;
            ijk[b] = corner[q][0];
            ijk[c] = corner[q][1];
            quad[q] = id[(ijk[0] * n[1] + ijk[1]) * n[2] + ijk[2]];
          }
          int tri[2][3] = {{quad[0], quad[1], quad[2]},
                           {quad[0], quad[2], quad[3]}};
          for (auto& t : tri) {
            if (s == 0) {
              std::swap(t[1], t[2]);
            }
            for (int vid : t) {
              *element += std::to_string(vid) + " ";
            }
          }
        }
      }
    }
  }
}

// pillow: closed box surface of 10 x 10 x 2 cm with interpolated dofs, resting
// on a floor, optionally pressed by a plate
std::string Pillow(const std::string& cellcount, const std::string& selfcollide,
                   const std::string& dof = "trilinear", bool plate = false) {
  const int n[3] = {11, 11, 3};
  const mjtNum size[3] = {.05, .05, .01};
  std::string point, element;
  BoxSurface(n, size, &point, &element);

  std::string plate_xml = plate ? R"(
      <body name="plate" pos="0 0 .028">
        <joint name="press" type="slide" axis="0 0 1" damping="5"/>
        <geom type="box" size=".07 .07 .005" mass=".1"/>
      </body>)"
                                : "";
  std::string actuator_xml = plate ? R"(
    <actuator>
      <motor joint="press" gear="-1"/>
    </actuator>)"
                                   : "";
  // narrow phase only: all element pairs are allocated on the arena
  std::string memory = selfcollide == "narrow" ? "200M" : "10M";
  return R"(
  <mujoco>
    <option timestep="1e-3" solver="CG" tolerance="1e-8" integrator="discrete"/>
    <size memory=")" +
         memory + R"("/>
    <worldbody>
      <geom name="floor" type="plane" size="1 1 .1"/>)" +
         plate_xml + R"(
      <flexcomp name="pillow" type="direct" dim="2" pos="0 0 .0115"
                radius=".001" mass=".1" dof=")" +
         dof + R"(" cellcount=")" + cellcount + R"("
                point=")" +
         point + R"(" element=")" + element + R"(">
        <elasticity young="300" poisson=".2" damping=".002" elastic2d="none"/>
        <contact selfcollide=")" +
         selfcollide + R"(" solref=".005 1"/>
      </flexcomp>
    </worldbody>)" +
         actuator_xml + R"(
  </mujoco>
  )";
}

// self contacts of a flex: total, between its top and bottom faces, smallest
// distance
struct SelfContacts {
  int nself = 0;
  int ntopbottom = 0;
  mjtNum mindist = mjMAXVAL;
};

// vertex v (global index) of an interpolated flex on the top (side=1) or
// bottom (side=0) face of the cage, from its parametric coordinates
bool VertOnFace(const mjModel* m, int v, int side) {
  return std::abs(m->flex_vert0[3 * v + 2] - side) < 1e-6;
}

// element of interpolated flex f on the top (side=1) or bottom (side=0) face of
// the cage
bool OnFace(const mjModel* m, int f, int e, int side) {
  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e * (dim + 1);
  for (int i = 0; i <= dim; i++) {
    if (!VertOnFace(m, m->flex_vertadr[f] + edata[i], side)) {
      return false;
    }
  }
  return true;
}

SelfContacts CountSelfContacts(const mjModel* m, const mjData* d, int f) {
  SelfContacts count;
  for (int i = 0; i < d->ncon; i++) {
    const mjContact& con = d->contact[i];
    if (con.flex[0] != f || con.flex[1] != f) {
      continue;
    }
    count.nself++;
    count.mindist = std::min(count.mindist, con.dist);
    if ((OnFace(m, f, con.elem[0], 0) && OnFace(m, f, con.elem[1], 1)) ||
        (OnFace(m, f, con.elem[0], 1) && OnFace(m, f, con.elem[1], 0))) {
      count.ntopbottom++;
    }
  }
  return count;
}

// move the nodes of interpolated flex f such that the cage is flattened along z
// to thickness eps above its lowest node
void FlattenCage(const mjModel* m, mjData* d, int f, mjtNum eps) {
  mj_resetData(m, d);
  mj_kinematics(m, d);
  int order = std::abs(m->flex_interp[f]);
  int nz = m->flex_cellnum[3 * f + 2] * order + 1;
  int nodeadr = m->flex_nodeadr[f];
  mjtNum zmin = mjMAXVAL;
  for (int i = 0; i < m->flex_nodenum[f]; i++) {
    zmin = std::min(zmin, d->xpos[3 * m->flex_nodebodyid[nodeadr + i] + 2]);
  }
  for (int i = 0; i < m->flex_nodenum[f]; i++) {
    int b = m->flex_nodebodyid[nodeadr + i];
    mjtNum ztarget = zmin + eps * (i % nz) / (nz - 1);
    for (int j = m->body_jntadr[b]; j < m->body_jntadr[b] + m->body_jntnum[b];
         j++) {
      if (m->jnt_type[j] == mjJNT_SLIDE && m->jnt_axis[3 * j + 2] == 1) {
        d->qpos[m->jnt_qposadr[j]] += ztarget - d->xpos[3 * b + 2];
      }
    }
  }
  mj_fwdPosition(m, d);
}

TEST_F(MjCollisionTest, InterpolatedSelfCollisionNoContactsAtRest) {
  for (const char* cellcount : {"1 1 1", "2 2 3", "4 4 4"}) {
    for (const char* selfcollide : {"narrow", "bvh", "sap", "auto"}) {
      for (const char* dof : {"trilinear", "quadratic"}) {
        char error[1024];
        MjModelPtr m = LoadModelFromString(Pillow(cellcount, selfcollide, dof),
                                           error, sizeof(error));
        ASSERT_THAT(m.get(), NotNull()) << error;
        ASSERT_NE(m->flex_interp[0], 0);
        MjDataPtr d = MakeData(m);
        mj_fwdPosition(m.get(), d.get());

        // adjacent elements touch, but they share cage nodes: no self contacts
        EXPECT_EQ(CountSelfContacts(m.get(), d.get(), 0).nself, 0)
            << cellcount << ", " << selfcollide << ", " << dof;
      }
    }
  }
}

TEST_F(MjCollisionTest, InterpolatedSelfCollisionExcludesSharedNodes) {
  for (const char* selfcollide : {"narrow", "bvh", "sap", "auto"}) {
    // top and bottom faces are in cells 2 and 0, which have no common node
    char error[1024];
    MjModelPtr m =
        LoadModelFromString(Pillow("2 2 3", selfcollide), error, sizeof(error));
    ASSERT_THAT(m.get(), NotNull()) << error;
    MjDataPtr d = MakeData(m);
    FlattenCage(m.get(), d.get(), 0, 0.5 * m->flex_radius[0]);
    SelfContacts flat = CountSelfContacts(m.get(), d.get(), 0);
    EXPECT_GT(flat.ntopbottom, 0) << selfcollide;
    EXPECT_LE(flat.nself, mjMAXCONPAIR) << selfcollide;

    // top and bottom faces are in neighboring cells, which share the middle
    // layer of nodes: no self contacts
    for (const char* cellcount : {"2 2 1", "2 2 2"}) {
      MjModelPtr m_near = LoadModelFromString(Pillow(cellcount, selfcollide),
                                              error, sizeof(error));
      ASSERT_THAT(m_near.get(), NotNull()) << error;
      MjDataPtr d_near = MakeData(m_near);
      FlattenCage(m_near.get(), d_near.get(), 0, 0.5 * m->flex_radius[0]);
      EXPECT_EQ(CountSelfContacts(m_near.get(), d_near.get(), 0).nself, 0)
          << cellcount << ", " << selfcollide;
    }
  }
}

TEST_F(MjCollisionTest, InterpolatedSelfCollisionMidphaseParity) {
  // compare selfcollide="auto" (cell-level midphase) against "narrow", "bvh",
  // and "sap" on trilinear and quadratic pillows (including a 3x3x3 cell grid
  // where the 10x10x2 vertex grid has elements straddling cell boundaries)
  for (const char* cellcount : {"2 2 3", "3 3 3"}) {
    for (const char* dof : {"trilinear", "quadratic"}) {
      char error[1024];
      MjModelPtr m_auto = LoadModelFromString(Pillow(cellcount, "auto", dof),
                                              error, sizeof(error));
      ASSERT_THAT(m_auto.get(), NotNull()) << error;
      MjDataPtr d_auto = MakeData(m_auto);

      // push the top-corner cage node down to the bottom face so a small patch
      // of opposite cells collides (< mjMAXCONPAIR contacts, avoiding tie-break
      // truncation differences when bvh/sap swap (e1, e2) in GJK)
      mj_resetData(m_auto.get(), d_auto.get());
      mj_kinematics(m_auto.get(), d_auto.get());
      int nodeadr = m_auto->flex_nodeadr[0];
      mjtNum zmin = mjMAXVAL;
      for (int i = 0; i < m_auto->flex_nodenum[0]; i++) {
        zmin = std::min(
            zmin, d_auto->xpos[3 * m_auto->flex_nodebodyid[nodeadr + i] + 2]);
      }
      int b = m_auto->flex_nodebodyid[nodeadr + m_auto->flex_nodenum[0] - 1];
      mjtNum ztarget = zmin - 0.5 * m_auto->flex_radius[0];
      for (int j = m_auto->body_jntadr[b];
           j < m_auto->body_jntadr[b] + m_auto->body_jntnum[b]; j++) {
        if (m_auto->jnt_type[j] == mjJNT_SLIDE &&
            m_auto->jnt_axis[3 * j + 2] == 1) {
          d_auto->qpos[m_auto->jnt_qposadr[j]] +=
              ztarget - d_auto->xpos[3 * b + 2];
        }
      }
      mj_forward(m_auto.get(), d_auto.get());
      SelfContacts c_auto = CountSelfContacts(m_auto.get(), d_auto.get(), 0);
      ASSERT_GT(c_auto.ntopbottom, 0);
      ASSERT_LT(c_auto.nself, mjMAXCONPAIR);

      for (const char* mode : {"narrow", "bvh", "sap"}) {
        MjModelPtr m = LoadModelFromString(Pillow(cellcount, mode, dof), error,
                                           sizeof(error));
        ASSERT_THAT(m.get(), NotNull()) << error;
        MjDataPtr d = MakeData(m);
        mju_copy(d->qpos, d_auto->qpos, m->nq);
        mj_forward(m.get(), d.get());

        ASSERT_EQ(d->ncon, d_auto->ncon)
            << cellcount << ", " << dof << ", " << mode;

        if (std::string(mode) == "narrow") {
          // auto and narrow both canonicalize (e1 < e2) and sort pairs, so
          // their contacts are bit-for-bit identical
          for (int i = 0; i < d->ncon; i++) {
            EXPECT_EQ(d->contact[i].elem[0], d_auto->contact[i].elem[0])
                << cellcount << ", " << dof << ", contact " << i;
            EXPECT_EQ(d->contact[i].elem[1], d_auto->contact[i].elem[1])
                << cellcount << ", " << dof << ", contact " << i;
            EXPECT_EQ(d->contact[i].dist, d_auto->contact[i].dist)
                << cellcount << ", " << dof << ", contact " << i;
            for (int k = 0; k < 3; k++) {
              EXPECT_EQ(d->contact[i].pos[k], d_auto->contact[i].pos[k]);
              EXPECT_EQ(d->contact[i].frame[k], d_auto->contact[i].frame[k]);
            }
          }
          for (int i = 0; i < m->nv; i++) {
            EXPECT_EQ(d->qfrc_constraint[i], d_auto->qfrc_constraint[i])
                << cellcount << ", " << dof << ", dof " << i;
          }
        } else {
          // bvh and sap do not canonicalize e1 < e2 before GJK/EPA, so GJK
          // distances can differ by ~1e-7 when swapping (e1, e2)
          SelfContacts c_mode = CountSelfContacts(m.get(), d.get(), 0);
          EXPECT_EQ(c_mode.nself, c_auto.nself);
          EXPECT_EQ(c_mode.ntopbottom, c_auto.ntopbottom);
          EXPECT_NEAR(c_mode.mindist, c_auto.mindist, 1e-6);
          for (int i = 0; i < m->nv; i++) {
            EXPECT_THAT(d->qfrc_constraint[i],
                        MjNear(d_auto->qfrc_constraint[i], 1e-4, 1e-3))
                << cellcount << ", " << dof << ", " << mode << ", dof " << i;
          }
        }
      }
    }
  }

  // also test an element spanning >= 2 cells colliding with another element in
  // the same primary cell (both have e_lo = 0, but one spans cells [0, 3] and
  // has only 1 vertex in cell 0 and 2 vertices in cell 3, while the other is
  // in cell 0; after flattening the cage, only the vertices in cell 3 touch
  // cell 0)
  std::string xml_span = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="span" type="direct" dim="2" dof="trilinear"
                cellcount="1 1 4" radius=".01" mass="1"
                element="0 1 2  3 4 5  6 7 8"
                point="0 0 0  1 0 0  0 1 0
                       1 1 0  1 0 1  0 1 1
                       5 0 0  5 1 0  5 0 1">
        <elasticity young="1"/>
        <contact selfcollide="auto"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m_span_auto =
      LoadModelFromString(xml_span.c_str(), error, sizeof(error));
  ASSERT_THAT(m_span_auto.get(), NotNull()) << error;
  MjDataPtr d_span_auto = MakeData(m_span_auto);
  FlattenCage(m_span_auto.get(), d_span_auto.get(), 0,
              0.5 * m_span_auto->flex_radius[0]);
  mj_forward(m_span_auto.get(), d_span_auto.get());

  for (const char* mode : {"narrow", "bvh", "sap"}) {
    std::string xml_mode = xml_span;
    size_t pos = xml_mode.find("auto");
    xml_mode.replace(pos, 4, mode);
    MjModelPtr m_span =
        LoadModelFromString(xml_mode.c_str(), error, sizeof(error));
    ASSERT_THAT(m_span.get(), NotNull()) << error;
    MjDataPtr d_span = MakeData(m_span);
    FlattenCage(m_span.get(), d_span.get(), 0, 0.5 * m_span->flex_radius[0]);
    mj_forward(m_span.get(), d_span.get());
    ASSERT_EQ(d_span->ncon, d_span_auto->ncon) << mode;
    for (int i = 0; i < d_span->ncon; i++) {
      EXPECT_EQ(
          std::min(d_span->contact[i].elem[0], d_span->contact[i].elem[1]),
          d_span_auto->contact[i].elem[0]);
      EXPECT_EQ(
          std::max(d_span->contact[i].elem[0], d_span->contact[i].elem[1]),
          d_span_auto->contact[i].elem[1]);
      EXPECT_NEAR(d_span->contact[i].dist, d_span_auto->contact[i].dist, 1e-6);
    }
  }
}

// number of contacts between elements 0 and 1 of the flex with two horizontal
// triangles at heights z1 and z2 (and a vertical triangle far away, so that the
// cage spans z in [0, 1] with 4 cells along z), after flattening the cage
int FlattenedTrianglePairContacts(mjtNum z1, mjtNum z2) {
  char point[512];
  std::snprintf(point, sizeof(point),
                "0 0 %.17g  1 0 %.17g  0 1 %.17g  0 0 %.17g  1 0 %.17g  "
                "0 1 %.17g  5 0 0  5 1 0  5 0 1",
                z1, z1, z1, z2, z2, z2);
  std::string xml = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="tris" type="direct" dim="2" dof="trilinear"
                cellcount="1 1 4" radius=".001" mass="1"
                element="0 1 2  3 4 5  6 7 8" point=")" +
                    std::string(point) + R"(">
        <contact selfcollide="narrow"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  EXPECT_THAT(m.get(), NotNull()) << error;
  if (!m) return -1;
  MjDataPtr d = MakeData(m);
  FlattenCage(m.get(), d.get(), 0, 0.5 * m->flex_radius[0]);
  int ncon = 0;
  for (int i = 0; i < d->ncon; i++) {
    const mjContact& con = d->contact[i];
    if (con.flex[0] == 0 && con.flex[1] == 0 &&
        std::min(con.elem[0], con.elem[1]) == 0 &&
        std::max(con.elem[0], con.elem[1]) == 1) {
      ncon++;
    }
  }
  return ncon;
}

TEST_F(MjCollisionTest, InterpolatedSelfCollisionBoundarySymmetry) {
  // with 4 cells along z, heights 0, .25, .5, .75 and 1 are cell planes 0 to 4;
  // a triangle on an internal plane belongs to the cells on both sides

  // symmetric about plane 2: a triangle on plane 2 is in cells 1 and 2, which
  // neighbor cells 0 and 3
  EXPECT_EQ(FlattenedTrianglePairContacts(0, .5), 0);
  EXPECT_EQ(FlattenedTrianglePairContacts(.5, 1), 0);
  EXPECT_GT(FlattenedTrianglePairContacts(0, .75), 0);
  EXPECT_GT(FlattenedTrianglePairContacts(.25, 1), 0);
  EXPECT_GT(FlattenedTrianglePairContacts(0, 1), 0);

  // rounding below a cell plane does not change the cells of a triangle
  EXPECT_EQ(FlattenedTrianglePairContacts(0, .5 - 1e-16), 0);
  EXPECT_EQ(FlattenedTrianglePairContacts(.5 - 1e-16, 1), 0);

  // a triangle inside cell 1 neighbors cells 0 and 2, not 3
  EXPECT_EQ(FlattenedTrianglePairContacts(.375, 0), 0);
  EXPECT_GT(FlattenedTrianglePairContacts(.375, 1), 0);
}

// lowest vertex of the top face and highest vertex of the bottom face of the
// cage
std::pair<mjtNum, mjtNum> FaceHeights(const mjModel* m, const mjData* d,
                                      int f) {
  mjtNum ztop = mjMAXVAL, zbottom = -mjMAXVAL;
  for (int v = m->flex_vertadr[f]; v < m->flex_vertadr[f] + m->flex_vertnum[f];
       v++) {
    mjtNum z = d->flexvert_xpos[3 * v + 2];
    if (VertOnFace(m, v, 1)) {
      ztop = std::min(ztop, z);
    } else if (VertOnFace(m, v, 0)) {
      zbottom = std::max(zbottom, z);
    }
  }
  return {ztop, zbottom};
}

TEST_F(MjCollisionTest, InterpolatedSelfCollisionPressedPillow) {
  for (const char* selfcollide : {"auto", "none"}) {
    char error[1024];
    MjModelPtr m = LoadModelFromString(
        Pillow("2 2 3", selfcollide, "trilinear", /*plate=*/true), error,
        sizeof(error));
    ASSERT_THAT(m.get(), NotNull()) << error;
    MjDataPtr d = MakeData(m);

    // press the plate down with a force that collapses the cage, then hold it
    // until the pillow settles; track the smallest gap between the faces
    bool is_auto = std::string(selfcollide) == "auto";
    int nstep = is_auto ? 1000 : 500;
    mjtNum mingap = mjMAXVAL;
    for (int i = 0; i < nstep; i++) {
      d->ctrl[0] = 3 * std::min(1.0, i / 400.0);
      mj_step(m.get(), d.get());
      if (!is_auto) {
        auto [top, bottom] = FaceHeights(m.get(), d.get(), 0);
        mingap = std::min(mingap, top - bottom);
      }
    }
    SelfContacts pressed = CountSelfContacts(m.get(), d.get(), 0);
    auto [ztop, zbottom] = FaceHeights(m.get(), d.get(), 0);
    mjtNum vmax = 0;
    for (int i = 0; i < m->nv; i++) {
      vmax = std::max(vmax, std::abs(d->qvel[i]));
    }

    // simulation is stable
    for (int i = 0; i < m->nq; i++) {
      EXPECT_TRUE(std::isfinite(d->qpos[i])) << selfcollide;
    }
    for (int i = 0; i < mjNWARNING; i++) {
      EXPECT_EQ(d->warning[i].number, 0) << selfcollide << ", warning " << i;
    }

    if (std::string(selfcollide) == "auto") {
      // settled: top and bottom faces are in contact, penetration is bounded
      // (only the settled state is checked: the deepest transient penetration
      // depends on the platform)
      EXPECT_LT(vmax, 0.06);
      EXPECT_GT(pressed.ntopbottom, 0);
      EXPECT_GT(pressed.mindist, -m->flex_radius[0]);
      EXPECT_GT(ztop, zbottom);
    } else {
      // without self-collision, the top face passes through the bottom face
      EXPECT_EQ(pressed.ntopbottom, 0);
      EXPECT_LT(mingap, 0);
    }
  }
}

TEST_F(MjCollisionTest, ParallelCapsuleCapsule) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body euler="30 45 60">
        <geom name="a" type="capsule" size="0.01 2.0" pos="0 0.005 0"/>
      </body>
      <body euler="30 0 0">
        <body euler="0 45 60">
          <geom name="b" type="capsule" size="0.01 1.5" pos="0 -0.005 0.3"/>
          <geom name="c" type="capsule" size="0.01 1.5" pos="0 -0.095 0.3"/>
          <geom name="d" type="capsule" size="0.01 1.5" pos="0 -0.095 0.3" euler="0 0.1 0"/>
          <geom name="e" type="capsule" size="0.01 1.5" pos="0 -0.015 0.3" euler="0 0.01 0"/>
        </body>
      </body>
    </worldbody>
    <contact>
      <pair geom1="a" geom2="b"/>
    </contact>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // Penetrating rotated parallel capsules (a, b) should generate 2 contacts.
  ASSERT_EQ(d->ncon, 2);
  EXPECT_THAT(d->contact[0].dist, MjNear(-0.01, 1e-12, 1e-5));
  EXPECT_THAT(d->contact[1].dist, MjNear(-0.01, 1e-12, 1e-5));

  // Separated rotated parallel capsules (a, c) have surface distance 0.08.
  int a = mj_name2id(m.get(), mjOBJ_GEOM, "a");
  int c = mj_name2id(m.get(), mjOBJ_GEOM, "c");
  EXPECT_THAT(mj_geomDistance(m.get(), d.get(), a, c, 0.2, nullptr),
              MjNear(0.08, 1e-12, 1e-5));

  // Near-parallel (0.1 deg) capsules (a, d): surface distance 0.08, closest
  // point on d is at its center cross-section (distance 0.01 from d's xpos).
  int d_id = mj_name2id(m.get(), mjOBJ_GEOM, "d");
  mjtNum fromto[6];
  EXPECT_THAT(mj_geomDistance(m.get(), d.get(), a, d_id, 0.2, fromto),
              MjNear(0.08, 1e-12, 1e-5));
  EXPECT_THAT(mju_dist3(fromto + 3, d->geom_xpos + 3 * d_id),
              MjNear(0.01, 1e-12, 1e-2));

  // Near-parallel (0.01 deg) capsules (a, e): surface distance 0.0, closest
  // point on e is at its center cross-section (distance 0.01 from e's xpos).
  int e_id = mj_name2id(m.get(), mjOBJ_GEOM, "e");
  EXPECT_THAT(mj_geomDistance(m.get(), d.get(), a, e_id, 0.2, fromto),
              MjNear(0.0, 1e-12, 1e-5));
  EXPECT_THAT(mju_dist3(fromto + 3, d->geom_xpos + 3 * e_id),
              MjNear(0.01, 1e-12, 2e-1));
}

// Parallel capsules where the first one overhangs the second: the closest
// points come from the second capsule's endpoints, so the distance must not
// depend on distmax (the collision margin) or on the geom order.
TEST_F(MjCollisionTest, ParallelCapsuleCapsuleOverhang) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="long" type="capsule" size="0.05 0.3"/>
      <body>
        <geom name="short" type="capsule" size="0.08 0.2" pos="0.2 0 0.05"/>
      </body>
    </worldbody>
    <contact>
      <pair geom1="long" geom2="short" margin="1"/>
    </contact>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());
  mj_forward(m.get(), d.get());

  // Surface distance: 0.2 - (0.05 + 0.08) = 0.07 at both ends of short's span
  // [-0.15, 0.25].
  ASSERT_EQ(d->ncon, 2);
  EXPECT_MJTNUM_EQ(d->contact[0].dist, 0.07);
  EXPECT_MJTNUM_EQ(d->contact[1].dist, 0.07);
  EXPECT_MJTNUM_EQ(d->contact[0].pos[2], -0.15);
  EXPECT_MJTNUM_EQ(d->contact[1].pos[2], 0.25);

  int l = mj_name2id(m.get(), mjOBJ_GEOM, "long");
  int s = mj_name2id(m.get(), mjOBJ_GEOM, "short");
  for (mjtNum distmax : {0.08, 1.0}) {
    mjtNum fromto[6];
    EXPECT_MJTNUM_EQ(mj_geomDistance(m.get(), d.get(), l, s, distmax, fromto),
                     0.07);
    EXPECT_MJTNUM_EQ(fromto[2], -0.15);
    EXPECT_MJTNUM_EQ(fromto[5], -0.15);

    EXPECT_MJTNUM_EQ(mj_geomDistance(m.get(), d.get(), s, l, distmax, fromto),
                     0.07);
    EXPECT_MJTNUM_EQ(fromto[2], -0.15);
    EXPECT_MJTNUM_EQ(fromto[5], -0.15);
  }
}

TEST_F(MjCollisionTest, MaxContactSdfPairs) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <asset>
      <mesh name="cube" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  -0.5 0.5 -0.5  0.5 0.5 -0.5
                                -0.5 -0.5  0.5  0.5 -0.5  0.5  -0.5 0.5  0.5  0.5 0.5  0.5"/>
      <hfield name="hf" nrow="2" ncol="2" size="1 1 1 1" elevation="0 0 0 0"/>
    </asset>
    <worldbody>
      <geom name="sdf" type="sdf" mesh="cube"/>
      <geom name="plane" type="plane" size="1 1 0.1"/>
      <geom name="mesh" type="mesh" mesh="cube"/>
      <geom name="sphere" type="sphere" size="0.5"/>
      <geom name="hfield" type="hfield" hfield="hf"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;

  int sdf = mj_name2id(m.get(), mjOBJ_GEOM, "sdf");
  int plane = mj_name2id(m.get(), mjOBJ_GEOM, "plane");
  int mesh = mj_name2id(m.get(), mjOBJ_GEOM, "mesh");
  int sphere = mj_name2id(m.get(), mjOBJ_GEOM, "sphere");
  int hfield = mj_name2id(m.get(), mjOBJ_GEOM, "hfield");

  m->opt.sdf_initpoints = 2;
  EXPECT_EQ(mj_maxContact(m.get(), plane, sdf, 0), 4);
  EXPECT_EQ(mj_maxContact(m.get(), sdf, plane, 0), 4);
  EXPECT_EQ(mj_maxContact(m.get(), mesh, sdf, 0), mjMAXCONPAIR);
  EXPECT_EQ(mj_maxContact(m.get(), sdf, mesh, 0), mjMAXCONPAIR);
  EXPECT_EQ(mj_maxContact(m.get(), hfield, sdf, 0), 0);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, sdf, 0), 2);

  m->opt.sdf_initpoints = 2 * mjMAXCONPAIR;
  EXPECT_EQ(mj_maxContact(m.get(), sphere, sdf, 0), mjMAXCONPAIR);

  m->opt.sdf_initpoints = -5;
  EXPECT_EQ(mj_maxContact(m.get(), sphere, sdf, 0), 0);
}

TEST_F(MjCollisionTest, FlexBVHPruningIncludesFlexGap) {
  // flex : flex intermediate BVH pruning includes flex_gap.
  static constexpr char kFlexFlexXml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="vol1" type="grid" dim="3" count="4 4 4"
                spacing="0.06 0.06 0.06" pos="0 0 0" radius="0.005" mass="1">
        <contact selfcollide="none" gap="0.05"/>
        <edge equality="true"/>
      </flexcomp>
      <flexcomp name="vol2" type="grid" dim="3" count="4 4 4"
                spacing="0.06 0.06 0.06" pos="0 0 0.22" radius="0.005" mass="1">
        <contact selfcollide="none" gap="0.05"/>
        <edge equality="true"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kFlexFlexXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;

  m->opt.disableflags |= mjDSBL_MIDPHASE;
  MjDataPtr d_all = MakeData(m);
  mj_forward(m.get(), d_all.get());
  ASSERT_GT(d_all->ncon, 0);

  m->opt.disableflags &= ~mjDSBL_MIDPHASE;
  MjDataPtr d_bvh = MakeData(m);
  mj_forward(m.get(), d_bvh.get());
  EXPECT_EQ(d_bvh->ncon, d_all->ncon);

  // body : flex intermediate BVH pruning includes flex_gap.
  static constexpr char kBodyFlexXml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 -0.15">
        <geom type="sphere" pos="-0.03 0 0.03" size="0.01"/>
        <geom type="sphere" pos="0.03 0 -0.03" size="0.01"/>
      </body>
      <flexcomp name="vol" type="grid" dim="3" count="4 4 4"
                spacing="0.06 0.06 0.06" pos="0 0 0" radius="0.005" mass="1">
        <contact selfcollide="none" gap="0.05"/>
        <edge equality="true"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr mb = LoadModelFromString(kBodyFlexXml, error, sizeof(error));
  ASSERT_THAT(mb.get(), NotNull()) << error;

  mb->opt.disableflags |= mjDSBL_MIDPHASE;
  MjDataPtr db_all = MakeData(mb);
  mj_forward(mb.get(), db_all.get());
  ASSERT_GT(db_all->ncon, 0);

  mb->opt.disableflags &= ~mjDSBL_MIDPHASE;
  MjDataPtr db_bvh = MakeData(mb);
  mj_forward(mb.get(), db_bvh.get());
  EXPECT_EQ(db_bvh->ncon, db_all->ncon);
}

TEST_F(MjCollisionTest, PlaneAndSdfFlexRespectGeomBitmaskInMidphase) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <asset>
      <mesh name="cube" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  -0.5 0.5 -0.5  0.5 0.5 -0.5
                                -0.5 -0.5  0.5  0.5 -0.5  0.5  -0.5 0.5  0.5  0.5 0.5  0.5"/>
    </asset>
    <worldbody>
      <geom name="far_sphere" type="sphere" size="0.01" pos="10 0 0"
            contype="1" conaffinity="1"/>
      <geom name="plane_filtered" type="plane" size="1 1 0.1" pos="0 0 0.05"
            contype="0" conaffinity="0"/>
      <geom name="sdf_filtered" type="sdf" mesh="cube" pos="0 0 -0.445"
            contype="2" conaffinity="2"/>
      <flexcomp name="cloth" type="grid" count="3 3 1" spacing=".05 .05 .05"
                dim="2" pos="0 0 0.053" radius=".005">
        <edge equality="true"/>
        <contact selfcollide="none" contype="1" conaffinity="1"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;

  MjDataPtr d_bvh = MakeData(m);
  mj_fwdPosition(m.get(), d_bvh.get());
  EXPECT_EQ(d_bvh->ncon, 0);

  m->opt.disableflags |= mjDSBL_MIDPHASE;
  MjDataPtr d_all = MakeData(m);
  mj_fwdPosition(m.get(), d_all.get());
  EXPECT_EQ(d_all->ncon, 0);
}

// Exclude two-pointer merge with out-of-order excludes, midphase, and <pair>.
TEST_F(MjCollisionTest, ExcludeMergeAndPairOverride) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="gw" size="0.3"/>
      <replicate count="20" offset="0.001 0 0">
        <body name="r">
          <freejoint/>
          <geom name="gr" size="0.3" contype="2" conaffinity="0"/>
        </body>
      </replicate>
      <body name="a" pos="0.01 0 0">
        <freejoint/>
        <geom name="ga" size="0.3"/>
      </body>
      <body name="b" pos="0.02 0 0">
        <freejoint/>
        <geom name="gb0" size="0.3"/>
        <geom name="gb1" pos="0 0 0.01" size="0.3"/>
      </body>
    </worldbody>
    <contact>
      <exclude body1="b" body2="a"/>
      <exclude body1="b" body2="r10"/>
      <exclude body1="a" body2="world"/>
      <exclude body1="r05" body2="world"/>
      <pair geom1="gb1" geom2="ga"/>
    </contact>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  for (int disable : {0, static_cast<int>(mjDSBL_MIDPHASE)}) {
    m->opt.disableflags = disable;
    mj_forward(m.get(), d.get());
    EXPECT_THAT(colliding_pairs(m.get(), d.get()),
                ElementsAre(GeomPair("ga", "gb1"), GeomPair("gb0", "gw"),
                            GeomPair("gb1", "gw")));
  }
}

// Narrowphase skips CCD stack buffers unless convex or custom colliders run.
TEST_F(MjCollisionTest, NarrowphaseCCDBuffer) {
  static constexpr char sphere_xml[] = R"(
  <mujoco>
    <worldbody>
      <replicate count="20" offset="0.01 0 0">
        <body>
          <freejoint/>
          <geom type="sphere" size="0.2"/>
        </body>
      </replicate>
    </worldbody>
  </mujoco>
  )";
  static constexpr char ellipsoid_xml[] = R"(
  <mujoco>
    <worldbody>
      <replicate count="20" offset="0.01 0 0">
        <body>
          <freejoint/>
          <geom type="ellipsoid" size="0.2 0.2 0.2"/>
        </body>
      </replicate>
    </worldbody>
  </mujoco>
  )";
  auto max_stack = [](const char* xml, int workers, int disable = 0) {
    char error[1024];
    MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
    EXPECT_THAT(m.get(), NotNull()) << error;
    m->opt.disableflags = disable;
    MjDataPtr d = MakeData(m);
    mju_threadpool(d.get(), workers);
    mj_forward(m.get(), d.get());
    d->maxuse_stack = 0;
    mj_collision(m.get(), d.get());
    EXPECT_EQ(d->ncon, 190);
    return static_cast<ptrdiff_t>(d->maxuse_stack);
  };

  ptrdiff_t prim = max_stack(sphere_xml, 4);
  ptrdiff_t convex = max_stack(ellipsoid_xml, 4);
  ptrdiff_t ccd_buf = max_stack(ellipsoid_xml, 0) - max_stack(sphere_xml, 0);
  ASSERT_GT(ccd_buf, 0);
  EXPECT_LE(prim, max_stack(ellipsoid_xml, 4, mjDSBL_NATIVECCD));
  EXPECT_LT(prim - max_stack(sphere_xml, 0), ccd_buf / 10);
  EXPECT_GE(convex - prim, 4 * ccd_buf);

  static mjfCollision ccd = mjCOLLISIONFUNC[mjGEOM_ELLIPSOID][mjGEOM_ELLIPSOID];
  mjfCollision orig = mjCOLLISIONFUNC[mjGEOM_SPHERE][mjGEOM_SPHERE];
  mjCOLLISIONFUNC[mjGEOM_SPHERE][mjGEOM_SPHERE] =
      [](const mjModel* m, mjData* d, mjPreContact* c, int g1, int g2,
         mjtNum margin) { return ccd(m, d, c, g1, g2, margin); };
  ptrdiff_t custom = max_stack(sphere_xml, 4);
  mjCOLLISIONFUNC[mjGEOM_SPHERE][mjGEOM_SPHERE] = orig;
  EXPECT_NEAR(custom, convex, ccd_buf / 4);
  EXPECT_LT(custom - prim, 10 * ccd_buf);
}

TEST_F(MjCollisionTest, ArenaExhaustionPrefixContactsAndUnclampedDemand) {
  auto make_xml = [](const std::string& size_tag) {
    return R"(
    <mujoco>
      )" + size_tag +
           R"(
      <option ccd_iterations="0">
        <flag multiccd="disable"/>
      </option>
      <worldbody>
        <geom name="floor" type="plane" size="5 5 0.1"/>
        <body name="boxes" pos="0 0 0.5">
          <joint name="z" type="slide" axis="0 0 1"/>
          <geom type="box" size="0.1 0.1 0.1" pos="-2.0 0 0"/>
          <geom type="box" size="0.1 0.1 0.1" pos="-1.5 0 0"/>
          <geom type="box" size="0.1 0.1 0.1" pos="-1.0 0 0"/>
          <geom type="box" size="0.1 0.1 0.1" pos="-0.5 0 0"/>
          <geom type="box" size="0.1 0.1 0.1" pos="0.0 0 0"/>
          <geom type="box" size="0.1 0.1 0.1" pos="0.5 0 0"/>
          <geom type="box" size="0.1 0.1 0.1" pos="1.0 0 0"/>
          <geom type="box" size="0.1 0.1 0.1" pos="1.5 0 0"/>
          <geom type="box" size="0.1 0.1 0.1" pos="2.0 0 0"/>
          <geom type="box" size="0.1 0.1 0.1" pos="2.5 0 0"/>
        </body>
      </worldbody>
    </mujoco>
    )";
  };

  char error[1024];
  std::string full_xml = make_xml("");
  MjModelPtr m_full = LoadModelFromString(full_xml, error, sizeof(error));
  ASSERT_THAT(m_full.get(), NotNull()) << error;
  MjDataPtr d_full = MakeData(m_full);
  ASSERT_THAT(d_full, NotNull());

  d_full->qpos[0] = -0.41;
  mj_kinematics(m_full.get(), d_full.get());
  mj_comPos(m_full.get(), d_full.get());
  mj_collision(m_full.get(), d_full.get());

  // 10 box-plane pairs * 4 contacts each = 40 contacts
  int full_ncon = d_full->ncon;
  ASSERT_EQ(full_ncon, 40);
  EXPECT_EQ(d_full->warning[mjWARN_CONTACTFULL].number, 0);

  // size memory via XML so stack allocations succeed, but only a prefix of
  // contacts fits on the arena during addPairContacts
  size_t mem = d_full->maxuse_arena - (full_ncon / 2) * sizeof(mjContact);
  ASSERT_GT(mem, d_full->maxuse_stack);
  std::string tight_xml =
      make_xml("<size memory=\"" + std::to_string(mem) + "\"/>");
  MjModelPtr m = LoadModelFromString(tight_xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mock_warning_handler.ExpectWarnings("Too many contacts");
  d->qpos[0] = -0.41;
  mj_kinematics(m.get(), d.get());
  mj_comPos(m.get(), d.get());
  mj_collision(m.get(), d.get());

  EXPECT_EQ(d->warning[mjWARN_CONTACTFULL].number, 1);
  EXPECT_EQ(d->warning[mjWARN_CONTACTFULL].lastinfo, full_ncon);
  ASSERT_GT(d->ncon, 0);
  ASSERT_LT(d->ncon, full_ncon);
  for (int i = 0; i < d->ncon; i++) {
    EXPECT_EQ(d->contact[i].geom[0], d_full->contact[i].geom[0]);
    EXPECT_EQ(d->contact[i].geom[1], d_full->contact[i].geom[1]);
    EXPECT_MJTNUM_EQ(d->contact[i].dist, d_full->contact[i].dist);
    EXPECT_MJTNUM_EQ(d->contact[i].pos[0], d_full->contact[i].pos[0]);
    EXPECT_MJTNUM_EQ(d->contact[i].pos[1], d_full->contact[i].pos[1]);
    EXPECT_MJTNUM_EQ(d->contact[i].pos[2], d_full->contact[i].pos[2]);
  }
}

TEST_F(MjCollisionTest, SingleCollidingGeomWithVisualGeoms) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <body name="a">
        <freejoint/>
        <geom name="a_vis0" size="0.5" contype="0" conaffinity="0"/>
        <geom name="a_col" pos="1 0 0" size="0.2"/>
        <geom name="a_vis1" pos="100 0 0" size="50"
          contype="0" conaffinity="0"/>
      </body>
      <body name="b">
        <freejoint/>
        <geom name="b_vis" size="0.5" contype="0" conaffinity="0"/>
        <geom name="b_col" pos="1.3 0 0" size="0.2"/>
      </body>
      <body name="c" pos="100 0 0">
        <freejoint/>
        <geom name="c_col" size="0.2"/>
      </body>
      <body name="d" pos="0.1 0 0">
        <freejoint/>
        <geom name="d_vis" size="0.2" contype="0" conaffinity="0"/>
      </body>
    </worldbody>
    <contact>
      <pair geom1="a_vis0" geom2="d_vis"/>
    </contact>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  int body_a = mj_name2id(m.get(), mjOBJ_BODY, "a");
  int body_b = mj_name2id(m.get(), mjOBJ_BODY, "b");
  ASSERT_EQ(m->body_bvhnum[body_a], 1);
  ASSERT_EQ(m->body_bvhnum[body_b], 1);

  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());
  EXPECT_THAT(
      colliding_pairs(m.get(), d.get()),
      ElementsAre(GeomPair("a_col", "b_col"), GeomPair("a_vis0", "d_vis")));

  // bvactive bypasses the single-BVH-node fast-path and marks active BVHs
  m->vis.global.bvactive = 1;
  mj_forward(m.get(), d.get());
  EXPECT_THAT(
      colliding_pairs(m.get(), d.get()),
      ElementsAre(GeomPair("a_col", "b_col"), GeomPair("a_vis0", "d_vis")));
  EXPECT_TRUE(d->bvh_active[m->body_bvhadr[body_a]]);
  EXPECT_TRUE(d->bvh_active[m->body_bvhadr[body_b]]);
  m->vis.global.bvactive = 0;

  // mjDSBL_MIDPHASE bypasses body_bvhnum == 1 and falls back to all-to-all
  int a_vis1 = mj_name2id(m.get(), mjOBJ_GEOM, "a_vis1");
  m->geom_contype[a_vis1] = 1;
  m->opt.disableflags = mjDSBL_MIDPHASE;
  mj_forward(m.get(), d.get());
  EXPECT_THAT(
      colliding_pairs(m.get(), d.get()),
      ElementsAre(GeomPair("a_col", "b_col"), GeomPair("a_vis0", "d_vis"),
                  GeomPair("a_vis1", "c_col")));
}

TEST_F(MjCollisionTest, ContactFilterCallbackIncludesZeroMaskGeoms) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <option>
      <flag midphase="disable"/>
    </option>
    <worldbody>
      <body name="a">
        <freejoint/>
        <geom name="a_zero" size="0.2" contype="0" conaffinity="0"/>
        <geom name="a_far" pos="100 0 0" size="0.2"/>
      </body>
      <body name="b" pos="0.1 0 0">
        <freejoint/>
        <geom name="b_col" size="0.2"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  mj_forward(m.get(), d.get());
  EXPECT_THAT(colliding_pairs(m.get(), d.get()), IsEmpty());

  mjcb_contactfilter = [](const mjModel*, mjData*, int, int) { return 0; };
  mj_forward(m.get(), d.get());
  mjcb_contactfilter = nullptr;
  EXPECT_THAT(colliding_pairs(m.get(), d.get()),
              ElementsAre(GeomPair("a_zero", "b_col")));
}

TEST_F(MjCollisionTest, MidphaseLeafBitmaskPrunesSubtree) {
  // Body a has two geoms: a_far (mask 1) far away so pairCollidable(a, b) is
  // true, and a_near (mask 2) overlapping body b's multi-geom BVH (mask 1).
  // Once a descends to leaf a_near, filterGeomBody(a_near, b) prunes b's
  // entire subtree without traversing b's children.
  static constexpr char kXml[] = R"(
  <mujoco>
    <visual>
      <global bvactive="true"/>
    </visual>
    <worldbody>
      <body name="a">
        <freejoint/>
        <geom name="a_near" size="0.5" contype="2" conaffinity="2"/>
        <geom name="a_far" pos="10 0 0" size="0.1"
          contype="1" conaffinity="1"/>
      </body>
      <body name="b">
        <freejoint/>
        <geom name="b0" pos="-0.1 0 0" size="0.05"
          contype="1" conaffinity="1"/>
        <geom name="b1" pos="0.1 0 0" size="0.05"
          contype="1" conaffinity="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  mj_forward(m.get(), d.get());
  EXPECT_THAT(colliding_pairs(m.get(), d.get()), IsEmpty());

  int body_b = mj_name2id(m.get(), mjOBJ_BODY, "b");
  int bvhadr_b = m->body_bvhadr[body_b];
  ASSERT_EQ(m->body_bvhnum[body_b], 3);
  EXPECT_FALSE(d->bvh_active[bvhadr_b + 1]);
  EXPECT_FALSE(d->bvh_active[bvhadr_b + 2]);
}

TEST_F(MjCollisionTest, SpeculativeFastGeomIgnoresNonCollidingGeoms) {
  // Tiny visual-only geom (1e-4) must not shrink min_size in
  // bodyPairHasFastGeom and disable BVH midphase under speculative CCD.
  static constexpr char kXml[] = R"(
  <mujoco>
    <option timestep="0.01">
      <flag speculative="enable"/>
    </option>
    <visual>
      <global bvactive="true"/>
    </visual>
    <worldbody>
      <body name="a">
        <freejoint/>
        <geom name="a_tiny_vis" size="0.0001" contype="0" conaffinity="0"/>
        <geom name="a_col0" pos="-0.2 0 0" size="0.1"/>
        <geom name="a_col1" pos="0.2 0 0" size="0.1"/>
      </body>
      <body name="b" pos="0 0 0.15">
        <freejoint/>
        <geom name="b_col0" pos="-0.2 0 0" size="0.1"/>
        <geom name="b_col1" pos="0.2 0 0" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  // step sweep = 0.01 * 1.0 = 0.01: > 1e-4 (tiny visual) and < 0.1 (colliders)
  d->qvel[2] = 1.0;
  mj_forward(m.get(), d.get());
  EXPECT_THAT(
      colliding_pairs(m.get(), d.get()),
      ElementsAre(GeomPair("a_col0", "b_col0"), GeomPair("a_col1", "b_col1")));
  int body_a = mj_name2id(m.get(), mjOBJ_BODY, "a");
  EXPECT_TRUE(d->bvh_active[m->body_bvhadr[body_a]]);
}

TEST_F(MjCollisionTest, BroadphaseSleepMocapAndStaticPlanes) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <option>
      <flag sleep="enable"/>
    </option>
    <worldbody>
      <body name="floor_body">
        <geom name="floor" type="plane" size="5 5 0.1"/>
      </body>
      <body name="vis_plane_body" pos="20 0 0">
        <geom name="vis_plane" type="plane" size="5 5 0.1"
          contype="0" conaffinity="0"/>
        <geom name="far_obstacle" size="0.1"/>
      </body>
      <body name="sleeper1" pos="0 0 0.09" sleep="init">
        <freejoint/>
        <geom name="s1" size="0.1"/>
      </body>
      <body name="sleeper2" pos="0.15 0 0.09" sleep="init">
        <freejoint/>
        <geom name="s2" size="0.1"/>
      </body>
      <body name="pusher" mocap="true" pos="5 0 0.09">
        <geom name="mocap_geom" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  // All dynamic bodies asleep and mocap far away: no contacts generated.
  mj_forward(m.get(), d.get());
  EXPECT_THAT(colliding_pairs(m.get(), d.get()), IsEmpty());

  // Move mocap body to touch only sleeper1 while all dynamic bodies are asleep:
  // mj_collision must detect mocap-vs-asleep even with nawake == 0, and
  // mj_forward then wakes the entire sleeping island {sleeper1, sleeper2}.
  int pusher = mj_name2id(m.get(), mjOBJ_BODY, "pusher");
  int mocap_id = m->body_mocapid[pusher];
  d->mocap_pos[3 * mocap_id + 0] = -0.1;
  mj_kinematics(m.get(), d.get());
  mj_comPos(m.get(), d.get());
  mj_collision(m.get(), d.get());
  EXPECT_THAT(colliding_pairs(m.get(), d.get()),
              ElementsAre(GeomPair("mocap_geom", "s1")));

  mj_forward(m.get(), d.get());
  EXPECT_THAT(colliding_pairs(m.get(), d.get()),
              ElementsAre(GeomPair("floor", "s1"), GeomPair("floor", "s2"),
                          GeomPair("mocap_geom", "s1"), GeomPair("s1", "s2")));
}

TEST_F(MjCollisionTest, BroadphaseSAPMatchesBruteForce) {
  std::string xml = R"(
  <mujoco>
    <option>
      <flag sleep="enable"/>
    </option>
    <worldbody>
      <body name="rail_a" pos="0 -0.15 0.5">
        <geom name="rail_a" type="box" size="2.0 0.03 0.03"/>
      </body>
      <body name="rail_b" pos="0 0.15 0.5">
        <geom name="rail_b" type="box" size="2.0 0.03 0.03"/>
      </body>
  )";
  for (int i = 0; i < 40; ++i) {
    double x = -0.9 + 0.048 * i;
    double y = 0.06 * std::sin(1.3 * i);
    double z = (i < 20 ? 0.2 : 0.8) + 0.05 * std::cos(0.9 * i);
    xml += "      <body pos=\"" + std::to_string(x) + " " + std::to_string(y) +
           " " + std::to_string(z) + "\"" + (i < 20 ? " sleep=\"init\"" : "") +
           ">\n";
    xml += "        <freejoint/>\n";
    xml += "        <geom name=\"s" + std::to_string(i) +
           "\" type=\"sphere\" size=\"0.055\"/>\n";
    xml += "      </body>\n";
  }
  xml += "    </worldbody>\n  </mujoco>\n";

  char error[1024];
  MjModelPtr m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  mj_kinematics(m.get(), d.get());
  mj_comPos(m.get(), d.get());
  mj_collision(m.get(), d.get());

  // Compute exact colliding awake sphere-sphere pairs by brute force
  std::vector<GeomPair> expected_spheres;
  for (int g1 = 0; g1 < m->ngeom; ++g1) {
    if (m->geom_type[g1] != mjGEOM_SPHERE) continue;
    int b1 = m->geom_bodyid[g1];
    bool awake1 = d->body_awake[b1] == mjS_AWAKE;
    for (int g2 = g1 + 1; g2 < m->ngeom; ++g2) {
      if (m->geom_type[g2] != mjGEOM_SPHERE) continue;
      int b2 = m->geom_bodyid[g2];
      bool awake2 = d->body_awake[b2] == mjS_AWAKE;
      if (!awake1 && !awake2) continue;
      mjtNum dx = d->geom_xpos[3 * g1 + 0] - d->geom_xpos[3 * g2 + 0];
      mjtNum dy = d->geom_xpos[3 * g1 + 1] - d->geom_xpos[3 * g2 + 1];
      mjtNum dz = d->geom_xpos[3 * g1 + 2] - d->geom_xpos[3 * g2 + 2];
      mjtNum rsum = m->geom_size[3 * g1] + m->geom_size[3 * g2];
      if (dx * dx + dy * dy + dz * dz < rsum * rsum) {
        std::string n1 = mj_id2name(m.get(), mjOBJ_GEOM, g1);
        std::string n2 = mj_id2name(m.get(), mjOBJ_GEOM, g2);
        expected_spheres.emplace_back(std::min(n1, n2), std::max(n1, n2));
      }
    }
  }
  std::sort(expected_spheres.begin(), expected_spheres.end());

  std::vector<GeomPair> actual_spheres;
  for (const GeomPair& p : colliding_pairs(m.get(), d.get())) {
    if (p.first[0] == 's' && p.second[0] == 's') {
      actual_spheres.push_back(p);
    }
  }
  EXPECT_GT(expected_spheres.size(), 5u);
  EXPECT_EQ(actual_spheres, expected_spheres);
}

TEST_F(MjCollisionTest, FlexSAPSelfCollideMatchesNarrow) {
  auto make_xml = [](const char* mode) {
    return std::string(R"(
    <mujoco>
      <worldbody>
        <flexcomp name="cloth" type="grid" dim="2" count="4 4 1"
                  spacing="0.05 0.05 0.05" pos="0 0 0.5" radius="0.008" mass="1">
          <edge equality="true"/>
          <contact selfcollide=")") +
           mode + R"("/>
        </flexcomp>
      </worldbody>
    </mujoco>)";
  };

  char error[1024];
  std::string xml_sap = make_xml("sap");
  std::string xml_narrow = make_xml("narrow");
  MjModelPtr m_sap = LoadModelFromString(xml_sap.c_str(), error, sizeof(error));
  ASSERT_THAT(m_sap.get(), NotNull()) << error;
  MjModelPtr m_narrow =
      LoadModelFromString(xml_narrow.c_str(), error, sizeof(error));
  ASSERT_THAT(m_narrow.get(), NotNull()) << error;

  MjDataPtr d_sap = MakeData(m_sap);
  MjDataPtr d_narrow = MakeData(m_narrow);

  // Fold the cloth along X so non-adjacent triangles self-intersect
  int nvert = m_sap->flex_vertnum[0];
  for (int v = 0; v < nvert; ++v) {
    int b = m_sap->flex_vertbodyid[v];
    int qadr = m_sap->jnt_qposadr[m_sap->body_jntadr[b]];
    double x = m_sap->body_pos[3 * b + 0];
    if (x > 0.02) {
      d_sap->qpos[qadr + 0] = -2.0 * x + 0.01;
      d_sap->qpos[qadr + 2] = 0.005;
    }
    d_narrow->qpos[qadr + 0] = d_sap->qpos[qadr + 0];
    d_narrow->qpos[qadr + 1] = d_sap->qpos[qadr + 1];
    d_narrow->qpos[qadr + 2] = d_sap->qpos[qadr + 2];
  }

  mj_forward(m_sap.get(), d_sap.get());
  mj_forward(m_narrow.get(), d_narrow.get());

  EXPECT_GT(d_sap->ncon, 0);
  ASSERT_EQ(d_sap->ncon, d_narrow->ncon);

  auto sorted_pairs = [](const mjData* d) {
    std::vector<std::pair<std::pair<int, int>, mjtNum>> out;
    out.reserve(d->ncon);
    for (int i = 0; i < d->ncon; ++i) {
      int e0 = std::min(d->contact[i].elem[0], d->contact[i].elem[1]);
      int e1 = std::max(d->contact[i].elem[0], d->contact[i].elem[1]);
      out.push_back({{e0, e1}, d->contact[i].dist});
    }
    std::sort(out.begin(), out.end());
    return out;
  };

  auto c_sap = sorted_pairs(d_sap.get());
  auto c_narrow = sorted_pairs(d_narrow.get());
  for (size_t i = 0; i < c_sap.size(); ++i) {
    EXPECT_EQ(c_sap[i].first, c_narrow[i].first);
    EXPECT_NEAR(c_sap[i].second, c_narrow[i].second, MjTol(1e-5, 1e-4));
  }
}

TEST_F(MjCollisionTest, HFieldSubgridAndPrismHelpers) {
  // Verify prism id encode/decode round-trip.
  int r = -1, c = -1, tri = -1;
  int id = mjc_hfieldPrismId(/*r=*/2, /*c=*/3, /*tri=*/1, /*nc=*/4);
  mjc_hfieldPrismCoords(id, /*nc=*/4, &r, &c, &tri);
  EXPECT_EQ(r, 2);
  EXPECT_EQ(c, 3);
  EXPECT_EQ(tri, 1);

  // 5x5 hfield covering [-1, 1] x [-1, 1] (4x4 cells, grid lines at -1, -0.5,
  // 0, 0.5, 1).
  static constexpr char kXml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hf" nrow="5" ncol="5" size="1 1 0.1 0.1"/>
    </asset>
    <worldbody>
      <geom name="hf" type="hfield" hfield="hf"/>
      <body name="far" pos="0 0 2.0">
        <freejoint/>
        <geom name="far_sphere" type="sphere" size="0.1"/>
      </body>
      <body name="mid" pos="0.25 0.25 0.05">
        <freejoint/>
        <geom name="mid_box" type="box" size="0.1 0.1 0.1"/>
      </body>
      <flexcomp name="gridline_flex" type="direct" dim="1" radius="0"
                point="0 0 0  0 0 0.05" element="0 1">
        <contact selfcollide="none"/>
      </flexcomp>
      <flexcomp name="border_flex" type="direct" dim="1" radius="0"
                pos="-1 -1 0" point="0 0 0  0 0 0.05" element="0 1">
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());

  int g_hf = mj_name2id(m.get(), mjOBJ_GEOM, "hf");
  int g_far = mj_name2id(m.get(), mjOBJ_GEOM, "far_sphere");
  int g_mid = mj_name2id(m.get(), mjOBJ_GEOM, "mid_box");

  int rmin = 0, rmax = 0, cmin = 0, cmax = 0;
  mjtNum zmin = 0;

  // Disjoint geom returns 0.
  EXPECT_EQ(mjc_hfieldSubgridGeom(m.get(), d.get(), g_hf, g_far, 0, &rmin,
                                  &rmax, &cmin, &cmax, &zmin),
            0);

  // Box centered at (0.25, 0.25) with half-size 0.1 lies strictly inside cell
  // (2, 2).
  EXPECT_EQ(mjc_hfieldSubgridGeom(m.get(), d.get(), g_hf, g_mid, 0, &rmin,
                                  &rmax, &cmin, &cmax, &zmin),
            1);
  EXPECT_EQ(rmin, 2);
  EXPECT_EQ(rmax, 3);
  EXPECT_EQ(cmin, 2);
  EXPECT_EQ(cmax, 3);

  // Zero-radius vertical flex segment on interior grid line (0, 0) expands
  // symmetrically to [1, 3).
  int f_grid = mj_name2id(m.get(), mjOBJ_FLEX, "gridline_flex");
  EXPECT_EQ(mjc_hfieldSubgridElem(m.get(), d.get(), g_hf, f_grid, 0, 0, &rmin,
                                  &rmax, &cmin, &cmax, &zmin),
            1);
  EXPECT_EQ(rmin, 1);
  EXPECT_EQ(rmax, 3);
  EXPECT_EQ(cmin, 1);
  EXPECT_EQ(cmax, 3);

  // Zero-radius vertical flex segment on outer border (-1, -1) expands to [0,
  // 1).
  int f_border = mj_name2id(m.get(), mjOBJ_FLEX, "border_flex");
  EXPECT_EQ(mjc_hfieldSubgridElem(m.get(), d.get(), g_hf, f_border, 0, 0, &rmin,
                                  &rmax, &cmin, &cmax, &zmin),
            1);
  EXPECT_EQ(rmin, 0);
  EXPECT_EQ(rmax, 1);
  EXPECT_EQ(cmin, 0);
  EXPECT_EQ(cmax, 1);
}

TEST_F(MjCollisionTest, HFieldThreadedParity) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hf" nrow="9" ncol="9" size="1 1 0.1 0.1"/>
    </asset>
    <worldbody>
      <geom name="hf" type="hfield" hfield="hf"/>
      <body name="b1" pos="0 0 0.08">
        <freejoint/>
        <geom type="box" size="0.3 0.3 0.1"/>
      </body>
      <body name="b2" pos="0.5 0.2 0.08">
        <freejoint/>
        <geom type="sphere" size="0.1"/>
      </body>
      <body name="b3" pos="-0.4 -0.3 0.08">
        <freejoint/>
        <geom type="capsule" size="0.08 0.15"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;

  MjDataPtr d_serial = MakeData(m);
  mj_forward(m.get(), d_serial.get());
  ASSERT_GT(d_serial->ncon, 0);

  MjDataPtr d_threaded = MakeData(m);
  mju_threadpool(d_threaded.get(), 4);
  mj_forward(m.get(), d_threaded.get());

  EXPECT_EQ(d_threaded->ncon, d_serial->ncon);
  EXPECT_MJTNUM_EQ(d_threaded->contact[0].dist, d_serial->contact[0].dist);
  EXPECT_MJTNUM_EQ(d_threaded->contact[0].pos[2], d_serial->contact[0].pos[2]);
  int last = d_serial->ncon - 1;
  EXPECT_MJTNUM_EQ(d_threaded->contact[last].dist,
                   d_serial->contact[last].dist);
  EXPECT_MJTNUM_EQ(d_threaded->contact[last].pos[2],
                   d_serial->contact[last].pos[2]);
}

TEST_F(MjCollisionTest, HFieldMaxConPairReductionAndArenaFallback) {
  // Large flat box overlapping 14x14x2 = 392 prisms (> mjMAXCONPAIR).
  static constexpr char kXmlNormal[] = R"(
  <mujoco>
    <asset>
      <hfield name="hf" nrow="15" ncol="15" size="1 1 0.1 0.1"/>
    </asset>
    <worldbody>
      <geom name="hf" type="hfield" hfield="hf"/>
      <body name="big_box" pos="0 0 0.09">
        <freejoint/>
        <geom type="box" size="0.9 0.9 0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXmlNormal, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());
  EXPECT_EQ(d->ncon, mjMAXCONPAIR);

  // Tight arena budget forces canAllocPairs() to fall back to unexpanded
  // mjc_ConvexHField without triggering a stack overflow in mj_narrowphase.
  static constexpr char kXmlTight[] = R"(
  <mujoco>
    <size memory="30K"/>
    <asset>
      <hfield name="hf" nrow="15" ncol="15" size="1 1 0.1 0.1"/>
    </asset>
    <worldbody>
      <geom name="hf" type="hfield" hfield="hf"/>
      <body name="big_box" pos="0 0 0.09">
        <freejoint/>
        <geom type="box" size="0.9 0.9 0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m_tight = LoadModelFromString(kXmlTight, error, sizeof(error));
  ASSERT_THAT(m_tight.get(), NotNull()) << error;
  MjDataPtr d_tight = MakeData(m_tight);
  mj_kinematics(m_tight.get(), d_tight.get());
  mj_collision(m_tight.get(), d_tight.get());
  EXPECT_GT(d_tight->ncon, 0);
  EXPECT_LE(d_tight->ncon, mjMAXCONPAIR);
}

TEST_F(MjCollisionTest, HFieldFlexCollisionAndPinnedSkip) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hf" nrow="5" ncol="5" size="1 1 0.1 0.1"/>
    </asset>
    <worldbody>
      <geom name="hf" type="hfield" hfield="hf"/>
      <flexcomp name="free_tri" type="direct" dim="2" radius="0.02"
                point="-0.2 -0.2 0.01  0.2 -0.2 0.01  0 0.2 0.01"
                element="0 1 2">
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());
  EXPECT_GT(d->ncon, 0);

  // Pinning a vertex to body 0 (same body as the heightfield) skips the element
  // in pushGeomElem.
  static constexpr char kXmlPinned[] = R"(
  <mujoco>
    <asset>
      <hfield name="hf" nrow="5" ncol="5" size="1 1 0.1 0.1"/>
    </asset>
    <worldbody>
      <geom name="hf" type="hfield" hfield="hf"/>
      <flexcomp name="pinned_tri" type="direct" dim="2" radius="0.02"
                point="-0.2 -0.2 0.01  0.2 -0.2 0.01  0 0.2 0.01"
                element="0 1 2">
        <pin id="0"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m_pin = LoadModelFromString(kXmlPinned, error, sizeof(error));
  ASSERT_THAT(m_pin.get(), NotNull()) << error;
  MjDataPtr d_pin = MakeData(m_pin);
  mj_forward(m_pin.get(), d_pin.get());
  EXPECT_EQ(d_pin->ncon, 0);
}

TEST_F(MjCollisionTest, PlaneCapsuleNearParallelFrame) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <default>
      <geom margin="1"/>
    </default>
    <worldbody>
      <geom type="plane" size="1 1 .1" zaxis=".4 .3 .866"/>
      <body pos="0 0 .1">
        <freejoint/>
        <geom type="capsule" size=".08 .2" zaxis=".401 .3 .866" mass="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  mj_forward(m.get(), d.get());
  ASSERT_EQ(d->ncon, 2);

  const mjtNum ortho_tol = MjTol(1e-12, 1e-6);
  const mjtNum normal[3] = {d->geom_xmat[2], d->geom_xmat[5], d->geom_xmat[8]};
  for (int c = 0; c < d->ncon; ++c) {
    const mjtNum* f = d->contact[c].frame;
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(f[i], normal[i], ortho_tol);
      for (int j = 0; j < 3; ++j) {
        EXPECT_NEAR(mju_dot3(f + 3 * i, f + 3 * j), (i == j) ? 1 : 0,
                    ortho_tol);
      }
    }
  }
}

TEST_F(MjCollisionTest, PlaneCylinderAlignedOnTiltedPlane) {
  const char* kEulers[] = {"17 23 9", "11 78 4", "3 0 0"};
  const mjtNum tol = MjTol(1e-13, 1e-5);

  for (const char* euler : kEulers) {
    std::string xml = std::string(R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <default>
      <geom condim="1"/>
    </default>
    <worldbody>
      <geom type="plane" size="2 2 .1" euler=")") +
                      euler + R"("/>
      <body>
        <freejoint/>
        <geom type="cylinder" size=".05 .1" mass="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
    char error[1024];
    MjModelPtr m = LoadModelFromString(xml.c_str(), error, sizeof(error));
    ASSERT_THAT(m.get(), NotNull()) << error;
    MjDataPtr d = MakeData(m);

    // Align the cylinder with the tilted plane normal so the bottom cap
    // penetrates by 0.05.
    mj_kinematics(m.get(), d.get());
    const mjtNum normal[3] = {d->geom_xmat[2], d->geom_xmat[5],
                              d->geom_xmat[8]};
    for (int i = 0; i < 3; ++i) {
      d->qpos[i] = static_cast<mjtNum>(0.05) * normal[i];
    }
    for (int i = 0; i < 4; ++i) {
      d->qpos[3 + i] = m->geom_quat[i];
    }
    mj_forward(m.get(), d.get());

    ASSERT_EQ(d->ncon, 3);
    const mjtNum* cmat = d->geom_xmat + 9;
    const mjtNum cpos[3] = {d->geom_xpos[3], d->geom_xpos[4], d->geom_xpos[5]};
    const mjtNum radius = m->geom_size[3];
    const mjtNum half_height = m->geom_size[4];
    mjtNum expected_pos0[3];
    for (int i = 0; i < 3; ++i) {
      expected_pos0[i] = cpos[i] + cmat[3 * i] * radius -
                         cmat[3 * i + 2] * half_height +
                         normal[i] * static_cast<mjtNum>(0.025);
    }
    for (int c = 0; c < d->ncon; ++c) {
      EXPECT_NEAR(d->contact[c].dist, static_cast<mjtNum>(-0.05), tol);
    }
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(d->contact[0].pos[i], expected_pos0[i], tol);
    }
  }
}

// Verify that contacts in d->contact are ordered monotonically by canonical
// (min_body, max_body) signature.
void ExpectCanonicalBodyPairOrder(const mjModel* m, const mjData* d) {
  uint32_t prev_sig = 0;
  for (int i = 0; i < d->ncon; ++i) {
    int b1 = m->geom_bodyid[d->contact[i].geom[0]];
    int b2 = m->geom_bodyid[d->contact[i].geom[1]];
    uint32_t sig = (static_cast<uint32_t>(std::min(b1, b2)) << 16) |
                   static_cast<uint32_t>(std::max(b1, b2));
    EXPECT_GE(sig, prev_sig);
    prev_sig = sig;
  }
}

TEST_F(MjCollisionTest, BroadphaseRadixSortMatchesBruteForce) {
  // 120 spheres along a tilted 45-degree diagonal (triggers sapRadixSort
  // n >= 96 and bfRadixSort 2-pass counting sort), spanning negative, zero,
  // and positive coordinates plus a world plane.
  std::string xml = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="20 20 0.1"/>
  )";
  for (int i = 0; i < 120; ++i) {
    double t = (i - 60) * 0.045;
    double x = t;
    double y = t + 0.03 * std::sin(1.7 * i);
    double z = 0.05 + 0.02 * std::cos(1.1 * i);
    xml += "      <body pos=\"" + std::to_string(x) + " " + std::to_string(y) +
           " " + std::to_string(z) + "\">\n";
    xml += "        <freejoint/>\n";
    xml += "        <geom name=\"s" + std::to_string(i) +
           "\" type=\"sphere\" size=\"0.055\"/>\n";
    xml += "      </body>\n";
  }
  xml += "    </worldbody>\n  </mujoco>\n";

  char error[1024];
  MjModelPtr m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_kinematics(m.get(), d.get());
  mj_comPos(m.get(), d.get());
  mj_collision(m.get(), d.get());

  // Exact brute-force sphere-sphere and sphere-plane contacts
  std::vector<GeomPair> expected;
  for (int g1 = 1; g1 < m->ngeom; ++g1) {
    if (d->geom_xpos[3 * g1 + 2] < m->geom_size[3 * g1]) {
      expected.emplace_back("floor", mj_id2name(m.get(), mjOBJ_GEOM, g1));
    }
    for (int g2 = g1 + 1; g2 < m->ngeom; ++g2) {
      mjtNum dx = d->geom_xpos[3 * g1 + 0] - d->geom_xpos[3 * g2 + 0];
      mjtNum dy = d->geom_xpos[3 * g1 + 1] - d->geom_xpos[3 * g2 + 1];
      mjtNum dz = d->geom_xpos[3 * g1 + 2] - d->geom_xpos[3 * g2 + 2];
      mjtNum rsum = m->geom_size[3 * g1] + m->geom_size[3 * g2];
      if (dx * dx + dy * dy + dz * dz < rsum * rsum) {
        std::string n1 = mj_id2name(m.get(), mjOBJ_GEOM, g1);
        std::string n2 = mj_id2name(m.get(), mjOBJ_GEOM, g2);
        expected.emplace_back(std::min(n1, n2), std::max(n1, n2));
      }
    }
  }
  std::sort(expected.begin(), expected.end());

  EXPECT_GT(expected.size(), 150u);
  EXPECT_EQ(colliding_pairs(m.get(), d.get()), expected);
  ExpectCanonicalBodyPairOrder(m.get(), d.get());
}

TEST_F(MjCollisionTest, BroadphaseSapRadixEdgeCases) {
  // 1) All-equal x_min keys (last_pass < 0 in sapRadixSort): 100 coincident
  // equal-radius spheres at the origin.
  {
    std::string xml = "<mujoco><worldbody>\n";
    for (int i = 0; i < 100; ++i) {
      xml +=
          "  <body pos=\"0 0 0\"><freejoint/>"
          "<geom type=\"sphere\" size=\"0.05\"/></body>\n";
    }
    xml += "</worldbody></mujoco>\n";

    char error[1024];
    MjModelPtr m = LoadModelFromString(xml.c_str(), error, sizeof(error));
    ASSERT_THAT(m.get(), NotNull()) << error;
    MjDataPtr d = MakeData(m);
    mj_kinematics(m.get(), d.get());
    mj_comPos(m.get(), d.get());
    mj_collision(m.get(), d.get());

    EXPECT_EQ(d->ncon, 100 * 99 / 2);
    ExpectCanonicalBodyPairOrder(m.get(), d.get());
  }

  // 2) Explicit -0.0f / +0.0f x_min endpoints reaching floatToRadixKey in
  // sapRadixSort (9x9 flex -> 128 elements >= 96, elongated in X so axis = 0).
  {
    auto make_xml = [](const char* mode) {
      return std::string(R"(
      <mujoco>
        <worldbody>
          <flexcomp name="cloth" type="grid" dim="2" count="9 9 1"
            spacing="0.08 0.04 0.04" pos="0 0 0.5" radius="0.008" mass="1">
            <edge equality="true"/>
            <contact selfcollide=")") +
             mode + R"("/>
          </flexcomp>
        </worldbody>
      </mujoco>)";
    };

    char error[1024];
    std::string xml_sap = make_xml("sap");
    std::string xml_narrow = make_xml("narrow");
    MjModelPtr m_sap =
        LoadModelFromString(xml_sap.c_str(), error, sizeof(error));
    ASSERT_THAT(m_sap.get(), NotNull()) << error;
    MjModelPtr m_narrow =
        LoadModelFromString(xml_narrow.c_str(), error, sizeof(error));
    ASSERT_THAT(m_narrow.get(), NotNull()) << error;
    MjDataPtr d_sap = MakeData(m_sap);
    MjDataPtr d_narrow = MakeData(m_narrow);

    // Fold cloth so self-collisions occur, then place two elements at exact
    // -0.0 and +0.0 x_min endpoints (center = +-0.0, radius = 0.0 along X).
    int nvert = m_sap->flex_vertnum[0];
    for (int v = 0; v < nvert; ++v) {
      int b = m_sap->flex_vertbodyid[v];
      int qadr = m_sap->jnt_qposadr[m_sap->body_jntadr[b]];
      double x = m_sap->body_pos[3 * b + 0];
      if (x > 0.04) {
        d_sap->qpos[qadr + 0] = -2.0 * x + 0.01;
        d_sap->qpos[qadr + 2] = 0.005;
      }
      d_narrow->qpos[qadr + 0] = d_sap->qpos[qadr + 0];
      d_narrow->qpos[qadr + 1] = d_sap->qpos[qadr + 1];
      d_narrow->qpos[qadr + 2] = d_sap->qpos[qadr + 2];
    }

    for (auto [m, d] : std::vector<std::pair<mjModel*, mjData*>>{
             {m_sap.get(), d_sap.get()}, {m_narrow.get(), d_narrow.get()}}) {
      mj_kinematics(m, d);
      mj_comPos(m, d);
      mj_flex(m, d);
      const mjtNum* bvh =
          d->bvh_aabb_dyn + 6 * (m->flex_bvhadr[0] - m->nbvhstatic);
      int axis =
          (bvh[3] > bvh[4] && bvh[3] > bvh[5]) ? 0 : (bvh[4] > bvh[5] ? 1 : 2);
      // Higher element id gets -0.0, lower gets +0.0 along the SAP axis so
      // canonicalization keeps their relative order consistent with SAPcmp.
      d->flexelem_aabb[6 * 10 + axis] = 0.0;
      d->flexelem_aabb[6 * 10 + 3 + axis] = 0.0;
      d->flexelem_aabb[6 * 20 + axis] = -0.0;
      d->flexelem_aabb[6 * 20 + 3 + axis] = 0.0;
      ASSERT_TRUE(std::signbit(
          static_cast<float>(d->flexelem_aabb[6 * 20 + axis] -
                             d->flexelem_aabb[6 * 20 + 3 + axis])));
      mj_collision(m, d);
    }

    EXPECT_GT(d_sap->ncon, 10);
    EXPECT_EQ(d_sap->ncon, d_narrow->ncon);
  }
}

TEST_F(MjCollisionTest, BroadphaseSparseHighIdRadix) {
  // 1) 4-pass unsorted case: 320 bodies (bound = 321 > 256, n >= 128,
  // n < bound/2) with colliding non-floor box pairs across low and high body
  // IDs placed so SAP emits them after (0, 272) in reverse body-ID order.
  {
    std::string xml = R"(
    <mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="200 200 0.1"/>
    )";
    for (int i = 0; i < 320; ++i) {
      double x = i * 0.5;
      double z = 0.05;
      // Place colliding non-floor pairs in reverse ID order along X:
      // (271, 272) at x=100, (260, 261) at x=110, (2, 3) at x=120.
      if (i == 270) {
        x = 100.0;
        z = 1.0;
      } else if (i == 271) {
        x = 100.05;
        z = 1.0;
      } else if (i == 259) {
        x = 110.0;
        z = 1.0;
      } else if (i == 260) {
        x = 110.05;
        z = 1.0;
      } else if (i == 1) {
        x = 120.0;
        z = 0.05;
      } else if (i == 2) {
        x = 120.05;
        z = 0.05;
      }
      xml += "      <body pos=\"" + std::to_string(x) + " 0 " +
             std::to_string(z) + "\">\n";
      xml += "        <freejoint/>\n";
      if (i < 138 || i == 259 || i == 260 || i == 270 || i == 271) {
        xml += "        <geom name=\"c" + std::to_string(i) +
               "\" type=\"box\" size=\"0.06 0.06 0.06\"/>\n";
      } else {
        xml +=
            "        <geom type=\"sphere\" size=\"0.05\" contype=\"0\""
            " conaffinity=\"0\"/>\n";
      }
      xml += "      </body>\n";
    }
    xml += "    </worldbody>\n  </mujoco>\n";

    char error[1024];
    MjModelPtr m = LoadModelFromString(xml.c_str(), error, sizeof(error));
    ASSERT_THAT(m.get(), NotNull()) << error;
    MjDataPtr d = MakeData(m);
    mj_kinematics(m.get(), d.get());
    mj_comPos(m.get(), d.get());
    mj_collision(m.get(), d.get());

    ExpectCanonicalBodyPairOrder(m.get(), d.get());
    int floor_contacts = 0;
    int low_pair = 0, mid_pair = 0, high_pair = 0;
    for (int i = 0; i < d->ncon; ++i) {
      int b1 = m->geom_bodyid[d->contact[i].geom[0]];
      int b2 = m->geom_bodyid[d->contact[i].geom[1]];
      int lo = std::min(b1, b2);
      int hi = std::max(b1, b2);
      if (lo == 0) {
        floor_contacts++;
      } else if (lo == 2 && hi == 3) {
        low_pair++;
      } else if (lo == 260 && hi == 261) {
        mid_pair++;
      } else if (lo == 271 && hi == 272) {
        high_pair++;
      }
    }
    EXPECT_EQ(floor_contacts, 138 * 4);
    EXPECT_GT(low_pair, 0);
    EXPECT_GT(mid_pair, 0);
    EXPECT_GT(high_pair, 0);
  }

  // 2) 3-pass (odd) case exercising the src != pair memcpy copy-back:
  // all pairs have hi < 256 (pass 3 skipped) while lo spans < 256 and >= 256.
  {
    std::string xml = R"(
    <mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="200 200 0.1"/>
    )";
    for (int i = 0; i < 320; ++i) {
      double x = i * 0.5;
      double z = 0.05;
      // Non-floor pairs (200, 270) at x=100 and (2, 3) at x=120:
      // hi in {0, 2, 200} (< 256), lo includes 270 (>= 256).
      if (i == 199) {
        x = 100.0;
        z = 1.0;
      } else if (i == 269) {
        x = 100.05;
        z = 1.0;
      } else if (i == 1) {
        x = 120.0;
        z = 0.05;
      } else if (i == 2) {
        x = 120.05;
        z = 0.05;
      }
      xml += "      <body pos=\"" + std::to_string(x) + " 0 " +
             std::to_string(z) + "\">\n";
      xml += "        <freejoint/>\n";
      if (i < 138 || i == 199 || i == 269) {
        xml += "        <geom name=\"c" + std::to_string(i) +
               "\" type=\"box\" size=\"0.06 0.06 0.06\"/>\n";
      } else {
        xml +=
            "        <geom type=\"sphere\" size=\"0.05\" contype=\"0\""
            " conaffinity=\"0\"/>\n";
      }
      xml += "      </body>\n";
    }
    xml += "    </worldbody>\n  </mujoco>\n";

    char error[1024];
    MjModelPtr m = LoadModelFromString(xml.c_str(), error, sizeof(error));
    ASSERT_THAT(m.get(), NotNull()) << error;
    MjDataPtr d = MakeData(m);
    mj_kinematics(m.get(), d.get());
    mj_comPos(m.get(), d.get());
    mj_collision(m.get(), d.get());

    ExpectCanonicalBodyPairOrder(m.get(), d.get());
    int floor_contacts = 0;
    int low_pair = 0, cross_pair = 0;
    for (int i = 0; i < d->ncon; ++i) {
      int b1 = m->geom_bodyid[d->contact[i].geom[0]];
      int b2 = m->geom_bodyid[d->contact[i].geom[1]];
      int lo = std::min(b1, b2);
      int hi = std::max(b1, b2);
      if (lo == 0) {
        floor_contacts++;
      } else if (lo == 2 && hi == 3) {
        low_pair++;
      } else if (lo == 200 && hi == 270) {
        cross_pair++;
      }
    }
    EXPECT_EQ(floor_contacts, 138 * 4);
    EXPECT_GT(low_pair, 0);
    EXPECT_GT(cross_pair, 0);
  }
}

TEST_F(MjCollisionTest, FlexSAPOrderedLargeMatchesNarrow) {
  // 8x8 grid -> 128 triangles (>= 96), exercising sapRadixSort with ordered=1
  auto make_xml = [](const char* mode) {
    return std::string(R"(
    <mujoco>
      <worldbody>
        <flexcomp name="cloth" type="grid" dim="2" count="9 9 1"
          spacing="0.04 0.04 0.04" pos="0 0 0.5" radius="0.008" mass="1">
          <edge equality="true"/>
          <contact selfcollide=")") +
           mode + R"("/>
        </flexcomp>
      </worldbody>
    </mujoco>)";
  };

  char error[1024];
  std::string xml_sap = make_xml("sap");
  std::string xml_narrow = make_xml("narrow");
  MjModelPtr m_sap = LoadModelFromString(xml_sap.c_str(), error, sizeof(error));
  ASSERT_THAT(m_sap.get(), NotNull()) << error;
  MjModelPtr m_narrow =
      LoadModelFromString(xml_narrow.c_str(), error, sizeof(error));
  ASSERT_THAT(m_narrow.get(), NotNull()) << error;

  MjDataPtr d_sap = MakeData(m_sap);
  MjDataPtr d_narrow = MakeData(m_narrow);

  int nvert = m_sap->flex_vertnum[0];
  for (int v = 0; v < nvert; ++v) {
    int b = m_sap->flex_vertbodyid[v];
    int qadr = m_sap->jnt_qposadr[m_sap->body_jntadr[b]];
    double x = m_sap->body_pos[3 * b + 0];
    if (x > 0.02) {
      d_sap->qpos[qadr + 0] = -2.0 * x + 0.01;
      d_sap->qpos[qadr + 2] = 0.005;
    }
    d_narrow->qpos[qadr + 0] = d_sap->qpos[qadr + 0];
    d_narrow->qpos[qadr + 1] = d_sap->qpos[qadr + 1];
    d_narrow->qpos[qadr + 2] = d_sap->qpos[qadr + 2];
  }

  mj_forward(m_sap.get(), d_sap.get());
  mj_forward(m_narrow.get(), d_narrow.get());

  EXPECT_GT(d_sap->ncon, 20);
  ASSERT_EQ(d_sap->ncon, d_narrow->ncon);

  auto sorted_pairs = [](const mjData* d) {
    std::vector<std::pair<std::pair<int, int>, mjtNum>> out;
    out.reserve(d->ncon);
    for (int i = 0; i < d->ncon; ++i) {
      int e0 = std::min(d->contact[i].elem[0], d->contact[i].elem[1]);
      int e1 = std::max(d->contact[i].elem[0], d->contact[i].elem[1]);
      out.push_back({{e0, e1}, d->contact[i].dist});
    }
    std::sort(out.begin(), out.end());
    return out;
  };

  auto c_sap = sorted_pairs(d_sap.get());
  auto c_narrow = sorted_pairs(d_narrow.get());
  for (size_t i = 0; i < c_sap.size(); ++i) {
    EXPECT_EQ(c_sap[i].first, c_narrow[i].first);
    EXPECT_EQ(c_sap[i].second, c_narrow[i].second);
  }
}

TEST_F(MjCollisionTest, BroadphaseFlushZeroMarginBoxesIdentityFrame) {
  // Two zero-margin boxes placed mathematically flush at z = 0.002:
  //   floor_box top:    0.001 + 0.001 = 0.002
  //   top_box bottom:  (0.002 + 0.016) - 0.016 > 0.002 by 1-4 ULPs
  // Symmetric X-axis spacers force a strictly diagonal covariance with largest
  // variance along X, so even mju_eig3Tol returns the exact identity SAP frame
  // and Z is a secondary mjtNum axis in mj_SAP.
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 0">
        <freejoint/>
        <geom name="floor_box" type="box" pos="0 0 0.001"
              size="0.0864 0.0630 0.001"/>
      </body>
      <body pos="0 0 0.002">
        <freejoint/>
        <geom name="top_box" type="box" pos="0 0 0.016"
              size="0.010 0.010 0.016"/>
      </body>
      <body pos="-1 0 0.0095">
        <freejoint/>
        <geom type="sphere" size="0.01"/>
      </body>
      <body pos="1 0 0.0095">
        <freejoint/>
        <geom type="sphere" size="0.01"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());

  EXPECT_EQ(colliding_pairs(m.get(), d.get()),
            std::vector<GeomPair>(4, {"floor_box", "top_box"}));
  EXPECT_EQ(d->ncon, 4);
}

TEST_F(MjCollisionTest, BroadphaseAlignedAndTiltedFrameMatch) {
  // Axis-aligned identity fast path and tilted PCA power-iteration frame must
  // produce identical contacts for an asymmetric mesh, box, and flex.
  auto make_xml = [](bool tilted_frame) {
    std::string s = R"(
    <mujoco>
      <asset>
        <mesh name="asym"
          vertex="0.1 0 0  0.3 0 0  0.1 0.2 0  0.1 0 0.2"/>
      </asset>
      <worldbody>
        <body name="mesh_body" pos="0.0 0.0 0.05" euler="15 25 35">
          <freejoint/>
          <geom name="mesh_geom" type="mesh" mesh="asym"/>
        </body>
        <body name="box_body" pos="0.12 0.05 0.05">
          <freejoint/>
          <geom name="box_geom" type="box" size="0.06 0.06 0.06"/>
        </body>
        <flexcomp name="sheet6" type="grid" dim="2" count="3 2 1"
          spacing="0.05 0.05 0.05" pos="0.12 0.02 0.05"
          radius="0.02" mass="1">
          <edge equality="true"/>
          <contact selfcollide="none"/>
        </flexcomp>
    )";
    if (tilted_frame) {
      s += R"(
        <body pos="5 5 2">
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
        <body pos="-5 -5 -2">
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
      )";
    } else {
      s += R"(
        <body pos="5 0 0">
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
        <body pos="-5 0 0">
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
      )";
    }
    s += "      </worldbody>\n    </mujoco>\n";
    return s;
  };

  char error[1024];
  std::string xml_aligned = make_xml(false);
  std::string xml_tilted = make_xml(true);
  MjModelPtr ma =
      LoadModelFromString(xml_aligned.c_str(), error, sizeof(error));
  ASSERT_THAT(ma.get(), NotNull()) << error;
  MjModelPtr mt = LoadModelFromString(xml_tilted.c_str(), error, sizeof(error));
  ASSERT_THAT(mt.get(), NotNull()) << error;
  MjDataPtr da = MakeData(ma);
  MjDataPtr dt = MakeData(mt);
  mj_forward(ma.get(), da.get());
  mj_forward(mt.get(), dt.get());

  EXPECT_GT(da->ncon, 0);
  ASSERT_EQ(da->ncon, dt->ncon);
  int has_mesh_box = 0, has_flex = 0;
  for (int i = 0; i < da->ncon; ++i) {
    EXPECT_EQ(da->contact[i].geom[0], dt->contact[i].geom[0]);
    EXPECT_EQ(da->contact[i].geom[1], dt->contact[i].geom[1]);
    EXPECT_EQ(da->contact[i].flex[0], dt->contact[i].flex[0]);
    EXPECT_EQ(da->contact[i].flex[1], dt->contact[i].flex[1]);
    EXPECT_EQ(da->contact[i].elem[0], dt->contact[i].elem[0]);
    EXPECT_EQ(da->contact[i].elem[1], dt->contact[i].elem[1]);
    EXPECT_EQ(da->contact[i].dist, dt->contact[i].dist);
    if (da->contact[i].flex[1] >= 0) {
      has_flex = 1;
    } else if (da->contact[i].geom[0] >= 0 && da->contact[i].geom[1] >= 0) {
      has_mesh_box = 1;
    }
  }
  EXPECT_EQ(has_mesh_box, 1);
  EXPECT_EQ(has_flex, 1);
}

TEST_F(MjCollisionTest, FlexAAMMVertexCountsAlignedAndTiltedMatch) {
  // Exercise all flexAAMM vertex-count paths in both axis-aligned and tilted
  // SAP frames:
  // - tri3 (3 verts): < 4 init + tail, probed only at tail vertex 2
  // - quad4 (4 verts): exact multiple of 4 (no scalar tail)
  // - sheet10 (10 verts): 2 full 4-wide SIMD chunks (0..3, 4..7) + 2-vert tail
  //   (8..9), with separate probes overlapping ONLY chunk 1 (verts 4..7 at
  //   y=-0.50) and ONLY the scalar tail (vert 9 at x=+0.52).
  auto make_xml = [](bool tilted_frame) {
    std::string s = R"(
    <mujoco>
      <worldbody>
        <body name="probe_tri3_tail" pos="0.00 0.35 0.05">
          <freejoint/>
          <geom name="g_tri3_tail" type="sphere" size="0.02"/>
        </body>
        <body name="probe_quad4" pos="-0.40 0.00 0.05">
          <freejoint/>
          <geom name="g_quad4" type="sphere" size="0.02"/>
        </body>
        <body name="probe_sheet10_chunk1" pos="0.025 -0.50 0.05">
          <freejoint/>
          <geom name="g_sheet10_chunk1" type="sphere" size="0.02"/>
        </body>
        <body name="probe_sheet10_tail" pos="0.52 0.05 0.05">
          <freejoint/>
          <geom name="g_sheet10_tail" type="sphere" size="0.02"/>
        </body>
        <flexcomp name="tri3" type="direct" dim="2" radius="0.02" mass="1"
          point="0.00 0.00 0.05  0.05 0.00 0.05  0.00 0.35 0.05"
          element="0 1 2">
          <edge equality="true"/>
          <contact selfcollide="none"/>
        </flexcomp>
        <flexcomp name="quad4" type="grid" dim="2" count="2 2 1"
          spacing="0.05 0.05 0.05" pos="-0.40 0.00 0.05"
          radius="0.02" mass="1">
          <edge equality="true"/>
          <contact selfcollide="none"/>
        </flexcomp>
        <flexcomp name="sheet10" type="direct" dim="2" radius="0.02" mass="1"
          point="0.00  0.00 0.05  0.05  0.00 0.05
                 0.00  0.05 0.05  0.05  0.05 0.05
                 0.00 -0.45 0.05  0.05 -0.45 0.05
                 0.00 -0.50 0.05  0.05 -0.50 0.05
                 0.48  0.00 0.05  0.52  0.05 0.05"
          element="0 1 2  1 3 2  4 5 6  5 7 6  1 8 9">
          <edge equality="true"/>
          <contact selfcollide="none"/>
        </flexcomp>
    )";
    if (tilted_frame) {
      s += R"(
        <body pos="5 5 2">
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
        <body pos="-5 -5 -2">
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
      )";
    } else {
      s += R"(
        <body pos="5 0 0.05">
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
        <body pos="-5 0 0.05">
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
      )";
    }
    s += "      </worldbody>\n    </mujoco>\n";
    return s;
  };

  char error[1024];
  std::string xml_aligned = make_xml(false);
  std::string xml_tilted = make_xml(true);
  MjModelPtr ma =
      LoadModelFromString(xml_aligned.c_str(), error, sizeof(error));
  ASSERT_THAT(ma.get(), NotNull()) << error;
  MjModelPtr mt = LoadModelFromString(xml_tilted.c_str(), error, sizeof(error));
  ASSERT_THAT(mt.get(), NotNull()) << error;
  ASSERT_EQ(ma->nflex, 3);
  EXPECT_EQ(ma->flex_vertnum[0], 3);
  EXPECT_EQ(ma->flex_vertnum[1], 4);
  EXPECT_EQ(ma->flex_vertnum[2], 10);

  MjDataPtr da = MakeData(ma);
  MjDataPtr dt = MakeData(mt);
  mj_forward(ma.get(), da.get());
  mj_forward(mt.get(), dt.get());

  EXPECT_GT(da->ncon, 0);
  ASSERT_EQ(da->ncon, dt->ncon);

  int g_tri3_tail = mj_name2id(ma.get(), mjOBJ_GEOM, "g_tri3_tail");
  int g_quad4 = mj_name2id(ma.get(), mjOBJ_GEOM, "g_quad4");
  int g_chunk1 = mj_name2id(ma.get(), mjOBJ_GEOM, "g_sheet10_chunk1");
  int g_tail = mj_name2id(ma.get(), mjOBJ_GEOM, "g_sheet10_tail");
  int hit_tri3_tail = 0, hit_quad4 = 0, hit_chunk1 = 0, hit_tail = 0;

  for (int i = 0; i < da->ncon; ++i) {
    EXPECT_EQ(da->contact[i].geom[0], dt->contact[i].geom[0]);
    EXPECT_EQ(da->contact[i].geom[1], dt->contact[i].geom[1]);
    EXPECT_EQ(da->contact[i].flex[0], dt->contact[i].flex[0]);
    EXPECT_EQ(da->contact[i].flex[1], dt->contact[i].flex[1]);
    EXPECT_EQ(da->contact[i].elem[0], dt->contact[i].elem[0]);
    EXPECT_EQ(da->contact[i].elem[1], dt->contact[i].elem[1]);
    EXPECT_EQ(da->contact[i].dist, dt->contact[i].dist);
    int g = da->contact[i].geom[0] >= 0 ? da->contact[i].geom[0]
                                        : da->contact[i].geom[1];
    int f = da->contact[i].flex[0] >= 0 ? da->contact[i].flex[0]
                                        : da->contact[i].flex[1];
    if (g == g_tri3_tail && f == 0) hit_tri3_tail++;
    if (g == g_quad4 && f == 1) hit_quad4++;
    if (g == g_chunk1 && f == 2) hit_chunk1++;
    if (g == g_tail && f == 2) hit_tail++;
  }
  EXPECT_GT(hit_tri3_tail, 0);
  EXPECT_GT(hit_quad4, 0);
  EXPECT_GT(hit_chunk1, 0);
  EXPECT_GT(hit_tail, 0);
}

TEST_F(MjCollisionTest, Flex2DCapsuleShaftEdgeAndCoplanar) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <body name="cap" pos="0.5 -0.03 0">
        <freejoint/>
        <geom name="cap" type="capsule" size="0.05 0.5"/>
      </body>
      <flexcomp name="tri" type="direct" dim="2" radius="0"
                point="0 0 0  1 0 0  0 1 0" element="0 1 2">
        <edge equality="true"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  const mjtNum tol = MjTol(1e-10, 1e-5);

  // 1. Shaft pressing against edge (t1, t2) with both end caps far outside.
  mj_forward(m.get(), d.get());
  ASSERT_EQ(d->ncon, 1);
  EXPECT_NEAR(d->contact[0].dist, -0.02, tol);
  EXPECT_NEAR(d->contact[0].frame[0], 0, tol);
  EXPECT_NEAR(d->contact[0].frame[1], 1, tol);
  EXPECT_NEAR(d->contact[0].frame[2], 0, tol);
  EXPECT_NEAR(d->contact[0].pos[0], 0.5, tol);
  EXPECT_NEAR(d->contact[0].pos[1], 0.01, tol);
  EXPECT_NEAR(d->contact[0].pos[2], 0, tol);

  // 2. Coplanar capsule lying flat in the triangle plane emits two -z contacts.
  m->geom_size[1] = 0.1;
  mjtNum qpos_coplanar[7] = {
      0.25, 0.25, 0, mju_cos(mjPI / 4), 0, mju_sin(mjPI / 4), 0};
  mju_copy(d->qpos, qpos_coplanar, 7);
  mj_forward(m.get(), d.get());
  ASSERT_EQ(d->ncon, 2);
  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(d->contact[i].dist, -0.05, tol);
    EXPECT_NEAR(d->contact[i].frame[0], 0, tol);
    EXPECT_NEAR(d->contact[i].frame[1], 0, tol);
    EXPECT_NEAR(d->contact[i].frame[2], -1, tol);
  }

  // 3. Tilted capsule with one end slightly below and one above keeps both.
  const mjtNum angle = mju_atan2(0.2, 0.025);  // axis tilted mostly horizontal
  mjtNum qpos_tilted[7] = {
      0.3, 0.3, 0.0075, mju_cos(angle / 2), 0, mju_sin(angle / 2), 0};
  mju_copy(d->qpos, qpos_tilted, 7);
  mj_forward(m.get(), d.get());
  ASSERT_EQ(d->ncon, 2);
  for (int i = 0; i < 2; ++i) {
    EXPECT_LT(d->contact[i].frame[2], -0.99);
  }

  // 4. Far from the origin: precision is preserved relative to t1.
  m->geom_size[1] = 0.5;
  const mjtNum shift[3] = {1000, -1000, 500};
  const mjtNum qpos_far[7] = {static_cast<mjtNum>(shift[0] + 0.5),
                              static_cast<mjtNum>(shift[1] - 0.03),
                              shift[2],
                              1,
                              0,
                              0,
                              0};
  mju_copy(d->qpos, qpos_far, 7);
  for (int v = 0; v < 3; ++v) {
    for (int j = 0; j < 3; ++j) {
      d->qpos[7 + 3 * v + j] = shift[j];
    }
  }
  mj_forward(m.get(), d.get());
  ASSERT_EQ(d->ncon, 1);
  const mjtNum far_tol = MjTol(1e-10, 1e-4);
  EXPECT_NEAR(d->contact[0].dist, -0.02, far_tol);
  EXPECT_NEAR(d->contact[0].frame[0], 0, far_tol);
  EXPECT_NEAR(d->contact[0].frame[1], 1, far_tol);
  EXPECT_NEAR(d->contact[0].frame[2], 0, far_tol);

  // 5. Coplanar capsule inside a rotated triangle: rounding must not let a
  // boundary edge axis replace the two end caps. Odd rotations are also far
  // from the origin, where single precision rounds the geometry itself out of
  // the plane (hence the loose tolerance).
  m->geom_size[1] = 0.1;
  const mjtNum point[3][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  const mjtNum center[3] = {0.25, 0.25, 0};
  const mjtNum flat[4] = {mju_cos(mjPI / 4), 0, mju_sin(mjPI / 4), 0};
  const mjtNum zaxis[3] = {0, 0, 1};
  const mjtNum offset[3] = {100, -100, 50};
  for (int i = 0; i < 200; ++i) {
    mjtNum quat[4] = {mju_sin(1.3 * i + 0.1), mju_sin(2.1 * i + 0.7),
                      mju_sin(3.7 * i + 1.9), mju_sin(5.3 * i + 2.3)};
    mju_normalize4(quat);
    mju_rotVecQuat(d->qpos, center, quat);
    mju_mulQuat(d->qpos + 3, quat, flat);
    for (int v = 0; v < 3; ++v) {
      mjtNum rotated[3];
      mju_rotVecQuat(rotated, point[v], quat);
      mju_sub3(d->qpos + 7 + 3 * v, rotated, point[v]);
    }
    if (i % 2) {
      mju_addTo3(d->qpos, offset);
      for (int v = 0; v < 3; ++v) {
        mju_addTo3(d->qpos + 7 + 3 * v, offset);
      }
    }
    mjtNum normal[3];
    mju_rotVecQuat(normal, zaxis, quat);
    mj_forward(m.get(), d.get());
    const mjtNum rot_tol = (i % 2) ? MjTol(1e-13, 1e-2) : MjTol(4e-15, 2e-6);
    ASSERT_EQ(d->ncon, 2) << "rotation " << i;
    for (int c = 0; c < 2; ++c) {
      EXPECT_NEAR(d->contact[c].dist, -0.05, rot_tol) << "rotation " << i;
      EXPECT_NEAR(mju_abs(mju_dot3(d->contact[c].frame, normal)), 1, rot_tol)
          << "rotation " << i;
    }
    EXPECT_NEAR(mju_dot3(d->contact[0].frame, d->contact[1].frame), 1, rot_tol)
        << "rotation " << i;
    EXPECT_NEAR(mju_dist3(d->contact[0].pos, d->contact[1].pos), 0.2, rot_tol)
        << "rotation " << i;
  }
}

TEST_F(MjCollisionTest, Flex2DCapsuleCoplanarInteriorTriangle) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <body name="cap" pos="-0.3 0 0.5">
        <freejoint/>
        <geom name="cap" type="capsule" size="0.05 1"/>
      </body>
      <flexcomp name="cloth" type="direct" dim="2" radius="0"
                point="0 0 0  0 0 1  -1 0 0.5  1 0 0.5  -1 0 1.5  -1 0 -0.5"
                element="0 1 2  1 0 3  2 1 4  0 2 5">
        <edge equality="true"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  const mjtNum tol = MjTol(1e-10, 1e-5);

  // Capsule lying in the cloth plane with both end caps outside the cloth: its
  // axis crosses two interior edges of the center triangle, which has three
  // neighbours. Each of the three crossed triangles must emit one contact, at
  // the first edge (in element order) that the axis crosses.
  mj_forward(m.get(), d.get());
  ASSERT_EQ(d->ncon, 3);
  std::vector<mjtNum> z;
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(d->contact[i].dist, -0.05, tol);
    EXPECT_NEAR(d->contact[i].frame[0], 0, tol);
    EXPECT_NEAR(d->contact[i].frame[1], 1, tol);
    EXPECT_NEAR(d->contact[i].frame[2], 0, tol);
    EXPECT_NEAR(d->contact[i].pos[0], -0.3, tol);
    EXPECT_NEAR(d->contact[i].pos[1], 0.025, tol);
    z.push_back(d->contact[i].pos[2]);
  }
  std::sort(z.begin(), z.end());
  EXPECT_NEAR(z[0], -0.15, tol);
  EXPECT_NEAR(z[1], 0.85, tol);
  EXPECT_NEAR(z[2], 1.15, tol);
}

TEST_F(MjCollisionTest, Flex2DCapsuleInteriorEdgeAndVFold) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <body name="cap" pos="-0.2 0.5 0.08">
        <freejoint/>
        <geom name="cap" type="capsule" size="0.05 0.1"/>
      </body>
      <flexcomp name="cloth" type="direct" dim="2" radius="0"
                point="0 0 0  0 1 0  -1 0.5 0  1 0.5 0"
                element="0 1 2  1 0 3">
        <edge equality="true"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  const mjtNum tol = MjTol(1e-10, 1e-5);

  // 1. Piercing flat two-triangle cloth near shared interior edge: exit along
  // -z only, with no opposing lateral contact from the adjacent triangle.
  mj_forward(m.get(), d.get());
  ASSERT_EQ(d->ncon, 1);
  EXPECT_NEAR(d->contact[0].dist, -0.07, tol);
  EXPECT_NEAR(d->contact[0].frame[0], 0, tol);
  EXPECT_NEAR(d->contact[0].frame[1], 0, tol);
  EXPECT_NEAR(d->contact[0].frame[2], -1, tol);

  // 2. Convex V-fold crease: drop the two outer vertices to z = -0.5 and press
  // a horizontal capsule shaft across the shared ridge from above.
  d->qpos[7 + 3 * 2 + 2] = -0.5;
  d->qpos[7 + 3 * 3 + 2] = -0.5;
  m->geom_size[1] = 0.5;
  mjtNum qpos_ridge[7] = {0, 0.5, 0.03, mju_cos(mjPI / 4), 0, mju_sin(mjPI / 4),
                          0};
  mju_copy(d->qpos, qpos_ridge, 7);
  mj_forward(m.get(), d.get());
  ASSERT_GE(d->ncon, 1);
  mjtNum min_dist = mjMAXVAL;
  for (int i = 0; i < d->ncon; ++i) {
    if (d->contact[i].dist < min_dist) {
      min_dist = d->contact[i].dist;
    }
    EXPECT_LT(d->contact[i].frame[2], -0.99);
  }
  EXPECT_NEAR(min_dist, -0.02, tol);
}

TEST_F(MjCollisionTest, Flex2DCapsuleDegenerateNoNaN) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <body name="cap" pos="0.5 0.03 0">
        <freejoint/>
        <geom name="cap" type="capsule" size="0.05 0.1"/>
      </body>
      <flexcomp name="tri" type="direct" dim="2" radius="0"
                point="0 0 0  1 0 0  0 1 0" element="0 1 2">
        <edge equality="true"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  // Zero-length capsule and collapsed triangle (v1 == v0) must produce a single
  // finite contact without NaN (the two coincident end caps count once).
  m->geom_size[1] = 0;
  d->qpos[7 + 3 * 1 + 0] = -1;  // collapse vertex 1 onto vertex 0
  d->qpos[7 + 3 * 2 + 0] = 1;   // move vertex 2 to (1, 0, 0)
  d->qpos[7 + 3 * 2 + 1] = -1;
  mj_forward(m.get(), d.get());
  ASSERT_EQ(d->ncon, 1);
  for (int i = 0; i < d->ncon; ++i) {
    EXPECT_TRUE(std::isfinite(d->contact[i].dist));
    for (int j = 0; j < 3; ++j) {
      EXPECT_TRUE(std::isfinite(d->contact[i].pos[j]));
      EXPECT_TRUE(std::isfinite(d->contact[i].frame[j]));
    }
  }
  EXPECT_NEAR(d->contact[0].dist, -0.02, MjTol(1e-10, 1e-5));
}

}  // namespace
}  // namespace mujoco
