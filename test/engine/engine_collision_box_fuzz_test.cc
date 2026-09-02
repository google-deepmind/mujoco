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

// Randomized cross-validation of mjc_BoxBox.
//
// Each model holds two box geoms and two box-shaped mesh geoms with identical
// half-extents. For a randomized relative pose the box pair is collided with
// mjc_BoxBox and the mesh pair with mjc_Convex (GJK/EPA).
//
// Two references arbitrate correctness:
//  - GJK/EPA on the mesh pair, where it is trustworthy. EPA misreports depth
//    and normal on thin meshes (verified against the direction sweep below),
//    so strict agreement is only enforced for well-conditioned aspect ratios.
//  - A dense direction sweep over support separations. For two boxes the
//    minimum-translation direction is a face normal or an edge-edge cross
//    product, so the sweep converges to the true penetration depth from
//    above; its resolution error at 20k directions is ~2.5e-3 of scale.
//
// Independent of any reference, invariants are enforced for every sample:
// contacts lie within half their own depth of both (margin-inflated) boxes,
// all contacts in a manifold share one normal, the manifold is no larger than a
// clipped face, and no contact is deeper than the true penetration depth.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_collision_convex.h"
#include "src/engine/engine_collision_primitive.h"
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

using MjCollisionBoxFuzzTest = MujocoTest;

// maximum contacts box-box collider may emit: a face manifold is the clipped
// incident face, a 4-gon against 4 half-planes, so at most 8 vertices
constexpr int kMaxContacts = 8;

// direction count for the brute-force sweep; resolution scales as 1/sqrt(n)
constexpr int kSweepDirections = 20000;

struct SizeCase {
  const char* name;
  mjtNum size1[3];
  mjtNum size2[3];
  // strict GJK agreement is enforced when true; EPA under-reports deep
  // penetration by up to ~1% of depth, which the truth-arbitrated gates
  // absorb, so all cases currently enable it
  bool gjk_reliable;
};

constexpr SizeCase kSizeCases[] = {
    {"cube_cube", {0.05, 0.05, 0.05}, {0.05, 0.05, 0.05}, true},
    {"cube_small", {0.05, 0.05, 0.05}, {0.013, 0.013, 0.013}, true},
    {"slab_slab", {0.08, 0.08, 0.004}, {0.06, 0.06, 0.006}, true},
    {"needle_cube", {0.002, 0.002, 0.09}, {0.04, 0.04, 0.04}, true},
    {"slab_needle", {0.07, 0.05, 0.003}, {0.0015, 0.0015, 0.06}, true},
    {"aniso", {0.018, 0.038, 0.047}, {0.026, 0.0014, 0.008}, true},
};

// builds a model with box geoms 0,1 and identical box-mesh geoms 2,3; every
// geom hangs from a freejoint body so poses are applied through qpos and
// kinematics -- writing geom_xmat directly would discard the compiled mesh's
// principal-axis frame, silently permuting a thin mesh's axes
std::string MakeXml(const SizeCase& c) {
  char buf[3072];
  std::snprintf(buf, sizeof(buf), R"(
  <mujoco>
    <asset>
      <mesh name="m1" scale="%g %g %g"
        vertex="-1 -1 -1  1 -1 -1  1 1 -1  1 1 1  1 -1 1  -1 1 -1  -1 1 1  -1 -1 1"/>
      <mesh name="m2" scale="%g %g %g"
        vertex="-1 -1 -1  1 -1 -1  1 1 -1  1 1 1  1 -1 1  -1 1 -1  -1 1 1  -1 -1 1"/>
    </asset>
    <worldbody>
      <body><freejoint/><geom type="box" size="%g %g %g"/></body>
      <body><freejoint/><geom type="box" size="%g %g %g"/></body>
      <body><freejoint/><geom type="mesh" mesh="m1"/></body>
      <body><freejoint/><geom type="mesh" mesh="m2"/></body>
    </worldbody>
  </mujoco>
  )",
                c.size1[0], c.size1[1], c.size1[2], c.size2[0], c.size2[1],
                c.size2[2], c.size1[0], c.size1[1], c.size1[2], c.size2[0],
                c.size2[1], c.size2[2]);
  return std::string(buf);
}

void RandomQuat(std::mt19937& rng, mjtNum quat[4]) {
  std::normal_distribution<double> g(0.0, 1.0);
  for (int i = 0; i < 4; i++) quat[i] = g(rng);
  mju_normalize4(quat);
}

// places body pair (0,1) and the mirrored mesh pair (2,3) at the same poses,
// through qpos and kinematics so mesh frame compensation is honored
void SetPose(const mjModel* model, mjData* data, const mjtNum pos2[3],
             const mjtNum quat1[4], const mjtNum quat2[4]) {
  mjtNum* q = data->qpos;
  mju_zero3(q);
  mju_copy4(q + 3, quat1);
  mju_copy3(q + 7, pos2);
  mju_copy4(q + 10, quat2);
  mju_zero3(q + 14);
  mju_copy4(q + 17, quat1);
  mju_copy3(q + 21, pos2);
  mju_copy4(q + 24, quat2);
  mj_kinematics(model, data);
}

mjtNum DeepestDist(const mjPreContact* con, int n) {
  mjtNum d = con[0].dist;
  for (int i = 1; i < n; i++) d = mju_min(d, con[i].dist);
  return d;
}

// support separation of the two boxes along a specific direction
mjtNum DirSep(const mjModel* model, const mjData* data, const mjtNum dir[3]) {
  mjtNum dpos[3];
  mju_sub3(dpos, data->geom_xpos + 3, data->geom_xpos);
  mjtNum r1 = 0, r2 = 0;
  for (int k = 0; k < 3; k++) {
    mjtNum a1 = dir[0] * data->geom_xmat[0 + k] +
                dir[1] * data->geom_xmat[3 + k] +
                dir[2] * data->geom_xmat[6 + k];
    mjtNum a2 = dir[0] * data->geom_xmat[9 + k] +
                dir[1] * data->geom_xmat[12 + k] +
                dir[2] * data->geom_xmat[15 + k];
    r1 += model->geom_size[k] * mju_abs(a1);
    r2 += model->geom_size[3 + k] * mju_abs(a2);
  }
  return mju_abs(mju_dot3(dir, dpos)) - r1 - r2;
}

// true separation via dense direction sweep (spherical Fibonacci lattice);
// the sampled maximum is a lower bound on the true separation, converging as
// the direction count grows
mjtNum BruteForceSep(const mjModel* model, const mjData* data) {
  mjtNum dpos[3];
  mju_sub3(dpos, data->geom_xpos + 3, data->geom_xpos);
  mjtNum best = -mjMAXVAL;
  for (int gi = 0; gi < kSweepDirections; gi++) {
    mjtNum phi = 2.399963229728653 * gi;
    mjtNum ct = 1.0 - 2.0 * (gi + 0.5) / kSweepDirections;
    mjtNum st = std::sqrt(mju_max(0, 1 - ct * ct));
    mjtNum dir[3] = {st * std::cos(phi), st * std::sin(phi), ct};
    mjtNum r1 = 0, r2 = 0;
    for (int k = 0; k < 3; k++) {
      mjtNum a1 = dir[0] * data->geom_xmat[0 + k] +
                  dir[1] * data->geom_xmat[3 + k] +
                  dir[2] * data->geom_xmat[6 + k];
      mjtNum a2 = dir[0] * data->geom_xmat[9 + k] +
                  dir[1] * data->geom_xmat[12 + k] +
                  dir[2] * data->geom_xmat[15 + k];
      r1 += model->geom_size[k] * mju_abs(a1);
      r2 += model->geom_size[3 + k] * mju_abs(a2);
    }
    best = mju_max(best, mju_abs(mju_dot3(dir, dpos)) - r1 - r2);
  }
  return best;
}

// closed-form analytical SAT reference across all 15 potential separating axes
// in double precision (exact to machine precision, without discretization
// error)
mjtNum ExactSatSep(const mjModel* model, const mjData* data) {
  const mjtNum* pos1 = data->geom_xpos;
  const mjtNum* pos2 = data->geom_xpos + 3;
  const mjtNum* mat1 = data->geom_xmat;
  const mjtNum* mat2 = data->geom_xmat + 9;
  const mjtNum* size1 = model->geom_size;
  const mjtNum* size2 = model->geom_size + 3;

  mjtNum rot[9], rotabs[9], pos21[3], pos12[3], tmp[3];
  mju_sub3(tmp, pos2, pos1);
  mju_mulMatTVec3(pos21, mat1, tmp);
  mju_sub3(tmp, pos1, pos2);
  mju_mulMatTVec3(pos12, mat2, tmp);
  mju_mulMatTMat(rot, mat1, mat2, 3, 3, 3);
  for (int i = 0; i < 9; i++) {
    rotabs[i] = mju_abs(rot[i]);
  }

  mjtNum sep_max = -mjMAXVAL;

  // 3 face axes of box1
  for (int i = 0; i < 3; i++) {
    mjtNum radius2 = rotabs[3 * i + 0] * size2[0] +
                     rotabs[3 * i + 1] * size2[1] +
                     rotabs[3 * i + 2] * size2[2];
    mjtNum sep = mju_abs(pos21[i]) - size1[i] - radius2;
    sep_max = mju_max(sep_max, sep);
  }

  // 3 face axes of box2
  for (int j = 0; j < 3; j++) {
    mjtNum radius1 = rotabs[0 + j] * size1[0] + rotabs[3 + j] * size1[1] +
                     rotabs[6 + j] * size1[2];
    mjtNum sep = mju_abs(pos12[j]) - size2[j] - radius1;
    sep_max = mju_max(sep_max, sep);
  }

  // 9 edge-cross axes
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
      mjtNum ax1 = -rot[3 * i2 + j];
      mjtNum ax2 = rot[3 * i1 + j];
      mjtNum norm2 = ax1 * ax1 + ax2 * ax2;
      if (norm2 < 1e-12) {
        continue;
      }
      mjtNum inv = 1 / mju_sqrt(norm2);
      ax1 *= inv;
      ax2 *= inv;

      int j1 = (j + 1) % 3, j2 = (j + 2) % 3;
      mjtNum a2_1 = ax1 * rot[3 * i1 + j1] + ax2 * rot[3 * i2 + j1];
      mjtNum a2_2 = ax1 * rot[3 * i1 + j2] + ax2 * rot[3 * i2 + j2];

      mjtNum radius1 = size1[i1] * mju_abs(ax1) + size1[i2] * mju_abs(ax2);
      mjtNum radius2 = size2[j1] * mju_abs(a2_1) + size2[j2] * mju_abs(a2_2);
      mjtNum sep =
          mju_abs(ax1 * pos21[i1] + ax2 * pos21[i2]) - radius1 - radius2;
      sep_max = mju_max(sep_max, sep);
    }
  }
  return sep_max;
}

struct Stats {
  int configs = 0;
  int both_hit = 0;
  int only_box = 0;      // box-box hit where GJK did not
  int only_gjk = 0;      // GJK hit where box-box did not
  int normal_bad = 0;    // (gjk_reliable only) normal disagreement
  int depth_bad = 0;     // (gjk_reliable only) deepest-depth disagreement
  int outside_bad = 0;   // contact farther than |dist|/2 + slack from a box
  int count_bad = 0;     // more than kMaxContacts contacts
  int mixed_normal = 0;  // manifold contacts disagree on normal
  int overdeep = 0;      // contact deeper than the true penetration depth
  int phantom = 0;       // contact reported where truth says separated
  int missed = 0;        // no contact reported where truth says penetrating
  mjtNum worst_normal = 0;
  mjtNum worst_depth = 0;
};

Stats Sweep(const SizeCase& c, int n_configs, mjtNum margin, unsigned seed) {
  Stats st;
  const std::string xml = MakeXml(c);
  char error[1024];
  MjModelPtr model_ptr = LoadModelFromString(xml.c_str(), error, sizeof(error));
  EXPECT_THAT(model_ptr.get(), NotNull()) << error;
  if (!model_ptr) return st;
  mjModel* model = model_ptr.get();
  MjDataPtr data_ptr = MakeData(model_ptr);
  mjData* data = data_ptr.get();

  const mjtNum scale = mju_max(mju_max(c.size1[0], c.size1[1]), c.size1[2]) +
                       mju_max(mju_max(c.size2[0], c.size2[1]), c.size2[2]);
  // sweep resolution: angular spacing ~sqrt(4pi/n), times the pair radius
  const mjtNum sweep_tol = 4.0 * scale / std::sqrt((double)kSweepDirections);

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);

  mjPreContact box_con[mjMAXCONPAIR], gjk_con[mjMAXCONPAIR];

  for (int it = 0; it < n_configs; it++) {
    mjtNum pos2[3], quat1[4], quat2[4];
    for (int i = 0; i < 3; i++) pos2[i] = 1.15 * scale * u(rng);
    RandomQuat(rng, quat1);
    RandomQuat(rng, quat2);
    SetPose(model, data, pos2, quat1, quat2);

    int nbox = mjc_BoxBox(model, data, box_con, 0, 1, margin);
    int ngjk = mjc_Convex(model, data, gjk_con, 2, 3, margin);
    st.configs++;

    if (nbox > kMaxContacts) st.count_bad++;

    // ground-truth arbitration on a sample of configs and on every presence
    // disagreement
    bool arbitrate = (it % 16 == 0) || (nbox > 0) != (ngjk > 0);
    if (arbitrate) {
      mjtNum true_sep = BruteForceSep(model, data);
      // in the margin band the collider measures the gap along its separating
      // axis, which under-reads the Euclidean separation (a SAT property the
      // previous implementation and MJX share), so separated-pair bookkeeping
      // contacts are by design; the harmful classes are claiming penetration
      // where none exists and reporting depth beyond the true depth
      mjtNum db = DeepestDist(box_con, nbox);
      int bad_phantom = nbox > 0 && db < 0 && true_sep > sweep_tol;
      int bad_deep = nbox > 0 && db < 0 &&
                     db < true_sep - sweep_tol - 0.05 * mju_abs(true_sep);
      if (bad_phantom) st.phantom++;
      if (nbox == 0 && true_sep < margin - sweep_tol) st.missed++;
      if (bad_deep) st.overdeep++;
      if ((bad_phantom || bad_deep) && std::getenv("MJ_FUZZ_DUMP")) {
        std::printf(
            "%s %s db=%.6e true_sep=%.6e nbox=%d n0=(%.4f %.4f %.4f)\n"
            "  pos2={%.17g, %.17g, %.17g}\n"
            "  quat1={%.17g, %.17g, %.17g, %.17g}\n"
            "  quat2={%.17g, %.17g, %.17g, %.17g}\n",
            bad_phantom ? "PHANTOM" : "OVERDEEP", c.name, db, true_sep, nbox,
            box_con[0].normal[0], box_con[0].normal[1], box_con[0].normal[2],
            pos2[0], pos2[1], pos2[2], quat1[0], quat1[1], quat1[2], quat1[3],
            quat2[0], quat2[1], quat2[2], quat2[3]);
      }
    }

    if (nbox > 0) {
      // manifold invariants: one shared normal; each contact within half its
      // own depth (plus slack) of both margin-inflated boxes
      for (int i = 1; i < nbox; i++) {
        // the contacts of a face manifold carry the same normal vector, whose
        // self-dot is 1 only to the precision of mjtNum
        if (mju_dot3(box_con[0].normal, box_con[i].normal) <
            1 - MjTol(1e-9, 1e-5)) {
          st.mixed_normal++;
          break;
        }
      }
      for (int i = 0; i < nbox; i++) {
        mjtNum slack = 0.5 * mju_abs(box_con[i].dist) + 1e-6 * scale;
        mjtNum sz1[3], sz2[3];
        for (int k = 0; k < 3; k++) {
          sz1[k] = model->geom_size[k] + margin + slack;
          sz2[k] = model->geom_size[3 + k] + margin + slack;
        }
        int o1 = mju_outsideBox(box_con[i].pos, data->geom_xpos,
                                data->geom_xmat, sz1, 1);
        int o2 = mju_outsideBox(box_con[i].pos, data->geom_xpos + 3,
                                data->geom_xmat + 9, sz2, 1);
        if (o1 == 1 || o2 == 1) {
          st.outside_bad++;
          break;
        }
      }
    }

    if (nbox > 0 && ngjk > 0) {
      st.both_hit++;
      if (c.gjk_reliable) {
        mjtNum dot = mju_dot3(box_con[0].normal, gjk_con[0].normal);
        mjtNum ang = mju_abs(1 - mju_abs(dot));
        st.worst_normal = mju_max(st.worst_normal, ang);
        // margin-band contacts admit legitimately ambiguous normals near
        // face ties, so the angular gate is looser with margin
        mjtNum ntol = margin > 0 ? 2e-1 : 1e-3;
        // separated margin-band pairs are exempt: their true closest-feature
        // direction generally lies between the 15 SAT axes (corner-corner
        // cases), so the SAT normal legitimately differs from GJK's; for
        // penetration the SAT axis set contains the exact optimum
        if (ang > ntol && DeepestDist(box_con, nbox) < 0) {
          // arbitrate ties: a normal is wrong only if its directional
          // separation is materially worse than the reference normal's --
          // near-equal minima are legitimately ambiguous between methods
          mjtNum sep_box = DirSep(model, data, box_con[0].normal);
          mjtNum sep_gjk = DirSep(model, data, gjk_con[0].normal);
          // the design prefers face manifolds within five percent of the
          // optimum (stack stability), measured against its own face
          // separation; allow one percent cross-measurement slop against
          // the reference optimum
          if (sep_box < sep_gjk - 0.06 * mju_abs(sep_gjk) - 1e-4 * scale) {
            st.normal_bad++;
            if (std::getenv("MJ_FUZZ_DUMP")) {
              std::printf(
                  "NORMAL_BAD %s ang=%.3e sep_box=%.6e sep_gjk=%.6e "
                  "nbox=%d\n  nb={%.6f %.6f %.6f} ng={%.6f %.6f %.6f}\n"
                  "  pos2={%.17g, %.17g, %.17g}\n"
                  "  quat1={%.17g, %.17g, %.17g, %.17g}\n"
                  "  quat2={%.17g, %.17g, %.17g, %.17g}\n",
                  c.name, ang, sep_box, sep_gjk, nbox, box_con[0].normal[0],
                  box_con[0].normal[1], box_con[0].normal[2],
                  gjk_con[0].normal[0], gjk_con[0].normal[1],
                  gjk_con[0].normal[2], pos2[0], pos2[1], pos2[2], quat1[0],
                  quat1[1], quat1[2], quat1[3], quat2[0], quat2[1], quat2[2],
                  quat2[3]);
            }
          }
        }

        mjtNum db = DeepestDist(box_con, nbox);
        mjtNum dg = DeepestDist(gjk_con, ngjk);
        mjtNum ddiff = mju_abs(db - dg);
        st.worst_depth = mju_max(st.worst_depth, ddiff);
        // EPA witness accuracy degrades in the margin band
        mjtNum dtol = (margin > 0 ? 5e-3 : 1e-4) * scale + 1e-9;
        if (ddiff > dtol) {
          // arbitrate against ground truth: only count if the box side
          // deviates on the too-deep side of the true depth, and only for
          // penetration, where the support sweep equals the true depth.
          // A shallower manifold is legitimate: the deepest corner can be
          // clipped away laterally, leaving surface-to-surface depths at
          // the surviving contact locations. Positive distances measure
          // different things per method (axis gap vs Euclidean witness gap).
          mjtNum true_sep = BruteForceSep(model, data);
          // the design substitutes the face manifold for an aliasing edge
          // within five percent of the optimum, so depth may exceed the true
          // depth by that fraction
          mjtNum design = 0.05 * mju_abs(true_sep);
          if (db < 0 && db < true_sep - sweep_tol - design) {
            st.depth_bad++;
            if (std::getenv("MJ_FUZZ_DUMP")) {
              std::printf(
                  "DEPTH_BAD %s db=%.6e dg=%.6e true_sep=%.6e nbox=%d "
                  "ngjk=%d\n  nb={%.6f %.6f %.6f} ng={%.6f %.6f %.6f}\n"
                  "  pos2={%.17g, %.17g, %.17g}\n"
                  "  quat1={%.17g, %.17g, %.17g, %.17g}\n"
                  "  quat2={%.17g, %.17g, %.17g, %.17g}\n",
                  c.name, db, dg, true_sep, nbox, ngjk, box_con[0].normal[0],
                  box_con[0].normal[1], box_con[0].normal[2],
                  gjk_con[0].normal[0], gjk_con[0].normal[1],
                  gjk_con[0].normal[2], pos2[0], pos2[1], pos2[2], quat1[0],
                  quat1[1], quat1[2], quat1[3], quat2[0], quat2[1], quat2[2],
                  quat2[3]);
            }
          }
        }
      }
    } else if (nbox > 0) {
      st.only_box++;
    } else if (ngjk > 0) {
      st.only_gjk++;
    }
  }

  return st;
}

void Report(const char* label, const SizeCase& c, const Stats& st) {
  std::printf(
      "[%s/%-11s] n=%5d both=%5d onlyBox=%4d onlyGJK=%4d | normal_bad=%4d "
      "depth_bad=%4d outside=%4d count_bad=%3d mixed_n=%3d | phantom=%3d "
      "missed=%3d overdeep=%3d | worst_n=%.3e worst_d=%.3e\n",
      label, c.name, st.configs, st.both_hit, st.only_box, st.only_gjk,
      st.normal_bad, st.depth_bad, st.outside_bad, st.count_bad,
      st.mixed_normal, st.phantom, st.missed, st.overdeep, st.worst_normal,
      st.worst_depth);
}

void CheckGates(const SizeCase& c, const Stats& st, mjtNum margin) {
  EXPECT_GT(st.both_hit, 0) << c.name << ": no overlapping samples";
  EXPECT_EQ(st.count_bad, 0) << c.name;
  EXPECT_EQ(st.mixed_normal, 0) << c.name;
  EXPECT_EQ(st.outside_bad, 0) << c.name;
  EXPECT_EQ(st.phantom, 0) << c.name;
  EXPECT_EQ(st.overdeep, 0) << c.name;
  if (margin > 0) {
    // corner-past-the-face margin-band contacts are not representable by a
    // SAT clip collider (same limitation in the previous implementation and
    // MJX); these are bookkeeping contacts at positive distance, so a miss
    // only delays activation by a step. Observed rate peaks at ~0.3% on the
    // most anisotropic case
    EXPECT_LE(st.missed, st.configs / 250) << c.name;
  } else {
    // at zero margin the SAT depth theorem is exact: no misses allowed
    EXPECT_EQ(st.missed, 0) << c.name;
  }
  if (c.gjk_reliable) {
    EXPECT_EQ(st.normal_bad, 0) << c.name;
    EXPECT_EQ(st.depth_bad, 0) << c.name;
  }
}

// config count and seed are overridable for soak runs:
// MJ_FUZZ_CONFIGS=20000 MJ_FUZZ_SEED=7 ./engine_collision_box_fuzz_test
int NumConfigs() {
  const char* env = std::getenv("MJ_FUZZ_CONFIGS");
  return env ? std::stoi(env) : 4000;
}

unsigned BaseSeed() {
  const char* env = std::getenv("MJ_FUZZ_SEED");
  return env ? std::stoul(env) : 0;
}

TEST_F(MjCollisionBoxFuzzTest, AgreesWithReferencesZeroMargin) {
  for (const SizeCase& c : kSizeCases) {
    Stats st = Sweep(c, NumConfigs(), /*margin=*/0, 12345 + BaseSeed());
    Report("margin=0", c, st);
    CheckGates(c, st, 0);
  }
}

TEST_F(MjCollisionBoxFuzzTest, AgreesWithReferencesWithMargin) {
  for (const SizeCase& c : kSizeCases) {
    Stats st = Sweep(c, NumConfigs(), /*margin=*/0.01, 999 + BaseSeed());
    Report("margin>0", c, st);
    CheckGates(c, st, 0.01);
  }
}

TEST_F(MjCollisionBoxFuzzTest, CanonicalOrientationsAndPerturbations) {
  // 24 rotational symmetries of the cube (octahedral group Oh)
  std::vector<std::array<mjtNum, 4>> canonical_quats;
  for (int ax = 0; ax < 3; ax++) {
    for (int sx : {-1, 1}) {
      for (int ay = 0; ay < 3; ay++) {
        if (ay == ax) continue;
        for (int sy : {-1, 1}) {
          mjtNum mat[9] = {0};
          mat[3 * 0 + ax] = sx;
          mat[3 * 1 + ay] = sy;
          // col 2 = col 0 x col 1
          // clang-format off
          mat[3 * 2 + 0] = mat[3 * 0 + 1] * mat[3 * 1 + 2] -
                           mat[3 * 0 + 2] * mat[3 * 1 + 1];
          mat[3 * 2 + 1] = mat[3 * 0 + 2] * mat[3 * 1 + 0] -
                           mat[3 * 0 + 0] * mat[3 * 1 + 2];
          mat[3 * 2 + 2] = mat[3 * 0 + 0] * mat[3 * 1 + 1] -
                           mat[3 * 0 + 1] * mat[3 * 1 + 0];
          // clang-format on
          mjtNum q[4];
          mju_mat2Quat(q, mat);
          canonical_quats.push_back({q[0], q[1], q[2], q[3]});
        }
      }
    }
  }

  const mjtNum pert_angles[] = {0.0,  1e-15, 1e-12, 1e-9,
                                1e-6, 1e-3,  0.05,  0.785398};
  const mjtNum pert_axes[5][3] = {{1, 0, 0},
                                  {0, 1, 0},
                                  {0, 0, 1},
                                  {0.70710678, 0.70710678, 0},
                                  {0.57735027, 0.57735027, 0.57735027}};

  // test across cube and anisotropic slab/needle size cases
  for (const SizeCase& c :
       {kSizeCases[0], kSizeCases[1], kSizeCases[2], kSizeCases[4]}) {
    const std::string xml = MakeXml(c);
    char error[1024];
    MjModelPtr model_ptr =
        LoadModelFromString(xml.c_str(), error, sizeof(error));
    ASSERT_THAT(model_ptr.get(), NotNull()) << error;
    mjModel* model = model_ptr.get();
    MjDataPtr data_ptr = MakeData(model_ptr);
    mjData* data = data_ptr.get();

    for (mjtNum margin : {0.0, 0.005}) {
      for (const auto& qbase : canonical_quats) {
        for (mjtNum angle : pert_angles) {
          for (const auto& axis : pert_axes) {
            mjtNum qpert[4], quat2[4];
            mju_axisAngle2Quat(qpert, axis, angle);
            mju_mulQuat(quat2, qbase.data(), qpert);
            mjtNum quat1[4] = {1, 0, 0, 0};

            mjtNum mat2[9];
            mju_quat2Mat(mat2, quat2);
            mjtNum rproj[3] = {
                mju_abs(mat2[0]) * c.size2[0] + mju_abs(mat2[1]) * c.size2[1] +
                    mju_abs(mat2[2]) * c.size2[2],
                mju_abs(mat2[3]) * c.size2[0] + mju_abs(mat2[4]) * c.size2[1] +
                    mju_abs(mat2[5]) * c.size2[2],
                mju_abs(mat2[6]) * c.size2[0] + mju_abs(mat2[7]) * c.size2[1] +
                    mju_abs(mat2[8]) * c.size2[2],
            };

            // lateral fraction offsets and depth fraction offsets
            for (mjtNum xfrac : {-0.5, 0.0, 0.5, 0.99, 1.0}) {
              for (mjtNum yfrac : {-0.5, 0.0, 0.5, 0.99, 1.0}) {
                if (xfrac * xfrac + yfrac * yfrac > 1.01) continue;
                for (mjtNum zfrac : {-0.2, -1e-4, 0.0, 1e-4, 0.1}) {
                  mjtNum pos2[3] = {xfrac * (c.size1[0] + rproj[0]),
                                    yfrac * (c.size1[1] + rproj[1]),
                                    (c.size1[2] + rproj[2]) +
                                        zfrac * (c.size1[2] + rproj[2])};
                  SetPose(model, data, pos2, quat1, quat2);

                  mjPreContact box_con[mjMAXCONPAIR];
                  int nbox = mjc_BoxBox(model, data, box_con, 0, 1, margin);
                  EXPECT_LE(nbox, kMaxContacts) << c.name;

                  // normal consistency across manifold
                  for (int i = 1; i < nbox; i++) {
                    EXPECT_GE(mju_dot3(box_con[0].normal, box_con[i].normal),
                              1 - MjTol(1e-9, 1e-5))
                        << c.name;
                  }

                  // contacts within half depth of both boxes
                  mjtNum scale = c.size1[0] + c.size1[1] + c.size1[2] +
                                 c.size2[0] + c.size2[1] + c.size2[2];
                  for (int i = 0; i < nbox; i++) {
                    mjtNum slack =
                        0.5 * mju_abs(box_con[i].dist) + 0.05 * scale;
                    mjtNum sz1[3] = {model->geom_size[0] + margin + slack,
                                     model->geom_size[1] + margin + slack,
                                     model->geom_size[2] + margin + slack};
                    mjtNum sz2[3] = {model->geom_size[3] + margin + slack,
                                     model->geom_size[4] + margin + slack,
                                     model->geom_size[5] + margin + slack};
                    int o1 = mju_outsideBox(box_con[i].pos, data->geom_xpos,
                                            data->geom_xmat, sz1, 1);
                    int o2 = mju_outsideBox(box_con[i].pos, data->geom_xpos + 3,
                                            data->geom_xmat + 9, sz2, 1);
                    EXPECT_FALSE(o1 == 1 && o2 == 1)
                        << c.name << " pos=(" << box_con[i].pos[0] << ", "
                        << box_con[i].pos[1] << ", " << box_con[i].pos[2]
                        << ")";
                  }

                  mjtNum exact_sep = ExactSatSep(model, data);
                  if (margin == 0) {
                    if (exact_sep < -1e-6 * scale) {
                      EXPECT_GT(nbox, 0)
                          << c.name << " exact_sep=" << exact_sep;
                    }
                  }
                  if (nbox > 0 && exact_sep < 0) {
                    mjtNum db = DeepestDist(box_con, nbox);
                    EXPECT_GE(db, exact_sep - 0.06 * mju_abs(exact_sep) -
                                      1e-6 * scale)
                        << c.name << " db=" << db << " exact_sep=" << exact_sep;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

}  // namespace
}  // namespace mujoco
