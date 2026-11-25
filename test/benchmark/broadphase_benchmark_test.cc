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

// Microbenchmarks isolating broadphase Sweep-and-Prune (mj_SAP), flex SAP
// self-collision, dynamic BVH refit (mj_updateDynamicBVH), and BVH midphase.

#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include <benchmark/benchmark.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "absl/base/attributes.h"
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// Deterministic linear congruential generator in [-1, 1] for reproducible
// spatial jitter.
double NextSigned(uint32_t& state) {
  state = state * 1664525u + 1013904223u;
  return (static_cast<double>(state >> 8) / 8388608.0) - 1.0;
}

// Builds a 3D cloud of nx * ny * nz free sphere bodies.
std::string BuildSphereCloudXml(int nx, int ny, int nz, bool enable_sleep,
                                bool add_floor) {
  std::ostringstream out;
  out << "<mujoco model=\"broadphase_cloud\">\n";
  out << "  <size memory=\"64M\"/>\n";
  out << "  <option timestep=\"0.002\">\n";
  if (enable_sleep) {
    out << "    <flag sleep=\"enable\"/>\n";
  }
  out << "  </option>\n";
  out << "  <worldbody>\n";
  if (add_floor) {
    out << "    <geom name=\"floor\" type=\"plane\" size=\"20 20 0.1\"/>\n";
  }

  uint32_t rng = 0xC0FFEE12u;
  const double spacing = 0.24;
  const double radius = 0.065;
  const double jitter = enable_sleep ? 0.015 : 0.045;
  for (int ix = 0; ix < nx; ++ix) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int iz = 0; iz < nz; ++iz) {
        double x = (ix - 0.5 * nx) * spacing + jitter * NextSigned(rng);
        double y = (iy - 0.5 * ny) * spacing + jitter * NextSigned(rng);
        double z = 0.25 + iz * spacing + jitter * NextSigned(rng);
        out << "    <body pos=\"" << x << " " << y << " " << z << "\"";
        if (enable_sleep && (iz < nz / 2)) {
          out << " sleep=\"init\"";
        }
        out << ">\n";
        out << "      <freejoint/>\n";
        out << "      <geom type=\"sphere\" size=\"" << radius
            << "\" mass=\"0.1\"/>\n";
        out << "    </body>\n";
      }
    }
  }
  out << "  </worldbody>\n</mujoco>\n";
  return out.str();
}

// Builds a multi-aisle arena with long spanning rails/shelves and 1,500
// scattered dynamic spheres (high active-interval depth in mj_SAP with almost
// zero narrowphase contacts).
std::string BuildArenaXml() {
  std::ostringstream out;
  out << "<mujoco model=\"broadphase_arena\">\n";
  out << "  <size memory=\"64M\"/>\n";
  out << "  <worldbody>\n";
  for (int r = 0; r < 24; ++r) {
    double y = (r - 11.5) * 0.65;
    double z = -1.5 + (r % 3) * 0.4;
    out << "    <body pos=\"0 " << y << " " << z << "\">\n";
    out << "      <geom type=\"box\" size=\"12.0 0.05 0.05\"/>\n";
    out << "    </body>\n";
  }
  uint32_t rng = 0x9E3779B9u;
  for (int ix = 0; ix < 25; ++ix) {
    for (int iy = 0; iy < 20; ++iy) {
      for (int iz = 0; iz < 3; ++iz) {
        double x = (ix - 12.0) * 0.90 + 0.15 * NextSigned(rng);
        double y = (iy - 9.5) * 0.70 + 0.12 * NextSigned(rng);
        double z = 0.5 + iz * 0.45 + 0.08 * NextSigned(rng);
        out << "    <body pos=\"" << x << " " << y << " " << z << "\">\n";
        out << "      <freejoint/>\n";
        out << "      <geom type=\"sphere\" size=\"0.07\" mass=\"0.1\"/>\n";
        out << "    </body>\n";
      }
    }
  }
  out << "  </worldbody>\n</mujoco>\n";
  return out.str();
}

void DeformFlexCage(const mjModel* m, mjData* d, double amp) {
  for (int f = 0; f < m->nflex; ++f) {
    int nstart = m->flex_nodeadr[f];
    int nnum = m->flex_nodenum[f];
    for (int i = 0; i < nnum; ++i) {
      int b = m->flex_nodebodyid[nstart + i];
      int jadr = m->body_jntadr[b];
      if (jadr >= 0 && m->body_jntnum[b] >= 3) {
        int qadr = m->jnt_qposadr[jadr];
        d->qpos[qadr + 0] += amp * std::sin(0.7 * i + f);
        d->qpos[qadr + 1] += amp * std::cos(1.1 * i + 2.0 * f);
        d->qpos[qadr + 2] += amp * std::sin(1.3 * i - f);
      }
    }
  }
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_BroadphaseCloud500(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static const std::string* xml =
      new std::string(BuildSphereCloudXml(8, 8, 8, false, false));
  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(xml->c_str(), error, sizeof(error));
  ASSERT_THAT(m.get(), ::testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_kinematics(m.get(), d.get());
  mj_comPos(m.get(), d.get());

  for (auto s : state) {
    mj_collision(m.get(), d.get());
    benchmark::DoNotOptimize(d->ncon);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_BroadphaseCloud500);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_BroadphaseCloud2000(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static const std::string* xml =
      new std::string(BuildSphereCloudXml(10, 10, 20, false, false));
  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(xml->c_str(), error, sizeof(error));
  ASSERT_THAT(m.get(), ::testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_kinematics(m.get(), d.get());
  mj_comPos(m.get(), d.get());

  for (auto s : state) {
    mj_collision(m.get(), d.get());
    benchmark::DoNotOptimize(d->ncon);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_BroadphaseCloud2000);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_BroadphaseArena1500(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static const std::string* xml = new std::string(BuildArenaXml());
  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(xml->c_str(), error, sizeof(error));
  ASSERT_THAT(m.get(), ::testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_kinematics(m.get(), d.get());
  mj_comPos(m.get(), d.get());

  for (auto s : state) {
    mj_collision(m.get(), d.get());
    benchmark::DoNotOptimize(d->ncon);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_BroadphaseArena1500);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_BroadphaseSleepMix1000(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static const std::string* xml =
      new std::string(BuildSphereCloudXml(10, 10, 10, true, false));
  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(xml->c_str(), error, sizeof(error));
  ASSERT_THAT(m.get(), ::testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());

  for (auto s : state) {
    mj_collision(m.get(), d.get());
    benchmark::DoNotOptimize(d->ncon);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_BroadphaseSleepMix1000);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_BroadphaseStep500(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static const std::string* xml =
      new std::string(BuildSphereCloudXml(8, 8, 8, false, true));
  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(xml->c_str(), error, sizeof(error));
  ASSERT_THAT(m.get(), ::testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);
  for (int i = 0; i < 15; ++i) {
    mj_step(m.get(), d.get());
  }
  int spec = mjSTATE_INTEGRATION;
  std::vector<mjtNum> saved(mj_stateSize(m.get(), spec));
  mj_getState(m.get(), d.get(), saved.data(), spec);

  constexpr int kBatch = 10;
  while (state.KeepRunningBatch(kBatch)) {
    mj_setState(m.get(), d.get(), saved.data(), spec);
    for (int i = 0; i < kBatch; ++i) {
      mj_step(m.get(), d.get());
    }
    benchmark::DoNotOptimize(d->ncon);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_BroadphaseStep500);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_DynamicBVHRefitCloth(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static constexpr char kXml[] = R"(
  <mujoco model="bvh_refit_cloth">
    <size memory="32M"/>
    <worldbody>
      <flexcomp name="cloth" type="box" dim="2" dof="trilinear"
                count="18 18 3" cellcount="4 4 1" spacing="0.03 0.03 0.03"
                pos="0 0 0.5" radius="0.004" mass="1">
        <elasticity young="100" damping="0.01" thickness="0.002"/>
        <contact selfcollide="bvh"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), ::testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);
  DeformFlexCage(m.get(), d.get(), 0.04);
  mj_kinematics(m.get(), d.get());

  for (auto s : state) {
    mj_flex(m.get(), d.get());
    benchmark::DoNotOptimize(d->bvh_aabb_dyn[0]);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_DynamicBVHRefitCloth);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_DynamicBVHRefitTet(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static constexpr char kXml[] = R"(
  <mujoco model="bvh_refit_tet">
    <size memory="64M"/>
    <worldbody>
      <flexcomp name="vol" type="grid" dim="3" dof="trilinear"
                count="10 10 10" cellcount="3 3 3" spacing="0.04 0.04 0.04"
                pos="0 0 0.5" radius="0.004" mass="1">
        <elasticity young="100" damping="0.01"/>
        <contact selfcollide="bvh"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), ::testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);
  DeformFlexCage(m.get(), d.get(), 0.03);
  mj_kinematics(m.get(), d.get());

  for (auto s : state) {
    mj_flex(m.get(), d.get());
    benchmark::DoNotOptimize(d->bvh_aabb_dyn[0]);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_DynamicBVHRefitTet);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_FlexSAPSelfCollide(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static constexpr char kXml[] = R"(
  <mujoco model="flex_sap_self">
    <size memory="64M"/>
    <worldbody>
      <flexcomp name="cloth" type="box" dim="2" dof="trilinear"
                count="20 20 3" cellcount="4 4 3" spacing="0.03 0.03 0.05"
                pos="0 0 0.5" radius="0.004" mass="1">
        <elasticity young="100" damping="0.01" thickness="0.002"/>
        <contact selfcollide="sap"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), ::testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);
  DeformFlexCage(m.get(), d.get(), 0.015);
  mj_kinematics(m.get(), d.get());
  mj_comPos(m.get(), d.get());
  mj_flex(m.get(), d.get());

  for (auto s : state) {
    mj_collision(m.get(), d.get());
    benchmark::DoNotOptimize(d->ncon);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FlexSAPSelfCollide);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_FlexBVHCollide(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static constexpr char kXml[] = R"(
  <mujoco model="flex_bvh_collide">
    <size memory="64M"/>
    <worldbody>
      <flexcomp name="sheet_a" type="box" dim="2" dof="trilinear"
                count="18 18 3" cellcount="4 4 3" spacing="0.03 0.03 0.04"
                pos="0 0 0.50" radius="0.004" mass="1">
        <elasticity young="100" damping="0.01" thickness="0.002"/>
        <contact selfcollide="bvh"/>
      </flexcomp>
      <flexcomp name="sheet_b" type="box" dim="2" dof="trilinear"
                count="18 18 3" cellcount="4 4 3" spacing="0.03 0.03 0.04"
                pos="0.24 0.24 0.585" radius="0.004" mass="1">
        <elasticity young="100" damping="0.01" thickness="0.002"/>
        <contact selfcollide="bvh"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), ::testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);
  DeformFlexCage(m.get(), d.get(), 0.012);
  mj_kinematics(m.get(), d.get());
  mj_comPos(m.get(), d.get());
  mj_flex(m.get(), d.get());

  for (auto s : state) {
    mj_collision(m.get(), d.get());
    benchmark::DoNotOptimize(d->ncon);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FlexBVHCollide);

}  // namespace
}  // namespace mujoco
