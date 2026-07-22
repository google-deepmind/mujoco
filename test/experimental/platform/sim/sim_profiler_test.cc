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

#include "experimental/platform/sim/sim_profiler.h"

#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"
#include "experimental/platform/sim/model_holder.h"

namespace mujoco::platform {
namespace {

using ::testing::NotNull;

using SimProfilerTest = MujocoTest;

// Checks the average, min and max of a single profiler metric.
#define EXPECT_METRIC_EQ(metric, avg, lo, hi) \
  {                                           \
    EXPECT_FLOAT_EQ((metric).average, (avg)); \
    EXPECT_FLOAT_EQ((metric).min, (lo));      \
    EXPECT_FLOAT_EQ((metric).max, (hi));      \
  }

TEST_F(SimProfilerTest, SummaryAcrossWrap) {
  // A minimal one-body free-joint model is enough: only the model dimensions
  // and the (manually set) CPU timers feed the summary.
  mjSpec* spec = mj_makeSpec();
  mjsBody* body = mjs_addBody(mjs_findBody(spec, "world"), nullptr);
  mjs_addJoint(body, nullptr)->type = mjJNT_FREE;
  mjsGeom* geom = mjs_addGeom(body, nullptr);
  geom->type = mjGEOM_SPHERE;
  geom->size[0] = 0.1;
  std::unique_ptr<ModelHolder> holder = ModelHolder::FromSpec(spec);
  ASSERT_THAT(holder->model(), NotNull()) << holder->error();

  // Records one frame with step time 1, 2, 3, ... on successive calls, so the
  // recorded cpu_total equals kCpuScale times the call count (Update() scales
  // the raw timer values by 1000 to display microseconds).
  constexpr float kCpuScale = 1000.0f;
  SimProfiler profiler;
  int frame = 0;
  auto record_frame = [&]() {
    holder->data()->timer[mjTIMER_STEP].duration = static_cast<mjtNum>(++frame);
    holder->data()->timer[mjTIMER_STEP].number = 1;
    profiler.Update(holder->model(), holder->data());
  };

  const int max_frames = SimProfiler().GetSummary().max_frames;
  ASSERT_GT(max_frames, 1);

  // Record max_frames - 1 frames with step times 1, 2, ..., max_frames - 1.
  for (int i = 1; i <= max_frames - 1; ++i) {
    record_frame();
  }
  {
    SimProfiler::Summary s = profiler.GetSummary();
    EXPECT_EQ(s.num_frames, max_frames - 1);
    EXPECT_EQ(s.max_frames, max_frames);
    // cpu_total is the mean of step times 1..max_frames-1. The component timers
    // are zero, so cpu_other == cpu_total.
    EXPECT_METRIC_EQ(s.cpu_total, kCpuScale * max_frames / 2.0f, kCpuScale,
                     kCpuScale * (max_frames - 1));
    EXPECT_FLOAT_EQ(s.cpu_other.average, s.cpu_total.average);
    const int nv = holder->model()->nv;
    EXPECT_METRIC_EQ(s.dim_dof, nv, nv, nv);
  }

  // One more frame fills the buffer exactly (values 1..max_frames).
  record_frame();
  {
    SimProfiler::Summary s = profiler.GetSummary();
    EXPECT_EQ(s.num_frames, max_frames);
    EXPECT_METRIC_EQ(s.cpu_total, kCpuScale * (max_frames + 1) / 2.0f,
                     kCpuScale, kCpuScale * max_frames);
  }

  // One more frame wraps: slot 0 (value 1) is overwritten with max_frames + 1,
  // so the retained window is 2..max_frames+1 and num_frames saturates.
  record_frame();
  {
    SimProfiler::Summary s = profiler.GetSummary();
    EXPECT_EQ(s.num_frames, max_frames);
    EXPECT_METRIC_EQ(s.cpu_total, kCpuScale * (max_frames + 3) / 2.0f,
                     kCpuScale * 2, kCpuScale * (max_frames + 1));
  }

  // Clear the profiler and check for empty summary.
  profiler.Clear();
  {
    SimProfiler::Summary s = profiler.GetSummary();
    EXPECT_EQ(s.num_frames, 0);
    EXPECT_EQ(s.max_frames, max_frames);
  }
}

}  // namespace
}  // namespace mujoco::platform
