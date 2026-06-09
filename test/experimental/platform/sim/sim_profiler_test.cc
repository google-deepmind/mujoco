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

  // Records one frame with a known average step time. The component CPU timers
  // are left at zero, so cpu_total (and hence cpu_other) is exactly `step_ms`.
  SimProfiler profiler;
  mjData* data = holder->data();
  auto record_frame = [&](float step_ms) {
    data->timer[mjTIMER_STEP].duration = step_ms;
    data->timer[mjTIMER_STEP].number = 1;
    profiler.Update(holder->model(), data);
  };

  const int capacity = SimProfiler().GetSummary().capacity;
  ASSERT_GT(capacity, 1);
  const float nv = static_cast<float>(holder->model()->nv);

  // Record capacity - 1 frames with step times 1, 2, ..., capacity - 1.
  for (int i = 1; i <= capacity - 1; ++i) {
    record_frame(static_cast<float>(i));
  }
  {
    SimProfiler::Summary s = profiler.GetSummary();
    EXPECT_EQ(s.num_frames, capacity - 1);
    EXPECT_EQ(s.capacity, capacity);
    // Mean of 1..capacity-1, and cpu_other absorbs the whole step time.
    EXPECT_METRIC_EQ(s.cpu_total, capacity / 2.0f, 1.0f,
                     static_cast<float>(capacity - 1));
    EXPECT_FLOAT_EQ(s.cpu_other.average, s.cpu_total.average);
    // Dimensions are constant across frames.
    EXPECT_METRIC_EQ(s.dim_dof, nv, nv, nv);
  }

  // One more frame fills the buffer exactly (values 1..capacity).
  record_frame(static_cast<float>(capacity));
  {
    SimProfiler::Summary s = profiler.GetSummary();
    EXPECT_EQ(s.num_frames, capacity);
    EXPECT_METRIC_EQ(s.cpu_total, (capacity + 1) / 2.0f, 1.0f,
                     static_cast<float>(capacity));
  }

  // One more frame wraps: slot 0 (value 1) is overwritten with capacity + 1,
  // so the retained window is 2..capacity+1 and num_frames saturates.
  record_frame(static_cast<float>(capacity + 1));
  {
    SimProfiler::Summary s = profiler.GetSummary();
    EXPECT_EQ(s.num_frames, capacity);
    EXPECT_METRIC_EQ(s.cpu_total, (capacity + 3) / 2.0f, 2.0f,
                     static_cast<float>(capacity + 1));
  }

  // Clear resets the counts and statistics (capacity is intrinsic and stays).
  profiler.Clear();
  {
    SimProfiler::Summary s = profiler.GetSummary();
    EXPECT_EQ(s.num_frames, 0);
    EXPECT_EQ(s.capacity, capacity);
    EXPECT_METRIC_EQ(s.cpu_total, 0.0f, 0.0f, 0.0f);
    EXPECT_METRIC_EQ(s.dim_dof, 0.0f, 0.0f, 0.0f);
  }
}

}  // namespace
}  // namespace mujoco::platform
