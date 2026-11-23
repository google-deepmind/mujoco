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

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_thread.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using ThreadTest = MujocoTest;

TEST_F(ThreadTest, SingleAndMultiThreadedMatch) {
  auto model_path = GetModelPath("humanoid/22_humanoids.xml");
  std::array<char, 1024> error;
  mjModel* model =
      mj_loadXML(model_path.c_str(), nullptr, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  model->opt.solver = mjSOL_CG;  // use CG solver

  mjModel* model_threaded =
      mj_loadXML(model_path.c_str(), nullptr, error.data(), error.size());
  ASSERT_THAT(model_threaded, NotNull()) << error.data();
  model_threaded->opt.solver = mjSOL_CG;  // use CG solver

  mjData* data = mj_makeData(model);
  mjData* data_threaded = mj_makeData(model_threaded);

  // warm-up rollout to get a steady state
  static constexpr int kNumWarmupSteps = 100;
  for (int i = 0; i < kNumWarmupSteps; i++) {
    mj_step(model, data);
  }

  // sync the models and data
  int spec = mjSTATE_INTEGRATION;
  int size = mj_stateSize(model, spec);
  std::vector<mjtNum> initial_state(size);
  mj_getState(model, data, initial_state.data(), spec);
  mj_setState(model_threaded, data_threaded, initial_state.data(), spec);

  // bind a threadpool to the data_threaded
  mju_threadpool(data_threaded, 10);

  for (int i = 0; i < 10; ++i) {
    mj_step(model, data);
    mj_step(model_threaded, data_threaded);
  }

  // compare the mjData's.
  {
#define X(type, name, nr, nc)                               \
  EXPECT_EQ(std::memcmp(data->name, data_threaded->name,    \
                        sizeof(type) * (model->nr) * (nc)), \
            0)                                              \
      << "mjData::" #name " differs";
    MJDATA_POINTERS
#undef X
  }

  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteData(data_threaded);
  mj_deleteModel(model_threaded);
}

TEST_F(ThreadTest, IslandSingleAndMultiThreadedMatch) {
  auto model_path = GetModelPath("humanoid/22_humanoids.xml");
  std::array<char, 1024> error;
  mjModel* model =
      mj_loadXML(model_path.c_str(), nullptr, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  model->opt.solver = mjSOL_CG;               // use CG solver
  model->opt.disableflags &= ~mjDSBL_ISLAND;  // enable islands

  mjModel* model_threaded =
      mj_loadXML(model_path.c_str(), nullptr, error.data(), error.size());
  ASSERT_THAT(model_threaded, NotNull()) << error.data();
  model_threaded->opt.solver = mjSOL_CG;               // use CG solver
  model_threaded->opt.disableflags &= ~mjDSBL_ISLAND;  // enable islands

  mjData* data = mj_makeData(model);
  mjData* data_threaded = mj_makeData(model_threaded);

  // warm-up rollout to get a steady state
  static constexpr int kNumWarmupSteps = 100;
  for (int i = 0; i < kNumWarmupSteps; i++) {
    mj_step(model, data);
  }

  // sync the models and data
  int spec = mjSTATE_INTEGRATION;
  int size = mj_stateSize(model, spec);
  std::vector<mjtNum> initial_state(size);
  mj_getState(model, data, initial_state.data(), spec);
  mj_setState(model_threaded, data_threaded, initial_state.data(), spec);

  // bind a threadpool to the data_threaded
  mju_threadpool(data_threaded, 10);

  for (int i = 0; i < 10; ++i) {
    mj_step(model, data);
    mj_step(model_threaded, data_threaded);
  }

  // compare the mjData's.
  {
#define X(type, name, nr, nc)                               \
  EXPECT_EQ(std::memcmp(data->name, data_threaded->name,    \
                        sizeof(type) * (model->nr) * (nc)), \
            0)                                              \
      << "mjData::" #name " differs";
    MJDATA_POINTERS
#undef X
  }

  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteData(data_threaded);
  mj_deleteModel(model_threaded);
}

TEST_F(ThreadTest, ResetAndCopyPreserveThreadPoolAndClearStack) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 1">
        <freejoint/>
        <geom type="sphere" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  mjData* d = mj_makeData(m.get());
  mjData* src = mj_makeData(m.get());

  mju_threadpool(d, 4);
  ASSERT_NE(d->threadpool, 0u);
  EXPECT_EQ(mju_numThread(d), 5);

  d->pstack = 128;
  mj_resetData(m.get(), d);
  EXPECT_EQ(d->pstack, 0u);
  EXPECT_EQ(mju_numThread(d), 5);

  mj_step(m.get(), src);
  mj_copyData(d, m.get(), src);
  EXPECT_NE(d->threadpool, 0u);
  EXPECT_EQ(mju_numThread(d), 5);

  mj_deleteData(src);
  mj_deleteData(d);
}

struct DispatchVerifyArgs {
  std::vector<std::atomic<int>> counts;
  std::vector<int> owner_thread;
  int heavy_task = -1;

  explicit DispatchVerifyArgs(int n) : counts(n), owner_thread(n, -1) {
    for (int i = 0; i < n; ++i) {
      counts[i].store(0, std::memory_order_relaxed);
    }
  }
};

void VerifyTaskCallback(const mjModel* m, mjData* d, void* arg, int thread_id,
                        int task_id) {
  auto* v = static_cast<DispatchVerifyArgs*>(arg);
  v->counts[task_id].fetch_add(1, std::memory_order_relaxed);
  v->owner_thread[task_id] = thread_id;
  if (task_id == v->heavy_task) {
    volatile double sink = 1.0;
    for (int k = 0; k < 2000; ++k) {
      sink = sink * 1.0001 + 0.0001;
    }
  }
}

TEST_F(ThreadTest, DispatchTaskCountsAndStealing) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody/>
  </mujoco>
  )";

  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  mjData* d = mj_makeData(m.get());
  mju_threadpool(d, 4);
  ASSERT_EQ(mju_numThread(d), 5);

  // Test 0, 1, < W, == W, > W, and >> W tasks via both mju_dispatch and
  // mju_dispatchPool, including skewed task loads to exercise work-stealing.
  for (int ntask : {0, 1, 2, 4, 5, 6, 7, 19, 64, 512}) {
    for (bool direct_pool : {false, true}) {
      DispatchVerifyArgs args(ntask > 0 ? ntask : 1);
      args.heavy_task = ntask > 4 ? 0 : -1;
      if (direct_pool) {
        mju_dispatchPool(m.get(), d, VerifyTaskCallback, &args, ntask);
      } else {
        mju_dispatch(m.get(), d, VerifyTaskCallback, &args, ntask);
      }
      if (ntask == 0) {
        EXPECT_EQ(args.counts[0].load(), 0);
      }
      for (int i = 0; i < ntask; ++i) {
        EXPECT_EQ(args.counts[i].load(), 1) << "ntask=" << ntask << " i=" << i;
        EXPECT_GE(args.owner_thread[i], 0);
        EXPECT_LT(args.owner_thread[i], 5);
      }
    }
  }

  mj_deleteData(d);
}

void MainThreadTaskCallback(const mjModel* m, mjData* d, void* arg,
                            int thread_id, int task_id) {
  auto* main_ran = static_cast<std::atomic<bool>*>(arg);
  if (thread_id == 0) {
    main_ran->store(true, std::memory_order_release);
    return;
  }
  // hold worker tasks until the main thread has run one (bounded wait)
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!main_ran->load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
  }
}

TEST_F(ThreadTest, DispatchMainThreadRunsTasks) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody/>
  </mujoco>
  )";

  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  mjData* d = mj_makeData(m.get());
  mju_threadpool(d, 4);

  // the main thread works through its own slice instead of only waiting
  std::atomic<bool> main_ran{false};
  mju_dispatchPool(m.get(), d, MainThreadTaskCallback, &main_ran, 64);
  EXPECT_TRUE(main_ran.load());

  mj_deleteData(d);
}

struct HomeSliceArgs {
  static constexpr int kWorkers = 5;
  std::atomic<int> started{0};
  std::array<std::atomic<int>, kWorkers> first_task;
  HomeSliceArgs() {
    for (auto& t : first_task) t.store(-1, std::memory_order_relaxed);
  }
};

void HomeSliceTaskCallback(const mjModel* m, mjData* d, void* arg,
                           int thread_id, int task_id) {
  auto* a = static_cast<HomeSliceArgs*>(arg);
  int expected = -1;
  if (!a->first_task[thread_id].compare_exchange_strong(expected, task_id)) {
    return;
  }
  // barrier: no thread can steal before every thread has claimed a task
  a->started.fetch_add(1, std::memory_order_acq_rel);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (a->started.load(std::memory_order_acquire) < HomeSliceArgs::kWorkers &&
         std::chrono::steady_clock::now() < deadline) {
  }
}

TEST_F(ThreadTest, DispatchStartsEachThreadOnItsHomeSlice) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody/>
  </mujoco>
  )";

  char error[1024] = "";
  MjModelPtr m = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  mjData* d = mj_makeData(m.get());
  mju_threadpool(d, HomeSliceArgs::kWorkers - 1);

  // 20 tasks over 5 threads: thread t owns tasks [4t, 4t+4)
  HomeSliceArgs args;
  mju_dispatchPool(m.get(), d, HomeSliceTaskCallback, &args, 20);
  for (int t = 0; t < HomeSliceArgs::kWorkers; ++t) {
    EXPECT_EQ(args.first_task[t].load(), 4 * t) << "thread " << t;
  }

  mj_deleteData(d);
}

}  // namespace
}  // namespace mujoco
