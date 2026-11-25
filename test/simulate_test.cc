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

#include <array>
#include <chrono>
#include <cstddef>
#include <future>
#include <memory>
#include <thread>
#include <utility>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "simulate/platform_ui_adapter.h"
#include "simulate/simulate.h"

namespace mujoco {
namespace {

using namespace std::chrono_literals;

// No window or graphics context is needed to exercise the upload lifecycle.
class TestAdapter : public PlatformUIAdapter {
 public:
  std::pair<double, double> GetCursorPosition() const override { return {}; }
  double GetDisplayPixelsPerInch() const override { return 96; }
  std::pair<int, int> GetFramebufferSize() const override { return {}; }
  std::pair<int, int> GetWindowSize() const override { return {}; }
  bool IsGPUAccelerated() const override { return false; }
  void PollEvents() override {}
  void SetClipboardString(const char*) override {}
  void SetVSync(bool) override {}
  void SetWindowTitle(const char*) override {}
  bool ShouldCloseWindow() const override { return true; }
  void SwapBuffers() override {}
  void ToggleFullscreen() override {}
  bool IsLeftMouseButtonPressed() const override { return false; }
  bool IsMiddleMouseButtonPressed() const override { return false; }
  bool IsRightMouseButtonPressed() const override { return false; }
  bool IsAltKeyPressed() const override { return false; }
  bool IsCtrlKeyPressed() const override { return false; }
  bool IsShiftKeyPressed() const override { return false; }
  bool IsMouseButtonDownEvent(int) const override { return false; }
  bool IsKeyDownEvent(int) const override { return false; }
  int TranslateKeyCode(int key) const override { return key; }
  mjtButton TranslateMouseButton(int) const override { return mjBUTTON_NONE; }
};

TEST(SimulateTest, PendingAssetUploadsReturnWhenRenderLoopEnds) {
  mjvCamera cam{};
  mjvOption opt{};
  mjvPerturb pert{};
  auto sim = std::make_unique<Simulate>(std::make_unique<TestAdapter>(), &cam,
                                       &opt, &pert, false);
  mjModel model{};
  model.nhfield = model.nmesh = model.ntex = 1;
  sim->m_ = &model;

  constexpr std::array updates = {&Simulate::UpdateHField,
                                  &Simulate::UpdateMesh,
                                  &Simulate::UpdateTexture};
  std::array<std::future<void>, updates.size()> pending;
  for (std::size_t i = 0; i < updates.size(); ++i) {
    pending[i] = std::async(std::launch::async, updates[i], sim.get(), 0);
  }

  // Observe all three queued requests under the same lock as the render loop.
  const auto deadline = Simulate::Clock::now() + 5s;
  bool queued = false;
  while (Simulate::Clock::now() < deadline) {
    {
      const MutexLock lock(sim->mtx);
      queued = sim->hfield_upload_ == 0 && sim->mesh_upload_ == 0 &&
               sim->texture_upload_ == 0;
    }
    if (queued) {
      break;
    }
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(queued);

  sim->FinishRenderLoop();
  EXPECT_EQ(sim->exitrequest.load(), 2);

  auto check_returned = [&](std::future<void>& upload) {
    if (upload.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "Asset upload did not return after the render loop ended";
      // Do not strand a worker if the shutdown regression reappears.
      const MutexLock lock(sim->mtx);
      sim->hfield_upload_ = sim->mesh_upload_ = sim->texture_upload_ = -1;
      sim->cond_upload_.notify_all();
    }
    upload.get();
  };
  for (auto& upload : pending) {
    check_returned(upload);
  }

  // Calls arriving after shutdown must also return without queuing work.
  for (auto update : updates) {
    auto late = std::async(std::launch::async, update, sim.get(), 0);
    check_returned(late);
  }
  EXPECT_EQ(sim->hfield_upload_, -1);
  EXPECT_EQ(sim->mesh_upload_, -1);
  EXPECT_EQ(sim->texture_upload_, -1);
}

}  // namespace
}  // namespace mujoco
