// Copyright 2025 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_TEST_RUNNER_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_TEST_RUNNER_H_

#include <mutex>
#include <string>
#include <vector>

struct ImGuiTestEngine;
struct ImGuiTest;
struct ImGuiTestContext;

namespace mujoco::studio {

// Owns the Dear ImGui Test Engine and runs LLM-authored UI programs through it.
//
// A program is a JSON array of typed ops (item_click, menu_click, set_float,
// ...) whose refs are ImGui item IDs/paths -- the sole actuator from
// LLM_INTEGRATION_DESIGN.md. The model emits the program via the run_ui_program
// tool; Run() enqueues it (safe to call from the LLM worker thread), and the UI
// thread (Pump, called from PostSwap) queues it on the engine, which performs
// the real clicks across frames. So widget mutation always happens on the UI
// thread regardless of who asked.
class TestRunner {
 public:
  TestRunner() = default;
  ~TestRunner();

  TestRunner(const TestRunner&) = delete;
  TestRunner& operator=(const TestRunner&) = delete;

  // Creates + starts the engine bound to the current ImGui context. Call after
  // the context exists. Restartable across graphics-mode switches.
  void Start();
  // Stops the engine. Must be called while the ImGui context is still alive.
  void Stop();
  // Destroys the engine. Must be called AFTER ImGui::DestroyContext() (the test
  // engine asserts on the order), i.e. after the owning window is destroyed.
  void Destroy();
  bool started() const { return engine_ != nullptr && running_; }

  // Per-frame tick: drains the program queue (queues a test if the engine is
  // idle) and advances the engine. Call once per frame after rendering.
  void PostSwap();

  // Enqueues a UI program. `json_args` is the run_ui_program arguments object,
  // i.e. {"ops":[ ... ]}. Returns the number of ops parsed. Thread-safe.
  int Run(const std::string& json_args);

 private:
  static void TestFuncThunk(ImGuiTestContext* ctx);
  void Execute(ImGuiTestContext* ctx, const std::string& ops_json);

  ImGuiTestEngine* engine_ = nullptr;
  ImGuiTest* test_ = nullptr;
  bool running_ = false;

  std::mutex mu_;
  std::vector<std::string> queue_;  // pending programs (ops arrays as JSON)
  std::string running_ops_;         // program the queued test is executing
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_TEST_RUNNER_H_
