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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_UI_AGENT_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_UI_AGENT_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "experimental/studio/llm/llm_provider.h"

namespace mujoco::studio {

// Routes plain-English command-prompt input to an LLM and keeps the running
// conversation. Networking happens on a detached worker thread; the UI thread
// calls Poll() once per frame to fold a finished reply into the history. For
// deterministic headless capture, set_synchronous(true) makes Ask() block
// inline (used with the mock provider).
//
// This is the MVP seam from LLM_INTEGRATION_DESIGN.md: text in, text out, no
// tools yet. The provider abstraction and conversation model are what the Test
// Engine tool-use loop will build on.
class UiAgent {
 public:
  struct Turn {
    std::string role;  // "user" or "assistant"
    std::string text;
  };

  UiAgent();  // ClaudeProvider if ANTHROPIC_API_KEY is set, else MockProvider.
  ~UiAgent() = default;

  UiAgent(const UiAgent&) = delete;
  UiAgent& operator=(const UiAgent&) = delete;

  // Submits a user question. No-op if empty or a request is already in flight.
  void Ask(const std::string& question);

  // Folds any finished worker result into the history. Call once per frame.
  void Poll();

  bool busy() const { return busy_; }
  const std::vector<Turn>& history() const { return history_; }
  const std::string& provider_name() const { return provider_name_; }

  void set_synchronous(bool s) { synchronous_ = s; }
  void set_provider(std::unique_ptr<LlmProvider> provider);

 private:
  std::shared_ptr<LlmProvider> provider_;
  std::string provider_name_;
  std::string system_;
  std::vector<Turn> history_;

  bool synchronous_ = false;
  std::atomic<bool> busy_{false};

  // Result hand-off from the worker thread. Held by shared_ptr so the worker is
  // safe even if the UiAgent is destroyed mid-request.
  struct Pending {
    std::mutex mu;
    bool done = false;
    LlmResult result;
  };
  std::shared_ptr<Pending> pending_;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_UI_AGENT_H_
