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

#include "experimental/studio/llm/ui_agent.h"

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "experimental/studio/llm/llm_claude.h"
#include "experimental/studio/llm/llm_mock.h"
#include "experimental/studio/llm/llm_provider.h"

namespace mujoco::studio {

UiAgent::UiAgent() {
  system_ =
      "You are an AI assistant embedded in MuJoCo Studio, a GUI for the MuJoCo "
      "physics simulator. The user types requests into a command box. You act on "
      "the UI exclusively by calling the run_ui_program tool, which drives the "
      "real on-screen widgets through the ImGui Test Engine (clicking buttons, "
      "opening panels, etc.) -- see that tool's description for how to reference "
      "items.\n"
      "The refs you use MUST correspond to real widgets; do not invent ids. The "
      "rail-button refs in the run_ui_program description are reliable. For "
      "joint names/sliders, grep_source to confirm the exact name (e.g. grep "
      "'knee').\n"
      "STRICT BUDGET: use at most ~4 grep_source calls TOTAL, then emit one "
      "run_ui_program and finish. Do not keep exploring. If a control has no "
      "clean ref, use its keyboard shortcut instead -- in particular, to "
      "pause/play press Space via {\"op\":\"key_chars\",\"text\":\" \"}; never "
      "hunt for the pause button's ref. If you can't reference something after a "
      "grep or two, skip it and proceed with the rest.\n"
      "Keep any text replies to one short sentence.";

  std::string key = ClaudeProvider::KeyFromEnv();
  if (!key.empty()) {
    provider_ = std::make_shared<ClaudeProvider>(std::move(key));
  } else {
    provider_ = std::make_shared<MockProvider>();
  }
  provider_name_ = provider_->name();
}

void UiAgent::set_provider(std::unique_ptr<LlmProvider> provider) {
  if (!provider) return;
  provider_ = std::move(provider);
  provider_name_ = provider_->name();
}

void UiAgent::set_tools(std::vector<ToolDef> tools, ToolExecutor exec) {
  tools_ = std::move(tools);
  executor_ = std::move(exec);
}

void UiAgent::Ask(const std::string& question) {
  if (question.empty() || busy_) return;

  history_.push_back({"user", question});
  if (on_ask_) on_ask_();  // reset per-turn budgets (e.g. grep count)
  busy_ = true;

  std::vector<LlmMessage> messages;
  messages.reserve(history_.size());
  for (const Turn& t : history_) messages.push_back({t.role, t.text});

  if (synchronous_) {
    LlmResult r = provider_->Send(system_, messages, tools_, executor_);
    history_.push_back({"assistant", r.ok ? r.text : ("[error] " + r.error)});
    busy_ = false;
    return;
  }

  pending_ = std::make_shared<Pending>();
  auto provider = provider_;          // shared: outlives the agent if needed.
  auto pending = pending_;
  std::string system = system_;
  std::vector<ToolDef> tools = tools_;
  ToolExecutor exec = executor_;
  std::thread([provider, pending, system, messages, tools, exec] {
    LlmResult r = provider->Send(system, messages, tools, exec);
    std::lock_guard<std::mutex> lk(pending->mu);
    pending->result = std::move(r);
    pending->done = true;
  }).detach();
}

void UiAgent::Poll() {
  if (!pending_) return;
  auto pending = pending_;  // keep alive across the unlock below.
  std::lock_guard<std::mutex> lk(pending->mu);
  if (!pending->done) return;
  const LlmResult& r = pending->result;
  history_.push_back({"assistant", r.ok ? r.text : ("[error] " + r.error)});
  busy_ = false;
  pending_.reset();
}

}  // namespace mujoco::studio
