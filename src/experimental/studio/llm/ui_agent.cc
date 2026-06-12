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
      "physics simulator. The user types requests into a command box. You can "
      "open Studio's tool windows by calling the open_tool_window tool (the "
      "panels on the left icon rail: Physics, Rendering, Visualization, Joints, "
      "Controls, Sensor, Watch, State, Explorer, Editor). When the user asks to "
      "open, show, or bring up a panel, call the tool. Keep any text replies to "
      "one short sentence.";

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
