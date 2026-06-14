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

#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "experimental/studio/llm/llm_claude.h"
#include "experimental/studio/llm/llm_mock.h"
#include "experimental/studio/llm/llm_provider.h"

#ifndef MUJOCO_STUDIO_SOURCE_DIR
#define MUJOCO_STUDIO_SOURCE_DIR ""
#endif

namespace mujoco::studio {
namespace {

std::string ReadFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return "";
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// The system prompt lives in an editable file on disk so it can be tweaked
// without recompiling (and it hot-reloads each turn). Lookup order: the
// MUJOCO_STUDIO_SYSTEM_PROMPT env var, then llm/system_prompt.md in the source
// tree, then a built-in fallback.
std::string LoadSystemPrompt() {
  if (const char* env = std::getenv("MUJOCO_STUDIO_SYSTEM_PROMPT");
      env && *env) {
    if (std::string s = ReadFile(env); !s.empty()) return s;
  }
  const std::string root = MUJOCO_STUDIO_SOURCE_DIR;
  if (!root.empty()) {
    if (std::string s = ReadFile(root + "/studio/llm/system_prompt.md");
        !s.empty()) {
      return s;
    }
  }
  return
      "You are an AI assistant embedded in MuJoCo Studio. Act on the UI only by "
      "calling run_ui_program (it drives the real widgets via the ImGui Test "
      "Engine). Use model_info for joint/actuator/body names and grep_source for "
      "source-level ids; do not invent refs. Keep replies to one short sentence.";
}

}  // namespace

UiAgent::UiAgent() {
  system_ = LoadSystemPrompt();

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

  // "/model [opus|sonnet|haiku|<id>]" is a local command: switch the provider's
  // model (or report the current one) without calling the model.
  if (question.rfind("/model", 0) == 0) {
    std::string arg = question.substr(6);
    const size_t b = arg.find_first_not_of(" \t");
    const size_t e = arg.find_last_not_of(" \t");
    arg = (b == std::string::npos) ? "" : arg.substr(b, e - b + 1);
    std::string msg;
    if (arg.empty()) {
      const std::string cur = provider_->Model();
      msg = "Current model: " + (cur.empty() ? std::string("(n/a)") : cur) +
            ". Usage: /model opus | sonnet | haiku (or a full claude-... id).";
    } else if (std::string id = provider_->SetModel(arg); !id.empty()) {
      msg = "Switched to " + id + ".";
    } else {
      msg = "Unknown model \"" + arg + "\". Try: opus, sonnet, haiku.";
    }
    history_.push_back({"user", question});
    history_.push_back({"assistant", msg});
    return;
  }

  system_ = LoadSystemPrompt();  // hot-reload so edits apply without a restart
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
