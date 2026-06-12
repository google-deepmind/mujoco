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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_MOCK_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_MOCK_H_

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include "experimental/studio/llm/llm_provider.h"

namespace mujoco::studio {

// A deterministic, offline provider. Used for headless GIF capture and when no
// ANTHROPIC_API_KEY is set, so the UI plumbing (including the open_tool_window
// tool) can be exercised without a network call. It "decides" to open a panel
// by scanning the prompt for a tool's panel name.
class MockProvider : public LlmProvider {
 public:
  LlmResult Send(const std::string& /*system*/,
                 const std::vector<LlmMessage>& messages,
                 const std::vector<ToolDef>& /*tools*/,
                 const ToolExecutor& exec) override {
    LlmResult r;
    r.ok = true;
    std::string q = messages.empty() ? "" : messages.back().text;
    std::string lower = q;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // Crudely mirror Claude: if the prompt names a known panel, "call" the tool.
    static const char* kPanels[] = {"physics",   "rendering", "visualization",
                                    "joints",    "controls",  "sensor",
                                    "watch",     "state",     "explorer",
                                    "editor"};
    for (const char* p : kPanels) {
      if (lower.find(p) != std::string::npos) {
        std::string title(1, static_cast<char>(std::toupper(p[0])));
        title += (p + 1);
        if (exec) {
          exec("open_tool_window", "{\"name\":\"" + title + "\"}");
        }
        r.text = "Opened the " + title + " panel.";
        return r;
      }
    }
    r.text =
        "Connected. Ask me to open a panel (e.g. \"open the physics menu\").";
    return r;
  }

  const char* name() const override { return "mock"; }
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_MOCK_H_
