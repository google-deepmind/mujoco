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

#include <string>
#include <vector>

#include "experimental/studio/llm/llm_provider.h"

namespace mujoco::studio {

// A deterministic, offline provider. Used for headless GIF capture and when no
// ANTHROPIC_API_KEY is set, so the UI plumbing can be exercised end to end
// without a network call. The reply is canned but echoes the question so demos
// read naturally.
class MockProvider : public LlmProvider {
 public:
  LlmResult Send(const std::string& /*system*/,
                 const std::vector<LlmMessage>& messages) override {
    const std::string q = messages.empty() ? "" : messages.back().text;
    LlmResult r;
    r.ok = true;
    r.text =
        "Connected. Once UI control is wired I'll do this by clicking the "
        "Studio buttons for you. For now, try the Physics panel to adjust the "
        "relevant field.";
    return r;
  }

  const char* name() const override { return "mock"; }
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_MOCK_H_
