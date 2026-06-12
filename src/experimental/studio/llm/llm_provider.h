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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_PROVIDER_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_PROVIDER_H_

#include <string>
#include <vector>

namespace mujoco::studio {

// One turn in the conversation. `role` is "user" or "assistant".
struct LlmMessage {
  std::string role;
  std::string text;
};

// The outcome of a single request.
struct LlmResult {
  bool ok = false;
  std::string text;   // assistant reply, when ok.
  std::string error;  // human-readable message, when !ok.
};

// Provider-agnostic seam for talking to an LLM. The MVP only needs a plain
// text turn; tool use (the ImGui Test Engine path) will extend this later
// without changing how UiAgent drives it. Implementations are called on a
// worker thread, so Send() may block on network I/O.
class LlmProvider {
 public:
  virtual ~LlmProvider() = default;

  // `system` is the system prompt; `messages` is the running conversation,
  // oldest first, ending with the latest user turn.
  virtual LlmResult Send(const std::string& system,
                         const std::vector<LlmMessage>& messages) = 0;

  // Short name for the status line (e.g. "Claude", "mock").
  virtual const char* name() const = 0;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_PROVIDER_H_
