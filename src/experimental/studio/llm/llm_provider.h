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

#include <functional>
#include <string>
#include <vector>

namespace mujoco::studio {

// One turn in the conversation. `role` is "user" or "assistant".
struct LlmMessage {
  std::string role;
  std::string text;
};

// A tool the model may call. `input_schema` is a JSON-Schema object (as a JSON
// string) describing the arguments.
struct ToolDef {
  std::string name;
  std::string description;
  std::string input_schema;
};

// Executes a tool call and returns a short human-readable result string that is
// fed back to the model. `json_args` is the raw JSON arguments object the model
// produced. The provider runs the tool-use loop internally and invokes this for
// each call; implementations should be quick and side-effecting (e.g. open a
// window) -- this is the "as if clicking the button" seam.
using ToolExecutor =
    std::function<std::string(const std::string& name,
                              const std::string& json_args)>;

// The outcome of a request (after any tool-use round trips).
struct LlmResult {
  bool ok = false;
  std::string text;   // final assistant reply, when ok.
  std::string error;  // human-readable message, when !ok.
};

// Provider-agnostic seam for talking to an LLM. Implementations are called on a
// worker thread (or inline in synchronous capture), so Send() may block on
// network I/O and on `exec`.
class LlmProvider {
 public:
  virtual ~LlmProvider() = default;

  // `system` is the system prompt; `messages` is the running conversation,
  // oldest first, ending with the latest user turn. `tools` are offered to the
  // model; when it calls one, `exec` runs it and the loop continues until the
  // model returns a plain-text answer.
  virtual LlmResult Send(const std::string& system,
                         const std::vector<LlmMessage>& messages,
                         const std::vector<ToolDef>& tools,
                         const ToolExecutor& exec) = 0;

  // Short name for the status line (e.g. "Claude", "mock").
  virtual const char* name() const = 0;

  // Switch the active model. `id_or_alias` is a friendly alias ("opus",
  // "sonnet", "haiku") or a full model id. Returns the resolved model id, or ""
  // if unrecognized/unsupported. Default: no-op.
  virtual std::string SetModel(const std::string& /*id_or_alias*/) {
    return "";
  }

  // The current model id (e.g. "claude-opus-4-8"), or "" if not applicable.
  virtual std::string Model() const { return ""; }
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_PROVIDER_H_
