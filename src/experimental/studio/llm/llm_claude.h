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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_CLAUDE_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_CLAUDE_H_

#include <string>
#include <vector>

#include "experimental/studio/llm/llm_provider.h"

namespace mujoco::studio {

// Talks to the Anthropic Messages API (POST /v1/messages). Defaults to model
// claude-opus-4-8 with adaptive thinking. The API key is read from the
// ANTHROPIC_API_KEY environment variable (see KeyFromEnv). Transport is WinHTTP
// on Windows; other platforms return an error result for now (no SDK in-tree).
//
// The MVP uses a single, non-streaming request: the conversation is short and
// the reply (a few sentences) fits comfortably under max_tokens. Streaming is a
// later refinement noted in the design doc.
class ClaudeProvider : public LlmProvider {
 public:
  explicit ClaudeProvider(std::string api_key);

  // Returns the ANTHROPIC_API_KEY value, or "" if unset/empty.
  static std::string KeyFromEnv();

  LlmResult Send(const std::string& system,
                 const std::vector<LlmMessage>& messages,
                 const std::vector<ToolDef>& tools,
                 const ToolExecutor& exec) override;

  const char* name() const override { return "Claude"; }

  // Accepts "opus"/"sonnet"/"haiku" or a full "claude-..." id; updates the
  // thinking mode to suit (adaptive for the opus/sonnet 4.6+ family, off for
  // Haiku, which doesn't accept it). Returns the resolved id, or "".
  std::string SetModel(const std::string& id_or_alias) override;
  std::string Model() const override { return model_; }

 private:
  std::string api_key_;
  std::string model_ = "claude-opus-4-8";
  bool adaptive_thinking_ = true;
  int max_tokens_ = 8192;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_CLAUDE_H_
