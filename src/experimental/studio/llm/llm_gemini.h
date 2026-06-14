// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_GEMINI_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_GEMINI_H_

#include <string>
#include <vector>

#include "experimental/studio/llm/llm_provider.h"

namespace mujoco::studio {

// Talks to the Google Gemini API (generativelanguage.googleapis.com,
// :generateContent) with function calling, mirroring ClaudeProvider's own
// hand-rolled tool-use loop. Defaults to gemini-2.5-flash; switchable via
// SetModel / the "/model gemini-..." command. The API key is read from
// GEMINI_API_KEY (or GOOGLE_API_KEY). Transport is WinHTTP on Windows; other
// platforms return an error result for now (no SDK in-tree).
class GeminiProvider : public LlmProvider {
 public:
  explicit GeminiProvider(std::string api_key);

  // Returns GEMINI_API_KEY (or GOOGLE_API_KEY), or "" if unset/empty.
  static std::string KeyFromEnv();

  LlmResult Send(const std::string& system,
                 const std::vector<LlmMessage>& messages,
                 const std::vector<ToolDef>& tools,
                 const ToolExecutor& exec) override;

  const char* name() const override { return "Gemini"; }

  // Accepts "flash"/"pro", "gemini" (-> default flash), or a full "gemini-..."
  // id. Returns the resolved id, or "".
  std::string SetModel(const std::string& id_or_alias) override;
  std::string Model() const override { return model_; }

 private:
  std::string api_key_;
  std::string model_ = "gemini-2.5-flash";
  int max_tokens_ = 8192;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_GEMINI_H_
