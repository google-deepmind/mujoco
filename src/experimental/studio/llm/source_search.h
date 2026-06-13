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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_SOURCE_SEARCH_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_SOURCE_SEARCH_H_

#include <string>

namespace mujoco::studio {

// Case-insensitive substring grep over the Studio C++ source tree (configured
// at build time via MUJOCO_STUDIO_SOURCE_DIR). Returns up to `max_results`
// matches as "relative/path:line: trimmed source line", or a short message if
// nothing matched / the source dir isn't available. Backs the LLM grep_source
// tool, which the agent uses to verify item ids/labels exist before referencing
// them in a UI program (instead of hallucinating refs).
std::string GrepSource(const std::string& pattern, int max_results);

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_SOURCE_SEARCH_H_
