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

#include "experimental/studio/llm/source_search.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#ifndef MUJOCO_STUDIO_SOURCE_DIR
#define MUJOCO_STUDIO_SOURCE_DIR ""
#endif

namespace mujoco::studio {
namespace {

namespace fs = std::filesystem;

std::string ToLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return s;
}

bool IsSourceFile(const fs::path& p) {
  const std::string ext = p.extension().string();
  return ext == ".cc" || ext == ".cpp" || ext == ".h" || ext == ".hpp";
}

std::string Trim(const std::string& s) {
  const size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const size_t e = s.find_last_not_of(" \t\r\n");
  std::string t = s.substr(b, e - b + 1);
  if (t.size() > 200) t = t.substr(0, 200) + "...";
  return t;
}

}  // namespace

std::string GrepSource(const std::string& pattern, int max_results) {
  if (pattern.empty()) return "(empty pattern)";
  const std::string root = MUJOCO_STUDIO_SOURCE_DIR;
  if (root.empty()) return "(source search not available in this build)";

  const std::string needle = ToLower(pattern);
  std::string out;
  int count = 0;
  std::error_code ec;

  fs::recursive_directory_iterator it(root, ec), end;
  if (ec) return "(could not open source dir: " + root + ")";

  for (; it != end; it.increment(ec)) {
    if (ec) break;
    const fs::path& p = it->path();
    if (!fs::is_regular_file(p, ec) || !IsSourceFile(p)) continue;

    std::ifstream f(p);
    if (!f) continue;
    std::string line;
    int lineno = 0;
    while (std::getline(f, line)) {
      ++lineno;
      if (ToLower(line).find(needle) == std::string::npos) continue;
      std::string rel = fs::relative(p, root, ec).string();
      if (ec) rel = p.filename().string();
      out += rel + ":" + std::to_string(lineno) + ": " + Trim(line) + "\n";
      if (++count >= max_results) {
        out += "(truncated at " + std::to_string(max_results) + " matches)\n";
        return out;
      }
    }
  }

  if (count == 0) return "No matches for \"" + pattern + "\".";
  return out;
}

}  // namespace mujoco::studio
