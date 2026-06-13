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
#include <vector>

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

bool IsSearchableFile(const fs::path& p) {
  const std::string ext = ToLower(p.extension().string());
  return ext == ".cc" || ext == ".cpp" || ext == ".h" || ext == ".hpp" ||
         ext == ".xml" || ext == ".urdf" || ext == ".mjcf" ||
         ext == ".xacro" || ext == ".txt";
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

std::string GrepSource(const std::string& pattern, const std::string& extra_dir,
                       int max_results) {
  if (pattern.empty()) return "(empty pattern)";

  std::vector<std::string> roots;
  if (const std::string src = MUJOCO_STUDIO_SOURCE_DIR; !src.empty()) {
    roots.push_back(src);
  }
  if (!extra_dir.empty()) roots.push_back(extra_dir);
  if (roots.empty()) return "(no search roots available)";

  const std::string needle = ToLower(pattern);
  std::string out;
  int count = 0;
  std::error_code ec;

  for (const std::string& root : roots) {
    fs::recursive_directory_iterator it(root, ec), end;
    if (ec) continue;  // skip a root we can't open
    for (; it != end; it.increment(ec)) {
      if (ec) break;
      const fs::path& p = it->path();
      if (!fs::is_regular_file(p, ec) || !IsSearchableFile(p)) continue;

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
  }

  if (count == 0) return "No matches for \"" + pattern + "\".";
  return out;
}

}  // namespace mujoco::studio
