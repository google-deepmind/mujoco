// Copyright 2022 DeepMind Technologies Limited
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

#include "engine/engine_crossplatform.h"  // IWYU pragma: keep

#if defined(__APPLE__) && defined(__AVX__)
#include <sys/sysctl.h>

#include <cstdio>
#include <cstring>

namespace {
__attribute__((weak, visibility("default")))
extern "C" void _mj_rosettaError(const char* msg) {
  fprintf(stderr, "%s\n", msg);
  __asm__ __volatile__ ("ud2");  // raises SIGILL but leave this function at the top of the stack
}

__attribute__((constructor(10000), target("no-avx")))
void CheckRosetta() {
  int is_translated = 0;
  {
    size_t len = sizeof(is_translated);
    if (sysctlbyname("sysctl.proc_translated", &is_translated, &len, NULL, 0)) {
      is_translated = 0;
    }
  }
  if (is_translated) {
    _mj_rosettaError("MuJoCo cannot be run under Rosetta 2 on an Apple Silicon machine.");
  }
}
}  // namespace
#endif  // defined(__APPLE__) && defined(__AVX__)

#ifdef ADDRESS_SANITIZER
#include <sanitizer/common_interface_defs.h>

#include <array>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace {
const std::pair<std::string, std::string>&
FuncNameAndDebugInfoCached(void* pc) {
  static const std::unordered_set<std::string>* const kIgnoredInlinedFunctions =
      []() {
        return new std::unordered_set<std::string>{
            "mj_freeStack",
            "mj_markStack",
        };
      }();

  static auto* mu = new std::shared_mutex;
  static auto* pc_to_func_name_map =
      new std::unordered_map<void*, std::pair<std::string, std::string>>;

  {
    std::shared_lock lock(*mu);
    auto it = pc_to_func_name_map->find(pc);
    if (it != pc_to_func_name_map->end()) {
      return it->second;
    }
  }

  std::array<char, 256> buf;
  __sanitizer_symbolize_pc(pc, "%f", buf.data(), buf.size());

  // buf contains sequence of null-terminated strings of inlined function names
  // so we walk through the sequence until we find the first unignored function
  std::string_view func_name(buf.data());
  int idx = 0;
  while (kIgnoredInlinedFunctions->find(func_name.data()) !=
         kIgnoredInlinedFunctions->end()) {
    func_name = func_name.data() + func_name.size() + 1;
    ++idx;
  }

  std::array<char, 1024> buf2;
  __sanitizer_symbolize_pc(pc, "%F at %S", buf2.data(), buf2.size());
  std::string_view debug_info(buf2.data());
  for (int i = 0; i < idx; ++i) {
    debug_info = debug_info.data() + debug_info.size() + 1;
  }

  {
    std::unique_lock lock(*mu);
    return pc_to_func_name_map
        ->emplace(
            pc, std::make_pair(std::string(func_name), std::string(debug_info)))
        .first->second;
  }
}

const std::string& SymbolizeCached(void* pc) {
  return FuncNameAndDebugInfoCached(pc).first;
}

const std::string& DebugInfoCached(void* pc) {
  return FuncNameAndDebugInfoCached(pc).second;
}
}  // namespace

int mj__comparePcFuncName(void* pc1, void* pc2) {
  static auto* mu = new std::shared_mutex;
  static auto* same_func_map = new std::map<std::pair<void*, void*>, bool>;

  auto pc_pair = std::make_pair(pc1, pc2);
  {
    std::shared_lock lock(*mu);
    auto it = same_func_map->find(pc_pair);
    if (it != same_func_map->end()) {
      return it->second;
    }
  }

  bool is_same = (SymbolizeCached(pc1) == SymbolizeCached(pc2));
  {
    std::unique_lock lock(*mu);
    return same_func_map->emplace(pc_pair, is_same).first->second;
  }
}

const char* mj__getPcDebugInfo(void* pc) {
  return DebugInfoCached(pc).c_str();
}
#endif  // ADDRESS_SANITIZER
