#include "third_party/mujoco/wasm/unpack.h"

#include <cstddef>
#include <cstring>
#include <string>

namespace mujoco::wasm {

std::string StripWrapperSuffix(const char* func) {
  const char* suffix = "_wrapper";
  size_t name_len = strlen(func);
  size_t suffix_len = strlen(suffix);

  if (name_len >= suffix_len &&
      strcmp(func + name_len - suffix_len, suffix) == 0) {
    return std::string(func, name_len - suffix_len);
  }
  return std::string(func);
}

}  // namespace mujoco::wasm
