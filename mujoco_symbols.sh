#!/bin/bash
# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Writes the list of symbols the shared MuJoCo library exports, like CMake's
# libmujoco: every C API function and variable the headers declare with MJAPI,
# plus the private _mjPRIVATE_* functions the Python bindings use.
#
# Usage: mujoco_symbols.sh lds|exported_symbols <output> <header>...
#
# "lds" writes an ELF version script; "exported_symbols" writes a macOS
# -exported_symbols_list, where names get the extra leading underscore.
#
# Each name is written as a glob that only matches itself (mj_step becomes
# mj_ste[p]), so the linker doesn't fail on names a build doesn't define, such
# as the stack functions that differ with AddressSanitizer.

set -euo pipefail

format="$1"
output="$2"
shift 2

# C API declarations look like "MJAPI void mj_step(...);",
# "MJAPI extern const char* mjDISABLESTRING[...];" or
# "MJAPI extern void* (*mju_user_malloc)(size_t);". Internal C++ headers also
# put MJAPI on a few non-extern-"C" symbols (SetGlobalXmlSpec, mjuu_*); skip
# those, since they are C++-mangled in the object files.
names="$(awk '
  /^MJAPI/ {
    s = $0
    sub(/^MJAPI[ \t]+/, "", s)
    if (match(s, /\(\*[ \t]*[A-Za-z_][A-Za-z0-9_]*/)) {
      name = substr(s, RSTART + 2, RLENGTH - 2)
      gsub(/[ \t]/, "", name)
    } else {
      sub(/[(\[;].*/, "", s)
      name = s
      sub(/.*[^A-Za-z0-9_]/, "", name)
    }
    if (name ~ /^mj/ && name !~ /^mjuu_/) print name
  }' "$@" | sort -u)"

if [[ "$(wc -l <<< "${names}")" -lt 100 ]]; then
  echo "Found only $(wc -l <<< "${names}") MJAPI names in $*" >&2
  exit 1
fi

# mj_step -> mj_ste[p]
patterns="$(sed 's/\(.\)$/[\1]/' <<< "${names}")"

case "${format}" in
  lds)
    {
      echo "{"
      echo "  global:"
      sed 's/^/    /; s/$/;/' <<< "${patterns}"
      echo "    _mjPRIVATE_*;"
      echo "  local: *;"
      echo "};"
    } > "${output}"
    ;;
  exported_symbols)
    {
      sed 's/^/_/' <<< "${patterns}"
      echo "__mjPRIVATE_*"
    } > "${output}"
    ;;
  *)
    echo "Unknown format: ${format}" >&2
    exit 1
    ;;
esac
