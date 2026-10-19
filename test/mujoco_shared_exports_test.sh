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

# Checks that the shared MuJoCo library exports MuJoCo's API and nothing else:
# every function and variable mujoco.h declares with MJAPI, and otherwise only
# mj* symbols (such as the private _mj* functions used by the Python bindings),
# so that engine internals and the bundled third-party libraries stay hidden.
#
# Usage: mujoco_shared_exports_test.sh [--exclude=REGEX] <exports> <mujoco.h>
#
# <exports> lists the library's exported symbols, one per line. --exclude skips
# API names matching REGEX that the library isn't expected to export.

set -euo pipefail

exclude=""
positional=()
for arg in "$@"; do
  case "${arg}" in
    --exclude=*) exclude="${arg#--exclude=}" ;;
    *) positional+=("${arg}") ;;
  esac
done
exports="${positional[0]}"
header="${positional[1]}"

symbols="$(sort -u "${exports}")"

# Ignore the __start_*/__stop_* section bounds the linker itself adds for named
# sections in linked-in code (e.g. malloc hooks); they aren't part of the API.
# Also allow C++ symbols in MuJoCo's own namespace (_ZN6mujoco..., _ZZN6mujoco...):
# some toolchains can't hide inline and thread-local statics with a linker
# script (GCC 13 on Ubuntu 24.04 exports two), and unlike third-party symbols
# they can't clash with other libraries.
unexpected="$(grep -v -E '^_?mj' <<< "${symbols}" |
  grep -v -E '^__(start|stop)_' |
  grep -v -E '^_ZZ?N6mujoco' || true)"
if [[ -n "${unexpected}" ]]; then
  echo "FAIL: the library exports symbols other than mj* and _mj*:"
  echo "${unexpected}" | head -50
  exit 1
fi

# The names mujoco.h declares with MJAPI, outside of conditional blocks (other
# than its include guard), e.g. "MJAPI void mj_step(...);" or
# "MJAPI extern void* (*mju_user_malloc)(size_t);".
api="$(awk '
  /^#[ \t]*if/ { depth++ }
  /^#[ \t]*endif/ { depth-- }
  depth == 1 && /^MJAPI/ {
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
    print name
  }' "${header}" | sort -u)"
if [[ -n "${exclude}" ]]; then
  api="$(grep -v -E "${exclude}" <<< "${api}" || true)"
fi

# Guard against a broken header parse.
if [[ "$(wc -l <<< "${api}")" -lt 100 ]]; then
  echo "FAIL: found only $(wc -l <<< "${api}") MJAPI names in ${header}"
  exit 1
fi

missing="$(comm -23 <(echo "${api}") <(echo "${symbols}"))"
if [[ -n "${missing}" ]]; then
  echo "FAIL: the library doesn't export these MJAPI names from mujoco.h:"
  echo "${missing}" | head -50
  exit 1
fi

echo "PASS: the library exports $(wc -l <<< "${symbols}") symbols, all mj* or" \
  "_mj*, including all $(wc -l <<< "${api}") MJAPI names checked from mujoco.h"
