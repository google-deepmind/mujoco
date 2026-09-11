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
# ==============================================================================
"""Unified resource loader for MuJoCo documentation and schema generators.

Provides environment-agnostic file path resolution across test runfiles and
open-source repository checkouts without requiring comment scrubbing.
"""

import os
import pathlib
import sys
from typing import Union

# Centralized path bootstrap for doc generation scripts and tests.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _dir in (
    _REPO_ROOT,
    os.path.join(_REPO_ROOT, "doc", "ext"),
    os.path.join(_REPO_ROOT, "doc", "generate"),
):
  if os.path.isdir(_dir) and _dir not in sys.path:
    sys.path.insert(0, _dir)


def find_repo_root() -> pathlib.Path:
  """Discovers the MuJoCo repository root directory."""
  curr = pathlib.Path(os.path.abspath(__file__)).parent
  for parent in [curr] + list(curr.parents):
    if (parent / ".git").exists() or (parent / "WORKSPACE").exists() or (parent / "METADATA").exists():
      return parent
  if len(curr.parents) >= 2:
    return curr.parents[1]
  return curr


def is_monorepo() -> bool:
  """Returns True if running in a monorepo workspace."""
  return (find_repo_root() / "METADATA").exists() or "TEST_SRCDIR" in os.environ


def resolve_path(relative_path: Union[str, pathlib.Path]) -> pathlib.Path:
  """Resolves a file path across internal and OSS directory structures."""
  path_str = str(relative_path).strip("/")
  basename = os.path.basename(path_str)

  # Check relative to repository root
  root = find_repo_root()
  normalized = path_str
  for prefix in ("third_party/mujoco/",):
    if normalized.startswith(prefix):
      normalized = normalized[len(prefix):]

  fs_candidates = [
      root / path_str,
      root / normalized,
      root / normalized.replace("include/mujoco/", "include/"),
      root / normalized.replace("include/", "include/mujoco/"),
      root / "include" / basename,
      root / "include" / "mujoco" / basename,
  ]
  for candidate in fs_candidates:
    if candidate.exists():
      return candidate.resolve()

  # Check TEST_SRCDIR for test runfiles if available
  test_srcdir = os.environ.get("TEST_SRCDIR")
  if test_srcdir:
    for search_root in (pathlib.Path(test_srcdir),):
      for match in search_root.glob(f"**/{basename}"):
        if match.is_file():
          return match.resolve()

  # Fallback to direct path
  return pathlib.Path(path_str).resolve()


def read_text(relative_path: Union[str, pathlib.Path], encoding: str = "utf-8") -> str:
  """Reads and returns the text content of a resolved resource file."""
  resolved = resolve_path(relative_path)
  return resolved.read_text(encoding=encoding)
