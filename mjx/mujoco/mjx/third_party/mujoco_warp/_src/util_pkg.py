# Copyright 2026 The Newton Developers
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
"""Package version checking utilities."""

import importlib.metadata
import operator
import re


def _parse_version(version_str: str) -> tuple[tuple[int, int | str], ...]:
  """Parse a version string into comparable components.

  Both '.' and '-' are treated as separators. Each component is wrapped in a
  tuple: (0, int) for numeric parts, (-1, str) for non-numeric. A (0, 0)
  sentinel is appended so that stable releases sort above pre-release suffixes
  during Python tuple comparison (e.g., 1.2.3 >= 1.2.3.dev). Non-numeric
  components are compared lexicographically (e.g., b >= a).

  Args:
    version_str: Version string like "3.5.0" or "3.5.0.dev869102767".

  Returns:
    Tuple of (type_order, value) pairs for comparison, where type_order is 0
    for integers and -1 for strings, followed by a (0, 0) sentinel.
  """
  # Split on both '.' and '-'
  parts = re.split(r"[.\-]", version_str)
  return tuple([(0, int(p)) if p.isdigit() else (-1, p) for p in parts] + [(0, 0)])


def check_version(spec: str) -> bool:
  """Check if an installed package satisfies a version requirement.

  Supports operators: >=, <=, >, <, ==, !=

  Version comparison rules:
  - Both '.' and '-' are treated as separators
  - Numeric components are compared numerically
  - Non-numeric components are compared lexicographically
  - Stable releases are greater than pre-releases (e.g., 1.2.3 >= 1.2.3.dev)

  Args:
    spec: Version specification like "numpy>=1.20.0".

  Returns:
    True if the installed version satisfies the requirement.

  Raises:
    ValueError: If the spec cannot be parsed.
    importlib.metadata.PackageNotFoundError: If the package is not installed.
  """
  match = re.match(r"^([a-zA-Z0-9_\-]+)(>=|<=|>|<|==|!=)(.+)$", spec)
  if not match:
    raise ValueError(f"Invalid version spec '{spec}'. Expected format: 'package>=version'")
  package_name, op, version_str = match.groups()

  required_version = _parse_version(version_str)

  try:
    installed_str = importlib.metadata.version(package_name)
  except importlib.metadata.PackageNotFoundError as e:
    # Fallback: import the package and read __version__
    try:
      import importlib as _importlib  # noqa: F811

      mod = _importlib.import_module(package_name)
      installed_str = mod.__version__
    except (ImportError, AttributeError):
      raise e

  installed_version = _parse_version(installed_str)

  ops = {
    ">=": operator.ge,
    "<=": operator.le,
    ">": operator.gt,
    "<": operator.lt,
    "==": operator.eq,
    "!=": operator.ne,
  }
  return ops[op](installed_version, required_version)
