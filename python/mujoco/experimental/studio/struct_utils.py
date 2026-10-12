# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Helpers to copy the fields of MuJoCo structs, e.g. MjOption."""

from typing import TypeVar

_T = TypeVar('_T')


def copy_struct(src: _T, dst: _T) -> None:
  """Copies all fields from ``src`` into ``dst``, a struct of the same type."""
  for field in getattr(type(src), '_all_fields'):
    value = getattr(src, field)
    try:
      getattr(dst, field)[:] = value  # An array field: copy in place.
    except (TypeError, AttributeError):
      setattr(dst, field, value)  # A scalar field.
