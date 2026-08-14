# Copyright 2022 DeepMind Technologies Limited
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
"""Code generator for function_traits.h."""

from typing import Mapping, Sequence

from absl import app

from introspect import ast_nodes
from introspect import enums

ENUMS: Mapping[str, ast_nodes.EnumDecl] = enums.ENUMS


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  struct_decls = []
  for enum in ENUMS.values():
    value_decls = []
    for k in enum.values:
      value_decls.append(f'std::make_pair("{k}", ::{enum.name}::{k})')
    if len(value_decls) < 2:
      value_decls = ''.join(value_decls)
    else:
      value_decls = '\n    ' + ',\n    '.join(value_decls)
    struct_decls.append(f"""
struct {enum.name} {{
  static constexpr char name[] = "{enum.name}";
  using type = ::{enum.name};
  static constexpr auto values = std::array{{{value_decls}}};
}};
""".strip())

  all_structs = '\n\n'.join(struct_decls)

  all_enum_inits = '\n    ' + '{},\n    '.join(ENUMS.keys()) + '{}'

  print(f"""
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

#ifndef MUJOCO_PYTHON_CODEGEN_ENUM_TRAITS_H_
#define MUJOCO_PYTHON_CODEGEN_ENUM_TRAITS_H_

#include <array>
#include <tuple>
#include <utility>

#include <mujoco/mujoco.h>

namespace mujoco::python_traits {{

{all_structs}

static constexpr auto kAllEnums = std::make_tuple({all_enum_inits});

}}  // namespace mujoco::python_traits

#endif  // MUJOCO_PYTHON_CODEGEN_ENUM_TRAITS_H_
""".lstrip())


if __name__ == '__main__':
  app.run(main)
