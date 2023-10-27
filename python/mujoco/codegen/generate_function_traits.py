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

import keyword
from typing import Mapping, Sequence

from absl import app

from introspect import ast_nodes
from introspect import functions

FUNCTIONS: Mapping[str, ast_nodes.FunctionDecl] = functions.FUNCTIONS


def _sanitize_keyword(s: str) -> str:
  if keyword.iskeyword(s):
    return s + '_'
  return s


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  struct_decls = []
  for func in FUNCTIONS.values():
    # Skip mju_error_{i,s} and mju_warning_{i,s} as these are not
    # supported in the Python bindings, and Introspect currently
    # doesn't support variadic functions.
    if func.name.startswith('mju_error') or func.name == 'mju_warning':
      continue

    # Modify some parameter types.
    parameters = []
    modified = False
    for p in func.parameters:
      # Expose array parameters as pointer-to-arrays so that we can determine
      # array extents in C++ templates.
      if isinstance(p.type, ast_nodes.ArrayType):
        parameters.append(ast_nodes.FunctionParameterDecl(
            name=p.name, type=ast_nodes.PointerType(
                ast_nodes.ArrayType(
                    inner_type=p.type.inner_type, extents=p.type.extents))))
        modified = True
      else:
        parameters.append(p)

    if modified:
      func = ast_nodes.FunctionDecl(
          name=func.name, return_type=func.return_type,
          parameters=parameters, doc=func.doc)
      getfunc = f'*reinterpret_cast<type*>(&::{func.name})'
    else:
      getfunc = f'::{func.name}'

    param_names = ', '.join(
        f'"{_sanitize_keyword(p.name)}"' for p in parameters
    )

    struct_decls.append(f"""
struct {func.name} {{
  static constexpr char name[] = "{func.name}";
  static constexpr char doc[] = "{func.doc}";
  using type = {func.decltype};
  static constexpr auto param_names = std::make_tuple({param_names});

  MUJOCO_ALWAYS_INLINE static type& GetFunc() {{
    return {getfunc};
  }}
}};
""".strip())

  all_structs = '\n\n'.join(struct_decls)

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

#ifndef MUJOCO_PYTHON_CODEGEN_FUNCTION_TRAITS_H_
#define MUJOCO_PYTHON_CODEGEN_FUNCTION_TRAITS_H_

#include <tuple>

#include <mujoco/mujoco.h>
#include "util/crossplatform.h"

namespace mujoco::python_traits {{

{all_structs}

}}  // namespace mujoco::python_traits

#endif  // MUJOCO_PYTHON_CODEGEN_FUNCTION_TRAITS_H_
""".lstrip())


if __name__ == '__main__':
  app.run(main)
