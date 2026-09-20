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
"""File tools."""

import ast
import os
import subprocess
from typing import Dict
from absl import logging
from etils import epath

LICENSE_TEXT = """
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
"""


def get_base_path() -> epath.Path:
  """Resolves the base workspace path."""
  base_path = os.environ.get('BUILD_WORKSPACE_DIRECTORY')
  if base_path:
    return epath.Path(base_path)
  # Assume this file is at <root>/mujoco/mjx/codegen/file.py.
  root = epath.Path(os.path.abspath(__file__)).parents[3]
  if not (root / 'mujoco' / 'mjx' / 'codegen').is_dir():
    raise RuntimeError(
        f'Unexpected codegen layout, resolved root: {root}'
    )
  return root


def format_file(target_fpath: epath.Path):
  """Formats a Python file."""
  logging.info('Running pyink on: %s', target_fpath)
  subprocess.run(
      ['pyink', str(target_fpath)],
      check=True,
      text=True,
      capture_output=True,
  )
  logging.info('Running isort on: %s', target_fpath)
  subprocess.run(
      ['isort', str(target_fpath)],
      check=True,
      text=True,
      capture_output=True,
  )


def write_license(target_fpath: epath.Path):
  """Writes license to the target file."""
  src = target_fpath.read_text()
  target_fpath.write_text(LICENSE_TEXT + src)


def get_cls_type_annotations(src: str) -> Dict[str, Dict[str, str]]:
  """Return classes with their field annotation strings from source code."""
  ret = {}
  tree = ast.parse(src)

  class Visitor(ast.NodeVisitor):

    def visit_ClassDef(self, node: ast.ClassDef):  # pylint: disable=invalid-name
      class_name = node.name
      ret[class_name] = {}
      for item in node.body:
        if not isinstance(item, ast.AnnAssign):
          continue
        field_name = item.target.id  # pytype: disable=attribute-error
        annotation_str = ast.unparse(item.annotation).strip()
        ret[class_name][field_name] = annotation_str

  Visitor().visit(tree)
  return ret
