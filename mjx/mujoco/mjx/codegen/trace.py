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
"""Static AST tracing to find MuJoCo Model and Data field usages."""

import ast
import dataclasses
import functools
import importlib
import importlib.util
import os
from typing import Dict, Optional, Sequence, Set, Tuple

from absl import logging
from etils import epath
from mujoco.mjx.codegen import file


def _get_imported_module_names(fpath: epath.Path) -> Sequence[Tuple[str, str]]:
  """Returns set of (fully qualified module_name, alias) tuples."""
  module_names = set()
  tree = ast.parse(fpath.read_text(), filename=fpath)

  for node in ast.walk(tree):
    if isinstance(node, ast.Import):
      for alias in node.names:
        module_names.add((alias.name, alias.name))
    elif isinstance(node, ast.ImportFrom):
      if node.module:
        for name in node.names:
          name_ = name.name if not name.asname else name.asname
          module_names.add((f'{node.module}.{name.name}', f'{name_}'))
  return list(module_names)


def _resolve_module_name_to_fpath(fully_qualified_name: str) -> Optional[str]:
  """Resolves a fully qualified name to its file path using importlib."""
  name_parts = fully_qualified_name.split('.')
  for i in range(len(name_parts), 0, -1):
    module_name_to_try = '.'.join(name_parts[:i])
    try:
      spec = importlib.util.find_spec(module_name_to_try)
      if spec and spec.origin and spec.origin != 'built-in':
        return os.path.abspath(spec.origin)
    except (ModuleNotFoundError, ImportError):
      continue


def _get_imported_module_fpaths(fpath: epath.Path) -> Dict[str, str]:
  """Returns the file paths of all imported modules."""
  all_imported_names = _get_imported_module_names(fpath)

  all_resolved_fpaths = {}
  for fully_qualified_name, alias in all_imported_names:
    fpath = _resolve_module_name_to_fpath(fully_qualified_name)
    if fpath:
      all_resolved_fpaths[alias] = fpath
  return all_resolved_fpaths


@dataclasses.dataclass
class FieldInfo:
  param_source: str
  expected_type: str
  param_order: tuple[int, str]


class _FunctionFieldUsageVisitor(ast.NodeVisitor):
  """AST visitor to find attribute usages on 'm' and 'd' variables."""

  def __init__(
      self,
      current_fpath: epath.Path,
      visited_fns: Set[Tuple[str, str]],
      mjwarp_field_info: Dict[str, FieldInfo],
  ):
    self.model_fields = set()
    self.data_fields = set()
    self.data_out_fields = set()
    self._current_fpath = current_fpath.as_posix()
    self._visited_fns = visited_fns
    self._mjwarp_field_info = mjwarp_field_info
    self._module_fpaths = _get_imported_module_fpaths(current_fpath)
    self._in_outputs_context = False

  def visit_FunctionDef(self, node: ast.FunctionDef):
    """Visits nested function definitions."""
    self.generic_visit(node)

  def add_field_usage(self, node: ast.Attribute, is_output: bool):
    """Adds field to the appropriate set."""
    attr_parts = []
    curr_node = node
    while isinstance(curr_node, ast.Attribute):
      attr_parts.append(curr_node.attr)
      curr_node = curr_node.value

    if isinstance(curr_node, ast.Name):
      attr_parts.reverse()
      full_attribute_str = '__'.join(attr_parts)
      in_field_info = full_attribute_str in self._mjwarp_field_info
      if in_field_info:
        if curr_node.id == 'm':
          self.model_fields.add(full_attribute_str)
        if curr_node.id == 'd':
          self.data_fields.add(full_attribute_str)
        if curr_node.id == 'd' and is_output:
          self.data_out_fields.add(full_attribute_str)

  def visit_Attribute(self, node: ast.Attribute):
    self.add_field_usage(node, self._in_outputs_context)
    self.generic_visit(node)

  def visit_keyword(self, node: ast.keyword):
    """Visit a keyword argument node (e.g., outputs=[...])."""
    previous_in_outputs_context = self._in_outputs_context
    if node.arg == 'outputs':
      self._in_outputs_context = True
    try:
      self.generic_visit(node)
    finally:
      self._in_outputs_context = previous_in_outputs_context

  def recurse_trace(self, next_fpath: str, called_fn_name: str):
    """Recursively trace into a function."""
    try:
      field_usage = trace_function(
          next_fpath,
          called_fn_name,
          self._mjwarp_field_info,
          self._visited_fns,
      )
      self.model_fields.update(field_usage.model_fields)
      self.data_fields.update(field_usage.data_fields)
      self.data_out_fields.update(field_usage.data_out_fields)
    except ValueError as e:
      logging.warning(
          'Could not trace function %s in %s: %s',
          called_fn_name,
          next_fpath,
          e,
      )

  def visit_Call(self, node: ast.Call):
    """Visit a function call node and recursively find all attribute usages."""
    if isinstance(node.func, ast.Name):
      called_fn_name = node.func.id
      key = (hash(self._current_fpath), called_fn_name)
      if key not in self._visited_fns:
        self._visited_fns.add(key)
        next_fpath = self._module_fpaths.get(
            called_fn_name, self._current_fpath
        )
        self.recurse_trace(next_fpath, called_fn_name)
    elif isinstance(node.func, ast.Attribute):
      parts = []
      current = node.func
      while isinstance(current, ast.Attribute):
        parts.append(current.attr)
        current = current.value
      if isinstance(current, ast.Name):
        parts.append(current.id)
      parts = parts[::-1]

      if len(parts) == 2 and parts[0] == 'wp' and parts[1] == 'copy':
        if len(node.args) != 2:
          raise ValueError(f'wp.copy() must have 2 arguments, got {node.args}.')
        out_node, in_node = node.args
        self.add_field_usage(out_node, True)
        self.add_field_usage(in_node, False)
        return

      for arg in node.args:
        if isinstance(arg, ast.Attribute):
          self.add_field_usage(arg, is_output=True)

      called_fn_name = '.'.join(parts[1:])
      key = (hash(self._current_fpath), called_fn_name)
      next_fpath = self._module_fpaths.get(parts[0])
      if next_fpath and key not in self._visited_fns:
        self._visited_fns.add(key)
        self.recurse_trace(next_fpath, called_fn_name)

    self.generic_visit(node)


@dataclasses.dataclass
class FieldUsage:
  model_fields: Sequence[str] = dataclasses.field(default_factory=list)
  data_fields: Sequence[str] = dataclasses.field(default_factory=list)
  data_out_fields: Sequence[str] = dataclasses.field(default_factory=list)
  render_context_in_caller: bool = False


def trace_function(
    fpath: str,
    fn: str,
    mjwarp_field_info: Dict[str, FieldInfo],
    visited_fns: Set[Tuple[str, str]] | None = None,
) -> FieldUsage:
  """Traces the function statically to find usages of model and data fields."""
  base_path = file.get_base_path()
  fpath = base_path / fpath
  logging.info('Tracing function: "%s" in "%s"', fn, fpath)

  src = fpath.read_text()
  parsed_ast = ast.parse(src, filename=str(fpath))

  target_fn_nodes = (
      node
      for node in parsed_ast.body
      if isinstance(node, ast.FunctionDef) and node.name == fn
  )
  target_fn_node = next(target_fn_nodes, None)
  if not target_fn_node:
    raise ValueError(f'Function "{fn}" not found in "{fpath}".')

  args_node = target_fn_node.args
  args_tuple = tuple(
      map(functools.partial(ast.get_source_segment, src), args_node.args)
  )
  check_args_tuple = (
      ('m: Model', 'd: Data'),
      ('m: types.Model', 'd: types.Data'),
      ('m', 'd'),
      ('m: Model',),
      ('m: types.Model',),
      ('m',),
  )
  if (
      args_tuple[:2] not in check_args_tuple
      and args_tuple[:1] not in check_args_tuple
  ):
    raise ValueError(
        f'Function "{fn}" in "{fpath}" must have arguments in'
        f' {check_args_tuple}  got {args_tuple}.'
    )

  if visited_fns is None:
    visited_fns = set()

  visitor = _FunctionFieldUsageVisitor(fpath, visited_fns, mjwarp_field_info)
  for body in target_fn_node.body:
    visitor.visit(body)

  render_context_in_caller = False
  if len(target_fn_node.args.args) > 2:
    third_param = target_fn_node.args.args[2]
    if third_param.annotation:
      annotation_str = ast.unparse(third_param.annotation)
      render_context_in_caller = 'RenderContext' in annotation_str

  logging.info(
      'End trace function "%s". Output fields: %s, RenderContext: %s',
      fn, visitor.data_out_fields, render_context_in_caller
  )
  return FieldUsage(
      model_fields=sorted(list(visitor.model_fields)),
      data_fields=sorted(list(visitor.data_fields)),
      data_out_fields=sorted(list(visitor.data_out_fields)),
      render_context_in_caller=render_context_in_caller,
  )


def get_mjwarp_field_info(src: str, get_cls_type_annotations) -> Dict[str, FieldInfo]:
  """Return field info for mujoco_warp/_src/types.py."""
  dataclass_map = {
      'opt': 'Option',
      'stat': 'Statistic',
      'efc': 'Constraint',
      'contact': 'Contact',
  }
  field_info = {}
  type_classes = get_cls_type_annotations(src)
  for field, typ in type_classes['Model'].items():
    if field == 'callback':
      continue
    if field in field_info:
      raise AssertionError(f'Field {field} is duplicated in Model.')
    if field in dataclass_map:
      for sfield, styp in type_classes[dataclass_map[field]].items():
        field_name = field + '__' + sfield
        field_info[field_name] = FieldInfo('Model', styp, (1, field_name))
    else:
      field_info[field] = FieldInfo('Model', typ, (0, field))

  for field, typ in type_classes['Data'].items():
    if field in field_info:
      raise AssertionError(f'Field {field} is duplicated.')
    if field in dataclass_map:
      for sfield, styp in type_classes[dataclass_map[field]].items():
        field_name = field + '__' + sfield
        field_info[field_name] = FieldInfo('Data', styp, (3, field_name))
    else:
      field_info[field] = FieldInfo('Data', typ, (2, field))
  return field_info


def get_mjx_warp_field_info(src: str, get_cls_type_annotations) -> Dict[str, FieldInfo]:
  """Return field info for mjx/warp/types.py."""
  field_info = {}
  type_classes = get_cls_type_annotations(src)
  for field, typ in type_classes['DataWarp'].items():
    field_info[field] = FieldInfo('Data', typ, (0, field))
  for field, typ in type_classes['ModelWarp'].items():
    field_info[field] = FieldInfo('Model', typ, (1, field))
  return field_info
