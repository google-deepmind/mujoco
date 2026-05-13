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
"""Generate types for MJX warp integration."""

import ast
import dataclasses
import enum
import logging
import typing
from typing import Any, Callable, Dict, List, Optional, Set

from absl import app
from absl import flags
from etils import epath
import mujoco
from mujoco.mjx.codegen import file
import mujoco.mjx.third_party.mujoco_warp as mjwarp
import numpy as np
import warp as wp
from mujoco.mjx.third_party.warp._src.jax_experimental import ffi


_MJX_WARP_TYPES_OUT_FPATH = flags.DEFINE_string(
    'mjx_warp_types_out_path',
    'third_party/py/mujoco/mjx/warp/types.py',
    'Path to write the mjWarp types into.',
)

_MJX_TYPES_PATH = flags.DEFINE_string(
    'mjx_types_path',
    'third_party/py/mujoco/mjx/_src/types.py',
    'Path to read the MJX types from.',
)

_DATA_SHAPE_PROPERTY_FIELD = 'cacc'
_DUMMY_XML = """
  <mujoco>
    <worldbody>
      <body>
        <joint type="free"/>
        <geom pos="0 0 0" size="0.2" type="sphere"/>
      </body>
      <body >
        <joint type="free"/>
        <geom pos="0 0.3 0" size="0.11" type="sphere"/>
      </body>
    </worldbody>
  </mujoco>
"""


def _to_py_string(value, indent=0):
  """Converts a dictionary/set/tuple/type to a Python code string."""
  indent_str = '  ' * indent
  next_indent_str = '  ' * (indent + 1)
  if isinstance(value, tuple):
    items = [_to_py_string(item, indent) for item in value]
    return f'({', '.join(items)})'

  if isinstance(value, type):
    if value.__module__ == 'builtins':
      return value.__name__
    return f'{value.__module__}.{value.__name__}'

  if isinstance(value, dict):
    items = [
        f'\n{next_indent_str}{repr(k)}: {_to_py_string(v, indent + 1)}'
        for k, v in sorted(value.items(), key=lambda x: x[0])
    ]
    return f'{{{','.join(items)}\n{indent_str}}}'

  if isinstance(value, set):
    items = sorted([_to_py_string(item, indent) for item in value])
    items = [f'\n{next_indent_str}{item}' for item in items]
    return f'{{{",".join(items)}\n{indent_str}}}'

  return repr(value)


def _ast_parse_type(type_repr: str) -> ast.expr:
  """Parses a string representation of a type into an AST node."""
  try:
    return ast.parse(type_repr, mode='eval').body
  except SyntaxError as e:
    raise ValueError(f'Failed to parse type repr "{type_repr}": {e}') from e


def _get_target_annotation_node(
    key: str,
    target_annotations: Dict[str, Any],
) -> ast.expr:
  """Determines the AST node for the target type annotation for MJX."""
  annotation = target_annotations.get(key)
  if annotation == np.ndarray:
    return _ast_parse_type('np.ndarray')

  if (isinstance(annotation, wp.array) or
      type(annotation).__name__ == '_ArrayAnnotation'):
    return _ast_parse_type('jax.Array')

  if annotation in (int, float, bool):
    return _ast_parse_type(annotation.__name__)

  if annotation is ffi.GraphMode:
    return _ast_parse_type('GraphMode')

  if isinstance(annotation, type) and issubclass(annotation, enum.Enum):
    return _ast_parse_type('int')

  if dataclasses.is_dataclass(annotation):
    return _ast_parse_type(annotation.__name__)

  is_tuple = typing.get_origin(annotation) == tuple
  if is_tuple and typing.get_args(annotation)[1] != ...:
    raise NotImplementedError(
        'Only variadic tuples are supported. Got annotation type'
        f' {annotation} for key {key}.'
    )

  if is_tuple and typing.get_args(annotation)[0] in (int, float, bool):
    type_ = typing.get_args(annotation)[0].__name__
    return _ast_parse_type(f'Tuple[{type_}, ...]')

  if is_tuple and (
      isinstance(typing.get_args(annotation)[0], wp.array)
      or type(typing.get_args(annotation)[0]).__name__ == '_ArrayAnnotation'
  ):
    return _ast_parse_type('Tuple[np.ndarray, ...]')

  if is_tuple and dataclasses.is_dataclass(typing.get_args(annotation)[0]):
    cls_type = typing.get_args(annotation)[0].__name__
    return _ast_parse_type(f'Tuple[{cls_type}, ...]')

  raise NotImplementedError(
      f'Unhandled annotation type {annotation} for key {key}.'
  )


def _get_annotations_recursive(
    annotations: Dict[str, Any], prefix: str = ''
) -> Dict[str, Any]:
  """Recursively flattens type annotations, handling nested classes.

  Args:
      annotations: A dictionary of type annotations (field_name: type).
      prefix: The prefix to add to each key, for nested classes.

  Returns:
      A dictionary of flattened annotations (e.g., 'contact__dist').
  """
  flattened = {}
  for key, annotation in annotations.items():
    full_key = f'{prefix}{key}'
    if hasattr(
        annotation, '__annotations__'
    ) and 'mujoco_warp' in annotation.__module__:
      nested = _get_annotations_recursive(
          dict(annotation.__annotations__), prefix=f'{full_key}__'
      )
      flattened.update(nested)
    else:
      flattened[full_key] = annotation  # Leaf node.

  return flattened


def _build_new_class_body_ast(
    keys: Set[str],
    cls_name: str,
    target_annotations: Dict[str, Any],
    shape_property: Optional[str] = None,
    add_docstring: bool = True,
) -> List[ast.AST]:
  """Builds the list of AST nodes for the new class body."""
  new_body_nodes: List[ast.AST] = []

  if add_docstring:
    docstring = f'Derived fields from {cls_name}.'
    new_body_nodes.append(ast.Expr(value=ast.Constant(value=docstring)))

  # Sort keys alphabetically before creating AST nodes
  sorted_keys = sorted(list(keys))
  for key in sorted_keys:
    annotation_node = _get_target_annotation_node(key, target_annotations)

    new_body_nodes.append(
        ast.AnnAssign(
            target=ast.Name(id=key, ctx=ast.Store()),
            annotation=annotation_node,
            simple=1,  # No value assignment
        )
    )

  if shape_property is not None:
    property_string = (
        f'shape = property(lambda self: self.{shape_property}.shape)'
    )
    shape_property_node = ast.parse(property_string).body[0]
    new_body_nodes.append(shape_property_node)

  return new_body_nodes


def _write_class_in_file(
    target_fpath: epath.Path,
    target_cls_name: str,
    target_base_name: str,
    new_body_ast: List[ast.AST],
) -> None:
  """Reads target file, writes the specified class, and saves."""
  target_code = target_fpath.read_text()
  target_tree = ast.parse(target_code)

  if target_cls_name in target_code:
    raise ValueError(
        f'Class {target_cls_name} already exists in file: {target_fpath}'
    )

  new_class_def = ast.ClassDef(
      name=target_cls_name,
      bases=[ast.Name(id=target_base_name, ctx=ast.Load())],
      body=new_body_ast,  # pytype: disable=wrong-arg-types
      decorator_list=[],
      keywords=[],
      type_params=[],
  )
  target_tree.body.append(new_class_def)

  ast.fix_missing_locations(target_tree)
  modified_code = ast.unparse(target_tree)

  logging.info('Writing modified code back to: %s', target_fpath)
  with target_fpath.open('w') as f:
    f.write(modified_code)

  logging.info('File successfully rewritten.')


def write_header(target_fpath: epath.Path):
  """Writes imports and pre-defined class definitions to types.py."""
  header = '''
"""MJX Warp types.
DO NOT EDIT. This file is auto-generated.
"""
import dataclasses
import typing
from typing import Tuple
import jax
from jax import tree_util
from jax.interpreters import batching
from mujoco.mjx._src import dataclasses as mjx_dataclasses
import numpy as np

if typing.TYPE_CHECKING:
  GraphMode = int  # Type alias for pytype.
  @dataclasses.dataclass
  class Callback:
    pass
else:
  try:
    from warp._src.jax_experimental.ffi import GraphMode
    from mujoco.mjx.third_party.mujoco_warp._src import types as mjwp_types
    Callback = mjwp_types.Callback
  except ImportError:
    GraphMode = int  # Fallback when warp not installed.
    Callback = None

PyTreeNode = mjx_dataclasses.PyTreeNode
'''
  target_fpath.write_text(header)


_FLATTEN_UNFLATTEN = """
  # flatten/unflatten all fields for custom jax_callable, but prevent the parent
  # PyTreeNode from putting these fields on device.
  def tree_flatten(self):
    children = list(getattr(self, k) for k in self.__dataclass_fields__)
    return (children, None)
  @classmethod
  def tree_unflatten(cls, aux_data, children):
    del aux_data
    return cls(*children)
"""


def write_nested_dataclass(target_fpath: epath.Path, cls: Any):
  new_class_body = _build_new_class_body_ast(
      set(cls.__annotations__.keys()),
      cls.__name__,
      dict(cls.__annotations__),
      add_docstring=False,
  )
  cls_str = '\n'.join(['  ' + ast.unparse(node) for node in new_class_body])
  cls_str = cls_str.replace('jax.Array', 'np.ndarray')
  with target_fpath.open('a') as f:
    f.write(f'''
@dataclasses.dataclass(frozen=True)
@tree_util.register_pytree_node_class
class {cls.__name__}:
  """{cls.__doc__}"""
{cls_str}
{_FLATTEN_UNFLATTEN}
''')


def _get_meta_fields(cls_name: str) -> Set[str]:
  """Returns the set of fields that should be meta-fields in the pytree."""
  m = mujoco.MjModel.from_xml_string(_DUMMY_XML)
  d = mujoco.MjData(m)
  mujoco.mj_step(m, d)

  if cls_name == 'Data':
    with wp.ScopedDevice('cpu'):
      dw = mjwarp.put_data(m, d, nworld=113, naconmax=1113, njmax=1113)
    cond_fn = lambda x: x.shape[0] not in {113, 1113}
    dw_meta = _get_fields_with_cond(dw, cond_fn)
    data_non_vmap = _get_non_vmap_data_fields()
    return {k for k in dw_meta if k not in data_non_vmap}

  with wp.ScopedDevice('cpu'):
    mw = mjwarp.put_model(m)
  cond_fn = lambda x: not hasattr(x, '_is_batched')
  mw_meta = _get_fields_with_cond(mw, cond_fn)
  if cls_name == 'Model':
    return mw_meta

  if cls_name == 'Option':
    return {k[len('opt__') :] for k in mw_meta if k.startswith('opt')}

  if cls_name == 'Statistic':
    return {k[len('stat__') :] for k in mw_meta if k.startswith('stat')}

  raise NotImplementedError(f'Unhandled class name {cls_name}.')


def write_core_cls(
    cls_name: str,
    target_fpath: epath.Path,
    mjx_types_fpath: epath.Path,
    flatten_fields: bool = False,
    set_diff: bool = True,
    extra_annotations: dict[str, type] | None = None,
):
  """Writes a core API class (e.g. Model/Data/Option/Statistic)."""
  cls = {
      'Model': mjwarp.Model,
      'Data': mjwarp.Data,
      'Option': mjwarp.Option,
      'Statistic': mjwarp.Statistic,
  }[cls_name]

  annotations = dict(cls.__annotations__)  # pytype: disable=attribute-error
  if flatten_fields:
    annotations = _get_annotations_recursive(annotations)

  meta_fields = _get_meta_fields(cls_name)
  for k, v in annotations.items():
    if k not in meta_fields:
      continue
    if isinstance(v, wp.array) or type(v).__name__ == '_ArrayAnnotation':
      annotations[k] = np.ndarray

  warp_keys = annotations.keys()
  mjx_annotations = file.get_cls_type_annotations(mjx_types_fpath.read_text())[
      cls_name
  ]

  keys = warp_keys
  if set_diff:
    # Take the set difference between warp and mjx annotation keys.
    keys = warp_keys - mjx_annotations.keys()

  if extra_annotations:
    annotations.update(extra_annotations)
    keys = set(keys) | extra_annotations.keys()

  if not keys:
    raise ValueError('No derived keys found')

  shape_property = None
  if cls_name == 'Data':
    shape_property = _DATA_SHAPE_PROPERTY_FIELD

  new_class_body = _build_new_class_body_ast(
      keys,
      cls_name,
      annotations,
      shape_property=shape_property,
  )

  _write_class_in_file(
      target_fpath=target_fpath,
      target_cls_name=f'{cls_name}Warp',
      target_base_name='PyTreeNode',
      new_body_ast=new_class_body,
  )


def _get_fields_with_cond(
    d: Any,
    cond_fn: Callable[[Any], bool],
    s: Optional[Set[str]] = None,
    prefix: str = '',
    not_in: bool = False,
    add_static: bool = False,
) -> Set[str]:
  """Recursively finds fields given a condition on the leading dimensions."""
  if s is None:
    s = set()
  for f in dataclasses.fields(d):
    attr = getattr(d, f.name)
    if dataclasses.is_dataclass(f.type):
      s = _get_fields_with_cond(attr, cond_fn, s, f.name + '__', not_in)
      continue
    if f.type in (int, float, bool) and add_static:
      s.add(prefix + f.name)
      continue
    if not (isinstance(f.type, wp.array) or
            type(f.type).__name__ == '_ArrayAnnotation'):
      continue
    if cond_fn(attr):
      s.add(prefix + f.name)
  return s


def _get_non_vmap_data_fields() -> Set[str]:
  """Returns the fields that are not be vmapped but are still jax.Array."""
  m = mujoco.MjModel.from_xml_string(_DUMMY_XML)
  d = mujoco.MjData(m)
  mujoco.mj_step(m, d)

  dw = mjwarp.put_data(m, d, nworld=113, naconmax=1113, njmax=1113)
  cond_fn = lambda x: x.shape[0] not in {113}
  non_vmap = _get_fields_with_cond(dw, cond_fn, add_static=True)
  return non_vmap


def write_register_vmappable(target_fpath: epath.Path):
  """Writes register_vmappable to types.py."""
  data_non_vmap = _get_non_vmap_data_fields()
  with target_fpath.open('a') as f:
    f.write('\nDATA_NON_VMAP =' + _to_py_string(data_non_vmap))
    f.write("""\n
def _to_elt(cont, _, d, axis):
  return DataWarp(**{f.name: cont(getattr(d, f.name), axis)
                     if f.name not in DATA_NON_VMAP
                     else getattr(d, f.name) for f in DataWarp.fields()})
def _from_elt(cont, axis_size, d, axis_dest):
  return DataWarp(**{f.name: cont(axis_size, getattr(d, f.name), axis_dest)
                     if f.name not in DATA_NON_VMAP
                     else getattr(d, f.name) for f in DataWarp.fields()})
batching.register_vmappable(DataWarp, int, int, _to_elt, _from_elt, None)
""")


def _is_ffi_compatible(wp_type: Any) -> bool:
  """Returns True if the type is an array, scalar, or variadic tuple."""
  if (isinstance(wp_type, wp.array) or
      type(wp_type).__name__ == '_ArrayAnnotation'):
    return True
  if wp_type in wp._src.types.value_types:
    return True
  if typing.get_origin(wp_type) is tuple:
    return True
  return False


def _to_jax_ndim(name: str, wp_type: Any) -> int:
  if typing.get_origin(wp_type) is tuple:
    if typing.get_args(wp_type)[1] != ...:
      raise NotImplementedError('Only variadic tuples are supported.')
    return -1  # signals that dim should be untouched in downstream code.
  ffi_arg = ffi.FfiArg(name, wp_type)
  return ffi_arg.jax_ndim


def write_ndim_annotations(target_fpath: epath.Path):
  """Writes a dictionary of ndim for every warp field."""
  ndim_annotations = {}
  for cls in [mjwarp.Model, mjwarp.Data, mjwarp.Option, mjwarp.Statistic]:
    ndim_annotations[cls.__name__] = {}
    annotations = _get_annotations_recursive(cls.__annotations__)
    for name, type_ in annotations.items():
      if not _is_ffi_compatible(type_):
        continue
      ndim = _to_jax_ndim(name, type_)
      ndim_annotations[cls.__name__][name] = ndim

  with target_fpath.open('a') as f:
    f.write('\n_NDIM = ' + _to_py_string(ndim_annotations))


def write_nworld_leading_dim(target_fpath: epath.Path):
  """Writes a dictionary of which MJW fields are batched."""
  # TODO(btaba): check that batch fields have MJX jax.Array annotations, and
  # that non-batch fields have np.ndarray annotations. Fail early.

  m = mujoco.MjModel.from_xml_string(_DUMMY_XML)
  d = mujoco.MjData(m)

  batched = {}
  for cls in [mjwarp.Model, mjwarp.Data, mjwarp.Option, mjwarp.Statistic]:
    batched[cls.__name__] = {}

    if cls.__name__ == 'Data':
      with wp.ScopedDevice('cpu'):
        dw = mjwarp.put_data(m, d, nworld=113, naconmax=1113, njmax=1113)
      cond_fn = lambda x: x.shape[0] == 113
      batched_fields = _get_fields_with_cond(dw, cond_fn)
    else:
      with wp.ScopedDevice('cpu'):
        obj = mjwarp.put_model(m)
      if cls.__name__ == 'Option':
        obj = obj.opt
      elif cls.__name__ == 'Statistic':
        obj = obj.stat
      cond_fn = lambda x: hasattr(x, '_is_batched')
      batched_fields = _get_fields_with_cond(obj, cond_fn)

    all_annotations = _get_annotations_recursive(cls.__annotations__)
    for name, type_ in all_annotations.items():
      if not _is_ffi_compatible(type_):
        continue
      batched[cls.__name__][name] = name in batched_fields

  with target_fpath.open('a') as f:
    f.write('\n_BATCH_DIM = ' + _to_py_string(batched))


def main(argv):
  del argv

  base_path = file.get_base_path()
  target_fpath = base_path / _MJX_WARP_TYPES_OUT_FPATH.value
  mjx_types_fpath = base_path / _MJX_TYPES_PATH.value

  write_header(target_fpath)
  # TODO(btaba): consider automated grabbing of nested dataclasses from mjwarp.
  write_nested_dataclass(target_fpath, mjwarp._src.types.TileSet)
  write_nested_dataclass(target_fpath, mjwarp._src.types.BlockDim)

  write_core_cls('Statistic', target_fpath, mjx_types_fpath, set_diff=False)
  write_core_cls(
      'Option', target_fpath, mjx_types_fpath,
      extra_annotations={'graph_mode': ffi.GraphMode},
  )
  write_core_cls('Model', target_fpath, mjx_types_fpath)
  write_core_cls('Data', target_fpath, mjx_types_fpath, flatten_fields=True)
  write_register_vmappable(target_fpath)
  write_ndim_annotations(target_fpath)
  write_nworld_leading_dim(target_fpath)

  file.write_license(target_fpath)
  file.format_file(target_fpath)


if __name__ == '__main__':
  app.run(main)
