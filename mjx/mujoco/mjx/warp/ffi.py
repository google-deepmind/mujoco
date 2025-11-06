# Copyright 2025 DeepMind Technologies Limited
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
"""FFI helper functions for MJX."""

import dataclasses
import functools
import inspect
import typing
from typing import Any, Callable, Optional, Sequence, Tuple, Union

import jax
from jax import numpy as jp
from mujoco.mjx.warp import types as mjx_warp_types
import numpy as np
import warp as wp
from mujoco.mjx.third_party.warp._src.jax_experimental import ffi


def flatten_signature(signature: inspect.Signature, args: Tuple[Any, ...]):
  """Flattens a tuple/dataclass signature."""

  def expand_parameter(parameter, arg_iter):
    p = parameter
    if p.kind != inspect.Parameter.POSITIONAL_OR_KEYWORD:
      raise ValueError(f'Unsupported parameter kind: {p.kind}')

    try:
      arg = next(arg_iter)
    except StopIteration:
      # We ran out input arguments. Let us keep output parameters as is.
      assert typing.get_origin(p.annotation) != tuple, p.annotation
      return [p]

    # If it is a tuple, we need to duplicate the parameter for each element.
    if isinstance(arg, tuple):
      assert typing.get_origin(p.annotation) == tuple, p.annotation
      type_args = typing.get_args(p.annotation)
      if len(type_args) != 2 or type_args[1] != ...:
        raise NotImplementedError(
            f'Unsupported tuple argument: {type_args} '
            '(currently, only Tuple[t, ...] is supported).'
        )
      type_ = type_args[0]
      types = [type_]
      if dataclasses.is_dataclass(type_):
        # If the tuple element is a dataclass, we need to recycle the
        # types in the same order as they appear in the dataclass.
        fields = list(type_.__dataclass_fields__.values())
        types = [f.type for f in fields]
      return [
          inspect.Parameter(
              f'{p.name}__{i}',
              p.kind,
              default=p.default,
              annotation=types[i % len(types)],
          )
          for i in range(len(arg) * len(types))
      ]
    elif dataclasses.is_dataclass(arg):
      assert dataclasses.is_dataclass(p.annotation), p.annotation
      fields = list(arg.__dataclass_fields__.values())
      return [
          inspect.Parameter(
              f'{p.name}__{fields[i].name}',
              p.kind,
              default=p.default,
              annotation=fields[i].type,
          )
          for i in range(len(fields))
      ]

    assert typing.get_origin(p.annotation) != tuple, p.annotation
    return [p]

  parameters = []
  arg_iter = iter(args)
  for p in signature.parameters.values():
    parameters.extend(expand_parameter(p, arg_iter))

  return inspect.Signature(
      parameters=parameters, return_annotation=signature.return_annotation
  )


def jax_callable_variadic_tuple(
    func: Callable,  # pylint: disable=g-bare-generic
    num_outputs: int = 1,
    graph_mode: ffi.GraphMode = ffi.GraphMode.WARP,
    vmap_method: Optional[str] = None,
    output_dims: Optional[dict[str, tuple[int, ...]]] = None,
    in_out_argnames: Optional[Sequence[str]] = None,
):
  """Wraps a JAX callable to support variadic tuples and dataclasses."""

  def callable_wrapper(*args, **kwargs):
    def func_wrapper(*flat_args, **kwargs):
      num_inputs = in_tree.num_leaves
      flat_inputs = flat_args[:num_inputs]
      output_buffers = flat_args[num_inputs:]
      unflat_args = jax.tree.unflatten(in_tree, flat_inputs)
      return func(*unflat_args, *output_buffers, **kwargs)

    # Provide a flattened signature for the Warp callable machinery.
    new_signature = flatten_signature(inspect.signature(func), args)
    func_wrapper.__signature__ = new_signature
    func_wrapper.__annotations__ = {
        p.name: p.annotation
        for p in new_signature.parameters.values()
        if p.annotation is not inspect.Parameter.empty
    }
    if new_signature.return_annotation is not inspect.Signature.empty:
      func_wrapper.__annotations__['return'] = new_signature.return_annotation

    my_callable = ffi.jax_callable(
        func_wrapper,
        num_outputs=num_outputs,
        graph_mode=graph_mode,
        vmap_method=vmap_method,
        output_dims=output_dims,
        in_out_argnames=in_out_argnames,
    )

    flat_args, in_tree = jax.tree.flatten(args)
    return my_callable(*flat_args, **kwargs)

  return callable_wrapper


def _format_arg(arg: Any, name: str, annotation: Any, verbose: bool):
  """Formats a single argument for warp."""
  typ_args = typing.get_args(annotation)
  annotation_origin = typing.get_origin(annotation)

  # Handle variadic tuples.
  if annotation_origin == tuple and len(typ_args) == 2 and typ_args[1] == ...:
    return tuple(
        _format_arg(arg[i], name + f'_{i}', typ_args[0], verbose)
        for i in range(len(arg))
    )

  if not isinstance(annotation, wp.types.array):
    if verbose:
      print(f'Skipping {name}: {arg}')
    return arg

  expected_ndim = annotation.ndim
  if arg.ndim != expected_ndim:
    raise AssertionError(
        f'Arg ndim {arg.ndim} does not match expected ndim {expected_ndim}.'
    )

  # Add stride 0 to first axis in case the underlying argument should be
  # batched.
  # NB: the outer marshalling does an "expand_dims" on Model fields.
  is_batch_field = mjx_warp_types._BATCH_DIM['Model'].get(name, False)  # pylint: disable=protected-access
  if arg.shape[0] == 1 and is_batch_field:
    old_strides = arg.strides
    arg.strides = (0,) + arg.strides[1:]
    if verbose:
      print(
          f'Leading batch dim of 1, adding stride: {name} {old_strides} =>'
          f' {arg.strides}'
      )
    return arg

  if verbose:
    print(f'Did nothing: {name}: {arg.shape}')
  return arg


def format_args_for_warp(func, verbose=False):
  @functools.wraps(func)
  def wrapper(*args):
    args = list(args)
    annotations = func.__annotations__
    assert len(args) == len(annotations)
    for i, (name, annotation) in enumerate(annotations.items()):
      args[i] = _format_arg(args[i], name, annotation, verbose)
    return func(*args)

  return wrapper


def _tree_path_to_attr_str(path: jax.tree_util.KeyPath) -> str:
  """Converts a tree path to a dataclass attribute string."""
  if not isinstance(path, tuple):
    raise NotImplementedError(
        f'Parsing for jax tree path {path} not implemented.'
    )

  if any(isinstance(p, jax.tree_util.SequenceKey) for p in path):
    # get the path up to the first sequence key, we assume variadic sequences
    is_seq_key = [isinstance(p, jax.tree_util.SequenceKey) for p in path]
    path = path[: is_seq_key.index(True)]

  assert all(isinstance(p, jax.tree_util.GetAttrKey) for p in path)
  path = [p for p in path if p.name != '_impl']
  return '__'.join(p.name for p in path)


def _get_mapping_from_tree_path(
    path: jax.tree_util.KeyPath,
    mapping: dict[str, int],
) -> Optional[int]:
  """Gets the mapped value from a tree path."""
  attr = _tree_path_to_attr_str(path)
  # None if the MJX public field is not present in the MJX-Warp mapping.
  return mapping.get(attr)


def _expand_dim_from_path(
    path: jax.tree_util.KeyPath, leaf: Any, ndim_map: dict[str, int]
) -> Any:
  """Expands the dimension of a leaf node based on the ndim_map."""
  ndim = _get_mapping_from_tree_path(path, ndim_map)
  if ndim is None or ndim < 0:
    return leaf
  if ndim > leaf.ndim:
    leaf = jp.expand_dims(leaf, axis=np.arange(ndim - leaf.ndim))
  if ndim != leaf.ndim:
    raise AssertionError(
        f'Leaf node ndim ({leaf.ndim}) and expected ndim ({ndim}) do not match'
        f' for path {path}.'
    )
  return leaf


def _squeeze_dim(leaf_expanded: Any, leaf: Any) -> Any:
  if leaf_expanded.ndim < leaf.ndim:
    raise AssertionError(
        f'Expanded leaf ndim {leaf_expanded.ndim} is smaller than original leaf'
        f' ndim {leaf.ndim}'
    )
  if leaf_expanded.ndim > leaf.ndim:
    return jp.squeeze(leaf_expanded, np.arange(leaf_expanded.ndim - leaf.ndim))
  return leaf_expanded


def marshal_jax_warp_callable(func, raw_output: bool = False):
  """Marshal fields into a MuJoCo Warp function."""

  @functools.wraps(func)
  def wrapper(m, d):
    # Expand dims for Warp implicit vmap before calling into the FFI wrapped
    # function.
    m_expanded = jax.tree.map_with_path(
        lambda path, x: _expand_dim_from_path(
            path, x, mjx_warp_types._NDIM['Model']  # pylint: disable=protected-access
        ),
        m,
    )
    d_expanded = jax.tree.map_with_path(
        lambda path, x: _expand_dim_from_path(
            path, x, mjx_warp_types._NDIM['Data']  # pylint: disable=protected-access
        ),
        d,
    )
    d_expanded_result = func(m_expanded, d_expanded)

    if raw_output:
      return d_expanded_result
    d_result = jax.tree.map(_squeeze_dim, d_expanded_result, d)
    return d_result

  return wrapper


def _flatten_batch_dim(
    path: jax.tree_util.KeyPath, leaf: Any, ndim_map: dict[str, int]
) -> Any:
  ndim = _get_mapping_from_tree_path(path, ndim_map)
  if ndim is None or ndim < 0:
    return leaf
  if ndim < leaf.ndim:
    assert leaf.ndim - ndim == 1
    batch_dim = np.prod(leaf.shape[: leaf.ndim - ndim + 1])
    leaf = jp.reshape(leaf, (batch_dim,) + leaf.shape[leaf.ndim - ndim + 1 :])
  return leaf


def _unflatten_batch_dim(leaf_squeezed: Any, leaf: Any) -> Any:
  if leaf_squeezed.ndim > leaf.ndim:
    raise AssertionError(
        f'Squeezed leaf ndim {leaf_squeezed.ndim} is greater than original leaf'
        f' ndim {leaf.ndim}'
    )
  if leaf_squeezed.ndim < leaf.ndim:
    return leaf_squeezed.reshape(leaf.shape)
  return leaf_squeezed


def _maybe_broadcast_to(
    path: jax.tree_util.KeyPath,
    leaf: Any,
    is_batched: bool,
    axis_size: Union[int, tuple[int, ...]],
    cls_str: str,
) -> Any:
  """Broadcasts fields that are used in MuJoCo Warp."""
  ndim = _get_mapping_from_tree_path(path, mjx_warp_types._NDIM[cls_str])
  needs_batch_dim = _get_mapping_from_tree_path(
      path, mjx_warp_types._BATCH_DIM[cls_str]  # pylint: disable=protected-access
  )
  needs_batch_dim = bool(needs_batch_dim) and (ndim is not None and ndim > 0)
  if needs_batch_dim and not is_batched:
    leaf = jp.broadcast_to(leaf, (axis_size,) + leaf.shape)
  return leaf


def _check_leading_dim(
    path: jax.tree_util.KeyPath,
    leaf: Any,
    expected_batch_dim: int,
    expected_nconmax: int,
    expected_njmax: int,
):
  """Asserts that the batch dimension of a leaf node matches the expected batch dimension."""
  has_batch_dim = _get_mapping_from_tree_path(
      path, mjx_warp_types._BATCH_DIM['Data']
  )
  attr = _tree_path_to_attr_str(path)
  if has_batch_dim and leaf.shape[0] != expected_batch_dim:
    raise ValueError(
        f'Leaf node batch size ({leaf.shape[0]}) and expected batch size'
        f' ({expected_batch_dim}) do not match for field {attr}.'
    )
  if (
      not has_batch_dim
      and attr.startswith('contact__')
      and leaf.shape[0] != expected_nconmax
  ):
    raise ValueError(
        f'Leaf node leading dim ({leaf.shape[0]}) does not match nconmax'
        f' ({expected_nconmax}) for field {attr}.'
    )
  if (
      not has_batch_dim
      and attr.startswith('efc__')
      and leaf.shape[0] != expected_njmax
  ):
    raise ValueError(
        f'Leaf node leading dim ({leaf.shape[0]}) does not match njmax'
        f' ({expected_njmax}) for field {attr}.'
    )


def marshal_custom_vmap(vmap_func, raw_output: bool = False):
  """Marshal fields for a custom vmap into an MuJoCo Warp function."""

  @functools.wraps(vmap_func)
  def wrapper(axis_size, is_batched, m, d):
    # Vmappable data fields may not have been broadcasted if vmap_func is called
    # within a vmap trace. Since data fields are read/write in warp, we need to
    # explicitly broadcast them here.
    d_broadcast = jax.tree.map_with_path(
        lambda path, x, is_b: _maybe_broadcast_to(
            path, x, is_b, axis_size, 'Data'
        ),
        d, is_batched[1],  # fmt: skip
    )
    # Check leading dimensions.
    jax.tree.map_with_path(
        lambda path, x: _check_leading_dim(
            path, x, d_broadcast.qpos.shape[0], d._impl.naconmax, d._impl.njmax  # pylint: disable=protected-access
        ),
        d_broadcast,
    )
    # Flatten batch dims into the first axis if the vmap was nested.
    m_flat = jax.tree.map_with_path(
        lambda path, x: _flatten_batch_dim(
            path, x, mjx_warp_types._NDIM['Model']  # pylint: disable=protected-access
        ),
        m,
    )
    d_broadcast_flat = jax.tree.map_with_path(
        lambda path, x: _flatten_batch_dim(
            path, x, mjx_warp_types._NDIM['Data']  # pylint: disable=protected-access
        ),
        d_broadcast,
    )
    d_broadcast_flat_result, out_batched = vmap_func(
        axis_size, is_batched, m_flat, d_broadcast_flat
    )
    if raw_output:
      return d_broadcast_flat_result, out_batched

    # Explicitly mark MuJoCo Warp data fields as batched after vmapping is done.
    out_batched = jax.tree.map_with_path(
        # NB: if a field is not in MuJoCo Warp, we let JAX do its magic.
        lambda path, x: _get_mapping_from_tree_path(
            path, mjx_warp_types._BATCH_DIM['Data']  # pylint: disable=protected-access
        )
        or x,
        out_batched,
    )
    # Unflatten batch dimensions but keep the broadcasting.
    d_result = jax.tree.map(
        _unflatten_batch_dim, d_broadcast_flat_result, d_broadcast
    )
    return d_result, out_batched

  return wrapper
