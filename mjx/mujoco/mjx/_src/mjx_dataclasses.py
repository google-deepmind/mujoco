# Copyright 2023 DeepMind Technologies Limited
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
"""Wrapper that automatically registers dataclass as a Jax PyTree."""

import copy
import dataclasses
import typing
from typing import Dict, Optional, Sequence, Tuple, TypeVar, Union
import warnings
import jax
import numpy as np

_T = TypeVar('_T')


def _jax_in_args(typ) -> bool:
  if typ is jax.Array:
    return True
  if dataclasses.is_dataclass(typ):
    return any(_jax_in_args(f.type) for f in dataclasses.fields(typ))
  if typing.get_origin(typ) in (tuple, list, dict, Union, set):
    return any(_jax_in_args(t) for t in typing.get_args(typ))
  return False


def dataclass(clz: _T, register_as_pytree: bool) -> _T:
  """Wraps a dataclass with metadata for which fields are pytrees.

  This is based off flax.struct.dataclass, but instead of using field
  descriptors to specify which fields are pytrees, we follow a simple rule:
  a leaf field is a pytree node if and only if it's a jax.Array

  Args:
    clz: the class to register as a dataclass

  Returns:
    the resulting dataclass, registered with Jax
  """
  data_clz = dataclasses.dataclass(frozen=True)(clz)
  data_clz.replace = dataclasses.replace

  if register_as_pytree:
    meta_fields, data_fields = [], []
    for field in dataclasses.fields(data_clz):
      if _jax_in_args(field.type):
        data_fields.append(field)
      else:
        meta_fields.append(field)

    def iterate_clz_with_keys(x):
      def to_meta(field, obj):
        val = getattr(obj, field.name)
        # numpy arrays are not hashable so return raw bytes instead
        if isinstance(val, np.ndarray):
          return (val.tobytes(), val.dtype, val.shape)
        else:
          return val

      def to_data(field, obj):
        return (jax.tree_util.GetAttrKey(field.name), getattr(obj, field.name))

      data = tuple(to_data(f, x) for f in data_fields)
      meta = tuple(to_meta(f, x) for f in meta_fields)
      return data, meta

    def clz_from_iterable(meta, data):

      def from_meta(field, meta):
        if field.type is np.ndarray:
          arr = np.frombuffer(meta[0], dtype=meta[1]).reshape(meta[2])
          return (field.name, arr)
        else:
          return (field.name, meta)

      from_data = lambda field, meta: (field.name, meta)

      meta_args = tuple(from_meta(f, m) for f, m in zip(meta_fields, meta))
      data_args = tuple(from_data(f, m) for f, m in zip(data_fields, data))

      return data_clz(**dict(meta_args + data_args))

    jax.tree_util.register_pytree_with_keys(
        data_clz, iterate_clz_with_keys, clz_from_iterable
    )

  return data_clz


TNode = TypeVar('TNode', bound='PyTreeNode')


class PyTreeNode:
  """Base class for dataclasses that should act like a JAX pytree node.

  This base class additionally avoids type checking errors when using PyType.
  """

  def __init_subclass__(cls, register_as_pytree: bool = True, **kwargs):
    super().__init_subclass__(**kwargs)
    dataclass(cls, register_as_pytree=register_as_pytree)

  def __init__(self, *args, **kwargs):
    # stub for pytype
    raise NotImplementedError

  def replace(self: TNode, **overrides) -> TNode:
    # stub for pytype
    raise NotImplementedError

  @classmethod
  def fields(cls) -> Tuple[dataclasses.Field, ...]:  # pylint: disable=g-bare-generic
    return dataclasses.fields(cls)

  def tree_replace(
      self, params: Dict[str, Optional[jax.typing.ArrayLike]]
  ) -> 'PyTreeNode':
    new = self
    for k, v in params.items():
      new = _tree_replace(new, k.split('.'), v)
    return new


def _tree_replace(
    base: PyTreeNode,
    attr: Sequence[str],
    val: Optional[jax.typing.ArrayLike],
) -> PyTreeNode:
  """Sets attributes in a struct.dataclass with values."""
  if not attr:
    return base

  # special case for List attribute
  if len(attr) > 1 and isinstance(getattr(base, attr[0]), list):
    lst = copy.deepcopy(getattr(base, attr[0]))

    for i, g in enumerate(lst):
      if not hasattr(g, attr[1]):
        continue
      v = val if not hasattr(val, '__iter__') else val[i]
      lst[i] = _tree_replace(g, attr[1:], v)

    return base.replace(**{attr[0]: lst})

  if len(attr) == 1:
    return base.replace(**{attr[0]: val})

  return base.replace(
      **{attr[0]: _tree_replace(getattr(base, attr[0]), attr[1:], val)}
  )
