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
"""Scan across data ordered by body joint types and kinematic tree order."""

from typing import Any, Callable, TypeVar

import jax
from jax import numpy as jp
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import JointType
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import TrnType
# pylint: enable=g-importing-member
import numpy as np


Y = TypeVar('Y')


# TODO(erikfrey): re-check if this really helps perf
def _take(obj: Y, idx: np.ndarray) -> Y:
  """Takes idxs on any pytree given to it.

  XLA executes x[jp.array([1, 2, 3])] slower than x[1:4], so we detect when
  take indices are contiguous, and convert them to slices.

  Args:
    obj: an input pytree
    idx: indices to take

  Returns:
    obj pytree with leaves taken by idxs
  """

  if isinstance(obj, np.ndarray):
    return obj[idx]

  def take(x):
    # TODO(erikfrey): if this helps perf, add support for striding too
    if not x.shape[0]:
      return x
    elif (
        len(idx.shape) == 1
        and idx.size > 0
        and (idx == np.arange(idx[0], idx[0] + idx.size)).all()
        and (idx > 0).all()
    ):
      x = x[idx[0] : idx[-1] + 1]
    else:
      x = x.take(jp.array(idx), axis=0, mode='wrap')
    return x

  return jax.tree_util.tree_map(take, obj)


def _q_bodyid(m: Model) -> np.ndarray:
  """Returns the bodyid for each qpos adress."""
  q_bodyids = [np.array([], dtype=np.int32)]
  for jnt_type, jnt_bodyid in zip(m.jnt_type, m.jnt_bodyid):
    width = {JointType.FREE: 7, JointType.BALL: 4}.get(jnt_type, 1)
    q_bodyids.append(np.repeat(jnt_bodyid, width))
  return np.concatenate(q_bodyids)


def _q_jointid(m: Model) -> np.ndarray:
  """Returns the jointid for each qpos adress."""
  q_jointid = [np.array([], dtype=np.int32)]
  for i, jnt_type in enumerate(m.jnt_type):
    width = {JointType.FREE: 7, JointType.BALL: 4}.get(jnt_type, 1)
    q_jointid.append(np.repeat(i, width))
  return np.concatenate(q_jointid)


def _index(haystack: np.ndarray, needle: np.ndarray) -> np.ndarray:
  """Returns indexes in haystack for elements in needle."""
  idx = np.argsort(haystack)
  sorted_haystack = haystack[idx]
  sorted_idx = np.searchsorted(sorted_haystack, needle)
  idx = np.take(idx, sorted_idx, mode='clip')
  idx[haystack[idx] != needle] = -1

  return idx


def _nvmap(f: Callable[..., Y], *args) -> Y:
  """A vmap that accepts numpy arrays.

  Numpy arrays are statically vmapped, and the elements are passed to f as
  static arguments.  The implication is that all the elements of numpy array
  arguments must be the same.

  Args:
    f: function to be mapped over
    *args: args to be mapped along, passed to f

  Returns:
    the result of vmapping f over args

  Raises:
    RuntimeError: if numpy arg elements do not match
  """
  for arg in args:
    if isinstance(arg, np.ndarray) and not np.all(arg == arg[0]):
      raise RuntimeError(f'numpy arg elements do not match: {arg}')

  # split out numpy and jax args
  np_args = [a[0] if isinstance(a, np.ndarray) else None for a in args]
  args = [a if n is None else None for n, a in zip(np_args, args)]

  # remove empty args that we should not vmap over
  args = jax.tree_util.tree_map(lambda a: a if a.shape[0] else None, args)
  in_axes = [None if a is None else 0 for a in args]

  def outer_f(*args, np_args=np_args):
    args = [a if n is None else n for n, a in zip(args, np_args)]
    return f(*args)

  return jax.vmap(outer_f, in_axes=in_axes)(*args)


def _check_input(m: Model, args: Any, in_types: str) -> None:
  """Checks that scan input has the right shape."""
  if m.nv == 0:
    raise ValueError('Scan across Model with zero DoFs unsupported.')
  size = {
      'b': m.nbody,
      'j': m.njnt,
      'q': m.nq,
      'v': m.nv,
      'u': m.nu,
      'a': m.na,
      's': m.nsite,
      'c': m.ncam,
  }
  for idx, (arg, typ) in enumerate(zip(args, in_types)):
    if len(arg) != size[typ]:
      raise IndexError((
          f'f argument "{idx}" with type "{typ}" has length "{len(arg)}"'
          f' which does not match the in_types[{idx}] expected length of '
          f'"{size[typ]}".'
      ))


def _check_output(
    y: jax.Array, take_ids: np.ndarray, typ: str, idx: int
) -> None:
  """Checks that scan output has the right shape."""
  if y.shape[0] != take_ids.shape[0]:
    raise IndexError((
        f'f output "{idx}" with type "{typ}" has shape "{y.shape[0]}" '
        f'which does not match the out_types[{idx}] expected size of'
        f' "{take_ids.shape[0]}".'
    ))


def flat(
    m: Model,
    f: Callable[..., Y],
    in_types: str,
    out_types: str,
    *args,
    group_by: str = 'j',
) -> Y:
  r"""Scan a function across bodies or actuators.

  Scan group data according to type and batch shape then calls vmap(f) on it.\

  Args:
    m: an mjx model
    f: a function to be scanned with the following type signature:
        def f(key, *args) -> y
      where
        ``key`` gives grouping key for this function instance
        ``*args`` are input arguments with types matching ``in_types``
        ``y`` is an output arguments with types matching ``out_type``
    in_types: string specifying the type of each input arg:
      'b': split according to bodies
      'j': split according to joint types
      'q': split according to generalized coordinates (len(qpos))
      'v': split according to degrees of freedom (len(qvel))
      'u': split according to actuators
      'a': split according to actuator activations
      'c': split according to camera
    out_types: string specifying the types the output dimension matches
    *args: the input arguments corresponding to ``in_types``
    group_by: the type to group by, either joints or actuators

  Returns:
    The stacked outputs of ``f`` matching the model's order.

  Raises:
      IndexError: if function output shape does not match out_types shape
  """
  _check_input(m, args, in_types)

  if group_by not in {'j', 'u', 'c'}:
    raise NotImplementedError(f'group by type "{group_by}" not implemented.')

  def key_j(type_ids):
    if any(t in 'jqv' for t in in_types + out_types):
      return tuple(m.jnt_type[type_ids['j']])
    return ()

  def type_ids_j(m, i):
    return {
        'b': i,
        'j': np.nonzero(m.jnt_bodyid == i)[0],
        'v': np.nonzero(m.dof_bodyid == i)[0],
        'q': np.nonzero(_q_bodyid(m) == i)[0],
    }

  def key_u(type_ids):
    ids_u, ids_j = type_ids['u'], type_ids['j']
    return (
        m.actuator_biastype[ids_u],
        m.actuator_gaintype[ids_u],
        m.actuator_dyntype[ids_u],
        m.actuator_trntype[ids_u],
        m.jnt_type[ids_j],
        m.actuator_trnid[ids_u, 1] == -1,  # key by refsite being present
    )

  def type_ids_u(m, i):
    typ_ids = {
        'u': i,
        'a': m.actuator_actadr[i],
        'j': (
            m.actuator_trnid[i, 0]
            if m.actuator_trntype[i] in (TrnType.JOINT, TrnType.JOINTINPARENT)
            else -1
        ),
        's': (
            m.actuator_trnid[i]
            if m.actuator_trntype[i] == TrnType.SITE
            else np.array([-1, -1])
        ),
    }
    v, q = np.array([-1]), np.array([-1])
    if m.actuator_trntype[i] in (TrnType.JOINT, TrnType.JOINTINPARENT):
      # v/q are associated with the joint transmissions only
      v = np.nonzero(m.dof_jntid == typ_ids['j'])[0]
      q = np.nonzero(_q_jointid(m) == typ_ids['j'])[0]

    typ_ids.update({'v': v, 'q': q})

    return typ_ids

  def key_c(type_ids):
    return m.cam_mode[type_ids['c']], m.cam_targetbodyid[type_ids['c']] >= 0

  def type_ids_c(unused_m, i):
    return {
        'c': i,
    }

  type_ids_fn = {'j': type_ids_j, 'u': type_ids_u, 'c': type_ids_c}[group_by]
  key_fn = {'j': key_j, 'u': key_u, 'c': key_c}[group_by]

  # build up a grouping of type take-ids in body/actuator order
  key_typ_ids, order = {}, []
  all_types = set(in_types + out_types)
  n_items = {'j': m.nbody, 'u': m.nu, 'c': m.ncam}[group_by]
  for i in np.arange(n_items, dtype=np.int32):
    typ_ids = type_ids_fn(m, i)

    # create grouping key
    key = key_fn(typ_ids)
    order.append((key, typ_ids))

    # add ids per type to the corresponding group
    for t in all_types:
      out = key_typ_ids.setdefault(key, {})
      val = np.expand_dims(typ_ids[t], axis=0)
      out[t] = np.concatenate((out[t], val)) if t in out else val

  key_typ_ids = list(sorted(key_typ_ids.items()))

  # use this grouping to take the right data subsets and call vmap(f)
  ys = []
  for _, typ_ids in key_typ_ids:
    # only execute f if we would actually take something from the result
    if any(typ_ids[v].size > 0 for v in out_types):
      f_args = [_take(arg, typ_ids[typ]) for arg, typ in zip(args, in_types)]
      y = _nvmap(f, *f_args)
      ys.append(y)
    else:
      ys.append(None)

  # remove None results from the final output
  key_typ_ids = [v for y, v in zip(ys, key_typ_ids) if y is not None]
  ys = [y for y in ys if y is not None]
  ys_keys = set([k for k, *_ in key_typ_ids])
  order = [o for k, o in order if k in ys_keys]

  # get the original input order
  order = [[o[t] for o in order] for t in all_types]
  order = [
      np.concatenate(o) if isinstance(o[0], np.ndarray) else np.array(o)
      for o in order
  ]
  order = dict(zip(all_types, order))

  # concatenate back to a single tree and drop the grouping dimension
  f_ret_is_seq = isinstance(ys[0], (list, tuple))
  ys = ys if f_ret_is_seq else [[y] for y in ys]
  flat_ = {'j': 'b', 'u': 'uaj', 'c': 'c'}[group_by]
  ys = [
      [v if typ in flat_ else jp.concatenate(v) for v, typ in zip(y, out_types)]
      for y in ys
  ]
  ys = jax.tree_util.tree_map(lambda *x: jp.concatenate(x), *ys)

  # put concatenated results back in order
  reordered_ys = []
  for i, (y, typ) in enumerate(zip(ys, out_types)):
    _check_output(y, order[typ], typ, i)
    ids = np.concatenate([np.hstack(v[typ]) for _, v in key_typ_ids])
    input_order = order[typ][np.where(order[typ] != -1)]
    reordered_ys.append(_take(y, _index(ids, input_order)))
  y = reordered_ys if f_ret_is_seq else reordered_ys[0]

  return y


def body_tree(
    m: Model,
    f: Callable[..., Y],
    in_types: str,
    out_types: str,
    *args,
    reverse: bool = False,
) -> Y:
  r"""Scan ``f`` across bodies in tree order, carrying results up/down the tree.

  This function groups bodies according to level and attached joints, then calls
  vmap(f) on them.\

  Args:
    m: an mjx mjmodel
    f: a function to be scanned with the following type signature:
        def f(y, *args) -> y
      where
        ``y`` is the carry value and return value
        ``*args`` are input arguments with types matching ``in_types``
    in_types: string specifying the type of each input arg:
      'b': split according to bodies
      'j': split according to joint types
      'q': split according to generalized coordinates (len(qpos))
      'v': split according to degrees of freedom (len(qvel))
    out_types: string specifying the types the output dimension matches
    *args: the input arguments corresponding to ``in_types``
    reverse: if True, scans up the body tree from leaves to root, otherwise
      root to leaves

  Returns:
    The stacked outputs of ``f`` matching the model's body order.

  Raises:
      IndexError: if function output shape does not match out_types shape
  """
  _check_input(m, args, in_types)

  # group together bodies that will be processed together.  grouping key:
  #   1) the tree depth: parent bodies are processed first, so that they are
  #      available as carry input to child bodies (or reverse if reverse=True)
  #   2) the types of arguments passed to f, both carry and *args:
  #      * for 'b' arguments, there is no extra grouping
  #      * for 'j' arguments, we group by joint type
  #      * for 'q' arguments, we group by q width
  #      * for 'v' arguments, we group by dof width
  depths = np.zeros(m.nbody, dtype=np.int32)

  # map key => body id
  key_body_ids = {}
  for body_id in range(m.nbody):
    parent_id = -1
    if body_id > 0:
      parent_id = m.body_parentid[body_id]
      depths[body_id] = 1 + depths[parent_id]

    # create grouping key: depth, carry, args
    key = (depths[body_id],)

    for i, t in enumerate(out_types + in_types):
      id_ = parent_id if i < len(out_types) else body_id
      if t == 'b':
        continue
      elif t == 'j':
        key += tuple(m.jnt_type[np.nonzero(m.jnt_bodyid == id_)[0]])
      elif t == 'v':
        key += (len(np.nonzero(m.dof_bodyid == id_)[0]),)
      elif t == 'q':
        key += (len(np.nonzero(_q_bodyid(m) == id_)[0]),)

    body_ids = key_body_ids.get(key, np.array([], dtype=np.int32))
    key_body_ids[key] = np.append(body_ids, body_id)

  # find parent keys of each key.  a key may have multiple parents if the
  # carry output keys of distinct parents are the same. e.g.:
  # - depth 0 body 1 (slide joint)
  # -- depth 1 body 1 (hinge joint)
  # - depth 0 body 2 (ball joint)
  # -- depth 1 body 2 (hinge joint)
  # given a scan with 'j' in the in_types, we would group depth 0 bodies
  # separately but we may group depth 1 bodies together
  key_parents = {}

  for key, body_ids in key_body_ids.items():
    body_ids = body_ids[body_ids != 0]  # ignore worldbody, has no parent
    if body_ids.size == 0:
      continue
    # find any key which has a body id that is a parent of these body_ids
    pids = m.body_parentid[body_ids]
    parents = {k for k, v in key_body_ids.items() if np.isin(v, pids).any()}
    key_parents[key] = list(sorted(parents))

  # key => take indices
  key_in_take, key_y_take = {}, {}
  for key, body_ids in key_body_ids.items():
    for i, typ in enumerate(in_types + out_types):
      if typ == 'b':
        ids = body_ids
      elif typ == 'j':
        ids = np.stack([np.nonzero(m.jnt_bodyid == b)[0] for b in body_ids])
      elif typ == 'v':
        ids = np.stack([np.nonzero(m.dof_bodyid == b)[0] for b in body_ids])
      elif typ == 'q':
        ids = np.stack([np.nonzero(_q_bodyid(m) == b)[0] for b in body_ids])
      else:
        raise ValueError(f'Unknown in_type: {typ}')
      if i < len(in_types):
        key_in_take.setdefault(key, []).append(ids)
      else:
        key_y_take.setdefault(key, []).append(np.hstack(ids))

  # use this grouping to take the right data subsets and call vmap(f)
  keys = sorted(key_body_ids, reverse=reverse)
  key_y = {}
  for key in keys:
    carry = None

    if reverse:
      child_keys = [k for k, v in key_parents.items() if key in v]

      for child_key in child_keys:
        y = key_y[child_key]
        body_ids = key_body_ids[key]
        parent_ids = m.body_parentid[key_body_ids[child_key]]
        id_map = _index(body_ids, parent_ids)

        def index_sum(x, i=id_map, s=body_ids.size):
          return jax.ops.segment_sum(x, i, s)

        y = jax.tree_util.tree_map(index_sum, y)
        carry = y if carry is None else jax.tree_util.tree_map(jp.add, carry, y)
    elif key in key_parents:
      ys = [key_y[p] for p in key_parents[key]]
      y = jax.tree_util.tree_map(lambda *x: jp.concatenate(x), *ys)
      body_ids = np.concatenate([key_body_ids[p] for p in key_parents[key]])
      parent_ids = m.body_parentid[key_body_ids[key]]
      take_fn = lambda x, i=_index(body_ids, parent_ids): _take(x, i)
      carry = jax.tree_util.tree_map(take_fn, y)

    f_args = [_take(arg, ids) for arg, ids in zip(args, key_in_take[key])]
    key_y[key] = _nvmap(f, carry, *f_args)

  # slice None results from the final output
  keys = [k for k in keys if key_y[k] is not None]

  # concatenate ys, drop grouping dimensions, put back in order
  y = []
  for i, typ in enumerate(out_types):
    y_typ = [key_y[key] for key in keys]
    if len(out_types) > 1:
      y_typ = [y_[i] for y_ in y_typ]
    if typ != 'b':
      y_typ = jax.tree_util.tree_map(jp.concatenate, y_typ)
    y_typ = jax.tree_util.tree_map(lambda *x: jp.concatenate(x), *y_typ)
    y_take = np.argsort(np.concatenate([key_y_take[key][i] for key in keys]))
    _check_output(y_typ, y_take, typ, i)
    y.append(_take(y_typ, y_take))

  y = y[0] if len(out_types) == 1 else y

  return y
