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
"""Experimental: many simulations of one model, stepped on a thread pool.

Each simulation is its mjSTATE_INTEGRATION row; a call loads the row into an
MjData, runs, copies out the bound derived fields, and saves the row back, on
nthread threads including the caller's. Results are bit-identical to a loop over
one MjData per simulation at any thread count.

  batch = Batch(model, nsim=1024)
  qpos, ctrl, xpos = batch.bind('qpos'), batch.bind('ctrl'), batch.bind('xpos')
  for _ in range(1000):
    ctrl[:] = policy(qpos)
    batch.step()

bind() returns a live (nsim, ...) array. State fields (qpos, qvel, ctrl, mocap_pos,
...) are views of the state rows: a write is the simulation's state at its next
call. Derived fields (xpos, sensordata, ...) are refreshed after every call on a
simulation, one substep behind the state after step(), as with mj_step; a derived
field bound between calls is zero until the next call. joint(name), body(name),
sensor(name) and the other MjData named views slice the bound fields per object:
batch.joint('hinge').qpos is (nsim, 1).

By default an MjData serves many simulations, so memory scales with threads; only
the fields a call computes are then meaningful, and a field it does not compute
(qfrc_inverse after step, quantities computed only for the sensors that need them)
holds another simulation's value. persistent=True keeps one MjData per simulation
instead, which every bound field then reflects, and is required for models with
sleep enabled.

The batch's threads run across simulations; the engine's thread pool
(mju_threadpool) runs within one, and the two do not combine. Physics callbacks
(mjcb_*) must be C functions, installed through ctypes: a Python function
installed as one makes the batch's calls raise ValueError.
"""

from typing import Any, Dict, Optional, Sequence, Tuple, Union

import mujoco
from mujoco import _batch
import numpy as np
from numpy import typing as npt

# MjData fields stored in the state rows, by state component
_STATE_FIELDS = {
    'time': mujoco.mjtState.mjSTATE_TIME,
    'qpos': mujoco.mjtState.mjSTATE_QPOS,
    'qvel': mujoco.mjtState.mjSTATE_QVEL,
    'act': mujoco.mjtState.mjSTATE_ACT,
    'history': mujoco.mjtState.mjSTATE_HISTORY,
    'qacc_warmstart': mujoco.mjtState.mjSTATE_WARMSTART,
    'ctrl': mujoco.mjtState.mjSTATE_CTRL,
    'qfrc_applied': mujoco.mjtState.mjSTATE_QFRC_APPLIED,
    'xfrc_applied': mujoco.mjtState.mjSTATE_XFRC_APPLIED,
    'eq_active': mujoco.mjtState.mjSTATE_EQ_ACTIVE,
    'mocap_pos': mujoco.mjtState.mjSTATE_MOCAP_POS,
    'mocap_quat': mujoco.mjtState.mjSTATE_MOCAP_QUAT,
    'userdata': mujoco.mjtState.mjSTATE_USERDATA,
    'plugin_state': mujoco.mjtState.mjSTATE_PLUGIN,
}

Ids = Optional[npt.ArrayLike]
RecordItem = Union[str, int, mujoco.mjtState]


class SimulationError(RuntimeError):
  """MuJoCo raised an error in some simulations of a call.

  Those simulations kept the state they had before the call, and the others ran
  to completion. ids lists them; Batch.error(i) has each message.
  """

  def __init__(self, ids: np.ndarray, message: str):
    self.ids = ids
    super().__init__(f'MuJoCo raised in simulations {ids.tolist()}: {message}')


class Batch:
  """nsim simulations of a copy of model, on nthread threads (0: all CPUs).

  With persistent=True, the batch keeps one MjData per simulation.
  """

  def __init__(
      self,
      model: mujoco.MjModel,
      nsim: int,
      nthread: int = 0,
      persistent: bool = False,
  ):
    self._b = _batch.Batch(model, nsim, nthread, persistent)
    self.model = model
    self._data = mujoco.MjData(model)  # field shapes and dtypes
    self.state = self._b.state()
    self.warning = self._b.warning()
    self.status = self._b.status()
    self.status.flags.writeable = False

  @property
  def nsim(self) -> int:
    return self._b.nsim

  @property
  def nthread(self) -> int:
    return self._b.nthread

  @property
  def nstate(self) -> int:
    return self._b.nstate

  @property
  def persistent(self) -> bool:
    return self._b.persistent

  def error(self, sim: int) -> str:
    """Message of the error simulation sim raised in its last call, or ''."""
    return self._b.error(sim)

  def bind(self, name: str) -> np.ndarray:
    """Return a live (nsim, ...) array over an MjData field."""
    if name in _STATE_FIELDS:
      offset, size = self._component(_STATE_FIELDS[name])
      view = self.state[:, offset : offset + size]
      if name == 'time':
        return view[:, 0]
      return view.reshape(self.nsim, *getattr(self._data, name).shape)
    template = getattr(self._data, name, None)
    raw = self._b.output(name)
    if raw is None or not isinstance(template, np.ndarray):
      raise ValueError(f'{name} is not an MjData array field')
    return raw.view(template.dtype).reshape(self.nsim, *template.shape)

  def actuator(self, key: Union[int, str]) -> '_Named':
    return _Named(self, 'actuator', key)

  def body(self, key: Union[int, str]) -> '_Named':
    return _Named(self, 'body', key)

  def camera(self, key: Union[int, str]) -> '_Named':
    return _Named(self, 'camera', key)

  def geom(self, key: Union[int, str]) -> '_Named':
    return _Named(self, 'geom', key)

  def joint(self, key: Union[int, str]) -> '_Named':
    return _Named(self, 'joint', key)

  def light(self, key: Union[int, str]) -> '_Named':
    return _Named(self, 'light', key)

  def sensor(self, key: Union[int, str]) -> '_Named':
    return _Named(self, 'sensor', key)

  def site(self, key: Union[int, str]) -> '_Named':
    return _Named(self, 'site', key)

  def tendon(self, key: Union[int, str]) -> '_Named':
    return _Named(self, 'tendon', key)

  def expand(self, name: str) -> np.ndarray:
    """Give a model field per-simulation values; return them, (nsim, ...).

    Array fields are named as in MjModel, MjOption fields as 'opt.<name>'.
    """
    if name.startswith('opt.'):
      template = np.asarray(getattr(self.model.opt, name[4:], None))
      dtype = np.float64 if template.dtype.kind == 'f' else np.int32
    else:
      template = getattr(self.model, name, None)
      dtype = getattr(template, 'dtype', None)
    raw = self._b.expand(name)
    if raw is None or not isinstance(template, np.ndarray):
      raise ValueError(f'{name} is not an expandable MjModel field')
    return raw.view(dtype).reshape(self.nsim, *template.shape)

  def step(self, ids: Ids = None, nstep: int = 1) -> None:
    """Advance each simulation by nstep calls to mj_step."""
    ids = _ids(ids, self.nsim)
    self._check(ids, self._b.step(ids, nstep))

  def forward(self, ids: Ids = None) -> None:
    """Run mj_forward on each simulation."""
    ids = _ids(ids, self.nsim)
    self._check(ids, self._b.forward(ids))

  def reset(self, ids: Ids = None, keyframe: int = -1) -> None:
    """Reset to the model's defaults or to a keyframe, then run mj_forward.

    Writes to the reset simulations' state made before the call are discarded.
    """
    ids = _ids(ids, self.nsim)
    self._check(ids, self._b.reset(ids, keyframe))

  def set_const(self, ids: Ids = None) -> None:
    """Run mj_setConst per simulation; derived fields become per-simulation."""
    ids = _ids(ids, self.nsim)
    self._check(ids, self._b.set_const(ids))

  def rollout(
      self,
      nstep: int,
      control: Optional[npt.ArrayLike] = None,
      control_spec: int = mujoco.mjtState.mjSTATE_CTRL,
      record: Sequence[RecordItem] = ('qpos',),
      ids: Ids = None,
  ) -> Dict[RecordItem, np.ndarray]:
    """Advance each simulation by nstep substeps, recording after each.

    As mujoco.rollout: before substep k, the state components in control_spec
    are set from control, (n, nstep, ncontrol) for the n simulations of the call,
    or the inputs are left as they are when control is None. A simulation whose
    warning counters rise stops stepping, and its remaining records repeat its
    last ones.

    Args:
      nstep: number of substeps.
      control: (n, nstep, mj_stateSize(model, control_spec)) or None.
      control_spec: mjtState bits within mjSTATE_USER.
      record: MjData field names, or mjtState specs for state vectors.
      ids: the simulations, or None for all.

    Returns:
      A dict from each record item to its (n, nstep, ...) array.
    """
    ids = _ids(ids, self.nsim)
    n = self.nsim if ids is None else len(ids)
    if control is not None:
      ncontrol = mujoco.mj_stateSize(self.model, int(control_spec))
      control = np.ascontiguousarray(
          np.broadcast_to(control, (n, nstep, ncontrol)), dtype=np.float64
      )
    out, records = {}, []
    for item in record:
      if isinstance(item, str):
        template = getattr(self._data, item, None)
        if not isinstance(template, np.ndarray):
          raise ValueError(f'{item} is not an MjData array field')
        buf = np.empty((n, nstep, *template.shape), template.dtype)
        records.append((item, 0, buf))
      else:
        size = mujoco.mj_stateSize(self.model, int(item))
        buf = np.empty((n, nstep, size), np.float64)
        records.append((None, int(item), buf))
      out[item] = buf
    self._check(
        ids,
        self._b.rollout(ids, nstep, control, int(control_spec), records),
    )
    return out

  def jac(
      self, body: int, point: npt.ArrayLike, ids: Ids = None
  ) -> Tuple[np.ndarray, np.ndarray]:
    """Translational and rotational Jacobians of a world point on a body.

    Runs mj_kinematics and mj_comPos per simulation, from its state, then
    mj_jac; the state and bound fields are untouched.

    Args:
      body: body id.
      point: world-frame points, (n, 3) for the n simulations of the call.
      ids: the simulations, or None for all.

    Returns:
      jacp, jacr: (n, 3, nv) each.
    """
    ids = _ids(ids, self.nsim)
    sel = slice(None) if ids is None else ids
    full = np.zeros((self.nsim, 3))
    full[sel] = point
    jacp = np.zeros((self.nsim, 3, self.model.nv))
    jacr = np.zeros((self.nsim, 3, self.model.nv))
    self._check(ids, self._b.jac(ids, body, full, jacp, jacr))
    return jacp[sel], jacr[sel]

  def ray(
      self,
      pnt: npt.ArrayLike,
      vec: npt.ArrayLike,
      geomgroup: Optional[npt.ArrayLike] = None,
      flg_static: bool = True,
      bodyexclude: int = -1,
      ids: Ids = None,
  ) -> Tuple[np.ndarray, np.ndarray]:
    """Intersect rays with each simulation's geoms, as mj_ray.

    Runs mj_kinematics (and mj_flex) per simulation, from its state; the state
    and bound fields are untouched.

    Args:
      pnt: ray origins, (n, nray, 3) for the n simulations of the call.
      vec: ray directions, (n, nray, 3).
      geomgroup: geom groups to include, (mjNGROUP,), or None for all.
      flg_static: whether to include static geoms.
      bodyexclude: body whose geoms are excluded, or -1.
      ids: the simulations, or None for all.

    Returns:
      dist, geomid: (n, nray) each; dist is -1 and geomid -1 for a miss.
    """
    ids = _ids(ids, self.nsim)
    sel = slice(None) if ids is None else ids
    pnt, vec = np.asarray(pnt, np.float64), np.asarray(vec, np.float64)
    nray = pnt.shape[1]
    full_pnt = np.zeros((self.nsim, nray, 3))
    full_vec = np.zeros((self.nsim, nray, 3))
    full_pnt[sel], full_vec[sel] = pnt, vec
    if geomgroup is not None:
      geomgroup = np.ascontiguousarray(geomgroup, dtype=np.uint8)
    dist = np.zeros((self.nsim, nray))
    geomid = np.zeros((self.nsim, nray), np.int32)
    self._check(
        ids,
        self._b.ray(
            ids, full_pnt, full_vec, geomgroup, flg_static, bodyexclude, dist,
            geomid,
        ),
    )
    return dist[sel], geomid[sel]

  def _component(self, component: Any) -> Tuple[int, int]:
    component = int(component)
    full = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    offset = mujoco.mj_stateSize(self.model, full & (component - 1))
    return offset, mujoco.mj_stateSize(self.model, component)

  def _check(self, ids: Optional[np.ndarray], nfail: int) -> None:
    if not nfail:
      return
    sel = np.arange(self.nsim) if ids is None else ids
    failed = sel[self.status[sel] != 0]
    if failed.size != nfail:  # set_const recomputed every simulation
      failed = np.flatnonzero(self.status)
    raise SimulationError(failed, self.error(int(failed[0])))


class _Named:
  """One object's slices of bound fields, (nsim, ...), named as in MjData.

  batch.joint('hinge').qpos is the part of batch.bind('qpos') that
  data.joint('hinge').qpos is of data.qpos, and binds the field as bind() does.
  """

  # MjData field prefixes of each object kind's attributes
  _PREFIX = {
      'actuator': 'actuator_',
      'body': '',
      'camera': 'cam_',
      'geom': 'geom_',
      'joint': '',
      'light': 'light_',
      'sensor': 'sensor',
      'site': 'site_',
      'tendon': 'ten_',
  }

  def __init__(self, batch: Batch, kind: str, key: Union[int, str]):
    view = getattr(batch._data, kind)(key)  # pylint: disable=protected-access
    object.__setattr__(self, '_batch', batch)
    object.__setattr__(self, '_kind', kind)
    object.__setattr__(self, '_view', view)
    object.__setattr__(self, 'id', view.id)
    object.__setattr__(self, 'name', view.name)

  def __getattr__(self, attr: str) -> np.ndarray:
    part = getattr(self._view, attr)
    data = self._batch._data  # pylint: disable=protected-access
    for name in (self._PREFIX[self._kind] + attr, attr):
      full = getattr(data, name, None)
      if isinstance(full, np.ndarray) and full.dtype == part.dtype:
        start = (part.ctypes.data - full.ctypes.data) // full.itemsize
        if 0 <= start and start + part.size <= full.size:
          break
    else:
      raise AttributeError(f'{attr} is not an MjData field of a {self._kind}')
    bound = self._batch.bind(name).reshape(self._batch.nsim, -1)
    out = bound[:, start : start + part.size].reshape(
        self._batch.nsim, *part.shape
    )
    assert np.shares_memory(out, bound)
    return out

  def __setattr__(self, attr: str, value: Any) -> None:
    """Write value to every simulation's slice: batch.joint('j').qpos = 0.5."""
    if attr in ('id', 'name') or attr.startswith('_'):
      raise AttributeError(f'{attr} is read-only')
    self.__getattr__(attr)[...] = value


def _ids(ids: Ids, nsim: int) -> Optional[np.ndarray]:
  """Validated int32 ids from integer ids or a boolean mask; None for all.

  Shape, dtype and range are checked before narrowing to int32, so that no id is
  truncated or wrapped into a valid one. Order and uniqueness are checked by the
  batch.
  """
  if ids is None:
    return None
  ids = np.asarray(ids)
  if ids.dtype == bool:
    if ids.shape != (nsim,):
      raise ValueError(f'a boolean mask must have shape ({nsim},)')
    return np.flatnonzero(ids).astype(np.int32)
  if ids.ndim != 1:
    raise ValueError('ids must be one-dimensional')
  if ids.size == 0:
    return np.zeros(0, np.int32)
  if not np.issubdtype(ids.dtype, np.integer):
    raise ValueError(f'ids must be integers, got {ids.dtype}')
  if ids.min() < 0 or ids.max() >= nsim:
    raise ValueError(f'ids must be in [0, {nsim})')
  return np.ascontiguousarray(ids, dtype=np.int32)
