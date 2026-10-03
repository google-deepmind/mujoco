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
sensor(name) and the other named views slice the bound and expanded fields per
object, with the attributes of MjData's and MjModel's: batch.joint('hinge').qpos
is (nsim, 1), and batch.body('pole').mass = masses writes per-simulation masses.
get_model(sim) and get_data(sim) materialize one simulation, to render it or to
pass it to any function. Fields are also attributes, as on MjData:
batch.xpos binds xpos, and batch.ctrl = u writes every simulation's controls.
DATA_FIELDS and MODEL_FIELDS name the fields bind() and expand() accept.

By default an MjData serves many simulations, so memory scales with threads; only
the fields a call computes are then meaningful, and a field it does not compute
(qfrc_inverse after step, quantities computed only for the sensors that need them)
holds another simulation's value. persistent=True keeps one MjData per simulation
instead, which every bound field then reflects, and is required for models with
sleep or IPC enabled. Likewise, engine plugins that keep state outside plugin_state
share it among the simulations a thread runs, unless the batch is persistent.

The batch's threads run across simulations; the engine's thread pool
(mju_threadpool) runs within one, and the two do not combine. Physics callbacks
(mjcb_*) must be C functions, installed through ctypes: a Python function
installed as one makes the batch's calls raise ValueError.
"""

import copy
from typing import Any, Dict, FrozenSet, Optional, Sequence, Tuple, Union

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

  def __getattr__(self, name: str) -> np.ndarray:
    # consulted only for names Batch does not have: MjData fields, as bind()
    if name in DATA_FIELDS:
      return self.bind(name)
    raise AttributeError(f"'Batch' object has no attribute {name!r}")

  def __setattr__(self, name: str, value: Any) -> None:
    if name in DATA_FIELDS:  # write every simulation's value, as with MjData
      self.bind(name)[...] = value
    else:
      object.__setattr__(self, name, value)

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
    """Return a live (nsim, ...) array over an MjData field.

    State fields are views of the rows, which hold mjtNum (float64 in the
    standard build), so eq_active binds as mjtNum rather than as MjData's uint8.
    Other fields are outputs, copied out after every call, and read-only.
    """
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
    out = raw.view(template.dtype).reshape(self.nsim, *template.shape)
    out.flags.writeable = False  # an output: a write would be lost at the next call
    return out

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
    bvh_aabb holds the bodies' bounding volumes, which precede the meshes'.
    """
    if name.startswith('opt.'):
      template = np.asarray(getattr(self.model.opt, name[4:], None))
      dtype = mujoco.MJTNUM_DTYPE if template.dtype.kind == 'f' else np.int32
    else:
      template = getattr(self.model, name, None)
      dtype = getattr(template, 'dtype', None)
    raw = self._b.expand(name)
    if raw is None or not isinstance(template, np.ndarray):
      raise ValueError(f'{name} is not an expandable MjModel field')
    out = raw.view(dtype)
    if out.size == self.nsim * template.size:
      return out.reshape(self.nsim, *template.shape)
    # bvh_aabb holds the bodies' hierarchies, a prefix of the model's rows
    return out.reshape(self.nsim, -1, *template.shape[1:])

  def step(
      self,
      ids: Ids = None,
      nstep: int = 1,
      stage: int = mujoco.mjtStage.mjSTAGE_NONE,
  ) -> None:
    """Advance each simulation by nstep calls to mj_step, then compute stages.

    As with mj_step, derived fields describe the state before the last substep.
    stage computes the forward stages up to it at the new state: mjSTAGE_VEL
    makes position- and velocity-dependent fields and sensors current, as
    mj_step1 does; mjSTAGE_ACC runs mj_forward, whose acceleration-dependent
    values use controls that will not be applied. A persistent batch continues
    its next step from those stages, so that mjSTAGE_VEL costs nothing extra.

    Args:
      ids: the simulations, or None for all.
      nstep: number of substeps.
      stage: mjtStage, the stages to compute at the new state.
    """
    ids = _ids(ids, self.nsim)
    self._check(ids, self._b.step(ids, nstep, int(stage)))

  def forward(
      self, ids: Ids = None, stage: int = mujoco.mjtStage.mjSTAGE_ACC
  ) -> None:
    """Compute the forward stages up to stage, by default mj_forward."""
    ids = _ids(ids, self.nsim)
    self._check(ids, self._b.forward(ids, int(stage)))

  def reset(
      self,
      ids: Ids = None,
      keyframe: npt.ArrayLike = -1,
      stage: int = mujoco.mjtStage.mjSTAGE_ACC,
  ) -> None:
    """Reset to the model's defaults or to keyframes, then compute stages.

    Writes to the reset simulations' state made before the call are discarded.
    By default the reset runs mj_forward; stage=mjSTAGE_NONE skips it, for a
    reset that is followed by writes to the state and a forward().

    Args:
      ids: the simulations, or None for all.
      keyframe: a keyframe for every simulation of the call, or one each, (n,);
        -1 for the model's defaults.
      stage: mjtStage, the stages to compute after the reset.
    """
    ids = _ids(ids, self.nsim)
    n = self.nsim if ids is None else len(ids)
    keyframe = np.asarray(keyframe)
    if keyframe.dtype.kind not in 'iu':
      raise ValueError(f'keyframe must be integers, got {keyframe.dtype}')
    key = None
    if keyframe.ndim or keyframe != -1:
      key = np.ascontiguousarray(np.broadcast_to(keyframe, (n,)), np.int32)
    self._check(ids, self._b.reset(ids, key, int(stage)))

  def get_model(
      self, sim: int, model: Optional[mujoco.MjModel] = None
  ) -> mujoco.MjModel:
    """Simulation sim's model: the batch's, with its per-simulation fields.

    Args:
      sim: the simulation.
      model: an MjModel with the batch's sizes to write into, or None for a
        copy of the batch's model.

    Returns:
      The model, for rendering or for any mj_ function.
    """
    if model is None:
      model = copy.copy(self.model)
    self._b.get_model(sim, model)
    return model

  def get_data(
      self, sim: int, data: Optional[mujoco.MjData] = None
  ) -> mujoco.MjData:
    """Simulation sim's state, and in a persistent batch its whole MjData.

    Derived fields are those of the simulation's last call in a persistent
    batch; otherwise run mj_forward with get_model(sim) to compute them.

    Args:
      sim: the simulation.
      data: an MjData with the batch's sizes to write into, or None for a new
        one.

    Returns:
      The data.
    """
    if data is None:
      data = mujoco.MjData(self.model)
    self._b.get_data(sim, data)
    return data

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
      out: Optional[Dict[RecordItem, np.ndarray]] = None,
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
      out: arrays to record into, by record item, of the shapes and dtypes this
        call returns; the items not in it get new arrays.

    Returns:
      A dict from each record item to its (n, nstep, ...) array.
    """
    ids = _ids(ids, self.nsim)
    n = self.nsim if ids is None else len(ids)
    if control is not None:
      ncontrol = mujoco.mj_stateSize(self.model, int(control_spec))
      control = np.ascontiguousarray(
          np.broadcast_to(control, (n, nstep, ncontrol)), dtype=mujoco.MJTNUM_DTYPE
      )
    given, out, records = out or {}, {}, []
    for item in record:
      if isinstance(item, str):
        template = getattr(self._data, item, None)
        if not isinstance(template, np.ndarray):
          raise ValueError(f'{item} is not an MjData array field')
        shape, dtype = (n, nstep, *template.shape), template.dtype
      else:
        shape = (n, nstep, mujoco.mj_stateSize(self.model, int(item)))
        dtype = np.dtype(mujoco.MJTNUM_DTYPE)
      buf = given.get(item)
      if buf is None:
        buf = np.empty(shape, dtype)
      elif (
          not isinstance(buf, np.ndarray)
          or buf.shape != shape
          or buf.dtype != dtype
          or not buf.flags.c_contiguous
          or not buf.flags.writeable
      ):
        raise ValueError(
            f'out[{item!r}] must be a writable C-contiguous {dtype} array of'
            f' shape {shape}'
        )
      if isinstance(item, str):
        records.append((item, 0, buf))
      else:
        records.append((None, int(item), buf))
      out[item] = buf
    self._check(
        ids,
        self._b.rollout(ids, nstep, control, int(control_spec), records),
    )
    return out

  def jac(
      self, body: npt.ArrayLike, point: npt.ArrayLike, ids: Ids = None
  ) -> Tuple[np.ndarray, np.ndarray]:
    """Translational and rotational Jacobians of a world point on a body.

    Runs mj_kinematics and mj_comPos per simulation, from its state, then
    mj_jac; the state and bound fields are untouched.

    Args:
      body: body id, or one per simulation of the call, (n,).
      point: world-frame points, (3,) or (n, 3) for the n simulations of the
        call.
      ids: the simulations, or None for all.

    Returns:
      jacp, jacr: (n, 3, nv) each.
    """
    ids = _ids(ids, self.nsim)
    n = self.nsim if ids is None else len(ids)
    num = mujoco.MJTNUM_DTYPE
    body = np.ascontiguousarray(np.broadcast_to(body, (n,)), np.int32)
    point = np.ascontiguousarray(np.broadcast_to(point, (n, 3)), num)
    jacp = np.zeros((n, 3, self.model.nv), num)
    jacr = np.zeros((n, 3, self.model.nv), num)
    self._check(ids, self._b.jac(ids, body, point, jacp, jacr))
    return jacp, jacr

  def ray(
      self,
      pnt: npt.ArrayLike,
      vec: npt.ArrayLike,
      geomgroup: Optional[npt.ArrayLike] = None,
      flg_static: bool = True,
      bodyexclude: npt.ArrayLike = -1,
      ids: Ids = None,
      normal: bool = False,
  ) -> Tuple[np.ndarray, ...]:
    """Intersect rays with each simulation's geoms, as mj_ray.

    Runs mj_kinematics (and mj_flex) per simulation, from its state; the state
    and bound fields are untouched.

    Args:
      pnt: ray origins, broadcast to (n, nray, 3) for the n simulations of the
        call: (3,) is one ray, (nray, 3) the same rays in every simulation.
      vec: ray directions, broadcast likewise.
      geomgroup: geom groups to include, (mjNGROUP,), or None for all.
      flg_static: whether to include static geoms.
      bodyexclude: body whose geoms a ray ignores, or -1; broadcast to (n, nray).
      ids: the simulations, or None for all.
      normal: whether to return the surface normal at each hit.

    Returns:
      dist, geomid: (n, nray) each; dist is -1 and geomid -1 for a miss. With
      normal, also the normals, (n, nray, 3), zero for a miss.
    """
    ids = _ids(ids, self.nsim)
    n = self.nsim if ids is None else len(ids)
    num = mujoco.MJTNUM_DTYPE
    pnt = np.atleast_2d(np.asarray(pnt, num))
    vec = np.atleast_2d(np.asarray(vec, num))
    nray = np.broadcast_shapes(pnt.shape[:-1], vec.shape[:-1])[-1]
    pnt = np.ascontiguousarray(np.broadcast_to(pnt, (n, nray, 3)))
    vec = np.ascontiguousarray(np.broadcast_to(vec, (n, nray, 3)))
    exclude = np.ascontiguousarray(np.broadcast_to(bodyexclude, (n, nray)), np.int32)
    if geomgroup is not None:
      geomgroup = np.ascontiguousarray(geomgroup, dtype=np.uint8)
    dist = np.zeros((n, nray), num)
    geomid = np.zeros((n, nray), np.int32)
    normals = np.zeros((n, nray, 3), num) if normal else None
    self._check(
        ids,
        self._b.ray(
            ids, pnt, vec, geomgroup, flg_static, exclude, dist, geomid, normals
        ),
    )
    return (dist, geomid, normals) if normal else (dist, geomid)

  def _component(self, component: Any) -> Tuple[int, int]:
    return self._b.state_component(int(component))

  def _check(self, ids: Optional[np.ndarray], nfail: int) -> None:
    if not nfail:
      return
    sel = np.arange(self.nsim) if ids is None else ids
    failed = sel[self.status[sel] != 0]
    raise SimulationError(failed, self.error(int(failed[0])))


class _Named:
  """One object's slices of per-simulation fields, (nsim, ...), named as in
  MjData and MjModel.

  batch.joint('hinge').qpos is the part of batch.bind('qpos') that
  data.joint('hinge').qpos is of data.qpos, and batch.body('pole').mass the part
  of batch.expand('body_mass') that model.body('pole').mass is of
  model.body_mass. Data attributes bind their field, model attributes expand
  theirs, and assignment writes every simulation's value.
  """

  # (kind, attribute, model or data) -> the field that holds it
  _fields: Dict[Tuple[str, str, bool], str] = {}

  def __init__(self, batch: Batch, kind: str, key: Union[int, str]):
    data_view = getattr(batch._data, kind)(key)  # pylint: disable=protected-access
    object.__setattr__(self, '_batch', batch)
    object.__setattr__(self, '_kind', kind)
    object.__setattr__(self, '_data_view', data_view)
    object.__setattr__(self, '_model_view', getattr(batch.model, kind)(key))
    object.__setattr__(self, 'id', data_view.id)
    object.__setattr__(self, 'name', data_view.name)

  def __getattr__(self, attr: str) -> np.ndarray:
    batch = self._batch
    sources = (
        (False, self._data_view, batch._data, DATA_FIELDS, batch.bind),  # pylint: disable=protected-access
        (True, self._model_view, batch.model, MODEL_FIELDS, batch.expand),
    )
    for is_model, view, obj, names, rows in sources:
      part = getattr(view, attr, None)
      if not isinstance(part, np.ndarray):
        continue
      key = (self._kind, attr, is_model)
      if key not in self._fields:
        field = _containing(obj, names, part)
        if field is None:
          continue
        self._fields[key] = field
      field = self._fields[key]
      full = getattr(obj, field)
      start = (part.ctypes.data - full.ctypes.data) // full.itemsize
      flat = rows(field).reshape(batch.nsim, -1)
      out = flat[:, start : start + part.size].reshape(batch.nsim, *part.shape)
      assert np.shares_memory(out, flat)
      return out
    raise AttributeError(f'{attr} is not a field of a {self._kind}')

  def __setattr__(self, attr: str, value: Any) -> None:
    """Write value to every simulation's slice: batch.joint('j').qpos = 0.5."""
    if attr in ('id', 'name') or attr.startswith('_'):
      raise AttributeError(f'{attr} is read-only')
    self.__getattr__(attr)[...] = value


def _containing(obj: Any, names: FrozenSet[str], part: np.ndarray) -> Optional[str]:
  """The field of obj among names whose memory holds the array part."""
  for name in names:
    full = getattr(obj, name, None)
    if not isinstance(full, np.ndarray) or full.dtype != part.dtype:
      continue
    offset = part.ctypes.data - full.ctypes.data
    if 0 <= offset and offset + part.nbytes <= full.nbytes and part.size:
      return name
  return None


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


def _field_names() -> Tuple[FrozenSet[str], FrozenSet[str]]:
  """The names bind() and expand() accept, as the batch's own tables decide."""
  model = mujoco.MjModel.from_xml_string('<mujoco/>')
  data = mujoco.MjData(model)
  probe = _batch.Batch(model, 1, 1)
  public = lambda obj: [n for n in dir(obj) if not n.startswith('_')]
  data_fields = set(_STATE_FIELDS) | {
      n
      for n in public(data)
      if n not in _STATE_FIELDS
      and isinstance(getattr(data, n, None), np.ndarray)
      and probe.output(n) is not None
  }
  candidates = [
      n for n in public(model) if isinstance(getattr(model, n, None), np.ndarray)
  ] + [
      'opt.' + n for n in public(model.opt) if not callable(getattr(model.opt, n))
  ]
  model_fields = {n for n in candidates if probe.expand(n) is not None}
  return frozenset(data_fields), frozenset(model_fields)


# MjData fields that bind() accepts, and MjModel fields that expand() accepts
DATA_FIELDS, MODEL_FIELDS = _field_names()
