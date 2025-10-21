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
"""Utilities for testing MJX MjWarp integration."""
import jax
import jax.numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util as mjx_test_util
import numpy as np

try:
  from mujoco.mjx.warp import forward as mjxw_forward  # pylint: disable=g-import-not-at-top
except ImportError:
  mjxw_forward = None

# tolerance for difference between MuJoCo and MJX smooth calculations - mostly
# due to float precision
_TOLERANCE = 5e-5


def assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def assert_attr_eq(a, b, attr):
  assert_eq(getattr(a, attr), getattr(b, attr), attr)


def make_data(
    m: mujoco.MjModel, worldid: int, nconmax: int = 1_000, njmax: int = 100
):
  """Make data for a given worldid using keyframes when available."""
  dx = mjx.make_data(m, impl='warp', nconmax=nconmax, njmax=njmax)

  rng = jax.random.PRNGKey(worldid)
  rng, key = jax.random.split(rng)
  qpos = m.qpos0 + jax.random.uniform(key, (m.nq,), minval=-0.05, maxval=0.05)
  key_qpos = (
      jp.array(m.key_qpos)[worldid] if m.nkey > 0 else jp.zeros_like(qpos)
  )
  qpos = jp.where(worldid < m.nkey, key_qpos, qpos)

  rng, key = jax.random.split(rng)
  qvel = jax.random.uniform(key, (m.nv,), minval=-0.05, maxval=0.05)
  key_qvel = (
      jp.array(m.key_qvel)[worldid] if m.nkey > 0 else jp.zeros_like(qvel)
  )
  qvel = jp.where(worldid < m.nkey, key_qvel, qvel)

  rng, key1, key2 = jax.random.split(rng, 3)
  mocap_pos = jax.random.normal(key1, (m.nmocap, 3))
  key_mpos = (
      jp.array(m.key_mpos)[worldid]
      if m.nkey > 0 and m.nmocap > 0
      else jp.zeros_like(mocap_pos)
  )
  mocap_pos = jp.where(worldid < m.nkey, key_mpos, mocap_pos)
  mocap_quat = jax.random.normal(key2, (m.nmocap, 4))
  key_mquat = (
      jp.array(m.key_mquat)[worldid]
      if m.nkey > 0 and m.nmocap > 0
      else jp.zeros_like(mocap_quat)
  )
  mocap_quat = jp.where(worldid < m.nkey, key_mquat, mocap_quat)

  _, key = jax.random.split(rng)
  ctrl = jax.random.uniform(key, (m.nu,), minval=-0.1, maxval=0.1)
  key_ctrl = (
      jp.array(m.key_ctrl)[worldid] if m.nkey > 0 else jp.zeros_like(ctrl)
  )
  ctrl = jp.where(worldid < m.nkey, key_ctrl, ctrl)

  dx = dx.replace(
      qpos=qpos,
      qvel=qvel,
      mocap_pos=mocap_pos,
      mocap_quat=mocap_quat,
      ctrl=ctrl,
  )

  # Mimic forward call within a vmap trace (since make_data gets called in a
  # vmap in tests). Some fields get vmapped (qpos, qvel) while others need to be
  # broadcasted by the custom vmap rule (e.g. geom_xpos).
  mx = mjx.put_model(m, impl='warp')
  dx = mjxw_forward.forward(mx, dx)

  return dx


def _mjx_contact(dx, worldid: int):
  keys = []
  for i in range(dx._impl.nacon[0]):
    if dx._impl.contact__worldid[i] != worldid:
      continue

    g1, g2 = tuple(map(int, dx._impl.contact__geom[i]))
    dist = float(dx._impl.contact__dist[i])
    keys.append((g1, g2, -dist, i))

  keys = sorted(keys)
  geom1 = np.array([k[0] for k in keys])
  geom2 = np.array([k[1] for k in keys])
  dist = np.array([dx._impl.contact__dist[k[-1]] for k in keys])
  normal = np.array([dx._impl.contact__frame[k[-1]][0] for k in keys])
  return geom1, geom2, dist, normal


def _mj_contact(d):
  keys = []
  for i in range(d.ncon):
    g1, g2 = tuple(map(int, d.contact.geom[i]))
    dist = float(d.contact.dist[i])
    keys.append((g1, g2, -dist, i))

  keys = sorted(keys)
  geom1 = np.array([k[0] for k in keys])
  geom2 = np.array([k[1] for k in keys])
  dist = np.array([d.contact.dist[k[-1]] for k in keys])
  normal = np.array([d.contact.frame[k[-1]][:3] for k in keys])
  return geom1, geom2, dist, normal


def assert_contact_eq(d, dx, worldid: int):
  *geom, dist, normal = _mj_contact(d)
  *geomp, distp, normalp = _mjx_contact(dx, worldid)
  assert_eq(geomp, geom, 'geom')
  assert_eq(distp, dist, 'dist')
  assert_eq(normalp, normal, 'normal')


def _mjx_efc(dx, worldid: int):
  """Gets unpacked efc data for a given worldid."""
  select = lambda x: x if dx._impl.nefc.ndim == 0 else x[worldid]
  nefc = select(dx._impl.nefc)
  keys = np.arange(nefc)
  if not keys.size:
    empty = np.array([])
    return 0, empty, empty, np.zeros((0, dx.qvel.shape[0])), empty, empty
  efc_pos = select(dx._impl.efc__pos[:nefc])
  efc_type = select(dx._impl.efc__type[:nefc])
  efc_d = select(dx._impl.efc__D[:nefc])
  keys_sorted = np.lexsort((-efc_pos, efc_type, efc_d))
  keys = keys[keys_sorted]

  nefc = len(keys)
  type_ = efc_type[keys]
  pos = efc_pos[keys]
  j = select(dx._impl.efc__J[:nefc])[keys]
  aref = select(dx._impl.efc__aref[:nefc])[keys]
  d_ = select(dx._impl.efc__D[:nefc])[keys]
  return nefc, type_, pos, j, aref, d_


def _mj_efc(d):
  """Gets unpacked efc data."""
  efc_j = np.zeros((d.efc_J_rownnz.shape[0], d.qvel.shape[0]))
  if d.efc_J.shape[0] < efc_j.shape[0] * efc_j.shape[1]:
    mujoco.mju_sparse2dense(
        efc_j,
        d.efc_J,
        d.efc_J_rownnz,
        d.efc_J_rowadr,
        d.efc_J_colind,
    )
  else:
    efc_j = d.efc_J.reshape((-1, d.qvel.shape[0]))

  keys = np.lexsort((-d.efc_pos, d.efc_type, d.efc_D))
  type_ = d.efc_type[keys]
  pos = d.efc_pos[keys]
  efc_j = efc_j[keys]
  aref = d.efc_aref[keys]
  d_ = d.efc_D[keys]
  return d.nefc, type_, pos, efc_j, aref, d_


def assert_efc_eq(d, dx, worldid: int):
  nefc, type_, pos, j, aref, d_ = _mj_efc(d)
  nefcp, typep, posp, jp_, arefp, dp = _mjx_efc(dx, worldid)

  assert_eq(nefcp, nefc, 'nefc')
  assert_eq(typep, type_, 'type')
  assert_eq(posp, pos, 'pos')
  assert_eq(jp_, j, 'J')
  assert_eq(arefp, aref, 'aref')
  assert_eq(dp, d_, 'D')


def load_test_file(name: str) -> mujoco.MjModel:
  """Loads a mujoco.MjModel based on the file name."""
  return mjx_test_util.load_test_file(name)
