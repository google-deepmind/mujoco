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
"""Tests for mujoco.batch."""

import ctypes

from absl.testing import absltest
from absl.testing import parameterized
import mujoco
from mujoco import batch
from mujoco import rollout
import numpy as np

XML = """
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="2 2 .1"/>
    <body name="cart" pos="0 0 .1">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="box" size=".1 .1 .05" mass="1"/>
      <body name="pole" pos="0 0 .05">
        <joint name="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 .5" size=".02" mass=".1"/>
        <site name="tip" pos="0 0 .5"/>
      </body>
    </body>
    <body name="mocap" mocap="true" pos="1 0 1">
      <geom type="sphere" size=".02" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="slide" gear="5"/>
    <general joint="hinge" dyntype="filter" dynprm="0.02" gainprm="5"
             biastype="affine" biasprm="0 -5 0"/>
  </actuator>
  <sensor>
    <jointpos joint="hinge"/>
    <accelerometer site="tip"/>
  </sensor>
  <keyframe>
    <key qpos="0.3 0.5" ctrl="0.1 0.2" act="0.2"/>
  </keyframe>
</mujoco>
"""

SLEEP_XML = """
<mujoco>
  <option integrator="implicitfast" viscosity="10" sleep_tolerance="0.01">
    <flag sleep="enable" gravity="disable" constraint="disable" contact="disable"/>
  </option>
  <worldbody>
    <body><freejoint/><geom type="box" size=".1 .2 .3" mass="1"/></body>
  </worldbody>
</mujoco>
"""
N = 8


class BatchTest(parameterized.TestCase):

  def setUp(self):
    super().setUp()
    self.model = mujoco.MjModel.from_xml_string(XML)

  @parameterized.parameters((1, False), (3, False), (3, True))
  def test_lockstep(self, nthread, persistent):
    m = self.model
    b = batch.Batch(m, N, nthread, persistent)
    self.assertEqual(b.nthread, nthread)
    self.assertEqual(b.persistent, persistent)
    qpos, ctrl = b.bind('qpos'), b.bind('ctrl')
    sensordata, xpos = b.bind('sensordata'), b.bind('xpos')
    ref = [mujoco.MjData(m) for _ in range(N)]
    rng = np.random.default_rng(0)
    for call in range(30):
      u = rng.uniform(-1, 1, (N, m.nu))
      ctrl[:] = u
      for i, d in enumerate(ref):
        d.ctrl[:] = u[i]
      if call == 12:
        ids = np.array([1, 4])
        b.reset(ids, keyframe=0)
        for i in ids:
          mujoco.mj_resetDataKeyframe(m, ref[i], 0)
          mujoco.mj_forward(m, ref[i])
        ran = ids
      else:
        mask = np.arange(N) % 3 != 2 if call % 5 == 4 else None
        b.step(mask, nstep=2)
        ran = np.arange(N) if mask is None else np.flatnonzero(mask)
        for i in ran:
          for _ in range(2):
            mujoco.mj_step(m, ref[i])
      for i in ran:
        np.testing.assert_array_equal(qpos[i], ref[i].qpos)
        np.testing.assert_array_equal(sensordata[i], ref[i].sensordata)
        np.testing.assert_array_equal(xpos[i], ref[i].xpos)

  def test_bind_shapes(self):
    b = batch.Batch(self.model, N)
    self.assertEqual(b.bind('time').shape, (N,))
    self.assertEqual(b.bind('qpos').shape, (N, self.model.nq))
    self.assertEqual(b.bind('mocap_quat').shape, (N, 1, 4))
    self.assertEqual(b.bind('xmat').shape, (N, self.model.nbody, 9))
    self.assertEqual(b.state.shape, (N, b.nstate))
    with self.assertRaises(ValueError):
      b.bind('nope')
    with self.assertRaises(ValueError):
      b.status[0] = 1  # read-only

  def test_named(self):
    m = self.model
    b = batch.Batch(m, N, 2)
    b.reset(keyframe=0)
    hinge = b.joint('hinge')
    self.assertEqual((hinge.id, hinge.name), (1, 'hinge'))
    hinge.qpos[:, 0] = np.arange(N) * 0.1  # a view of the state rows
    b.actuator(0).ctrl[:, 0] = 0.5
    views = [
        (kind, key, attr, getattr(getattr(b, kind)(key), attr))
        for kind, key, attr in (
            ('joint', 'hinge', 'qpos'),
            ('joint', 'hinge', 'qacc'),
            ('body', 'pole', 'xpos'),
            ('body', 'pole', 'xquat'),
            ('site', 'tip', 'xmat'),
            ('sensor', 1, 'data'),
            ('actuator', 1, 'force'),
        )
    ]
    b.step()

    data = mujoco.MjData(m)
    for i in range(N):
      mujoco.mj_resetDataKeyframe(m, data, 0)
      mujoco.mj_forward(m, data)
      data.qpos[1] = i * 0.1
      data.ctrl[0] = 0.5
      mujoco.mj_step(m, data)
      for kind, key, attr, got in views:
        expected = getattr(getattr(data, kind)(key), attr)
        np.testing.assert_array_equal(got[i], expected, f'{kind} {attr}')
    with self.assertRaises(AttributeError):
      b.joint('hinge').nope
    with self.assertRaises(KeyError):
      b.body('nope')

  def test_named_assignment(self):
    b = batch.Batch(self.model, N)
    b.joint('hinge').qpos = 0.5  # writes through, as with MjData
    np.testing.assert_array_equal(b.bind('qpos')[:, 1], 0.5)
    b.actuator(1).ctrl = np.arange(N).reshape(N, 1)
    np.testing.assert_array_equal(b.bind('ctrl')[:, 1], np.arange(N))
    with self.assertRaises(AttributeError):
      b.joint('hinge').id = 0
    with self.assertRaises(AttributeError):
      b.joint('hinge').nope = 0

  def test_expand_and_set_const(self):
    m = self.model
    pole = m.body('pole').id
    b = batch.Batch(m, N, 4)
    mass = b.expand('body_mass')
    mass[:, pole] = np.linspace(0.1, 1.0, N)
    b.set_const()
    np.testing.assert_array_equal(
        b.expand('body_subtreemass')[:, pole], mass[:, pole]
    )
    b.bind('ctrl')[:, 0] = 1.0
    b.step(nstep=10)
    for i in range(N):
      mi = mujoco.MjModel.from_xml_string(XML)
      mi.body_mass[pole] = mass[i, pole]
      di = mujoco.MjData(mi)
      mujoco.mj_setConst(mi, di)
      mujoco.mj_resetData(mi, di)
      di.ctrl[0] = 1.0
      for _ in range(10):
        mujoco.mj_step(mi, di)
      np.testing.assert_array_equal(b.bind('qvel')[i], di.qvel)

  def test_expand_option(self):
    m = self.model
    b = batch.Batch(m, N, 2)
    gravity = b.expand('opt.gravity')
    self.assertEqual(gravity.shape, (N, 3))
    np.testing.assert_array_equal(gravity, np.tile(m.opt.gravity, (N, 1)))
    iterations = b.expand('opt.iterations')
    self.assertEqual(iterations.shape, (N,))
    self.assertEqual(iterations.dtype, np.int32)
    gravity[:, 2] = -np.arange(N)
    b.step(nstep=5)
    for i in range(N):
      mi = mujoco.MjModel.from_xml_string(XML)
      mi.opt.gravity[2] = -i
      di = mujoco.MjData(mi)
      for _ in range(5):
        mujoco.mj_step(mi, di)
      np.testing.assert_array_equal(b.bind('qpos')[i], di.qpos)

  def test_errors_name_every_simulation(self):
    xml = XML.replace(
        '</worldbody>',
        '</worldbody><equality><weld body1="mocap" body2="pole"/></equality>',
    )
    m = mujoco.MjModel.from_xml_string(xml)
    b = batch.Batch(m, N, 2)
    eq_type = b.expand('eq_type')
    eq_type[[3, 6]] = 99  # an unknown constraint type: mj_step raises
    time = b.bind('time')
    with self.assertRaisesRegex(batch.SimulationError, r'\[3, 6\]') as e:
      b.step()
    np.testing.assert_array_equal(e.exception.ids, [3, 6])
    np.testing.assert_array_equal(np.flatnonzero(b.status), [3, 6])
    self.assertNotEmpty(b.error(3))
    self.assertEmpty(b.error(0))
    self.assertEqual(time[3], 0)  # it kept its state
    self.assertTrue(np.all(np.delete(time, [3, 6]) > 0))  # the others ran
    eq_type[[3, 6]] = m.eq_type[0]
    b.step()
    self.assertFalse(np.any(b.status))

  def test_set_const_errors_name_recomputed_simulations(self):
    m = mujoco.MjModel.from_xml_string("""
      <mujoco><worldbody><body>
        <joint type="slide"/><geom size=".1" mass="1" contype="0" conaffinity="0"/>
      </body></worldbody></mujoco>""")
    b = batch.Batch(m, 3)
    b.expand('body_mass')[:, 1] = [0, 2, 0]  # zero mass: mj_setConst raises
    with self.assertRaises(batch.SimulationError) as e:
      b.set_const([0, 1])  # the fields it changes are expanded: all are recomputed
    np.testing.assert_array_equal(e.exception.ids, [0, 2])

  def test_bad_ids(self):
    b = batch.Batch(self.model, N)
    time = b.bind('time')
    with self.assertRaises(mujoco.FatalError):
      b.step([3, 1])
    # rejected before narrowing to int32, rather than wrapped or truncated to 0
    for bad in ([2**32], [0.9], [[0]], [N], [-1], np.ones(N + 1, bool)):
      with self.assertRaises(ValueError, msg=str(bad)):
        b.step(bad)
    np.testing.assert_array_equal(time, 0)
    b.step([])  # an empty selection runs nothing
    np.testing.assert_array_equal(time, 0)
    b.step()  # still usable
    self.assertTrue(np.all(time > 0))

  def test_sleep_needs_persistent(self):
    m = mujoco.MjModel.from_xml_string(SLEEP_XML)
    with self.assertRaisesRegex(ValueError, 'persistent'):
      batch.Batch(m, N)
    b = batch.Batch(m, N, 3, persistent=True)
    qvel = b.bind('qvel')
    ref = [mujoco.MjData(m) for _ in range(N)]
    for i, d in enumerate(ref):
      qvel[i] = d.qvel[:] = 0.1 * (i + 1)
    for _ in range(50):
      b.step(nstep=10)
      for d in ref:
        for _ in range(10):
          mujoco.mj_step(m, d)
    self.assertTrue(any(d.ntree_awake < m.ntree for d in ref))
    for i, d in enumerate(ref):
      np.testing.assert_array_equal(qvel[i], d.qvel)
      np.testing.assert_array_equal(b.bind('qpos')[i], d.qpos)

  @parameterized.parameters(1, 3)
  def test_rollout_matches_mujoco_rollout(self, nthread):
    m = self.model
    nstep, ids = 7, np.array([0, 2, 5])
    b = batch.Batch(m, N, nthread)
    rng = np.random.default_rng(1)
    control = rng.uniform(-1, 1, (len(ids), nstep, m.nu))
    full = mujoco.mjtState.mjSTATE_FULLPHYSICS
    out = b.rollout(nstep, control, record=(full, 'sensordata'), ids=ids)

    nstate = mujoco.mj_stateSize(m, full)
    initial = np.empty((len(ids), nstate))
    d = mujoco.MjData(m)
    mujoco.mj_getState(m, d, initial[0], full)
    initial[:] = initial[0]
    state, sensordata = rollout.rollout(m, d, initial, control)
    np.testing.assert_array_equal(out[full], state)
    np.testing.assert_array_equal(out['sensordata'], sensordata)
    # the rows continue from the last state; the others did not move
    np.testing.assert_array_equal(b.bind('time')[ids], state[:, -1, 0])
    self.assertEqual(b.bind('time')[1], 0)

  def test_rollout_records(self):
    b = batch.Batch(self.model, N)
    out = b.rollout(3, record=('qpos', 'xmat', mujoco.mjtState.mjSTATE_QVEL))
    self.assertEqual(out['qpos'].shape, (N, 3, self.model.nq))
    self.assertEqual(out['xmat'].shape, (N, 3, self.model.nbody, 9))
    self.assertEqual(out[mujoco.mjtState.mjSTATE_QVEL].shape, (N, 3, self.model.nv))
    with self.assertRaises(ValueError):
      b.rollout(3, record=('nope',))

  def test_rollout_checks_buffers(self):
    b = batch.Batch(self.model, N)
    m = self.model
    ctrl = int(mujoco.mjtState.mjSTATE_CTRL)
    good = np.empty((N, 3, m.nq))
    b._b.rollout(None, 3, None, ctrl, [('qpos', 0, good)])
    for record in (
        ('qpos', 0, np.empty((N, 1, m.nq))),  # too short
        ('qpos', 0, np.empty((N, 3, m.nq), np.float32)),  # wrong element size
        (None, int(mujoco.mjtState.mjSTATE_QVEL), np.empty((N, 3, m.nv + 1))),
        (None, -1, good),  # invalid spec
        ('nope', 0, good),
    ):
      with self.assertRaises(ValueError):
        b._b.rollout(None, 3, None, ctrl, [record])
    with self.assertRaises(ValueError):  # control too short
      b._b.rollout(None, 3, np.zeros((N, 2, m.nu)), ctrl, [])

  def test_python_callbacks_rejected(self):
    b = batch.Batch(self.model, N, 2)
    b.step()  # validates the installed callbacks
    mujoco.set_mjcb_control(lambda m, d: None)
    try:
      with self.assertRaisesRegex(ValueError, 'mjcb_control'):
        b.step()
    finally:
      mujoco.set_mjcb_control(None)
    b.step()

    # a C function, here a ctypes thunk, is called with the batch's model and data
    calls = []
    cfunc = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)(
        lambda m, d: calls.append(1)
    )
    mujoco.set_mjcb_control(cfunc)
    try:
      b.step()
    finally:
      mujoco.set_mjcb_control(None)
    self.assertLen(calls, N)

  def test_jac(self):
    m = self.model
    b = batch.Batch(m, N, 3)
    qpos = b.bind('qpos')
    qpos[:, 1] = np.linspace(-1, 1, N)
    pole = m.body('pole').id
    point = np.random.default_rng(2).uniform(-1, 1, (N, 3))
    jacp, jacr = b.jac(pole, point)
    d = mujoco.MjData(m)
    for i in range(N):
      d.qpos[:] = qpos[i]
      mujoco.mj_kinematics(m, d)
      mujoco.mj_comPos(m, d)
      ep, er = np.zeros((3, m.nv)), np.zeros((3, m.nv))
      mujoco.mj_jac(m, d, ep, er, point[i], pole)
      np.testing.assert_array_equal(jacp[i], ep)
      np.testing.assert_array_equal(jacr[i], er)
    sub = np.array([1, 6])
    jacp_sub, _ = b.jac(pole, point[sub], ids=sub)
    np.testing.assert_array_equal(jacp_sub, jacp[sub])

  def test_ray(self):
    m = self.model
    b = batch.Batch(m, N, 3)
    b.bind('qpos')[:, 0] = np.linspace(-0.5, 0.5, N)
    nray = 5
    pnt = np.tile([0.0, -1.0, 0.12], (N, nray, 1))
    pnt[:, :, 0] = np.linspace(-0.6, 0.6, nray)
    vec = np.tile([0.0, 1.0, 0.0], (N, nray, 1))
    dist, geomid = b.ray(pnt, vec)
    d = mujoco.MjData(m)
    hits = 0
    for i in range(N):
      d.qpos[:] = b.bind('qpos')[i]
      mujoco.mj_kinematics(m, d)
      for k in range(nray):
        gid = np.zeros(1, np.int32)
        expected = mujoco.mj_ray(m, d, pnt[i, k], vec[i, k], None, 1, -1, gid, None)
        self.assertEqual(dist[i, k], expected)
        self.assertEqual(geomid[i, k], gid[0])
        hits += gid[0] >= 0
    self.assertGreater(hits, 0)


if __name__ == '__main__':
  absltest.main()
