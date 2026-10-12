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

import copy
import ctypes

from absl.testing import absltest
from absl.testing import parameterized
import mujoco
from mujoco import batch
from mujoco import rollout
import numpy as np

NUM = mujoco.MJTNUM_DTYPE  # float64, or float32 in a single-precision build
# a floating-point type of another element size
OTHER = np.float32 if NUM == np.float64 else np.float64

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

  @parameterized.parameters(False, True)
  def test_step_stage(self, persistent):
    m = self.model
    b = batch.Batch(m, N, 2, persistent)
    xpos, qpos = b.bind('xpos'), b.bind('qpos')
    b.bind('qvel')[:] = 1
    d = mujoco.MjData(m)

    def fresh(i):
      d.qpos[:] = qpos[i]
      mujoco.mj_kinematics(m, d)
      return np.array_equal(xpos[i], d.xpos)

    b.step(nstep=2, stage=mujoco.mjtStage.mjSTAGE_VEL)
    self.assertTrue(all(fresh(i) for i in range(N)))  # positions of the new state
    b.step(nstep=2)  # the default: those before the last substep, as mj_step
    self.assertFalse(any(fresh(i) for i in range(N)))

  def test_reset_stage(self):
    m = self.model
    full, none = batch.Batch(m, N, 2), batch.Batch(m, N, 2)
    for b, stage in ((full, mujoco.mjtStage.mjSTAGE_ACC),
                     (none, mujoco.mjtStage.mjSTAGE_NONE)):
      b.step(nstep=3)
      b.reset(keyframe=0, stage=stage)
      b.qpos[:, 1] = np.linspace(0, 1, N)  # randomization after the reset
      b.forward()
    np.testing.assert_array_equal(none.state, full.state)
    np.testing.assert_array_equal(none.xpos, full.xpos)

  def test_field_sets_and_attributes(self):
    m = self.model
    b = batch.Batch(m, N)
    self.assertIn('xpos', batch.DATA_FIELDS)
    self.assertIn('qpos', batch.DATA_FIELDS)
    self.assertNotIn('contact', batch.DATA_FIELDS)  # not a fixed-size field
    self.assertIn('body_mass', batch.MODEL_FIELDS)
    self.assertIn('opt.gravity', batch.MODEL_FIELDS)
    self.assertNotIn('mesh_vert', batch.MODEL_FIELDS)  # shared, not expandable
    for name in batch.DATA_FIELDS:
      self.assertEqual(b.bind(name).shape[0], N, name)
    for name in batch.MODEL_FIELDS:
      self.assertEqual(b.expand(name).shape[0], N, name)

    np.testing.assert_array_equal(b.xpos, b.bind('xpos'))  # attribute access binds
    b.ctrl = 0.5  # and writes through, as with MjData
    np.testing.assert_array_equal(b.bind('ctrl'), 0.5)
    b.qpos[:, 1] = 0.25
    np.testing.assert_array_equal(b.bind('qpos')[:, 1], 0.25)
    with self.assertRaises(AttributeError):
      b.nope  # pylint: disable=pointless-statement
    with self.assertRaises(AttributeError):
      b.contact  # pylint: disable=pointless-statement

  def test_derived_fields_are_read_only(self):
    b = batch.Batch(self.model, N)
    with self.assertRaises(ValueError):
      b.xpos[0, 0, 0] = 1  # an output: a write would be lost at the next call
    with self.assertRaises(ValueError):
      b.body('pole').xpos = 0
    b.qpos[0, 0] = 1  # inputs stay writable

  @parameterized.parameters(False, True)
  def test_get_model_and_data(self, persistent):
    m = self.model
    b = batch.Batch(m, N, 2, persistent)
    pole = m.body('pole').id
    b.expand('body_mass')[:, pole] = np.linspace(0.1, 1, N)
    b.set_const()
    b.qpos[:, 1] = np.linspace(-1, 1, N)
    xpos = b.xpos  # bound before the call that fills it
    b.forward()
    for sim in (0, 5):
      ms = b.get_model(sim)
      self.assertEqual(ms.body_mass[pole], b.expand('body_mass')[sim, pole])
      ds = b.get_data(sim)
      np.testing.assert_array_equal(ds.qpos, b.qpos[sim])
      mujoco.mj_forward(ms, ds)
      np.testing.assert_array_equal(ds.xpos, xpos[sim])
    # into given objects
    ms, ds = mujoco.MjModel.from_xml_string(XML), mujoco.MjData(m)
    self.assertIs(b.get_model(5, ms), ms)
    self.assertIs(b.get_data(5, ds), ds)
    with self.assertRaises(Exception):
      b.get_data(N)

  def test_named_model_fields(self):
    m = self.model
    b = batch.Batch(m, N)
    pole = b.body('pole')
    np.testing.assert_array_equal(pole.mass, np.full((N, 1), m.body('pole').mass))
    pole.mass = np.arange(1, N + 1).reshape(N, 1)  # per simulation, as expand()
    np.testing.assert_array_equal(b.expand('body_mass')[:, pole.id], np.arange(1, N + 1))
    hinge = b.joint('hinge')  # joint attributes in dof arrays
    hinge.damping = 0.5
    np.testing.assert_array_equal(b.expand('dof_damping')[:, m.joint('hinge').dofadr[0]], 0.5)
    self.assertEqual(b.geom(0).size.shape, (N, 3))
    np.testing.assert_array_equal(hinge.qpos, b.qpos[:, 1:2])  # data attributes still bind

  def test_reset_keyframe_per_simulation(self):
    xml = XML.replace(
        '</keyframe>', '<key qpos="-0.3 -0.5" ctrl="0.2 0.1" act="0.1"/></keyframe>'
    )
    m = mujoco.MjModel.from_xml_string(xml)
    b = batch.Batch(m, N, 2)
    b.step(nstep=5)
    keys = np.array([0, -1, 1, 1, 0, -1, 0, 1])
    b.reset(keyframe=keys)
    d = mujoco.MjData(m)
    for i, key in enumerate(keys):
      if key < 0:
        mujoco.mj_resetData(m, d)
      else:
        mujoco.mj_resetDataKeyframe(m, d, key)
      mujoco.mj_forward(m, d)
      np.testing.assert_array_equal(b.qpos[i], d.qpos)
    b.reset([2, 3], keyframe=1)  # one keyframe for every simulation of the call
    np.testing.assert_array_equal(b.qpos[2], b.qpos[3])
    with self.assertRaises(Exception):
      b.reset(keyframe=2)

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

  def test_set_const_refits_body_hierarchies(self):
    m = mujoco.MjModel.from_xml_string("""
    <mujoco>
      <asset><mesh name="tet" vertex="0 0 0  .2 0 0  0 .2 0  0 0 .2"/></asset>
      <worldbody>
        <body>
          <freejoint/>
          <geom name="ball" size=".05" pos=".1 0 0"/>
          <geom size=".05" pos="-.1 0 0"/>
          <geom type="mesh" mesh="tet"/>
        </body>
      </worldbody>
    </mujoco>
    """)
    ball = m.geom('ball').id
    b = batch.Batch(m, N, 2)
    b.geom('ball').size = np.linspace(0.05, 0.2, N).reshape(N, 1)
    b.set_const()
    self.assertEqual(b.expand('bvh_aabb').shape, (N, m.mesh_bvhadr[0], 6))
    size = b.expand('geom_size')
    for i in range(N):
      mi = copy.copy(m)
      mi.geom_size[ball] = size[i, ball]
      mujoco.mj_setConst(mi, mujoco.MjData(mi))
      np.testing.assert_array_equal(b.get_model(i).bvh_aabb, mi.bvh_aabb)

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

  def test_set_const_errors_name_its_simulations(self):
    m = mujoco.MjModel.from_xml_string("""
      <mujoco><worldbody><body>
        <joint type="slide"/><geom size=".1" mass="1" contype="0" conaffinity="0"/>
      </body></worldbody></mujoco>""")
    b = batch.Batch(m, 3)
    b.expand('body_mass')[:, 1] = [0, 2, 0]  # zero mass: mj_setConst raises
    with self.assertRaises(batch.SimulationError) as e:
      b.set_const([0, 1])
    np.testing.assert_array_equal(e.exception.ids, [0])
    self.assertEqual(b.status[2], 0)  # not recomputed

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
    initial = np.empty((len(ids), nstate), NUM)
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

  def test_rollout_out(self):
    b = batch.Batch(self.model, N)
    qvel = mujoco.mjtState.mjSTATE_QVEL
    qpos = np.empty((N, 3, self.model.nq), NUM)
    out = b.rollout(3, record=('qpos', qvel), out={'qpos': qpos})
    self.assertIs(out['qpos'], qpos)  # recorded into the given array
    self.assertEqual(out[qvel].shape, (N, 3, self.model.nv))
    np.testing.assert_array_equal(qpos[:, -1], b.bind('qpos'))
    with self.assertRaises(ValueError):
      b.rollout(
          3, record=('qpos',), out={'qpos': np.empty((N, 2, self.model.nq), NUM)}
      )
    with self.assertRaises(ValueError):
      b.rollout(3, record=('qpos',), out={'qpos': qpos.astype(OTHER)})

  def test_rollout_checks_buffers(self):
    b = batch.Batch(self.model, N)
    m = self.model
    ctrl = int(mujoco.mjtState.mjSTATE_CTRL)
    good = np.empty((N, 3, m.nq), NUM)
    b._b.rollout(None, 3, None, ctrl, [('qpos', 0, good)])
    for record in (
        ('qpos', 0, np.empty((N, 1, m.nq), NUM)),  # too short
        ('qpos', 0, np.empty((N, 3, m.nq), OTHER)),  # wrong element size
        (None, int(mujoco.mjtState.mjSTATE_QVEL), np.empty((N, 3, m.nv + 1), NUM)),
        (None, -1, good),  # invalid spec
        ('nope', 0, good),
    ):
      with self.assertRaises(ValueError):
        b._b.rollout(None, 3, None, ctrl, [record])
    with self.assertRaises(ValueError):  # control too short
      b._b.rollout(None, 3, np.zeros((N, 2, m.nu), NUM), ctrl, [])

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
      ep, er = np.zeros((3, m.nv), NUM), np.zeros((3, m.nv), NUM)
      mujoco.mj_jac(m, d, ep, er, point[i], pole)
      np.testing.assert_array_equal(jacp[i], ep)
      np.testing.assert_array_equal(jacr[i], er)
    sub = np.array([1, 6])
    jacp_sub, _ = b.jac(pole, point[sub], ids=sub)
    np.testing.assert_array_equal(jacp_sub, jacp[sub])

    # a body per simulation
    cart = m.body('cart').id
    bodies = np.where(np.arange(N) % 2, pole, cart)
    jacp_mixed, _ = b.jac(bodies, point)
    for i in range(N):
      d.qpos[:] = qpos[i]
      mujoco.mj_kinematics(m, d)
      mujoco.mj_comPos(m, d)
      ep = np.zeros((3, m.nv), NUM)
      mujoco.mj_jac(m, d, ep, None, point[i], bodies[i])
      np.testing.assert_array_equal(jacp_mixed[i], ep)
    with self.assertRaises(ValueError):
      b.jac(m.nbody, point)

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

    # normals, and a body excluded per ray
    pole = m.body('pole').id
    exclude = np.where(np.arange(nray) % 2, pole, -1)
    dist2, geomid2, normal = b.ray(pnt, vec, bodyexclude=exclude, normal=True)
    for i in range(N):
      d.qpos[:] = b.bind('qpos')[i]
      mujoco.mj_kinematics(m, d)
      for k in range(nray):
        gid, nrm = np.zeros(1, np.int32), np.zeros(3, NUM)
        expected = mujoco.mj_ray(
            m, d, pnt[i, k], vec[i, k], None, 1, exclude[k], gid, nrm
        )
        self.assertEqual(dist2[i, k], expected)
        self.assertEqual(geomid2[i, k], gid[0])
        np.testing.assert_array_equal(normal[i, k], nrm)

    # rays broadcast: one ray, and the same rays in every simulation
    one, _ = b.ray(pnt[0, 0], vec[0, 0])
    self.assertEqual(one.shape, (N, 1))
    np.testing.assert_array_equal(one[:, 0], dist[:, 0])
    same, _ = b.ray(pnt[0], vec[0])
    self.assertEqual(same.shape, (N, nray))
    np.testing.assert_array_equal(same, dist)  # pnt, vec are the same in every simulation
    sub = np.array([2, 5])
    dist_sub, geomid_sub = b.ray(pnt[sub], vec[sub], ids=sub)
    np.testing.assert_array_equal(dist_sub, dist[sub])
    np.testing.assert_array_equal(geomid_sub, geomid[sub])


if __name__ == '__main__':
  absltest.main()
