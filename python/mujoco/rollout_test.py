# Copyright 2022 DeepMind Technologies Limited
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
"""tests for rollout function."""

import concurrent.futures
import threading

from absl.testing import absltest
from absl.testing import parameterized
import mujoco
from mujoco import rollout
import numpy as np

# -------------------------- models used for testing ---------------------------

TEST_XML = r"""
<mujoco>
  <worldbody>
    <light pos="0 0 2"/>
    <geom type="plane" size="5 5 .1"/>
    <body pos="0 0 .1">
      <joint name="yaw" axis="0 0 1"/>
      <joint name="pitch" axis="0 1 0"/>
      <geom type="capsule" size=".02" fromto="0 0 0 1 0 0"/>
      <geom type="box" pos="1 0 0" size=".1 .1 .1"/>
      <site name="site" pos="1 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <general joint="pitch" gainprm="100"/>
    <general joint="yaw" dyntype="filter" dynprm="1" gainprm="100"/>
  </actuator>
  <sensor>
    <accelerometer site="site"/>
  </sensor>
</mujoco>
"""

TEST_XML_NO_SENSORS = r"""
<mujoco>
  <worldbody>
    <light pos="0 0 2"/>
    <geom type="plane" size="5 5 .1"/>
    <body pos="0 0 .1">
      <joint name="yaw" axis="0 0 1"/>
      <joint name="pitch" axis="0 1 0"/>
      <geom type="capsule" size=".02" fromto="0 0 0 1 0 0"/>
      <geom type="box" pos="1 0 0" size=".1 .1 .1"/>
      <site name="site" pos="1 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <general joint="pitch" gainprm="100"/>
    <general joint="yaw" dyntype="filter" dynprm="1" gainprm="100"/>
  </actuator>
</mujoco>
"""

TEST_XML_NO_ACTUATORS = r"""
<mujoco>
  <worldbody>
    <light pos="0 0 2"/>
    <geom type="plane" size="5 5 .1"/>
    <body pos="0 0 .1">
      <joint name="yaw" axis="0 0 1"/>
      <joint name="pitch" axis="0 1 0"/>
      <geom type="capsule" size=".02" fromto="0 0 0 1 0 0"/>
      <geom type="box" pos="1 0 0" size=".1 .1 .1"/>
      <site name="site" pos="1 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <accelerometer site="site"/>
  </sensor>
</mujoco>
"""

TEST_XML_MOCAP = r"""
<mujoco>
  <worldbody>
    <body name="1" mocap="true">
    </body>
    <body name="2" mocap="true">
    </body>
  </worldbody>
  <sensor>
    <framepos objtype="xbody" objname="1"/>
    <framequat objtype="xbody" objname="1"/>
  </sensor>
</mujoco>
"""

TEST_XML_EMPTY = r"""
<mujoco>
</mujoco>
"""

TEST_XML_DIVERGE = r"""
<mujoco>
  <option>
    <flag gravity="disable"/>
  </option>

  <worldbody>
    <geom type="plane" size="5 5 .1"/>
    <body pos="0 0 -.3" euler="30 45 90">
      <freejoint/>
      <geom type="box" size=".1 .2 .4"/>
    </body>
  </worldbody>

  <keyframe>
    <key name="non-diverging" qpos="0 0 .5 1 0 0 0"/>
  </keyframe>
</mujoco>
"""

ALL_MODELS = {'TEST_XML': TEST_XML,
              'TEST_XML_NO_SENSORS': TEST_XML_NO_SENSORS,
              'TEST_XML_NO_ACTUATORS': TEST_XML_NO_ACTUATORS,
              'TEST_XML_EMPTY': TEST_XML_EMPTY}

# ------------------------------ tests -----------------------------------------


class MuJoCoRolloutTest(parameterized.TestCase):

  def setUp(self):
    super().setUp()
    np.random.seed(42)

  # ----------------------------- test basic operation

  @parameterized.parameters(ALL_MODELS.keys())
  def test_single_step(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    initial_state = np.random.randn(nstate)
    control = np.random.randn(model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, control)

    mujoco.mj_resetData(model, data)
    py_state, py_sensordata = py_rollout(model, data, initial_state, control)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  @parameterized.parameters(ALL_MODELS.keys())
  def test_one_rollout(self, model_name):
    nstep = 3  # number of timesteps

    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    initial_state = np.random.randn(nstate)
    control = np.random.randn(nstep, model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, control)

    py_state, py_sensordata = py_rollout(model, data, initial_state, control)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  @parameterized.parameters(ALL_MODELS.keys())
  def test_multi_step(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    nroll = 5  # number of rollouts
    nstep = 1  # number of steps

    initial_state = np.random.randn(nroll, nstate)
    control = np.random.randn(nroll, nstep, model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, control)

    mujoco.mj_resetData(model, data)
    py_state, py_sensordata = py_rollout(model, data, initial_state, control)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  @parameterized.parameters(ALL_MODELS.keys())
  def test_one_rollout_fixed_ctrl(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    nroll = 1  # number of rollouts
    nstep = 3  # number of steps

    initial_state = np.random.randn(nstate)
    control = np.random.randn(model.nu)
    state = np.empty((nroll, nstep, nstate))
    sensordata = np.empty((nroll, nstep, model.nsensordata))
    rollout.rollout(model, data, initial_state, control,
                    state=state, sensordata=sensordata)

    control = np.tile(control, (nstep, 1))
    py_state, py_sensordata = py_rollout(model, data, initial_state, control)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  @parameterized.parameters(ALL_MODELS.keys())
  def test_multi_rollout(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    nroll = 2  # number of initial states
    nstep = 3  # number of timesteps

    initial_state = np.random.randn(nroll, nstate)
    control = np.random.randn(nroll, nstep, model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, control)

    py_state, py_sensordata = py_rollout(model, data, initial_state, control)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  @parameterized.parameters(ALL_MODELS.keys())
  def test_multi_rollout_fixed_ctrl_infer_from_output(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    nroll = 2  # number of rollouts
    nstep = 3  # number of timesteps

    initial_state = np.random.randn(nroll, nstate)
    control = np.random.randn(nroll, 1, model.nu)
    state = np.empty((nroll, nstep, nstate))
    state, sensordata = rollout.rollout(model, data, initial_state, control,
                                        state=state)

    control = np.repeat(control, nstep, axis=1)
    py_state, py_sensordata = py_rollout(model, data, initial_state, control)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  @parameterized.parameters(ALL_MODELS.keys())
  def test_py_rollout_generalized_control(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    nroll = 4  # number of rollouts
    nstep = 3  # number of timesteps

    initial_state = np.random.randn(nroll, nstate)

    control_spec = (mujoco.mjtState.mjSTATE_CTRL |
                    mujoco.mjtState.mjSTATE_QFRC_APPLIED |
                    mujoco.mjtState.mjSTATE_XFRC_APPLIED)
    ncontrol = mujoco.mj_stateSize(model, control_spec)
    control = np.random.randn(nroll, nstep, ncontrol)

    state, sensordata = rollout.rollout(model, data, initial_state, control,
                                        control_spec=control_spec)

    py_state, py_sensordata = py_rollout(model, data, initial_state, control,
                                         control_spec=control_spec)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  def test_detect_divergence(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML_DIVERGE)
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    nroll = 4  # number of rollouts
    initial_state = np.empty((nroll, nstate))

    # get diverging (0, 2) and non-diverging (1, 3) states
    mujoco.mj_getState(model, data, initial_state[0],
                       mujoco.mjtState.mjSTATE_FULLPHYSICS)
    mujoco.mj_getState(model, data, initial_state[2],
                       mujoco.mjtState.mjSTATE_FULLPHYSICS)
    mujoco.mj_resetDataKeyframe(model, data, 0)  # keyframe 0 does not diverge
    mujoco.mj_getState(model, data, initial_state[1],
                       mujoco.mjtState.mjSTATE_FULLPHYSICS)
    mujoco.mj_getState(model, data, initial_state[3],
                       mujoco.mjtState.mjSTATE_FULLPHYSICS)

    nstep = 10000  # divergence after ~15s, timestep = 2e-3

    state = np.random.randn(nroll, nstep, nstate)

    rollout.rollout(model, data, initial_state, state=state)

    # initial_state[0,2] diverged, final timesteps are identical
    assert state[0][-1][0] == state[0][-2][0]
    assert state[2][-1][0] == state[2][-2][0]

    # initial_state[1,3] did not diverge, final timesteps are different
    assert state[1][-1][0] != state[1][-2][0]
    assert state[3][-1][0] != state[3][-2][0]

  # ----------------------------- test threaded operation

  def test_threading(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    num_workers = 32
    nroll = 10000
    nstep = 5
    initial_state = np.random.randn(nroll, nstate)
    state = np.empty((nroll, nstep, nstate))
    sensordata = np.empty((nroll, nstep, model.nsensordata))
    control = np.random.randn(nroll, nstep, model.nu)

    thread_local = threading.local()

    def thread_initializer():
      thread_local.data = mujoco.MjData(model)

    def call_rollout(initial_state, control, state, sensordata):
      rollout.rollout(model, thread_local.data, initial_state, control,
                      skip_checks=True, nroll=initial_state.shape[0],
                      nstep=nstep, state=state, sensordata=sensordata)

    n = nroll // num_workers  # integer division
    chunks = []  # a list of tuples, one per worker
    for i in range(num_workers-1):
      chunks.append((initial_state[i*n:(i+1)*n],
                     control[i*n:(i+1)*n],
                     state[i*n:(i+1)*n],
                     sensordata[i*n:(i+1)*n]))

    # last chunk, absorbing the remainder:
    chunks.append((initial_state[(num_workers-1)*n:],
                   control[(num_workers-1)*n:],
                   state[(num_workers-1)*n:],
                   sensordata[(num_workers-1)*n:]))

    with concurrent.futures.ThreadPoolExecutor(
        max_workers=num_workers, initializer=thread_initializer) as executor:
      futures = []
      for chunk in chunks:
        futures.append(executor.submit(call_rollout, *chunk))
      for future in concurrent.futures.as_completed(futures):
        future.result()

    data = mujoco.MjData(model)
    py_state, py_sensordata = py_rollout(model, data, initial_state, control)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  # ---------------------------- test advanced operation

  def test_warmstart(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    # take one step, save the state
    state0 = np.zeros(nstate)
    control = np.zeros(model.nu)
    state1, _ = step(model, data, state0, control)

    # save qacc_warmstart
    initial_warmstart = data.qacc_warmstart.copy()

    # take one more step (uses correct warmstart)
    state2, _ = step(model, data, state1[0], control)

    # take step using rollout, don't take warmstart into account
    state, _ = rollout.rollout(model, data, state1[0], control)

    # assert that stepping without warmstarts is not exact
    np.testing.assert_raises(AssertionError,
                             np.testing.assert_array_equal, state, state2)

    # take step using rollout, take warmstart into account
    state, _ = rollout.rollout(model, data, state1, control,
                               initial_warmstart=initial_warmstart)

    # assert exact equality
    np.testing.assert_array_equal(state, np.expand_dims(state2, axis=0))

  def test_mocap(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML_MOCAP)
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    initial_state = np.zeros(nstate)

    control_spec = (mujoco.mjtState.mjSTATE_MOCAP_POS |
                    mujoco.mjtState.mjSTATE_MOCAP_QUAT)

    pos1 = np.array((1., 2., 3.))
    quat1 = np.array((1., 2., 3., 4.))
    quat1 /= np.linalg.norm(quat1)
    pos2 = np.array((2., 3., 4.))
    quat2 = np.array((2., 3., 4., 5.))
    quat2 /= np.linalg.norm(quat2)
    control = np.hstack((pos1, pos2, quat1, quat2))

    _, sensordata = rollout.rollout(model, data, initial_state, control,
                                    control_spec=control_spec)

    np.testing.assert_array_almost_equal(sensordata[0][0][:3], pos1)
    np.testing.assert_array_almost_equal(sensordata[0][0][3:], quat1)

  # ---------------------------- test correctness

  def test_intercept_mj_errors(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    nroll = 1
    nstep = 3

    initial_state = np.zeros((nroll, nstate))
    ctrl = np.zeros((nroll, nstep, model.nu))

    model.opt.solver = 10  # invalid solver type
    with self.assertRaisesWithLiteralMatch(
        mujoco.FatalError, 'mj_fwdConstraint: unknown solver type 10'):
      rollout.rollout(model, data, initial_state, ctrl)

  def test_invalid(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    nroll = 1

    initial_state = np.zeros((nroll, nstate))

    control = 'string'
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'control must be a numpy array or float'):
      rollout.rollout(model, data, initial_state, control)

    control = np.zeros((2, 3, 4, 5))
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'control can have at most 3 dimensions'):
      rollout.rollout(model, data, initial_state, control)

  def test_bad_sizes(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    nroll = 1
    nstep = 3

    initial_state = np.random.randn(nroll, nstate + 1)
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'trailing dimension of initial_state must be 6, got 7'):
      rollout.rollout(model, data, initial_state)

    initial_state = np.random.randn(nroll, nstate)
    control = np.random.randn(1, nstep, model.nu + 1)
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'trailing dimension of control must be 2, got 3'):
      rollout.rollout(model, data, initial_state, control)

    control = np.random.randn(nroll, nstep, model.nu)
    state = np.random.randn(nroll, nstep+1, nstate)  # incompatible nstep
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'dimension 1 inferred as 3 but state has 4'):
      rollout.rollout(model, data, initial_state, control, state=state)

    initial_state = np.random.randn(nroll, nstate)
    control = np.random.randn(nroll, nstep, model.nu)
    bad_spec = mujoco.mjtState.mjSTATE_ACT
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'control_spec can only contain bits in mjSTATE_USER'):
      rollout.rollout(model, data, initial_state, control,
                      control_spec=bad_spec)

  def test_stateless(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    data = mujoco.MjData(model)

    # step with a clean mjData
    initial_state = np.random.randn(nstate)
    control = np.random.randn(3, 3, model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, control)

    # fill user fields with random values
    for attr in [
        'ctrl',
        'qfrc_applied',
        'xfrc_applied',
        'mocap_pos',
        'mocap_quat',
    ]:
      setattr(data, attr, np.random.randn(*getattr(data, attr).shape))

    # roll out again
    state2, sensordata2 = rollout.rollout(model, data, initial_state, control)

    # assert that we still get the same outputs
    np.testing.assert_array_equal(state, state2)
    np.testing.assert_array_equal(sensordata, sensordata2)


# -------------- Python implementation of rollout functionality ----------------


def get_state(model, data):
  nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
  state = np.empty(nstate)
  mujoco.mj_getState(model, data, state, mujoco.mjtState.mjSTATE_FULLPHYSICS)
  return state.reshape((1, nstate))


def step(model, data, state, control,
         control_spec=mujoco.mjtState.mjSTATE_CTRL):
  if state is not None:
    mujoco.mj_setState(model, data, state, mujoco.mjtState.mjSTATE_FULLPHYSICS)
  mujoco.mj_setState(model, data, control, control_spec)
  mujoco.mj_step(model, data)
  return (get_state(model, data), data.sensordata)


def one_rollout(model, data, initial_state, control,
                control_spec=mujoco.mjtState.mjSTATE_CTRL):
  nstep = control.shape[0]
  nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)
  state = np.empty((nstep, nstate))
  sensordata = np.empty((nstep, model.nsensordata))

  mujoco.mj_resetData(model, data)
  for t in range(nstep):
    state[t], sensordata[t] = step(model, data,
                                   initial_state if t == 0 else None,
                                   control[t], control_spec)
  return state, sensordata


def ensure_2d(arg):
  if arg is None:
    return None
  else:
    return np.ascontiguousarray(np.atleast_2d(arg), dtype=np.float64)


def ensure_3d(arg):
  if arg is None:
    return None
  else:
    # np.atleast_3d adds both leading and trailing dims, we want only leading
    if arg.ndim == 0:
      arg = arg[np.newaxis, np.newaxis, np.newaxis, ...]
    elif arg.ndim == 1:
      arg = arg[np.newaxis, np.newaxis, ...]
    elif arg.ndim == 2:
      arg = arg[np.newaxis, ...]
    return np.ascontiguousarray(arg, dtype=np.float64)


def py_rollout(model, data, initial_state, control,
               control_spec=mujoco.mjtState.mjSTATE_CTRL):
  initial_state = ensure_2d(initial_state)
  control = ensure_3d(control)
  nroll = initial_state.shape[0]
  nstep = control.shape[1]
  nstate = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_FULLPHYSICS)

  state = np.empty((nroll, nstep, nstate))
  sensordata = np.empty((nroll, nstep, model.nsensordata))
  for r in range(nroll):
    state_r, sensordata_r = one_rollout(
        model, data, initial_state[r], control[r], control_spec
    )
    state[r] = state_r
    sensordata[r] = sensordata_r
  return state, sensordata


if __name__ == '__main__':
  absltest.main()
