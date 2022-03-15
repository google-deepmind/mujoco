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

from absl.testing import absltest
from absl.testing import parameterized
import mujoco
import numpy as np
import concurrent.futures
import threading
from mujoco import rollout

#--------------------------- models used for testing ---------------------------

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
    <framequat objtype="xbody" objname="2"/>
  </sensor>
</mujoco>
"""

TEST_XML_EMPTY = r"""
<mujoco>
</mujoco>
"""

ALL_MODELS = {'TEST_XML': TEST_XML,
              'TEST_XML_NO_SENSORS': TEST_XML_NO_SENSORS,
              'TEST_XML_NO_ACTUATORS': TEST_XML_NO_ACTUATORS,
              'TEST_XML_EMPTY': TEST_XML_EMPTY}

#------------------------------- tests -----------------------------------------

class MuJoCoRolloutTest(parameterized.TestCase):

  def setUp(self):
    super().setUp()
    np.random.seed(42)

  #----------------------------- test basic operation

  @parameterized.parameters(ALL_MODELS.keys())
  def test_single_step(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    data = mujoco.MjData(model)

    initial_state = np.random.randn(model.nq + model.nv + model.na)
    ctrl = np.random.randn(model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, ctrl)

    mujoco.mj_resetData(model, data)
    py_state, py_sensordata = step(model, data, initial_state, ctrl=ctrl)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)



  @parameterized.parameters(ALL_MODELS.keys())
  def test_single_rollout(self, model_name):
    nstep = 3  # number of timesteps

    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    data = mujoco.MjData(model)

    initial_state = np.random.randn(model.nq + model.nv + model.na)
    ctrl = np.random.randn(nstep, model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, ctrl)

    py_state, py_sensordata = single_rollout(model, data, initial_state,
                                             ctrl=ctrl)
    np.testing.assert_array_equal(state, np.asarray(py_state))
    np.testing.assert_array_equal(sensordata, np.asarray(py_sensordata))

  @parameterized.parameters(ALL_MODELS.keys())
  def test_multi_step(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    data = mujoco.MjData(model)

    nstate = 5  # number of initial states

    initial_state = np.random.randn(nstate, model.nq + model.nv + model.na)
    ctrl = np.random.randn(nstate, 1, model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, ctrl)

    mujoco.mj_resetData(model, data)
    py_state, py_sensordata = multi_rollout(model, data, initial_state,
                                            ctrl=ctrl)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  @parameterized.parameters(ALL_MODELS.keys())
  def test_single_rollout_fixed_ctrl(self, model_name):
    nstep = 3
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    data = mujoco.MjData(model)

    initial_state = np.random.randn(model.nq + model.nv + model.na)
    ctrl = np.random.randn(model.nu)
    state = np.empty((nstep, model.nq + model.nv + model.na))
    sensordata = np.empty((nstep, model.nsensordata))
    rollout.rollout(model, data, initial_state, ctrl,
                    state=state, sensordata=sensordata)

    ctrl = np.tile(ctrl, (nstep, 1))  # repeat??
    py_state, py_sensordata = single_rollout(model, data, initial_state,
                                             ctrl=ctrl)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  @parameterized.parameters(ALL_MODELS.keys())
  def test_multi_rollout(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    data = mujoco.MjData(model)

    nstate = 2  # number of initial states
    nstep = 3  # number of timesteps

    initial_state = np.random.randn(nstate, model.nq + model.nv + model.na)
    ctrl = np.random.randn(nstate, nstep, model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, ctrl)

    py_state, py_sensordata = multi_rollout(model, data, initial_state,
                                            ctrl=ctrl)
    np.testing.assert_array_equal(py_state, py_state)
    np.testing.assert_array_equal(py_sensordata, py_sensordata)

  @parameterized.parameters(ALL_MODELS.keys())
  def test_multi_rollout_fixed_ctrl_infer_from_output(self, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    data = mujoco.MjData(model)

    nstate = 2  # number of initial states
    nstep = 3  # number of timesteps

    initial_state = np.random.randn(nstate, model.nq + model.nv + model.na)
    ctrl = np.random.randn(nstate, 1, model.nu)  # 1 control in the time dimension
    state = np.empty((nstate, nstep, model.nq + model.nv + model.na))
    state, sensordata = rollout.rollout(model, data, initial_state, ctrl,
                                        state=state)

    ctrl = np.repeat(ctrl, nstep, axis=1)
    py_state, py_sensordata = multi_rollout(model, data, initial_state,
                                            ctrl=ctrl)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  @parameterized.product(arg_nstep=[[3, 1, 1], [3, 3, 1], [3, 1, 3]],
                         model_name=list(ALL_MODELS.keys()))
  def test_multi_rollout_multiple_inputs(self, arg_nstep, model_name):
    model = mujoco.MjModel.from_xml_string(ALL_MODELS[model_name])
    data = mujoco.MjData(model)

    nstate = 4  # number of initial states

    initial_state = np.random.randn(nstate, model.nq + model.nv + model.na)

    # arg_nstep is the horizon for {ctrl, qfrc_applied, xfrc_applied}, respectively
    ctrl = np.random.randn(nstate, arg_nstep[0], model.nu)
    qfrc_applied = np.random.randn(nstate, arg_nstep[1], model.nv)
    xfrc_applied = np.random.randn(nstate, arg_nstep[2], model.nbody*6)

    state, sensordata = rollout.rollout(model, data, initial_state, ctrl,
                                        qfrc_applied=qfrc_applied,
                                        xfrc_applied=xfrc_applied)

    # tile singleton arguments
    nstep = max(arg_nstep)
    if arg_nstep[0] == 1:
      ctrl = np.repeat(ctrl, nstep, axis=1)
    if arg_nstep[1] == 1:
      qfrc_applied = np.repeat(qfrc_applied, nstep, axis=1)
    if arg_nstep[2] == 1:
      xfrc_applied = np.repeat(xfrc_applied, nstep, axis=1)

    py_state, py_sensordata = multi_rollout(model, data, initial_state,
                                            ctrl=ctrl,
                                            qfrc_applied=qfrc_applied,
                                            xfrc_applied=xfrc_applied)
    np.testing.assert_array_equal(state, py_state)
    np.testing.assert_array_equal(sensordata, py_sensordata)

  #----------------------------- test threaded operation

  def test_threading(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    num_workers = 32
    nstate = 10000
    nstep = 5
    initial_state = np.random.randn(nstate, model.nq+model.nv+model.na)
    state = np.zeros((nstate, nstep, model.nq+model.nv+model.na))
    sensordata = np.zeros((nstate, nstep, model.nsensordata))
    ctrl = np.random.randn(nstate, nstep, model.nu)

    thread_local = threading.local()

    def thread_initializer():
      thread_local.data = mujoco.MjData(model)

    def call_rollout(initial_state, ctrl, state):
      rollout.rollout(model, thread_local.data, skip_checks=True,
                      nstate=initial_state.shape[0], nstep=nstep,
                      initial_state=initial_state, ctrl=ctrl, state=state)

    n = initial_state.shape[0] // num_workers  # integer division
    chunks = []  # a list of tuples, one per worker
    for i in range(num_workers-1):
      chunks.append(
          (initial_state[i*n:(i+1)*n], ctrl[i*n:(i+1)*n], state[i*n:(i+1)*n]))
    # last chunk, absorbing the remainder:
    chunks.append(
        (initial_state[(num_workers-1)*n:], ctrl[(num_workers-1)*n:],
         state[(num_workers-1)*n:]))

    with concurrent.futures.ThreadPoolExecutor(
        max_workers=num_workers, initializer=thread_initializer) as executor:
      futures = []
      for chunk in chunks:
        futures.append(executor.submit(call_rollout, *chunk))
      for future in concurrent.futures.as_completed(futures):
        future.result()

    data = mujoco.MjData(model)
    py_state, py_sensordata = multi_rollout(model, data, initial_state,
                                            ctrl=ctrl)
    np.testing.assert_array_equal(state, py_state)

  #----------------------------- test advanced operation

  def test_time(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    data = mujoco.MjData(model)

    nstate = 1
    nstep = 3

    initial_time = np.array([[2.]])
    initial_state = np.random.randn(nstate, model.nq + model.nv + model.na)
    ctrl = np.random.randn(nstate, nstep, model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, ctrl,
                                        initial_time=initial_time)

    self.assertAlmostEqual(data.time, 2 + nstep*model.opt.timestep)

  def test_warmstart(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    data = mujoco.MjData(model)

    state0 = np.zeros(model.nq + model.nv + model.na)
    ctrl = np.zeros(model.nu)
    state1, _ = step(model, data, state0, ctrl=ctrl)
    initial_warmstart = data.qacc_warmstart.copy()

    state2, _ = step(model, data, state1, ctrl=ctrl)

    state, _ = rollout.rollout(model, data, state1, ctrl)
    assert np.linalg.norm(state-state2) > 0

    state, _ = rollout.rollout(model, data, state1, ctrl,
                               initial_warmstart=initial_warmstart)
    np.testing.assert_array_equal(state, state2)

  def test_mocap(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML_MOCAP)
    data = mujoco.MjData(model)

    initial_state = np.zeros(model.nq + model.nv + model.na)
    pos1 = np.array((1., 2., 3.))
    quat1 = np.array((1., 2., 3., 4.))
    quat1 /= np.linalg.norm(quat1)
    pos2 = np.array((2., 3., 4.))
    quat2 = np.array((2., 3., 4., 5.))
    quat2 /= np.linalg.norm(quat2)
    mocap = np.hstack((pos1, quat1, pos2, quat2))

    state, sensordata = rollout.rollout(model, data, initial_state, mocap=mocap)

    np.testing.assert_array_almost_equal(sensordata[:3], pos1)
    np.testing.assert_array_almost_equal(sensordata[3:], quat2)

  #----------------------------- test correctness

  def test_intercept_mj_errors(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    data = mujoco.MjData(model)

    initial_state = np.zeros(model.nq + model.nv + model.na)
    ctrl = np.zeros((3, model.nu))

    model.opt.solver = 10  # invalid solver type
    with self.assertRaisesWithLiteralMatch(mujoco.FatalError,
                                           'Unknown solver type 10'):
      state, sensordata = rollout.rollout(model, data, initial_state, ctrl)

  def test_invalid(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    data = mujoco.MjData(model)

    initial_state = np.zeros(model.nq + model.nv + model.na)

    ctrl = 'string'
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'ctrl must be a numpy array or float'):
      state, sensordata = rollout.rollout(model, data, initial_state, ctrl)

    qfrc_applied = np.zeros((2, 3, 4, 5))
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'qfrc_applied can have at most 3 dimensions'):
      state, sensordata = rollout.rollout(model, data, initial_state,
                                          qfrc_applied=qfrc_applied)

  def test_bad_sizes(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    data = mujoco.MjData(model)

    initial_state = np.random.randn(model.nq + model.nv + model.na+1)
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'trailing dimension of initial_state must be 5, got 6'):
      state, sensordata = rollout.rollout(model, data, initial_state)

    initial_state = np.random.randn(model.nq + model.nv + model.na)
    ctrl = np.random.randn(model.nu+1)
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'trailing dimension of ctrl must be 2, got 3'):
      state, sensordata = rollout.rollout(model, data, initial_state, ctrl)

    ctrl = np.random.randn(2, model.nu)
    qfrc_applied = np.random.randn(3, model.nv)  # incompatible horizon
    with self.assertRaisesWithLiteralMatch(
        ValueError, 'dimension 1 inferred as 2 but qfrc_applied has 3'):
      state, sensordata = rollout.rollout(model, data, initial_state, ctrl,
                                          qfrc_applied=qfrc_applied)

  def test_stateless(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_WARMSTART.value
    data = mujoco.MjData(model)

    # call step with a clean mjData
    initial_state = np.random.randn(model.nq + model.nv + model.na)
    ctrl = np.random.randn(model.nu)
    state, sensordata = rollout.rollout(model, data, initial_state, ctrl)

    # fill mjData with some debug value, see that we still get the same outputs
    mujoco.mj_resetDataDebug(model, data, 255)
    debug_state, debug_sensordata = rollout.rollout(model, data, initial_state,
                                                    ctrl)

    np.testing.assert_array_equal(state, debug_state)
    np.testing.assert_array_equal(sensordata, debug_sensordata)


#--------------- Python implementation of rollout functionality ----------------

def get_state(data):
  return np.hstack((data.qpos, data.qvel, data.act))

def set_state(model, data, state):
  data.qpos = state[:model.nq]
  data.qvel = state[model.nq:model.nq+model.nv]
  data.act = state[model.nq+model.nv:model.nq+model.nv+model.na]

def step(model, data, state, **kwargs):
  if state is not None:
    set_state(model, data, state)
  for key, value in kwargs.items():
    if value is not None:
      setattr(data, key, np.reshape(value, getattr(data, key).shape))
  mujoco.mj_step(model, data)
  return (get_state(data), data.sensordata)

def single_rollout(model, data, initial_state, **kwargs):
  arg_nstep = set([a.shape[0] for a in kwargs.values()])
  assert len(arg_nstep) == 1  # nstep dimensions must match
  nstep = arg_nstep.pop()

  state = np.empty((nstep, model.nq + model.nv + model.na))
  sensordata = np.empty((nstep, model.nsensordata))

  mujoco.mj_resetData(model, data)
  for t in range(nstep):
    kwargs_t = {}
    for key, value in kwargs.items():
      kwargs_t[key] = value[0 if value.ndim == 1 else t]
    state[t], sensordata[t] = step(model, data,
                                   initial_state if t==0 else None,
                                   **kwargs_t)
  return state, sensordata

def multi_rollout(model, data, initial_state, **kwargs):
  nstate = initial_state.shape[0]
  arg_nstep = set([a.shape[1] for a in kwargs.values()])
  assert len(arg_nstep) == 1  # nstep dimensions must match
  nstep = arg_nstep.pop()

  state = np.empty((nstate, nstep, model.nq + model.nv + model.na))
  sensordata = np.empty((nstate, nstep, model.nsensordata))
  for s in range(nstate):
    kwargs_s = {key : value[s] for key, value in kwargs.items()}
    state_s, sensordata_s = single_rollout(model, data, initial_state[s],
                                           **kwargs_s)
    state[s] = state_s
    sensordata[s] = sensordata_s
  return state.squeeze(), sensordata.squeeze()

if __name__ == '__main__':
  absltest.main()
