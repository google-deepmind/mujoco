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
"""Named control recordings are applied in the model's actuator order."""

import numpy as np
import pytest

import mujoco
from mujoco import sysid
import mujoco.rollout as rollout

_MODEL_XML = """
<mujoco>
  <option timestep="0.01" gravity="0 0 0"/>
  <worldbody>
    <body name="x_body">
      <joint name="x" type="slide" axis="1 0 0"/>
      <geom size="0.1" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="y_body" pos="0 1 0">
      <joint name="y" type="slide" axis="0 1 0"/>
      <geom size="0.1" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <actuator><motor name="mx" joint="x"/><motor name="my" joint="y"/></actuator>
  <sensor>
    <jointactuatorfrc name="fx" joint="x"/>
    <jointactuatorfrc name="fy" joint="y"/>
  </sensor>
</mujoco>
"""


@pytest.mark.parametrize("named", [False, True])
@pytest.mark.parametrize("precomputed_mappings", [False, True])
def test_rollout_applies_reordered_named_controls(named, precomputed_mappings):
  model = mujoco.MjModel.from_xml_string(_MODEL_XML)
  initial = sysid.create_initial_state(model, np.array([0.2, 0.3]))
  times = np.arange(4) * model.opt.timestep
  canonical = np.tile([1.0, 2.0], (len(times), 1))
  recorded = canonical[:, ::-1].copy() if named else canonical.copy()
  if named:
    control = sysid.TimeSeries.from_control_names(
        times, recorded, model, names=["my", "mx"]
    )
  else:
    control = sysid.TimeSeries(times, recorded)
  expected_state, expected_sensor = rollout.rollout(
      model, mujoco.MjData(model), initial, canonical[:-1]
  )

  kwargs = {}
  if precomputed_mappings:
    qpos, qvel, act, ctrl = sysid.TimeSeries.compute_all_state_mappings(model)
    kwargs = {
        "rollout_signal_mapping": sysid.TimeSeries.compute_all_sensor_mapping(
            model
        ),
        "rollout_state_mapping": qpos | qvel | act,
        "ctrl_mapping": ctrl,
    }
  actual = sysid.sysid_rollout(
      model, mujoco.MjData(model), control, initial, **kwargs
  )[0]

  np.testing.assert_array_equal(actual.sensordata.data, expected_sensor[0])
  np.testing.assert_array_equal(actual.state.data, expected_state[0])
  np.testing.assert_array_equal(actual.control.data, canonical[:-1])
  np.testing.assert_array_equal(actual.control.get_indices("mx_ctrl")[1], [0])
  np.testing.assert_array_equal(actual.control.get_indices("my_ctrl")[1], [1])
  np.testing.assert_array_equal(control.data, recorded)


def test_each_batch_uses_its_own_control_mapping():
  model = mujoco.MjModel.from_xml_string(_MODEL_XML)
  initial = sysid.create_initial_state(model, np.array([0.2, 0.3]))
  times = np.arange(4) * model.opt.timestep
  controls = [
      sysid.TimeSeries.from_control_names(
          times, np.tile([1.0, 2.0], (len(times), 1)), model
      ),
      sysid.TimeSeries.from_control_names(
          times, np.tile([4.0, 3.0], (len(times), 1)), model, names=["my", "mx"]
      ),
  ]

  actual = sysid.sysid_rollout(
      model, mujoco.MjData(model), controls, [initial, initial]
  )

  for trajectory, expected in zip(
      actual, [[1.0, 2.0], [3.0, 4.0]], strict=True
  ):
    np.testing.assert_array_equal(
        trajectory.sensordata.data, np.tile(expected, (len(times) - 1, 1))
    )
    np.testing.assert_array_equal(
        trajectory.control.data, np.tile(expected, (len(times) - 1, 1))
    )


def test_rollout_without_actuators():
  model = mujoco.MjModel.from_xml_string("""
  <mujoco>
    <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
    <sensor><jointpos name="pos" joint="j"/></sensor>
  </mujoco>
  """)
  initial = sysid.create_initial_state(model, np.array([0.2]))
  times = np.arange(4) * model.opt.timestep
  control = sysid.TimeSeries.from_control_names(
      times, np.empty((len(times), 0)), model
  )

  actual = sysid.sysid_rollout(model, mujoco.MjData(model), control, initial)[0]

  assert actual.control.data.shape == (len(times) - 1, 0)
  np.testing.assert_allclose(actual.sensordata.data, 0.2)


def test_canonical_controls_for_unnamed_actuators_keep_their_order():
  xml = _MODEL_XML.replace('name="mx"', "").replace('name="my"', "")
  model = mujoco.MjModel.from_xml_string(xml)
  initial = sysid.create_initial_state(model, np.array([0.2, 0.3]))
  times = np.arange(4) * model.opt.timestep
  control = sysid.TimeSeries.from_control_names(
      times, np.tile([1.0, 2.0], (len(times), 1)), model
  )

  actual = sysid.sysid_rollout(model, mujoco.MjData(model), control, initial)[0]

  np.testing.assert_array_equal(actual.sensordata.data, control.data[:-1])


@pytest.mark.parametrize(
    "invalid,message", [("missing", "mx_ctrl"), ("duplicate", "exactly once")]
)
def test_invalid_control_mapping_is_not_silently_ignored(invalid, message):
  model = mujoco.MjModel.from_xml_string(_MODEL_XML)
  initial = sysid.create_initial_state(model, np.array([0.2, 0.3]))
  times = np.arange(4) * model.opt.timestep
  mapping = sysid.TimeSeries.compute_all_control_mapping(model)
  if invalid == "missing":
    del mapping["mx_ctrl"]
  else:
    mapping["my_ctrl"] = (sysid.SignalType.MjCtrl, np.array([0]))
  control = sysid.TimeSeries(times, np.ones((len(times), model.nu)), mapping)

  with pytest.raises(ValueError, match=message):
    sysid.sysid_rollout(model, mujoco.MjData(model), control, initial)


@pytest.mark.parametrize("optimizer", ["mujoco", "scipy"])
def test_native_gain_calibration_with_reordered_controls(optimizer):
  spec = mujoco.MjSpec.from_string(_MODEL_XML)
  model = spec.compile()
  initial = sysid.create_initial_state(model, np.array([0.2, 0.3]))
  times = np.arange(4) * model.opt.timestep
  canonical = np.tile([1.0, 2.0], (len(times), 1))
  state, sensor = rollout.rollout(
      model, mujoco.MjData(model), initial, canonical[:-1]
  )
  recorded = sysid.TimeSeries.from_control_names(
      times, canonical[:, ::-1].copy(), model, names=["my", "mx"]
  )
  sequence = sysid.ModelSequences(
      "motors",
      spec,
      "measurement",
      initial,
      recorded,
      sysid.TimeSeries.from_names(state[0, :, 0], sensor[0], model),
  )

  def set_gains(spec, param):
    spec.actuator("mx").gainprm[0] = param.value[0]
    spec.actuator("my").gainprm[0] = param.value[1]
    return spec

  params = sysid.ParameterDict({
      "gains": sysid.Parameter(
          "gains",
          np.array([1.2, 0.8]),
          np.full(2, 0.25),
          np.full(2, 3.0),
          modifier=set_gains,
      )
  })
  residual_fn = sysid.build_residual_fn(
      models_sequences=[sequence],
      modify_residual=sysid.SignalTransform(normalize=False).apply,
      n_threads=1,
  )

  optimized, _ = sysid.optimize(
      params, residual_fn, optimizer=optimizer, verbose=False, max_iters=20
  )

  np.testing.assert_allclose(optimized["gains"].value, [1.0, 1.0], atol=1e-6)
