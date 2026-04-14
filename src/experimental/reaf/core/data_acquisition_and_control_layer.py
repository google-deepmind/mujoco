# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""REAF data acquisition and control layer to interface with the robotic setup."""

from collections.abc import Iterable, Mapping

from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types
from reaf.core import device as reaf_device
from reaf.core import device_coordinator as reaf_coordinator
from reaf.core import trigger


class DataAcquisitionAndControlLayer:
  """REAF data acquisition and control layer.

  The DACL is responsible to provide an interface for the robotic setup.
  """

  def __init__(
      self,
      *,
      device_coordinator: reaf_coordinator.DeviceCoordinator,
      commands_trigger: trigger.Trigger | None,
      measurements_trigger: trigger.Trigger | None,
  ):
    """Initializes the DataAcquisitionAndControlLayer.

    Args:
      device_coordinator: The coordinator representing a specific robotic setup.
        Note that callers need to explicitly initialize and finalise the
        coordinator.
      commands_trigger: A trigger to unblock processing commands during a call
        to `step`.
      measurements_trigger: A trigger to unblock processing measurements during
        a call to `step`.
    """
    self._coordinator = device_coordinator
    self._devices = self._coordinator.get_devices()
    # The following checks that names of the devices are unique and their keys
    # are "mergeable".
    self._check_device_names_and_keys(self._devices)

    self._commands_trigger = commands_trigger
    self._measurements_trigger = measurements_trigger

    # Create a map of supported commands keys for each Device.
    self._commands_for_device = {
        device.name: device.commands_spec().keys() for device in self._devices
    }

  def begin_stepping(self) -> Mapping[str, gdmr_types.ArrayType]:
    """Begins stepping the DACL and returns the current measurements."""
    self._coordinator.on_begin_stepping()

    # Wait for the first trigger to happen before collecting the measurements.
    if self._measurements_trigger is not None:
      self._measurements_trigger.wait_for_event()
    return self._get_measurements()

  def end_stepping(self) -> None:
    """Ends stepping the data acquisition and control layer."""
    self._coordinator.on_end_stepping()

  def _set_commands(self, commands: Mapping[str, gdmr_types.ArrayType]) -> None:
    """Sets the commands of the data acquisition and control layer."""
    self._coordinator.before_set_commands()
    for device in self._devices:
      device_commands = {
          k: v
          for k, v in commands.items()
          if k in self._commands_for_device[device.name]
      }
      device.set_commands(device_commands)
    self._coordinator.after_set_commands()

  def _get_measurements(self) -> Mapping[str, gdmr_types.ArrayType]:
    """Gets the measurements of the data acquisition and control layer."""
    measurements = {}
    self._coordinator.before_get_measurements()
    for device in self._devices:
      measurements.update(device.get_measurements())

    return measurements

  def step(
      self, commands: Mapping[str, gdmr_types.ArrayType]
  ) -> Mapping[str, gdmr_types.ArrayType]:
    """Steps the data acquisition and control layer."""
    if self._commands_trigger is not None:
      self._commands_trigger.wait_for_event()
    self._set_commands(commands)

    if self._measurements_trigger is not None:
      self._measurements_trigger.wait_for_event()
    return self._get_measurements()

  def commands_spec(self) -> Mapping[str, gdmr_types.AnyArraySpec]:
    """Returns the specs for the commands."""
    spec = {}
    for device in self._devices:
      spec.update(device.commands_spec())
    return spec

  def measurements_spec(self) -> Mapping[str, specs.Array]:
    """Returns the specs for the measurements."""
    spec = {}
    for device in self._devices:
      spec.update(device.measurements_spec())
    return spec

  @property
  def device_coordinator(self) -> reaf_coordinator.DeviceCoordinator:
    return self._coordinator

  def _check_keys_have_been_formatted_correctly(
      self, current_key_set: Iterable[str]
  ) -> None:
    """Check that keys haven't been left unformatted."""
    for key in current_key_set:
      if key.find("{}") != -1:
        raise ValueError(
            "Keys should not contain '{}'. Did you mean to use format()?"
        )

  def _check_device_names_and_keys(
      self, devices: Iterable[reaf_device.Device]
  ) -> None:
    """Raises error if device names are not unique or keys are not exclusive."""
    # Check names first.
    all_names = [device.name for device in devices]
    unique_names = set(all_names)
    if len(unique_names) != len(all_names):
      raise RuntimeError(f"Duplicate names when checking devices: {all_names}")

    # Check commands.
    devices = tuple(devices)
    current_specs = set()
    for device in devices:
      device_keys = device.commands_spec().keys()
      self._check_keys_have_been_formatted_correctly(device_keys)
      if not current_specs.isdisjoint(device_keys):
        raise RuntimeError(
            f"Duplicate keys when checking device {device.name}:"
            f" {current_specs.intersection(device_keys)}"
        )
      current_specs.update(device_keys)

    # Check measurements.
    current_specs = set()
    for device in devices:
      device_keys = device.measurements_spec().keys()
      self._check_keys_have_been_formatted_correctly(device_keys)
      if not current_specs.isdisjoint(device_keys):
        raise RuntimeError(
            f"Duplicate keys when checking device {device.name}:"
            f" {current_specs.intersection(device_keys)}"
        )
      current_specs.update(device_keys)
