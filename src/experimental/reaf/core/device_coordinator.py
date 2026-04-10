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
"""Coordinates the devices composing a robotic setup."""

import abc
from collections.abc import Iterable
from reaf.core import device


class DeviceCoordinator(abc.ABC):
  """Coordinates the devices composing a robotic setup.

  The `DeviceCoordinator` object is responsible for coordinating all the
  devices constituting the robotic setup. Whilst the Device is hermetic,
  the coordinator is responsible for passing information from one device to
  the other if required. For example in a bimanual setup the coordinator is
  charged with passing the position of each robot to the other so we can ensure
  proper and safe interaction such as for example collision avoidance.

  The `DeviceCoordinator` can be configurable to enable different
  properties on the robotic setup, e.g. adding or not adding a `Device` or
  forwarding configuration to each `Device`.

  At the very least, the coordinator must implement `get_devices`
  to return all the devices. We also provide `on_begin_stepping` and
  `on_end_stepping` methods that will be called before the start of an episode
  and after the end of the episode respectively. Note that resource acquisition
  and subsequent release is completely up to the implementation.

  Finally, `before_set_commands`/`before_get_measurements` can be implemented to
  coordinate devices behaviour before their corresponding functions are
  called.
  """

  @property
  @abc.abstractmethod
  def name(self) -> str:
    """Returns the name of the coordinator."""

  @abc.abstractmethod
  def get_devices(self) -> Iterable[device.Device]:
    """Returns the devices composing the embodiment."""

  # Lifecycle methods.

  def on_begin_stepping(self) -> None:
    """Prepares the coordinator for having its devices called repeatedly.

    After `on_begin_stepping` the devices returned by `get_devices` will have
    their `set_commands` and `get_measurements` called repeatedly until
    `on_end_stepping` is called on this coordinator.
    """

  def on_end_stepping(self) -> None:
    """Notifies the coordinator that the devices are no longer called.

    After `on_end_stepping` the devices returned by `get_devices` will not have
    their `set_commands` and `get_measurements` called anymore until this
    coordinator `on_begin_stepping` method is notified again.
    """

  # Step hooks methods.

  def before_set_commands(self) -> None:
    """Prepares the coordinator to have its devices set_commands called."""

  def after_set_commands(self) -> None:
    """Notifies the coordinator that its devices got `set_commands` called."""

  def before_get_measurements(self) -> None:
    """Prepares the coordinator to have its devices get_measurements called.

    This method gets called immediately before the devices `get_measurements`
    method is called and can be used to customise the devices state given the
    whole setup state.
    """
