# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Default sim-side stepping plugin."""

import contextlib

from mujoco.experimental.studio import messages
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import viewer_handle


class StepControl:
  """Default sim plugin that steps CPU physics with real-time pacing.

  Pass your own plugin instead to step differently (e.g. a GPU simulation).

  Pacing: this plugin owns real-time pacing. It steps as much sim time as fits
  the wall-clock budget of one loop iteration. Without a plugin handling
  StepEvent that at minimum paces each iteration, a bare while
  handle.is_running() and handle.sync(...) loop busy-spins, since sync itself
  never sleeps.
  """

  def __init__(self) -> None:
    self._handle: viewer_handle.ViewerHandle | None = None
    self._model_path: str = ''
    self.step_control = sim.StepControl()

  def _send_if_changes(self) -> contextlib.AbstractContextManager[None]:
    if self._handle is None:
      return contextlib.nullcontext()
    return self._handle.send_if_changes(
        lambda: messages.StepControlSnapshot.from_step_control(
            self.step_control
        )
    )

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_sim_init(self, event: viewer_handle.SimInitEvent) -> None:
    self._handle = event.handle

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_post_model(self, event: messages.PostModelEvent) -> None:
    same_scene = bool(event.path and event.path == self._model_path)
    self._model_path = event.path
    if same_scene:
      # Same scene reloaded: keep pause, speed, and noise; only resync pacing.
      self.step_control.force_sync()
    else:
      self.step_control = sim.StepControl()

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_step_control(self, event: messages.StepControlSnapshot) -> bool:
    self.step_control.set_pause_state(event.pause_state)
    self.step_control.set_speed(event.speed)
    self.step_control.set_noise_parameters(event.noise_scale, event.noise_rate)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_single_step(self, _: messages.SingleStepEvent) -> bool:
    self.step_control.request_single_step()
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_pause(self, event: messages.RequestPauseEvent) -> bool:
    with self._send_if_changes():
      self.step_control.set_pause_state(event.pause_state)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_step(self, event: messages.StepEvent) -> None:
    with self._send_if_changes():
      self.step_control.advance(event.model, event.data)
