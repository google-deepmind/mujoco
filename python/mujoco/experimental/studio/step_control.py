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

from mujoco.experimental.studio import messages
from mujoco.experimental.studio import sim


class StepControl:
  """Default sim plugin that steps CPU physics with real-time pacing; pass your
  own plugin instead to step differently (e.g. a GPU simulation).

  Pacing: this plugin owns real-time pacing. It steps as much sim time as fits
  the wall-clock budget of one loop iteration. A sim loop without a stepping
  plugin must pace itself: a bare while handle.is_running() and handle.sync(...)
  loop busy-spins, since sync itself never sleeps.
  """

  def __init__(self) -> None:
    self.step_control = sim.StepControl()

  @messages.handler(priority=messages.Priority.LIBRARY)
  def _on_model(self, event: messages.ModelEvent) -> bool:
    del event
    # Fresh step control so the new model starts time-synchronized. Runs at
    # LIBRARY priority, before ViewerHandle swaps model/data and consumes the
    # event at INTERNAL priority.
    self.step_control = sim.StepControl()
    return False

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_step_control(self, event: messages.StepControlSnapshot) -> bool:
    sc = self.step_control
    sc.set_pause_state(event.pause_state)
    sc.set_speed(event.speed)
    sc.set_noise_parameters(event.noise_scale, event.noise_rate)
    return True

  @messages.handler(priority=messages.Priority.INTERNAL)
  def _on_step(self, event: messages.StepEvent) -> bool:
    self.step_control.advance(event.model, event.data)
    # Do not consume: lower-priority plugins may observe the post-step state.
    return False
