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
"""Deprecated compatibility shim; use ``mujoco.studio.app.inputs`` instead."""

from mujoco.studio.app import inputs

handle_camera_select_keyboard_events = (
    inputs.handle_camera_select_keyboard_events
)
handle_camera_tracking_mouse_events = inputs.handle_camera_tracking_mouse_events
handle_freecam_wasd_keyboard_events = inputs.handle_freecam_wasd_keyboard_events
handle_keyboard_events = inputs.handle_keyboard_events
handle_miscellaneous_keyboard_events = (
    inputs.handle_miscellaneous_keyboard_events
)
handle_mouse_events = inputs.handle_mouse_events
handle_reset_keyboard_events = inputs.handle_reset_keyboard_events
handle_sim_history_keyboard_events = inputs.handle_sim_history_keyboard_events
handle_step_control_keyboard_events = inputs.handle_step_control_keyboard_events
handle_vis_options_keyboard_events = inputs.handle_vis_options_keyboard_events

__all__ = [
    'handle_camera_select_keyboard_events',
    'handle_camera_tracking_mouse_events',
    'handle_freecam_wasd_keyboard_events',
    'handle_keyboard_events',
    'handle_miscellaneous_keyboard_events',
    'handle_mouse_events',
    'handle_reset_keyboard_events',
    'handle_sim_history_keyboard_events',
    'handle_step_control_keyboard_events',
    'handle_vis_options_keyboard_events',
]
