# Copyright 2025 Kevin Zakka
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

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, cast

import mujoco
import mujoco_warp as mjwarp
import numpy as np
import warp as wp

from mujoco.experimental.mjwarp.sim_data import WarpBridge

HEIGHT = 240
WIDTH = 320
LS_PARALLEL = False

# Type aliases for better IDE support while maintaining runtime compatibility
# At runtime, WarpBridge wraps the actual MJWarp objects.
if TYPE_CHECKING:
  ModelBridge = mjwarp.Model
  DataBridge = mjwarp.Data
else:
  ModelBridge = WarpBridge
  DataBridge = WarpBridge


class Simulation:
  """GPU-accelerated MuJoCo simulation powered by MJWarp."""

  def __init__(
    self, num_envs: int, nconmax: int, njmax: int, model: mujoco.MjModel, device: str
  ):
    self.device = device
    self.wp_device = wp.get_device(self.device)
    self.num_envs = num_envs

    self._mj_model = model
    self._mj_data = mujoco.MjData(model)
    mujoco.mj_forward(self._mj_model, self._mj_data)

    with wp.ScopedDevice(self.wp_device):
      self._wp_model = mjwarp.put_model(self._mj_model)
      self._wp_model.opt.ls_parallel = LS_PARALLEL

      self._wp_data = mjwarp.put_data(
        self._mj_model,
        self._mj_data,
        nworld=self.num_envs,
        nconmax=nconmax,
        njmax=njmax,
      )

    self._model_bridge = WarpBridge(self._wp_model, nworld=self.num_envs)
    self._data_bridge = WarpBridge(self._wp_data)

    self.use_cuda_graph = self.wp_device.is_cuda and wp.is_mempool_enabled(
      self.wp_device
    )
    self.create_graph()

    self._camera = -1
    self._renderer: mujoco.Renderer | None = None

  def initialize_renderer(self) -> None:
    if self._renderer is not None:
      raise RuntimeError(
        "Renderer is already initialized. Call 'close()' first to reinitialize."
      )
    self._renderer = mujoco.Renderer(
      model=self._mj_model, height=HEIGHT, width=WIDTH
    )

  def create_graph(self) -> None:
    self.step_graph = None
    self.forward_graph = None
    if self.use_cuda_graph:
      with wp.ScopedCapture() as capture:
        mjwarp.step(self.wp_model, self.wp_data)
      self.step_graph = capture.graph
      with wp.ScopedCapture() as capture:
        mjwarp.forward(self.wp_model, self.wp_data)
      self.forward_graph = capture.graph

  # Properties.

  @property
  def mj_model(self) -> mujoco.MjModel:
    return self._mj_model

  @property
  def mj_data(self) -> mujoco.MjData:
    return self._mj_data

  @property
  def wp_model(self) -> mjwarp.Model:
    return self._wp_model

  @property
  def wp_data(self) -> mjwarp.Data:
    return self._wp_data

  @property
  def data(self) -> "DataBridge":
    return cast("DataBridge", self._data_bridge)

  @property
  def model(self) -> "ModelBridge":
    return cast("ModelBridge", self._model_bridge)

  @property
  def renderer(self) -> mujoco.Renderer:
    if self._renderer is None:
      raise ValueError("Renderer not initialized. Call 'initialize_renderer()' first.")

    return self._renderer

  # Methods.
  def reset(self) -> None:
    # TODO(kevin): Should we be doing anything here?
    pass

  def forward(self) -> None:
    with wp.ScopedDevice(self.wp_device):
      if self.use_cuda_graph and self.forward_graph is not None:
        wp.capture_launch(self.forward_graph)
      else:
        mjwarp.forward(self.wp_model, self.wp_data)

  def step(self) -> None:
    with wp.ScopedDevice(self.wp_device):
      if self.use_cuda_graph and self.step_graph is not None:
        wp.capture_launch(self.step_graph)
      else:
        mjwarp.step(self.wp_model, self.wp_data)

  def update_render(self) -> None:
    if self._renderer is None:
      raise ValueError("Renderer not initialized. Call 'initialize_renderer()' first.")

    mjwarp.get_data_into(self._mj_data, self._mj_model, self._wp_data)
    mujoco.mj_forward(self._mj_model, self._mj_data)
    self._renderer.update_scene(data=self._mj_data, camera=self._camera)

  def render(self) -> np.ndarray:
    if self._renderer is None:
      raise ValueError("Renderer not initialized. Call 'initialize_renderer()' first.")

    return self._renderer.render()

  def close(self) -> None:
    if self._renderer is not None:
      self._renderer.close()
      self._renderer = None
