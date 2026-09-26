# Copyright 2026 keeeeenw
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

"""Metal Kernel Manager for PyTorch MPS.

Compiles Metal Shading Language (MSL) source strings via torch.mps.compile_shader,
validates buffer contiguity, checks device placement, and provides clean launch helpers.
"""

from pathlib import Path
from typing import Any, Tuple, Union

import torch


def assert_contiguous(tensor: torch.Tensor, name: str = "tensor") -> None:
  """Validate that the tensor is contiguous in physical memory.

  Non-contiguous (strided) layouts must be rejected because Metal kernels
  access device memory as flat 1D/2D arrays.
  """
  if not tensor.is_contiguous():
    raise ValueError(
        f"Tensor '{name}' must be contiguous. Shape: {tensor.shape}, "
        f"strides: {tensor.stride()}. Slices like tensor[:, :14] or transposed tensors "
        "are non-contiguous and must be made contiguous before passing to Metal kernels."
    )


def assert_mps(tensor: torch.Tensor, name: str = "tensor") -> None:
  """Validate that the tensor resides on the MPS device."""
  if tensor.device.type != "mps":
    raise ValueError(
        f"Tensor '{name}' must reside on 'mps', got: {tensor.device}"
    )


class MetalKernelManager:
  """Loads and compiles Metal compute kernels for use with PyTorch MPS tensors."""

  def __init__(self, shader_path: Union[str, Path]):
    self.shader_path = Path(shader_path)
    if not self.shader_path.exists():
      raise FileNotFoundError(f"Shader source not found: {self.shader_path}")

    self.source = self.shader_path.read_text()
    if not torch.backends.mps.is_available():
      raise RuntimeError("PyTorch MPS is not available on this machine.")
    if not hasattr(torch.mps, "compile_shader"):
      raise RuntimeError(
          "Installed PyTorch does not expose torch.mps.compile_shader."
      )

    self.library = torch.mps.compile_shader(self.source)

  def get_kernel(self, name: str):
    if not hasattr(self.library, name):
      raise AttributeError(
          f"Kernel '{name}' not found in compiled Metal library."
      )
    return getattr(self.library, name)

  def launch(
      self,
      kernel_name: str,
      *args: Any,
      threads: Union[int, Tuple[int, ...]] = None,
      group_size: Union[int, Tuple[int, ...]] = None,
      validate_layouts: bool = True,
  ) -> None:
    """Launch a compiled Metal kernel on PyTorch MPS arguments."""
    if validate_layouts:
      for i, arg in enumerate(args):
        if isinstance(arg, torch.Tensor):
          assert_mps(arg, f"arg_{i}")
          assert_contiguous(arg, f"arg_{i}")

    kernel = self.get_kernel(kernel_name)
    kwargs = {}
    if threads is not None:
      kwargs["threads"] = threads
    if group_size is not None:
      kwargs["group_size"] = group_size

    kernel(*args, **kwargs)
