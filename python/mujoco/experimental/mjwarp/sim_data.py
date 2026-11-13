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

"""Bridge for seamless PyTorch-Warp interoperability with zero-copy memory sharing.

Provides automatic wrapping of Warp arrays as PyTorch-compatible objects while
preserving shared memory and CUDA graph compatibility.
"""

from typing import Any, Dict, Generic, Optional, Tuple, TypeVar

import torch
import warp as wp

T = TypeVar("T")


class TorchArray:
  """Warp array that behaves like a torch.Tensor with shared memory."""

  def __init__(self, wp_array: wp.array, nworld: int | None = None) -> None:
    """Initialize the tensor proxy with a Warp array."""
    if (
      nworld is not None
      and len(wp_array.shape) > 0
      and wp_array.strides[0] == 0
      and wp_array.shape[0] > nworld
    ):
      wp_array = wp_array[:nworld]  # type: ignore

    self._wp_array = wp_array
    self._tensor = wp.to_torch(wp_array)
    self._is_cuda = not self._wp_array.device.is_cpu  # type: ignore
    self._torch_stream = self._setup_stream()

  def _setup_stream(self) -> Optional[torch.cuda.Stream]:
    """Setup appropriate stream for the device."""
    if not self._is_cuda:
      return None

    try:
      warp_stream = wp.get_stream(self._wp_array.device)
      return torch.cuda.ExternalStream(warp_stream.cuda_stream)
    except Exception as e:
      # Fallback to default stream if external stream creation fails.
      print(f"Warning: Could not create external stream: {e}")
      return torch.cuda.current_stream(self._tensor.device)

  @property
  def wp_array(self) -> wp.array:
    return self._wp_array

  def __repr__(self) -> str:
    """Return string representation of the underlying tensor."""
    return repr(self._tensor)

  def __getitem__(self, idx: Any) -> Any:
    """Get item(s) from the tensor using standard indexing."""
    return self._tensor[idx]

  def __setitem__(self, idx: Any, value: Any) -> None:
    """Set item(s) in the tensor using standard indexing."""
    if self._is_cuda and self._torch_stream is not None:
      with torch.cuda.stream(self._torch_stream):
        self._tensor[idx] = value
    else:
      self._tensor[idx] = value

  def __getattr__(self, name: str) -> Any:
    """Delegate attribute access to the underlying tensor."""
    return getattr(self._tensor, name)

  @classmethod
  def __torch_function__(
    cls,
    func: Any,
    types: Tuple[type, ...],
    args: Tuple[Any, ...] = (),
    kwargs: Optional[Dict[str, Any]] = None,
  ) -> Any:
    """Intercept torch.* function calls to unwrap TorchArray objects."""
    if kwargs is None:
      kwargs = {}

    # Only intercept when at least one argument is our proxy.
    if not any(issubclass(t, cls) for t in types):
      return NotImplemented

    def _unwrap(x: Any) -> Any:
      """Unwrap TorchArray objects to their underlying tensors."""
      return x._tensor if isinstance(x, cls) else x

    # Unwrap all TorchArray objects in args and kwargs.
    unwrapped_args = tuple(_unwrap(arg) for arg in args)
    unwrapped_kwargs = {k: _unwrap(v) for k, v in kwargs.items()}

    return func(*unwrapped_args, **unwrapped_kwargs)

  # Arithmetic operators.

  def __add__(self, other: Any) -> Any:
    return self._tensor + other

  def __radd__(self, other: Any) -> Any:
    return other + self._tensor

  def __sub__(self, other: Any) -> Any:
    return self._tensor - other

  def __rsub__(self, other: Any) -> Any:
    return other - self._tensor

  def __mul__(self, other: Any) -> Any:
    return self._tensor * other

  def __rmul__(self, other: Any) -> Any:
    return other * self._tensor

  def __truediv__(self, other: Any) -> Any:
    return self._tensor / other

  def __rtruediv__(self, other: Any) -> Any:
    return other / self._tensor

  def __pow__(self, other: Any) -> Any:
    return self._tensor**other

  def __rpow__(self, other: Any) -> Any:
    return other**self._tensor

  def __neg__(self) -> Any:
    return -self._tensor

  def __pos__(self) -> Any:
    return +self._tensor

  def __abs__(self) -> Any:
    return abs(self._tensor)

  # Comparison operators.

  def __eq__(self, other: Any) -> Any:
    return self._tensor == other

  def __ne__(self, other: Any) -> Any:
    return self._tensor != other

  def __lt__(self, other: Any) -> Any:
    return self._tensor < other

  def __le__(self, other: Any) -> Any:
    return self._tensor <= other

  def __gt__(self, other: Any) -> Any:
    return self._tensor > other

  def __ge__(self, other: Any) -> Any:
    return self._tensor >= other


def _contains_warp_arrays(obj: Any) -> bool:
  """Check if an object or its attributes contain any Warp arrays."""
  if isinstance(obj, wp.array):
    return True

  # Check if it's a struct-like object with attributes
  if hasattr(obj, "__dict__"):
    return any(
      isinstance(getattr(obj, attr), wp.array)
      for attr in dir(obj)
      if not attr.startswith("_")
    )

  return False


class WarpBridge(Generic[T]):
  """Wraps mjwarp objects to expose Warp arrays as PyTorch tensors.

  Automatically converts Warp array attributes to TorchArray objects
  on access, enabling direct PyTorch operations on simulation data.
  Recursively wraps nested structures that contain Warp arrays.

  IMPORTANT: This wrapper is read-only. To modify array data, use
  in-place operations like `obj.field[:] = value`. Direct assignment
  like `obj.field = new_array` will raise an AttributeError to prevent
  accidental memory address changes that break CUDA graphs.
  """

  def __init__(self, struct: T, nworld: int | None = None) -> None:
    object.__setattr__(self, "_struct", struct)
    object.__setattr__(self, "_wrapped_cache", {})
    object.__setattr__(self, "_nworld", nworld)

  def __getattr__(self, name: str) -> Any:
    """Get attribute from the wrapped data, wrapping Warp arrays as TorchArray."""
    # Check cache first to avoid recreating wrappers.
    if name in self._wrapped_cache:
      return self._wrapped_cache[name]

    val = getattr(self._struct, name)

    # Wrap Warp arrays.
    if isinstance(val, wp.array):
      wrapped = TorchArray(val, nworld=self._nworld)
      self._wrapped_cache[name] = wrapped
      return wrapped

    # Recursively wrap nested structures that contain Warp arrays.
    if _contains_warp_arrays(val):
      wrapped = WarpBridge(val, nworld=self._nworld)
      self._wrapped_cache[name] = wrapped
      return wrapped

    return val

  def __setattr__(self, name: str, value: Any) -> None:
    """Prevent attribute setting to maintain CUDA graph safety."""
    raise AttributeError(
      f"Cannot set attribute '{name}' on WarpBridge. "
      f"This wrapper is read-only to preserve memory addresses for CUDA graphs. "
      f"Use in-place operations instead: obj.{name}[:] = value"
    )

  def __repr__(self) -> str:
    """Return string representation of the wrapped struct."""
    return f"WarpBridge({repr(self._struct)})"

  @property
  def struct(self) -> T:
    """Access the underlying wrapped struct."""
    return self._struct
