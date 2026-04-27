# Copyright 2018 The dm_control Authors
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
"""An EGL context for headless accelerated OpenGL rendering on GPU devices."""

import atexit
import ctypes
import os

PYOPENGL_PLATFORM = os.environ.get('PYOPENGL_PLATFORM')

if not PYOPENGL_PLATFORM:
  os.environ['PYOPENGL_PLATFORM'] = 'egl'
elif PYOPENGL_PLATFORM.lower() != 'egl':
  raise ImportError(
      'Cannot use EGL rendering platform. '
      'The PYOPENGL_PLATFORM environment variable is set to {!r} '
      '(should be either unset or \'egl\').')

from mujoco.egl import egl_ext as EGL
from OpenGL import error


def _get_egl_device_cuda_id(device):
  """Returns the CUDA device id for an EGL device, if the driver exposes it."""
  try:
    return EGL.eglQueryDeviceAttribEXT(device, EGL.EGL_CUDA_DEVICE_NV)
  except (AttributeError, ImportError, TypeError, error.GLError):
    return None


def _get_egl_device_cuda_map(devices):
  """Returns a CUDA id to EGL device map, or None if it is unavailable."""
  cuda_map = {}
  for device in devices:
    cuda_id = _get_egl_device_cuda_id(device)
    if cuda_id is None or cuda_id in cuda_map:
      return None
    cuda_map[cuda_id] = device
  return cuda_map


def _parse_cuda_visible_devices():
  """Returns integer CUDA_VISIBLE_DEVICES entries, if they are parseable."""
  value = os.environ.get('CUDA_VISIBLE_DEVICES')
  if not value:
    return None

  device_ids = []
  for raw_device_id in value.split(','):
    raw_device_id = raw_device_id.strip()
    if not raw_device_id:
      continue
    try:
      device_id = int(raw_device_id)
    except ValueError:
      return None
    if device_id < 0:
      return None
    device_ids.append(device_id)

  return device_ids or None


def _visible_cuda_candidates(cuda_map):
  """Returns EGL devices ordered by CUDA_VISIBLE_DEVICES, if possible."""
  visible_cuda_devices = _parse_cuda_visible_devices()
  if not visible_cuda_devices:
    return None

  candidates = []
  for logical_id, physical_id in enumerate(visible_cuda_devices):
    if physical_id in cuda_map:
      candidates.append(cuda_map[physical_id])
    elif logical_id in cuda_map:
      candidates.append(cuda_map[logical_id])
    else:
      return None

  return candidates


def _ordered_egl_devices(devices):
  """Returns EGL devices in CUDA order when possible."""
  cuda_map = _get_egl_device_cuda_map(devices)
  if cuda_map is None:
    return devices, None

  candidates = _visible_cuda_candidates(cuda_map)
  if candidates:
    return candidates, cuda_map

  return [cuda_map[cuda_id] for cuda_id in sorted(cuda_map)], cuda_map


def _select_egl_device(devices, cuda_map, device_id):
  """Selects an EGL device by CUDA id when possible, otherwise by index."""
  visible_cuda_devices = _parse_cuda_visible_devices()
  if cuda_map is not None and visible_cuda_devices:
    if 0 <= device_id < len(visible_cuda_devices):
      physical_id = visible_cuda_devices[device_id]
      if physical_id in cuda_map:
        return cuda_map[physical_id]
      if device_id in cuda_map:
        return cuda_map[device_id]

    if device_id in visible_cuda_devices:
      logical_id = visible_cuda_devices.index(device_id)
      if device_id in cuda_map:
        return cuda_map[device_id]
      if logical_id in cuda_map:
        return cuda_map[logical_id]

  if cuda_map is not None and device_id in cuda_map:
    return cuda_map[device_id]

  if not 0 <= device_id < len(devices):
    raise RuntimeError(
        f'The MUJOCO_EGL_DEVICE_ID environment variable must be an integer '
        f'between 0 and {len(devices)-1} (inclusive), got {device_id}.')

  return devices[device_id]


def create_initialized_egl_device_display():
  """Creates an initialized EGL display directly on a device."""
  all_devices = EGL.eglQueryDevicesEXT()
  ordered_devices, cuda_map = _ordered_egl_devices(all_devices)
  selected_device = os.environ.get('MUJOCO_EGL_DEVICE_ID', None)
  if selected_device is None:
    candidates = ordered_devices
  else:
    device_idx = int(selected_device)
    candidates = [_select_egl_device(ordered_devices, cuda_map, device_idx)]
  for device in candidates:
    display = EGL.eglGetPlatformDisplayEXT(
        EGL.EGL_PLATFORM_DEVICE_EXT, device, None)
    if display != EGL.EGL_NO_DISPLAY and EGL.eglGetError() == EGL.EGL_SUCCESS:
      # `eglInitialize` may or may not raise an exception on failure depending
      # on how PyOpenGL is configured. We therefore catch a `GLError` and also
      # manually check the output of `eglGetError()` here.
      try:
        initialized = EGL.eglInitialize(display, None, None)
      except error.GLError:
        pass
      else:
        if initialized == EGL.EGL_TRUE and EGL.eglGetError() == EGL.EGL_SUCCESS:
          return display
  return EGL.EGL_NO_DISPLAY


EGL_DISPLAY = None


EGL_ATTRIBUTES = (
    EGL.EGL_RED_SIZE, 8,
    EGL.EGL_GREEN_SIZE, 8,
    EGL.EGL_BLUE_SIZE, 8,
    EGL.EGL_ALPHA_SIZE, 8,
    EGL.EGL_DEPTH_SIZE, 24,
    EGL.EGL_STENCIL_SIZE, 8,
    EGL.EGL_COLOR_BUFFER_TYPE, EGL.EGL_RGB_BUFFER,
    EGL.EGL_SURFACE_TYPE, EGL.EGL_PBUFFER_BIT,
    EGL.EGL_RENDERABLE_TYPE, EGL.EGL_OPENGL_BIT,
    EGL.EGL_NONE
)


class GLContext:
  """An EGL context for headless accelerated OpenGL rendering on GPU devices."""

  def __init__(self, max_width, max_height):
    del max_width, max_height  # unused
    num_configs = ctypes.c_long()
    config_size = 1
    # ctypes syntax for making an array of length config_size.
    configs = (EGL.EGLConfig * config_size)()
    EGL.eglReleaseThread()
    global EGL_DISPLAY
    if EGL_DISPLAY is None:
      # only initialize for the first time
      EGL_DISPLAY = create_initialized_egl_device_display()
      if EGL_DISPLAY == EGL.EGL_NO_DISPLAY:
        raise ImportError(
          "Cannot initialize a EGL device display. This likely means that your EGL "
          "driver does not support the PLATFORM_DEVICE extension, which is "
          "required for creating a headless rendering context."
        )
      atexit.register(EGL.eglTerminate, EGL_DISPLAY)
    EGL.eglChooseConfig(
        EGL_DISPLAY,
        EGL_ATTRIBUTES,
        configs,
        config_size,
        num_configs)
    if num_configs.value < 1:
      raise RuntimeError(
          'EGL failed to find a framebuffer configuration that matches the '
          'desired attributes: {}'.format(EGL_ATTRIBUTES))
    EGL.eglBindAPI(EGL.EGL_OPENGL_API)
    self._context = EGL.eglCreateContext(
        EGL_DISPLAY, configs[0], EGL.EGL_NO_CONTEXT, None)
    if not self._context:
      raise RuntimeError('Cannot create an EGL context.')

  def make_current(self):
    global EGL_DISPLAY
    if not EGL.eglMakeCurrent(
        EGL_DISPLAY, EGL.EGL_NO_SURFACE, EGL.EGL_NO_SURFACE, self._context):
      raise RuntimeError('Failed to make the EGL context current.')

  def free(self):
    """Frees resources associated with this context."""
    global EGL_DISPLAY
    if self._context:
      current_context = EGL.eglGetCurrentContext()
      if current_context and self._context.address == current_context.address:
        EGL.eglMakeCurrent(EGL_DISPLAY, EGL.EGL_NO_SURFACE,
                           EGL.EGL_NO_SURFACE, EGL.EGL_NO_CONTEXT)
      EGL.eglDestroyContext(EGL_DISPLAY, self._context)
      EGL.eglReleaseThread()
    self._context = None

  def __del__(self):
    self.free()
