# Copyright 2023 DeepMind Technologies Limited
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
"""An Apple CGL context for offscreen rendering on macOS."""

import atexit
import ctypes
import os

from mujoco.cgl import cgl

_ATTRIB = cgl.CGLPixelFormatAttribute
_PROFILE = cgl.CGLOpenGLProfile


class GLContext:
  """An EGL context for headless accelerated OpenGL rendering on GPU devices."""

  def __init__(self, max_width, max_height):
    del max_width, max_height  # unused
    attrib_values = (
        _ATTRIB.CGLPFAOpenGLProfile, _PROFILE.CGLOGLPVersion_Legacy,
        _ATTRIB.CGLPFAColorSize, 24,
        _ATTRIB.CGLPFAAlphaSize, 8,
        _ATTRIB.CGLPFADepthSize, 24,
        _ATTRIB.CGLPFAStencilSize, 8,
        _ATTRIB.CGLPFAMultisample,
        _ATTRIB.CGLPFASampleBuffers, 1,
        _ATTRIB.CGLPFASample, 4,
        _ATTRIB.CGLPFAAccelerated,
        0,
    )
    attribs = (ctypes.c_int * len(attrib_values))(*attrib_values)
    self._pix = cgl.CGLPixelFormatObj()
    npix = cgl.GLint()
    cgl.CGLChoosePixelFormat(
        attribs, ctypes.byref(self._pix), ctypes.byref(npix)
    )

    self._context = cgl.CGLContextObj()
    cgl.CGLCreateContext(self._pix, 0, ctypes.byref(self._context))

  def make_current(self):
    cgl.CGLSetCurrentContext(self._context)
    cgl.CGLLockContext(self._context)

  def free(self):
    """Frees resources associated with this context."""
    if self._context:
      cgl.CGLUnlockContext(self._context)
      cgl.CGLSetCurrentContext(None)
      cgl.CGLReleaseContext(self._context)
    self._context = None

    if self._pix:
      cgl.CGLReleasePixelFormat(self._pix)
    self._context = None

  def __del__(self):
    self.free()
