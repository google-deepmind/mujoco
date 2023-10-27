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
"""Bindings for Apple CGL."""

import ctypes
import enum

_CGL = ctypes.CDLL('/System/Library/Frameworks/OpenGL.framework/OpenGL')

CGLContextObj = ctypes.c_void_p
CGLPixelFormatObj = ctypes.c_void_p
GLint = ctypes.c_int

_CGLChoosePixelFormat = _CGL.CGLChoosePixelFormat
_CGLChoosePixelFormat.argtypes = (
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(CGLPixelFormatObj),
    ctypes.POINTER(GLint),
)

_CGLCreateContext = _CGL.CGLCreateContext
_CGLCreateContext.argtypes = (
    CGLPixelFormatObj,
    ctypes.c_int,
    CGLContextObj,
)

_CGLErrorString = _CGL.CGLErrorString
_CGLErrorString.restype = ctypes.c_char_p
_CGLErrorString.argtype = (ctypes.c_int,)

_CGLLockContext = _CGL.CGLLockContext
_CGLLockContext.argtypes = (CGLContextObj,)

_CGLReleaseContext = _CGL.CGLReleaseContext
_CGLReleaseContext.restype = None
_CGLReleaseContext.argtypes = (CGLContextObj,)

_CGLReleasePixelFormat = _CGL.CGLReleasePixelFormat
_CGLReleasePixelFormat.restype = None
_CGLReleasePixelFormat.argtypes = (CGLPixelFormatObj,)

_CGLSetCurrentContext = _CGL.CGLSetCurrentContext
_CGLSetCurrentContext.argtypes = (CGLContextObj,)

_CGLUnlockContext = _CGL.CGLUnlockContext
_CGLUnlockContext.argtypes = (CGLContextObj,)


# pylint: disable=invalid-name


class CGLOpenGLProfile(enum.IntEnum):
  CGLOGLPVersion_Legacy = 0x1000  # renderer compatible with GL1.0
  CGLOGLPVersion_3_2_Core = 0x3200  # renderer capable of GL3.2 or later
  CGLOGLPVersion_GL3_Core = 0x3200  # renderer capable of GL3.2 or later
  CGLOGLPVersion_GL4_Core = 0x4100  # renderer capable of GL4.1 or later


class CGLPixelFormatAttribute(enum.IntEnum):
  """CGLPixelFormatAttribute enum values."""
  CGLPFAAllRenderers = 1  # choose from all available renderers
  CGLPFATripleBuffer = 3  # choose a triple buffered pixel format
  CGLPFADoubleBuffer = 5  # choose a double buffered pixel format
  CGLPFAColorSize = 8  # number of color buffer bits
  CGLPFAAlphaSize = 11  # number of alpha component bits
  CGLPFADepthSize = 12  # number of depth buffer bits
  CGLPFAStencilSize = 13  # number of stencil buffer bits
  CGLPFAMinimumPolicy = 51  # never choose smaller buffers than requested
  CGLPFAMaximumPolicy = 52  # choose largest buffers of type requested
  CGLPFASampleBuffers = 55  # number of multi sample buffers
  CGLPFASample = 56  # number of samples per multi sample buffer
  CGLPFAColorFloat = 58  # color buffers store floating point pixels
  CGLPFAMultisample = 59  # choose multisampling
  CGLPFASupersample = 60  # choose supersampling
  CGLPFASampleAlpha = 61  # request alpha filtering
  CGLPFARendererID = 70  # request renderer by ID
  CGLPFANoRecovery = 72  # disable all failure recovery systems
  CGLPFAAccelerated = 73  # choose a hardware accelerated renderer
  CGLPFAClosestPolicy = 74  # choose the closest color buffer to request
  CGLPFABackingStore = 76  # back buffer contents are valid after swap
  CGLPFABackingVolatile = 77  # back buffer contents are volatile after swap
  CGLPFADisplayMask = 84  # mask limiting supported displays
  CGLPFAAllowOfflineRenderers = 96  # show offline renderers in pixel formats
  CGLPFAAcceleratedCompute = 97  # choose a hardware accelerated compute device
  CGLPFAOpenGLProfile = 99  # specify an OpenGL Profile to use
  CGLPFASupportsAutomaticGraphicsSwitching = 101  # responds to display changes
  CGLPFAVirtualScreenCount = 128  # number of virtual screens in this format

  # Note: the following attributes are deprecated in Core Profile
  CGLPFAAuxBuffers = 7  # number of aux buffers
  CGLPFAAccumSize = 14  # number of accum buffer bits
  CGLPFAAuxDepthStencil = 57  # each aux buffer has its own depth stencil

  CGLPFAStereo = 6
  CGLPFAOffScreen = 53
  CGLPFAWindow = 80
  CGLPFACompliant = 83
  CGLPFAPBuffer = 90
  CGLPFARemotePBuffer = 91

  CGLPFASingleRenderer = 71
  CGLPFARobust = 75
  CGLPFAMPSafe = 78
  CGLPFAMultiScreen = 81
  CGLPFAFullScreen = 54


# pylint: enable=invalid-name


class CGLError(RuntimeError):  # pylint: disable=g-bad-exception-name
  pass


def _make_checked(func):
  def checked_func(*args):
    err = func(*args)
    if err:
      raise CGLError(_CGLErrorString(err).decode())
  return checked_func


CGLChoosePixelFormat = _make_checked(_CGLChoosePixelFormat)
CGLCreateContext = _make_checked(_CGLCreateContext)
CGLLockContext = _make_checked(_CGLLockContext)
CGLReleaseContext = _CGLReleaseContext
CGLReleasePixelFormat = _CGLReleasePixelFormat
CGLSetCurrentContext = _make_checked(_CGLSetCurrentContext)
CGLUnlockContext = _make_checked(_CGLUnlockContext)
