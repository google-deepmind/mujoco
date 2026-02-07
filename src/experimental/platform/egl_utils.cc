// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "experimental/platform/egl_utils.h"

#include <memory>

#if __has_include("third_party/GL/gl/include/EGL/egl.h")
  #define MUJOCO_HAS_EGL
  #include <EGL/egl.h>
  #include <EGL/eglext.h>
#endif
#include <mujoco/mujoco.h>

namespace mujoco::platform {

#ifdef MUJOCO_HAS_EGL

static EGLDisplay CreateInitializedEglDisplay() {
  auto eglQueryDevicesEXT =
      (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
  auto eglGetPlatformDisplayEXT =
      (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress(
          "eglGetPlatformDisplayEXT");

  if (!eglQueryDevicesEXT || !eglGetPlatformDisplayEXT) {
    mju_error("Failed to load EGL functions");
    return EGL_NO_DISPLAY;
  }

  // Query all available EGL devices.
  constexpr int kMaxDevices = 32;
  EGLDeviceEXT devices[kMaxDevices];
  int ndevice = 0;
  if (!eglQueryDevicesEXT(kMaxDevices, devices, &ndevice)) {
    mju_error("eglQueryDevices error: 0x%x", eglGetError());
    return EGL_NO_DISPLAY;
  }

  // Initialize the first valid EGL display.
  for (int i = 0; i < ndevice; ++i) {
    EGLDisplay display =
        eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, devices[i], nullptr);
    if (display != EGL_NO_DISPLAY) {
      int major, minor;
      EGLBoolean initialized = eglInitialize(display, &major, &minor);
      if (initialized) {
        return display;
      }
    }
  }
  mju_error("Failed to create and initialize a valid EGL display!");
  return EGL_NO_DISPLAY;
}

static EGLContext InitializeEglContext(EGLDisplay display) {
  constexpr EGLint config_attribs[] = {EGL_RED_SIZE,
                                       8,
                                       EGL_GREEN_SIZE,
                                       8,
                                       EGL_BLUE_SIZE,
                                       8,
                                       EGL_ALPHA_SIZE,
                                       8,
                                       EGL_DEPTH_SIZE,
                                       24,
                                       EGL_STENCIL_SIZE,
                                       8,
                                       EGL_COLOR_BUFFER_TYPE,
                                       EGL_RGB_BUFFER,
                                       EGL_SURFACE_TYPE,
                                       EGL_PBUFFER_BIT,
                                       EGL_RENDERABLE_TYPE,
                                       EGL_OPENGL_BIT,
                                       EGL_NONE};
  EGLint nconfig;
  EGLConfig config;
  if (!eglChooseConfig(display, config_attribs, &config, 1, &nconfig)) {
    mju_error("eglChooseConfig error: 0x%x", eglGetError());
  }

  // Bind the OpenGL API to the EGL.
  if (!eglBindAPI(EGL_OPENGL_API)) {
    mju_error("eglBindAPI error: 0x%x", eglGetError());
  }

  // Create an EGL context.
  constexpr EGLint context_attribs[] = {
      EGL_CONTEXT_MAJOR_VERSION,
      1,
      EGL_CONTEXT_MINOR_VERSION,
      5,
      EGL_CONTEXT_OPENGL_PROFILE_MASK,
      EGL_CONTEXT_OPENGL_COMPATIBILITY_PROFILE_BIT,
      EGL_NONE};
  EGLContext context =
      eglCreateContext(display, config, EGL_NO_CONTEXT, context_attribs);
  if (context == EGL_NO_CONTEXT) {
    mju_error("eglCreateContext error: 0x%x", eglGetError());
    return EGL_NO_CONTEXT;
  }

  // Make the EGL context current.
  if (!eglMakeCurrent(display, nullptr, nullptr, context)) {
    mju_error("eglMakeCurrent error: 0x%x", eglGetError());
    return EGL_NO_CONTEXT;
  }
  return context;
}

struct EglContext {
  EglContext() {
    display_ = CreateInitializedEglDisplay();
    context_ = InitializeEglContext(display_);
  }

  ~EglContext() {
    if (context_ != EGL_NO_CONTEXT) {
      eglMakeCurrent(display_, nullptr, nullptr, EGL_NO_CONTEXT);
      eglDestroyContext(display_, context_);
      context_ = EGL_NO_CONTEXT;
    }
    if (display_ != EGL_NO_DISPLAY) {
      eglTerminate(display_);
      display_ = EGL_NO_DISPLAY;
    }
  }

  EGLDisplay display_;
  EGLContext context_;
};

#endif  // MUJOCO_HAS_EGL

std::shared_ptr<void> CreateEglContext() {
  #ifdef MUJOCO_HAS_EGL
  auto egl_context = std::make_shared<EglContext>();
  return std::static_pointer_cast<void>(egl_context);
  #else
  mju_error("EGL is not supported");
  return nullptr;
  #endif
}

}  // namespace mujoco::platform
