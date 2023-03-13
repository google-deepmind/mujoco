// Copyright 2022 DeepMind Technologies Limited
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

#ifndef MUJOCO_SIMULATE_GLFW_DISPATCH_H_
#define MUJOCO_SIMULATE_GLFW_DISPATCH_H_

#include <GLFW/glfw3.h>

#ifdef __APPLE__
#define GLFW_EXPOSE_NATIVE_NSGL
#include <GLFW/glfw3native.h>
#endif

namespace mujoco {
// Dynamic dispatch table for GLFW functions required by Simulate.
// This allows us to use GLFW without introducing a link-time dependency on the
// library, which is useful e.g. when using GLFW via Python.
struct Glfw {
#define mjGLFW_DECLARE_SYMBOL(func) decltype(&::func) func
  // go/keep-sorted start
  mjGLFW_DECLARE_SYMBOL(glfwCreateWindow);
  mjGLFW_DECLARE_SYMBOL(glfwDestroyWindow);
  mjGLFW_DECLARE_SYMBOL(glfwGetCursorPos);
  mjGLFW_DECLARE_SYMBOL(glfwGetFramebufferSize);
  mjGLFW_DECLARE_SYMBOL(glfwGetKey);
  mjGLFW_DECLARE_SYMBOL(glfwGetMonitorPhysicalSize);
  mjGLFW_DECLARE_SYMBOL(glfwGetMouseButton);
  mjGLFW_DECLARE_SYMBOL(glfwGetPrimaryMonitor);
  mjGLFW_DECLARE_SYMBOL(glfwGetTime);
  mjGLFW_DECLARE_SYMBOL(glfwGetVideoMode);
  mjGLFW_DECLARE_SYMBOL(glfwGetWindowMonitor);
  mjGLFW_DECLARE_SYMBOL(glfwGetWindowPos);
  mjGLFW_DECLARE_SYMBOL(glfwGetWindowSize);
  mjGLFW_DECLARE_SYMBOL(glfwGetWindowUserPointer);
  mjGLFW_DECLARE_SYMBOL(glfwInit);
  mjGLFW_DECLARE_SYMBOL(glfwMakeContextCurrent);
  mjGLFW_DECLARE_SYMBOL(glfwPollEvents);
  mjGLFW_DECLARE_SYMBOL(glfwSetClipboardString);
  mjGLFW_DECLARE_SYMBOL(glfwSetCursorPosCallback);
  mjGLFW_DECLARE_SYMBOL(glfwSetDropCallback);
  mjGLFW_DECLARE_SYMBOL(glfwSetKeyCallback);
  mjGLFW_DECLARE_SYMBOL(glfwSetMouseButtonCallback);
  mjGLFW_DECLARE_SYMBOL(glfwSetScrollCallback);
  mjGLFW_DECLARE_SYMBOL(glfwSetWindowMonitor);
  mjGLFW_DECLARE_SYMBOL(glfwSetWindowRefreshCallback);
  mjGLFW_DECLARE_SYMBOL(glfwSetWindowSizeCallback);
  mjGLFW_DECLARE_SYMBOL(glfwSetWindowTitle);
  mjGLFW_DECLARE_SYMBOL(glfwSetWindowUserPointer);
  mjGLFW_DECLARE_SYMBOL(glfwSwapBuffers);
  mjGLFW_DECLARE_SYMBOL(glfwSwapInterval);
  mjGLFW_DECLARE_SYMBOL(glfwTerminate);
  mjGLFW_DECLARE_SYMBOL(glfwWindowHint);
  mjGLFW_DECLARE_SYMBOL(glfwWindowShouldClose);
  // go/keep-sorted end

#ifdef __APPLE__
  mjGLFW_DECLARE_SYMBOL(glfwGetNSGLContext);
#endif

#undef mjGLFW_DECLARE_SYMBOL
};

const struct Glfw& Glfw(void* dlhandle = nullptr);
}  // namespace mujoco

#endif  // MUJOCO_SIMULATE_GLFW_DISPATCH_H_
