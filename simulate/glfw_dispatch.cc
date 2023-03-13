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

#include "glfw_dispatch.h"

#ifdef mjGLFW_DYNAMIC_SYMBOLS
  #ifdef _MSC_VER
    #include <windows.h>
    #include <libloaderapi.h>
  #else
    #include <dlfcn.h>
  #endif
#endif

#include <cstdlib>
#include <iostream>

namespace mujoco {

// return dispatch table for glfw functions
const struct Glfw& Glfw(void* dlhandle) {
  {
    // set static init_dlhandle
    static const void* init_dlhandle = dlhandle;

    // check that not already initialized
    if (dlhandle && dlhandle != init_dlhandle) {
      std::cerr << "dlhandle is specified when GLFW dispatch table is already "
                   "initialized\n";
      abort();
    }
  }

  // make and intialize dispatch table
  static const struct Glfw glfw = [&]() {  // create and call constructor
    // allocate
    struct Glfw glfw;

    // load glfw dynamically
#ifdef mjGLFW_DYNAMIC_SYMBOLS
  #ifdef _MSC_VER
    if (!dlhandle) dlhandle = LoadLibraryA("glfw3.dll");
    if (!dlhandle) {
      std::cerr << "cannot obtain a shared object handle\n";
      abort();
    }
    #define mjGLFW_RESOLVE_SYMBOL(func)                   \
      glfw.func = reinterpret_cast<decltype(glfw.func)>(  \
          GetProcAddress(reinterpret_cast<HMODULE>(dlhandle), #func))
  #else
    if (!dlhandle) dlhandle = dlopen("nullptr", RTLD_GLOBAL | RTLD_NOW);
    if (!dlhandle) {
      std::cerr << "cannot obtain a shared object handle\n";
      abort();
    }
    #define mjGLFW_RESOLVE_SYMBOL(func) \
      glfw.func = reinterpret_cast<decltype(glfw.func)>(dlsym(dlhandle, #func))
  #endif
#else
  #define mjGLFW_RESOLVE_SYMBOL(func) glfw.func = &::func
#endif

    // set pointers in dispatch table
#define mjGLFW_INITIALIZE_SYMBOL(func)                  \
    if (!(mjGLFW_RESOLVE_SYMBOL(func))) {               \
      std::cerr << "cannot dlsym " #func "\n";          \
      abort();                                          \
    }

    // go/keep-sorted start
    mjGLFW_INITIALIZE_SYMBOL(glfwCreateWindow);
    mjGLFW_INITIALIZE_SYMBOL(glfwDestroyWindow);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetCursorPos);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetFramebufferSize);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetKey);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetMonitorPhysicalSize);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetMouseButton);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetPrimaryMonitor);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetTime);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetVideoMode);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetWindowMonitor);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetWindowPos);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetWindowSize);
    mjGLFW_INITIALIZE_SYMBOL(glfwGetWindowUserPointer);
    mjGLFW_INITIALIZE_SYMBOL(glfwInit);
    mjGLFW_INITIALIZE_SYMBOL(glfwMakeContextCurrent);
    mjGLFW_INITIALIZE_SYMBOL(glfwPollEvents);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetClipboardString);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetCursorPosCallback);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetDropCallback);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetKeyCallback);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetMouseButtonCallback);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetScrollCallback);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetWindowMonitor);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetWindowRefreshCallback);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetWindowSizeCallback);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetWindowTitle);
    mjGLFW_INITIALIZE_SYMBOL(glfwSetWindowUserPointer);
    mjGLFW_INITIALIZE_SYMBOL(glfwSwapBuffers);
    mjGLFW_INITIALIZE_SYMBOL(glfwSwapInterval);
    mjGLFW_INITIALIZE_SYMBOL(glfwTerminate);
    mjGLFW_INITIALIZE_SYMBOL(glfwWindowHint);
    mjGLFW_INITIALIZE_SYMBOL(glfwWindowShouldClose);
    // go/keep-sorted end

#ifdef __APPLE__
    mjGLFW_INITIALIZE_SYMBOL(glfwGetNSGLContext);
#endif

#undef mjGLFW_INITIALIZE_SYMBOL

    return glfw;
  }();
  return glfw;
}
}  // namespace mujoco
