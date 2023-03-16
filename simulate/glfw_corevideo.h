// Copyright 2023 DeepMind Technologies Limited
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

#ifndef MUJOCO_SIMULATE_GLFW_COREVIDEO_H_
#define MUJOCO_SIMULATE_GLFW_COREVIDEO_H_

#ifndef __APPLE__
#error "This header only works on macOS."
#endif

#include <atomic>
#include <condition_variable>
#include <mutex>

#include "glfw_dispatch.h"

#ifdef __OBJC__
#import <CoreVideo/CoreVideo.h>
#else
typedef void* CVDisplayLinkRef;
#endif

// Workaround for perpertually broken OpenGL VSync on macOS,
// most recently https://github.com/glfw/glfw/issues/2249.
namespace mujoco {
class GlfwCoreVideo {
 public:
  GlfwCoreVideo(GLFWwindow* window);
  ~GlfwCoreVideo();

  void WaitForDisplayRefresh();
  int DisplayLinkCallback();
  void UpdateDisplayLink();

 private:
  GLFWwindow* window_;
  CVDisplayLinkRef display_link_;

  std::atomic_bool waiting_;
  std::mutex mu_;
  std::condition_variable cond_;
};
}  // namespace mujoco


#endif  // MUJOCO_SIMULATE_GLFW_COREVIDEO_H_
