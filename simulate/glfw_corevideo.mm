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

#include "glfw_corevideo.h"

#include <condition_variable>
#include <mutex>

#include "glfw_adapter.h"
#include "glfw_dispatch.h"

// Workaround for perpertually broken OpenGL VSync on macOS,
// most recently https://github.com/glfw/glfw/issues/2249.
namespace mujoco {
namespace {
static int DisplayLinkCallbackTrampoline(CVDisplayLinkRef display_link,
                                         const CVTimeStamp* now,
                                         const CVTimeStamp* output_time,
                                         CVOptionFlags flags_in,
                                         CVOptionFlags* flags_out,
                                         void* user_context) {
  return static_cast<GlfwCoreVideo*>(user_context)->DisplayLinkCallback();
}
}  // namespace

GlfwCoreVideo::GlfwCoreVideo(GLFWwindow* window) : window_(window) {
  CVDisplayLinkCreateWithActiveCGDisplays(&display_link_);
  CVDisplayLinkSetOutputCallback(display_link_, &DisplayLinkCallbackTrampoline, this);
  CVDisplayLinkStart(display_link_);
}

GlfwCoreVideo::~GlfwCoreVideo() {
  CVDisplayLinkStop(display_link_);
  CVDisplayLinkRelease(display_link_);
}

void GlfwCoreVideo::WaitForDisplayRefresh() {
  std::unique_lock lock(mu_);
  waiting_.store(true);
  cond_.wait(lock, [this]() { return !this->waiting_; });
}

int GlfwCoreVideo::DisplayLinkCallback() {
  if (waiting_.load()) {
    std::unique_lock lock(mu_);
    waiting_.store(false);
    cond_.notify_one();
  }
  return kCVReturnSuccess;
}

void GlfwCoreVideo::UpdateDisplayLink() {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
  NSOpenGLContext* nsgl = Glfw().glfwGetNSGLContext(window_);
#pragma clang diagnostic pop
  CVDisplayLinkSetCurrentCGDisplayFromOpenGLContext(display_link_, [nsgl CGLContextObj],
                                                    [nsgl.pixelFormat CGLPixelFormatObj]);
}
}  // namespace mujoco
