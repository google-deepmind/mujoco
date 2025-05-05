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

#ifndef MUJOCO_SIMULATE_GLFW_ADAPTER_H_
#define MUJOCO_SIMULATE_GLFW_ADAPTER_H_

#include <utility>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#ifdef __APPLE__
#include <optional>
#include "glfw_corevideo.h"
#endif

namespace mujoco {
class GlfwAdapter {
 public:
  GlfwAdapter();
  ~GlfwAdapter();

  void MakeCurrent();
  std::pair<double, double> GetCursorPosition() const;
  double GetDisplayPixelsPerInch() const;
  std::pair<int, int> GetFramebufferSize() const;
  std::pair<int, int> GetWindowSize() const;
  bool IsGPUAccelerated() const;
  void PollEvents();
  void SetClipboardString(const char* text);
  void SetVSync(bool enabled);
  void SetWindowTitle(const char* title);
  bool ShouldCloseWindow() const;
  void SwapBuffers();
  void ToggleFullscreen();

  bool IsLeftMouseButtonPressed() const;
  bool IsMiddleMouseButtonPressed() const;
  bool IsRightMouseButtonPressed() const;

  bool IsAltKeyPressed() const;
  bool IsCtrlKeyPressed() const;
  bool IsShiftKeyPressed() const;

  bool IsMouseButtonDownEvent(int act) const;
  bool IsKeyDownEvent(int act) const;

  int TranslateKeyCode(int key) const;
  mjtButton TranslateMouseButton(int button) const;

  /* Platform Adapter members */
  inline mjuiState& state() { return state_; }
  inline const mjuiState& state() const { return state_; }

  inline mjrContext& mjr_context() { return con_; }
  inline const mjrContext& mjr_context() const { return con_; }

  inline void SetEventCallback(void (*event_callback)(mjuiState*)) {
    event_callback_ = event_callback;
  }

  inline void SetLayoutCallback(void (*layout_callback)(mjuiState*)) {
    layout_callback_ = layout_callback;
  }

  // Optionally overridable function to (re)create an mjrContext for an mjModel
  virtual bool RefreshMjrContext(const mjModel* m, int fontscale);

  virtual bool EnsureContextSize();

  // Pure virtual functions to be implemented by individual adapters

 protected:
  void FreeMjrContext();

  // Event handlers
  void OnFilesDrop(int count, const char** paths);
  virtual void OnKey(int key, int scancode, int act);
  void OnMouseButton(int button, int act);
  void OnMouseMove(double x, double y);
  void OnScroll(double xoffset, double yoffset);
  void OnWindowRefresh();
  void OnWindowResize(int width, int height);

  mjuiState state_;
  int last_key_;
  void (*event_callback_)(mjuiState*);
  void (*layout_callback_)(mjuiState*);

  mjrContext con_;
  const mjModel* last_model_ = nullptr;
  int last_fontscale_ = -1;

 private:
   void UpdateMjuiState();

  GLFWvidmode vidmode_;
  GLFWwindow* window_;

  // store last window information when going to full screen
  std::pair<int, int> window_pos_;
  std::pair<int, int> window_size_;

#ifdef __APPLE__
  // Workaround for perpertually broken OpenGL VSync on macOS,
  // most recently https://github.com/glfw/glfw/issues/2249.
  std::optional<GlfwCoreVideo> core_video_;
#endif
};
}  // namespace mujoco

#endif  // MUJOCO_SIMULATE_GLFW_ADAPTER_H_
