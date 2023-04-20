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

#ifndef MUJOCO_SIMULATE_PLATFORM_UI_ADAPTER_H_
#define MUJOCO_SIMULATE_PLATFORM_UI_ADAPTER_H_

#include <utility>

#include <mujoco/mujoco.h>

namespace mujoco {
class PlatformUIAdapter {
 public:
  virtual ~PlatformUIAdapter();

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

  // Optionally overrideable function to (re)create an mjrContext for an mjModel
  virtual bool RefreshMjrContext(const mjModel* m, int fontscale);

  virtual bool EnsureContextSize();

  // Pure virtual functions to be implemented by individual adapters
  virtual std::pair<double, double> GetCursorPosition() const = 0;
  virtual double GetDisplayPixelsPerInch() const = 0;
  virtual std::pair<int, int> GetFramebufferSize() const = 0;
  virtual std::pair<int, int> GetWindowSize() const = 0;
  virtual bool IsGPUAccelerated() const = 0;
  virtual void PollEvents() = 0;
  virtual void SetClipboardString(const char* text) = 0;
  virtual void SetVSync(bool enabled) = 0;
  virtual void SetWindowTitle(const char* title) = 0;
  virtual bool ShouldCloseWindow() const = 0;
  virtual void SwapBuffers() = 0;
  virtual void ToggleFullscreen() = 0;

  virtual bool IsLeftMouseButtonPressed() const = 0;
  virtual bool IsMiddleMouseButtonPressed() const = 0;
  virtual bool IsRightMouseButtonPressed() const = 0;

  virtual bool IsAltKeyPressed() const = 0;
  virtual bool IsCtrlKeyPressed() const = 0;
  virtual bool IsShiftKeyPressed() const = 0;

  virtual bool IsMouseButtonDownEvent(int act) const = 0;
  virtual bool IsKeyDownEvent(int act) const = 0;

  virtual int TranslateKeyCode(int key) const = 0;
  virtual mjtButton TranslateMouseButton(int button) const = 0;

 protected:
  PlatformUIAdapter();

  // Event handlers
  void OnFilesDrop(int count, const char** paths);
  void OnKey(int key, int scancode, int act);
  void OnMouseButton(int button, int act);
  void OnMouseMove(double x, double y);
  void OnScroll(double xoffset, double yoffset);
  void OnWindowRefresh();
  void OnWindowResize(int width, int height);

  mjuiState state_;
  void (*event_callback_)(mjuiState*);
  void (*layout_callback_)(mjuiState*);

  mjrContext con_;
  const mjModel* last_model_ = nullptr;
  int last_fontscale_ = -1;

 private:
  void UpdateMjuiState();
};
}  // namespace mujoco

#endif  // MUJOCO_SIMULATE_PLATFORM_UI_ADAPTER_H_
