// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This file implements the Web Viewer Browser Client. NetImgui is used to
// transmit remote ImGui draw data in a browser and the filament renderer is
// used to render the UI and the 3D scene.

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/fetch.h>
#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <cfloat>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <new>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <implot.h>
#include <mujoco/mujoco.h>
#include "experimental/studio/hal/filament_renderer.h"
#include "experimental/studio/hal/window.h"
#include "experimental/studio/sim/model_holder.h"
#include "experimental/studio/ux/interaction.h"
#include <NetImgui_Api.h>
#include "google/logging.h"
#include "state_payload.h"
#include "web_client_local_ui.h"
#include "web_client_remote_ui.h"
#include "web_client_session.h"

#if !defined(__EMSCRIPTEN__)
#error "web_client.cc is only supported for Emscripten builds"
#endif

// No-op wrapper for glGetError(), installed via -Wl,--wrap=glGetError.
// Filament's GL backend calls glGetError() hundreds of times per frame
// through GLUtils::checkGLError/assertGLError. In WebGL each call forces a
// synchronous GPU pipeline flush across the JS-WASM bridge (~0.2ms each).
// This stub eliminates that cost entirely.
extern "C" GLenum __wrap_glGetError(void) { return GL_NO_ERROR; }

using mujoco::studio::DisconnectNotice;
using mujoco::studio::RemoteUi;
using mujoco::studio::RemoteUiState;
using mujoco::studio::RoleWindow;
using mujoco::studio::Session;
using mujoco::studio::SessionRole;
using mujoco::studio::SessionView;
using mujoco::studio::StatePayloadView;

// How a spectating page drives its camera. The free modes control the local
// camera directly; kSpecCamFollow mirrors the controller's camera from the
// state broadcast.
enum SpectatorCamMode {
  kSpecCamTumble = 0,  // Orbit around the lookat point with the mouse.
  kSpecCamWasd,        // Fly camera: WASD/QE moves, mouse drag turns.
  kSpecCamFollow,      // Follow the controller's camera.
};

// Byte-rate telemetry shown in the role window.
struct Telemetry {
  double last_rate_time = 0;
  uint64_t gui_bytes_per_sec = 0;
  uint64_t sim_bytes_per_sec = 0;
};

struct ModelDownloadStatus {
  bool is_downloading = true;
  size_t bytes_downloaded = 0;
  size_t total_bytes = 0;
  int retry_count = 0;
};

// The implementation of every interface needed by the session and remote UI.
class AppCallbacks final : public RemoteUi::Callbacks,
                           public Session::Callbacks {
 public:
  // RemoteUi::Callbacks
  uintptr_t UploadTexture(uintptr_t current, const std::byte* rgba,
                          uint32_t width, uint32_t height) override;
  bool GpuReady() override;

  // Session::Callbacks
  bool ReadyForPayload() override;
  void OnPayload(const StatePayloadView& view) override;
  void OnModelChanged() override;
  void ConnectRemoteUi() override;
  void ShutdownRemoteUi() override;
  void SetCameraMode(int mode) override;
};

struct App {
  std::unique_ptr<mujoco::studio::Window> window;
  std::unique_ptr<mujoco::studio::ModelHolder> model_holder;
  mujoco::studio::Renderer* renderer = nullptr;
  mjvPerturb perturb;
  mjvCamera camera;
  mjvOption vis_options;

  // Spectator camera
  int spectator_cam_mode = kSpecCamTumble;
  float spectator_cam_speed = 0.001f;  // WASD speed; accelerates while held.

  Telemetry telemetry;

  // Main loop frame counters (MainLoopImpl): reconnect pacing for the state
  // WebSocket. (The session paces its own /ui claims by time.)
  int frame_count = 0;
  int last_state_retry_frame = 0;

  // Backend state received from the Python simulation via WebSocket.
  std::vector<mjtNum> backend_state;
  int backend_state_sig = 0;
  bool backend_state_dirty = false;

  // User-injected geoms received with the state payload.
  std::vector<mjvGeom> extra_geoms;

  ModelDownloadStatus download_status;

  AppCallbacks callbacks;
  RemoteUi remote_ui{callbacks};
  Session session{callbacks};
  DisconnectNotice disconnect_notice;
  RoleWindow role_window;
};
App g_app;

// WebSocket base URL matching the page origin, e.g. "ws://host:8080" or
// "wss://tunnel.example.com" behind an HTTPS tunnel. All viewer WebSockets
// are served as paths (/ui, /state) on the same host and port as the page
// itself, so exposing or tunneling that single port exposes the whole
// viewer.
std::string GetWsBaseUrl() {
  char* base = emscripten_run_script_string(
      "(window.location.protocol === 'https:' ? 'wss://' : 'ws://') + "
      "window.location.host");
  std::string url = base != nullptr ? base : "";
  if (url == "ws://" || url == "wss://" || url.empty()) {
    // No host in the page URL (e.g. file://) — assume a local server.
    return "ws://localhost:8080";
  }
  return url;
}

// Stable per-tab id sent with every WebSocket connect (?sid=...), letting the
// server tie this page's /ui and /state connections together. It lives in
// sessionStorage so it survives page reloads: on a model change every page
// reloads and the server restarts with the controller slot reserved for the
// previous controller's sid, a reload-stable id is what lets the controller
// keep control.
std::string GetSessionId() {
  static const std::string sid = emscripten_run_script_string(
      "(function() {"
      "  Module.session_id = Module.session_id ||"
      "      sessionStorage.getItem('mjwv_sid') ||"
      "      (Date.now().toString(36) + Math.random().toString(36).slice(2));"
      "  sessionStorage.setItem('mjwv_sid', Module.session_id);"
      "  return Module.session_id;"
      "})()");
  return sid;
}

std::string WsUrl(const char* path) {
  return GetWsBaseUrl() + path + "?sid=" + GetSessionId();
}

// Returns true if the Filament rendering context is initialized and ready for
// GPU texture uploads. The Renderer object is created in main() and is always
// non-null, but the Filament context is only initialized when Renderer::Init()
// is called from SetupScene after the async model fetch completes.
bool IsFilamentReady() {
  return g_app.renderer && g_app.model_holder && g_app.model_holder->ok();
}

// Applies a parsed state payload to the app. Called via AppCallbacks::OnPayload
// after Session validates model CRC and payload readiness.
void ApplyStatePayload(const StatePayloadView& view) {
  mjModel* model = g_app.model_holder->model();

  // Physics state. Guard against a size mismatch (e.g. a stale packet from
  // before a model change).
  if (view.physics != nullptr) {
    const size_t expected_bytes =
        mj_stateSize(model, view.physics_spec) * sizeof(mjtNum);
    if (view.physics_bytes == expected_bytes) {
      g_app.backend_state.resize(view.physics_bytes / sizeof(mjtNum));
      memcpy(g_app.backend_state.data(), view.physics, view.physics_bytes);
      g_app.backend_state_sig = view.physics_spec;
      g_app.backend_state_dirty = true;
    } else {
      LOG(Warning, "Physics state size mismatch (%zu != %zu); dropping",
          view.physics_bytes, expected_bytes);
    }
  }

  // Render state. The headless viewer owns the camera and perturbation state
  // (all input is forwarded to it and handled by the same code as the native
  // viewer); the browser just renders them.
  if (view.render_state != nullptr) {
    mujoco::studio::RenderStateView rs;
    mujoco::studio::ParseRenderState(view.render_state, &rs);

    // Spectators in a free camera mode keep their own local camera; everyone
    // else (the controller, and spectators in Follow Controller) mirrors the
    // controller's camera.
    if (g_app.session.Role() != SessionRole::kSpectating ||
        g_app.spectator_cam_mode == kSpecCamFollow) {
      g_app.camera = rs.camera;
    }

    g_app.perturb = rs.perturb;
    g_app.vis_options = rs.vis_options;
    model->opt = rs.opt;
    model->vis = rs.vis;
    model->stat = rs.stat;

    // Apply render flags to the renderer's scene if available.
    if (g_app.renderer) {
      memcpy(g_app.renderer->GetRenderFlags(), rs.render_flags, mjNRNDFLAG);
    }
  }

  // Extra geoms. memcpy since the payload data is not guaranteed to be aligned.
  g_app.extra_geoms.resize(view.extra_geom_count);
  if (view.extra_geom_count > 0) {
    memcpy(g_app.extra_geoms.data(), view.extra_geoms,
           view.extra_geom_count * sizeof(mjvGeom));
  }
}

void SetSpectatorCameraMode(int mode) {
  if (mode == g_app.spectator_cam_mode) {
    return;
  }
  g_app.spectator_cam_mode = mode;
  if (!g_app.model_holder || !g_app.model_holder->ok()) {
    return;
  }
  const mjModel* model = g_app.model_holder->model();
  if (mode == kSpecCamTumble) {
    mujoco::studio::SetCamera(model, &g_app.camera,
                                mujoco::studio::kTumbleCameraIdx);
  } else if (mode == kSpecCamWasd) {
    mujoco::studio::SetCamera(model, &g_app.camera,
                                mujoco::studio::kFreeCameraIdx);
  }
  // kSpecCamFollow: the next state payload restores the controller's camera.
}

// Local camera control for a spectating page in a free camera mode. The
// controller's input goes to the headless viewer instead (CaptureAndSendInput),
// which streams its camera back over the state WebSocket.
// TODO(matijak): Share the camera handling code with the studio app (e.g. in
// platform/ux/interaction.cc) instead of duplicating it here.
void HandleSpectatorCameraInput() {
  if (g_app.spectator_cam_mode == kSpecCamFollow) {
    return;
  }
  if (!g_app.model_holder || !g_app.model_holder->ok()) {
    return;
  }
  const mjModel* model = g_app.model_holder->model();
  ImGuiIO& io = ImGui::GetIO();
  const bool wasd = g_app.spectator_cam_mode == kSpecCamWasd;

  // The camera can be fixed on entry (SetupScene honours the model's
  // vis.global.cameraid, and Follow Controller mirrors whatever camera the
  // controller uses), and mjv_moveCamera ignores fixed cameras. Coerce to a
  // free camera so the free modes always respond to input.
  if (g_app.camera.type == mjCAMERA_FIXED) {
    mjv_defaultFreeCamera(model, &g_app.camera);
    if (wasd) {
      mujoco::studio::SetCamera(model, &g_app.camera,
                                  mujoco::studio::kFreeCameraIdx);
    }
  }

  if (!io.WantCaptureMouse && io.DisplaySize.x > 0 && io.DisplaySize.y > 0) {
    const float mouse_dx = io.MouseDelta.x / io.DisplaySize.x;
    const float mouse_dy = io.MouseDelta.y / io.DisplaySize.y;
    const bool is_mouse_dragging =
        (mouse_dx != 0.0f || mouse_dy != 0.0f) &&
        (ImGui::IsMouseDown(ImGuiMouseButton_Left) ||
         ImGui::IsMouseDown(ImGuiMouseButton_Right) ||
         ImGui::IsMouseDown(ImGuiMouseButton_Middle));
    if (is_mouse_dragging) {
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        if (wasd) {
          mjv_moveCamera(model, mjMOUSE_TURN_H, mouse_dx, 0.f, &g_app.camera);
          mjv_moveCamera(model, mjMOUSE_TURN_V, 0.f, mouse_dy, &g_app.camera);
        } else {
          mjv_moveCamera(model, mjMOUSE_ROTATE_H, mouse_dx, 0.f, &g_app.camera);
          mjv_moveCamera(model, mjMOUSE_ROTATE_V, 0.f, mouse_dy, &g_app.camera);
        }
      } else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle) && !wasd) {
        mjv_moveCamera(model, mjMOUSE_ZOOM, 0.f, mouse_dy, &g_app.camera);
      }
      if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        mjv_moveCamera(model, io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V,
                       mouse_dx, mouse_dy, &g_app.camera);
      }
    }
    // Mouse scroll zooms towards/away from the lookat point; ignored by the
    // user-centered WASD camera which has no lookat point.
    const float mouse_scroll = io.MouseWheel / 50.0f;
    if (mouse_scroll != 0.0f && !wasd) {
      mjv_moveCamera(model, mjMOUSE_ZOOM, 0.f, -mouse_scroll, &g_app.camera);
    }
  }

  // WASD/QE flying, with the same accelerating speed as the studio app.
  if (wasd && !io.WantCaptureKeyboard) {
    bool moved = false;
    const float speed = g_app.spectator_cam_speed;
    if (ImGui::IsKeyDown(ImGuiKey_W)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_H_REL, 0, speed, &g_app.camera);
      moved = true;
    } else if (ImGui::IsKeyDown(ImGuiKey_S)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_H_REL, 0, -speed, &g_app.camera);
      moved = true;
    }
    if (ImGui::IsKeyDown(ImGuiKey_A)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_H_REL, -speed, 0, &g_app.camera);
      moved = true;
    } else if (ImGui::IsKeyDown(ImGuiKey_D)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_H_REL, speed, 0, &g_app.camera);
      moved = true;
    }
    if (ImGui::IsKeyDown(ImGuiKey_Q)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_V_REL, 0, speed, &g_app.camera);
      moved = true;
    } else if (ImGui::IsKeyDown(ImGuiKey_E)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_V_REL, 0, -speed, &g_app.camera);
      moved = true;
    }
    if (moved) {
      const float max_speed = io.KeyShift ? 0.1f : 0.01f;
      g_app.spectator_cam_speed =
          std::min(g_app.spectator_cam_speed + 0.001f, max_speed);
    } else {
      g_app.spectator_cam_speed = 0.001f;
    }
  }
}

// --- AppCallbacks method bodies (see the class declaration above App). ---

uintptr_t AppCallbacks::UploadTexture(uintptr_t current, const std::byte* rgba,
                                      uint32_t width, uint32_t height) {
  return g_app.renderer->UploadImage(current, rgba, width, height,
                                     rgba ? 4 : 0);
}

bool AppCallbacks::GpuReady() { return IsFilamentReady(); }

bool AppCallbacks::ReadyForPayload() {
  return g_app.model_holder && g_app.model_holder->ok();
}

void AppCallbacks::OnPayload(const StatePayloadView& view) {
  ApplyStatePayload(view);
}

void AppCallbacks::OnModelChanged() {
  // Mark downloading immediately so the main loop stops applying state updates
  // to the old model while the new one downloads asynchronously.
  g_app.download_status.is_downloading = true;
  EM_ASM({ reloadModel(); });
}

void AppCallbacks::ConnectRemoteUi() { g_app.remote_ui.Connect(WsUrl("/ui")); }

void AppCallbacks::ShutdownRemoteUi() { g_app.remote_ui.Shutdown(); }

void AppCallbacks::SetCameraMode(int mode) { SetSpectatorCameraMode(mode); }

void BuildBrowserGui() {
  // Refresh the byte-rate telemetry the role window shows.
  const double now = ImGui::GetTime();
  if (now - g_app.telemetry.last_rate_time >= 1.0) {
    g_app.telemetry.gui_bytes_per_sec = g_app.remote_ui.ConsumeByteCount();
    g_app.telemetry.sim_bytes_per_sec = g_app.session.ConsumeByteCount();
    g_app.telemetry.last_rate_time = now;
  }

  const double last_msg = g_app.session.LastMessageTime();
  const double stale_sec =
      last_msg > 0 ? emscripten_get_now() / 1000.0 - last_msg : -1.0;
  g_app.disconnect_notice.Draw(g_app.session.ServerCloseCode(), stale_sec,
                               g_app.download_status.is_downloading);

  SessionView view;
  g_app.session.FillView(&view);
  view.gui_bytes_per_sec = g_app.telemetry.gui_bytes_per_sec;
  view.sim_bytes_per_sec = g_app.telemetry.sim_bytes_per_sec;
  view.have_remote_frame = g_app.remote_ui.RemoteDrawData() != nullptr;
  view.camera_mode = g_app.spectator_cam_mode;
  view.is_downloading = g_app.download_status.is_downloading;
  view.bytes_downloaded = g_app.download_status.bytes_downloaded;
  view.total_bytes = g_app.download_status.total_bytes;
  view.retry_count = g_app.download_status.retry_count;
  // The session is the SessionActions implementation: the role window's
  // intents land there directly.
  g_app.role_window.Draw(view, g_app.session);
}

//=================================================================================================
// Main loop — called once per frame.
//=================================================================================================
void MainLoopImpl();

void MainLoop() {
  // An exception escaping the requestAnimationFrame callback kills the main
  // loop silently causing the canvas to freeze on the last rendered frame and
  // input capture to stop with only an opaque "Uncaught <ptr>" in the console.
  // Here we catch, log, and stop explicitly instead.
  try {
    MainLoopImpl();
  } catch (const std::exception& e) {
    LOG(Error, "FATAL: uncaught exception in MainLoop: %s", e.what());
    emscripten_cancel_main_loop();
  } catch (...) {
    LOG(Error, "FATAL: uncaught non-std exception in MainLoop");
    emscripten_cancel_main_loop();
  }
}

void MainLoopImpl() {
  g_app.frame_count++;

  // Session upkeep (the liveness heartbeat).
  g_app.session.Update();

  // Reconnect the state WebSocket if it dropped. Receiving a payload with a
  // different model identity then triggers a page reload (Session). Deliberate
  // server closes (session full, inactivity) are transient; retry them too,
  // using a gentler pace.
  const int state_retry_interval =
      g_app.session.ServerCloseCode() != 0 ? 300 : 60;
  if (!g_app.session.HasSocket() && g_app.model_holder &&
      g_app.model_holder->ok() && !g_app.download_status.is_downloading &&
      g_app.frame_count - g_app.last_state_retry_frame > state_retry_interval) {
    g_app.last_state_retry_frame = g_app.frame_count;
    LOG(Info, "State WebSocket down; reconnecting...");
    g_app.session.Connect(WsUrl("/state"));
  }

  // Pass the remote UI (/ui) WebSocket state to the session state machine.
  // The session uses this to manage attempts to claim the single controller
  // slot, handle retry pacing if a claim is rejected or dropped, and drive role
  // transitions (Controlling vs Spectating).
  RemoteUiState ui_state = RemoteUiState::kNoSocket;
  if (g_app.remote_ui.HasSocket()) {
    switch (g_app.remote_ui.ConnectionState()) {
      case RemoteUi::ReadyState::kOpen:
        ui_state = RemoteUiState::kOpen;
        break;
      case RemoteUi::ReadyState::kClosed:
      case RemoteUi::ReadyState::kError:
        ui_state = RemoteUiState::kClosedOrError;
        break;
      default:
        ui_state = RemoteUiState::kConnecting;
        break;
    }
  }
  g_app.session.HandleRemoteUiState(ui_state, g_app.remote_ui.CloseCode());

  // Process incoming UI data BEFORE the ImGui frame: textures and draw
  // frames must be ready before we start the local ImGui frame and render.
  if (g_app.window) {
    g_app.remote_ui.SetMaxClip(static_cast<float>(g_app.window->GetWidth()),
                               static_cast<float>(g_app.window->GetHeight()));
  }
  g_app.remote_ui.ReceiveAndProcessCommands(g_app.frame_count);

  // Event loop and ImGui NewFrame via window abstraction.
  mujoco::studio::Window::Status status = g_app.window->NewFrame();
  if (status == mujoco::studio::Window::kQuitting) {
    // NewFrame() started an ImGui frame; end it before bailing out.
    ImGui::EndFrame();
    emscripten_cancel_main_loop();
    return;
  }

  // For the controller, all scene interaction (camera orbit/zoom, perturbation,
  // picking) is handled by the headless viewer: CaptureAndSendInput()
  // forwards this frame's input over NetImgui, and the resulting camera/perturb
  // state streams back over the state WebSocket (see ApplyStatePayload).
  // Spectators have no input channel; in a free camera mode they drive their
  // local camera directly.
  g_app.remote_ui.CaptureAndSendInput();
  if (g_app.session.Role() == SessionRole::kSpectating) {
    HandleSpectatorCameraInput();
  }

  BuildBrowserGui();

  // Finalize the local ImGui draw data. ImguiBridge::Update() (called inside
  // Filament's Render) will call ImGui::Render() again, but that is a no-op
  // once the frame has already been rendered.
  ImGui::Render();

  // Inject remote draw lists into the local ImDrawData so that Filament's
  // ImguiBridge renders them alongside the local UI. Remote lists are inserted
  // first (background) and local lists are re-added after (foreground), so the
  // local UI always renders on top of remote content. Note that
  // ImGui::GetDrawData() is only valid after ImGui::Render() and until the next
  // call to ImGui::NewFrame().
  ImDrawData* remote_draw_data = g_app.remote_ui.RemoteDrawData();
  if (remote_draw_data && remote_draw_data->Valid &&
      g_app.session.Role() == SessionRole::kControlling) {
    ImDrawData* local_draw_data = ImGui::GetDrawData();
    if (local_draw_data) {
      // Save local draw lists.
      ImVector<ImDrawList*> local_lists;
      local_lists.reserve(local_draw_data->CmdListsCount);
      for (int i = 0; i < local_draw_data->CmdListsCount; ++i) {
        local_lists.push_back(local_draw_data->CmdLists[i]);
      }

      // Clear and rebuild: remote first, then local.
      local_draw_data->CmdLists.resize(0);
      local_draw_data->CmdListsCount = 0;
      local_draw_data->TotalVtxCount = 0;
      local_draw_data->TotalIdxCount = 0;

      // Remote draw lists (background).
      for (int i = 0; i < remote_draw_data->CmdListsCount; ++i) {
        local_draw_data->AddDrawList(remote_draw_data->CmdLists[i]);
      }

      // Local draw lists (foreground).
      for (int i = 0; i < local_lists.Size; ++i) {
        local_draw_data->AddDrawList(local_lists[i]);
      }
    }
  }

  // Render the scene and all ImGui UI (local and remote).
  if (g_app.renderer && g_app.model_holder && g_app.model_holder->ok()) {
    // Apply backend state if available.
    if (g_app.backend_state_dirty && !g_app.download_status.is_downloading) {
      mjModel* model = g_app.model_holder->model();
      mjData* data = g_app.model_holder->data();
      mj_setState(model, data, g_app.backend_state.data(),
                  g_app.backend_state_sig);
      mj_forward(model, data);
      g_app.backend_state_dirty = false;
    }

    int width =
        static_cast<int>(g_app.window->GetWidth() * g_app.window->GetScale());
    int height =
        static_cast<int>(g_app.window->GetHeight() * g_app.window->GetScale());
    if (width > 0 && height > 0) {
      g_app.renderer->Render(g_app.model_holder->model(),
                             g_app.model_holder->data(), &g_app.perturb,
                             &g_app.camera, &g_app.vis_options, width, height,
                             /*pixels=*/{}, std::span(g_app.extra_geoms));
    }
  }

  g_app.window->Present();
}

void SetupScene(const mjModel* m) {
  g_app.renderer->Init(m);

  // Invalidate all texture caches (ImGui font atlas + NetImgui streamed
  // textures) and re-upload them on the new Filament context.
  g_app.remote_ui.UpdateTextures();

  mjv_defaultPerturb(&g_app.perturb);
  mjv_defaultCamera(&g_app.camera);
  mjv_defaultOption(&g_app.vis_options);

  const int model_cam = m->vis.global.cameraid;
  if (model_cam >= 0 && model_cam < m->ncam) {
    mujoco::studio::SetCamera(m, &g_app.camera, model_cam);
  } else {
    mjv_defaultFreeCamera(m, &g_app.camera);
  }
}

// Standard CRC-32 (ISO-HDLC, poly 0xEDB88320), matching Python's zlib.crc32
// so the fetched model's checksum can be compared against the crc the Python
// side stamps into each state payload.
uint32_t Crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1u) + 1u));
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

bool ParseModelBufferImpl(const char* data, size_t size) {
  LOG(Info, "Fetched model.mjb, size: %zu", size);
  g_app.model_holder = mujoco::studio::ModelHolder::FromBuffer(
      std::span<const std::byte>(reinterpret_cast<const std::byte*>(data),
                                 size),
      "application/mjb", "model.mjb");
  if (g_app.model_holder && g_app.model_holder->ok()) {
    g_app.session.SetModelCrc32(
        Crc32(reinterpret_cast<const uint8_t*>(data), size));
    LOG(Info, "Model parsed successfully!");
    return true;
  }
  LOG(Error, "Failed to load model: %s",
      g_app.model_holder ? g_app.model_holder->error().data()
                         : "Unknown error");
  return false;
}

void UpdateModelDownloadProgress(size_t bytes_downloaded, size_t total_bytes,
                                 int retry_count) {
  g_app.download_status.is_downloading = true;
  g_app.download_status.bytes_downloaded = bytes_downloaded;
  g_app.download_status.total_bytes = total_bytes;
  g_app.download_status.retry_count = retry_count;
}

void FinishModelLoad() {
  mj_forward(g_app.model_holder->model(), g_app.model_holder->data());
  if (g_app.window) {
    SetupScene(g_app.model_holder->model());
  }
  // Connect the state WebSocket to receive simulation state from Python.
  if (!g_app.session.HasSocket()) {
    g_app.session.Connect(WsUrl("/state"));
  }
  g_app.download_status.is_downloading = false;
  LOG(Info, "Model loaded and scene initialized successfully!");
}

// Allocates a buffer in WASM linear memory for zero-copy streaming chunk
// downloads. The caller owns the returned pointer and must free it via
// FreeModelBuffer when done.
uintptr_t AllocModelBuffer(size_t size) {
  char* ptr = new (std::nothrow) char[size];
  if (!ptr) {
    LOG(Error, "Failed to allocate WASM model buffer of size %zu", size);
    return 0;
  }
  return reinterpret_cast<uintptr_t>(ptr);
}

// Frees a buffer previously returned by AllocModelBuffer.
void FreeModelBuffer(uintptr_t ptr_val) {
  if (ptr_val) {
    delete[] reinterpret_cast<char*>(ptr_val);
  }
}

// Parses an MJB model from a buffer.
void ParseModelBuffer(uintptr_t ptr_val, size_t size) {
  if (ParseModelBufferImpl(reinterpret_cast<char*>(ptr_val), size)) {
    FinishModelLoad();
  }
}

// Holds assets (Filament assets and local ImGui fonts) that the page fetches
// and pushes in via the registerAsset() binding before startApp() runs, so the
// resource providers below can resolve them without a filesystem.
class AssetRegistry {
 public:
  static AssetRegistry& Instance() {
    static AssetRegistry instance;
    return instance;
  }

  void RegisterAsset(std::string filename, std::string contents) {
    assets_[std::move(filename)] = std::move(contents);
  }

  const std::string& Get(std::string_view filename) const {
    filename = filename.substr(filename.find_first_of(':') + 1);
    static std::string empty;
    auto it = assets_.find(std::string(filename));
    return it != assets_.end() ? it->second : empty;
  }

 private:
  std::unordered_map<std::string, std::string> assets_;
};

// Exposed to JS (see EMSCRIPTEN_BINDINGS): the page calls this once per asset.
void RegisterAsset(std::string filename, emscripten::val contents) {
  std::string data;
  if (contents.typeOf().as<std::string>() == "string") {
    data = contents.as<std::string>();
  } else if (contents.instanceof(emscripten::val::global("Uint8Array")) ||
             contents.hasOwnProperty("length")) {
    size_t len = contents["length"].as<size_t>();
    data.resize(len);
    if (len > 0) {
      emscripten::val memory_view =
          emscripten::val(emscripten::typed_memory_view(len, data.data()));
      memory_view.call<void>("set", contents);
    }
  } else if (contents.instanceof(emscripten::val::global("ArrayBuffer"))) {
    emscripten::val u8 = emscripten::val::global("Uint8Array").new_(contents);
    size_t len = u8["length"].as<size_t>();
    data.resize(len);
    if (len > 0) {
      emscripten::val memory_view =
          emscripten::val(emscripten::typed_memory_view(len, data.data()));
      memory_view.call<void>("set", u8);
    }
  }
  AssetRegistry::Instance().RegisterAsset(std::move(filename), std::move(data));
}

// Registers resource providers so that "filament:" and "font:" asset requests
// resolve from the AssetRegistry populated by registerAsset().
static void RegisterAssetProviders() {
  mjpResourceProvider resource_provider;
  mjp_defaultResourceProvider(&resource_provider);

  resource_provider.open = [](mjResource* resource) {
    AssetRegistry& r = AssetRegistry::Instance();
    return static_cast<int>(r.Get(resource->name).size());
  };
  resource_provider.read = [](mjResource* resource, const void** buffer) {
    AssetRegistry& r = AssetRegistry::Instance();
    const std::string& contents = r.Get(resource->name);
    *buffer = contents.data();
    return static_cast<int>(contents.size());
  };
  resource_provider.close = [](mjResource* resource) {};
  for (const char* prefix : {"filament", "font"}) {
    resource_provider.prefix = prefix;
    mjp_registerResourceProvider(&resource_provider);
  }
}

// Starts the viewer once the page has registered every asset. Exposed to JS
// and called from index.html after the fetches complete.
void StartApp() {
  mujoco::studio::Window::Config config;
  config.gfx_mode = mujoco::studio::GraphicsMode::FilamentWebGl;
  // Load the Studio UI fonts for the browser's local ImGui (the role window);
  // the "font:" resource provider above resolves them from the AssetRegistry.
  // (The streamed/remote UI carries its own font atlas separately.)
  config.load_fonts = true;

  g_app.window = std::make_unique<mujoco::studio::Window>("MuJoCo Web Viewer",
                                                            1400, 720, config);
  ImPlot::CreateContext();  // Needed if the server app uses ImPlot.

  g_app.renderer = new mujoco::studio::FilamentRenderer(
      g_app.window->GetNativeWindowHandle(), config.gfx_mode);

  // Initialize an empty dummy scene so Filament and ImGui are ready to render
  // the "DOWNLOADING..." progress bar while /model downloads asynchronously
  g_app.model_holder = mujoco::studio::ModelHolder::FromSpec(mj_makeSpec());
  if (g_app.model_holder && g_app.model_holder->ok()) {
    SetupScene(g_app.model_holder->model());
  }

  NetImgui::Internal::Network::Startup();
  SDL_StartTextInput();

  // The UI stream is a path on the page's own host and port.
  g_app.remote_ui.Connect(WsUrl("/ui"));

  emscripten_set_main_loop(MainLoop, 0, 0);
}

int main(int argc, char** argv) {
  RegisterAssetProviders();
  return 0;
}

EMSCRIPTEN_BINDINGS(web_client_bindings) {
  // Registers staged static files fetched by index.html into the C++ in-memory
  // AssetRegistry before startApp() runs.
  emscripten::function("registerAsset", &RegisterAsset);

  // Allocates a buffer in WASM linear memory for zero-copy streaming model
  // chunk downloads. The caller must free it via freeModelBuffer when done.
  emscripten::function("allocModelBuffer", &AllocModelBuffer);

  // Frees a buffer previously allocated by allocModelBuffer.
  emscripten::function("freeModelBuffer", &FreeModelBuffer);

  // Parses a completed MJB model buffer and reinitializes the Filament scene.
  emscripten::function("parseModelBuffer", &ParseModelBuffer);

  // Initializes the Filament window, ImGui context, WebSocket connections, and
  // starts the Emscripten main simulation loop.
  emscripten::function("startApp", &StartApp);

  // Updates the model download progress in C++ so the ImGui role window can
  // display a real-time progress bar and status while downloading.
  emscripten::function("updateModelDownloadProgress",
                       &UpdateModelDownloadProgress);
}
