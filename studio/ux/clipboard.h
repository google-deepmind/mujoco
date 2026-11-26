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

// Copy and paste between Studio's UI and the system clipboard in the browser builds of Studio
// (MuJoCo Live and the Web Viewer). There, ImGui's clipboard is only a buffer inside the page,
// and a page can reach the system clipboard only in limited ways:
// - Copy: text copied in the UI is also written to the browser's clipboard; see
//   SyncClipboardWithBrowser().
// - Paste: a page can only read the system clipboard from the browser's "paste" event, by which
//   time the real Ctrl+V key press is over (or was never seen at all). The page hands the event's
//   text over with SetClipboardTextFromBrowser(), and the paste shortcut is then synthesized; see
//   QueuePasteShortcut().
//
// Synthesizing the paste shortcut
//
// ImGui has no call that makes a text field paste: a field pastes only when it sees Ctrl+V being
// pressed. PressPasteShortcut() queues those key presses and ReleasePasteShortcut() ends them. The
// field only registers the shortcut if V is still down when a frame runs, so the release must take
// effect a frame after the press. Either call it on the next frame, or queue both together as
// QueuePasteShortcut() does: ImGui's input queue (io.ConfigInputTrickleEventQueue, on by default)
// then plays them back on consecutive frames.
//
// Manual test checklist
//
// The unit tests cannot reach the browser side (the page's key and paste listeners and the
// browser's clipboard), so after changing clipboard handling, run the steps below by hand in
// MuJoCo Live and in the Web Viewer. The steps name kinds of widget rather than specific ones,
// because the UI changes; each example is one that fits at the time of writing, in both apps
// unless it says otherwise:
// - "outside": another browser tab or application.
// - "InputText": an editable text field, e.g. the one labelled Field in Inspector > Watch.
// - "InputInt": a number field, e.g. the one labelled Index in Inspector > Watch.
// - "copy button": a button that copies text, e.g. the one with the Copy Camera tooltip.
// - "browser InputInt": Web Viewer only. An "InputInt" drawn by the page itself rather than
//   streamed from Python, e.g. the spectator limit in the controller's role bar.
//
// 1. Copy from "outside", then paste into an "InputText".
// 2. Copy from an "InputText", then paste "outside".
// 3. Copy from an "InputInt", then paste into an "InputText" (copy and paste within the app).
// 4. Press a "copy button", then paste "outside".
// 5. Press a "copy button", then paste into an "InputText".
// 6. With no field focused, plain V still runs its own shortcut (toggling tendons at the time of
//    writing).
// 7. Web Viewer only, browser-side copy and paste: repeat 1 to 3 using a "browser InputInt".
//
// Only the Ctrl+V / Cmd+V shortcut is supported for pasting from "outside". Shift+Insert and the
// browser's Edit > Paste menu are not.

#ifndef MUJOCO_STUDIO_UX_CLIPBOARD_H_
#define MUJOCO_STUDIO_UX_CLIPBOARD_H_

#include <string>

#include <imgui.h>

namespace mujoco::studio {

// Saves `contents` to the clipboard if the clipboard is available.
void MaybeSaveToClipboard(const std::string& contents);

// Configures the current ImGui context to forward copied text to the browser's clipboard.
// Call once after initializing the platform backend. Has no effect outside browser builds.
void SyncClipboardWithBrowser();

// Sets ImGui's clipboard to `text` without invoking the copy callback or writing it back to the
// browser's clipboard.
void SetClipboardTextFromBrowser(const std::string& text);

// Callback invoked with text copied in the UI.
using ClipboardCopyFn = void (*)(const char* text);

// Registers `on_copy` to be called whenever text is copied in the current ImGui context, replacing
// any previous callback. Pass nullptr to clear the callback. SyncClipboardWithBrowser() is built on
// this; it is exposed separately so copying can be tested without a browser.
void SetClipboardCopyCallback(ClipboardCopyFn on_copy);

// Queues key events on `io` to press the paste shortcut, causing the focused text field to paste
// the clipboard text. Callers must follow up with ReleasePasteShortcut(); see "Synthesizing the
// paste shortcut" above.
void PressPasteShortcut(ImGuiIO& io);

// Physical state of the paste-shortcut keys currently held by the user, passed to
// ReleasePasteShortcut(). `ctrl` and `super` represent the physical keys before ImGui swaps
// them for `ImGuiIO::ConfigMacOSXBehaviors`.
struct HeldPasteKeys {
  bool v = false;
  bool ctrl = false;
  bool shift = false;
  bool alt = false;
  bool super = false;
};

// Queues key events on `io` to end PressPasteShortcut(), restoring V and the modifier keys to the
// states in `held`.
void ReleasePasteShortcut(ImGuiIO& io, const HeldPasteKeys& held);

// Queues key events on `io` to press and release the paste shortcut while preserving any modifier
// keys currently held by the user.
void QueuePasteShortcut(ImGuiIO& io);

}  // namespace mujoco::studio

#endif  // MUJOCO_STUDIO_UX_CLIPBOARD_H_
