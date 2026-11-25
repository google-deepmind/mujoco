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

#include "studio/ux/clipboard.h"

#include <string>

#include <imgui.h>

#if defined(__EMSCRIPTEN__)
  #include <emscripten/em_js.h>

// Writes text to the browser's clipboard, which only JavaScript can reach.
// clang-format off
EM_JS(void, WriteBrowserClipboard, (const char* text), {
  const str = UTF8ToString(text);
  // Selects the text in an off-screen textarea and copies the selection, giving
  // keyboard focus back to whatever had it, normally the canvas.
  const copyWithExecCommand = () => {
    if (typeof document === "undefined" || !document.body) {
      return false;
    }
    const focused = document.activeElement;
    const textarea = document.createElement("textarea");
    textarea.value = str;
    textarea.setAttribute("readonly", "");
    textarea.style.position = "fixed";
    textarea.style.left = "-9999px";
    document.body.appendChild(textarea);
    textarea.focus();
    textarea.select();
    let copied = false;
    try {
      copied = document.execCommand("copy");
    } catch (error) {
      copied = false;
    }
    document.body.removeChild(textarea);
    if (focused && focused.focus) {
      focused.focus();
    }
    return copied;
  };
  // Prefer the asynchronous Clipboard API, which browsers only expose in secure
  // contexts (https or localhost) and with a recent user gesture.
  if (typeof navigator !== "undefined" && navigator.clipboard &&
      navigator.clipboard.writeText) {
    navigator.clipboard.writeText(str).catch((error) => {
      // Fall back to execCommand("copy") if writeText refuses, e.g. because the
      // document is not focused or the user gesture expired.
      if (!copyWithExecCommand()) {
        console.warn("Copy to clipboard failed:", error);
      }
    });
    return;
  }
  // Fall back to the deprecated execCommand("copy") when the Clipboard API is
  // missing, e.g. the WebViewer served over plain http (still needs a recent
  // user gesture).
  if (!copyWithExecCommand()) {
    console.warn("Copy to clipboard failed: the Clipboard API needs https or " +
                 "localhost, and execCommand('copy') was refused.");
  }
});
// clang-format on
#endif  // defined(__EMSCRIPTEN__)

namespace mujoco::studio {

namespace {

// State of SetClipboardCopyCallback(). The browser builds have one Dear ImGui context, so one
// global instance is enough.
struct ClipboardSync {
  // Called with each text copied in the UI, or null if no callback is set.
  ClipboardCopyFn on_copy = nullptr;
  // The platform's own handler, which SetClipboardTextAndNotify() replaced and still calls.
  void (*platform_set_text)(ImGuiContext* ctx, const char* text) = nullptr;
  // True inside SetClipboardTextFromBrowser(), whose text must not be written back.
  bool text_from_browser = false;
};
ClipboardSync g_clipboard_sync;

// Dear ImGui's Platform_SetClipboardTextFn once SetClipboardCopyCallback() was called. Stores the
// text with the platform's own handler, then passes it to the copy callback unless it came from the
// browser.
void SetClipboardTextAndNotify(ImGuiContext* ctx, const char* text) {
  if (g_clipboard_sync.platform_set_text) {
    g_clipboard_sync.platform_set_text(ctx, text);
  }
  if (g_clipboard_sync.on_copy && !g_clipboard_sync.text_from_browser) {
    g_clipboard_sync.on_copy(text);
  }
}

}  // namespace

void SetClipboardCopyCallback(ClipboardCopyFn on_copy) {
  g_clipboard_sync.on_copy = on_copy;
  ImGuiPlatformIO& platform_io = ImGui::GetPlatformIO();
  if (platform_io.Platform_SetClipboardTextFn != &SetClipboardTextAndNotify) {
    g_clipboard_sync.platform_set_text = platform_io.Platform_SetClipboardTextFn;
    platform_io.Platform_SetClipboardTextFn = &SetClipboardTextAndNotify;
  }
}

void SyncClipboardWithBrowser() {
#if defined(__EMSCRIPTEN__)
  // In the browser builds the platform clipboard is only SDL's internal buffer, since SDL's
  // Emscripten port has no clipboard. The other builds already use the system clipboard.
  SetClipboardCopyCallback(&WriteBrowserClipboard);
#endif
}

void SetClipboardTextFromBrowser(const std::string& text) {
  // The same as ImGui::SetClipboardText() when no copy callback is set.
  g_clipboard_sync.text_from_browser = true;
  ImGui::SetClipboardText(text.c_str());
  g_clipboard_sync.text_from_browser = false;
}

void MaybeSaveToClipboard(const std::string& contents) {
  // ImGui's platform clipboard, which in-app paste reads. In the browser builds this also writes
  // the browser's clipboard once SyncClipboardWithBrowser() was called; do it here otherwise.
  ImGui::SetClipboardText(contents.c_str());
#if defined(__EMSCRIPTEN__)
  if (g_clipboard_sync.on_copy == nullptr) {
    WriteBrowserClipboard(contents.c_str());
  }
#endif
}

void PressPasteShortcut(ImGuiIO& io) {
  // AddKeyEvent() swaps Super and Ctrl when ConfigMacOSXBehaviors is set, so pass Super there to
  // produce ImGuiMod_Ctrl for InputText's paste chord.
  const ImGuiKey paste_mod = io.ConfigMacOSXBehaviors ? ImGuiMod_Super : ImGuiMod_Ctrl;
  const ImGuiKey other_mod = io.ConfigMacOSXBehaviors ? ImGuiMod_Ctrl : ImGuiMod_Super;
  // Lift the other modifiers first, because the shortcut only matches exactly Ctrl+V (a
  // Shift+Insert paste would otherwise read as Ctrl+Shift+V).
  io.AddKeyEvent(other_mod, false);
  io.AddKeyEvent(ImGuiMod_Shift, false);
  io.AddKeyEvent(ImGuiMod_Alt, false);
  io.AddKeyEvent(paste_mod, true);
  io.AddKeyEvent(ImGuiKey_V, false);
  io.AddKeyEvent(ImGuiKey_V, true);
}

void ReleasePasteShortcut(ImGuiIO& io, const HeldPasteKeys& held) {
  // V and the modifiers go back to `held` rather than simply being lifted, so a key the user is
  // still holding does not look released until they press it again.
  io.AddKeyEvent(ImGuiKey_V, held.v);
  io.AddKeyEvent(ImGuiMod_Ctrl, held.ctrl);
  io.AddKeyEvent(ImGuiMod_Shift, held.shift);
  io.AddKeyEvent(ImGuiMod_Alt, held.alt);
  io.AddKeyEvent(ImGuiMod_Super, held.super);
}

void QueuePasteShortcut(ImGuiIO& io) {
  // What the user is holding. V is taken to be up: the page stops the real key press before it
  // reaches Dear ImGui. io.KeyCtrl and io.KeySuper are read after Dear ImGui swapped them for
  // ConfigMacOSXBehaviors, so swap them back to the physical keys.
  const bool macos = io.ConfigMacOSXBehaviors;
  HeldPasteKeys held;
  held.ctrl = macos ? io.KeySuper : io.KeyCtrl;
  held.super = macos ? io.KeyCtrl : io.KeySuper;
  held.shift = io.KeyShift;
  held.alt = io.KeyAlt;
  PressPasteShortcut(io);
  ReleasePasteShortcut(io, held);
}

}  // namespace mujoco::studio
