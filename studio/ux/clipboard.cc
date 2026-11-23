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

void MaybeSaveToClipboard(const std::string& contents) {
  // ImGui's platform clipboard, which in-app paste reads. In the browser builds
  // this is only SDL's internal buffer, since SDL's Emscripten port has no
  // clipboard, so also write the browser's clipboard there.
  ImGui::SetClipboardText(contents.c_str());
#if defined(__EMSCRIPTEN__)
  WriteBrowserClipboard(contents.c_str());
#endif
}

}  // namespace mujoco::studio
