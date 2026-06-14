// Copyright 2025 DeepMind Technologies Limited
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

#include "experimental/studio/llm/test_runner.h"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>

#include "imgui_test_engine/imgui_te_context.h"
#include "imgui_test_engine/imgui_te_engine.h"

namespace mujoco::studio {
namespace {

// --- Tiny JSON readers (the op-program is a constrained shape we control). ---

size_t SkipJsonValue(const std::string& s, size_t pos) {
  if (pos >= s.size() || (s[pos] != '{' && s[pos] != '[')) {
    return std::string::npos;
  }
  const char open = s[pos];
  const char close = (open == '{') ? '}' : ']';
  int depth = 0;
  bool in_str = false;
  for (size_t i = pos; i < s.size(); ++i) {
    char c = s[i];
    if (in_str) {
      if (c == '\\') { ++i; continue; }
      if (c == '"') in_str = false;
    } else if (c == '"') {
      in_str = true;
    } else if (c == open) {
      ++depth;
    } else if (c == close) {
      if (--depth == 0) return i + 1;
    }
  }
  return std::string::npos;
}

std::string ParseJsonString(const std::string& s, size_t quote_pos) {
  std::string out;
  if (quote_pos >= s.size() || s[quote_pos] != '"') return out;
  for (size_t i = quote_pos + 1; i < s.size(); ++i) {
    char c = s[i];
    if (c == '"') break;
    if (c == '\\' && i + 1 < s.size()) {
      char e = s[++i];
      switch (e) {
        case 'n': out += '\n'; break;
        case 't': out += '\t'; break;
        case 'r': out += '\r'; break;
        default: out += e; break;
      }
    } else {
      out += c;
    }
  }
  return out;
}

// Reads a string field `"<key>"` from a JSON object substring.
std::string ReadString(const std::string& obj, const char* key) {
  size_t k = obj.find(key);
  if (k == std::string::npos) return "";
  size_t colon = obj.find(':', k + std::char_traits<char>::length(key));
  if (colon == std::string::npos) return "";
  size_t q = obj.find('"', colon + 1);
  if (q == std::string::npos) return "";
  return ParseJsonString(obj, q);
}

// Reads a numeric field `"<key>"` from a JSON object substring (0 if absent).
double ReadNumber(const std::string& obj, const char* key) {
  size_t k = obj.find(key);
  if (k == std::string::npos) return 0.0;
  size_t colon = obj.find(':', k + std::char_traits<char>::length(key));
  if (colon == std::string::npos) return 0.0;
  return std::strtod(obj.c_str() + colon + 1, nullptr);
}

// Returns the "ops" array substring (with brackets), or the first array found.
std::string ExtractOpsArray(const std::string& json_args) {
  size_t k = json_args.find("\"ops\"");
  size_t b = json_args.find('[', k == std::string::npos ? 0 : k);
  if (b == std::string::npos) return "";
  size_t e = SkipJsonValue(json_args, b);
  if (e == std::string::npos) return "";
  return json_args.substr(b, e - b);
}

// Maps a small set of key names to ImGuiKey (enough for the demo: typing into
// the palette and pressing Enter).
ImGuiKey KeyFromName(const std::string& n) {
  if (n == "Enter") return ImGuiKey_Enter;
  if (n == "Tab") return ImGuiKey_Tab;
  if (n == "Escape" || n == "Esc") return ImGuiKey_Escape;
  if (n == "Space") return ImGuiKey_Space;
  if (n == "Backspace") return ImGuiKey_Backspace;
  if (n.size() == 1 && n[0] >= 'A' && n[0] <= 'Z') {
    return static_cast<ImGuiKey>(ImGuiKey_A + (n[0] - 'A'));
  }
  return ImGuiKey_None;
}

// Calls `fn` with each top-level object substring inside an array.
template <typename Fn>
void ForEachObject(const std::string& arr, Fn fn) {
  size_t i = arr.find('[');
  if (i == std::string::npos) return;
  ++i;
  while (i < arr.size()) {
    while (i < arr.size() && (arr[i] == ' ' || arr[i] == ',' || arr[i] == '\n' ||
                             arr[i] == '\t' || arr[i] == '\r')) {
      ++i;
    }
    if (i >= arr.size() || arr[i] != '{') break;
    size_t e = SkipJsonValue(arr, i);
    if (e == std::string::npos) break;
    fn(arr.substr(i, e - i));
    i = e;
  }
}

}  // namespace

// Only Stop() here (engine leaks at process exit): ImGuiTestEngine_DestroyContext
// must run after ImGui::DestroyContext(), but the window never destroys the
// ImGui context, so destroying the engine would assert. Leaking at exit is fine.
TestRunner::~TestRunner() { Stop(); }

void TestRunner::Start() {
  if (engine_) return;
  engine_ = ImGuiTestEngine_CreateContext();
  ImGuiTestEngineIO& io = ImGuiTestEngine_GetIO(engine_);
  io.ConfigRunSpeed = ImGuiTestRunSpeed_Fast;
  io.ConfigVerboseLevel = ImGuiTestVerboseLevel_Warning;
  io.ConfigVerboseLevelOnError = ImGuiTestVerboseLevel_Info;
  io.ConfigRestoreFocusAfterTests = false;
  ImGuiTestEngine_Start(engine_, ImGui::GetCurrentContext());
  ImGuiTestEngine_InstallDefaultCrashHandler();

  test_ = ImGuiTestEngine_RegisterTest(engine_, "studio", "run_program",
                                       __FILE__, __LINE__);
  test_->UserData = this;
  test_->TestFunc = &TestRunner::TestFuncThunk;
  running_ = true;
}

void TestRunner::Stop() {
  if (engine_ && running_) {
    ImGuiTestEngine_Stop(engine_);
    running_ = false;
  }
}

void TestRunner::Destroy() {
  if (!engine_) return;
  ImGuiTestEngine_DestroyContext(engine_);
  engine_ = nullptr;
  test_ = nullptr;
}

void TestRunner::PostSwap() {
  if (!engine_ || !running_) return;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!jobs_.empty() && ImGuiTestEngine_IsTestQueueEmpty(engine_)) {
      running_job_ = std::move(jobs_.front());
      jobs_.erase(jobs_.begin());
      ImGuiTestEngine_QueueTest(engine_, test_, ImGuiTestRunFlags_None);
    }
  }
  // Draw the simulated cursor while a program is running so the action is
  // visible (e.g. in captured GIFs).
  ImGui::GetIO().MouseDrawCursor = !ImGuiTestEngine_IsTestQueueEmpty(engine_);
  ImGuiTestEngine_PostSwap(engine_);
}

bool TestRunner::idle() {
  if (!engine_ || !running_) return true;
  std::lock_guard<std::mutex> lk(mu_);
  return jobs_.empty() && ImGuiTestEngine_IsTestQueueEmpty(engine_);
}

int TestRunner::Run(const std::string& json_args) {
  std::string ops = ExtractOpsArray(json_args);
  if (ops.empty()) return 0;
  int count = 0;
  ForEachObject(ops, [&](const std::string&) { ++count; });
  {
    std::lock_guard<std::mutex> lk(mu_);
    jobs_.push_back(Job{/*gather=*/false, std::move(ops), nullptr});
  }
  return count;
}

std::string TestRunner::Inspect() {
  if (!engine_ || !running_) return "(test engine not running)";
  auto result = std::make_shared<GatherResult>();
  {
    std::lock_guard<std::mutex> lk(mu_);
    jobs_.push_back(Job{/*gather=*/true, std::string(), result});
  }
  std::unique_lock<std::mutex> lk(result->mu);
  if (!result->cv.wait_for(lk, std::chrono::seconds(20),
                           [&] { return result->done; })) {
    return "(inspect timed out)";
  }
  return result->text;
}

void TestRunner::TestFuncThunk(ImGuiTestContext* ctx) {
  auto* self = static_cast<TestRunner*>(ctx->Test->UserData);
  // running_job_ was set under the lock in PostSwap before the test was queued;
  // by the time the test runs no other job can start, so reading it here (on the
  // UI thread) is safe.
  if (self->running_job_.gather) {
    self->DoGather(ctx, self->running_job_.result);
  } else {
    self->Execute(ctx, self->running_job_.payload);
  }
}

void TestRunner::Execute(ImGuiTestContext* ctx, const std::string& ops_json) {
  ForEachObject(ops_json, [&](const std::string& obj) {
    const std::string op = ReadString(obj, "\"op\"");
    const std::string ref = ReadString(obj, "\"ref\"");
    const std::string path = ReadString(obj, "\"path\"");
    const std::string text = ReadString(obj, "\"text\"");
    // An item op may carry an "id" (exact ImGuiID from inspect_ui) instead of a
    // ref; the id always wins. This makes both {op:click_id,id:N} and
    // {op:item_click,id:N} work, and is truncation/clip-proof.
    const ImGuiID id = static_cast<ImGuiID>(ReadNumber(obj, "\"id\""));
    const ImGuiTestRef tref =
        (id != 0) ? ImGuiTestRef(id) : ImGuiTestRef(ref.c_str());

    if (op == "item_click" || op == "click_id") {
      ctx->ItemClick(tref);
    } else if (op == "menu_click") {
      ctx->MenuClick((path.empty() ? ref : path).c_str());
    } else if (op == "item_check") {
      ctx->ItemCheck(tref);
    } else if (op == "item_uncheck") {
      ctx->ItemUncheck(tref);
    } else if (op == "set_float" || op == "set_float_id") {
      ctx->ItemInputValue(tref,
                          static_cast<float>(ReadNumber(obj, "\"value\"")));
    } else if (op == "set_int") {
      ctx->ItemInputValue(tref, static_cast<int>(ReadNumber(obj, "\"value\"")));
    } else if (op == "key_chars") {
      ctx->KeyChars(text.c_str());
    } else if (op == "key_press") {
      ImGuiKey key = KeyFromName(ReadString(obj, "\"key\""));
      if (key != ImGuiKey_None) ctx->KeyPress(key);
    } else if (op == "set_ref") {
      ctx->SetRef(tref);
    }
    // Unknown ops are ignored (forward-compatible).
  });
}

void TestRunner::DoGather(ImGuiTestContext* ctx,
                          const std::shared_ptr<GatherResult>& out) {
  // Snapshot the currently-active top-level windows up front (by id+name), so
  // gathering -- which yields across frames -- can't trip over the window list
  // changing under us.
  struct WinRef {
    ImGuiID id;
    std::string name;
  };
  std::vector<WinRef> wins;
  ImGuiContext& g = *ImGui::GetCurrentContext();
  for (ImGuiWindow* w : g.Windows) {
    if (w == nullptr || !w->WasActive || w->Hidden) continue;
    if (w->Flags & ImGuiWindowFlags_ChildWindow) continue;
    std::string name = w->Name ? w->Name : "";
    const size_t h = name.find("##");  // strip the "##"/"###" id suffix
    if (h != std::string::npos) name = name.substr(0, h);
    if (name.empty()) continue;
    wins.push_back({w->ID, name});
  }

  std::string text;
  int total = 0;
  for (const WinRef& wr : wins) {
    ImGuiTestItemList items;
    ctx->GatherItems(&items, ImGuiTestRef(wr.id), 99);
    std::string lines;
    int n = 0;
    for (int i = 0; i < items.GetSize() && total < 250; ++i) {
      const ImGuiTestItemInfo* it = items.GetByIndex(i);
      if (it == nullptr || it->DebugLabel[0] == '\0') continue;
      // Show the display name (strip any "###id" suffix, which may be truncated
      // and is confusing) for recognition only -- the id is the exact address.
      std::string label = it->DebugLabel;
      const size_t h = label.find("###");
      if (h != std::string::npos) label = label.substr(0, h);
      lines += std::string("  ") + label + "  [id=" + std::to_string(it->ID) +
               "]\n";
      ++total;
      if (++n >= 40) {
        lines += "  ...\n";
        break;
      }
    }
    if (!lines.empty()) text += "[" + wr.name + "]\n" + lines;
    if (total >= 250) break;
  }
  if (text.empty()) text = "(no visible items found)";

  if (out) {
    std::lock_guard<std::mutex> lk(out->mu);
    out->text = std::move(text);
    out->done = true;
    out->cv.notify_all();
  }
}

}  // namespace mujoco::studio
