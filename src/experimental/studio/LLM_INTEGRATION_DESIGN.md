# LLM Integration Design — MuJoCo Studio

Status: **design only, no code yet.** This is the plan we agreed to write before
implementing. It covers three things:

1. Routing plain-English text typed in the command prompt to an LLM.
2. A way for the LLM to *do* things in the model/data "as if we used the buttons,"
   designed so a future Python layer can register the available actions.
3. Asking the LLM to *show* how to do things in the UI, driven by the ImGui Test
   Engine API.

The key idea that ties (2) and (3) together — and the reason this is worth
building — is this:

> **The LLM can read the Studio source code to learn what the UI does, and then
> control the UI through the ImGui Test Engine API.**

That single capability subsumes a lot of bespoke plumbing. We don't have to
hand-describe every control to the model; the code is the source of truth, and
the Test Engine is a universal actuator that can press any button, type into any
field, and drive any window a human could. The structured action registry (2)
becomes the *fast, safe, deterministic* path for common edits, while
source-reading + Test Engine (3) is the *general* path for everything else.

---

## 1. Where this plugs into the existing app

We already have the seams we need:

- **Command palette** (`command_palette.{h,cc}`). `CommandPalette` is a
  self-contained `{name, std::function run}` widget. Today, typing `>` enters
  command mode and filters the rail-button commands collected by
  `App::CollectCommands()`. Input that does **not** start with `>` currently does
  nothing — that is exactly the channel we route to the LLM.
- **Tool-window registry** (`App::tool_windows_`, `App::RegisterToolWindows()`).
  A `std::vector<ToolWindow{icon, title, render, open}>`. Each entry is already
  addressable by `title` via `ToggleToolWindowByName()`. This is the natural
  place to attach machine-readable metadata.
- **Recorded UI geometry** (`rail_button_center_`, `tool_window_rect_`) and the
  **stable window IDs** (`###ToolWindowN`). These were added for the capture
  script, but they are precisely the anchors a UI-automation layer needs.
- **Headless render + framebuffer readback** (`pixels_`, used by
  `SaveCaptureFrame`). This already lets us run the UI without a visible window
  and grab frames — the basis for "show me" playback and for letting the LLM
  *see* the result of an action if we ever want vision-in-the-loop.

So the command prompt becomes a dispatcher:

```
prompt text
  ├── starts with ">"  → existing command-palette mode (unchanged)
  └── otherwise        → LLM turn (new)
```

---

## 2. New files (per the "separate files" rule)

Nothing below bloats `app.cc`. Each concern gets its own translation unit so we
can split work later.

| File | Responsibility |
|---|---|
| `llm/llm_provider.h` | Provider-agnostic interface: send a turn, stream text, surface tool calls, return tool results. No Anthropic/Gemini types leak out. |
| `llm/llm_claude.{h,cc}` | Claude implementation. Default model `claude-opus-4-8`, adaptive thinking, streaming. (C++ uses raw HTTPS to `/v1/messages`; see §6.) |
| `llm/llm_gemini.{h,cc}` | Gemini implementation (function calling). Optional, behind the same interface. |
| `llm/action_registry.{h,cc}` | The `{name, description, json_schema, fn}` registry of "as-if-the-buttons" edits, plus a C ABI so Python can register actions. |
| `llm/ui_agent.{h,cc}` | Orchestrates a chat turn: builds the system prompt, runs the tool-use loop, calls into `action_registry` and/or `test_runner`. Owns the conversation/history. |
| `llm/test_runner.{h,cc}` | Wraps ImGui Test Engine: register a generated test, queue it, run it (optionally headless), record a GIF via the existing capture path. Compiled only when `IMGUI_ENABLE_TEST_ENGINE` is on. |
| `llm/source_index.{h,cc}` | Optional helper that gives the model curated access to the Studio source (see §5). Can start as a baked-in manifest, grow into on-demand file reads. |
| `llm/llm_panel.{h,cc}` | The chat/conversation UI (history, streaming tokens, "Run"/"Show me" affordances). The command prompt stays the entry point; this is where longer exchanges render. |

`app.cc` gains only thin wiring: construct a `UiAgent`, hand it the registry +
test runner, and forward non-`>` prompt submissions to it.

---

## 3. Command-prompt routing (feature 1)

`CommandPalette::Draw()` already returns control to the app when the user presses
Enter. We add an `on_submit_plain(std::string)` callback (or a return enum) so
that:

- `>`-prefixed input runs a `Command` exactly as today.
- plain input is forwarded to `UiAgent::Ask(text)`.

`UiAgent::Ask` runs **asynchronously** on the existing threadpool (we already
have an `nthread` Preferences setting) so the render loop never blocks on the
network. Streaming deltas are pushed into `llm_panel`'s history buffer; the UI
thread just draws whatever is there. This matches the skill's guidance to stream
any turn that may be long.

A turn ends when the model stops requesting tools. Each tool call is dispatched
to either the action registry (§4) or the test runner (§5), and the result is
fed back into the loop.

---

## 4. Action registry — the "as if we used the buttons" path (feature 2)

A registry of small, well-described, schema-typed actions. This is the
deterministic, low-risk path: no UI scripting, just a direct call that does the
same thing a button does.

```cpp
struct Action {
  std::string name;                       // "set_gravity"
  std::string description;                 // for the model
  std::string json_schema;                 // JSON-Schema for the args object
  std::function<std::string(const Json&)> run;  // returns a short result string
};
```

Seed actions (all things the UI already does — we route them through the *same*
code the buttons call, not a parallel implementation):

- `open_window` / `close_window` / `toggle_window {title}` → `ToggleToolWindowByName`.
- `reload_model`, `reset`, `set_paused {bool}`, `step {n}`, `set_speed {index}`.
- `set_gravity {x,y,z}`, `set_flag {name,bool}` (visualization / rendering flags
  in `vis_options_`), `set_camera {name}`, `set_key {index}`.
- `set_joint {name, value}`, `set_ctrl {name, value}` (data edits, guarded by
  `has_data()`).
- `get_state {what}` — read-only introspection so the model can answer questions
  ("how many contacts are there?") without scripting the UI.

Each action maps to a Claude **tool** (name + description + `input_schema`) or a
Gemini **function declaration** — the registry serializes to either. With the
Anthropic Python SDK this is the `@beta_tool` + `tool_runner` shape; in our C++
host we send the same JSON tool definitions over `/v1/messages` and run the loop
ourselves (§6).

### Why a registry *and* the Test Engine path?

- The registry is **fast, safe, and testable**: bounded surface, validated args,
  no dependence on widget labels or layout. Use it for the common verbs.
- It is also **the thing Python will own** (below), so it's worth having as a
  first-class object even though Test Engine could technically do the same edits.

### Python registration (the forward-looking requirement)

The user's constraint: assume the whole app is Python later, and Python should be
able to *provide* these action calls. So the registry is designed to be
populated from outside C++:

- Expose a tiny **C ABI**: `studio_register_action(name, description,
  json_schema, callback, user_data)`. Python (via `ctypes`/`pybind11`) registers
  a callback that receives the JSON args and returns a result string.
- The C++ seed actions register through the *same* entry point, so there is no
  "C++ is special" path — when the app becomes Python, the C++ seeds simply go
  away and Python registers the equivalents. The `UiAgent` neither knows nor
  cares who registered an action.
- `tool_windows_` already demonstrates the pattern (data-driven list of
  callbacks); the action registry is the same idea with metadata attached.

This means the LLM tool list is **discovered at runtime** from whatever is
registered — no recompile to add a capability.

---

## 5. Source-reading + Test Engine — the "show me / drive the UI" path (feature 3)

This is the part the user got excited about, and the more powerful mechanism.
Instead of (or in addition to) calling a registered action, the LLM **reads the
source to understand the UI, then writes an ImGui Test Engine script** that
drives the real widgets. Two uses:

1. **"Show me how to do X."** The model produces a Test Engine script; we run it
   (optionally headless) and record a GIF via the existing capture pipeline, so
   the user sees the exact clicks. This is a demonstration, not just prose.
2. **"Do X"** for things with no registered action. The same script, run once,
   actuates the UI directly — the universal fallback.

### Why source-reading makes this tractable

ImGui Test Engine addresses items by path/label, e.g.:

```cpp
ctx->SetRef("Physics");                 // a tool window (stable ###ToolWindowN id)
ctx->ItemClick("//Tools/Physics");      // a rail button
ctx->ItemInputValue("Gravity/$$2", -9.81f);
ctx->KeyChars("qpos");
ctx->MenuClick("Edit/Preferences");
```

The hard part is normally *knowing the labels and paths*. But those strings live
in the source — `MainMenuGui()`, `RegisterToolWindows()`, each `render` lambda.
By letting the model read that source, it can derive the correct refs itself
rather than us maintaining a brittle hand-written map. The code is the
documentation. When the UI changes, the model re-reads and adapts; there's no
second artifact to keep in sync.

We make this reliable, not magical, with three supports:

- **`source_index`** curates *what* the model may read: the Studio UI `.cc/.h`
  files (`app.cc`, `command_palette.cc`, the `render` lambdas), the FontAwesome
  label constants, and the `###ToolWindowN` ID scheme. Start with a baked-in
  manifest (paths + a short map of "window → ImGui ID"); optionally grow into an
  on-demand "read this file/range" tool so the model pulls only what it needs.
  Either way the model is grounded in the real strings, not guessing.
- **Recorded geometry as ground truth.** `rail_button_center_` and
  `tool_window_rect_` already capture where things actually are at runtime. The
  test runner (or a validation pass) can confirm a generated ref resolves before
  we run it, and report back to the model if it doesn't — a self-correcting loop.
- **The Test Engine's own item lookup.** Test Engine can enumerate items and
  fail loudly when a ref is wrong. A failed step becomes a tool-result error the
  model fixes on the next iteration, the same way a coding agent fixes a failing
  test.

### The generated-script contract

We do **not** let the model emit arbitrary C++ to compile. Instead it emits a
small **JSON test program** — a list of typed steps — that `test_runner`
interprets against a fixed vocabulary:

```json
[
  {"op": "menu_click", "path": "View/Tools"},
  {"op": "item_click", "ref": "//Tools/Physics"},
  {"op": "set_float",  "ref": "Physics/Gravity Z", "value": -3.0},
  {"op": "screenshot", "label": "after"}
]
```

Benefits:

- **Safe** — the op vocabulary is closed; no code execution, no recompile.
- **Replayable** — the same JSON drives a live demo or a headless GIF.
- **Inspectable** — we can show the user the steps before running, and diff them.
- **Mappable to Test Engine** — each op is one `ImGuiTestContext` call.

`test_runner` registers a single generic `ImGuiTest` whose `TestFunc`
interprets the current JSON program; we don't compile a new test per request.

### Build prerequisite (called out honestly)

Test Engine requires building Dear ImGui with `IMGUI_ENABLE_TEST_ENGINE` and
linking `imgui_test_engine` (+ its capture tool for GIFs). That's a real build
change, so it lives behind a CMake option (e.g. `MUJOCO_STUDIO_TEST_ENGINE`,
default OFF) and `test_runner.cc` compiles to a no-op stub when it's off. This
keeps the default build untouched and the feature opt-in — and it's why
`test_runner` is its own file. (For the earlier GIF we deliberately avoided this
and used a scripted harness; for an LLM that needs to drive *arbitrary* UI, the
real Test Engine is worth the build cost.)

---

## 6. Provider abstraction (Claude default, Gemini swappable)

`llm_provider.h` defines the seam; neither `ui_agent` nor the registry depends on
a vendor SDK.

```cpp
struct ToolDef { std::string name, description, json_schema; };
struct ToolCall { std::string id, name; std::string json_args; };

class LlmProvider {
 public:
  virtual ~LlmProvider() = default;
  // Streams assistant text via on_text; returns any tool calls to execute.
  virtual std::vector<ToolCall> SendTurn(
      const Conversation&, const std::vector<ToolDef>&,
      std::function<void(std::string_view)> on_text) = 0;
};
```

- **Claude** (`llm_claude.cc`) is the default: model `claude-opus-4-8`, adaptive
  thinking (`thinking:{type:"adaptive"}`), streaming. Our host is C++, which has
  no official Anthropic SDK, so this is **raw HTTPS to `POST /v1/messages`** with
  our own tool-use loop — the one case the skill sanctions raw HTTP. (If/when the
  app is Python, swap this for the official `anthropic` SDK and
  `@beta_tool`/`client.beta.messages.tool_runner()` — the `LlmProvider` interface
  stays identical.)
- **Gemini** (`llm_gemini.cc`) implements the same interface via function
  calling; tool defs serialize to Gemini's `functionDeclarations`.
- API keys come from the environment / Preferences, never hard-coded.

The action registry and the Test-Engine op-vocabulary are both just `ToolDef`s
to the provider; "edit the model" and "show me in the UI" are two tools (or two
tool families) in the same loop. The model chooses the registry action when one
fits and falls back to writing a UI script otherwise.

---

## 7. End-to-end flow

```
User types in command prompt (no ">")
  → UiAgent::Ask(text)               [threadpool, async]
  → LlmProvider::SendTurn(history, tools=[registry actions + ui_script + read_source])
       model streams reasoning/text  → llm_panel
       model emits tool calls:
         - registry action  → action_registry.run(args)         → result string
         - read_source      → source_index.read(path/range)      → file text
         - ui_script(json)  → test_runner.run(program, record?)  → ok / error / gif path
  → results fed back; loop until no more tool calls
  → final answer rendered in llm_panel; any GIF surfaced inline
```

"Set gravity to -3" → one `set_gravity` action call, done. "How do I change the
integrator?" → `read_source` on the relevant `render` lambda, then a `ui_script`
that opens Physics and demonstrates it, returned as a short GIF. Same loop, same
plumbing.

---

## 8. Safety & guardrails

- **Closed vocabularies.** Registry actions and UI ops are both fixed sets with
  validated args. No arbitrary code, no arbitrary file writes.
- **Source reads are read-only and allow-listed** to the Studio UI tree.
- **Confirmation for destructive verbs.** `reload_model`, `reset`, anything that
  discards state prompts the user (or runs only in an explicit "auto" mode).
- **Dry-run / preview.** A generated UI program can be shown (and its refs
  validated against recorded geometry) before it runs.
- **Bounded loops.** Cap tool iterations per turn; surface errors to the panel.
- **Keys & network** stay out of the source; off by default until configured.

---

## 9. Suggested build order

1. **Routing + panel + Claude provider, no tools.** Non-`>` text → Claude →
   streamed answer in `llm_panel`. Proves the plumbing.
2. **Action registry (C++ seeds).** Wire `toggle_window`, `set_gravity`,
   `set_paused`, `reset` as tools. Now "open physics and pause" works.
3. **C ABI for registration.** Move the seeds to register through the ABI so the
   path Python will use is exercised from day one.
4. **`source_index` read tool.** Let the model read curated UI source; verify it
   can answer "how do I…" in prose with correct labels.
5. **Test Engine behind `MUJOCO_STUDIO_TEST_ENGINE`.** Implement the JSON op
   interpreter + live run.
6. **GIF recording of generated programs** via the existing capture pipeline —
   "show me how to X" returns a clip.
7. **Gemini provider** to validate the abstraction.
8. **Python**: when the app moves to Python, drop the C++ seeds, register actions
   from Python, swap `llm_claude.cc` for the `anthropic` SDK. Interfaces unchanged.

---

## 10. Open questions to settle before coding

- C++ HTTPS client: reuse an existing dependency in-tree, or add a small one?
- Conversation persistence: per-session only, or saved with the `.ini`?
- Do we want vision-in-the-loop (feed `pixels_` frames back to the model so it
  can *see* the UI state), or is text + Test Engine introspection enough at first?
- Exact `source_index` policy: baked-in manifest vs. on-demand file reads — start
  baked-in, revisit once the model is choosing what to read.
