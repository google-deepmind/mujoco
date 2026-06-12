# LLM Integration Design — MuJoCo Studio

Status: **design only, no code yet.**

## One mechanism, on purpose

The LLM operates Studio through **exactly one** actuator: the **ImGui Test
Engine**. It does not call a parallel "action API," it does not poke `mjModel` /
`mjData` directly, and it does not get a hand-written registry of verbs. To change
anything, it does what a person does — clicks the button, types in the field,
drags the slider — by emitting a UI program that the Test Engine plays back.

The capability that makes this work:

> **The LLM reads the Studio source to learn what the UI does, then drives that
> UI through the ImGui Test Engine API.**

The source code is the contract. There is no second description of the app to
write or keep in sync. When the UI changes, the model re-reads the source and
adapts. "Do X" and "show me how to do X" are the *same* program — run it live, or
run it headless and hand back a GIF.

This is deliberately the **only** path. Everything below serves it.

---

## 1. Why exclusively Test Engine

- **No second API to maintain.** A registry of `set_gravity` / `toggle_window`
  functions is a parallel implementation that drifts from the buttons and has to
  be re-described to the model. The Test Engine reuses the real widgets, so the
  app's behavior and the LLM's behavior can never diverge.
- **The code is the documentation.** Test Engine addresses items by label/path;
  those labels already live in `MainMenuGui()`, `RegisterToolWindows()`, and each
  `render` lambda. Reading the source *is* reading the API.
- **It matches the user's mental model.** "Show me how" and "do it for me" are the
  same clicks; one mechanism produces both an action and a teachable demo.
- **Safer by default.** The model emits a closed op-program interpreted against a
  fixed vocabulary — not arbitrary code — so the actuator is sandboxed by
  construction (see §7).
- **It's the migration on-ramp.** When Studio becomes Python, "the LLM writes a
  script that drives the Test Engine" stays true verbatim; only the script
  language changes (see §6).

---

## 2. Where it plugs into the existing app

We already have the seams:

- **Command palette** (`command_palette.{h,cc}`). Typing `>` enters command mode
  over `App::CollectCommands()`. Input that does **not** start with `>` currently
  does nothing — that is the channel we route to the LLM.
- **Tool-window registry** (`App::tool_windows_`) with stable `###ToolWindowN`
  IDs — clean, predictable Test Engine refs for every floating window.
- **Recorded UI geometry** (`rail_button_center_`, `tool_window_rect_`) — runtime
  ground truth for where items actually are, usable to validate a generated ref
  before we run it.
- **Headless render + framebuffer readback** (`pixels_`, `SaveCaptureFrame`) —
  already lets us run the UI with no visible window and dump frames. This is the
  basis for headless playback, GIF demos, and (optionally) letting the model
  *see* UI state.

The command prompt becomes a dispatcher:

```
prompt text
  ├── starts with ">"  → existing command-palette mode (unchanged)
  └── otherwise        → LLM turn → ImGui Test Engine program
```

---

## 3. New files (per the "separate files" rule)

Nothing bloats `app.cc`; each concern is its own translation unit.

| File | Responsibility |
|---|---|
| `llm/llm_provider.h` | Provider-agnostic interface: send a turn, stream text, surface tool calls, return tool results. No vendor types leak out. |
| `llm/llm_claude.{h,cc}` | Claude implementation. Default `claude-opus-4-8`, adaptive thinking, streaming. (C++ host → raw HTTPS to `/v1/messages`; see §6.) |
| `llm/llm_gemini.{h,cc}` | Optional Gemini implementation (function calling), same interface. |
| `llm/source_index.{h,cc}` | Gives the model curated, read-only access to the Studio UI source so it can derive correct Test Engine refs (see §4). |
| `llm/test_runner.{h,cc}` | The actuator. Wraps ImGui Test Engine: interpret a generated UI program, run it live or headless, validate refs, report step failures, record a GIF via the existing capture path. Compiled only when `IMGUI_ENABLE_TEST_ENGINE` is on. |
| `llm/ui_agent.{h,cc}` | Orchestrates a turn: build the system prompt, run the tool-use loop whose tools are `read_source` and `run_ui_program`, own the conversation/history. |
| `llm/llm_panel.{h,cc}` | The chat UI (history, streaming tokens, the generated program shown before it runs, inline GIF result). The command prompt is the entry point; longer exchanges render here. |

There is intentionally **no** `action_registry` file — that path is removed.

`app.cc` gains only thin wiring: construct a `UiAgent`, give it the `test_runner`
and `source_index`, and forward non-`>` prompt submissions to it.

---

## 4. The actuator: source-grounded Test Engine programs

### 4.1 Test Engine is a small VM

Its API takes string refs and runtime values:

```cpp
ctx->MenuClick("View/Tools");
ctx->ItemClick("//Tools/Physics");        // a rail button
ctx->SetRef("Physics");                    // a tool window (###ToolWindowN)
ctx->ItemInputValue("Gravity/$$2", -3.0f);
ctx->KeyChars("qpos");
```

So we never generate C++. We generate the *arguments*. The hard part is normally
*knowing the labels and paths* — and those are in the source.

### 4.2 The model reads the source (`read_source`)

`source_index` exposes a `read_source` tool that returns curated UI source: the
`render` lambdas, `MainMenuGui()`, the FontAwesome label constants, and the
`###ToolWindowN` ID scheme. Start as a baked-in manifest (file list + a short
"window → ImGui ID" map); optionally grow into on-demand "read this file/range."
Either way the model is grounded in the real strings, not guessing.

### 4.3 The generated-program contract (`run_ui_program`)

The model does **not** emit arbitrary C++ to compile, and does **not** get raw
`ctx` access. It emits a small **JSON program** — a list of typed steps —
interpreted by `test_runner` against a fixed op vocabulary:

```json
[
  {"op": "menu_click", "path": "View/Tools"},
  {"op": "item_click", "ref": "//Tools/Physics"},
  {"op": "set_float",  "ref": "Physics/Gravity Z", "value": -3.0},
  {"op": "screenshot", "label": "after"}
]
```

A starter vocabulary (each op = one `ImGuiTestContext` call): `menu_click`,
`item_click`, `item_check`/`item_uncheck`, `set_float`, `set_int`, `key_chars`,
`set_ref`, `item_read` (observe a value back), `screenshot`, `wait_frames`.

Properties:

- **Closed** — no code execution, no recompile, bounded surface.
- **Replayable** — the same JSON drives a live run *or* a headless GIF.
- **Inspectable** — show the user the steps (and let them approve) before running.
- **Self-correcting** — Test Engine fails loudly on a bad ref; that becomes a
  tool-result error the model fixes next iteration, exactly like a coding agent
  fixing a failing test. `tool_window_rect_` / `rail_button_center_` let us
  pre-validate a ref resolves before running.

`test_runner` registers **one** generic `ImGuiTest` whose `TestFunc` interprets
the current program; we don't compile a new test per request.

### 4.4 "Do" and "show me" are the same program

- **Do X** → run the program once, live.
- **Show me how to X** → run the same program headless and record a GIF through
  the existing `pixels_` capture pipeline; surface the clip in `llm_panel`.

### 4.5 Reading state — also through the UI

Because Test Engine is the only actuator, the model learns current state the same
way: `item_read` pulls a widget's value back, or it opens the Watch window and
reads it. (Optionally, vision-in-the-loop: feed `pixels_` frames to the model so
it can *see* the result — observation, not a second actuator.) No direct
`mjModel`/`mjData` poke path is introduced.

---

## 5. Control flow & data-dependent tasks

A flat JSON program can't branch or loop ("set *every* hinge to zero" needs
`for j in range(njnt)`). Three escalating answers, same `test_runner` seam:

1. **Multi-turn agent loop (default, cheapest).** The model runs a short program
   that *observes* (e.g. `item_read`, or opens a window and reads it), gets the
   result back, then emits a concrete linear program. Control flow lives at the
   *turn* granularity — and we already have that loop. Covers most cases without
   any new machinery.
2. **Embed Lua** (when linear-per-turn gets clumsy). The LLM writes Lua that calls
   bindings into the Test Engine context; real loops/conditionals, still
   sandboxed. `test_runner` swaps its JSON interpreter for a Lua VM behind the
   same "execute a program" interface.
3. **Embed Python via pybind11** (the maximal interpreter). The LLM writes Python
   that drives a thin Test Engine binding — literally a preview of the end state
   (§6), de-risking the migration.

The vocabulary/interpreter is a dial: JSON-ops is the minimum, embedded
Python is the maximum, and `test_runner`'s contract ("run a program against the
Test Engine") never changes as we slide along it.

---

## 6. Provider abstraction (Claude default, Gemini swappable)

`llm_provider.h` is the seam; neither `ui_agent` nor `test_runner` depends on a
vendor SDK.

```cpp
struct ToolDef { std::string name, description, json_schema; };
struct ToolCall { std::string id, name; std::string json_args; };

class LlmProvider {
 public:
  virtual ~LlmProvider() = default;
  virtual std::vector<ToolCall> SendTurn(
      const Conversation&, const std::vector<ToolDef>&,
      std::function<void(std::string_view)> on_text) = 0;
};
```

The model is given exactly two tools: `read_source` and `run_ui_program`.

- **Claude** (`llm_claude.cc`) is the default: `claude-opus-4-8`, adaptive
  thinking (`thinking:{type:"adaptive"}`), streaming. Our host is C++ (no official
  Anthropic SDK), so this is **raw HTTPS to `POST /v1/messages`** with our own
  tool-use loop — the one case the skill sanctions raw HTTP.
- **Gemini** (`llm_gemini.cc`) implements the same interface via function calling.
- Keys come from the environment / Preferences, never hard-coded.

**Python end-state.** When Studio is Python, swap `llm_claude.cc` for the official
`anthropic` SDK (`@beta_tool` + `client.beta.messages.tool_runner()`), and let
`run_ui_program` accept Python that drives the Test Engine binding directly. The
`LlmProvider` interface and the single-actuator rule are unchanged — only the
script language graduates from JSON to Python.

---

## 7. End-to-end flow

```
User types in command prompt (no ">")
  → UiAgent::Ask(text)               [threadpool, async — never blocks render]
  → LlmProvider::SendTurn(history, tools = [read_source, run_ui_program])
       model streams reasoning/text   → llm_panel
       model emits tool calls:
         - read_source(path/range)    → curated UI source text
         - run_ui_program(json)       → test_runner: validate refs → run
                                          live or headless → ok / error / gif path
  → results fed back; loop until no more tool calls
  → final answer in llm_panel; any GIF surfaced inline
```

"Set gravity to -3" → read the Physics `render` lambda, emit a 3-step program,
run it. "How do I change the integrator?" → read the lambda, emit + run a headless
program, return a short GIF. One mechanism, both outcomes.

---

## 8. Safety & guardrails

- **Single, closed actuator.** `run_ui_program` interprets a fixed op set; no
  arbitrary code, no direct state poking. Sandboxed by construction.
- **`read_source` is read-only and allow-listed** to the Studio UI tree.
- **Preview + approval.** The generated program is shown (and its refs validated
  against recorded geometry) before it runs; destructive verbs (reload, reset)
  require confirmation unless an explicit "auto" mode is on.
- **Bounded loops.** Cap tool iterations and program length per turn; surface
  step errors to the panel.
- **Keys & network** stay out of source; off until configured.

---

## 9. Build prerequisite (called out honestly)

Test Engine requires building Dear ImGui with `IMGUI_ENABLE_TEST_ENGINE` and
linking `imgui_test_engine` (+ its capture tool for GIFs). That's a real build
change, so it lives behind a CMake option (`MUJOCO_STUDIO_TEST_ENGINE`, default
OFF); `test_runner.cc` compiles to a no-op stub when off. This keeps the default
build untouched and is why `test_runner` is its own file. (The earlier GIF used a
scripted harness specifically to avoid this; for an LLM that must drive
*arbitrary* UI, the real Test Engine earns its build cost.)

---

## 10. Suggested build order

1. **Routing + panel + Claude provider, no tools.** Non-`>` text → Claude →
   streamed answer in `llm_panel`. Proves the plumbing.
2. **`read_source`.** Curated UI-source tool; verify the model can answer "how do
   I…" in prose with correct labels.
3. **Test Engine behind `MUJOCO_STUDIO_TEST_ENGINE`.** Implement the JSON op
   interpreter + live run + ref validation + step-error feedback.
4. **`run_ui_program` end-to-end.** "Open physics and set gravity to -3" actually
   works via clicks.
5. **GIF recording** of generated programs via the existing capture pipeline —
   "show me how to X" returns a clip.
6. **Multi-turn observe→act** (`item_read` / Watch) for data-dependent tasks.
7. **Gemini provider** to validate the abstraction.
8. **Python**: swap the provider to the `anthropic` SDK and let `run_ui_program`
   take Python driving the Test Engine binding. Single-actuator rule unchanged.

---

## 11. Open questions to settle before coding

- C++ HTTPS client: reuse an in-tree dependency, or add a small one?
- Conversation persistence: per-session only, or saved with the `.ini`?
- Op vocabulary v1: which ops are must-have vs. deferred?
- `source_index` policy: baked-in manifest vs. on-demand file reads — start
  baked-in, revisit once the model is choosing what to read.
- Vision-in-the-loop: feed `pixels_` frames back for observation, or rely on
  `item_read` + Watch at first?
