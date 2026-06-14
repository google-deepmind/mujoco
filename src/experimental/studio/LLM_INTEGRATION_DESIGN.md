# LLM Integration Design — MuJoCo Studio

Status: **implemented.** An LLM agent drives the Studio UI through the Dear ImGui
Test Engine. This document describes the design and the system as built; future
rungs (vision, Lua/Python, more providers) are called out as such.

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
- **The code is the documentation.** The Test Engine addresses items by
  label/path/id; those labels already live in `MainMenuGui()`,
  `RegisterToolWindows()`, the plugin registrations, and each `render` lambda.
  Reading the source *is* reading the API.
- **It matches the user's mental model.** "Show me how" and "do it for me" are the
  same clicks; one mechanism produces both an action and a teachable demo.
- **Safer by default.** The model emits a closed op-program interpreted against a
  fixed vocabulary — not arbitrary code — so the actuator is sandboxed by
  construction (see §7).
- **It's the migration on-ramp.** When Studio becomes Python, "the LLM writes a
  script that drives the Test Engine" stays true verbatim; only the script
  language changes (see §6).

---

## 2. How it's wired

The command palette is the entry point. `command_palette.{h,cc}`: typing `>`
enters command mode over `App::CollectCommands()`; input that does **not** start
with `>` is forwarded to the agent (`app.cc`: the palette's submit callback calls
`ui_agent_.Ask(text)`).

```
prompt text
  ├── starts with ">"  → command-palette mode (App::CollectCommands)
  └── otherwise        → UiAgent::Ask → Claude turn → run_ui_program
```

`UiAgent::Ask` runs the agent **asynchronously on a worker thread** so the render
loop never blocks. The worker runs the Claude tool-use loop; tool calls that touch
the live UI (`inspect_ui`, `run_ui_program`) are posted as jobs to a queue that
the **UI thread drains in `TestRunner::PostSwap()`** (after each frame's swap), so
the Test Engine only ever touches ImGui from the UI thread. `inspect_ui` blocks
the worker on a condition variable until the UI thread's gather completes.

Headless render + framebuffer readback (`pixels_`, `SaveCaptureFrame`) lets the
same agent run with no visible window and dump one PPM per frame — the basis for
the scripted GIF capture used throughout development (`ui_capture.{h,cc}`,
`--capture_gif`/`--capture_prompt`).

---

## 3. Files

Each concern is its own translation unit under `studio/llm/`; `app.cc` gains only
thin wiring (construct a `UiAgent`, give it the `TestRunner`, register the tools,
forward non-`>` prompts).

| File | Responsibility |
|---|---|
| `llm_provider.h` | Provider-agnostic interface: `ToolDef`, `LlmMessage`, `LlmResult`, and `LlmProvider::Send(system, messages, tools, executor)`. No vendor types leak out. |
| `llm_claude.{h,cc}` | Claude implementation: defaults to `claude-haiku-4-5` (switchable via `/model`), raw HTTPS to `POST /v1/messages` (WinHTTP), with our own tool-use loop. `MUJOCO_STUDIO_LLM_VERBOSE` dumps the full transcript. |
| `llm_mock.h` | A trivial offline provider for development without a key. |
| `source_search.{h,cc}` | `GrepSource(pattern, …)` — the `grep` tool's backend; searches the Studio source dir (baked via CMake) plus the loaded model's input-file dir. |
| `test_runner.{h,cc}` | The actuator. Owns the ImGui Test Engine + a single generic test; interprets a `run_ui_program` op-program, runs the cross-thread `inspect_ui` gather, guards every item op so a bad ref/id can't hang. |
| `ui_agent.{h,cc}` | Orchestrates a turn: loads the system prompt (hot-reloaded), runs the tool-use loop, owns the conversation. Runs async on a worker thread. |
| `llm_panel.{h,cc}` | Chat UI rendered from the command palette. |
| `system_prompt.md` | The system prompt, hot-reloaded each turn (override path via `MUJOCO_STUDIO_SYSTEM_PROMPT`) so it can be edited without recompiling. |
| `WIDGET_GUIDELINES.md` | Rules for authoring widgets that the agent can address, plus the audit TODO of code that isn't yet agent-usable. |

There is intentionally **no** `action_registry` and **no** direct-state-poke path.

---

## 4. The actuator: source-grounded Test Engine programs

### 4.1 The model gets three tools

Registered in `App::RegisterLlmTools()`:

- **`grep`** — case-insensitive substring search over the Studio C++ source AND
  the loaded model's input files. Widget labels/ids and shortcuts live in the
  source; model entity names (joints, bodies, actuators) live in the input files
  (e.g. grep `knee` to find the real joint name).
- **`inspect_ui`** — lists the widgets currently visible on screen, grouped by
  window, with their labels and exact `ImGuiID`. The runtime sibling of grep: it
  reports the live UI so the agent can confirm the right panel opened and read
  exact ids. (Combos and numeric-input value fields don't report a label to the
  test engine, so they don't appear here — see §9 and WIDGET_GUIDELINES.)
- **`run_ui_program`** — the sole actuator. Runs a JSON op-program of Test Engine
  operations against the real widgets.

### 4.2 The generated-program contract (`run_ui_program`)

The model emits a small **JSON program** — a list of typed steps — interpreted by
`TestRunner::Execute` against a fixed op vocabulary. It never emits C++ and never
gets raw `ctx` access.

```json
{"ops": [
  {"op": "item_click", "ref": "//ToolRail/###Physics"},
  {"op": "click_id",   "id": 3332464552},
  {"op": "set_float",  "ref": "//Physics/Timestep", "value": 0.002},
  {"op": "combo_select","ref": "//ObjectLauncher/Shape", "value": "Sphere"},
  {"op": "wait",       "seconds": 2}
]}
```

The op vocabulary (each maps to one or a few `ImGuiTestContext` calls):

| op | effect |
|---|---|
| `item_click` / `click_id` | left-click an item (by `ref`, or by exact `id` from inspect_ui) |
| `right_click` / `double_click` | right-/double-click an item (context menus, etc.) |
| `hover` / `item_hold` | move the mouse over an item / press-and-hold for `seconds` |
| `item_check` / `item_uncheck` | set a checkbox-like item |
| `item_open` / `item_close` | expand/collapse a tree node or header by absolute state |
| `set_float` / `set_float_id` / `set_int` | type a number into an input or slider (by ref or id) |
| `set_text` | type a string into a text input |
| `combo_select` | open a combo/popup-menu button and click an entry (`ref`/`id` + `value`) |
| `menu_click` | click a main-menu-bar path, e.g. `View/Tools` |
| `scroll` | scroll a window to top/bottom to reveal clipped content |
| `key_chars` / `key_press` | type text / press a key |
| `wait` | hold for `seconds` (renders frames in place; visible in a recording) |
| `set_ref` | set the base ref for following relative ops |

Each op maps to one `ImGuiTestContext` call, so the vocabulary tracks the Test
Engine's own surface; covering a new non-esoteric ImGui interaction is one more
`else if` in `TestRunner::Execute`.

Addressing: the **most reliable** reference is a widget's exact `ImGuiID` from
`inspect_ui` (`click_id`/`set_float_id`), which is truncation- and clip-proof.
Otherwise refs follow the Test Engine grammar (`//Window/Label`, `**/<label>`
wildcard, `$$N` for `PushID(int)`) — see §9.

Properties:

- **Closed** — no code execution, no recompile, bounded surface.
- **Replayable** — the same JSON drives a live run *or* a headless GIF.
- **Robust** — every item-targeting op is guarded: `TestRunner::Execute` checks
  `ItemExists()` first and passes `ImGuiTestOpFlags_NoError`, so a stale or
  hallucinated ref/id is **skipped (and logged)**, never a deadlock. The
  remaining ops still run.

`TestRunner` registers **one** generic `ImGuiTest` whose `TestFunc` interprets the
current program; we don't compile a new test per request.

### 4.3 "Do" and "show me" are the same program

- **Do X** → run the program once, live.
- **Show me how to X** → run the same program headless and record a GIF through
  the `pixels_` capture pipeline.

### 4.4 Reading state — also through the UI

`inspect_ui` reports the live widget tree; the agent opens the panel it needs and
reads values/labels there, or reads the status bar / Watch window. No direct
`mjModel`/`mjData` read path is introduced. (Vision-in-the-loop — feeding
`pixels_` frames to the model — is a future observation channel, not a second
actuator; see §5.)

---

## 5. Control flow & data-dependent tasks

A flat JSON program can't branch or loop and can't *see* whether a step worked.
The escalating answers, same `TestRunner` seam:

1. **Multi-turn agent loop — built, and the default.** The model runs a program
   (often `inspect_ui` to observe), gets the result back, then emits a concrete
   linear program. Control flow lives at the *turn* granularity. This covers the
   tasks exercised so far (toggle flags, scrub history, drive plugins, add a spec
   element) without new machinery.
2. **Vision in the loop — future, do before Lua.** Feed rendered `pixels_` frames
   back as image input so the model can verify a step landed and pursue visual
   goals. Reuses the framebuffer readback we already have, so it's localized
   provider plumbing with no new subsystem.
3. **Embed Lua — future.** The LLM writes Lua calling Test Engine bindings: real
   loops/conditionals, still sandboxed. `TestRunner` swaps its JSON interpreter
   for a Lua VM behind the same "execute a program" interface.
4. **Embed Python via pybind11 — future, the maximal interpreter** and a preview
   of the Python end state (§6).

The vocabulary/interpreter is a dial; `TestRunner`'s contract ("run a program
against the Test Engine") never changes as we slide along it. Vision is an
orthogonal *observation* channel that improves every rung without changing the
actuator.

---

## 6. Provider abstraction

`llm_provider.h` is the seam; neither `UiAgent` nor `TestRunner` depends on a
vendor SDK. `LlmProvider::Send(system, messages, tools, executor)` runs a full
tool-use loop and calls `executor(name, json_args)` for each tool call.

- **Claude** (`llm_claude.cc`) is the only implementation today. It defaults to
  `claude-haiku-4-5` and is switchable at runtime with the `/model` command
  (opus / sonnet / haiku, or a full id); adaptive thinking
  (`thinking:{type:"adaptive"}`) is used for the opus/sonnet 4.6+ family and
  omitted for Haiku, which doesn't accept it. The host is C++ (no official
  Anthropic SDK), so this is **raw HTTPS to `POST /v1/messages`** via WinHTTP with
  our own tool-use loop (`max_tokens` 8192, generous timeouts, ≤20 iterations).
  The key comes from `ANTHROPIC_API_KEY`, never hard-coded.
- **Other providers** (e.g. Gemini function-calling) would implement the same
  interface; not built yet.

**Python end-state.** When Studio is Python, swap `llm_claude.cc` for the official
`anthropic` SDK and let `run_ui_program` accept Python that drives a Test Engine
binding directly. The `LlmProvider` seam and the single-actuator rule are
unchanged — only the script language graduates from JSON to Python.

---

## 7. End-to-end flow

```
User types in the command palette (no ">")
  → UiAgent::Ask(text)                         [worker thread, async]
  → LlmProvider::Send(system, history, tools = [grep, inspect_ui, run_ui_program])
       model streams reasoning + tool calls:
         - grep(pattern)        → matching file:line lines (source + model files)
         - inspect_ui()         → visible widgets + ids   [UI thread gathers]
         - run_ui_program(json) → TestRunner runs the ops [UI thread, PostSwap]
  → tool results fed back; loop until no more tool calls
  → final answer; any GIF surfaced via the capture pipeline
```

"Turn on contact forces" → inspect the Rendering panel, click the toggle by id.
"How do I do it?" → the same program headless, returned as a GIF. One mechanism,
both outcomes.

---

## 8. Safety & guardrails

- **Single, closed actuator.** `run_ui_program` interprets a fixed op set; no
  arbitrary code, no direct state poking. Sandboxed by construction.
- **`grep` is read-only**, scoped to the Studio source dir and the loaded model's
  input-file dir.
- **Can't hang on a bad action.** Every item op is gated by `ItemExists()` +
  `ImGuiTestOpFlags_NoError`; an unresolved ref/id is skipped and logged. Headless
  runs guard against UI that can't render offscreen (e.g. Picture-in-Picture's
  render-to-texture is skipped headless).
- **Bounded loops.** Tool iterations (≤20), `run_ui_program` length, `wait`
  duration, and the per-turn `grep` budget are all capped.
- **Keys & network** stay out of source; the agent is inert until a key is set.

Not yet built: a preview/approval gate before a program runs, and explicit
confirmation for destructive verbs (reload/reset). Programs currently run directly
(echoed to the console for debugging).

---

## 9. Build

The Test Engine is part of the Studio build: Dear ImGui is compiled with
`IMGUI_ENABLE_TEST_ENGINE` (PUBLIC on the `dear_imgui` target), and
`imgui_test_engine` is fetched and linked
(`cmake/third_party_deps/imgui_test_engine.cmake`), built with
`IMGUI_TEST_ENGINE_ENABLE_COROUTINE_STDTHREAD_IMPL=1` (otherwise
`ImGuiTestEngine_Start` asserts on missing coroutine funcs). `MUJOCO_STUDIO_SOURCE_DIR`
is baked in so `grep` can find the source at runtime. Headless runs set
`_set_error_mode(_OUT_TO_STDERR)` so asserts don't pop a blocking dialog.

Teardown: only `ImGuiTestEngine_Stop()` is called at exit and the engine is leaked
— the window never destroys the ImGui context, and
`ImGui::DestroyContext()` must precede `ImGuiTestEngine_DestroyContext()`.

---

## 10. Status & next steps

Built: command-palette routing, async agent, Claude provider with tool-use loop,
the three tools, the op interpreter with guards, cross-thread `inspect_ui`, the
`combo_select`/`wait` ops, headless GIF capture, the hot-reloaded system prompt,
and the widget-authoring guidelines.

Next, roughly in order: **vision in the loop** (feed `pixels_` frames back —
cheap, big robustness win), then Lua, then a second provider, then the Python
end-state. Plus the WIDGET_GUIDELINES TODO (make the spec property editor and
the native file dialog agent-addressable).

---

## 11. Resolving accurate ImGui Test Engine refs (the hard part)

The single biggest robustness risk is **ref construction**: for `run_ui_program`
to act, the model must produce a ref (or use an id) that the Test Engine resolves
to the intended widget. This section is grounded in the actual id code
(`imgui.cpp` `ImHashStr`/`GetID`/`PushID`, `imgui_te_utils.cpp`
`ImHashDecoratedPath`, `imgui_te_context.cpp` wildcard search) and the
[Named-References wiki](https://github.com/ocornut/imgui_test_engine/wiki/Named-References).

### 11.1 How ids and refs actually work (the ground rules)

An item's id is `hash(id_significant_label, seed = top of the ID stack)`:

- **`ImHashStr` rules.** The display label is everything before the first `##`.
  The *id-significant* substring is everything from the **last `###`** onward
  (the `###` resets the hash), else the whole label — and a plain `##` (two
  hashes) is **included** in the id. So `Button("Save")`→id of `"Save"`;
  `Button("Save##2")`→id of `"Save##2"` (display "Save"); `Button("Save###x")`→id
  of `"###x"` (display "Save").
- **The ID stack (seed).** `Begin`, `TreeNode`, `PushID`, tabs, tables, etc. push
  levels; each item is seeded by the level above it. `PushID(const char*)` pushes
  `hash(str)`; **`PushID(int n)` pushes `hash(raw int bytes)`**; **`PushID(void*
  p)` pushes `hash(raw pointer bytes)`**.
- **Test Engine ref grammar** (`ImHashDecoratedPath`): `/` chains the seed (== the
  ID stack); a leading `//` resets the seed to 0 (absolute); `###` resets;
  `$$N` encodes a `PushID(int N)` level; `$$(type)value` encodes a pointer level
  (discouraged); `\` escapes a literal `/`; `//$FOCUSED` = focused window.
- **Wildcard `**/leaf`** resolves on the GUI side by matching an item whose
  **label** satisfies `ImHashStr(label,0) == ImHashStr(leaf,0)` — i.e. it matches
  the *leaf label string anywhere*, ignoring the whole seed/stack. Powerful, but:
  only matches **visible (unclipped)** items, no partial matches, and is
  ambiguous if the label isn't unique.

### 11.2 The cases that occur in ImGui code

Ordered roughly easy → hard, with whether a *static* (source+grep) ref is
derivable:

1. **Literal label, window scope** — `Button("OK")`. Ref `//Win/OK`. **Greppable.**
2. **`##` disambiguator** — `Button("Play##2")`. Ref `//Win/Play##2`. **Greppable.**
3. **`###` stable id** — `Button("Play###x")`. Ref must use `###x`, not `Play`.
   **Greppable** (if the model applies the `###` rule).
4. **Nested literal stack** — `TreeNode("N")` … `Button("OK")`. Ref `//Win/N/OK`.
   **Greppable** (if all levels are literals).
5. **`PushID("key")`** — literal string key. Ref includes the key segment.
   **Greppable.**
6. **`PushID(literal int)`** — e.g. `PushID(3)`. Ref `…/$$3/…`. **Greppable.**
7. **Widgets in a loop with `PushID(i)`** — the seed depends on the runtime index;
   the same source line makes N items. **Not directly greppable**: the model sees
   `PushID(i)` but not the values. Sometimes recoverable as `$$0,$$1,…` *if* it
   can bound the loop (often from input-file data); the leaf label is frequently
   identical across iterations → wildcard is ambiguous.
8. **`PushID(ptr)`** — seeded from a runtime pointer. **Not greppable, not even
   stable across runs.** Only `$$(ptr)value` (unknowable) or a *unique leaf
   label* via wildcard can reach it. (The spec property editor does this — see
   WIDGET_GUIDELINES.)
9. **Computed/`snprintf` labels** — `snprintf(b,"joint %d",i); Slider(b)`. The
   label is runtime; source has only the format string. **Not greppable;** needs
   the exact rendered label.
10. **Runtime-data labels (our knee)** — `Slider(joint_name)` where `joint_name`
    comes from the loaded model. The string isn't in the C++ at all — it's in the
    **input file**. **Greppable only because we grep input files too;** ref via
    `**/knee_right` (visible) or the constructed window path.
11. **Framework `PushOverrideID`** — docking nodes, tab items, table instance ids,
    popups. Ids are computed from window/table ids, not labels. **Not statically
    referenceable**; the engine exposes helpers (`WindowInfo`, table/tab APIs)
    instead.
12. **Child windows / tool windows** — `BeginChild`/`Begin("T###ToolWindowN")`
    produce mangled or index-dependent ids. **Partially greppable** (the literal
    parts), but index/mangling defeats a clean path.
13. **Geometry/pos-based ids and clipped items** (off-screen in a scroll region) —
    **not statically referenceable / invisible to wildcard** without scrolling.

The easy bucket (1–6) is reliably greppable. The hard bucket (7–13) is what the
runtime tool below addresses: loops, pointers, computed labels, framework ids,
child/tool windows, clipping.

### 11.3 What we use (and why)

The design analysed several strategies — source conventions, `__FILE__:__LINE__`
macros, a runtime id→path tool, opt-in `data-testid` markers, and pure
wildcard+grep. The **built** answer layers the cheapest-and-most-general first:

1. **`inspect_ui` — the live id query — is the primary mechanism.** It enumerates
   the visible items and reports each one's exact `ImGuiID`, and the agent acts on
   that id (`click_id`/`set_float_id`). This is the runtime analog of grep and is
   ground truth: it reflects loops, `PushID(int)`, computed labels, and dynamic
   data, because it reads the live UI as items draw. It is the answer for the hard
   cases (7, 9, 10) for any **visible, label-reporting** item, and it's
   truncation- and clip-proof since the id is exact.
2. **`combo_select`** handles combos and popup-menu buttons (which `inspect_ui`
   can't list — see §9): it opens the control and clicks the named entry in the
   focused popup, all guarded.
3. **The id/`$$`/wildcard rules in the system prompt**, plus `grep` over source
   *and* input files, cover the static cases (1–6, 10) and the fallback
   `**/<leaf>` for uniquely-labelled visible items.
4. **Widget-authoring conventions** (WIDGET_GUIDELINES): give interactive widgets
   a descriptive `###<Name>` id and keep ids stable, so our own UI is addressable
   by inspection without per-widget markers.

Known gaps, all documented in WIDGET_GUIDELINES: combos/inputs don't *list* in
`inspect_ui` (still operable via grep — a combo at `//Window/Label`, a Studio
`ImGui_Input` field at `//Window/Label/$$0` since it wraps `InputScalarN`); the
spec property editor uses `PushID(&val)` (case 8) so its fields aren't
addressable; the native file dialog isn't drivable by the Test Engine.

Deferred (analysed, not built): opt-in `data-testid`-style ref markers for the
untagged long tail, and **video feedback** as the corrector that lets the agent
see nothing happened and retry — the eventual safety net for clipped, ambiguous,
and framework-id cases that no static ref can express.

Explicitly *not* pursued: making the model statically reverse-engineer the full id
algorithm. `PushID(int/ptr)` in loops and `PushOverrideID` are fundamentally
runtime; chasing them from source text is the brittle path. Keep the static layer
simple (rules + wildcard + grep), let `inspect_ui` resolve live ids, and let
runtime feedback close the rest.

---

## 12. Extracting a standalone ImGui agent library (assessment)

The integration is already **independent of MuJoCo**: `studio/llm/*.{h,cc}`
contain zero MuJoCo symbols (`mjModel`/`mjData`/`mjs_`/`mjv_`/`mj_`, no
`<mujoco>` include) and `#include` only each other plus `imgui_test_engine`. So
extracting it into a reusable `imgui_llm`-style library is *packaging, not
surgery* — a payoff of the single-actuator design.

- **Free (already decoupled):** the actuator (`test_runner` — op interpreter,
  `inspect_ui` gather, `ItemExists` guards, `combo_select`, …), the provider seam
  (`llm_provider`), `ui_agent`, `llm_panel`.
- **Small / mechanical:** rename the `mujoco::studio` namespace + include paths;
  parameterize `source_search`'s source dir (today a `MUJOCO_STUDIO_SOURCE_DIR`
  build define) to host-supplied; move the app-specific bits of the
  `run_ui_program` tool *description* (it names `ToolRail` / Studio panels) into
  the host's system prompt.
- **Moderate (the real work):** a small public API (host registers its prompt,
  grep dirs, extra tools, and calls `Start`/`Stop`/`PostSwap` each frame) + a
  standalone CMake that fetches dear_imgui (with `IMGUI_ENABLE_TEST_ENGINE`) and
  `imgui_test_engine` (the existing `cmake/third_party_deps/imgui_test_engine.cmake`
  is reusable); and a cross-platform HTTP transport (today the Claude transport is
  WinHTTP-only with a non-Windows stub).
- **Stays in the host:** `ui_capture` (the headless GIF harness) drives app
  state, not the library.
- **Host constraint (inherent, not work):** the embedding app must build ImGui
  with `IMGUI_ENABLE_TEST_ENGINE` + a coroutine impl and link
  `imgui_test_engine`.

Rough estimate: **~1–2 days** Windows-only (namespace + include paths + CMake +
API façade; near-zero logic change), **+2–4 days** for cross-platform (a libcurl
transport and Linux/macOS testing).

---

## 13. A "prompt-only" WASM demo (brainstorm)

Goal: a stripped WASM build where the user sees only the 3D viewport + the Ctrl+P
command palette. The full ImGui UI (rail, panels, menus, scrubber, …) still
exists and runs so the agent can address and drive it, but is invisible — so the
app *appears* to be controlled purely by the prompt.

**The enabling fact:** the Test Engine addresses items from the per-frame item
registry (`ItemAdd` ids + rects), populated by **building** the UI, not by
**drawing** it. Addressability is orthogonal to visibility. So we can build the
whole UI every frame (agent can inspect/click it) while suppressing its pixels.

### Simplest approach — "ghost UI" (alpha 0)
Add a `prompt_only` flag. When on, render every ImGui window EXCEPT the command
palette with `style.Alpha = 0` (set it at the top of `BuildGui`, restore to 1
just before the palette, which is already drawn last). The windows are still
submitted, so their items keep ids/rects and stay fully clickable by the Test
Engine; they simply draw nothing. The 3D viewport (the renderer, not ImGui) draws
normally behind, the palette draws on top. The user sees: scene + palette.
- Why it works: alpha is render-only — `ItemAdd`, hit-testing, `GatherItems`, and
  `ItemClick` all operate on the registry + rects, unaffected by alpha. The agent
  toggles e.g. "Contact Force" in the invisible Rendering panel and the *visible*
  viewport updates, because the hidden UI mutates the same shared state the
  renderer reads.
- Effort: tens of lines — gate the non-palette windows' alpha behind the flag.
- Caveat: an invisible window still *captures mouse* if the user clicks over its
  area. For a passive, prompt-driven demo that's fine (the user only types). To
  remove even that, also push hidden windows off-screen (below).

### Refinement — off-screen positioning
Force the hidden windows to coordinates outside the visible viewport
(`SetNextWindowPos` Always). They're invisible AND never under the user's mouse
(no input stealing), while the Test Engine still clicks them by moving its
simulated mouse to their off-screen rects (ImGui hit-tests in its own coordinate
space, not the OS window). Wrinkle: ImGui's keep-windows-on-screen clamp can pull
them back, so it needs a forced position (and possibly an internal pos set) —
slightly fiddlier than alpha-0, hence a refinement, not the default.

### Heavier alternatives (not needed)
- Filter `ImDrawData`: drop the hidden windows' draw lists before the backend
  renders. Clean visually, but ImGui exposes no tidy cmd-list→window map.
- Render the hidden UI into a separate offscreen ImGui context. Most isolation,
  most plumbing.

### WASM fit
The emscripten target already builds (`emscripten.cc`, WebGL2 Filament). This is
a render-mode toggle on top of it — no new subsystem — so the prompt-only build
is the existing WASM app plus the ghost-UI flag.

### Verdict
Yes, simple. The **ghost-UI alpha-0 mode** yields a convincing "controlled only by
the prompt" app for ~tens of lines, WASM-compatible, precisely because the Test
Engine needs the UI *built*, not *shown*. Mouse-capture by the invisible windows
is the only rough edge, removed by off-screen positioning if it matters.
