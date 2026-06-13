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
`for j in range(njnt)`), and it can't *see* whether a step worked. Four
escalating answers, same `test_runner` seam — **add them in this order**:

1. **Multi-turn agent loop (default, cheapest).** The model runs a short program
   that *observes* (e.g. `item_read`, or opens a window and reads it), gets the
   result back, then emits a concrete linear program. Control flow lives at the
   *turn* granularity — and we already have that loop. Covers most cases without
   any new machinery.
2. **Vision in the loop (do this before Lua).** Feed rendered `pixels_` frames
   back to the model as image input so it can verify a step landed, read values
   off the screen (status bar, charts), and pursue visual goals ("it fell over —
   reset it"). This is the bigger robustness win *and the cheaper build*: it
   reuses the framebuffer readback we already have for capture, so it's localized
   provider plumbing (encode + attach an image block) with no new subsystem.
   Lua, by contrast, needs a new dependency, a hand-written binding layer, and a
   tricky interaction with the Test Engine frame-yield coroutine — so vision
   comes first.
3. **Embed Lua** (when linear-per-turn gets clumsy). The LLM writes Lua that calls
   bindings into the Test Engine context; real loops/conditionals, in-script
   readback, still sandboxed. `test_runner` swaps its JSON interpreter for a Lua
   VM behind the same "execute a program" interface.
4. **Embed Python via pybind11** (the maximal interpreter). The LLM writes Python
   that drives a thin Test Engine binding — literally a preview of the end state
   (§6), de-risking the migration.

The vocabulary/interpreter is a dial: JSON-ops is the minimum, embedded
Python is the maximum, and `test_runner`'s contract ("run a program against the
Test Engine") never changes as we slide along it. Vision is an orthogonal
*observation* channel that improves every rung without changing the actuator.

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
7. **Vision in the loop.** Feed `pixels_` frames back to the model for
   verification and visual goals — small change, reuses the capture path.
   **Comes before Lua** (cheaper to build, bigger robustness win).
8. **Lua (optional)** — only once per-step turn cost on bulk/iterative tasks
   actually hurts; gives in-script loops/branches/readback.
9. **Gemini provider** to validate the abstraction.
10. **Python**: swap the provider to the `anthropic` SDK and let `run_ui_program`
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

---

## 12. Resolving accurate ImGui Test Engine refs (the hard part)

The single biggest robustness risk is **ref construction**: for `run_ui_program`
to act, the model must produce a ref string that the Test Engine resolves to the
intended widget. Today it does this from grep over source + input files. The
question: can the model *reliably* get a correct ref this way, and what does it
take to make that true? This section is grounded in the actual id code
(`imgui.cpp` `ImHashStr`/`GetID`/`PushID`, `imgui_te_utils.cpp`
`ImHashDecoratedPath`, `imgui_te_context.cpp` wildcard search) and the
[Named-References wiki](https://github.com/ocornut/imgui_test_engine/wiki/Named-References).

### 12.1 How ids and refs actually work (the ground rules)

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

### 12.2 The cases that occur in ImGui code (check every solution against these)

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
   label* via wildcard can reach it.
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
12. **Child windows / our tool windows** — `BeginChild`/`Begin("T###ToolWindowN")`
    produce mangled or index-dependent ids. **Partially greppable** (the literal
    parts), but index/mangling defeats a clean path → needs a convention or
    `WindowInfo(...)`.
13. **Geometry/pos-based ids** (`GetIDFromRectangle`, some internal widgets) and
    **clipped items** (off-screen in a scroll region) — **not statically
    referenceable / invisible to wildcard** without scrolling.

The easy bucket (1–6) is already reliably greppable today. The hard bucket
(7–13) is what any real solution must address: **loops, pointers, computed
labels, framework ids, child/tool windows, clipping.**

### 12.3 The options

**Option 1 — Conventions for our + plugin UI code.**
Require interactive widgets to carry a stable, greppable, human-meaningful id
(an explicit `###key`), and in loops to `PushID(stable_string_key)` (never an
`int`/`ptr`); forbid `PushID(ptr)`; append `###key` to computed labels.
- *Pros:* makes refs deterministic and greppable; easy to state in one sentence
  ("every interactive widget needs a stable `###id`; in loops push a stable
  string key"); zero runtime cost; generalizes to plugins that follow it.
- *Cons:* it's a constraint/footgun on UI authors — exactly what we said we want
  to minimize; unenforceable on third-party/legacy code; the loop key still has
  to *come from somewhere greppable* (works when it's the model name we already
  grep from the input file; doesn't help for purely runtime keys); nothing for
  framework ids (11) or clipping (13).
- *Covers:* 1–7, 10, 12 — **if everyone complies.**

**Option 2 — Macros that bake `__FILE__:__LINE__` into the id.**
A widget macro pushes `###<file>:<line>` so the model greps the source location
(trivially findable) and builds the ref from it.
- *Pros:* the call site is the most greppable thing there is; works even when the
  *label* is dynamic (the id is the location); deterministic at a point in time.
- *Cons:* **a single line in a loop yields N identical `file:line` ids → collision**
  (still needs a per-iteration key — back to case 7); it's a convention/footgun
  (every widget must use the macro, incl. plugins); ids are opaque to humans and
  churn when code moves lines; the model must know the exact `__FILE__` spelling.
- *Covers:* 1–6, 9, 10 *single-instance*; **not 7/8 loops/pointers, not 11.**
  Solves "which call site," not "which iteration."

**Option 3 — Repurpose ImGui's ID Stack Tool / Item Picker (runtime id→path).**
`DebugHookIdInfo` already reconstructs the decorated path for a target id; the
Item Picker captures an item by clicking; `GatherItems()` enumerates; the engine
stores a `DebugLabel` per item. Build a tool that lists live items with their
reconstructed refs.
- *Pros:* **ground truth** — reflects loops, `PushID(int/ptr)`, computed labels,
  dynamic data, all of it, because it reads the live stack as items draw; no
  conventions, no footguns; generalizes to any plugin automatically; this is the
  runtime analog of grep.
- *Cons:* it's **runtime, not source+grep** (breaks the stated "from source"
  premise — though it shares DNA with video feedback, also runtime); only sees
  **visible** items; `PushOverrideID` levels reconstruct as raw hex, not a name
  (referenceable by id, not by readable path); it's one more tool (we want few).
- *Covers:* essentially **all visible cases**; weakest on framework ids (names)
  and clipped items.

**Option 4 — A ref "sentinel"/registry the UI code drops and the prompt knows.**
A one-liner like `StudioRef("knee_slider")` next to (or inside the loop of) an
important widget records that widget's live id under a stable name. The model
greps the source for `StudioRef("…")` names (purely static!) and references by
name; the runtime resolves name→live id. This is the web `data-testid` /
accessibility-id pattern, which is the proven answer in E2E UI testing.
- *Pros:* clean split — **the model works purely from source** (greps marker
  names that *are* in the source) while the runtime does the hard id math;
  robust for annotated widgets regardless of loops/pointers/dynamic labels
  (the marker captures the live id); human-meaningful; **opt-in** (annotate only
  what matters) so it's low-footgun.
- *Cons:* only covers what authors annotate → not automatic/complete; plugins
  must annotate to be first-class; a registry to maintain.
- *Covers:* any case the author chooses to tag, including 7–10; not the untagged
  long tail.

**Option 5 — Alternatives / what we already lean on.**
- *5a. Wildcard + grep-over-inputs + the rules in the prompt (today, zero code).*
  Model greps source+inputs for the leaf label, uses `**/<id-part>`. Covers
  1–10 for **uniquely-labelled, visible** items with **no UI-code constraints**.
  Fails on duplicate labels (7/8), clipping (13), framework ids (11). Cheapest,
  least footgun, not complete.
- *5b. A single generic "list live UI items" introspection tool* (Option 3
  framed minimally: "search the live UI" as the runtime sibling of "search the
  disk"). One generic tool, no conventions, most robust general mechanism.
- *5c. Video feedback as the corrector* — don't require perfect refs; the agent
  sees nothing happened and retries / picks another ref. Subsumes the long tail.

### 12.4 Recommended mix

Layer cheapest-and-most-general first; only add structure where it pays:

1. **Document the exact rules in the system prompt** (§12.1) and **default to
   `**/<leaf>` wildcards + grep over source *and* input files.** Zero UI-code
   constraints; handles the common unique-label case (incl. the knee). *(We have
   most of this; just add the `###`/`$$`/wildcard rules to `system_prompt.md`.)*
2. **An opt-in `data-testid`-style ref marker (Option 4)** for the widgets/loops
   that matter — the low-footgun way to make loops, pointers, and dynamic labels
   reliably addressable without constraining *all* UI code. Plugins that want
   first-class agent control tag their key widgets; the rest still get best
   effort. Prefer this over blanket conventions (Option 1) or `file:line` macros
   (Option 2), which constrain everyone and still don't solve loops.
3. **Runtime safety nets for the long tail:** a single generic *list-live-items*
   tool (5b/Option 3) and/or **video feedback** (5c) for clipped, ambiguous, or
   framework-id cases that no static ref can express.

Explicitly *not* recommended: making the model statically reverse-engineer the
full id algorithm. The hashing is knowable, but `PushID(int/ptr)` in loops and
`PushOverrideID` are fundamentally runtime — chasing them from source text is the
brittle path. Keep the static layer simple (rules + wildcard + grep), make the
important things addressable by name (test-ids), and let runtime feedback close
the rest.
