# Widget guidelines for the LLM agent

MuJoCo Studio lets an LLM agent drive the UI through the Dear ImGui Test Engine
(see `LLM_INTEGRATION_DESIGN.md`). The agent learns the UI by reading source and
by calling `inspect_ui`, which lists the on-screen widgets that report a label as

```
[<Window>]
  <label>  [id=<ImGuiID>]
```

and then acts on widgets by their exact `ImGuiID`. Two authoring rules keep your
widgets operable by the agent (and, as a bonus, give you robust test ids). Both
come from real failures while wiring up the agent.

## 1. Give interactive controls a descriptive `###<Name>` id

An ImGui widget's id text is whatever you pass as its label, with everything up
to and including the last `###` removed (`###` resets the id hash). For the test
engine and `inspect_ui`, that id text is the ONLY human-readable handle on the
widget — it is the truncated `DebugLabel` the engine reports.

So an icon-only button such as

```cpp
ImGui::Button(ICON_FA_CARET_UP);            // BAD: id text is the icon's bytes
```

has an id derived from the icon glyph's raw UTF-8 bytes. `inspect_ui` shows it as
gibberish (or empty), and neither you nor the agent can tell what it does or
build a wildcard ref for it. The agent can still click it *if* it happens to know
the id, but it can't recognise the control in the first place.

Append a self-describing `###<Name>`:

```cpp
ImGui::Button((std::string(ICON_FA_CARET_UP) + "###Next frame").c_str());
//             ^ visible: the icon            ^ id text: "Next frame"
```

The glyph still renders; the id text becomes `Next frame`; `inspect_ui` prints
`Next frame  [id=...]`; and the agent can find and operate it. Prefer to expose
the *operation the agent will want* as its own self-describing control — e.g. a
"Go to start" / "Oldest frame" button is far easier for the agent to use than
asking it to drag a slider to an unknown minimum value.

Rule of thumb: every clickable/editable widget should read like a verb or a name
after its `###`. Decorative-only widgets (separators, static text) don't need it.

## 2. Keep ids stable — don't let state leak into the id

A widget's identity must not change when its *state* changes, or ImGui treats it
as a brand-new widget every time the state flips (losing focus/active state) and
any stored ref or id the agent captured goes stale.

The trap is putting state-dependent text *before* the `###` (or with no `###` at
all):

```cpp
// BAD: the icon depends on `on`, so the id flips every toggle.
const char* icon = on ? ICON_FA_CHECK_SQUARE_O : ICON_FA_SQUARE_O;
ImGui::Button((std::string(icon) + " " + label).c_str());
```

Anchor the id with a stable `###<key>` so only the *appearance* before it
changes:

```cpp
// GOOD: appearance varies, id == hash of "<label>" stays put.
ImGui::Button((std::string(icon) + " " + label + "###" + label).c_str());
```

The same applies to any label that embeds a value, a frame counter, a row index
from sorted/filtered data, etc. — compute a stable key for the `###` part and let
the volatile text live before it. (`ImGui::SetItemTooltip`, value readouts, and
`%d`-formatted display strings are fine *before* the `###`.)

### Why this matters here specifically

- `inspect_ui` reports a truncated `DebugLabel` (32 chars). The exact `ImGuiID`
  is always correct, but the agent picks its target by the readable label — so a
  bad/empty label means a control the agent can see but not understand.
- The agent addresses widgets by the `[id=N]` it saw, so a stable id means a ref
  the agent captured this turn is still valid next turn.
- Wildcard refs (`**/<label>`) match an item's label hash and require the item to
  be unique and on screen; a descriptive `###<Name>` makes those refs meaningful
  too.

## TODO: existing code to make LLM-usable

From an audit of the UI against the rules above (2026-06-14). Worst first.

- [ ] **Spec property editor fields are unaddressable.** `Table::Input<T>`
  (`platform/ux/imgui_widgets.h`, used by `ElementSpecGui` in
  `platform/ux/gui_spec.cc`) wraps every property field in
  `ImGui::PushID(&val)` — a runtime pointer no test-engine ref can encode — and
  the fields use empty `##` labels so they don't appear in `inspect_ui` either.
  Effect: the agent can add/delete elements in the Editor but cannot edit ANY
  element property (body pos/quat, geom type/size, joint range, mass, ...). Fix:
  scope each row by a stable string — thread the row label from `Table::Label()`
  into `Table::Input()` and use `PushID(label)` instead of `PushID(&val)`.

- **Combos and numeric inputs don't surface in `inspect_ui` — accepted, not a
  blocker.** ImGui only forwards a label to the test engine when a widget calls
  `IMGUI_TEST_ENGINE_ITEM_INFO` itself; `BeginCombo` never does and stepped
  `InputScalar` forwards its inner `""`, so both reach `TestRunner::DoGather`
  with an empty `DebugLabel` and get filtered. They are still OPERABLE — combos
  via `combo_select` on `//Window/Label`, and numeric inputs via
  `set_float`/`set_int` on `//Window/Label/$$0` (Studio's `ImGui_Input` wraps
  `InputScalarN`, which nests the value field one `PushID(int)` level deeper) —
  both found by grepping the source, so this is a discoverability gap, not a
  wall. Auto-listing them would need either a multi-frame ID-Stack-
  Tool resolver driven from the app loop (ImGui zeroes `DebugHookIdInfoId` in
  `NewFrame` and only follows the hovered item, so it can't be driven from the
  gather coroutine) or a patch to vendored ImGui. Both judged not worth it for
  now; left as grep-discoverable.

- [x] **Cryptic `##`-only combo ids** — DONE. The top-overlay Speed/Label/Frame/
  Camera combos (`platform/ux/gui.cc`) now use `###<Name>` ids.

- [x] **`menu_click`** — DONE. It now roots a bare path at the `##MainMenuBar`
  window (a bare first segment was being read as a window name). Verified by
  opening the ImGui demo window from `Help/Developer` and driving it.

- [ ] **File open/save uses a NATIVE OS dialog.** `App::FileDialogGui`
  (`studio/app.cc`) calls `platform::OpenFileDialog` / `SaveFileDialog`, which on
  Windows is `IFileOpenDialog` (a native COM dialog), on Linux zenity, on macOS
  Cocoa. The test engine cannot drive any of these, and headless they block / pop
  a real desktop dialog -- so "use the File menu to load X" is not doable by the
  agent and is unsafe to run. Fix: add an in-app (ImGui) path field + Open button
  in the modal (the native browser can stay as a "Browse..." button), so the
  agent can type a path and click Open.

Done (kept for reference): icon-only buttons — Editor undo/redo, camera copy,
PiP remove, spec-tree add-child/delete, transport pause/viscous/play — now carry
`###<Name>` ids, and `ImGui_ColorButtonEx` draws only the pre-`##` label.
