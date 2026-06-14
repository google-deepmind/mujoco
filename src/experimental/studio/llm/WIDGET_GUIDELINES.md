# Widget guidelines for the LLM agent

MuJoCo Studio lets an LLM agent drive the UI through the Dear ImGui Test Engine
(see `LLM_INTEGRATION_DESIGN.md`). The agent learns the UI by reading source and
by calling `inspect_ui`, which lists every on-screen widget as

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

- [ ] **Combos and numeric inputs don't surface in `inspect_ui`.** Combos
  register no label and `ImGui_Input`/`InputScalar` uses an inner `""` field, so
  both have an empty `DebugLabel` and `TestRunner::DoGather` filters them out.
  They ARE operable — combos via the `combo_select` op, inputs via
  `set_float`/`set_int` — but only by a direct `//Window/Label` path the agent
  has to learn from source, never from inspection. Improve by recording these
  labels (Studio-side, or in `DoGather`) so `inspect_ui` lists them.

- [ ] **Cryptic `##`-only combo ids.** The top-overlay combos use `##Speed`,
  `##Label`, `##Frame`, `##Camera` (`platform/ux/gui.cc`). Operable via
  `combo_select` with e.g. `//<window>/##Speed`, but undiscoverable without
  grepping — give them descriptive `###<Name>` ids.

- [ ] **Verify `menu_click`** drives the main menu bar (File / Edit / Simulation
  / View / Plugins in `studio/app.cc`) end-to-end; the op is implemented but has
  not been exercised.

Done (kept for reference): icon-only buttons — Editor undo/redo, camera copy,
PiP remove, spec-tree add-child/delete, transport pause/viscous/play — now carry
`###<Name>` ids, and `ImGui_ColorButtonEx` draws only the pre-`##` label.
