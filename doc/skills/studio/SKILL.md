---
name: mujoco-studio
description: >-
  MuJoCo Studio viewer applications, interactive visualization, decoupled
  sim/viewer threads, message passing (Snapshot latest-wins vs Event FIFO queue),
  lifecycle events, @messages.handler priorities, blocking viewer runners and
  non-blocking viewer launchers, Python/C++ plugin registry (mjPLUGIN_LIB_INIT),
  NetImgui/WASM WebViewer streaming,
  Python plugin hot reload (--hot_reload, HotReloadEvent).
  Use for launching Studio viewers, authoring Studio plugins, simulation-viewer messaging.
  Do NOT use for ImGui widget layout (mujoco-gui) or physics simulation (mujoco-python).
---

# MuJoCo Studio: Interactive Simulation, UI & Plugin Architecture

"Studio" is the term used to refer to all the components used to build
visualizations that replace the legacy `simulate` tools. The default user
interface is known as the **Studio Viewer App**, which can be customized and
extended with **Studio Plugins**.

Under the hood, Studio modernizes the interactive visualization stack using
[Dear ImGui](https://github.com/ocornut/imgui) for UI, Google Filament (PBR with
Vulkan, OpenGL, Metal, and WebGL backends) for rendering, and browser-based
remote streaming out of the box via WebViewer (NetImgui WASM).

A core goal of Studio is to make plugins **composable and reusable across
workflows**:
-   **Environment Authoring**: Teams developing environment frameworks (such as
    Gymnasium or custom robotics suites) can author plugins that everyone
    running an environment can use.
-   **Asset Management**: Engineers building asset tools can write plugins that
    allow anyone to look up, inspect, or load assets in their environment.
-   **Agent Orchestration**: Developers working on agent orchestration can
    create monitoring and control plugins shared by everyone using agents in
    their work.

> [!TIP]
>
> **Related skills:**
>
> -   [mujoco-gui](../gui/SKILL.md) — Dear ImGui agent development skill with
>     best practices for building custom UI plugins across C++ and Python.
> -   [mujoco-python](../python/SKILL.md) — Core simulation lifecycle (`MjSpec` →
>     `MjModel` → `MjData`), named access, and spatial math.

## 1. Studio vs Legacy Simulate

MuJoCo Studio modernizes the interactive visualization stack:

| Feature             | Simulate / `mujoco.viewer` (Legacy)      | Studio / `mujoco.experimental.studio`                                                     |
| :------------------ | :--------------------------------------- | :---------------------------------------------------------------------------------------- |
| **GUI Framework**   | Custom fixed MuJoCo UI (`mjui`)          | **Dear ImGui**                                                                            |
| **Data Plotting**   | Built-in 2D line graphs (`mjr_figure`)   | **ImPlot**                                                                                |
| **Render Backend**  | Classic OpenGL only (`mjrContext`)       | **Filament** (PBR, Vulkan, OpenGL, Metal, WebGL)                                          |
| **Web & Remote**    | None                                     | Web viewer with **NetImgui** (ui/sim/render data streaming over HTTP/WS to a WASM client) |
| **Extensibility**   | Hardcoded UI panels and keybinds         | **Plugin Architecture** in both C++ and Python                                            |
| **Threading**       | Sim/Viewer on separate thread            | Python: Decoupled Sim/Viewer threads; C++ App: Single-threaded                            |
| **Synchronization** | Sim/Viewer share model/data with locking | Python: Message passing (Snapshots & Events); C++: Shared in main loop                    |

## 2. Architecture: Decoupled Sim & Viewer Threads

The Python Studio API introduced in `viewer_protocol.py` and `viewer_handle.py`
replaces the monolithic, synchronous viewer loop with a thread-decoupled,
message-driven plugin architecture:

```text
┌────────────────────────────────────────────────────────┐
│              Simulation Thread (Caller)                │
│  - Owns MjModel and MjData                             │
│  - Runs physics loop, RL policies, or MPC planners     │
│  - Executes SimPlugin instances (e.g. StepControl)     │
│  - Drives simulation via handle.sync()                 │
└────────────────────────────────────────────────────────┘
                        ▲
                        │   Queued Events & Snapshots
                        │ (SimEndpoint <-> ViewerEndpoint)
                        ▼
┌────────────────────────────────────────────────────────┐
│              Viewer Thread (or Web Client)             │
│  - Owns Window, Renderer, Camera, VisOptions, Perturb  │
│  - Owns local copy of MjModel and MjData for rendering │
│  - Executes ViewerPlugin instances (ViewerApp, UI)     │
│  - Dispatches lifecycle events (UpdateEvent, BuildGuiEvent) │
└────────────────────────────────────────────────────────┘
```

-   **Thread / Process Decoupling (`launch_passive`)**: The simulation thread
    and the viewer thread run independently. The simulation communicates with
    the viewer non-blockingly through endpoints (`SimEndpoint ↔ ViewerEndpoint`)
    using queued `Event`s and latest-value `Snapshot`s.
-   **Priority-based Plugin System (`plugin.py`, `plugin_registry.py`)**: State
    and UI behaviors are decoupled into standalone plugin classes decorated with
    `@messages.handler(priority=...)`; `plugin_registry.PluginRegistry` discovers
    the handlers and dispatches messages to them. Launchers accept a single
    `plugins=[...]` argument and partition instances by base class:
    -   **Simulation plugins (`plugin.SimPlugin`)**:
        Subclass `SimPlugin` to run on the simulation thread, automatically
        caching `self.handle` on `SimInitEvent`, and handle simulation-side
        events (e.g. physics reset, step control, perturbations, model
        reloading, policy changes).
    -   **Viewer plugins (`plugin.ViewerPlugin`)**:
        Subclass `ViewerPlugin` to run on the viewer thread, automatically
        caching `self.viewer` on `ViewerInitEvent`, and handle rendering and UI
        lifecycle events (e.g. viewer initialization, model reloading,
        per-frame visual updates, GUI construction).
-   **Standard Samples (`python/mujoco/experimental/studio/`)**:
    -   `python/mujoco/experimental/studio/viewer.py` —
        Standard Studio viewer (`viewer_app.ViewerApp`); supports
        `--model=<file>` and `--plugins=a.py,b.py` (instantiates every
        `ViewerPlugin`/`SimPlugin` defined in those files).
    -   `python/mujoco/experimental/studio/sample/ghost.py` —
        Demonstrates a viewer plugin (`GhostRenderer`) injecting extra geoms via
        `extra_geoms` and custom Dear ImGui controls on `BuildGuiEvent`; run
        with `viewer.py --model=... --plugins=sample/ghost.py`.
    -   `python/mujoco/experimental/studio/sample/implot.py` —
        Demonstrates real-time responsive charting with Dear ImGui and ImPlot
        via `BodyInspector`; run with
        `viewer.py --model=... --plugins=sample/implot.py`.
    -   `python/mujoco/experimental/studio/sample/custom_sim_loop.py` —
        Demonstrates a script that owns its stepping and pacing loop via
        `handle.sync(model, data)`.

## 3. Messaging & Channel Protocol (`messages.py`)

All communication between the simulation and viewer runs over two types of
channels. **Convention:** custom message classes derive from `Snapshot` or
`Event` and end in `*Snapshot` or `*Event` to indicate their delivery semantics.

### Snapshots vs Events

-   **`Snapshot` (Lossy, Latest-Wins)**: Each new snapshot replaces the previous
    unread value of the same type. Ideal for high-frequency continuous state
    where intermediate drops are harmless.
    -   *Canonical example:* `StateSnapshot` — transports the high-frequency
        MuJoCo simulation state vector (`state`, `state_sig`) from simulation to
        viewer.
-   **`Event` (Lossless, Reliable FIFO Queue)**: Guaranteed in-order,
    exactly-once delivery without dropping. Used for discrete actions,
    structural reloads, and user inputs.
    -   *Canonical example:* `ModelEvent` — transports a compiled `MjModel` (and
        optional path) when a model is initially loaded or recompiled.

**Rule of thumb for GUI controls (sliders, checkboxes, settings):**

-   If the control's *complete* value fits in one message and only the latest
    value matters, use a `Snapshot` and send it on change (or every frame);
    latest-wins coalesces redundant updates. Examples: `StepControlSnapshot`
    (pause state, speed, noise), `MjOptionSnapshot` (`mjOption` fields), or a
    plugin's own `FooConfigSnapshot` carrying its slider values.
-   Use an `Event` when distinct edits of the same message type would clobber
    each other in the per-type latest-wins slot and each must be applied —
    `StateEvent` for joint/control slider edits, which differ by `state_sig` —
    and for one-shot requests: `ResetEvent`, `SingleStepEvent`,
    `RequestPauseEvent`.

### Send Only What Changed (`send_if_changes`)

Bidirectional snapshots (`MjOptionSnapshot`, `StepControlSnapshot`) are sent
only by the side that changed the value; if either side sent its full value
every frame, it would overwrite the other side's edits. Both sides use the same
helper, `messages.send_if_changes(send_fn, *make_messages, should_send=...)`:
a context manager that calls each `make_message()` (pass lambdas, so they are
re-read) on entering and on leaving the `with` block and sends the messages
that differ.

-   **Viewer**: `viewer.send_if_changes(...)` wraps the blocks in which the UI
    edits state (`ViewerApp.update()` and `build_gui()`, the Inspector panels).
    Incoming sim snapshots are applied outside those blocks, so they never echo
    back.
-   **Sim**: `ViewerHandle.sync()` wraps the dispatch of `PreStepEvent`,
    `StepEvent` and `PostStepEvent` for `model.opt`, and a plugin that owns
    state the viewer mirrors wraps its own edits in `handle.send_if_changes`
    (`StepControl` does so around `advance()` and `RequestPauseEvent`).
    Incoming viewer snapshots are applied outside those blocks, so they never
    echo back either.

The pattern only sees edits made inside a wrapped block. So edit `model.opt`
from a step-event handler, never from the launcher loop between `sync()` calls,
and change another plugin's state by dispatching an event it handles (e.g.
`RequestPauseEvent` for `StepControl`) rather than by reaching into it.

### Lifecycle Events

Lifecycle events are standard framework events dispatched by the runner loops to
plugins to hook into rendering, UI layout, and physics execution. Each list
below is ordered chronologically from startup through per-iteration execution to
shutdown:

-   **Viewer Lifecycle Events** (dispatched on the viewer side by
    `run_viewer_loop` to `ViewerPlugin` instances):
    1.  `ViewerInitEvent` (`viewer_protocol.ViewerInitEvent`, also exported as
        `plugin.ViewerInitEvent`) — dispatched once on viewer initialization
        (`ViewerPlugin` caches `self.viewer` automatically above `CRITICAL`
        priority before subclass handlers run).
    2.  `ModelEvent` — received when a model is published by the sim; `Viewer`
        deep-copies `viewer.model`, creates `viewer.data`, records
        `viewer.model_path` (`event.path`), and runs `mj_forward`.
    3.  `PostModelEvent` — dispatched locally by `Viewer` immediately after
        `ModelEvent` has initialized `viewer.model` and `viewer.data`. Carries
        `(model, data, path)`: reloading the same `path` preserves viewer UI
        state (camera, pause, and playback speed in `ViewerApp`), while a
        different `path` (or empty) resets it. Subscribe to `PostModelEvent`
        (rather than `ModelEvent`) when resetting viewer-plugin state on model
        changes so `viewer.model` and `viewer.data` are already up to date.
    4.  `UpdateEvent` — dispatched every frame (after incoming sim events and
        snapshots are applied) to update visuals, camera, perturbations,
        transient geoms, and prepare data or upload textures for the GUI.
    5.  `BuildGuiEvent` — dispatched every frame immediately after `UpdateEvent`
        to construct and lay out Dear ImGui / ImPlot widgets (keep handlers
        focused on ImGui/ImPlot calls; prepare GUI data in `UpdateEvent`).
    6.  `ExitEvent` — dispatched once when the viewer shuts down, whether the
        exit came from the sim side, from the window being closed, or from a
        viewer plugin handler raising (the exception is then re-raised on the
        sim side by `ViewerHandle.close()`). Handle it to release resources
        (threads, pools, GPU handles).
-   **Simulation Lifecycle Events** (dispatched on the sim side to
    `SimPlugin` instances):
    1.  `SimInitEvent` (`viewer_handle.SimInitEvent`, also exported as
        `plugin.SimInitEvent`) — dispatched once by `ViewerHandle.__init__`
        (`SimPlugin` caches `self.handle` automatically above `CRITICAL`
        priority before subclass handlers run).
    2.  `LoadModelEvent` — sent from the viewer (on **Reload** or file
        drag-and-drop) or dispatched on the sim thread (e.g. by
        `run(config, model_path=...)`). `ViewerHandle` loads `event.path` (or
        reloads `handle.model_path` when `event.path` is empty) via
        `parser.load` and publishes the compiled model with
        `handle.set_model(model, path=path)`. If loading fails, the error is
        logged and the previous model stays active.
    3.  `ModelEvent` — dispatched when a model is published via
        `handle.set_model()` (or when `handle.sync(model, data)` sees a new
        model object); `ViewerHandle` sets `handle.model`, `handle.data`, and
        `handle.model_path`, runs `mj_forward`, and forwards the model to the
        viewer.
    4.  `PostModelEvent` — dispatched locally by `ViewerHandle` immediately
        after `ModelEvent` has installed `handle.model` and `handle.data` and
        run `mj_forward`. Carries `(model, data, path)`: reloading the same
        `path` preserves `StepControl` pause, speed, and noise settings.
        Subscribe to `PostModelEvent` (rather than `ModelEvent`) when resetting
        sim-plugin state on model changes.
    5.  `PreStepEvent` — dispatched by `ViewerHandle.sync` on every call (after
        incoming viewer events and snapshots are applied) so plugins can prepare
        the simulation before physics advances (e.g. writing actuator controls,
        applying forces, or hot-swapping models via `handle.set_model`).
    6.  `StepEvent` — dispatched by `ViewerHandle.sync` immediately after
        `PreStepEvent` so sim-side stepping plugins (such as `StepControl`) can
        advance the physics.
    7.  `PostStepEvent` — dispatched by `ViewerHandle.sync` immediately after
        `StepEvent` (before `StateSnapshot` is broadcast) so observer/recorder
        plugins (such as `SimHistory`) can inspect or record the resulting
        simulation state.
    8.  `ExitEvent` — dispatched once by `ViewerHandle.close()`, which the
        `with` block calls on every exit path (normal exit, `KeyboardInterrupt`,
        an exception, or the viewer dying). Handle it to release resources.

> [!IMPORTANT]
>
> Do not return `True` from an `ExitEvent` handler: teardown must reach every
> registered plugin, so no handler should consume the event.

-   **Plugin-Dispatched Lifecycle Events**:
    -   `ViewerAppInitEvent` (`viewer_app.ViewerAppInitEvent`) — dispatched by
        the `ViewerApp` viewer plugin on startup so other viewer plugins can
        cache application state and window handles (`event.viewer_app`).

### The `@messages.handler` Decorator

Plugins subclass `ViewerPlugin` (`plugin.ViewerPlugin`) or `SimPlugin`
(`plugin.SimPlugin`) and subscribe to messages by annotating methods with
`@messages.handler`:

```python
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import plugin

class MyPlugin(plugin.ViewerPlugin):
  @messages.handler
  def on_build_gui(self, event: messages.BuildGuiEvent) -> bool | None:
    # Build Dear ImGui elements here (self.viewer is populated automatically)
    return False  # Return True to consume and halt lower-priority handlers
```

#### Priorities (`messages.Priority`)

1.  `CRITICAL` (1000) — Core framework state synchronization and dockspace setup.
2.  `USER` (100, default) — User plugins and custom UI tools.
3.  `LIBRARY` (10) — Standard framework extensions and helpers.
4.  `INTERNAL` (1) — Core framework operations and baseline fallbacks.

##### Pre/Post Lifecycle Events vs. Handler Priorities

There is an intentional distinction between **lifecycle phase events** (`Pre*` /
`Post*`) and **handler priorities** (`messages.Priority`):

-   **Why they can appear to overlap**: Because handlers for a single event run
    in descending `Priority` order, it is tempting to sequence work before or
    after a framework action by subscribing to the main action event and
    fiddling with `priority=...` — for example, subscribing to `StepEvent` with
    a priority higher or lower than `StepControl` (`Priority.INTERNAL`) to run
    code before or after physics stepping, or subscribing to `ModelEvent` with a
    custom priority to run before or after `ViewerHandle` / `Viewer` swaps the
    model.
-   **Why you should prefer `Pre*` / `Post*` events instead**: Using priorities
    to order distinct lifecycle stages couples your plugin to internal priority
    choices of other plugins and can expose stale references (for instance, a
    high-priority `ModelEvent` handler runs *before* `ViewerHandle` or `Viewer`
    has created the new `MjData` and run `mj_forward`, and a `StepEvent` handler
    that calls `handle.set_model()` mid-dispatch leaves the in-flight
    `StepEvent` holding the old `model` and `data`).
-   **Rule of thumb**: Always choose the lifecycle event that matches the phase
    you want and use a bare `@messages.handler` decorator:
    -   **Reacting to a model load**: After `ModelEvent` installs the new model
        and runs `mj_forward`, subscribe to `PostModelEvent` so `model` and
        `data` are guaranteed to be initialized and forwarded.
    -   **Before / during / after a physics step**: Subscribe to `PreStepEvent`
        (to write controls, apply forces, or hot-swap models via
        `handle.set_model`), `StepEvent` (only when your plugin is itself the
        physics stepper), or `PostStepEvent` (to observe or record the stepped
        state).
    -   **Before / during GUI construction**: Subscribe to `UpdateEvent` for
        per-frame state updates, GUI data preparation, and texture uploads
        (e.g. `viewer.upload_image`), and `BuildGuiEvent` solely for Dear ImGui
        / ImPlot widget submission.
-   **When to use `messages.Priority`**: Reserve `priority=...` strictly for
    cases where multiple handlers target the **exact same event and phase** and
    one handler needs to override or consume (`return True`) the event before a
    fallback handler runs.

> [!IMPORTANT]
>
> If a handler returns `True`, the message is marked as consumed and **will not
> be delivered** to lower-priority handlers. Return `False` or `None` if
> downstream plugins or default handlers also need the message.

## 4. Python Runners: `launch_web`, `launch_native` & `launch_passive`

Studio provides three runner modules that launch the viewer and execute
the simulation sync loop via `.run(...)`:

1.  **`launch_web.run`**
    (`python/mujoco/experimental/studio/launch_web.py`):
    Spawns a WebViewer in a background daemon thread and runs the simulation
    loop. **This is usually the most practical choice.** Its major advantage is
    that the host Python simulation script has **no native graphics
    dependencies** (e.g., no desktop display server, X11/Wayland, or Filament
    libraries required on the host): the host executes physics and computes Dear
    ImGui layout in a headless context, while streaming draw commands (via
    NetImgui) and state snapshots over WebSockets to a WebAssembly client
    running in the user's browser. This makes it ideal for headless servers,
    containers, and remote development.
2.  **`launch_native.run`**
    (`python/mujoco/experimental/studio/launch_native.py`):
    Spawns a local desktop window with native Filament graphics (Vulkan or
    OpenGL) and runs the simulation loop.
3.  **`launch_passive.run`**
    (`python/mujoco/experimental/studio/launch_passive.py`):
    A unified wrapper that inspects `config.gfx` and automatically delegates to
    `launch_web` (if `config.gfx in ('web', 'webgl')`) or `launch_native`.

Unless you specifically need your application to switch dynamically between
native desktop and web streaming, **use just one of these directly**. In most
workflows, **`launch_web.run` is recommended**.

For a full reference implementation, see
`python/mujoco/experimental/studio/viewer.py`.

### Standard Blocking Runners

All three modules share the same `.run(...)` interface, accepting either
`model_path=...` (loaded on the sim thread via `LoadModelEvent`, which sets
All three modules share the same `.run(...)` interface, accepting either
`model_path=...` (loaded on the sim thread by `ViewerHandle` via
`LoadModelEvent`, which sets `handle.model_path` so **Reload** works) or an
in-memory `model=...` (with optional `data=...`, published via
`handle.set_model`). `viewer_app.SIM_PLUGINS` bundles the default sim-side
plugins that back `ViewerApp` (`step_control.StepControl()` and
`sim_history.SimHistory()`):

```python
from mujoco.experimental.studio import launch_web  # Or launch_native / launch_passive
from mujoco.experimental.studio import viewer_app
from mujoco.experimental.studio import viewer_protocol

config = viewer_protocol.ViewerConfig(title='Studio')

launch_web.run(
    config,
    model_path=model_path,  # Or model=model, data=data for an in-memory model.
    plugins=[
        viewer_app.ViewerApp(),
        *viewer_app.SIM_PLUGINS,
    ],
)
```

> [!WARNING]
>
> **Stepping and pacing are plugin responsibilities.** `viewer_app.SIM_PLUGINS`
> includes `step_control.StepControl()` (real-time-paced CPU stepping) and
> `sim_history.SimHistory()` (history scrubbing). Pass individual plugins
> instead when replacing `StepControl()` (for example, when running GPU rollouts
> or custom RL loops): without a `StepEvent` plugin, `handle.sync()` will **not
> advance physics or sleep**, so your stepping plugin must manage its own pacing
> to avoid busy-spinning.

### Advanced: Non-Blocking Launchers (`launch_web.launch`, `launch_native.launch`, `launch_passive.launch`)

When attaching a viewer to an **external loop that you do not want to invert into
a `StepEvent` plugin** (such as an existing RL training loop, trajectory replay
script, or unit test), each module also provides a non-blocking `.launch(...)`
context manager returning a `ViewerHandle`:

```python
with launch_web.launch(
    config,
    plugins=[
        viewer_app.ViewerApp(),
        *viewer_app.SIM_PLUGINS,
    ],
) as handle:
  while handle.is_running():
    # Do NOT edit model, data or plugin state here; do it in step-event
    # handlers, or sync() cannot send the change to the viewer (see §8.2).
    model, data = handle.sync(model, data)
```

To step and pace physics directly in the loop instead, omit `StepControl()` and
step after `handle.sync(model, data)` (see
`python/mujoco/experimental/studio/sample/custom_sim_loop.py`).

## 5. Authoring Python Plugins

Explore `python/mujoco/experimental/studio/sample/`
for actively maintained reference implementations.

### Key Plugin Patterns

-   **Viewer UI Plugins (ImGui & ImPlot)**:
    See `python/mujoco/experimental/studio/sample/implot.py`
    for a complete implementation of `BodyInspector`, demonstrating how to
    receive `ViewerAppInitEvent` to cache app references, draw responsive ImPlot
    charts on `BuildGuiEvent`, and use persistent window IDs (`###`) to preserve
    docking and window state. Implement plugins as one or two ImGui windows so
    they can dock directly into the Studio Viewer App dockspaces. (See the
    [`mujoco-gui`](../gui/SKILL.md) skill for Studio GUI plugin best practices
    across C++ and Python using ImGui and extensions like ImPlot.)

-   **Visual Overlays & Geoms**:
    See `python/mujoco/experimental/studio/sample/ghost.py`
    for `GhostRenderer`, demonstrating how to subscribe to `UpdateEvent` and
    populate `viewer.extra_geoms` with transient `mjvGeom` instances without
    modifying the compiled model.

> [!WARNING]
>
> **Upcoming API Migration for Extra Geoms:**
> The `viewer.extra_geoms` mechanism in `sample/ghost.py` is an interim solution
> and will soon be replaced by the direct scene/geom API from
> `python/mujoco/rendering/filament/renderer.py`.
> Prefer referencing that upcoming renderer API when architecting new custom
> visual overlays.

-   **Split Sim/Viewer Architecture (Spec Editing & Asset Insertion)**:

> [!WARNING]
>
> **Split Sim/Viewer Plugin Architecture (Asset Browser Example):**
> Avoid monolithic designs where viewer-side UI directly accesses or mutates
> mutable model specifications (`MjSpec`).
>
> In an interactive asset browser plugin, responsibilities should be cleanly
> split:
>
> 1.  **`AssetBrowserGui` (viewer plugin)**: Renders the asset browser UI and
>     emits a custom event (e.g. `AssetDropEvent`) carrying asset data and
>     target locations.
> 2.  **`AssetBrowserSim` (sim plugin)**: Receives `AssetDropEvent` on the
>     simulation side (which owns the `MjSpec`), performs spec attachment,
>     recompiles the model, and publishes it via
>     `handle.set_model(new_model, path=handle.model_path)` (passing
>     `path=handle.model_path` preserves camera and pause state across in-place
>     scene edits).

### Hot Reload (Python only, opt-in)

With hot reload enabled (`viewer.py --hot_reload=all`, or
`viewer_protocol.ViewerConfig(hot_reload='all')`; off by default), saving a
plugin's `.py` file or the watched model file updates the running viewer on the
next frame or step without restarting, keeping the browser connection, camera,
UI layout, and plugin state. The value says what to reload: `all` (plugin
modules and the model file), `plugins`, `model`, or comma-separated plugin
class names such as `Scene,SceneGui` (combinable, e.g. `model,Scene`); see
`hot_reload.Options.parse`.

See the module docstring of
`python/mujoco/experimental/studio/hot_reload.py` for the full design and usage
guide, including:

-   **Reloadable vs. fixed boundary:** registered plugin modules (user and
    built-in plugins such as `ViewerApp`, `StepControl`, and `SimHistory`),
    same-directory helper modules, and the active XML/URDF model file (with its
    `<include>`s and assets) reload on save; framework modules, `__main__`, the
    C++ binary, and the set of registered plugin instances are fixed. Register
    the viewer/sim plugins you expect to grow up front, and compose separate
    plugins rather than subclassing built-in plugins.
-   **Preserving and reconciling state (`HotReloadEvent`):** declare default
    attribute values on the class (e.g. `height: int = 5`) so mutated attributes
    on `self` survive class swaps while newly added class attributes appear
    without re-running `__init__`, and handle `messages.HotReloadEvent` (`if
    self in event.plugins:`) to initialize new mutable buffers, invalidate
    cached IDs, migrate reshaped state, or trigger immediate rebuilds.
-   **Error recovery:** syntax/import/model errors keep the previous code and
    model and retry on the next save; exceptions raised by a swapped plugin's
    handlers are caught and logged once per handler until the next reload.

## 6. C++ Studio & Plugin Architecture

In C++, MuJoCo Studio is structured around `mujoco::studio::App` and launched
via `mujoco::studio::LaunchStudio`.

> [!NOTE]
>
> **Architecture Convergence Note:** Unlike the Python Studio API, the current
> C++ Studio application is single-threaded (the main loop runs UI, rendering,
> and physics sequentially on a single thread). The Python and C++ plugin
> architectures will converge in future updates to share a unified
> message-driven and lifecycle design.

### C++ Plugin Registry (`mujoco::platform::RegisterPlugin`)

C++ extensions define plugins using structs declared in
`src/experimental/studio/ux/plugin.h`:

| Plugin Struct          | Role                                                 | Key Callbacks                                             |
| :--------------------- | :--------------------------------------------------- | :-------------------------------------------------------- |
| **`GuiPlugin`**        | Creates an ImGui window listed in the "Plugins" menu | `update(GuiPlugin* self)`                                 |
| **`ScenePlugin`**      | Adds custom `mjvGeom` elements to the `mjvScene`     | `enhance_scene(self, model, data, scene)`                 |
| **`ModelPlugin`**      | Hooks into model loading and physics updates         | `get_model_to_load`, `pre_step`, `post_step`, `do_update` |
| **`KeyHandlerPlugin`** | Binds keyboard shortcuts via ImGui key chords        | `key_chord`, `on_key_pressed(self)`                       |
| **`SpecEditorPlugin`** | Edits `mjSpec` with pre/post-recompile triggers      | `pre_compile(self, spec, m, d, cam)`, `post_compile`      |

### Registering and Launching in C++

Plugins can be registered in two ways:

1.  **Automatic Self-Registration (`mjPLUGIN_LIB_INIT`)**: Plugins can be
    authored in self-contained translation units using the `mjPLUGIN_LIB_INIT`
    macro (from `#include <mujoco/mjplugin.h>`). This defines a static
    constructor that calls `mujoco::platform::RegisterPlugin` on library load
    before `main()` runs. Simply linking the plugin's library into the Studio
    binary registers the plugin without having to modify `main()`.

```cpp
#include "imgui.h"
#include <mujoco/mjplugin.h>
#include "src/experimental/studio/ux/plugin.h"

namespace {

void MyGuiUpdate(mujoco::platform::GuiPlugin* self) {
  ImGui::TextUnformatted("Custom Tool Window");
}

}  // namespace

mjPLUGIN_LIB_INIT(my_tool_plugin) {
  mujoco::platform::GuiPlugin gui;
  gui.name = "My Tool";
  gui.active = true;
  gui.update = MyGuiUpdate;
  mujoco::platform::RegisterPlugin(gui);
}
```

2.  **Explicit Registration in Custom `main()`**: When writing a standalone
    application or custom launcher entrypoint, plugins can also be registered
    explicitly before calling `LaunchStudio`:

```cpp
#include "absl/flags/parse.h"
#include "imgui.h"
#include "src/experimental/studio/launcher.h"
#include "src/experimental/studio/ux/plugin.h"

void MyGuiUpdate(mujoco::platform::GuiPlugin* self) {
  ImGui::TextUnformatted("Custom Tool Window");
}

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  // Register plugins explicitly before LaunchStudio
  mujoco::platform::GuiPlugin gui;
  gui.name = "My Tool";
  gui.active = true;
  gui.update = MyGuiUpdate;
  mujoco::platform::RegisterPlugin(gui);

  mujoco::studio::LauncherConfig config;
  config.title = "My Project Studio";
  config.model_file = "path/to/robot.xml";

  return mujoco::studio::LaunchStudio(argc, argv, config);
}
```

## 7. Graphics Backends & WebViewer Streaming

Graphics backends are configured via `ViewerConfig.gfx` using values from the
`GFX_MODES` tuple in `viewer_protocol.py` (matching `platform::GraphicsMode` in
C++):

-   **`NativeViewer` modes**: `opengl` (default Filament OpenGL), `vulkan`
    (Filament Vulkan), `classic` (legacy OpenGL), and software/headless modes
    (`classic_headless`, `opengl_headless`, `opengl_software`,
    `vulkan_software`).
-   **`WebViewer` modes**: `web` / `webgl`.

> [!TIP]
>
> **Decoupling Visualization Dependencies with WebViewer:**
> Users running simulations on remote workstations, cloud VMs, or Docker
> containers may prefer launching via `WebViewer` (`gfx='web'`). This completely
> prevents the simulation binary from linking or loading heavy graphical and
> windowing dependencies (Filament, SDL2, Vulkan, X11): the host only executes
> physics and a headless Dear ImGui context, while all 3D scene rendering and
> GPU rasterization happen inside the client's web browser via WebAssembly
> (WASM).

### How WebViewer Works

```text
┌────────────────────────────────────────────────────────┐
│              Simulation Thread (Host)                  │
│  - Runs physics loop & SimPlugin instances             │
│  - Pushes state snapshots & receives events            │
└────────────────────────────────────────────────────────┘
                         ▲ Endpoints (IPC / Queues)
                         │ (SimEndpoint ↔ ViewerEndpoint)
                         ▼
┌────────────────────────────────────────────────────────┐
│              WebViewer Thread (Host Server)            │
│  - Runs ViewerPlugin instances in headless Dear ImGui  │
│  - Serves single HTTP port (default 8080)              │
│  - Streams UI via NetImgui (/ui) & state (/state)      │
└────────────────────────────────────────────────────────┘
                         ▲ WebSocket (HTTP / Port 8080)
                         │ Streamed draw commands & input
                         ▼
┌────────────────────────────────────────────────────────┐
│              Browser Client (WASM)                     │
│  - WebAssembly client running in user's browser        │
│  - Renders 3D scene with Filament WebGL / WebGPU       │
│  - Overlays streamed ImGui draw commands               │
│  - Forwards user mouse & keyboard events to server     │
└────────────────────────────────────────────────────────┘
```

-   **Independence from Sim/Viewer Messages**: The data stream from the viewer
    thread to the browser is completely independent of the messages, events, and
    snapshots passed between the simulation and viewer threads.
-   **Stream Separation**:
    -   **GUI Stream**: ImGui draw commands and user interaction flow back and
        forth through the NetImgui protocol over WebSocket (`/ui`).
    -   **3D Scene Stream**: Rendering state (`/state`) is transmitted primarily
        via the compiled binary `.mjb` for the model and the raw MuJoCo state,
        accompanied by a small payload of extra data covering visual elements
        not represented in model/state (such as camera transforms, perturb
        forces, and transient geoms).
-   **Single Port**: WebViewer serves everything through one port (default
    `8080`): the HTML/WASM client application, the `/ui` NetImgui stream, and
    the `/state` WebSocket stream.
-   **No Display Forwarding**: No need for X11 forwarding, VNC, or remote
    desktop setups.

> [!WARNING]
>
> **Trust Model (Local & Trusted Networks Only):**
> `WebViewer` has no authentication or encryption (plain HTTP and WebSockets)
> and listens on all network interfaces, so anyone who can reach its port can
> connect. Connected browsers can see host-side information such as model file
> paths, exception messages, and logs. Use it locally, on trusted networks, or
> through an authenticated tunnel, and do not expose the port to untrusted
> networks. The startup banner repeats this as a short warning.

## 8. Common Gotchas & Best Practices

### 1. Explicit Message Passing & Thread Boundaries (Python)

In Python, MuJoCo Studio uses a strictly decoupled, multi-threaded architecture
(whereas the C++ Studio app is single-threaded). The simulation thread and the
viewer thread communicate exclusively via **thread-safe message channels**
(`Snapshot` and `Event`, via `handle.send_to_viewer(...)` and
`viewer.send_to_sim(...)`). Direct concurrent access causes data races and
rendering artifacts.

As a direct consequence of this principle:
-   The simulation thread should **never directly inspect or touch viewer data**
    (such as `viewer.camera`, `viewer.vis_options`, or viewer-side model
    copies).
-   The viewer thread should **never directly mutate simulation-side state**
    outside of handling inbound events and sending requests to the sim.

> [!IMPORTANT]
>
> **Keep `ViewerHandle` minimal.** It is the framework's sim-side orchestrator
> and should only own `sync()`, message dispatch, and model/data lifecycle.
> Domain-specific sim-side behaviours belong in **`SimPlugin` subclasses** — for
> example `StepControl` (stepping/pacing) and `SimHistory` (history,
> scrubbing). Do not add `ux` or `sim` imports to `viewer_handle.py`.

### 2. Preserve Simulation Loop Boilerplate (Use Plugins & Events)

The standard simulation runner loop (`viewer_handle.run_sim_loop(handle)`):

```python
while handle.is_running():
  handle.sync()
```

is standardized boilerplate used across all Studio launchers (`launch_web`,
`launch_native`, `launch_passive`), with `ViewerHandle` owning the active
`handle.model`, `handle.data`, and `handle.model_path`. Do not insert
domain-specific simulation logic, direct physics stepping, or ad-hoc state
mutations into a runner loop. If your application needs custom behavior (e.g.
custom stepping, perturbation forces, sensor logging, or RL resets):
-   **Simulation Plugins**: Encapsulate the behavior in a sim-side plugin
    subscribing to `SimInitEvent`, `PostModelEvent`, `PreStepEvent`,
    `StepEvent`, `PostStepEvent`, or custom messages.
-   **Event Dispatch**: Dispatch an `Event` to the simulation thread rather than
    mutating state directly in the outer loop.
-   **Edit `model` and `data` only from step-event handlers**: `sync()` sends
    `model.opt` when it changed across its `PreStepEvent` / `StepEvent` /
    `PostStepEvent` dispatch (see
    [Send Only What Changed](#send-only-what-changed-send_if_changes)), so an
    `opt` edit made outside those handlers is never sent and the viewer's next
    snapshot overwrites it; other model fields are not synced at all, so publish
    the model again with `handle.set_model()` after editing them. `data` is sent
    after every step, but `sync()` applies the viewer's resets, slider edits
    and perturbations before `PreStepEvent`, so controls or forces written in
    an outer loop can be overwritten in the same `sync()`.
-   **Custom Loops Owning `(model, data)`**: Must reassign
    `model, data = handle.sync(model, data)`. When a script uses non-blocking
    `.launch(...)` and owns its own `model` and `data` (as in
    `sample/custom_sim_loop.py`), pass them to `handle.sync(model, data)` and
    always rebind the returned `(model, data)` tuple. When a `LoadModelEvent`
    (e.g. a dropped file or **Reload**) or a plugin's `handle.set_model()`
    replaces the model, `handle.sync()` returns the newly instantiated `model`
    and `data`; failing to capture the return values leaves the caller loop
    stepping a stale physics model while the viewer renders the new one.

### 3. NumPy Views vs Copies in Events

NumPy arrays returned by MuJoCo bindings are views into mutable C buffers. If
your plugin caches history across timesteps (e.g., position trajectories, Euler
angles), **always call `.copy()`**:

```python
# ❌ WRONG — reference updates in place, all history items point to latest state
self.history.append(app.data.xpos[body_id])

# ✅ RIGHT — stores a distinct copy
self.history.append(app.data.xpos[body_id].copy())
```

### 4. Stepping Pacing & `NullSimulation` for Pure Viewers

If you do not pass `step_control.StepControl()` to `plugins`,
`handle.sync()` will not pace itself against real wall-clock time. A tight
`while handle.is_running(): handle.sync(...)` loop without a stepping plugin
will spin at 100% CPU. Ensure you either include `StepControl()` or insert a
sleep/pacing mechanism.

To build a **pure viewer** (e.g., an interactive mesh browser, asset inspector,
or model visualizer where physics is not simulated), implement a
`NullSimulation` plugin that handles pacing (or sleeps/throttles each iteration)
without calling `mj_step`.

### 5. Returning Values from Handlers

-   Return `True` only when your handler has fully satisfied the event and wants
    to **block all remaining handlers** from seeing it.
-   Return `False` or `None` if the event should continue propagating down the
    priority chain.

### 6. UI Guidelines

When authoring Studio plugin user interfaces in Dear ImGui and extensions like
ImPlot (in either C++ or Python), refer to the [`mujoco-gui`](../gui/SKILL.md)
skill for comprehensive architecture, styling, and design best practices,
including:

-   **Window Granularity & Docking**: It is strongly suggested that viewer GUI
    plugins are implemented as **one or two ImGui windows** (e.g.
    `imgui.Begin(...)`). This ensures plugin panels can be cleanly docked into
    the Studio Viewer App dockspaces alongside standard built-in tools.
-   **Common Widget Functionality (`imgui_widgets.h`/`cc`)**: Useful, shared
    custom ImGui widget functionality lives in the `imgui_widgets.h`/`cc`
    files in `src/experimental/studio/ux/`.
    > [!NOTE]
    > **Future Shared Widgets:** Consider introducing a similar shared widgets
    > module for widgets authored by Python users under
    > `python/mujoco/experimental/studio/`.
-   **Python vs C++ Widgets (GIL, Performance & Portability)**: When creating a
    widget in Python, consider whether you actually need the GIL:
    -   **Releasing the GIL**: If the widget does not depend on Python-specific
        runtime state, consider authoring it as a native C++ widget with a
        Python binding so the GIL can be released during execution.
    -   **Maximum Performance**: Native C++ widgets avoid per-frame Python
        interpreter and binding overhead, which may be important for widgets
        doing intensive computation.
    -   **JavaScript & Web Portability**: Native C++ widgets can be compiled to
        WebAssembly (via Emscripten) and bound to JavaScript for client-side
        browser environments like **MuJoCo Live**. Note the architectural
        distinction with **WebViewer**: WebViewer executes plugins on the server
        and primarily streams the resulting ImGui draw lists to the browser.
        While the WebViewer's WASM browser client (see the `web/` folder) does
        run a small amount of local C++ ImGui code for client-side chrome (such
        as connection banners and disconnect dialogs), rich plugin widgets
        rarely need to run inside the WebViewer's browser client itself.
-   **Naming Conventions**: Use `build_gui()` or `build_*_gui()` for layout and
    widget submission; reserve `render` / `draw` for low-level graphics
    rasterization.
-   **Decoupling GUI from Application Logic**: Keep UI layout code strictly
    focused on presentation and input routing. Run simulation steps, heavy
    computation, and I/O asynchronously or on the simulation side.
-   **ID Management (`##` vs `###`)**: Use `##` for disambiguating duplicate
    labels without changing displayed text, and `###` to maintain persistent IDs
    when labels contain dynamic text (preserving window docking, focus, and
    collapse states) or to assign semantic names for test automation.
-   **List Virtualization (`ImGuiListClipper`)**: Always virtualize large lists
    and tables to avoid saturating WebSocket bandwidth in WebViewer / NetImgui.
-   **Test Engine Readiness**: Design immediate-mode controls to be discoverable
    and automatable by Dear ImGui Test Engine drivers.

### 7. Avoiding WebViewer WebSocket Timeouts ("Server Not Reachable")

In WebViewer mode (`launch_web`), the host HTTP/WebSocket server and browser
client maintain continuous heartbeat ping-pongs over `/ui` and `/state`.
-   **Never Block the Viewer Thread**: If a viewer plugin performs heavy
    synchronous operations (such as synchronous network downloads, blocking disk
    I/O, or CPU-intensive spec compiling), the server event loop stalls.
-   **Consequence**: Heartbeat packets drop, causing the browser client to
    assume the server died and display a `DISCONNECTED` / "viewer server not
    reachable" overlay banner.
-   **Remedy**: Always offload asset loading or heavy computations to background
    threads, and dispatch completed assets or specs to the simulation side via
    asynchronous `Event` messages.

### 8. Run with `-c opt` for Best Performance

Always build and run Studio targets with `-c opt` so physics stepping, Filament
rendering, and UI streaming execute with compiler optimizations enabled.

### 9. Debugging: Terminal, Studio Console & Browser Console

Host and browser logs end up in different places:

1.  **Host logs (Python and the MuJoCo engine)**: Once a `ViewerApp` exists
    (and in the C++ Studio app), MuJoCo log messages (`mju_warning`,
    `mju_message`, ...) and Python `logging` records, `warnings.warn` calls,
    and uncaught exceptions from any thread are added to the Studio Console
    (`ux.console_gui` / `Console::WindowGui`). Each hook still passes messages
    to the existing handlers (falling back to `logging.lastResort` when no other
    Python handler is configured), so errors and warnings still appear in the
    terminal. The status bar shows the message, warning, and error counts;
    click it to toggle the Console. Right-click the Console to clear it, copy
    all lines, or change the engine's log config (`mju_setLogConfig`): whether
    it prints to the terminal, writes `MUJOCO_LOG.TXT`, and which info topics
    are on; the Console itself captures regardless. Tests and scripts can read
    the Console with `ux.get_console_lines()` and `ux.get_console_counts()`.
2.  **Browser logs (WebViewer rendering, WASM, or WebSocket issues)**: These are
    only in the browser console. Read them with the `chrome-devtools` MCP
    (`list_console_messages`) if available, or run headless Chrome with
    `--remote-debugging-port=9222` and subscribe to the DevTools Protocol's
    `Runtime.consoleAPICalled` and `Runtime.exceptionThrown` events.
