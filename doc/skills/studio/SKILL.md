---
name: mujoco-studio
description: >-
  Builds and launches interactive MuJoCo Studio viewers and plugins in Python
  and C++, covering decoupled sim/viewer threads, Snapshot and Event messaging,
  lifecycle events, WebViewer browser streaming, and Python hot reload. Use when
  launching Studio viewers (launch_web, launch_native, launch_passive,
  LaunchStudio), authoring Python ViewerPlugin/SimPlugin or C++ RegisterPlugin
  (mjPLUGIN_LIB_INIT) extensions, synchronizing sim and viewer state
  (ViewerHandle.sync, @messages.handler), hot-reloading Python plugins or models
  (HotReloadEvent), configuring graphics modes or WebViewer NetImgui/WASM
  streaming, or debugging Studio and browser console logs. Don't use for Dear
  ImGui/ImPlot widget layout or styling (use mujoco-gui), core physics stepping
  (use mujoco-python), or headless offscreen rendering (use mujoco-rendering).
---

# MuJoCo Studio: Interactive Simulation, UI & Plugin Architecture

Studio replaces the legacy `simulate` tools with a modular interactive
visualization stack built on [Dear ImGui](https://github.com/ocornut/imgui) and
ImPlot for UI, Google Filament (PBR with Vulkan, OpenGL, Metal, and WebGL
backends) for rendering, and out-of-the-box browser streaming via `WebViewer`
(NetImgui WASM). Its default interface, the **Studio Viewer App**, is designed
around composable **Studio Plugins** that teams can share across environment
authoring (e.g., Gymnasium or robotics suites), asset inspection/loading tools,
and agent monitoring/control workflows.

> [!TIP]
>
> **Related skills:**
>
> -   [mujoco-gui](../gui/SKILL.md) — Dear ImGui and ImPlot widget layout,
>     Tables API alignment, `##` vs `###` IDs, and `ImGuiListClipper`.
> -   [mujoco-python](../python/SKILL.md) — Core simulation lifecycle
>     (`MjSpec` -> `MjModel` -> `MjData`), named access, and spatial math.
> -   [mujoco-spec-editing](../spec_editing/SKILL.md) — Procedural `MjSpec`
>     modeling, attachments, and runtime recompilation.
> -   [mujoco-rendering](../rendering/SKILL.md) — Headless offscreen RGB, depth,
>     and segmentation rendering.

## Reference Guides (Load on Demand)

Read the matching reference file when your task touches these areas:

-   **[references/messaging_and_lifecycle.md](references/messaging_and_lifecycle.md)** —
    Read when designing custom `*Snapshot` or `*Event` messages, synchronizing
    bidirectional state with `messages.send_if_changes`, looking up the full
    chronological Viewer and Simulation lifecycle event sequences, or setting
    `messages.Priority` overrides.
-   **[references/python_plugins_and_hot_reload.md](references/python_plugins_and_hot_reload.md)** —
    Read when authoring Python `ViewerPlugin` or `SimPlugin` extensions (ImGui
    panels, `extra_geoms` overlays, split Sim/Viewer `MjSpec` editors, Python vs
    C++ widget trade-offs) or enabling Python plugin/model hot reload
    (`hot_reload`, `HotReloadEvent`).
-   **[references/cpp_studio.md](references/cpp_studio.md)** — Read when
    building C++ Studio applications (`mujoco::studio::App`, `LaunchStudio`) or
    registering C++ plugins (`mujoco::studio::RegisterPlugin`,
    `mjPLUGIN_LIB_INIT`).
-   **[references/web_viewer_and_debugging.md](references/web_viewer_and_debugging.md)** —
    Read when configuring `ViewerConfig.gfx` graphics backends, understanding or
    troubleshooting `WebViewer` NetImgui/WASM streaming and WebSocket timeouts,
    or reading Studio host and browser console logs.

## 1. Architecture & Threading Model

### Studio vs Legacy `simulate`

| Feature | Legacy `simulate` / `mujoco.viewer` | Studio (`mujoco.experimental.studio`) |
| :--- | :--- | :--- |
| **GUI & Plotting** | Fixed `mjui` panels & `mjr_figure` | **Dear ImGui** & **ImPlot** |
| **Render Backend** | Classic OpenGL (`mjrContext`) | **Filament** PBR (Vulkan, OpenGL, Metal, WebGL) |
| **Web & Remote** | None | **WebViewer** (NetImgui UI + state over HTTP/WS to WASM) |
| **Extensibility** | Hardcoded panels and keybinds | **Plugin Architecture** in Python and C++ |
| **Threading & Sync** | Shared `model`/`data` with locking | Python: decoupled threads + messages; C++: single-threaded main loop |

### Decoupled Sim & Viewer Threads (Python)

`viewer_protocol.py` and `viewer_handle.py` decouple simulation execution from
rendering and UI through non-blocking message endpoints:

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

-   **Priority-based Plugin System (`plugin.py`, `plugin_registry.py`)**:
    `plugin_registry.PluginRegistry` discovers `@messages.handler` methods and
    dispatches messages in priority order. Launchers accept a single
    `plugins=[...]` list and partition instances by base class:
    -   **`plugin.SimPlugin`**: Runs on the simulation thread, automatically
        caches `self.handle` on `SimInitEvent`, and handles physics stepping,
        resets, perturbations, model reloads, and policy updates.
    -   **`plugin.ViewerPlugin`**: Runs on the viewer thread, automatically
        caches `self.viewer` on `ViewerInitEvent`, and handles visual updates,
        rendering overlays, and Dear ImGui construction.
-   **Canonical Samples (`python/mujoco/experimental/studio/`)**:
    -   `python/mujoco/experimental/studio/viewer.py` — Standard Studio viewer
        (`viewer_app.ViewerApp`); run with
        `viewer.py --model=<file> --plugins=a.py,b.py` (instantiates every
        `ViewerPlugin`/`SimPlugin` defined in those files).
    -   `sample/ghost.py` — `GhostRenderer` injecting transient `extra_geoms` on
        `UpdateEvent` and ImGui controls on `BuildGuiEvent`
        (`viewer.py --model=... --plugins=sample/ghost.py`).
    -   `sample/implot.py` — `BodyInspector` demonstrating real-time ImGui +
        ImPlot charting (`viewer.py --model=... --plugins=sample/implot.py`).
    -   `sample/custom_sim_loop.py` — External script owning its stepping and
        pacing loop via `handle.sync(model, data)`.

## 2. Launching Python Viewers

Studio provides three runner modules sharing the same `.run(...)` and
`.launch(...)` interfaces:

1.  **`launch_web.run`** (`python/mujoco/experimental/studio/launch_web.py`) —
    **Recommended default.** Spawns `WebViewer` in a background daemon thread
    with **zero host graphics dependencies** (no X11/Wayland display server or
    host Filament libraries required): physics and headless Dear ImGui layout
    run on the host while draw commands (NetImgui) and state stream over
    WebSockets to a browser WASM client. Ideal for remote workstations,
    containers, and headless servers.
2.  **`launch_native.run`**
    (`python/mujoco/experimental/studio/launch_native.py`) — Opens a local
    desktop window with native Filament graphics (Vulkan or OpenGL).
3.  **`launch_passive.run`**
    (`python/mujoco/experimental/studio/launch_passive.py`) — Inspects
    `config.gfx` and delegates to `launch_web`
    (`config.gfx in ('web', 'webgl')`) or `launch_native`. Use `launch_web` or
    `launch_native` directly unless you need runtime switching via `config.gfx`.

### Blocking Runner (`.run`)

Accepts either `model_path=...` (loaded on the sim thread by `ViewerHandle` via
`LoadModelEvent`, setting `handle.model_path` so **Reload** works) or an
in-memory `model=...` (with optional `data=...`, published via
`handle.set_model`). `viewer_app.SIM_PLUGINS` bundles
`step_control.StepControl()` and `sim_history.SimHistory()`:

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
> `sim_history.SimHistory()` (history scrubbing). Without `StepControl()` (or
> your own `StepEvent` plugin), `handle.sync()` does **not** advance physics or
> sleep—a tight `while handle.is_running(): handle.sync(...)` loop will spin at
> 100% CPU. When replacing `StepControl()` (e.g., for GPU rollouts, custom RL
> loops, or a pure non-physics mesh/asset viewer via a `NullSimulation` plugin
> that throttles each iteration without calling `mj_step`), pass individual
> plugins and manage pacing explicitly.

### Non-Blocking Launchers (`launch_web.launch`, `launch_native.launch`, `launch_passive.launch`)

Use `.launch(...)` only when attaching a viewer to an external loop that you do
not want to invert into a `StepEvent` plugin (e.g., an existing RL training
loop, trajectory replay script, or test):

```python
with launch_web.launch(
    config,
    plugins=[
        viewer_app.ViewerApp(),
        *viewer_app.SIM_PLUGINS,
    ],
) as handle:
  while handle.is_running():
    # Mutate model.opt or data in step-event handlers, NOT here (see §4.2).
    model, data = handle.sync(model, data)
```

To step and pace physics directly in the caller loop instead, omit
`StepControl()` and step after `model, data = handle.sync(model, data)` (see
`python/mujoco/experimental/studio/sample/custom_sim_loop.py`).

## 3. Plugin & Messaging Essentials

All cross-thread communication in `messages.py` uses two channel base classes
(custom classes end in `*Snapshot` or `*Event` by convention):

-   **`Snapshot` (Lossy, Latest-Wins)**: Replaces any unread previous value of
    the same type; use for continuous state or complete control configs where
    only the latest value matters (e.g., `StateSnapshot`, `StepControlSnapshot`,
    `MjOptionSnapshot`).
-   **`Event` (Lossless FIFO Queue)**: Guaranteed in-order, exactly-once
    delivery; use for discrete actions, reloads, and distinct edits that must
    not clobber one another (e.g., `ModelEvent`, `StateEvent`, `ResetEvent`,
    `SingleStepEvent`, `RequestPauseEvent`).

Plugins subscribe to messages via `@messages.handler`:

```python
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import plugin

class MyPlugin(plugin.ViewerPlugin):
  @messages.handler
  def on_build_gui(self, event: messages.BuildGuiEvent) -> bool | None:
    # Build Dear ImGui widgets here (self.viewer is cached automatically).
    return False  # Return True ONLY to consume and block lower-priority handlers.
```

-   **Prefer `Pre*`/`Post*` lifecycle events over `messages.Priority`**: Use a
    bare `@messages.handler` on the phase event matching your intent rather than
    tweaking priorities on `ModelEvent` or `StepEvent`:
    -   **After model load/reload**: Subscribe to `PostModelEvent` (so `model`
        and `data` are already installed and `mj_forward` has run).
    -   **Sim step phases**: `PreStepEvent` (write controls, apply forces, or
        call `handle.set_model`), `StepEvent` (physics stepper only),
        `PostStepEvent` (observe/record stepped state).
    -   **Viewer frame phases**: `UpdateEvent` (visual state, `extra_geoms`, GUI
        data prep, `viewer.upload_image`), `BuildGuiEvent` (Dear ImGui / ImPlot
        submission only).
-   **Handler return values**: Return `False` or `None` so downstream handlers
    also receive the message; return `True` only to consume the event and halt
    propagation. **Never return `True` from an `ExitEvent` handler**—teardown
    must reach every registered plugin.

For bidirectional `send_if_changes` synchronization, the complete chronological
Viewer and Simulation lifecycle event catalogs, and `messages.Priority` levels,
load [references/messaging_and_lifecycle.md](references/messaging_and_lifecycle.md).

## 4. High-Frequency Gotchas

1.  **Respect Thread Boundaries & Keep `ViewerHandle` Minimal**: In Python, the
    sim thread must never read or mutate viewer objects (`viewer.camera`,
    `viewer.vis_options`, viewer model/data copies), and the viewer thread must
    never directly mutate sim state outside of handling inbound events and
    sending requests; always communicate via `handle.send_to_viewer(...)` and
    `viewer.send_to_sim(...)`, because direct concurrent access causes data
    races and rendering artifacts. Keep `ViewerHandle` minimal (`sync()`,
    message dispatch, model/data lifecycle)—put domain sim logic in `SimPlugin`
    subclasses (`StepControl`, `SimHistory`) and do not add `ux` or `sim`
    imports to `viewer_handle.py`.
2.  **Preserve `run_sim_loop` Boilerplate & Edit State in Step Handlers**: Keep
    `viewer_handle.run_sim_loop(handle)`
    (`while handle.is_running(): handle.sync()`) free of ad-hoc logic; put
    custom behavior (custom stepping, perturbation forces, sensor logging, RL
    resets) in a `SimPlugin` subscribing to `SimInitEvent`, `PostModelEvent`,
    `PreStepEvent`, `StepEvent`, `PostStepEvent`, or custom messages, or
    dispatch an `Event` to the sim thread. `sync()` only detects `model.opt`
    changes made inside `PreStepEvent` / `StepEvent` / `PostStepEvent` (edits in
    an outer loop are not sent and get overwritten by the viewer's next
    snapshot; structural model edits require `handle.set_model()`), and `sync()`
    applies viewer resets, slider edits, and perturbations before `PreStepEvent`
    (overwriting controls/forces set in an outer loop). In `.launch(...)` loops
    that own `(model, data)`, always rebind
    `model, data = handle.sync(model, data)` so reloads (`LoadModelEvent` or
    `handle.set_model()`) do not leave the loop stepping a stale model.
3.  **Call `.copy()` on MuJoCo NumPy Views**: MuJoCo array bindings return views
    into mutable C buffers; copy them when recording history across timesteps
    (e.g., position trajectories, Euler angles):

    ```python
    # Avoid: view updates in place, so all history entries alias latest state
    self.history.append(app.data.xpos[body_id])

    # Good: stores an independent snapshot
    self.history.append(app.data.xpos[body_id].copy())
    ```

4.  **Build and Run with `-c opt`**: Always compile and run Studio targets with
    `-c opt` so physics stepping, Filament rendering, and NetImgui/WebSocket
    streaming execute with compiler optimizations enabled.
