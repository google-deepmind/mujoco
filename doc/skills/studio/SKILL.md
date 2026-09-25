---
name: mujoco-studio
description: >-
  MuJoCo Studio viewer applications, interactive visualization, decoupled
  sim/viewer threads, message passing (Snapshot latest-wins vs Event FIFO queue),
  lifecycle events (ViewerInitEvent, UpdateEvent, BuildGuiEvent, StepEvent),
  @messages.handler priorities, launchers (launch_web, launch_native, launch_passive),
  Python/C++ plugin registry (mjPLUGIN_LIB_INIT), NetImgui/WASM WebViewer streaming.
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
│  - Executes sim_plugins (e.g. StepControl, reset)      │
│  - Drives simulation via handle.sync(model, data)      │
└────────────────────────────────────────────────────────┘
                        ▲
                        │   Queued Events & Snapshots
                        │ (SimEndpoint <-> ViewerEndpoint)
                        ▼
┌────────────────────────────────────────────────────────┐
│              Viewer Thread (or Web Client)             │
│  - Owns Window, Renderer, Camera, VisOptions, Perturb  │
│  - Owns local copy of MjModel and MjData for rendering │
│  - Executes viewer_plugins (ViewerApp, custom UI)      │
│  - Dispatches lifecycle events (UpdateEvent, BuildGuiEvent) │
└────────────────────────────────────────────────────────┘
```

-   **Thread / Process Decoupling (`launch_passive`)**: The simulation thread
    and the viewer thread run independently. The simulation communicates with
    the viewer non-blockingly through endpoints (`SimEndpoint ↔ ViewerEndpoint`)
    using queued `Event`s and latest-value `Snapshot`s.
-   **Priority-based Plugin System (`plugin_registry.py`)**: State and UI
    behaviors are decoupled into standalone plugin classes decorated with
    `@messages.handler(priority=...)`.
    -   **Simulation plugins (`sim_plugins`)**: Handle simulation-side events
        (physics reset, step control, perturbations, policy changes,
        `StepEvent`).
    -   **Viewer plugins (`viewer_plugins`)**: Handle rendering and UI lifecycle
        events dispatched by `run_viewer_loop` (`ViewerInitEvent`,
        `UpdateEvent`, `BuildGuiEvent`).
-   **Standard Samples (`python/mujoco/experimental/studio/`)**:
    -   `python/mujoco/experimental/studio/viewer.py` —
        Minimal baseline with standard GUI (`viewer_app.ViewerApp`).
    -   `python/mujoco/experimental/studio/sample/ghost.py` —
        Demonstrates a viewer plugin (`GhostRenderer`) injecting extra geoms via
        `extra_geoms` and custom Dear ImGui controls on `BuildGuiEvent`.
    -   `python/mujoco/experimental/studio/sample/implot.py` —
        Demonstrates real-time responsive charting with Dear ImGui and ImPlot
        via `BodyInspector`.

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

### Lifecycle Events

Lifecycle events are standard framework events dispatched by the runner loops to
plugins to hook into rendering, UI layout, and physics execution:

-   **Viewer Lifecycle Events** (sent from the viewer loop, `run_viewer_loop`,
    to `viewer_plugins`):
    -   `ViewerInitEvent` (`viewer_protocol.ViewerInitEvent`) — dispatched on
        viewer initialization so stateful plugins can cache the viewer instance
        (`event.viewer`).
    -   `ViewerAppInitEvent` (`viewer_app.ViewerAppInitEvent`) — dispatched on
        `ViewerApp` startup so plugins can cache application state and window
        handles (`event.viewer_app`).
    -   `BuildGuiEvent` — dispatched every frame to construct and lay out Dear
        ImGui / ImPlot widgets.
    -   `UpdateEvent` — dispatched every frame before building GUI to update
        visuals, camera, perturbations, and transient geoms.
    -   `ExitEvent` — dispatched once when the viewer shuts down, whether the
        exit came from the sim side or from the window being closed. Handle it
        to release resources (threads, pools, GPU handles).
-   **Simulation Lifecycle Events** (dispatched on the sim side to
    `sim_plugins`):
    -   `SimInitEvent` (`viewer_handle.SimInitEvent`) — dispatched once by
        `ViewerHandle.__init__` so stateful plugins can cache the handle
        (`event.handle`) instead of the launcher having to hand it to them.
    -   `StepEvent` — dispatched by `ViewerHandle.sync` on every call so
        sim-side stepping plugins (such as `StepControl`) can advance the
        physics.
    -   `ExitEvent` — dispatched once by `ViewerHandle.close()`, which the
        `with` block calls on every exit path (normal exit, `KeyboardInterrupt`,
        an exception, or the viewer dying). Handle it to release resources.

> [!IMPORTANT]
>
> Do not return `True` from an `ExitEvent` handler: teardown must reach every
> registered plugin, so no handler should consume the event.

### The `@messages.handler` Decorator

Plugins subscribe to messages by annotating methods with `@messages.handler`:

```python
from mujoco.experimental.studio import messages

class MyPlugin:
  @messages.handler(priority=messages.Priority.USER)
  def on_build_gui(self, event: messages.BuildGuiEvent) -> bool | None:
    # Build Dear ImGui elements here
    return False  # Return True to consume and halt lower-priority handlers
```

#### Priorities (`messages.Priority`)

1.  `CRITICAL` (1000) — System teardown, exit events, and emergency overrides.
2.  `USER` (100, default) — User plugins and custom UI tools.
3.  `LIBRARY` (10) — Standard framework extensions and helpers.
4.  `INTERNAL` (1) — Core framework operations and baseline fallbacks.

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

All three modules share the same `.run(...)` interface:

```python
from mujoco.experimental.studio import launch_web  # Or launch_native / launch_passive
from mujoco.experimental.studio import step_control
from mujoco.experimental.studio import viewer_app
from mujoco.experimental.studio import viewer_protocol

config = viewer_protocol.ViewerConfig(title='Studio')

launch_web.run(
    config,
    model=model,
    data=data,
    viewer_plugins=[viewer_app.ViewerApp()],
    sim_plugins=[step_control.StepControl()],
)
```

> [!WARNING]
>
> **Stepping and pacing are plugin responsibilities.** Including
> `step_control.StepControl()` in `sim_plugins` provides standard
> real-time-paced CPU stepping. If you omit `StepControl()` (for example, when
> running GPU rollouts or custom RL loops), `handle.sync()` will **not advance
> physics or sleep**, so your stepping plugin must manage its own pacing to
> avoid busy-spinning.

### Advanced: Non-Blocking Launchers (`launch_web.launch`, `launch_native.launch`, `launch_passive.launch`)

When attaching a viewer to an **external loop that you do not want to invert into
a `StepEvent` plugin** (such as an existing RL training loop, trajectory replay
script, or unit test), each module also provides a non-blocking `.launch(...)`
context manager returning a `ViewerHandle`:

```python
with launch_web.launch(
    config,
    viewer_plugins=[viewer_app.ViewerApp()],
    sim_plugins=[step_control.StepControl()],
) as handle:
  while handle.is_running():
    model, data = handle.sync(model, data)
```

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
>     recompiles the model, and broadcasts `ModelEvent(model=new_model)` back to
>     the viewer.

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
│  - Runs physics loop & sim_plugins                     │
│  - Pushes state snapshots & receives events            │
└────────────────────────────────────────────────────────┘
                         ▲ Endpoints (IPC / Queues)
                         │ (SimEndpoint ↔ ViewerEndpoint)
                         ▼
┌────────────────────────────────────────────────────────┐
│              WebViewer Thread (Host Server)            │
│  - Runs viewer_plugins in headless Dear ImGui context  │
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

### 2. Preserve Simulation Loop Boilerplate (Use Plugins & Events)

The outer simulation runner loop:

```python
while handle.is_running():
  model, data = handle.sync(model, data)
```

is standardized boilerplate intended to remain identical across all Studio
launchers (`launch_passive`, web streaming, native C++).  Do not insert
domain-specific simulation logic, direct physics stepping, or ad-hoc state
mutations into this loop. If your application needs custom behavior (e.g. custom
stepping, perturbation forces, sensor logging, or RL resets):
-   **Simulation Plugins**: Encapsulate the behavior in a sim-side plugin
    subscribing to `StepEvent` or custom messages.
-   **Event Dispatch**: Dispatch an `Event` to the simulation thread rather than
    mutating state directly in the outer loop.
-   **Always Reassign `model, data = handle.sync(model, data)`**: Unlike legacy
    `mujoco.viewer.Handle.sync()` (which took no arguments and returned `None`),
    Studio's `ViewerHandle.sync(model, data)` expects the current model/data and
    returns an updated `(model, data)` tuple. When plugins or user actions
    dynamically reload models or attach specs (e.g. via `ModelEvent` or asset
    drops), `handle.sync()` updates internal references and returns the newly
    instantiated `model` and `data`. All Studio callers in the codebase reassign
    `model, data = handle.sync(model, data)`; failing to capture the return
    values leaves caller loops stepping a stale physics model while the viewer
    renders the new one.

> [!NOTE]
> **Future Evolution of Sim Plugins & Handle Access:**
> Strictly keeping the runner loop boilerplate-free might not be fully
> achievable in all situations right now (for example, if sim plugins need
> direct access to the handle or mechanisms not yet exposed via existing
> lifecycle events). We will monitor how launcher boilerplate propagates and
> evolve the event and plugin APIs accordingly. In the meantime, try to avoid
> inserting simulation logic into the loop and consider carefully whether it
> should be handled as an event or plugin instead.

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

If you do not pass `step_control.StepControl()` to `sim_plugins`,
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
