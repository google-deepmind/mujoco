# Messaging Protocol & Lifecycle Events (`messages.py`)

All communication between the simulation thread and viewer thread in Python
Studio runs over two channel base classes. By convention, custom message classes
derive from `Snapshot` or `Event` and end in `*Snapshot` or `*Event` to make
their delivery semantics explicit at every call site.

## 1. `Snapshot` vs `Event` Channels

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

### Choosing Between `Snapshot` and `Event` for GUI Controls

Rule of thumb for GUI controls (sliders, checkboxes, settings):

-   **Use a `Snapshot`** when a control's *complete* state fits in one message
    and only the latest value matters; send it on change (or every frame) and
    let latest-wins coalesce redundant updates. Examples: `StepControlSnapshot`
    (pause state, speed, noise), `MjOptionSnapshot` (`mjOption` fields), or a
    plugin's own `FooConfigSnapshot` carrying its slider values.
-   **Use an `Event`** when distinct edits of the same message type would
    clobber each other in the per-type latest-wins slot and every edit must be
    applied — e.g., `StateEvent` for joint/control slider edits, which differ by
    `state_sig` — and for one-shot requests such as `ResetEvent`,
    `SingleStepEvent`, and `RequestPauseEvent`.

## 2. Bidirectional State Sync (`messages.send_if_changes`)

Bidirectional snapshots (`MjOptionSnapshot`, `StepControlSnapshot`) must be sent
only by the side that actually mutated the value; if either side broadcast its
full value every frame, it would overwrite concurrent edits from the other side.

Both threads use
`messages.send_if_changes(send_fn, *make_messages, should_send=...)`, a context
manager that evaluates each `make_message()` callable (pass lambdas so state is
freshly read) on entering and leaving the `with` block and sends only the
messages whose values changed:

-   **Viewer side**: `viewer.send_if_changes(...)` wraps UI mutation blocks
    (`studio_app.App.update()`, `build_gui()`, and Inspector panels).
    Incoming sim snapshots are applied *outside* those blocks, preventing echo
    loops.
-   **Sim side**: `ViewerHandle.sync()` wraps `PreStepEvent`, `StepEvent`, and
    `PostStepEvent` dispatch for `model.opt`, and plugins that own mirrored
    state wrap their own mutations in `handle.send_if_changes` (for example,
    `StepControl` wraps `advance()` and `RequestPauseEvent`). Incoming viewer
    snapshots are applied outside those blocks so they never echo back either.

Because `send_if_changes` only observes mutations made inside a wrapped block:

-   Edit `model.opt` inside a step-event handler, never in an outer launcher
    loop between `sync()` calls.
-   Change another plugin's state by dispatching an event it handles (such as
    `RequestPauseEvent` for `StepControl`) rather than mutating its attributes
    directly.

## 3. Chronological Lifecycle Events

Runner loops dispatch standard lifecycle events in chronological order from
startup through per-frame/per-step execution to teardown:

### Viewer Lifecycle Events (`run_viewer_loop` -> `ViewerPlugin`)

1.  **`ViewerInitEvent`** (`viewer_protocol.ViewerInitEvent`, also exported as
    `plugin.ViewerInitEvent`) — Dispatched once on viewer initialization.
    `ViewerPlugin` caches `self.viewer` automatically above `CRITICAL` priority
    before subclass handlers run.
2.  **`ModelEvent`** — Received when a model is published by the sim; `Viewer`
    deep-copies `viewer.model`, creates `viewer.data`, records
    `viewer.model_path` (`event.path`), and runs `mj_forward`.
3.  **`PostModelEvent`** — Dispatched locally by `Viewer` immediately after
    `ModelEvent` has initialized `viewer.model` and `viewer.data`. Carries
    `(model, data, path)`: reloading the same `path` preserves viewer UI state
    (camera, pause, and playback speed in `studio_app.App`), whereas a
    different or empty `path` resets it. Subscribe to `PostModelEvent` (not
    `ModelEvent`) when resetting viewer-plugin state on model changes so
    `viewer.model` and `viewer.data` are already up to date.
4.  **`UpdateEvent`** — Dispatched every frame (after incoming sim events and
    snapshots are applied) to update visuals, camera, perturbations, and
    transient geoms, and prepare GUI data or upload textures (e.g.,
    `viewer.upload_image`).
5.  **`BuildGuiEvent`** — Dispatched every frame immediately after `UpdateEvent`
    to construct and lay out Dear ImGui / ImPlot widgets. Keep handlers focused
    strictly on ImGui/ImPlot submission; prepare data in `UpdateEvent`.
6.  **`ExitEvent`** — Dispatched once when the viewer shuts down, whether exit
    originated from the sim side, window close, or a viewer plugin handler
    raising an exception (which is then re-raised on the sim side by
    `ViewerHandle.close()`). Handle it to release threads, pools, or GPU
    handles.

### Simulation Lifecycle Events (`ViewerHandle` -> `SimPlugin`)

1.  **`SimInitEvent`** (`viewer_handle.SimInitEvent`, also exported as
    `plugin.SimInitEvent`) — Dispatched once by `ViewerHandle.__init__`.
    `SimPlugin` caches `self.handle` automatically above `CRITICAL` priority
    before subclass handlers run.
2.  **`LoadModelEvent`** — Sent from the viewer (on **Reload** or file
    drag-and-drop) or dispatched on the sim thread (e.g., by
    `run(config, model_path=...)`). `ViewerHandle` loads `event.path` (or
    reloads `handle.model_path` when `event.path` is empty) via `parser.load`
    and publishes the compiled model with `handle.set_model(model, path=path)`.
    If loading fails, the error is logged and the previous model remains active.
3.  **`ModelEvent`** — Dispatched when a model is published via
    `handle.set_model()` (or when `handle.sync(model, data)` sees a new model
    object); `ViewerHandle` sets `handle.model`, `handle.data`, and
    `handle.model_path`, runs `mj_forward`, and forwards the model to the
    viewer.
4.  **`PostModelEvent`** — Dispatched locally by `ViewerHandle` immediately
    after `ModelEvent` has installed `handle.model` and `handle.data` and run
    `mj_forward`. Carries `(model, data, path)`: reloading the same `path`
    preserves `StepControl` pause, speed, and noise settings. Subscribe to
    `PostModelEvent` (not `ModelEvent`) when resetting sim-plugin state on model
    changes.
5.  **`PreStepEvent`** — Dispatched by `ViewerHandle.sync` on every call (after
    incoming viewer events and snapshots are applied) so plugins can prepare the
    simulation before physics advances (writing actuator controls, applying
    forces, or hot-swapping models via `handle.set_model`).
6.  **`StepEvent`** — Dispatched by `ViewerHandle.sync` immediately after
    `PreStepEvent` so sim-side stepping plugins (such as `StepControl`) can
    advance physics.
7.  **`PostStepEvent`** — Dispatched by `ViewerHandle.sync` immediately after
    `StepEvent` (before `StateSnapshot` is broadcast) so observer/recorder
    plugins (such as `SimHistory`) can inspect or record stepped state.
8.  **`ExitEvent`** — Dispatched once by `ViewerHandle.close()`, called on every
    `with` exit path (normal exit, `KeyboardInterrupt`, exception, or viewer
    termination). Handle it to release resources.

> [!IMPORTANT]
>
> Never return `True` from an `ExitEvent` handler: teardown must reach every
> registered plugin, so no handler may consume `ExitEvent`.

### Plugin-Dispatched Lifecycle Events

-   **`studio_app.InitEvent`** — Dispatched by `studio_app.App` on
    startup so sibling viewer plugins can cache application state and window
    handles (`event.app`).

## 4. Handler Priorities vs. `Pre*`/`Post*` Lifecycle Events

`@messages.handler` (or `@messages.handler(priority=...)`) runs handlers for a
given message type in descending `messages.Priority` order:

1.  `CRITICAL` (`1000`) — Core framework state synchronization and dockspace
    setup.
2.  `USER` (`100`, default) — User plugins and custom UI tools.
3.  `LIBRARY` (`10`) — Standard framework extensions and helpers.
4.  `INTERNAL` (`1`) — Core framework operations and baseline fallbacks.

### Why `Pre*`/`Post*` Phases Beat Priority Tweaking

Ordering distinct lifecycle stages by adjusting `priority=...` on `StepEvent`
(e.g., relative to `StepControl` at `Priority.INTERNAL`) or `ModelEvent` couples
plugins to internal framework priorities and exposes stale state:

-   A high-priority `ModelEvent` handler executes *before* `ViewerHandle` or
    `Viewer` has instantiated the new `MjData` and run `mj_forward`.
-   Calling `handle.set_model()` inside a `StepEvent` handler leaves the
    in-flight `StepEvent` holding references to the replaced `model` and `data`.

Always subscribe to the explicit phase event (`PostModelEvent`, `PreStepEvent`,
`StepEvent`, `PostStepEvent`, `UpdateEvent`, `BuildGuiEvent`) with a bare
`@messages.handler`. Reserve `priority=...` strictly for cases where multiple
handlers target the **exact same event and phase** and one must override or
consume (`return True`) the message before a fallback runs.
