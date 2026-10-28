# Authoring Python Studio Plugins & Hot Reload

Explore `python/mujoco/sample/studio/` for actively maintained
reference implementations.

> [!IMPORTANT]
>
> **Always Put Plugins in Separate Module Files (Never in `__main__`):**
> Always define `ViewerPlugin` and `SimPlugin` classes—along with any helper
> logic or procedural `MjSpec` builders you want to iterate on live—in their own
> importable `.py` module(s), separate from the `__main__` launcher script (or
> load them directly via `viewer.py --plugins=my_plugins.py`). Python's
> `__main__` script is fixed and **cannot** be reloaded by `--hot_reload`;
> keeping the entrypoint script as a thin launcher (flag parsing, one-time data
> loading, and `launch_web.run(...)`) makes all plugin and UI code
> hot-reloadable out of the box.
>
> **Existing Code with Inline Plugins:** If the user points you to a script
> that defines plugins directly inside the `__main__` file (e.g., alongside
> `if __name__ == '__main__':`), ask the user whether they would like to split
> the plugins into a separate module, noting that this unlocks Studio's
> `--hot_reload` workflow for fast live iteration without viewer restarts.

## 1. Key Python Plugin Patterns

### Viewer UI Plugins (Dear ImGui & ImPlot)

See `python/mujoco/sample/studio/implot.py` (`BodyInspector`) for a
complete viewer UI plugin demonstrating how to:

-   Subscribe to `ViewerAppInitEvent` to cache `event.viewer_app` references.
-   Draw responsive ImPlot charts during `BuildGuiEvent` (preparing any heavy
    frame data or texture uploads earlier in `UpdateEvent`).
-   Use persistent window IDs (`###`) to preserve docking and window state
    across dynamic title changes.

### Visual Overlays & Transient Geoms

See `python/mujoco/sample/studio/ghost.py` (`GhostRenderer`) for
subscribing to `UpdateEvent` and populating `viewer.extra_geoms` with transient
`mjvGeom` instances without mutating the compiled `MjModel`.

> [!WARNING]
>
> **Upcoming API Migration for Extra Geoms:**
> `viewer.extra_geoms` in `sample/studio/ghost.py` is an interim mechanism and will be
> replaced by the direct scene/geom API in
> `python/mujoco/rendering/filament/renderer.py`. Prefer referencing that
> renderer API when architecting new custom visual overlays.

### Split Sim/Viewer Architecture (`MjSpec` Editing & Asset Insertion)

Never access or mutate mutable `MjSpec` state directly from a viewer-side UI
plugin. Split scene-editing plugins across both threads:

1.  **Viewer plugin (e.g., `AssetBrowserGui`)**: Renders the browser UI and
    sends a custom `Event` (e.g., `AssetDropEvent`) carrying the selected asset
    identifier and target drop pose.
2.  **Simulation plugin (e.g., `AssetBrowserSim`)**: Owns the `MjSpec` on the
    simulation thread, handles `AssetDropEvent`, attaches/edits the spec,
    recompiles `new_model`, and publishes it with
    `handle.set_model(new_model, path=handle.model_path)`. Passing
    `path=handle.model_path` preserves viewer camera, pause, and playback speed
    across in-place scene edits.

## 2. Studio UI Plugin Architecture

For general Dear ImGui and ImPlot conventions (`build_gui` vs `render` naming,
decoupling GUI glue from logic, Tables API grid alignment, `##` vs `###` ID
hashing, `ImGuiListClipper` virtualization for WebViewer/NetImgui bandwidth, and
Dear ImGui Test Engine readiness), load the `mujoco-gui` skill
(`doc/skills/gui/SKILL.md`). In addition, follow these Studio-specific rules:

-   **Window Granularity & Docking**: Implement each viewer GUI plugin as **one
    or two ImGui windows** (`imgui.Begin(...)`) so users can dock panels cleanly
    into the Studio Viewer App dockspaces alongside built-in tools.
-   **Shared Custom Widgets (`imgui_widgets.h`/`cc`)**: Reusable C++ custom
    ImGui widgets live in `studio/ux/imgui_widgets.h` and
    `imgui_widgets.cc`. (A counterpart shared widgets module under
    `python/mujoco/studio/` can be used as Python-authored widgets
    grow.)
-   **Python vs C++ Widgets (GIL, Performance & Web Portability)**: Before
    implementing a compute-heavy or reusable widget in Python, consider whether
    it actually needs the Python GIL:
    -   **Releasing the GIL & Performance**: If a widget does not depend on
        Python-specific runtime state, author it as a native C++ widget exposed
        via a thin Python binding so the GIL can be released during execution
        and per-frame Python interpreter and `pybind11` overhead is avoided
        (important for widgets doing intensive computation).
    -   **MuJoCo Live vs WebViewer Portability**: Native C++ widgets compile to
        WebAssembly (via Emscripten) and bind to JavaScript for client-side
        browser apps such as **MuJoCo Live**. By contrast, **WebViewer** runs
        plugins on the host server and streams ImGui draw lists to the browser;
        its WASM browser client (`web/`) only runs minimal C++ ImGui code for
        client-side chrome (such as connection banners and disconnect dialogs),
        so server-side Python plugins already work in `WebViewer` without WASM
        compilation.

## 3. Hot Reload (Python Only, Opt-In)

When hot reload is enabled (`viewer.py --hot_reload=all`, or
`viewer_protocol.ViewerConfig(hot_reload='all')`; disabled by default), saving a
watched `.py` plugin module or model file updates the running viewer on the next
frame or step without restarting—preserving the browser WebSocket connection,
camera pose, UI docking layout, and plugin state.

The `hot_reload` option selects what to watch (parsed via
`hot_reload.Options.parse`):

-   `'all'` — Registered plugin modules and the active model file.
-   `'plugins'` — Registered plugin modules only.
-   `'model'` — The active model file only.
-   Comma-separated plugin class names (e.g., `'Scene,SceneGui'`), combinable
    with `'model'` (e.g., `'model,Scene'`).

See the module docstring of `python/mujoco/studio/hot_reload.py`
for full design details:

-   **Reloadable vs. fixed boundary**: Registered plugin modules (both user
    plugins and built-ins like `ViewerApp`, `StepControl`, and `SimHistory`),
    same-directory helper modules, and the active XML/URDF model file (including
    its `<include>`s and assets) reload on save. Framework modules, `__main__`,
    the C++ binary, and the list of registered plugin instances are fixed.
    Always keep plugins in separate module files rather than `__main__` (and if
    a user's script defines plugins inline in `__main__`, ask whether to extract
    them so hot reload is unlocked), register the viewer/sim plugin classes you
    expect to grow up front, and compose separate plugins rather than
    subclassing built-in plugins.
-   **Preserving and reconciling state (`HotReloadEvent`)**: Declare default
    attribute values on the plugin class (e.g., `height: int = 5`) so mutated
    instance attributes on `self` survive class swaps while newly added class
    attributes appear automatically without re-running `__init__`. Subscribe to
    `messages.HotReloadEvent` (`if self in event.plugins:`) to allocate new
    mutable buffers, invalidate cached element IDs, migrate reshaped state, or
    trigger immediate rebuilds.
-   **Error recovery**: Syntax, import, or model compilation errors retain the
    last working code and model and retry on the next file save. Exceptions
    raised inside a hot-swapped plugin handler are caught and logged once per
    handler until the next reload.

### Live Iteration Workflow When Hot Reload Is Active

When a user is running Studio with `--hot_reload` enabled and asks for changes,
optimize for **minimum time-to-visual-feedback** rather than upfront batch
verification:

-   **Edit Reloadable Files Immediately & Postpone Tests**: Write changes to the
    watched plugin `.py` or model files as quickly as possible so the running
    viewer picks them up on the next tick. Postpone running test suites, full
    rebuilds, or slow static checks until the interactive iteration wraps up (or
    until a non-reloadable file must be rebuilt), as running them on every turn
    stalls the live feedback loop.
-   **Ship in Small, Incremental Steps**: Break multi-part features or scene
    adjustments into small, self-contained edits saved sequentially rather than
    one large monolithic change. Seeing incremental updates in the live viewer
    lets the user steer layout, behavior, or placement early.
-   **Rely on Error Isolation — and Fix Anything That Breaks It**: The hot
    reload system is designed to be resilient to syntax errors, import failures,
    model compilation errors, and handler exceptions in reloadable files without
    terminating the viewer or simulation. Trust this safety net to iterate fast;
    if an edit during hot reload ever *does* bring down the viewer or
    simulation, remember to track down and fix that crash or error-isolation gap
    so future reloads stay robust.
-   **Stay Strictly On-Task (Ask Before Deviating)**: Users turn on hot reload
    to collaborate interactively at high speed. Do not take unrequested detours
    (such as tangential refactors, unrelated cleanups, or speculative side
    investigations); if you want to deviate from the requested task—including
    pausing feature iteration to fix an underlying framework issue—ask the user
    first.
