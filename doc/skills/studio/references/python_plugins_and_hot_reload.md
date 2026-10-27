# Authoring Python Studio Plugins & Hot Reload

Explore `python/mujoco/experimental/studio/sample/` for actively maintained
reference implementations.

## 1. Key Python Plugin Patterns

### Viewer UI Plugins (Dear ImGui & ImPlot)

See `python/mujoco/experimental/studio/sample/implot.py` (`BodyInspector`) for a
complete viewer UI plugin demonstrating how to:

-   Subscribe to `ViewerAppInitEvent` to cache `event.viewer_app` references.
-   Draw responsive ImPlot charts during `BuildGuiEvent` (preparing any heavy
    frame data or texture uploads earlier in `UpdateEvent`).
-   Use persistent window IDs (`###`) to preserve docking and window state
    across dynamic title changes.

### Visual Overlays & Transient Geoms

See `python/mujoco/experimental/studio/sample/ghost.py` (`GhostRenderer`) for
subscribing to `UpdateEvent` and populating `viewer.extra_geoms` with transient
`mjvGeom` instances without mutating the compiled `MjModel`.

> [!WARNING]
>
> **Upcoming API Migration for Extra Geoms:**
> `viewer.extra_geoms` in `sample/ghost.py` is an interim mechanism and will be
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
    ImGui widgets live in `src/experimental/studio/ux/imgui_widgets.h` and
    `imgui_widgets.cc`. (A counterpart shared widgets module under
    `python/mujoco/experimental/studio/` can be used as Python-authored widgets
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

See the module docstring of `python/mujoco/experimental/studio/hot_reload.py`
for full design details:

-   **Reloadable vs. fixed boundary**: Registered plugin modules (both user
    plugins and built-ins like `ViewerApp`, `StepControl`, and `SimHistory`),
    same-directory helper modules, and the active XML/URDF model file (including
    its `<include>`s and assets) reload on save. Framework modules, `__main__`,
    the C++ binary, and the list of registered plugin instances are fixed.
    Register the viewer/sim plugin classes you expect to iterate on up front,
    and compose separate plugins rather than subclassing built-in plugins.
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
