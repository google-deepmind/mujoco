# Graphics Backends, WebViewer Streaming & Debugging

## 1. Graphics Backends (`ViewerConfig.gfx`)

Graphics backends are selected via `ViewerConfig.gfx` using values from
`GFX_MODES` in `viewer_protocol.py` (matching `mujoco::studio::GraphicsMode` in
C++):

-   **`NativeViewer` modes**: `opengl` (default Filament OpenGL), `vulkan`
    (Filament Vulkan), and headless/software modes (`opengl_headless`,
    `opengl_software`, `vulkan_software`).
-   **`WebViewer` modes**: `web` / `webgl`.

> [!TIP]
>
> **Decoupling Host Visualization Dependencies:**
> Launching via `WebViewer` (`gfx='web'` or `launch_web`) keeps the host
> simulation process completely free of native graphics and windowing libraries
> (Filament, SDL, Vulkan, X11/Wayland). The host executes physics and a headless
> Dear ImGui context, while all 3D scene rendering and GPU rasterization run in
> the user's browser via WebAssembly (WASM).

## 2. How `WebViewer` Streaming Works

```text
┌────────────────────────────────────────────────────────┐
│              Simulation Thread (Host)                  │
│  - Runs physics loop & SimPlugin instances             │
│  - Pushes state snapshots & receives events            │
└────────────────────────────────────────────────────────┘
                         ▲ Endpoints (IPC / Queues)
                         │ (SimEndpoint <-> ViewerEndpoint)
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

-   **Independent from Sim/Viewer Messages**: The wire protocol between the
    `WebViewer` thread and the browser is completely separate from the
    `Snapshot` and `Event` channels connecting the simulation and viewer
    threads.
-   **Two Multiplexed Streams on One Port**: `WebViewer` serves the HTML/WASM
    client and both WebSocket streams over a single HTTP port (default `8080`),
    requiring no X11 forwarding, VNC, or remote desktop setup:
    -   **GUI Stream (`/ui`)**: Bidirectional NetImgui draw lists and user
        mouse/keyboard input.
    -   **3D Scene Stream (`/state`)**: Compiled binary `.mjb` model payloads
        and raw MuJoCo state vectors, plus compact metadata for visual state not
        stored in `MjModel`/`MjData` (camera transforms, perturbation forces,
        and transient geoms).

> [!WARNING]
>
> **Trust Model (Local & Trusted Networks Only):**
> `WebViewer` uses plain HTTP and WebSockets with no authentication or
> encryption and listens on all network interfaces. Connected browsers can view
> host-side information including model file paths, exception messages, and
> logs. Run it only locally, on trusted networks, or behind an authenticated
> tunnel—never expose its port to untrusted networks. The startup banner repeats
> this as a short warning.

## 3. Preventing WebViewer WebSocket Timeouts ("Server Not Reachable")

In `WebViewer` mode (`launch_web`), the host server and browser client exchange
continuous heartbeat ping-pongs over `/ui` and `/state`:

-   **Never Block the Viewer Thread**: Synchronous network downloads, blocking
    disk I/O, or CPU-heavy `MjSpec` compilation inside `UpdateEvent` or
    `BuildGuiEvent` stalls the server event loop.
-   **Symptom**: Dropped heartbeats trigger a `DISCONNECTED` / "viewer server
    not reachable" overlay banner in the browser.
-   **Fix**: Offload heavy I/O and asset preparation to background worker
    threads, then send completed payloads or specs to the simulation thread via
    asynchronous `Event` messages.

## 4. Debugging: Terminal, Studio Console & Browser Console

Host-side and browser-side logs surface in different places:

1.  **Host Logs (Python & MuJoCo Engine)**: Once `ViewerApp` is initialized (and
    in the C++ Studio app), MuJoCo engine messages (`mju_warning`,
    `mju_message`, ...), Python `logging` records, `warnings.warn` calls, and
    uncaught exceptions from any thread are captured into the Studio Console
    (`ux.console_gui` / `Console::WindowGui`) while still forwarding to existing
    handlers (falling back to `logging.lastResort` when no Python handler is
    configured, so terminal output is preserved).
    -   **Interactive UI**: The status bar displays message, warning, and error
        counts—click it to toggle the Console. Right-click inside the Console to
        clear lines, copy all lines, or adjust `mju_setLogConfig` (terminal
        printing, `MUJOCO_LOG.TXT` file output, and active info topics; the
        Console itself captures regardless). In the browser builds (`WebViewer`
        and MuJoCo Live), copying from the UI writes the browser's clipboard
        through `MaybeSaveToClipboard` (`imgui_widgets.cc`); it needs a recent
        click and, for the Clipboard API, https or localhost (plain http falls
        back to `execCommand('copy')`), and logs failures to the browser
        console.
    -   **Programmatic Inspection**: Tests and scripts can inspect captured host
        logs via `ux.get_console_lines()` and `ux.get_console_counts()`.
2.  **Browser Logs (WebViewer Rendering, WASM, or WebSocket Errors)**: Client
    errors appear only in the browser developer console. Inspect them via the
    `chrome-devtools` MCP (`list_console_messages`) when available, or launch
    headless Chrome with `--remote-debugging-port=9222` and subscribe to the
    Chrome DevTools Protocol `Runtime.consoleAPICalled` and
    `Runtime.exceptionThrown` events.
