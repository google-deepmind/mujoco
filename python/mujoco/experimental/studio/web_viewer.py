# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Simulation-agnostic web viewer for MuJoCo models.

The WebViewer streams UI and simulation state to a browser:

  * The ImGui UI is built into a headless ImGui context and streamed to the
    browser with the NetImgui protocol through a WebSocket-to-TCP proxy.
    Input captured in the browser flows back over the same connection and is
    injected into the headless context, so all viewer-side handlers work
    unmodified.
  * Physics state and render function state are streamed to the browser over
    a WebSocket with latest-wins semantics. Note that Message types are only
    streamed between the Python simulation and viewer; the data streamed to
    the browser is fixed and independent of the user's custom message types.
  * The browser runs the ``web_client`` WASM app, which renders the MuJoCo
    scene with Filament and overlays the remote ImGui draw data.
  * Everything is served through one public port (default 8080): the page and
    model over HTTP, the UI stream at path /ui and the state stream at /state.
    Exposing or tunneling that single port exposes the whole viewer; the other
    ports are loopback-internal.
"""

import multiprocessing
import os
import queue
import shutil
import socket
import tempfile
from typing import Any
import zlib

import mujoco
from mujoco.experimental.studio import endpoints
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import ux
from mujoco.experimental.studio import viewer_protocol
from mujoco.experimental.studio.web import headless_ui
from mujoco.experimental.studio.web import state_payload
from mujoco.experimental.studio.web import web_server
import numpy as np

from mujoco.experimental.dear_imgui import dear_imgui as imgui
from mujoco.experimental.implot import implot

# File extensions a dropped file may load as a model.
_MODEL_EXTENSIONS = ('.xml', '.urdf', '.mjb', '.mjz', '.zip')


def _find_assets_dir() -> str:
  """Locates the Studio assets directory inside the mujoco package."""
  env_dir = os.environ.get('MUJOCO_STUDIO_ASSETS_DIR')
  if env_dir and os.path.isdir(env_dir):
    return env_dir
  package_dir = os.path.dirname(os.path.abspath(mujoco.__file__))
  assets_dir = os.path.join(package_dir, 'experimental', 'studio', 'assets')
  if os.path.isdir(assets_dir):
    return assets_dir
  return ''


def _lan_ips() -> tuple[str | None, str | None]:
  """Returns this machine's outbound-interface (IPv6, IPv4) addresses.

  These are the addresses other machines on the same network can reach the
  viewer at. A None entry means that family has no shareable address.
  """

  def probe(
      family: int | socket.AddressFamily, dest: tuple[str, int]
  ) -> str | None:
    try:
      with socket.socket(family, socket.SOCK_DGRAM) as s:
        s.connect(dest)
        return s.getsockname()[0]
    except OSError:
      return None

  ipv6 = probe(socket.AF_INET6, ('2001:4860:4860::8888', 80))  # pytype: disable=wrong-arg-types
  if ipv6 and ipv6.startswith(('fe80', '::1')):
    ipv6 = None
  ipv4 = probe(socket.AF_INET, ('8.8.8.8', 80))  # pytype: disable=wrong-arg-types
  if ipv4 and ipv4.startswith('127.'):
    ipv4 = None
  return ipv6, ipv4


def _print_url_banner(host: str, port: int) -> None:
  """Prints a prominent boxed banner with the URLs browsers can use."""
  rows = [('local', f'http://localhost:{port}')]
  if host in ('::', '0.0.0.0'):
    # Shareable with other machines on the same network. Both families are
    # always listed: a visitor may only be reachable over one of them, and an
    # explicit "(unavailable)" beats a silently missing row. IPv6 literals must
    # be bracketed in URLs.
    ipv6, ipv4 = _lan_ips()
    rows.append((
        'network (IPv6)',
        f'http://[{ipv6}]:{port}' if ipv6 else '(unavailable)',
    ))
    rows.append(
        ('network (IPv4)', f'http://{ipv4}:{port}' if ipv4 else '(unavailable)')
    )

  label_width = max(len(label) for label, _ in rows)
  lines = ['MuJoCo Web Viewer running at:', '']
  lines += [f'  {label.ljust(label_width)}  {url}' for label, url in rows]
  lines += ['', 'Ctrl+C to quit']

  width = max(len(line) for line in lines)
  banner = [
      '+' + '-' * (width + 2) + '+',
      *(f'| {line.ljust(width)} |' for line in lines),
      '+' + '-' * (width + 2) + '+',
  ]
  print('\n'.join(banner), flush=True)


def _pick_drop_root(paths: list[str]) -> str | None:
  """Picks the model file to load from a dropped set of files.

  Rules, in order: the only loadable file wins; otherwise, among the loadable
  files at the shallowest directory depth, prefer one named after its folder
  (the mjz convention, e.g. cards/cards.xml), then scene.xml (the
  mujoco_menagerie convention), then the alphabetically first.

  Args:
    paths: A list of relative file paths.

  Returns:
    The path to the model file to load, or None if no model file was found.
  """
  candidates = [p for p in paths if p.lower().endswith(_MODEL_EXTENSIONS)]

  if not candidates:
    return None

  if len(candidates) == 1:
    return candidates[0]

  depth = min(p.count('/') for p in candidates)
  shallow = sorted(p for p in candidates if p.count('/') == depth)
  for path in shallow:
    parts = path.split('/')
    stem = os.path.splitext(parts[-1])[0]
    if len(parts) > 1 and stem == parts[-2]:
      return path

  for path in shallow:
    if os.path.basename(path) == 'scene.xml':
      return path

  return shallow[0]


class WebViewer(viewer_protocol.Viewer):
  """Simulation-agnostic web viewer for MuJoCo models."""

  def __init__(
      self,
      config: viewer_protocol.ViewerConfig,
      endpoint: endpoints.ViewerEndpoint,
      *,
      model: mujoco.MjModel | None = None,
      model_path: str = '',
      handlers: list[Any] | None = None,
      camera: mujoco.MjvCamera | None = None,
      vis_options: mujoco.MjvOption | None = None,
      perturb: mujoco.MjvPerturb | None = None,
      render_flags: ux.RenderFlags | None = None,
      extra_geoms: list[mujoco.MjvGeom] | None = None,
      host: str = '::',
      http_port: int | None = None,
      ui_tcp_port: int = 0,
  ) -> None:
    """Initializes the WebViewer.

    Args:
      config: Viewer window configuration.
      endpoint: The viewer endpoint for communication with the sim side.
      model: Optional initial MjModel. Forwarded to the base Viewer.
      model_path: Optional path to the model file.
      handlers: Optional list of handler instances.
      camera: Camera parameters. Internal object is created if None.
      vis_options: Visualization options. Internal object is created if None.
      perturb: Perturbation parameters. Internal object is created if None.
      render_flags: Render flags. Internal object is created if None.
      extra_geoms: List of extra geoms. Internal list is created if None.
      host: Public interface the server binds to. The default "::" accepts both
        IPv6 and IPv4 connections (IPv4-only where IPv6 is unavailable).
      http_port: The single public port: page, WASM, /model.mjb, and the /ui and
        /state WebSocket paths. None falls back to config.http_port; 0 picks the
        first free port starting at 8080, so several viewers can run side by
        side.
      ui_tcp_port: Loopback TCP port the headless NetImgui client connects to. 0
        (the default) uses an OS-assigned ephemeral port — both endpoints live
        in this process tree, so no fixed number is needed.
    """
    super().__init__(
        config,
        endpoint,
        model=model,
        model_path=model_path,
        handlers=handlers,
        camera=camera,
        vis_options=vis_options,
        perturb=perturb,
        render_flags=render_flags,
        extra_geoms=extra_geoms,
    )

    self._host = host
    # Bind both listening sockets up front, in this process: bind conflicts
    # surface here as one clear error, the public port stays stable across
    # server restarts, and the loopback port is OS-assigned so viewer instances
    # can never collide on it.
    requested_port = config.http_port if http_port is None else http_port
    self._http_sock = web_server.bind_public_socket(host, requested_port)
    self._http_port = self._http_sock.getsockname()[1]
    self._tcp_sock = web_server.bind_loopback_socket(ui_tcp_port)
    self._ui_tcp_port = self._tcp_sock.getsockname()[1]

    # Headless ImGui context streaming UI draw data via NetImgui.
    self._headless_ui = headless_ui.HeadlessUi(
        config.title or 'MuJoCo Web Viewer',
        self._ui_tcp_port,
        _find_assets_dir(),
    )

    # Point the Python Dear ImGui bindings at the headless context so that
    # viewer-side handlers build their GUI into it. The ImPlot context must be
    # shared in the same way.
    ctx = self._headless_ui.get_context()
    imgui.SetCurrentContext(ctx)
    ux.set_imgui_context(ctx)
    ux.set_implot_context(self._headless_ui.get_implot_context())
    implot.set_imgui_context(ctx)
    implot.set_implot_context(self._headless_ui.get_implot_context())

    # The single-port server (HTTP + /ui + /state + /drop). This is restarted
    # whenever the model changes.
    self._web_server: web_server.WebServer | None = None
    self._model_crc32 = 0

    # Files dropped onto the browser page arrive here from the server child as a
    # dict of relative path -> bytes; owned by the viewer so it survives server
    # restarts.
    self._drop_queue = multiprocessing.get_context('fork').Queue()

    # The controlling page's session id, written by the server child. Owned by
    # the viewer so a fresh server (model change restarts it) can reserve the
    # controller slot for the same page instead of letting whichever page
    # reconnects first win it.
    self._controller_sid = multiprocessing.get_context('fork').Array('c', 64)

    # Temp dir holding the most recent drop's files; removed when the next drop
    # supersedes it (its model is already parsed) and on close.
    self._drop_dir = None
    self._start_servers()
    _print_url_banner(self._host, self._http_port)

    # Dispatch lifecycle event so handlers can cache the viewer reference.
    self.dispatch(viewer_protocol.ViewerInitEvent(viewer=self))

  # ---------------------------------------------------------------------------
  # Server lifecycle.
  # ---------------------------------------------------------------------------

  def _start_servers(self) -> None:
    """Starts (or restarts) the web server, serving the current model."""
    self._stop_servers()

    # Serialize the compiled model to MJB bytes (served as /model.mjb).
    buffer = np.empty(mujoco.mj_sizeModel(self.model), np.uint8)
    mujoco.mj_saveModel(self.model, None, buffer)
    mjb_data = buffer.tobytes()

    # Identity of the served model, included in every state payload. When it
    # changes, the browser refetches /model.mjb by reloading the page.
    self._model_crc32 = zlib.crc32(mjb_data)

    state_sig = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    state_size = mujoco.mj_stateSize(self.model, state_sig)
    max_payload = state_payload.max_state_payload_size(
        state_size * np.float64().itemsize
    )

    self._web_server = web_server.WebServer(
        http_sock=self._http_sock,
        tcp_sock=self._tcp_sock,
        mjb_data=mjb_data,
        max_payload_size=max_payload,
        drop_queue=self._drop_queue,
        controller_sid_shared=self._controller_sid,
    )
    self._web_server.start()

  def _stop_servers(self) -> None:
    if self._web_server is not None:
      self._web_server.stop()
      self._web_server = None

  # ---------------------------------------------------------------------------
  # Message handlers.
  # ---------------------------------------------------------------------------

  @messages.handler(priority=messages.Priority.CRITICAL)
  def _on_model(self, event: messages.ModelEvent) -> bool:
    """Loads the new model, then restarts the servers to serve it.

    The handler registry discovers handlers by name, so this override replaces
    the base Viewer's _on_model and must call it explicitly to load the model
    before the servers serialize it. The browser reconnects to the new state
    server, notices the changed model identity in the payload, and reloads
    itself to fetch the new model.
    """
    super()._on_model(event)
    self._start_servers()
    print('Model changed: the browser page reloads automatically.', flush=True)
    return False  # Do not consume; let other handlers see the event.

  # ---------------------------------------------------------------------------
  # Viewer interface.
  # ---------------------------------------------------------------------------

  def prepare_next_frame(self) -> bool:
    """Advances to the next headless frame; returns False when disconnected."""
    return self._headless_ui.new_frame()

  def sync(self) -> None:
    """Streams state to the browser and ends the headless ImGui frame."""
    if self._web_server is not None:
      state_sig = int(mujoco.mjtState.mjSTATE_INTEGRATION)
      state_size = mujoco.mj_stateSize(self.model, state_sig)
      state = np.empty(state_size, np.float64)
      mujoco.mj_getState(self.model, self.data, state, state_sig)
      payload = state_payload.serialize_state_payload(
          self._model_crc32,
          state_sig,
          state.tobytes(),
          self.camera,
          self.perturb,
          self.vis_options,
          self.model,
          list(self.render_flags.flags),
          self.extra_geoms[: state_payload.MAX_EXTRA_GEOMS],
      )
      self._web_server.update_state(payload)

    # Finish the ImGui frame; NetImgui sends the draw data to the browser.
    self._headless_ui.end_frame()

  def close(self) -> None:
    self._stop_servers()
    self._http_sock.close()
    self._tcp_sock.close()
    if self._drop_dir is not None:
      shutil.rmtree(self._drop_dir, ignore_errors=True)
      self._drop_dir = None
    super().close()

  def get_drop_file(self) -> str:
    """Returns the path of a model file dropped onto the browser page, or ''.

    The browser uploads the dropped files' bytes over the /drop WebSocket
    (so drops work even when the browser runs on another machine). They are
    written to a temporary directory here, preserving names and relative
    paths — a dropped folder's asset references resolve, and the mjz/zip
    decoder can locate an archive's root XML by the archive's own name —
    and the regular drop-loading flow (ViewerApp -> parser.parse) applies.
    """
    try:
      files = self._drop_queue.get_nowait()
    except queue.Empty:
      return ''

    # TODO(matijak): This could work without disk access: mjVFS can hold the
    # dropped files in memory (mj_addBufferVFS) and mj_parse resolves
    # includes/assets from it (ModelHolder::InitFromBuffer already covers
    # the single-buffer XML/MJB/ZIP cases). Needs a buffer-based parser
    # entry point and a viewer drop interface that isn't a file path.
    # The previous drop's model has already been parsed, so its temp dir can
    # go now; keeping only the newest avoids leaking one dir per drop.
    if self._drop_dir is not None:
      shutil.rmtree(self._drop_dir, ignore_errors=True)
    self._drop_dir = tempfile.mkdtemp(prefix='mujoco_drop_')
    drop_dir = self._drop_dir
    written = []
    for name, payload in files.items():
      rel = name.replace('\\', '/').lstrip('/')
      parts = [p for p in rel.split('/') if p not in ('', '.', '..')]
      if not parts:
        continue
      rel = '/'.join(parts)
      path = os.path.join(drop_dir, *parts)
      os.makedirs(os.path.dirname(path), exist_ok=True)
      with open(path, 'wb') as f:
        f.write(payload)
      written.append(rel)
    root = _pick_drop_root(written)
    if root is None:
      print(
          'Dropped file(s) contain no loadable model '
          f'({", ".join(_MODEL_EXTENSIONS)}).',
          flush=True,
      )
      return ''
    return os.path.join(drop_dir, *root.split('/'))

  def upload_image(
      self, tex_id: int, img: str | bytes, width: int, height: int, bpp: int
  ) -> int:
    """Uploads an image to the browser over the NetImgui texture channel."""
    if isinstance(img, str):
      img = img.encode('latin-1')
    return self._headless_ui.upload_image(tex_id, img, width, height, bpp)
