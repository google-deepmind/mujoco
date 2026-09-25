"""Exercise packaged WebAssembly modules in Chromium with cross-origin isolation."""

import argparse
import functools
import http.server
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import threading

from playwright.sync_api import sync_playwright


class Handler(http.server.SimpleHTTPRequestHandler):
  def end_headers(self):
    self.send_header("Cross-Origin-Opener-Policy", "same-origin")
    self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
    super().end_headers()

  def log_message(self, *args):
    pass


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--archive", type=Path, required=True)
  args = parser.parse_args()
  with tempfile.TemporaryDirectory(prefix="mujoco-browser-") as temporary:
    root = Path(temporary)
    with tarfile.open(args.archive) as archive:
      archive.extractall(root, filter="data")
    package = root / "package"
    (package / "index.html").write_text("<!doctype html><title>MuJoCo runtime</title>")
    server = http.server.ThreadingHTTPServer(
        ("127.0.0.1", 0), functools.partial(Handler, directory=str(package))
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
      with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        results = {"browser": browser.version}
        for mode, loader in [("st", "./mujoco.js"), ("mt", "./mt/mujoco.js")]:
          print(f"Chromium {browser.version}: {mode}", file=sys.stderr, flush=True)
          page = browser.new_page()
          page.on("pageerror", lambda error: print(error, file=sys.stderr, flush=True))
          page.on("console", lambda message: print(
              message.text, file=sys.stderr, flush=True) if message.type == "error" else None)
          page.on("requestfailed", lambda request: print(
              request.url, request.failure, file=sys.stderr, flush=True))
          page.add_init_script(
              "Object.defineProperty(navigator, 'hardwareConcurrency', {get: () => 2});"
          )
          page.goto(f"http://127.0.0.1:{server.server_port}/index.html")
          result = page.evaluate("""async (loader) => {
            let stage = 'import';
            return Promise.race([
            (async () => {
            if (!crossOriginIsolated) throw Error('Cross-origin isolation is missing');
            const {default: loadMujoco} = await import(loader);
            stage = 'initialization';
            const mujoco = await loadMujoco();
            stage = 'simulation';
            mujoco.FS.writeFile('/model.xml', `<mujoco><worldbody>
              <body pos="0 0 1"><freejoint/><geom size="0.1"/></body>
              </worldbody></mujoco>`);
            const model = mujoco.MjModel.mj_loadXML('/model.xml');
            const data = new mujoco.MjData(model);
            mujoco.mj_step(model, data);
            const result = {time: data.time, height: data.qpos[2]};
            data.delete();
            model.delete();
            if (!(result.time > 0 && result.height < 1)) {
              throw Error('Simulation did not advance under gravity');
            }
            mujoco.FS.writeFile('/tetra.obj',
              'v 0 0 0\\nv 1 0 0\\nv 0 1 0\\nv 0 0 1\\n' +
              'f 1 3 2\\nf 1 2 4\\nf 1 4 3\\nf 2 3 4\\n');
            mujoco.FS.writeFile('/mesh.xml', `<mujoco><asset>
              <mesh name="tetra" file="/tetra.obj"/></asset>
              <worldbody><geom type="mesh" mesh="tetra"/></worldbody></mujoco>`);
            const mesh = mujoco.MjModel.mj_loadXML('/mesh.xml');
            if (!mesh || mesh.nmesh !== 1) throw Error('Mesh decoding failed');
            mesh.delete();
            return result;
            })(),
            new Promise((_, reject) => setTimeout(
              () => reject(Error(`WebAssembly ${stage} timed out`)), 120000)),
          ]);
          }""", loader)
          results[mode] = result
          page.close()
        browser.close()
        print(json.dumps(results, indent=2))
    finally:
      server.shutdown()
      server.server_close()
      thread.join()


if __name__ == "__main__":
  main()
