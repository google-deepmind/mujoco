"""Launch a Studio web viewer and check its browser rendering and state stream."""

import argparse
import functools
import io
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

from PIL import Image
from playwright.sync_api import sync_playwright


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--python", type=Path, required=True)
  parser.add_argument("--dist", type=Path, required=True)
  parser.add_argument("--model", type=Path, required=True)
  parser.add_argument("--screenshot", type=Path)
  args = parser.parse_args()
  with socket.socket() as sock:
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
  url = f"http://127.0.0.1:{port}"
  environment = dict(os.environ, MUJOCO_WEB_VIEWER_DIST=str(args.dist.resolve()))
  with tempfile.TemporaryDirectory(prefix="mujoco-studio-browser-") as directory:
    log_path = Path(directory) / "server.log"
    with log_path.open("w") as output:
      process = subprocess.Popen(
          [str(args.python.absolute()), "-m", "mujoco.experimental.studio.viewer",
           "--gfx=web", f"--port={port}", f"--model={args.model.resolve()}"],
          cwd=directory, env=environment, stdout=output, stderr=subprocess.STDOUT,
          start_new_session=os.name == "posix",
      )
    try:
      deadline = time.monotonic() + 60
      while True:
        if process.poll() is not None:
          raise RuntimeError("Studio exited before serving the browser client")
        try:
          with urllib.request.urlopen(url, timeout=1) as response:
            if response.status == 200:
              break
        except (urllib.error.URLError, TimeoutError):
          pass
        if time.monotonic() >= deadline:
          raise TimeoutError("Studio did not serve the browser client")
        time.sleep(0.1)
      with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            headless=True, args=["--use-angle=swiftshader", "--enable-unsafe-swiftshader"]
        )
        page = browser.new_page(viewport={"width": 960, "height": 720})
        frames = {"state": 0, "state_ack": 0, "ui_received": 0, "ui_sent": 0}
        times = []
        errors = []

        def receive(kind, payload):
          if not isinstance(payload, bytes):
            return
          if kind == "ui":
            frames["ui_received"] += bool(payload)
          elif payload.startswith(b"MJWS") and len(payload) >= 12:
            _, version, blocks, _ = struct.unpack_from("<IHHI", payload)
            if version != 1:
              errors.append(f"Unexpected state protocol version: {version}")
              return
            offset = 12
            for _ in range(blocks):
              tag, size = struct.unpack_from("<II", payload, offset)
              offset += 8
              if tag == 1 and size >= 12:
                spec, timestamp = struct.unpack_from("<id", payload, offset)
                if spec & 1:
                  times.append(timestamp)
                  frames["state"] += 1
              offset += size

        def send(kind, payload):
          if kind == "state" and payload == "state_ack":
            frames["state_ack"] += 1
          elif kind == "ui" and isinstance(payload, bytes) and payload:
            frames["ui_sent"] += 1

        def connect(websocket):
          for kind in ("state", "ui"):
            if f"/{kind}" in websocket.url:
              websocket.on("framereceived", functools.partial(receive, kind))
              websocket.on("framesent", functools.partial(send, kind))

        page.on("websocket", connect)
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.on("console", lambda message: errors.append(message.text)
                if message.type == "error" else None)
        page.goto(url)
        page.wait_for_function(
            "typeof Module !== 'undefined' && typeof Module.startApp === 'function'",
            timeout=120000,
        )
        deadline = time.monotonic() + 120
        while (frames["state"] < 2 or frames["state_ack"] < 2
               or not frames["ui_received"] or not frames["ui_sent"]
               or times[-1] <= times[0]):
          if errors:
            raise RuntimeError("; ".join(errors))
          if time.monotonic() >= deadline:
            raise TimeoutError(f"Studio browser streams did not start: {frames}")
          page.wait_for_timeout(100)
        screenshot = page.locator("canvas").screenshot()
        pixels = Image.open(io.BytesIO(screenshot)).convert("RGB")
        if max(high - low for low, high in pixels.getextrema()) < 20:
          raise RuntimeError("Studio rendered a uniform canvas")
        if args.screenshot:
          args.screenshot.write_bytes(screenshot)
        if errors:
          raise RuntimeError("; ".join(errors))
        print(f"Chromium {browser.version}: Studio rendered; {frames}; state times {times[:2]}")
        browser.close()
    except BaseException:
      print(log_path.read_text(), file=sys.stderr)
      raise
    finally:
      try:
        if os.name == "posix":
          os.killpg(process.pid, signal.SIGINT)
        elif process.poll() is None:
          process.terminate()
      except ProcessLookupError:
        pass
      if process.poll() is None:
        try:
          process.wait(timeout=15)
        except subprocess.TimeoutExpired:
          if os.name == "posix":
            os.killpg(process.pid, signal.SIGKILL)
          else:
            process.kill()
          process.wait()


if __name__ == "__main__":
  main()
