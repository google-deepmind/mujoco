#!/usr/bin/env python3
"""Check rendered GLFW windows and orderly shutdown under an X11 display."""

import argparse
import ctypes
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time


class MessageData(ctypes.Union):
  _fields_ = [
      ("bytes", ctypes.c_char * 20),
      ("shorts", ctypes.c_short * 10),
      ("longs", ctypes.c_long * 5),
  ]


class ClientMessage(ctypes.Structure):
  _fields_ = [
      ("type", ctypes.c_int),
      ("serial", ctypes.c_ulong),
      ("send_event", ctypes.c_int),
      ("display", ctypes.c_void_p),
      ("window", ctypes.c_ulong),
      ("message_type", ctypes.c_ulong),
      ("format", ctypes.c_int),
      ("data", MessageData),
  ]


class Event(ctypes.Union):
  _fields_ = [("client", ClientMessage), ("padding", ctypes.c_long * 24)]


def connect():
  x11 = ctypes.CDLL("libX11.so.6")
  signatures = {
      "XOpenDisplay": ([ctypes.c_char_p], ctypes.c_void_p),
      "XCloseDisplay": ([ctypes.c_void_p], ctypes.c_int),
      "XGetImage": ([ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                     ctypes.c_int, ctypes.c_uint, ctypes.c_uint,
                     ctypes.c_ulong, ctypes.c_int], ctypes.c_void_p),
      "XGetPixel": ([ctypes.c_void_p, ctypes.c_int, ctypes.c_int],
                    ctypes.c_ulong),
      "XDestroyImage": ([ctypes.c_void_p], ctypes.c_int),
      "XInternAtom": ([ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int],
                      ctypes.c_ulong),
      "XSendEvent": ([ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                      ctypes.c_long, ctypes.POINTER(Event)], ctypes.c_int),
      "XFlush": ([ctypes.c_void_p], ctypes.c_int),
  }
  for name, (arguments, result) in signatures.items():
    function = getattr(x11, name)
    function.argtypes = arguments
    function.restype = result
  connection = x11.XOpenDisplay(None)
  if not connection:
    raise RuntimeError("Cannot open DISPLAY; run this script with xvfb-run -a")
  return x11, connection


def rendered_window(x11, connection, process_id):
  tree = subprocess.check_output(
      ["xwininfo", "-root", "-tree"], text=True, timeout=5)
  window_id = None
  for candidate in re.findall(r"^\s+(0x[0-9a-f]+)\s", tree, re.MULTILINE):
    properties = subprocess.check_output(
        ["xprop", "-id", candidate, "_NET_WM_PID"], text=True, timeout=5)
    if re.search(r"=\s*" + str(process_id) + r"\s*$", properties):
      window_id = candidate
      break
  if window_id is None:
    return None
  window = int(window_id, 16)
  info = subprocess.check_output(
      ["xwininfo", "-id", window_id], text=True, timeout=5)
  if "Map State: IsViewable" not in info:
    return None
  width = int(re.search(r"Width: (\d+)", info)[1])
  height = int(re.search(r"Height: (\d+)", info)[1])
  frame = x11.XGetImage(connection, window, 0, 0, width, height,
                        ctypes.c_ulong(-1).value, 2)
  if not frame:
    return None
  try:
    pixels = {
        x11.XGetPixel(frame, x, y)
        for x in range(0, width, max(1, width // 64))
        for y in range(0, height, max(1, height // 64))
    }
  finally:
    x11.XDestroyImage(frame)
  if len(pixels) <= 16:
    return None
  return window, width, height, len(pixels)


def check(binary, model, arguments, x11, connection):
  with tempfile.TemporaryDirectory(prefix="mujoco-gui-") as directory:
    process = subprocess.Popen(
        [str(binary), str(model), *arguments], cwd=directory,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
      deadline = time.monotonic() + 30
      while time.monotonic() < deadline:
        if process.poll() is not None:
          stdout, stderr = process.communicate()
          raise RuntimeError(f"{binary} exited before rendering:\n{stdout}\n{stderr}")
        rendered = rendered_window(x11, connection, process.pid)
        if rendered:
          break
        time.sleep(0.2)
      else:
        process.kill()
        stdout, stderr = process.communicate()
        raise RuntimeError(
            f"{binary} did not render a nonuniform window:\n{stdout}\n{stderr}")
      window, width, height, colors = rendered
      event = Event()
      event.client = ClientMessage(
          type=33, send_event=1, display=connection, window=window,
          message_type=x11.XInternAtom(connection, b"WM_PROTOCOLS", 0),
          format=32)
      event.client.data.longs[0] = x11.XInternAtom(
          connection, b"WM_DELETE_WINDOW", 0)
      if not x11.XSendEvent(connection, window, 0, 0, ctypes.byref(event)):
        raise RuntimeError(f"Cannot request orderly shutdown of {binary}")
      x11.XFlush(connection)
      stdout, stderr = process.communicate(timeout=15)
      if process.returncode:
        raise RuntimeError(
            f"{binary} exited with {process.returncode}:\n{stdout}\n{stderr}")
      print(f"{binary}: rendered {width}x{height}, {colors} sampled colors;"
            " clean window close")
    finally:
      if process.poll() is None:
        process.kill()
        process.wait()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--model", type=Path, default=(
      Path(__file__).resolve().parents[1] / "model/humanoid/humanoid.xml"))
  parser.add_argument("binaries", nargs="+", type=Path)
  parser.add_argument("--argument", action="append", default=[])
  args = parser.parse_args()
  os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
  x11, connection = connect()
  try:
    for binary in args.binaries:
      check(binary.resolve(strict=True), args.model.resolve(strict=True),
            args.argument, x11, connection)
  finally:
    x11.XCloseDisplay(connection)


if __name__ == "__main__":
  main()
