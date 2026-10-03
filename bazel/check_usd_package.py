"""Load the extracted USD SDK libraries without Bazel runfiles."""

import argparse
import ctypes
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile


def check_combined(root):
  suffix = ".dylib" if sys.platform == "darwin" else ".so"
  libraries = [ctypes.CDLL(str(root / "lib" / (name + suffix)))
               for name in ("libmjcPhysics", "libusdMjcf")]
  core_path = next((root / "lib").glob("libmujoco.*"))
  core = ctypes.CDLL(str(core_path))
  signatures = {
      "mj_loadAllPluginLibraries": ([ctypes.c_char_p, ctypes.c_void_p], None),
      "mj_parse": ([ctypes.c_char_p, ctypes.c_char_p, ctypes.c_void_p,
                    ctypes.c_char_p, ctypes.c_int], ctypes.c_void_p),
      "mj_compile": ([ctypes.c_void_p, ctypes.c_void_p], ctypes.c_void_p),
      "mj_makeData": ([ctypes.c_void_p], ctypes.c_void_p),
      "mj_step": ([ctypes.c_void_p, ctypes.c_void_p], None),
      "mj_getState": ([ctypes.c_void_p, ctypes.c_void_p,
                       ctypes.POINTER(ctypes.c_double), ctypes.c_int], None),
      "mjs_getError": ([ctypes.c_void_p], ctypes.c_char_p),
      "mj_deleteSpec": ([ctypes.c_void_p], None),
      "mj_deleteModel": ([ctypes.c_void_p], None),
      "mj_deleteData": ([ctypes.c_void_p], None),
  }
  for name, (arguments, result) in signatures.items():
    function = getattr(core, name)
    function.argtypes = arguments
    function.restype = result
  core.mj_loadAllPluginLibraries(str(root / "bin/mujoco_plugin").encode(), None)
  model_path = root / "model.xml"
  model_path.write_text('''<mujoco><worldbody><body name="sphere" pos="0 0 1">
    <freejoint/><geom type="sphere" size="0.1"/>
    </body></worldbody></mujoco>''')
  error = ctypes.create_string_buffer(1024)
  spec = core.mj_parse(str(model_path).encode(), b"model/usd", None, error, len(error))
  if not spec:
    raise RuntimeError(error.value.decode())
  model = data = None
  try:
    model = core.mj_compile(spec, None)
    if not model:
      raise RuntimeError(core.mjs_getError(spec).decode())
    data = core.mj_makeData(model)
    if not data:
      raise RuntimeError("Cannot allocate simulation data")
    core.mj_step(model, data)
    time = ctypes.c_double()
    core.mj_getState(model, data, ctypes.byref(time), 1)
    if time.value <= 0:
      raise RuntimeError("USD model did not advance")
  finally:
    if data:
      core.mj_deleteData(data)
    if model:
      core.mj_deleteModel(model)
    core.mj_deleteSpec(spec)
  del libraries


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  source = parser.add_mutually_exclusive_group(required=True)
  source.add_argument("--archive", type=Path)
  source.add_argument("--extracted-root", type=Path, help=argparse.SUPPRESS)
  args = parser.parse_args()
  if args.extracted_root:
    check_combined(args.extracted_root)
    return
  with tempfile.TemporaryDirectory(prefix="mujoco-usd-sdk-") as directory:
    root = Path(directory)
    with tarfile.open(args.archive) as archive:
      archive.extractall(root, filter="data")
    suffix = ".dylib" if sys.platform == "darwin" else ".so"
    libraries = [
        root / "lib" / ("libmjcPhysics" + suffix),
        root / "lib" / ("libusdMjcf" + suffix),
        root / "bin/mujoco_plugin" / ("libusd_decoder_plugin" + suffix),
    ]
    for library in libraries:
      subprocess.run(
          [sys.executable, "-c", "import ctypes, sys; ctypes.CDLL(sys.argv[1])",
           str(library)], cwd=root, check=True, timeout=60)
    subprocess.run(
        [sys.executable, str(Path(__file__).resolve()), "--extracted-root", str(root)],
        cwd=root, check=True, timeout=60)
    print("USD SDK libraries loaded independently and together; model stepped")


if __name__ == "__main__":
  main()
