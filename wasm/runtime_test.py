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

import subprocess
import sys
import shutil
import tempfile
from pathlib import Path

from python.runfiles import runfiles


if __name__ == "__main__":
  resolver = runfiles.Create()
  paths = [resolver.Rlocation(path) for path in sys.argv[1:]]
  if not all(paths):
    raise RuntimeError(f"Missing runtime test inputs: {sys.argv[1:]}")
  node, script, loader, metadata = paths
  with tempfile.TemporaryDirectory() as directory:
    destination = Path(directory)
    for suffix in [".js", ".wasm", ".wasm.map", ".d.ts"]:
      source = Path(loader).with_name("mujoco" + suffix)
      shutil.copyfile(source, destination / source.name)
    shutil.copyfile(metadata, destination / "package.json")
    subprocess.run([node, script, str(destination / "mujoco.js")], check=True)
