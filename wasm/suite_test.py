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

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from python.runfiles import runfiles


if __name__ == "__main__":
  resolver = runfiles.Create()
  paths = [resolver.Rlocation(path) for path in sys.argv[1:]]
  if not all(paths):
    raise RuntimeError(f"Missing suite inputs: {sys.argv[1:]}")
  node, runner, loader, typescript, metadata, config = paths
  with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    shutil.copytree(Path(runner).parent, root / "tests")
    shutil.copyfile(metadata, root / "package.json")
    shutil.copyfile(config, root / "tsconfig.json")
    (root / "node_modules").symlink_to(Path(typescript).resolve().parent.parent)
    (root / "dist").mkdir()
    for suffix in [".js", ".wasm", ".wasm.map", ".d.ts"]:
      source = Path(loader).with_name("mujoco" + suffix)
      shutil.copyfile(source, root / "dist" / source.name)
    subprocess.run(
        [node, "--loader", str(root / "node_modules/ts-node/esm.mjs"),
         "--trace-warnings", "tests/run_tests.mjs"],
        cwd=root,
        check=True,
    )
