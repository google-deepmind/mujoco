"""Compile and run a CMake consumer against an extracted MuJoCo SDK."""

import argparse
import os
import pathlib
import subprocess
import tarfile
import tempfile


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--archive", type=pathlib.Path, required=True)
  parser.add_argument("--cmake", default="cmake")
  args = parser.parse_args()
  with tempfile.TemporaryDirectory(prefix="mujoco-sdk-") as directory:
    root = pathlib.Path(directory)
    with tarfile.open(args.archive) as archive:
      archive.extractall(root, filter="data")
    prefix = root / "mujoco-3.14.1"
    (root / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.16)
project(mujoco_consumer C CXX)
find_package(mujoco 3.14.1 REQUIRED CONFIG)
add_executable(consumer main.c)
target_link_libraries(consumer PRIVATE mujoco::mujoco)
add_executable(simulate_consumer simulate.cc)
target_link_libraries(simulate_consumer PRIVATE mujoco::libmujoco_simulate)
''')
    (root / "simulate.cc").write_text('''#define GLFW_INCLUDE_NONE
#include <mujoco/glfw_adapter.h>
#include <mujoco/simulate.h>
int main(int argc, char**) {
  void (mujoco::Simulate::* volatile render)() = &mujoco::Simulate::RenderLoop;
  if (argc == 123) {
    auto* adapter = new mujoco::GlfwAdapter;
    delete adapter;
  }
  return render == nullptr;
}
''')
    (root / "main.c").write_text('''#include <mujoco/mujoco.h>
int main(int argc, char** argv) {
  if (argc != 2 || mj_version() != mjVERSION_HEADER) return 1;
  int before = mjp_pluginCount();
  mj_loadAllPluginLibraries(argv[1], 0);
  if (mjp_pluginCount() <= before) return 2;
  return 0;
}
''')
    subprocess.run([args.cmake, "-S", str(root), "-B", str(root / "build"),
                    "-DCMAKE_PREFIX_PATH=" + str(prefix)], check=True)
    subprocess.run([args.cmake, "--build", str(root / "build"), "--config", "Release"], check=True)
    executable = root / "build/consumer"
    env = os.environ.copy()
    if os.name == "nt":
      executable = root / "build/Release/consumer.exe"
      if not executable.exists():
        executable = root / "build/consumer.exe"
      env["PATH"] = str(prefix / "bin") + os.pathsep + env.get("PATH", "")
    subprocess.run([str(executable), str(prefix / "bin/mujoco_plugin")], env=env, check=True)
    simulate_consumer = executable.with_name("simulate_consumer" + executable.suffix)
    subprocess.run([str(simulate_consumer)], env=env, check=True)
    compiler = prefix / "bin" / ("compile.exe" if os.name == "nt" else "compile")
    subprocess.run([str(compiler), str(prefix / "model/humanoid/humanoid.xml"),
                    str(root / "humanoid.mjb")], env=env, check=True)


if __name__ == "__main__":
  main()
