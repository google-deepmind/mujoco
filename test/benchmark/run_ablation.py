#!/usr/bin/env python3
# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A script for running step_benchmark_test for a variety of build settings."""

import argparse
import os
import re
import shutil
import subprocess
import sys
from typing import NamedTuple, Tuple


class _Setting(NamedTuple):
  name: str
  cmake_args: Tuple[str]


def run_benchmark_variants(output_directory: str, build_directory: str,
                           benchmark_repetitions: int, delete_build: bool):
  """Builds different variants of the benchmark test, and runs them."""
  # For each of the settings, the first option is what we use as a baseline.
  # We chose to pick the settings that were likely best (so this is an ablation
  # study), so interactions between settings are tested.
  settings_ranges = {
      "avx": [
          _Setting("on", ("-DMUJOCO_ENABLE_AVX=ON",)),
          _Setting("off", ("-DMUJOCO_ENABLE_AVX=OFF",))
      ],
      "avx_intrinsics": [
          _Setting("on", ("-DMUJOCO_ENABLE_AVX_INTRINSICS=ON",)),
          _Setting("off", ("-DMUJOCO_ENABLE_AVX_INTRINSICS=OFF",))
      ],
      "lto": [
          _Setting("on", ("-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON",)),
          _Setting("off", ("-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF",)),
      ],
      "compiler": compiler_cmake_flags_variants(),
  }

  baseline_settings = {
      key: value[0] for key, value in settings_ranges.items()
  }
  run_benchmark(
      f"{sys.platform}_baseline",
      baseline_settings,
      output_directory=output_directory,
      build_directory=f"{build_directory}_baseline",
      benchmark_repetitions=benchmark_repetitions,
      delete_build=delete_build,
  )

  for key in sorted(settings_ranges):
    settings = baseline_settings.copy()
    for setting in settings_ranges[key][1:]:
      settings[key] = setting
      run_benchmark(
          f"{sys.platform}_{key}_{setting.name}",
          settings,
          output_directory=output_directory,
          build_directory=f"{build_directory}_{key}_{setting.name}",
          benchmark_repetitions=benchmark_repetitions,
          delete_build=delete_build,
      )


def run_benchmark(name, settings, output_directory: str, build_directory: str,
                  benchmark_repetitions: int, delete_build: bool):
  """Builds and runs a single benchmark."""
  current_dir = os.getcwd()
  try:
    if delete_build:
      shutil.rmtree(build_directory, ignore_errors=True)
    os.makedirs(build_directory, exist_ok=True)
    os.makedirs(output_directory, exist_ok=True)
    os.chdir(build_directory)
    cmake_args = [
        "cmake", "..", "-DMUJOCO_BUILD_TESTS=ON", "-DCMAKE_BUILD_TYPE=Release"
    ]
    cmake_args.extend(os_specific_cmake_flags())
    for _, setting in sorted(settings.items()):
      cmake_args.extend(setting.cmake_args)

    print(f"Running ({name}):", " ".join(cmake_args))
    subprocess.check_call(cmake_args)
    subprocess.check_call([
        "cmake", "--build", ".", "-j8", "--config=Release", "-t",
        "step_benchmark_test"
    ])
    output_path = os.path.join(output_directory, f"{name}.json")

    os.chdir("../mujoco/test")
    if sys.platform == "win32":
      cygwin_run_benchmark(build_directory, output_path, benchmark_repetitions)
    else:
      binary_path = os.path.join(build_directory, "bin", "step_benchmark_test")
      subprocess.check_call([
          binary_path,
          "--benchmark_filter=all",
          "--benchmark_enable_random_interleaving=true",
          "--benchmark_min_time=0.5",
          f"--benchmark_repetitions={benchmark_repetitions}",
          "--benchmark_format=json",
          f"--benchmark_out={output_path}",
      ])
  finally:
    os.chdir(current_dir)


def os_specific_cmake_flags():
  """CMake args that should be passed to all benchmarks on current OS."""
  if sys.platform == "win32":
    return ("-A", "x64", "-Thost=x86",
            "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded",
            "-DCMAKE_SYSTEM_VERSION=10.0.19041.0")
  elif sys.platform.startswith("linux"):
    return ()
  elif sys.platform == "darwin":
    # macOS
    return ()
  else:
    raise ValueError(f"Unknown OS: {sys.platform}")


def compiler_cmake_flags_variants():
  """Returns a list of benchmark settings for different compilers."""
  if sys.platform == "win32":
    return [
        _Setting("VS2022", ("-G", "Visual Studio 17 2022")),
        # MSVC 2017 doesn't seem to support C17 standard.
        # _Setting("VS2017", ("-G", "Visual Studio 15 2017")),
    ]
  elif sys.platform.startswith("linux"):
    return [
        _Setting("clang-11",
                 ("-DCMAKE_C_COMPILER=clang-11",
                  "-DCMAKE_CXX_COMPILER=clang++-11", "-DCMAKE_LINKER=lld-11")),
        _Setting("clang-8",
                 ("-DCMAKE_C_COMPILER=clang-8",
                  "-DCMAKE_CXX_COMPILER=clang++-8", "-DCMAKE_LINKER=lld-8")),
    ]
  elif sys.platform == "darwin":
    # macOS
    return [_Setting("clang", ())]
  else:
    raise ValueError(f"Unknown OS: {sys.platform}")


def cygwin_run_benchmark(build_directory, output_path, benchmark_repetitions):
  """Runs the benchmark command under the cygwin environment."""
  # The subprocess module interacts badly with cygwin.
  # Rather than trying to run the binary directly through Popen, use bash on
  # cygwin.
  lib_path = cygwin_path(os.path.join(build_directory, "lib", "Release"))
  binary_path = cygwin_path(
      os.path.join(build_directory, "bin", "Release",
                   "step_benchmark_test"))
  cygwin = subprocess.Popen(["bash"], stdin=subprocess.PIPE)
  command = (
      f'PATH="$PATH:{lib_path}" {binary_path} --benchmark_filter=all '
      "--benchmark_enable_random_interleaving=true ",
      "--benchmark_min_time=0.5 "
      f"--benchmark_repetitions={benchmark_repetitions} "
      "--benchmark_format=json "
      f"--benchmark_out='{output_path}'")
  cygwin.communicate(input=bytes(command, "utf-8"))
  if cygwin.returncode:
    raise ValueError(f"Benchmark returned error code: {cygwin.returncode}")


def cygwin_path(windows_path):
  path = windows_path.replace("\\", "/")
  path = re.sub(r".*cygwin64", "", path)
  return path


def main(argv):
  parser = argparse.ArgumentParser()
  parser.add_argument("-o", "--output_directory", default="benchmark_results")
  parser.add_argument(
      "-b",
      "--build_directory",
      default="build",
      help="Base path for build directories. Benchmark names will be appended."
  )
  parser.add_argument("-n", "--repetitions", type=int, default=30)
  parser.add_argument(
      "-d",
      "--delete_build",
      nargs="?",
      type=bool,
      const=True,
      default=False,
      help="Delete the build directory before building benchmark.")
  args = parser.parse_args(argv[1:])
  run_benchmark_variants(
      output_directory=os.path.abspath(args.output_directory),
      build_directory=os.path.abspath(args.build_directory),
      delete_build=args.delete_build,
      benchmark_repetitions=args.repetitions)


if __name__ == "__main__":
  main(sys.argv)
