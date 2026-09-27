"""Detect drift between the CMake and Bazel core source lists and archive pins."""

import pathlib
import re


def main():
  root = pathlib.Path(__file__).resolve().parents[1]
  version = re.search(r"project\(\s*mujoco\s+VERSION\s+([\d.]+)",
                      (root / "CMakeLists.txt").read_text()).group(1)
  for filename, pattern in [
      ("MODULE.bazel", r'\bversion\s*=\s*"([\d.]+)"'),
      ("python/BUILD.bazel", r'\bversion\s*=\s*"([\d.]+)"'),
      ("mjx/BUILD.bazel", r'\bversion\s*=\s*"([\d.]+)"'),
      ("bazel/package/BUILD.bazel", r'package_dir\s*=\s*"mujoco-([\d.]+)"'),
      ("bazel/package/mujocoConfigVersion.cmake", r'set\(PACKAGE_VERSION "([\d.]+)"'),
  ]:
    declared = re.search(pattern, (root / filename).read_text())
    if not declared or declared.group(1) != version:
      raise SystemExit(f"Package version drift: {filename}, expected {version}")
  cmake_sources = set()
  for directory in ["src/engine", "src/user", "src/xml", "src/xml/mjz", "src/render/classic", "src/ui"]:
    text = (root / directory / "CMakeLists.txt").read_text()
    cmake_sources.update(directory + "/" + name for name in re.findall(r"(?<![\w/])([\w/]+\.(?:c|cc))(?=[\s)])", text))
  cmake_sources.update(["plugin/obj_decoder/obj_decoder.cc", "plugin/stl_decoder/stl_decoder.cc"])
  build = (root / "BUILD.bazel").read_text().split("mujoco_core(", 1)[1].split("\ncc_library(", 1)[0]
  bazel_sources = set(re.findall(r'"([^"\n]+\.(?:c|cc))"', build))
  if cmake_sources != bazel_sources:
    raise SystemExit(f"Core source drift: missing={sorted(cmake_sources - bazel_sources)}, extra={sorted(bazel_sources - cmake_sources)}")
  cmake = (root / "cmake/MujocoDependencies.cmake").read_text()
  cmake += (root / "cmake/third_party_deps/lodepng.cmake").read_text()
  repositories = (root / "bazel/repositories.bzl").read_text()
  # BCR release revisions must track the CMake source pins.
  bcr_pins = {
      "lodepng": ("lodepng", "0.0.0-20250506-17d08dd", "17d08dd26cac4d63f43af217ebd70318bfb8189c"),
      "MarchingCubeCpp": ("marchingcubecpp", "0.0.0-20230911-f03a1b3", "f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d"),
      "miniz": ("miniz", "3.1.1", "d10b03cc73475af673df40f06e5cefd1d5f940d9"),
  }
  modules = dict(re.findall(
      r'bazel_dep\(name = "([^"]+)", version = "([^"]+)"',
      (root / "MODULE.bazel").read_text(),
  ))
  for dependency, revision in re.findall(r"set\(MUJOCO_DEP_VERSION_(\w+)\s+([0-9a-f]{40})", cmake):
    if dependency in {"abseil", "gtest"}:
      continue
    if dependency in bcr_pins:
      module, version, expected_revision = bcr_pins[dependency]
      if revision != expected_revision or modules.get(module) != version:
        raise SystemExit(f"BCR dependency pin drift: {dependency} at {revision}")
      continue
    if revision not in repositories:
      raise SystemExit(f"Dependency pin drift: {dependency} at {revision}")
  optional = (root / "bazel/optional/repositories.bzl").read_text()
  for path in sorted((root / "cmake/third_party_deps").glob("*.cmake")):
    for dependency, revision in re.findall(
        r"set\(MUJOCO_DEP_VERSION_(\w+)\s+([^\s#)]+)", path.read_text()
    ):
      if dependency in bcr_pins:
        continue
      if revision not in repositories and revision not in optional:
        raise SystemExit(f"Dependency pin drift: {dependency} at {revision}")
  print("Package versions, core sources, and archive pins match CMake.")


if __name__ == "__main__":
  main()
