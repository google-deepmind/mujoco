# Copyright 2026 DeepMind Technologies Limited
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

"""Build rules for the MuJoCo Python extension modules."""

load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

def mujoco_pybind_extension(name, dynamic_deps = [], **kwargs):  # buildifier: disable=unused-variable
    """Builds a MuJoCo pybind11 extension module.

    In the Bazel build, every extension links the MuJoCo engine dynamically
    from //:mujoco_shared, so that all extension modules share one copy of the
    engine's global state (plugin registry, error handlers, allocators)
    instead of each linking a private static copy. Each extension exports only
    its PyInit_ function; pybind11 shares types between extensions through the
    Python interpreter, not through exported C++ symbols.

    Args:
      name: The name of the extension module.
      dynamic_deps: Additional cc_shared_library targets to link against. Only
        used by the Bazel build.
      **kwargs: Passed through to pybind_extension.
    """
    lds = Label("//python/mujoco:pybind_extension.lds")
    exported_symbols = Label("//python/mujoco:pybind_extension.exported_symbols")
    macos = Label("@platforms//os:macos")
    pybind_extension(
        name = name,
        additional_linker_inputs = select({
            macos: [exported_symbols],
            "//conditions:default": [lds],
        }),
        dynamic_deps = dynamic_deps + [Label("//:mujoco_shared")],
        linkopts = select({
            macos: ["-Wl,-exported_symbols_list,$(execpath %s)" % exported_symbols],
            "//conditions:default": ["-Wl,--version-script=$(execpath %s)" % lds],
        }),
        **kwargs
    )
