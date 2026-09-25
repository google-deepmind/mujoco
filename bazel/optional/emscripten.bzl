"""Declare the Emscripten compiler tools and their execution dependencies."""

load("@emsdk//emscripten_toolchain:platform_info.bzl", "platform_info")
load("@mujoco_emscripten//emscripten_toolchain:toolchain.bzl", "emscripten_cc_toolchain_config_rule", "emscripten_python_interpreter_files")
load("@rules_cc//cc:defs.bzl", "cc_toolchain")

def mujoco_emscripten_toolchain(name):
    """Populate the generated toolchain package with compiler and linker tools.

    Args:
        name: Target name for the C++ toolchain.
    """
    native.filegroup(name = "empty")
    platform_info(name = "platform_info", script_extension = ".sh")
    emscripten_python_interpreter_files(name = "python")
    native.filegroup(
        name = "common",
        srcs = [
            "default_config",
            "env.sh",
            ":python",
            "//:builtin_cache",
            "@rules_nodejs//nodejs:current_node_toolchain",
        ],
    )
    for group_name, srcs in [
        ("compiler", ["emcc.sh", "//:compiler_files"]),
        ("linker", ["emcc_link.sh", "link_wrapper.py", "//:linker_files"]),
        ("archiver", ["emar.sh", "//:ar_files"]),
    ]:
        native.filegroup(name = group_name, srcs = srcs + [":common"])
    native.filegroup(name = "all", srcs = [":compiler", ":linker", ":archiver"])
    emscripten_cc_toolchain_config_rule(
        name = "config",
        cpu = "wasm",
        em_config = "default_config",
        emscripten_binaries = "//:compiler_files",
    )
    cc_toolchain(
        name = name,
        all_files = ":all",
        ar_files = ":archiver",
        as_files = ":empty",
        compiler_files = ":compiler",
        dwp_files = ":empty",
        linker_files = ":linker",
        objcopy_files = ":empty",
        strip_files = ":empty",
        toolchain_config = ":config",
        toolchain_identifier = "mujoco-emscripten-4.0.10-linux-x86_64",
    )
