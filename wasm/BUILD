load("//third_party/bazel_rules/rules_cc/cc:cc_library.bzl", "cc_library")

cc_library(
    name = "unpack",
    srcs = ["unpack.cc"],
    hdrs = ["unpack.h"],
    copts = [
        "-fexceptions",
    ],
    # doesn't build without emscripten support, is tested by the Wasm build
    # rule, see go/wasm-cc-binary-tags
    tags = [
        "notap",
        "requires_wasm_config",
    ],
    visibility = ["//third_party/mujoco:visibility"],
    deps = [
        "//third_party/emscripten:embind",  # buildcleaner: keep
        "//third_party/mujoco/src/engine:engine_util_errmem",
    ],
)
