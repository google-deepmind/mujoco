load("//devtools/python/blaze:strict.bzl", "py_strict_test")
load("//javascript/typescript:build_defs.bzl", "ts_declaration")
load("//third_party/bazel_rules/rules_cc/cc:cc_binary.bzl", "cc_binary")
load("//third_party/bazel_rules/rules_cc/cc:cc_library.bzl", "cc_library")
load("//third_party/bazel_rules/rules_wasm/wasm:defs.bzl", "wasm_cc_binary")

cc_library(
    name = "bindings_h",
    hdrs = ["bindings.h"],
    tags = [
        "notap",
        "requires_wasm_config",
    ],
    visibility = ["//third_party/mujoco:visibility"],
    deps = ["//third_party/mujoco:libmujoco_hdrs"],
)

cc_binary(
    name = "mujoco",
    srcs = ["bindings.cc"],
    copts = [
        "-fexceptions",
    ],
    features = ["-use_header_modules"],
    linkopts = [
        "-sALLOW_MEMORY_GROWTH=1",
        "-sFILESYSTEM=1",
        # Emitted code is wrapped in a function returning a module promise rather than global scope.
        "-sMODULARIZE=1",
        "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','FS','MEMFS']",
        # If using MODULARIZE, the name of the function to call to get the module promise.
        "-sEXPORT_NAME=loadMujoco",
        #"-g--emit-tsd",
        "-sMINIMAL_RUNTIME=0",
        "-sWASM_BIGINT=1",
    ],
    # doesn't build without emscripten support, is tested by the Wasm build
    # rule, see go/wasm-cc-binary-tags
    tags = [
        "notap",
        "requires_wasm_config",
    ],
    deps = [
        ":bindings_h",
        "//third_party/emscripten:embind",  # buildcleaner: keep
        "//third_party/mujoco:mujoco_no_gl",
        "//third_party/mujoco/src/engine:engine_util_errmem",
        "//third_party/mujoco/wasm:unpack",
    ],
)

wasm_cc_binary(
    name = "bindings",
    cc_target = ":mujoco",
    exception_style = "emscripten",
    exceptions = "enabled",
    generate_ts_defs = "embind",
    include_data_dependencies = True,
    visibility = ["//third_party/mujoco:visibility"],
)

ts_declaration(
    name = "wasm_types",
    srcs = [
        ":bindings/mujoco.d.ts",
    ],
    visibility = ["//third_party/mujoco:visibility"],
    deps = ["//third_party/javascript/typings/emscripten"],
)

py_strict_test(
    name = "bindings_diff_test",
    srcs = ["bindings_diff_test.py"],
    data = [
        ":bindings.cc",
        ":bindings.h",
        "//third_party/mujoco/wasm/codegen/templates:bindings.cc",
        "//third_party/mujoco/wasm/codegen/templates:bindings.h",
    ],
    deps = [
        "//pyglib:resources",
        "//testing/pybase",
        "//third_party/mujoco/wasm/codegen:binding_builder",
        "//third_party/py/absl/testing:absltest",
    ],
)

py_strict_test(
    name = "coverage_test",
    srcs = ["coverage_test.py"],
    data = [
        "//third_party/mujoco/wasm/codegen/generated:bindings.cc",
        "//third_party/mujoco/wasm/codegen/generated:bindings.h",
    ],
    deps = [
        "//pyglib:resources",
        "//third_party/mujoco/wasm/codegen/helpers:common",
        "//third_party/mujoco/wasm/codegen/helpers:constants",
        "//third_party/mujoco/wasm/codegen/helpers:function_utils",
        "//third_party/py/absl/testing:absltest",
        "//third_party/py/mujoco/introspect:functions",
        "//third_party/py/mujoco/introspect:structs",
    ],
)
