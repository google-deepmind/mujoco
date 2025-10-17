load("//devtools/python/blaze:pytype.bzl", "pytype_strict_library")
load("//devtools/python/blaze:strict.bzl", "py_strict_test")

pytype_strict_library(
    name = "enums",
    srcs = ["enums.py"],
    data = ["//third_party/mujoco/wasm/codegen/templates:template_files"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = [
        "//third_party/mujoco/wasm/codegen/helpers:code_builder",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)

pytype_strict_library(
    name = "structs",
    srcs = ["structs.py"],
    data = ["//third_party/mujoco/wasm/codegen/templates:template_files"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = [
        "//third_party/mujoco/wasm/codegen/helpers:constants",
        "//third_party/mujoco/wasm/codegen/helpers:structs_parser",
        "//third_party/mujoco/wasm/codegen/helpers:structs_wrappers_data",
    ],
)

pytype_strict_library(
    name = "functions",
    srcs = ["functions.py"],
    data = ["//third_party/mujoco/wasm/codegen/templates:template_files"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = [
        "//third_party/mujoco/wasm/codegen/helpers:code_builder",
        "//third_party/mujoco/wasm/codegen/helpers:common",
        "//third_party/mujoco/wasm/codegen/helpers:function_utils",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)

pytype_strict_library(
    name = "error_handler",
    srcs = ["error_handler.py"],
    data = ["//third_party/mujoco/wasm/codegen/templates:template_files"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = ["//third_party/mujoco/wasm/codegen/helpers:common"],
)

pytype_strict_library(
    name = "constants",
    srcs = ["constants.py"],
    data = ["//third_party/mujoco/wasm/codegen/templates:template_files"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = ["//third_party/mujoco/wasm/codegen/helpers:common"],
)

py_strict_test(
    name = "enums_test",
    srcs = ["enums_test.py"],
    deps = [
        ":enums",
        "//third_party/py/absl/testing:absltest",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)

py_strict_test(
    name = "functions_test",
    srcs = ["functions_test.py"],
    deps = [
        ":functions",
        "//third_party/py/absl/testing:absltest",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)
