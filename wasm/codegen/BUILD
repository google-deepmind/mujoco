load("//devtools/python/blaze:pytype.bzl", "pytype_strict_binary", "pytype_strict_library")

pytype_strict_binary(
    name = "update",
    srcs = ["update.py"],
    deps = [
        ":binding_builder",
        "//third_party/mujoco/wasm/codegen/helpers:common",
    ],
)

pytype_strict_library(
    name = "binding_builder",
    srcs = ["binding_builder.py"],
    data = [
        "//third_party/mujoco/wasm/codegen/templates:template_files",
    ],
    visibility = ["//third_party/mujoco:visibility"],
    deps = [
        "//third_party/mujoco/wasm/codegen/generators:enums",
        "//third_party/mujoco/wasm/codegen/generators:functions",
        "//third_party/mujoco/wasm/codegen/generators:structs",
        "//third_party/mujoco/wasm/codegen/helpers:common",
        "//third_party/mujoco/wasm/codegen/helpers:constants",
        "//third_party/mujoco/wasm/codegen/helpers:function_utils",
        "//third_party/py/mujoco/introspect:enums",
        "//third_party/py/mujoco/introspect:functions",
    ],
)
