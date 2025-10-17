load("//devtools/python/blaze:pytype.bzl", "pytype_strict_library")
load(
    "//devtools/python/blaze:strict.bzl",
    "py_strict_test",
)

pytype_strict_library(
    name = "common",
    srcs = ["common.py"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = [":constants"],
)

py_strict_test(
    name = "common_test",
    srcs = ["common_test.py"],
    deps = [
        ":common",
        ":constants",
        "//testing/pybase",
        "//third_party/py/absl/testing:absltest",
    ],
)

pytype_strict_library(
    name = "function_utils",
    srcs = ["function_utils.py"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = [
        ":code_builder",
        ":common",
        ":constants",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)

py_strict_test(
    name = "function_utils_test",
    srcs = ["function_utils_test.py"],
    deps = [
        ":constants",
        ":function_utils",
        "//third_party/py/absl/testing:absltest",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)

pytype_strict_library(
    name = "code_builder",
    srcs = ["code_builder.py"],
    visibility = ["//third_party/mujoco:visibility"],
)

pytype_strict_library(
    name = "structs_wrappers_data",
    srcs = ["structs_wrappers_data.py"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = [":common"],
)

pytype_strict_library(
    name = "struct_field_handler",
    srcs = ["struct_field_handler.py"],
    deps = [
        ":common",
        ":constants",
        ":struct_field_code_builder",
        ":structs_wrappers_data",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)

py_strict_test(
    name = "struct_field_handler_test",
    srcs = ["struct_field_handler_test.py"],
    deps = [
        ":struct_field_handler",
        "//third_party/py/absl/testing:absltest",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)

pytype_strict_library(
    name = "struct_field_code_builder",
    srcs = ["struct_field_code_builder.py"],
    deps = [
        ":code_builder",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)

pytype_strict_library(
    name = "structs_parser",
    srcs = ["structs_parser.py"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = [
        ":common",
        ":constants",
        ":struct_constructor_code_builder",
        ":struct_field_handler",
        ":structs_wrappers_data",
        "//third_party/py/mujoco/introspect:ast_nodes",
        "//third_party/py/mujoco/introspect:structs",
    ],
)

pytype_strict_library(
    name = "struct_constructor_code_builder",
    srcs = ["struct_constructor_code_builder.py"],
    deps = [
        ":code_builder",
        ":common",
        ":constants",
        ":structs_wrappers_data",
        "//third_party/py/mujoco/introspect:ast_nodes",
        "//third_party/py/mujoco/introspect:structs",
    ],
)

pytype_strict_library(
    name = "constants",
    srcs = ["constants.py"],
    visibility = ["//third_party/mujoco:visibility"],
    deps = ["//third_party/py/mujoco/introspect:structs"],
)

py_strict_test(
    name = "struct_field_code_builder_test",
    srcs = ["struct_field_code_builder_test.py"],
    deps = [
        ":struct_field_code_builder",
        "//third_party/py/absl/testing:absltest",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)

py_strict_test(
    name = "code_builder_test",
    srcs = ["code_builder_test.py"],
    deps = [
        ":code_builder",
        "//third_party/py/absl/testing:absltest",
    ],
)

py_strict_test(
    name = "structs_parser_test",
    srcs = ["structs_parser_test.py"],
    deps = [
        ":structs_parser",
        ":structs_wrappers_data",
        "//testing/pybase",
        "//third_party/py/absl/testing:absltest",
        "//third_party/py/mujoco/introspect:ast_nodes",
        "//third_party/py/mujoco/introspect:structs",
    ],
)

py_strict_test(
    name = "struct_constructor_code_builder_test",
    srcs = ["struct_constructor_code_builder_test.py"],
    deps = [
        ":struct_constructor_code_builder",
        ":struct_field_handler",
        "//third_party/py/absl/testing:absltest",
        "//third_party/py/mujoco/introspect:ast_nodes",
    ],
)
