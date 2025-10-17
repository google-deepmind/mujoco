load("//devtools/python/blaze:pytype.bzl", "pytype_strict_binary")
load("//javascript/typescript:build_defs.bzl", "ts_declaration", "ts_library")
load("//testing/web/build_defs:web.bzl", "web_test_config")
load("//testing/web/build_defs/js_test:js_test_environment.bzl", "js_test_environment")
load("//testing/web/build_defs/js_test:js_test_suite.bzl", "js_test_suite")
load("//third_party/bazel_rules/rules_cc/cc:cc_binary.bzl", "cc_binary")
load("//third_party/bazel_rules/rules_wasm/wasm:defs.bzl", "wasm_cc_binary")

cc_binary(
    name = "mujoco_benchmarks",
    srcs = ["benchmark_test.cc"],
    copts = [
        "-fexceptions",
    ],
    features = ["-use_header_modules"],
    linkopts = [
        "-sALLOW_MEMORY_GROWTH=1",
        "-sFILESYSTEM=1",
        "-sMODULARIZE=1",
        "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','FS','MEMFS']",
        "-sEXPORT_NAME=loadMujoco",
        "-sMINIMAL_RUNTIME=0",
        "-sWASM_BIGINT=1",
    ],
    tags = [
        "notap",
        "requires_wasm_config",
    ],
    deps = [
        "//third_party/emscripten:embind",  # buildcleaner: keep
        "//third_party/mujoco:mujoco_no_gl",
        "//third_party/mujoco/wasm:unpack",
    ],
)

wasm_cc_binary(
    name = "benchmarks",
    cc_target = ":mujoco_benchmarks",
    exception_style = "emscripten",
    exceptions = "enabled",
    generate_ts_defs = "embind",
    include_data_dependencies = True,
    visibility = ["//third_party/mujoco:visibility"],
)

ts_declaration(
    name = "benchmarks_types",
    srcs = [
        ":benchmarks/mujoco_benchmarks.d.ts",
    ],
    visibility = ["//third_party/mujoco:visibility"],
    deps = ["//third_party/javascript/typings/emscripten"],
)

pytype_strict_binary(
    name = "enums_test_generator",
    srcs = ["enums_test_generator.py"],
    deps = [
        "//third_party/mujoco/wasm/codegen/helpers:common",
        "//third_party/py/mujoco/introspect:enums",
    ],
)

web_test_config(
    name = "karma_config",
    metadata = "karma.conf.json",
)

ts_library(
    name = "wasm_test_lib",
    testonly = True,
    srcs = [
        "bindings_test.ts",
        "enums_test.ts",
    ],
    deps = [
        "//javascript/common/asserts",
        "//testing/web/jasmine/parameterize",
        "//third_party/javascript/typings/jasmine",
        "//third_party/mujoco/wasm/codegen/generated:wasm_types",
    ],
)

ts_library(
    name = "benchmark_test_lib",
    testonly = True,
    srcs = [
        "benchmark_test.ts",
    ],
    deps = [
        ":benchmarks_types",
        "//javascript/common/asserts",
        "//testing/web/jasmine/parameterize",
        "//third_party/javascript/typings/jasmine",
    ],
)

js_test_environment(
    name = "chrome_linux",
    bootstrap_files = [
        "//third_party/mujoco/wasm/codegen/generated:bindings/mujoco.js",
    ],
    browser = "//testing/web/browsers:chrome-linux",
    static_files = [
        "//third_party/mujoco/wasm/codegen/generated:bindings",
    ],
)

js_test_environment(
    name = "chrome_linux_benchmark",
    bootstrap_files = [
        "//third_party/mujoco/wasm/tests:benchmarks/mujoco_benchmarks.js",
    ],
    browser = "//testing/web/browsers:chrome-linux",
    static_files = [
        "//third_party/mujoco/wasm/tests:benchmarks",
    ],
)

js_test_suite(
    name = "unit_tests",
    environments = [":chrome_linux"],
    scuba = False,
    tags = ["nofloat32"],
    test_lib = ":wasm_test_lib",
)

js_test_suite(
    name = "benchmark_tests",
    configs = [":karma_config"],
    environments = [":chrome_linux_benchmark"],
    scuba = False,
    tags = [
        "nofloat32",
        "notap",
    ],
    test_lib = ":benchmark_test_lib",
)
