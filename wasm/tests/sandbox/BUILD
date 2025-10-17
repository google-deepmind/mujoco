load("//javascript/typescript:build_defs.bzl", "ts_config", "ts_development_sources", "ts_library")
load("//tools/build_defs/js/devserver:web_dev_server.bzl", "web_dev_server")

ts_library(
    name = "demo",
    srcs = [
        "main.ts",
    ],
    deps = ["//third_party/mujoco/wasm/codegen/generated:wasm_types"],
)

ts_config(
    name = "tsconfig",
    deps = [
        ":demo",
    ],
)

ts_development_sources(
    name = "devsrcs",
    deps = [":demo"],
)

ts_development_sources(
    name = "devsrcs_test",
    testonly = True,
    deps = [],
)

web_dev_server(
    name = "devserver",
    concatjs_routes = {":devsrcs": "/app_bundle.js"},
    static_files = [
        "index.html",
        "//third_party/mujoco/wasm/codegen/generated:bindings",
    ],
)
