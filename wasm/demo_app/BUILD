load("//javascript/typescript:build_defs.bzl", "ts_config", "ts_development_sources", "ts_library")
load("//tools/build_defs/js/devserver:web_dev_server.bzl", "web_dev_server")

ts_library(
    name = "app",
    srcs = [
        "app.ts",
    ],
    deps = [
        "//third_party/javascript/threejs/r134/examples/jsm/controls:orbitcontrols-noexterns-types",
        "//third_party/javascript/threejs/r134/typings",
        "//third_party/mujoco/wasm/codegen/generated:wasm_types",
    ],
)

ts_config(
    name = "tsconfig",
    deps = [":app"],
)

ts_development_sources(
    name = "app_srcs",
    deps = [":app"],
)

web_dev_server(
    name = "app_server",
    concatjs_routes = {":app_srcs": "/app_bundle.js"},
    static_files = [
        "index.html",
        "//third_party/mujoco/wasm/codegen/generated:bindings",
    ],
)
