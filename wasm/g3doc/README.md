# MuJoCo WASM Bindings

<!--*
# Document freshness: For more information, see go/fresh-source.
freshness: { owner: 'manevi' reviewed: '2025-07-31' }
*-->

This package is the WASM bindings for the
[MuJoCo physics engine](https://github.com/google-deepmind/mujoco). These
bindings are developed and maintained by Google DeepMind, and are kept up-to-date
with the latest developments in MuJoCo itself.

The `mujoco` package provides direct access to raw MuJoCo C API functions,
structs, constants, and enumerations. Structs are provided as Javascript classes.

> **Warning:** The bindings are under development phase and should not be considered production-ready.

## Build

Generate C++ bindings code

```sh
blaze run //third_party/mujoco/wasm/codegen:update
```

Build MuJoCo WASM bindings

```sh
blaze build //third_party/mujoco/wasm/codegen/generated:bindings
```

Check that TypeScript definitions file and see what's generated.

```sh
ls blaze-bin/third_party/mujoco/wasm/codegen/generated/bindings
```

Build bindings typescript

```sh
blaze build //third_party/mujoco/wasm/codegen/generated:wasm_types
```

## THREE.js demo

Run the following and visit the logged webpage to see a simulation running in
the browser using THREE.js for visualization.

```sh
iblaze run //third_party/mujoco/wasm/demo_app:app_server
```

Note: By using iblaze you get hot-reloading ie updates to the webpage when any
relevant build target changes. The link to where the webapp is running (Web Dev
Server) will be printed near the start of the iblaze logs. If you refresh the
webpage everything is rerun and the demo script is executed from the start.

## Manual browser tests

Use the following target to run manual tests in the browser, inspect the console
logs.

```sh
iblaze run //third_party/mujoco/wasm/tests/sandbox:devserver
```

### Testing

Make sure the wasm bindings are up-to-date:
```sh
blaze run //third_party/mujoco/wasm/codegen:update
```

Then run all the tests using:

```sh
blaze test //third_party/mujoco/wasm/...  --build_tests_only --test_output=errors
```

Use `--test_output=streamed` if you want to see benchmark results on the logs.

Emscripten aggressively optimizes code, so you should also run the tests with
optimization enabled:

```sh
blaze test //third_party/mujoco/wasm/... --compilation_mode=opt --build_tests_only --test_output=errors
```

