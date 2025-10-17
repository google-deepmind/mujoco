# MuJoCo WASM

## Prerequisites

- Have a stable version of `node` and `npm` installed, we recommend [nvm](https://github.com/nvm-sh/nvm) to manage that.
- Have a `gcc` version 10 or higher (needs to support -std=c++20).
- Have `python`, `pytest` and `cmake` installed.
- Install Javascript dependencies running `npm install` inside the wasm folder. These dependencies are for all JS/TS targets: test suite, demo apps and bindings build process.
- Install Emscripten SDK ([emsdk](https://emscripten.org/docs/getting_started/downloads.html)), we recommend you follow these steps:

    ```sh
    # Get the emsdk repo and cd into it
    git clone https://github.com/emscripten-core/emsdk.git
    cd emsdk
    ```
    Version `4.0.10` must be installed, later versions do not build, read [this](https://github.com/ekumenlabs/mujoco_internal/pull/44#issuecomment-3339343789) for more details

    ```sh
    # Run these inside `emsdk` folder
    ./emsdk install 4.0.10
    ./emsdk activate 4.0.10
    source ./emsdk_env.sh
    ```

> [!IMPORTANT]
> Make sure to run the following steps in the same terminal session in which the `emsdk` environment was sourced.

> [!TIP]
> Windows users: Please run the following commands in a Git Bash terminal.

## Bindings generation

The following script will use MuJoCo's python introspect library to generate a C++ file with the relevant Embind `EMSCRIPTEN_BINDINGS` block which is used to bind C++ functions and classes to Javascript. The script will also generate wrapper functions around MuJoCo's C API which provide a place for adding conveniences like bounds checking, etc.

```sh
# Run this inside `wasm` folder
[ ! -d ".venv" ] && python3 -m venv .venv && source .venv/bin/activate; && \
PYTHONPATH=../python/mujoco:./codegen python codegen/update.py && \
PYTHONPATH=../python/mujoco:../wasm python tests/enums_test_generator.py && \
PYTHONPATH=../python/mujoco:./codegen pytest
```

Once the C++ files are generated (note that for convenience we provide the output of the above script in the `wasm/codegen/generated` folder already) the next step is to build the `.wasm`, `.js`, and `.d.ts` files from the C++ files (which are the files you will call MuJoCo from JavaScript/TypeScript). To do this run the following:

> [!TIP]
> Before running these make sure no `build` and `dist` folder exist in the root of the project or inside `wasm` folder.

```sh
# Run this inside `wasm` folder
WASM_DIR=$(pwd) && PROJECT_ROOT=$(dirname "$WASM_DIR") && \
export PATH="${WASM_DIR}/node_modules/.bin:$PATH" && \
emcmake cmake -S "$PROJECT_ROOT" -B "$PROJECT_ROOT/build" && cmake --build "$PROJECT_ROOT/build" && \
emcmake cmake -S "$WASM_DIR" -B "$WASM_DIR/build" && cmake --build "$WASM_DIR/build"
```

This will generate a few folders:
- `<project-root>/build` is the result of compiling MuJoCo with emscripten.
- `<project-root>/wasm/build` is the result of compiling MuJoCo bindings with emscripten.
- `<project-root>/wasm/dist` is MuJoCo WebAssembly module. To use it, the file you need to import here is the one ended in `.js`.

## Tests

The test suite will run three kind of tests:
1. Enums bindings: assess all MuJoCo enums are declared.
2. Function bindings: verifies a wide variety of MuJoCo functions and classes work correctly when called from Javascript.
3. Benchmarks: measures shared memory buffer management approaches.
Run the following commands (the benchmark bindings code is compilied first):
```sh
# Run this inside `wasm` folder
WASM_DIR=$(pwd) && \
export PATH="${WASM_DIR}/node_modules/.bin:$PATH" && \
emcmake cmake -S "$WASM_DIR/tests" -B "$WASM_DIR/tests/build" && \
cmake --build "$WASM_DIR/tests/build" && \
npm run test
```

## Give it a try

- To run the demo app use `npm run dev:demo`. This is a use case scenario with a small ThreeJS app using MuJoCo WASM module.
- To run the sandbox app use `npm run dev:sandbox`. This is a playground app meant for you to experiment any MuJoCo functionality you want in the browser.
