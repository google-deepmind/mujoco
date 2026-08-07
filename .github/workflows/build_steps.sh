#!/bin/bash
# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# TODO(matijak): Make all cmake commands run from the top-level directory, and
# consider making the builds parallel.


# Wrap the compiler with ccache when it is available (set up by ccache-action in
# CI). This makes warm rebuilds - including the expensive, pinned Filament build -
# much faster. ccache is content-addressed on the full compiler invocation, so
# changing a flag or source forces a recompile: a stale object is never reused.
# Guarded by `command -v` so the script still works locally without ccache.
CCACHE_ARGS=""
if command -v ccache >/dev/null 2>&1; then
    CCACHE_ARGS="-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"
fi


# Emit the build matrix for build.yml as a step output. On pull_request we run
# only the representative "core" compiler set; on push (e.g. to main) we run the
# full compiler sweep. Tiers are defined in build_matrix.json.
generate_matrix() {
    echo "Generating build matrix for event '${GITHUB_EVENT_NAME}'..."
    local file=".github/workflows/build_matrix.json"
    local matrix
    if [[ "${GITHUB_EVENT_NAME}" == "pull_request" ]]; then
        matrix="$(jq -c '{include: [.include[] | select(.tier == "core") | del(.tier)]}' "${file}")"
    else
        matrix="$(jq -c '{include: [.include[] | del(.tier)]}' "${file}")"
    fi
    echo "matrix=${matrix}" >> "${GITHUB_OUTPUT}"
    echo "${matrix}" | jq .
}


prepare_linux() {
    echo "Preparing Linux..."
    sudo apt-get update && sudo apt-get install \
        libgl1-mesa-dev \
        libwayland-dev \
        libxinerama-dev \
        libxcursor-dev \
        libxkbcommon-dev \
        libxrandr-dev \
        libxi-dev \
        ninja-build
}


prepare_python() {
    echo "Preparing Python..."
    repo="${PWD}"
    pushd "${TMPDIR}" > /dev/null
    python -m venv venv
    if [[ $RUNNER_OS == "Windows" ]]; then
    mkdir venv/bin
    fixpath="$(s="$(cat venv/Scripts/activate | grep VIRTUAL_ENV=)"; echo "${s:13:-1}")"
    sed -i "s#$(printf "%q" "${fixpath}")#$(cygpath "${fixpath}")#g" venv/Scripts/activate
    ln -s ../Scripts/activate venv/bin/activate
    fi
    source venv/bin/activate
    # Install build deps with uv when available (set up by setup-uv in CI on
    # POSIX) - much faster than pip. Fall back to pip otherwise (e.g. Windows,
    # local dev). The venv is still created by `python -m venv`, so pip stays
    # available for later steps (pip wheel / python -m build).
    if command -v uv > /dev/null 2>&1; then
        uv pip install --require-hashes -r "${repo}/python/build_requirements.txt"
        uv pip install --require-hashes -r "${repo}/python/build_requirements_usd.txt"
    else
        python -m pip install --upgrade --require-hashes -r "${repo}/python/build_requirements.txt"
        python -m pip install --upgrade --require-hashes -r "${repo}/python/build_requirements_usd.txt"
    fi
    popd > /dev/null
}


npm_ci() {
    echo "Installing NPM dependencies for WASM bindings..."
    pushd wasm
    npm ci
    popd
}


setup_emsdk() {
    echo "Setting up Emscripten..."
    git clone https://github.com/emscripten-core/emsdk.git
    ./emsdk/emsdk install 4.0.10
    ./emsdk/emsdk activate 4.0.10
    # Force installing emscripten's typescript dependencies. This is a
    # workaround for the github update to a newer typescript, which gives an
    # error on the deprecated `--outFile` flag.
    pushd emsdk/upstream/emscripten
    npm i
    popd
}


configure_mujoco() {
    echo "Configuring MuJoCo..."
    # Disable IPO/LTO to cut build time. Skip this on Windows: turning off MSVC's
    # whole-program optimization (/GL) exposes a latent heap corruption in
    # SetConstTest.SleepingNotAllowed (a real bug worth a separate investigation),
    # and Windows build time is not a CI bottleneck.
    local ipo_off="-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF"
    if [[ "${RUNNER_OS}" == "Windows" ]]; then
        ipo_off=""
    fi
    mkdir build &&
    cd build &&
    cmake .. \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        ${ipo_off} \
        -DCMAKE_INSTALL_PREFIX:STRING=${TMPDIR}/mujoco_install \
        -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
        ${CCACHE_ARGS} \
        ${CMAKE_ARGS}
}


build_mujoco() {
    echo "Building MuJoCo..."
    cmake --build . --config=Release ${CMAKE_BUILD_ARGS}
}


test_mujoco() {
    echo "Testing MuJoCo..."
    # ctest defaults to serial. The suite is ~1650 independent tests that use
    # unique temp files (mkstemp / testing::TempDir) and declare no RUN_SERIAL /
    # RESOURCE_LOCK, so running them in parallel is safe and ~2x faster on POSIX.
    # Windows is kept serial conservatively: parallel-safety on the Windows file
    # system is unverified and its test time is not a CI bottleneck.
    if [[ "${RUNNER_OS}" == "Windows" ]]; then
        ctest -C Release --output-on-failure .
    else
        local ncpu
        ncpu="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo "${NUMBER_OF_PROCESSORS:-2}")"
        ctest -C Release --output-on-failure --parallel "${ncpu}" .
    fi
}


install_mujoco() {
    echo "Installing MuJoCo..."
    cmake --install .
}


copy_plugins_posix() {
    echo "Copying plugins..."
    mkdir -p ${TMPDIR}/mujoco_install/mujoco_plugin &&
    cp lib/libactuator.* ${TMPDIR}/mujoco_install/mujoco_plugin &&
    cp lib/libelasticity.* ${TMPDIR}/mujoco_install/mujoco_plugin &&
    cp lib/libsensor.* ${TMPDIR}/mujoco_install/mujoco_plugin &&
    cp lib/libsdf_plugin.* ${TMPDIR}/mujoco_install/mujoco_plugin
}


copy_plugins_window() {
    echo "Copying plugins..."
    mkdir -p ${TMPDIR}/mujoco_install/mujoco_plugin &&
    cp bin/Release/actuator.dll ${TMPDIR}/mujoco_install/mujoco_plugin &&
    cp bin/Release/elasticity.dll ${TMPDIR}/mujoco_install/mujoco_plugin &&
    cp bin/Release/sensor.dll ${TMPDIR}/mujoco_install/mujoco_plugin
}


configure_samples() {
    echo "Configuring samples..."
    # Samples are tiny, so they keep the default IPO/LTO: disabling it saves no
    # meaningful build time and would expose the same gcc -Werror false positives
    # that -O3-without-LTO triggers (see configure_mujoco).
    mkdir build &&
    cd build &&
    cmake .. \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        -Dmujoco_ROOT:STRING=${TMPDIR}/mujoco_install \
        ${CCACHE_ARGS} \
        ${CMAKE_ARGS}
}


configure_simulate() {
    echo "Configuring simulate..."
    # See configure_samples: keep the default IPO/LTO for this small build.
    mkdir build &&
    cd build &&
    cmake .. \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        -Dmujoco_ROOT:STRING=${TMPDIR}/mujoco_install \
        ${CCACHE_ARGS} \
        ${CMAKE_ARGS}
}


build_simulate() {
    echo "Building simulate..."
    cmake --build . --config=Release ${CMAKE_BUILD_ARGS}
}


configure_studio() {
    echo "Configuring Studio..."
    cmake -B build \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
        -DUSE_STATIC_LIBCXX=OFF \
        -DBUILD_SHARED_LIBS=OFF \
        -DMUJOCO_BUILD_EXAMPLES=OFF \
        -DMUJOCO_BUILD_SIMULATE=OFF \
        -DMUJOCO_BUILD_STUDIO=ON \
        -DMUJOCO_BUILD_TESTS=OFF \
        -DMUJOCO_TEST_PYTHON_UTIL=OFF \
        -DMUJOCO_WITH_USD=OFF \
        -DMUJOCO_USE_FILAMENT=ON \
        ${CCACHE_ARGS} \
        ${CMAKE_ARGS}
    echo "Configuring Studio... DONE"
}


build_studio() {
    echo "Building Studio..."
    cmake --build build --config=Release --target mujoco_studio --parallel
    echo "Building Studio... DONE"
}


make_python_sdist() {
    echo "Making Python sdist..."
    source ${TMPDIR}/venv/bin/activate &&
    ./make_sdist.sh
}


build_python_bindings() {
    echo "Building Python bindings..."
    source ${TMPDIR}/venv/bin/activate
    # pip unpacks the sdist into a randomized temp dir every run, so the absolute
    # source/include paths differ each time and defeat ccache (0% hit, full
    # recompile). CCACHE_BASEDIR rewrites absolute paths under it to paths relative
    # to the (also-in-temp) build cwd, cancelling the random component so objects
    # hash identically across runs. CCACHE_SLOPPINESS ignores timestamp/path noise.
    #
    # Do NOT add system_headers here: CMake adds the imported mujoco target's
    # include dir (MUJOCO_PATH/include) as -isystem, so ccache would treat the
    # public MuJoCo headers as system headers and skip hashing them. A change that
    # lives only in those headers (a new mjData field, a new enum value) would then
    # go undetected and ccache would reuse an object compiled against the old struct
    # layout, producing an ABI-mismatched binding (wrong field offsets, stale
    # mjNENABLE, signature mismatch). The mtime/ctime flags are kept: they handle the
    # temp-dir churn without affecting header content detection.
    export CCACHE_BASEDIR="${TMPDIR}"
    export CCACHE_SLOPPINESS="time_macros,include_file_mtime,include_file_ctime,pch_defines,locale"
    MUJOCO_PATH="${TMPDIR}/mujoco_install" \
    MUJOCO_PLUGIN_PATH="${TMPDIR}/mujoco_install/mujoco_plugin" \
    MUJOCO_CMAKE_ARGS="-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF ${CCACHE_ARGS} ${CMAKE_ARGS}" \
    pip wheel -v --no-deps mujoco-*.tar.gz
}


install_python_bindings() {
    echo "Installing Python bindings..."
    source ${TMPDIR}/venv/bin/activate &&
    pip install --no-index mujoco-*.whl
}


test_python_bindings() {
    echo "Testing Python bindings..."
    source ${TMPDIR}/venv/bin/activate &&
    pytest -v --pyargs mujoco
}


build_test_wasm() {
    echo "Building and testing WASM bindings..."
    source emsdk/emsdk_env.sh
    export PATH="$(pwd)/node_modules/.bin:$PATH"
    echo "Build MuJoCo with Emscripten (Multi-Threaded)..."
    emcmake cmake -B build_wasm_mt \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
        -DMUJOCO_WASM_THREADS=ON \
        ${CCACHE_ARGS} \
        $WASM_CMAKE_ARGS
    cmake --build build_wasm_mt --parallel $(nproc)

    echo "Run bindings tests for Multi-Threaded version..."
    npm run test --prefix ./wasm

    echo "Moving Multi-Thread version under mt subfolder..."
    mkdir -p wasm/dist/mt
    mv wasm/dist/mujoco.* wasm/dist/mt/

    echo "Build MuJoCo with Emscripten (Single-Threaded)..."
    emcmake cmake -B build_wasm_st \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
        -DMUJOCO_WASM_THREADS=OFF \
        ${CCACHE_ARGS} \
        $WASM_CMAKE_ARGS
    cmake --build build_wasm_st --parallel $(nproc)

    echo "Run bindings tests for Single-Threaded version..."
    npm run test --prefix ./wasm
}

package_wasm() {
    echo "Publishing WASM bindings..."
    cp wasm/package.npm.json wasm/dist/package.json
    cp wasm/README.md wasm/dist/README.md
    VERSION="${VERSION:-${GITHUB_REF#refs/tags/}}"
    npm --prefix wasm/dist version "${VERSION}" --no-git-tag-version
    npm pack --dry-run ./wasm/dist
    npm publish ./wasm/dist --access public --provenance
}


package_mjx() {
    echo "Packaging MJX..."
    source ${TMPDIR}/venv/bin/activate &&
    python -m build .
}


install_mjx() {
    echo "Installing MJX..."
    source ${TMPDIR}/venv/bin/activate
    # The MJX requirements (jax, jaxlib, scipy, ...) are a big install; use uv when
    # available. Keep pip for the local --no-index wheel.
    if command -v uv > /dev/null 2>&1; then
        uv pip install --require-hashes -r requirements.txt
    else
        pip install --require-hashes -r requirements.txt
    fi
    pip install --no-index dist/mujoco_mjx-*.whl
}


test_mjx() {
    echo "Testing MJX..."
    source ${TMPDIR}/venv/bin/activate &&
    pytest -n auto -v -k 'not IntegrationTest' --pyargs mujoco.mjx
}


notify_team_chat() {
    CHATMSG="$(cat <<-'EOF' | python3
import json
import os
env = lambda x: os.getenv(x, '')
data = dict(
    result=env('JOB_URL'),
    job=env('CHATMSG_JOB_ID'),
    commit=env('GITHUB_SHA')[:6],
    name=env('CHATMSG_AUTHOR_NAME').replace('```', ''),
    email=env('CHATMSG_AUTHOR_EMAIL'),
    msg=env('CHATMSG_COMMIT_MESSAGE').replace('```', '')
)
text = '<{result}|*FAILURE*>: job `{job}` commit `{commit}`\n```Author: {name} <{email}>\n\n{msg}```'.format(**data)
print(json.dumps({'text' : text}))
EOF
)" &&

    curl "$GCHAT_API_URL&threadKey=$GITHUB_SHA&messageReplyOption=REPLY_MESSAGE_FALLBACK_TO_NEW_THREAD" \
    -X POST \
    -H "Content-Type: application/json" \
    --data-raw "${CHATMSG}"
}


build_mujoco_live() {
    echo "Setting up Emscripten SDK..."
    source emsdk/emsdk_env.sh

    echo "Building Filament tools, targeting host platform..."
    cmake -S . -B build_host -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DUSE_STATIC_LIBCXX=OFF \
        -DMUJOCO_BUILD_STUDIO=ON \
        -DMUJOCO_USE_FILAMENT=ON \
        -DMUJOCO_BUILD_TESTS=OFF \
        -DMUJOCO_BUILD_EXAMPLES=OFF \
        -DMUJOCO_BUILD_SIMULATE=OFF
    cmake --build build_host --target matc resgen cmgen mujoco_filament_assets -j$(nproc)

    echo "Building WASM app..."
    emcmake cmake -S . -B build_wasm -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DMUJOCO_BUILD_STUDIO=ON \
        -DMUJOCO_USE_FILAMENT=ON \
        -DMUJOCO_BUILD_TESTS_WASM=OFF \
        -DMUJOCO_NATIVE_BUILD_DIR=$(pwd)/build_host
    cmake --build build_wasm --target mujoco_studio -j$(nproc)
}


# Discover functions defined in this script by finding identifiers followed by
# "()" and capturing the identifier as a valid function name.
VALID_FUNCTIONS=()
while IFS= read -r func_name; do
  VALID_FUNCTIONS+=("$func_name")
done < <(grep -E '^[[:alnum:]_]+\(\)' "$0" | sed 's/().*$//')

# Exit with an error if the requested function is not found.
if [[ ! " ${VALID_FUNCTIONS[*]} " =~ " ${1} " ]]; then
    echo "Usage: $0 {$(IFS='|'; echo "${VALID_FUNCTIONS[*]}")}, got '$1'"
    exit 1
fi

# Set options to print the commands being run, and cause the script to exit with
# an error code if any command fails. Note we do this just before executing
# the requested function to avoid cluttering the output with the above command
# discovery code.
set -xe

# Execute the requested function.
"$1"
