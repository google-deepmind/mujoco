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
    python -m pip install --upgrade --require-hashes -r "${repo}/python/build_requirements.txt"
    python -m pip install --upgrade --require-hashes -r "${repo}/python/build_requirements_usd.txt"
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
}


configure_mujoco() {
    echo "Configuring MuJoCo..."
    mkdir build &&
    cd build &&
    cmake .. \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
        -DCMAKE_INSTALL_PREFIX:STRING=${TMPDIR}/mujoco_install \
        -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
        ${CMAKE_ARGS}
}


build_mujoco() {
    echo "Building MuJoCo..."
    cmake --build . --config=Release ${CMAKE_BUILD_ARGS}
}


test_mujoco() {
    echo "Testing MuJoCo..."
    ctest -C Release --output-on-failure .
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
    mkdir build &&
    cd build &&
    cmake .. \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
        -Dmujoco_ROOT:STRING=${TMPDIR}/mujoco_install \
        ${CMAKE_ARGS}
}


configure_simulate() {
    echo "Configuring simulate..."
    mkdir build &&
    cd build &&
    cmake .. \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
        -Dmujoco_ROOT:STRING=${TMPDIR}/mujoco_install \
        ${CMAKE_ARGS}
}


build_simulate() {
    echo "Building simulate..."
    cmake --build . --config=Release ${CMAKE_BUILD_ARGS}
}


_configure_studio() {
    # Invoke cmake will all options OFF assuming that the caller will enable
    # needed options by running `export _CONFIGURE_STUDIO_CMAKE_ARGS=...` first
    cmake -B build \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
        -DUSE_STATIC_LIBCXX=OFF \
        -DBUILD_SHARED_LIBS=OFF \
        -DMUJOCO_BUILD_EXAMPLES=OFF \
        -DMUJOCO_BUILD_SIMULATE=OFF \
        -DMUJOCO_BUILD_STUDIO=OFF \
        -DMUJOCO_BUILD_TESTS=OFF \
        -DMUJOCO_TEST_PYTHON_UTIL=OFF \
        -DMUJOCO_WITH_USD=OFF \
        -DMUJOCO_USE_FILAMENT=OFF \
        -DMUJOCO_USE_FILAMENT_VULKAN=OFF \
        ${_CONFIGURE_STUDIO_CMAKE_ARGS}
}


configure_studio_legacy_opengl() {
    echo "Configuring Studio (legacy OpenGL)..."
    export _CONFIGURE_STUDIO_CMAKE_ARGS="-DMUJOCO_BUILD_STUDIO=ON ${CMAKE_ARGS}"
    _configure_studio
    echo "Configuring Studio (legacy OpenGL)... DONE"
}


configure_studio() {
    echo "Configuring Studio..."
    export _CONFIGURE_STUDIO_CMAKE_ARGS="-DMUJOCO_BUILD_STUDIO=ON -DMUJOCO_USE_FILAMENT=ON ${CMAKE_ARGS}"
    _configure_studio
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
    source ${TMPDIR}/venv/bin/activate &&
    MUJOCO_PATH="${TMPDIR}/mujoco_install" \
    MUJOCO_PLUGIN_PATH="${TMPDIR}/mujoco_install/mujoco_plugin" \
    MUJOCO_CMAKE_ARGS="-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF ${CMAKE_ARGS}" \
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

    emcmake cmake -B build_wasm -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF $WASM_CMAKE_ARGS
    cmake --build build_wasm

    npm run test --prefix ./wasm
}


package_mjx() {
    echo "Packaging MJX..."
    source ${TMPDIR}/venv/bin/activate &&
    python -m build .
}


install_mjx() {
    echo "Installing MJX..."
    source ${TMPDIR}/venv/bin/activate &&
    pip install --require-hashes -r requirements.txt &&
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
