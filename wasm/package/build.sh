#!/bin/sh
# This script is designed to be run from the 'wasm/package' directory.

# Exit immediately if any command fails
set -e

echo "--- Starting MuJoCo WASM Build & Prep ---"

# 1. Get the directory of this script (wasm/package)
#    and 'cd' one level up to the 'wasm' directory.
cd "$(dirname "$0")/.."

echo "Changed working directory to: $(pwd)"

# 2. Now that we are in 'wasm/', run your exact build commands.
#    All paths (.., ../build, ., build) are now correct.
echo "Setting path and running cmake..."
export PATH="./node_modules/.bin:$PATH"
emcmake cmake -S .. -B ../build

echo "Building main project..."
cmake --build ../build

echo "Running cmake for wasm bindings..."
emcmake cmake -S . -B build

echo "Building wasm bindings..."
cmake --build build

# 3. Now that 'wasm/dist/' is created, copy the artifacts
#    into the 'wasm/package/' directory so npm can find them.
echo "Copying build artifacts from dist/ to package/..."
cp dist/* package/

echo "--- Build and Copy Complete ---"
