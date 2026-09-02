#!/usr/bin/env bash
set -e

cd "$HOME/mujoco"

mkdir -p build/bin/mujoco_plugin

if [ -f build/lib/libusd_decoder_plugin.so ]; then
  cp -f build/lib/libusd_decoder_plugin.so build/bin/mujoco_plugin/
fi

export LD_LIBRARY_PATH="$HOME/mujoco/build/lib:$HOME/mujoco/build/_deps/openusd-build/lib:$LD_LIBRARY_PATH"
export PXR_PLUGINPATH_NAME="$HOME/mujoco/build/lib/mujoco-usd-resources:$PXR_PLUGINPATH_NAME"

LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe ./build/bin/simulate "$@"
