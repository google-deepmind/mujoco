# MuJoCo + OpenUSD Setup Notes

This documents the current OpenUSD-enabled MuJoCo setup used for USD asset/environment import testing.

## Current Purpose

This work is focused on the OpenUSD -> MuJoCo import path.

It is not policy training yet. The goal is to make MuJoCo able to load and visualize USD-based assets/environments so they can later be tested with Seahorse, RoboJudo, and ProtoMotion workflows.

## Build Flow

1. Clone stable MuJoCo.
2. Build OpenUSD using MuJoCo's helper:

   `cmake/third_party_deps/openusd`

3. Configure MuJoCo with OpenUSD enabled:

   `-DMUJOCO_WITH_USD=True`

4. Build MuJoCo.
5. Verify that `build/bin/simulate` exists.
6. Launch MuJoCo using the runtime helper script:

   `scripts/openusd/run_mujoco_usd.sh`

## Runtime Helper

The runtime helper script sets the paths needed for MuJoCo to find the USD decoder plugin and OpenUSD libraries.

It handles:

- `libusd_decoder_plugin.so`
- `LD_LIBRARY_PATH`
- `PXR_PLUGINPATH_NAME`
- WSL graphics settings using `llvmpipe`

## XML Wrapper Testing

USD assets are loaded through MJCF XML wrapper files.

The important MJCF line is:

    <model name="asset_name" file="/path/to/file.usd" content_type="text/usd"/>

The wrapper gives MuJoCo simulation context such as:

- asset path
- worldbody
- lighting
- floor
- attach point

## Current Limitation

This does not automatically convert a full Isaac Sim task into a MuJoCo training environment.

Physics, contacts, joints, actuators, rewards, and policy training still need separate validation.
