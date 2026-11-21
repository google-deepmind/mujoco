# Full WSL Build Process: MuJoCo + OpenUSD

This document records the full working process used to build MuJoCo with OpenUSD support inside WSL Ubuntu.

The goal of this setup is to support USD asset/environment import testing in MuJoCo. This is not policy training yet. The current focus is:

- Build MuJoCo with OpenUSD enabled
- Register the USD decoder plugin at runtime
- Load USD assets through MJCF XML wrappers
- Prepare for Seahorse/G1 asset testing
- Later integrate with RoboJudo and ProtoMotion workflows

---

## 1. Windows WSL / Ubuntu Setup

This part is done from Windows PowerShell.

Open PowerShell as Administrator when enabling WSL for the first time.

```powershell
wsl --install -d Ubuntu
```

After installation, restart the computer if Windows says changes will not take effect until reboot.

Check installed distributions:

```powershell
wsl --list --verbose
```

Open Ubuntu:

```powershell
wsl
```

Inside Ubuntu, confirm the Linux home directory:

```bash
cd ~
pwd
```

Expected output should look like:

```text
/home/your_linux_username
```

---

## 2. Ubuntu Dependency Install

Update Ubuntu packages:

```bash
sudo apt update
```

Install build tools, CMake, Python tools, and graphics/windowing dependencies:

```bash
sudo apt install -y git cmake build-essential python3 python3-pip curl \
pkg-config libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev \
libxrandr-dev libxxf86vm-dev libgl1-mesa-dev libx11-dev \
libwayland-dev wayland-protocols libxkbcommon-dev mesa-utils \
libgl1-mesa-dri libegl1 libglvnd0 mesa-vulkan-drivers
```

These packages cover:

- `build-essential`: GCC/G++ and Make
- `cmake`: configures MuJoCo/OpenUSD builds
- `python3`, `python3-pip`: build scripts/tools
- `pkg-config`: helps CMake locate libraries
- GLFW/X11/Wayland/Mesa packages: needed for MuJoCo `simulate` viewer in WSL/Ubuntu

---

## 3. Git Clone Stability Fixes

These settings were used to reduce GitHub/OpenUSD clone failures such as:

- `RPC failed`
- `early EOF`
- `invalid index-pack`

Run:

```bash
git config --global http.version HTTP/1.1
git config --global http.postBuffer 524288000
git config --global core.compression 0
```

Meaning:

- `http.version HTTP/1.1`: forces Git/libcurl to use HTTP/1.1
- `http.postBuffer 524288000`: increases Git smart HTTP buffer size
- `core.compression 0`: disables Git compression

These are stability workarounds. They do not change MuJoCo source code.

---

## 4. Clone Stable MuJoCo

Clone MuJoCo from the official DeepMind repository.

```bash
cd ~

git clone --branch 3.8.1 https://github.com/google-deepmind/mujoco.git mujoco
```

Enter the repo:

```bash
cd ~/mujoco
```

Confirm checkout:

```bash
git log -1 --oneline
```

Expected commit/tag seen during setup:

```text
68820957 (HEAD, tag: 3.8.1) Update changelog for the 3.8.1 release.
```

---

## 5. Build OpenUSD Helper

MuJoCo includes an OpenUSD helper project at:

```text
cmake/third_party_deps/openusd
```

This helper is not the runtime plugin. It is a build-time CMake helper that downloads and builds OpenUSD as a dependency.

Configure the OpenUSD helper:

```bash
cd ~/mujoco

cmake -Bcmake/third_party_deps/openusd/build cmake/third_party_deps/openusd
```

Build OpenUSD:

```bash
cmake --build cmake/third_party_deps/openusd/build -j$(nproc)
```

On lower-RAM computers, reduce parallel jobs:

```bash
cmake --build cmake/third_party_deps/openusd/build -j2
```

or:

```bash
cmake --build cmake/third_party_deps/openusd/build -j1
```

Success checkpoint:

```text
Success! USD libraries were built.
[100%] Completed 'openusd'
[100%] Built target openusd
```

The helper downloads OpenUSD, pulls a stable release, builds shared USD libraries, keeps optional features off, and uses TBB as a required dependency.

---

## 6. Configure MuJoCo with OpenUSD Enabled

After OpenUSD is built, MuJoCo still needs to be configured to use it.

Main flag:

```text
-DMUJOCO_WITH_USD=True
```

Configure MuJoCo:

```bash
cd ~/mujoco

cmake -Bbuild -S. -DMUJOCO_WITH_USD=True \
  -DCMAKE_C_FLAGS="-Wno-error=discarded-qualifiers -Wno-error" \
  -DCMAKE_CXX_FLAGS="-Wno-error"
```

Why the warning flags were used:

- GCC 15 warnings were stopping the build as errors
- `-Wno-error` prevents warnings from terminating configuration/build
- `-Wno-error=discarded-qualifiers` specifically handled a C warning seen during this setup

Success checkpoint:

```text
Configuring done
Generating done
Build files have been written to: /home/your_user/mujoco/build
```

During this stage, CMake should find required viewer dependencies such as:

- Wayland
- X11
- GLFW
- `pkg-config`

---

## 7. Build MuJoCo

Build MuJoCo:

```bash
cd ~/mujoco

cmake --build build -j$(nproc)
```

On lower-RAM computers, reduce jobs:

```bash
cmake --build build -j2
```

or:

```bash
cmake --build build -j1
```

Success checkpoint examples:

```text
[100%] Built target mjcf_file_format_test
[100%] Built target xml_native_reader_test
[100%] Built target xml_native_writer_test
```

---

## 8. Verify `simulate` and USD Artifacts

Confirm the MuJoCo viewer executable exists:

```bash
cd ~/mujoco

find build -type f -executable -name simulate
```

Expected:

```text
build/bin/simulate
```

Check for USD-related build artifacts:

```bash
find build -type f | grep -Ei "usd|mjcPhysics|plugInfo|decoder" | head -50
```

Useful items may include:

- USD decoder build files
- `mjcPhysics` files
- USD/MJCF plugin build outputs
- USD test/build artifacts

This confirms the build included OpenUSD-related components rather than a normal MuJoCo-only build.

---

## 9. Runtime Helper Script

The build creates the components, but MuJoCo still needs correct runtime paths when launching `simulate`.

Create the runtime helper script:

```bash
cat > ~/run_mujoco_usd.sh <<'EOF_SCRIPT'
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
EOF_SCRIPT

chmod +x ~/run_mujoco_usd.sh
```

The script does four important things:

1. Copies `libusd_decoder_plugin.so` into the MuJoCo plugin folder
2. Sets `LD_LIBRARY_PATH` so Linux can find USD shared libraries
3. Sets `PXR_PLUGINPATH_NAME` so OpenUSD can find MuJoCo USD plugin/schema resources
4. Forces WSL software rendering with `llvmpipe`

Run MuJoCo:

```bash
~/run_mujoco_usd.sh
```

Runtime proof:

```text
MuJoCo version 3.8.1
Plugins registered by library 'libusd_decoder_plugin.so':
```

---

## 10. WSL Graphics Restart Fix

If `simulate` opens but the GUI does not appear, or WSL graphics is stuck, restart WSLg from PowerShell.

Exit Ubuntu first:

```bash
exit
```

Then in PowerShell:

```powershell
wsl --shutdown
wsl --update
wsl
```

Back inside Ubuntu, test graphics:

```bash
xeyes
```

If `xeyes` opens, try MuJoCo again:

```bash
~/run_mujoco_usd.sh
```

---

## 11. XML Wrapper Testing

USD assets can be tested through MJCF XML wrapper files.

The wrapper gives MuJoCo simulation context:

- asset path
- worldbody
- floor
- lighting
- attach point

Example wrapper:

```xml
<mujoco model="load_usd_asset">
  <asset>
    <model name="usd_asset" file="/path/to/file.usd" content_type="text/usd"/>
  </asset>

  <worldbody>
    <light pos="0 0 5"/>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="usd_holder" pos="0 0 0">
      <attach model="usd_asset" prefix="usd_"/>
    </body>
  </worldbody>
</mujoco>
```

Run a wrapper:

```bash
~/run_mujoco_usd.sh ~/mujoco/path/to/wrapper.xml
```

Important:

The USD file path inside the wrapper must actually exist. If it does not, OpenUSD may fail to open the layer and `simulate` can terminate.

Check a USD file path before running:

```bash
ls "/path/to/file.usd"
```

Find USD files:

```bash
find ~/usd_tests -type f \( -iname "*.usd" -o -iname "*.usda" -o -iname "*.usdc" -o -iname "*.usdz" \) | head -50
```

Example path used during testing:

```text
/mnt/c/Users/David/Hard_Documents/mujoco/usd_tests/Industrial_NVD_10012
```

---

## 12. Known Issues and Fixes

### Issue: PowerShell commands mixed with Ubuntu commands

Problem:

```text
cat > file.xml <<EOF
```

fails in PowerShell because heredoc syntax is Linux shell syntax.

Fix:

Run Linux commands inside WSL/Ubuntu, not PowerShell.

PowerShell:

```powershell
wsl
```

Then Ubuntu:

```bash
cd ~/mujoco
```

---

### Issue: `~/run_mujoco_usd.sh` not recognized in PowerShell

Problem:

```powershell
~/run_mujoco_usd.sh
```

PowerShell does not understand Linux home/script paths.

Fix:

Run from WSL:

```bash
~/run_mujoco_usd.sh
```

Or from PowerShell through WSL:

```powershell
wsl -- bash -lc "~/run_mujoco_usd.sh"
```

---

### Issue: Missing USD file causes OpenUSD crash

Problem:

```text
Failed to open layer @/path/to/file.usd@
```

Fix:

Verify the file exists:

```bash
ls "/path/to/file.usd"
```

Find the asset pack:

```bash
find "/mnt/c/Users/David" -iname "*Industrial*" 2>/dev/null | head -50
```

---

### Issue: OpenUSD build fails or machine runs out of RAM

Problem:

OpenUSD build exits or gets killed during long build.

Fix:

Use fewer build jobs:

```bash
cmake --build cmake/third_party_deps/openusd/build -j2
```

or:

```bash
cmake --build cmake/third_party_deps/openusd/build -j1
```

---

### Issue: MuJoCo GUI does not appear in WSL

Fix from PowerShell:

```powershell
wsl --shutdown
wsl --update
wsl
```

Then run:

```bash
~/run_mujoco_usd.sh
```

---

## Current Scope

This setup supports:

- OpenUSD-enabled MuJoCo build
- Runtime USD decoder plugin registration
- USD asset import testing
- MJCF XML wrapper testing
- Seahorse/G1 asset visualization tests

This setup does not yet provide:

- ProtoMotion training
- RoboJudo integration
- Isaac Lab reward/task conversion
- Real robot deployment
- Full physics equivalence between USD and MJCF

Those are future integration steps.
