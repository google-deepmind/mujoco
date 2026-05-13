# MJX Warp Codegen

Generates the MJX-Warp shim layer in `mujoco/mjx/warp/` by reading the vendored
`mujoco_warp` source in `mujoco/mjx/third_party/mujoco_warp/`.

## Setup

From the root `mjx/` directory, once you install [`uv`](https://docs.astral.sh/uv/getting-started/installation/), install the latest MuJoCo and local MJX:

```bash
uv venv .venv --default-index https://pypi.org/simple
source .venv/bin/activate
uv pip install --upgrade --force-reinstall mujoco --default-index https://pypi.org/simple --extra-index-url https://py.mujoco.org/
uv pip install -e ".[warp,dev]" --default-index https://pypi.org/simple
```

## Run codegen

From the root `mjx/` directory:

```bash
bash mujoco/mjx/codegen/update_for_mujoco_warp.sh
```
