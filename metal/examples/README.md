# Side-by-side Mac pendulum demo

This adapts the four-hinge chaotic pendulum from MuJoCo's
[Python tutorial](../../python/tutorial.ipynb), Copyright 2021 DeepMind
Technologies Limited, Apache-2.0. It has no contacts, actuators or passive forces.

**Left:** experimental Metal mass matrix and gravity/inertial bias, followed by
CPU NumPy acceleration solve and CPU semi-implicit Euler integration.
**Right:** independent standard CPU `mj_step`, with the same initial state.
Both are displayed by MuJoCo's OpenGL renderer. This is a **hybrid demo**, not
full native Metal stepping, contact qualification or a speed benchmark.
No CPU fallback is allowed for the Metal computations.

From the repository root, in an isolated Python 3.12 environment:

```sh
python3.12 -m venv .venv-demo
.venv-demo/bin/python -m pip install './metal[metal,test]'
PYTHONPATH=metal .venv-demo/bin/mjpython metal/examples/pendulum.py
```

Use `mjpython` for the interactive viewer on macOS. Press **R** to reset both
pendulums; close the window to stop. Avoid dragging or applying forces: these
viewer interactions are outside this fixed-model comparison. The right-hand
pendulum is translated only for display. Each frame advances ten 1 ms steps;
slow machines display slower-than-real-time motion without skipping steps.
The simulation continues until closed; reset periodically when comparing:
chaotic trajectories eventually diverge due to floating-point differences.

Run a short numerical check without opening a window:

```sh
PYTHONPATH=metal .venv-demo/bin/python metal/examples/pendulum.py --headless --check
MUJOCO_METAL_RUN_GPU=1 PYTHONPATH=metal .venv-demo/bin/python -m pytest -q metal/tests/test_pendulum.py
```

`--check` requires at most 200 steps and checks maximum absolute joint-position
error below 0.001 rad and velocity error below 0.01 rad/s throughout the rollout.
`--mode cpu` provides a viewer/control baseline with CPU physics on both sides.
`--viewer-seconds 10` closes the viewer automatically for a smoke test.

Optional PNG export still needs an OpenGL context (headless means no interactive
viewer, not software rendering):

```sh
.venv-demo/bin/python -m pip install pillow
PYTHONPATH=metal .venv-demo/bin/python metal/examples/pendulum.py --headless --check --image pendulum.png
```

On the local Apple M1 validation machine, the initial 200-step hybrid rollout
had maximum errors of approximately 2.2e-8 rad and 5.8e-7 rad/s. The GPU test
also reruns after reset. These results apply to this bundled model and pinned
dependencies, not arbitrary XML models or full MuJoCo feature coverage.

Some `uv` Python installations need their base interpreter's library directory
for `mjpython`. If startup reports `libpython3.12.dylib` missing, launch with:

```sh
DEMO_PYTHON_LIB="$(.venv-demo/bin/python -c 'import sys; print(sys.base_prefix + "/lib")')"
DYLD_FALLBACK_LIBRARY_PATH="$DEMO_PYTHON_LIB${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}" PYTHONPATH=metal .venv-demo/bin/mjpython metal/examples/pendulum.py
```
