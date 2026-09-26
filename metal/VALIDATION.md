# Port validation — September 26, 2026

Source: Microduck RL Mac `60f0b26a55e3236d2112126386b25ac922228083`.
Host: M1 Max, 32 GB unified memory, macOS 26.6.2, Python 3.12.10.
Isolated environment: MuJoCo 3.10.0, Torch 2.9.1, NumPy 2.5.3.

| Check | Result |
| --- | --- |
| CPU tests against source install | 97 passed; 165 GPU cases skipped |
| CPU tests against installed wheel from outside checkout | 97 passed; 165 GPU cases skipped |
| CPU-only preflight | Loads the installed model and shader without importing Torch |
| Wheel resources | 38 STL assets, one XML model, one Metal shader; about 7.8 MB wheel |
| Source distribution | Includes all 25 NPZ reference fixtures and tests |
| Python style | isort and pyink checks pass |
| Shader preservation | Identical after removing trailing whitespace |
| Copied Python implementation | All 19 top-level class/function syntax trees match after excluding docstrings; imports relocated |
| Actual GPU execution of this port | **Not run** |
| Controlled performance comparison | **Not run** |

Shader SHA256:
`c085ac0f07d55067aaf1f5915a96ef7213eb6f7138478db9c98254e44b0ba7e6`.

The CPU suite exercises reference constraint/contact equations and CPU dynamics,
plus public-API validation, CPU-backed reset/restore logic and qualification
failure detection. CPU tests prevent shader compilation and MPS synchronization.
CPU tensor stand-ins test ownership and transactional validation only; they are
not evidence of MPS execution or physics equivalence.

GPU qualification was deferred because a production training run was active.
The production checkout, interpreter, processes and training artifacts were not
changed. Historical downstream results do not replace testing this port, its new
public wrapper or the foot-ground-only profile. The public API changes collision
masks and solver options explicitly; the original corpus retains its original
canonical model and exercises the unchanged low-level pipeline.

Still required: run both commands below on an idle Apple Silicon machine from
repository root after the isolated install in the README:

```bash
.venv-metal/bin/python -m pytest -q metal/tests --run-metal
.venv-metal/bin/python -m mujoco_metal smoke --batch-size 4 --steps 10
```

Record numerical failures, nonconverged solver statuses, continuation residuals
and device details. A successful smoke does not establish long-horizon accuracy,
a useful walking policy, arbitrary-model support or a performance improvement.
Only the standard double-precision MuJoCo 3.10.0 wheel was used as CPU reference
here. Single-precision reference builds, other OS/chip combinations, newer MuJoCo
versions and the full upstream test suite have not been qualified for this port.
