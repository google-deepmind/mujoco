# Experimental MuJoCo Metal package

This optional package targets Python 3.12 and MuJoCo **3.10.0**. The surrounding MuJoCo source checkout is 3.14.1; that version is not a target. Use an isolated environment so the pinned package does not alter the surrounding checkout:

```sh
cd metal
python3.12 -m venv .venv-metal
source .venv-metal/bin/activate
python -m pip install -e '.[metal,test]'
```

For CPU-only utilities, replace the install extra with `.[test]`. Importing `mujoco_metal` and running preflight do not import Torch or initialize MPS.

Run `python -m mujoco_metal preflight --model path/to/model.xml --json --inventory` to inspect runtime version, model dimensions, package and shader paths/hash, capability boundaries, and the versioned feature/API inventory. Inventory completeness is explicitly false because the captured enum and Python binding inventory is not exhaustive.

`load_model(xml_or_path)` returns an immutable, dimension-derived descriptor. `descriptor.forward_kinematics(qpos)` is a CPU reference for body, inertial, geom, site, and joint-anchor/axis world poses across hinge, slide, ball, and free joints. `MetalKinematics(descriptor).run(qpos_batch)` computes the same pose fields through the batched native Metal kinematics kernel. GPU checks have passed on an Apple M1 for empty and fixed worlds, mixed hinge/slide/ball/free models with off-center joints and multiple free roots, world-attached sites/geoms, and a 32-DOF chain. This is a narrow correctness qualification, not a general model-support or performance claim. Constructing `MetalKinematics` initializes MPS and compiles the bundled shader.

`smooth_dynamics(descriptor, qpos, qvel)` is the CPU reference returning a dense joint-space mass matrix and inertial/gravity bias forces. `MetalSmoothDynamics(descriptor).run(qpos_batch, qvel_batch)` returns those two outputs as MPS tensors. Native GPU checks have passed on an Apple M1 across empty/fixed, mixed-joint, rotated-inertia, massless-ancestor, disabled-gravity, and 32-DOF-chain fixtures. This is a narrow correctness qualification for the smooth M and bias stages, not a general model-support or performance claim. Each batch shares one immutable model descriptor; per-environment native mass randomization is not connected to the CPU lifecycle utilities. Inputs are host NumPy arrays and outputs remain MPS tensors. Models with actuators or nonzero tendon armature are rejected. Actuator forces/armature, tendon armature, passive forces, contacts, constraints, sensors, integration, and stepping are outside this stage.

`ModelLifecycle` performs transactional CPU body-mass updates through MuJoCo `mj_setConst`. `BatchedConstants` maintains per-environment body masses and derived `body_invweight0` rows with atomic recomputation/restore and seeded mass randomization. `KinematicsBatchState` tracks explicit environment rows with generation-based FK cache invalidation, snapshots, restore, and tangent-space joint randomization. These are CPU lifecycle utilities and do not advance physics.

Run the opt-in GPU correctness tests only on an available Apple GPU with the pinned Torch extra installed: `MUJOCO_METAL_RUN_GPU=1 python -m pytest -m gpu`. Ordinary `python -m pytest` runs CPU tests and skips the GPU cases. The standalone source tree carries the Apache 2.0 license and notices.
