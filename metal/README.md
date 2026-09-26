# Experimental MuJoCo Metal package

This optional package targets the Python bindings for MuJoCo **3.10.0**. The surrounding source checkout is 3.14.1; that version is not a target. Install from `metal/` with `pip install -e '.[test]'`. Importing `mujoco_metal` and running preflight do not import Torch or initialize MPS.

Run `python -m mujoco_metal preflight --model path/to/model.xml --json --inventory` for the runtime version, model dimensions, package and shader paths/hash, capability boundaries, and versioned feature/API inventory. Inventory completeness is explicitly false because the release feature surface is broader than the enum and binding inventory captured so far.

`load_model(xml_or_path)` returns an immutable, dimension-derived descriptor. `descriptor.forward_kinematics(qpos)` is a CPU reference for body, inertial, geom, and site world poses across hinge, slide, ball, and free joints. `MetalKinematics(descriptor).run(qpos_batch)` is an explicit opt-in Torch MPS API for the same kinematics stage. Constructing `MetalKinematics` initializes MPS and compiles the bundled shader. It has not been GPU-qualified. Set `MUJOCO_METAL_RUN_GPU=1` to opt into `pytest -m gpu`; run this only on an idle Apple GPU after qualification constraints are satisfied.

This stage does not compute collision, forces, constraints, sensors, integration, or physics stepping. It is not a general Metal physics backend yet. The standalone source tree carries the Apache 2.0 license and notices.
