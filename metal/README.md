# Experimental MuJoCo Metal package

This optional package targets the Python bindings for MuJoCo **3.10.0**. The surrounding source checkout is 3.14.1; that version is not a target. Install from `metal/` with `pip install -e '.[test]'`. Importing `mujoco_metal` does not import Torch or initialize MPS.

The current stage implements immutable generic model lowering and CPU forward kinematics for hinge, slide, ball, and free joints, including fixed/branched bodies and world poses for bodies, inertias, geoms, and sites. `forward_kinematics` is a kinematics-only API. It does not perform collision, force calculation, constraints, integration, or stepping. GPU execution is not yet qualified.

The feature inventory is available through `mujoco_metal.registry.feature_status()`. “Implemented” names code present; qualification is tracked independently. No claim of complete MuJoCo API coverage is made. Follow-up work must enumerate the pinned 3.10 API/features before any broad support claim.
