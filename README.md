# MuJoCo Physics

**MuJoCo** stands for **Mu**lti-**Jo**int dynamics with **Co**ntact. It is a
general purpose physics engine that aims to facilitate research and development
in robotics, biomechanics, graphics and animation, machine learning, and other
areas which demand fast and accurate simulation of articulated structures
interacting with their environment.

DeepMind has acquired MuJoCo, and we are currently making preparations to open
source the codebase. In the meantime, MuJoCo is available for download as a free
and unrestricted precompiled binary under the Apache 2.0 license from
[mujoco.org](https://mujoco.org/).

MuJoCo's source code will be released through this GitHub repository once it is
ready. In the meantime, the repository hosts MuJoCo's documentation, C header
files for its public API, and sample program code. If you wish to report bugs or
make feature requests, please file them as [GitHub Issues]. You are also
welcome to make pull requests for the [documentation source files].

## Overview

MuJoCo is a C/C++ library with a C API, intended for researchers and developers.
The runtime simulation module is tuned to maximize performance and operates on
low-level data structures which are preallocated by the built-in XML parser and
compiler. The user defines models in the native MJCF scene description language
-- an XML file format designed to be as human readable and editable as possible.
URDF model files can also be loaded. The library includes interactive
visualization with a native GUI, rendered in OpenGL. MuJoCo further exposes a
large number of utility functions for computing physics-related quantities, not
necessarily in a simulation loop. Features include

-   Simulation in generalized coordinates, avoiding joint violations.

-   Inverse dynamics that are well-defined even in the presence of contacts.

-   Unified continuous-time formulation of constraints via convex optimization.

-   Constraints include soft contacts, limits, dry friction, equality
    constraints.

-   Simulation of particle systems, cloth, rope and soft objects.

-   Actuators including motors, cylinders, muscles, tendons, slider-cranks.

-   Choice of Newton, Conjugate Gradient, or Projected Gauss-Seidel solvers.

-   Choice of pyramidal or elliptic friction cones, dense or sparse Jacobians.

-   Choice of Euler or Runge-Kutta numerical integrators.

-   Multi-threaded sampling and finite-difference approximations.

-   Intuitive XML model format (called MJCF) and built-in model compiler.

-   Cross-platform GUI with interactive 3D visualization in OpenGL.

-   Run-time module written in ANSI C and hand-tuned for performance.


## Requirements

MuJoCo binaries are currently built for Linux, macOS (Intel), and Windows.


## Documentation

MuJoco's current documentation is available at [mujoco.org/book], which is
serving Sphinx-based webpages derived from the ReStructuredText
[documentation source files].


## Citation

If you use MuJoCo for published research, please cite:

```
@inproceedings{todorov2012mujoco,
  title={Mujoco: A physics engine for model-based control},
  author={Todorov, Emanuel and Erez, Tom and Tassa, Yuval},
  booktitle={2012 IEEE/RSJ International Conference on Intelligent Robots and Systems},
  pages={5026--5033},
  year={2012},
  organization={IEEE}
}
```


## License and Disclaimer

Copyright 2021 DeepMind Technologies Limited

ReStructuredText documents, images, and videos in the `doc` directory are made
available under the terms of the Creative Commons Attribution 4.0 (CC BY 4.0)
license. You may obtain a copy of the License at
https://creativecommons.org/licenses/by/4.0/legalcode.

Source code is licensed under the Apache License, Version 2.0. You may obtain a
copy of the License at https://www.apache.org/licenses/LICENSE-2.0.

This is not an officially supported Google product.


[GitHub Issues]: https://github.com/deepmind/mujoco/issues
[documentation source files]: https://github.com/deepmind/mujoco/tree/main/doc
[mujoco.org/book]: https://mujoco.org/book
