# MuJoCo Physics

**MuJoCo** stands for **Mu**lti-**Jo**int dynamics with **Co**ntact. It is a
general purpose physics engine that aims to facilitate research and development
in robotics, biomechanics, graphics and animation, machine learning, and other
areas which demand fast and accurate simulation of articulated structures
interacting with their environment.

This repository is maintained by DeepMind, please see our [acquisition] and
[open sourcing] announcements.

MuJoCo has a C API and is intended for researchers and developers. The runtime
simulation module is tuned to maximize performance and operates on low-level
data structures that are preallocated by the built-in XML compiler. The library
includes interactive visualization with a native GUI, rendered in OpenGL. MuJoCo
further exposes a large number of utility functions for computing physics-
related quantities. We also provide Python bindings and a plug-in for the Unity
game engine.

## Documentation

MuJoCo's documentation is available at [mujoco.readthedocs.io], which serves
webpages derived from the [documentation source files].

## Releases

Versioned releases are available as precompiled binaries from the GitHub
[releases page], built for Linux (x86-64 and AArch64), Windows (x86-64 only),
and macOS (universal). This is the recommended way to use the software.

Users who wish to build MuJoCo from source, please consult the [build from
source] section of the documentation. However, please note that the commit at
the tip of the `main` branch branch may be unstable.


## Getting Started

There are two easy ways to get started with MuJoCo:

1. **Run `simulate` on your machine.**
[This video](https://www.youtube.com/watch?v=0ORsj_E17B0) shows a screen capture
of `simulate`, MuJoCo's native interactive viewer. Follow the steps described in
the [Getting Started] section of the documentation to get `simulate` running on
your machine.

2. **Explore our online IPython notebooks.**
If you are a Python user, you might want to start with our tutorial notebooks,
running on Google Colab:

  - The first tutorial focuses on the basic MuJoco Python bindings: [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/deepmind/dm_control/blob/main/dm_control/mujoco/tutorial.ipynb).

  - The second tutorial includes more examples of `dm_control`-specific functionality: [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/deepmind/dm_control/blob/main/tutorial.ipynb).


## Citation

If you use MuJoCo for published research, please cite:

```
@inproceedings{todorov2012mujoco,
  title={MuJoCo: A physics engine for model-based control},
  author={Todorov, Emanuel and Erez, Tom and Tassa, Yuval},
  booktitle={2012 IEEE/RSJ International Conference on Intelligent Robots and Systems},
  pages={5026--5033},
  year={2012},
  organization={IEEE},
  doi={10.1109/IROS.2012.6386109}
}
```


## License and Disclaimer

Copyright 2021 DeepMind Technologies Limited.

Box collision code ([`engine_collision_box.c`](https://github.com/deepmind/mujoco/tree/main/src/engine/engine_collision_box.c))
is Copyright 2016 Svetoslav Kolev.

ReStructuredText documents, images, and videos in the `doc` directory are made
available under the terms of the Creative Commons Attribution 4.0 (CC BY 4.0)
license. You may obtain a copy of the License at
https://creativecommons.org/licenses/by/4.0/legalcode.

Source code is licensed under the Apache License, Version 2.0. You may obtain a
copy of the License at https://www.apache.org/licenses/LICENSE-2.0.

This is not an officially supported Google product.

[build from source]: https://mujoco.readthedocs.io/en/latest/programming.html#building-mujoco-from-source
[Getting Started]: https://mujoco.readthedocs.io/en/latest/programming.html#getting-started
[acquisition]: https://www.deepmind.com/blog/opening-up-a-physics-simulator-for-robotics
[open sourcing]: https://www.deepmind.com/blog/open-sourcing-mujoco
[releases page]: https://github.com/deepmind/mujoco/releases
[GitHub Issues]: https://github.com/deepmind/mujoco/issues
[documentation source files]: https://github.com/deepmind/mujoco/tree/main/doc
[mujoco.readthedocs.io]: https://mujoco.readthedocs.io
