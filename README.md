<h1>
  <a href="#"><img alt="MuJoCo" src="banner.png" width="100%"/></a>
</h1>

<p>
  <a href="https://github.com/google-deepmind/mujoco/actions/workflows/build.yml?query=branch%3Amain" alt="GitHub Actions">
    <img src="https://img.shields.io/github/actions/workflow/status/google-deepmind/mujoco/build.yml?branch=main">
  </a>
  <a href="https://mujoco.readthedocs.io/" alt="Documentation">
    <img src="https://readthedocs.org/projects/mujoco/badge/?version=latest">
  </a>
  <a href="https://github.com/google-deepmind/mujoco/blob/main/LICENSE" alt="License">
    <img src="https://img.shields.io/github/license/google-deepmind/mujoco">
  </a>
</p>

**MuJoCo** stands for **Mu**lti-**Jo**int dynamics with **Co**ntact. It is a
general purpose physics engine that aims to facilitate research and development
in robotics, biomechanics, graphics and animation, machine learning, and other
areas which demand fast and accurate simulation of articulated structures
interacting with their environment.

This repository is maintained by [Google DeepMind](https://www.deepmind.com/).

MuJoCo has a C API and is intended for researchers and developers. The runtime
simulation module is tuned to maximize performance and operates on low-level
data structures that are preallocated by the built-in XML compiler. The library
includes interactive visualization with a native GUI, rendered in OpenGL. MuJoCo
further exposes a large number of utility functions for computing
physics-related quantities.

We also provide [Python bindings] and a plug-in for the [Unity] game engine.

## Documentation

MuJoCo's documentation can be found at [mujoco.readthedocs.io]. Upcoming features due for the next
release can be found in the [changelog] in the latest branch.

## Getting Started

There are two easy ways to get started with MuJoCo:

1. **Run `simulate` on your machine.**
[This video](https://www.youtube.com/watch?v=0ORsj_E17B0) shows a screen capture
of `simulate`, MuJoCo's native interactive viewer. Follow the steps described in
the [Getting Started] section of the documentation to get `simulate` running on
your machine.

2. **Explore our online IPython notebooks.**
If you are a Python user, you might want to start with our tutorial notebooks
running on Google Colab:

 - The **introductory** tutorial teaches MuJoCo basics:
   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb)
 - The **LQR** tutorial synthesizes a linear-quadratic controller, balancing a humanoid on one leg:
   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/LQR.ipynb)
 - The **least-squares** tutorial explains how to use the Python-based nonlinear least-squares solver:
   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/least_squares.ipynb)
 - The **MJX** tutorial provides usage examples of
   [MuJoCo XLA](https://mujoco.readthedocs.io/en/stable/mjx.html), a branch of MuJoCo written in JAX:
   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/tutorial.ipynb)
 - The **differentiable physics** tutorial trains locomotion policies with analytical gradients automatically derived from MuJoCo's physics step:
   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/training_apg.ipynb)

## Installation

### Prebuilt binaries

Versioned releases are available as precompiled binaries from the GitHub
[releases page], built for Linux (x86-64 and AArch64), Windows (x86-64 only),
and macOS (universal). This is the recommended way to use the software.

### Building from source

Users who wish to build MuJoCo from source should consult the [build from
source] section of the documentation. However, please note that the commit at
the tip of the `main` branch may be unstable.

### Python (>= 3.8)

The native Python bindings, which come pre-packaged with a copy of MuJoCo, can
be installed from [PyPI] via:

```bash
pip install mujoco
```

Note that Pre-built Linux wheels target `manylinux2014`, see
[here](https://github.com/pypa/manylinux) for compatible distributions. For more
information such as building the bindings from source, see the [Python bindings]
section of the documentation.

## Contributing

We welcome community engagement: questions, requests for help, bug reports and
feature requests. To read more about bug reports, feature requests and more
ambitious contributions, please see our [contributors guide](CONTRIBUTING.md)
and [style guide](STYLEGUIDE.md).

## Asking Questions

Questions and requests for help are welcome on the GitHub
[Issues](https://github.com/google-deepmind/mujoco/issues) page and should focus
on a specific problem or question.

[Discussions](https://github.com/google-deepmind/mujoco/discussions) should
address wider concerns that might require input from multiple participants.

Here are some guidelines for asking good questions:

1. Search for existing questions or issues that touch on the same subject.

   You can add comments to existing threads or start new ones. If you start a
   new thread and there are existing relevant threads, please link to them.

2. Use a clear and specific title. Try to include keywords that will make your
   question easy for other to find in the future.

3. Introduce yourself and your project more generally.

   If your level of expertise is exceptional (either high or low), and it might
   be relevant to what we can assume you know, please state that as well.

4. Take a step back and tell us what you're trying to accomplish, if we
   understand you goal we might suggest a different type of solution than the
   one you are having problems with

5. Make it easy for others to reproduce the problem or understand your question.

   If this requires a model, please include it. Try to make the model minimal:
   remove elements that are unrelated to your question. Pure XML models should
   be inlined. Models requiring binary assets (meshes, textures), should be
   attached as a `.zip` file. Please make sure the included model is loadable
   before you attach it.

6. Include an illustrative screenshot or video, if relevant.

7. Tell us how you are accessing MuJoCo (C API, Python bindings, etc.) and which
   MuJoCo version and operating system you are using.

## Related software
MuJoCo forms the backbone of many environment packages, but these are too many
to list here individually. Below we focus on bindings and converters.

### Bindings

These packages give users of various languages access to MuJoCo functionality:

#### First-party bindings:

- [Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)
  - [dm_control](https://github.com/google-deepmind/dm_control), Google
    DeepMind's related environment stack, includes
    [PyMJCF](https://github.com/google-deepmind/dm_control/blob/main/dm_control/mjcf/README.md),
    a module for procedural manipulation of MuJoCo models.
- [C# bindings and Unity plug-in](https://mujoco.readthedocs.io/en/stable/unity.html)

#### Third-party bindings:

- **WebAssembly**: [mujoco_wasm](https://github.com/zalo/mujoco_wasm) by [@zalo](https://github.com/zalo) with contributions by
  [@kevinzakka](https://github.com/kevinzakka), based on the [emscripten build](https://github.com/stillonearth/MuJoCo-WASM) by
  [@stillonearth](https://github.com/stillonearth).

  :arrow_right: [Click here](https://zalo.github.io/mujoco_wasm/) for a live demo of MuJoCo running in your browser.
- **MATLAB Simulink**: [Simulink Blockset for MuJoCo Simulator](https://github.com/mathworks-robotics/mujoco-simulink-blockset)
  by [Manoj Velmurugan](https://github.com/vmanoj1996).
- **Swift**: [swift-mujoco](https://github.com/liuliu/swift-mujoco)
- **Java**: [mujoco-java](https://github.com/CommonWealthRobotics/mujoco-java)
- **Julia**: [MuJoCo.jl](https://github.com/JamieMair/MuJoCo.jl)


### Converters

- **OpenSim**: [MyoConverter](https://github.com/MyoHub/myoconverter) converts
  OpenSim models to MJCF.
- **SDFormat**: [gz-mujoco](https://github.com/gazebosim/gz-mujoco/) is a
  two-way SDFormat <-> MJCF conversion tool.
- **OBJ**: [obj2mjcf](https://github.com/kevinzakka/obj2mjcf)
  a script for converting composite OBJ files into a loadable MJCF model.

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

Box collision code ([`engine_collision_box.c`](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_collision_box.c))
is Copyright 2016 Svetoslav Kolev.

ReStructuredText documents, images, and videos in the `doc` directory are made
available under the terms of the Creative Commons Attribution 4.0 (CC BY 4.0)
license. You may obtain a copy of the License at
https://creativecommons.org/licenses/by/4.0/legalcode.

Source code is licensed under the Apache License, Version 2.0. You may obtain a
copy of the License at https://www.apache.org/licenses/LICENSE-2.0.

This is not an officially supported Google product.

[build from source]: https://mujoco.readthedocs.io/en/latest/programming#building-mujoco-from-source
[Getting Started]: https://mujoco.readthedocs.io/en/latest/programming#getting-started
[Unity]: https://unity.com/
[releases page]: https://github.com/google-deepmind/mujoco/releases
[GitHub Issues]: https://github.com/google-deepmind/mujoco/issues
[mujoco.readthedocs.io]: https://mujoco.readthedocs.io
[changelog]: https://mujoco.readthedocs.io/en/latest/changelog.html
[Python bindings]: https://mujoco.readthedocs.io/en/stable/python.html#python-bindings
[PyPI]: https://pypi.org/project/mujoco/
