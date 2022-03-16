# MuJoCo Python Bindings

This package is the canonical Python bindings for the
[MuJoCo physics engine](https://github.com/deepmind/mujoco).
These bindings are developed and maintained by DeepMind, and is kept up-to-date
with the latest developments in MuJoCo itself.

The `mujoco` package provides direct access to raw MuJoCo C API functions,
structs, constants, and enumerations. Structs are provided as Python classes,
with Pythonic initialization and deletion semantics.

It is not the aim of this package to provide fully fledged
scene/environment/game authoring API, as there are already a number of existing
packages that do this well. However, this package does provide a number of
lower-level components outside of MuJoCo itself that are likely to be useful to
most users who access MuJoCo through Python. For example, the `egl`, `glfw`, and
`osmesa` subpackages contain utilities for setting up OpenGL rendering contexts.

## Installation

The package can be installed from [PyPI](https://pypi.org/project/mujoco/) via

```sh
pip install mujoco
```

A copy of the MuJoCo library is provided as part of the package and does **not**
need to be downloaded or installed separately.

If you wish to modify and build the bindings from source, you should clone the
entire `mujoco` repository from GitHub, then run the `make_sdist.sh` script to
generate a [source distribution](https://packaging.python.org/en/latest/glossary/#term-Source-Distribution-or-sdist)
tarball, then run `pip wheel name_of_sdist.tar.gz`. The `make_sdist.sh` script
generates additional C++ header files that are needed to build the bindings,
and also pulls in required files from elsewhere in the repository outside the
`python` directory into the sdist.

CMake and a C++17 compiler are needed to build the bindings from source.

## Usage

Once installed, the package can be imported via `import mujoco`. Please consult
our [documentation](https://mujoco.readthedocs.io/en/latest/python.html) for
further detail on the package's API.

## Versioning

The `major.minor.micro` portion of the version number matches the version of
MuJoCo that the bindings provide. Optionally, if we release updates to the
Python bindings themselves that target the same version of MuJoCo, a `.postN`
suffix is added, for example `2.1.2.post2` represents the second update to the
bindings for MuJoCo 2.1.2.

## License and Disclaimer

Copyright 2022 DeepMind Technologies Limited

MuJoCo and its Python bindings are licensed under the Apache License,
Version 2.0. You may obtain a copy of the License at
https://www.apache.org/licenses/LICENSE-2.0.

This is not an officially supported Google product.
