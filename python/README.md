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

### PyPI (Recommended)

The recommended way to install this package is via [PyPI](https://pypi.org/project/mujoco/):

```sh
pip install mujoco
```

A copy of the MuJoCo library is provided as part of the package and does **not**
need to be downloaded or installed separately.

### Source

**Note.** Building from source is only necessary if you are modifying the Python bindings (or are trying to run on exceptionally old Linux systems). If that's not the case, then we recommend installing the prebuilt binaries from PyPI.

Before proceeding, make sure you have CMake and a C++17 compiler installed.

1. Download the latest release of the [binary archives](https://github.com/deepmind/mujoco/releases) from GitHub. On macOS, the download corresponds to a DMG file from which you can drag the `MuJoCo` app into your Applications folder.

2. Clone the entire `mujoco` repository from GitHub and `cd` into the python directory.

```bash
git clone https://github.com/deepmind/mujoco.git
cd mujoco/python
```

3. Create a virtual environment:

```bash
python3 -m venv /tmp/mujoco
source /tmp/mujoco/bin/activate
```

4. Generate a [source distribution](https://packaging.python.org/en/latest/glossary/#term-Source-Distribution-or-sdist)
tarball with the `make_sdist.sh` script.

```bash
cd python
bash make_sdist.sh
```

The `make_sdist.sh` script generates additional C++ header files that are needed to build the bindings, and also pulls in required files from elsewhere in the repository outside the `python` directory into the sdist. Upon completion, the script will create a `dist` directory with a `mujoco-2.1.X.tar.gz` file (where X is the version number of the release).

5. Install the generated tarball. You'll need to specify the path to the MuJoCo library you downloaded earlier. For example, on macOS, this will be `/Applications/MuJoCo.app/Contents/Frameworks/MuJoCo.framework` if you dragged it to your Applications folder.

```bash
cd dist
MUJOCO_PATH=/PATH/TO/MUJOCO pip install mujoco-2.1.X.tar.gz
```

The Python bindings should now be installed! To check that they've been successfully installed, `cd` outside of the `mujoco` directory and run `python -c "import mujoco"`.

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
