# MuJoCo XLA (MJX)

[![PyPI Python Version][pypi-versions-badge]][pypi]
[![PyPI version][pypi-badge]][pypi]

[pypi-versions-badge]: https://img.shields.io/pypi/pyversions/mujoco-mjx
[pypi-badge]: https://badge.fury.io/py/mujoco-mjx.svg
[pypi]: https://pypi.org/project/mujoco-mjx/

This package is a re-implementation of the
[MuJoCo physics engine](https://github.com/google-deepmind/mujoco) in
[JAX](https://github.com/jax-ml/jax). This library is developed and maintained
by Google DeepMind, and is kept up-to-date with the latest developments in
MuJoCo itself.

The `mujoco-mjx` package is API-compatible with MuJoCo, but is missing some
features found in MuJoCo.  See our
[documentation](https://mujoco.readthedocs.io/en/stable/mjx.html) for more
details concerning feature parity.

## Installation

The recommended way to install this package is via [PyPI](https://pypi.org/project/mujoco-mjx/):

```sh
pip install mujoco-mjx
```

## Usage

Once installed, the package can be imported via `from mujoco import mjx`. Please
consult our [documentation](https://mujoco.readthedocs.io/en/stable/mjx.html)
for further detail on the package's API.

We recommend going through the tutorial notebook which introduces the MJX API
and trains a reinforcement learning policy in a few minutes: [![Open In
Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/tutorial.ipynb)

## Versioning

The `major.minor.micro` portion of the version number matches the version of
MuJoCo that this library provides. Optionally, if we release updates to MJX that
target the same version of MuJoCo, a `.postN` suffix is added, for example
`3.0.1.post2` represents the second update to MJX for MuJoCo 3.0.1.

## License and Disclaimer

Copyright 2023 DeepMind Technologies Limited

MuJoCo and its libraries are licensed under the Apache License,
Version 2.0. You may obtain a copy of the License at
https://www.apache.org/licenses/LICENSE-2.0.

This is not an officially supported Google product.
