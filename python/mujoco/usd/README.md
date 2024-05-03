# MuJoCo USD Integration

USD is file format originally introduced by Pixar to compose 3D scenes in virtual worlds. We implement the ability to export MuJoCo models and trajectories as USD files for visualization with external renderers such as NVIDIA Omniverse and Blender. 

## Installation

We assume that users have already installed MuJoCo either directly with pip or from source. For information on how to build MuJoCo from source, please visit this [reference](https://mujoco.readthedocs.io/en/stable/python.html#building-from-source). We also assume Python>=3.8.

Using the USD bridge functionality requires the installation of additional libraries. Specifically, we utilize the `open3d` and `usd-core` packages. These can be installed directly using pip.

```bash
pip install open3d
pip install usd-core
```

## Getting Started

We provide a demo script to get started with generating USD files. This demo script is located at `python/mujoco/usd/demo.py`. In this script, we load a the `cards.xml` model and step through it for a given number of steps. At each step, we make a call to the USD exporter to store information for any state changes including the positions and orientations of geoms in the scene.

## TODOs and Known Bugs

TODOs:

- Include tendons and contact forces in USD file
- Include references to third party materials for NVIDIA Omniverse in USD file
- Including custom defined matrials for NVIDIA Omniverse
- Bridge for modeling environments and assets from third party modeling engines to MuJoCo

Known issues 🐛: 
- Texture mapping for non-UV objects not consistent with MuJoCo renderer when ported to new renderers