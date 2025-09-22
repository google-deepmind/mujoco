.. _MJW:

====================
MuJoCo Warp (MJWarp)
====================

.. toctree::
    :hidden:

    API <api.rst>

MuJoCo Warp (MJWarp) is an implementation of MuJoCo written in `Warp <https://nvidia.github.io/warp/>`__ and optimized
for `Nvidia <https://nvidia.com>`__ GPUs. MJWarp lives in the
`google-deepmind/mujoco_warp <https://github.com/google-deepmind/mujoco_warp>`__ GitHub repository and is currently in
beta.

.. _MJW_getstarted:

Getting Started
===============

.. code-block:: python

    import mujoco
    import mujoco_warp as mjw

    mjm = mujoco.MjModel.from_file("example.xml")

    m = mjw.put_model(mjm)
    d = mjw.make_data(mjm)

    mjw.step(m, d)

.. _MJW_install:

Installation
============

The beta version of MuJoCo Warp is installed from GitHub. Please note that the beta version of MuJoCo Warp does not
support all versions of MuJoCo, Warp, CUDA, Nvidia drivers, etc.

.. code-block:: shell

    git clone https://github.com/google-deepmind/mujoco_warp.git
    cd mujoco_warp
    python3 -m venv env
    source env/bin/activate
    pip install --upgrade pip
    pip install uv
    uv pip install -e .[dev,cuda]

Test the Installation

.. code-block:: shell

    pytest

.. _MJW_Cli:

Utilities
=========

Benchmark an environment with testspeed

.. code-block:: shell

    mjwarp-testspeed benchmark/humanoid/humanoid.xml

Interactive environment simulation with MJWarp

.. code-block:: shell

    mjwarp-viewer benchmark/humanoid/humanoid.xml
