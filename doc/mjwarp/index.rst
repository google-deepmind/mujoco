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

.. _MJW_Usage:

Basic usage
===========

Once installed, the package can be imported via ``import mujoco_warp as mjw``. Structs, functions, and enums are
available directly from the top-level ``mjw`` module.

Structs
-------
Before running MJWarp functions on an Nvidia GPU, structs must be copied onto the device via ``mjw.put_model`` and
``mjw.make_data`` or ``mjw.put_data`` functions. Placing an :ref:`mjModel` on device yields an ``mjw.Model``. Placing
an :ref:`mjData` on device yields an ``mjw.Data``:

.. code-block:: python

  mjm = mujoco.MjModel.from_xml_string("...")
  mjd = mujoco.MjData(mjm)
  m = mjw.put_model(mjm)
  d = mjw.put_data(mjm, mjd)

These MJWarp variants mirror their MuJoCo counterparts but have a few key differences:

#. ``mjw.Model`` and ``mjw.Data`` contain Warp arrays that are copied onto device.
#. Some fields are missing from ``mjw.Model`` and ``mjw.Data`` for features that are unsupported.

Functions
_________

MuJoCo functions are exposed as MJWarp functions of the same name, but following
`PEP 8 <https://peps.python.org/pep-0008/>`__-compliant names. Most of the :ref:`main simulation <Mainsimulation>` and
some of the :ref:`sub-components <Subcomponents>` for forward simulation are available from the top-level ``mjw``
module.

Minimal example
---------------

.. code-block:: python

   # Throw a ball at 100 different velocities.

   import mujoco
   import mujoco_warp as mjw
   import warp as wp

   _MJCF=r"""
   <mujoco>
     <worldbody>
       <body>
         <freejoint/>
         <geom size=".15" mass="1" type="sphere"/>
       </body>
     </worldbody>
   </mujoco>
   """

   mjm = mujoco.MjModel.from_xml_string(_MJCF)
   m = mjw.put_model(mjm)
   d = mjw.make_data(mjm, nworld=100)

   # initialize velocities
   wp.copy(d.qvel, wp.array([[float(i) / 100, 0.0, 0.0, 0.0, 0.0, 0.0] for i in range(100)], dtype=float))

   # simulate physics
   mjw.step(m, d)

   print(f'qpos:\n{d.qpos.numpy()}')

A call to ``mjw.step`` is comprised of a collection of kernel launches. Warp will launch these kernels individually if
this function is called directly. To improve performance, especially if the function will be called multiple times, it
is recommended to capture the operations that comprise the function as a CUDA graph

.. code-block:: python

  with wp.ScopedCapture() as capture:
    mjw.step(m, d)

The graph can then be launched or re-launched

.. code-block:: python

  wp.capture_launch(capture.graph)

and will typically be significantly faster compared to calling ``mjw.step`` directly. Please see the
`Warp Graph API reference <https://nvidia.github.io/warp/modules/runtime.html#graph-api-reference>`__ for details.

.. _MJW_Cli:

Helpful Command Line Scripts
----------------------------

Benchmark an environment with testspeed

.. code-block:: shell

    mjwarp-testspeed benchmark/humanoid/humanoid.xml

Interactive environment simulation with MJWarp

.. code-block:: shell

    mjwarp-viewer benchmark/humanoid/humanoid.xml

Feature Parity
==============

MJWarp supports most of the main simulation features of MuJoCo, with a few exceptions. MJWarp will raise an exception if
asked to copy to device an :ref:`mjModel` with field values referencing unsupported features.

The following features are **not supported** in MJWarp:

.. list-table::
   :width: 90%
   :align: left
   :widths: 2 5
   :header-rows: 1

   * - Category
     - Feature
   * - :ref:`Equality <mjtEq>`
     - ``FLEX``
   * - :ref:`Integrator <mjtIntegrator>`
     - ``IMPLICIT``, ``IMPLICITFAST`` not supported with fluid drag
   * - :ref:`Solver <mjtSolver>`
     - ``PGS``, ``noslip``, :ref:`islands <soIsland>`
   * - Fluid Model
     - :ref:`flEllipsoid`
   * - :ref:`Sensors <mjtSensor>`
     - ``GEOMDIST``, ``GEOMNORMAL``, ``GEOMFROMTO``
   * - Flex
     - ``VERTCOLLIDE=false``, ``INTERNAL=true``, ``nflex > 1``
   * - Jacobian format
     - ``SPARSE``
   * - Option
     - :ref:`contact override <COverride>`
   * - Plugins
     - ``All`` except ``SDF``
   * - :ref:`User parameters <CUser>`
     - ``All``
