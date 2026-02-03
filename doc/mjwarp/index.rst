.. _MJW:

====================
MuJoCo Warp (MJWarp)
====================

.. toctree::
    :hidden:

    API <api.rst>

MuJoCo Warp (MJWarp) is an implementation of MuJoCo written in `Warp <https://nvidia.github.io/warp/>`__ and optimized
for `NVIDIA <https://nvidia.com>`__ hardware and parallel simulation. MJWarp lives in the
`google-deepmind/mujoco_warp <https://github.com/google-deepmind/mujoco_warp>`__ GitHub repository and is currently in
beta.

MJWarp is developed and maintained as a joint effort by `NVIDIA <https://nvidia.com>`__ and
`Google DeepMind <https://deepmind.google/>`__.

.. _MJW_tutorial:

Tutorial notebook
=================

The MJWarp basics are covered in a
`tutorial
notebook <https://colab.research.google.com/github/google-deepmind/mujoco_warp/blob/main/notebooks/tutorial.ipynb>`__.

When To Use MJWarp?
===================

.. TODO(robotics-simulation): batch renderer

High throughput
---------------

The MuJoCo ecosystem offers multiple options for batched simulation.

- :ref:`mujoco.rollout <PyRollout>`: Python API for multi-threaded calls to :ref:`mj_step` on CPU. High throughput
  can be achieved with hardware that has fast cores and large thread counts, but overall performance of applications
  requiring frequent host<>device transfers (e.g., reinforcement learning with simulation on CPU and learning on GPU)
  may be bottlenecked by transfer overhead.
- **mjx.step**: `jax.vmap` and `jax.pmap` enable multi-threaded and multi-device simulation with JAX on CPUs, GPUs, or
  TPUs.
- :func:`mujoco_warp.step <mujoco_warp.step>`: Python API for multi-threaded and multi-device simulation with CUDA via
  Warp on NVIDIA GPUs. Improved scaling for contact-rich scenes compared to the MJX JAX implementation.

.. TODO(robotics-simulation): add link to mjx.step
.. TODO(robotics-simulation): add step/time comparison plot

Low latency
-----------

MJWarp is optimized for throughput: the total number of simulation steps per unit time whereas MuJoCo is optimized for
latency: time for one simulation step. It is expected that a simulation step with MJWarp will be less performant
than a step with MuJoCo for the same simulation.

As a result, MJWarp is well suited for applications where large numbers of
samples are required, like reinforcement learning, while MuJoCo is likely more useful for real-time applications like
online control (e.g., model predictive control) or interactive graphical interfaces (e.g., simulation-based
teleoperation).

Complex scenes
--------------

MJWarp scales better than MJX for scenes with many geoms or degrees of freedom, but not as well as MuJoCo. There may be
significant performance degradation in MJWarp for scenes beyond 60 DoFs. Supporting these larger scenes is a high
priority and progress is tracked in GitHub issues for: sparse Jacobians
`#88 <https://github.com/google-deepmind/mujoco_warp/issues/88>`__, block Cholesky factorization and solve
`#320 <https://github.com/google-deepmind/mujoco_warp/issues/320>`__, constraint islands
`#886 <https://github.com/google-deepmind/mujoco_warp/issues/886>`__, and sleeping islands
`#887 <https://github.com/google-deepmind/mujoco_warp/issues/887>`__.

.. TODO(robotic-simulation): add graph for ngeom and nv scaling

Differentiability
-----------------

The dynamics API in MJX is automatically differentiable via JAX. We are considering whether to support this in MJWarp
via Warp - if this feature is important to you, please chime in on this issue
`here <https://github.com/google-deepmind/mujoco_warp/issues/500>`__.

.. TODO(robotics-simulation): Newton multi-physics

.. _MJW_install:

Installation
============

The beta version of MuJoCo Warp is installed from GitHub. Please note that the beta version of MuJoCo Warp does not
support all versions of MuJoCo, Warp, CUDA, NVIDIA drivers, etc.

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

Basic Usage
===========

Once installed, the package can be imported via ``import mujoco_warp as mjw``. Structs, functions, and enums are
available directly from the top-level :mod:`mjw <mujoco_warp>` module.

Structs
-------
Before running MJWarp functions on an NVIDIA GPU, structs must be copied onto the device via
:func:`mjw.put_model <mujoco_warp.put_model>` and :func:`mjw.make_data <mujoco_warp.make_data>` or
:func:`mjw.put_data <mujoco_warp.put_data>` functions. Placing an :ref:`mjModel` on device yields an
:class:`mjw.Model <mujoco_warp.Model>`. Placing an :ref:`mjData` on device yields an
:class:`mjw.Data <mujoco_warp.Data>`.

.. code-block:: python

  mjm = mujoco.MjModel.from_xml_string("...")
  mjd = mujoco.MjData(mjm)
  m = mjw.put_model(mjm)
  d = mjw.put_data(mjm, mjd)

These MJWarp variants mirror their MuJoCo counterparts but have a few key differences:

#. :class:`mjw.Model <mujoco_warp.Model>` and :class:`mjw.Data <mujoco_warp.Data>` contain Warp arrays that are copied
   onto device.
#. Some fields are missing from :class:`mjw.Model <mujoco_warp.Model>` and :class:`mjw.Data <mujoco_warp.Data>` for
   features that are unsupported.

Batch sizes
-----------

MJWarp is optimized for parallel simulation. A batch of simulations can be specified with three parameters:

- :attr:`nworld <mujoco_warp.Data.nworld>`: Number of worlds to simulate.
- _`nconmax`: Expected number of contacts per world. The maximum number of contacts for all worlds is
  ``nconmax * nworld``.
- _`naconmax`: Alternative to `nconmax`_, maximum number of contacts over all worlds. If `nconmax`_ and `naconmax`_ are
  both set then `nconmax`_ is ignored.
- _`njmax`: Maximum number of constraints per world.

.. admonition:: Semantic difference for `nconmax`_ and `njmax`_.
  :class: note

  It is possible for the number of contacts per world to exceed `nconmax`_ if the total number of contacts for all
  worlds does not exceed ``nworld x nconmax``. However, the number of constraints per world is strictly limited by
  `njmax`_.

.. admonition:: XML parsing
  :class: note

  Values for `nconmax`_ and `njmax`_ are not parsed from :ref:`size/nconmax <size-nconmax>` and
  :ref:`size/njmax <size-njmax>` (these parameters are deprecated). Values for these parameters must be provided to
  :func:`mjw.make_data <mujoco_warp.make_data>` or :func:`mjw.put_data <mujoco_warp.put_data>`.

Functions
---------

MuJoCo functions are exposed as MJWarp functions of the same name, but following
`PEP 8 <https://peps.python.org/pep-0008/>`__-compliant names. Most of the :ref:`main simulation <Mainsimulation>` and
some of the :ref:`sub-components <Subcomponents>` for forward simulation are available from the top-level
:mod:`mjw <mujoco_warp>` module.

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
   wp.copy(d.qvel, wp.array([[float(i) / 100, 0, 0, 0, 0, 0] for i in range(100)], dtype=float))

   # simulate physics
   mjw.step(m, d)

   print(f'qpos:\n{d.qpos.numpy()}')

.. _mjwCLI:

Command line scripts
--------------------

Benchmark an environment with _`testspeed`

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
   * - :ref:`Integrator <mjtIntegrator>`
     - ``IMPLICIT``, ``IMPLICITFAST`` not supported with fluid drag
   * - :ref:`Solver <mjtSolver>`
     - ``PGS``, ``noslip``, :ref:`islands <soIsland>`
   * - Fluid Model
     - :ref:`flEllipsoid`
   * - :ref:`Sensors <mjtSensor>`
     - ``GEOMDIST``, ``GEOMNORMAL``, ``GEOMFROMTO``
   * - Flex
     - ``VERTCOLLIDE=false``, ``INTERNAL=true``
   * - Jacobian format
     - ``SPARSE``
   * - Option
     - :ref:`contact override <COverride>`
   * - Plugins
     - ``All`` except ``SDF``
   * - :ref:`User parameters <CUser>`
     - ``All``

.. _mjwPerf:

Performance Tuning
==================

The following are considerations for optimizing the performance of MJWarp.

.. _mjwGC:

Graph capture
-------------

MJWarp functions, for example :func:`mjw.step <mujoco_warp.step>`, often comprise a collection of kernel launches. Warp
will launch these kernels individually if the function is called directly. To improve performance, especially if the
function will be called multiple times, it is recommended to capture the operations that comprise the function as a CUDA
graph

.. code-block:: python

  with wp.ScopedCapture() as capture:
    mjw.step(m, d)

The graph can then be launched or re-launched

.. code-block:: python

  wp.capture_launch(capture.graph)

and will typically be significantly faster compared to calling the function directly. Please see the
`Warp Graph API reference <https://nvidia.github.io/warp/modules/runtime.html#graph-api-reference>`__ for details.

Batch sizes
-----------

The maximum numbers of contacts and constraints, `nconmax`_ / `naconmax`_ and `njmax`_ respectively, are specified when
creating :class:`mjw.Data <mujoco_warp.Data>` with :func:`mjw.make_data <mujoco_warp.make_data>` or
:func:`mjw.put_data <mujoco_warp.put_data>`. Memory and computation scales with the values of these parameters. For best
performance, the values of these parameters should be set as small as possible while ensuring the simulation does not
exceed these limits.

It is expected that good values for these limits will be environment specific. In practice, selecting good values
typically involves trial-and-error. :func:`mjwarp-testspeed <mujoco_warp.testspeed>` with the flag `--measure_alloc` for
printing the number of contacts and constraints at each simulation step and interacting with the simulation via
:func:`mjwarp-viewer <mujoco_warp.viewer>` and checking for overflow errors can both be useful techniques for
iteratively testing values for these parameters.

Solver iterations
-----------------

MuJoCo's default solver settings for the maximum numbers of :ref:`solver iterations<option-iterations>` and
:ref:`linesearch iterations<option-ls_iterations>` are expected to provide reasonable performance. Reducing MJWarp's
settings :attr:`Option.iterations <mujoco_warp.Option.iterations>` and/or
:attr:`Option.ls_iterations <mujoco_warp.Option.ls_iterations>` limits may improve performance and should be secondary
considerations after tuning `nconmax`_ / `naconmax`_ and `njmax`_.

Reducing these limits too much may prevent the constraint solver from converging and can lead to inaccurate or unstable
simulation.

.. admonition:: Impact on Performance: MJX (JAX) and MJWarp
  :class: note

  In :ref:`MJX<mjx>` these solver parameters are key for controlling simulation performance. With MJWarp, in contrast,
  once all worlds have converged the solver can early exit and avoid unnecessary computation. As a result, the values
  of these settings have comparatively less impact on performance.

Contact sensor matching
-----------------------

Scenes that include :ref:`contact sensors<sensor-contact>` have a parameter that specifies the maximum number of matched
contacts per sensor :attr:`Option.contact_sensor_max_match <mujoco_warp.Option.contact_sensor_max_match>`. For best
performance, the value of this parameter should be as small as possible while ensuring the simulation does not exceed
the limit. Matched contacts that exceed this limit will be ignored.

The value of this parameter can be set directly, for example ``model.opt.contact_sensor_maxmatch = 16``, or via an XML
custom numeric field

.. code-block:: xml

   <custom>
     <numeric name="contact_sensor_maxmatch" data="16"/>
   </custom>

Similar to the maximum numbers of contacts and constraints, a good value for this setting is expected to be environment
specific. :func:`mjwarp-testspeed <mujoco_warp.testspeed>` and :func:`mjwarp-viewer <mujoco_warp.viewer>` may be useful
for tuning the value of this parameter.

Parallel linesearch
-------------------

In addition to the constraint solver's iterative linesearch, MJWarp provides a parallel linesearch routine that
evaluates a set of step sizes in parallel and selects the best one. The step sizes are spaced logarithmically from
:attr:`Model.opt.ls_parallel_min_step <mujoco_warp.Option.ls_parallel_min_step>` to 1 and the number of step sizes to
evaluate is set via :attr:`Model.opt.ls_iterations <mujoco_warp.Option.ls_iterations>`.

In some cases the parallel routine may provide improved performance compared to the constraint solver's default
iterative linesearch.

To enable this routine set ``Model.opt.ls_parallel=True`` or add a custom numeric field to the XML

.. code-block:: xml

   <custom>
     <numeric name="ls_parallel" data="1"/>
   </custom>

.. admonition:: Experimental feature
  :class: note

  The parallel linesearch is currently an experimental feature.

.. _mjwBatch:


Memory
------

Simulation throughput is often limited by memory requirements for large numbers of worlds. Considerations for optimizing
memory utilization include:

- CCD colliders require more memory than primitive colliders, see MuJoCo's :ref:`pair-wise colliders table <coPairwise>`
  for information about colliders.
- :ref:`multiccd <option-flag-multiccd>` requires more memory than CCD.
- CCD memory requirements scale linearly with :ref:`Option.ccd_iterations <option-ccd_iterations>`.
- A scene with at least one mesh geom and using :ref:`multiccd <option-flag-multiccd>` will have memory requirements
  that scale linearly with the maximum number of vertices per face and with the maximum number of edges per vertex,
  computed over all meshes.

`testspeed`_ provides the flag ``--memory`` for reporting a simulation's total memory utilization and information about
:class:`mjw.Model <mujoco_warp.Model>` and :class:`mjw.Data <mujoco_warp.Data>` fields that require significant memory.
Memory allocated inline, including for CCD and the constraint solver, can also be significant and is reported as
``Other memory``.

.. admonition:: Maximum number of contacts per collider
  :class: note

  Some MJWarp colliders have a different maximum number of contacts compared to MuJoCo:

  - ``PLANE<>MESH``: 4 versus 3
  - ``HFieldCCD``: 4 versus ``mjMAXCONPAIR``

.. admonition:: Sparsity
  :class: note

  Sparse Jacobians can enable significant memory savings. Updates for this feature are tracked in GitHub issue
  `#88 <https://github.com/google-deepmind/mujoco_warp/issues/88>`__.

Batched :class:`Model <mujoco_warp.Model>` Fields
=================================================

To enable batched simulation with different model parameter values, many :class:`mjw.Model <mujoco_warp.Model>` fields
have a leading batch dimension. By default, the leading dimension is 1 (i.e., ``field.shape[0] == 1``) and the same
value(s) will be applied to all worlds. It is possible to override one of these fields with a ``wp.array`` that has a
leading dimension greater than one. This field will be indexed with a modulo operation of the world id and batch
dimension: ``field[worldid % field.shape[0]]``.

.. admonition:: Graph capture
  :class: warning

  The field array should be overridden prior to
  :ref:`graph capture <mjwGC>` (i.e., ``wp.ScopedCapture``)
  since the update will not be applied to an existing graph.

.. code-block:: python

   # override shape and values
   m.dof_damping = wp.array([[0.1], [0.2]], dtype=float)

   with wp.ScopedCapture() as capture:
     mjw.step(m, d)

It is possible to override the field shape and set the field values after graph capture

.. code-block:: python

   # override shape
   m.dof_damping = wp.empty((2, 1), dtype=float)

   with wp.ScopedCapture() as capture:
     mjw.step(m, d)

   # set batched values
   dof_damping = wp.array([[0.1], [0.2]], dtype=float)
   wp.copy(m.dof_damping, dof_damping)  # m.dof = dof_damping will not update the captured graph

Modifying fields
----------------

The recommended workflow for modifying an :ref:`mjModel` field is to first modify the corresponding :ref:`mjSpec` and
then compile to create a new :ref:`mjModel` with the updated field. However, compilation currently requires a host call:
1 call per new field instance, i.e., ``nworld`` host calls for ``nworld`` instances.

Certain fields are safe to modify directly without compilation, enabling on-device updates. Please see
:ref:`mjModel changes<sichange>` for details about specific fields. Additionally,
`GitHub issue 893 <https://github.com/google-deepmind/mujoco_warp/issues/893>`__ tracks adding on-device updates for a
subset of fields.

.. admonition:: Heterogeneous worlds
   :class: note

   Heterogeneous worlds, for example: per-world meshes or number of degrees of freedom, are not currently available.

.. _mjwFAQ:

Frequently Asked Questions
==========================

Learning frameworks
-------------------

**Does MJWarp work with JAX?**

Yes. MJWarp is interoperable with `JAX <https://jax.readthedocs.io/>`__. Please see the
`Warp Interoperability <https://nvidia.github.io/warp/modules/interoperability.html#jax>`__ documentation for details.

Additionally, :ref:`MJX <mjx>` provides a JAX API for a subset of MJWarp's :doc:`API <api>`. The implementation is
specified with ``impl='warp'``.

**Does MJWarp work with PyTorch?**

Yes. MJWarp is interoperable with `PyTorch <https://pytorch.org>`__. Please see the
`Warp Interoperability <https://nvidia.github.io/warp/modules/interoperability.html#pytorch>`__ documentation for
details.

**How to train policies with MJWarp physics?**

For examples that train policies with MJWarp physics, please see:

- `Isaac Lab <https://github.com/isaac-sim/IsaacLab/tree/feature/newton>`__: Train via
  `Newton API <https://github.com/newton-physics/newton>`__.
- `mjlab <https://github.com/mujocolab/mjlab>`__: Train directly with MJWarp using PyTorch.
- `MuJoCo Playground <https://github.com/google-deepmind/mujoco_playground>`__: Train via :ref:`MJX API <mjx>`.

Features
--------

.. _mjwDiff:

**Is MJWarp differentiable?**

No. MJWarp is not currently differentiable via
Warp's `automatic differentiation <https://nvidia.github.io/warp/modules/differentiability.html#differentiability>`__
functionality. Updates from the team related to enabling automatic differentiation for MJWarp are tracked in this
`GitHub issue <https://github.com/google-deepmind/mujoco_warp/issues/500>`__.

**Does MJWarp work with multiple GPUs?**

Yes. Warp's ``wp.ScopedDevice`` enables multi-GPU computation

.. code-block:: python

   # create a graph for each device
   graph = {}
   for device in wp.get_cuda_devices():
     with wp.ScopedDevice(device):
       m = mjw.put_model(mjm)
       d = mjw.make_data(mjm)
       with wp.ScopedCapture(device) as capture:
         mjw.step(m, d)
       graph[device] = capture.graph

   # launch a graph on each device
   for device in wp.get_cuda_devices():
     wp.capture_launch(graph[device])

Please see the
`Warp documentation <https://nvidia.github.io/modules/devices.html#example-using-wp-scopeddevice-with-multiple-gpus>`__
for details and
`mjlab distributed training <https://github.com/mujocolab/mjlab/tree/main/docs/api/distributed_training.md>`__ for a
reinforcement learning example.

**Is MJWarp on GPU deterministic?**

No. There may be ordering or *small* numerical differences between results computed by different executions of the same
code. This is characteristic of non-deterministic atomic operations on GPU. Set device to CPU with
``wp.set_device("cpu")`` for deterministic results.

Developments for deterministic results on GPU are tracked in this
`GitHub issue <https://github.com/google-deepmind/mujoco_warp/issues/562>`__.

**How are orientations represented?**

Orientations are represented as unit quaternions and follow :ref:`MuJoCo's conventions<siLayout>`:
``w, x, y, z`` or ``scalar, vector``.

.. admonition:: ``wp.quaternion``
  :class: note

  MJWarp utilizes Warp's `built-in type <https://nvidia.github.io/warp/modules/functions.html#warp.quaternion>`__
  ``wp.quaternion``. Importantly however, MJWarp does not utilize Warp's ``x, y, z, w`` quaternion convention or
  operations and instead implements quaternion routines that follow MuJoCo's conventions. Please see
  `math.py <https://github.com/google-deepmind/mujoco_warp/blob/main/mujoco_warp/_src/math.py>`__ for the
  implementations.

**Does MJWarp have a named access API / bind?**

No. Updates for this feature are tracked in this
`GitHub issue <https://github.com/google-deepmind/mujoco_warp/issues/884>`__.

**Why are contacts reported when there are no collisions?**

1 contact will be reported for each unique geom pair that contributes to any collision sensor, even if this geom pair is
not in collision. Unlike MuJoCo or MJX where :ref:`collision sensors<collision-sensors>` make separate calls to
collision routines while computing sensor data, MJWarp computes and stores the data for these sensors in contacts while
running its main collision pipeline.

:ref:`Contact sensors<sensor-contact>` will report the correct information for contacts affecting the physics.

**Why are Jacobians always dense?**

Sparse Jacobians are not currently implemented and ``Data`` fields: ``ten_J``, ``actuator_moment``, ``flexedge_J``, and
``efc.J`` are always represented as dense matrices. Support for sparse Jacobians is tracked in GitHub issue
`#88 <https://github.com/google-deepmind/mujoco_warp/issues/88>`__.

**Why do some arrays have different shapes compared to mjModel or mjData?**

By default for batched simulation, many :class:`mjw.Data <mujoco_warp.Data>` fields having a leading batch dimension of
size ``Data.nworld``. Some :class:`mjw.Model <mujoco_warp.Model>` fields having a leading batch dimension with size
``1``, indicating that this
:ref:`field can be overridden with an array of batched parameters for domain randomization <mjwBatch>`.

Additionally, certain fields including ``Model.qM``, ``Data.efc.J``, and ``Data.efc.D`` are padded to enable fast
loading on GPU.

**Why are numerical results from MJWarp and MuJoCo different?**

MJWarp utilizes `float <https://nvidia.github.io/warp/modules/functions.html#warp.float32>`__s in contrast to MuJoCo's
default double representation for :ref:`mjtNum`. Solver settings, including iterations, collision detection, and small
friction values may be sensitive to differences in floating point representation.

If you encounter unexpected results, including NaNs, please open a GitHub issue.

**Why is inertia matrix qM sparsity not consistent with MuJoCo / MJX?**

.. admonition:: ``mjtJacobian`` semantics
   :class: note

   - MuJoCo's inertia matrix is always sparse and :ref:`mjtJacobian` affects constraint Jacobians and related quantities
   - MJWarp's (and MJX's) constraint Jacobian is always dense and :ref:`mjtJacobian` is repurposed to affect the inertia
     matrix that can be represented as dense or sparse

The automatic sparsity threshold utilized by MJWarp for ``AUTO`` is optimized for GPU and set to ``nv > 32``,
unlike MuJoCo and MJX which use ``nv >= 60``. Dense ``DENSE`` and sparse ``SPARSE`` settings are consistent with MuJoCo
and MJX.

This feature is likely to change in the future.

**How to fix simulation runtime warnings?**

Warnings are provided when memory requirements exceed existing allocations during simulation:

- `nconmax`_ / `njmax`_: The maximum number of contacts / constraints has been exceeded. Increase the value of the
  setting by updating the relevant argument to :func:`mjw.make_data <mujoco_warp.make_data>` or
  :func:`mjw.put_data <mujoco_warp.put_data>`.
- ``mjw.Option.ccd_iterations``: The convex collision detection algorithm has exceeded the maximum number of iterations.
  Increase the value of this setting in the XML / :ref:`mjSpec` / :ref:`mjModel`. Importantly, this change must be made
  to the :ref:`mjModel` instance that is provided to :func:`mjw.put_model <mujoco_warp.put_model>` and
  :func:`mjw.make_data <mujoco_warp.make_data>` / :func:`mjw.put_data <mujoco_warp.put_data>`.
- ``mjw.Option.contact_sensor_maxmatch``: The maximum number of contact matches for a
  :ref:`contact sensor<sensor-contact>`'s matching criteria has been exceeded. Increase the value of this MJWarp-only
  setting `m.opt.contact_sensor_maxmatch`. Alternatively, refactor the contact sensor matching criteria, for example if
  the 2 geoms of interest are known, specify ``geom1`` and ``geom2``.
- ``height field collision overflow``: The number of potential contacts generated by a height field exceeds
  :ref:`mjMAXCONPAIR <glNumeric>` and some contacts are ignored. To resolve this warning, reduce the height field
  resolution or reduce the size of the geom interacting with the height field.

Compilation
-----------

**How can compilation time be improved?**

Limit the number of unique colliders that require the general convex collision pipeline. These colliders are listed as
``_CONVEX_COLLISION_PAIRS`` in
`collision_convex.py <https://github.com/google-deepmind/mujoco_warp/blob/main/mujoco_warp/_src/collision_convex.py>`__.
Improvements to the compilation time for the pipeline are tracked in this
`GitHub issue <https://github.com/google-deepmind/mujoco_warp/issues/813>`__.

**Why are the physics not working as expected after upgrading MJWarp?**

The Warp cache may be incompatible with the current code and should be cleared as part of the debugging process. This
can be accomplished by deleting the directory ``~/.cache/warp`` or via Python

.. code-block:: python

   import warp as wp
   wp.clear_kernel_cache()

**Is it possible to compile MJWarp ahead of time instead of at runtime?**

Yes. Please see Warp's
`Ahead-of-Time Compilation Workflows <https://nvidia.github.io/warp/codegen.html#ahead-of-time-compilation-workflows>`__
documentation for details.

Differences from MuJoCo
=======================

This section notes differences between MJWarp and MuJoCo.

Warmstart
---------

If warmstarts are not :ref:`disabled <option-flag-warmstart>`, the MJWarp solver warmstart always initializes the
acceleration with ``qacc_warmstart``. In contrast, MuJoCo performs a comparison between ``qacc_smooth`` and
``qacc_warmstart`` to determine which one is utilized for the initialization.

Inertia matrix factorization
----------------------------

When using dense computation, MJWarp's factorization of the inertia matrix ``qLD`` is computed with Warp's ``L'L``
Cholesky factorization
`wp.tile_cholesky <https://nvidia.github.io/warp/language_reference/_generated/warp._src.lang.tile_cholesky.html>`__
and the result is not expected to match MuJoCo's corresponding field because a different reverse-mode ``L'DL`` routine
:ref:`mj_factorM` is utilized.

Options
-------

:class:`mjw.Option <mujoco_warp.Option>` fields correspond to their :ref:`mjOption` counterparts with the following
exceptions:

- :ref:`impratio <option-impratio>` is stored as its inverse square root ``impratio_invsqrt``.
- The constraint solver setting :ref:`tolerance <option-tolerance>` is clamped to a minimum value of ``1e-6``.
- Contact :ref:`override <option-flag-override>` parameters :ref:`o_margin <option-o_margin>`,
  :ref:`o_solref <option-o_solref>`, :ref:`o_solimp <option-o_solimp>`, and :ref:`o_friction <option-o_friction>` are
  not available.

:ref:`disableflags <option-flag>` has the following differences:

- :ref:`mjDSBL_MIDPHASE <mjtDisablebit>` is not available.
- :ref:`mjDSBL_AUTORESET <mjtDisablebit>` is not available.
- :ref:`mjDSBL_NATIVECCD <mjtDisablebit>` changes the default box-box collider from CCD to a primitive collider.
- :ref:`mjDSBL_ISLAND <mjtDisablebit>` is not currently available. Constraint island discovery is tracked in GitHub issue
  `#886 <https://github.com/google-deepmind/mujoco_warp/issues/886>`__.

:ref:`enableflags <option-flag>` has the following differences:

- :ref:`mjENBL_OVERRIDE <mjtEnablebit>` is not available.
- :ref:`mjENBL_FWDINV <mjtEnablebit>` is not available.
- Constraint island sleeping enabled via :ref:`mjENBL_ISLAND <mjtEnablebit>` is not currently available. This feature is
  tracked in GitHub issues `#886 <https://github.com/google-deepmind/mujoco_warp/issues/886>`__ and
  `#887 <https://github.com/google-deepmind/mujoco_warp/issues/887>`__.

Additional MJWarp-only options are available:

- ``is_sparse``: use sparse representation
- ``ls_parallel``: use parallel linesearch with the constraint solver
- ``ls_parallel_min_step``: minimum step size for the parallel linesearch
- ``has_fluid``: scene has non-zero wind, density, or viscosity at ``put_model`` time; if true, compute and apply fluid
  forces and torques
- ``broadphase``: type of broadphase algorithm (:class:`mjw.BroadphaseType <mujoco_warp.BroadphaseType>`)
- ``broadphase_filter``: type of filtering utilized by broadphase
  (:class:`mjw.BroadphaseFilter <mujoco_warp.BroadphaseFilter>`)
- ``graph_conditional``: use CUDA graph conditional
- ``run_collision_detection``: use collision detection routine
- ``contact_sensor_maxmatch``: maximum number of contacts for contact sensor matching criteria

Unlike MuJoCo where all :ref:`mjOption` fields are configurable at runtime, the following
:class:`mjw.Option <mujoco_warp.Option>` fields should not be directly modified:

- ``density``: may require updating ``has_fluid``
- ``viscosity``: may require updating ``has_fluid``
- ``wind``: may require updating ``has_fluid``
- ``is_sparse``: this field should not be set directly, instead modify the :ref:`mjOption` field
  :ref:`jacobian <option-jacobian>`

These fields should be updated as recommended above or by modifying an :ref:`mjModel` instance that is utilized to
construct new :class:`mjw.Model <mujoco_warp.Model>` and :class:`mjw.Data <mujoco_warp.Data>` instances with
:func:`mjw.put_model <mujoco_warp.put_model>` and :func:`mjw.make_data <mujoco_warp.make_data>` /
:func:`mjw.put_data <mujoco_warp.put_data>`. Directly modifying these fields at runtime may lead to unintended side
effects.

.. admonition:: Graph capture
  :class: note

  A new :ref:`graph capture <mjwGC>` may be necessary after modifying an :class:`mjw.Option <mujoco_warp.Option>` field
  in order for the updated setting to take effect.
