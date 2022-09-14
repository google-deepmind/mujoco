=========
Changelog
=========


Upcoming version (not yet released)
-----------------------------------

General
^^^^^^^

- Updates to humanoid model:

   - Added two keyframes (stand-on-one-leg and squat).
   - Increased maximum hip flexion angle.
   - Added hamstring tendons which couple the hip and knee at high hip flexion angles.
   - General cosmetic improvements, including improved use of defaults and better naming scheme.

- Added :ref:`mju_boxQP` and allocation function :ref:`mju_boxQPmalloc` for solving the box-constrained
  Quadratic Program:

   .. math::

      x^* = \text{argmin} \; \tfrac{1}{2} x^T H x + x^T g \quad \text{s.t.} \quad l \le x \le u

   - The algorithm, introduced in `Tassa et al. 2014 <https://doi.org/10.1109/ICRA.2014.6907001>`_,
     converges after 2-5 Cholesky factorisations, independent of problem size.
- Added :ref:`mju_mulVecMatVec` to multiply a square matrix :math:`M` with vectors :math:`x` and :math:`y` on both
  sides. The function returns :math:`x^TMy`.

Version 2.2.2 (September 7, 2022)
---------------------------------

General
^^^^^^^

.. youtube:: BcHZ5BFeTmU
   :align: right
   :height: 150px

1. Added :ref:`adhesion actuators<adhesion>` mimicking vacuum grippers and adhesive biomechanical appendages.
#. Added related `example model <https://github.com/deepmind/mujoco/tree/main/model/adhesion>`_ and video:
#. Added :ref:`mj_jacSubtreeCom` for computing the translational Jacobian of the center-of-mass of a subtree.
#. Added :at:`torquescale` and :at:`anchor` attributes to :el:`weld` constraints. :at:`torquescale` sets the
   torque-to-force ratio exerted by the constraint, :at:`anchor` sets the point at which the weld wrench is
   applied. See :ref:`weld <equality-weld>` for more details.
#. Increased ``mjNEQDATA``, the row length of equality constraint parameters in ``mjModel.eq_data``, from 7 to 11.
#. Added visualisation of anchor points for both :el:`connect` and :el:`weld` constraints (activated by the 'N' key in
   ``simulate``).
#. Added `weld.xml <https://github.com/deepmind/mujoco/tree/main/test/engine/testdata/weld.xml>`_ showing different
   uses of new weld attributes.

   .. youtube:: s-0JHanqV1A
      :align: right
      :height: 150px

#. Cartesian 6D end-effector control is now possible by adding a reference site to actuators with :at:`site`
   transmission. See description of new :at:`refsite` attribute in the :ref:`actuator<general>` documentation and
   `refsite.xml <https://github.com/deepmind/mujoco/tree/main/test/engine/testdata/refsite.xml>`_ example model.
#. Added :at:`autolimits` compiler option. If ``true``, joint and tendon :at:`limited` attributes and actuator
   :at:`ctrllimited`, :at:`forcelimited` and :at:`actlimited` attributes will automatically be set to ``true`` if the
   corresponding range *is defined* and ``false`` otherwise.

   If ``autolimits="false"`` (the default) models where a :at:`range` attribute is specified without the :at:`limited`
   attribute will fail to compile. A future release will change the default of :at:`autolimits` to ``true``, and this
   compilation error allows users to catch this future change of behavior.

   .. attention::
     This is a breaking change. In models where a range was defined but :at:`limited` was unspecified, explicitly set
     limited to ``false`` or remove the range to maintain the current behavior of your model.

#. Added moment of inertia computation for all well-formed meshes. This option is activated by setting the compiler
   flag :at:`exactmeshinertia` to ``true`` (defaults to ``false``). This default may change in the future.
#. Added parameter :at:`shellinertia` to :at:`geom`, for locating the inferred inertia on the boundary (shell).
   Currently only meshes are supported.
#. For meshes from which volumetric inertia is inferred, raise error if the orientation of mesh faces is not consistent.
   If this occurs, fix the mesh in e.g., MeshLab or Blender.

   .. youtube:: I2q7D0Vda-A
      :align: right
      :height: 150px

#. Added catenary visualisation for hanging tendons. The model seen in the video can be found
   `here <https://github.com/deepmind/mujoco/tree/main/test/engine/testdata/catenary.xml>`_.
#. Added ``azimuth`` and ``elevation`` attributes to :ref:`visual/global<global>`, defining the initial orientation of
   the free camera at model load time.
#. Added ``mjv_defaultFreeCamera`` which sets the default free camera, respecting the above attributes.
#. ``simulate`` now supports taking a screenshot via a button in the File section or via ``Ctrl-P``.
#. Improvements to time synchronisation in `simulate`, in particular report actual real-time factor if different from
   requested factor (if e.g., the timestep is so small that simulation cannot keep up with real-time).
#. Added a disable flag for sensors.
#. :ref:`mju_mulQuat` and :ref:`mju_mulQuatAxis` support in place computation. For example
   |br| ``mju_mulQuat(a, a, b);`` sets the quaternion ``a`` equal to the product of ``a`` and ``b``.
#. Added sensor matrices to ``mjd_transitionFD`` (note this is an API change).

Deleted/deprecated features
^^^^^^^^^^^^^^^^^^^^^^^^^^^

21. Removed ``distance`` constraints.

Bug fixes
^^^^^^^^^

22. Fixed rendering of some transparent geoms in reflection.
#.  Fixed ``intvelocity`` defaults parsing.


Version 2.2.1 (July 18, 2022)
-----------------------------

General
^^^^^^^

1. Added ``mjd_transitionFD`` to compute efficient finite difference approximations of the state-transition and
   control-transition matrices, :ref:`see here<derivatives>` for more details.
#. Added derivatives for the ellipsoid fluid model.
#. Added ``ctrl`` attribute to :ref:`keyframes<keyframe>`.
#. Added ``clock`` sensor which :ref:`measures time<sensor-clock>`.
#. Added visualisation groups to skins.
#. Added actuator visualisation for ``free`` and ``ball`` joints and for actuators with ``site`` transmission.
#. Added visualisation for actuator activations.
#. Added ``<intvelocity>`` actuator shortcut for "integrated velocity" actuators, documented :ref:`here <intvelocity>`.
#. Added ``<damper>`` actuator shortcut for active-damping actuators, documented :ref:`here <damper>`.
#. ``mju_rotVecMat`` and ``mju_rotVecMatT`` now support in-place multiplication.
#. ``mjData.ctrl`` values are no longer clamped in-place, remain untouched by the engine.
#. Arrays in mjData's buffer now align to 64-byte boundaries rather than 8-byte.
#. Added memory poisoning when building with Address Sanitizer (ASAN) and Memory Sanitizer (MSAN). This allows ASAN to
   detect reads and writes to regions in ``mjModel.buffer`` and ``mjData.buffer`` that do not lie within an array, and
   for MSAN to detect reads from uninitialised fields in ``mjData`` following ``mj_resetData``.
#. Added a `slider-crank example model <https://github.com/deepmind/mujoco/tree/main/model/slider_crank>`_.

Bug fixes
^^^^^^^^^

15. :ref:`Activation clamping <CActRange>` was not being applied in the :ref:`implicit integrator<geIntegration>`.
#. Stricter parsing of orientation specifiers. Before this change, a specification that included both ``quat`` and an
   :ref:`alternative specifier<COrientation>` e.g., ``<geom ... quat=".1 .2 .3 .4" euler="10 20 30">``, would lead to
   the ``quat`` being ignored and only ``euler`` being used. After this change a parse error will be thrown.
#. Stricter parsing of XML attributes. Before this change an erroneous XML snippet like ``<geom size="1/2 3 4">`` would
   have been parsed as ``size="1 0 0"`` and no error would have been thrown. Now throws an error.
#. Trying to load a ``NaN`` via XML like ``<geom size="1 NaN 4">``, while allowed for debugging purposes, will now print
   a warning.
#. Fixed null pointer dereference in ``mj_loadModel``.
#. Fixed memory leaks when loading an invalid model from MJB.
#. Integer overflows are now avoided when computing ``mjModel`` buffer sizes.
#. Added missing warning string for ``mjWARN_BADCTRL``.

Packaging
^^^^^^^^^

23. Changed MacOS packaging so that the copy of ``mujoco.framework`` embedded in ``MuJoCo.app`` can be used to build
    applications externally.


Version 2.2.0 (May 23, 2022)
----------------------------

Open Sourcing
^^^^^^^^^^^^^

1. MuJoCo is now fully open-source software. Newly available top level directories are:

   a. ``src/``: All source files. Subdirectories correspond to the modules described in the Programming chapter
   :ref:`introduction<inIntro>`:

   - ``src/engine/``: Core engine.
   - ``src/xml/``: XML parser.
   - ``src/user/``: Model compiler.
   - ``src/visualize/``: Abstract visualizer.
   - ``src/ui/``: UI framework.

   b. ``test/``: Tests and corresponding asset files.

   c. ``dist/``: Files related to packaging and binary distribution.

#. Added `contributor's guide <https://github.com/deepmind/mujoco/blob/main/CONTRIBUTING.md>`_ and
   `style guide <https://github.com/deepmind/mujoco/blob/main/STYLEGUIDE.md>`_.

General
^^^^^^^

3. Added analytic derivatives of smooth (unconstrained) dynamics forces, with respect to velocities:

   - Centripetal and Coriolis forces computed by the Recursive Newton-Euler algorithm.
   - Damping and fluid-drag passive forces.
   - Actuation forces.

#. Added ``implicit`` integrator. Using the analytic derivatives above, a new implicit-in-velocity integrator was added.
   This integrator lies between the Euler and Runge Kutta integrators in terms of both stability and computational
   cost. It is most useful for models which use fluid drag (e.g. for flying or swimming) and for models which use
   :ref:`velocity actuators<velocity>`. For more details, see the :ref:`Numerical Integration<geIntegration>` section.

#. Added :at:`actlimited` and :at:`actrange` attributes to :ref:`general actuators<general>`, for clamping actuator
   internal states (activations). This clamping is useful for integrated-velocity actuators, see the :ref:`Activation
   clamping <CActRange>` section for details.

#. ``mjData`` fields ``qfrc_unc`` (unconstrained forces) and ``qacc_unc`` (unconstrained accelerations) were renamed
   ``qfrc_smooth`` and ``qacc_smooth``, respectively. While "unconstrained" is precise, "smooth" is more intelligible
   than "unc".

#. Public headers have been moved from ``/include`` to ``/include/mujoco/``, in line with the directory layout common in
   other open source projects. Developers are encouraged to include MuJoCo public headers in their own codebase via
   ``#include <mujoco/filename.h>``.

#. The default shadow resolution specified by the :ref:`shadowsize<quality>` attribute was increased from 1024 to 4096.

#. Saved XMLs now use 2-space indents.

Bug fixes
^^^^^^^^^

10. Antialiasing was disabled for segmentation rendering. Before this change, if the :ref:`offsamples<quality>`
    attribute was greater than 0 (the default value is 4), pixels that overlapped with multiple geoms would receive
    averaged segmentation IDs, leading to incorrect or non-existent IDs. After this change :at:`offsamples` is ignored
    during segmentation rendering.

#.  The value of the enable flag for the experimental multiCCD feature was made sequential with other enable flags.
    Sequentiality is assumed in the ``simulate`` UI and elsewhere.

#.  Fix issue of duplicated meshes when saving models with OBJ meshes using mj_saveLastXML.


Version 2.1.5 (Apr. 13, 2022)
-----------------------------

General
^^^^^^^

1. Added an experimental feature: multi-contact convex collision detection, activated by an enable flag. See full
   description :ref:`here <option-flag>`.

Bug fixes
^^^^^^^^^

2. GLAD initialization logic on Linux now calls ``dlopen`` to load a GL platform dynamic library if a
   ``*GetProcAddress`` function is not already present in the process' global symbol table. In particular, processes
   that use GLFW to set up a rendering context that are not explicitly linked against ``libGLX.so`` (this applies to the
   Python interpreter, for example) will now work correctly rather than fail with a ``gladLoadGL`` error when
   ``mjr_makeContext`` is called.

#. In the Python bindings, named indexers for scalar fields (e.g. the ``ctrl`` field for actuators) now return a NumPy
   array of shape ``(1,)`` rather than ``()``. This allows values to be assigned to these fields more straightforwardly.

Version 2.1.4 (Apr. 4, 2022)
----------------------------

General
^^^^^^^

1. MuJoCo now uses GLAD to manage OpenGL API access instead of GLEW. On Linux, there is no longer a need to link against
   different GL wrangling libraries depending on whether GLX, EGL, or OSMesa is being used. Instead, users can simply
   use GLX, EGL, or OSMesa to create a GL context and ``mjr_makeContext`` will detect which one is being used.

#. Added visualisation for contact frames. This is useful when writing or modifying collision functions, when the actual
   direction of the x and y axes of a contact can be important.

Binary build
^^^^^^^^^^^^

3. The ``_nogl`` dynamic library is no longer provided on Linux and Windows. The switch to GLAD allows us to resolve
   OpenGL symbols when ``mjr_makeContext`` is called rather than when the library is loaded. As a result, the MuJoCo
   library no longer has an explicit dynamic dependency on OpenGL, and can be used on system where OpenGL is not
   present.

Simulate
^^^^^^^^

4. Fixed a bug in simulate where pressing '[' or ']' when a model is not loaded causes a crash.

#. Contact frame visualisation was added to the Simulate GUI.

#. Renamed "set key", "reset to key" to "save key" and "load key", respectively.

#. Changed bindings of F6 and F7 from the not very useful "vertical sync" and "busy wait" to the more useful cycling of
   frames and labels.

Bug fixes
^^^^^^^^^

8. ``mj_resetData`` zeroes out the ``solver_nnz`` field.

#. Removed a special branch in ``mju_quat2mat`` for unit quaternions. Previously, ``mju_quat2mat`` skipped all
   computation if the real part of the quaternion equals 1.0. For very small angles (e.g. when finite differencing), the
   cosine can evaluate to exactly 1.0 at double precision while the sine is still nonzero.


Version 2.1.3 (Mar. 23, 2022)
-----------------------------

General
^^^^^^^

1. ``simulate`` now supports cycling through cameras (with the ``[`` and ``]`` keys).
#. ``mjVIS_STATIC`` toggles all static bodies, not just direct children of the world.

Python bindings
^^^^^^^^^^^^^^^

3. Added a ``free()`` method to ``MjrContext``.
#. Enums now support arithmetic and bitwise operations with numbers.

Bug fixes
^^^^^^^^^

5. Fixed rendering bug for planes, introduced in 2.1.2. This broke maze environments in
   `dm_control <https://github.com/deepmind/dm_control>`_.


Version 2.1.2 (Mar. 15, 2022)
-----------------------------

New modules
^^^^^^^^^^^

1. Added new :doc:`Python bindings<python>`, which can be installed via ``pip install mujoco``,
   and imported as ``import mujoco``.
#. Added new :doc:`Unity plug-in<unity>`.
#. Added a new ``introspect`` module, which provides reflection-like capability for MuJoCo's public API, currently
   describing functions and enums. While implemented in Python, this module is expected to be generally useful for
   automatic code generation targeting multiple languages. (This is not shipped as part of the ``mujoco`` Python
   bindings package.)

API changes
^^^^^^^^^^^

4. Moved definition of ``mjtNum`` floating point type into a new header
   `mjtnum.h <https://github.com/deepmind/mujoco/blob/3577e2cf8bf841475b489aefff52276a39f24d51/include/mjtnum.h>`_.
#. Renamed header `mujoco_export.h` to :ref:`mjexport.h<inHeader>`.
#. Added ``mj_printFormattedData``, which accepts a format string for floating point numbers, for example to increase
   precision.

General
^^^^^^^

7. MuJoCo can load `OBJ <https://en.wikipedia.org/wiki/Wavefront_.obj_file>`_ mesh files.

   a. Meshes containing polygons with more than 4 vertices are not supported.
   #. In OBJ files containing multiple object groups, any groups after the first one will be ignored.
   #. Added (post-release, not included in the 2.1.2 archive) textured
      `mug <https://github.com/deepmind/mujoco/blob/main/model/mug/mug.xml>`_ example model:

      .. image:: images/changelog/mug.png
         :width: 300px


#. Added optional frame-of-reference specification to :ref:`framepos<sensor-framepos>`,
   :ref:`framequat<sensor-framequat>`, :ref:`framexaxis<sensor-framexaxis>`, :ref:`frameyaxis<sensor-frameyaxis>`,
   :ref:`framezaxis<sensor-framezaxis>`, :ref:`framelinvel<sensor-framelinvel>`, and
   :ref:`frameangvel<sensor-frameangvel>` sensors. The frame-of-reference is specified by new :at:`reftype` and
   :at:`refname` attributes.

#. Sizes of :ref:`user parameters <CUser>` are now automatically inferred.

   a. Declarations of user parameters in the top-level :ref:`size <size>` clause (e.g. :at:`nuser_body`,
      :at:`nuser_jnt`, etc.) now accept a value of -1, which is the default. This will automatically set the value to
      the length of the maximum associated :at:`user` attribute defined in the model.
   #. Setting a value smaller than -1 will lead to a compiler error (previously a segfault).
   #. Setting a value to a length smaller than some :at:`user` attribute defined in the model will lead to an error
      (previously additional values were ignored).

#. Increased the maximum number of lights in an :ref:`mjvScene` from 8 to 100.

#. Saved XML files only contain explicit :ref:`inertial <inertial>` elements if the original XML included them. Inertias
   that were automatically inferred by the compiler's :ref:`inertiafromgeom <compiler>` mechanism remain unspecified.

#. User-selected geoms are always rendered as opaque. This is useful in interactive visualizers.

#. Static geoms now respect their :ref:`geom group<geom>` for visualisation. Until this change rendering of static geoms
   could only be toggled using the :ref:`mjVIS_STATIC<mjtVisFlag>` visualisation flag . After this change, both the geom
   group and the visualisation flag need to be enabled for the geom to be rendered.

#. Pointer parameters in function declarations in :ref:`mujoco.h<inHeader>` that are supposed to represent fixed-length
   arrays are now spelled as arrays with extents, e.g. ``mjtNum quat[4]`` rather than ``mjtNum* quat``. From the
   perspective of C and C++, this is a non-change since array types in function signatures decay to pointer types.
   However, it allows autogenerated code to be aware of expected input shapes.

#. Experimental stateless fluid interaction model. As described :ref:`here <gePassive>`, fluid forces use sizes computed
   from body inertia. While sometimes convenient, this is very rarely a good approximation. In the new model forces act
   on geoms, rather than bodies, and have a several user-settable parameters. The model is activated by setting a new
   attribute: ``<geom fluidshape="ellipsoid"/>``. The parameters are described succinctly :ref:`here<geom>`, but we
   leave a full description or the model and its parameters to when this feature leaves experimental status.

Bug fixes
^^^^^^^^^

16. ``mj_loadXML`` and ``mj_saveLastXML`` are now locale-independent. The Unity plugin should now work correctly for
    users whose system locales use commas as decimal separators.
#.  XML assets in VFS no longer need to end in a null character. Instead, the file size is determined by the size
    parameter of the corresponding VFS entry.
#.  Fix a vertex buffer object memory leak in ``mjrContext`` when skins are used.
#.  Camera quaternions are now normalized during XML compilation.

Binary build
^^^^^^^^^^^^

20. Windows binaries are now built with Clang.

Version 2.1.1 (Dec. 16, 2021)
-----------------------------

API changes
^^^^^^^^^^^

1. Added ``mj_printFormattedModel``, which accepts a format string for floating point numbers, for example to increase
   precision.
#. Added ``mj_versionString``, which returns human-readable string that represents the version of the MuJoCo binary.
#. Converted leading underscores to trailing underscores in private instances of API struct definitions, to conform to
   reserved identifier directive, see
   `C standard: Section 7.1.3 <http://www.open-std.org/jtc1/sc22/wg14/www/docs/n1570.pdf>`__.

   .. attention::
      This is a minor breaking change. Code which references private instances will break. To fix, replace leading
      underscores with trailing underscores, e.g. ``_mjModel`` |rarr| ``mjModel_``.

General
^^^^^^^

4. Safer string handling: replaced ``strcat``, ``strcpy``, and ``sprintf`` with ``strncat``, ``strncpy``, and
   ``snprintf`` respectively.
#. Changed indentation from 4 spaces to 2 spaces, K&R bracing style, added braces to one-line conditionals.

Bug Fixes
^^^^^^^^^

6. Fixed reading from uninitialized memory in PGS solver.
#. Computed capsule inertias are now exact. Until this change, capsule masses and inertias computed by the
   :ref:`compiler <compiler>`'s :at:`inertiafromgeom` mechanism were approximated by a cylinder, formed by the
   capsule's cylindrical middle section, extended on both ends by half the capsule radius. Capsule inertias are now
   computed with the `Parallel Axis theorem <https://en.wikipedia.org/wiki/Parallel_axis_theorem>`_, applied to the two
   hemispherical end-caps.

   .. attention::
      This is a minor breaking change. Simulation of a model with automatically-computed capsule inertias will be
      numerically different, leading to, for example, breakage of golden-value tests.
#. Fixed bug related to :ref:`force <sensor-force>` and :ref:`torque <sensor-torque>` sensors. Until this change, forces
   and torques reported by F/T sensors ignored out-of-tree constraint wrenches except those produced by contacts. Force
   and torque sensors now correctly take into account the effects of :ref:`connect <equality-connect>` and
   :ref:`weld <equality-weld>` constraints.

   .. note::
      Forces generated by :ref:`spatial tendons <spatial>` which are outside the kinematic tree (i.e., between bodies
      which have no ancestral relationship) are still not taken into account by force and torque sensors. This remains a
      future work item.

Code samples
^^^^^^^^^^^^

9. ``testspeed``: Added injection of pseudo-random control noise, turned on by default. This is to avoid settling into
   some fixed contact configuration and providing an unrealistic timing measure.
#. ``simulate``:

   a. Added slower-than-real-time functionality, which is controlled via the '+' and '-' keys.
   #. Added sliders for injecting Brownian noise into the controls.
   #. Added "Print Camera" button to print an MJCF clause with the pose of the current camera.
   #. The camera pose is not reset when reloading the same model file.

Updated dependencies
^^^^^^^^^^^^^^^^^^^^

11. ``TinyXML`` was replaced with ``TinyXML2`` 6.2.0.
#. ``qhull`` was upgraded to version 8.0.2.
#. ``libCCD`` was upgraded to version 1.4.
#. On Linux, ``libstdc++`` was replaced with ``libc++``.

Binary build
^^^^^^^^^^^^

15. MacOS packaging. We now ship Universal binaries that natively support both Apple Silicon and Intel CPUs.

    a. MuJoCo library is now packaged as a
       `Framework Bundle <https://developer.apple.com/library/archive/documentation/MacOSX/Conceptual/BPFrameworks/Concepts/FrameworkAnatomy.html>`_,
       allowing it to be incorporated more easily into Xcode projects (including Swift projects). Developers are
       encouraged to compile and link against MuJoCo using the ``-framework mujoco`` flag, however all header files and
       the ``libmujoco.2.1.1.dylib`` library can still be directly accessed inside the framework.
    #. Sample applications are now packaged into an Application Bundle called ``MuJoCo.app``. When launched via GUI,
       the bundle launches the ``simulate`` executable. Other precompiled sample programs are shipped inside that bundle
       (in ``MuJoCo.app/Contents/MacOS``) and can be launched via command line.
    #. Binaries are now signed and the disk image is notarized.

#. Windows binaries and libraries are now signed.
#. Link-time optimization is enabled on Linux and macOS, leading to an average of \~20% speedup when benchmarked on
   three test models (``cloth.xml``, ``humanoid.xml``, and ``humanoid100.xml``).
#. Linux binaries are now built with LLVM/Clang instead of GCC.
#. An AArch64 (aka ARM64) Linux build is also provided.
#. Private symbols are no longer stripped from shared libraries on Linux and MacOS.

Sample models
^^^^^^^^^^^^^
21. Clean-up of the ``model/`` directory.

    a. Rearranged into subdirectories which include all dependencies.
    #. Added descriptions in XML comments, cleaned up XMLs.
    #. Deleted some composite models: ``grid1``, ``grid1pin``, ``grid2``, ``softcylinder``, ``softellipsoid``.

#. Added descriptive animations in ``docs/images/models/`` :

|humanoid|   |particle|


Version 2.1.0 (Oct. 18, 2021)
-----------------------------

New features
^^^^^^^^^^^^

1. Keyframes now have ``mocap_pos`` and ``mocap_quat`` fields (mpos and quat attributes in the XML) allowing mocap
   poses to be stored in keyframes.
2. New utility functions: ``mju_insertionSortInt`` (integer insertion sort) and ``mju_sigmoid`` (constructing a
   sigmoid from two half-quadratics).

General
^^^^^^^

3. The preallocated sizes in the virtual file system (VFS) increased to 2000 and 1000, to allow for larger projects.
#. The C structs in the ``mjuiItem`` union are now named, for compatibility.
#. Fixed: ``mjcb_contactfilter`` type is ``mjfConFilt`` (was ``mjfGeneric``).
#. Fixed: The array of sensors in ``mjCModel`` was not cleared.
#. Cleaned up cross-platform code (internal changes, not visible via the API).
#. Fixed a bug in parsing of XML ``texcoord`` data (related to number of vertices).
#. Fixed a bug in `simulate.cc <https://github.com/deepmind/mujoco/blob/main/sample/simulate.cc>`_ related to ``nkey``
   (the number of keyframes).
#. Accelerated collision detection in the presence of large numbers of non-colliding geoms (with ``contype==0 and
   conaffinity==0``).

UI
^^

11. Figure selection type changed from ``int`` to ``float``.
#. Figures now show data coordinates, when selection and highlight are enabled.
#. Changed ``mjMAXUIMULTI`` to 35, ``mjMAXUITEXT`` to 300, ``mjMAXUIRECT`` to 25.
#. Added collapsable sub-sections, implemented as separators with state: ``mjSEPCLOSED`` collapsed, ``mjSEPCLOSED+1``
   expanded.
#. Added ``mjITEM_RADIOLINE`` item type.
#. Added function ``mjui_addToSection`` to simplify UI section construction.
#. Added subplot titles to ``mjvFigure``.

Rendering
^^^^^^^^^

18. ``render_gl2`` guards against non-finite floating point data in the axis range computation.
#. ``render_gl2`` draws lines from back to front for better visibility.
#. Added function ``mjr_label`` (for text labels).
#. ``mjr_render`` exits immediately if ``ngeom==0``, to avoid errors from uninitialized scenes (e.g. ``frustrum==0``).
#. Added scissor box in ``mjr_render``, so we don't clear the entire window at every frame.


License manager
^^^^^^^^^^^^^^^

23. Removed the entire license manager. The functions ``mj_activate`` and ``mj_deactivate`` are still there for
    backward compatibility, but now they do nothing and it is no longer necessary to call them.
#. Removed the remote license certificate functions ``mj_certXXX``.

Earlier versions
----------------

For changelogs of earlier versions please see `roboti.us <https://www.roboti.us/download.html>`_.

.. |humanoid| image:: images/models/humanoid.gif
   :width: 270px
.. |particle| image:: images/models/particle.gif
   :width: 270px
