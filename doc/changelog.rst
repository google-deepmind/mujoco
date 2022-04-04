=========
Changelog
=========

Version 2.1.4 (Apr. 4, 2022)
-----------------------------

General
^^^^^^^

1. MuJoCo now uses GLAD to manage OpenGL API access instead of GLEW. On Linux, there is no longer a need to link against
   different GL wrangling libraries depending on whether GLX, EGL, or OSMesa is being used. Instead, users can simply
   use GLX, EGL, or OSMesa to create a GL context and ``mjr_makeContext`` will detect which one is being used.

#. Add visualisation for contact frames. This is useful when writing or modifying collision functions, when the actual
   direction of the x and y axes of a contact can be important.

Binary build
^^^^^^^^^^^^

3. The ``_nogl`` dynamic library is no longer provided on Linux and Windows. The switch to GLAD allows us to resolve
   OpenGL symbols when ``mjr_makeContext`` is called rather than when the library is loaded. As a result, the MuJoCo
   library no longer has an explicit dynamic dependency on OpenGL, and can be used on system where OpenGL is not
   present.

Simulate
^^^^^^^^

4. Fix a bug in simulate where pressing '[' or ']' when a model is not loaded causes a crash.

#. Contact frame visualisation is added to the Simulate GUI.

#. Rename "set key", "reset to key" to "save key" and "load key", respectively.

#. Change bindings of F6 and F7 from the not very useful "vertical sync" and "busy wait" to the more useful cycling of
   frames and labels.

Bug fixes
^^^^^^^^^

8. ``mj_resetData`` zeroes out the ``solver_nnz`` field.

#. Remove a special branch in ``mju_quat2mat`` for unit quaternions. Previously, ``mju_quat2mat`` skipped all
   computation if the real part of the quaternion equals 1.0. For very small angles (e.g. when finite differencing), the
   cosine can evaluate to exactly 1.0 at double precision while the sine is still nonzero.


Version 2.1.3 (Mar. 23, 2022)
-----------------------------

General
^^^^^^^

1. ``simulate`` now support cycling through cameras (with ``[`` and ``]`` keys).
#. ``mjVIS_STATIC`` toggles all static bodies, not just direct children of the world.

Python bindings
^^^^^^^^^^^^^^^

3. Add a ``free()`` method to ``MjrContext``.
#. Enums now support arithmetic and bitwise operations with numbers.

Bug fixes
^^^^^^^^^

5. Fix rendering bug for planes, introduced in 2.1.2. This broke maze environments in
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
   `mjtnum.h <https://github.com/deepmind/mujoco/blob/main/include/mjtnum.h>`_.
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
    backward compabitibily, but now they do nothing and it is no longer necessary to call them.
#. Removed the remote license certificate functions ``mj_certXXX``.

Earlier versions
----------------

For changelogs of earlier versions please see `roboti.us <https://www.roboti.us/download.html>`_.

.. |humanoid| image:: images/models/humanoid.gif
   :width: 270px
.. |particle| image:: images/models/particle.gif
   :width: 270px
