=====
Types
=====

MuJoCo defines a large number of types:

- Two :ref:`primitive types<tyPrimitive>`.
- C enums used to define categorical values. These can be classified according to their use in:

  - :ref:`mjModel<tyModelEnums>`.
  - :ref:`mjData<tyDataEnums>`.
  - Abstract :ref:`visualization<tyVisEnums>`.
  - The :ref:`openGL renderer<tyRenderEnums>`.
  - The :ref:`mjUI<tyUIEnums>` user interface package.

  Note that the API does not use these enum types directly. Instead it uses ints, and the documentation/comments state
  that certain ints correspond to certain enum types. This is because we want the API to be compiler-independent, and
  the C standard does not dictate how many bytes must be used to represent an enum type. Nevertheless, for improved
  readiblity, we recommend using these types when calling API functions which take them as arguments.

- C struct types. These can be classified as:

  - :ref:`Main struct types<tyMainStructure>`. These are :ref:`mjModel`, :ref:`mjOption` and :ref:`mjData`.
  - :ref:`Auxillary struct types<tyAuxStructure>`, also used by the engine.
  - Structs for collecting :ref:`simulation statistics<tyStatStructure>`.
  - Structs for :ref:`abstract visualization<tyVisStructure>`.
  - Structs used by the :ref:`openGL renderer<tyRenderStructure>`.
  - Structs used by the :ref:`UI framework<tyUIStructure>`.

- Several :ref:`tyFunction` for user-defined callbacks.

.. _tyPrimitive:

Primitive types
^^^^^^^^^^^^^^^

The two types below are defined in `mjtnum.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjtnum.h>`_.

.. _mjtNum:

mjtNum
~~~~~~

This is the floating-point type used throughout the simulator. If the symbol ``mjUSEDOUBLE`` is defined in
``mjmodel.h``, this type is defined as ``double``, otherwise it is defined as ``float``. Currently only the
double-precision version of MuJoCo is distributed, although the entire code base works with single-precision as well.
We may release the single-precision version in the future for efficiency reasons, but the double-precision version
will always be available. Thus it is safe to write user code assuming double precision. However, our preference is to
write code that works with either single or double precision. To this end we provide math utility functions that are
always defined with the correct floating-point type.

Note that changing ``mjUSEDOUBLE`` in ``mjtnum.h`` will not change how the library was compiled, and instead will
result in numerous link errors. In general, the header files distributed with precompiled MuJoCo should never be
changed by the user.

.. code-block:: C

   #ifdef mjUSEDOUBLE
       typedef double mjtNum;
   #else
       typedef float mjtNum;
   #endif


.. _mjtByte:

mjtByte
~~~~~~~

Byte type used to represent boolean variables.

.. code-block:: C

   typedef unsigned char mjtByte;



.. _tyModelEnums:

Model enums
^^^^^^^^^^^

The enums below are defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_.


.. _mjtDisableBit:

mjtDisableBit
~~~~~~~~~~~~~

Constants which are powers of 2. They are used as bitmasks for the field ``disableflags`` of :ref:`mjOption`.
At runtime this field is ``m->opt.disableflags``. The number of these constants is given by ``mjNDISABLE`` which is
also the length of the global string array :ref:`mjDISABLESTRING` with text descriptions of these flags.

.. mujoco-include:: mjtDisableBit


.. _mjtEnableBit:

mjtEnableBit
~~~~~~~~~~~~

Constants which are powers of 2. They are used as bitmasks for the field ``enableflags`` of :ref:`mjOption`.
At runtime this field is ``m->opt.enableflags``. The number of these constants is given by ``mjNENABLE`` which is also
the length of the global string array :ref:`mjENABLESTRING` with text descriptions of these flags.

.. mujoco-include:: mjtEnableBit


.. _mjtJoint:

mjtJoint
~~~~~~~~

Primitive joint types. These values are used in ``m->jnt_type``. The numbers in the comments indicate how many
positional coordinates each joint type has. Note that ball joints and rotational components of free joints are
represented as unit quaternions - which have 4 positional coordinates but 3 degrees of freedom each.

.. mujoco-include:: mjtJoint


.. _mjtGeom:

mjtGeom
~~~~~~~

Geometric types supported by MuJoCo. The first group are "official" geom types that can be used in the model. The
second group are geom types that cannot be used in the model but are used by the visualizer to add decorative
elements. These values are used in ``m->geom_type`` and ``m->site_type``.

.. mujoco-include:: mjtGeom


.. _mjtCamLight:

mjtCamLight
~~~~~~~~~~~

Dynamic modes for cameras and lights, specifying how the camera/light position and orientation are computed. These
values are used in ``m->cam_mode`` and ``m->light_mode``.

.. mujoco-include:: mjtCamLight


.. _mjtTexture:

mjtTexture
~~~~~~~~~~

Texture types, specifying how the texture will be mapped. These values are used in ``m->tex_type``.

.. mujoco-include:: mjtTexture


.. _mjtIntegrator:

mjtIntegrator
~~~~~~~~~~~~~

Numerical integrator types. These values are used in ``m->opt.integrator``.

.. mujoco-include:: mjtIntegrator


.. _mjtCollision:

mjtCollision
~~~~~~~~~~~~

Collision modes specifying how candidate geom pairs are generated for near-phase collision checking. These values are
used in ``m->opt.collision``.

.. mujoco-include:: mjtCollision


.. _mjtCone:

mjtCone
~~~~~~~

Available friction cone types. These values are used in ``m->opt.cone``.

.. mujoco-include:: mjtCone

.. _mjtJacobian:

mjtJacobian
~~~~~~~~~~~

Available Jacobian types. These values are used in ``m->opt.jacobian``.

.. mujoco-include:: mjtJacobian


.. _mjtSolver:

mjtSolver
~~~~~~~~~

Available constraint solver algorithms. These values are used in ``m->opt.solver``.

.. mujoco-include:: mjtSolver


.. _mjtEq:

mjtEq
~~~~~

Equality constraint types. These values are used in ``m->eq_type``.

.. mujoco-include:: mjtEq


.. _mjtWrap:

mjtWrap
~~~~~~~

Tendon wrapping object types. These values are used in ``m->wrap_type``.

.. mujoco-include:: mjtWrap


.. _mjtTrn:

mjtTrn
~~~~~~

Actuator transmission types. These values are used in ``m->actuator_trntype``.

.. mujoco-include:: mjtTrn


.. _mjtDyn:

mjtDyn
~~~~~~

Actuator dynamics types. These values are used in ``m->actuator_dyntype``.

.. mujoco-include:: mjtDyn


.. _mjtGain:

mjtGain
~~~~~~~

Actuator gain types. These values are used in ``m->actuator_gaintype``.

.. mujoco-include:: mjtGain


.. _mjtBias:

mjtBias
~~~~~~~

Actuator bias types. These values are used in ``m->actuator_biastype``.

.. mujoco-include:: mjtBias


.. _mjtObj:

mjtObj
~~~~~~

MuJoCo object types. These are used, for example, in the support functions :ref:`mj_name2id` and
:ref:`mj_id2name` to convert between object names and integer ids.

.. mujoco-include:: mjtObj


.. _mjtConstraint:

mjtConstraint
~~~~~~~~~~~~~

Constraint types. These values are not used in mjModel, but are used in the mjData field ``d->efc_type`` when the list
of active constraints is constructed at each simulation time step.

.. mujoco-include:: mjtConstraint

.. _mjtConstraintState:

mjtConstraintState
~~~~~~~~~~~~~~~~~~

These values are used by the solver internally to keep track of the constraint states.

.. mujoco-include:: mjtConstraintState


.. _mjtSensor:

mjtSensor
~~~~~~~~~

Sensor types. These values are used in ``m->sensor_type``.

.. mujoco-include:: mjtSensor


.. _mjtStage:

mjtStage
~~~~~~~~

These are the compute stages for the skipstage parameters of :ref:`mj_forwardSkip` and
:ref:`mj_inverseSkip`.

.. mujoco-include:: mjtStage


.. _mjtDataType:

mjtDataType
~~~~~~~~~~~

These are the possible sensor data types, used in ``mjData.sensor_datatype``.

.. mujoco-include:: mjtDataType



.. _tyDataEnums:

Data enums
^^^^^^^^^^

The enums below are defined in `mjmdata.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmdata.h>`_.


.. _mjtWarning:

mjtWarning
~~~~~~~~~~

Warning types. The number of warning types is given by ``mjNWARNING`` which is also the length of the array
``mjData.warning``.

.. mujoco-include:: mjtWarning


.. _mjtTimer:

mjtTimer
~~~~~~~~

Timer types. The number of timer types is given by ``mjNTIMER`` which is also the length of the array
``mjData.timer``, as well as the length of the string array :ref:`mjTIMERSTRING` with timer names.

.. mujoco-include:: mjtTimer



.. _tyVisEnums:

Visualization enums
^^^^^^^^^^^^^^^^^^^

The enums below are defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_.


.. _mjtCatBit:

mjtCatBit
~~~~~~~~~

These are the available categories of geoms in the abstract visualizer. The bitmask can be used in the function
:ref:`mjr_render` to specify which categories should be rendered.

.. mujoco-include:: mjtCatBit


.. _mjtMouse:

mjtMouse
~~~~~~~~

These are the mouse actions that the abstract visualizer recognizes. It is up to the user to intercept mouse events
and translate them into these actions, as illustrated in :ref:`simulate.cc <saSimulate>`.

.. mujoco-include:: mjtMouse


.. _mjtPertBit:

mjtPertBit
~~~~~~~~~~

These bitmasks enable the translational and rotational components of the mouse perturbation. For the regular mouse,
only one can be enabled at a time. For the 3D mouse (SpaceNavigator) both can be enabled simultaneously. They are used
in ``mjvPerturb.active``.

.. mujoco-include:: mjtPertBit


.. _mjtCamera:

mjtCamera
~~~~~~~~~

These are the possible camera types, used in ``mjvCamera.type``.

.. mujoco-include:: mjtCamera


.. _mjtLabel:

mjtLabel
~~~~~~~~

These are the abstract visualization elements that can have text labels. Used in ``mjvOption.label``.

.. mujoco-include:: mjtLabel


.. _mjtFrame:

mjtFrame
~~~~~~~~

These are the MuJoCo objects whose spatial frames can be rendered. Used in ``mjvOption.frame``.

.. mujoco-include:: mjtFrame


.. _mjtVisFlag:

mjtVisFlag
~~~~~~~~~~

These are indices in the array ``mjvOption.flags``, whose elements enable/disable the visualization of the
corresponding model or decoration element.

.. mujoco-include:: mjtVisFlag


.. _mjtRndFlag:

mjtRndFlag
~~~~~~~~~~

These are indices in the array ``mjvScene.flags``, whose elements enable/disable OpenGL rendering effects.

.. mujoco-include:: mjtRndFlag


.. _mjtStereo:

mjtStereo
~~~~~~~~~

These are the possible stereo rendering types. They are used in ``mjvScene.stereo``.

.. mujoco-include:: mjtStereo



.. _tyRenderEnums:

Rendering enums
^^^^^^^^^^^^^^^

The enums below are defined in `mjrender.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjrender.h>`_.


.. _mjtGridPos:

mjtGridPos
~~~~~~~~~~

These are the possible grid positions for text overlays. They are used as an argument to the function
:ref:`mjr_overlay`.

.. mujoco-include:: mjtGridPos


.. _mjtFramebuffer:

mjtFramebuffer
~~~~~~~~~~~~~~

These are the possible framebuffers. They are used as an argument to the function :ref:`mjr_setBuffer`.

.. mujoco-include:: mjtFramebuffer


.. _mjtFontScale:

mjtFontScale
~~~~~~~~~~~~

These are the possible font sizes. The fonts are predefined bitmaps stored in the dynamic library at three different
sizes.

.. mujoco-include:: mjtFontScale


.. _mjtFont:

mjtFont
~~~~~~~

These are the possible font types.

.. mujoco-include:: mjtFont


.. _tyUIEnums:

User Interface enums
^^^^^^^^^^^^^^^^^^^^

The enums below are defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_.


.. _mjtButton:

mjtButton
~~~~~~~~~

Mouse button IDs used in the UI framework.

.. mujoco-include:: mjtButton


.. _mjtEvent:

mjtEvent
~~~~~~~~

Event types used in the UI framework.

.. mujoco-include:: mjtEvent


.. _mjtItem:

mjtItem
~~~~~~~

Item types used in the UI framework.

.. mujoco-include:: mjtItem



.. _tyMainStructure:

Main structs
^^^^^^^^^^^^
The three central struct types for physics simulation are :ref:`mjModel`, :ref:`mjOption` (embedded in :ref:`mjModel`)
and :ref:`mjData`. An introductory discussion of these strucures can be found in the Overview under :ref:`Separation of
model and data<Features>`.


.. _mjModel:

mjModel
~~~~~~~

This is the main data structure holding the MuJoCo model. It is treated as constant by the simulator.

.. mujoco-include:: mjModel


.. _mjOption:

mjOption
~~~~~~~~

This is the data structure with simulation options. It corresponds to the MJCF element
:ref:`option <option>`. One instance of it is embedded in mjModel.

.. mujoco-include:: mjOption


.. _mjData:

mjData
~~~~~~

This is the main data structure holding the simulation state. It is the workspace where all functions read their
modifiable inputs and write their outputs.

.. mujoco-include:: mjData



.. _tyAuxStructure:

Auxillary structs
^^^^^^^^^^^^^^^^^
These struct types are used in the engine and their names are prefixed with ``mj``. :ref:`mjVisual`
and :ref:`mjStatistic` are embedded in :ref:`mjModel`, :ref:`mjContact` is embedded in :ref:`mjData`, and :ref:`mjVFS`
is a library-level struct used for loading assets.


.. _mjVisual:

mjVisual
~~~~~~~~

This is the data structure with abstract visualization options. It corresponds to the MJCF element
:ref:`visual <visual>`. One instance of it is embedded in mjModel.

.. mujoco-include:: mjVisual


.. _mjStatistic:

mjStatistic
~~~~~~~~~~~

This is the data structure with model statistics precomputed by the compiler or set by the user. It corresponds to the
MJCF element :ref:`statistic <statistic>`. One instance of it is embedded in mjModel.

.. mujoco-include:: mjStatistic


.. _mjContact:

mjContact
~~~~~~~~~

This is the data structure holding information about one contact. ``mjData.contact`` is a preallocated array of
mjContact data structures, populated at runtime with the contacts found by the collision detector. Additional contact
information is then filled-in by the simulator.

.. mujoco-include:: mjContact


.. _mjVFS:

mjVFS
~~~~~

This is the data structure of the virtual file system. It can only be constructed programmatically, and does not
have an analog in MJCF.

.. mujoco-include:: mjVFS


.. _mjLROpt:

mjLROpt
~~~~~~~

Options for configuring the automatic :ref:`actuator length-range computation<CLengthRange>`.

.. mujoco-include:: mjLROpt



.. _tyStatStructure:

Sim statistics structs
^^^^^^^^^^^^^^^^^^^^^^
These structs are all embedded in :ref:`mjData`, and collect simulation-related statistics.


.. _mjWarningStat:

mjWarningStat
~~~~~~~~~~~~~

This is the data structure holding information about one warning type. ``mjData.warning`` is a preallocated array of
mjWarningStat data structures, one for each warning type.

.. mujoco-include:: mjWarningStat


.. _mjTimerStat:

mjTimerStat
~~~~~~~~~~~

This is the data structure holding information about one timer. ``mjData.timer`` is a preallocated array of
mjTimerStat data structures, one for each timer type.

.. mujoco-include:: mjTimerStat


.. _mjSolverStat:

mjSolverStat
~~~~~~~~~~~~

This is the data structure holding information about one solver iteration. ``mjData.solver`` is a preallocated array
of mjSolverStat data structures, one for each iteration of the solver, up to a maximum of mjNSOLVER. The actual number
of solver iterations is given by ``mjData.solver_iter``.

.. mujoco-include:: mjSolverStat



.. _tyVisStructure:

Visualisation structs
^^^^^^^^^^^^^^^^^^^^^
The names of these struct types are prefixed with ``mjv``.

.. _mjvPerturb:

mjvPerturb
~~~~~~~~~~

This is the data structure holding information about mouse perturbations.

.. mujoco-include:: mjvPerturb


.. _mjvCamera:

mjvCamera
~~~~~~~~~

This is the data structure describing one abstract camera.

.. mujoco-include:: mjvCamera


.. _mjvGLCamera:

mjvGLCamera
~~~~~~~~~~~

This is the data structure describing one OpenGL camera.

.. mujoco-include:: mjvGLCamera


.. _mjvGeom:

mjvGeom
~~~~~~~

This is the data structure describing one abstract visualization geom - which could correspond to a model geom or to a
decoration element constructed by the visualizer.

.. mujoco-include:: mjvGeom


.. _mjvLight:

mjvLight
~~~~~~~~

This is the data structure describing one OpenGL light.

.. mujoco-include:: mjvLight


.. _mjvOption:

mjvOption
~~~~~~~~~

This structure contains options that enable and disable the visualization of various elements.

.. mujoco-include:: mjvOption


.. _mjvScene:

mjvScene
~~~~~~~~

This structure contains everything needed to render the 3D scene in OpenGL.

.. mujoco-include:: mjvScene


.. _mjvFigure:

mjvFigure
~~~~~~~~~

This structure contains everything needed to render a 2D plot in OpenGL. The buffers for line points etc. are
preallocated, and the user has to populate them before calling the function :ref:`mjr_figure` with this
data structure as an argument.

.. mujoco-include:: mjvFigure


.. _tyRenderStructure:

Rendering structs
^^^^^^^^^^^^^^^^^
The names of these struct types are prefixed with ``mjr``.

.. _mjrRect:

mjrRect
~~~~~~~

This structure specifies a rectangle.

.. mujoco-include:: mjrRect


.. _mjrContext:

mjrContext
~~~~~~~~~~

This structure contains the custom OpenGL rendering context, with the ids of all OpenGL resources uploaded to the GPU.

.. mujoco-include:: mjrContext


.. _tyUIStructure:

User Interface structs
^^^^^^^^^^^^^^^^^^^^^^
The names of these struct types are prefixed with ``mjui``.

.. _mjuiState:

mjuiState
~~~~~~~~~

This structure contains the keyboard and mouse state used by the UI framework.

.. mujoco-include:: mjuiState


.. _mjuiThemeSpacing:

mjuiThemeSpacing
~~~~~~~~~~~~~~~~

This structure defines the spacing of UI items in the theme.

.. mujoco-include:: mjuiThemeSpacing


.. _mjuiThemeColor:

mjuiThemeColor
~~~~~~~~~~~~~~

This structure defines the colors of UI items in the theme.

.. mujoco-include:: mjuiThemeColor


.. _mjuiItem:

mjuiItem
~~~~~~~~

This structure defines one UI item.

.. mujoco-include:: mjuiItem


.. _mjuiSection:

mjuiSection
~~~~~~~~~~~

This structure defines one section of the UI.

.. mujoco-include:: mjuiSection


.. _mjUI:

mjUI
~~~~

This structure defines the entire UI.

.. mujoco-include:: mjUI


.. _mjuiDef:

mjuiDef
~~~~~~~

This structure defines one entry in the definition table used for simplified UI construction.

.. mujoco-include:: mjuiDef



.. _tyFunction:

Function types
^^^^^^^^^^^^^^

MuJoCo callbacks have corresponding function types. They are defined in `mjdata.h
<https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`_ and in `mjui.h
<https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_. The actual callback functions are documented
in the :doc:`globals<APIglobals>` page.


.. _mjfGeneric:

mjfGeneric
~~~~~~~~~~

.. code-block:: C

   typedef void (*mjfGeneric)(const mjModel* m, mjData* d);

This is the function type of the callbacks :ref:`mjcb_passive` and :ref:`mjcb_control`.


.. _mjfConFilt:

mjfConFilt
~~~~~~~~~~

.. code-block:: C

   typedef int (*mjfConFilt)(const mjModel* m, mjData* d, int geom1, int geom2);

This is the function type of the callback :ref:`mjcb_contactfilter`. The return value is 1: discard,
0: proceed with collision check.


.. _mjfSensor:

mjfSensor
~~~~~~~~~

.. code-block:: C

   typedef void (*mjfSensor)(const mjModel* m, mjData* d, int stage);

This is the function type of the callback :ref:`mjcb_sensor`.


.. _mjfTime:

mjfTime
~~~~~~~

.. code-block:: C

   typedef mjtNum (*mjfTime)(void);

This is the function type of the callback :ref:`mjcb_time`.


.. _mjfAct:

mjfAct
~~~~~~

.. code-block:: C

   typedef mjtNum (*mjfAct)(const mjModel* m, const mjData* d, int id);

This is the function type of the callbacks :ref:`mjcb_act_dyn`, :ref:`mjcb_act_gain` and :ref:`mjcb_act_bias`.


.. _mjfCollision:

mjfCollision
~~~~~~~~~~~~

.. code-block:: C

   typedef int (*mjfCollision)(const mjModel* m, const mjData* d,
                               mjContact* con, int g1, int g2, mjtNum margin);

This is the function type of the callbacks in the collision table :ref:`mjCOLLISIONFUNC`.


.. _mjfItemEnable:

mjfItemEnable
~~~~~~~~~~~~~

.. code-block:: C

   typedef int (*mjfItemEnable)(int category, void* data);

This is the function type of the predicate function used by the UI framework to determine if each item is enabled or
disabled.
