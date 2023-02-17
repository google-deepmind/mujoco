===============
Data Structures
===============

.. _Type:

Type definitions
----------------

.. _tyPrimitive:

Primitive types
^^^^^^^^^^^^^^^

MuJoCo defines a large number of primitive types described here. Except for :ref:`mjtNum` and
:ref:`mjtByte`, all other primitive types are C enums used to define various integer constants. Note that the
rest of the API does not use these enum types directly. Instead it uses ints, and only the documentation/comments state
that certain ints correspond to certain enum types. This is because we want the API to be compiler-independent, and the
C standard does not dictate how many bytes must be used to represent an enum type. Nevertheless we recommend using these
types when calling the API functions (and letting the compiler do the enum-to-int type cast).

.. _mjtNum:

mjtNum
~~~~~~

.. code-block:: C

   #ifdef mjUSEDOUBLE
       typedef double mjtNum;
   #else
       typedef float mjtNum;
   #endif

| Defined in `mjtnum.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjtnum.h>`_

| This is the floating-point type used throughout the simulator. If the symbol ``mjUSEDOUBLE`` is defined in
  ``mjmodel.h``, this type is defined as ``double``, otherwise it is defined as ``float``. Currently only the
  double-precision version of MuJoCo is distributed, although the entire code base works with single-precision as well.
  We may release the single-precision version in the future for efficiency reasons, but the double-precision version
  will always be available. Thus it is safe to write user code assuming double precision. However, our preference is to
  write code that works with either single or double precision. To this end we provide math utility functions that are
  always defined with the correct floating-point type.

| Note that changing ``mjUSEDOUBLE`` in ``mjtnum.h`` will not change how the library was compiled, and instead will
  result in numerous link errors. In general, the header files distributed with precompiled MuJoCo should never be
  changed by the user.

.. _mjtByte:

mjtByte
~~~~~~~

.. code-block:: C

   typedef unsigned char mjtByte;

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Byte type used to represent boolean variables.

.. _mjtDisableBit:

mjtDisableBit
~~~~~~~~~~~~~

.. mujoco-include:: mjtDisableBit

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Constants which are powers of 2. They are used as bitmasks for the field ``disableflags`` of :ref:`mjOption`.
  At runtime this field is ``m->opt.disableflags``. The number of these constants is given by ``mjNDISABLE`` which is
  also the length of the global string array :ref:`mjDISABLESTRING` with text descriptions of these
  flags.

.. _mjtEnableBit:

mjtEnableBit
~~~~~~~~~~~~

.. mujoco-include:: mjtEnableBit

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Constants which are powers of 2. They are used as bitmasks for the field ``enableflags`` of :ref:`mjOption`.
  At runtime this field is ``m->opt.enableflags``. The number of these constants is given by ``mjNENABLE`` which is also
  the length of the global string array :ref:`mjENABLESTRING` with text descriptions of these flags.

.. _mjtJoint:

mjtJoint
~~~~~~~~

.. mujoco-include:: mjtJoint

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Primitive joint types. These values are used in ``m->jnt_type``. The numbers in the comments indicate how many
  positional coordinates each joint type has. Note that ball joints and rotational components of free joints are
  represented as unit quaternions - which have 4 positional coordinates but 3 degrees of freedom each.

.. _mjtGeom:

mjtGeom
~~~~~~~

.. mujoco-include:: mjtGeom

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Geometric types supported by MuJoCo. The first group are "official" geom types that can be used in the model. The
  second group are geom types that cannot be used in the model but are used by the visualizer to add decorative
  elements. These values are used in ``m->geom_type`` and ``m->site_type``.

.. _mjtCamLight:

mjtCamLight
~~~~~~~~~~~

.. mujoco-include:: mjtCamLight

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Dynamic modes for cameras and lights, specifying how the camera/light position and orientation are computed. These
  values are used in ``m->cam_mode`` and ``m->light_mode``.

.. _mjtTexture:

mjtTexture
~~~~~~~~~~

.. mujoco-include:: mjtTexture

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Texture types, specifying how the texture will be mapped. These values are used in ``m->tex_type``.

.. _mjtIntegrator:

mjtIntegrator
~~~~~~~~~~~~~

.. mujoco-include:: mjtIntegrator

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Numerical integrator types. These values are used in ``m->opt.integrator``.

.. _mjtCollision:

mjtCollision
~~~~~~~~~~~~

.. mujoco-include:: mjtCollision

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Collision modes specifying how candidate geom pairs are generated for near-phase collision checking. These values are
  used in ``m->opt.collision``.

.. _mjtCone:

mjtCone
~~~~~~~

.. mujoco-include:: mjtCone

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Available friction cone types. These values are used in ``m->opt.cone``.

.. _mjtJacobian:

mjtJacobian
~~~~~~~~~~~

.. mujoco-include:: mjtJacobian

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Available Jacobian types. These values are used in ``m->opt.jacobian``.

.. _mjtSolver:

mjtSolver
~~~~~~~~~

.. mujoco-include:: mjtSolver

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Available constraint solver algorithms. These values are used in ``m->opt.solver``.

.. _mjtEq:

mjtEq
~~~~~

.. mujoco-include:: mjtEq

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Equality constraint types. These values are used in ``m->eq_type``.

.. _mjtWrap:

mjtWrap
~~~~~~~

.. mujoco-include:: mjtWrap

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Tendon wrapping object types. These values are used in ``m->wrap_type``.

.. _mjtTrn:

mjtTrn
~~~~~~

.. mujoco-include:: mjtTrn

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Actuator transmission types. These values are used in ``m->actuator_trntype``.

.. _mjtDyn:

mjtDyn
~~~~~~

.. mujoco-include:: mjtDyn

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Actuator dynamics types. These values are used in ``m->actuator_dyntype``.

.. _mjtGain:

mjtGain
~~~~~~~

.. mujoco-include:: mjtGain

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Actuator gain types. These values are used in ``m->actuator_gaintype``.

.. _mjtBias:

mjtBias
~~~~~~~

.. mujoco-include:: mjtBias

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Actuator bias types. These values are used in ``m->actuator_biastype``.

.. _mjtObj:

mjtObj
~~~~~~

.. mujoco-include:: mjtObj

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| MuJoCo object types. These values are used in the support functions :ref:`mj_name2id` and
  :ref:`mj_id2name` to convert between object names and integer ids.

.. _mjtConstraint:

mjtConstraint
~~~~~~~~~~~~~

.. mujoco-include:: mjtConstraint

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Constraint types. These values are not used in mjModel, but are used in the mjData field ``d->efc_type`` when the list
  of active constraints is constructed at each simulation time step.

.. _mjtConstraintState:

mjtConstraintState
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjtConstraintState

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| These values are used by the solver internally to keep track of the constraint states.

.. _mjtSensor:

mjtSensor
~~~~~~~~~

.. mujoco-include:: mjtSensor

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| Sensor types. These values are used in ``m->sensor_type``.

.. _mjtStage:

mjtStage
~~~~~~~~

.. mujoco-include:: mjtStage

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| These are the compute stages for the skipstage parameters of :ref:`mj_forwardSkip` and
  :ref:`mj_inverseSkip`.

.. _mjtDataType:

mjtDataType
~~~~~~~~~~~

.. mujoco-include:: mjtDataType

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| These are the possible sensor data types, used in ``mjData.sensor_datatype``.

.. _mjtWarning:

mjtWarning
~~~~~~~~~~

.. mujoco-include:: mjtWarning

| Defined in `mjdata.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`_

| Warning types. The number of warning types is given by ``mjNWARNING`` which is also the length of the array
  ``mjData.warning``.

.. _mjtTimer:

mjtTimer
~~~~~~~~

.. mujoco-include:: mjtTimer

| Defined in `mjdata.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`_

| Timer types. The number of timer types is given by ``mjNTIMER`` which is also the length of the array
  ``mjData.timer``, as well as the length of the string array :ref:`mjTIMERSTRING` with timer names.

.. _mjtCatBit:

mjtCatBit
~~~~~~~~~

.. mujoco-include:: mjtCatBit

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| These are the available categories of geoms in the abstract visualizer. The bitmask can be used in the function
  :ref:`mjr_render` to specify which categories should be rendered.

.. _mjtMouse:

mjtMouse
~~~~~~~~

.. mujoco-include:: mjtMouse

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| These are the mouse actions that the abstract visualizer recognizes. It is up to the user to intercept mouse events
  and translate them into these actions, as illustrated in :ref:`simulate.cc <saSimulate>`.

.. _mjtPertBit:

mjtPertBit
~~~~~~~~~~

.. mujoco-include:: mjtPertBit

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| These bitmasks enable the translational and rotational components of the mouse perturbation. For the regular mouse,
  only one can be enabled at a time. For the 3D mouse (SpaceNavigator) both can be enabled simultaneously. They are used
  in ``mjvPerturb.active``.

.. _mjtCamera:

mjtCamera
~~~~~~~~~

.. mujoco-include:: mjtCamera

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| These are the possible camera types, used in ``mjvCamera.type``.

.. _mjtLabel:

mjtLabel
~~~~~~~~

.. mujoco-include:: mjtLabel

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| These are the abstract visualization elements that can have text labels. Used in ``mjvOption.label``.

.. _mjtFrame:

mjtFrame
~~~~~~~~

.. mujoco-include:: mjtFrame

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| These are the MuJoCo objects whose spatial frames can be rendered. Used in ``mjvOption.frame``.

.. _mjtVisFlag:

mjtVisFlag
~~~~~~~~~~

.. mujoco-include:: mjtVisFlag

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| These are indices in the array ``mjvOption.flags``, whose elements enable/disable the visualization of the
  corresponding model or decoration element.

.. _mjtRndFlag:

mjtRndFlag
~~~~~~~~~~

.. mujoco-include:: mjtRndFlag

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| These are indices in the array ``mjvScene.flags``, whose elements enable/disable OpenGL rendering effects.

.. _mjtStereo:

mjtStereo
~~~~~~~~~

.. mujoco-include:: mjtStereo

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| These are the possible stereo rendering types. They are used in ``mjvScene.stereo``.

.. _mjtGridPos:

mjtGridPos
~~~~~~~~~~

.. mujoco-include:: mjtGridPos

| Defined in `mjrender.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjrender.h>`_

| These are the possible grid positions for text overlays. They are used as an argument to the function
  :ref:`mjr_overlay`.

.. _mjtFramebuffer:

mjtFramebuffer
~~~~~~~~~~~~~~

.. mujoco-include:: mjtFramebuffer

| Defined in `mjrender.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjrender.h>`_

| These are the possible framebuffers. They are used as an argument to the function :ref:`mjr_setBuffer`.

.. _mjtFontScale:

mjtFontScale
~~~~~~~~~~~~

.. mujoco-include:: mjtFontScale

| Defined in `mjrender.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjrender.h>`_

| These are the possible font sizes. The fonts are predefined bitmaps stored in the dynamic library at three different
  sizes.

.. _mjtFont:

mjtFont
~~~~~~~

.. mujoco-include:: mjtFont

| Defined in `mjrender.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjrender.h>`_

| These are the possible font types.

.. _mjtButton:

mjtButton
~~~~~~~~~

.. mujoco-include:: mjtButton

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| Mouse button IDs used in the UI framework.

.. _mjtEvent:

mjtEvent
~~~~~~~~

.. mujoco-include:: mjtEvent

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| Event types used in the UI framework.

.. _mjtItem:

mjtItem
~~~~~~~

.. mujoco-include:: mjtItem

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| Item types used in the UI framework.

.. _tyFunction:

Function types
^^^^^^^^^^^^^^

MuJoCo callbacks have corresponding function types. They are defined in `mjdata.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`_ and in
`mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_. The actual callback functions are documented later.

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

This is the function type of the callbacks :ref:`mjcb_act_dyn`, :ref:`mjcb_act_gain` and
:ref:`mjcb_act_bias`.

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

.. _tyStructure:

Data structures
^^^^^^^^^^^^^^^

MuJoCo uses several data structures shown below. They are taken directly from the header files which contain comments
for each field.

.. _mjVFS:

mjVFS
~~~~~

.. mujoco-include:: mjVFS

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| This is the data structure with the virtual file system. It can only be constructed programmatically, and does not
  have an analog in MJCF.

.. _mjOption:

mjOption
~~~~~~~~

.. mujoco-include:: mjOption

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| This is the data structure with simulation options. It corresponds to the MJCF element
  :ref:`option <option>`. One instance of it is embedded in mjModel.

.. _mjVisual:

mjVisual
~~~~~~~~

.. mujoco-include:: mjVisual

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| This is the data structure with abstract visualization options. It corresponds to the MJCF element
  :ref:`visual <visual>`. One instance of it is embedded in mjModel.

.. _mjStatistic:

mjStatistic
~~~~~~~~~~~

.. mujoco-include:: mjStatistic

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| This is the data structure with model statistics precomputed by the compiler or set by the user. It corresponds to the
  MJCF element :ref:`statistic <statistic>`. One instance of it is embedded in mjModel.

.. _mjModel:

mjModel
~~~~~~~

.. mujoco-include:: mjModel

| Defined in `mjmodel.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_

| This is the main data structure holding the MuJoCo model. It is treated as constant by the simulator.

.. _mjContact:

mjContact
~~~~~~~~~

.. mujoco-include:: mjContact

| Defined in `mjdata.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`_

| This is the data structure holding information about one contact. ``mjData.contact`` is a preallocated array of
  mjContact data structures, populated at runtime with the contacts found by the collision detector. Additional contact
  information is then filled-in by the simulator.

.. _mjWarningStat:

mjWarningStat
~~~~~~~~~~~~~

.. mujoco-include:: mjWarningStat

| Defined in `mjdata.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`_

| This is the data structure holding information about one warning type. ``mjData.warning`` is a preallocated array of
  mjWarningStat data structures, one for each warning type.

.. _mjTimerStat:

mjTimerStat
~~~~~~~~~~~

.. mujoco-include:: mjTimerStat

| Defined in `mjdata.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`_

| This is the data structure holding information about one timer. ``mjData.timer`` is a preallocated array of
  mjTimerStat data structures, one for each timer type.

.. _mjSolverStat:

mjSolverStat
~~~~~~~~~~~~

.. mujoco-include:: mjSolverStat

| Defined in `mjdata.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`_

| This is the data structure holding information about one solver iteration. ``mjData.solver`` is a preallocated array
  of mjSolverStat data structures, one for each iteration of the solver, up to a maximum of mjNSOLVER. The actual number
  of solver iterations is given by ``mjData.solver_iter``.

.. _mjData:

mjData
~~~~~~

.. mujoco-include:: mjData

| Defined in `mjdata.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`_

| This is the main data structure holding the simulation state. It is the workspace where all functions read their
  modifiable inputs and write their outputs.

.. _mjvPerturb:

mjvPerturb
~~~~~~~~~~

.. mujoco-include:: mjvPerturb

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| This is the data structure holding information about mouse perturbations.

.. _mjvCamera:

mjvCamera
~~~~~~~~~

.. mujoco-include:: mjvCamera

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| This is the data structure describing one abstract camera.

.. _mjvGLCamera:

mjvGLCamera
~~~~~~~~~~~

.. mujoco-include:: mjvGLCamera

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| This is the data structure describing one OpenGL camera.

.. _mjvGeom:

mjvGeom
~~~~~~~

.. mujoco-include:: mjvGeom

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| This is the data structure describing one abstract visualization geom - which could correspond to a model geom or to a
  decoration element constructed by the visualizer.

.. _mjvLight:

mjvLight
~~~~~~~~

.. mujoco-include:: mjvLight

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| This is the data structure describing one OpenGL light.

.. _mjvOption:

mjvOption
~~~~~~~~~

.. mujoco-include:: mjvOption

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| This structure contains options that enable and disable the visualization of various elements.

.. _mjvScene:

mjvScene
~~~~~~~~

.. mujoco-include:: mjvScene

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| This structure contains everything needed to render the 3D scene in OpenGL.

.. _mjvFigure:

mjvFigure
~~~~~~~~~

.. mujoco-include:: mjvFigure

| Defined in `mjvisualize.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_

| This structure contains everything needed to render a 2D plot in OpenGL. The buffers for line points etc. are
  preallocated, and the user has to populate them before calling the function :ref:`mjr_figure` with this
  data structure as an argument.

.. _mjrRect:

mjrRect
~~~~~~~

.. mujoco-include:: mjrRect

| Defined in `mjrender.h (57) <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjrender.h#L57>`_

| This structure specifies a rectangle.

.. _mjrContext:

mjrContext
~~~~~~~~~~

.. mujoco-include:: mjrContext

| Defined in `mjrender.h (67) <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjrender.h#L67>`_

| This structure contains the custom OpenGL rendering context, with the ids of all OpenGL resources uploaded to the GPU.

.. _mjuiState:

mjuiState
~~~~~~~~~

.. mujoco-include:: mjuiState

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| This structure contains the keyboard and mouse state used by the UI framework.

.. _mjuiThemeSpacing:

mjuiThemeSpacing
~~~~~~~~~~~~~~~~

.. mujoco-include:: mjuiThemeSpacing

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| This structure defines the spacing of UI items in the theme.

.. _mjuiThemeColor:

mjuiThemeColor
~~~~~~~~~~~~~~

.. mujoco-include:: mjuiThemeColor

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| This structure defines the colors of UI items in the theme.

.. _mjuiItem:

mjuiItem
~~~~~~~~

.. mujoco-include:: mjuiItem

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| This structure defines one UI item.

.. _mjuiSection:

mjuiSection
~~~~~~~~~~~

.. mujoco-include:: mjuiSection

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| This structure defines one section of the UI.

.. _mjUI:

mjUI
~~~~

.. mujoco-include:: mjUI

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| This structure defines the entire UI.

.. _mjuiDef:

mjuiDef
~~~~~~~

.. mujoco-include:: mjuiDef

| Defined in `mjui.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_

| This structure defines one entry in the definition table used for simplified UI construction.

.. _tyXMacro:

X Macros
^^^^^^^^

The X Macros are not needed in most user projects. They are used internally to allocate the model, and are also
available for users who know how to use this programming technique. See the header file `mjxmacro.h
<https://github.com/deepmind/mujoco/blob/main/include/mujoco/mjxmacro.h>`_ for the actual definitions. They are
particularly useful in writing MuJoCo wrappers for scripting languages, where dynamic structures matching the MuJoCo
data structures need to be constructed programmatically.

.. _MJOPTION_SCALARS:

MJOPTION_SCALARS
~~~~~~~~~~~~~~~~

Scalar fields of mjOption.

.. _MJOPTION_VECTORS:

MJOPTION_VECTORS
~~~~~~~~~~~~~~~~

Vector fields of mjOption.

.. _MJMODEL_INTS:

MJMODEL_INTS
~~~~~~~~~~~~

Int fields of mjModel.

.. _MJMODEL_POINTERS:

MJMODEL_POINTERS
~~~~~~~~~~~~~~~~

Pointer fields of mjModel.

.. _MJDATA_SCALAR:

MJDATA_SCALAR
~~~~~~~~~~~~~

Scalar fields of mjData.

.. _MJDATA_VECTOR:

MJDATA_VECTOR
~~~~~~~~~~~~~

Vector fields of mjData.

.. _MJDATA_POINTERS:

MJDATA_POINTERS
~~~~~~~~~~~~~~~

Pointer fields of mjData.

.. _Global-variables:

Global variables
----------------

.. _glError:

Error callbacks
^^^^^^^^^^^^^^^

All user callbacks (i.e., global function pointers whose name starts with 'mjcb') are initially set to NULL, which
disables them and allows the default processing to take place. To install a callback, simply set the corresponding
global pointer to a user function of the correct type. Keep in mind that these are global and not model-specific. So if
you are simulating multiple models in parallel, they use the same set of callbacks.

.. _mju_user_error:

mju_user_error
~~~~~~~~~~~~~~

.. code-block:: C

   extern void (*mju_user_error)(const char*);

This is called from within the main error function :ref:`mju_error`. When installed, this function overrides the default
error processing. Once it prints error messages (or whatever else the user wants to do), it must **exit** the program.
MuJoCo is written with the assumption that mju_error will not return. If it does, the behavior of the software is
undefined.

.. _mju_user_warning:

mju_user_warning
~~~~~~~~~~~~~~~~

.. code-block:: C

   extern void (*mju_user_warning)(const char*);

This is called from within the main warning function :ref:`mju_warning`. It is similar to the error handler, but instead
it must return without exiting the program.

.. _glMemory:

Memory callbacks
^^^^^^^^^^^^^^^^

The purpose of the memory callbacks is to allow the user to install custom memory allocation and deallocation
mechanisms. One example where we have found this to be useful is a MATLAB wrapper for MuJoCo, where mex files are
expected to use MATLAB's memory mechanism for permanent memory allocation.

.. _mju_user_malloc:

mju_user_malloc
~~~~~~~~~~~~~~~

.. code-block:: C

   extern void* (*mju_user_malloc)(size_t);

If this is installed, the MuJoCo runtime will use it to allocate all heap memory it needs (instead of using aligned
malloc). The user allocator must allocate memory aligned on 8-byte boundaries. Note that the parser and compiler are
written in C++ and sometimes allocate memory with the "new" operator which bypasses this mechanism.

.. _mju_user_free:

mju_user_free
~~~~~~~~~~~~~

.. code-block:: C

   extern void (*mju_user_free)(void*);

If this is installed, MuJoCo will free any heap memory it allocated by calling this function (instead of using aligned
free).

.. _glPhysics:

Physics callbacks
^^^^^^^^^^^^^^^^^

| The physics callbacks are the main mechanism for modifying the behavior of the simulator, beyond setting various
  options. The options control the operation of the default pipeline, while callbacks extend the pipeline at
  well-defined places. This enables advanced users to implement many interesting functions which we have not thought of,
  while still taking advantage of the default pipeline. As with all other callbacks, there is no automated error
  checking - instead we assume that the authors of callback functions know what they are doing.

| Custom physics callbacks will often need parameters that are not standard in MJCF. This is largely why we have
  provided custom fields as well as user data arrays in MJCF. The idea is to "instrument" the MJCF model by entering the
  necessary user parameters, and then write callbacks that look for those parameters and perform the corresponding
  computations. We strongly encourage users to write callbacks that check the model for the presence of user parameters
  before accessing them - so that when a regular model is loaded, the callback disables itself automatically instead of
  causing the software to crash.

.. _mjcb_passive:

mjcb_passive
~~~~~~~~~~~~

.. code-block:: C

   extern mjfGeneric mjcb_passive;

This is used to implement a custom passive force in joint space; if the force is more naturally defined in Cartesian
space, use the end-effector Jacobian to map it to joint space. By "passive" we do not mean a force that does no positive
work (as in physics), but simply a force that depends only on position and velocity but not on control. There are
standard passive forces in MuJoCo arising from springs, dampers, viscosity and density of the medium. They are computed
in ``mjData.qfrc_passive`` before mjcb_passive is called. The user callback should add to this vector instead of
overwriting it (otherwise the standard passive forces will be lost).

.. _mjcb_control:

mjcb_control
~~~~~~~~~~~~

.. code-block:: C

   extern mjfGeneric mjcb_control;

This is the most commonly used callback. It implements a control law, by writing in the vector of controls
``mjData.ctrl``. It can also write in ``mjData.qfrc_applied`` and ``mjData.xfrc_applied``. The values written in these
vectors can depend on position, velocity and all other quantities derived from them, but cannot depend on contact forces
and other quantities that are computed after the control is specified. If the callback accesses the latter fields, their
values do not correspond to the current time step.

The control callback is called from within :ref:`mj_forward` and :ref:`mj_step`, just before the controls and applied
forces are needed. When using the RK integrator, it will be called 4 times per step. The alternative way of specifying
controls and applied forces is to set them before ``mj_step``, or use ``mj_step1`` and ``mj_step2``. The latter approach
allows setting the controls after the position and velocity computations have been performed by ``mj_step1``, allowing
these results to be utilized in computing the control (similar to using mjcb_control). However, the only way to change
the controls between sub-steps of the RK integrator is to define the control callback.

.. _mjcb_contactfilter:

mjcb_contactfilter
~~~~~~~~~~~~~~~~~~

.. code-block:: C

   extern mjfConFilt mjcb_contactfilter;

This callback can be used to replace MuJoCo's default collision filtering. When installed, this function is called for
each pair of geoms that have passed the broad-phase test (or are predefined geom pairs in the MJCF) and are candidates
for near-phase collision. The default processing uses the contype and conaffinity masks, the parent-child filter and
some other considerations related to welded bodies to decide if collision should be allowed. This callback replaces the
default processing, but keep in mind that the entire mechanism is being replaced. So for example if you still want to
take advantage of contype/conaffinity, you have to re-implement it in the callback.

.. _mjcb_sensor:

mjcb_sensor
~~~~~~~~~~~

.. code-block:: C

   extern mjfSensor mjcb_sensor;

This callback populates fields of ``mjData.sensordata`` corresponding to user-defined sensors. It is called if it is
installed and the model contains user-defined sensors. It is called once per compute stage (mjSTAGE_POS, mjSTAGE_VEL,
mjSTAGE_ACC) and must fill in all user sensor values for that stage. The user-defined sensors have dimensionality and
data types defined in the MJCF model which must be respected by the callback.

.. _mjcb_time:

mjcb_time
~~~~~~~~~

.. code-block:: C

   extern mjfTime mjcb_time;

Installing this callback enables the built-in profiler, and keeps timing statistics in ``mjData.timer``. The return type
is mjtNum, while the time units are up to the user. :ref:`simulate.cc <saSimulate>` assumes the unit is 1 millisecond.
In order to be useful, the callback should use high-resolution timers with at least microsecond precision. This is
because the computations being timed are very fast.

.. _mjcb_act_dyn:

mjcb_act_dyn
~~~~~~~~~~~~

.. code-block:: C

   extern mjfAct mjcb_act_dyn;

This callback implements custom activation dynamics: it must return the value of ``mjData.act_dot`` for the specified
actuator. This is the time-derivative of the activation state vector ``mjData.act``. It is called for model actuators
with user dynamics (mjDYN_USER). If such actuators exist in the model but the callback is not installed, their
time-derivative is set to 0.

.. _mjcb_act_gain:

mjcb_act_gain
~~~~~~~~~~~~~

.. code-block:: C

   extern mjfAct mjcb_act_gain;

This callback implements custom actuator gains: it must return the gain for the specified actuator with
``mjModel.actuator_gaintype`` set to mjGAIN_USER. If such actuators exist in the model and this callback is not
installed, their gains are set to 1.

.. _mjcb_act_bias:

mjcb_act_bias
~~~~~~~~~~~~~

.. code-block:: C

   extern mjfAct mjcb_act_bias;

This callback implements custom actuator biases: it must return the bias for the specified actuator with
``mjModel.actuator_biastype`` set to mjBIAS_USER. If such actuators exist in the model and this callback is not
installed, their biases are set to 0.

.. _glCollision:

Collision table
^^^^^^^^^^^^^^^

.. _mjCOLLISIONFUNC:

mjCOLLISIONFUNC
~~~~~~~~~~~~~~~

.. code-block:: C

   extern mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES];

Table of pairwise collision functions indexed by geom types. Only the upper-right triangle is used. The user can replace
these function pointers with custom routines, replacing MuJoCo's collision mechanism. If a given entry is NULL, the
corresponding pair of geom types cannot be collided. Note that these functions apply only to near-phase collisions. The
broadphase mechanism is built-in and cannot be modified.

.. _glString:

String constants
^^^^^^^^^^^^^^^^

The string constants described here are provided for user convenience. They correspond to the English names of lists of
options, and can be displayed in menus or dialogs in a GUI. The code sample :ref:`simulate.cc <saSimulate>` illustrates
how they can be used.

.. _mjDISABLESTRING:

mjDISABLESTRING
~~~~~~~~~~~~~~~

.. code-block:: C

   extern const char* mjDISABLESTRING[mjNDISABLE];

Names of the disable bits defined by :ref:`mjtDisableBit`.

.. _mjENABLESTRING:

mjENABLESTRING
~~~~~~~~~~~~~~

.. code-block:: C

   extern const char* mjENABLESTRING[mjNENABLE];

Names of the enable bits defined by :ref:`mjtEnableBit`.

.. _mjTIMERSTRING:

mjTIMERSTRING
~~~~~~~~~~~~~

.. code-block:: C

   extern const char* mjTIMERSTRING[mjNTIMER];

Names of the mjData timers defined by :ref:`mjtTimer`.

.. _mjLABELSTRING:

mjLABELSTRING
~~~~~~~~~~~~~

.. code-block:: C

   extern const char* mjLABELSTRING[mjNLABEL];

Names of the visual labeling modes defined by :ref:`mjtLabel`.

.. _mjFRAMESTRING:

mjFRAMESTRING
~~~~~~~~~~~~~

.. code-block:: C

   extern const char* mjFRAMESTRING[mjNFRAME];

Names of the frame visualization modes defined by :ref:`mjtFrame`.

.. _mjVISSTRING:

mjVISSTRING
~~~~~~~~~~~

.. code-block:: C

   extern const char* mjVISSTRING[mjNVISFLAG][3];

| Descriptions of the abstract visualization flags defined by :ref:`mjtVisFlag`. For each flag there are three strings,
  with the following meaning:

| [0]: flag name;

| [1]: the string "0" or "1" indicating if the flag is on or off by default, as set by
  :ref:`mjv_defaultOption`;

| [2]: one-character string with a suggested keyboard shortcut, used in :ref:`simulate.cc <saSimulate>`.

.. _mjRNDSTRING:

mjRNDSTRING
~~~~~~~~~~~

.. code-block:: C

   extern const char* mjRNDSTRING[mjNRNDFLAG][3];

Descriptions of the OpenGL rendering flags defined by :ref:`mjtRndFlag`. The three strings for each flag have the same
format as above, except the defaults here are set by :ref:`mjv_makeScene`.

.. _glNumeric:

Numeric constants
^^^^^^^^^^^^^^^^^

| Many integer constants were already documented in the primitive types above. In addition, the header files define
  several other constants documented here. Unless indicated otherwise, each entry in the table below is defined in
  mjmodel.h. Note that some extended key codes are defined in mjui.h which are not shown in the table below. Their names
  are in the format mjKEY_XXX. They correspond to GLFW key codes.

+------------------+--------+----------------------------------------------------------------------------------------+
| symbol           | value  | description                                                                            |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMINVAL         | 1E-15  | The minimal value allowed in any denominator, and in general any mathematical          |
|                  |        | operation where 0 is not allowed. In almost all cases, MuJoCo silently clamps smaller  |
|                  |        | values to mjMINVAL.                                                                    |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjPI             | pi     | The value of pi. This is used in various trigonometric functions, and also for         |
|                  |        | conversion from degrees to radians in the compiler.                                    |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXVAL         | 1E+10  | The maximal absolute value allowed in mjData.qpos, mjData.qvel, mjData.qacc. The API   |
|                  |        | functions :ref:`mj_checkPos`, :ref:`mj_checkVel`, :ref:`mj_checkAcc` use this constant |
|                  |        | to detect instability.                                                                 |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMINMU          | 1E-5   | The minimal value allowed in any friction coefficient. Recall that MuJoCo's contact    |
|                  |        | model allows different number of friction dimensions to be included, as specified by   |
|                  |        | the :at:`condim`    attribute. If however a given friction dimension is included, its  |
|                  |        | friction is not allowed to be smaller than this constant. Smaller values are           |
|                  |        | automatically clamped to this constant.                                                |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMINIMP         | 0.0001 | The minimal value allowed in any constraint impedance. Smaller values are              |
|                  |        | automatically clamped to this constant.                                                |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXIMP         | 0.9999 | The maximal value allowed in any constraint impedance. Larger values are automatically |
|                  |        | clamped to this constant.                                                              |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXCONPAIR     | 50     | The maximal number of contacts points that can be generated per geom pair. MuJoCo's    |
|                  |        | built-in collision functions respect this limit, and user-defined functions should     |
|                  |        | also respect it. Such functions are called with a return buffer of size mjMAXCONPAIR;  |
|                  |        | attempting to write more contacts in the buffer can cause unpredictable behavior.      |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXVFS         | 200    | The maximal number of files in the virtual file system.                                |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXVFSNAME     | 100    | The maximal number of characters in the name of each file in the virtual file system.  |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNEQDATA        | 11     | The maximal number of real-valued parameters used to define each equality constraint.  |
|                  |        | Determines the size of mjModel.eq_data. This and the next five constants correspond to |
|                  |        | array sizes which we have not fully settled. There may be reasons to increase them in  |
|                  |        | the future, so as to accommodate extra parameters needed for more elaborate            |
|                  |        | computations. This is why we maintain them as symbolic constants that can be easily    |
|                  |        | changed, as opposed to the array size for representing quaternions for example - which |
|                  |        | has no reason to change.                                                               |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNDYN           | 10     | The maximal number of real-valued parameters used to define the activation dynamics of |
|                  |        | each actuator. Determines the size of mjModel.actuator_dynprm.                         |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNGAIN          | 10     | The maximal number of real-valued parameters used to define the gain of each actuator. |
|                  |        | Determines the size of mjModel.actuator_gainprm.                                       |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNBIAS          | 10     | The maximal number of real-valued parameters used to define the bias of each actuator. |
|                  |        | Determines the size of mjModel.actuator_biasprm.                                       |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNFLUID         | 12     | The number of per-geom fluid interaction parameters required by the ellipsoidal model. |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNREF           | 2      | The maximal number of real-valued parameters used to define the reference acceleration |
|                  |        | of each scalar constraint. Determines the size of all mjModel.XXX_solref fields.       |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNIMP           | 5      | The maximal number of real-valued parameters used to define the impedance of each      |
|                  |        | scalar constraint. Determines the size of all mjModel.XXX_solimp fields.               |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNSOLVER        | 1000   | The size of the preallocated array ``mjData.solver``. This is used to store diagnostic |
|                  |        | information about each iteration of the constraint solver. The actual number of        |
|                  |        | iterations is given by ``mjData.solver_iter``.                                         |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNGROUP         | 6      | The number of geom, site, joint, tendon and actuator groups whose rendering can be     |
|                  |        | enabled and disabled via mjvOption. Defined in mjvisualize.h.                          |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXOVERLAY     | 500    | The maximal number of characters in overlay text for rendering. Defined in             |
|                  |        | mjvisualize.h.                                                                         |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXLINE        | 100    | The maximal number of lines per 2D figure (mjvFigure). Defined in mjvisualize.h.       |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXLINEPNT     | 1000   | The maximal number of points in each line in a 2D figure. Note that the buffer         |
|                  |        | mjvFigure.linepnt has length 2*mjMAXLINEPNT because each point has X and Y             |
|                  |        | coordinates. Defined in mjvisualize.h.                                                 |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXPLANEGRID   | 200    | The maximal number of grid lines in each dimension for rendering planes. Defined in    |
|                  |        | mjvisualize.h.                                                                         |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjNAUX           | 10     | Number of auxiliary buffers that can be allocated in mjrContext. Defined in            |
|                  |        | mjrender.h.                                                                            |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXTEXTURE     | 1000   | Maximum number of textures allowed. Defined in mjrender.h.                             |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXUISECT      | 10     | Maximum number of UI sections. Defined in mjui.h.                                      |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXUIITEM      | 80     | Maximum number of items per UI section. Defined in mjui.h.                             |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXUITEXT      | 500    | Maximum number of characters in UI fields 'edittext' and 'other'. Defined in mjui.h.   |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXUINAME      | 40     | Maximum number of characters in any UI name. Defined in mjui.h.                        |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXUIMULTI     | 20     | Maximum number of radio and select items in UI group. Defined in mjui.h.               |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXUIEDIT      | 5      | Maximum number of elements in UI edit list. Defined in mjui.h.                         |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjMAXUIRECT      | 15     | Maximum number of UI rectangles. Defined in mjui.h.                                    |
+------------------+--------+----------------------------------------------------------------------------------------+
| mjVERSION_HEADER | 211    | The version of the MuJoCo headers; changes with every release. This is an integer      |
|                  |        | equal to 100x the software version, so 210 corresponds to version 2.1. Defined in      |
|                  |        | mujoco.h. The API function :ref:`mj_version` returns a number with the same meaning    |
|                  |        | but for the compiled library.                                                          |
+------------------+--------+----------------------------------------------------------------------------------------+

