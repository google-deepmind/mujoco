=============
API Reference
=============

Introduction
------------

This chapter is the reference manual for MuJoCo. It is generated from the header files included with MuJoCo, but also
contains additional text not available in the headers.

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

.. _API:

API functions
-------------

The main header `mujoco.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mujoco.h>`_ exposes a very large
number of functions. However the functions that most users are likely to need are a small fraction. For example,
:ref:`simulate.cc <saSimulate>` which is as elaborate as a MuJoCo application is likely to get, calls around 40 of these
functions, while ``basic.cc`` calls around 20. The rest are explosed just in case someone has a use for them. This
includes us as users of MuJoCo -- we do our own work with the public library instead of relying on internal builds.

.. _Activation:

Activation
^^^^^^^^^^

The functions in this section are maintained for backward compatibility with the now-removed activation mechanism.

.. _mj_activate:

mj_activate
~~~~~~~~~~~

.. mujoco-include:: mj_activate

Does nothing, returns 1.

.. _mj_deactivate:

mj_deactivate
~~~~~~~~~~~~~

.. mujoco-include:: mj_deactivate

Does nothing.

.. _Virtualfilesystem:

Virtual file system
^^^^^^^^^^^^^^^^^^^

Virtual file system (VFS) functionality was introduced in MuJoCo 1.50. It enables the user to load all necessary files
in memory, including MJB binary model files, XML files (MJCF, URDF and included files), STL meshes, PNGs for textures
and height fields, and HF files in our custom height field format. Model and resource files in the VFS can also be
constructed programmatically (say using a Python library that writes to memory). Once all desired files are in the VFS,
the user can call :ref:`mj_loadModel` or :ref:`mj_loadXML` with a pointer to the VFS. When this pointer is not NULL, the
loaders will first check the VFS for any file they are about to load, and only access the disk if the file is not found
in the VFS. The file names stored in the VFS have their name and extension but the path information is stripped; this
can be bypassed however by using a custom path symbol in the file names, say "mydir_myfile.xml".

The entire VFS is contained in the data structure :ref:`mjVFS`. All utility functions for maintaining the VFS operate on
this data structure. The common usage pattern is to first clear it with mj_defaultVFS, then add disk files to it with
mj_addFileVFS (which allocates memory buffers and loads the file content in memory), then call mj_loadXML or
mj_loadModel, and then clear everything with mj_deleteVFS.

.. _mj_defaultVFS:

mj_defaultVFS
~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultVFS

Initialize VFS to empty (no deallocation).

.. _mj_addFileVFS:

mj_addFileVFS
~~~~~~~~~~~~~

.. mujoco-include:: mj_addFileVFS

Add file to VFS, return 0: success, 1: full, 2: repeated name, -1: not found on disk.

.. _mj_makeEmptyFileVFS:

mj_makeEmptyFileVFS
~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_makeEmptyFileVFS

Make empty file in VFS, return 0: success, 1: full, 2: repeated name.

.. _mj_findFileVFS:

mj_findFileVFS
~~~~~~~~~~~~~~

.. mujoco-include:: mj_findFileVFS

Return file index in VFS, or -1 if not found in VFS.

.. _mj_deleteFileVFS:

mj_deleteFileVFS
~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_deleteFileVFS

Delete file from VFS, return 0: success, -1: not found in VFS.

.. _mj_deleteVFS:

mj_deleteVFS
~~~~~~~~~~~~

.. mujoco-include:: mj_deleteVFS

Delete all files from VFS.

.. _Parseandcompile:

Parse and compile
^^^^^^^^^^^^^^^^^

The key function here is mj_loadXML. It invokes the built-in parser and compiler, and either returns a pointer to a
valid mjModel, or NULL - in which case the user should check the error information in the user-provided string. The
model and all files referenced in it can be loaded from disk or from a VFS when provided.

.. _mj_loadXML:

mj_loadXML
~~~~~~~~~~

.. mujoco-include:: mj_loadXML

Parse XML file in MJCF or URDF format, compile it, return low-level model. If vfs is not NULL, look up files in vfs
before reading from disk. If error is not NULL, it must have size error_sz.

.. _mj_saveLastXML:

mj_saveLastXML
~~~~~~~~~~~~~~

.. mujoco-include:: mj_saveLastXML

Update XML data structures with info from low-level model, save as MJCF. If error is not NULL, it must have size
error_sz.

.. _mj_freeLastXML:

mj_freeLastXML
~~~~~~~~~~~~~~

.. mujoco-include:: mj_freeLastXML

Free last XML model if loaded. Called internally at each load.

.. _mj_printSchema:

mj_printSchema
~~~~~~~~~~~~~~

.. mujoco-include:: mj_printSchema

Print internal XML schema as plain text or HTML, with style-padding or ``&nbsp;``.

.. _Mainsimulation:

Main simulation
^^^^^^^^^^^^^^^

These are the main entry points to the simulator. Most users will only need to call ``mj_step``, which computes
everything and advanced the simulation state by one time step. Controls and applied forces must either be set in advance
(in mjData.ctrl, qfrc_applied and xfrc_applied), or a control callback mjcb_control must be installed which will be
called just before the controls and applied forces are needed. Alternatively, one can use ``mj_step1`` and ``mj_step2``
which break down the simulation pipeline into computations that are executed before and after the controls are needed;
in this way one can set controls that depend on the results from ``mj_step1``. Keep in mind though that the RK4 solver
does not work with mj_step1/2.

mj_forward performs the same computations as ``mj_step`` but without the integration. It is useful after loading or
resetting a model (to put the entire mjData in a valid state), and also for out-of-order computations that involve
sampling or finite-difference approximations.

mj_inverse runs the inverse dynamics, and writes its output in mjData.qfrc_inverse. Note that mjData.qacc must be set
before calling this function. Given the state (qpos, qvel, act), mj_forward maps from force to acceleration, while
mj_inverse maps from acceleration to force. Mathematically these functions are inverse of each other, but numerically
this may not always be the case because the forward dynamics rely on a constraint optimization algorithm which is
usually terminated early. The difference between the results of forward and inverse dynamics can be computed with the
function :ref:`mj_compareFwdInv`, which can be though of as another solver accuracy check (as well as a general sanity
check).

The skip version of mj_forward and mj_inverse are useful for example when qpos was unchanged but qvel was changed
(usually in the context of finite differencing). Then there is no point repeating the computations that only depend on
qpos. Calling the dynamics with skipstage = mjSTAGE_POS will achieve these savings.

.. _mj_step:

mj_step
~~~~~~~

.. mujoco-include:: mj_step

Advance simulation, use control callback to obtain external force and control.

.. _mj_step1:

mj_step1
~~~~~~~~

.. mujoco-include:: mj_step1

Advance simulation in two steps: before external force and control is set by user.

.. _mj_step2:

mj_step2
~~~~~~~~

.. mujoco-include:: mj_step2

Advance simulation in two steps: after external force and control is set by user.

.. _mj_forward:

mj_forward
~~~~~~~~~~

.. mujoco-include:: mj_forward

Forward dynamics: same as mj_step but do not integrate in time.

.. _mj_inverse:

mj_inverse
~~~~~~~~~~

.. mujoco-include:: mj_inverse

Inverse dynamics: qacc must be set before calling.

.. _mj_forwardSkip:

mj_forwardSkip
~~~~~~~~~~~~~~

.. mujoco-include:: mj_forwardSkip

Forward dynamics with skip; skipstage is mjtStage.

.. _mj_inverseSkip:

mj_inverseSkip
~~~~~~~~~~~~~~

.. mujoco-include:: mj_inverseSkip

Inverse dynamics with skip; skipstage is mjtStage.

.. _Initialization:

Initialization
^^^^^^^^^^^^^^

This section contains functions that load/initialize the model or other data structures. Their use is well illustrated
in the code samples.

.. _mj_defaultLROpt:

mj_defaultLROpt
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultLROpt

Set default options for length range computation.

.. _mj_defaultSolRefImp:

mj_defaultSolRefImp
~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultSolRefImp

Set solver parameters to default values.

.. _mj_defaultOption:

mj_defaultOption
~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultOption

Set physics options to default values.

.. _mj_defaultVisual:

mj_defaultVisual
~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultVisual

Set visual options to default values.

.. _mj_copyModel:

mj_copyModel
~~~~~~~~~~~~

.. mujoco-include:: mj_copyModel

Copy mjModel, allocate new if dest is NULL.

.. _mj_saveModel:

mj_saveModel
~~~~~~~~~~~~

.. mujoco-include:: mj_saveModel

Save model to binary MJB file or memory buffer; buffer has precedence when given.

.. _mj_loadModel:

mj_loadModel
~~~~~~~~~~~~

.. mujoco-include:: mj_loadModel

Load model from binary MJB file. If vfs is not NULL, look up file in vfs before reading from disk.

.. _mj_deleteModel:

mj_deleteModel
~~~~~~~~~~~~~~

.. mujoco-include:: mj_deleteModel

Free memory allocation in model.

.. _mj_sizeModel:

mj_sizeModel
~~~~~~~~~~~~

.. mujoco-include:: mj_sizeModel

Return size of buffer needed to hold model.

.. _mj_makeData:

mj_makeData
~~~~~~~~~~~

.. mujoco-include:: mj_makeData

Allocate mjData corresponding to given model.

.. _mj_copyData:

mj_copyData
~~~~~~~~~~~

.. mujoco-include:: mj_copyData

Copy mjData.

.. _mj_resetData:

mj_resetData
~~~~~~~~~~~~

.. mujoco-include:: mj_resetData

Reset data to defaults.

.. _mj_resetDataDebug:

mj_resetDataDebug
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_resetDataDebug

Reset data to defaults, fill everything else with debug_value.

.. _mj_resetDataKeyframe:

mj_resetDataKeyframe
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_resetDataKeyframe

Reset data, set fields from specified keyframe.

.. _mj_stackAlloc:

mj_stackAlloc
~~~~~~~~~~~~~

.. mujoco-include:: mj_stackAlloc

Allocate array of specified size on mjData stack. Call mju_error on stack overflow.

.. _mj_deleteData:

mj_deleteData
~~~~~~~~~~~~~

.. mujoco-include:: mj_deleteData

Free memory allocation in mjData.

.. _mj_resetCallbacks:

mj_resetCallbacks
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_resetCallbacks

Reset all callbacks to NULL pointers (NULL is the default).

.. _mj_setConst:

mj_setConst
~~~~~~~~~~~

.. mujoco-include:: mj_setConst

Set constant fields of mjModel, corresponding to qpos0 configuration.

.. _mj_setLengthRange:

mj_setLengthRange
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_setLengthRange

Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error.

.. _Printing:

Printing
^^^^^^^^

These functions can be used to print various quantities to the screen for debugging purposes.


.. _mj_printFormattedModel:

mj_printFormattedModel
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printFormattedModel

Print ``mjModel`` to text file, specifying format. ``float_format`` must be a valid printf-style format string for a
single float value.

.. _mj_printModel:

mj_printModel
~~~~~~~~~~~~~

.. mujoco-include:: mj_printModel

Print model to text file.

.. _mj_printFormattedData:

mj_printFormattedData
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printFormattedData

Print ``mjData`` to text file, specifying format. ``float_format`` must be a valid printf-style format string for a
single float value.

.. _mj_printData:

mj_printData
~~~~~~~~~~~~

.. mujoco-include:: mj_printData

Print data to text file.

.. _mju_printMat:

mju_printMat
~~~~~~~~~~~~

.. mujoco-include:: mju_printMat

Print matrix to screen.

.. _mju_printMatSparse:

mju_printMatSparse
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_printMatSparse

Print sparse matrix to screen.

.. _Components:

Components
^^^^^^^^^^

These are components of the simulation pipeline, called internally from mj_step, mj_forward and mj_inverse. It is
unlikely that the user will need to call them.

.. _mj_fwdPosition:

mj_fwdPosition
~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdPosition

Run position-dependent computations.

.. _mj_fwdVelocity:

mj_fwdVelocity
~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdVelocity

Run velocity-dependent computations.

.. _mj_fwdActuation:

mj_fwdActuation
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdActuation

Compute actuator force qfrc_actuator.

.. _mj_fwdAcceleration:

mj_fwdAcceleration
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdAcceleration

Add up all non-constraint forces, compute qacc_smooth.

.. _mj_fwdConstraint:

mj_fwdConstraint
~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdConstraint

Run selected constraint solver.

.. _mj_Euler:

mj_Euler
~~~~~~~~

.. mujoco-include:: mj_Euler

Euler integrator, semi-implicit in velocity.

.. _mj_RungeKutta:

mj_RungeKutta
~~~~~~~~~~~~~

.. mujoco-include:: mj_RungeKutta

Runge-Kutta explicit order-N integrator.

.. _mj_invPosition:

mj_invPosition
~~~~~~~~~~~~~~

.. mujoco-include:: mj_invPosition

Run position-dependent computations in inverse dynamics.

.. _mj_invVelocity:

mj_invVelocity
~~~~~~~~~~~~~~

.. mujoco-include:: mj_invVelocity

Run velocity-dependent computations in inverse dynamics.

.. _mj_invConstraint:

mj_invConstraint
~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_invConstraint

Apply the analytical formula for inverse constraint dynamics.

.. _mj_compareFwdInv:

mj_compareFwdInv
~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_compareFwdInv

Compare forward and inverse dynamics, save results in fwdinv.

.. _Subcomponents:

Sub components
^^^^^^^^^^^^^^

These are sub-components of the simulation pipeline, called internally from the components above. It is very unlikely
that the user will need to call them.

.. _mj_sensorPos:

mj_sensorPos
~~~~~~~~~~~~

.. mujoco-include:: mj_sensorPos

Evaluate position-dependent sensors.

.. _mj_sensorVel:

mj_sensorVel
~~~~~~~~~~~~

.. mujoco-include:: mj_sensorVel

Evaluate velocity-dependent sensors.

.. _mj_sensorAcc:

mj_sensorAcc
~~~~~~~~~~~~

.. mujoco-include:: mj_sensorAcc

Evaluate acceleration and force-dependent sensors.

.. _mj_energyPos:

mj_energyPos
~~~~~~~~~~~~

.. mujoco-include:: mj_energyPos

Evaluate position-dependent energy (potential).

.. _mj_energyVel:

mj_energyVel
~~~~~~~~~~~~

.. mujoco-include:: mj_energyVel

Evaluate velocity-dependent energy (kinetic).

.. _mj_checkPos:

mj_checkPos
~~~~~~~~~~~

.. mujoco-include:: mj_checkPos

Check qpos, reset if any element is too big or nan.

.. _mj_checkVel:

mj_checkVel
~~~~~~~~~~~

.. mujoco-include:: mj_checkVel

Check qvel, reset if any element is too big or nan.

.. _mj_checkAcc:

mj_checkAcc
~~~~~~~~~~~

.. mujoco-include:: mj_checkAcc

Check qacc, reset if any element is too big or nan.

.. _mj_kinematics:

mj_kinematics
~~~~~~~~~~~~~

.. mujoco-include:: mj_kinematics

Run forward kinematics.

.. _mj_comPos:

mj_comPos
~~~~~~~~~

.. mujoco-include:: mj_comPos

Map inertias and motion dofs to global frame centered at CoM.

mj_camlight
~~~~~~~~~~~

.. mujoco-include:: mj_camlight

Compute camera and light positions and orientations.

mj_tendon
~~~~~~~~~

.. mujoco-include:: mj_tendon

Compute tendon lengths, velocities and moment arms.

mj_transmission
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_transmission

Compute actuator transmission lengths and moments.

mj_crb
~~~~~~

.. mujoco-include:: mj_crb

Run composite rigid body inertia algorithm (CRB).

.. _mj_factorM:

mj_factorM
~~~~~~~~~~

.. mujoco-include:: mj_factorM

Compute sparse :math:`L^T D L` factorizaton of inertia matrix.

.. _mj_solveM:

mj_solveM
~~~~~~~~~

.. mujoco-include:: mj_solveM

Solve linear system :math:`M x = y` using factorization: :math:`x = (L^T D L)^{-1} y`

.. _mj_solveM2:

mj_solveM2
~~~~~~~~~~

.. mujoco-include:: mj_solveM2

Half of linear solve: :math:`x = \sqrt{D^{-1}} (L^T)^{-1} y`

.. _mj_comVel:

mj_comVel
~~~~~~~~~

.. mujoco-include:: mj_comVel

Compute cvel, cdof_dot.

mj_passive
~~~~~~~~~~

.. mujoco-include:: mj_passive

Compute qfrc_passive from spring-dampers, viscosity and density.

.. _mj_subtreeVel:

mj_subtreeVel
~~~~~~~~~~~~~

.. mujoco-include:: mj_subtreeVel

subtree linear velocity and angular momentum

mj_rne
~~~~~~

.. mujoco-include:: mj_rne

RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term.

.. _mj_rnePostConstraint:

mj_rnePostConstraint
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_rnePostConstraint

RNE with complete data: compute cacc, cfrc_ext, cfrc_int.

.. _mj_collision:

mj_collision
~~~~~~~~~~~~

.. mujoco-include:: mj_collision

Run collision detection.

.. _mj_makeConstraint:

mj_makeConstraint
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_makeConstraint

Construct constraints.

.. _mj_projectConstraint:

mj_projectConstraint
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_projectConstraint

Compute inverse constraint inertia efc_AR.

.. _mj_referenceConstraint:

mj_referenceConstraint
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_referenceConstraint

Compute efc_vel, efc_aref.

. _mj_constraintUpdate:

mj_constraintUpdate
~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_constraintUpdate

Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians. If cost is not NULL, set \*cost = s(jar)
where jar = Jac*qacc-aref.

.. _Support:

Support
^^^^^^^

These are support functions that need access to mjModel and mjData, unlike the utility functions which do not need such
access. Support functions are called within the simulator but some of them can also be useful for custom computations,
and are documented in more detail below.

.. _mj_addContact:

mj_addContact
~~~~~~~~~~~~~

.. mujoco-include:: mj_addContact

Add contact to d->contact list; return 0 if success; 1 if buffer full.

.. _mj_isPyramidal:

mj_isPyramidal
~~~~~~~~~~~~~~

.. mujoco-include:: mj_isPyramidal

Determine type of friction cone.

.. _mj_isSparse:

mj_isSparse
~~~~~~~~~~~

.. mujoco-include:: mj_isSparse

Determine type of constraint Jacobian.

.. _mj_isDual:

mj_isDual
~~~~~~~~~

.. mujoco-include:: mj_isDual

Determine type of solver (PGS is dual, CG and Newton are primal).

.. _mj_mulJacVec:

mj_mulJacVec
~~~~~~~~~~~~

.. mujoco-include:: mj_mulJacVec

This function multiplies the constraint Jacobian mjData.efc_J by a vector. Note that the Jacobian can be either dense or
sparse; the function is aware of this setting. Multiplication by J maps velocities from joint space to constraint space.

.. _mj_mulJacTVec:

mj_mulJacTVec
~~~~~~~~~~~~~

.. mujoco-include:: mj_mulJacTVec

Same as mj_mulJacVec but multiplies by the transpose of the Jacobian. This maps forces from constraint space to joint
space.

.. _mj_jac:

mj_jac
~~~~~~

.. mujoco-include:: mj_jac

This function computes an "end-effector" Jacobian, which is unrelated to the constraint Jacobian above. Any MuJoCo body
can be treated as end-effector, and the point for which the Jacobian is computed can be anywhere in space (it is treated
as attached to the body). The Jacobian has translational (jacp) and rotational (jacr) components. Passing NULL for
either pointer will skip part of the computation. Each component is a 3-by-nv matrix. Each row of this matrix is the
gradient of the corresponding 3D coordinate of the specified point with respect to the degrees of freedom. The ability
to compute end-effector Jacobians analytically is one of the advantages of working in minimal coordinates - so use it!

.. _mj_jacBody:

mj_jacBody
~~~~~~~~~~

.. mujoco-include:: mj_jacBody

This and the remaining variants of the Jacobian function call mj_jac internally, with the center of the body, geom or
site. They are just shortcuts; the same can be achieved by calling mj_jac directly.

.. _mj_jacBodyCom:

mj_jacBodyCom
~~~~~~~~~~~~~

.. mujoco-include:: mj_jacBodyCom

Compute body center-of-mass end-effector Jacobian.

.. _mj_jacSubtreeCom:

mj_jacSubtreeCom
~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jacSubtreeCom

Compute subtree center-of-mass end-effector Jacobian. ``jacp`` is 3 x nv.

.. _mj_jacGeom:

mj_jacGeom
~~~~~~~~~~

.. mujoco-include:: mj_jacGeom

Compute geom end-effector Jacobian.

.. _mj_jacSite:

mj_jacSite
~~~~~~~~~~

.. mujoco-include:: mj_jacSite

Compute site end-effector Jacobian.

.. _mj_jacPointAxis:

mj_jacPointAxis
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jacPointAxis

Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.

.. _mj_name2id:

mj_name2id
~~~~~~~~~~

.. mujoco-include:: mj_name2id

Get id of object with specified name, return -1 if not found; type is mjtObj.

.. _mj_id2name:

mj_id2name
~~~~~~~~~~

.. mujoco-include:: mj_id2name

Get name of object with specified id, return 0 if invalid type or id; type is mjtObj.

.. _mj_fullM:

mj_fullM
~~~~~~~~

.. mujoco-include:: mj_fullM

Convert sparse inertia matrix M into full (i.e. dense) matrix.

.. _mj_mulM:

mj_mulM
~~~~~~~

.. mujoco-include:: mj_mulM

This function multiplies the joint-space inertia matrix stored in mjData.qM by a vector. qM has a custom sparse format
that the user should not attempt to manipulate directly. Alternatively one can convert qM to a dense matrix with
mj_fullM and then user regular matrix-vector multiplication, but this is slower because it no longer benefits from
sparsity.

.. _mj_mulM2:

mj_mulM2
~~~~~~~~

.. mujoco-include:: mj_mulM2

Multiply vector by (inertia matrix)^(1/2).

.. _mj_addM:

mj_addM
~~~~~~~

.. mujoco-include:: mj_addM

Add inertia matrix to destination matrix. Destination can be sparse uncompressed, or dense when all int\* are NULL

.. _mj_applyFT:

mj_applyFT
~~~~~~~~~~

.. mujoco-include:: mj_applyFT

This function can be used to apply a Cartesian force and torque to a point on a body, and add the result to the vector
mjData.qfrc_applied of all applied forces. Note that the function requires a pointer to this vector, because sometimes
we want to add the result to a different vector.

.. _mj_objectVelocity:

mj_objectVelocity
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_objectVelocity

Compute object 6D velocity in object-centered frame, world/local orientation.

.. _mj_objectAcceleration:

mj_objectAcceleration
~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_objectAcceleration

Compute object 6D acceleration in object-centered frame, world/local orientation.

.. _mj_contactForce:

mj_contactForce
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_contactForce

Extract 6D force:torque given contact id, in the contact frame.

.. _mj_differentiatePos:

mj_differentiatePos
~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_differentiatePos

This function subtracts two vectors in the format of qpos (and divides the result by dt), while respecting the
properties of quaternions. Recall that unit quaternions represent spatial orientations. They are points on the unit
sphere in 4D. The tangent to that sphere is a 3D plane of rotational velocities. Thus when we subtract two quaternions
in the right way, the result is a 3D vector and not a 4D vector. This the output qvel has dimensionality nv while the
inputs have dimensionality nq.

.. _mj_integratePos:

mj_integratePos
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_integratePos

This is the opposite of mj_differentiatePos. It adds a vector in the format of qvel (scaled by dt) to a vector in the
format of qpos.

.. _mj_normalizeQuat:

mj_normalizeQuat
~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_normalizeQuat

Normalize all quaternions in qpos-type vector.

.. _mj_local2Global:

mj_local2Global
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_local2Global

Map from body local to global Cartesian coordinates.

.. _mj_getTotalmass:

mj_getTotalmass
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_getTotalmass

Sum all body masses.

.. _mj_setTotalmass:

mj_setTotalmass
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_setTotalmass

Scale body masses and inertias to achieve specified total mass.

.. _mj_version:

mj_version
~~~~~~~~~~

.. mujoco-include:: mj_version

Return version number: 1.0.2 is encoded as 102.

.. mj_versionString:

mj_versionString
~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_versionString

Return the current version of MuJoCo as a null-terminated string.

.. _Raycollisions:

Ray collisions
^^^^^^^^^^^^^^

Ray collision functionality was added in MuJoCo 1.50. This is a new collision detection module that uses analytical
formulas to intersect a ray (p + x*v, x>=0) with a geom, where p is the origin of the ray and v is the vector specifying
the direction. All functions in this family return the distance to the nearest geom surface, or -1 if there is no
intersection. Note that if p is inside a geom, the ray will intersect the surface from the inside which still counts as
an intersection.

All ray collision functions rely on quantities computed by :ref:`mj_kinematics` (see :ref:`mjData`), so must be called
after  :ref:`mj_kinematics`, or functions that call it (e.g. :ref:`mj_fwdPosition`).

.. _mj_ray:

mj_ray
~~~~~~

.. mujoco-include:: mj_ray

Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude. Return geomid and distance (x) to
nearest surface, or -1 if no intersection.

geomgroup is an array of length mjNGROUP, where 1 means the group should be included. Pass geomgroup=NULL to skip
group exclusion.
If flg_static is 0, static geoms will be excluded.
bodyexclude=-1 can be used to indicate that all bodies are included.

.. _mj_rayHfield:

mj_rayHfield
~~~~~~~~~~~~

.. mujoco-include:: mj_rayHfield

Interect ray with hfield, return nearest distance or -1 if no intersection.

.. _mj_rayMesh:

mj_rayMesh
~~~~~~~~~~

.. mujoco-include:: mj_rayMesh

Interect ray with mesh, return nearest distance or -1 if no intersection.

.. _mju_rayGeom:

mju_rayGeom
~~~~~~~~~~~

.. mujoco-include:: mju_rayGeom

Interect ray with pure geom, return nearest distance or -1 if no intersection.

.. _mju_raySkin:

mju_raySkin
~~~~~~~~~~~

.. mujoco-include:: mju_raySkin

Interect ray with skin, return nearest vertex id.

.. _Interaction:

Interaction
^^^^^^^^^^^

These function implement abstract mouse interactions, allowing control over cameras and perturbations. Their use is well
illustrated in :ref:`simulate.cc <saSimulate>`.

.. _mjv_defaultCamera:

mjv_defaultCamera
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultCamera

Set default camera.

.. _mjv_defaultFreeCamera:

mjv_defaultFreeCamera
~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultFreeCamera

Set default free camera.

.. _mjv_defaultPerturb:

mjv_defaultPerturb
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultPerturb

Set default perturbation.

.. _mjv_room2model:

mjv_room2model
~~~~~~~~~~~~~~

.. mujoco-include:: mjv_room2model

Transform pose from room to model space.

.. _mjv_model2room:

mjv_model2room
~~~~~~~~~~~~~~

.. mujoco-include:: mjv_model2room

Transform pose from model to room space.

.. _mjv_cameraInModel:

mjv_cameraInModel
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_cameraInModel

Get camera info in model space; average left and right OpenGL cameras.

.. _mjv_cameraInRoom:

mjv_cameraInRoom
~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_cameraInRoom

Get camera info in room space; average left and right OpenGL cameras.

.. _mjv_frustumHeight:

mjv_frustumHeight
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_frustumHeight

Get frustum height at unit distance from camera; average left and right OpenGL cameras.

.. _mjv_alignToCamera:

mjv_alignToCamera
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_alignToCamera

Rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y).

.. _mjv_moveCamera:

mjv_moveCamera
~~~~~~~~~~~~~~

.. mujoco-include:: mjv_moveCamera

Move camera with mouse; action is mjtMouse.

.. _mjv_movePerturb:

mjv_movePerturb
~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_movePerturb

Move perturb object with mouse; action is mjtMouse.

.. _mjv_moveModel:

mjv_moveModel
~~~~~~~~~~~~~

.. mujoco-include:: mjv_moveModel

Move model with mouse; action is mjtMouse.

.. _mjv_initPerturb:

mjv_initPerturb
~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_initPerturb

Copy perturb pos,quat from selected body; set scale for perturbation.

.. _mjv_applyPerturbPose:

mjv_applyPerturbPose
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_applyPerturbPose

Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise. Write d->qpos only if flg_paused
and subtree root for selected body has free joint.

.. _mjv_applyPerturbForce:

mjv_applyPerturbForce
~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_applyPerturbForce

Set perturb force,torque in d->xfrc_applied, if selected body is dynamic.

.. _mjv_averageCamera:

mjv_averageCamera
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_averageCamera

Return the average of two OpenGL cameras.

.. _mjv_select:

mjv_select
~~~~~~~~~~

.. mujoco-include:: mjv_select

This function is used for mouse selection. Previously selection was done via OpenGL, but as of MuJoCo 1.50 it relies on
ray intersections which are much more efficient. aspectratio is the viewport width/height. relx and rely are the
relative coordinates of the 2D point of interest in the viewport (usually mouse cursor). The function returns the id of
the geom under the specified 2D point, or -1 if there is no geom (note that they skybox if present is not a model geom).
The 3D coordinates of the clicked point are returned in selpnt. See :ref:`simulate.cc <saSimulate>` for an illustration.

.. _Visualization-api:

Visualization
^^^^^^^^^^^^^

The functions in this section implement abstract visualization. The results are used by the OpenGL rendered, and can
also be used by users wishing to implement their own rendered, or hook up MuJoCo to advanced rendering tools such as
Unity or Unreal Engine. See :ref:`simulate.cc <saSimulate>` for illustration of how to use these functions.

.. _mjv_defaultOption:

mjv_defaultOption
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultOption

Set default visualization options.

.. _mjv_defaultFigure:

mjv_defaultFigure
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultFigure

Set default figure.

.. _mjv_initGeom:

mjv_initGeom
~~~~~~~~~~~~

.. mujoco-include:: mjv_initGeom

Initialize given geom fields when not NULL, set the rest to their default values.

.. _mjv_makeConnector:

mjv_makeConnector
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_makeConnector

Set (type, size, pos, mat) for connector-type geom between given points. Assume that mjv_initGeom was already called to
set all other properties.

.. _mjv_defaultScene:

mjv_defaultScene
~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultScene

Set default abstract scene.

.. _mjv_makeScene:

mjv_makeScene
~~~~~~~~~~~~~

.. mujoco-include:: mjv_makeScene

Allocate resources in abstract scene.

.. _mjv_freeScene:

mjv_freeScene
~~~~~~~~~~~~~

.. mujoco-include:: mjv_freeScene

Free abstract scene.

.. _mjv_updateScene:

mjv_updateScene
~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_updateScene

Update entire scene given model state.

.. _mjv_addGeoms:

mjv_addGeoms
~~~~~~~~~~~~

.. mujoco-include:: mjv_addGeoms

Add geoms from selected categories.

.. _mjv_makeLights:

mjv_makeLights
~~~~~~~~~~~~~~

.. mujoco-include:: mjv_makeLights

Make list of lights.

.. _mjv_updateCamera:

mjv_updateCamera
~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_updateCamera

Update camera.

.. _mjv_updateSkin:

mjv_updateSkin
~~~~~~~~~~~~~~

.. mujoco-include:: mjv_updateSkin

Update skins.

.. _OpenGLrendering:

OpenGL rendering
^^^^^^^^^^^^^^^^

These functions expose the OpenGL renderer. See :ref:`simulate.cc <saSimulate>` for an illustration
of how to use these functions.

.. _mjr_defaultContext:

mjr_defaultContext
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_defaultContext

Set default mjrContext.

.. _mjr_makeContext:

mjr_makeContext
~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_makeContext

Allocate resources in custom OpenGL context; fontscale is mjtFontScale.

.. _mjr_changeFont:

mjr_changeFont
~~~~~~~~~~~~~~

.. mujoco-include:: mjr_changeFont

Change font of existing context.

.. _mjr_addAux:

mjr_addAux
~~~~~~~~~~

.. mujoco-include:: mjr_addAux

Add Aux buffer with given index to context; free previous Aux buffer.

.. _mjr_freeContext:

mjr_freeContext
~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_freeContext

Free resources in custom OpenGL context, set to default.

.. _mjr_uploadTexture:

mjr_uploadTexture
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_uploadTexture

Upload texture to GPU, overwriting previous upload if any.

.. _mjr_uploadMesh:

mjr_uploadMesh
~~~~~~~~~~~~~~

.. mujoco-include:: mjr_uploadMesh

Upload mesh to GPU, overwriting previous upload if any.

.. _mjr_uploadHField:

mjr_uploadHField
~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_uploadHField

Upload height field to GPU, overwriting previous upload if any.

.. _mjr_restoreBuffer:

mjr_restoreBuffer
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_restoreBuffer

Make con->currentBuffer current again.

.. _mjr_setBuffer:

mjr_setBuffer
~~~~~~~~~~~~~

.. mujoco-include:: mjr_setBuffer

Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN. If only one buffer is available, set that buffer
and ignore framebuffer argument.

.. _mjr_readPixels:

mjr_readPixels
~~~~~~~~~~~~~~

.. mujoco-include:: mjr_readPixels

Read pixels from current OpenGL framebuffer to client buffer. Viewport is in OpenGL framebuffer; client buffer starts at
(0,0).

.. _mjr_drawPixels:

mjr_drawPixels
~~~~~~~~~~~~~~

.. mujoco-include:: mjr_drawPixels

Draw pixels from client buffer to current OpenGL framebuffer. Viewport is in OpenGL framebuffer; client buffer starts at
(0,0).

.. _mjr_blitBuffer:

mjr_blitBuffer
~~~~~~~~~~~~~~

.. mujoco-include:: mjr_blitBuffer

Blit from src viewpoint in current framebuffer to dst viewport in other framebuffer. If src, dst have different size and
flg_depth==0, color is interpolated with GL_LINEAR.

.. _mjr_setAux:

mjr_setAux
~~~~~~~~~~

.. mujoco-include:: mjr_setAux

Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done).

.. _mjr_blitAux:

mjr_blitAux
~~~~~~~~~~~

.. mujoco-include:: mjr_blitAux

Blit from Aux buffer to con->currentBuffer.

.. _mjr_text:

mjr_text
~~~~~~~~

.. mujoco-include:: mjr_text

Draw text at (x,y) in relative coordinates; font is mjtFont.

.. _mjr_overlay:

mjr_overlay
~~~~~~~~~~~

.. mujoco-include:: mjr_overlay

Draw text overlay; font is mjtFont; gridpos is mjtGridPos.

.. _mjr_maxViewport:

mjr_maxViewport
~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_maxViewport

Get maximum viewport for active buffer.

.. _mjr_rectangle:

mjr_rectangle
~~~~~~~~~~~~~

.. mujoco-include:: mjr_rectangle

Draw rectangle.

.. _mjr_label:

mjr_label
~~~~~~~~~~~~~

.. mujoco-include:: mjr_label

Draw rectangle with centered text.

.. _mjr_figure:

mjr_figure
~~~~~~~~~~

.. mujoco-include:: mjr_figure

Draw 2D figure.

.. _mjr_render:

mjr_render
~~~~~~~~~~

.. mujoco-include:: mjr_render

Render 3D scene.

.. _mjr_finish:

mjr_finish
~~~~~~~~~~

.. mujoco-include:: mjr_finish

Call glFinish.

.. _mjr_getError:

mjr_getError
~~~~~~~~~~~~

.. mujoco-include:: mjr_getError

Call glGetError and return result.

.. _mjr_findRect:

mjr_findRect
~~~~~~~~~~~~

.. mujoco-include:: mjr_findRect

Find first rectangle containing mouse, -1: not found.

.. _UIframework:

UI framework
^^^^^^^^^^^^

.. _mjui_themeSpacing:

mjui_themeSpacing
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_themeSpacing

Get builtin UI theme spacing (ind: 0-1).

.. _mjui_themeColor:

mjui_themeColor
~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_themeColor

Get builtin UI theme color (ind: 0-3).

mjui_add
~~~~~~~~

.. mujoco-include:: mjui_add

Add definitions to UI.

mjui_addToSection
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_addToSection

Add definitions to UI section.

mjui_resize
~~~~~~~~~~~

.. mujoco-include:: mjui_resize

Compute UI sizes.

.. _mjui_update:

mjui_update
~~~~~~~~~~~

.. mujoco-include:: mjui_update

Update specific section/item; -1: update all.

mjui_event
~~~~~~~~~~

.. mujoco-include:: mjui_event

Handle UI event, return pointer to changed item, NULL if no change.

mjui_render
~~~~~~~~~~~

.. mujoco-include:: mjui_render

Copy UI image to current buffer.

.. _Errorandmemory:

Error and memory
^^^^^^^^^^^^^^^^

.. _mju_error:

mju_error
~~~~~~~~~

.. mujoco-include:: mju_error

Main error function; does not return to caller.

.. _mju_error_i:

mju_error_i
~~~~~~~~~~~

.. mujoco-include:: mju_error_i

Error function with int argument; msg is a printf format string.

.. _mju_error_s:

mju_error_s
~~~~~~~~~~~

.. mujoco-include:: mju_error_s

Error function with string argument.

.. _mju_warning:

mju_warning
~~~~~~~~~~~

.. mujoco-include:: mju_warning

Main warning function; returns to caller.

mju_warning_i
~~~~~~~~~~~~~

.. mujoco-include:: mju_warning_i

Warning function with int argument.

mju_warning_s
~~~~~~~~~~~~~

.. mujoco-include:: mju_warning_s

Warning function with string argument.

.. _mju_clearHandlers:

mju_clearHandlers
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_clearHandlers

Clear user error and memory handlers.

.. _mju_malloc:

mju_malloc
~~~~~~~~~~

.. mujoco-include:: mju_malloc

Allocate memory; byte-align on 64; pad size to multiple of 64.

.. _mju_free:

mju_free
~~~~~~~~

.. mujoco-include:: mju_free

Free memory, using free() by default.

.. _mj_warning:

mj_warning
~~~~~~~~~~

.. mujoco-include:: mj_warning

High-level warning function: count warnings in mjData, print only the first.

.. _mju_writeLog:

mju_writeLog
~~~~~~~~~~~~

.. mujoco-include:: mju_writeLog

Write [datetime, type: message] to MUJOCO_LOG.TXT.

.. _Standardmath:

Standard math
^^^^^^^^^^^^^

The "functions" in this section are preprocessor macros replaced with the corresponding C standard library math
functions. When MuJoCo is compiled with single precision (which is not currently available to the public, but we
sometimes use it internally) these macros are replaced with the corresponding single-precision functions (not shown
here). So one can think of them as having inputs and outputs of type mjtNum, where mjtNum is defined as double or float
depending on how MuJoCo is compiled. We will not document these functions here; see the C standard library
specification.

mju_sqrt
~~~~~~~~

.. code-block:: C

   #define mju_sqrt    sqrt

mju_exp
~~~~~~~

.. code-block:: C

   #define mju_exp     exp

mju_sin
~~~~~~~

.. code-block:: C

   #define mju_sin     sin

mju_cos
~~~~~~~

.. code-block:: C

   #define mju_cos     cos

mju_tan
~~~~~~~

.. code-block:: C

   #define mju_tan     tan

mju_asin
~~~~~~~~

.. code-block:: C

   #define mju_asin    asin

mju_acos
~~~~~~~~

.. code-block:: C

   #define mju_acos    acos

mju_atan2
~~~~~~~~~

.. code-block:: C

   #define mju_atan2   atan2

mju_tanh
~~~~~~~~

.. code-block:: C

   #define mju_tanh    tanh

mju_pow
~~~~~~~

.. code-block:: C

   #define mju_pow     pow

mju_abs
~~~~~~~

.. code-block:: C

   #define mju_abs     fabs

mju_log
~~~~~~~

.. code-block:: C

   #define mju_log     log

mju_log10
~~~~~~~~~

.. code-block:: C

   #define mju_log10   log10

mju_floor
~~~~~~~~~

.. code-block:: C

   #define mju_floor   floor

mju_ceil
~~~~~~~~

.. code-block:: C

   #define mju_ceil    ceil

.. _Vectormath:

Vector math
^^^^^^^^^^^

mju_zero3
~~~~~~~~~

.. mujoco-include:: mju_zero3

Set res = 0.

mju_copy3
~~~~~~~~~

.. mujoco-include:: mju_copy3

Set res = vec.

mju_scl3
~~~~~~~~

.. mujoco-include:: mju_scl3

Set res = vec*scl.

mju_add3
~~~~~~~~

.. mujoco-include:: mju_add3

Set res = vec1 + vec2.

mju_sub3
~~~~~~~~

.. mujoco-include:: mju_sub3

Set res = vec1 - vec2.

.. _mju_addTo3:

mju_addTo3
~~~~~~~~~~

.. mujoco-include:: mju_addTo3

Set res = res + vec.

.. _mju_subFrom3:

mju_subFrom3
~~~~~~~~~~~~

.. mujoco-include:: mju_subFrom3

Set res = res - vec.

.. _mju_addToScl3:

mju_addToScl3
~~~~~~~~~~~~~

.. mujoco-include:: mju_addToScl3

Set res = res + vec*scl.

.. _mju_addScl3:

mju_addScl3
~~~~~~~~~~~

.. mujoco-include:: mju_addScl3

Set res = vec1 + vec2*scl.

mju_normalize3
~~~~~~~~~~~~~~

.. mujoco-include:: mju_normalize3

Normalize vector, return length before normalization.

mju_norm3
~~~~~~~~~

.. mujoco-include:: mju_norm3

Return vector length (without normalizing the vector).

mju_dot3
~~~~~~~~

.. mujoco-include:: mju_dot3

Return dot-product of vec1 and vec2.

mju_dist3
~~~~~~~~~

.. mujoco-include:: mju_dist3

Return Cartesian distance between 3D vectors pos1 and pos2.

.. _mju_rotVecMat:

mju_rotVecMat
~~~~~~~~~~~~~

.. mujoco-include:: mju_rotVecMat

Multiply vector by 3D rotation matrix: res = mat \* vec.

.. _mju_rotVecMatT:

mju_rotVecMatT
~~~~~~~~~~~~~~

.. mujoco-include:: mju_rotVecMatT

Multiply vector by transposed 3D rotation matrix: res = mat' \* vec.

mju_cross
~~~~~~~~~

.. mujoco-include:: mju_cross

Compute cross-product: res = cross(a, b).

mju_zero4
~~~~~~~~~

.. mujoco-include:: mju_zero4

Set res = 0.

mju_unit4
~~~~~~~~~

.. mujoco-include:: mju_unit4

Set res = (1,0,0,0).

mju_copy4
~~~~~~~~~

.. mujoco-include:: mju_copy4

Set res = vec.

mju_normalize4
~~~~~~~~~~~~~~

.. mujoco-include:: mju_normalize4

Normalize vector, return length before normalization.

mju_zero
~~~~~~~~

.. mujoco-include:: mju_zero

Set res = 0.

.. _mju_fill:

mju_fill
~~~~~~~~

.. mujoco-include:: mju_fill

Set res = val.

mju_copy
~~~~~~~~

.. mujoco-include:: mju_copy

Set res = vec.

mju_sum
~~~~~~~

.. mujoco-include:: mju_sum

Return sum(vec).

.. _mju_L1:

mju_L1
~~~~~~

.. mujoco-include:: mju_L1

Return L1 norm: sum(abs(vec)).

.. _mju_scl:

mju_scl
~~~~~~~

.. mujoco-include:: mju_scl

Set res = vec*scl.

mju_add
~~~~~~~

.. mujoco-include:: mju_add

Set res = vec1 + vec2.

mju_sub
~~~~~~~

.. mujoco-include:: mju_sub

Set res = vec1 - vec2.

.. _mju_addTo:

mju_addTo
~~~~~~~~~

.. mujoco-include:: mju_addTo

Set res = res + vec.

.. _mju_subFrom:

mju_subFrom
~~~~~~~~~~~

.. mujoco-include:: mju_subFrom

Set res = res - vec.

.. _mju_addToScl:

mju_addToScl
~~~~~~~~~~~~

.. mujoco-include:: mju_addToScl

Set res = res + vec*scl.

.. _mju_addScl:

mju_addScl
~~~~~~~~~~

.. mujoco-include:: mju_addScl

Set res = vec1 + vec2*scl.

mju_normalize
~~~~~~~~~~~~~

.. mujoco-include:: mju_normalize

Normalize vector, return length before normalization.

mju_norm
~~~~~~~~

.. mujoco-include:: mju_norm

Return vector length (without normalizing vector).

mju_dot
~~~~~~~

.. mujoco-include:: mju_dot

Return dot-product of vec1 and vec2.

.. _mju_mulMatVec:

mju_mulMatVec
~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatVec

Multiply matrix and vector: res = mat \* vec.

.. _mju_mulMatTVec:

mju_mulMatTVec
~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatTVec

Multiply transposed matrix and vector: res = mat' \* vec.

.. _mju_mulVecMatVec:

mju_mulVecMatVec
~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulVecMatVec

Multiply square matrix with vectors on both sides: return vec1' \* mat \* vec2.

mju_transpose
~~~~~~~~~~~~~

.. mujoco-include:: mju_transpose

Transpose matrix: res = mat'.

.. _mju_symmetrize:

mju_symmetrize
~~~~~~~~~~~~~~

.. mujoco-include:: mju_symmetrize

Symmetrize square matrix :math:`R = \frac{1}{2}(M + M^T)`.

.. _mju_eye:

mju_eye
~~~~~~~

.. mujoco-include:: mju_eye

Set mat to the identity matrix.

.. _mju_mulMatMat:

mju_mulMatMat
~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatMat

Multiply matrices: res = mat1 \* mat2.

.. _mju_mulMatMatT:

mju_mulMatMatT
~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatMatT

Multiply matrices, second argument transposed: res = mat1 \* mat2'.

.. _mju_mulMatTMat:

mju_mulMatTMat
~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatTMat

Multiply matrices, first argument transposed: res = mat1' \* mat2.

.. _mju_sqrMatTD:

mju_sqrMatTD
~~~~~~~~~~~~

.. mujoco-include:: mju_sqrMatTD

Set res = mat' \* diag \* mat if diag is not NULL, and res = mat' \* mat otherwise.

.. _mju_transformSpatial:

mju_transformSpatial
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_transformSpatial

Coordinate transform of 6D motion or force vector in rotation:translation format. rotnew2old is 3-by-3, NULL means no
rotation; flg_force specifies force or motion type.

.. _Quaternions:

Quaternions
^^^^^^^^^^^

.. _mju_rotVecQuat:

mju_rotVecQuat
~~~~~~~~~~~~~~

.. mujoco-include:: mju_rotVecQuat

Rotate vector by quaternion.

.. _mju_negQuat:

mju_negQuat
~~~~~~~~~~~

.. mujoco-include:: mju_negQuat

Negate quaternion.

.. _mju_mulQuat:

mju_mulQuat
~~~~~~~~~~~

.. mujoco-include:: mju_mulQuat

Multiply quaternions.

.. _mju_mulQuatAxis:

mju_mulQuatAxis
~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulQuatAxis

Multiply quaternion and axis.

.. _mju_axisAngle2Quat:

mju_axisAngle2Quat
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_axisAngle2Quat

Convert axisAngle to quaternion.

.. _mju_quat2Vel:

mju_quat2Vel
~~~~~~~~~~~~

.. mujoco-include:: mju_quat2Vel

Convert quaternion (corresponding to orientation difference) to 3D velocity.

.. _mju_subQuat:

mju_subQuat
~~~~~~~~~~~

.. mujoco-include:: mju_subQuat

Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.

.. _mju_quat2Mat:

mju_quat2Mat
~~~~~~~~~~~~

.. mujoco-include:: mju_quat2Mat

Convert quaternion to 3D rotation matrix.

.. _mju_mat2Quat:

mju_mat2Quat
~~~~~~~~~~~~

.. mujoco-include:: mju_mat2Quat

Convert 3D rotation matrix to quaternion.

.. _mju_derivQuat:

mju_derivQuat
~~~~~~~~~~~~~

.. mujoco-include:: mju_derivQuat

Compute time-derivative of quaternion, given 3D rotational velocity.

.. _mju_quatIntegrate:

mju_quatIntegrate
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_quatIntegrate

Integrate quaternion given 3D angular velocity.

.. _mju_quatZ2Vec:

mju_quatZ2Vec
~~~~~~~~~~~~~

.. mujoco-include:: mju_quatZ2Vec

Construct quaternion performing rotation from z-axis to given vector.

.. _Poses:

Poses
^^^^^

.. _mju_mulPose:

mju_mulPose
~~~~~~~~~~~

.. mujoco-include:: mju_mulPose

Multiply two poses.

.. _mju_negPose:

mju_negPose
~~~~~~~~~~~

.. mujoco-include:: mju_negPose

Negate pose.

.. _mju_trnVecPose:

mju_trnVecPose
~~~~~~~~~~~~~~

.. mujoco-include:: mju_trnVecPose

Transform vector by pose.

.. _Decompositions:

Decompositions
^^^^^^^^^^^^^^

.. _mju_cholFactor:

mju_cholFactor
~~~~~~~~~~~~~~

.. mujoco-include:: mju_cholFactor

Cholesky decomposition: mat = L*L'; return rank, decomposition performed in-place into mat.

.. _mju_cholSolve:

mju_cholSolve
~~~~~~~~~~~~~

.. mujoco-include:: mju_cholSolve

Solve mat \* res = vec, where mat is Cholesky-factorized

.. _mju_cholUpdate:

mju_cholUpdate
~~~~~~~~~~~~~~

.. mujoco-include:: mju_cholUpdate

Cholesky rank-one update: L*L' +/- x*x'; return rank.

mju_eig3
~~~~~~~~

.. mujoco-include:: mju_eig3

Eigenvalue decomposition of symmetric 3x3 matrix.

.. _mju_boxQP:

mju_boxQP
~~~~~~~~~

.. mujoco-include:: mju_boxQP

Minimize :math:`\tfrac{1}{2} x^T H x + x^T g \quad \text{s.t.} \quad l \le x \le u`, return rank or -1 if failed.

inputs:
  ``n``           - problem dimension

  ``H``           - SPD matrix                ``n*n``

  ``g``           - bias vector               ``n``

  ``lower``       - lower bounds              ``n``

  ``upper``       - upper bounds              ``n``

  ``res``         - solution warmstart        ``n``

return value:
  ``nfree <= n``  - rank of unconstrained subspace, -1 if failure

outputs (required):
  ``res``         - solution                  ``n``

  ``R``           - subspace Cholesky factor  ``nfree*nfree``,    allocated: ``n*(n+7)``

outputs (optional):
  ``index``       - set of free dimensions    ``nfree``,          allocated: ``n``

notes:
  The initial value of ``res`` is used to warmstart the solver.
  ``R`` must have allocatd size ``n*(n+7)``, but only ``nfree*nfree`` values are used in output.
  ``index`` (if given) must have allocated size ``n``, but only ``nfree`` values are used in output.
  The convenience function :ref:`mju_boxQPmalloc` allocates the required data structures.
  Only the lower triangles of H and R and are read from and written to, respectively.

.. _mju_boxQPmalloc:

mju_boxQPmalloc
~~~~~~~~~~~~~~~

.. mujoco-include:: mju_boxQPmalloc

Allocate heap memory for box-constrained Quadratic Program.
As in :ref:`mju_boxQP`, ``index``, ``lower``, and ``upper`` are optional.
Free all pointers with ``mju_free()``.

.. _Miscellaneous:

Miscellaneous
^^^^^^^^^^^^^

.. _mju_muscleGain:

mju_muscleGain
~~~~~~~~~~~~~~

.. mujoco-include:: mju_muscleGain

Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).

.. _mju_muscleBias:

mju_muscleBias
~~~~~~~~~~~~~~

.. mujoco-include:: mju_muscleBias

Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).

.. _mju_muscleDynamics:

mju_muscleDynamics
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_muscleDynamics

Muscle activation dynamics, prm = (tau_act, tau_deact).

.. _mju_encodePyramid:

mju_encodePyramid
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_encodePyramid

Convert contact force to pyramid representation.

.. _mju_decodePyramid:

mju_decodePyramid
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_decodePyramid

Convert pyramid representation to contact force.

.. _mju_springDamper:

mju_springDamper
~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_springDamper

Integrate spring-damper analytically, return pos(dt).

.. _mju_min:

mju_min
~~~~~~~

.. mujoco-include:: mju_min

Return min(a,b) with single evaluation of a and b.

.. _mju_max:

mju_max
~~~~~~~

.. mujoco-include:: mju_max

Return max(a,b) with single evaluation of a and b.

mju_sign
~~~~~~~~

.. mujoco-include:: mju_sign

Return sign of x: +1, -1 or 0.

mju_round
~~~~~~~~~

.. mujoco-include:: mju_round

Round x to nearest integer.

.. _mju_type2Str:

mju_type2Str
~~~~~~~~~~~~

.. mujoco-include:: mju_type2Str

Convert type id (mjtObj) to type name.

.. _mju_str2Type:

mju_str2Type
~~~~~~~~~~~~

.. mujoco-include:: mju_str2Type

Convert type name to type id (mjtObj).

.. _mju_writeNumBytes:

mju_writeNumBytes
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_writeNumBytes

Construct a human readable number of bytes using standard letter suffix.

.. _mju_warningText:

mju_warningText
~~~~~~~~~~~~~~~

.. mujoco-include:: mju_warningText

Construct a warning message given the warning type and info.

.. _mju_isBad:

mju_isBad
~~~~~~~~~

.. mujoco-include:: mju_isBad

Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions.

.. _mju_isZero:

mju_isZero
~~~~~~~~~~

.. mujoco-include:: mju_isZero

Return 1 if all elements are 0.

.. _mju_standardNormal:

mju_standardNormal
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_standardNormal

Standard normal random number generator (optional second number).

mju_f2n
~~~~~~~

.. mujoco-include:: mju_f2n

Convert from float to mjtNum.

mju_n2f
~~~~~~~

.. mujoco-include:: mju_n2f

Convert from mjtNum to float.

mju_d2n
~~~~~~~

.. mujoco-include:: mju_d2n

Convert from double to mjtNum.

mju_n2d
~~~~~~~

.. mujoco-include:: mju_n2d

Convert from mjtNum to double.

.. _mju_insertionSort:

mju_insertionSort
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_insertionSort

Insertion sort, resulting list is in increasing order.

.. _mju_insertionSortInt:

mju_insertionSortInt
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_insertionSortInt

Integer insertion sort, resulting list is in increasing order.

.. _mju_Halton:

mju_Halton
~~~~~~~~~~

.. mujoco-include:: mju_Halton

Generate Halton sequence.

mju_strncpy
~~~~~~~~~~~

.. mujoco-include:: mju_strncpy

Call strncpy, then set dst[n-1] = 0.

mju_sigmoid
~~~~~~~~~~~

.. mujoco-include:: mju_sigmoid

Sigmoid function over 0<=x<=1 constructed from half-quadratics.

.. _mjd_transitionFD:

mjd_transitionFD
~~~~~~~~~~~~~~~~

.. mujoco-include:: mjd_transitionFD

Finite differenced state-transition and control-transition matrices dx(t+h) = A*dx(t) + B*du(t). Required output matrix
dimensions: A: (2*nv+na x 2*nv+na), B: (2*nv+na x nu).

.. _Macros:

Macros
^^^^^^

.. _mjMARKSTACK:

mjMARKSTACK
~~~~~~~~~~~

.. code-block:: C

   #define mjMARKSTACK int _mark = d->pstack;

This macro is helpful when using the MuJoCo stack in custom computations. It works together with the next macro and the
:ref:`mj_stackAlloc` function, and assumes that mjData\* d is defined. The use pattern is this:

::

       mjMARKSTACK
       mjtNum* temp = mj_stackAlloc(d, 100);
       // ... use temp as needed
       mjFREESTACK

.. _mjFREESTACK:

mjFREESTACK
~~~~~~~~~~~

.. code-block:: C

   #define mjFREESTACK d->pstack = _mark;

Reset the MuJoCo stack pointer to the variable \_mark, normally saved by mjMARKSTACK.

.. _mjDISABLED:

mjDISABLED
~~~~~~~~~~

.. code-block:: C

   #define mjDISABLED(x) (m->opt.disableflags & (x))

Check if a given standard feature has been disabled via the physics options, assuming mjModel\* m is defined. x is of
type :ref:`mjtDisableBit`.

.. _mjENABLED:

mjENABLED
~~~~~~~~~

.. code-block:: C

   #define mjENABLED(x) (m->opt.enableflags & (x))

Check if a given optional feature has been enabled via the physics options, assuming mjModel\* m is defined. x is of
type :ref:`mjtEnableBit`.

.. _mjMAX:

mjMAX
~~~~~

.. code-block:: C

   #define mjMAX(a,b) (((a) > (b)) ? (a) : (b))

Return maximum value. To avoid repeated evaluation with mjtNum types, use the function :ref:`mju_max`.

.. _mjMIN:

mjMIN
~~~~~

.. code-block:: C

   #define mjMIN(a,b) (((a) < (b)) ? (a) : (b))

Return minimum value. To avoid repeated evaluation with mjtNum types, use the function :ref:`mju_min`.
