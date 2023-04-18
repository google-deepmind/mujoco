.. _API:

=========
Functions
=========

The main header `mujoco.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mujoco.h>`_ exposes a large
number of functions. However the functions that most users are likely to need are a small fraction.

API function can be classified as:

- :ref:`Parse and compile<Parseandcompile>` an :ref:`mjModel` from XML files and assets.
- :ref:`Main simulation<Mainsimulation>` entry points, including :ref:`mj_step`.
- :ref:`Support<Support>` functions requiring :ref:`mjModel` and :ref:`mjData`.
- :ref:`Components<Components>` of the simulation pipeline, called from :ref:`mj_step`, :ref:`mj_forward` and :ref:`mj_inverse`.
- :ref:`Sub components<Subcomponents>` of the simulation pipeline.
- :ref:`Ray collisions<Raycollisions>`.
- :ref:`Printing<Printing>` of various quantities.
- :ref:`Virtual file system<Virtualfilesystem>`, used to load assets from memory.
- :ref:`Initialization<Initialization>` of data structures.
- :ref:`Abstract interaction<Interaction>`: mouse control of cameras and perturbations.
- :ref:`Abstract Visualization<Visualization-api>`.
- :ref:`OpenGL rendering<OpenGLrendering>`.
- :ref:`UI framework<UIframework>`.
- :ref:`Error and memory<Errorandmemory>`.
- Deprecated :ref:`activation<Activation>` mechanism.
- :ref:`Aliases for C standard math<Standardmath>` functions.
- :ref:`Vector math<Vectormath>`.
- :ref:`Quaternions<Quaternions>`.
- :ref:`Poses transformations<Poses>`.
- :ref:`Matrix decompositions and solvers<Decompositions>`.
- :ref:`Miscellaneous<Miscellaneous>` functions.
- :ref:`Dynamics derivatives<Derivatives-api>`.
- :ref:`Plugin<Plugins-api>` related functions.
- :ref:`Macros<Macros>`.

.. TODO(b/273075045): Better category-label namespacing.

.. include:: functions.rst

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

.. code-block:: C

   mjMARKSTACK;
   mjtNum* temp = mj_stackAlloc(d, 100);
   // ... use temp as needed
   mjFREESTACK;


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


.. _mjPLUGIN_LIB_INIT:

mjPLUGIN_LIB_INIT
~~~~~~~~~~~~~~~~~

.. code-block:: C

   #define mjPLUGIN_LIB_INIT                                                                 \
     static void _mjplugin_dllmain(void);                                                    \
     mjEXTERNC int __stdcall mjDLLMAIN(void* hinst, unsigned long reason, void* reserved) {  \
       if (reason == 1) {                                                                    \
         _mjplugin_dllmain();                                                                \
       }                                                                                     \
       return 1;                                                                             \
     }                                                                                       \
     static void _mjplugin_dllmain(void)

Register a plugin as a dynamic library. See :ref:`plugin registration<exRegistration>` for more details.
