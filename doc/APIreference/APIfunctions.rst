.. _API:

=========
Functions
=========

The main header `mujoco.h <https://github.com/deepmind/mujoco/blob/main/include/mujoco/mujoco.h>`_ exposes a very large
number of functions. However the functions that most users are likely to need are a small fraction. For example,
:ref:`simulate.cc <saSimulate>` which is as elaborate as a MuJoCo application is likely to get, calls around 40 of these
functions, while ``basic.cc`` calls around 20. The rest are explosed just in case someone has a use for them. This
includes us as users of MuJoCo -- we do our own work with the public library instead of relying on internal builds.

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
