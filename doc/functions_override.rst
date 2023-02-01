..
  This file contains each section text along with function doc overrides.  By default the docs use the function doc
  pulled from the header files.

.. _Activation:

The functions in this section are maintained for backward compatibility with the now-removed activation mechanism.

.. _Virtualfilesystem:

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

.. _Parseandcompile:

The key function here is mj_loadXML. It invokes the built-in parser and compiler, and either returns a pointer to a
valid mjModel, or NULL - in which case the user should check the error information in the user-provided string. The
model and all files referenced in it can be loaded from disk or from a VFS when provided.

.. _Mainsimulation:

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

.. _Initialization:

This section contains functions that load/initialize the model or other data structures. Their use is well illustrated
in the code samples.

.. _Printing:

These functions can be used to print various quantities to the screen for debugging purposes.

.. _Components:

These are components of the simulation pipeline, called internally from mj_step, mj_forward and mj_inverse. It is
unlikely that the user will need to call them.

.. _Subcomponents:

These are sub-components of the simulation pipeline, called internally from the components above. It is very unlikely
that the user will need to call them.

.. _mj_factorM:

Compute sparse :math:`L^T D L` factorizaton of inertia matrix.

.. _mj_solveM:

Solve linear system :math:`M x = y` using factorization: :math:`x = (L^T D L)^{-1} y`

.. _mj_solveM2:

Half of linear solve: :math:`x = \sqrt{D^{-1}} (L^T)^{-1} y`

.. _mj_constraintUpdate:

Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians. If cost is not NULL, set \*cost = s(jar)
where jar = Jac*qacc-aref.

.. _Support:

These are support functions that need access to mjModel and mjData, unlike the utility functions which do not need such
access. Support functions are called within the simulator but some of them can also be useful for custom computations,
and are documented in more detail below.

.. _mj_mulJacVec:

This function multiplies the constraint Jacobian mjData.efc_J by a vector. Note that the Jacobian can be either dense or
sparse; the function is aware of this setting. Multiplication by J maps velocities from joint space to constraint space.

.. _mj_mulJacTVec:

Same as mj_mulJacVec but multiplies by the transpose of the Jacobian. This maps forces from constraint space to joint
space.

.. _mj_jac:

This function computes an "end-effector" Jacobian, which is unrelated to the constraint Jacobian above. Any MuJoCo body
can be treated as end-effector, and the point for which the Jacobian is computed can be anywhere in space (it is treated
as attached to the body). The Jacobian has translational (jacp) and rotational (jacr) components. Passing NULL for
either pointer will skip part of the computation. Each component is a 3-by-nv matrix. Each row of this matrix is the
gradient of the corresponding 3D coordinate of the specified point with respect to the degrees of freedom. The ability
to compute end-effector Jacobians analytically is one of the advantages of working in minimal coordinates - so use it!

.. _mj_jacBody:

This and the remaining variants of the Jacobian function call mj_jac internally, with the center of the body, geom or
site. They are just shortcuts; the same can be achieved by calling mj_jac directly.

.. _mj_mulM:

This function multiplies the joint-space inertia matrix stored in mjData.qM by a vector. qM has a custom sparse format
that the user should not attempt to manipulate directly. Alternatively one can convert qM to a dense matrix with
mj_fullM and then user regular matrix-vector multiplication, but this is slower because it no longer benefits from
sparsity.

.. _mj_applyFT:

This function can be used to apply a Cartesian force and torque to a point on a body, and add the result to the vector
mjData.qfrc_applied of all applied forces. Note that the function requires a pointer to this vector, because sometimes
we want to add the result to a different vector.

.. _mj_differentiatePos:

This function subtracts two vectors in the format of qpos (and divides the result by dt), while respecting the
properties of quaternions. Recall that unit quaternions represent spatial orientations. They are points on the unit
sphere in 4D. The tangent to that sphere is a 3D plane of rotational velocities. Thus when we subtract two quaternions
in the right way, the result is a 3D vector and not a 4D vector. This the output qvel has dimensionality nv while the
inputs have dimensionality nq.

.. _mj_integratePos:

This is the opposite of mj_differentiatePos. It adds a vector in the format of qvel (scaled by dt) to a vector in the
format of qpos.

.. _Raycollisions:

Ray collision functionality was added in MuJoCo 1.50. This is a new collision detection module that uses analytical
formulas to intersect a ray (p + x*v, x>=0) with a geom, where p is the origin of the ray and v is the vector specifying
the direction. All functions in this family return the distance to the nearest geom surface, or -1 if there is no
intersection. Note that if p is inside a geom, the ray will intersect the surface from the inside which still counts as
an intersection.

All ray collision functions rely on quantities computed by :ref:`mj_kinematics` (see :ref:`mjData`), so must be called
after  :ref:`mj_kinematics`, or functions that call it (e.g. :ref:`mj_fwdPosition`).

.. _mj_ray:

Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude. Return geomid and distance (x) to
nearest surface, or -1 if no intersection.

geomgroup is an array of length mjNGROUP, where 1 means the group should be included. Pass geomgroup=NULL to skip
group exclusion.
If flg_static is 0, static geoms will be excluded.
bodyexclude=-1 can be used to indicate that all bodies are included.

.. _Interaction:

These function implement abstract mouse interactions, allowing control over cameras and perturbations. Their use is well
illustrated in :ref:`simulate.cc <saSimulate>`.

.. _mjv_select:

This function is used for mouse selection. Previously selection was done via OpenGL, but as of MuJoCo 1.50 it relies on
ray intersections which are much more efficient. aspectratio is the viewport width/height. relx and rely are the
relative coordinates of the 2D point of interest in the viewport (usually mouse cursor). The function returns the id of
the geom under the specified 2D point, or -1 if there is no geom (note that they skybox if present is not a model geom).
The 3D coordinates of the clicked point are returned in selpnt. See :ref:`simulate.cc <saSimulate>` for an illustration.

.. _Visualization-api:

The functions in this section implement abstract visualization. The results are used by the OpenGL rendered, and can
also be used by users wishing to implement their own rendered, or hook up MuJoCo to advanced rendering tools such as
Unity or Unreal Engine. See :ref:`simulate.cc <saSimulate>` for illustration of how to use these functions.

.. _OpenGLrendering:

These functions expose the OpenGL renderer. See :ref:`simulate.cc <saSimulate>` for an illustration
of how to use these functions.

.. _UIframework:

.. _Errorandmemory:

.. _Standardmath:

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

.. _Quaternions:

.. _Poses:

.. _Decompositions:

.. _mju_boxQP:

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

Allocate heap memory for box-constrained Quadratic Program.
As in :ref:`mju_boxQP`, ``index``, ``lower``, and ``upper`` are optional.
Free all pointers with ``mju_free()``.

.. _mju_symmetrize:

Symmetrize square matrix :math:`R = \frac{1}{2}(M + M^T)`.

.. _Miscellaneous:

.. _Derivatives-api:

.. _mjd_transitionFD:

Finite differenced transition matrices. Letting :math:`x, u` denote the current :ref:`state<gePhysicsState>` and control
vectors and letting :math:`y, s` denote the next state and sensor values, the top-level :ref:`mj_step` function computes
:math:`(x,u) \rightarrow (y,s)`. :ref:`mjd_transitionFD` computes the four associated Jacobians using
finite-differencing. These matrices and their dimensions are:

.. csv-table::
   :header: "matrix", "Jacobian", "dimension"
   :widths: auto
   :align: left

   ``A``, :math:`\partial y / \partial x`, ``2*nv+na x 2*nv+na``
   ``B``, :math:`\partial y / \partial u`, ``2*nv+na x nu``
   ``C``, :math:`\partial s / \partial x`, ``nsensordata x 2*nv+na``
   ``D``, :math:`\partial s / \partial u`, ``nsensordata x nu``

- All four matrix outputs are optional (can be NULL).
- ``eps`` is the finite-differencing epsilon.
- ``centered`` is a flag denoting whether to use forward (0) or centered (1) differences.

