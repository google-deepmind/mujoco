..
  This file contains each section text along with function doc overrides.  By default the docs use the function doc
  pulled from the header files.

.. _Virtualfilesystem:

Virtual file system (VFS) enables the user to load all necessary files in memory, including MJB binary model files, XML
files (MJCF, URDF and included files), STL meshes, PNGs for textures and height fields, and HF files in our custom
height field format. Model and resource files in the VFS can also be constructed programmatically (say using a Python
library that writes to memory). Once all desired files are in the VFS, the user can call :ref:`mj_loadModel` or
:ref:`mj_loadXML` with a pointer to the VFS. When this pointer is not NULL, the loaders will first check the VFS for any
files they are about to load, and only access the disk if the file is not found in the VFS.

The VFS must first be allocated using :ref:`mj_defaultVFS` and must be freed with :ref:`mj_deleteVFS`.


.. _mj_defaultVFS:

Initialize an empty VFS, :ref:`mj_deleteVFS` must be called to deallocate the VFS.

.. _mj_addFileVFS:

Add file to VFS. The directory argument is optional and can be NULL or empty. Returns 0 on success,
2 on name collision, or -1 when an internal error occurs.

.. _Parseandcompile:

The key function here is :ref:`mj_loadXML`. It invokes the built-in parser and compiler, and either returns a pointer to
a valid mjModel, or NULL - in which case the user should check the error information in the user-provided string.
The model and all files referenced in it can be loaded from disk or from a VFS when provided.

.. _mj_compile:

Compile :ref:`mjSpec` to :ref:`mjModel`. A spec can be edited and compiled multiple times, returning a new
:ref:`mjModel` instance that takes the edits into account.
If compilation fails, :ref:`mj_compile` returns ``NULL``; the error can be read with :ref:`mjs_getError`.

.. _mj_recompile:

Recompile spec to model, preserving the state. Like :ref:`mj_compile`, this function compiles an :ref:`mjSpec` to an
:ref:`mjModel`, with two differences. First, rather than returning an entirely new model, it will
reallocate existing :ref:`mjModel` and :ref:`mjData` instances in-place. Second, it will preserve the
:ref:`integration state<geIntegrationState>`, as given in the provided :ref:`mjData` instance, while accounting for
newly added or removed degrees of freedom. This allows the user to continue simulation with the same model and data
struct pointers while editing the model programmatically.

:ref:`mj_recompile` returns 0 if compilation succeed. In the case of failure, the given :ref:`mjModel` and :ref:`mjData`
instances will be deleted; as in :ref:`mj_compile`, the compilation error can be read with :ref:`mjs_getError`.

.. _mj_saveLastXML:

Update XML data structures with info from low-level model created with :ref:`mj_loadXML`, save as MJCF.
If error is not NULL, it must have size error_sz.

Note that this function only saves models that have been loaded with :ref:`mj_loadXML`, the legacy loading mechanism.
See the :ref:`model editing<meOverview>` chapter to understand the difference between the old and new model loading and
saving mechanisms.

.. _mj_saveXMLString:

Save spec to XML string, return 0 on success, -1 on failure. If the length of the output buffer is too small, returns
the required size. XML saving automatically compiles the spec before saving.

.. _mj_saveXML:

Save spec to XML file, return 0 on success, -1 otherwise. XML saving requires that the spec first be compiled.

.. _Mainsimulation:

These are the main entry points to the simulator. Most users will only need to call :ref:`mj_step`, which computes
everything and advanced the simulation state by one time step. Controls and applied forces must either be set in advance
(in ``mjData.{ctrl, qfrc_applied, xfrc_applied}``), or a control callback :ref:`mjcb_control` must be installed which
will be called just before the controls and applied forces are needed. Alternatively, one can use :ref:`mj_step1` and
:ref:`mj_step2` which break down the simulation pipeline into computations that are executed before and after the
controls are needed; in this way one can set controls that depend on the results from :ref:`mj_step1`. Keep in mind
though that the RK4 solver does not work with mj_step1/2. See :ref:`Pipeline` for a more detailed description.

mj_forward performs the same computations as :ref:`mj_step` but without the integration. It is useful after loading or
resetting a model (to put the entire mjData in a valid state), and also for out-of-order computations that involve
sampling or finite-difference approximations.

:ref:`mj_inverse` runs the inverse dynamics, and writes its output in ``mjData.qfrc_inverse``. Note that ``mjData.qacc``
must be set before calling this function. Given the state (qpos, qvel, act), mj_forward maps from force to acceleration,
while mj_inverse maps from acceleration to force. Mathematically these functions are inverse of each other, but
numerically this may not always be the case because the forward dynamics rely on a constraint optimization algorithm
which is usually terminated early. The difference between the results of forward and inverse dynamics can be computed
with the function :ref:`mj_compareFwdInv`, which can be thought of as another solver accuracy check (as well as a
general sanity check).

The skip version of :ref:`mj_forward` and :ref:`mj_inverse` are useful for example when qpos was unchanged but qvel was
changed (usually in the context of finite differencing). Then there is no point repeating the computations that only
depend on qpos. Calling the dynamics with skipstage = :ref:`mjSTAGE_POS<mjtStage>` will achieve these savings.

.. _Initialization:

This section contains functions that load/initialize the model or other data structures. Their use is well illustrated
in the code samples.

.. _Printing:

These functions can be used to print various quantities to the screen for debugging purposes.

.. _Components:

These are components of the simulation pipeline, called internally from :ref:`mj_step`, :ref:`mj_forward` and
:ref:`mj_inverse`. It is unlikely that the user will need to call them.

.. _mj_implicit:

Integrates the simulation state using an implicit-in-velocity integrator (either "implicit" or "implicitfast", see
:ref:`Numerical Integration<geIntegration>`), and advances simulation time. See `mjdata.h
<https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`__ for fields computed by this function.

.. _Subcomponents:

These are sub-components of the simulation pipeline, called internally from the components above.

.. _mj_makeM:

Compute the composite rigid body inertia with :ref:`mj_crb`, add terms due
to :ref:`tendon armature<tendon-spatial-armature>`. The joint-space inertia matrix is stored in both ``mjData.qM`` and
``mjData.M``. These arrays represent the same quantity using different layouts (parent-based and compressed sparse row,
respectively).

.. _mj_factorM:

Compute sparse :math:`L^T D L` factorizaton of inertia matrix.

.. _mj_solveM:

Solve linear system :math:`M x = y` using factorization: :math:`x = (L^T D L)^{-1} y`

.. _mj_solveM2:

Half of linear solve: :math:`x = \sqrt{D^{-1}} (L^T)^{-1} y`

.. _mj_subtreeVel:

Sub-tree linear velocity and angular momentum: compute ``subtree_linvel``, ``subtree_angmom``.
This function is triggered automatically if the subtree :ref:`velocity<sensor-subtreelinvel>` or
:ref:`momentum<sensor-subtreeangmom>` sensors are present in the model.
It is also triggered for :ref:`user sensors<sensor-user>` of :ref:`stage<sensor-user-needstage>` "vel".

.. _mj_rne:

Recursive Newton Euler: compute :math:`M(q) \ddot q + C(q,\dot q)`. ``flg_acc=0`` removes the inertial term (i.e.
assumes :math:`\ddot q = 0`).

.. _mj_rnePostConstraint:

Recursive Newton Euler with final computed forces and accelerations.
Computes three body-level ``nv x 6`` arrays, all defined in the subtreecom-based
:ref:`c-frame<tyNotesCom>` and arranged in ``[rotation(3), translation(3)]`` order.

- ``cacc``: Body acceleration, required for :ref:`mj_objectAcceleration`.
- ``cfrc_int``: Interaction force with the parent body.
- ``cfrc_ext``: External force acting on the body.

This function is triggered automatically if the following sensors are present in the model:
:ref:`accelerometer<sensor-accelerometer>`, :ref:`force<sensor-force>`, :ref:`torque<sensor-torque>`,
:ref:`framelinacc<sensor-framelinacc>`, :ref:`frameangacc<sensor-frameangacc>`.
It is also triggered for :ref:`user sensors<sensor-user>` of :ref:`stage<sensor-user-needstage>` "acc".

The computed force arrays ``cfrc_int`` and ``cfrc_ext`` currently suffer from a know bug, they do not take into account
the effect of spatial tendons, see :github:issue:`832`.

.. _mj_constraintUpdate:

Compute ``efc_state``, ``efc_force``, ``qfrc_constraint``, and (optionally) cone Hessians.
If ``cost`` is not ``NULL``, set ``*cost = s(jar)`` where ``jar = Jac*qacc - aref``.

.. _Support:

These are support functions that need access to :ref:`mjModel` and :ref:`mjData`, unlike the utility functions which do
not need such access. Support functions are called within the simulator but some of them can also be useful for custom
computations, and are documented in more detail below.

.. _mj_stateSize:

Returns the number of :ref:`mjtNum` |-| s required for a given state specification. The bits of the integer ``spec``
correspond to element fields of :ref:`mjtState`.

.. _mj_getState:

Copy concatenated state components specified by ``spec`` from ``d`` into ``state``. The bits of the integer
``spec`` correspond to element fields of :ref:`mjtState`. Fails with :ref:`mju_error` if ``spec`` is invalid.

.. _mj_setState:

Copy concatenated state components specified by ``spec`` from  ``state`` into ``d``. The bits of the integer
``spec`` correspond to element fields of :ref:`mjtState`. Fails with :ref:`mju_error` if ``spec`` is invalid.

.. _mj_mulJacVec:

This function multiplies the constraint Jacobian mjData.efc_J by a vector. Note that the Jacobian can be either dense or
sparse; the function is aware of this setting. Multiplication by J maps velocities from joint space to constraint space.

.. _mj_mulJacTVec:

Same as mj_mulJacVec but multiplies by the transpose of the Jacobian. This maps forces from constraint space to joint
space.

.. _mj_jac:

This function computes an end-effector kinematic Jacobian, describing the local linear relationship between the
degrees-of-freedom and a given point. Given a body specified by its integer id (``body``) and a 3D point in the world
frame (``point``) treated as attached to the body, the Jacobian has both translational (``jacp``) and rotational
(``jacr``) components. Passing ``NULL`` for either pointer will skip that part of the computation. Each component is a
3-by-nv matrix. Each row of this matrix is the gradient of the corresponding coordinate of the specified point with
respect to the degrees-of-freedom. The frame with respect to which the Jacobian is computed is centered at the body
center-of-mass but aligned with the world frame. The minimal :ref:`pipeline stages<piForward>` required for Jacobian
computations to be consistent with the current generalized positions ``mjData.qpos`` are :ref:`mj_kinematics` followed
by :ref:`mj_comPos`.

.. _mj_jacBody:

This and the remaining variants of the Jacobian function call mj_jac internally, with the center of the body, geom or
site. They are just shortcuts; the same can be achieved by calling mj_jac directly.

.. _mj_jacDot:

This function computes the time-derivative of an end-effector kinematic Jacobian computed by :ref:`mj_jac`.
The minimal :ref:`pipeline stages<piStages>` required for computation to be
consistent with the current generalized positions and velocities ``mjData.{qpos, qvel}`` are
:ref:`mj_kinematics`, :ref:`mj_comPos`, :ref:`mj_comVel` (in that order).

.. _mj_angmomMat:

This function computes the ``3 x nv`` angular momentum matrix :math:`H(q)`, providing the linear mapping from
generalized velocities to subtree angular momentum. More precisely if :math:`h` is the subtree angular momentum of
body index ``body`` in ``mjData.subtree_angmom`` (reported by the :ref:`subtreeangmom<sensor-subtreeangmom>` sensor)
and :math:`\dot q` is the generalized velocity ``mjData.qvel``, then :math:`h = H \dot q`.

.. _mj_geomDistance:

Returns the smallest signed distance between two geoms and optionally the segment from ``geom1`` to ``geom2``.
Returned distances are bounded from above by ``distmax``. |br| If no collision of distance smaller than ``distmax`` is
found, the function will return ``distmax`` and ``fromto``, if given, will be set to (0, 0, 0, 0, 0, 0).

   .. admonition:: different (correct) behavior under `nativeccd`
      :class: note

      As explained in :ref:`Collision Detection<coDistance>`, distances are inaccurate when using the
      :ref:`legacy CCD pipeline<coCCD>`, and its use is discouraged.

.. _mj_fullM:

Convert sparse inertia matrix ``M`` into full (i.e. dense) matrix.
|br| ``dst`` must be of size ``nv x nv``, ``M`` must be of the same size as ``mjData.qM``.

.. _mj_mulM:

This function multiplies the joint-space inertia matrix stored in mjData.qM by a vector. qM has a custom sparse format
that the user should not attempt to manipulate directly. Alternatively one can convert qM to a dense matrix with
mj_fullM and then user regular matrix-vector multiplication, but this is slower because it no longer benefits from
sparsity.

.. _mj_applyFT:

This function can be used to apply a Cartesian force and torque to a point on a body, and add the result to the vector
mjData.qfrc_applied of all applied forces. Note that the function requires a pointer to this vector, because sometimes
we want to add the result to a different vector.

.. _mj_objectAcceleration:

Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation. If acceleration or force
sensors are not present in the model, :ref:`mj_rnePostConstraint` must be manually called in order to calculate
mjData.cacc -- the total body acceleration, including contributions from the constraint solver.

.. _mj_differentiatePos:

This function subtracts two vectors in the format of qpos (and divides the result by dt), while respecting the
properties of quaternions. Recall that unit quaternions represent spatial orientations. They are points on the unit
sphere in 4D. The tangent to that sphere is a 3D plane of rotational velocities. Thus when we subtract two quaternions
in the right way, the result is a 3D vector and not a 4D vector. Thus the output qvel has dimensionality nv while the
inputs have dimensionality nq.

.. _mj_integratePos:

This is the opposite of mj_differentiatePos. It adds a vector in the format of qvel (scaled by dt) to a vector in the
format of qpos.

.. _Raycollisions:

Ray collisions, also known as ray casting, find the distance ``x`` of a ray's intersection with a geom, where a ray is
a line emanating from the 3D point ``p`` in the direction ``v`` i.e., ``(p + x*v, x >= 0)``. All functions in this
family return the distance to the nearest geom surface, or -1 if there is no intersection. Note that if ``p`` is inside
a geom, the ray will intersect the surface from the inside which still counts as an intersection.

All ray collision functions rely on quantities computed by :ref:`mj_kinematics` (see :ref:`mjData`), so must be called
after  :ref:`mj_kinematics`, or functions that call it (e.g. :ref:`mj_fwdPosition`). The top level functions, which
intersect with all geoms types, are :ref:`mj_ray` which casts a single ray, and :ref:`mj_multiRay` which casts multiple
rays from a single point.

.. _mj_ray:

Intersect ray ``(pnt+x*vec, x >= 0)`` with visible geoms, except geoms in bodyexclude.

Return geomid and distance (x) to nearest surface, or -1 if no intersection.

geomgroup is an array of length mjNGROUP, where 1 means the group should be included. Pass geomgroup=NULL to skip
group exclusion.

If flg_static is 0, static geoms will be excluded.

bodyexclude=-1 can be used to indicate that all bodies are included.

.. _Interaction:

These functions implement abstract mouse interactions, allowing control over cameras and perturbations. Their use is well
illustrated in :ref:`simulate<saSimulate>`.

.. _mjv_select:

This function is used for mouse selection, relying on ray intersections. aspectratio is the viewport width/height. relx
and rely are the relative coordinates of the 2D point of interest in the viewport (usually mouse cursor). The function
returns the id of the geom under the specified 2D point, or -1 if there is no geom (note that they skybox if present is
not a model geom). The 3D coordinates of the clicked point are returned in selpnt. See :ref:`simulate<saSimulate>` for
an illustration.

.. _Visualization-api:

The functions in this section implement abstract visualization. The results are used by the OpenGL renderer, and can
also be used by users wishing to implement their own renderer, or hook up MuJoCo to advanced rendering tools such as
Unity or Unreal Engine. See :ref:`simulate<saSimulate>` for illustration of how to use these functions.

.. _OpenGLrendering:

These functions expose the OpenGL renderer. See :ref:`simulate<saSimulate>` for an illustration
of how to use these functions.

.. _UIframework:

For a high-level description of the UI framework, see :ref:`UI`.

.. _mjui_add:

This is the helper function used to construct a UI. The second argument points to an array of :ref:`mjuiDef` structs,
each corresponding to one item. The last (unused) item has its type set to -1, to mark termination. The items are added
after the end of the last used section. There is also another version of this function
(:ref:`mjui_addToSection<mjui_addToSection>`) which adds items to a specified section instead of adding them at the end
of the UI. Keep in mind that there is a maximum preallocated number of sections and items per section, given by
:ref:`mjMAXUISECT<glNumeric>` and :ref:`mjMAXUIITEM<glNumeric>`. Exceeding these maxima results in low-level errors.

.. _mjui_update:

This is the main UI update function. It needs to be called whenever the user data (pointed to by the item data pointers)
changes, or when the UI state itself changes. It is normally called by a higher-level function implemented by the user
(``UiModify`` in :ref:`simulate.cc <saSimulate>`) which also recomputes the layout of all rectangles and associated
auxiliary buffers. The function updates the pixels in the offscreen OpenGL buffer. To perform minimal updates, the user
specifies the section and the item that was modified. A value of -1 means all items and/or sections need to be updated
(which is needed following major changes.)

.. _mjui_event:

This function is the low-level event handler. It makes the necessary changes in the UI and returns a pointer to the item
that received the event (or ``NULL`` if no valid event was recorded). This is normally called within the event handler
implemented by the user (``UiEvent`` in :ref:`simulate.cc <saSimulate>`), and then some action is taken by user code
depending on which UI item was modified and what the state of that item is after the event is handled.


.. _mjui_render:

This function is called in the screen refresh loop. It copies the offscreen OpenGL buffer to the window framebuffer. If
there are multiple UIs in the application, it should be called once for each UI. Thus ``mjui_render`` is called all the
time, while :ref:`mjui_update` is called only when changes in the UI take place. dsffsdg




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

.. _mju_cholFactorBand:

Band-dense Cholesky decomposition.
|br| Add ``diagadd + diagmul*mat_ii`` to diagonal before decomposition.
|br| Returns the minimum value of the factorized diagonal or 0 if rank-deficient.

   **Symmetric band-dense matrices**

   :ref:`mju_cholFactorBand` and subsequent functions containing the substring "band" operate on matrices which are a
   generalization of symmetric `band matrices <https://en.wikipedia.org/wiki/Band_matrix>`_. *Symmetric band-dense* or
   "arrowhead" matrices have non-zeros along proximal diagonal bands and dense blocks on the bottom rows and right
   columns. These matrices have the property that Cholesky factorization creates no fill-in and can therefore be
   performed efficiently in-place. Matrix structure is defined by three integers:

   - ``ntotal``: the number of rows (columns) of the symmetric matrix.
   - ``nband``: the number of bands under (over) the diagonal, inclusive of the diagonal.
   - ``ndense``: the number of dense rows (columns) at the bottom (right).

   The non-zeros are stored in memory as two contiguous row-major blocks, colored green and blue in the illustration
   below. The first block has size ``nband x (ntotal-ndense)`` and contains the diagonal and the bands below it. The
   second block has size ``ndense x ntotal`` and contains the dense part. Total required memory is the sum of the block
   sizes.

   .. figure:: /images/APIreference/arrowhead.svg
      :width: 750px
      :align: left

   For example, consider an arrowhead matrix with ``nband = 3``, ``ndense = 2`` and ``ntotal = 8``. In this example, the
   total memory required is ``3*(8-2) + 2*8 = 34`` mjtNum's, laid out as follows:

   .. code-block::

      0   1   2
          3   4   5
              6   7   8
                  9   10  11
                      12  13  14
                          15  16  17
              18  19  20  21  22  23  24  25
              26  27  28  29  30  31  32  33


   The diagonal elements are ``2, 5, 8, 11, 14, 17, 24, 33``.
   |br| Elements ``0, 1, 3, 25`` are present in memory but never touched.

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
  ``R`` must have allocated size ``n*(n+7)``, but only ``nfree*nfree`` values are used as output.
  ``index`` (if given) must have allocated size ``n``, but only ``nfree`` values are used as output.
  The convenience function :ref:`mju_boxQPmalloc` allocates the required data structures.
  Only the lower triangles of H and R are read from and written to, respectively.

.. _mju_boxQPmalloc:

Allocate heap memory for box-constrained Quadratic Program.
As in :ref:`mju_boxQP`, ``index``, ``lower``, and ``upper`` are optional.
Free all pointers with ``mju_free()``.

.. _mju_symmetrize:

Symmetrize square matrix :math:`R = \frac{1}{2}(M + M^T)`.

.. _Miscellaneous:

.. _mju_sigmoid:

Twice continuously differentiable sigmoid function using a quintic polynomial:

.. math::
   s(x) =
   \begin{cases}
      0,                    &       & x \le 0  \\
      6x^5 - 15x^4 + 10x^3, & 0 \lt & x \lt 1  \\
      1,                    & 1 \le & x \qquad
   \end{cases}

.. _Derivatives-api:

The functions below provide useful derivatives of various functions, both analytic and
finite-differenced. The latter have names with the suffix ``FD``. Note that unlike much of the API,
outputs of derivative functions are the trailing rather than leading arguments.

.. _mjd_transitionFD:

Compute finite-differenced discrete-time transition matrices.

Letting :math:`x, u` denote the current :ref:`state<gePhysicsState>` and :ref:`control<geInput>`
vector in an mjData instance, and letting :math:`y, s` denote the next state and sensor
values, the top-level :ref:`mj_step` function computes :math:`(x,u) \rightarrow (y,s)`
:ref:`mjd_transitionFD` computes the four associated Jacobians using finite-differencing.
These matrices and their dimensions are:

.. csv-table::
   :header: "matrix", "Jacobian", "dimension"
   :widths: auto
   :align: left

   ``A``, :math:`\partial y / \partial x`, ``2*nv+na x 2*nv+na``
   ``B``, :math:`\partial y / \partial u`, ``2*nv+na x nu``
   ``C``, :math:`\partial s / \partial x`, ``nsensordata x 2*nv+na``
   ``D``, :math:`\partial s / \partial u`, ``nsensordata x nu``

- All outputs are optional (can be NULL).
- ``eps`` is the finite-differencing epsilon.
- ``flg_centered`` denotes whether to use forward (0) or centered (1) differences.
- The Runge-Kutta integrator (:ref:`mjINT_RK4<mjtIntegrator>`) is not supported.

.. admonition:: Improving speed and accuracy
   :class: tip

   warmstart
     If warm-starts are not :ref:`disabled<option-flag-warmstart>`, the warm-start accelerations
     ``mjData.qacc_warmstart`` which are present at call-time are loaded at the start of every relevant pipeline call,
     to preserve determinism. If solver computations are an expensive part of the simulation, the following trick can
     lead to significant speed-ups: First call :ref:`mj_forward` to let the solver converge, then reduce :ref:`solver
     iterations<option-iterations>` significantly, then call :ref:`mjd_transitionFD`, finally, restore the original
     value of :ref:`iterations<option-iterations>`. Because we are already near the solution, few iteration are required
     to find the new minimum. This is especially true for the :ref:`Newton<option-solver>` solver, where the required
     number of iteration for convergence near the minimum can be as low as 1.

   tolerance
      Accuracy can be improved if solver :ref:`tolerance<option-tolerance>` is set to 0. This means that all calls to
      the solver will perform exactly the same number of iterations, preventing numerical errors due to early
      termination. Of course, this means that :ref:`solver iterations<option-iterations>` should be small, to not tread
      water at the minimum. This method and the one described above can and should be combined.


.. _mjd_inverseFD:

Finite differenced continuous-time inverse-dynamics Jacobians.

Letting :math:`x, a` denote the current :ref:`state<gePhysicsState>` and acceleration vectors in an mjData instance, and
letting :math:`f, s` denote the forces computed by the inverse dynamics (``qfrc_inverse``), the function
:ref:`mj_inverse` computes :math:`(x,a) \rightarrow (f,s)`. :ref:`mjd_inverseFD` computes seven associated Jacobians
using finite-differencing. These matrices and their dimensions are:

.. csv-table::
   :header: "matrix", "Jacobian", "dimension"
   :widths: auto
   :align: left

   ``DfDq``, :math:`\partial f / \partial q`, ``nv x nv``
   ``DfDv``, :math:`\partial f / \partial v`, ``nv x nv``
   ``DfDa``, :math:`\partial f / \partial a`, ``nv x nv``
   ``DsDq``, :math:`\partial s / \partial q`, ``nv x nsensordata``
   ``DsDv``, :math:`\partial s / \partial v`, ``nv x nsensordata``
   ``DsDa``, :math:`\partial s / \partial a`, ``nv x nsensordata``
   ``DmDq``, :math:`\partial M / \partial q`, ``nv x nM``

- All outputs are optional (can be NULL).
- All outputs are transposed relative to Control Theory convention (i.e., column major).
- ``DmDq``, which contains a sparse representation of the ``nv x nv x nv`` tensor :math:`\partial M / \partial q`, is
  not strictly an inverse dynamics Jacobian but is useful in related applications. It is provided as a convenience to
  the user, since the required values are already computed if either of the other two :math:`\partial / \partial q`
  Jacobians are requested.
- ``eps`` is the (forward) finite-differencing epsilon.
- ``flg_actuation`` denotes whether to subtract actuation forces (``qfrc_actuator``) from the output of the inverse
  dynamics. If this flag is positive, actuator forces are not considered as external.
- The model option flag ``invdiscrete`` should correspond to the representation of ``mjData.qacc`` in order to compute
  the correct derivative information.

.. attention::
   - The Runge-Kutta 4th-order integrator (``mjINT_RK4``) is not supported.
   - The noslip solver is not supported.

.. _mjd_subQuat:

Derivatives of :ref:`mju_subQuat` (quaternion difference).

.. _mjd_quatIntegrate:

Derivatives of :ref:`mju_quatIntegrate`.

:math:`{\tt \small mju\_quatIntegrate}(q, v, h)` performs the in-place rotation :math:`q \leftarrow q + v h`,
where :math:`q \in \mathbf{S}^3` is a unit quaternion, :math:`v \in \mathbf{R}^3` is a 3D angular velocity and
:math:`h \in \mathbf{R^+}` is a timestep. This is equivalent to :math:`{\tt \small mju\_quatIntegrate}(q, s, 1.0)`,
where :math:`s` is the scaled velocity :math:`s = h v`.

:math:`{\tt \small mjd\_quatIntegrate}(v, h, D_q, D_v, D_h)` computes the Jacobians of the output :math:`q` with respect
to the inputs. Below, :math:`\bar q` denotes the pre-modified quaternion:

.. math::
   \begin{aligned}
      D_q &= \partial q / \partial \bar q \\
      D_v &= \partial q / \partial v \\
      D_h &= \partial q / \partial h
   \end{aligned}

Note that derivatives depend only on :math:`h` and :math:`v` (in fact, on :math:`s = h v`).
All outputs are optional.

.. _mjs_delete:

Delete object corresponding to the given element, return 0 on success. This function should only be used for element
types that cannot have children, i.e. excluding bodies and default classes.
