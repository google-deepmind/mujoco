..
  AUTOGENERATE: DO NOT EDIT


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

Add file to VFS, return 0: success, 1: full, 2: repeated name, -1: failed to load.

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

Parse XML file in MJCF or URDF format, compile it, return low-level model.
If vfs is not ``NULL``, look up files in vfs before reading from disk.
If error is not ``NULL``, it must have size error_sz.

.. _mj_saveLastXML:

mj_saveLastXML
~~~~~~~~~~~~~~

.. mujoco-include:: mj_saveLastXML

Update XML data structures with info from low-level model, save as MJCF.
If error is not ``NULL``, it must have size error_sz.

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

Copy ``mjModel``, allocate new if dest is ``NULL``.

.. _mj_saveModel:

mj_saveModel
~~~~~~~~~~~~

.. mujoco-include:: mj_saveModel

Save model to binary MJB file or memory buffer; buffer has precedence when given.

.. _mj_loadModel:

mj_loadModel
~~~~~~~~~~~~

.. mujoco-include:: mj_loadModel

Load model from binary MJB file.
If vfs is not ``NULL``, look up file in vfs before reading from disk.

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

Allocate ``mjData`` corresponding to given model.
If the model buffer is unallocated the initial configuration will not be set.

.. _mj_copyData:

mj_copyData
~~~~~~~~~~~

.. mujoco-include:: mj_copyData

Copy ``mjData``.
m is only required to contain the size fields from MJMODEL_INTS.

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

Allocate array of specified size on ``mjData`` stack. Call mju_error on stack overflow.

.. _mj_deleteData:

mj_deleteData
~~~~~~~~~~~~~

.. mujoco-include:: mj_deleteData

Free memory allocation in ``mjData``.

.. _mj_resetCallbacks:

mj_resetCallbacks
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_resetCallbacks

Reset all callbacks to ``NULL`` pointers (``NULL`` is the default).

.. _mj_setConst:

mj_setConst
~~~~~~~~~~~

.. mujoco-include:: mj_setConst

Set constant fields of ``mjModel``, corresponding to qpos0 configuration.

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

Print ``mjModel`` to text file, specifying format.
float_format must be a valid printf-style format string for a single float value.

.. _mj_printModel:

mj_printModel
~~~~~~~~~~~~~

.. mujoco-include:: mj_printModel

Print model to text file.

.. _mj_printFormattedData:

mj_printFormattedData
~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printFormattedData

Print ``mjData`` to text file, specifying format.
float_format must be a valid printf-style format string for a single float value

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

.. _mj_camlight:

mj_camlight
~~~~~~~~~~~

.. mujoco-include:: mj_camlight

Compute camera and light positions and orientations.

.. _mj_tendon:

mj_tendon
~~~~~~~~~

.. mujoco-include:: mj_tendon

Compute tendon lengths, velocities and moment arms.

.. _mj_transmission:

mj_transmission
~~~~~~~~~~~~~~~

.. mujoco-include:: mj_transmission

Compute actuator transmission lengths and moments.

.. _mj_crb:

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

.. _mj_passive:

mj_passive
~~~~~~~~~~

.. mujoco-include:: mj_passive

Compute qfrc_passive from spring-dampers, viscosity and density.

.. _mj_subtreeVel:

mj_subtreeVel
~~~~~~~~~~~~~

.. mujoco-include:: mj_subtreeVel

subtree linear velocity and angular momentum

.. _mj_rne:

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

.. _mj_constraintUpdate:

mj_constraintUpdate
~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_constraintUpdate

Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians. If cost is not ``NULL``, set \*cost = s(jar)
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

This function multiplies the constraint Jacobian ``mjData.efc_J`` by a vector. Note that the Jacobian can be either dense or
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
as attached to the body). The Jacobian has translational (jacp) and rotational (jacr) components. Passing ``NULL`` for
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

Compute subtree center-of-mass end-effector Jacobian.

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

This function multiplies the joint-space inertia matrix stored in ``mjData.qM`` by a vector. qM has a custom sparse format
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

Add inertia matrix to destination matrix.
Destination can be sparse uncompressed, or dense when all int* are ``NULL``

.. _mj_applyFT:

mj_applyFT
~~~~~~~~~~

.. mujoco-include:: mj_applyFT

This function can be used to apply a Cartesian force and torque to a point on a body, and add the result to the vector
``mjData.qfrc_applied`` of all applied forces. Note that the function requires a pointer to this vector, because sometimes
we want to add the result to a different vector.

.. _mj_objectVelocity:

mj_objectVelocity
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_objectVelocity

Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation.

.. _mj_objectAcceleration:

mj_objectAcceleration
~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_objectAcceleration

Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation.

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

.. _mj_getPluginConfig:

mj_getPluginConfig
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_getPluginConfig

Return a config attribute value of a plugin instance;
``NULL``: invalid plugin instance ID or attribute name

.. _mj_loadPluginLibrary:

mj_loadPluginLibrary
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_loadPluginLibrary

Load a dynamic library. The dynamic library is assumed to register one or more plugins.

.. _mj_loadAllPluginLibraries:

mj_loadAllPluginLibraries
~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_loadAllPluginLibraries

Scan a directory and load all dynamic libraries. Dynamic libraries in the specified directory
are assumed to register one or more plugins. Optionally, if a callback is specified, it is called
for each dynamic library encountered that registers plugins.

.. _mj_version:

mj_version
~~~~~~~~~~

.. mujoco-include:: mj_version

Return version number: 1.0.2 is encoded as 102.

.. _mj_versionString:

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

geomgroup is an array of length mjNGROUP, where 1 means the group should be included. Pass geomgroup=``NULL`` to skip
group exclusion.
If flg_static is 0, static geoms will be excluded.
bodyexclude=-1 can be used to indicate that all bodies are included.

.. _mj_rayHfield:

mj_rayHfield
~~~~~~~~~~~~

.. mujoco-include:: mj_rayHfield

Intersect ray with hfield, return nearest distance or -1 if no intersection.

.. _mj_rayMesh:

mj_rayMesh
~~~~~~~~~~

.. mujoco-include:: mj_rayMesh

Intersect ray with mesh, return nearest distance or -1 if no intersection.

.. _mju_rayGeom:

mju_rayGeom
~~~~~~~~~~~

.. mujoco-include:: mju_rayGeom

Intersect ray with pure geom, return nearest distance or -1 if no intersection.

.. _mju_raySkin:

mju_raySkin
~~~~~~~~~~~

.. mujoco-include:: mju_raySkin

Intersect ray with skin, return nearest distance or -1 if no intersection,
and also output nearest vertex id.

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

Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise.
Write d->qpos only if flg_paused and subtree root for selected body has free joint.

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

Initialize given geom fields when not ``NULL``, set the rest to their default values.

.. _mjv_makeConnector:

mjv_makeConnector
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_makeConnector

Set (type, size, pos, mat) for connector-type geom between given points.
Assume that mjv_initGeom was already called to set all other properties.

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

Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN.
If only one buffer is available, set that buffer and ignore framebuffer argument.

.. _mjr_readPixels:

mjr_readPixels
~~~~~~~~~~~~~~

.. mujoco-include:: mjr_readPixels

Read pixels from current OpenGL framebuffer to client buffer.
Viewport is in OpenGL framebuffer; client buffer starts at (0,0).

.. _mjr_drawPixels:

mjr_drawPixels
~~~~~~~~~~~~~~

.. mujoco-include:: mjr_drawPixels

Draw pixels from client buffer to current OpenGL framebuffer.
Viewport is in OpenGL framebuffer; client buffer starts at (0,0).

.. _mjr_blitBuffer:

mjr_blitBuffer
~~~~~~~~~~~~~~

.. mujoco-include:: mjr_blitBuffer

Blit from src viewpoint in current framebuffer to dst viewport in other framebuffer.
If src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR.

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
~~~~~~~~~

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

.. _mjui_add:

mjui_add
~~~~~~~~

.. mujoco-include:: mjui_add

Add definitions to UI.

.. _mjui_addToSection:

mjui_addToSection
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_addToSection

Add definitions to UI section.

.. _mjui_resize:

mjui_resize
~~~~~~~~~~~

.. mujoco-include:: mjui_resize

Compute UI sizes.

.. _mjui_update:

mjui_update
~~~~~~~~~~~

.. mujoco-include:: mjui_update

Update specific section/item; -1: update all.

.. _mjui_event:

mjui_event
~~~~~~~~~~

.. mujoco-include:: mjui_event

Handle UI event, return pointer to changed item, ``NULL`` if no change.

.. _mjui_render:

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

.. _mju_warning_i:

mju_warning_i
~~~~~~~~~~~~~

.. mujoco-include:: mju_warning_i

Warning function with int argument.

.. _mju_warning_s:

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

High-level warning function: count warnings in ``mjData``, print only the first.

.. _mju_writeLog:

mju_writeLog
~~~~~~~~~~~~

.. mujoco-include:: mju_writeLog

Write [datetime, type: message] to MUJOCO_LOG.TXT.

.. _Activation:

Activation
^^^^^^^^^^

The functions in this section are maintained for backward compatibility with the now-removed activation mechanism.

.. _mj_activate:

mj_activate
~~~~~~~~~~~

.. mujoco-include:: mj_activate

Return 1 (for backward compatibility).

.. _mj_deactivate:

mj_deactivate
~~~~~~~~~~~~~

.. mujoco-include:: mj_deactivate

Do nothing (for backward compatibility).

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

.. _mju_zero3:

mju_zero3
~~~~~~~~~

.. mujoco-include:: mju_zero3

Set res = 0.

.. _mju_copy3:

mju_copy3
~~~~~~~~~

.. mujoco-include:: mju_copy3

Set res = vec.

.. _mju_scl3:

mju_scl3
~~~~~~~~

.. mujoco-include:: mju_scl3

Set res = vec*scl.

.. _mju_add3:

mju_add3
~~~~~~~~

.. mujoco-include:: mju_add3

Set res = vec1 + vec2.

.. _mju_sub3:

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

.. _mju_normalize3:

mju_normalize3
~~~~~~~~~~~~~~

.. mujoco-include:: mju_normalize3

Normalize vector, return length before normalization.

.. _mju_norm3:

mju_norm3
~~~~~~~~~

.. mujoco-include:: mju_norm3

Return vector length (without normalizing the vector).

.. _mju_dot3:

mju_dot3
~~~~~~~~

.. mujoco-include:: mju_dot3

Return dot-product of vec1 and vec2.

.. _mju_dist3:

mju_dist3
~~~~~~~~~

.. mujoco-include:: mju_dist3

Return Cartesian distance between 3D vectors pos1 and pos2.

.. _mju_rotVecMat:

mju_rotVecMat
~~~~~~~~~~~~~

.. mujoco-include:: mju_rotVecMat

Multiply vector by 3D rotation matrix: res = mat * vec.

.. _mju_rotVecMatT:

mju_rotVecMatT
~~~~~~~~~~~~~~

.. mujoco-include:: mju_rotVecMatT

Multiply vector by transposed 3D rotation matrix: res = mat' * vec.

.. _mju_cross:

mju_cross
~~~~~~~~~

.. mujoco-include:: mju_cross

Compute cross-product: res = cross(a, b).

.. _mju_zero4:

mju_zero4
~~~~~~~~~

.. mujoco-include:: mju_zero4

Set res = 0.

.. _mju_unit4:

mju_unit4
~~~~~~~~~

.. mujoco-include:: mju_unit4

Set res = (1,0,0,0).

.. _mju_copy4:

mju_copy4
~~~~~~~~~

.. mujoco-include:: mju_copy4

Set res = vec.

.. _mju_normalize4:

mju_normalize4
~~~~~~~~~~~~~~

.. mujoco-include:: mju_normalize4

Normalize vector, return length before normalization.

.. _mju_zero:

mju_zero
~~~~~~~~

.. mujoco-include:: mju_zero

Set res = 0.

.. _mju_fill:

mju_fill
~~~~~~~~

.. mujoco-include:: mju_fill

Set res = val.

.. _mju_copy:

mju_copy
~~~~~~~~

.. mujoco-include:: mju_copy

Set res = vec.

.. _mju_sum:

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

.. _mju_add:

mju_add
~~~~~~~

.. mujoco-include:: mju_add

Set res = vec1 + vec2.

.. _mju_sub:

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

.. _mju_normalize:

mju_normalize
~~~~~~~~~~~~~

.. mujoco-include:: mju_normalize

Normalize vector, return length before normalization.

.. _mju_norm:

mju_norm
~~~~~~~~

.. mujoco-include:: mju_norm

Return vector length (without normalizing vector).

.. _mju_dot:

mju_dot
~~~~~~~

.. mujoco-include:: mju_dot

Return dot-product of vec1 and vec2.

.. _mju_mulMatVec:

mju_mulMatVec
~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatVec

Multiply matrix and vector: res = mat * vec.

.. _mju_mulMatTVec:

mju_mulMatTVec
~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatTVec

Multiply transposed matrix and vector: res = mat' * vec.

.. _mju_mulVecMatVec:

mju_mulVecMatVec
~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulVecMatVec

Multiply square matrix with vectors on both sides: returns vec1' * mat * vec2.

.. _mju_transpose:

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

Multiply matrices: res = mat1 * mat2.

.. _mju_mulMatMatT:

mju_mulMatMatT
~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatMatT

Multiply matrices, second argument transposed: res = mat1 * mat2'.

.. _mju_mulMatTMat:

mju_mulMatTMat
~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatTMat

Multiply matrices, first argument transposed: res = mat1' * mat2.

.. _mju_sqrMatTD:

mju_sqrMatTD
~~~~~~~~~~~~

.. mujoco-include:: mju_sqrMatTD

Set res = mat' * diag * mat if diag is not ``NULL``, and res = mat' * mat otherwise.

.. _mju_transformSpatial:

mju_transformSpatial
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_transformSpatial

Coordinate transform of 6D motion or force vector in rotation:translation format.
rotnew2old is 3-by-3, ``NULL`` means no rotation; flg_force specifies force or motion type.

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

Conjugate quaternion, corresponding to opposite rotation.

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

Conjugate pose, corresponding to the opposite spatial transformation.

.. _mju_trnVecPose:

mju_trnVecPose
~~~~~~~~~~~~~~

.. mujoco-include:: mju_trnVecPose

Transform vector by pose.

.. _Decompositions:

Decompositions / Solvers
^^^^^^^^^^^^^^^^^^^^^^^^

.. _mju_cholFactor:

mju_cholFactor
~~~~~~~~~~~~~~

.. mujoco-include:: mju_cholFactor

Cholesky decomposition: mat = L*L'; return rank, decomposition performed in-place into mat.

.. _mju_cholSolve:

mju_cholSolve
~~~~~~~~~~~~~

.. mujoco-include:: mju_cholSolve

Solve mat * res = vec, where mat is Cholesky-factorized

.. _mju_cholUpdate:

mju_cholUpdate
~~~~~~~~~~~~~~

.. mujoco-include:: mju_cholUpdate

Cholesky rank-one update: L*L' +/- x*x'; return rank.

.. _mju_eig3:

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

.. _mju_clip:

mju_clip
~~~~~~~~

.. mujoco-include:: mju_clip

Clip x to the range [min, max].

.. _mju_sign:

mju_sign
~~~~~~~~

.. mujoco-include:: mju_sign

Return sign of x: +1, -1 or 0.

.. _mju_round:

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

Return human readable number of bytes using standard letter suffix.

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

.. _mju_f2n:

mju_f2n
~~~~~~~

.. mujoco-include:: mju_f2n

Convert from float to mjtNum.

.. _mju_n2f:

mju_n2f
~~~~~~~

.. mujoco-include:: mju_n2f

Convert from mjtNum to float.

.. _mju_d2n:

mju_d2n
~~~~~~~

.. mujoco-include:: mju_d2n

Convert from double to mjtNum.

.. _mju_n2d:

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

.. _mju_strncpy:

mju_strncpy
~~~~~~~~~~~

.. mujoco-include:: mju_strncpy

Call strncpy, then set dst[n-1] = 0.

.. _mju_sigmoid:

mju_sigmoid
~~~~~~~~~~~

.. mujoco-include:: mju_sigmoid

Sigmoid function over 0<=x<=1 constructed from half-quadratics.

.. _Derivatives-api:

Derivatives
^^^^^^^^^^^

.. _mjd_transitionFD:

mjd_transitionFD
~~~~~~~~~~~~~~~~

.. mujoco-include:: mjd_transitionFD

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

- All four matrix outputs are optional (can be ``NULL``).
- ``eps`` is the finite-differencing epsilon.
- ``centered`` is a flag denoting whether to use forward (0) or centered (1) differences.

.. _Plugins-api:

Plugins
^^^^^^^
.. _mjp_defaultPlugin:

mjp_defaultPlugin
~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_defaultPlugin

Set default plugin definition.

.. _mjp_registerPlugin:

mjp_registerPlugin
~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_registerPlugin

Globally register a plugin. This function is thread-safe.
If an identical mjpPlugin is already registered, this function does nothing.
If a non-identical mjpPlugin with the same name is already registered, an mju_error is raised.
Two mjpPlugins are considered identical if all member function pointers and numbers are equal,
and the name and attribute strings are all identical, however the char pointers to the strings
need not be the same.

.. _mjp_pluginCount:

mjp_pluginCount
~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_pluginCount

Return the number of globally registered plugins.

.. _mjp_getPlugin:

mjp_getPlugin
~~~~~~~~~~~~~

.. mujoco-include:: mjp_getPlugin

Look up a plugin by name. If slot is not ``NULL``, also write its registered slot number into it.

.. _mjp_getPluginAtSlot:

mjp_getPluginAtSlot
~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_getPluginAtSlot

Look up a plugin by the registered slot number that was returned by mjp_registerPlugin.

