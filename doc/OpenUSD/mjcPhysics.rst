mjcPhysics
==========

.. WARNING:: OpenUSD support is currently experimental and subject to frequent change.

The ``mjcPhysics`` `schema <https://openusd.org/release/api/_usd__page__generating_schemas.html>`__ allows for detailed
specification of a MuJoCo simulation environment directly within a USD file. The aim is not to replace `UsdPhysics
<https://openusd.org/release/api/usd_physics_page_front.html>`__, but to extend existing concepts and create new types
only where is necessary.

The schema can be use `codeless <https://openusd.org/dev/api/_usd__page__generating_schemas.html#Codeless_Schemas>`__,
or can be built with its C++ bindings. We've pre-generated `the code
<https://github.com/google-deepmind/mujoco/tree/main/src/experimental/usd/mjcPhysics>`__ via `usdGenSchema
<https://openusd.org/dev/api/_usd__page__generating_schemas.html>`_ for internal MuJoCo usage, but it should also work
outside of MuJoCo.

API Schemas
-----------

MjcSceneAPI
^^^^^^^^^^^

This API schema provides global options for the MuJoCo simulation. It is an
amalgamation of the ``<option>``, ``<option/flag>`` and ``<compiler>`` elements in
MJCF. Users should apply this to an existing
`UsdPhysicsScene <https://openusd.org/dev/api/class_usd_physics_scene.html>`__
prim.

Key attributes include:

-   **mjc:option**: Attributes in this namespace map to the ``<option>`` element.
-   **mjc:flag**: Attributes in this namespace map to the ``<option/flag>`` element.
-   **mjc:compiler**: Attributes in this namespace map to the ``<compiler>`` element.

MjcSiteAPI
^^^^^^^^^^

This API class is used to define a MuJoCo site, it can be applied to
`UsdGeomSphere <https://openusd.org/dev/api/class_usd_geom_sphere.html>`__,
`UsdGeomCapsule <https://openusd.org/dev/api/class_usd_geom_capsule.html>`__,
`UsdGeomCylinder <https://openusd.org/dev/api/class_usd_geom_cylinder.html>`__, and
`UsdGeomCube <https://openusd.org/dev/api/class_usd_geom_cube.html>`__.

MjcImageableAPI
^^^^^^^^^^^^^^^

This API class provides attributes for strictly visual entities in MuJoCo, in
MuJoCo terms we would quantify these has having ``contype = conaffinity = 0``.

MjcCollisionAPI
^^^^^^^^^^^^^^^

This API class is applied to prims that represent collision geometry and should
be applied alongside
`UsdPhysicsCollisionAPI <https://openusd.org/dev/api/class_usd_physics_collision_a_p_i.html>`__.

MjcMeshCollisionAPI
^^^^^^^^^^^^^^^^^^^

This API class is applied to prims that represent mesh collision geometry and
should be applied alongside
`UsdPhysicsMeshCollisionAPI <https://openusd.org/dev/api/class_usd_physics_mesh_collision_a_p_i.html>`__.

MjcJointAPI
^^^^^^^^^^^

This API class is applied to `UsdPhysicsJoint <https://openusd.org/dev/api/class_usd_physics_joint.html>`__ prims,
adding extra attributes to fully describe MuJoCo joints.

MjcMaterialAPI
^^^^^^^^^^^^^^

This API class provides attributes for physical materials and is an extension of `UsdPhysicsMaterialAPI
<https://openusd.org/dev/api/class_usd_physics_material_a_p_i.html>`__

Type Schemas
------------

MjcActuator
^^^^^^^^^^^

This class represents a MuJoCo actuator, which is responsible for applying force to a transmission target joint, body,
or site specific via a `relationship <https://openusd.org/dev/api/class_usd_relationship.html>`__.

We do not use the existing `UsdPhysicsDriveAPI <https://openusd.org/dev/api/class_usd_physics_drive_a_p_i.html>`__ as it
is closer to a runtime construct and the concepts do not map very closely.

MjcKeyframe
^^^^^^^^^^^

This type holds tensor values representing simulator state at specific time values.

In MJCF this is the ``<keyframe>`` element and has a ``time`` attribute. In USD we map the time attribute to
`timeSamples <https://openusd.org/release/tut_xforms.html>`__ instead.

The order of the values in the keyframes should map to the depth first ordered traversal of rigidbodies in the composed
stage.

MjcTendon
^^^^^^^^^

This type represents both fixed and spatial tendons.

In MJCF this is the ``<tendon>`` element. The tendon path is represented by the ordered list of targets in the
``mjc:path`` relationship attribute. In MJCF we can specify attributes such as ``sidesite`` and ``divisor`` on
path targets; but in USD we cannot attach data to relationship attributes as elegantly, so these become indexed
array attributes such as ``mjc:sideSites`` and ``mjs:path:divisors``.
