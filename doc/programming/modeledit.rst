Model Editing
-------------

.. admonition:: Unstable API
   :class: attention

   The API described below is new and unstable. There may be latent bugs and function signatures may change. Early
   adopters are welcome (indeed, encouraged) to try it out and report any issues on GitHub.

As of MuJoCo 3.2, it is possible to create and modify models using the :ref:`mjSpec` struct and related API.
This datastructure is in one-to-one correspondence with MJCF and indeed, MuJoCo's own XML parsers (both MJCF and URDF)
use this API when loading a model.


.. _meOverview:

Overview
~~~~~~~~

The new API augments the traditional workflow of creating and editing models using XML files, breaking up the *parse* and
*compile* steps. As summarized in the the :ref:`Overview chapter<Instance>`, the traditional workflow is:

 1. Create an XML model description file (MJCF or URDF) and associated assets. |br|
 2. Call :ref:`mj_loadXML`, obtain an :ref:`mjModel` instance.

The new workflow is:

 1. :ref:`Create<mj_makeSpec>` an empty :ref:`mjSpec` or :ref:`parse<mj_parseXML>` an existing XML file to an
    :ref:`mjSpec`.
 2. Edit the mutable :ref:`mjSpec` datastructure adding, changing and removing elements.
 3. Compile the :ref:`mjSpec` at any point, obtaining an updated :ref:`mjModel` instance. After compilation, the
    :ref:`mjSpec` remains editable, so steps 2 and 3 are interchangable.


.. _meUsage:

Usage
~~~~~
Here we describe the C API for procedural model editing, but it is also exposed in the
:ref:`Python bindings<PyModelEdit>`.
After creating a new :ref:`mjSpec` or parsing an existing XML file to an :ref:`mjSpec`, procedural editing corresponds
to setting attributes. For example, in order to change the timestep, one can do:

.. code-block:: C

   mjSpec* spec = mj_makeSpec();
   spec->opt.timestep = 0.01;
   ...
   mjModel* model = mj_compile(spec);

Attributes which have variable length are C++ vectors and strings, :ref:`exposed to C as opaque types<ArrayHandles>`.
In C one uses the provided :ref:`getters<AttributeGetters>` and :ref:`setters<AttributeSetters>`:

.. code-block:: C

   mjs_setString(model->modelname, "my_model");

In C++ one can use these directly:

.. code-block:: C++

   std::string modelname = "my_model";
   *spec->modelname = modelname;

.. _meMjsElements:

Model elements
^^^^^^^^^^^^^^

Model elements corresponding to MJCF are added to the spec using the corresponding functions. For example, to add a box
geom to the world body, one would do

.. code-block:: C

   mjSpec* spec = mj_makeSpec();
   mjsBody* world = mjs_findBody(spec, "world");
   mjsGeom* my_geom = mjs_addGeom(world, NULL);
   my_geom->type = mjGEOM_BOX;
   my_geom->size[0] = my_geom->size[1] = my_geom->size[2] = 0.5;
   mjModel* model = mj_compile(spec);

The ``NULL`` second argument to :ref:`mjs_addGeom` is the optional default class pointer. When using defaults
procedurally, default classes are passed in explicitly to element constructors. The global defaults of all elements
(used when no default class is passed in) can be inspected in
`user_init.c <https://github.com/google-deepmind/mujoco/blob/main/src/user/user_init.c>`__.


.. _meAttachment:

Attachment
^^^^^^^^^^
The new framework introduces a powerful new feature: attaching and detaching model subtrees. Attachment allows the user
copy a subtree from one model into another, while also copying related referenced assets and referencing elements from
outside the kinematic tree (e.g., actuators and sensors). Similarly, detaching a subtree will remove all associated
elements from the model.

This feature is incomplete and will be described in detail once it is fully implemented, but it is already used to power
the :ref:`attach<body-attach>` and :ref:`replicate<replicate>` meta-elements in MJCF.


.. _meKnownIssues:

Known issues
~~~~~~~~~~~~

- Better documentation is still missing and will be added in the future. In the meantime, advanced users can refer
  to `user_api_test.cc <https://github.com/google-deepmind/mujoco/blob/main/test/user/user_api_test.cc>`__ and the MJCF
  parser in `xml_native_reader.cc <https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_native_reader.cc>`__,
  which is already using this API.
- One of the central design considerations of the new API is incremental compilation, meaning that after making small
  changes to a spec that has already been compiled, subsequent re-compilation will be very fast. While the code is
  written to support incremental compilation, this functionality is not fully implemented and will be added in the
  future, resulting in faster re-compilation times.
- Since the main test for the new API is the MJCF parser, which always constructs a model from scratch, there
  might be latent bugs related to model editing. Please report such bugs if you encounter them.
