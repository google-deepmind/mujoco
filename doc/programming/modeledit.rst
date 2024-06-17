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

As summarized in the the :ref:`Overview chapter<Instance>`, the traditional workflow to create compiled :ref:`mjModel`
instances is:

1. Create an XML model description file (MJCF or URDF).
2. Call :ref:`mj_loadXML` passing in the XML (and associated assets), obtain an :ref:`mjModel` instance.

The new workflow looks like:

1. Create an :ref:`mjSpec`, either an empty one corresponding to the XML ``<mujoco/>``, or by loading an existing XML
   file.
2. Modify the :ref:`mjSpec` as desired, adding, editing and removing elements.
3. Compile the :ref:`mjSpec` at any point, obtaining an updated :ref:`mjModel` instance. After compilation, the
   :ref:`mjSpec` remains editable, so steps 2 and 3 are interchangable.


.. _meUsage:

Usage
~~~~~

Detailed documentation is still missing. In the meantime, advanced users can refer to
`user_api_test.cc <https://github.com/google-deepmind/mujoco/blob/main/test/user/user_api_test.cc>`__ and the MJCF
parser in `xml_native_reader.cc <https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_native_reader.cc>`__,
which is already using this API.

