File Format Plugin
=========================

What is an SdfFileFormat plugin?
--------------------------------

In the OpenUSD framework, ``Sdf`` stands for Scene Description Foundations. It's the underlying layer that handles the
serialization and composition of scene data. A ``SdfFileFormat`` plugin is a component that teaches USD how to read and
write a specific file format.

By default, USD comes with plugins for its own formats (``.usda``, ``.usdc``, ``.usdz``) and the community has created
several plugin extension such as the `Adobe File Format Plugins
<https://github.com/adobe/USD-Fileformat-plugins/tree/main>`__.

The MJCF ``SdfFileFormat`` plugin allows USD-aware applications to directly understand and interact with MuJoCo's native
``.xml`` (MJCF) files as if they were native USD files.

What does it enable?
------------------------

This plugin enables:

1.  **Referencing MJCF files in USD:** Using standard USD composition arcs (like references, payloads) to include an
    MJCF file directly within a larger USD scene. For example, you can place a MuJoCo robot defined in an ``.xml`` file
    into a room scene modeled in USD.
2.  **Load MJCF files in USD tools:** Tools like ``usdview`` or other USD-based
    applications can open, inspect, and render MJCF files, translating the MJCF elements into USD prims and attributes
    on the fly.
3.  **Convert MJCF to USD:** The plugin can be used as a basis for converting MJCF files to persistent
    USD files (e.g., ``.usda`` or ``.usdc``).

Essentially, it makes MJCF a first-class citizen in the USD ecosystem.

Usage
------------------

1.  **Installation:** refer to :doc:`building`.

2.  **Referencing in a USD file (e.g., ``.usda``):**

    .. code-block:: usd
        :caption: example.usda

        #usda 1.0
        (
            upAxis = "Z"
        )

        def Xform "world"
        {
            def "robot" (
                prepend references = @./my_robot.xml@
            )
            {
            }
        }

    In this example, ``my_robot.xml`` is an MJCF file in the same directory. USD will use the plugin to load and
    interpret its contents.

3.  **Opening in usdview:**

    .. code-block:: bash

       usdview my_robot.xml

    If the plugin is correctly set up, ``usdview`` will render the robot defined in the MJCF file.

4.  **Using in Python (with USD API):**

    .. code-block:: python

        from pxr import Usd

        # Load an MJCF file as a USD stage
        stage = Usd.Stage.Open('my_robot.xml')

        if stage:
            print(f"Successfully opened {stage.GetRootLayer().identifier}")
            # You can now inspect the stage as any other USD stage
            for prim in stage.TraverseAll():
                print(prim.GetPath())
        else:
            print("Failed to open MJCF file")

    This plugin significantly enhances the interoperability between MuJoCo and USD-based workflows, allowing
    seamless integration of physics assets defined in MJCF into broader 3D environments.
