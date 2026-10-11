OpenUSD
===========

.. toctree::
    :hidden:

    building
    mjcPhysics
    mjcf_file_format_plugin
    importing
    exporting

.. WARNING:: OpenUSD support is currently experimental and subject to frequent change.

Introduction
------------

This chapter describes MuJoCo's support for `OpenUSD <https://openusd.org/release/intro.html>`__. USD (Universal Scene
Description) is an open-source framework developed by Pixar for describing 3D scenes. MuJoCo's integration allows users
to leverage USD's rich ecosystem and tooling.

What is OpenUSD?
----------------

USD is a high-performance, extensible system for describing, composing, simulating, and collaborating on 3D data.
Originally developed by Pixar Animation Studios, USD is now used across various industries, including visual effects,
animation, gaming, and robotics, to streamline complex 3D workflows. It provides a common language for different
software applications to exchange 3D scene information.

Why do we care about OpenUSD?
-----------------------------

Integrating USD with MuJoCo offers several advantages:

*   **Interoperability:** USD is supported by a wide range of 3D content creation tools (e.g., Houdini, Maya, Blender).
    This allows MuJoCo users to easily import scenes and assets created in these tools.
*   **Rich Scene Description:** USD provides a powerful and flexible way to represent complex scenes, including
    geometry, materials, lighting, and hierarchies.
*   **Collaboration:** USD's layering and composition features enable powerful and efficient
    non-destructive authoring pipelines.

USD support overview
------------------------------------------------------

*   **Import:** You can load USD assets (specifically ``.usd``, ``.usda``, ``.usdc``, ``.usdz`` files) into MuJoCo via
    MJCF or dragging and dropping into :ref:`simulate.cc <saSimulate>`.
*   **Schemas:** MuJoCo primarily uses the standard `UsdPhysics
    <https://openusd.org/dev/api/usd_physics_page_front.html>`__ schemas for representing physics properties.
*   **Extensions:** Custom :doc:`mjcPhysics` schemas are provided to cover MuJoCo-specific features not available in
    ``UsdPhysics``.
*   **MJCF File Format Plugin:** A :doc:`mjcf_file_format_plugin` allows treating MJCF files as USD layers in any native USD
    application.
*   **Export:** MuJoCo scenes can be exported to USD.

Where do I learn more about USD?
------------------------------------------

*   `Remedy's Book of USD <https://remedy-entertainment.github.io/USDBook>`__: Friendly introduction to USD.
*   `Official OpenUSD Documentation <https://openusd.org/release/intro.html>`__: Official documentation for API and
    implementation details.
*   `Pixar's USD Introduction <https://graphics.pixar.com/usd/release/index.html>`__: Simple example usage of USD.
*   `NVIDIA's USD Resources <https://developer.nvidia.com/usd>`__: Set of USD resources primarily concerned with asset
    structure.
