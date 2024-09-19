============
USD Exporter
============

Introduction
------------

The MuJoCo `USD exporter <https://github.com/google-deepmind/mujoco/tree/main/python/mujoco/usd>`_ allows users to save scenes and trajectories in USD format for rendering in external renderers such as NVIDIA Omniverse or Blender. These renderers provide higher quality rendering capabilities not provided with MuJoCo's default renderer. Some of these capabilities include real time ray tracing and and path tracing. Additionally, exporting to USD allows users to include different types of texture maps to make objects in the scene look more realistic.

.. _USDInstallation:

Installation instructions
-------------------------

The recommended way to install the necessary requirements for the USD exporter is via `PyPI <https://pypi.org/project/mujoco/>`_:

.. code-block:: shell

   pip install mujoco[usd]

This installs the optional dependencies ``usd-core`` and ``pillow`` required by the USD exporter. If you are building from source, please ensure to `build the Python bindings <https://mujoco.readthedocs.io/en/stable/python.html#building-from-source>`_. Then, using pip, install the required ``usd-core`` and ``pillow`` packages.

.. _USDExporter:

USDExporter
----------------

In order to export MuJoCo scenes to USD, use the ``mujoco.usd.exporter`` module. Specifically, ``USDExporter`` allows you save full trajectories from MuJoCo in addition to defining custom cameras and lights. The arguments to the USDExporter are the following:

- ``model`` : an MjModel instance. The USD exporter reads relevant information from the model including details about cameras, lights, textures, and object geometries to name a few. 

- ``max_geom`` : defines the maximum number of geoms in a scene. This argument is only used when create an MuJoCo scene. Please refer to `mjvScene <https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjvscene>`_ for more details. 

- ``output_directory_name`` : defines the name of the directory under which the exported USD file and all relevant assets are stored. When saving a scene/trajectory as a USD file, the exporter creates the following directory structure.

    .. code-block:: text

        output_directory_root/
        └-output_directory_name/
        ├-assets/
        | ├-texture_0.png
        | ├-texture_1.png
        | └-...
        └─frames/
            └-frame_301.usd

    Using this file structure allows users to easily zip the directory ``output_directory_name`` and share it with other machines. All paths to assets in the USD file are relative paths. Thus, even if unzipped and rendered on another machine, the USD file still has access to the necessary assets. This is especially important when rendering offline on a server.

- ``output_directory_root`` : defines the root directory to add USD trajectory to.

- ``light_intensity`` : intensity of the lights in the renderer. Note that intensity may be defined differently in different renderers. As such, it might be valuable to set the value for light intensity based on your renderer of choice. 

- ``camera_names`` : a list of cameras to be stored in the USD file. At each time step, for each camera defined, we calculate its position and orientation and add that value for that given frame in the USD. USD allows us to store multiple cameras. However, when rendering, we typically only render with a singular camera at a time.

- ``verbose`` : decides whether or not to print updates of the exporter.

If you wish to export a MuJoCo model loaded directly from an MJCF, we provide a `demo <https://github.com/abhihjoshi/mujoco/blob/main/python/mujoco/usd/demo.py>`_ file in the ``usd`` directory of the repository to do so. This demo file also serves as a good example to how to use the USD export functionality.

.. _USDBasicUsage:

Basic usage
-----------

Once the optional dependencies are installed, the USD exporter can be imported via ``from mujoco.usd import exporter``. 

``USDExporter`` handles all logic related to creating USD files. When instantiated, it will create the necessary output directories along with all the textures used by the model. In order to make an update to the USD trajectory, call the `update_scene` function. We describe the interface for this function in the USD Export API section of this page. Updating the scene updates all geoms, lights, and cameras in the scene with their pose. Additionally, we account for visibility of the geom. We store these updates for a specific frame. We maintain an interal counter of the number of times `update_scene` is called. This internal counter serves as a reference for the current frame. This means that you can step through the MuJoCo simulation for multiple time steps before updating the USD scene. Only when we call `update_scene` do we store any values in our USD file. 

.. _USDExportAPI:

USD Export API
--------------

- ``update_scene(self, data, scene_option)`` : updates the scene with the latest simulation data passed in by the user. This function updates the geom, cameras, and lights in the scene.

- ``add_light(self, pos, intensity, radius, color, obj_name, light_type)`` : adds a light to the USD scene with the given properties post hoc. 

- ``add_camera(self, pos, rotation_xyz, obj_name)`` : adds a camera to the USD scene with the given properties post hoc. 

- ``save_scene(self, filetype)`` :  exports the USD scene using one of the usd filetype extensions .usd, .usda, or .usdc 

.. _USDTodos:

TODOs
-----

In this section, we list remaining action items for the USD exporter. Please feel free to suggest additional requests by creating a new `feature request <https://github.com/google-deepmind/mujoco/issues/new/choose>`_ in the GitHub repository.

- Add support for additional texture maps including metallic, occlusion, roughness, bump, etc.

- Add support for online rendering with Isaac.

- Add support for custom cameras