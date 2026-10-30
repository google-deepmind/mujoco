.. _StudioNativeViewers:
.. _StudioLaunchers:

==============
Native Viewers
==============

In Python, ``NativeViewer`` (`native_viewer.py
<https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/studio/native_viewer.py>`__) renders both the 3D
scene and `Dear ImGui <https://github.com/ocornut/imgui>`__ UI locally in an SDL2 desktop window or offscreen buffer via
:ref:`Filament <FilamentRendering>`. Use **blocking mode** (``launch_native.run``) when Studio drives the simulation
loop, or **non-blocking mode** (``launch_native.launch``) when attaching a viewer to your own Python loop. For the
standalone C++ desktop executable ``mujoco_studio``, see :ref:`C++ <StudioCpp>`.

Blocking Mode
=============

To open the standalone Studio viewer from the command line with optional :ref:`plugin files <StudioPythonPlugins>` and
:ref:`hot reload <StudioHotReload>`:

.. code-block:: shell

   python -m mujoco.studio.viewer \
     --model=path/to/model.xml \
     --gfx=opengl \
     --plugins=path/to/my_plugin.py \
     --hot_reload=all

From Python, `launch_native.run
<https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/studio/launch_native.py>`__ starts ``NativeViewer``
and runs the blocking simulation sync loop until the window closes. Pass ``model_path=...``
to load a model file on the simulation thread so UI **Reload** and model :ref:`hot reload <StudioHotReload>` stay
enabled, or pass an in-memory ``model=...`` with optional ``data=...``:

.. code-block:: python

   from mujoco.studio import app as studio_app
   from mujoco.studio import launch_native
   from mujoco.studio import viewer_protocol

   config = viewer_protocol.ViewerConfig(title='MuJoCo Studio', gfx='opengl')

   launch_native.run(
       config,
       model_path='humanoid.xml',  # Or model=model, data=data for an in-memory model.
       plugins=[studio_app.App(), *studio_app.SIM_PLUGINS],
   )

.. _StudioExecutionModes:

Non-Blocking Mode
=================

When attaching ``NativeViewer`` to an existing Python loop—such as an RL training loop, trajectory replay script, or
test—use `launch_native.launch
<https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/studio/launch_native.py>`__ as a non-blocking context
manager. It starts ``NativeViewer`` on a daemon thread and returns a ``ViewerHandle`` (`viewer_handle.py
<https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/studio/viewer_handle.py>`__) to the calling thread:

.. code-block:: python

   import time
   import mujoco
   from mujoco.studio import app as studio_app
   from mujoco.studio import launch_native

   model = mujoco.MjModel.from_xml_path('humanoid.xml')
   data = mujoco.MjData(model)

   with launch_native.launch(plugins=[studio_app.App()]) as handle:
     while handle.is_running():
       tick = time.perf_counter()
       model, data = handle.sync(model, data)
       mujoco.mj_step(model, data)
       time.sleep(max(0.0, model.opt.timestep - (time.perf_counter() - tick)))

Key details when driving a ``ViewerHandle`` loop directly:

- **Always rebind** ``model, data = handle.sync(model, data)`` on every iteration. ``sync()`` publishes ``model`` and
  ``data`` if your script created a new model object, and returns replacement ``model`` and ``data`` objects whenever UI
  **Reload**, a dropped file, or a plugin's ``handle.set_model()`` swaps the active model.
- **Stepping and pacing**: ``handle.sync()`` never sleeps. If you omit ``StepControl()``, advance ``data`` and pace the
  loop yourself after ``handle.sync()``; if you include ``*studio_app.SIM_PLUGINS``, ``StepControl()`` steps and paces
  physics inside ``handle.sync()`` during ``StepEvent``.
- **Mutations**: Keep ``model.opt`` mutations inside step-event handlers so ``ViewerHandle.sync()``'s
  :ref:`diff-based sync <StudioDiffSync>` detects and forwards them to the viewer.
- **macOS (``mjpython``)**: AppKit requires native windows to be created and serviced on the OS main thread. Blocking
  mode (``python -m mujoco.studio.viewer`` and ``launch_native.run``) works under standard ``python``, whereas
  non-blocking ``launch_native.launch`` requires running your script with ``mjpython`` (unlike :ref:`launch_web
  <StudioWebViewer>`, which works under standard ``python``).

See `custom_sim_loop.py
<https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/sample/studio/custom_sim_loop.py>`__ for a complete
non-blocking example that mirrors viewer pause and playback-speed controls while dynamically rebuilding the scene.

.. _StudioBackends:

Rendering Backends
==================

Set ``ViewerConfig.gfx`` or pass ``--gfx`` on the command line to select the native :ref:`Filament
<FilamentRendering>` backend:

- ``''`` *(default)*: Uses the platform default native Filament backend.
- ``'opengl'`` / ``'vulkan'``: Hardware-accelerated OpenGL or Vulkan desktop window.
- ``'opengl_headless'``: Offscreen OpenGL context without a visible window, suited for automated tests and offscreen
  rendering.
- ``'opengl_software'`` / ``'vulkan_software'``: Software-rasterized OpenGL or Vulkan for headless CI machines without
  GPUs.

On Linux, direct window rendering expects X11 (``'opengl'`` is rejected under Wayland). In a Wayland session, omitting
``gfx`` falls back to ``'opengl_headless'``; for direct GPU presentation, use an X11 session.
