.. _saSimulateShortcuts:

Simulate keyboard shortcuts
----------------------------

This page lists the built-in keyboard shortcuts and mouse controls for the MuJoCo **Simulate** viewer.

**Applies to:**

- The standalone ``simulate`` binary.
- The Python viewer window launched via ``mujoco.viewer`` (it uses the same underlying C++ Simulate UI).

.. tip::
   Press ``F1`` in the viewer to display a quick-reference help overlay.


.. _ssSimulationControl:

Simulation control
~~~~~~~~~~~~~~~~~~

.. list-table::
   :width: 95%
   :align: left
   :widths: 2 4 5
   :header-rows: 1

   * - Key
     - Function
     - Notes
   * - ``Space``
     - Play / Pause
     - Managed mode only (viewer owns stepping). In passive mode, stepping is owned by user code.
   * - ``Right Arrow``
     - Step forward
     - Managed mode only. When scrubbing history (negative scrub index), moves scrubber toward 0.
   * - ``Left Arrow``
     - Step backward
     - Managed mode only (uses history scrubber).
   * - ``-``
     - Slow down
     - Decreases the real-time simulation speed percentage.
   * - ``=``
     - Speed up
     - Increases the real-time simulation speed percentage.
   * - ``Backspace``
     - Reset
     - Managed mode only. Resets the simulation to the initial state.
   * - ``Ctrl+L``
     - Reload model
     - Managed mode only. Reloads the current model file from disk.


.. _ssCameraControl:

Camera control
~~~~~~~~~~~~~~

.. list-table::
   :width: 95%
   :align: left
   :widths: 2 4 5
   :header-rows: 1

   * - Key
     - Function
     - Notes
   * - ``Esc``
     - Free camera
     - Switches to the free (unconstrained) camera.
   * - ``[``
     - Cycle fixed cameras down
     - Wraps through model-defined cameras.
   * - ``]``
     - Cycle fixed cameras up
     - Wraps through model-defined cameras.
   * - ``Page Up``
     - Select parent body
     - Requires an active selection. Walks up the kinematic tree.
   * - ``F6``
     - Cycle frame visualization
     - Cycles through ``mjvOption.frame`` modes (None, Body, Geom, Site, Camera, Light, Contact, World).
   * - ``F7``
     - Cycle label visualization
     - Cycles through ``mjvOption.label`` modes (None, Body, Joint, Geom, Site, etc.).


.. _ssUIOverlayToggles:

UI and overlay toggles
~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :width: 95%
   :align: left
   :widths: 2 4
   :header-rows: 1

   * - Key
     - Function
   * - ``Tab``
     - Toggle left UI panel
   * - ``Shift+Tab``
     - Toggle right UI panel
   * - ``F1``
     - Toggle Help overlay
   * - ``F2``
     - Toggle Info overlay
   * - ``F3``
     - Toggle Profiler overlay
   * - ``F4``
     - Toggle Sensors overlay
   * - ``F5``
     - Toggle Fullscreen


.. _ssFileMenuShortcuts:

File and simulation menu shortcuts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :width: 95%
   :align: left
   :widths: 2 4 5
   :header-rows: 1

   * - Key
     - Function
     - Notes
   * - ``Ctrl+M``
     - Print model
     - Saves the compiled model as a text file.
   * - ``Ctrl+D``
     - Print data
     - Saves the current simulation data as a text file.
   * - ``Ctrl+P``
     - Screenshot
     - Captures a screenshot of the current view.
   * - ``Ctrl+Q``
     - Quit
     -
   * - ``Ctrl+A``
     - Align
     - Aligns and scales the free camera to fit the model.
   * - ``Ctrl+C``
     - Copy state
     - Copies the current simulation state to the selected keyframe. Hold ``Shift`` for full precision.


.. _ssVisualizationFlags:

Visualization flag shortcuts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

These single-key toggles control the abstract visualization flags from :ref:`mjVISSTRING`.
Only flags with assigned shortcuts are listed.

.. list-table::
   :width: 95%
   :align: left
   :widths: 1 3
   :header-rows: 1

   * - Key
     - Toggles
   * - ``H``
     - Convex Hull
   * - ``X``
     - Texture
   * - ``J``
     - Joint
   * - ``Q``
     - Camera
   * - ``U``
     - Actuator
   * - ``,``
     - Activation
   * - ``Z``
     - Light
   * - ``V``
     - Tendon
   * - ``Y``
     - Range Finder
   * - ``E``
     - Equality
   * - ``I``
     - Inertia
   * - ``'``
     - Scale Inertia
   * - ``B``
     - Perturb Force
   * - ``O``
     - Perturb Object
   * - ``C``
     - Contact Point
   * - ``N``
     - Island
   * - ``F``
     - Contact Force
   * - ``P``
     - Contact Split
   * - ``T``
     - Transparent
   * - ``A``
     - Auto Connect
   * - ``M``
     - Center of Mass
   * - ``D``
     - Static Body
   * - ``;``
     - Skin
   * - :literal:`\``
     - Body Tree
   * - ``\``
     - Mesh Tree

.. note::
   Visualization flags without a shortcut (Select Point, Flex Vert, Flex Edge, Flex Face, Flex Skin, SDF iters) can
   only be toggled via the UI panel.


.. _ssRenderFlags:

Render flag shortcuts
~~~~~~~~~~~~~~~~~~~~~

These single-key toggles control the OpenGL rendering flags from :ref:`mjRNDSTRING`.
Only flags with assigned shortcuts are listed.

.. list-table::
   :width: 95%
   :align: left
   :widths: 1 3
   :header-rows: 1

   * - Key
     - Toggles
   * - ``S``
     - Shadow
   * - ``W``
     - Wireframe
   * - ``R``
     - Reflection
   * - ``L``
     - Additive
   * - ``K``
     - Skybox
   * - ``G``
     - Fog
   * - ``/``
     - Haze
   * - ``,``
     - Segment

.. note::
   The ``,`` key is shared between the **Activation** visualization flag and the **Segment** render flag. Both will
   toggle simultaneously. Render flags without a shortcut (Depth, Id Color, Cull Face) can only be toggled via the
   UI panel.

.. _ssGroupVisibility:

Group visibility shortcuts
~~~~~~~~~~~~~~~~~~~~~~~~~~

These shortcuts toggle the visibility of geom and site groups in the viewer.

.. list-table::
   :width: 95%
   :align: left
   :widths: 2 4
   :header-rows: 1

   * - Key
     - Function
   * - ``0`` – ``5``
     - Toggle Geom groups 0–5
   * - ``Shift+0`` – ``Shift+5``
     - Toggle Site groups 0–5

.. note::
   Other element groups (Joint, Tendon, Actuator, Flex, Skin) do not have keyboard shortcuts and can only be toggled
   via the Group enable section in the UI panel.


.. _ssMouseInteraction:

Mouse interaction
~~~~~~~~~~~~~~~~~

.. list-table::
   :width: 95%
   :align: left
   :widths: 3 4
   :header-rows: 1

   * - Action
     - Function
   * - Left double-click
     - Select body
   * - Right double-click
     - Center camera on click point
   * - Ctrl + Right double-click
     - Set tracking camera on clicked body
   * - Left drag
     - Rotate camera (orbit)
   * - Right drag
     - Pan camera (vertical plane)
   * - Shift + Right drag
     - Pan camera (horizontal plane)
   * - Scroll / Middle drag
     - Zoom
   * - Ctrl + Left drag
     - Rotate selected object (perturbation)
   * - Ctrl + Right drag
     - Translate selected object (perturbation)
   * - Ctrl + Shift + Left drag
     - Rotate selected object (perturbation, horizontal)
   * - Ctrl + Shift + Right drag
     - Translate selected object (perturbation, horizontal)
   * - UI right-button hold
     - Show UI shortcuts
   * - UI title double-click
     - Expand / collapse all sections


.. _ssPythonExtension:

Python extension point
~~~~~~~~~~~~~~~~~~~~~~

When using ``mujoco.viewer.launch_passive(...)``, you can pass a ``key_callback`` argument to implement custom keyboard
shortcuts in Python. Your callback is invoked on key-down events and can implement any additional shortcuts. The
built-in shortcuts listed above still apply. See the :ref:`passive viewer documentation<PyViewerPassive>` for details.
