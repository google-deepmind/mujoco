.. _FilamentRendering:

Filament Rendering
------------------

MuJoCo provides a renderer based on `Filament <https://github.com/google/filament>`__, Google's real-time physically
based rendering engine. Filament has a small and fast runtime, runs on Linux, Windows, macOS, Android, iOS and the Web,
and supports the OpenGL, Vulkan, Metal and WebGL graphics backends. The Filament renderer powers `MuJoCo Studio
<https://github.com/google-deepmind/mujoco/blob/main/src/experimental/studio>`__ and `live.mujoco.org
<https://live.mujoco.org>`__.

The Filament renderer is an optional, self-contained library named ``mujoco_filament``. It implements the ``mjrf``
:ref:`types<tyFilamentRenderStructure>` and :ref:`functions<FilamentRenderingApi>` defined in `mjrfilament.h
<https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjrfilament.h>`__.

While the filament runtime is very small and efficient, it is a rather large build dependency. As such, if you intend to
:ref:`build MuJoCo from source<inBuild>`, you will need to set the CMake ``MUJOCO_USE_FILAMENT`` option in order to
include the Filament renderer.

You can find an example of using the Filament Renderer in the :ref:`render.cc <saRender>` sample and, specifically, in
the ``RenderFilament()`` function.

.. _fiPbr:

Physically-Based Rendering
~~~~~~~~~~~~~~~~~~~~~~~~~~

The :doc:`classic renderer<visualization>` implements a traditional Phong lighting model: surface color is a sum of
ambient, diffuse, specular and emissive terms, and light intensities are dimensionless values tuned by eye. Modern
physically-based rendering (PBR) engines describe surfaces with empirically measurable properties and lights with
photometric units. Because material and lighting parameters are physical, assets retain their appearance across scenes
and lighting conditions, and effects like `image-based lighting
<https://google.github.io/filament/Filament.html#lighting/imagebasedlights>`__ and realistic metals become possible. For
an introduction to the underlying theory, see Filament's `PBR documentation
<https://google.github.io/filament/Filament.html>`__.

The Filament renderer supports both models. In general, we recommend that scenes are authored with modern PBR
(e.g. metallic-roughness) methods to produce the best results. However, we assume that many existing models are authored
for the classic renderer and so we default to using Filament's PBR-based approximation of Phong shading. This fallback
mode has been tuned to produce reasonable results, but the final image can vary significantly compared to the classic
renderer.

.. _fiAuthoring:

Authoring Scenes
~~~~~~~~~~~~~~~~

Materials
^^^^^^^^^

A material is shaded with the PBR lighting model if it defines any physical property: a non-negative
:ref:`metallic<asset-material-metallic>` or :ref:`roughness<asset-material-roughness>` coefficient, or texture
:ref:`layers<material-layer>` with PBR roles (metallic, roughness, packed occlusion-roughness-metallic, normal,
emissive, occlusion). Materials with no physical properties are assumed to be designed with the classic renderer in mind
and are shaded with a Phong lighting model.

Lighting
^^^^^^^^

Physical lighting is enabled by the lights themselves: if any light in the model has positive
:ref:`intensity<body-light-intensity>`, all lights are assumed to be photometric, with intensities measured in
`candela <https://en.wikipedia.org/wiki/Candela>`__. Lights of :ref:`type<body-light-type>` ``image`` provide
image-based lighting: the referenced :ref:`texture<body-light-texture>` illuminates the scene as an environment map. A
light's :ref:`bulbradius<body-light-bulbradius>` controls the softness of its shadows.

If no light has positive intensity, the model is treated as authored for the classic renderer: a default environment
light is added and default intensities are distributed among the model's lights. These defaults are set by the
``filament.fallback`` :ref:`custom flags<fiFlags>`.

.. _fiFlags:

Custom Flags
^^^^^^^^^^^^

The Filament renderer provides a set of options for controlling how the final image is rendered. While some of these
options can be set directly on the scene elements as described above (e.g. setting the PBR properties of the material or
the intensities of the lights), some features are controlled by :ref:`custom<custom>` model elements (e.g.
:ref:`numeric<custom-numeric>` or :ref:`text<custom-text>`) whose names begin with ``filament.`` As the feature set
stabilizes, important flags will be promoted to first-class MJCF attributes.

.. code-block:: xml

   <custom>
     <numeric name="filament.clear_color" data="0.5 0.6 0.7 1"/>
   </custom>

Most flags correspond directly to Filament's `rendering options
<https://github.com/google/filament/blob/main/filament/include/filament/Options.h>`__. Boolean flags are numerics taking
values 0 or 1. Enum flags (e.g. ``filament.ao.quality``) take string values that correspond to the underlying enum's
name identifier. Flags not specified in the model use the defaults listed below.

Screen-space ambient occlusion:
 .. list-table::
    :widths: 32 10 12 46
    :header-rows: 1

    * - Flag
      - Type
      - Default
      - Description
    * - ``filament.ao.enabled``
      - bool
      - 1 (true)
      - Enable screen-space ambient occlusion.
    * - ``filament.ao.quality``
      - string
      - "ultra"
      - Sampling quality; "low", "medium", "high", or "ultra".
    * - ``filament.ao.low_pass_filter``
      - string
      - "ultra"
      - Quality of the depth-aware blur filter; "low", "medium", "high", or "ultra".
    * - ``filament.ao.upsampling``
      - string
      - "ultra"
      - Quality of the occlusion buffer upsampling; "low", "medium", "high", or "ultra".
    * - ``filament.ao.bent_normals``
      - bool
      - 0 (false)
      - Compute bent normals for specular occlusion.
    * - ``filament.ao.ssct``
      - bool
      - 0 (false)
      - Enable screen-space cone tracing.
    * - ``filament.ao.bilateral_threshold``
      - real
      - [filament default]
      - Depth difference treated as an edge by the blur filter.

Bloom:
 .. list-table::
    :widths: 32 10 12 46
    :header-rows: 1

    * - Flag
      - Type
      - Default
      - Description
    * - ``filament.bloom.enabled``
      - bool
      - 0 (false)
      - Enable bloom.
    * - ``filament.bloom.strength``
      - real
      - [filament default]
      - Strength of the bloom effect, between 0 and 1.
    * - ``filament.bloom.dirt_strength``
      - real
      - [filament default]
      - Strength of the lens-dirt effect.
    * - ``filament.bloom.quality``
      - string
      - "low"
      - Quality of the bloom passes; "low", "medium", "high", or "ultra".
    * - ``filament.bloom.resolution``
      - int
      - [filament default]
      - Resolution of the bloom's minor axis, in pixels.
    * - ``filament.bloom.levels``
      - int
      - [filament default]
      - Number of blur levels.

`Color grading <https://github.com/google/filament/blob/main/filament/include/filament/ColorGrading.h>`__:
 .. list-table::
    :widths: 32 10 12 46
    :header-rows: 1

    * - Flag
      - Type
      - Default
      - Description
    * - ``filament.cg.exposure``
      - real
      - 0
      - Exposure adjustment, in stops.
    * - ``filament.cg.temperature``
      - real
      - 0
      - White balance temperature, between -1 (cool) and 1 (warm).
    * - ``filament.cg.tint``
      - real
      - 0
      - White balance tint, between -1 (green) and 1 (magenta).
    * - ``filament.cg.slope``
      - real(3)
      - 1 1 1
      - `ASC CDL <https://en.wikipedia.org/wiki/ASC_CDL>`__ slope.
    * - ``filament.cg.offset``
      - real(3)
      - 0 0 0
      - `ASC CDL <https://en.wikipedia.org/wiki/ASC_CDL>`__ offset.
    * - ``filament.cg.power``
      - real(3)
      - 1 1 1
      - `ASC CDL <https://en.wikipedia.org/wiki/ASC_CDL>`__ power.
    * - ``filament.cg.shadows``
      - real(4)
      - 1 1 1 1
      - Shadow color shift; the fourth component shifts luminance.
    * - ``filament.cg.midtones``
      - real(4)
      - 1 1 1 1
      - Midtone color shift; the fourth component shifts luminance.
    * - ``filament.cg.highlights``
      - real(4)
      - 1 1 1 1
      - Highlight color shift; the fourth component shifts luminance.
    * - ``filament.cg.tonal_ranges``
      - real(4)
      - 0 0.333 0.55 1
      - Boundaries of the shadow and highlight tonal ranges.
    * - ``filament.cg.shadow_gamma``
      - real(3)
      - 1 1 1
      - Gamma adjustment of shadows.
    * - ``filament.cg.mid_point``
      - real(3)
      - 1 1 1
      - Boundary between shadows and highlights.
    * - ``filament.cg.highlight_scale``
      - real(3)
      - 1 1 1
      - Scale of highlights.
    * - ``filament.cg.contrast``
      - real
      - 1
      - Contrast, between 0 and 2.
    * - ``filament.cg.vibrance``
      - real
      - 1
      - Vibrance, between 0 and 2.
    * - ``filament.cg.saturation``
      - real
      - 1
      - Saturation, between 0 and 2.
    * - ``filament.cg.tone_mapping``
      - string
      - "pbr_neutral"
      - Tone mapping operator; "aces", "aces_legacy", "filmic", "linear", or "pbr_neutral".
    * - ``filament.cg.luminance_scaling``
      - bool
      - 0
      - Scale luminance to desaturate very bright highlights.
    * - ``filament.cg.gamut_mapping``
      - bool
      - 0
      - Map out-of-gamut colors into the output gamut.

Exponential height fog:
 .. list-table::
    :widths: 32 10 12 46
    :header-rows: 1

    * - Flag
      - Type
      - Default
      - Description
    * - ``filament.fog.enabled``
      - bool
      - 0 (false)
      - Enable fog.
    * - ``filament.fog.color``
      - real(3)
      - [filament default]
      - Fog color.
    * - ``filament.fog.distance``
      - real
      - [filament default]
      - Distance from the camera at which the fog starts.
    * - ``filament.fog.density``
      - real
      - [filament default]
      - Fog density.
    * - ``filament.fog.cut_off_distance``
      - real
      - [filament default]
      - Distance beyond which fog is not applied.
    * - ``filament.fog.maximum_opacity``
      - real
      - [filament default]
      - Maximum opacity of the fog, between 0 and 1.
    * - ``filament.fog.height``
      - real
      - [filament default]
      - Height above which the fog density decreases.
    * - ``filament.fog.height_falloff``
      - real
      - [filament default]
      - Falloff of fog density with height.
    * - ``filament.fog.in_scattering_start``
      - real
      - [filament default]
      - Distance at which light in-scattering starts.
    * - ``filament.fog.in_scattering_size``
      - real
      - [filament default]
      - Size of the light in-scattering halo (>0 to activate).

Vignette:
 .. list-table::
    :widths: 32 10 12 46
    :header-rows: 1

    * - Flag
      - Type
      - Default
      - Description
    * - ``filament.vignette.enabled``
      - bool
      - 0 (false)
      - Enable vignette.
    * - ``filament.vignette.midpoint``
      - real
      - [filament default]
      - How close to the corners the vignette is restricted, between 0 and 1.

Shadows and anti-aliasing:
 .. list-table::
    :widths: 32 10 12 46
    :header-rows: 1

    * - Flag
      - Type
      - Default
      - Description
    * - ``filament.shadows.type``
      - string
      - "pcf"
      - `Shadow algorithm <https://github.com/google/filament/blob/main/filament/include/filament/Options.h>`__; "pcf", "vsm", or "pcss".
    * - ``filament.shadows.map_size``
      - int
      - min(:ref:`shadowsize<visual-quality-shadowsize>`, 2048)
      - Shadow map resolution, in texels.
    * - ``filament.msaa.enabled``
      - bool
      - 1 (true)
      - Enable multi-sample anti-aliasing.

Scene, fallback lighting, and legacy materials:
 .. list-table::
    :widths: 32 10 12 46
    :header-rows: 1

    * - Flag
      - Type
      - Default
      - Description
    * - ``filament.clear_color``
      - real(4)
      - 0 0 0 1
      - Background color (see also :ref:`mjrf_setClearColor` as well as the note below).
    * - ``filament.fallback.environment_light_intensity``
      - real
      - 5000
      - Intensity of the fallback environment light.
    * - ``filament.fallback.scene_light_intensity``
      - real
      - 80000
      - Total intensity distributed among the model's lights.
    * - ``filament.fallback.head_light_intensity``
      - real
      - 40000
      - Intensity of the :ref:`headlight<visual-headlight>`.
    * - ``filament.phong.specular_multiplier``
      - real
      - 0.2
      - Multiplier applied to legacy (Phong) material :ref:`specular<asset-material-specular>`.
    * - ``filament.phong.shininess_multiplier``
      - real
      - 0.1
      - Multiplier applied to legacy (Phong) material :ref:`shininess<asset-material-shininess>`.
    * - ``filament.phong.emissive_multiplier``
      - real
      - 0.3
      - Multiplier applied to legacy (Phong) material :ref:`emission<asset-material-emission>`.

.. admonition:: clear_color
  :class: note

  The ``filament.clear_color`` flag often requires custom handling as it is not technically a scene-specific property
  but rather a property of the renderer itself. (You can, for example, render multiple scenes to the same target to
  display them simultaneously.) In cases where we know we are only rendering a single scene to a single buffer, we will
  manually apply this setting to the active target. However, if you are managing your own render targets, you are
  responsible for setting the clear color yourself.

.. _fiProgramming:

Programming
~~~~~~~~~~~

Architecture
^^^^^^^^^^^^

The Filament renderer architecture is separated into three layers.

**The Filament library.**  At the base, we have the externally developed `Filament library
<https://github.com/google/filament>`__ itself.

**The mjrf API.** The public interface to the renderer is the ``mjrf`` C API, declared in `mjrfilament.h
<https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjrfilament.h>`__. Unlike the classic ``mjr`` API,
which redraws an :ref:`mjvScene` every frame, ``mjrf`` is *retained-mode* and *asynchronous*: applications create
long-lived objects -- contexts, scenes, meshes, textures, lights, renderables and render targets -- mutate them
incrementally, and submit batches of render and read-pixels requests. Requests are processed on a dedicated render
thread; :ref:`mjrf_render` returns a frame handle which can be waited on with :ref:`mjrf_waitForFrame`. The API itself
is not thread-safe: all calls are expected from a single thread. See the ``mjrf``
:ref:`type<tyFilamentRenderStructure>` and :ref:`function<FilamentRenderingApi>` reference.

**The support layer.** The support library in `src/render/filament/support
<https://github.com/google-deepmind/mujoco/blob/main/src/render/filament/support>`__ (along with
:ref:`mjrf_configureSceneFromModel`) builds and configures ``mjrf`` objects directly from :ref:`mjModel` -- meshes,
textures, materials, lights and the skybox -- and updates their poses and colors each frame from :ref:`mjData`. In this
path no :ref:`mjvScene` is involved: geometry is uploaded once and updated in place.

We also provide a few simple C++ functions in `support/mjrf_ptr.h
<https://github.com/google-deepmind/mujoco/blob/main/src/render/filament/support/mjrf_ptr.h>`__
that wrap some of the ``mjrf`` objects into ``std::unique_ptr`` types to simplify memory management.
