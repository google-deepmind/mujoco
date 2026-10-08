# Dry-stone arches

Three unmortared masonry arches, held up by nothing but gravity and friction. Each is a chain of
free-floating voussoirs resting on static abutments: remove the friction and they collapse, so they
are a direct test of how faithfully a solver resolves large frictional contact networks at rest.

<p float="left">
  <a href="https://live.mujoco.org/?model=github:google-deepmind/mujoco/main/model/arch/roman.xml" title="Open in live.mujoco.org"><img src="https://www.gstatic.com/mujoco/model/arch/roman.png" width="32%"></a>
  <a href="https://live.mujoco.org/?model=github:google-deepmind/mujoco/main/model/arch/gothic.xml" title="Open in live.mujoco.org"><img src="https://www.gstatic.com/mujoco/model/arch/gothic.png" width="32%"></a>
  <a href="https://live.mujoco.org/?model=github:google-deepmind/mujoco/main/model/arch/hyperbolic.xml" title="Open in live.mujoco.org"><img src="https://www.gstatic.com/mujoco/model/arch/hyperbolic.png" width="32%"></a>
</p>

Click an image to open the model in the browser viewer at [live.mujoco.org](https://live.mujoco.org).

| Model | Arch | Voussoirs | Timestep |
| --- | --- | --- | --- |
| [`roman.xml`](roman.xml) | semicircular | 27 | 6 ms |
| [`gothic.xml`](gothic.xml) | equilateral pointed | 19 | 8 ms |
| [`hyperbolic.xml`](hyperbolic.xml) | weighted catenary | 29 | 8 ms |

All three run under `integrator="discrete"`, which honors the contact stiffness they are authored
with at these timesteps, and each loads at rest: the voussoir poses are the arch as settled under
load. Left alone they drift by a millimetre or two over five minutes; the Roman arch, whose joints
approach vertical near the springings, also creeps down about a centimetre in that time. The
hyperbolic arch is a funicular: its centerline is the weighted catenary of its own tapered
self-weight, so it carries pure compression and does not creep at all.

The Roman and gothic voussoirs are plain wedges, shrunk so that neighbours sit 0.05 mm apart. The
hyperbolic arch is the delicate one, and two modelling choices matter more for it than any solver
setting; both amount to never handing the collider a tie to break:

* **Joints are small-face-on-large-face.** The voussoirs taper in depth along the arch, so each
  joint has a clear owner rather than two coincident faces of equal size.
* **Lower faces are inset radially.** Without the inset, the intrados and extrados edges meet
  edge-on-edge in the rocking plane and pump a sway mode; with it, ringing energy drops by more
  than an order of magnitude.

Frictional stacking of this kind requires `cone="elliptic"` and a large `impratio`; the arches use
10–100. See the comments in each file for the specific recipe.

## Changelog

* 07-09-2026: Initial release.
