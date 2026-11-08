# Elastic mechanisms

Mechanisms inspired by [Miles Macklin's Reduced Elastic Links experiments](https://reports.mmacklin.com/newton-reduced/reduced_elastic_links_implementation.html), implemented with MuJoCo's multicell quadratic flexes and 1D flex rods. The geometry and MJCF are original; no Newton source or external mesh assets are required.

<p float="left">
  <a href="https://live.mujoco.org/?model=github:google-deepmind/mujoco/main/model/flex/mechanisms/cantilever.xml" title="Open in live.mujoco.org"><img src="https://www.gstatic.com/mujoco/model/flex/mechanisms/cantilever.png" width="32%"></a>
  <a href="https://live.mujoco.org/?model=github:google-deepmind/mujoco/main/model/flex/mechanisms/slidercrank.xml" title="Open in live.mujoco.org"><img src="https://www.gstatic.com/mujoco/model/flex/mechanisms/slidercrank.png" width="32%"></a>
  <a href="https://live.mujoco.org/?model=github:google-deepmind/mujoco/main/model/flex/mechanisms/dipper.xml" title="Open in live.mujoco.org"><img src="https://www.gstatic.com/mujoco/model/flex/mechanisms/dipper.png" width="32%"></a>
</p>

Click an image to open the model in the browser viewer at [live.mujoco.org](https://live.mujoco.org).

| Model | Mechanism | Flex discretization | Total DOFs |
| --- | --- | --- | --- |
| [`cantilever.xml`](cantilever.xml) | Three 3D beams with different stiffness or damping and a 1D rod, with directly attached tip masses | 2 × 1 × 1 quadratic per 3D beam | 414 |
| [`slidercrank.xml`](slidercrank.xml) | A crank wheel drives a sliding piston through a flexible connecting rod | 1D rod, 21 vertices | 67 |
| [`dipper.xml`](dipper.xml) | A cylinder-driven flexible arm with a freely swinging tendon-suspended payload | 1D rod, 21 vertices | 86 |

The slider-crank motor uses an affine force bias: torque = 50 × (2π/3 − angular velocity). The dipper uses the same velocity feedback with a 2π rad/s target. A visible 7 cm eccentric and connecting rod convert its rotation into piston motion, giving approximately a 1 Hz stroke. Actual speed varies slightly with load. Each `crank` control adds an offset to the target angular velocity in rad/s; setting it to the negative nominal target requests zero speed. The cantilevers move under gravity.

## Cantilever comparison

All four beams have the same 0.9 m length, 0.07 × 0.055 m cross-section, 0.12 kg beam mass and 0.2 kg tip mass. Each tip mass is centered on the end cross-section, without a hanging link or eccentric load. Their material parameters are:

| Color / beam | Young's modulus | Stiffness-proportional Rayleigh damping coefficient |
| --- | --- | --- |
| Teal / soft | 8 MPa | 0.002 s |
| Orange / stiff | 16 MPa | 0.002 s |
| Violet / damped | 8 MPa | 0.04 s |
| Yellow / rod | 8 MPa | 0.002 s |

The two soft beams have the same static equilibrium, but different transient decay. The stiff beam bends less and oscillates faster. Node joint damping is zero so it does not obscure the specified material damping.

The yellow beam is a 1D flex with discrete elastic rod bending and a `box` elasticity section of the same cross-section, so it bends with the stiffness of the weak (vertical) axis. Its 24 vertices are kept inextensible by edge equalities. Its root edge straddles the clamp face and is pinned, and two point equalities on its last edge attach the tip mass. It settles at a tip deflection of 0.0746 m, within 1% of the 0.0752 m Euler–Bernoulli prediction (tip mass and self-weight), at a fraction of the cost of the 3D beams; most of the remaining gap is the geometric nonlinearity of the 8% deflection, which linear beam theory ignores. The edge equalities set their own `solref` and `solimp`, since flex edge constraints do not use the `equality` default: with the default edge solref of 0.02 s, the light rod vertices let the edges stretch by about 2% under load, and the rod sags 4% more than Euler–Bernoulli.

## Attachments and suspension

In the 3D cantilevers, the fine tetrahedral grid supplies the visible surface and `cellcount` controls the coarser deformation grid. They pin the nine nodes of their root cross-sections. Quadratic cells do not shear-lock in bending, so two cells per beam come within about 10% of Euler–Bernoulli tip deflection.

The connecting rod and the dipper arm are 1D rods with discrete elastic rod bending, the same mass and `box` cross-section as the trilinear 3D flexes they replaced, a flex radius of half the section height so that they look as thick, and edge equalities that keep them inextensible. The rods follow Euler–Bernoulli bending, while one-cell-thick trilinear cells shear-lock and were much stiffer than their Young's modulus implies, so the rods use 1.8× (connecting rod) and 2.25× (dipper arm) that modulus to keep the deflections of the original mechanisms. Their Rayleigh damping coefficients are divided by the same factors, which keeps the physical damping (coefficient times stiffness) unchanged. They bend in the mechanism plane, about the weak axis of their cross-sections, which is the stiffness the isotropic rod model uses. Their vertices are world-frame slide bodies, which support the discrete integrator's sparse Newton solve.

Point equalities clamp each attachment to a rigid fitting or tip mass: the four corner nodes of a 3D end cross-section, or two vertices of a rod (the last two at a rod end, and the two neighbors of the fulcrum vertex in the dipper arm). The fitting can carry an ordinary hinge, as at the connecting rod ends and dipper fulcrum. In the dipper, a base hinge, crank-driven prismatic rod and tip point connection form the drive cylinder. The flex DOFs are passive. The slider joint has 30 N·s/m damping to resist piston motion and load the connecting rod.

The dipper's 0.5 kg payload is a separate free body. A 0.42 m spatial tendon with only an upper length limit connects its top to the arm's tip fitting: it transmits tension and permits slack, without a rigid suspension link or payload pose controller. At 1 Hz the suspension goes slack and the payload tumbles through large swings. Disabling that tendon causes the payload to fall freely.

## Changelog

* 05-10-2026: Initial release.
