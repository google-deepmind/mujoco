# Elastic mechanisms

Mechanisms inspired by [Miles Macklin's Reduced Elastic Links experiments](https://reports.mmacklin.com/newton-reduced/reduced_elastic_links_implementation.html), implemented with MuJoCo's multicell trilinear and quadratic flexes. The geometry and MJCF are original; no Newton source or external mesh assets are required.

<p float="left">
  <a href="https://live.mujoco.org/?model=github:google-deepmind/mujoco/main/model/flex/mechanisms/cantilever.xml" title="Open in live.mujoco.org"><img src="https://www.gstatic.com/mujoco/model/flex/mechanisms/cantilever.png" width="32%"></a>
  <a href="https://live.mujoco.org/?model=github:google-deepmind/mujoco/main/model/flex/mechanisms/slidercrank.xml" title="Open in live.mujoco.org"><img src="https://www.gstatic.com/mujoco/model/flex/mechanisms/slidercrank.png" width="32%"></a>
  <a href="https://live.mujoco.org/?model=github:google-deepmind/mujoco/main/model/flex/mechanisms/dipper.xml" title="Open in live.mujoco.org"><img src="https://www.gstatic.com/mujoco/model/flex/mechanisms/dipper.png" width="32%"></a>
</p>

Click an image to open the model in the browser viewer at [live.mujoco.org](https://live.mujoco.org).

| Model | Mechanism | Interpolation cells | Total DOFs |
| --- | --- | --- | --- |
| [`cantilever.xml`](cantilever.xml) | Three beams with different stiffness or damping and directly attached tip masses | 2 × 1 × 1 quadratic per beam | 342 |
| [`slidercrank.xml`](slidercrank.xml) | A crank wheel drives a sliding piston through a flexible connecting rod | 12 × 1 × 1 trilinear | 160 |
| [`dipper.xml`](dipper.xml) | A cylinder-driven flexible arm with a freely swinging tendon-suspended payload | 12 × 1 × 1 trilinear | 179 |

The slider-crank motor uses an affine force bias: torque = 50 × (2π/3 − angular velocity). The dipper uses the same velocity feedback with a 2π rad/s target. A visible 7 cm eccentric and connecting rod convert its rotation into piston motion, giving approximately a 1 Hz stroke. Actual speed varies slightly with load. Each `crank` control adds an offset to the target angular velocity in rad/s; setting it to the negative nominal target requests zero speed. The cantilevers move under gravity.

## Cantilever comparison

All three beams have the same 0.9 m length, 0.07 × 0.055 m cross-section, 0.12 kg beam mass and 0.2 kg tip mass. Each tip mass is centered on the end cross-section, without a hanging link or eccentric load. Their material parameters are:

| Color / beam | Young's modulus | Stiffness-proportional Rayleigh damping coefficient |
| --- | --- | --- |
| Teal / soft | 8 MPa | 0.002 s |
| Orange / stiff | 16 MPa | 0.002 s |
| Violet / damped | 8 MPa | 0.04 s |

The two soft beams have the same static equilibrium, but different transient decay. The stiff beam bends less and oscillates faster. Node joint damping is zero so it does not obscure the specified material damping.

## Attachments and suspension

The fine tetrahedral grid supplies the visible surface; `cellcount` controls the coarser deformation grid. The cantilevers pin the nine nodes of their root cross-sections. Quadratic cells do not shear-lock in bending, so two cells per beam come within about 10% of Euler–Bernoulli tip deflection. The moving mechanisms use world-frame flex nodes, which support the discrete integrator's sparse Newton solve.

Four point equalities clamp each attached cross-section to a rigid fitting. The fitting can carry an ordinary hinge, as at the connecting rod ends and dipper fulcrum. In the dipper, a base hinge, crank-driven prismatic rod and tip point connection form the drive cylinder. The flex DOFs are passive. The slider joint has 30 N·s/m damping to resist piston motion and load the connecting rod.

The dipper's 0.5 kg payload is a separate free body. A 0.42 m spatial tendon with only an upper length limit connects its top to the arm's tip fitting: it transmits tension and permits slack, without a rigid suspension link or payload pose controller. At 1 Hz the suspension goes slack and the payload tumbles through large swings. Disabling that tendon causes the payload to fall freely.

## Changelog

* 05-10-2026: Initial release.
