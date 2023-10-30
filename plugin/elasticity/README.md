<h1>
  <a href="#"><img alt="MuJoCo" src="../../banner.png" width="100%"/></a>
</h1>

## Elasticity plugins

These are first-party plugins that implement passive forces based on discretized continuum mechanics models. They can be applied to **flexes** and **bodies** (via the **composite** functionality). Sample models can be found in [this folder](../../model/plugin/elasticity/).

### Cable

Implemented in [cable.cc](cable.cc).

The cable plugin discretizes an inextensible 1D continuum. It is intended to simulate the twist and bending of rods where the stretching in negligible compared to the other deformation modes.

Parameters:

   - `twist` [Pa]: twisting stiffness.
   - `bend` [Pa]: bending stiffness.
   - `flat` [bool]: if true, the stress-equilibrium configuration is that of a straight cable; if true, it is the configuration defined in the XML.
   - `vmax` [N/m^2]: If greater than zero, the cable is colored using mechanical stresses; the value represent the maximum stress in the color scale.

### Shell

Implemented in [shell.cc](shell.cc).

The shell plugin discretizes an inextensible 2D continuum. It is intended to simulate the bending of plates where the stretching is negligible compared to other deformation modes.

Parameters:

   - `young` [Pa]: Young's modulus.
   - `poisson` [Pa]: Poisson's ratio; if 0, then the material only opposed shear deformations; if near 0.5, then the material is nearly incompressible (rubber-like).
   - `thickness` [m]: shell thickness, used to scale the bending stiffness.

### Membrane

Implemented in [membrane.cc](membrane.cc).

The membrane plugin discretized an extensible 2D continuum. It is intended to simulate the stretching of membranes subjected to tensile stresses, where the bending is negligible.

Parameters:

   - `young` [Pa]: Young's modulus.
   - `poisson` [Pa]: Poisson's ratio; if 0, then the material only opposed shear deformations; if near 0.5, then the material is nearly incompressible (rubber-like).
   - `thickness` [m]: shell thickness, used to scale the stretching stiffness.

### Solid

Implemented in [solid.cc](solid.cc).

The membrane plugin discretized an extensible 3D continuum. It is Saint Venant–Kirchhoff model intended to simulate the compression or elongation of hyperelastic materials subjected to large displacements (finite rotations) and small strains, since it uses a nonlinear strain-displacement but a linear stress-strain relationship.

Parameters:

   - `young` [Pa]: Young's modulus.
   - `poisson` [Pa]: Poisson's ratio; if 0, then the material only opposed shear deformations; if near 0.5, then the material is nearly incompressible (rubber-like).
   - `damping` [Ns/m^2]: damping coefficient for Rayleigh damping.
