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
   - `flat` [bool]: if true, the stress-equilibrium configuration is that of a straight cable; if false or unspecified, it is the configuration defined in the XML.
   - `vmax` [N/m^2]: If greater than zero, the cable is colored using mechanical stresses; the value represent the maximum stress in the color scale.

### Shell

Implemented in [shell.cc](shell.cc).

The shell plugin discretizes an inextensible 2D continuum. It is intended to simulate the bending of plates where the stretching is negligible compared to other deformation modes.

Parameters:

   - `young` [Pa]: Young's modulus.
   - `poisson` [Pa]: Poisson's ratio; if 0, then the material only opposed shear deformations; if near 0.5, then the material is nearly incompressible (rubber-like).
   - `thickness` [m]: shell thickness, used to scale the bending stiffness.
