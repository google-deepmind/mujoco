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
