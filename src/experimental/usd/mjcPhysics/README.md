# mjcPhysics -- MuJoCo Physics Schemas for OpenUSD

This directory contains the USD schema definition and accompanying code for
`mjcPhysics`. These schemas allow for detailed specification of a MuJoCo
simulation environment directly within USD. The aim is not to replace
UsdPhysics, but to extend existing concepts and create new types only where is
necessary.

The schemas can be used codeless, or can be built with its C++ bindings. We've
pre-generated the code here via usdGenSchema for internal MuJoCo usage, but it
should also work outside of MuJoCo in apps that support USD natively such as
Omniverse, Houdini, and Maya.

For more detailed information please refer to
[our OpenUSD documentation](https://mujoco.readthedocs.io/en/latest/OpenUSD/index.html)
