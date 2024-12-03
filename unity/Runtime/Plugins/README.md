## User MuJoCo Plug-ins

As Unity's physics functionality is extended by the MuJoCo library, so too can the base MuJoCo engine be extended with user plug-ins. For more information, see the [documentation ](https://mujoco.readthedocs.io/en/latest/programming/extension.html#explugin). Place plug-in libraries in this folder to load them when importing or running a model with user plugins. 

The multiple types of plug-ins in this contexts necessitates some clarification of the terminology:
- MuJoCo engine library: The `.dll` or `.so` file containing the core physics functionality of MuJoCo.
- Unity plug-in: The interface and bindings to the MuJoCo engine library in Unity.
- MuJoCo Unity package: Additional scripts and components provided with the Unity plug-in to facilitate easier scene creation and running. E.g., Unity Editor components to visualise and configure joints, tools to import and export scenes.
- MuJoCo (user) plug-ins: The libraries extending MuJoCo's features even outside of Unity. E.g., the elasticity plugin that adds shorthands for bendable and deformable structures.

As with the base MuJoCo engine library, the MuJoCo plug-ins must match the version of the Unity plug-in. If you update the version of your Unity plug-in, be sure to update the binaries of the engine and MuJoCo plug-ins too.

User plugin support for the Unity package is currently experimental, consider comparing results with simulations outside of Unity.