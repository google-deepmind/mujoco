<h1>
  <a href="#"><img alt="MuJoCo" src="../../banner.png" width="100%"/></a>
</h1>

## Package URI resource provider

A [resource provider](https://mujoco.readthedocs.io/en/latest/programming/extension.html#resource-providers) that
serves ROS-style `package://<package>/<path>` URIs from the OS filesystem, so URDF and MJCF models can reference assets
shipped in ROS packages without rewriting their file names. Implemented in [package_uri.cc](package_uri.cc).

The provider does not depend on ROS. It locates `<package>` the way ROS tools do, by reading two environment variables:

1. `AMENT_PREFIX_PATH` (ROS 2 install spaces): for each prefix, `<prefix>/share/<package>` is used when the ament
   index marker `<prefix>/share/ament_index/resource_index/packages/<package>` exists.
2. `ROS_PACKAGE_PATH` (ROS 1 install spaces and plain source trees): for each directory, `<dir>/<package>` is used
   when it contains a `package.xml`. Unlike `rospack`, nested directories are not searched.

The first match wins, and the resource is `<package directory>/<path>`. A `<path>` that is absolute or contains `..`
is rejected, so a URI cannot escape its package. A sourced ROS workspace sets both variables, but nothing about them
requires ROS: point `ROS_PACKAGE_PATH` at any directory of packages to use the provider outside of a ROS installation.

The library registers the provider when it is loaded: copy it into the `mujoco_plugin` directory next to `simulate`,
or load it from any program with `mj_loadPluginLibrary`:

```xml
<robot name="example">
  <link name="base">
    <visual>
      <geometry>
        <mesh filename="package://example_description/meshes/base.stl"/>
      </geometry>
    </visual>
  </link>
</robot>
```
