# Sensor Plugins

Custom sensors implemented as [engine
plugins](https://mujoco.readthedocs.io/en/latest/programming/extension.html#engine-plugins).

- [Touch Grid](#touch-grid)
  - [Example model](#example-model)
  - [Illustration of fields-of-view in spherical coordinates](#illustration-of-fields-of-view-in-spherical-coordinates)
  - [Illustration of foveal deformation](#illustration-of-foveal-deformation)
  - [Illustration combining resolution, fields-of-view and foveal deformation](#illustration-combining-resolution-fields-of-view-and-foveal-deformation)

## [Touch Grid](touch_grid.h)

This sensor aggregates contact forces into "taxels": a rectangular array of pixel-like elements.

A `touch_grid` sensor is associated with a site and senses contact forces and
torques between the site's parent body and all other bodies. The site's frame
determines the orientation of the sensor with the same convention used for
cameras and lights: the sensor points in the frame's **negative-z** direction,
so the x and y axes correspond to horizontal and vertical, respectively.

The output of the sensor is a stack of 1 to 6 "touch images" corresponding to forces
and torques in the frame of the sensor. Forces and torques are in the in [z, x,
y] order, corresponding to the ordering in contact frames: [normal, tangent,
tangent] and [torsional, rolling, rolling]. Each "taxel" corresponds to an angular bin
in spherical coordinates, and aggregates all the forces occuring inside this bin, which occur
between the body in which the sensor's site is defined and any other body.

The sensor is parametrized by 6 numbers:

1. Number of channels, in the order given above. _positive integer in [1 6]_
2. Horizontal resolution. _positive integer_
3. Vertical resolution. _positive integer_
4. Horizontal field-of-view. _positive float in (0, 180] degrees_
5. Vertical field-of-view. _positive float in (0, 90] degrees_
6. Foveal deformation. _positive float in [0, 1]_

See illustrations below for a visual explanation of the field-of-view and foveal
deformation parameters. These parameters are passed as plugin config attributes:

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.sensor.touch_grid"/>
  </extension>
  ...
  <sensor>
    <plugin name="touch" plugin="mujoco.sensor.touch_grid" objtype="site" objname="touch">
      <config key="nchannel" value="3"/>
      <config key="size" value="7 7"/>
      <config key="fov" value="45 45"/>
      <config key="gamma" value="0"/>
    </plugin>
  </sensor>
</mujoco>
```

Note the following:

- The dimensionality of the sensor output is `nchannel * size_x *size_y`.
- `objtype="site" objname="touch"` specify that the sensor is associated with a
  site, and the name of the specific site.
- Field-of-view angles are always in degrees, disregarding the `<compiler>`
  "angle" directive.

### Example model

<a href="https://youtu.be/0LOJ3WMnqeA" target="_blank">
 <img src="http://img.youtube.com/vi/0LOJ3WMnqeA/hqdefault.jpg" alt="Watch the video" width="560" height="315"/>
</a>

See [touch_grid.xml](../../model/plugin/sensor/touch_grid.xml) to play with the model above.

### Illustration of fields-of-view in spherical coordinates

<img src="images/30-30.png" style="width: 300px;"/>
<img src="images/180-30.png" style="width: 300px;"/>
<img src="images/180-90.png" style="width: 300px;"/>

### Illustration of foveal deformation

![foveal deformation](images/fovea.png)

### Illustration combining resolution, fields-of-view and foveal deformation

[![touch grid illustration](https://img.youtube.com/vi/YScjmR8LwQI/0.jpg)](https://www.youtube.com/watch?v=YScjmR8LwQI)
