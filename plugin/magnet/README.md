# Actuator plugins

## PID

The `mujoco.pid` actuator plugin implements a configurable [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller):

$$f(t) = K_\text{p} e(t) + K_\text{i} \int_0^t e(\tau) \mathrm{d}\tau + K_\text{d} \frac{\mathrm{d}e(t)}{\mathrm{d}t},$$
where $e(t) = u(t) - \ell(t)$ is the difference between the control $u$ and the actuator length $\ell$.

You can use it like:

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid">
        <config key="kp" value="40.0"/>
        <config key="ki" value="40"/>
        <config key="kd" value="4"/>
        <config key="slewmax" value="3" />
        <config key="imax" value="1"/>
      </instance>
    </plugin>
  </extension>

  <worldbody>
    <body>
      <joint name="j" type="slide" axis="0 0 1" />
      <geom size="0.01" mass="1"/>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j" plugin="mujoco.pid" instance="pid" />
  </actuator>
</mujoco>
```

The available options are:

|Attribute | Default | Meaning |
|----------|---------|---------|
|`kp` | 0 | **P** gain for the controller. |
|`ki` | 0 | **I** gain for the controller.<p/>If nonzero, one activation variable will be added to `mjData.act`, containing the current I term (in units of force). |
|`kd` | 0 | **D** gain for the controller. |
|`imax` | Optional | If specified, the force produced by the I term will be clipped to the range `[-imax, imax]`. |
|`slewmax` | Optional | The maximum rate at which the setpoint for the PID controller can change.<p/>If a bigger change is requested between two timesteps, it will be clipped to the range `[ctrl - slewmax * dt, ctrl + slewmax * dt]`<p/>If specified, one activation variable will be added to `mjData.act` containing the previous value of `ctrl`. |
