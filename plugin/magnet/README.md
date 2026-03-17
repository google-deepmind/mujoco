# Magnetic plugin

## Introduction

The `mujoco.magnet` plugin implements magnetic forces and torques for induced magnets.

Given a magnetic field at the origin $B_0$, and a constant gradient $\Delta B $, the plugin defines de magnetic field at any point in the simulation.

To define the induced magnets, the user must provide the relative permeability $\mu_r$, the volume of the magnet $V$, and optionally the diamagnetization factors $N$.

You can use it like:

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.magnet"/>
  </extension>

  <worldbody>
    <!-- The first instance of the plugin MUST define the magnetic field -->
    <plugin plugin="mujoco.magnet">
      <config key="B0" value="0.01 0.01 0.01"/>
      <config key="dB" value="0.01 0 0"/>
    </plugin>

    <!-- We define the magnets in the scene -->
    <body>
      <geom type="cylinder" size="0.035 0.01" rgba="0 0 1 0.2"/>
      <joint type="free" damping="0.05"/>
      <plugin plugin="mujoco.magnet">
        <config key="mu_r" value="1000"/>
        <config key="V" value="7.7e-5"/>
        <config key="N" value="0.2 0.2 0.6"/>
      </plugin>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j" plugin="mujoco.pid" instance="pid" />
  </actuator>
</mujoco>
```

## Parameters

* [Optional] `B0`: The magnetic field at the origin in the inertial frame. Defaults to zero.
* [Optional] `dB`: The diagonal of the matrix to define the gradient of the magnetic field in the inertial frame. Defaults to zero.
* [Mandatory] `mu_r`: The relative permeability of the magnet.
* [Mandatory] `V`: The volume of the magnet.
* [Optional] `N`: The diagonal of the matrix to define the diamagnetization factors of the magnet in the magnet frame. Defaults to isotropic.

## Formulation

The magnetic field can be defined in the following way:

$$
\begin{equation}
\vec{M} = \chi \vec{H}
\quad
\vec{B} = \mu \vec{H}
\end{equation}
$$

$$
\begin{equation}
\vec{B} = \mu_0 (\vec{H} + \vec{M}) = \mu_0 (1 + \chi) \vec{H}
\end{equation}
$$

Where the magnetization $\vec{M}$ is proportional to the magnetic field strength $\vec{H}$ and the magnetic flux $\vec{B}$. This linearity is given by the susceptibility $\chi$ and the absolute permeability of the material itself $\mu$.

From here we can get the following relationship:

$$
\begin{equation}
\mu = \mu_0 (1 + \chi)
\quad
\mu_r = \frac{\mu}{\mu_0}
\quad
\chi = \mu_r - 1
\end{equation}
$$

Where $\mu_0$ is the absolute permeability of free space.

If the material is anisotropic (meaning that it gets magnetized differently depending on the orientation), then we need to add demagnetizing factors into the formulation. These demagnetizing factors are taken into account when computing the susceptibility of the material. In this case we'll consider this the apparent susceptibility $\chi_{app}$. Unless otherwise specified, the material will be considered isotropic, which means the demagnetizing factors are all equal in every axis.

$$
\begin{equation}
\chi_{app} = \frac{\chi}{1 + \chi N}
\end{equation}
$$

Including $V$ and $\mu_0$ we can obtain the full magnetic polarizability tensor $\alpha$ in the body frame $B$.

$$
\begin{equation}
_{B}\alpha = \frac{V}{\mu_0}   {}_{B}\chi_{app}
\end{equation}
$$

Now we rotate this tensor following tensor rotation math to express it in the inertial frame $I$ so that we can obtain results in the inertial frame

$$
\begin{equation}
_{I}\alpha = {}_{I}R_F {}_{F}\alpha {}_{I}R_F^T
\end{equation}
$$

Finally we compute the magnetic moment of our object with this new factor.

$$
\begin{equation}
{}_I m = {}_{I}\alpha {}_I\vec{B}_F
\end{equation}
$$

Which applied to equations for force and torque for magnets shown ahead, yields the correspondent force and torque in the inertial frame which we can then apply to the body.

$$
\begin{equation}
F = \nabla B \cdot m
\end{equation}
$$

$$
\begin{equation}
\tau = m \times{B}
\end{equation}
$$
