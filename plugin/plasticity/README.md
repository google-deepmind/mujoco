# J2 plasticity plugin

This plugin adds history-dependent J2 elastoplasticity to 3D tetrahedral
MuJoCo flexes.

The initial implementation provides perfect J2 plasticity with a von Mises
yield criterion and an associative radial-return update. It reuses MuJoCo's
native flex elasticity rather than implementing a separate elastic material
model.

## Overview

For each tetrahedron, the plugin stores the plastic Green strain

$$
E^p =
\begin{bmatrix}
E^p_{xx} & E^p_{xy} & E^p_{xz} \\
E^p_{xy} & E^p_{yy} & E^p_{yz} \\
E^p_{xz} & E^p_{yz} & E^p_{zz}
\end{bmatrix}.
$$

The six independent components are stored as

```text
[Ep_xx, Ep_yy, Ep_zz, Ep_xy, Ep_xz, Ep_yz]
```

per tetrahedral element.

The current implementation uses Green-Lagrange strain,

$$
E = \frac{1}{2}(F^T F - I),
$$

and an additive elastic-plastic decomposition,

$$
E^e = E - E^p.
$$

The trial deviatoric stress is

$$
s^{\mathrm{trial}} = 2G\,\mathrm{dev}(E^e),
$$

with equivalent von Mises stress

$$
\sigma_{\mathrm{eq}}^{\mathrm{trial}} =
\sqrt{\frac{3}{2}\,s^{\mathrm{trial}} : s^{\mathrm{trial}}}.
$$

The yield function is

$$
f = \sigma_{\mathrm{eq}}^{\mathrm{trial}} - \sigma_y.
$$

If $f \le 0$, the step is elastic and the plastic state is unchanged.

For $f > 0$, perfect J2 plasticity uses the radial-return increment

$$
\Delta\lambda =
\frac{\sigma_{\mathrm{eq}}^{\mathrm{trial}} - \sigma_y}{3G},
$$

with associative plastic flow

$$
\Delta E^p =
\Delta\lambda
\frac{3}{2}
\frac{s^{\mathrm{trial}}}{\sigma_{\mathrm{eq}}^{\mathrm{trial}}}.
$$

The corrected deviatoric stress lies on the yield surface:

$$
s_{n+1} =
\frac{\sigma_y}{\sigma_{\mathrm{eq}}^{\mathrm{trial}}}
s^{\mathrm{trial}}.
$$

## Coupling to native flex elasticity

The plugin does not duplicate MuJoCo's tetrahedral elastic stiffness.

For a tetrahedron, MuJoCo represents deformation using six
squared-edge-length coordinates. For reference edge vectors $A_i$,

$$
q_i = l_i^2 - l_{i0}^2 = 2A_i^T E A_i.
$$

The plastic contribution is represented in the same coordinates:

$$
q_i^p = 2A_i^T E^p A_i.
$$

Therefore the desired elastic coordinate is

$$
q^e = q - q^p.
$$

MuJoCo's native flex elasticity already contributes the force associated with

$$
-G_x^T Kq,
$$

where $K$ is the tetrahedral stiffness matrix stored in `flex_stiffness` and
$G_x$ denotes the mapping from the six edge coordinates to vertex forces.

The plugin adds the plastic correction

$$
+G_x^T Kq^p,
$$

so that the combined response is

$$
-G_x^T K(q-q^p).
$$

This allows the plasticity model to reuse MuJoCo's existing 3D flex elasticity,
including its stiffness matrix and force projection to generalized
coordinates.

The shear modulus $G$ required by the J2 update is reconstructed directly from
the native stiffness matrix using a traceless strain probe. No duplicate
Young's modulus or Poisson ratio configuration is required by the plugin.

## State update

Plastic strain is irreversible state.

During a dynamics evaluation, `Compute()` reads the committed state $E_n^p$,
evaluates the trial state and radial return, and stores a candidate

$$
E_{n+1}^{p,*}.
$$

The returned plastic state is also used for the current force correction, but
`Compute()` does not modify `mjData::plugin_state`.

`Advance()` commits the candidate after the integration step:

$$
E_n^p \rightarrow E_{n+1}^p.
$$

This separates force evaluation from mutation of the committed material
history.

## Usage

The plugin is attached to a 3D elastic `flexcomp`.

For example:

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.plasticity.j2"/>
  </extension>

  <worldbody>
    <flexcomp
        name="solid"
        type="grid"
        dim="3"
        count="3 3 3"
        spacing="0.05 0.05 0.05"
        mass="2.7">

      <elasticity
          young="7e10"
          poisson="0.33"/>

      <plugin plugin="mujoco.plasticity.j2">
        <config
            key="yield"
            value="2.5e8"/>
      </plugin>
    </flexcomp>
  </worldbody>
</mujoco>
```

The only plugin parameter is:

| Parameter | Meaning |
| --- | --- |
| `yield` | von Mises yield stress $\sigma_y$ |

Native flex elasticity must be enabled because the plugin reuses its stiffness
matrix.

## Validation

The implementation is tested at several levels:

- deformation-gradient and Green-strain kinematics;
- rigid-motion invariance;
- trial deviatoric stress and von Mises stress;
- perfect-plastic radial return;
- elastic unloading;
- persistent `plugin_state`;
- plastic edge-force construction;
- native `qfrc_passive` integration;
- loading, state commit, and unloading through the actual plugin callbacks;
- physical single-tetrahedron material regression.

The automated tests are validated in both MuJoCo double-precision and
single-precision configurations, with precision-dependent numerical tolerances.

### Multi-element validation

A homogeneous aluminum cube discretized into 48 tetrahedra was used as an
additional development validation case.

For

$$
E = 70\ \mathrm{GPa}, \qquad
\nu = 0.33, \qquad
\sigma_y = 250\ \mathrm{MPa},
$$

the plugin reconstructed

$$
G = 26.31578947\ \mathrm{GPa},
$$

matching the analytical shear modulus.

Under a homogeneous deviatoric loading-unloading path, all 48 tetrahedra
yielded consistently. The predicted residual strain matched the analytical
perfect-plastic solution, and the force norm at the residual configuration was
approximately $2.0 \times 10^{-7}$ N compared with approximately $2.88$ MN
for native elasticity alone.

The corresponding residual-force ratio was approximately

$$
6.95 \times 10^{-14}.
$$

This benchmark was used as a development validation case; the automated test
suite contains the smaller regression tests shipped with the plugin.

### Interactive validation

In addition to the automated tests, the implementation was manually inspected
using an interactive MuJoCo visualization under multi-tetrahedron loading and
unloading scenarios.

These checks were used to verify that:

- deformation remains spatially coherent across neighboring tetrahedra;
- yielding produces the expected permanent deformation;
- unloading preserves the plastic history;
- the residual configuration is visually consistent with the analytical
  deformation path.

The visualization setup was used as a development validation tool and is not
part of the plugin interface.

## Current scope and limitations

This first implementation intentionally keeps the constitutive model small.

Current assumptions are:

- 3D tetrahedral flexes only;
- native MuJoCo flex elasticity is required;
- von Mises / J2 yield criterion;
- perfect plasticity;
- no hardening;
- additive decomposition $E^e = E - E^p$;
- Green-Lagrange strain;
- six plastic-strain state variables per tetrahedron;
- non-interpolated flexes only.

The additive Green-strain formulation is invariant to rigid-body rotations, but
it is not a general finite-strain plasticity formulation for large irreversible
deformation.

A finite-strain extension would instead typically use a multiplicative
decomposition such as

$$
F = F^e F^p.
$$

Possible future extensions include:

- isotropic hardening;
- kinematic or combined hardening;
- constitutive substepping for large increments;
- explicit validation of additional integration schemes;
- finite-strain plasticity;
- additional yield surfaces;
- viscoplasticity;
- plastic-damage and fracture models;
- per-element plasticity diagnostics for visualization.

## Files

The plugin implementation is contained in:

```text
plugin/plasticity/
├── CMakeLists.txt
├── README.md
├── j2.cc
├── j2.h
└── register.cc
```

Associated tests are under:

```text
test/plugin/plasticity/
```