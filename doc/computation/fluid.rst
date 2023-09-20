Fluid forces
============

Proper simulation of fluid dynamics is beyond the scope of MuJoCo, and would be too slow for the applications we aim to
facilitate. Nevertheless we provide two phenomenological models which are sufficient for simulating behaviors
such as flying and swimming. These models are *stateless*, in the sense that no additional states are assigned to the
surrounding fluid, yet are able to capture the salient features of rigid bodies moving through a fluid medium.

Both models are enabled by setting the :ref:`density<option-density>` and :ref:`viscosity<option-viscosity>` attributes
to positive values. These parameters correspond to the density :math:`\rho` and viscosity :math:`\beta` of the medium.

1. The :ref:`Inertia-based model<flInertia>`, uses only viscosity and density, inferring geometry from body
   equivalent-inertia boxes.
2. The :ref:`Ellipsoid-based model <flEllipsoid>` is more elaborate, using an ellipsoid approximation of geoms.
   In addition to the global viscosity and density of the medium, this model exposes 5 tunable parameters per
   interacting geom.

.. tip::
   As detailed in the :ref:`Numerical Integration<geIntegration>` section, implicit integration significantly improves
   simulation stability in the presence of velocity-dependent forces. Both of the fluid-force models described below
   exhibit this property, so the ``implicit`` or ``implicitfast`` :ref:`intergrators<option-integrator>` are
   recommended when using fluid forces. The required analytic derivatives for both models are fully implemented.

.. _flInertia:

Inertia model
-------------

In this model, the shape of each body, for fluid dynamics purposes, is assumed to be the *equivalent inertia box*,
which can also be visualized. Each forward-facing (relative to the linear velocity) face of the box experiences force
along its normal direction. All faces also experience torque due to the angular velocity; this torque is obtained by
integrating the force resulting from the rotation over the surface area. In this sub-section, let :math:`v` and
:math:`\omega` denote the linear and angular body velocity in the body local frame (aligned with the equivalent
inertia box), and :math:`s` the 3D vector of box sizes. When the contributions from all faces are added, the resulting
force and torque applied to the body by a fluid of density :math:`\rho`, in local body coordinates, have the
:math:`i`-th component

.. math::
   \begin{aligned}
   \text{density force} : \quad &- {1 \over 2} \rho s_j s_k |v_i| v_i \\
   \text{density torque} : \quad &- {1 \over 64} \rho s_i \left(s_j^4 + s_k^4 \right) |\omega_i| \omega_i \\
   \end{aligned}

This model implicitly assumes high Reynolds numbers, with lift-to-drag ratio equal to the tangent of the angle of
attack. One can also specify a non-zero :ref:`wind<option-wind>`, which is a 3D vector subtracted from the body linear
velocity in the fluid dynamics computation.

Each body also experiences a force and a torque proportional to the viscosity :math:`\beta` and opposite to its linear and
angular velocity. Note that viscosity can be used independent of density, to make the simulation more damped. We use the
formulas for a sphere at low Reynolds numbers, with diameter :math:`d` equal to the average of the equivalent inertia
box sizes. The resulting 3D force and torque in local body coordinates are

.. math::
   \begin{aligned}
   \text{viscosity force} : \quad &- 3 \beta \pi d v \\
   \text{viscosity torque} : \quad &- \beta \pi d^3 \omega \\
   \end{aligned}

.. _flEllipsoid:

Ellipsoid model
---------------

.. cssclass:: caption-small
.. figure:: ../images/computation/fruitfly.png
   :figwidth: 50%
   :align: right

   The flight-capable Drosophila Melanogaster model in this figure will be described in a
   forthcoming publication.


In this section we describe and derive a stateless model of the forces exerted onto a moving rigid body by the
surrounding fluid, based on an ellipsoidal approximation of geom shape. This model provides finer-grained control of the
different types of fluid forces than the inertia-based model of the previous section. The motivating use-case for this
model is insect flight, see figure on the right.


Summary
~~~~~~~

The model is activated per-geom by setting the :ref:`fluidshape<body-geom-fluidshape>` attribute to ``ellipsoid``, which
also disables the inertia-based model for the parent body. The
5 numbers in the :ref:`fluidcoef<body-geom-fluidcoef>` attribute correspond to the following semantics


.. list-table::
   :width: 60%
   :align: left
   :widths: 1 5 2 1
   :header-rows: 1

   * - Index
     - Description
     - Symbol
     - Default
   * - 0
     - Blunt drag coefficient
     - :math:`C_{D, \text{blunt}}`
     - 0.5
   * - 1
     - Slender drag coeficient
     - :math:`C_{D, \text{slender}}`
     - 0.25
   * - 2
     - Angular drag coeficient
     - :math:`C_{D, \text{angular}}`
     - 1.5
   * - 3
     - Kutta lift coeficient
     - :math:`C_K`
     - 1.0
   * - 4
     - Magnus lift coeficient
     - :math:`C_M`
     - 1.0

Elements of the model are a generalization of :cite:t:`andersen2005b` to 3 dimensions.
The force :math:`\mathbf{f}_{\text{fluid}\rightarrow \text{solid}}` and torque
:math:`\mathbf{g}_{\text{fluid} \rightarrow \text{solid}}` exerted by the fluid onto the solid are
the sum of of the terms

.. math::
   \begin{align*}
   \mathbf{f}_{\text{fluid} \rightarrow \text{solid}} &= \mathbf{f}_A + \mathbf{f}_D + \mathbf{f}_M + \mathbf{f}_K  \\
   \mathbf{g}_{\text{fluid} \rightarrow \text{solid}} &= \mathbf{g}_A + \mathbf{g}_D
   \end{align*}

Where subscripts :math:`A`, :math:`D`, :math:`M` and :math:`K`, denote Added mass, viscous Drag, Magnus lift and
Kutta lift, respectively. The :math:`D`, :math:`M` and :math:`K` terms are scaled by the respective
:math:`C_D`, :math:`C_M` and :math:`C_K` coefficients above, while the added mass term cannot be scaled.

Notation
~~~~~~~~

We describe the motion of the object in an inviscid, incompressible quiescent fluid of density :math:`\rho`. The
arbitrarily-shaped object is described in the model as the equivalent ellipsoid of semi-axes
:math:`\mathbf{d} = \{d_x, d_y, d_z\}`.
The problem is described in a reference frame aligned with the sides of the ellipsoid and moving with it. The
body has velocity :math:`\mathbf{v} = \{v_x, v_y, v_z\}` and angular velocity
:math:`\boldsymbol{\omega} = \{\omega_x, \omega_y, \omega_z\}`. We will also use

.. math::
   \begin{align*}
       d_\text{max} &= \max(d_x, d_y, d_z) \\
       d_\text{min} &= \min(d_x, d_y, d_z) \\
       d_\text{mid} &= d_x + d_y + d_z - d_\text{max} - d_\text{min}
   \end{align*}

The Reynolds number is the ratio between inertial and viscous forces within a flow and is defined as :math:`Re=u~l/\beta`, where
:math:`\beta` is the kinematic viscosity of the fluid, :math:`u` is the characteristic speed of the flow (or, by change of frame, the
speed of the body), and :math:`l` is a characteristic size of the flow or the body.

We will use :math:`\Gamma` to denote circulation, which is the line integral of the velocity field around a closed curve
:math:`\Gamma = \oint \mathbf{v} \cdot \textrm{d} \mathbf{l}` and, due to Stokes' Theorem,
:math:`\Gamma = \int_S \nabla \times \mathbf{v} \cdot \textrm{d}\mathbf{s}`.
In fluid dynamics notation the symbol :math:`\boldsymbol{\omega}` is often used for the
vorticity, defined as :math:`\nabla \times \mathbf{v}`, rather than the angular velocity. For a rigid-body motion, the
vorticity is twice the angular velocity.

Finally, we use the subscripts :math:`i, j, k` to denote triplets of equations that apply symmetrically to
:math:`x, y, z`. For example :math:`a_i = b_j + b_k` is shorthand for the 3 equations

.. math::
   \begin{align*}
       a_x &= b_y + b_z \\
       a_y &= b_x + b_z \\
       a_z &= b_x + b_y
   \end{align*}

.. _flProjection:

Ellipsoid projection
~~~~~~~~~~~~~~~~~~~~

We provide the following result without proof. For the derivation, contact the development team.

.. admonition:: Lemma
   :class: note

   Given an ellipsoid at the origin with semi-axes :math:`(d_x, d_y, d_z)` aligned
   with the coordinate axes :math:`(x, y, z)`, and a unit vector :math:`\mathbf{u} = (u_x, u_y, u_z)`,
   the area projected by the ellipsoid onto the plane normal to :math:`\mathbf{u}` is

   .. math::
      A^{\mathrm{proj}}_{\mathbf{u}} = \pi \sqrt{\frac{d_y^4 d_z^4 u_x^2 + d_x^4 d_z^4 u_y^2 + d_x^4 d_y^4 u_z^2}{d_y^2 d_z^2 u_x^2 + d_x^2 d_z^2 u_y^2 + d_x^2 d_y^2 u_z^2}}


Added mass
~~~~~~~~~~

For a body moving in a fluid, added mass or virtual mass measures the inertia of the fluid that is moved due to the
body's motion. It can be derived from potential flow theory (i.e. it is present also for inviscid flows).

Following Chapter 5 of :cite:t:`lamb1932`, the forces :math:`\mathbf{f}_{V}` and torques :math:`\mathbf{g}_{V}` exerted
onto a moving body due to generation of motion in the fluid from rest can be written as:

.. math::
   \begin{align*}
       \mathbf{f}_{A} &= - \frac{\textrm{d}}{\textrm{d} t} \nabla_{\mathbf{v}} \mathcal{T} + \nabla_{\mathbf{v}} \mathcal{T} \times \boldsymbol{\omega} \\
       \mathbf{g}_{A} &= - \frac{\textrm{d}}{\textrm{d} t} \nabla_{\boldsymbol{\omega}} \mathcal{T} + \nabla_{\mathbf{v}} \mathcal{T} \times \mathbf{v} + \boldsymbol{\omega} \times \nabla_{\boldsymbol{\omega}} \mathcal{T}
   \end{align*}

where :math:`\mathcal{T}` is the kinetic energy of the fluid alone. These forces are often described as added or
virtual mass because they are due to the inertia of the fluid that is to moved or deflected by the accelerating body. In
fact, for a body with constant linear velocity these forces reduce to zero. We consider the body as having three planes
of symmetry because under this assumption the kinetic energy greatly simplifies and can be written as:

.. math::
   2 \mathcal{T} = m_{A, x} v_x^2 + m_{A, y} v_y^2 + m_{A, z} v_z^2 +
                 I_{A, x} \omega_x^2 + I_ {A, y} \omega_y^2 + I_{A, y} \omega_z^2


For convenience we introduce the added-mass vector :math:`\mathbf{m}_A = \{m_{A, x}, m_{A, y}, m_{A, z}\}` and added-moment of
inertia vector :math:`\mathbf{I}_A = \{I_{A, x}, I_{A, y}, I_{A, z}\}`. Each of these quantities should estimate the inertia
of the moved fluid due the motion of the body in the corresponding direction and can be derived from potential flow
theory for some simple geometries.

For a body with three planes of symmetry, we can write in compact form the forces and torques due to added inertia:

.. math::
   \begin{align*}
       \mathbf{f}_{A} &= - \mathbf{m}_A \circ \dot{\mathbf{v}} + \left(\mathbf{m}_A \circ \mathbf{v} \right) \times \boldsymbol{\omega} \\
       \mathbf{g}_{A} &= - \mathbf{I}_A \circ \dot{\boldsymbol{\omega}} + \left(\mathbf{m}_A \circ \mathbf{v} \right) \times \mathbf{v} + \left(\mathbf{I}_A \circ \boldsymbol{\omega} \right) \times \boldsymbol{\omega}
   \end{align*}

Here :math:`\circ` denotes an element-wise product, :math:`\dot{\mathbf{v}}` is the linear acceleration and
:math:`\dot{\boldsymbol{\omega}}` is the angular acceleration. :math:`\mathbf{m}_A \circ \mathbf{v}` and
:math:`\mathbf{I}_A \circ \boldsymbol{\omega}` are the virtual linear and angular momentum respectively.

For an ellipsoid of semi-axis :math:`\mathbf{d} = \{d_x, d_y, d_z\}` and volume :math:`V = 4 \pi d_x d_y d_z / 3`, the
virtual inertia coefficients were derived by :cite:t:`tuckerman1925`. Let:

.. math::
   \kappa_i = \int_0^\infty \frac{d_i d_j d_k}{\sqrt{(d_i^2 + \lambda)^3 (d_j^2 + \lambda) (d_k^2 + \lambda)}} \textrm{d} \lambda


It should be noted that these coefficients are non-dimensional (i.e. if all semi-axes are multiplied by the same scalar
the coefficients remain the same). The virtual masses of the ellipsoid are:

.. math::
   m_{A, i} = \rho V \frac{\kappa_i}{2 - \kappa_i}

And the virtual moments of inertia are:

.. math::
   I_{A, i} = \frac{\rho V}{5} \frac{(d_j^2 - d_k^2)^2 (\kappa_k-\kappa_j)}{2(d_j^2 - d_k^2) + (d_j^2 + d_k^2) (\kappa_j-\kappa_k)}

Viscous drag
~~~~~~~~~~~~

The drag force acts to oppose the motion of the body relative to the surrounding flow. We found that viscous forces
serve also to reduce the stiffness of the equations of motion extended with the fluid dynamic terms. For this reason, we
opted to err on the conservative side and chose approximations of the viscous terms that may overestimate dissipation.

Despite being ultimately caused by viscous dissipation, for high Reynolds numbers the drag is independent of the
viscosity and scales with the second power of the velocity. It can be written as:

.. math::
   \begin{align*}
   \mathbf{f}_\text{D} = - C_D~\rho~ A_D ~ \|\mathbf{v}\|~ \mathbf{v}\\
   \mathbf{g}_\text{D} = - C_D \rho~ I_D ~ \|\boldsymbol{\omega}\| ~ \boldsymbol{\omega}
   \end{align*}

Where :math:`C_D` is a drag coefficient, and :math:`A_D` is a reference surface area (e.g. a measure of the projected
area on the plane normal to the flow), and :math:`I_D` a reference moment of inertia.

Even for simple shapes, the terms :math:`C_D`, :math:`A_D` and :math:`I_D` need to be tuned to the problem-specific
physics and dynamical scales :cite:p:`duan2015`. For example, the drag coefficient :math:`C_D` generally decreases with
increasing Reynolds numbers, and a single reference area :math:`A_D` may not be sufficient to account for the skin
drag for highly irregular or slender bodies. For example, experimental fits are derived from problems ranging from
falling playing cards :cite:p:`wang2004,andersen2005a,andersen2005b` to particle transport :cite:p:`loth2008,
bagheri2016`.

We derive a formula for :math:`\mathbf{f}_\text{D}` based on two surfaces :math:`A^\text{proj}_\mathbf{v}` and
:math:`A_\text{max}`. The first, :math:`A^\text{proj}_\mathbf{v}`, is the cylindrical projection of the body onto a
plane normal to the velocity :math:`\mathbf{v}`. The second is the maximum projected surface
:math:`A_\text{max} = 4 \pi d_{max} d_{min}`.

.. math::
   \mathbf{f}_\text{D} = - \rho~ \big[  C_{D, \text{blunt}} ~ A^\text{proj}_\mathbf{v} ~ +
   C_{D, \text{slender}}\left(A_\text{max} - A^\text{proj}_\mathbf{v} \right) \big] ~ \|\mathbf{v}\|~ \mathbf{v}

The formula and derivation for :math:`A^\text{proj}_\mathbf{v}` is given in the :ref:`lemma<flProjection>` above.

We propose an analogous model for the angular drag. For each Cartesian axis we consider the moment of inertia of the
maximum swept ellipsoid obtained by the rotation of the body around the axis. The resulting diagonal entries of the
moment of inertia are:

.. math::
   \mathbf{I}_{D,ii} = \frac{8\pi}{15} ~d_i ~\max(d_j, ~d_k)^4 .

Given this reference moment of inertia, the angular drag torque is computed as:

.. math::
   \mathbf{g}_\text{D} = - \rho ~ \boldsymbol{\omega} ~ \Big( \big[ C_{D, \text{angular}} ~ \mathbf{I}_D ~ +
   C_{D, \text{slender}} \left(\mathbf{I}_\text{max} - \mathbf{I}_D \right) \big] \cdot \boldsymbol{\omega} \Big)


Here :math:`\mathbf{I}_\text{max}` is a vector with each entry equal to the maximal component of :math:`\mathbf{I}_D`.

The viscosity :math:`\beta`
For Reynolds numbers around or below :math:`O(10)`, the drag is best approximated as linear in the flow velocity
(e.g. Stokes' law). For example, for a sphere the drag force :cite:p:`stokes1850` and torque :cite:p:`lamb1932` are:

.. math::
   \begin{align*}
   \mathbf{f}_\text{S} &= - 6 \pi r_D \rho ~ \beta \mathbf{v}\\
   \mathbf{g}_\text{S} &= - 8 \pi r_D^3 \rho ~ \beta \boldsymbol{\omega}
   \end{align*}

Here, :math:`r_D` is the radius of the sphere and :math:`\beta` is the kinematic viscosity of the medium (e.g.
:math:`1.48~\times 10^{-5}~m^2/s` for ambient-temperature air and :math:`0.89 \times 10^{-4}~m^2/s` for water). Here,
for simplicity, we estimate the radius of the equivalent sphere as :math:`r_D = (d_x + d_y + d_z)/3`. To make a
quantitative example, Stokes' law become accurate for room-temperature air if
:math:`u\cdot l \lesssim 2 \times 10^{-4}~m^2/s`, where :math:`u` is the speed and :math:`l` a characteristic length of
the body.

Viscous lift
~~~~~~~~~~~~

The Kutta-Joukowski theorem calculates the lift :math:`L` of a two-dimensional body translating in a uniform flow with
speed `u` as :math:`L = \rho u \Gamma`. Here, :math:`\Gamma` is the circulation around the body. In the next
subsections we define two sources of circulation and the resulting lift forces.

Magnus force
^^^^^^^^^^^^

.. cssclass:: caption-small
.. figure:: ../images/computation/magnus.png
   :figwidth: 45%
   :align: right

   Smoke flow visualization of the flow past a rotating cylinder (WikiMedia Commons, CC BY-SA 4.0). Due to viscosity,
   the rotating cylinder deflects the incoming flow upward and receives a downwards force (red arrow).

The Magnus effect describes the motion of a rotating object moving through a fluid. Through viscous effects, a spinning
object induces rotation in the surrounding fluid. This rotation deflects the trajectory of the fluid past the object
(i.e. it causes linear acceleration), and the object receives an equal an opposite reaction. For a cylinder, the Magnus
force per unit length of the cylinder can be computed as :math:`F_\text{M} / L = \rho v \Gamma`, where :math:`\Gamma`
is the circulation of the flow caused by the rotation and :math:`v` the velocity of the object. We estimate this force
for an arbitrary body as:

.. math::
   \mathbf{f}_{\text{M}} = C_M ~\rho~ V~ \boldsymbol{\omega}\times\mathbf{v} ,

where :math:`V` is the volume of the body and :math:`C_M` is a coefficient for the force, typically set to 1.

It's worth making an example. To reduce the number of variables, suppose a body rotating in only one direction, e.g.
:math:`\boldsymbol{\omega} = \{0, 0, \omega_z\}`, translating along the other two, e.g. :math:`\mathbf{v} = \{v_x, v_y, 0\}`. The
sum of the force due to added mass and the force due to the Magnus effect along, for example, :math:`x` is:

.. math::
   \frac{f}{\pi \rho d_z} = v_y \omega_z \left(2 d_x \min\{d_x, d_z\} - (d_x + d_z)^2\right)

Note that the two terms have opposite signs.

Kutta condition
^^^^^^^^^^^^^^^

A stagnation point is a location in the flow field where the velocity is zero. For a body moving in a flow (in 2D, in
the frame moving with the body) there are two stagnation points: in the front, where the stream-lines separate to either
sides of the body, and in the rear, where they reconnect. A moving body with a sharp trailing (rear) edge will generate
in the surrounding flow a circulation of sufficient strength to hold the rear stagnation point at the trailing edge.
This is the Kutta condition, a fluid dynamic phenomenon that can be observed for solid bodies with sharp corners, such
as slender bodies or the trailing edges of airfoils.

.. cssclass:: caption-small
.. figure:: ../images/computation/kutta_cond_plate.svg
   :figwidth: 95%
   :align: left

   Sketch of the Kutta condition. Blue lines are streamlines and the two magenta points are the stagnation points. The
   dividing streamline, which connects the two stagnation points, is marked in green. The dividing streamline and the
   body inscribe an area where the flow is said to be "separated" and recirculates within. This circulation produces an
   upward force acting on the plate.

For a two-dimensional flow sketched in the figure above, the circulation due to the Kutta condition can be estimated as:
:math:`\Gamma_\text{K} = C_K ~ d_x ~ \| \mathbf{v}\| ~ \sin(2\alpha)`,
where :math:`C_K` is a lift coefficient, and :math:`\alpha` is the angle between the velocity vector and its projection
onto the surface. The lift force per unit length can be computed with the Kutta–Joukowski theorem as
:math:`\mathbf{f}_K / L = \rho \Gamma_\text{K} \times \mathbf{v}`.

In order to extend the lift force equation to three-dimensional motions, we consider the normal
:math:`\mathbf{n}_{s, \mathbf{v}} = \{\frac{d_y d_z}{d_x}v_x, \frac{d_z d_x}{d_y}v_y, \frac{d_x d_x}{d_z}v_z\}`
to the cross-section of the body which generates the body's projection :math:`A^\text{proj}_\mathbf{v}` onto a plane
normal to the velocity given in the :ref:`lemma<flProjection>` above and the corresponding unit vector
:math:`\hat{\mathbf{n}}_{s, \mathbf{v}}`.
We use this direction to decompose :math:`\mathbf{v} = \mathbf{v}_\parallel ~+~ \mathbf{v}_\perp` with
:math:`\mathbf{v}_\perp = \left(\mathbf{v} \cdot \hat{\mathbf{n}}_{s, \mathbf{v}}\right) \hat{\mathbf{n}}_{s, \mathbf{v}}`.
We write the lift force as:

.. math::
   \begin{align*}
       \mathbf{f}_\text{K} &= \frac{C_K~\rho~ A^\text{proj}_\mathbf{v}}{\|\mathbf{v}\|}
                                 \left( \mathbf{v} \times \mathbf{v}_\parallel\right)\times \mathbf{v} \\
       &= C_K~\rho~ A^\text{proj}_\mathbf{v} \left(\hat{\mathbf{v}} \cdot \hat{\mathbf{n}}_{s, \mathbf{v}}\right)
                                 \left( \hat{\mathbf{n}}_{s, \mathbf{v}} \times \mathbf{v} \right)\times \mathbf{v}
   \end{align*}

Here, :math:`\hat{\mathbf{v}}` is the unit-normal along :math:`\mathbf{v}`. Note that the direction of :math:`\hat{\mathbf{n}}_{s,
\mathbf{v}}` differs from :math:`\hat{\mathbf{v}}` only on the planes where the semi-axes of the body are unequal. So for
example, for spherical bodies :math:`\hat{\mathbf{n}}_{s, \mathbf{v}} \equiv \hat{\mathbf{v}}` and by construction
:math:`\mathbf{f}_\text{K} = 0`.

Let's unpack the relation with an example. Suppose a body with :math:`d_x = d_y` and :math:`d_z \ll d_x`. Note that the vector
:math:`\hat{\mathbf{n}}_{s, \mathbf{v}} \times \hat{\mathbf{v}}` gives the direction of the circulation induced by the
deflection of the flow by the solid body. Along :math:`z`, the circulation will be proportional to :math:`\frac{d_y d_z}{d_x}v_x v_y
- \frac{d_z d_x}{d_y}v_x v_y = 0` (due to :math:`d_x = d_y`). Therefore, on the plane where the solid is blunt, the motion
produces no circulation.

Now, for simplicity, let :math:`v_x = 0`. In this case also the circulation along :math:`y`, proportional
to :math:`\frac{d_y d_z}{d_x}v_x v_z - \frac{d_y d_x}{d_y}v_x v_z`, is zero. The only non-zero component of the circulation
will be along :math:`x` and be proportional to :math:`\left(\frac{d_x d_z}{d_y} - \frac{d_x d_y}{d_z}\right) v_y v_z \approx
\frac{d_x^2}{d_z} v_y v_z`.

We would have :math:`\mathbf{v}_\parallel = \{v_x, 0, v_z\}` and
:math:`\Gamma \propto \{d_z v_y v_z, ~ 0,~ - d_x v_x v_y \} / \|\mathbf{v}\|`.
The motion produces no circulation on the plane where the solid is blunt, and on the other two planes
the circulation is
:math:`\Gamma \propto r_\Gamma ~ \|\mathbf{v}\|~ \sin(2 \alpha) ~ = ~2 r_\Gamma ~\|\mathbf{v}\| ~\sin(\alpha)~\cos(\alpha)`
with :math:`\alpha` the angle between the velocity and its projection on the body on the plane (e.g. on the plane
orthogonal to :math:`x` we have :math:`\sin(\alpha) = v_y/\|\mathbf{v}\|` and
:math:`\cos(\alpha) = v_z/\|\mathbf{v}\|`), and :math:`r_\Gamma`, the lift surface on the plane (e.g. :math:`d_z` for
the plane orthogonal to :math:`x`). Furthermore, the direction of the circulation is given by the cross product (because
the solid boundary "rotates" the incoming flow velocity towards its projection on the body).

Acknowledgements
~~~~~~~~~~~~~~~~

The design and implementation of the model in this section are the work of Guido Novati.

References
~~~~~~~~~~

.. bibliography::
