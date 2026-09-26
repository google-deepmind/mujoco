# Experimental native Metal physics implementation

This branch now contains an installable implementation under [`metal/`](../metal/README.md).
The earlier RFC has been replaced by a standalone package that can be used and
improved independently of the upstream review decision.

The package ports Metal kinematics, articulated dynamics, convex-foot/plane
contacts, constraint solving and integration from the public Microduck prototype.
It adds a batched stepping/reset/snapshot API, explicit support checks, bundled
resources, CPU tests and opt-in GPU qualification tests. It leaves the normal
MuJoCo build and Python dependencies unchanged.

The first public profile remains explicitly bounded to the bundled Microduck
model and foot-ground collisions. Arbitrary MJCF, self-collision, newer MuJoCo
versions and double-precision Metal arithmetic are not supported. Dependencies
are pinned to the original MuJoCo 3.10.0/Torch 2.9.1 reference. This is community
experimental code, not official MuJoCo Metal support.

See the [package README](../metal/README.md) for installation, usage, limitations,
validation commands and provenance. New GPU qualification and controlled
performance measurements remain pending; CPU tests are not physics qualification.
