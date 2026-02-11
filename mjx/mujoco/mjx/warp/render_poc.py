"""Proof of concept: wrap mujoco_warp render as a JAX callable.

This script demonstrates how to use warp.jax_experimental.ffi.jax_callable to
wrap mujoco_warp's GPU-accelerated ray-tracing renderer so it can be called from
JAX, including inside jax.jit and jax.lax.scan.

The approach follows the canonical pattern from mujoco_warp/contrib/jax_unroll.py:
  - Static state (Model, Data, RenderContext) is captured in a closure.
  - Only the dynamic arrays that change each step flow through JAX via
    function arguments and outputs.

Tests:
  1. Direct call to the jax_callable wrapper
  2. Inside jax.jit
  3. Inside jax.lax.scan
"""

import os

os.environ["XLA_FLAGS"] = "--xla_gpu_graph_min_graph_size=1"

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
import warp as wp
from warp.jax_experimental.ffi import GraphMode, jax_callable

import mujoco_warp as mjwarp

# ---------------------------------------------------------------------------
# Scene setup
# ---------------------------------------------------------------------------
NWORLDS = 4
RENDER_WIDTH = 32
RENDER_HEIGHT = 32

XML = """
<mujoco>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
    <body pos="0 0 0.5">
      <geom type="sphere" size="0.2" rgba="1 0 0 1"/>
    </body>
    <camera name="cam0" pos="0 -2 1" xyaxes="1 0 0 0 0.5 1"/>
  </worldbody>
</mujoco>
"""

mjm = mujoco.MjModel.from_xml_string(XML)
mjd = mujoco.MjData(mjm)
mujoco.mj_forward(mjm, mjd)

m = mjwarp.put_model(mjm)
d = mjwarp.put_data(mjm, mjd, nworld=NWORLDS)
mjwarp.forward(m, d)

rc = mjwarp.create_render_context(
    mjm,
    m,
    d,
    cam_res=(RENDER_WIDTH, RENDER_HEIGHT),
    render_rgb=[True],
    render_depth=[False],
    use_textures=False,
    use_shadows=False,
)

total_pixels = rc.total_rays
ngeom = mjm.ngeom
ncam = mjm.ncam
nlight = mjm.nlight

print(f"Scene: ngeom={ngeom}, ncam={ncam}, nlight={nlight}")
print(f"Render: {RENDER_WIDTH}x{RENDER_HEIGHT}, total_pixels={total_pixels}")
print(f"NWORLDS={NWORLDS}")

# ---------------------------------------------------------------------------
# Warp render wrapper (closure pattern from jax_unroll.py)
# ---------------------------------------------------------------------------
# m, d, rc are captured in the closure.
# Only the dynamic arrays that change per step are function arguments/outputs.


def warp_render(
    geom_xpos: wp.array2d(dtype=wp.vec3),
    geom_xmat: wp.array2d(dtype=wp.mat33),
    cam_xpos: wp.array2d(dtype=wp.vec3),
    cam_xmat: wp.array2d(dtype=wp.mat33),
    light_xpos: wp.array2d(dtype=wp.vec3),
    light_xdir: wp.array2d(dtype=wp.vec3),
    rgb_out: wp.array2d(dtype=wp.uint32),
):
    # Copy dynamic inputs into the captured Data object
    wp.copy(d.geom_xpos, geom_xpos)
    wp.copy(d.geom_xmat, geom_xmat)
    wp.copy(d.cam_xpos, cam_xpos)
    wp.copy(d.cam_xmat, cam_xmat)
    wp.copy(d.light_xpos, light_xpos)
    wp.copy(d.light_xdir, light_xdir)

    # Refit BVH acceleration structures for the new geometry positions
    mjwarp.refit_bvh(m, d, rc)

    # Render
    mjwarp.render(m, d, rc)

    # Copy output from render context
    wp.copy(rgb_out, rc.rgb_data)


# Wrap with jax_callable.
# GraphMode.NONE avoids CUDA graph capture issues with BVH operations.
_raw_render_fn = jax_callable(
    warp_render,
    num_outputs=1,
    graph_mode=GraphMode.NONE,
    output_dims={"rgb_out": (NWORLDS, total_pixels)},
)


# jax_callable with num_outputs=1 returns a list of 1 element.
# Unwrap it so callers get a single JAX array back.
def render_fn(geom_xpos, geom_xmat, cam_xpos, cam_xmat, light_xpos, light_xdir):
    (rgb,) = _raw_render_fn(
        geom_xpos, geom_xmat, cam_xpos, cam_xmat, light_xpos, light_xdir
    )
    return rgb


# ---------------------------------------------------------------------------
# Extract current state as JAX arrays
# ---------------------------------------------------------------------------
jax_geom_xpos = jnp.array(d.geom_xpos.numpy())
jax_geom_xmat = jnp.array(d.geom_xmat.numpy())
jax_cam_xpos = jnp.array(d.cam_xpos.numpy())
jax_cam_xmat = jnp.array(d.cam_xmat.numpy())
jax_light_xpos = jnp.array(d.light_xpos.numpy())
jax_light_xdir = jnp.array(d.light_xdir.numpy())

print(f"\nJAX array shapes:")
print(f"  geom_xpos:  {jax_geom_xpos.shape}  dtype={jax_geom_xpos.dtype}")
print(f"  geom_xmat:  {jax_geom_xmat.shape}  dtype={jax_geom_xmat.dtype}")
print(f"  cam_xpos:   {jax_cam_xpos.shape}  dtype={jax_cam_xpos.dtype}")
print(f"  cam_xmat:   {jax_cam_xmat.shape}  dtype={jax_cam_xmat.dtype}")
print(f"  light_xpos: {jax_light_xpos.shape}  dtype={jax_light_xpos.dtype}")
print(f"  light_xdir: {jax_light_xdir.shape}  dtype={jax_light_xdir.dtype}")

# ---------------------------------------------------------------------------
# Test 1: Direct call
# ---------------------------------------------------------------------------
print("\n--- Test 1: Direct jax_callable call ---")
rgb = render_fn(
    jax_geom_xpos,
    jax_geom_xmat,
    jax_cam_xpos,
    jax_cam_xmat,
    jax_light_xpos,
    jax_light_xdir,
)
rgb_np = np.array(rgb)
print(f"RGB shape: {rgb_np.shape}, dtype: {rgb_np.dtype}")
print(f"Non-zero pixels: {np.count_nonzero(rgb_np)} / {rgb_np.size}")
assert rgb_np.shape == (NWORLDS, total_pixels), f"Shape mismatch: {rgb_np.shape}"
assert np.count_nonzero(rgb_np) > 0, "All pixels are zero -- rendering failed"
print("PASSED")

# ---------------------------------------------------------------------------
# Test 2: Inside jax.jit
# ---------------------------------------------------------------------------
print("\n--- Test 2: Inside jax.jit ---")


@jax.jit
def jitted_render(gxp, gxm, cxp, cxm, lxp, lxd):
    return render_fn(gxp, gxm, cxp, cxm, lxp, lxd)


rgb2 = jitted_render(
    jax_geom_xpos,
    jax_geom_xmat,
    jax_cam_xpos,
    jax_cam_xmat,
    jax_light_xpos,
    jax_light_xdir,
)
jax.block_until_ready(rgb2)
rgb2_np = np.array(rgb2)
print(f"JIT render: shape={rgb2.shape}, non-zero={np.count_nonzero(rgb2_np)}")
assert rgb2_np.shape == (NWORLDS, total_pixels), f"Shape mismatch: {rgb2_np.shape}"
assert np.count_nonzero(rgb2_np) > 0, "All pixels are zero -- JIT render failed"
print("PASSED")

# ---------------------------------------------------------------------------
# Test 3: Inside jax.lax.scan
# ---------------------------------------------------------------------------
print("\n--- Test 3: Inside jax.lax.scan ---")

SCAN_LENGTH = 3


@jax.jit
def scan_render(gxp, gxm, cxp, cxm, lxp, lxd):
    def body(carry, _):
        rgb = render_fn(gxp, gxm, cxp, cxm, lxp, lxd)
        return carry, rgb

    _, rgbs = jax.lax.scan(body, None, length=SCAN_LENGTH)
    return rgbs


rgbs = scan_render(
    jax_geom_xpos,
    jax_geom_xmat,
    jax_cam_xpos,
    jax_cam_xmat,
    jax_light_xpos,
    jax_light_xdir,
)
jax.block_until_ready(rgbs)
rgbs_np = np.array(rgbs)
print(f"Scan render: shape={rgbs.shape}")
print(
    f"All frames identical (expected with same state): "
    f"{np.allclose(rgbs_np[0], rgbs_np[1])}"
)
assert rgbs_np.shape == (
    SCAN_LENGTH,
    NWORLDS,
    total_pixels,
), f"Shape mismatch: {rgbs_np.shape}"
assert np.count_nonzero(rgbs_np) > 0, "All pixels are zero -- scan render failed"
print("PASSED")

# ---------------------------------------------------------------------------
print("\n=== All tests passed! ===")
