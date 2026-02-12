"""Benchmark JAX-callable render vs physics-only throughput.

Measures:
  A) Physics-only throughput (mjx.step in jax.lax.scan)
  B) Physics + JAX render throughput (mjx.step + render_fn in jax.lax.scan)
  C) Render-only throughput (render_fn alone, no physics)
  D) Render overhead = B - A
"""

import os

os.environ["XLA_FLAGS"] = "--xla_gpu_graph_min_graph_size=1"

import functools
import time

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
import warp as wp  # noqa: F401 -- needed for mujoco_warp initialization

# Version mismatch workaround: standalone mujoco_warp may incorrectly set
# BLEEDING_EDGE_MUJOCO if the installed MuJoCo build lacks new attributes.
try:
    import mujoco_warp._src.io as _mjwarp_io

    if _mjwarp_io.BLEEDING_EDGE_MUJOCO:
        _probe = mujoco.MjModel.from_xml_string("<mujoco/>")
        if not hasattr(_probe, "flexedge_J_rownnz"):
            _mjwarp_io.BLEEDING_EDGE_MUJOCO = False
except (ImportError, AttributeError):
    pass

import mujoco_warp as mjwarp
from mujoco import mjx
from mujoco.mjx.warp.render_jax import create_mjx_render_fn

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
NWORLDS = 16
UNROLL = 10
WIDTH, HEIGHT = 32, 32
N_WARMUP = 3
N_TRIALS = 10
NACONMAX = 384  # avoid overflow warnings

# Use the humanoid model -- it has cameras (back, side, egocentric) and lights.
MODEL_PATH = (
    "/home/talmolab/Desktop/SalkResearch/mujoco_warp/"
    "benchmarks/humanoid/humanoid.xml"
)

_COMPILER_OPTIONS = {"xla_gpu_graph_min_graph_size": 1}
jax_jit = functools.partial(jax.jit, compiler_options=_COMPILER_OPTIONS)

# ---------------------------------------------------------------------------
# Model / data setup
# ---------------------------------------------------------------------------
print("Setting up model and data ...")
mjm = mujoco.MjModel.from_xml_path(MODEL_PATH)
mjd = mujoco.MjData(mjm)
mujoco.mj_forward(mjm, mjd)

print(f"  ncam={mjm.ncam}, ngeom={mjm.ngeom}, nlight={mjm.nlight}")
assert mjm.ncam > 0, (
    "Model has no cameras -- rendering benchmark needs >= 1 camera."
)

# MJX model (warp backend)
mx = mjx.put_model(mjm, impl="warp")


# Batched MJX data via vmap over make_data (follows testspeed.py pattern)
@jax.vmap
def _init(key):
    d = mjx.make_data(mjm, impl="warp", naconmax=NACONMAX)
    # Initialize from the first keyframe if available
    if mjm.nkey > 0:
        d = d.replace(qpos=jnp.array(mjm.key_qpos[0]))
    return d


keys = jax.random.split(jax.random.PRNGKey(0), NWORLDS)
print(f"  Creating {NWORLDS} batched environments ...")
dx = jax_jit(_init)(keys)
jax.block_until_ready(dx)
print("  Batched MJX data created.")


# Do one forward pass to populate kinematic fields.
@jax_jit
def _warmup_step(mx, dx):
    return mjx.step(mx, dx)


dx = _warmup_step(mx, dx)
jax.block_until_ready(dx)
print("  Forward step done.")

# Warp-level data for the renderer (separate from the MJX JAX data)
m_warp = mjwarp.put_model(mjm)
d_warp = mjwarp.put_data(mjm, mjd, nworld=NWORLDS)

# Create the JAX-callable render function
render_fn, info = create_mjx_render_fn(
    mjm,
    m_warp,
    d_warp,
    cam_res=(WIDTH, HEIGHT),
)

print(
    f"\nConfig: {NWORLDS} worlds, {UNROLL} unrolled steps, "
    f"{WIDTH}x{HEIGHT} render"
)
print(
    f"Scene:  {info['ngeom']} geoms, {info['ncam']} cameras, "
    f"{info['nlight']} lights"
)
print(f"Total pixels/world: {info['total_pixels']}")

# ---------------------------------------------------------------------------
# Benchmark functions
# ---------------------------------------------------------------------------


@jax_jit
def physics_only(mx, dx):
    """A: Physics-only -- mjx.step in jax.lax.scan."""

    def body(dx, _):
        dx = mjx.step(mx, dx)
        return dx, None

    dx, _ = jax.lax.scan(body, dx, length=UNROLL)
    return dx


@jax_jit
def physics_and_render(mx, dx):
    """B: Physics + render -- mjx.step and render_fn in jax.lax.scan."""

    def body(dx, _):
        dx = mjx.step(mx, dx)
        rgb = render_fn(
            dx.geom_xpos,
            dx.geom_xmat,
            dx.cam_xpos,
            dx.cam_xmat,
            dx._impl.light_xpos,
            dx._impl.light_xdir,
        )
        return dx, rgb

    dx, rgbs = jax.lax.scan(body, dx, length=UNROLL)
    return dx, rgbs


@jax_jit
def render_only(dx):
    """C: Render-only -- just the render call, no physics."""

    def body(_, i):
        rgb = render_fn(
            dx.geom_xpos,
            dx.geom_xmat,
            dx.cam_xpos,
            dx.cam_xmat,
            dx._impl.light_xpos,
            dx._impl.light_xdir,
        )
        return None, rgb

    _, rgbs = jax.lax.scan(body, None, jnp.arange(UNROLL))
    return rgbs


# ---------------------------------------------------------------------------
# Warmup (JIT compile + first runs)
# ---------------------------------------------------------------------------
print("\nWarming up (JIT compilation) ...")
for i in range(N_WARMUP):
    jax.block_until_ready(physics_only(mx, dx))
    jax.block_until_ready(physics_and_render(mx, dx))
    jax.block_until_ready(render_only(dx))
    if i == 0:
        print("  First warmup done (JIT compiled).")
print("Warmup complete.\n")

# ---------------------------------------------------------------------------
# Timed trials
# ---------------------------------------------------------------------------
print(f"Running {N_TRIALS} timed trials ...")
times_a, times_b, times_c = [], [], []

for trial in range(N_TRIALS):
    # A: Physics only
    t0 = time.perf_counter()
    jax.block_until_ready(physics_only(mx, dx))
    times_a.append(time.perf_counter() - t0)

    # B: Physics + Render
    t0 = time.perf_counter()
    jax.block_until_ready(physics_and_render(mx, dx))
    times_b.append(time.perf_counter() - t0)

    # C: Render only
    t0 = time.perf_counter()
    jax.block_until_ready(render_only(dx))
    times_c.append(time.perf_counter() - t0)

# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
mean_a = np.mean(times_a) * 1000
mean_b = np.mean(times_b) * 1000
mean_c = np.mean(times_c) * 1000
std_a = np.std(times_a) * 1000
std_b = np.std(times_b) * 1000
std_c = np.std(times_c) * 1000

print(f"\n{'=' * 65}")
print(
    f" Config: {NWORLDS} worlds | {UNROLL} steps | "
    f"{WIDTH}x{HEIGHT} render | {info['ncam']} cameras"
)
print(f"{'=' * 65}")
print(f" A) Physics only:      {mean_a:7.2f} +/- {std_a:.2f} ms")
print(f" B) Physics + Render:  {mean_b:7.2f} +/- {std_b:.2f} ms")
print(f" C) Render only:       {mean_c:7.2f} +/- {std_c:.2f} ms")
print(f"{'=' * 65}")
overhead = mean_b - mean_a
overhead_pct = (mean_b / mean_a - 1) * 100 if mean_a > 0 else float("inf")
print(f" Render overhead (B-A):  {overhead:.2f} ms  ({overhead_pct:.1f}%)")
print(f" Per-step render (B-A)/{UNROLL}: {overhead / UNROLL:.3f} ms")
print(f" Per-step render-only (C/{UNROLL}):  {mean_c / UNROLL:.3f} ms")
print(f"{'=' * 65}")

# Steps per second
sps_a = NWORLDS * UNROLL / (mean_a / 1000)
sps_b = NWORLDS * UNROLL / (mean_b / 1000)
sps_c = NWORLDS * UNROLL / (mean_c / 1000)  # "frames" per second

print(f" Physics SPS:           {sps_a:>12,.0f}")
print(f" Physics+Render SPS:    {sps_b:>12,.0f}")
print(f" Render-only FPS:       {sps_c:>12,.0f}")
print(f"{'=' * 65}")
