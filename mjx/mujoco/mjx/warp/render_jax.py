# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""JAX-callable wrapper for mujoco_warp batch rendering.

This module provides a factory function that wraps mujoco_warp's GPU-accelerated
ray-tracing renderer so it can be called from JAX, including inside jax.jit and
jax.lax.scan.

The approach follows the canonical pattern from mujoco_warp/contrib/jax_unroll.py:
  - Static state (Model, Data, RenderContext) is captured in a closure.
  - Only the dynamic arrays that change each step flow through JAX via
    function arguments and outputs.

Usage:
  render_fn, info = create_mjx_render_fn(mjm, m, d, cam_res=(64, 64))
  rgb = render_fn(geom_xpos, geom_xmat, cam_xpos, cam_xmat,
                  light_xpos, light_xdir)
  images = unpack_rgb(rgb, height=64, width=64)
"""

from __future__ import annotations

from typing import Callable

import jax.numpy as jnp
import mujoco
import warp as wp
from warp.jax_experimental.ffi import GraphMode
from warp.jax_experimental.ffi import jax_callable

try:
    import mujoco_warp as mjwarp  # Standalone install
    # Work around version mismatch: the standalone mujoco_warp may detect
    # a dev MuJoCo version via metadata but the build may not have all
    # bleeding-edge attributes (e.g. flexedge_J_rownnz). Force the flag
    # to False so put_model falls back to reading these from MjData.
    try:
        import mujoco_warp._src.io as _mjwarp_io
        if _mjwarp_io.BLEEDING_EDGE_MUJOCO:
            _mjm_probe = mujoco.MjModel.from_xml_string('<mujoco/>')
            if not hasattr(_mjm_probe, 'flexedge_J_rownnz'):
                _mjwarp_io.BLEEDING_EDGE_MUJOCO = False
    except (ImportError, AttributeError):
        pass
except ImportError:
    from mujoco.mjx.third_party import mujoco_warp as mjwarp  # In-tree

# Verify that the imported mujoco_warp has the rendering API.
if not hasattr(mjwarp, 'create_render_context'):
    raise ImportError(
        'mujoco_warp does not have rendering support '
        '(create_render_context not found). Install the standalone '
        'mujoco_warp package with rendering support, or add its path '
        'to PYTHONPATH.'
    )


def create_mjx_render_fn(
    mjm: mujoco.MjModel,
    m: mjwarp.Model,
    d: mjwarp.Data,
    cam_res: tuple[int, int] = (32, 32),
    render_depth: bool = False,
    use_textures: bool = False,
    use_shadows: bool = False,
    enabled_geom_groups: list[int] | None = None,
    cam_active: list[bool] | None = None,
    graph_mode: GraphMode = GraphMode.NONE,
) -> tuple[Callable, dict]:
    """Create a JAX-callable rendering function.

    This factory function creates a closure over mujoco_warp's Model, Data, and
    RenderContext objects, exposing only the dynamic kinematic arrays as JAX
    function inputs. The returned function can be used inside jax.jit and
    jax.lax.scan.

    Args:
        mjm: MuJoCo model (host).
        m: mujoco_warp Model on device.
        d: mujoco_warp Data on device (with nworld set).
        cam_res: Render resolution as (width, height).
        render_depth: Whether to also output a depth buffer.
        use_textures: Whether to use textures.
        use_shadows: Whether to use shadows.
        enabled_geom_groups: Geom groups to render (default [0, 1, 2]).
        cam_active: Which cameras to render (None = all).
        graph_mode: CUDA graph capture mode (default NONE for BVH compat).

    Returns:
        Tuple of (render_fn, info_dict) where:
        - render_fn: JAX-callable with signature:
            render_fn(geom_xpos, geom_xmat, cam_xpos, cam_xmat,
                      light_xpos, light_xdir) -> rgb [, depth]
            Inputs are JAX float32 arrays. Output rgb is uint32 packed RGBA.
            When render_depth=True, also returns a float32 depth buffer.
        - info_dict: Dict with keys 'rgb_shape', 'depth_shape',
            'total_pixels', 'nworld', 'cam_res', 'ngeom', 'ncam', 'nlight'.
    """
    nworld = d.nworld

    # Run forward to populate kinematic fields (cam/light positions, etc.)
    mjwarp.forward(m, d)

    # Create render context.
    # Note: render_depth and render_rgb are passed as lists to avoid an
    # upstream bug where bool values cause issues in create_render_context.
    if enabled_geom_groups is None:
        enabled_geom_groups = [0, 1, 2]

    rc = mjwarp.create_render_context(
        mjm,
        m,
        d,
        cam_res=cam_res,
        render_rgb=[True],
        render_depth=[render_depth],
        use_textures=use_textures,
        use_shadows=use_shadows,
        enabled_geom_groups=enabled_geom_groups,
        cam_active=cam_active,
    )
    total_pixels = rc.total_rays

    info = {
        'rgb_shape': (nworld, total_pixels),
        'depth_shape': (nworld, total_pixels) if render_depth else None,
        'total_pixels': total_pixels,
        'nworld': nworld,
        'cam_res': cam_res,
        'ngeom': mjm.ngeom,
        'ncam': mjm.ncam,
        'nlight': mjm.nlight,
    }

    # Build the Warp closure that copies dynamic arrays into the captured Data
    # object, refits BVH, renders, and copies output to JAX-visible buffers.
    if render_depth:
        def warp_render(
            geom_xpos: wp.array2d(dtype=wp.vec3),
            geom_xmat: wp.array2d(dtype=wp.mat33),
            cam_xpos: wp.array2d(dtype=wp.vec3),
            cam_xmat: wp.array2d(dtype=wp.mat33),
            light_xpos: wp.array2d(dtype=wp.vec3),
            light_xdir: wp.array2d(dtype=wp.vec3),
            rgb_out: wp.array2d(dtype=wp.uint32),
            depth_out: wp.array2d(dtype=float),
        ):
            wp.copy(d.geom_xpos, geom_xpos)
            wp.copy(d.geom_xmat, geom_xmat)
            wp.copy(d.cam_xpos, cam_xpos)
            wp.copy(d.cam_xmat, cam_xmat)
            wp.copy(d.light_xpos, light_xpos)
            wp.copy(d.light_xdir, light_xdir)
            mjwarp.refit_bvh(m, d, rc)
            mjwarp.render(m, d, rc)
            wp.copy(rgb_out, rc.rgb_data)
            wp.copy(depth_out, rc.depth_data)

        _raw_fn = jax_callable(
            warp_render,
            num_outputs=2,
            graph_mode=graph_mode,
            output_dims={
                'rgb_out': (nworld, total_pixels),
                'depth_out': (nworld, total_pixels),
            },
        )

        def render_fn(
            geom_xpos, geom_xmat, cam_xpos, cam_xmat, light_xpos, light_xdir
        ):
            """Render and return (rgb, depth) packed arrays."""
            rgb, depth = _raw_fn(
                geom_xpos, geom_xmat, cam_xpos, cam_xmat,
                light_xpos, light_xdir,
            )
            return rgb, depth

    else:
        def warp_render(
            geom_xpos: wp.array2d(dtype=wp.vec3),
            geom_xmat: wp.array2d(dtype=wp.mat33),
            cam_xpos: wp.array2d(dtype=wp.vec3),
            cam_xmat: wp.array2d(dtype=wp.mat33),
            light_xpos: wp.array2d(dtype=wp.vec3),
            light_xdir: wp.array2d(dtype=wp.vec3),
            rgb_out: wp.array2d(dtype=wp.uint32),
        ):
            wp.copy(d.geom_xpos, geom_xpos)
            wp.copy(d.geom_xmat, geom_xmat)
            wp.copy(d.cam_xpos, cam_xpos)
            wp.copy(d.cam_xmat, cam_xmat)
            wp.copy(d.light_xpos, light_xpos)
            wp.copy(d.light_xdir, light_xdir)
            mjwarp.refit_bvh(m, d, rc)
            mjwarp.render(m, d, rc)
            wp.copy(rgb_out, rc.rgb_data)

        _raw_fn = jax_callable(
            warp_render,
            num_outputs=1,
            graph_mode=graph_mode,
            output_dims={'rgb_out': (nworld, total_pixels)},
        )

        def render_fn(
            geom_xpos, geom_xmat, cam_xpos, cam_xmat, light_xpos, light_xdir
        ):
            """Render and return rgb packed array."""
            (rgb,) = _raw_fn(
                geom_xpos, geom_xmat, cam_xpos, cam_xmat,
                light_xpos, light_xdir,
            )
            return rgb

    return render_fn, info


def unpack_rgb(
    rgb_packed: jnp.ndarray, height: int, width: int
) -> jnp.ndarray:
    """Unpack uint32 packed RGBA to float32 RGB array.

    The mujoco_warp renderer packs pixel colors as uint32 values with
    R in bits 16-23, G in bits 8-15, B in bits 0-7.

    Args:
        rgb_packed: (nworld, total_pixels) uint32 packed pixel data.
        height: Image height in pixels.
        width: Image width in pixels.

    Returns:
        (nworld, height, width, 3) float32 array with values in [0, 1].
    """
    r = ((rgb_packed >> 16) & 0xFF).astype(jnp.float32) / 255.0
    g = ((rgb_packed >> 8) & 0xFF).astype(jnp.float32) / 255.0
    b = (rgb_packed & 0xFF).astype(jnp.float32) / 255.0
    nworld = rgb_packed.shape[0]
    rgb = jnp.stack([r, g, b], axis=-1)
    return rgb.reshape(nworld, height, width, 3)


def unpack_grayscale(
    rgb_packed: jnp.ndarray, height: int, width: int
) -> jnp.ndarray:
    """Unpack uint32 packed RGBA to float32 grayscale array.

    Uses the standard luminance formula: 0.299*R + 0.587*G + 0.114*B.

    Args:
        rgb_packed: (nworld, total_pixels) uint32 packed pixel data.
        height: Image height in pixels.
        width: Image width in pixels.

    Returns:
        (nworld, height, width, 1) float32 array with values in [0, 1].
    """
    r = ((rgb_packed >> 16) & 0xFF).astype(jnp.float32)
    g = ((rgb_packed >> 8) & 0xFF).astype(jnp.float32)
    b = (rgb_packed & 0xFF).astype(jnp.float32)
    gray = (0.299 * r + 0.587 * g + 0.114 * b) / 255.0
    nworld = rgb_packed.shape[0]
    return gray.reshape(nworld, height, width, 1)
