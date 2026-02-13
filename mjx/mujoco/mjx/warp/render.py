# Copyright 2026 DeepMind Technologies Limited
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

"""DO NOT EDIT. This file is auto-generated."""

import dataclasses
import functools
import jax
from mujoco.mjx._src import types
from mujoco.mjx.warp import ffi
from mujoco.mjx.warp.io import _MJX_RENDER_CONTEXT_BUFFERS
from mujoco.mjx.warp.types import RenderContext
import mujoco.mjx.third_party.mujoco_warp as mjwarp
from mujoco.mjx.third_party.mujoco_warp._src import types as mjwp_types
import warp as wp


_m = mjwarp.Model(
    **{f.name: None for f in dataclasses.fields(mjwarp.Model) if f.init}
)
_d = mjwarp.Data(
    **{f.name: None for f in dataclasses.fields(mjwarp.Data) if f.init}
)
_o = mjwarp.Option(
    **{f.name: None for f in dataclasses.fields(mjwarp.Option) if f.init}
)
_s = mjwarp.Statistic(
    **{f.name: None for f in dataclasses.fields(mjwarp.Statistic) if f.init}
)
_c = mjwarp.Contact(
    **{f.name: None for f in dataclasses.fields(mjwarp.Contact) if f.init}
)
_e = mjwarp.Constraint(
    **{f.name: None for f in dataclasses.fields(mjwarp.Constraint) if f.init}
)


@ffi.format_args_for_warp
def _render_shim(
    # Model
    nworld: int,
    cam_fovy: wp.array2d(dtype=float),
    cam_intrinsic: wp.array2d(dtype=wp.vec4),
    cam_projection: wp.array(dtype=int),
    cam_sensorsize: wp.array(dtype=wp.vec2),
    geom_dataid: wp.array(dtype=int),
    geom_matid: wp.array2d(dtype=int),
    geom_rgba: wp.array2d(dtype=wp.vec4),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_type: wp.array(dtype=int),
    light_active: wp.array2d(dtype=bool),
    light_castshadow: wp.array2d(dtype=bool),
    light_type: wp.array2d(dtype=int),
    mat_rgba: wp.array2d(dtype=wp.vec4),
    mat_texid: wp.array3d(dtype=int),
    mat_texrepeat: wp.array2d(dtype=wp.vec2),
    mesh_faceadr: wp.array(dtype=int),
    nflex: int,
    nlight: int,
    # Data
    cam_xmat: wp.array2d(dtype=wp.mat33),
    cam_xpos: wp.array2d(dtype=wp.vec3),
    geom_xmat: wp.array2d(dtype=wp.mat33),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    light_xdir: wp.array2d(dtype=wp.vec3),
    light_xpos: wp.array2d(dtype=wp.vec3),
    # Registry
    rc_id: int,
    rgb: wp.array2d(dtype=wp.uint32),
    depth: wp.array2d(dtype=wp.float32),
):
  _m.stat = _s
  _m.opt = _o
  _d.efc = _e
  _d.contact = _c
  _m.cam_fovy = cam_fovy
  _m.cam_intrinsic = cam_intrinsic
  _m.cam_projection = cam_projection
  _m.cam_sensorsize = cam_sensorsize
  _m.geom_dataid = geom_dataid
  _m.geom_matid = geom_matid
  _m.geom_rgba = geom_rgba
  _m.geom_size = geom_size
  _m.geom_type = geom_type
  _m.light_active = light_active
  _m.light_castshadow = light_castshadow
  _m.light_type = light_type
  _m.mat_rgba = mat_rgba
  _m.mat_texid = mat_texid
  _m.mat_texrepeat = mat_texrepeat
  _m.mesh_faceadr = mesh_faceadr
  _m.nflex = nflex
  _m.nlight = nlight
  _d.cam_xmat = cam_xmat
  _d.cam_xpos = cam_xpos
  _d.geom_xmat = geom_xmat
  _d.geom_xpos = geom_xpos
  _d.light_xdir = light_xdir
  _d.light_xpos = light_xpos
  _d.nworld = nworld
  render_context = _MJX_RENDER_CONTEXT_BUFFERS[rc_id]
  render_context.rgb_data = rgb
  render_context.depth_data = depth
  mjwarp.render(_m, _d, render_context)


def _render_jax_impl(m: types.Model, d: types.Data, ctx: RenderContext):
  render_ctx = _MJX_RENDER_CONTEXT_BUFFERS[ctx.key]
  output_dims = {
      'rgb': render_ctx.rgb_data_shape,
      'depth': render_ctx.depth_data_shape,
  }
  jf = ffi.jax_callable_variadic_tuple(
      _render_shim,
      num_outputs=2,
      output_dims=output_dims,
      vmap_method=None,
      in_out_argnames=set([]),
      stage_in_argnames=set([
          'cam_fovy',
          'cam_intrinsic',
          'cam_xmat',
          'cam_xpos',
          'geom_matid',
          'geom_rgba',
          'geom_size',
          'geom_xmat',
          'geom_xpos',
          'light_castshadow',
          'light_type',
          'mat_rgba',
          'mat_texid',
      ]),
      stage_out_argnames=set([]),
      graph_mode=m.opt._impl.graph_mode,
  )
  out = jf(
      d.qpos.shape[0],
      m.cam_fovy,
      m.cam_intrinsic,
      m._impl.cam_projection,
      m.cam_sensorsize,
      m.geom_dataid,
      m.geom_matid,
      m.geom_rgba,
      m.geom_size,
      m.geom_type,
      m._impl.light_active,
      m.light_castshadow,
      m.light_type,
      m.mat_rgba,
      m.mat_texid,
      m._impl.mat_texrepeat,
      m.mesh_faceadr,
      m._impl.nflex,
      m.nlight,
      d.cam_xmat,
      d.cam_xpos,
      d.geom_xmat,
      d.geom_xpos,
      d._impl.light_xdir,
      d._impl.light_xpos,
      ctx.key,
  )
  d = d.tree_replace({})
  return out


@jax.custom_batching.custom_vmap
@functools.partial(ffi.marshal_jax_warp_callable, skip_output_dim_reshape=True)
def render(m: types.Model, d: types.Data, ctx: RenderContext):
  return _render_jax_impl(m, d, ctx)


@render.def_vmap
@functools.partial(ffi.marshal_custom_vmap, skip_output_dim_reshape=True)
def render_vmap(
    unused_axis_size,
    is_batched,
    m: types.Model,
    d: types.Data,
    ctx: RenderContext,
):
  out = render(m, d, ctx)
  return out, [True, True]
