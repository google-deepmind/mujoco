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
import mujoco
from mujoco.mjx._src import types
from mujoco.mjx.warp import ffi
# Re-use the render registry
from mujoco.mjx.warp.render import _MJX_RENDER_CONTEXT_BUFFERS
from mujoco.mjx.warp.types import RenderContext
import mujoco.mjx.third_party.mujoco_warp as mjwarp
import warp as wp

_m = mjwarp.Model(
    **{f.name: None for f in dataclasses.fields(mjwarp.Model) if f.init}
)
_d = mjwarp.Data(
    **{f.name: None for f in dataclasses.fields(mjwarp.Data) if f.init}
)


@ffi.format_args_for_warp
def _refit_bvh_shim(
    # Model
    geom_dataid: wp.array(dtype=int),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_type: wp.array(dtype=int),
    nflex: int,
    nflexelemdata: int,
    nflexvert: int,
    flex_dim: wp.array(dtype=int),
    flex_elem: wp.array(dtype=int),
    flex_elemnum: wp.array(dtype=int),
    flex_vertadr: wp.array(dtype=int),
    # Data
    geom_xmat: wp.array2d(dtype=wp.mat33),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    flexvert_xpos: wp.array2d(dtype=wp.vec3),
    # Registry
    rc_id: int,
    geom_xpos_out: wp.array2d(dtype=wp.vec3),
):
  _m.geom_dataid = geom_dataid
  _m.geom_size = geom_size
  _m.geom_type = geom_type
  _m.nflex = nflex
  _m.nflexelemdata = nflexelemdata
  _m.nflexvert = nflexvert
  _m.flex_dim = flex_dim
  _m.flex_elem = flex_elem
  _m.flex_elemnum = flex_elemnum
  _m.flex_vertadr = flex_vertadr
  _d.geom_xmat = geom_xmat
  _d.geom_xpos = geom_xpos
  _d.flexvert_xpos = flexvert_xpos
  _d.nworld = geom_xpos.shape[0]

  render_context = _MJX_RENDER_CONTEXT_BUFFERS[rc_id]
  mjwarp.refit_bvh(_m, _d, render_context)
  wp.copy(geom_xpos_out, geom_xpos)


def _refit_bvh_jax_impl(m: types.Model, d: types.Data, ctx: RenderContext):
  nworld = d.qpos.shape[0]
  ngeom = d.geom_xpos.shape[1]

  jf = ffi.jax_callable_variadic_tuple(
      _refit_bvh_shim,
      num_outputs=1,
      output_dims={'geom_xpos_out': (nworld, ngeom, 3)},
      vmap_method=None,
  )
  out = jf(
      m.geom_dataid,
      m.geom_size,
      m.geom_type,
      m.nflex,
      m.nflexelemdata,
      m.nflexvert,
      m.flex_dim,
      m.flex_elem,
      m.flex_elemnum,
      m.flex_vertadr,
      d.geom_xmat,
      d.geom_xpos,
      d.flexvert_xpos,
      ctx.key,
  )
  return d.replace(geom_xpos=out[0])


@jax.custom_batching.custom_vmap
@functools.partial(ffi.marshal_jax_warp_callable)
def refit_bvh(m: types.Model, d: types.Data, ctx: RenderContext):
  return _refit_bvh_jax_impl(m, d, ctx)


@refit_bvh.def_vmap
@functools.partial(ffi.marshal_custom_vmap)
def refit_bvh_vmap(unused_axis_size, is_batched, m, d, ctx):
  out = refit_bvh(m, d, ctx)
  return out, is_batched[1]
