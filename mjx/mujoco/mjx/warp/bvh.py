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
def _refit_bvh_shim(
    # Model
    nworld: int,
    flex_dim: wp.array(dtype=int),
    flex_elem: wp.array(dtype=int),
    flex_elemnum: wp.array(dtype=int),
    flex_vertadr: wp.array(dtype=int),
    geom_dataid: wp.array(dtype=int),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_type: wp.array(dtype=int),
    nflex: int,
    nflexelemdata: int,
    nflexvert: int,
    # Data
    flexvert_xpos: wp.array2d(dtype=wp.vec3),
    geom_xmat: wp.array2d(dtype=wp.mat33),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    # Registry
    rc_id: int,
    # Dummy output
    dummy: wp.array(dtype=int),
):
  _m.stat = _s
  _m.opt = _o
  _d.efc = _e
  _d.contact = _c
  _m.flex_dim = flex_dim
  _m.flex_elem = flex_elem
  _m.flex_elemnum = flex_elemnum
  _m.flex_vertadr = flex_vertadr
  _m.geom_dataid = geom_dataid
  _m.geom_size = geom_size
  _m.geom_type = geom_type
  _m.nflex = nflex
  _m.nflexelemdata = nflexelemdata
  _m.nflexvert = nflexvert
  _d.flexvert_xpos = flexvert_xpos
  _d.geom_xmat = geom_xmat
  _d.geom_xpos = geom_xpos
  _d.nworld = nworld
  render_context = _MJX_RENDER_CONTEXT_BUFFERS[rc_id]
  dummy.zero_()
  mjwarp.refit_bvh(_m, _d, render_context)


def _refit_bvh_jax_impl(m: types.Model, d: types.Data, ctx: RenderContext):
  output_dims = {'dummy': (d.qpos.shape[0],)}
  jf = ffi.jax_callable_variadic_tuple(
      _refit_bvh_shim,
      num_outputs=1,
      output_dims=output_dims,
      vmap_method=None,
      in_out_argnames=set([]),
      stage_in_argnames=set(['geom_size', 'geom_xmat', 'geom_xpos']),
      stage_out_argnames=set([]),
      graph_mode=m.opt._impl.graph_mode,
  )
  out = jf(
      d.qpos.shape[0],
      m._impl.flex_dim,
      m._impl.flex_elem,
      m._impl.flex_elemnum,
      m._impl.flex_vertadr,
      m.geom_dataid,
      m.geom_size,
      m.geom_type,
      m._impl.nflex,
      m._impl.nflexelemdata,
      m._impl.nflexvert,
      d._impl.flexvert_xpos,
      d.geom_xmat,
      d.geom_xpos,
      ctx.key,
  )
  d = d.tree_replace({'time': d.time + out[0]})
  return d


@jax.custom_batching.custom_vmap
@ffi.marshal_jax_warp_callable
def refit_bvh(m: types.Model, d: types.Data, ctx: RenderContext):
  return _refit_bvh_jax_impl(m, d, ctx)


@refit_bvh.def_vmap
@ffi.marshal_custom_vmap
def refit_bvh_vmap(
    unused_axis_size,
    is_batched,
    m: types.Model,
    d: types.Data,
    ctx: RenderContext,
):
  d = refit_bvh(m, d, ctx)
  return d, is_batched[1]
