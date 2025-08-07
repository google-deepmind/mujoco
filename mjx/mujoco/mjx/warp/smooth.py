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

"""DO NOT EDIT. This file is auto-generated."""
import dataclasses
import jax
from mujoco.mjx._src import types
from mujoco.mjx.warp import ffi
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
def _kinematics_shim(
    # Model
    nworld: int,
    body_dofadr: wp.array(dtype=int),
    body_ipos: wp.array2d(dtype=wp.vec3),
    body_iquat: wp.array2d(dtype=wp.quat),
    body_jntadr: wp.array(dtype=int),
    body_jntnum: wp.array(dtype=int),
    body_parentid: wp.array(dtype=int),
    body_pos: wp.array2d(dtype=wp.vec3),
    body_quat: wp.array2d(dtype=wp.quat),
    body_tree: tuple[wp.array(dtype=int), ...],
    flex_edge: wp.array(dtype=wp.vec2i),
    flex_vertadr: wp.array(dtype=int),
    flex_vertbodyid: wp.array(dtype=int),
    geom_bodyid: wp.array(dtype=int),
    geom_pos: wp.array2d(dtype=wp.vec3),
    geom_quat: wp.array2d(dtype=wp.quat),
    jnt_axis: wp.array2d(dtype=wp.vec3),
    jnt_pos: wp.array2d(dtype=wp.vec3),
    jnt_qposadr: wp.array(dtype=int),
    jnt_type: wp.array(dtype=int),
    mocap_bodyid: wp.array(dtype=int),
    nflexedge: int,
    nflexvert: int,
    ngeom: int,
    nmocap: int,
    nsite: int,
    qpos0: wp.array2d(dtype=float),
    site_bodyid: wp.array(dtype=int),
    site_pos: wp.array2d(dtype=wp.vec3),
    site_quat: wp.array2d(dtype=wp.quat),
    # Data
    flexedge_length: wp.array2d(dtype=float),
    flexedge_velocity: wp.array2d(dtype=float),
    flexvert_xpos: wp.array2d(dtype=wp.vec3),
    geom_skip: wp.array(dtype=bool),
    geom_xmat: wp.array2d(dtype=wp.mat33),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    mocap_pos: wp.array2d(dtype=wp.vec3),
    mocap_quat: wp.array2d(dtype=wp.quat),
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    site_xmat: wp.array2d(dtype=wp.mat33),
    site_xpos: wp.array2d(dtype=wp.vec3),
    xanchor: wp.array2d(dtype=wp.vec3),
    xaxis: wp.array2d(dtype=wp.vec3),
    ximat: wp.array2d(dtype=wp.mat33),
    xipos: wp.array2d(dtype=wp.vec3),
    xmat: wp.array2d(dtype=wp.mat33),
    xpos: wp.array2d(dtype=wp.vec3),
    xquat: wp.array2d(dtype=wp.quat),
):
  _m.stat = _s
  _m.opt = _o
  _d.efc = _e
  _d.contact = _c
  _m.body_dofadr = body_dofadr
  _m.body_ipos = body_ipos
  _m.body_iquat = body_iquat
  _m.body_jntadr = body_jntadr
  _m.body_jntnum = body_jntnum
  _m.body_parentid = body_parentid
  _m.body_pos = body_pos
  _m.body_quat = body_quat
  _m.body_tree = body_tree
  _m.flex_edge = flex_edge
  _m.flex_vertadr = flex_vertadr
  _m.flex_vertbodyid = flex_vertbodyid
  _m.geom_bodyid = geom_bodyid
  _m.geom_pos = geom_pos
  _m.geom_quat = geom_quat
  _m.jnt_axis = jnt_axis
  _m.jnt_pos = jnt_pos
  _m.jnt_qposadr = jnt_qposadr
  _m.jnt_type = jnt_type
  _m.mocap_bodyid = mocap_bodyid
  _m.nflexedge = nflexedge
  _m.nflexvert = nflexvert
  _m.ngeom = ngeom
  _m.nmocap = nmocap
  _m.nsite = nsite
  _m.qpos0 = qpos0
  _m.site_bodyid = site_bodyid
  _m.site_pos = site_pos
  _m.site_quat = site_quat
  _d.flexedge_length = flexedge_length
  _d.flexedge_velocity = flexedge_velocity
  _d.flexvert_xpos = flexvert_xpos
  _d.geom_skip = geom_skip
  _d.geom_xmat = geom_xmat
  _d.geom_xpos = geom_xpos
  _d.mocap_pos = mocap_pos
  _d.mocap_quat = mocap_quat
  _d.qpos = qpos
  _d.qvel = qvel
  _d.site_xmat = site_xmat
  _d.site_xpos = site_xpos
  _d.xanchor = xanchor
  _d.xaxis = xaxis
  _d.ximat = ximat
  _d.xipos = xipos
  _d.xmat = xmat
  _d.xpos = xpos
  _d.xquat = xquat
  _d.nworld = nworld
  mjwarp.kinematics(_m, _d)


def _kinematics_jax_impl(m: types.Model, d: types.Data):
  output_dims = {
      'flexedge_length': d._impl.flexedge_length.shape,
      'flexedge_velocity': d._impl.flexedge_velocity.shape,
      'flexvert_xpos': d._impl.flexvert_xpos.shape,
      'geom_skip': d._impl.geom_skip.shape,
      'geom_xmat': d.geom_xmat.shape,
      'geom_xpos': d.geom_xpos.shape,
      'mocap_pos': d.mocap_pos.shape,
      'mocap_quat': d.mocap_quat.shape,
      'qpos': d.qpos.shape,
      'qvel': d.qvel.shape,
      'site_xmat': d.site_xmat.shape,
      'site_xpos': d.site_xpos.shape,
      'xanchor': d.xanchor.shape,
      'xaxis': d.xaxis.shape,
      'ximat': d.ximat.shape,
      'xipos': d.xipos.shape,
      'xmat': d.xmat.shape,
      'xpos': d.xpos.shape,
      'xquat': d.xquat.shape,
  }
  jf = ffi.jax_callable_variadic_tuple(
      _kinematics_shim,
      num_outputs=19,
      output_dims=output_dims,
      vmap_method=None,
      in_out_argnames={
          'flexedge_length',
          'flexedge_velocity',
          'flexvert_xpos',
          'geom_skip',
          'geom_xmat',
          'geom_xpos',
          'mocap_pos',
          'mocap_quat',
          'qpos',
          'qvel',
          'site_xmat',
          'site_xpos',
          'xanchor',
          'xaxis',
          'ximat',
          'xipos',
          'xmat',
          'xpos',
          'xquat',
      },
  )
  out = jf(
      d.qpos.shape[0],
      m.body_dofadr,
      m.body_ipos,
      m.body_iquat,
      m.body_jntadr,
      m.body_jntnum,
      m.body_parentid,
      m.body_pos,
      m.body_quat,
      m._impl.body_tree,
      m._impl.flex_edge,
      m._impl.flex_vertadr,
      m._impl.flex_vertbodyid,
      m.geom_bodyid,
      m.geom_pos,
      m.geom_quat,
      m.jnt_axis,
      m.jnt_pos,
      m.jnt_qposadr,
      m.jnt_type,
      m._impl.mocap_bodyid,
      m._impl.nflexedge,
      m._impl.nflexvert,
      m.ngeom,
      m.nmocap,
      m.nsite,
      m.qpos0,
      m.site_bodyid,
      m.site_pos,
      m.site_quat,
      d._impl.flexedge_length,
      d._impl.flexedge_velocity,
      d._impl.flexvert_xpos,
      d._impl.geom_skip,
      d.geom_xmat,
      d.geom_xpos,
      d.mocap_pos,
      d.mocap_quat,
      d.qpos,
      d.qvel,
      d.site_xmat,
      d.site_xpos,
      d.xanchor,
      d.xaxis,
      d.ximat,
      d.xipos,
      d.xmat,
      d.xpos,
      d.xquat,
  )
  d = d.tree_replace({
      '_impl.flexedge_length': out[0],
      '_impl.flexedge_velocity': out[1],
      '_impl.flexvert_xpos': out[2],
      '_impl.geom_skip': out[3],
      'geom_xmat': out[4],
      'geom_xpos': out[5],
      'mocap_pos': out[6],
      'mocap_quat': out[7],
      'qpos': out[8],
      'qvel': out[9],
      'site_xmat': out[10],
      'site_xpos': out[11],
      'xanchor': out[12],
      'xaxis': out[13],
      'ximat': out[14],
      'xipos': out[15],
      'xmat': out[16],
      'xpos': out[17],
      'xquat': out[18],
  })
  return d


@jax.custom_batching.custom_vmap
@ffi.marshal_jax_warp_callable
def kinematics(m: types.Model, d: types.Data):
  return _kinematics_jax_impl(m, d)
@kinematics.def_vmap
@ffi.marshal_custom_vmap
def kinematics_vmap(unused_axis_size, is_batched, m, d):
  d = kinematics(m, d)
  return d, is_batched[1]
