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
    body_ipos: wp.array2d(dtype=wp.vec3),
    body_iquat: wp.array2d(dtype=wp.quat),
    body_jntadr: wp.array(dtype=int),
    body_jntnum: wp.array(dtype=int),
    body_mocapid: wp.array(dtype=int),
    body_parentid: wp.array(dtype=int),
    body_pos: wp.array2d(dtype=wp.vec3),
    body_quat: wp.array2d(dtype=wp.quat),
    body_rootid: wp.array(dtype=int),
    body_tree: tuple[wp.array(dtype=int), ...],
    body_weldid: wp.array(dtype=int),
    geom_bodyid: wp.array(dtype=int),
    geom_pos: wp.array2d(dtype=wp.vec3),
    geom_quat: wp.array2d(dtype=wp.quat),
    jnt_axis: wp.array2d(dtype=wp.vec3),
    jnt_pos: wp.array2d(dtype=wp.vec3),
    jnt_qposadr: wp.array(dtype=int),
    jnt_type: wp.array(dtype=int),
    ngeom: int,
    nsite: int,
    qpos0: wp.array2d(dtype=float),
    site_bodyid: wp.array(dtype=int),
    site_pos: wp.array2d(dtype=wp.vec3),
    site_quat: wp.array2d(dtype=wp.quat),
    # Data
    geom_xmat: wp.array2d(dtype=wp.mat33),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    mocap_pos: wp.array2d(dtype=wp.vec3),
    mocap_quat: wp.array2d(dtype=wp.quat),
    qpos: wp.array2d(dtype=float),
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
  _m.body_ipos = body_ipos
  _m.body_iquat = body_iquat
  _m.body_jntadr = body_jntadr
  _m.body_jntnum = body_jntnum
  _m.body_mocapid = body_mocapid
  _m.body_parentid = body_parentid
  _m.body_pos = body_pos
  _m.body_quat = body_quat
  _m.body_rootid = body_rootid
  _m.body_tree = body_tree
  _m.body_weldid = body_weldid
  _m.geom_bodyid = geom_bodyid
  _m.geom_pos = geom_pos
  _m.geom_quat = geom_quat
  _m.jnt_axis = jnt_axis
  _m.jnt_pos = jnt_pos
  _m.jnt_qposadr = jnt_qposadr
  _m.jnt_type = jnt_type
  _m.ngeom = ngeom
  _m.nsite = nsite
  _m.qpos0 = qpos0
  _m.site_bodyid = site_bodyid
  _m.site_pos = site_pos
  _m.site_quat = site_quat
  _d.geom_xmat = geom_xmat
  _d.geom_xpos = geom_xpos
  _d.mocap_pos = mocap_pos
  _d.mocap_quat = mocap_quat
  _d.qpos = qpos
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
      'geom_xmat': d.geom_xmat.shape,
      'geom_xpos': d.geom_xpos.shape,
      'mocap_pos': d.mocap_pos.shape,
      'mocap_quat': d.mocap_quat.shape,
      'qpos': d.qpos.shape,
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
      num_outputs=14,
      output_dims=output_dims,
      vmap_method=None,
      in_out_argnames={
          'geom_xmat',
          'geom_xpos',
          'mocap_pos',
          'mocap_quat',
          'qpos',
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
      m.body_ipos,
      m.body_iquat,
      m.body_jntadr,
      m.body_jntnum,
      m.body_mocapid,
      m.body_parentid,
      m.body_pos,
      m.body_quat,
      m.body_rootid,
      m._impl.body_tree,
      m.body_weldid,
      m.geom_bodyid,
      m.geom_pos,
      m.geom_quat,
      m.jnt_axis,
      m.jnt_pos,
      m.jnt_qposadr,
      m.jnt_type,
      m.ngeom,
      m.nsite,
      m.qpos0,
      m.site_bodyid,
      m.site_pos,
      m.site_quat,
      d.geom_xmat,
      d.geom_xpos,
      d.mocap_pos,
      d.mocap_quat,
      d.qpos,
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
      'geom_xmat': out[0],
      'geom_xpos': out[1],
      'mocap_pos': out[2],
      'mocap_quat': out[3],
      'qpos': out[4],
      'site_xmat': out[5],
      'site_xpos': out[6],
      'xanchor': out[7],
      'xaxis': out[8],
      'ximat': out[9],
      'xipos': out[10],
      'xmat': out[11],
      'xpos': out[12],
      'xquat': out[13],
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
def _tendon_shim(
    # Model
    nworld: int,
    body_parentid: wp.array(dtype=int),
    body_rootid: wp.array(dtype=int),
    dof_bodyid: wp.array(dtype=int),
    geom_bodyid: wp.array(dtype=int),
    geom_size: wp.array2d(dtype=wp.vec3),
    jnt_dofadr: wp.array(dtype=int),
    jnt_qposadr: wp.array(dtype=int),
    ntendon: int,
    nv: int,
    nwrap: int,
    site_bodyid: wp.array(dtype=int),
    tendon_adr: wp.array(dtype=int),
    tendon_geom_adr: wp.array(dtype=int),
    tendon_jnt_adr: wp.array(dtype=int),
    tendon_num: wp.array(dtype=int),
    tendon_site_pair_adr: wp.array(dtype=int),
    wrap_geom_adr: wp.array(dtype=int),
    wrap_jnt_adr: wp.array(dtype=int),
    wrap_objid: wp.array(dtype=int),
    wrap_prm: wp.array(dtype=float),
    wrap_pulley_scale: wp.array(dtype=float),
    wrap_site_pair_adr: wp.array(dtype=int),
    wrap_type: wp.array(dtype=int),
    # Data
    cdof: wp.array2d(dtype=wp.spatial_vector),
    geom_xmat: wp.array2d(dtype=wp.mat33),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    qpos: wp.array2d(dtype=float),
    site_xpos: wp.array2d(dtype=wp.vec3),
    subtree_com: wp.array2d(dtype=wp.vec3),
    ten_J: wp.array3d(dtype=float),
    ten_length: wp.array2d(dtype=float),
    ten_wrapadr: wp.array2d(dtype=int),
    ten_wrapnum: wp.array2d(dtype=int),
    wrap_obj: wp.array2d(dtype=wp.vec2i),
    wrap_xpos: wp.array2d(dtype=wp.spatial_vector),
):
  _m.stat = _s
  _m.opt = _o
  _d.efc = _e
  _d.contact = _c
  _m.body_parentid = body_parentid
  _m.body_rootid = body_rootid
  _m.dof_bodyid = dof_bodyid
  _m.geom_bodyid = geom_bodyid
  _m.geom_size = geom_size
  _m.jnt_dofadr = jnt_dofadr
  _m.jnt_qposadr = jnt_qposadr
  _m.ntendon = ntendon
  _m.nv = nv
  _m.nwrap = nwrap
  _m.site_bodyid = site_bodyid
  _m.tendon_adr = tendon_adr
  _m.tendon_geom_adr = tendon_geom_adr
  _m.tendon_jnt_adr = tendon_jnt_adr
  _m.tendon_num = tendon_num
  _m.tendon_site_pair_adr = tendon_site_pair_adr
  _m.wrap_geom_adr = wrap_geom_adr
  _m.wrap_jnt_adr = wrap_jnt_adr
  _m.wrap_objid = wrap_objid
  _m.wrap_prm = wrap_prm
  _m.wrap_pulley_scale = wrap_pulley_scale
  _m.wrap_site_pair_adr = wrap_site_pair_adr
  _m.wrap_type = wrap_type
  _d.cdof = cdof
  _d.geom_xmat = geom_xmat
  _d.geom_xpos = geom_xpos
  _d.qpos = qpos
  _d.site_xpos = site_xpos
  _d.subtree_com = subtree_com
  _d.ten_J = ten_J
  _d.ten_length = ten_length
  _d.ten_wrapadr = ten_wrapadr
  _d.ten_wrapnum = ten_wrapnum
  _d.wrap_obj = wrap_obj
  _d.wrap_xpos = wrap_xpos
  _d.nworld = nworld
  mjwarp.tendon(_m, _d)


def _tendon_jax_impl(m: types.Model, d: types.Data):
  output_dims = {
      'cdof': d.cdof.shape,
      'geom_xmat': d.geom_xmat.shape,
      'geom_xpos': d.geom_xpos.shape,
      'qpos': d.qpos.shape,
      'site_xpos': d.site_xpos.shape,
      'subtree_com': d.subtree_com.shape,
      'ten_J': d._impl.ten_J.shape,
      'ten_length': d.ten_length.shape,
      'ten_wrapadr': d._impl.ten_wrapadr.shape,
      'ten_wrapnum': d._impl.ten_wrapnum.shape,
      'wrap_obj': d._impl.wrap_obj.shape,
      'wrap_xpos': d._impl.wrap_xpos.shape,
  }
  jf = ffi.jax_callable_variadic_tuple(
      _tendon_shim,
      num_outputs=12,
      output_dims=output_dims,
      vmap_method=None,
      in_out_argnames={
          'cdof',
          'geom_xmat',
          'geom_xpos',
          'qpos',
          'site_xpos',
          'subtree_com',
          'ten_J',
          'ten_length',
          'ten_wrapadr',
          'ten_wrapnum',
          'wrap_obj',
          'wrap_xpos',
      },
  )
  out = jf(
      d.qpos.shape[0],
      m.body_parentid,
      m.body_rootid,
      m.dof_bodyid,
      m.geom_bodyid,
      m.geom_size,
      m.jnt_dofadr,
      m.jnt_qposadr,
      m.ntendon,
      m.nv,
      m.nwrap,
      m.site_bodyid,
      m.tendon_adr,
      m._impl.tendon_geom_adr,
      m._impl.tendon_jnt_adr,
      m.tendon_num,
      m._impl.tendon_site_pair_adr,
      m._impl.wrap_geom_adr,
      m._impl.wrap_jnt_adr,
      m.wrap_objid,
      m.wrap_prm,
      m._impl.wrap_pulley_scale,
      m._impl.wrap_site_pair_adr,
      m.wrap_type,
      d.cdof,
      d.geom_xmat,
      d.geom_xpos,
      d.qpos,
      d.site_xpos,
      d.subtree_com,
      d._impl.ten_J,
      d.ten_length,
      d._impl.ten_wrapadr,
      d._impl.ten_wrapnum,
      d._impl.wrap_obj,
      d._impl.wrap_xpos,
  )
  d = d.tree_replace({
      'cdof': out[0],
      'geom_xmat': out[1],
      'geom_xpos': out[2],
      'qpos': out[3],
      'site_xpos': out[4],
      'subtree_com': out[5],
      '_impl.ten_J': out[6],
      'ten_length': out[7],
      '_impl.ten_wrapadr': out[8],
      '_impl.ten_wrapnum': out[9],
      '_impl.wrap_obj': out[10],
      '_impl.wrap_xpos': out[11],
  })
  return d


@jax.custom_batching.custom_vmap
@ffi.marshal_jax_warp_callable
def tendon(m: types.Model, d: types.Data):
  return _tendon_jax_impl(m, d)
@tendon.def_vmap
@ffi.marshal_custom_vmap
def tendon_vmap(unused_axis_size, is_batched, m, d):
  d = tendon(m, d)
  return d, is_batched[1]
