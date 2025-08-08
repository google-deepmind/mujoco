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
def _collision_shim(
    # Model
    nworld: int,
    block_dim: mjwp_types.BlockDim,
    geom_aabb: wp.array2d(dtype=wp.vec3),
    geom_condim: wp.array(dtype=int),
    geom_dataid: wp.array(dtype=int),
    geom_friction: wp.array2d(dtype=wp.vec3),
    geom_gap: wp.array2d(dtype=float),
    geom_margin: wp.array2d(dtype=float),
    geom_pair_type_count: tuple[int, ...],
    geom_plugin_index: wp.array(dtype=int),
    geom_pos: wp.array2d(dtype=wp.vec3),
    geom_priority: wp.array(dtype=int),
    geom_quat: wp.array2d(dtype=wp.quat),
    geom_rbound: wp.array2d(dtype=float),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_solimp: wp.array2d(dtype=mjwp_types.vec5),
    geom_solmix: wp.array2d(dtype=float),
    geom_solref: wp.array2d(dtype=wp.vec2),
    geom_type: wp.array(dtype=int),
    geompair2hfgeompair: wp.array(dtype=int),
    has_sdf_geom: bool,
    hfield_adr: wp.array(dtype=int),
    hfield_data: wp.array(dtype=float),
    hfield_ncol: wp.array(dtype=int),
    hfield_nrow: wp.array(dtype=int),
    hfield_size: wp.array(dtype=wp.vec4),
    mesh_graph: wp.array(dtype=int),
    mesh_graphadr: wp.array(dtype=int),
    mesh_polyadr: wp.array(dtype=int),
    mesh_polymap: wp.array(dtype=int),
    mesh_polymapadr: wp.array(dtype=int),
    mesh_polymapnum: wp.array(dtype=int),
    mesh_polynormal: wp.array(dtype=wp.vec3),
    mesh_polynum: wp.array(dtype=int),
    mesh_polyvert: wp.array(dtype=int),
    mesh_polyvertadr: wp.array(dtype=int),
    mesh_polyvertnum: wp.array(dtype=int),
    mesh_vert: wp.array(dtype=wp.vec3),
    mesh_vertadr: wp.array(dtype=int),
    mesh_vertnum: wp.array(dtype=int),
    ngeom: int,
    nhfield: int,
    nxn_geom_pair_filtered: wp.array(dtype=wp.vec2i),
    nxn_pairid: wp.array(dtype=int),
    nxn_pairid_filtered: wp.array(dtype=int),
    pair_dim: wp.array(dtype=int),
    pair_friction: wp.array2d(dtype=mjwp_types.vec5),
    pair_gap: wp.array2d(dtype=float),
    pair_margin: wp.array2d(dtype=float),
    pair_solimp: wp.array2d(dtype=mjwp_types.vec5),
    pair_solref: wp.array2d(dtype=wp.vec2),
    pair_solreffriction: wp.array2d(dtype=wp.vec2),
    plugin: wp.array(dtype=int),
    plugin_attr: wp.array(dtype=wp.vec3f),
    opt__broadphase: int,
    opt__broadphase_filter: int,
    opt__disableflags: int,
    opt__epa_iterations: int,
    opt__gjk_iterations: int,
    opt__graph_conditional: bool,
    opt__sdf_initpoints: int,
    opt__sdf_iterations: int,
    # Data
    nconmax: int,
    collision_hftri_index: wp.array(dtype=int),
    collision_pair: wp.array(dtype=wp.vec2i),
    collision_pairid: wp.array(dtype=int),
    collision_worldid: wp.array(dtype=int),
    epa_face: wp.array2d(dtype=wp.vec3i),
    epa_horizon: wp.array2d(dtype=int),
    epa_index: wp.array2d(dtype=int),
    epa_map: wp.array2d(dtype=int),
    epa_norm2: wp.array2d(dtype=float),
    epa_pr: wp.array2d(dtype=wp.vec3),
    epa_vert: wp.array2d(dtype=wp.vec3),
    epa_vert1: wp.array2d(dtype=wp.vec3),
    epa_vert2: wp.array2d(dtype=wp.vec3),
    epa_vert_index1: wp.array2d(dtype=int),
    epa_vert_index2: wp.array2d(dtype=int),
    geom_xmat: wp.array2d(dtype=wp.mat33),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    ncollision: wp.array(dtype=int),
    ncon: wp.array(dtype=int),
    ncon_hfield: wp.array2d(dtype=int),
    sap_cumulative_sum: wp.array2d(dtype=int),
    sap_projection_lower: wp.array3d(dtype=float),
    sap_projection_upper: wp.array2d(dtype=float),
    sap_range: wp.array2d(dtype=int),
    sap_segment_index: wp.array2d(dtype=int),
    sap_sort_index: wp.array3d(dtype=int),
    contact__dim: wp.array(dtype=int),
    contact__dist: wp.array(dtype=float),
    contact__frame: wp.array(dtype=wp.mat33),
    contact__friction: wp.array(dtype=mjwp_types.vec5),
    contact__geom: wp.array(dtype=wp.vec2i),
    contact__includemargin: wp.array(dtype=float),
    contact__pos: wp.array(dtype=wp.vec3),
    contact__solimp: wp.array(dtype=mjwp_types.vec5),
    contact__solref: wp.array(dtype=wp.vec2),
    contact__solreffriction: wp.array(dtype=wp.vec2),
    contact__worldid: wp.array(dtype=int),
):
  _m.stat = _s
  _m.opt = _o
  _d.efc = _e
  _d.contact = _c
  _m.block_dim = block_dim
  _m.geom_aabb = geom_aabb
  _m.geom_condim = geom_condim
  _m.geom_dataid = geom_dataid
  _m.geom_friction = geom_friction
  _m.geom_gap = geom_gap
  _m.geom_margin = geom_margin
  _m.geom_pair_type_count = geom_pair_type_count
  _m.geom_plugin_index = geom_plugin_index
  _m.geom_pos = geom_pos
  _m.geom_priority = geom_priority
  _m.geom_quat = geom_quat
  _m.geom_rbound = geom_rbound
  _m.geom_size = geom_size
  _m.geom_solimp = geom_solimp
  _m.geom_solmix = geom_solmix
  _m.geom_solref = geom_solref
  _m.geom_type = geom_type
  _m.geompair2hfgeompair = geompair2hfgeompair
  _m.has_sdf_geom = has_sdf_geom
  _m.hfield_adr = hfield_adr
  _m.hfield_data = hfield_data
  _m.hfield_ncol = hfield_ncol
  _m.hfield_nrow = hfield_nrow
  _m.hfield_size = hfield_size
  _m.mesh_graph = mesh_graph
  _m.mesh_graphadr = mesh_graphadr
  _m.mesh_polyadr = mesh_polyadr
  _m.mesh_polymap = mesh_polymap
  _m.mesh_polymapadr = mesh_polymapadr
  _m.mesh_polymapnum = mesh_polymapnum
  _m.mesh_polynormal = mesh_polynormal
  _m.mesh_polynum = mesh_polynum
  _m.mesh_polyvert = mesh_polyvert
  _m.mesh_polyvertadr = mesh_polyvertadr
  _m.mesh_polyvertnum = mesh_polyvertnum
  _m.mesh_vert = mesh_vert
  _m.mesh_vertadr = mesh_vertadr
  _m.mesh_vertnum = mesh_vertnum
  _m.ngeom = ngeom
  _m.nhfield = nhfield
  _m.nxn_geom_pair_filtered = nxn_geom_pair_filtered
  _m.nxn_pairid = nxn_pairid
  _m.nxn_pairid_filtered = nxn_pairid_filtered
  _m.opt.broadphase = opt__broadphase
  _m.opt.broadphase_filter = opt__broadphase_filter
  _m.opt.disableflags = opt__disableflags
  _m.opt.epa_iterations = opt__epa_iterations
  _m.opt.gjk_iterations = opt__gjk_iterations
  _m.opt.graph_conditional = opt__graph_conditional
  _m.opt.sdf_initpoints = opt__sdf_initpoints
  _m.opt.sdf_iterations = opt__sdf_iterations
  _m.pair_dim = pair_dim
  _m.pair_friction = pair_friction
  _m.pair_gap = pair_gap
  _m.pair_margin = pair_margin
  _m.pair_solimp = pair_solimp
  _m.pair_solref = pair_solref
  _m.pair_solreffriction = pair_solreffriction
  _m.plugin = plugin
  _m.plugin_attr = plugin_attr
  _d.collision_hftri_index = collision_hftri_index
  _d.collision_pair = collision_pair
  _d.collision_pairid = collision_pairid
  _d.collision_worldid = collision_worldid
  _d.contact.dim = contact__dim
  _d.contact.dist = contact__dist
  _d.contact.frame = contact__frame
  _d.contact.friction = contact__friction
  _d.contact.geom = contact__geom
  _d.contact.includemargin = contact__includemargin
  _d.contact.pos = contact__pos
  _d.contact.solimp = contact__solimp
  _d.contact.solref = contact__solref
  _d.contact.solreffriction = contact__solreffriction
  _d.contact.worldid = contact__worldid
  _d.epa_face = epa_face
  _d.epa_horizon = epa_horizon
  _d.epa_index = epa_index
  _d.epa_map = epa_map
  _d.epa_norm2 = epa_norm2
  _d.epa_pr = epa_pr
  _d.epa_vert = epa_vert
  _d.epa_vert1 = epa_vert1
  _d.epa_vert2 = epa_vert2
  _d.epa_vert_index1 = epa_vert_index1
  _d.epa_vert_index2 = epa_vert_index2
  _d.geom_xmat = geom_xmat
  _d.geom_xpos = geom_xpos
  _d.ncollision = ncollision
  _d.ncon = ncon
  _d.ncon_hfield = ncon_hfield
  _d.nconmax = nconmax
  _d.sap_cumulative_sum = sap_cumulative_sum
  _d.sap_projection_lower = sap_projection_lower
  _d.sap_projection_upper = sap_projection_upper
  _d.sap_range = sap_range
  _d.sap_segment_index = sap_segment_index
  _d.sap_sort_index = sap_sort_index
  _d.nworld = nworld
  mjwarp.collision(_m, _d)


def _collision_jax_impl(m: types.Model, d: types.Data):
  output_dims = {
      'collision_hftri_index': d._impl.collision_hftri_index.shape,
      'collision_pair': d._impl.collision_pair.shape,
      'collision_pairid': d._impl.collision_pairid.shape,
      'collision_worldid': d._impl.collision_worldid.shape,
      'epa_face': d._impl.epa_face.shape,
      'epa_horizon': d._impl.epa_horizon.shape,
      'epa_index': d._impl.epa_index.shape,
      'epa_map': d._impl.epa_map.shape,
      'epa_norm2': d._impl.epa_norm2.shape,
      'epa_pr': d._impl.epa_pr.shape,
      'epa_vert': d._impl.epa_vert.shape,
      'epa_vert1': d._impl.epa_vert1.shape,
      'epa_vert2': d._impl.epa_vert2.shape,
      'epa_vert_index1': d._impl.epa_vert_index1.shape,
      'epa_vert_index2': d._impl.epa_vert_index2.shape,
      'geom_xmat': d.geom_xmat.shape,
      'geom_xpos': d.geom_xpos.shape,
      'ncollision': d._impl.ncollision.shape,
      'ncon': d._impl.ncon.shape,
      'ncon_hfield': d._impl.ncon_hfield.shape,
      'sap_cumulative_sum': d._impl.sap_cumulative_sum.shape,
      'sap_projection_lower': d._impl.sap_projection_lower.shape,
      'sap_projection_upper': d._impl.sap_projection_upper.shape,
      'sap_range': d._impl.sap_range.shape,
      'sap_segment_index': d._impl.sap_segment_index.shape,
      'sap_sort_index': d._impl.sap_sort_index.shape,
      'contact__dim': d._impl.contact__dim.shape,
      'contact__dist': d._impl.contact__dist.shape,
      'contact__frame': d._impl.contact__frame.shape,
      'contact__friction': d._impl.contact__friction.shape,
      'contact__geom': d._impl.contact__geom.shape,
      'contact__includemargin': d._impl.contact__includemargin.shape,
      'contact__pos': d._impl.contact__pos.shape,
      'contact__solimp': d._impl.contact__solimp.shape,
      'contact__solref': d._impl.contact__solref.shape,
      'contact__solreffriction': d._impl.contact__solreffriction.shape,
      'contact__worldid': d._impl.contact__worldid.shape,
  }
  jf = ffi.jax_callable_variadic_tuple(
      _collision_shim,
      num_outputs=37,
      output_dims=output_dims,
      vmap_method=None,
      in_out_argnames={
          'collision_hftri_index',
          'collision_pair',
          'collision_pairid',
          'collision_worldid',
          'epa_face',
          'epa_horizon',
          'epa_index',
          'epa_map',
          'epa_norm2',
          'epa_pr',
          'epa_vert',
          'epa_vert1',
          'epa_vert2',
          'epa_vert_index1',
          'epa_vert_index2',
          'geom_xmat',
          'geom_xpos',
          'ncollision',
          'ncon',
          'ncon_hfield',
          'sap_cumulative_sum',
          'sap_projection_lower',
          'sap_projection_upper',
          'sap_range',
          'sap_segment_index',
          'sap_sort_index',
          'contact__dim',
          'contact__dist',
          'contact__frame',
          'contact__friction',
          'contact__geom',
          'contact__includemargin',
          'contact__pos',
          'contact__solimp',
          'contact__solref',
          'contact__solreffriction',
          'contact__worldid',
      },
  )
  out = jf(
      d.qpos.shape[0],
      m._impl.block_dim,
      m.geom_aabb,
      m.geom_condim,
      m.geom_dataid,
      m.geom_friction,
      m.geom_gap,
      m.geom_margin,
      m._impl.geom_pair_type_count,
      m._impl.geom_plugin_index,
      m.geom_pos,
      m.geom_priority,
      m.geom_quat,
      m.geom_rbound,
      m.geom_size,
      m.geom_solimp,
      m.geom_solmix,
      m.geom_solref,
      m.geom_type,
      m._impl.geompair2hfgeompair,
      m._impl.has_sdf_geom,
      m.hfield_adr,
      m.hfield_data,
      m.hfield_ncol,
      m.hfield_nrow,
      m.hfield_size,
      m.mesh_graph,
      m.mesh_graphadr,
      m._impl.mesh_polyadr,
      m._impl.mesh_polymap,
      m._impl.mesh_polymapadr,
      m._impl.mesh_polymapnum,
      m._impl.mesh_polynormal,
      m._impl.mesh_polynum,
      m._impl.mesh_polyvert,
      m._impl.mesh_polyvertadr,
      m._impl.mesh_polyvertnum,
      m.mesh_vert,
      m.mesh_vertadr,
      m.mesh_vertnum,
      m.ngeom,
      m.nhfield,
      m._impl.nxn_geom_pair_filtered,
      m._impl.nxn_pairid,
      m._impl.nxn_pairid_filtered,
      m.pair_dim,
      m.pair_friction,
      m.pair_gap,
      m.pair_margin,
      m.pair_solimp,
      m.pair_solref,
      m.pair_solreffriction,
      m._impl.plugin,
      m._impl.plugin_attr,
      m.opt._impl.broadphase,
      m.opt._impl.broadphase_filter,
      m.opt.disableflags,
      m.opt._impl.epa_iterations,
      m.opt._impl.gjk_iterations,
      m.opt._impl.graph_conditional,
      m.opt._impl.sdf_initpoints,
      m.opt._impl.sdf_iterations,
      d._impl.nconmax,
      d._impl.collision_hftri_index,
      d._impl.collision_pair,
      d._impl.collision_pairid,
      d._impl.collision_worldid,
      d._impl.epa_face,
      d._impl.epa_horizon,
      d._impl.epa_index,
      d._impl.epa_map,
      d._impl.epa_norm2,
      d._impl.epa_pr,
      d._impl.epa_vert,
      d._impl.epa_vert1,
      d._impl.epa_vert2,
      d._impl.epa_vert_index1,
      d._impl.epa_vert_index2,
      d.geom_xmat,
      d.geom_xpos,
      d._impl.ncollision,
      d._impl.ncon,
      d._impl.ncon_hfield,
      d._impl.sap_cumulative_sum,
      d._impl.sap_projection_lower,
      d._impl.sap_projection_upper,
      d._impl.sap_range,
      d._impl.sap_segment_index,
      d._impl.sap_sort_index,
      d._impl.contact__dim,
      d._impl.contact__dist,
      d._impl.contact__frame,
      d._impl.contact__friction,
      d._impl.contact__geom,
      d._impl.contact__includemargin,
      d._impl.contact__pos,
      d._impl.contact__solimp,
      d._impl.contact__solref,
      d._impl.contact__solreffriction,
      d._impl.contact__worldid,
  )
  d = d.tree_replace({
      '_impl.collision_hftri_index': out[0],
      '_impl.collision_pair': out[1],
      '_impl.collision_pairid': out[2],
      '_impl.collision_worldid': out[3],
      '_impl.epa_face': out[4],
      '_impl.epa_horizon': out[5],
      '_impl.epa_index': out[6],
      '_impl.epa_map': out[7],
      '_impl.epa_norm2': out[8],
      '_impl.epa_pr': out[9],
      '_impl.epa_vert': out[10],
      '_impl.epa_vert1': out[11],
      '_impl.epa_vert2': out[12],
      '_impl.epa_vert_index1': out[13],
      '_impl.epa_vert_index2': out[14],
      'geom_xmat': out[15],
      'geom_xpos': out[16],
      '_impl.ncollision': out[17],
      '_impl.ncon': out[18],
      '_impl.ncon_hfield': out[19],
      '_impl.sap_cumulative_sum': out[20],
      '_impl.sap_projection_lower': out[21],
      '_impl.sap_projection_upper': out[22],
      '_impl.sap_range': out[23],
      '_impl.sap_segment_index': out[24],
      '_impl.sap_sort_index': out[25],
      '_impl.contact__dim': out[26],
      '_impl.contact__dist': out[27],
      '_impl.contact__frame': out[28],
      '_impl.contact__friction': out[29],
      '_impl.contact__geom': out[30],
      '_impl.contact__includemargin': out[31],
      '_impl.contact__pos': out[32],
      '_impl.contact__solimp': out[33],
      '_impl.contact__solref': out[34],
      '_impl.contact__solreffriction': out[35],
      '_impl.contact__worldid': out[36],
  })
  return d


@jax.custom_batching.custom_vmap
@ffi.marshal_jax_warp_callable
def collision(m: types.Model, d: types.Data):
  return _collision_jax_impl(m, d)
@collision.def_vmap
@ffi.marshal_custom_vmap
def collision_vmap(unused_axis_size, is_batched, m, d):
  d = collision(m, d)
  return d, is_batched[1]
