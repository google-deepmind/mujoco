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
def _forward_shim(
    # Model
    nworld: int,
    M_rowadr: wp.array(dtype=int),
    M_rownnz: wp.array(dtype=int),
    actuator_acc0: wp.array(dtype=float),
    actuator_actadr: wp.array(dtype=int),
    actuator_actearly: wp.array(dtype=bool),
    actuator_actlimited: wp.array(dtype=bool),
    actuator_actnum: wp.array(dtype=int),
    actuator_actrange: wp.array2d(dtype=wp.vec2),
    actuator_biasprm: wp.array2d(dtype=mjwp_types.vec10f),
    actuator_biastype: wp.array(dtype=int),
    actuator_cranklength: wp.array(dtype=float),
    actuator_ctrllimited: wp.array(dtype=bool),
    actuator_ctrlrange: wp.array2d(dtype=wp.vec2),
    actuator_dynprm: wp.array2d(dtype=mjwp_types.vec10f),
    actuator_dyntype: wp.array(dtype=int),
    actuator_forcelimited: wp.array(dtype=bool),
    actuator_forcerange: wp.array2d(dtype=wp.vec2),
    actuator_gainprm: wp.array2d(dtype=mjwp_types.vec10f),
    actuator_gaintype: wp.array(dtype=int),
    actuator_gear: wp.array2d(dtype=wp.spatial_vector),
    actuator_lengthrange: wp.array(dtype=wp.vec2),
    actuator_trnid: wp.array(dtype=wp.vec2i),
    actuator_trntype: wp.array(dtype=int),
    actuator_trntype_body_adr: wp.array(dtype=int),
    block_dim: mjwp_types.BlockDim,
    body_dofadr: wp.array(dtype=int),
    body_dofnum: wp.array(dtype=int),
    body_fluid_ellipsoid: wp.array(dtype=bool),
    body_geomadr: wp.array(dtype=int),
    body_geomnum: wp.array(dtype=int),
    body_gravcomp: wp.array2d(dtype=float),
    body_inertia: wp.array2d(dtype=wp.vec3),
    body_invweight0: wp.array2d(dtype=wp.vec2),
    body_ipos: wp.array2d(dtype=wp.vec3),
    body_iquat: wp.array2d(dtype=wp.quat),
    body_jntadr: wp.array(dtype=int),
    body_jntnum: wp.array(dtype=int),
    body_mass: wp.array2d(dtype=float),
    body_mocapid: wp.array(dtype=int),
    body_parentid: wp.array(dtype=int),
    body_pos: wp.array2d(dtype=wp.vec3),
    body_quat: wp.array2d(dtype=wp.quat),
    body_rootid: wp.array(dtype=int),
    body_subtreemass: wp.array2d(dtype=float),
    body_tree: tuple[wp.array(dtype=int), ...],
    body_weldid: wp.array(dtype=int),
    cam_bodyid: wp.array(dtype=int),
    cam_fovy: wp.array(dtype=float),
    cam_intrinsic: wp.array(dtype=wp.vec4),
    cam_mat0: wp.array2d(dtype=wp.mat33),
    cam_mode: wp.array(dtype=int),
    cam_pos: wp.array2d(dtype=wp.vec3),
    cam_pos0: wp.array2d(dtype=wp.vec3),
    cam_poscom0: wp.array2d(dtype=wp.vec3),
    cam_quat: wp.array2d(dtype=wp.quat),
    cam_resolution: wp.array(dtype=wp.vec2i),
    cam_sensorsize: wp.array(dtype=wp.vec2),
    cam_targetbodyid: wp.array(dtype=int),
    dof_Madr: wp.array(dtype=int),
    dof_armature: wp.array2d(dtype=float),
    dof_bodyid: wp.array(dtype=int),
    dof_damping: wp.array2d(dtype=float),
    dof_frictionloss: wp.array2d(dtype=float),
    dof_invweight0: wp.array2d(dtype=float),
    dof_jntid: wp.array(dtype=int),
    dof_parentid: wp.array(dtype=int),
    dof_solimp: wp.array2d(dtype=mjwp_types.vec5),
    dof_solref: wp.array2d(dtype=wp.vec2),
    dof_tri_col: wp.array(dtype=int),
    dof_tri_row: wp.array(dtype=int),
    eq_connect_adr: wp.array(dtype=int),
    eq_data: wp.array2d(dtype=mjwp_types.vec11),
    eq_flex_adr: wp.array(dtype=int),
    eq_jnt_adr: wp.array(dtype=int),
    eq_obj1id: wp.array(dtype=int),
    eq_obj2id: wp.array(dtype=int),
    eq_objtype: wp.array(dtype=int),
    eq_solimp: wp.array2d(dtype=mjwp_types.vec5),
    eq_solref: wp.array2d(dtype=wp.vec2),
    eq_ten_adr: wp.array(dtype=int),
    eq_wld_adr: wp.array(dtype=int),
    flex_bending: wp.array2d(dtype=float),
    flex_damping: wp.array(dtype=float),
    flex_dim: wp.array(dtype=int),
    flex_edge: wp.array(dtype=wp.vec2i),
    flex_edgeadr: wp.array(dtype=int),
    flex_edgeflap: wp.array(dtype=wp.vec2i),
    flex_edgenum: wp.array(dtype=int),
    flex_elem: wp.array(dtype=int),
    flex_elemadr: wp.array(dtype=int),
    flex_elemedge: wp.array(dtype=int),
    flex_elemedgeadr: wp.array(dtype=int),
    flex_elemnum: wp.array(dtype=int),
    flex_stiffness: wp.array2d(dtype=float),
    flex_vertadr: wp.array(dtype=int),
    flex_vertbodyid: wp.array(dtype=int),
    flexedge_invweight0: wp.array(dtype=float),
    flexedge_length0: wp.array(dtype=float),
    geom_aabb: wp.array3d(dtype=wp.vec3),
    geom_bodyid: wp.array(dtype=int),
    geom_condim: wp.array(dtype=int),
    geom_dataid: wp.array(dtype=int),
    geom_fluid: wp.array2d(dtype=float),
    geom_friction: wp.array2d(dtype=wp.vec3),
    geom_gap: wp.array2d(dtype=float),
    geom_group: wp.array(dtype=int),
    geom_margin: wp.array2d(dtype=float),
    geom_matid: wp.array2d(dtype=int),
    geom_pair_type_count: tuple[int, ...],
    geom_plugin_index: wp.array(dtype=int),
    geom_pos: wp.array2d(dtype=wp.vec3),
    geom_priority: wp.array(dtype=int),
    geom_quat: wp.array2d(dtype=wp.quat),
    geom_rbound: wp.array2d(dtype=float),
    geom_rgba: wp.array2d(dtype=wp.vec4),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_solimp: wp.array2d(dtype=mjwp_types.vec5),
    geom_solmix: wp.array2d(dtype=float),
    geom_solref: wp.array2d(dtype=wp.vec2),
    geom_type: wp.array(dtype=int),
    has_sdf_geom: bool,
    hfield_adr: wp.array(dtype=int),
    hfield_data: wp.array(dtype=float),
    hfield_ncol: wp.array(dtype=int),
    hfield_nrow: wp.array(dtype=int),
    hfield_size: wp.array(dtype=wp.vec4),
    jnt_actfrclimited: wp.array(dtype=bool),
    jnt_actfrcrange: wp.array2d(dtype=wp.vec2),
    jnt_actgravcomp: wp.array(dtype=int),
    jnt_axis: wp.array2d(dtype=wp.vec3),
    jnt_bodyid: wp.array(dtype=int),
    jnt_dofadr: wp.array(dtype=int),
    jnt_limited_ball_adr: wp.array(dtype=int),
    jnt_limited_slide_hinge_adr: wp.array(dtype=int),
    jnt_margin: wp.array2d(dtype=float),
    jnt_pos: wp.array2d(dtype=wp.vec3),
    jnt_qposadr: wp.array(dtype=int),
    jnt_range: wp.array2d(dtype=wp.vec2),
    jnt_solimp: wp.array2d(dtype=mjwp_types.vec5),
    jnt_solref: wp.array2d(dtype=wp.vec2),
    jnt_stiffness: wp.array2d(dtype=float),
    jnt_type: wp.array(dtype=int),
    light_bodyid: wp.array(dtype=int),
    light_dir: wp.array2d(dtype=wp.vec3),
    light_dir0: wp.array2d(dtype=wp.vec3),
    light_mode: wp.array(dtype=int),
    light_pos: wp.array2d(dtype=wp.vec3),
    light_pos0: wp.array2d(dtype=wp.vec3),
    light_poscom0: wp.array2d(dtype=wp.vec3),
    light_targetbodyid: wp.array(dtype=int),
    mapM2M: wp.array(dtype=int),
    mat_rgba: wp.array2d(dtype=wp.vec4),
    mesh_face: wp.array(dtype=wp.vec3i),
    mesh_faceadr: wp.array(dtype=int),
    mesh_graph: wp.array(dtype=int),
    mesh_graphadr: wp.array(dtype=int),
    mesh_normal: wp.array(dtype=wp.vec3),
    mesh_normaladr: wp.array(dtype=int),
    mesh_polyadr: wp.array(dtype=int),
    mesh_polymap: wp.array(dtype=int),
    mesh_polymapadr: wp.array(dtype=int),
    mesh_polymapnum: wp.array(dtype=int),
    mesh_polynormal: wp.array(dtype=wp.vec3),
    mesh_polynum: wp.array(dtype=int),
    mesh_polyvert: wp.array(dtype=int),
    mesh_polyvertadr: wp.array(dtype=int),
    mesh_polyvertnum: wp.array(dtype=int),
    mesh_quat: wp.array(dtype=wp.quat),
    mesh_vert: wp.array(dtype=wp.vec3),
    mesh_vertadr: wp.array(dtype=int),
    mesh_vertnum: wp.array(dtype=int),
    nC: int,
    na: int,
    nacttrnbody: int,
    nbody: int,
    ncam: int,
    neq: int,
    nflex: int,
    nflexedge: int,
    nflexelem: int,
    nflexvert: int,
    ngeom: int,
    ngravcomp: int,
    njnt: int,
    nlight: int,
    nmaxcondim: int,
    nmaxmeshdeg: int,
    nmaxpolygon: int,
    nmaxpyramid: int,
    nmeshface: int,
    nrangefinder: int,
    nsensorcollision: int,
    nsensorcontact: int,
    nsensortaxel: int,
    nsite: int,
    ntendon: int,
    nu: int,
    nv: int,
    nv_pad: int,
    nwrap: int,
    nxn_geom_pair_filtered: wp.array(dtype=wp.vec2i),
    nxn_pairid: wp.array(dtype=wp.vec2i),
    nxn_pairid_filtered: wp.array(dtype=wp.vec2i),
    oct_aabb: wp.array2d(dtype=wp.vec3),
    oct_child: wp.array(dtype=mjwp_types.vec8i),
    oct_coeff: wp.array(dtype=mjwp_types.vec8),
    pair_dim: wp.array(dtype=int),
    pair_friction: wp.array2d(dtype=mjwp_types.vec5),
    pair_gap: wp.array2d(dtype=float),
    pair_margin: wp.array2d(dtype=float),
    pair_solimp: wp.array2d(dtype=mjwp_types.vec5),
    pair_solref: wp.array2d(dtype=wp.vec2),
    pair_solreffriction: wp.array2d(dtype=wp.vec2),
    plugin: wp.array(dtype=int),
    plugin_attr: wp.array(dtype=wp.vec3f),
    qLD_updates: tuple[wp.array(dtype=wp.vec3i), ...],
    qM_fullm_i: wp.array(dtype=int),
    qM_fullm_j: wp.array(dtype=int),
    qM_madr_ij: wp.array(dtype=int),
    qM_mulm_i: wp.array(dtype=int),
    qM_mulm_j: wp.array(dtype=int),
    qM_tiles: tuple[mjwp_types.TileSet, ...],
    qpos0: wp.array2d(dtype=float),
    qpos_spring: wp.array2d(dtype=float),
    rangefinder_sensor_adr: wp.array(dtype=int),
    sensor_acc_adr: wp.array(dtype=int),
    sensor_adr: wp.array(dtype=int),
    sensor_adr_to_contact_adr: wp.array(dtype=int),
    sensor_contact_adr: wp.array(dtype=int),
    sensor_cutoff: wp.array(dtype=float),
    sensor_datatype: wp.array(dtype=int),
    sensor_dim: wp.array(dtype=int),
    sensor_e_kinetic: bool,
    sensor_e_potential: bool,
    sensor_intprm: wp.array2d(dtype=int),
    sensor_limitfrc_adr: wp.array(dtype=int),
    sensor_limitpos_adr: wp.array(dtype=int),
    sensor_limitvel_adr: wp.array(dtype=int),
    sensor_objid: wp.array(dtype=int),
    sensor_objtype: wp.array(dtype=int),
    sensor_pos_adr: wp.array(dtype=int),
    sensor_rangefinder_adr: wp.array(dtype=int),
    sensor_rangefinder_bodyid: wp.array(dtype=int),
    sensor_refid: wp.array(dtype=int),
    sensor_reftype: wp.array(dtype=int),
    sensor_rne_postconstraint: bool,
    sensor_subtree_vel: bool,
    sensor_tendonactfrc_adr: wp.array(dtype=int),
    sensor_touch_adr: wp.array(dtype=int),
    sensor_type: wp.array(dtype=int),
    sensor_vel_adr: wp.array(dtype=int),
    site_bodyid: wp.array(dtype=int),
    site_pos: wp.array2d(dtype=wp.vec3),
    site_quat: wp.array2d(dtype=wp.quat),
    site_size: wp.array(dtype=wp.vec3),
    site_type: wp.array(dtype=int),
    taxel_sensorid: wp.array(dtype=int),
    taxel_vertadr: wp.array(dtype=int),
    tendon_actfrclimited: wp.array(dtype=bool),
    tendon_actfrcrange: wp.array2d(dtype=wp.vec2),
    tendon_adr: wp.array(dtype=int),
    tendon_armature: wp.array2d(dtype=float),
    tendon_damping: wp.array2d(dtype=float),
    tendon_frictionloss: wp.array2d(dtype=float),
    tendon_geom_adr: wp.array(dtype=int),
    tendon_invweight0: wp.array2d(dtype=float),
    tendon_jnt_adr: wp.array(dtype=int),
    tendon_length0: wp.array2d(dtype=float),
    tendon_lengthspring: wp.array2d(dtype=wp.vec2),
    tendon_limited_adr: wp.array(dtype=int),
    tendon_margin: wp.array2d(dtype=float),
    tendon_num: wp.array(dtype=int),
    tendon_range: wp.array2d(dtype=wp.vec2),
    tendon_site_pair_adr: wp.array(dtype=int),
    tendon_solimp_fri: wp.array2d(dtype=mjwp_types.vec5),
    tendon_solimp_lim: wp.array2d(dtype=mjwp_types.vec5),
    tendon_solref_fri: wp.array2d(dtype=wp.vec2),
    tendon_solref_lim: wp.array2d(dtype=wp.vec2),
    tendon_stiffness: wp.array2d(dtype=float),
    wrap_geom_adr: wp.array(dtype=int),
    wrap_jnt_adr: wp.array(dtype=int),
    wrap_objid: wp.array(dtype=int),
    wrap_prm: wp.array(dtype=float),
    wrap_pulley_scale: wp.array(dtype=float),
    wrap_site_pair_adr: wp.array(dtype=int),
    wrap_type: wp.array(dtype=int),
    opt__broadphase: int,
    opt__broadphase_filter: int,
    opt__ccd_iterations: int,
    opt__ccd_tolerance: wp.array(dtype=float),
    opt__cone: int,
    opt__contact_sensor_maxmatch: int,
    opt__density: wp.array(dtype=float),
    opt__disableflags: int,
    opt__enableflags: int,
    opt__graph_conditional: bool,
    opt__gravity: wp.array(dtype=wp.vec3),
    opt__has_fluid: bool,
    opt__impratio_invsqrt: wp.array(dtype=float),
    opt__is_sparse: bool,
    opt__iterations: int,
    opt__ls_iterations: int,
    opt__ls_parallel: bool,
    opt__ls_parallel_min_step: float,
    opt__ls_tolerance: wp.array(dtype=float),
    opt__magnetic: wp.array(dtype=wp.vec3),
    opt__run_collision_detection: bool,
    opt__sdf_initpoints: int,
    opt__sdf_iterations: int,
    opt__solver: int,
    opt__timestep: wp.array(dtype=float),
    opt__tolerance: wp.array(dtype=float),
    opt__viscosity: wp.array(dtype=float),
    opt__wind: wp.array(dtype=wp.vec3),
    stat__meaninertia: float,
    # Data
    naconmax: int,
    njmax: int,
    act: wp.array2d(dtype=float),
    act_dot: wp.array2d(dtype=float),
    actuator_force: wp.array2d(dtype=float),
    actuator_length: wp.array2d(dtype=float),
    actuator_moment: wp.array3d(dtype=float),
    actuator_velocity: wp.array2d(dtype=float),
    cacc: wp.array2d(dtype=wp.spatial_vector),
    cam_xmat: wp.array2d(dtype=wp.mat33),
    cam_xpos: wp.array2d(dtype=wp.vec3),
    cdof: wp.array2d(dtype=wp.spatial_vector),
    cdof_dot: wp.array2d(dtype=wp.spatial_vector),
    cfrc_ext: wp.array2d(dtype=wp.spatial_vector),
    cfrc_int: wp.array2d(dtype=wp.spatial_vector),
    cinert: wp.array2d(dtype=mjwp_types.vec10),
    collision_pair: wp.array(dtype=wp.vec2i),
    collision_pairid: wp.array(dtype=wp.vec2i),
    collision_worldid: wp.array(dtype=int),
    crb: wp.array2d(dtype=mjwp_types.vec10),
    ctrl: wp.array2d(dtype=float),
    cvel: wp.array2d(dtype=wp.spatial_vector),
    energy: wp.array(dtype=wp.vec2),
    eq_active: wp.array2d(dtype=bool),
    flexedge_J: wp.array3d(dtype=float),
    flexedge_length: wp.array2d(dtype=float),
    flexedge_velocity: wp.array2d(dtype=float),
    flexvert_xpos: wp.array2d(dtype=wp.vec3),
    geom_xmat: wp.array2d(dtype=wp.mat33),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    light_xdir: wp.array2d(dtype=wp.vec3),
    light_xpos: wp.array2d(dtype=wp.vec3),
    mocap_pos: wp.array2d(dtype=wp.vec3),
    mocap_quat: wp.array2d(dtype=wp.quat),
    nacon: wp.array(dtype=int),
    ncollision: wp.array(dtype=int),
    ne: wp.array(dtype=int),
    ne_connect: wp.array(dtype=int),
    ne_flex: wp.array(dtype=int),
    ne_jnt: wp.array(dtype=int),
    ne_ten: wp.array(dtype=int),
    ne_weld: wp.array(dtype=int),
    nefc: wp.array(dtype=int),
    nf: wp.array(dtype=int),
    nl: wp.array(dtype=int),
    nsolving: wp.array(dtype=int),
    qLD: wp.array3d(dtype=float),
    qLDiagInv: wp.array2d(dtype=float),
    qM: wp.array3d(dtype=float),
    qacc: wp.array2d(dtype=float),
    qacc_smooth: wp.array2d(dtype=float),
    qacc_warmstart: wp.array2d(dtype=float),
    qfrc_actuator: wp.array2d(dtype=float),
    qfrc_applied: wp.array2d(dtype=float),
    qfrc_bias: wp.array2d(dtype=float),
    qfrc_constraint: wp.array2d(dtype=float),
    qfrc_damper: wp.array2d(dtype=float),
    qfrc_fluid: wp.array2d(dtype=float),
    qfrc_gravcomp: wp.array2d(dtype=float),
    qfrc_passive: wp.array2d(dtype=float),
    qfrc_smooth: wp.array2d(dtype=float),
    qfrc_spring: wp.array2d(dtype=float),
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    sensordata: wp.array2d(dtype=float),
    site_xmat: wp.array2d(dtype=wp.mat33),
    site_xpos: wp.array2d(dtype=wp.vec3),
    solver_niter: wp.array(dtype=int),
    subtree_angmom: wp.array2d(dtype=wp.vec3),
    subtree_bodyvel: wp.array2d(dtype=wp.spatial_vector),
    subtree_com: wp.array2d(dtype=wp.vec3),
    subtree_linvel: wp.array2d(dtype=wp.vec3),
    ten_J: wp.array3d(dtype=float),
    ten_length: wp.array2d(dtype=float),
    ten_velocity: wp.array2d(dtype=float),
    ten_wrapadr: wp.array2d(dtype=int),
    ten_wrapnum: wp.array2d(dtype=int),
    time: wp.array(dtype=float),
    wrap_obj: wp.array2d(dtype=wp.vec2i),
    wrap_xpos: wp.array2d(dtype=wp.spatial_vector),
    xanchor: wp.array2d(dtype=wp.vec3),
    xaxis: wp.array2d(dtype=wp.vec3),
    xfrc_applied: wp.array2d(dtype=wp.spatial_vector),
    ximat: wp.array2d(dtype=wp.mat33),
    xipos: wp.array2d(dtype=wp.vec3),
    xmat: wp.array2d(dtype=wp.mat33),
    xpos: wp.array2d(dtype=wp.vec3),
    xquat: wp.array2d(dtype=wp.quat),
    contact__dim: wp.array(dtype=int),
    contact__dist: wp.array(dtype=float),
    contact__efc_address: wp.array2d(dtype=int),
    contact__frame: wp.array(dtype=wp.mat33),
    contact__friction: wp.array(dtype=mjwp_types.vec5),
    contact__geom: wp.array(dtype=wp.vec2i),
    contact__geomcollisionid: wp.array(dtype=int),
    contact__includemargin: wp.array(dtype=float),
    contact__pos: wp.array(dtype=wp.vec3),
    contact__solimp: wp.array(dtype=mjwp_types.vec5),
    contact__solref: wp.array(dtype=wp.vec2),
    contact__solreffriction: wp.array(dtype=wp.vec2),
    contact__type: wp.array(dtype=int),
    contact__worldid: wp.array(dtype=int),
    efc__D: wp.array2d(dtype=float),
    efc__J: wp.array3d(dtype=float),
    efc__Jaref: wp.array2d(dtype=float),
    efc__Ma: wp.array2d(dtype=float),
    efc__Mgrad: wp.array2d(dtype=float),
    efc__alpha: wp.array(dtype=float),
    efc__aref: wp.array2d(dtype=float),
    efc__beta: wp.array(dtype=float),
    efc__cost: wp.array(dtype=float),
    efc__done: wp.array(dtype=bool),
    efc__force: wp.array2d(dtype=float),
    efc__frictionloss: wp.array2d(dtype=float),
    efc__gauss: wp.array(dtype=float),
    efc__grad: wp.array2d(dtype=float),
    efc__grad_dot: wp.array(dtype=float),
    efc__id: wp.array2d(dtype=int),
    efc__jv: wp.array2d(dtype=float),
    efc__margin: wp.array2d(dtype=float),
    efc__mv: wp.array2d(dtype=float),
    efc__pos: wp.array2d(dtype=float),
    efc__prev_Mgrad: wp.array2d(dtype=float),
    efc__prev_cost: wp.array(dtype=float),
    efc__prev_grad: wp.array2d(dtype=float),
    efc__quad: wp.array2d(dtype=wp.vec3),
    efc__quad_gauss: wp.array(dtype=wp.vec3),
    efc__search: wp.array2d(dtype=float),
    efc__search_dot: wp.array(dtype=float),
    efc__state: wp.array2d(dtype=int),
    efc__type: wp.array2d(dtype=int),
    efc__vel: wp.array2d(dtype=float),
):
  _m.stat = _s
  _m.opt = _o
  _d.efc = _e
  _d.contact = _c
  _m.M_rowadr = M_rowadr
  _m.M_rownnz = M_rownnz
  _m.actuator_acc0 = actuator_acc0
  _m.actuator_actadr = actuator_actadr
  _m.actuator_actearly = actuator_actearly
  _m.actuator_actlimited = actuator_actlimited
  _m.actuator_actnum = actuator_actnum
  _m.actuator_actrange = actuator_actrange
  _m.actuator_biasprm = actuator_biasprm
  _m.actuator_biastype = actuator_biastype
  _m.actuator_cranklength = actuator_cranklength
  _m.actuator_ctrllimited = actuator_ctrllimited
  _m.actuator_ctrlrange = actuator_ctrlrange
  _m.actuator_dynprm = actuator_dynprm
  _m.actuator_dyntype = actuator_dyntype
  _m.actuator_forcelimited = actuator_forcelimited
  _m.actuator_forcerange = actuator_forcerange
  _m.actuator_gainprm = actuator_gainprm
  _m.actuator_gaintype = actuator_gaintype
  _m.actuator_gear = actuator_gear
  _m.actuator_lengthrange = actuator_lengthrange
  _m.actuator_trnid = actuator_trnid
  _m.actuator_trntype = actuator_trntype
  _m.actuator_trntype_body_adr = actuator_trntype_body_adr
  _m.block_dim = block_dim
  _m.body_dofadr = body_dofadr
  _m.body_dofnum = body_dofnum
  _m.body_fluid_ellipsoid = body_fluid_ellipsoid
  _m.body_geomadr = body_geomadr
  _m.body_geomnum = body_geomnum
  _m.body_gravcomp = body_gravcomp
  _m.body_inertia = body_inertia
  _m.body_invweight0 = body_invweight0
  _m.body_ipos = body_ipos
  _m.body_iquat = body_iquat
  _m.body_jntadr = body_jntadr
  _m.body_jntnum = body_jntnum
  _m.body_mass = body_mass
  _m.body_mocapid = body_mocapid
  _m.body_parentid = body_parentid
  _m.body_pos = body_pos
  _m.body_quat = body_quat
  _m.body_rootid = body_rootid
  _m.body_subtreemass = body_subtreemass
  _m.body_tree = body_tree
  _m.body_weldid = body_weldid
  _m.cam_bodyid = cam_bodyid
  _m.cam_fovy = cam_fovy
  _m.cam_intrinsic = cam_intrinsic
  _m.cam_mat0 = cam_mat0
  _m.cam_mode = cam_mode
  _m.cam_pos = cam_pos
  _m.cam_pos0 = cam_pos0
  _m.cam_poscom0 = cam_poscom0
  _m.cam_quat = cam_quat
  _m.cam_resolution = cam_resolution
  _m.cam_sensorsize = cam_sensorsize
  _m.cam_targetbodyid = cam_targetbodyid
  _m.dof_Madr = dof_Madr
  _m.dof_armature = dof_armature
  _m.dof_bodyid = dof_bodyid
  _m.dof_damping = dof_damping
  _m.dof_frictionloss = dof_frictionloss
  _m.dof_invweight0 = dof_invweight0
  _m.dof_jntid = dof_jntid
  _m.dof_parentid = dof_parentid
  _m.dof_solimp = dof_solimp
  _m.dof_solref = dof_solref
  _m.dof_tri_col = dof_tri_col
  _m.dof_tri_row = dof_tri_row
  _m.eq_connect_adr = eq_connect_adr
  _m.eq_data = eq_data
  _m.eq_flex_adr = eq_flex_adr
  _m.eq_jnt_adr = eq_jnt_adr
  _m.eq_obj1id = eq_obj1id
  _m.eq_obj2id = eq_obj2id
  _m.eq_objtype = eq_objtype
  _m.eq_solimp = eq_solimp
  _m.eq_solref = eq_solref
  _m.eq_ten_adr = eq_ten_adr
  _m.eq_wld_adr = eq_wld_adr
  _m.flex_bending = flex_bending
  _m.flex_damping = flex_damping
  _m.flex_dim = flex_dim
  _m.flex_edge = flex_edge
  _m.flex_edgeadr = flex_edgeadr
  _m.flex_edgeflap = flex_edgeflap
  _m.flex_edgenum = flex_edgenum
  _m.flex_elem = flex_elem
  _m.flex_elemadr = flex_elemadr
  _m.flex_elemedge = flex_elemedge
  _m.flex_elemedgeadr = flex_elemedgeadr
  _m.flex_elemnum = flex_elemnum
  _m.flex_stiffness = flex_stiffness
  _m.flex_vertadr = flex_vertadr
  _m.flex_vertbodyid = flex_vertbodyid
  _m.flexedge_invweight0 = flexedge_invweight0
  _m.flexedge_length0 = flexedge_length0
  _m.geom_aabb = geom_aabb
  _m.geom_bodyid = geom_bodyid
  _m.geom_condim = geom_condim
  _m.geom_dataid = geom_dataid
  _m.geom_fluid = geom_fluid
  _m.geom_friction = geom_friction
  _m.geom_gap = geom_gap
  _m.geom_group = geom_group
  _m.geom_margin = geom_margin
  _m.geom_matid = geom_matid
  _m.geom_pair_type_count = geom_pair_type_count
  _m.geom_plugin_index = geom_plugin_index
  _m.geom_pos = geom_pos
  _m.geom_priority = geom_priority
  _m.geom_quat = geom_quat
  _m.geom_rbound = geom_rbound
  _m.geom_rgba = geom_rgba
  _m.geom_size = geom_size
  _m.geom_solimp = geom_solimp
  _m.geom_solmix = geom_solmix
  _m.geom_solref = geom_solref
  _m.geom_type = geom_type
  _m.has_sdf_geom = has_sdf_geom
  _m.hfield_adr = hfield_adr
  _m.hfield_data = hfield_data
  _m.hfield_ncol = hfield_ncol
  _m.hfield_nrow = hfield_nrow
  _m.hfield_size = hfield_size
  _m.jnt_actfrclimited = jnt_actfrclimited
  _m.jnt_actfrcrange = jnt_actfrcrange
  _m.jnt_actgravcomp = jnt_actgravcomp
  _m.jnt_axis = jnt_axis
  _m.jnt_bodyid = jnt_bodyid
  _m.jnt_dofadr = jnt_dofadr
  _m.jnt_limited_ball_adr = jnt_limited_ball_adr
  _m.jnt_limited_slide_hinge_adr = jnt_limited_slide_hinge_adr
  _m.jnt_margin = jnt_margin
  _m.jnt_pos = jnt_pos
  _m.jnt_qposadr = jnt_qposadr
  _m.jnt_range = jnt_range
  _m.jnt_solimp = jnt_solimp
  _m.jnt_solref = jnt_solref
  _m.jnt_stiffness = jnt_stiffness
  _m.jnt_type = jnt_type
  _m.light_bodyid = light_bodyid
  _m.light_dir = light_dir
  _m.light_dir0 = light_dir0
  _m.light_mode = light_mode
  _m.light_pos = light_pos
  _m.light_pos0 = light_pos0
  _m.light_poscom0 = light_poscom0
  _m.light_targetbodyid = light_targetbodyid
  _m.mapM2M = mapM2M
  _m.mat_rgba = mat_rgba
  _m.mesh_face = mesh_face
  _m.mesh_faceadr = mesh_faceadr
  _m.mesh_graph = mesh_graph
  _m.mesh_graphadr = mesh_graphadr
  _m.mesh_normal = mesh_normal
  _m.mesh_normaladr = mesh_normaladr
  _m.mesh_polyadr = mesh_polyadr
  _m.mesh_polymap = mesh_polymap
  _m.mesh_polymapadr = mesh_polymapadr
  _m.mesh_polymapnum = mesh_polymapnum
  _m.mesh_polynormal = mesh_polynormal
  _m.mesh_polynum = mesh_polynum
  _m.mesh_polyvert = mesh_polyvert
  _m.mesh_polyvertadr = mesh_polyvertadr
  _m.mesh_polyvertnum = mesh_polyvertnum
  _m.mesh_quat = mesh_quat
  _m.mesh_vert = mesh_vert
  _m.mesh_vertadr = mesh_vertadr
  _m.mesh_vertnum = mesh_vertnum
  _m.nC = nC
  _m.na = na
  _m.nacttrnbody = nacttrnbody
  _m.nbody = nbody
  _m.ncam = ncam
  _m.neq = neq
  _m.nflex = nflex
  _m.nflexedge = nflexedge
  _m.nflexelem = nflexelem
  _m.nflexvert = nflexvert
  _m.ngeom = ngeom
  _m.ngravcomp = ngravcomp
  _m.njnt = njnt
  _m.nlight = nlight
  _m.nmaxcondim = nmaxcondim
  _m.nmaxmeshdeg = nmaxmeshdeg
  _m.nmaxpolygon = nmaxpolygon
  _m.nmaxpyramid = nmaxpyramid
  _m.nmeshface = nmeshface
  _m.nrangefinder = nrangefinder
  _m.nsensorcollision = nsensorcollision
  _m.nsensorcontact = nsensorcontact
  _m.nsensortaxel = nsensortaxel
  _m.nsite = nsite
  _m.ntendon = ntendon
  _m.nu = nu
  _m.nv = nv
  _m.nv_pad = nv_pad
  _m.nwrap = nwrap
  _m.nxn_geom_pair_filtered = nxn_geom_pair_filtered
  _m.nxn_pairid = nxn_pairid
  _m.nxn_pairid_filtered = nxn_pairid_filtered
  _m.oct_aabb = oct_aabb
  _m.oct_child = oct_child
  _m.oct_coeff = oct_coeff
  _m.opt.broadphase = opt__broadphase
  _m.opt.broadphase_filter = opt__broadphase_filter
  _m.opt.ccd_iterations = opt__ccd_iterations
  _m.opt.ccd_tolerance = opt__ccd_tolerance
  _m.opt.cone = opt__cone
  _m.opt.contact_sensor_maxmatch = opt__contact_sensor_maxmatch
  _m.opt.density = opt__density
  _m.opt.disableflags = opt__disableflags
  _m.opt.enableflags = opt__enableflags
  _m.opt.graph_conditional = opt__graph_conditional
  _m.opt.gravity = opt__gravity
  _m.opt.has_fluid = opt__has_fluid
  _m.opt.impratio_invsqrt = opt__impratio_invsqrt
  _m.opt.is_sparse = opt__is_sparse
  _m.opt.iterations = opt__iterations
  _m.opt.ls_iterations = opt__ls_iterations
  _m.opt.ls_parallel = opt__ls_parallel
  _m.opt.ls_parallel_min_step = opt__ls_parallel_min_step
  _m.opt.ls_tolerance = opt__ls_tolerance
  _m.opt.magnetic = opt__magnetic
  _m.opt.run_collision_detection = opt__run_collision_detection
  _m.opt.sdf_initpoints = opt__sdf_initpoints
  _m.opt.sdf_iterations = opt__sdf_iterations
  _m.opt.solver = opt__solver
  _m.opt.timestep = opt__timestep
  _m.opt.tolerance = opt__tolerance
  _m.opt.viscosity = opt__viscosity
  _m.opt.wind = opt__wind
  _m.pair_dim = pair_dim
  _m.pair_friction = pair_friction
  _m.pair_gap = pair_gap
  _m.pair_margin = pair_margin
  _m.pair_solimp = pair_solimp
  _m.pair_solref = pair_solref
  _m.pair_solreffriction = pair_solreffriction
  _m.plugin = plugin
  _m.plugin_attr = plugin_attr
  _m.qLD_updates = qLD_updates
  _m.qM_fullm_i = qM_fullm_i
  _m.qM_fullm_j = qM_fullm_j
  _m.qM_madr_ij = qM_madr_ij
  _m.qM_mulm_i = qM_mulm_i
  _m.qM_mulm_j = qM_mulm_j
  _m.qM_tiles = qM_tiles
  _m.qpos0 = qpos0
  _m.qpos_spring = qpos_spring
  _m.rangefinder_sensor_adr = rangefinder_sensor_adr
  _m.sensor_acc_adr = sensor_acc_adr
  _m.sensor_adr = sensor_adr
  _m.sensor_adr_to_contact_adr = sensor_adr_to_contact_adr
  _m.sensor_contact_adr = sensor_contact_adr
  _m.sensor_cutoff = sensor_cutoff
  _m.sensor_datatype = sensor_datatype
  _m.sensor_dim = sensor_dim
  _m.sensor_e_kinetic = sensor_e_kinetic
  _m.sensor_e_potential = sensor_e_potential
  _m.sensor_intprm = sensor_intprm
  _m.sensor_limitfrc_adr = sensor_limitfrc_adr
  _m.sensor_limitpos_adr = sensor_limitpos_adr
  _m.sensor_limitvel_adr = sensor_limitvel_adr
  _m.sensor_objid = sensor_objid
  _m.sensor_objtype = sensor_objtype
  _m.sensor_pos_adr = sensor_pos_adr
  _m.sensor_rangefinder_adr = sensor_rangefinder_adr
  _m.sensor_rangefinder_bodyid = sensor_rangefinder_bodyid
  _m.sensor_refid = sensor_refid
  _m.sensor_reftype = sensor_reftype
  _m.sensor_rne_postconstraint = sensor_rne_postconstraint
  _m.sensor_subtree_vel = sensor_subtree_vel
  _m.sensor_tendonactfrc_adr = sensor_tendonactfrc_adr
  _m.sensor_touch_adr = sensor_touch_adr
  _m.sensor_type = sensor_type
  _m.sensor_vel_adr = sensor_vel_adr
  _m.site_bodyid = site_bodyid
  _m.site_pos = site_pos
  _m.site_quat = site_quat
  _m.site_size = site_size
  _m.site_type = site_type
  _m.stat.meaninertia = stat__meaninertia
  _m.taxel_sensorid = taxel_sensorid
  _m.taxel_vertadr = taxel_vertadr
  _m.tendon_actfrclimited = tendon_actfrclimited
  _m.tendon_actfrcrange = tendon_actfrcrange
  _m.tendon_adr = tendon_adr
  _m.tendon_armature = tendon_armature
  _m.tendon_damping = tendon_damping
  _m.tendon_frictionloss = tendon_frictionloss
  _m.tendon_geom_adr = tendon_geom_adr
  _m.tendon_invweight0 = tendon_invweight0
  _m.tendon_jnt_adr = tendon_jnt_adr
  _m.tendon_length0 = tendon_length0
  _m.tendon_lengthspring = tendon_lengthspring
  _m.tendon_limited_adr = tendon_limited_adr
  _m.tendon_margin = tendon_margin
  _m.tendon_num = tendon_num
  _m.tendon_range = tendon_range
  _m.tendon_site_pair_adr = tendon_site_pair_adr
  _m.tendon_solimp_fri = tendon_solimp_fri
  _m.tendon_solimp_lim = tendon_solimp_lim
  _m.tendon_solref_fri = tendon_solref_fri
  _m.tendon_solref_lim = tendon_solref_lim
  _m.tendon_stiffness = tendon_stiffness
  _m.wrap_geom_adr = wrap_geom_adr
  _m.wrap_jnt_adr = wrap_jnt_adr
  _m.wrap_objid = wrap_objid
  _m.wrap_prm = wrap_prm
  _m.wrap_pulley_scale = wrap_pulley_scale
  _m.wrap_site_pair_adr = wrap_site_pair_adr
  _m.wrap_type = wrap_type
  _d.act = act
  _d.act_dot = act_dot
  _d.actuator_force = actuator_force
  _d.actuator_length = actuator_length
  _d.actuator_moment = actuator_moment
  _d.actuator_velocity = actuator_velocity
  _d.cacc = cacc
  _d.cam_xmat = cam_xmat
  _d.cam_xpos = cam_xpos
  _d.cdof = cdof
  _d.cdof_dot = cdof_dot
  _d.cfrc_ext = cfrc_ext
  _d.cfrc_int = cfrc_int
  _d.cinert = cinert
  _d.collision_pair = collision_pair
  _d.collision_pairid = collision_pairid
  _d.collision_worldid = collision_worldid
  _d.contact.dim = contact__dim
  _d.contact.dist = contact__dist
  _d.contact.efc_address = contact__efc_address
  _d.contact.frame = contact__frame
  _d.contact.friction = contact__friction
  _d.contact.geom = contact__geom
  _d.contact.geomcollisionid = contact__geomcollisionid
  _d.contact.includemargin = contact__includemargin
  _d.contact.pos = contact__pos
  _d.contact.solimp = contact__solimp
  _d.contact.solref = contact__solref
  _d.contact.solreffriction = contact__solreffriction
  _d.contact.type = contact__type
  _d.contact.worldid = contact__worldid
  _d.crb = crb
  _d.ctrl = ctrl
  _d.cvel = cvel
  _d.efc.D = efc__D
  _d.efc.J = efc__J
  _d.efc.Jaref = efc__Jaref
  _d.efc.Ma = efc__Ma
  _d.efc.Mgrad = efc__Mgrad
  _d.efc.alpha = efc__alpha
  _d.efc.aref = efc__aref
  _d.efc.beta = efc__beta
  _d.efc.cost = efc__cost
  _d.efc.done = efc__done
  _d.efc.force = efc__force
  _d.efc.frictionloss = efc__frictionloss
  _d.efc.gauss = efc__gauss
  _d.efc.grad = efc__grad
  _d.efc.grad_dot = efc__grad_dot
  _d.efc.id = efc__id
  _d.efc.jv = efc__jv
  _d.efc.margin = efc__margin
  _d.efc.mv = efc__mv
  _d.efc.pos = efc__pos
  _d.efc.prev_Mgrad = efc__prev_Mgrad
  _d.efc.prev_cost = efc__prev_cost
  _d.efc.prev_grad = efc__prev_grad
  _d.efc.quad = efc__quad
  _d.efc.quad_gauss = efc__quad_gauss
  _d.efc.search = efc__search
  _d.efc.search_dot = efc__search_dot
  _d.efc.state = efc__state
  _d.efc.type = efc__type
  _d.efc.vel = efc__vel
  _d.energy = energy
  _d.eq_active = eq_active
  _d.flexedge_J = flexedge_J
  _d.flexedge_length = flexedge_length
  _d.flexedge_velocity = flexedge_velocity
  _d.flexvert_xpos = flexvert_xpos
  _d.geom_xmat = geom_xmat
  _d.geom_xpos = geom_xpos
  _d.light_xdir = light_xdir
  _d.light_xpos = light_xpos
  _d.mocap_pos = mocap_pos
  _d.mocap_quat = mocap_quat
  _d.nacon = nacon
  _d.naconmax = naconmax
  _d.ncollision = ncollision
  _d.ne = ne
  _d.ne_connect = ne_connect
  _d.ne_flex = ne_flex
  _d.ne_jnt = ne_jnt
  _d.ne_ten = ne_ten
  _d.ne_weld = ne_weld
  _d.nefc = nefc
  _d.nf = nf
  _d.njmax = njmax
  _d.nl = nl
  _d.nsolving = nsolving
  _d.qLD = qLD
  _d.qLDiagInv = qLDiagInv
  _d.qM = qM
  _d.qacc = qacc
  _d.qacc_smooth = qacc_smooth
  _d.qacc_warmstart = qacc_warmstart
  _d.qfrc_actuator = qfrc_actuator
  _d.qfrc_applied = qfrc_applied
  _d.qfrc_bias = qfrc_bias
  _d.qfrc_constraint = qfrc_constraint
  _d.qfrc_damper = qfrc_damper
  _d.qfrc_fluid = qfrc_fluid
  _d.qfrc_gravcomp = qfrc_gravcomp
  _d.qfrc_passive = qfrc_passive
  _d.qfrc_smooth = qfrc_smooth
  _d.qfrc_spring = qfrc_spring
  _d.qpos = qpos
  _d.qvel = qvel
  _d.sensordata = sensordata
  _d.site_xmat = site_xmat
  _d.site_xpos = site_xpos
  _d.solver_niter = solver_niter
  _d.subtree_angmom = subtree_angmom
  _d.subtree_bodyvel = subtree_bodyvel
  _d.subtree_com = subtree_com
  _d.subtree_linvel = subtree_linvel
  _d.ten_J = ten_J
  _d.ten_length = ten_length
  _d.ten_velocity = ten_velocity
  _d.ten_wrapadr = ten_wrapadr
  _d.ten_wrapnum = ten_wrapnum
  _d.time = time
  _d.wrap_obj = wrap_obj
  _d.wrap_xpos = wrap_xpos
  _d.xanchor = xanchor
  _d.xaxis = xaxis
  _d.xfrc_applied = xfrc_applied
  _d.ximat = ximat
  _d.xipos = xipos
  _d.xmat = xmat
  _d.xpos = xpos
  _d.xquat = xquat
  _d.nworld = nworld
  mjwarp.forward(_m, _d)


def _forward_jax_impl(m: types.Model, d: types.Data):
  output_dims = {
      'act': d.act.shape,
      'act_dot': d.act_dot.shape,
      'actuator_force': d.actuator_force.shape,
      'actuator_length': d.actuator_length.shape,
      'actuator_moment': d._impl.actuator_moment.shape,
      'actuator_velocity': d._impl.actuator_velocity.shape,
      'cacc': d._impl.cacc.shape,
      'cam_xmat': d.cam_xmat.shape,
      'cam_xpos': d.cam_xpos.shape,
      'cdof': d.cdof.shape,
      'cdof_dot': d.cdof_dot.shape,
      'cfrc_ext': d._impl.cfrc_ext.shape,
      'cfrc_int': d._impl.cfrc_int.shape,
      'cinert': d._impl.cinert.shape,
      'collision_pair': d._impl.collision_pair.shape,
      'collision_pairid': d._impl.collision_pairid.shape,
      'collision_worldid': d._impl.collision_worldid.shape,
      'crb': d._impl.crb.shape,
      'ctrl': d.ctrl.shape,
      'cvel': d.cvel.shape,
      'energy': d._impl.energy.shape,
      'eq_active': d.eq_active.shape,
      'flexedge_J': d._impl.flexedge_J.shape,
      'flexedge_length': d._impl.flexedge_length.shape,
      'flexedge_velocity': d._impl.flexedge_velocity.shape,
      'flexvert_xpos': d._impl.flexvert_xpos.shape,
      'geom_xmat': d.geom_xmat.shape,
      'geom_xpos': d.geom_xpos.shape,
      'light_xdir': d._impl.light_xdir.shape,
      'light_xpos': d._impl.light_xpos.shape,
      'mocap_pos': d.mocap_pos.shape,
      'mocap_quat': d.mocap_quat.shape,
      'nacon': d._impl.nacon.shape,
      'ncollision': d._impl.ncollision.shape,
      'ne': d._impl.ne.shape,
      'ne_connect': d._impl.ne_connect.shape,
      'ne_flex': d._impl.ne_flex.shape,
      'ne_jnt': d._impl.ne_jnt.shape,
      'ne_ten': d._impl.ne_ten.shape,
      'ne_weld': d._impl.ne_weld.shape,
      'nefc': d._impl.nefc.shape,
      'nf': d._impl.nf.shape,
      'nl': d._impl.nl.shape,
      'nsolving': d._impl.nsolving.shape,
      'qLD': d._impl.qLD.shape,
      'qLDiagInv': d._impl.qLDiagInv.shape,
      'qM': d._impl.qM.shape,
      'qacc': d.qacc.shape,
      'qacc_smooth': d.qacc_smooth.shape,
      'qacc_warmstart': d.qacc_warmstart.shape,
      'qfrc_actuator': d.qfrc_actuator.shape,
      'qfrc_applied': d.qfrc_applied.shape,
      'qfrc_bias': d.qfrc_bias.shape,
      'qfrc_constraint': d.qfrc_constraint.shape,
      'qfrc_damper': d._impl.qfrc_damper.shape,
      'qfrc_fluid': d.qfrc_fluid.shape,
      'qfrc_gravcomp': d.qfrc_gravcomp.shape,
      'qfrc_passive': d.qfrc_passive.shape,
      'qfrc_smooth': d.qfrc_smooth.shape,
      'qfrc_spring': d._impl.qfrc_spring.shape,
      'qpos': d.qpos.shape,
      'qvel': d.qvel.shape,
      'sensordata': d.sensordata.shape,
      'site_xmat': d.site_xmat.shape,
      'site_xpos': d.site_xpos.shape,
      'solver_niter': d._impl.solver_niter.shape,
      'subtree_angmom': d._impl.subtree_angmom.shape,
      'subtree_bodyvel': d._impl.subtree_bodyvel.shape,
      'subtree_com': d.subtree_com.shape,
      'subtree_linvel': d._impl.subtree_linvel.shape,
      'ten_J': d._impl.ten_J.shape,
      'ten_length': d.ten_length.shape,
      'ten_velocity': d._impl.ten_velocity.shape,
      'ten_wrapadr': d._impl.ten_wrapadr.shape,
      'ten_wrapnum': d._impl.ten_wrapnum.shape,
      'time': d.time.shape,
      'wrap_obj': d._impl.wrap_obj.shape,
      'wrap_xpos': d._impl.wrap_xpos.shape,
      'xanchor': d.xanchor.shape,
      'xaxis': d.xaxis.shape,
      'xfrc_applied': d.xfrc_applied.shape,
      'ximat': d.ximat.shape,
      'xipos': d.xipos.shape,
      'xmat': d.xmat.shape,
      'xpos': d.xpos.shape,
      'xquat': d.xquat.shape,
      'contact__dim': d._impl.contact__dim.shape,
      'contact__dist': d._impl.contact__dist.shape,
      'contact__efc_address': d._impl.contact__efc_address.shape,
      'contact__frame': d._impl.contact__frame.shape,
      'contact__friction': d._impl.contact__friction.shape,
      'contact__geom': d._impl.contact__geom.shape,
      'contact__geomcollisionid': d._impl.contact__geomcollisionid.shape,
      'contact__includemargin': d._impl.contact__includemargin.shape,
      'contact__pos': d._impl.contact__pos.shape,
      'contact__solimp': d._impl.contact__solimp.shape,
      'contact__solref': d._impl.contact__solref.shape,
      'contact__solreffriction': d._impl.contact__solreffriction.shape,
      'contact__type': d._impl.contact__type.shape,
      'contact__worldid': d._impl.contact__worldid.shape,
      'efc__D': d._impl.efc__D.shape,
      'efc__J': d._impl.efc__J.shape,
      'efc__Jaref': d._impl.efc__Jaref.shape,
      'efc__Ma': d._impl.efc__Ma.shape,
      'efc__Mgrad': d._impl.efc__Mgrad.shape,
      'efc__alpha': d._impl.efc__alpha.shape,
      'efc__aref': d._impl.efc__aref.shape,
      'efc__beta': d._impl.efc__beta.shape,
      'efc__cost': d._impl.efc__cost.shape,
      'efc__done': d._impl.efc__done.shape,
      'efc__force': d._impl.efc__force.shape,
      'efc__frictionloss': d._impl.efc__frictionloss.shape,
      'efc__gauss': d._impl.efc__gauss.shape,
      'efc__grad': d._impl.efc__grad.shape,
      'efc__grad_dot': d._impl.efc__grad_dot.shape,
      'efc__id': d._impl.efc__id.shape,
      'efc__jv': d._impl.efc__jv.shape,
      'efc__margin': d._impl.efc__margin.shape,
      'efc__mv': d._impl.efc__mv.shape,
      'efc__pos': d._impl.efc__pos.shape,
      'efc__prev_Mgrad': d._impl.efc__prev_Mgrad.shape,
      'efc__prev_cost': d._impl.efc__prev_cost.shape,
      'efc__prev_grad': d._impl.efc__prev_grad.shape,
      'efc__quad': d._impl.efc__quad.shape,
      'efc__quad_gauss': d._impl.efc__quad_gauss.shape,
      'efc__search': d._impl.efc__search.shape,
      'efc__search_dot': d._impl.efc__search_dot.shape,
      'efc__state': d._impl.efc__state.shape,
      'efc__type': d._impl.efc__type.shape,
      'efc__vel': d._impl.efc__vel.shape,
  }
  jf = ffi.jax_callable_variadic_tuple(
      _forward_shim,
      num_outputs=130,
      output_dims=output_dims,
      vmap_method=None,
      in_out_argnames={
          'act',
          'act_dot',
          'actuator_force',
          'actuator_length',
          'actuator_moment',
          'actuator_velocity',
          'cacc',
          'cam_xmat',
          'cam_xpos',
          'cdof',
          'cdof_dot',
          'cfrc_ext',
          'cfrc_int',
          'cinert',
          'collision_pair',
          'collision_pairid',
          'collision_worldid',
          'crb',
          'ctrl',
          'cvel',
          'energy',
          'eq_active',
          'flexedge_J',
          'flexedge_length',
          'flexedge_velocity',
          'flexvert_xpos',
          'geom_xmat',
          'geom_xpos',
          'light_xdir',
          'light_xpos',
          'mocap_pos',
          'mocap_quat',
          'nacon',
          'ncollision',
          'ne',
          'ne_connect',
          'ne_flex',
          'ne_jnt',
          'ne_ten',
          'ne_weld',
          'nefc',
          'nf',
          'nl',
          'nsolving',
          'qLD',
          'qLDiagInv',
          'qM',
          'qacc',
          'qacc_smooth',
          'qacc_warmstart',
          'qfrc_actuator',
          'qfrc_applied',
          'qfrc_bias',
          'qfrc_constraint',
          'qfrc_damper',
          'qfrc_fluid',
          'qfrc_gravcomp',
          'qfrc_passive',
          'qfrc_smooth',
          'qfrc_spring',
          'qpos',
          'qvel',
          'sensordata',
          'site_xmat',
          'site_xpos',
          'solver_niter',
          'subtree_angmom',
          'subtree_bodyvel',
          'subtree_com',
          'subtree_linvel',
          'ten_J',
          'ten_length',
          'ten_velocity',
          'ten_wrapadr',
          'ten_wrapnum',
          'time',
          'wrap_obj',
          'wrap_xpos',
          'xanchor',
          'xaxis',
          'xfrc_applied',
          'ximat',
          'xipos',
          'xmat',
          'xpos',
          'xquat',
          'contact__dim',
          'contact__dist',
          'contact__efc_address',
          'contact__frame',
          'contact__friction',
          'contact__geom',
          'contact__geomcollisionid',
          'contact__includemargin',
          'contact__pos',
          'contact__solimp',
          'contact__solref',
          'contact__solreffriction',
          'contact__type',
          'contact__worldid',
          'efc__D',
          'efc__J',
          'efc__Jaref',
          'efc__Ma',
          'efc__Mgrad',
          'efc__alpha',
          'efc__aref',
          'efc__beta',
          'efc__cost',
          'efc__done',
          'efc__force',
          'efc__frictionloss',
          'efc__gauss',
          'efc__grad',
          'efc__grad_dot',
          'efc__id',
          'efc__jv',
          'efc__margin',
          'efc__mv',
          'efc__pos',
          'efc__prev_Mgrad',
          'efc__prev_cost',
          'efc__prev_grad',
          'efc__quad',
          'efc__quad_gauss',
          'efc__search',
          'efc__search_dot',
          'efc__state',
          'efc__type',
          'efc__vel',
      },
  )
  out = jf(
      d.qpos.shape[0],
      m._impl.M_rowadr,
      m._impl.M_rownnz,
      m.actuator_acc0,
      m.actuator_actadr,
      m.actuator_actearly,
      m.actuator_actlimited,
      m.actuator_actnum,
      m.actuator_actrange,
      m.actuator_biasprm,
      m.actuator_biastype,
      m.actuator_cranklength,
      m.actuator_ctrllimited,
      m.actuator_ctrlrange,
      m.actuator_dynprm,
      m.actuator_dyntype,
      m.actuator_forcelimited,
      m.actuator_forcerange,
      m.actuator_gainprm,
      m.actuator_gaintype,
      m.actuator_gear,
      m.actuator_lengthrange,
      m.actuator_trnid,
      m.actuator_trntype,
      m._impl.actuator_trntype_body_adr,
      m._impl.block_dim,
      m.body_dofadr,
      m.body_dofnum,
      m._impl.body_fluid_ellipsoid,
      m.body_geomadr,
      m.body_geomnum,
      m.body_gravcomp,
      m.body_inertia,
      m.body_invweight0,
      m.body_ipos,
      m.body_iquat,
      m.body_jntadr,
      m.body_jntnum,
      m.body_mass,
      m.body_mocapid,
      m.body_parentid,
      m.body_pos,
      m.body_quat,
      m.body_rootid,
      m.body_subtreemass,
      m._impl.body_tree,
      m.body_weldid,
      m.cam_bodyid,
      m.cam_fovy,
      m.cam_intrinsic,
      m.cam_mat0,
      m.cam_mode,
      m.cam_pos,
      m.cam_pos0,
      m.cam_poscom0,
      m.cam_quat,
      m.cam_resolution,
      m.cam_sensorsize,
      m.cam_targetbodyid,
      m.dof_Madr,
      m.dof_armature,
      m.dof_bodyid,
      m.dof_damping,
      m.dof_frictionloss,
      m.dof_invweight0,
      m.dof_jntid,
      m.dof_parentid,
      m.dof_solimp,
      m.dof_solref,
      m._impl.dof_tri_col,
      m._impl.dof_tri_row,
      m._impl.eq_connect_adr,
      m.eq_data,
      m._impl.eq_flex_adr,
      m._impl.eq_jnt_adr,
      m.eq_obj1id,
      m.eq_obj2id,
      m.eq_objtype,
      m.eq_solimp,
      m.eq_solref,
      m._impl.eq_ten_adr,
      m._impl.eq_wld_adr,
      m._impl.flex_bending,
      m._impl.flex_damping,
      m._impl.flex_dim,
      m._impl.flex_edge,
      m._impl.flex_edgeadr,
      m._impl.flex_edgeflap,
      m._impl.flex_edgenum,
      m._impl.flex_elem,
      m._impl.flex_elemadr,
      m._impl.flex_elemedge,
      m._impl.flex_elemedgeadr,
      m._impl.flex_elemnum,
      m._impl.flex_stiffness,
      m._impl.flex_vertadr,
      m._impl.flex_vertbodyid,
      m._impl.flexedge_invweight0,
      m._impl.flexedge_length0,
      m.geom_aabb,
      m.geom_bodyid,
      m.geom_condim,
      m.geom_dataid,
      m.geom_fluid,
      m.geom_friction,
      m.geom_gap,
      m.geom_group,
      m.geom_margin,
      m.geom_matid,
      m._impl.geom_pair_type_count,
      m._impl.geom_plugin_index,
      m.geom_pos,
      m.geom_priority,
      m.geom_quat,
      m.geom_rbound,
      m.geom_rgba,
      m.geom_size,
      m.geom_solimp,
      m.geom_solmix,
      m.geom_solref,
      m.geom_type,
      m._impl.has_sdf_geom,
      m.hfield_adr,
      m.hfield_data,
      m.hfield_ncol,
      m.hfield_nrow,
      m.hfield_size,
      m.jnt_actfrclimited,
      m.jnt_actfrcrange,
      m.jnt_actgravcomp,
      m.jnt_axis,
      m.jnt_bodyid,
      m.jnt_dofadr,
      m._impl.jnt_limited_ball_adr,
      m._impl.jnt_limited_slide_hinge_adr,
      m.jnt_margin,
      m.jnt_pos,
      m.jnt_qposadr,
      m.jnt_range,
      m.jnt_solimp,
      m.jnt_solref,
      m.jnt_stiffness,
      m.jnt_type,
      m._impl.light_bodyid,
      m.light_dir,
      m.light_dir0,
      m.light_mode,
      m.light_pos,
      m.light_pos0,
      m.light_poscom0,
      m._impl.light_targetbodyid,
      m._impl.mapM2M,
      m.mat_rgba,
      m.mesh_face,
      m.mesh_faceadr,
      m.mesh_graph,
      m.mesh_graphadr,
      m.mesh_normal,
      m.mesh_normaladr,
      m._impl.mesh_polyadr,
      m._impl.mesh_polymap,
      m._impl.mesh_polymapadr,
      m._impl.mesh_polymapnum,
      m._impl.mesh_polynormal,
      m._impl.mesh_polynum,
      m._impl.mesh_polyvert,
      m._impl.mesh_polyvertadr,
      m._impl.mesh_polyvertnum,
      m.mesh_quat,
      m.mesh_vert,
      m.mesh_vertadr,
      m.mesh_vertnum,
      m.nC,
      m.na,
      m._impl.nacttrnbody,
      m.nbody,
      m.ncam,
      m.neq,
      m._impl.nflex,
      m._impl.nflexedge,
      m._impl.nflexelem,
      m._impl.nflexvert,
      m.ngeom,
      m.ngravcomp,
      m.njnt,
      m.nlight,
      m._impl.nmaxcondim,
      m._impl.nmaxmeshdeg,
      m._impl.nmaxpolygon,
      m._impl.nmaxpyramid,
      m.nmeshface,
      m._impl.nrangefinder,
      m._impl.nsensorcollision,
      m._impl.nsensorcontact,
      m._impl.nsensortaxel,
      m.nsite,
      m.ntendon,
      m.nu,
      m.nv,
      m._impl.nv_pad,
      m.nwrap,
      m._impl.nxn_geom_pair_filtered,
      m._impl.nxn_pairid,
      m._impl.nxn_pairid_filtered,
      m._impl.oct_aabb,
      m._impl.oct_child,
      m._impl.oct_coeff,
      m.pair_dim,
      m.pair_friction,
      m.pair_gap,
      m.pair_margin,
      m.pair_solimp,
      m.pair_solref,
      m.pair_solreffriction,
      m._impl.plugin,
      m._impl.plugin_attr,
      m._impl.qLD_updates,
      m._impl.qM_fullm_i,
      m._impl.qM_fullm_j,
      m._impl.qM_madr_ij,
      m._impl.qM_mulm_i,
      m._impl.qM_mulm_j,
      m._impl.qM_tiles,
      m.qpos0,
      m.qpos_spring,
      m._impl.rangefinder_sensor_adr,
      m._impl.sensor_acc_adr,
      m.sensor_adr,
      m._impl.sensor_adr_to_contact_adr,
      m._impl.sensor_contact_adr,
      m.sensor_cutoff,
      m.sensor_datatype,
      m.sensor_dim,
      m._impl.sensor_e_kinetic,
      m._impl.sensor_e_potential,
      m.sensor_intprm,
      m._impl.sensor_limitfrc_adr,
      m._impl.sensor_limitpos_adr,
      m._impl.sensor_limitvel_adr,
      m.sensor_objid,
      m.sensor_objtype,
      m._impl.sensor_pos_adr,
      m._impl.sensor_rangefinder_adr,
      m._impl.sensor_rangefinder_bodyid,
      m.sensor_refid,
      m.sensor_reftype,
      m._impl.sensor_rne_postconstraint,
      m._impl.sensor_subtree_vel,
      m._impl.sensor_tendonactfrc_adr,
      m._impl.sensor_touch_adr,
      m.sensor_type,
      m._impl.sensor_vel_adr,
      m.site_bodyid,
      m.site_pos,
      m.site_quat,
      m.site_size,
      m.site_type,
      m._impl.taxel_sensorid,
      m._impl.taxel_vertadr,
      m.tendon_actfrclimited,
      m.tendon_actfrcrange,
      m.tendon_adr,
      m.tendon_armature,
      m.tendon_damping,
      m.tendon_frictionloss,
      m._impl.tendon_geom_adr,
      m.tendon_invweight0,
      m._impl.tendon_jnt_adr,
      m.tendon_length0,
      m.tendon_lengthspring,
      m._impl.tendon_limited_adr,
      m.tendon_margin,
      m.tendon_num,
      m.tendon_range,
      m._impl.tendon_site_pair_adr,
      m.tendon_solimp_fri,
      m.tendon_solimp_lim,
      m.tendon_solref_fri,
      m.tendon_solref_lim,
      m.tendon_stiffness,
      m._impl.wrap_geom_adr,
      m._impl.wrap_jnt_adr,
      m.wrap_objid,
      m.wrap_prm,
      m._impl.wrap_pulley_scale,
      m._impl.wrap_site_pair_adr,
      m.wrap_type,
      m.opt._impl.broadphase,
      m.opt._impl.broadphase_filter,
      m.opt._impl.ccd_iterations,
      m.opt._impl.ccd_tolerance,
      m.opt.cone,
      m.opt._impl.contact_sensor_maxmatch,
      m.opt.density,
      m.opt.disableflags,
      m.opt.enableflags,
      m.opt._impl.graph_conditional,
      m.opt.gravity,
      m.opt._impl.has_fluid,
      m.opt._impl.impratio_invsqrt,
      m.opt._impl.is_sparse,
      m.opt.iterations,
      m.opt.ls_iterations,
      m.opt._impl.ls_parallel,
      m.opt._impl.ls_parallel_min_step,
      m.opt.ls_tolerance,
      m.opt.magnetic,
      m.opt._impl.run_collision_detection,
      m.opt._impl.sdf_initpoints,
      m.opt._impl.sdf_iterations,
      m.opt.solver,
      m.opt.timestep,
      m.opt.tolerance,
      m.opt.viscosity,
      m.opt.wind,
      m.stat.meaninertia,
      d._impl.naconmax,
      d._impl.njmax,
      d.act,
      d.act_dot,
      d.actuator_force,
      d.actuator_length,
      d._impl.actuator_moment,
      d._impl.actuator_velocity,
      d._impl.cacc,
      d.cam_xmat,
      d.cam_xpos,
      d.cdof,
      d.cdof_dot,
      d._impl.cfrc_ext,
      d._impl.cfrc_int,
      d._impl.cinert,
      d._impl.collision_pair,
      d._impl.collision_pairid,
      d._impl.collision_worldid,
      d._impl.crb,
      d.ctrl,
      d.cvel,
      d._impl.energy,
      d.eq_active,
      d._impl.flexedge_J,
      d._impl.flexedge_length,
      d._impl.flexedge_velocity,
      d._impl.flexvert_xpos,
      d.geom_xmat,
      d.geom_xpos,
      d._impl.light_xdir,
      d._impl.light_xpos,
      d.mocap_pos,
      d.mocap_quat,
      d._impl.nacon,
      d._impl.ncollision,
      d._impl.ne,
      d._impl.ne_connect,
      d._impl.ne_flex,
      d._impl.ne_jnt,
      d._impl.ne_ten,
      d._impl.ne_weld,
      d._impl.nefc,
      d._impl.nf,
      d._impl.nl,
      d._impl.nsolving,
      d._impl.qLD,
      d._impl.qLDiagInv,
      d._impl.qM,
      d.qacc,
      d.qacc_smooth,
      d.qacc_warmstart,
      d.qfrc_actuator,
      d.qfrc_applied,
      d.qfrc_bias,
      d.qfrc_constraint,
      d._impl.qfrc_damper,
      d.qfrc_fluid,
      d.qfrc_gravcomp,
      d.qfrc_passive,
      d.qfrc_smooth,
      d._impl.qfrc_spring,
      d.qpos,
      d.qvel,
      d.sensordata,
      d.site_xmat,
      d.site_xpos,
      d._impl.solver_niter,
      d._impl.subtree_angmom,
      d._impl.subtree_bodyvel,
      d.subtree_com,
      d._impl.subtree_linvel,
      d._impl.ten_J,
      d.ten_length,
      d._impl.ten_velocity,
      d._impl.ten_wrapadr,
      d._impl.ten_wrapnum,
      d.time,
      d._impl.wrap_obj,
      d._impl.wrap_xpos,
      d.xanchor,
      d.xaxis,
      d.xfrc_applied,
      d.ximat,
      d.xipos,
      d.xmat,
      d.xpos,
      d.xquat,
      d._impl.contact__dim,
      d._impl.contact__dist,
      d._impl.contact__efc_address,
      d._impl.contact__frame,
      d._impl.contact__friction,
      d._impl.contact__geom,
      d._impl.contact__geomcollisionid,
      d._impl.contact__includemargin,
      d._impl.contact__pos,
      d._impl.contact__solimp,
      d._impl.contact__solref,
      d._impl.contact__solreffriction,
      d._impl.contact__type,
      d._impl.contact__worldid,
      d._impl.efc__D,
      d._impl.efc__J,
      d._impl.efc__Jaref,
      d._impl.efc__Ma,
      d._impl.efc__Mgrad,
      d._impl.efc__alpha,
      d._impl.efc__aref,
      d._impl.efc__beta,
      d._impl.efc__cost,
      d._impl.efc__done,
      d._impl.efc__force,
      d._impl.efc__frictionloss,
      d._impl.efc__gauss,
      d._impl.efc__grad,
      d._impl.efc__grad_dot,
      d._impl.efc__id,
      d._impl.efc__jv,
      d._impl.efc__margin,
      d._impl.efc__mv,
      d._impl.efc__pos,
      d._impl.efc__prev_Mgrad,
      d._impl.efc__prev_cost,
      d._impl.efc__prev_grad,
      d._impl.efc__quad,
      d._impl.efc__quad_gauss,
      d._impl.efc__search,
      d._impl.efc__search_dot,
      d._impl.efc__state,
      d._impl.efc__type,
      d._impl.efc__vel,
  )
  d = d.tree_replace({
      'act': out[0],
      'act_dot': out[1],
      'actuator_force': out[2],
      'actuator_length': out[3],
      '_impl.actuator_moment': out[4],
      '_impl.actuator_velocity': out[5],
      '_impl.cacc': out[6],
      'cam_xmat': out[7],
      'cam_xpos': out[8],
      'cdof': out[9],
      'cdof_dot': out[10],
      '_impl.cfrc_ext': out[11],
      '_impl.cfrc_int': out[12],
      '_impl.cinert': out[13],
      '_impl.collision_pair': out[14],
      '_impl.collision_pairid': out[15],
      '_impl.collision_worldid': out[16],
      '_impl.crb': out[17],
      'ctrl': out[18],
      'cvel': out[19],
      '_impl.energy': out[20],
      'eq_active': out[21],
      '_impl.flexedge_J': out[22],
      '_impl.flexedge_length': out[23],
      '_impl.flexedge_velocity': out[24],
      '_impl.flexvert_xpos': out[25],
      'geom_xmat': out[26],
      'geom_xpos': out[27],
      '_impl.light_xdir': out[28],
      '_impl.light_xpos': out[29],
      'mocap_pos': out[30],
      'mocap_quat': out[31],
      '_impl.nacon': out[32],
      '_impl.ncollision': out[33],
      '_impl.ne': out[34],
      '_impl.ne_connect': out[35],
      '_impl.ne_flex': out[36],
      '_impl.ne_jnt': out[37],
      '_impl.ne_ten': out[38],
      '_impl.ne_weld': out[39],
      '_impl.nefc': out[40],
      '_impl.nf': out[41],
      '_impl.nl': out[42],
      '_impl.nsolving': out[43],
      '_impl.qLD': out[44],
      '_impl.qLDiagInv': out[45],
      '_impl.qM': out[46],
      'qacc': out[47],
      'qacc_smooth': out[48],
      'qacc_warmstart': out[49],
      'qfrc_actuator': out[50],
      'qfrc_applied': out[51],
      'qfrc_bias': out[52],
      'qfrc_constraint': out[53],
      '_impl.qfrc_damper': out[54],
      'qfrc_fluid': out[55],
      'qfrc_gravcomp': out[56],
      'qfrc_passive': out[57],
      'qfrc_smooth': out[58],
      '_impl.qfrc_spring': out[59],
      'qpos': out[60],
      'qvel': out[61],
      'sensordata': out[62],
      'site_xmat': out[63],
      'site_xpos': out[64],
      '_impl.solver_niter': out[65],
      '_impl.subtree_angmom': out[66],
      '_impl.subtree_bodyvel': out[67],
      'subtree_com': out[68],
      '_impl.subtree_linvel': out[69],
      '_impl.ten_J': out[70],
      'ten_length': out[71],
      '_impl.ten_velocity': out[72],
      '_impl.ten_wrapadr': out[73],
      '_impl.ten_wrapnum': out[74],
      'time': out[75],
      '_impl.wrap_obj': out[76],
      '_impl.wrap_xpos': out[77],
      'xanchor': out[78],
      'xaxis': out[79],
      'xfrc_applied': out[80],
      'ximat': out[81],
      'xipos': out[82],
      'xmat': out[83],
      'xpos': out[84],
      'xquat': out[85],
      '_impl.contact__dim': out[86],
      '_impl.contact__dist': out[87],
      '_impl.contact__efc_address': out[88],
      '_impl.contact__frame': out[89],
      '_impl.contact__friction': out[90],
      '_impl.contact__geom': out[91],
      '_impl.contact__geomcollisionid': out[92],
      '_impl.contact__includemargin': out[93],
      '_impl.contact__pos': out[94],
      '_impl.contact__solimp': out[95],
      '_impl.contact__solref': out[96],
      '_impl.contact__solreffriction': out[97],
      '_impl.contact__type': out[98],
      '_impl.contact__worldid': out[99],
      '_impl.efc__D': out[100],
      '_impl.efc__J': out[101],
      '_impl.efc__Jaref': out[102],
      '_impl.efc__Ma': out[103],
      '_impl.efc__Mgrad': out[104],
      '_impl.efc__alpha': out[105],
      '_impl.efc__aref': out[106],
      '_impl.efc__beta': out[107],
      '_impl.efc__cost': out[108],
      '_impl.efc__done': out[109],
      '_impl.efc__force': out[110],
      '_impl.efc__frictionloss': out[111],
      '_impl.efc__gauss': out[112],
      '_impl.efc__grad': out[113],
      '_impl.efc__grad_dot': out[114],
      '_impl.efc__id': out[115],
      '_impl.efc__jv': out[116],
      '_impl.efc__margin': out[117],
      '_impl.efc__mv': out[118],
      '_impl.efc__pos': out[119],
      '_impl.efc__prev_Mgrad': out[120],
      '_impl.efc__prev_cost': out[121],
      '_impl.efc__prev_grad': out[122],
      '_impl.efc__quad': out[123],
      '_impl.efc__quad_gauss': out[124],
      '_impl.efc__search': out[125],
      '_impl.efc__search_dot': out[126],
      '_impl.efc__state': out[127],
      '_impl.efc__type': out[128],
      '_impl.efc__vel': out[129],
  })
  return d


@jax.custom_batching.custom_vmap
@ffi.marshal_jax_warp_callable
def forward(m: types.Model, d: types.Data):
  return _forward_jax_impl(m, d)
@forward.def_vmap
@ffi.marshal_custom_vmap
def forward_vmap(unused_axis_size, is_batched, m, d):
  d = forward(m, d)
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
def _step_shim(
    # Model
    nworld: int,
    M_rowadr: wp.array(dtype=int),
    M_rownnz: wp.array(dtype=int),
    actuator_acc0: wp.array(dtype=float),
    actuator_actadr: wp.array(dtype=int),
    actuator_actearly: wp.array(dtype=bool),
    actuator_actlimited: wp.array(dtype=bool),
    actuator_actnum: wp.array(dtype=int),
    actuator_actrange: wp.array2d(dtype=wp.vec2),
    actuator_biasprm: wp.array2d(dtype=mjwp_types.vec10f),
    actuator_biastype: wp.array(dtype=int),
    actuator_cranklength: wp.array(dtype=float),
    actuator_ctrllimited: wp.array(dtype=bool),
    actuator_ctrlrange: wp.array2d(dtype=wp.vec2),
    actuator_dynprm: wp.array2d(dtype=mjwp_types.vec10f),
    actuator_dyntype: wp.array(dtype=int),
    actuator_forcelimited: wp.array(dtype=bool),
    actuator_forcerange: wp.array2d(dtype=wp.vec2),
    actuator_gainprm: wp.array2d(dtype=mjwp_types.vec10f),
    actuator_gaintype: wp.array(dtype=int),
    actuator_gear: wp.array2d(dtype=wp.spatial_vector),
    actuator_lengthrange: wp.array(dtype=wp.vec2),
    actuator_trnid: wp.array(dtype=wp.vec2i),
    actuator_trntype: wp.array(dtype=int),
    actuator_trntype_body_adr: wp.array(dtype=int),
    block_dim: mjwp_types.BlockDim,
    body_dofadr: wp.array(dtype=int),
    body_dofnum: wp.array(dtype=int),
    body_fluid_ellipsoid: wp.array(dtype=bool),
    body_geomadr: wp.array(dtype=int),
    body_geomnum: wp.array(dtype=int),
    body_gravcomp: wp.array2d(dtype=float),
    body_inertia: wp.array2d(dtype=wp.vec3),
    body_invweight0: wp.array2d(dtype=wp.vec2),
    body_ipos: wp.array2d(dtype=wp.vec3),
    body_iquat: wp.array2d(dtype=wp.quat),
    body_jntadr: wp.array(dtype=int),
    body_jntnum: wp.array(dtype=int),
    body_mass: wp.array2d(dtype=float),
    body_mocapid: wp.array(dtype=int),
    body_parentid: wp.array(dtype=int),
    body_pos: wp.array2d(dtype=wp.vec3),
    body_quat: wp.array2d(dtype=wp.quat),
    body_rootid: wp.array(dtype=int),
    body_subtreemass: wp.array2d(dtype=float),
    body_tree: tuple[wp.array(dtype=int), ...],
    body_weldid: wp.array(dtype=int),
    cam_bodyid: wp.array(dtype=int),
    cam_fovy: wp.array(dtype=float),
    cam_intrinsic: wp.array(dtype=wp.vec4),
    cam_mat0: wp.array2d(dtype=wp.mat33),
    cam_mode: wp.array(dtype=int),
    cam_pos: wp.array2d(dtype=wp.vec3),
    cam_pos0: wp.array2d(dtype=wp.vec3),
    cam_poscom0: wp.array2d(dtype=wp.vec3),
    cam_quat: wp.array2d(dtype=wp.quat),
    cam_resolution: wp.array(dtype=wp.vec2i),
    cam_sensorsize: wp.array(dtype=wp.vec2),
    cam_targetbodyid: wp.array(dtype=int),
    dof_Madr: wp.array(dtype=int),
    dof_armature: wp.array2d(dtype=float),
    dof_bodyid: wp.array(dtype=int),
    dof_damping: wp.array2d(dtype=float),
    dof_frictionloss: wp.array2d(dtype=float),
    dof_invweight0: wp.array2d(dtype=float),
    dof_jntid: wp.array(dtype=int),
    dof_parentid: wp.array(dtype=int),
    dof_solimp: wp.array2d(dtype=mjwp_types.vec5),
    dof_solref: wp.array2d(dtype=wp.vec2),
    dof_tri_col: wp.array(dtype=int),
    dof_tri_row: wp.array(dtype=int),
    eq_connect_adr: wp.array(dtype=int),
    eq_data: wp.array2d(dtype=mjwp_types.vec11),
    eq_flex_adr: wp.array(dtype=int),
    eq_jnt_adr: wp.array(dtype=int),
    eq_obj1id: wp.array(dtype=int),
    eq_obj2id: wp.array(dtype=int),
    eq_objtype: wp.array(dtype=int),
    eq_solimp: wp.array2d(dtype=mjwp_types.vec5),
    eq_solref: wp.array2d(dtype=wp.vec2),
    eq_ten_adr: wp.array(dtype=int),
    eq_wld_adr: wp.array(dtype=int),
    flex_bending: wp.array2d(dtype=float),
    flex_damping: wp.array(dtype=float),
    flex_dim: wp.array(dtype=int),
    flex_edge: wp.array(dtype=wp.vec2i),
    flex_edgeadr: wp.array(dtype=int),
    flex_edgeflap: wp.array(dtype=wp.vec2i),
    flex_edgenum: wp.array(dtype=int),
    flex_elem: wp.array(dtype=int),
    flex_elemadr: wp.array(dtype=int),
    flex_elemedge: wp.array(dtype=int),
    flex_elemedgeadr: wp.array(dtype=int),
    flex_elemnum: wp.array(dtype=int),
    flex_stiffness: wp.array2d(dtype=float),
    flex_vertadr: wp.array(dtype=int),
    flex_vertbodyid: wp.array(dtype=int),
    flexedge_invweight0: wp.array(dtype=float),
    flexedge_length0: wp.array(dtype=float),
    geom_aabb: wp.array3d(dtype=wp.vec3),
    geom_bodyid: wp.array(dtype=int),
    geom_condim: wp.array(dtype=int),
    geom_dataid: wp.array(dtype=int),
    geom_fluid: wp.array2d(dtype=float),
    geom_friction: wp.array2d(dtype=wp.vec3),
    geom_gap: wp.array2d(dtype=float),
    geom_group: wp.array(dtype=int),
    geom_margin: wp.array2d(dtype=float),
    geom_matid: wp.array2d(dtype=int),
    geom_pair_type_count: tuple[int, ...],
    geom_plugin_index: wp.array(dtype=int),
    geom_pos: wp.array2d(dtype=wp.vec3),
    geom_priority: wp.array(dtype=int),
    geom_quat: wp.array2d(dtype=wp.quat),
    geom_rbound: wp.array2d(dtype=float),
    geom_rgba: wp.array2d(dtype=wp.vec4),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_solimp: wp.array2d(dtype=mjwp_types.vec5),
    geom_solmix: wp.array2d(dtype=float),
    geom_solref: wp.array2d(dtype=wp.vec2),
    geom_type: wp.array(dtype=int),
    has_sdf_geom: bool,
    hfield_adr: wp.array(dtype=int),
    hfield_data: wp.array(dtype=float),
    hfield_ncol: wp.array(dtype=int),
    hfield_nrow: wp.array(dtype=int),
    hfield_size: wp.array(dtype=wp.vec4),
    jnt_actfrclimited: wp.array(dtype=bool),
    jnt_actfrcrange: wp.array2d(dtype=wp.vec2),
    jnt_actgravcomp: wp.array(dtype=int),
    jnt_axis: wp.array2d(dtype=wp.vec3),
    jnt_bodyid: wp.array(dtype=int),
    jnt_dofadr: wp.array(dtype=int),
    jnt_limited_ball_adr: wp.array(dtype=int),
    jnt_limited_slide_hinge_adr: wp.array(dtype=int),
    jnt_margin: wp.array2d(dtype=float),
    jnt_pos: wp.array2d(dtype=wp.vec3),
    jnt_qposadr: wp.array(dtype=int),
    jnt_range: wp.array2d(dtype=wp.vec2),
    jnt_solimp: wp.array2d(dtype=mjwp_types.vec5),
    jnt_solref: wp.array2d(dtype=wp.vec2),
    jnt_stiffness: wp.array2d(dtype=float),
    jnt_type: wp.array(dtype=int),
    light_bodyid: wp.array(dtype=int),
    light_dir: wp.array2d(dtype=wp.vec3),
    light_dir0: wp.array2d(dtype=wp.vec3),
    light_mode: wp.array(dtype=int),
    light_pos: wp.array2d(dtype=wp.vec3),
    light_pos0: wp.array2d(dtype=wp.vec3),
    light_poscom0: wp.array2d(dtype=wp.vec3),
    light_targetbodyid: wp.array(dtype=int),
    mapM2M: wp.array(dtype=int),
    mat_rgba: wp.array2d(dtype=wp.vec4),
    mesh_face: wp.array(dtype=wp.vec3i),
    mesh_faceadr: wp.array(dtype=int),
    mesh_graph: wp.array(dtype=int),
    mesh_graphadr: wp.array(dtype=int),
    mesh_normal: wp.array(dtype=wp.vec3),
    mesh_normaladr: wp.array(dtype=int),
    mesh_polyadr: wp.array(dtype=int),
    mesh_polymap: wp.array(dtype=int),
    mesh_polymapadr: wp.array(dtype=int),
    mesh_polymapnum: wp.array(dtype=int),
    mesh_polynormal: wp.array(dtype=wp.vec3),
    mesh_polynum: wp.array(dtype=int),
    mesh_polyvert: wp.array(dtype=int),
    mesh_polyvertadr: wp.array(dtype=int),
    mesh_polyvertnum: wp.array(dtype=int),
    mesh_quat: wp.array(dtype=wp.quat),
    mesh_vert: wp.array(dtype=wp.vec3),
    mesh_vertadr: wp.array(dtype=int),
    mesh_vertnum: wp.array(dtype=int),
    nC: int,
    nM: int,
    na: int,
    nacttrnbody: int,
    nbody: int,
    ncam: int,
    neq: int,
    nflex: int,
    nflexedge: int,
    nflexelem: int,
    nflexvert: int,
    ngeom: int,
    ngravcomp: int,
    njnt: int,
    nlight: int,
    nmaxcondim: int,
    nmaxmeshdeg: int,
    nmaxpolygon: int,
    nmaxpyramid: int,
    nmeshface: int,
    nrangefinder: int,
    nsensorcollision: int,
    nsensorcontact: int,
    nsensortaxel: int,
    nsite: int,
    ntendon: int,
    nu: int,
    nv: int,
    nv_pad: int,
    nwrap: int,
    nxn_geom_pair_filtered: wp.array(dtype=wp.vec2i),
    nxn_pairid: wp.array(dtype=wp.vec2i),
    nxn_pairid_filtered: wp.array(dtype=wp.vec2i),
    oct_aabb: wp.array2d(dtype=wp.vec3),
    oct_child: wp.array(dtype=mjwp_types.vec8i),
    oct_coeff: wp.array(dtype=mjwp_types.vec8),
    pair_dim: wp.array(dtype=int),
    pair_friction: wp.array2d(dtype=mjwp_types.vec5),
    pair_gap: wp.array2d(dtype=float),
    pair_margin: wp.array2d(dtype=float),
    pair_solimp: wp.array2d(dtype=mjwp_types.vec5),
    pair_solref: wp.array2d(dtype=wp.vec2),
    pair_solreffriction: wp.array2d(dtype=wp.vec2),
    plugin: wp.array(dtype=int),
    plugin_attr: wp.array(dtype=wp.vec3f),
    qLD_updates: tuple[wp.array(dtype=wp.vec3i), ...],
    qM_fullm_i: wp.array(dtype=int),
    qM_fullm_j: wp.array(dtype=int),
    qM_madr_ij: wp.array(dtype=int),
    qM_mulm_i: wp.array(dtype=int),
    qM_mulm_j: wp.array(dtype=int),
    qM_tiles: tuple[mjwp_types.TileSet, ...],
    qpos0: wp.array2d(dtype=float),
    qpos_spring: wp.array2d(dtype=float),
    rangefinder_sensor_adr: wp.array(dtype=int),
    sensor_acc_adr: wp.array(dtype=int),
    sensor_adr: wp.array(dtype=int),
    sensor_adr_to_contact_adr: wp.array(dtype=int),
    sensor_contact_adr: wp.array(dtype=int),
    sensor_cutoff: wp.array(dtype=float),
    sensor_datatype: wp.array(dtype=int),
    sensor_dim: wp.array(dtype=int),
    sensor_e_kinetic: bool,
    sensor_e_potential: bool,
    sensor_intprm: wp.array2d(dtype=int),
    sensor_limitfrc_adr: wp.array(dtype=int),
    sensor_limitpos_adr: wp.array(dtype=int),
    sensor_limitvel_adr: wp.array(dtype=int),
    sensor_objid: wp.array(dtype=int),
    sensor_objtype: wp.array(dtype=int),
    sensor_pos_adr: wp.array(dtype=int),
    sensor_rangefinder_adr: wp.array(dtype=int),
    sensor_rangefinder_bodyid: wp.array(dtype=int),
    sensor_refid: wp.array(dtype=int),
    sensor_reftype: wp.array(dtype=int),
    sensor_rne_postconstraint: bool,
    sensor_subtree_vel: bool,
    sensor_tendonactfrc_adr: wp.array(dtype=int),
    sensor_touch_adr: wp.array(dtype=int),
    sensor_type: wp.array(dtype=int),
    sensor_vel_adr: wp.array(dtype=int),
    site_bodyid: wp.array(dtype=int),
    site_pos: wp.array2d(dtype=wp.vec3),
    site_quat: wp.array2d(dtype=wp.quat),
    site_size: wp.array(dtype=wp.vec3),
    site_type: wp.array(dtype=int),
    taxel_sensorid: wp.array(dtype=int),
    taxel_vertadr: wp.array(dtype=int),
    tendon_actfrclimited: wp.array(dtype=bool),
    tendon_actfrcrange: wp.array2d(dtype=wp.vec2),
    tendon_adr: wp.array(dtype=int),
    tendon_armature: wp.array2d(dtype=float),
    tendon_damping: wp.array2d(dtype=float),
    tendon_frictionloss: wp.array2d(dtype=float),
    tendon_geom_adr: wp.array(dtype=int),
    tendon_invweight0: wp.array2d(dtype=float),
    tendon_jnt_adr: wp.array(dtype=int),
    tendon_length0: wp.array2d(dtype=float),
    tendon_lengthspring: wp.array2d(dtype=wp.vec2),
    tendon_limited_adr: wp.array(dtype=int),
    tendon_margin: wp.array2d(dtype=float),
    tendon_num: wp.array(dtype=int),
    tendon_range: wp.array2d(dtype=wp.vec2),
    tendon_site_pair_adr: wp.array(dtype=int),
    tendon_solimp_fri: wp.array2d(dtype=mjwp_types.vec5),
    tendon_solimp_lim: wp.array2d(dtype=mjwp_types.vec5),
    tendon_solref_fri: wp.array2d(dtype=wp.vec2),
    tendon_solref_lim: wp.array2d(dtype=wp.vec2),
    tendon_stiffness: wp.array2d(dtype=float),
    wrap_geom_adr: wp.array(dtype=int),
    wrap_jnt_adr: wp.array(dtype=int),
    wrap_objid: wp.array(dtype=int),
    wrap_prm: wp.array(dtype=float),
    wrap_pulley_scale: wp.array(dtype=float),
    wrap_site_pair_adr: wp.array(dtype=int),
    wrap_type: wp.array(dtype=int),
    opt__broadphase: int,
    opt__broadphase_filter: int,
    opt__ccd_iterations: int,
    opt__ccd_tolerance: wp.array(dtype=float),
    opt__cone: int,
    opt__contact_sensor_maxmatch: int,
    opt__density: wp.array(dtype=float),
    opt__disableflags: int,
    opt__enableflags: int,
    opt__graph_conditional: bool,
    opt__gravity: wp.array(dtype=wp.vec3),
    opt__has_fluid: bool,
    opt__impratio_invsqrt: wp.array(dtype=float),
    opt__integrator: int,
    opt__is_sparse: bool,
    opt__iterations: int,
    opt__ls_iterations: int,
    opt__ls_parallel: bool,
    opt__ls_parallel_min_step: float,
    opt__ls_tolerance: wp.array(dtype=float),
    opt__magnetic: wp.array(dtype=wp.vec3),
    opt__run_collision_detection: bool,
    opt__sdf_initpoints: int,
    opt__sdf_iterations: int,
    opt__solver: int,
    opt__timestep: wp.array(dtype=float),
    opt__tolerance: wp.array(dtype=float),
    opt__viscosity: wp.array(dtype=float),
    opt__wind: wp.array(dtype=wp.vec3),
    stat__meaninertia: float,
    # Data
    naconmax: int,
    njmax: int,
    act: wp.array2d(dtype=float),
    act_dot: wp.array2d(dtype=float),
    actuator_force: wp.array2d(dtype=float),
    actuator_length: wp.array2d(dtype=float),
    actuator_moment: wp.array3d(dtype=float),
    actuator_velocity: wp.array2d(dtype=float),
    cacc: wp.array2d(dtype=wp.spatial_vector),
    cam_xmat: wp.array2d(dtype=wp.mat33),
    cam_xpos: wp.array2d(dtype=wp.vec3),
    cdof: wp.array2d(dtype=wp.spatial_vector),
    cdof_dot: wp.array2d(dtype=wp.spatial_vector),
    cfrc_ext: wp.array2d(dtype=wp.spatial_vector),
    cfrc_int: wp.array2d(dtype=wp.spatial_vector),
    cinert: wp.array2d(dtype=mjwp_types.vec10),
    collision_pair: wp.array(dtype=wp.vec2i),
    collision_pairid: wp.array(dtype=wp.vec2i),
    collision_worldid: wp.array(dtype=int),
    crb: wp.array2d(dtype=mjwp_types.vec10),
    ctrl: wp.array2d(dtype=float),
    cvel: wp.array2d(dtype=wp.spatial_vector),
    energy: wp.array(dtype=wp.vec2),
    eq_active: wp.array2d(dtype=bool),
    flexedge_J: wp.array3d(dtype=float),
    flexedge_length: wp.array2d(dtype=float),
    flexedge_velocity: wp.array2d(dtype=float),
    flexvert_xpos: wp.array2d(dtype=wp.vec3),
    geom_xmat: wp.array2d(dtype=wp.mat33),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    light_xdir: wp.array2d(dtype=wp.vec3),
    light_xpos: wp.array2d(dtype=wp.vec3),
    mocap_pos: wp.array2d(dtype=wp.vec3),
    mocap_quat: wp.array2d(dtype=wp.quat),
    nacon: wp.array(dtype=int),
    ncollision: wp.array(dtype=int),
    ne: wp.array(dtype=int),
    ne_connect: wp.array(dtype=int),
    ne_flex: wp.array(dtype=int),
    ne_jnt: wp.array(dtype=int),
    ne_ten: wp.array(dtype=int),
    ne_weld: wp.array(dtype=int),
    nefc: wp.array(dtype=int),
    nf: wp.array(dtype=int),
    nl: wp.array(dtype=int),
    nsolving: wp.array(dtype=int),
    qLD: wp.array3d(dtype=float),
    qLDiagInv: wp.array2d(dtype=float),
    qM: wp.array3d(dtype=float),
    qacc: wp.array2d(dtype=float),
    qacc_smooth: wp.array2d(dtype=float),
    qacc_warmstart: wp.array2d(dtype=float),
    qfrc_actuator: wp.array2d(dtype=float),
    qfrc_applied: wp.array2d(dtype=float),
    qfrc_bias: wp.array2d(dtype=float),
    qfrc_constraint: wp.array2d(dtype=float),
    qfrc_damper: wp.array2d(dtype=float),
    qfrc_fluid: wp.array2d(dtype=float),
    qfrc_gravcomp: wp.array2d(dtype=float),
    qfrc_passive: wp.array2d(dtype=float),
    qfrc_smooth: wp.array2d(dtype=float),
    qfrc_spring: wp.array2d(dtype=float),
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    sensordata: wp.array2d(dtype=float),
    site_xmat: wp.array2d(dtype=wp.mat33),
    site_xpos: wp.array2d(dtype=wp.vec3),
    solver_niter: wp.array(dtype=int),
    subtree_angmom: wp.array2d(dtype=wp.vec3),
    subtree_bodyvel: wp.array2d(dtype=wp.spatial_vector),
    subtree_com: wp.array2d(dtype=wp.vec3),
    subtree_linvel: wp.array2d(dtype=wp.vec3),
    ten_J: wp.array3d(dtype=float),
    ten_length: wp.array2d(dtype=float),
    ten_velocity: wp.array2d(dtype=float),
    ten_wrapadr: wp.array2d(dtype=int),
    ten_wrapnum: wp.array2d(dtype=int),
    time: wp.array(dtype=float),
    wrap_obj: wp.array2d(dtype=wp.vec2i),
    wrap_xpos: wp.array2d(dtype=wp.spatial_vector),
    xanchor: wp.array2d(dtype=wp.vec3),
    xaxis: wp.array2d(dtype=wp.vec3),
    xfrc_applied: wp.array2d(dtype=wp.spatial_vector),
    ximat: wp.array2d(dtype=wp.mat33),
    xipos: wp.array2d(dtype=wp.vec3),
    xmat: wp.array2d(dtype=wp.mat33),
    xpos: wp.array2d(dtype=wp.vec3),
    xquat: wp.array2d(dtype=wp.quat),
    contact__dim: wp.array(dtype=int),
    contact__dist: wp.array(dtype=float),
    contact__efc_address: wp.array2d(dtype=int),
    contact__frame: wp.array(dtype=wp.mat33),
    contact__friction: wp.array(dtype=mjwp_types.vec5),
    contact__geom: wp.array(dtype=wp.vec2i),
    contact__geomcollisionid: wp.array(dtype=int),
    contact__includemargin: wp.array(dtype=float),
    contact__pos: wp.array(dtype=wp.vec3),
    contact__solimp: wp.array(dtype=mjwp_types.vec5),
    contact__solref: wp.array(dtype=wp.vec2),
    contact__solreffriction: wp.array(dtype=wp.vec2),
    contact__type: wp.array(dtype=int),
    contact__worldid: wp.array(dtype=int),
    efc__D: wp.array2d(dtype=float),
    efc__J: wp.array3d(dtype=float),
    efc__Jaref: wp.array2d(dtype=float),
    efc__Ma: wp.array2d(dtype=float),
    efc__Mgrad: wp.array2d(dtype=float),
    efc__alpha: wp.array(dtype=float),
    efc__aref: wp.array2d(dtype=float),
    efc__beta: wp.array(dtype=float),
    efc__cost: wp.array(dtype=float),
    efc__done: wp.array(dtype=bool),
    efc__force: wp.array2d(dtype=float),
    efc__frictionloss: wp.array2d(dtype=float),
    efc__gauss: wp.array(dtype=float),
    efc__grad: wp.array2d(dtype=float),
    efc__grad_dot: wp.array(dtype=float),
    efc__id: wp.array2d(dtype=int),
    efc__jv: wp.array2d(dtype=float),
    efc__margin: wp.array2d(dtype=float),
    efc__mv: wp.array2d(dtype=float),
    efc__pos: wp.array2d(dtype=float),
    efc__prev_Mgrad: wp.array2d(dtype=float),
    efc__prev_cost: wp.array(dtype=float),
    efc__prev_grad: wp.array2d(dtype=float),
    efc__quad: wp.array2d(dtype=wp.vec3),
    efc__quad_gauss: wp.array(dtype=wp.vec3),
    efc__search: wp.array2d(dtype=float),
    efc__search_dot: wp.array(dtype=float),
    efc__state: wp.array2d(dtype=int),
    efc__type: wp.array2d(dtype=int),
    efc__vel: wp.array2d(dtype=float),
):
  _m.stat = _s
  _m.opt = _o
  _d.efc = _e
  _d.contact = _c
  _m.M_rowadr = M_rowadr
  _m.M_rownnz = M_rownnz
  _m.actuator_acc0 = actuator_acc0
  _m.actuator_actadr = actuator_actadr
  _m.actuator_actearly = actuator_actearly
  _m.actuator_actlimited = actuator_actlimited
  _m.actuator_actnum = actuator_actnum
  _m.actuator_actrange = actuator_actrange
  _m.actuator_biasprm = actuator_biasprm
  _m.actuator_biastype = actuator_biastype
  _m.actuator_cranklength = actuator_cranklength
  _m.actuator_ctrllimited = actuator_ctrllimited
  _m.actuator_ctrlrange = actuator_ctrlrange
  _m.actuator_dynprm = actuator_dynprm
  _m.actuator_dyntype = actuator_dyntype
  _m.actuator_forcelimited = actuator_forcelimited
  _m.actuator_forcerange = actuator_forcerange
  _m.actuator_gainprm = actuator_gainprm
  _m.actuator_gaintype = actuator_gaintype
  _m.actuator_gear = actuator_gear
  _m.actuator_lengthrange = actuator_lengthrange
  _m.actuator_trnid = actuator_trnid
  _m.actuator_trntype = actuator_trntype
  _m.actuator_trntype_body_adr = actuator_trntype_body_adr
  _m.block_dim = block_dim
  _m.body_dofadr = body_dofadr
  _m.body_dofnum = body_dofnum
  _m.body_fluid_ellipsoid = body_fluid_ellipsoid
  _m.body_geomadr = body_geomadr
  _m.body_geomnum = body_geomnum
  _m.body_gravcomp = body_gravcomp
  _m.body_inertia = body_inertia
  _m.body_invweight0 = body_invweight0
  _m.body_ipos = body_ipos
  _m.body_iquat = body_iquat
  _m.body_jntadr = body_jntadr
  _m.body_jntnum = body_jntnum
  _m.body_mass = body_mass
  _m.body_mocapid = body_mocapid
  _m.body_parentid = body_parentid
  _m.body_pos = body_pos
  _m.body_quat = body_quat
  _m.body_rootid = body_rootid
  _m.body_subtreemass = body_subtreemass
  _m.body_tree = body_tree
  _m.body_weldid = body_weldid
  _m.cam_bodyid = cam_bodyid
  _m.cam_fovy = cam_fovy
  _m.cam_intrinsic = cam_intrinsic
  _m.cam_mat0 = cam_mat0
  _m.cam_mode = cam_mode
  _m.cam_pos = cam_pos
  _m.cam_pos0 = cam_pos0
  _m.cam_poscom0 = cam_poscom0
  _m.cam_quat = cam_quat
  _m.cam_resolution = cam_resolution
  _m.cam_sensorsize = cam_sensorsize
  _m.cam_targetbodyid = cam_targetbodyid
  _m.dof_Madr = dof_Madr
  _m.dof_armature = dof_armature
  _m.dof_bodyid = dof_bodyid
  _m.dof_damping = dof_damping
  _m.dof_frictionloss = dof_frictionloss
  _m.dof_invweight0 = dof_invweight0
  _m.dof_jntid = dof_jntid
  _m.dof_parentid = dof_parentid
  _m.dof_solimp = dof_solimp
  _m.dof_solref = dof_solref
  _m.dof_tri_col = dof_tri_col
  _m.dof_tri_row = dof_tri_row
  _m.eq_connect_adr = eq_connect_adr
  _m.eq_data = eq_data
  _m.eq_flex_adr = eq_flex_adr
  _m.eq_jnt_adr = eq_jnt_adr
  _m.eq_obj1id = eq_obj1id
  _m.eq_obj2id = eq_obj2id
  _m.eq_objtype = eq_objtype
  _m.eq_solimp = eq_solimp
  _m.eq_solref = eq_solref
  _m.eq_ten_adr = eq_ten_adr
  _m.eq_wld_adr = eq_wld_adr
  _m.flex_bending = flex_bending
  _m.flex_damping = flex_damping
  _m.flex_dim = flex_dim
  _m.flex_edge = flex_edge
  _m.flex_edgeadr = flex_edgeadr
  _m.flex_edgeflap = flex_edgeflap
  _m.flex_edgenum = flex_edgenum
  _m.flex_elem = flex_elem
  _m.flex_elemadr = flex_elemadr
  _m.flex_elemedge = flex_elemedge
  _m.flex_elemedgeadr = flex_elemedgeadr
  _m.flex_elemnum = flex_elemnum
  _m.flex_stiffness = flex_stiffness
  _m.flex_vertadr = flex_vertadr
  _m.flex_vertbodyid = flex_vertbodyid
  _m.flexedge_invweight0 = flexedge_invweight0
  _m.flexedge_length0 = flexedge_length0
  _m.geom_aabb = geom_aabb
  _m.geom_bodyid = geom_bodyid
  _m.geom_condim = geom_condim
  _m.geom_dataid = geom_dataid
  _m.geom_fluid = geom_fluid
  _m.geom_friction = geom_friction
  _m.geom_gap = geom_gap
  _m.geom_group = geom_group
  _m.geom_margin = geom_margin
  _m.geom_matid = geom_matid
  _m.geom_pair_type_count = geom_pair_type_count
  _m.geom_plugin_index = geom_plugin_index
  _m.geom_pos = geom_pos
  _m.geom_priority = geom_priority
  _m.geom_quat = geom_quat
  _m.geom_rbound = geom_rbound
  _m.geom_rgba = geom_rgba
  _m.geom_size = geom_size
  _m.geom_solimp = geom_solimp
  _m.geom_solmix = geom_solmix
  _m.geom_solref = geom_solref
  _m.geom_type = geom_type
  _m.has_sdf_geom = has_sdf_geom
  _m.hfield_adr = hfield_adr
  _m.hfield_data = hfield_data
  _m.hfield_ncol = hfield_ncol
  _m.hfield_nrow = hfield_nrow
  _m.hfield_size = hfield_size
  _m.jnt_actfrclimited = jnt_actfrclimited
  _m.jnt_actfrcrange = jnt_actfrcrange
  _m.jnt_actgravcomp = jnt_actgravcomp
  _m.jnt_axis = jnt_axis
  _m.jnt_bodyid = jnt_bodyid
  _m.jnt_dofadr = jnt_dofadr
  _m.jnt_limited_ball_adr = jnt_limited_ball_adr
  _m.jnt_limited_slide_hinge_adr = jnt_limited_slide_hinge_adr
  _m.jnt_margin = jnt_margin
  _m.jnt_pos = jnt_pos
  _m.jnt_qposadr = jnt_qposadr
  _m.jnt_range = jnt_range
  _m.jnt_solimp = jnt_solimp
  _m.jnt_solref = jnt_solref
  _m.jnt_stiffness = jnt_stiffness
  _m.jnt_type = jnt_type
  _m.light_bodyid = light_bodyid
  _m.light_dir = light_dir
  _m.light_dir0 = light_dir0
  _m.light_mode = light_mode
  _m.light_pos = light_pos
  _m.light_pos0 = light_pos0
  _m.light_poscom0 = light_poscom0
  _m.light_targetbodyid = light_targetbodyid
  _m.mapM2M = mapM2M
  _m.mat_rgba = mat_rgba
  _m.mesh_face = mesh_face
  _m.mesh_faceadr = mesh_faceadr
  _m.mesh_graph = mesh_graph
  _m.mesh_graphadr = mesh_graphadr
  _m.mesh_normal = mesh_normal
  _m.mesh_normaladr = mesh_normaladr
  _m.mesh_polyadr = mesh_polyadr
  _m.mesh_polymap = mesh_polymap
  _m.mesh_polymapadr = mesh_polymapadr
  _m.mesh_polymapnum = mesh_polymapnum
  _m.mesh_polynormal = mesh_polynormal
  _m.mesh_polynum = mesh_polynum
  _m.mesh_polyvert = mesh_polyvert
  _m.mesh_polyvertadr = mesh_polyvertadr
  _m.mesh_polyvertnum = mesh_polyvertnum
  _m.mesh_quat = mesh_quat
  _m.mesh_vert = mesh_vert
  _m.mesh_vertadr = mesh_vertadr
  _m.mesh_vertnum = mesh_vertnum
  _m.nC = nC
  _m.nM = nM
  _m.na = na
  _m.nacttrnbody = nacttrnbody
  _m.nbody = nbody
  _m.ncam = ncam
  _m.neq = neq
  _m.nflex = nflex
  _m.nflexedge = nflexedge
  _m.nflexelem = nflexelem
  _m.nflexvert = nflexvert
  _m.ngeom = ngeom
  _m.ngravcomp = ngravcomp
  _m.njnt = njnt
  _m.nlight = nlight
  _m.nmaxcondim = nmaxcondim
  _m.nmaxmeshdeg = nmaxmeshdeg
  _m.nmaxpolygon = nmaxpolygon
  _m.nmaxpyramid = nmaxpyramid
  _m.nmeshface = nmeshface
  _m.nrangefinder = nrangefinder
  _m.nsensorcollision = nsensorcollision
  _m.nsensorcontact = nsensorcontact
  _m.nsensortaxel = nsensortaxel
  _m.nsite = nsite
  _m.ntendon = ntendon
  _m.nu = nu
  _m.nv = nv
  _m.nv_pad = nv_pad
  _m.nwrap = nwrap
  _m.nxn_geom_pair_filtered = nxn_geom_pair_filtered
  _m.nxn_pairid = nxn_pairid
  _m.nxn_pairid_filtered = nxn_pairid_filtered
  _m.oct_aabb = oct_aabb
  _m.oct_child = oct_child
  _m.oct_coeff = oct_coeff
  _m.opt.broadphase = opt__broadphase
  _m.opt.broadphase_filter = opt__broadphase_filter
  _m.opt.ccd_iterations = opt__ccd_iterations
  _m.opt.ccd_tolerance = opt__ccd_tolerance
  _m.opt.cone = opt__cone
  _m.opt.contact_sensor_maxmatch = opt__contact_sensor_maxmatch
  _m.opt.density = opt__density
  _m.opt.disableflags = opt__disableflags
  _m.opt.enableflags = opt__enableflags
  _m.opt.graph_conditional = opt__graph_conditional
  _m.opt.gravity = opt__gravity
  _m.opt.has_fluid = opt__has_fluid
  _m.opt.impratio_invsqrt = opt__impratio_invsqrt
  _m.opt.integrator = opt__integrator
  _m.opt.is_sparse = opt__is_sparse
  _m.opt.iterations = opt__iterations
  _m.opt.ls_iterations = opt__ls_iterations
  _m.opt.ls_parallel = opt__ls_parallel
  _m.opt.ls_parallel_min_step = opt__ls_parallel_min_step
  _m.opt.ls_tolerance = opt__ls_tolerance
  _m.opt.magnetic = opt__magnetic
  _m.opt.run_collision_detection = opt__run_collision_detection
  _m.opt.sdf_initpoints = opt__sdf_initpoints
  _m.opt.sdf_iterations = opt__sdf_iterations
  _m.opt.solver = opt__solver
  _m.opt.timestep = opt__timestep
  _m.opt.tolerance = opt__tolerance
  _m.opt.viscosity = opt__viscosity
  _m.opt.wind = opt__wind
  _m.pair_dim = pair_dim
  _m.pair_friction = pair_friction
  _m.pair_gap = pair_gap
  _m.pair_margin = pair_margin
  _m.pair_solimp = pair_solimp
  _m.pair_solref = pair_solref
  _m.pair_solreffriction = pair_solreffriction
  _m.plugin = plugin
  _m.plugin_attr = plugin_attr
  _m.qLD_updates = qLD_updates
  _m.qM_fullm_i = qM_fullm_i
  _m.qM_fullm_j = qM_fullm_j
  _m.qM_madr_ij = qM_madr_ij
  _m.qM_mulm_i = qM_mulm_i
  _m.qM_mulm_j = qM_mulm_j
  _m.qM_tiles = qM_tiles
  _m.qpos0 = qpos0
  _m.qpos_spring = qpos_spring
  _m.rangefinder_sensor_adr = rangefinder_sensor_adr
  _m.sensor_acc_adr = sensor_acc_adr
  _m.sensor_adr = sensor_adr
  _m.sensor_adr_to_contact_adr = sensor_adr_to_contact_adr
  _m.sensor_contact_adr = sensor_contact_adr
  _m.sensor_cutoff = sensor_cutoff
  _m.sensor_datatype = sensor_datatype
  _m.sensor_dim = sensor_dim
  _m.sensor_e_kinetic = sensor_e_kinetic
  _m.sensor_e_potential = sensor_e_potential
  _m.sensor_intprm = sensor_intprm
  _m.sensor_limitfrc_adr = sensor_limitfrc_adr
  _m.sensor_limitpos_adr = sensor_limitpos_adr
  _m.sensor_limitvel_adr = sensor_limitvel_adr
  _m.sensor_objid = sensor_objid
  _m.sensor_objtype = sensor_objtype
  _m.sensor_pos_adr = sensor_pos_adr
  _m.sensor_rangefinder_adr = sensor_rangefinder_adr
  _m.sensor_rangefinder_bodyid = sensor_rangefinder_bodyid
  _m.sensor_refid = sensor_refid
  _m.sensor_reftype = sensor_reftype
  _m.sensor_rne_postconstraint = sensor_rne_postconstraint
  _m.sensor_subtree_vel = sensor_subtree_vel
  _m.sensor_tendonactfrc_adr = sensor_tendonactfrc_adr
  _m.sensor_touch_adr = sensor_touch_adr
  _m.sensor_type = sensor_type
  _m.sensor_vel_adr = sensor_vel_adr
  _m.site_bodyid = site_bodyid
  _m.site_pos = site_pos
  _m.site_quat = site_quat
  _m.site_size = site_size
  _m.site_type = site_type
  _m.stat.meaninertia = stat__meaninertia
  _m.taxel_sensorid = taxel_sensorid
  _m.taxel_vertadr = taxel_vertadr
  _m.tendon_actfrclimited = tendon_actfrclimited
  _m.tendon_actfrcrange = tendon_actfrcrange
  _m.tendon_adr = tendon_adr
  _m.tendon_armature = tendon_armature
  _m.tendon_damping = tendon_damping
  _m.tendon_frictionloss = tendon_frictionloss
  _m.tendon_geom_adr = tendon_geom_adr
  _m.tendon_invweight0 = tendon_invweight0
  _m.tendon_jnt_adr = tendon_jnt_adr
  _m.tendon_length0 = tendon_length0
  _m.tendon_lengthspring = tendon_lengthspring
  _m.tendon_limited_adr = tendon_limited_adr
  _m.tendon_margin = tendon_margin
  _m.tendon_num = tendon_num
  _m.tendon_range = tendon_range
  _m.tendon_site_pair_adr = tendon_site_pair_adr
  _m.tendon_solimp_fri = tendon_solimp_fri
  _m.tendon_solimp_lim = tendon_solimp_lim
  _m.tendon_solref_fri = tendon_solref_fri
  _m.tendon_solref_lim = tendon_solref_lim
  _m.tendon_stiffness = tendon_stiffness
  _m.wrap_geom_adr = wrap_geom_adr
  _m.wrap_jnt_adr = wrap_jnt_adr
  _m.wrap_objid = wrap_objid
  _m.wrap_prm = wrap_prm
  _m.wrap_pulley_scale = wrap_pulley_scale
  _m.wrap_site_pair_adr = wrap_site_pair_adr
  _m.wrap_type = wrap_type
  _d.act = act
  _d.act_dot = act_dot
  _d.actuator_force = actuator_force
  _d.actuator_length = actuator_length
  _d.actuator_moment = actuator_moment
  _d.actuator_velocity = actuator_velocity
  _d.cacc = cacc
  _d.cam_xmat = cam_xmat
  _d.cam_xpos = cam_xpos
  _d.cdof = cdof
  _d.cdof_dot = cdof_dot
  _d.cfrc_ext = cfrc_ext
  _d.cfrc_int = cfrc_int
  _d.cinert = cinert
  _d.collision_pair = collision_pair
  _d.collision_pairid = collision_pairid
  _d.collision_worldid = collision_worldid
  _d.contact.dim = contact__dim
  _d.contact.dist = contact__dist
  _d.contact.efc_address = contact__efc_address
  _d.contact.frame = contact__frame
  _d.contact.friction = contact__friction
  _d.contact.geom = contact__geom
  _d.contact.geomcollisionid = contact__geomcollisionid
  _d.contact.includemargin = contact__includemargin
  _d.contact.pos = contact__pos
  _d.contact.solimp = contact__solimp
  _d.contact.solref = contact__solref
  _d.contact.solreffriction = contact__solreffriction
  _d.contact.type = contact__type
  _d.contact.worldid = contact__worldid
  _d.crb = crb
  _d.ctrl = ctrl
  _d.cvel = cvel
  _d.efc.D = efc__D
  _d.efc.J = efc__J
  _d.efc.Jaref = efc__Jaref
  _d.efc.Ma = efc__Ma
  _d.efc.Mgrad = efc__Mgrad
  _d.efc.alpha = efc__alpha
  _d.efc.aref = efc__aref
  _d.efc.beta = efc__beta
  _d.efc.cost = efc__cost
  _d.efc.done = efc__done
  _d.efc.force = efc__force
  _d.efc.frictionloss = efc__frictionloss
  _d.efc.gauss = efc__gauss
  _d.efc.grad = efc__grad
  _d.efc.grad_dot = efc__grad_dot
  _d.efc.id = efc__id
  _d.efc.jv = efc__jv
  _d.efc.margin = efc__margin
  _d.efc.mv = efc__mv
  _d.efc.pos = efc__pos
  _d.efc.prev_Mgrad = efc__prev_Mgrad
  _d.efc.prev_cost = efc__prev_cost
  _d.efc.prev_grad = efc__prev_grad
  _d.efc.quad = efc__quad
  _d.efc.quad_gauss = efc__quad_gauss
  _d.efc.search = efc__search
  _d.efc.search_dot = efc__search_dot
  _d.efc.state = efc__state
  _d.efc.type = efc__type
  _d.efc.vel = efc__vel
  _d.energy = energy
  _d.eq_active = eq_active
  _d.flexedge_J = flexedge_J
  _d.flexedge_length = flexedge_length
  _d.flexedge_velocity = flexedge_velocity
  _d.flexvert_xpos = flexvert_xpos
  _d.geom_xmat = geom_xmat
  _d.geom_xpos = geom_xpos
  _d.light_xdir = light_xdir
  _d.light_xpos = light_xpos
  _d.mocap_pos = mocap_pos
  _d.mocap_quat = mocap_quat
  _d.nacon = nacon
  _d.naconmax = naconmax
  _d.ncollision = ncollision
  _d.ne = ne
  _d.ne_connect = ne_connect
  _d.ne_flex = ne_flex
  _d.ne_jnt = ne_jnt
  _d.ne_ten = ne_ten
  _d.ne_weld = ne_weld
  _d.nefc = nefc
  _d.nf = nf
  _d.njmax = njmax
  _d.nl = nl
  _d.nsolving = nsolving
  _d.qLD = qLD
  _d.qLDiagInv = qLDiagInv
  _d.qM = qM
  _d.qacc = qacc
  _d.qacc_smooth = qacc_smooth
  _d.qacc_warmstart = qacc_warmstart
  _d.qfrc_actuator = qfrc_actuator
  _d.qfrc_applied = qfrc_applied
  _d.qfrc_bias = qfrc_bias
  _d.qfrc_constraint = qfrc_constraint
  _d.qfrc_damper = qfrc_damper
  _d.qfrc_fluid = qfrc_fluid
  _d.qfrc_gravcomp = qfrc_gravcomp
  _d.qfrc_passive = qfrc_passive
  _d.qfrc_smooth = qfrc_smooth
  _d.qfrc_spring = qfrc_spring
  _d.qpos = qpos
  _d.qvel = qvel
  _d.sensordata = sensordata
  _d.site_xmat = site_xmat
  _d.site_xpos = site_xpos
  _d.solver_niter = solver_niter
  _d.subtree_angmom = subtree_angmom
  _d.subtree_bodyvel = subtree_bodyvel
  _d.subtree_com = subtree_com
  _d.subtree_linvel = subtree_linvel
  _d.ten_J = ten_J
  _d.ten_length = ten_length
  _d.ten_velocity = ten_velocity
  _d.ten_wrapadr = ten_wrapadr
  _d.ten_wrapnum = ten_wrapnum
  _d.time = time
  _d.wrap_obj = wrap_obj
  _d.wrap_xpos = wrap_xpos
  _d.xanchor = xanchor
  _d.xaxis = xaxis
  _d.xfrc_applied = xfrc_applied
  _d.ximat = ximat
  _d.xipos = xipos
  _d.xmat = xmat
  _d.xpos = xpos
  _d.xquat = xquat
  _d.nworld = nworld
  mjwarp.step(_m, _d)


def _step_jax_impl(m: types.Model, d: types.Data):
  output_dims = {
      'act': d.act.shape,
      'act_dot': d.act_dot.shape,
      'actuator_force': d.actuator_force.shape,
      'actuator_length': d.actuator_length.shape,
      'actuator_moment': d._impl.actuator_moment.shape,
      'actuator_velocity': d._impl.actuator_velocity.shape,
      'cacc': d._impl.cacc.shape,
      'cam_xmat': d.cam_xmat.shape,
      'cam_xpos': d.cam_xpos.shape,
      'cdof': d.cdof.shape,
      'cdof_dot': d.cdof_dot.shape,
      'cfrc_ext': d._impl.cfrc_ext.shape,
      'cfrc_int': d._impl.cfrc_int.shape,
      'cinert': d._impl.cinert.shape,
      'collision_pair': d._impl.collision_pair.shape,
      'collision_pairid': d._impl.collision_pairid.shape,
      'collision_worldid': d._impl.collision_worldid.shape,
      'crb': d._impl.crb.shape,
      'ctrl': d.ctrl.shape,
      'cvel': d.cvel.shape,
      'energy': d._impl.energy.shape,
      'eq_active': d.eq_active.shape,
      'flexedge_J': d._impl.flexedge_J.shape,
      'flexedge_length': d._impl.flexedge_length.shape,
      'flexedge_velocity': d._impl.flexedge_velocity.shape,
      'flexvert_xpos': d._impl.flexvert_xpos.shape,
      'geom_xmat': d.geom_xmat.shape,
      'geom_xpos': d.geom_xpos.shape,
      'light_xdir': d._impl.light_xdir.shape,
      'light_xpos': d._impl.light_xpos.shape,
      'mocap_pos': d.mocap_pos.shape,
      'mocap_quat': d.mocap_quat.shape,
      'nacon': d._impl.nacon.shape,
      'ncollision': d._impl.ncollision.shape,
      'ne': d._impl.ne.shape,
      'ne_connect': d._impl.ne_connect.shape,
      'ne_flex': d._impl.ne_flex.shape,
      'ne_jnt': d._impl.ne_jnt.shape,
      'ne_ten': d._impl.ne_ten.shape,
      'ne_weld': d._impl.ne_weld.shape,
      'nefc': d._impl.nefc.shape,
      'nf': d._impl.nf.shape,
      'nl': d._impl.nl.shape,
      'nsolving': d._impl.nsolving.shape,
      'qLD': d._impl.qLD.shape,
      'qLDiagInv': d._impl.qLDiagInv.shape,
      'qM': d._impl.qM.shape,
      'qacc': d.qacc.shape,
      'qacc_smooth': d.qacc_smooth.shape,
      'qacc_warmstart': d.qacc_warmstart.shape,
      'qfrc_actuator': d.qfrc_actuator.shape,
      'qfrc_applied': d.qfrc_applied.shape,
      'qfrc_bias': d.qfrc_bias.shape,
      'qfrc_constraint': d.qfrc_constraint.shape,
      'qfrc_damper': d._impl.qfrc_damper.shape,
      'qfrc_fluid': d.qfrc_fluid.shape,
      'qfrc_gravcomp': d.qfrc_gravcomp.shape,
      'qfrc_passive': d.qfrc_passive.shape,
      'qfrc_smooth': d.qfrc_smooth.shape,
      'qfrc_spring': d._impl.qfrc_spring.shape,
      'qpos': d.qpos.shape,
      'qvel': d.qvel.shape,
      'sensordata': d.sensordata.shape,
      'site_xmat': d.site_xmat.shape,
      'site_xpos': d.site_xpos.shape,
      'solver_niter': d._impl.solver_niter.shape,
      'subtree_angmom': d._impl.subtree_angmom.shape,
      'subtree_bodyvel': d._impl.subtree_bodyvel.shape,
      'subtree_com': d.subtree_com.shape,
      'subtree_linvel': d._impl.subtree_linvel.shape,
      'ten_J': d._impl.ten_J.shape,
      'ten_length': d.ten_length.shape,
      'ten_velocity': d._impl.ten_velocity.shape,
      'ten_wrapadr': d._impl.ten_wrapadr.shape,
      'ten_wrapnum': d._impl.ten_wrapnum.shape,
      'time': d.time.shape,
      'wrap_obj': d._impl.wrap_obj.shape,
      'wrap_xpos': d._impl.wrap_xpos.shape,
      'xanchor': d.xanchor.shape,
      'xaxis': d.xaxis.shape,
      'xfrc_applied': d.xfrc_applied.shape,
      'ximat': d.ximat.shape,
      'xipos': d.xipos.shape,
      'xmat': d.xmat.shape,
      'xpos': d.xpos.shape,
      'xquat': d.xquat.shape,
      'contact__dim': d._impl.contact__dim.shape,
      'contact__dist': d._impl.contact__dist.shape,
      'contact__efc_address': d._impl.contact__efc_address.shape,
      'contact__frame': d._impl.contact__frame.shape,
      'contact__friction': d._impl.contact__friction.shape,
      'contact__geom': d._impl.contact__geom.shape,
      'contact__geomcollisionid': d._impl.contact__geomcollisionid.shape,
      'contact__includemargin': d._impl.contact__includemargin.shape,
      'contact__pos': d._impl.contact__pos.shape,
      'contact__solimp': d._impl.contact__solimp.shape,
      'contact__solref': d._impl.contact__solref.shape,
      'contact__solreffriction': d._impl.contact__solreffriction.shape,
      'contact__type': d._impl.contact__type.shape,
      'contact__worldid': d._impl.contact__worldid.shape,
      'efc__D': d._impl.efc__D.shape,
      'efc__J': d._impl.efc__J.shape,
      'efc__Jaref': d._impl.efc__Jaref.shape,
      'efc__Ma': d._impl.efc__Ma.shape,
      'efc__Mgrad': d._impl.efc__Mgrad.shape,
      'efc__alpha': d._impl.efc__alpha.shape,
      'efc__aref': d._impl.efc__aref.shape,
      'efc__beta': d._impl.efc__beta.shape,
      'efc__cost': d._impl.efc__cost.shape,
      'efc__done': d._impl.efc__done.shape,
      'efc__force': d._impl.efc__force.shape,
      'efc__frictionloss': d._impl.efc__frictionloss.shape,
      'efc__gauss': d._impl.efc__gauss.shape,
      'efc__grad': d._impl.efc__grad.shape,
      'efc__grad_dot': d._impl.efc__grad_dot.shape,
      'efc__id': d._impl.efc__id.shape,
      'efc__jv': d._impl.efc__jv.shape,
      'efc__margin': d._impl.efc__margin.shape,
      'efc__mv': d._impl.efc__mv.shape,
      'efc__pos': d._impl.efc__pos.shape,
      'efc__prev_Mgrad': d._impl.efc__prev_Mgrad.shape,
      'efc__prev_cost': d._impl.efc__prev_cost.shape,
      'efc__prev_grad': d._impl.efc__prev_grad.shape,
      'efc__quad': d._impl.efc__quad.shape,
      'efc__quad_gauss': d._impl.efc__quad_gauss.shape,
      'efc__search': d._impl.efc__search.shape,
      'efc__search_dot': d._impl.efc__search_dot.shape,
      'efc__state': d._impl.efc__state.shape,
      'efc__type': d._impl.efc__type.shape,
      'efc__vel': d._impl.efc__vel.shape,
  }
  jf = ffi.jax_callable_variadic_tuple(
      _step_shim,
      num_outputs=130,
      output_dims=output_dims,
      vmap_method=None,
      in_out_argnames={
          'act',
          'act_dot',
          'actuator_force',
          'actuator_length',
          'actuator_moment',
          'actuator_velocity',
          'cacc',
          'cam_xmat',
          'cam_xpos',
          'cdof',
          'cdof_dot',
          'cfrc_ext',
          'cfrc_int',
          'cinert',
          'collision_pair',
          'collision_pairid',
          'collision_worldid',
          'crb',
          'ctrl',
          'cvel',
          'energy',
          'eq_active',
          'flexedge_J',
          'flexedge_length',
          'flexedge_velocity',
          'flexvert_xpos',
          'geom_xmat',
          'geom_xpos',
          'light_xdir',
          'light_xpos',
          'mocap_pos',
          'mocap_quat',
          'nacon',
          'ncollision',
          'ne',
          'ne_connect',
          'ne_flex',
          'ne_jnt',
          'ne_ten',
          'ne_weld',
          'nefc',
          'nf',
          'nl',
          'nsolving',
          'qLD',
          'qLDiagInv',
          'qM',
          'qacc',
          'qacc_smooth',
          'qacc_warmstart',
          'qfrc_actuator',
          'qfrc_applied',
          'qfrc_bias',
          'qfrc_constraint',
          'qfrc_damper',
          'qfrc_fluid',
          'qfrc_gravcomp',
          'qfrc_passive',
          'qfrc_smooth',
          'qfrc_spring',
          'qpos',
          'qvel',
          'sensordata',
          'site_xmat',
          'site_xpos',
          'solver_niter',
          'subtree_angmom',
          'subtree_bodyvel',
          'subtree_com',
          'subtree_linvel',
          'ten_J',
          'ten_length',
          'ten_velocity',
          'ten_wrapadr',
          'ten_wrapnum',
          'time',
          'wrap_obj',
          'wrap_xpos',
          'xanchor',
          'xaxis',
          'xfrc_applied',
          'ximat',
          'xipos',
          'xmat',
          'xpos',
          'xquat',
          'contact__dim',
          'contact__dist',
          'contact__efc_address',
          'contact__frame',
          'contact__friction',
          'contact__geom',
          'contact__geomcollisionid',
          'contact__includemargin',
          'contact__pos',
          'contact__solimp',
          'contact__solref',
          'contact__solreffriction',
          'contact__type',
          'contact__worldid',
          'efc__D',
          'efc__J',
          'efc__Jaref',
          'efc__Ma',
          'efc__Mgrad',
          'efc__alpha',
          'efc__aref',
          'efc__beta',
          'efc__cost',
          'efc__done',
          'efc__force',
          'efc__frictionloss',
          'efc__gauss',
          'efc__grad',
          'efc__grad_dot',
          'efc__id',
          'efc__jv',
          'efc__margin',
          'efc__mv',
          'efc__pos',
          'efc__prev_Mgrad',
          'efc__prev_cost',
          'efc__prev_grad',
          'efc__quad',
          'efc__quad_gauss',
          'efc__search',
          'efc__search_dot',
          'efc__state',
          'efc__type',
          'efc__vel',
      },
  )
  out = jf(
      d.qpos.shape[0],
      m._impl.M_rowadr,
      m._impl.M_rownnz,
      m.actuator_acc0,
      m.actuator_actadr,
      m.actuator_actearly,
      m.actuator_actlimited,
      m.actuator_actnum,
      m.actuator_actrange,
      m.actuator_biasprm,
      m.actuator_biastype,
      m.actuator_cranklength,
      m.actuator_ctrllimited,
      m.actuator_ctrlrange,
      m.actuator_dynprm,
      m.actuator_dyntype,
      m.actuator_forcelimited,
      m.actuator_forcerange,
      m.actuator_gainprm,
      m.actuator_gaintype,
      m.actuator_gear,
      m.actuator_lengthrange,
      m.actuator_trnid,
      m.actuator_trntype,
      m._impl.actuator_trntype_body_adr,
      m._impl.block_dim,
      m.body_dofadr,
      m.body_dofnum,
      m._impl.body_fluid_ellipsoid,
      m.body_geomadr,
      m.body_geomnum,
      m.body_gravcomp,
      m.body_inertia,
      m.body_invweight0,
      m.body_ipos,
      m.body_iquat,
      m.body_jntadr,
      m.body_jntnum,
      m.body_mass,
      m.body_mocapid,
      m.body_parentid,
      m.body_pos,
      m.body_quat,
      m.body_rootid,
      m.body_subtreemass,
      m._impl.body_tree,
      m.body_weldid,
      m.cam_bodyid,
      m.cam_fovy,
      m.cam_intrinsic,
      m.cam_mat0,
      m.cam_mode,
      m.cam_pos,
      m.cam_pos0,
      m.cam_poscom0,
      m.cam_quat,
      m.cam_resolution,
      m.cam_sensorsize,
      m.cam_targetbodyid,
      m.dof_Madr,
      m.dof_armature,
      m.dof_bodyid,
      m.dof_damping,
      m.dof_frictionloss,
      m.dof_invweight0,
      m.dof_jntid,
      m.dof_parentid,
      m.dof_solimp,
      m.dof_solref,
      m._impl.dof_tri_col,
      m._impl.dof_tri_row,
      m._impl.eq_connect_adr,
      m.eq_data,
      m._impl.eq_flex_adr,
      m._impl.eq_jnt_adr,
      m.eq_obj1id,
      m.eq_obj2id,
      m.eq_objtype,
      m.eq_solimp,
      m.eq_solref,
      m._impl.eq_ten_adr,
      m._impl.eq_wld_adr,
      m._impl.flex_bending,
      m._impl.flex_damping,
      m._impl.flex_dim,
      m._impl.flex_edge,
      m._impl.flex_edgeadr,
      m._impl.flex_edgeflap,
      m._impl.flex_edgenum,
      m._impl.flex_elem,
      m._impl.flex_elemadr,
      m._impl.flex_elemedge,
      m._impl.flex_elemedgeadr,
      m._impl.flex_elemnum,
      m._impl.flex_stiffness,
      m._impl.flex_vertadr,
      m._impl.flex_vertbodyid,
      m._impl.flexedge_invweight0,
      m._impl.flexedge_length0,
      m.geom_aabb,
      m.geom_bodyid,
      m.geom_condim,
      m.geom_dataid,
      m.geom_fluid,
      m.geom_friction,
      m.geom_gap,
      m.geom_group,
      m.geom_margin,
      m.geom_matid,
      m._impl.geom_pair_type_count,
      m._impl.geom_plugin_index,
      m.geom_pos,
      m.geom_priority,
      m.geom_quat,
      m.geom_rbound,
      m.geom_rgba,
      m.geom_size,
      m.geom_solimp,
      m.geom_solmix,
      m.geom_solref,
      m.geom_type,
      m._impl.has_sdf_geom,
      m.hfield_adr,
      m.hfield_data,
      m.hfield_ncol,
      m.hfield_nrow,
      m.hfield_size,
      m.jnt_actfrclimited,
      m.jnt_actfrcrange,
      m.jnt_actgravcomp,
      m.jnt_axis,
      m.jnt_bodyid,
      m.jnt_dofadr,
      m._impl.jnt_limited_ball_adr,
      m._impl.jnt_limited_slide_hinge_adr,
      m.jnt_margin,
      m.jnt_pos,
      m.jnt_qposadr,
      m.jnt_range,
      m.jnt_solimp,
      m.jnt_solref,
      m.jnt_stiffness,
      m.jnt_type,
      m._impl.light_bodyid,
      m.light_dir,
      m.light_dir0,
      m.light_mode,
      m.light_pos,
      m.light_pos0,
      m.light_poscom0,
      m._impl.light_targetbodyid,
      m._impl.mapM2M,
      m.mat_rgba,
      m.mesh_face,
      m.mesh_faceadr,
      m.mesh_graph,
      m.mesh_graphadr,
      m.mesh_normal,
      m.mesh_normaladr,
      m._impl.mesh_polyadr,
      m._impl.mesh_polymap,
      m._impl.mesh_polymapadr,
      m._impl.mesh_polymapnum,
      m._impl.mesh_polynormal,
      m._impl.mesh_polynum,
      m._impl.mesh_polyvert,
      m._impl.mesh_polyvertadr,
      m._impl.mesh_polyvertnum,
      m.mesh_quat,
      m.mesh_vert,
      m.mesh_vertadr,
      m.mesh_vertnum,
      m.nC,
      m.nM,
      m.na,
      m._impl.nacttrnbody,
      m.nbody,
      m.ncam,
      m.neq,
      m._impl.nflex,
      m._impl.nflexedge,
      m._impl.nflexelem,
      m._impl.nflexvert,
      m.ngeom,
      m.ngravcomp,
      m.njnt,
      m.nlight,
      m._impl.nmaxcondim,
      m._impl.nmaxmeshdeg,
      m._impl.nmaxpolygon,
      m._impl.nmaxpyramid,
      m.nmeshface,
      m._impl.nrangefinder,
      m._impl.nsensorcollision,
      m._impl.nsensorcontact,
      m._impl.nsensortaxel,
      m.nsite,
      m.ntendon,
      m.nu,
      m.nv,
      m._impl.nv_pad,
      m.nwrap,
      m._impl.nxn_geom_pair_filtered,
      m._impl.nxn_pairid,
      m._impl.nxn_pairid_filtered,
      m._impl.oct_aabb,
      m._impl.oct_child,
      m._impl.oct_coeff,
      m.pair_dim,
      m.pair_friction,
      m.pair_gap,
      m.pair_margin,
      m.pair_solimp,
      m.pair_solref,
      m.pair_solreffriction,
      m._impl.plugin,
      m._impl.plugin_attr,
      m._impl.qLD_updates,
      m._impl.qM_fullm_i,
      m._impl.qM_fullm_j,
      m._impl.qM_madr_ij,
      m._impl.qM_mulm_i,
      m._impl.qM_mulm_j,
      m._impl.qM_tiles,
      m.qpos0,
      m.qpos_spring,
      m._impl.rangefinder_sensor_adr,
      m._impl.sensor_acc_adr,
      m.sensor_adr,
      m._impl.sensor_adr_to_contact_adr,
      m._impl.sensor_contact_adr,
      m.sensor_cutoff,
      m.sensor_datatype,
      m.sensor_dim,
      m._impl.sensor_e_kinetic,
      m._impl.sensor_e_potential,
      m.sensor_intprm,
      m._impl.sensor_limitfrc_adr,
      m._impl.sensor_limitpos_adr,
      m._impl.sensor_limitvel_adr,
      m.sensor_objid,
      m.sensor_objtype,
      m._impl.sensor_pos_adr,
      m._impl.sensor_rangefinder_adr,
      m._impl.sensor_rangefinder_bodyid,
      m.sensor_refid,
      m.sensor_reftype,
      m._impl.sensor_rne_postconstraint,
      m._impl.sensor_subtree_vel,
      m._impl.sensor_tendonactfrc_adr,
      m._impl.sensor_touch_adr,
      m.sensor_type,
      m._impl.sensor_vel_adr,
      m.site_bodyid,
      m.site_pos,
      m.site_quat,
      m.site_size,
      m.site_type,
      m._impl.taxel_sensorid,
      m._impl.taxel_vertadr,
      m.tendon_actfrclimited,
      m.tendon_actfrcrange,
      m.tendon_adr,
      m.tendon_armature,
      m.tendon_damping,
      m.tendon_frictionloss,
      m._impl.tendon_geom_adr,
      m.tendon_invweight0,
      m._impl.tendon_jnt_adr,
      m.tendon_length0,
      m.tendon_lengthspring,
      m._impl.tendon_limited_adr,
      m.tendon_margin,
      m.tendon_num,
      m.tendon_range,
      m._impl.tendon_site_pair_adr,
      m.tendon_solimp_fri,
      m.tendon_solimp_lim,
      m.tendon_solref_fri,
      m.tendon_solref_lim,
      m.tendon_stiffness,
      m._impl.wrap_geom_adr,
      m._impl.wrap_jnt_adr,
      m.wrap_objid,
      m.wrap_prm,
      m._impl.wrap_pulley_scale,
      m._impl.wrap_site_pair_adr,
      m.wrap_type,
      m.opt._impl.broadphase,
      m.opt._impl.broadphase_filter,
      m.opt._impl.ccd_iterations,
      m.opt._impl.ccd_tolerance,
      m.opt.cone,
      m.opt._impl.contact_sensor_maxmatch,
      m.opt.density,
      m.opt.disableflags,
      m.opt.enableflags,
      m.opt._impl.graph_conditional,
      m.opt.gravity,
      m.opt._impl.has_fluid,
      m.opt._impl.impratio_invsqrt,
      m.opt.integrator,
      m.opt._impl.is_sparse,
      m.opt.iterations,
      m.opt.ls_iterations,
      m.opt._impl.ls_parallel,
      m.opt._impl.ls_parallel_min_step,
      m.opt.ls_tolerance,
      m.opt.magnetic,
      m.opt._impl.run_collision_detection,
      m.opt._impl.sdf_initpoints,
      m.opt._impl.sdf_iterations,
      m.opt.solver,
      m.opt.timestep,
      m.opt.tolerance,
      m.opt.viscosity,
      m.opt.wind,
      m.stat.meaninertia,
      d._impl.naconmax,
      d._impl.njmax,
      d.act,
      d.act_dot,
      d.actuator_force,
      d.actuator_length,
      d._impl.actuator_moment,
      d._impl.actuator_velocity,
      d._impl.cacc,
      d.cam_xmat,
      d.cam_xpos,
      d.cdof,
      d.cdof_dot,
      d._impl.cfrc_ext,
      d._impl.cfrc_int,
      d._impl.cinert,
      d._impl.collision_pair,
      d._impl.collision_pairid,
      d._impl.collision_worldid,
      d._impl.crb,
      d.ctrl,
      d.cvel,
      d._impl.energy,
      d.eq_active,
      d._impl.flexedge_J,
      d._impl.flexedge_length,
      d._impl.flexedge_velocity,
      d._impl.flexvert_xpos,
      d.geom_xmat,
      d.geom_xpos,
      d._impl.light_xdir,
      d._impl.light_xpos,
      d.mocap_pos,
      d.mocap_quat,
      d._impl.nacon,
      d._impl.ncollision,
      d._impl.ne,
      d._impl.ne_connect,
      d._impl.ne_flex,
      d._impl.ne_jnt,
      d._impl.ne_ten,
      d._impl.ne_weld,
      d._impl.nefc,
      d._impl.nf,
      d._impl.nl,
      d._impl.nsolving,
      d._impl.qLD,
      d._impl.qLDiagInv,
      d._impl.qM,
      d.qacc,
      d.qacc_smooth,
      d.qacc_warmstart,
      d.qfrc_actuator,
      d.qfrc_applied,
      d.qfrc_bias,
      d.qfrc_constraint,
      d._impl.qfrc_damper,
      d.qfrc_fluid,
      d.qfrc_gravcomp,
      d.qfrc_passive,
      d.qfrc_smooth,
      d._impl.qfrc_spring,
      d.qpos,
      d.qvel,
      d.sensordata,
      d.site_xmat,
      d.site_xpos,
      d._impl.solver_niter,
      d._impl.subtree_angmom,
      d._impl.subtree_bodyvel,
      d.subtree_com,
      d._impl.subtree_linvel,
      d._impl.ten_J,
      d.ten_length,
      d._impl.ten_velocity,
      d._impl.ten_wrapadr,
      d._impl.ten_wrapnum,
      d.time,
      d._impl.wrap_obj,
      d._impl.wrap_xpos,
      d.xanchor,
      d.xaxis,
      d.xfrc_applied,
      d.ximat,
      d.xipos,
      d.xmat,
      d.xpos,
      d.xquat,
      d._impl.contact__dim,
      d._impl.contact__dist,
      d._impl.contact__efc_address,
      d._impl.contact__frame,
      d._impl.contact__friction,
      d._impl.contact__geom,
      d._impl.contact__geomcollisionid,
      d._impl.contact__includemargin,
      d._impl.contact__pos,
      d._impl.contact__solimp,
      d._impl.contact__solref,
      d._impl.contact__solreffriction,
      d._impl.contact__type,
      d._impl.contact__worldid,
      d._impl.efc__D,
      d._impl.efc__J,
      d._impl.efc__Jaref,
      d._impl.efc__Ma,
      d._impl.efc__Mgrad,
      d._impl.efc__alpha,
      d._impl.efc__aref,
      d._impl.efc__beta,
      d._impl.efc__cost,
      d._impl.efc__done,
      d._impl.efc__force,
      d._impl.efc__frictionloss,
      d._impl.efc__gauss,
      d._impl.efc__grad,
      d._impl.efc__grad_dot,
      d._impl.efc__id,
      d._impl.efc__jv,
      d._impl.efc__margin,
      d._impl.efc__mv,
      d._impl.efc__pos,
      d._impl.efc__prev_Mgrad,
      d._impl.efc__prev_cost,
      d._impl.efc__prev_grad,
      d._impl.efc__quad,
      d._impl.efc__quad_gauss,
      d._impl.efc__search,
      d._impl.efc__search_dot,
      d._impl.efc__state,
      d._impl.efc__type,
      d._impl.efc__vel,
  )
  d = d.tree_replace({
      'act': out[0],
      'act_dot': out[1],
      'actuator_force': out[2],
      'actuator_length': out[3],
      '_impl.actuator_moment': out[4],
      '_impl.actuator_velocity': out[5],
      '_impl.cacc': out[6],
      'cam_xmat': out[7],
      'cam_xpos': out[8],
      'cdof': out[9],
      'cdof_dot': out[10],
      '_impl.cfrc_ext': out[11],
      '_impl.cfrc_int': out[12],
      '_impl.cinert': out[13],
      '_impl.collision_pair': out[14],
      '_impl.collision_pairid': out[15],
      '_impl.collision_worldid': out[16],
      '_impl.crb': out[17],
      'ctrl': out[18],
      'cvel': out[19],
      '_impl.energy': out[20],
      'eq_active': out[21],
      '_impl.flexedge_J': out[22],
      '_impl.flexedge_length': out[23],
      '_impl.flexedge_velocity': out[24],
      '_impl.flexvert_xpos': out[25],
      'geom_xmat': out[26],
      'geom_xpos': out[27],
      '_impl.light_xdir': out[28],
      '_impl.light_xpos': out[29],
      'mocap_pos': out[30],
      'mocap_quat': out[31],
      '_impl.nacon': out[32],
      '_impl.ncollision': out[33],
      '_impl.ne': out[34],
      '_impl.ne_connect': out[35],
      '_impl.ne_flex': out[36],
      '_impl.ne_jnt': out[37],
      '_impl.ne_ten': out[38],
      '_impl.ne_weld': out[39],
      '_impl.nefc': out[40],
      '_impl.nf': out[41],
      '_impl.nl': out[42],
      '_impl.nsolving': out[43],
      '_impl.qLD': out[44],
      '_impl.qLDiagInv': out[45],
      '_impl.qM': out[46],
      'qacc': out[47],
      'qacc_smooth': out[48],
      'qacc_warmstart': out[49],
      'qfrc_actuator': out[50],
      'qfrc_applied': out[51],
      'qfrc_bias': out[52],
      'qfrc_constraint': out[53],
      '_impl.qfrc_damper': out[54],
      'qfrc_fluid': out[55],
      'qfrc_gravcomp': out[56],
      'qfrc_passive': out[57],
      'qfrc_smooth': out[58],
      '_impl.qfrc_spring': out[59],
      'qpos': out[60],
      'qvel': out[61],
      'sensordata': out[62],
      'site_xmat': out[63],
      'site_xpos': out[64],
      '_impl.solver_niter': out[65],
      '_impl.subtree_angmom': out[66],
      '_impl.subtree_bodyvel': out[67],
      'subtree_com': out[68],
      '_impl.subtree_linvel': out[69],
      '_impl.ten_J': out[70],
      'ten_length': out[71],
      '_impl.ten_velocity': out[72],
      '_impl.ten_wrapadr': out[73],
      '_impl.ten_wrapnum': out[74],
      'time': out[75],
      '_impl.wrap_obj': out[76],
      '_impl.wrap_xpos': out[77],
      'xanchor': out[78],
      'xaxis': out[79],
      'xfrc_applied': out[80],
      'ximat': out[81],
      'xipos': out[82],
      'xmat': out[83],
      'xpos': out[84],
      'xquat': out[85],
      '_impl.contact__dim': out[86],
      '_impl.contact__dist': out[87],
      '_impl.contact__efc_address': out[88],
      '_impl.contact__frame': out[89],
      '_impl.contact__friction': out[90],
      '_impl.contact__geom': out[91],
      '_impl.contact__geomcollisionid': out[92],
      '_impl.contact__includemargin': out[93],
      '_impl.contact__pos': out[94],
      '_impl.contact__solimp': out[95],
      '_impl.contact__solref': out[96],
      '_impl.contact__solreffriction': out[97],
      '_impl.contact__type': out[98],
      '_impl.contact__worldid': out[99],
      '_impl.efc__D': out[100],
      '_impl.efc__J': out[101],
      '_impl.efc__Jaref': out[102],
      '_impl.efc__Ma': out[103],
      '_impl.efc__Mgrad': out[104],
      '_impl.efc__alpha': out[105],
      '_impl.efc__aref': out[106],
      '_impl.efc__beta': out[107],
      '_impl.efc__cost': out[108],
      '_impl.efc__done': out[109],
      '_impl.efc__force': out[110],
      '_impl.efc__frictionloss': out[111],
      '_impl.efc__gauss': out[112],
      '_impl.efc__grad': out[113],
      '_impl.efc__grad_dot': out[114],
      '_impl.efc__id': out[115],
      '_impl.efc__jv': out[116],
      '_impl.efc__margin': out[117],
      '_impl.efc__mv': out[118],
      '_impl.efc__pos': out[119],
      '_impl.efc__prev_Mgrad': out[120],
      '_impl.efc__prev_cost': out[121],
      '_impl.efc__prev_grad': out[122],
      '_impl.efc__quad': out[123],
      '_impl.efc__quad_gauss': out[124],
      '_impl.efc__search': out[125],
      '_impl.efc__search_dot': out[126],
      '_impl.efc__state': out[127],
      '_impl.efc__type': out[128],
      '_impl.efc__vel': out[129],
  })
  return d


@jax.custom_batching.custom_vmap
@ffi.marshal_jax_warp_callable
def step(m: types.Model, d: types.Data):
  return _step_jax_impl(m, d)
@step.def_vmap
@ffi.marshal_custom_vmap
def step_vmap(unused_axis_size, is_batched, m, d):
  d = step(m, d)
  return d, is_batched[1]
