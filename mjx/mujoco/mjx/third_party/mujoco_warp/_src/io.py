# Copyright 2025 The Newton Developers
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

from typing import Any, Optional, Sequence, Union

import mujoco
import numpy as np
import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src import warp_util

# max number of worlds supported
MAX_WORLDS = 2**24

# tolerance override for float32
_TOLERANCE_F32 = 1.0e-6


def _max_meshdegree(mjm: mujoco.MjModel) -> int:
  if mjm.mesh_polyvertnum.size == 0:
    return 4
  return max(3, mjm.mesh_polymapnum.max())


def _max_npolygon(mjm: mujoco.MjModel) -> int:
  if mjm.mesh_polyvertnum.size == 0:
    return 4
  return max(4, mjm.mesh_polyvertnum.max())


def put_model(mjm: mujoco.MjModel) -> types.Model:
  """
  Creates a model on device.

  Args:
    mjm (mujoco.MjModel): The model containing kinematic and dynamic information (host).

  Returns:
    Model: The model containing kinematic and dynamic information (device).
  """
  # check supported features
  for field, field_types, field_str in (
    (mjm.actuator_trntype, types.TrnType, "Actuator transmission type"),
    (mjm.actuator_dyntype, types.DynType, "Actuator dynamics type"),
    (mjm.actuator_gaintype, types.GainType, "Gain type"),
    (mjm.actuator_biastype, types.BiasType, "Bias type"),
    (mjm.eq_type, types.EqType, "Equality constraint types"),
    (mjm.geom_type, types.GeomType, "Geom type"),
    (mjm.sensor_type, types.SensorType, "Sensor types"),
    (mjm.wrap_type, types.WrapType, "Wrap types"),
  ):
    unsupported = ~np.isin(field, list(field_types))
    if unsupported.any():
      raise NotImplementedError(f"{field_str} {field[unsupported]} not supported.")

  plugin_id = []
  plugin_attr = []
  geom_plugin_index = np.full_like(mjm.geom_type, -1)

  if mjm.nplugin > 0:
    if (mjm.body_plugin != -1).any():
      raise NotImplementedError("Body plugins not supported.")
    if (mjm.actuator_plugin != -1).any():
      raise NotImplementedError("Actuator plugins not supported.")
    if (mjm.sensor_plugin != -1).any():
      raise NotImplementedError("Sensor plugins not supported.")
    for i in range(len(mjm.geom_plugin)):
      if mjm.geom_plugin[i] != -1:
        p = mjm.geom_plugin[i]
        geom_plugin_index[i] = len(plugin_id)
        plugin_id.append(mjm.plugin[p])
        start = mjm.plugin_attradr[p]
        end = mjm.plugin_attradr[p + 1] if p + 1 < mjm.nplugin else len(mjm.plugin_attr)
        values = mjm.plugin_attr[start:end]
        attr_values = []
        current = []
        for v in values:
          if v == 0:
            if current:
              s = "".join(chr(int(x)) for x in current)
              attr_values.append(float(s))
              current = []
          else:
            current.append(v)
        # Pad with zeros if less than 3
        attr_values += [0.0] * (3 - len(attr_values))
        plugin_attr.append(attr_values[:3])

  plugin_id = np.array(plugin_id)
  plugin_attr = np.array(plugin_attr)

  if mjm.nflex > 1:
    raise NotImplementedError("Only one flex is unsupported.")

  if ((mjm.flex_contype != 0) | (mjm.flex_conaffinity != 0)).any():
    raise NotImplementedError("Flex collisions are not implemented.")

  # check options
  for opt, opt_types, msg in (
    (mjm.opt.integrator, types.IntegratorType, "Integrator"),
    (mjm.opt.cone, types.ConeType, "Cone"),
    (mjm.opt.solver, types.SolverType, "Solver"),
  ):
    if opt not in set(opt_types):
      raise NotImplementedError(f"{msg} {opt} is unsupported.")

  if mjm.opt.noslip_iterations > 0:
    raise NotImplementedError(f"noslip solver not implemented.")

  if (mjm.opt.viscosity > 0 or mjm.opt.density > 0) and mjm.opt.integrator in (
    mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    mujoco.mjtIntegrator.mjINT_IMPLICIT,
  ):
    raise NotImplementedError(f"Implicit integrators and fluid model not implemented.")

  # TODO(team): remove after _update_gradient for Newton uses tile operations for islands
  nv_max = 60
  if mjm.nv > nv_max and mjm.opt.jacobian == mujoco.mjtJacobian.mjJAC_DENSE:
    raise ValueError(f"Dense is unsupported for nv > {nv_max} (nv = {mjm.nv}).")

  is_sparse = mujoco.mj_isSparse(mjm)

  # calculate some fields that cannot be easily computed inline
  nlsp = mjm.opt.ls_iterations  # TODO(team): how to set nlsp?

  # dof lower triangle row and column indices (used in solver)
  dof_tri_row, dof_tri_col = np.tril_indices(mjm.nv)

  # indices for sparse qM_fullm (used in solver)
  qM_fullm_i, qM_fullm_j = [], []
  for i in range(mjm.nv):
    j = i
    while j > -1:
      qM_fullm_i.append(i)
      qM_fullm_j.append(j)
      j = mjm.dof_parentid[j]

  # indices for sparse qM mul_m (used in support)
  qM_mulm_i, qM_mulm_j, qM_madr_ij = [], [], []
  for i in range(mjm.nv):
    madr_ij, j = mjm.dof_Madr[i], i

    while True:
      madr_ij, j = madr_ij + 1, mjm.dof_parentid[j]
      if j == -1:
        break
      qM_mulm_i.append(i)
      qM_mulm_j.append(j)
      qM_madr_ij.append(madr_ij)

  # body_tree is a list of body ids grouped by tree level
  bodies, body_depth = {}, np.zeros(mjm.nbody, dtype=int) - 1
  for i in range(mjm.nbody):
    body_depth[i] = body_depth[mjm.body_parentid[i]] + 1
    bodies.setdefault(body_depth[i], []).append(i)
  body_tree = tuple(wp.array(bodies[i], dtype=int) for i in sorted(bodies))

  # qLD_updates has dof tree ordering of qLD updates for sparse factor m
  qLD_updates, dof_depth = {}, np.zeros(mjm.nv, dtype=int) - 1

  for k in range(mjm.nv):
    # skip diagonal rows
    if mjm.M_rownnz[k] == 1:
      continue
    dof_depth[k] = dof_depth[mjm.dof_parentid[k]] + 1
    i = mjm.dof_parentid[k]
    diag_k = mjm.M_rowadr[k] + mjm.M_rownnz[k] - 1
    Madr_ki = diag_k - 1
    while i > -1:
      qLD_updates.setdefault(dof_depth[i], []).append((i, k, Madr_ki))
      i = mjm.dof_parentid[i]
      Madr_ki -= 1

  qLD_updates = tuple(wp.array(qLD_updates[i], dtype=wp.vec3i) for i in sorted(qLD_updates))

  # qM_tiles records the block diagonal structure of qM
  tile_corners = [i for i in range(mjm.nv) if mjm.dof_parentid[i] == -1]
  tiles = {}
  for i in range(len(tile_corners)):
    tile_beg = tile_corners[i]
    tile_end = mjm.nv if i == len(tile_corners) - 1 else tile_corners[i + 1]
    tiles.setdefault(tile_end - tile_beg, []).append(tile_beg)

  qM_tiles = tuple(types.TileSet(adr=wp.array(tiles[sz], dtype=int), size=sz) for sz in sorted(tiles.keys()))

  # subtree_mass is a precalculated array used in smooth
  subtree_mass = np.copy(mjm.body_mass)
  # TODO(team): should this be [mjm.nbody - 1, 0) ?
  for i in range(mjm.nbody - 1, -1, -1):
    subtree_mass[mjm.body_parentid[i]] += subtree_mass[i]

  # actuator_moment tiles are grouped by dof size and number of actuators
  tree_id = np.arange(len(tile_corners), dtype=np.int32)
  num_trees = int(np.max(tree_id)) if len(tree_id) > 0 else 0
  bodyid = []
  for i in range(mjm.nu):
    trntype = mjm.actuator_trntype[i]
    if trntype == mujoco.mjtTrn.mjTRN_JOINT or trntype == mujoco.mjtTrn.mjTRN_JOINTINPARENT:
      jntid = mjm.actuator_trnid[i, 0]
      bodyid.append(mjm.jnt_bodyid[jntid])
    elif trntype == mujoco.mjtTrn.mjTRN_TENDON:
      tenid = mjm.actuator_trnid[i, 0]
      adr = mjm.tendon_adr[tenid]
      if mjm.wrap_type[adr] == mujoco.mjtWrap.mjWRAP_JOINT:
        ten_num = mjm.tendon_num[tenid]
        for i in range(ten_num):
          bodyid.append(mjm.jnt_bodyid[mjm.wrap_objid[adr + i]])
      else:
        for i in range(mjm.nv):
          bodyid.append(mjm.dof_bodyid[i])
    elif trntype == mujoco.mjtTrn.mjTRN_BODY:
      pass
    elif trntype == mujoco.mjtTrn.mjTRN_SITE:
      siteid = mjm.actuator_trnid[i, 0]
      bid = mjm.site_bodyid[siteid]
      while bid > 0:
        bodyid.append(bid)
        bid = mjm.body_parentid[bid]
    elif trntype == mujoco.mjtTrn.mjTRN_SLIDERCRANK:
      for i in range(mjm.nv):
        bodyid.append(mjm.dof_bodyid[i])
    else:
      raise NotImplementedError(f"Transmission type {trntype} not implemented.")
  tree = mjm.body_treeid[np.array(bodyid, dtype=int)]
  counts, ids = np.histogram(tree, bins=np.arange(0, num_trees + 2))
  acts_per_tree = dict(zip(ids, counts))

  tiles = {}
  act_beg = 0
  for i in range(len(tile_corners)):
    tile_beg = tile_corners[i]
    tile_end = mjm.nv if i == len(tile_corners) - 1 else tile_corners[i + 1]
    tree = int(tree_id[i])
    act_num = acts_per_tree[tree]
    tiles.setdefault((tile_end - tile_beg, act_num), []).append((tile_beg, act_beg))
    act_beg += act_num

  actuator_moment_tiles_nv, actuator_moment_tiles_nu = tuple(), tuple()

  for (nv, nu), adr in sorted(tiles.items()):
    adr_nv = wp.array([nv for nv, _ in adr], dtype=int)
    adr_nu = wp.array([nu for _, nu in adr], dtype=int)
    actuator_moment_tiles_nv += (types.TileSet(adr=adr_nv, size=nv),)
    actuator_moment_tiles_nu += (types.TileSet(adr=adr_nu, size=nu),)

  # fixed tendon
  tendon_jnt_adr = []
  wrap_jnt_adr = []
  for i in range(mjm.ntendon):
    adr = mjm.tendon_adr[i]
    if mjm.wrap_type[adr] == mujoco.mjtWrap.mjWRAP_JOINT:
      tendon_num = mjm.tendon_num[i]
      for j in range(tendon_num):
        tendon_jnt_adr.append(i)
        wrap_jnt_adr.append(adr + j)

  # spatial tendon
  tendon_site_pair_adr = []
  tendon_geom_adr = []

  ten_wrapadr_site = [0]
  ten_wrapnum_site = []
  for i, tendon_num in enumerate(mjm.tendon_num):
    adr = mjm.tendon_adr[i]
    # sites
    if (mjm.wrap_type[adr : adr + tendon_num] == mujoco.mjtWrap.mjWRAP_SITE).all():
      if i < mjm.ntendon:
        ten_wrapadr_site.append(ten_wrapadr_site[-1] + tendon_num)
      ten_wrapnum_site.append(tendon_num)
    else:
      if i < mjm.ntendon:
        ten_wrapadr_site.append(ten_wrapadr_site[-1])
      ten_wrapnum_site.append(0)

    # geoms
    for j in range(tendon_num):
      wrap_type = mjm.wrap_type[adr + j]
      if j < tendon_num - 1:
        next_wrap_type = mjm.wrap_type[adr + j + 1]
        if wrap_type == mujoco.mjtWrap.mjWRAP_SITE and next_wrap_type == mujoco.mjtWrap.mjWRAP_SITE:
          tendon_site_pair_adr.append(i)
      if wrap_type == mujoco.mjtWrap.mjWRAP_SPHERE or wrap_type == mujoco.mjtWrap.mjWRAP_CYLINDER:
        tendon_geom_adr.append(i)

  wrap_site_adr = np.nonzero(mjm.wrap_type == mujoco.mjtWrap.mjWRAP_SITE)[0]
  wrap_site_pair_adr = np.setdiff1d(wrap_site_adr[np.nonzero(np.diff(wrap_site_adr) == 1)[0]], mjm.tendon_adr[1:] - 1)
  wrap_geom_adr = np.nonzero(np.isin(mjm.wrap_type, [mujoco.mjtWrap.mjWRAP_SPHERE, mujoco.mjtWrap.mjWRAP_CYLINDER]))[0]

  # pulley scaling
  wrap_pulley_scale = np.ones(mjm.nwrap, dtype=float)
  pulley_adr = np.nonzero(mjm.wrap_type == mujoco.mjtWrap.mjWRAP_PULLEY)[0]
  for tadr, tnum in zip(mjm.tendon_adr, mjm.tendon_num):
    for padr in pulley_adr:
      if tadr <= padr < tadr + tnum:
        wrap_pulley_scale[padr : tadr + tnum] = 1.0 / mjm.wrap_prm[padr]

  # mocap
  mocap_bodyid = np.arange(mjm.nbody)[mjm.body_mocapid >= 0]
  mocap_bodyid = mocap_bodyid[mjm.body_mocapid[mjm.body_mocapid >= 0].argsort()]

  # precalculated geom pairs
  filterparent = not (mjm.opt.disableflags & types.DisableBit.FILTERPARENT)

  geom1, geom2 = np.triu_indices(mjm.ngeom, k=1)
  nxn_geom_pair = np.stack((geom1, geom2), axis=1)

  bodyid1 = mjm.geom_bodyid[geom1]
  bodyid2 = mjm.geom_bodyid[geom2]
  contype1 = mjm.geom_contype[geom1]
  contype2 = mjm.geom_contype[geom2]
  conaffinity1 = mjm.geom_conaffinity[geom1]
  conaffinity2 = mjm.geom_conaffinity[geom2]
  weldid1 = mjm.body_weldid[bodyid1]
  weldid2 = mjm.body_weldid[bodyid2]
  weld_parentid1 = mjm.body_weldid[mjm.body_parentid[weldid1]]
  weld_parentid2 = mjm.body_weldid[mjm.body_parentid[weldid2]]

  self_collision = weldid1 == weldid2
  parent_child_collision = (
    filterparent & (weldid1 != 0) & (weldid2 != 0) & ((weldid1 == weld_parentid2) | (weldid2 == weld_parentid1))
  )
  mask = np.array((contype1 & conaffinity2) | (contype2 & conaffinity1), dtype=bool)
  exclude = np.isin((bodyid1 << 16) + bodyid2, mjm.exclude_signature)

  nxn_pairid = -1 * np.ones(len(geom1), dtype=int)
  nxn_pairid[~(mask & ~self_collision & ~parent_child_collision & ~exclude)] = -2

  # contact pairs
  for i in range(mjm.npair):
    pair_geom1 = mjm.pair_geom1[i]
    pair_geom2 = mjm.pair_geom2[i]

    if pair_geom2 < pair_geom1:
      pairid = np.int32(math.upper_tri_index(mjm.ngeom, int(pair_geom2), int(pair_geom1)))
    else:
      pairid = np.int32(math.upper_tri_index(mjm.ngeom, int(pair_geom1), int(pair_geom2)))

    nxn_pairid[pairid] = i

  include = nxn_pairid > -2
  nxn_pairid_filtered = nxn_pairid[include]
  nxn_geom_pair_filtered = nxn_geom_pair[include]

  # count contact pair types
  geom_type_pair_count = np.bincount(
    [
      math.upper_trid_index(len(types.GeomType), int(mjm.geom_type[geom1[i]]), int(mjm.geom_type[geom2[i]]))
      for i in np.arange(len(geom1))
      if nxn_pairid[i] > -2
    ],
    minlength=len(types.GeomType) * (len(types.GeomType) + 1) // 2,
  )

  # Disable collisions if there are no potentially colliding pairs
  if np.sum(geom_type_pair_count) == 0:
    mjm.opt.disableflags |= types.DisableBit.CONTACT

  def create_nmodel_batched_array(mjm_array, dtype, expand_dim=True):
    array = wp.array(mjm_array, dtype=dtype)
    # add private attribute for JAX to determine which fields are batched
    array._is_batched = True
    if not expand_dim:
      array.strides = (0,) + array.strides[1:]
      array.shape = (MAX_WORLDS,) + array.shape[1:]
      return array
    array.strides = (0,) + array.strides
    array.ndim += 1
    array.shape = (MAX_WORLDS,) + array.shape
    return array

  # rangefinder
  is_rangefinder = mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER
  sensor_rangefinder_adr = np.nonzero(is_rangefinder)[0]
  rangefinder_sensor_adr = np.full(mjm.nsensor, -1)
  rangefinder_sensor_adr[sensor_rangefinder_adr] = np.arange(len(sensor_rangefinder_adr))

  # contact sensor
  sensor_adr_to_contact_adr = np.clip(np.cumsum(mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT) - 1, a_min=0, a_max=None)

  if nxn_geom_pair_filtered.shape[0] < 250_000:
    broadphase = types.BroadphaseType.NXN
  elif mjm.ngeom < 1000:
    broadphase = types.BroadphaseType.SAP_TILE
  else:
    broadphase = types.BroadphaseType.SAP_SEGMENTED

  condim = np.concatenate((mjm.geom_condim, mjm.pair_dim))
  condim_max = np.max(condim) if len(condim) > 0 else 0

  # collision sensors
  is_collision_sensor = np.isin(
    mjm.sensor_type, [mujoco.mjtSensor.mjSENS_GEOMDIST, mujoco.mjtSensor.mjSENS_GEOMNORMAL, mujoco.mjtSensor.mjSENS_GEOMFROMTO]
  )
  sensor_collision_adr = np.nonzero(is_collision_sensor)[0]
  collision_sensor_adr = np.full(mjm.nsensor, -1)
  collision_sensor_adr[sensor_collision_adr] = np.arange(len(sensor_collision_adr))

  if is_collision_sensor.any():

    def _collision_sensor_check(sensor_type, sensor_id, geom_type, err_msg):
      for type_, id_ in zip(sensor_type, sensor_id):
        if type_ == mujoco.mjtObj.mjOBJ_BODY:
          geomnum = mjm.body_geomnum[id_]
          geomadr = mjm.body_geomadr[id_]
          for geomid in range(geomadr, geomadr + geomnum):
            if mjm.geom_type[geomid] == geom_type:
              raise NotImplementedError(err_msg)
        elif type_ == mujoco.mjtObj.mjOBJ_GEOM:
          if mjm.geom_type[id_] == geom_type:
            raise NotImplementedError(err_msg)

    sensor_collision_objtype = mjm.sensor_objtype[is_collision_sensor]
    sensor_collision_objid = mjm.sensor_objid[is_collision_sensor]
    sensor_collision_reftype = mjm.sensor_reftype[is_collision_sensor]
    sensor_collision_refid = mjm.sensor_refid[is_collision_sensor]

    _collision_sensor_check(
      sensor_collision_objtype,
      sensor_collision_objid,
      mujoco.mjtGeom.mjGEOM_PLANE,
      "Collision sensors with planes are not implemented.",
    )
    _collision_sensor_check(
      sensor_collision_reftype,
      sensor_collision_refid,
      mujoco.mjtGeom.mjGEOM_PLANE,
      "Collision sensors with planes are not implemented.",
    )
    _collision_sensor_check(
      sensor_collision_objtype,
      sensor_collision_objid,
      mujoco.mjtGeom.mjGEOM_HFIELD,
      "Collision sensors with height fields are not implemented.",
    )
    _collision_sensor_check(
      sensor_collision_reftype,
      sensor_collision_refid,
      mujoco.mjtGeom.mjGEOM_HFIELD,
      "Collision sensors with height fields are not implemented.",
    )

  if mjm.geom_fluid.size:
    geom_fluid_params = mjm.geom_fluid.reshape(mjm.ngeom, mujoco.mjNFLUID)
  else:
    geom_fluid_params = np.zeros((mjm.ngeom, mujoco.mjNFLUID))

  body_fluid_ellipsoid = np.zeros(mjm.nbody, dtype=bool)
  if mjm.ngeom:
    active_geom = geom_fluid_params[:, 0] > 0
    if np.any(active_geom):
      body_fluid_ellipsoid[mjm.geom_bodyid[active_geom]] = True

  m = types.Model(
    nq=mjm.nq,
    nv=mjm.nv,
    nu=mjm.nu,
    na=mjm.na,
    nbody=mjm.nbody,
    njnt=mjm.njnt,
    ngeom=mjm.ngeom,
    nsite=mjm.nsite,
    ncam=mjm.ncam,
    nlight=mjm.nlight,
    nmat=mjm.nmat,
    nflex=mjm.nflex,
    nflexvert=mjm.nflexvert,
    nflexedge=mjm.nflexedge,
    nflexelem=mjm.nflexelem,
    nflexelemdata=mjm.nflexelemdata,
    nexclude=mjm.nexclude,
    neq=mjm.neq,
    nmocap=mjm.nmocap,
    ngravcomp=mjm.ngravcomp,
    nM=mjm.nM,
    nC=mjm.nC,
    ntendon=mjm.ntendon,
    nwrap=mjm.nwrap,
    nsensor=mjm.nsensor,
    nsensordata=mjm.nsensordata,
    nsensortaxel=sum(mjm.mesh_vertnum[mjm.sensor_objid[mjm.sensor_type == mujoco.mjtSensor.mjSENS_TACTILE]]),
    nmeshvert=mjm.nmeshvert,
    nmeshface=mjm.nmeshface,
    nmeshgraph=mjm.nmeshgraph,
    nmeshpoly=mjm.nmeshpoly,
    nmeshpolyvert=mjm.nmeshpolyvert,
    nmeshpolymap=mjm.nmeshpolymap,
    nlsp=nlsp,
    npair=mjm.npair,
    opt=types.Option(
      timestep=create_nmodel_batched_array(np.array(mjm.opt.timestep), dtype=float, expand_dim=False),
      tolerance=create_nmodel_batched_array(
        np.array(np.maximum(mjm.opt.tolerance, _TOLERANCE_F32)), dtype=float, expand_dim=False
      ),
      ls_tolerance=create_nmodel_batched_array(np.array(mjm.opt.ls_tolerance), dtype=float, expand_dim=False),
      ccd_tolerance=create_nmodel_batched_array(np.array(mjm.opt.ccd_tolerance), dtype=float, expand_dim=False),
      gravity=create_nmodel_batched_array(mjm.opt.gravity, dtype=wp.vec3, expand_dim=False),
      magnetic=create_nmodel_batched_array(mjm.opt.magnetic, dtype=wp.vec3, expand_dim=False),
      wind=create_nmodel_batched_array(mjm.opt.wind, dtype=wp.vec3, expand_dim=False),
      has_fluid=bool(mjm.opt.wind.any() or mjm.opt.density or mjm.opt.viscosity),
      density=create_nmodel_batched_array(np.array(mjm.opt.density), dtype=float, expand_dim=False),
      viscosity=create_nmodel_batched_array(np.array(mjm.opt.viscosity), dtype=float, expand_dim=False),
      cone=mjm.opt.cone,
      solver=mjm.opt.solver,
      iterations=mjm.opt.iterations,
      ls_iterations=mjm.opt.ls_iterations,
      integrator=mjm.opt.integrator,
      disableflags=mjm.opt.disableflags,
      enableflags=mjm.opt.enableflags,
      impratio=create_nmodel_batched_array(np.array(mjm.opt.impratio), dtype=float, expand_dim=False),
      is_sparse=bool(is_sparse),
      ls_parallel=False,
      ls_parallel_min_step=1.0e-6,  # TODO(team): determine good default setting
      ccd_iterations=mjm.opt.ccd_iterations,
      broadphase=int(broadphase),
      broadphase_filter=int(types.BroadphaseFilter.PLANE | types.BroadphaseFilter.SPHERE | types.BroadphaseFilter.OBB),
      graph_conditional=True and warp_util.conditional_graph_supported(),
      sdf_initpoints=mjm.opt.sdf_initpoints,
      sdf_iterations=mjm.opt.sdf_iterations,
      run_collision_detection=True,
      legacy_gjk=False,
      contact_sensor_maxmatch=64,
    ),
    stat=types.Statistic(
      meaninertia=mjm.stat.meaninertia,
    ),
    qpos0=create_nmodel_batched_array(mjm.qpos0, dtype=float),
    qpos_spring=create_nmodel_batched_array(mjm.qpos_spring, dtype=float),
    qM_fullm_i=wp.array(qM_fullm_i, dtype=int),
    qM_fullm_j=wp.array(qM_fullm_j, dtype=int),
    qM_mulm_i=wp.array(qM_mulm_i, dtype=int),
    qM_mulm_j=wp.array(qM_mulm_j, dtype=int),
    qM_madr_ij=wp.array(qM_madr_ij, dtype=int),
    qLD_updates=qLD_updates,
    M_rownnz=wp.array(mjm.M_rownnz, dtype=int),
    M_rowadr=wp.array(mjm.M_rowadr, dtype=int),
    M_colind=wp.array(mjm.M_colind, dtype=int),
    mapM2M=wp.array(mjm.mapM2M, dtype=int),
    qM_tiles=qM_tiles,
    body_tree=body_tree,
    body_parentid=wp.array(mjm.body_parentid, dtype=int),
    body_rootid=wp.array(mjm.body_rootid, dtype=int),
    body_weldid=wp.array(mjm.body_weldid, dtype=int),
    body_mocapid=wp.array(mjm.body_mocapid, dtype=int),
    mocap_bodyid=wp.array(mocap_bodyid, dtype=int),
    body_jntnum=wp.array(mjm.body_jntnum, dtype=int),
    body_jntadr=wp.array(mjm.body_jntadr, dtype=int),
    body_dofnum=wp.array(mjm.body_dofnum, dtype=int),
    body_dofadr=wp.array(mjm.body_dofadr, dtype=int),
    body_geomnum=wp.array(mjm.body_geomnum, dtype=int),
    body_geomadr=wp.array(mjm.body_geomadr, dtype=int),
    body_pos=create_nmodel_batched_array(mjm.body_pos, dtype=wp.vec3),
    body_quat=create_nmodel_batched_array(mjm.body_quat, dtype=wp.quat),
    body_ipos=create_nmodel_batched_array(mjm.body_ipos, dtype=wp.vec3),
    body_iquat=create_nmodel_batched_array(mjm.body_iquat, dtype=wp.quat),
    body_mass=create_nmodel_batched_array(mjm.body_mass, dtype=float),
    body_subtreemass=create_nmodel_batched_array(mjm.body_subtreemass, dtype=float),
    subtree_mass=create_nmodel_batched_array(subtree_mass, dtype=float),
    body_inertia=create_nmodel_batched_array(mjm.body_inertia, dtype=wp.vec3),
    body_invweight0=create_nmodel_batched_array(mjm.body_invweight0, dtype=wp.vec2),
    body_contype=wp.array(mjm.body_contype, dtype=int),
    body_conaffinity=wp.array(mjm.body_conaffinity, dtype=int),
    body_gravcomp=create_nmodel_batched_array(mjm.body_gravcomp, dtype=float),
    body_fluid_ellipsoid=wp.array(body_fluid_ellipsoid, dtype=bool),
    jnt_type=wp.array(mjm.jnt_type, dtype=int),
    jnt_qposadr=wp.array(mjm.jnt_qposadr, dtype=int),
    jnt_dofadr=wp.array(mjm.jnt_dofadr, dtype=int),
    jnt_bodyid=wp.array(mjm.jnt_bodyid, dtype=int),
    jnt_limited=wp.array(mjm.jnt_limited, dtype=int),
    jnt_actfrclimited=wp.array(mjm.jnt_actfrclimited, dtype=bool),
    jnt_solref=create_nmodel_batched_array(mjm.jnt_solref, dtype=wp.vec2),
    jnt_solimp=create_nmodel_batched_array(mjm.jnt_solimp, dtype=types.vec5),
    jnt_pos=create_nmodel_batched_array(mjm.jnt_pos, dtype=wp.vec3),
    jnt_axis=create_nmodel_batched_array(mjm.jnt_axis, dtype=wp.vec3),
    jnt_stiffness=create_nmodel_batched_array(mjm.jnt_stiffness, dtype=float),
    jnt_range=create_nmodel_batched_array(mjm.jnt_range, dtype=wp.vec2),
    jnt_actfrcrange=create_nmodel_batched_array(mjm.jnt_actfrcrange, dtype=wp.vec2),
    jnt_margin=create_nmodel_batched_array(mjm.jnt_margin, dtype=float),
    # these jnt_limited adrs are used in constraint.py
    jnt_limited_slide_hinge_adr=wp.array(
      np.nonzero(
        mjm.jnt_limited & ((mjm.jnt_type == mujoco.mjtJoint.mjJNT_SLIDE) | (mjm.jnt_type == mujoco.mjtJoint.mjJNT_HINGE))
      )[0],
      dtype=int,
    ),
    jnt_limited_ball_adr=wp.array(
      np.nonzero(mjm.jnt_limited & (mjm.jnt_type == mujoco.mjtJoint.mjJNT_BALL))[0],
      dtype=int,
    ),
    jnt_actgravcomp=wp.array(mjm.jnt_actgravcomp, dtype=int),
    dof_bodyid=wp.array(mjm.dof_bodyid, dtype=int),
    dof_jntid=wp.array(mjm.dof_jntid, dtype=int),
    dof_parentid=wp.array(mjm.dof_parentid, dtype=int),
    dof_Madr=wp.array(mjm.dof_Madr, dtype=int),
    dof_armature=create_nmodel_batched_array(mjm.dof_armature, dtype=float),
    dof_damping=create_nmodel_batched_array(mjm.dof_damping, dtype=float),
    dof_invweight0=create_nmodel_batched_array(mjm.dof_invweight0, dtype=float),
    dof_frictionloss=create_nmodel_batched_array(mjm.dof_frictionloss, dtype=float),
    dof_solimp=create_nmodel_batched_array(mjm.dof_solimp, dtype=types.vec5),
    dof_solref=create_nmodel_batched_array(mjm.dof_solref, dtype=wp.vec2),
    dof_tri_row=wp.array(dof_tri_row, dtype=int),
    dof_tri_col=wp.array(dof_tri_col, dtype=int),
    geom_type=wp.array(mjm.geom_type, dtype=int),
    geom_contype=wp.array(mjm.geom_contype, dtype=int),
    geom_conaffinity=wp.array(mjm.geom_conaffinity, dtype=int),
    geom_condim=wp.array(mjm.geom_condim, dtype=int),
    geom_bodyid=wp.array(mjm.geom_bodyid, dtype=int),
    geom_dataid=wp.array(mjm.geom_dataid, dtype=int),
    geom_group=wp.array(mjm.geom_group, dtype=int),
    geom_matid=create_nmodel_batched_array(mjm.geom_matid, dtype=int),
    geom_priority=wp.array(mjm.geom_priority, dtype=int),
    geom_solmix=create_nmodel_batched_array(mjm.geom_solmix, dtype=float),
    geom_solref=create_nmodel_batched_array(mjm.geom_solref, dtype=wp.vec2),
    geom_solimp=create_nmodel_batched_array(mjm.geom_solimp, dtype=types.vec5),
    geom_size=create_nmodel_batched_array(mjm.geom_size, dtype=wp.vec3),
    geom_fluid=wp.array(geom_fluid_params, dtype=float),
    geom_aabb=wp.array2d(mjm.geom_aabb, dtype=wp.vec3),
    geom_rbound=create_nmodel_batched_array(mjm.geom_rbound, dtype=float),
    geom_pos=create_nmodel_batched_array(mjm.geom_pos, dtype=wp.vec3),
    geom_quat=create_nmodel_batched_array(mjm.geom_quat, dtype=wp.quat),
    geom_friction=create_nmodel_batched_array(mjm.geom_friction, dtype=wp.vec3),
    geom_margin=create_nmodel_batched_array(mjm.geom_margin, dtype=float),
    geom_gap=create_nmodel_batched_array(mjm.geom_gap, dtype=float),
    geom_rgba=create_nmodel_batched_array(mjm.geom_rgba, dtype=wp.vec4),
    site_type=wp.array(mjm.site_type, dtype=int),
    site_bodyid=wp.array(mjm.site_bodyid, dtype=int),
    site_size=wp.array(mjm.site_size, dtype=wp.vec3),
    site_pos=create_nmodel_batched_array(mjm.site_pos, dtype=wp.vec3),
    site_quat=create_nmodel_batched_array(mjm.site_quat, dtype=wp.quat),
    cam_mode=wp.array(mjm.cam_mode, dtype=int),
    cam_bodyid=wp.array(mjm.cam_bodyid, dtype=int),
    cam_targetbodyid=wp.array(mjm.cam_targetbodyid, dtype=int),
    cam_pos=create_nmodel_batched_array(mjm.cam_pos, dtype=wp.vec3),
    cam_quat=create_nmodel_batched_array(mjm.cam_quat, dtype=wp.quat),
    cam_poscom0=create_nmodel_batched_array(mjm.cam_poscom0, dtype=wp.vec3),
    cam_pos0=create_nmodel_batched_array(mjm.cam_pos0, dtype=wp.vec3),
    cam_mat0=create_nmodel_batched_array(mjm.cam_mat0, dtype=wp.mat33),
    cam_fovy=wp.array(mjm.cam_fovy, dtype=float),
    cam_resolution=wp.array(mjm.cam_resolution, dtype=wp.vec2i),
    cam_sensorsize=wp.array(mjm.cam_sensorsize, dtype=wp.vec2),
    cam_intrinsic=wp.array(mjm.cam_intrinsic, dtype=wp.vec4),
    light_mode=wp.array(mjm.light_mode, dtype=int),
    light_bodyid=wp.array(mjm.light_bodyid, dtype=int),
    light_targetbodyid=wp.array(mjm.light_targetbodyid, dtype=int),
    light_type=create_nmodel_batched_array(mjm.light_type, dtype=int),
    light_castshadow=create_nmodel_batched_array(mjm.light_castshadow, dtype=bool),
    light_active=create_nmodel_batched_array(mjm.light_active, dtype=bool),
    light_pos=create_nmodel_batched_array(mjm.light_pos, dtype=wp.vec3),
    light_dir=create_nmodel_batched_array(mjm.light_dir, dtype=wp.vec3),
    light_poscom0=create_nmodel_batched_array(mjm.light_poscom0, dtype=wp.vec3),
    light_pos0=create_nmodel_batched_array(mjm.light_pos0, dtype=wp.vec3),
    light_dir0=create_nmodel_batched_array(mjm.light_dir0, dtype=wp.vec3),
    flex_dim=wp.array(mjm.flex_dim, dtype=int),
    flex_vertadr=wp.array(mjm.flex_vertadr, dtype=int),
    flex_vertnum=wp.array(mjm.flex_vertnum, dtype=int),
    flex_edgeadr=wp.array(mjm.flex_edgeadr, dtype=int),
    flex_elemedgeadr=wp.array(mjm.flex_elemedgeadr, dtype=int),
    flex_vertbodyid=wp.array(mjm.flex_vertbodyid, dtype=int),
    flex_edge=wp.array(mjm.flex_edge, dtype=wp.vec2i),
    flex_edgeflap=wp.array(mjm.flex_edgeflap, dtype=wp.vec2i),
    flex_elem=wp.array(mjm.flex_elem, dtype=int),
    flex_elemedge=wp.array(mjm.flex_elemedge, dtype=int),
    flexedge_length0=wp.array(mjm.flexedge_length0, dtype=float),
    flex_stiffness=wp.array(mjm.flex_stiffness.flatten(), dtype=float),
    flex_bending=wp.array(mjm.flex_bending.flatten(), dtype=float),
    flex_damping=wp.array(mjm.flex_damping, dtype=float),
    mesh_vertadr=wp.array(mjm.mesh_vertadr, dtype=int),
    mesh_vertnum=wp.array(mjm.mesh_vertnum, dtype=int),
    mesh_vert=wp.array(mjm.mesh_vert, dtype=wp.vec3),
    mesh_normaladr=wp.array(mjm.mesh_normaladr, dtype=int),
    mesh_normal=wp.array(mjm.mesh_normal, dtype=wp.vec3),
    mesh_faceadr=wp.array(mjm.mesh_faceadr, dtype=int),
    mesh_face=wp.array(mjm.mesh_face, dtype=wp.vec3i),
    mesh_graphadr=wp.array(mjm.mesh_graphadr, dtype=int),
    mesh_graph=wp.array(mjm.mesh_graph, dtype=int),
    mesh_quat=wp.array(mjm.mesh_quat, dtype=wp.quat),
    mesh_polynum=wp.array(mjm.mesh_polynum, dtype=int),
    mesh_polyadr=wp.array(mjm.mesh_polyadr, dtype=int),
    mesh_polynormal=wp.array(mjm.mesh_polynormal, dtype=wp.vec3),
    mesh_polyvertadr=wp.array(mjm.mesh_polyvertadr, dtype=int),
    mesh_polyvertnum=wp.array(mjm.mesh_polyvertnum, dtype=int),
    mesh_polyvert=wp.array(mjm.mesh_polyvert, dtype=int),
    mesh_polymapadr=wp.array(mjm.mesh_polymapadr, dtype=int),
    mesh_polymapnum=wp.array(mjm.mesh_polymapnum, dtype=int),
    mesh_polymap=wp.array(mjm.mesh_polymap, dtype=int),
    oct_aabb=wp.array2d(mjm.oct_aabb, dtype=wp.vec3),
    oct_child=wp.array(mjm.oct_child, dtype=types.vec8i),
    oct_coeff=wp.array(mjm.oct_coeff, dtype=types.vec8f),
    nhfield=mjm.nhfield,
    nhfielddata=mjm.nhfielddata,
    hfield_adr=wp.array(mjm.hfield_adr, dtype=int),
    hfield_nrow=wp.array(mjm.hfield_nrow, dtype=int),
    hfield_ncol=wp.array(mjm.hfield_ncol, dtype=int),
    hfield_size=wp.array(mjm.hfield_size, dtype=wp.vec4),
    hfield_data=wp.array(mjm.hfield_data, dtype=float),
    eq_type=wp.array(mjm.eq_type, dtype=int),
    eq_obj1id=wp.array(mjm.eq_obj1id, dtype=int),
    eq_obj2id=wp.array(mjm.eq_obj2id, dtype=int),
    eq_objtype=wp.array(mjm.eq_objtype, dtype=int),
    eq_active0=wp.array(mjm.eq_active0, dtype=bool),
    eq_solref=create_nmodel_batched_array(mjm.eq_solref, dtype=wp.vec2),
    eq_solimp=create_nmodel_batched_array(mjm.eq_solimp, dtype=types.vec5),
    eq_data=create_nmodel_batched_array(mjm.eq_data, dtype=types.vec11),
    # pre-compute indices of equality constraints
    eq_connect_adr=wp.array(np.nonzero(mjm.eq_type == types.EqType.CONNECT)[0], dtype=int),
    eq_wld_adr=wp.array(np.nonzero(mjm.eq_type == types.EqType.WELD)[0], dtype=int),
    eq_jnt_adr=wp.array(np.nonzero(mjm.eq_type == types.EqType.JOINT)[0], dtype=int),
    eq_ten_adr=wp.array(np.nonzero(mjm.eq_type == types.EqType.TENDON)[0], dtype=int),
    actuator_moment_tiles_nv=actuator_moment_tiles_nv,
    actuator_moment_tiles_nu=actuator_moment_tiles_nu,
    actuator_trntype=wp.array(mjm.actuator_trntype, dtype=int),
    actuator_dyntype=wp.array(mjm.actuator_dyntype, dtype=int),
    actuator_gaintype=wp.array(mjm.actuator_gaintype, dtype=int),
    actuator_biastype=wp.array(mjm.actuator_biastype, dtype=int),
    actuator_trnid=wp.array(mjm.actuator_trnid, dtype=wp.vec2i),
    actuator_actadr=wp.array(mjm.actuator_actadr, dtype=int),
    actuator_actnum=wp.array(mjm.actuator_actnum, dtype=int),
    actuator_ctrllimited=wp.array(mjm.actuator_ctrllimited, dtype=bool),
    actuator_forcelimited=wp.array(mjm.actuator_forcelimited, dtype=bool),
    actuator_actlimited=wp.array(mjm.actuator_actlimited, dtype=bool),
    actuator_dynprm=create_nmodel_batched_array(mjm.actuator_dynprm, dtype=types.vec10f),
    actuator_gainprm=create_nmodel_batched_array(mjm.actuator_gainprm, dtype=types.vec10f),
    actuator_biasprm=create_nmodel_batched_array(mjm.actuator_biasprm, dtype=types.vec10f),
    actuator_actearly=wp.array(mjm.actuator_actearly, dtype=bool),
    actuator_ctrlrange=create_nmodel_batched_array(mjm.actuator_ctrlrange, dtype=wp.vec2),
    actuator_forcerange=create_nmodel_batched_array(mjm.actuator_forcerange, dtype=wp.vec2),
    actuator_actrange=create_nmodel_batched_array(mjm.actuator_actrange, dtype=wp.vec2),
    actuator_gear=create_nmodel_batched_array(mjm.actuator_gear, dtype=wp.spatial_vector),
    actuator_cranklength=wp.array(mjm.actuator_cranklength, dtype=float),
    actuator_acc0=wp.array(mjm.actuator_acc0, dtype=float),
    actuator_lengthrange=wp.array(mjm.actuator_lengthrange, dtype=wp.vec2),
    exclude_signature=wp.array(mjm.exclude_signature, dtype=int),
    nxn_geom_pair=wp.array(nxn_geom_pair, dtype=wp.vec2i),
    nxn_geom_pair_filtered=wp.array(nxn_geom_pair_filtered, dtype=wp.vec2i),
    nxn_pairid=wp.array(nxn_pairid, dtype=int),
    nxn_pairid_filtered=wp.array(nxn_pairid_filtered, dtype=int),
    pair_dim=wp.array(mjm.pair_dim, dtype=int),
    pair_geom1=wp.array(mjm.pair_geom1, dtype=int),
    pair_geom2=wp.array(mjm.pair_geom2, dtype=int),
    pair_solref=create_nmodel_batched_array(mjm.pair_solref, dtype=wp.vec2),
    pair_solreffriction=create_nmodel_batched_array(mjm.pair_solreffriction, dtype=wp.vec2),
    pair_solimp=create_nmodel_batched_array(mjm.pair_solimp, dtype=types.vec5),
    pair_margin=create_nmodel_batched_array(mjm.pair_margin, dtype=float),
    pair_gap=create_nmodel_batched_array(mjm.pair_gap, dtype=float),
    pair_friction=create_nmodel_batched_array(mjm.pair_friction, dtype=types.vec5),
    condim_max=condim_max,  # TODO(team): get max after filtering,
    tendon_adr=wp.array(mjm.tendon_adr, dtype=int),
    tendon_num=wp.array(mjm.tendon_num, dtype=int),
    tendon_limited=wp.array(mjm.tendon_limited, dtype=int),
    tendon_limited_adr=wp.array(np.nonzero(mjm.tendon_limited)[0], dtype=int),
    tendon_actfrclimited=wp.array(mjm.tendon_actfrclimited, dtype=bool),
    tendon_solref_lim=create_nmodel_batched_array(mjm.tendon_solref_lim, dtype=wp.vec2f),
    tendon_solimp_lim=create_nmodel_batched_array(mjm.tendon_solimp_lim, dtype=types.vec5),
    tendon_solref_fri=create_nmodel_batched_array(mjm.tendon_solref_fri, dtype=wp.vec2f),
    tendon_solimp_fri=create_nmodel_batched_array(mjm.tendon_solimp_fri, dtype=types.vec5),
    tendon_range=create_nmodel_batched_array(mjm.tendon_range, dtype=wp.vec2f),
    tendon_actfrcrange=create_nmodel_batched_array(mjm.tendon_actfrcrange, dtype=wp.vec2),
    tendon_margin=create_nmodel_batched_array(mjm.tendon_margin, dtype=float),
    tendon_stiffness=create_nmodel_batched_array(mjm.tendon_stiffness, dtype=float),
    tendon_damping=create_nmodel_batched_array(mjm.tendon_damping, dtype=float),
    tendon_armature=create_nmodel_batched_array(mjm.tendon_armature, dtype=float),
    tendon_frictionloss=create_nmodel_batched_array(mjm.tendon_frictionloss, dtype=float),
    tendon_lengthspring=create_nmodel_batched_array(mjm.tendon_lengthspring, dtype=wp.vec2),
    tendon_length0=create_nmodel_batched_array(mjm.tendon_length0, dtype=float),
    tendon_invweight0=create_nmodel_batched_array(mjm.tendon_invweight0, dtype=float),
    wrap_objid=wp.array(mjm.wrap_objid, dtype=int),
    wrap_prm=wp.array(mjm.wrap_prm, dtype=float),
    wrap_type=wp.array(mjm.wrap_type, dtype=int),
    tendon_jnt_adr=wp.array(tendon_jnt_adr, dtype=int),
    tendon_site_pair_adr=wp.array(tendon_site_pair_adr, dtype=int),
    tendon_geom_adr=wp.array(tendon_geom_adr, dtype=int),
    ten_wrapadr_site=wp.array(ten_wrapadr_site, dtype=int),
    ten_wrapnum_site=wp.array(ten_wrapnum_site, dtype=int),
    wrap_jnt_adr=wp.array(wrap_jnt_adr, dtype=int),
    wrap_site_adr=wp.array(wrap_site_adr, dtype=int),
    wrap_site_pair_adr=wp.array(wrap_site_pair_adr, dtype=int),
    wrap_geom_adr=wp.array(wrap_geom_adr, dtype=int),
    wrap_pulley_scale=wp.array(wrap_pulley_scale, dtype=float),
    sensor_type=wp.array(mjm.sensor_type, dtype=int),
    sensor_datatype=wp.array(mjm.sensor_datatype, dtype=int),
    sensor_objtype=wp.array(mjm.sensor_objtype, dtype=int),
    sensor_objid=wp.array(mjm.sensor_objid, dtype=int),
    sensor_reftype=wp.array(mjm.sensor_reftype, dtype=int),
    sensor_refid=wp.array(mjm.sensor_refid, dtype=int),
    sensor_intprm=wp.array(mjm.sensor_intprm, dtype=int),
    sensor_dim=wp.array(mjm.sensor_dim, dtype=int),
    sensor_adr=wp.array(mjm.sensor_adr, dtype=int),
    sensor_cutoff=wp.array(mjm.sensor_cutoff, dtype=float),
    sensor_pos_adr=wp.array(
      np.nonzero(
        (mjm.sensor_needstage == mujoco.mjtStage.mjSTAGE_POS)
        & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_JOINTLIMITPOS)
        & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONLIMITPOS)
      )[0],
      dtype=int,
    ),
    sensor_limitpos_adr=wp.array(
      np.nonzero(
        (mjm.sensor_type == mujoco.mjtSensor.mjSENS_JOINTLIMITPOS) | (mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONLIMITPOS)
      )[0],
      dtype=int,
    ),
    sensor_vel_adr=wp.array(
      np.nonzero(
        (mjm.sensor_needstage == mujoco.mjtStage.mjSTAGE_VEL)
        & (
          (mjm.sensor_type != mujoco.mjtSensor.mjSENS_JOINTLIMITVEL)
          | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONLIMITVEL)
        )
      )[0],
      dtype=int,
    ),
    sensor_limitvel_adr=wp.array(
      np.nonzero(
        (mjm.sensor_type == mujoco.mjtSensor.mjSENS_JOINTLIMITVEL) | (mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONLIMITVEL)
      )[0],
      dtype=int,
    ),
    sensor_acc_adr=wp.array(
      np.nonzero(
        (mjm.sensor_needstage == mujoco.mjtStage.mjSTAGE_ACC)
        & (
          (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TOUCH)
          | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_JOINTLIMITFRC)
          | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONLIMITFRC)
          | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONACTFRC)
        )
      )[0],
      dtype=int,
    ),
    sensor_rangefinder_adr=wp.array(sensor_rangefinder_adr, dtype=int),
    rangefinder_sensor_adr=wp.array(rangefinder_sensor_adr, dtype=int),
    collision_sensor_adr=wp.array(collision_sensor_adr, dtype=int),
    sensor_touch_adr=wp.array(
      np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_TOUCH)[0],
      dtype=int,
    ),
    sensor_limitfrc_adr=wp.array(
      np.nonzero(
        (mjm.sensor_type == mujoco.mjtSensor.mjSENS_JOINTLIMITFRC) | (mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONLIMITFRC)
      )[0],
      dtype=int,
    ),
    sensor_e_potential=(mjm.sensor_type == mujoco.mjtSensor.mjSENS_E_POTENTIAL).any(),
    sensor_e_kinetic=(mjm.sensor_type == mujoco.mjtSensor.mjSENS_E_KINETIC).any(),
    sensor_tendonactfrc_adr=wp.array(
      np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONACTFRC)[0],
      dtype=int,
    ),
    sensor_subtree_vel=np.isin(
      mjm.sensor_type,
      [mujoco.mjtSensor.mjSENS_SUBTREELINVEL, mujoco.mjtSensor.mjSENS_SUBTREEANGMOM],
    ).any(),
    sensor_contact_adr=wp.array(np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT)[0], dtype=int),
    sensor_adr_to_contact_adr=wp.array(sensor_adr_to_contact_adr, dtype=int),
    sensor_rne_postconstraint=np.isin(
      mjm.sensor_type,
      [
        mujoco.mjtSensor.mjSENS_ACCELEROMETER,
        mujoco.mjtSensor.mjSENS_FORCE,
        mujoco.mjtSensor.mjSENS_TORQUE,
        mujoco.mjtSensor.mjSENS_FRAMELINACC,
        mujoco.mjtSensor.mjSENS_FRAMEANGACC,
      ],
    ).any(),
    sensor_rangefinder_bodyid=wp.array(
      mjm.site_bodyid[mjm.sensor_objid[mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER]], dtype=int
    ),
    plugin=wp.array(plugin_id, dtype=int),
    plugin_attr=wp.array(plugin_attr, dtype=wp.vec3f),
    geom_plugin_index=wp.array(geom_plugin_index, dtype=int),
    mat_texid=create_nmodel_batched_array(mjm.mat_texid, dtype=int),
    mat_texrepeat=create_nmodel_batched_array(mjm.mat_texrepeat, dtype=wp.vec2),
    mat_rgba=create_nmodel_batched_array(mjm.mat_rgba, dtype=wp.vec4),
    actuator_trntype_body_adr=wp.array(np.nonzero(mjm.actuator_trntype == mujoco.mjtTrn.mjTRN_BODY)[0], dtype=int),
    block_dim=types.BlockDim(),
    geom_pair_type_count=tuple(geom_type_pair_count),
    has_sdf_geom=bool(np.any(mjm.geom_type == mujoco.mjtGeom.mjGEOM_SDF)),
    taxel_vertadr=wp.array(
      [
        j + mjm.mesh_vertadr[mjm.sensor_objid[i]]
        for i in range(mjm.nsensor)
        if mjm.sensor_type[i] == mujoco.mjtSensor.mjSENS_TACTILE
        for j in range(mjm.mesh_vertnum[mjm.sensor_objid[i]])
      ],
      dtype=int,
    ),
    taxel_sensorid=wp.array(
      [
        i
        for i in range(mjm.nsensor)
        if mjm.sensor_type[i] == mujoco.mjtSensor.mjSENS_TACTILE
        for j in range(mjm.mesh_vertnum[mjm.sensor_objid[i]])
      ],
      dtype=int,
    ),
  )

  return m


def make_data(
  mjm: mujoco.MjModel,
  nworld: int = 1,
  nconmax: Optional[int] = None,
  njmax: Optional[int] = None,
  naconmax: Optional[int] = None,
) -> types.Data:
  """
  Creates a data object on device.

  Args:
    mjm (mujoco.MjModel): The model containing kinematic and dynamic information (host).
    nworld (int, optional): Number of worlds. Defaults to 1.
    nworld (int, optional): The number of worlds. Defaults to 1.
    nconmax (int, optional): Number of contacts to allocate per world.  Contacts exist in large
                             heterogenous arrays: one world may have more than nconmax contacts.
    njmax (int, optional): Number of constraints to allocate per world.  Constraint arrays are
                           batched by world: no world may have more than njmax constraints.
    naconmax (int, optional): Number of contacts to allocate for all worlds.  Overrides nconmax.

  Returns:
    Data: The data object containing the current state and output arrays (device).
  """

  # TODO(team): move nconmax, njmax to Model?
  # TODO(team): improve heuristic for nconmax and njmax
  nconmax = nconmax or 20
  njmax = njmax or nconmax * 6

  if nworld < 1 or nworld > MAX_WORLDS:
    raise ValueError(f"nworld must be >= 1 and <= {MAX_WORLDS}")

  if naconmax is None:
    if nconmax < 0:
      raise ValueError("nconmax must be >= 0")
    naconmax = max(512, nworld * nconmax)
  elif naconmax < 0:
    raise ValueError("naconmax must be >= 0")

  if njmax < 0:
    raise ValueError("njmax must be >= 0")

  if mujoco.mj_isSparse(mjm):
    qM = wp.zeros((nworld, 1, mjm.nM), dtype=float)
    qLD = wp.zeros((nworld, 1, mjm.nC), dtype=float)
    qM_integration = wp.zeros((nworld, 1, mjm.nM), dtype=float)
    qLD_integration = wp.zeros((nworld, 1, mjm.nM), dtype=float)
  else:
    qM = wp.zeros((nworld, mjm.nv, mjm.nv), dtype=float)
    qLD = wp.zeros((nworld, mjm.nv, mjm.nv), dtype=float)
    qM_integration = wp.zeros((nworld, mjm.nv, mjm.nv), dtype=float)
    qLD_integration = wp.zeros((nworld, mjm.nv, mjm.nv), dtype=float)

  condim = np.concatenate((mjm.geom_condim, mjm.pair_dim))
  condim_max = np.max(condim) if len(condim) > 0 else 0
  max_npolygon = _max_npolygon(mjm)
  max_meshdegree = _max_meshdegree(mjm)
  nsensorcontact = np.sum(mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT)
  nrangefinder = sum(mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER)

  return types.Data(
    nworld=nworld,
    naconmax=naconmax,
    njmax=njmax,
    solver_niter=wp.zeros(nworld, dtype=int),
    nacon=wp.zeros(1, dtype=int),
    ne=wp.zeros(nworld, dtype=int),
    ne_connect=wp.zeros(nworld, dtype=int),  # warp only
    ne_weld=wp.zeros(nworld, dtype=int),  # warp only
    ne_jnt=wp.zeros(nworld, dtype=int),  # warp only
    ne_ten=wp.zeros(nworld, dtype=int),  # warp only
    nf=wp.zeros(nworld, dtype=int),
    nl=wp.zeros(nworld, dtype=int),
    nefc=wp.zeros(nworld, dtype=int),
    nsolving=wp.zeros(1, dtype=int),  # warp only
    time=wp.zeros(nworld, dtype=float),
    energy=wp.zeros(nworld, dtype=wp.vec2),
    qpos=wp.zeros((nworld, mjm.nq), dtype=float),
    qvel=wp.zeros((nworld, mjm.nv), dtype=float),
    act=wp.zeros((nworld, mjm.na), dtype=float),
    qacc_warmstart=wp.zeros((nworld, mjm.nv), dtype=float),
    qacc_discrete=wp.zeros((nworld, mjm.nv), dtype=float),
    ctrl=wp.zeros((nworld, mjm.nu), dtype=float),
    qfrc_applied=wp.zeros((nworld, mjm.nv), dtype=float),
    xfrc_applied=wp.zeros((nworld, mjm.nbody), dtype=wp.spatial_vector),
    fluid_applied=wp.zeros((nworld, mjm.nbody), dtype=wp.spatial_vector),
    eq_active=wp.array(np.tile(mjm.eq_active0, (nworld, 1)), dtype=bool),
    mocap_pos=wp.zeros((nworld, mjm.nmocap), dtype=wp.vec3),
    mocap_quat=wp.zeros((nworld, mjm.nmocap), dtype=wp.quat),
    qacc=wp.zeros((nworld, mjm.nv), dtype=float),
    act_dot=wp.zeros((nworld, mjm.na), dtype=float),
    xpos=wp.zeros((nworld, mjm.nbody), dtype=wp.vec3),
    xquat=wp.zeros((nworld, mjm.nbody), dtype=wp.quat),
    xmat=wp.zeros((nworld, mjm.nbody), dtype=wp.mat33),
    xipos=wp.zeros((nworld, mjm.nbody), dtype=wp.vec3),
    ximat=wp.zeros((nworld, mjm.nbody), dtype=wp.mat33),
    xanchor=wp.zeros((nworld, mjm.njnt), dtype=wp.vec3),
    xaxis=wp.zeros((nworld, mjm.njnt), dtype=wp.vec3),
    geom_skip=wp.zeros(mjm.ngeom, dtype=bool),  # warp only
    geom_xpos=wp.zeros((nworld, mjm.ngeom), dtype=wp.vec3),
    geom_xmat=wp.zeros((nworld, mjm.ngeom), dtype=wp.mat33),
    site_xpos=wp.zeros((nworld, mjm.nsite), dtype=wp.vec3),
    site_xmat=wp.zeros((nworld, mjm.nsite), dtype=wp.mat33),
    cam_xpos=wp.zeros((nworld, mjm.ncam), dtype=wp.vec3),
    cam_xmat=wp.zeros((nworld, mjm.ncam), dtype=wp.mat33),
    light_xpos=wp.zeros((nworld, mjm.nlight), dtype=wp.vec3),
    light_xdir=wp.zeros((nworld, mjm.nlight), dtype=wp.vec3),
    subtree_com=wp.zeros((nworld, mjm.nbody), dtype=wp.vec3),
    cdof=wp.zeros((nworld, mjm.nv), dtype=wp.spatial_vector),
    cinert=wp.zeros((nworld, mjm.nbody), dtype=types.vec10),
    flexvert_xpos=wp.zeros((nworld, mjm.nflexvert), dtype=wp.vec3),
    flexedge_length=wp.zeros((nworld, mjm.nflexedge), dtype=wp.float32),
    flexedge_velocity=wp.zeros((nworld, mjm.nflexedge), dtype=wp.float32),
    actuator_length=wp.zeros((nworld, mjm.nu), dtype=float),
    actuator_moment=wp.zeros((nworld, mjm.nu, mjm.nv), dtype=float),
    crb=wp.zeros((nworld, mjm.nbody), dtype=types.vec10),
    qM=qM,
    qLD=qLD,
    qLDiagInv=wp.zeros((nworld, mjm.nv), dtype=float),
    ten_velocity=wp.zeros((nworld, mjm.ntendon), dtype=float),
    actuator_velocity=wp.zeros((nworld, mjm.nu), dtype=float),
    cvel=wp.zeros((nworld, mjm.nbody), dtype=wp.spatial_vector),
    cdof_dot=wp.zeros((nworld, mjm.nv), dtype=wp.spatial_vector),
    qfrc_bias=wp.zeros((nworld, mjm.nv), dtype=float),
    qfrc_spring=wp.zeros((nworld, mjm.nv), dtype=float),
    qfrc_damper=wp.zeros((nworld, mjm.nv), dtype=float),
    qfrc_gravcomp=wp.zeros((nworld, mjm.nv), dtype=float),
    qfrc_fluid=wp.zeros((nworld, mjm.nv), dtype=float),
    qfrc_passive=wp.zeros((nworld, mjm.nv), dtype=float),
    subtree_linvel=wp.zeros((nworld, mjm.nbody), dtype=wp.vec3),
    subtree_angmom=wp.zeros((nworld, mjm.nbody), dtype=wp.vec3),
    subtree_bodyvel=wp.zeros((nworld, mjm.nbody), dtype=wp.spatial_vector),  # warp only
    actuator_force=wp.zeros((nworld, mjm.nu), dtype=float),
    qfrc_actuator=wp.zeros((nworld, mjm.nv), dtype=float),
    qfrc_smooth=wp.zeros((nworld, mjm.nv), dtype=float),
    qacc_smooth=wp.zeros((nworld, mjm.nv), dtype=float),
    qfrc_constraint=wp.zeros((nworld, mjm.nv), dtype=float),
    qfrc_inverse=wp.zeros((nworld, mjm.nv), dtype=float),
    contact=types.Contact(
      dist=wp.zeros((naconmax,), dtype=float),
      pos=wp.zeros((naconmax,), dtype=wp.vec3f),
      frame=wp.zeros((naconmax,), dtype=wp.mat33f),
      includemargin=wp.zeros((naconmax,), dtype=float),
      friction=wp.zeros((naconmax,), dtype=types.vec5),
      solref=wp.zeros((naconmax,), dtype=wp.vec2f),
      solreffriction=wp.zeros((naconmax,), dtype=wp.vec2f),
      solimp=wp.zeros((naconmax,), dtype=types.vec5),
      dim=wp.zeros((naconmax,), dtype=int),
      geom=wp.zeros((naconmax,), dtype=wp.vec2i),
      efc_address=wp.zeros((naconmax, np.maximum(1, 2 * (condim_max - 1))), dtype=int),
      worldid=wp.zeros((naconmax,), dtype=int),
    ),
    efc=types.Constraint(
      type=wp.zeros((nworld, njmax), dtype=int),
      id=wp.zeros((nworld, njmax), dtype=int),
      J=wp.zeros((nworld, njmax, mjm.nv), dtype=float),
      pos=wp.zeros((nworld, njmax), dtype=float),
      margin=wp.zeros((nworld, njmax), dtype=float),
      D=wp.zeros((nworld, njmax), dtype=float),
      vel=wp.zeros((nworld, njmax), dtype=float),
      aref=wp.zeros((nworld, njmax), dtype=float),
      frictionloss=wp.zeros((nworld, njmax), dtype=float),
      force=wp.zeros((nworld, njmax), dtype=float),
      Jaref=wp.zeros((nworld, njmax), dtype=float),
      Ma=wp.zeros((nworld, mjm.nv), dtype=float),
      grad=wp.zeros((nworld, mjm.nv), dtype=float),
      cholesky_L_tmp=wp.zeros((nworld, mjm.nv, mjm.nv), dtype=float),
      cholesky_y_tmp=wp.zeros((nworld, mjm.nv), dtype=float),
      grad_dot=wp.zeros((nworld,), dtype=float),
      Mgrad=wp.zeros((nworld, mjm.nv), dtype=float),
      search=wp.zeros((nworld, mjm.nv), dtype=float),
      search_dot=wp.zeros((nworld,), dtype=float),
      gauss=wp.zeros((nworld,), dtype=float),
      cost=wp.zeros((nworld,), dtype=float),
      prev_cost=wp.zeros((nworld,), dtype=float),
      state=wp.zeros((nworld, njmax), dtype=int),
      mv=wp.zeros((nworld, mjm.nv), dtype=float),
      jv=wp.zeros((nworld, njmax), dtype=float),
      quad=wp.zeros((nworld, njmax), dtype=wp.vec3f),
      quad_gauss=wp.zeros((nworld,), dtype=wp.vec3f),
      h=wp.zeros((nworld, mjm.nv, mjm.nv), dtype=float),
      alpha=wp.zeros((nworld,), dtype=float),
      prev_grad=wp.zeros((nworld, mjm.nv), dtype=float),
      prev_Mgrad=wp.zeros((nworld, mjm.nv), dtype=float),
      beta=wp.zeros((nworld,), dtype=float),
      done=wp.zeros((nworld,), dtype=bool),
      # linesearch
      cost_candidate=wp.zeros((nworld, mjm.opt.ls_iterations), dtype=float),
    ),
    # RK4
    qpos_t0=wp.zeros((nworld, mjm.nq), dtype=float),
    qvel_t0=wp.zeros((nworld, mjm.nv), dtype=float),
    act_t0=wp.zeros((nworld, mjm.na), dtype=float),
    qvel_rk=wp.zeros((nworld, mjm.nv), dtype=float),
    qacc_rk=wp.zeros((nworld, mjm.nv), dtype=float),
    act_dot_rk=wp.zeros((nworld, mjm.na), dtype=float),
    # euler + implicit integration
    qfrc_integration=wp.zeros((nworld, mjm.nv), dtype=float),
    qacc_integration=wp.zeros((nworld, mjm.nv), dtype=float),
    act_vel_integration=wp.zeros((nworld, mjm.nu), dtype=float),
    qM_integration=qM_integration,
    qLD_integration=qLD_integration,
    qLDiagInv_integration=wp.zeros((nworld, mjm.nv), dtype=float),
    # sweep-and-prune broadphase
    sap_projection_lower=wp.zeros((nworld, mjm.ngeom, 2), dtype=float),
    sap_projection_upper=wp.zeros((nworld, mjm.ngeom), dtype=float),
    sap_sort_index=wp.zeros((nworld, mjm.ngeom, 2), dtype=int),
    sap_range=wp.zeros((nworld, mjm.ngeom), dtype=int),
    sap_cumulative_sum=wp.zeros((nworld, mjm.ngeom), dtype=int),
    sap_segment_index=wp.array(
      np.array([i * mjm.ngeom if i < nworld + 1 else 0 for i in range(2 * nworld)]).reshape((nworld, 2)), dtype=int
    ),
    # collision driver
    collision_pair=wp.zeros((naconmax,), dtype=wp.vec2i),
    collision_pairid=wp.zeros((naconmax,), dtype=int),
    collision_worldid=wp.zeros((naconmax,), dtype=int),
    ncollision=wp.zeros((1,), dtype=int),
    # narrowphase (EPA polytope)
    epa_vert=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=wp.vec3),
    epa_vert1=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=wp.vec3),
    epa_vert2=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=wp.vec3),
    epa_vert_index1=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=int),
    epa_vert_index2=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=int),
    epa_face=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=wp.vec3i),
    epa_pr=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=wp.vec3),
    epa_norm2=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=float),
    epa_index=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=int),
    epa_map=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=int),
    epa_horizon=wp.zeros(shape=(naconmax, 2 * types.MJ_MAX_EPAHORIZON), dtype=int),
    multiccd_polygon=wp.zeros(shape=(naconmax, 2 * max_npolygon), dtype=wp.vec3),
    multiccd_clipped=wp.zeros(shape=(naconmax, 2 * max_npolygon), dtype=wp.vec3),
    multiccd_pnormal=wp.zeros(shape=(naconmax, max_npolygon), dtype=wp.vec3),
    multiccd_pdist=wp.zeros(shape=(naconmax, max_npolygon), dtype=float),
    multiccd_idx1=wp.zeros(shape=(naconmax, max_meshdegree), dtype=int),
    multiccd_idx2=wp.zeros(shape=(naconmax, max_meshdegree), dtype=int),
    multiccd_n1=wp.zeros(shape=(naconmax, max_meshdegree), dtype=wp.vec3),
    multiccd_n2=wp.zeros(shape=(naconmax, max_meshdegree), dtype=wp.vec3),
    multiccd_endvert=wp.zeros(shape=(naconmax, max_meshdegree), dtype=wp.vec3),
    multiccd_face1=wp.zeros(shape=(naconmax, max_npolygon), dtype=wp.vec3),
    multiccd_face2=wp.zeros(shape=(naconmax, max_npolygon), dtype=wp.vec3),
    # rne_postconstraint
    cacc=wp.zeros((nworld, mjm.nbody), dtype=wp.spatial_vector),
    cfrc_int=wp.zeros((nworld, mjm.nbody), dtype=wp.spatial_vector),
    cfrc_ext=wp.zeros((nworld, mjm.nbody), dtype=wp.spatial_vector),
    # tendon
    ten_length=wp.zeros((nworld, mjm.ntendon), dtype=float),
    ten_J=wp.zeros((nworld, mjm.ntendon, mjm.nv), dtype=float),
    ten_Jdot=wp.zeros((nworld, mjm.ntendon, mjm.nv), dtype=float),
    ten_bias_coef=wp.zeros((nworld, mjm.ntendon), dtype=float),
    ten_wrapadr=wp.zeros((nworld, mjm.ntendon), dtype=int),
    ten_wrapnum=wp.zeros((nworld, mjm.ntendon), dtype=int),
    ten_actfrc=wp.zeros((nworld, mjm.ntendon), dtype=float),
    wrap_obj=wp.zeros((nworld, mjm.nwrap), dtype=wp.vec2i),
    wrap_xpos=wp.zeros((nworld, mjm.nwrap), dtype=wp.spatial_vector),
    wrap_geom_xpos=wp.zeros((nworld, mjm.nwrap), dtype=wp.spatial_vector),
    # sensors
    sensordata=wp.zeros((nworld, mjm.nsensordata), dtype=float),
    sensor_rangefinder_pnt=wp.zeros((nworld, nrangefinder), dtype=wp.vec3),
    sensor_rangefinder_vec=wp.zeros((nworld, nrangefinder), dtype=wp.vec3),
    sensor_rangefinder_dist=wp.zeros((nworld, nrangefinder), dtype=float),
    sensor_rangefinder_geomid=wp.zeros((nworld, nrangefinder), dtype=int),
    sensor_contact_nmatch=wp.zeros((nworld, nsensorcontact), dtype=int),
    sensor_contact_matchid=wp.zeros((nworld, nsensorcontact, types.MJ_MAXCONPAIR), dtype=int),
    sensor_contact_criteria=wp.zeros((nworld, nsensorcontact, types.MJ_MAXCONPAIR), dtype=float),
    sensor_contact_direction=wp.zeros((nworld, nsensorcontact, types.MJ_MAXCONPAIR), dtype=float),
    # ray
    ray_bodyexclude=wp.zeros(1, dtype=int),
    ray_dist=wp.zeros((nworld, 1), dtype=float),
    ray_geomid=wp.zeros((nworld, 1), dtype=int),
    # mul_m
    energy_vel_mul_m_skip=wp.zeros((nworld,), dtype=bool),
    inverse_mul_m_skip=wp.zeros((nworld,), dtype=bool),
    # actuator
    actuator_trntype_body_ncon=wp.zeros((nworld, np.sum(mjm.actuator_trntype == mujoco.mjtTrn.mjTRN_BODY)), dtype=int),
  )


def put_data(
  mjm: mujoco.MjModel,
  mjd: mujoco.MjData,
  nworld: int = 1,
  nconmax: Optional[int] = None,
  njmax: Optional[int] = None,
  naconmax: Optional[int] = None,
) -> types.Data:
  """
  Moves data from host to a device.

  Args:
    mjm (mujoco.MjModel): The model containing kinematic and dynamic information (host).
    mjd (mujoco.MjData): The data object containing current state and output arrays (host).
    nworld (int, optional): The number of worlds. Defaults to 1.
    nconmax (int, optional): Number of contacts to allocate per world.  Contacts exist in large
                             heterogenous arrays: one world may have more than nconmax contacts.
    njmax (int, optional): Number of constraints to allocate per world.  Constraint arrays are
                           batched by world: no world may have more than njmax constraints.
    naconmax (int, optional): Number of contacts to allocate for all worlds.  Overrides nconmax.

  Returns:
    Data: The data object containing the current state and output arrays (device).
  """
  # TODO(team): move nconmax and njmax to Model?
  # TODO(team): decide what to do about uninitialized warp-only fields created by put_data
  #             we need to ensure these are only workspace fields and don't carry state

  # TODO(team): better heuristic for nconmax and njmax
  nconmax = nconmax or max(5, 4 * mjd.ncon)
  njmax = njmax or max(5, 4 * mjd.nefc)

  if nworld < 1 or nworld > MAX_WORLDS:
    raise ValueError(f"nworld must be >= 1 and <= {MAX_WORLDS}")

  if naconmax is None:
    if nconmax < 0:
      raise ValueError("nconmax must be >= 0")

    if mjd.ncon > nconmax:
      raise ValueError(f"nconmax overflow (nconmax must be >= {mjd.ncon})")

    naconmax = max(512, nworld * nconmax)
  elif naconmax < mjd.ncon * nworld:
    raise ValueError(f"naconmax overflow (naconmax must be >= {mjd.ncon * nworld})")

  if njmax < 0:
    raise ValueError("njmax must be >= 0")

  if mjd.nefc > njmax:
    raise ValueError(f"njmax overflow (njmax must be >= {mjd.nefc})")

  max_npolygon = _max_npolygon(mjm)
  max_meshdegree = _max_meshdegree(mjm)

  # calculate some fields that cannot be easily computed inline:
  if mujoco.mj_isSparse(mjm):
    qM = np.expand_dims(mjd.qM, axis=0)
    qLD = np.expand_dims(mjd.qLD, axis=0)
    qM_integration = np.zeros((1, mjm.nM), dtype=float)
    qLD_integration = np.zeros((1, mjm.nM), dtype=float)
    efc_J = np.zeros((mjd.nefc, mjm.nv))
    mujoco.mju_sparse2dense(efc_J, mjd.efc_J, mjd.efc_J_rownnz, mjd.efc_J_rowadr, mjd.efc_J_colind)
    ten_J = np.zeros((mjm.ntendon, mjm.nv))
    mujoco.mju_sparse2dense(
      ten_J,
      mjd.ten_J.reshape(-1),
      mjd.ten_J_rownnz,
      mjd.ten_J_rowadr,
      mjd.ten_J_colind.reshape(-1),
    )
  else:
    qM = np.zeros((mjm.nv, mjm.nv))
    mujoco.mj_fullM(mjm, qM, mjd.qM)
    if (mjd.qM == 0.0).all() or (mjd.qLD == 0.0).all():
      qLD = np.zeros((mjm.nv, mjm.nv))
    else:
      qLD = np.linalg.cholesky(qM)
    qM_integration = np.zeros((mjm.nv, mjm.nv), dtype=float)
    qLD_integration = np.zeros((mjm.nv, mjm.nv), dtype=float)
    efc_J = mjd.efc_J.reshape((mjd.nefc, mjm.nv))
    ten_J = mjd.ten_J.reshape((mjm.ntendon, mjm.nv))

  # TODO(taylorhowell): sparse actuator_moment
  actuator_moment = np.zeros((mjm.nu, mjm.nv))
  mujoco.mju_sparse2dense(
    actuator_moment,
    mjd.actuator_moment,
    mjd.moment_rownnz,
    mjd.moment_rowadr,
    mjd.moment_colind,
  )

  condim = np.concatenate((mjm.geom_condim, mjm.pair_dim))
  condim_max = np.max(condim) if len(condim) > 0 else 0
  contact_efc_address = np.zeros((naconmax, np.maximum(1, 2 * (condim_max - 1))), dtype=int)
  for i in range(nworld):
    for j in range(mjd.ncon):
      condim = mjd.contact.dim[j]
      efc_address = mjd.contact.efc_address[j]
      if efc_address == -1:
        continue
      if condim == 1:
        nconvar = 1
      else:
        nconvar = condim if mjm.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC else 2 * (condim - 1)
      for k in range(nconvar):
        contact_efc_address[i * mjd.ncon + j, k] = mjd.nefc * i + efc_address + k

  contact_worldid = np.pad(np.repeat(np.arange(nworld), mjd.ncon), (0, naconmax - nworld * mjd.ncon))

  ne_connect = int(3 * np.sum((mjm.eq_type == mujoco.mjtEq.mjEQ_CONNECT) & mjd.eq_active))
  ne_weld = int(6 * np.sum((mjm.eq_type == mujoco.mjtEq.mjEQ_WELD) & mjd.eq_active))
  ne_jnt = int(np.sum((mjm.eq_type == mujoco.mjtEq.mjEQ_JOINT) & mjd.eq_active))
  ne_ten = int(np.sum((mjm.eq_type == mujoco.mjtEq.mjEQ_TENDON) & mjd.eq_active))

  efc_type_fill = np.zeros((nworld, njmax))
  efc_id_fill = np.zeros((nworld, njmax))
  efc_J_fill = np.zeros((nworld, njmax, mjm.nv))
  efc_D_fill = np.zeros((nworld, njmax))
  efc_vel_fill = np.zeros((nworld, njmax))
  efc_pos_fill = np.zeros((nworld, njmax))
  efc_aref_fill = np.zeros((nworld, njmax))
  efc_frictionloss_fill = np.zeros((nworld, njmax))
  efc_force_fill = np.zeros((nworld, njmax))
  efc_margin_fill = np.zeros((nworld, njmax))

  nefc = mjd.nefc
  efc_type_fill[:, :nefc] = np.tile(mjd.efc_type, (nworld, 1))
  efc_id_fill[:, :nefc] = np.tile(mjd.efc_id, (nworld, 1))
  efc_J_fill[:, :nefc, :] = np.tile(efc_J, (nworld, 1, 1))
  efc_D_fill[:, :nefc] = np.tile(mjd.efc_D, (nworld, 1))
  efc_vel_fill[:, :nefc] = np.tile(mjd.efc_vel, (nworld, 1))
  efc_pos_fill[:, :nefc] = np.tile(mjd.efc_pos, (nworld, 1))
  efc_aref_fill[:, :nefc] = np.tile(mjd.efc_aref, (nworld, 1))
  efc_frictionloss_fill[:, :nefc] = np.tile(mjd.efc_frictionloss, (nworld, 1))
  efc_force_fill[:, :nefc] = np.tile(mjd.efc_force, (nworld, 1))
  efc_margin_fill[:, :nefc] = np.tile(mjd.efc_margin, (nworld, 1))

  nsensorcontact = np.sum(mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT)
  nrangefinder = sum(mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER)

  # some helper functions to simplify the data field definitions below

  def arr(x, dtype=None):
    if not isinstance(x, np.ndarray):
      x = np.array(x)
    if dtype is None:
      if np.issubdtype(x.dtype, np.integer):
        dtype = wp.int32
      elif np.issubdtype(x.dtype, np.floating):
        dtype = wp.float32
      elif np.issubdtype(x.dtype, bool):
        dtype = wp.bool
      else:
        raise ValueError(f"Unsupported dtype: {x.dtype}")
    wp_array = {1: wp.array, 2: wp.array2d, 3: wp.array3d}[x.ndim]
    return wp_array(x, dtype=dtype)

  def tile(x, dtype=None):
    return arr(np.tile(x, (nworld,) + (1,) * len(x.shape)), dtype)

  def padtile(x, length, dtype=None):
    x = np.repeat(x, nworld, axis=0)
    width = ((0, length - x.shape[0]),) + ((0, 0),) * (x.ndim - 1)
    return arr(np.pad(x, width), dtype)

  return types.Data(
    nworld=nworld,
    naconmax=naconmax,
    njmax=njmax,
    solver_niter=tile(mjd.solver_niter[0]),
    nacon=arr([mjd.ncon * nworld]),
    ne=wp.full(shape=(nworld), value=mjd.ne),
    ne_connect=wp.full(shape=(nworld), value=ne_connect),
    ne_weld=wp.full(shape=(nworld), value=ne_weld),
    ne_jnt=wp.full(shape=(nworld), value=ne_jnt),
    ne_ten=wp.full(shape=(nworld), value=ne_ten),
    nf=wp.full(shape=(nworld), value=mjd.nf),
    nl=wp.full(shape=(nworld), value=mjd.nl),
    nefc=wp.full(shape=(nworld), value=mjd.nefc),
    nsolving=arr([nworld]),
    time=arr(mjd.time * np.ones(nworld)),
    energy=tile(mjd.energy, dtype=wp.vec2),
    qpos=tile(mjd.qpos),
    qvel=tile(mjd.qvel),
    act=tile(mjd.act),
    qacc_warmstart=tile(mjd.qacc_warmstart),
    qacc_discrete=wp.zeros((nworld, mjm.nv), dtype=float),
    ctrl=tile(mjd.ctrl),
    qfrc_applied=tile(mjd.qfrc_applied),
    xfrc_applied=tile(mjd.xfrc_applied, dtype=wp.spatial_vector),
    fluid_applied=wp.zeros((nworld, mjm.nbody), dtype=wp.spatial_vector),
    eq_active=tile(mjd.eq_active.astype(bool)),
    mocap_pos=tile(mjd.mocap_pos, dtype=wp.vec3),
    mocap_quat=tile(mjd.mocap_quat, dtype=wp.quat),
    qacc=tile(mjd.qacc),
    act_dot=tile(mjd.act_dot),
    xpos=tile(mjd.xpos, dtype=wp.vec3),
    xquat=tile(mjd.xquat, dtype=wp.quat),
    xmat=tile(mjd.xmat, dtype=wp.mat33),
    xipos=tile(mjd.xipos, dtype=wp.vec3),
    ximat=tile(mjd.ximat, dtype=wp.mat33),
    xanchor=tile(mjd.xanchor, dtype=wp.vec3),
    xaxis=tile(mjd.xaxis, dtype=wp.vec3),
    geom_skip=wp.zeros(mjm.ngeom, dtype=bool),  # warp only
    geom_xpos=tile(mjd.geom_xpos, dtype=wp.vec3),
    geom_xmat=tile(mjd.geom_xmat, dtype=wp.mat33),
    site_xpos=tile(mjd.site_xpos, dtype=wp.vec3),
    site_xmat=tile(mjd.site_xmat, dtype=wp.mat33),
    cam_xpos=tile(mjd.cam_xpos, dtype=wp.vec3),
    cam_xmat=tile(mjd.cam_xmat, dtype=wp.mat33),
    light_xpos=tile(mjd.light_xpos, dtype=wp.vec3),
    light_xdir=tile(mjd.light_xdir, dtype=wp.vec3),
    subtree_com=tile(mjd.subtree_com, dtype=wp.vec3),
    cdof=tile(mjd.cdof, dtype=wp.spatial_vector),
    cinert=tile(mjd.cinert, dtype=types.vec10),
    flexvert_xpos=tile(mjd.flexvert_xpos, dtype=wp.vec3),
    flexedge_length=tile(mjd.flexedge_length),
    flexedge_velocity=tile(mjd.flexedge_velocity),
    actuator_length=tile(mjd.actuator_length),
    actuator_moment=tile(actuator_moment),
    crb=tile(mjd.crb, dtype=types.vec10),
    qM=tile(qM),
    qLD=tile(qLD),
    qLDiagInv=tile(mjd.qLDiagInv),
    ten_velocity=tile(mjd.ten_velocity),
    actuator_velocity=tile(mjd.actuator_velocity),
    cvel=tile(mjd.cvel, dtype=wp.spatial_vector),
    cdof_dot=tile(mjd.cdof_dot, dtype=wp.spatial_vector),
    qfrc_bias=tile(mjd.qfrc_bias),
    qfrc_spring=tile(mjd.qfrc_spring),
    qfrc_damper=tile(mjd.qfrc_damper),
    qfrc_gravcomp=tile(mjd.qfrc_gravcomp),
    qfrc_fluid=tile(mjd.qfrc_fluid),
    qfrc_passive=tile(mjd.qfrc_passive),
    subtree_linvel=tile(mjd.subtree_linvel, dtype=wp.vec3),
    subtree_angmom=tile(mjd.subtree_angmom, dtype=wp.vec3),
    subtree_bodyvel=wp.zeros((nworld, mjm.nbody), dtype=wp.spatial_vector),
    actuator_force=tile(mjd.actuator_force),
    qfrc_actuator=tile(mjd.qfrc_actuator),
    qfrc_smooth=tile(mjd.qfrc_smooth),
    qacc_smooth=tile(mjd.qacc_smooth),
    qfrc_constraint=tile(mjd.qfrc_constraint),
    qfrc_inverse=tile(mjd.qfrc_inverse),
    contact=types.Contact(
      dist=padtile(mjd.contact.dist, naconmax),
      pos=padtile(mjd.contact.pos, naconmax, dtype=wp.vec3),
      frame=padtile(mjd.contact.frame, naconmax, dtype=wp.mat33),
      includemargin=padtile(mjd.contact.includemargin, naconmax),
      friction=padtile(mjd.contact.friction, naconmax, dtype=types.vec5),
      solref=padtile(mjd.contact.solref, naconmax, dtype=wp.vec2f),
      solreffriction=padtile(mjd.contact.solreffriction, naconmax, dtype=wp.vec2f),
      solimp=padtile(mjd.contact.solimp, naconmax, dtype=types.vec5),
      dim=padtile(mjd.contact.dim, naconmax),
      geom=padtile(mjd.contact.geom, naconmax, dtype=wp.vec2i),
      efc_address=arr(contact_efc_address),
      worldid=arr(contact_worldid),
    ),
    efc=types.Constraint(
      type=wp.array2d(efc_type_fill, dtype=int),
      id=wp.array2d(efc_id_fill, dtype=int),
      J=wp.array3d(efc_J_fill, dtype=float),
      pos=wp.array2d(efc_pos_fill, dtype=float),
      margin=wp.array2d(efc_margin_fill, dtype=float),
      D=wp.array2d(efc_D_fill, dtype=float),
      vel=wp.array2d(efc_vel_fill, dtype=float),
      aref=wp.array2d(efc_aref_fill, dtype=float),
      frictionloss=wp.array2d(efc_frictionloss_fill, dtype=float),
      force=wp.array2d(efc_force_fill, dtype=float),
      Jaref=wp.empty(shape=(nworld, njmax), dtype=float),
      Ma=wp.empty(shape=(nworld, mjm.nv), dtype=float),
      grad=wp.empty(shape=(nworld, mjm.nv), dtype=float),
      cholesky_L_tmp=wp.empty(shape=(nworld, mjm.nv, mjm.nv), dtype=float),
      cholesky_y_tmp=wp.empty(shape=(nworld, mjm.nv), dtype=float),
      grad_dot=wp.empty(shape=(nworld,), dtype=float),
      Mgrad=wp.empty(shape=(nworld, mjm.nv), dtype=float),
      search=wp.empty(shape=(nworld, mjm.nv), dtype=float),
      search_dot=wp.empty(shape=(nworld,), dtype=float),
      gauss=wp.empty(shape=(nworld,), dtype=float),
      cost=wp.empty(shape=(nworld,), dtype=float),
      prev_cost=wp.empty(shape=(nworld,), dtype=float),
      state=wp.empty(shape=(nworld, njmax), dtype=int),
      mv=wp.empty(shape=(nworld, mjm.nv), dtype=float),
      jv=wp.empty(shape=(nworld, njmax), dtype=float),
      quad=wp.empty(shape=(nworld, njmax), dtype=wp.vec3f),
      quad_gauss=wp.empty(shape=(nworld,), dtype=wp.vec3f),
      h=wp.empty(shape=(nworld, mjm.nv, mjm.nv), dtype=float),
      alpha=wp.empty(shape=(nworld,), dtype=float),
      prev_grad=wp.empty(shape=(nworld, mjm.nv), dtype=float),
      prev_Mgrad=wp.empty(shape=(nworld, mjm.nv), dtype=float),
      beta=wp.empty(shape=(nworld,), dtype=float),
      done=wp.empty(shape=(nworld,), dtype=bool),
      cost_candidate=wp.empty(shape=(nworld, mjm.opt.ls_iterations), dtype=float),
    ),
    # TODO(team): skip allocation if integrator != RK4
    qpos_t0=wp.empty((nworld, mjm.nq), dtype=float),
    qvel_t0=wp.empty((nworld, mjm.nv), dtype=float),
    act_t0=wp.empty((nworld, mjm.na), dtype=float),
    qvel_rk=wp.empty((nworld, mjm.nv), dtype=float),
    qacc_rk=wp.empty((nworld, mjm.nv), dtype=float),
    act_dot_rk=wp.empty((nworld, mjm.na), dtype=float),
    # TODO(team): skip allocation if integrator != euler | implicit
    qfrc_integration=wp.zeros((nworld, mjm.nv), dtype=float),
    qacc_integration=wp.zeros((nworld, mjm.nv), dtype=float),
    act_vel_integration=wp.zeros((nworld, mjm.nu), dtype=float),
    qM_integration=tile(qM_integration),
    qLD_integration=tile(qLD_integration),
    qLDiagInv_integration=wp.zeros((nworld, mjm.nv), dtype=float),
    # TODO(team): skip allocation if broadphase != sap
    sap_projection_lower=wp.zeros((nworld, mjm.ngeom, 2), dtype=float),
    sap_projection_upper=wp.zeros((nworld, mjm.ngeom), dtype=float),
    sap_sort_index=wp.zeros((nworld, mjm.ngeom, 2), dtype=int),
    sap_range=wp.zeros((nworld, mjm.ngeom), dtype=int),
    sap_cumulative_sum=wp.zeros((nworld, mjm.ngeom), dtype=int),
    sap_segment_index=arr(np.array([i * mjm.ngeom if i < nworld + 1 else 0 for i in range(2 * nworld)]).reshape((nworld, 2))),
    # collision driver
    collision_pair=wp.empty(naconmax, dtype=wp.vec2i),
    collision_pairid=wp.empty(naconmax, dtype=int),
    collision_worldid=wp.empty(naconmax, dtype=int),
    ncollision=wp.zeros(1, dtype=int),
    # narrowphase (EPA polytope)
    epa_vert=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=wp.vec3),
    epa_vert1=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=wp.vec3),
    epa_vert2=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=wp.vec3),
    epa_vert_index1=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=int),
    epa_vert_index2=wp.zeros(shape=(naconmax, 5 + mjm.opt.ccd_iterations), dtype=int),
    epa_face=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=wp.vec3i),
    epa_pr=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=wp.vec3),
    epa_norm2=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=float),
    epa_index=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=int),
    epa_map=wp.zeros(shape=(naconmax, 6 + types.MJ_MAX_EPAFACES * mjm.opt.ccd_iterations), dtype=int),
    epa_horizon=wp.zeros(shape=(naconmax, 2 * types.MJ_MAX_EPAHORIZON), dtype=int),
    multiccd_polygon=wp.zeros(shape=(naconmax, 2 * max_npolygon), dtype=wp.vec3),
    multiccd_clipped=wp.zeros(shape=(naconmax, 2 * max_npolygon), dtype=wp.vec3),
    multiccd_pnormal=wp.zeros(shape=(naconmax, max_npolygon), dtype=wp.vec3),
    multiccd_pdist=wp.zeros(shape=(naconmax, max_npolygon), dtype=float),
    multiccd_idx1=wp.zeros(shape=(naconmax, max_meshdegree), dtype=int),
    multiccd_idx2=wp.zeros(shape=(naconmax, max_meshdegree), dtype=int),
    multiccd_n1=wp.zeros(shape=(naconmax, max_meshdegree), dtype=wp.vec3),
    multiccd_n2=wp.zeros(shape=(naconmax, max_meshdegree), dtype=wp.vec3),
    multiccd_endvert=wp.zeros(shape=(naconmax, max_meshdegree), dtype=wp.vec3),
    multiccd_face1=wp.zeros(shape=(naconmax, max_npolygon), dtype=wp.vec3),
    multiccd_face2=wp.zeros(shape=(naconmax, max_npolygon), dtype=wp.vec3),
    # rne_postconstraint but also smooth
    cacc=tile(mjd.cacc, dtype=wp.spatial_vector),
    cfrc_int=tile(mjd.cfrc_int, dtype=wp.spatial_vector),
    cfrc_ext=tile(mjd.cfrc_ext, dtype=wp.spatial_vector),
    # tendon
    ten_length=tile(mjd.ten_length),
    ten_J=tile(ten_J),
    ten_Jdot=wp.zeros((nworld, mjm.ntendon, mjm.nv), dtype=float),
    ten_bias_coef=wp.zeros((nworld, mjm.ntendon), dtype=float),
    ten_wrapadr=tile(mjd.ten_wrapadr),
    ten_wrapnum=tile(mjd.ten_wrapnum),
    ten_actfrc=wp.zeros((nworld, mjm.ntendon), dtype=float),
    wrap_obj=tile(mjd.wrap_obj, dtype=wp.vec2i),
    wrap_xpos=tile(mjd.wrap_xpos, dtype=wp.spatial_vector),
    wrap_geom_xpos=wp.zeros((nworld, mjm.nwrap), dtype=wp.spatial_vector),
    # sensors
    sensordata=tile(mjd.sensordata),
    sensor_rangefinder_pnt=wp.zeros((nworld, nrangefinder), dtype=wp.vec3),
    sensor_rangefinder_vec=wp.zeros((nworld, nrangefinder), dtype=wp.vec3),
    sensor_rangefinder_dist=wp.zeros((nworld, nrangefinder), dtype=float),
    sensor_rangefinder_geomid=wp.zeros((nworld, nrangefinder), dtype=int),
    sensor_contact_nmatch=wp.zeros((nworld, nsensorcontact), dtype=int),
    sensor_contact_matchid=wp.zeros((nworld, nsensorcontact, types.MJ_MAXCONPAIR), dtype=int),
    sensor_contact_criteria=wp.zeros((nworld, nsensorcontact, types.MJ_MAXCONPAIR), dtype=float),
    sensor_contact_direction=wp.zeros((nworld, nsensorcontact, types.MJ_MAXCONPAIR), dtype=float),
    # ray
    ray_bodyexclude=wp.zeros(1, dtype=int),
    ray_dist=wp.zeros((nworld, 1), dtype=float),
    ray_geomid=wp.zeros((nworld, 1), dtype=int),
    # mul_m
    energy_vel_mul_m_skip=wp.zeros((nworld,), dtype=bool),
    inverse_mul_m_skip=wp.zeros((nworld,), dtype=bool),
    # actuator
    actuator_trntype_body_ncon=wp.zeros((nworld, np.sum(mjm.actuator_trntype == mujoco.mjtTrn.mjTRN_BODY)), dtype=int),
  )


def get_data_into(
  result: mujoco.MjData,
  mjm: mujoco.MjModel,
  d: types.Data,
):
  """Gets data from a device into an existing mujoco.MjData.

  Args:
    result (mujoco.MjData): The data object containing the current state and output arrays
                            (host).
    mjm (mujoco.MjModel): The model containing kinematic and dynamic information (host).
    d (Data): The data object containing the current state and output arrays (device).
  """
  if d.nworld > 1:
    raise NotImplementedError("only nworld == 1 supported for now")

  result.solver_niter[0] = d.solver_niter.numpy()[0]

  nacon = d.nacon.numpy()[0]
  nefc = d.nefc.numpy()[0]

  if nacon != result.ncon or nefc != result.nefc:
    mujoco._functions._realloc_con_efc(result, ncon=nacon, nefc=nefc)

  result.time = d.time.numpy()[0]
  result.energy = d.energy.numpy()[0]
  result.ne = d.ne.numpy()[0]
  result.qpos[:] = d.qpos.numpy()[0]
  result.qvel[:] = d.qvel.numpy()[0]
  result.qacc_warmstart = d.qacc_warmstart.numpy()[0]
  result.qfrc_applied = d.qfrc_applied.numpy()[0]
  result.mocap_pos = d.mocap_pos.numpy()[0]
  result.mocap_quat = d.mocap_quat.numpy()[0]
  result.qacc = d.qacc.numpy()[0]
  result.xanchor = d.xanchor.numpy()[0]
  result.xaxis = d.xaxis.numpy()[0]
  result.xmat = d.xmat.numpy().reshape((-1, 9))
  result.xpos = d.xpos.numpy()[0]
  result.xquat = d.xquat.numpy()[0]
  result.xipos = d.xipos.numpy()[0]
  result.ximat = d.ximat.numpy().reshape((-1, 9))
  result.subtree_com = d.subtree_com.numpy()[0]
  result.geom_xpos = d.geom_xpos.numpy()[0]
  result.geom_xmat = d.geom_xmat.numpy().reshape((-1, 9))
  result.site_xpos = d.site_xpos.numpy()[0]
  result.site_xmat = d.site_xmat.numpy().reshape((-1, 9))
  result.cam_xpos = d.cam_xpos.numpy()[0]
  result.cam_xmat = d.cam_xmat.numpy().reshape((-1, 9))
  result.light_xpos = d.light_xpos.numpy()[0]
  result.light_xdir = d.light_xdir.numpy()[0]
  result.cinert = d.cinert.numpy()[0]
  result.flexvert_xpos = d.flexvert_xpos.numpy()[0]
  result.flexedge_length = d.flexedge_length.numpy()[0]
  result.flexedge_velocity = d.flexedge_velocity.numpy()[0]
  result.cdof = d.cdof.numpy()[0]
  result.crb = d.crb.numpy()[0]
  result.qLDiagInv = d.qLDiagInv.numpy()[0]
  result.ctrl = d.ctrl.numpy()[0]
  result.ten_velocity = d.ten_velocity.numpy()[0]
  result.actuator_velocity = d.actuator_velocity.numpy()[0]
  result.actuator_force = d.actuator_force.numpy()[0]
  result.actuator_length = d.actuator_length.numpy()[0]
  mujoco.mju_dense2sparse(
    result.actuator_moment,
    d.actuator_moment.numpy()[0],
    result.moment_rownnz,
    result.moment_rowadr,
    result.moment_colind,
  )
  result.cvel = d.cvel.numpy()[0]
  result.cdof_dot = d.cdof_dot.numpy()[0]
  result.qfrc_bias = d.qfrc_bias.numpy()[0]
  result.qfrc_fluid = d.qfrc_fluid.numpy()[0]
  result.qfrc_passive = d.qfrc_passive.numpy()[0]
  result.subtree_linvel = d.subtree_linvel.numpy()[0]
  result.subtree_angmom = d.subtree_angmom.numpy()[0]
  result.qfrc_spring = d.qfrc_spring.numpy()[0]
  result.qfrc_damper = d.qfrc_damper.numpy()[0]
  result.qfrc_gravcomp = d.qfrc_gravcomp.numpy()[0]
  result.qfrc_fluid = d.qfrc_fluid.numpy()[0]
  result.qfrc_actuator = d.qfrc_actuator.numpy()[0]
  result.qfrc_smooth = d.qfrc_smooth.numpy()[0]
  result.qfrc_constraint = d.qfrc_constraint.numpy()[0]
  result.qfrc_inverse = d.qfrc_inverse.numpy()[0]
  result.qacc_smooth = d.qacc_smooth.numpy()[0]
  result.act = d.act.numpy()[0]
  result.act_dot = d.act_dot.numpy()[0]

  result.contact.dist[:] = d.contact.dist.numpy()[:nacon]
  result.contact.pos[:] = d.contact.pos.numpy()[:nacon]
  result.contact.frame[:] = d.contact.frame.numpy()[:nacon].reshape((-1, 9))
  result.contact.includemargin[:] = d.contact.includemargin.numpy()[:nacon]
  result.contact.friction[:] = d.contact.friction.numpy()[:nacon]
  result.contact.solref[:] = d.contact.solref.numpy()[:nacon]
  result.contact.solreffriction[:] = d.contact.solreffriction.numpy()[:nacon]
  result.contact.solimp[:] = d.contact.solimp.numpy()[:nacon]
  result.contact.dim[:] = d.contact.dim.numpy()[:nacon]
  result.contact.efc_address[:] = d.contact.efc_address.numpy()[:nacon, 0]

  if mujoco.mj_isSparse(mjm):
    result.qM[:] = d.qM.numpy()[0, 0]
    result.qLD[:] = d.qLD.numpy()[0, 0]
    # TODO(team): set efc_J after fix to _realloc_con_efc lands
    # efc_J = d.efc_J.numpy()[0, :nefc]
    # mujoco.mju_dense2sparse(
    #   result.efc_J, efc_J, result.efc_J_rownnz, result.efc_J_rowadr, result.efc_J_colind
    # )
  else:
    qM = d.qM.numpy()
    adr = 0
    for i in range(mjm.nv):
      j = i
      while j >= 0:
        result.qM[adr] = qM[0, i, j]
        j = mjm.dof_parentid[j]
        adr += 1
    mujoco.mj_factorM(mjm, result)
    # TODO(team): set efc_J after fix to _realloc_con_efc lands
    # if nefc > 0:
    #   result.efc_J[:nefc * mjm.nv] = d.efc_J.numpy()[:nefc].flatten()
  result.xfrc_applied[:] = d.xfrc_applied.numpy()[0]
  result.eq_active[:] = d.eq_active.numpy()[0]

  # TODO(team): set these efc_* fields after fix to _realloc_con_efc
  # Safely copy only up to the minimum of the destination and source sizes
  # n = min(result.efc_D.shape[0], d.efc.D.numpy()[:nefc].shape[0])
  # result.efc_D[:n] = d.efc.D.numpy()[:nefc][:n]
  # n_pos = min(result.efc_pos.shape[0], d.efc.pos.numpy()[:nefc].shape[0])
  # result.efc_pos[:n_pos] = d.efc.pos.numpy()[:nefc][:n_pos]

  # n_aref = min(result.efc_aref.shape[0], d.efc.aref.numpy()[:nefc].shape[0])
  # result.efc_aref[:n_aref] = d.efc.aref.numpy()[:nefc][:n_aref]

  # n_force = min(result.efc_force.shape[0], d.efc.force.numpy()[:nefc].shape[0])
  # result.efc_force[:n_force] = d.efc.force.numpy()[:nefc][:n_force]

  # n_margin = min(result.efc_margin.shape[0], d.efc.margin.numpy()[:nefc].shape[0])
  # result.efc_margin[:n_margin] = d.efc.margin.numpy()[:nefc][:n_margin]

  result.cacc[:] = d.cacc.numpy()[0]
  result.cfrc_int[:] = d.cfrc_int.numpy()[0]
  result.cfrc_ext[:] = d.cfrc_ext.numpy()[0]

  # TODO: other efc_ fields, anything else missing

  # tendon
  result.ten_length[:] = d.ten_length.numpy()[0]
  result.ten_J[:] = d.ten_J.numpy()[0]
  result.ten_wrapadr[:] = d.ten_wrapadr.numpy()[0]
  result.ten_wrapnum[:] = d.ten_wrapnum.numpy()[0]
  result.wrap_obj[:] = d.wrap_obj.numpy()[0]
  result.wrap_xpos[:] = d.wrap_xpos.numpy()[0]

  # sensors
  result.sensordata[:] = d.sensordata.numpy()


# TODO(thowell): shared @wp.func for _reset kernel?


@wp.kernel
def _reset_xfrc_applied_all(xfrc_applied_out: wp.array2d(dtype=wp.spatial_vector)):
  worldid, bodyid, elemid = wp.tid()
  xfrc_applied_out[worldid, bodyid][elemid] = 0.0


@wp.kernel
def _reset_xfrc_applied(reset_in: wp.array(dtype=bool), xfrc_applied_out: wp.array2d(dtype=wp.spatial_vector)):
  worldid, bodyid, elemid = wp.tid()

  if not reset_in[worldid]:
    return

  xfrc_applied_out[worldid, bodyid][elemid] = 0.0


@wp.kernel
def _reset_qM_all(qM_out: wp.array3d(dtype=float)):
  worldid, elemid1, elemid2 = wp.tid()
  qM_out[worldid, elemid1, elemid2] = 0.0


@wp.kernel
def _reset_qM(reset_in: wp.array(dtype=bool), qM_out: wp.array3d(dtype=float)):
  worldid, elemid1, elemid2 = wp.tid()

  if not reset_in[worldid]:
    return

  qM_out[worldid, elemid1, elemid2] = 0.0


@wp.kernel
def _reset_nworld_all(
  # Model:
  nq: int,
  nv: int,
  nu: int,
  na: int,
  neq: int,
  nsensordata: int,
  qpos0: wp.array2d(dtype=float),
  eq_active0: wp.array(dtype=bool),
  # Data in:
  nworld_in: int,
  # Data out:
  solver_niter_out: wp.array(dtype=int),
  nacon_out: wp.array(dtype=int),
  ne_out: wp.array(dtype=int),
  ne_connect_out: wp.array(dtype=int),
  ne_weld_out: wp.array(dtype=int),
  ne_jnt_out: wp.array(dtype=int),
  ne_ten_out: wp.array(dtype=int),
  nf_out: wp.array(dtype=int),
  nl_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  nsolving_out: wp.array(dtype=int),
  time_out: wp.array(dtype=float),
  energy_out: wp.array(dtype=wp.vec2),
  qpos_out: wp.array2d(dtype=float),
  qvel_out: wp.array2d(dtype=float),
  act_out: wp.array2d(dtype=float),
  qacc_warmstart_out: wp.array2d(dtype=float),
  ctrl_out: wp.array2d(dtype=float),
  qfrc_applied_out: wp.array2d(dtype=float),
  eq_active_out: wp.array2d(dtype=bool),
  qacc_out: wp.array2d(dtype=float),
  act_dot_out: wp.array2d(dtype=float),
  sensordata_out: wp.array2d(dtype=float),
):
  worldid = wp.tid()

  solver_niter_out[worldid] = 0
  if worldid == 0:
    nacon_out[0] = 0
  ne_out[worldid] = 0
  ne_connect_out[worldid] = 0
  ne_weld_out[worldid] = 0
  ne_jnt_out[worldid] = 0
  ne_ten_out[worldid] = 0
  nf_out[worldid] = 0
  nl_out[worldid] = 0
  nefc_out[worldid] = 0
  if worldid == 0:
    nsolving_out[0] = nworld_in
  time_out[worldid] = 0.0
  energy_out[worldid] = wp.vec2(0.0, 0.0)
  for i in range(nq):
    qpos_out[worldid, i] = qpos0[worldid, i]
    if i < nv:
      qvel_out[worldid, i] = 0.0
      qacc_warmstart_out[worldid, i] = 0.0
      qfrc_applied_out[worldid, i] = 0.0
      qacc_out[worldid, i] = 0.0
  for i in range(nu):
    ctrl_out[worldid, i] = 0.0
    if i < na:
      act_out[worldid, i] = 0.0
      act_dot_out[worldid, i] = 0.0
  for i in range(neq):
    eq_active_out[worldid, i] = eq_active0[i]
  for i in range(nsensordata):
    sensordata_out[worldid, i] = 0.0


@wp.kernel
def _reset_nworld(
  # Model:
  nq: int,
  nv: int,
  nu: int,
  na: int,
  neq: int,
  nsensordata: int,
  qpos0: wp.array2d(dtype=float),
  eq_active0: wp.array(dtype=bool),
  # Data in:
  nworld_in: int,
  # In:
  reset_in: wp.array(dtype=bool),
  # Data out:
  solver_niter_out: wp.array(dtype=int),
  nacon_out: wp.array(dtype=int),
  ne_out: wp.array(dtype=int),
  ne_connect_out: wp.array(dtype=int),
  ne_weld_out: wp.array(dtype=int),
  ne_jnt_out: wp.array(dtype=int),
  ne_ten_out: wp.array(dtype=int),
  nf_out: wp.array(dtype=int),
  nl_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  nsolving_out: wp.array(dtype=int),
  time_out: wp.array(dtype=float),
  energy_out: wp.array(dtype=wp.vec2),
  qpos_out: wp.array2d(dtype=float),
  qvel_out: wp.array2d(dtype=float),
  act_out: wp.array2d(dtype=float),
  qacc_warmstart_out: wp.array2d(dtype=float),
  ctrl_out: wp.array2d(dtype=float),
  qfrc_applied_out: wp.array2d(dtype=float),
  eq_active_out: wp.array2d(dtype=bool),
  qacc_out: wp.array2d(dtype=float),
  act_dot_out: wp.array2d(dtype=float),
  sensordata_out: wp.array2d(dtype=float),
):
  worldid = wp.tid()

  if not reset_in[worldid]:
    return

  solver_niter_out[worldid] = 0
  if worldid == 0:
    nacon_out[0] = 0
  ne_out[worldid] = 0
  ne_connect_out[worldid] = 0
  ne_weld_out[worldid] = 0
  ne_jnt_out[worldid] = 0
  ne_ten_out[worldid] = 0
  nf_out[worldid] = 0
  nl_out[worldid] = 0
  nefc_out[worldid] = 0
  if worldid == 0:
    nsolving_out[0] = nworld_in
  time_out[worldid] = 0.0
  energy_out[worldid] = wp.vec2(0.0, 0.0)
  for i in range(nq):
    qpos_out[worldid, i] = qpos0[worldid, i]
    if i < nv:
      qvel_out[worldid, i] = 0.0
      qacc_warmstart_out[worldid, i] = 0.0
      qfrc_applied_out[worldid, i] = 0.0
      qacc_out[worldid, i] = 0.0
  for i in range(nu):
    ctrl_out[worldid, i] = 0.0
    if i < na:
      act_out[worldid, i] = 0.0
      act_dot_out[worldid, i] = 0.0
  for i in range(neq):
    eq_active_out[worldid, i] = eq_active0[i]
  for i in range(nsensordata):
    sensordata_out[worldid, i] = 0.0


@wp.kernel
def _reset_mocap_all(
  # Model:
  body_mocapid: wp.array(dtype=int),
  body_pos: wp.array2d(dtype=wp.vec3),
  body_quat: wp.array2d(dtype=wp.quat),
  # Data out:
  mocap_pos_out: wp.array2d(dtype=wp.vec3),
  mocap_quat_out: wp.array2d(dtype=wp.quat),
):
  worldid, bodyid = wp.tid()

  mocapid = body_mocapid[bodyid]

  if mocapid >= 0:
    mocap_pos_out[worldid, mocapid] = body_pos[worldid, bodyid]
    mocap_quat_out[worldid, mocapid] = body_quat[worldid, bodyid]


@wp.kernel
def _reset_mocap(
  # Model:
  body_mocapid: wp.array(dtype=int),
  body_pos: wp.array2d(dtype=wp.vec3),
  body_quat: wp.array2d(dtype=wp.quat),
  # In:
  reset_in: wp.array(dtype=bool),
  # Data out:
  mocap_pos_out: wp.array2d(dtype=wp.vec3),
  mocap_quat_out: wp.array2d(dtype=wp.quat),
):
  worldid, bodyid = wp.tid()

  if not reset_in[worldid]:
    return

  mocapid = body_mocapid[bodyid]

  if mocapid >= 0:
    mocap_pos_out[worldid, mocapid] = body_pos[worldid, bodyid]
    mocap_quat_out[worldid, mocapid] = body_quat[worldid, bodyid]


@wp.kernel
def _reset_contact_all(
  # Data in:
  nacon_in: wp.array(dtype=int),
  # In:
  nefcaddress: int,
  # Data out:
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=types.vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=types.vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_efc_address_out: wp.array2d(dtype=int),
  contact_worldid_out: wp.array(dtype=int),
):
  conid = wp.tid()

  if conid >= nacon_in[0]:
    return

  contact_dist_out[conid] = 0.0
  contact_pos_out[conid] = wp.vec3(0.0, 0.0, 0.0)
  contact_frame_out[conid] = wp.mat33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  contact_includemargin_out[conid] = 0.0
  contact_friction_out[conid] = types.vec5(0.0, 0.0, 0.0, 0.0, 0.0)
  contact_solref_out[conid] = wp.vec2(0.0, 0.0)
  contact_solreffriction_out[conid] = wp.vec2(0.0, 0.0)
  contact_solimp_out[conid] = types.vec5(0.0, 0.0, 0.0, 0.0, 0.0)
  contact_dim_out[conid] = 0
  contact_geom_out[conid] = wp.vec2i(0, 0)
  for i in range(nefcaddress):
    contact_efc_address_out[conid, i] = 0
  contact_worldid_out[conid] = 0


@wp.kernel
def _reset_contact(
  # Data in:
  nacon_in: wp.array(dtype=int),
  # In:
  reset_in: wp.array(dtype=bool),
  nefcaddress: int,
  # Data out:
  contact_dist_out: wp.array(dtype=float),
  contact_pos_out: wp.array(dtype=wp.vec3),
  contact_frame_out: wp.array(dtype=wp.mat33),
  contact_includemargin_out: wp.array(dtype=float),
  contact_friction_out: wp.array(dtype=types.vec5),
  contact_solref_out: wp.array(dtype=wp.vec2),
  contact_solreffriction_out: wp.array(dtype=wp.vec2),
  contact_solimp_out: wp.array(dtype=types.vec5),
  contact_dim_out: wp.array(dtype=int),
  contact_geom_out: wp.array(dtype=wp.vec2i),
  contact_efc_address_out: wp.array2d(dtype=int),
  contact_worldid_out: wp.array(dtype=int),
):
  conid = wp.tid()

  if conid >= nacon_in[0]:
    return

  worldid = contact_worldid_out[conid]
  if worldid >= 0:
    if not reset_in[worldid]:
      return

  contact_dist_out[conid] = 0.0
  contact_pos_out[conid] = wp.vec3(0.0)
  contact_frame_out[conid] = wp.mat33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  contact_includemargin_out[conid] = 0.0
  contact_friction_out[conid] = types.vec5(0.0, 0.0, 0.0, 0.0, 0.0)
  contact_solref_out[conid] = wp.vec2(0.0, 0.0)
  contact_solreffriction_out[conid] = wp.vec2(0.0, 0.0)
  contact_solimp_out[conid] = types.vec5(0.0, 0.0, 0.0, 0.0, 0.0)
  contact_dim_out[conid] = 0
  contact_geom_out[conid] = wp.vec2i(0, 0)
  for i in range(nefcaddress):
    contact_efc_address_out[conid, i] = 0
  contact_worldid_out[conid] = 0


def reset_data(m: types.Model, d: types.Data, reset: Optional[wp.array] = None):
  """Clear data, set defaults."""
  if m.opt.is_sparse:
    qM_dim = (1, m.nM)
  else:
    qM_dim = (m.nv, m.nv)

  if reset is not None:
    wp.launch(_reset_xfrc_applied, dim=(d.nworld, m.nbody, 6), inputs=[reset], outputs=[d.xfrc_applied])
    wp.launch(_reset_qM, dim=(d.nworld, qM_dim[0], qM_dim[1]), inputs=[reset], outputs=[d.qM])

    # set mocap_pos/quat = body_pos/quat for mocap bodies
    wp.launch(
      _reset_mocap,
      dim=(d.nworld, m.nbody),
      inputs=[m.body_mocapid, m.body_pos, m.body_quat, reset],
      outputs=[d.mocap_pos, d.mocap_quat],
    )

    # clear contacts
    wp.launch(
      _reset_contact,
      dim=d.naconmax,
      inputs=[d.nacon, reset, d.contact.efc_address.shape[1]],
      outputs=[
        d.contact.dist,
        d.contact.pos,
        d.contact.frame,
        d.contact.includemargin,
        d.contact.friction,
        d.contact.solref,
        d.contact.solreffriction,
        d.contact.solimp,
        d.contact.dim,
        d.contact.geom,
        d.contact.efc_address,
        d.contact.worldid,
      ],
    )

    wp.launch(
      _reset_nworld,
      dim=d.nworld,
      inputs=[m.nq, m.nv, m.nu, m.na, m.neq, m.nsensordata, m.qpos0, m.eq_active0, d.nworld, reset],
      outputs=[
        d.solver_niter,
        d.nacon,
        d.ne,
        d.ne_connect,
        d.ne_weld,
        d.ne_jnt,
        d.ne_ten,
        d.nf,
        d.nl,
        d.nefc,
        d.nsolving,
        d.time,
        d.energy,
        d.qpos,
        d.qvel,
        d.act,
        d.qacc_warmstart,
        d.ctrl,
        d.qfrc_applied,
        d.eq_active,
        d.qacc,
        d.act_dot,
        d.sensordata,
      ],
    )
  else:
    wp.launch(_reset_xfrc_applied_all, dim=(d.nworld, m.nbody, 6), outputs=[d.xfrc_applied])
    wp.launch(_reset_qM_all, dim=(d.nworld, qM_dim[0], qM_dim[1]), outputs=[d.qM])
    wp.launch(
      _reset_mocap_all,
      dim=(d.nworld, m.nbody),
      inputs=[m.body_mocapid, m.body_pos, m.body_quat],
      outputs=[d.mocap_pos, d.mocap_quat],
    )
    wp.launch(
      _reset_contact_all,
      dim=d.naconmax,
      inputs=[d.nacon, d.contact.efc_address.shape[1]],
      outputs=[
        d.contact.dist,
        d.contact.pos,
        d.contact.frame,
        d.contact.includemargin,
        d.contact.friction,
        d.contact.solref,
        d.contact.solreffriction,
        d.contact.solimp,
        d.contact.dim,
        d.contact.geom,
        d.contact.efc_address,
        d.contact.worldid,
      ],
    )
    wp.launch(
      _reset_nworld_all,
      dim=d.nworld,
      inputs=[m.nq, m.nv, m.nu, m.na, m.neq, m.nsensordata, m.qpos0, m.eq_active0, d.nworld],
      outputs=[
        d.solver_niter,
        d.nacon,
        d.ne,
        d.ne_connect,
        d.ne_weld,
        d.ne_jnt,
        d.ne_ten,
        d.nf,
        d.nl,
        d.nefc,
        d.nsolving,
        d.time,
        d.energy,
        d.qpos,
        d.qvel,
        d.act,
        d.qacc_warmstart,
        d.ctrl,
        d.qfrc_applied,
        d.eq_active,
        d.qacc,
        d.act_dot,
        d.sensordata,
      ],
    )


def override_model(model: Union[types.Model, mujoco.MjModel], overrides: Union[dict[str, Any], Sequence[str]]):
  """Overrides model parameters.

  Overrides are of the format:
    opt.iterations = 1
    opt.ls_parallel = True
    opt.cone = pyramidal
    opt.disableflags = contact | spring
  """

  enum_fields = {
    "opt.broadphase": types.BroadphaseType,
    "opt.broadphase_filter": types.BroadphaseFilter,
    "opt.cone": types.ConeType,
    "opt.disableflags": types.DisableBit,
    "opt.enableflags": types.EnableBit,
    "opt.integrator": types.IntegratorType,
    "opt.solver": types.SolverType,
  }
  mjw_only_fields = {"opt.broadphase", "opt.broadphase_filter", "opt.ls_parallel", "opt.graph_conditional"}
  mj_only_fields = {"opt.jacobian"}

  if not isinstance(overrides, dict):
    overrides_dict = {}
    for override in overrides:
      if "=" not in override:
        raise ValueError(f"Invalid override format: {override}")
      k, v = override.split("=", 1)
      overrides_dict[k.strip()] = v.strip()
    overrides = overrides_dict

  for key, val in overrides.items():
    # skip overrides on MjModel for properties that are only on mjw.Model
    if key in mjw_only_fields and isinstance(model, mujoco.MjModel):
      continue
    if key in mj_only_fields and isinstance(model, types.Model):
      continue

    obj, attrs = model, key.split(".")
    for i, attr in enumerate(attrs):
      if not hasattr(obj, attr):
        raise ValueError(f"Unrecognized model field: {key}")
      if i < len(attrs) - 1:
        obj = getattr(obj, attr)
        continue

      typ = type(getattr(obj, attr))

      if key in enum_fields and isinstance(val, str):
        # special case: enum value
        enum_members = val.split("|")
        val = 0
        for enum_member in enum_members:
          enum_member = enum_member.strip().upper()
          if enum_member not in enum_fields[key].__members__:
            raise ValueError(f"Unrecognized enum value for {enum_fields[key].__name__}: {enum_member}")
          val |= int(enum_fields[key][enum_member])
      elif typ is bool and isinstance(val, str):
        # special case: "true", "TRUE", "false", "FALSE" etc.
        if val.upper() not in ("TRUE", "FALSE"):
          raise ValueError(f"Unrecognized value for field: {key}")
        val = val.upper() == "TRUE"
      else:
        val = typ(val)

      setattr(obj, attr, val)


def find_keys(model: mujoco.MjModel, keyname_prefix: str) -> list[int]:
  """Finds keyframes that start with keyname_prefix."""
  keys = []

  for keyid in range(model.nkey):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, keyid)
    if name.startswith(keyname_prefix):
      keys.append(keyid)

  return keys


def make_trajectory(model: mujoco.MjModel, keys: list[int]) -> np.ndarray:
  """Make a ctrl trajectory with linear interpolation."""
  ctrls = []
  prev_ctrl_key = np.zeros(model.nu, dtype=np.float64)
  prev_time, time = 0.0, 0.0

  for key in keys:
    ctrl_key, ctrl_time = model.key_ctrl[key], model.key_time[key]
    if not ctrls and ctrl_time != 0.0:
      raise ValueError("first keyframe must have time 0.0")
    elif ctrls and ctrl_time <= prev_time:
      raise ValueError("keyframes must be in time order")

    while time < ctrl_time:
      frac = (time - prev_time) / (ctrl_time - prev_time)
      ctrls.append(prev_ctrl_key * (1 - frac) + ctrl_key * frac)
      time += model.opt.timestep

    ctrls.append(ctrl_key)
    time += model.opt.timestep
    prev_ctrl_key = ctrl_key
    prev_time = time

  return np.array(ctrls)
