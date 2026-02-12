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

import dataclasses
import importlib.metadata
import warnings
from typing import Any, Optional, Sequence

import mujoco
import numpy as np
import packaging.version
import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import bvh
from mujoco.mjx.third_party.mujoco_warp._src import render_util
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src import warp_util


def _is_mujoco_dev() -> bool:
  """Checks if mujoco version is > 3.4.0."""
  version_str = getattr(mujoco, "__version__", None)
  if not version_str:
    version_str = importlib.metadata.version("mujoco")
  version_str = version_str.split("-")[0].split(".dev")[0]
  return packaging.version.parse(version_str) > packaging.version.parse("3.4.0")


BLEEDING_EDGE_MUJOCO = _is_mujoco_dev()


def _create_array(data: Any, spec: wp.array, sizes: dict[str, int]) -> wp.array | None:
  """Creates a warp array and populates it with data.

  The array shape is determined by a field spec referencing MjModel / MjData array sizes.
  """
  shape = None
  if spec.shape != (0,):
    shape = tuple(sizes[dim] if isinstance(dim, str) else dim for dim in spec.shape)

  if data is None and shape is None:
    return None  # nothing to do
  elif data is None:
    array = wp.zeros(shape, dtype=spec.dtype)
  else:
    array = wp.array(np.array(data), dtype=spec.dtype, shape=shape)

  if spec.shape[0] == "*":
    # add private attribute for JAX to determine which fields are batched
    array._is_batched = True
    # also set stride 0 to 0 which is expected legacy behavior (but is deprecated)
    array.strides = (0,) + array.strides[1:]
  return array


def is_sparse(mjm: mujoco.MjModel) -> bool:
  if mjm.opt.jacobian == mujoco.mjtJacobian.mjJAC_AUTO:
    if mjm.nv > 32:
      return True
    else:
      return False
  else:
    return bool(mujoco.mj_isSparse(mjm))


def put_model(mjm: mujoco.MjModel) -> types.Model:
  """Creates a model on device.

  Args:
    mjm: The model containing kinematic and dynamic information (host).

  Returns:
    The model containing kinematic and dynamic information (device).
  """
  # check for compatible cuda toolkit and driver versions
  warp_util.check_toolkit_driver()

  # model: check supported features in array types
  for field, field_type, mj_type in (
    (mjm.actuator_trntype, types.TrnType, mujoco.mjtTrn),
    (mjm.actuator_dyntype, types.DynType, mujoco.mjtDyn),
    (mjm.actuator_gaintype, types.GainType, mujoco.mjtGain),
    (mjm.actuator_biastype, types.BiasType, mujoco.mjtBias),
    (mjm.eq_type, types.EqType, mujoco.mjtEq),
    (mjm.geom_type, types.GeomType, mujoco.mjtGeom),
    (mjm.sensor_type, types.SensorType, mujoco.mjtSensor),
    (mjm.wrap_type, types.WrapType, mujoco.mjtWrap),
  ):
    missing = ~np.isin(field, field_type)
    if missing.any():
      names = [mj_type(v).name for v in field[missing]]
      raise NotImplementedError(f"{names} not supported.")

  # opt: check supported features in scalar types
  for field, field_type, mj_type in (
    (mjm.opt.integrator, types.IntegratorType, mujoco.mjtIntegrator),
    (mjm.opt.cone, types.ConeType, mujoco.mjtCone),
    (mjm.opt.solver, types.SolverType, mujoco.mjtSolver),
  ):
    if field not in set(field_type):
      raise NotImplementedError(f"{mj_type(field).name} is unsupported.")

  # opt: check supported features in scalar flag types
  for field, field_type, mj_type in (
    (mjm.opt.disableflags, types.DisableBit, mujoco.mjtDisableBit),
    (mjm.opt.enableflags, types.EnableBit, mujoco.mjtEnableBit),
  ):
    unsupported = field & ~np.bitwise_or.reduce(field_type)
    if unsupported:
      raise NotImplementedError(f"{mj_type(unsupported).name} is unsupported.")

  if ((mjm.flex_contype != 0) | (mjm.flex_conaffinity != 0)).any():
    raise NotImplementedError("Flex collisions are not implemented.")

  if mjm.opt.noslip_iterations > 0:
    raise NotImplementedError(f"noslip solver not implemented.")

  if (mjm.opt.viscosity > 0 or mjm.opt.density > 0) and mjm.opt.integrator in (
    mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    mujoco.mjtIntegrator.mjINT_IMPLICIT,
  ):
    raise NotImplementedError(f"Implicit integrators and fluid model not implemented.")

  if (mjm.body_plugin != -1).any():
    raise NotImplementedError("Body plugins not supported.")

  if (mjm.actuator_plugin != -1).any():
    raise NotImplementedError("Actuator plugins not supported.")

  if (mjm.sensor_plugin != -1).any():
    raise NotImplementedError("Sensor plugins not supported.")

  # TODO(team): remove after _update_gradient for Newton uses tile operations for islands
  nv_max = 60
  if mjm.nv > nv_max and mjm.opt.jacobian == mujoco.mjtJacobian.mjJAC_DENSE:
    raise ValueError(f"Dense is unsupported for nv > {nv_max} (nv = {mjm.nv}).")

  collision_sensors = (mujoco.mjtSensor.mjSENS_GEOMDIST, mujoco.mjtSensor.mjSENS_GEOMNORMAL, mujoco.mjtSensor.mjSENS_GEOMFROMTO)
  is_collision_sensor = np.isin(mjm.sensor_type, collision_sensors)

  def not_implemented(objtype, objid, geomtype):
    if objtype == mujoco.mjtObj.mjOBJ_BODY:
      geomnum = mjm.body_geomnum[objid]
      geomadr = mjm.body_geomadr[objid]
      for geomid in range(geomadr, geomadr + geomnum):
        if mjm.geom_type[geomid] == geomtype:
          return True
    elif objtype == mujoco.mjtObj.mjOBJ_GEOM:
      if mjm.geom_type[objid] == geomtype:
        return True
    return False

  def _check_friction(name: str, id_: int, condim: int, friction, checks):
    for min_condim, indices in checks:
      if condim >= min_condim:
        for idx in indices:
          if friction[idx] < types.MJ_MINMU:
            warnings.warn(
              f"{name} {id_}: friction[{idx}] ({friction[idx]}) < MJ_MINMU ({types.MJ_MINMU}) with condim={condim} may cause NaN"
            )

  for geomid in range(mjm.ngeom):
    _check_friction("geom", geomid, mjm.geom_condim[geomid], mjm.geom_friction[geomid], [(3, [0]), (4, [1]), (6, [2])])

  for pairid in range(mjm.npair):
    _check_friction("pair", pairid, mjm.pair_dim[pairid], mjm.pair_friction[pairid], [(3, [0]), (4, [1, 2]), (6, [3, 4])])

  # create opt
  opt_kwargs = {f.name: getattr(mjm.opt, f.name, None) for f in dataclasses.fields(types.Option)}
  if hasattr(mjm.opt, "impratio"):
    opt_kwargs["impratio_invsqrt"] = 1.0 / np.sqrt(np.maximum(mjm.opt.impratio, mujoco.mjMINVAL))
  opt = types.Option(**opt_kwargs)

  # C MuJoCo tolerance was chosen for float64 architecture, but we default to float32 on GPU
  # adjust the tolerance for lower precision, to avoid the solver spending iterations needlessly
  # bouncing around the optimal solution
  opt.tolerance = max(opt.tolerance, 1e-6)

  # warp only fields
  ls_parallel_id = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_NUMERIC, "ls_parallel")
  opt.ls_parallel = (ls_parallel_id > -1) and (mjm.numeric_data[mjm.numeric_adr[ls_parallel_id]] == 1)
  opt.ls_parallel_min_step = 1.0e-6  # TODO(team): determine good default setting
  opt.broadphase = types.BroadphaseType.NXN
  opt.broadphase_filter = types.BroadphaseFilter.PLANE | types.BroadphaseFilter.SPHERE | types.BroadphaseFilter.OBB
  opt.graph_conditional = True
  opt.run_collision_detection = True
  contact_sensor_maxmatch_id = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_NUMERIC, "contact_sensor_maxmatch")
  if contact_sensor_maxmatch_id > -1:
    opt.contact_sensor_maxmatch = mjm.numeric_data[mjm.numeric_adr[contact_sensor_maxmatch_id]]
  else:
    opt.contact_sensor_maxmatch = 64

  # place opt on device
  for f in dataclasses.fields(types.Option):
    if isinstance(f.type, wp.array):
      setattr(opt, f.name, _create_array(getattr(opt, f.name), f.type, {"*": 1}))
    else:
      setattr(opt, f.name, f.type(getattr(opt, f.name)))

  # create stat
  stat = types.Statistic(meaninertia=_create_array([mjm.stat.meaninertia], types.array("*", float), {"*": 1}))

  # create model
  m = types.Model(**{f.name: getattr(mjm, f.name, None) for f in dataclasses.fields(types.Model)})

  m.opt = opt
  m.stat = stat

  m.nv_pad = _get_padded_sizes(
    mjm.nv, 0, is_sparse(mjm), types.TILE_SIZE_JTDAJ_SPARSE if is_sparse(mjm) else types.TILE_SIZE_JTDAJ_DENSE
  )[1]
  m.nacttrnbody = (mjm.actuator_trntype == mujoco.mjtTrn.mjTRN_BODY).sum()
  m.nsensortaxel = mjm.mesh_vertnum[mjm.sensor_objid[mjm.sensor_type == mujoco.mjtSensor.mjSENS_TACTILE]].sum()
  m.nsensorcontact = (mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT).sum()
  m.nrangefinder = (mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER).sum()
  m.nmaxcondim = np.concatenate(([0], mjm.geom_condim, mjm.pair_dim)).max()
  m.nmaxpyramid = np.maximum(1, 2 * (m.nmaxcondim - 1))
  m.has_sdf_geom = (mjm.geom_type == mujoco.mjtGeom.mjGEOM_SDF).any()
  m.block_dim = types.BlockDim()
  m.is_sparse = is_sparse(mjm)
  m.has_fluid = mjm.opt.wind.any() or mjm.opt.density > 0 or mjm.opt.viscosity > 0

  # body ids grouped by tree level (depth-based traversal)
  bodies, body_depth = {}, np.zeros(mjm.nbody, dtype=int) - 1
  for i in range(mjm.nbody):
    body_depth[i] = body_depth[mjm.body_parentid[i]] + 1
    bodies.setdefault(body_depth[i], []).append(i)
  m.body_tree = tuple(wp.array(bodies[i], dtype=int) for i in sorted(bodies))

  # branch-based traversal data
  children_count = np.bincount(mjm.body_parentid[1:], minlength=mjm.nbody)
  ancestor_chain = lambda b: ancestor_chain(mjm.body_parentid[b]) + [b] if b else []
  branches = [ancestor_chain(l) for l in np.where(children_count[1:] == 0)[0] + 1]
  m.nbranch = len(branches)

  body_branches = []
  body_branch_start = []
  offset = 0

  for branch in branches:
    body_branches.extend(branch)
    body_branch_start.append(offset)
    offset += len(branch)
  body_branch_start.append(offset)

  m.body_branches = np.array(body_branches, dtype=int)
  m.body_branch_start = np.array(body_branch_start, dtype=int)

  m.mocap_bodyid = np.arange(mjm.nbody)[mjm.body_mocapid >= 0]
  m.mocap_bodyid = m.mocap_bodyid[mjm.body_mocapid[mjm.body_mocapid >= 0].argsort()]
  m.body_fluid_ellipsoid = np.zeros(mjm.nbody, dtype=bool)
  m.body_fluid_ellipsoid[mjm.geom_bodyid[mjm.geom_fluid.reshape(mjm.ngeom, mujoco.mjNFLUID)[:, 0] > 0]] = True
  jnt_limited_slide_hinge = mjm.jnt_limited & np.isin(mjm.jnt_type, (mujoco.mjtJoint.mjJNT_SLIDE, mujoco.mjtJoint.mjJNT_HINGE))
  m.jnt_limited_slide_hinge_adr = np.nonzero(jnt_limited_slide_hinge)[0]
  m.jnt_limited_ball_adr = np.nonzero(mjm.jnt_limited & (mjm.jnt_type == mujoco.mjtJoint.mjJNT_BALL))[0]
  m.dof_tri_row, m.dof_tri_col = np.tril_indices(mjm.nv)

  # precalculated geom pairs
  filterparent = not (mjm.opt.disableflags & types.DisableBit.FILTERPARENT)

  geom1, geom2 = np.triu_indices(mjm.ngeom, k=1)
  m.nxn_geom_pair = np.stack((geom1, geom2), axis=1)

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

  nxn_pairid_contact = -1 * np.ones(len(geom1), dtype=int)
  nxn_pairid_contact[~(mask & ~self_collision & ~parent_child_collision & ~exclude)] = -2

  # contact pairs
  def upper_tri_index(n, i, j):
    i, j = (j, i) if j < i else (i, j)
    return (i * (2 * n - i - 3)) // 2 + j - 1

  for i in range(mjm.npair):
    nxn_pairid_contact[upper_tri_index(mjm.ngeom, mjm.pair_geom1[i], mjm.pair_geom2[i])] = i

  sensor_collision_adr = np.nonzero(is_collision_sensor)[0]
  collision_sensor_adr = np.full(mjm.nsensor, -1)
  collision_sensor_adr[sensor_collision_adr] = np.arange(len(sensor_collision_adr))

  nxn_pairid_collision = -1 * np.ones(len(geom1), dtype=int)
  pairids = []
  sensor_collision_start_adr = []
  for i in range(sensor_collision_adr.size):
    sensorid = sensor_collision_adr[i]
    objtype = mjm.sensor_objtype[sensorid]
    objid = mjm.sensor_objid[sensorid]
    reftype = mjm.sensor_reftype[sensorid]
    refid = mjm.sensor_refid[sensorid]

    # get lists of geoms to collide
    if objtype == types.ObjType.BODY:
      n1 = mjm.body_geomnum[objid]
      id1 = mjm.body_geomadr[objid]
    else:
      n1 = 1
      id1 = objid
    if reftype == types.ObjType.BODY:
      n2 = mjm.body_geomnum[refid]
      id2 = mjm.body_geomadr[refid]
    else:
      n2 = 1
      id2 = refid

    # collide all pairs
    for geom1id in range(id1, id1 + n1):
      for geom2id in range(id2, id2 + n2):
        pairid = upper_tri_index(mjm.ngeom, geom1id, geom2id)

        if pairid in pairids:
          sensor_collision_start_adr.append(nxn_pairid_collision[pairid])
        else:
          npairids = len(pairids)
          nxn_pairid_collision[pairid] = npairids
          sensor_collision_start_adr.append(npairids)
          pairids.append(pairid)

  m.nsensorcollision = (nxn_pairid_collision >= 0).sum()
  m.sensor_collision_start_adr = np.array(sensor_collision_start_adr)
  nxn_include = (nxn_pairid_contact > -2) | (nxn_pairid_collision >= 0)

  if nxn_include.sum() < 250_000:
    opt.broadphase = types.BroadphaseType.NXN
  elif mjm.ngeom < 1000:
    opt.broadphase = types.BroadphaseType.SAP_TILE
  else:
    opt.broadphase = types.BroadphaseType.SAP_SEGMENTED

  m.nxn_geom_pair_filtered = m.nxn_geom_pair[nxn_include]
  m.nxn_pairid = np.hstack([nxn_pairid_contact.reshape((-1, 1)), nxn_pairid_collision.reshape((-1, 1))])
  m.nxn_pairid_filtered = m.nxn_pairid[nxn_include]

  # count contact pair types
  def geom_trid_index(i, j):
    i, j = (j, i) if j < i else (i, j)
    return (i * (2 * len(types.GeomType) - i - 1)) // 2 + j

  m.geom_pair_type_count = tuple(
    np.bincount(
      [geom_trid_index(mjm.geom_type[geom1[i]], mjm.geom_type[geom2[i]]) for i in np.arange(len(geom1)) if nxn_include[i]],
      minlength=len(types.GeomType) * (len(types.GeomType) + 1) // 2,
    )
  )

  m.nmaxpolygon = np.append(mjm.mesh_polyvertnum, 0).max()
  m.nmaxmeshdeg = np.append(mjm.mesh_polymapnum, 0).max()

  # filter plugins for only geom plugins, drop the rest
  m.plugin, m.plugin_attr = [], []
  m.geom_plugin_index = np.full_like(mjm.geom_type, -1)

  for i in range(len(mjm.geom_plugin)):
    if mjm.geom_plugin[i] == -1:
      continue
    p = mjm.geom_plugin[i]
    m.geom_plugin_index[i] = len(m.plugin)
    m.plugin.append(mjm.plugin[p])
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
    m.plugin_attr.append(attr_values[:3])

  # equality constraint addresses
  m.eq_connect_adr = np.nonzero(mjm.eq_type == types.EqType.CONNECT)[0]
  m.eq_wld_adr = np.nonzero(mjm.eq_type == types.EqType.WELD)[0]
  m.eq_jnt_adr = np.nonzero(mjm.eq_type == types.EqType.JOINT)[0]
  m.eq_ten_adr = np.nonzero(mjm.eq_type == types.EqType.TENDON)[0]
  m.eq_flex_adr = np.nonzero(mjm.eq_type == types.EqType.FLEX)[0]

  # fixed tendon
  m.tendon_jnt_adr, m.wrap_jnt_adr = [], []
  for i in range(mjm.ntendon):
    adr = mjm.tendon_adr[i]
    if mjm.wrap_type[adr] == mujoco.mjtWrap.mjWRAP_JOINT:
      tendon_num = mjm.tendon_num[i]
      for j in range(tendon_num):
        m.tendon_jnt_adr.append(i)
        m.wrap_jnt_adr.append(adr + j)

  # spatial tendon
  m.tendon_site_pair_adr, m.tendon_geom_adr = [], []
  m.ten_wrapadr_site, m.ten_wrapnum_site = [0], []
  for i, tendon_num in enumerate(mjm.tendon_num):
    adr = mjm.tendon_adr[i]
    # sites
    if (mjm.wrap_type[adr : adr + tendon_num] == mujoco.mjtWrap.mjWRAP_SITE).all():
      if i < mjm.ntendon:
        m.ten_wrapadr_site.append(m.ten_wrapadr_site[-1] + tendon_num)
      m.ten_wrapnum_site.append(tendon_num)
    else:
      if i < mjm.ntendon:
        m.ten_wrapadr_site.append(m.ten_wrapadr_site[-1])
      m.ten_wrapnum_site.append(0)

    # geoms
    for j in range(tendon_num):
      wrap_type = mjm.wrap_type[adr + j]
      if j < tendon_num - 1:
        next_wrap_type = mjm.wrap_type[adr + j + 1]
        if wrap_type == mujoco.mjtWrap.mjWRAP_SITE and next_wrap_type == mujoco.mjtWrap.mjWRAP_SITE:
          m.tendon_site_pair_adr.append(i)
      if wrap_type == mujoco.mjtWrap.mjWRAP_SPHERE or wrap_type == mujoco.mjtWrap.mjWRAP_CYLINDER:
        m.tendon_geom_adr.append(i)

  m.tendon_limited_adr = np.nonzero(mjm.tendon_limited)[0]
  m.wrap_site_adr = np.nonzero(mjm.wrap_type == mujoco.mjtWrap.mjWRAP_SITE)[0]
  m.wrap_site_pair_adr = np.setdiff1d(m.wrap_site_adr[np.nonzero(np.diff(m.wrap_site_adr) == 1)[0]], mjm.tendon_adr[1:] - 1)
  m.wrap_geom_adr = np.nonzero(np.isin(mjm.wrap_type, [mujoco.mjtWrap.mjWRAP_SPHERE, mujoco.mjtWrap.mjWRAP_CYLINDER]))[0]

  # pulley scaling
  m.wrap_pulley_scale = np.ones(mjm.nwrap, dtype=float)
  pulley_adr = np.nonzero(mjm.wrap_type == mujoco.mjtWrap.mjWRAP_PULLEY)[0]
  for tadr, tnum in zip(mjm.tendon_adr, mjm.tendon_num):
    for padr in pulley_adr:
      if tadr <= padr < tadr + tnum:
        m.wrap_pulley_scale[padr : tadr + tnum] = 1.0 / mjm.wrap_prm[padr]

  m.actuator_trntype_body_adr = np.nonzero(mjm.actuator_trntype == mujoco.mjtTrn.mjTRN_BODY)[0]

  # sensor addresses
  m.sensor_pos_adr = np.nonzero(
    (mjm.sensor_needstage == mujoco.mjtStage.mjSTAGE_POS)
    & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_JOINTLIMITPOS)
    & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONLIMITPOS)
  )[0]
  m.sensor_limitpos_adr = np.nonzero(
    (mjm.sensor_type == mujoco.mjtSensor.mjSENS_JOINTLIMITPOS) | (mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONLIMITPOS)
  )[0]
  m.sensor_vel_adr = np.nonzero(
    (mjm.sensor_needstage == mujoco.mjtStage.mjSTAGE_VEL)
    & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_JOINTLIMITVEL)
    & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONLIMITVEL)
  )[0]
  m.sensor_limitvel_adr = np.nonzero(
    (mjm.sensor_type == mujoco.mjtSensor.mjSENS_JOINTLIMITVEL) | (mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONLIMITVEL)
  )[0]
  m.sensor_acc_adr = np.nonzero(
    (mjm.sensor_needstage == mujoco.mjtStage.mjSTAGE_ACC)
    & (
      (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TOUCH)
      | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_JOINTLIMITFRC)
      | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONLIMITFRC)
      | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONACTFRC)
    )
  )[0]
  m.sensor_rangefinder_adr = np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER)[0]
  m.rangefinder_sensor_adr = np.full(mjm.nsensor, -1)
  m.rangefinder_sensor_adr[m.sensor_rangefinder_adr] = np.arange(len(m.sensor_rangefinder_adr))
  m.collision_sensor_adr = np.full(mjm.nsensor, -1)
  m.collision_sensor_adr[sensor_collision_adr] = np.arange(len(sensor_collision_adr))
  m.sensor_touch_adr = np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_TOUCH)[0]
  limitfrc_sensors = (mujoco.mjtSensor.mjSENS_JOINTLIMITFRC, mujoco.mjtSensor.mjSENS_TENDONLIMITFRC)
  m.sensor_limitfrc_adr = np.nonzero(np.isin(mjm.sensor_type, limitfrc_sensors))[0]
  m.sensor_e_potential = (mjm.sensor_type == mujoco.mjtSensor.mjSENS_E_POTENTIAL).any()
  m.sensor_e_kinetic = (mjm.sensor_type == mujoco.mjtSensor.mjSENS_E_KINETIC).any()
  m.sensor_tendonactfrc_adr = np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONACTFRC)[0]
  subtreevel_sensors = (mujoco.mjtSensor.mjSENS_SUBTREELINVEL, mujoco.mjtSensor.mjSENS_SUBTREEANGMOM)
  m.sensor_subtree_vel = np.isin(mjm.sensor_type, subtreevel_sensors).any()
  m.sensor_contact_adr = np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT)[0]
  m.sensor_adr_to_contact_adr = np.clip(np.cumsum(mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT) - 1, a_min=0, a_max=None)
  m.sensor_rne_postconstraint = np.isin(
    mjm.sensor_type,
    [
      mujoco.mjtSensor.mjSENS_ACCELEROMETER,
      mujoco.mjtSensor.mjSENS_FORCE,
      mujoco.mjtSensor.mjSENS_TORQUE,
      mujoco.mjtSensor.mjSENS_FRAMELINACC,
      mujoco.mjtSensor.mjSENS_FRAMEANGACC,
    ],
  ).any()
  m.sensor_rangefinder_bodyid = mjm.site_bodyid[mjm.sensor_objid[mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER]]
  m.taxel_vertadr = [
    j + mjm.mesh_vertadr[mjm.sensor_objid[i]]
    for i in range(mjm.nsensor)
    if mjm.sensor_type[i] == mujoco.mjtSensor.mjSENS_TACTILE
    for j in range(mjm.mesh_vertnum[mjm.sensor_objid[i]])
  ]
  m.taxel_sensorid = [
    i
    for i in range(mjm.nsensor)
    if mjm.sensor_type[i] == mujoco.mjtSensor.mjSENS_TACTILE
    for j in range(mjm.mesh_vertnum[mjm.sensor_objid[i]])
  ]

  # qM_tiles records the block diagonal structure of qM
  tile_corners = [i for i in range(mjm.nv) if mjm.dof_parentid[i] == -1]
  tiles = {}
  for i in range(len(tile_corners)):
    tile_beg = tile_corners[i]
    tile_end = mjm.nv if i == len(tile_corners) - 1 else tile_corners[i + 1]
    tiles.setdefault(tile_end - tile_beg, []).append(tile_beg)
  m.qM_tiles = tuple(types.TileSet(adr=wp.array(tiles[sz], dtype=int), size=sz) for sz in sorted(tiles.keys()))

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
  m.qLD_updates = tuple(wp.array(qLD_updates[i], dtype=wp.vec3i) for i in sorted(qLD_updates))

  # indices for sparse qM_fullm (used in solver)
  m.qM_fullm_i, m.qM_fullm_j = [], []
  for i in range(mjm.nv):
    j = i
    while j > -1:
      m.qM_fullm_i.append(i)
      m.qM_fullm_j.append(j)
      j = mjm.dof_parentid[j]

  # Gather-based sparse mul_m: for each row, all (col, madr) including diagonal
  row_elements = [[] for _ in range(mjm.nv)]

  # Add diagonal
  for i in range(mjm.nv):
    row_elements[i].append((i, mjm.dof_Madr[i]))

  # Add off-diagonals: ancestors (lower) and descendants (upper)
  for i in range(mjm.nv):
    madr_ij, j = mjm.dof_Madr[i], i
    while True:
      madr_ij, j = madr_ij + 1, mjm.dof_parentid[j]
      if j == -1:
        break
      row_elements[i].append((j, madr_ij))  # row i gathers M[i,j] * vec[j]
      row_elements[j].append((i, madr_ij))  # row j gathers M[j,i] * vec[i]

  # Flatten into CSR-like arrays
  m.qM_mulm_rowadr = [0]
  m.qM_mulm_col = []
  m.qM_mulm_madr = []
  for i in range(mjm.nv):
    for col, madr in row_elements[i]:
      m.qM_mulm_col.append(col)
      m.qM_mulm_madr.append(madr)
    m.qM_mulm_rowadr.append(len(m.qM_mulm_col))

  # TODO(team): remove after mjwarp depends on mujoco > 3.4.0 in pyproject.toml
  if BLEEDING_EDGE_MUJOCO:
    m.flexedge_J_rownnz = mjm.flexedge_J_rownnz
    m.flexedge_J_rowadr = mjm.flexedge_J_rowadr
    m.flexedge_J_colind = mjm.flexedge_J_colind.reshape(-1)
  else:
    mjd = mujoco.MjData(mjm)
    mujoco.mj_forward(mjm, mjd)
    m.flexedge_J_rownnz = mjd.flexedge_J_rownnz
    m.flexedge_J_rowadr = mjd.flexedge_J_rowadr
    m.flexedge_J_colind = mjd.flexedge_J_colind.reshape(-1)

  # place m on device
  sizes = dict({"*": 1}, **{f.name: getattr(m, f.name) for f in dataclasses.fields(types.Model) if f.type is int})
  for f in dataclasses.fields(types.Model):
    if isinstance(f.type, wp.array):
      setattr(m, f.name, _create_array(getattr(m, f.name), f.type, sizes))

  return m


def _get_padded_sizes(nv: int, njmax: int, is_sparse: bool, tile_size: int):
  # if dense - we just pad to the next multiple of 4 for nv, to get the fast load path.
  #            we pad to the next multiple of tile_size for njmax to avoid out of bounds accesses.
  # if sparse - we pad to the next multiple of tile_size for njmax, and nv.

  def round_up(x, multiple):
    return ((x + multiple - 1) // multiple) * multiple

  njmax_padded = round_up(njmax, tile_size)
  nv_padded = round_up(nv, tile_size) if (is_sparse or nv > 32) else round_up(nv, 4)

  return njmax_padded, nv_padded


def _default_nconmax(mjm: mujoco.MjModel, mjd: Optional[mujoco.MjData] = None) -> int:
  """Returns a default guess for an ideal nconmax given a Model and optional Data.

  This guess is based off a very simple heuristic, and may need to be manually raised if MJWarp
  reports ncon overflow, or lowered in order to get the very best performance.
  """
  valid_sizes = (2 + (np.arange(19) % 2)) * (2 ** (np.arange(19) // 2 + 3))  # 16, 24, 32, 48, ... 8192
  has_sdf = (mjm.geom_type == mujoco.mjtGeom.mjGEOM_SDF).any()
  has_flex = mjm.nflex > 0
  nconmax = max(mjm.nv * 0.35 * (mjm.nhfield > 0) * 10 + 45, 256 * has_flex, 64 * has_sdf, mjd.ncon if mjd else 0)
  return int(valid_sizes[np.searchsorted(valid_sizes, nconmax)])


def _default_njmax(mjm: mujoco.MjModel, mjd: Optional[mujoco.MjData] = None) -> int:
  """Returns a default guess for an ideal njmax given a Model and optional Data.

  This guess is based off a very simple heuristic, and may need to be manually raised if MJWarp
  reports ncon overflow, or lowered in order to get the very best performance.
  """
  valid_sizes = (2 + (np.arange(19) % 2)) * (2 ** (np.arange(19) // 2 + 3))  # 16, 24, 32, 48, ... 8192
  has_sdf = (mjm.geom_type == mujoco.mjtGeom.mjGEOM_SDF).any()
  has_flex = mjm.nflex > 0
  njmax = max(mjm.nv * 2.26 * (mjm.nhfield > 0) * 18 + 53, 512 * has_flex, 256 * has_sdf, mjd.nefc if mjd else 0)
  return int(valid_sizes[np.searchsorted(valid_sizes, njmax)])


def make_data(
  mjm: mujoco.MjModel,
  nworld: int = 1,
  nconmax: Optional[int] = None,
  nccdmax: Optional[int] = None,
  njmax: Optional[int] = None,
  naconmax: Optional[int] = None,
  naccdmax: Optional[int] = None,
) -> types.Data:
  """Creates a data object on device.

  Args:
    mjm: The model containing kinematic and dynamic information (host).
    nworld: Number of worlds.
    nconmax: Number of contacts to allocate per world. Contacts exist in large
             heterogeneous arrays: one world may have more than nconmax contacts.
    nccdmax: Number of CCD contacts to allocate per world. Same semantics as nconmax.
    njmax: Number of constraints to allocate per world. Constraint arrays are
           batched by world: no world may have more than njmax constraints.
    naconmax: Number of contacts to allocate for all worlds. Overrides nconmax.
    naccdmax: Maximum number of CCD contacts. Defaults to naconmax.

  Returns:
    The data object containing the current state and output arrays (device).
  """
  # TODO(team): move nconmax, njmax to Model?
  if nconmax is None:
    nconmax = _default_nconmax(mjm)

  if nconmax < 0:
    raise ValueError("nconmax must be >= 0")

  if nccdmax is None:
    nccdmax = nconmax
  elif nccdmax < 0:
    raise ValueError("nccdmax must be >= 0")
  elif nccdmax > nconmax:
    raise ValueError(f"nccdmax ({nccdmax}) must be <= nconmax ({nconmax})")

  if njmax is None:
    njmax = _default_njmax(mjm)

  if njmax < 0:
    raise ValueError("njmax must be >= 0")

  if nworld < 1:
    raise ValueError(f"nworld must be >= 1")

  if naconmax is None:
    naconmax = nworld * nconmax
  elif naconmax < 0:
    raise ValueError("naconmax must be >= 0")

  if naccdmax is None:
    naccdmax = nworld * nccdmax
  elif naccdmax < 0:
    raise ValueError("naccdmax must be >= 0")
  elif naccdmax > naconmax:
    raise ValueError(f"naccdmax ({naccdmax}) must be <= naconmax ({naconmax})")

  sizes = dict({"*": 1}, **{f.name: getattr(mjm, f.name, None) for f in dataclasses.fields(types.Model) if f.type is int})
  sizes["nmaxcondim"] = np.concatenate(([0], mjm.geom_condim, mjm.pair_dim)).max()
  sizes["nmaxpyramid"] = np.maximum(1, 2 * (sizes["nmaxcondim"] - 1))
  tile_size = types.TILE_SIZE_JTDAJ_SPARSE if is_sparse(mjm) else types.TILE_SIZE_JTDAJ_DENSE
  sizes["njmax_pad"], sizes["nv_pad"] = _get_padded_sizes(mjm.nv, njmax, is_sparse(mjm), tile_size)
  sizes["nworld"] = nworld
  sizes["naconmax"] = naconmax
  sizes["njmax"] = njmax

  contact = types.Contact(**{f.name: _create_array(None, f.type, sizes) for f in dataclasses.fields(types.Contact)})
  efc = types.Constraint(**{f.name: _create_array(None, f.type, sizes) for f in dataclasses.fields(types.Constraint)})

  # world body and static geom (attached to the world) poses are precomputed
  # this speeds up scenes with many static geoms (e.g. terrains)
  # TODO(team): remove this when we introduce dof islands + sleeping
  mjd = mujoco.MjData(mjm)
  mujoco.mj_kinematics(mjm, mjd)

  # mocap
  mocap_body = np.nonzero(mjm.body_mocapid >= 0)[0]
  mocap_id = mjm.body_mocapid[mocap_body]

  d_kwargs = {
    "qpos": wp.array(np.tile(mjm.qpos0, nworld), shape=(nworld, mjm.nq), dtype=float),
    "contact": contact,
    "efc": efc,
    "nworld": nworld,
    "naconmax": naconmax,
    "naccdmax": naccdmax,
    "njmax": njmax,
    "qM": None,
    "qLD": None,
    # world body
    "xquat": wp.array(np.tile(mjd.xquat, (nworld, 1)), shape=(nworld, mjm.nbody), dtype=wp.quat),
    "xmat": wp.array(np.tile(mjd.xmat, (nworld, 1)), shape=(nworld, mjm.nbody), dtype=wp.mat33),
    "ximat": wp.array(np.tile(mjd.ximat, (nworld, 1)), shape=(nworld, mjm.nbody), dtype=wp.mat33),
    # static geoms
    "geom_xpos": wp.array(np.tile(mjd.geom_xpos, (nworld, 1)), shape=(nworld, mjm.ngeom), dtype=wp.vec3),
    "geom_xmat": wp.array(np.tile(mjd.geom_xmat, (nworld, 1)), shape=(nworld, mjm.ngeom), dtype=wp.mat33),
    # mocap
    "mocap_pos": wp.array(np.tile(mjm.body_pos[mocap_body[mocap_id]], (nworld, 1)), shape=(nworld, mjm.nmocap), dtype=wp.vec3),
    "mocap_quat": wp.array(
      np.tile(mjm.body_quat[mocap_body[mocap_id]], (nworld, 1)), shape=(nworld, mjm.nmocap), dtype=wp.quat
    ),
    # equality constraints
    "eq_active": wp.array(np.tile(mjm.eq_active0.astype(bool), (nworld, 1)), shape=(nworld, mjm.neq), dtype=bool),
    # flexedge
    "flexedge_J": None,
  }
  for f in dataclasses.fields(types.Data):
    if f.name in d_kwargs:
      continue
    d_kwargs[f.name] = _create_array(None, f.type, sizes)

  d = types.Data(**d_kwargs)

  if is_sparse(mjm):
    d.qM = wp.zeros((nworld, 1, mjm.nM), dtype=float)
    d.qLD = wp.zeros((nworld, 1, mjm.nC), dtype=float)
  else:
    d.qM = wp.zeros((nworld, sizes["nv_pad"], sizes["nv_pad"]), dtype=float)
    d.qLD = wp.zeros((nworld, mjm.nv, mjm.nv), dtype=float)

  d.flexedge_J = wp.zeros((nworld, 1, mjd.flexedge_J.size), dtype=float)

  return d


def put_data(
  mjm: mujoco.MjModel,
  mjd: mujoco.MjData,
  nworld: int = 1,
  nconmax: Optional[int] = None,
  nccdmax: Optional[int] = None,
  njmax: Optional[int] = None,
  naconmax: Optional[int] = None,
  naccdmax: Optional[int] = None,
) -> types.Data:
  """Moves data from host to a device.

  Args:
    mjm: The model containing kinematic and dynamic information (host).
    mjd: The data object containing current state and output arrays (host).
    nworld: The number of worlds.
    nconmax: Number of contacts to allocate per world.  Contacts exist in large
             heterogenous arrays: one world may have more than nconmax contacts.
    nccdmax: Number of CCD contacts to allocate per world. Same semantics as nconmax.
    njmax: Number of constraints to allocate per world.  Constraint arrays are
           batched by world: no world may have more than njmax constraints.
    naconmax: Number of contacts to allocate for all worlds. Overrides nconmax.
    naccdmax: Maximum number of CCD contacts. Defaults to naconmax.

  Returns:
    The data object containing the current state and output arrays (device).
  """
  # TODO(team): move nconmax and njmax to Model?
  # TODO(team): decide what to do about uninitialized warp-only fields created by put_data
  #             we need to ensure these are only workspace fields and don't carry state

  if nconmax is None:
    nconmax = _default_nconmax(mjm, mjd)

  if nconmax < 0:
    raise ValueError("nconmax must be >= 0")

  if nccdmax is None:
    nccdmax = nconmax
  elif nccdmax < 0:
    raise ValueError("nccdmax must be >= 0")
  elif nccdmax > nconmax:
    raise ValueError(f"nccdmax ({nccdmax}) must be <= nconmax ({nconmax})")

  if njmax is None:
    njmax = _default_njmax(mjm, mjd)

  if njmax < 0:
    raise ValueError("njmax must be >= 0")

  if nworld < 1:
    raise ValueError(f"nworld must be >= 1")

  if naconmax is None:
    if mjd.ncon > nconmax:
      raise ValueError(f"nconmax overflow (nconmax must be >= {mjd.ncon})")
    naconmax = nworld * nconmax
  elif naconmax < mjd.ncon * nworld:
    raise ValueError(f"naconmax overflow (naconmax must be >= {mjd.ncon * nworld})")

  if naccdmax is None:
    naccdmax = nworld * nccdmax
  elif naccdmax < 0:
    raise ValueError("naccdmax must be >= 0")
  elif naccdmax > naconmax:
    raise ValueError(f"naccdmax ({naccdmax}) must be <= naconmax ({naconmax})")

  if mjd.nefc > njmax:
    raise ValueError(f"njmax overflow (njmax must be >= {mjd.nefc})")

  sizes = dict({"*": 1}, **{f.name: getattr(mjm, f.name, None) for f in dataclasses.fields(types.Model) if f.type is int})
  sizes["nmaxcondim"] = np.concatenate(([0], mjm.geom_condim, mjm.pair_dim)).max()
  sizes["nmaxpyramid"] = np.maximum(1, 2 * (sizes["nmaxcondim"] - 1))
  tile_size = types.TILE_SIZE_JTDAJ_SPARSE if is_sparse(mjm) else types.TILE_SIZE_JTDAJ_DENSE
  sizes["njmax_pad"], sizes["nv_pad"] = _get_padded_sizes(mjm.nv, njmax, is_sparse(mjm), tile_size)
  sizes["nworld"] = nworld
  sizes["naconmax"] = naconmax
  sizes["njmax"] = njmax

  # ensure static geom positions are computed
  # TODO: remove once MjData creation semantics are fixed
  mujoco.mj_kinematics(mjm, mjd)

  # create contact
  contact_kwargs = {"efc_address": None, "worldid": None, "type": None, "geomcollisionid": None}
  for f in dataclasses.fields(types.Contact):
    if f.name in contact_kwargs:
      continue
    val = getattr(mjd.contact, f.name)
    val = np.repeat(val, nworld, axis=0)
    width = ((0, naconmax - val.shape[0]),) + ((0, 0),) * (val.ndim - 1)
    val = np.pad(val, width)
    contact_kwargs[f.name] = _create_array(val, f.type, sizes)

  contact = types.Contact(**contact_kwargs)

  contact.efc_address = np.zeros((naconmax, sizes["nmaxpyramid"]), dtype=int)
  for i in range(mjd.ncon):
    efc_address = mjd.contact.efc_address[i]
    if efc_address == -1:
      continue
    condim = mjd.contact.dim[i]
    ndim = max(1, 2 * (condim - 1)) if mjm.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL else condim
    for j in range(nworld):
      contact.efc_address[j * mjd.ncon + i, :ndim] = efc_address + np.arange(ndim)

  contact.efc_address = wp.array(contact.efc_address, dtype=int)
  contact.worldid = np.pad(np.repeat(np.arange(nworld), mjd.ncon), (0, naconmax - nworld * mjd.ncon))
  contact.worldid = wp.array(contact.worldid, dtype=int)
  contact.type = wp.ones((naconmax,), dtype=int)  # TODO(team): set values
  contact.geomcollisionid = wp.empty((naconmax,), dtype=int)  # TODO(team): set values

  # create efc
  efc_kwargs = {"J": None}

  for f in dataclasses.fields(types.Constraint):
    if f.name in efc_kwargs:
      continue
    shape = tuple(sizes[dim] if isinstance(dim, str) else dim for dim in f.type.shape)
    val = np.zeros(shape, dtype=f.type.dtype)
    if f.name in ("type", "id", "pos", "margin", "D", "vel", "aref", "frictionloss", "force"):
      val[:, : mjd.nefc] = np.tile(getattr(mjd, "efc_" + f.name), (nworld, 1))
    efc_kwargs[f.name] = wp.array(val, dtype=f.type.dtype)

  efc = types.Constraint(**efc_kwargs)

  if mujoco.mj_isSparse(mjm):
    efc_j = np.zeros((mjd.nefc, mjm.nv))
    mujoco.mju_sparse2dense(efc_j, mjd.efc_J, mjd.efc_J_rownnz, mjd.efc_J_rowadr, mjd.efc_J_colind)
  else:
    efc_j = mjd.efc_J.reshape((mjd.nefc, mjm.nv))
  efc.J = np.zeros((nworld, sizes["njmax_pad"], sizes["nv_pad"]), dtype=f.type.dtype)
  efc.J[:, : mjd.nefc, : mjm.nv] = np.tile(efc_j, (nworld, 1, 1))
  efc.J = wp.array(efc.J, dtype=float)

  # create data
  d_kwargs = {
    "contact": contact,
    "efc": efc,
    "nworld": nworld,
    "naconmax": naconmax,
    "naccdmax": naccdmax,
    "njmax": njmax,
    # fields set after initialization:
    "solver_niter": None,
    "qM": None,
    "qLD": None,
    "ten_J": None,
    "actuator_moment": None,
    "flexedge_J": None,
    "nacon": None,
  }
  for f in dataclasses.fields(types.Data):
    if f.name in d_kwargs:
      continue
    val = getattr(mjd, f.name, None)
    if val is not None:
      shape = val.shape if hasattr(val, "shape") else ()
      val = np.full((nworld,) + shape, val)
    d_kwargs[f.name] = _create_array(val, f.type, sizes)

  d = types.Data(**d_kwargs)
  d.solver_niter = wp.full((nworld,), mjd.solver_niter[0], dtype=int)

  if is_sparse(mjm):
    d.qM = wp.array(np.full((nworld, 1, mjm.nM), mjd.qM), dtype=float)
    d.qLD = wp.array(np.full((nworld, 1, mjm.nC), mjd.qLD), dtype=float)
  else:
    qM = np.zeros((mjm.nv, mjm.nv))
    mujoco.mj_fullM(mjm, qM, mjd.qM)
    qLD = np.linalg.cholesky(qM) if (mjd.qM != 0.0).any() and (mjd.qLD != 0.0).any() else np.zeros((mjm.nv, mjm.nv))
    padding = sizes["nv_pad"] - mjm.nv
    qM_padded = np.pad(qM, ((0, padding), (0, padding)), mode="constant", constant_values=0.0)
    d.qM = wp.array(np.full((nworld, sizes["nv_pad"], sizes["nv_pad"]), qM_padded), dtype=float)
    d.qLD = wp.array(np.full((nworld, mjm.nv, mjm.nv), qLD), dtype=float)

  d.flexedge_J = wp.array(np.tile(mjd.flexedge_J.reshape(-1), (nworld, 1)).reshape((nworld, 1, -1)), dtype=float)

  if mujoco.mj_isSparse(mjm):
    ten_J = np.zeros((mjm.ntendon, mjm.nv))
    mujoco.mju_sparse2dense(ten_J, mjd.ten_J.reshape(-1), mjd.ten_J_rownnz, mjd.ten_J_rowadr, mjd.ten_J_colind.reshape(-1))
    d.ten_J = wp.array(np.full((nworld, mjm.ntendon, mjm.nv), ten_J), dtype=float)
  else:
    ten_J = mjd.ten_J.reshape((mjm.ntendon, mjm.nv))
    d.ten_J = wp.array(np.full((nworld, mjm.ntendon, mjm.nv), ten_J), dtype=float)

  # TODO(taylorhowell): sparse actuator_moment
  actuator_moment = np.zeros((mjm.nu, mjm.nv))
  mujoco.mju_sparse2dense(actuator_moment, mjd.actuator_moment, mjd.moment_rownnz, mjd.moment_rowadr, mjd.moment_colind)
  d.actuator_moment = wp.array(np.full((nworld, mjm.nu, mjm.nv), actuator_moment), dtype=float)

  d.nacon = wp.array([mjd.ncon * nworld], dtype=int)

  return d


def get_data_into(
  result: mujoco.MjData,
  mjm: mujoco.MjModel,
  d: types.Data,
  world_id: int = 0,
):
  """Gets data from a device into an existing mujoco.MjData.

  Args:
    result: The data object containing the current state and output arrays (host).
    mjm: The model containing kinematic and dynamic information (host).
    d: The data object containing the current state and output arrays (device).
    world_id: The id of the world to get the data from.
  """
  # nacon and nefc can overflow.  in that case, only pull up to the max contacts and constraints
  nacon = min(d.nacon.numpy()[0], d.naconmax)
  nefc = min(d.nefc.numpy()[world_id], d.njmax)

  ncon_filter = np.zeros_like(d.contact.worldid.numpy(), dtype=bool)
  ncon_filter[:nacon] = d.contact.worldid.numpy()[:nacon] == world_id
  ncon = ncon_filter.sum()

  if ncon != result.ncon or nefc != result.nefc:
    # TODO(team): if sparse, set nJ based on sparse efc_J
    mujoco._functions._realloc_con_efc(result, ncon=ncon, nefc=nefc, nJ=nefc * mjm.nv)

  ne = d.ne.numpy()[world_id]
  nf = d.nf.numpy()[world_id]
  nl = d.nl.numpy()[world_id]

  # efc indexing
  # mujoco expects contiguous efc ordering for contacts
  # this ordering is not guaranteed with mujoco warp, we enforce order here
  if ncon > 0:
    efc_idx_efl = np.arange(ne + nf + nl)

    contact_dim = d.contact.dim.numpy()[ncon_filter]
    contact_efc_address = d.contact.efc_address.numpy()[ncon_filter]

    efc_idx_c = []
    contact_efc_address_ordered = [ne + nf + nl]
    for i in range(ncon):
      dim = contact_dim[i]
      if mjm.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL:
        ndim = np.maximum(1, 2 * (dim - 1))
      else:
        ndim = dim
      efc_idx_c.append(contact_efc_address[i, :ndim])
      if i < ncon - 1:
        contact_efc_address_ordered.append(contact_efc_address_ordered[-1] + ndim)
    efc_idx = np.concatenate((efc_idx_efl, *efc_idx_c))
    contact_efc_address_ordered = np.array(contact_efc_address_ordered)
  else:
    efc_idx = np.array(np.arange(nefc))
    contact_efc_address_ordered = np.empty(0)

  efc_idx = efc_idx[:nefc]  # dont emit indices for overflow constraints

  result.solver_niter[0] = d.solver_niter.numpy()[world_id]
  result.ncon = ncon
  result.ne = ne
  result.nf = nf
  result.nl = nl
  result.time = d.time.numpy()[world_id]
  result.energy[:] = d.energy.numpy()[world_id]
  result.qpos[:] = d.qpos.numpy()[world_id]
  result.qvel[:] = d.qvel.numpy()[world_id]
  result.act[:] = d.act.numpy()[world_id]
  result.qacc_warmstart[:] = d.qacc_warmstart.numpy()[world_id]
  result.ctrl[:] = d.ctrl.numpy()[world_id]
  result.qfrc_applied[:] = d.qfrc_applied.numpy()[world_id]
  result.xfrc_applied[:] = d.xfrc_applied.numpy()[world_id]
  result.eq_active[:] = d.eq_active.numpy()[world_id]
  result.mocap_pos[:] = d.mocap_pos.numpy()[world_id]
  result.mocap_quat[:] = d.mocap_quat.numpy()[world_id]
  result.qacc[:] = d.qacc.numpy()[world_id]
  result.act_dot[:] = d.act_dot.numpy()[world_id]
  result.xpos[:] = d.xpos.numpy()[world_id]
  result.xquat[:] = d.xquat.numpy()[world_id]
  result.xmat[:] = d.xmat.numpy()[world_id].reshape((-1, 9))
  result.xipos[:] = d.xipos.numpy()[world_id]
  result.ximat[:] = d.ximat.numpy()[world_id].reshape((-1, 9))
  result.xanchor[:] = d.xanchor.numpy()[world_id]
  result.xaxis[:] = d.xaxis.numpy()[world_id]
  result.geom_xpos[:] = d.geom_xpos.numpy()[world_id]
  result.geom_xmat[:] = d.geom_xmat.numpy()[world_id].reshape((-1, 9))
  result.site_xpos[:] = d.site_xpos.numpy()[world_id]
  result.site_xmat[:] = d.site_xmat.numpy()[world_id].reshape((-1, 9))
  result.cam_xpos[:] = d.cam_xpos.numpy()[world_id]
  result.cam_xmat[:] = d.cam_xmat.numpy()[world_id].reshape((-1, 9))
  result.light_xpos[:] = d.light_xpos.numpy()[world_id]
  result.light_xdir[:] = d.light_xdir.numpy()[world_id]
  result.subtree_com[:] = d.subtree_com.numpy()[world_id]
  result.cdof[:] = d.cdof.numpy()[world_id]
  result.cinert[:] = d.cinert.numpy()[world_id]
  result.flexvert_xpos[:] = d.flexvert_xpos.numpy()[world_id]
  if mjm.nflexedge > 0:
    # TODO(team): remove after mjwarp depends on mujoco > 3.4.0 in pyproject.toml
    if not BLEEDING_EDGE_MUJOCO:
      m = put_model(mjm)
      result.flexedge_J_rownnz[:] = m.flexedge_J_rownnz.numpy()
      result.flexedge_J_rowadr[:] = m.flexedge_J_rowadr.numpy()
      result.flexedge_J_colind[:, :] = m.flexedge_J_colind.numpy().reshape((mjm.nflexedge, mjm.nv))
      mujoco.mju_sparse2dense(
        result.flexedge_J,
        d.flexedge_J.numpy()[world_id].reshape(-1),
        m.flexedge_J_rownnz.numpy(),
        m.flexedge_J_rowadr.numpy(),
        m.flexedge_J_colind.numpy(),
      )
    else:
      result.flexedge_J[:] = d.flexedge_J.numpy()[world_id].reshape(-1)
  result.flexedge_length[:] = d.flexedge_length.numpy()[world_id]
  result.flexedge_velocity[:] = d.flexedge_velocity.numpy()[world_id]
  result.actuator_length[:] = d.actuator_length.numpy()[world_id]
  actuator_moment = d.actuator_moment.numpy()[world_id]
  mujoco.mju_dense2sparse(
    result.actuator_moment, actuator_moment, result.moment_rownnz, result.moment_rowadr, result.moment_colind
  )
  result.crb[:] = d.crb.numpy()[world_id]
  result.qLDiagInv[:] = d.qLDiagInv.numpy()[world_id]
  result.ten_velocity[:] = d.ten_velocity.numpy()[world_id]
  result.actuator_velocity[:] = d.actuator_velocity.numpy()[world_id]
  result.cvel[:] = d.cvel.numpy()[world_id]
  result.cdof_dot[:] = d.cdof_dot.numpy()[world_id]
  result.qfrc_bias[:] = d.qfrc_bias.numpy()[world_id]
  result.qfrc_spring[:] = d.qfrc_spring.numpy()[world_id]
  result.qfrc_damper[:] = d.qfrc_damper.numpy()[world_id]
  result.qfrc_gravcomp[:] = d.qfrc_gravcomp.numpy()[world_id]
  result.qfrc_fluid[:] = d.qfrc_fluid.numpy()[world_id]
  result.qfrc_passive[:] = d.qfrc_passive.numpy()[world_id]
  result.subtree_linvel[:] = d.subtree_linvel.numpy()[world_id]
  result.subtree_angmom[:] = d.subtree_angmom.numpy()[world_id]
  result.actuator_force[:] = d.actuator_force.numpy()[world_id]
  result.qfrc_actuator[:] = d.qfrc_actuator.numpy()[world_id]
  result.qfrc_smooth[:] = d.qfrc_smooth.numpy()[world_id]
  result.qacc_smooth[:] = d.qacc_smooth.numpy()[world_id]
  result.qfrc_constraint[:] = d.qfrc_constraint.numpy()[world_id]
  result.qfrc_inverse[:] = d.qfrc_inverse.numpy()[world_id]

  # contact
  result.contact.dist[:ncon] = d.contact.dist.numpy()[ncon_filter]
  result.contact.pos[:ncon] = d.contact.pos.numpy()[ncon_filter]
  result.contact.frame[:ncon] = d.contact.frame.numpy()[ncon_filter].reshape((-1, 9))
  result.contact.includemargin[:ncon] = d.contact.includemargin.numpy()[ncon_filter]
  result.contact.friction[:ncon] = d.contact.friction.numpy()[ncon_filter]
  result.contact.solref[:ncon] = d.contact.solref.numpy()[ncon_filter]
  result.contact.solreffriction[:ncon] = d.contact.solreffriction.numpy()[ncon_filter]
  result.contact.solimp[:ncon] = d.contact.solimp.numpy()[ncon_filter]
  result.contact.dim[:ncon] = d.contact.dim.numpy()[ncon_filter]
  result.contact.geom[:ncon] = d.contact.geom.numpy()[ncon_filter]
  result.contact.efc_address[:ncon] = contact_efc_address_ordered[:ncon]

  if is_sparse(mjm):
    result.qM[:] = d.qM.numpy()[world_id, 0]
    result.qLD[:] = d.qLD.numpy()[world_id, 0]
  else:
    qM = d.qM.numpy()[world_id]
    adr = 0
    for i in range(mjm.nv):
      j = i
      while j >= 0:
        result.qM[adr] = qM[i, j]
        j = mjm.dof_parentid[j]
        adr += 1
    mujoco.mj_factorM(mjm, result)

  if nefc > 0:
    if mujoco.mj_isSparse(mjm):
      efc_J = d.efc.J.numpy()[world_id, efc_idx, : mjm.nv]
      mujoco.mju_dense2sparse(result.efc_J, efc_J, result.efc_J_rownnz, result.efc_J_rowadr, result.efc_J_colind)
    else:
      result.efc_J[: nefc * mjm.nv] = d.efc.J.numpy()[world_id, :nefc, : mjm.nv].flatten()

  # efc
  result.efc_type[:] = d.efc.type.numpy()[world_id, efc_idx]
  result.efc_id[:] = d.efc.id.numpy()[world_id, efc_idx]
  result.efc_pos[:] = d.efc.pos.numpy()[world_id, efc_idx]
  result.efc_margin[:] = d.efc.margin.numpy()[world_id, efc_idx]
  result.efc_D[:] = d.efc.D.numpy()[world_id, efc_idx]
  result.efc_vel[:] = d.efc.vel.numpy()[world_id, efc_idx]
  result.efc_aref[:] = d.efc.aref.numpy()[world_id, efc_idx]
  result.efc_frictionloss[:] = d.efc.frictionloss.numpy()[world_id, efc_idx]
  result.efc_state[:] = d.efc.state.numpy()[world_id, efc_idx]
  result.efc_force[:] = d.efc.force.numpy()[world_id, efc_idx]

  # rne_postconstraint
  result.cacc[:] = d.cacc.numpy()[world_id]
  result.cfrc_int[:] = d.cfrc_int.numpy()[world_id]
  result.cfrc_ext[:] = d.cfrc_ext.numpy()[world_id]

  # tendon
  result.ten_length[:] = d.ten_length.numpy()[world_id]
  result.ten_J[:] = d.ten_J.numpy()[world_id]
  result.ten_wrapadr[:] = d.ten_wrapadr.numpy()[world_id]
  result.ten_wrapnum[:] = d.ten_wrapnum.numpy()[world_id]
  result.wrap_obj[:] = d.wrap_obj.numpy()[world_id]
  result.wrap_xpos[:] = d.wrap_xpos.numpy()[world_id]

  # sensors
  result.sensordata[:] = d.sensordata.numpy()[world_id]


def reset_data(m: types.Model, d: types.Data, reset: Optional[wp.array] = None):
  """Clear data, set defaults; optionally by world.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    reset: Per-world bitmask. Reset if True.
  """

  @wp.kernel(module="unique", enable_backward=False)
  def reset_xfrc_applied(reset_in: wp.array(dtype=bool), xfrc_applied_out: wp.array2d(dtype=wp.spatial_vector)):
    worldid, bodyid, elemid = wp.tid()

    if wp.static(reset is not None):
      if not reset_in[worldid]:
        return

    xfrc_applied_out[worldid, bodyid][elemid] = 0.0

  @wp.kernel(module="unique", enable_backward=False)
  def reset_qM(reset_in: wp.array(dtype=bool), qM_out: wp.array3d(dtype=float)):
    worldid, elemid1, elemid2 = wp.tid()

    if wp.static(reset is not None):
      if not reset_in[worldid]:
        return

    qM_out[worldid, elemid1, elemid2] = 0.0

  @wp.kernel(module="unique", enable_backward=False)
  def reset_nworld(
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
    ne_out: wp.array(dtype=int),
    nf_out: wp.array(dtype=int),
    nl_out: wp.array(dtype=int),
    nefc_out: wp.array(dtype=int),
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
    nacon_out: wp.array(dtype=int),
  ):
    worldid = wp.tid()

    if wp.static(reset is not None):
      if not reset_in[worldid]:
        return

    solver_niter_out[worldid] = 0
    if worldid == 0:
      nacon_out[0] = 0
    ne_out[worldid] = 0
    nf_out[worldid] = 0
    nl_out[worldid] = 0
    nefc_out[worldid] = 0
    time_out[worldid] = 0.0
    energy_out[worldid] = wp.vec2(0.0, 0.0)
    qpos0_id = worldid % qpos0.shape[0]
    for i in range(nq):
      qpos_out[worldid, i] = qpos0[qpos0_id, i]
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

  @wp.kernel(module="unique", enable_backward=False)
  def reset_mocap(
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

    if wp.static(reset is not None):
      if not reset_in[worldid]:
        return

    mocapid = body_mocapid[bodyid]

    if mocapid >= 0:
      mocap_pos_out[worldid, mocapid] = body_pos[worldid, bodyid]
      mocap_quat_out[worldid, mocapid] = body_quat[worldid, bodyid]

  @wp.kernel(module="unique", enable_backward=False)
  def reset_contact(
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
    contact_type_out: wp.array(dtype=int),
    contact_geomcollisionid_out: wp.array(dtype=int),
  ):
    conid = wp.tid()

    if conid >= nacon_in[0]:
      return

    worldid = contact_worldid_out[conid]
    if wp.static(reset is not None):
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
    contact_type_out[conid] = 0
    contact_geomcollisionid_out[conid] = 0

  reset_input = reset or wp.ones(d.nworld, dtype=bool)

  wp.launch(reset_xfrc_applied, dim=(d.nworld, m.nbody, 6), inputs=[reset_input], outputs=[d.xfrc_applied])
  wp.launch(
    reset_qM,
    dim=(d.nworld, d.qM.shape[1], d.qM.shape[2]),
    inputs=[reset_input],
    outputs=[d.qM],
  )

  # set mocap_pos/quat = body_pos/quat for mocap bodies
  wp.launch(
    reset_mocap,
    dim=(d.nworld, m.nbody),
    inputs=[m.body_mocapid, m.body_pos, m.body_quat, reset_input],
    outputs=[d.mocap_pos, d.mocap_quat],
  )

  # clear contacts
  wp.launch(
    reset_contact,
    dim=d.naconmax,
    inputs=[d.nacon, reset_input, d.contact.efc_address.shape[1]],
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
      d.contact.type,
      d.contact.geomcollisionid,
    ],
  )

  wp.launch(
    reset_nworld,
    dim=d.nworld,
    inputs=[m.nq, m.nv, m.nu, m.na, m.neq, m.nsensordata, m.qpos0, m.eq_active0, d.nworld, reset_input],
    outputs=[
      d.solver_niter,
      d.ne,
      d.nf,
      d.nl,
      d.nefc,
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
      d.nacon,
    ],
  )


# kernel_analyzer: off
@wp.kernel
def _init_subtreemass(
  body_mass_in: wp.array2d(dtype=float),
  body_subtreemass_out: wp.array2d(dtype=float),
):
  worldid, bodyid = wp.tid()
  body_mass_id = worldid % body_mass_in.shape[0]
  body_subtreemass_id = worldid % body_subtreemass_out.shape[0]
  body_subtreemass_out[body_subtreemass_id, bodyid] = body_mass_in[body_mass_id, bodyid]


@wp.kernel
def _accumulate_subtreemass(
  body_parentid: wp.array(dtype=int),
  body_subtreemass_io: wp.array2d(dtype=float),
  body_tree_: wp.array(dtype=int),
):
  worldid, nodeid = wp.tid()
  body_subtreemass_id = worldid % body_subtreemass_io.shape[0]
  bodyid = body_tree_[nodeid]
  parentid = body_parentid[bodyid]
  if bodyid != 0:
    wp.atomic_add(body_subtreemass_io, body_subtreemass_id, parentid, body_subtreemass_io[body_subtreemass_id, bodyid])


@wp.kernel
def _copy_qpos0_to_qpos(
  qpos0: wp.array2d(dtype=float),
  qpos_out: wp.array2d(dtype=float),
):
  worldid, i = wp.tid()
  qpos0_id = worldid % qpos0.shape[0]
  qpos_out[worldid, i] = qpos0[qpos0_id, i]


@wp.kernel
def _copy_tendon_length0(
  ten_length_in: wp.array2d(dtype=float),
  tendon_length0_out: wp.array2d(dtype=float),
):
  worldid, tenid = wp.tid()
  tendon_length0_id = worldid % tendon_length0_out.shape[0]
  tendon_length0_out[tendon_length0_id, tenid] = ten_length_in[worldid, tenid]


@wp.kernel
def _compute_meaninertia(
  nv: int,
  is_sparse: bool,
  dof_Madr_in: wp.array(dtype=int),
  qM_in: wp.array3d(dtype=float),
  meaninertia_out: wp.array(dtype=float),
):
  """Compute mean diagonal inertia from qM at qpos0."""
  worldid = wp.tid()

  if nv == 0:
    meaninertia_out[worldid % meaninertia_out.shape[0]] = 1.0  # Default from MuJoCo
    return

  total = float(0.0)
  for i in range(nv):
    if is_sparse:
      # Sparse: qM is flattened lower triangular, diagonal at dof_Madr[i]
      madr = dof_Madr_in[i]
      total += qM_in[worldid, 0, madr]
    else:
      # Dense: qM is 2D matrix, diagonal at [i,i]
      total += qM_in[worldid, i, i]

  meaninertia_out[worldid % meaninertia_out.shape[0]] = total / float(nv)


@wp.kernel
def _set_unit_vector(
  dofid_target: int,
  unit_vec_out: wp.array2d(dtype=float),
):
  worldid = wp.tid()
  nv = unit_vec_out.shape[1]
  for i in range(nv):
    if i == dofid_target:
      unit_vec_out[worldid, i] = 1.0
    else:
      unit_vec_out[worldid, i] = 0.0


@wp.kernel
def _extract_dof_A_diag(
  dofid: int,
  result_vec_in: wp.array2d(dtype=float),
  dof_A_diag_out: wp.array2d(dtype=float),
):
  worldid = wp.tid()
  dof_A_diag_id = worldid % dof_A_diag_out.shape[0]
  dof_A_diag_out[dof_A_diag_id, dofid] = result_vec_in[worldid, dofid]


@wp.kernel
def _finalize_dof_invweight0(
  dof_jntid: wp.array(dtype=int),
  jnt_type: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  dof_A_diag_in: wp.array2d(dtype=float),
  dof_invweight0_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  dof_invweight0_id = worldid % dof_invweight0_out.shape[0]
  dof_A_diag_id = worldid % dof_A_diag_in.shape[0]

  jntid = dof_jntid[dofid]
  jtype = jnt_type[jntid]
  dofadr = jnt_dofadr[jntid]

  if jtype == int(types.JointType.FREE.value):
    # FREE joint: 6 DOFs, average first 3 (trans) and last 3 (rot) separately
    if dofid < dofadr + 3:
      avg = wp.static(1.0 / 3.0) * (
        dof_A_diag_in[dof_A_diag_id, dofadr + 0]
        + dof_A_diag_in[dof_A_diag_id, dofadr + 1]
        + dof_A_diag_in[dof_A_diag_id, dofadr + 2]
      )
    else:
      avg = wp.static(1.0 / 3.0) * (
        dof_A_diag_in[dof_A_diag_id, dofadr + 3]
        + dof_A_diag_in[dof_A_diag_id, dofadr + 4]
        + dof_A_diag_in[dof_A_diag_id, dofadr + 5]
      )
    dof_invweight0_out[dof_invweight0_id, dofid] = avg
  elif jtype == int(types.JointType.BALL.value):
    # BALL joint: 3 DOFs, average all
    avg = wp.static(1.0 / 3.0) * (
      dof_A_diag_in[dof_A_diag_id, dofadr + 0]
      + dof_A_diag_in[dof_A_diag_id, dofadr + 1]
      + dof_A_diag_in[dof_A_diag_id, dofadr + 2]
    )
    dof_invweight0_out[dof_invweight0_id, dofid] = avg
  else:
    # HINGE/SLIDE: 1 DOF, no averaging
    dof_invweight0_out[dof_invweight0_id, dofid] = dof_A_diag_in[dof_A_diag_id, dofid]


@wp.kernel
def _compute_body_jac_row(
  nv: int,
  bodyid_target: int,
  row_idx: int,
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  body_dofadr: wp.array(dtype=int),
  body_dofnum: wp.array(dtype=int),
  dof_parentid: wp.array(dtype=int),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  xipos_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  body_jac_row_out: wp.array2d(dtype=float),
):
  worldid = wp.tid()

  for i in range(nv):
    body_jac_row_out[worldid, i] = 0.0

  bodyid = bodyid_target
  while bodyid > 0 and body_dofnum[bodyid] == 0:
    bodyid = body_parentid[bodyid]

  if bodyid == 0:
    return

  # Compute offset from point (xipos) to subtree_com of root body
  point = xipos_in[worldid, bodyid_target]
  offset = point - subtree_com_in[worldid, body_rootid[bodyid_target]]

  # Get last dof that affects this body
  dofid = body_dofadr[bodyid] + body_dofnum[bodyid] - 1

  # Backward pass over dof ancestor chain
  while dofid >= 0:
    cdof = cdof_in[worldid, dofid]
    cdof_ang = wp.spatial_top(cdof)
    cdof_lin = wp.spatial_bottom(cdof)

    if row_idx < 3:
      tmp = wp.cross(cdof_ang, offset)
      if row_idx == 0:
        body_jac_row_out[worldid, dofid] = cdof_lin[0] + tmp[0]
      elif row_idx == 1:
        body_jac_row_out[worldid, dofid] = cdof_lin[1] + tmp[1]
      else:
        body_jac_row_out[worldid, dofid] = cdof_lin[2] + tmp[2]
    else:
      if row_idx == 3:
        body_jac_row_out[worldid, dofid] = cdof_ang[0]
      elif row_idx == 4:
        body_jac_row_out[worldid, dofid] = cdof_ang[1]
      else:
        body_jac_row_out[worldid, dofid] = cdof_ang[2]

    dofid = dof_parentid[dofid]


@wp.kernel
def _compute_body_A_diag_entry(
  nv: int,
  bodyid_target: int,
  row_idx: int,
  body_jac_row_in: wp.array2d(dtype=float),
  result_vec_in: wp.array2d(dtype=float),
  body_A_diag_out: wp.array3d(dtype=float),
):
  worldid = wp.tid()
  body_A_diag_id = worldid % body_A_diag_out.shape[0]
  # A[row,row] = J[row] · inv(M) · J[row]' = J[row] · result_vec
  dot_prod = float(0.0)
  for i in range(nv):
    dot_prod += body_jac_row_in[worldid, i] * result_vec_in[worldid, i]
  body_A_diag_out[body_A_diag_id, bodyid_target, row_idx] = dot_prod


@wp.kernel
def _finalize_body_invweight0(
  body_weldid: wp.array(dtype=int),
  body_A_diag_in: wp.array3d(dtype=float),
  body_invweight0_out: wp.array2d(dtype=wp.vec2),
):
  worldid, bodyid = wp.tid()
  body_invweight0_id = worldid % body_invweight0_out.shape[0]
  body_A_diag_id = worldid % body_A_diag_in.shape[0]

  # World body and static bodies have zero invweight
  if bodyid == 0 or body_weldid[bodyid] == 0:
    body_invweight0_out[body_invweight0_id, bodyid] = wp.vec2(0.0, 0.0)
    return

  # Average diagonal: trans = (A[0,0]+A[1,1]+A[2,2])/3, rot = (A[3,3]+A[4,4]+A[5,5])/3
  inv_trans = wp.static(1.0 / 3.0) * (
    body_A_diag_in[body_A_diag_id, bodyid, 0]
    + body_A_diag_in[body_A_diag_id, bodyid, 1]
    + body_A_diag_in[body_A_diag_id, bodyid, 2]
  )
  inv_rot = wp.static(1.0 / 3.0) * (
    body_A_diag_in[body_A_diag_id, bodyid, 3]
    + body_A_diag_in[body_A_diag_id, bodyid, 4]
    + body_A_diag_in[body_A_diag_id, bodyid, 5]
  )

  # Prevent degenerate constraints: if one component is near zero, use the other as fallback
  if inv_trans < mujoco.mjMINVAL and inv_rot > mujoco.mjMINVAL:
    inv_trans = inv_rot  # use rotation as fallback for translation
  elif inv_rot < mujoco.mjMINVAL and inv_trans > mujoco.mjMINVAL:
    inv_rot = inv_trans  # use translation as fallback for rotation

  body_invweight0_out[body_invweight0_id, bodyid] = wp.vec2(inv_trans, inv_rot)


@wp.kernel
def _copy_tendon_jacobian(
  tenid_target: int,
  ten_J_in: wp.array3d(dtype=float),
  ten_J_vec_out: wp.array2d(dtype=float),
):
  worldid = wp.tid()
  nv = ten_J_in.shape[2]
  for i in range(nv):
    ten_J_vec_out[worldid, i] = ten_J_in[worldid, tenid_target, i]


@wp.kernel
def _compute_tendon_dot_product(
  tenid_target: int,
  nv: int,
  ten_J_in: wp.array3d(dtype=float),
  result_vec_in: wp.array2d(dtype=float),
  tendon_invweight0_out: wp.array2d(dtype=float),
):
  worldid = wp.tid()
  tendon_invweight0_id = worldid % tendon_invweight0_out.shape[0]
  dot_prod = float(0.0)
  for i in range(nv):
    dot_prod += ten_J_in[worldid, tenid_target, i] * result_vec_in[worldid, i]
  tendon_invweight0_out[tendon_invweight0_id, tenid_target] = dot_prod


@wp.kernel
def _compute_cam_pos0(
  cam_bodyid: wp.array(dtype=int),
  cam_targetbodyid: wp.array(dtype=int),
  cam_xpos_in: wp.array2d(dtype=wp.vec3),
  cam_xmat_in: wp.array2d(dtype=wp.mat33),
  xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cam_pos0_out: wp.array2d(dtype=wp.vec3),
  cam_poscom0_out: wp.array2d(dtype=wp.vec3),
  cam_mat0_out: wp.array2d(dtype=wp.mat33),
):
  worldid, camid = wp.tid()
  cam_pos0_id = worldid % cam_pos0_out.shape[0]
  bodyid = cam_bodyid[camid]
  targetid = cam_targetbodyid[camid]
  cam_xpos = cam_xpos_in[worldid, camid]

  cam_pos0_out[cam_pos0_id, camid] = cam_xpos - xpos_in[worldid, bodyid]
  if targetid >= 0:
    cam_poscom0_out[cam_pos0_id, camid] = cam_xpos - subtree_com_in[worldid, targetid]
  else:
    cam_poscom0_out[cam_pos0_id, camid] = cam_xpos - subtree_com_in[worldid, bodyid]
  cam_mat0_out[cam_pos0_id, camid] = cam_xmat_in[worldid, camid]


@wp.kernel
def _compute_light_pos0(
  light_bodyid: wp.array(dtype=int),
  light_targetbodyid: wp.array(dtype=int),
  light_xpos_in: wp.array2d(dtype=wp.vec3),
  light_xdir_in: wp.array2d(dtype=wp.vec3),
  xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  light_pos0_out: wp.array2d(dtype=wp.vec3),
  light_poscom0_out: wp.array2d(dtype=wp.vec3),
  light_dir0_out: wp.array2d(dtype=wp.vec3),
):
  worldid, lightid = wp.tid()
  light_pos0_id = worldid % light_pos0_out.shape[0]
  bodyid = light_bodyid[lightid]
  targetid = light_targetbodyid[lightid]
  light_xpos = light_xpos_in[worldid, lightid]

  light_pos0_out[light_pos0_id, lightid] = light_xpos - xpos_in[worldid, bodyid]
  if targetid >= 0:
    light_poscom0_out[light_pos0_id, lightid] = light_xpos - subtree_com_in[worldid, targetid]
  else:
    light_poscom0_out[light_pos0_id, lightid] = light_xpos - subtree_com_in[worldid, bodyid]
  light_dir0_out[light_pos0_id, lightid] = light_xdir_in[worldid, lightid]


@wp.kernel
def _copy_actuator_moment(
  actid_target: int,
  actuator_moment_in: wp.array3d(dtype=float),
  act_moment_vec_out: wp.array2d(dtype=float),
):
  worldid = wp.tid()
  nv = actuator_moment_in.shape[2]
  for i in range(nv):
    act_moment_vec_out[worldid, i] = actuator_moment_in[worldid, actid_target, i]


@wp.kernel
def _compute_actuator_acc0(
  actid_target: int,
  nv: int,
  result_vec_in: wp.array2d(dtype=float),
  actuator_acc0_out: wp.array(dtype=float),
):
  worldid = wp.tid()
  norm_sq = float(0.0)
  for i in range(nv):
    norm_sq += result_vec_in[worldid, i] * result_vec_in[worldid, i]
  actuator_acc0_out[actid_target] = wp.sqrt(norm_sq)


# kernel_analyzer: on


def set_const_fixed(m: types.Model, d: types.Data):
  """Compute fixed quantities (independent of qpos0).

  Computes:
    - body_subtreemass: mass of body and all descendants (depends on body_mass)
    - ngravcomp: count of bodies with gravity compensation (depends on body_gravcomp)

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
  """
  wp.launch(_init_subtreemass, dim=(d.nworld, m.nbody), inputs=[m.body_mass], outputs=[m.body_subtreemass])
  for i in reversed(range(len(m.body_tree))):
    body_tree = m.body_tree[i]
    wp.launch(
      _accumulate_subtreemass,
      dim=(d.nworld, body_tree.size),
      inputs=[m.body_parentid, m.body_subtreemass, body_tree],
    )

  # TODO(team): refactor for graph capture compatibility
  body_gravcomp_np = m.body_gravcomp.numpy()
  m.ngravcomp = int((body_gravcomp_np > 0.0).any(axis=0).sum())


def set_const_0(m: types.Model, d: types.Data):
  """Compute quantities that depend on qpos0.

  Computes:
    - tendon_length0: tendon resting lengths
    - dof_invweight0: inverse inertia for DOFs
    - body_invweight0: inverse spatial inertia for bodies
    - tendon_invweight0: inverse weight for tendons
    - cam_pos0, cam_poscom0, cam_mat0: camera references
    - light_pos0, light_poscom0, light_dir0: light references
    - actuator_acc0: acceleration from unit actuator force

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
  """
  qpos_saved = wp.clone(d.qpos)

  wp.launch(_copy_qpos0_to_qpos, dim=(d.nworld, m.nq), inputs=[m.qpos0], outputs=[d.qpos])

  smooth.kinematics(m, d)
  smooth.com_pos(m, d)
  smooth.camlight(m, d)
  smooth.flex(m, d)
  smooth.tendon(m, d)
  smooth.crb(m, d)
  smooth.tendon_armature(m, d)
  smooth.factor_m(m, d)
  smooth.transmission(m, d)

  # Compute meaninertia from qM diagonal at qpos0
  wp.launch(
    _compute_meaninertia,
    dim=d.nworld,
    inputs=[m.nv, m.is_sparse, m.dof_Madr, d.qM],
    outputs=[m.stat.meaninertia],
  )

  wp.launch(_copy_tendon_length0, dim=(d.nworld, m.ntendon), inputs=[d.ten_length], outputs=[m.tendon_length0])

  # dof_invweight0: computed per joint with averaging for multi-DOF joints
  # FREE: 6 DOFs, trans gets mean(A[0:3]), rot gets mean(A[3:6])
  # BALL: 3 DOFs, all get mean(A[0:3])
  # HINGE/SLIDE: 1 DOF, gets A[0,0]
  if m.nv > 0:
    unit_vec = wp.zeros((d.nworld, m.nv), dtype=float)
    result_vec = wp.zeros((d.nworld, m.nv), dtype=float)
    dof_A_diag = wp.zeros((d.nworld, m.nv), dtype=float)

    # TODO(team): more efficient approach instead of looping over nv?
    for dofid in range(m.nv):
      wp.launch(_set_unit_vector, dim=d.nworld, inputs=[dofid], outputs=[unit_vec])
      smooth.solve_m(m, d, result_vec, unit_vec)
      wp.launch(_extract_dof_A_diag, dim=d.nworld, inputs=[dofid, result_vec], outputs=[dof_A_diag])

    wp.launch(
      _finalize_dof_invweight0,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_jntid, m.jnt_type, m.jnt_dofadr, dof_A_diag],
      outputs=[m.dof_invweight0],
    )

  # body_invweight0: computed as mean diagonal of J * inv(M) * J'
  # where J is the 6xnv body Jacobian (3 rows translation, 3 rows rotation)
  if m.nv > 0:
    body_jac_row = wp.zeros((d.nworld, m.nv), dtype=float)
    body_result_vec = wp.zeros((d.nworld, m.nv), dtype=float)
    body_A_diag = wp.zeros((d.nworld, m.nbody, 6), dtype=float)

    # TODO(team): more efficient approach instead of nested iterations?
    for bodyid in range(1, m.nbody):
      for row_idx in range(6):
        wp.launch(
          _compute_body_jac_row,
          dim=d.nworld,
          inputs=[
            m.nv,
            bodyid,
            row_idx,
            m.body_parentid,
            m.body_rootid,
            m.body_dofadr,
            m.body_dofnum,
            m.dof_parentid,
            d.subtree_com,
            d.xipos,
            d.cdof,
          ],
          outputs=[body_jac_row],
        )
        smooth.solve_m(m, d, body_result_vec, body_jac_row)
        wp.launch(
          _compute_body_A_diag_entry,
          dim=d.nworld,
          inputs=[m.nv, bodyid, row_idx, body_jac_row, body_result_vec],
          outputs=[body_A_diag],
        )

    wp.launch(
      _finalize_body_invweight0,
      dim=(d.nworld, m.nbody),
      inputs=[m.body_weldid, body_A_diag],
      outputs=[m.body_invweight0],
    )
  else:
    m.body_invweight0.zero_()

  # tendon_invweight0[t] = J_t * inv(M) * J_t'
  if m.ntendon > 0:
    ten_J_vec = wp.zeros((d.nworld, m.nv), dtype=float)
    ten_result_vec = wp.zeros((d.nworld, m.nv), dtype=float)

    for tenid in range(m.ntendon):
      wp.launch(_copy_tendon_jacobian, dim=d.nworld, inputs=[tenid, d.ten_J], outputs=[ten_J_vec])
      smooth.solve_m(m, d, ten_result_vec, ten_J_vec)
      wp.launch(
        _compute_tendon_dot_product,
        dim=d.nworld,
        inputs=[tenid, m.nv, d.ten_J, ten_result_vec],
        outputs=[m.tendon_invweight0],
      )

  wp.launch(
    _compute_cam_pos0,
    dim=(d.nworld, m.ncam),
    inputs=[m.cam_bodyid, m.cam_targetbodyid, d.cam_xpos, d.cam_xmat, d.xpos, d.subtree_com],
    outputs=[m.cam_pos0, m.cam_poscom0, m.cam_mat0],
  )

  wp.launch(
    _compute_light_pos0,
    dim=(d.nworld, m.nlight),
    inputs=[m.light_bodyid, m.light_targetbodyid, d.light_xpos, d.light_xdir, d.xpos, d.subtree_com],
    outputs=[m.light_pos0, m.light_poscom0, m.light_dir0],
  )

  # actuator_acc0[i] = ||inv(M) * actuator_moment[i]|| - acceleration from unit actuator force
  if m.nu > 0 and m.nv > 0:
    act_moment_vec = wp.zeros((d.nworld, m.nv), dtype=float)
    act_result_vec = wp.zeros((d.nworld, m.nv), dtype=float)

    for actid in range(m.nu):
      wp.launch(_copy_actuator_moment, dim=d.nworld, inputs=[actid, d.actuator_moment], outputs=[act_moment_vec])
      smooth.solve_m(m, d, act_result_vec, act_moment_vec)
      wp.launch(_compute_actuator_acc0, dim=d.nworld, inputs=[actid, m.nv, act_result_vec], outputs=[m.actuator_acc0])

  wp.copy(d.qpos, qpos_saved)


def set_const(m: types.Model, d: types.Data):
  """Recomputes qpos0-dependent constant model fields.

  This function propagates changes from some model fields to derived fields,
  allowing modifications that would otherwise be unsafe. It should be called
  after modifying model parameters at runtime.

  Model fields that can be modified safely with set_const:

    Field                            | Notes
    ---------------------------------|----------------------------------------------
    qpos0, qpos_spring               |
    body_mass, body_inertia,         | Mass and inertia are usually scaled together
    body_ipos, body_iquat            | since inertia is sum(m * r^2).
    body_pos, body_quat              | Unsafe for static bodies (invalidates BVH).
    body_gravcomp                    | If changing from 0 to >0 bodies, required.
    dof_armature                     |
    eq_data                          | For connect/weld, offsets computed if not set.
    hfield_size                      |
    tendon_stiffness, tendon_damping | Only if changing from/to zero.
    actuator_gainprm, actuator_biasprm | For position actuators with dampratio.

  For selective updates, use the sub-functions directly based on what changed:

    Modified Field  | Call
    ----------------|------------------
    body_mass       | set_const
    body_gravcomp   | set_const_fixed
    body_inertia    | set_const_0
    qpos0           | set_const_0

  Computes:
    - Fixed quantities (via set_const_fixed):
      - body_subtreemass: mass of body and all descendants
      - ngravcomp: count of bodies with gravity compensation
    - qpos0-dependent quantities (via set_const_0):
      - tendon_length0: tendon resting lengths
      - dof_invweight0: inverse inertia for DOFs
      - body_invweight0: inverse spatial inertia for bodies
      - tendon_invweight0: inverse weight for tendons
      - cam_pos0, cam_poscom0, cam_mat0: camera references
      - light_pos0, light_poscom0, light_dir0: light references
      - actuator_acc0: acceleration from unit actuator force

  Skips: dof_M0, actuator_length0 (not in mjwarp).

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
  """
  set_const_fixed(m, d)
  set_const_0(m, d)


def override_model(model: types.Model | mujoco.MjModel, overrides: dict[str, Any] | Sequence[str]):
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
  # MuJoCo pybind11 enums don't support iteration, so we provide explicit mappings
  mj_enum_fields = {
    "opt.jacobian": {
      "DENSE": mujoco.mjtJacobian.mjJAC_DENSE,
      "SPARSE": mujoco.mjtJacobian.mjJAC_SPARSE,
      "AUTO": mujoco.mjtJacobian.mjJAC_AUTO,
    },
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

      if key in mj_enum_fields and isinstance(val, str):
        enum_member = val.strip().upper()
        if enum_member not in mj_enum_fields[key]:
          raise ValueError(f"Unrecognized enum value for {key}: {enum_member}")
        val = mj_enum_fields[key][enum_member]
      elif key in enum_fields and isinstance(val, str):
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


@wp.kernel
def _build_rays(
  # In:
  offset: int,
  img_w: int,
  img_h: int,
  projection: int,
  fovy: float,
  sensorsize: wp.vec2,
  intrinsic: wp.vec4,
  znear: float,
  # Out:
  ray_out: wp.array(dtype=wp.vec3),
):
  xid, yid = wp.tid()
  ray_out[offset + xid + yid * img_w] = render_util.compute_ray(
    projection, fovy, sensorsize, intrinsic, img_w, img_h, xid, yid, znear
  )


def create_render_context(
  mjm: mujoco.MjModel,
  m: types.Model,
  d: types.Data,
  cam_res: list[tuple[int, int]] | tuple[int, int] | None = None,
  render_rgb: list[bool] | bool | None = None,
  render_depth: list[bool] | bool | None = None,
  use_textures: bool = True,
  use_shadows: bool = False,
  enabled_geom_groups: list[int] = [0, 1, 2],
  cam_active: list[bool] | None = None,
  flex_render_smooth: bool = True,
) -> types.RenderContext:
  """Creates a render context on device.

  Args:
    mjm: The model containing kinematic and dynamic information on host.
    m: The model on device.
    d: The data on device.
    cam_res: The width and height to render each camera image. If None, uses the
             MuJoCo model values.
    render_rgb: Whether to render RGB images. If None, uses the MuJoCo model values.
    render_depth: Whether to render depth images. If None, uses the MuJoCo model values.
    use_textures: Whether to use textures.
    use_shadows: Whether to use shadows.
    enabled_geom_groups: The geom groups to render.
    cam_active: List of booleans indicating which cameras to include in rendering.
                If None, all cameras are included.
    flex_render_smooth: Whether to render flex meshes smoothly.

  Returns:
    The render context containing rendering fields and output arrays on device.
  """
  # TODO(team): remove after mjwarp depends on warp-lang >= 1.12 in pyproject.toml
  if use_textures and not hasattr(wp, "Texture2D"):
    warnings.warn("Textures require warp >= 1.12. Disabling textures.")
    use_textures = False

  # Mesh BVHs
  nmesh = mjm.nmesh
  geom_enabled_mask = np.isin(mjm.geom_group, list(enabled_geom_groups))
  mesh_geom_mask = geom_enabled_mask & (mjm.geom_type == types.GeomType.MESH) & (mjm.geom_dataid >= 0)
  used_mesh_id = set(mjm.geom_dataid[mesh_geom_mask].astype(int))
  geom_enabled_idx = np.nonzero(geom_enabled_mask)[0]

  mesh_registry = {}
  mesh_bvh_id = [wp.uint64(0) for _ in range(nmesh)]
  mesh_bounds_size = [wp.vec3(0.0, 0.0, 0.0) for _ in range(nmesh)]

  for mid in used_mesh_id:
    mesh, half = bvh.build_mesh_bvh(mjm, mid)
    mesh_registry[mesh.id] = mesh
    mesh_bvh_id[mid] = mesh.id
    mesh_bounds_size[mid] = half

  mesh_bvh_id_arr = wp.array(mesh_bvh_id, dtype=wp.uint64)
  mesh_bounds_size_arr = wp.array(mesh_bounds_size, dtype=wp.vec3)

  # HField BVHs
  nhfield = mjm.nhfield
  hfield_geom_mask = geom_enabled_mask & (mjm.geom_type == types.GeomType.HFIELD) & (mjm.geom_dataid >= 0)
  used_hfield_id = set(mjm.geom_dataid[hfield_geom_mask].astype(int))
  hfield_registry = {}
  hfield_bvh_id = [wp.uint64(0) for _ in range(nhfield)]
  hfield_bounds_size = [wp.vec3(0.0, 0.0, 0.0) for _ in range(nhfield)]

  for hid in used_hfield_id:
    hmesh, hhalf = bvh.build_hfield_bvh(mjm, hid)
    hfield_registry[hmesh.id] = hmesh
    hfield_bvh_id[hid] = hmesh.id
    hfield_bounds_size[hid] = hhalf

  hfield_bvh_id_arr = wp.array(hfield_bvh_id, dtype=wp.uint64)
  hfield_bounds_size_arr = wp.array(hfield_bounds_size, dtype=wp.vec3)

  # Flex BVHs
  flex_bvh_id = wp.uint64(0)
  flex_group_root = wp.zeros(d.nworld, dtype=int)
  flex_mesh = None
  flex_face_point = None
  flex_elemdataadr = None
  flex_shell = None
  flex_shelldataadr = None
  flex_faceadr = None
  flex_nface = 0
  flex_radius = None
  flex_workadr = None
  flex_worknum = None
  flex_nwork = 0

  if mjm.nflex > 0:
    (
      fmesh,
      face_point,
      flex_group_roots,
      flex_shell_data,
      flex_faceadr_data,
      flex_nface,
    ) = bvh.build_flex_bvh(mjm, m, d)

    flex_mesh = fmesh
    flex_bvh_id = fmesh.id
    flex_face_point = face_point
    flex_group_root = flex_group_roots
    flex_elemdataadr = wp.array(mjm.flex_elemdataadr, dtype=int)
    flex_shell = flex_shell_data
    flex_shelldataadr = wp.array(mjm.flex_shelldataadr, dtype=int)
    flex_faceadr = wp.array(flex_faceadr_data, dtype=int)
    flex_radius = wp.array(mjm.flex_radius, dtype=float)

    # precompute work item layout for unified refit kernel
    nflex = mjm.nflex
    workadr = np.zeros(nflex, dtype=np.int32)
    worknum = np.zeros(nflex, dtype=np.int32)
    cumsum = 0
    for f in range(nflex):
      workadr[f] = cumsum
      if mjm.flex_dim[f] == 2:
        worknum[f] = mjm.flex_elemnum[f] + mjm.flex_shellnum[f]
      else:
        worknum[f] = mjm.flex_shellnum[f]
      cumsum += worknum[f]
    flex_workadr = wp.array(workadr, dtype=int)
    flex_worknum = wp.array(worknum, dtype=int)
    flex_nwork = int(cumsum)

  textures_registry = []
  # TODO: remove after mjwarp depends on warp-lang >= 1.12 in pyproject.toml
  if hasattr(wp, "Texture2D"):
    for i in range(mjm.ntex):
      textures_registry.append(render_util.create_warp_texture(mjm, i))
    textures = wp.array(textures_registry, dtype=wp.Texture2D)
  else:
    # Dummy array when texture support isn't available (warp < 1.12)
    textures = wp.zeros(1, dtype=int)

  # Filter active cameras
  if cam_active is not None:
    assert len(cam_active) == mjm.ncam, f"cam_active must have length {mjm.ncam} (got {len(cam_active)})"
    active_cam_indices = np.nonzero(cam_active)[0]
  else:
    active_cam_indices = list(range(mjm.ncam))

  ncam = len(active_cam_indices)

  if cam_res is not None:
    if isinstance(cam_res, tuple):
      cam_res = [cam_res] * ncam
    assert len(cam_res) == ncam, (
      f"Camera resolutions must be provided for all active cameras (got {len(cam_res)}, expected {ncam})"
    )
    active_cam_res = cam_res
  else:
    active_cam_res = mjm.cam_resolution[active_cam_indices]

  cam_res_arr = wp.array(active_cam_res, dtype=wp.vec2i)

  if render_rgb and isinstance(render_rgb, bool):
    render_rgb = [render_rgb] * ncam
  elif render_rgb is None:
    # TODO: remove after mjwarp depends on mujoco >= 3.4.1 in pyproject.toml
    if BLEEDING_EDGE_MUJOCO:
      render_rgb = [mjm.cam_output[i] & mujoco.mjtCamOutBit.mjCAMOUT_RGB for i in active_cam_indices]
    else:
      render_rgb = [True] * ncam

  if render_depth and isinstance(render_depth, bool):
    render_depth = [render_depth] * ncam
  elif render_depth is None:
    # TODO: remove after mjwarp depends on mujoco >= 3.4.1 in pyproject.toml
    if BLEEDING_EDGE_MUJOCO:
      render_depth = [mjm.cam_output[i] & mujoco.mjtCamOutBit.mjCAMOUT_DEPTH for i in active_cam_indices]
    else:
      render_depth = [True] * ncam

  assert len(render_rgb) == ncam and len(render_depth) == ncam, (
    f"Render RGB and depth must be provided for all active cameras (got {len(render_rgb)}, {len(render_depth)}, expected {ncam})"
  )

  rgb_adr = -1 * np.ones(ncam, dtype=int)
  depth_adr = -1 * np.ones(ncam, dtype=int)
  cam_res_np = cam_res_arr.numpy()
  ri = 0
  di = 0
  total = 0

  for idx in range(ncam):
    if render_rgb[idx]:
      rgb_adr[idx] = ri
      ri += cam_res_np[idx][0] * cam_res_np[idx][1]
    if render_depth[idx]:
      depth_adr[idx] = di
      di += cam_res_np[idx][0] * cam_res_np[idx][1]

    total += cam_res_np[idx][0] * cam_res_np[idx][1]

  znear = mjm.vis.map.znear * mjm.stat.extent

  if m.cam_fovy.shape[0] > 1 or m.cam_intrinsic.shape[0] > 1:
    ray = None
  else:
    ray = wp.zeros(int(total), dtype=wp.vec3)

    offset = 0
    for idx, cam_id in enumerate(active_cam_indices):
      img_w = cam_res_np[idx][0]
      img_h = cam_res_np[idx][1]
      wp.launch(
        kernel=_build_rays,
        dim=(img_w, img_h),
        inputs=[
          offset,
          img_w,
          img_h,
          m.cam_projection.numpy()[cam_id].item(),
          m.cam_fovy.numpy()[0, cam_id].item(),
          wp.vec2(m.cam_sensorsize.numpy()[cam_id]),
          wp.vec4(m.cam_intrinsic.numpy()[0, cam_id]),
          znear,
        ],
        outputs=[ray],
      )
      offset += img_w * img_h

  bvh_ngeom = len(geom_enabled_idx)

  rc = types.RenderContext(
    nrender=ncam,
    cam_res=cam_res_arr,
    cam_id_map=wp.array(active_cam_indices, dtype=int),
    use_textures=use_textures,
    use_shadows=use_shadows,
    background_color=render_util.pack_rgba_to_uint32(0.1 * 255.0, 0.1 * 255.0, 0.2 * 255.0, 1.0 * 255.0),
    bvh_ngeom=bvh_ngeom,
    enabled_geom_ids=wp.array(geom_enabled_idx, dtype=int),
    mesh_registry=mesh_registry,
    mesh_bvh_id=mesh_bvh_id_arr,
    mesh_bounds_size=mesh_bounds_size_arr,
    mesh_texcoord=wp.array(mjm.mesh_texcoord, dtype=wp.vec2),
    mesh_texcoord_offsets=wp.array(mjm.mesh_texcoordadr, dtype=int),
    mesh_facetexcoord=wp.array(mjm.mesh_facetexcoord, dtype=wp.vec3i),
    textures=textures,
    textures_registry=textures_registry,
    hfield_registry=hfield_registry,
    hfield_bvh_id=hfield_bvh_id_arr,
    hfield_bounds_size=hfield_bounds_size_arr,
    flex_mesh=flex_mesh,
    flex_rgba=wp.array(mjm.flex_rgba, dtype=wp.vec4),
    flex_bvh_id=flex_bvh_id,
    flex_face_point=flex_face_point,
    flex_faceadr=flex_faceadr,
    flex_nface=flex_nface,
    flex_nwork=flex_nwork,
    flex_group_root=flex_group_root,
    flex_elemdataadr=flex_elemdataadr,
    flex_shell=flex_shell,
    flex_shelldataadr=flex_shelldataadr,
    flex_radius=flex_radius,
    flex_workadr=flex_workadr,
    flex_worknum=flex_worknum,
    flex_render_smooth=flex_render_smooth,
    bvh=None,
    bvh_id=None,
    lower=wp.zeros(d.nworld * bvh_ngeom, dtype=wp.vec3),
    upper=wp.zeros(d.nworld * bvh_ngeom, dtype=wp.vec3),
    group=wp.zeros(d.nworld * bvh_ngeom, dtype=int),
    group_root=wp.zeros(d.nworld, dtype=int),
    ray=ray,
    rgb_data=wp.zeros((d.nworld, ri), dtype=wp.uint32),
    rgb_adr=wp.array(rgb_adr, dtype=int),
    depth_data=wp.zeros((d.nworld, di), dtype=wp.float32),
    depth_adr=wp.array(depth_adr, dtype=int),
    render_rgb=wp.array(render_rgb, dtype=bool),
    render_depth=wp.array(render_depth, dtype=bool),
    znear=znear,
    total_rays=int(total),
  )

  bvh.build_scene_bvh(m, d, rc)

  return rc
