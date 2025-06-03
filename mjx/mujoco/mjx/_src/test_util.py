# Copyright 2023 DeepMind Technologies Limited
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
"""Utilities for testing."""

import os
import sys
import time
from typing import Dict, Optional, Tuple
from xml.etree import ElementTree as ET

from etils import epath
import jax
import mujoco
# pylint: disable=g-importing-member
from mujoco.mjx._src import forward
from mujoco.mjx._src import io
from mujoco.mjx._src.types import Data
# pylint: enable=g-importing-member
import numpy as np


def _measure(fn, *args) -> Tuple[float, float]:
  """Reports jit time and op time for a function."""

  beg = time.perf_counter()
  compiled_fn = fn.lower(*args).compile()
  end = time.perf_counter()
  jit_time = end - beg

  beg = time.perf_counter()
  result = compiled_fn(*args)
  jax.block_until_ready(result)
  end = time.perf_counter()
  run_time = end - beg

  return jit_time, run_time


def benchmark(
    m: mujoco.MjModel,
    nstep: int = 1000,
    batch_size: int = 1024,
    unroll_steps: int = 1,
    solver: str = 'newton',
    iterations: int = 1,
    ls_iterations: int = 4,
) -> Tuple[float, float, int]:
  """Benchmark a model."""

  xla_flags = os.environ.get('XLA_FLAGS', '')
  xla_flags += ' --xla_gpu_triton_gemm_any=True'
  os.environ['XLA_FLAGS'] = xla_flags

  m.opt.solver = {
      'cg': mujoco.mjtSolver.mjSOL_CG,
      'newton': mujoco.mjtSolver.mjSOL_NEWTON,
  }[solver.lower()]
  m.opt.iterations = iterations
  m.opt.ls_iterations = ls_iterations
  m = io.put_model(m)

  @jax.pmap
  def init(key):
    key = jax.random.split(key, batch_size // jax.device_count())

    @jax.vmap
    def random_init(key):
      d = io.make_data(m)
      qvel = 0.01 * jax.random.normal(key, shape=(m.nv,))
      d = d.replace(qvel=qvel)
      return d

    return random_init(key)

  key = jax.random.split(jax.random.key(0), jax.device_count())
  d = init(key)
  jax.block_until_ready(d)

  @jax.pmap
  def unroll(d):
    @jax.vmap
    def step(d, _):
      d = forward.step(m, d)
      return d, None

    d, _ = jax.lax.scan(step, d, None, length=nstep, unroll=unroll_steps)

    return d

  jit_time, run_time = _measure(unroll, d)
  steps = nstep * batch_size

  return jit_time, run_time, steps


def efc_order(m: mujoco.MjModel, d: mujoco.MjData, dx: Data) -> np.ndarray:
  """Returns a sort order such that dx.efc_*[order][:d._impl.nefc] == d.efc_*."""  # pytype: disable=attribute-error
  # reorder efc rows to skip inactive constraints and match contact order
  efl = dx._impl.ne + dx._impl.nf + dx._impl.nl  # pytype: disable=attribute-error
  order = np.arange(efl)
  order[(dx._impl.efc_J[:efl] == 0).all(axis=1)] = 2**16  # move empty rows to end  # pytype: disable=attribute-error
  for i in range(dx._impl.ncon):  # pytype: disable=attribute-error
    num_rows = dx._impl.contact.dim[i]  # pytype: disable=attribute-error
    if dx._impl.contact.dim[i] > 1 and m.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL:  # pytype: disable=attribute-error
      num_rows = (dx._impl.contact.dim[i] - 1) * 2  # pytype: disable=attribute-error
    if dx._impl.contact.dist[i] > 0:  # move empty contacts to end  # pytype: disable=attribute-error
      order = np.append(order, np.repeat(2**16, num_rows))
      continue
    contact_match = (d.contact.geom == dx._impl.contact.geom[i]).all(axis=-1)  # pytype: disable=attribute-error
    contact_match &= (d.contact.pos == dx._impl.contact.pos[i]).all(axis=-1)  # pytype: disable=attribute-error
    assert contact_match.any(), f'contact {i} not found'
    contact_id = np.nonzero(contact_match)[0][0]
    order = np.append(order, np.repeat(efl + contact_id, num_rows))

  return np.argsort(order, kind='stable')


_ACTUATOR_TYPES = ['motor', 'velocity', 'position', 'general', 'intvelocity']
_DYN_TYPES = ['none', 'integrator', 'filter', 'filterexact']
_DYN_PRMS = ['0.189', '2.1']
_JOINT_TYPES = ['free', 'hinge', 'slide', 'ball']
_JOINT_AXES = ['1 0 0', '0 1 0', '0 0 1']
_FRICTIONS = ['1.2 0.003 0.0002', '0.2 0.0001 0.0005']
_KP_POS = ['1', '2']
_KP_INTVEL = ['10000', '2000']
_KV_VEL = ['12', '1', '0', '0.1']
_PAIR_FRICTIONS = ['1.2 0.9 0.003 0.0002 0.0001']
_SOLREFS = ['0.04 1.01', '0.05 1.02', '0.03 1.1', '0.015 1.0']
_SOLIMPS = [
    '0.75 0.94 0.002 0.2 2',
    '0.8 0.99 0.001 0.3 6',
    '0.6 0.9 0.003 0.1 1',
]
_DIMS = ['3']
_MARGINS = ['0.0', '0.01', '0.02']
_GAPS = ['0.0', '0.005']
_GEARS = ['2.1 0.0 3.3 0 2.3 0', '5.0 3.1 0 2.3 0.0 1.1']


def p(pct: int) -> bool:
  assert 0 <= pct <= 100
  return np.random.uniform(low=0, high=100) < pct


def _make_joint(joint_type: str, name: str) -> Dict[str, str]:
  """Returns attributes for a joint."""
  joint_attr = {'type': joint_type, 'name': name}

  if joint_type not in ('free', 'ball'):
    joint_attr['axis'] = np.random.choice(_JOINT_AXES)
    lb, ub = -np.random.uniform() * 90, np.random.uniform() * 90
    joint_attr['range'] = f'{lb:.2f} {ub:.2f}'
  elif joint_type == 'ball':
    joint_attr['axis'] = '1 0 0'
    ub = np.random.uniform() * 90
    joint_attr['range'] = f'0.0 {ub:.2f}'

  if p(50) and joint_type != 'free':
    lb, ub = -np.random.uniform(), np.random.uniform()
    joint_attr['actuatorfrcrange'] = f'{lb:.2f} {ub:.2f}'

  if joint_type not in ('free',):
    joint_attr['damping'] = '{:.2f}'.format(np.random.uniform() * 20)
    joint_attr['stiffness'] = '{:.2f}'.format(np.random.uniform() * 20)

  joint_attr['actuatorgravcomp'] = np.random.choice(['true', 'false'])
  return joint_attr


def _geom_solparams(
    pair: bool = False, enable_contact: bool = True
) -> Dict[str, str]:
  """Returns geom solver parameters."""
  params = {
      'contype': np.random.choice(['0', '1']) if enable_contact else '0',
      'conaffinity': np.random.choice(['0', '1']) if enable_contact else '0',
      'priority': np.random.choice(['-1', '2']),
      'solmix': np.random.choice(['0.0', '1.6']),
      'friction': np.random.choice(_FRICTIONS),
      'condim': np.random.choice(_DIMS),
  }
  pair_params = {
      'solreffriction': np.random.choice(_SOLREFS),
      'friction': np.random.choice(_PAIR_FRICTIONS),
      'condim': np.random.choice(_DIMS),
  }
  params = pair_params if pair else params
  params.update({
      'solimp': np.random.choice(_SOLIMPS),
      'solref': np.random.choice(_SOLREFS),
      'margin': np.random.choice(_MARGINS),
      'gap': np.random.choice(_GAPS),
  })

  return params


def _make_geom(
    pos: str, size: float, name: str, enable_contact: bool = True
) -> Dict[str, str]:
  """Returns attributes for a sphere geom."""
  attr = {
      'pos': pos,
      'type': 'sphere',
      'name': name,
      'size': f'{size:.2f}',
      'mass': '1',
  }
  attr.update(_geom_solparams(pair=False, enable_contact=enable_contact))

  return attr


def _make_actuator(
    actuator_type: str,
    joint: Optional[str] = None,
    site: Optional[str] = None,
    refsite: Optional[str] = None,
) -> Dict[str, str]:
  """Returns attributes for an actuator."""
  if joint:
    attr = {'joint': joint}
  elif site:
    attr = {'site': site}
  else:
    raise ValueError('must provide a joint or site name')

  if refsite:
    attr['refsite'] = refsite

  attr['gear'] = np.random.choice(_GEARS)

  # set actuator type
  if actuator_type == 'position':
    attr['kp'] = np.random.choice(_KP_POS)
    attr['kv'] = np.random.choice(_KV_VEL)
  elif actuator_type == 'general':
    attr['biastype'] = 'affine'
    attr['gainprm'] = '35 0 0'
    attr['biasprm'] = '0 -35 -0.65'
  elif actuator_type == 'intvelocity':
    attr['kp'] = np.random.choice(_KP_INTVEL)
    lb, ub = -np.random.uniform(), np.random.uniform()
    attr['actrange'] = f'{lb:.2f} {ub:.2f}'
  elif actuator_type == 'velocity':
    attr['kv'] = np.random.choice(_KV_VEL)

  # set dyntype
  if actuator_type == 'general':
    attr['dyntype'] = np.random.choice(_DYN_TYPES)
    if attr['dyntype'] != 'none':
      attr['dynprm'] = np.random.choice(_DYN_PRMS)

  # ctrlrange
  if p(50) and actuator_type != 'intvelocity':
    lb, ub = -np.random.uniform(), np.random.uniform()
    attr['ctrlrange'] = f'{lb:.2f} {ub:.2f}'

  # forcerange
  if p(50):
    lb, ub = -np.random.uniform(), np.random.uniform()
    attr['forcerange'] = f'{lb*10:.2f} {ub*10:.2f}'

  return attr


def create_mjcf(
    seed: int,
    min_trees: int = 1,
    max_trees: int = 1,
    max_tree_depth: int = 5,
    body_pos: Tuple[float, float, float] = (0.0, 0.0, -0.5),
    geom_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    max_stacked_joints=4,
    max_geoms_per_body=2,
    max_contact_excludes=1,
    max_contact_pairs=4,
    disable_actuation_pct: int = 0,
    add_actuators: bool = False,
    root_always_free: bool = False,
    enable_contact: bool = True,
) -> str:
  """Creates a random MJCF for testing.

  Args:
    seed: seed for rng
    min_trees: minimum number of kinematic trees to generate
    max_trees: maximum number of kinematic trees to generate
    max_tree_depth: the maximum tree depth
    body_pos: the default body position relative to the parent
    geom_pos: the default geom position in the body frame
    max_stacked_joints: maximum number of joints to stack for each body
    max_geoms_per_body: maximum number of geoms per body
    max_contact_excludes: maximum number of bodies to exlude from contact
    max_contact_pairs: maximum number of explicit geom contact pairs in the xml
    disable_actuation_pct: the percentage of time to disable actuation via the
      disable flag
    add_actuators: whether to add actuators
    root_always_free: if True, the root body of each kinematic tree has a free
      joint with the world
    enable_contact: if False, disables all contacts via contype/conaffinity

  Returns:
    an XML string for the MuJoCo config
  Raises:
    AssertionError when args are not in the correct ranges
  """
  np.random.seed(seed)

  assert min_trees <= max_trees
  assert max_tree_depth >= 1
  assert 0 <= disable_actuation_pct <= 100
  assert max_stacked_joints >= 1
  assert max_geoms_per_body >= 1
  assert max_contact_excludes >= 1
  assert max_contact_pairs >= 1

  mjcf = ET.Element('mujoco')
  opt = ET.SubElement(mjcf, 'option', {'timestep': '0.005', 'solver': 'CG'})
  world = ET.SubElement(mjcf, 'worldbody')
  ET.SubElement(mjcf, 'compiler', {'autolimits': 'true'})

  # disable flags
  if p(disable_actuation_pct):
    ET.SubElement(opt, 'flag', {'actuation': 'disable'})

  ET.SubElement(
      world,
      'geom',
      {
          'name': 'plane',
          'type': 'plane',
          'contype': '1' if enable_contact else '0',
          'conaffinity': '1' if enable_contact else '0',
          'size': '40 40 40',
      },
  )

  # kinematic trees
  tree_depth = np.random.randint(1, max_tree_depth + 1)

  def make_tree(body: ET.Element, depth: int) -> None:
    if depth >= tree_depth:
      return

    z_pos = np.random.uniform(low=-1, high=1) * 0.01  # small jitter
    pos = f'{body_pos[0]:.3f} {body_pos[1]:.3f} {body_pos[2] + z_pos:.3f}'
    n_bodies = len(list(mjcf.iter('body')))
    gravcomp = np.random.uniform() * p(50)
    child = ET.SubElement(
        body,
        'body',
        {
            'pos': pos,
            'name': f'body{n_bodies}',
            'gravcomp': f'{gravcomp:.3f}',
        },
    )
    ET.SubElement(child, 'site', {'name': f'site{n_bodies}'})

    n_joints = len(list(mjcf.iter('joint')))
    for nj in range(np.random.randint(1, max_stacked_joints + 1)):
      joint_type = np.random.choice(_JOINT_TYPES)
      if nj == 0 and depth == 0 and root_always_free:
        joint_type = 'free'

      # free joint only allowed at top level
      while joint_type == 'free' and (depth > 0 or nj > 0):
        joint_type = np.random.choice(_JOINT_TYPES)

      joint_attr = _make_joint(joint_type, name=f'joint{n_joints + nj}')
      ET.SubElement(child, 'joint', joint_attr)

      prev_joints = child.findall('joint')
      had_ball_or_free = any(
          [j.get('type') in ('ball', 'free') for j in prev_joints]
      )
      if had_ball_or_free:
        break  # do not stack more joints

    n_geoms = len(list(mjcf.iter('geom')))
    for _ in range(np.random.randint(1, max_geoms_per_body + 1)):
      pos = ('{:.2f} ' * 3).format(*geom_pos).strip()
      size = 0.2 + np.random.uniform(low=-1, high=1) * 0.02
      geom_attr = _make_geom(
          pos, size, name=f'geom{n_geoms}', enable_contact=enable_contact
      )
      ET.SubElement(child, 'geom', geom_attr)
      n_geoms += 1

    make_tree(child, depth + 1)

  num_trees = np.random.randint(min_trees, max_trees + 1)
  for _ in range(num_trees):
    make_tree(world, 0)

  bodies = list(mjcf.iter('body'))
  n_bodies = len(bodies)

  # actuators
  if add_actuators:
    actuator = ET.SubElement(mjcf, 'actuator')
    n_joints = len(list(mjcf.iter('joint')))
    nu = np.random.randint(1, n_joints + 1)
    actuators = []

    # joint transmission
    for i in range(nu):
      actuator_type = np.random.choice(_ACTUATOR_TYPES)
      attr = _make_actuator(actuator_type, joint=f'joint{i}')
      actuators.append((actuator_type, attr))

    # site transmission
    for i in range(np.random.randint(0, n_bodies)):
      actuator_type = np.random.choice(_ACTUATOR_TYPES)
      attr = _make_actuator(actuator_type, site=f'site{i}')
      actuators.append((actuator_type, attr))

    # site transmission with refsite
    for i in range(np.random.randint(0, n_bodies)):
      j = np.random.randint(0, n_bodies)
      actuator_type = np.random.choice(_ACTUATOR_TYPES)
      attr = _make_actuator(actuator_type, site=f'site{i}', refsite=f'site{j}')
      actuators.append((actuator_type, attr))

    np.random.shuffle(actuators)
    for typ, attr in actuators:
      ET.SubElement(actuator, typ, attr)

  # contact pairs
  contact = ET.SubElement(mjcf, 'contact')
  geoms = list(mjcf.iter('geom'))
  geom_names = [geom.get('name') for geom in geoms]
  n_geoms = len(geoms)
  pairs = set()
  for _ in range(min(max_contact_pairs, n_geoms * (n_geoms - 1) // 2)):
    if p(80):
      continue

    geom1, geom2 = np.random.choice(geom_names, replace=False, size=2)
    if geom1 > geom2:
      geom1, geom2 = geom2, geom1

    if (geom1, geom2) in pairs:
      continue

    pairs.add((geom1, geom2))
    attr = {'geom1': geom1, 'geom2': geom2}
    attr.update(_geom_solparams(pair=True))
    ET.SubElement(contact, 'pair', attr)

  # exclude contacts
  body_names = [b.get('name') for b in bodies]
  for _ in range(min(max_contact_excludes, (n_bodies * (n_bodies - 1) // 2))):
    if p(50):
      continue

    body1, body2 = np.random.choice(body_names, replace=False, size=2)
    ET.SubElement(contact, 'exclude', {'body1': body1, 'body2': body2})

  # ElementTree.indent is not available before Python 3.9
  if sys.version_info.minor >= 9:
    ET.indent(mjcf)

  return ET.tostring(mjcf).decode('utf-8')


def load_test_file(name: str) -> mujoco.MjModel:
  """Loads a mujoco.MjModel based on the file name."""
  path = epath.resource_path('mujoco.mjx') / 'test_data' / name
  m = mujoco.MjModel.from_xml_path(path.as_posix())
  return m
