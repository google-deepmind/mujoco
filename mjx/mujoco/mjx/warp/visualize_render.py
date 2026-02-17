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
"""Visualize batch rendering output from MuJoCo Warp for debugging."""

import functools
import os
from typing import Sequence

from absl import app
from absl import flags
import jax
import jax.numpy as jp
import mediapy as media
import mujoco
from mujoco import mjx
from mujoco.mjx._src import bvh
from mujoco.mjx._src import forward
from mujoco.mjx._src import render
from mujoco.mjx._src import render_util
from mujoco.mjx._src import test_util
from mujoco.mjx.warp import io
import numpy as np
import warp as wp


_MODELFILE = flags.DEFINE_string(
    'modelfile',
    'humanoid/humanoid.xml',
    'path to model',
)
_NWORLD = flags.DEFINE_integer(
    'nworld', 4, 'number of worlds to render'
)
_WIDTH = flags.DEFINE_integer('width', 512, 'image width')
_HEIGHT = flags.DEFINE_integer('height', 512, 'image height')
_CAMERA_ID = flags.DEFINE_integer(
    'camera_id', 0, 'camera id to visualize'
)
_OUTPUT_DIR = flags.DEFINE_string(
    'output_dir', '/tmp/visualize_render', 'output directory'
)
_RANDOMIZE_QPOS = flags.DEFINE_boolean(
    'randomize_qpos', False, 'randomize initial qpos'
)
_USE_TEXTURES = flags.DEFINE_boolean(
    'use_textures', True, 'enable textures'
)
_USE_SHADOWS = flags.DEFINE_boolean(
    'use_shadows', True, 'enable shadows'
)
_WP_KERNEL_CACHE_DIR = flags.DEFINE_string(
    'wp_kernel_cache_dir',
    '/tmp/wp_kernel_cache_dir_visualize_render',
    'warp kernel cache directory',
)

_COMPILER_OPTIONS = {'xla_gpu_graph_min_graph_size': 1}
jax_jit = functools.partial(
    jax.jit, compiler_options=_COMPILER_OPTIONS
)


def _save_single(rgb, out_path):
  """Save first world as a single image."""
  img = np.asarray(rgb[0])
  img_uint8 = (img * 255).astype(np.uint8)
  media.write_image(out_path, img_uint8)
  print(f'  single image: {out_path}')


def _save_tiled(rgb, out_path):
  """Save all worlds as a tiled grid."""
  nworld, height, width, _ = rgb.shape
  cols = int(np.ceil(np.sqrt(nworld)))
  rows = int(np.ceil(nworld / cols))
  canvas = np.zeros(
      (rows * height, cols * width, 3), dtype=np.uint8
  )

  for w in range(nworld):
    img_uint8 = (np.asarray(rgb[w]) * 255).astype(
        np.uint8
    )
    r, c = w // cols, w % cols
    y0, y1 = r * height, (r + 1) * height
    x0, x1 = c * width, (c + 1) * width
    canvas[y0:y1, x0:x1, :] = img_uint8

  media.write_image(out_path, canvas)
  print(f'  tiled image:  {out_path}')


def _main(_: Sequence[str]):
  os.environ['MJX_WARP_ENABLED'] = 'true'

  wp.config.kernel_cache_dir = _WP_KERNEL_CACHE_DIR.value

  os.makedirs(_OUTPUT_DIR.value, exist_ok=True)

  try:
    m = test_util.load_test_file(_MODELFILE.value)
  except Exception:
    m = mujoco.MjModel.from_xml_path(_MODELFILE.value)

  print('visualize_render.py:\n')
  print(f'  modelfile   : {_MODELFILE.value}')
  print(f'  nworld      : {_NWORLD.value}')
  print(f'  resolution  : {_WIDTH.value}x{_HEIGHT.value}')
  print(f'  camera_id   : {_CAMERA_ID.value}')
  print(f'  use_textures: {_USE_TEXTURES.value}')
  print(f'  use_shadows : {_USE_SHADOWS.value}')
  print(f'  output_dir  : {_OUTPUT_DIR.value}\n')

  mx = mjx.put_model(m, impl='warp')

  worldids = jp.arange(_NWORLD.value)

  @jax.vmap
  def init(worldid):
    dx = mjx.make_data(m, impl='warp')
    rng = jax.random.PRNGKey(worldid)
    qpos0 = jp.array(m.qpos0)
    qpos = qpos0
    if _RANDOMIZE_QPOS.value:
      # TODO(robotics-team): consider integrating velocity if there are free
      # joints.
      qpos = qpos0 + jax.random.uniform(
          rng, (m.nq,), minval=-0.2, maxval=0.05
      )
    return dx.replace(qpos=qpos)

  print('initializing data...')
  dx_batch = jax_jit(init)(worldids)

  print('running forward...')
  dx_batch = jax_jit(
      jax.vmap(forward.forward, in_axes=(None, 0))
  )(mx, dx_batch)

  print('creating render context...')
  rc = io.create_render_context(
      mjm=m,
      nworld=_NWORLD.value,
      cam_res=(_WIDTH.value, _HEIGHT.value),
      use_textures=_USE_TEXTURES.value,
      use_shadows=_USE_SHADOWS.value,
      render_rgb=True,
      render_depth=True,
      enabled_geom_groups=[0, 1, 2],
  )

  print('rendering...')
  dx_batch = jax_jit(
      jax.vmap(
          bvh.refit_bvh, in_axes=(None, 0, None)
      )
  )(mx, dx_batch, rc)

  out_batch = jax_jit(
      jax.vmap(
          render.render, in_axes=(None, 0, None)
      )
  )(mx, dx_batch, rc)

  rgb_packed = out_batch[0]
  depth_packed = out_batch[1]
  print(f'  rgb shape:   {rgb_packed.shape}')
  print(f'  depth shape: {depth_packed.shape}\n')

  rgb = jax.vmap(
      render_util.get_rgb, in_axes=(None, 0, None)
  )(rc, rgb_packed, _CAMERA_ID.value)

  depth = jax.vmap(
      render_util.get_depth, in_axes=(None, 0, None, None)
  )(rc, depth_packed, _CAMERA_ID.value, 10.0)

  single_path = os.path.join(
      _OUTPUT_DIR.value, f'camera_{_CAMERA_ID.value}.png'
  )
  _save_single(rgb, single_path)

  depth_rgb = np.repeat(np.asarray(depth)[..., None], 3, axis=-1)
  depth_single_path = os.path.join(
      _OUTPUT_DIR.value, f'depth_{_CAMERA_ID.value}.png'
  )
  _save_single(depth_rgb, depth_single_path)

  if _NWORLD.value > 1:
    tiled_path = os.path.join(
        _OUTPUT_DIR.value, f'tiled_{_CAMERA_ID.value}.png'
    )
    _save_tiled(rgb, tiled_path)

    depth_tiled_path = os.path.join(
        _OUTPUT_DIR.value, f'depth_tiled_{_CAMERA_ID.value}.png'
    )
    _save_tiled(depth_rgb, depth_tiled_path)

  print('\ndone.')


def main():
  app.run(_main)


if __name__ == '__main__':
  main()
