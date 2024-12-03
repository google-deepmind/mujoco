# Copyright 2024 DeepMind Technologies Limited
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
"""Demo script for USD exporter."""

import argparse
import pathlib

import mujoco
from mujoco.usd import exporter


def generate_usd_trajectory(local_args):
  """Generates a USD file given the user arguments."""
  # load a model to mujoco
  model_path = local_args.model_path
  m = mujoco.MjModel.from_xml_path(model_path)
  d = mujoco.MjData(m)

  # create an instance of the USDExporter
  exp = exporter.USDExporter(
      model=m,
      output_directory_name=pathlib.Path(local_args.model_path).stem,
      output_directory_root=local_args.output_directory_root,
      camera_names=local_args.camera_names,
  )

  # step through the simulation for the given duration of time
  while d.time < local_args.duration:
    mujoco.mj_step(m, d)
    if exp.frame_count < d.time * local_args.framerate:
      exp.update_scene(data=d)

  exp.add_light(pos=(0, 0, 0),
                intensity=2000,
                light_type='dome')

  exp.save_scene(filetype=local_args.export_extension)


if __name__ == '__main__':

  parser = argparse.ArgumentParser()

  parser.add_argument(
      '--model_path', type=str, required=True, help='path to mjcf xml model'
  )

  parser.add_argument(
      '--duration',
      type=int,
      default=5,
      help='duration in seconds for the generated video',
  )

  parser.add_argument(
      '--framerate',
      type=int,
      default=60,
      help='frame rate of the generated video',
  )

  parser.add_argument(
      '--output_directory_root',
      type=str,
      default='../usd_trajectories/',
      help='location where to create usd files',
  )

  parser.add_argument(
      '--camera_names',
      type=str,
      nargs='+',
      help='cameras to include in usd'
  )

  parser.add_argument(
      '--export_extension',
      type=str,
      default='usd',
      help='extension of exported file (usd, usda, usdc)',
  )

  args = parser.parse_args()
  generate_usd_trajectory(args)
