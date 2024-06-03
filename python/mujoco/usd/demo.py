import argparse
from tqdm import tqdm
from pathlib import Path

import mujoco
from mujoco.usd import exporter

def generate_usd_trajectory(args):

  # load a model to mujoco
  model_path = args.model_path
  m = mujoco.MjModel.from_xml_path(model_path)
  d = mujoco.MjData(m)

  # create an instance of the USDExporter
  exp = exporter.USDExporter(model=m,
                             output_directory_name=Path(args.model_path).stem,
                             output_directory_root=args.output_directory_root,
                             camera_names=args.camera_names)

  # step through the model for length steps
  for i in tqdm(range(args.length)):
    for i in range(args.steps_per_frame):
      mujoco.mj_step(m, d)
    exp.update_scene(d)

  exp.save_scene(filetype=args.export_extension)

if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  parser.add_argument('--model_path', 
                      type=str,
                      required=True,
                      help='path to mjcf xml model')

  parser.add_argument('--length',
                      type=int,
                      default=100,
                      help='length of trajectory to render')

  parser.add_argument('--output_directory_root', 
                      type=str,
                      default="../usd_trajectories/",
                      help='location where to create usd files')

  parser.add_argument('--camera_names', 
                      type=str,
                      nargs='+',
                      help='cameras to include in usd')

  parser.add_argument('--export_extension', 
                      type=str,
                      default="usd",
                      help='extension of exported file (can be usd, usda, or usdc)')

  parser.add_argument('--steps_per_frame',
                      type=int,
                      default=1,
                      help='number of frames to skip for each rendering step')

  args = parser.parse_args()
  generate_usd_trajectory(args)

  


