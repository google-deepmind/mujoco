import mujoco
from mujoco.usd import exporter

if __name__ == "__main__":

  # load a model to mujoco
  m = mujoco.MjModel.from_xml_path("/Users/abhishek/Documents/research/mujoco/model/humanoid/humanoid.xml")
  d = mujoco.MjData(m)

  # create an instance of the USDExporter
  exp = exporter.USDExporter(model=m)

  mujoco.mj_step(m, d)

  exp.update_scene(d)

  exp.save_scene(filetype="usd")


