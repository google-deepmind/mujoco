
import mujoco
# from mujoco.usd import exporter
import exporter

if __name__ == "__main__":

  # load a model to mujoco
  m = mujoco.MjModel.from_xml_path("/Users/abhishek/Documents/research/mujoco/model/car/car.xml")
  d = mujoco.MjData(m)

  # create an instance of the USDExporter
  exp = exporter.USDExporter(model=m)

  for i in range(100):
    mujoco.mj_step(m, d)
    exp.update_scene(d)

  exp.save_scene(filetype="usd")


