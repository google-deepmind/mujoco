"""Optional MuJoCo Metal experiments; import does not initialize a GPU."""

__version__ = "0.2.0"


def __getattr__(name):
  if name in ("load_model", "ModelDescriptor"):
    from mujoco_metal import model

    return getattr(model, name)
  raise AttributeError(name)
