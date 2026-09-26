# Copyright 2026 The MuJoCo Metal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Optional MuJoCo Metal experiments; import does not initialize a GPU."""

__version__ = "0.2.0"


def __getattr__(name):
  if name in ("load_model", "ModelDescriptor"):
    from mujoco_metal import model

    return getattr(model, name)
  if name in ("ModelLifecycle", "KinematicsBatchState", "BatchedConstants"):
    from mujoco_metal import lifecycle

    return getattr(lifecycle, name)
  if name == "MetalKinematics":
    from mujoco_metal.metal_kinematics import MetalKinematics

    return MetalKinematics
  if name == "smooth_dynamics":
    from mujoco_metal.smooth import smooth_dynamics

    return smooth_dynamics
  raise AttributeError(name)
