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
"""Built-in shapes for USD exporter."""

from typing import Dict, Any

import mujoco
import numpy as np
from open3d import open3d as o3d


def create_hemisphere(
    radius: float, theta_steps: int = 50, phi_steps: int = 50
):
  """Creates a hemisphere mesh from a point cloud."""
  points = []
  for i in range(phi_steps + 1):
    phi = np.pi / 2 * i / phi_steps
    for j in range(theta_steps + 1):
      theta = 2 * np.pi * j / theta_steps
      x = radius * np.sin(phi) * np.cos(theta)
      y = radius * np.sin(phi) * np.sin(theta)
      z = radius * np.cos(phi)
      points.append([x, y, z])

  pcd = o3d.geometry.PointCloud()
  pcd.points = o3d.utility.Vector3dVector(points)

  mesh = pcd.compute_convex_hull()[0]

  return mesh


def decouple_config(config: Dict[str, Any]):
  """Breaks a shape config into is subcomponent shapes."""
  decoupled_config = []
  for key, value in config.items():
    if key == "name":
      continue
    decoupled_config.append({
        "parent_name": config["name"],
        "name": config["name"] + "_" + key,
        key: value.copy(),
    })

  return decoupled_config


def mesh_config_generator(
    name: str,
    geom_type: int | mujoco.mjtGeom,
    size: np.ndarray,
    decouple: bool = False,
):
  """Creates a config for a particular mesh."""
  if geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
    config = {
        "name": name,
        "box": {
            "width": size[0] * 2 if size[0] > 0 else 100,
            "height": size[1] * 2 if size[1] > 0 else 100,
            "depth": 0.001,
            "map_texture_to_each_face": True,
        },
    }
  elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
    config = {"name": name, "sphere": {"radius": float(size[0])}}
  elif geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
    cylinder = mesh_config_generator(name, mujoco.mjtGeom.mjGEOM_CYLINDER, size)
    config = {
        "name": name,
        "cylinder": cylinder["cylinder"],
        "left_hemisphere": {
            "radius": size[0],
            "transform": {
                "translate": (0, 0, -size[2]),
                "rotate": (np.pi, 0, 0),
            },
        },
        "right_hemisphere": {
            "radius": size[0],
            "transform": {"translate": (0, 0, size[2])},
        },
    }
  elif geom_type == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
    sphere = mesh_config_generator(
        name, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([1.0])
    )
    sphere["sphere"]["transform"] = {"scale": tuple(size)}
    config = {
        "name": name,
        "sphere": sphere["sphere"],
    }
  elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
    config = {
        "name": name,
        "cylinder": {
            "radius": size[0],
            "height": size[2] * 2,
        },
    }
  elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
    config = {
        "name": name,
        "box": {
            "width": size[0] * 2,
            "height": size[1] * 2,
            "depth": size[2] * 2,
        },
    }
  else:
    raise NotImplementedError(
        f"{geom_type} primitive geom type not implemented with USD integration"
    )

  if decouple:
    config = decouple_config(config)

  return config


def mesh_generator(
    mesh_config: Dict[str, Any],
    resolution: int = 100,
):
  """Generates a mesh given a config consisting of shapes."""
  assert "name" in mesh_config

  mesh = None

  for shape, config in mesh_config.items():

    if "name" in shape:
      continue

    if "box" in shape:
      prim_mesh = o3d.geometry.TriangleMesh.create_box(
          width=mesh_config[shape]["width"],
          height=mesh_config[shape]["height"],
          depth=mesh_config[shape]["depth"],
          create_uv_map=True,
          map_texture_to_each_face=True,
      )
    elif "hemisphere" in shape:
      prim_mesh = create_hemisphere(radius=mesh_config[shape]["radius"])
    elif "sphere" in shape:
      prim_mesh = o3d.geometry.TriangleMesh.create_sphere(
          radius=mesh_config[shape]["radius"],
          resolution=resolution,
          create_uv_map=True,
      )
    elif "cylinder" in shape:
      prim_mesh = o3d.geometry.TriangleMesh.create_cylinder(
          radius=mesh_config[shape]["radius"],
          height=mesh_config[shape]["height"],
          resolution=resolution,
          create_uv_map=True,
      )
    else:
      raise ValueError("Shape not supported")

    if "transform" in config:
      if "rotate" in config["transform"]:
        rotation = np.zeros(9)
        quat = np.zeros(4)
        euler = config["transform"]["rotate"]
        seq = "xyz"
        mujoco.mju_euler2Quat(quat, euler, seq)
        mujoco.mju_quat2Mat(rotation, quat)
        rotation = rotation.reshape((3, 3))
        prim_mesh.rotate(rotation, center=(0, 0, 0))
      if "scale" in config["transform"]:
        prim_mesh.vertices = o3d.utility.Vector3dVector(
            np.asarray(prim_mesh.vertices)
            * np.array(config["transform"]["scale"])
        )
      if "translate" in config["transform"]:
        prim_mesh.translate(config["transform"]["translate"])

    if not mesh:
      mesh = prim_mesh
    else:
      mesh += prim_mesh

  return mesh_config["name"], mesh
