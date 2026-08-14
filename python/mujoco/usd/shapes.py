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

from typing import Any, Dict, Optional, Tuple, Union

import mujoco
import numpy as np


def get_triangle_uvs(
    vertices: np.ndarray,
    triangles: np.ndarray,
    texture_type: Optional[mujoco.mjtTexture]
):
  """Returns UV coordinates for a given mesh."""
  if texture_type is None:
    return None

  triangle_uvs = []
  if texture_type == mujoco.mjtTexture.mjTEXTURE_2D:
    triangle_uvs = [
        [vertices[i][0], vertices[i][1]] for i in np.nditer(triangles)
    ]

  elif texture_type == mujoco.mjtTexture.mjTEXTURE_CUBE:
    center = np.mean(vertices, axis=0)
    for vertex_id in np.nditer(triangles):
      x, y, z = vertices[vertex_id] - center

      abs_x, abs_y, abs_z = abs(x), abs(y), abs(z)
      u = 0
      v = 0

      if x > 0 and abs_x >= abs_y and abs_x >= abs_z:
        u = -z / abs_x
        v = y / abs_x
      elif x <= 0 and abs_x >= abs_y and abs_x >= abs_z:
        u = z / abs_x
        v = y / abs_x
      elif y > 0 and abs_y >= abs_x and abs_y >= abs_z:
        u = x / abs_y
        v = -z / abs_y
      elif y <= 0 and abs_y >= abs_x and abs_y >= abs_z:
        u = x / abs_y
        v = z / abs_y
      elif z > 0 and abs_z >= abs_x and abs_z >= abs_y:
        u = x / abs_z
        v = y / abs_z
      elif z <= 0 and abs_z >= abs_x and abs_z >= abs_y:
        u = -x / abs_z
        v = y / abs_z

      u = (u + 1.0) / 2.0
      v = (v + 1.0) / 2.0
      v /= 6

      assert 0 <= u and u <= 1 and 0 <= v and v <= 1

      triangle_uvs.append([u, v])
  elif texture_type == mujoco.mjtTexture.mjTEXTURE_SKYBOX:
    # defaults to 2D mapping temporarily
    triangle_uvs = [
        [vertices[i][0], vertices[i][1]] for i in np.nditer(triangles)
    ]

  return np.array(triangle_uvs)


class TriangleMesh:
  """Store UV and geometry information for a primitive mesh.

  Attributes:
    vertices: A numpy array of vertices.
    triangles: A numpy array of triangles.
    triangle_uvs: A numpy array of UV coordinates.
  """

  def __init__(self,
               vertices: np.ndarray,
               triangles: np.ndarray,
               triangle_uvs: np.ndarray):
    """Creates a TriangleMesh object.

    Args:
      vertices: A numpy array of vertices.
      triangles: A numpy array of triangles.
      triangle_uvs: A numpy array of UV coordinates.
    """
    self.vertices = vertices
    self.triangles = triangles
    self.triangle_uvs = triangle_uvs

  @classmethod
  def create_box(
      cls,
      width: float,
      height: float,
      depth: float,
      texture_type: Optional[mujoco.mjtTexture]
  ) -> "TriangleMesh":
    """Creates a box."""
    vertices = np.array([[0.0, 0.0, 0.0],
                         [width, 0.0, 0.0],
                         [0.0, 0.0, depth],
                         [width, 0.0, depth],
                         [0.0, height, 0.0],
                         [width, height, 0.0],
                         [0.0, height, depth],
                         [width, height, depth]])

    triangles = np.array([[4, 7, 5],
                          [4, 6, 7],
                          [0, 2, 4],
                          [2, 6, 4],
                          [0, 1, 2],
                          [1, 3, 2],
                          [1, 5, 7],
                          [1, 7, 3],
                          [2, 3, 7],
                          [2, 7, 6],
                          [0, 4, 1],
                          [1, 4, 5]])

    triangle_uvs = get_triangle_uvs(vertices, triangles, texture_type)

    return TriangleMesh(vertices, triangles, triangle_uvs)

  @classmethod
  def create_sphere(
      cls,
      radius: float,
      texture_type: Optional[mujoco.mjtTexture],
      resolution: int
  ) -> "TriangleMesh":
    """Creates a sphere."""
    vertices = []
    triangles = []
    for i in range(2*resolution + 1):
      phi = np.pi * i / (2*resolution)
      for j in range(resolution + 1):
        theta = 2 * np.pi * j / resolution
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi)
        vertices.append([x, y, z])

    for i in range(2*resolution):
      for j in range(resolution):
        first = i * (resolution + 1) + j
        second = first + resolution + 1

        triangles.append([first, second, first + 1])
        triangles.append([second, second + 1, first + 1])

    vertices = np.array(vertices)
    triangles = np.array(triangles)

    triangle_uvs = get_triangle_uvs(vertices, triangles, texture_type)

    return TriangleMesh(vertices, triangles, triangle_uvs)

  @classmethod
  def create_hemisphere(
      cls,
      radius: float,
      texture_type: Optional[mujoco.mjtTexture],
      resolution: int,
  ) -> "TriangleMesh":
    """Creates a hemisphere."""
    vertices = []
    triangles = []
    for i in range(resolution + 1):
      phi = np.pi / 2 * i / (resolution)
      for j in range(resolution + 1):
        theta = 2 * np.pi * j / resolution
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi)
        vertices.append([x, y, z])
    vertices.append([0, 0, 0])

    for i in range(resolution):
      for j in range(resolution):
        first = i * (resolution + 1) + j
        second = first + resolution + 1

        triangles.append([first, second, first + 1])
        triangles.append([second, second + 1, first + 1])

    for i in range(resolution):
      first = resolution * (resolution + 1) + i
      triangles.append([first, first + 1, len(vertices) - 1])

    vertices = np.array(vertices)
    triangles = np.array(triangles)

    triangle_uvs = get_triangle_uvs(vertices, triangles, texture_type)

    return TriangleMesh(vertices, triangles, triangle_uvs)

  @classmethod
  def create_cylinder(
      cls,
      radius: float,
      height: float,
      texture_type: Optional[mujoco.mjtTexture],
      resolution: int
  ) -> "TriangleMesh":
    """Creates a cylinder."""
    vertices = []
    triangles = []

    # adding all the vertices for the cylinder including
    # two center vertices at ends
    for i in range(2):
      z = 0 if i == 0 else height
      for j in range(resolution + 1):
        theta = 2 * np.pi * j / resolution
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        vertices.append([x, y, z])
    vertices.append([0, 0, 0])
    vertices.append([0, 0, height])

    # constructing the end faces for the cylinder
    for i in range(2):
      for j in range(resolution):
        first = (resolution + 1) * i + j
        triangles.append([first, first + 1, len(vertices) - (2 - i)])

    # constructing side of cylinder
    for i in range(resolution):
      second = resolution + 1 + i
      triangles.append([i, second, second + 1])
      triangles.append([i, i + 1, second + 1])

    vertices = np.array(vertices)
    triangles = np.array(triangles)

    triangle_uvs = get_triangle_uvs(vertices, triangles, texture_type)

    return TriangleMesh(vertices, triangles, triangle_uvs)

  def translate(self, translation: np.ndarray) -> None:
    self.vertices = self.vertices + translation

  def rotate(self, rotation: np.ndarray, center: Tuple[float, ...]) -> None:
    translated_point = self.vertices - center
    self.vertices = np.dot(translated_point, rotation) + center

  def scale(self, scale: np.ndarray) -> None:
    self.vertices = self.vertices * scale

  def get_center(self):
    center = np.mean(self.vertices, axis=0)
    return center

  def __add__(self, other):
    if isinstance(other, TriangleMesh):
      new_vertices = np.vstack((self.vertices, other.vertices))
      other_triangles = other.triangles + len(self.vertices)
      new_triangles = np.vstack((self.triangles, other_triangles))
      new_triangle_uvs = None
      if self.triangle_uvs is not None:
        new_triangle_uvs = np.vstack((self.triangle_uvs, other.triangle_uvs))
      return TriangleMesh(new_vertices, new_triangles, new_triangle_uvs)
    raise TypeError(f"Cannot add TriangleMesh with {type(other)}")


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
    geom_type: Union[int, mujoco.mjtGeom],
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
    cylinder["cylinder"]["transform"] = {
        "transform": {"translate": (0, 0, size[2])}
    }
    config = {
        "name": name,
        "cylinder": cylinder["cylinder"],
        "left_hemisphere": {
            "radius": size[0],
            "transform": {
                "rotate": (np.pi, 0, 0),
            },
        },
        "right_hemisphere": {
            "radius": size[0],
            "transform": {"translate": (0, 0, 2*size[2])},
        }
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


def mesh_factory(
    mesh_config: Dict[str, Any],
    texture_type: Optional[mujoco.mjtTexture],
    resolution: int = 100,
):
  """Generates a mesh given a config consisting of shapes."""
  assert "name" in mesh_config

  mesh = None

  for shape, config in mesh_config.items():

    if "name" in shape:
      continue

    if "box" in shape:
      prim_mesh = TriangleMesh.create_box(
          width=mesh_config[shape]["width"],
          height=mesh_config[shape]["height"],
          depth=mesh_config[shape]["depth"],
          texture_type=texture_type
      )
    elif "hemisphere" in shape:
      prim_mesh = TriangleMesh.create_hemisphere(
          radius=mesh_config[shape]["radius"],
          texture_type=texture_type,
          resolution=resolution
      )
    elif "sphere" in shape:
      prim_mesh = TriangleMesh.create_sphere(
          radius=mesh_config[shape]["radius"],
          texture_type=texture_type,
          resolution=resolution
      )
    elif "cylinder" in shape:
      prim_mesh = TriangleMesh.create_cylinder(
          radius=mesh_config[shape]["radius"],
          height=mesh_config[shape]["height"],
          texture_type=texture_type,
          resolution=resolution
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
        prim_mesh.scale(config["transform"]["scale"])
      if "translate" in config["transform"]:
        prim_mesh.translate(config["transform"]["translate"])

    if not mesh:
      mesh = prim_mesh
    else:
      mesh += prim_mesh

  if mesh is None:
    raise ValueError("Mesh not found.")

  return mesh_config["name"], mesh
