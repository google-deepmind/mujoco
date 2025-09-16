# Copyright 2025 The Newton Developers
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

import warp as wp
from absl.testing import absltest

from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import kernel as nested_kernel

from mujoco.mjx.third_party.mujoco_warp._src import test_util
from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk import ccd
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import Geom
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model

MAX_ITERATIONS = 20


def _geom_dist(m: Model, d: Data, gid1: int, gid2: int, iterations: int, multiccd=False):
  @nested_kernel(module="unique", enable_backward=False)
  def _gjk_kernel(
    # Model:
    geom_type: wp.array(dtype=int),
    geom_dataid: wp.array(dtype=int),
    geom_size: wp.array2d(dtype=wp.vec3),
    mesh_vertadr: wp.array(dtype=int),
    mesh_vertnum: wp.array(dtype=int),
    mesh_vert: wp.array(dtype=wp.vec3),
    mesh_polynum: wp.array(dtype=int),
    mesh_polyadr: wp.array(dtype=int),
    mesh_polynormal: wp.array(dtype=wp.vec3),
    mesh_polyvertadr: wp.array(dtype=int),
    mesh_polyvertnum: wp.array(dtype=int),
    mesh_polyvert: wp.array(dtype=int),
    mesh_polymapadr: wp.array(dtype=int),
    mesh_polymapnum: wp.array(dtype=int),
    mesh_polymap: wp.array(dtype=int),
    # Data in:
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    # In:
    gid1: int,
    gid2: int,
    iterations: int,
    vert: wp.array(dtype=wp.vec3),
    vert1: wp.array(dtype=wp.vec3),
    vert2: wp.array(dtype=wp.vec3),
    vert_index1: wp.array(dtype=int),
    vert_index2: wp.array(dtype=int),
    face: wp.array(dtype=wp.vec3i),
    face_pr: wp.array(dtype=wp.vec3),
    face_norm2: wp.array(dtype=float),
    face_index: wp.array(dtype=int),
    face_map: wp.array(dtype=int),
    horizon: wp.array(dtype=int),
    polygon: wp.array(dtype=wp.vec3),
    clipped: wp.array(dtype=wp.vec3),
    pnormal: wp.array(dtype=wp.vec3),
    pdist: wp.array(dtype=float),
    idx1: wp.array(dtype=int),
    idx2: wp.array(dtype=int),
    n1: wp.array(dtype=wp.vec3),
    n2: wp.array(dtype=wp.vec3),
    endvert: wp.array(dtype=wp.vec3),
    face1: wp.array(dtype=wp.vec3),
    face2: wp.array(dtype=wp.vec3),
    # Out:
    dist_out: wp.array(dtype=float),
    ncon_out: wp.array(dtype=int),
    pos_out: wp.array(dtype=wp.vec3),
  ):
    MESHGEOM = int(GeomType.MESH.value)

    geom1 = Geom()
    geom1.index = -1
    geomtype1 = geom_type[gid1]
    geom1.pos = geom_xpos_in[0, gid1]
    geom1.rot = geom_xmat_in[0, gid1]
    geom1.size = geom_size[0, gid1]
    geom1.graphadr = -1
    geom1.mesh_polyadr = -1

    if geom_dataid[gid1] >= 0 and geom_type[gid1] == MESHGEOM:
      dataid = geom_dataid[gid1]
      geom1.vertadr = mesh_vertadr[dataid]
      geom1.vertnum = mesh_vertnum[dataid]
      geom1.mesh_polynum = mesh_polynum[dataid]
      geom1.mesh_polyadr = mesh_polyadr[dataid]
      geom1.vert = mesh_vert
      geom1.mesh_polynormal = mesh_polynormal
      geom1.mesh_polyvertadr = mesh_polyvertadr
      geom1.mesh_polyvertnum = mesh_polyvertnum
      geom1.mesh_polyvert = mesh_polyvert
      geom1.mesh_polymapadr = mesh_polymapadr
      geom1.mesh_polymapnum = mesh_polymapnum
      geom1.mesh_polymap = mesh_polymap

    geom2 = Geom()
    geom2.index = -1
    geomtype2 = geom_type[gid2]
    geom2.pos = geom_xpos_in[0, gid2]
    geom2.rot = geom_xmat_in[0, gid2]
    geom2.size = geom_size[0, gid2]
    geom2.graphadr = -1
    geom2.mesh_polyadr = -1

    if geom_dataid[gid2] >= 0 and geom_type[gid2] == MESHGEOM:
      dataid = geom_dataid[gid2]
      geom2.vertadr = mesh_vertadr[dataid]
      geom2.vertnum = mesh_vertnum[dataid]
      geom2.mesh_polynum = mesh_polynum[dataid]
      geom2.mesh_polyadr = mesh_polyadr[dataid]
      geom2.vert = mesh_vert
      geom2.mesh_polynormal = mesh_polynormal
      geom2.mesh_polyvertadr = mesh_polyvertadr
      geom2.mesh_polyvertnum = mesh_polyvertnum
      geom2.mesh_polyvert = mesh_polyvert
      geom2.mesh_polymapadr = mesh_polymapadr
      geom2.mesh_polymapnum = mesh_polymapnum
      geom2.mesh_polymap = mesh_polymap

    x_1 = geom_xpos_in[0, gid1]
    x_2 = geom_xpos_in[0, gid2]

    (
      dist,
      ncon,
      x1,
      x2,
    ) = ccd(
      multiccd,
      1e-6,
      1.0e30,
      iterations,
      iterations,
      geom1,
      geom2,
      geomtype1,
      geomtype2,
      x_1,
      x_2,
      vert,
      vert1,
      vert2,
      vert_index1,
      vert_index2,
      face,
      face_pr,
      face_norm2,
      face_index,
      face_map,
      horizon,
      polygon,
      clipped,
      pnormal,
      pdist,
      idx1,
      idx2,
      n1,
      n2,
      endvert,
      face1,
      face2,
    )

    dist_out[0] = dist
    ncon_out[0] = ncon
    pos_out[0] = x1[0]
    pos_out[1] = x2[0]

  vert = wp.array(shape=(iterations,), dtype=wp.vec3)
  vert1 = wp.array(shape=(iterations,), dtype=wp.vec3)
  vert2 = wp.array(shape=(iterations,), dtype=wp.vec3)
  vert_index1 = wp.array(shape=(iterations,), dtype=int)
  vert_index2 = wp.array(shape=(iterations,), dtype=int)
  face = wp.array(shape=(6 * iterations,), dtype=wp.vec3i)
  face_pr = wp.array(shape=(6 * iterations,), dtype=wp.vec3)
  face_norm2 = wp.array(shape=(6 * iterations,), dtype=float)
  face_index = wp.array(shape=(6 * iterations,), dtype=int)
  face_map = wp.array(shape=(6 * iterations,), dtype=int)
  horizon = wp.array(shape=(6 * iterations,), dtype=int)
  polygon = wp.array(shape=(150,), dtype=wp.vec3)
  clipped = wp.array(shape=(150,), dtype=wp.vec3)
  pnormal = wp.array(shape=(150,), dtype=wp.vec3)
  pdist = wp.array(shape=(150,), dtype=float)
  idx1 = wp.array(shape=(150,), dtype=int)
  idx2 = wp.array(shape=(150,), dtype=int)
  n1 = wp.array(shape=(150,), dtype=wp.vec3)
  n2 = wp.array(shape=(150,), dtype=wp.vec3)
  endvert = wp.array(shape=(150,), dtype=wp.vec3)
  face1 = wp.array(shape=(150,), dtype=wp.vec3)
  face2 = wp.array(shape=(150,), dtype=wp.vec3)
  dist_out = wp.array(shape=(1,), dtype=float)
  ncon_out = wp.array(shape=(1,), dtype=int)
  pos_out = wp.array(shape=(2,), dtype=wp.vec3)
  wp.launch(
    _gjk_kernel,
    dim=(1,),
    inputs=[
      m.geom_type,
      m.geom_dataid,
      m.geom_size,
      m.mesh_vertadr,
      m.mesh_vertnum,
      m.mesh_vert,
      m.mesh_polynum,
      m.mesh_polyadr,
      m.mesh_polynormal,
      m.mesh_polyvertadr,
      m.mesh_polyvertnum,
      m.mesh_polyvert,
      m.mesh_polymapadr,
      m.mesh_polymapnum,
      m.mesh_polymap,
      d.geom_xpos,
      d.geom_xmat,
      gid1,
      gid2,
      iterations,
      vert,
      vert1,
      vert2,
      vert_index1,
      vert_index2,
      face,
      face_pr,
      face_norm2,
      face_index,
      face_map,
      horizon,
      polygon,
      clipped,
      pnormal,
      pdist,
      idx1,
      idx2,
      n1,
      n2,
      endvert,
      face1,
      face2,
    ],
    outputs=[
      dist_out,
      ncon_out,
      pos_out,
    ],
  )
  return dist_out.numpy()[0], ncon_out.numpy()[0], pos_out.numpy()[0], pos_out.numpy()[1]


class GJKTest(absltest.TestCase):
  """Tests for GJK/EPA."""

  def test_spheres_distance(self):
    """Test distance between two spheres."""

    _, _, m, d = test_util.fixture(
      xml=f"""
      <mujoco>
        <worldbody>
          <geom name="geom1" type="sphere" pos="-1.5 0 0" size="1"/>
          <geom name="geom2" type="sphere" pos="1.5 0 0" size="1"/>
        </worldbody>
       </mujoco>
       """
    )

    dist, _, _, _ = _geom_dist(m, d, 0, 1, MAX_ITERATIONS)
    self.assertEqual(1.0, dist)

  def test_spheres_touching(self):
    """Test two touching spheres have zero distance"""

    _, _, m, d = test_util.fixture(
      xml=f"""
      <mujoco>
        <worldbody>
          <geom type="sphere" pos="-1 0 0" size="1"/>
          <geom type="sphere" pos="1 0 0" size="1"/>
        </worldbody>
       </mujoco>
       """
    )

    dist, _, _, _ = _geom_dist(m, d, 0, 1, MAX_ITERATIONS)
    self.assertEqual(0.0, dist)

  def test_box_mesh_distance(self):
    """Test distance between a mesh and box"""

    _, _, m, d = test_util.fixture(
      xml=f"""
      <mujoco model="MuJoCo Model">
        <asset>
          <mesh name="smallbox" scale="0.1 0.1 0.1"
                vertex="-1 -1 -1
                         1 -1 -1
                         1  1 -1
                         1  1  1
                         1 -1  1
                        -1  1 -1
                        -1  1  1
                        -1 -1  1"/>
         </asset>
         <worldbody>
           <geom pos="0 0 .90" type="box" size="0.5 0.5 0.1"/>
           <geom pos="0 0 1.2" type="mesh" mesh="smallbox"/>
          </worldbody>
       </mujoco>
       """
    )

    dist, _, _, _ = _geom_dist(m, d, 0, 1, MAX_ITERATIONS)
    self.assertAlmostEqual(0.1, dist)

  def test_sphere_sphere_contact(self):
    """Test penetration depth between two spheres."""

    _, _, m, d = test_util.fixture(
      xml=f"""
      <mujoco>
        <worldbody>
          <geom type="sphere" pos="-1 0 0" size="3"/>
          <geom type="sphere" pos=" 3 0 0" size="3"/>
        </worldbody>
      </mujoco>
      """
    )

    dist, _, _, _ = _geom_dist(m, d, 0, 1, 0)
    self.assertAlmostEqual(-2, dist)

  def test_box_box_contact(self):
    """Test penetration between two boxes."""

    _, _, m, d = test_util.fixture(
      xml=f"""
      <mujoco>
        <worldbody>
          <geom type="box" pos="-1 0 0" size="2.5 2.5 2.5"/>
          <geom type="box" pos="1.5 0 0" size="1 1 1"/>
        </worldbody>
      </mujoco>
      """
    )
    dist, _, x1, x2 = _geom_dist(m, d, 0, 1, MAX_ITERATIONS)
    self.assertAlmostEqual(-1, dist)
    normal = wp.normalize(x1 - x2)
    self.assertAlmostEqual(normal[0], 1)
    self.assertAlmostEqual(normal[1], 0)
    self.assertAlmostEqual(normal[2], 0)

  def test_mesh_mesh_contact(self):
    """Test penetration between two meshes."""

    _, _, m, d = test_util.fixture(
      xml=f"""
    <mujoco>
      <asset>
        <mesh name="box" scale=".5 .5 .1"
              vertex="-1 -1 -1
                       1 -1 -1
                       1  1 -1
                       1  1  1
                       1 -1  1
                      -1  1 -1
                      -1  1  1
                      -1 -1  1"/>
        <mesh name="smallbox" scale=".1 .1 .1"
              vertex="-1 -1 -1
                       1 -1 -1
                       1  1 -1
                       1  1  1
                       1 -1  1
                      -1  1 -1
                      -1  1  1
                      -1 -1  1"/>
      </asset>

      <worldbody>
        <geom pos="0 0 .09" type="mesh" mesh="smallbox"/>
        <geom pos="0 0 -.1" type="mesh" mesh="box"/>
      </worldbody>
    </mujoco>
    """
    )
    dist, _, _, _ = _geom_dist(m, d, 0, 1, MAX_ITERATIONS)
    self.assertAlmostEqual(-0.01, dist)

  def test_cylinder_cylinder_contact(self):
    """Test penetration between two cylinder."""

    _, _, m, d = test_util.fixture(
      xml=f"""
      <mujoco>
        <worldbody>
          <geom pos="0 0 0" type="cylinder" size="1 .5"/>
          <geom pos="1.999 0 0" type="cylinder" size="1 .5"/>
        </worldbody>
      </mujoco>
    """
    )

    dist, _, _, _ = _geom_dist(m, d, 0, 1, 50)
    self.assertAlmostEqual(-0.001, dist)

  def test_box_edge(self):
    """Test box edge."""

    _, _, m, d = test_util.fixture(
      xml=f"""
    <mujoco>
      <worldbody>
        <geom pos="0 0 2" type="box" name="box2" size="1 1 1"/>
        <geom pos="0 0 4.4" euler="0 90 40" type="box" name="box3" size="1 1 1"/>
      </worldbody>
    </mujoco>"""
    )
    _, ncon, _, _ = _geom_dist(m, d, 0, 1, MAX_ITERATIONS, multiccd=True)
    self.assertEqual(ncon, 2)

  def test_box_box_ccd(self):
    """Test box box."""

    _, _, m, d = test_util.fixture(
      xml=f"""
       <mujoco>
         <worldbody>
           <geom name="geom1" type="box" pos="0 0 1.9" size="1 1 1"/>
           <geom name="geom2" type="box" pos="0 0 0" size="10 10 1"/>
         </worldbody>
       </mujoco>
       """
    )
    _, ncon, _, _ = _geom_dist(m, d, 0, 1, MAX_ITERATIONS, multiccd=True)
    self.assertEqual(ncon, 4)

  def test_mesh_mesh_ccd(self):
    """Test mesh-mesh multiccd."""

    _, _, m, d = test_util.fixture(
      xml=f"""
       <mujoco>
         <asset>
           <mesh name="smallbox"
                 vertex="-1 -1 -1 1 -1 -1 1 1 -1 1 1 1 1 -1 1 -1 1 -1 -1 1 1 -1 -1 1"/>
         </asset>
         <worldbody>
           <geom pos="0 0 2" type="mesh" name="box1" mesh="smallbox"/>
          <geom pos="0 1 3.99" euler="0 0 40" type="mesh" name="box2" mesh="smallbox"/>
         </worldbody>
       </mujoco>
       """
    )

    _, ncon, _, _ = _geom_dist(m, d, 0, 1, MAX_ITERATIONS, multiccd=True)
    self.assertEqual(ncon, 4)

  def test_box_box_ccd2(self):
    """Test box-box multiccd 2."""

    _, _, m, d = test_util.fixture(
      xml=f"""
       <mujoco>
         <worldbody>
           <geom size="1 1 1" pos="0 0 2" type="box"/>
          <geom size="1 1 1" pos="0 1 3.99" euler="0 0 40" type="box"/>
         </worldbody>
       </mujoco>
       """
    )

    _, ncon, _, _ = _geom_dist(m, d, 0, 1, MAX_ITERATIONS, multiccd=True)
    self.assertEqual(ncon, 4)


if __name__ == "__main__":
  wp.init()
  absltest.main()
