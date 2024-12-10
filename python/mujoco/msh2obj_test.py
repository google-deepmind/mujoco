# Copyright 2023 DeepMind Technologies Limited
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
"""Tests for msh2obj.py."""

from absl.testing import absltest
from etils import epath
import mujoco
from mujoco import msh2obj
import numpy as np


_MESH_FIELDS = (
    "mesh_vertadr",
    "mesh_vertnum",
    "mesh_faceadr",
    "mesh_facenum",
    "mesh_bvhadr",
    "mesh_bvhnum",
    "mesh_normaladr",
    "mesh_normalnum",
    "mesh_texcoordadr",
    "mesh_texcoordnum",
    "mesh_graphadr",
    "mesh_vert",
    "mesh_normal",
    "mesh_face",
    "mesh_facenormal",
    "mesh_facetexcoord",
    "mesh_graph",
    "mesh_texcoord",
)

_XML = """
<mujoco>
  <asset>
    <mesh name="abdomen_1_body" file="abdomen_1_body.obj"/>
  </asset>
</mujoco>
"""


class MshTest(absltest.TestCase):

  def test_obj_model_matches_msh_model(self) -> None:
    test_path = epath.resource_path("mujoco") / "testdata"

    msh_xml = test_path / "msh.xml"
    msh_model = mujoco.MjModel.from_xml_path(msh_xml.as_posix())

    msh_path = test_path / "abdomen_1_body.msh"
    obj = msh2obj.msh_to_obj(msh_path)

    obj_model = mujoco.MjModel.from_xml_string(
        _XML, {"abdomen_1_body.obj": obj.encode()}
    )

    for field in _MESH_FIELDS:
      np.testing.assert_allclose(
          getattr(msh_model, field),
          getattr(obj_model, field),
          atol=1e-6,
          err_msg=f"Field {field} does not match between msh and obj models.",
      )


if __name__ == "__main__":
  absltest.main()
