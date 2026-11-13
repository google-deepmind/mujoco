# Copyright 2026 DeepMind Technologies Limited
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
"""Deterministic tests for code snippets and engine claims in SKILL.md."""

import pathlib
import re
import tempfile

from absl.testing import absltest
import mujoco
import numpy as np

# Keep symlink path in runfiles intact (do not call .resolve()).
_SKILL_MD = pathlib.Path(__file__).parent.parent / 'SKILL.md'


def _fenced_blocks(lang: str) -> list[str]:
  text = _SKILL_MD.read_text(encoding='utf-8')
  pattern = rf'^```{lang}\n(.*?)^```$'
  return re.findall(pattern, text, flags=re.MULTILINE | re.DOTALL)


class SkillClaimsTest(absltest.TestCase):

  def test_all_xml_snippets_compile_and_step(self):
    xml_blocks = _fenced_blocks('xml')
    self.assertGreaterEqual(len(xml_blocks), 5)
    for idx, xml in enumerate(xml_blocks):
      with self.subTest(snippet=idx):
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        if 'model="compliant_pad"' in xml:
          mujoco.mj_step(model, data, 100)
          self.assertGreater(data.ncon, 0)
        elif 'model="clamped_cable"' in xml:
          # Verify v1 sits at mount (x = 0) and v16 sits at slider (x = 0.6).
          mujoco.mj_forward(model, data)
          self.assertAlmostEqual(float(data.flexvert_xpos[1, 0]), 0.0, places=6)
          self.assertAlmostEqual(
              float(data.flexvert_xpos[16, 0]), 0.6, places=6
          )
          data.ctrl[0] = 0.005
          mujoco.mj_step(model, data, 250)
          self.assertGreater(float(data.xpos[model.body('slider').id, 0]), 0.35)
          twist = float(data.joint('cable_twist').qpos[0])
          self.assertBetween(twist, 0.8, 3.0)
        elif (
            'model="soft_and_plastic"' in xml
            and mujoco.MJTNUM_DTYPE != np.float32
        ):
          # 3D dof="full" yield onset overflows R*R in float32 (gramDet3D).
          mujoco.mj_step(model, data, 150)
          self.assertGreater(float(np.max(np.abs(data.plastic))), 0.0)
        else:
          mujoco.mj_step(model, data, 5)
        self.assertEqual(
            data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number, 0
        )
        self.assertTrue(np.all(np.isfinite(data.flexvert_xpos)))

  def test_all_python_snippets_execute(self):
    py_blocks = _fenced_blocks('python')
    self.assertGreaterEqual(len(py_blocks), 2)
    for idx, code in enumerate(py_blocks):
      with self.subTest(snippet=idx):
        if 'point_attr' in code and 'import mujoco' not in code:
          ns = {'np': np, 'a': 0.2, 'nx': 7, 'ny': 8}
          exec(code, ns)  # pylint: disable=exec-used
          xml = f"""<mujoco>
            <option timestep="0.002" integrator="discrete" solver="Newton"/>
            <worldbody>
              <flexcomp name="plate" type="direct" dim="2" radius="0.001"
                        mass="0.1" point="{ns['point_attr']}"
                        element="{ns['elem_attr']}">
                <pin id="0"/>
                <elasticity young="1e6" poisson="0.3" thickness="0.002"
                            elastic2d="both" damping="0.01"/>
                <contact selfcollide="none"/>
              </flexcomp>
            </worldbody>
          </mujoco>"""
          model = mujoco.MjModel.from_xml_string(xml)
          self.assertGreater(model.nflexelem, 0)
        else:
          ns = {}
          exec(code, ns)  # pylint: disable=exec-used
          self.assertIsInstance(ns['model'], mujoco.MjModel)

  def test_interpolated_dim_and_elastic2d_are_independent(self):
    for dim in (2, 3):
      for elastic2d, expect_shell in (('none', False), ('bend', True)):
        thick = 'thickness="0.01"' if expect_shell else ''
        xml = f"""<mujoco>
          <option integrator="discrete"/>
          <worldbody>
            <flexcomp name="f" type="box" dim="{dim}" dof="trilinear"
                      count="4 4 4" cellcount="2 2 2" spacing="0.04 0.04 0.04"
                      pos="0 0 0.2" radius="0.002" mass="0.2">
              <elasticity young="1e5" poisson="0.2"
                          elastic2d="{elastic2d}" {thick}/>
              <contact selfcollide="none"/>
            </flexcomp>
          </worldbody>
        </mujoco>"""
        model = mujoco.MjModel.from_xml_string(xml)
        self.assertEqual(int(model.flex_dim[0]), dim)
        if expect_shell:
          self.assertLess(int(model.flex_interp[0]), 0)
          # Center node (1, 1, 1) of a 2x2x2 shell lattice is auto-pinned.
          center_body = int(model.flex_nodebodyid[1 * 9 + 1 * 3 + 1])
          self.assertEqual(int(model.body_weldid[center_body]), 0)
        else:
          self.assertGreater(int(model.flex_interp[0]), 0)
          center_body = int(model.flex_nodebodyid[1 * 9 + 1 * 3 + 1])
          self.assertGreater(int(model.body_dofnum[center_body]), 0)

    # Standalone continuum shell requires elastic2d='both' for membrane stretch;
    # elastic2d='bend' without strain equalities collapses under load.
    def shell_load_z(elastic2d: str) -> float:
      xml = f"""<mujoco>
        <option timestep="0.002" integrator="discrete" solver="Newton"/>
        <worldbody>
          <geom type="plane" size="1 1 0.1" solref="0.004 1"/>
          <flexcomp name="c" type="grid" dim="3" dof="trilinear" count="7 7 7"
                    cellcount="2 2 2" spacing="0.02 0.02 0.02" pos="0 0 0.062"
                    radius="0.002" mass="0.3">
            <elasticity young="8e4" poisson="0.25" thickness="0.004"
                        elastic2d="{elastic2d}"/>
            <contact selfcollide="none" solref="0.004 1"/>
          </flexcomp>
          <body name="load" pos="0 0 0.16">
            <freejoint/>
            <geom type="sphere" size="0.03" mass="0.25" solref="0.004 1"/>
          </body>
        </worldbody>
      </mujoco>"""
      m = mujoco.MjModel.from_xml_string(xml)
      d = mujoco.MjData(m)
      mujoco.mj_step(m, d, 400)
      return float(d.body('load').xpos[2])

    self.assertGreater(shell_load_z('both'), 0.06)
    self.assertLess(shell_load_z('bend'), 0.045)

    # Flat 2D sheet has a degenerate 3D bounding box for interpolated lattices.
    for dof in ('trilinear', 'quadratic'):
      flat_xml = f"""<mujoco>
        <option integrator="discrete"/>
        <worldbody>
          <flexcomp name="flat" type="grid" dim="2" dof="{dof}" count="5 5 1"
                    cellcount="2 2 1" spacing="0.05 0.05 0.05" radius="0.002">
            <elasticity young="1e5" thickness="0.002" elastic2d="both"/>
          </flexcomp>
        </worldbody>
      </mujoco>"""
      with self.assertRaisesRegex(ValueError, 'R0 is not orthonormal'):
        mujoco.MjModel.from_xml_string(flat_xml)

  def test_effsolve_pcg_runs_for_all_solvers_and_exact_on_rods_and_2d(self):
    # 1D rod and non-interpolated 2D flex use exact sparse Cholesky (1 iter).
    rod_xml = """<mujoco>
      <option timestep="0.01" integrator="discrete" iterations="1"
              tolerance="1e-8"/>
      <worldbody>
        <body name="m">
          <flexcomp name="r" type="grid" dim="1" count="9 1 1"
                    spacing="0.05 1 1" radius="0.005" mass="0.05">
            <pin id="0 1"/>
            <elasticity young="1e8" section="circle"/>
            <contact contype="0" conaffinity="0" selfcollide="none"/>
          </flexcomp>
        </body>
      </worldbody>
    </mujoco>"""
    m_rod = mujoco.MjModel.from_xml_string(rod_xml)
    d_rod = mujoco.MjData(m_rod)
    mujoco.mj_step(m_rod, d_rod)
    self.assertEqual(d_rod.warning[mujoco.mjtWarning.mjWARN_INERTIA].number, 0)

    for elastic2d, iters in (('stretch', 1), ('both', 15)):
      cloth_xml = f"""<mujoco>
        <size memory="4M"/>
        <option timestep="0.01" integrator="discrete" iterations="{iters}"
                tolerance="1e-8"/>
        <worldbody>
          <flexcomp name="c" type="grid" dim="2" count="5 5 1"
                    spacing="0.05 0.05 0.05" radius="0.002" mass="0.1">
            <pin id="0 4"/>
            <elasticity young="1e7" thickness="0.002" elastic2d="{elastic2d}"/>
            <contact contype="0" conaffinity="0" selfcollide="none"/>
          </flexcomp>
        </worldbody>
      </mujoco>"""
      m_cloth = mujoco.MjModel.from_xml_string(cloth_xml)
      d_cloth = mujoco.MjData(m_cloth)
      mujoco.mj_step(m_cloth, d_cloth)
      self.assertEqual(
          d_cloth.warning[mujoco.mjtWarning.mjWARN_CNSTRFULL].number, 0
      )
      self.assertEqual(
          d_cloth.warning[mujoco.mjtWarning.mjWARN_INERTIA].number, 0
      )

    # Stiff 3D trilinear flex iterates in mj_effSolve for BOTH Newton and CG.
    for solver in ('Newton', 'CG'):
      solid_xml = f"""<mujoco>
        <option timestep="0.01" integrator="discrete" solver="{solver}"
                iterations="1" tolerance="1e-8"/>
        <worldbody>
          <flexcomp name="s" type="grid" dim="3" dof="trilinear" count="5 3 3"
                    cellcount="4 1 1" spacing="0.05 0.02 0.02" radius="0.001"
                    mass="0.05">
            <pin gridrange="0 0 0 0 1 1"/>
            <elasticity young="1e9" poisson="0.4"/>
            <contact contype="0" conaffinity="0" selfcollide="none"/>
          </flexcomp>
        </worldbody>
      </mujoco>"""
      m_solid = mujoco.MjModel.from_xml_string(solid_xml)
      d_solid = mujoco.MjData(m_solid)
      mujoco.mj_step(m_solid, d_solid)
      self.assertGreater(
          d_solid.warning[mujoco.mjtWarning.mjWARN_INERTIA].number, 0
      )

  def test_refsafe_clamps_equalities_under_euler_and_implicitfast_only(self):
    def edge_aref(integrator: str, timeconst: float) -> float:
      xml = f"""<mujoco>
        <option timestep="0.002" integrator="{integrator}"/>
        <worldbody>
          <flexcomp name="rod" type="grid" dim="1" count="2 1 1"
                    spacing="0.05 1 1" radius="0.002" mass="0.04">
            <pin id="0"/>
            <edge equality="true" solref="{timeconst} 1"/>
            <contact contype="0" conaffinity="0" selfcollide="none"/>
          </flexcomp>
        </worldbody>
      </mujoco>"""
      m = mujoco.MjModel.from_xml_string(xml)
      d = mujoco.MjData(m)
      d.qpos[0] = 1e-4  # Stretch the single edge by 0.1 mm at zero velocity.
      mujoco.mj_forward(m, d)
      return float(abs(d.efc_aref[0]))

    for integ in ('Euler', 'implicitfast'):
      # Below 2 * timestep = 0.004, refsafe clamps timeconst to 0.004.
      self.assertAlmostEqual(
          edge_aref(integ, 0.001), edge_aref(integ, 0.004), places=9
      )
    # Under discrete, equalities skip refsafe (4x smaller timeconst stiffens).
    self.assertGreater(
        edge_aref('discrete', 0.001) / edge_aref('discrete', 0.004), 3.5
    )

  def test_elastic3d_snh_is_mjspec_only_full_3d_discrete(self):
    bad_xml = """<mujoco>
      <option integrator="discrete"/>
      <worldbody>
        <flexcomp name="b" type="grid" dim="3" count="2 2 2"
                  spacing="0.02 0.02 0.02" radius="0.001">
          <elasticity young="1e4" elastic3d="1"/>
        </flexcomp>
      </worldbody>
    </mujoco>"""
    with self.assertRaisesRegex(ValueError, 'elastic3d'):
      mujoco.MjModel.from_xml_string(bad_xml)

    def make_snh(
        dim=3, dof='full', integrator=mujoco.mjtIntegrator.mjINT_DISCRETE
    ):
      spec = mujoco.MjSpec()
      spec.option.integrator = integrator
      holder = spec.worldbody.add_body(name='h')
      flex = holder.make_flex(
          name='b',
          type='grid',
          dim=dim,
          dof=dof,
          count=[3, 3, 1] if dim == 2 else [3, 3, 3],
          cellcount=[1, 1, 1],
          spacing=[0.02, 0.02, 0.02],
          radius=0.001,
      )
      flex.young = 5e4
      flex.poisson = 0.2
      if dim == 2:
        flex.thickness = 0.002
        flex.elastic2d = 3
      flex.elastic3d = 1
      return spec

    spec = make_snh()
    model = spec.compile()
    self.assertNotEqual(float(model.flex_stiffness[21]), 0.0)
    with tempfile.TemporaryDirectory() as tmpdir:
      xml_path = pathlib.Path(tmpdir) / 'model.xml'
      with self.assertRaisesRegex(
          (ValueError, mujoco.FatalError), 'cannot be written to MJCF'
      ):
        spec.to_file(str(xml_path))
      mjb_path = pathlib.Path(tmpdir) / 'model.mjb'
      mujoco.mj_saveModel(model, str(mjb_path), None)
      loaded = mujoco.MjModel.from_binary_path(str(mjb_path))
      self.assertNotEqual(float(loaded.flex_stiffness[21]), 0.0)

    with self.assertRaisesRegex(ValueError, 'non-interpolated 3d flex'):
      make_snh(dof='trilinear').compile()
    with self.assertRaisesRegex(ValueError, 'non-interpolated 3d flex'):
      make_snh(dim=2).compile()
    with self.assertRaisesRegex(ValueError, "integrator='discrete'"):
      make_snh(integrator=mujoco.mjtIntegrator.mjINT_EULER).compile()

  def test_weld_locks_orientation_and_exclude_ignores_flex_contacts(self):
    # Weld to a 3-slide flex vertex body locks world orientation; <exclude> does
    # not filter flex:geom contacts while contype/conaffinity does.
    for con_attr, expect_contacts in (
        ('contype="1" conaffinity="1"', True),
        ('contype="0" conaffinity="0"', False),
    ):
      xml = f"""<mujoco>
        <option timestep="0.002" integrator="discrete" solver="Newton"/>
        <worldbody>
          <flexcomp name="r" type="grid" dim="1" count="5 1 1"
                    spacing="0.05 1 1" pos="0.1 0 0.2" radius="0.005"
                    mass="0.05">
            <pin id="0 1"/>
            <edge equality="true" solref="0.002 1"/>
            <elasticity young="2e6" section="circle" damping="0.01"/>
            <contact selfcollide="none"/>
          </flexcomp>
          <body name="tip" pos="0.2 0 0.2">
            <freejoint/>
            <geom type="sphere" size="0.02" mass="0.05" {con_attr}/>
          </body>
        </worldbody>
        <contact><exclude body1="r_4" body2="tip"/></contact>
        <equality><weld body1="r_4" body2="tip" solref="0.002 1"/></equality>
      </mujoco>"""
      m = mujoco.MjModel.from_xml_string(xml)
      d = mujoco.MjData(m)
      mujoco.mj_step(m, d, 20)
      if expect_contacts:
        self.assertGreater(d.ncon, 0)
      else:
        self.assertEqual(d.ncon, 0)
        # Welded tip keeps identity world quaternion despite beam bending.
        np.testing.assert_allclose(
            d.body('tip').xquat, [1, 0, 0, 0], atol=1e-3
        )

  def test_closed_surface_mesh_embeds_as_cut_cell_volume_at_dim2(self):
    # Closed genus-1 square torus surface mesh (16 vertices, 32 triangles).
    torus_obj = '\n'.join([
        'v -0.08 -0.08 -0.04', 'v  0.08 -0.08 -0.04',
        'v  0.08  0.08 -0.04', 'v -0.08  0.08 -0.04',
        'v -0.08 -0.08  0.04', 'v  0.08 -0.08  0.04',
        'v  0.08  0.08  0.04', 'v -0.08  0.08  0.04',
        'v -0.025 -0.025 -0.04', 'v  0.025 -0.025 -0.04',
        'v  0.025  0.025 -0.04', 'v -0.025  0.025 -0.04',
        'v -0.025 -0.025  0.04', 'v  0.025 -0.025  0.04',
        'v  0.025  0.025  0.04', 'v -0.025  0.025  0.04',
        'f 1 2 6', 'f 1 6 5', 'f 2 3 7', 'f 2 7 6',
        'f 3 4 8', 'f 3 8 7', 'f 4 1 5', 'f 4 5 8',
        'f 10 9 13', 'f 10 13 14', 'f 11 10 14', 'f 11 14 15',
        'f 12 11 15', 'f 12 15 16', 'f 9 12 16', 'f 9 16 13',
        'f 5 6 14', 'f 5 14 13', 'f 6 7 15', 'f 6 15 14',
        'f 7 8 16', 'f 7 16 15', 'f 8 5 13', 'f 8 13 16',
        'f 2 1 9', 'f 2 9 10', 'f 3 2 10', 'f 3 10 11',
        'f 4 3 11', 'f 4 11 12', 'f 1 4 12', 'f 1 12 9',
    ])
    with tempfile.TemporaryDirectory() as tmpdir:
      obj_path = pathlib.Path(tmpdir) / 'torus.obj'
      obj_path.write_text(torus_obj, encoding='utf-8')
      # Omitting origin at dim=3 fails; adding origin=[0,0,0] plugs the hole.
      with self.assertRaisesRegex(ValueError, 'origin must be specified'):
        bad = mujoco.MjSpec()
        bad.option.integrator = mujoco.mjtIntegrator.mjINT_DISCRETE
        bad.worldbody.add_body(name='h').make_flex(
            name='ring', type='mesh', dim=3, dof='trilinear',
            cellcount=[2, 2, 1], file=str(obj_path), radius=0.002
        )
      plugged = mujoco.MjSpec()
      plugged.option.integrator = mujoco.mjtIntegrator.mjINT_DISCRETE
      fl_p = plugged.worldbody.add_body(name='h').make_flex(
          name='ring', type='mesh', dim=3, dof='trilinear',
          cellcount=[2, 2, 1], file=str(obj_path), origin=[0, 0, 0],
          radius=0.002
      )
      fl_p.young = 6e4
      m_plugged = plugged.compile()
      self.assertEqual(int(m_plugged.flex_vertnum[0]), 17)

      # dim=2 + elastic2d='none' embeds surface triangles as cut-cell volume.
      spec = mujoco.MjSpec()
      spec.option.integrator = mujoco.mjtIntegrator.mjINT_DISCRETE
      spec.worldbody.add_geom(
          name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.1]
      )
      holder = spec.worldbody.add_body(name='h', pos=[0, 0, 0.045])
      fl = holder.make_flex(
          name='ring', type='mesh', dim=2, dof='trilinear',
          cellcount=[2, 2, 1], file=str(obj_path), radius=0.002, mass=0.25
      )
      fl.young, fl.poisson, fl.damping = 6e4, 0.3, 0.015
      ball = spec.worldbody.add_body(name='ball', pos=[0, 0, 0.11])
      ball.add_joint(
          name='bz', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1]
      )
      ball.add_geom(
          type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.045, 0, 0], mass=0.3
      )
      m = spec.compile()
      self.assertEqual(int(m.flex_dim[0]), 2)
      self.assertEqual(int(m.flex_interp[0]), 1)
      d = mujoco.MjData(m)
      mujoco.mj_step(m, d, 200)
      self.assertEqual(d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number, 0)
      self.assertGreater(float(d.body('ball').xpos[2]), 0.06)

  def test_newton_converges_gradient_in_contact_where_cg_exits_early(self):
    tol = 5e-5 if mujoco.MJTNUM_DTYPE == np.float32 else 1e-6

    def conv_fraction(solver: str) -> float:
      xml = f"""<mujoco>
        <option timestep="0.002" integrator="discrete" solver="{solver}"
                tolerance="1e-6" iterations="200"/>
        <worldbody>
          <body name="base">
            <flexcomp name="pad" type="grid" dim="3" dof="trilinear"
                      count="10 10 6" cellcount="3 3 2"
                      spacing="0.01333333 0.01333333 0.016" pos="0 0 0.04"
                      radius="0.0015" mass="0.20">
              <pin gridrange="0 0 0 3 3 0"/>
              <elasticity young="6e4" poisson="0.25" damping="0.015"/>
              <contact selfcollide="none" solref="0.004 1"/>
            </flexcomp>
          </body>
          <body name="platen" pos="0 0 0.097">
            <joint name="pz" type="slide" axis="0 0 1" damping="5"/>
            <geom type="cylinder" size="0.045 0.015" mass="1.8"
                  solref="0.004 1"/>
          </body>
        </worldbody>
      </mujoco>"""
      m = mujoco.MjModel.from_xml_string(xml)
      d = mujoco.MjData(m)
      grads = []
      for _ in range(250):
        mujoco.mj_step(m, d)
        if d.nefc > 0 and d.solver_niter[0] > 0:
          grads.append(float(d.solver[d.solver_niter[0] - 1].gradient))
      self.assertGreater(len(grads), 150)
      platen_bottom = float(d.xipos[m.body('platen').id, 2]) - 0.015
      self.assertBetween(platen_bottom, 0.055, 0.081)
      return float(np.mean(np.array(grads) < tol))

    self.assertGreaterEqual(conv_fraction('Newton'), 0.95)
    self.assertLess(conv_fraction('CG'), 0.10)

  def test_staggered_plate_beats_grid_bias_within_5pct_kirchhoff(self):
    if mujoco.MJTNUM_DTYPE == np.float32:
      self.skipTest('Sub-millimeter Kirchhoff plate test requires float64.')
    a, t, e_mod, nu, q = 0.24, 0.003, 1.8e8, 0.30, 5.0
    d_rig = e_mod * t**3 / (12.0 * (1.0 - nu**2))
    w_ref = 0.00406235 * q * a**4 / d_rig

    def plate_err(geom_attrs: str, pin_attr: str) -> float:
      xml = f"""<mujoco>
        <option timestep="0.002" integrator="discrete" solver="Newton"
                tolerance="1e-10" gravity="0 0 0"/>
        <worldbody>
          <body name="frame" pos="0 0 0.2">
            <flexcomp name="p" dim="2" {geom_attrs} radius="0.001" mass="0.2">
              {pin_attr}
              <elasticity young="{e_mod}" poisson="{nu}" thickness="{t}"
                          elastic2d="both" damping="0.005"/>
              <contact contype="0" conaffinity="0" selfcollide="none"/>
            </flexcomp>
          </body>
        </worldbody>
      </mujoco>"""
      m = mujoco.MjModel.from_xml_string(xml)
      d = mujoco.MjData(m)
      mujoco.mj_forward(m, d)
      v0 = d.flexvert_xpos.copy()
      dadr = int(m.flex_elemdataadr[0])
      elems = m.flex_elem[dadr : dadr + 3 * int(m.flex_elemnum[0])].reshape(
          -1, 3
      )
      nodal_fz = np.zeros(m.nflexvert)
      for tri in elems:
        p0, p1, p2 = v0[tri[0]], v0[tri[1]], v0[tri[2]]
        area = 0.5 * float(np.linalg.norm(np.cross(p1 - p0, p2 - p0)))
        for vid in tri:
          nodal_fz[vid] -= q * area / 3.0
      for vid, fz in enumerate(nodal_fz):
        bid = int(m.flex_vertbodyid[vid])
        if m.body_weldid[bid] != 0:
          d.xfrc_applied[bid, 2] += fz
      center = int(np.argmin(v0[:, 0] ** 2 + v0[:, 1] ** 2))
      mujoco.mj_step(m, d, 1000)
      w = float(v0[center, 2] - d.flexvert_xpos[center, 2])
      return w / w_ref - 1.0

    sp = a / 16.0
    grid_err = plate_err(
        f'type="grid" count="17 17 1" spacing="{sp} {sp} {sp}"',
        '<pin gridrange="0 0 16 0 0 16 16 16 0 0 0 16 16 0 16 16"/>',
    )
    self.assertLess(grid_err, -0.35)

    py_blocks = _fenced_blocks('python')
    stagger_code = next(c for c in py_blocks if 'point_attr' in c)
    ns = {'np': np, 'a': a, 'nx': 25, 'ny': 29}
    exec(stagger_code, ns)  # pylint: disable=exec-used
    pts = np.fromstring(ns['point_attr'], sep=' ').reshape(-1, 3)
    edge = np.flatnonzero(np.max(np.abs(pts[:, :2]), axis=1) > a / 2 - 1e-6)
    pin_ids = ' '.join(map(str, edge))
    p_attr, e_attr = ns['point_attr'], ns['elem_attr']
    self.assertLessEqual(3 * (len(pts) - len(edge)), 2000)
    stag_err = plate_err(
        f'type="direct" point="{p_attr}" element="{e_attr}"',
        f'<pin id="{pin_ids}"/>',
    )
    self.assertLessEqual(abs(stag_err), 0.05)

  def test_rod_full_segment_clamp_offset_is_exact_in_torsion(self):
    length, radius, shear, torque, n = 0.45, 0.004, 1.0e6, 1.2e-3, 9
    ref = torque * length / (shear * np.pi * radius**4 / 2.0)

    def end_twist_err(full_offset: bool) -> float:
      seg = length / (n - 2) if full_offset else length / (n - 1)
      pos_x = (length - seg) / 2.0 if full_offset else length / 2.0
      xml = f"""<mujoco>
        <option timestep="0.002" integrator="discrete" solver="Newton"
                tolerance="1e-10" gravity="0 0 0"/>
        <worldbody>
          <body name="mount">
            <flexcomp name="rod" type="grid" dim="1" count="{n} 1 1"
                      spacing="{seg:.12g} 1 1" pos="{pos_x:.12g} 0 0"
                      radius="{radius}" mass="0.025">
              <pin id="0 1"/>
              <edge equality="true" solref="0.002 1"/>
              <elasticity young="3e6" shear="{shear}" damping="0.02"
                          section="circle"/>
              <contact contype="0" conaffinity="0" selfcollide="none"/>
            </flexcomp>
          </body>
        </worldbody>
        <actuator><motor name="t" joint="rod_twist" gear="1"/></actuator>
      </mujoco>"""
      m = mujoco.MjModel.from_xml_string(xml)
      self.assertEqual(m.nv, 28)
      d = mujoco.MjData(m)
      d.ctrl[0] = torque
      mujoco.mj_step(m, d, 600)
      return float(d.joint('rod_twist').qpos[0]) / ref - 1.0

    self.assertLess(abs(end_twist_err(True)), 1e-3)
    self.assertAlmostEqual(end_twist_err(False), -1.0 / (n - 1), delta=2e-3)


if __name__ == '__main__':
  absltest.main()
