# Copyright 2026 The MuJoCo Metal contributors
# Licensed under the Apache License, Version 2.0.
"""End-to-end checks for the source-tree pendulum demonstration."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np
import pytest

_PATH = Path(__file__).parents[1] / "examples" / "pendulum.py"
_SPEC = importlib.util.spec_from_file_location("pendulum_demo", _PATH)
_DEMO = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_DEMO)


def test_demo_force_profile_and_reset():
  sim = _DEMO.Comparison("cpu")
  m = sim.model
  assert (m.nq, m.nv, m.nu, m.ntendon, m.neq) == (4, 4, 0, 0, 0)
  assert m.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
  assert m.opt.integrator == mujoco.mjtIntegrator.mjINT_EULER
  for field in (
      m.dof_damping,
      m.dof_frictionloss,
      m.jnt_stiffness,
      m.body_gravcomp,
      m.jnt_limited,
  ):
    assert not np.any(field)
  assert m.opt.density == m.opt.viscosity == 0
  initial_qpos = sim.actual.qpos.copy()
  initial_qvel = sim.actual.qvel.copy()
  for _ in range(200):
    sim.step()
  assert not np.allclose(sim.actual.qpos, initial_qpos)
  assert sim.max_qpos_error == sim.max_qvel_error == 0
  sim.reset()
  for state in (sim.actual, sim.reference):
    np.testing.assert_array_equal(state.qpos, initial_qpos)
    np.testing.assert_array_equal(state.qvel, initial_qvel)
    assert state.time == 0


def test_cpu_mode_remains_explicit_and_native_mode_is_listed():
  sim = _DEMO.Comparison("cpu")
  report = sim.report()
  assert report["physics"] == "CPU MuJoCo"
  assert report["native_contact_free_stepping"] is False
  assert report["full_mujoco_metal_support"] is False
  help_text = subprocess.run(
      [sys.executable, str(_PATH), "--help"],
      check=True,
      capture_output=True,
      text=True,
  ).stdout
  assert "metal-hybrid" in help_text
  assert "metal" in help_text
  assert "cpu" in help_text


def test_mac_display_guard_only_applies_to_interactive_viewer():
  parser = _DEMO.argparse.ArgumentParser()
  with pytest.raises(SystemExit):
    _DEMO._check_interactive_display(
        parser, headless=False, system="darwin", display_count=0
    )
  _DEMO._check_interactive_display(
      parser, headless=True, system="darwin", display_count=0
  )
  _DEMO._check_interactive_display(
      parser, headless=False, system="linux", display_count=0
  )


@pytest.mark.gpu
@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit opt-in to GPU checks",
)
def test_metal_demo_short_rollout_and_reset():
  sim = _DEMO.Comparison()
  for _ in range(200):
    sim.step()
  assert sim.max_qpos_error < 1e-3
  assert sim.max_qvel_error < 1e-2
  end = sim.actual.qpos.copy()
  sim.reset()
  for _ in range(200):
    sim.step()
  np.testing.assert_allclose(sim.actual.qpos, end, atol=1e-6, rtol=1e-5)
  assert sim.max_qpos_error < 1e-3
  assert sim.max_qvel_error < 1e-2


@pytest.mark.gpu
@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit opt-in to GPU checks",
)
def test_native_metal_demo_short_rollout_and_reset():
  sim = _DEMO.Comparison("metal")
  assert sim.native is not None
  for _ in range(200):
    sim.step()
  assert sim.max_qpos_error < 1e-3
  assert sim.max_qvel_error < 1e-2
  end = sim.native.state.snapshot()
  sim.reset()
  for _ in range(200):
    sim.step()
  current = sim.native.state.snapshot()
  np.testing.assert_allclose(current.qpos, end.qpos, atol=1e-6, rtol=1e-5)
  np.testing.assert_allclose(current.qvel, end.qvel, atol=1e-6, rtol=1e-5)
  assert sim.max_qpos_error < 1e-3
  assert sim.max_qvel_error < 1e-2
