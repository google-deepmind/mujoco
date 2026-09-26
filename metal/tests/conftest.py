# Copyright 2026 keeeeenw
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

"""GPU qualification is opt-in; ordinary collection cannot launch a shader."""

import pytest


def pytest_addoption(parser):
  parser.addoption(
      "--run-metal",
      action="store_true",
      default=False,
      help="Run GPU qualification on an idle Apple Silicon machine",
  )


def pytest_collection_modifyitems(config, items):
  for item in items:
    name = item.originalname or item.name
    cpu = (
        item.path.name == "test_api.py"
        or name.startswith("test_harness_")
        or name
        in {
            "test_exact_cpu_constraint_parameter_equations",
            "test_pinned_mujoco_contact_manifold_python_parity",
        }
    )
    if not cpu:
      item.add_marker(pytest.mark.gpu)
      if not config.getoption("--run-metal"):
        item.add_marker(
            pytest.mark.skip(reason="requires --run-metal on idle Apple GPU")
        )


@pytest.fixture(autouse=True)
def forbid_gpu_in_cpu_tests(request, monkeypatch):
  if request.node.get_closest_marker("gpu") is None:
    import torch

    def forbidden(*args, **kwargs):
      raise AssertionError("CPU test attempted to compile or synchronize Metal")

    monkeypatch.setattr(torch.mps, "compile_shader", forbidden)
    monkeypatch.setattr(torch.mps, "synchronize", forbidden)
