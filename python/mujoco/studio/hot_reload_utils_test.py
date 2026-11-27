# Copyright 2026 DeepMind Technologies Limited
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
"""Tests for Studio hot-reload default comparisons."""

import dataclasses
import importlib
import pathlib
import sys
import tempfile
from unittest import mock

from absl.testing import absltest
from absl.testing import parameterized

from . import hot_reload_utils


class SameDefaultTest(parameterized.TestCase):

  @parameterized.product(slots=[False, True], value=[1, 2])
  def test_dataclass_defaults_after_module_reload(self, slots, value):
    with tempfile.TemporaryDirectory() as directory:
      path = pathlib.Path(directory) / "_hot_reload_default_fixture.py"
      source = (
          "import dataclasses\n"
          f"@dataclasses.dataclass(slots={slots})\n"
          "class Options:\n"
          "  value: int = {}\n"
      )
      path.write_text(source.format(1))
      with mock.patch.object(sys, "path", [directory] + sys.path):
        with mock.patch.dict(sys.modules):
          module = importlib.import_module(path.stem)
          before = module.Options()
          path.write_text(source.format(value))
          hot_reload_utils.reload_module(module)
          after = module.Options()
          self.assertIsNot(type(before), type(after))
          self.assertEqual(
              hot_reload_utils.same_default(before, after), value == 1
          )

  def test_slotted_fields_with_an_inherited_instance_dictionary(self):
    class Base:
      pass

    before_cls = dataclasses.make_dataclass(
        "Options", [("value", int)], bases=(Base,), slots=True
    )
    after_cls = dataclasses.make_dataclass(
        "Options", [("value", int)], bases=(Base,), slots=True
    )
    self.assertTrue(hot_reload_utils.same_default(before_cls(1), after_cls(1)))
    self.assertFalse(hot_reload_utils.same_default(before_cls(1), after_cls(2)))

  def test_preserves_dynamic_attributes_on_ordinary_dataclasses(self):
    before_cls = dataclasses.make_dataclass("Options", [("value", int)])
    after_cls = dataclasses.make_dataclass("Options", [("value", int)])
    before, after = before_cls(1), after_cls(1)
    before.extra, after.extra = 1, 2
    self.assertFalse(hot_reload_utils.same_default(before, after))

  def test_uninitialized_slotted_fields(self):
    fields = [("value", int), ("later", int, dataclasses.field(init=False))]
    before_cls = dataclasses.make_dataclass("Options", fields, slots=True)
    after_cls = dataclasses.make_dataclass("Options", fields, slots=True)
    before, after = before_cls(1), after_cls(1)
    self.assertTrue(hot_reload_utils.same_default(before, after))
    after.later = 2
    self.assertFalse(hot_reload_utils.same_default(before, after))

  @parameterized.parameters(
      (1, 1, True),
      (1, 2, False),
      ([1, {"value": 2}], [1, {"value": 2}], True),
      ([1, {"value": 2}], [1, {"value": 3}], False),
  )
  def test_existing_defaults(self, before, after, expected):
    self.assertEqual(hot_reload_utils.same_default(before, after), expected)


if __name__ == "__main__":
  absltest.main()
