# Copyright 2022 DeepMind Technologies Limited
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
"""Tests for MuJoCo Python function bindings."""

from absl.testing import absltest
from absl.testing import parameterized

from introspect import functions

import mujoco

# Omitted UI framework functions (no support yet)
UI_FUNCTIONS = [
    'mjui_add', 'mjui_addToSection', 'mjui_event', 'mjui_render', 'mjui_resize',
    'mjui_themeColor', 'mjui_themeSpacing', 'mjui_update'
]

# Omitted error and alloc/free memory functions (not needed in Python)
MEMORY_FUNCTIONS = [
    'mju_strncpy', 'mju_clearHandlers', 'mju_warning', 'mju_warning_i',
    'mju_warning_s', 'mju_malloc', 'mju_free', 'mju_error', 'mju_error_i',
    'mju_error_s', 'mju_boxQPmalloc', 'mj_warning', 'mj_stackAlloc',
    'mj_makeData', 'mj_copyData', 'mj_deleteData', 'mj_loadModel',
    'mj_copyModel', 'mj_deleteModel'
]

# Omitted irrelevant renderer context functions
CONTEXT_FUNCTIONS = ['mjr_makeContext', 'mjr_freeContext', 'mjr_defaultContext']

# Omitted irrelevant visual functions
VISUAL_FUNCTIONS = [
    'mjv_defaultScene', 'mjv_makeScene', 'mjv_freeScene', 'mjv_averageCamera'
]

# Omitted XML and Virtual Filesystem (VFS) functions
VFS_FUNCTIONS = [
    'mj_loadXML', 'mj_makeEmptyFileVFS', 'mj_freeLastXML', 'mj_findFileVFS',
    'mj_deleteVFS', 'mj_defaultVFS', 'mj_addFileVFS', 'mj_deleteFileVFS'
]

# Omitted plugin functions
PLUGIN_FUNCTIONS = [
    'mj_getPluginConfig', 'mjp_getPlugin', 'mjp_getPluginAtSlot',
    'mjp_pluginCount', 'mjp_registerPlugin', 'mjp_defaultPlugin'
]

# All omitted functions from the bindings
OMITTED_LIST = [
    *UI_FUNCTIONS, *MEMORY_FUNCTIONS, *CONTEXT_FUNCTIONS, *VISUAL_FUNCTIONS,
    *VFS_FUNCTIONS, *PLUGIN_FUNCTIONS
]

class MuJoCoFunctionsTest(parameterized.TestCase):
  def test_for_missing_function_bindings(self):
    # All functions found by introspect
    possible_funcs = functions.FUNCTIONS.keys()

    # All functions accounted for (have bindings or otherwise omitted)
    accounted_funcs = set(OMITTED_LIST + dir(mujoco))

    # List of possibly missing functions
    missing_funcs = [f for f in possible_funcs if f not in accounted_funcs]

    # Verify that there are no missing functions in the Python bindings
    self.assertEmpty(missing_funcs)

if __name__ == '__main__':
  absltest.main()
