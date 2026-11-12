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
"""Compatibility shim re-exporting _render_filament from mujoco.studio._studio."""

import importlib as _importlib
import importlib.machinery as _machinery
import sys as _sys

# pylint: disable-next=protected-access
_rf = _importlib.import_module('mujoco.studio._studio')._render_filament
_rf.__spec__ = _machinery.ModuleSpec(__name__, None)
if 'mujoco' in _sys.modules:
  setattr(_sys.modules['mujoco'], '_render_filament', _rf)
_sys.modules[__name__] = _rf
