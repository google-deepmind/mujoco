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
"""Precompiled Emscripten browser client for the MuJoCo Studio Web Viewer."""

import os as _os

__version__ = '4.0.1'
DIST_DIR = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), 'dist')

del _os
