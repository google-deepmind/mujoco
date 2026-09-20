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

"""Utility functions for report rendering."""

import math

from plotly import offline as plt_offline


def plotly_script_tag() -> str:
  """Returns an HTML <script> tag for Plotly.

  currently installed in the environment.
  """
  plotly_js_version = plt_offline.get_plotlyjs_version()
  return (
      "<script"
      f' src="https://cdn.plot.ly/plotly-{plotly_js_version}.min.js"></script>'
  )


def get_text_color(bg_color_hex: str) -> str:
  """Returns 'black' or 'white' for text over a background color.

  luminance of the given background hex color.
  Useful for heatmaps and colored data tables.
  """
  # Convert hex to RGB
  hex_color = bg_color_hex.lstrip("#")
  if len(hex_color) != 6:
    return "black"  # Fallback

  rgb = tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4))
  r, g, b = [x / 255.0 for x in rgb]

  # Calculate luminance (per WCAG guidelines)
  # https://www.w3.org/TR/WCAG20/#relativeluminancedef
  def lum_component(c):
    return c / 12.92 if c <= 0.03928 else math.pow((c + 0.055) / 1.055, 2.4)

  luminance = (
      0.2126 * lum_component(r)
      + 0.7152 * lum_component(g)
      + 0.0722 * lum_component(b)
  )

  # Return 'black' for light backgrounds, 'white' for dark backgrounds
  return "black" if luminance > 0.4 else "white"
