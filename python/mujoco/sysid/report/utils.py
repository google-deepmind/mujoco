import math

from plotly import offline as plt_offline


def plotly_script_tag() -> str:
  """
  Returns an HTML <script> tag for importing the specific version of Plotly
  currently installed in the environment.
  """
  plotly_js_version = plt_offline.get_plotlyjs_version()
  return (
    f'<script src="https://cdn.plot.ly/plotly-{plotly_js_version}.min.js"></script>'
  )


def get_text_color(bg_color_hex: str) -> str:
  """
  Determines whether text should be 'black' or 'white' based on the
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
    0.2126 * lum_component(r) + 0.7152 * lum_component(g) + 0.0722 * lum_component(b)
  )

  # Return 'black' for light backgrounds, 'white' for dark backgrounds
  return "black" if luminance > 0.4 else "white"
