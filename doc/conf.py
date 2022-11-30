# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Configuration file for the Sphinx documentation builder."""

import os
import sys

# -- Path setup --------------------------------------------------------------
#
# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.

sys.path.insert(0, os.path.abspath('../'))
sys.path.append(os.path.abspath('ext'))

import sphinxcontrib.katex as katex  # pylint: disable=g-import-not-at-top
import sphinxcontrib.youtube as youtube  # pylint: disable=g-import-not-at-top

# -- Project information -----------------------------------------------------

project = 'MuJoCo'
copyright = 'DeepMind Technologies Limited'  # pylint: disable=redefined-builtin
author = 'DeepMind'

# -- General configuration ---------------------------------------------------

master_doc = 'index'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinxcontrib.katex',
    'sphinxcontrib.youtube',
    'sphinx_reredirects',
    'sphinx_toolbox.collapse',
    "mujoco_include",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'includes/*']

redirects = {
    # index.rst just contains the table of contents definition.
    'index': 'overview.html',
}

rst_prolog = """
.. include:: includes/macros.rst
.. include:: includes/roles.rst
.. include:: <isonum.txt>
"""

# -- Options for autodoc -----------------------------------------------------

autodoc_default_options = {
    'member-order': 'bysource',
    'special-members': True,
    'exclude-members': '__repr__, __str__, __weakref__',
}

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'furo'
html_title = 'MuJoCo Documentation'
html_logo = 'images/banner.svg'

SHARED_CSS_VARIABLES = {
    "admonition-font-size": "1rem",
    "admonition-title-font-size": "1rem",
    "sidebar-item-font-size": "115%",
}

html_theme_options = {
    "light_css_variables": {
        "at-color": "#bc103e",
        "at-val-color": "#bc103e",
        "body-color": "#14234b",
        "color-highlight-on-target": "#e5e8ed",
        "primary-header-color": "#0053d6",
        "row-odd-background-color": "#f0f3f7",
        "rst-content-a-color": "#2980b9",
        "secondary-header-color": "#123693",
        "wy-menu-vertical-background-color": "#0053d6",
        "wy-menu-vertical-color": "white",
        "wy-nav-side-background-color": "#0053d6",
    },
    "dark_css_variables": {
        "at-color": "#ff95a6",
        "at-val-color": "#ff95a6",
        "body-color": "#14234b",
        "color-admonition-background": "#1e1e21",
        "color-highlight-on-target": "#3d4045",
        "primary-header-color": "#a8caff",
        "row-odd-background-color": "#222326",
        "rst-content-a-color": "#2980b9",
        "secondary-header-color": "#458dff",
        "wy-menu-vertical-background-color": "#0053d6",
        "wy-menu-vertical-color": "white",
        "wy-nav-side-background-color": "#0053d6",
    },
}

for v in html_theme_options.values():
  v.update(SHARED_CSS_VARIABLES)

pygments_style = "default"
pygments_dark_style = "monokai"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = [
    '_static',
    'css',
    'favicons',
]
html_css_files = [
    'theme_overrides.css',
]

html_favicon = 'favicons/favicon-32x32.png'

# -- Options for katex ------------------------------------------------------

# See: https://sphinxcontrib-katex.readthedocs.io/en/0.4.1/macros.html
latex_macros = r"""
    \def \d              #1{\operatorname{#1}}
"""

# Translate LaTeX macros to KaTeX and add to options for HTML builder
katex_macros = katex.latex_defs_to_katex_macros(latex_macros)
katex_options = 'macros: {' + katex_macros + '}'

# Add LaTeX macros for LATEX builder
latex_elements = {'preamble': latex_macros}
