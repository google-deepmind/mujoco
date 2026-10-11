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
"""HTML report builder using Jinja2 templates."""

# report/builder.py
import os
from typing import Any

import jinja2
from mujoco.sysid.report.sections.base import ReportSection

# Path to the templates directory
TEMPLATE_DIR = os.path.join(os.path.dirname(__file__), "templates")


class ReportBuilder:
  """Assembles report sections into a complete HTML report."""

  def __init__(self, title: str, global_context: dict[str, Any] | None = None):
    self._title = title
    self._sections: list[ReportSection] = []
    self._global_context = global_context or {}

    # Setup Jinja to load from files
    self._env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(TEMPLATE_DIR),
        autoescape=jinja2.select_autoescape(["html", "xml"]),
    )

  def add_section(self, section: ReportSection):
    self._sections.append(section)

  def build(self) -> str:
    """Render all sections into a complete HTML report string."""
    # Load the main layout
    layout_template = self._env.get_template("layout.html")

    # Render each section individually
    rendered_sections = []
    all_header_includes = set()

    for section in self._sections:
      # Get the specific template for this section
      sec_template = self._env.get_template(section.template_filename)

      # Check if this is a GroupSection (has 'sections' attribute)
      extra_context = {}
      child_sections: list[ReportSection] = getattr(section, "sections", [])
      if child_sections:
        child_sections_content = []
        for child in child_sections:
          child_template = self._env.get_template(child.template_filename)
          child_html = child_template.render(child.get_context())
          all_header_includes.update(child.header_includes())

          # Fallback anchor for child
          child_anchor = child.anchor
          if not child_anchor:
            child_anchor = (
                child.title.lower()
                .replace(" ", "-")
                .replace("[", "")
                .replace("]", "")
                .replace("(", "")
                .replace(")", "")
            )

          child_sections_content.append({
              "title": child.title,
              "content": child_html,
              "anchor": child_anchor,
          })
        extra_context["child_sections"] = child_sections_content

      # Render the section HTML (main wrapper)
      html_content = sec_template.render(section.get_context() | extra_context)
      # Collect header requirements (scripts/css)
      all_header_includes.update(section.header_includes())

      # Fallback anchor generation
      anchor = section.anchor
      if not anchor:
        anchor = (
            section.title.lower()
            .replace(" ", "-")
            .replace("[", "")
            .replace("]", "")
            .replace("(", "")
            .replace(")", "")
        )

      rendered_sections.append({
          "title": section.title,
          "anchor": anchor,
          "collapsible": section.collapsible,
          "is_open": section.is_open,
          "content": html_content,
      })

    # Render final report
    return layout_template.render(
        report_title=self._title,
        sections=rendered_sections,
        header_includes=all_header_includes,
        **self._global_context,
    )

  def save(self, path: str):
    with open(path, "w", encoding="utf-8") as f:
      f.write(self.build())
