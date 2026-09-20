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

"""Row layout report section."""

from typing import Any

from mujoco.sysid.report.sections.base import ReportSection


class RowSection(ReportSection):
  """A report section that displays multiple other sections side-by-side."""

  def __init__(
      self,
      title: str,
      sections: list[ReportSection],
      anchor: str = "",
      collapsible: bool = True,
      description: str = "",
  ):
    super().__init__(collapsible=collapsible)
    self._title = title
    self.sections = sections
    self._anchor = anchor
    self._description = description

  @property
  def title(self) -> str:
    return self._title

  @property
  def anchor(self) -> str:
    return self._anchor

  @property
  def template_filename(self) -> str:
    return "row.html"

  def header_includes(self) -> set[str]:
    includes = set()
    for section in self.sections:
      includes.update(section.header_includes())
    return includes

  def get_context(self) -> dict[str, Any]:
    return {
        "title": self.title,
        "anchor": self.anchor,
        "description": self._description,
    }
