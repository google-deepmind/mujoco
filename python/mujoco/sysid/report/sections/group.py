from typing import Any

from mujoco.sysid.report.sections.base import ReportSection


class GroupSection(ReportSection):
  """A report section that groups multiple other sections together."""

  def __init__(
    self,
    title: str,
    sections: list[ReportSection],
    anchor: str = "",
    collapsible: bool = True,
  ):
    super().__init__(collapsible=collapsible)
    self._title = title
    self.sections = sections
    self._anchor = anchor

  @property
  def title(self) -> str:
    return self._title

  @property
  def anchor(self) -> str:
    return self._anchor

  @property
  def template_filename(self) -> str:
    return "group.html"

  def header_includes(self) -> set[str]:
    includes = set()
    for section in self.sections:
      includes.update(section.header_includes())
    return includes

  def get_context(self) -> dict[str, Any]:
    return {
      "title": self.title,
      "anchor": self.anchor,
    }
