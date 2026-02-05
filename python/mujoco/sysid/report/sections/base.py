import abc
from collections.abc import Iterable
from typing import Any


class ReportSection(abc.ABC):
  """Abstract base class for all report sections."""

  @property
  @abc.abstractmethod
  def template_filename(self) -> str:
    """The filename of the Jinja2 template (e.g., 'parameters.html')."""
    pass

  @abc.abstractmethod
  def get_context(self) -> dict[str, Any]:
    """Returns data needed by the template."""
    pass

  def __init__(self, collapsible: bool = True, is_open: bool = True):
    self._collapsible = collapsible
    self._is_open = is_open
    self._anchor = ""

  @property
  def title(self) -> str:
    return ""

  @property
  def anchor(self) -> str:
    """Returns a unique HTML anchor string."""
    # Auto-generate a safe anchor from title if not provided
    if not hasattr(self, "_anchor") or not self._anchor:
      return self.title.lower().replace(" ", "-")
    return self._anchor

  @property
  def collapsible(self) -> bool:
    return self._collapsible

  @property
  def is_open(self) -> bool:
    return self._is_open

  def header_includes(self) -> Iterable[str]:
    """Returns strings (like <script> tags) to add to <head>."""
    return set()
