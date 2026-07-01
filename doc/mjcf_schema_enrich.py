#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
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

"""Enrich an auto-generated MJCF XSD with docs and defaults mined from RST.

The C++ generator (sample/xmlschema.cc) emits type info. This script mines
doc/XMLreference.rst for human-facing `:at:` blocks and patches each
<xs:attribute> with:
  * `default="..."` when the RST :at-val: contains a quoted default
  * <xs:annotation><xs:documentation>...</xs:documentation></xs:annotation>

Typical usage (release workflow):
    ./build/bin/xmlschema /tmp/raw.xsd
    ./doc/mjcf_schema_enrich.py \
        --in /tmp/raw.xsd \
        --rst doc/XMLreference.rst \
        --out doc/mjcf.xsd

The shebang invokes `uv run --script`, which provisions a matching
Python interpreter from the PEP 723 metadata block above.

The script fails soft: attrs with no matching RST block keep their
auto-generated type but lose the doc annotation. A --report flag prints
the drift list.
"""

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass
class DocEntry:
    """One parsed `:at:` block from the RST."""
    element: str              # element the attr belongs to (last anchor segment -1)
    attr: str                 # attribute name
    type_str: str = ""        # RST type annotation, e.g. "real(3)", "[a,b], \"a\""
    default: Optional[str] = None
    description: str = ""


# ---------------------------------------------------------------------------
# RST parser
# ---------------------------------------------------------------------------

# Example matched lines:
#   .. _option-timestep:
#   .. _body-joint-solreflimit:
ANCHOR_RE = re.compile(r"^\.\. _([a-zA-Z0-9_-]+):\s*$")

# Example matched lines:
#   :at:`angle`: :at-val:`[radian, degree], "degree"`
#   :at:`o_solref`, :at:`o_solimp`, :at:`o_friction`
#   :at:`gridlayout`: :at-val:`string, "............"`
#
# The line starts with :at: and contains one or more backtick-wrapped attr
# names, optionally followed by a :at-val:`...` block.
AT_LINE_RE = re.compile(r"^:at:`")
AT_NAME_RE = re.compile(r":at:`([^`]+)`")
AT_VAL_RE = re.compile(r":at-val:`([^`]*)`")

# Inside :at-val:`TYPE, "DEFAULT"` or :at-val:`TYPE, required` or
# :at-val:`TYPE, optional`. TYPE may contain a comma, e.g.
# "[rgb, depth, normal], \"rgb\"" -- so we split on the LAST top-level comma.
# Easier: try to match the quoted default first; fall back to a status word.
DEFAULT_RE = re.compile(r'"([^"]*)"\s*$')
STATUS_RE = re.compile(r"(required|optional)\s*$", re.IGNORECASE)


def _split_type_and_default(at_val: str) -> tuple[str, Optional[str]]:
    """Split an :at-val: body into (type_str, default_or_None).

    at_val looks like:
      'real, "0.002"'            -> ('real', '0.002')
      '[Euler, RK4, implicit, implicitfast], "Euler"' -> (list, 'Euler')
      'int, required'            -> ('int', None)
      'string, optional'         -> ('string', None)
    """
    s = at_val.strip()
    m = DEFAULT_RE.search(s)
    if m:
        default = m.group(1)
        type_part = s[: m.start()].rstrip(", \t")
        return type_part, default
    m = STATUS_RE.search(s)
    if m:
        type_part = s[: m.start()].rstrip(", \t")
        return type_part, None
    return s, None


def _parse_at_line(at_line: str) -> list[tuple[str, str, Optional[str]]]:
    """Return a list of (attr_name, type_str, default_or_None) from an :at: line.

    Grouped lines share the same (type, default). When no :at-val: is present,
    the type/default are empty/None.
    """
    names = AT_NAME_RE.findall(at_line)
    val_m = AT_VAL_RE.search(at_line)
    type_str, default = ("", None)
    if val_m:
        type_str, default = _split_type_and_default(val_m.group(1))
    return [(n, type_str, default) for n in names]


def _anchor_to_element_attr(anchor: str) -> tuple[str, str]:
    """Split a hyphenated anchor like 'body-joint-pos' into (element, attr).

    Rule: the attr is the LAST segment, the element is the second-to-last.
    For single-segment anchors like 'option' (section anchor), the element
    is the whole anchor and attr is empty -- caller should filter those.
    """
    parts = anchor.split("-")
    if len(parts) == 1:
        return parts[0], ""
    return parts[-2], parts[-1]


def parse_rst(rst_path: str) -> dict[tuple[str, str], DocEntry]:
    """Build a (element, attr) -> DocEntry dict from the RST."""
    with open(rst_path, encoding="utf-8") as f:
        lines = f.readlines()

    entries: dict[tuple[str, str], DocEntry] = {}
    pending_anchors: list[str] = []  # anchors accumulated since last :at: line
    i = 0
    n = len(lines)

    while i < n:
        line = lines[i]
        stripped = line.strip()

        m = ANCHOR_RE.match(line)
        if m:
            pending_anchors.append(m.group(1))
            i += 1
            continue

        if stripped.startswith(":at:`"):
            # Parse the :at: line (which may continue on next line for grouped
            # attrs, but mostly doesn't in practice).
            at_line = stripped
            parsed = _parse_at_line(at_line)

            # Skim the description: all lines indented by >=3 spaces until a
            # blank-line-then-unindented sentinel or another anchor/at block.
            desc_lines: list[str] = []
            j = i + 1
            while j < n:
                lj = lines[j]
                sj = lj.strip()
                if not sj:
                    # Blank line: peek ahead; if next non-blank is indented-
                    # continuation or blank, keep collecting.
                    k = j + 1
                    while k < n and not lines[k].strip():
                        k += 1
                    if k >= n:
                        break
                    lk = lines[k]
                    if (not lk.startswith((" ", "\t"))) or \
                       ANCHOR_RE.match(lk) or \
                       lk.strip().startswith(":at:`"):
                        break
                    desc_lines.append("")
                    j = k
                    continue
                if lj.startswith((" ", "\t")):
                    desc_lines.append(lj.rstrip())
                    j += 1
                else:
                    break

            description = "\n".join(desc_lines).rstrip()

            # Build DocEntry per attr. If grouped attrs share anchors, match
            # them by position; otherwise fall back to the last section anchor.
            # The common case is: N anchors immediately preceding, one per attr
            # in order.
            attr_anchors = [a for a in pending_anchors if "-" in a]
            if len(attr_anchors) == len(parsed):
                pairs = [(_anchor_to_element_attr(a), p)
                         for a, p in zip(attr_anchors, parsed)]
            else:
                # Fallback: use the last anchor's element prefix for all attrs.
                last = attr_anchors[-1] if attr_anchors else \
                       (pending_anchors[-1] if pending_anchors else "")
                element_prefix = (
                    _anchor_to_element_attr(last)[0]
                    if "-" in last else last
                )
                pairs = [((element_prefix, p[0]), p) for p in parsed]

            for (element, _anchor_attr), (attr_name, type_str, default) in pairs:
                key = (element, attr_name)
                # Only set if empty (first occurrence wins). Duplicate
                # element-attr anchors are rare.
                if key not in entries:
                    entries[key] = DocEntry(
                        element=element,
                        attr=attr_name,
                        type_str=type_str,
                        default=default,
                        description=description,
                    )

            pending_anchors.clear()
            i = j
            continue

        # Any non-anchor, non-at line flushes the pending anchor buffer
        # unless it's blank.
        if stripped:
            pending_anchors.clear()
        i += 1

    return entries


# ---------------------------------------------------------------------------
# Sphinx markup cleanup
# ---------------------------------------------------------------------------

# Handle the common inline-role forms; we drop the role marker and keep text.
# :ref:`display text <target>`    -> display text
# :ref:`target`                    -> target
# :at:`name`                       -> name
# :math:`a + b`                    -> a + b
# :at-val:`real, "0"`              -> real, "0"
# ``inline code``                  -> inline code
# |br|, |nbsp|, |*|, etc.          -> removed / replaced

ROLE_WITH_TEXT_RE = re.compile(r":[a-zA-Z-]+:`([^`<]*?)\s*<[^`>]*>`")
ROLE_BARE_RE = re.compile(r":[a-zA-Z-]+:`([^`]*)`")
DOUBLE_BACKTICK_RE = re.compile(r"``([^`]+)``")
SUBSTITUTION_RE = re.compile(r"\|([a-zA-Z0-9_]+)\|")
DIRECTIVE_BLOCK_RE = re.compile(r"\.\. [a-zA-Z-]+::.*", re.MULTILINE)

SUBSTITUTIONS = {
    "br": "\n",
    "nbsp": " ",
    "*": "*",
    "!": "",
    "?": "",
    "@": "",
    "-": "-",
    "minus": "-",
}


def clean_rst_markup(text: str) -> str:
    text = ROLE_WITH_TEXT_RE.sub(r"\1", text)
    text = ROLE_BARE_RE.sub(r"\1", text)
    text = DOUBLE_BACKTICK_RE.sub(r"\1", text)
    text = SUBSTITUTION_RE.sub(
        lambda m: SUBSTITUTIONS.get(m.group(1), ""), text)
    # Strip Sphinx directive markers like ".. raw:: html" on their own lines.
    text = DIRECTIVE_BLOCK_RE.sub("", text)
    # Dedent consistently by stripping leading indentation from every line.
    lines = [ln.lstrip() for ln in text.splitlines()]
    # Collapse >=2 blank lines to a single blank line.
    out: list[str] = []
    prev_blank = False
    for ln in lines:
        blank = not ln.strip()
        if blank and prev_blank:
            continue
        out.append(ln)
        prev_blank = blank
    return "\n".join(out).strip()


# ---------------------------------------------------------------------------
# XSD patcher
# ---------------------------------------------------------------------------

XS = "http://www.w3.org/2001/XMLSchema"
ET.register_namespace("xs", XS)
NS = {"xs": XS}


# Defaults in RST sometimes use `...` as a "pad with zeros" convention
# (e.g. `"0 0 ..."` for `user` attrs). These aren't valid XSD defaults.
_DOTS_RE = re.compile(r"\.\.\.")


def _default_invalid_reason(value: str, xsd_type: str) -> Optional[str]:
    """Return a reason string if this RST default is not XSD-compatible,
    or None if it's fine."""
    is_numeric_list = xsd_type and (
        xsd_type.startswith("list_") or
        xsd_type.startswith("uptolist_") or
        xsd_type.startswith("vec_"))
    is_numeric_scalar = xsd_type in ("xs:int", "xs:double")

    if (is_numeric_list or is_numeric_scalar) and _DOTS_RE.search(value):
        return "ellipsis in numeric default"
    if is_numeric_list:
        tokens = value.split()
        if not tokens:
            return "empty list default"
        item_kind = "real" if "real" in xsd_type else "int"
        for t in tokens:
            try:
                float(t) if item_kind == "real" else int(t)
            except ValueError:
                return f"non-numeric token '{t}' in {item_kind} list"
    elif xsd_type == "xs:int":
        try:
            int(value)
        except ValueError:
            return f"non-integer value '{value}' for xs:int"
    elif xsd_type == "xs:double":
        try:
            float(value)
        except ValueError:
            return f"non-double value '{value}' for xs:double"
    return None


def _find_containing_element(xs_root: ET.Element,
                             attr_node: ET.Element,
                             parents: dict[ET.Element, ET.Element]) -> str:
    """Return the 'name' of the nearest ancestor <xs:element>, or "" if none."""
    node = parents.get(attr_node)
    while node is not None:
        if node.tag == f"{{{XS}}}element":
            name = node.get("name")
            if name:
                return name
        node = parents.get(node)
    return ""


# Fallback elements when the direct (element, attr) lookup misses.
# Actuator shortcuts all inherit the bulk of their attrs from <general>; the
# RST documents the shared attrs once under actuator-general-*. Same story for
# <spatial>/<fixed> (which share <tendon>'s attrs).
ELEMENT_ALIASES: dict[str, list[str]] = {
    "motor":       ["general"],
    "position":    ["general"],
    "velocity":    ["general"],
    "intvelocity": ["general"],
    "damper":      ["general"],
    "cylinder":    ["general"],
    "muscle":      ["general"],
    "adhesion":    ["general"],
    "dcmotor":     ["general"],
    "plugin":      ["general"],
    "spatial":     ["tendon"],
    "fixed":       ["tendon"],
}


def _lookup_doc(docs: dict[tuple[str, str], DocEntry],
                element: str, attr: str) -> Optional[DocEntry]:
    doc = docs.get((element, attr))
    if doc:
        return doc
    for alias in ELEMENT_ALIASES.get(element, ()):
        doc = docs.get((alias, attr))
        if doc:
            return doc
    return None


@dataclass
class EnrichmentReport:
    """Summary + per-category drift info."""
    matched: int = 0                                        # attrs found in RST
    total: int = 0                                          # total xs:attributes
    no_doc: list[tuple[str, str]] = field(default_factory=list)        # (element, attr)
    bad_default: list[tuple[str, str, str, str]] = field(default_factory=list)  # (element, attr, raw_value, reason)
    unused_rst: list[tuple[str, str]] = field(default_factory=list)    # RST entry -> no XSD match


def patch_xsd(xsd_path: str,
              docs: dict[tuple[str, str], DocEntry],
              out_path: str,
              add_defaults: bool = True,
              add_annotations: bool = True,
              ) -> EnrichmentReport:
    """Patch the XSD in place. Returns an EnrichmentReport."""
    tree = ET.parse(xsd_path)
    root = tree.getroot()

    parents: dict[ET.Element, Optional[ET.Element]] = {
        child: parent for parent in root.iter() for child in parent}
    parents[root] = None

    used_keys: set[tuple[str, str]] = set()
    report = EnrichmentReport()

    for attr_node in root.iter(f"{{{XS}}}attribute"):
        attr_name = attr_node.get("name")
        if not attr_name:
            continue
        report.total += 1

        # Resolve the containing element name. Falls through to named
        # complexType for body_type / default_type.
        element_name = _find_containing_element(root, attr_node, parents)
        if not element_name:
            anc = parents.get(attr_node)
            while anc is not None:
                if anc.tag == f"{{{XS}}}complexType":
                    tname = anc.get("name", "")
                    if tname.endswith("_type"):
                        element_name = tname[:-len("_type")]
                        break
                anc = parents.get(anc)
        if not element_name:
            continue

        doc = _lookup_doc(docs, element_name, attr_name)
        if doc is None:
            report.no_doc.append((element_name, attr_name))
            continue
        used_keys.add((doc.element, attr_name))
        report.matched += 1

        if add_defaults and doc.default is not None \
                and "default" not in attr_node.attrib:
            xsd_type = attr_node.get("type", "")
            reason = _default_invalid_reason(doc.default, xsd_type)
            if reason is None:
                attr_node.set("default", doc.default)
            else:
                report.bad_default.append(
                    (element_name, attr_name, doc.default, reason))

        if add_annotations and doc.description:
            cleaned = clean_rst_markup(doc.description)
            if cleaned:
                # Remove any existing annotation we previously injected.
                for existing in list(
                    attr_node.findall(f"{{{XS}}}annotation")
                ):
                    attr_node.remove(existing)
                ann = ET.SubElement(attr_node, f"{{{XS}}}annotation")
                docnode = ET.SubElement(ann, f"{{{XS}}}documentation")
                docnode.text = cleaned

    _indent(root)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    report.unused_rst = sorted(set(docs.keys()) - used_keys)
    return report


def _indent(elem: ET.Element, level: int = 0) -> None:
    """In-place pretty-print (Python 3.9+ has ET.indent, but we reimplement
    here to stay compatible with older interpreters)."""
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
        for child in elem:
            _indent(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = pad
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _print_summary(r: EnrichmentReport, out_path: str) -> None:
    pct = (100.0 * r.matched / r.total) if r.total else 0.0
    print(f"\nEnrichment summary for {out_path}:", file=sys.stderr)
    print(f"  matched       {r.matched}/{r.total}  ({pct:.1f}%) "
          "xs:attributes have RST docs",
          file=sys.stderr)
    if r.no_doc:
        print(f"  warn          {len(r.no_doc)} xs:attributes have no "
              "RST documentation (RST may need updating)",
              file=sys.stderr)
    if r.bad_default:
        print(f"  warn          {len(r.bad_default)} RST defaults rejected "
              "(not XSD-compatible; RST may need updating)",
              file=sys.stderr)
    if r.unused_rst:
        print(f"  warn          {len(r.unused_rst)} RST entries have no "
              "matching xs:attribute (possible renamed/removed attrs)",
              file=sys.stderr)
    if r.no_doc or r.bad_default or r.unused_rst:
        print("  (run with --report for per-item details)", file=sys.stderr)


def _print_report(r: EnrichmentReport) -> None:
    if r.no_doc:
        by_element: dict[str, list[str]] = defaultdict(list)
        for (e, a) in sorted(r.no_doc):
            by_element[e].append(a)
        print(f"\n=== xs:attributes with NO RST docs ({len(r.no_doc)}) ===",
              file=sys.stderr)
        for e, attrs in sorted(by_element.items()):
            print(f"  <{e or '?'}>: {', '.join(attrs)}", file=sys.stderr)

    if r.bad_default:
        print(f"\n=== RST defaults rejected ({len(r.bad_default)}) ===",
              file=sys.stderr)
        print("  (fix by rewording the :at-val:`...` in the RST)",
              file=sys.stderr)
        for (e, a, v, reason) in r.bad_default:
            print(f"  <{e}>/{a}: default={v!r}  ({reason})", file=sys.stderr)

    if r.unused_rst:
        by_element = defaultdict(list)
        for (e, a) in r.unused_rst:
            by_element[e].append(a)
        print(f"\n=== RST entries with no matching xs:attribute "
              f"({len(r.unused_rst)}) ===",
              file=sys.stderr)
        print("  (possible stale RST: attr renamed or removed in the C++ "
              "table, or element aliasing missing in ELEMENT_ALIASES)",
              file=sys.stderr)
        for e, attrs in sorted(by_element.items()):
            print(f"  <{e or '?'}>: {', '.join(attrs)}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("--in", dest="in_path", required=True,
                    help="input XSD (from bin/xmlschema)")
    ap.add_argument("--rst", dest="rst_path", required=True,
                    help="path to doc/XMLreference.rst")
    ap.add_argument("--out", dest="out_path", required=True,
                    help="output enriched XSD")
    ap.add_argument("--no-defaults", action="store_true",
                    help="skip injecting default=... from RST")
    ap.add_argument("--no-annotations", action="store_true",
                    help="skip injecting xs:annotation prose")
    ap.add_argument("--report", action="store_true",
                    help="print per-item drift details")
    ap.add_argument("--strict", action="store_true",
                    help="exit non-zero if any drift warnings are emitted "
                         "(for CI gating)")
    args = ap.parse_args(argv)

    print(f"Parsing {args.rst_path} ...", file=sys.stderr)
    docs = parse_rst(args.rst_path)
    print(f"Extracted {len(docs)} (element, attr) doc entries from RST.",
          file=sys.stderr)

    report = patch_xsd(
        args.in_path, docs, args.out_path,
        add_defaults=not args.no_defaults,
        add_annotations=not args.no_annotations,
    )
    print(f"Wrote {args.out_path}.", file=sys.stderr)
    _print_summary(report, args.out_path)
    if args.report:
        _print_report(report)

    has_warnings = bool(report.no_doc or report.bad_default or report.unused_rst)
    return 1 if (args.strict and has_warnings) else 0


if __name__ == "__main__":
    sys.exit(main())
