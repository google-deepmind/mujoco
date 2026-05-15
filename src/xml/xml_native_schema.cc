// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// XSD generator for MJCF.
//
// Walks MJCF[] from xml_native_reader.h for tree structure, and consults
// kMjXAttrTable (also in xml_native_reader.cc) for per-attribute type info.
//
// Release pipeline (regenerating doc/mjcf.xsd):
//   $ cmake --build build --target xmlschema
//   $ ./build/bin/xmlschema /tmp/raw.xsd
//   $ ./doc/mjcf_schema_enrich.py --in /tmp/raw.xsd \
//         --rst doc/XMLreference.rst --out doc/mjcf.xsd --strict --report

#include "xml/xml_native_schema.h"

#include <cstddef>
#include <cstring>
#include <sstream>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "xml/xml_native_reader.h"  // MJCF[], nMJCF, mjXAttr*, kMjXAttrTable
#include "xml/xml_util.h"           // mjMap


namespace mujoco {


//---------------------------------- MJCF tree parser ----------------------------------------------

namespace {

// MJCF "body family": <body>, <frame>, <replicate> share <body>'s content
// model but each carries its own attribute set, and <worldbody> is the name
// <body> takes at the top level of <mujoco>. mjXSchema::NameMatch in
// xml_util.cc encodes the same rule at parse time -- keep the two in sync.
constexpr const char* kBodyAliases[] = {"body", "frame", "replicate"};
constexpr const char* kWorldbody = "worldbody";

bool IsBody(const char* s)      { return std::strcmp(s, "body") == 0; }
bool IsWorldbody(const char* s) { return std::strcmp(s, kWorldbody) == 0; }

struct Node {
  const char* name = nullptr;
  char type = '?';
  std::vector<const char*> attrs;
  std::vector<Node> children;
};

int ParseNode(int start, int nrow, Node& out) {
  out.name = MJCF[start][0];
  out.type = MJCF[start][1][0];
  const int nattr = static_cast<int>(MJCF[start].size()) - 2;
  for (int i = 0; i < nattr; i++) {
    out.attrs.push_back(MJCF[start][2 + i]);
  }

  int next = start + 1;
  if (next < nrow && MJCF[next][0][0] == '<') {
    next++;  // skip '<'
    while (next < nrow && MJCF[next][0][0] != '>') {
      Node child;
      next = ParseNode(next, nrow, child);
      out.children.push_back(std::move(child));
    }
    next++;  // skip '>'
  }
  return next;
}


//---------------------------------- XSD emit helpers ----------------------------------------------

void Indent(std::stringstream& out, int level) {
  for (int i = 0; i < level; i++) out << "  ";
}

// O(1) lookup into kMjXAttrTable by (element, attr). All strings in the
// table (and in MJCF[], which supplies the query) are string literals with
// static lifetime, so storing string_view in the key is safe.
const mjXAttr* Lookup(const char* element, const char* attr) {
  using Key = std::pair<std::string_view, std::string_view>;
  struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept {
      return std::hash<std::string_view>{}(k.first) * 31 +
             std::hash<std::string_view>{}(k.second);
    }
  };
  static const auto table = [] {
    std::unordered_map<Key, const mjXAttr*, KeyHash> m;
    m.reserve(kMjXAttrTableSize);
    for (int i = 0; i < kMjXAttrTableSize; i++) {
      const mjXAttr& e = kMjXAttrTable[i];
      m.emplace(Key{e.element, e.attr}, &e);
    }
    return m;
  }();
  auto it = table.find(Key{element, attr});
  return it == table.end() ? nullptr : it->second;
}

void CardinalityToOccurs(char type, const char** min_occurs,
                         const char** max_occurs) {
  switch (type) {
    case '!': *min_occurs = "1"; *max_occurs = "1";         break;
    case '?': *min_occurs = "0"; *max_occurs = "1";         break;
    case '*': *min_occurs = "0"; *max_occurs = "unbounded"; break;
    case 'R': *min_occurs = "0"; *max_occurs = "unbounded"; break;
    default:  *min_occurs = "0"; *max_occurs = "1";         break;
  }
}

std::string EnumTypeName(const char* element, const char* attr) {
  return std::string(element) + "_" + attr + "_enum";
}

// Composite list-like attribute type (int/real list, fixed/variable length,
// optionally capped). Collected into an ordered set and emitted once per
// distinct (kind, size) combination.
struct ListType {
  mjXAttrKind kind;
  int size;  // ignored for Vec variants

  bool IsReal() const {
    return kind == kMjXRealN || kind == kMjXRealUpToN || kind == kMjXRealVec;
  }
  bool IsVariable() const { return kind == kMjXIntVec || kind == kMjXRealVec; }
  bool IsCapped()   const { return kind == kMjXRealUpToN; }

  std::string Name() const {
    switch (kind) {
      case kMjXIntN:      return "list_int_"      + std::to_string(size);
      case kMjXRealN:     return "list_real_"     + std::to_string(size);
      case kMjXRealUpToN: return "uptolist_real_" + std::to_string(size);
      case kMjXIntVec:    return "vec_int";
      case kMjXRealVec:   return "vec_real";
      default:            return "unknown";
    }
  }

  // Ordered by name so iteration over std::set<ListType> produces the same
  // emission order as the previous string-keyed set.
  bool operator<(const ListType& other) const {
    return Name() < other.Name();
  }
};


//---------------------------------- Emit: enum / list types ---------------------------------------

// Canonical enum-type name per distinct mjMap*. Populated during CollectTypes
// by walking MJCF[] in tree order: the first (element, attr) pair that
// references a given map provides the name, and every subsequent attribute
// using the same map reuses it. Dramatically shrinks the generated XSD
// because maps like bool_map, interp_map, enable_map are referenced by many
// attributes.
std::unordered_map<const mjMap*, std::string>& CanonicalEnumNames() {
  static std::unordered_map<const mjMap*, std::string> m;
  return m;
}

std::string EnumTypeFor(const mjXAttr& info, const char* element,
                        const char* attr) {
  auto& map = CanonicalEnumNames();
  auto it = map.find(info.map);
  if (it != map.end()) return it->second;
  // Fallback for attributes that are emitted before CollectTypes has run
  // (e.g. direct calls from tests). Matches the pre-dedup naming scheme.
  return EnumTypeName(element, attr);
}

void CollectTypes(
    const Node& node,
    std::set<ListType>& list_types,
    std::vector<std::pair<std::string, const mjXAttr*>>& enum_types) {
  for (const char* a : node.attrs) {
    const mjXAttr* info = Lookup(node.name, a);
    if (!info) continue;

    if (info->kind == kMjXEnum) {
      auto [it, inserted] = CanonicalEnumNames().try_emplace(
          info->map, EnumTypeName(node.name, a));
      if (inserted) {
        enum_types.emplace_back(it->second, info);
      }
    } else if (info->kind == kMjXIntN || info->kind == kMjXRealN ||
               info->kind == kMjXRealUpToN || info->kind == kMjXIntVec ||
               info->kind == kMjXRealVec) {
      list_types.insert(ListType{info->kind, info->size});
    }
  }
  for (const Node& child : node.children) {
    CollectTypes(child, list_types, enum_types);
  }
}

void EmitListType(std::stringstream& out, const ListType& t) {
  Indent(out, 1);
  out << "<xs:simpleType name=\"" << t.Name() << "\">\n";
  Indent(out, 2);
  out << "<xs:restriction>\n";
  Indent(out, 3);
  out << "<xs:simpleType>\n";
  Indent(out, 4);
  out << "<xs:list itemType=\"" << (t.IsReal() ? "xs:double" : "xs:int")
      << "\"/>\n";
  Indent(out, 3);
  out << "</xs:simpleType>\n";
  if (t.IsVariable()) {
    // unconstrained length
  } else if (t.IsCapped()) {
    Indent(out, 3);
    out << "<xs:minLength value=\"1\"/>\n";
    Indent(out, 3);
    out << "<xs:maxLength value=\"" << t.size << "\"/>\n";
  } else {
    Indent(out, 3);
    out << "<xs:length value=\"" << t.size << "\"/>\n";
  }
  Indent(out, 2);
  out << "</xs:restriction>\n";
  Indent(out, 1);
  out << "</xs:simpleType>\n";
}

void EmitEnumType(std::stringstream& out, const std::string& name,
                  const mjXAttr& info) {
  Indent(out, 1);
  out << "<xs:simpleType name=\"" << name << "\">\n";
  Indent(out, 2);
  out << "<xs:restriction base=\"xs:string\">\n";
  for (int i = 0; i < info.map_size; i++) {
    // Some map arrays in xml_native_reader.cc are sized larger than the
    // number of initialized entries (e.g. elastic2d_map[5] with 4 entries);
    // skip nullptr sentinels defensively.
    if (!info.map[i].key) break;
    Indent(out, 3);
    out << "<xs:enumeration value=\"" << info.map[i].key << "\"/>\n";
  }
  Indent(out, 2);
  out << "</xs:restriction>\n";
  Indent(out, 1);
  out << "</xs:simpleType>\n";
}


//---------------------------------- Emit: elements ------------------------------------------------

void EmitAttribute(std::stringstream& out, int level, const char* element,
                   const char* attr) {
  const mjXAttr* info = Lookup(element, attr);
  Indent(out, level);
  out << "<xs:attribute name=\"" << attr << "\"";
  if (!info) {
    out << " type=\"xs:string\"/>  <!-- No type found -->\n";
    return;
  }
  switch (info->kind) {
    case kMjXString: out << " type=\"xs:string\"/>\n"; break;
    case kMjXInt:    out << " type=\"xs:int\"/>\n";    break;
    case kMjXReal:   out << " type=\"xs:double\"/>\n"; break;
    case kMjXEnum:
      out << " type=\"" << EnumTypeFor(*info, element, attr) << "\"/>\n";
      break;
    case kMjXIntN:
    case kMjXRealN:
    case kMjXRealUpToN:
    case kMjXIntVec:
    case kMjXRealVec:
      out << " type=\"" << ListType{info->kind, info->size}.Name() << "\"/>\n";
      break;
  }
}

// Is this node recursive? ('R' cardinality)
bool IsRecursive(const Node& n) { return n.type == 'R'; }

// Emit the body of a complexType (choice + attributes) for a given element.
// Separated from EmitElement so it can be reused for named types that are
// referenced from multiple places.
void EmitComplexTypeBody(std::stringstream& out, const Node& node, int level,
                         const char* effective_name) {
  if (!node.children.empty() || IsRecursive(node) ||
      IsBody(effective_name)) {
    Indent(out, level);
    out << "<xs:choice minOccurs=\"0\" maxOccurs=\"unbounded\">\n";
    for (const Node& child : node.children) {
      extern void EmitElementInternal(std::stringstream&, const Node&, int,
                                       bool, const char*);
      EmitElementInternal(out, child, level + 1, /*top_level=*/false, nullptr);
    }
    // Recursive self-reference for 'R' elements.
    if (IsRecursive(node)) {
      Indent(out, level + 1);
      out << "<xs:element name=\"" << effective_name << "\" type=\""
          << effective_name << "_type\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
    }
    // Body special-cases: <body> can also contain any body-family alias
    // (per mjXSchema::NameMatch). Each alias points at its own named
    // complexType so its attributes (declared in kMjXAttrTable) are honored
    // -- body_type would drop them.
    if (IsBody(effective_name) ||
        IsWorldbody(effective_name)) {
      for (const char* alias : kBodyAliases) {
        Indent(out, level + 1);
        out << "<xs:element name=\"" << alias << "\" type=\"" << alias << "_type\""
            << " minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
      }
    }
    // <include> is handled by the XML preprocessor (xml.cc::IncludeXML) and
    // can appear as a child of any element. Always allow it.
    Indent(out, level + 1);
    out << "<xs:element ref=\"include\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
    Indent(out, level);
    out << "</xs:choice>\n";
  }

  for (const char* a : node.attrs) {
    EmitAttribute(out, level, node.name, a);
  }
}

// Internal implementation (exposed via the `extern` inside EmitComplexTypeBody).
void EmitElementInternal(std::stringstream& out, const Node& node, int level,
                         bool top_level, const char* name_override) {
  const char* min_occurs = "0";
  const char* max_occurs = "1";
  CardinalityToOccurs(node.type, &min_occurs, &max_occurs);

  const char* element_name = name_override ? name_override : node.name;

  // Recursive elements use a named complexType so they can be self-referenced.
  const bool use_named_type = IsRecursive(node) ||
                              IsBody(node.name);

  Indent(out, level);
  out << "<xs:element name=\"" << element_name << "\"";
  if (use_named_type) {
    out << " type=\"" << node.name << "_type\"";
  }
  if (!top_level) {
    if (std::string(min_occurs) != "1") out << " minOccurs=\"" << min_occurs << "\"";
    if (std::string(max_occurs) != "1") out << " maxOccurs=\"" << max_occurs << "\"";
  }
  if (use_named_type) {
    out << "/>\n";
    return;
  }
  out << ">\n";

  Indent(out, level + 1);
  out << "<xs:complexType>\n";
  EmitComplexTypeBody(out, node, level + 2, element_name);
  Indent(out, level + 1);
  out << "</xs:complexType>\n";
  Indent(out, level);
  out << "</xs:element>\n";
}

// Collect recursive (or body-family) nodes for named-type emission.
void CollectNamedTypes(const Node& node, std::vector<const Node*>& out) {
  if (IsRecursive(node) || IsBody(node.name)) {
    out.push_back(&node);
  }
  for (const Node& child : node.children) {
    CollectNamedTypes(child, out);
  }
}

void EmitNamedType(std::stringstream& out, const Node& node) {
  Indent(out, 1);
  out << "<xs:complexType name=\"" << node.name << "_type\">\n";
  EmitComplexTypeBody(out, node, 2, node.name);
  Indent(out, 1);
  out << "</xs:complexType>\n";
}

// Emit a named complexType that shares the <body> content model but has its
// own attributes (from kMjXAttrTable). Used for <frame> and <replicate>,
// which are aliases of <body> at the tree level but carry distinct attrs.
void EmitAliasType(std::stringstream& out, const Node& body_node,
                   const char* alias_name) {
  Indent(out, 1);
  out << "<xs:complexType name=\"" << alias_name << "_type\">\n";
  // Choice block mirrors body_type: same children + body-family aliases.
  Indent(out, 2);
  out << "<xs:choice minOccurs=\"0\" maxOccurs=\"unbounded\">\n";
  for (const Node& child : body_node.children) {
    EmitElementInternal(out, child, 3, /*top_level=*/false, nullptr);
  }
  // Body-family recursion: allow every alias nested inside.
  for (const char* sub : kBodyAliases) {
    Indent(out, 3);
    out << "<xs:element name=\"" << sub << "\" type=\"" << sub << "_type\""
        << " minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
  }
  Indent(out, 3);
  out << "<xs:element ref=\"include\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
  Indent(out, 2);
  out << "</xs:choice>\n";
  // Attributes come from the flat kMjXAttrTable entries for this element.
  for (int i = 0; i < kMjXAttrTableSize; i++) {
    const mjXAttr& row = kMjXAttrTable[i];
    if (std::string(row.element) == alias_name) {
      EmitAttribute(out, 2, alias_name, row.attr);
    }
  }
  Indent(out, 1);
  out << "</xs:complexType>\n";
}

}  // namespace


//---------------------------------- Entry point ---------------------------------------------------

void PrintSchemaXSD(std::stringstream& out) {
  Node root;
  ParseNode(0, nMJCF, root);

  // Reset canonical enum-name state in case this function is called more
  // than once per process lifetime.
  CanonicalEnumNames().clear();

  std::set<ListType> list_types;
  std::vector<std::pair<std::string, const mjXAttr*>> enum_types;
  CollectTypes(root, list_types, enum_types);

  // Find recursive / body-family nodes so we can emit them as named types.
  std::vector<const Node*> named_types;
  CollectNamedTypes(root, named_types);

  out << "<?xml version=\"1.0\"?>\n";
  out << "<!-- Auto-generated from MuJoCo MJCF schema. DO NOT EDIT. -->\n";
  out << "<xs:schema xmlns:xs=\"http://www.w3.org/2001/XMLSchema\""
         " elementFormDefault=\"qualified\">\n";

  if (!list_types.empty()) {
    out << "\n  <!-- List types -->\n";
    for (const ListType& t : list_types) {
      EmitListType(out, t);
    }
  }

  if (!enum_types.empty()) {
    out << "\n  <!-- Enum types -->\n";
    for (const auto& [n, info] : enum_types) {
      EmitEnumType(out, n, *info);
    }
  }

  // Named complex types (for recursive elements like <body> and <default>).
  if (!named_types.empty()) {
    out << "\n  <!-- Recursive complex types -->\n";
    const Node* body_node = nullptr;
    for (const Node* n : named_types) {
      EmitNamedType(out, *n);
      if (IsBody(n->name)) body_node = n;
    }
    // Non-canonical body-family aliases share body's content model but each
    // has its own attribute set in kMjXAttrTable; emit dedicated types so
    // those attrs are actually applied when validating.
    if (body_node) {
      for (const char* alias : kBodyAliases) {
        if (IsBody(alias)) continue;
        EmitAliasType(out, *body_node, alias);
      }
    }
  }

  // Global <include> element (preprocessor directive, allowed as a child of
  // any element in the tree).
  out << "\n  <!-- Preprocessor include -->\n";
  Indent(out, 1);
  out << "<xs:element name=\"include\">\n";
  Indent(out, 2);
  out << "<xs:complexType>\n";
  Indent(out, 3);
  out << "<xs:attribute name=\"file\" type=\"xs:string\" use=\"required\"/>\n";
  Indent(out, 3);
  out << "<xs:attribute name=\"prefix\" type=\"xs:string\"/>\n";
  Indent(out, 2);
  out << "</xs:complexType>\n";
  Indent(out, 1);
  out << "</xs:element>\n";

  out << "\n  <!-- Root element -->\n";

  // Emit the <mujoco> root manually so we can rename top-level <body> →
  // <worldbody> (mjXSchema::NameMatch handles this at level==1).
  Indent(out, 1);
  out << "<xs:element name=\"" << root.name << "\">\n";
  Indent(out, 2);
  out << "<xs:complexType>\n";
  Indent(out, 3);
  out << "<xs:choice minOccurs=\"0\" maxOccurs=\"unbounded\">\n";
  for (const Node& child : root.children) {
    const char* override_name = IsBody(child.name) ? kWorldbody : nullptr;
    EmitElementInternal(out, child, 4, /*top_level=*/false, override_name);
  }
  Indent(out, 4);
  out << "<xs:element ref=\"include\" minOccurs=\"0\" maxOccurs=\"unbounded\"/>\n";
  Indent(out, 3);
  out << "</xs:choice>\n";
  for (const char* a : root.attrs) {
    EmitAttribute(out, 3, root.name, a);
  }
  Indent(out, 2);
  out << "</xs:complexType>\n";
  Indent(out, 1);
  out << "</xs:element>\n";

  out << "</xs:schema>\n";
}

}  // namespace mujoco
