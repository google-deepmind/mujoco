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

#ifndef MUJOCO_SRC_XML_XML_NATIVE_SCHEMA_H_
#define MUJOCO_SRC_XML_XML_NATIVE_SCHEMA_H_

#include <sstream>

#include "xml/xml_util.h"

namespace mujoco {

// Kind of data accepted in an attribute value. Mirrors the ReadAttr* / MapValue
// calls performed by mjXReader::OneX() parsers.
enum mjXAttrKind {
  kMjXString = 0,   // arbitrary string
  kMjXInt,          // single int
  kMjXIntN,         // exactly N ints        (fixed-length list)
  kMjXIntVec,       // variable-length int list
  kMjXReal,         // single real
  kMjXRealN,        // exactly N reals       (fixed-length list)
  kMjXRealUpToN,    // up to N reals         (variable, capped)
  kMjXRealVec,      // variable-length real list
  kMjXEnum,         // keyword from an mjMap
};

// One (element, attribute) binding. The generator looks up entries by
// (element, attr) to decide which xs:simpleType to reference.
// `map`/`map_size` are only used for kMjXEnum.
struct mjXAttr {
  const char* element;
  const char* attr;
  mjXAttrKind kind;
  int size;                  // for kMjXIntN / kMjXRealN / kMjXRealUpToN
  const mjMap* map;          // for kMjXEnum
  int map_size;              // for kMjXEnum
};

// Flat table of typed bindings. The generator walks MJCF[] for tree structure
// and consults this table for the type of each (element, attr) pair.
// Any pair present in MJCF[] but missing here is emitted as xs:string with a
// warning comment in the output.
extern const mjXAttr kMjXAttrTable[];
extern const int kMjXAttrTableSize;

// Emit an XSD document describing the MJCF format to `out`.
void PrintSchemaXSD(std::stringstream& out);

}  // namespace mujoco

#endif  // MUJOCO_SRC_XML_XML_NATIVE_SCHEMA_H_
