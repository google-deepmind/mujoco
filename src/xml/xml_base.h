// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_XML_XML_BASE_H_
#define MUJOCO_SRC_XML_XML_BASE_H_

#include <cstdlib>
#include <string>

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include "xml/xml_util.h"
#include "tinyxml2.h"


// keyword maps, one per schema enum, generated into mjcf_map.h
#include "xml/generated/mjcf_map.h"  // IWYU pragma: export

//---------------------------------- Base XML class ------------------------------------------------

class mjXBase : public mjXUtil {
 public:
  mjXBase();
  virtual ~mjXBase() = default;

  // parse: implemented in derived parser classes
  virtual void Parse(tinyxml2::XMLElement* root, const mjVFS* vfs = nullptr) {};

  // write: implemented in derived writer class
  virtual std::string Write(char* error, std::size_t error_sz) { return ""; };

  // set the model allocated externally
  virtual void SetModel(mjSpec*, const mjModel* = nullptr);

  // read alternative orientation specification
  static int ReadAlternative(tinyxml2::XMLElement* elem, mjsOrientation& alt);

 protected:
  mjSpec* spec;  // internally-allocated model
};

// typed attribute row for table-driven reading/writing; per-element row arrays are
// generated from mjcf.schema into mjcf_read_table.inc
struct mjXAttr {
  enum Kind {
    kName,       // element name, set via mjs_setName
    kString,     // mjString* field, set via mjs_setString
    kStringVec,  // mjStringVec* field: space-separated names
    kInt,        // int field
    kDouble,     // double field, scalar or vector
    kNum,        // mjtNum field, scalar or vector
    kFloat,      // float field, scalar or vector
    kEnum,       // int-sized enum field, keyword mapped through `map`
    kFlags,      // int field ORing several keywords through `map`
    kEnumByte,   // mjtByte enum field, keyword mapped through `map`
    kBool,       // mjtBool field, keywords true/false
    kConst,      // int-sized field set to `value`: what the tag implies
    kDoubleVec,  // mjDoubleVec* field, set via mjs_setDouble
    kFloatVec,   // mjFloatVec* field, set via mjs_setFloat
    kIntVec,     // mjIntVec* field, set via mjs_setInt
    kChars,      // char[len] field: text copied in place, length-checked
  };
  const char*  attr;  // XML attribute name
  Kind         kind;
  int          len;    // number of values (1 = scalar); numeric kinds only
  bool         exact;  // exactly len values, else up to len
  bool         required;
  bool         nodefault;  // not read in default classes
  bool         handwrite;  // custom save policy (writing=custom): OneX() remnant
  int          offset;     // byte offset of the bound field; -1 for kName
  const mjMap* map;        // keyword map; kEnum only
  int          mapsz;      // keyword map size; kEnum only
  int          value;      // constant the field takes; kConst only
};

// the parameters of an actuator shortcut (position, muscle, ...): what its tag's attributes say,
// in one struct for all shortcuts. Reader and writer share the mapping to and from the general
// control model, so what a shortcut reads is what the writer gives it back
struct mjXShortcut {
  double kp, kv, dampratio, timeconst[2], tausmooth, range[2], force, scale, lmin, lmax, vmax,
      fpmax, fvmax, bias[3], area, diameter, gain, ki, imax, slewmax, motorconst[2], resistance,
      nominal[3], saturation[3], inductance[2], cogging[3], controller[6], thermal[6], lugre[5],
      inheritrange;
  int  ctrlspec;
  bool has_kv, has_dampratio, has_timeconst;  // kv/dampratio and timeconst are given or not
};

// the class slot a shortcut takes its parameters from: the class's own if written with the same
// shortcut, or with general and the shortcut's gain model, else the nearest such ancestor class;
// null when the shortcut starts from its documented defaults
class mjCDef;
const mjsActuator* mjXShortcutSource(const mjsActuator* slot, const mjCDef* def, mjtActuator type);

// the documented defaults of a shortcut's parameters
void mjXShortcutDefaults(mjXShortcut* s, mjtActuator type);

// the shortcut parameters of the given type which gave an actuator its control model
void mjXShortcutFromActuator(mjXShortcut* s, const mjsActuator* actuator, mjtActuator type);

// set an actuator's control model from a shortcut; empty string on success
const char* mjXSetToShortcut(mjsActuator* actuator, mjtActuator type, const mjXShortcut& s);

#endif  // MUJOCO_SRC_XML_XML_BASE_H_
