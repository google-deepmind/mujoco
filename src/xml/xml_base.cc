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

#include "xml/xml_base.h"

#include <cfloat>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "user/user_model.h"
#include "user/user_objects.h"
#include "tinyxml2.h"

namespace {

using std::string;
using tinyxml2::XMLElement;

}  // namespace


//--------------------------------- Base class, helper functions -----------------------------------

// base constructor
mjXBase::mjXBase() {
  model = NULL;
}



// set model field
void mjXBase::SetModel(mjCModel* _model) {
  model = _model;
}



// read alternative orientation specification
void mjXBase::ReadAlternative(XMLElement* elem, mjCAlternative& alt) {
  string text;
  int read = (int)(elem->Attribute("quat") != 0) +
             (ReadAttr(elem, "axisangle", 4, alt.axisangle, text) ? 1 : 0) +
             (ReadAttr(elem, "xyaxes", 6, alt.xyaxes, text) ? 1 : 0) +
             (ReadAttr(elem, "zaxis", 3, alt.zaxis, text) ? 1 : 0) +
             (ReadAttr(elem, "euler", 3, alt.euler, text) ? 1 : 0) +
             (ReadAttr(elem, "fullinertia", 6, alt.fullinertia, text) ? 1 : 0);
  if (read > 1)
    throw mjXError(elem, "multiple orientation specifiers for the same field are not allowed");
}
