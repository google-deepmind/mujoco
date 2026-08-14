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

#ifndef MUJOCO_SRC_XML_XML_NUMERIC_FORMAT_H_
#define MUJOCO_SRC_XML_XML_NUMERIC_FORMAT_H_

#include <mujoco/mjexport.h>

namespace mujoco {

extern "C" {
  MJAPI int _mjPRIVATE__get_xml_precision();
  MJAPI void _mjPRIVATE__set_xml_precision(const int precision);
}

// Full precision printing of floating point numbers in saved XMLs, useful for testing
class FullFloatPrecision {
 public:
  FullFloatPrecision() { _mjPRIVATE__set_xml_precision(17);}
  ~FullFloatPrecision() { _mjPRIVATE__set_xml_precision(6);}
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_XML_NUMERIC_FORMAT_H_

