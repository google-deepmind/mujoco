// Copyright 2022 DeepMind Technologies Limited
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

#include "elasticity.h"
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::elasticity {

void String2Vector(const std::string& txt, std::vector<int>& vec) {
  std::stringstream strm(txt);
  vec.clear();

  while (!strm.eof()) {
    int num;
    strm >> num;
    if (strm.fail()) {
      break;
    } else {
      vec.push_back(num);
    }
  }
}

bool CheckAttr(const char* name, const mjModel* m, int instance) {
  char* end;
  std::string value = mj_getPluginConfig(m, instance, name);
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

}  // namespace mujoco::plugin::elasticity
