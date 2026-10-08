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

#include "xml/xml_numeric_format.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

namespace mujoco {

namespace {
thread_local int precision = 6;
}

int _mjPRIVATE__get_xml_precision() {
  return precision;
}

void _mjPRIVATE__set_xml_precision(const int new_precision) {
  precision = new_precision;
}

namespace {

// the shortest text which reads back as value. A number which has such a text with up to
// digits10 significant digits is printed as that text when it is printed with digits10 of them,
// so fewer digits need not be tried
template <typename T>
std::string Shortest(T value, T (*read)(const char*, char**)) {
  char text[40];

  // a whole number is written without an exponent
  if (value == std::trunc(value) && std::abs(value) < 1e15) {
    std::snprintf(text, sizeof(text), "%.0f", static_cast<double>(value));
    return text;
  }

  const int most = std::numeric_limits<T>::max_digits10;
  for (int digits = std::numeric_limits<T>::digits10; digits <= most; digits++) {
    std::snprintf(text, sizeof(text), "%.*g", digits, static_cast<double>(value));
    if (read(text, nullptr) == value) { break; }
  }
  return text;
}

}  // namespace

std::string ShortestNumber(double value) {
  return Shortest<double>(value, std::strtod);
}

std::string ShortestNumber(float value) {
  return Shortest<float>(value, std::strtof);
}

}  // namespace mujoco
