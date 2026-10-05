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

#ifndef MUJOCO_TEST_COMPARE_SPEC_H_
#define MUJOCO_TEST_COMPARE_SPEC_H_

#include <string>

#include <mujoco/mujoco.h>

namespace mujoco {

// Compares what was authored in two mjSpecs: the fields of the spec and of
// every element, and the name, default class, parent body, frame and order of
// the elements. Default classes are compared if an element or a childclass
// refers to them. Ids, signatures and anything else that compilation records
// are not compared.
// Returns the differences, one per line, or an empty string if there are none.
std::string CompareSpec(const mjSpec* s1, const mjSpec* s2);

}  // namespace mujoco

#endif  // MUJOCO_TEST_COMPARE_SPEC_H_
