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

#include "bolt.h"
#include "bowl.h"
#include "gear.h"
#include "nut.h"
#include "torus.h"
#include "sdflib.h"

namespace mujoco::plugin::sdf {

mjPLUGIN_LIB_INIT {
  Bolt::RegisterPlugin();
  Bowl::RegisterPlugin();
  Gear::RegisterPlugin();
  Nut::RegisterPlugin();
  Torus::RegisterPlugin();
  SdfLib::RegisterPlugin();
}

}  // namespace mujoco::plugin::sdf
