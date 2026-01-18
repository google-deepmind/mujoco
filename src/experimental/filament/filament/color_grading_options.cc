// Copyright 2025 DeepMind Technologies Limited
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

#include "experimental/filament/filament/color_grading_options.h"

#include <memory>

#include <filament/ToneMapper.h>
#include <mujoco/mujoco.h>

namespace mujoco {

std::unique_ptr<filament::ToneMapper> CreateToneMapper(ToneMapperType type) {
  filament::ToneMapper* ptr = nullptr;
  switch (type) {
    case ToneMapperType::kPBRNeutral:
      ptr = new filament::PBRNeutralToneMapper();
      break;
    case ToneMapperType::kACES:
      ptr = new filament::ACESToneMapper();
      break;
    case ToneMapperType::kACESLegacy:
      ptr = new filament::ACESLegacyToneMapper();
      break;
    case ToneMapperType::kFilmic:
      ptr = new filament::FilmicToneMapper();
      break;
    case ToneMapperType::kLinear:
      ptr = new filament::LinearToneMapper();
      break;
    default:
      mju_error("Unsupported tone mapper type");
  }
  return std::unique_ptr<filament::ToneMapper>(ptr);
}

}  // namespace mujoco
