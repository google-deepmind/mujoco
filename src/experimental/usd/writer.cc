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

#include <mujoco/experimental/usd/writer.h>

#include <utility>
#include <vector>

#include <mujoco/experimental/usd/utils.h>
#include <mujoco/mujoco.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>

namespace mujoco {
namespace usd {

// Writer is the main entry point for mj_usd. It is responsible
// for converting the USD scene into a Mujoco model and exposing stepping
// API.
Writer::Writer(pxr::UsdStageRefPtr stage, mjSpec* spec, mjModel_* model)
    : stage_(stage), spec_(spec), model_(model) {
  BuildMjUsdMapping();
}

Writer::~Writer() = default;

void Writer::BuildMjUsdMapping() {
  body_id_to_path_.assign(model_->nbody, pxr::SdfPath());

  // World body (ID 0) maps to an empty SdfPath.
  if (model_->nbody > 0) {
    body_id_to_path_[0] = pxr::SdfPath();
  }

  // Iterate over MuJoCo body IDs from the mjModel.
  for (int body_idx = 1; body_idx < model_->nbody; ++body_idx) {
    const char* body_name = mj_id2name(model_, mjOBJ_BODY, body_idx);
    if (body_name) {
      mjsBody* spec_body = mjs_findBody(spec_, body_name);
      body_id_to_path_[body_idx] =
          mujoco::usd::GetUsdPrimPathUserValue(spec_body->element);
    }
  }
}

void Writer::Update(const mjData* const data) {
  for (const auto& sink_fn : pose_sinks_) {
    sink_fn(data, body_id_to_path_);
  }
}

void Writer::AddSink(PoseSinkFn sink_fn) {
  pose_sinks_.push_back(std::move(sink_fn));
}

}  // namespace usd
}  // namespace mujoco
