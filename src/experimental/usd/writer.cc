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
#include "experimental/usd/plugins/mjcf/mujoco_to_usd.h"
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>

namespace mujoco {
namespace usd {

// Writer is the main entry point for mj_usd. It is responsible
// for converting the USD scene into a Mujoco model and exposing stepping
// API.
Writer::Writer(pxr::UsdStageRefPtr stage, mjSpec* spec, mjModel_* model)
    : stage_(stage), spec_(spec), model_(model) {
  WriteSpecsWithoutUSDOriginToLayer();
  BuildMjUsdMapping();
}

Writer::~Writer() = default;

void Writer::WriteSpecsWithoutUSDOriginToLayer() {
  // The user may have constructed the spec from sources other than USD.
  // In this case, we will write write all those spec elements to a new layer in
  // the stage. This allows us to keep the same writing API for a simulation regardless
  // of how a spec was produced.
  mujoco::usd::WriteSpecToData(spec_, stage_->GetEditTarget().GetLayer(), true);
}

void Writer::BuildMjUsdMapping() {
  body_id_to_path_.assign(model_->nbody, pxr::SdfPath());

  // Use compiled element IDs rather than names: MJCF bodies may be unnamed.
  // Leave the world body and any elements without a compiled ID unmapped.
  for (mjsElement* element = mjs_firstElement(spec_, mjOBJ_BODY); element;
       element = mjs_nextElement(spec_, element)) {
    int body_id = mjs_getId(element);
    if (body_id > 0 && body_id < model_->nbody) {
      body_id_to_path_[body_id] =
          mujoco::usd::GetUsdPrimPathUserValue(element);
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
