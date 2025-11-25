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

#include "experimental/toolbox/physics.h"

#include <chrono>
#include <functional>
#include <ratio>
#include <span>
#include <string>
#include <utility>

#include "experimental/toolbox/helpers.h"
#include "experimental/toolbox/step_control.h"
#include <mujoco/mujoco.h>

namespace mujoco::toolbox {

static mjtNum Timer() {
  using Clock = std::chrono::steady_clock;
  using Milliseconds = std::chrono::duration<double, std::milli>;
  static Clock::time_point start = Clock::now();
  return Milliseconds(Clock::now() - start).count();
}

Physics::Physics(OnModelLoadedFn on_model_loaded)
    : on_model_loaded_(std::move(on_model_loaded)) {
  mjcb_time = Timer;
}

Physics::~Physics() { Clear(); }

void Physics::LoadModel(std::string model_file, const mjVFS* vfs) {
  pending_load_ = std::move(model_file);
  vfs_ = vfs;
}

bool Physics::ProcessPendingLoad() {
  if (!pending_load_.has_value()) {
    return model_ && data_;
  }

  Clear();

  step_control_.SetSpeed(100.f);

  std::string model_file = std::move(pending_load_.value());
  pending_load_.reset();

  model_ = LoadMujocoModel(model_file, vfs_);
  if (!model_) {
    error_ = "Error loading model!";
    step_control_.Pause();
    model_ = LoadMujocoModel("", vfs_);
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    error_ = "Error making data!";
    step_control_.Pause();
  }

  on_model_loaded_(model_file);

  const int state_size = mj_stateSize(model_, mjSTATE_INTEGRATION);
  sim_history_.Init(state_size);

  return model_ && data_;
}

void Physics::Clear() {
  if (model_) {
    mj_deleteData(data_);
    data_ = nullptr;
    mj_deleteModel(model_);
    model_ = nullptr;
    error_ = "";
  }
}

void Physics::Reset() {
  mj_resetData(model_, data_);
  mj_forward(model_, data_);
  error_ = "";
}

bool Physics::Update() {
  ProcessPendingLoad();

  if (!model_ || !data_) {
    return false;
  }

  if (data_) {
    for (int i = 0; i < mjNTIMER; i++) {
      data_->timer[i].duration = 0;
      data_->timer[i].number = 0;
    }
  }

  StepControl::Status status = step_control_.Advance(model_, data_);
  if (status == StepControl::Status::kOk) {
    std::span<mjtNum> state = sim_history_.AddToHistory();
    if (!state.empty()) {
      mj_getState(model_, data_, state.data(), mjSTATE_INTEGRATION);
    }
    // If we are adding to the history we didn't have a divergence error
    error_ = "";
  } else if (status == StepControl::Status::kPaused) {
    // do nothing
  } else if (status == StepControl::Status::kAutoReset) {
    Reset();
  } else if (status == StepControl::Status::kDiverged) {
    for (mjtWarning w : StepControl::kDivergedWarnings) {
      if (data_->warning[w].number > 0) {
        error_ = mju_warningText(w, data_->warning[w].lastinfo);
      }
    }
  }

  return true;
}

bool Physics::UpdateState(mjtNum* state, unsigned int state_sig) {
  ProcessPendingLoad();
  if (!model_ || !data_) {
    return false;
  }
  mj_setState(model_, data_, state, state_sig);
  mj_forward(model_, data_);
  return true;
}

void Physics::LoadHistory(int offset) {
  std::span<mjtNum> state = sim_history_.SetIndex(offset);
  if (!state.empty()) {
    // Pause simulation when entering history mode.
    step_control_.Pause();

    // Load the state into the data buffer.
    mj_setState(model_, data_, state.data(), mjSTATE_INTEGRATION);
    mj_forward(model_, data_);
  }
}

int Physics::GetHistoryIndex() const {
  return sim_history_.GetIndex();
}

}  // namespace mujoco::toolbox
