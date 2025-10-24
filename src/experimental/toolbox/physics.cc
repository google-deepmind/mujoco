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

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstring>
#include <functional>
#include <ratio>
#include <string>
#include <utility>
#include <vector>

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

  std::string model_file = std::move(pending_load_.value());
  pending_load_.reset();

  model_ = LoadMujocoModel(model_file, vfs_);
  if (!model_) mju_error("Error loading model");

  data_ = mj_makeData(model_);
  if (!data_) mju_error("Error making data.");

  on_model_loaded_(model_file);

  InitHistory();

  return model_ && data_;
}

void Physics::Clear() {
  if (model_) {
    mj_deleteData(data_);
    data_ = nullptr;
    mj_deleteModel(model_);
    model_ = nullptr;

    history_.clear();
    history_cursor_ = 0;
    steps_ = 0;
    GetStepControl().SetSpeed(100.f);

    error_ = "";
  }
}

void Physics::Reset() {
  mj_resetData(model_, data_);
  mj_forward(model_, data_);
  error_ = "";
  history_cursor_ = 0;
}

bool Physics::Update(const mjvPerturb* perturb) {
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

  if (!IsPaused()) {
    mju_zero(data_->xfrc_applied, 6 * model_->nbody);
    mjv_applyPerturbPose(model_, data_, perturb, 0);
    mjv_applyPerturbForce(model_, data_, perturb);
  } else {
    mjv_applyPerturbPose(model_, data_, perturb, 1);
  }

  if (IsPaused() && !single_step_) {
    // run mj_forward, to update rendering and joint sliders
    mj_forward(model_, data_);
    if (pause_update_) {
      mju_copy(data_->qacc_warmstart, data_->qacc, model_->nv);
    }

    // When unpaused make sure we sync to immediately and step once. Without
    // this we could step many times before rendering resulting in a noticeable
    // delay before the simulation restarts (especially for large slowdowns)
    GetStepControl().ForceSync();
  } else {
    if (single_step_) {
      GetStepControl().ForceSync();
      single_step_ = false;
    }

    StepControl::Status status = GetStepControl().Advance(model_, data_);
    if (status == StepControl::Status::kOk) {
      AddToHistory();
    } else if (status == StepControl::Status::kAutoReset) {
      Reset();
    } else if (status == StepControl::Status::kDiverged) {
      for (mjtWarning w : StepControl::kDivergedWarnings) {
        if (data_->warning[w].number > 0) {
          paused_ = true;
          error_ = mju_warningText(w, data_->warning[w].lastinfo);
        }
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

void Physics::TogglePause() { paused_ = !paused_; }

void Physics::RequestSingleStep() { single_step_ = true; }

void Physics::InitHistory() {
  const int state_size = mj_stateSize(model_, mjSTATE_INTEGRATION);

  // History buffer will be smaller of 2000 states or 100 MB.
  constexpr int kMaxBytes = 1e8;
  constexpr int kMaxHistory = 2000;
  const int state_bytes = state_size * sizeof(mjtNum);
  const int history_length = std::min(INT_MAX / state_bytes, kMaxHistory);
  const int history_bytes = std::min(state_bytes * history_length, kMaxBytes);
  const int num_history = history_bytes / state_bytes;

  history_.resize(num_history);
  for (std::vector<mjtNum>& state : history_) {
    state.resize(state_size, 0);
  }
  history_cursor_ = 0;
}

void Physics::AddToHistory() {
  if (!history_.empty()) {
    mjtNum* state = history_[history_cursor_].data();
    mj_getState(model_, data_, state, mjSTATE_INTEGRATION);
    history_cursor_ = (history_cursor_ + 1) % history_.size();
    steps_++;

    // If we are adding to the history we didn't have a divergence error
    error_ = "";
  }
}

int Physics::LoadHistory(int offset) {
  // No history to load.
  if (steps_ == 0) {
    return 0;
  }

  // Pause simulation when entering history mode.
  paused_ = true;

  // Ensure the offset is within a valid range. It's a negative value since
  // we will be going backwards from the "latest" frame.
  const int max_history = std::min<int>(steps_, history_.size());
  offset = std::clamp(offset, -max_history + 1, 0);

  // Determine the index in the history buffer that corresponds to the frame
  // index.
  const int idx = (history_cursor_ + offset - 1) % history_.size();
  const mjtNum* state = history_[idx].data();

  // Load the state into the data buffer.
  mj_setState(model_, data_, state, mjSTATE_INTEGRATION);
  mj_forward(model_, data_);
  return offset;
}

}  // namespace mujoco::toolbox
