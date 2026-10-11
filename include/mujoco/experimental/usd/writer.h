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

#ifndef MUJOCO_SRC_EXPERIMENTAL_USD_WRITER_H_
#define MUJOCO_SRC_EXPERIMENTAL_USD_WRITER_H_

#include <functional>
#include <vector>

#include <mujoco/mujoco.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>

namespace mujoco {
namespace usd {
using PoseSinkFn =
    std::function<void(const mjData* const, std::vector<pxr::SdfPath>)>;

class Writer {
 public:
  Writer(pxr::UsdStageRefPtr stage, mjSpec* spec, mjModel_* model);
  ~Writer();
  Writer(const Writer&) = delete;
  Writer& operator=(const Writer&) = delete;

  void Update(const mjData* const data);
  void AddSink(PoseSinkFn sink_fn);

 private:
  void WriteSpecsWithoutUSDOriginToLayer();
  void BuildMjUsdMapping();

  pxr::UsdStageRefPtr stage_;
  std::vector<pxr::SdfPath> body_id_to_path_;
  mjSpec* spec_ = nullptr;
  mjModel_* model_ = nullptr;
  std::vector<PoseSinkFn> pose_sinks_;
};
}  // namespace usd
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_USD_WRITER_H_
