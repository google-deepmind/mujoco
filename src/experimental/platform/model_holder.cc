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

#include "experimental/platform/model_holder.h"

#include <cstddef>
#include <cstring>
#include <memory>
#include <span>
#include <string_view>

#include <mujoco/mujoco.h>
#include "user/user_resource.h"

namespace mujoco::platform {

struct BufferProvider : public mjpResourceProvider {
  BufferProvider(std::span<const std::byte> buffer) : buffer(buffer) {
    mjp_defaultResourceProvider(this);
    open = [](mjResource* resource) {
      return 1;
    };
    read = [](mjResource* resource, const void** buffer) {
      BufferProvider* self = (BufferProvider*)resource->provider;
      *buffer = self->buffer.data();
      return static_cast<int>(self->buffer.size());
    };
    close = [](mjResource* resource) {};
  }
  std::span<const std::byte> buffer;
};

std::unique_ptr<ModelHolder> ModelHolder::FromSpec(mjSpec* spec) {
  auto mh = std::unique_ptr<ModelHolder>(new ModelHolder());
  mh->InitFromSpec(spec);
  return mh;
}

std::unique_ptr<ModelHolder> ModelHolder::FromFile(std::string_view filepath) {
  auto mh = std::unique_ptr<ModelHolder>(new ModelHolder());
  mh->InitFromFile(filepath);
  return mh;
}

std::unique_ptr<ModelHolder> ModelHolder::FromBuffer(
    std::span<const std::byte> buffer, std::string_view content_type,
    std::string_view filename) {
  auto mh = std::unique_ptr<ModelHolder>(new ModelHolder());
  mh->InitFromBuffer(buffer, content_type, filename);
  return mh;
}

ModelHolder::~ModelHolder() {
  if (data_) {
    mj_deleteData(data_);
  }
  if (model_) {
    mj_deleteModel(model_);
  }
  if (spec_) {
    mj_deleteSpec(spec_);
  }
  mj_deleteVFS(&vfs_);
}

void ModelHolder::PostInit() {
  if (spec_ && !model_) {
    model_ = mj_compile(spec_, &vfs_);
    if (!model_) {
      SetLoadError("Error compiling model from spec.");
      return;
    }
  }
  data_ = mj_makeData(model_);
  if (!data_) {
    SetLoadError("Error making data for model.");
  }
}

void ModelHolder::InitFromSpec(mjSpec* spec) {
  spec_ = spec;
  PostInit();
}

void ModelHolder::InitFromFile(std::string_view filepath) {
  mj_defaultVFS(&vfs_);
  if (filepath.ends_with(".mjb")) {
    model_ = mj_loadModel(filepath.data(), &vfs_);
  } else {
    spec_ = mj_parse(filepath.data(), nullptr, &vfs_, error_, sizeof(error_));
  }
  if (error_[0] == 0) {
    PostInit();
  }
}

void ModelHolder::InitFromBuffer(std::span<const std::byte> buffer,
                                 std::string_view content_type,
                                 std::string_view filename) {
  mj_defaultVFS(&vfs_);

  if (content_type == "text/xml") {
    const char* ptr = reinterpret_cast<const char*>(buffer.data());
    spec_ = mj_parseXMLString(ptr, nullptr, error_, sizeof(error_));
  } else if (content_type == "application/mjb") {
    model_ = mj_loadModelBuffer(buffer.data(), buffer.size());
  } else if (content_type == "application/zip") {
    BufferProvider provider(buffer);
    mjResource resource;
    std::memset(&resource, 0, sizeof(mjResource));
    resource.vfs = &vfs_;
    resource.provider = &provider;
    resource.name = const_cast<char*>(filename.data());
    spec_ = mju_decodeResource(&resource, content_type.data(), &vfs_);
  } else {
    SetLoadError(
        "Unknown content type; expected text/xml or application/mjb");
  }
  if (error_[0] == 0) {
    PostInit();
  }
}

void ModelHolder::SetLoadError(std::string_view error) {
  strncpy(error_, error.data(), sizeof(error_) - 1);
  error_[sizeof(error_) - 1] = 0;
}

}  // namespace mujoco::platform
