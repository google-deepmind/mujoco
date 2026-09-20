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

#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wunused-function"
#elif defined(__GNUC__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include <miniz.h>
#if defined(__clang__)
  #pragma clang diagnostic pop
#elif defined(__GNUC__)
  #pragma GCC diagnostic pop
#endif

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "user/user_resource.h"
#include "user/user_util.h"

namespace {

struct FileInfo {
  int index = 0;
  int size  = 0;
};

struct ZipArchiveHandle {
  std::string                               name;
  mz_zip_archive                            archive;
  std::vector<char>                         raw_buffer;
  std::unordered_map<std::string, FileInfo> files;
  mutable std::mutex                        mutex;
  bool                                      is_valid = false;

  ZipArchiveHandle() = default;
  ~ZipArchiveHandle() {
    if (is_valid) { mz_zip_reader_end(&archive); }
  }

  ZipArchiveHandle(const ZipArchiveHandle&)            = delete;
  ZipArchiveHandle& operator=(const ZipArchiveHandle&) = delete;

  bool Contains(std::string_view filename) const {
    return files.find(std::string(filename)) != files.end();
  }
};

// Context for a resource representing a member in an mjz archive.
struct MemberContext {
  ZipArchiveHandle* handle = nullptr;
  std::string       member_path;
  std::vector<char> buffer;
};

static int ZipMount(mjResource* resource) {
  if (!resource || !resource->name) { return 0; }

  auto handle  = std::make_unique<ZipArchiveHandle>();
  handle->name = mujoco::user::FilePath(resource->name).Str();
  std::memset(&handle->archive, 0, sizeof(handle->archive));

  std::error_code ec;
  if (std::filesystem::is_regular_file(resource->name, ec)) {
    if (!mz_zip_reader_init_file(&handle->archive, resource->name, 0)) { return 0; }
  } else {
    mjResource* raw_res = mju_openResource("", resource->name, resource->vfs, nullptr, 0);
    const char* buffer  = nullptr;
    int         size    = -1;
    if (raw_res) { size = mju_readResource(raw_res, (const void**)&buffer); }

    if (size <= 0 || !buffer) {
      if (raw_res) { mju_closeResource(raw_res); }
      return 0;
    }

    handle->raw_buffer.assign(buffer, buffer + size);
    mju_closeResource(raw_res);

    if (!mz_zip_reader_init_mem(&handle->archive,
                                handle->raw_buffer.data(),
                                handle->raw_buffer.size(),
                                0)) {
      return 0;
    }
  }

  const int num_files = mz_zip_reader_get_num_files(&handle->archive);
  for (int i = 0; i < num_files; ++i) {
    mz_zip_archive_file_stat stat;
    if (!mz_zip_reader_file_stat(&handle->archive, i, &stat)) { return 0; }
    if (stat.m_uncomp_size > 0) {
      handle->files[stat.m_filename] = FileInfo{i, static_cast<int>(stat.m_uncomp_size)};
    }
  }

  handle->is_valid = true;
  resource->data   = handle.release();
  return 1;
}

static std::optional<std::string> GetMemberPath(const mjResource* resource,
                                                std::string_view  archive_name) {
  if (!resource || !resource->name) { return std::nullopt; }

  std::string fullpath = mujoco::user::FilePath(resource->name).Str();
  if (fullpath == archive_name) { return ""; }
  if (fullpath.starts_with(archive_name) &&
      fullpath.size() > archive_name.size() &&
      (fullpath[archive_name.size()] == '/' || fullpath[archive_name.size()] == '\\')) {
    std::string_view sub = std::string_view(fullpath).substr(archive_name.size());
    while (!sub.empty() && (sub.front() == '/' || sub.front() == '\\')) { sub.remove_prefix(1); }
    return std::string(sub);
  }

  return std::nullopt;
}

static int ZipOpen(mjResource* resource) {
  if (!resource || !resource->name || !resource->data) { return 0; }

  auto* handle = static_cast<ZipArchiveHandle*>(resource->data);

  auto member = GetMemberPath(resource, handle->name);
  if (!member.has_value()) { return 0; }

  if (!member->empty()) {
    std::lock_guard<std::mutex> lock(handle->mutex);
    if (!handle->Contains(*member)) { return 0; }
  }

  auto* ctx        = new MemberContext();
  ctx->handle      = handle;
  ctx->member_path = std::move(*member);
  resource->data   = ctx;
  return 1;
}

static int ZipRead(mjResource* resource, const void** buffer) {
  if (!resource || !resource->data || !buffer) { return -1; }

  auto* ctx    = static_cast<MemberContext*>(resource->data);
  auto* handle = ctx->handle;
  if (!handle || ctx->member_path.empty()) { return -1; }

  std::lock_guard<std::mutex> lock(handle->mutex);
  auto                        it = handle->files.find(ctx->member_path);
  if (it == handle->files.end()) { return -1; }

  if (ctx->buffer.empty() && it->second.size > 0) {
    ctx->buffer.resize(it->second.size);
    if (!mz_zip_reader_extract_to_mem(&handle->archive,
                                      it->second.index,
                                      ctx->buffer.data(),
                                      ctx->buffer.size(),
                                      0)) {
      ctx->buffer.clear();
      return -1;
    }
  }

  *buffer = ctx->buffer.data();
  return static_cast<int>(ctx->buffer.size());
}

static void ZipClose(mjResource* resource) {
  if (!resource || !resource->data) { return; }
  auto* ctx = static_cast<MemberContext*>(resource->data);
  delete ctx;
  resource->data = nullptr;
}

static int ZipUnmount(mjResource* resource) {
  if (!resource || !resource->data) { return 1; }
  auto* handle = static_cast<ZipArchiveHandle*>(resource->data);
  delete handle;
  resource->data = nullptr;
  return 1;
}

}  // namespace

mjPLUGIN_LIB_INIT(mjz_archive_provider) {
  mjpResourceProvider provider;
  mjp_defaultResourceProvider(&provider);
  provider.prefix  = ".mjz|.zip";
  provider.open    = ZipOpen;
  provider.read    = ZipRead;
  provider.close   = ZipClose;
  provider.mount   = ZipMount;
  provider.unmount = ZipUnmount;
  mjp_registerArchiveResourceProvider(&provider);
}
