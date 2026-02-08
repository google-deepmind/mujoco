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

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <string_view>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

#include <miniz_zip.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "user/user_resource.h"

static void mjPRINTFLIKE(3, 4)
    SetError(char* error, int error_sz, const char* format, ...) {
  if (error) {
    va_list args;
    va_start(args, format);
    std::vsnprintf(error, error_sz, format, args);
    va_end(args);
  }
}

// A mjpResourceProvider that reads files from a zip archive.
//
// The zip archive itself is provided as a byte buffer in the constructor. This
// class can then be used to lazily read the contents of the individual files
// from within the archive as needed.
class ZipArchiveProvider : public mjpResourceProvider {
 public:
  ZipArchiveProvider(std::string name, const void* buffer, int nbuffer,
                     char* error, int error_sz)
      : name_(std::move(name)), buffer_((char*)buffer, (char*)buffer + nbuffer) {
    mjp_defaultResourceProvider(this);

    std::memset(&archive_, 0, sizeof(archive_));
    if (!mz_zip_reader_init_mem(&archive_, buffer_.data(), nbuffer, 0)) {
      SetError(error, error_sz, "Zip error: invalid zip archive");
      return;
    }

    // Create an index of the files in the archive.
    const int num_files = mz_zip_reader_get_num_files(&archive_);
    for (int i = 0; i < num_files; ++i) {
      mz_zip_archive_file_stat stat;
      if (!mz_zip_reader_file_stat(&archive_, i, &stat)) {
        SetError(error, error_sz, "Zip error: failed to stat item %d.", i);
        files_.clear();
        return;
      }
      const int size = static_cast<int>(stat.m_uncomp_size);
      if (size == 0) {
        continue;
      }
      files_[stat.m_filename] = FileInfo{i, size, {}};
    }

    // Look for the root XML model in the archive. First look for an XML file
    // with the same name as the archive itself. Failing that, look for an XML
    // file with the same name including the parent directory.
    const std::filesystem::path path(name_);
    root_model_ = (path / path.stem()).string() + ".xml";
    if (!Contains(root_model_)) {
      root_model_ = (path / path.parent_path() / path.stem()).string() + ".xml";
      if (!Contains(root_model_)) {
        SetError(error, error_sz, "Zip error: no root XML file found.");
        return;
      }
    }

    // Setup mjpResourceProvider callbacks.
    mount = [](mjResource* resource) {
      return 0;
    };
    unmount = [](mjResource* resource) {
      ZipArchiveProvider* self = (ZipArchiveProvider*)resource->provider;
      delete self;
      return 0;
    };
    open = [](mjResource* resource) {
      ZipArchiveProvider* self = (ZipArchiveProvider*)resource->provider;
      const bool found = self->Contains(resource->name);
      return found ? 1 : 0;
    };
    read = [](mjResource* resource, const void** buffer) {
      ZipArchiveProvider* self = (ZipArchiveProvider*)resource->provider;
      std::span<char> bytes = self->Read(resource->name);
      *buffer = bytes.data();
      return static_cast<int>(bytes.size());
    };
    close = [](mjResource* resource) {
      // no-op
    };
  }

  ~ZipArchiveProvider() {
    mz_zip_reader_end(&archive_);
  }

  ZipArchiveProvider(const ZipArchiveProvider&) = delete;
  ZipArchiveProvider& operator=(const ZipArchiveProvider&) = delete;

  // Returns the path to the root XML model in the archive.
  std::string GetRootModelPath() const {
    return root_model_;
  }

  // Returns true if the archive contains a file with the given name/path.
  bool Contains(std::string_view name) const {
    const std::string_view filename = name.substr(name_.size() + 1);
    return files_.find(filename.data()) != files_.end();
  }

  // Reads the contents of the file with the given name/path. The contents are
  // cached internally so that subsequent reads for the same file do not need to
  // re-read the file from the archive.
  std::span<char> Read(const std::string& name) {
    const std::string filename = name.substr(name_.size() + 1);
    auto it = files_.find(filename);
    if (it == files_.end()) {
      return {};
    }

    FileInfo& info = it->second;

    // Lazily read and store the file contents from the archive.
    if (info.contents.empty()) {
      info.contents.resize(info.size);
      if (!mz_zip_reader_extract_to_mem(&archive_, info.index,
                                        info.contents.data(), info.size, 0)) {
        return {};
      }
    }
    return info.contents;
  }

 private:
  struct FileInfo {
    // Index of the file in the archive.
    int index = 0;

    // Size (in bytes) of the uncompressed file
    int size = 0;

    // Contents of the uncompressed file.
    std::vector<char> contents;
  };

  std::string name_;
  std::string root_model_;
  mz_zip_archive archive_;
  std::vector<char> buffer_;
  std::unordered_map<std::string, FileInfo> files_;
};

static mjSpec* ParseZipBuffer(const void* buffer, int nbuffer, const char* name,
                              mjVFS* vfs, char* error, int error_sz) {
  if (error) {
    error[0] = 0;
  }

  ZipArchiveProvider* provider =
      new ZipArchiveProvider(name, buffer, nbuffer, error, error_sz);
  if (error && error[0]) {
    return nullptr;
  }

  const int status = mj_mountVFS(vfs, name, provider);
  if (status != 0) {
    SetError(error, error_sz, "Failed to mount zip archive: %s", name);
    return nullptr;
  }

  const std::string root = provider->GetRootModelPath();
  return mj_parseXML(root.c_str(), vfs, error, error_sz);
}

mjPLUGIN_LIB_INIT {
  mjpDecoder decoder;
  decoder.content_type = "application/zip";
  decoder.extension = ".mjz|.zip";
  decoder.can_decode = +[](const mjResource* resource) {
    const char* ext = strrchr(resource->name, '.');
    return ext ? (!strcmp(ext, ".mjz") || !strcmp(ext, ".zip")) : 0;
  };
  decoder.decode = +[](mjResource* resource, const mjVFS* vfs) -> mjSpec* {
    const char* buffer = nullptr;
    const int size = mju_readResource(resource, (const void**)&buffer);
    if (size <= 0) {
      return nullptr;
    }
    return ParseZipBuffer(buffer, size, resource->name, const_cast<mjVFS*>(vfs),
                          nullptr, 0);
  };
  mjp_registerDecoder(&decoder);
}
