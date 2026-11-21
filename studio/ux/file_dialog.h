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

#ifndef MUJOCO_STUDIO_UX_FILE_DIALOG_H_
#define MUJOCO_STUDIO_UX_FILE_DIALOG_H_

#include <cstddef>
#include <span>
#include <string>
#include <string_view>

namespace mujoco::studio {

// Formats a comma-separated extension list (e.g. "xml,mjb") as wildcard
// patterns joined by `separator` (e.g. "*.xml *.mjb" or "*.xml;*.mjb").
inline std::string FormatFileFilter(std::string_view filter, char separator) {
  std::string result;
  while (!filter.empty()) {
    const size_t pos = filter.find(',');
    const std::string_view ext = filter.substr(0, pos);
    if (!ext.empty()) {
      if (!result.empty()) {
        result += separator;
      }
      result += "*.";
      result.append(ext);
    }
    if (pos == std::string_view::npos) {
      break;
    }
    filter.remove_prefix(pos + 1);
  }
  return result;
}

// A default dialog path split into the folder to open and the file name to
// prefill. Either part may be empty.
struct DialogDefaultPath {
  std::string folder;
  std::string file_name;
};

// Splits a default dialog path at its last '/' or '\' separator. If
// `native_separator` is not zero, every separator in the folder is replaced
// by it, so that a path built with mixed separators (e.g. "C:\dir/file.xml")
// can be passed to platform APIs that only accept one kind.
inline DialogDefaultPath SplitDialogDefaultPath(std::string_view path,
                                                char native_separator = 0) {
  DialogDefaultPath result;
  const size_t pos = path.find_last_of("/\\");
  if (pos == std::string_view::npos) {
    result.file_name = std::string(path);
    return result;
  }
  // keep the separator of a root folder ("/" or "C:\"), drop any other
  const bool is_root = pos == 0 || (pos == 2 && path[1] == ':');
  result.folder = std::string(path.substr(0, is_root ? pos + 1 : pos));
  result.file_name = std::string(path.substr(pos + 1));
  if (native_separator) {
    for (char& c : result.folder) {
      if (c == '/' || c == '\\') {
        c = native_separator;
      }
    }
  }
  return result;
}

// Result returned by various file dialog functions.
struct DialogResult {
  // The status of the dialog.
  enum Status {
    // The dialog is still open.
    kPending,
    // A file/folder was selected.
    kAccepted,
    // The dialog was dismissed.
    kCancelled,
    // An error occurred.
    kError,
  };

  // The status of the dialog.
  Status status = kPending;

  // The path selected by the dialog if Status is kAccepted. If status is
  // kError, the path may contain an error message. Empty otherwise.
  std::string path;
};

// Opens a file dialog used for selecting a file to open.
DialogResult OpenFileDialog(std::string_view path, std::span<std::string_view> filters = {});

// Opens a file dialog used for selecting a file to save.
DialogResult SaveFileDialog(std::string_view path, std::span<std::string_view> filters = {});

// Opens a file dialog used for selecting a folder.
DialogResult SelectPathDialog(std::string_view path);

}  // namespace mujoco::studio

#endif  // MUJOCO_STUDIO_UX_FILE_DIALOG_H_
