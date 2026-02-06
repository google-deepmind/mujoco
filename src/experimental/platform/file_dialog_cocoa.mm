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

#include <sstream>
#include <string>

#include <Cocoa/Cocoa.h>
#include "experimental/platform/file_dialog.h"

namespace mujoco::platform {

DialogResult OpenFileDialog(std::string_view path,
                            std::span<std::string_view> filters) {
  NSOpenPanel* panel = [NSOpenPanel openPanel];
  NSURL* userDocumentsDir = [NSFileManager.defaultManager URLsForDirectory:NSDocumentDirectory
                                                          inDomains:NSUserDomainMask].firstObject;
  [panel setDirectoryURL:userDocumentsDir];
  [panel setNameFieldStringValue:[NSString stringWithUTF8String:path.data()]];

  DialogResult result;
  if ([panel runModal] == NSModalResponseOK) {
    std::ostringstream s;
    s << [panel.URL.path cStringUsingEncoding:NSUTF8StringEncoding];
    result.path = s.str();
    result.status = DialogResult::kAccepted;
  } else {
    result.status = DialogResult::kCancelled;
  }
  return result;
}

DialogResult SaveFileDialog(std::string_view path,
                            std::span<std::string_view> filters) {
  NSSavePanel* panel = [NSSavePanel savePanel];
  NSURL* userDocumentsDir = [NSFileManager.defaultManager URLsForDirectory:NSDocumentDirectory
                                                          inDomains:NSUserDomainMask].firstObject;
  [panel setDirectoryURL:userDocumentsDir];
  [panel setNameFieldStringValue:[NSString stringWithUTF8String:path.data()]];

  DialogResult result;
  if ([panel runModal] == NSModalResponseOK) {
    std::ostringstream s;
    s << [panel.URL.path cStringUsingEncoding:NSUTF8StringEncoding];
    result.path = s.str();
    result.status = DialogResult::kAccepted;
  } else {
    result.status = DialogResult::kCancelled;
  }
  return result;
}

DialogResult SelectPathDialog(std::string_view path) {
  return OpenFileDialog(path);
}

}  // namespace mujoco::platform
