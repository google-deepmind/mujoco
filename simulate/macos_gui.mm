// Copyright 2022 DeepMind Technologies Limited
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

std::string GetSavePath(const char* filename) {
  NSSavePanel* panel = [NSSavePanel savePanel];
  NSURL* userDocumentsDir = [NSFileManager.defaultManager URLsForDirectory:NSDocumentDirectory
                                                          inDomains:NSUserDomainMask].firstObject;
  [panel setDirectoryURL:userDocumentsDir];
  [panel setNameFieldStringValue:[NSString stringWithUTF8String:filename]];
  if ([panel runModal] == NSModalResponseOK) {
    std::ostringstream s;
    s << [panel.URL.path cStringUsingEncoding:NSUTF8StringEncoding];
    return s.str();
  } else {
    return "";
  }
}

#ifdef __AVX__
void DisplayErrorDialogBox(const char* title, const char* msg) {
  NSAlert *alert = [[[NSAlert alloc] init] autorelease];
  [alert setMessageText:[NSString stringWithUTF8String:title]];
  [alert setInformativeText:[NSString stringWithUTF8String:msg]];
  [alert setAlertStyle:NSAlertStyleCritical];
  [alert addButtonWithTitle:@"Exit"];
  [alert runModal];
}
#endif
