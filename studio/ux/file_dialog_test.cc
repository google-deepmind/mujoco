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

#include "studio/ux/file_dialog.h"

#include "testing/base/public/gunit.h"

namespace mujoco::studio {
namespace {

TEST(FileDialogTest, FormatFileFilterSingleAndMultipleExtensions) {
  EXPECT_EQ(FormatFileFilter("xml", ' '), "*.xml");
  EXPECT_EQ(FormatFileFilter("xml", ';'), "*.xml");

  EXPECT_EQ(FormatFileFilter("xml,mjb", ' '), "*.xml *.mjb");
  EXPECT_EQ(FormatFileFilter("xml,mjb", ';'), "*.xml;*.mjb");

  EXPECT_EQ(FormatFileFilter("xml,mjb,mjz", ' '), "*.xml *.mjb *.mjz");
  EXPECT_EQ(FormatFileFilter("xml,mjb,mjz", ';'), "*.xml;*.mjb;*.mjz");
}

TEST(FileDialogTest, FormatFileFilterEmptyAndMalformedCommas) {
  EXPECT_EQ(FormatFileFilter("", ' '), "");
  EXPECT_EQ(FormatFileFilter(",", ';'), "");
  EXPECT_EQ(FormatFileFilter(",xml,,mjb,", ' '), "*.xml *.mjb");
  EXPECT_EQ(FormatFileFilter(",xml,,mjb,", ';'), "*.xml;*.mjb");
}

TEST(FileDialogTest, SplitDialogDefaultPathFileInFolder) {
  DialogDefaultPath split = SplitDialogDefaultPath("/home/user/robot_saved.xml");
  EXPECT_EQ(split.folder, "/home/user");
  EXPECT_EQ(split.file_name, "robot_saved.xml");

  split = SplitDialogDefaultPath("C:\\models\\robot_saved.xml");
  EXPECT_EQ(split.folder, "C:\\models");
  EXPECT_EQ(split.file_name, "robot_saved.xml");
}

TEST(FileDialogTest, SplitDialogDefaultPathMixedSeparators) {
  // a Windows folder joined to a file name with '/'
  DialogDefaultPath split =
      SplitDialogDefaultPath("C:\\generated_robot-mjcf/robot_saved.xml", '\\');
  EXPECT_EQ(split.folder, "C:\\generated_robot-mjcf");
  EXPECT_EQ(split.file_name, "robot_saved.xml");

  split = SplitDialogDefaultPath("C:/a\\b/c.mjb", '\\');
  EXPECT_EQ(split.folder, "C:\\a\\b");
  EXPECT_EQ(split.file_name, "c.mjb");

  // without a native separator the folder is returned as given
  split = SplitDialogDefaultPath("C:/a\\b/c.mjb");
  EXPECT_EQ(split.folder, "C:/a\\b");
}

TEST(FileDialogTest, SplitDialogDefaultPathRootsAndBareNames) {
  DialogDefaultPath split = SplitDialogDefaultPath("robot_saved.xml");
  EXPECT_EQ(split.folder, "");
  EXPECT_EQ(split.file_name, "robot_saved.xml");

  split = SplitDialogDefaultPath("/robot_saved.xml");
  EXPECT_EQ(split.folder, "/");
  EXPECT_EQ(split.file_name, "robot_saved.xml");

  split = SplitDialogDefaultPath("C:/robot_saved.xml", '\\');
  EXPECT_EQ(split.folder, "C:\\");
  EXPECT_EQ(split.file_name, "robot_saved.xml");

  split = SplitDialogDefaultPath("C:\\models\\");
  EXPECT_EQ(split.folder, "C:\\models");
  EXPECT_EQ(split.file_name, "");

  split = SplitDialogDefaultPath("");
  EXPECT_EQ(split.folder, "");
  EXPECT_EQ(split.file_name, "");
}

}  // namespace
}  // namespace mujoco::studio
