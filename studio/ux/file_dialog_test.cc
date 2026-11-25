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

#ifdef __linux__
  #include <sys/stat.h>

  #include <cstdlib>
  #include <filesystem>
  #include <fstream>
  #include <optional>
  #include <string>
  #include <string_view>
  #include <vector>
#endif

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

#ifdef __linux__

class ScopedEnvVar {
 public:
  ScopedEnvVar(const char* name, const std::string& value) : name_(name) {
    if (const char* prev = std::getenv(name_)) {
      previous_ = prev;
    }
    setenv(name_, value.c_str(), 1);
  }

  ~ScopedEnvVar() {
    if (previous_.has_value()) {
      setenv(name_, previous_->c_str(), 1);
    } else {
      unsetenv(name_);
    }
  }

 private:
  const char* name_;
  std::optional<std::string> previous_;
};

std::filesystem::path CreateFakeZenityDir(const std::string& name) {
  const std::filesystem::path dir = std::filesystem::path(::testing::TempDir()) / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  const std::filesystem::path script = dir / "zenity";
  {
    std::ofstream out(script);
    out << "#!/bin/sh\n"
        << "printf '%s\\n' \"$@\" > \"$TEST_ZENITY_ARGS_FILE\"\n"
        << "if [ -n \"$TEST_ZENITY_STDOUT\" ]; then\n"
        << "  printf '%s\\n' \"$TEST_ZENITY_STDOUT\"\n"
        << "fi\n"
        << "exit \"${TEST_ZENITY_EXIT_CODE:-0}\"\n";
  }
  chmod(script.c_str(), 0755);
  return dir;
}

std::vector<std::string> ReadLines(const std::filesystem::path& path) {
  std::vector<std::string> lines;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    lines.push_back(line);
  }
  return lines;
}

TEST(FileDialogTest, ZenityDoesNotInvokeShellOnMetacharacters) {
  const std::filesystem::path bin_dir = CreateFakeZenityDir("zenity_metachars");
  const std::filesystem::path args_file = bin_dir / "args.txt";
  const std::filesystem::path canary_cmd = bin_dir / "canary_cmd";
  const std::filesystem::path canary_bt = bin_dir / "canary_bt";
  const std::filesystem::path canary_semi = bin_dir / "canary_semi";

  const std::string malicious_path = "/tmp/dir with spaces/model$(touch " + canary_cmd.string() +
                                     ")`touch " + canary_bt.string() + "`;touch " +
                                     canary_semi.string() + ".xml";
  const std::string long_output = "/tmp/" + std::string(5000, 'a') + "/selected.xml";

  const char* old_path = std::getenv("PATH");
  const std::string new_path = bin_dir.string() + (old_path ? ":" + std::string(old_path) : "");
  ScopedEnvVar path_guard("PATH", new_path);
  ScopedEnvVar args_guard("TEST_ZENITY_ARGS_FILE", args_file.string());
  ScopedEnvVar stdout_guard("TEST_ZENITY_STDOUT", long_output);
  ScopedEnvVar exit_guard("TEST_ZENITY_EXIT_CODE", "0");

  std::string_view filters[] = {"xml,mjb"};
  const DialogResult result = OpenFileDialog(malicious_path, filters);

  EXPECT_EQ(result.status, DialogResult::kAccepted);
  EXPECT_EQ(result.path, long_output);
  EXPECT_FALSE(std::filesystem::exists(canary_cmd));
  EXPECT_FALSE(std::filesystem::exists(canary_bt));
  EXPECT_FALSE(std::filesystem::exists(canary_semi));

  const std::vector<std::string> expected_args = {
      "--file-selection",
      "--title=Open File",
      "--filename=" + malicious_path,
      "--file-filter=*.xml *.mjb",
  };
  EXPECT_EQ(ReadLines(args_file), expected_args);
}

TEST(FileDialogTest, ZenityHandlesCancelErrorAndMissingBinary) {
  const std::filesystem::path bin_dir = CreateFakeZenityDir("zenity_status");
  const std::filesystem::path args_file = bin_dir / "args.txt";

  const char* old_path = std::getenv("PATH");
  const std::string new_path = bin_dir.string() + (old_path ? ":" + std::string(old_path) : "");
  {
    ScopedEnvVar path_guard("PATH", new_path);
    ScopedEnvVar args_guard("TEST_ZENITY_ARGS_FILE", args_file.string());
    ScopedEnvVar stdout_guard("TEST_ZENITY_STDOUT", "");
    ScopedEnvVar exit_guard("TEST_ZENITY_EXIT_CODE", "1");

    const DialogResult cancelled = SaveFileDialog("/tmp/model.xml");
    EXPECT_EQ(cancelled.status, DialogResult::kCancelled);
    const std::vector<std::string> expected_save_args = {
        "--file-selection",
        "--title=Save File",
        "--save",
        "--filename=/tmp/model.xml",
    };
    EXPECT_EQ(ReadLines(args_file), expected_save_args);
  }

  {
    ScopedEnvVar path_guard("PATH", new_path);
    ScopedEnvVar args_guard("TEST_ZENITY_ARGS_FILE", args_file.string());
    ScopedEnvVar stdout_guard("TEST_ZENITY_STDOUT", "");
    ScopedEnvVar exit_guard("TEST_ZENITY_EXIT_CODE", "2");

    const DialogResult failed = SelectPathDialog("/tmp/dir");
    EXPECT_EQ(failed.status, DialogResult::kError);
    const std::vector<std::string> expected_dir_args = {
        "--file-selection",
        "--title=Select Folder",
        "--directory",
        "--filename=/tmp/dir",
    };
    EXPECT_EQ(ReadLines(args_file), expected_dir_args);
  }

  {
    const std::filesystem::path empty_dir =
        std::filesystem::path(::testing::TempDir()) / "zenity_empty_path";
    std::filesystem::create_directories(empty_dir);
    ScopedEnvVar path_guard("PATH", empty_dir.string());

    const DialogResult missing = OpenFileDialog("/tmp/model.xml");
    EXPECT_EQ(missing.status, DialogResult::kError);
  }
}

#endif  // __linux__

}  // namespace
}  // namespace mujoco::studio
