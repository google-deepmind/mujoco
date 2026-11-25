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

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mujoco::studio {

static DialogResult RunZenity(std::vector<std::string>& args) {
  int pipe_fds[2];
  if (pipe(pipe_fds) != 0) {
    return DialogResult{.status = DialogResult::kError};
  }
  fcntl(pipe_fds[0], F_SETFD, FD_CLOEXEC);
  fcntl(pipe_fds[1], F_SETFD, FD_CLOEXEC);

  std::string program = "zenity";
  std::vector<char*> argv;
  argv.reserve(args.size() + 2);
  argv.push_back(program.data());
  for (std::string& arg : args) {
    argv.push_back(arg.data());
  }
  argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    close(pipe_fds[0]);
    close(pipe_fds[1]);
    return DialogResult{.status = DialogResult::kError};
  }
  if (pid == 0) {
    close(pipe_fds[0]);
    if (pipe_fds[1] != STDOUT_FILENO) {
      while (dup2(pipe_fds[1], STDOUT_FILENO) == -1) {
        if (errno != EINTR) {
          _exit(127);
        }
      }
      close(pipe_fds[1]);
    } else if (fcntl(STDOUT_FILENO, F_SETFD, 0) == -1) {
      _exit(127);
    }
    execvp(argv[0], argv.data());
    _exit(127);
  }

  close(pipe_fds[1]);
  std::string output;
  char buffer[4096];
  bool read_error = false;
  while (true) {
    const ssize_t bytes_read = read(pipe_fds[0], buffer, sizeof(buffer));
    if (bytes_read > 0) {
      output.append(buffer, static_cast<size_t>(bytes_read));
    } else if (bytes_read == 0) {
      break;
    } else if (errno != EINTR) {
      read_error = true;
      break;
    }
  }
  close(pipe_fds[0]);

  int status = 0;
  pid_t waited = -1;
  do {
    waited = waitpid(pid, &status, 0);
  } while (waited == -1 && errno == EINTR);

  if (read_error || waited == -1 || !WIFEXITED(status)) {
    return DialogResult{.status = DialogResult::kError};
  }
  const int exit_code = WEXITSTATUS(status);
  if (exit_code == 1) {
    return DialogResult{.status = DialogResult::kCancelled};
  }
  if (exit_code != 0) {
    return DialogResult{.status = DialogResult::kError};
  }
  // Zenity outputs a trailing newline which we need to remove.
  if (!output.empty() && output.back() == '\n') {
    output.pop_back();
  }
  return DialogResult{.status = DialogResult::kAccepted, .path = std::move(output)};
}

static void UpdateArgs(std::vector<std::string>& args, std::string_view path,
                       std::span<std::string_view> filters = {}) {
  if (!path.empty() && path.front() != '\0') {
    args.push_back("--filename=" + std::string(path));
  }
  for (std::string_view filter : filters) {
    std::string pattern = FormatFileFilter(filter, ' ');
    if (!pattern.empty()) {
      args.push_back("--file-filter=" + pattern);
    }
  }
}

DialogResult OpenFileDialog(std::string_view path, std::span<std::string_view> filters) {
  std::vector<std::string> args;
  args.push_back("--file-selection");
  args.push_back("--title=Open File");
  UpdateArgs(args, path, filters);
  return RunZenity(args);
}

DialogResult SaveFileDialog(std::string_view path, std::span<std::string_view> filters) {
  std::vector<std::string> args;
  args.push_back("--file-selection");
  args.push_back("--title=Save File");
  args.push_back("--save");
  UpdateArgs(args, path, filters);
  return RunZenity(args);
}

DialogResult SelectPathDialog(std::string_view path) {
  std::vector<std::string> args;
  args.push_back("--file-selection");
  args.push_back("--title=Select Folder");
  args.push_back("--directory");
  UpdateArgs(args, path);
  return RunZenity(args);
}

}  // namespace mujoco::studio
