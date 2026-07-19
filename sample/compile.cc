// Copyright 2021 DeepMind Technologies Limited
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

#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string_view>

#include <mujoco/mujoco.h>

// help
static constexpr char kHelp[] =
    "\n Usage:  compile infile [outfile]\n"
    "   infile can be in any format with a registered decoder (e.g. mjcf, "
    "urdf) or mjb\n"
    "   outfile can be in mjcf, mjb, txt format, or empty\n\n"
    "   if outfile is empty, compilation will be "
    "timed twice to measure the impact of caching\n\n"
    " Example: compile model.xml [model.mjb]\n";

// deallocate and print message
int finish(const char* msg = 0, int exitcode = EXIT_SUCCESS, mjModel* m = 0,
           mjVFS* vfs = 0) {
  // deallocated everything
  if (m) {
    mj_deleteModel(m);
  }
  if (vfs) {
    mj_deleteVFS(vfs);
  }

  // print message
  if (msg) {
    std::cout << msg << std::endl;
  }

  return exitcode;
}

// check if filename has extension (case-insensitive)
bool HasExtension(std::string_view filename, std::string_view ext) {
  if (filename.length() < ext.length()) return false;

  std::string_view file_ext = filename.substr(filename.length() - ext.length());
  for (std::size_t i = 0; i < ext.length(); ++i) {
    if (std::tolower(static_cast<unsigned char>(file_ext[i])) !=
        std::tolower(static_cast<unsigned char>(ext[i]))) {
      return false;
    }
  }
  return true;
}

// main function
int main(int argc, char** argv) {

  // model and error
  mjModel* m = 0;
  char error[1000];

  // print help if arguments are missing
  if (argc != 3 && argc != 2) {
    return finish(kHelp, EXIT_FAILURE);
  }

  const bool is_mjb = HasExtension(argv[1], ".mjb");

  if (is_mjb && argc == 3 && HasExtension(argv[2], ".xml")) {
    return finish("Illegal combination: cannot save binary model to XML",
                  EXIT_FAILURE);
  }

  // check if output file exists
  if (argc == 3) {
    std::FILE* fp = std::fopen(argv[2], "r");
    if (fp) {
      std::fclose(fp);
      std::cout << "Output file already exists, overwrite? (Y/n) ";
      char c;
      std::cin >> c;
      if (c != 'y' && c != 'Y') {
        return finish();
      }
    }
  }

  // enable compile timing diagnostics
  if (argc == 2) {
    mjLogConfig config = mju_getLogConfig();
    config.logfile[0] = '\0';
    config.topics |= (1 << (mjTOPIC_TIME_CMP - 1));
    mju_setLogConfig(config);
  }

  // load model
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mjSpec* spec = nullptr;

  if (is_mjb) {
    m = mj_loadModel(argv[1], &vfs);
    if (!m) {
      return finish("Could not load binary model", EXIT_FAILURE, nullptr, &vfs);
    }
  } else {
    spec = mj_parse(argv[1], nullptr, &vfs, error, 1000);
    if (!spec) {
      return finish(error, EXIT_FAILURE, nullptr, &vfs);
    }

    if (argc == 2) {
      std::cout << "Compile 1 (cold cache)\n";
    }
    m = mj_compile(spec, &vfs);
    if (!m) {
      auto err_msg = mjs_getError(spec);
      mj_deleteSpec(spec);
      return finish(err_msg, EXIT_FAILURE, nullptr, &vfs);
    }

    if (argc == 2) {
      mj_deleteModel(m);
      std::cout << "Compile 2 (warm cache)\n";
      m = mj_compile(spec, &vfs);
    }
  }

  // check error
  if (!m) {
    if (spec) {
      auto err_msg = mjs_getError(spec);
      mj_deleteSpec(spec);
      return finish(err_msg, EXIT_FAILURE, nullptr, &vfs);
    } else {
      return finish("Could not load model", EXIT_FAILURE, nullptr, &vfs);
    }
  }

  // encode output
  if (argc == 3) {
    if (mj_encode(spec, m, argv[2], nullptr, &vfs, error, 1000) < 0) {
      if (spec) mj_deleteSpec(spec);
      return finish(error, EXIT_FAILURE, m, &vfs);
    }
  }

  // finalize
  if (spec) mj_deleteSpec(spec);
  return finish("Done.", EXIT_SUCCESS, m, &vfs);
}
