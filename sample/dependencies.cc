// Copyright 2025 DeepMind Technologies Limited
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

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <mujoco/mujoco.h>

// help
static constexpr char helpstring[] =
  "\n Usage:  list_dependencies infile\n"
  "   infile must be in MJCF\n"
  " Example: list_dependencies model.xml\n";


// main function
int main(int argc, char** argv) {

  // print help if arguments are missing
  if (argc!=3 && argc!=2) {
    std::cout << helpstring << std::endl;
    return EXIT_SUCCESS;
  }

  mjStringVec dependencies;
  mju_getXMLDependencies(argv[1], &dependencies);
  std::sort(dependencies.begin(), dependencies.end());

  std::cout << "Dependencies:" << std::endl;
  for (int i = 0; i < dependencies.size(); ++i) {
    std::cout << "\t " << dependencies[i] << std::endl;
  }
  return EXIT_SUCCESS;
}
