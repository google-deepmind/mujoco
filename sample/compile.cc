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

#include <chrono>
#include <cctype>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <mujoco/mujoco.h>


// help
static constexpr char helpstring[] =
  "\n Usage:  compile infile [outfile]\n"
  "   infile can be in mjcf, urdf, mjb format\n"
  "   outfile can be in mjcf, mjb, txt format\n\n"
  "   if infile is mjcf or urdf and outfile is omitted, a detailed\n"
  "   timing breakdown is printed for two compilations (cold and warm cache)\n\n"
  " Example: compile model.xml [model.mjb]\n";


// timer (seconds)
double gettm(void) {
  using Clock = std::chrono::steady_clock;
  using Seconds = std::chrono::duration<double>;
  static const Clock::time_point tm_start = Clock::now();
  return Seconds(Clock::now() - tm_start).count();
}


// deallocate and print message
int finish(const char* msg = 0, int exitcode = EXIT_SUCCESS, mjModel* m = 0) {
  // deallocated everything
  if (m) {
    mj_deleteModel(m);
  }

  // print message
  if (msg) {
    std::cout << msg << std::endl;
  }

  return exitcode;
}


// possible file types
enum {
  typeUNKNOWN = 0,
  typeXML,
  typeMJB,
  typeTXT,
  typeNONE
};


// determine file type
int filetype(const char* filename) {
  // convert to lower case for string comparison
  char lower[1000];
  std::size_t i=0;
  while (i<std::strlen(filename) && i<999) {
    lower[i] = (char)tolower(filename[i]);
    i++;
  }
  lower[i] = 0;

  // find last dot
  int dot = (int)std::strlen(lower);
  while (dot>=0 && lower[dot]!='.') {
    dot--;
  }

  // no dot found
  if (dot<0) {
    return typeUNKNOWN;
  }

  // check extension
  if (!std::strcmp(lower+dot, ".xml") || !std::strcmp(lower+dot, ".urdf")) {
    return typeXML;
  } else if (!std::strcmp(lower+dot, ".mjb")) {
    return typeMJB;
  } else if (!std::strcmp(lower+dot, ".txt")) {
    return typeTXT;
  } else {
    return typeUNKNOWN;
  }
}


// main function
int main(int argc, char** argv) {

  // model and error
  mjModel* m = 0;
  char error[1000];

  // print help if arguments are missing
  if (argc!=3 && argc!=2) {
    return finish(helpstring, EXIT_FAILURE);
  }

  // determine file types
  int type1 = filetype(argv[1]);
  int type2 = argc==2 ? typeNONE : filetype(argv[2]);

  // check types
  if (type1 == typeUNKNOWN || type1 == typeTXT ||
      type2 == typeUNKNOWN || (type1 == typeMJB && type2 == typeXML)) {
    return finish("Illegal combination of file formats", EXIT_FAILURE);
  }

  // check if output file exists
  std::FILE* fp = std::fopen(argv[2], "r");
  if (fp) {
    std::cout << "Output file already exists, overwrite? (Y/n) ";
    char c;
    std::cin >> c;
    if (c!='y' && c!='Y') {
      std::fclose(fp);
      return finish();
    }
  }

  // print compiler timing diagnostics
  auto print_timers = [](const mjSpec* s, const char* label) {
    const double* timer = mjs_getTimer(const_cast<mjSpec*>(s));
    std::printf("\n%s:\n", label);
    std::printf("  total:   %8.1f ms\n", 1e3 * timer[mjCTIMER_TOTAL]);
    std::printf("  assets:  %8.1f ms (wall clock)\n", 1e3 * timer[mjCTIMER_ASSETS]);
    std::printf("    load:  %8.1f ms\n", 1e3 * timer[mjCTIMER_MESH_LOAD]);
    std::printf("    hull:  %8.1f ms\n", 1e3 * timer[mjCTIMER_MESH_HULL]);
    std::printf("    poly:  %8.1f ms\n", 1e3 * timer[mjCTIMER_MESH_POLYGON]);
    std::printf("    inert: %8.1f ms\n", 1e3 * timer[mjCTIMER_MESH_INERTIA]);
    std::printf("    bvh:   %8.1f ms\n", 1e3 * timer[mjCTIMER_MESH_BVH]);
    std::printf("    octr:  %8.1f ms\n", 1e3 * timer[mjCTIMER_MESH_OCTREE]);
    std::printf("    tex:   %8.1f ms\n", 1e3 * timer[mjCTIMER_TEXTURE]);
    std::printf("  other:   %8.1f ms\n",
                1e3 * (timer[mjCTIMER_TOTAL] - timer[mjCTIMER_ASSETS]));
  };

  // load model
  mjSpec* s = nullptr;
  if (type1==typeXML) {
    s = mj_parseXML(argv[1], 0, error, 1000);
    if (!s) {
      return finish(error, EXIT_FAILURE);
    }

    m = mj_compile(s, 0);
    if (!m) {
      mj_deleteSpec(s);
      return finish("Could not compile model", EXIT_FAILURE);
    }

    print_timers(s, "Compile 1 (cold cache)");

    if (type2 == typeNONE) {
      mj_deleteModel(m);
      m = mj_compile(s, 0);
      if (m) {
        print_timers(s, "Compile 2 (warm cache)");
      }
    }
  } else {
    m = mj_loadModel(argv[1], 0);
  }

  // check error
  if (!m) {
    if (s) mj_deleteSpec(s);
    return finish("Could not load model", EXIT_FAILURE);
  }

  // save model
  if (type2 == typeXML) {
    if (!mj_saveLastXML(argv[2], m, error, 1000)) {
      if (s) mj_deleteSpec(s);
      return finish(error, EXIT_FAILURE, m);
    }
  } else if (type2 == typeMJB) {
    mj_saveModel(m, argv[2], 0, 0);
  } else if (type2 == typeTXT) {
    mj_printModel(m, argv[2]);
  }

  // finalize
  if (s) mj_deleteSpec(s);
  return finish("\nDone.", EXIT_SUCCESS, m);
}
