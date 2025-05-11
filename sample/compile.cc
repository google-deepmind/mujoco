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
#include <cstring>
#include <iostream>

#include <mujoco/mujoco.h>

// help
static constexpr char helpstring[] =
  "\n Usage:  compile infile outfile\n"
  "   infile can be in mjcf, urdf, mjb format\n"
  "   outfile can be in mjcf, mjb, txt format, or empty\n\n"
  "   if infile is mjcf, compilation will be timed twice to measure the impact of caching\n\n"
  " Example: compile model.xml [model.mjb]\n";

// timer (seconds)
mjtNum gettm(void) {
  using Clock = std::chrono::steady_clock;
  using Seconds = std::chrono::duration<mjtNum>;
  static const Clock::time_point tm_start = Clock::now();
  return Seconds(Clock::now() - tm_start).count();
}

// deallocate and print message
int finish(const char* msg = 0, mjModel* m = 0) {
  // deallocated everything
  if (m) {
    mj_deleteModel(m);
  }

  // print message
  if (msg) {
    std::cout << msg << std::endl;
  }

  return 0;
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
    return finish(helpstring);
  }

  // determine file types
  int type1 = filetype(argv[1]);
  int type2 = argc==2 ? typeNONE : filetype(argv[2]);

  // check types
  if (type1==typeUNKNOWN || type1==typeTXT ||
      type2==typeUNKNOWN || (type1==typeMJB && type2==typeXML)) {
    return finish("Illegal combination of file formats");
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

  // load model
  double first=0, second=0;
  if (type1==typeXML) {
    double starttime = gettm();
    m = mj_loadXML(argv[1], 0, error, 1000);
    first = gettm() - starttime;
    if (m) {
      mj_deleteModel(m);
      starttime = gettm();
      m = mj_loadXML(argv[1], 0, error, 1000);
      second = gettm() - starttime;
    }
  } else {
    m = mj_loadModel(argv[1], 0);
  }

  // check error
  if (!m) {
    if (type1==typeXML) {
      return finish(error, 0);
    } else {
      return finish("Could not load model", 0);
    }
  }

  // save model
  if (type2==typeXML) {
    if (!mj_saveLastXML(argv[2], m, error, 1000)) {
      return finish(error, m);
    }
  } else if (type2==typeMJB) {
    mj_saveModel(m, argv[2], 0, 0);
  } else if (type2==typeTXT) {
    mj_printModel(m, argv[2]);
  }

  // finalize
  char msg[1000];
  if (first) {
    snprintf(msg, sizeof(msg), "Done.\n"
             "First compile: %.4gs\n"
             "Second compile: %.4gs",
             first, second);
  } else {
    snprintf(msg, sizeof(msg), "Done.");
  }

  return finish(msg, m);
}
