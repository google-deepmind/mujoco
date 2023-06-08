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

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>

#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>

#include "array_safety.h"
namespace mju = ::mujoco::sample_util;

static constexpr int kFieldSize = 500;

// help
const char helpstring[] = "\n Usage:  testxml modelfile.xml\n";


// deallocate and print message
int finish(const char* msg = 0, mjModel* m = 0, mjData* d = 0) {
  // deallocated everything
  if (d) {
    mj_deleteData(d);
  }
  if (m) {
    mj_deleteModel(m);
  }

  // print message
  if (msg) {
    std::printf("%s\n", msg);
  }

  return EXIT_SUCCESS;
}


// return absolute difference if it is below 1, relative difference otherwise
static mjtNum _compare(mjtNum val1, mjtNum val2) {
  mjtNum magnitude = mju_max(mju_abs(val1), mju_abs(val2));

  if (magnitude>1.0) {
    return mju_abs(val1-val2) / magnitude;
  } else {
    return mju_abs(val1-val2);
  }
}


// compare two models, return largest difference and field name
mjtNum compareModel(const mjModel* m1, const mjModel* m2, char (&field)[kFieldSize]) {
  int r, c;
  mjtNum dif, maxdif = 0.0;

  // define symbols corresponding to number of columns (needed in MJMODEL_POINTERS)
  MJMODEL_POINTERS_PREAMBLE(m1);

  // compare ints
  #define X(name) if(m1->name!=m2->name) {mju::strcpy_arr(field, #name); return 1.0;}

    MJMODEL_INTS
  #undef X

    // compare arrays
  #define X(type, name, nr, nc)                                   \
          for( r=0; r<m1->nr; r++ )                               \
          for( c=0; c<nc; c++ ) {                                 \
              dif = _compare(m1->name[r*nc+c], m2->name[r*nc+c]); \
              if(dif>maxdif) {maxdif=dif; mju::strcpy_arr(field, #name);} }

    MJMODEL_POINTERS
  #undef X

  // compare scalars in mjOption
  #define X(type, name)                                           \
          dif = _compare(m1->opt.name, m2->opt.name);             \
          if(dif>maxdif) {maxdif=dif; mju::strcpy_arr(field, #name);}

    MJOPTION_SCALARS
  #undef X

  // compare arrays in mjOption
  #define X(name, n)                                              \
          for( c=0; c<n; c++ ) {                                  \
              dif = _compare(m1->opt.name[c], m2->opt.name[c]);   \
              if(dif>maxdif) {maxdif=dif; mju::strcpy_arr(field, #name);} }

    MJOPTION_VECTORS
  #undef X

  // mjVisual and mjStatistics ignored for now

  return maxdif;
}



// main function
int main(int argc, const char** argv) {
  // print help if arguments are missing
  if (argc<2) {
    return finish(helpstring);
  }

  // get filename, check file type
  std::string filename(argv[1]);
  if (filename.find(".xml")==std::string::npos) {
    return finish("xml model file is required");
  }

  // load model
  char error[1000];
  mjModel* m = mj_loadXML(argv[1], 0, error, 1000);
  if (!m) {
    return finish(error);
  }

  // make data
  mjData* d = mj_makeData(m);
  if (!d) {
    return finish("Could not allocate mjData", m);
  }

  // prepare temp filename in the same directory as original (for asset loading)
  std::string tempfile;
  std::size_t lastpath = filename.find_last_of("/\\");
  if (lastpath==std::string::npos) {
    tempfile = "_tempfile_.xml";
  } else {
    tempfile = filename.substr(0, lastpath+1) + "_tempfile_.xml";
  }

  // save
  if (!mj_saveLastXML(tempfile.c_str(), m, error, 1000)) {
    return finish(error, m, d);
  }

  // load back
  mjModel* mtemp = mj_loadXML(tempfile.c_str(), 0, error, 100);
  if (!mtemp) {
    return finish(error, m, d);
  }

  // compare
  char field[kFieldSize] = "";
  mjtNum result = compareModel(m, mtemp, field);
  std::printf("\nComparison of original and saved model\n");
  std::printf(" Max difference : %.3g\n", result);
  std::printf(" Field name     : %s\n", field);

  // delete temp model and file
  mj_deleteModel(mtemp);
  remove(tempfile.c_str());

  // finalize
  return finish();
}
