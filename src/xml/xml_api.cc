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

#include "xml/xml_api.h"

#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <random>

#include "user/user_model.h"
#include "xml/xml.h"
#include "xml/xml_native_reader.h"
#include "xml/xml_util.h"

//---------------------------------- Globals -------------------------------------------------------

// global user model class
class GlobalModel {
 public:
  GlobalModel();
  ~GlobalModel();
  void Clear(void);

  mjCModel* model;
};


GlobalModel::GlobalModel() {
  // clear pointers
  model = 0;
}


GlobalModel::~GlobalModel() {
  Clear();
}


void GlobalModel::Clear() {
  // de-allocate models
  if (model) {
    delete model;
  }

  // clear pointers
  model = 0;
}



// single instance of global model, protected with mutex
static GlobalModel themodel;
static std::mutex themutex;

//---------------------------------- Functions -----------------------------------------------------

// Return 1 (for backward compatibility).
int mj_activate(const char* filename) {
  return 1;
}



// Do nothing (for backward compatibility).
void mj_deactivate(void) {
}



// parse XML file in MJCF or URDF format, compile it, return low-level model
//  if vfs is not NULL, look up files in vfs before reading from disk
//  error can be NULL; otherwise assumed to have size error_sz
mjModel* mj_loadXML(const char* filename, const mjVFS* vfs,
                    char* error, int error_sz) {
  // serialize access to themodel
  std::lock_guard<std::mutex> lock(themutex);

  // parse new model
  mjCModel* newmodel = mjParseXML(filename, vfs, error, error_sz);
  if (!newmodel) {
    return 0;
  }

  // compile new model
  mjModel* m = newmodel->Compile(vfs);
  if (!m) {
    mjCopyError(error, newmodel->GetError().message, error_sz);
    delete newmodel;
    return 0;
  }

  // clear old and assign new
  themodel.Clear();
  themodel.model = newmodel;

  // handle compile warning
  if (themodel.model->GetError().warning) {
    mjCopyError(error, themodel.model->GetError().message, error_sz);
  } else if (error) {
    error[0] = 0;
  }

  return m;
}




// update XML data structures with info from low-level model, save as MJCF
//  returns 1 if successful, 0 otherwise
//  error can be NULL; otherwise assumed to have size error_sz
int mj_saveLastXML(const char* filename, const mjModel* m, char* error, int error_sz) {
  // serialize access to themodel
  std::lock_guard<std::mutex> lock(themutex);

  if (!themodel.model) {
    mjCopyError(error, "No XML model loaded", error_sz);
    return 0;
  }

  themodel.model->CopyBack(m);
  if (mjWriteXML(themodel.model, filename, error, error_sz)) {
    if (error) {
      error[0] = 0;
    }
    return 1;
  } else {
    return 0;
  }
}


// free last XML
void mj_freeLastXML(void) {
  // serialize access to themodel
  std::lock_guard<std::mutex> lock(themutex);

  themodel.Clear();
}




// print internal XML schema as plain text or HTML, with style-padding or &nbsp;
int mj_printSchema(const char* filename, char* buffer, int buffer_sz, int flg_html, int flg_pad) {
  // serialize access, even though it is not necessary
  std::lock_guard<std::mutex> lock(themutex);

  // print to stringstream
  mjXReader reader;
  std::stringstream str;
  reader.PrintSchema(str, flg_html!=0, flg_pad!=0);

  // filename given: write to file
  if (filename) {
    std::ofstream file;
    file.open(filename);
    file << str.str();
    file.close();
  }

  // buffer given: write to buffer
  if (buffer && buffer_sz) {
    strncpy(buffer, str.str().c_str(), buffer_sz);
    buffer[buffer_sz-1] = 0;
  }

  // return string length
  return str.str().size();
}
