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

#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>

#include <mujoco/mjmodel.h>
#include "user/user_api.h"
#include "xml/xml.h"
#include "xml/xml_native_reader.h"
#include "xml/xml_util.h"

//---------------------------------- Globals -------------------------------------------------------

// global user model class
class GlobalModel {
 public:
  // deletes current model and takes ownership of model
  void Set(mjmModel* model = nullptr);

  // writes XML to string
  std::optional<std::string> ToXML(const mjModel* m, char* error,
                                      int error_sz);

 private:
  // using raw pointers as GlobalModel needs to be trivially destructible
  std::mutex* mutex_ = new std::mutex();
  mjmModel* model_ = nullptr;
};

std::optional<std::string> GlobalModel::ToXML(const mjModel* m, char* error,
                                              int error_sz) {
  std::lock_guard<std::mutex> lock(*mutex_);
  if (!model_) {
    mjCopyError(error, "No XML model loaded", error_sz);
    return std::nullopt;
  }
  mjm_copyBack(model_, m);
  std::string result = mjWriteXML(model_, error, error_sz);
  if (result.empty()) {
    return std::nullopt;
  }
  return result;
}

void GlobalModel::Set(mjmModel* model) {
  std::lock_guard<std::mutex> lock(*mutex_);
  if (model_ != nullptr) {
    mjm_deleteModel(model_);
  }
  model_ = model;
}


// returns a single instance of the global model
GlobalModel& GetGlobalModel() {
  static GlobalModel global_model;

  // global variables must be trivially destructible
  static_assert(std::is_trivially_destructible_v<decltype(global_model)>);
  return global_model;
}

//---------------------------------- Functions -----------------------------------------------------

// parse XML file in MJCF or URDF format, compile it, return low-level model
//  if vfs is not NULL, look up files in vfs before reading from disk
//  error can be NULL; otherwise assumed to have size error_sz
mjModel* mj_loadXML(const char* filename, const mjVFS* vfs,
                    char* error, int error_sz) {

  // parse new model
  std::unique_ptr<mjmModel, std::function<void(mjmModel*)>> model(
      mjParseXML(filename, vfs, error, error_sz),
      [](mjmModel* m) { mjm_deleteModel(m); });
  if (!model) {
    return nullptr;
  }

  // compile new model
  mjModel* m = mjm_compileModel(model.get(), vfs);
  if (!m) {
    mjCopyError(error, mjm_getError(model.get()), error_sz);
    return nullptr;
  }

  // handle compile warning
  if (mjm_isWarning(model.get())) {
    mjCopyError(error, mjm_getError(model.get()), error_sz);
  } else if (error) {
    error[0] = '\0';
  }

  // clear old and assign new
  GetGlobalModel().Set(model.release());
  return m;
}



// update XML data structures with info from low-level model, save as MJCF
//  returns 1 if successful, 0 otherwise
//  error can be NULL; otherwise assumed to have size error_sz
int mj_saveLastXML(const char* filename, const mjModel* m, char* error, int error_sz) {
  FILE *fp = stdout;
  if (filename != nullptr && filename[0] != '\0') {
    fp = fopen(filename, "w");
    if (!fp) {
      mjCopyError(error, "File not found", error_sz);
      return 0;
    }
  }

  auto result = GetGlobalModel().ToXML(m, error, error_sz);
  if (result.has_value()) {
    fprintf(fp, "%s", result->c_str());
  }

  if (fp != stdout) {
    fclose(fp);
  }

  return result.has_value();
}



// free last XML
void mj_freeLastXML(void) {
  GetGlobalModel().Set();
}




// print internal XML schema as plain text or HTML, with style-padding or &nbsp;
int mj_printSchema(const char* filename, char* buffer, int buffer_sz, int flg_html, int flg_pad) {
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
