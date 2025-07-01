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

#include <array>
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
#include "engine/engine_io.h"
#include <mujoco/mjspec.h>
#include "user/user_resource.h"
#include "user/user_vfs.h"
#include "xml/xml.h"
#include "xml/xml_native_reader.h"
#include "xml/xml_util.h"

//---------------------------------- Globals -------------------------------------------------------

namespace {

// global user model class
class GlobalModel {
 public:
  // deletes current model and takes ownership of model
  void Set(mjSpec* spec = nullptr);

  // writes XML to string
  std::optional<std::string> ToXML(const mjModel* m, char* error,
                                      int error_sz);

 private:
  // using raw pointers as GlobalModel needs to be trivially destructible
  std::mutex* mutex_ = new std::mutex();
  mjSpec* spec_ = nullptr;
};

std::optional<std::string> GlobalModel::ToXML(const mjModel* m, char* error,
                                              int error_sz) {
  std::lock_guard<std::mutex> lock(*mutex_);
  if (!spec_) {
    mjCopyError(error, "No XML model loaded", error_sz);
    return std::nullopt;
  }
  std::string result = WriteXML(m, spec_, error, error_sz);
  if (result.empty()) {
    return std::nullopt;
  }
  return result;
}

void GlobalModel::Set(mjSpec* spec) {
  std::lock_guard<std::mutex> lock(*mutex_);
  if (spec_ != nullptr) {
    mj_deleteSpec(spec_);
  }
  spec_ = spec;
}


// returns a single instance of the global model
GlobalModel& GetGlobalModel() {
  static GlobalModel global_model;

  // global variables must be trivially destructible
  static_assert(std::is_trivially_destructible_v<decltype(global_model)>);
  return global_model;
}

}  // namespace

//---------------------------------- Functions -----------------------------------------------------

// parse XML file in MJCF or URDF format, compile it, return low-level model
//  if vfs is not NULL, look up files in vfs before reading from disk
//  error can be NULL; otherwise assumed to have size error_sz
mjModel* mj_loadXML(const char* filename, const mjVFS* vfs,
                    char* error, int error_sz) {

  // parse new model
  std::unique_ptr<mjSpec, std::function<void(mjSpec*)> > spec(
    ParseXML(filename, vfs, error, error_sz),
    [](mjSpec* s) {
      mj_deleteSpec(s);
    });
  if (!spec) {
    return nullptr;
  }

  // compile new model
  mjModel* m = mj_compile(spec.get(), vfs);
  if (!m) {
    mjCopyError(error, mjs_getError(spec.get()), error_sz);
    return nullptr;
  }

  // handle compile warning
  if (mjs_isWarning(spec.get())) {
    mjCopyError(error, mjs_getError(spec.get()), error_sz);
  } else if (error) {
    error[0] = '\0';
  }

  // clear old and assign new
  GetGlobalModel().Set(spec.release());
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
  reader.PrintSchema(str, flg_html != 0, flg_pad != 0);

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



// load model from binary MJB resource
mjModel* mj_loadModel(const char* filename, const mjVFS* vfs) {
  std::array<char, 1024> error;
  mjResource* resource = mju_openResource("", filename, vfs,
                                          error.data(), error.size());
  if (resource == nullptr) {
    mju_warning("%s", error.data());
    return nullptr;
  }

  const void* buffer = NULL;
  int buffer_sz = mju_readResource(resource, &buffer);
  if (buffer_sz < 1) {
    mju_closeResource(resource);
    return nullptr;
  }

  mjModel* m = mj_loadModelBuffer(buffer, buffer_sz);
  mju_closeResource(resource);
  return m;
}



// parse spec from file
mjSpec* mj_parseXML(const char* filename, const mjVFS* vfs, char* error, int error_sz) {
  return ParseXML(filename, vfs, error, error_sz);
}



// parse spec from string
mjSpec* mj_parseXMLString(const char* xml, const mjVFS* vfs, char* error, int error_sz) {
  return ParseSpecFromString(xml, vfs, error, error_sz);
}



// save spec to XML file, return 0 on success, -1 otherwise
int mj_saveXML(const mjSpec* s, const char* filename, char* error, int error_sz) {
  // cast to mjSpec since WriteXML can in principle perform mj_copyBack (not here)
  std::string result = WriteXML(NULL, (mjSpec*)s, error, error_sz);
  if (result.empty()) {
    return -1;
  }

  std::ofstream file;
  file.open(filename);
  file << result;
  file.close();
  return 0;
}



// save spec to XML string, return 0 on success, -1 on failure
// if length of the output buffer is too small, returns the required size
int mj_saveXMLString(const mjSpec* s, char* xml, int xml_sz, char* error, int error_sz) {
  std::string result = WriteXML(NULL, (mjSpec*)s, error, error_sz);
  if (result.empty()) {
    return -1;
  } else if (result.size() >= xml_sz) {
    std::string error_msg = "Output string too short, should be at least " +
                            std::to_string(result.size()+1);
    mjCopyError(error, error_msg.c_str(), error_sz);
    return result.size();
  }

  result.copy(xml, xml_sz);
  xml[result.size()] = 0;
  return 0;
}

