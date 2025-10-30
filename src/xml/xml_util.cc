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

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include "cc/array_safety.h"
#include "engine/engine_util_errmem.h"
#include "user/user_resource.h"
#include "user/user_util.h"
#include "xml/xml_util.h"
#include "xml/xml_numeric_format.h"
#include "tinyxml2.h"

namespace {

using tinyxml2::XMLAttribute;
using tinyxml2::XMLElement;
using mujoco::user::FilePath;

namespace mju = ::mujoco::util;

template <typename T>
static std::optional<T> ParseInfOrNan(const std::string& s) {
  const char* str = s.c_str();
  if constexpr (std::is_floating_point_v<T>) {
    T sign = 1;
    if (s.size() == 4 && s[0] == '-') {
      sign = -1;
      ++str;
    } else if (s.size() != 3) {
      return std::nullopt;
    }
    if (std::numeric_limits<T>::has_infinity &&
        (str[0] == 'i' || str[0] == 'I') &&
        (str[1] == 'n' || str[1] == 'N') &&
        (str[2] == 'f' || str[2] == 'F')) {
      return sign * std::numeric_limits<T>::infinity();
    } else if (std::numeric_limits<T>::has_quiet_NaN &&
               (str[0] == 'n' || str[0] == 'N') &&
               (str[1] == 'a' || str[1] == 'A') &&
               (str[2] == 'n' || str[2] == 'N')) {
      return sign * std::numeric_limits<T>::quiet_NaN();
    }
  }
  return std::nullopt;
}

FilePath ResolveFilePath(XMLElement* e, const FilePath& filename,
                         const FilePath& dir, const mjVFS* vfs) {
  std::string path = "";
  if (filename.IsAbs()) {
    return filename;
  }

  // TODO(kylebayes): We first look in the base model directory for files to
  // remain backwards compatible.
  FilePath fullname = dir + filename;
  mjResource *resource = mju_openResource("", fullname.c_str(), vfs,
                                          nullptr, 0);
  if (resource != nullptr) {
    mju_closeResource(resource);
    return filename;
  }

  XMLElement* parent = e->Parent()->ToElement();
  for (; parent; parent = parent->Parent()->ToElement()) {
    if (!std::strcmp(parent->Value(), "include")) {
      auto file_attr = mjXUtil::ReadAttrStr(parent, "dir", false);
      if (file_attr.has_value()) {
        path = file_attr.value();
      }
      break;
    }
  }
  return FilePath(path) + filename;
}

void AccumulateFiles(std::unordered_set<std::string> &files,
                     tinyxml2::XMLElement *root, const FilePath &model_dir) {
  std::optional<FilePath> asset_dir;
  std::optional<FilePath> mesh_dir;
  std::optional<FilePath> texture_dir;
  std::set<std::string> include_and_model_files;
  std::set<std::string> texture_files;
  std::set<std::string> mesh_files;
  std::set<std::string> hfield_files;

  auto accumulate_files = [&](const std::set<std::string> &candidate_files,
                              std::optional<FilePath> prefix) {
    for (const auto &file : candidate_files) {
      FilePath file_with_prefix = !prefix.has_value() ? FilePath(file) : prefix.value() + FilePath(file);
      if (file_with_prefix.IsAbs()) {
        files.insert(file_with_prefix.Str());
      } else {
        // Else insert dir_path / prefix / file.
        auto full_path = model_dir + file_with_prefix;
        files.insert(full_path.Str());
      }
    }
  };

  std::stack<tinyxml2::XMLElement *> elements;
  elements.push(root);
  while (!elements.empty()) {
    tinyxml2::XMLElement *elem = elements.top();
    elements.pop();

    if (!std::strcmp(elem->Value(), "include") ||
        !std::strcmp(elem->Value(), "model")) {
      auto file_attr = mjXUtil::ReadAttrFile(elem, "file", nullptr);
      if (file_attr.has_value()) {
        include_and_model_files.insert(file_attr->Str());
        // Neither of these elements should have children.
        continue;
      }
    } else if (!std::strcmp(elem->Value(), "compiler")) {
      auto assetdir_str = mjXUtil::ReadAttrStr(elem, "assetdir", false);
      if (assetdir_str.has_value()) asset_dir = FilePath(assetdir_str.value());
      auto meshdir_str = mjXUtil::ReadAttrStr(elem, "meshdir", false);
      if (meshdir_str.has_value()) mesh_dir = FilePath(meshdir_str.value());
      auto texturedir_str = mjXUtil::ReadAttrStr(elem, "texturedir", false);
      if (texturedir_str.has_value()) texture_dir = FilePath(texturedir_str.value());

      // compiler elements don't have children.
      continue;
    } else if (!std::strcmp(elem->Value(), "mesh") ||
               !std::strcmp(elem->Value(), "flexcomp") ||
               !std::strcmp(elem->Value(), "skin")) {
      // mesh elements don't have children.
      auto file_attr = mjXUtil::ReadAttrFile(elem, "file", nullptr);
      if (file_attr.has_value()) {
        mesh_files.insert(file_attr->Str());
      }
      continue;
    } else if (!std::strcmp(elem->Value(), "hfield")) {
      // hfield elements don't have children.
      auto file_attr = mjXUtil::ReadAttrFile(elem, "file", nullptr);
      if (file_attr.has_value()) {
        hfield_files.insert(file_attr->Str());
      }
      continue;
    } else if (!std::strcmp(elem->Value(), "texture")) {
      static const char *attributes[] = {"file",     "fileright", "fileup",
                                         "fileleft", "filedown",  "filefront",
                                         "fileback"};
      for (const auto &attribute : attributes) {
        auto file_attr = mjXUtil::ReadAttrFile(elem, attribute, nullptr);
        if (file_attr.has_value()) {
          texture_files.insert(file_attr->Str());
        }
      }
    }

    tinyxml2::XMLElement *child = elem->FirstChildElement();
    while (child) {
      elements.push(child);
      child = child->NextSiblingElement();
    }
  }

  // TODO(shaves): When we have resource decoders implemented they should have a
  // "get dependencies" function to call here. For non XML types we assume they
  // have no dependencies here.

  // First resolve all dependent XML files.
  for (const auto &file : include_and_model_files) {
    mjStringVec subdeps;
    FilePath full_path = model_dir + FilePath(file);
    mju_getXMLDependencies(full_path.Str().c_str(), &subdeps);
    for (const auto &subdep : subdeps) {
      files.insert(subdep);
    }
  }
  // Then for each non MJCF resource file, add them to the set of files using their respective
  // compiler prefixes (if they exist).
  accumulate_files(texture_files,
                   texture_dir.has_value() ? texture_dir : asset_dir);
  accumulate_files(mesh_files, mesh_dir.has_value() ? mesh_dir : asset_dir);
  accumulate_files(hfield_files, asset_dir);
}
}

//---------------------------------- utility functions ---------------------------------------------

// error string copy
void mjCopyError(char* dst, const char* src, int maxlen) {
  if (dst && maxlen > 0) {
    strncpy(dst, src, maxlen);
    dst[maxlen-1] = 0;
  }
}

void mju_getXMLDependencies(const char* filename, mjStringVec* dependencies) {
  // load XML file or parse string
  tinyxml2::XMLDocument doc;
  doc.LoadFile(filename);

  // error checking
  if (doc.Error()) {
    mju_error("Problem reading XML file '%s': %s", filename, doc.ErrorStr());
  }

  // get top-level element
  tinyxml2::XMLElement *root = doc.RootElement();
  if (!root) {
    mju_error("XML root element not found");
  }
  std::unordered_set<std::string> files = {filename};

  std::optional<FilePath> model_dir = std::nullopt;
  mjResource *resource = mju_openResource("", filename, nullptr,
                                          nullptr, 0);
  if (resource != nullptr) {
    const char* dir;
    int ndir;
    mju_getResourceDir(resource, &dir, &ndir);
    model_dir = FilePath(std::string(dir, ndir));
    mju_closeResource(resource);
  }
  // Get file references from include and model tags.
  AccumulateFiles(files, root, model_dir.value());

  *dependencies = {files.begin(), files.end()};
}

// error constructor
mjXError::mjXError(const XMLElement* elem, const char* msg, const char* str, int pos) {
  char temp[500];

  // construct error message
  mju::sprintf_arr(message, "XML Error");
  if (msg) {
    mju::sprintf_arr(temp, msg, str, pos);
    mju::strcat_arr(message, ": ");
    mju::strcat_arr(message, temp);
  }

  // append element, line numbers
  if (elem) {
    mju::sprintf_arr(temp, "\nElement '%s', line %d\n", elem->Value(), elem->GetLineNum());

    mju::strcat_arr(message, temp);
  }
}



//---------------------------------- class mjXSchema implementation --------------------------------

XMLElement* FirstChildElement(XMLElement* e, const char* name) {
  XMLElement* child = e->FirstChildElement();
  for (; child; child = child->NextSiblingElement()) {
    if (!std::strcmp(child->Name(), "include")) {
      XMLElement* temp = FirstChildElement(child, name);
      if (temp) {
        return temp;
      }
      continue;
    }

    if (!name || !std::strcmp(child->Name(), name)) {
      return child;
    }
  }
  return nullptr;
}

XMLElement* NextSiblingElement(XMLElement* e, const char* name) {
  XMLElement* elem = e->NextSiblingElement();
  for (; elem; elem = elem->NextSiblingElement()) {
    if (!std::strcmp(elem->Name(), "include")) {
      XMLElement* temp = FirstChildElement(elem, name);
      if (temp) {
        return temp;
      }
      continue;
    }

    if (!name || !std::strcmp(elem->Name(), name)) {
      return elem;
    }
  }

  XMLElement* parent = e->Parent()->ToElement();
  if (parent && !std::strcmp(parent->Name(), "include")) {
    return NextSiblingElement(parent, name);
  }

  return nullptr;
}

// constructor
mjXSchema::mjXSchema(const char* schema[][mjXATTRNUM], unsigned nrow) {
  // set name and type
  name_ = schema[0][0];
  type_ = schema[0][1][0];

  // set attributes
  int nattr = atoi(schema[0][2]);
  for (int i = 0; i < nattr; i++) {
    attr_.emplace(schema[0][3 + i]);
  }

  // process sub-elements of complex element
  if (nrow > 1) {
    // parse block into simple and complex elements, create children
    int start = 2;
    while (start < nrow-1) {
      int end = start;

      // look for bracketed block at start+1
      if (schema[start+1][0][0] == '<') {
        // look for corresponding closing bracket
        int cnt = 0;
        while (end <= nrow-1) {
          if (schema[end][0][0] == '<') {
            cnt++;
          } else if (schema[end][0][0] == '>') {
            cnt--;
            if (cnt == 0) {
              break;
            }
          }

          end++;
        }
      }

      // add child element
      subschema_.emplace_back(schema+start, end-start+1);

      // proceed with next subelement
      start = end+1;
    }
  }
}



// get pointer to error message
std::string mjXSchema::GetError() {
  return error;
}



// print spaces
static void printspace(std::stringstream& str, int n, const char* space) {
  for (int i=0; i < n; i++) {
    str << space;
  }
}



// print schema as text
void mjXSchema::Print(std::stringstream& str, int level) const {
  // replace body with (world)body
  std::string name1 = (name_ == "body") ? "(world)body" : name_;

  // space, name, type
  printspace(str, 3*level, " ");
  str << name1 << " (" << type_ << ")";
  int baselen = 3*level + (int)name1.size() + 4;
  if (baselen < 30) {
    printspace(str, 30-baselen, " ");
  }

  // attributes
  int cnt = std::max(baselen, 30);
  for (const std::string& attr : attr_) {
    if (cnt > 60) {
      str << "\n";
      printspace(str, (cnt = std::max(30, baselen)), " ");
    }

    str << attr << " ";
    cnt += (int)attr.size() + 1;
  }
  str << "\n";

  // children
  for (const mjXSchema& subschema : subschema_) {
    subschema.Print(str, level+1);
  }
}



// print schema as HTML table
void mjXSchema::PrintHTML(std::stringstream& str, int level, bool pad) const {
  // replace body with (world)body
  std::string name1 = (name_ == "body" ? "(world)body" : name_);

  // open table
  if (level == 0) {
    str << "<table border=\"1\">\n";
  }

  // name: with HTML padding
  if (pad) {
    str << "<tr>\n\t<td style=\"padding-left:" << 5 + 15*level;
    str << "\" bgcolor=\"#EEEEEE\" class=\"el\">" << name1 << "</td>\n";
  }

  // name: with &nbsp; for browsers that ignore padding
  else {
    str << "<tr>\n\t<td bgcolor=\"#EEEEEE\" class=\"el\">";
    if (level) {
      printspace(str, 4*level, "&nbsp;");
    }
    str << name1 << "</td>\n";
  }

  // type
  str << "\t<td class=\"ty\">" << type_ << "</td>\n";

  // attributes
  str << "\t<td class=\"at\">";
  if (!attr_.empty()) {
    for (const std::string& attr : attr_) {
      str << attr << " ";
    }
  } else {
    str << "<span style=\"color:black\"><i>no attributes</i></span>";
  }
  str << "</td>\n</tr>\n";

  // children
  for (const mjXSchema& subschema : subschema_) {
    subschema.PrintHTML(str, level+1, pad);
  }

  // close table
  if (!level) {
    str << "</table>\n";
  }
}



// check for name match
bool mjXSchema::NameMatch(XMLElement* elem, int level) {
  // special handling of body, worldbody, and frame
  if (name_ == "body" &&
      ((level == 1 && !strcmp(elem->Value(), "worldbody")) ||
       (level != 1 && !strcmp(elem->Value(), "body")) ||
       (level >= 1 && !strcmp(elem->Value(), "frame")) ||
       (level >= 1 && !strcmp(elem->Value(), "replicate")))) {
    return true;
  }

  // regular check
  return name_ == elem->Value();
}



// validator
XMLElement* mjXSchema::Check(XMLElement* elem, int level) {
  bool missing;
  char msg[100];
  XMLElement *bad, *sub;

  error.clear();
  if (!elem) {
    return 0;  // SHOULD NOT OCCUR
  }

  // check name (already done by parent, but hard to avoid)
  if (!NameMatch(elem, level)) {
    error = "unrecognized element";
    return elem;
  }

  // check attributes
  const XMLAttribute* attribute = elem->FirstAttribute();
  for (; attribute != nullptr; attribute = attribute->Next()) {
    if (attr_.find(attribute->Name()) == attr_.end()) {
      error = "unrecognized attribute: '" + std::string(attribute->Name()) + "'";
      return elem;
    }
  }

  // handle recursion
  if (type_ == 'R') {
    // check child elements with same name
    sub = FirstChildElement(elem, name_.c_str());
    for (; sub != nullptr; sub = NextSiblingElement(sub, name_.c_str())) {
      if ((bad = Check(sub, level+1))) {
        return bad;
      }
    }
  }

  // clear reference counts
  for (mjXSchema& subschema : subschema_) {
    subschema.refcnt_ = 0;
  }

  // check sub-elements, update refcnt
  sub = FirstChildElement(elem);
  for (; sub != nullptr; sub = NextSiblingElement(sub)) {
    missing = true;

    for (mjXSchema& subschema : subschema_) {
      if (subschema.NameMatch(sub, level+1)) {
        // check sub-tree
        if ((bad = subschema.Check(sub, level+1))) {
          error = subschema.error;
          return bad;
        }

        // mark found
        missing = false;
        subschema.refcnt_++;
        break;
      }
    }

    // missing, unless recursive
    if (missing && !(type_ == 'R' && NameMatch(sub, level+1))) {
      error = "unrecognized element";
      return sub;
    }
  }

  // enforce sub-element types
  msg[0] = '\0';
  for (mjXSchema& subschema : subschema_) {
    switch (subschema.type_) {
      case '!':
        if (subschema.refcnt_ > 1)
          mju::sprintf_arr(msg, "unique element '%s' found %d times",
                           subschema.name_.c_str(), subschema.refcnt_);
        else if (subschema.refcnt_ < 1)
          mju::sprintf_arr(msg, "element '%s' is required",
                           subschema.name_.c_str());
        break;

      case '?':
        if (subschema.refcnt_ > 1)
          mju::sprintf_arr(msg, "unique element '%s' found %d times",
                           subschema.name_.c_str(), subschema.refcnt_);
        break;

      default:
        break;
    }
  }

  // handle error
  if (msg[0]) {
    error = msg;
    return elem;
  }
  return nullptr;
}



//---------------------------------- class mjXUtil implementation ----------------------------------

// helper function to read multiple numerical values from an attribute
// return false if the entire attribute wasn't read (max was reached)
// throw error if syntax error while trying to read numerical data
template<typename T>
bool mjXUtil::ReadAttrValues(XMLElement* elem, const char* attr,
                             std::function<void (int, T)> push, int max) {
  const char* pstr = elem->Attribute(attr);
  T item;

  if (pstr == nullptr) {
    return true;
  }

  // get input stream
  std::string str = std::string(pstr);
  std::istringstream strm(str);
  std::string token;

  // read numbers
  for (int i = 0; (max < 0 || i < max) && !strm.eof(); ++i) {
    strm >> token;
    std::istringstream token_strm(token);
    token_strm >> item;
    if (token_strm.fail() || !token_strm.eof()) {
      // C++ standard libraries do not always parse inf and nan as valid floating point values.
      std::optional<T> maybe_result = ParseInfOrNan<T>(token);
      if (maybe_result.has_value()) {
        item = maybe_result.value();
      } else {
        throw mjXError(elem, "problem reading attribute '%s'", attr);
      }
    }

    push(i, item);
    if constexpr (std::is_floating_point_v<T>) {
      if (std::isnan(item)) {
        mju_warning("XML contains a 'NaN'. Please check it carefully.");
      }
    }
    // clear any trailing whitespace
    strm >> std::ws;
  }

  return strm.eof();
}

template bool mjXUtil::ReadAttrValues(XMLElement* elem, const char* attr,
                                      std::function<void (int, double)> push, int max);
template bool mjXUtil::ReadAttrValues(XMLElement* elem, const char* attr,
                                      std::function<void (int, float)> push, int max);
template bool mjXUtil::ReadAttrValues(XMLElement* elem, const char* attr,
                                      std::function<void (int, int)> push, int max);
template bool mjXUtil::ReadAttrValues(XMLElement* elem, const char* attr,
                                      std::function<void (int, unsigned char)> push, int max);



// compare two vectors
template<typename T>
bool mjXUtil::SameVector(const T* vec1, const T* vec2, int n) {
  if (!vec1 || !vec2) {
    return false;
  }

  for (int i = 0; i < n; i++) {
    if (std::abs(vec1[i] - vec2[i]) > std::numeric_limits<T>::epsilon()) {
      return false;
    }
  }

  return true;
}

template bool mjXUtil::SameVector(const double* vec1, const double* vec2, int n);
template bool mjXUtil::SameVector(const float* vec1, const float* vec2, int n);
template bool mjXUtil::SameVector(const int* vec1, const int* vec2, int n);
template bool mjXUtil::SameVector(const unsigned char* vec1, const unsigned char* vec2, int n);


// find string in map, return corresponding integer (-1: not found)
int mjXUtil::FindKey(const mjMap* map, int mapsz, std::string key) {
  for (int i=0; i < mapsz; i++) {
    if (map[i].key == key) {
      return map[i].value;
    }
  }

  return -1;
}



// find integer in map, return corresponding string ("": not found)
std::string mjXUtil::FindValue(const mjMap* map, int mapsz, int value) {
  for (int i=0; i < mapsz; i++) {
    if (map[i].value == value) {
      return map[i].key;
    }
  }

  return "";
}



// if attribute is present, return vector of numerical data
template<typename T>
std::optional<std::vector<T> > mjXUtil::ReadAttrVec(XMLElement* elem, const char* attr,
                                                    bool required) {
  std::vector<T> v;
  const char* raw_cstr = elem->Attribute(attr);
  if (raw_cstr) {
    v = mujoco::user::StringToVector<T>(raw_cstr);
    if (errno == EDOM) {
      mju_warning("XML contains a 'NaN'. Please check it carefully.");
    } else if (errno == ERANGE) {
      throw mjXError(elem, "number is too large in attribute '%s'", attr);
    } else if (errno == EINVAL) {
      throw mjXError(elem, "bad format in attribute '%s'", attr);
    } else if (errno != 0) {
      throw mjXError(elem, "unknown error in attribute '%s'", attr);
    }
  }

  if (!v.size()) {
    if (required) {
      throw mjXError(elem, "required attribute missing: '%s'", attr);
    } else {
      return std::nullopt;
    }
  }

  return v;
}

template std::optional<std::vector<double> >
mjXUtil::ReadAttrVec(XMLElement* elem, const char* attr, bool required);
template std::optional<std::vector<float> >
mjXUtil::ReadAttrVec(XMLElement* elem, const char* attr, bool required);
template std::optional<std::vector<int> >
mjXUtil::ReadAttrVec(XMLElement* elem, const char* attr, bool required);
template std::optional<std::vector<unsigned char> >
mjXUtil::ReadAttrVec(XMLElement* elem, const char* attr, bool required);



// if attribute is present, return attribute as a string
std::optional<std::string>
mjXUtil::ReadAttrStr(XMLElement* elem, const char* attr, bool required) {
  const char* pstr = elem->Attribute(attr);

  // check if attribute exists
  if (pstr == nullptr) {
    if (required) {
      throw mjXError(elem, "required attribute missing: '%s'", attr);
    } else {
      return std::nullopt;
    }
  }

  return std::string(pstr);
}

// if attribute is present, return attribute as a filename
std::optional<FilePath>
mjXUtil::ReadAttrFile(XMLElement* elem, const char* attr, const mjVFS* vfs,
                      const FilePath& dir, bool required) {
  auto maybe_str = ReadAttrStr(elem, attr, required);
  if (!maybe_str.has_value()) {
    return std::nullopt;
  }
  FilePath filename(maybe_str.value());
  return ResolveFilePath(elem, filename, dir, vfs);
}

// if attribute is present, return numerical value of attribute
template<typename T>
std::optional<T> mjXUtil::ReadAttrNum(XMLElement* elem, const char* attr,
                                      bool required) {
  auto maybe_arr = ReadAttrArr<T, 1>(elem, attr, required);
  if (!maybe_arr.has_value()) {
    return std::nullopt;
  }

  return maybe_arr.value()[0];
}

template std::optional<double>
mjXUtil::ReadAttrNum(XMLElement* elem, const char* attr, bool required);
template std::optional<float>
mjXUtil::ReadAttrNum(XMLElement* elem, const char* attr, bool required);
template std::optional<int>
mjXUtil::ReadAttrNum(XMLElement* elem, const char* attr, bool required);
template std::optional<unsigned char>
mjXUtil::ReadAttrNum(XMLElement* elem, const char* attr, bool required);



// read attribute "attr" of element "elem"
//  "len" is the number of floats or doubles to be read
//  the content is returned in "text", the numeric data in "data"
//  return number of elements found
template<typename T>
int mjXUtil::ReadAttr(XMLElement* elem, const char* attr, const int len,
                      T* data, std::string& text, bool required, bool exact) {
  auto maybe_vec = ReadAttrVec<T>(elem, attr, required);
  if (!maybe_vec.has_value()) {
    return 0;
  }

  // check if there is not enough data
  if (exact && maybe_vec->size() < len) {
    throw mjXError(elem, "attribute '%s' does not have enough data", attr);
  }

  // check if there is too much data
  if (maybe_vec->size() > len) {
    throw mjXError(elem, "attribute '%s' has too much data", attr);
  }

  std::copy(maybe_vec->begin(), maybe_vec->end(), data);
  return maybe_vec->size();
}

template int mjXUtil::ReadAttr(XMLElement* elem, const char* attr, int len,
                               double* data, std::string& text, bool required, bool exact);

template int mjXUtil::ReadAttr(XMLElement* elem, const char* attr, int len,
                               float* data, std::string& text, bool required, bool exact);

template int mjXUtil::ReadAttr(XMLElement* elem, const char* attr, int len,
                               int* data, std::string& text, bool required, bool exact);

template int mjXUtil::ReadAttr(XMLElement* elem, const char* attr, int len,
                               unsigned char* data, std::string& text, bool required,
                               bool exact);

// read quaternion attribute
//  throw error if identically zero
int mjXUtil::ReadQuat(XMLElement* elem, const char* attr, double* data, std::string& text,
                      bool required) {
  ReadAttr(elem, attr, /*len=*/4, data, text, required, /*exact=*/true);

  // check for 0 quaternion
  if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 0) {
    throw mjXError(elem, "zero quaternion is not allowed");
  }

  return 4;
}

// read DOUBLE array into C++ vector, return number read
int mjXUtil::ReadVector(XMLElement* elem, const char* attr,
                        std::vector<double>& vec, std::string& text, bool required) {
  auto maybe_vec = ReadAttrVec<double>(elem, attr, required);
  if (!maybe_vec.has_value()) {
    return 0;
  }

  vec = std::move(maybe_vec.value());
  return vec.size();
}



// read text field
bool mjXUtil::ReadAttrTxt(tinyxml2::XMLElement* elem, const char* attr,
                          std::string& text, bool required) {
  auto maybe_str = ReadAttrStr(elem, attr, required);
  if (!maybe_str.has_value()) {
    return false;
  }

  text = maybe_str.value();
  return true;
}

// read single int
bool mjXUtil::ReadAttrInt(XMLElement* elem, const char* attr, int* data, bool required) {
  auto maybe_int = ReadAttrNum<int>(elem, attr, required);
  if (!maybe_int.has_value()) {
    return false;
  }

  *data = maybe_int.value();
  return true;
}


// write vector<float> to string
void mjXUtil::Vector2String(std::string& txt, const std::vector<float>& vec, int ncol) {
  std::stringstream strm;

  for (size_t i=0; i < vec.size(); i++) {
    if (ncol && (i % ncol) == 0) {
      strm << "\n            ";
    } else if (i > 0) {
      strm << " ";
    }
    strm << vec[i];
  }

  txt = strm.str();
}

// find subelement with given name, make sure it is unique
XMLElement* mjXUtil::FindSubElem(XMLElement* elem, std::string name, bool required) {
  XMLElement* subelem = 0;

  XMLElement* iter = elem->FirstChildElement();
  while (iter) {
    // identify elements with given name
    if (name == iter->Value()) {
      // make sure name is not repeated
      if (subelem) {
        throw mjXError(subelem, "repeated element: '%s'", name.c_str());
      }

      // save found element
      subelem = iter;
    }

    // advance to next element
    iter = iter->NextSiblingElement();
  }

  if (required && !subelem) {
    throw mjXError(elem, "missing element: '%s'", name.c_str());
  }

  return subelem;
}



// find attribute, translate key into data, return true if found
bool mjXUtil::MapValue(XMLElement* elem, const char* attr, int* data,
                       const mjMap* map, int mapSz, bool required) {
  // get attribute text
  auto maybe_text = ReadAttrStr(elem, attr, required);
  if (!maybe_text.has_value()) {
    return false;
  }

  // find keyword in map
  int value = FindKey(map, mapSz, maybe_text.value());
  if (value < 0) {
    throw mjXError(elem, "invalid keyword: '%s'", maybe_text->c_str());
  }

  // copy
  *data = value;
  return true;
}



// find attribute, translate unique space-separated keys to data, return number of keys found
int mjXUtil::MapValues(XMLElement* elem, const char* attr, int* data,
                       const mjMap* map, int mapSz, bool required) {
  // get attribute text
  auto maybe_text = ReadAttrStr(elem, attr, required);
  if (!maybe_text.has_value()) {
    return 0;
  }

  std::string text = maybe_text.value();
  std::istringstream strm(text);
  std::string key;
  std::set<std::string> found_keys;
  int count = 0;

  while (strm >> key) {
    if (found_keys.count(key)) {
      throw mjXError(elem, "duplicate keyword: '%s'");
      return 0;
    }

    int value = FindKey(map, mapSz, key);
    if (value == -1) {
      throw mjXError(elem, "invalid keyword: '%s'");
      return 0;
    }

    found_keys.insert(key);
    data[count++] = value;
  }

  return count;
}



//---------------------------------- write functions -----------------------------------------------

// check if double is int
static bool isint(double x) {
  return ((std::abs(x - floor(x)) < 1E-12) || (std::abs(x - ceil(x)) < 1E-12));
}


// round to nearest int
static int Round(double x) {
  if (std::abs(x - floor(x)) < std::abs(x - ceil(x))) {
    return (int)floor(x);
  } else {
    return (int)ceil(x);
  }
}


// write attribute
template<typename T>
void mjXUtil::WriteAttr(XMLElement* elem, std::string name, int n, const T* data, const T* def,
                        bool trim) {
  // make sure all are defined
  if constexpr (std::is_floating_point_v<T>) {
    for (int i=0; i < n; i++) {
      if (std::isnan(data[i])) {
        return;
      }
    }
  }

  // skip default attributes
  if (SameVector(data, def, n)) {
    return;
  }

  // trim identical trailing default values
  if (trim) {
    while (n > 0 && data[n-1] == def[n-1]) {
      n--;
    }
  }

  // increase precision for testing
  std::stringstream stream;
  stream.precision(mujoco::_mjPRIVATE__get_xml_precision());

  // process all numbers
  for (int i=0; i < n; i++) {
    // add space between numbers
    if (i > 0) {
      stream << " ";
    }

    // append number
    double doubledata = static_cast<double>(data[i]);
    if (doubledata < INT_MAX && doubledata > -INT_MAX && isint(data[i])) {
      stream << Round(data[i]);
    } else {
      stream << data[i];
    }
  }

  // set attribute as string
  WriteAttrTxt(elem, name, stream.str());
}


template void mjXUtil::WriteAttr(XMLElement* elem, std::string name, int n,
                                 const double* data, const double* def, bool trim);

template void mjXUtil::WriteAttr(XMLElement* elem, std::string name, int n,
                                 const float* data, const float* def, bool trim);

template void mjXUtil::WriteAttr(XMLElement* elem, std::string name, int n,
                                 const int* data, const int* def, bool trim);

template void mjXUtil::WriteAttr(XMLElement* elem, std::string name, int n,
                                 const unsigned char* data,
                                 const unsigned char* def, bool trim);


// write vector<double> attribute, default = zero array
void mjXUtil::WriteVector(XMLElement* elem, std::string name, const std::vector<double>& vec) {
  // proceed only if non-zero found
  bool ok = false;
  for (size_t i=0; i < vec.size(); i++) {
    if (vec[i]) {
      ok = true;
      break;
    }
  }
  if (!ok) {
    return;
  }

  // write
  WriteAttr(elem, name, vec.size(), vec.data());
}


// write vector<double> attribute, default with same size
void mjXUtil::WriteVector(XMLElement* elem, std::string name, const std::vector<double>& vec,
                          const std::vector<double>& def) {
  // proceed only if non-zero found
  bool ok = false;
  for (size_t i=0; i < vec.size(); i++) {
    if (vec[i] != def[i]) {
      ok = true;
      break;
    }
  }
  if (!ok) {
    return;
  }

  // write
  WriteAttr(elem, name, vec.size(), vec.data());
}


// write attribute- string
void mjXUtil::WriteAttrTxt(XMLElement* elem, std::string name, std::string value) {
  // skip if value is empty
  if (value.empty()) {
    return;
  }

  // set attribute
  elem->SetAttribute(name.c_str(), value.c_str());
}



// write attribute- single int
void mjXUtil::WriteAttrInt(XMLElement* elem, std::string name, int data, int def) {
  // skip default
  if (data == def) {
    return;
  }

  elem->SetAttribute(name.c_str(), data);
}



// write attribute- keyword
void mjXUtil::WriteAttrKey(XMLElement* elem, std::string name,
                           const mjMap* map, int mapsz, int data, int def) {
  // skip default
  if (data == def) {
    return;
  }

  WriteAttrTxt(elem, name, FindValue(map, mapsz, data));
}


// write attribute- space-separated keywords
void mjXUtil::WriteAttrKeys(XMLElement* elem, std::string name, const mjMap* map,
                            int mapsz, int* data, int ndata, int def) {
  // skip default
  if (ndata == 1 && data[0] == def) {
    return;
  }

  std::string text = FindValue(map, mapsz, data[0]);
  for (int i = 1; i < ndata; ++i) {
    text += " " + FindValue(map, mapsz, data[i]);
  }

  WriteAttrTxt(elem, name, text);
}
