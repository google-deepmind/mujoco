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

#include "xml/xml.h"

#include <locale.h>

#if defined(__APPLE__) || defined(__FreeBSD__)
#include <xlocale.h>
#endif

#include <array>
#include <cstdio>
#include <string>
#include <string_view>
#include <unordered_set>

#include "tinyxml2.h"

#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include "cc/array_safety.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_resource.h"
#include "engine/engine_vfs.h"
#include "user/user_model.h"
#include "user/user_util.h"
#include "xml/xml_native_reader.h"
#include "xml/xml_native_writer.h"
#include "xml/xml_urdf.h"
#include "xml/xml_util.h"

namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

namespace mju = ::mujoco::util;

// We are using "locale-sensitive" sprintf to read and write XML.
// When MuJoCo is being used as a plug-in for an application that respects the system locale
// (e.g. Unity), the user's locale setting can affect the formatting of numbers into strings.
// Specifically, a number of European locales (e.g. de_DE) uses commas to as decimal separators.
// In order to ensure that XMLs are locale-inpendent, we temporarily switch to the "C" locale
// when handling. Since the standard C `setlocale` is not thread-safe, we instead use
// platform-specific extensions to override the locale only in the calling thread.
// See also https://github.com/google-deepmind/mujoco/issues/131.
#ifdef _WIN32
class LocaleOverride {
 public:
  LocaleOverride()
      : old_per_thread_locale_type_(_configthreadlocale(0)),
        old_locale_(setlocale(LC_ALL, nullptr)) {
    _configthreadlocale(_ENABLE_PER_THREAD_LOCALE);
    setlocale(LC_ALL, "C");
  }

  ~LocaleOverride() {
    setlocale(LC_ALL, old_locale_.c_str());
    _configthreadlocale(old_per_thread_locale_type_);
  }

 private:
  int old_per_thread_locale_type_;
  std::string old_locale_;
};
#else
class LocaleOverride {
 public:
  static locale_t PosixLocale() {
    static locale_t posix_locale = newlocale(LC_ALL_MASK, "C", 0);
    return posix_locale;
  }

  LocaleOverride() : old_locale_(uselocale(PosixLocale())) {}

  ~LocaleOverride() {
    uselocale(old_locale_);
  }

 private:
  locale_t old_locale_;
};
#endif

}  // namespace

// Main writer function - calls mjXWrite
std::string mjWriteXML(mjCModel* model, char* error, int error_sz) {
  LocaleOverride locale_override;

  // check for empty model
  if (!model) {
    mjCopyError(error, "Cannot write empty model", error_sz);
    return "";
  }

  mjXWriter writer;
  writer.SetModel(model);
  return writer.Write(error, error_sz);
}



// find include elements recursively, replace them with subtree from xml file
static void mjIncludeXML(mjXReader& reader, XMLElement* elem,
                         std::string_view dir, const mjVFS* vfs,
                         std::unordered_set<std::string>& included) {
  // capture directory defaults on first pass of XML tree
  if (!strcasecmp(elem->Value(), "compiler")) {
    auto assetdir_attr = mjXUtil::ReadAttrStr(elem, "assetdir");
    if (assetdir_attr.has_value()) {
      reader.SetAssetDir(assetdir_attr.value());
    }

    auto texturedir_attr = mjXUtil::ReadAttrStr(elem, "texturedir");
    if (texturedir_attr.has_value()) {
      reader.SetTextureDir(texturedir_attr.value());
    }

    auto meshdir_attr = mjXUtil::ReadAttrStr(elem, "meshdir");
    if (meshdir_attr.has_value()) {
      reader.SetMeshDir(meshdir_attr.value());
    }
  }

  //  not an include, recursively go through all children
  if (strcasecmp(elem->Value(), "include")) {
    XMLElement* child = elem->FirstChildElement();
    for (; child; child = child->NextSiblingElement()) {
      mjIncludeXML(reader, child, dir, vfs, included);
    }
    return;
  }

  // make sure include has no children
  if (!elem->NoChildren()) {
    throw mjXError(elem, "Include element cannot have children");
  }

  // get filename
  auto file_attr = mjXUtil::ReadAttrStr(elem, "file", true);
  if (!file_attr.has_value()) {
    throw mjXError(elem, "Include element missing file attribute");
  }
  std::string filename = file_attr.value();


  // block repeated include files
  if (included.find(filename) != included.end()) {
    throw mjXError(elem, "File '%s' already included", filename.c_str());
  }

  // TODO: b/325905702 - We have a messy wrapper here to remain backwards
  // compatible, which will be removed in the near future.
  std::string fullname;
  if (!mjuu_isabspath(filename)) {
    fullname = reader.ModelFileDir() + filename;
  } else {
    fullname = filename;
  }
  mjResource *resource = mju_openVfsResource(fullname.c_str(), vfs);
  if (!resource) {
    // load from provider or OS filesystem
    std::array<char, 1024> error;
    resource = mju_openResource(fullname.c_str(), error.data(), error.size());
    if (!resource) {
      if (!mjuu_isabspath(filename)) {
        fullname = std::string(dir) + filename;
      } else {
        fullname = filename;
      }

      // load from provider or OS filesystem
      std::array<char, 1024> error;
      resource = mju_openResource(fullname.c_str(), error.data(), error.size());
      if (!resource) {
        throw mjXError(elem, "%s", error.data());
      }
    }
  }

  if (!mjuu_isabspath(filename)) {
    filename = std::string(dir) + filename;
  }

  const char* include_dir = nullptr;
  int ninclude_dir = 0;
  mju_getResourceDir(resource, &include_dir, &ninclude_dir);
  std::string next_dir = std::string(include_dir, ninclude_dir);
  if (!mjuu_isabspath(filename)) {
    next_dir = std::string(dir) + next_dir;
  }
  elem->SetAttribute("dir", next_dir.data());

  const char* xmlstring = nullptr;
  int buffer_size = mju_readResource(resource, (const void**) &xmlstring);
  if (buffer_size < 0) {
    mju_closeResource(resource);
    throw mjXError(elem, "Error reading file '%s'", filename.c_str());
  } else if (!buffer_size) {
    mju_closeResource(resource);
    throw mjXError(elem, "Empty file '%s'", filename.c_str());
  }

  // load XML file or parse string
  XMLDocument doc;
  doc.Parse(xmlstring, buffer_size);

  // close resource
  mju_closeResource(resource);

  // check error
  if (doc.Error()) {
    char err[1000];
    mju::sprintf_arr(err, "XML parse error %d:\n%s\n", doc.ErrorID(), doc.ErrorStr());
    throw mjXError(elem, "Include error: '%s'", err);
  }

  // remember that file was included
  included.insert(filename);

  // get and check root element
  XMLElement* docroot = doc.RootElement();
  if (!docroot) {
    throw mjXError(elem, "Root element missing in file '%s'", filename.c_str());
  }

  // get and check first child
  XMLElement* eleminc = docroot->FirstChildElement();
  if (!eleminc) {
    throw mjXError(elem, "Empty include file '%s'", filename.c_str());
  }

  // get <include> element
  XMLElement* include = elem->ToElement();
  XMLDocument* include_doc = include->GetDocument();

  // clone first child of included document
  XMLNode* first = include->InsertFirstChild(eleminc->DeepClone(include_doc));

  // point to first
  XMLElement* child = first->ToElement();

  // insert remaining elements from included document as siblings
  eleminc = eleminc->NextSiblingElement();
  while (eleminc) {
    child = include->InsertAfterChild(child, eleminc->DeepClone(include_doc))->ToElement();
    eleminc = eleminc->NextSiblingElement();
  }

  // recursively run include
  child = include->FirstChildElement();
  for (; child; child = child->NextSiblingElement()) {
    mjIncludeXML(reader, child, next_dir, vfs, included);
  }
}



// Main parser function
mjCModel* mjParseXML(const char* filename, const mjVFS* vfs,
                     char* error, int error_sz) {
  LocaleOverride locale_override;

  // check arguments
  if (!filename) {
    if (error) {
      std::snprintf(error, error_sz, "mjParseXML: filename argument required\n");
    }
    return nullptr;
  }

  // clear
  mjCModel* model = 0;
  if (error) {
    error[0] = '\0';
  }

  // get data source
  const char* xmlstring = nullptr;
  mjResource* resource = mju_openVfsResource(filename, vfs);

  if (!resource) {
    // load from provider or fallback to OS filesystem
    std::array<char, 1024> rerror;
    resource = mju_openResource(filename, rerror.data(), rerror.size());
    if (!resource) {
      std::snprintf(error, error_sz, "mjParseXML: %s", rerror.data());
      return nullptr;
    }
  }

  int buffer_size = mju_readResource(resource, (const void**) &xmlstring);
  if (buffer_size < 0) {
    if (error) {
      std::snprintf(error, error_sz,
                    "mjParseXML: error reading file '%s'", filename);
    }
    mju_closeResource(resource);
    return nullptr;
  } else if (!buffer_size) {
    if (error) {
      std::snprintf(error, error_sz, "mjParseXML: empty file '%s'", filename);
    }
    mju_closeResource(resource);
    return nullptr;
  }


  // load XML file or parse string
  XMLDocument doc;
  doc.Parse(xmlstring, buffer_size);

  // error checking
  if (doc.Error()) {
    if (error) {
      snprintf(error, error_sz, "XML parse error %d:\n%s\n",
               doc.ErrorID(), doc.ErrorStr());
    }
    mju_closeResource(resource);
    return nullptr;
  }

  // get top-level element
  XMLElement* root = doc.RootElement();
  if (!root) {
    mju_closeResource(resource);
    mjCopyError(error, "XML root element not found", error_sz);
    return nullptr;
  }

  // create model, set filedir
  model = new mjCModel;
  const char* dir;
  int ndir = 0;
  mju_getResourceDir(resource, &dir, &ndir);
  if (dir != nullptr) {
    model->modelfiledir = std::string(dir, ndir);
  } else {
    model->modelfiledir = "";
  }

  // close resource
  mju_closeResource(resource);

  // parse with exceptions
  try {
    if (!strcasecmp(root->Value(), "mujoco")) {
      // find include elements, replace them with subtree from xml file
      std::unordered_set<std::string> included = {filename};
      mjXReader parser;
      parser.SetModelFileDir(model->modelfiledir);
      mjIncludeXML(parser, root, model->modelfiledir, vfs, included);

      // parse MuJoCo model
      parser.SetModel(model);
      parser.Parse(root);
    }

    else if (!strcasecmp(root->Value(), "robot")) {
      // parse URDF model
      mjXURDF parser;

      // set reasonable default for parsing a URDF
      // this is separate from the Parser to allow multiple URDFs to be loaded.
      model->spec.strippath = true;
      model->spec.fusestatic = true;
      model->spec.discardvisual = true;

      parser.SetModel(model);
      parser.Parse(root);
    }

    else {
      throw mjXError(0, "Unrecognized XML model type: '%s'", root->Value());
    }
  }

  // catch known errors
  catch (mjXError err) {
    mjCopyError(error, err.message, error_sz);
    delete model;
    return nullptr;
  }

  return model;
}
