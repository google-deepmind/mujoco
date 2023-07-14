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

#include <string>

#include "cc/array_safety.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_resource.h"
#include "user/user_model.h"
#include "user/user_util.h"
#include "xml/xml_native_reader.h"
#include "xml/xml_native_writer.h"
#include "xml/xml_urdf.h"
#include "xml/xml_util.h"
#include "tinyxml2.h"

namespace {

using std::string;
using std::vector;
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
// See also https://github.com/deepmind/mujoco/issues/131.
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
string mjWriteXML(mjCModel* model, char* error, int error_sz) {
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
static XMLElement* mjIncludeXML(XMLElement* elem, string dir,
                                int vfs_provider, vector<string>& included) {
  // include element: process
  if (!strcasecmp(elem->Value(), "include")) {
    // make sure include has no children
    if (!elem->NoChildren()) {
      throw mjXError(elem, "Include element cannot have children");
    }

    // get filename
    string filename;
    mjXUtil::ReadAttrTxt(elem, "file", filename, true);
    filename = dir + filename;

    // block repeated include files
    for (size_t i=0; i<included.size(); i++) {
      if (!strcasecmp(included[i].c_str(), filename.c_str())) {
        throw mjXError(elem, "File '%s' already included", filename.c_str());
      }
    }

    // get data source
    mjResource *resource = nullptr;
    const char* xmlstring = nullptr;
    if ((resource = mju_openResource(filename.c_str(), vfs_provider)) == nullptr) {
      // load from OS filesystem
      if (!vfs_provider || (resource = mju_openResource(filename.c_str(), 0)) == nullptr) {
        throw mjXError(elem, "Could not open file '%s'", filename.c_str());
      }
    }

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
    included.push_back(filename);

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

    // get parent of <include>
    XMLElement* parent = (XMLElement*)elem->Parent();

    // clone first child of included document, insert it after <include>
    XMLNode* first = parent->InsertAfterChild(elem, eleminc->DeepClone(parent->GetDocument()));

    // delete <include> element, point to first
    parent->DeleteChild(elem);
    elem = first->ToElement();

    // insert remaining elements from included document as siblings
    eleminc = eleminc->NextSiblingElement();
    while (eleminc) {
      elem = (XMLElement*)parent->InsertAfterChild(elem, eleminc->DeepClone(parent->GetDocument()));
      eleminc = eleminc->NextSiblingElement();
    }

    // run XMLInclude on first new child
    return mjIncludeXML(first->ToElement(), dir, vfs_provider, included);
  }

  // otherwise check all child elements, return self
  else {
    XMLElement* child = elem->FirstChildElement();
    while (child) {
      child = mjIncludeXML(child, dir, vfs_provider, included);
      if (child) {
        child = child->NextSiblingElement();
      }
    }
    return elem;
  }
}



// Main parser function
mjCModel* mjParseXML(const char* filename, int vfs_provider, char* error, int error_sz) {
  LocaleOverride locale_override;

  // check arguments
  if (!filename) {
    if (error) {
      snprintf(error, error_sz, "mjParseXML: filename argument required\n");
    }
    return nullptr;
  }

  // clear
  mjCModel* model = 0;
  if (error) {
    error[0] = '\0';
  }

  // get data source
  mjResource* resource = nullptr;
  const char* xmlstring = nullptr;
  if ((resource = mju_openResource(filename, vfs_provider)) == nullptr) {
    // load from OS filesystem
    if (!vfs_provider || (resource = mju_openResource(filename, 0)) == nullptr) {
      if (error) {
        snprintf(error, error_sz, "mjParseXML: could not open file '%s'", filename);
      }
      return nullptr;
    }
  }

  int buffer_size = mju_readResource(resource, (const void**) &xmlstring);
  if (buffer_size < 0) {
    if (error) {
      snprintf(error, error_sz, "mjParseXML: error reading file '%s'", filename);
    }
    mju_closeResource(resource);
    return nullptr;
  } else if (!buffer_size) {
    if (error) {
      snprintf(error, error_sz, "mjParseXML: empty file '%s'", filename);
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
      vector<string> included;
      included.push_back(filename);
      mjIncludeXML(root, model->modelfiledir, vfs_provider, included);

      // parse MuJoCo model
      mjXReader parser;
      parser.SetModel(model);
      parser.Parse(root);
    }

    else if (!strcasecmp(root->Value(), "robot")) {
      // parse URDF model
      mjXURDF parser;

      // set reasonable default for parsing a URDF
      // this is separate from the Parser to allow multiple URDFs to be loaded.
      model->strippath = true;
      model->fusestatic = true;

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
