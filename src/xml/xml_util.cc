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

#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cc/array_safety.h"
#include "xml/xml_util.h"
#include "xml/xml_numeric_format.h"

namespace {

using std::istringstream;
using std::size_t;
using std::string;
using std::stringstream;
using std::vector;
using tinyxml2::XMLAttribute;
using tinyxml2::XMLElement;

namespace mju = ::mujoco::util;

}  // namespace


//---------------------------------- utility functions ---------------------------------------------

// error string copy
void mjCopyError(char* dst, const char* src, int maxlen) {
  if (dst && maxlen>0) {
    strncpy(dst, src, maxlen);
    dst[maxlen-1] = 0;
  }
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

// constructor
mjXSchema::mjXSchema(const char* schema[][mjXATTRNUM], int nrow, bool checkptr) {
  // clear fields
  name.clear();
  type = '?';
  child.clear();
  attr.clear();
  error.clear();

  // checks nrow and first element
  if (nrow<1) {
    error = "number of rows must be positive";
    return;
  }
  if (schema[0][0][0]=='<' || schema[0][0][0]=='>') {
    error = "expected element, found bracket";
    return;
  }

  // check entire schema for null pointers
  if (checkptr) {
    char msg[100];

    for (int i=0; i<nrow; i++) {
      // base pointers
      if (!schema[i] || !schema[i][0]) {
        mju::sprintf_arr(msg, "null pointer found in row %d", i);
        error = msg;
        return;
      }

      // detect element
      if (schema[i][0][0]!='<' && schema[i][0][0]!='>') {
        // first 3 pointers required
        if (!schema[i][1] || !schema[i][2]) {
          mju::sprintf_arr(msg, "null pointer in row %d, element %s", i, schema[i][0]);
          error = msg;
          return;
        }

        // check type
        if (schema[i][1][0]!='!' && schema[i][1][0]!='?' &&
            schema[i][1][0]!='*' && schema[i][1][0]!='R') {
          mju::sprintf_arr(msg, "invalid type in row %d, element %s", i, schema[i][0]);
          error = msg;
          return;
        }

        // number of attributes
        int nattr = atoi(schema[i][2]);
        if (nattr<0 || nattr>mjXATTRNUM-3) {
          mju::sprintf_arr(msg,
                           "invalid number of attributes in row %d, element %s", i, schema[i][0]);
          error = msg;
          return;
        }

        // attribute pointers
        for (int j=0; j<nattr; j++) {
          if (!schema[i][3+j]) {
            mju::sprintf_arr(msg, "null attribute %d in row %d, element %s", j, i, schema[i][0]);
            error = msg;
            return;
          }
        }
      }
    }
  }

  // set name and type
  name = schema[0][0];
  type = schema[0][1][0];

  // set attributes
  int nattr = atoi(schema[0][2]);
  for (int i=0; i<nattr; i++) {
    attr.push_back(schema[0][3+i]);
  }

  // process sub-elements of complex element
  if (nrow>1) {
    // check for bracketed block
    if (schema[1][0][0]!='<' || schema[nrow-1][0][0]!='>') {
      error = "expected brackets after complex element";
      return;
    }

    // parse block into simple and complex elements, create children
    int start = 2;
    while (start < nrow-1) {
      int end = start;

      // look for bracketed block at start+1
      if (schema[start+1][0][0]=='<') {
        // look for corresponding closing bracket
        int cnt = 0;
        while (end <= nrow-1) {
          if (schema[end][0][0]=='<') {
            cnt++;
          } else if (schema[end][0][0]=='>') {
            cnt--;
            if (cnt==0) {
              break;
            }
          }

          end++;
        }

        // closing bracket not found
        if (end > nrow-1) {
          error = "matching closing bracket not found";
          return;
        }
      }

      // add element, check for error
      mjXSchema* elem = new mjXSchema(schema+start, end-start+1, false);
      child.push_back(elem);
      if (!elem->error.empty()) {
        error = elem->error;
        return;
      }

      // proceed with next subelement
      start = end+1;
    }
  }
}



// destructor
mjXSchema::~mjXSchema() {
  // delete children recursively
  for (unsigned int i=0; i<child.size(); i++) {
    delete child[i];
  }

  // clear fields
  child.clear();
  attr.clear();
  error.clear();
}



// get pointer to error message
string mjXSchema::GetError(void) {
  return error;
}



// print spaces
static void printspace(std::stringstream& str, int n, const char* space) {
  for (int i=0; i<n; i++) {
    str << space;
  }
}


// max
static int _max(int a, int b) {
  if (a>b) {
    return a;
  } else {
    return b;
  }
}


// print schema as text
void mjXSchema::Print(std::stringstream& str, int level) {
  int i;

  // replace body with (world)body
  string name1 = (name=="body" ? "(world)body" : name);

  // space, name, type
  printspace(str, 3*level, " ");
  str << name1 << " (" << type << ")";
  int baselen = 3*level + (int)name1.size() + 4;
  if (baselen<30) {
    printspace(str, 30-baselen, " ");
  }

  // attributes
  int cnt = _max(baselen, 30);
  for (i=0; i<(int)attr.size(); i++) {
    if (cnt>60) {
      str << "\n";
      printspace(str, (cnt = _max(30, baselen)), " ");

    }

    str << attr[i] << " ";
    cnt += (int)attr[i].size() + 1;
  }
  str << "\n";

  // children
  for (i=0; i<(int)child.size(); i++) {
    child[i]->Print(str, level+1);
  }
}



// print schema as HTML table
void mjXSchema::PrintHTML(std::stringstream& str, int level, bool pad) {
  int i;

  // replace body with (world)body
  string name1 = (name=="body" ? "(world)body" : name);

  // open table
  if (level==0) {
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
  str << "\t<td class=\"ty\">" << type << "</td>\n";

  // attributes
  str << "\t<td class=\"at\">";
  if (!attr.empty()) {
    for (i=0; i<(int)attr.size(); i++) {
      str << attr[i] << " ";
    }
  } else {
    str << "<span style=\"color:black\"><i>no attributes</i></span>";
  }
  str << "</td>\n</tr>\n";

  // children
  for (i=0; i<(int)child.size(); i++) {
    child[i]->PrintHTML(str, level+1, pad);
  }

  // close table
  if (level==0) {
    str << "</table>\n";
  }
}



// check for name match
bool mjXSchema::NameMatch(XMLElement* elem, int level) {
  // special handling of body and worldbody
  if (name=="body") {
    if (level==1 && !strcmp(elem->Value(), "worldbody")) {
      return true;
    }

    if (level!=1 && !strcmp(elem->Value(), "body")) {
      return true;
    }

    return false;
  }

  // regular check
  return (name==elem->Value());
}



// validator
XMLElement* mjXSchema::Check(XMLElement* elem, int level) {
  int i;
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
  while (attribute) {
    missing = true;
    for (i=0; i<(int)attr.size(); i++) {
      if (attr[i]==attribute->Name()) {
        missing = false;
        break;
      }
    }
    if (missing) {
      error = "unrecognized attribute: '" + string(attribute->Name()) + "'";
      return elem;
    }

    // next attribute
    attribute = attribute->Next();
  }

  // handle recursion
  if (type=='R') {
    // loop over sub-elements with same name
    sub = elem->FirstChildElement((const char*)name.c_str());
    while (sub) {
      // check sub-tree
      if ((bad = Check(sub, level+1))) {
        return bad;
      }

      // advance to next sub-element with same name
      sub = sub->NextSiblingElement((const char*)name.c_str());
    }
  }

  // clear reference counts
  for (i=0; i<(int)child.size(); i++) {
    child[i]->refcnt = 0;
  }

  // check sub-elements, update refcnt
  sub = elem->FirstChildElement();
  while (sub) {
    // find in child array, update refcnt
    missing = true;
    for (i=0; i<(int)child.size(); i++) {
      if (child[i]->NameMatch(sub, level+1)) {
        // check sub-tree
        if ((bad = child[i]->Check(sub, level+1))) {
          error = child[i]->error;
          return bad;
        }

        // mark found
        missing = false;
        child[i]->refcnt++;
        break;
      }
    }

    // missing, unless recursive
    if (missing && !(type=='R' && NameMatch(sub, level+1))) {
      error = "unrecognized element";
      return sub;
    }

    // advance to next sub-element
    sub = sub->NextSiblingElement();
  }

  // enforce sub-element types
  msg[0] = 0;
  for (i=0; i<(int)child.size(); i++) {
    switch (child[i]->type) {
    case '!':
      if (child[i]->refcnt != 1)
        mju::sprintf_arr(msg, "required sub-element '%s' found %d time(s)",
                         child[i]->name.c_str(), child[i]->refcnt);
      break;

    case '?':
      if (child[i]->refcnt > 1)
        mju::sprintf_arr(msg, "unique sub-element '%s' found %d time(s)",
                         child[i]->name.c_str(), child[i]->refcnt);
      break;

    default:
      break;
    }
  }

  // handle error
  if (msg[0]) {
    error = msg;
    return elem;
  } else {
    return 0;
  }
}



//---------------------------------- class mjXUtil implementation ----------------------------------

// compare two vectors: double
bool mjXUtil::SameVector(const double* vec1, const double* vec2, int n) {
  if (!vec1 || !vec2) {
    return false;
  }

  bool same = true;
  for (int i=0; i<n; i++) {
    if (fabs(vec1[i] - vec2[i]) > 1E-10) {
      same = false;
    }
  }

  return same;
}



// compare two vectors: double
bool mjXUtil::SameVector(const float* vec1, const float* vec2, int n) {
  if (!vec1 || !vec2) {
    return false;
  }

  bool same = true;
  for (int i=0; i<n; i++) {
    if (fabs(vec1[i] - vec2[i]) > 1E-7) {
      same = false;
    }
  }

  return same;
}



// find string in map, return corresponding integer (-1: not found)
int mjXUtil::FindKey(const mjMap* map, int mapsz, string key) {
  for (int i=0; i<mapsz; i++) {
    if (map[i].key == key) {
      return map[i].value;
    }
  }

  return -1;
}



// find integer in map, return corresponding string ("": not found)
string mjXUtil::FindValue(const mjMap* map, int mapsz, int value) {
  for (int i=0; i<mapsz; i++) {
    if (map[i].value == value) {
      return map[i].key;
    }
  }

  return "";
}



// read attribute "attr" of element "elem"
//  "len" is the number of floats or doubles to be read
//  the content is returned in "text", the numeric data in "data"
//  return true if attribute found, false if not found and not required
int mjXUtil::ReadAttr(XMLElement* elem, const char* attr, const int len,
                      double* data, string& text, bool required, bool exact) {
  const char* pstr = elem->Attribute(attr);

  // check if attribute exists
  if (!pstr) {
    if (required) {
      throw mjXError(elem, "required attribute missing: '%s'", attr);
    } else {
      return 0;
    }
  }

  // convert to string
  text = string(pstr);

  // get input stream
  istringstream strm(text);

  // read numbers
  int i;
  for (i=0; i<len; i++) {
    strm >> data[i];
    if (strm.eof()) {
      i++;
      break;
    } else if (strm.bad()) {
      throw mjXError(elem, "problem reading attribute '%s'", attr);
    }
  }

  // determine available length
  int available = i;
  if (strm.good()) {
    double dummy;
    strm >> dummy;
    if (!strm.bad() && !strm.fail()) {
      available++;
    }
  }

  // check
  if (exact && available<len) {
    throw mjXError(elem, "attribute '%s' does not have enough data", attr);
  }
  if (available>len) {
    throw mjXError(elem, "attribute '%s' has too much data", attr);
  }

  return i;
}



// float version
int mjXUtil::ReadAttr(XMLElement* elem, const char* attr, const int len,
                      float* data, string& text, bool required, bool exact) {
  const char* pstr = elem->Attribute(attr);

  // check if attribute exists
  if (!pstr) {
    if (required) {
      throw mjXError(elem, "required attribute missing: '%s'", attr);
    } else {
      return 0;
    }
  }

  // convert to string, remove trailing white space
  text = string(pstr);
  text.erase(text.find_last_not_of(" \t\n\r\f\v") + 1);

  // get input stream
  istringstream strm(text);

  // read numbers
  int i;
  for (i=0; i<len; i++) {
    strm >> data[i];
    if (strm.eof()) {
      i++;
      break;
    } else if (strm.bad()) {
      throw mjXError(elem, "problem reading attribute '%s'", attr);
    }
  }

  // determine available length
  int available = i;
  if (strm.good()) {
    float dummy;
    strm >> dummy;
    if (!strm.bad() && !strm.fail()) {
      available++;
    }
  }

  // check
  if (exact && available<len) {
    throw mjXError(elem, "attribute '%s' does not have enough data", attr);
  }
  if (available>len) {
    throw mjXError(elem, "attribute '%s' has too much data", attr);
  }

  return i;
}



// int version
int mjXUtil::ReadAttr(XMLElement* elem, const char* attr, const int len,
                      int* data, string& text, bool required, bool exact) {
  const char* pstr = elem->Attribute(attr);

  // check if attribute exists
  if (!pstr) {
    if (required) {
      throw mjXError(elem, "required attribute missing: '%s'", attr);
    } else {
      return 0;
    }
  }

  // convert to string, remove trailing white space
  text = string(pstr);
  text.erase(text.find_last_not_of(" \t\n\r\f\v") + 1);

  // get input stream
  istringstream strm(text);

  // read numbers
  int i;
  for (i=0; i<len; i++) {
    strm >> data[i];
    if (strm.eof()) {
      i++;
      break;
    } else if (strm.bad()) {
      throw mjXError(elem, "problem reading attribute '%s'", attr);
    }
  }

  // determine available length
  int available = i;
  if (strm.good()) {
    mjtByte dummy;
    strm >> dummy;
    if (!strm.bad() && !strm.fail()) {
      available++;
    }
  }

  // check
  if (exact && available<len) {
    throw mjXError(elem, "attribute '%s' does not have enough data", attr);
  }
  if (available>len) {
    throw mjXError(elem, "attribute '%s' has too much data", attr);
  }

  return i;
}



// byte version
int mjXUtil::ReadAttr(XMLElement* elem, const char* attr, const int len,
                      mjtByte* data, string& text, bool required, bool exact) {
  const char* pstr = elem->Attribute(attr);

  // check if attribute exists
  if (!pstr) {
    if (required) {
      throw mjXError(elem, "required attribute missing: '%s'", attr);
    } else {
      return 0;
    }
  }

  // convert to string, remove trailing white space
  text = string(pstr);
  text.erase(text.find_last_not_of(" \t\n\r\f\v") + 1);

  // get input stream
  istringstream strm(text);

  // read numbers
  int i, tmp;
  for (i=0; i<len; i++) {
    strm >> tmp;
    data[i] = (mjtByte)(tmp & 0xFF);
    if (strm.eof()) {
      i++;
      break;
    } else if (strm.bad()) {
      throw mjXError(elem, "problem reading attribute '%s'", attr);
    }
  }

  // determine available length
  int available = i;
  if (strm.good()) {
    mjtByte dummy;
    strm >> dummy;
    if (!strm.bad() && !strm.fail()) {
      available++;
    }
  }

  // check
  if (exact && available<len) {
    throw mjXError(elem, "attribute '%s' does not have enough data", attr);
  }
  if (available>len) {
    throw mjXError(elem, "attribute '%s' has too much data", attr);
  }

  return i;
}



// read DOUBLE array into C++ vector, return number read
int mjXUtil::ReadVector(XMLElement* elem, const char* attr,
                        vector<double>& vec, string& text, bool required) {
  double buffer[1000];
  int n = ReadAttr(elem, attr, 1000, buffer, text, required, false);
  if (n>0) {
    vec.resize(n);
    memcpy(vec.data(), buffer, n*sizeof(double));
  }
  return n;
}



// read text field
bool mjXUtil::ReadAttrTxt(XMLElement* elem, const char* attr, string& text, bool required) {
  const char* pstr = elem->Attribute(attr);

  // check if attribute exists
  if (!pstr) {
    if (required) {
      throw mjXError(elem, "required attribute missing: '%s'", attr);
    } else {
      return false;
    }
  }

  // read text
  text = string(pstr);

  return true;
}



// read single int
bool mjXUtil::ReadAttrInt(XMLElement* elem, const char* attr, int* data, bool required) {
  const char* pstr = elem->Attribute(attr);

  // check if attribute exists
  if (!pstr) {
    if (required) {
      throw mjXError(elem, "required attribute missing: '%s'", attr);
    } else {
      return false;
    }
  }

  // convert to int, check
  int buffer[2] = {0, 0};
  if (sscanf(pstr, "%d", buffer) != 1) {
    throw mjXError(elem, "single int expected in attribute %s", attr);
  }

  // copy data
  *data = buffer[0];
  return true;
}



// read vector<float> from string
void mjXUtil::String2Vector(const string& txt, vector<float>& vec) {
  stringstream strm(txt);
  vec.clear();

  while (!strm.eof()) {
    float num;
    strm >> num;
    if (strm.fail()) {
      break;
    } else {
      vec.push_back(num);
    }
  }
}



// read vector<int> from string
void mjXUtil::String2Vector(const string& txt, vector<int>& vec) {
  stringstream strm(txt);
  vec.clear();

  while (!strm.eof()) {
    int num;
    strm >> num;
    if (strm.fail()) {
      break;
    } else {
      vec.push_back(num);
    }
  }
}



// write vector<float> to string
void mjXUtil::Vector2String(string& txt, const vector<float>& vec) {
  stringstream strm;

  for (size_t i=0; i<vec.size(); i++) {
    if (i>0) {
      strm << " ";
    }
    strm << vec[i];
  }

  txt = strm.str();
}



// write vector<int> to string
void mjXUtil::Vector2String(string& txt, const vector<int>& vec) {
  stringstream strm;

  for (size_t i=0; i<vec.size(); i++) {
    if (i>0) {
      strm << " ";
    }
    strm << vec[i];
  }

  txt = strm.str();
}



// find subelement with given name, make sure it is unique
XMLElement* mjXUtil::FindSubElem(XMLElement* elem, string name, bool required) {
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



// find attribute, translate key, return int value
bool mjXUtil::MapValue(XMLElement* elem, const char* attr, int* data,
                       const mjMap* map, int mapSz, bool required) {
  // get attribute text
  string text;
  if (!ReadAttrTxt(elem, attr, text, required)) {
    return false;
  }

  // find keyword in map
  int value = FindKey(map, mapSz, text);
  if (value<0) {
    throw mjXError(elem, "invalid keyword: '%s'", text.c_str());
  }

  // copy
  *data = value;
  return true;
}



//---------------------------------- write functions -----------------------------------------------

// check if double is int
static bool isint(double x) {
  return ((fabs(x - floor(x)) < 1E-12) || (fabs(x - ceil(x)) < 1E-12));
}


// round to nearest int
static int Round(double x) {
  if (fabs(x - floor(x)) < fabs(x - ceil(x))) {
    return (int)floor(x);
  } else {
    return (int)ceil(x);
  }
}


// write attribute- double
void mjXUtil::WriteAttr(XMLElement* elem, string name, int n, double* data,
                        const double* def) {
  char buf[100];
  string value;
  value.clear();

  // make sure all are defined
  for (int i=0; i<n; i++) {
    if (std::isnan(data[i])) {
      return;
    }
  }

  // skip default attributes
  if (SameVector(data, def, n)) {
    return;
  }

  // process all numbers
  for (int i=0; i<n; i++) {
    // add space between numbers
    if (i>0) {
      value = value + " ";
    }

    // write integer or float
    if (isint(data[i])) {
      mju::sprintf_arr(buf, "%d", Round(data[i]));
    } else {
      mju::sprintf_arr(buf, mujoco::_mjPRIVATE__get_xml_precision(), data[i]);
    }

    // append number
    value = value + buf;
  }

  // set attribute as string
  WriteAttrTxt(elem, name, value);
}



// write attribute- float
void mjXUtil::WriteAttr(XMLElement* elem, string name, int n, float* data,
                        const float* def) {
  char buf[100];
  string value;
  value.clear();

  // skip default attributes
  if (SameVector(data, def, n)) {
    return;
  }

  // process all numbers
  for (int i=0; i<n; i++) {
    // add space between numbers
    if (i>0) {
      value = value + " ";
    }

    // write integer or float
    if (isint(data[i])) {
      mju::sprintf_arr(buf, "%d", Round(data[i]));
    } else {
      mju::sprintf_arr(buf, "%g", data[i]);
    }

    // append number
    value = value + buf;
  }

  // set attribute as string
  WriteAttrTxt(elem, name, value);
}



// write attribute- byte
void mjXUtil::WriteAttr(XMLElement* elem, string name, int n, mjtByte* data,
                        const mjtByte* def) {
  char buf[100];
  string value;
  value.clear();

  // skip default attributes
  if (def) {
    bool skip = true;
    for (int i=0; i<n; i++)
      if (data[i] != def[i]) {
        skip = false;
      }

    if (skip) {
      return;
    }
  }

  // process all numbers
  for (int i=0; i<n; i++) {
    // add space between numbers
    if (i>0) {
      value = value + " ";
    }

    // write integer
    mju::sprintf_arr(buf, "%d", data[i]);

    // append number
    value = value + buf;
  }

  // set attribute as string
  WriteAttrTxt(elem, name, value);
}


// write attribute- int
void mjXUtil::WriteAttr(XMLElement* elem, string name, int n, int* data,
                        const int* def) {
  char buf[100];
  string value;
  value.clear();

  // skip default attributes
  if (def) {
    bool skip = true;
    for (int i=0; i<n; i++) {
      if (data[i] != def[i]) {
        skip = false;
      }
    }
    if (skip) {
      return;
    }
  }

  // process all numbers
  for (int i=0; i<n; i++) {
    // add space between numbers
    if (i>0) {
      value = value + " ";
    }

    // write integer
    mju::sprintf_arr(buf, "%d", data[i]);

    // append number
    value = value + buf;
  }

  // set attribute as string
  WriteAttrTxt(elem, name, value);
}


// write vector<double> attribute, default = zero array
void mjXUtil::WriteVector(XMLElement* elem, string name, vector<double>& vec) {
  // proceed only if non-zero found
  bool ok = false;
  for (size_t i=0; i<vec.size(); i++) {
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
void mjXUtil::WriteVector(XMLElement* elem, string name, vector<double>& vec,
                          vector<double>& def) {
  // proceed only if non-zero found
  bool ok = false;
  for (size_t i=0; i<vec.size(); i++) {
    if (vec[i]!=def[i]) {
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
void mjXUtil::WriteAttrTxt(XMLElement* elem, string name, string value) {
  // skip if value is empty
  if (value.empty()) {
    return;
  }

  // set attribute
  elem->SetAttribute(name.c_str(), value.c_str());
}



// write attribute- single int
void mjXUtil::WriteAttrInt(XMLElement* elem, string name, int data, int def) {
  // skip default
  if (data==def) {
    return;
  }

  elem->SetAttribute(name.c_str(), data);
}



// write attribute- keyword
void mjXUtil::WriteAttrKey(XMLElement* elem, string name,
                           const mjMap* map, int mapsz, int data, int def) {
  // skip default
  if (data==def) {
    return;
  }

  WriteAttrTxt(elem, name, FindValue(map, mapsz, data));
}
