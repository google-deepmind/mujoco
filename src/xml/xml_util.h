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


#ifndef MUJOCO_SRC_XML_XML_UTIL_H_
#define MUJOCO_SRC_XML_XML_UTIL_H_

// stl
#include <string>
#include <vector>
#include <sstream>

#include <mujoco/mjmodel.h>


// TinyXML
#include "tinyxml2.h"


// error string copy
void mjCopyError(char* dst, const char* src, int maxlen);


// XML Error info
class mjXError {
 public:
  mjXError(const tinyxml2::XMLElement* elem = 0,
           const char* msg = 0,
           const char* str = 0,
           int pos = 0);
  ~mjXError() = default;

  char message[1000];             // error message
};


// max number of attribute fields in schema (plus 3)
#define mjXATTRNUM 36


// Custom XML file validation
class mjXSchema {
 public:
  mjXSchema(const char* schema[][mjXATTRNUM],         // constructor
            int nrow, bool checkptr = true);
  ~mjXSchema();                                       // destructor

  std::string GetError(void);                         // return error
  void Print(std::stringstream& str, int level);      // print schema
  void PrintHTML(std::stringstream& str, int level, bool pad);

  bool NameMatch(tinyxml2::XMLElement* elem, int level);               // does name match
  tinyxml2::XMLElement* Check(tinyxml2::XMLElement* elem, int level);  // validator

 private:
  std::string name;                   // element name
  char type;                          // element type: '?', '!', '*', 'R'
  std::vector<std::string> attr;      // allowed attributes
  std::vector<mjXSchema*> child;      // allowed child elements

  int refcnt;                         // refcount used for validation
  std::string error;                  // error from constructor or Check
};


// key(string) : value(int) map
struct _mjMap {
  const char* key;
  int value;
};
typedef struct _mjMap mjMap;


// XML read and write utility functions
class mjXUtil {
 public:
  mjXUtil() = default;
  virtual ~mjXUtil() = default;

  // compare two vectors
  template<typename T>
  static bool SameVector(const T* vec1, const T* vec2, int n);

  // find key in map, return value (-1: not found)
  static int FindKey(const mjMap* map, int mapsz, std::string key);

  // find value in map, return key ("": not found)
  static std::string FindValue(const mjMap* map, int mapsz, int value);

  // read any type from attribute, return number read
  template<typename T>
  static int ReadAttr(tinyxml2::XMLElement* elem, const char* attr, const int len,
                      T* data, std::string& text,
                      bool required = false, bool exact = true);

  // read DOUBLE array into C++ vector, return number read
  static int ReadVector(tinyxml2::XMLElement* elem, const char* attr,
                        std::vector<double>& vec, std::string& text, bool required = false);

  // read text attribute
  static bool ReadAttrTxt(tinyxml2::XMLElement* elem, const char* attr, std::string& text,
                          bool required = false);

  // read int attribute
  static bool ReadAttrInt(tinyxml2::XMLElement* elem, const char* attr, int* data,
                          bool required = false);

  // read vector<float> from string
  static void String2Vector(const std::string& txt, std::vector<float>& vec);

  // read vector<int> from string
  static void String2Vector(const std::string& txt, std::vector<int>& vec);

  // write vector<float> to string
  static void Vector2String(std::string& txt, const std::vector<float>& vec);

  // write vector<int> to string
  static void Vector2String(std::string& txt, const std::vector<int>& vec);

  // find subelement with given name, make sure it is unique
  static tinyxml2::XMLElement* FindSubElem(tinyxml2::XMLElement* elem, std::string name,
                                           bool required = false);

  // find attribute, translate key, return int value
  static bool MapValue(tinyxml2::XMLElement* elem, const char* attr, int* data,
                       const mjMap* map, int mapSz, bool required = false);

  // write attribute- any type
  template<typename T>
  static void WriteAttr(tinyxml2::XMLElement* elem, std::string name, int n, T* data,
                        const T* def = 0);

  // write vector<double> attribute, with and without default
  static void WriteVector(tinyxml2::XMLElement* elem, std::string name, std::vector<double>& vec);
  static void WriteVector(tinyxml2::XMLElement* elem, std::string name, std::vector<double>& vec,
                          std::vector<double>& def);

  // write attribute- string
  static void WriteAttrTxt(tinyxml2::XMLElement* elem, std::string name, std::string value);

  // write attribute- single int
  static void WriteAttrInt(tinyxml2::XMLElement* elem, std::string name, int data, int def = -12345);

  // write attribute- keyword
  static void WriteAttrKey(tinyxml2::XMLElement* elem, std::string name,
                           const mjMap* map, int mapsz, int data, int def = -12345);
};

#endif  // MUJOCO_SRC_XML_XML_UTIL_H_
