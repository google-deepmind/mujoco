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

#ifndef MUJOCO_SRC_XML_XML_NATIVE_READER_H_
#define MUJOCO_SRC_XML_XML_NATIVE_READER_H_

#include <sstream>

#include "user/user_model.h"
#include "xml/xml_base.h"
#include "tinyxml2.h"

class mjXReader : public mjXBase {
 public:
  mjXReader();                                                         // constructor
  virtual ~mjXReader() = default;                                      // destructor

  void Parse(tinyxml2::XMLElement* root);                              // parse XML document
  void PrintSchema(std::stringstream& str, bool html, bool pad);       // print text or HTML schema

  // XML sections embedded in all formats
  static void Compiler(tinyxml2::XMLElement* section, mjCModel* mod);  // compiler section
  static void Option(tinyxml2::XMLElement* section, mjOption* opt);    // option section
  static void Size(tinyxml2::XMLElement* section, mjCModel* mod);      // size section

 private:
  // XML section specific to MJCF
  void Default(tinyxml2::XMLElement* section,   int parentid);         // default section
  void Custom(tinyxml2::XMLElement* section);                          // custom section
  void Visual(tinyxml2::XMLElement* section);                          // visual section
  void Statistic(tinyxml2::XMLElement* section);                       // statistic section
  void Asset(tinyxml2::XMLElement* section);                           // asset section
  void Body(tinyxml2::XMLElement* section, mjCBody* pbody);            // body/world section
  void Contact(tinyxml2::XMLElement* section);                         // contact section
  void Equality(tinyxml2::XMLElement* section);                        // equality section
  void Tendon(tinyxml2::XMLElement* section);                          // tendon section
  void Actuator(tinyxml2::XMLElement* section);                        // actuator section
  void Sensor(tinyxml2::XMLElement* section);                          // sensor section
  void Keyframe(tinyxml2::XMLElement* section);                        // keyframe section

  // single element parsers, used in defaults and main body
  void OneMesh(tinyxml2::XMLElement* elem, mjCMesh* pmesh);
  void OneSkin(tinyxml2::XMLElement* elem, mjCSkin* pskin);
  void OneMaterial(tinyxml2::XMLElement* elem, mjCMaterial* pmaterial);
  void OneJoint(tinyxml2::XMLElement* elem, mjCJoint* pjoint);
  void OneGeom(tinyxml2::XMLElement* elem, mjCGeom* pgeom);
  void OneSite(tinyxml2::XMLElement* elem, mjCSite* psite);
  void OneCamera(tinyxml2::XMLElement* elem, mjCCamera* pcamera);
  void OneLight(tinyxml2::XMLElement* elem, mjCLight* plight);
  void OnePair(tinyxml2::XMLElement* elem, mjCPair* ppair);
  void OneEquality(tinyxml2::XMLElement* elem, mjCEquality* pequality);
  void OneTendon(tinyxml2::XMLElement* elem, mjCTendon* ptendon);
  void OneActuator(tinyxml2::XMLElement* elem, mjCActuator* pactuator);
  void OneComposite(tinyxml2::XMLElement* elem, mjCBody* pbody, mjCDef* def);

  mjXSchema schema;                                                   // schema used for validation
  mjCDef* GetClass(tinyxml2::XMLElement* section);                    // get default class name
  static void GetXMLPos(tinyxml2::XMLElement* elem, mjCBase* obj);    // get xml position

  bool readingdefaults; // true while reading defaults
};

#endif  // MUJOCO_SRC_XML_XML_NATIVE_READER_H_
