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

#ifndef MUJOCO_SRC_XML_XML_NATIVE_WRITER_H_
#define MUJOCO_SRC_XML_XML_NATIVE_WRITER_H_

#include <string>

#include "xml/xml_base.h"
#include "tinyxml2.h"

class mjXWriter : public mjXBase {
 public:
  mjXWriter();                                        // constructor
  virtual ~mjXWriter() = default;                     // destructor
  void Write(FILE* fp);                               // write XML document

 private:
  // insert end child with given name, return child
  tinyxml2::XMLElement* InsertEnd(tinyxml2::XMLElement* parent, const char* name);

  // XML section writers
  void Compiler(tinyxml2::XMLElement* root);                    // compiler section
  void Option(tinyxml2::XMLElement* root);                      // option section
  void Size(tinyxml2::XMLElement* root);                        // size section
  void Visual(tinyxml2::XMLElement* root);                      // visual section
  void Statistic(tinyxml2::XMLElement* root);                   // statistic section
  void Default(tinyxml2::XMLElement* root, mjCDef* def);        // default section
  void Custom(tinyxml2::XMLElement* root);                      // custom section
  void Asset(tinyxml2::XMLElement* root);                       // asset section
  void Body(tinyxml2::XMLElement* elem, mjCBody* body);         // body/world section
  void Contact(tinyxml2::XMLElement* root);                     // contact section
  void Equality(tinyxml2::XMLElement* root);                    // equality constraint section
  void Tendon(tinyxml2::XMLElement* root);                      // tendon section
  void Actuator(tinyxml2::XMLElement* root);                    // actuator section
  void Sensor(tinyxml2::XMLElement* root);                      // sensor section
  void Keyframe(tinyxml2::XMLElement* root);                    // keyframe section

  // single element writers, used in defaults and main body
  void OneMesh(tinyxml2::XMLElement* elem,      mjCMesh* pmesh,         mjCDef* def);
  void OneSkin(tinyxml2::XMLElement* elem,      mjCSkin* pskin);
  void OneMaterial(tinyxml2::XMLElement* elem,  mjCMaterial* pmaterial, mjCDef* def);
  void OneJoint(tinyxml2::XMLElement* elem,     mjCJoint* pjoint,       mjCDef* def);
  void OneGeom(tinyxml2::XMLElement* elem,      mjCGeom* pgeom,         mjCDef* def);
  void OneSite(tinyxml2::XMLElement* elem,      mjCSite* psite,         mjCDef* def);
  void OneCamera(tinyxml2::XMLElement* elem,    mjCCamera* pcamera,     mjCDef* def);
  void OneLight(tinyxml2::XMLElement* elem,     mjCLight* plight,       mjCDef* def);
  void OnePair(tinyxml2::XMLElement* elem,      mjCPair* ppair,         mjCDef* def);
  void OneEquality(tinyxml2::XMLElement* elem,  mjCEquality* pequality, mjCDef* def);
  void OneTendon(tinyxml2::XMLElement* elem,    mjCTendon* ptendon,     mjCDef* def);
  void OneActuator(tinyxml2::XMLElement* elem,  mjCActuator* pactuator, mjCDef* def);

  bool writingdefaults;                       // true during defaults write
};

#endif  // MUJOCO_SRC_XML_XML_NATIVE_WRITER_H_
