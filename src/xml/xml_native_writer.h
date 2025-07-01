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

#include <cstdlib>
#include <string>
#include <string_view>

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include "user/user_objects.h"
#include "xml/xml_base.h"
#include "tinyxml2.h"

class mjXWriter : public mjXBase {
 public:
  mjXWriter();                                        // constructor
  virtual ~mjXWriter() = default;                     // destructor
  void SetModel(mjSpec* _spec, const mjModel* m = nullptr);

  // write XML document to string
  std::string Write(char *error, std::size_t error_sz);

 private:
  // insert end child with given name, return child
  tinyxml2::XMLElement* InsertEnd(tinyxml2::XMLElement* parent, const char* name);

  // compiled model
  mjCModel* model = 0;

  // XML section writers
  void Compiler(tinyxml2::XMLElement* root);                              // compiler section
  void Option(tinyxml2::XMLElement* root);                                // option section
  void Size(tinyxml2::XMLElement* root);                                  // size section
  void Visual(tinyxml2::XMLElement* root);                                // visual section
  void Statistic(tinyxml2::XMLElement* root);                             // statistic section
  void Default(tinyxml2::XMLElement* root, mjCDef* def);                  // default section
  void Extension(tinyxml2::XMLElement* root);                             // extension section
  void Custom(tinyxml2::XMLElement* root);                                // custom section
  void Asset(tinyxml2::XMLElement* root);                                 // asset section
  void Contact(tinyxml2::XMLElement* root);                               // contact section
  void Deformable(tinyxml2::XMLElement* root);                            // deformable section
  void Equality(tinyxml2::XMLElement* root);                              // equality section
  void Tendon(tinyxml2::XMLElement* root);                                // tendon section
  void Actuator(tinyxml2::XMLElement* root);                              // actuator section
  void Sensor(tinyxml2::XMLElement* root);                                // sensor section
  void Keyframe(tinyxml2::XMLElement* root);                              // keyframe section

  // body/world section
  void Body(tinyxml2::XMLElement* elem, mjCBody* body, mjCFrame* frame, std::string_view childclass = "");

  // single element writers, used in defaults and main body
  void OneFlex(tinyxml2::XMLElement* elem, const mjCFlex* pflex);
  void OneMesh(tinyxml2::XMLElement* elem, const mjCMesh* pmesh,         mjCDef* def);
  void OneSkin(tinyxml2::XMLElement* elem, const mjCSkin* pskin);
  void OneMaterial(tinyxml2::XMLElement* elem, const mjCMaterial* pmaterial, mjCDef* def);
  void OneJoint(tinyxml2::XMLElement* elem, const mjCJoint* pjoint, mjCDef* def,
                std::string_view classname = "");
  void OneGeom(tinyxml2::XMLElement* elem, const mjCGeom* pgeom, mjCDef* def,
               std::string_view classname = "");
  void OneSite(tinyxml2::XMLElement* elem, const mjCSite* psite, mjCDef* def,
               std::string_view classname = "");
  void OneCamera(tinyxml2::XMLElement* elem, const mjCCamera* pcamera,
                 mjCDef* def, std::string_view classname = "");
  void OneLight(tinyxml2::XMLElement* elem, const mjCLight* plight, mjCDef* def,
                std::string_view classname = "");
  void OnePair(tinyxml2::XMLElement* elem, const mjCPair* ppair,         mjCDef* def);
  void OneEquality(tinyxml2::XMLElement* elem, const mjCEquality* pequality, mjCDef* def);
  void OneTendon(tinyxml2::XMLElement* elem, const mjCTendon* ptendon,     mjCDef* def);
  void OneActuator(tinyxml2::XMLElement* elem, const mjCActuator* pactuator, mjCDef* def);
  void OnePlugin(tinyxml2::XMLElement* elem, const mjsPlugin* plugin);
  tinyxml2::XMLElement* OneFrame(tinyxml2::XMLElement* elem, mjCFrame* frame);

  bool writingdefaults;                       // true during defaults write
};

#endif  // MUJOCO_SRC_XML_XML_NATIVE_WRITER_H_
