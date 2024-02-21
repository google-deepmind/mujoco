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
#include <string>

#include "tinyxml2.h"

#include <mujoco/mujoco.h>
#include "user/user_api.h"
#include "xml/xml_base.h"
#include "xml/xml_util.h"

class mjXReader : public mjXBase {
 public:
  mjXReader();                                                         // constructor
  virtual ~mjXReader() = default;                                      // destructor

  void Parse(tinyxml2::XMLElement* root);                              // parse XML document
  void PrintSchema(std::stringstream& str, bool html, bool pad);       // print text or HTML schema

  void SetModelFileDir(std::string modelfiledir);
  const std::string& ModelFileDir() const { return modelfiledir_; }

  // setters for directory defaults
  void SetAssetDir(std::string assetdir);
  void SetMeshDir(std::string meshdir);
  void SetTextureDir(std::string texturedir);

  // XML sections embedded in all formats
  static void Compiler(tinyxml2::XMLElement* section, mjmModel* mod);  // compiler section
  static void Option(tinyxml2::XMLElement* section, mjOption* opt);    // option section
  static void Size(tinyxml2::XMLElement* section, mjmModel* mod);      // size section

 private:
  // XML section specific to MJCF
  void Default(tinyxml2::XMLElement* section,   int parentid);         // default section
  void Extension(tinyxml2::XMLElement* section);                       // extension section
  void Custom(tinyxml2::XMLElement* section);                          // custom section
  void Visual(tinyxml2::XMLElement* section);                          // visual section
  void Statistic(tinyxml2::XMLElement* section);                       // statistic section
  void Asset(tinyxml2::XMLElement* section);                           // asset section
  void Body(tinyxml2::XMLElement* section, mjmBody* pbody,
            mjmFrame* pframe);                                         // body/world section
  void Contact(tinyxml2::XMLElement* section);                         // contact section
  void Deformable(tinyxml2::XMLElement* section);                      // deformable section
  void Equality(tinyxml2::XMLElement* section);                        // equality section
  void Tendon(tinyxml2::XMLElement* section);                          // tendon section
  void Actuator(tinyxml2::XMLElement* section);                        // actuator section
  void Sensor(tinyxml2::XMLElement* section);                          // sensor section
  void Keyframe(tinyxml2::XMLElement* section);                        // keyframe section

  // single element parsers, used in defaults and main body
  void OneFlex(tinyxml2::XMLElement* elem, mjmFlex* pflex);
  void OneMesh(tinyxml2::XMLElement* elem, mjmMesh* pmesh);
  void OneSkin(tinyxml2::XMLElement* elem, mjmSkin* pskin);
  void OneMaterial(tinyxml2::XMLElement* elem, mjmMaterial* pmaterial);
  void OneJoint(tinyxml2::XMLElement* elem, mjmJoint* pjoint);
  void OneGeom(tinyxml2::XMLElement* elem, mjmGeom* pgeom);
  void OneSite(tinyxml2::XMLElement* elem, mjmSite* site);
  void OneCamera(tinyxml2::XMLElement* elem, mjmCamera* pcamera);
  void OneLight(tinyxml2::XMLElement* elem, mjmLight* plight);
  void OnePair(tinyxml2::XMLElement* elem, mjmPair* ppair);
  void OneEquality(tinyxml2::XMLElement* elem, mjmEquality* pequality);
  void OneTendon(tinyxml2::XMLElement* elem, mjmTendon* ptendon);
  void OneActuator(tinyxml2::XMLElement* elem, mjmActuator* pactuator);
  void OneComposite(tinyxml2::XMLElement* elem, mjmBody* pbody, mjmDefault* def);
  void OneFlexcomp(tinyxml2::XMLElement* elem, mjmBody* pbody);
  void OnePlugin(tinyxml2::XMLElement* elem, mjmPlugin* plugin);

  mjXSchema schema;                                                   // schema used for validation
  mjmDefault* GetClass(tinyxml2::XMLElement* section);                    // get default class name

  bool readingdefaults;  // true while reading defaults

  // accessors for directory defaults
  std::string AssetDir() const;
  std::string MeshDir() const;
  std::string TextureDir() const;

  std::string modelfiledir_;
  std::string assetdir_;
  std::string meshdir_;
  std::string texturedir_;
};

// MJCF schema
#define nMJCF 227
extern const char* MJCF[nMJCF][mjXATTRNUM];

#endif  // MUJOCO_SRC_XML_XML_NATIVE_READER_H_
