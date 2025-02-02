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

#ifndef THIRD_PARTY_MUJOCO_SRC_XML_XML_URDF_
#define THIRD_PARTY_MUJOCO_SRC_XML_XML_URDF_

#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include "xml/xml_base.h"
#include "tinyxml2.h"

// rgb data
typedef struct _mjRGBA {
  float val[4];
} mjRGBA;

// XML parser for URDF files
class mjXURDF : public mjXBase {
 public:
  mjXURDF();                                          // constructor
  virtual ~mjXURDF();                                 // destructor

  // parse and set frame of base link and append a prefix to the name
  void Parse(
      tinyxml2::XMLElement* root,
      const std::string& prefix,
      double* pos,
      double* quat,
      bool static_body);
  void Parse(tinyxml2::XMLElement* root, const mjVFS* vfs = nullptr);  // main parser

 private:
  std::string GetPrefixedName(const std::string& name);            // get prefix/name of element
  int FindName(std::string name, std::vector<std::string>& list);  // find name in list
  void AddName(std::string name, std::vector<std::string>& list);  // add name to list
  void AddBody(std::string name);                                  // add body to local table
  void AddToTree(int n);                                           // add body to mjCModel tree
  void Body(tinyxml2::XMLElement* body_elem);                      // parse body
  void Joint(tinyxml2::XMLElement* joint_elem);                    // parse joint
  mjsGeom* Geom(tinyxml2::XMLElement* geom_elem,
                mjsBody* pbody, bool collision);      // parse origin and geometry of geom
  void Origin(tinyxml2::XMLElement* origin_elem, double* pos, double* quat); // parse origin element

  void MakeMaterials(tinyxml2::XMLElement* elem);     // find all materials recursively
  void Clear(void);                                   // clear local objects

  // URDF parser variables
  std::vector<std::string> urName;              // body name
  std::vector<int> urParent;                    // body parent (index in name vector)
  std::vector<std::vector<int> > urChildren;    // body children (index in name vector)
  std::vector<std::string> urMat;               // material name
  std::vector<mjRGBA> urRGBA;                   // material RBG value
  std::unordered_set<std::string> urGeomNames;  // geom name
  std::map<std::string, std::vector<mjsMesh*>> meshes;  // map from name to mjsMesh

  std::string urPrefix;                         // prefix to apply to all names
};

#endif  // THIRD_PARTY_MUJOCO_SRC_XML_XML_URDF_
