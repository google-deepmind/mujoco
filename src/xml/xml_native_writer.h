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
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include "user/user_objects.h"
#include "xml/xml_base.h"
#include "tinyxml2.h"

class mjXWriter : public mjXBase {
 public:
  mjXWriter();                     // constructor
  virtual ~mjXWriter() = default;  // destructor
  void SetModel(mjSpec* _spec, const mjModel* m = nullptr);

  // write XML document to string
  std::string Write(char* error, std::size_t error_sz);

 private:
  // insert end child with given name, return child
  tinyxml2::XMLElement* InsertEnd(tinyxml2::XMLElement* parent, const char* name);

  // compiled model
  mjCModel* model = 0;

  // XML section writers
  void Compiler(tinyxml2::XMLElement* root);              // compiler section
  void Option(tinyxml2::XMLElement* root);                // option section
  void Size(tinyxml2::XMLElement* root);                  // size section
  void Visual(tinyxml2::XMLElement* root);                // visual section
  void Statistic(tinyxml2::XMLElement* root);             // statistic section
  void Default(tinyxml2::XMLElement* root, mjCDef* def);  // default section
  void Extension(tinyxml2::XMLElement* root);             // extension section
  void Custom(tinyxml2::XMLElement* root);                // custom section
  void Asset(tinyxml2::XMLElement* root);                 // asset section
  void Contact(tinyxml2::XMLElement* root);               // contact section
  void Deformable(tinyxml2::XMLElement* root);            // deformable section
  void Equality(tinyxml2::XMLElement* root);              // equality section
  void Tendon(tinyxml2::XMLElement* root);                // tendon section
  void Actuator(tinyxml2::XMLElement* root);              // actuator section
  void Sensor(tinyxml2::XMLElement* root);                // sensor section
  void Keyframe(tinyxml2::XMLElement* root);              // keyframe section

  // body/world section
  void Body(tinyxml2::XMLElement* elem,
            mjCBody*              body,
            mjCFrame*             frame,
            std::string_view      childclass = "");

  // table-driven attribute writing: the mechanical attributes of an element,
  // driven by the same generated mjXAttr rows the reader uses. obj and def
  // are the element's bound spec struct and its class-default counterpart;
  // each bound field is compared against the default at the same offset, so
  // attributes equal to their default are skipped. Strings, files and
  // custom-read attributes remain in the OneX() remnants.
  // With `given`, obj is a struct of the spec of the model (or a copy of the
  // one at `field`), and an attribute which the spec records as written is
  // not skipped when it has its default value.
  template <typename T>
  void WriteAttrTable(tinyxml2::XMLElement* elem,
                      const T*              obj,
                      const T*              def,
                      const struct mjXAttr* rows,
                      int                   nrow,
                      bool                  given = false,
                      const T*              field = nullptr);

  // single element writers, used in defaults and main body
  void OneFlex(tinyxml2::XMLElement* elem, const mjCFlex* pflex);
  void OneMesh(tinyxml2::XMLElement* elem, const mjCMesh* pmesh, mjCDef* def);
  void OneSkin(tinyxml2::XMLElement* elem, const mjCSkin* pskin);
  void OneMaterial(tinyxml2::XMLElement* elem, const mjCMaterial* pmaterial, mjCDef* def);
  void OneJoint(tinyxml2::XMLElement* elem,
                const mjCJoint*       pjoint,
                mjCDef*               def,
                std::string_view      classname = "");
  void OneGeom(tinyxml2::XMLElement* elem,
               const mjCGeom*        pgeom,
               mjCDef*               def,
               std::string_view      classname = "");
  void OneSite(tinyxml2::XMLElement* elem,
               const mjCSite*        psite,
               mjCDef*               def,
               std::string_view      classname = "");
  void OneCamera(tinyxml2::XMLElement* elem,
                 const mjCCamera*      pcamera,
                 mjCDef*               def,
                 std::string_view      classname = "");
  void OneLight(tinyxml2::XMLElement* elem,
                const mjCLight*       plight,
                mjCDef*               def,
                std::string_view      classname = "");
  void OnePair(tinyxml2::XMLElement* elem, const mjCPair* ppair, mjCDef* def);
  void OneEquality(tinyxml2::XMLElement* elem, const mjCEquality* pequality, mjCDef* def);
  void OneTendon(tinyxml2::XMLElement* elem, const mjCTendon* ptendon, mjCDef* def);
  tinyxml2::XMLElement* OneActuator(tinyxml2::XMLElement* section,
                                    const mjCActuator*    pactuator,
                                    mjCDef*               def);
  void                  OnePlugin(tinyxml2::XMLElement* elem, const mjsPlugin* plugin);
  void                  PluginConfig(tinyxml2::XMLElement* elem,
                                     const mjCPlugin*      instance,
                                     const std::string&    plugin);
  void                  FreeJoint(tinyxml2::XMLElement* elem, const mjCJoint* joint);
  tinyxml2::XMLElement* OneFrame(tinyxml2::XMLElement* elem,
                                 mjCFrame*             frame,
                                 std::string_view      childclass);

  // strip a frame from a compiled pose: pos/quat become relative to the frame
  static void FrameLocal(const mjCFrame* frame, double pos[3], double quat[4]);

  // the values of an element which are saved: those which the spec gives, or those which
  // compilation made of them
  template <typename S, typename C>
  const S* Values(const C* element) const {
    return authored_ ? &element->spec : static_cast<const S*>(element);
  }

  // an angle of an element, of a joint or of an orientation, in the unit of the saved file
  double Angle(const mjCBase* element, double angle, bool orientation = false) const;

  // write the position and orientation which the spec gives an element, where they differ from
  // those of its class: the orientation as it was written, or as a quaternion if the notation is
  // canonical
  void WriteSpecPose(tinyxml2::XMLElement* elem,
                     const mjCBase*        element,
                     const double          pos[3],
                     const double          quat[4],
                     const mjsOrientation& alt,
                     const double*         defpos  = nullptr,
                     const double*         defquat = nullptr,
                     const mjsOrientation* defalt  = nullptr);

  // the quaternion which the spec gives an element (null: an element of a default class)
  void SpecQuat(double                result[4],
                const mjCBase*        element,
                const double          quat[4],
                const mjsOrientation& alt);

  // write the size and the pose which the spec gives a geom or a site, where they differ from
  // those of its class
  template <typename S>
  void WriteSpecShape(tinyxml2::XMLElement* elem,
                      const mjCBase*        element,
                      const S&              given,
                      const S&              defgiven);

  // write the user data which the spec gives an element, unless it is that of its default class
  void WriteSpecUser(tinyxml2::XMLElement*      elem,
                     const std::vector<double>& user,
                     const std::vector<double>& defuser);

  // write the inertial which the spec gives a body, or the one calculated for it
  void WriteSpecInertial(tinyxml2::XMLElement* elem, const mjCBody* body);

  // the compiler setting of an attached spec which the saved file cannot give the elements of that
  // spec, as an error which names an element; empty if there is none
  std::string AttachedSettings() const;

  // true if the saved file gives a body the inertia which compilation gave it, without an
  // inertial element
  bool InertialReproduced(const mjCBody* body) const;

  // the shortcut whose tag gives an actuator back when reloaded over its class default, as an
  // entry of the generated actuator dispatch, else the general entry; s and d get the shortcut
  // parameters of the actuator and of the class slot the shortcut pre-reads
  const struct mjXActuatorEntry* ShortcutEntry(const mjsActuator* actuator,
                                               const mjCDef*      def,
                                               mjXShortcut*       s,
                                               mjXShortcut*       d) const;

  bool writingdefaults;     // true during defaults write
  bool authored_  = false;  // save what the spec gives, not what compilation made of it
  bool canonical_ = true;   // save quaternions, radians and sizes, not the notation of the spec
  bool degree_    = false;  // angles in the saved file are in degrees
};


#endif  // MUJOCO_SRC_XML_XML_NATIVE_WRITER_H_
