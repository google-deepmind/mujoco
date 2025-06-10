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

#include "xml/xml_native_writer.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "engine/engine_io.h"
#include "engine/engine_plugin.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_util.h"
#include "xml/xml_base.h"
#include "xml/xml_util.h"
#include "tinyxml2.h"

namespace {

using std::string;
using std::string_view;
using tinyxml2::XMLComment;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using mujoco::user::VectorToString;

}  // namespace



// custom XML indentation: 2 spaces rather than the default 4
class mj_XMLPrinter : public tinyxml2::XMLPrinter {
  using tinyxml2::XMLPrinter::XMLPrinter;

 public:
    void PrintSpace( int depth ) {
      for (int i=0; i < depth; ++i) {
        Write( "  " );
      }
    }
};


// save XML file using custom 2-space indentation
static string WriteDoc(XMLDocument& doc, char *error, size_t error_sz) {
  doc.ClearError();
  mj_XMLPrinter stream(nullptr, /*compact=*/false);
  doc.Print(&stream);
  if (doc.ErrorID()) {
    mjCopyError(error, doc.ErrorStr(), error_sz);
    return "";
  }
  string str = string(stream.CStr());

  // top level sections
  std::array<string, 17> sections = {
    "<actuator", "<asset",      "<compiler", "<contact",   "<custom",
    "<default>", "<deformable", "<equality", "<extension", "<keyframe",
    "<option",   "<sensor",     "<size",     "<statistic", "<tendon",
    "<visual",   "<worldbody"};

  // position of newline before first section
  size_t first_pos = string::npos;

  // insert newlines before section headers
  for (const string& section : sections) {
    std::size_t pos = 0;
    while ((pos = str.find(section, pos)) != string::npos) {
      // find newline before this section
      std::size_t line_pos = str.rfind('\n', pos);

      // save position of first section
      if (line_pos < first_pos) first_pos = line_pos;

      // insert another newline
      if (line_pos != string::npos) {
        str.insert(line_pos + 1, "\n");
        pos++;  // account for inserted newline
      }

      // advance
      pos += section.length();
    }
  }

  // remove added newline before the first section
  if (first_pos != string::npos) {
    str.erase(first_pos, 1);
  }

  return str;
}


// insert end child with given name, return child
XMLElement* mjXWriter::InsertEnd(XMLElement* parent, const char* name) {
  XMLElement* result = parent->GetDocument()->NewElement(name);
  parent->InsertEndChild(result);

  return result;
}


//---------------------------------- class mjXWriter: one-element writers --------------------------

// write flex
void mjXWriter::OneFlex(XMLElement* elem, const mjCFlex* flex) {
  string text;
  mjCFlex defflex;

  // common attributes
  WriteAttrTxt(elem, "name", flex->name);
  WriteAttr(elem, "radius", 1, &flex->radius, &defflex.radius);
  if (flex->get_material() != defflex.get_material()) {
    WriteAttrTxt(elem, "material", flex->get_material());
  }
  WriteAttr(elem, "rgba", 4, flex->rgba, defflex.rgba);
  WriteAttrKey(elem, "flatskin", bool_map, 2, flex->flatskin, defflex.flatskin);
  WriteAttrInt(elem, "dim", flex->dim, defflex.dim);
  WriteAttrInt(elem, "group", flex->group, defflex.group);

  // data vectors
  if (!flex->get_vertbody().empty()) {
    text = VectorToString(flex->get_vertbody());
    WriteAttrTxt(elem, "body", text);
  }
  if (!flex->get_vert().empty()) {
    text = VectorToString(flex->get_vert());
    WriteAttrTxt(elem, "vertex", text);
  }
  if (!flex->get_elem().empty()) {
    text = VectorToString(flex->get_elem());
    WriteAttrTxt(elem, "element", text);
  }
  if (!flex->get_texcoord().empty()) {
    text = VectorToString(flex->get_texcoord());
    WriteAttrTxt(elem, "texcoord", text);
  }
  if (!flex->get_elemtexcoord().empty()) {
    text = VectorToString(flex->get_elemtexcoord());
    WriteAttrTxt(elem, "elemtexcoord", text);
  }
  if (!flex->get_nodebody().empty()) {
    text = VectorToString(flex->get_nodebody());
    WriteAttrTxt(elem, "node", text);
  }

  // contact subelement
  XMLElement* cont = InsertEnd(elem, "contact");
  WriteAttrInt(cont, "contype", flex->contype, defflex.contype);
  WriteAttrInt(cont, "conaffinity", flex->conaffinity, defflex.conaffinity);
  WriteAttrInt(cont, "condim", flex->condim, defflex.condim);
  WriteAttrInt(cont, "priority", flex->priority, defflex.priority);
  WriteAttr(cont, "friction", 3, flex->friction, defflex.friction);
  WriteAttr(cont, "solmix", 1, &flex->solmix, &defflex.solmix);
  WriteAttr(cont, "solref", mjNREF, flex->solref, defflex.solref);
  WriteAttr(cont, "solimp", mjNIMP, flex->solimp, defflex.solimp);
  WriteAttr(cont, "margin", 1, &flex->margin, &defflex.margin);
  WriteAttr(cont, "gap", 1, &flex->gap, &defflex.gap);
  WriteAttrKey(cont, "internal", bool_map, 2, flex->internal, defflex.internal);
  WriteAttrKey(cont, "selfcollide", flexself_map, 5, flex->selfcollide, defflex.selfcollide);
  WriteAttrInt(cont, "activelayers", flex->activelayers, defflex.activelayers);

  // remove contact is no attributes
  if (!cont->FirstAttribute()) {
    elem->DeleteChild(cont);
  }

  // elasticity subelement
  XMLElement* elastic = InsertEnd(elem, "elasticity");
  WriteAttr(elastic, "young", 1, &flex->young, &defflex.young);
  WriteAttr(elastic, "poisson", 1, &flex->poisson, &defflex.poisson);
  WriteAttr(elastic, "thickness", 1, &flex->thickness, &defflex.thickness);
  WriteAttr(elastic, "damping", 1, &flex->damping, &defflex.damping);
  WriteAttrKey(elastic, "elastic2d", elastic2d_map, 2, flex->elastic2d, defflex.elastic2d);

  // edge subelement
  XMLElement* edge = InsertEnd(elem, "edge");
  WriteAttr(edge, "stiffness", 1, &flex->edgestiffness, &defflex.edgestiffness);
  WriteAttr(edge, "damping", 1, &flex->edgedamping, &defflex.edgedamping);

  // remove edge if no attributes
  if (!edge->FirstAttribute()) {
    elem->DeleteChild(edge);
  }
}



// write mesh
void mjXWriter::OneMesh(XMLElement* elem, const mjCMesh* mesh, mjCDef* def) {
  string text;

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", mesh->name);
    if (mesh->classname != "main") {
      WriteAttrTxt(elem, "class", mesh->classname);
    }
    WriteAttrTxt(elem, "content_type", mesh->ContentType());
    WriteAttrTxt(elem, "file", mesh->File());

    // write vertex data
    if (!mesh->UserVert().empty()) {
      text = VectorToString(mesh->UserVert());
      WriteAttrTxt(elem, "vertex", text);
    }

    // write normal data
    if (!mesh->UserNormal().empty()) {
      text = VectorToString(mesh->UserNormal());
      WriteAttrTxt(elem, "normal", text);
    }

    // write texcoord data
    if (!mesh->UserTexcoord().empty()) {
      text = VectorToString(mesh->UserTexcoord());
      WriteAttrTxt(elem, "texcoord", text);
    }

    // write face data
    if (!mesh->UserFace().empty()) {
      text = VectorToString(mesh->UserFace());
      WriteAttrTxt(elem, "face", text);
    }
  }

  // defaults and regular
  if (mesh->Inertia() != def->Mesh().Inertia()) {
    WriteAttrTxt(elem, "inertia", FindValue(meshinertia_map, 4, mesh->Inertia()));
  }
  WriteAttr(elem, "refpos", 3, mesh->Refpos(), def->Mesh().Refpos());
  WriteAttr(elem, "refquat", 4, mesh->Refquat(), def->Mesh().Refquat());
  WriteAttr(elem, "scale", 3, mesh->Scale(), def->Mesh().Scale());
  WriteAttrKey(elem, "smoothnormal", bool_map, 2, mesh->SmoothNormal(),
               def->Mesh().SmoothNormal());
}



// write skin
void mjXWriter::OneSkin(XMLElement* elem, const mjCSkin* skin) {
  string text;
  mjCDef mydef;
  float zero = 0;

  // write attributes
  WriteAttrTxt(elem, "name", skin->name);
  WriteAttrTxt(elem, "file", skin->File());
  WriteAttrTxt(elem, "material", skin->get_material());
  WriteAttrInt(elem, "group", skin->group, 0);
  WriteAttr(elem, "rgba", 4, skin->rgba, mydef.Geom().rgba);
  WriteAttr(elem, "inflate", 1, &skin->inflate, &zero);

  // write data if no file
  if (skin->File().empty()) {
    // mesh vert
    text = VectorToString(skin->get_vert());
    WriteAttrTxt(elem, "vertex", text);

    // mesh texcoord
    if (!skin->get_texcoord().empty()) {
      text = VectorToString(skin->get_texcoord());
      WriteAttrTxt(elem, "texcoord", text);
    }

    // mesh face
    text = VectorToString(skin->get_face());
    WriteAttrTxt(elem, "face", text);

    // bones
    for (size_t i=0; i < skin->get_bodyname().size(); i++) {
      // make bone
      XMLElement* bone = InsertEnd(elem, "bone");

      // write attributes
      WriteAttrTxt(bone, "body", skin->get_bodyname()[i]);
      WriteAttr(bone, "bindpos", 3, skin->get_bindpos().data()+3*i);
      WriteAttr(bone, "bindquat", 4, skin->get_bindquat().data()+4*i);

      // write vertid
      text = VectorToString(skin->get_vertid()[i]);
      WriteAttrTxt(bone, "vertid", text);

      // write vertweight
      text = VectorToString(skin->get_vertweight()[i]);
      WriteAttrTxt(bone, "vertweight", text);
    }
  }
}



// write material
void mjXWriter::OneMaterial(XMLElement* elem, const mjCMaterial* material, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", material->name);
    if (material->classname != "main") {
      WriteAttrTxt(elem, "class", material->classname);
    }
  }

  // defaults and regular
  // check if we have non-rgb textures
  bool has_non_rgb = false;
  for (int i=1; i < mjNTEXROLE; i++) {
    if (!material->textures_[i].empty()) {
      if (i != mjTEXROLE_RGB) {
        has_non_rgb = true;
      }
    }
  }

  // if we have non-rgb textures, write them as layers
  if (has_non_rgb) {
    for (int i=1; i < mjNTEXROLE; i++) {
      if (!material->textures_[i].empty()) {
        XMLElement * child_elem = InsertEnd(elem, "layer");
        WriteAttrTxt(child_elem, "texture", material->textures_[i]);
        WriteAttrTxt(child_elem, "role", FindValue(texrole_map, 9, i));
      }
    }
  } else {
    if (material->textures_[mjTEXROLE_RGB] != def->Material().textures_[mjTEXROLE_RGB]) {
      WriteAttrTxt(elem, "texture", material->get_texture(mjTEXROLE_RGB));
    }
  }

  WriteAttrKey(elem, "texuniform", bool_map, 2, material->texuniform, def->Material().texuniform);
  WriteAttr(elem, "texrepeat", 2, material->texrepeat, def->Material().texrepeat);
  WriteAttr(elem, "emission", 1, &material->emission, &def->Material().emission);
  WriteAttr(elem, "specular", 1, &material->specular, &def->Material().specular);
  WriteAttr(elem, "shininess", 1, &material->shininess, &def->Material().shininess);
  WriteAttr(elem, "reflectance", 1, &material->reflectance, &def->Material().reflectance);
  WriteAttr(elem, "metallic", 1, &material->metallic, &def->Material().metallic);
  WriteAttr(elem, "roughness", 1, &material->roughness, &def->Material().roughness);
  WriteAttr(elem, "rgba", 4, material->rgba, def->Material().rgba);
}



// write joint
void mjXWriter::OneJoint(XMLElement* elem, const mjCJoint* joint, mjCDef* def,
                         string_view classname) {
  double zero = 0;

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", joint->name);
    if (classname != joint->classname && joint->classname != "main") {
      WriteAttrTxt(elem, "class", joint->classname);
    }
    if (joint->type != mjJNT_FREE) {
      WriteAttr(elem, "pos", 3, joint->pos);
    }
    if (joint->type != mjJNT_FREE && joint->type != mjJNT_BALL) {
      WriteAttr(elem, "axis", 3, joint->axis);
    }
  }

  // defaults and regular
  if (joint->type != def->Joint().type) {
    WriteAttrTxt(elem, "type", FindValue(joint_map, joint_sz, joint->type));
  }
  WriteAttrInt(elem, "group", joint->group, def->Joint().group);
  WriteAttr(elem, "ref", 1, &joint->ref, &zero);
  WriteAttr(elem, "springref", 1, &joint->springref, &zero);
  WriteAttr(elem, "solreflimit", mjNREF, joint->solref_limit, def->Joint().solref_limit, true);
  WriteAttr(elem, "solimplimit", mjNIMP, joint->solimp_limit, def->Joint().solimp_limit, true);
  WriteAttr(elem, "solreffriction", mjNREF, joint->solref_friction, def->Joint().solref_friction,
            true);
  WriteAttr(elem, "solimpfriction", mjNIMP, joint->solimp_friction, def->Joint().solimp_friction,
            true);
  WriteAttr(elem, "stiffness", 1, &joint->stiffness, &def->Joint().stiffness);
  WriteAttrKey(elem, "limited", TFAuto_map, 3, joint->limited, def->Joint().limited);
  WriteAttr(elem, "range", 2, joint->range, def->Joint().range);
  WriteAttrKey(elem, "actuatorfrclimited", TFAuto_map, 3, joint->actfrclimited,
               def->Joint().actfrclimited);
  WriteAttrKey(elem, "actuatorgravcomp", bool_map, 2, joint->actgravcomp, def->Joint().actgravcomp);
  WriteAttr(elem, "actuatorfrcrange", 2, joint->actfrcrange, def->Joint().actfrcrange);
  WriteAttr(elem, "margin", 1, &joint->margin, &def->Joint().margin);
  WriteAttr(elem, "armature", 1, &joint->armature, &def->Joint().armature);
  WriteAttr(elem, "damping", 1, &joint->damping, &def->Joint().damping);
  WriteAttr(elem, "frictionloss", 1, &joint->frictionloss, &def->Joint().frictionloss);

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", joint->get_userdata());
  } else {
    WriteVector(elem, "user", joint->get_userdata(), def->Joint().get_userdata());
  }
}

// write geom
void mjXWriter::OneGeom(XMLElement* elem, const mjCGeom* geom, mjCDef* def, string_view classname) {
  double unitq[4] = {1, 0, 0, 0};
  double mass = 0;

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", geom->name);
    if (classname != geom->classname && geom->classname != "main") {
      WriteAttrTxt(elem, "class", geom->classname);
    }
    if (mjGEOMINFO[geom->type]) {
      WriteAttr(elem, "size", mjGEOMINFO[geom->type], geom->size, def->Geom().size);
    }
    if (mjuu_defined(geom->mass)) {
      mass = geom->GetVolume() * def->Geom().density;
    }

    // mesh geom
    if (geom->type == mjGEOM_MESH || geom->type == mjGEOM_SDF) {
      mjCMesh* mesh = geom->mesh;

      // write pos/quat if there is a difference
      if (!SameVector(geom->pos, mesh->GetPosPtr(), 3) ||
          !SameVector(geom->quat, mesh->GetQuatPtr(), 4)) {
        // recover geom pos/quat before mesh frame transformation
        double p[3], q[4];
        mjuu_copyvec(p, geom->pos, 3);
        mjuu_copyvec(q, geom->quat, 4);
        mjuu_frameaccuminv(p, q, mesh->GetPosPtr(),
                           mesh->GetQuatPtr());

        // write
        WriteAttr(elem, "pos", 3, p, unitq+1);
        WriteAttr(elem, "quat", 4, q, unitq);
      }
    }

    // non-mesh geom
    else {
      WriteAttr(elem, "pos", 3, geom->pos, unitq+1);
      WriteAttr(elem, "quat", 4, geom->quat, unitq);
    }
  } else {
    WriteAttr(elem, "size", 3, geom->size, def->Geom().size);
  }

  // defaults and regular
  WriteAttrKey(elem, "type", geom_map, mjNGEOMTYPES, geom->type, def->Geom().type);
  WriteAttrInt(elem, "contype", geom->contype, def->Geom().contype);
  WriteAttrInt(elem, "conaffinity", geom->conaffinity, def->Geom().conaffinity);
  WriteAttrInt(elem, "condim", geom->condim, def->Geom().condim);
  WriteAttrInt(elem, "group", geom->group, def->Geom().group);
  WriteAttrInt(elem, "priority", geom->priority, def->Geom().priority);
  WriteAttr(elem, "friction", 3, geom->friction, def->Geom().friction, true);
  WriteAttr(elem, "solmix", 1, &geom->solmix, &def->Geom().solmix);
  WriteAttr(elem, "solref", mjNREF, geom->solref, def->Geom().solref, true);
  WriteAttr(elem, "solimp", mjNIMP, geom->solimp, def->Geom().solimp, true);
  WriteAttr(elem, "margin", 1, &geom->margin, &def->Geom().margin);
  WriteAttr(elem, "gap", 1, &geom->gap, &def->Geom().gap);
  WriteAttr(elem, "gap", 1, &geom->gap, &def->Geom().gap);
  WriteAttrKey(elem, "fluidshape",
               fluid_map, 2, geom->fluid_ellipsoid, def->Geom().fluid_ellipsoid);
  WriteAttr(elem, "fluidcoef", 5, geom->fluid_coefs, def->Geom().fluid_coefs);
  if (geom->type != mjGEOM_MESH) {
    WriteAttrKey(elem, "shellinertia", meshtype_map, 2, geom->typeinertia,
                 def->Geom().typeinertia);
  }
  if (mjuu_defined(geom->mass)) {
    WriteAttr(elem, "mass", 1, &geom->mass_, &mass);
  } else {
    WriteAttr(elem, "density", 1, &geom->density, &def->Geom().density);
  }
  if (geom->get_material() != def->Geom().get_material()) {
    WriteAttrTxt(elem, "material", geom->get_material());
  }
  WriteAttr(elem, "rgba", 4, geom->rgba, def->Geom().rgba);

  // hfield and mesh attributes
  if (geom->type == mjGEOM_HFIELD) {
    WriteAttrTxt(elem, "hfield", geom->get_hfieldname());
  }
  if (geom->type == mjGEOM_MESH || geom->type == mjGEOM_SDF) {
    WriteAttrTxt(elem, "mesh", geom->get_meshname());
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", geom->get_userdata());
  } else {
    WriteVector(elem, "user", geom->get_userdata(), def->Geom().get_userdata());
  }

  // write plugin
  if (geom->plugin.active) {
    OnePlugin(InsertEnd(elem, "plugin"), &geom->plugin);
  }
}

// write site
void mjXWriter::OneSite(XMLElement* elem, const mjCSite* site, mjCDef* def, string_view classname) {
  double unitq[4] = {1, 0, 0, 0};

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", site->name);
    if (classname != site->classname && site->classname != "main") {
      WriteAttrTxt(elem, "class", site->classname);
    }
    WriteAttr(elem, "pos", 3, site->pos);
    WriteAttr(elem, "quat", 4, site->quat, unitq);
    if (mjGEOMINFO[site->type]) {
      WriteAttr(elem, "size", mjGEOMINFO[site->type], site->size, def->Site().size);
    }
  } else {
    WriteAttr(elem, "size", 3, site->size, def->Site().size);
  }

  // defaults and regular
  WriteAttrInt(elem, "group", site->group, def->Site().group);
  WriteAttrKey(elem, "type", geom_map, mjNGEOMTYPES, site->type, def->Site().type);
  if (site->get_material() != def->Site().get_material()) {
    WriteAttrTxt(elem, "material", site->get_material());
  }
  WriteAttr(elem, "rgba", 4, site->rgba, def->Site().rgba);

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", site->get_userdata());
  } else {
    WriteVector(elem, "user", site->get_userdata(), def->Site().get_userdata());
  }
}

// write camera
void mjXWriter::OneCamera(XMLElement* elem, const mjCCamera* camera, mjCDef* def,
                          string_view classname) {
  double unitq[4] = {1, 0, 0, 0};

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", camera->name);
    if (classname != camera->classname && camera->classname != "main") {
      WriteAttrTxt(elem, "class", camera->classname);
    }
    WriteAttrTxt(elem, "target", camera->get_targetbody());
    WriteAttr(elem, "pos", 3, camera->pos);
    WriteAttr(elem, "quat", 4, camera->quat, unitq);
  }

  // defaults and regular
  WriteAttr(elem, "ipd", 1, &camera->ipd, &def->Camera().ipd);
  WriteAttrKey(elem, "mode", camlight_map, camlight_sz, camera->mode, def->Camera().mode);
  WriteAttr(elem, "resolution", 2, camera->resolution, def->Camera().resolution);
  WriteAttrKey(elem, "orthographic", bool_map, 2, camera->orthographic, def->Camera().orthographic);

  // camera intrinsics if specified
  if (camera->sensor_size[0] > 0 && camera->sensor_size[1] > 0) {
    WriteAttr(elem, "sensorsize", 2, camera->sensor_size);
    WriteAttr(elem, "focal", 2, camera->focal_length, def->Camera().focal_length);
    WriteAttr(elem, "focalpixel", 2, camera->focal_pixel, def->Camera().focal_pixel);
    WriteAttr(elem, "principal", 2, camera->principal_length, def->Camera().principal_length);
    WriteAttr(elem, "principalpixel", 2, camera->principal_pixel, def->Camera().principal_pixel);
  } else {
    WriteAttr(elem, "fovy", 1, &camera->fovy, &def->Camera().fovy);
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", camera->get_userdata());
  } else {
    WriteVector(elem, "user", camera->get_userdata(), def->Camera().get_userdata());
  }
}

// write light
void mjXWriter::OneLight(XMLElement* elem, const mjCLight* light, mjCDef* def,
                         string_view classname) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", light->name);
    if (classname != light->classname && light->classname != "main") {
      WriteAttrTxt(elem, "class", light->classname);
    }
    WriteAttrTxt(elem, "target", light->get_targetbody());
    WriteAttr(elem, "pos", 3, light->pos);
    WriteAttr(elem, "dir", 3, light->dir);
  }

  // defaults and regular
  WriteAttr(elem, "bulbradius", 1, &light->bulbradius, &def->Light().bulbradius);
  WriteAttr(elem, "intensity", 1, &light->intensity, &def->Light().intensity);
  WriteAttr(elem, "range", 1, &light->range, &def->Light().range);
  WriteAttrKey(elem, "type", lighttype_map, lighttype_sz, light->type, def->Light().type);
  WriteAttrTxt(elem, "texture", light->get_texture());
  WriteAttrKey(elem, "castshadow", bool_map, 2, light->castshadow, def->Light().castshadow);
  WriteAttrKey(elem, "active", bool_map, 2, light->active, def->Light().active);
  WriteAttr(elem, "attenuation", 3, light->attenuation, def->Light().attenuation);
  WriteAttr(elem, "cutoff", 1, &light->cutoff, &def->Light().cutoff);
  WriteAttr(elem, "exponent", 1, &light->exponent, &def->Light().exponent);
  WriteAttr(elem, "ambient", 3, light->ambient, def->Light().ambient);
  WriteAttr(elem, "diffuse", 3, light->diffuse, def->Light().diffuse);
  WriteAttr(elem, "specular", 3, light->specular, def->Light().specular);
  WriteAttrKey(elem, "mode", camlight_map, camlight_sz, light->mode, def->Light().mode);
}

// write pair
void mjXWriter::OnePair(XMLElement* elem, const mjCPair* pair, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    if (pair->classname != "main") {
      WriteAttrTxt(elem, "class", pair->classname);
    }
    WriteAttrTxt(elem, "geom1", pair->get_geomname1());
    WriteAttrTxt(elem, "geom2", pair->get_geomname2());
  }

  // defaults and regular
  WriteAttrTxt(elem, "name", pair->name);
  WriteAttrInt(elem, "condim", pair->condim, def->Pair().spec.condim);
  WriteAttr(elem, "margin", 1, &pair->margin, &def->Pair().spec.margin);
  WriteAttr(elem, "gap", 1, &pair->gap, &def->Pair().spec.gap);
  WriteAttr(elem, "solref", mjNREF, pair->solref, def->Pair().spec.solref, true);
  WriteAttr(elem, "solreffriction", mjNREF, pair->solreffriction, def->Pair().spec.solreffriction,
            true);
  WriteAttr(elem, "solimp", mjNIMP, pair->solimp, def->Pair().spec.solimp, true);
  WriteAttr(elem, "friction", 5, pair->friction, def->Pair().spec.friction);  // all 5 values
}



// write equality
void mjXWriter::OneEquality(XMLElement* elem, const mjCEquality* equality, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", equality->name);
    if (equality->classname != "main") {
      WriteAttrTxt(elem, "class", equality->classname);
    }

    switch (equality->type) {
      case mjEQ_CONNECT:
        if (equality->objtype == mjOBJ_BODY) {
          WriteAttrTxt(elem, "body1", mjs_getString(equality->name1));
          WriteAttrTxt(elem, "body2", mjs_getString(equality->name2));
          WriteAttr(elem, "anchor", 3, equality->data);
        } else {
          WriteAttrTxt(elem, "site1", mjs_getString(equality->name1));
          WriteAttrTxt(elem, "site2", mjs_getString(equality->name2));
        }
        break;

      case mjEQ_WELD:
        if (equality->objtype == mjOBJ_BODY) {
          WriteAttrTxt(elem, "body1", mjs_getString(equality->name1));
          WriteAttrTxt(elem, "body2", mjs_getString(equality->name2));
          WriteAttr(elem, "anchor", 3, equality->data);
          WriteAttr(elem, "relpose", 7, equality->data+3);
        } else {
          WriteAttrTxt(elem, "site1", mjs_getString(equality->name1));
          WriteAttrTxt(elem, "site2", mjs_getString(equality->name2));
        }
        WriteAttr(elem, "torquescale", 1, equality->data+10);
        break;

      case mjEQ_JOINT:
        WriteAttrTxt(elem, "joint1", mjs_getString(equality->name1));
        WriteAttrTxt(elem, "joint2", mjs_getString(equality->name2));
        WriteAttr(elem, "polycoef", 5, equality->data);
        break;

      case mjEQ_TENDON:
        WriteAttrTxt(elem, "tendon1", mjs_getString(equality->name1));
        WriteAttrTxt(elem, "tendon2", mjs_getString(equality->name2));
        WriteAttr(elem, "polycoef", 5, equality->data);
        break;

      case mjEQ_FLEX:
        WriteAttrTxt(elem, "flex", mjs_getString(equality->name1));
        break;

      default:
        mju_error("mjXWriter: unknown equality type.");
    }
  }

  // defaults and regular
  WriteAttrKey(elem, "active", bool_map, 2, equality->active, def->Equality().active);
  WriteAttr(elem, "solref", mjNREF, equality->solref, def->Equality().solref, true);
  WriteAttr(elem, "solimp", mjNIMP, equality->solimp, def->Equality().solimp, true);
}



// write tendon
void mjXWriter::OneTendon(XMLElement* elem, const mjCTendon* tendon, mjCDef* def) {
  bool fixed = (tendon->GetWrap(0) && tendon->GetWrap(0)->type == mjWRAP_JOINT);

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", tendon->name);
    if (tendon->classname != "main") {
      WriteAttrTxt(elem, "class", tendon->classname);
    }
  }

  // defaults and regular
  WriteAttrInt(elem, "group", tendon->group, def->Tendon().group);
  WriteAttr(elem, "solreflimit", mjNREF, tendon->solref_limit, def->Tendon().solref_limit, true);
  WriteAttr(elem, "solimplimit", mjNIMP, tendon->solimp_limit, def->Tendon().solimp_limit, true);
  WriteAttr(elem, "solreffriction", mjNREF, tendon->solref_friction, def->Tendon().solref_friction,
            true);
  WriteAttr(elem, "solimpfriction", mjNIMP, tendon->solimp_friction, def->Tendon().solimp_friction,
            true);
  WriteAttrKey(elem, "limited", TFAuto_map, 3, tendon->limited, def->Tendon().limited);
  WriteAttrKey(elem, "actuatorfrclimited", TFAuto_map, 3, tendon->actfrclimited, def->Tendon().actfrclimited);
  WriteAttr(elem, "range", 2, tendon->range, def->Tendon().range);
  WriteAttr(elem, "actuatorfrcrange", 2, tendon->actfrcrange, def->Tendon().actfrcrange);
  WriteAttr(elem, "margin", 1, &tendon->margin, &def->Tendon().margin);
  WriteAttr(elem, "stiffness", 1, &tendon->stiffness, &def->Tendon().stiffness);
  WriteAttr(elem, "damping", 1, &tendon->damping, &def->Tendon().damping);
  WriteAttr(elem, "armature", 1, &tendon->armature, &def->Tendon().armature);
  WriteAttr(elem, "frictionloss", 1, &tendon->frictionloss, &def->Tendon().frictionloss);
  if (tendon->springlength[0] != tendon->springlength[1] ||
      def->Tendon().springlength[0] != def->Tendon().springlength[1]) {
    WriteAttr(elem, "springlength", 2, tendon->springlength, def->Tendon().springlength);
  } else {
    WriteAttr(elem, "springlength", 1, tendon->springlength, def->Tendon().springlength);
  }
  // spatial only
  if (!fixed) {
    if (tendon->get_material() != def->Tendon().get_material()) {
      WriteAttrTxt(elem, "material", tendon->get_material());
    }
    WriteAttr(elem, "width", 1, &tendon->width, &def->Tendon().width);
    WriteAttr(elem, "rgba", 4, tendon->rgba, def->Tendon().rgba);
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", tendon->get_userdata());
  } else {
    WriteVector(elem, "user", tendon->get_userdata(), def->Tendon().get_userdata());
  }
}



// write actuator
void mjXWriter::OneActuator(XMLElement* elem, const mjCActuator* actuator, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", actuator->name);
    if (actuator->classname != "main") {
      WriteAttrTxt(elem, "class", actuator->classname);
    }

    // transmission target
    switch (actuator->trntype) {
      case mjTRN_JOINT:
        WriteAttrTxt(elem, "joint", actuator->get_target());
        break;

      case mjTRN_JOINTINPARENT:
        WriteAttrTxt(elem, "jointinparent", actuator->get_target());
        break;

      case mjTRN_TENDON:
        WriteAttrTxt(elem, "tendon", actuator->get_target());
        break;

      case mjTRN_SLIDERCRANK:
        WriteAttrTxt(elem, "cranksite", actuator->get_target());
        WriteAttrTxt(elem, "slidersite", actuator->get_slidersite());
        break;

      case mjTRN_SITE:
        WriteAttrTxt(elem, "site", actuator->get_target());
        WriteAttrTxt(elem, "refsite", actuator->get_refsite());
        break;

      case mjTRN_BODY:
        WriteAttrTxt(elem, "body", actuator->get_target());
        break;

      default:      // SHOULD NOT OCCUR
        break;
    }
  }

  // defaults and regular
  WriteAttrInt(elem, "group", actuator->group, def->Actuator().group);
  WriteAttrKey(elem, "ctrllimited", TFAuto_map, 3, actuator->ctrllimited, def->Actuator().ctrllimited);
  WriteAttr(elem, "ctrlrange", 2, actuator->ctrlrange, def->Actuator().ctrlrange);
  WriteAttrKey(elem, "forcelimited", TFAuto_map, 3, actuator->forcelimited, def->Actuator().forcelimited);
  WriteAttr(elem, "forcerange", 2, actuator->forcerange, def->Actuator().forcerange);
  WriteAttrKey(elem, "actlimited", TFAuto_map, 3, actuator->actlimited, def->Actuator().actlimited);
  WriteAttr(elem, "actrange", 2, actuator->actrange, def->Actuator().actrange);
  WriteAttr(elem, "lengthrange", 2, actuator->lengthrange, def->Actuator().lengthrange);
  WriteAttr(elem, "gear", 6, actuator->gear, def->Actuator().gear);
  WriteAttr(elem, "cranklength", 1, &actuator->cranklength, &def->Actuator().cranklength);
  WriteAttrKey(elem, "actearly", bool_map, 2, actuator->actearly,
               def->Actuator().actearly);
  // special handling of actdim which has default value of -1
  if (writingdefaults) {
    WriteAttrInt(elem, "actdim", actuator->actdim, def->Actuator().actdim);
  } else {
    int default_actdim = actuator->dyntype == mjDYN_NONE ? 0 : 1;
    WriteAttrInt(elem, "actdim", actuator->actdim, default_actdim);
  }
  WriteAttrKey(elem, "dyntype", dyn_map, dyn_sz, actuator->dyntype, def->Actuator().dyntype);
  WriteAttr(elem, "dynprm", mjNDYN, actuator->dynprm, def->Actuator().dynprm);

  // plugins: write config attributes
  if (actuator->plugin.active) {
    OnePlugin(elem, &actuator->plugin);
  }

  // non-plugins: write actuator parameters
  else {
    WriteAttrKey(elem, "gaintype", gain_map, gain_sz, actuator->gaintype, def->Actuator().gaintype);
    WriteAttrKey(elem, "biastype", bias_map, bias_sz, actuator->biastype, def->Actuator().biastype);
    WriteAttr(elem, "gainprm", mjNGAIN, actuator->gainprm, def->Actuator().gainprm, true);
    WriteAttr(elem, "biasprm", mjNBIAS, actuator->biasprm, def->Actuator().biasprm, true);
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", actuator->get_userdata());
  } else {
    WriteVector(elem, "user", actuator->get_userdata(), def->Actuator().get_userdata());
  }
}



// write plugin
void mjXWriter::OnePlugin(XMLElement* elem, const mjsPlugin* plugin) {
  const string instance_name = string(mjs_getString(plugin->name));
  const string plugin_name = string(mjs_getString(plugin->plugin_name));
  if (!instance_name.empty()) {
    WriteAttrTxt(elem, "instance", instance_name);
  } else {
    WriteAttrTxt(elem, "plugin", plugin_name);
    const mjpPlugin* pplugin = mjp_getPluginAtSlot(
      static_cast<mjCPlugin*>(plugin->element)->plugin_slot);
    const char* c = &(static_cast<mjCPlugin*>(plugin->element)->flattened_attributes[0]);
    for (int i = 0; i < pplugin->nattribute; ++i) {
      string value(c);
      if (!value.empty()) {
        XMLElement* config_elem = InsertEnd(elem, "config");
        WriteAttrTxt(config_elem, "key", pplugin->attributes[i]);
        WriteAttrTxt(config_elem, "value", value);
        c += value.size();
      }
      ++c;
    }
  }
}



//---------------------------------- class mjXWriter: top-level API --------------------------------

// constructor
mjXWriter::mjXWriter(void) {
  writingdefaults = false;
}


// cast model
void mjXWriter::SetModel(mjSpec* _spec, const mjModel* m) {
  if (_spec) {
    model = static_cast<mjCModel*>(_spec->element);
  }
  if (m) {
    mj_copyBack(&model->spec, m);
  }
}


// save existing model in MJCF canonical format, must be compiled
string mjXWriter::Write(char *error, size_t error_sz) {
  // check model
  if (!model || !model->IsCompiled()) {
    mjCopyError(error, "XML Write error: Only compiled model can be written", error_sz);
    return "";
  }

  // create document and root
  XMLDocument doc;
  XMLElement* root = doc.NewElement("mujoco");
  root->SetAttribute("model", mjs_getString(model->modelname));

  // insert root
  doc.InsertFirstChild(root);

  // write comment if present
  string text = mjs_getString(model->comment);
  if (!text.empty()) {
    XMLComment* comment = doc.NewComment(text.c_str());
    root->LinkEndChild(comment);
  }

  // create DOM
  Compiler(root);
  Option(root);
  Size(root);
  Visual(root);
  Statistic(root);
  writingdefaults = true;
  Default(root, model->Default());
  writingdefaults = false;
  Extension(root);
  Custom(root);
  Asset(root);
  Body(InsertEnd(root, "worldbody"), model->GetWorld(), nullptr);
  Contact(root);
  Deformable(root);
  Equality(root);
  Tendon(root);
  Actuator(root);
  Sensor(root);
  Keyframe(root);

  return WriteDoc(doc, error, error_sz);
}



// compiler section
void mjXWriter::Compiler(XMLElement* root) {
  XMLElement* section = InsertEnd(root, "compiler");

  // settings
  WriteAttrTxt(section, "angle", "radian");
  if (!model->get_meshdir().empty()) {
    WriteAttrTxt(section, "meshdir", model->get_meshdir());
  }
  if (!model->get_texturedir().empty()) {
    WriteAttrTxt(section, "texturedir", model->get_texturedir());
  }
  if (!model->compiler.usethread) {
    WriteAttrTxt(section, "usethread", "false");
  }

  if (model->compiler.boundmass) {
    WriteAttr(section, "boundmass", 1, &model->compiler.boundmass);
  }
  if (model->compiler.boundinertia) {
    WriteAttr(section, "boundinertia", 1, &model->compiler.boundinertia);
  }
  if (model->compiler.alignfree) {
    WriteAttrTxt(section, "alignfree", "true");
  }
  if (!model->compiler.autolimits) {
    WriteAttrTxt(section, "autolimits", "false");
  }
}



// option section
void mjXWriter::Option(XMLElement* root) {
  mjOption opt;
  mj_defaultOption(&opt);

  XMLElement* section = InsertEnd(root, "option");

  // option
  WriteAttr(section, "timestep", 1, &model->option.timestep, &opt.timestep);
  WriteAttr(section, "apirate", 1, &model->option.apirate, &opt.apirate);
  WriteAttr(section, "impratio", 1, &model->option.impratio, &opt.impratio);
  WriteAttr(section, "tolerance", 1, &model->option.tolerance, &opt.tolerance);
  WriteAttr(section, "ls_tolerance", 1, &model->option.ls_tolerance, &opt.ls_tolerance);
  WriteAttr(section, "noslip_tolerance", 1, &model->option.noslip_tolerance, &opt.noslip_tolerance);
  WriteAttr(section, "ccd_tolerance", 1, &model->option.ccd_tolerance, &opt.ccd_tolerance);
  WriteAttr(section, "gravity", 3, model->option.gravity, opt.gravity);
  WriteAttr(section, "wind", 3, model->option.wind, opt.wind);
  WriteAttr(section, "magnetic", 3, model->option.magnetic, opt.magnetic);
  WriteAttr(section, "density", 1, &model->option.density, &opt.density);
  WriteAttr(section, "viscosity", 1, &model->option.viscosity, &opt.viscosity);

  WriteAttr(section, "o_margin", 1, &model->option.o_margin, &opt.o_margin);
  WriteAttr(section, "o_solref", mjNREF, model->option.o_solref, opt.o_solref);
  WriteAttr(section, "o_solimp", mjNIMP, model->option.o_solimp, opt.o_solimp);
  WriteAttr(section, "o_friction", 5, model->option.o_friction, opt.o_friction);

  WriteAttrKey(section, "integrator", integrator_map, integrator_sz,
               model->option.integrator, opt.integrator);
  WriteAttrKey(section, "cone", cone_map, cone_sz,
               model->option.cone, opt.cone);
  WriteAttrKey(section, "jacobian", jac_map, jac_sz,
               model->option.jacobian, opt.jacobian);
  WriteAttrKey(section, "solver", solver_map, solver_sz,
               model->option.solver, opt.solver);
  WriteAttrInt(section, "iterations", model->option.iterations, opt.iterations);
  WriteAttrInt(section, "ls_iterations", model->option.ls_iterations, opt.ls_iterations);
  WriteAttrInt(section, "noslip_iterations", model->option.noslip_iterations, opt.noslip_iterations);
  WriteAttrInt(section, "ccd_iterations", model->option.ccd_iterations, opt.ccd_iterations);
  WriteAttrInt(section, "sdf_iterations", model->option.sdf_iterations, opt.sdf_iterations);
  WriteAttrInt(section, "sdf_initpoints", model->option.sdf_initpoints, opt.sdf_initpoints);

  // actuator group disable
  int disabled_groups[31];
  int ndisabled = 0;
  for (int i = 0; i < 31; ++i) {
    if (model->option.disableactuator & (1 << i)) {
      disabled_groups[ndisabled++] = i;
    }
  }
  WriteAttr(section, "actuatorgroupdisable", ndisabled, disabled_groups);

  // write disable/enable flags if any of them are set; invert while writing
  if (model->option.disableflags || model->option.enableflags) {
    XMLElement* sub = InsertEnd(section, "flag");

#define WRITEDSBL(NAME, MASK) \
    if( model->option.disableflags & MASK ) \
      WriteAttrKey(sub, NAME, enable_map, 2, 0);
    WRITEDSBL("constraint",     mjDSBL_CONSTRAINT)
    WRITEDSBL("equality",       mjDSBL_EQUALITY)
    WRITEDSBL("frictionloss",   mjDSBL_FRICTIONLOSS)
    WRITEDSBL("limit",          mjDSBL_LIMIT)
    WRITEDSBL("contact",        mjDSBL_CONTACT)
    WRITEDSBL("passive",        mjDSBL_PASSIVE)
    WRITEDSBL("gravity",        mjDSBL_GRAVITY)
    WRITEDSBL("clampctrl",      mjDSBL_CLAMPCTRL)
    WRITEDSBL("warmstart",      mjDSBL_WARMSTART)
    WRITEDSBL("filterparent",   mjDSBL_FILTERPARENT)
    WRITEDSBL("actuation",      mjDSBL_ACTUATION)
    WRITEDSBL("refsafe",        mjDSBL_REFSAFE)
    WRITEDSBL("sensor",         mjDSBL_SENSOR)
    WRITEDSBL("midphase",       mjDSBL_MIDPHASE)
    WRITEDSBL("eulerdamp",      mjDSBL_EULERDAMP)
    WRITEDSBL("autoreset",      mjDSBL_AUTORESET)
    WRITEDSBL("nativeccd",      mjDSBL_NATIVECCD)
#undef WRITEDSBL

#define WRITEENBL(NAME, MASK) \
    if( model->option.enableflags & MASK ) \
      WriteAttrKey(sub, NAME, enable_map, 2, 1);
    WRITEENBL("override",       mjENBL_OVERRIDE)
    WRITEENBL("energy",         mjENBL_ENERGY)
    WRITEENBL("fwdinv",         mjENBL_FWDINV)
    WRITEENBL("invdiscrete",    mjENBL_INVDISCRETE)
    WRITEENBL("multiccd",       mjENBL_MULTICCD)
    WRITEENBL("island",         mjENBL_ISLAND)
#undef WRITEENBL
  }

  // remove entire section if no attributes or elements
  if (!section->FirstAttribute() && !section->FirstChildElement()) {
    root->DeleteChild(section);
  }
}



// size section
void mjXWriter::Size(XMLElement* root) {
  XMLElement* section = InsertEnd(root, "size");

  // write memory
  if (model->memory != -1) {
    WriteAttrTxt(section, "memory", mju_writeNumBytes(model->memory));
  }

  // write sizes
  WriteAttrInt(section, "njmax", model->njmax, -1);
  WriteAttrInt(section, "nconmax", model->nconmax, -1);
  WriteAttrInt(section, "nstack", model->nstack, -1);
  WriteAttrInt(section, "nuserdata", model->nuserdata, 0);
  WriteAttrInt(section, "nkey", model->nkey, 0);
  WriteAttrInt(section, "nuser_body", model->nuser_body, 0);
  WriteAttrInt(section, "nuser_jnt", model->nuser_jnt, 0);
  WriteAttrInt(section, "nuser_geom", model->nuser_geom, 0);
  WriteAttrInt(section, "nuser_site", model->nuser_site, 0);
  WriteAttrInt(section, "nuser_cam", model->nuser_cam, 0);
  WriteAttrInt(section, "nuser_tendon", model->nuser_tendon, 0);
  WriteAttrInt(section, "nuser_actuator", model->nuser_actuator, 0);
  WriteAttrInt(section, "nuser_sensor", model->nuser_sensor, 0);

  // remove entire section if no attributes
  if (!section->FirstAttribute()) root->DeleteChild(section);
}



// statistic section
void mjXWriter::Statistic(XMLElement* root) {
  XMLElement* section = InsertEnd(root, "statistic");
  mjStatistic* s = &model->stat;

  if (mjuu_defined(s->meaninertia)) WriteAttr(section, "meaninertia", 1, &s->meaninertia);
  if (mjuu_defined(s->meanmass)) WriteAttr(section, "meanmass", 1, &s->meanmass);
  if (mjuu_defined(s->meansize)) WriteAttr(section, "meansize", 1, &s->meansize);
  if (mjuu_defined(s->extent)) WriteAttr(section, "extent", 1, &s->extent);
  if (mjuu_defined(s->center[0])) WriteAttr(section, "center", 3, s->center);

  // remove entire section if no attributes
  if (!section->FirstAttribute()) root->DeleteChild(section);
}



// visual section
void mjXWriter::Visual(XMLElement* root) {
  mjVisual visdef, *vis = &model->visual;
  mj_defaultVisual(&visdef);
  XMLElement* elem;

  XMLElement* section = InsertEnd(root, "visual");

  // global
  elem = InsertEnd(section, "global");
  WriteAttrKey(elem, "orthographic",
               bool_map, 2, vis->global.orthographic, visdef.global.orthographic);
  WriteAttr(elem,    "fovy",      1,   &vis->global.fovy,        &visdef.global.fovy);
  WriteAttr(elem,    "ipd",       1,   &vis->global.ipd,         &visdef.global.ipd);
  WriteAttr(elem,    "azimuth",   1,   &vis->global.azimuth,     &visdef.global.azimuth);
  WriteAttr(elem,    "elevation", 1,   &vis->global.elevation,   &visdef.global.elevation);
  WriteAttr(elem,    "linewidth", 1,   &vis->global.linewidth,   &visdef.global.linewidth);
  WriteAttr(elem,    "glow",      1,   &vis->global.glow,        &visdef.global.glow);
  WriteAttr(elem,    "realtime",  1,   &vis->global.realtime,    &visdef.global.realtime);
  WriteAttrInt(elem, "offwidth",       vis->global.offwidth,     visdef.global.offwidth);
  WriteAttrInt(elem, "offheight",      vis->global.offheight,    visdef.global.offheight);
  WriteAttrKey(elem, "ellipsoidinertia",
               bool_map, 2, vis->global.ellipsoidinertia, visdef.global.ellipsoidinertia);
  WriteAttrKey(elem, "bvactive", bool_map, 2, vis->global.bvactive, visdef.global.bvactive);
  if (!elem->FirstAttribute()) {
    section->DeleteChild(elem);
  }

  // quality
  elem = InsertEnd(section, "quality");
  WriteAttrInt(elem, "shadowsize", vis->quality.shadowsize, visdef.quality.shadowsize);
  WriteAttrInt(elem, "offsamples", vis->quality.offsamples, visdef.quality.offsamples);
  WriteAttrInt(elem, "numslices",  vis->quality.numslices,  visdef.quality.numslices);
  WriteAttrInt(elem, "numstacks",  vis->quality.numstacks,  visdef.quality.numstacks);
  WriteAttrInt(elem, "numquads",   vis->quality.numquads,   visdef.quality.numquads);
  if (!elem->FirstAttribute()) {
    section->DeleteChild(elem);
  }

  // headlight
  elem = InsertEnd(section, "headlight");
  WriteAttr(elem, "ambient",  3, vis->headlight.ambient,   visdef.headlight.ambient);
  WriteAttr(elem, "diffuse",  3, vis->headlight.diffuse,   visdef.headlight.diffuse);
  WriteAttr(elem, "specular", 3, vis->headlight.specular,  visdef.headlight.specular);
  WriteAttrInt(elem, "active",   vis->headlight.active,    visdef.headlight.active);
  if (!elem->FirstAttribute()) {
    section->DeleteChild(elem);
  }

  // map
  elem = InsertEnd(section, "map");
  WriteAttr(elem, "stiffness",      1, &vis->map.stiffness,      &visdef.map.stiffness);
  WriteAttr(elem, "stiffnessrot",   1, &vis->map.stiffnessrot,   &visdef.map.stiffnessrot);
  WriteAttr(elem, "force",          1, &vis->map.force,          &visdef.map.force);
  WriteAttr(elem, "torque",         1, &vis->map.torque,         &visdef.map.torque);
  WriteAttr(elem, "alpha",          1, &vis->map.alpha,          &visdef.map.alpha);
  WriteAttr(elem, "fogstart",       1, &vis->map.fogstart,       &visdef.map.fogstart);
  WriteAttr(elem, "fogend",         1, &vis->map.fogend,         &visdef.map.fogend);
  WriteAttr(elem, "znear",          1, &vis->map.znear,          &visdef.map.znear);
  WriteAttr(elem, "zfar",           1, &vis->map.zfar,           &visdef.map.zfar);
  WriteAttr(elem, "haze",           1, &vis->map.haze,           &visdef.map.haze);
  WriteAttr(elem, "shadowclip",     1, &vis->map.shadowclip,     &visdef.map.shadowclip);
  WriteAttr(elem, "shadowscale",    1, &vis->map.shadowscale,    &visdef.map.shadowscale);
  WriteAttr(elem, "actuatortendon", 1, &vis->map.actuatortendon, &visdef.map.actuatortendon);
  if (!elem->FirstAttribute()) {
    section->DeleteChild(elem);
  }

  // scale
  elem = InsertEnd(section, "scale");
  WriteAttr(elem, "forcewidth",     1, &vis->scale.forcewidth,     &visdef.scale.forcewidth);
  WriteAttr(elem, "contactwidth",   1, &vis->scale.contactwidth,   &visdef.scale.contactwidth);
  WriteAttr(elem, "contactheight",  1, &vis->scale.contactheight,  &visdef.scale.contactheight);
  WriteAttr(elem, "connect",        1, &vis->scale.connect,        &visdef.scale.connect);
  WriteAttr(elem, "com",            1, &vis->scale.com,            &visdef.scale.com);
  WriteAttr(elem, "camera",         1, &vis->scale.camera,         &visdef.scale.camera);
  WriteAttr(elem, "light",          1, &vis->scale.light,          &visdef.scale.light);
  WriteAttr(elem, "selectpoint",    1, &vis->scale.selectpoint,    &visdef.scale.selectpoint);
  WriteAttr(elem, "jointlength",    1, &vis->scale.jointlength,    &visdef.scale.jointlength);
  WriteAttr(elem, "jointwidth",     1, &vis->scale.jointwidth,     &visdef.scale.jointwidth);
  WriteAttr(elem, "actuatorlength", 1, &vis->scale.actuatorlength, &visdef.scale.actuatorlength);
  WriteAttr(elem, "actuatorwidth",  1, &vis->scale.actuatorwidth,  &visdef.scale.actuatorwidth);
  WriteAttr(elem, "framelength",    1, &vis->scale.framelength,    &visdef.scale.framelength);
  WriteAttr(elem, "framewidth",     1, &vis->scale.framewidth,     &visdef.scale.framewidth);
  WriteAttr(elem, "constraint",     1, &vis->scale.constraint,     &visdef.scale.constraint);
  WriteAttr(elem, "slidercrank",    1, &vis->scale.slidercrank,    &visdef.scale.slidercrank);
  WriteAttr(elem, "frustum",        1, &vis->scale.frustum,        &visdef.scale.frustum);
  if (!elem->FirstAttribute()) {
    section->DeleteChild(elem);
  }

  // rgba
  elem = InsertEnd(section, "rgba");
  WriteAttr(elem, "fog",              4, vis->rgba.fog,              visdef.rgba.fog);
  WriteAttr(elem, "haze",             4, vis->rgba.haze,             visdef.rgba.haze);
  WriteAttr(elem, "force",            4, vis->rgba.force,            visdef.rgba.force);
  WriteAttr(elem, "inertia",          4, vis->rgba.inertia,          visdef.rgba.inertia);
  WriteAttr(elem, "joint",            4, vis->rgba.joint,            visdef.rgba.joint);
  WriteAttr(elem, "actuator",         4, vis->rgba.actuator,         visdef.rgba.actuator);
  WriteAttr(elem, "actuatornegative", 4, vis->rgba.actuatornegative, visdef.rgba.actuatornegative);
  WriteAttr(elem, "actuatorpositive", 4, vis->rgba.actuatorpositive, visdef.rgba.actuatorpositive);
  WriteAttr(elem, "com",              4, vis->rgba.com,              visdef.rgba.com);
  WriteAttr(elem, "camera",           4, vis->rgba.camera,           visdef.rgba.camera);
  WriteAttr(elem, "light",            4, vis->rgba.light,            visdef.rgba.light);
  WriteAttr(elem, "selectpoint",      4, vis->rgba.selectpoint,      visdef.rgba.selectpoint);
  WriteAttr(elem, "connect",          4, vis->rgba.connect,          visdef.rgba.connect);
  WriteAttr(elem, "contactpoint",     4, vis->rgba.contactpoint,     visdef.rgba.contactpoint);
  WriteAttr(elem, "contactforce",     4, vis->rgba.contactforce,     visdef.rgba.contactforce);
  WriteAttr(elem, "contactfriction",  4, vis->rgba.contactfriction,  visdef.rgba.contactfriction);
  WriteAttr(elem, "contacttorque",    4, vis->rgba.contacttorque,    visdef.rgba.contacttorque);
  WriteAttr(elem, "contactgap",       4, vis->rgba.contactgap,       visdef.rgba.contactgap);
  WriteAttr(elem, "rangefinder",      4, vis->rgba.rangefinder,      visdef.rgba.rangefinder);
  WriteAttr(elem, "constraint",       4, vis->rgba.constraint,       visdef.rgba.constraint);
  WriteAttr(elem, "slidercrank",      4, vis->rgba.slidercrank,      visdef.rgba.slidercrank);
  WriteAttr(elem, "crankbroken",      4, vis->rgba.crankbroken,      visdef.rgba.crankbroken);
  WriteAttr(elem, "frustum",          4, vis->rgba.frustum,          visdef.rgba.frustum);
  WriteAttr(elem, "bv",               4, vis->rgba.bv,               visdef.rgba.bv);
  WriteAttr(elem, "bvactive",         4, vis->rgba.bvactive,         visdef.rgba.bvactive);
  if (!elem->FirstAttribute()) {
    section->DeleteChild(elem);
  }

  // remove entire section if no elements
  if (!section->FirstChildElement()) {
    root->DeleteChild(section);
  }
}



// default section
void mjXWriter::Default(XMLElement* root, mjCDef* def) {
  XMLElement* elem;
  XMLElement* section;

  // pointer to parent defaults
  mjCDef* parent;
  if (def->parent) {
    parent = def->parent;
  } else {
    parent = new mjCDef;
  }

  // create section, write class name
  section = InsertEnd(root, "default");
  if (def->name != "main") {
    WriteAttrTxt(section, "class", def->name);
  }

  // mesh
  elem = InsertEnd(section, "mesh");
  OneMesh(elem, &def->Mesh(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // material
  elem = InsertEnd(section, "material");
  OneMaterial(elem, &def->Material(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // joint
  elem = InsertEnd(section, "joint");
  OneJoint(elem, &def->Joint(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // geom
  elem = InsertEnd(section, "geom");
  OneGeom(elem, &def->Geom(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // site
  elem = InsertEnd(section, "site");
  OneSite(elem, &def->Site(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // camera
  elem = InsertEnd(section, "camera");
  OneCamera(elem, &def->Camera(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // light
  elem = InsertEnd(section, "light");
  OneLight(elem, &def->Light(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // pair
  elem = InsertEnd(section, "pair");
  OnePair(elem, &def->Pair(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // equality
  elem = InsertEnd(section, "equality");
  OneEquality(elem, &def->Equality(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // tendon
  elem = InsertEnd(section, "tendon");
  OneTendon(elem, &def->Tendon(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // actuator
  elem = InsertEnd(section, "general");
  OneActuator(elem, &def->Actuator(), parent);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // if top-level class has no members or children, delete it and return
  if (!def->parent && section->NoChildren() && def->child.empty()) {
    root->DeleteChild(section);
    delete parent;
    return;
  }

  // add children recursively
  for (int i=0; i < (int)def->child.size(); i++) {
    Default(section, def->child[i]);
  }

  // delete parent defaults if allocated here
  if (!def->parent) {
    delete parent;
  }
}



// extension section
void mjXWriter::Extension(XMLElement* root) {
  // skip section if there is no required plugin
  if (model->ActivePlugins().empty()) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "extension");

  // keep track of plugins whose <plugin> section have been created
  std::unordered_set<const mjpPlugin*> seen_plugins;

  // write all plugins
  const mjpPlugin* last_plugin = nullptr;
  XMLElement* plugin_elem = nullptr;
  for (int i = 0; i < model->Plugins().size(); ++i) {
    mjCPlugin* pp = static_cast<mjCPlugin*>(model->GetObject(mjOBJ_PLUGIN, i));

    if (pp->name.empty()) {
      // reached the first unnamed plugin instance, meaning that it was created through an
      // "implicit" plugin element, e.g. sensor or actuator
      break;
    }

    // check if we need to open a new <plugin> section
    const mjpPlugin* plugin = mjp_getPluginAtSlot(pp->plugin_slot);
    if (plugin != last_plugin) {
      plugin_elem = InsertEnd(section, "plugin");
      WriteAttrTxt(plugin_elem, "plugin", plugin->name);
      seen_plugins.insert(plugin);
      last_plugin = plugin;
    }

    // write instance element
    XMLElement* elem = InsertEnd(plugin_elem, "instance");
    WriteAttrTxt(elem, "name", pp->name);

    // write plugin config attributes
    const char* c = &pp->flattened_attributes[0];
    for (int i = 0; i < plugin->nattribute; ++i) {
      string value(c);
      if (!value.empty()) {
        XMLElement* config_elem = InsertEnd(elem, "config");
        WriteAttrTxt(config_elem, "key", plugin->attributes[i]);
        WriteAttrTxt(config_elem, "value", value);
        c += value.size();
      }
      ++c;
    }
  }

  // write <plugin> elements for plugins without explicit instances
  for (const auto& [plugin, slot] : model->ActivePlugins()) {
    if (seen_plugins.find(plugin) == seen_plugins.end()) {
      plugin_elem = InsertEnd(section, "plugin");
      WriteAttrTxt(plugin_elem, "plugin", plugin->name);
    }
  }
}



// custom section
void mjXWriter::Custom(XMLElement* root) {
  XMLElement* elem;

  // get sizes, skip section if empty
  int nnum = model->NumObjects(mjOBJ_NUMERIC);
  int ntxt = model->NumObjects(mjOBJ_TEXT);
  int ntup = model->NumObjects(mjOBJ_TUPLE);

  // skip section if empty
  if (nnum == 0 && ntxt == 0 && ntup == 0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "custom");

  // write all numerics
  for (int i=0; i < nnum; i++) {
    mjCNumeric* numeric = (mjCNumeric*)model->GetObject(mjOBJ_NUMERIC, i);
    elem = InsertEnd(section, "numeric");
    WriteAttrTxt(elem, "name", numeric->name);
    WriteAttrInt(elem, "size", numeric->size);
    WriteAttr(elem, "data", numeric->size, numeric->data_.data());
  }

  // write all texts
  for (int i=0; i < ntxt; i++) {
    mjCText* text = (mjCText*)model->GetObject(mjOBJ_TEXT, i);
    elem = InsertEnd(section, "text");
    WriteAttrTxt(elem, "name", text->name);
    WriteAttrTxt(elem, "data", text->data_.c_str());
  }

  // write all tuples
  for (int i=0; i < ntup; i++) {
    mjCTuple* tuple = (mjCTuple*)model->GetObject(mjOBJ_TUPLE, i);
    elem = InsertEnd(section, "tuple");
    WriteAttrTxt(elem, "name", tuple->name);

    // write objects in tuple
    for (int j=0; j < (int)tuple->objtype_.size(); j++) {
      XMLElement* obj = InsertEnd(elem, "element");
      WriteAttrTxt(obj, "objtype", mju_type2Str((int)tuple->objtype_[j]));
      WriteAttrTxt(obj, "objname", tuple->objname_[j].c_str());
      double oprm = tuple->objprm_[j];
      if (oprm != 0) {
        WriteAttr(obj, "prm", 1, &oprm);
      }
    }
  }
}



// asset section
void mjXWriter::Asset(XMLElement* root) {
  XMLElement* elem;

  // get sizes
  int ntex = model->NumObjects(mjOBJ_TEXTURE);
  int nmat = model->NumObjects(mjOBJ_MATERIAL);
  int nmesh = model->NumObjects(mjOBJ_MESH);
  int nhfield = model->NumObjects(mjOBJ_HFIELD);

  // return if empty
  if (ntex == 0 && nmat == 0 && nmesh == 0 && nhfield == 0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "asset");

  // write textures
  mjCTexture deftex(0);
  for (int i=0; i < ntex; i++) {
    // create element
    mjCTexture* texture = (mjCTexture*)model->GetObject(mjOBJ_TEXTURE, i);
    elem = InsertEnd(section, "texture");

    // write common attributes
    WriteAttrKey(elem, "type", texture_map, texture_sz, texture->type);
    WriteAttrKey(elem, "colorspace", colorspace_map, colorspace_sz, texture->colorspace);
    WriteAttrTxt(elem, "name", texture->name);

    // write builtin
    if (texture->builtin != mjBUILTIN_NONE) {
      WriteAttrKey(elem, "builtin", builtin_map, builtin_sz, texture->builtin);
      WriteAttrKey(elem, "mark", mark_map, mark_sz, texture->mark, deftex.mark);
      WriteAttr(elem, "rgb1", 3, texture->rgb1, deftex.rgb1);
      WriteAttr(elem, "rgb2", 3, texture->rgb2, deftex.rgb2);
      WriteAttr(elem, "markrgb", 3, texture->markrgb, deftex.markrgb);
      WriteAttr(elem, "random", 1, &texture->random, &deftex.random);
      WriteAttrInt(elem, "width", texture->width);
      WriteAttrInt(elem, "height", texture->height);
    }

    // write buffer
    else if (texture->get_cubefiles()[0].empty() && texture->get_cubefiles()[1].empty() &&
             texture->get_cubefiles()[2].empty() && texture->get_cubefiles()[3].empty() &&
             texture->get_cubefiles()[4].empty() && texture->get_cubefiles()[5].empty() &&
             texture->File().empty() && texture->gridsize[0] == 1 && texture->gridsize[1] == 1) {
      throw mjXError(0, "no support for buffer textures.");
    }

    // write textures loaded from files
    else {
      // write single file
      WriteAttrTxt(elem, "content_type", texture->get_content_type());
      WriteAttrTxt(elem, "file", texture->File());

      // write separate files
      WriteAttrTxt(elem, "fileright", texture->get_cubefiles()[0]);
      WriteAttrTxt(elem, "fileleft", texture->get_cubefiles()[1]);
      WriteAttrTxt(elem, "fileup", texture->get_cubefiles()[2]);
      WriteAttrTxt(elem, "filedown", texture->get_cubefiles()[3]);
      WriteAttrTxt(elem, "filefront", texture->get_cubefiles()[4]);
      WriteAttrTxt(elem, "fileback", texture->get_cubefiles()[5]);
      if (texture->hflip) {
        WriteAttrKey(elem, "hflip", bool_map, 2, 1);
      }
      if (texture->vflip) {
        WriteAttrKey(elem, "vflip", bool_map, 2, 1);
      }

      // write grid
      if (texture->gridsize[0] != 1 || texture->gridsize[1] != 1) {
        double gsize[2] = { (double)texture->gridsize[0], (double)texture->gridsize[1] };
        WriteAttr(elem, "gridsize", 2, gsize);
        WriteAttrTxt(elem, "gridlayout", texture->gridlayout);
      }
    }
  }

  // write materials
  for (int i=0; i < nmat; i++) {
    // create element and write
    mjCMaterial* material = (mjCMaterial*)model->GetObject(mjOBJ_MATERIAL, i);
    elem = InsertEnd(section, "material");
    OneMaterial(elem, material, model->def_map[material->classname]);
  }

  // write meshes
  for (int i=0; i < nmesh; i++) {
    // create element and write
    mjCMesh* mesh = (mjCMesh*)model->GetObject(mjOBJ_MESH, i);
    if (mesh->Plugin().active) {
      elem = InsertEnd(section, "mesh");
      WriteAttrTxt(elem, "name", mesh->name);
      WriteAttrTxt(elem, "file", mesh->File());
      OnePlugin(InsertEnd(elem, "plugin"), &mesh->Plugin());
    } else{
      elem = InsertEnd(section, "mesh");
      OneMesh(elem, mesh, model->def_map[mesh->classname]);
    }
  }

  // write hfields
  for (int i=0; i < nhfield; i++) {
    // create element
    mjCHField* hfield = (mjCHField*)model->GetObject(mjOBJ_HFIELD, i);
    elem = InsertEnd(section, "hfield");

    // write attributes
    WriteAttrTxt(elem, "name", hfield->name);
    WriteAttr(elem, "size", 4, hfield->size);
    if (!hfield->file_.empty()) {
      WriteAttrTxt(elem, "content_type", hfield->content_type_);
      WriteAttrTxt(elem, "file", hfield->file_);
    } else {
      WriteAttrInt(elem, "nrow", hfield->nrow);
      WriteAttrInt(elem, "ncol", hfield->ncol);
      if (!hfield->get_userdata().empty()) {
        string text;
        Vector2String(text, hfield->get_userdata(), hfield->ncol);
        WriteAttrTxt(elem, "elevation", text);
      }
    }
  }
}



XMLElement* mjXWriter::OneFrame(XMLElement* elem, mjCFrame* frame) {
  if (!frame) {
    return elem;
  }

  // TODO: empty classname should not occur (but does)
  if (frame->name.empty() && (frame->classname.empty() || frame->classname == "main")) {
    return elem;
  }

  XMLElement* frame_elem = InsertEnd(elem, "frame");
  WriteAttrTxt(frame_elem, "name", frame->name);
  if (frame->classname != "main") {
    WriteAttrTxt(frame_elem, "childclass", frame->classname);
  }
  return frame_elem;
}



// recursive body and frame writer
void mjXWriter::Body(XMLElement* elem, mjCBody* body, mjCFrame* frame, string_view childclass) {
  double unitq[4] = {1, 0, 0, 0};

  if (!body) {
    throw mjXError(0, "missing body in XML write");  // SHOULD NOT OCCUR
  }

  // write body attributes and inertial
  else if (!frame && body != model->GetWorld()) {
    WriteAttrTxt(elem, "name", body->name);
    if (childclass != body->classname && body->classname != "main") {
      WriteAttrTxt(elem, "childclass", body->classname);
    }

    // write pos if it's not {0, 0, 0}
    if (body->pos[0] || body->pos[1] || body->pos[2]) {
      WriteAttr(elem, "pos", 3, body->pos);
    }
    WriteAttr(elem, "quat", 4, body->quat, unitq);
    if (body->mocap) {
      WriteAttrKey(elem, "mocap", bool_map, 2, 1);
    }

    // gravity compensation
    if (body->gravcomp) {
      WriteAttr(elem, "gravcomp", 1, &body->gravcomp);
    }
    // userdata
    WriteVector(elem, "user", body->get_userdata());

    // write inertial
    if (model->compiler.saveinertial ||
        (body->explicitinertial && model->compiler.inertiafromgeom != mjINERTIAFROMGEOM_TRUE)) {
      XMLElement* inertial = InsertEnd(elem, "inertial");
      WriteAttr(inertial, "pos", 3, body->ipos);
      WriteAttr(inertial, "quat", 4, body->iquat, unitq);
      WriteAttr(inertial, "mass", 1, &body->mass);
      WriteAttr(inertial, "diaginertia", 3, body->inertia);
    }
  }

  // joints in this frame
  for (int i = 0; i < body->joints.size(); i++) {
    if (body->joints[i]->frame != frame) {
      continue;
    }
    string classname = body->joints[i]->frame && !body->joints[i]->frame->classname.empty()
                           ? body->joints[i]->frame->classname
                           : body->classname;
    OneJoint(InsertEnd(elem, "joint"), body->joints[i],
             model->def_map[body->joints[i]->classname],
             classname.empty() ? childclass : classname);
  }

  // geoms in this frame
  for (int i = 0; i < body->geoms.size(); i++) {
    if (body->geoms[i]->frame != frame) {
      continue;
    }
    string classname = body->geoms[i]->frame && !body->geoms[i]->frame->classname.empty()
                           ? body->geoms[i]->frame->classname
                           : body->classname;
    OneGeom(InsertEnd(elem, "geom"), body->geoms[i],
            model->def_map[body->geoms[i]->classname],
            classname.empty() ? childclass : classname);
  }

  // sites in this frame
  for (int i = 0; i < body->sites.size(); i++) {
    if (body->sites[i]->frame != frame) {
      continue;
    }
    string classname = body->sites[i]->frame && !body->sites[i]->frame->classname.empty()
                           ? body->sites[i]->frame->classname
                           : body->classname;
    OneSite(InsertEnd(elem, "site"), body->sites[i],
            model->def_map[body->sites[i]->classname],
            classname.empty() ? childclass : classname);
  }

  // cameras in this frame
  for (int i = 0; i < body->cameras.size(); i++) {
    if (body->cameras[i]->frame != frame) {
      continue;
    }
    string classname = body->cameras[i]->frame && !body->cameras[i]->frame->classname.empty()
                           ? body->cameras[i]->frame->classname
                           : body->classname;
    OneCamera(InsertEnd(elem, "camera"), body->cameras[i],
              model->def_map[body->cameras[i]->classname],
              classname.empty() ? childclass : classname);
  }

  // lights in this frame
  for (int i = 0; i < body->lights.size(); i++) {
    if (body->lights[i]->frame != frame) {
      continue;
    }
    string classname = body->lights[i]->frame && !body->lights[i]->frame->classname.empty()
                           ? body->lights[i]->frame->classname
                           : body->classname;
    OneLight(InsertEnd(elem, "light"), body->lights[i],
             model->def_map[body->lights[i]->classname],
             classname.empty() ? childclass : classname);
  }

  // write plugin
  if (body->plugin.active) {
    OnePlugin(InsertEnd(elem, "plugin"), &body->plugin);
  }

  // write children recursively
  int i = 0, j = 0;
  while (i < body->bodies.size() || body->bodies.empty()) {
    mjCFrame* bframe = body->bodies.empty() ? nullptr : body->bodies[i]->frame;

    // write body if its frame matches the current frame, avoid access if there are no bodies
    if (bframe == frame && !body->bodies.empty()) {
      string classname = bframe && !bframe->classname.empty()
                             ? bframe->classname
                             : body->classname;
      Body(InsertEnd(elem, "body"), body->bodies[i], nullptr,
           classname.empty() ? childclass : classname);
    }

    i++;

    // do not go to frames until we reach a body with a frame or we are done with bodies
    if (!bframe && i < body->bodies.size()) {
      continue;
    }

    // loop over the remaining frames in the current body
    while (j < body->frames.size()) {
      mjCFrame* fframe = body->frames[j++];

      // write frame if its frame matches the current frame
      if (fframe->frame == frame) {
        string classname = fframe && !fframe->classname.empty()
                               ? fframe->classname
                               : body->classname;
        Body(OneFrame(elem, fframe), body, fframe, childclass);
      }
    }

    // if there are no bodies, we only want to run the loop once
    if (body->bodies.empty()) {
      break;
    }
  }
}



// collision section
void mjXWriter::Contact(XMLElement* root) {
  XMLElement* elem;

  // get number of pairs of each type
  int npair = model->NumObjects(mjOBJ_PAIR);
  int nexclude = model->NumObjects(mjOBJ_EXCLUDE);

  // skip if section is empty
  if (npair == 0 && nexclude == 0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "contact");

  // write all geom pairs
  for (int i=0; i < npair; i++) {
    // create element and write
    mjCPair* pair = (mjCPair*)model->GetObject(mjOBJ_PAIR, i);
    elem = InsertEnd(section, "pair");
    OnePair(elem, pair, model->def_map[pair->classname]);
  }

  // write all exclude pairs
  for (int i=0; i < nexclude; i++) {
    // create element
    mjCBodyPair* exclude = (mjCBodyPair*)model->GetObject(mjOBJ_EXCLUDE, i);
    elem = InsertEnd(section, "exclude");

    // write attributes
    WriteAttrTxt(elem, "name", exclude->name);
    WriteAttrTxt(elem, "body1", exclude->get_bodyname1());
    WriteAttrTxt(elem, "body2", exclude->get_bodyname2());
  }
}



// constraint section
void mjXWriter::Equality(XMLElement* root) {
  // skip section if empty
  int num;
  if ((num=model->NumObjects(mjOBJ_EQUALITY)) == 0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "equality");

  // write all constraints
  for (int i=0; i < num; i++) {
    mjCEquality* equality = (mjCEquality*)model->GetObject(mjOBJ_EQUALITY, i);
    XMLElement* elem = InsertEnd(section,
                                 FindValue(equality_map, equality_sz, equality->type).c_str());
    OneEquality(elem, equality, model->def_map[equality->classname]);
  }
}



// deformable section
void mjXWriter::Deformable(XMLElement* root) {
  XMLElement* elem;

  // get sizes
  int nflex = model->NumObjects(mjOBJ_FLEX);
  int nskin = model->NumObjects(mjOBJ_SKIN);

  // return if empty
  if (nflex == 0 && nskin == 0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "deformable");

  // write flexes
  for (int i=0; i < nflex; i++) {
    // create element and write
    mjCFlex* flex = (mjCFlex*)model->GetObject(mjOBJ_FLEX, i);
    elem = InsertEnd(section, "flex");
    OneFlex(elem, flex);
  }

  // write skins
  for (int i=0; i < nskin; i++) {
    // create element and write
    mjCSkin* skin = (mjCSkin*)model->GetObject(mjOBJ_SKIN, i);
    elem = InsertEnd(section, "skin");
    OneSkin(elem, skin);
  }
}



// tendon section
void mjXWriter::Tendon(XMLElement* root) {
  // skip section if empty
  int num;
  if ((num=model->NumObjects(mjOBJ_TENDON)) == 0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "tendon");

  // write all tendons
  for (int i=0; i < num; i++) {
    // write tendon element and attributes
    mjCTendon* tendon = (mjCTendon*)model->GetObject(mjOBJ_TENDON, i);
    if (!tendon->NumWraps()) {        // SHOULD NOT OCCUR
      continue;
    }
    XMLElement* elem = InsertEnd(section,
                                 tendon->GetWrap(0)->type == mjWRAP_JOINT ? "fixed" : "spatial");
    OneTendon(elem, tendon, model->def_map[tendon->classname]);

    // write wraps
    XMLElement* wrapelem;
    for (int j=0; j < tendon->NumWraps(); j++) {
      const mjCWrap* wrap = tendon->GetWrap(j);
      switch (wrap->type) {
        case mjWRAP_JOINT:
          wrapelem = InsertEnd(elem, "joint");
          WriteAttrTxt(wrapelem, "joint", wrap->obj->name);
          WriteAttr(wrapelem, "coef", 1, &wrap->prm);
          break;

        case mjWRAP_SITE:
          wrapelem = InsertEnd(elem, "site");
          WriteAttrTxt(wrapelem, "site", wrap->obj->name);
          break;

        case mjWRAP_SPHERE:
        case mjWRAP_CYLINDER:
          wrapelem = InsertEnd(elem, "geom");
          WriteAttrTxt(wrapelem, "geom", wrap->obj->name);
          if (!wrap->sidesite.empty()) {
            WriteAttrTxt(wrapelem, "sidesite", wrap->sidesite);
          }
          break;

        case mjWRAP_PULLEY:
          wrapelem = InsertEnd(elem, "pulley");
          WriteAttr(wrapelem, "divisor", 1, &wrap->prm);
          break;

        default:
          break;
      }
    }
  }
}



// actuator section
void mjXWriter::Actuator(XMLElement* root) {
  // skip section if empty
  int num;
  if ((num=model->NumObjects(mjOBJ_ACTUATOR)) == 0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "actuator");

  // write all actuators
  for (int i=0; i < num; i++) {
    mjCActuator* actuator = (mjCActuator*)model->GetObject(mjOBJ_ACTUATOR, i);
    XMLElement* elem;
    if (actuator->plugin.active) {
      elem = InsertEnd(section, "plugin");
    } else {
      elem = InsertEnd(section, "general");
    }
    OneActuator(elem, actuator, model->def_map[actuator->classname]);
  }
}



// sensor section
void mjXWriter::Sensor(XMLElement* root) {
  double zero = 0;

  // skip section if empty
  int num;
  if ((num=model->NumObjects(mjOBJ_SENSOR)) == 0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "sensor");

  // write all sensors
  for (int i=0; i < num; i++) {
    XMLElement* elem = 0;
    mjCSensor* sensor = model->Sensors()[i];
    string instance_name = "";
    string plugin_name = "";

    // write sensor type and type-specific attributes
    switch (sensor->type) {
      // common robotic sensors, attached to a site
      case mjSENS_TOUCH:
        elem = InsertEnd(section, "touch");
        WriteAttrTxt(elem, "site", sensor->get_objname());
        break;
      case mjSENS_ACCELEROMETER:
        elem = InsertEnd(section, "accelerometer");
        WriteAttrTxt(elem, "site", sensor->get_objname());
        break;
      case mjSENS_VELOCIMETER:
        elem = InsertEnd(section, "velocimeter");
        WriteAttrTxt(elem, "site", sensor->get_objname());
        break;
      case mjSENS_GYRO:
        elem = InsertEnd(section, "gyro");
        WriteAttrTxt(elem, "site", sensor->get_objname());
        break;
      case mjSENS_FORCE:
        elem = InsertEnd(section, "force");
        WriteAttrTxt(elem, "site", sensor->get_objname());
        break;
      case mjSENS_TORQUE:
        elem = InsertEnd(section, "torque");
        WriteAttrTxt(elem, "site", sensor->get_objname());
        break;
      case mjSENS_MAGNETOMETER:
        elem = InsertEnd(section, "magnetometer");
        WriteAttrTxt(elem, "site", sensor->get_objname());
        break;
      case mjSENS_RANGEFINDER:
        elem = InsertEnd(section, "rangefinder");
        WriteAttrTxt(elem, "site", sensor->get_objname());
        break;
      case mjSENS_CAMPROJECTION:
        elem = InsertEnd(section, "camprojection");
        WriteAttrTxt(elem, "site", sensor->get_objname());
        WriteAttrTxt(elem, "camera", sensor->get_refname());
        break;

      // sensors related to scalar joints, tendons, actuators
      case mjSENS_JOINTPOS:
        elem = InsertEnd(section, "jointpos");
        WriteAttrTxt(elem, "joint", sensor->get_objname());
        break;
      case mjSENS_JOINTVEL:
        elem = InsertEnd(section, "jointvel");
        WriteAttrTxt(elem, "joint", sensor->get_objname());
        break;
      case mjSENS_TENDONPOS:
        elem = InsertEnd(section, "tendonpos");
        WriteAttrTxt(elem, "tendon", sensor->get_objname());
        break;
      case mjSENS_TENDONVEL:
        elem = InsertEnd(section, "tendonvel");
        WriteAttrTxt(elem, "tendon", sensor->get_objname());
        break;
      case mjSENS_ACTUATORPOS:
        elem = InsertEnd(section, "actuatorpos");
        WriteAttrTxt(elem, "actuator", sensor->get_objname());
        break;
      case mjSENS_ACTUATORVEL:
        elem = InsertEnd(section, "actuatorvel");
        WriteAttrTxt(elem, "actuator", sensor->get_objname());
        break;
      case mjSENS_ACTUATORFRC:
        elem = InsertEnd(section, "actuatorfrc");
        WriteAttrTxt(elem, "actuator", sensor->get_objname());
        break;
      case mjSENS_JOINTACTFRC:
        elem = InsertEnd(section, "jointactuatorfrc");
        WriteAttrTxt(elem, "joint", sensor->get_objname());
        break;
      case mjSENS_TENDONACTFRC:
        elem = InsertEnd(section, "tendonactuatorfrc");
        WriteAttrTxt(elem, "tendon", sensor->get_objname());
        break;

      // sensors related to ball joints
      case mjSENS_BALLQUAT:
        elem = InsertEnd(section, "ballquat");
        WriteAttrTxt(elem, "joint", sensor->get_objname());
        break;
      case mjSENS_BALLANGVEL:
        elem = InsertEnd(section, "ballangvel");
        WriteAttrTxt(elem, "joint", sensor->get_objname());
        break;

      // joint and tendon limit sensors
      case mjSENS_JOINTLIMITPOS:
        elem = InsertEnd(section, "jointlimitpos");
        WriteAttrTxt(elem, "joint", sensor->get_objname());
        break;
      case mjSENS_JOINTLIMITVEL:
        elem = InsertEnd(section, "jointlimitvel");
        WriteAttrTxt(elem, "joint", sensor->get_objname());
        break;
      case mjSENS_JOINTLIMITFRC:
        elem = InsertEnd(section, "jointlimitfrc");
        WriteAttrTxt(elem, "joint", sensor->get_objname());
        break;
      case mjSENS_TENDONLIMITPOS:
        elem = InsertEnd(section, "tendonlimitpos");
        WriteAttrTxt(elem, "tendon", sensor->get_objname());
        break;
      case mjSENS_TENDONLIMITVEL:
        elem = InsertEnd(section, "tendonlimitvel");
        WriteAttrTxt(elem, "tendon", sensor->get_objname());
        break;
      case mjSENS_TENDONLIMITFRC:
        elem = InsertEnd(section, "tendonlimitfrc");
        WriteAttrTxt(elem, "tendon", sensor->get_objname());
        break;

      // sensors attached to an object with spatial frame: (x)body, geom, site, camera
      case mjSENS_FRAMEPOS:
        elem = InsertEnd(section, "framepos");
        WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        if (sensor->reftype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "reftype", mju_type2Str(sensor->reftype));
          WriteAttrTxt(elem, "refname", sensor->get_refname());
        }
        break;
      case mjSENS_FRAMEQUAT:
        elem = InsertEnd(section, "framequat");
        WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        if (sensor->reftype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "reftype", mju_type2Str(sensor->reftype));
          WriteAttrTxt(elem, "refname", sensor->get_refname());
        }
        break;
      case mjSENS_FRAMEXAXIS:
        elem = InsertEnd(section, "framexaxis");
        WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        if (sensor->reftype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "reftype", mju_type2Str(sensor->reftype));
          WriteAttrTxt(elem, "refname", sensor->get_refname());
        }
        break;
      case mjSENS_FRAMEYAXIS:
        elem = InsertEnd(section, "frameyaxis");
        WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        if (sensor->reftype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "reftype", mju_type2Str(sensor->reftype));
          WriteAttrTxt(elem, "refname", sensor->get_refname());
        }
        break;
      case mjSENS_FRAMEZAXIS:
        elem = InsertEnd(section, "framezaxis");
        WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        if (sensor->reftype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "reftype", mju_type2Str(sensor->reftype));
          WriteAttrTxt(elem, "refname", sensor->get_refname());
        }
        break;
      case mjSENS_FRAMELINVEL:
        elem = InsertEnd(section, "framelinvel");
        WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        if (sensor->reftype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "reftype", mju_type2Str(sensor->reftype));
          WriteAttrTxt(elem, "refname", sensor->get_refname());
        }
        break;
      case mjSENS_FRAMEANGVEL:
        elem = InsertEnd(section, "frameangvel");
        WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        if (sensor->reftype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "reftype", mju_type2Str(sensor->reftype));
          WriteAttrTxt(elem, "refname", sensor->get_refname());
        }
        break;
      case mjSENS_FRAMELINACC:
        elem = InsertEnd(section, "framelinacc");
        WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        if (sensor->reftype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "reftype", mju_type2Str(sensor->reftype));
          WriteAttrTxt(elem, "refname", sensor->get_refname());
        }
        break;
      case mjSENS_FRAMEANGACC:
        elem = InsertEnd(section, "frameangacc");
        WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        if (sensor->reftype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "reftype", mju_type2Str(sensor->reftype));
          WriteAttrTxt(elem, "refname", sensor->get_refname());
        }
        break;

      // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
      case mjSENS_SUBTREECOM:
        elem = InsertEnd(section, "subtreecom");
        WriteAttrTxt(elem, "body", sensor->get_objname());
        break;
      case mjSENS_SUBTREELINVEL:
        elem = InsertEnd(section, "subtreelinvel");
        WriteAttrTxt(elem, "body", sensor->get_objname());
        break;
      case mjSENS_SUBTREEANGMOM:
        elem = InsertEnd(section, "subtreeangmom");
        WriteAttrTxt(elem, "body", sensor->get_objname());
        break;
      case mjSENS_GEOMDIST:
        elem = InsertEnd(section, "distance");
        WriteAttrTxt(elem, sensor->objtype == mjOBJ_BODY ? "body1" : "geom1", sensor->get_objname());
        WriteAttrTxt(elem, sensor->reftype == mjOBJ_BODY ? "body2" : "geom2", sensor->get_refname());
        break;
      case mjSENS_GEOMNORMAL:
        elem = InsertEnd(section, "normal");
        WriteAttrTxt(elem, sensor->objtype == mjOBJ_BODY ? "body1" : "geom1", sensor->get_objname());
        WriteAttrTxt(elem, sensor->reftype == mjOBJ_BODY ? "body2" : "geom2", sensor->get_refname());
        break;
      case mjSENS_GEOMFROMTO:
        elem = InsertEnd(section, "fromto");
        WriteAttrTxt(elem, sensor->objtype == mjOBJ_BODY ? "body1" : "geom1", sensor->get_objname());
        WriteAttrTxt(elem, sensor->reftype == mjOBJ_BODY ? "body2" : "geom2", sensor->get_refname());
        break;

      // global sensors
      case mjSENS_E_POTENTIAL:
        elem = InsertEnd(section, "potential");
        break;
      case mjSENS_E_KINETIC:
        elem = InsertEnd(section, "kinetic");
        break;
      case mjSENS_CLOCK:
        elem = InsertEnd(section, "clock");
        break;


      // plugin-controlled sensor
      case mjSENS_PLUGIN:
        elem = InsertEnd(section, "plugin");
        if (sensor->objtype != mjOBJ_UNKNOWN) {
          WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
          WriteAttrTxt(elem, "objname", sensor->get_objname());
        }
        OnePlugin(elem, &sensor->plugin);
        break;

      // user-defined sensor
      case mjSENS_USER:
        elem = InsertEnd(section, "user");
        if (mju_type2Str(sensor->objtype)) {
          WriteAttrTxt(elem, "objtype", mju_type2Str(sensor->objtype));
        }
        WriteAttrTxt(elem, "objname", sensor->get_objname());
        WriteAttrInt(elem, "dim", sensor->dim);
        WriteAttrKey(elem, "needstage", stage_map, stage_sz, (int)sensor->needstage);
        WriteAttrKey(elem, "datatype", datatype_map, datatype_sz, (int)sensor->datatype);
        break;

      default:
        mju_error("Unknown sensor type in XML write");
    }

    // write name, noise, userdata
    WriteAttrTxt(elem, "name", sensor->name);
    WriteAttr(elem, "cutoff", 1, &sensor->cutoff, &zero);
    if (sensor->type != mjSENS_PLUGIN) {
      WriteAttr(elem, "noise", 1, &sensor->noise, &zero);
    }
    WriteVector(elem, "user", sensor->get_userdata());
  }

  // remove section if empty
  if (!section->FirstChildElement()) {
    root->DeleteChild(section);
  }
}



// keyframe section
void mjXWriter::Keyframe(XMLElement* root) {
  // create section
  XMLElement* section = InsertEnd(root, "keyframe");

  if (!model->key_pending_.empty()) {
    throw mjXError(0, "Model has pending keyframes. It must be (re)compiled before writing XML.");
  }

  // write all keyframes
  for (int i=0; i < model->nkey; i++) {
    XMLElement* elem = InsertEnd(section, "key");
    bool change = false;

    mjCKey* key = model->Keys()[i];

    // check name and write
    if (!key->name.empty()) {
      WriteAttrTxt(elem, "name", key->name);
      change = true;
    }

    // check time and write
    if (key->time != 0) {
      WriteAttr(elem, "time", 1, &key->time);
      change = true;
    }

    // check qpos and write
    for (int j=0; j < model->nq; j++) {
      if (key->qpos_[j] != model->qpos0[j]) {
        WriteAttr(elem, "qpos", model->nq, key->qpos_.data());
        change = true;
        break;
      }
    }

    // check qvel and write
    for (int j=0; j < model->nv; j++) {
      if (key->qvel_[j] != 0) {
        WriteAttr(elem, "qvel", model->nv, key->qvel_.data());
        change = true;
        break;
      }
    }

    // check act and write
    for (int j=0; j < model->na; j++) {
      if (key->act_[j] != 0) {
        WriteAttr(elem, "act", model->na, key->act_.data());
        change = true;
        break;
      }
    }

    // check mpos and write
    if (model->nmocap) {
      for (int j=0; j < model->nbody; j++) {
        if (model->Bodies()[j]->mocap) {
          mjCBody* body = model->Bodies()[j];
          int id = body->mocapid;
          if (body->pos[0] != key->mpos_[3*id] ||
              body->pos[1] != key->mpos_[3*id+1] ||
              body->pos[2] != key->mpos_[3*id+2]) {
            WriteAttr(elem, "mpos", 3*model->nmocap, key->mpos_.data());
            change = true;
            break;
          }
        }
      }
    }

    // check mquat and write
    if (model->nmocap) {
      for (int j=0; j < model->nbody; j++) {
        if (model->Bodies()[j]->mocap) {
          mjCBody* body = model->Bodies()[j];
          int id = body->mocapid;
          if (body->quat[0] != key->mquat_[4*id] ||
              body->quat[1] != key->mquat_[4*id+1] ||
              body->quat[2] != key->mquat_[4*id+2] ||
              body->quat[3] != key->mquat_[4*id+3]) {
            WriteAttr(elem, "mquat", 4*model->nmocap, key->mquat_.data());
            change = true;
            break;
          }
        }
      }
    }

    // check ctrl and write
    for (int j=0; j < model->nu; j++) {
      if (key->ctrl_[j] != 0) {
        WriteAttr(elem, "ctrl", model->nu, key->ctrl_.data());
        change = true;
        break;
      }
    }

    // remove elem if empty
    if (!change) {
      section->DeleteChild(elem);
    }
  }

  // remove section if empty
  if (!section->FirstChildElement()) {
    root->DeleteChild(section);
  }
}
