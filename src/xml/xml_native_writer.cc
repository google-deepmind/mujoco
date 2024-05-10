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

#include <cstddef>
#include <cstdio>
#include <string>
#include <unordered_set>
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include "engine/engine_io.h"
#include "engine/engine_plugin.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_api.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_util.h"
#include "xml/xml_base.h"
#include "xml/xml_util.h"
#include "tinyxml2.h"

namespace {

using std::size_t;
using std::string;
using tinyxml2::XMLComment;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

}  // namespace



// custom XML indentation: 2 spaces rather than the default 4
class mj_XMLPrinter : public tinyxml2::XMLPrinter {
  using tinyxml2::XMLPrinter::XMLPrinter;

 public:
    void PrintSpace( int depth ) {
      for (int i=0; i<depth; ++i) {
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
  return string(stream.CStr());
}


// insert end child with given name, return child
XMLElement* mjXWriter::InsertEnd(XMLElement* parent, const char* name) {
  XMLElement* result = parent->GetDocument()->NewElement(name);
  parent->InsertEndChild(result);

  return result;
}


//---------------------------------- class mjXWriter: one-element writers --------------------------

// write flex
void mjXWriter::OneFlex(XMLElement* elem, mjCFlex* pflex) {
  string text;
  mjCFlex defflex;

  // common attributes
  WriteAttrTxt(elem, "name", pflex->name);
  WriteAttr(elem, "radius", 1, &pflex->radius, &defflex.radius);
  if (pflex->get_material() != defflex.get_material()) {
    WriteAttrTxt(elem, "material", pflex->get_material());
  }
  WriteAttr(elem, "rgba", 4, pflex->rgba, defflex.rgba);
  WriteAttrKey(elem, "flatskin", bool_map, 2, pflex->flatskin, defflex.flatskin);
  WriteAttrInt(elem, "dim", pflex->dim, defflex.dim);
  WriteAttrInt(elem, "group", pflex->group, defflex.group);

  // data vectors
  if (!pflex->get_vertbody().empty()) {
    Vector2String(text, pflex->get_vertbody());
    WriteAttrTxt(elem, "body", text);
  }
  if (!pflex->get_vert().empty()) {
    Vector2String(text, pflex->get_vert());
    WriteAttrTxt(elem, "vertex", text);
  }
  if (!pflex->get_elem().empty()) {
    Vector2String(text, pflex->get_elem());
    WriteAttrTxt(elem, "element", text);
  }
  if (!pflex->get_texcoord().empty()) {
    Vector2String(text, pflex->get_texcoord());
    WriteAttrTxt(elem, "texcoord", text);
  }

  // contact subelement
  XMLElement* cont = InsertEnd(elem, "contact");
  WriteAttrInt(cont, "contype", pflex->contype, defflex.contype);
  WriteAttrInt(cont, "conaffinity", pflex->conaffinity, defflex.conaffinity);
  WriteAttrInt(cont, "condim", pflex->condim, defflex.condim);
  WriteAttrInt(cont, "priority", pflex->priority, defflex.priority);
  WriteAttr(cont, "friction", 3, pflex->friction, defflex.friction);
  WriteAttr(cont, "solmix", 1, &pflex->solmix, &defflex.solmix);
  WriteAttr(cont, "solref", mjNREF, pflex->solref, defflex.solref);
  WriteAttr(cont, "solimp", mjNIMP, pflex->solimp, defflex.solimp);
  WriteAttr(cont, "margin", 1, &pflex->margin, &defflex.margin);
  WriteAttr(cont, "gap", 1, &pflex->gap, &defflex.gap);
  WriteAttrKey(cont, "internal", bool_map, 2, pflex->internal, defflex.internal);
  WriteAttrKey(cont, "selfcollide", flexself_map, 5, pflex->selfcollide, defflex.selfcollide);
  WriteAttrInt(cont, "activelayers", pflex->activelayers, defflex.activelayers);

  // remove contact is no attributes
  if (!cont->FirstAttribute()) {
    elem->DeleteChild(cont);
  }

  // edge subelement
  XMLElement* edge = InsertEnd(elem, "edge");
  WriteAttr(edge, "stiffness", 1, &pflex->edgestiffness, &defflex.edgestiffness);
  WriteAttr(edge, "damping", 1, &pflex->edgedamping, &defflex.edgedamping);

  // remove edge if no attributes
  if (!edge->FirstAttribute()) {
    elem->DeleteChild(edge);
  }
}



// write mesh
void mjXWriter::OneMesh(XMLElement* elem, mjCMesh* pmesh, mjCDef* def) {
  string text;

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pmesh->name);
    WriteAttrTxt(elem, "class", pmesh->classname);
    WriteAttrTxt(elem, "content_type", pmesh->get_content_type());
    WriteAttrTxt(elem, "file", pmesh->get_file());

    // write vertex data
    if (!pmesh->get_uservert().empty()) {
      Vector2String(text, pmesh->get_uservert());
      WriteAttrTxt(elem, "vertex", text);
    }

    // write normal data
    if (!pmesh->get_usernormal().empty()) {
      Vector2String(text, pmesh->get_usernormal());
      WriteAttrTxt(elem, "normal", text);
    }

    // write texcoord data
    if (!pmesh->get_usertexcoord().empty()) {
      Vector2String(text, pmesh->get_usertexcoord());
      WriteAttrTxt(elem, "texcoord", text);
    }

    // write face data
    if (!pmesh->get_userface().empty()) {
      Vector2String(text, pmesh->get_userface());
      WriteAttrTxt(elem, "face", text);
    }
  }

  // defaults and regular
  WriteAttr(elem, "refpos", 3, pmesh->refpos, def->mesh.refpos);
  WriteAttr(elem, "refquat", 4, pmesh->refquat, def->mesh.refquat);
  WriteAttr(elem, "scale", 3, pmesh->scale, def->mesh.scale);
  WriteAttrKey(elem, "smoothnormal", bool_map, 2, pmesh->get_smoothnormal(),
               def->mesh.get_smoothnormal());
}



// write skin
void mjXWriter::OneSkin(XMLElement* elem, mjCSkin* pskin) {
  string text;
  mjCDef mydef;
  float zero = 0;

  // write attributes
  WriteAttrTxt(elem, "name", pskin->name);
  WriteAttrTxt(elem, "file", pskin->get_file());
  WriteAttrTxt(elem, "material", pskin->get_material());
  WriteAttrInt(elem, "group", pskin->group, 0);
  WriteAttr(elem, "rgba", 4, pskin->rgba, mydef.geom.rgba);
  WriteAttr(elem, "inflate", 1, &pskin->inflate, &zero);

  // write data if no file
  if (pskin->get_file().empty()) {
    // mesh vert
    Vector2String(text, pskin->get_vert());
    WriteAttrTxt(elem, "vertex", text);

    // mesh texcoord
    if (!pskin->get_texcoord().empty()) {
      Vector2String(text, pskin->get_texcoord());
      WriteAttrTxt(elem, "texcoord", text);
    }

    // mesh face
    Vector2String(text, pskin->get_face());
    WriteAttrTxt(elem, "face", text);

    // bones
    for (size_t i=0; i<pskin->get_bodyname().size(); i++) {
      // make bone
      XMLElement* bone = InsertEnd(elem, "bone");

      // write attributes
      WriteAttrTxt(bone, "body", pskin->get_bodyname()[i]);
      WriteAttr(bone, "bindpos", 3, pskin->get_bindpos().data()+3*i);
      WriteAttr(bone, "bindquat", 4, pskin->get_bindquat().data()+4*i);

      // write vertid
      Vector2String(text, pskin->get_vertid()[i]);
      WriteAttrTxt(bone, "vertid", text);

      // write vertweight
      Vector2String(text, pskin->get_vertweight()[i]);
      WriteAttrTxt(bone, "vertweight", text);
    }
  }
}



// write material
void mjXWriter::OneMaterial(XMLElement* elem, mjCMaterial* pmat, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pmat->name);
    WriteAttrTxt(elem, "class", pmat->classname);
  }

  // defaults and regular
  if (pmat->texture != def->material.texture) {
    WriteAttrTxt(elem, "texture", pmat->get_texture());
  }
  WriteAttrKey(elem, "texuniform", bool_map, 2, pmat->texuniform, def->material.texuniform);
  WriteAttr(elem, "texrepeat", 2, pmat->texrepeat, def->material.texrepeat);
  WriteAttr(elem, "emission", 1, &pmat->emission, &def->material.emission);
  WriteAttr(elem, "specular", 1, &pmat->specular, &def->material.specular);
  WriteAttr(elem, "shininess", 1, &pmat->shininess, &def->material.shininess);
  WriteAttr(elem, "reflectance", 1, &pmat->reflectance, &def->material.reflectance);
  WriteAttr(elem, "metallic", 1, &pmat->metallic, &def->material.metallic);
  WriteAttr(elem, "roughness", 1, &pmat->roughness, &def->material.roughness);
  WriteAttr(elem, "rgba", 4, pmat->rgba, def->material.rgba);
}



// write joint
void mjXWriter::OneJoint(XMLElement* elem, mjCJoint* pjoint, mjCDef* def) {
  double zero = 0;

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pjoint->name);
    WriteAttrTxt(elem, "class", pjoint->classname);
    if (pjoint->type != mjJNT_FREE) {
      WriteAttr(elem, "pos", 3, pjoint->pos);
    }
    if (pjoint->type != mjJNT_FREE && pjoint->type != mjJNT_BALL) {
      WriteAttr(elem, "axis", 3, pjoint->axis);
    }
  }

  // defaults and regular
  if (pjoint->type != def->joint.type) {
    WriteAttrTxt(elem, "type", FindValue(joint_map, joint_sz, pjoint->type));
  }
  WriteAttrInt(elem, "group", pjoint->group, def->joint.group);
  WriteAttr(elem, "ref", 1, &pjoint->ref, &zero);
  WriteAttr(elem, "springref", 1, &pjoint->springref, &zero);
  WriteAttr(elem, "solreflimit", mjNREF, pjoint->solref_limit, def->joint.solref_limit, true);
  WriteAttr(elem, "solimplimit", mjNIMP, pjoint->solimp_limit, def->joint.solimp_limit, true);
  WriteAttr(elem, "solreffriction", mjNREF, pjoint->solref_friction, def->joint.solref_friction,
            true);
  WriteAttr(elem, "solimpfriction", mjNIMP, pjoint->solimp_friction, def->joint.solimp_friction,
            true);
  WriteAttr(elem, "stiffness", 1, &pjoint->stiffness, &def->joint.stiffness);
  WriteAttrKey(elem, "limited", TFAuto_map, 3, pjoint->limited, def->joint.limited);
  WriteAttr(elem, "range", 2, pjoint->range, def->joint.range);
  WriteAttrKey(elem, "actuatorfrclimited", TFAuto_map, 3, pjoint->actfrclimited,
               def->joint.actfrclimited);
  WriteAttrKey(elem, "actuatorgravcomp", bool_map, 2, pjoint->actgravcomp, def->joint.actgravcomp);
  WriteAttr(elem, "actuatorfrcrange", 2, pjoint->actfrcrange, def->joint.actfrcrange);
  WriteAttr(elem, "margin", 1, &pjoint->margin, &def->joint.margin);
  WriteAttr(elem, "armature", 1, &pjoint->armature, &def->joint.armature);
  WriteAttr(elem, "damping", 1, &pjoint->damping, &def->joint.damping);
  WriteAttr(elem, "frictionloss", 1, &pjoint->frictionloss, &def->joint.frictionloss);

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pjoint->get_userdata());
  } else {
    WriteVector(elem, "user", pjoint->get_userdata(), def->joint.get_userdata());
  }
}



// write geom
void mjXWriter::OneGeom(XMLElement* elem, mjCGeom* pgeom, mjCDef* def) {
  double unitq[4] = {1, 0, 0, 0};
  double mass = 0;

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pgeom->name);
    WriteAttrTxt(elem, "class", pgeom->classname);
    if (mjGEOMINFO[pgeom->type]) {
      WriteAttr(elem, "size", mjGEOMINFO[pgeom->type], pgeom->size, def->geom.size);
    }
    if (mjuu_defined(pgeom->mass)) {
      mass = pgeom->GetVolume() * def->geom.density;
    }

    // mesh geom
    if (pgeom->type==mjGEOM_MESH || pgeom->type==mjGEOM_SDF) {
      mjCMesh* pmesh = pgeom->mesh;

      // write pos/quat if there is a difference
      if (!SameVector(pgeom->pos, pmesh->GetPosPtr(pgeom->typeinertia), 3) ||
          !SameVector(pgeom->quat, pmesh->GetQuatPtr(pgeom->typeinertia), 4)) {
        // recover geom pos/quat before mesh frame transformation
        double p[3], q[4];
        mjuu_copyvec(p, pgeom->pos, 3);
        mjuu_copyvec(q, pgeom->quat, 4);
        mjuu_frameaccuminv(p, q, pmesh->GetPosPtr(pgeom->typeinertia),
                           pmesh->GetQuatPtr(pgeom->typeinertia));

        // write
        WriteAttr(elem, "pos", 3, p, unitq+1);
        WriteAttr(elem, "quat", 4, q, unitq);
      }
    }

    // non-mesh geom
    else {
      WriteAttr(elem, "pos", 3, pgeom->pos, unitq+1);
      WriteAttr(elem, "quat", 4, pgeom->quat, unitq);
    }
  } else {
    WriteAttr(elem, "size", 3, pgeom->size, def->geom.size);
  }

  // defaults and regular
  WriteAttrKey(elem, "type", geom_map, mjNGEOMTYPES, pgeom->type, def->geom.type);
  WriteAttrInt(elem, "contype", pgeom->contype, def->geom.contype);
  WriteAttrInt(elem, "conaffinity", pgeom->conaffinity, def->geom.conaffinity);
  WriteAttrInt(elem, "condim", pgeom->condim, def->geom.condim);
  WriteAttrInt(elem, "group", pgeom->group, def->geom.group);
  WriteAttrInt(elem, "priority", pgeom->priority, def->geom.priority);
  WriteAttr(elem, "friction", 3, pgeom->friction, def->geom.friction, true);
  WriteAttr(elem, "solmix", 1, &pgeom->solmix, &def->geom.solmix);
  WriteAttr(elem, "solref", mjNREF, pgeom->solref, def->geom.solref, true);
  WriteAttr(elem, "solimp", mjNIMP, pgeom->solimp, def->geom.solimp, true);
  WriteAttr(elem, "margin", 1, &pgeom->margin, &def->geom.margin);
  WriteAttr(elem, "gap", 1, &pgeom->gap, &def->geom.gap);
  WriteAttr(elem, "gap", 1, &pgeom->gap, &def->geom.gap);
  WriteAttrKey(elem, "fluidshape", fluid_map, 2, pgeom->fluid_ellipsoid, def->geom.fluid_ellipsoid);
  WriteAttr(elem, "fluidcoef", 5, pgeom->fluid_coefs, def->geom.fluid_coefs);
  WriteAttrKey(elem, "shellinertia", meshtype_map, 2, pgeom->typeinertia, def->geom.typeinertia);
  if (mjuu_defined(pgeom->mass)) {
    WriteAttr(elem, "mass", 1, &pgeom->mass_, &mass);
  } else {
    WriteAttr(elem, "density", 1, &pgeom->density, &def->geom.density);
  }
  if (pgeom->get_material() != def->geom.get_material()) {
    WriteAttrTxt(elem, "material", pgeom->get_material());
  }
  WriteAttr(elem, "rgba", 4, pgeom->rgba, def->geom.rgba);

  // hfield and mesh attributes
  if (pgeom->type==mjGEOM_HFIELD) {
    WriteAttrTxt(elem, "hfield", pgeom->get_hfieldname());
  }
  if (pgeom->type==mjGEOM_MESH || pgeom->type==mjGEOM_SDF) {
    WriteAttrTxt(elem, "mesh", pgeom->get_meshname());
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pgeom->get_userdata());
  } else {
    WriteVector(elem, "user", pgeom->get_userdata(), def->geom.get_userdata());
  }

  // write plugin
  if (pgeom->plugin.active) {
    OnePlugin(InsertEnd(elem, "plugin"), &pgeom->plugin);
  }
}



// write site
void mjXWriter::OneSite(XMLElement* elem, mjCSite* psite, mjCDef* def) {
  double unitq[4] = {1, 0, 0, 0};

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", psite->name);
    WriteAttrTxt(elem, "class", psite->classname);
    WriteAttr(elem, "pos", 3, psite->pos);
    WriteAttr(elem, "quat", 4, psite->quat, unitq);
    if (mjGEOMINFO[psite->type]) {
      WriteAttr(elem, "size", mjGEOMINFO[psite->type], psite->size, def->site.size);
    }
  } else {
    WriteAttr(elem, "size", 3, psite->size, def->site.size);
  }

  // defaults and regular
  WriteAttrInt(elem, "group", psite->group, def->site.group);
  WriteAttrKey(elem, "type", geom_map, mjNGEOMTYPES, psite->type, def->site.type);
  if (psite->get_material() != def->site.get_material()) {
    WriteAttrTxt(elem, "material", psite->get_material());
  }
  WriteAttr(elem, "rgba", 4, psite->rgba, def->site.rgba);

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", psite->get_userdata());
  } else {
    WriteVector(elem, "user", psite->get_userdata(), def->site.get_userdata());
  }
}



// write camera
void mjXWriter::OneCamera(XMLElement* elem, mjCCamera* pcam, mjCDef* def) {
  double unitq[4] = {1, 0, 0, 0};

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pcam->name);
    WriteAttrTxt(elem, "class", pcam->classname);
    WriteAttrTxt(elem, "target", pcam->get_targetbody());
    WriteAttr(elem, "pos", 3, pcam->pos);
    WriteAttr(elem, "quat", 4, pcam->quat, unitq);
  }

  // defaults and regular
  WriteAttr(elem, "ipd", 1, &pcam->ipd, &def->camera.ipd);
  WriteAttrKey(elem, "mode", camlight_map, camlight_sz, pcam->mode, def->camera.mode);
  WriteAttr(elem, "resolution", 2, pcam->resolution, def->camera.resolution);

  // resolution if positive
  WriteAttr(elem, "resolution", 2, pcam->resolution, def->camera.resolution);

  // camera intrinsics if specified
  if (pcam->sensor_size[0]>0 && pcam->sensor_size[1]>0) {
    WriteAttr(elem, "sensorsize", 2, pcam->sensor_size);
    WriteAttr(elem, "focal", 2, pcam->focal_length, def->camera.focal_length);
    WriteAttr(elem, "focalpixel", 2, pcam->focal_pixel, def->camera.focal_pixel);
    WriteAttr(elem, "principal", 2, pcam->principal_length, def->camera.principal_length);
    WriteAttr(elem, "principalpixel", 2, pcam->principal_pixel, def->camera.principal_pixel);
  } else {
    WriteAttr(elem, "fovy", 1, &pcam->fovy, &def->camera.fovy);
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pcam->get_userdata());
  } else {
    WriteVector(elem, "user", pcam->get_userdata(), def->camera.get_userdata());
  }
}



// write light
void mjXWriter::OneLight(XMLElement* elem, mjCLight* plight, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", plight->name);
    WriteAttrTxt(elem, "class", plight->classname);
    WriteAttrTxt(elem, "target", plight->get_targetbody());
    WriteAttr(elem, "pos", 3, plight->pos);
    WriteAttr(elem, "dir", 3, plight->dir);
  }

  // defaults and regular
  WriteAttr(elem, "bulbradius", 1, &plight->bulbradius, &def->light.bulbradius);
  WriteAttrKey(elem, "directional", bool_map, 2, plight->directional, def->light.directional);
  WriteAttrKey(elem, "castshadow", bool_map, 2, plight->castshadow, def->light.castshadow);
  WriteAttrKey(elem, "active", bool_map, 2, plight->active, def->light.active);
  WriteAttr(elem, "attenuation", 3, plight->attenuation, def->light.attenuation);
  WriteAttr(elem, "cutoff", 1, &plight->cutoff, &def->light.cutoff);
  WriteAttr(elem, "exponent", 1, &plight->exponent, &def->light.exponent);
  WriteAttr(elem, "ambient", 3, plight->ambient, def->light.ambient);
  WriteAttr(elem, "diffuse", 3, plight->diffuse, def->light.diffuse);
  WriteAttr(elem, "specular", 3, plight->specular, def->light.specular);
  WriteAttrKey(elem, "mode", camlight_map, camlight_sz, plight->mode, def->light.mode);
}



// write pair
void mjXWriter::OnePair(XMLElement* elem, mjCPair* ppair, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "class", ppair->classname);
    WriteAttrTxt(elem, "geom1", ppair->get_geomname1());
    WriteAttrTxt(elem, "geom2", ppair->get_geomname2());
  }

  // defaults and regular
  WriteAttrTxt(elem, "name", ppair->name);
  WriteAttrInt(elem, "condim", ppair->condim, def->pair.spec.condim);
  WriteAttr(elem, "margin", 1, &ppair->margin, &def->pair.spec.margin);
  WriteAttr(elem, "gap", 1, &ppair->gap, &def->pair.spec.gap);
  WriteAttr(elem, "solref", mjNREF, ppair->solref, def->pair.spec.solref, true);
  WriteAttr(elem, "solreffriction", mjNREF, ppair->solreffriction, def->pair.spec.solreffriction,
            true);
  WriteAttr(elem, "solimp", mjNIMP, ppair->solimp, def->pair.spec.solimp, true);
  WriteAttr(elem, "friction", 5, ppair->friction, def->pair.spec.friction);  // all 5 values
}



// write equality
void mjXWriter::OneEquality(XMLElement* elem, mjCEquality* peq, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", peq->name);
    WriteAttrTxt(elem, "class", peq->classname);

    switch (peq->type) {
    case mjEQ_CONNECT:
      WriteAttrTxt(elem, "body1", mjs_getString(peq->name1));
      WriteAttrTxt(elem, "body2", mjs_getString(peq->name2));
      WriteAttr(elem, "anchor", 3, peq->data);
      break;

    case mjEQ_WELD:
      WriteAttrTxt(elem, "body1", mjs_getString(peq->name1));
      WriteAttrTxt(elem, "body2", mjs_getString(peq->name2));
      WriteAttr(elem, "anchor", 3, peq->data);
      WriteAttr(elem, "torquescale", 1, peq->data+10);
      WriteAttr(elem, "relpose", 7, peq->data+3);
      break;

    case mjEQ_JOINT:
      WriteAttrTxt(elem, "joint1", mjs_getString(peq->name1));
      WriteAttrTxt(elem, "joint2", mjs_getString(peq->name2));
      WriteAttr(elem, "polycoef", 5, peq->data);
      break;

    case mjEQ_TENDON:
      WriteAttrTxt(elem, "tendon1", mjs_getString(peq->name1));
      WriteAttrTxt(elem, "tendon2", mjs_getString(peq->name2));
      WriteAttr(elem, "polycoef", 5, peq->data);
      break;

    case mjEQ_FLEX:
      WriteAttrTxt(elem, "flex", mjs_getString(peq->name1));
      break;

    default:
      mju_error("mjXWriter: unknown equality type.");
    }
  }

  // defaults and regular
  WriteAttrKey(elem, "active", bool_map, 2, peq->active, def->equality.active);
  WriteAttr(elem, "solref", mjNREF, peq->solref, def->equality.solref, true);
  WriteAttr(elem, "solimp", mjNIMP, peq->solimp, def->equality.solimp, true);
}



// write tendon
void mjXWriter::OneTendon(XMLElement* elem, mjCTendon* pten, mjCDef* def) {
  bool fixed = (pten->GetWrap(0) && pten->GetWrap(0)->type==mjWRAP_JOINT);

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pten->name);
    WriteAttrTxt(elem, "class", pten->classname);
  }

  // defaults and regular
  WriteAttrInt(elem, "group", pten->group, def->tendon.group);
  WriteAttr(elem, "solreflimit", mjNREF, pten->solref_limit, def->tendon.solref_limit, true);
  WriteAttr(elem, "solimplimit", mjNIMP, pten->solimp_limit, def->tendon.solimp_limit, true);
  WriteAttr(elem, "solreffriction", mjNREF, pten->solref_friction, def->tendon.solref_friction,
            true);
  WriteAttr(elem, "solimpfriction", mjNIMP, pten->solimp_friction, def->tendon.solimp_friction,
            true);
  WriteAttrKey(elem, "limited", TFAuto_map, 3, pten->limited, def->tendon.limited);
  WriteAttr(elem, "range", 2, pten->range, def->tendon.range);
  WriteAttr(elem, "margin", 1, &pten->margin, &def->tendon.margin);
  WriteAttr(elem, "stiffness", 1, &pten->stiffness, &def->tendon.stiffness);
  WriteAttr(elem, "damping", 1, &pten->damping, &def->tendon.damping);
  WriteAttr(elem, "frictionloss", 1, &pten->frictionloss, &def->tendon.frictionloss);
  if (pten->springlength[0] != pten->springlength[1] ||
      def->tendon.springlength[0] != def->tendon.springlength[1]) {
    WriteAttr(elem, "springlength", 2, pten->springlength, def->tendon.springlength);
  } else {
    WriteAttr(elem, "springlength", 1, pten->springlength, def->tendon.springlength);
  }
  // spatial only
  if (!fixed) {
    if (pten->get_material()!=def->tendon.get_material()) {
      WriteAttrTxt(elem, "material", pten->get_material());
    }
    WriteAttr(elem, "width", 1, &pten->width, &def->tendon.width);
    WriteAttr(elem, "rgba", 4, pten->rgba, def->tendon.rgba);
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pten->get_userdata());
  } else {
    WriteVector(elem, "user", pten->get_userdata(), def->tendon.get_userdata());
  }
}



// write actuator
void mjXWriter::OneActuator(XMLElement* elem, mjCActuator* pact, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pact->name);
    WriteAttrTxt(elem, "class", pact->classname);

    // transmission target
    switch (pact->trntype) {
    case mjTRN_JOINT:
      WriteAttrTxt(elem, "joint", pact->get_target());
      break;

    case mjTRN_JOINTINPARENT:
      WriteAttrTxt(elem, "jointinparent", pact->get_target());
      break;

    case mjTRN_TENDON:
      WriteAttrTxt(elem, "tendon", pact->get_target());
      break;

    case mjTRN_SLIDERCRANK:
      WriteAttrTxt(elem, "cranksite", pact->get_target());
      WriteAttrTxt(elem, "slidersite", pact->get_slidersite());
      break;

    case mjTRN_SITE:
      WriteAttrTxt(elem, "site", pact->get_target());
      WriteAttrTxt(elem, "refsite", pact->get_refsite());
      break;

    case mjTRN_BODY:
      WriteAttrTxt(elem, "body", pact->get_target());
      break;

    default:        // SHOULD NOT OCCUR
      break;
    }
  }

  // defaults and regular
  WriteAttrInt(elem, "group", pact->group, def->actuator.group);
  WriteAttrKey(elem, "ctrllimited", TFAuto_map, 3, pact->ctrllimited, def->actuator.ctrllimited);
  WriteAttr(elem, "ctrlrange", 2, pact->ctrlrange, def->actuator.ctrlrange);
  WriteAttrKey(elem, "forcelimited", TFAuto_map, 3, pact->forcelimited, def->actuator.forcelimited);
  WriteAttr(elem, "forcerange", 2, pact->forcerange, def->actuator.forcerange);
  WriteAttrKey(elem, "actlimited", TFAuto_map, 3, pact->actlimited, def->actuator.actlimited);
  WriteAttr(elem, "actrange", 2, pact->actrange, def->actuator.actrange);
  WriteAttr(elem, "lengthrange", 2, pact->lengthrange, def->actuator.lengthrange);
  WriteAttr(elem, "gear", 6, pact->gear, def->actuator.gear);
  WriteAttr(elem, "cranklength", 1, &pact->cranklength, &def->actuator.cranklength);
  WriteAttrKey(elem, "actearly", bool_map, 2, pact->actearly,
               def->actuator.actearly);
  WriteAttrKey(elem, "dyntype", dyn_map, dyn_sz, pact->dyntype, def->actuator.dyntype);
  WriteAttr(elem, "dynprm", mjNDYN, pact->dynprm, def->actuator.dynprm);

  // plugins: write config attributes
  if (pact->plugin.active) {
    OnePlugin(elem, &pact->plugin);
  }

  // non-plugins: write actuator parameters
  else {
    // special handling of actdim which has default value of -1
    if (writingdefaults) {
      WriteAttrInt(elem, "actdim", pact->actdim, def->actuator.actdim);
    } else {
      int default_actdim = pact->dyntype == mjDYN_NONE ? 0 : 1;
      WriteAttrInt(elem, "actdim", pact->actdim, default_actdim);
    }
    WriteAttrKey(elem, "gaintype", gain_map, gain_sz, pact->gaintype, def->actuator.gaintype);
    WriteAttrKey(elem, "biastype", bias_map, bias_sz, pact->biastype, def->actuator.biastype);
    WriteAttr(elem, "gainprm", mjNGAIN, pact->gainprm, def->actuator.gainprm, true);
    WriteAttr(elem, "biasprm", mjNBIAS, pact->biasprm, def->actuator.biasprm, true);
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pact->get_userdata());
  } else {
    WriteVector(elem, "user", pact->get_userdata(), def->actuator.get_userdata());
  }
}



// write plugin
void mjXWriter::OnePlugin(XMLElement* elem, mjsPlugin* plugin) {
  const std::string instance_name = std::string(mjs_getString(plugin->instance_name));
  const std::string plugin_name = std::string(mjs_getString(plugin->name));
  if (!instance_name.empty()) {
    WriteAttrTxt(elem, "instance", instance_name);
  } else {
    WriteAttrTxt(elem, "plugin", plugin_name);
    const mjpPlugin* pplugin = mjp_getPluginAtSlot(
        static_cast<mjCPlugin*>(plugin->instance)->spec.plugin_slot);
    const char* c = &(static_cast<mjCPlugin*>(plugin->instance)->flattened_attributes[0]);
    for (int i = 0; i < pplugin->nattribute; ++i) {
      std::string value(c);
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
void mjXWriter::SetModel(mjSpec* spec) {
  if (spec) {
    model = (mjCModel*)spec->element;
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
  Default(root, model->Defaults()[0]);
  writingdefaults = false;
  Extension(root);
  Custom(root);
  Asset(root);
  Body(InsertEnd(root, "worldbody"), model->GetWorld());
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
  if (!model->convexhull) {
    WriteAttrTxt(section, "convexhull", FindValue(bool_map, 2, model->convexhull));
  }
  WriteAttrTxt(section, "angle", "radian");
  if (!model->get_meshdir().empty()) {
    WriteAttrTxt(section, "meshdir", model->get_meshdir());
  }
  if (!model->get_texturedir().empty()) {
    WriteAttrTxt(section, "texturedir", model->get_texturedir());
  }
  if (!model->usethread) {
    WriteAttrTxt(section, "usethread", "false");
  }
  if (model->exactmeshinertia) {
    WriteAttrTxt(section, "exactmeshinertia", "true");
  }
  if (model->boundmass) {
    WriteAttr(section, "boundmass", 1, &model->boundmass);
  }
  if (model->boundinertia) {
    WriteAttr(section, "boundinertia", 1, &model->boundinertia);
  }
  if (!model->autolimits) {
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
  WriteAttr(section, "mpr_tolerance", 1, &model->option.mpr_tolerance, &opt.mpr_tolerance);
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
  WriteAttrInt(section, "mpr_iterations", model->option.mpr_iterations, opt.mpr_iterations);
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
  WriteAttr(elem,    "fovy",      1,   &vis->global.fovy,       &visdef.global.fovy);
  WriteAttr(elem,    "ipd",       1,   &vis->global.ipd,        &visdef.global.ipd);
  WriteAttr(elem,    "azimuth",   1,   &vis->global.azimuth,    &visdef.global.azimuth);
  WriteAttr(elem,    "elevation", 1,   &vis->global.elevation,  &visdef.global.elevation);
  WriteAttr(elem,    "linewidth", 1,   &vis->global.linewidth,  &visdef.global.linewidth);
  WriteAttr(elem,    "glow",      1,   &vis->global.glow,       &visdef.global.glow);
  WriteAttr(elem,    "realtime",  1,   &vis->global.realtime,   &visdef.global.realtime);
  WriteAttrInt(elem, "offwidth",       vis->global.offwidth,    visdef.global.offwidth);
  WriteAttrInt(elem, "offheight",      vis->global.offheight,   visdef.global.offheight);
  WriteAttrKey(elem, "ellipsoidinertia", bool_map, 2, vis->global.ellipsoidinertia, visdef.global.ellipsoidinertia);
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
  mjCDef* par;
  if (def->parentid>=0) {
    par = model->Defaults()[def->parentid];
  } else {
    par = new mjCDef;
  }

  // create section, write class name
  section = InsertEnd(root, "default");
  WriteAttrTxt(section, "class", def->name);

  // mesh
  elem = InsertEnd(section, "mesh");
  OneMesh(elem, &def->mesh, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // material
  elem = InsertEnd(section, "material");
  OneMaterial(elem, &def->material, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // joint
  elem = InsertEnd(section, "joint");
  OneJoint(elem, &def->joint, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // geom
  elem = InsertEnd(section, "geom");
  OneGeom(elem, &def->geom, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // site
  elem = InsertEnd(section, "site");
  OneSite(elem, &def->site, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // camera
  elem = InsertEnd(section, "camera");
  OneCamera(elem, &def->camera, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // light
  elem = InsertEnd(section, "light");
  OneLight(elem, &def->light, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // pair
  elem = InsertEnd(section, "pair");
  OnePair(elem, &def->pair, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // equality
  elem = InsertEnd(section, "equality");
  OneEquality(elem, &def->equality, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // tendon
  elem = InsertEnd(section, "tendon");
  OneTendon(elem, &def->tendon, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // actuator
  elem = InsertEnd(section, "general");
  OneActuator(elem, &def->actuator, par);
  if (!elem->FirstAttribute()) section->DeleteChild(elem);

  // if top-level class has no members or children, delete it and return
  if (def->parentid<0 && section->NoChildren() && def->childid.empty()) {
    root->DeleteChild(section);
    delete par;
    return;
  }

  // add children recursively
  for (int i=0; i<(int)def->childid.size(); i++) {
    Default(section, model->Defaults()[def->childid[i]]);
  }

  // delete parent defaults if allocated here
  if (def->parentid<0) {
    delete par;
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
    const mjpPlugin* plugin = mjp_getPluginAtSlot(pp->spec.plugin_slot);
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
      std::string value(c);
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
  if (nnum==0 && ntxt==0 && ntup==0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "custom");

  // write all numerics
  for (int i=0; i<nnum; i++) {
    mjCNumeric* ptr = (mjCNumeric*)model->GetObject(mjOBJ_NUMERIC, i);
    elem = InsertEnd(section, "numeric");
    WriteAttrTxt(elem, "name", ptr->name);
    WriteAttrInt(elem, "size", ptr->size);
    WriteAttr(elem, "data", ptr->size, ptr->data_.data());
  }

  // write all texts
  for (int i=0; i<ntxt; i++) {
    mjCText* ptr = (mjCText*)model->GetObject(mjOBJ_TEXT, i);
    elem = InsertEnd(section, "text");
    WriteAttrTxt(elem, "name", ptr->name);
    WriteAttrTxt(elem, "data", ptr->data_.c_str());
  }

  // write all tuples
  for (int i=0; i<ntup; i++) {
    mjCTuple* ptr = (mjCTuple*)model->GetObject(mjOBJ_TUPLE, i);
    elem = InsertEnd(section, "tuple");
    WriteAttrTxt(elem, "name", ptr->name);

    // write objects in tuple
    for (int j=0; j<(int)ptr->objtype_.size(); j++) {
      XMLElement* obj = InsertEnd(elem, "element");
      WriteAttrTxt(obj, "objtype", mju_type2Str((int)ptr->objtype_[j]));
      WriteAttrTxt(obj, "objname", ptr->objname_[j].c_str());
      double oprm = ptr->objprm_[j];
      if (oprm!=0) {
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
  if (ntex==0 && nmat==0 && nmesh==0 && nhfield==0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "asset");

  // write textures
  mjCTexture deftex(0);
  for (int i=0; i<ntex; i++) {
    // create element
    mjCTexture* ptex = (mjCTexture*)model->GetObject(mjOBJ_TEXTURE, i);
    elem = InsertEnd(section, "texture");

    // write common attributes
    WriteAttrKey(elem, "type", texture_map, texture_sz, ptex->type);
    WriteAttrTxt(elem, "name", ptex->name);

    // write builtin
    if (ptex->builtin!=mjBUILTIN_NONE) {
      WriteAttrKey(elem, "builtin", builtin_map, builtin_sz, ptex->builtin);
      WriteAttrKey(elem, "mark", mark_map, mark_sz, ptex->mark, deftex.mark);
      WriteAttr(elem, "rgb1", 3, ptex->rgb1, deftex.rgb1);
      WriteAttr(elem, "rgb2", 3, ptex->rgb2, deftex.rgb2);
      WriteAttr(elem, "markrgb", 3, ptex->markrgb, deftex.markrgb);
      WriteAttr(elem, "random", 1, &ptex->random, &deftex.random);
      WriteAttrInt(elem, "width", ptex->width);
      WriteAttrInt(elem, "height", ptex->height);
    }

    // write textures loaded from files
    else {
      // write single file
      WriteAttrTxt(elem, "content_type", ptex->get_content_type());
      WriteAttrTxt(elem, "file", ptex->get_file());

      // write separate files
      WriteAttrTxt(elem, "fileright", ptex->get_cubefiles()[0]);
      WriteAttrTxt(elem, "fileleft", ptex->get_cubefiles()[1]);
      WriteAttrTxt(elem, "fileup", ptex->get_cubefiles()[2]);
      WriteAttrTxt(elem, "filedown", ptex->get_cubefiles()[3]);
      WriteAttrTxt(elem, "filefront", ptex->get_cubefiles()[4]);
      WriteAttrTxt(elem, "fileback", ptex->get_cubefiles()[5]);
      if (ptex->hflip) {
        WriteAttrKey(elem, "hflip", bool_map, 2, 1);
      }
      if (ptex->vflip) {
        WriteAttrKey(elem, "vflip", bool_map, 2, 1);
      }

      // write grid
      if (ptex->gridsize[0] != 1 || ptex->gridsize[1] != 1) {
        double gsize[2] = { (double)ptex->gridsize[0], (double)ptex->gridsize[1] };
        WriteAttr(elem, "gridsize", 2, gsize);
        WriteAttrTxt(elem, "gridlayout", ptex->gridlayout);
      }
    }
  }

  // write materials
  for (int i=0; i<nmat; i++) {
    // create element and write
    mjCMaterial* pmat = (mjCMaterial*)model->GetObject(mjOBJ_MATERIAL, i);
    elem = InsertEnd(section, "material");
    OneMaterial(elem, pmat, pmat->def);
  }

  // write meshes
  for (int i=0; i<nmesh; i++) {
    // create element and write
    mjCMesh* pmesh = (mjCMesh*)model->GetObject(mjOBJ_MESH, i);
    if (pmesh->plugin.active) {
      elem = InsertEnd(section, "mesh");
      WriteAttrTxt(elem, "name", pmesh->name);
      OnePlugin(InsertEnd(elem, "plugin"), &pmesh->plugin);
    } else{
      elem = InsertEnd(section, "mesh");
      OneMesh(elem, pmesh, pmesh->def);
    }
  }

  // write hfields
  for (int i=0; i<nhfield; i++) {
    // create element
    mjCHField* phf = (mjCHField*)model->GetObject(mjOBJ_HFIELD, i);
    elem = InsertEnd(section, "hfield");

    // write attributes
    WriteAttrTxt(elem, "name", phf->name);
    WriteAttr(elem, "size", 4, phf->size);
    if (!phf->file_.empty()) {
      WriteAttrTxt(elem, "content_type", phf->content_type_);
      WriteAttrTxt(elem, "file", phf->file_);
    } else {
      WriteAttrInt(elem, "nrow", phf->nrow);
      WriteAttrInt(elem, "ncol", phf->ncol);
      if (!phf->get_userdata().empty()) {
        string text;
        Vector2String(text, phf->get_userdata(), phf->ncol);
        WriteAttrTxt(elem, "elevation", text);
      }
    }
  }
}



XMLElement* mjXWriter::OneFrame(XMLElement* elem, mjCFrame* frame) {
  if (!frame) {
    return elem;
  }

  if (frame->name.empty() && frame->classname.empty()) {
    return elem;
  }

  XMLElement* frame_elem = InsertEnd(elem, "frame");
  WriteAttrTxt(frame_elem, "name", frame->name);
  WriteAttrTxt(frame_elem, "childclass", frame->classname);
  return frame_elem;
}



// recursive body and frame writer
void mjXWriter::Body(XMLElement* elem, mjCBody* body) {
  double unitq[4] = {1, 0, 0, 0};

  if (!body) {
    throw mjXError(0, "missing body in XML write");  // SHOULD NOT OCCUR
  }

  // write body attributes and inertial
  else if (body!=model->GetWorld()) {
    WriteAttrTxt(elem, "name", body->name);
    WriteAttrTxt(elem, "childclass", body->classname);

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
    if (body->explicitinertial &&
        model->inertiafromgeom!=mjINERTIAFROMGEOM_TRUE) {
      XMLElement* inertial = InsertEnd(elem, "inertial");
      WriteAttr(inertial, "pos", 3, body->ipos);
      WriteAttr(inertial, "quat", 4, body->iquat, unitq);
      WriteAttr(inertial, "mass", 1, &body->mass);
      WriteAttr(inertial, "diaginertia", 3, body->inertia);
    }
  }

  // write joints
  for (int i=0; i<body->joints.size(); i++) {
    XMLElement* celem = OneFrame(elem, body->joints[i]->frame);
    OneJoint(InsertEnd(celem, "joint"), body->joints[i], body->joints[i]->def);
  }

  // write geoms
  for (int i=0; i<body->geoms.size(); i++) {
    XMLElement* celem = OneFrame(elem, body->geoms[i]->frame);
    OneGeom(InsertEnd(celem, "geom"), body->geoms[i], body->geoms[i]->def);
  }

  // write sites
  for (int i=0; i<body->sites.size(); i++) {
    XMLElement* celem = OneFrame(elem, body->sites[i]->frame);
    OneSite(InsertEnd(celem, "site"), body->sites[i], body->sites[i]->def);
  }

  // write cameras
  for (int i=0; i<body->cameras.size(); i++) {
    XMLElement* celem = OneFrame(elem, body->cameras[i]->frame);
    OneCamera(InsertEnd(celem, "camera"), body->cameras[i], body->cameras[i]->def);
  }

  // write lights
  for (int i=0; i<body->lights.size(); i++) {
    XMLElement* celem = OneFrame(elem, body->lights[i]->frame);
    OneLight(InsertEnd(celem, "light"), body->lights[i], body->lights[i]->def);
  }

  // write plugin
  if (body->plugin.active) {
    OnePlugin(InsertEnd(elem, "plugin"), &body->plugin);
  }

  // write child bodies recursively
  for (int i=0; i<body->bodies.size(); i++) {
    XMLElement* celem = OneFrame(elem, body->bodies[i]->frame);
    Body(InsertEnd(celem, "body"), body->bodies[i]);
  }
}



// collision section
void mjXWriter::Contact(XMLElement* root) {
  XMLElement* elem;

  // get number of pairs of each type
  int npair = model->NumObjects(mjOBJ_PAIR);
  int nexclude = model->NumObjects(mjOBJ_EXCLUDE);

  // skip if section is empty
  if (npair==0 && nexclude==0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "contact");

  // write all geom pairs
  for (int i=0; i<npair; i++) {
    // create element and write
    mjCPair* ppair = (mjCPair*)model->GetObject(mjOBJ_PAIR, i);
    elem = InsertEnd(section, "pair");
    OnePair(elem, ppair, ppair->def);
  }

  // write all exclude pairs
  for (int i=0; i<nexclude; i++) {
    // create element
    mjCBodyPair* pexclude = (mjCBodyPair*)model->GetObject(mjOBJ_EXCLUDE, i);
    elem = InsertEnd(section, "exclude");

    // write attributes
    WriteAttrTxt(elem, "name", pexclude->name);
    WriteAttrTxt(elem, "body1", pexclude->get_bodyname1());
    WriteAttrTxt(elem, "body2", pexclude->get_bodyname2());
  }
}



// constraint section
void mjXWriter::Equality(XMLElement* root) {
  // skip section if empty
  int num;
  if ((num=model->NumObjects(mjOBJ_EQUALITY))==0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "equality");

  // write all constraints
  for (int i=0; i<num; i++) {
    mjCEquality* peq = (mjCEquality*)model->GetObject(mjOBJ_EQUALITY, i);
    XMLElement* elem = InsertEnd(section, FindValue(equality_map, equality_sz, peq->type).c_str());
    OneEquality(elem, peq, peq->def);
  }
}



// deformable section
void mjXWriter::Deformable(XMLElement* root) {
  XMLElement* elem;

  // get sizes
  int nflex = model->NumObjects(mjOBJ_FLEX);
  int nskin = model->NumObjects(mjOBJ_SKIN);

  // return if empty
  if (nflex==0 && nskin==0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "deformable");

  // write flexes
  for (int i=0; i<nflex; i++) {
    // create element and write
    mjCFlex* pflex = (mjCFlex*)model->GetObject(mjOBJ_FLEX, i);
    elem = InsertEnd(section, "flex");
    OneFlex(elem, pflex);
  }

  // write skins
  for (int i=0; i<nskin; i++) {
    // create element and write
    mjCSkin* pskin = (mjCSkin*)model->GetObject(mjOBJ_SKIN, i);
    elem = InsertEnd(section, "skin");
    OneSkin(elem, pskin);
  }
}



// tendon section
void mjXWriter::Tendon(XMLElement* root) {
  // skip section if empty
  int num;
  if ((num=model->NumObjects(mjOBJ_TENDON))==0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "tendon");

  // write all tendons
  for (int i=0; i<num; i++) {
    // write tendon element and attributes
    mjCTendon* pten = (mjCTendon*)model->GetObject(mjOBJ_TENDON, i);
    if (!pten->NumWraps()) {        // SHOULD NOT OCCUR
      continue;
    }
    XMLElement* elem = InsertEnd(section,
                                 pten->GetWrap(0)->type==mjWRAP_JOINT ? "fixed" : "spatial");
    OneTendon(elem, pten, pten->def);

    // write wraps
    XMLElement* wrap;
    for (int j=0; j<pten->NumWraps(); j++) {
      mjCWrap* pw = pten->GetWrap(j);
      switch (pw->type) {
      case mjWRAP_JOINT:
        wrap = InsertEnd(elem, "joint");
        WriteAttrTxt(wrap, "joint", pw->obj->name);
        WriteAttr(wrap, "coef", 1, &pw->prm);
        break;

      case mjWRAP_SITE:
        wrap = InsertEnd(elem, "site");
        WriteAttrTxt(wrap, "site", pw->obj->name);
        break;

      case mjWRAP_SPHERE:
      case mjWRAP_CYLINDER:
        wrap = InsertEnd(elem, "geom");
        WriteAttrTxt(wrap, "geom", pw->obj->name);
        if (!pw->sidesite.empty()) {
          WriteAttrTxt(wrap, "sidesite", pw->sidesite);
        }
        break;

      case mjWRAP_PULLEY:
        wrap = InsertEnd(elem, "pulley");
        WriteAttr(wrap, "divisor", 1, &pw->prm);
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
  if ((num=model->NumObjects(mjOBJ_ACTUATOR))==0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "actuator");

  // write all actuators
  for (int i=0; i<num; i++) {
    mjCActuator* pact = (mjCActuator*)model->GetObject(mjOBJ_ACTUATOR, i);
    XMLElement* elem;
    if (pact->plugin.active) {
      elem = InsertEnd(section, "plugin");
    } else {
      elem = InsertEnd(section, "general");
    }
    OneActuator(elem, pact, pact->def);
  }
}



// sensor section
void mjXWriter::Sensor(XMLElement* root) {
  double zero = 0;

  // skip section if empty
  int num;
  if ((num=model->NumObjects(mjOBJ_SENSOR))==0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "sensor");

  // write all sensors
  for (int i=0; i<num; i++) {
    XMLElement* elem = 0;
    mjCSensor* psen = model->Sensors()[i];
    std::string instance_name = "";
    std::string plugin_name = "";

    // write sensor type and type-specific attributes
    switch (psen->type) {
    // common robotic sensors, attached to a site
    case mjSENS_TOUCH:
      elem = InsertEnd(section, "touch");
      WriteAttrTxt(elem, "site", psen->get_objname());
      break;
    case mjSENS_ACCELEROMETER:
      elem = InsertEnd(section, "accelerometer");
      WriteAttrTxt(elem, "site", psen->get_objname());
      break;
    case mjSENS_VELOCIMETER:
      elem = InsertEnd(section, "velocimeter");
      WriteAttrTxt(elem, "site", psen->get_objname());
      break;
    case mjSENS_GYRO:
      elem = InsertEnd(section, "gyro");
      WriteAttrTxt(elem, "site", psen->get_objname());
      break;
    case mjSENS_FORCE:
      elem = InsertEnd(section, "force");
      WriteAttrTxt(elem, "site", psen->get_objname());
      break;
    case mjSENS_TORQUE:
      elem = InsertEnd(section, "torque");
      WriteAttrTxt(elem, "site", psen->get_objname());
      break;
    case mjSENS_MAGNETOMETER:
      elem = InsertEnd(section, "magnetometer");
      WriteAttrTxt(elem, "site", psen->get_objname());
      break;
    case mjSENS_RANGEFINDER:
      elem = InsertEnd(section, "rangefinder");
      WriteAttrTxt(elem, "site", psen->get_objname());
      break;
    case mjSENS_CAMPROJECTION:
      elem = InsertEnd(section, "camprojection");
      WriteAttrTxt(elem, "site", psen->get_objname());
      WriteAttrTxt(elem, "camera", psen->get_refname());
      break;

    // sensors related to scalar joints, tendons, actuators
    case mjSENS_JOINTPOS:
      elem = InsertEnd(section, "jointpos");
      WriteAttrTxt(elem, "joint", psen->get_objname());
      break;
    case mjSENS_JOINTVEL:
      elem = InsertEnd(section, "jointvel");
      WriteAttrTxt(elem, "joint", psen->get_objname());
      break;
    case mjSENS_TENDONPOS:
      elem = InsertEnd(section, "tendonpos");
      WriteAttrTxt(elem, "tendon", psen->get_objname());
      break;
    case mjSENS_TENDONVEL:
      elem = InsertEnd(section, "tendonvel");
      WriteAttrTxt(elem, "tendon", psen->get_objname());
      break;
    case mjSENS_ACTUATORPOS:
      elem = InsertEnd(section, "actuatorpos");
      WriteAttrTxt(elem, "actuator", psen->get_objname());
      break;
    case mjSENS_ACTUATORVEL:
      elem = InsertEnd(section, "actuatorvel");
      WriteAttrTxt(elem, "actuator", psen->get_objname());
      break;
    case mjSENS_ACTUATORFRC:
      elem = InsertEnd(section, "actuatorfrc");
      WriteAttrTxt(elem, "actuator", psen->get_objname());
      break;
    case mjSENS_JOINTACTFRC:
      elem = InsertEnd(section, "jointactuatorfrc");
      WriteAttrTxt(elem, "joint", psen->get_objname());
      break;

    // sensors related to ball joints
    case mjSENS_BALLQUAT:
      elem = InsertEnd(section, "ballquat");
      WriteAttrTxt(elem, "joint", psen->get_objname());
      break;
    case mjSENS_BALLANGVEL:
      elem = InsertEnd(section, "ballangvel");
      WriteAttrTxt(elem, "joint", psen->get_objname());
      break;

    // joint and tendon limit sensors
    case mjSENS_JOINTLIMITPOS:
      elem = InsertEnd(section, "jointlimitpos");
      WriteAttrTxt(elem, "joint", psen->get_objname());
      break;
    case mjSENS_JOINTLIMITVEL:
      elem = InsertEnd(section, "jointlimitvel");
      WriteAttrTxt(elem, "joint", psen->get_objname());
      break;
    case mjSENS_JOINTLIMITFRC:
      elem = InsertEnd(section, "jointlimitfrc");
      WriteAttrTxt(elem, "joint", psen->get_objname());
      break;
    case mjSENS_TENDONLIMITPOS:
      elem = InsertEnd(section, "tendonlimitpos");
      WriteAttrTxt(elem, "tendon", psen->get_objname());
      break;
    case mjSENS_TENDONLIMITVEL:
      elem = InsertEnd(section, "tendonlimitvel");
      WriteAttrTxt(elem, "tendon", psen->get_objname());
      break;
    case mjSENS_TENDONLIMITFRC:
      elem = InsertEnd(section, "tendonlimitfrc");
      WriteAttrTxt(elem, "tendon", psen->get_objname());
      break;

    // sensors attached to an object with spatial frame: (x)body, geom, site, camera
    case mjSENS_FRAMEPOS:
      elem = InsertEnd(section, "framepos");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      if (psen->reftype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
        WriteAttrTxt(elem, "refname", psen->get_refname());
      }
      break;
    case mjSENS_FRAMEQUAT:
      elem = InsertEnd(section, "framequat");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      if (psen->reftype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
        WriteAttrTxt(elem, "refname", psen->get_refname());
      }
      break;
    case mjSENS_FRAMEXAXIS:
      elem = InsertEnd(section, "framexaxis");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      if (psen->reftype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
        WriteAttrTxt(elem, "refname", psen->get_refname());
      }
      break;
    case mjSENS_FRAMEYAXIS:
      elem = InsertEnd(section, "frameyaxis");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      if (psen->reftype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
        WriteAttrTxt(elem, "refname", psen->get_refname());
      }
      break;
    case mjSENS_FRAMEZAXIS:
      elem = InsertEnd(section, "framezaxis");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      if (psen->reftype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
        WriteAttrTxt(elem, "refname", psen->get_refname());
      }
      break;
    case mjSENS_FRAMELINVEL:
      elem = InsertEnd(section, "framelinvel");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      if (psen->reftype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
        WriteAttrTxt(elem, "refname", psen->get_refname());
      }
      break;
    case mjSENS_FRAMEANGVEL:
      elem = InsertEnd(section, "frameangvel");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      if (psen->reftype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
        WriteAttrTxt(elem, "refname", psen->get_refname());
      }
      break;
    case mjSENS_FRAMELINACC:
      elem = InsertEnd(section, "framelinacc");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      if (psen->reftype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
        WriteAttrTxt(elem, "refname", psen->get_refname());
      }
      break;
    case mjSENS_FRAMEANGACC:
      elem = InsertEnd(section, "frameangacc");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      if (psen->reftype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
        WriteAttrTxt(elem, "refname", psen->get_refname());
      }
      break;

    // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    case mjSENS_SUBTREECOM:
      elem = InsertEnd(section, "subtreecom");
      WriteAttrTxt(elem, "body", psen->get_objname());
      break;
    case mjSENS_SUBTREELINVEL:
      elem = InsertEnd(section, "subtreelinvel");
      WriteAttrTxt(elem, "body", psen->get_objname());
      break;
    case mjSENS_SUBTREEANGMOM:
      elem = InsertEnd(section, "subtreeangmom");
      WriteAttrTxt(elem, "body", psen->get_objname());
      break;
    case mjSENS_GEOMDIST:
      elem = InsertEnd(section, "distance");
      WriteAttrTxt(elem, psen->objtype == mjOBJ_BODY ? "body1" : "geom1", psen->get_objname());
      WriteAttrTxt(elem, psen->reftype == mjOBJ_BODY ? "body2" : "geom2", psen->get_refname());
      break;
    case mjSENS_GEOMNORMAL:
      elem = InsertEnd(section, "normal");
      WriteAttrTxt(elem, psen->objtype == mjOBJ_BODY ? "body1" : "geom1", psen->get_objname());
      WriteAttrTxt(elem, psen->reftype == mjOBJ_BODY ? "body2" : "geom2", psen->get_refname());
      break;
    case mjSENS_GEOMFROMTO:
      elem = InsertEnd(section, "fromto");
      WriteAttrTxt(elem, psen->objtype == mjOBJ_BODY ? "body1" : "geom1", psen->get_objname());
      WriteAttrTxt(elem, psen->reftype == mjOBJ_BODY ? "body2" : "geom2", psen->get_refname());
      break;

    // global sensors
    case mjSENS_CLOCK:
      elem = InsertEnd(section, "clock");
      break;


    // plugin-controlled sensor
    case mjSENS_PLUGIN:
      elem = InsertEnd(section, "plugin");
      if (psen->objtype != mjOBJ_UNKNOWN) {
        WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
        WriteAttrTxt(elem, "objname", psen->get_objname());
      }
      OnePlugin(elem, &psen->plugin);
      break;

    // user-defined sensor
    case mjSENS_USER:
      elem = InsertEnd(section, "user");
      if (mju_type2Str(psen->objtype)) WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->get_objname());
      WriteAttrInt(elem, "dim", psen->dim);
      WriteAttrKey(elem, "needstage", stage_map, stage_sz, (int)psen->needstage);
      WriteAttrKey(elem, "datatype", datatype_map, datatype_sz, (int)psen->datatype);
      break;

    default:
      mju_error("Unknown sensor type in XML write");
    }

    // write name, noise, userdata
    WriteAttrTxt(elem, "name", psen->name);
    WriteAttr(elem, "cutoff", 1, &psen->cutoff, &zero);
    if (psen->type != mjSENS_PLUGIN) {
      WriteAttr(elem, "noise", 1, &psen->noise, &zero);
    }
    WriteVector(elem, "user", psen->get_userdata());
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

  // write all keyframes
  for (int i=0; i<model->nkey; i++) {
    XMLElement* elem = InsertEnd(section, "key");
    bool change = false;

    mjCKey* pk = model->Keys()[i];

    // check name and write
    if (!pk->name.empty()) {
      WriteAttrTxt(elem, "name", pk->name);
      change = true;
    }

    // check time and write
    if (pk->time!=0) {
      WriteAttr(elem, "time", 1, &pk->time);
      change = true;
    }

    // check qpos and write
    for (int j=0; j<model->nq; j++) {
      if (pk->qpos_[j]!=model->qpos0[j]) {
        WriteAttr(elem, "qpos", model->nq, pk->qpos_.data());
        change = true;
        break;
      }
    }

    // check qvel and write
    for (int j=0; j<model->nv; j++) {
      if (pk->qvel_[j]!=0) {
        WriteAttr(elem, "qvel", model->nv, pk->qvel_.data());
        change = true;
        break;
      }
    }

    // check act and write
    for (int j=0; j<model->na; j++) {
      if (pk->act_[j]!=0) {
        WriteAttr(elem, "act", model->na, pk->act_.data());
        change = true;
        break;
      }
    }

    // check mpos and write
    if (model->nmocap) {
      for (int j=0; j<model->nbody; j++) {
        if (model->Bodies()[j]->mocap) {
          mjCBody* pb = model->Bodies()[j];
          int id = pb->mocapid;
          if (pb->pos[0] != pk->mpos_[3*id] ||
              pb->pos[1] != pk->mpos_[3*id+1] ||
              pb->pos[2] != pk->mpos_[3*id+2]) {
            WriteAttr(elem, "mpos", 3*model->nmocap, pk->mpos_.data());
            change = true;
            break;
          }
        }
      }
    }

    // check mquat and write
    if (model->nmocap) {
      for (int j=0; j<model->nbody; j++) {
        if (model->Bodies()[j]->mocap) {
          mjCBody* pb = model->Bodies()[j];
          int id = pb->mocapid;
          if (pb->quat[0] != pk->mquat_[4*id] ||
              pb->quat[1] != pk->mquat_[4*id+1] ||
              pb->quat[2] != pk->mquat_[4*id+2] ||
              pb->quat[3] != pk->mquat_[4*id+3]) {
            WriteAttr(elem, "mquat", 4*model->nmocap, pk->mquat_.data());
            change = true;
            break;
          }
        }
      }
    }

    // check ctrl and write
    for (int j=0; j<model->nu; j++) {
      if (pk->ctrl_[j]!=0) {
        WriteAttr(elem, "ctrl", model->nu, pk->ctrl_.data());
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
