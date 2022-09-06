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

#include <cfloat>
#include <cstddef>
#include <cstdio>
#include <string>

#include "engine/engine_io.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_util.h"
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
class TINYXML2_LIB mj_XMLPrinter : public tinyxml2::XMLPrinter {
  using tinyxml2::XMLPrinter::XMLPrinter;

  public:
    void PrintSpace( int depth ) {
      for( int i=0; i<depth; ++i ) {
          Write( "  " );
      }
    }
};


// save XML file using custom 2-space indentation
tinyxml2::XMLError SaveFile(XMLDocument& doc, FILE* fp) {
  doc.ClearError();
  mj_XMLPrinter stream(fp, /*compact=*/false);
  doc.Print(&stream);
  return doc.ErrorID();
}


// insert end child with given name, return child
XMLElement* mjXWriter::InsertEnd(XMLElement* parent, const char* name) {
  XMLElement* result = parent->GetDocument()->NewElement(name);
  parent->InsertEndChild(result);

  return result;
}


//---------------------------------- class mjXWriter: one-element writers --------------------------

// write mesh
void mjXWriter::OneMesh(XMLElement* elem, mjCMesh* pmesh, mjCDef* def) {
  string text;

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pmesh->name);
    WriteAttrTxt(elem, "class", pmesh->classname);
    WriteAttrTxt(elem, "file", pmesh->file);

    // write vertex data
    if (!pmesh->uservert.empty()) {
      Vector2String(text, pmesh->uservert);
      WriteAttrTxt(elem, "vertex", text);
    }

    // write normal data
    if (!pmesh->usernormal.empty()) {
      Vector2String(text, pmesh->usernormal);
      WriteAttrTxt(elem, "normal", text);
    }

    // write texcoord data
    if (!pmesh->usertexcoord.empty()) {
      Vector2String(text, pmesh->usertexcoord);
      WriteAttrTxt(elem, "texcoord", text);
    }

    // write face data
    if (!pmesh->userface.empty()) {
      Vector2String(text, pmesh->userface);
      WriteAttrTxt(elem, "face", text);
    }
  }

  // defaults and regular
  WriteAttr(elem, "refpos", 3, pmesh->refpos, def->mesh.refpos);
  WriteAttr(elem, "refquat", 4, pmesh->refquat, def->mesh.refquat);
  WriteAttr(elem, "scale", 3, pmesh->scale, def->mesh.scale);
  WriteAttrKey(elem, "smoothnormal", bool_map, 2, pmesh->smoothnormal, def->mesh.smoothnormal);
}



// write skin
void mjXWriter::OneSkin(XMLElement* elem, mjCSkin* pskin) {
  string text;
  mjCDef mydef;
  float zero = 0;

  // write attributes
  WriteAttrTxt(elem, "name", pskin->name);
  WriteAttrTxt(elem, "file", pskin->file);
  WriteAttrTxt(elem, "material", pskin->material);
  WriteAttrInt(elem, "group", pskin->group, 0);
  WriteAttr(elem, "rgba", 4, pskin->rgba, mydef.geom.rgba);
  WriteAttr(elem, "inflate", 1, &pskin->inflate, &zero);

  // write data if no file
  if (pskin->file.empty()) {
    // mesh vert
    Vector2String(text, pskin->vert);
    WriteAttrTxt(elem, "vertex", text);

    // mesh texcoord
    if (!pskin->texcoord.empty()) {
      Vector2String(text, pskin->texcoord);
      WriteAttrTxt(elem, "texcoord", text);
    }

    // mesh face
    Vector2String(text, pskin->face);
    WriteAttrTxt(elem, "face", text);

    // bones
    for (size_t i=0; i<pskin->bodyname.size(); i++) {
      // make bone
      XMLElement* bone = InsertEnd(elem, "bone");

      // write attributes
      WriteAttrTxt(bone, "body", pskin->bodyname[i]);
      WriteAttr(bone, "bindpos", 3, pskin->bindpos.data()+3*i);
      WriteAttr(bone, "bindquat", 4, pskin->bindquat.data()+4*i);

      // write vertid
      Vector2String(text, pskin->vertid[i]);
      WriteAttrTxt(bone, "vertid", text);

      // write vertweight
      Vector2String(text, pskin->vertweight[i]);
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
    WriteAttrTxt(elem, "texture", pmat->texture);
  }
  WriteAttrKey(elem, "texuniform", bool_map, 2, pmat->texuniform, def->material.texuniform);
  WriteAttr(elem, "texrepeat", 2, pmat->texrepeat, def->material.texrepeat);
  WriteAttr(elem, "emission", 1, &pmat->emission, &def->material.emission);
  WriteAttr(elem, "specular", 1, &pmat->specular, &def->material.specular);
  WriteAttr(elem, "shininess", 1, &pmat->shininess, &def->material.shininess);
  WriteAttr(elem, "reflectance", 1, &pmat->reflectance, &def->material.reflectance);
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
      WriteAttr(elem, "pos", 3, pjoint->locpos);
    }
    if (pjoint->type != mjJNT_FREE && pjoint->type != mjJNT_BALL) {
      WriteAttr(elem, "axis", 3, pjoint->locaxis);
    }
  }

  // special handling of limits
  bool range_defined = pjoint->range[0]!=0 || pjoint->range[1]!=0;
  bool limited_inferred = def->joint.limited==2 && pjoint->limited==range_defined;
  if (writingdefaults || !limited_inferred) {
    WriteAttrKey(elem, "limited", TFAuto_map, 3, pjoint->limited, def->joint.limited);
  }

  // defaults and regular
  if (pjoint->type != def->joint.type) {
    WriteAttrTxt(elem, "type", FindValue(joint_map, joint_sz, pjoint->type));
  }
  WriteAttrInt(elem, "group", pjoint->group, def->joint.group);
  WriteAttr(elem, "ref", 1, &pjoint->ref, &zero);
  WriteAttr(elem, "springref", 1, &pjoint->springref, &zero);
  WriteAttr(elem, "solreflimit", mjNREF, pjoint->solref_limit, def->joint.solref_limit);
  WriteAttr(elem, "solimplimit", mjNIMP, pjoint->solimp_limit, def->joint.solimp_limit);
  WriteAttr(elem, "solreffriction", mjNREF, pjoint->solref_friction, def->joint.solref_friction);
  WriteAttr(elem, "solimpfriction", mjNIMP, pjoint->solimp_friction, def->joint.solimp_friction);
  WriteAttr(elem, "stiffness", 1, &pjoint->stiffness, &def->joint.stiffness);
  WriteAttr(elem, "range", 2, pjoint->range, def->joint.range);
  WriteAttr(elem, "margin", 1, &pjoint->margin, &def->joint.margin);
  WriteAttr(elem, "armature", 1, &pjoint->armature, &def->joint.armature);
  WriteAttr(elem, "damping", 1, &pjoint->damping, &def->joint.damping);
  WriteAttr(elem, "frictionloss", 1, &pjoint->frictionloss, &def->joint.frictionloss);

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pjoint->userdata);
  } else {
    WriteVector(elem, "user", pjoint->userdata, def->joint.userdata);
  }
}



// write geom
void mjXWriter::OneGeom(XMLElement* elem, mjCGeom* pgeom, mjCDef* def) {
  double unitq[4] = {1, 0, 0, 0};

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pgeom->name);
    WriteAttrTxt(elem, "class", pgeom->classname);
    if (mjGEOMINFO[pgeom->type]) {
      WriteAttr(elem, "size", mjGEOMINFO[pgeom->type], pgeom->size, def->geom.size);
    }

    // mesh geom
    if (pgeom->type==mjGEOM_MESH) {
      mjCMesh* pmesh = model->meshes[pgeom->meshid];

      // write pos/quat if there is a difference
      if (!SameVector(pgeom->locpos, pmesh->GetPosPtr(pgeom->typeinertia), 3) ||
          !SameVector(pgeom->locquat, pmesh->GetQuatPtr(pgeom->typeinertia), 4)) {
        // recover geom pos/quat before mesh frame transformation
        double p[3], q[4];
        mjuu_copyvec(p, pgeom->locpos, 3);
        mjuu_copyvec(q, pgeom->locquat, 4);
        mjuu_frameaccuminv(p, q, pmesh->GetPosPtr(pgeom->typeinertia),
                           pmesh->GetQuatPtr(pgeom->typeinertia));

        // write
        WriteAttr(elem, "pos", 3, p, unitq+1);
        WriteAttr(elem, "quat", 4, q, unitq);
      }
    }

    // non-mesh geom
    else {
      WriteAttr(elem, "pos", 3, pgeom->locpos, unitq+1);
      WriteAttr(elem, "quat", 4, pgeom->locquat, unitq);
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
  WriteAttr(elem, "friction", 3, pgeom->friction, def->geom.friction);
  WriteAttr(elem, "solmix", 1, &pgeom->solmix, &def->geom.solmix);
  WriteAttr(elem, "solref", mjNREF, pgeom->solref, def->geom.solref);
  WriteAttr(elem, "solimp", mjNIMP, pgeom->solimp, def->geom.solimp);
  WriteAttr(elem, "margin", 1, &pgeom->margin, &def->geom.margin);
  WriteAttr(elem, "gap", 1, &pgeom->gap, &def->geom.gap);
  WriteAttr(elem, "gap", 1, &pgeom->gap, &def->geom.gap);
  WriteAttrKey(elem, "fluidshape", fluid_map, 2, pgeom->fluid_switch, def->geom.fluid_switch);
  WriteAttr(elem, "fluidcoef", 5, pgeom->fluid_coefs, def->geom.fluid_coefs);
  WriteAttrKey(elem, "shellinertia", meshtype_map, 2, pgeom->typeinertia, def->geom.typeinertia);
  if (mjuu_defined(pgeom->_mass)) {
    double mass = pgeom->GetVolume() * def->geom.density;
    WriteAttr(elem, "mass", 1, &pgeom->mass, &mass);
  } else {
    WriteAttr(elem, "density", 1, &pgeom->density, &def->geom.density);
  }
  if (pgeom->material != def->geom.material) {
    WriteAttrTxt(elem, "material", pgeom->material);
  }
  WriteAttr(elem, "rgba", 4, pgeom->rgba, def->geom.rgba);

  // hfield and mesh attributes
  if (pgeom->type==mjGEOM_HFIELD) {
    WriteAttrTxt(elem, "hfield", pgeom->hfield);
  }
  if (pgeom->type==mjGEOM_MESH) {
    WriteAttrTxt(elem, "mesh", pgeom->mesh);
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pgeom->userdata);
  } else {
    WriteVector(elem, "user", pgeom->userdata, def->geom.userdata);
  }
}



// write site
void mjXWriter::OneSite(XMLElement* elem, mjCSite* psite, mjCDef* def) {
  double unitq[4] = {1, 0, 0, 0};

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", psite->name);
    WriteAttrTxt(elem, "class", psite->classname);
    WriteAttr(elem, "pos", 3, psite->locpos);
    WriteAttr(elem, "quat", 4, psite->locquat, unitq);
    if (mjGEOMINFO[psite->type]) {
      WriteAttr(elem, "size", mjGEOMINFO[psite->type], psite->size, def->site.size);
    }
  } else {
    WriteAttr(elem, "size", 3, psite->size, def->site.size);
  }

  // defaults and regular
  WriteAttrInt(elem, "group", psite->group, def->site.group);
  WriteAttrKey(elem, "type", geom_map, mjNGEOMTYPES, psite->type, def->site.type);
  if (psite->material != def->site.material) {
    WriteAttrTxt(elem, "material", psite->material);
  }
  WriteAttr(elem, "rgba", 4, psite->rgba, def->site.rgba);

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", psite->userdata);
  } else {
    WriteVector(elem, "user", psite->userdata, def->site.userdata);
  }
}



// write camera
void mjXWriter::OneCamera(XMLElement* elem, mjCCamera* pcam, mjCDef* def) {
  double unitq[4] = {1, 0, 0, 0};

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pcam->name);
    WriteAttrTxt(elem, "class", pcam->classname);
    WriteAttrTxt(elem, "target", pcam->targetbody);
    WriteAttr(elem, "pos", 3, pcam->locpos);
    WriteAttr(elem, "quat", 4, pcam->locquat, unitq);
  }

  // defaults and regular
  WriteAttr(elem, "ipd", 1, &pcam->ipd, &def->camera.ipd);
  WriteAttr(elem, "fovy", 1, &pcam->fovy, &def->camera.fovy);
  WriteAttrKey(elem, "mode", camlight_map, camlight_sz, pcam->mode, def->camera.mode);

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pcam->userdata);
  } else {
    WriteVector(elem, "user", pcam->userdata, def->camera.userdata);
  }
}



// write light
void mjXWriter::OneLight(XMLElement* elem, mjCLight* plight, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", plight->name);
    WriteAttrTxt(elem, "class", plight->classname);
    WriteAttrTxt(elem, "target", plight->targetbody);
    WriteAttr(elem, "pos", 3, plight->locpos);
    WriteAttr(elem, "dir", 3, plight->locdir);
  }

  // defaults and regular
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
    WriteAttrTxt(elem, "geom1", ppair->geomname1);
    WriteAttrTxt(elem, "geom2", ppair->geomname2);
  }

  // defaults and regular
  WriteAttrTxt(elem, "name", ppair->name);
  WriteAttrInt(elem, "condim", ppair->condim, def->pair.condim);
  WriteAttr(elem, "margin", 1, &ppair->margin, &def->pair.margin);
  WriteAttr(elem, "gap", 1, &ppair->gap, &def->pair.gap);
  WriteAttr(elem, "solref", mjNREF, ppair->solref, def->pair.solref);
  WriteAttr(elem, "solimp", mjNIMP, ppair->solimp, def->pair.solimp);
  WriteAttr(elem, "friction", 5, ppair->friction, def->pair.friction);
}



// write equality
void mjXWriter::OneEquality(XMLElement* elem, mjCEquality* peq, mjCDef* def) {
  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", peq->name);
    WriteAttrTxt(elem, "class", peq->classname);

    switch (peq->type) {
    case mjEQ_CONNECT:
      WriteAttrTxt(elem, "body1", peq->name1);
      WriteAttrTxt(elem, "body2", peq->name2);
      WriteAttr(elem, "anchor", 3, peq->data);
      break;

    case mjEQ_WELD:
      WriteAttrTxt(elem, "body1", peq->name1);
      WriteAttrTxt(elem, "body2", peq->name2);
      WriteAttr(elem, "anchor", 3, peq->data);
      WriteAttr(elem, "torquescale", 1, peq->data+10);
      WriteAttr(elem, "relpose", 7, peq->data+3);
      break;

    case mjEQ_JOINT:
      WriteAttrTxt(elem, "joint1", peq->name1);
      WriteAttrTxt(elem, "joint2", peq->name2);
      WriteAttr(elem, "polycoef", 5, peq->data);
      break;

    case mjEQ_TENDON:
      WriteAttrTxt(elem, "tendon1", peq->name1);
      WriteAttrTxt(elem, "tendon2", peq->name2);
      WriteAttr(elem, "polycoef", 5, peq->data);
      break;

    case mjEQ_DISTANCE:
      WriteAttrTxt(elem, "geom1", peq->name1);
      WriteAttrTxt(elem, "geom2", peq->name2);
      WriteAttr(elem, "distance", 1, peq->data);
      break;
    }
  }

  // defaults and regular
  WriteAttrKey(elem, "active", bool_map, 2, peq->active, def->equality.active);
  WriteAttr(elem, "solref", mjNREF, peq->solref, def->equality.solref);
  WriteAttr(elem, "solimp", mjNIMP, peq->solimp, def->equality.solimp);
}



// write tendon
void mjXWriter::OneTendon(XMLElement* elem, mjCTendon* pten, mjCDef* def) {
  bool fixed = (pten->GetWrap(0) && pten->GetWrap(0)->type==mjWRAP_JOINT);

  // regular
  if (!writingdefaults) {
    WriteAttrTxt(elem, "name", pten->name);
    WriteAttrTxt(elem, "class", pten->classname);
  }

  // special handling of limits
  bool range_defined = pten->range[0]!=0 || pten->range[1]!=0;
  bool limited_inferred = def->tendon.limited==2 && pten->limited==range_defined;
  if (writingdefaults || !limited_inferred) {
    WriteAttrKey(elem, "limited", TFAuto_map, 3, pten->limited, def->tendon.limited);
  }

  // defaults and regular
  WriteAttrInt(elem, "group", pten->group, def->tendon.group);
  WriteAttr(elem, "solreflimit", mjNREF, pten->solref_limit, def->tendon.solref_limit);
  WriteAttr(elem, "solimplimit", mjNIMP, pten->solimp_limit, def->tendon.solimp_limit);
  WriteAttr(elem, "solreffriction", mjNREF, pten->solref_friction, def->tendon.solref_friction);
  WriteAttr(elem, "solimpfriction", mjNIMP, pten->solimp_friction, def->tendon.solimp_friction);
  WriteAttr(elem, "range", 2, pten->range, def->tendon.range);
  WriteAttr(elem, "margin", 1, &pten->margin, &def->tendon.margin);
  WriteAttr(elem, "stiffness", 1, &pten->stiffness, &def->tendon.stiffness);
  WriteAttr(elem, "damping", 1, &pten->damping, &def->tendon.damping);
  WriteAttr(elem, "frictionloss", 1, &pten->frictionloss, &def->tendon.frictionloss);
  WriteAttr(elem, "springlength", 1, &pten->springlength, &def->tendon.springlength);

  // spatial only
  if (!fixed) {
    if (pten->material!=def->tendon.material) {
      WriteAttrTxt(elem, "material", pten->material);
    }
    WriteAttr(elem, "width", 1, &pten->width, &def->tendon.width);
    WriteAttr(elem, "rgba", 4, pten->rgba, def->tendon.rgba);
  }

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pten->userdata);
  } else {
    WriteVector(elem, "user", pten->userdata, def->tendon.userdata);
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
      WriteAttrTxt(elem, "joint", pact->target);
      break;

    case mjTRN_JOINTINPARENT:
      WriteAttrTxt(elem, "jointinparent", pact->target);
      break;

    case mjTRN_TENDON:
      WriteAttrTxt(elem, "tendon", pact->target);
      break;

    case mjTRN_SLIDERCRANK:
      WriteAttrTxt(elem, "cranksite", pact->target);
      WriteAttrTxt(elem, "slidersite", pact->slidersite);
      break;

    case mjTRN_SITE:
      WriteAttrTxt(elem, "site", pact->target);
      WriteAttrTxt(elem, "refsite", pact->refsite);
      break;

    case mjTRN_BODY:
      WriteAttrTxt(elem, "body", pact->target);
      break;

    default:        // SHOULD NOT OCCUR
      break;
    }
  }

  // special handling of limits
  bool range_defined, limited_inferred;
  range_defined = pact->ctrlrange[0]!=0 || pact->ctrlrange[1]!=0;
  limited_inferred = def->actuator.ctrllimited==2 && pact->ctrllimited==range_defined;
  if (writingdefaults || !limited_inferred) {
    WriteAttrKey(elem, "ctrllimited", TFAuto_map, 3, pact->ctrllimited, def->actuator.ctrllimited);
  }
  range_defined = pact->forcerange[0]!=0 || pact->forcerange[1]!=0;
  limited_inferred = def->actuator.forcelimited==2 && pact->forcelimited==range_defined;
  if (writingdefaults || !limited_inferred) {
    WriteAttrKey(elem, "forcelimited", TFAuto_map, 3, pact->forcelimited, def->actuator.forcelimited);
  }
  range_defined = pact->actrange[0]!=0 || pact->actrange[1]!=0;
  limited_inferred = def->actuator.actlimited==2 && pact->actlimited==range_defined;
  if (writingdefaults || !limited_inferred) {
    WriteAttrKey(elem, "actlimited", TFAuto_map, 3, pact->actlimited, def->actuator.actlimited);
  }

  // defaults and regular
  WriteAttrInt(elem, "group", pact->group, def->actuator.group);
  WriteAttr(elem, "ctrlrange", 2, pact->ctrlrange, def->actuator.ctrlrange);
  WriteAttr(elem, "forcerange", 2, pact->forcerange, def->actuator.forcerange);
  WriteAttr(elem, "actrange", 2, pact->actrange, def->actuator.actrange);
  WriteAttr(elem, "lengthrange", 2, pact->lengthrange, def->actuator.lengthrange);
  WriteAttr(elem, "gear", 6, pact->gear, def->actuator.gear);
  WriteAttr(elem, "cranklength", 1, &pact->cranklength, &def->actuator.cranklength);
  WriteAttrKey(elem, "dyntype", dyn_map, dyn_sz, pact->dyntype, def->actuator.dyntype);
  WriteAttrKey(elem, "gaintype", gain_map, gain_sz, pact->gaintype, def->actuator.gaintype);
  WriteAttrKey(elem, "biastype", bias_map, bias_sz, pact->biastype, def->actuator.biastype);
  WriteAttr(elem, "dynprm", mjNDYN, pact->dynprm, def->actuator.dynprm);
  WriteAttr(elem, "gainprm", mjNGAIN, pact->gainprm, def->actuator.gainprm);
  WriteAttr(elem, "biasprm", mjNBIAS, pact->biasprm, def->actuator.biasprm);

  // userdata
  if (writingdefaults) {
    WriteVector(elem, "user", pact->userdata);
  } else {
    WriteVector(elem, "user", pact->userdata, def->actuator.userdata);
  }
}



//---------------------------------- class mjXWriter: top-level API --------------------------------

// constructor
mjXWriter::mjXWriter(void) {
  writingdefaults = false;
}


// save existing model in MJCF canonical format, must be compiled
void mjXWriter::Write(FILE* fp) {
  // check model
  if (!model || !model->IsCompiled()) {
    throw mjXError(0, "XML Write error: Only compiled model can be written");
  }

  // create document and root
  XMLDocument doc;
  XMLElement* root = doc.NewElement("mujoco");
  root->SetAttribute("model", model->modelname.c_str());

  // insert root
  doc.InsertFirstChild(root);

  // write comment if present
  if (!model->comment.empty()) {
    XMLComment* comment = doc.NewComment(model->comment.c_str());
    root->LinkEndChild(comment);
  }

  // create DOM
  Compiler(root);
  Option(root);
  Size(root);
  Visual(root);
  Statistic(root);
  writingdefaults = true;
  Default(root, model->defaults[0]);
  writingdefaults = false;
  Custom(root);
  Asset(root);
  Body(InsertEnd(root, "worldbody"), model->GetWorld());
  Contact(root);
  Equality(root);
  Tendon(root);
  Actuator(root);
  Sensor(root);
  Keyframe(root);

  // save file
  SaveFile(doc, fp);
}



// compiler section
void mjXWriter::Compiler(XMLElement* root) {
  XMLElement* section = InsertEnd(root, "compiler");

  // settings
  if (!model->convexhull) {
    WriteAttrTxt(section, "convexhull", FindValue(bool_map, 2, model->convexhull));
  }
  WriteAttrTxt(section, "angle", "radian");
  if (!model->meshdir.empty()) {
    WriteAttrTxt(section, "meshdir", model->meshdir);
  }
  if (!model->texturedir.empty()) {
    WriteAttrTxt(section, "texturedir", model->texturedir);
  }
  if (!model->usethread) {
    WriteAttrTxt(section, "usethread", "false");
  }
  if (model->exactmeshinertia) {
    WriteAttrTxt(section, "exactmeshinertia", "true");
  }
  // always enable autolimits. limited attributes will be written appropriately
  // TODO(b/245077553): Remove this when the default is true.
  WriteAttrTxt(section, "autolimits", "true");
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

  WriteAttrKey(section, "integrator", integrator_map, integrator_sz,
               model->option.integrator, opt.integrator);
  WriteAttrKey(section, "collision", collision_map, collision_sz,
               model->option.collision, opt.collision);
  WriteAttrKey(section, "cone", cone_map, cone_sz,
               model->option.cone, opt.cone);
  WriteAttrKey(section, "jacobian", jac_map, jac_sz,
               model->option.jacobian, opt.jacobian);
  WriteAttrKey(section, "solver", solver_map, solver_sz,
               model->option.solver, opt.solver);
  WriteAttrInt(section, "iterations", model->option.iterations, opt.iterations);
  WriteAttrInt(section, "noslip_iterations", model->option.noslip_iterations, opt.noslip_iterations);
  WriteAttrInt(section, "mpr_iterations", model->option.mpr_iterations, opt.mpr_iterations);

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
#undef WRITEDSBL

#define WRITEENBL(NAME, MASK) \
    if( model->option.enableflags & MASK ) \
      WriteAttrKey(sub, NAME, enable_map, 2, 1);
    WRITEENBL("override",       mjENBL_OVERRIDE)
    WRITEENBL("energy",         mjENBL_ENERGY)
    WRITEENBL("fwdinv",         mjENBL_FWDINV)
    WRITEENBL("sensornoise",    mjENBL_SENSORNOISE)
    WRITEENBL("multiccd",       mjENBL_MULTICCD)
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
}



// statistic section
void mjXWriter::Statistic(XMLElement* root) {
  XMLElement* section = InsertEnd(root, "statistic");

  if (mjuu_defined(model->meaninertia)) WriteAttr(section, "meaninertia", 1, &model->meaninertia);
  if (mjuu_defined(model->meanmass)) WriteAttr(section, "meanmass", 1, &model->meanmass);
  if (mjuu_defined(model->meansize)) WriteAttr(section, "meansize", 1, &model->meansize);
  if (mjuu_defined(model->extent)) WriteAttr(section, "extent", 1, &model->extent);
  if (mjuu_defined(model->center[0])) WriteAttr(section, "center", 3, model->center);

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
  WriteAttrInt(elem, "offwidth",       vis->global.offwidth,    visdef.global.offwidth);
  WriteAttrInt(elem, "offheight",      vis->global.offheight,   visdef.global.offheight);
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
    par = model->defaults[def->parentid];
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

  // add children recursively
  for (int i=0; i<(int)def->childid.size(); i++) {
    Default(section, model->defaults[def->childid[i]]);
  }

  // delete parent defaults if allocated here
  if (def->parentid<0) {
    delete par;
  }
}



// custom section
void mjXWriter::Custom(XMLElement* root) {
  XMLElement* elem;
  int i, j;

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
  for (i=0; i<nnum; i++) {
    mjCNumeric* ptr = (mjCNumeric*)model->GetObject(mjOBJ_NUMERIC, i);
    elem = InsertEnd(section, "numeric");
    WriteAttrTxt(elem, "name", ptr->name);
    WriteAttrInt(elem, "size", ptr->size);
    WriteAttr(elem, "data", ptr->size, ptr->data.data());
  }

  // write all texts
  for (i=0; i<ntxt; i++) {
    mjCText* ptr = (mjCText*)model->GetObject(mjOBJ_TEXT, i);
    elem = InsertEnd(section, "text");
    WriteAttrTxt(elem, "name", ptr->name);
    WriteAttrTxt(elem, "data", ptr->data.c_str());
  }

  // write all tuples
  for (i=0; i<ntup; i++) {
    mjCTuple* ptr = (mjCTuple*)model->GetObject(mjOBJ_TUPLE, i);
    elem = InsertEnd(section, "tuple");
    WriteAttrTxt(elem, "name", ptr->name);

    // write objects in tuple
    for (j=0; j<(int)ptr->objtype.size(); j++) {
      XMLElement* obj = InsertEnd(elem, "element");
      WriteAttrTxt(obj, "objtype", mju_type2Str((int)ptr->objtype[j]));
      WriteAttrTxt(obj, "objname", ptr->objname[j].c_str());
      double oprm = ptr->objprm[j];
      if (oprm!=0) {
        WriteAttr(obj, "prm", 1, &oprm);
      }
    }
  }
}



// asset section
void mjXWriter::Asset(XMLElement* root) {
  XMLElement* elem;
  int i;

  // get sizes
  int ntex = model->NumObjects(mjOBJ_TEXTURE);
  int nmat = model->NumObjects(mjOBJ_MATERIAL);
  int nmesh = model->NumObjects(mjOBJ_MESH);
  int nskin = model->NumObjects(mjOBJ_SKIN);
  int nhfield = model->NumObjects(mjOBJ_HFIELD);

  // return if empty
  if (ntex==0 && nmat==0 && nmesh==0 && nhfield==0 && nskin==0) {
    return;
  }

  // create section
  XMLElement* section = InsertEnd(root, "asset");

  // write textures
  mjCTexture deftex(0);
  for (i=0; i<ntex; i++) {
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

    // write texures loaded from files
    else {
      // write singe file
      WriteAttrTxt(elem, "file", ptex->file);

      // write separate files
      WriteAttrTxt(elem, "fileright", ptex->cubefiles[0]);
      WriteAttrTxt(elem, "fileleft", ptex->cubefiles[1]);
      WriteAttrTxt(elem, "fileup", ptex->cubefiles[2]);
      WriteAttrTxt(elem, "filedown", ptex->cubefiles[3]);
      WriteAttrTxt(elem, "filefront", ptex->cubefiles[4]);
      WriteAttrTxt(elem, "fileback", ptex->cubefiles[5]);
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
  for (i=0; i<nmat; i++) {
    // create element and write
    mjCMaterial* pmat = (mjCMaterial*)model->GetObject(mjOBJ_MATERIAL, i);
    elem = InsertEnd(section, "material");
    OneMaterial(elem, pmat, pmat->def);
  }

  // write meshes
  for (i=0; i<nmesh; i++) {
    // create element and write
    mjCMesh* pmesh = (mjCMesh*)model->GetObject(mjOBJ_MESH, i);
    elem = InsertEnd(section, "mesh");
    OneMesh(elem, pmesh, pmesh->def);
  }

  // write skins
  for (i=0; i<nskin; i++) {
    // create element and write
    mjCSkin* pskin = (mjCSkin*)model->GetObject(mjOBJ_SKIN, i);
    elem = InsertEnd(section, "skin");
    OneSkin(elem, pskin);
  }

  // write hfields
  for (i=0; i<nhfield; i++) {
    // create element
    mjCHField* phf = (mjCHField*)model->GetObject(mjOBJ_HFIELD, i);
    elem = InsertEnd(section, "hfield");

    // write attributes
    WriteAttrTxt(elem, "name", phf->name);
    WriteAttr(elem, "size", 4, phf->size);
    if (!phf->file.empty()) {
      WriteAttrTxt(elem, "file", phf->file);
    } else {
      WriteAttrInt(elem, "nrow", phf->nrow);
      WriteAttrInt(elem, "ncol", phf->ncol);
    }
  }
}



// recursive body writer
void mjXWriter::Body(XMLElement* elem, mjCBody* body) {
  double unitq[4] = {1, 0, 0, 0};
  unsigned int i;

  if (!body) {
    throw mjXError(0, "missing body in XML write");  // SHOULD NOT OCCUR
  }

  // write body attributes and inertial
  if (body!=model->GetWorld()) {
    WriteAttrTxt(elem, "name", body->name);
    WriteAttrTxt(elem, "childclass", body->classname);
    WriteAttr(elem, "pos", 3, body->locpos);
    WriteAttr(elem, "quat", 4, body->locquat, unitq);
    if (body->mocap) {
      WriteAttrKey(elem, "mocap", bool_map, 2, 1);
    }

    // userdata
    WriteVector(elem, "user", body->userdata);

    // write inertial
    if (body->explicitinertial &&
        model->inertiafromgeom!=mjINERTIAFROMGEOM_TRUE) {
      XMLElement* inertial = InsertEnd(elem, "inertial");
      WriteAttr(inertial, "pos", 3, body->locipos);
      WriteAttr(inertial, "quat", 4, body->lociquat, unitq);
      WriteAttr(inertial, "mass", 1, &body->mass);
      WriteAttr(inertial, "diaginertia", 3, body->inertia);
    }
  }

  // write joints
  for (i=0; i<body->joints.size(); i++) {
    OneJoint(InsertEnd(elem, "joint"), body->joints[i], body->joints[i]->def);
  }

  // write geoms
  for (i=0; i<body->geoms.size(); i++) {
    OneGeom(InsertEnd(elem, "geom"), body->geoms[i], body->geoms[i]->def);
  }

  // write sites
  for (i=0; i<body->sites.size(); i++) {
    OneSite(InsertEnd(elem, "site"), body->sites[i], body->sites[i]->def);
  }

  // write cameras
  for (i=0; i<body->cameras.size(); i++) {
    OneCamera(InsertEnd(elem, "camera"), body->cameras[i], body->cameras[i]->def);
  }

  // write lights
  for (i=0; i<body->lights.size(); i++) {
    OneLight(InsertEnd(elem, "light"), body->lights[i], body->lights[i]->def);
  }

  // write child bodies recursively
  for (i=0; i<body->bodies.size(); i++) {
    Body(InsertEnd(elem, "body"), body->bodies[i]);
  }
}



// collision section
void mjXWriter::Contact(XMLElement* root) {
  XMLElement* elem;
  int i;

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
  for (i=0; i<npair; i++) {
    // create element and write
    mjCPair* ppair = (mjCPair*)model->GetObject(mjOBJ_PAIR, i);
    elem = InsertEnd(section, "pair");
    OnePair(elem, ppair, ppair->def);
  }

  // write all exclude pairs
  for (i=0; i<nexclude; i++) {
    // create element
    mjCBodyPair* pexclude = (mjCBodyPair*)model->GetObject(mjOBJ_EXCLUDE, i);
    elem = InsertEnd(section, "exclude");

    // write attributes
    WriteAttrTxt(elem, "name", pexclude->name);
    WriteAttrTxt(elem, "body1", pexclude->bodyname1);
    WriteAttrTxt(elem, "body2", pexclude->bodyname2);
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
    mjCBase* pobj;
    XMLElement* wrap;
    for (int j=0; j<pten->NumWraps(); j++) {
      mjCWrap* pw = pten->GetWrap(j);
      switch (pw->type) {
      case mjWRAP_JOINT:
        if ((pobj = model->GetObject(mjOBJ_JOINT, pw->objid))) {
          wrap = InsertEnd(elem, "joint");
          WriteAttrTxt(wrap, "joint", pobj->name);
          WriteAttr(wrap, "coef", 1, &pw->prm);
        }
        break;

      case mjWRAP_SITE:
        if ((pobj = model->GetObject(mjOBJ_SITE, pw->objid))) {
          wrap = InsertEnd(elem, "site");
          WriteAttrTxt(wrap, "site", pobj->name);
        }
        break;

      case mjWRAP_SPHERE:
      case mjWRAP_CYLINDER:
        if ((pobj = model->GetObject(mjOBJ_GEOM, pw->objid))) {
          wrap = InsertEnd(elem, "geom");
          WriteAttrTxt(wrap, "geom", pobj->name);
          if (!pw->sidesite.empty()) {
            WriteAttrTxt(wrap, "sidesite", pw->sidesite);
          }
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
    XMLElement* elem = InsertEnd(section, "general");
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
    mjCSensor* psen = model->sensors[i];

    // write sensor type and type-specific attributes
    switch (psen->type) {
    // common robotic sensors, attached to a site
    case mjSENS_TOUCH:
      elem = InsertEnd(section, "touch");
      WriteAttrTxt(elem, "site", psen->objname);
      break;
    case mjSENS_ACCELEROMETER:
      elem = InsertEnd(section, "accelerometer");
      WriteAttrTxt(elem, "site", psen->objname);
      break;
    case mjSENS_VELOCIMETER:
      elem = InsertEnd(section, "velocimeter");
      WriteAttrTxt(elem, "site", psen->objname);
      break;
    case mjSENS_GYRO:
      elem = InsertEnd(section, "gyro");
      WriteAttrTxt(elem, "site", psen->objname);
      break;
    case mjSENS_FORCE:
      elem = InsertEnd(section, "force");
      WriteAttrTxt(elem, "site", psen->objname);
      break;
    case mjSENS_TORQUE:
      elem = InsertEnd(section, "torque");
      WriteAttrTxt(elem, "site", psen->objname);
      break;
    case mjSENS_MAGNETOMETER:
      elem = InsertEnd(section, "magnetometer");
      WriteAttrTxt(elem, "site", psen->objname);
      break;
    case mjSENS_RANGEFINDER:
      elem = InsertEnd(section, "rangefinder");
      WriteAttrTxt(elem, "site", psen->objname);
      break;

    // sensors related to scalar joints, tendons, actuators
    case mjSENS_JOINTPOS:
      elem = InsertEnd(section, "jointpos");
      WriteAttrTxt(elem, "joint", psen->objname);
      break;
    case mjSENS_JOINTVEL:
      elem = InsertEnd(section, "jointvel");
      WriteAttrTxt(elem, "joint", psen->objname);
      break;
    case mjSENS_TENDONPOS:
      elem = InsertEnd(section, "tendonpos");
      WriteAttrTxt(elem, "tendon", psen->objname);
      break;
    case mjSENS_TENDONVEL:
      elem = InsertEnd(section, "tendonvel");
      WriteAttrTxt(elem, "tendon", psen->objname);
      break;
    case mjSENS_ACTUATORPOS:
      elem = InsertEnd(section, "actuatorpos");
      WriteAttrTxt(elem, "actuator", psen->objname);
      break;
    case mjSENS_ACTUATORVEL:
      elem = InsertEnd(section, "actuatorvel");
      WriteAttrTxt(elem, "actuator", psen->objname);
      break;
    case mjSENS_ACTUATORFRC:
      elem = InsertEnd(section, "actuatorfrc");
      WriteAttrTxt(elem, "actuator", psen->objname);
      break;

    // sensors related to ball joints
    case mjSENS_BALLQUAT:
      elem = InsertEnd(section, "ballquat");
      WriteAttrTxt(elem, "joint", psen->objname);
      break;
    case mjSENS_BALLANGVEL:
      elem = InsertEnd(section, "ballangvel");
      WriteAttrTxt(elem, "joint", psen->objname);
      break;

    // joint and tendon limit sensors
    case mjSENS_JOINTLIMITPOS:
      elem = InsertEnd(section, "jointlimitpos");
      WriteAttrTxt(elem, "joint", psen->objname);
      break;
    case mjSENS_JOINTLIMITVEL:
      elem = InsertEnd(section, "jointlimitvel");
      WriteAttrTxt(elem, "joint", psen->objname);
      break;
    case mjSENS_JOINTLIMITFRC:
      elem = InsertEnd(section, "jointlimitfrc");
      WriteAttrTxt(elem, "joint", psen->objname);
      break;
    case mjSENS_TENDONLIMITPOS:
      elem = InsertEnd(section, "tendonlimitpos");
      WriteAttrTxt(elem, "tendon", psen->objname);
      break;
    case mjSENS_TENDONLIMITVEL:
      elem = InsertEnd(section, "tendonlimitvel");
      WriteAttrTxt(elem, "tendon", psen->objname);
      break;
    case mjSENS_TENDONLIMITFRC:
      elem = InsertEnd(section, "tendonlimitfrc");
      WriteAttrTxt(elem, "tendon", psen->objname);
      break;

    // sensors attached to an object with spatial frame: (x)body, geom, site, camera
    case mjSENS_FRAMEPOS:
      elem = InsertEnd(section, "framepos");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
      break;
    case mjSENS_FRAMEQUAT:
      elem = InsertEnd(section, "framequat");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
      break;
    case mjSENS_FRAMEXAXIS:
      elem = InsertEnd(section, "framexaxis");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
      break;
    case mjSENS_FRAMEYAXIS:
      elem = InsertEnd(section, "frameyaxis");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
      break;
    case mjSENS_FRAMEZAXIS:
      elem = InsertEnd(section, "framezaxis");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
      break;
    case mjSENS_FRAMELINVEL:
      elem = InsertEnd(section, "framelinvel");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
      break;
    case mjSENS_FRAMEANGVEL:
      elem = InsertEnd(section, "frameangvel");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
      break;
    case mjSENS_FRAMELINACC:
      elem = InsertEnd(section, "framelinacc");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
      break;
    case mjSENS_FRAMEANGACC:
      elem = InsertEnd(section, "frameangacc");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
      break;

    // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    case mjSENS_SUBTREECOM:
      elem = InsertEnd(section, "subtreecom");
      WriteAttrTxt(elem, "body", psen->objname);
      break;
    case mjSENS_SUBTREELINVEL:
      elem = InsertEnd(section, "subtreelinvel");
      WriteAttrTxt(elem, "body", psen->objname);
      break;
    case mjSENS_SUBTREEANGMOM:
      elem = InsertEnd(section, "subtreeangmom");
      WriteAttrTxt(elem, "body", psen->objname);
      break;

    // global sensors
    case mjSENS_CLOCK:
      elem = InsertEnd(section, "clock");
      break;

    // user-defined sensor
    case mjSENS_USER:
      elem = InsertEnd(section, "user");
      WriteAttrTxt(elem, "objtype", mju_type2Str(psen->objtype));
      WriteAttrTxt(elem, "objname", psen->objname);
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
    WriteAttr(elem, "noise", 1, &psen->noise, &zero);
    WriteVector(elem, "user", psen->userdata);

    // add reference if present
    if (psen->reftype > 0) {
      WriteAttrTxt(elem, "reftype", mju_type2Str(psen->reftype));
      WriteAttrTxt(elem, "refname", psen->refname);
    }
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

    mjCKey* pk = model->keys[i];

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
      if (pk->qpos[j]!=model->qpos0[j]) {
        WriteAttr(elem, "qpos", model->nq, pk->qpos.data());
        change = true;
        break;
      }
    }

    // check qvel and write
    for (int j=0; j<model->nv; j++) {
      if (pk->qvel[j]!=0) {
        WriteAttr(elem, "qvel", model->nv, pk->qvel.data());
        change = true;
        break;
      }
    }

    // check act and write
    for (int j=0; j<model->na; j++) {
      if (pk->act[j]!=0) {
        WriteAttr(elem, "act", model->na, pk->act.data());
        change = true;
        break;
      }
    }

    // check mpos and write
    if (model->nmocap) {
      for (int j=0; j<model->nbody; j++) {
        if (model->bodies[j]->mocap) {
          mjCBody* pb = model->bodies[j];
          int id = pb->mocapid;
          if (pb->locpos[0] != pk->mpos[3*id] ||
              pb->locpos[1] != pk->mpos[3*id+1] ||
              pb->locpos[2] != pk->mpos[3*id+2]) {
            WriteAttr(elem, "mpos", 3*model->nmocap, pk->mpos.data());
            change = true;
            break;
          }
        }
      }
    }

    // check mquat and write
    if (model->nmocap) {
      for (int j=0; j<model->nbody; j++) {
        if (model->bodies[j]->mocap) {
          mjCBody* pb = model->bodies[j];
          int id = pb->mocapid;
          if (pb->locquat[0] != pk->mquat[4*id] ||
              pb->locquat[1] != pk->mquat[4*id+1] ||
              pb->locquat[2] != pk->mquat[4*id+2] ||
              pb->locquat[3] != pk->mquat[4*id+3]) {
            WriteAttr(elem, "mquat", 4*model->nmocap, pk->mquat.data());
            change = true;
            break;
          }
        }
      }
    }

    // check ctrl and write
    for (int j=0; j<model->nu; j++) {
      if (pk->ctrl[j]!=0) {
        WriteAttr(elem, "ctrl", model->nu, pk->ctrl.data());
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
