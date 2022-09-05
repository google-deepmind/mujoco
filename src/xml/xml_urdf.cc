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

#include <cstring>
#include <string>
#include <vector>

#include <mujoco/mjmodel.h>
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_util.h"
#include "xml/xml_native_reader.h"
#include "xml/xml_urdf.h"

#include "tinyxml2.h"

using tinyxml2::XMLElement;

// URDF joint type
static const int urJoint_sz = 6;
static const mjMap urJoint_map[urJoint_sz] = {
  {"revolute",    0},
  {"continuous",  1},
  {"prismatic",   2},
  {"fixed",       3},
  {"floating",    4},
  {"planar",      5}
};



//---------------------------------- class mjXURDF -------------------------------------------------

// constructor
mjXURDF::mjXURDF() {
  Clear();
}



// destructor
mjXURDF::~mjXURDF() {
  Clear();
}


// clear internal variables
void mjXURDF::Clear(void) {
  model = 0;

  urName.clear();
  urParent.clear();
  urChildren.clear();
  urMat.clear();
  urRGBA.clear();
}



// actual parser
void mjXURDF::Parse(XMLElement* root) {
  std::string name, text;
  XMLElement *elem, *temp;
  int id_parent, id_child, i;

  // set compiler defaults suitable for URDF
  model->strippath = true;
  model->discardvisual = true;
  model->fusestatic = true;

  // parse MuJoCo sections (not part of URDF)
  XMLElement* mjc = FindSubElem(root, "mujoco");
  if (mjc) {
    XMLElement *section;
    if ((section = FindSubElem(mjc, "compiler"))) {
      mjXReader::Compiler(section, model);
    }

    if ((section = FindSubElem(mjc, "option"))) {
      mjXReader::Option(section, &model->option);
    }

    if ((section = FindSubElem(mjc, "size"))) {
      mjXReader::Size(section, model);
    }
  }

  // enfore required compiler defaults for URDF
  model->global = false;
  model->degree = false;

  // get model name
  ReadAttrTxt(root, "name", model->modelname);

  // find and register all materials
  MakeMaterials(root);

  // find all links/bodies, save names
  elem = root->FirstChildElement();
  while (elem) {
    // identify link elements
    name = elem->Value();
    if (name=="link") {
      ReadAttrTxt(elem, "name", text, true);
      AddBody(text);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }

  // find all joints, assign parent and child pointers
  elem = root->FirstChildElement();
  while (elem) {
    // identify joint elements
    name = elem->Value();
    if (name=="joint") {
      // find parent, get name and id
      temp = FindSubElem(elem, "parent", true);
      ReadAttrTxt(temp, "link", text, true);
      id_parent = FindName(text, urName);

      // find child, get name and id
      temp = FindSubElem(elem, "child", true);
      ReadAttrTxt(temp, "link", text, true);
      id_child = FindName(text, urName);

      // make sure parent and child exist
      if (id_parent<0 || id_child<0) {
        throw mjXError(elem, "URDF joint parent or child missing");
      }

      // check for multiple parents
      if (urParent[id_child]>=0) {
        throw mjXError(elem, "URDF body has multiple parents:", urName[id_child].c_str());
      }

      // add parent and child info
      urParent[id_child] = id_parent;
      urChildren[id_parent].push_back(id_child);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }

  // find all top-level bodies, call recursive tree constructor
  for (i=0; i<(int)urName.size(); i++) {
    if (urParent[i] < 0) {
      AddToTree(i);
    }
  }

  // parse bodies
  elem = root->FirstChildElement();
  while (elem) {
    // identify body/link elements
    name = elem->Value();
    if (name=="link") {
      Body(elem);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }

  // parse joints
  elem = root->FirstChildElement();
  while (elem) {
    // identify body/link elements
    name = elem->Value();
    if (name=="joint") {
      Joint(elem);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// parse body/link
void mjXURDF::Body(XMLElement* body_elem) {
  std::string name, text;
  XMLElement *elem, *temp, *temp1;
  mjCBody* pbody;
  mjCGeom* pgeom;

  // get body name and pointer to mjCBody
  ReadAttrTxt(body_elem, "name", name, true);
  pbody = (mjCBody*) model->GetWorld()->FindObject(mjOBJ_BODY, name);
  if (!pbody) {
    throw mjXError(body_elem, "URDF body not found");  // SHOULD NOT OCCUR
  }

  // inertial element: copy into alternative body frame
  if ((elem = FindSubElem(body_elem, "inertial"))) {
    pbody->explicitinertial = true;
    // origin- relative to joint frame for now
    Origin(elem, pbody->ipos, pbody->iquat);

    // mass
    temp = FindSubElem(elem, "mass", true);
    ReadAttr(temp, "value", 1, &pbody->mass, text, true);

    // inertia
    temp = FindSubElem(elem, "inertia", true);
    mjCAlternative alt;
    ReadAttr(temp, "ixx", 1, alt.fullinertia+0, text, true);
    ReadAttr(temp, "iyy", 1, alt.fullinertia+1, text, true);
    ReadAttr(temp, "izz", 1, alt.fullinertia+2, text, true);
    ReadAttr(temp, "ixy", 1, alt.fullinertia+3, text, true);
    ReadAttr(temp, "ixz", 1, alt.fullinertia+4, text, true);
    ReadAttr(temp, "iyz", 1, alt.fullinertia+5, text, true);

    // process inertia
    //  lquat = rotation from specified to default (joint/body) inertial frame
    double lquat[4], tmpquat[4];
    const char* altres =
      alt.Set(lquat, pbody->inertia, model->degree, model->euler);

    // inertia are sometimes 0 in URDF files: ignore error in altres, fix later
    (void) altres;

    // correct for alignment of full inertia matrix
    mjuu_mulquat(tmpquat, pbody->iquat, lquat);
    mjuu_copyvec(pbody->iquat, tmpquat, 4);
  }

  // clear body frame; set by joint later
  mjuu_setvec(pbody->pos, 0, 0, 0);
  mjuu_setvec(pbody->quat, 1, 0, 0, 0);

  // process all visual and geometry elements in order
  float rgba[4] = {-1, 0, 0, 0};
  elem = body_elem->FirstChildElement();
  while (elem) {
    name = elem->Value();

    // visual element
    if (name=="visual") {
      // parse material
      if ((temp = FindSubElem(elem, "material"))) {
        // if color specified - use directly
        if ((temp1 = FindSubElem(temp, "color"))) {
          ReadAttr(temp1, "rgba", 4, rgba, text, /*required=*/true);
        }

        // otherwise use material table
        else {
          ReadAttrTxt(temp, "name", name, true);
          int imat = FindName(name, urMat);
          if (imat>=0) {
            std::memcpy(rgba, urRGBA[imat].val, 4*sizeof(float));
          }
        }
      }

      // create geom if not discarded
      if (!model->discardvisual) {
        pgeom = Geom(elem, pbody, false);

        // save color
        if (rgba[0]>=0) {
          std::memcpy(pgeom->rgba, rgba, 4*sizeof(float));
        }
      }
    }

    // collision element
    else if (name=="collision") {
      pgeom = Geom(elem, pbody, true);

      // use color from last visual
      if (rgba[0]>=0) {
        std::memcpy(pgeom->rgba, rgba, 4*sizeof(float));
      }
    }

    // advance
    elem = elem->NextSiblingElement();
  }
}



// parse joint
void mjXURDF::Joint(XMLElement* joint_elem) {
  std::string jntname, name, text;
  XMLElement *elem;
  mjCBody *pbody, *parent;
  mjCJoint *pjoint=0, *pjoint1=0, *pjoint2=0;
  int jointtype;

  // get type and name
  ReadAttrTxt(joint_elem, "type", text, true);
  jointtype = FindKey(urJoint_map, urJoint_sz, text);
  if (jointtype < 0) {
    mjXError(joint_elem, "invalid joint type in URDF joint definition");
  }
  ReadAttrTxt(joint_elem, "name", jntname, true);

  // get parent, check
  elem = FindSubElem(joint_elem, "parent", true);
  ReadAttrTxt(elem, "link", name, true);
  parent = (mjCBody*) model->GetWorld()->FindObject(mjOBJ_BODY, name);
  if (!parent) {                      // SHOULD NOT OCCUR
    mjXError(elem, "invalid parent name in URDF joint definition");
  }

  // get child=this, check
  elem = FindSubElem(joint_elem, "child", true);
  ReadAttrTxt(elem, "link", name, true);
  pbody = (mjCBody*) model->GetWorld()->FindObject(mjOBJ_BODY, name);
  if (!pbody) {                       // SHOULD NOT OCCUR
    throw mjXError(elem, "invalid child name in URDF joint definition");
  }

  // read origin and axis
  double axis[3] = {1, 0, 0};
  Origin(joint_elem, pbody->pos, pbody->quat);
  if ((elem = FindSubElem(joint_elem, "axis"))) {
    ReadAttr(elem, "xyz", 3, axis, text, /*required=*/true);
  }

  // create joint (unless fixed)
  double mat[9], quat[4], tmpaxis[3];
  switch (jointtype) {
  case 0:     // revolute
  case 1:     // continuous
    pjoint = pbody->AddJoint();
    pjoint->name = jntname;
    pjoint->type = mjJNT_HINGE;
    mjuu_setvec(pjoint->pos, 0, 0, 0);
    mjuu_copyvec(pjoint->axis, axis, 3);
    break;

  case 2:     // prismatic
    pjoint = pbody->AddJoint();
    pjoint->name = jntname;
    pjoint->type = mjJNT_SLIDE;
    mjuu_setvec(pjoint->pos, 0, 0, 0);
    mjuu_copyvec(pjoint->axis, axis, 3);
    break;

  case 3:     // fixed- no joint, return
    return;

  case 4:     // floating
    pjoint = pbody->AddJoint();
    pjoint->name = jntname;
    pjoint->type = mjJNT_FREE;
    break;

  case 5:     // planar- construct complex joint
    // make frame with axis = z
    mjuu_z2quat(quat, axis);
    mjuu_quat2mat(mat, quat);

    // construct slider along x
    pjoint = pbody->AddJoint();
    pjoint->name = jntname + "_TX";
    pjoint->type = mjJNT_SLIDE;
    tmpaxis[0] = mat[0];
    tmpaxis[1] = mat[3];
    tmpaxis[2] = mat[6];
    mjuu_setvec(pjoint->pos, 0, 0, 0);
    mjuu_copyvec(pjoint->axis, tmpaxis, 3);

    // construct slider along y
    pjoint1 = pbody->AddJoint();
    pjoint1->name = jntname + "_TY";
    pjoint1->type = mjJNT_SLIDE;
    tmpaxis[0] = mat[1];
    tmpaxis[1] = mat[4];
    tmpaxis[2] = mat[7];
    mjuu_setvec(pjoint1->pos, 0, 0, 0);
    mjuu_copyvec(pjoint1->axis, tmpaxis, 3);

    // construct hinge around z = locaxis
    pjoint2 = pbody->AddJoint();
    pjoint2->name = jntname + "_RZ";
    pjoint2->type = mjJNT_HINGE;
    mjuu_setvec(pjoint2->pos, 0, 0, 0);
    mjuu_copyvec(pjoint2->axis, axis, 3);
  }

  // dynamics element
  if ((elem = FindSubElem(joint_elem, "dynamics"))) {
    ReadAttr(elem, "damping", 1, &pjoint->damping, text);
    ReadAttr(elem, "friction", 1, &pjoint->frictionloss, text);

    // copy parameters to all elements of planar joint
    if (pjoint1) {
      pjoint1->damping = pjoint2->damping = pjoint->damping;
      pjoint1->frictionloss = pjoint2->frictionloss = pjoint->frictionloss;
    }
  }

  // limit element
  if ((elem = FindSubElem(joint_elem, "limit"))) {
    ReadAttr(elem, "lower", 1, pjoint->range, text);
    ReadAttr(elem, "upper", 1, pjoint->range+1, text);
    pjoint->limited = (mjuu_defined(pjoint->range[0]) &&
                       mjuu_defined(pjoint->range[1]) &&
                       pjoint->range[0] < pjoint->range[1]);

    // ReadAttr(elem, "velocity", 1, &pjoint->maxvel, text); // no maxvel in MuJoCo
    ReadAttr(elem, "effort", 1, &pjoint->urdfeffort, text);
  } else {
    pjoint->limited = 0;
  }
}



// parse origin and geometry elements of visual or collision
mjCGeom* mjXURDF::Geom(XMLElement* geom_elem, mjCBody* pbody, bool collision) {
  XMLElement *elem, *temp;
  std::string text, meshfile;

  // get geometry element
  elem = FindSubElem(geom_elem, "geometry", true);

  // add BOX geom, modify type later
  mjCGeom* pgeom = pbody->AddGeom();
  pgeom->name = "";
  pgeom->type = mjGEOM_BOX;
  if (collision) {
    pgeom->contype = 1;
    pgeom->conaffinity = 1;
  } else {
    pgeom->contype = 0;
    pgeom->conaffinity = 0;
    pgeom->group = 1;
    pgeom->density = 0;
  }

  // box
  if ((temp = FindSubElem(elem, "box"))) {
    ReadAttr(temp, "size", 3, pgeom->size, text, true, true);
    for (int i=0; i<3; i++) {
      pgeom->size[i] /= 2;  // MuJoCo uses half-length
    }
  }

  // cylinder
  else if ((temp = FindSubElem(elem, "cylinder"))) {
    pgeom->type = mjGEOM_CYLINDER;
    ReadAttr(temp, "radius", 1, pgeom->size, text, true, true);
    ReadAttr(temp, "length", 1, pgeom->size+1, text, true, true);
    pgeom->size[1] /= 2;            // MuJoCo uses half-length
  }

  // sphere
  else if ((temp = FindSubElem(elem, "sphere"))) {
    pgeom->type = mjGEOM_SPHERE;
    ReadAttr(temp, "radius", 1, pgeom->size, text, true, true);
  }

  // mesh
  else if ((temp = FindSubElem(elem, "mesh"))) {
    // set geom type and read mesh attributes
    double meshscale[3] = {1, 1, 1};
    pgeom->type = mjGEOM_MESH;
    ReadAttrTxt(temp, "filename", meshfile, true);
    ReadAttr(temp, "scale", 3, meshscale, text);

    // strip file name if necessary
    if (model->strippath) {
      meshfile = mjuu_strippath(meshfile);
    }

    // construct mesh name: always stripped
    std::string meshname = mjuu_strippath(meshfile);
    meshname = mjuu_stripext(meshname);

    // look for existing mesh
    mjCMesh* pmesh = (mjCMesh*)model->FindObject(mjOBJ_MESH, meshname);

    // does not exist: create
    if (!pmesh) {
      pmesh = model->AddMesh();
    }

    // exists with different scale: append name with '1', create
    else if (pmesh->scale[0]!=meshscale[0] ||
             pmesh->scale[1]!=meshscale[1] ||
             pmesh->scale[2]!=meshscale[2]) {
      pmesh = model->AddMesh();
      meshname = meshname + "1";
    }

    // set fields
    pmesh->file = meshfile;
    pmesh->name = meshname;
    pgeom->mesh = meshname;
    mjuu_copyvec(pmesh->scale, meshscale, 3);
  }

  else {
    throw mjXError(elem, "visual geometry specification not found");
  }

  // origin element
  Origin(geom_elem, pgeom->pos, pgeom->quat);

  return pgeom;
}



// parse origin element
void mjXURDF::Origin(XMLElement* origin_elem, double* pos, double* quat) {
  XMLElement* temp;
  std::string text;

  // set defaults
  mjuu_setvec(pos, 0, 0, 0);
  mjuu_setvec(quat, 1, 0, 0, 0);

  // read origin element if present
  if ((temp = FindSubElem(origin_elem, "origin"))) {
    // position
    ReadAttr(temp, "xyz", 3, pos, text);

    // orientation
    mjCAlternative alt;
    if (ReadAttr(temp, "rpy", 3, alt.euler, text)) {
      alt.Set(quat, 0, 0, "XYZ");
    }
  }
}



// find body with given name in list, return -1 if not found
int mjXURDF::FindName(std::string name, std::vector<std::string>& list) {
  for (unsigned int i=0; i<list.size(); i++)
    if (list[i] == name) {
      return i;
    }

  return -1;
}



// add name to list, error if name already exists
void mjXURDF::AddName(std::string name, std::vector<std::string>& list) {
  // make sure name is unique
  if (FindName(name, list)>=0) {
    throw mjXError(0, "repeated URDF name: ", name.c_str());
  }

  list.push_back(name);
}



// add body name to list of URDF bodies, error if name already exists
void mjXURDF::AddBody(std::string name) {
  // add body name, make sure it is unique
  AddName(name, urName);

  // add parent and child elements
  urParent.push_back(-1);
  std::vector<int> children;
  children.clear();
  urChildren.push_back(children);
}



// add body with given number to the mjCModel tree, process children
void mjXURDF::AddToTree(int n) {
  // get pointer to parent in mjCModel tree
  mjCBody *parent = 0, *child = 0;
  if (urParent[n]>=0) {
    parent = (mjCBody*) model->GetWorld()->FindObject(mjOBJ_BODY, urName[urParent[n]]);

    if (!parent)
      throw mjXError(0, "URDF body parent should already be in tree: %s",
                     urName[urParent[n]].c_str());       // SHOULD NOT OCCUR
  } else {
    parent = model->GetWorld();
  }

  // add this body
  if (urName[n] != "world") {
    child = parent->AddBody();
    child->name = urName[n];
  }

  // add children recursively
  for (int i=0; i<(int)urChildren[n].size(); i++) {
    AddToTree(urChildren[n][i]);
  }
}



// find all materials recursively
void mjXURDF::MakeMaterials(XMLElement* elem) {
  std::string name, text;
  XMLElement* color = 0;
  mjRGBA rgba;

  // process this element
  if (!std::strcmp(elem->Value(), "material")) {
    // make sure material is named
    if (ReadAttrTxt(elem, "name", name)) {
      // make sure name is not already registered
      if (FindName(name, urMat) < 0) {
        // add rgba value if available
        if ((color = FindSubElem(elem, "color"))) {
          ReadAttr(color, "rgba", 4, rgba.val, text, /*required=*/true);
          AddName(name, urMat);
          urRGBA.push_back(rgba);
        }
      }
    }
  }

  // process children recursively
  elem = elem->FirstChildElement();
  while (elem) {
    MakeMaterials(elem);
    elem = elem->NextSiblingElement();
  }
}
