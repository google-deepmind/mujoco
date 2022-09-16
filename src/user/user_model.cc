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

#include "user/user_model.h"

#include <algorithm>
#include <csetjmp>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include "cc/array_safety.h"
#include "engine/engine_forward.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_setconst.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_objects.h"
#include "user/user_util.h"

namespace {
namespace mju = ::mujoco::util;
using std::string;
using std::vector;
}  // namespace

// pthread on Linux, std::thread on Mac and Windows
#if defined(__APPLE__) || defined(_WIN32)
  #include <thread>
  using std::thread;

  int getnumproc(void) {
    return thread::hardware_concurrency();
  }
#else
  #include <pthread.h>
  #include <sys/sysinfo.h>

  int getnumproc(void) {
    return get_nprocs();
  }
#endif


// copy real-valued vector
template <class T1, class T2>
static void copyvec(T1* dest, T2* src, int n) {
  for (int i=0; i<n; i++) {
    dest[i] = (T1)src[i];
  }
}



//---------------------------------- CONSTRUCTOR AND DESTRUCTOR ------------------------------------

// constructor
mjCModel::mjCModel() {
  comment.clear();
  modelfiledir.clear();

  //------------------------ compiler settings
  // TODO(b/245077553): Toggle to true by default.
  autolimits = false;
  boundmass = 0;
  boundinertia = 0;
  settotalmass = -1;
  balanceinertia = false;
  strippath = false;
  fitaabb = false;
  global = false;
  degree = true;
  euler[0] = 'x';
  euler[1] = 'y';
  euler[2] = 'z';
  meshdir.clear();
  texturedir.clear();
  discardvisual = false;
  convexhull = true;
  usethread = true;
  fusestatic = false;
  inertiafromgeom = mjINERTIAFROMGEOM_AUTO;
  inertiagrouprange[0] = 0;
  inertiagrouprange[1] = mjNGROUP-1;
  exactmeshinertia = false;
  mj_defaultLROpt(&LRopt);

  //------------------------ statistics override
  meaninertia = mjNAN;
  meanmass = mjNAN;
  meansize = mjNAN;
  extent = mjNAN;
  center[0] = mjNAN;
  center[1] = center[2] = 0;

  //------------------------ engine data
  modelname = "MuJoCo Model";
  mj_defaultOption(&option);
  mj_defaultVisual(&visual);
  nemax = 0;
  njmax = -1;
  nconmax = -1;
  nstack = -1;
  nuserdata = 0;
  nkey = 0;
  nmocap = 0;
  nuser_body = -1;
  nuser_jnt = -1;
  nuser_geom = -1;
  nuser_site = -1;
  nuser_cam = -1;
  nuser_tendon = -1;
  nuser_actuator = -1;
  nuser_sensor = -1;

  //------------------------ private variables
  cameras.clear();
  lights.clear();
  meshes.clear();
  skins.clear();
  hfields.clear();
  textures.clear();
  materials.clear();
  pairs.clear();
  excludes.clear();
  equalities.clear();
  tendons.clear();
  actuators.clear();
  numerics.clear();
  texts.clear();
  tuples.clear();
  keys.clear();
  defaults.clear();
  Clear();

  //------------------------ master default set
  defaults.push_back(new mjCDef);

  //------------------------ world body
  mjCBody* world = new mjCBody(this);
  mjuu_zerovec(world->pos, 3);
  mjuu_zerovec(world->locpos, 3);
  mjuu_zerovec(world->locipos, 3);
  mjuu_setvec(world->quat, 1, 0, 0, 0);
  mjuu_setvec(world->locquat, 1, 0, 0, 0);
  mjuu_setvec(world->lociquat, 1, 0, 0, 0);
  world->mass = 0;
  mjuu_zerovec(world->inertia, 3);
  world->id = 0;
  world->parentid = 0;
  world->weldid = 0;
  world->name = "world";
  world->def = defaults[0];
  bodies.push_back(world);
}



// destructor
mjCModel::~mjCModel() {
  unsigned int i;

  // delete kinematic tree and all objects allocated in it
  delete bodies[0];

  // delete objects allocated in mjCModel
  for (i=0; i<meshes.size(); i++) delete meshes[i];
  for (i=0; i<skins.size(); i++) delete skins[i];
  for (i=0; i<hfields.size(); i++) delete hfields[i];
  for (i=0; i<textures.size(); i++) delete textures[i];
  for (i=0; i<materials.size(); i++) delete materials[i];
  for (i=0; i<pairs.size(); i++) delete pairs[i];
  for (i=0; i<excludes.size(); i++) delete excludes[i];
  for (i=0; i<equalities.size(); i++) delete equalities[i];
  for (i=0; i<tendons.size(); i++) delete tendons[i];  // also deletes wraps
  for (i=0; i<actuators.size(); i++) delete actuators[i];
  for (i=0; i<sensors.size(); i++) delete sensors[i];
  for (i=0; i<numerics.size(); i++) delete numerics[i];
  for (i=0; i<texts.size(); i++) delete texts[i];
  for (i=0; i<tuples.size(); i++) delete tuples[i];
  for (i=0; i<keys.size(); i++) delete keys[i];
  for (i=0; i<defaults.size(); i++) delete defaults[i];

  // clear pointer lists created in model construction
  meshes.clear();
  skins.clear();
  hfields.clear();
  textures.clear();
  materials.clear();
  pairs.clear();
  excludes.clear();
  equalities.clear();
  tendons.clear();
  actuators.clear();
  sensors.clear();
  numerics.clear();
  texts.clear();
  tuples.clear();
  keys.clear();
  defaults.clear();

  // clear sizes and pointer lists created in Compile
  Clear();
}



// clear objects allocated by Compile
void mjCModel::Clear(void) {
  // sizes set from list lengths
  nbody = 0;
  njnt = 0;
  ngeom = 0;
  nsite = 0;
  ncam = 0;
  nlight = 0;
  nmesh = 0;
  nskin = 0;
  nhfield = 0;
  ntex = 0;
  nmat = 0;
  npair = 0;
  nexclude = 0;
  neq = 0;
  ntendon = 0;
  nsensor = 0;
  nnumeric = 0;
  ntext = 0;

  // sizes set by Compile
  nq = 0;
  nv = 0;
  nu = 0;
  na = 0;
  nmeshvert = 0;
  nmeshtexvert = 0;
  nmeshface = 0;
  nmeshgraph = 0;
  nskinvert = 0;
  nskintexvert = 0;
  nskinface = 0;
  nskinbone = 0;
  nskinbonevert = 0;
  nhfielddata = 0;
  ntexdata = 0;
  nwrap = 0;
  nsensordata = 0;
  nnumericdata = 0;
  ntextdata = 0;
  ntupledata = 0;
  nnames = 0;
  nemax = 0;
  nM = 0;
  nD = 0;
  njmax = -1;
  nconmax = -1;

  // pointer lists created by Compile
  bodies.clear();
  joints.clear();
  geoms.clear();
  sites.clear();
  cameras.clear();
  lights.clear();

  // internal variables
  compiled = false;
  errInfo = mjCError();
  fixCount = 0;
  qpos0.clear();
}



//------------------------ API FOR ADDING MODEL ELEMENTS -------------------------------------------

// add object of any type
template <class T>
T* mjCModel::AddObject(vector<T*>& list, string type) {
  T* obj = new T(this);
  obj->id = (int)list.size();
  list.push_back(obj);
  return obj;
}


// add object of any type, with def parameter
template <class T>
T* mjCModel::AddObjectDef(vector<T*>& list, string type, mjCDef* def) {
  T* obj = new T(this, def ? def : defaults[0]);
  obj->id = (int)list.size();
  obj->def = def ? def : defaults[0];
  list.push_back(obj);
  return obj;
}


// add mesh
mjCMesh* mjCModel::AddMesh(mjCDef* def) {
  return AddObjectDef(meshes, "mesh", def);
}


// add skin
mjCSkin* mjCModel::AddSkin(void) {
  return AddObject(skins, "skin");
}


// add hfield
mjCHField* mjCModel::AddHField(void) {
  return AddObject(hfields, "hfield");
}


// add texture
mjCTexture* mjCModel::AddTexture(void) {
  return AddObject(textures, "texture");
}


// add material
mjCMaterial* mjCModel::AddMaterial(mjCDef* def) {
  return AddObjectDef(materials, "material", def);
}


// add geom pair to include in collisions
mjCPair* mjCModel::AddPair(mjCDef* def) {
  return AddObjectDef(pairs, "pair", def);
}


// add body pair to exclude from collisions
mjCBodyPair* mjCModel::AddExclude(void) {
  return AddObject(excludes, "exclude");
}


// add constraint
mjCEquality* mjCModel::AddEquality(mjCDef* def) {
  return AddObjectDef(equalities, "equality", def);
}


// add tendon
mjCTendon* mjCModel::AddTendon(mjCDef* def) {
  return AddObjectDef(tendons, "tendon", def);
}


// add actuator
mjCActuator* mjCModel::AddActuator(mjCDef* def) {
  return AddObjectDef(actuators, "actuator", def);
}


// add sensor
mjCSensor* mjCModel::AddSensor(void) {
  return AddObject(sensors, "sensor");
}



// add custom
mjCNumeric* mjCModel::AddNumeric(void) {
  return AddObject(numerics, "numeric");
}


// add text
mjCText* mjCModel::AddText(void) {
  return AddObject(texts, "text");
}


// add tuple
mjCTuple* mjCModel::AddTuple(void) {
  return AddObject(tuples, "tuple");
}


// add keyframe
mjCKey* mjCModel::AddKey(void) {
  return AddObject(keys, "key");
}




//------------------------ API FOR ACCESS TO MODEL ELEMENTS  ---------------------------------------

// get number of objects of specified type
int mjCModel::NumObjects(mjtObj type) {
  switch (type) {
  case mjOBJ_BODY:
  case mjOBJ_XBODY:
    return (int)bodies.size();
  case mjOBJ_JOINT:
    return (int)joints.size();
  case mjOBJ_GEOM:
    return (int)geoms.size();
  case mjOBJ_SITE:
    return (int)sites.size();
  case mjOBJ_CAMERA:
    return (int)cameras.size();
  case mjOBJ_LIGHT:
    return (int)lights.size();
  case mjOBJ_MESH:
    return (int)meshes.size();
  case mjOBJ_SKIN:
    return (int)skins.size();
  case mjOBJ_HFIELD:
    return (int)hfields.size();
  case mjOBJ_TEXTURE:
    return (int)textures.size();
  case mjOBJ_MATERIAL:
    return (int)materials.size();
  case mjOBJ_PAIR:
    return (int)pairs.size();
  case mjOBJ_EXCLUDE:
    return (int)excludes.size();
  case mjOBJ_EQUALITY:
    return (int)equalities.size();
  case mjOBJ_TENDON:
    return (int)tendons.size();
  case mjOBJ_ACTUATOR:
    return (int)actuators.size();
  case mjOBJ_SENSOR:
    return (int)sensors.size();
  case mjOBJ_NUMERIC:
    return (int)numerics.size();
  case mjOBJ_TEXT:
    return (int)texts.size();
  case mjOBJ_TUPLE:
    return (int)tuples.size();
  case mjOBJ_KEY:
    return (int)keys.size();
  default:
    return 0;
  }
}



// get poiner to specified object
mjCBase* mjCModel::GetObject(mjtObj type, int id) {
  if (id>=0 && id<NumObjects(type)) {
    switch (type) {
    case mjOBJ_BODY:
    case mjOBJ_XBODY:
      return bodies[id];
    case mjOBJ_JOINT:
      return joints[id];
    case mjOBJ_GEOM:
      return geoms[id];
    case mjOBJ_SITE:
      return sites[id];
    case mjOBJ_CAMERA:
      return cameras[id];
    case mjOBJ_LIGHT:
      return lights[id];
    case mjOBJ_MESH:
      return meshes[id];
    case mjOBJ_SKIN:
      return skins[id];
    case mjOBJ_HFIELD:
      return hfields[id];
    case mjOBJ_TEXTURE:
      return textures[id];
    case mjOBJ_MATERIAL:
      return materials[id];
    case mjOBJ_PAIR:
      return pairs[id];
    case mjOBJ_EXCLUDE:
      return excludes[id];
    case mjOBJ_EQUALITY:
      return equalities[id];
    case mjOBJ_TENDON:
      return tendons[id];
    case mjOBJ_ACTUATOR:
      return actuators[id];
    case mjOBJ_SENSOR:
      return sensors[id];
    case mjOBJ_NUMERIC:
      return numerics[id];
    case mjOBJ_TEXT:
      return texts[id];
    case mjOBJ_TUPLE:
      return tuples[id];
    case mjOBJ_KEY:
      return keys[id];
    default:
      return 0;
    }
  }

  return 0;
}



//------------------------ API FOR ACCESS TO PRIVATE VARIABLES -------------------------------------

// compiled flag
bool mjCModel::IsCompiled(void) {
  return compiled;
}



// number of massless bodies that were fixed
int mjCModel::GetFixed(void) {
  return fixCount;
}



// copy of error object
mjCError mjCModel::GetError(void) {
  return errInfo;
}



// pointer to world body
mjCBody* mjCModel::GetWorld(void) {
  return bodies[0];
}



// find default class name in array
mjCDef* mjCModel::FindDef(string name) {
  for (int i=0; i<(int)defaults.size(); i++) {
    if (defaults[i]->name==name) {
      return defaults[i];
    }
  }

  return 0;
}



// add default class to array
mjCDef* mjCModel::AddDef(string name, int parentid) {
  // check for repeated name
  int thisid = (int)defaults.size();
  for (int i=0; i<thisid; i++) {
    if (defaults[i]->name==name) {
      return 0;
    }
  }

  // create new object
  mjCDef* def = new mjCDef;
  defaults.push_back(def);

  // initialize contents
  if (parentid>=0 && parentid<thisid) {
    *def = *defaults[parentid];
    defaults[parentid]->childid.push_back(thisid);
  }
  def->parentid = parentid;
  def->name = name;
  def->childid.clear();

  return def;
}



// find object by name in given list
template <class T>
static T* findobject(string name, vector<T*>& list) {
  for (unsigned int i=0; i<list.size(); i++) {
    if (list[i]->name == name) {
      return list[i];
    }
  }

  return 0;
}

// find object in global lists given string type and name
mjCBase* mjCModel::FindObject(mjtObj type, string name) {
  switch (type) {
  case mjOBJ_BODY:
  case mjOBJ_XBODY:
    return findobject(name, bodies);
  case mjOBJ_JOINT:
    return findobject(name, joints);
  case mjOBJ_GEOM:
    return findobject(name, geoms);
  case mjOBJ_SITE:
    return findobject(name, sites);
  case mjOBJ_CAMERA:
    return findobject(name, cameras);
  case mjOBJ_LIGHT:
    return findobject(name, lights);
  case mjOBJ_MESH:
    return findobject(name, meshes);
  case mjOBJ_SKIN:
    return findobject(name, skins);
  case mjOBJ_HFIELD:
    return findobject(name, hfields);
  case mjOBJ_TEXTURE:
    return findobject(name, textures);
  case mjOBJ_MATERIAL:
    return findobject(name, materials);
  case mjOBJ_PAIR:
    return findobject(name, pairs);
  case mjOBJ_EXCLUDE:
    return findobject(name, excludes);
  case mjOBJ_EQUALITY:
    return findobject(name, equalities);
  case mjOBJ_TENDON:
    return findobject(name, tendons);
  case mjOBJ_ACTUATOR:
    return findobject(name, actuators);
  case mjOBJ_SENSOR:
    return findobject(name, sensors);
  case mjOBJ_NUMERIC:
    return findobject(name, numerics);
  case mjOBJ_TEXT:
    return findobject(name, texts);
  case mjOBJ_TUPLE:
    return findobject(name, tuples);
  default:
    return 0;
  }
}



// detect null pose
bool mjCModel::IsNullPose(const mjtNum* pos, const mjtNum* quat) {
  bool result = true;

  // check position if given
  if (pos) {
    if (pos[0] || pos[1] || pos[2]) {
      result = false;
    }
  }

  // check orientation if given
  if (quat) {
    if (quat[0]!=1 || quat[1] || quat[2] || quat[3]) {
      result = false;
    }
  }

  return result;
}



//------------------------------- COMPILER PHASES --------------------------------------------------

// make lists of objects in tree: bodies, geoms, joints, sites, cameras, lights
void mjCModel::MakeLists(mjCBody* body) {
  unsigned int i;

  // add this body if not world
  if (body!=bodies[0]) {
    bodies.push_back(body);
  }

  // add body's geoms, joints, sites, cameras, lights
  for (i=0; i<body->geoms.size(); i++) geoms.push_back(body->geoms[i]);
  for (i=0; i<body->joints.size(); i++) joints.push_back(body->joints[i]);
  for (i=0; i<body->sites.size(); i++) sites.push_back(body->sites[i]);
  for (i=0; i<body->cameras.size(); i++) cameras.push_back(body->cameras[i]);
  for (i=0; i<body->lights.size(); i++) lights.push_back(body->lights[i]);

  // recursive call to all child bodies
  for (i=0; i<body->bodies.size(); i++) MakeLists(body->bodies[i]);
}



// index assets
void mjCModel::IndexAssets(void) {
  unsigned int i;

  // assets referenced in geoms
  for (i=0; i<geoms.size(); i++) {
    mjCGeom* pgeom = geoms[i];

    // find material by name
    if (!pgeom->material.empty()) {
      mjCBase* m = FindObject(mjOBJ_MATERIAL, pgeom->material);
      if (m) {
        pgeom->matid = m->id;
      } else {
        throw mjCError(pgeom, "material '%s' not found in geom %d", pgeom->material.c_str(), i);
      }
    }

    // find mesh by name
    if (!pgeom->mesh.empty()) {
      mjCBase* m = FindObject(mjOBJ_MESH, pgeom->mesh);
      if (m) {
        pgeom->meshid = m->id;
      } else {
        throw mjCError(pgeom, "mesh '%s' not found in geom %d", pgeom->mesh.c_str(), i);
      }
    }

    // find hfield by name
    if (!pgeom->hfield.empty()) {
      mjCBase* m = FindObject(mjOBJ_HFIELD, pgeom->hfield);
      if (m) {
        pgeom->hfieldid = m->id;
      } else {
        throw mjCError(pgeom, "hfield '%s' not found in geom %d", pgeom->hfield.c_str(), i);
      }
    }
  }

  // assets referenced in skins
  for (i=0; i<skins.size(); i++) {
    mjCSkin* pskin = skins[i];

    // find material by name
    if (!pskin->material.empty()) {
      mjCBase* m = FindObject(mjOBJ_MATERIAL, pskin->material);
      if (m) {
        pskin->matid = m->id;
      } else {
        throw mjCError(pskin, "material '%s' not found in skin %d", pskin->material.c_str(), i);
      }
    }
  }

  // materials referenced in sites
  for (i=0; i<sites.size(); i++) {
    mjCSite* psite = sites[i];

    // find material by name
    if (!psite->material.empty()) {
      mjCBase* m = FindObject(mjOBJ_MATERIAL, psite->material);
      if (m) {
        psite->matid = m->id;
      } else {
        throw mjCError(psite, "material '%s' not found in site %d", psite->material.c_str(), i);
      }
    }
  }

  // materials referenced in tendons
  for (i=0; i<tendons.size(); i++) {
    mjCTendon* pten = tendons[i];

    // find material by name
    if (!pten->material.empty()) {
      mjCBase* m = FindObject(mjOBJ_MATERIAL, pten->material);
      if (m) {
        pten->matid = m->id;
      } else {
        throw mjCError(pten, "material '%s' not found in tendon %d", pten->material.c_str(), i);
      }
    }
  }

  // textures referenced in materials
  for (i=0; i<materials.size(); i++) {
    mjCMaterial* pmat = materials[i];

    // find texture by name
    if (!pmat->texture.empty()) {
      mjCBase* m = FindObject(mjOBJ_TEXTURE, pmat->texture);
      if (m) {
        pmat->texid = m->id;
      } else {
        throw mjCError(pmat, "texture '%s' not found in material %d", pmat->texture.c_str(), i);
      }
    }
  }
}



// if asset name is missing, set to filename
void mjCModel::SetDefaultNames(void) {
  string stripped;
  unsigned int i;

  // meshes
  for (i=0; i<meshes.size(); i++) {
    if (meshes[i]->name.empty()) {
      stripped = mjuu_strippath(meshes[i]->file);
      meshes[i]->name = mjuu_stripext(stripped);

      // name cannot be empty
      if (meshes[i]->name.empty()) {
        throw mjCError(meshes[i], "empty name in mesh");
      }
    }
  }

  // skins
  for (i=0; i<skins.size(); i++) {
    if (skins[i]->name.empty()) {
      stripped = mjuu_strippath(skins[i]->file);
      skins[i]->name = mjuu_stripext(stripped);
    }
  }

  // hfields
  for (i=0; i<hfields.size(); i++) {
    if (hfields[i]->name.empty()) {
      stripped = mjuu_strippath(hfields[i]->file);
      hfields[i]->name = mjuu_stripext(stripped);

      // name cannot be empty
      if (hfields[i]->name.empty()) {
        throw mjCError(hfields[i], "empty name in height field");
      }
    }
  }

  // textures
  for (i=0; i<textures.size(); i++) {
    if (textures[i]->name.empty()) {
      stripped = mjuu_strippath(textures[i]->file);
      textures[i]->name = mjuu_stripext(stripped);

      // name cannot be empty, except for skybox
      if (textures[i]->name.empty() && textures[i]->type!=mjTEXTURE_SKYBOX) {
        throw mjCError(textures[i], "empty name in texture");
      }
    }
  }

  // materials: name check only
  for (i=0; i<materials.size(); i++) {
    if (materials[i]->name.empty()) {
      throw mjCError(materials[i], "empty name in material");
    }
  }
}



// number of position and velocity coordinates for each joint type
const int nPOS[4] = {7, 4, 1, 1};
const int nVEL[4] = {6, 3, 1, 1};

// set array sizes
void mjCModel::SetSizes(void) {
  int i, j;

  // set from object list sizes
  nbody = (int)bodies.size();
  njnt = (int)joints.size();
  ngeom = (int)geoms.size();
  nsite = (int)sites.size();
  ncam = (int)cameras.size();
  nlight = (int)lights.size();
  nmesh = (int)meshes.size();
  nskin = (int)skins.size();
  nhfield = (int)hfields.size();
  ntex = (int)textures.size();
  nmat = (int)materials.size();
  npair = (int)pairs.size();
  nexclude = (int)excludes.size();
  neq = (int)equalities.size();
  ntendon = (int)tendons.size();
  nsensor = (int)sensors.size();
  nnumeric = (int)numerics.size();
  ntext = (int)texts.size();
  ntuple = (int)tuples.size();
  nkey = (int)keys.size();

  // nq, nv
  for (i=0; i<njnt; i++) {
    nq += nPOS[joints[i]->type];
    nv += nVEL[joints[i]->type];
  }

  // nu, na
  for (i=0; i<(int)actuators.size(); i++) {
    if (actuators[i]->dyntype == mjDYN_NONE) {
      //  make sure all 2nd-order come before all 3rd-order
      if (na) {
        throw mjCError(0, "stateless actuators must come before stateful actuators");
      }

      nu++;
    } else {
      nu++;
      na++;
    }
  }

  // nmeshvert, nmeshface, nmeshtexvert, nmeshgraph
  for (i=0; i<nmesh; i++) {
    nmeshvert += meshes[i]->nvert;
    nmeshface += meshes[i]->nface;
    nmeshtexvert += (meshes[i]->texcoord ? meshes[i]->nvert : 0);
    nmeshgraph += meshes[i]->szgraph;
  }

  // nskinvert, nskintexvert, nskinface, nskinbone, nskinbonevert
  for (i=0; i<nskin; i++) {
    nskinvert += skins[i]->vert.size()/3;
    nskintexvert += skins[i]->texcoord.size()/2;
    nskinface += skins[i]->face.size()/3;
    nskinbone += skins[i]->bodyid.size();
    for (j=0; j<skins[i]->bodyid.size(); j++) {
      nskinbonevert += skins[i]->vertid[j].size();
    }
  }

  // nhfielddata
  for (i=0; i<nhfield; i++) nhfielddata += hfields[i]->nrow * hfields[i]->ncol;

  // ntexdata
  for (i=0; i<ntex; i++) ntexdata += 3 * textures[i]->width * textures[i]->height;

  // nwrap
  for (i=0; i<ntendon; i++) nwrap += (int)tendons[i]->path.size();

  // nsensordata
  for (i=0; i<nsensor; i++) nsensordata += sensors[i]->dim;

  // nnumericdata
  for (i=0; i<nnumeric; i++) nnumericdata += numerics[i]->size;

  // ntextdata
  for (i=0; i<ntext; i++) ntextdata += (int)texts[i]->data.size() + 1;

  // ntupledata
  for (i=0; i<ntuple; i++) ntupledata += (int)tuples[i]->objtype.size();

  // nnames
  nnames = (int)modelname.size() + 1;
  for (i=0; i<nbody; i++)    nnames += (int)bodies[i]->name.length() + 1;
  for (i=0; i<njnt; i++)     nnames += (int)joints[i]->name.length() + 1;
  for (i=0; i<ngeom; i++)    nnames += (int)geoms[i]->name.length() + 1;
  for (i=0; i<nsite; i++)    nnames += (int)sites[i]->name.length() + 1;
  for (i=0; i<ncam; i++)     nnames += (int)cameras[i]->name.length() + 1;
  for (i=0; i<nlight; i++)   nnames += (int)lights[i]->name.length() + 1;
  for (i=0; i<nmesh; i++)    nnames += (int)meshes[i]->name.length() + 1;
  for (i=0; i<nskin; i++)    nnames += (int)skins[i]->name.length() + 1;
  for (i=0; i<nhfield; i++)  nnames += (int)hfields[i]->name.length() + 1;
  for (i=0; i<ntex; i++)     nnames += (int)textures[i]->name.length() + 1;
  for (i=0; i<nmat; i++)     nnames += (int)materials[i]->name.length() + 1;
  for (i=0; i<npair; i++)    nnames += (int)pairs[i]->name.length() + 1;
  for (i=0; i<nexclude; i++) nnames += (int)excludes[i]->name.length() + 1;
  for (i=0; i<neq; i++)      nnames += (int)equalities[i]->name.length() + 1;
  for (i=0; i<ntendon; i++)  nnames += (int)tendons[i]->name.length() + 1;
  for (i=0; i<nu; i++)       nnames += (int)actuators[i]->name.length() + 1;
  for (i=0; i<nsensor; i++)  nnames += (int)sensors[i]->name.length() + 1;
  for (i=0; i<nnumeric; i++) nnames += (int)numerics[i]->name.length() + 1;
  for (i=0; i<ntext; i++)    nnames += (int)texts[i]->name.length() + 1;
  for (i=0; i<ntuple; i++)   nnames += (int)tuples[i]->name.length() + 1;
  for (i=0; i<nkey; i++)     nnames += (int)keys[i]->name.length() + 1;

  // nemax
  for (i=0; i<neq; i++)
    if (equalities[i]->type==mjEQ_CONNECT) {
      nemax += 3;
    } else if (equalities[i]->type==mjEQ_WELD) {
      nemax += 7;
    } else {
      nemax += 1;
    }

  // nconmax
  if (nconmax<0) {
    nconmax = 100;
  }

  // njmax
  if (njmax<0) {
    njmax = 500;
  }
}



// automatic stiffness and damping computation
void mjCModel::AutoSpringDamper(mjModel* m) {
  // process all joints
  for (int n=0; n<m->njnt; n++) {
    // get joint dof address and number of dimensions
    int adr = m->jnt_dofadr[n];
    int ndim = nVEL[m->jnt_type[n]];

    // get timeconst and dampratio from joint specificatin
    mjtNum timeconst = (mjtNum)joints[n]->springdamper[0];
    mjtNum dampratio = (mjtNum)joints[n]->springdamper[1];

    // skip joint if either parameter is non-positive
    if (timeconst<=0 || dampratio<=0) {
      continue;
    }

    // get average inertia (dof_invweight0 in free joint is different for tran and rot)
    mjtNum inertia = 0;
    for (int i=0; i<ndim; i++) {
      inertia += m->dof_invweight0[adr+i];
    }
    inertia = ((mjtNum)ndim) / mju_max(mjMINVAL, inertia);

    // compute stiffness and damping (same as solref computation)
    mjtNum stiffness = inertia / mju_max(mjMINVAL, timeconst*timeconst*dampratio*dampratio);
    mjtNum damping = 2 * inertia / mju_max(mjMINVAL, timeconst);

    // assign
    m->jnt_stiffness[n] = stiffness;
    for (int i=0; i<ndim; i++) {
      m->dof_damping[adr+i] = damping;
    }
  }
}



// arguments for lengthrange thread function
struct _LRThreadArg {
  mjModel* m;
  mjData* data;
  int start;
  int num;
  const mjLROpt* LRopt;
  char* error;
  int error_sz;
};
typedef struct _LRThreadArg LRThreadArg;


// thread function for lengthrange computation
void* LRfunc(void* arg) {
  LRThreadArg* larg = (LRThreadArg*)arg;

  for (int i=larg->start; i<larg->start+larg->num; i++) {
    if (i<larg->m->nu) {
      if (!mj_setLengthRange(larg->m, larg->data, i, larg->LRopt, larg->error, larg->error_sz)) {
        return NULL;
      }
    }
  }

  return NULL;
}


// compute actuator lengthrange
void mjCModel::LengthRange(mjModel* m, mjData* data) {
  int i;

  // save options and modify
  mjOption saveopt = m->opt;
  m->opt.disableflags = mjDSBL_FRICTIONLOSS | mjDSBL_CONTACT | mjDSBL_PASSIVE |
                        mjDSBL_GRAVITY | mjDSBL_ACTUATION;
  if (LRopt.timestep>0) {
    m->opt.timestep = LRopt.timestep;
  }

  // number of threads available, max 16
  const int nthread = mjMIN(16, getnumproc()/2);

  // count actuators that need computation
  int cnt = 0;
  for (i=0; i<m->nu; i++) {
    // skip depending on mode and type
    int ismuscle = (m->actuator_gaintype[i]==mjGAIN_MUSCLE ||
                    m->actuator_biastype[i]==mjBIAS_MUSCLE);
    int isuser = (m->actuator_gaintype[i]==mjGAIN_USER ||
                  m->actuator_biastype[i]==mjBIAS_USER);
    if ((LRopt.mode==mjLRMODE_NONE) ||
        (LRopt.mode==mjLRMODE_MUSCLE && !ismuscle) ||
        (LRopt.mode==mjLRMODE_MUSCLEUSER && !ismuscle && !isuser)) {
      continue;
    }

    // use existing length range if available
    if (LRopt.useexisting &&
        (m->actuator_lengthrange[2*i] < m->actuator_lengthrange[2*i+1])) {
      continue;
    }

    // count
    cnt++;
  }

  // single thread
  if (!usethread || cnt<2 || nthread<2) {
    char err[200];
    for (i=0; i<m->nu; i++) {
      if (!mj_setLengthRange(m, data, i, &LRopt, err, 200)) {
        throw mjCError(0, err);
      }
    }
  }

  // multiple threads
  else {
    // allocate mjData for each thread
    char err[16][200];
    mjData* pdata[16] = {data};
    for (i=1; i<nthread; i++) {
      pdata[i] = mj_makeData(m);
    }

    // number of actuators per thread
    int num = m->nu / nthread;
    while (num*nthread < m->nu) {
      num++;
    }

    // prepare thread function arguments, clear errors
    LRThreadArg arg[16];
    for (i=0; i<nthread; i++) {
      LRThreadArg temp = {m, pdata[i], i*num, num, &LRopt, err[i], 200};
      arg[i] = temp;
      err[i][0] = 0;
    }

    // use std::thread
#if defined(_WIN32) || defined(__APPLE__)
    // launch threads
    thread th[16];
    for (i=0; i<nthread; i++) {
      th[i] = thread(LRfunc, arg+i);
    }

    // wait for threads to finish
    for (i=0; i<nthread; i++) {
      th[i].join();
    }

    // use pthread
#else
    // launch threads
    pthread_t th[16];
    for (i=0; i<nthread; i++) {
      pthread_create(th+i, NULL, LRfunc, arg+i);
    }

    // wait for threads to finish
    for (i=0; i<nthread; i++) {
      pthread_join(th[i], NULL);
    }
#endif

    // free mjData allocated here
    for (i=1; i<nthread; i++) {
      mj_deleteData(pdata[i]);
    }

    // report first error
    for (i=0; i<nthread; i++) {
      if (err[i][0]) {
        throw mjCError(0, err[i]);
      }
    }
  }

  // restore options
  m->opt = saveopt;
}



// process names from one list: concatenate, compute addresses
template <class T>
static int namelist(vector<T*>& list, int adr, int* name_adr, char* names) {
  for (unsigned int i=0; i<list.size(); i++) {
    name_adr[i] = adr;

    // copy name
    memcpy(names+adr, list[i]->name.c_str(), list[i]->name.size());
    adr += (int)list[i]->name.size();

    // append 0
    names[adr] = 0;
    adr++;
  }

  return adr;
}


// copy names, compute name addresses
void mjCModel::CopyNames(mjModel* m) {
  // start with model name
  int adr = (int)modelname.size()+1;
  mju_strncpy(m->names, modelname.c_str(), m->nnames);

  // process all lists
  adr = namelist(bodies, adr, m->name_bodyadr, m->names);
  adr = namelist(joints, adr, m->name_jntadr, m->names);
  adr = namelist(geoms, adr, m->name_geomadr, m->names);
  adr = namelist(sites, adr, m->name_siteadr, m->names);
  adr = namelist(cameras, adr, m->name_camadr, m->names);
  adr = namelist(lights, adr, m->name_lightadr, m->names);
  adr = namelist(meshes, adr, m->name_meshadr, m->names);
  adr = namelist(skins, adr, m->name_skinadr, m->names);
  adr = namelist(hfields, adr, m->name_hfieldadr, m->names);
  adr = namelist(textures, adr, m->name_texadr, m->names);
  adr = namelist(materials, adr, m->name_matadr, m->names);
  adr = namelist(pairs, adr, m->name_pairadr, m->names);
  adr = namelist(excludes, adr, m->name_excludeadr, m->names);
  adr = namelist(equalities, adr, m->name_eqadr, m->names);
  adr = namelist(tendons, adr, m->name_tendonadr, m->names);
  adr = namelist(actuators, adr, m->name_actuatoradr, m->names);
  adr = namelist(sensors, adr, m->name_sensoradr, m->names);
  adr = namelist(numerics, adr, m->name_numericadr, m->names);
  adr = namelist(texts, adr, m->name_textadr, m->names);
  adr = namelist(tuples, adr, m->name_tupleadr, m->names);
  adr = namelist(keys, adr, m->name_keyadr, m->names);

  // check size, SHOULD NOT OCCUR
  if (adr != nnames) {
    throw mjCError(0, "size mismatch in %s: expected %d, got %d", "names", nnames, adr);
  }
}



// copy objects inside kinematic tree
void mjCModel::CopyTree(mjModel* m) {
  int i, j, j1;
  int jntadr = 0;         // addresses in global arrays
  int dofadr = 0;
  int qposadr = 0;

  // main loop over bodies
  for (i=0; i<nbody; i++) {
    // get body and parent pointers
    mjCBody* pb = bodies[i];
    mjCBody* par = bodies[pb->parentid];

    // set body fields
    m->body_parentid[i] = pb->parentid;
    m->body_weldid[i] = pb->weldid;
    m->body_mocapid[i] = pb->mocapid;
    m->body_jntnum[i] = (int)pb->joints.size();
    m->body_jntadr[i] = (!pb->joints.empty() ? jntadr : -1);
    m->body_dofnum[i] = pb->dofnum;
    m->body_dofadr[i] = (pb->dofnum ? dofadr : -1);
    m->body_geomnum[i] = (int)pb->geoms.size();
    m->body_geomadr[i] = (!pb->geoms.empty() ? pb->geoms[0]->id : -1);
    copyvec(m->body_pos+3*i, pb->locpos, 3);
    copyvec(m->body_quat+4*i, pb->locquat, 4);
    copyvec(m->body_ipos+3*i, pb->locipos, 3);
    copyvec(m->body_iquat+4*i, pb->lociquat, 4);
    m->body_mass[i] = (mjtNum)pb->mass;
    copyvec(m->body_inertia+3*i, pb->inertia, 3);
    copyvec(m->body_user+nuser_body*i, pb->userdata.data(), nuser_body);

    // count free joints
    int cntfree = 0;
    for (j=0; j<(int)pb->joints.size(); j++) {
      cntfree += (pb->joints[j]->type == mjJNT_FREE);
    }

    // check validity of free joint
    if (cntfree>1 || (cntfree==1 && pb->joints.size()>1)) {
      throw mjCError(pb, "free joint can only appear by itself");
    }
    if (cntfree && pb->parentid) {
      throw mjCError(pb, "free joint can only be used on top level");
    }

    // rootid: self if world or child of world, otherwise parent's rootid
    if (i==0 || pb->parentid==0) {
      m->body_rootid[i] = i;
    } else {
      m->body_rootid[i] = m->body_rootid[pb->parentid];
    }

    // init lastdof from parent
    pb->lastdof = par->lastdof;

    // set sameframe
    m->body_sameframe[i] = IsNullPose(m->body_ipos+3*i, m->body_iquat+4*i);

    // init simple: sameframe, and (self-root, or parent is fixed child of world)
    j = m->body_parentid[i];
    m->body_simple[i] = (m->body_sameframe[i] &&
                         (m->body_rootid[i]==i ||
                          (m->body_parentid[j]==0 &&
                           m->body_dofnum[j]==0)));

    // parent is not simple (unless world)
    if (m->body_parentid[i]>0) {
      m->body_simple[m->body_parentid[i]] = 0;
    }

    // loop over joints for this body
    int rotfound = 0;
    for (j=0; j<(int)pb->joints.size(); j++) {
      // get pointer and id
      mjCJoint* pj = pb->joints[j];
      int jid = pj->id;

      // set joint fields
      m->jnt_type[jid] = pj->type;
      m->jnt_group[jid] = pj->group;
      m->jnt_limited[jid] = pj->limited;
      m->jnt_qposadr[jid] = qposadr;
      m->jnt_dofadr[jid] = dofadr;
      m->jnt_bodyid[jid] = pj->body->id;
      copyvec(m->jnt_pos+3*jid, pj->locpos, 3);
      copyvec(m->jnt_axis+3*jid, pj->locaxis, 3);
      m->jnt_stiffness[jid] = (mjtNum)pj->stiffness;
      copyvec(m->jnt_range+2*jid, pj->range, 2);
      copyvec(m->jnt_solref+mjNREF*jid, pj->solref_limit, mjNREF);
      copyvec(m->jnt_solimp+mjNIMP*jid, pj->solimp_limit, mjNIMP);
      m->jnt_margin[jid] = (mjtNum)pj->margin;
      copyvec(m->jnt_user+nuser_jnt*jid, pj->userdata.data(), nuser_jnt);

      // not simple if: rotation already found, or pos not zero, or mis-aligned axis
      if (rotfound ||
          !IsNullPose(m->jnt_pos+3*jid, NULL) ||
          ((pj->type==mjJNT_HINGE || pj->type==mjJNT_SLIDE) &&
           ((pj->locaxis[0]!=0) + (pj->locaxis[1]!=0) + (pj->locaxis[2]!=0))>1)) {
        m->body_simple[i] = 0;
      }

      // mark rotation
      if (pj->type==mjJNT_BALL || pj->type==mjJNT_HINGE) {
        rotfound = 1;
      }

      // set qpos0 and qpos_spring, check type
      switch (pj->type) {
      case mjJNT_FREE:
        copyvec(m->qpos0+qposadr, pb->pos, 3);
        copyvec(m->qpos0+qposadr+3, pb->quat, 4);
        mju_copy(m->qpos_spring+qposadr, m->qpos0+qposadr, 7);
        break;

      case mjJNT_BALL:
        m->qpos0[qposadr] = 1;
        m->qpos0[qposadr+1] = 0;
        m->qpos0[qposadr+2] = 0;
        m->qpos0[qposadr+3] = 0;
        mju_copy4(m->qpos_spring+qposadr, m->qpos0+qposadr);
        break;

      case mjJNT_SLIDE:
      case mjJNT_HINGE:
        m->qpos0[qposadr] = (mjtNum)pj->ref;
        m->qpos_spring[qposadr] = (mjtNum)pj->springref;
        break;

      default:
        throw mjCError(pj, "unknown joint type");
      }

      // set dof fields for this joint
      for (j1=0; j1<nVEL[pj->type]; j1++) {
        // set attributes
        m->dof_bodyid[dofadr] = pb->id;
        m->dof_jntid[dofadr] = jid;
        copyvec(m->dof_solref+mjNREF*dofadr, pj->solref_friction, mjNREF);
        copyvec(m->dof_solimp+mjNIMP*dofadr, pj->solimp_friction, mjNIMP);
        m->dof_frictionloss[dofadr] = (mjtNum)pj->frictionloss;
        m->dof_armature[dofadr] = (mjtNum)pj->armature;
        m->dof_damping[dofadr] = (mjtNum)pj->damping;

        // set dof_parentid, update body.lastdof
        m->dof_parentid[dofadr] = pb->lastdof;
        pb->lastdof = dofadr;

        // advance dof counter
        dofadr++;
      }

      // advance joint and qpos counters
      jntadr++;
      qposadr += nPOS[pj->type];
    }

    // loop over geoms for this body
    for (j=0; j<(int)pb->geoms.size(); j++) {
      // get pointer and id
      mjCGeom* pg = pb->geoms[j];
      int gid = pg->id;

      // set geom fields
      m->geom_type[gid] = pg->type;
      m->geom_contype[gid] = pg->contype;
      m->geom_conaffinity[gid] = pg->conaffinity;
      m->geom_condim[gid] = pg->condim;
      m->geom_bodyid[gid] = pg->body->id;
      if (pg->meshid>=0) {
        m->geom_dataid[gid] = pg->meshid;
      } else if (pg->hfieldid>=0) {
        m->geom_dataid[gid] = pg->hfieldid;
      } else {
        m->geom_dataid[gid] = -1;
      }
      m->geom_matid[gid] = pg->matid;
      m->geom_group[gid] = pg->group;
      m->geom_priority[gid] = pg->priority;
      copyvec(m->geom_size+3*gid, pg->size, 3);
      copyvec(m->geom_pos+3*gid, pg->locpos, 3);
      copyvec(m->geom_quat+4*gid, pg->locquat, 4);
      copyvec(m->geom_friction+3*gid, pg->friction, 3);
      m->geom_solmix[gid] = (mjtNum)pg->solmix;
      copyvec(m->geom_solref+mjNREF*gid, pg->solref, mjNREF);
      copyvec(m->geom_solimp+mjNIMP*gid, pg->solimp, mjNIMP);
      m->geom_margin[gid] = (mjtNum)pg->margin;
      m->geom_gap[gid] = (mjtNum)pg->gap;
      copyvec(m->geom_fluid+mjNFLUID*gid, pg->fluid, mjNFLUID);
      copyvec(m->geom_user+nuser_geom*gid, pg->userdata.data(), nuser_geom);
      copyvec(m->geom_rgba+4*gid, pg->rgba, 4);

      // determine sameframe
      if (IsNullPose(m->geom_pos+3*gid, m->geom_quat+4*gid)) {
        m->geom_sameframe[gid] = 1;
      } else if (pg->locpos[0]==pb->locipos[0] &&
                 pg->locpos[1]==pb->locipos[1] &&
                 pg->locpos[2]==pb->locipos[2] &&
                 pg->locquat[0]==pb->lociquat[0] &&
                 pg->locquat[1]==pb->lociquat[1] &&
                 pg->locquat[2]==pb->lociquat[2] &&
                 pg->locquat[3]==pb->lociquat[3]) {
        m->geom_sameframe[gid] = 2;
      } else {
        m->geom_sameframe[gid] = 0;
      }

      // compute rbound
      m->geom_rbound[gid] = (mjtNum)pg->GetRBound();
    }

    // loop over sites for this body
    for (j=0; j<(int)pb->sites.size(); j++) {
      // get pointer and id
      mjCSite* ps = pb->sites[j];
      int sid = ps->id;

      // set site fields
      m->site_type[sid] = ps->type;
      m->site_bodyid[sid] = ps->body->id;
      m->site_matid[sid] = ps->matid;
      m->site_group[sid] = ps->group;
      copyvec(m->site_size+3*sid, ps->size, 3);
      copyvec(m->site_pos+3*sid, ps->locpos, 3);
      copyvec(m->site_quat+4*sid, ps->locquat, 4);
      copyvec(m->site_user+nuser_site*sid, ps->userdata.data(), nuser_site);
      copyvec(m->site_rgba+4*sid, ps->rgba, 4);

      // determine sameframe
      if (IsNullPose(m->site_pos+3*sid, m->site_quat+4*sid)) {
        m->site_sameframe[sid] = 1;
      } else if (ps->locpos[0]==pb->locipos[0] &&
                 ps->locpos[1]==pb->locipos[1] &&
                 ps->locpos[2]==pb->locipos[2] &&
                 ps->locquat[0]==pb->lociquat[0] &&
                 ps->locquat[1]==pb->lociquat[1] &&
                 ps->locquat[2]==pb->lociquat[2] &&
                 ps->locquat[3]==pb->lociquat[3]) {
        m->site_sameframe[sid] = 2;
      } else {
        m->site_sameframe[sid] = 0;
      }
    }

    // loop over cameras for this body
    for (j=0; j<(int)pb->cameras.size(); j++) {
      // get pointer and id
      mjCCamera* pc = pb->cameras[j];
      int cid = pc->id;

      // set camera fields
      m->cam_bodyid[cid] = pc->body->id;
      m->cam_mode[cid] = pc->mode;
      m->cam_targetbodyid[cid] = pc->targetbodyid;
      copyvec(m->cam_pos+3*cid, pc->locpos, 3);
      copyvec(m->cam_quat+4*cid, pc->locquat, 4);
      m->cam_fovy[cid] = (mjtNum)pc->fovy;
      m->cam_ipd[cid] = (mjtNum)pc->ipd;
      copyvec(m->cam_user+nuser_cam*cid, pc->userdata.data(), nuser_cam);
    }

    // loop over lights for this body
    for (j=0; j<(int)pb->lights.size(); j++) {
      // get pointer and id
      mjCLight* pl = pb->lights[j];
      int lid = pl->id;

      // set light fields
      m->light_bodyid[lid] = pl->body->id;
      m->light_mode[lid] = (int)pl->mode;
      m->light_targetbodyid[lid] = pl->targetbodyid;
      m->light_directional[lid] = (mjtByte)pl->directional;
      m->light_castshadow[lid] = (mjtByte)pl->castshadow;
      m->light_active[lid] = (mjtByte)pl->active;
      copyvec(m->light_pos+3*lid, pl->locpos, 3);
      copyvec(m->light_dir+3*lid, pl->locdir, 3);
      copyvec(m->light_attenuation+3*lid, pl->attenuation, 3);
      m->light_cutoff[lid] = pl->cutoff;
      m->light_exponent[lid] = pl->exponent;
      copyvec(m->light_ambient+3*lid, pl->ambient, 3);
      copyvec(m->light_diffuse+3*lid, pl->diffuse, 3);
      copyvec(m->light_specular+3*lid, pl->specular, 3);
    }
  }

  // check number of dof's constructed, SHOULD NOT OCCUR
  if (nv!=dofadr) {
    throw mjCError(0, "unexpected number of DOFs");
  }

  // compute nM and dof_Madr
  nM = 0;
  for (i=0; i<nv; i++) {
    // set address of this dof
    m->dof_Madr[i] = nM;

    // count ancestor dofs including self
    j = i;
    while (j>=0) {
      nM++;
      j = m->dof_parentid[j];
    }
  }
  m->nM = nM;

  // set nD
  nD = 2*nM - nv;
  m->nD = nD;

  // set dof_simplenum
  int scnt = 0;
  for (i=nv-1; i>=0; i--) {
    // dof in simple body
    if (m->body_simple[m->dof_bodyid[i]]) {
      scnt++;
      m->dof_simplenum[i] = scnt;
    }

    // dof in regular body
    else {
      scnt = 0;
      m->dof_simplenum[i] = 0;
    }
  }
}



// copy objects outside kinematic tree
void mjCModel::CopyObjects(mjModel* m) {
  int i, j, adr, bone_adr, vert_adr, face_adr, texcoord_adr;
  int bonevert_adr, graph_adr, data_adr;

  // sizes outside call to mj_makeModel
  m->nemax = nemax;
  m->njmax = njmax;
  m->nconmax = nconmax;
  m->nstack = nstack;
  m->nsensordata = nsensordata;
  m->nuserdata = nuserdata;

  // meshes
  vert_adr = 0;
  texcoord_adr = 0;
  face_adr = 0;
  graph_adr = 0;
  for (i=0; i<nmesh; i++) {
    // get pointer
    mjCMesh* pme = meshes[i];

    // set fields
    m->mesh_vertadr[i] = vert_adr;
    m->mesh_vertnum[i] = pme->nvert;
    m->mesh_texcoordadr[i] = (pme->texcoord ? texcoord_adr : -1);
    m->mesh_faceadr[i] = face_adr;
    m->mesh_facenum[i] = pme->nface;
    m->mesh_graphadr[i] = (pme->szgraph ? graph_adr : -1);

    // copy vertices, normals, faces, texcoords, aux data
    memcpy(m->mesh_vert + 3*vert_adr, pme->vert, 3*pme->nvert*sizeof(float));
    memcpy(m->mesh_normal + 3*vert_adr, pme->normal, 3*pme->nvert*sizeof(float));
    memcpy(m->mesh_face + 3*face_adr, pme->face, 3*pme->nface*sizeof(float));
    if (pme->texcoord) {
      memcpy(m->mesh_texcoord + 2*texcoord_adr, pme->texcoord, 2*pme->nvert*sizeof(float));
    }
    if (pme->szgraph) {
      memcpy(m->mesh_graph + graph_adr, pme->graph, pme->szgraph*sizeof(int));
    }

    // advance counters
    vert_adr += pme->nvert;
    texcoord_adr += (pme->texcoord ? pme->nvert : 0);
    face_adr += pme->nface;
    graph_adr += pme->szgraph;
  }

  // skins
  vert_adr = 0;
  face_adr = 0;
  texcoord_adr = 0;
  bone_adr = 0;
  bonevert_adr = 0;
  for (i=0; i<nskin; i++) {
    // get pointer
    mjCSkin* psk = skins[i];

    // set fields
    m->skin_matid[i] = psk->matid;
    copyvec(m->skin_rgba+4*i, psk->rgba, 4);
    m->skin_group[i] = psk->group;
    m->skin_inflate[i] = psk->inflate;
    m->skin_vertadr[i] = vert_adr;
    m->skin_vertnum[i] = psk->vert.size()/3;
    m->skin_texcoordadr[i] = (!psk->texcoord.empty() ? texcoord_adr : -1);
    m->skin_faceadr[i] = face_adr;
    m->skin_facenum[i] = psk->face.size()/3;
    m->skin_boneadr[i] = bone_adr;
    m->skin_bonenum[i] = psk->bodyid.size();

    // copy mesh data
    memcpy(m->skin_vert + 3*vert_adr, psk->vert.data(), psk->vert.size()*sizeof(float));
    if (!psk->texcoord.empty())
      memcpy(m->skin_texcoord + 2*texcoord_adr, psk->texcoord.data(),
             psk->texcoord.size()*sizeof(float));
    memcpy(m->skin_face + 3*face_adr, psk->face.data(), psk->face.size()*sizeof(int));

    // copy bind poses and body ids
    memcpy(m->skin_bonebindpos+3*bone_adr, psk->bindpos.data(),
           psk->bindpos.size()*sizeof(float));
    memcpy(m->skin_bonebindquat+4*bone_adr, psk->bindquat.data(),
           psk->bindquat.size()*sizeof(float));
    memcpy(m->skin_bonebodyid+bone_adr, psk->bodyid.data(),
           psk->bodyid.size()*sizeof(int));

    // copy per-bone vertex data, advance vertex counter
    for (j=0; j<m->skin_bonenum[i]; j++) {
      // set fields
      m->skin_bonevertadr[bone_adr+j] = bonevert_adr;
      m->skin_bonevertnum[bone_adr+j] = (int)psk->vertid[j].size();

      // copy data
      memcpy(m->skin_bonevertid+bonevert_adr, psk->vertid[j].data(),
             psk->vertid[j].size()*sizeof(int));
      memcpy(m->skin_bonevertweight+bonevert_adr, psk->vertweight[j].data(),
             psk->vertid[j].size()*sizeof(float));

      // advance counter
      bonevert_adr += m->skin_bonevertnum[bone_adr+j];
    }

    // advance mesh and bone counters
    vert_adr += m->skin_vertnum[i];
    texcoord_adr += psk->texcoord.size()/2;
    face_adr += m->skin_facenum[i];
    bone_adr += m->skin_bonenum[i];
  }

  // hfields
  data_adr = 0;
  for (i=0; i<nhfield; i++) {
    // get pointer
    mjCHField* phf = hfields[i];

    // set fields
    copyvec(m->hfield_size+4*i, phf->size, 4);
    m->hfield_nrow[i] = phf->nrow;
    m->hfield_ncol[i] = phf->ncol;
    m->hfield_adr[i] = data_adr;

    // copy elevation data
    memcpy(m->hfield_data + data_adr, phf->data, phf->nrow*phf->ncol*sizeof(float));

    // advance counter
    data_adr += phf->nrow*phf->ncol;
  }

  // textures
  data_adr = 0;
  for (i=0; i<ntex; i++) {
    // get pointer
    mjCTexture* ptex = textures[i];

    // set fields
    m->tex_type[i] = ptex->type;
    m->tex_height[i] = ptex->height;
    m->tex_width[i] = ptex->width;
    m->tex_adr[i] = data_adr;

    // copy rgb data
    memcpy(m->tex_rgb + data_adr, ptex->rgb, 3*ptex->width*ptex->height);

    // advance counter
    data_adr += 3*ptex->width*ptex->height;
  }

  // materials
  for (i=0; i<nmat; i++) {
    // get pointer
    mjCMaterial* pmat = materials[i];

    // set fields
    m->mat_texid[i] = pmat->texid;
    m->mat_texuniform[i] = pmat->texuniform;
    copyvec(m->mat_texrepeat+2*i, pmat->texrepeat, 2);
    m->mat_emission[i] = pmat->emission;
    m->mat_specular[i] = pmat->specular;
    m->mat_shininess[i] = pmat->shininess;
    m->mat_reflectance[i] = pmat->reflectance;
    copyvec(m->mat_rgba+4*i, pmat->rgba, 4);
  }

  // geom pairs to include
  for (i=0; i<npair; i++) {
    m->pair_dim[i] = pairs[i]->condim;
    m->pair_geom1[i] = pairs[i]->geom1;
    m->pair_geom2[i] = pairs[i]->geom2;
    m->pair_signature[i] = pairs[i]->signature;
    copyvec(m->pair_solref+mjNREF*i, pairs[i]->solref, mjNREF);
    copyvec(m->pair_solimp+mjNIMP*i, pairs[i]->solimp, mjNIMP);
    m->pair_margin[i] = (mjtNum)pairs[i]->margin;
    m->pair_gap[i] = (mjtNum)pairs[i]->gap;
    copyvec(m->pair_friction+5*i, pairs[i]->friction, 5);
  }

  // body pairs to exclude
  for (i=0; i<nexclude; i++) {
    m->exclude_signature[i] = excludes[i]->signature;
  }

  // equality constraints
  for (i=0; i<neq; i++) {
    // get pointer
    mjCEquality* peq = equalities[i];

    // set fields
    m->eq_type[i] = peq->type;
    m->eq_obj1id[i] = peq->obj1id;
    m->eq_obj2id[i] = peq->obj2id;
    m->eq_active[i] = peq->active;
    copyvec(m->eq_solref+mjNREF*i, peq->solref, mjNREF);
    copyvec(m->eq_solimp+mjNIMP*i, peq->solimp, mjNIMP);
    copyvec(m->eq_data+mjNEQDATA*i, peq->data, mjNEQDATA);
  }

  // tendons and wraps
  adr = 0;
  for (i=0; i<ntendon; i++) {
    // get pointer
    mjCTendon* pte = tendons[i];

    // set fields
    m->tendon_adr[i] = adr;
    m->tendon_num[i] = (int)pte->path.size();
    m->tendon_matid[i] = pte->matid;
    m->tendon_group[i] = pte->group;;
    m->tendon_limited[i] = pte->limited;
    m->tendon_width[i] = (mjtNum)pte->width;
    copyvec(m->tendon_solref_lim+mjNREF*i, pte->solref_limit, mjNREF);
    copyvec(m->tendon_solimp_lim+mjNIMP*i, pte->solimp_limit, mjNIMP);
    copyvec(m->tendon_solref_fri+mjNREF*i, pte->solref_friction, mjNREF);
    copyvec(m->tendon_solimp_fri+mjNIMP*i, pte->solimp_friction, mjNIMP);
    m->tendon_range[2*i] = (mjtNum)pte->range[0];
    m->tendon_range[2*i+1] = (mjtNum)pte->range[1];
    m->tendon_margin[i] = (mjtNum)pte->margin;
    m->tendon_stiffness[i] = (mjtNum)pte->stiffness;
    m->tendon_damping[i] = (mjtNum)pte->damping;
    m->tendon_frictionloss[i] = (mjtNum)pte->frictionloss;
    m->tendon_lengthspring[i] = (mjtNum)pte->springlength;
    copyvec(m->tendon_user+nuser_tendon*i, pte->userdata.data(), nuser_tendon);
    copyvec(m->tendon_rgba+4*i, pte->rgba, 4);

    // set wraps
    for (j=0; j<(int)pte->path.size(); j++) {
      m->wrap_type[adr+j] = pte->path[j]->type;
      m->wrap_objid[adr+j] = pte->path[j]->objid;
      m->wrap_prm[adr+j] = (mjtNum)pte->path[j]->prm;
      if (pte->path[j]->type==mjWRAP_SPHERE || pte->path[j]->type==mjWRAP_CYLINDER) {
        m->wrap_prm[adr+j] = (mjtNum)pte->path[j]->sideid;
      }
    }

    // advance address counter
    adr += (int)pte->path.size();
  }

  // actuators
  for (i=0; i<nu; i++) {
    // get pointer
    mjCActuator* pac = actuators[i];

    // set fields
    m->actuator_trntype[i] = pac->trntype;
    m->actuator_dyntype[i] = pac->dyntype;
    m->actuator_gaintype[i] = pac->gaintype;
    m->actuator_biastype[i] = pac->biastype;
    m->actuator_trnid[2*i] = pac->trnid[0];
    m->actuator_trnid[2*i+1] = pac->trnid[1];
    m->actuator_group[i] = pac->group;
    m->actuator_ctrllimited[i] = pac->ctrllimited;
    m->actuator_forcelimited[i] = pac->forcelimited;
    m->actuator_actlimited[i] = pac->actlimited;
    m->actuator_cranklength[i] = (mjtNum)pac->cranklength;
    copyvec(m->actuator_gear + 6*i, pac->gear, 6);
    copyvec(m->actuator_dynprm + mjNDYN*i, pac->dynprm, mjNDYN);
    copyvec(m->actuator_gainprm + mjNGAIN*i, pac->gainprm, mjNGAIN);
    copyvec(m->actuator_biasprm + mjNBIAS*i, pac->biasprm, mjNBIAS);
    copyvec(m->actuator_ctrlrange + 2*i, pac->ctrlrange, 2);
    copyvec(m->actuator_forcerange + 2*i, pac->forcerange, 2);
    copyvec(m->actuator_actrange + 2*i, pac->actrange, 2);
    copyvec(m->actuator_lengthrange + 2*i, pac->lengthrange, 2);
    copyvec(m->actuator_user+nuser_actuator*i, pac->userdata.data(), nuser_actuator);
  }

  // sensors
  adr = 0;
  for (i=0; i<nsensor; i++) {
    // get pointer
    mjCSensor* psen = sensors[i];

    // set fields
    m->sensor_type[i] = psen->type;
    m->sensor_datatype[i] = psen->datatype;
    m->sensor_needstage[i] = psen->needstage;
    m->sensor_objtype[i] = psen->objtype;
    m->sensor_objid[i] = psen->objid;
    m->sensor_reftype[i] = psen->reftype;
    m->sensor_refid[i] = psen->refid;
    m->sensor_dim[i] = psen->dim;
    m->sensor_cutoff[i] = (mjtNum)psen->cutoff;
    m->sensor_noise[i] = (mjtNum)psen->noise;
    copyvec(m->sensor_user+nuser_sensor*i, psen->userdata.data(), nuser_sensor);

    // calculate address and advance
    m->sensor_adr[i] = adr;
    adr += psen->dim;
  }

  // numeric fields
  adr = 0;
  for (i=0; i<nnumeric; i++) {
    // get pointer
    mjCNumeric* pcu = numerics[i];

    // set fields
    m->numeric_adr[i] = adr;
    m->numeric_size[i] = pcu->size;
    for (j=0; j<(int)pcu->data.size(); j++) {
      m->numeric_data[adr+j] = (mjtNum)pcu->data[j];
    }
    for (j=(int)pcu->data.size(); j<(int)pcu->size; j++) {
      m->numeric_data[adr+j] = 0;
    }

    // advance address counter
    adr += m->numeric_size[i];
  }

  // text fields
  adr = 0;
  for (i=0; i<ntext; i++) {
    // get pointer
    mjCText* pte = texts[i];

    // set fields
    m->text_adr[i] = adr;
    m->text_size[i] = (int)pte->data.size()+1;
    mju_strncpy(m->text_data + adr, pte->data.c_str(), m->ntextdata - adr);

    // advance address counter
    adr += m->text_size[i];
  }

  // tuple fields
  adr = 0;
  for (i=0; i<ntuple; i++) {
    // get pointer
    mjCTuple* ptu = tuples[i];

    // set fields
    m->tuple_adr[i] = adr;
    m->tuple_size[i] = (int)ptu->objtype.size();
    for (j=0; j<m->tuple_size[i]; j++) {
      m->tuple_objtype[adr+j] = (int)ptu->objtype[j];
      m->tuple_objid[adr+j] = ptu->objid[j];
      m->tuple_objprm[adr+j] = (mjtNum)ptu->objprm[j];
    }

    // advance address counter
    adr += m->tuple_size[i];
  }

  // copy keyframe data
  for (i=0; i<nkey; i++) {
    // copy data
    m->key_time[i] = (mjtNum)keys[i]->time;
    copyvec(m->key_qpos+i*nq, keys[i]->qpos.data(), nq);
    copyvec(m->key_qvel+i*nv, keys[i]->qvel.data(), nv);
    if (na) {
      copyvec(m->key_act+i*na, keys[i]->act.data(), na);
    }
    if (nmocap) {
      copyvec(m->key_mpos + i*3*nmocap, keys[i]->mpos.data(), 3*nmocap);
      copyvec(m->key_mquat + i*4*nmocap, keys[i]->mquat.data(), 4*nmocap);
    }

    // normalize quaternions in m->key_qpos
    for (j=0; j<m->njnt; j++) {
      if (m->jnt_type[j]==mjJNT_BALL || m->jnt_type[j]==mjJNT_FREE) {
        mju_normalize4(m->key_qpos+i*nq+m->jnt_qposadr[j]+3*(m->jnt_type[j]==mjJNT_FREE));
      }
    }

    // normalize quaternions in m->key_mquat
    for (j=0; j<nmocap; j++) {
      mju_normalize4(m->key_mquat+i*4*nmocap+4*j);
    }

    copyvec(m->key_ctrl+i*nu, keys[i]->ctrl.data(), nu);
  }

  // save qpos0 in user model (to recognize changed key_qpos in write)
  qpos0.resize(nq);
  mju_copy(qpos0.data(), m->qpos0, nq);
}



//------------------------------- FUSE STATIC ------------------------------------------------------

// change frame to parent body
static void changeframe(double childpos[3], double childquat[4],
                        const double bodypos[3], const double bodyquat[4]) {
  double pos[3], quat[4];
  mjuu_copyvec(pos, bodypos, 3);
  mjuu_copyvec(quat, bodyquat, 4);
  mjuu_frameaccum(pos, quat, childpos, childquat);
  mjuu_copyvec(childpos, pos, 3);
  mjuu_copyvec(childquat, quat, 4);
}



// reindex elements during fuse
void mjCModel::FuseReindex(mjCBody* body) {
  size_t i;

  // set parentid and weldid of children
  for (i=0; i<body->bodies.size(); i++) {
    body->bodies[i]->parentid = body->id;
    body->bodies[i]->weldid = (!body->bodies[i]->joints.empty() ?
                               body->bodies[i]->id : body->weldid);
  }

  // joints
  for (i=0; i<body->joints.size(); i++) {
    body->joints[i]->id = (int)joints.size();
    joints.push_back(body->joints[i]);
  }

  // geoms
  for (i=0; i<body->geoms.size(); i++) {
    body->geoms[i]->id = (int)geoms.size();
    geoms.push_back(body->geoms[i]);
  }

  // sites
  for (i=0; i<body->sites.size(); i++) {
    body->sites[i]->id = (int)sites.size();
    sites.push_back(body->sites[i]);
  }

  // process children recursively
  for (i=0; i<body->bodies.size(); i++) {
    FuseReindex(body->bodies[i]);
  }
}



// fuse static bodies with their parent
void mjCModel::FuseStatic(void) {
  int i, j, k;

  // skip if model has potential to reference elements with changed ids
  if (!skins.empty()        ||
      !pairs.empty()        ||
      !excludes.empty()     ||
      !equalities.empty()   ||
      !tendons.empty()      ||
      !actuators.empty()    ||
      !sensors.empty()      ||
      !tuples.empty()       ||
      !cameras.empty()      ||
      !lights.empty()) {
    return;
  }

  // process fusable bodies
  for (i=1; i<bodies.size(); i++) {
    // get body and parent
    mjCBody* body = bodies[i];
    mjCBody* par = bodies[body->parentid];

    // skip if body has joints or mocap
    if (!body->joints.empty() || body->mocap) {
      continue;
    }

    //------------- add mass and inertia (if parent not world)

    if (body->parentid>0 && body->mass>=mjMINVAL) {
      // body_ipose = body_pose * body_ipose
      changeframe(body->locipos, body->lociquat,
                  body->locpos, body->locquat);

      // organize data
      double mass[2] = {
        par->mass,
        body->mass
      };
      double inertia[2][3] = {
        {par->inertia[0], par->inertia[1], par->inertia[2]},
        {body->inertia[0], body->inertia[1], body->inertia[2]}
      };
      double ipos[2][3] = {
        {par->locipos[0], par->locipos[1], par->locipos[2]},
        {body->locipos[0], body->locipos[1], body->locipos[2]}
      };
      double iquat[2][4] = {
        {par->lociquat[0], par->lociquat[1], par->lociquat[2], par->lociquat[3]},
        {body->lociquat[0], body->lociquat[1], body->lociquat[2], body->lociquat[3]}
      };

      // compute total mass
      par->mass = 0;
      mjuu_setvec(par->locipos, 0, 0, 0);
      for (j=0; j<2; j++) {
        par->mass += mass[j];
        par->locipos[0] += mass[j]*ipos[j][0];
        par->locipos[1] += mass[j]*ipos[j][1];
        par->locipos[2] += mass[j]*ipos[j][2];
      }

      // small mass: allow for now, check for errors later
      if (par->mass<mjMINVAL) {
        par->mass = 0;
        mjuu_setvec(par->inertia, 0, 0, 0);
        mjuu_setvec(par->locipos, 0, 0, 0);
        mjuu_setvec(par->lociquat, 1, 0, 0, 0);
      }

      // proceed with regular computation
      else {
        // locipos = center-of-mass
        par->locipos[0] /= par->mass;
        par->locipos[1] /= par->mass;
        par->locipos[2] /= par->mass;

        // add inertias
        double toti[6] = {0, 0, 0, 0, 0, 0};
        for (j=0; j<2; j++) {
          double inertA[6], inertB[6];
          double dpos[3] = {
            ipos[j][0] - par->locipos[0],
            ipos[j][1] - par->locipos[1],
            ipos[j][2] - par->locipos[2]
          };

          mjuu_globalinertia(inertA, inertia[j], iquat[j]);
          mjuu_offcenter(inertB, mass[j], dpos);
          for (k=0; k<6; k++) {
            toti[k] += inertA[k] + inertB[k];
          }
        }

        // compute principal axes of inertia
        mjCAlternative alt;
        mjuu_copyvec(alt.fullinertia, toti, 6);
        const char* err1 = alt.Set(par->lociquat, par->inertia, degree, euler);
        if (err1) {
          throw mjCError(NULL, "error '%s' in fusing static body inertias", err1);
        }
      }
    }

    //------------- replace body with its children in parent body list

    // change frames of child bodies
    for (j=0; j<body->bodies.size(); j++)
      changeframe(body->bodies[j]->locpos, body->bodies[j]->locquat,
                  body->locpos, body->locquat);

    // find body in parent list, insert children before it
    bool found = false;
    for (auto iter=par->bodies.begin(); iter!=par->bodies.end(); iter++) {
      if (*iter==body) {
        par->bodies.insert(iter, body->bodies.begin(), body->bodies.end());
        found = true;
        break;
      }
    }
    if (!found) {
      mju_error("Internal error: FuseStatic: body not found");
    }

    // find body in parent list, erase
    found = false;
    for (auto iter=par->bodies.begin(); iter!=par->bodies.end(); iter++) {
      if (*iter==body) {
        par->bodies.erase(iter);
        found = true;
        break;
      }
    }
    if (!found) {
      mju_error("Internal error: FuseStatic: body not found");
    }

    //------------- assign geoms and sites to parent, change frames

    // geoms
    for (j=0; j<body->geoms.size(); j++) {
      // assign
      body->geoms[j]->body = par;
      par->geoms.push_back(body->geoms[j]);

      // change frame
      changeframe(body->geoms[j]->locpos, body->geoms[j]->locquat, body->locpos, body->locquat);
    }

    // sites
    for (j=0; j<body->sites.size(); j++) {
      // assign
      body->sites[j]->body = par;
      par->sites.push_back(body->sites[j]);

      // change frame
      changeframe(body->sites[j]->locpos, body->sites[j]->locquat, body->locpos, body->locquat);
    }

    //------------- remove from global body list, reduce global counts

    // find in global and erase
    found = false;
    for (auto iter=bodies.begin(); iter!=bodies.end(); iter++) {
      if (*iter==body) {
        bodies.erase(iter);
        found = true;
        break;
      }
    }
    if (!found) {
      mju_error("Internal error: FuseStatic: body not found");
    }

    // reduce counts
    nbody--;
    nnames -= ((int)body->name.length() + 1);

    //------------- re-index bodies, joints, geoms, sites

    // body ids
    for (j=0; j<bodies.size(); j++) {
      bodies[j]->id = j;
    }

    // everything else
    joints.clear();
    geoms.clear();
    sites.clear();
    FuseReindex(bodies[0]);

    //------------- delete body (without deleting children)

    // delete allocation
    body->bodies.clear();
    body->geoms.clear();
    body->sites.clear();
    delete body;

    // check index i again (we have a new body at this index)
    i--;
  }
}




//------------------------------- COMPILER ---------------------------------------------------------

// signature comparisons
static int comparePair(mjCPair* el1, mjCPair* el2) {
  return el1->GetSignature() < el2->GetSignature();
}
static int compareBodyPair(mjCBodyPair* el1, mjCBodyPair* el2) {
  return el1->GetSignature() < el2->GetSignature();
}


// reassign ids
template <class T>
static void reassignid(vector<T*>& list) {
  for (int i=0; i<(int)list.size(); i++) {
    list[i]->id = i;
  }
}


// set ids, check for repeated names
template <class T>
static void processlist(vector<T*>& list, string defname, bool checkrepeat=true) {
  // loop over list elements
  for (int i=0; i<(int)list.size(); i++) {
    // check for incompatible id setting; SHOULD NOT OCCUR
    if (list[i]->id!=-1 && list[i]->id!=i) {
      throw mjCError(list[i], "incompatible id in %s array, position %d", defname.c_str(), i);
    }

    // id equals position in array
    list[i]->id = i;

    // compare to all previous names
    if (checkrepeat) {
      for (int j=0; j<i; j++) {
        if (list[i]->name == list[j]->name && list[j]->name != "") {
          throw mjCError(list[i], "repeated name in %s array, position %d", defname.c_str(), i);
        }
      }
    }
  }
}


// error handler for low-level engine
static thread_local std::jmp_buf error_jmp_buf;
static thread_local char errortext[500] = "";
static void errorhandler(const char* msg) {
  mju::strcpy_arr(errortext, msg);
  std::longjmp(error_jmp_buf, 1);
}


// warning handler for low-level engine
static thread_local char warningtext[500] = "";
static void warninghandler(const char* msg) {
  mju::strcpy_arr(warningtext, msg);
}


// compiler
mjModel* mjCModel::Compile(const mjVFS* vfs) {
  // The volatile keyword is necessary to prevent a possible memory leak due to
  // an interaction between longjmp and compiler optimization. Specifically, at
  // the point where the setjmp takes places, these pointers have never been
  // reassigned from their nullptr initialization. Without the volatile keyword,
  // the compiler is free to assume that these pointers remain nullptr when the
  // setjmp returns, and therefore to pass nullptr directly to the
  // mj_deleteModel and mj_deleteData calls in the subsequent catch block,
  // without ever reading the actual pointer values.
  mjModel* volatile m = nullptr;
  mjData* volatile data = nullptr;

  // save error and warning handlers
  void (*save_error)(const char*) = _mjPRIVATE__get_tls_error_fn();
  void (*save_warning)(const char*) = _mjPRIVATE__get_tls_warning_fn();

  // install error and warning handlers, clear error and warning
  _mjPRIVATE__set_tls_error_fn(errorhandler);
  _mjPRIVATE__set_tls_warning_fn(warninghandler);

  errInfo = mjCError();
  warningtext[0] = 0;

  // init random number generator, to make textures reproducible
  srand(123);

  try {
    if (setjmp(error_jmp_buf) != 0) {
      // TryCompile resulted in an mju_error which was converted to a longjmp.
      throw mjCError(0, "engine error: %s", errortext);
    }
    TryCompile(*const_cast<mjModel**>(&m), *const_cast<mjData**>(&data), vfs);
  } catch (mjCError err) {
    // deallocate everything allocated in Compile
    mj_deleteModel(m);
    mj_deleteData(data);
    mjCBody* world = bodies[0];
    Clear();
    bodies.push_back(world);

    // save error info
    errInfo = err;

    // restore handler, return 0
    _mjPRIVATE__set_tls_error_fn(save_error);
    _mjPRIVATE__set_tls_warning_fn(save_warning);
    return nullptr;
  }

  // restore error handler, mark as compiled, return mjModel
  _mjPRIVATE__set_tls_error_fn(save_error);
  _mjPRIVATE__set_tls_warning_fn(save_warning);
  compiled = true;
  return m;
}


void mjCModel::TryCompile(mjModel*& m, mjData*& d, const mjVFS* vfs) {
  // check if nan test works
  double test = mjNAN;
  if (mjuu_defined(test)) {
    throw mjCError(0, "NaN test does not work for present compiler/options");
  }

  // check for repeated compilation
  if (compiled) {
    throw mjCError(0, "model already compiled");
  }

  // check for joints in world body
  if (!bodies[0]->joints.empty()) {
    throw mjCError(0, "joint found in world body");
  }

  // append directory separator
  if (!meshdir.empty()) {
    int n = meshdir.length();
    if (meshdir[n-1]!='/' && meshdir[n-1]!='\\') {
      meshdir += '/';
    }
  }
  if (!texturedir.empty()) {
    int n = texturedir.length();
    if (texturedir[n-1]!='/' && texturedir[n-1]!='\\') {
      texturedir += '/';
    }
  }

  // add missing keyframes
  for (int i=keys.size(); i<nkey; i++) {
    AddKey();
  }

  // make lists of objects created in kinematic tree
  MakeLists(bodies[0]);

  // set object ids and default names, check for repeated names
  processlist(bodies, "body");
  processlist(joints, "joint");
  processlist(geoms, "geom");
  processlist(sites, "site");
  processlist(cameras, "camera");
  processlist(lights, "light");
  processlist(meshes, "mesh");
  processlist(skins, "skin");
  processlist(hfields, "hfield");
  processlist(textures, "texture");
  processlist(materials, "material");
  processlist(pairs, "pair");
  processlist(excludes, "exclude");
  processlist(equalities, "equality");
  processlist(tendons, "tendon");
  processlist(actuators, "actuator");
  processlist(sensors, "sensor");
  processlist(numerics, "numeric");
  processlist(texts, "text");
  processlist(tuples, "tuple");
  processlist(keys, "key");

  // set default names, convert names into indices
  SetDefaultNames();
  IndexAssets();

  // mark meshes that need convex hull
  for (int i=0; i<geoms.size(); i++) {
    if (geoms[i]->meshid>=0 && geoms[i]->type==mjGEOM_MESH &&
        (geoms[i]->contype || geoms[i]->conaffinity)) {
      meshes[geoms[i]->meshid]->needhull = true;
    }
  }

  // compile meshes (needed for geom compilation)
  for (int i=0; i<meshes.size(); i++) {
    meshes[i]->Compile(vfs);
  }

  // automatically set nuser fields
  if (nuser_body == -1) {
    nuser_body = 0;
    for (int i=0; i<bodies.size(); i++) {
      nuser_body = mjMAX(nuser_body, bodies[i]->userdata.size());
    }
  }
  if (nuser_jnt == -1) {
    nuser_jnt = 0;
    for (int i=0; i<joints.size(); i++) {
      nuser_jnt = mjMAX(nuser_jnt, joints[i]->userdata.size());
    }
  }
  if (nuser_geom == -1) {
    nuser_geom = 0;
    for (int i=0; i<geoms.size(); i++) {
      nuser_geom = mjMAX(nuser_geom, geoms[i]->userdata.size());
    }
  }
  if (nuser_site == -1) {
    nuser_site = 0;
    for (int i=0; i<sites.size(); i++) {
      nuser_site = mjMAX(nuser_site, sites[i]->userdata.size());
    }
  }
  if (nuser_cam == -1) {
    nuser_cam = 0;
    for (int i=0; i<cameras.size(); i++) {
      nuser_cam = mjMAX(nuser_cam, cameras[i]->userdata.size());
    }
  }
  if (nuser_tendon == -1) {
    nuser_tendon = 0;
    for (int i=0; i<tendons.size(); i++) {
      nuser_tendon = mjMAX(nuser_tendon, tendons[i]->userdata.size());
    }
  }
  if (nuser_actuator == -1) {
    nuser_actuator = 0;
    for (int i=0; i<actuators.size(); i++) {
      nuser_actuator = mjMAX(nuser_actuator, actuators[i]->userdata.size());
    }
  }
  if (nuser_sensor == -1) {
    nuser_sensor = 0;
    for (int i=0; i<sensors.size(); i++) {
      nuser_sensor = mjMAX(nuser_sensor, sensors[i]->userdata.size());
    }
  }

  // compile objects in kinematic tree
  for (int i=0; i<bodies.size(); i++) {
    bodies[i]->Compile();  // also compiles joints, geoms, sites, cameras, lights
  }

  // compile all other objects except for keyframes
  for (int i=0; i<skins.size(); i++) skins[i]->Compile(vfs);
  for (int i=0; i<hfields.size(); i++) hfields[i]->Compile(vfs);
  for (int i=0; i<textures.size(); i++) textures[i]->Compile(vfs);
  for (int i=0; i<materials.size(); i++) materials[i]->Compile();
  for (int i=0; i<pairs.size(); i++) pairs[i]->Compile();
  for (int i=0; i<excludes.size(); i++) excludes[i]->Compile();
  for (int i=0; i<equalities.size(); i++) equalities[i]->Compile();
  for (int i=0; i<tendons.size(); i++) tendons[i]->Compile();
  for (int i=0; i<actuators.size(); i++) actuators[i]->Compile();
  for (int i=0; i<sensors.size(); i++) sensors[i]->Compile();
  for (int i=0; i<numerics.size(); i++) numerics[i]->Compile();
  for (int i=0; i<texts.size(); i++) texts[i]->Compile();
  for (int i=0; i<tuples.size(); i++) tuples[i]->Compile();

  // compile defaults: to enforce userdata length for writer
  for (int i=0; i<defaults.size(); i++) {
    defaults[i]->Compile(this);
  }

  // sort pair, exclude in increasing signature order; reassign ids
  sort(pairs.begin(), pairs.end(), comparePair);
  sort(excludes.begin(), excludes.end(), compareBodyPair);
  reassignid(pairs);
  reassignid(excludes);

  // resolve asset references, compute sizes
  IndexAssets();
  SetSizes();

  // fuse static if enabled
  if (fusestatic) {
    FuseStatic();
  }

  // set nmocap and body.mocapid
  for (int i=0; i<bodies.size(); i++) {
    if (bodies[i]->mocap) {
      bodies[i]->mocapid = nmocap;
      nmocap++;
    } else {
      bodies[i]->mocapid = -1;
    }
  }

  // check body mass and inertia
  for (int i=1; i<bodies.size(); i++) {
    mjCBody* b = bodies[i];

    // find moving body with small mass or inertia
    if (!b->joints.empty() &&
        (b->mass<mjMINVAL ||
          b->inertia[0]<mjMINVAL ||
          b->inertia[1]<mjMINVAL ||
          b->inertia[2]<mjMINVAL)) {
      // does it have static children with mass and inertia
      bool ok = false;
      for (size_t j=0; j<b->bodies.size(); j++) {
        if (b->bodies[j]->joints.empty() &&
            b->bodies[j]->mass>=mjMINVAL &&
            b->bodies[j]->inertia[0]>=mjMINVAL &&
            b->bodies[j]->inertia[1]>=mjMINVAL &&
            b->bodies[j]->inertia[2]>=mjMINVAL) {
          ok = true;
          break;
        }
      }

      // error
      if (!ok) {
        throw mjCError(b, "mass and inertia of moving bodies must be larger than mjMINVAL");
      }
    }
  }

  // create low-level model
  m = mj_makeModel(nq, nv, nu, na, nbody, njnt, ngeom, nsite, ncam, nlight,
                   nmesh, nmeshvert, nmeshtexvert, nmeshface, nmeshgraph,
                   nskin, nskinvert, nskintexvert, nskinface, nskinbone, nskinbonevert,
                   nhfield, nhfielddata, ntex, ntexdata, nmat, npair, nexclude,
                   neq, ntendon, nwrap, nsensor,
                   nnumeric, nnumericdata, ntext, ntextdata,
                   ntuple, ntupledata, nkey, nmocap,
                   nuser_body, nuser_jnt, nuser_geom, nuser_site, nuser_cam,
                   nuser_tendon, nuser_actuator, nuser_sensor, nnames);
  if (!m) {
    throw mjCError(0, "could not create mjModel");
  }

  // copy everything into low-level model
  m->opt = option;
  m->vis = visual;
  CopyNames(m);
  CopyTree(m);

  // keyframe compilation needs access to nq, nv, na, nmocap, qpos0
  for (int i=0; i<keys.size(); i++) {
    keys[i]->Compile(m);
  }

  // copy objects outsite kinematic tree (including keyframes)
  CopyObjects(m);

  // scale mass
  if (settotalmass>0) {
    mj_setTotalmass(m, settotalmass);
  }

  // set stack size: user-specified or conservative heuristic
  if (nstack>0) {
    m->nstack = nstack;
  } else {
    m->nstack = mjMAX(
        1000,
        5*(m->njmax + m->neq + m->nv)*(m->njmax + m->neq + m->nv) +
        20*(m->nq + m->nv + m->nu + m->na + m->nbody + m->njnt +
            m->ngeom + m->nsite + m->neq + m->ntendon +  m->nwrap));
  }

  // create data
  d = mj_makeData(m);
  if (!d) {
    throw mjCError(0, "could not create mjData");
  }

  // normalize keyframe quaternions
  for (int i=0; i<m->nkey; i++) {
    mj_normalizeQuat(m, m->key_qpos+i*m->nq);
  }

  // set constant fields
  mj_setConst(m, d);

  // automatic spring-damper adjustment
  AutoSpringDamper(m);

  // actuator lengthrange computation
  LengthRange(m, d);

  // override model statistics if defined by user
  if (mjuu_defined(extent)) m->stat.extent = (mjtNum)extent;
  if (mjuu_defined(meaninertia)) m->stat.meaninertia = (mjtNum)meaninertia;
  if (mjuu_defined(meanmass)) m->stat.meanmass = (mjtNum)meanmass;
  if (mjuu_defined(meansize)) m->stat.meansize = (mjtNum)meansize;
  if (mjuu_defined(center[0])) copyvec(m->stat.center, center, 3);

  // assert that model has valid references
  const char* validationerr = mj_validateReferences(m);
  if (validationerr) {  // SHOULD NOT OCCUR
    throw mjCError(0, validationerr);
  }
  // test forward simulation
  mj_resetData(m, d);
  mj_step(m, d);

  // delete data
  mj_deleteData(d);
  d = nullptr;

  // pass warning back
  if (warningtext[0]) {
    mju::strcpy_arr(errInfo.message, warningtext);
    errInfo.warning = true;
  }
}



//------------------------------- DECOMPILER -------------------------------------------------------

// get numeric data back from mjModel
bool mjCModel::CopyBack(const mjModel* m) {
  int i, j;

  // check for null pointer
  if (!m) {
    errInfo = mjCError(0, "mjModel pointer is null in CopyBack");
    return false;
  }

  // make sure model has been compiled
  if (!compiled) {
    errInfo = mjCError(0, "mjCModel has not been compiled in CopyBack");
    return false;
  }

  // make sure sizes match
  if (nq!=m->nq || nv!=m->nv || nu!=m->nu || na!=m->na ||
      nbody!=m->nbody ||njnt!=m->njnt || ngeom!=m->ngeom || nsite!=m->nsite ||
      ncam!=m->ncam || nlight != m->nlight || nmesh!=m->nmesh ||
      nskin!=m->nskin || nhfield!=m->nhfield ||
      nmat != m->nmat || ntex != m->ntex || npair!=m->npair || nexclude!=m->nexclude ||
      neq!=m->neq || ntendon!=m->ntendon || nwrap!=m->nwrap || nsensor!=m->nsensor ||
      nnumeric!=m->nnumeric || nnumericdata!=m->nnumericdata || ntext!=m->ntext ||
      ntextdata!=m->ntextdata || nnames!=m->nnames || nM!=m->nM || nD!=m->nD ||
      nemax!=m->nemax || nconmax!=m->nconmax || njmax!=m->njmax) {
    errInfo = mjCError(0, "incompatible models in CopyBack");
    return false;
  }

  // option and visual
  option = m->opt;
  visual = m->vis;

  // qpos0, qpos_spring
  for (i=0; i<njnt; i++) {
    switch (joints[i]->type) {
    case mjJNT_FREE:
      copyvec(bodies[m->jnt_bodyid[i]]->pos, m->qpos0+m->jnt_qposadr[i], 3);
      copyvec(bodies[m->jnt_bodyid[i]]->quat, m->qpos0+m->jnt_qposadr[i]+3, 4);
      break;

    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      joints[i]->ref = (double)m->qpos0[m->jnt_qposadr[i]];
      joints[i]->springref = (double)m->qpos_spring[m->jnt_qposadr[i]];
      break;

    case mjJNT_BALL:
      // nothing to do, qpos = unit quaternion always
      break;
    }
  }
  mju_copy(qpos0.data(), m->qpos0, m->nq);

  // body
  mjCBody* pb;
  for (i=0; i<nbody; i++) {
    pb = bodies[i];

    copyvec(pb->locpos, m->body_pos+3*i, 3);
    copyvec(pb->locquat, m->body_quat+4*i, 4);
    copyvec(pb->locipos, m->body_ipos+3*i, 3);
    copyvec(pb->lociquat, m->body_iquat+4*i, 4);
    pb->mass = (double)m->body_mass[i];
    copyvec(pb->inertia, m->body_inertia+3*i, 3);

    if (nuser_body) {
      copyvec(pb->userdata.data(), m->body_user + nuser_body*i, nuser_body);
    }
  }

  // joint and dof
  mjCJoint* pj;
  for (i=0; i<njnt; i++) {
    pj = joints[i];

    // joint data
    copyvec(pj->locpos, m->jnt_pos+3*i, 3);
    copyvec(pj->locaxis, m->jnt_axis+3*i, 3);
    pj->stiffness = (double)m->jnt_stiffness[i];
    copyvec(pj->range, m->jnt_range+2*i, 2);
    copyvec(pj->solref_limit, m->jnt_solref+mjNREF*i, mjNREF);
    copyvec(pj->solimp_limit, m->jnt_solimp+mjNIMP*i, mjNIMP);
    pj->margin = (double)m->jnt_margin[i];

    if (nuser_jnt) {
      copyvec(pj->userdata.data(), m->jnt_user + nuser_jnt*i, nuser_jnt);
    }

    // dof data
    j = m->jnt_dofadr[i];
    copyvec(pj->solref_friction, m->dof_solref+mjNREF*j, mjNREF);
    copyvec(pj->solimp_friction, m->dof_solimp+mjNIMP*j, mjNIMP);
    pj->armature = (double)m->dof_armature[j];
    pj->damping = (double)m->dof_damping[j];
    pj->frictionloss = (double)m->dof_frictionloss[j];
  }

  // geom
  mjCGeom* pg;
  for (i=0; i<ngeom; i++) {
    pg = geoms[i];

    copyvec(pg->size, m->geom_size+3*i, 3);
    copyvec(pg->locpos, m->geom_pos+3*i, 3);
    copyvec(pg->locquat, m->geom_quat+4*i, 4);
    copyvec(pg->friction, m->geom_friction+3*i, 3);
    copyvec(pg->solref, m->geom_solref+mjNREF*i, mjNREF);
    copyvec(pg->solimp, m->geom_solimp+mjNIMP*i, mjNIMP);
    copyvec(pg->rgba, m->geom_rgba+4*i, 4);
    pg->solmix = (double)m->geom_solmix[i];
    pg->margin = (double)m->geom_margin[i];
    pg->gap = (double)m->geom_gap[i];

    if (nuser_geom) {
      copyvec(pg->userdata.data(), m->geom_user + nuser_geom*i, nuser_geom);
    }
  }

  // sites
  for (i=0; i<nsite; i++) {
    copyvec(sites[i]->size, m->site_size + 3 * i, 3);
    copyvec(sites[i]->locpos, m->site_pos+3*i, 3);
    copyvec(sites[i]->locquat, m->site_quat+4*i, 4);
    copyvec(sites[i]->rgba, m->site_rgba+4*i, 4);

    if (nuser_site) {
      copyvec(sites[i]->userdata.data(), m->site_user + nuser_site*i, nuser_site);
    }
  }

  // cameras
  for (i=0; i<ncam; i++) {
    copyvec(cameras[i]->pos, m->cam_pos+3*i, 3);
    copyvec(cameras[i]->quat, m->cam_quat+4*i, 4);
    cameras[i]->fovy = (double)m->cam_fovy[i];
    cameras[i]->ipd = (double)m->cam_ipd[i];

    if (nuser_cam) {
      copyvec(cameras[i]->userdata.data(), m->cam_user + nuser_cam*i, nuser_cam);
    }
  }

  // lights
  for (i=0; i<nlight; i++) {
    copyvec(lights[i]->pos, m->light_pos+3*i, 3);
    copyvec(lights[i]->dir, m->light_dir+3*i, 3);
    copyvec(lights[i]->attenuation, m->light_attenuation+3*i, 3);
    lights[i]->cutoff = m->light_cutoff[i];
    lights[i]->exponent = m->light_exponent[i];
    copyvec(lights[i]->ambient, m->light_ambient+3*i, 3);
    copyvec(lights[i]->diffuse, m->light_diffuse+3*i, 3);
    copyvec(lights[i]->specular, m->light_specular+3*i, 3);
  }

  // materials
  for (i=0; i<nmat; i++) {
    copyvec(materials[i]->texrepeat, m->mat_texrepeat+2*i, 2);
    materials[i]->emission = m->mat_emission[i];
    materials[i]->specular = m->mat_specular[i];
    materials[i]->shininess = m->mat_shininess[i];
    materials[i]->reflectance = m->mat_reflectance[i];
    copyvec(materials[i]->rgba, m->mat_rgba+4*i, 4);
  }

  // pairs
  for (i=0; i<npair; i++) {
    copyvec(pairs[i]->solref, m->pair_solref+mjNREF*i, mjNREF);
    copyvec(pairs[i]->solimp, m->pair_solimp+mjNIMP*i, mjNIMP);
    pairs[i]->margin = (double)m->pair_margin[i];
    pairs[i]->gap = (double)m->pair_gap[i];
    copyvec(pairs[i]->friction, m->pair_friction+5*i, 5);
  }

  // equality constraints
  for (i=0; i<neq; i++) {
    copyvec(equalities[i]->data, m->eq_data+mjNEQDATA*i, mjNEQDATA);
    copyvec(equalities[i]->solref, m->eq_solref+mjNREF*i, mjNREF);
    copyvec(equalities[i]->solimp, m->eq_solimp+mjNIMP*i, mjNIMP);
  }

  // tendons
  for (i=0; i<ntendon; i++) {
    copyvec(tendons[i]->range, m->tendon_range+2*i, 2);
    copyvec(tendons[i]->solref_limit, m->tendon_solref_lim+mjNREF*i, mjNREF);
    copyvec(tendons[i]->solimp_limit, m->tendon_solimp_lim+mjNIMP*i, mjNIMP);
    copyvec(tendons[i]->solref_friction, m->tendon_solref_fri+mjNREF*i, mjNREF);
    copyvec(tendons[i]->solimp_friction, m->tendon_solimp_fri+mjNIMP*i, mjNIMP);
    copyvec(tendons[i]->rgba, m->tendon_rgba+4*i, 4);
    tendons[i]->width = (double)m->tendon_width[i];
    tendons[i]->margin = (double)m->tendon_margin[i];
    tendons[i]->stiffness = (double)m->tendon_stiffness[i];
    tendons[i]->damping = (double)m->tendon_damping[i];
    tendons[i]->frictionloss = (double)m->tendon_frictionloss[i];

    if (nuser_tendon) {
      copyvec(tendons[i]->userdata.data(), m->tendon_user + nuser_tendon*i, nuser_tendon);
    }
  }

  // actuators
  mjCActuator* pa;
  for (i=0; i<nu; i++) {
    pa = actuators[i];

    copyvec(pa->dynprm, m->actuator_dynprm+i*mjNDYN, mjNDYN);
    copyvec(pa->gainprm, m->actuator_gainprm+i*mjNGAIN, mjNGAIN);
    copyvec(pa->biasprm, m->actuator_biasprm+i*mjNBIAS, mjNBIAS);
    copyvec(pa->ctrlrange, m->actuator_ctrlrange+2*i, 2);
    copyvec(pa->forcerange, m->actuator_forcerange+2*i, 2);
    copyvec(pa->actrange, m->actuator_actrange+2*i, 2);
    copyvec(pa->lengthrange, m->actuator_lengthrange+2*i, 2);
    copyvec(pa->gear, m->actuator_gear+6*i, 6);
    pa->cranklength = (double)m->actuator_cranklength[i];

    if (nuser_actuator) {
      copyvec(pa->userdata.data(), m->actuator_user + nuser_actuator*i, nuser_actuator);
    }
  }

  // sensors
  for (i=0; i<nsensor; i++) {
    sensors[i]->cutoff = (double)m->sensor_cutoff[i];
    sensors[i]->noise = (double)m->sensor_noise[i];

    if (nuser_sensor) {
      copyvec(sensors[i]->userdata.data(), m->sensor_user + nuser_sensor*i, nuser_sensor);
    }
  }

  // numeric data
  for (i=0; i<nnumeric; i++) {
    for (j=0; j<m->numeric_size[i]; j++) {
      numerics[i]->data[j] = (double)m->numeric_data[m->numeric_adr[i]+j];
    }
  }

  // tuple data
  for (i=0; i<ntuple; i++) {
    for (j=0; j<m->tuple_size[i]; j++) {
      tuples[i]->objprm[j] = (double)m->tuple_objprm[m->tuple_adr[i]+j];
    }
  }

  // keyframes
  for (i=0; i<m->nkey; i++) {
    mjCKey* pk = keys[i];

    pk->time = (double)m->key_time[i];
    copyvec(pk->qpos.data(), m->key_qpos + i*nq, nq);
    copyvec(pk->qvel.data(), m->key_qvel + i*nv, nv);
    if (na) {
      copyvec(pk->act.data(), m->key_act + i*na, na);
    }
    if (nmocap) {
      copyvec(pk->mpos.data(), m->key_mpos + i*3*nmocap, 3*nmocap);
      copyvec(pk->mquat.data(), m->key_mquat + i*4*nmocap, 4*nmocap);
    }
    if (nu) {
      copyvec(pk->ctrl.data(), m->key_ctrl + i*nu, nu);
    }
  }

  return true;
}
