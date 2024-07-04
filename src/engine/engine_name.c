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

#include "engine/engine_name.h"

#include <stdint.h>
#include <string.h>

#include <mujoco/mjmodel.h>
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"



//-------------------------- name functions --------------------------------------------------------
// get number of objects and name addresses for given object type
static int _getnumadr(const mjModel* m, mjtObj type, int** padr, int* mapadr) {
  int num = -1;
  // map address starts at the end, subtract with explicit switch fallthrough below
  *mapadr = m->nnames_map;

  // get address list and size for object type
  switch (type) {
  case mjOBJ_BODY:
  case mjOBJ_XBODY:
    *mapadr -= mjLOAD_MULTIPLE*m->nbody;
    *padr = m->name_bodyadr;
    num = m->nbody;
    mjFALLTHROUGH;

  case mjOBJ_JOINT:
    *mapadr -= mjLOAD_MULTIPLE*m->njnt;
    if (num < 0) {
      *padr = m->name_jntadr;
      num = m->njnt;
    }
    mjFALLTHROUGH;

  case mjOBJ_GEOM:
    *mapadr -= mjLOAD_MULTIPLE*m->ngeom;
    if (num < 0) {
      *padr = m->name_geomadr;
      num = m->ngeom;
    }
    mjFALLTHROUGH;

  case mjOBJ_SITE:
    *mapadr -= mjLOAD_MULTIPLE*m->nsite;
    if (num < 0) {
      *padr = m->name_siteadr;
      num = m->nsite;
    }
    mjFALLTHROUGH;

  case mjOBJ_CAMERA:
    *mapadr -= mjLOAD_MULTIPLE*m->ncam;
    if (num < 0) {
      *padr = m->name_camadr;
      num = m->ncam;
    }
    mjFALLTHROUGH;

  case mjOBJ_LIGHT:
    *mapadr -= mjLOAD_MULTIPLE*m->nlight;
    if (num < 0) {
      *padr = m->name_lightadr;
      num = m->nlight;
    }
    mjFALLTHROUGH;

  case mjOBJ_FLEX:
    *mapadr -= mjLOAD_MULTIPLE*m->nflex;
    if (num < 0) {
      *padr = m->name_flexadr;
      num =  m->nflex;
    }
    mjFALLTHROUGH;

  case mjOBJ_MESH:
    *mapadr -= mjLOAD_MULTIPLE*m->nmesh;
    if (num < 0) {
      *padr = m->name_meshadr;
      num =  m->nmesh;
    }
    mjFALLTHROUGH;

  case mjOBJ_SKIN:
    *mapadr -= mjLOAD_MULTIPLE*m->nskin;
    if (num < 0) {
      *padr = m->name_skinadr;
      num = m->nskin;
    }
    mjFALLTHROUGH;

  case mjOBJ_HFIELD:
    *mapadr -= mjLOAD_MULTIPLE*m->nhfield;
    if (num < 0) {
      *padr = m->name_hfieldadr;
      num = m->nhfield;
    }
    mjFALLTHROUGH;

  case mjOBJ_TEXTURE:
    *mapadr -= mjLOAD_MULTIPLE*m->ntex;
    if (num < 0) {
      *padr = m->name_texadr;
      num = m->ntex;
    }
    mjFALLTHROUGH;

  case mjOBJ_MATERIAL:
    *mapadr -= mjLOAD_MULTIPLE*m->nmat;
    if (num < 0) {
      *padr = m->name_matadr;
      num = m->nmat;
    }
    mjFALLTHROUGH;

  case mjOBJ_PAIR:
    *mapadr -= mjLOAD_MULTIPLE*m->npair;
    if (num < 0) {
      *padr = m->name_pairadr;
      num = m->npair;
    }
    mjFALLTHROUGH;

  case mjOBJ_EXCLUDE:
    *mapadr -= mjLOAD_MULTIPLE*m->nexclude;
    if (num < 0) {
      *padr = m->name_excludeadr;
      num = m->nexclude;
    }
    mjFALLTHROUGH;

  case mjOBJ_EQUALITY:
    *mapadr -= mjLOAD_MULTIPLE*m->neq;
    if (num < 0) {
      *padr = m->name_eqadr;
      num = m->neq;
    }
    mjFALLTHROUGH;

  case mjOBJ_TENDON:
    *mapadr -= mjLOAD_MULTIPLE*m->ntendon;
    if (num < 0) {
      *padr = m->name_tendonadr;
      num = m->ntendon;
    }
    mjFALLTHROUGH;

  case mjOBJ_ACTUATOR:
    *mapadr -= mjLOAD_MULTIPLE*m->nu;
    if (num < 0) {
      *padr = m->name_actuatoradr;
      num = m->nu;
    }
    mjFALLTHROUGH;

  case mjOBJ_SENSOR:
    *mapadr -= mjLOAD_MULTIPLE*m->nsensor;
    if (num < 0) {
      *padr = m->name_sensoradr;
      num = m->nsensor;
    }
    mjFALLTHROUGH;

  case mjOBJ_NUMERIC:
    *mapadr -= mjLOAD_MULTIPLE*m->nnumeric;
    if (num < 0) {
      *padr = m->name_numericadr;
      num = m->nnumeric;
    }
    mjFALLTHROUGH;

  case mjOBJ_TEXT:
    *mapadr -= mjLOAD_MULTIPLE*m->ntext;
    if (num < 0) {
      *padr = m->name_textadr;
      num = m->ntext;
    }
    mjFALLTHROUGH;

  case mjOBJ_TUPLE:
    *mapadr -= mjLOAD_MULTIPLE*m->ntuple;
    if (num < 0) {
      *padr = m->name_tupleadr;
      num = m->ntuple;
    }
    mjFALLTHROUGH;

  case mjOBJ_KEY:
    *mapadr -= mjLOAD_MULTIPLE*m->nkey;
    if (num < 0) {
      *padr = m->name_keyadr;
      num = m->nkey;
    }
    mjFALLTHROUGH;

  case mjOBJ_PLUGIN:
    *mapadr -= mjLOAD_MULTIPLE*m->nplugin;
    if (num < 0) {
      *padr = m->name_pluginadr;
      num = m->nplugin;
    }
    mjFALLTHROUGH;

  default:
    if (num < 0) {
      *padr = 0;
      num = 0;
    }
  }

  return num;
}

// get string hash, see http://www.cse.yorku.ca/~oz/hash.html
uint64_t mj_hashString(const char* s, uint64_t n) {
  uint64_t h = 5381;
  int c;
  while ((c = *s++)) {
    h = ((h << 5) + h) ^ c;
  }
  return h % n;
}

// get id of object with the specified mjtObj type and name,
// returns -1 if id not found
int mj_name2id(const mjModel* m, int type, const char* name) {
  int mapadr;
  int* adr = 0;

  // get number of objects and name addresses
  int num = mjLOAD_MULTIPLE*_getnumadr(m, type, &adr, &mapadr);

  // search
  if (num) {    // look up at hash address
    uint64_t hash = mj_hashString(name, num);
    uint64_t i = hash;

    do {
      int j = m->names_map[mapadr + i];
      if (j < 0) {
        return -1;
      }

      if (!strncmp(name, m->names+adr[j], m->nnames-adr[j])) {
        return j;
      }
      if ((++i) == num)i = 0;
    } while (i != hash);
  }
  return -1;
}



// get name of object with the specified mjtObj type and id,
// returns NULL if name not found
const char* mj_id2name(const mjModel* m, int type, int id) {
  int mapadr;
  int* adr = 0;

  // get number of objects and name addresses
  int num = _getnumadr(m, type, &adr, &mapadr);

  // id is in [0, num) and the found name is not the empty string "\0"
  if (id >= 0 && id < num && m->names[adr[id]]) {
    return m->names+adr[id];
  }

  return NULL;
}
