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

#include "engine/engine_print.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjxmacro.h>
#include "engine/engine_core_constraint.h"
#include "engine/engine_io.h"
#include "engine/engine_support.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

#define FLOAT_FORMAT "% -9.2g"
#define FLOAT_FORMAT_MAX_LEN 20
#define INT_FORMAT " %d"
#define SIZE_T_FORMAT " %zu"
#define NAME_FORMAT "%-21s"



//----------------------------------- static utility functions -------------------------------------

// print 2D array of mjtNum into file
static void printArray(const char* str, int nr, int nc, const mjtNum* data, FILE* fp,
                       const char* float_format) {
  if (!data) {
    return;
  }
  if (nr && nc) {
    fprintf(fp, "%s\n", str);
    for (int r=0; r < nr; r++) {
      fprintf(fp, " ");
      for (int c=0; c < nc; c++) {
        fprintf(fp, " ");
        fprintf(fp, float_format, data[c + r*nc]);
      }
      fprintf(fp, "\n");
    }
    fprintf(fp, "\n");
  }
}


// print 2D array of int into file
static void printArrayInt(const char* str, int nr, int nc, const int* data, FILE* fp) {
  if (!data) {
    return;
  }
  if (nr && nc) {
    fprintf(fp, "%s\n", str);
    for (int r=0; r < nr; r++) {
      fprintf(fp, " ");
      for (int c=0; c < nc; c++) {
        fprintf(fp, " ");
        fprintf(fp, "%d", data[c + r*nc]);
      }
      fprintf(fp, "\n");
    }
    fprintf(fp, "\n");
  }
}



// print sparse matrix
static void printSparse(const char* str, const mjtNum* mat, int nr,
                        const int* rownnz, const int* rowadr,
                        const int* colind, FILE* fp, const char* float_format) {
  if (!mat) {
    return;
  }
  fprintf(fp, "%s\n", str);

  for (int r=0; r < nr; r++) {
    fprintf(fp, "  ");
    for (int adr=rowadr[r]; adr < rowadr[r]+rownnz[r]; adr++) {
      fprintf(fp, "  ");
      fprintf(fp, "%d: ", colind[adr]);
      fprintf(fp, float_format, mat[adr]);
    }
    fprintf(fp, "\n");
  }
  fprintf(fp, "\n");
}



// print vector
static void printVector(const char* str, const mjtNum* data, int n, FILE* fp,
                        const char* float_format) {
  if (!data) {
    return;
  }
  // print str
  fprintf(fp, "%s", str);

  // print data
  for (int i=0; i < n; i++) {
    fprintf(fp, " ");
    fprintf(fp, float_format, data[i]);
  }
  fprintf(fp, "\n");
}



//------------------------------ printing functions ------------------------------------------------

// return whether float_format is a valid format string for a single float
static bool validateFloatFormat(const char* float_format) {
  // check for nullptr;
  if (!float_format) {
    return false;
  }

  // example valid format string: "% -9.2g"
  if (strnlen(float_format, FLOAT_FORMAT_MAX_LEN + 1) > FLOAT_FORMAT_MAX_LEN) {
    mju_warning("Format string longer than limit of %d.", FLOAT_FORMAT_MAX_LEN);
    return false;
  }

  int cur_idx = 0;
  if (float_format[cur_idx] != '%') {
    mju_warning("Format string must start with '%%'.");
    return false;
  }
  cur_idx++;

  // flag characters. allow at most one of each flag
  const char flag_characters[] = "-+ #0";
  int flag_character_counts[sizeof(flag_characters)] = { 0 };
  char* c;
  while (c = strchr(flag_characters, float_format[cur_idx]), c != NULL) {
    int flag_idx = (c - flag_characters)/sizeof(char);
    flag_character_counts[flag_idx]++;
    if (flag_character_counts[flag_idx] > 1) {
      mju_warning("Format string contains repeated flag.");
      return false;
    }
    cur_idx++;
  }

  // width. disallow *, which requires additional argument
  while (strchr("0123456789", float_format[cur_idx]) != NULL) {
    cur_idx++;
  }

  // precision. disallow *, which requires additional argument
  if (float_format[cur_idx] == '.') {
    cur_idx++;
    while (strchr("0123456789", float_format[cur_idx]) != NULL) {
      cur_idx++;
    }
  }

  // length
  if (float_format[cur_idx] == 'L') {
    cur_idx++;
  }

  // specifier must be a valid float format
  if (strchr("fgGeE", float_format[cur_idx]) == NULL) {
    mju_warning("Format string specifier must be one of \"fgGeE\".");
    return false;
  }
  cur_idx++;

  if (float_format[cur_idx] == '\0') {
    return true;
  } else {
    mju_warning("Unable to match format string %s with expected pattern for a single float.",
                float_format);
    return false;
  }
}


   // Clang sometimes goes OOM when the -Wuninitialized warning is enabled for this function
 #ifdef __clang__
 #pragma clang diagnostic push
 #pragma clang diagnostic ignored "-Wuninitialized"
 #endif


// print mjModel to text file, specifying format. float_format must be a
// valid printf-style format string for a single float value
void mj_printFormattedModel(const mjModel* m, const char* filename, const char* float_format) {
  // get file
  FILE* fp;
  if (filename) {
    fp = fopen(filename, "wt");
  } else {
    fp = stdout;
  }

  // check for nullptr
  if (!fp) {
    mju_warning("Could not open file '%s' for writing mjModel", filename);
    return;
  }

  // validate format string
  if (!validateFloatFormat(float_format)) {
    mju_warning("WARNING: Received invalid float_format. Using default instead.");
    float_format = FLOAT_FORMAT;
  }

  // compute total body mass
  mjtNum totalmass = 0;
  for (int i=0; i < m->nbody; i++) {
    totalmass += m->body_mass[i];
  }

  // software version and model name
  fprintf(fp, "MuJoCo version %s\n", mj_versionString());
  fprintf(fp, "model name     %s\n\n", m->names);

  // sizes
#define X( name )                                 \
  if (m->name) {                                  \
    fprintf(fp, NAME_FORMAT, #name);              \
    fprintf(fp, INT_FORMAT "\n", m->name);        \
  }

  MJMODEL_INTS
#undef X
  fprintf(fp, "\n");

  // scalar options
#define X( type, name )                           \
  fprintf(fp, NAME_FORMAT, #name);                \
  fprintf(fp, float_format, m->opt.name);         \
  fprintf(fp, "\n");

  MJOPTION_FLOATS
#undef X

#define X( type, name )                           \
  fprintf(fp, NAME_FORMAT, #name);                \
  fprintf(fp, INT_FORMAT "\n", m->opt.name);

  MJOPTION_INTS
#undef X

  // vector options
#define X( name, sz )                             \
  fprintf(fp, NAME_FORMAT, #name);                \
  for (int i=0; i < sz; i++) {                    \
    fprintf(fp, float_format, m->opt.name[i]);    \
    fprintf(fp, " ");                             \
  }                                               \
  fprintf(fp, "\n");

  MJOPTION_VECTORS
#undef X
  fprintf(fp, "\n");

  // total mass
  fprintf(fp, NAME_FORMAT, "totalmass");
  fprintf(fp, float_format, totalmass);
  fprintf(fp, "\n\n");

  // statistics
  fprintf(fp, NAME_FORMAT, "meaninertia");
  fprintf(fp, float_format, m->stat.meaninertia);
  fprintf(fp, "\n");
  fprintf(fp, NAME_FORMAT, "meanmass");
  fprintf(fp, float_format, m->stat.meanmass);
  fprintf(fp, "\n");
  fprintf(fp, NAME_FORMAT, "meansize");
  fprintf(fp, float_format, m->stat.meansize);
  fprintf(fp, "\n");
  fprintf(fp, NAME_FORMAT, "extent");
  fprintf(fp, float_format, m->stat.extent);
  fprintf(fp, "\n");
  fprintf(fp, NAME_FORMAT, "center");
  fprintf(fp, float_format, m->stat.center[0]);
  fprintf(fp, float_format, m->stat.center[1]);
  fprintf(fp, float_format, m->stat.center[2]);
  fprintf(fp, "\n\n");

  // qpos0
  fprintf(fp, NAME_FORMAT, "qpos0");
  for (int i=0; i < m->nq; i++) {
    fprintf(fp, float_format, m->qpos0[i]);
    fprintf(fp, " ");
  }
  fprintf(fp, "\n\n");

  // qpos_spring
  fprintf(fp, NAME_FORMAT, "qpos_spring");
  for (int i=0; i < m->nq; i++) {
    fprintf(fp, float_format, m->qpos_spring[i]);
    fprintf(fp, " ");
  }
  fprintf(fp, "\n\n");

  // values used by MJMODEL_POINTERS macro
  MJMODEL_POINTERS_PREAMBLE(m)

  // object_class points to the integer size identifying the class of arrays currently being printed
  //   used to organise the printout into category groups
  //   note that comparison is based on the integer address, not its value
  const int* object_class;

#define X(type, name, num, sz)                                              \
  if (&m->num == object_class && (strncmp(#name, "name_", 5) != 0) && sz) { \
    const char* format = _Generic(*m->name,                                 \
                                  double:  float_format,                    \
                                  float:   float_format,                    \
                                  int:     INT_FORMAT,                      \
                                  mjtByte: INT_FORMAT,                      \
                                  default: NULL);                           \
    if (format) {                                                           \
      fprintf(fp, "  ");                                                    \
      fprintf(fp, NAME_FORMAT, #name);                                      \
      for (int j = 0; j < sz; j++) {                                        \
          fprintf(fp, format, m->name[sz * i + j]);                         \
          fprintf(fp, " ");                                                 \
      }                                                                     \
      fprintf(fp, "\n");                                                    \
    }                                                                       \
  }

  // bodies
  for (int i=0; i < m->nbody; i++) {
    fprintf(fp, "\nBODY %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_bodyadr[i]);
    object_class = &m->nbody;
    MJMODEL_POINTERS
  }
  if (m->nbody) fprintf(fp, "\n");

  // joints
  for (int i=0; i < m->njnt; i++) {
    fprintf(fp, "\nJOINT %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_jntadr[i]);
    object_class = &m->njnt;
    MJMODEL_POINTERS
  }
  if (m->njnt) fprintf(fp, "\n");

  // dofs
  for (int i=0; i < m->nv; i++) {
    fprintf(fp, "\nDOF %d:\n", i);
    object_class = &m->nv;
    MJMODEL_POINTERS
  }
  if (m->nv) fprintf(fp, "\n");

  // geoms
  for (int i=0; i < m->ngeom; i++) {
    fprintf(fp, "\nGEOM %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_geomadr[i]);
    object_class = &m->ngeom;
    MJMODEL_POINTERS
  }
  if (m->ngeom) fprintf(fp, "\n");

  // sites
  for (int i=0; i < m->nsite; i++) {
    fprintf(fp, "\nSITE %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_siteadr[i]);
    object_class = &m->nsite;
    MJMODEL_POINTERS
  }
  if (m->nsite) fprintf(fp, "\n");

  // cameras
  for (int i=0; i < m->ncam; i++) {
    fprintf(fp, "\nCAMERA %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_camadr[i]);
    object_class = &m->ncam;
    MJMODEL_POINTERS
  }
  if (m->ncam) fprintf(fp, "\n");

  // lights
  for (int i=0; i < m->nlight; i++) {
    fprintf(fp, "\nLIGHT %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_lightadr[i]);
    object_class = &m->nlight;
    MJMODEL_POINTERS
  }
  if (m->nlight) fprintf(fp, "\n");

  // meshes
  for (int i=0; i < m->nmesh; i++) {
    fprintf(fp, "\nMESH %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_meshadr[i]);
    object_class = &m->nmesh;
    MJMODEL_POINTERS
    if (m->mesh_graphadr[i] >= 0) {
      fprintf(fp, "  " NAME_FORMAT, "qhull face");
      fprintf(fp, " %d\n", m->mesh_graph[m->mesh_graphadr[i]+1]);
      fprintf(fp, "  " NAME_FORMAT, "qhull vert");
      fprintf(fp, " %d\n", m->mesh_graph[m->mesh_graphadr[i]]);
    }
  }
  if (m->nmesh) fprintf(fp, "\n");

  // skins
  for (int i=0; i < m->nskin; i++) {
    fprintf(fp, "\nSKIN %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_skinadr[i]);
    object_class = &m->nskin;
    MJMODEL_POINTERS
  }
  if (m->nskin) fprintf(fp, "\n");

  // hfields
  for (int i=0; i < m->nhfield; i++) {
    fprintf(fp, "\nHEIGHTFIELD %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, "  %s\n", m->names + m->name_hfieldadr[i]);
    object_class = &m->nhfield;
    MJMODEL_POINTERS
  }
  if (m->nhfield) fprintf(fp, "\n");

  // textures
  for (int i=0; i < m->ntex; i++) {
    fprintf(fp, "\nTEXTURE %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_texadr[i]);
    object_class = &m->ntex;
    MJMODEL_POINTERS
  }
  if (m->ntex) fprintf(fp, "\n");

  // materials
  for (int i=0; i < m->nmat; i++) {
    fprintf(fp, "\nMATERIAL %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_matadr[i]);
    object_class = &m->nmat;
    MJMODEL_POINTERS
  }
  if (m->nmat) fprintf(fp, "\n");

  // pairs
  for (int i=0; i < m->npair; i++) {
    fprintf(fp, "\nPAIR %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_pairadr[i]);
    object_class = &m->npair;
    MJMODEL_POINTERS
  }
  if (m->npair) fprintf(fp, "\n");

  // excludes
  for (int i=0; i < m->nexclude; i++) {
    fprintf(fp, "\nEXCLUDE %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_excludeadr[i]);
    object_class = &m->nexclude;
    MJMODEL_POINTERS
  }
  if (m->nexclude) fprintf(fp, "\n");

  // equality constraints
  for (int i=0; i < m->neq; i++) {
    fprintf(fp, "\nEQUALITY %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_eqadr[i]);
    object_class = &m->neq;
    MJMODEL_POINTERS
  }
  if (m->neq) fprintf(fp, "\n");

  // tendons
  for (int i=0; i < m->ntendon; i++) {
    fprintf(fp, "\nTENDON %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_tendonadr[i]);
    object_class = &m->ntendon;
    MJMODEL_POINTERS
    fprintf(fp, "  path         \n");
    for (int j=0; j < m->tendon_num[i]; j++) {
      int k = m->tendon_adr[i]+j;
      fprintf(fp, "    %d %d ", m->wrap_type[k], m->wrap_objid[k]);
      fprintf(fp, float_format, m->wrap_prm[k]);
      fprintf(fp, "\n");
    }
    fprintf(fp, "\n");
  }
  if (m->ntendon) fprintf(fp, "\n");

  // actuators
  for (int i=0; i < m->nu; i++) {
    fprintf(fp, "\nACTUATOR %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_actuatoradr[i]);
    object_class = &m->nu;
    MJMODEL_POINTERS
  }
  if (m->nu) fprintf(fp, "\n");

  // sensors
  for (int i=0; i < m->nsensor; i++) {
    fprintf(fp, "\nSENSOR %d:\n", i);
    fprintf(fp, "  " NAME_FORMAT, "name");
    fprintf(fp, " %s\n", m->names + m->name_sensoradr[i]);
    object_class = &m->nsensor;
    MJMODEL_POINTERS
  }
  if (m->nsensor) fprintf(fp, "\n");

  // custom numeric parameters
  for (int i=0; i < m->nnumeric; i++) {
    fprintf(fp, "\nNUMERIC %d:\n", i);
    fprintf(fp, "  name         %s\n", m->names + m->name_numericadr[i]);
    fprintf(fp, "  size         %d\n", m->numeric_size[i]);
    fprintf(fp, "  value       ");
    for (int j=0; j < m->numeric_size[i]; j++) {
      fprintf(fp, float_format, m->numeric_data[m->numeric_adr[i]+j]);
    }
    fprintf(fp, "\n");
  }
  if (m->nnumeric) fprintf(fp, "\n");

  // custom text parameters
  for (int i=0; i < m->ntext; i++) {
    fprintf(fp, "\nTEXT %d:\n", i);
    fprintf(fp, "  name         %s\n", m->names + m->name_textadr[i]);
    fprintf(fp, "  size         %d\n", m->text_size[i]);
    fprintf(fp, "  value        %s\n", m->text_data + m->text_adr[i]);
  }
  if (m->ntext) fprintf(fp, "\n");

  // custom tuple parameters
  for (int i=0; i < m->ntuple; i++) {
    fprintf(fp, "\nTUPLE %d:\n", i);
    fprintf(fp, "  name         %s\n", m->names + m->name_tupleadr[i]);
    fprintf(fp, "  size         %d\n", m->tuple_size[i]);
    fprintf(fp, "  elements\n");
    for (int j=m->tuple_adr[i]; j < m->tuple_adr[i]+m->tuple_size[i]; j++) {
      fprintf(fp, "       %s %d, prm = ",
              mju_type2Str(m->tuple_objtype[j]), m->tuple_objid[j]);
      fprintf(fp, float_format, m->tuple_objprm[j]);
      fprintf(fp, "\n");
    }
  }
  if (m->ntuple) fprintf(fp, "\n");

  // keyframes (only if different from default)
  for (int i=0; i < m->nkey; i++) {
    // print name
    if (m->names[m->name_keyadr[i]]) {
      fprintf(fp, "key_name%d    %s\n", i, m->names + m->name_keyadr[i]);
    }

    // print time if non-0
    if (m->key_time[i] != 0) {
      fprintf(fp, "key_time%d    %.4f\n", i, m->key_time[i]);
    }

    // check qpos for difference
    int k = 0;
    for (int j=0; j < m->nq; j++)
      if (m->qpos0[j] != m->key_qpos[i*m->nq + j]) {
        k = 1;
      }

    // print if different
    if (k == 1) {
      fprintf(fp, "key_qpos%d   ", i);
      for (int j=0; j < m->nq; j++) {
        fprintf(fp, float_format, m->key_qpos[i*m->nq + j]);
      }
      fprintf(fp, "\n");
    }

    // check qvel for nonzero
    for (int j=0; j < m->nv; j++)
      if (m->key_qvel[i*m->nv + j]) {
        k = 2;
      }

    // print if nonzero
    if (k == 2) {
      fprintf(fp, "key_qvel%d   ", i);
      for (int j=0; j < m->nv; j++) {
        fprintf(fp, float_format, m->key_qvel[i*m->nv + j]);
      }
      fprintf(fp, "\n");
    }

    // check act for nonzero
    for (int j=0; j < m->na; j++)
      if (m->key_act[i*m->na + j]) {
        k = 3;
      }

    // print if nonzero
    if (k == 3) {
      fprintf(fp, "key_act%d   ", i);
      for (int j=0; j < m->na; j++) {
        fprintf(fp, float_format, m->key_act[i*m->na + j]);
      }
      fprintf(fp, "\n");
    }

    // check mpos for difference
    if (m->nmocap) {
      for (int j=0; j < m->nbody; j++) {
        if (m->body_mocapid[j] >= 0) {
          int id = m->body_mocapid[j];
          if (m->body_pos[3*j] != m->key_mpos[i*3*m->nmocap + 3*id] ||
              m->body_pos[3*j+1] != m->key_mpos[i*3*m->nmocap + 3*id+1] ||
              m->body_pos[3*j+2] != m->key_mpos[i*3*m->nmocap + 3*id+2]) {
            k = 4;
            break;
          }
        }
      }
    }

    // print if nonzero
    if (k == 4) {
      fprintf(fp, "key_mpos%d   ", i);
      for (int j=0; j < 3*m->nmocap; j++) {
        fprintf(fp, float_format, m->key_mpos[i*3*m->nmocap + j]);
      }
      fprintf(fp, "\n");
    }

    // check mquat for difference
    if (m->nmocap) {
      for (int j=0; j < m->nbody; j++) {
        if (m->body_mocapid[j] >= 0) {
          int id = m->body_mocapid[j];
          if (m->body_quat[4*j] != m->key_mquat[i*4*m->nmocap + 4*id] ||
              m->body_quat[4*j+1] != m->key_mquat[i*4*m->nmocap + 4*id+1] ||
              m->body_quat[4*j+2] != m->key_mquat[i*4*m->nmocap + 4*id+2] ||
              m->body_quat[4*j+3] != m->key_mquat[i*4*m->nmocap + 4*id+3]) {
            k = 5;
            break;
          }
        }
      }
    }

    // print if nonzero
    if (k == 5) {
      fprintf(fp, "key_mquat%d   ", i);
      for (int j=0; j < 4*m->nmocap; j++) {
        fprintf(fp, float_format, m->key_mquat[i*4*m->nmocap + j]);
      }
      fprintf(fp, "\n");
    }

    // check ctrl for nonzero
    for (int j=0; j < m->nu; j++) {
      if (m->key_ctrl[i*m->nu + j]) {
        k = 6;
        break;
      }
    }

    // print if nonzero
    if (k == 6) {
      fprintf(fp, "key_ctrl%d   ", i);
      for (int j=0; j < m->nu; j++) {
        fprintf(fp, float_format, m->key_ctrl[i*m->nu + j]);
      }
      fprintf(fp, "\n");
    }


    // new line if any data was written
    if (k) {
      fprintf(fp, "\n");
    }
  }

#undef X

  if (filename) {
    fclose(fp);
  }
}


// print mjModel to text file
void mj_printModel(const mjModel* m, const char* filename) {
  mj_printFormattedModel(m, filename, FLOAT_FORMAT);
}


// print mjModel to text file, specifying format. float_format must be a
// valid printf-style format string for a single float value
void mj_printFormattedData(const mjModel* m, mjData* d, const char* filename,
                           const char* float_format) {
  mjtNum *M;
  mjMARKSTACK;

  // check format string
  if (!validateFloatFormat(float_format)) {
    mju_warning("WARNING: Received invalid float_format. Using default instead.");
    float_format = FLOAT_FORMAT;
  }

  // stack in use, SHOULD NOT OCCUR
  if (d->pstack) {
    mjERROR("attempting to print mjData when stack is in use");
  }

  // get file
  FILE* fp;
  if (filename) {
    fp = fopen(filename, "wt");
  } else {
    fp = stdout;
  }

  // check for nullptr
  if (!fp) {
    mju_warning("Could not open file '%s' for writing mjModel", filename);
    mjFREESTACK;
    return;
  }

  // allocate full inertia
  M = mj_stackAlloc(d, m->nv*m->nv);

#ifdef MEMORY_SANITIZER
  // If memory sanitizer is active, d->buffer will be marked as poisoned, even
  // though it's really initialized to 0. This catches unintentionally
  // using uninitialized values, but in engine_print it's OK to output zeroes.

  // save current poison status of buffer before marking unpoisoned
  void* shadow = mju_malloc(d->nbuffer);
  __msan_copy_shadow(shadow, d->buffer, d->nbuffer);
  __msan_unpoison(d->buffer, d->nbuffer);
#endif
  // ---------------------------------- print mjData fields

  fprintf(fp, "SIZES\n");
#define X(type, name)                                                         \
  if (strcmp(#name, "pstack") != 0 && strcmp(#name, "parena") != 0) {         \
    const char* format = _Generic(                                            \
        d->name,                                                              \
        int : INT_FORMAT,                                                     \
        size_t : SIZE_T_FORMAT,                                               \
        default : NULL);                                                      \
    if (format) {                                                             \
      fprintf(fp, "  ");                                                      \
      fprintf(fp, NAME_FORMAT, #name);                                        \
      fprintf(fp, format, d->name);                                           \
      fprintf(fp, "\n");                                                      \
    }                                                                         \
  }

  MJDATA_SCALAR
#undef X
  fprintf(fp, "\n");

  // WARNING
  int active_warnings = 0;
  for (int i=0; i < mjNWARNING; i++) {
    active_warnings += d->warning[i].number;
  }
  if (active_warnings) {
    fprintf(fp, "WARNING\n");
    for (int i=0; i < mjNWARNING; i++)
      if (d->warning[i].number)
        fprintf(fp, "    %d:  lastinfo = %d   number = %d\n",
                i, d->warning[i].lastinfo, d->warning[i].number);
    fprintf(fp, "\n");
  }

  // TIMER
  mjtNum active_timers = 0;
  for (int i=0; i < mjNTIMER; i++) {
    active_timers += d->timer[i].duration;
  }
  if (active_timers) {
    fprintf(fp, "TIMER\n");
    for (int i=0; i < mjNTIMER; i++) {
      fprintf(fp, "    %d:  duration = ", i);
      fprintf(fp, float_format, d->timer[i].duration);
      fprintf(fp, "   number = %d\n", d->timer[i].number);
    }
    fprintf(fp, "\n");
  }

  // SOLVER STAT
  if (d->solver_iter) {
    fprintf(fp, "SOLVER STAT\n");
    fprintf(fp, "  solver_iter = %d\n", d->solver_iter);
    fprintf(fp, "  solver_nnz = %d\n",  d->solver_nnz);
    for (int i=0; i < mjMIN(mjNSOLVER, d->solver_iter); i++) {
      fprintf(fp, "    %d:  improvement = ", i);
      fprintf(fp, float_format, d->solver[i].improvement);
      fprintf(fp, "  gradient = ");
      fprintf(fp, float_format, d->solver[i].gradient);
      fprintf(fp, "  lineslope = ");
      fprintf(fp, float_format, d->solver[i].lineslope);
      fprintf(fp, "\n");
      fprintf(fp, "        nactive = %d   nchange = %d   neval = %d   nupdate = %d\n",
              d->solver[i].nactive, d->solver[i].nchange,
              d->solver[i].neval, d->solver[i].nupdate);
    }
    printVector("solver_fwdinv = ", d->solver_fwdinv, 2, fp, float_format);
    fprintf(fp, "\n");
  }

  printVector("ENERGY = ", d->energy, 2, fp, float_format);
  fprintf(fp, "\n");

  fprintf(fp, "TIME = ");
  fprintf(fp, float_format, d->time);
  fprintf(fp, "\n\n");

  printArray("QPOS", m->nq, 1, d->qpos, fp, float_format);
  printArray("QVEL", m->nv, 1, d->qvel, fp, float_format);
  printArray("ACT", m->na, 1, d->act, fp, float_format);
  printArray("QACC_WARMSTART", m->nv, 1, d->qacc_warmstart, fp, float_format);
  printArray("CTRL", m->nu, 1, d->ctrl, fp, float_format);
  printArray("QFRC_APPLIED", m->nv, 1, d->qfrc_applied, fp, float_format);
  printArray("XFRC_APPLIED", m->nbody, 6, d->xfrc_applied, fp, float_format);
  printArray("MOCAP_POS", m->nmocap, 3, d->mocap_pos, fp, float_format);
  printArray("MOCAP_QUAT", m->nmocap, 4, d->mocap_quat, fp, float_format);
  printArray("QACC", m->nv, 1, d->qacc, fp, float_format);
  printArray("ACT_DOT", m->na, 1, d->act_dot, fp, float_format);
  printArray("USERDATA", m->nuserdata, 1, d->userdata, fp, float_format);
  printArray("SENSOR", m->nsensordata, 1, d->sensordata, fp, float_format);

  printArray("XPOS", m->nbody, 3, d->xpos, fp, float_format);
  printArray("XQUAT", m->nbody, 4, d->xquat, fp, float_format);
  printArray("XMAT", m->nbody, 9, d->xmat, fp, float_format);
  printArray("XIPOS", m->nbody, 3, d->xipos, fp, float_format);
  printArray("XIMAT", m->nbody, 9, d->ximat, fp, float_format);
  printArray("XANCHOR", m->njnt, 3, d->xanchor, fp, float_format);
  printArray("XAXIS", m->njnt, 3, d->xaxis, fp, float_format);
  printArray("GEOM_XPOS", m->ngeom, 3, d->geom_xpos, fp, float_format);
  printArray("GEOM_XMAT", m->ngeom, 9, d->geom_xmat, fp, float_format);
  printArray("SITE_XPOS", m->nsite, 3, d->site_xpos, fp, float_format);
  printArray("SITE_XMAT", m->nsite, 9, d->site_xmat, fp, float_format);
  printArray("CAM_XPOS", m->ncam, 3, d->cam_xpos, fp, float_format);
  printArray("CAM_XMAT", m->ncam, 9, d->cam_xmat, fp, float_format);
  printArray("LIGHT_XPOS", m->nlight, 3, d->light_xpos, fp, float_format);
  printArray("LIGHT_XDIR", m->nlight, 3, d->light_xdir, fp, float_format);

  printArray("SUBTREE_COM", m->nbody, 3, d->subtree_com, fp, float_format);
  printArray("CDOF", m->nv, 6, d->cdof, fp, float_format);
  printArray("CINERT", m->nbody, 10, d->cinert, fp, float_format);

  printArray("TEN_LENGTH", m->ntendon, 1, d->ten_length, fp, float_format);
  if (!mj_isSparse(m)) {
    printArray("TEN_MOMENT", m->ntendon, m->nv, d->ten_J, fp, float_format);
  } else {
    printArrayInt("TEN_J_ROWNNZ", m->ntendon, 1, d->ten_J_rownnz, fp);
    printArrayInt("TEN_J_ROWADR", m->ntendon, 1, d->ten_J_rowadr, fp);
    printSparse("TEN_J", d->ten_J, m->ntendon, d->ten_J_rownnz,
                d->ten_J_rowadr, d->ten_J_colind, fp, float_format);
  }
  for (int i=0; i < m->ntendon; i++) {
    fprintf(fp, "TENDON %d: %d wrap points\n", i, d->ten_wrapnum[i]);
    for (int j=0; j < d->ten_wrapnum[i]; j++) {
      fprintf(fp, "    %d:  ", d->wrap_obj[d->ten_wrapadr[i]+j]);
      printVector("", d->wrap_xpos+3*(d->ten_wrapadr[i]+j), 3, fp, float_format);
    }
    fprintf(fp, "\n");
  }

  printArray("ACTUATOR_LENGTH", m->nu, 1, d->actuator_length, fp, float_format);
  printArray("ACTUATOR_MOMENT", m->nu, m->nv, d->actuator_moment, fp, float_format);
  printArray("CRB", m->nbody, 10, d->crb, fp, float_format);

  // construct and print full M matrix
  mj_fullM(m, M, d->qM);
  printArray("QM", m->nv, m->nv, M, fp, float_format);

  // construct and print full LD matrix
  mj_fullM(m, M, d->qLD);
  printArray("QLD", m->nv, m->nv, M, fp, float_format);

  printArray("QLDIAGINV", m->nv, 1, d->qLDiagInv, fp, float_format);
  printArray("QLDIAGSQRTINV", m->nv, 1, d->qLDiagSqrtInv, fp, float_format);

  // D_rownnz
  fprintf(fp, NAME_FORMAT, "D_rownnz");
  for (int i = 0; i < m->nv; i++) {
    fprintf(fp, " %d", d->D_rownnz[i]);
  }
  fprintf(fp, "\n\n");

  // D_rowadr
  fprintf(fp, NAME_FORMAT, "D_rowadr");
  for (int i = 0; i < m->nv; i++) {
    fprintf(fp, " %d", d->D_rowadr[i]);
  }
  fprintf(fp, "\n\n");

  // D_colind
  fprintf(fp, NAME_FORMAT, "D_colind");
  for (int i = 0; i < m->nD; i++) {
    fprintf(fp, " %d", d->D_colind[i]);
  }
  fprintf(fp, "\n\n");

  // B_rownnz
  fprintf(fp, NAME_FORMAT, "B_rownnz");
  for (int i = 0; i < m->nbody; i++) {
    fprintf(fp, " %d", d->B_rownnz[i]);
  }
  fprintf(fp, "\n\n");

  // B_rowadr
  fprintf(fp, NAME_FORMAT, "B_rowadr");
  for (int i = 0; i < m->nbody; i++) {
    fprintf(fp, " %d", d->B_rowadr[i]);
  }
  fprintf(fp, "\n\n");

  // B_colind
  fprintf(fp, NAME_FORMAT, "B_colind");
  for (int i = 0; i < m->nB; i++) {
    fprintf(fp, " %d", d->B_colind[i]);
  }
  fprintf(fp, "\n\n");

  // print qDeriv
  mju_sparse2dense(M, d->qDeriv, m->nv, m->nv, d->D_rownnz, d->D_rowadr, d->D_colind);
  printArray("QDERIV", m->nv, m->nv, M, fp, float_format);

  // print qLU
  mju_sparse2dense(M, d->qLU, m->nv, m->nv, d->D_rownnz, d->D_rowadr,
                   d->D_colind);
  printArray("QLU", m->nv, m->nv, M, fp, float_format);

  // contact
  fprintf(fp, "CONTACT\n");
  for (int i=0; i < d->ncon; i++) {
    fprintf(fp, "  %d:\n     dim           %d\n     geom          ", i, d->contact[i].dim);
    const char* geom1 = mj_id2name(m, mjOBJ_GEOM, d->contact[i].geom1);
    if (geom1) {
      fprintf(fp, "%s ", geom1);
    } else {
      fprintf(fp, "%d ", d->contact[i].geom1);
    }
    const char* geom2 = mj_id2name(m, mjOBJ_GEOM, d->contact[i].geom2);
    if (geom2) {
      if (geom1) fprintf(fp, " ");  // two spaces between two names
      fprintf(fp, "%s\n", geom2);
    } else {
      fprintf(fp, "%d\n", d->contact[i].geom2);
    }
    fprintf(fp, "     exclude       %d\n     efc_address   %d\n",
            d->contact[i].exclude, d->contact[i].efc_address);
    printVector("     solref       ", d->contact[i].solref, mjNREF, fp, float_format);
    printVector("     solimp       ", d->contact[i].solimp, mjNIMP, fp, float_format);
    printVector("     dist         ", &d->contact[i].dist, 1, fp, float_format);
    printVector("     includemargin", &d->contact[i].includemargin, 1, fp, float_format);
    printVector("     pos          ", d->contact[i].pos, 3, fp, float_format);
    printVector("     frame        ", d->contact[i].frame, 9, fp, float_format);
    printVector("     friction     ", d->contact[i].friction, 5, fp, float_format);
    printVector("     mu           ", &d->contact[i].mu, 1, fp, float_format);
  }
  if (d->ncon) fprintf(fp, "\n");

  printArrayInt("EFC_TYPE", d->nefc, 1, d->efc_type, fp);
  printArrayInt("EFC_ID", d->nefc, 1, d->efc_id, fp);

  if (!mj_isSparse(m)) {
    printArray("EFC_J", d->nefc, m->nv, d->efc_J, fp, float_format);
    printArray("EFC_AR", d->nefc, d->nefc, d->efc_AR, fp, float_format);
  } else {
    printArrayInt("EFC_J_ROWNNZ", d->nefc, 1, d->efc_J_rownnz, fp);
    printArrayInt("EFC_J_ROWADR", d->nefc, 1, d->efc_J_rowadr, fp);
    printSparse("EFC_J", d->efc_J, d->nefc, d->efc_J_rownnz,
                d->efc_J_rowadr, d->efc_J_colind, fp, float_format);

    printArrayInt("EFC_AR_ROWNNZ", d->nefc, 1, d->efc_AR_rownnz, fp);
    printArrayInt("EFC_AR_ROWADR", d->nefc, 1, d->efc_AR_rowadr, fp);
    printSparse("EFC_AR", d->efc_AR, d->nefc, d->efc_AR_rownnz,
                d->efc_AR_rowadr, d->efc_AR_colind, fp, float_format);
  }

  printArray("EFC_POS", d->nefc, 1, d->efc_pos, fp, float_format);
  printArray("EFC_MARGIN", d->nefc, 1, d->efc_margin, fp, float_format);
  printArray("EFC_FRICTIONLOSS", d->nefc, 1, d->efc_frictionloss, fp, float_format);
  printArray("EFC_DIAGAPPROX", d->nefc, 1, d->efc_diagApprox, fp, float_format);
  printArray("EFC_KBIP", d->nefc, 4, d->efc_KBIP, fp, float_format);
  printArray("EFC_D", d->nefc, 1, d->efc_D, fp, float_format);
  printArray("EFC_R", d->nefc, 1, d->efc_R, fp, float_format);

  printArray("TEN_VELOCITY", m->ntendon, 1, d->ten_velocity, fp, float_format);
  printArray("ACTUATOR_VELOCITY", m->nu, 1, d->actuator_velocity, fp, float_format);

  printArray("CVEL", m->nbody, 6, d->cvel, fp, float_format);
  printArray("CDOF_DOT", m->nv, 6, d->cdof_dot, fp, float_format);

  printArray("QFRC_BIAS", m->nv, 1, d->qfrc_bias, fp, float_format);

  printArray("QFRC_PASSIVE", m->nv, 1, d->qfrc_passive, fp, float_format);

  printArray("EFC_VEL", d->nefc, 1, d->efc_vel, fp, float_format);
  printArray("EFC_AREF", d->nefc, 1, d->efc_aref, fp, float_format);

  printArray("SUBTREE_LINVEL", m->nbody, 3, d->subtree_linvel, fp, float_format);
  printArray("SUBTREE_ANGMOM", m->nbody, 3, d->subtree_angmom, fp, float_format);

  printArray("ACTUATOR_FORCE", m->nu, 1, d->actuator_force, fp, float_format);
  printArray("QFRC_ACTUATOR", m->nv, 1, d->qfrc_actuator, fp, float_format);

  printArray("QFRC_SMOOTH", m->nv, 1, d->qfrc_smooth, fp, float_format);
  printArray("QACC_SMOOTH", m->nv, 1, d->qacc_smooth, fp, float_format);

  printArray("EFC_B", d->nefc, 1, d->efc_b, fp, float_format);
  printArray("EFC_FORCE", d->nefc, 1, d->efc_force, fp, float_format);
  printArrayInt("EFC_STATE", d->nefc, 1, d->efc_state, fp);
  printArray("QFRC_CONSTRAINT", m->nv, 1, d->qfrc_constraint, fp, float_format);

  printArray("QFRC_INVERSE", m->nv, 1, d->qfrc_inverse, fp, float_format);

  printArray("CACC", m->nbody, 6, d->cacc, fp, float_format);
  printArray("CFRC_INT", m->nbody, 6, d->cfrc_int, fp, float_format);
  printArray("CFRC_EXT", m->nbody, 6, d->cfrc_ext, fp, float_format);

#ifdef MEMORY_SANITIZER
  // restore poisoned status
  __msan_copy_shadow(d->buffer, shadow, d->nbuffer);
  mju_free(shadow);
#endif

  if (filename) {
    fclose(fp);
  }

  mjFREESTACK;
}


 #ifdef __clang__
 #pragma clang diagnostic pop
 #endif


// print mjData to text file
void mj_printData(const mjModel* m, mjData* d, const char* filename) {
  mj_printFormattedData(m, d, filename, FLOAT_FORMAT);
}
