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

#include "engine/engine_io.h"

#include <inttypes.h>  // IWYU pragma: keep
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjxmacro.h>
#include "engine/engine_core_smooth.h"
#include "engine/engine_forward.h"
#include "engine/engine_init.h"
#include "engine/engine_macro.h"
#include "engine/engine_memory.h"
#include "engine/engine_plugin.h"
#include "engine/engine_sleep.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

#ifdef ADDRESS_SANITIZER
  #include <sanitizer/asan_interface.h>
  #include <sanitizer/common_interface_defs.h>
#endif

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

#ifdef _MSC_VER
  #pragma warning (disable: 4305)  // disable MSVC warning: truncation from 'double' to 'float'
#endif

static const int MAX_ARRAY_SIZE = INT_MAX / 4;


//----------------------------------- static utility functions -------------------------------------


//----------------------------------- static utility functions -------------------------------------

// id used to identify binary mjModel file/buffer
static const int ID = 54321;


// number of ints in the mjb header
#define NHEADER 5


// macro for referring to a mjModel member in generic expressions
#define MJMODEL_MEMBER(name) (((mjModel*) NULL)->name)


// count ints in mjModel
static int getnint(void) {
  int cnt = 0;

#define X(name) cnt += _Generic(MJMODEL_MEMBER(name), int: 1, default: 0);
  MJMODEL_INTS
#undef X

  return cnt;
}


// count buffer members in mjModel (mjtSize)
static int getnbuffer(void) {
  int cnt = 0;

#define X(name) cnt += _Generic(MJMODEL_MEMBER(name), mjtSize: 1, default: 0);
  MJMODEL_INTS
#undef X

  return cnt;
}


// count pointers in mjModel
static int getnptr(void) {
  int cnt = 0;

#define X(type, name, nr, nc) cnt++;
  MJMODEL_POINTERS
#undef X

  return cnt;
}


// write to memory buffer
static void bufwrite(const void* src, int num, int szbuf, void* buf, int* ptrbuf) {
  // check pointers
  if (!src || !buf || !ptrbuf) {
    mjERROR("NULL pointer passed to bufwrite");
  }

  // check size
  if (*ptrbuf+num > szbuf) {
    mjERROR("attempting to write outside model buffer");
  }

  // write, advance pointer
  memcpy((char*)buf + *ptrbuf, src, num);
  *ptrbuf += num;
}


// read from memory buffer
static void bufread(void* dest, int num, int szbuf, const void* buf, int* ptrbuf) {
  // check pointers
  if (!dest || !buf || !ptrbuf) {
    mjERROR("NULL pointer passed to bufread");
  }

  // check size
  if (*ptrbuf+num > szbuf) {
    mjERROR("attempting to read outside model buffer");
  }

  // read, advance pointer
  memcpy(dest, (char*)buf + *ptrbuf, num);
  *ptrbuf += num;
}


// number of bytes to be skipped to achieve 64-byte alignment
static inline unsigned int SKIP(intptr_t offset) {
  const unsigned int align = 64;
  // compute skipped bytes
  return (align - (offset % align)) % align;
}


//----------------------------------- mjModel construction -----------------------------------------

// set pointers in mjModel buffer
static void mj_setPtrModel(mjModel* m) {
  char* ptr = (char*)m->buffer;

  // prepare symbols needed by xmacro
  MJMODEL_POINTERS_PREAMBLE(m);

  // assign pointers with padding
#define X(type, name, nr, nc)                             \
  m->name = (type*)(ptr + SKIP((intptr_t)ptr));           \
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF(m->name, ptr));  \
  ptr += SKIP((intptr_t)ptr) + sizeof(type)*(m->nr)*(nc);

  MJMODEL_POINTERS
#undef X

  // check size
  ptrdiff_t sz = ptr - (char*)m->buffer;
  if (m->nbuffer != sz) {
    mjERROR(
        "mjModel buffer size mismatch, "
        "expected size: %" PRIu64 ",  actual size: %td",
        m->nbuffer, sz);
  }
}


// increases buffer size without causing integer overflow, returns 0 if
// operation would cause overflow
// performs the following operations:
// *nbuffer += SKIP(*offset) + type_size*nr*nc;
// *offset += SKIP(*offset) + type_size*nr*nc;
static int safeAddToBufferSize(intptr_t* offset, mjtSize* nbuffer,
                               size_t type_size, int nr, int nc) {
  if (type_size < 0 || nr < 0 || nc < 0) {
    return 0;
  }
#if (__has_builtin(__builtin_add_overflow) && __has_builtin(__builtin_mul_overflow)) \
  || (defined(__GNUC__) && __GNUC__ >= 5)
  // supported by GCC and Clang
  size_t to_add = 0;
  if (__builtin_mul_overflow(nc, nr, &to_add)) return 0;
  if (__builtin_mul_overflow(to_add, type_size, &to_add)) return 0;
  if (__builtin_add_overflow(to_add, SKIP(*offset), &to_add)) return 0;
  if (__builtin_add_overflow(*nbuffer, to_add, nbuffer)) return 0;
  if (__builtin_add_overflow(*offset, to_add, offset)) return 0;
#else
  // TODO: offer a safe implementation for MSVC or other compilers that don't have the builtins
  *nbuffer += SKIP(*offset) + type_size*nr*nc;
  *offset += SKIP(*offset) + type_size*nr*nc;
#endif

  return 1;
}


// free model memory without destroying the struct
static void freeModelBuffers(mjModel* m) {
  mju_free(m->buffer);
}


// allocate and initialize mjModel structure
void mj_makeModel(mjModel** dest,
    int nq, int nv, int nu, int na, int nbody, int nbvh,
    int nbvhstatic, int nbvhdynamic, int noct, int njnt, int ntree,
    int nM, int nB, int nC, int nD, int ngeom, int nsite, int ncam,
    int nlight, int nflex, int nflexnode, int nflexvert, int nflexedge, int nflexelem,
    int nflexelemdata, int nflexelemedge, int nflexshelldata, int nflexevpair, int nflextexcoord,
    int nmesh, int nmeshvert, int nmeshnormal, int nmeshtexcoord, int nmeshface,
    int nmeshgraph, int nmeshpoly, int nmeshpolyvert, int nmeshpolymap, int nskin, int nskinvert,
    int nskintexvert, int nskinface,
    int nskinbone, int nskinbonevert, int nhfield, int nhfielddata, int ntex,
    int ntexdata, int nmat, int npair, int nexclude, int neq, int ntendon,
    int nwrap, int nsensor, int nnumeric, int nnumericdata, int ntext,
    int ntextdata, int ntuple, int ntupledata, int nkey, int nmocap,
    int nplugin, int npluginattr, int nuser_body, int nuser_jnt, int nuser_geom,
    int nuser_site, int nuser_cam, int nuser_tendon, int nuser_actuator,
    int nuser_sensor, int nnames, int npaths) {
  intptr_t offset = 0;
  int allocate = *dest ? 0 : 1;
  mjModel* m = NULL;

  // allocate mjModel
  if (!allocate) {
    m = *dest;
    freeModelBuffers(m);
  } else {
    m = (mjModel*)mju_malloc(sizeof(mjModel));
  }

  if (!m) {
    mjERROR("could not allocate mjModel");
  }
  memset(m, 0, sizeof(mjModel));

  // set size parameters
  m->nq = nq;
  m->nv = nv;
  m->nu = nu;
  m->na = na;
  m->nbody = nbody;
  m->nbvh = nbvh;
  m->nbvhstatic = nbvhstatic;
  m->nbvhdynamic = nbvhdynamic;
  m->noct = noct;
  m->njnt = njnt;
  m->ntree = ntree;
  m->nM = nM;
  m->nB = nB;
  m->nC = nC;
  m->nD = nD;
  m->ngeom = ngeom;
  m->nsite = nsite;
  m->ncam = ncam;
  m->nlight = nlight;
  m->nflex = nflex;
  m->nflexnode = nflexnode;
  m->nflexvert = nflexvert;
  m->nflexedge = nflexedge;
  m->nflexelem = nflexelem;
  m->nflexelemdata = nflexelemdata;
  m->nflexelemedge = nflexelemedge;
  m->nflexshelldata = nflexshelldata;
  m->nflexevpair = nflexevpair;
  m->nflextexcoord = nflextexcoord;
  m->nmesh = nmesh;
  m->nmeshvert = nmeshvert;
  m->nmeshnormal = nmeshnormal;
  m->nmeshtexcoord = nmeshtexcoord;
  m->nmeshface = nmeshface;
  m->nmeshgraph = nmeshgraph;
  m->nmeshpoly = nmeshpoly;
  m->nmeshpolyvert = nmeshpolyvert;
  m->nmeshpolymap = nmeshpolymap;
  m->nskin = nskin;
  m->nskinvert = nskinvert;
  m->nskintexvert = nskintexvert;
  m->nskinface = nskinface;
  m->nskinbone = nskinbone;
  m->nskinbonevert = nskinbonevert;
  m->nhfield = nhfield;
  m->nhfielddata = nhfielddata;
  m->ntex = ntex;
  m->ntexdata = ntexdata;
  m->nmat = nmat;
  m->npair = npair;
  m->nexclude = nexclude;
  m->neq = neq;
  m->ntendon = ntendon;
  m->nwrap = nwrap;
  m->nsensor = nsensor;
  m->nnumeric = nnumeric;
  m->nnumericdata = nnumericdata;
  m->ntext = ntext;
  m->ntextdata = ntextdata;
  m->ntuple = ntuple;
  m->ntupledata = ntupledata;
  m->nkey = nkey;
  m->nmocap = nmocap;
  m->nplugin = nplugin;
  m->npluginattr = npluginattr;
  m->nuser_body = nuser_body;
  m->nuser_jnt = nuser_jnt;
  m->nuser_geom = nuser_geom;
  m->nuser_site = nuser_site;
  m->nuser_cam = nuser_cam;
  m->nuser_tendon = nuser_tendon;
  m->nuser_actuator = nuser_actuator;
  m->nuser_sensor = nuser_sensor;
  m->nnames = nnames;
  long nnames_map = (long)nbody + njnt + ngeom + nsite + ncam + nlight + nflex + nmesh + nskin +
                    nhfield + ntex + nmat + npair + nexclude + neq + ntendon + nu + nsensor +
                    nnumeric + ntext + ntuple + nkey + nplugin;
  if (nnames_map >= INT_MAX / mjLOAD_MULTIPLE) {
    if (allocate) mju_free(m);
    mju_warning("Invalid model: size of nnames_map is larger than INT_MAX");
    return;
  }
  m->nnames_map = mjLOAD_MULTIPLE * nnames_map;
  m->npaths = npaths;

#define X(name)                                    \
  if ((m->name) < 0) {                             \
    if (allocate) mju_free(m);                     \
    mju_warning("Invalid model: negative " #name); \
    return;                                        \
  }
  MJMODEL_INTS;
#undef X

  // nbody should always be positive
  if (m->nbody == 0) {
    if (allocate) mju_free(m);
    mju_warning("Invalid model: nbody == 0");
    return;
  }

  // nmocap is going to get multiplied by 4, and shouldn't overflow
  if (m->nmocap >= MAX_ARRAY_SIZE) {
    if (allocate) mju_free(m);
    mju_warning("Invalid model: nmocap too large");
    return;
  }

  // compute buffer size
  m->nbuffer = 0;
#define X(type, name, nr, nc)                                                \
  if (!safeAddToBufferSize(&offset, &m->nbuffer, sizeof(type), m->nr, nc)) { \
    if (allocate) mju_free(m);                                               \
    mju_warning("Invalid model: " #name " too large.");                      \
    return;                                                                  \
  }

  MJMODEL_POINTERS
#undef X

  // allocate buffer
  m->buffer = mju_malloc(m->nbuffer);
  if (!m->buffer) {
    if (allocate) mju_free(m);
    mjERROR("could not allocate mjModel buffer");
  }

  // clear, set pointers in buffer
  memset(m->buffer, 0, m->nbuffer);
#ifdef MEMORY_SANITIZER
  // tell msan to treat the entire buffer as uninitialized
  __msan_allocated_memory(m->buffer, m->nbuffer);
#endif
  mj_setPtrModel(m);

  // set default options
  mj_defaultOption(&m->opt);
  mj_defaultVisual(&m->vis);
  mj_defaultStatistic(&m->stat);

  // copy pointer if allocated here
  if (allocate) {
    *dest = m;
  }
}


// copy mjModel, if dest==NULL create new model
mjModel* mj_copyModel(mjModel* dest, const mjModel* src) {
  // allocate new model if needed
  if (!dest) {
    mj_makeModel(
        &dest, src->nq, src->nv, src->nu, src->na, src->nbody, src->nbvh,
        src->nbvhstatic, src->nbvhdynamic, src->noct, src->njnt, src->ntree,
        src->nM, src->nB, src->nC, src->nD, src->ngeom, src->nsite, src->ncam,
        src->nlight, src->nflex, src->nflexnode, src->nflexvert, src->nflexedge,
        src->nflexelem, src->nflexelemdata, src->nflexelemedge,
        src->nflexshelldata, src->nflexevpair, src->nflextexcoord, src->nmesh,
        src->nmeshvert, src->nmeshnormal, src->nmeshtexcoord, src->nmeshface,
        src->nmeshgraph, src->nmeshpoly, src->nmeshpolyvert, src->nmeshpolymap,
        src->nskin, src->nskinvert, src->nskintexvert, src->nskinface,
        src->nskinbone, src->nskinbonevert, src->nhfield, src->nhfielddata,
        src->ntex, src->ntexdata, src->nmat, src->npair, src->nexclude,
        src->neq, src->ntendon, src->nwrap, src->nsensor, src->nnumeric,
        src->nnumericdata, src->ntext, src->ntextdata, src->ntuple,
        src->ntupledata, src->nkey, src->nmocap, src->nplugin, src->npluginattr,
        src->nuser_body, src->nuser_jnt, src->nuser_geom, src->nuser_site,
        src->nuser_cam, src->nuser_tendon, src->nuser_actuator,
        src->nuser_sensor, src->nnames, src->npaths);
  }
  if (!dest) {
    mjERROR("failed to make mjModel. Invalid sizes.");
  }

  // check sizes
  if (dest->nbuffer != src->nbuffer) {
    mj_deleteModel(dest);
    mjERROR("dest and src models have different buffer size");
  }

  // save buffer ptr, copy struct, restore buffer and other pointers
  void* save_bufptr = dest->buffer;
  *dest = *src;
  dest->buffer = save_bufptr;
  mj_setPtrModel(dest);

  // copy buffer contents
  {
    MJMODEL_POINTERS_PREAMBLE(src)
    #define X(type, name, nr, nc)  \
      memcpy((char*)dest->name, (const char*)src->name, sizeof(type)*(src->nr)*nc);
    MJMODEL_POINTERS
    #undef X
  }

  return dest;
}


// copy mjModel, skip large arrays not required for abstract visualization
void mjv_copyModel(mjModel* dest, const mjModel* src) {
  // check sizes
  if (dest->nbuffer != src->nbuffer) {
    mjERROR("dest and src models have different buffer size");
  }

  // save buffer ptr, copy struct, restore buffer and other pointers
  void* save_bufptr = dest->buffer;
  *dest = *src;
  dest->buffer = save_bufptr;
  mj_setPtrModel(dest);

  // redefine XNV to do nothing
  #undef XNV
  #define XNV(type, name, nr, nc)

  // copy buffer contents, skipping arrays marked XNV
  {
    MJMODEL_POINTERS_PREAMBLE(src)
    #define X(type, name, nr, nc)    \
      memcpy((char*)dest->name, (const char*)src->name, sizeof(type)*(src->nr)*nc);
    MJMODEL_POINTERS
    #undef X
  }
  // redefine XNV to be the same as X
  #undef XNV
  #define XNV X
}


// save model to binary file, or memory buffer of szbuf>0
void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz) {
  FILE* fp = 0;
  int ptrbuf = 0;

  // standard header
  int header[NHEADER] = {ID, sizeof(mjtNum), getnint(), getnbuffer(), getnptr()};

  // open file for writing if no buffer
  if (!buffer) {
    fp = fopen(filename, "wb");
    if (!fp) {
      mju_warning("Could not open file '%s'", filename);
      return;
    }
  }

  // write standard header, info, options, buffer (omit pointers)
  if (fp) {
    fwrite(header, sizeof(int), NHEADER, fp);
    #define X(name) fwrite(&m->name, sizeof(m->name), 1, fp);
    MJMODEL_INTS
    #undef X
    fwrite((void*)&m->opt, sizeof(mjOption), 1, fp);
    fwrite((void*)&m->vis, sizeof(mjVisual), 1, fp);
    fwrite((void*)&m->stat, sizeof(mjStatistic), 1, fp);
    {
      MJMODEL_POINTERS_PREAMBLE(m)
      #define X(type, name, nr, nc)  \
        fwrite((void*)m->name, sizeof(type), (m->nr)*(nc), fp);
      MJMODEL_POINTERS
      #undef X
    }
  } else {
    bufwrite(header, sizeof(header), buffer_sz, buffer, &ptrbuf);
    #define X(name) bufwrite(&m->name, sizeof(m->name), buffer_sz, buffer, &ptrbuf);
    MJMODEL_INTS
    #undef X
    bufwrite((void*)&m->opt, sizeof(mjOption), buffer_sz, buffer, &ptrbuf);
    bufwrite((void*)&m->vis, sizeof(mjVisual), buffer_sz, buffer, &ptrbuf);
    bufwrite((void*)&m->stat, sizeof(mjStatistic), buffer_sz, buffer, &ptrbuf);
    {
      MJMODEL_POINTERS_PREAMBLE(m)
      #define X(type, name, nr, nc)  \
        bufwrite((void*)m->name, sizeof(type)*(m->nr)*(nc), buffer_sz, buffer, &ptrbuf);
      MJMODEL_POINTERS
      #undef X
    }
  }

  if (fp) {
    fclose(fp);
  }
}


// load binary MJB model
mjModel* mj_loadModelBuffer(const void* buffer, int buffer_sz) {
  int ptrbuf = 0;
  mjModel *m = 0;

  if (buffer_sz < NHEADER*sizeof(int)) {
    mju_warning("Model file has an incomplete header");
    return NULL;
  }

  int header[NHEADER] = {0};
  bufread(header, NHEADER*sizeof(int), buffer_sz, buffer, &ptrbuf);

  // check header
  int expected_header[NHEADER] = {ID, sizeof(mjtNum), getnint(), getnbuffer(), getnptr()};
  for (int i=0; i < NHEADER; i++) {
    if (header[i] != expected_header[i]) {
      switch (i) {
      case 0:
        mju_warning("Model missing header ID");
        return NULL;

      case 1:
        mju_warning("Model and executable have different floating point precision");
        return NULL;

      case 2:
        mju_warning("Model and executable have different number of ints in mjModel");
        return NULL;

      case 3:
        mju_warning("Model and executable have different number of size_t members in mjModel");
        return NULL;

      default:
        mju_warning("Model and executable have different number of pointers in mjModel");
        return NULL;
      }
    }
  }

  if (ptrbuf + sizeof(int)*getnint() + sizeof(mjtSize)*getnbuffer() > buffer_sz) {
    mju_warning("Truncated model file - ran out of data while reading sizes");
    return NULL;
  }

  // read mjModel construction fields
  int ints[256];
  bufread(ints, sizeof(int)*getnint(), buffer_sz, buffer, &ptrbuf);

  // allocate new mjModel
  mj_makeModel(&m,
               ints[0],  ints[1],  ints[2],  ints[3],  ints[4],  ints[5],  ints[6],
               ints[7],  ints[8],  ints[9],  ints[10], ints[11], ints[12], ints[13],
               ints[14], ints[15], ints[16], ints[17], ints[18], ints[19], ints[20],
               ints[21], ints[22], ints[23], ints[24], ints[25], ints[26], ints[27],
               ints[28], ints[29], ints[30], ints[31], ints[32], ints[33], ints[34],
               ints[35], ints[36], ints[37], ints[38], ints[39], ints[40], ints[41],
               ints[42], ints[43], ints[44], ints[45], ints[46], ints[47], ints[48],
               ints[49], ints[50], ints[51], ints[52], ints[53], ints[54], ints[55],
               ints[56], ints[57], ints[58], ints[59], ints[60], ints[61], ints[62],
               ints[63], ints[64], ints[65], ints[66], ints[67], ints[68], ints[69],
               ints[70], ints[71], ints[72], ints[73], ints[74]);

  // read mjModel mjtSize fields
  mjtSize sizes[8];
  bufread(sizes, sizeof(mjtSize)*getnbuffer(), buffer_sz, buffer, &ptrbuf);

  // check mjtSize fields
  if (!m || m->nbuffer != sizes[getnbuffer()-1]) {
    mju_warning("Corrupted model, wrong nbuffer field");
    mj_deleteModel(m);
    return NULL;
  }

  // set integer fields
  {
    int int_idx = 0;
    int size_idx = 0;
    #define X(name) \
        m->name = _Generic(m->name, mjtSize: sizes[size_idx++], default: ints[int_idx++]);
    MJMODEL_INTS
    #undef X
  }

  // read options and buffer
  if (ptrbuf + sizeof(mjOption) + sizeof(mjVisual) + sizeof(mjStatistic) > buffer_sz) {
    mju_warning("Truncated model file - ran out of data while reading structs");
    return NULL;
  }
  bufread((void*)&m->opt, sizeof(mjOption), buffer_sz, buffer, &ptrbuf);
  bufread((void*)&m->vis, sizeof(mjVisual), buffer_sz, buffer, &ptrbuf);
  bufread((void*)&m->stat, sizeof(mjStatistic), buffer_sz, buffer, &ptrbuf);
  {
    MJMODEL_POINTERS_PREAMBLE(m)
    #define X(type, name, nr, nc)                                           \
      if (ptrbuf + sizeof(type) * (m->nr) * (nc) > buffer_sz) {             \
        mju_warning(                                                        \
            "Truncated model file - ran out of data while reading " #name); \
        mj_deleteModel(m);                                                  \
        return NULL;                                                        \
      }                                                                     \
      bufread(m->name, sizeof(type)*(m->nr)*(nc), buffer_sz, buffer, &ptrbuf);

    MJMODEL_POINTERS
    #undef X
  }

  // make sure buffer is the correct size
  if (ptrbuf != buffer_sz) {
    mju_warning("Model file is too large");
    mj_deleteModel(m);
    return NULL;
  }

  const char* validationError = mj_validateReferences(m);
  if (validationError) {
    mju_warning("%s", validationError);
    mj_deleteModel(m);
    return NULL;
  }

  return m;
}


// de-allocate mjModel
void mj_deleteModel(mjModel* m) {
  if (m) {
    freeModelBuffers(m);
    mju_free(m);
  }
}


// size of buffer needed to hold model
mjtSize mj_sizeModel(const mjModel* m) {
  mjtSize size = (
    sizeof(int)*(NHEADER+getnint())
    + sizeof(mjtSize)*getnbuffer()
    + sizeof(mjOption)
    + sizeof(mjVisual)
    + sizeof(mjStatistic));

  MJMODEL_POINTERS_PREAMBLE(m)
#define X(type, name, nr, nc)         \
  size += sizeof(type)*(m->nr)*(nc);
  MJMODEL_POINTERS
#undef X

  return size;
}


//-------------------------- sparse system matrix construction -------------------------------------

// construct sparse representation of dof-dof matrix
void mj_makeDofDofSparse(int nv, int nC, int nD, int nM,
                         const int* dof_parentid, const int* dof_simplenum,
                         int* rownnz, int* rowadr, int* diag, int* colind,
                         int reduced, int upper,
                         int* remaining) {
  // no dofs, nothing to do
  if (!nv) {
    return;
  }

  // compute rownnz
  mju_zeroInt(rownnz, nv);
  for (int i = nv - 1; i >= 0; i--) {
    // init at diagonal
    int j = i;
    rownnz[i]++;

    // process below diagonal unless reduced and dof is simple
    if (!(reduced && dof_simplenum[i])) {
      while ((j = dof_parentid[j]) >= 0) {
        // both reduced and non-reduced have lower triangle
        rownnz[i]++;

        // add upper triangle if requested
        if (upper) rownnz[j]++;
      }
    }
  }

  // accumulate rowadr
  rowadr[0] = 0;
  for (int i = 1; i < nv; i++) {
    rowadr[i] = rowadr[i - 1] + rownnz[i - 1];
  }

  // populate colind
  mju_copyInt(remaining, rownnz, nv);
  for (int i = nv - 1; i >= 0; i--) {
    // init at diagonal
    remaining[i]--;
    colind[rowadr[i] + remaining[i]] = i;

    // process below diagonal unless reduced and dof is simple
    if (!(reduced && dof_simplenum[i])) {
      int j = i;
      while ((j = dof_parentid[j]) >= 0) {
        remaining[i]--;
        colind[rowadr[i] + remaining[i]] = j;

        // add upper triangle if requested
        if (upper) {
          remaining[j]--;
          colind[rowadr[j] + remaining[j]] = i;
        }
      }
    }
  }

  // check for remaining; SHOULD NOT OCCUR
  for (int i = 0; i < nv; i++) {
    if (remaining[i] != 0) {
      mjERROR("unexpected remaining");
    }
  }

  // check total nnz; SHOULD NOT OCCUR
  int expected_nnz = upper ? nD : (reduced ? nC : nM);
  if (rowadr[nv - 1] + rownnz[nv - 1] != expected_nnz) {
    mjERROR("sum of rownnz different from expected");
  }

  // find diagonal indices
  if (diag) {
    for (int i = 0; i < nv; i++) {
      int adr = rowadr[i];
      int j = 0;
      while (colind[adr + j] < i && j < rownnz[i]) {
        j++;
      }
      if (colind[adr + j] != i) {
        mjERROR("diagonal index not found");
      }
      diag[i] = j;
    }
  }
}

// construct sparse representation of body-dof matrix
void mj_makeBSparse(int nv, int nbody, int nB,
                    const int* body_dofnum, const int* body_parentid, const int* body_dofadr,
                    int* B_rownnz, int* B_rowadr, int* B_colind,
                    int* count) {
  // set rownnz to subtree dofs counts, including self
  mju_zeroInt(B_rownnz, nbody);
  for (int i = nbody - 1; i > 0; i--) {
    B_rownnz[i] += body_dofnum[i];
    B_rownnz[body_parentid[i]] += B_rownnz[i];
  }

  // check if rownnz[0] != nv; SHOULD NOT OCCUR
  if (B_rownnz[0] != nv) {
    mjERROR("rownnz[0] different from nv");
  }

  // add dofs in ancestors bodies
  for (int i = 0; i < nbody; i++) {
    int j = body_parentid[i];
    while (j > 0) {
      B_rownnz[i] += body_dofnum[j];
      j = body_parentid[j];
    }
  }

  // compute rowadr
  B_rowadr[0] = 0;
  for (int i = 1; i < nbody; i++) {
    B_rowadr[i] = B_rowadr[i - 1] + B_rownnz[i - 1];
  }

  // check if total nnz != nB; SHOULD NOT OCCUR
  if (nB != B_rowadr[nbody - 1] + B_rownnz[nbody - 1]) {
    mjERROR("sum of rownnz different from nB");
  }

  // clear incremental row counts
  mju_zeroInt(count, nbody);

  // add subtree dofs to colind
  for (int i = nbody - 1; i > 0; i--) {
    // add this body's dofs to subtree
    for (int n = 0; n < body_dofnum[i]; n++) {
      B_colind[B_rowadr[i] + count[i]] = body_dofadr[i] + n;
      count[i]++;
    }

    // add body subtree to parent
    int par = body_parentid[i];
    for (int n = 0; n < count[i]; n++) {
      B_colind[B_rowadr[par] + count[par]] = B_colind[B_rowadr[i] + n];
      count[par]++;
    }
  }

  // add all ancestor dofs
  for (int i = 0; i < nbody; i++) {
    int par = body_parentid[i];
    while (par > 0) {
      // add ancestor body dofs
      for (int n = 0; n < body_dofnum[par]; n++) {
        B_colind[B_rowadr[i] + count[i]] = body_dofadr[par] + n;
        count[i]++;
      }

      // advance to parent
      par = body_parentid[par];
    }
  }

  // process all bodies
  for (int i = 0; i < nbody; i++) {
    // make sure cnt = rownnz; SHOULD NOT OCCUR
    if (B_rownnz[i] != count[i]) {
      mjERROR("cnt different from rownnz");
    }

    // sort colind in each row
    if (count[i] > 1) {
      mju_insertionSortInt(B_colind + B_rowadr[i], count[i]);
    }
  }
}


// check D and B sparsity for consistency
static void checkDBSparse(const mjModel* m) {
  // process all dofs
  for (int j = 0; j < m->nv; j++) {
    // get body for this dof
    int i = m->dof_bodyid[j];

    // D[row j] and B[row i] should be identical
    if (m->D_rownnz[j] != m->B_rownnz[i]) {
      mjERROR("rows have different nnz");
    }
    for (int k = 0; k < m->D_rownnz[j]; k++) {
      if (m->D_colind[m->D_rowadr[j] + k] != m->B_colind[m->B_rowadr[i] + k]) {
        mjERROR("rows have different colind");
      }
    }
  }
}


// integer valued dst[D or C or M] = src[M (legacy)], handle different sparsity representations
static void copyM2Sparse(int nv,
                         const int* dof_Madr, const int* dof_simplenum, const int* dof_parentid,
                         const int* rownnz, const int* rowadr, const int* src,
                         int* dst,
                         int reduced, int upper, int* remaining) {
  // init remaining
  mju_copyInt(remaining, rownnz, nv);

  // copy data
  for (int i = nv - 1; i >= 0; i--) {
    // init at diagonal
    int adr = dof_Madr[i];
    remaining[i]--;
    dst[rowadr[i] + remaining[i]] = src[adr];
    adr++;

    // process below diagonal unless reduced and dof is simple
    if (!(reduced && dof_simplenum[i])) {
      int j = i;
      while ((j = dof_parentid[j]) >= 0) {
        remaining[i]--;
        dst[rowadr[i] + remaining[i]] = src[adr];

        // add upper triangle if requested
        if (upper) {
          remaining[j]--;
          dst[rowadr[j] + remaining[j]] = src[adr];
        }

        adr++;
      }
    }
  }

  // check that none remaining
  for (int i=0; i < nv; i++) {
    if (remaining[i]) {
      mjERROR("unassigned index");
    }
  }
}


// construct index mappings between M <-> D, M -> C, M (legacy) -> M (CSR)
void mj_makeDofDofMaps(int nv, int nM, int nC, int nD,
                       const int* dof_Madr, const int* dof_simplenum, const int* dof_parentid,
                       const int* D_rownnz, const int* D_rowadr, const int* D_colind,
                       const int* M_rownnz, const int* M_rowadr, const int* M_colind,
                       int* mapM2D, int* mapD2M, int* mapM2M,
                       int* M, int* scratch) {
  // make mapM2D: M -> D (lower to symmetric)
  mju_lower2SymMap(mapM2D, nv, D_rowadr, D_rownnz, D_colind, M_rowadr, M_rownnz, M_colind, scratch);

  // make mapD2M: D -> M (symmetric to lower)
  mju_sparseMap(mapD2M, nv, M_rowadr, M_rownnz, M_colind, D_rowadr, D_rownnz, D_colind);

  // make mapM2M
  for (int i=0; i < nM; i++) M[i] = i;
  mju_fillInt(mapM2M, -1, nC);
  copyM2Sparse(nv, dof_Madr, dof_simplenum, dof_parentid, M_rownnz,
               M_rowadr, M, mapM2M, /*reduced=*/1, /*upper=*/0, scratch);

  // check that all indices are filled in
  for (int i=0; i < nC; i++) {
    if (mapM2M[i] < 0) {
      mjERROR("unassigned index in mapM2M");
    }
  }
}


//----------------------------------- mjData construction ------------------------------------------

// set pointers into mjData buffer
static void mj_setPtrData(const mjModel* m, mjData* d) {
  char* ptr = (char*)d->buffer;

  // prepare symbols needed by xmacro
  MJDATA_POINTERS_PREAMBLE(m);

  // assign pointers with padding
#define X(type, name, nr, nc)                             \
  d->name = (type*)(ptr + SKIP((intptr_t)ptr));           \
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF(d->name, ptr));  \
  ptr += SKIP((intptr_t)ptr) + sizeof(type)*(m->nr)*(nc);

  MJDATA_POINTERS
#undef X

  // check size
  ptrdiff_t sz = ptr - (char*)d->buffer;
  if (d->nbuffer != sz) {
    mjERROR(
        "mjData buffer size mismatch, "
        "expected size: %" PRIu64 ",  actual size: %td",
        d->nbuffer, sz);
  }

  // zero-initialize arena pointers
#define X(type, name, nr, nc) d->name = NULL;
  MJDATA_ARENA_POINTERS
#undef X

  d->contact = d->arena;
}


// initialize plugins, copy into d (required for deletion)
void mj_initPlugin(const mjModel* m, mjData* d) {
  d->nplugin = m->nplugin;
  for (int i = 0; i < m->nplugin; ++i) {
    d->plugin[i] = m->plugin[i];
    const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[i]);
    if (plugin->init && plugin->init(m, d, i) < 0) {
      mju_free(d->buffer);
      mju_free(d->arena);
      mju_free(d);
      mjERROR("plugin->init failed for plugin id %d", i);
    }
  }
}


// free mjData memory without destroying the struct
static void freeDataBuffers(mjData* d) {
#ifdef ADDRESS_SANITIZER
    // raise an error if there's a dangling stack frame
    mj_freeStack(d);
#endif

    // destroy plugin instances
    for (int i = 0; i < d->nplugin; ++i) {
      const mjpPlugin* plugin = mjp_getPluginAtSlot(d->plugin[i]);
      if (plugin->destroy) {
        plugin->destroy(d, i);
      }
    }
    mju_free(d->buffer);
    mju_free(d->arena);
}


// allocate and initialize raw mjData structure
void mj_makeRawData(mjData** dest, const mjModel* m) {
  intptr_t offset = 0;
  int allocate = *dest ? 0 : 1;
  mjData* d = NULL;

  // allocate mjData
  if (!allocate) {
    d = *dest;
    freeDataBuffers(d);
  } else {
    d = (mjData*) mju_malloc(sizeof(mjData));
  }

  if (!d) {
    mjERROR("could not allocate mjData");
  }

  // prepare symbols needed by xmacro
  MJDATA_POINTERS_PREAMBLE(m);

  // compute buffer size
  d->nbuffer = 0;
  d->buffer = d->arena = NULL;
#define X(type, name, nr, nc)                                                \
  if (!safeAddToBufferSize(&offset, &d->nbuffer, sizeof(type), m->nr, nc)) { \
    if (allocate) mju_free(d);                                               \
    mju_warning("Invalid data: " #name " too large.");                       \
    return;                                                                  \
  }

  MJDATA_POINTERS
#undef X

  // copy stack size from model
  d->narena = m->narena;

  // allocate buffer
  d->buffer = mju_malloc(d->nbuffer);
  if (!d->buffer) {
    if (allocate) mju_free(d);
    mjERROR("could not allocate mjData buffer");
  }

  // allocate arena
  d->arena = mju_malloc(d->narena);
  if (!d->arena) {
    mju_free(d->buffer);
    if (allocate) mju_free(d);
    mjERROR("could not allocate mjData arena");
  }

  // set pointers into buffer
  mj_setPtrData(m, d);

  // clear threadpool
  d->threadpool = 0;

  // clear nplugin (overwritten by _initPlugin)
  d->nplugin = 0;

  // set awake array sizes to default (all awake)
  d->ntree_awake = m->ntree;
  d->nbody_awake = d->nparent_awake = m->nbody;
  d->nv_awake = m->nv;

  // copy pointer if allocated here
  if (allocate) {
    *dest = d;
  }
}


// allocate and initialize mjData structure
mjData* mj_makeData(const mjModel* m) {
  mjData* d = NULL;
  mj_makeRawData(&d, m);
  if (d) {
    mj_initPlugin(m, d);
    mj_resetData(m, d);
  }
  return d;
}


// copy mjData, if dest==NULL create new data;
// flg_all  1: copy all fields,  0: skip fields not required for visualization
mjData* mj_copyDataVisual(mjData* dest, const mjModel* m, const mjData* src, int flg_all) {
  void* save_buffer;
  void* save_arena;

  // allocate new data if needed
  if (!dest) {
    mj_makeRawData(&dest, m);
    mj_initPlugin(m, dest);
  }

  // check sizes
  if (dest->nbuffer != src->nbuffer) {
    mjERROR("dest and src data buffers have different size");
  }
  if (dest->narena != src->narena) {
    mjERROR("dest and src stacks have different size");
  }

  // stack is in use
  if (src->pstack) {
    mjERROR("attempting to copy mjData while stack is in use");
  }

  // save pointers, copy everything, restore pointers
  save_buffer = dest->buffer;
  save_arena = dest->arena;
  *dest = *src;
  dest->buffer = save_buffer;
  dest->arena = save_arena;
  mj_setPtrData(m, dest);

  // save plugin_data, since the X macro copying block below will override it
  const size_t plugin_data_size = sizeof(*dest->plugin_data) * dest->nplugin;
  uintptr_t* save_plugin_data = NULL;
  if (plugin_data_size) {
    save_plugin_data = (uintptr_t*)mju_malloc(plugin_data_size);
    if (!save_plugin_data) {
      mjERROR("failed to allocate temporary memory for plugin_data");
    }
    memcpy(save_plugin_data, dest->plugin_data, plugin_data_size);
  }

  // copy buffer
  {
    MJDATA_POINTERS_PREAMBLE(m)
    if (flg_all) {
      #define X(type, name, nr, nc)  \
        memcpy((char*)dest->name, (const char*)src->name, sizeof(type)*(m->nr)*nc);
      MJDATA_POINTERS
      #undef X
    } else {
      // redefine XNV to nothing
      #undef XNV
      #define XNV(type, name, nr, nc)

      #define X(type, name, nr, nc)  \
        memcpy((char*)dest->name, (const char*)src->name, sizeof(type)*(m->nr)*nc);
      MJDATA_POINTERS
      #undef X

      // redefine XNV to be the same as X
      #undef XNV
      #define XNV X
    }
  }


  // copy arena memory
  #undef MJ_D
  #define MJ_D(n) (src->n)
  #undef MJ_M
  #define MJ_M(n) (m->n)

  if (flg_all) {
    #define X(type, name, nr, nc)                                                \
    if (src->name) {                                                             \
      dest->name = (type*)((char*)dest->arena + PTRDIFF(src->name, src->arena)); \
      ASAN_UNPOISON_MEMORY_REGION(dest->name, sizeof(type) * nr * nc);           \
      memcpy((char*)dest->name, (const char*)src->name, sizeof(type) * nr * nc); \
    } else {                                                                     \
      dest->name = NULL;                                                         \
    }
    MJDATA_ARENA_POINTERS
    #undef X
  } else {
    // redefine XNV to nothing
    #undef XNV
    #define XNV(type, name, nr, nc)

    #define X(type, name, nr, nc)                                                \
    if (src->name) {                                                             \
      dest->name = (type*)((char*)dest->arena + PTRDIFF(src->name, src->arena)); \
      ASAN_UNPOISON_MEMORY_REGION(dest->name, sizeof(type) * nr * nc);           \
      memcpy((char*)dest->name, (const char*)src->name, sizeof(type) * nr * nc); \
    } else {                                                                     \
      dest->name = NULL;                                                         \
    }
    MJDATA_ARENA_POINTERS
    #undef X

    // redefine XNV to be the same as X
    #undef XNV
    #define XNV X
  }

  #undef MJ_M
  #define MJ_M(n) n
  #undef MJ_D
  #define MJ_D(n) n

  // restore contact pointer
  dest->contact = dest->arena;

  // restore plugin_data
  if (plugin_data_size) {
    memcpy(dest->plugin_data, save_plugin_data, plugin_data_size);
    mju_free(save_plugin_data);
    save_plugin_data = NULL;
  }

  // copy plugin instances
  dest->nplugin = m->nplugin;
  for (int i = 0; i < m->nplugin; ++i) {
    const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[i]);
    if (plugin->copy) {
      plugin->copy(dest, m, src, i);
    }
  }

  dest->threadpool = src->threadpool;

  return dest;
}


mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src) {
  return mj_copyDataVisual(dest, m, src, /*flg_all=*/1);
}


mjData* mjv_copyData(mjData* dest, const mjModel* m, const mjData* src) {
  return mj_copyDataVisual(dest, m, src, /*flg_all=*/0);
}


// clear data, set defaults
static void _resetData(const mjModel* m, mjData* d, unsigned char debug_value) {
  //------------------------------ save plugin state and data
  mjtNum* plugin_state;
  uintptr_t* plugindata;
  if (d->nplugin) {
    plugin_state = mju_malloc(sizeof(mjtNum) * m->npluginstate);
    memcpy(plugin_state, d->plugin_state, sizeof(mjtNum) * m->npluginstate);
    plugindata = mju_malloc(sizeof(uintptr_t) * m->nplugin);
    memcpy(plugindata, d->plugin_data, sizeof(uintptr_t) * m->nplugin);
  }

  //------------------------------ clear header

  // clear stack pointer
  if (!d->threadpool) {
    d->pstack = 0;
  }
  d->pbase = 0;

  // clear arena pointers
  d->parena = 0;

  // poison the entire arena+stack memory region when built with asan
#ifdef ADDRESS_SANITIZER
  ASAN_POISON_MEMORY_REGION(d->arena, d->narena);
#endif

#ifdef MEMORY_SANITIZER
  __msan_allocated_memory(d->arena, d->narena);
#endif

#define X(type, name, nr, nc) d->name = NULL;
  MJDATA_ARENA_POINTERS
#undef X
  d->contact = d->arena;

  // clear memory utilization stats
  d->maxuse_stack = 0;
  memset(d->maxuse_threadstack, 0, mjMAXTHREAD*sizeof(mjtSize));
  d->maxuse_arena = 0;
  d->maxuse_con = 0;
  d->maxuse_efc = 0;

  // clear solver diagnostics
  memset(d->warning, 0, mjNWARNING*sizeof(mjWarningStat));
  memset(d->timer, 0, mjNTIMER*sizeof(mjTimerStat));
  memset(d->solver, 0, mjNSOLVER*mjNISLAND*sizeof(mjSolverStat));
  mju_zeroInt(d->solver_niter, mjNISLAND);
  mju_zeroInt(d->solver_nnz, mjNISLAND);
  mju_zero(d->solver_fwdinv, 2);

  // clear variable sizes
  d->ncon = 0;
  d->ne = 0;
  d->nf = 0;
  d->nl = 0;
  d->nefc = 0;
  d->nJ = 0;
  d->nA = 0;
  d->nisland = 0;
  d->nidof = 0;

  // clear global properties
  d->time = 0;
  mju_zero(d->energy, 2);

  //------------------------------ clear buffer, set defaults

  // fill buffer with debug_value (normally 0)
#ifdef ADDRESS_SANITIZER
  {
    #define X(type, name, nr, nc) memset(d->name, (int)debug_value, sizeof(type)*(m->nr)*(nc));
    MJDATA_POINTERS_PREAMBLE(m)
    MJDATA_POINTERS
    #undef X
  }
#else
  memset(d->buffer, (int)debug_value, d->nbuffer);
#endif

#ifdef MEMORY_SANITIZER
  // under MSAN, mark the entire buffer as uninitialized
  __msan_allocated_memory(d->buffer, d->nbuffer);
#endif

  // zero out user-settable state and input arrays (MSAN: mark as initialized)
  mju_zero(d->qpos, m->nq);
  mju_zero(d->qvel, m->nv);
  mju_zero(d->act, m->na);
  mju_zero(d->ctrl, m->nu);
  for (int i=0; i < m->neq; i++) d->eq_active[i] = m->eq_active0[i];
  mju_zero(d->qfrc_applied, m->nv);
  mju_zero(d->xfrc_applied, 6*m->nbody);
  mju_zero(d->qacc, m->nv);  // input to inverse dynamics
  mju_zero(d->qacc_warmstart, m->nv);
  mju_zero(d->act_dot, m->na);
  mju_zero(d->userdata, m->nuserdata);
  mju_zero(d->mocap_pos, 3*m->nmocap);
  mju_zero(d->mocap_quat, 4*m->nmocap);

  // zero out qM, special case because scattering from M skips simple body off-diagonals
  mju_zero(d->qM, m->nM);

  // copy qpos0 from model
  if (m->qpos0) {
    mju_copy(d->qpos, m->qpos0, m->nq);
  }

  static int kAwake = -(1+mjMINAWAKE);  // tree_asleep value for fully awake tree

  // set all trees to awake
  mju_fillInt(d->tree_asleep, kAwake, m->ntree);

  // sleep enabled: handle static bodies and trees marked as mjSLEEP_INIT
  if (mjENABLED(mjENBL_SLEEP)) {
    // count trees initialized as asleep
    int num_asleep_init = 0;
    for (int i=0; i < m->ntree; i++) {
      num_asleep_init += (m->tree_sleep_policy[i] == mjSLEEP_INIT);
    }

    // update sleep arrays, treat static bodies as awake
    mj_updateSleepInit(m, d, /*flg_staticawake*/ 1);

    // partial mj_fwdPosition, functions that update STATIC values
    if (!num_asleep_init) {
      mj_kinematics(m, d);
      mj_comPos(m, d);
      mj_camlight(m, d);
      mj_tendon(m, d);
    }

    // if any trees initialized as sleeping call entire mj_forward, put them to sleep
    else {
      mj_forward(m, d);

      // mark asleep-init trees as ready to sleep
      for (int i=0; i < m->ntree; i++) {
        int init = m->tree_sleep_policy[i] == mjSLEEP_INIT;
        d->tree_asleep[i] = init ? -1 : kAwake;
      }

      int nslept = mj_sleep(m, d);

      // raise error if any failed to sleep
      if (nslept != num_asleep_init) {
        // find root body of the first tree that could not be slept
        int root = -1;
        for (int i=0; i < m->ntree; i++) {
          if (m->tree_sleep_policy[i] == mjSLEEP_INIT && d->tree_asleep[i] < 0) {
            root = m->tree_bodyadr[i];
            break;
          }
        }

        // free all memory held by d just before aborting
        mj_deleteData(d);

        // raise error and abort
        const char* hasname = mj_id2name(m, mjOBJ_BODY, root);
        const char* name = hasname ? hasname : "";
        mjERROR("%d trees were marked as sleep='init' but only %d could be slept.\n"
                "Body '%s' (id=%d) is the root of the first tree that could not be slept.",
                num_asleep_init, nslept, name, root);
      }

      // clear arrays to avoid MSAN errors upon mid-step wake
      mju_zero(d->qacc_smooth, m->nv);
      mju_zero(d->qfrc_smooth, m->nv);

      // clear arena
      mj_clearEfc(d);
    }
  }

  // update sleep arrays and counters
  mj_updateSleep(m, d);

  // set mocap_pos/quat = body_pos/quat for mocap bodies
  if (m->body_mocapid) {
    for (int i=0; i < m->nbody; i++) {
      int id = m->body_mocapid[i];
      if (id >= 0) {
        mju_copy3(d->mocap_pos+3*id, m->body_pos+3*i);
        mju_copy4(d->mocap_quat+4*id, m->body_quat+4*i);
      }
    }
  } else {
    // set the mocap_quats to {1, 0, 0, 0}
    for (int i=0; i < m->nmocap; i++) {
      d->mocap_quat[4*i] = 1.0;
    }
  }

  // check consistency of sparse matrix representations
  checkDBSparse(m);

  // restore pluginstate and plugindata
  if (d->nplugin) {
    memcpy(d->plugin_state, plugin_state, sizeof(mjtNum) * m->npluginstate);
    mju_free(plugin_state);
    memcpy(d->plugin_data, plugindata, sizeof(uintptr_t) * m->nplugin);
    mju_free(plugindata);

    // restore the plugin array back into d and reset the instances
    for (int i = 0; i < m->nplugin; ++i) {
      d->plugin[i] = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[i]);
      if (plugin->reset) {
        plugin->reset(m, &d->plugin_state[m->plugin_stateadr[i]],
                      (void*)(d->plugin_data[i]), i);
      }
    }
  }

  // copy signature from model
  d->signature = m->signature;
}


// clear data, set data->qpos = model->qpos0
void mj_resetData(const mjModel* m, mjData* d) {
  _resetData(m, d, 0);
}


// clear data, set data->qpos = model->qpos0, fill with debug_value
void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value) {
  _resetData(m, d, debug_value);
}


// Reset data. If 0 <= key < nkey, set fields from specified keyframe.
void mj_resetDataKeyframe(const mjModel* m, mjData* d, int key) {
  _resetData(m, d, 0);

  // copy keyframe data if key is valid
  if (key >= 0 && key < m->nkey) {
    d->time = m->key_time[key];
    mju_copy(d->qpos, m->key_qpos+key*m->nq, m->nq);
    mju_copy(d->qvel, m->key_qvel+key*m->nv, m->nv);
    mju_copy(d->act,  m->key_act+ key*m->na, m->na);
    mju_copy(d->mocap_pos,  m->key_mpos+key*3*m->nmocap, 3*m->nmocap);
    mju_copy(d->mocap_quat, m->key_mquat+key*4*m->nmocap, 4*m->nmocap);
    mju_copy(d->ctrl, m->key_ctrl+key*m->nu, m->nu);
  }
}


// de-allocate mjData
void mj_deleteData(mjData* d) {
  if (d) {
    freeDataBuffers(d);
    mju_free(d);
  }
}


// number of position and velocity coordinates for each joint type
const int nPOS[4] = {7, 4, 1, 1};
const int nVEL[4] = {6, 3, 1, 1};

static int sensorSize(mjtSensor sensor_type, int sensor_dim) {
  switch (sensor_type) {
  case mjSENS_TOUCH:
  case mjSENS_RANGEFINDER:
  case mjSENS_JOINTPOS:
  case mjSENS_JOINTVEL:
  case mjSENS_TENDONPOS:
  case mjSENS_TENDONVEL:
  case mjSENS_ACTUATORPOS:
  case mjSENS_ACTUATORVEL:
  case mjSENS_ACTUATORFRC:
  case mjSENS_JOINTACTFRC:
  case mjSENS_TENDONACTFRC:
  case mjSENS_JOINTLIMITPOS:
  case mjSENS_JOINTLIMITVEL:
  case mjSENS_JOINTLIMITFRC:
  case mjSENS_TENDONLIMITPOS:
  case mjSENS_TENDONLIMITVEL:
  case mjSENS_TENDONLIMITFRC:
  case mjSENS_GEOMDIST:
  case mjSENS_INSIDESITE:
  case mjSENS_E_POTENTIAL:
  case mjSENS_E_KINETIC:
  case mjSENS_CLOCK:
    return 1;

  case mjSENS_CAMPROJECTION:
    return 2;

  case mjSENS_ACCELEROMETER:
  case mjSENS_VELOCIMETER:
  case mjSENS_GYRO:
  case mjSENS_FORCE:
  case mjSENS_TORQUE:
  case mjSENS_MAGNETOMETER:
  case mjSENS_BALLANGVEL:
  case mjSENS_FRAMEPOS:
  case mjSENS_FRAMEXAXIS:
  case mjSENS_FRAMEYAXIS:
  case mjSENS_FRAMEZAXIS:
  case mjSENS_FRAMELINVEL:
  case mjSENS_FRAMEANGVEL:
  case mjSENS_FRAMELINACC:
  case mjSENS_FRAMEANGACC:
  case mjSENS_SUBTREECOM:
  case mjSENS_SUBTREELINVEL:
  case mjSENS_SUBTREEANGMOM:
  case mjSENS_GEOMNORMAL:
    return 3;

  case mjSENS_GEOMFROMTO:
    return 6;

  case mjSENS_BALLQUAT:
  case mjSENS_FRAMEQUAT:
    return 4;

  case mjSENS_CONTACT:
  case mjSENS_TACTILE:
  case mjSENS_USER:
    return sensor_dim;

  case mjSENS_PLUGIN:
    return -1;

    // don't use a 'default' case, so compiler warns about missing values
  }
  return -1;
}


// returns the number of objects of the given type
//   -1: mjOBJ_UNKNOWN
//   -2: invalid objtype
static int numObjects(const mjModel* m, mjtObj objtype) {
  switch (objtype) {
  case mjOBJ_DEFAULT:
  case mjOBJ_FRAME:
  case mjOBJ_UNKNOWN:
  case mjOBJ_MODEL:
    return -1;
  case mjOBJ_BODY:
  case mjOBJ_XBODY:
    return m->nbody;
  case mjOBJ_JOINT:
    return m->njnt;
  case mjOBJ_DOF:
    return m->nv;
  case mjOBJ_GEOM:
    return m->ngeom;
  case mjOBJ_SITE:
    return m->nsite;
  case mjOBJ_CAMERA:
    return m->ncam;
  case mjOBJ_LIGHT:
    return m->nlight;
  case mjOBJ_FLEX:
    return m->nflex;
  case mjOBJ_MESH:
    return m->nmesh;
  case mjOBJ_SKIN:
    return m->nskin;
  case mjOBJ_HFIELD:
    return m->nhfield;
  case mjOBJ_TEXTURE:
    return m->ntex;
  case mjOBJ_MATERIAL:
    return m->nmat;
  case mjOBJ_PAIR:
    return m->npair;
  case mjOBJ_EXCLUDE:
    return m->nexclude;
  case mjOBJ_EQUALITY:
    return m->neq;
  case mjOBJ_TENDON:
    return m->ntendon;
  case mjOBJ_ACTUATOR:
    return m->nu;
  case mjOBJ_SENSOR:
    return m->nsensor;
  case mjOBJ_NUMERIC:
    return m->nnumeric;
  case mjOBJ_TEXT:
    return m->ntext;
  case mjOBJ_TUPLE:
    return m->ntuple;
  case mjOBJ_KEY:
    return m->nkey;
  case mjOBJ_PLUGIN:
    return m->nplugin;
  case mjNOBJECT:
    return -2;
  }
  return -2;
}


// validate reference fields in a model; return null if valid, error message otherwise
const char* mj_validateReferences(const mjModel* m) {
  // for each field in mjModel that refers to another field, call X with:
  //   adrarray: array containing the references
  //   nadrs:    number of elements in refarray
  //   ntarget:  number of elements in array where references are pointing
  //   numarray: if refarray is an adr array, numarray is the corresponding num array, otherwise 0
#define MJMODEL_REFERENCES                                                       \
  X(body_parentid,      nbody,          nbody         , 0                      ) \
  X(body_rootid,        nbody,          nbody         , 0                      ) \
  X(body_weldid,        nbody,          nbody         , 0                      ) \
  X(body_mocapid,       nbody,          nmocap        , 0                      ) \
  X(body_jntadr,        nbody,          njnt          , m->body_jntnum         ) \
  X(body_dofadr,        nbody,          nv            , m->body_dofnum         ) \
  X(body_geomadr,       nbody,          ngeom         , m->body_geomnum        ) \
  X(body_bvhadr,        nbody,          nbvh          , m->body_bvhnum         ) \
  X(body_plugin,        nbody,          nplugin       , 0                      ) \
  X(jnt_qposadr,        njnt,           nq            , 0                      ) \
  X(jnt_dofadr,         njnt,           nv            , 0                      ) \
  X(jnt_bodyid,         njnt,           nbody         , 0                      ) \
  X(dof_bodyid,         nv,             nbody         , 0                      ) \
  X(dof_jntid,          nv,             njnt          , 0                      ) \
  X(dof_parentid,       nv,             nv            , 0                      ) \
  X(dof_Madr,           nv,             nM            , 0                      ) \
  X(tree_bodyadr,       ntree,          nbody         , m->tree_bodynum        ) \
  X(tree_dofadr,        ntree,          nv            , m->tree_dofnum         ) \
  X(geom_bodyid,        ngeom,          nbody         , 0                      ) \
  X(geom_matid,         ngeom,          nmat          , 0                      ) \
  X(site_bodyid,        nsite,          nbody         , 0                      ) \
  X(site_matid,         nsite,          nmat          , 0                      ) \
  X(cam_bodyid,         ncam,           nbody         , 0                      ) \
  X(cam_targetbodyid,   ncam,           nbody         , 0                      ) \
  X(light_bodyid,       nlight,         nbody         , 0                      ) \
  X(light_targetbodyid, nlight,         nbody         , 0                      ) \
  X(mesh_vertadr,       nmesh,          nmeshvert     , m->mesh_vertnum        ) \
  X(mesh_normaladr,     nmesh,          nmeshnormal   , m->mesh_normalnum      ) \
  X(mesh_texcoordadr,   nmesh,          nmeshtexcoord , m->mesh_texcoordnum    ) \
  X(mesh_faceadr,       nmesh,          nmeshface     , m->mesh_facenum        ) \
  X(mesh_bvhadr,        nmesh,          nbvh          , m->mesh_bvhnum         ) \
  X(mesh_graphadr,      nmesh,          nmeshgraph    , 0                      ) \
  X(mesh_polyadr,       nmesh,          nmeshpoly     , m->mesh_polynum        ) \
  X(mesh_polyvertadr,   nmeshpoly,      nmeshpolyvert , m->mesh_polyvertnum    ) \
  X(mesh_polymapadr,    nmeshvert,      nmeshpolymap  , m->mesh_polymapnum     ) \
  X(flex_vertadr,       nflex,          nflexvert     , m->flex_vertnum        ) \
  X(flex_edgeadr,       nflex,          nflexedge     , m->flex_edgenum        ) \
  X(flex_elemadr,       nflex,          nflexelem     , m->flex_elemnum        ) \
  X(flex_evpairadr,     nflex,          nflexevpair   , m->flex_evpairnum      ) \
  X(flex_texcoordadr,   nflex,          nflextexcoord , 0                      ) \
  X(flex_elemdataadr,   nflex,          nflexelemdata , 0                      ) \
  X(flex_elemedgeadr,   nflex,          nflexelemedge , 0                      ) \
  X(flex_shelldataadr,  nflex,          nflexshelldata, 0                      ) \
  X(flex_edge,          nflexedge*2,    nflexvert     , 0                      ) \
  X(flex_elem,          nflexelemdata,  nflexvert     , 0                      ) \
  X(flex_elemedge,      nflexelemedge,  nflexedge     , 0                      ) \
  X(flex_shell,         nflexshelldata, nflexvert     , 0                      ) \
  X(flex_bvhadr,        nflex,          nbvh          , m->flex_bvhnum         ) \
  X(skin_matid,         nskin,          nmat          , 0                      ) \
  X(skin_vertadr,       nskin,          nskinvert     , m->skin_vertnum        ) \
  X(skin_texcoordadr,   nskin,          nskintexvert  , 0                      ) \
  X(skin_faceadr,       nskin,          nskinface     , m->skin_facenum        ) \
  X(skin_boneadr,       nskin,          nskinbone     , m->skin_bonenum        ) \
  X(skin_bonevertadr,   nskinbone,      nskinbonevert , m->skin_bonevertnum    ) \
  X(skin_bonebodyid,    nskinbone,      nbody         , 0                      ) \
  X(skin_bonevertid,    nskinbonevert,  nskinvert     , 0                      ) \
  X(pair_geom1,         npair,          ngeom         , 0                      ) \
  X(pair_geom2,         npair,          ngeom         , 0                      ) \
  X(actuator_plugin,    nu,             nplugin       , 0                      ) \
  X(actuator_actadr,    nu,             na            , m->actuator_actnum     ) \
  X(sensor_plugin,      nsensor,        nplugin       , 0                      ) \
  X(plugin_stateadr,    nplugin,        npluginstate  , m->plugin_statenum     ) \
  X(plugin_attradr,     nplugin,        npluginattr   , 0                      ) \
  X(tendon_adr,         ntendon,        nwrap         , m->tendon_num          ) \
  X(tendon_matid,       ntendon,        nmat          , 0                      ) \
  X(tendon_treeid,      ntendon*2,      ntree         , 0                      ) \
  X(numeric_adr,        nnumeric,       nnumericdata  , m->numeric_size        ) \
  X(text_adr,           ntext,          ntextdata     , m->text_size           ) \
  X(tuple_adr,          ntuple,         ntupledata    , m->tuple_size          ) \
  X(name_bodyadr,       nbody,          nnames        , 0                      ) \
  X(name_jntadr,        njnt,           nnames        , 0                      ) \
  X(name_geomadr,       ngeom,          nnames        , 0                      ) \
  X(name_siteadr,       nsite,          nnames        , 0                      ) \
  X(name_camadr,        ncam,           nnames        , 0                      ) \
  X(name_lightadr,      nlight,         nnames        , 0                      ) \
  X(name_meshadr,       nmesh,          nnames        , 0                      ) \
  X(name_skinadr,       nskin,          nnames        , 0                      ) \
  X(name_hfieldadr,     nhfield,        nnames        , 0                      ) \
  X(name_texadr,        ntex,           nnames        , 0                      ) \
  X(name_matadr,        nmat,           nnames        , 0                      ) \
  X(name_pairadr,       npair,          nnames        , 0                      ) \
  X(name_excludeadr,    nexclude,       nnames        , 0                      ) \
  X(name_eqadr,         neq,            nnames        , 0                      ) \
  X(name_tendonadr,     ntendon,        nnames        , 0                      ) \
  X(name_actuatoradr,   nu,             nnames        , 0                      ) \
  X(name_sensoradr,     nsensor,        nnames        , 0                      ) \
  X(name_numericadr,    nnumeric,       nnames        , 0                      ) \
  X(name_textadr,       ntext,          nnames        , 0                      ) \
  X(name_tupleadr,      ntuple,         nnames        , 0                      ) \
  X(name_keyadr,        nkey,           nnames        , 0                      ) \
  X(hfield_pathadr,     nhfield,        npaths        , 0                      ) \
  X(mesh_pathadr,       nmesh,          npaths        , 0                      ) \
  X(skin_pathadr,       nskin,          npaths        , 0                      ) \
  X(tex_pathadr,        ntex,           npaths        , 0                      )

  #define X(adrarray, nadrs, ntarget, numarray) {             \
    int *nums = (numarray);                                   \
    for (int i=0; i<m->nadrs; i++) {                          \
      int adrsmin = m->adrarray[i];                           \
      int num = (nums ? nums[i] : 1);                         \
      if (num < 0) {                                          \
        return "Invalid model: " #numarray " is negative.";   \
      }                                                       \
      if (num > MAX_ARRAY_SIZE) {                             \
        return "Invalid model: " #numarray " is too large.";  \
      }                                                       \
      int adrsmax = m->adrarray[i] + num;                     \
      if (adrsmax > m->ntarget || adrsmin < -1) {             \
        return "Invalid model: " #adrarray " out of bounds."; \
      }                                                       \
    }                                                         \
  }

  MJMODEL_REFERENCES;
  #undef X
#undef MJMODEL_REFERENCES

  // special logic that doesn't fit in the macro:
  for (int i=0; i < m->nbody; i++) {
    if (i > 0 && m->body_parentid[i] >= i) {
      return "Invalid model: bad body_parentid.";
    }
    if (m->body_rootid[i] > i) {
      return "Invalid model: bad body_rootid.";
    }
    if (m->body_weldid[i] > i) {
      return "Invalid model: bad body_weldid.";
    }
  }
  for (int i=0; i < m->njnt; i++) {
    if (m->jnt_type[i] >= 4 || m->jnt_type[i] < 0) {
      return "Invalid model: jnt_type out of bounds.";
    }
    int jnt_qposadr = m->jnt_qposadr[i] + nPOS[m->jnt_type[i]];
    if (jnt_qposadr > m->nq || m->jnt_qposadr[i] < 0) {
      return "Invalid model: jnt_qposadr out of bounds.";
    }
    int jnt_dofadr = m->jnt_dofadr[i] + nVEL[m->jnt_type[i]];
    if (jnt_dofadr > m->nv || m->jnt_dofadr[i] < 0) {
      return "Invalid model: jnt_dofadr out of bounds.";
    }
  }
  for (int i=0; i < m->nv; i++) {
    if (m->dof_parentid[i] >= i) {
      return "Invalid model: bad dof_parentid.";
    }
  }
  for (int i=0; i < m->ngeom; i++) {
    if (m->geom_condim[i] > 6 || m->geom_condim[i] < 0) {
      return "Invalid model: geom_condim out of bounds.";
    }
    if (m->geom_type[i] == mjGEOM_HFIELD) {
      if (m->geom_dataid[i] >= m->nhfield || m->geom_dataid[i] < -1) {
        return "Invalid model: geom_dataid out of bounds.";
      }
    } else if ((m->geom_type[i] == mjGEOM_MESH) || (m->geom_type[i] == mjGEOM_SDF)) {
      if (m->geom_dataid[i] >= m->nmesh || m->geom_dataid[i] < -1) {
        return "Invalid model: geom_dataid out of bounds.";
      }
    }
  }
  for (int i=0; i < m->nhfield; i++) {
    int hfield_adr = m->hfield_adr[i] + m->hfield_nrow[i]*m->hfield_ncol[i];
    if (hfield_adr > m->nhfielddata || m->hfield_adr[i] < 0) {
      return "Invalid model: hfield_adr out of bounds.";
    }
  }
  for (int i=0; i < m->ntex; i++) {
    int tex_adr = m->tex_adr[i] + m->tex_nchannel[i]*m->tex_height[i]*m->tex_width[i];
    if (tex_adr > m->ntexdata || m->tex_adr[i] < 0) {
      return "Invalid model: tex_adr out of bounds.";
    }
  }
  for (int i=0; i < m->npair; i++) {
    int pair_body1 = (m->pair_signature[i] & 0xFFFF);
    if (pair_body1 >= m->nbody || pair_body1 < 0) {
      return "Invalid model: pair_body1 out of bounds.";
    }
    int pair_body2 = (m->pair_signature[i] >> 16);
    if (pair_body2 >= m->nbody || pair_body2 < 0) {
      return "Invalid model: pair_body2 out of bounds.";
    }
  }
  for (int i=0; i < m->neq; i++) {
    int obj1id = m->eq_obj1id[i];
    int obj2id = m->eq_obj2id[i];
    int objtype = m->eq_objtype[i];
    switch ((mjtEq) m->eq_type[i]) {
    case mjEQ_JOINT:
      if (obj1id >= m->njnt || obj1id < 0) {
        return "Invalid model: eq_obj1id out of bounds.";
      }
      // -1 is the value used if second object is omitted.
      if (obj2id >= m->njnt || obj2id < -1) {
        return "Invalid model: eq_obj2id out of bounds.";
      }
      break;

    case mjEQ_TENDON:
      if (obj1id >= m->ntendon || obj1id < 0) {
        return "Invalid model: eq_obj1id out of bounds.";
      }
      // -1 is the value used if second object is omitted.
      if (obj2id >= m->ntendon || obj2id < -1) {
        return "Invalid model: eq_obj2id out of bounds.";
      }
      break;

    case mjEQ_WELD:
    case mjEQ_CONNECT:
      if (objtype == mjOBJ_BODY) {
        if (obj1id >= m->nbody || obj1id < 0) {
          return "Invalid model: eq_obj1id out of bounds.";
        }
        if (obj2id >= m->nbody || obj2id < 0) {
          return "Invalid model: eq_obj2id out of bounds.";
        }
      } else if (objtype == mjOBJ_SITE) {
        if (obj1id >= m->nsite || obj1id < 0) {
          return "Invalid model: eq_obj1id out of bounds.";
        }
        if (obj2id >= m->nsite || obj2id < 0) {
          return "Invalid model: eq_obj2id out of bounds.";
        }
      } else {
        return "Invalid model: eq_objtype is not body or site.";
      }
      break;

    case mjEQ_FLEX:
      if (obj1id >= m->nflex || obj1id < 0) {
        return "Invalid model: eq_obj1id out of bounds.";
      }

      // -1 is the value used if second object is omitted
      if (obj2id != -1) {
        return "Invalid model: eq_obj2id must be -1.";
      }
      break;

    default:
      // might occur in case of the now-removed distance equality constraint
      mjERROR("unknown equality constraint type.");
    }
  }
  for (int i=0; i < m->nwrap; i++) {
    int wrap_objid = m->wrap_objid[i];
    switch ((mjtWrap) m->wrap_type[i]) {
    case mjWRAP_NONE:
    case mjWRAP_PULLEY:
      // wrap_objid not used.
      break;
    case mjWRAP_JOINT:
      if (wrap_objid >= m->njnt || wrap_objid < 0) {
        return "Invalid model: wrap_objid out of bounds.";
      }
      break;
    case mjWRAP_SITE:
      if (wrap_objid >= m->nsite || wrap_objid < 0) {
        return "Invalid model: wrap_objid out of bounds.";
      }
      break;
    case mjWRAP_SPHERE:
    case mjWRAP_CYLINDER:
      if (wrap_objid >= m->ngeom || wrap_objid < 0) {
        return "Invalid model: wrap_objid out of bounds.";
      }
      break;
    }
  }
  for (int i=0; i < m->nu; i++) {
    int actuator_trntype = m->actuator_trntype[i];
    int id = m->actuator_trnid[2*i];
    int idslider = m->actuator_trnid[2*i+1];
    switch ((mjtTrn) actuator_trntype) {
    case mjTRN_JOINT:
    case mjTRN_JOINTINPARENT:
      if (id < 0 || id >= m->njnt) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case mjTRN_TENDON:
      if (id < 0 || id >= m->ntendon) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case mjTRN_SITE:
      if (id < 0 || id >= m->nsite) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case mjTRN_SLIDERCRANK:
      if (id < 0 || id >= m->nsite) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      if (idslider < 0 || idslider >= m->nsite) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case mjTRN_BODY:
      if (id < 0 || id >= m->nbody) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case mjTRN_UNDEFINED:
      // actuator_trnid not used.
      break;
    }
  }
  for (int i=0; i < m->nsensor; i++) {
    mjtSensor sensor_type = m->sensor_type[i];
    int sensor_size;
    if (sensor_type == mjSENS_PLUGIN) {
      const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[m->sensor_plugin[i]]);
      if (!plugin->nsensordata) {
        mjERROR("`nsensordata` is a null function pointer for plugin at slot %d",
                m->plugin[m->sensor_plugin[i]]);
      }
      sensor_size = plugin->nsensordata(m, m->sensor_plugin[i], i);
    } else {
      sensor_size = sensorSize(sensor_type, m->sensor_dim[i]);
    }
    if (sensor_size < 0) {
      return "Invalid model: Bad sensor_type.";
    }
    int sensor_adr = m->sensor_adr[i];
    if (sensor_adr < 0 || sensor_adr + sensor_size > m->nsensordata) {
      return "Invalid model: sensor_adr out of bounds.";
    }
    int nobj = numObjects(m, m->sensor_objtype[i]);
    if (nobj == -2) {
      return "Invalid model: invalid sensor_objtype";
    }
    if (nobj != -1 && (m->sensor_objid[i] < 0 || m->sensor_objid[i] >= nobj)) {
      return "Invalid model: invalid sensor_objid";
    }
    nobj = numObjects(m, m->sensor_reftype[i]);
    if (nobj == -2) {
      return "Invalid model: invalid sensor_reftype";
    }
    if (nobj != -1 && (m->sensor_refid[i] < -1 || m->sensor_refid[i] >= nobj)) {
      return "Invalid model: invalid sensor_refid";
    }
    if (sensor_type == mjSENS_TACTILE) {
      int obj_id = m->sensor_objid[i];
      int parent_body = m->geom_bodyid[obj_id];
      int collision_geoms = 0;
      for (int b = 0; b < m->body_geomnum[parent_body]; ++b) {
        int geom_id = m->body_geomadr[parent_body]+b;
        if (m->geom_contype[geom_id] || m->geom_conaffinity[geom_id]) {
          collision_geoms++;
        }
      }
      if (collision_geoms == 0) {
        return "Touch sensor requires a body with at least one collision geom";
      }
    }
  }
  for (int i=0; i < m->nexclude; i++) {
    int exclude_body1 = (m->exclude_signature[i] & 0xFFFF);
    if (exclude_body1 >= m->nbody || exclude_body1 < 0) {
      return "Invalid model: exclude_body1 out of bounds.";
    }
    int exclude_body2 = (m->exclude_signature[i] >> 16);
    if (exclude_body2 >= m->nbody || exclude_body2 < 0) {
      return "Invalid model: exclude_body2 out of bounds.";
    }
  }
  for (int i=0; i < m->ntuple; i++) {
    for (int j=0; j < m->tuple_size[i]; j++) {
      int adr = m->tuple_adr[i] + j;
      int nobj = numObjects(m, m->tuple_objtype[adr]);
      if (nobj == -2) {
        return "Invalid model: invalid tuple_objtype";
      }
      if (nobj != -1 && (m->tuple_objid[adr] < 0 || m->tuple_objid[adr] >= nobj)) {
        return "Invalid model: invalid tuple_objid";
      }
    }
  }

  return NULL;
}
