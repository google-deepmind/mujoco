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
#include "engine/engine_core_util.h"
#include "engine/engine_forward.h"
#include "engine/engine_init.h"
#include "engine/engine_macro.h"
#include "engine/engine_memory.h"
#include "engine/engine_plugin.h"
#include "engine/engine_sleep.h"
#include "engine/engine_support.h"
#include "engine/engine_thread.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

#ifdef mjUSEASAN
  #include <sanitizer/asan_interface.h>
  #include <sanitizer/common_interface_defs.h>
#endif

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

#ifdef _MSC_VER
  #pragma warning (disable: 4305)  // disable MSVC warning: truncation from 'double' to 'float'
#endif

static const int MAX_ARRAY_SIZE = INT_MAX;

// tree_asleep value for fully awake tree
static const int kAwake = -(1+mjMINAWAKE);


//----------------------------------- static utility functions -------------------------------------

// id used to identify binary mjModel file/buffer
static const int ID = 54321;


// number of ints in the mjb header
#define NHEADER 5


// macro for referring to a mjModel member in generic expressions
#define MJMODEL_MEMBER(name) (((mjModel*) NULL)->name)


// count mjtSize members in mjModel
static int getnsize(void) {
  int cnt = 0;

#define X(name) cnt += _Generic(MJMODEL_MEMBER(name), mjtSize: 1, default: 0);
  MJMODEL_SIZES
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
static void bufwrite(const void* src, mjtSize num, mjtSize szbuf, void* buf, mjtSize* ptrbuf) {
  // check pointers
  if (!src || !buf || !ptrbuf) {
    mjERROR("NULL pointer passed to bufwrite");
  }

  // check size
  if (num < 0 || szbuf - *ptrbuf < num) {
    mjERROR("attempting to write outside model buffer");
  }

  // write, advance pointer
  memcpy((char*)buf + *ptrbuf, src, num);
  *ptrbuf += num;
}


// read from memory buffer
static void bufread(void* dest, mjtSize num, mjtSize szbuf, const void* buf, mjtSize* ptrbuf) {
  // check pointers
  if (!dest || !buf || !ptrbuf) {
    mjERROR("NULL pointer passed to bufread");
  }

  // check size
  if (num < 0 || szbuf - *ptrbuf < num) {
    mjERROR("attempting to read outside model buffer");
  }

  // read, advance pointer
  memcpy(dest, (char*)buf + *ptrbuf, num);
  *ptrbuf += num;
}


// alignment of the arrays in the mjModel and mjData buffers
#define BUFFER_ALIGN 64


// number of bytes to be skipped to achieve BUFFER_ALIGN-byte alignment
static inline unsigned int SKIP(intptr_t offset) {
  // compute skipped bytes
  return (BUFFER_ALIGN - (offset % BUFFER_ALIGN)) % BUFFER_ALIGN;
}


//----------------------------------- mjModel construction -----------------------------------------

// set pointers in mjModel buffer
void mj_setPtrModel(mjModel* m) {
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

  // check size, nbuffer includes the largest possible padding before the first array
  ptrdiff_t sz = ptr - (char*)m->buffer - SKIP((intptr_t)m->buffer) + BUFFER_ALIGN - 1;
  if (m->nbuffer != sz) {
    mjERROR(
        "mjModel buffer size mismatch, "
        "expected size: %" PRIu64 ",  actual size: %td",
        m->nbuffer, sz);
  }

  // poison unused bytes at the end of the buffer
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF((char*)m->buffer + m->nbuffer, ptr));
}


// increases buffer size without causing integer overflow, returns 0 if
// operation would cause overflow
// performs the following operations:
// *nbuffer += SKIP(*offset) + type_size*nr*nc;
// *offset += SKIP(*offset) + type_size*nr*nc;
static mjtSize safeAddToBufferSize(intptr_t* offset, mjtSize* nbuffer,
                                   size_t type_size, mjtSize nr, mjtSize nc) {
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
  // safe overflow checks for MSVC and other compilers without __builtin_*_overflow
  {
    size_t product;
    size_t to_add;
    size_t skip = SKIP(*offset);

    // nc * nr
    if (nr > 0 && (size_t)nc > SIZE_MAX / (size_t)nr) return 0;
    product = (size_t)nc * (size_t)nr;

    // product * type_size
    if (type_size > 0 && product > SIZE_MAX / type_size) return 0;
    product *= type_size;

    // product + SKIP(*offset)
    if (product > SIZE_MAX - skip) return 0;
    to_add = product + skip;

    // *nbuffer + to_add
    if ((size_t)*nbuffer > SIZE_MAX - to_add) return 0;
    *nbuffer += to_add;

    // *offset + to_add
    if (*offset > 0 && to_add > (size_t)(INTPTR_MAX - *offset)) return 0;
    *offset += to_add;
  }
#endif

  return 1;
}


// free model memory without destroying the struct
static void freeModelBuffers(mjModel* m) {
  mju_free(m->buffer);
}


// return 1 if the mjModel size at the given struct offset has a valid value, 0 otherwise
static int validSize(size_t offset, mjtSize size) {
  switch (offset) {
    // njmax and nconmax are -1 if they are not specified
    case offsetof(mjModel, njmax):
    case offsetof(mjModel, nconmax):
      return size >= -1 && size < MAX_ARRAY_SIZE;

    // numbers of bytes are not limited to the range of int
    case offsetof(mjModel, ntexdata):
    case offsetof(mjModel, ntextdata):
    case offsetof(mjModel, narena):
      return size >= 0;

    default:
      return size >= 0 && size < MAX_ARRAY_SIZE;
  }
}


// return 1 if the num elements at address adr are within an array of n elements, 0 otherwise
static int validRange(mjtSize adr, mjtSize num, mjtSize n) {
  return adr >= 0 && num >= 0 && num <= n && adr <= n - num;
}


// return a*b, -1 if an argument is negative or if the product is larger than maxval
static mjtSize boundedProduct(mjtSize a, mjtSize b, mjtSize maxval) {
  if (a < 0 || b < 0 || maxval < 0 || (a > 0 && b > maxval / a)) {
    return -1;
  }
  return a*b;
}


// allocate and initialize mjModel structure
void mj_makeModel(mjModel** dest, const mjModel* sizes) {
  // copy the sizes before *dest is cleared, in case they belong to the same model
#define X(name) mjtSize name = sizes->name;
  MJMODEL_SIZES_LAYOUT
#undef X

  intptr_t offset = 0;
  int allocate = *dest ? 0 : 1;
  mjModel* m = NULL;

  // CHECK SIZE PARAMETERS
  {
    #define X(name)                                                                       \
      if (!validSize(offsetof(mjModel, name), name)) {                                    \
        mju_warning("Invalid model: %s is out of range (%lld).", #name, (long long)name); \
        return;                                                                           \
      }
    MJMODEL_SIZES_LAYOUT
#undef X
  }

  // nbody should always be positive
  if (nbody == 0) {
    mju_warning("Invalid model: nbody == 0");
    return;
  }

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
#define X(name) m->name = name;
  MJMODEL_SIZES_LAYOUT
#undef X

  mjtSize nnames_map = nbody + njnt + ngeom + nsite + ncam + nlight + nflex + nmesh + nskin +
                       nhfield + ntex + nmat + npair + nexclude + neq + ntendon + nactuator +
                       nsensor + nnumeric + ntext + ntuple + nkey + nplugin;
  if (nnames_map >= INT_MAX / mjLOAD_MULTIPLE) {
    if (allocate) mju_free(m);
    mju_warning("Invalid model: size of nnames_map is larger than INT_MAX");
    return;
  }
  m->nnames_map = mjLOAD_MULTIPLE * nnames_map;

  // compute buffer size, including padding to align the first array at any buffer address
  m->nbuffer = BUFFER_ALIGN - 1;
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
    mj_makeModel(&dest, src);
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
void mj_saveModel(const mjModel* m, const char* filename, void* buffer, mjtSize buffer_sz) {
  mjtSize ptrbuf = 0;

  // standard header
  int header[NHEADER] = {ID, sizeof(mjtNum), getnsize(), mj_version(), getnptr()};

  // no buffer: serialize to temporary buffer, then write via resource provider
  if (!buffer) {
    mjtSize sz = mj_sizeModel(m);
    void* tmpbuf = mju_malloc(sz);
    if (!tmpbuf) {
      mju_warning("Could not allocate buffer for saving model");
      return;
    }
    mj_saveModel(m, NULL, tmpbuf, sz);

    mjtSize written = mju_writeResource(filename, tmpbuf, sz, NULL, NULL, 0);
    if (written != sz) {
      mju_warning("Could not save model to '%s'", filename);
    }
    mju_free(tmpbuf);
    return;
  }

  // write standard header, info, options, buffer (omit pointers)
  bufwrite(header, sizeof(header), buffer_sz, buffer, &ptrbuf);
  #define X(name) bufwrite(&m->name, sizeof(m->name), buffer_sz, buffer, &ptrbuf);
  MJMODEL_SIZES
  #undef X
  bufwrite((void*)&m->opt, sizeof(mjOption), buffer_sz, buffer, &ptrbuf);
  bufwrite((void*)&m->vis, sizeof(mjVisual), buffer_sz, buffer, &ptrbuf);
  bufwrite((void*)&m->stat, sizeof(mjStatistic), buffer_sz, buffer, &ptrbuf);
  bufwrite(&m->flg_gravcomp, sizeof(mjtBool), buffer_sz, buffer, &ptrbuf);
  bufwrite(&m->flg_surfacevel, sizeof(mjtBool), buffer_sz, buffer, &ptrbuf);
  bufwrite(&m->flg_adhesion, sizeof(mjtBool), buffer_sz, buffer, &ptrbuf);
  {
    MJMODEL_POINTERS_PREAMBLE(m)
    #define X(type, name, nr, nc)  \
      bufwrite((void*)m->name, sizeof(type)*(m->nr)*(nc), buffer_sz, buffer, &ptrbuf);
    MJMODEL_POINTERS
    #undef X
  }
}


// copy the sizes in MJMODEL_SIZES_RUNTIME read from a binary model to a model allocated with
// the other sizes, and check that all sizes match; return null if valid, error message otherwise
static const char* setRuntimeSizes(mjModel* m, const mjModel* sizes) {
  // the sizes which do not affect the model buffer are the only ones copied from the file
#define X(name)                                             \
  if (!validSize(offsetof(mjModel, name), sizes->name)) {   \
    return "Corrupted model, " #name " is out of range";    \
  }                                                         \
  m->name = sizes->name;
  MJMODEL_SIZES_RUNTIME
#undef X

  // the sizes set by mj_makeModel define the layout of the buffer, they are never overwritten:
  // every size in the model must now match the file, including the computed nnames_map and nbuffer
#define X(name)                                             \
  if (m->name != sizes->name) {                             \
    return "Corrupted model, wrong " #name " field";        \
  }
  MJMODEL_SIZES
#undef X

  return NULL;
}


// load binary MJB model
mjModel* mj_loadModelBuffer(const void* buffer, mjtSize buffer_sz) {
  mjtSize ptrbuf = 0;
  mjModel *m = 0;

  if (buffer_sz < NHEADER*sizeof(int)) {
    mju_warning("Model file has an incomplete header");
    return NULL;
  }

  int header[NHEADER] = {0};
  bufread(header, NHEADER*sizeof(int), buffer_sz, buffer, &ptrbuf);

  // check header
  int expected_header[NHEADER] = {ID, sizeof(mjtNum), getnsize(), mj_version(), getnptr()};
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
        mju_warning("Model and executable have different number of sizes in mjModel");
        return NULL;

      case 3:
        mju_warning("Model and executable use different MuJoCo version");
        return NULL;

      default:
        mju_warning("Model and executable have different number of pointers in mjModel");
        return NULL;
      }
    }
  }

  int nsize = getnsize();  // number of mjtSize fields in mjModel

  if (ptrbuf + sizeof(mjtSize)*nsize > buffer_sz) {
    mju_warning("Truncated model file - ran out of data while reading sizes");
    return NULL;
  }

  // read mjModel construction fields
  mjtSize raw_sizes[256];
  bufread(raw_sizes, sizeof(mjtSize)*nsize, buffer_sz, buffer, &ptrbuf);

  // copy the sizes to an mjModel struct, only its sizes are used
  mjModel sizes;
  memset(&sizes, 0, sizeof(mjModel));
  {
    int int_idx = 0;
#define X(name) sizes.name = raw_sizes[int_idx++];
    MJMODEL_SIZES
#undef X
  }

  // allocate new mjModel
  mj_makeModel(&m, &sizes);

  // mj_makeModel may fail if the input buffer has invalid sizes
  if (!m) {
    mju_warning("Invalid sizes, unable to load model");
    return NULL;
  }

  // set the remaining sizes, check all of them against the file
  const char* sizeError = setRuntimeSizes(m, &sizes);
  if (sizeError) {
    mju_warning("%s", sizeError);
    mj_deleteModel(m);
    return NULL;
  }

  // read options and buffer
  if (ptrbuf + sizeof(mjOption) + sizeof(mjVisual) + sizeof(mjStatistic) +
      sizeof(mjtBool) * 3 > buffer_sz) {
    mju_warning("Truncated model file - ran out of data while reading structs");
    mj_deleteModel(m);
    return NULL;
  }
  bufread((void*)&m->opt, sizeof(mjOption), buffer_sz, buffer, &ptrbuf);
  bufread((void*)&m->vis, sizeof(mjVisual), buffer_sz, buffer, &ptrbuf);
  bufread((void*)&m->stat, sizeof(mjStatistic), buffer_sz, buffer, &ptrbuf);
  bufread(&m->flg_gravcomp, sizeof(mjtBool), buffer_sz, buffer, &ptrbuf);
  bufread(&m->flg_surfacevel, sizeof(mjtBool), buffer_sz, buffer, &ptrbuf);
  bufread(&m->flg_adhesion, sizeof(mjtBool), buffer_sz, buffer, &ptrbuf);
  {
    MJMODEL_POINTERS_PREAMBLE(m)
    #define X(type, name, nr, nc)                                           \
      if (buffer_sz - ptrbuf < sizeof(type) * (m->nr) * (nc)) {             \
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
    sizeof(int)*NHEADER
    + sizeof(mjtSize)*getnsize()
    + sizeof(mjOption)
    + sizeof(mjVisual)
    + sizeof(mjStatistic)
    + sizeof(mjtBool)*3);

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

  // assign pointers with padding
#define X(type, name, nr, nc)                             \
  d->name = (type*)(ptr + SKIP((intptr_t)ptr));           \
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF(d->name, ptr));  \
  ptr += SKIP((intptr_t)ptr) + sizeof(type)*(m->nr)*(nc);

  MJDATA_POINTERS
#undef X

  // check size, nbuffer includes the largest possible padding before the first array
  ptrdiff_t sz = ptr - (char*)d->buffer - SKIP((intptr_t)d->buffer) + BUFFER_ALIGN - 1;
  if (d->nbuffer != sz) {
    mjERROR(
        "mjData buffer size mismatch, "
        "expected size: %" PRIu64 ",  actual size: %td",
        d->nbuffer, sz);
  }

  // poison unused bytes at the end of the buffer
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF((char*)d->buffer + d->nbuffer, ptr));

  // zero-initialize arena pointers
#define X(type, name, nr, nc) d->name = NULL;
  MJDATA_ARENA_POINTERS
#undef X

  d->contact = d->arena;
}


// initialize plugins, copy into d (required for deletion); if an init callback fails, destroy
// the instances already initialized and return the id of the failed instance, otherwise -1
int mj_initPlugin(const mjModel* m, mjData* d) {
  d->nplugin = m->nplugin;
  for (int i = 0; i < m->nplugin; ++i) {
    d->plugin[i] = m->plugin[i];
    d->plugin_data[i] = 0;
    const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[i]);
    if (plugin->init && plugin->init(m, d, i) < 0) {
      // destroy previously initialized instances
      for (int j=0; j < i; j++) {
        const mjpPlugin* prev = mjp_getPluginAtSlot(d->plugin[j]);
        if (prev->destroy) {
          prev->destroy(d, j);
        }
      }
      d->nplugin = 0;
      return i;
    }
  }
  return -1;
}


// initialize plugins of mjData allocated by the caller, free it and error on failure
static void initPluginOrFree(const mjModel* m, mjData* d) {
  int failed = mj_initPlugin(m, d);
  if (failed >= 0) {
    mju_free(d->buffer);
    mju_free(d->arena);
    mju_free(d);
    mjERROR("plugin->init failed for plugin id %d", failed);
  }
}


// free mjData memory without destroying the struct
static void freeDataBuffers(mjData* d) {
#ifdef mjUSEASAN
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
  uintptr_t save_threadpool = 0;

  // allocate mjData
  if (!allocate) {
    d = *dest;
    save_threadpool = d->threadpool;
    freeDataBuffers(d);
  } else {
    d = (mjData*) mju_malloc(sizeof(mjData));
  }

  if (!d) {
    mjERROR("could not allocate mjData");
  }

  // prevent spurious timing print from mj_resetData before _resetData zeroes the struct
  d->timer[mjTIMER_STEP].number = 0;

  // compute buffer size, including padding to align the first array at any buffer address
  d->nbuffer = BUFFER_ALIGN - 1;
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

  // preserve or clear threadpool
  d->threadpool = save_threadpool;
  d->threadlock = 0;

  // clear nplugin (overwritten by _initPlugin)
  d->nplugin = 0;

  // set all trees awake, treat static bodies as awake: plugin init can call mj_kinematics
  mju_fillInt(d->tree_asleep, kAwake, m->ntree);
  mj_updateSleepInit(m, d, /*flg_staticawake*/ 1);

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
    initPluginOrFree(m, d);
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
    initPluginOrFree(m, dest);
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
  uintptr_t save_threadpool = dest->threadpool;
  *dest = *src;
  dest->buffer = save_buffer;
  dest->arena = save_arena;
  dest->threadpool = save_threadpool;
  dest->threadlock = 0;
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

  return dest;
}


mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src) {
  return mj_copyDataVisual(dest, m, src, /*flg_all=*/1);
}


mjData* mjv_copyData(mjData* dest, const mjModel* m, const mjData* src) {
  return mj_copyDataVisual(dest, m, src, /*flg_all=*/0);
}


// number of position and velocity coordinates for each joint type
const int nPOS[4] = {7, 4, 1, 1};
const int nVEL[4] = {6, 3, 1, 1};


// initialize plastic state from model spring references
void mj_resetPlastic(const mjModel* m, mjData* d) {
  if (!m->nplastic) {
    return;
  }
  if (m->qpos_spring && m->jnt_plasticadr) {
    for (int j=0; j < m->njnt; j++) {
      int adr = m->jnt_plasticadr[j];
      if (adr >= 0) {
        mju_copy(d->plastic + adr, m->qpos_spring + m->jnt_qposadr[j], nPOS[m->jnt_type[j]]);
      }
    }
  }
  if (m->tendon_lengthspring && m->tendon_plasticadr) {
    for (int i=0; i < m->ntendon; i++) {
      int adr = m->tendon_plasticadr[i];
      if (adr >= 0) {
        mju_copy(d->plastic + adr, m->tendon_lengthspring + 2*i, 2);
      }
    }
  }
  if (m->flex_plasticadr) {
    for (int f=0; f < m->nflex; f++) {
      int adr = m->flex_plasticadr[f];
      if (adr >= 0) {
        int dim = m->flex_dim[f];
        int n = 0;
        if (m->flex_interp[f] > 0) {
          n = 6 * m->flex_cellnum[3*f+0] * m->flex_cellnum[3*f+1] * m->flex_cellnum[3*f+2];
        } else if (m->flex_interp[f] < 0) {
          int cx = m->flex_cellnum[3*f+0];
          int cy = m->flex_cellnum[3*f+1];
          int cz = m->flex_cellnum[3*f+2];
          n = 3 * 2 * (cy*cz + cx*cz + cx*cy);
        } else if (dim == 1) {
          n = mj_rodStateSize(m, f);
        } else {
          if (m->flex_plasticmode[f] & 2) {
            n += (dim == 2 ? 3 : 6) * m->flex_elemnum[f];
          }
          if (m->flex_plasticmode[f] & 1) {
            n += m->flex_edgenum[f];
          }
        }
        mju_zero(d->plastic + adr, n);
        if (d->flex_hessian_valid) {
          d->flex_hessian_valid[f] = 0;
        }
      }
    }
  }
}


// clear data, set defaults; must not call mju_error (warn instead), as callers rely on
// mj_resetData as an infallible recovery path to restore a valid mjData after a trapped error
static void _resetData(const mjModel* m, mjData* d, unsigned char debug_value) {
  // warn if history buffers cannot be initialized
  mjtNum dt = m->opt.timestep;
  if (m->nhistory && dt <= 0) {
    mju_warning("history buffers require positive timestep, got %g", dt);
  }

  //------------------------------ save plugin state and data
  mjtNum* plugin_state = NULL;
  uintptr_t* plugindata = NULL;
  if (d->nplugin) {
    if (m->npluginstate) {
      plugin_state = mju_malloc(sizeof(mjtNum) * m->npluginstate);
      memcpy(plugin_state, d->plugin_state, sizeof(mjtNum) * m->npluginstate);
    }
    plugindata = mju_malloc(sizeof(uintptr_t) * m->nplugin);
    memcpy(plugindata, d->plugin_data, sizeof(uintptr_t) * m->nplugin);
  }

  //------------------------------ clear header

  // clear stack pointer
  d->pstack = 0;
  d->pbase = 0;
  d->threadlock = 0;

  // clear arena pointers
  d->parena = 0;

  // poison the entire arena+stack memory region when built with asan
#ifdef mjUSEASAN
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
  d->nY = 0;
  d->nA = 0;
  d->nisland = 0;
  d->nidof = 0;
  d->efm_active = 0;
  d->nefmK = 0;
  d->nefmcon = 0;
  d->nefmT = 0;
  d->nefmA = 0;
  d->nefmdof = 0;
  d->nefmL = 0;

  // clear global properties
  d->time = 0;
  mju_zero(d->energy, 2);

  // clear lazy evaluation flags
  d->flg_energypos = 0;
  d->flg_energyvel = 0;
  d->flg_subtreevel = 0;
  d->flg_rnepost = 0;

  //------------------------------ clear buffer, set defaults

  // fill buffer with debug_value (normally 0)
#ifdef mjUSEASAN
  {
    #define X(type, name, nr, nc) memset(d->name, (int)debug_value, sizeof(type)*(m->nr)*(nc));
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
  mju_zero(d->plastic, m->nplastic);
  mj_resetCtrl(m, d);
  for (int i=0; i < m->neq; i++) d->eq_active[i] = m->eq_active0[i];
  mju_zero(d->qfrc_applied, m->nv);
  mju_zero(d->xfrc_applied, 6*m->nbody);
  mju_zero(d->qacc, m->nv);  // input to inverse dynamics
  mju_zero(d->qacc_warmstart, m->nv);
  mju_zero(d->act_dot, m->na);
  mju_zero(d->userdata, m->nuserdata);
  mju_zero(d->mocap_pos, 3*m->nmocap);
  mju_zero(d->mocap_quat, 4*m->nmocap);
  mju_zero(d->flexvert_lambda, m->nflexvert);
  mju_zeroInt(d->flexvert_conage, m->nflexvert);
  memset(d->flex_hessian_valid, 0, m->nflex * sizeof(mjtBool));

  // initialize ctrl history buffers: timestamps at [-n*dt, ..., -dt]
  for (int i = 0; i < m->nactuator; i++) {
    int n = m->actuator_history[2*i];
    if (n > 0) {
      mjtNum* buf = d->history + m->actuator_historyadr[i];
      buf[0] = 0;          // user slot
      buf[1] = n - 1;      // cursor: newest at logical index n-1
      mjtNum* times = buf + 2;
      for (int j = 0; j < n; j++) {
        times[j] = -(n-j)*dt;
      }

      // clear values (or set identity quaternion for quat inputs)
      mjtNum* values = buf + 2 + n;
      int dim = m->actuator_ctrlnum[i];
      mju_zero(values, n*dim);
      if (m->actuator_gaintype[i] == mjGAIN_SO3 && m->actuator_ctrlspec[i] == mjCHART_QUAT) {
        for (int j = 0; j < n; j++) {
          values[4*j] = 1;
        }
      }
    }
  }

  // initialize sensor history buffers
  for (int i = 0; i < m->nsensor; i++) {
    int n = m->sensor_history[2*i];
    if (n > 0) {
      int dim = m->sensor_dim[i];
      mjtNum period = m->sensor_interval[2*i];
      mjtNum phase = m->sensor_interval[2*i+1];
      mjtNum* buf = d->history + m->sensor_historyadr[i];

      // user slot: last compute time (phase=0 means -period, i.e. first compute at t=0)
      mjtNum t0 = (phase == 0) ? -period : phase;
      buf[0] = (period > 0) ? t0 : -dt;
      buf[1] = n - 1;  // cursor: newest at logical index n-1

      mjtNum* times = buf + 2;
      if (period > 0) {
        if (dt <= 0) {
          for (int j = 0; j < n; j++) {
            times[j] = t0 - (n-1-j)*period;
          }
        } else if (period <= dt) {
          // sub-timestep or equal period: exact integer dt grid
          int offset = (phase == 0) ? n : n - 1;
          for (int j = 0; j < n; j++) {
            times[j] = -(offset-j)*dt;
          }
        } else {
          // samples spaced at period intervals, rounded up to dt grid
          // relative near-integer snap and k >= next_k clamp are floating-point safety nets
          mjtNum next_k = 0;
#ifdef mjUSESINGLE
          const mjtNum reltol = 1e-6f;
#else
          const mjtNum reltol = 1e-12;
#endif
          for (int j = n - 1; j >= 0; j--) {
            mjtNum r = (t0 - (n-1-j)*period) / dt;
            mjtNum r_round = mju_floor(r + 0.5);
            if (mju_abs(r - r_round) <= reltol * mju_max(1, mju_abs(r))) {
              r = r_round;
            }
            mjtNum k = mju_ceil(r);
            if (j < n - 1 && k >= next_k) {
              k = next_k - 1;
            }
            times[j] = k * dt;
            next_k = k;
          }
        }
      } else {
        // no period: timestamps at [-n*dt, ..., -dt]
        for (int j = 0; j < n; j++) {
          times[j] = -(n-j)*dt;
        }
      }

      // clear values
      mjtNum* values = buf + 2 + n;
      mju_zero(values, n*dim);
    }
  }

  // copy qpos0 from model
  if (m->qpos0) {
    mju_copy(d->qpos, m->qpos0, m->nq);
  }

  // initialize plastic state from model spring references
  mj_resetPlastic(m, d);

  // restore pluginstate and plugindata
  if (d->nplugin) {
    if (m->npluginstate) {
      memcpy(d->plugin_state, plugin_state, sizeof(mjtNum) * m->npluginstate);
      mju_free(plugin_state);
    }
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

      // warn if any failed to sleep, leave them awake
      if (nslept != num_asleep_init) {
        // find root body of the first tree that could not be slept, wake the others
        int root = -1;
        for (int i=0; i < m->ntree; i++) {
          if (m->tree_sleep_policy[i] == mjSLEEP_INIT && d->tree_asleep[i] < 0) {
            if (root < 0) root = m->tree_bodyadr[i];
            d->tree_asleep[i] = kAwake;
          }
        }

        // warn and continue
        const char* hasname = mj_id2name(m, mjOBJ_BODY, root);
        const char* name = hasname ? hasname : "";
        mju_warning("%d trees were marked as sleep='init' but only %d could be slept. "
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

  // copy signature from model
  d->signature = m->signature;
}


// emit step timing diagnostics
static void mj_logTimingDiagnostics(const mjData* d) {
  int nstep = d->timer[mjTIMER_STEP].number;
  if (nstep <= 0) {
    return;
  }

  mjtNum tstep = d->timer[mjTIMER_STEP].duration / nstep;
  if (tstep <= 0) {
    return;
  }

  char buf[2048];
  int pos = 0;
  mjtNum components = 0;

#define MJ_APPEND_TIMING(...) \
  do { \
    if (pos < (int)sizeof(buf) - 1) { \
      int n = snprintf(buf + pos, sizeof(buf) - pos, __VA_ARGS__); \
      if (n > 0) { \
        pos = mjMIN((int)sizeof(buf) - 1, pos + n); \
      } \
    } \
  } while (0)

  for (int i = mjTIMER_POSITION; i <= mjTIMER_ADVANCE; i++) {
    if (d->timer[i].number > 0) {
      mjtNum istep = d->timer[i].duration / d->timer[i].number;
      components += istep;
      MJ_APPEND_TIMING("%s  %-15s %8.1f  (%5.1f%%)",
                       pos > 0 ? "\n" : "",
                       mjTIMERSTRING[i], istep * 1000, 100 * istep / tstep);

      // position sub-breakdown
      if (i == mjTIMER_POSITION) {
        for (int p = mjTIMER_POS_KINEMATICS; p <= mjTIMER_POS_PROJECT; p++) {
          if (d->timer[p].number > 0) {
            mjtNum pstep = d->timer[p].duration / d->timer[p].number;
            MJ_APPEND_TIMING("\n    %-13s %8.1f  (%5.1f%%)",
                             mjTIMERSTRING[p] + 4, pstep * 1000, 100 * pstep / tstep);

            // collision sub-breakdown
            if (p == mjTIMER_POS_COLLISION) {
              for (int c = mjTIMER_COL_BROAD; c <= mjTIMER_COL_NARROW; c++) {
                if (d->timer[c].number > 0) {
                  mjtNum cstep = d->timer[c].duration / d->timer[c].number;
                  MJ_APPEND_TIMING("\n      %-11s %8.1f  (%5.1f%%)",
                                   mjTIMERSTRING[c] + 4, cstep * 1000, 100 * cstep / tstep);
                }
              }
            }
          }
        }
      }
    }
  }

  mjtNum other = tstep - components;
  MJ_APPEND_TIMING("%s  %-15s %8.1f  (%5.1f%%)",
                   pos > 0 ? "\n" : "",
                   "other", other * 1000, 100 * other / tstep);

  MJ_APPEND_TIMING("%s  %-15s %8.1f",
                   pos > 0 ? "\n" : "",
                   "total", tstep * 1000);

#undef MJ_APPEND_TIMING

  mjLogMessage msg = {.level = mjLOG_INFO, .topic = mjTOPIC_TIME_STP, .body = buf};
  snprintf(msg.subject, sizeof(msg.subject),
           "average time per step (%d steps, units: \u00B5s)", nstep);
  mju_message(&msg);
}


// set ctrl to neutral values: zero, except quaternion inputs which reset to the identity
void mj_resetCtrl(const mjModel* m, mjData* d) {
  mju_zero(d->ctrl, m->nu);
  for (int i=0; i < m->nactuator; i++) {
    if (m->actuator_gaintype[i] == mjGAIN_SO3 && m->actuator_ctrlspec[i] == mjCHART_QUAT) {
      d->ctrl[m->actuator_ctrladr[i]] = 1;
    }
  }
}


// clear data, set data->qpos = model->qpos0
void mj_resetData(const mjModel* m, mjData* d) {
  // emit step timing diagnostics before timers are cleared
  mj_logTimingDiagnostics(d);

  _resetData(m, d, 0);
}



// clear data, set data->qpos = model->qpos0, fill with debug_value
void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value) {
  _resetData(m, d, debug_value);
}


// Reset data. If 0 <= key < nkey, set fields from specified keyframe.
void mj_resetDataKeyframe(const mjModel* m, mjData* d, int key) {
  // emit step timing diagnostics before timers are cleared
  mj_logTimingDiagnostics(d);

  _resetData(m, d, 0);

  // copy keyframe data if key is valid
  if (key >= 0 && key < m->nkey) {
    d->time = m->key_time[key];

    // sleep='init' trees with displaced key_qpos or nonzero key_qvel are left for
    // mj_kinematics and mj_wake to wake on the next forward pass
    mju_copy(d->qpos, m->key_qpos+key*m->nq, m->nq);
    mju_copy(d->qvel, m->key_qvel+key*m->nv, m->nv);
    mju_copy(d->act,  m->key_act+ key*m->na, m->na);
    mju_copy(d->mocap_pos,  m->key_mpos+key*3*m->nmocap, 3*m->nmocap);
    mju_copy(d->mocap_quat, m->key_mquat+key*4*m->nmocap, 4*m->nmocap);
    mju_copy(d->ctrl, m->key_ctrl+key*m->nu, m->nu);
    mju_copy(d->plastic, m->key_plastic+key*m->nplastic, m->nplastic);
  }
}


// de-allocate mjData
void mj_deleteData(mjData* d) {
  if (d) {
    mju_threadpool(d, 0);
    freeDataBuffers(d);
    mju_free(d);
  }
}


// return the number of values written by sensor i, -1 if its type or its object type is invalid;
// assumes that sensor_objid was validated and that the sensor is not a plugin
static mjtSize sensorSize(const mjModel* m, int i) {
  int objtype = m->sensor_objtype[i];
  int objid = m->sensor_objid[i];
  int dataspec = m->sensor_intprm[i*mjNSENS];

  switch ((mjtSensor) m->sensor_type[i]) {
  case mjSENS_TOUCH:
  case mjSENS_JOINTPOS:
  case mjSENS_JOINTVEL:
  case mjSENS_TENDONPOS:
  case mjSENS_TENDONVEL:
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

  case mjSENS_ACTUATORPOS:
  case mjSENS_ACTUATORVEL:
  case mjSENS_ACTUATORFRC:
    // one value per output of the actuator
    return (objtype == mjOBJ_ACTUATOR) ? m->actuator_outnum[objid] : -1;

  case mjSENS_RANGEFINDER:
    // one ray for a site, one ray per pixel for a camera
    if (dataspec <= 0 || dataspec >= (1 << mjNRAYDATA)) {
      return -1;
    }
    if (objtype == mjOBJ_SITE) {
      return mju_raydataSize(dataspec);
    } else if (objtype == mjOBJ_CAMERA) {
      // 3 values per pixel are allocated on the stack, their number must fit in an int,
      // as must the number of values of the sensor
      mjtSize npixel = boundedProduct(m->cam_resolution[2*objid], m->cam_resolution[2*objid+1],
                                      INT_MAX / 3);
      return boundedProduct(npixel, mju_raydataSize(dataspec), INT_MAX);
    }
    return -1;

  case mjSENS_CONTACT: {
    // a number of slots, each with the fields in the data specification: at least one
    int nslot = m->sensor_intprm[i*mjNSENS+2];
    if (dataspec <= 0 || dataspec >= (1 << mjNCONDATA) || nslot <= 0) {
      return -1;
    }
    return boundedProduct(nslot, mju_condataSize(dataspec), INT_MAX);
  }

  case mjSENS_TACTILE:
    // 3 values per vertex of the mesh, with a geom reference
    if (objtype != mjOBJ_MESH || m->sensor_reftype[i] != mjOBJ_GEOM ||
        m->sensor_refid[i] < 0 || m->mesh_vertnum[objid] <= 0) {
      return -1;
    }
    return 3*(mjtSize)m->mesh_vertnum[objid];

  case mjSENS_USER:
    // the size is defined by sensor_dim
    return m->sensor_dim[i];

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
    return m->nactuator;
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


// validate CSR sparse matrix structure; return 0 if valid, error code otherwise
//   1: rownnz out of bounds
//   2: rowadr out of bounds
//   3: colind out of bounds or unsorted
static int validateSparse(int nrows, int ncols, mjtSize nnz,
                          const int* rownnz, const int* rowadr, const int* colind,
                          int strict) {
  for (int r=0; r < nrows; r++) {
    int nnz_r = rownnz[r];
    if (nnz_r < 0 || nnz_r > ncols) {
      return 1;
    }
    int adr_r = rowadr[r];
    if (!validRange(adr_r, nnz_r, nnz)) {
      return 2;
    }
    for (int k=0; k < nnz_r; k++) {
      int col = colind[adr_r+k];
      if (col < 0 || col >= ncols) {
        return 3;
      }
      if (k > 0) {
        int prev = colind[adr_r+k-1];
        if (strict ? (col <= prev) : (col < prev)) {
          return 3;
        }
      }
    }
  }
  return 0;
}


// validate a history buffer with nsample samples of dimension dim at address adr,
// return null if valid, error message otherwise
static const char* validateHistory(int nsample, int dim, int adr, mjtSize nhistory) {
  if (nsample < 0) {
    return "Invalid model: negative number of history samples.";
  }

  // layout is [user, cursor, times(nsample), values(nsample*dim)]
  if (nsample > 0 && !validRange(adr, 2 + (mjtSize)nsample + (mjtSize)nsample*dim, nhistory)) {
    return "Invalid model: history address out of bounds.";
  }
  return NULL;
}


// return an upper bound on the number of non-zeros written to actuator_moment by mj_transmission,
// -1 if a transmission or the output rows of an actuator are invalid: each actuator has as many
// output rows as its transmission writes, with no gaps between actuators;
// assumes that the other references of the model were already validated
static mjtSize momentSize(const mjModel* m) {
  int nv = m->nv;
  mjtSize count = 0, outadr = 0;
  int valid = 1;

  for (int i=0; valid && i < m->nactuator; i++) {
    int id = m->actuator_trnid[2*i];
    int refid = m->actuator_trnid[2*i+1];
    int trntype = m->actuator_trntype[i];

    // rows are written in actuator order, each row starts where the previous one ends
    int nrow = (trntype == mjTRN_SO3) ? 3 : 1;
    if (m->actuator_outadr[i] != outadr || m->actuator_outnum[i] != nrow) {
      valid = 0;
      break;
    }
    outadr += m->actuator_outnum[i];

    int b1 = 0, b2 = 0;
    switch ((mjtTrn) trntype) {
    case mjTRN_JOINT:
    case mjTRN_JOINTINPARENT:
      count += nVEL[m->jnt_type[id]];
      break;

    case mjTRN_SLIDERCRANK:
      b1 = m->site_bodyid[id];
      b2 = m->site_bodyid[refid];
      valid = (b1 >= 0 && b2 >= 0);
      count += valid ? mj_mergeChain(m, NULL, b1, b2, /*flg_skipcommon=*/0) : 0;
      break;

    case mjTRN_TENDON:
      count += m->ten_J_rownnz[id];
      break;

    case mjTRN_SO3:
      // ball joint: 3 identity rows; site and refsite: 3 dense rows
      count += (refid == -1) ? 3 : 3*(mjtSize)nv;
      break;

    case mjTRN_SITE:
      b1 = m->site_bodyid[id];
      b2 = (refid >= 0) ? m->site_bodyid[refid] : 0;
      valid = (b1 >= 0 && b2 >= 0);
      count += valid ? mj_mergeChain(m, NULL, b1, b2, /*flg_skipcommon=*/1) : 0;
      break;

    case mjTRN_BODY:
      count += nv;
      break;

    default:
      valid = 0;
    }
  }

  // the rows of the actuators cover all of the outputs
  if (outadr != m->nout) {
    valid = 0;
  }

  return valid ? count : -1;
}


// validate the layout of joints and dofs in the bodies, and of dofs and bodies in the trees;
// the engine walks these without checks, e.g. a body has a buffer for 6 dofs in mj_comVel
static const char* validateTreeLayout(const mjModel* m) {
  // joints and dofs are numbered in body order, the dofs of a body are those of its joints
  int dofadr = 0, jntadr = 0;
  for (int i=0; i < m->nbody; i++) {
    int dofnum = m->body_dofnum[i];
    int jntnum = m->body_jntnum[i];
    if (dofnum > 6) {
      return "Invalid model: body has more than 6 dofs.";
    }
    if ((dofnum > 0 && m->body_dofadr[i] != dofadr) || (jntnum > 0 && m->body_jntadr[i] != jntadr)) {
      return "Invalid model: body_jntadr or body_dofadr is not in body order.";
    }

    int ndof = 0;
    for (int j=jntadr; j < jntadr+jntnum; j++) {
      int type = m->jnt_type[j];
      if (m->jnt_bodyid[j] != i || m->jnt_dofadr[j] != dofadr+ndof) {
        return "Invalid model: jnt_bodyid or jnt_dofadr does not match the body.";
      }
      if (type == mjJNT_FREE && jntnum != 1) {
        return "Invalid model: free joint is not the only joint of its body.";
      }
      if (ndof + nVEL[type] > dofnum) {
        return "Invalid model: body_dofnum does not match the joints of the body.";
      }
      for (int k=0; k < nVEL[type]; k++) {
        if (m->dof_jntid[dofadr+ndof+k] != j || m->dof_bodyid[dofadr+ndof+k] != i) {
          return "Invalid model: dof_jntid or dof_bodyid does not match the joint.";
        }
      }
      ndof += nVEL[type];
    }
    if (ndof != dofnum) {
      return "Invalid model: body_dofnum does not match the joints of the body.";
    }
    dofadr += dofnum;
    jntadr += jntnum;
  }
  if (dofadr != m->nv || jntadr != m->njnt) {
    return "Invalid model: joints or dofs which do not belong to a body.";
  }

  // a tree starts at each dof without a parent, its dofs are contiguous
  int ntree = 0;
  for (int i=0; i < m->nv; i++) {
    if (m->dof_parentid[i] == -1) {
      if (ntree >= m->ntree || m->tree_dofadr[ntree] != i) {
        return "Invalid model: tree_dofadr does not match the dofs.";
      }
      ntree++;
    }
    if (m->dof_treeid[i] != ntree-1) {
      return "Invalid model: dof_treeid does not match the dofs.";
    }
  }
  if (ntree != m->ntree) {
    return "Invalid model: ntree does not match the dofs.";
  }
  for (int i=0; i < ntree; i++) {
    int end = (i+1 < ntree) ? m->tree_dofadr[i+1] : m->nv;
    if (m->tree_dofnum[i] != end - m->tree_dofadr[i]) {
      return "Invalid model: tree_dofnum does not match the dofs.";
    }
  }

  // a body is in the tree of the dofs of its weld body, the bodies of a tree are in tree order
  int tree = -1, bodynum = 0;
  for (int i=0; i < m->nbody; i++) {
    int weldid = m->body_weldid[i];
    int treeid = m->body_dofnum[weldid] ? m->dof_treeid[m->body_dofadr[weldid]] : -1;
    if (m->body_treeid[i] != treeid) {
      return "Invalid model: body_treeid does not match the dofs.";
    }
    if (treeid == -1) {
      continue;
    }
    if (treeid != tree) {
      if (treeid != tree+1 || m->tree_bodyadr[treeid] != i) {
        return "Invalid model: tree_bodyadr does not match the bodies.";
      }
      if (tree >= 0 && m->tree_bodynum[tree] != bodynum) {
        return "Invalid model: tree_bodynum does not match the bodies.";
      }
      tree = treeid;
      bodynum = 0;
    }

    // the bodies of a tree are contiguous, a static body cannot be between them
    if (m->tree_bodyadr[tree] + bodynum != i) {
      return "Invalid model: bodies of a tree are not contiguous.";
    }
    bodynum++;
  }
  if (tree != ntree-1 || (tree >= 0 && m->tree_bodynum[tree] != bodynum)) {
    return "Invalid model: tree_bodynum does not match the bodies.";
  }

  return NULL;
}


// return 1 if the objects of sensor i have the types that its sensor type reads, 0 otherwise;
// the ids of the objects are validated against these types, assumes they were already validated
static int validSensorObjects(const mjModel* m, int i) {
  int objtype = m->sensor_objtype[i];
  int reftype = m->sensor_reftype[i];
  int objid = m->sensor_objid[i];

  // objects with a spatial frame
  int objframe = (objtype == mjOBJ_BODY || objtype == mjOBJ_XBODY || objtype == mjOBJ_GEOM ||
                  objtype == mjOBJ_SITE || objtype == mjOBJ_CAMERA);
  int refframe = (reftype == mjOBJ_BODY || reftype == mjOBJ_XBODY || reftype == mjOBJ_GEOM ||
                  reftype == mjOBJ_SITE || reftype == mjOBJ_CAMERA);

  // an object has an id if and only if it has a type, a reference object has a spatial frame
  if ((objtype == mjOBJ_UNKNOWN) != (objid == -1) ||
      (reftype == mjOBJ_UNKNOWN) != (m->sensor_refid[i] == -1) ||
      (reftype != mjOBJ_UNKNOWN && !refframe)) {
    return 0;
  }

  switch ((mjtSensor) m->sensor_type[i]) {
  case mjSENS_TOUCH:
  case mjSENS_ACCELEROMETER:
  case mjSENS_VELOCIMETER:
  case mjSENS_GYRO:
  case mjSENS_FORCE:
  case mjSENS_TORQUE:
  case mjSENS_MAGNETOMETER:
    return objtype == mjOBJ_SITE;

  case mjSENS_CAMPROJECTION:
    return objtype == mjOBJ_SITE && reftype == mjOBJ_CAMERA;

  case mjSENS_RANGEFINDER:
    return objtype == mjOBJ_SITE || objtype == mjOBJ_CAMERA;

  case mjSENS_JOINTPOS:
  case mjSENS_JOINTVEL:
  case mjSENS_JOINTACTFRC:
  case mjSENS_JOINTLIMITPOS:
  case mjSENS_JOINTLIMITVEL:
  case mjSENS_JOINTLIMITFRC:
    return objtype == mjOBJ_JOINT;

  case mjSENS_BALLQUAT:
  case mjSENS_BALLANGVEL:
    return objtype == mjOBJ_JOINT && m->jnt_type[objid] == mjJNT_BALL;

  case mjSENS_TENDONPOS:
  case mjSENS_TENDONVEL:
  case mjSENS_TENDONACTFRC:
  case mjSENS_TENDONLIMITPOS:
  case mjSENS_TENDONLIMITVEL:
  case mjSENS_TENDONLIMITFRC:
    return objtype == mjOBJ_TENDON;

  case mjSENS_ACTUATORPOS:
  case mjSENS_ACTUATORVEL:
  case mjSENS_ACTUATORFRC:
    return objtype == mjOBJ_ACTUATOR;

  case mjSENS_FRAMEPOS:
  case mjSENS_FRAMEQUAT:
  case mjSENS_FRAMEXAXIS:
  case mjSENS_FRAMEYAXIS:
  case mjSENS_FRAMEZAXIS:
  case mjSENS_FRAMELINVEL:
  case mjSENS_FRAMEANGVEL:
  case mjSENS_FRAMELINACC:
  case mjSENS_FRAMEANGACC:
    return objframe;

  case mjSENS_SUBTREECOM:
  case mjSENS_SUBTREELINVEL:
  case mjSENS_SUBTREEANGMOM:
    return objtype == mjOBJ_BODY;

  case mjSENS_INSIDESITE:
    return objframe && reftype == mjOBJ_SITE;

  case mjSENS_GEOMDIST:
  case mjSENS_GEOMNORMAL:
  case mjSENS_GEOMFROMTO:
    return (objtype == mjOBJ_BODY || objtype == mjOBJ_GEOM) &&
           (reftype == mjOBJ_BODY || reftype == mjOBJ_GEOM);

  case mjSENS_CONTACT:
    return (objtype == mjOBJ_UNKNOWN || objtype == mjOBJ_SITE || objtype == mjOBJ_BODY ||
            objtype == mjOBJ_XBODY || objtype == mjOBJ_GEOM) &&
           (reftype == mjOBJ_UNKNOWN || reftype == mjOBJ_BODY || reftype == mjOBJ_XBODY ||
            reftype == mjOBJ_GEOM);

  case mjSENS_TACTILE:
    return objtype == mjOBJ_MESH && reftype == mjOBJ_GEOM;

  case mjSENS_E_POTENTIAL:
  case mjSENS_E_KINETIC:
  case mjSENS_CLOCK:
  case mjSENS_USER:
  case mjSENS_PLUGIN:
    // objects are not used
    return 1;
  }

  // unknown sensor type, rejected with its size
  return 1;
}


// validate the character buffers and the hash map of the names
static const char* validateStrings(const mjModel* m) {
  // strings are read up to their terminator, the last one must be in the buffer
  if (m->nnames > 0 && m->names[m->nnames-1]) {
    return "Invalid model: names is not terminated.";
  }
  if (m->npaths > 0 && m->paths[m->npaths-1]) {
    return "Invalid model: paths is not terminated.";
  }
  for (int i=0; i < m->ntext; i++) {
    if (m->text_size[i] < 1 || m->text_data[(mjtSize)m->text_adr[i] + m->text_size[i] - 1]) {
      return "Invalid model: text_data is not terminated.";
    }
  }

  // names_map has a section of mjLOAD_MULTIPLE*n slots per object type, in the order below;
  // a slot is the id of an object or -1, a lookup ends at the first empty slot
  mjtSize num[] = {
    m->nbody, m->njnt, m->ngeom, m->nsite, m->ncam, m->nlight, m->nflex, m->nmesh, m->nskin,
    m->nhfield, m->ntex, m->nmat, m->npair, m->nexclude, m->neq, m->ntendon, m->nactuator,
    m->nsensor, m->nnumeric, m->ntext, m->ntuple, m->nkey, m->nplugin
  };
  mjtSize adr = 0;
  for (int s=0; s < sizeof(num)/sizeof(num[0]); s++) {
    mjtSize nslot = mjLOAD_MULTIPLE*num[s];
    if (!validRange(adr, nslot, m->nnames_map)) {
      return "Invalid model: names_map is too small.";
    }
    int nempty = 0;
    for (mjtSize k=0; k < nslot; k++) {
      int id = m->names_map[adr+k];
      if (id < -1 || id >= num[s]) {
        return "Invalid model: names_map out of bounds.";
      }
      nempty += (id == -1);
    }
    if (nslot > 0 && !nempty) {
      return "Invalid model: names_map has no empty slot.";
    }
    adr += nslot;
  }

  return NULL;
}


// validate the plugin instances
static const char* validatePlugins(const mjModel* m) {
  for (int i=0; i < m->nplugin; i++) {
    // the plugin is dereferenced without checks, e.g. in mj_makeData
    const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[i]);
    if (!plugin) {
      return "Invalid model: plugin is not registered.";
    }

    // attributes are found by counting terminators, they must all be in the buffer
    int nattribute = mjMAX(1, plugin->nattribute);
    int nterminator = 0;
    for (mjtSize k=m->plugin_attradr[i]; k < m->npluginattr && nterminator < nattribute; k++) {
      nterminator += (m->plugin_attr[k] == 0);
    }
    if (nterminator < nattribute) {
      return "Invalid model: plugin_attr is not terminated.";
    }
  }

  return NULL;
}


// validate the paths of the tendons
static const char* validateTendons(const mjModel* m) {
  for (int i=0; i < m->ntendon; i++) {
    int type = m->tendon_type[i];
    int num = m->tendon_num[i];
    const int* wraptype = m->wrap_type + m->tendon_adr[i];
    const mjtNum* wrapprm = m->wrap_prm + m->tendon_adr[i];

    if (type != mjTENDON_FIXED && type != mjTENDON_SPATIAL) {
      return "Invalid model: tendon_type out of bounds.";
    }

    // the path is read without checking its length
    if (num < 1 || (type == mjTENDON_SPATIAL && num < 2)) {
      return "Invalid model: tendon_num is too small.";
    }

    for (int j=0; j < num; j++) {
      // fixed tendon: every object is used as a joint
      if (type == mjTENDON_FIXED) {
        if (wraptype[j] != mjWRAP_JOINT) {
          return "Invalid model: wrap_type of a fixed tendon is not a joint.";
        }
        continue;
      }

      // spatial tendon: sites and pulleys, a wrapping geom is read together with the sites
      // before and after it and with an optional side site, whose id is its parameter
      switch ((mjtWrap) wraptype[j]) {
      case mjWRAP_SITE:
      case mjWRAP_PULLEY:
        break;

      case mjWRAP_SPHERE:
      case mjWRAP_CYLINDER:
        if (j == 0 || j == num-1 || wraptype[j-1] != mjWRAP_SITE || wraptype[j+1] != mjWRAP_SITE) {
          return "Invalid model: wrapping geom is not between two sites.";
        }
        // the id is rounded by the engine, the range is checked first to reject NaN
        if (!(wrapprm[j] >= -1 && wrapprm[j] < m->nsite) || mju_round(wrapprm[j]) >= m->nsite) {
          return "Invalid model: side site of a wrapping geom out of bounds.";
        }
        break;

      default:
        return "Invalid model: wrap_type of a spatial tendon out of bounds.";
      }
    }

    // trees of the tendon, used to wake them up
    int treenum = m->tendon_treenum[i];
    if (treenum < 0 || (treenum >= 1 && m->tendon_treeid[2*i] < 0) ||
        (treenum >= 2 && m->tendon_treeid[2*i+1] < 0)) {
      return "Invalid model: tendon_treeid does not match tendon_treenum.";
    }
  }

  return NULL;
}


// validate the controls and activations read and written by the actuators;
// assumes that their transmissions and address ranges were already validated
static const char* validateActuators(const mjModel* m) {
  for (int i=0; i < m->nactuator; i++) {
    const mjtNum* dynprm = m->actuator_dynprm + mjNDYN*i;
    const mjtNum* gainprm = m->actuator_gainprm + mjNGAIN*i;
    int trntype = m->actuator_trntype[i];
    int dyntype = m->actuator_dyntype[i];
    int gaintype = m->actuator_gaintype[i];
    int biastype = m->actuator_biastype[i];
    int ctrlspec = m->actuator_ctrlspec[i];
    int ctrlnum = m->actuator_ctrlnum[i];
    int actnum = m->actuator_actnum[i];

    // joint target of an SO3 transmission: 4 positions and 3 dofs are used
    if (trntype == mjTRN_SO3 && m->actuator_trnid[2*i+1] == -1 &&
        m->jnt_type[m->actuator_trnid[2*i]] != mjJNT_BALL) {
      return "Invalid model: joint of an SO3 transmission is not a ball joint.";
    }

    // activations written and controls read by the dynamics; the activations are counted from
    // the last one of the actuator, the controls from its first one, which can be out of bounds
    // if the actuator has no controls
    switch ((mjtDyn) dyntype) {
    case mjDYN_INTEGRATOR:
      // one activation per control
      if (actnum < 1 || actnum < ctrlnum) {
        return "Invalid model: actuator_actnum is too small for an integrator.";
      }
      break;

    case mjDYN_FILTER:
    case mjDYN_FILTEREXACT:
    case mjDYN_MUSCLE:
      if (actnum < 1) {
        return "Invalid model: actuator_actnum is too small for the dynamics.";
      }
      if (ctrlnum < 1) {
        return "Invalid model: actuator_ctrlnum is too small for the dynamics.";
      }
      break;

    case mjDYN_PID:
      if (actnum < (dynprm[1] > 0) + (gainprm[0] > 0)) {
        return "Invalid model: actuator_actnum is too small for a PID controller.";
      }
      if ((dynprm[1] > 0 || gainprm[0] > 0) && ctrlnum < 1) {
        return "Invalid model: actuator_ctrlnum is too small for a PID controller.";
      }
      break;

    case mjDYN_DCMOTOR:
      if ((dynprm[7] > 0 || gainprm[5] > 0) && ctrlnum < 1) {
        return "Invalid model: actuator_ctrlnum is too small for a DC motor.";
      }
      break;

    default:
      break;
    }

    // DC motor states: read by the dynamics, gain and bias according to mj_dcmotorSlots
    int is_dcmotor = (dyntype == mjDYN_DCMOTOR || gaintype == mjGAIN_DCMOTOR ||
                      biastype == mjBIAS_DCMOTOR);
    if (is_dcmotor && m->actuator_plugin[i] < 0 &&
        mj_dcmotorSlots(dynprm, gainprm).num_slots != actnum) {
      return "Invalid model: actuator_actnum does not match the states of a DC motor.";
    }

    // SO3 servo: 3 outputs; without dynamics the target is read from 4 or 3 controls,
    // with dynamics it is an exponential map in 3 activations
    if (gaintype == mjGAIN_SO3) {
      if (trntype != mjTRN_SO3) {
        return "Invalid model: SO3 gain without an SO3 transmission.";
      }
      if (dyntype == mjDYN_NONE ? ctrlnum < (ctrlspec == mjCHART_QUAT ? 4 : 3)
                                : (actnum < 3 || ctrlspec == mjCHART_QUAT)) {
        return "Invalid model: SO3 gain with too few controls or activations.";
      }
    }

    // servo inputs: one control per input in the specification, at most 4 for a DC motor
    if (gaintype == mjGAIN_PID || is_dcmotor) {
      int ninput = 0;
      for (int bit=mjINPUT_POS; bit <= mjINPUT_VOLTAGE; bit <<= 1) {
        ninput += (ctrlspec & bit) != 0;
      }
      if (ctrlnum < ninput || (is_dcmotor && ctrlnum > 4)) {
        return "Invalid model: actuator_ctrlnum does not match actuator_ctrlspec.";
      }
    }
  }

  return NULL;
}


// validate num nodes of a bounding volume hierarchy at address adr, whose leaves have ids
// smaller than nleaf; return 1 if valid, 0 otherwise
static int validBvh(const mjModel* m, int adr, int num, int nleaf) {
  // the root is read without checking the number of nodes
  if (num == 0) {
    return adr == -1;
  }

  for (int i=0; i < num; i++) {
    const int* child = m->bvh_child + 2*((mjtSize)adr + i);
    int nodeid = m->bvh_nodeid[adr+i];

    // leaf: no children, the id of its element
    if (child[0] == -1 && child[1] == -1) {
      if (nodeid < 0 || nodeid >= nleaf) {
        return 0;
      }
      continue;
    }

    // internal node: no id, two children which are both used without checks; they are local
    // and after their parent, the traversal stays in the tree and terminates
    if (nodeid != -1) {
      return 0;
    }
    for (int k=0; k < 2; k++) {
      if (child[k] <= i || child[k] >= num) {
        return 0;
      }
    }
  }

  return 1;
}


// validate the bounding volume hierarchies of bodies, meshes and flexes
static const char* validateBvh(const mjModel* m) {
  // static trees come first, bvh_aabb has no room for the others
  for (int i=0; i < m->nbody; i++) {
    if (!validBvh(m, m->body_bvhadr[i], m->body_bvhnum[i], m->ngeom) ||
        (m->body_bvhnum[i] > 0 && !validRange(m->body_bvhadr[i], m->body_bvhnum[i], m->nbvhstatic))) {
      return "Invalid model: bounding volume hierarchy of a body out of bounds.";
    }
  }
  for (int i=0; i < m->nmesh; i++) {
    if (!validBvh(m, m->mesh_bvhadr[i], m->mesh_bvhnum[i], m->mesh_facenum[i]) ||
        (m->mesh_bvhnum[i] > 0 && !validRange(m->mesh_bvhadr[i], m->mesh_bvhnum[i], m->nbvhstatic))) {
      return "Invalid model: bounding volume hierarchy of a mesh out of bounds.";
    }
  }

  // dynamic trees: their boxes are in mjData, indexed from nbvhstatic
  for (int i=0; i < m->nflex; i++) {
    if (!validBvh(m, m->flex_bvhadr[i], m->flex_bvhnum[i], m->flex_elemnum[i]) ||
        (m->flex_bvhnum[i] > 0 &&
         !validRange(m->flex_bvhadr[i] - m->nbvhstatic, m->flex_bvhnum[i], m->nbvhdynamic))) {
      return "Invalid model: bounding volume hierarchy of a flex out of bounds.";
    }
  }

  return NULL;
}


// validate the convex hull graphs and the octrees of the meshes
static const char* validateMeshes(const mjModel* m) {
  for (int i=0; i < m->nmesh; i++) {
    // graph: numvert, numface, vert_edgeadr(numvert), vert_globalid(numvert),
    //        edge_localid(numvert+3*numface), face_globalid(3*numface)
    mjtSize adr = m->mesh_graphadr[i];
    if (adr >= 0) {
      if (!validRange(adr, 2, m->nmeshgraph)) {
        return "Invalid model: mesh_graph out of bounds.";
      }
      const int* graph = m->mesh_graph + adr;
      mjtSize numvert = graph[0], numface = graph[1];
      mjtSize numedge = numvert + 3*numface;
      if (numvert < 1 || numface < 0 ||
          !validRange(adr, 2 + 3*numvert + 6*numface, m->nmeshgraph)) {
        return "Invalid model: mesh_graph out of bounds.";
      }
      const int* vert_edgeadr = graph + 2;
      const int* vert_globalid = vert_edgeadr + numvert;
      const int* edge_localid = vert_globalid + numvert;
      const int* face_globalid = edge_localid + numedge;
      for (mjtSize j=0; j < numvert; j++) {
        if (vert_edgeadr[j] < 0 || vert_edgeadr[j] >= numedge ||
            vert_globalid[j] < 0 || vert_globalid[j] >= m->mesh_vertnum[i]) {
          return "Invalid model: mesh_graph vertex out of bounds.";
        }
      }

      // the edges of a vertex are read up to a negative id, the last one must be in the graph
      for (mjtSize j=0; j < numedge; j++) {
        if (edge_localid[j] < -1 || edge_localid[j] >= numvert) {
          return "Invalid model: mesh_graph edge out of bounds.";
        }
      }
      if (edge_localid[numedge-1] != -1) {
        return "Invalid model: mesh_graph edges are not terminated.";
      }
      for (mjtSize j=0; j < 3*numface; j++) {
        if (face_globalid[j] < 0 || face_globalid[j] >= m->mesh_vertnum[i]) {
          return "Invalid model: mesh_graph face out of bounds.";
        }
      }

      // extreme vertices, used to start the search in the graph
      for (int j=0; j < 27; j++) {
        if (m->mesh_extrema[27*(mjtSize)i+j] < 0 || m->mesh_extrema[27*(mjtSize)i+j] >= numvert) {
          return "Invalid model: mesh_extrema out of bounds.";
        }
      }
    }

    // octree: the root is read without checking the number of nodes; a node is a leaf or has
    // 8 local children after it, the traversal stays in the tree and terminates
    int octadr = m->mesh_octadr[i];
    int octnum = m->mesh_octnum[i];
    if (octnum == 0 && octadr != -1) {
      return "Invalid model: mesh_octadr of a mesh without octree is not -1.";
    }
    for (int j=0; j < octnum; j++) {
      const int* child = m->oct_child + 8*((mjtSize)octadr + j);
      int nleaf = 0;
      for (int k=0; k < 8; k++) {
        nleaf += (child[k] == -1);
        if (child[k] != -1 && (child[k] <= j || child[k] >= octnum)) {
          return "Invalid model: oct_child out of bounds.";
        }
      }
      if (nleaf != 0 && nleaf != 8) {
        return "Invalid model: oct_child out of bounds.";
      }
    }
  }

  return NULL;
}


// validate the indices of the skins, which are local to the vertices of each skin
static const char* validateSkins(const mjModel* m) {
  for (int i=0; i < m->nskin; i++) {
    int vertnum = m->skin_vertnum[i];
    mjtSize faceadr = m->skin_faceadr[i];
    for (mjtSize j=3*faceadr; j < 3*(faceadr + m->skin_facenum[i]); j++) {
      if (m->skin_face[j] < 0 || m->skin_face[j] >= vertnum) {
        return "Invalid model: skin_face out of bounds.";
      }
    }
    for (int b=m->skin_boneadr[i]; b < m->skin_boneadr[i] + m->skin_bonenum[i]; b++) {
      int adr = m->skin_bonevertadr[b];
      for (int j=adr; j < adr + m->skin_bonevertnum[b]; j++) {
        if (m->skin_bonevertid[j] >= vertnum) {
          return "Invalid model: skin_bonevertid out of bounds.";
        }
      }
    }
  }

  return NULL;
}


// validate the indices of the flexes, which are local to the vertices and edges of each flex
static const char* validateFlexes(const mjModel* m) {
  for (int f=0; f < m->nflex; f++) {
    int dim = m->flex_dim[f];
    int vertnum = m->flex_vertnum[f];
    int edgenum = m->flex_edgenum[f];
    int elemnum = m->flex_elemnum[f];

    // the dimension defines the number of vertices and edges per element
    if (dim < 1 || dim > 3) {
      return "Invalid model: flex_dim out of bounds.";
    }
    if (m->flex_interp[f] < -2 || m->flex_interp[f] > 2) {
      return "Invalid model: flex_interp out of bounds.";
    }

    // vertices of the elements, and their texture coordinates
    mjtSize elemdataadr = m->flex_elemdataadr[f];
    if (!validRange(elemdataadr, (mjtSize)elemnum*(dim+1), m->nflexelemdata)) {
      return "Invalid model: flex_elemdataadr out of bounds.";
    }
    mjtSize texcoordadr = m->flex_texcoordadr[f];
    for (mjtSize j=elemdataadr; j < elemdataadr + (mjtSize)elemnum*(dim+1); j++) {
      if (m->flex_elem[j] >= vertnum) {
        return "Invalid model: flex_elem out of bounds.";
      }
      int texcoord = m->flex_elemtexcoord[j];
      if (texcoord < 0 || (texcoordadr >= 0 ? texcoordadr + texcoord >= m->nflextexcoord
                                            : texcoord >= vertnum)) {
        return "Invalid model: flex_elemtexcoord out of bounds.";
      }
    }

    // vertices of the edges, and of their flaps: -1 if there is none
    for (mjtSize e=m->flex_edgeadr[f]; e < (mjtSize)m->flex_edgeadr[f] + edgenum; e++) {
      for (int k=0; k < 2; k++) {
        if (m->flex_edge[2*e+k] >= vertnum) {
          return "Invalid model: flex_edge out of bounds.";
        }
        if (m->flex_edgeflap[2*e+k] < -1 || m->flex_edgeflap[2*e+k] >= vertnum) {
          return "Invalid model: flex_edgeflap out of bounds.";
        }
      }
    }

    // edges of the elements
    int nelemedge = (dim == 1) ? 1 : (dim == 2) ? 3 : 6;
    mjtSize elemedgeadr = m->flex_elemedgeadr[f];
    if (!validRange(elemedgeadr, (mjtSize)elemnum*nelemedge, m->nflexelemedge)) {
      return "Invalid model: flex_elemedgeadr out of bounds.";
    }
    for (mjtSize j=elemedgeadr; j < elemedgeadr + (mjtSize)elemnum*nelemedge; j++) {
      if (m->flex_elemedge[j] >= edgenum) {
        return "Invalid model: flex_elemedge out of bounds.";
      }
    }

    // vertices of the shell fragments
    int shellnum = m->flex_shellnum[f];
    mjtSize shelldataadr = m->flex_shelldataadr[f];
    if (shellnum < 0 ||
        (shellnum > 0 && !validRange(shelldataadr, (mjtSize)shellnum*dim, m->nflexshelldata))) {
      return "Invalid model: flex_shelldataadr out of bounds.";
    }
    for (mjtSize j=0; j < (mjtSize)shellnum*dim; j++) {
      if (m->flex_shell[shelldataadr+j] >= vertnum) {
        return "Invalid model: flex_shell out of bounds.";
      }
    }

    // bodies of the vertices: only the vertices of an interpolated flex have none
    if (!m->flex_interp[f]) {
      for (int v=m->flex_vertadr[f]; v < m->flex_vertadr[f] + vertnum; v++) {
        if (m->flex_vertbodyid[v] < 0) {
          return "Invalid model: flex_vertbodyid is negative.";
        }
      }
    }
  }

  return NULL;
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
  X(flex_matid,         nflexelem,      nmat          , 0                      ) \
  X(flex_texcoordadr,   nflex,          nflextexcoord , 0                      ) \
  X(flex_elemdataadr,   nflex,          nflexelemdata , 0                      ) \
  X(flex_elemedgeadr,   nflex,          nflexelemedge , 0                      ) \
  X(flex_shelldataadr,  nflex,          nflexshelldata, 0                      ) \
  X(flex_celladr,       nflex,          nflexcell     , 0                      ) \
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
  X(actuator_plugin,    nactuator,      nplugin       , 0                      ) \
  X(actuator_actadr,    nactuator,      na            , m->actuator_actnum     ) \
  X(actuator_ctrladr,   nactuator,      nu            , m->actuator_ctrlnum    ) \
  X(actuator_outadr,    nactuator,      nout          , m->actuator_outnum     ) \
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
  X(name_actuatoradr,   nactuator,      nnames        , 0                      ) \
  X(name_sensoradr,     nsensor,        nnames        , 0                      ) \
  X(name_numericadr,    nnumeric,       nnames        , 0                      ) \
  X(name_textadr,       ntext,          nnames        , 0                      ) \
  X(name_tupleadr,      ntuple,         nnames        , 0                      ) \
  X(name_keyadr,        nkey,           nnames        , 0                      ) \
  X(hfield_pathadr,     nhfield,        npaths        , 0                      ) \
  X(mesh_pathadr,       nmesh,          npaths        , 0                      ) \
  X(skin_pathadr,       nskin,          npaths        , 0                      ) \
  X(tex_pathadr,        ntex,           npaths        , 0                      ) \
  X(tex_contenttypeadr, ntex,           npaths        , 0                      ) \
  X(body_treeid,        nbody,          ntree         , 0                      ) \
  X(dof_treeid,         nv,             ntree         , 0                      ) \
  X(geom_plugin,        ngeom,          nplugin       , 0                      ) \
  X(light_texid,        nlight,         ntex          , 0                      ) \
  X(mat_texid,          nmat*mjNTEXROLE, ntex         , 0                      ) \
  X(flex_nodeadr,       nflex,          nflexnode     , m->flex_nodenum        ) \
  X(flex_nodebodyid,    nflexnode,      nbody         , 0                      ) \
  X(flex_vertbodyid,    nflexvert,      nbody         , 0                      ) \
  X(flex_stiffnessadr,  nflex,          nflexstiffness, 0                      ) \
  X(flex_bendingadr,    nflex,          nflexbending  , 0                      ) \
  X(mesh_octadr,        nmesh,          noct          , m->mesh_octnum         ) \
  X(name_flexadr,       nflex,          nnames        , 0                      ) \
  X(name_pluginadr,     nplugin,        nnames        , 0                      )

  #define X(adrarray, nadrs, ntarget, numarray) {             \
    int *nums = (numarray);                                   \
    for (mjtSize i=0; i<m->nadrs; i++) {                      \
      int adr = m->adrarray[i];                               \
      int num = (nums ? nums[i] : 1);                         \
      if (num < 0) {                                          \
        return "Invalid model: " #numarray " is negative.";   \
      }                                                       \
      /* -1: no reference, or an empty range */               \
      if (adr == -1 ? (nums && num > 0)                       \
                    : !validRange(adr, num, m->ntarget)) {    \
        return "Invalid model: " #adrarray " out of bounds."; \
      }                                                       \
    }                                                         \
  }

  MJMODEL_REFERENCES;
  #undef X
#undef MJMODEL_REFERENCES

  // references which always point to an element and are used as indices without further checks,
  // unlike the optional references above they cannot be -1
  #define X(array, n)                                        \
    for (mjtSize i=0; i < m->n; i++) {                       \
      if (m->array[i] < 0) {                                 \
        return "Invalid model: " #array " is negative.";     \
      }                                                      \
    }
  X(jnt_bodyid,        njnt)
  X(dof_jntid,         nv)
  X(dof_treeid,        nv)
  X(geom_bodyid,       ngeom)
  X(site_bodyid,       nsite)
  X(cam_bodyid,        ncam)
  X(light_bodyid,      nlight)
  X(flex_edge,         nflexedge*2)
  X(flex_elem,         nflexelemdata)
  X(flex_elemedge,     nflexelemedge)
  X(flex_shell,        nflexshelldata)
  X(flex_elemdataadr,  nflex)
  X(flex_elemedgeadr,  nflex)
  X(flex_nodebodyid,   nflexnode)
  X(skin_bonebodyid,   nskinbone)
  X(skin_bonevertid,   nskinbonevert)
  X(pair_geom1,        npair)
  X(pair_geom2,        npair)
  X(plugin_attradr,    nplugin)
  X(name_bodyadr,      nbody)
  X(name_jntadr,       njnt)
  X(name_geomadr,      ngeom)
  X(name_siteadr,      nsite)
  X(name_camadr,       ncam)
  X(name_lightadr,     nlight)
  X(name_flexadr,      nflex)
  X(name_meshadr,      nmesh)
  X(name_skinadr,      nskin)
  X(name_hfieldadr,    nhfield)
  X(name_texadr,       ntex)
  X(name_matadr,       nmat)
  X(name_pairadr,      npair)
  X(name_excludeadr,   nexclude)
  X(name_eqadr,        neq)
  X(name_tendonadr,    ntendon)
  X(name_actuatoradr,  nactuator)
  X(name_sensoradr,    nsensor)
  X(name_numericadr,   nnumeric)
  X(name_textadr,      ntext)
  X(name_tupleadr,     ntuple)
  X(name_keyadr,       nkey)
  X(name_pluginadr,    nplugin)
  #undef X

  #define VALIDATE_SPARSE(prefix, nrows, ncols, nnz, strict)           \
    switch (validateSparse(nrows, ncols, nnz,                          \
                           m->prefix##_rownnz, m->prefix##_rowadr,     \
                           m->prefix##_colind, strict)) {              \
    case 1: return "Invalid model: " #prefix "_rownnz out of bounds."; \
    case 2: return "Invalid model: " #prefix "_rowadr out of bounds."; \
    case 3: return "Invalid model: " #prefix "_colind out of bounds."; \
    }

  VALIDATE_SPARSE(M,          m->nv,          m->nv,       m->nC,     1);
  VALIDATE_SPARSE(D,          m->nv,          m->nv,       m->nD,     1);
  VALIDATE_SPARSE(B,          m->nbody,       m->nv,       m->nB,     1);
  VALIDATE_SPARSE(ten_J,      m->ntendon,     m->nv,       m->nJten,  0);
  VALIDATE_SPARSE(flexedge_J, m->nflexedge,   m->nv,       m->nJfe,   1);
  VALIDATE_SPARSE(efm0_L,     m->nefm0dof,    m->nefm0dof, m->nefm0L, 1);
  #undef VALIDATE_SPARSE

  // special logic that doesn't fit in the macro:
  for (int i=0; i < m->nbody; i++) {
    if ((i > 0 && m->body_parentid[i] >= i) || m->body_parentid[i] < 0) {
      return "Invalid model: bad body_parentid.";
    }
    if (m->body_rootid[i] > i || m->body_rootid[i] < 0) {
      return "Invalid model: bad body_rootid.";
    }
    if (m->body_weldid[i] > i || m->body_weldid[i] < 0) {
      return "Invalid model: bad body_weldid.";
    }

    // the last dof of a body is body_dofadr + body_dofnum - 1, computed without checking
    // body_dofnum: it is negative for a body without dofs only if its address is -1
    if (m->body_dofnum[i] == 0 && m->body_dofadr[i] != -1) {
      return "Invalid model: body_dofadr of a body without dofs is not -1.";
    }
    if (i == 0 && m->body_dofnum[i] != 0) {
      return "Invalid model: world body has dofs.";
    }
    if (i == 0) {
      if (m->B_rownnz[0] != m->nv || (m->nv > 0 && m->B_rowadr[0] != 0)) {
        return "Invalid model: B_rownnz out of bounds.";
      }
    } else {
      int np = m->body_parentid[i];
      int c = 0, cp = 0;
      while (c < m->B_rownnz[i] && cp < m->B_rownnz[np]) {
        int col = m->B_colind[m->B_rowadr[i]+c];
        int colp = m->B_colind[m->B_rowadr[np]+cp];
        if (col == colp) {
          c++;
          cp++;
        } else if (col > colp) {
          cp++;
        } else {
          break;
        }
      }
      if (c < m->B_rownnz[i]) {
        return "Invalid model: B_colind out of bounds.";
      }

      mjtSize ndof = 0;
      int np_weld = m->body_weldid[np];
      while (np_weld > 0) {
        ndof += m->body_dofnum[np_weld];
        np_weld = m->body_weldid[m->body_parentid[np_weld]];
      }
      if (ndof > m->B_rownnz[i] || ndof > m->B_rownnz[np]) {
        return "Invalid model: B_rownnz out of bounds.";
      }
    }
  }
  for (int i=0; i < m->njnt; i++) {
    if (m->jnt_type[i] >= 4 || m->jnt_type[i] < 0) {
      return "Invalid model: jnt_type out of bounds.";
    }
    if (!validRange(m->jnt_qposadr[i], nPOS[m->jnt_type[i]], m->nq)) {
      return "Invalid model: jnt_qposadr out of bounds.";
    }
    if (!validRange(m->jnt_dofadr[i], nVEL[m->jnt_type[i]], m->nv)) {
      return "Invalid model: jnt_dofadr out of bounds.";
    }
    if (m->jnt_plasticadr[i] != -1 &&
        !validRange(m->jnt_plasticadr[i], nPOS[m->jnt_type[i]], m->nplastic)) {
      return "Invalid model: jnt_plasticadr out of bounds.";
    }
  }
  for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_plasticadr[i] != -1 && !validRange(m->tendon_plasticadr[i], 2, m->nplastic)) {
      return "Invalid model: tendon_plasticadr out of bounds.";
    }
  }
  mjtSize nM_post = 0;
  for (int i=0; i < m->nv; i++) {
    if (m->dof_parentid[i] >= i) {
      return "Invalid model: bad dof_parentid.";
    }
    int bodyid = m->dof_bodyid[i];
    if (bodyid < 0 || bodyid >= m->nbody) {
      return "Invalid model: dof_bodyid out of bounds.";
    }
    if (m->dof_Madr[i] != nM_post) {
      return "Invalid model: dof_Madr out of bounds.";
    }
    int nanc = 0;
    for (int j=i; j >= 0; j = m->dof_parentid[j]) {
      nanc++;
    }
    nM_post += nanc;

    int expected_nnz = m->dof_simplenum[i] ? 1 : nanc;
    if (m->M_rownnz[i] != expected_nnz) {
      return "Invalid model: M_rownnz out of bounds.";
    }
    int adr = m->M_rowadr[i];
    int k = expected_nnz - 1;
    for (int j=i; k >= 0; j = m->dof_parentid[j], k--) {
      if (m->M_colind[adr+k] != j) {
        return "Invalid model: M_colind out of bounds.";
      }
    }

    if (m->D_diag[i] != nanc - 1 || m->D_diag[i] >= m->D_rownnz[i] ||
        m->D_colind[m->D_rowadr[i]+m->D_diag[i]] != i) {
      return "Invalid model: D_diag out of bounds.";
    }

    if (m->D_rownnz[i] != m->B_rownnz[bodyid]) {
      return "Invalid model: D_rownnz out of bounds.";
    }
    for (int c=0; c < m->D_rownnz[i]; c++) {
      if (m->D_colind[m->D_rowadr[i]+c] != m->B_colind[m->B_rowadr[bodyid]+c]) {
        return "Invalid model: D_colind out of bounds.";
      }
    }
  }
  if (nM_post != m->nM) {
    return "Invalid model: dof_Madr out of bounds.";
  }
  for (int i=0; i < m->nC; i++) {
    if (m->mapM2M[i] < 0 || m->mapM2M[i] >= m->nM) {
      return "Invalid model: mapM2M out of bounds.";
    }
    if (m->mapD2M[i] < 0 || m->mapD2M[i] >= m->nD) {
      return "Invalid model: mapD2M out of bounds.";
    }
  }
  for (int i=0; i < m->nD; i++) {
    if (m->mapM2D[i] < -1 || m->mapM2D[i] >= m->nC) {
      return "Invalid model: mapM2D out of bounds.";
    }
  }
  for (int i=0; i < m->nefm0dof; i++) {
    if (m->efm0_dofid[i] < 0 || m->efm0_dofid[i] >= m->nv) {
      return "Invalid model: efm0_dofid out of bounds.";
    }
  }
  for (int i=0; i < m->nefmCvert; i++) {
    if (m->efmC_perm[i] < -1 || m->efmC_perm[i] >= m->nv) {
      return "Invalid model: efmC_perm out of bounds.";
    }
  }
  for (int f=0; f < m->nflex; f++) {
    int condim = m->flex_condim[f];
    if (condim != 1 && condim != 3 && condim != 4 && condim != 6) {
      return "Invalid model: flex_condim out of bounds.";
    }
    // number of cells of an interpolated flex, computed without overflow
    const int* cellnum = m->flex_cellnum + 3*f;
    mjtSize ncellsize = boundedProduct(boundedProduct(cellnum[0], cellnum[1], INT_MAX),
                                       cellnum[2], INT_MAX);
    if (m->flex_interp[f] ? !validRange(m->flex_celladr[f], ncellsize, m->nflexcell)
                          : (m->flex_celladr[f] != -1)) {
      return "Invalid model: flex_celladr out of bounds.";
    }
    int ncell = m->flex_interp[f] ? (int)ncellsize : 0;
    for (int v=0; v < m->flex_vertnum[f]; v++) {
      int c = m->flex_vertcell[m->flex_vertadr[f] + v];
      if (m->flex_interp[f] ? (c < 0 || c >= ncell) : (c != -1)) {
        return "Invalid model: flex_vertcell out of bounds.";
      }
    }
    int dim = m->flex_dim[f];
    mjtSize nplastic = 0;
    if (m->flex_interp[f] > 0) {
      nplastic = 6 * (mjtSize)ncell;
    } else if (m->flex_interp[f] < 0) {
      // faces of the grid of cells, computed without overflow
      mjtSize nyz = boundedProduct(cellnum[1], cellnum[2], INT_MAX);
      mjtSize nxz = boundedProduct(cellnum[0], cellnum[2], INT_MAX);
      mjtSize nxy = boundedProduct(cellnum[0], cellnum[1], INT_MAX);
      if (nyz < 0 || nxz < 0 || nxy < 0) {
        return "Invalid model: flex_cellnum out of bounds.";
      }
      nplastic = 3 * 2 * (nyz + nxz + nxy);
    } else if (dim == 1) {
      if (m->flex_plasticadr[f] >= 0 && m->flex_bendingadr[f] >= 0) {
        mjtNum nhinge = m->flex_bending[m->flex_bendingadr[f]];
        if (!(nhinge >= 0 && nhinge < (1 << 28))) {
          return "Invalid model: rod hinge count out of bounds.";
        }
        nplastic = mj_rodStateSize(m, f);
      }
    } else {
      nplastic = (m->flex_plasticmode[f] & 2) ? (dim == 2 ? 3 : 6) * (mjtSize)m->flex_elemnum[f] : 0;
      if (m->flex_plasticmode[f] & 1) {
        nplastic += m->flex_edgenum[f];
      }
    }
    int mode = m->flex_plasticmode[f];
    if (mode < 0 || mode > 3 || (mode && m->flex_plasticadr[f] < 0) ||
        ((mode & 2) && dim < 2) || ((mode & 1) && (dim != 2 || m->flex_interp[f]))) {
      return "Invalid model: flex_plasticmode out of bounds.";
    }
    if (m->flex_plasticadr[f] != -1 &&
        !validRange(m->flex_plasticadr[f], nplastic, m->nplastic)) {
      return "Invalid model: flex_plasticadr out of bounds.";
    }
  }
  for (int i=0; i < m->ngeom; i++) {
    int condim = m->geom_condim[i];
    if (condim != 1 && condim != 3 && condim != 4 && condim != 6) {
      return "Invalid model: geom_condim out of bounds.";
    }

    // geom_type is used as an index into the table of collision functions
    if (m->geom_type[i] < 0 || m->geom_type[i] >= mjNGEOMTYPES) {
      return "Invalid model: geom_type out of bounds.";
    }

    // height field and mesh geoms always have data, an SDF geom can have a plugin instead
    if (m->geom_type[i] == mjGEOM_HFIELD) {
      if (m->geom_dataid[i] >= m->nhfield || m->geom_dataid[i] < 0) {
        return "Invalid model: geom_dataid out of bounds.";
      }
    } else if (m->geom_type[i] == mjGEOM_MESH) {
      if (m->geom_dataid[i] >= m->nmesh || m->geom_dataid[i] < 0) {
        return "Invalid model: geom_dataid out of bounds.";
      }
    } else if (m->geom_type[i] == mjGEOM_SDF) {
      if (m->geom_dataid[i] >= m->nmesh || m->geom_dataid[i] < -1) {
        return "Invalid model: geom_dataid out of bounds.";
      }
    }
  }
  for (int i=0; i < m->nsite; i++) {
    if (m->site_type[i] < 0 || m->site_type[i] >= mjNGEOMTYPES) {
      return "Invalid model: site_type out of bounds.";
    }
    if (m->site_type[i] == mjGEOM_MESH) {
      if (m->site_dataid[i] >= m->nmesh || m->site_dataid[i] < -1) {
        return "Invalid model: site_dataid out of bounds.";
      }
    }
  }
  for (int i=0; i < m->nhfield; i++) {
    if (m->hfield_nrow[i] < 2 || m->hfield_ncol[i] < 2) {
      return "Invalid model: hfield_nrow or hfield_ncol less than 2.";
    }
    mjtSize npix = boundedProduct(m->hfield_nrow[i], m->hfield_ncol[i], INT_MAX);
    if (npix < 0) {
      return "Invalid model: hfield_nrow * hfield_ncol overflows int.";
    }
    if (!validRange(m->hfield_adr[i], npix, m->nhfielddata)) {
      return "Invalid model: hfield_adr out of bounds.";
    }
  }
  for (int i=0; i < m->ntex; i++) {
    // by-reference texture: no payload, the file is at tex_pathadr
    if (m->tex_adr[i] == -1) {
      if (m->tex_pathadr[i] < 0) {
        return "Invalid model: by-reference texture has no path.";
      }
      continue;
    }

    // number of bytes, computed without overflow
    if (m->tex_nchannel[i] < 0 || m->tex_height[i] < 0 || m->tex_width[i] < 0) {
      return "Invalid model: negative texture size.";
    }
    mjtSize npixel = boundedProduct(m->tex_height[i], m->tex_width[i], m->ntexdata);
    mjtSize nbytes = boundedProduct(npixel, m->tex_nchannel[i], m->ntexdata);
    if (!validRange(m->tex_adr[i], nbytes, m->ntexdata)) {
      return "Invalid model: tex_adr out of bounds.";
    }
  }

  // mesh index arrays: mesh-local indices, read verbatim from MJB files
  for (int i=0; i < m->nmesh; i++) {
    int vertnum = m->mesh_vertnum[i];
    int polynum = m->mesh_polynum[i];
    int normalnum = m->mesh_normalnum[i];
    int texcoordnum = m->mesh_texcoordnum[i];
    int has_texcoord = (m->mesh_texcoordadr[i] >= 0);

    // face vertex, normal and texcoord indices (texcoords only used if present)
    int faceadr = m->mesh_faceadr[i];
    for (int f=0; f < m->mesh_facenum[i]; f++) {
      for (int k=0; k < 3; k++) {
        mjtSize j = 3*((mjtSize)faceadr + f) + k;
        int v = m->mesh_face[j];
        if (v < 0 || v >= vertnum) {
          return "Invalid model: mesh_face vertex index out of bounds.";
        }
        int n = m->mesh_facenormal[j];
        if (n < 0 || n >= normalnum) {
          return "Invalid model: mesh_facenormal normal index out of bounds.";
        }
        if (has_texcoord) {
          int t = m->mesh_facetexcoord[j];
          if (t < 0 || t >= texcoordnum) {
            return "Invalid model: mesh_facetexcoord texcoord index out of bounds.";
          }
        }
      }
    }

    // polygon vertex indices
    int polyadr = m->mesh_polyadr[i];
    for (int p=0; p < polynum; p++) {
      int pvadr = m->mesh_polyvertadr[polyadr+p];
      for (int j=0; j < m->mesh_polyvertnum[polyadr+p]; j++) {
        int v = m->mesh_polyvert[pvadr+j];
        if (v < 0 || v >= vertnum) {
          return "Invalid model: mesh_polyvert vertex index out of bounds.";
        }
      }
    }

    // vertex-to-polygon map indices
    int vertadr = m->mesh_vertadr[i];
    for (int v=0; v < vertnum; v++) {
      int pmadr = m->mesh_polymapadr[vertadr+v];
      for (int j=0; j < m->mesh_polymapnum[vertadr+v]; j++) {
        int p = m->mesh_polymap[pmadr+j];
        if (p < 0 || p >= polynum) {
          return "Invalid model: mesh_polymap polygon index out of bounds.";
        }
      }
    }
  }

  for (int i=0; i < m->npair; i++) {
    int dim = m->pair_dim[i];
    if (dim != 1 && dim != 3 && dim != 4 && dim != 6) {
      return "Invalid model: pair_dim out of bounds.";
    }
    int pair_body1 = (m->pair_signature[i] & 0xFFFF);
    if (pair_body1 >= m->nbody || pair_body1 < 0) {
      return "Invalid model: pair_body1 out of bounds.";
    }

    // unsigned shift: a signed >> sign-extends signatures whose high id is >= 0x8000
    int pair_body2 = (int)((unsigned int)m->pair_signature[i] >> 16);
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
    case mjEQ_FLEXSTRAIN:
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
      return "Invalid model: unknown equality constraint type.";
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
  for (int i=0; i < m->nactuator; i++) {
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
      // -1 is the value used if the reference site is omitted
      if (idslider < -1 || idslider >= m->nsite) {
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
    case mjTRN_SO3:
      // ball joint target (idslider == -1) or site + refsite target
      if (idslider == -1) {
        if (id < 0 || id >= m->njnt) {
          return "Invalid model: actuator_trnid out of bounds.";
        }
      } else {
        if (id < 0 || id >= m->nsite || idslider < 0 || idslider >= m->nsite) {
          return "Invalid model: actuator_trnid out of bounds.";
        }
      }
      break;
    case mjTRN_UNDEFINED:
      // actuator_trnid not used.
      break;
    }

    const char* historyerr = validateHistory(m->actuator_history[2*i], m->actuator_ctrlnum[i],
                                             m->actuator_historyadr[i], m->nhistory);
    if (historyerr) {
      return historyerr;
    }
  }

  // actuator_moment is filled by mj_transmission without bounds checks
  mjtSize nJmom = momentSize(m);
  if (nJmom < 0) {
    return "Invalid model: bad actuator transmission or output address.";
  }
  if (m->nJmom < nJmom) {
    return "Invalid model: nJmom is too small.";
  }

  // plugin_state is filled by the plugins without bounds checks: a plugin writes the number
  // of states that it reports, which must be the number that the model has for it
  for (int i=0; i < m->nplugin; i++) {
    const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[i]);
    if (!plugin) {
      return "Invalid model: plugin is not registered.";
    }
    if (!plugin->nstate || plugin->nstate(m, i) != m->plugin_statenum[i]) {
      return "Invalid model: plugin_statenum does not match the states of the plugin.";
    }
  }

  // npolygonmax and nmeshdegmax are used to allocate the buffers of the convex collider
  for (int i=0; i < m->nmeshpoly; i++) {
    if (m->mesh_polyvertnum[i] > m->npolygonmax) {
      return "Invalid model: npolygonmax is too small.";
    }
  }
  for (int i=0; i < m->nmeshvert; i++) {
    if (m->mesh_polymapnum[i] > m->nmeshdegmax) {
      return "Invalid model: nmeshdegmax is too small.";
    }
  }
  for (int i=0; i < m->nsensor; i++) {
    mjtSensor sensor_type = m->sensor_type[i];

    // the objects of the sensor, its size can depend on them
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

    // the ids are used according to the type of the sensor
    if (!validSensorObjects(m, i)) {
      return "Invalid model: sensor_objtype or sensor_reftype does not match sensor_type.";
    }

    // number of values written by the sensor
    mjtSize sensor_size;
    if (sensor_type == mjSENS_PLUGIN) {
      if (m->sensor_plugin[i] < 0 || m->sensor_plugin[i] >= m->nplugin) {
        return "Invalid model: sensor_plugin out of bounds.";
      }
      const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[m->sensor_plugin[i]]);
      if (!plugin) {
        return "Invalid model: sensor plugin is not registered.";
      }
      if (!(plugin->capabilityflags & mjPLUGIN_SENSOR) || !plugin->nsensordata) {
        return "Invalid model: plugin of a sensor is not a sensor plugin.";
      }
      sensor_size = plugin->nsensordata(m, m->sensor_plugin[i], i);
    } else {
      sensor_size = sensorSize(m, i);
    }
    if (sensor_size < 0) {
      return "Invalid model: Bad sensor_type.";
    }

    // sensor_dim is the size of the sensor in sensordata and in the history buffer
    int sensor_dim = m->sensor_dim[i];
    if (sensor_dim != sensor_size) {
      return "Invalid model: sensor_dim does not match the size of the sensor.";
    }
    int sensor_adr = m->sensor_adr[i];
    if (!validRange(sensor_adr, sensor_size, m->nsensordata)) {
      return "Invalid model: sensor_adr out of bounds.";
    }
    const char* historyerr = validateHistory(m->sensor_history[2*i], sensor_dim,
                                             m->sensor_historyadr[i], m->nhistory);
    if (historyerr) {
      return historyerr;
    }
    if (sensor_type == mjSENS_TACTILE) {
      int parent_body = m->geom_bodyid[m->sensor_refid[i]];
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
    // unsigned shift: a signed >> sign-extends signatures whose high id is >= 0x8000
    int exclude_body2 = (int)((unsigned int)m->exclude_signature[i] >> 16);
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

  // consistency between fields, in functions which assume that all of the above is valid
  const char* (*validators[])(const mjModel*) = {
    validateTreeLayout, validateStrings, validatePlugins, validateTendons, validateActuators,
    validateBvh, validateMeshes, validateSkins, validateFlexes
  };
  for (int i=0; i < sizeof(validators)/sizeof(validators[0]); i++) {
    const char* err = validators[i](m);
    if (err) {
      return err;
    }
  }

  return NULL;
}
