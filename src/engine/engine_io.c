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

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mujoco/mjmodel.h>
#include <mujoco/mjxmacro.h>
#include "engine/engine_macro.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_vfs.h"

#ifdef ADDRESS_SANITIZER
  #include <sanitizer/asan_interface.h>
#elif defined(_MSC_VER)
  #define ASAN_POISON_MEMORY_REGION(addr, size)
  #define ASAN_UNPOISON_MEMORY_REGION(addr, size)
#else
  #define ASAN_POISON_MEMORY_REGION(addr, size) ((void)(addr), (void)(size))
  #define ASAN_UNPOISON_MEMORY_REGION(addr, size) ((void)(addr), (void)(size))
#endif

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

#ifdef _MSC_VER
  #pragma warning (disable: 4305)  // disable MSVC warning: truncation from 'double' to 'float'
#endif

#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

#define PTRDIFF(x, y) ((void*)(x) - (void*)(y))


//------------------------------ mjLROpt -----------------------------------------------------------

// set default options for length range computation
void mj_defaultLROpt(mjLROpt* opt) {
  opt->mode           = mjLRMODE_MUSCLE;
  opt->useexisting    = 1;
  opt->uselimit       = 0;

  opt->accel          = 20;
  opt->maxforce       = 0;
  opt->timeconst      = 1;
  opt->timestep       = 0.01;
  opt->inttotal       = 10;
  opt->inteval        = 2;
  opt->tolrange       = 0.05;
}



//------------------------------- mjOption ---------------------------------------------------------

// set default solver paramters
void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp) {
  if (solref) {
    solref[0] = 0.02;       // timeconst
    solref[1] = 1;          // dampratio
  }

  if (solimp) {
    solimp[0] = 0.9;        // dmin
    solimp[1] = 0.95;       // dmax
    solimp[2] = 0.001;      // width
    solimp[3] = 0.5;        // midpoint
    solimp[4] = 2;          // power
  }
}



// set model options to default values
void mj_defaultOption(mjOption* opt) {
  // timing parameters
  opt->timestep           = 0.002;
  opt->apirate            = 100;

  // solver parameters
  opt->impratio           = 1;
  opt->tolerance          = 1e-8;
  opt->noslip_tolerance   = 1e-6;
  opt->mpr_tolerance      = 1e-6;

  // physical constants
  opt->gravity[0]         = 0;
  opt->gravity[1]         = 0;
  opt->gravity[2]         = -9.81;
  opt->wind[0]            = 0;
  opt->wind[1]            = 0;
  opt->wind[2]            = 0;
  opt->magnetic[0]        = 0;
  opt->magnetic[1]        = -0.5;
  opt->magnetic[2]        = 0;
  opt->density            = 0;
  opt->viscosity          = 0;

  // solver overrides
  opt->o_margin           = 0;
  mj_defaultSolRefImp(opt->o_solref, opt->o_solimp);

  // discrete options
  opt->integrator         = mjINT_EULER;
  opt->collision          = mjCOL_ALL;
  opt->cone               = mjCONE_PYRAMIDAL;
  opt->jacobian           = mjJAC_AUTO;
  opt->solver             = mjSOL_NEWTON;
  opt->iterations         = 100;
  opt->noslip_iterations  = 0;
  opt->mpr_iterations     = 50;
  opt->disableflags       = 0;
  opt->enableflags        = 0;
}



//------------------------------- mjVisual ---------------------------------------------------------

// set 4 floats
static void setf4(float* rgba, float r, float g, float b, float a) {
  rgba[0] = r;
  rgba[1] = g;
  rgba[2] = b;
  rgba[3] = a;
}


// set visual options to default values
void mj_defaultVisual(mjVisual* vis) {
  // global
  vis->global.fovy                = 45;
  vis->global.ipd                 = 0.068;
  vis->global.azimuth             = 90;
  vis->global.elevation           = -45;
  vis->global.linewidth           = 1.0;
  vis->global.glow                = 0.3;
  vis->global.offwidth            = 640;
  vis->global.offheight           = 480;

  // rendering quality
  vis->quality.shadowsize         = 4096;
  vis->quality.offsamples         = 4;
  vis->quality.numslices          = 28;
  vis->quality.numstacks          = 16;
  vis->quality.numquads           = 4;

  // head light
  vis->headlight.ambient[0]       = 0.1;
  vis->headlight.ambient[1]       = 0.1;
  vis->headlight.ambient[2]       = 0.1;
  vis->headlight.diffuse[0]       = 0.4;
  vis->headlight.diffuse[1]       = 0.4;
  vis->headlight.diffuse[2]       = 0.4;
  vis->headlight.specular[0]      = 0.5;
  vis->headlight.specular[1]      = 0.5;
  vis->headlight.specular[2]      = 0.5;
  vis->headlight.active           = 1;

  // map parameters
  vis->map.stiffness              = 100;
  vis->map.stiffnessrot           = 500;
  vis->map.force                  = 0.005;
  vis->map.torque                 = 0.1;
  vis->map.alpha                  = 0.3;
  vis->map.fogstart               = 3.0;
  vis->map.fogend                 = 10.0;
  vis->map.znear                  = 0.01;
  vis->map.zfar                   = 50.0;
  vis->map.haze                   = 0.3;
  vis->map.shadowclip             = 1.0;
  vis->map.shadowscale            = 0.6;
  vis->map.actuatortendon         = 2.0;

  // size parameters
  vis->scale.forcewidth           = 0.1;
  vis->scale.contactwidth         = 0.3;
  vis->scale.contactheight        = 0.1;
  vis->scale.connect              = 0.2;
  vis->scale.com                  = 0.4;
  vis->scale.camera               = 0.3;
  vis->scale.light                = 0.3;
  vis->scale.selectpoint          = 0.2;
  vis->scale.jointlength          = 1.0;
  vis->scale.jointwidth           = 0.1;
  vis->scale.actuatorlength       = 0.7;
  vis->scale.actuatorwidth        = 0.2;
  vis->scale.framelength          = 1.0;
  vis->scale.framewidth           = 0.1;
  vis->scale.constraint           = 0.1;
  vis->scale.slidercrank          = 0.2;

  // colors
  setf4(vis->rgba.fog,              0., 0., 0., 1.);
  setf4(vis->rgba.haze,             1., 1., 1., 1.);
  setf4(vis->rgba.force,            1., .5, .5, 1.);
  setf4(vis->rgba.inertia,          .8, .2, .2, .6);
  setf4(vis->rgba.joint,            .2, .6, .8, 1.);
  setf4(vis->rgba.actuator,         .2, .25, .2, 1);
  setf4(vis->rgba.actuatornegative, .2, .6, .9, 1.);
  setf4(vis->rgba.actuatorpositive, .9, .4, .2, 1.);
  setf4(vis->rgba.com,              .9, .9, .9, 1.);
  setf4(vis->rgba.camera,           .6, .9, .6, 1.);
  setf4(vis->rgba.light,            .6, .6, .9, 1.);
  setf4(vis->rgba.selectpoint,      .9, .9, .1, 1.);
  setf4(vis->rgba.connect,          .2, .2, .8, 1.);
  setf4(vis->rgba.contactpoint,     .9, .6, .2, 1.);
  setf4(vis->rgba.contactforce,     .7, .9, .9, 1.);
  setf4(vis->rgba.contactfriction,  .9, .8, .4, 1.);
  setf4(vis->rgba.contacttorque,    .9, .7, .9, 1.);
  setf4(vis->rgba.contactgap,       .5, .8, .9, 1.);
  setf4(vis->rgba.rangefinder,      1., 1., .1, 1.);
  setf4(vis->rgba.constraint,       .9, .0, .0, 1.);
  setf4(vis->rgba.slidercrank,      .5, .3, .8, 1.);
  setf4(vis->rgba.crankbroken,      .9, .0, .0, 1.);
}



//------------------------------- mjStatistic ------------------------------------------------------

// set statistics to default values; compute later in compiler
void mj_defaultStatistic(mjStatistic* stat) {
  mju_zero3(stat->center);
  stat->extent = 2;
  stat->meaninertia = 1;
  stat->meanmass = 1;
  stat->meansize = 0.2;
}



//----------------------------------- static utility functions -------------------------------------

// id used to indentify binary mjModel file/buffer
static const int ID = 54321;


// number of bytes to be skipped to achieve 64-byte alignment
static unsigned int SKIP(intptr_t offset) {
  const unsigned int align = 64;
  // compute skipped bytes
  return (align - (offset % align)) % align;
}



// count ints in mjModel
static int getnint(void) {
  int cnt = 0;

#define X(name) cnt++;
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
    mju_error("NULL pointer passed to bufwrite");
  }

  // check size
  if (*ptrbuf+num>szbuf) {
    mju_error("Attempting to write outside model buffer");
  }

  // write, advance pointer
  memcpy((char*)buf + *ptrbuf, src, num);
  *ptrbuf += num;
}



// read from memory buffer
static void bufread(void* dest, int num, int szbuf, const void* buf, int* ptrbuf) {
  // check pointers
  if (!dest || !buf || !ptrbuf) {
    mju_error("NULL pointer passed to bufread");
  }

  // check size
  if (*ptrbuf+num>szbuf) {
    mju_error("Attempting to read outside model buffer");
  }

  // read, advance pointer
  memcpy(dest, (char*)buf + *ptrbuf, num);
  *ptrbuf += num;
}



//----------------------------------- mjModel construction -----------------------------------------

// set pointers in mjModel buffer
static void mj_setPtrModel(mjModel* m) {
  char* ptr = (char*)m->buffer;
  int sz;

  // prepare sybmols needed by xmacro
  MJMODEL_POINTERS_PREAMBLE(m);

  // assign pointers with padding
#define X(type, name, nr, nc)                             \
  m->name = (type*)(ptr + SKIP((intptr_t)ptr));           \
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF(m->name, ptr));  \
  ptr += SKIP((intptr_t)ptr) + sizeof(type)*(m->nr)*(nc);

  MJMODEL_POINTERS
#undef X

  // check size
  sz = (int)(ptr - (char*)m->buffer);
  if (m->nbuffer != sz) {
    printf("expected size: %d,  actual size: %d\n", m->nbuffer, sz);
    mju_error("mjModel buffer size mismatch");
  }
}


// increases buffer size without causing integer overflow, returns 0 if
// operation would cause overflow
// performs the following operations:
// *nbuffer += SKIP(*offset) + type_size*nr*nc;
// *offset += SKIP(*offset) + type_size*nr*nc;
static int safeAddToBufferSize(intptr_t* offset, int* nbuffer, size_t type_size, int nr, int nc) {
  if (type_size < 0 || nr < 0 || nc < 0) {
    return 0;
  }
#if (__has_builtin(__builtin_add_overflow) && __has_builtin(__builtin_mul_overflow)) \
    || (defined(__GNUC__) && __GNUC__ >= 5)
  // supported by GCC and Clang
  int to_add = 0;
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



// allocate and initialize mjModel structure
mjModel* mj_makeModel(int nq, int nv, int nu, int na, int nbody, int njnt,
                      int ngeom, int nsite, int ncam, int nlight,
                      int nmesh, int nmeshvert, int nmeshtexvert, int nmeshface, int nmeshgraph,
                      int nskin, int nskinvert, int nskintexvert, int nskinface,
                      int nskinbone, int nskinbonevert, int nhfield, int nhfielddata,
                      int ntex, int ntexdata, int nmat, int npair, int nexclude,
                      int neq, int ntendon, int nwrap, int nsensor,
                      int nnumeric, int nnumericdata, int ntext, int ntextdata,
                      int ntuple, int ntupledata, int nkey, int nmocap,
                      int nuser_body, int nuser_jnt, int nuser_geom, int nuser_site, int nuser_cam,
                      int nuser_tendon, int nuser_actuator, int nuser_sensor, int nnames) {
  intptr_t offset = 0;

  // allocate mjModel
  mjModel* m = (mjModel*)mju_malloc(sizeof(mjModel));
  if (!m) {
    mju_error("Could not allocate mjModel");
  }
  memset(m, 0, sizeof(mjModel));

  // set size parameters
  m->nq = nq;
  m->nv = nv;
  m->nu = nu;
  m->na = na;
  m->nbody = nbody;
  m->njnt = njnt;
  m->ngeom = ngeom;
  m->nsite = nsite;
  m->ncam = ncam;
  m->nlight = nlight;
  m->nmesh = nmesh;
  m->nmeshvert = nmeshvert;
  m->nmeshtexvert = nmeshtexvert;
  m->nmeshface = nmeshface;
  m->nmeshgraph = nmeshgraph;
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
  m->nuser_body = nuser_body;
  m->nuser_jnt = nuser_jnt;
  m->nuser_geom = nuser_geom;
  m->nuser_site = nuser_site;
  m->nuser_cam = nuser_cam;
  m->nuser_tendon = nuser_tendon;
  m->nuser_actuator = nuser_actuator;
  m->nuser_sensor = nuser_sensor;
  m->nnames = nnames;

#define X(name)                                    \
  if ((m->name) < 0) {                             \
    mju_free(m);                                   \
    mju_warning("Invalid model: negative " #name); \
    return 0;                                      \
  }
  MJMODEL_INTS;
#undef X

  // nbody should always be positive
  if (m->nbody == 0) {
    mju_free(m);
    mju_warning("Invalid model: nbody == 0");
    return 0;
  }

  // nmocap is going to get multiplied by 4, and shouldn't overflow
  if (m->nmocap >= INT_MAX / 4) {
    mju_free(m);
    mju_warning("Invalid model: nmocap too large");
    return 0;
  }

  // compute buffer size
  m->nbuffer = 0;
#define X(type, name, nr, nc)                                                \
  if (!safeAddToBufferSize(&offset, &m->nbuffer, sizeof(type), m->nr, nc)) { \
    mju_free(m);                                                             \
    mju_warning("Invalid model: " #name " too large.");                      \
    return 0;                                                                \
  }

  MJMODEL_POINTERS
#undef X

  // allocate buffer
  m->buffer = mju_malloc(m->nbuffer);
  if (!m->buffer) {
    mju_free(m);
    mju_error("Could not allocate mjModel buffer");
  }

  // clear, set pointers in buffer
  memset(m->buffer, 0, m->nbuffer);
#ifdef MEMORY_SANITIZER
  // Tell msan to treat the entire buffer as uninitialized
  __msan_allocated_memory(m->buffer, m->nbuffer);
#endif
  mj_setPtrModel(m);

  // set default options
  mj_defaultOption(&m->opt);
  mj_defaultVisual(&m->vis);
  mj_defaultStatistic(&m->stat);

  return m;
}



// copy mjModel, if dest==NULL create new model
mjModel* mj_copyModel(mjModel* dest, const mjModel* src) {
  void* save_bufptr;

  // allocate new model if needed
  if (!dest) {
    dest = mj_makeModel(src->nq, src->nv, src->nu, src->na, src->nbody, src->njnt,
                        src->ngeom, src->nsite, src->ncam, src->nlight, src->nmesh, src->nmeshvert,
                        src->nmeshtexvert, src->nmeshface, src->nmeshgraph,
                        src->nskin, src->nskinvert, src->nskintexvert, src->nskinface,
                        src->nskinbone, src->nskinbonevert, src->nhfield, src->nhfielddata,
                        src->ntex, src->ntexdata, src->nmat, src->npair, src->nexclude,
                        src->neq, src->ntendon, src->nwrap, src->nsensor,
                        src->nnumeric, src->nnumericdata, src->ntext, src->ntextdata,
                        src->ntuple, src->ntupledata, src->nkey, src->nmocap,
                        src->nuser_body, src->nuser_jnt, src->nuser_geom, src->nuser_site,
                        src->nuser_cam, src->nuser_tendon, src->nuser_actuator, src->nuser_sensor,
                        src->nnames);
  }
  if (!dest) {
    mju_error("Failed to make mjModel. Invalid sizes.");
  }

  // check sizes
  if (dest->nbuffer != src->nbuffer) {
    mj_deleteModel(dest);
    mju_error("dest and src models have different buffer size");
  }

  // save buffer ptr, copy everything, restore buffer and other pointers
  save_bufptr = dest->buffer;
  *dest = *src;
  dest->buffer = save_bufptr;
  mj_setPtrModel(dest);

  // copy buffer
  {
    MJMODEL_POINTERS_PREAMBLE(src)
    #define X(type, name, nr, nc)  \
      memcpy((char*)dest->name, (const char*)src->name, sizeof(type)*(src->nr)*nc);
    MJMODEL_POINTERS
    #undef X
  }

  return dest;
}



// save model to binary file, or memory buffer of szbuf>0
void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz) {
  FILE* fp = 0;
  int ptrbuf = 0;

  // standard header
  int header[4] = {ID, sizeof(mjtNum), getnint(), getnptr()};

  // open file for writing if no buffer
  if (!buffer) {
    fp = fopen(filename, "wb");
    if (!fp) {
      mju_warning_s("Could not open file '%s'", filename);
      return;
    }
  }

  // write standard header, info, options, buffer (omit pointers)
  if (fp) {
    fwrite(header, sizeof(int), 4, fp);
    fwrite(m, sizeof(int), getnint(), fp);
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
    bufwrite(header, sizeof(int)*4, buffer_sz, buffer, &ptrbuf);
    bufwrite(m, sizeof(int)*getnint(), buffer_sz, buffer, &ptrbuf);
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



// load model from binary MJB file
//  if vfs is not NULL, look up file in vfs before reading from disk
mjModel* mj_loadModel(const char* filename, const mjVFS* vfs) {
  int i, header[4] = {0};
  int expected_header[4] = {ID, sizeof(mjtNum), getnint(), getnptr()};
  int info[2000];
  int ptrbuf = 0;
  mjModel *m = 0;
  FILE* fp = 0;

  // find file in VFS if given
  const void* buffer = NULL;
  int buffer_sz = 0;
  if (vfs) {
    i = mj_findFileVFS(vfs, filename);
    if (i>=0) {
      buffer_sz = vfs->filesize[i];
      buffer = vfs->filedata[i];
    }
  }

  // open file for reading if no buffer
  if (!buffer) {
    fp = fopen(filename, "rb");
    if (!fp) {
      mju_warning_s("Could not open file '%s'", filename);
      return 0;
    }
  }

  // read header
  if (fp) {
    if (fread(header, 4, sizeof(int), fp) != 4) {
      mju_warning("Model file has an incomplete header");
      return 0;
    }
  } else {
    bufread(header, 4*sizeof(int), buffer_sz, buffer, &ptrbuf);
  }

  // check header
  for (i=0; i<4; i++) {
    if (header[i]!=expected_header[i]) {
      if (fp) {
        fclose(fp);
      }

      switch (i) {
      case 0:
        mju_warning("Model missing header ID");
        return 0;

      case 1:
        mju_warning("Model and executable have different floating point precision");
        return 0;

      case 2:
        mju_warning("Model and executable have different number of ints in mjModel");
        return 0;

      default:
        mju_warning("Model and executable have different number of pointers in mjModel");
        return 0;
      }
    }
  }

  // read mjModel structure: info only
  if (fp) {
    if (fread(info, sizeof(int), getnint(), fp) != getnint()) {
      mju_warning("Model file does not contain enough ints");
      return 0;
    }
  } else {
    bufread(info, sizeof(int)*getnint(), buffer_sz, buffer, &ptrbuf);
  }

  // allocate new mjModel, check sizes
  m = mj_makeModel(info[0],  info[1],  info[2],  info[3],  info[4],  info[5],  info[6],
                   info[7],  info[8],  info[9],  info[10], info[11], info[12], info[13],
                   info[14], info[15], info[16], info[17], info[18], info[19], info[20],
                   info[21], info[22], info[23], info[24], info[25], info[26], info[27],
                   info[28], info[29], info[30], info[31], info[32], info[33], info[34],
                   info[35], info[36], info[37], info[38], info[39], info[40], info[41],
                   info[42], info[43], info[44], info[45], info[46], info[47], info[48]);
  if (!m || m->nbuffer!=info[getnint()-1]) {
    if (fp) {
      fclose(fp);
    }
    mju_warning("Corrupted model, wrong size parameters");
    mj_deleteModel(m);
    return 0;
  }

  // set info fields
  memcpy(m, info, sizeof(int)*getnint());

  // read options and buffer
  if (fp) {
    if (fread((void*)&m->opt, sizeof(mjOption), 1, fp) != 1) {
      mju_warning("Model file does not have a complete mjOption");
      mj_deleteModel(m);
      return 0;
    }
    if (fread((void*)&m->vis, sizeof(mjVisual), 1, fp) != 1) {
      mju_warning("Model file does not have a complete mjVisual");
      mj_deleteModel(m);
      return 0;
    }
    if (fread((void*)&m->stat, sizeof(mjStatistic), 1, fp) != 1) {
      mju_warning("Model file does not have a complete mjStatistic");
      mj_deleteModel(m);
      return 0;
    }
    {
      MJMODEL_POINTERS_PREAMBLE(m)
      #define X(type, name, nr, nc)                                            \
        if (fread(m->name, sizeof(type), (m->nr)*(nc), fp) != (m->nr)*(nc)) {  \
          mju_warning("Model file does not contain a large enough buffer");    \
          mj_deleteModel(m);                                                   \
          return 0;                                                            \
        }
      MJMODEL_POINTERS
      #undef X
    }
  } else {
    bufread((void*)&m->opt, sizeof(mjOption), buffer_sz, buffer, &ptrbuf);
    bufread((void*)&m->vis, sizeof(mjVisual), buffer_sz, buffer, &ptrbuf);
    bufread((void*)&m->stat, sizeof(mjStatistic), buffer_sz, buffer, &ptrbuf);
    {
      MJMODEL_POINTERS_PREAMBLE(m)
      #define X(type, name, nr, nc)  \
        bufread(m->name, sizeof(type)*(m->nr)*(nc), buffer_sz, buffer, &ptrbuf);
      MJMODEL_POINTERS
      #undef X
    }
  }

  // make sure file size is correct
  if (fp) {
    if (feof(fp)) {
      fclose(fp);
      mju_warning("Model file is too small");
      mj_deleteModel(m);
      return 0;
    }
    char dummy;
    if (fread(&dummy, 1, 1, fp) || !feof(fp)) {
      fclose(fp);
      mju_warning("Model file is too large");
      mj_deleteModel(m);
      return 0;
    }
  }

  const char* validationError = mj_validateReferences(m);
  if (validationError) {
    mju_warning(validationError);
    mj_deleteModel(m);
    return 0;
  }

  if (fp) {
    fclose(fp);
  }
  return m;
}



// de-allocate mjModel
void mj_deleteModel(mjModel* m) {
  if (m) {
    mju_free(m->buffer);
    mju_free(m);
  }
}



// size of buffer needed to hold model
int mj_sizeModel(const mjModel* m) {
  return sizeof(int)*(4+getnint()) + sizeof(mjOption) +
         sizeof(mjVisual) + sizeof(mjStatistic) + m->nbuffer;
}



//----------------------------------- mjData construction ------------------------------------------

// set pointers into mjData buffer
static void mj_setPtrData(const mjModel* m, mjData* d) {
  char* ptr = (char*)d->buffer;
  int sz;

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
  sz = (int)(ptr - (char*)d->buffer);
  if (d->nbuffer != sz) {
    mju_error("mjData buffer size mismatch");
  }
}



// allocate and initialize mjData structure
static mjData* _makeData(const mjModel* m) {
  intptr_t offset = 0;

  // allocate mjData
  mjData* d = (mjData*) mju_malloc(sizeof(mjData));
  if (!d) {
    mju_error("Could not allocate mjData");
  }

  // prepare symbols needed by xmacro
  MJDATA_POINTERS_PREAMBLE(m);

  // compute buffer size
  d->nbuffer = 0;
  d->buffer = d->stack = NULL;
#define X(type, name, nr, nc)                                                \
  if (!safeAddToBufferSize(&offset, &d->nbuffer, sizeof(type), m->nr, nc)) { \
    mju_free(d);                                                             \
    mju_warning("Invalid data: " #name " too large.");                       \
    return 0;                                                                \
  }

  MJDATA_POINTERS
#undef X

  // copy stack size from model
  d->nstack = m->nstack;

  // allocate buffer
  d->buffer = mju_malloc(d->nbuffer);
  if (!d->buffer) {
    mju_free(d);
    mju_error("Could not allocate mjData buffer");
  }

  // allocate stack
  d->stack = (mjtNum*) mju_malloc(d->nstack * sizeof(mjtNum));
  if (!d->stack) {
    mju_free(d->buffer);
    mju_free(d);
    mju_error("Could not allocate mjData stack");
  }

  // set pointers into buffer, reset data
  mj_setPtrData(m, d);

  return d;
}

mjData* mj_makeData(const mjModel* m) {
  mjData* d = _makeData(m);
  if (d) {
    mj_resetData(m, d);
  }
  return d;
}


// copy mjData, if dest==NULL create new data
mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src) {
  void* save_buffer;
  mjtNum* save_stack;

  // allocate new data if needed
  if (!dest) {
    dest = _makeData(m);
  }

  // check sizes
  if (dest->nbuffer != src->nbuffer) {
    mju_error("dest and src data buffers have different size");
  }
  if (dest->nstack != src->nstack) {
    mju_error("dest and src stacks have different size");
  }

  // stack is in use
  if (src->pstack) {
    mju_error("Attempting to copy mjData while stack is in use");
  }

  // save pointers, copy everything, restore pointers
  save_buffer = dest->buffer;
  save_stack = dest->stack;
  *dest = *src;
  dest->buffer = save_buffer;
  dest->stack = save_stack;
  mj_setPtrData(m, dest);

  // copy buffer
  {
    MJDATA_POINTERS_PREAMBLE(m)
    #define X(type, name, nr, nc)  \
      memcpy((char*)dest->name, (const char*)src->name, sizeof(type)*(m->nr)*nc);
    MJDATA_POINTERS
    #undef X
  }

  return dest;
}



// allocate size mjtNums on the mjData stack
mjtNum* mj_stackAlloc(mjData* d, int size) {
  mjtNum* result;

  // return NULL if empty
  if (!size) {
    return 0;
  }

  // check size
  if (d->pstack + size > d->nstack) {
    mju_error("Stack overflow");
  }

  // allocate, update max, return pointer to buffer
  result = (mjtNum*)d->stack + d->pstack;
  d->pstack += size;
  d->maxuse_stack = mjMAX(d->maxuse_stack, d->pstack);
  return result;
}



// clear data, set defaults
static void _resetData(const mjModel* m, mjData* d, unsigned char debug_value) {
  //------------------------------ clear header

  // clear stack pointer
  d->pstack = 0;

  // clear memory utilization stats
  d->maxuse_stack = 0;
  d->maxuse_con = 0;
  d->maxuse_efc = 0;

  // clear diagnostics
  memset(d->warning, 0, mjNWARNING*sizeof(mjWarningStat));
  memset(d->timer, 0, mjNTIMER*sizeof(mjTimerStat));
  memset(d->solver, 0, mjNSOLVER*sizeof(mjSolverStat));
  d->solver_iter = 0;
  d->solver_nnz = 0;
  mju_zero(d->solver_fwdinv, 2);

  // clear variable sizes
  d->ne = 0;
  d->nf = 0;
  d->nefc = 0;
  d->ncon = 0;

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
  // Tell msan to treat the entire buffer as uninitialized
  __msan_allocated_memory(d->buffer, d->nbuffer);
#endif

  // zero out arrays that are not affected by mj_forward
  mju_zero(d->qpos, m->nq);
  mju_zero(d->qvel, m->nv);
  mju_zero(d->act, m->na);
  mju_zero(d->ctrl, m->nu);
  mju_zero(d->qfrc_applied, m->nv);
  mju_zero(d->xfrc_applied, 6*m->nbody);
  mju_zero(d->qacc, m->nv);
  mju_zero(d->qacc_warmstart, m->nv);
  mju_zero(d->act_dot, m->na);
  mju_zero(d->userdata, m->nuserdata);
  mju_zero(d->sensordata, m->nsensordata);
  mju_zero(d->mocap_pos, 3*m->nmocap);
  mju_zero(d->mocap_quat, 4*m->nmocap);

  // copy qpos0 from model
  if (m->qpos0) {
    memcpy(d->qpos, m->qpos0, m->nq*sizeof(mjtNum));
  }

  // set mocap_pos/quat = body_pos/quat for mocap bodies
  if (m->body_mocapid) {
    for (int i=0; i<m->nbody; i++) {
      int id = m->body_mocapid[i];
      if (id>=0) {
        mju_copy3(d->mocap_pos+3*id, m->body_pos+3*i);
        mju_copy4(d->mocap_quat+4*id, m->body_quat+4*i);
      }
    }
  } else {
    // set the mocap_quats to {1, 0, 0, 0}
    for (int i=0; i<m->nmocap; i++) {
      d->mocap_quat[4*i] = 1.0;
    }
  }
}



// clear data, set data->qpos = model->qpos0
void mj_resetData(const mjModel* m, mjData* d) {
  _resetData(m, d, 0);
}



// clear data, set data->qpos = model->qpos0, fill with debug_value
void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value) {
  _resetData(m, d, debug_value);
}



// reset data, set fields from specified keyframe
void mj_resetDataKeyframe(const mjModel* m, mjData* d, int key) {
  _resetData(m, d, 0);

  // copy keyframe data if key is valid
  if (key>=0 && key<m->nkey) {
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
    mju_free(d->buffer);
    mju_free(d->stack);
    mju_free(d);
  }
}


// number of position and velocity coordinates for each joint type
const int nPOS[4] = {7, 4, 1, 1};
const int nVEL[4] = {6, 3, 1, 1};

static int sensorSize(mjtSensor sensor_type, int nuser_sensor) {
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
  case mjSENS_JOINTLIMITPOS:
  case mjSENS_JOINTLIMITVEL:
  case mjSENS_JOINTLIMITFRC:
  case mjSENS_TENDONLIMITPOS:
  case mjSENS_TENDONLIMITVEL:
  case mjSENS_TENDONLIMITFRC:
  case mjSENS_CLOCK:
    return 1;

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
    return 3;

  case mjSENS_BALLQUAT:
  case mjSENS_FRAMEQUAT:
    return 4;

  case mjSENS_USER:
    return nuser_sensor;

  // don't use a 'default' case, so compiler warns about missing values
  }
  return -1;
}

// returns the number of objects of the given type
//   -1: mjOBJ_UNKNOWN
//   -2: invalid objtype
static int numObjects(const mjModel* m, mjtObj objtype) {
  switch (objtype) {
    case mjOBJ_UNKNOWN:
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
  X(body_parentid,      nbody,         nbody        , 0                      ) \
  X(body_rootid,        nbody,         nbody        , 0                      ) \
  X(body_weldid,        nbody,         nbody        , 0                      ) \
  X(body_mocapid,       nbody,         nmocap       , 0                      ) \
  X(body_jntadr,        nbody,         njnt         , m->body_jntnum         ) \
  X(body_dofadr,        nbody,         nv           , m->body_dofnum         ) \
  X(body_geomadr,       nbody,         ngeom        , m->body_geomnum        ) \
  X(jnt_qposadr,        njnt,          nq           , 0                      ) \
  X(jnt_dofadr,         njnt,          nv           , 0                      ) \
  X(jnt_bodyid,         njnt,          nbody        , 0                      ) \
  X(dof_bodyid,         nv,            nbody        , 0                      ) \
  X(dof_jntid,          nv,            njnt         , 0                      ) \
  X(dof_parentid,       nv,            nv           , 0                      ) \
  X(dof_Madr,           nv,            nM           , 0                      ) \
  X(geom_bodyid,        ngeom,         nbody        , 0                      ) \
  X(geom_matid,         ngeom,         nmat         , 0                      ) \
  X(site_bodyid,        nsite,         nbody        , 0                      ) \
  X(site_matid,         nsite,         nmat         , 0                      ) \
  X(cam_bodyid,         ncam,          nbody        , 0                      ) \
  X(cam_targetbodyid,   ncam,          nbody        , 0                      ) \
  X(light_bodyid,       nlight,        nbody        , 0                      ) \
  X(light_targetbodyid, nlight,        nbody        , 0                      ) \
  X(mesh_vertadr,       nmesh,         nmeshvert    , m->mesh_vertnum        ) \
  X(mesh_texcoordadr,   nmesh,         nmeshtexvert , 0                      ) \
  X(mesh_faceadr,       nmesh,         nmeshface    , m->mesh_facenum        ) \
  X(mesh_graphadr,      nmesh,         nmeshgraph   , 0                      ) \
  X(skin_matid,         nskin,         nmat         , 0                      ) \
  X(skin_vertadr,       nskin,         nskinvert    , m->skin_vertnum        ) \
  X(skin_texcoordadr,   nskin,         nskintexvert , 0                      ) \
  X(skin_faceadr,       nskin,         nskinface    , m->skin_facenum        ) \
  X(skin_boneadr,       nskin,         nskinbone    , m->skin_bonenum        ) \
  X(skin_bonevertadr,   nskinbone,     nskinbonevert, m->skin_bonevertnum    ) \
  X(skin_bonebodyid,    nskinbone,     nbody        , 0                      ) \
  X(skin_bonevertid,    nskinbonevert, nskinvert    , 0                      ) \
  X(pair_geom1,         npair,         ngeom        , 0                      ) \
  X(pair_geom2,         npair,         ngeom        , 0                      ) \
  X(tendon_adr,         ntendon,       nwrap        , m->tendon_num          ) \
  X(tendon_matid,       ntendon,       nmat         , 0                      ) \
  X(numeric_adr,        nnumeric,      nnumericdata , m->numeric_size        ) \
  X(text_adr,           ntext,         ntextdata    , m->text_size           ) \
  X(tuple_adr,          ntuple,        ntupledata   , m->tuple_size          ) \
  X(name_bodyadr,       nbody,         nnames       , 0                      ) \
  X(name_jntadr,        njnt,          nnames       , 0                      ) \
  X(name_geomadr,       ngeom,         nnames       , 0                      ) \
  X(name_siteadr,       nsite,         nnames       , 0                      ) \
  X(name_camadr,        ncam,          nnames       , 0                      ) \
  X(name_lightadr,      nlight,        nnames       , 0                      ) \
  X(name_meshadr,       nmesh,         nnames       , 0                      ) \
  X(name_skinadr,       nskin,         nnames       , 0                      ) \
  X(name_hfieldadr,     nhfield,       nnames       , 0                      ) \
  X(name_texadr,        ntex,          nnames       , 0                      ) \
  X(name_matadr,        nmat,          nnames       , 0                      ) \
  X(name_pairadr,       npair,         nnames       , 0                      ) \
  X(name_excludeadr,    nexclude,      nnames       , 0                      ) \
  X(name_eqadr,         neq,           nnames       , 0                      ) \
  X(name_tendonadr,     ntendon,       nnames       , 0                      ) \
  X(name_actuatoradr,   nu,            nnames       , 0                      ) \
  X(name_sensoradr,     nsensor,       nnames       , 0                      ) \
  X(name_numericadr,    nnumeric,      nnames       , 0                      ) \
  X(name_textadr,       ntext,         nnames       , 0                      ) \
  X(name_tupleadr,      ntuple,        nnames       , 0                      ) \
  X(name_keyadr,        nkey,          nnames       , 0                      )

  #define X(adrarray, nadrs, ntarget, numarray) {             \
    int *nums = (numarray);                                   \
    for (int i=0; i<m->nadrs; i++) {                          \
      int adrsmin = m->adrarray[i];                           \
      int num = (nums ? nums[i] : 1);                         \
      if (num < 0) {                                          \
        return "Invalid model: " #numarray " is negative.";   \
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
  for (int i=0; i<m->nbody; i++) {
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
  for (int i=0; i<m->njnt; i++) {
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
  for (int i=0; i<m->nv; i++) {
    if (m->dof_parentid[i] >= i) {
      return "Invalid model: bad dof_parentid.";
    }
  }
  for (int i=0; i<m->ngeom; i++) {
    if (m->geom_condim[i] > 6 || m->geom_condim[i] < 0) {
      return "Invalid model: geom_condim out of bounds.";
    }
    if (m->geom_type[i] == mjGEOM_HFIELD) {
      if (m->geom_dataid[i] >= m->nhfield || m->geom_dataid[i] < -1) {
        return "Invalid model: geom_dataid out of bounds.";
      }
    } else if (m->geom_type[i] == mjGEOM_MESH) {
      if (m->geom_dataid[i] >= m->nmesh || m->geom_dataid[i] < -1) {
        return "Invalid model: geom_dataid out of bounds.";
      }
    }
  }
  for (int i=0; i<m->nhfield; i++) {
    int hfield_adr = m->hfield_adr[i] + m->hfield_nrow[i]*m->hfield_ncol[i];
    if (hfield_adr > m->nhfielddata || m->hfield_adr[i] < 0) {
      return "Invalid model: hfield_adr out of bounds.";
    }
  }
  for (int i=0; i<m->ntex; i++) {
    int tex_adr = m->tex_adr[i] + 3*m->tex_height[i]*m->tex_width[i];
    if (tex_adr > m->ntexdata || m->tex_adr[i] < 0) {
      return "Invalid model: tex_adr out of bounds.";
    }
  }
  for (int i=0; i<m->npair; i++) {
    int pair_body1 = (m->pair_signature[i] & 0xFFFF) - 1;
    if (pair_body1 >= m->nbody || pair_body1 < 0) {
      return "Invalid model: pair_body1 out of bounds.";
    }
    int pair_body2 = (m->pair_signature[i] >> 16) - 1;
    if (pair_body2 >= m->nbody || pair_body2 < 0) {
      return "Invalid model: pair_body2 out of bounds.";
    }
  }
  for (int i=0; i<m->neq; i++) {
    int obj1id = m->eq_obj1id[i];
    int obj2id = m->eq_obj2id[i];
    switch (m->eq_type[i]) {
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
      case mjEQ_DISTANCE:
        return "distance equality constraints are no longer supported";
      case mjEQ_WELD:
      case mjEQ_CONNECT:
        if (obj1id >= m->nbody || obj1id < 0) {
          return "Invalid model: eq_obj1id out of bounds.";
        }
        if (obj2id >= m->nbody || obj2id < 0) {
          return "Invalid model: eq_obj2id out of bounds.";
        }
        break;
    }
  }
  for (int i=0; i<m->nwrap; i++) {
    int wrap_objid = m->wrap_objid[i];
    switch (m->wrap_type[i]) {
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
  for (int i=0; i<m->nu; i++) {
    int actuator_trntype = m->actuator_trntype[i];
    int id = m->actuator_trnid[2*i];
    int idslider = m->actuator_trnid[2*i+1];
    switch (actuator_trntype) {
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
      case mjTRN_UNDEFINED:
        // actuator_trnid not used.
        break;
    }
  }
  for (int i=0; i<m->nsensor; i++) {
    mjtSensor sensor_type = m->sensor_type[i];
    int sensor_size = sensorSize(sensor_type, m->nuser_sensor);
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
  }
  for (int i=0; i<m->nexclude; i++) {
    int exclude_body1 = (m->exclude_signature[i] & 0xFFFF) - 1;
    if (exclude_body1 >= m->nbody || exclude_body1 < 0) {
      return "Invalid model: exclude_body1 out of bounds.";
    }
    int exclude_body2 = (m->exclude_signature[i] >> 16) - 1;
    if (exclude_body2 >= m->nbody || exclude_body2 < 0) {
      return "Invalid model: exclude_body2 out of bounds.";
    }
  }
  for (int i=0; i<m->ntuple; i++) {
    for (int j=0; j<m->tuple_size[i]; j++) {
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
