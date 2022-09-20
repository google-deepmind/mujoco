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

#include "render/render_gl3.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "engine/engine_array_safety.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_vis_init.h"
#include "render/render_context.h"
#include "render/render_gl2.h"
#include "render/render_util.h"
#include "render/glad/glad.h"


//----------------------------- low-level 3D rendering ---------------------------------------------

// check if head is behind plane
static int isBehind(const float* headpos, const float* pos, const float* mat) {
  return ((headpos[0]-pos[0])*mat[2] +
          (headpos[1]-pos[1])*mat[5] +
          (headpos[2]-pos[2])*mat[8] < 0.0f);
}



// check if geom is reflective
static int isReflective(const mjvGeom* geom) {
  return ((geom->type==mjGEOM_PLANE || geom->type==mjGEOM_BOX) &&
          !geom->transparent &&
          (geom->reflectance>0));
}



// texture types for settexture
enum {
  mjtexSHADOW = 0,
  mjtexSKYBOX,
  mjtexREGULAR
};


// enable/disable texture mapping
static void settexture(int type, int state, const mjrContext* con, const mjvGeom* geom) {
  float plane[4], scl[2];

  // shadow
  if (type==mjtexSHADOW) {
    // enable
    if (state) {
      glActiveTexture(GL_TEXTURE1);
      glEnable(GL_TEXTURE_2D);
      glEnable(GL_TEXTURE_GEN_S);
      glEnable(GL_TEXTURE_GEN_T);
      glEnable(GL_TEXTURE_GEN_R);
      glEnable(GL_TEXTURE_GEN_Q);
      glBindTexture(GL_TEXTURE_2D, con->shadowTex);
    }

    // disable
    else {
      glActiveTexture(GL_TEXTURE1);
      glDisable(GL_TEXTURE_2D);
      glDisable(GL_TEXTURE_GEN_S);
      glDisable(GL_TEXTURE_GEN_T);
      glDisable(GL_TEXTURE_GEN_R);
      glDisable(GL_TEXTURE_GEN_Q);
    }
  }

  // explicit texture coordinates
  else if (type==mjtexREGULAR && geom->texcoord) {
    // enable
    if (state) {
      glActiveTexture(GL_TEXTURE0);
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, con->texture[geom->texid]);
    }

    // disable
    else {
      glActiveTexture(GL_TEXTURE0);
      glDisable(GL_TEXTURE_2D);
    }
  }

  // 2D
  else if (type==mjtexREGULAR && con->textureType[geom->texid]==mjTEXTURE_2D) {
    // enable
    if (state) {
      glActiveTexture(GL_TEXTURE0);
      glEnable(GL_TEXTURE_2D);
      glEnable(GL_TEXTURE_GEN_S);
      glEnable(GL_TEXTURE_GEN_T);
      glBindTexture(GL_TEXTURE_2D, con->texture[geom->texid]);

      // determine scaling, adjust for pre-scaled geoms
      scl[0] = geom->texrepeat[0];
      scl[1] = geom->texrepeat[1];
      if (geom->dataid>=0) {
        if (geom->size[0]>0) {
          scl[0] = scl[0] / mju_max(mjMINVAL, geom->size[0]);
        }

        if (geom->size[1]>0) {
          scl[1] = scl[1] / mju_max(mjMINVAL, geom->size[1]);
        }
      }

      // uniform: repeat relative to spatial units rather than object
      if (geom->texuniform) {
        if (geom->size[0]>0) {
          scl[0] = scl[0] * geom->size[0];
        }

        if (geom->size[1]>0) {
          scl[1] = scl[1] * geom->size[1];
        }
      }

      // set mapping
      mjr_setf4(plane, 0.5*scl[0], 0, 0, -0.5);
      glTexGenfv(GL_S, GL_OBJECT_PLANE, plane);
      mjr_setf4(plane, 0, -0.5*scl[1], 0, -0.5);
      glTexGenfv(GL_T, GL_OBJECT_PLANE, plane);
    }

    // disable
    else {
      glActiveTexture(GL_TEXTURE0);
      glDisable(GL_TEXTURE_2D);
      glDisable(GL_TEXTURE_GEN_S);
      glDisable(GL_TEXTURE_GEN_T);
    }
  }

  // cube or skybox
  else {
    // enable
    if (state) {
      glActiveTexture(GL_TEXTURE0);
      glEnable(GL_TEXTURE_CUBE_MAP);
      glEnable(GL_TEXTURE_GEN_S);
      glEnable(GL_TEXTURE_GEN_T);
      glEnable(GL_TEXTURE_GEN_R);
      glBindTexture(GL_TEXTURE_CUBE_MAP, con->texture[geom->texid]);

      // set mapping : cube
      if (type==mjtexREGULAR) {
        mjr_setf4(plane, geom->texuniform ? geom->size[0] : 1, 0, 0, 0);
        glTexGenfv(GL_S, GL_OBJECT_PLANE, plane);
        mjr_setf4(plane, 0, geom->texuniform ? geom->size[1] : 1, 0, 0);
        glTexGenfv(GL_T, GL_OBJECT_PLANE, plane);
        mjr_setf4(plane, 0, 0, geom->texuniform ? geom->size[2] : 1, 0);
        glTexGenfv(GL_R, GL_OBJECT_PLANE, plane);
      }

      // set mapping: skybox (rotate 90 deg around X)
      else {
        mjr_setf4(plane, 1, 0, 0, 0);
        glTexGenfv(GL_S, GL_OBJECT_PLANE, plane);
        mjr_setf4(plane, 0, 0, 1, 0);
        glTexGenfv(GL_T, GL_OBJECT_PLANE, plane);
        mjr_setf4(plane, 0, -1, 0, 0);
        glTexGenfv(GL_R, GL_OBJECT_PLANE, plane);
      }
    }

    // disable
    else {
      glActiveTexture(GL_TEXTURE0);
      glDisable(GL_TEXTURE_CUBE_MAP);
      glDisable(GL_TEXTURE_GEN_S);
      glDisable(GL_TEXTURE_GEN_T);
      glDisable(GL_TEXTURE_GEN_R);
    }
  }
}



// rendering modes for renderGeom
enum {
  mjrRND_NORMAL       = 0,
  mjrRND_SHADOWMAP    = 1,
  mjrRND_SHADOWCAST   = 2,
  mjrRND_SEGMENT      = 3,
  mjrRND_IDCOLOR      = 4
};


// render one geom
static void renderGeom(const mjvGeom* geom, int mode, const float* headpos,
                       const mjvScene* scn, const mjrContext* con) {
  const float* size = geom->size;
  float temp[4] = {0, 0, 0, 1}, av;
  float rgba[4] = {geom->rgba[0], geom->rgba[1], geom->rgba[2], geom->rgba[3]};
  int behind, whichface, lighting;

  // lines to do not cast shadows
  if (mode==mjrRND_SHADOWCAST && geom->type==mjGEOM_LINE) {
    return;
  }

  // make transformation matrix
  float mat[16] = {
    geom->mat[0], geom->mat[3], geom->mat[6], 0.0f,
    geom->mat[1], geom->mat[4], geom->mat[7], 0.0f,
    geom->mat[2], geom->mat[5], geom->mat[8], 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
  };

  // precompute isbehind
  behind = isBehind(headpos, geom->pos, geom->mat);

  // enable texture in normal and shadowmap mode
  if (geom->texid>=0 && con->ntexture>0 &&
      (mode==mjrRND_NORMAL || mode==mjrRND_SHADOWMAP)) {
    settexture(mjtexREGULAR, 1, con, geom);
  }

  // make plane more transparent from the back
  if (geom->type==mjGEOM_PLANE && behind && mode==mjrRND_NORMAL) {
    rgba[3] *= 0.3;
  }

  // set material emission: none in shadow mode
  if (mode==mjrRND_NORMAL) {
    mjr_setf4(temp,
              geom->emission*rgba[0],
              geom->emission*rgba[1],
              geom->emission*rgba[2], 1);
  } else {
    mjr_setf4(temp, 0, 0, 0, 1);
  }
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, temp);

  // set color and material
  glColor4fv(rgba);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, geom->shininess * 128.0f);
  mjr_setf4(temp, geom->specular, geom->specular, geom->specular, 1);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, temp);

  // set color for segmentation mode
  if (mode>=mjrRND_SEGMENT) {
    // segid color
    if (mode==mjrRND_IDCOLOR) {
      unsigned char seg[4] = {
        (geom->segid+1) & 0xFF,
        ((geom->segid+1)>>8) & 0xFF,
        ((geom->segid+1)>>16) & 0xFF,
        0xFF
      };
      glColor4ubv(seg);
    }

    // random color
    else {
      float seg[4] = {
        0.1f + 0.8f*mju_Halton(geom->segid+10, 2),
        0.1f + 0.8f*mju_Halton(geom->segid+10, 3),
        0.1f + 0.8f*mju_Halton(geom->segid+10, 5),
        1
      };
      glColor4fv(seg);
    }
  }

  // apply coordinate transformation, except for skin which is global
  glPushMatrix();
  if (geom->type!=mjGEOM_SKIN) {
    glTranslatef(geom->pos[0], geom->pos[1], geom->pos[2]);
    glMultMatrixf(mat);
  }

  // render geom
  switch (geom->type) {
  case mjGEOM_PLANE:                              // plane
    if (behind) {
      glGetIntegerv(GL_CULL_FACE_MODE, &whichface);
      if (whichface==GL_BACK) {
        glCullFace(GL_FRONT);
      }
    }

    // use plane-specific displaylist if available
    if (geom->dataid>=0) {
      glCallList(con->basePlane + geom->dataid+1);
    }

    // otherwise default list
    else {
      glScalef(size[0], size[1], 1.0f);
      glCallList(con->basePlane);
    }

    if (behind && whichface==GL_BACK) {
      glCullFace(GL_BACK);
    }
    break;

  case mjGEOM_HFIELD:                         // height field
    if (geom->dataid>=0) {
      glCallList(con->baseHField + geom->dataid);
    }
    break;

  case mjGEOM_SPHERE:                         // sphere
    glScalef(size[0], size[1], size[2]);
    glCallList(con->baseBuiltin + mjrSPHERE);
    break;

  case mjGEOM_CAPSULE:                        // capsule
    av = 0.5*(size[0]+size[1]);
    glScalef(size[0], size[1], size[2]);
    glCallList(con->baseBuiltin + mjrCYLINDEROPEN);
    glScalef(1, 1, av/size[2]);
    glTranslatef(0, 0, (size[2]-av)/av);
    glCallList(con->baseBuiltin + mjrSPHERETOP);
    glTranslatef(0, 0, 2*(av-size[2])/av);
    glCallList(con->baseBuiltin + mjrSPHEREBOTTOM);
    break;

  case mjGEOM_ELLIPSOID:                      // ellipsoid
    glScalef(size[0], size[1], size[2]);
    glCallList(con->baseBuiltin + mjrSPHERE);
    break;

  case mjGEOM_CYLINDER:                       // cylinder
    glScalef(size[0], size[1], size[2]);
    glCallList(con->baseBuiltin + mjrCYLINDER);
    break;

  case mjGEOM_BOX:                            // box
    glScalef(size[0], size[1], size[2]);
    glCallList(con->baseBuiltin + mjrBOX);
    break;

  case mjGEOM_MESH:                           // mesh
    if (geom->dataid>=0) {
      glCallList(con->baseMesh + geom->dataid);
    }
    break;

  case mjGEOM_ARROW:                          // arrow
    glScalef(size[0], size[1], size[2]/6.0f);
    glTranslatef(0, 0, 1);
    glCallList(con->baseBuiltin + mjrCYLINDER);
    glTranslatef(0, 0, 1);
    glScalef(1.75f, 1.75f, 1);
    glCallList(con->baseBuiltin + mjrCONE);
    break;

  case mjGEOM_ARROW1:                         // arrow without wedges
    glScalef(size[0], size[1], size[2]/6.0f);
    glTranslatef(0, 0, 1);
    glCallList(con->baseBuiltin + mjrCYLINDER);
    glTranslatef(0, 0, 1);
    glCallList(con->baseBuiltin + mjrCONE);
    break;

  case mjGEOM_ARROW2:                         // arrow in both directions
    glScalef(size[0], size[1], size[2]/3.0f);
    glCallList(con->baseBuiltin + mjrCYLINDEROPEN);
    glScalef(1.75f, 1.75f, 0.5f);
    glTranslatef(0, 0, -2);
    glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    glCallList(con->baseBuiltin + mjrCONE);
    glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    glTranslatef(0, 0, 4);
    glCallList(con->baseBuiltin + mjrCONE);
    break;

  case mjGEOM_LINE:                           // line
    glLineWidth(3*con->lineWidth);
    lighting = glIsEnabled(GL_LIGHTING);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, size[2]);
    glEnd();
    glLineWidth(con->lineWidth);
    if (lighting) {
      glEnable(GL_LIGHTING);
    }
    break;

  case mjGEOM_SKIN:                           // skin
    // vertex positions
    glEnableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, con->skinvertVBO[geom->objid]);
    glVertexPointer(3, GL_FLOAT, 0, NULL);

    // vertex normals
    glEnableClientState(GL_NORMAL_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, con->skinnormalVBO[geom->objid]);
    glNormalPointer(GL_FLOAT, 0, NULL);

    // vertex texture coordinates
    if (con->skintexcoordVBO[geom->objid]) {
      glEnableClientState(GL_TEXTURE_COORD_ARRAY);
      glBindBuffer(GL_ARRAY_BUFFER, con->skintexcoordVBO[geom->objid]);
      glTexCoordPointer(2, GL_FLOAT, 0, NULL);
    } else {
      glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    // triangle face indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, con->skinfaceVBO[geom->objid]);

    // draw
    glDrawElements(GL_TRIANGLES, 3*scn->skinfacenum[geom->objid], GL_UNSIGNED_INT, NULL);

    // restore
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    break;
  }

  // undo coordinate transformation
  glPopMatrix();

  // disable texture if enabled
  if (geom->texid>=0 && con->ntexture>0 &&
      (mode==mjrRND_NORMAL || mode==mjrRND_SHADOWMAP)) {
    settexture(mjtexREGULAR, 0, con, geom);
  }
}



void renderGeomReflection(int id, float reflectance, float headpos[3],
                          mjvScene* scn, const mjrContext* con) {
  float old_rgb[3];

  // save rgb, modulate rgb by this->reflectance
  for (int k=0; k<3; k++) {
    old_rgb[k] = scn->geoms[id].rgba[k];
    scn->geoms[id].rgba[k] *= reflectance;
  }

  // render
  renderGeom(scn->geoms+id, mjrRND_NORMAL, headpos, scn, con);

  // restore rgb
  for (int k=0; k<3; k++) {
    scn->geoms[id].rgba[k] = old_rgb[k];
  }
}



//----------------------------- high-level 3D rendering --------------------------------------------

// init, with or without special effects
static void initGL3(const mjvScene* scn, const mjrContext* con) {
  float rgbaWhite[4] = {1.0f, 1.0f, 1.0f, 1.0f};

  // special effects
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_LIGHTING);
  if (scn->flags[mjRND_FOG]) {
    glEnable(GL_FOG);
  } else {
    glDisable(GL_FOG);
  }

  // common options
  glDisable(GL_BLEND);
  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  if (scn->flags[mjRND_CULL_FACE]) {
    glEnable(GL_CULL_FACE);
  } else {
    glDisable(GL_CULL_FACE);
  }
  glShadeModel(GL_SMOOTH);
  glDepthFunc(GL_LEQUAL);
  glDepthRange(0, 1);
  glAlphaFunc(GL_GEQUAL, 0.99f);
  glClearColor(0, 0, 0, 0);
  glClearDepth(1);
  glClearStencil(0);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

  // polygon mode
  glPolygonMode(GL_FRONT_AND_BACK, scn->flags[mjRND_WIREFRAME] ? GL_LINE : GL_FILL);
  glLineWidth(con->lineWidth);
  glFrontFace(GL_CCW);

  // fixed material properties; the rest are set by glColor (track material)
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, rgbaWhite);
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
}



// init lights
static void initLights(mjvScene* scn, const float* headpos, const float* gazedir) {
  // create some ambient light if no ligths are present
  float global = scn->nlight ? 0 : 0.3f;
  float rgba_global[4] = {global, global, global, 1};

  // init light model
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, rgba_global);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);

  // headlight: set pos and dir
  if (scn->lights[0].headlight) {
    for (int i=0; i<3; i++) {
      scn->lights[0].pos[i] = headpos[i];
      scn->lights[0].dir[i] = gazedir[i];
    }
  }

  // set light properties
  for (int i=0; i<scn->nlight; i++) {
    // colors
    glLightfv(GL_LIGHT0+i, GL_AMBIENT, scn->lights[i].ambient);
    glLightfv(GL_LIGHT0+i, GL_DIFFUSE, scn->lights[i].diffuse);
    glLightfv(GL_LIGHT0+i, GL_SPECULAR, scn->lights[i].specular);

    // parameters for directional light
    if (scn->lights[i].directional) {
      glLightf(GL_LIGHT0+i, GL_SPOT_EXPONENT,         0);
      glLightf(GL_LIGHT0+i, GL_SPOT_CUTOFF,           180);
      glLightf(GL_LIGHT0+i, GL_CONSTANT_ATTENUATION,  1);
      glLightf(GL_LIGHT0+i, GL_LINEAR_ATTENUATION,    0);
      glLightf(GL_LIGHT0+i, GL_QUADRATIC_ATTENUATION, 0);
    }

    // parameters for spot light
    else {
      glLightf(GL_LIGHT0+i, GL_SPOT_EXPONENT,         scn->lights[i].exponent);
      glLightf(GL_LIGHT0+i, GL_SPOT_CUTOFF,           scn->lights[i].cutoff);
      glLightf(GL_LIGHT0+i, GL_CONSTANT_ATTENUATION,  scn->lights[i].attenuation[0]);
      glLightf(GL_LIGHT0+i, GL_LINEAR_ATTENUATION,    scn->lights[i].attenuation[1]);
      glLightf(GL_LIGHT0+i, GL_QUADRATIC_ATTENUATION, scn->lights[i].attenuation[2]);
    }
  }

  // disable all lights (enable selectively in render)
  for (int i=0; i<scn->nlight; i++) {
    glDisable(GL_LIGHT0+i);
  }
}



// set projection and modelview
static void setView(int view, mjrRect viewport, const mjvScene* scn, const mjrContext* con,
                    float* camProject, float* camView) {
  mjvGLCamera cam;

  // copy specified camera for stereo, average for mono (view = -1)
  if (view>=0) {
    cam = scn->camera[view];
  } else {
    cam = mjv_averageCamera(scn->camera, scn->camera+1);
  }

  // compute frustum halfwidth so as to match viewport aspect ratio
  float halfwidth = 0.5f * (float)viewport.width/(float)viewport.height *
                    (cam.frustum_top - cam.frustum_bottom);

  // set projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(cam.frustum_center - halfwidth,
            cam.frustum_center + halfwidth,
            cam.frustum_bottom,
            cam.frustum_top,
            cam.frustum_near,
            cam.frustum_far);

  // save projection matrix if requested
  if (camProject) {
    glGetFloatv(GL_PROJECTION_MATRIX, camProject);
  }

  // set modelview
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  mjr_lookAt(cam.pos, cam.forward, cam.up);
  if (scn->enabletransform) {
    mjr_transform(scn->translate, scn->rotate, scn->scale);
  }

  // save modelview matrix if requested
  if (camView) {
    glGetFloatv(GL_MODELVIEW_MATRIX, camView);
  }
}



// comparison function for geom sorting
quicksortfunc(geomcompare, context, el1, el2) {
  mjvGeom* geom = (mjvGeom*) context;
  float d1 = geom[*(int*)el1].camdist;
  float d2 = geom[*(int*)el2].camdist;

  if (d1 < d2) {
    return -1;
  } else if (d1 == d2) {
    return 0;
  } else {
    return 1;
  }
}



// adjust light n position and direction
static void adjustLight(const mjvLight* thislight, int n) {
  float temp[4];

  // set position and direction according to type
  if (thislight->directional) {
    mjr_setf4(temp, -thislight->dir[0], -thislight->dir[1], -thislight->dir[2], 0);
    glLightfv(GL_LIGHT0+n, GL_POSITION, temp);
  } else {
    mjr_setf4(temp, thislight->dir[0], thislight->dir[1], thislight->dir[2], 0);
    glLightfv(GL_LIGHT0+n, GL_SPOT_DIRECTION, temp);
    mjr_setf4(temp, thislight->pos[0], thislight->pos[1], thislight->pos[2], 1);
    glLightfv(GL_LIGHT0+n, GL_POSITION, temp);
  }
}



// render
void mjr_render(mjrRect viewport, mjvScene* scn, const mjrContext* con) {
  int stereo, nt, ngeom = scn->ngeom, nlight = mjMIN(mjMAXLIGHT, scn->nlight);
  unsigned int drawbuffer;
  mjvGLCamera cam;
  mjtNum hpos[3], hfwd[3];
  float temp[4], headpos[3], forward[3], skyboxdst;
  float camProject[16], camView[16], lightProject[16], lightView[16];
  double clipplane[4];
  float biasMatrix[16] = {
    0.5f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.5f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.5f, 0.0f,
    0.5f, 0.5f, 0.5f, 1.0f
  };
  float tempMatrix[16], textureMatrix[16];
  mjvGeom *thisgeom, tempgeom;
  mjvLight *thislight;

  // empty viewport: nothing to do
  if (viewport.width<=0 || viewport.height<=0) {
    return;
  }

  // average cameras
  cam = mjv_averageCamera(scn->camera, scn->camera+1);

  // check znear
  if (cam.frustum_near<mjMINVAL) {
    // geoms: error
    if (scn->ngeom) {
      mju_error("mjvScene frustum_near too small in mjr_render");
    }

    // no geoms: return silently
    else {
      return;
    }
  }

  // upload dynamic skin data to GPU
  for (int i=0; i<scn->nskin; i++) {
    // upload positions to VBO
    glBindBuffer(GL_ARRAY_BUFFER, con->skinvertVBO[i]);
    glBufferData(GL_ARRAY_BUFFER,
                 3*scn->skinvertnum[i]*sizeof(float),
                 scn->skinvert + 3*scn->skinvertadr[i],
                 GL_STREAM_DRAW);

    // upload normals to VBO
    glBindBuffer(GL_ARRAY_BUFFER, con->skinnormalVBO[i]);
    glBufferData(GL_ARRAY_BUFFER,
                 3*scn->skinvertnum[i]*sizeof(float),
                 scn->skinnormal + 3*scn->skinvertadr[i],
                 GL_STREAM_DRAW);
  }

  // determine drawbuffer; may be changed by stereo later
  if (con->currentBuffer==mjFB_WINDOW) {
    drawbuffer = (con->windowDoublebuffer ? GL_BACK : GL_FRONT);
  } else {
    drawbuffer = GL_COLOR_ATTACHMENT0;
  }

  // compute head position and gaze direction in model space
  mjv_cameraInModel(hpos, hfwd, NULL, scn);
  mju_n2f(headpos, hpos, 3);
  mju_n2f(forward, hfwd, 3);

  // init lights
  initLights(scn, headpos, forward);

  // make list of transparent geoms
  nt = 0;
  for (int i=0; i<ngeom; i++) {
    // get geom pointer
    thisgeom = scn->geoms + i;

    if (thisgeom->rgba[3]<0.995 || (thisgeom->type==mjGEOM_PLANE &&
                                    isBehind(headpos, thisgeom->pos, thisgeom->mat))) {
      // include index in list
      scn->geomorder[nt++] = i;
      thisgeom->transparent = 1;

      // compute distance to camera
      thisgeom->camdist = sqrtf((thisgeom->pos[0]-headpos[0])*(thisgeom->pos[0]-headpos[0]) +
                                (thisgeom->pos[1]-headpos[1])*(thisgeom->pos[1]-headpos[1]) +
                                (thisgeom->pos[2]-headpos[2])*(thisgeom->pos[2]-headpos[2]));

      // correct for rbound
      thisgeom->camdist -= mjv_rbound(thisgeom);

      // plane always far away
      if (thisgeom->type==mjGEOM_PLANE) {
        thisgeom->camdist = 1E+10;
      }
    } else {
      thisgeom->transparent = 0;
    }
  }

  // sort transparent geoms according to distance to camera
  mjQUICKSORT(scn->geomorder, nt, sizeof(int), geomcompare, scn->geoms);

  // allow only one reflective geom
  int j = 0;
  for (int i=0; i<ngeom; i++) {
    if (j) {
      scn->geoms[i].reflectance = 0;
    } else if (isReflective(scn->geoms + i)) {
      j = 1;
    }
  }

  // init OpenGL
  initGL3(scn, con);

  // set full viewport
  glViewport(viewport.left, viewport.bottom, viewport.width, viewport.height);

  // clear, with scissor
  glScissor(viewport.left, viewport.bottom, viewport.width, viewport.height);
  glEnable(GL_SCISSOR_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDisable(GL_SCISSOR_TEST);

  // determine stereo; quadbuffered reverts to sidebyside if hardware not available
  stereo = scn->stereo;
  if (stereo==mjSTEREO_QUADBUFFERED && (con->currentBuffer!=mjFB_WINDOW || !con->windowStereo)) {
    stereo = mjSTEREO_SIDEBYSIDE;
  }

  // SIDEBYSIDE: reduce viewport
  if (stereo==mjSTEREO_SIDEBYSIDE) {
    viewport.width /= 2;
  }

  // render with stereo
  for (int view=(stereo?0:-1); view<(stereo?2:0); view++) {
    // change drawbuffer for QUADBUFFERED stereo
    if (stereo==mjSTEREO_QUADBUFFERED) {
      if (con->windowDoublebuffer) {
        drawbuffer = (view ? GL_BACK_RIGHT : GL_BACK_LEFT);
      } else {
        drawbuffer = (view ? GL_FRONT_RIGHT : GL_FRONT_LEFT);
      }
      glDrawBuffer(drawbuffer);

      // clear depth buffer for second view (since it is shared)
      if (view) {
        glClear(GL_DEPTH_BUFFER_BIT);
      }
    }

    // change viewport for SIDEBYSIDE stereo
    else if (stereo==mjSTEREO_SIDEBYSIDE) {
      // move viewport to the right for view 1
      if (view) {
        viewport.left += viewport.width;
      }

      // set reduced/moved viewport
      glViewport(viewport.left, viewport.bottom, viewport.width, viewport.height);
    }

    // set projection and modelview
    setView(view, viewport, scn, con, camProject, camView);

    //---------------------------------- segmentation rendering

    if (scn->flags[mjRND_SEGMENT]) {
      // constant color rendering
      glShadeModel(GL_FLAT);
      glDisable(GL_LIGHTING);
      glDisable(GL_COLOR_MATERIAL);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      glDisable(GL_FOG);
      glDisable(GL_MULTISAMPLE);

      // render all geoms
      for (int i=0; i<ngeom; i++) {
        if (scn->geoms[i].segid>=0) {
          renderGeom(scn->geoms+i,
                     scn->flags[mjRND_IDCOLOR] ? mjrRND_IDCOLOR : mjrRND_SEGMENT,
                     headpos, scn, con);
        }
      }

      // skip the remaining passes
      continue;
    }

    //---------------------------------- reflection rendering

    // plane and box reflection rendering
    if (scn->flags[mjRND_REFLECTION]) {
      for (int i=0; i<ngeom; i++) {
        // get geom pointer
        thisgeom = scn->geoms + i;

        if (isReflective(thisgeom)) {
          // if box, replace with temporary plane matching Z+ box side
          if (thisgeom->type==mjGEOM_BOX) {
            // copy and convert to plane
            tempgeom = *thisgeom;
            tempgeom.type = mjGEOM_PLANE;

            // offset position to Z+ face side
            tempgeom.pos[0] += tempgeom.size[2]*tempgeom.mat[2];
            tempgeom.pos[1] += tempgeom.size[2]*tempgeom.mat[5];
            tempgeom.pos[2] += tempgeom.size[2]*tempgeom.mat[8];

            // redirect pointer
            thisgeom = &tempgeom;
          }

          // camera behind plane or box side: nothing to show
          if (isBehind(headpos, thisgeom->pos, thisgeom->mat)) {
            continue;
          }

          // prepare to render plane in stencil buffer
          glDisable(GL_DEPTH_TEST);
          glColorMask(0, 0, 0, 0);
          glEnable(GL_STENCIL_TEST);
          glClear(GL_STENCIL_BUFFER_BIT);
          glStencilFunc(GL_ALWAYS, 1, 0xFF);
          glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

          // render thisgeom in stencil buffer, always fill
          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
          renderGeom(thisgeom, mjrRND_NORMAL, headpos, scn, con);
          glPolygonMode(GL_FRONT_AND_BACK, scn->flags[mjRND_WIREFRAME] ? GL_LINE : GL_FILL);

          // prepare to render scene where stencil==1
          glEnable(GL_DEPTH_TEST);
          glColorMask(1, 1, 1, 1);
          glStencilFunc(GL_EQUAL, 1, 0xFF);
          glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

          // set clipplane
          glEnable(GL_CLIP_PLANE0);
          clipplane[0] = -thisgeom->mat[2];
          clipplane[1] = -thisgeom->mat[5];
          clipplane[2] = -thisgeom->mat[8];
          clipplane[3] =
            thisgeom->pos[0]*thisgeom->mat[2] +
            thisgeom->pos[1]*thisgeom->mat[5] +
            thisgeom->pos[2]*thisgeom->mat[8];
          glClipPlane(GL_CLIP_PLANE0, clipplane);

          // set reflection matrix
          glFrontFace(GL_CW);
          glPushMatrix();
          mjr_reflect(thisgeom->pos, thisgeom->mat);

          // set light position and direction, enable
          for (int j=0; j<nlight; j++) {
            adjustLight(scn->lights+j, j);
            glEnable(GL_LIGHT0+j);
          }

          // render reflected non-transparent geoms, except for thisgeom
          for (int j=0; j<ngeom; j++) {
            if (!scn->geoms[j].transparent && i!=j) {
              renderGeomReflection(j, thisgeom->reflectance, headpos, scn, con);
            }
          }

          // render reflected transparent geoms, except for thisgeom
          glDepthMask(GL_FALSE);
          glEnable(GL_BLEND);
          if (scn->flags[mjRND_ADDITIVE]) {
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);
          } else {
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
          }
          for (int j=0; j<nt; j++) {
            if (i!=scn->geomorder[j]) {
              renderGeomReflection(scn->geomorder[j], thisgeom->reflectance, headpos, scn, con);
            }
          }
          if (!scn->flags[mjRND_ADDITIVE]) {
            for (int j=nt-1; j>=0; j--) {
              if (i!=scn->geomorder[j]) {
                renderGeomReflection(scn->geomorder[j], thisgeom->reflectance, headpos, scn, con);
              }
            }
          }
          glDepthMask(GL_TRUE);
          glDisable(GL_BLEND);

          // disable lights
          for (int j=0; j<nlight; j++) {
            glDisable(GL_LIGHT0+j);
          }

          // end reflection rendering
          glDisable(GL_STENCIL_TEST);
          glDisable(GL_CLIP_PLANE0);
          glPopMatrix();
          glFrontFace(GL_CCW);
        }
      }
    }

    //---------------------------------- regular rendering

    // set light position and direction, enable non-shadow lights
    for (int i=0; i<nlight; i++) {
      // set light
      thislight = scn->lights + i;
      adjustLight(thislight, i);

      // enable lights without shadows
      if (!thislight->castshadow || !(scn->flags[mjRND_SHADOW] && con->shadowFBO)) {
        glEnable(GL_LIGHT0+i);
      }
    }

    // render reflecting geoms
    //  (only one allowed, more would result in weird transparency)
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    for (int i=0; i<ngeom; i++)
      if (isReflective(scn->geoms+i)) {
        renderGeom(scn->geoms+i, mjrRND_NORMAL, headpos, scn, con);
      }
    glDisable(GL_BLEND);

    // render remaining opaque geoms
    for (int i=0; i<ngeom; i++) {
      if (!scn->geoms[i].transparent && !isReflective(scn->geoms+i)) {
        renderGeom(scn->geoms+i, mjrRND_NORMAL, headpos, scn, con);
      }
    }

    // disable lights
    for (int i=0; i<nlight; i++) {
      glDisable(GL_LIGHT0+i);
    }

    //------------------------------------ shadow rendering

    // black fog, to avoid glow
    float black[4] = {0, 0, 0, 0};
    glFogfv(GL_FOG_COLOR, black);

    // shadow map rendering
    if (scn->flags[mjRND_SHADOW] && con->shadowFBO) {
      for (int i=0; i<nlight; i++) {
        // get pointer
        thislight = scn->lights + i;

        if (thislight->castshadow) {
          // prepare up-direction
          mjr_orthoVec(temp, thislight->dir);

          // set projection: from light viewpoint
          glMatrixMode(GL_PROJECTION);
          glLoadIdentity();
          if (thislight->directional) {
            glOrtho(-con->shadowClip, con->shadowClip,
                    -con->shadowClip, con->shadowClip,
                    cam.frustum_near, cam.frustum_far);
          } else {
            mjr_perspective(mju_min(2*thislight->cutoff*con->shadowScale, 160), 1,
                            cam.frustum_near, cam.frustum_far);
          }
          glGetFloatv(GL_PROJECTION_MATRIX, lightProject);

          // set modelview: from light viewpoint
          glMatrixMode(GL_MODELVIEW);
          glLoadIdentity();
          mjr_lookAt(thislight->pos, thislight->dir, temp);
          glGetFloatv(GL_MODELVIEW_MATRIX, lightView);

          // adjust OpenGL settings for shadow rendering
          glBindFramebuffer(GL_FRAMEBUFFER, con->shadowFBO);
          glDrawBuffer(GL_NONE);
          glClear(GL_DEPTH_BUFFER_BIT);
          glViewport(1, 1, con->shadowSize-2, con->shadowSize-2); // avoid infinite shadows from edges
          glCullFace(GL_FRONT);
          glShadeModel(GL_FLAT);
          glDisable(GL_LIGHTING);
          glColorMask(0, 0, 0, 0);

          // render all geoms to depth texture
          for (int j=0; j<ngeom; j++) {
            renderGeom(scn->geoms+j, mjrRND_SHADOWCAST, headpos, scn, con);
          }

          // restore OpenGL settings
          glBindFramebuffer(GL_FRAMEBUFFER,
                            con->currentBuffer==mjFB_WINDOW ? 0 : con->offFBO);
          glDrawBuffer(drawbuffer);
          glViewport(viewport.left, viewport.bottom, viewport.width, viewport.height);
          glCullFace(GL_BACK);
          glShadeModel(GL_SMOOTH);
          glEnable(GL_LIGHTING);
          glColorMask(1, 1, 1, 1);
          glMatrixMode(GL_PROJECTION);
          glLoadMatrixf(camProject);
          glMatrixMode(GL_MODELVIEW);
          glLoadMatrixf(camView);

          // compute camera-light mapping
          mjr_mulMat44(tempMatrix, lightProject, lightView);
          mjr_mulMat44(textureMatrix, biasMatrix, tempMatrix);

          // enable texture machinery
          settexture(mjtexSHADOW, 1, con, 0);
          mjr_getrow4(temp, textureMatrix, 0);
          glTexGenfv(GL_S, GL_EYE_PLANE, temp);
          mjr_getrow4(temp, textureMatrix, 1);
          glTexGenfv(GL_T, GL_EYE_PLANE, temp);
          mjr_getrow4(temp, textureMatrix, 2);
          glTexGenfv(GL_R, GL_EYE_PLANE, temp);
          mjr_getrow4(temp, textureMatrix, 3);
          glTexGenfv(GL_Q, GL_EYE_PLANE, temp);

          // render with shadow light i
          glEnable(GL_ALPHA_TEST);
          glEnable(GL_BLEND);
          glBlendFunc(GL_ONE, GL_ONE);
          glEnable(GL_LIGHT0+i);

          // only opaque geoms accept shadows
          for (int j=0; j<ngeom; j++) {
            if (!scn->geoms[j].transparent) {
              renderGeom(scn->geoms+j, mjrRND_SHADOWMAP, headpos, scn, con);
            }
          }

          glDisable(GL_LIGHT0+i);
          glDisable(GL_BLEND);
          glDisable(GL_ALPHA_TEST);

          // disable texture machinery
          settexture(mjtexSHADOW, 0, con, 0);
        }
      }
    }

    // restore fog color
    glFogfv(GL_FOG_COLOR, con->fogRGBA);

    //------------------------------------ skybox rendering

    if (scn->flags[mjRND_SKYBOX]) {
      // skybox always filled
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

      // find skybox texture
      for (int i=0; i<con->ntexture; i++) {
        if (con->textureType[i]==mjTEXTURE_SKYBOX) {
          // save first skybox texture id in tempgeom
          memset(&tempgeom, 0, sizeof(mjvGeom));
          tempgeom.texid = i;

          // modify settings
          glDisable(GL_LIGHTING);
          glDisable(GL_COLOR_MATERIAL);
          glCullFace(GL_FRONT);

          // center at headpos, scale to avoid far clipping: just below 1/sqrt(2)
          glPushMatrix();
          glTranslatef(headpos[0], headpos[1], headpos[2]);
          skyboxdst = cam.frustum_far*0.70f;
          if (scn->enabletransform) {
            skyboxdst /= scn->scale;
          }
          glScalef(skyboxdst, skyboxdst, skyboxdst);

          // render cylinder, with skybox texture
          settexture(mjtexSKYBOX, 1, con, &tempgeom);
          glColor4f(1, 1, 1, 1);
          glCallList(con->baseBuiltin + mjrCYLINDER);
          settexture(mjtexSKYBOX, 0, con, &tempgeom);

          // haze
          if (scn->flags[mjRND_HAZE]) {
            for (int j=0; j<ngeom; j++) {
              if (scn->geoms[j].type==mjGEOM_PLANE &&
                  scn->geoms[j].size[0]==0 &&
                  scn->geoms[j].size[1]==0) {
                // compute headpos elevation above plane
                float* mat3 = scn->geoms[j].mat;
                float elevation = ((headpos[0]-scn->geoms[j].pos[0])*mat3[2] +
                                   (headpos[1]-scn->geoms[j].pos[1])*mat3[5] +
                                   (headpos[2]-scn->geoms[j].pos[2])*mat3[8]) /
                                  skyboxdst;

                // below plane: no rendering
                if (elevation<0) {
                  break;
                }

                // rotate to plane
                float mat4[16] = {
                  mat3[0], mat3[3], mat3[6], 0.0f,
                  mat3[1], mat3[4], mat3[7], 0.0f,
                  mat3[2], mat3[5], mat3[8], 0.0f,
                  0.0f,    0.0f,    0.0f,    1.0f
                };
                glMultMatrixf(mat4);

                // translate and scale
                glTranslatef(0, 0, -elevation);
                glScalef(1, 1, elevation);

                // render
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glCallList(con->baseBuiltin + mjrHAZE);
                glDisable(GL_BLEND);

                // only first infinite plane
                break;
              }
            }
          }

          // recover settings
          glPopMatrix();
          glEnable(GL_LIGHTING);
          glEnable(GL_COLOR_MATERIAL);
          glCullFace(GL_BACK);

          // render only first skybox
          break;
        }
      }

      // wireframe or filled depending on flag
      glPolygonMode(GL_FRONT_AND_BACK, scn->flags[mjRND_WIREFRAME] ? GL_LINE : GL_FILL);
    }

    //------------------------------------ transparent regular rendering

    // enable lights
    for (int i=0; i<nlight; i++) {
      glEnable(GL_LIGHT0+i);
    }

    // blend mode
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    if (scn->flags[mjRND_ADDITIVE]) {
      glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    } else {
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    // render transparent geoms: front to back
    for (int i=0; i<nt; i++) {
      renderGeom(scn->geoms+scn->geomorder[i], mjrRND_NORMAL, headpos, scn, con);
    }

    // render transparent geoms: back to front, if not additive
    if (!scn->flags[mjRND_ADDITIVE]) {
      for (int i=nt-1; i>=0; i--) {
        renderGeom(scn->geoms+scn->geomorder[i], mjrRND_NORMAL, headpos, scn, con);
      }
    }

    // normal mode
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    // disable lights
    for (int i=0; i<nlight; i++) {
      glDisable(GL_LIGHT0+i);
    }

    //------------------------------------ label rendering

    // render text labels if present
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    for (int i=0; i<ngeom; i++) {
      thisgeom = scn->geoms + i;

      if (thisgeom->label[0]) {
        mjr_textActual(mjFONT_SHADOW, thisgeom->label, con,
                       thisgeom->pos[0], thisgeom->pos[1], thisgeom->pos[2], 1, 1, 1);
      }
    }
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
  }

  // frame
  if (scn->framewidth) {
    // prepare
    glClearColor(scn->framergb[0], scn->framergb[1], scn->framergb[2], 0);
    glEnable(GL_SCISSOR_TEST);

    // bottom edge
    glScissor(viewport.left, viewport.bottom,
              viewport.width, scn->framewidth);
    glClear(GL_COLOR_BUFFER_BIT);

    // top edge
    glScissor(viewport.left, viewport.bottom+viewport.height-scn->framewidth,
              viewport.width, scn->framewidth);
    glClear(GL_COLOR_BUFFER_BIT);

    // left edge
    glScissor(viewport.left, viewport.bottom,
              scn->framewidth, viewport.height);
    glClear(GL_COLOR_BUFFER_BIT);

    // right edge
    glScissor(viewport.left+viewport.width-scn->framewidth, viewport.bottom,
              scn->framewidth, viewport.height);
    glClear(GL_COLOR_BUFFER_BIT);

    // disable scissor
    glDisable(GL_SCISSOR_TEST);
  }

  // restore currentBuffer
  mjr_restoreBuffer(con);
}



// call glFinish
void mjr_finish(void) {
  glFinish();
}



// call glGetError and return result
int mjr_getError(void) {
  return (int)glGetError();
}
