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

#include "render/render_context.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mujoco/mjmacro.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "render/render_util.h"
#include "render/glad/glad.h"

// bitmap font definitions
#include "render/font/normal50.inc"
#include "render/font/normal100.inc"
#include "render/font/normal150.inc"
#include "render/font/normal200.inc"
#include "render/font/normal250.inc"
#include "render/font/normal300.inc"
#include "render/font/back50.inc"
#include "render/font/back100.inc"
#include "render/font/back150.inc"
#include "render/font/back200.inc"
#include "render/font/back250.inc"
#include "render/font/back300.inc"
#include "render/font/big50.inc"
#include "render/font/big100.inc"
#include "render/font/big150.inc"
#include "render/font/big200.inc"
#include "render/font/big250.inc"
#include "render/font/big300.inc"



//----------------------------- custom OpenGL context ----------------------------------------------

// set default mjrContext
void mjr_defaultContext(mjrContext* con) {
  memset(con, 0, sizeof(mjrContext));
}



// allocate lists
static void listAllocate(GLuint* base, GLsizei* range, GLsizei newrange) {
  // allocate lists
  *range = newrange;
  if (newrange) {
    *base = glGenLists(*range);
    if (*base <= 0) {
      mju_error("Could not allocate display lists");
    }
  }
}



// model planes: with grid
static void makePlane(const mjModel* m, mjrContext* con) {
  mjtNum zfar = m->vis.map.zfar * m->stat.extent;
  mjtNum pad, left, right, sz[2], sz2;
  double grid[2][mjMAXPLANEGRID+1];
  int nn[2], nplane;

  // count planes
  nplane = 0;
  for (int i=0; i < m->ngeom; i++) {
    if (m->geom_type[i] == mjGEOM_PLANE) {
      nplane++;
    }
  }

  // allocate list
  listAllocate(&con->basePlane, &con->rangePlane, nplane+1);

  // 0: box-division, for rendering plane geoms without dataid
  glNewList(con->basePlane, GL_COMPILE);
  glBegin(GL_QUADS);
  glNormal3d(0, 0, 1);
  double d = 2.0/m->vis.quality.numquads;
  for (int x=0; x < m->vis.quality.numquads; x++) {
    for (int y=0; y < m->vis.quality.numquads; y++) {
      glVertex3d(d*(x+0)-1, d*(y+0)-1, 0);
      glVertex3d(d*(x+1)-1, d*(y+0)-1, 0);
      glVertex3d(d*(x+1)-1, d*(y+1)-1, 0);
      glVertex3d(d*(x+0)-1, d*(y+1)-1, 0);
    }
  }
  glEnd();
  glEndList();

  // construct planes, offset by 1
  nplane = 0;
  for (int i=0; i < m->ngeom; i++) {
    if (m->geom_type[i] == mjGEOM_PLANE) {
      // get sizes
      sz[0] = m->geom_size[3*i];
      sz[1] = m->geom_size[3*i+1];

      // loop over (x, y)
      for (int k=0; k < 2; k++) {
        // regular dimension
        if (sz[k] > 0) {
          // limit number of grid lines, leave room for padding
          sz2 = mju_max(m->geom_size[3*i+2], sz[k]/(mjMAXPLANEGRID-2));

          // compute grid size, force even
          nn[k] = (int)mju_floor(sz[k]/mju_max(mjMINVAL, sz2));
          nn[k] = nn[k] - (nn[k]%2);

          // add padding
          pad = sz[k] - nn[k]*sz2;
          nn[k] += 2;

          // make grid
          for (int x=0; x < nn[k]; x++) {
            // compute left, right (which is bottom, top for y)
            if (x == 0) {
              left = -sz[k];
              right = left + pad;
            } else {
              left = -sz[k] + pad + (x-1)*sz2*2;
              right = left + (x == nn[k]-1 ? pad : sz2*2);
            }

            // record
            grid[k][x] = left;
            grid[k][x+1] = mjMAX(left, right);  // just in case
          }
        }

        // infinite dimension
        else {
          // get size increment
          mjtNum sX;
          int matid = m->geom_matid[i];
          if (matid >= 0 && m->mat_texrepeat[2*matid+k] > 0) {
            sX = 2/m->mat_texrepeat[2*matid+k];
          } else {
            sX = 2.1*zfar/(mjMAXPLANEGRID-2);
          }

          // create grid, larger than skybox
          d = (2.1*zfar + 2*sX)/mjMAXPLANEGRID;
          for (int x=0; x <= mjMAXPLANEGRID; x++) {
            grid[k][x] = d*x - d*(mjMAXPLANEGRID/2);
          }

          // save number
          nn[k] = mjMAXPLANEGRID;
        }
      }

      // init list
      glNewList(con->basePlane + nplane+1, GL_COMPILE);
      glBegin(GL_QUADS);
      glNormal3d(0, 0, 1);

      // make grid
      for (int x=0; x < nn[0]; x++) {
        for (int y=0; y < nn[1]; y++) {
          glVertex3d(grid[0][x+0], grid[1][y+0], 0);
          glVertex3d(grid[0][x+1], grid[1][y+0], 0);
          glVertex3d(grid[0][x+1], grid[1][y+1], 0);
          glVertex3d(grid[0][x+0], grid[1][y+1], 0);
        }
      }

      // end list
      glEnd();
      glEndList();

      // advance counter
      nplane++;
    }
  }
}



// model-specific meshes
static void makeMesh(const mjModel* m, mjrContext* con) {
  // allocate list
  listAllocate(&con->baseMesh, &con->rangeMesh, 2*m->nmesh);

  // process meshes
  for (int i=0; i < m->nmesh; i++) {
    mjr_uploadMesh(m, con, i);
  }
}



// (re) upload mesh to GPU
void mjr_uploadMesh(const mjModel* m, const mjrContext* con, int meshid) {
  int vertadr, numvert, normaladr, numface, texcoordadr;
  float normal[3], *v1, *v2, *v3, *n1, *n2, *n3, *t1, *t2, *t3;

  // check index
  if (meshid < 0 || meshid >= m->nmesh) {
    mju_error("Invalid mesh index %d", meshid);
  }

  // delete old lists (mesh and convex hull)
  glDeleteLists(con->baseMesh + 2*meshid, 2);

  // get vertex and texcoord address for this mesh
  vertadr = m->mesh_vertadr[meshid];
  normaladr = m->mesh_normaladr[meshid];
  texcoordadr = m->mesh_texcoordadr[meshid];

  // render original mesh
  glNewList(con->baseMesh + 2*meshid, GL_COMPILE);
  glBegin(GL_TRIANGLES);
  for (int face = m->mesh_faceadr[meshid];
       face < m->mesh_faceadr[meshid] + m->mesh_facenum[meshid];
       face++) {
    // compute vertex addresses
    v1 = m->mesh_vert + 3*(m->mesh_face[3*face]   + vertadr);
    v2 = m->mesh_vert + 3*(m->mesh_face[3*face+1] + vertadr);
    v3 = m->mesh_vert + 3*(m->mesh_face[3*face+2] + vertadr);

    // compute normal addresses
    n1 = m->mesh_normal + 3*(m->mesh_facenormal[3*face]   + normaladr);
    n2 = m->mesh_normal + 3*(m->mesh_facenormal[3*face+1] + normaladr);
    n3 = m->mesh_normal + 3*(m->mesh_facenormal[3*face+2] + normaladr);

    // compute texcoord addresses
    if (texcoordadr >= 0) {
      t1 = m->mesh_texcoord + 2*(m->mesh_facetexcoord[3*face]   + texcoordadr);
      t2 = m->mesh_texcoord + 2*(m->mesh_facetexcoord[3*face+1] + texcoordadr);
      t3 = m->mesh_texcoord + 2*(m->mesh_facetexcoord[3*face+2] + texcoordadr);
    } else {
      t1 = t2 = t3 = NULL;
    }

    // compute face normal
    mjr_makeNormal(normal, v1, v2, v3);

    // make OpenGL triangle
    // vertex1
    if (t1) {
      glTexCoord2fv(t1);
      glNormal3fv(n1);
    } else {
      if (n1[0]*normal[0]+n1[1]*normal[1]+n1[2]*normal[2] < 0.8) {
        glNormal3fv(normal);
      } else {
        glNormal3fv(n1);
      }
    }
    glVertex3fv(v1);

    // vertex2
    if (t2) {
      glTexCoord2fv(t2);
      glNormal3fv(n2);
    } else {
      if (n2[0]*normal[0]+n2[1]*normal[1]+n2[2]*normal[2] < 0.8) {
        glNormal3fv(normal);
      } else {
        glNormal3fv(n2);
      }
    }
    glVertex3fv(v2);

    // vertex3
    if (t3) {
      glTexCoord2fv(t3);
      glNormal3fv(n3);
    } else {
      if (n3[0]*normal[0]+n3[1]*normal[1]+n3[2]*normal[2] < 0.8) {
        glNormal3fv(normal);
      } else {
        glNormal3fv(n3);
      }
    }
    glVertex3fv(v3);
  }
  glEnd();
  glEndList();

  // render convex hull if present
  if (m->mesh_graphadr[meshid] >= 0) {
    // get sizes of convex hull
    numvert = m->mesh_graph[m->mesh_graphadr[meshid]];
    numface = m->mesh_graph[m->mesh_graphadr[meshid]+1];

    glNewList(con->baseMesh + 2*meshid+1, GL_COMPILE);
    glBegin(GL_TRIANGLES);
    for (int face=0; face < numface; face++) {
      // face address in graph
      int j = m->mesh_graphadr[meshid] + 2 + 3*numvert + 3*numface + 3*face;

      // compute vertex addresses
      v1 = m->mesh_vert + 3*(m->mesh_graph[j]   + vertadr);
      v2 = m->mesh_vert + 3*(m->mesh_graph[j+1] + vertadr);
      v3 = m->mesh_vert + 3*(m->mesh_graph[j+2] + vertadr);

      // compute texcoord addresses
      if (texcoordadr >= 0) {
        t1 = m->mesh_texcoord + 2*(m->mesh_graph[j]   + texcoordadr);
        t2 = m->mesh_texcoord + 2*(m->mesh_graph[j+1] + texcoordadr);
        t3 = m->mesh_texcoord + 2*(m->mesh_graph[j+2] + texcoordadr);
      } else {
        t1 = t2 = t3 = NULL;
      }

      // make OpenGL triangle
      mjr_makeNormal(normal, v1, v2, v3);
      glNormal3fv(normal);

      // vertex 1
      if (t1) glTexCoord2fv(t1);
      glVertex3fv(v1);

      // vertex 2
      if (t2) glTexCoord2fv(t2);
      glVertex3fv(v2);

      // vertex 3
      if (t3) glTexCoord2fv(t3);
      glVertex3fv(v3);
    }
    glEnd();
    glEndList();
  }
}



// helper structure for adding vertices to height field
struct vertbuf {
  int nvert;
  float vert1[3], vert2[3], vert3[3];
};


// helper function for adding vertices to height field
static void addVert(float x, float y, float z, float sclz, struct vertbuf* buf) {
  float normal[3];

  // update buffer
  memcpy(buf->vert1, buf->vert2, 3*sizeof(float));
  memcpy(buf->vert2, buf->vert3, 3*sizeof(float));
  buf->vert3[0] = x;
  buf->vert3[1] = y;
  buf->vert3[2] = z*sclz;
  buf->nvert++;

  // triangle ready
  if (buf->nvert >= 3) {
    // compensate for alternating orientation
    if (buf->nvert%2) {
      mjr_makeNormal(normal, buf->vert1, buf->vert2, buf->vert3);
      glNormal3fv(normal);
      glVertex3fv(buf->vert1);
      glVertex3fv(buf->vert2);
      glVertex3fv(buf->vert3);
    } else {
      mjr_makeNormal(normal, buf->vert1, buf->vert3, buf->vert2);
      glNormal3fv(normal);
      glVertex3fv(buf->vert1);
      glVertex3fv(buf->vert3);
      glVertex3fv(buf->vert2);
    }
  }
}



// model-specific height fields
static void makeHField(const mjModel* m, mjrContext* con) {
  // allocate list
  listAllocate(&con->baseHField, &con->rangeHField, m->nhfield);

  // uploaed all heightfields
  for (int i=0; i < m->nhfield; i++) {
    mjr_uploadHField(m, con, i);
  }
}



// (re) upload height field to GPU
void mjr_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid) {
  int d1 = 1, d2 = 0;
  float width, height, sz[4];
  struct vertbuf buf;
  float* data;

  // check index
  if (hfieldid < 0 || hfieldid >= m->nhfield) {
    mju_error("Invalid height field index %d", hfieldid);
  }

  // delete old list
  glDeleteLists(con->baseHField + hfieldid, 1);

  // init list, get elevation data address
  glNewList(con->baseHField + hfieldid, GL_COMPILE);
  data = m->hfield_data + m->hfield_adr[hfieldid];

  // (half) width and height of integer grid
  width = 0.5f * (m->hfield_ncol[hfieldid]-1);
  height = 0.5f * (m->hfield_nrow[hfieldid]-1);

  // convert size to float
  for (int r=0; r < 4; r++) {
    sz[r] = (float)m->hfield_size[4*hfieldid+r];
  }

  // render height field as triangles
  glBegin(GL_TRIANGLES);
  int nr = m->hfield_nrow[hfieldid];
  int nc = m->hfield_ncol[hfieldid];
  for (int r=0; r < nr-1; r++) {
    buf.nvert = 0;
    for (int c=0; c < nc; c++) {
      addVert(sz[0]*(c/width-1.0f), sz[1]*((r+d1)/height-1.0f), data[(r+d1)*nc+c], sz[2], &buf);
      addVert(sz[0]*(c/width-1.0f), sz[1]*((r+d2)/height-1.0f), data[(r+d2)*nc+c], sz[2], &buf);
    }
  }
  glEnd();

  // render sides as quads
  glBegin(GL_QUADS);
  for (int r=0; r < nr-1; r++) {
    // left
    glNormal3f(-1, 0, 0);
    glVertex3f(-sz[0], sz[1]*((r+1)/height-1.0f), -sz[3]);
    glVertex3f(-sz[0], sz[1]*((r+0)/height-1.0f), -sz[3]);
    glVertex3f(-sz[0], sz[1]*((r+0)/height-1.0f), data[(r+0)*nc]*sz[2]);
    glVertex3f(-sz[0], sz[1]*((r+1)/height-1.0f), data[(r+1)*nc]*sz[2]);

    // right
    glNormal3f(+1, 0, 0);
    glVertex3f(+sz[0], sz[1]*((r+0)/height-1.0f), -sz[3]);
    glVertex3f(+sz[0], sz[1]*((r+1)/height-1.0f), -sz[3]);
    glVertex3f(+sz[0], sz[1]*((r+1)/height-1.0f), data[(r+1)*nc + nc-1]*sz[2]);
    glVertex3f(+sz[0], sz[1]*((r+0)/height-1.0f), data[(r+0)*nc + nc-1]*sz[2]);
  }

  for (int c=0; c < nc-1; c++) {
    // front
    glNormal3f(0, -1, 0);
    glVertex3f(sz[0]*((c+0)/width-1.0f), -sz[1], -sz[3]);
    glVertex3f(sz[0]*((c+1)/width-1.0f), -sz[1], -sz[3]);
    glVertex3f(sz[0]*((c+1)/width-1.0f), -sz[1], data[c+1]*sz[2]);
    glVertex3f(sz[0]*((c+0)/width-1.0f), -sz[1], data[c]*sz[2]);

    // back
    glNormal3f(0, +1, 0);
    glVertex3f(sz[0]*((c+1)/width-1.0f), +sz[1], -sz[3]);
    glVertex3f(sz[0]*((c+0)/width-1.0f), +sz[1], -sz[3]);
    glVertex3f(sz[0]*((c+0)/width-1.0f), +sz[1], data[(nr-1)*nc + c]*sz[2]);
    glVertex3f(sz[0]*((c+1)/width-1.0f), +sz[1], data[(nr-1)*nc + c+1]*sz[2]);
  }

  // recompute for bottom drawing: different number of grid points
  width = 0.5f * m->vis.quality.numquads;
  height = 0.5f * m->vis.quality.numquads;

  // bottom
  glNormal3f(0, 0, -1);
  for (int r=0; r < m->vis.quality.numquads; r++) {
    for (int c=0; c < m->vis.quality.numquads; c++) {
      glVertex3f(sz[0]*((c+0)/width-1.0f), sz[1]*((r+0)/height-1.0f), -sz[3]);
      glVertex3f(sz[0]*((c+0)/width-1.0f), sz[1]*((r+1)/height-1.0f), -sz[3]);
      glVertex3f(sz[0]*((c+1)/width-1.0f), sz[1]*((r+1)/height-1.0f), -sz[3]);
      glVertex3f(sz[0]*((c+1)/width-1.0f), sz[1]*((r+0)/height-1.0f), -sz[3]);
    }
  }

  glEnd();

  // end list
  glEndList();
}



// set one vertex and normal on sphere, given az, el and sign(top/bottom)
static void setVertexSphere(float* v, float* n, float az, float el, int sign) {
  v[0] = cosf(az) * cosf(el);
  v[1] = sinf(az) * cosf(el);
  v[2] = sign + sinf(el);

  n[0] = v[0];
  n[1] = v[1];
  n[2] = v[2] - sign;
}


// make half a unit sphere: +1: top, -1: bottom
static void halfSphere(int sign, int nSlice, int nStack) {
  float az1, az2, el1, el2;
  float v1[3], v2[3], v3[3], v4[3];
  float n1[3], n2[3], n3[3], n4[3];

  // pole: use triangles
  glBegin(GL_TRIANGLES);
  el1 = (mjPI/2.0f * sign * (nStack-1)) / (float)nStack;
  for (int j=0; j < nSlice; j++) {
    az1 = (2.0f*mjPI * (j+0.0f)) / (float)nSlice;
    az2 = (2.0f*mjPI * (j+1.0f)) / (float)nSlice;

    // compute triangle vertices
    setVertexSphere(v1, n1, az1, el1, sign);
    setVertexSphere(v2, n2, az2, el1, sign);
    v3[0] = v3[1] = 0;
    v3[2] = 2*sign;
    n3[0] = n3[1] = 0;
    n3[2] = sign;

    // make triangle
    if (sign > 0) {
      glNormal3fv(n1);
      glVertex3fv(v1);
      glNormal3fv(n2);
      glVertex3fv(v2);
      glNormal3fv(n3);
      glVertex3fv(v3);
    } else {
      glNormal3fv(n3);
      glVertex3fv(v3);
      glNormal3fv(n2);
      glVertex3fv(v2);
      glNormal3fv(n1);
      glVertex3fv(v1);
    }
  }
  glEnd();

  // the rest: use quads
  glBegin(GL_QUADS);
  for (int i=0; i < nStack-1; i++) {
    el1 = (mjPI/2.0f * sign * (i+0)) / (float)nStack;
    el2 = (mjPI/2.0f * sign * (i+1)) / (float)nStack;

    for (int j=0; j < nSlice; j++) {
      az1 = (2.0f*mjPI * (j+0)) / (float)nSlice;
      az2 = (2.0f*mjPI * (j+1)) / (float)nSlice;

      // compute quad vertices
      setVertexSphere(v1, n1, az1, el1, sign);
      setVertexSphere(v2, n2, az2, el1, sign);
      setVertexSphere(v3, n3, az2, el2, sign);
      setVertexSphere(v4, n4, az1, el2, sign);

      // make quad
      if (sign > 0) {
        glNormal3fv(n1);
        glVertex3fv(v1);
        glNormal3fv(n2);
        glVertex3fv(v2);
        glNormal3fv(n3);
        glVertex3fv(v3);
        glNormal3fv(n4);
        glVertex3fv(v4);
      } else {
        glNormal3fv(n4);
        glVertex3fv(v4);
        glNormal3fv(n3);
        glVertex3fv(v3);
        glNormal3fv(n2);
        glVertex3fv(v2);
        glNormal3fv(n1);
        glVertex3fv(v1);
      }
    }
  }
  glEnd();
}



// make unit sphere
static void sphere(int nSlice, int nStack) {
  float az1, az2, el1, el2;
  float v1[3], v2[3], v3[3], v4[3];
  float n1[3], n2[3], n3[3], n4[3];

  // poles: use triangles
  glBegin(GL_TRIANGLES);
  for (int sign=-1; sign <= 1; sign+=2) {
    el1 = (0.5*mjPI * sign * (nStack/2-1)) / (float)(nStack/2);
    for (int j=0; j < nSlice; j++) {
      az1 = (2.0f*mjPI * (j+0.0f)) / (float)nSlice;
      az2 = (2.0f*mjPI * (j+1.0f)) / (float)nSlice;

      // compute triangle vertices
      setVertexSphere(v1, n1, az1, el1, 0);
      setVertexSphere(v2, n2, az2, el1, 0);
      v3[0] = v3[1] = 0;
      v3[2] = sign;
      n3[0] = n3[1] = 0;
      n3[2] = sign;

      // make triangle
      if (sign > 0) {
        glNormal3fv(n1);
        glVertex3fv(v1);
        glNormal3fv(n2);
        glVertex3fv(v2);
        glNormal3fv(n3);
        glVertex3fv(v3);
      } else {
        glNormal3fv(n3);
        glVertex3fv(v3);
        glNormal3fv(n2);
        glVertex3fv(v2);
        glNormal3fv(n1);
        glVertex3fv(v1);
      }
    }
  }
  glEnd();

  // the rest: use quads
  glBegin(GL_QUADS);
  for (int sign=-1; sign <= 1; sign+=2) {
    for (int i=0; i < nStack/2-1; i++) {
      el1 = (0.5*mjPI * sign * (i+0)) / (float)(nStack/2);
      el2 = (0.5*mjPI * sign * (i+1)) / (float)(nStack/2);

      for (int j=0; j < nSlice; j++) {
        az1 = (2.0f*mjPI * (j+0)) / (float)nSlice;
        az2 = (2.0f*mjPI * (j+1)) / (float)nSlice;

        // compute quad vertices
        setVertexSphere(v1, n1, az1, el1, 0);
        setVertexSphere(v2, n2, az2, el1, 0);
        setVertexSphere(v3, n3, az2, el2, 0);
        setVertexSphere(v4, n4, az1, el2, 0);

        // make quad
        if (sign > 0) {
          glNormal3fv(n1);
          glVertex3fv(v1);
          glNormal3fv(n2);
          glVertex3fv(v2);
          glNormal3fv(n3);
          glVertex3fv(v3);
          glNormal3fv(n4);
          glVertex3fv(v4);
        } else {
          glNormal3fv(n4);
          glVertex3fv(v4);
          glNormal3fv(n3);
          glVertex3fv(v3);
          glNormal3fv(n2);
          glVertex3fv(v2);
          glNormal3fv(n1);
          glVertex3fv(v1);
        }
      }
    }
  }
  glEnd();
}



// set one vertex on disk, given az, r and sign(top/bottom)
static void setVertexDisk(float* v, float az, float r, int sign) {
  v[0] = cosf(az) * r;
  v[1] = sinf(az) * r;
  v[2] = sign;
}


// make disk: +1: top, -1: bottom, 0: zero facing down
static void disk(int sign, int nSlice, int nStack) {
  float az1, az2, r1, r2;
  float v1[3], v2[3], v3[3], v4[3];
  float normal[3] = {0, 0, (sign == 0 ? -1 : sign)};

  // pole: use triangles
  glBegin(GL_TRIANGLES);
  glNormal3fv(normal);
  r1 = 1.0f / (float)nStack;
  for (int j=0; j < nSlice; j++) {
    az1 = (2.0f*mjPI * (j+0)) / (float)nSlice;
    az2 = (2.0f*mjPI * (j+1)) / (float)nSlice;

    // compute triangle vertices
    setVertexDisk(v1, az1, r1, sign);
    setVertexDisk(v2, az2, r1, sign);
    v3[0] = v3[1] = 0;
    v3[2] = sign;

    // make triangle
    if (sign > 0) {
      glVertex3fv(v1);
      glVertex3fv(v2);
      glVertex3fv(v3);
    } else {
      glVertex3fv(v3);
      glVertex3fv(v2);
      glVertex3fv(v1);
    }
  }
  glEnd();

  // the rest: use quads
  glBegin(GL_QUADS);
  glNormal3fv(normal);
  for (int i=0; i < nStack-1; i++) {
    r1 = (i+1) / (float)nStack;
    r2 = (i+2) / (float)nStack;

    for (int j=0; j < nSlice; j++) {
      az1 = (2.0f*mjPI * (j+0)) / (float)nSlice;
      az2 = (2.0f*mjPI * (j+1)) / (float)nSlice;

      // compute quad vertices
      setVertexDisk(v1, az1, r2, sign);
      setVertexDisk(v2, az2, r2, sign);
      setVertexDisk(v3, az2, r1, sign);
      setVertexDisk(v4, az1, r1, sign);

      // make quad
      if (sign > 0) {
        glVertex3fv(v1);
        glVertex3fv(v2);
        glVertex3fv(v3);
        glVertex3fv(v4);
      } else {
        glVertex3fv(v4);
        glVertex3fv(v3);
        glVertex3fv(v2);
        glVertex3fv(v1);
      }
    }
  }
  glEnd();
}



// set vertex and normal on cone, given az, r
static void setVertexCone(float* v, float* n, float az, float r) {
  const float scale = 1.0f/sqrtf(2.0f);

  // vertex
  v[0] = cosf(az) * r;
  v[1] = sinf(az) * r;
  v[2] = 1 - r;

  // normal
  n[0] = cosf(az) * scale;
  n[1] = sinf(az) * scale;
  n[2] = scale;
}


// make open cone
static void cone(int nSlice, int nStack) {
  float az1, az2, r1, r2;
  float v1[3], v2[3], v3[3], v4[3];
  float n1[3], n2[3], n3[3], n4[3];

  // pole: use triangles
  glBegin(GL_TRIANGLES);
  r1 = 1.0f / (float)nStack;
  for (int j=0; j < nSlice; j++) {
    az1 = (2.0f*mjPI * (j+0)) / (float)nSlice;
    az2 = (2.0f*mjPI * (j+1)) / (float)nSlice;

    // compute triangle vertices
    setVertexCone(v1, n1, az1, r1);
    setVertexCone(v2, n2, az2, r1);
    v3[0] = v3[1] = 0;
    v3[2] = 1;
    n3[0] = n1[0]+n2[0];
    n3[1] = n1[1]+n2[1];
    n3[2] = n1[2]+n2[2];
    mjr_normalizeVec(n3);

    // make triangle
    glNormal3fv(n1);
    glVertex3fv(v1);
    glNormal3fv(n2);
    glVertex3fv(v2);
    glNormal3fv(n3);
    glVertex3fv(v3);
  }
  glEnd();

  // the rest: use quads
  glBegin(GL_QUADS);
  for (int i=0; i < nStack-1; i++) {
    r1 = (i+1) / (float)nStack;
    r2 = (i+2) / (float)nStack;

    for (int j=0; j < nSlice; j++) {
      az1 = (2.0f*mjPI * (j+0)) / (float)nSlice;
      az2 = (2.0f*mjPI * (j+1)) / (float)nSlice;

      // compute quad vertices
      setVertexCone(v1, n1, az1, r2);
      setVertexCone(v2, n2, az2, r2);
      setVertexCone(v3, n3, az2, r1);
      setVertexCone(v4, n4, az1, r1);

      // make quad
      glNormal3fv(n1);
      glVertex3fv(v1);
      glNormal3fv(n2);
      glVertex3fv(v2);
      glNormal3fv(n3);
      glVertex3fv(v3);
      glNormal3fv(n4);
      glVertex3fv(v4);
    }
  }
  glEnd();
}



// set one vertex and normal on cylinder, given az and h
static void setVertexCylinder(float* v, float* n, float az, float h) {
  v[0] = cosf(az);
  v[1] = sinf(az);
  v[2] = h;

  n[0] = v[0]/sqrtf(v[0]*v[0]+v[1]*v[1]);
  n[1] = v[1]/sqrtf(v[0]*v[0]+v[1]*v[1]);
  n[2] = 0;
}


// open cylinder from -1 to +1 in z, radius 1
static void cylinder(int nSlice, int nStack) {
  float az1, az2, h1, h2;
  float v1[3], v2[3], v3[3], v4[3];
  float n1[3], n2[3], n3[3], n4[3];

  // use quads everywhere
  glBegin(GL_QUADS);
  for (int i=0; i < nStack; i++) {
    h1 = 2*(i+0)/(float)nStack - 1;
    h2 = 2*(i+1)/(float)nStack - 1;

    for (int j=0; j < nSlice; j++) {
      az1 = (2.0f*mjPI * (j+0)) / (float)nSlice;
      az2 = (2.0f*mjPI * (j+1)) / (float)nSlice;

      // compute quad vertices
      setVertexCylinder(v1, n1, az1, h1);
      setVertexCylinder(v2, n2, az2, h1);
      setVertexCylinder(v3, n3, az2, h2);
      setVertexCylinder(v4, n4, az1, h2);

      // make quad
      glNormal3fv(n1);
      glVertex3fv(v1);
      glNormal3fv(n2);
      glVertex3fv(v2);
      glNormal3fv(n3);
      glVertex3fv(v3);
      glNormal3fv(n4);
      glVertex3fv(v4);
    }
  }
  glEnd();
}



// set one vertex in haze
static void setVertexHaze(float* v, float az, float h, float r) {
  v[0] = cosf(az) * (1 - r*(1-h));
  v[1] = sinf(az) * (1 - r*(1-h));
  v[2] = h;
}


// truncated cone for haze rendering
static void haze(int nSlice, float r, const float* rgba) {
  // compute elevation h for transparency transition point
  float alpha = atan2f(1, r);
  float beta = (float)(0.75*mjPI) - alpha;
  float h = sqrtf(0.5f) * r * sinf(alpha) / sinf(beta);

  // use quads everywhere
  glBegin(GL_QUADS);

  // normal not needed (always rendered with lighting off)
  glNormal3f(0, 0, 1);

  // stacks = 2
  for (int i=0; i < 2; i++) {
    float h1 = (i == 0 ? 0 : h);
    float h2 = (i == 0 ? h : 1);

    for (int j=0; j < nSlice; j++) {
      float az1 = (2.0f*mjPI * (j+0)) / (float)nSlice;
      float az2 = (2.0f*mjPI * (j+1)) / (float)nSlice;

      // compute quad vertices
      float v1[3], v2[3], v3[3], v4[3];
      setVertexHaze(v1, az1, h1, r);
      setVertexHaze(v2, az2, h1, r);
      setVertexHaze(v3, az2, h2, r);
      setVertexHaze(v4, az1, h2, r);

      // colors at elevation h1 and h2
      float c1 = (i == 1);
      float c2 = (i == 0);

      // make quad, with colors
      glColor4f(rgba[0], rgba[1], rgba[2], c1);
      glVertex3fv(v1);
      glVertex3fv(v2);
      glColor4f(rgba[0], rgba[1], rgba[2], c2);
      glVertex3fv(v3);
      glVertex3fv(v4);
    }
  }
  glEnd();
}



// builtin geoms types
static void makeBuiltin(const mjModel* m, mjrContext* con) {
  int numstacks = m->vis.quality.numstacks;
  int numslices = m->vis.quality.numslices;
  int numquads  = m->vis.quality.numquads;
  float d = 2.0f/numquads;

  // allocate list
  listAllocate(&con->baseBuiltin, &con->rangeBuiltin, mjrNUM);

  // sphere
  glNewList(con->baseBuiltin + mjrSPHERE, GL_COMPILE);
  sphere(numslices, numstacks);
  glEndList();

  // sphere top
  glNewList(con->baseBuiltin + mjrSPHERETOP, GL_COMPILE);
  halfSphere(+1, numslices, numstacks/2);
  glEndList();

  // sphere bottom
  glNewList(con->baseBuiltin + mjrSPHEREBOTTOM, GL_COMPILE);
  halfSphere(-1, numslices, numstacks/2);
  glEndList();

  // closed cylinder, center at z=0
  glNewList(con->baseBuiltin + mjrCYLINDER, GL_COMPILE);
  disk(-1, numslices, numstacks/2);
  cylinder(numslices, numstacks);
  disk(+1, numslices, numstacks/2);
  glEndList();

  // open cylinder, center at z=0
  glNewList(con->baseBuiltin + mjrCYLINDEROPEN, GL_COMPILE);
  cylinder(numslices, numstacks);
  glEndList();

  // haze truncated cone
  glNewList(con->baseBuiltin + mjrHAZE, GL_COMPILE);
  haze(numslices, m->vis.map.haze, m->vis.rgba.haze);
  glEndList();

  // box
  glNewList(con->baseBuiltin + mjrBOX, GL_COMPILE);
  glBegin(GL_QUADS);
  for (int x=0; x < numquads; x++) {
    for (int y=0; y < numquads; y++) {
      glNormal3f(0, 0, 1);                        // top
      glVertex3f(d*(x+0)-1, d*(y+0)-1, 1);
      glVertex3f(d*(x+1)-1, d*(y+0)-1, 1);
      glVertex3f(d*(x+1)-1, d*(y+1)-1, 1);
      glVertex3f(d*(x+0)-1, d*(y+1)-1, 1);

      glNormal3f(0, 0, -1);                       // bottom
      glVertex3f(d*(x+0)-1, d*(y+1)-1, -1);
      glVertex3f(d*(x+1)-1, d*(y+1)-1, -1);
      glVertex3f(d*(x+1)-1, d*(y+0)-1, -1);
      glVertex3f(d*(x+0)-1, d*(y+0)-1, -1);

      glNormal3f(1, 0, 0);                        // right
      glVertex3f(1, d*(x+0)-1, d*(y+0)-1);
      glVertex3f(1, d*(x+1)-1, d*(y+0)-1);
      glVertex3f(1, d*(x+1)-1, d*(y+1)-1);
      glVertex3f(1, d*(x+0)-1, d*(y+1)-1);

      glNormal3f(-1, 0, 0);                       // left
      glVertex3f(-1, d*(x+0)-1, d*(y+1)-1);
      glVertex3f(-1, d*(x+1)-1, d*(y+1)-1);
      glVertex3f(-1, d*(x+1)-1, d*(y+0)-1);
      glVertex3f(-1, d*(x+0)-1, d*(y+0)-1);

      glNormal3f(0, -1, 0);                       // front
      glVertex3f(d*(x+0)-1, -1, d*(y+0)-1);
      glVertex3f(d*(x+1)-1, -1, d*(y+0)-1);
      glVertex3f(d*(x+1)-1, -1, d*(y+1)-1);
      glVertex3f(d*(x+0)-1, -1, d*(y+1)-1);

      glNormal3f(0, 1, 0);                        // back
      glVertex3f(d*(x+0)-1, 1, d*(y+1)-1);
      glVertex3f(d*(x+1)-1, 1, d*(y+1)-1);
      glVertex3f(d*(x+1)-1, 1, d*(y+0)-1);
      glVertex3f(d*(x+0)-1, 1, d*(y+0)-1);
    }
  }
  glEnd();
  glEndList();

  // cone, bottom at z=0
  glNewList(con->baseBuiltin + mjrCONE, GL_COMPILE);
  cone(numslices, numstacks);
  disk(0, numslices, numstacks);
  glEndList();
}



// make depth texture and FBO for shadow mapping
static void makeShadow(const mjModel* m, mjrContext* con) {
  // return if size is 0
  if (!con->shadowSize) {
    return;
  }

  // create FBO
  glGenFramebuffers(1, &con->shadowFBO);
  if (!con->shadowFBO) {
    mju_error("Could not allocate shadow framebuffer");
  }
  glBindFramebuffer(GL_FRAMEBUFFER, con->shadowFBO);

  // Create a shadow depth texture in TEXTURE1 and explicitly select an int24
  // depth buffer. A depth stencil format is used because that appears to be
  // more widely supported (MacOS does not support GL_DEPTH_COMPONENT24). Using
  // a fixed format makes it easier to choose glPolygonOffset parameters that
  // result in reasonably consistent and artifact free shadows across platforms.
  glGenTextures(1, &con->shadowTex);
  glActiveTexture(GL_TEXTURE1);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, con->shadowTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8,
               con->shadowSize, con->shadowSize, 0, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_GEQUAL);
  glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);
  glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
  glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);

  // attach to FBO
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, con->shadowTex, 0);
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);

  // check FBO status
  GLenum err = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (err != GL_FRAMEBUFFER_COMPLETE) {
    mju_error("Shadow framebuffer is not complete, error 0x%x", err);
  }

  glDisable(GL_TEXTURE_2D);
}



// make render buffers and FBO for offscreen rendering
static void makeOff(mjrContext* con) {
  // return if size is 0
  if (!con->offWidth || !con->offHeight) {
    return;
  }

  // create FBO
  glGenFramebuffers(1, &con->offFBO);
  if (!con->offFBO) {
    mju_error("Could not allocate offscreen framebuffer");
  }
  glBindFramebuffer(GL_FRAMEBUFFER, con->offFBO);

  // clamp samples request
  int sMax = 0;
  glGetIntegerv(GL_MAX_SAMPLES, &sMax);
  if (con->offSamples > sMax) {
    con->offSamples = sMax;
  }

  // create color buffer
  glGenRenderbuffers(1, &con->offColor);
  if (!con->offColor) {
    mju_error("Could not allocate offscreen color buffer");
  }
  glBindRenderbuffer(GL_RENDERBUFFER, con->offColor);
  if (con->offSamples) {
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, con->offSamples, GL_RGBA8,
                                     con->offWidth, con->offHeight);
  } else {
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, con->offWidth, con->offHeight);
  }
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, con->offColor);

  // create depth and stencil buffer
  glGenRenderbuffers(1, &con->offDepthStencil);
  if (!con->offDepthStencil) {
    mju_error("Could not allocate offscreen depth and stencil buffer");
  }
  glBindRenderbuffer(GL_RENDERBUFFER, con->offDepthStencil);

  GLenum depth_buffer_format =
      mjGLAD_GL_ARB_depth_buffer_float ? GL_DEPTH32F_STENCIL8 : GL_DEPTH24_STENCIL8;
  if (con->offSamples) {
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, con->offSamples, depth_buffer_format,
                                     con->offWidth, con->offHeight);
  } else {
    glRenderbufferStorage(GL_RENDERBUFFER, depth_buffer_format, con->offWidth, con->offHeight);
  }
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                            GL_RENDERBUFFER, con->offDepthStencil);

  // check FBO status
  GLenum err = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (err != GL_FRAMEBUFFER_COMPLETE) {
    mju_error("Offscreen framebuffer is not complete, error 0x%x", err);
  }

  // get actual number of samples
  glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_SAMPLES, &con->offSamples);

  // create FBO for resolving multisamples
  if (con->offSamples) {
    // create FBO
    glGenFramebuffers(1, &con->offFBO_r);
    if (!con->offFBO_r) {
      mju_error("Could not allocate offscreen framebuffer_r");
    }
    glBindFramebuffer(GL_FRAMEBUFFER, con->offFBO_r);

    // create color buffer
    glGenRenderbuffers(1, &con->offColor_r);
    if (!con->offColor_r) {
      mju_error("Could not allocate offscreen color buffer_r");
    }
    glBindRenderbuffer(GL_RENDERBUFFER, con->offColor_r);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, con->offWidth, con->offHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                              GL_RENDERBUFFER, con->offColor_r);

    // create depth and stencil buffer
    glGenRenderbuffers(1, &con->offDepthStencil_r);
    if (!con->offDepthStencil_r) {
      mju_error("Could not allocate offscreen depth and stencil buffer_r");
    }
    glBindRenderbuffer(GL_RENDERBUFFER, con->offDepthStencil_r);
    glRenderbufferStorage(GL_RENDERBUFFER, depth_buffer_format, con->offWidth, con->offHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                              GL_RENDERBUFFER, con->offDepthStencil_r);

    // check FBO status
    GLenum err = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (err != GL_FRAMEBUFFER_COMPLETE) {
      mju_error("Offscreen framebuffer_r is not complete, error 0x%x", err);
    }
  }
}



// make fonts
static void makeFont(mjrContext* con, int fontscale) {
  // data addresses
  int adr = 0, adr_big = 0;

  // font data pointers for given scaling
  const unsigned char* font_normal = NULL;
  const unsigned char* font_back = NULL;
  const unsigned char* font_big = NULL;
  switch (fontscale) {
  case mjFONTSCALE_50:
    font_normal = font_normal50;
    font_back = font_back50;
    font_big = font_big50;
    break;

  case mjFONTSCALE_100:
    font_normal = font_normal100;
    font_back = font_back100;
    font_big = font_big100;
    break;

  case mjFONTSCALE_150:
    font_normal = font_normal150;
    font_back = font_back150;
    font_big = font_big150;
    break;

  case mjFONTSCALE_200:
    font_normal = font_normal200;
    font_back = font_back200;
    font_big = font_big200;
    break;

  case mjFONTSCALE_250:
    font_normal = font_normal250;
    font_back = font_back250;
    font_big = font_big250;
    break;

  case mjFONTSCALE_300:
    font_normal = font_normal300;
    font_back = font_back300;
    font_big = font_big300;
    break;

  default:
    mju_error("Invalid fontscale");
  }

  // save fontScale
  con->fontScale = fontscale;

  // init lists
  con->rangeFont = 128;
  con->baseFontNormal = glGenLists(con->rangeFont);
  con->baseFontShadow = glGenLists(con->rangeFont);
  con->baseFontBig = glGenLists(con->rangeFont);
  if (con->baseFontNormal == 0 || con->baseFontShadow == 0 || con->baseFontBig == 0) {
    mju_error("Could not allocate font lists");
  }

  // loop over printable characters (32-126)
  for (unsigned char i=32; i <= 126; i++) {
    // assert character code; SHOULD NOT OCCUR
    if (font_normal[adr] != i || font_back[adr] != i || font_big[adr_big] != i) {
      mju_error("Invalid font data index");
    }

    // save character sizes
    con->charWidth[i] = font_normal[adr+1];
    con->charWidthBig[i] = font_big[adr_big+1];
    con->charHeight = font_normal[adr+2];
    con->charHeightBig = font_big[adr_big+2];

    // make bitmap display list: normal
    glNewList(con->baseFontNormal+i, GL_COMPILE);
    glBitmap(con->charWidth[i], con->charHeight, 0, 0, con->charWidth[i], 0,
             font_normal + adr + 3);
    glEndList();

    // make bitmap display list: back
    glNewList(con->baseFontShadow+i, GL_COMPILE);
    glBitmap(con->charWidth[i], con->charHeight, 0, 0, con->charWidth[i], 0,
             font_back + adr + 3);
    glEndList();

    // make bitmap display list: big
    glNewList(con->baseFontBig+i, GL_COMPILE);
    glBitmap(con->charWidthBig[i], con->charHeightBig, 0, 0, con->charWidthBig[i], 0,
             font_big + adr_big + 3);
    glEndList();

    // compute width in bytes
    int width = (con->charWidth[i]-1)/8 + 1;
    int widthBig = (con->charWidthBig[i]-1)/8 + 1;

    // advance addresses
    adr += 3 + width*con->charHeight;
    adr_big += 3 + widthBig*con->charHeightBig;
  }

  // assert 123 termination token; SHOULD NOT OCCUR
  if (font_normal[adr] != 123 || font_back[adr] != 123 || font_big[adr_big] != 123) {
    mju_error("Invalid font data termination");
  }
}

// make materials, just for those that have textures
static void makeMaterial(const mjModel* m, mjrContext* con) {
  memset(con->mat_texid, -1, sizeof(con->mat_texid));
  memset(con->mat_texuniform, 0, sizeof(con->mat_texuniform));
  memset(con->mat_texrepeat, 0, sizeof(con->mat_texrepeat));
  if (!m->nmat || !m->ntex) {
    return;
  }

  if (m->nmat >= mjMAXMATERIAL-1) {
    mju_error("Maximum number of materials is %d, got %d", mjMAXMATERIAL, m->nmat);
  }
  for (int i=0; i < m->nmat; i++) {
    if (m->mat_texid[i*mjNTEXROLE + mjTEXROLE_RGB] >= 0) {
      for (int j=0; j < mjNTEXROLE; j++) {
        con->mat_texid[i*mjNTEXROLE + j] = m->mat_texid[i*mjNTEXROLE + j];
      }
      con->mat_texuniform[i] = m->mat_texuniform[i];
      con->mat_texrepeat[2*i] = m->mat_texrepeat[2*i];
      con->mat_texrepeat[2*i+1] = m->mat_texrepeat[2*i+1];
    }
  }
  // find skybox texture
  for (int i=0; i < m->ntex; i++) {
    if (m->tex_type[i] == mjTEXTURE_SKYBOX) {
      if (m->nmat >= mjMAXMATERIAL-2) {
        mju_error("With skybox, maximum number of materials is %d, got %d",
                  mjMAXMATERIAL-1, m->nmat);
      }
      for (int j=0; j < mjNTEXROLE; j++) {
        con->mat_texid[mjNTEXROLE * (mjMAXMATERIAL-1) + j] = -1;
      }
      con->mat_texid[mjNTEXROLE * (mjMAXMATERIAL-1) + mjTEXROLE_RGB] = i;

      break;
    }
  }
}


// make textures
static void makeTexture(const mjModel* m, mjrContext* con) {
  // checks size
  if (m->ntex > mjMAXTEXTURE) {
    mju_error("Maximum number of textures is %d", mjMAXTEXTURE);
  }

  // save new size
  con->ntexture = m->ntex;
  if (!m->ntex) {
    return;
  }

  // allocate and upload
  glGenTextures(con->ntexture, con->texture);
  for (int i=0; i < m->ntex; i++) {
    con->textureType[i] = m->tex_type[i];
    mjr_uploadTexture(m, con, i);
  }
}



// (re) upload texture to GPU
void mjr_uploadTexture(const mjModel* m, const mjrContext* con, int texid) {
  int w = m->tex_width[texid];
  float plane[4];

  // 2D texture
  if (m->tex_type[texid] == mjTEXTURE_2D) {
    // OpenGL settings
    glActiveTexture(GL_TEXTURE0);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, con->texture[texid]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

    // set mapping
    mjr_setf4(plane, 1, 0, 0, 0);
    glTexGenfv(GL_S, GL_OBJECT_PLANE, plane);
    mjr_setf4(plane, 0, 1, 0, 0);
    glTexGenfv(GL_T, GL_OBJECT_PLANE, plane);

    // assign data
    int type = 0;
    int internaltype = 0;
    if (m->tex_nchannel[texid] == 3) {
      type = GL_RGB;
      internaltype = (m->tex_colorspace[texid] == mjCOLORSPACE_SRGB) ? GL_SRGB8_EXT : GL_RGB;
    } else if (m->tex_nchannel[texid] == 4) {
      type = GL_RGBA;
      internaltype = (m->tex_colorspace[texid] == mjCOLORSPACE_SRGB) ? GL_SRGB8_ALPHA8_EXT : GL_RGBA;
    } else {
      mju_error("Number of channels not supported: %d", m->tex_nchannel[texid]);
    }

    glTexImage2D(GL_TEXTURE_2D, 0, internaltype, m->tex_width[texid],
                 m->tex_height[texid], 0, type, GL_UNSIGNED_BYTE,
                 m->tex_data + m->tex_adr[texid]);

    // generate mipmaps
    glGenerateMipmap(GL_TEXTURE_2D);
    glDisable(GL_TEXTURE_2D);
  }

  // cube or skybox texture
  else {
    // OpenGL settings
    glActiveTexture(GL_TEXTURE0);
    glEnable(GL_TEXTURE_CUBE_MAP);
    glBindTexture(GL_TEXTURE_CUBE_MAP, con->texture[texid]);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
    glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

    // set mapping
    mjr_setf4(plane, 1, 0, 0, 0);
    glTexGenfv(GL_S, GL_OBJECT_PLANE, plane);
    mjr_setf4(plane, 0, 1, 0, 0);
    glTexGenfv(GL_T, GL_OBJECT_PLANE, plane);
    mjr_setf4(plane, 0, 0, 1, 0);
    glTexGenfv(GL_R, GL_OBJECT_PLANE, plane);

    // assign data: repeated
    if (m->tex_width[texid] == m->tex_height[texid]) {
      for (int i=0; i < 6; i++) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+i, 0, GL_RGB, w, w, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, m->tex_data + m->tex_adr[texid]);
      }
    }

    // assign data: separate faces
    else {
      for (int i=0; i < 6; i++) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+i, 0, GL_RGB, w, w, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, m->tex_data + m->tex_adr[texid] + i*3*w*w);
      }
    }

    // generate mipmaps
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
    glDisable(GL_TEXTURE_CUBE_MAP);
  }
}



// allocate buffers for skins, copy constants
static void makeSkin(const mjModel* m, mjrContext* con) {
  int nskin = m->nskin;

  // save number of skins (so we can delete context without model)
  con->nskin = m->nskin;

  // allocate buffers
  if (nskin) {
    // allocate VBO names
    con->skinvertVBO     = (unsigned int*) mju_malloc(nskin*sizeof(int));
    con->skinnormalVBO   = (unsigned int*) mju_malloc(nskin*sizeof(int));
    con->skintexcoordVBO = (unsigned int*) mju_malloc(nskin*sizeof(int));
    con->skinfaceVBO     = (unsigned int*) mju_malloc(nskin*sizeof(int));

    // generage VBOs
    glGenBuffers(nskin, con->skinvertVBO);
    glGenBuffers(nskin, con->skinnormalVBO);
    glGenBuffers(nskin, con->skintexcoordVBO);
    glGenBuffers(nskin, con->skinfaceVBO);

    // process skins
    for (int i=0; i < nskin; i++) {
      // texture coordinates
      if (m->skin_texcoordadr[i] >= 0) {
        glBindBuffer(GL_ARRAY_BUFFER, con->skintexcoordVBO[i]);
        glBufferData(GL_ARRAY_BUFFER,
                     2*m->skin_vertnum[i]*sizeof(float),
                     m->skin_texcoord + 2*m->skin_texcoordadr[i],
                     GL_STATIC_DRAW);
      } else {
        glDeleteBuffers(1, con->skintexcoordVBO+i);
        con->skintexcoordVBO[i] = 0;
      }

      // face indices
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, con->skinfaceVBO[i]);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                   3*m->skin_facenum[i]*sizeof(int),
                   m->skin_face + 3*m->skin_faceadr[i],
                   GL_STATIC_DRAW);
    }
  }
}



// callback to print out GL_DEBUG output (when enabled for internal testing)
void GLAPIENTRY debugCallback(GLenum source,
                              GLenum type,
                              GLuint id,
                              GLenum severity,
                              GLsizei length,
                              const GLchar* message,
                              const void* userParam) {
  printf("GL DEBUG: source = 0x%x, type = 0x%x, severity = 0x%x, id = 0x%x\nmessage = %s\n\n",
         source, type, severity, id, message);
}



// returns 1 if MUJOCO_GL_DEBUG environment variable is set to 1
static int glDebugEnabled(void) {
  char* debug = getenv("MUJOCO_GL_DEBUG");
  return debug && strcmp(debug, "1") == 0;
}



void mjr_makeContext_offSize(const mjModel* m, mjrContext* con, int fontscale,
                             int default_offwidth, int default_offheight) {
  // fix fontscale
  fontscale = 50 * mju_round(((mjtNum)fontscale)/50.0);
  if (fontscale < 100) {
    fontscale = 100;
  } else if (fontscale > 300) {
    fontscale = 300;
  }

  // initialize GLAD, determine window and FBO availability
  if (!con->glInitialized) {
    if (!mjGladLoadGL()) {
      mju_error("gladLoadGL error");
    }
    if (!mjGLAD_GL_VERSION_1_5) {
      mju_error("OpenGL version 1.5 or higher required");
    }
    if (!mjGLAD_GL_ARB_framebuffer_object) {
      mju_error("OpenGL ARB_framebuffer_object required");
    }
    if (!mjGLAD_GL_ARB_vertex_buffer_object) {
      mju_error("OpenGL ARB_vertex_buffer_object required");
    }
    con->glInitialized = 1;

    // determine window availability (could be EGL-headless)
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    unsigned int status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status == GL_FRAMEBUFFER_COMPLETE) {
      con->windowAvailable = 1;
    } else if (status == GL_FRAMEBUFFER_UNDEFINED) {
      con->windowAvailable = 0;
    } else {
      mju_error("Default framebuffer is not complete, error 0x%x", status);
    }
  }

  // OpenGL debug output
  if (glDebugEnabled() && mjGLAD_GL_KHR_debug) {
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(debugCallback, 0);
  }

  // determine samples, stereo and doublebuffer if window available
  if (con->windowAvailable) {
    // get stereo
    GLboolean b;
    glGetBooleanv(GL_STEREO, &b);
    con->windowStereo = (int)b;

    // get doublebuffer
    glGetBooleanv(GL_DOUBLEBUFFER, &b);
    con->windowDoublebuffer = (int)b;

    // get samples
    GLint n;
    glGetIntegerv(GL_SAMPLE_BUFFERS, &n);
    if (n) {
      glGetIntegerv(GL_SAMPLES, &n);
      con->windowSamples = (int)n;
    } else {
      con->windowSamples = 0;
    }
  }

  // set pixel (un)packing
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  mjr_freeContext(con);

  // no model: offscreen and font only
  if (!m) {
    // default offscreen
    con->offWidth = default_offwidth;
    con->offHeight = default_offheight;
    con->offSamples = 0;
    makeOff(con);

    // font
    makeFont(con, fontscale);

    // try to bind window (bind offscreen if no window)
    mjr_setBuffer(mjFB_WINDOW, con);

    return;
  }

  // map shadow clip and scale to absolute units
  con->shadowClip = m->stat.extent * m->vis.map.shadowclip;
  con->shadowScale = m->vis.map.shadowscale;

  // copy parameters
  con->offWidth = m->vis.global.offwidth;
  con->offHeight = m->vis.global.offheight;
  con->offSamples = m->vis.quality.offsamples;
  con->fogStart = (float)(m->stat.extent * m->vis.map.fogstart);
  con->fogEnd = (float)(m->stat.extent * m->vis.map.fogend);
  con->fogRGBA[0] = m->vis.rgba.fog[0];
  con->fogRGBA[1] = m->vis.rgba.fog[1];
  con->fogRGBA[2] = m->vis.rgba.fog[2];
  con->fogRGBA[3] = m->vis.rgba.fog[3];
  con->lineWidth = m->vis.global.linewidth;
  con->shadowSize = m->vis.quality.shadowsize;

  // set fog parameters (model-dependent)
  glFogi(GL_FOG_MODE, GL_LINEAR);
  glFogf(GL_FOG_START, con->fogStart);
  glFogf(GL_FOG_END, con->fogEnd);
  glFogfv(GL_FOG_COLOR, con->fogRGBA);
  glFogi(GL_FOG_COORD_SRC, GL_FRAGMENT_DEPTH);
  glHint(GL_FOG_HINT, GL_NICEST);

  // make everything
  makeOff(con);
  makeShadow(m, con);
  makeMaterial(m, con);
  makeTexture(m, con);
  makePlane(m, con);
  makeMesh(m, con);
  makeHField(m, con);
  makeBuiltin(m, con);
  makeSkin(m, con);
  makeFont(con, fontscale);

  // enable seamless cube maps if supported
  if (mjGLAD_GL_ARB_seamless_cube_map) {
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
  }

  // try to bind window (bind offscreen if no window)
  mjr_setBuffer(mjFB_WINDOW, con);

  // issue warnings for any OpenGL errors
  GLenum err;
  while ((err = glGetError())) {
    mju_warning("OpenGL error 0x%x in or before mjr_makeContext", err);
  }

  // set default color pixel format for mjr_readPixels
  con->readPixelFormat = GL_RGB;

  // set default depth mapping for mjr_readPixels
  con->readDepthMap = mjDEPTH_ZERONEAR;
}



// allocate resources in custom OpenGL context
void mjr_makeContext(const mjModel* m, mjrContext* con, int fontscale) {
  mjr_makeContext_offSize(m, con, fontscale, 800, 600);
}



// Change font of existing context.
void mjr_changeFont(int fontscale, mjrContext* con) {
  // free existing font
  if (con->rangeFont) {
    glDeleteLists(con->baseFontNormal, con->rangeFont);
    glDeleteLists(con->baseFontShadow, con->rangeFont);
    glDeleteLists(con->baseFontBig, con->rangeFont);
  }
  con->baseFontNormal = 0;
  con->baseFontShadow = 0;
  con->baseFontBig = 0;
  con->rangeFont = 0;

  // make new font
  makeFont(con, fontscale);
}



// Add Aux buffer to context; free previous Aux buffer.
void mjr_addAux(int index, int width, int height, int samples, mjrContext* con) {
  // check index
  if (index < 0 || index >= mjNAUX) {
    mju_error("Invalid aux buffer index");
  }

  // free previous
  if (con->auxColor[index]) {
    glDeleteRenderbuffers(1, con->auxColor + index);
  }
  if (con->auxColor_r[index]) {
    glDeleteRenderbuffers(1, con->auxColor_r + index);
  }
  if (con->auxFBO[index]) {
    glDeleteFramebuffers(1, con->auxFBO + index);
  }
  if (con->auxFBO_r[index]) {
    glDeleteFramebuffers(1, con->auxFBO_r + index);
  }
  con->auxColor[index] = 0;
  con->auxColor_r[index] = 0;
  con->auxFBO[index] = 0;
  con->auxFBO_r[index] = 0;

  // return if size is not positive
  if (width < 1 || height < 1) {
    return;
  }

  // check max size
  int maxSize = 0;
  glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &maxSize);
  if (width > maxSize) {
    mju_error(
        "Auxiliary buffer width exceeds maximum allowed by OpenGL "
        "implementation: %d > %d",
        width, maxSize);
  }
  if (height > maxSize) {
    mju_error(
        "Auxiliary buffer height exceeds maximum allowed by OpenGL "
        "implementation: %d > %d",
        height, maxSize);
  }

  // clamp samples request
  int maxSample = 0;
  glGetIntegerv(GL_MAX_SAMPLES, &maxSample);
  if (samples > maxSample) {
    samples = maxSample;
  }

  // assign sizes
  con->auxWidth[index] = width;
  con->auxHeight[index] = height;
  con->auxSamples[index] = samples;

  // create FBO
  glGenFramebuffers(1, con->auxFBO + index);
  if (!con->auxFBO[index]) {
    mju_error("Could not allocate auxiliary framebuffer");
  }
  glBindFramebuffer(GL_FRAMEBUFFER, con->auxFBO[index]);

  // create color buffer with multisamples
  glGenRenderbuffers(1, con->auxColor + index);
  if (!con->auxColor[index]) {
    mju_error("Could not allocate auxiliary color buffer");
  }
  glBindRenderbuffer(GL_RENDERBUFFER, con->auxColor[index]);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, con->auxSamples[index], GL_RGBA8,
                                   con->auxWidth[index], con->auxHeight[index]);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER, con->auxColor[index]);

  // check FBO status
  GLenum err = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (err != GL_FRAMEBUFFER_COMPLETE) {
    mju_error("Auxiliary framebuffer is not complete, error 0x%x", err);
  }

  // create FBO for resolving
  glGenFramebuffers(1, con->auxFBO_r + index);
  if (!con->auxFBO_r[index]) {
    mju_error("Could not allocate auxiliary resolve framebuffer");
  }
  glBindFramebuffer(GL_FRAMEBUFFER, con->auxFBO_r[index]);

  // create color buffer for resolving multisamples
  glGenRenderbuffers(1, con->auxColor_r + index);
  if (!con->auxColor_r[index]) {
    mju_error("Could not allocate auxiliary color resolve buffer");
  }
  glBindRenderbuffer(GL_RENDERBUFFER, con->auxColor_r[index]);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8,
                        con->auxWidth[index], con->auxHeight[index]);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER, con->auxColor_r[index]);

  // check FBO status
  err = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (err != GL_FRAMEBUFFER_COMPLETE) {
    mju_error("Auxiliary framebuffer resolve is not complete, error 0x%x", err);
  }

  // restore
  mjr_restoreBuffer(con);
}



// free resources in custom OpenGL context
void mjr_freeContext(mjrContext* con) {
  // save flags
  int glInitialized = con->glInitialized;
  int windowAvailable = con->windowAvailable;
  int windowSamples = con->windowSamples;
  int windowStereo = con->windowStereo;
  int windowDoublebuffer = con->windowDoublebuffer;

  // free GPU resources
  if (con->ntexture) glDeleteTextures(con->ntexture, con->texture);
  if (con->offColor) glDeleteRenderbuffers(1, &con->offColor);
  if (con->offColor_r) glDeleteRenderbuffers(1, &con->offColor_r);
  if (con->offDepthStencil) glDeleteRenderbuffers(1, &con->offDepthStencil);
  if (con->offDepthStencil_r) glDeleteRenderbuffers(1, &con->offDepthStencil_r);
  if (con->offFBO) glDeleteFramebuffers(1, &con->offFBO);
  if (con->offFBO_r) glDeleteFramebuffers(1, &con->offFBO_r);
  if (con->shadowTex) glDeleteTextures(1, &con->shadowTex);
  if (con->shadowFBO) glDeleteFramebuffers(1, &con->shadowFBO);
  for (int i=0; i < mjNAUX; i++) {
    if (con->auxColor[i]) glDeleteRenderbuffers(1, con->auxColor + i);
    if (con->auxColor_r[i]) glDeleteRenderbuffers(1, con->auxColor_r + i);
    if (con->auxFBO[i]) glDeleteFramebuffers(1, con->auxFBO + i);
    if (con->auxFBO_r[i]) glDeleteFramebuffers(1, con->auxFBO_r + i);
  }
  if (con->rangePlane) glDeleteLists(con->basePlane, con->rangePlane);
  if (con->rangeMesh) glDeleteLists(con->baseMesh, con->rangeMesh);
  if (con->rangeHField) glDeleteLists(con->baseHField, con->rangeHField);
  if (con->rangeBuiltin) glDeleteLists(con->baseBuiltin, con->rangeBuiltin);
  if (con->rangeFont) {
    glDeleteLists(con->baseFontNormal, con->rangeFont);
    glDeleteLists(con->baseFontShadow, con->rangeFont);
    glDeleteLists(con->baseFontBig, con->rangeFont);
  }

  // delete skin
  if (con->nskin) {
    // delete VBOs
    glDeleteBuffers(con->nskin, con->skinvertVBO);
    glDeleteBuffers(con->nskin, con->skinnormalVBO);
    glDeleteBuffers(con->nskin, con->skintexcoordVBO);
    glDeleteBuffers(con->nskin, con->skinfaceVBO);

    mju_free(con->skinvertVBO);
    mju_free(con->skinnormalVBO);
    mju_free(con->skintexcoordVBO);
    mju_free(con->skinfaceVBO);
  }

  // clear fields
  mjr_defaultContext(con);

  // restore flags
  con->glInitialized = glInitialized;
  con->windowAvailable = windowAvailable;
  con->windowSamples = windowSamples;
  con->windowStereo = windowStereo;
  con->windowDoublebuffer = windowDoublebuffer;
}



// resize offscreen buffers
void mjr_resizeOffscreen(int width, int height, mjrContext* con) {
  if (con->offWidth == width && con->offHeight == height) {
    return;
  }

  con->offWidth = width;
  con->offHeight = height;

  if (!width || !height) {
    return;
  }

  if (!con->offFBO) {
    makeOff(con);
    return;
  }

  glBindRenderbuffer(GL_RENDERBUFFER, con->offColor);
  if (con->offSamples) {
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, con->offSamples, GL_RGBA8,
                                     con->offWidth, con->offHeight);
  } else {
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, con->offWidth, con->offHeight);
  }

  glBindRenderbuffer(GL_RENDERBUFFER, con->offDepthStencil);
  if (con->offSamples) {
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, con->offSamples, GL_DEPTH32F_STENCIL8,
                                     con->offWidth, con->offHeight);
  } else {
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, con->offWidth, con->offHeight);
  }

  if (con->offSamples) {
    glBindRenderbuffer(GL_RENDERBUFFER, con->offColor_r);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, con->offWidth, con->offHeight);

    glBindRenderbuffer(GL_RENDERBUFFER, con->offDepthStencil_r);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, con->offWidth, con->offHeight);
  }
}
