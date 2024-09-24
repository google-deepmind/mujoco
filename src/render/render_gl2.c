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

#include "render/render_gl2.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include <mujoco/mjmacro.h>
#include <mujoco/mujoco.h>
#include "engine/engine_array_safety.h"
#include "render/glad/glad.h"

#define STRING_BUFSIZE 100


//------------------------- OpenGL framebuffer operations ------------------------------------------

// set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN
//  if only one buffer is available, set that buffer and ignore framebuffer argument
void mjr_setBuffer(int framebuffer, mjrContext* con) {
  // no buffer: error
  if (!con->windowAvailable && !con->offFBO) {
    mju_error("No OpenGL framebuffer available");
  }

  // window only: set window
  else if (con->windowAvailable && !con->offFBO) {
    // bind window
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glReadBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);
    glDrawBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);

    // save
    con->currentBuffer = mjFB_WINDOW;
  }

  // offscreen only: set offscreen
  else if (!con->windowAvailable && con->offFBO) {
    // bind offscreen
    glBindFramebuffer(GL_FRAMEBUFFER, con->offFBO);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);

    // save
    con->currentBuffer = mjFB_OFFSCREEN;
  }

  // both available: use selected framebuffer
  else {
    // bind selected buffer
    if (framebuffer == mjFB_WINDOW) {
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      glReadBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);
      glDrawBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);
    } else {
      glBindFramebuffer(GL_FRAMEBUFFER, con->offFBO);
      glReadBuffer(GL_COLOR_ATTACHMENT0);
      glDrawBuffer(GL_COLOR_ATTACHMENT0);
    }

    // save
    con->currentBuffer = framebuffer;
  }
}



// make con->currentBuffer current again
void mjr_restoreBuffer(const mjrContext* con) {
  if (con->currentBuffer == mjFB_WINDOW) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glReadBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);
    glDrawBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);
  } else {
    glBindFramebuffer(GL_FRAMEBUFFER, con->offFBO);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
  }
}



// emit one warning about mjGLAD_GL_ARB_clip_control
static void warnAboutARBClipControl(void) {
  // not thread safe, but the consequence is just too many log lines
  static int warned = 0;
  if (!warned) {
    warned = 1;
    mju_warning("ARB_clip_control unavailable while mjDEPTH_ZEROFAR requested, "
                "depth accuracy will be limited");
  }
}



// emit one warning about mjGLAD_GL_ARB_depth_buffer_float
static void warnAboutARBDepthBuffer(void) {
  // not thread safe, but the consequence is just too many log lines
  static int warned = 0;
  if (!warned) {
    warned = 1;
    mju_warning("ARB_depth_buffer_float unavailable while mjDEPTH_ZEROFAR requested, "
                "depth accuracy will be limited");
  }
}



static inline void flipDepthIfRequired(float* depth, mjrRect viewport, const mjrContext* con) {
  if (con->readDepthMap == mjDEPTH_ZERONEAR) {
    int npixel = viewport.width * viewport.height;
    for (int i = 0; i < npixel; i++) {
      depth[i] = 1.0 - depth[i];  // Reverse the reversed Z buffer
    }
  } else if (!mjGLAD_GL_ARB_clip_control) {
    warnAboutARBClipControl();
  } else if (!mjGLAD_GL_ARB_depth_buffer_float) {
    warnAboutARBDepthBuffer();
  }
}



// read pixels from current OpenGL framebuffer to client buffer
//  viewport is in OpenGL framebuffer; client buffer starts at (0,0)
void mjr_readPixels(unsigned char* rgb, float* depth,
                    mjrRect viewport, const mjrContext* con) {
  // construct mask resolve-blit
  GLbitfield mask = (rgb ? GL_COLOR_BUFFER_BIT : 0) |
                    (depth ? GL_DEPTH_BUFFER_BIT : 0);

  // make sure we have something to do
  if (!mask) {
    return;
  }

  // read from window
  if (con->currentBuffer == mjFB_WINDOW) {
    // read rgb and depth
    if (rgb) {
      glReadPixels(viewport.left, viewport.bottom, viewport.width, viewport.height,
                   GL_RGB, GL_UNSIGNED_BYTE, rgb);
    }
    if (depth) {
      glReadPixels(viewport.left, viewport.bottom, viewport.width, viewport.height,
                   GL_DEPTH_COMPONENT, GL_FLOAT, depth);
      flipDepthIfRequired(depth, viewport, con);
    }
  }

  // read from offscreen
  else {
    // multisample: blit to resolve buffer and read from there
    if (con->offSamples) {
      // make sure blit is supported
      if (!glBlitFramebuffer) {
        return;
      }

      // prepare for resolve-blit
      glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO);
      glReadBuffer(GL_COLOR_ATTACHMENT0);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con->offFBO_r);
      glDrawBuffer(GL_COLOR_ATTACHMENT0);

      // resolve-blit
      glBlitFramebuffer(viewport.left, viewport.bottom,
                        viewport.left+viewport.width, viewport.bottom+viewport.height,
                        viewport.left, viewport.bottom,
                        viewport.left+viewport.width, viewport.bottom+viewport.height,
                        mask, GL_NEAREST);

      // read from resolved
      glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO_r);
      glReadBuffer(GL_COLOR_ATTACHMENT0);
    }

    // no multisample: read from offscreen
    else {
      glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO);
      glReadBuffer(GL_COLOR_ATTACHMENT0);
    }

    // read rgb and depth
    if (rgb) {
      glReadPixels(viewport.left, viewport.bottom, viewport.width, viewport.height,
                   con->readPixelFormat, GL_UNSIGNED_BYTE, rgb);
    }
    if (depth) {
      glReadPixels(viewport.left, viewport.bottom, viewport.width, viewport.height,
                   GL_DEPTH_COMPONENT, GL_FLOAT, depth);
      flipDepthIfRequired(depth, viewport, con);
    }

    // restore currentBuffer
    mjr_restoreBuffer(con);
  }
}



// draw pixels from client buffer to current OpenGL framebuffer
//  viewport is in OpenGL framebuffer; client buffer starts at (0,0)
void mjr_drawPixels(const unsigned char* rgb, const float* depth,
                    mjrRect viewport, const mjrContext* con) {
  // set raster position
  glWindowPos2i(viewport.left, viewport.bottom);

  // write rgb and depth
  if (rgb) {
    glDrawPixels(viewport.width, viewport.height, GL_RGB, GL_UNSIGNED_BYTE, rgb);
  }
  if (depth) {
    glDrawPixels(viewport.width, viewport.height, GL_DEPTH_COMPONENT, GL_FLOAT, depth);
  }
}



// blit from src viewpoint in current framebuffer to dst viewport in other framebuffer
//  if src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR
void mjr_blitBuffer(mjrRect src, mjrRect dst,
                    int flg_color, int flg_depth, const mjrContext* con) {
  // construct mask and filter for blit
  GLbitfield mask = (flg_color ? GL_COLOR_BUFFER_BIT : 0) |
                    (flg_depth ? GL_DEPTH_BUFFER_BIT : 0);
  GLenum filter = (!flg_depth && (src.width != dst.width || src.height != dst.height)) ?
                  GL_LINEAR : GL_NEAREST;

  // make sure both buffers are available and we have something to do
  if (!con->windowAvailable || !con->offFBO || !mask) {
    return;
  }

  // make sure blit is supported
  if (!glBlitFramebuffer) {
    return;
  }

  // from window to offsreen
  if (con->currentBuffer == mjFB_WINDOW) {
    // offscreen is multisample: go through resolve
    if (con->offSamples) {
      // prepare to blit from window to resolve
      glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
      glReadBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con->offFBO_r);
      glDrawBuffer(GL_COLOR_ATTACHMENT0);

      // blit
      glBlitFramebuffer(src.left, src.bottom, src.left+src.width, src.bottom+src.height,
                        dst.left, dst.bottom, dst.left+dst.width, dst.bottom+dst.height,
                        mask, filter);

      // prepare to blit from resolve to offscreen
      glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO_r);
      glReadBuffer(GL_COLOR_ATTACHMENT0);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con->offFBO);
      glDrawBuffer(GL_COLOR_ATTACHMENT0);

      // blit: both dst
      glBlitFramebuffer(dst.left, dst.bottom, dst.left+dst.width, dst.bottom+dst.height,
                        dst.left, dst.bottom, dst.left+dst.width, dst.bottom+dst.height,
                        mask, filter);
    }

    // otherwise direct
    else {
      // prepare
      glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
      glReadBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con->offFBO);
      glDrawBuffer(GL_COLOR_ATTACHMENT0);

      // blit
      glBlitFramebuffer(src.left, src.bottom, src.left+src.width, src.bottom+src.height,
                        dst.left, dst.bottom, dst.left+dst.width, dst.bottom+dst.height,
                        mask, filter);
    }
  }

  // from offscreen to window
  else {
    // offscreen is multisample: go through resolve
    if (con->offSamples) {
      // prepare to blit from offscreen to resolve
      glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO);
      glReadBuffer(GL_COLOR_ATTACHMENT0);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con->offFBO_r);
      glDrawBuffer(GL_COLOR_ATTACHMENT0);

      // blit: both src
      glBlitFramebuffer(src.left, src.bottom, src.left+src.width, src.bottom+src.height,
                        src.left, src.bottom, src.left+src.width, src.bottom+src.height,
                        mask, filter);

      // prepare to blit from resolve to window
      glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO_r);
      glReadBuffer(GL_COLOR_ATTACHMENT0);
    }

    // otherwise direct
    else {
      glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO);
      glReadBuffer(GL_COLOR_ATTACHMENT0);
    }

    // set target and blit
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glDrawBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);
    glBlitFramebuffer(src.left, src.bottom, src.left+src.width, src.bottom+src.height,
                      dst.left, dst.bottom, dst.left+dst.width, dst.bottom+dst.height,
                      mask, filter);
  }

  // restore currentBuffer
  mjr_restoreBuffer(con);
}



// Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done).
void mjr_setAux(int index, const mjrContext* con) {
  // check index
  if (index < 0 || index >= mjNAUX) {
    mju_error("Invalid aux buffer index");
  }

  // set
  if (con->auxFBO[index]) {
    glBindFramebuffer(GL_FRAMEBUFFER, con->auxFBO[index]);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
  } else {
    mju_error("auxFBO %d does not exist", index);
  }
}



// Blit from Aux buffer to con->currentBuffer.
void mjr_blitAux(int index, mjrRect src, int left, int bottom, const mjrContext* con) {
  // check index
  if (index < 0 || index >= mjNAUX) {
    mju_error("Invalid aux buffer index");
  }

  // make sure aux buffers is available
  if (!con->auxFBO[index]) {
    return;
  }

  // make sure blit is supported
  if (!glBlitFramebuffer) {
    return;
  }

  // resolve blit
  glBindFramebuffer(GL_READ_FRAMEBUFFER, con->auxFBO[index]);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con->auxFBO_r[index]);
  glDrawBuffer(GL_COLOR_ATTACHMENT0);
  glBlitFramebuffer(src.left, src.bottom, src.left+src.width, src.bottom+src.height,
                    src.left, src.bottom, src.left+src.width, src.bottom+src.height,
                    GL_COLOR_BUFFER_BIT, GL_NEAREST);

  // set source
  glBindFramebuffer(GL_READ_FRAMEBUFFER, con->auxFBO_r[index]);
  glReadBuffer(GL_COLOR_ATTACHMENT0);

  // set destination
  if (con->currentBuffer == mjFB_WINDOW) {
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glDrawBuffer(con->windowDoublebuffer ? GL_BACK : GL_FRONT);
  } else {
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con->offFBO);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
  }

  // blit
  glBlitFramebuffer(src.left, src.bottom, src.left+src.width, src.bottom+src.height,
                    left, bottom, left+src.width, bottom+src.height,
                    GL_COLOR_BUFFER_BIT, GL_NEAREST);

  // restore
  mjr_restoreBuffer(con);
}



//---------------------------------- Text ----------------------------------------------------------

// init OpenGL for 2D
static void init2D(void) {
  // set OpenGL options
  glDisable(GL_NORMALIZE);
  glDisable(GL_DEPTH_TEST);
  glShadeModel(GL_FLAT);
  glDisable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  // standard 2D projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}



// actual draw text, do not initialize OpenGL options
void mjr_textActual(int font, const char* txt, const mjrContext* con,
                    float x, float y, float z, float r, float g, float b) {
  // return if font is not installed
  if (!con->rangeFont) {
    return;
  }

  // shadow text
  if (font == mjFONT_SHADOW) {
    // blend shadow with black
    glListBase(con->baseFontShadow);
    glColor4f(0, 0, 0, 0.5);
    glRasterPos3f(x, y, z);
    glCallLists((GLsizei)strlen(txt), GL_UNSIGNED_BYTE, txt);

    // render text
    glListBase(con->baseFontNormal);
    glColor4f(r, g, b, 1);
    glRasterPos3f(x, y, z);
    glCallLists((GLsizei)strlen(txt), GL_UNSIGNED_BYTE, txt);
  }

  // regular text (normal or big)
  else {
    glListBase(font == mjFONT_BIG ? con->baseFontBig : con->baseFontNormal);
    glColor4f(r, g, b, 1);
    glRasterPos3f(x, y, z);
    glCallLists((GLsizei)strlen(txt), GL_UNSIGNED_BYTE, txt);
  }
}



// draw text at (x,y) in relative coordinates; font is mjtFont
void mjr_text(int font, const char* txt, const mjrContext* con,
              float x, float y, float r, float g, float b) {
  // init OpenGL
  init2D();

  // call actual drawing function
  mjr_textActual(font, txt, con, x, y, 0, r, g, b);
}



// actual overlay drawing; used for each column; return width (used as skip for next call)
static int draw_overlay(int font, mjrRect viewport, int skip, int gridpos,
                        float red, float green, float blue,
                        const char* overlay, const mjrContext* con) {
  int pos, ncthis, nc, nr, W, H, flg_big = (font == mjFONT_BIG);
  int PAD = 5, sz = mjMIN(mjMAXOVERLAY, (int)strlen(overlay));
  char text[mjMAXOVERLAY];

  // count rows and columns of text rectangle in pixels
  nr = flg_big ? con->charHeightBig : con->charHeight;
  ncthis = nc = 0;
  for (int i=0; i < sz; i++) {
    // process this char
    if (overlay[i] != '\n') {
      ncthis += flg_big ? con->charWidthBig[(unsigned char)overlay[i]]
                        : con->charWidth[(unsigned char)overlay[i]];
      nc = mjMAX(nc, ncthis);
    }

    // advance to next line
    else {
      nr += PAD + (flg_big ? con->charHeightBig : con->charHeight);
      ncthis = 0;
    }
  }

  // viewport width and height
  W = PAD+nc+8;
  H = PAD+nr;

  // set viewport to specific grid position
  switch (gridpos) {
  case mjGRID_TOPLEFT:
    glViewport(viewport.left+skip+PAD,
               viewport.bottom+viewport.height-1-PAD-H, W, H);
    break;

  case mjGRID_TOPRIGHT:
    glViewport(viewport.left+viewport.width-skip-1-PAD-W,
               viewport.bottom+viewport.height-1-PAD-H, W, H);
    break;

  case mjGRID_BOTTOMLEFT:
    glViewport(viewport.left+skip+PAD,
               viewport.bottom+PAD, W, H);
    break;

  case mjGRID_BOTTOMRIGHT:
    glViewport(viewport.left+viewport.width-skip-1-PAD-W,
               viewport.bottom+PAD, W, H);
    break;

  case mjGRID_TOP:
    glViewport(viewport.left+(viewport.width-skip-W)/2-1-PAD,
               viewport.bottom+viewport.height-1-PAD-H, W, H);
    break;

  case mjGRID_BOTTOM:
    glViewport(viewport.left+(viewport.width-skip-W)/2-1-PAD,
               viewport.bottom+PAD, W, H);
    break;

  case mjGRID_LEFT:
    glViewport(viewport.left+skip+PAD,
               viewport.bottom+(viewport.height-H)/2-1-PAD, W, H);
    break;

  case mjGRID_RIGHT:
    glViewport(viewport.left+viewport.width-skip-1-PAD-W,
               viewport.bottom+(viewport.height-H)/2-1-PAD, W, H);
  }

  // set projection in pixels
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, W-1, 0, H-1, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // blend with black background
  glEnable(GL_BLEND);
  glColor4d(0, 0, 0, 0.5);
  glBegin(GL_QUADS);
  glNormal3d(0, 0, 1);
  glVertex2d(0, 0);
  glVertex2d(0, H);
  glVertex2d(W, H);
  glVertex2d(W, 0);
  glEnd();
  glDisable(GL_BLEND);

  // draw text line by line
  nr = flg_big ? con->charHeightBig : con->charHeight;
  pos = 0;
  for (int i=0; i < sz; i++) {
    // terminate line and draw
    if (overlay[i] == '\n' || i == sz-1) {
      if (overlay[i] == '\n') {
        text[pos] = 0;
      } else {
        text[pos] = overlay[i];
        text[pos+1] = 0;
      }
      mjr_textActual(font, text, con, 3, H-nr, 0, red, green, blue);

      // advance to next line
      nr += PAD + (flg_big ? con->charHeightBig : con->charHeight);
      pos = 0;
    }

    // copy char
    else {
      text[pos++] = overlay[i];
    }
  }

  return W-2;
}



// draw text overlay; font is mjtFont; gridpos is mjtGridPos
void mjr_overlay(int font, int gridpos, mjrRect viewport,
                 const char* overlay, const char* overlay2, const mjrContext* con) {
  int skip = 0;

  // empty viewport: nothing to do
  if (viewport.width <= 0 || viewport.height <= 0) {
    return;
  }

  // init OpenGL once per overlay, set viewport later
  init2D();

  // two-column
  if (overlay2 && overlay2[0]) {
    // left side
    if (gridpos == mjGRID_TOPLEFT || gridpos == mjGRID_BOTTOMLEFT) {
      skip = draw_overlay(font, viewport, 0, gridpos, .7, .7, .7, overlay, con);
      draw_overlay(font, viewport, skip, gridpos, 1, 1, 1, overlay2, con);
    }

    // right side
    else {
      skip = draw_overlay(font, viewport, 0, gridpos, 1, 1, 1, overlay2, con);
      draw_overlay(font, viewport, skip, gridpos, .7, .7, .7, overlay, con);
    }
  }

  // one-column
  else {
    draw_overlay(font, viewport, 0, gridpos, 1, 1, 1, overlay, con);
  }
}



//------------------------------ 2D drawing --------------------------------------------------------

// get maximum viewport for active buffer
mjrRect mjr_maxViewport(const mjrContext* con) {
  // init with offscreen
  mjrRect res = {0, 0, con->offWidth, con->offHeight};

  // window: get from scissor box
  int dims[4];
  if (con->currentBuffer == mjFB_WINDOW) {
    glGetIntegerv(GL_SCISSOR_BOX, dims);
    res.width = dims[2];
    res.height = dims[3];
  }

  return res;
}



// draw rectangle
void mjr_rectangle(mjrRect viewport, float r, float g, float b, float a) {
  // empty viewport: nothing to do
  if (viewport.width <= 0 || viewport.height <= 0) {
    return;
  }

  // init OpenGL, set viewport
  init2D();
  glViewport(viewport.left, viewport.bottom, viewport.width, viewport.height);

  // draw rectangle
  glEnable(GL_BLEND);
  glColor4f(r, g, b, a);
  glBegin(GL_QUADS);
  glVertex2f(0, 0);
  glVertex2f(0, 1);
  glVertex2f(1, 1);
  glVertex2f(1, 0);
  glEnd();
  glDisable(GL_BLEND);
}



// draw rectangle with centered text
void mjr_label(mjrRect viewport, int font, const char* txt,
               float r, float g, float b, float a, float rt, float gt, float bt,
               const mjrContext* con) {
  // empty viewport: nothing to do
  if (viewport.width <= 0 || viewport.height <= 0) {
    return;
  }

  // set OpenGL options
  glDisable(GL_NORMALIZE);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);
  glShadeModel(GL_FLAT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  // standard 2D projection, in framebuffer units
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, viewport.width, 0, viewport.height, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // set viewport
  glViewport(viewport.left, viewport.bottom, viewport.width, viewport.height);

  // get sizes
  int W = viewport.width;
  int H = viewport.height;

  // render
  glBegin(GL_QUADS);
  glColor4f(r, g, b, a);
  glVertex2i(0, 0);
  glVertex2i(W, 0);
  glVertex2i(W, H);
  glVertex2i(0, H);
  glEnd();

  // draw text
  if (txt && con->rangeFont) {
    // compute width
    int i = 0, width = 0;
    if (font == mjFONT_BIG) {
      while (txt[i]) {
        width += con->charWidthBig[(unsigned char)txt[i++]];
      }
    } else {
      while (txt[i]) {
        width += con->charWidth[(unsigned char)txt[i++]];
      }
    }

    // compute center
    int cx = (W - width)/2;
    int cy = (H - (font == mjFONT_BIG ? con->charHeightBig : con->charHeight))/2;

    // draw
    glListBase(font == mjFONT_BIG ? con->baseFontBig : con->baseFontNormal);
    glColor3f(rt, gt, bt);
    glRasterPos2i(mjMAX(0, cx), cy);
    glCallLists((GLsizei)strlen(txt), GL_UNSIGNED_BYTE, txt);
  }
}



// make text, strip trailing zeros
static void maketext(const char* format, char* txt, float num, int txt_sz) {
  int i, j;

  // full text
  snprintf(txt, txt_sz, format, num);

  // locate trailing zeros
  i = strlen(txt);
  while (i > 0 && txt[i-1] == '0') {
    i--;
  }
  if (i <= 1) {
    return;
  }

  // strip if preceding char is '.'
  if (txt[i-1] == '.') {
    txt[i-1] = 0;
  }

  // otherwise find earlier '.'
  else {
    // find regular preceding digits
    j = i-1;
    while (j >= 0 && txt[j] >= '0' && txt[j] <= '9') {
      j--;
    }

    // '.' found: strip
    if (j >= 0 && txt[j] == '.') {
      txt[i] = 0;
    }
  }
}



// compute text width in pixels
static int textwidth(const mjrContext* con, const char* text) {
  int i = 0, width = 0;

  // add character widths
  while (text[i]) {
    width += con->charWidth[(unsigned char)text[i++]];
  }

  return width;
}



// draw 2D figure
void mjr_figure(mjrRect viewport, mjvFigure* fig, const mjrContext* con) {
  float range[2][2];
  float griddata[100];
  const float minrange = 1e-5, offset = 0.001;
  const int PAD = con->charHeight/2;
  char datatxt[STRING_BUFSIZE];

  // empty viewport: nothing to do
  if (viewport.width <= 0 || viewport.height <= 0) {
    return;
  }

  // init OpenGL, set viewport
  init2D();
  glViewport(viewport.left, viewport.bottom, viewport.width, viewport.height);

  // clear background and blend
  glEnable(GL_BLEND);
  glColor4fv(fig->figurergba);
  glBegin(GL_QUADS);
  glVertex2d(0, 0);
  glVertex2d(0, 1);
  glVertex2d(1, 1);
  glVertex2d(1, 0);
  glEnd();
  glDisable(GL_BLEND);

  // determine range along (x,y)
  for (int axis=0; axis < 2; axis++) {
    // init ranges, set flag
    range[axis][0] = fig->range[axis][0];
    range[axis][1] = fig->range[axis][1];
    int flg;
    if (fig->range[axis][0] < fig->range[axis][1]) {
      flg = 1;
    } else {
      flg = 0;
    }

    // determine range: scan line data, find min and max
    if (!flg || fig->flg_extend) {
      for (int n=0; n < mjMAXLINE; n++) {
        for (int i=0; i < fig->linepnt[n]; i++) {
          // get data
          float ldata = fig->linedata[n][2*i+axis];

          // skip if not finite
          if (!isfinite(ldata)) {
            continue;
          }

          // copy first finite data point
          if (!flg) {
            range[axis][0] = ldata;
            range[axis][1] = ldata;
            flg = 1;
          }

          // otherwise regular update
          else {
            // update minimum
            if (range[axis][0] > ldata) {
              range[axis][0] = ldata;
            }

            // update maximum
            if (range[axis][1] < ldata) {
              range[axis][1] = ldata;
            }
          }
        }
      }
    }

    // make sure range is not too small
    if (range[axis][1]-range[axis][0] < minrange) {
      float needed = minrange - (range[axis][1]-range[axis][0]);
      range[axis][0] -= 0.5f*needed;
      range[axis][1] += 0.5f*needed;
    }

    // make y-range symmetric
    if (fig->flg_symmetric) {
      float ymax = mjMAX(fabsf(range[1][0]), fabsf(range[1][1]));
      range[1][0] = -ymax;
      range[1][1] = +ymax;
    }
  }

  // save ranges
  fig->xaxisdata[0] = range[0][0];
  fig->xaxisdata[1] = range[0][1];
  fig->yaxisdata[0] = range[1][0];
  fig->yaxisdata[1] = range[1][1];

  // set projection in pixels
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (viewport.width <= 1 || viewport.height <= 1) {
    return;
  }
  glOrtho(0, viewport.width-1, 0, viewport.height-1, -1, 1);

  // draw title, save vertical size
  int sy_title = 0;
  if (fig->title[0]) {
    // find address of selected subplot title
    int subadr = 0, subcnt = 0;
    while (subcnt < fig->subplot && subadr < 1000 && fig->title[subadr]) {
      // skip non-space
      while (subadr < 1000 && fig->title[subadr] && fig->title[subadr] != ' ') {
        subadr++;
      }

      // skip space, count
      int cntspace = 0;
      while (subadr < 1000 && fig->title[subadr] == ' ') {
        subadr++;
        cntspace++;
      }

      // count subplot if 2+ spaces
      if (cntspace > 1) {
        subcnt++;
      }
    }

    // find length of selected subplot title (skip non-space or single space)
    int sublen = 0;
    while ((subadr+sublen < 1000 &&
            fig->title[subadr+sublen] &&
            fig->title[subadr+sublen] != ' ') ||
           (subadr+sublen+1 < 1000 && fig->title[subadr+sublen+1] &&
            fig->title[subadr+sublen+1] != ' ')) {
      sublen++;
    }

    // proceed if non-empty
    if (sublen) {
      // save vertical size
      sy_title = con->charHeight;

      // get title size
      int sx_title = 0;
      for (int i=subadr; i < subadr+sublen; i++) {
        sx_title += con->charWidth[(unsigned char)fig->title[i]];
      }

      // compute left edge of title
      int left = mjMAX(0, viewport.width/2-sx_title/2);

      // compute left edge and address of extended title
      int left1 = left, subadr1 = subadr;
      while (subadr1 > 0 && left1 >= con->charWidth[(unsigned char)fig->title[subadr1-1]]) {
        left1 -= con->charWidth[(unsigned char)fig->title[subadr1-1]];
        subadr1--;
      }

      // compute right edge and length of extended title
      int sublen1 = 0, right1 = left1;
      while (sublen1+subadr1 < 1000 && fig->title[subadr1+sublen1] &&
             right1+con->charWidth[(unsigned char)fig->title[subadr1+sublen1]] < viewport.width) {
        right1 += con->charWidth[(unsigned char)fig->title[subadr1+sublen1]];
        sublen1++;
      }

      // extended title
      glListBase(con->baseFontNormal);
      glColor4f(0.4*fig->textrgb[0]+0.6*fig->figurergba[0],
                0.4*fig->textrgb[1]+0.6*fig->figurergba[1],
                0.4*fig->textrgb[2]+0.6*fig->figurergba[2], 1);
      glRasterPos3f(left1, viewport.height-PAD-sy_title, 0);
      glCallLists((GLsizei)sublen1, GL_UNSIGNED_BYTE, fig->title+subadr1);

      // actual title
      glListBase(con->baseFontNormal);
      glColor4f(fig->textrgb[0], fig->textrgb[1], fig->textrgb[2], 1);
      glRasterPos3f(left, viewport.height-PAD-sy_title, 0);
      glCallLists((GLsizei)sublen, GL_UNSIGNED_BYTE, fig->title+subadr);
    }
  }

  // draw xlabel, save size
  int sx_label = 0, sy_label = 0;
  if (fig->xlabel[0]) {
    // get xlabel size
    sx_label = textwidth(con, fig->xlabel);
    sy_label = con->charHeight;

    // render xlabel
    mjr_textActual(mjFONT_NORMAL, fig->xlabel, con,
                   mjMAX(0, viewport.width/2-sx_label/2), PAD, 0,
                   fig->textrgb[0], fig->textrgb[1], fig->textrgb[2]);
  }

  // reduce viewport to account for title and xlabel vertical size
  if (sy_title || sy_label) {
    // reduce viewport
    viewport.bottom += (sy_label ? sy_label + PAD/2 : 0);
    viewport.height -= (sy_label ? sy_label + PAD/2 : 0) + (sy_title ? sy_title + PAD : 0);
    if (viewport.width <= 0 || viewport.height <= 0) {
      return;
    }
    glViewport(viewport.left, viewport.bottom, viewport.width, viewport.height);

    // set matrix projection in pixels
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, viewport.width-1, 0, viewport.height-1, -1, 1);
  }

  // draw optional tick labels, reduce viewport
  if (fig->gridsize[0] > 1 && fig->gridsize[1] > 1) {
    char txt[STRING_BUFSIZE];

    // determine xtick height
    int xtick_height = 0;
    if (fig->flg_ticklabel[0]) {
      xtick_height = con->charHeight + PAD;
    }

    // determine ytick width
    int ytick_width = 0;
    if (fig->flg_ticklabel[1]) {
      // start with minwidth
      if (fig->minwidth[0]) {
        ytick_width = textwidth(con, fig->minwidth);
      }

      // limit to 20 ticks
      int n = mjMIN(20, fig->gridsize[1]);

      // generate all strings, find max width
      for (int i=0; i < n; i++) {
        const int txt_sz = mjSIZEOFARRAY(txt);
        maketext(fig->yformat, txt, range[1][0] + (range[1][1]-range[1][0])*i/(float)(n-1),
                 txt_sz);
        ytick_width = mjMAX(ytick_width, textwidth(con, txt));
      }

      ytick_width += PAD;
    }

    // draw xtick
    if (fig->flg_ticklabel[0]) {
      // limit to 20 ticks
      int n = mjMIN(20, fig->gridsize[0]);

      // process ticks
      for (int i=0; i < n; i++) {
        // make text, get width
        {
          const int txt_sz = mjSIZEOFARRAY(txt);
          maketext(fig->xformat, txt,
                   range[0][0] + (range[0][1]-range[0][0])*i/(float)(n-1), txt_sz);
        }
        int w = textwidth(con, txt);

        // draw
        mjr_textActual(mjFONT_NORMAL, txt, con,
                       PAD+ytick_width+
                       (i == 0 ? 0 : (i == n-1 ? -w : -w/2))+
                       (viewport.width-2*PAD-ytick_width)*i/(float)(n-1),
                       PAD, 0,
                       fig->textrgb[0], fig->textrgb[0], fig->textrgb[0]);
      }
    }

    // draw ytick
    if (fig->flg_ticklabel[1]) {
      // limit to 20 ticks
      int n = mjMIN(20, fig->gridsize[1]);

      // process ticks
      for (int i=0; i < n; i++) {
        // make text, get width
        {
          const int txt_sz = mjSIZEOFARRAY(txt);
          maketext(fig->yformat, txt,
                   range[1][0] + (range[1][1]-range[1][0])*i/(float)(n-1), txt_sz);
        }
        int w = textwidth(con, txt);

        // draw
        mjr_textActual(mjFONT_NORMAL, txt, con,
                       ytick_width-w,
                       PAD+xtick_height-con->charHeight/2+
                       (viewport.height-2*PAD-xtick_height)*i/(float)(n-1), 0,
                       fig->textrgb[0], fig->textrgb[0], fig->textrgb[0]);
      }
    }

    // adjust viewport size
    viewport.left += ytick_width;
    viewport.width -= ytick_width;
    viewport.bottom += xtick_height;
    viewport.height -= xtick_height;
  }

  // set plot viewport
  viewport.left += PAD;
  viewport.width -= 2*PAD;
  viewport.bottom += PAD;
  viewport.height -= 2*PAD;
  if (viewport.width <= 0 || viewport.height <= 0) {
    return;
  }
  glViewport(viewport.left, viewport.bottom, viewport.width, viewport.height);

  // save range
  fig->xaxispixel[0] = viewport.left;
  fig->xaxispixel[1] = viewport.left + viewport.width - 1;
  fig->yaxispixel[0] = viewport.bottom;
  fig->yaxispixel[1] = viewport.bottom + viewport.height - 1;

  // set matrix projection to (0,1) + offset
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-offset, 1 + offset, -offset, 1 + offset, -1, 1);

  // draw pane background
  glEnable(GL_BLEND);
  glColor4fv(fig->panergba);
  glBegin(GL_QUADS);
  glVertex2d(0, 0);
  glVertex2d(0, 1);
  glVertex2d(1, 1);
  glVertex2d(1, 0);
  glEnd();
  glDisable(GL_BLEND);

  // prepare to render lines
  glEnableClientState(GL_VERTEX_ARRAY);

  // draw grid
  if (fig->gridsize[0] > 1 && fig->gridsize[1] > 1) {
    // common GL state
    glVertexPointer(2, GL_FLOAT, 0, griddata);
    glColor3fv(fig->gridrgb);
    glLineWidth(fig->gridwidth);

    // prepare vertical lines
    int n = mjMIN(20, fig->gridsize[0]);
    for (int i=0; i < n; i++) {
      griddata[4*i] = i/(float)(n-1);
      griddata[4*i+1] = 0;
      griddata[4*i+2] = griddata[4*i];
      griddata[4*i+3] = 1;
    }

    // draw vertical lines
    glDrawArrays(GL_LINES, 0, 2*n);

    // prepare horizontal lines
    n = mjMIN(20, fig->gridsize[1]);
    for (int i=0; i < n; i++) {
      griddata[4*i] = 0;
      griddata[4*i+1] = i/(float)(n-1);
      griddata[4*i+2] = 1;
      griddata[4*i+3] = griddata[4*i+1];
    }

    // draw vertical lines
    glDrawArrays(GL_LINES, 0, 2*n);
  }

  // set matrix projection to range, with small padding
  float difx = range[0][1] - range[0][0];
  float dify = range[1][1] - range[1][0];
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(range[0][0] - difx*offset, range[0][1] + difx*offset,
          range[1][0] - dify*offset, range[1][1] + dify*offset, -1, 1);

  // draw lines: back to front
  for (int n=mjMAXLINE-1; n >= 0; n--)
    if (fig->linepnt[n]) {
      // GL state
      glVertexPointer(2, GL_FLOAT, 0, fig->linedata[n]);
      glColor3fv(fig->linergb[n]);
      glLineWidth(fig->linewidth);

      // draw line
      glDrawArrays(fig->flg_barplot ? GL_LINES : GL_LINE_STRIP, 0, fig->linepnt[n]);
    }

  // selection
  if (fig->flg_selection) {
    glColor3f(1, 1, 1);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex2f(fig->selection, range[1][0]);
    glVertex2f(fig->selection, range[1][1]);
    glEnd();
  }

  // draw legend, find line to highlight
  int hlight = fig->highlightid;
  if (fig->flg_legend) {
    // set matrix projection in pixels
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (viewport.width <= 1 || viewport.height <= 1) {
      return;
    }
    glOrtho(0, viewport.width-1, 0, viewport.height-1, -1, 1);

    // find legend size
    int lw = 0, lh = 0, cnt = 0;
    for (int n=fig->legendoffset; n < mjMAXLINE; n++) {
      if (fig->linename[n][0]) {
        // max width, accumulate height
        lw = mjMAX(lw, textwidth(con, fig->linename[n]));
        lh += con->charHeight;

        // count
        cnt++;

        // too big: correct and break
        if (lh+2*PAD >= viewport.height) {
          cnt--;
          lh -= con->charHeight;
          break;
        }
      }
    }

    // detect highlighted line
    int hcnt = -1;
    int hx = fig->highlight[0] - viewport.left;
    int hy = fig->highlight[1] - viewport.bottom;
    if (hx >= viewport.width-PAD-lw &&
        hx <= viewport.width-PAD &&
        hy >= viewport.height-PAD-lh &&
        hy <= viewport.height-PAD) {
      hcnt = (viewport.height-PAD-hy) / con->charHeight;
    }

    // draw background
    if (cnt) {
      glEnable(GL_BLEND);
      glColor4fv(fig->legendrgba);
      glBegin(GL_QUADS);
      glVertex2d(viewport.width-2*PAD-lw, viewport.height-2*PAD-lh);
      glVertex2d(viewport.width-2*PAD-lw, viewport.height);
      glVertex2d(viewport.width, viewport.height);
      glVertex2d(viewport.width, viewport.height-2*PAD-lh);
      glEnd();
      glDisable(GL_BLEND);
    }

    // find named lines and draw text
    cnt = 0;
    for (int n=fig->legendoffset; n < mjMAXLINE; n++) {
      if (fig->linename[n][0]) {
        // get width
        int width = textwidth(con, fig->linename[n]);

        // render right-aligned
        mjr_textActual(mjFONT_SHADOW, fig->linename[n], con,
                       viewport.width-PAD-width, viewport.height-PAD-(cnt+1)*con->charHeight, 0,
                       fig->linergb[n][0], fig->linergb[n][1], fig->linergb[n][2]);

        // save hlight
        if (hcnt == cnt) {
          hlight = n;
        }

        // count
        cnt++;

        // break if viewport is full
        if ((cnt+1)*con->charHeight+2*PAD >= viewport.height) {
          break;
        }
      }
    }
  }

  // draw highlight
  if (hlight >= 0 && hlight < mjMAXLINE) {
    // line-rendering projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(range[0][0] - difx*offset, range[0][1] + difx*offset,
            range[1][0] - dify*offset, range[1][1] + dify*offset, -1, 1);

    // render selected line with extra width
    glVertexPointer(2, GL_FLOAT, 0, fig->linedata[hlight]);
    glColor3fv(fig->linergb[hlight]);
    glLineWidth(5*fig->linewidth);
    glDrawArrays(fig->flg_barplot ? GL_LINES : GL_LINE_STRIP, 0, fig->linepnt[hlight]);

    // print data coordinates
    if (fig->flg_selection) {
      // find nearest x-value
      int ibest = -1;
      float best = 0;
      for (int i=0; i < fig->linepnt[hlight]; i++) {
        if (ibest < 0 || best > fabs(fig->selection-fig->linedata[hlight][2*i])) {
          ibest = i;
          best = fabs(fig->selection-fig->linedata[hlight][2*i]);
        }
      }

      // show text
      mjSNPRINTF(datatxt, "( %.4g : %.4g )",
                 fig->linedata[hlight][2*ibest], fig->linedata[hlight][2*ibest+1]);
      mjr_textActual(mjFONT_SHADOW, datatxt, con,
                     0.9*range[0][0]+0.1*range[0][1], 0.9*range[1][0]+0.1*range[1][1], 0,
                     fig->textrgb[0], fig->textrgb[1], fig->textrgb[2]);
    }
  }

  // stop rendering lines
  glDisableClientState(GL_VERTEX_ARRAY);
}
