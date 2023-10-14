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

#ifndef MUJOCO_MJRENDER_H_
#define MUJOCO_MJRENDER_H_


#if defined(__cplusplus)
extern "C" {
#endif

#define mjNAUX          10        // number of auxiliary buffers
#define mjMAXTEXTURE    1000      // maximum number of textures


//---------------------------------- primitive types (mjt) -----------------------------------------

typedef enum mjtGridPos_ {        // grid position for overlay
  mjGRID_TOPLEFT      = 0,        // top left
  mjGRID_TOPRIGHT,                // top right
  mjGRID_BOTTOMLEFT,              // bottom left
  mjGRID_BOTTOMRIGHT,             // bottom right
  mjGRID_TOP,                     // top center
  mjGRID_BOTTOM,                  // bottom center
  mjGRID_LEFT,                    // left center
  mjGRID_RIGHT                    // right center
} mjtGridPos;


typedef enum mjtFramebuffer_ {    // OpenGL framebuffer option
  mjFB_WINDOW         = 0,        // default/window buffer
  mjFB_OFFSCREEN                  // offscreen buffer
} mjtFramebuffer;

typedef enum mjtDepthMap_ {       // depth mapping for `mjr_readPixels`
  mjDEPTH_ZERONEAR    = 0,        // standard depth map; 0: znear, 1: zfar
  mjDEPTH_ZEROFAR     = 1         // reversed depth map; 1: znear, 0: zfar
} mjtDepthMap;

typedef enum mjtFontScale_ {      // font scale, used at context creation
  mjFONTSCALE_50      = 50,       // 50% scale, suitable for low-res rendering
  mjFONTSCALE_100     = 100,      // normal scale, suitable in the absence of DPI scaling
  mjFONTSCALE_150     = 150,      // 150% scale
  mjFONTSCALE_200     = 200,      // 200% scale
  mjFONTSCALE_250     = 250,      // 250% scale
  mjFONTSCALE_300     = 300       // 300% scale
} mjtFontScale;


typedef enum mjtFont_ {           // font type, used at each text operation
  mjFONT_NORMAL       = 0,        // normal font
  mjFONT_SHADOW,                  // normal font with shadow (for higher contrast)
  mjFONT_BIG                      // big font (for user alerts)
} mjtFont;


struct mjrRect_ {                 // OpenGL rectangle
  int left;                       // left (usually 0)
  int bottom;                     // bottom (usually 0)
  int width;                      // width (usually buffer width)
  int height;                     // height (usually buffer height)
};
typedef struct mjrRect_ mjrRect;


//---------------------------------- mjrContext ----------------------------------------------------

struct mjrContext_ {              // custom OpenGL context
  // parameters copied from mjVisual
  float lineWidth;                // line width for wireframe rendering
  float shadowClip;               // clipping radius for directional lights
  float shadowScale;              // fraction of light cutoff for spot lights
  float fogStart;                 // fog start = stat.extent * vis.map.fogstart
  float fogEnd;                   // fog end = stat.extent * vis.map.fogend
  float fogRGBA[4];               // fog rgba
  int shadowSize;                 // size of shadow map texture
  int offWidth;                   // width of offscreen buffer
  int offHeight;                  // height of offscreen buffer
  int offSamples;                 // number of offscreen buffer multisamples

  // parameters specified at creation
  int fontScale;                  // font scale
  int auxWidth[mjNAUX];           // auxiliary buffer width
  int auxHeight[mjNAUX];          // auxiliary buffer height
  int auxSamples[mjNAUX];         // auxiliary buffer multisamples

  // offscreen rendering objects
  unsigned int offFBO;            // offscreen framebuffer object
  unsigned int offFBO_r;          // offscreen framebuffer for resolving multisamples
  unsigned int offColor;          // offscreen color buffer
  unsigned int offColor_r;        // offscreen color buffer for resolving multisamples
  unsigned int offDepthStencil;   // offscreen depth and stencil buffer
  unsigned int offDepthStencil_r; // offscreen depth and stencil buffer for resolving multisamples

  // shadow rendering objects
  unsigned int shadowFBO;         // shadow map framebuffer object
  unsigned int shadowTex;         // shadow map texture

  // auxiliary buffers
  unsigned int auxFBO[mjNAUX];    // auxiliary framebuffer object
  unsigned int auxFBO_r[mjNAUX];  // auxiliary framebuffer object for resolving
  unsigned int auxColor[mjNAUX];  // auxiliary color buffer
  unsigned int auxColor_r[mjNAUX];// auxiliary color buffer for resolving

  // texture objects and info
  int ntexture;                   // number of allocated textures
  int textureType[100];           // type of texture (mjtTexture) (ntexture)
  unsigned int texture[100];      // texture names

  // displaylist starting positions
  unsigned int basePlane;         // all planes from model
  unsigned int baseMesh;          // all meshes from model
  unsigned int baseHField;        // all hfields from model
  unsigned int baseBuiltin;       // all buildin geoms, with quality from model
  unsigned int baseFontNormal;    // normal font
  unsigned int baseFontShadow;    // shadow font
  unsigned int baseFontBig;       // big font

  // displaylist ranges
  int rangePlane;                 // all planes from model
  int rangeMesh;                  // all meshes from model
  int rangeHField;                // all hfields from model
  int rangeBuiltin;               // all builtin geoms, with quality from model
  int rangeFont;                  // all characters in font

  // skin VBOs
  int nskin;                      // number of skins
  unsigned int* skinvertVBO;      // skin vertex position VBOs (nskin)
  unsigned int* skinnormalVBO;    // skin vertex normal VBOs (nskin)
  unsigned int* skintexcoordVBO;  // skin vertex texture coordinate VBOs (nskin)
  unsigned int* skinfaceVBO;      // skin face index VBOs (nskin)

  // character info
  int charWidth[127];             // character widths: normal and shadow
  int charWidthBig[127];          // chacarter widths: big
  int charHeight;                 // character heights: normal and shadow
  int charHeightBig;              // character heights: big

  // capabilities
  int glInitialized;              // is OpenGL initialized
  int windowAvailable;            // is default/window framebuffer available
  int windowSamples;              // number of samples for default/window framebuffer
  int windowStereo;               // is stereo available for default/window framebuffer
  int windowDoublebuffer;         // is default/window framebuffer double buffered

  // framebuffer
  int     currentBuffer;          // currently active framebuffer: mjFB_WINDOW or mjFB_OFFSCREEN

  // pixel output format
  int     readPixelFormat;        // default color pixel format for mjr_readPixels

  // depth output format
  int     readDepthMap;           // depth mapping: mjDEPTH_ZERONEAR or mjDEPTH_ZEROFAR
};
typedef struct mjrContext_ mjrContext;

#if defined(__cplusplus)
}
#endif
#endif  // MUJOCO_MJRENDER_H_
