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

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <mujoco/mujoco.h>

// select EGL, OSMESA or GLFW
#if defined(MJ_EGL)
  #include <EGL/egl.h>
#elif defined(MJ_OSMESA)
  #include <GL/osmesa.h>
  OSMesaContext ctx;
  unsigned char buffer[10000000];
#else
  #include <GLFW/glfw3.h>
#endif

#include "array_safety.h"
namespace mju = ::mujoco::sample_util;

//-------------------------------- global data ------------------------------------------

// MuJoCo model and data
mjModel* m = 0;
mjData* d = 0;

// MuJoCo visualization
mjvScene scn;
mjvCamera cam;
mjvOption opt;
mjrContext con;


//-------------------------------- utility functions ------------------------------------

// load model, init simulation and rendering
void initMuJoCo(const char* filename) {
  // load and compile
  char error[1000] = "Could not load binary model";
  if (std::strlen(filename)>4 && !std::strcmp(filename+std::strlen(filename)-4, ".mjb")) {
    m = mj_loadModel(filename, 0);
  } else {
    m = mj_loadXML(filename, 0, error, 1000);
  }
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data, run one computation to initialize all fields
  d = mj_makeData(m);
  mj_forward(m, d);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, 200);

  // default free camera
  mjv_defaultFreeCamera(m, &cam);
}


// deallocate everything
void closeMuJoCo(void) {
  mj_deleteData(d);
  mj_deleteModel(m);
  mjr_freeContext(&con);
  mjv_freeScene(&scn);
}


// create OpenGL context/window
void initOpenGL(void) {
  //------------------------ EGL
#if defined(MJ_EGL)
  // desired config
  const EGLint configAttribs[] = {
    EGL_RED_SIZE,           8,
    EGL_GREEN_SIZE,         8,
    EGL_BLUE_SIZE,          8,
    EGL_ALPHA_SIZE,         8,
    EGL_DEPTH_SIZE,         24,
    EGL_STENCIL_SIZE,       8,
    EGL_COLOR_BUFFER_TYPE,  EGL_RGB_BUFFER,
    EGL_SURFACE_TYPE,       EGL_PBUFFER_BIT,
    EGL_RENDERABLE_TYPE,    EGL_OPENGL_BIT,
    EGL_NONE
  };

  // get default display
  EGLDisplay eglDpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  if (eglDpy==EGL_NO_DISPLAY) {
    mju_error("Could not get EGL display, error 0x%x\n", eglGetError());
  }

  // initialize
  EGLint major, minor;
  if (eglInitialize(eglDpy, &major, &minor)!=EGL_TRUE) {
    mju_error("Could not initialize EGL, error 0x%x\n", eglGetError());
  }

  // choose config
  EGLint numConfigs;
  EGLConfig eglCfg;
  if (eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs)!=EGL_TRUE) {
    mju_error("Could not choose EGL config, error 0x%x\n", eglGetError());
  }

  // bind OpenGL API
  if (eglBindAPI(EGL_OPENGL_API)!=EGL_TRUE) {
    mju_error("Could not bind EGL OpenGL API, error 0x%x\n", eglGetError());
  }

  // create context
  EGLContext eglCtx = eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, NULL);
  if (eglCtx==EGL_NO_CONTEXT) {
    mju_error("Could not create EGL context, error 0x%x\n", eglGetError());
  }

  // make context current, no surface (let OpenGL handle FBO)
  if (eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, eglCtx)!=EGL_TRUE) {
    mju_error("Could not make EGL context current, error 0x%x\n", eglGetError());
  }

  //------------------------ OSMESA
#elif defined(MJ_OSMESA)
  // create context
  ctx = OSMesaCreateContextExt(GL_RGBA, 24, 8, 8, 0);
  if (!ctx) {
    mju_error("OSMesa context creation failed");
  }

  // make current
  if (!OSMesaMakeCurrent(ctx, buffer, GL_UNSIGNED_BYTE, 800, 800)) {
    mju_error("OSMesa make current failed");
  }

  //------------------------ GLFW
#else
  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create invisible window, single-buffered
  glfwWindowHint(GLFW_VISIBLE, 0);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(800, 800, "Invisible window", NULL, NULL);
  if (!window) {
    mju_error("Could not create GLFW window");
  }

  // make context current
  glfwMakeContextCurrent(window);
#endif
}


// close OpenGL context/window
void closeOpenGL(void) {
  //------------------------ EGL
#if defined(MJ_EGL)
  // get current display
  EGLDisplay eglDpy = eglGetCurrentDisplay();
  if (eglDpy==EGL_NO_DISPLAY) {
    return;
  }

  // get current context
  EGLContext eglCtx = eglGetCurrentContext();

  // release context
  eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

  // destroy context if valid
  if (eglCtx!=EGL_NO_CONTEXT) {
    eglDestroyContext(eglDpy, eglCtx);
  }

  // terminate display
  eglTerminate(eglDpy);

  //------------------------ OSMESA
#elif defined(MJ_OSMESA)
  OSMesaDestroyContext(ctx);

  //------------------------ GLFW
#else
  // terminate GLFW (crashes with Linux NVidia drivers)
  #if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
  #endif
#endif
}


//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv) {
  // check command-line arguments
  if (argc!=5) {
    std::printf(" USAGE:  record modelfile duration fps rgbfile\n");
    return 0;
  }

  // parse numeric arguments
  double duration = 10, fps = 30;
  std::sscanf(argv[2], "%lf", &duration);
  std::sscanf(argv[3], "%lf", &fps);

  // initialize OpenGL and MuJoCo
  initOpenGL();
  initMuJoCo(argv[1]);

  // set rendering to offscreen buffer
  mjr_setBuffer(mjFB_OFFSCREEN, &con);
  if (con.currentBuffer!=mjFB_OFFSCREEN) {
    std::printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");
  }

  // get size of active renderbuffer
  mjrRect viewport =  mjr_maxViewport(&con);
  int W = viewport.width;
  int H = viewport.height;

  // allocate rgb and depth buffers
  unsigned char* rgb = (unsigned char*)std::malloc(3*W*H);
  float* depth = (float*)std::malloc(sizeof(float)*W*H);
  if (!rgb || !depth) {
    mju_error("Could not allocate buffers");
  }

  // create output rgb file
  std::FILE* fp = std::fopen(argv[4], "wb");
  if (!fp) {
    mju_error("Could not open rgbfile for writing");
  }

  // main loop
  double frametime = 0;
  int framecount = 0;
  while (d->time<duration) {
    // render new frame if it is time (or first frame)
    if ((d->time-frametime)>1/fps || frametime==0) {
      // update abstract scene
      mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

      // render scene in offscreen buffer
      mjr_render(viewport, &scn, &con);

      // add time stamp in upper-left corner
      char stamp[50];
      mju::sprintf_arr(stamp, "Time = %.3f", d->time);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &con);

      // read rgb and depth buffers
      mjr_readPixels(rgb, depth, viewport, &con);

      // insert subsampled depth image in lower-left corner of rgb image
      const int NS = 3;           // depth image sub-sampling
      for (int r=0; r<H; r+=NS)
        for (int c=0; c<W; c+=NS) {
          int adr = (r/NS)*W + c/NS;
          rgb[3*adr] = rgb[3*adr+1] = rgb[3*adr+2] = (unsigned char)((1.0f-depth[r*W+c])*255.0f);
        }

      // write rgb image to file
      std::fwrite(rgb, 3, W*H, fp);

      // print every 10 frames: '.' if ok, 'x' if OpenGL error
      if (((framecount++)%10)==0) {
        if (mjr_getError()) {
          std::printf("x");
        } else {
          std::printf(".");
        }
      }

      // save simulation time
      frametime = d->time;
    }

    // advance simulation
    mj_step(m, d);
  }
  std::printf("\n");

  // close file, free buffers
  std::fclose(fp);
  std::free(rgb);
  std::free(depth);

  // close MuJoCo and OpenGL
  closeMuJoCo();
  closeOpenGL();

  return 1;
}
