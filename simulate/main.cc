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

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <mujoco/mjxmacro.h>
#include "glfw_dispatch.h"
#include "simulate.h"
#include "array_safety.h"

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using ::mujoco::Glfw;

// constants
const double syncmisalign = 0.1;    // maximum time mis-alignment before re-sync
const double refreshfactor = 0.7;   // fraction of refresh available for simulation
const int kErrorLength = 1024;

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// control noise variables
mjtNum* ctrlnoise = nullptr;



//------------------------------------------- simulation -------------------------------------------


mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, mj::Simulate::kMaxFilenameLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.loadError, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  double cpusync = 0;
  mjtNum simsync = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.load(sim.dropfilename, mnew, dnew, true);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = (mjtNum*) malloc(sizeof(mjtNum)*m->nu);
        mju_zero(ctrlnoise, m->nu);
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.load(sim.filename, mnew, dnew, true);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
        mju_zero(ctrlnoise, m->nu);
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      const std::lock_guard<std::mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          // record cpu time at start of iteration
          double tmstart = Glfw().glfwGetTime();

          // inject noise
          if (sim.ctrlnoisestd) {
            // convert rate and scale to discrete time given current timestep
            mjtNum rate = mju_exp(-m->opt.timestep / sim.ctrlnoiserate);
            mjtNum scale = sim.ctrlnoisestd * mju_sqrt(1-rate*rate);

            for (int i=0; i<m->nu; i++) {
              // update noise
              ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);
              // apply noise
              d->ctrl[i] = ctrlnoise[i];
            }
          }

          // out-of-sync (for any reason)
          mjtNum offset = mju_abs((d->time*sim.slow_down-simsync)-(tmstart-cpusync));
          if (d->time*sim.slow_down<simsync || tmstart<cpusync || cpusync==0 ||
              offset > syncmisalign*sim.slow_down || sim.speed_changed) {
            // re-sync
            cpusync = tmstart;
            simsync = d->time*sim.slow_down;
            sim.speed_changed = false;

            // clear old perturbations, apply new
            mju_zero(d->xfrc_applied, 6*m->nbody);
            sim.applyposepertubations(0);  // move mocap bodies only
            sim.applyforceperturbations();

            // run single step, let next iteration deal with timing
            mj_step(m, d);
          }

          // in-sync
          else {
            // step while simtime lags behind cputime, and within safefactor
            while ((d->time*sim.slow_down-simsync) < (Glfw().glfwGetTime()-cpusync) &&
                   (Glfw().glfwGetTime()-tmstart) < refreshfactor/sim.vmode.refreshRate) {
              // clear old perturbations, apply new
              mju_zero(d->xfrc_applied, 6*m->nbody);
              sim.applyposepertubations(0);  // move mocap bodies only
              sim.applyforceperturbations();

              // run mj_step
              mjtNum prevtm = d->time*sim.slow_down;
              mj_step(m, d);

              // break on reset
              if (d->time*sim.slow_down<prevtm) {
                break;
              }
            }
          }
        }

        // paused
        else {
          // apply pose perturbation
          sim.applyposepertubations(1);  // move mocap and dynamic bodies

          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
        }
      }
    }  // std::lock_guard<std::mutex>
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    m = LoadModel(filename, *sim);
    if (m) d = mj_makeData(m);
    if (d) {
      sim->load(filename, m, d, true);
      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

// run event loop
int main(int argc, const char** argv) {
  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>();

  // init GLFW
  if (!Glfw().glfwInit()) {
    mju_error("could not initialize GLFW");
  }

  const char* filename = nullptr;
  if (argc >  1) {
    filename = argv[1];
  }

  // start physics thread
  std::thread physicsthreadhandle = std::thread(&PhysicsThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->renderloop();
  physicsthreadhandle.join();

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  Glfw().glfwTerminate();
#endif

  return 0;
}
