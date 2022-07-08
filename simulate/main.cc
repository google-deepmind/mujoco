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
#include <mutex>
#include <string>
#include <thread>

#include <mjxmacro.h>
#include "uitools.h"
#include "simulate.h"

#include "array_safety.h"

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncmisalign = 0.1;    // maximum time mis-alignment before re-sync
const double refreshfactor = 0.7;   // fraction of refresh available for simulation

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// control noise variables
mjtNum* ctrlnoise = nullptr;

//---------------------------------- simulation --------------------------------------

// sim thread synchronization
std::mutex& GetMutex() {
  static std::mutex* mtx = new std::mutex();
  return *mtx;
}

mj::Simulate& GetInstance() {
  // the creation of this static member will immediately
  // initialize the glfw ui
  static mj::Simulate* simulate = new mj::Simulate();
  return *simulate;
}

// simulate in background thread (while rendering in main thread)
void simulate_thread(void) {
  // cpu-sim syncronization point
  double cpusync = 0;
  mjtNum simsync = 0;

  // run until asked to exit
  while (!GetInstance().exitrequest) {
    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (GetInstance().run && GetInstance().busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    { // start exclusive access
      const std::lock_guard<std::mutex> lock(GetMutex());

      // run only if model is present
      if (m) {
        // running
        if (GetInstance().run) {
          // record cpu time at start of iteration
          double tmstart = glfwGetTime();

          // inject noise
          if (GetInstance().ctrlnoisestd) {
            // convert rate and scale to discrete time given current timestep
            mjtNum rate = mju_exp(-m->opt.timestep / GetInstance().ctrlnoiserate);
            mjtNum scale = GetInstance().ctrlnoisestd * mju_sqrt(1-rate*rate);

            for (int i=0; i<m->nu; i++) {
              // update noise
              ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);
              // apply noise
              d->ctrl[i] = ctrlnoise[i];
            }
          }

          // out-of-sync (for any reason)
          mjtNum offset = mju_abs((d->time*GetInstance().slow_down-simsync)-(tmstart-cpusync));
          if( d->time*GetInstance().slow_down<simsync || tmstart<cpusync || cpusync==0 ||
              offset > syncmisalign*GetInstance().slow_down || GetInstance().speed_changed) {
            // re-sync
            cpusync = tmstart;
            simsync = d->time*GetInstance().slow_down;
            GetInstance().speed_changed = false;

            // clear old perturbations, apply new
            mju_zero(d->xfrc_applied, 6*m->nbody);
            mjv_applyPerturbPose(m, d, &GetInstance().pert, 0);  // move mocap bodies only
            mjv_applyPerturbForce(m, d, &GetInstance().pert);

            // run single step, let next iteration deal with timing
            mj_step(m, d);
          }

          // in-sync
          else {
            // step while simtime lags behind cputime, and within safefactor
            while ((d->time*GetInstance().slow_down-simsync) < (glfwGetTime()-cpusync) &&
                   (glfwGetTime()-tmstart) < refreshfactor/GetInstance().vmode.refreshRate) {
              // clear old perturbations, apply new
              mju_zero(d->xfrc_applied, 6*m->nbody);
              mjv_applyPerturbPose(m, d, &GetInstance().pert, 0);  // move mocap bodies only
              mjv_applyPerturbForce(m, d, &GetInstance().pert);

              // run mj_step
              mjtNum prevtm = d->time*GetInstance().slow_down;
              mj_step(m, d);

              // break on reset
              if (d->time*GetInstance().slow_down<prevtm) {
                break;
              }
            }
          }
        }

        // paused
        else {
          // apply pose perturbation
          mjv_applyPerturbPose(m, d, &GetInstance().pert, 1);      // move mocap and dynamic bodies

          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
        }
      }
    } // end exclusive access
  }
}
} // end unnamed namespace

//---------------------------------- main -------------------------------------------------

// run event loop
int main(int argc, const char** argv) {
  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // request loadmodel if file given (otherwise drag-and-drop)
  if (argc>1) {
    mju::strcpy_arr(GetInstance().filename, argv[1]);
    GetInstance().loadrequest = 2;
  }

  // start simulation thread
  std::thread simthread(simulate_thread);

  // run event loop
  while (!glfwWindowShouldClose(GetInstance().window) && !GetInstance().exitrequest) {
    { // start exclusive access (block simulation thread)
      const std::lock_guard<std::mutex> lock(GetMutex());

      // load model (not on first pass, to show "loading" label)
      if (GetInstance().loadrequest==1) {
        {
          GetInstance().loadmodel();
          m = GetInstance().m;
          d = GetInstance().d;

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum*) malloc(sizeof(mjtNum)*m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
      } else if (GetInstance().loadrequest>1) {
        GetInstance().loadrequest = 1;
      }

      // handle events (calls all callbacks)
      glfwPollEvents();

      // prepare to render
      GetInstance().prepare();
    } // end exclusive access (allow simulation thread to run)

    // render while simulation is running
    GetInstance().render();
  }

  // stop simulation thread
  GetInstance().exitrequest = 1;
  simthread.join();

  // delete everything we allocated
  GetInstance().clearcallback();
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
  mjv_freeScene(&GetInstance().scn);
  mjr_freeContext(&GetInstance().con);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 0;
}
