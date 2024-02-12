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
#include <ratio>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>


// maximum number of threads
const int maxthread = 512;

// model and per-thread data
mjModel* m = NULL;
mjData* d[maxthread];


// per-thread statistics
int contacts[maxthread];
int constraints[maxthread];
double simtime[maxthread];


// timer
std::chrono::steady_clock::time_point tm_start;
mjtNum gettm(void) {
  std::chrono::duration<double, std::micro> elapsed;
  elapsed = std::chrono::steady_clock::now() - tm_start;
  return elapsed.count();
}


// deallocate and print message
int finish(const char* msg = NULL, mjModel* m = NULL) {
  // deallocate model
  if (m) {
    mj_deleteModel(m);
  }

  // print message
  if (msg) {
    std::printf("%s\n", msg);
  }

  return 0;
}

std::vector<mjtNum> CtrlNoise(const mjModel* m, int nsteps, mjtNum ctrlnoise) {
  std::vector<mjtNum> ctrl;
  for (int step=0; step < nsteps; step++) {
    for (int i = 0; i < m->nu; i++) {
      mjtNum center = 0.0;
      mjtNum radius = 1.0;
      mjtNum* range = m->actuator_ctrlrange + 2 * i;
      if (m->actuator_ctrllimited[i]) {
        center = (range[1] + range[0]) / 2;
        radius = (range[1] - range[0]) / 2;
      }
      radius *= ctrlnoise;
      ctrl.push_back(center + radius * (2 * mju_Halton(step, i+2) - 1));
    }
  }
  return ctrl;
}


// thread function
void simulate(int id, int nstep, mjtNum* ctrl) {
  // clear statistics
  contacts[id] = 0;
  constraints[id] = 0;

  // run and time
  double start = gettm();
  for (int i=0; i < nstep; i++) {
    // inject pseudo-random control noise
    mju_copy(d[id]->ctrl, ctrl + i*m->nu, m->nu);

    // advance simulation
    mj_step(m, d[id]);

    // accumulate statistics
    contacts[id] += d[id]->ncon;
    constraints[id] += d[id]->nefc;
  }
  simtime[id] = 1e-6 * (gettm() - start);
}


// main function
int main(int argc, char** argv) {

  // print help if arguments are missing
  if (argc < 2 || argc > 6) {
    return finish(
      "\n"
      "Usage:  testspeed modelfile [nstep nthread ctrlnoise npoolthread]\n"
      "\n"
      "  argument      default     semantic\n"
      "  --------      -------     --------\n"
      "  modelfile                 path to model (required)\n"
      "  nstep         10000       number of steps per rollout\n"
      "  nthread       1           number of threads running parallel rollouts\n"
      "  ctrlnoise     0.01        scale of pseudo-random noise injected into actuators\n"
      "  npoolthread   0           number of threads in engine-internal threadpool\n"
      "\n"
      "Note: If the model has a keyframe named \"test\", it will be loaded prior to simulation\n");
  }

  // read arguments
  int nstep = 10000, nthread = 0, npoolthread = 0;
  // inject small noise by default, to avoid fixed contact state
  mjtNum ctrlnoise = 0.01;
  if (argc > 2 && (std::sscanf(argv[2], "%d", &nstep) != 1 || nstep <= 0)) {
    return finish("Invalid nstep argument");
  }
  if (argc > 3 && std::sscanf(argv[3], "%d", &nthread) != 1) {
    return finish("Invalid nthread argument");
  }
  if (argc > 4 && std::sscanf(argv[4], "%lf", &ctrlnoise) != 1) {
    return finish("Invalid ctrlnoise argument");
  }
  if (argc > 5 && std::sscanf(argv[5], "%d", &npoolthread) != 1) {
    return finish("Invalid npoolthread argument");
  }

  // clamp ctrlnoise to [0.0, 1.0]
  ctrlnoise = mjMAX(0.0, mjMIN(ctrlnoise, 1.0));

  // clamp nthread to [1, maxthread]
  nthread = mjMAX(1, mjMIN(maxthread, nthread));
  npoolthread = mjMAX(1, mjMIN(maxthread, npoolthread));

  // get filename, determine file type
  std::string filename(argv[1]);
  bool binary = (filename.find(".mjb") != std::string::npos);

  // load model
  char error[1000] = "Could not load binary model";
  if (binary) {
    m = mj_loadModel(argv[1], 0);
  } else {
    m = mj_loadXML(argv[1], 0, error, 1000);
  }
  if (!m) {
    return finish(error);
  }

  // make per-thread data
  int testkey = mj_name2id(m, mjOBJ_KEY, "test");
  for (int id=0; id < nthread; id++) {
    // make mjData(s)
    d[id] = mj_makeData(m);
    if (!d[id]) {
      return finish("Could not allocate mjData", m);
    }

    // reset to keyframe
    if (testkey >= 0) {
      mj_resetDataKeyframe(m, d[id], testkey);
    }

    // make and bind threadpool
    if (npoolthread > 1) {
      mjThreadPool* threadpool = mju_threadPoolCreate(npoolthread);
      mju_bindThreadPool(d[id], threadpool);
    }
  }

  // install timer callback for profiling
  mjcb_time = gettm;

  // print start
  std::printf("\nRolling out %d steps%s, at dt = %g",
              nstep,
              nthread > 1 ? " per thread" : "",
              m->opt.timestep);
  if (npoolthread > 1) {
    std::printf(", using %d threads for engine-internal threadpool", npoolthread);
  }
  std::printf("...\n\n");

  // create pseudo-random control sequence
  std::vector<mjtNum> ctrl = CtrlNoise(m, nstep, ctrlnoise);

  // run simulation, record total time
  std::thread th[maxthread];
  double starttime = gettm();
  for (int id=0; id < nthread; id++) {
    th[id] = std::thread(simulate, id, nstep, ctrl.data());
  }
  for (int id=0; id < nthread; id++) {
    th[id].join();
  }
  double tottime = 1e-6 * (gettm() - starttime);  // total time, in seconds

  // all-thread summary
  constexpr char mu_str[3] = "\u00B5";  // unicode mu character
  if (nthread > 1) {
    std::printf("Summary for all %d threads\n\n", nthread);
    std::printf(" Total simulation time  : %.2f s\n", tottime);
    std::printf(" Total steps per second : %.0f\n", nthread*nstep/tottime);
    std::printf(" Total realtime factor  : %.2f x\n", nthread*nstep*m->opt.timestep/tottime);
    std::printf(" Total time per step    : %.1f %ss\n\n", 1e6*tottime/(nthread*nstep), mu_str);

    std::printf("Details for thread 0\n\n");
  }

  // details for thread 0
  std::printf(" Simulation time      : %.2f s\n", simtime[0]);
  std::printf(" Steps per second     : %.0f\n", nstep/simtime[0]);
  std::printf(" Realtime factor      : %.2f x\n", nstep*m->opt.timestep/simtime[0]);
  std::printf(" Time per step        : %.1f %ss\n\n", 1e6*simtime[0]/nstep, mu_str);
  std::printf(" Contacts per step    : %.2f\n", static_cast<float>(contacts[0])/nstep);
  std::printf(" Constraints per step : %.2f\n", static_cast<float>(constraints[0])/nstep);
  std::printf(" Degrees of freedom   : %d\n\n", m->nv);

  // profiler, top-level
  printf(" Internal profiler%s, %ss per step\n", nthread > 1 ? " for thread 0" : "", mu_str);
  int number = d[0]->timer[mjTIMER_STEP].number;
  mjtNum tstep = number ? d[0]->timer[mjTIMER_STEP].duration/number : 0.0;
  mjtNum components = 0, total = 0;
  for (int i=0; i <= mjTIMER_ADVANCE; i++) {
    if (d[0]->timer[i].number > 0) {
      int number = d[0]->timer[i].number;
      mjtNum istep = number ? d[0]->timer[i].duration/number : 0.0;
      mjtNum percent = number ? 100*istep/tstep : 0.0;
      std::printf(" %17s : %6.1f  (%6.2f %%)\n", mjTIMERSTRING[i], istep, percent);

      // save step time, add up timing of components
      if (i == 0) total = istep;
      if (i >= mjTIMER_POSITION) {
        components += istep;
      }
    }
  }

  // "other" (computation not covered by timers)
  if (tstep > 0) {
    mjtNum other = total - components;
    std::printf(" %17s : %6.1f  (%6.2f %%)\n", "other", other, 100*other/tstep);
  }

  std::printf("\n");

  // mjTIMER_POSITION and its components
  for (int i : {mjTIMER_POSITION,
                mjTIMER_POS_KINEMATICS,
                mjTIMER_POS_INERTIA,
                mjTIMER_POS_COLLISION,
                mjTIMER_POS_MAKE,
                mjTIMER_POS_PROJECT}) {
    if (d[0]->timer[i].number > 0) {
      mjtNum istep = d[0]->timer[i].duration/d[0]->timer[i].number;
      if (i == mjTIMER_POSITION) {
        std::printf("   position total  : %6.1f  (%6.2f %%)\n",  istep, 100*istep/tstep);
      } else {
        std::printf("     %-10s    : %6.1f  (%6.2f %%)\n",
                    mjTIMERSTRING[i]+4, istep, 100*istep/tstep);
      }
    }

    // components of mjTIMER_POS_COLLISION
    if (i == mjTIMER_POS_COLLISION) {
      for (int j : {mjTIMER_COL_BROAD, mjTIMER_COL_NARROW}) {
        int number = d[0]->timer[j].number;
        mjtNum jstep = number ? d[0]->timer[j].duration/number : 0.0;
        mjtNum percent = number ? 100*jstep/tstep : 0.0;
        std::printf("       %-11s : %6.1f  (%6.2f %%)\n", mjTIMERSTRING[j]+4, jstep, percent);
      }
    }
  }


  // free per-thread data
  for (int id=0; id < nthread; id++) {
    mjThreadPool* threadpool = (mjThreadPool*) d[id]->threadpool;
    mj_deleteData(d[id]);
    if (threadpool) {
      mju_threadPoolDestroy(threadpool);
    }
  }

  // finalize
  return finish();
}
