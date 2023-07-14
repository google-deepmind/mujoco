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
double accuracy_broad[maxthread];
double accuracy_mid[maxthread];
int contacts[maxthread];
int constraints[maxthread];
double simtime[maxthread];


// timer
std::chrono::system_clock::time_point tm_start;
mjtNum gettm(void) {
  std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - tm_start;
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
  accuracy_broad[id] = 0;
  accuracy_mid[id] = 0;

  // run and time
  double start = gettm();
  for (int i=0; i<nstep; i++) {
    // inject pseuso-random control noise
    mju_copy(d[id]->ctrl, ctrl + i*m->nu, m->nu);

    // advance simulation
    mj_step(m, d[id]);

    // accumulate statistics
    contacts[id] += d[id]->ncon;
    constraints[id] += d[id]->nefc;
    if (d[id]->nbodypair_broad) {
      accuracy_broad[id] += (100.0*d[id]->nbodypair_narrow)/d[id]->nbodypair_broad;
    } else {
      accuracy_broad[id] += 100;
    }
    if (d[id]->ngeompair_mid) {
      accuracy_mid[id] += (100.0*d[id]->nbodypair_narrow)/d[id]->ngeompair_mid;
    } else {
      accuracy_mid[id] += 100;
    }
  }
  simtime[id] = gettm() - start;
}


// main function
int main(int argc, char** argv) {

  // print help if arguments are missing
  if (argc < 2 || argc > 6) {
    return finish("\n Usage:  testspeed modelfile [nstep nthread ctrlnoise profile]\n");
  }

  // read arguments
  int nstep = 10000, nthread = 0, profile = 1;
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
  if (argc > 5 && std::sscanf(argv[5], "%d", &profile) != 1) {
    return finish("Invalid profile argument");
  }

  // clamp ctrlnoise to [0.0, 1.0]
  ctrlnoise = mjMAX(0.0, mjMIN(ctrlnoise, 1.0));

  // clamp nthread to [1, maxthread]
  nthread = mjMAX(1, mjMIN(maxthread, nthread));

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
  for (int id=0; id<nthread; id++) {
    d[id] = mj_makeData(m);
    if (!d[id]) {
      return finish("Could not allocate mjData", m);
    }

    // init to keyframe "test" if present
    if (testkey>=0) {
      mju_copy(d[id]->qpos, m->key_qpos + testkey*m->nq, m->nq);
      mju_copy(d[id]->qvel, m->key_qvel + testkey*m->nv, m->nv);
      mju_copy(d[id]->act,  m->key_act  + testkey*m->na, m->na);
    }
  }

  // install timer callback for profiling if requested
  tm_start = std::chrono::system_clock::now();
  if (profile) {
    mjcb_time = gettm;
  }

  // print start
  if (nthread>1) {
    std::printf("\nRunning %d steps per thread at dt = %g ...\n\n", nstep, m->opt.timestep);
  } else {
    std::printf("\nRunning %d steps at dt = %g ...\n\n", nstep, m->opt.timestep);
  }

  // create pseudo-random control sequence
  std::vector<mjtNum> ctrl = CtrlNoise(m, nstep, ctrlnoise);

  // run simulation, record total time
  std::thread th[maxthread];
  double starttime = gettm();
  for (int id=0; id<nthread; id++) {
    th[id] = std::thread(simulate, id, nstep, ctrl.data());
  }
  for (int id=0; id<nthread; id++) {
    th[id].join();
  }
  double tottime = gettm() - starttime;

  // all-thread summary
  constexpr char mu_str[3] = "\u00B5";  // unicode mu character
  if (nthread>1) {
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
  std::printf(" Broadphase accuracy  : %.2f%%\n", accuracy_broad[0]/nstep);
  std::printf(" Midphase accuracy    : %.2f%%\n", accuracy_mid[0]/nstep);
  std::printf(" Contacts per step    : %.2f\n", static_cast<float>(contacts[0])/nstep);
  std::printf(" Constraints per step : %.2f\n", static_cast<float>(constraints[0])/nstep);
  std::printf(" Degrees of freedom   : %d\n\n", m->nv);

  // profiler results for thread 0
  if (profile) {
    printf(" Internal profiler for thread 0 (%ss per step)\n", mu_str);
    mjtNum tstep = d[0]->timer[mjTIMER_STEP].duration/d[0]->timer[mjTIMER_STEP].number;
    mjtNum components = 0, total = 0;
    for (int i=0; i<mjNTIMER; i++) {
      if (d[0]->timer[i].number > 0) {
        mjtNum istep = d[0]->timer[i].duration/d[0]->timer[i].number;
        std::printf(" %16s : %6.1f  (%6.2f %%)\n", mjTIMERSTRING[i], 1e6*istep, 100*istep/tstep);

        // save step time, add up timing of components
        if (i == 0) total = istep;
        if (i >= mjTIMER_POSITION && i <= mjTIMER_CONSTRAINT) {
          components += istep;
        }
      }
    }

    // compute "other" (computation not covered by timers)
    if (tstep > 0) {
      mjtNum other = total - components;
      std::printf(" %16s : %6.1f  (%6.2f %%)\n", "other", 1e6*other, 100*other/tstep);
    }
  }

  // free per-thread data
  for (int id=0; id<nthread; id++) {
    mj_deleteData(d[id]);
  }

  // finalize
  return finish();
}
