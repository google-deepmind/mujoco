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
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <mujoco/mujoco.h>
#include "glfw_dispatch.h"
#include "simulate.h"
#include "array_safety.h"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <dirent.h>
  #include <dlfcn.h>
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using ::mujoco::Glfw;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// control noise variables
mjtNum* ctrlnoise = nullptr;



//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}


#if defined(_WIN32) || defined(__CYGWIN__)
using unique_dlhandle = std::unique_ptr<std::remove_pointer_t<HMODULE>, decltype(&FreeLibrary)>;
#else
using unique_dlhandle = std::unique_ptr<void, decltype(&dlclose)>;
#endif


// scan for libraries in the plugin directory to load additional plugins
std::vector<unique_dlhandle> scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
  const std::string dso_suffix = ".dll";
#else
  const std::string sep = "/";
  #if defined(__APPLE__)
    const std::string dso_suffix = ".dylib";
  #else
    const std::string dso_suffix = ".so";
  #endif
#endif

  // output vectors containing DSO handles
  std::vector<unique_dlhandle> dso_handles;

  // platform-independent routine for checking and printing plugins registered by a dynamic library
  const auto check_and_print_plugins = [&](const std::string& name, unique_dlhandle&& dlhandle) {
    if (!dlhandle) {
      return;
    }

    const int nplugin_new = mjp_pluginCount();
    if (nplugin_new > nplugin) {
      dso_handles.push_back(std::move(dlhandle));

      // print all newly registered plugins
      std::printf("Plugins registered by library '%s':\n", name.c_str());
      for (int i = nplugin; i < nplugin_new; ++i) {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }

      // update counter for plugins registered so far
      nplugin = nplugin_new;
    }
  };

  // try to open the ${EXECDIR}/plugin directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return dso_handles;
  }

  const std::string plugin_dir = getExecutableDir() + sep + "plugin";

#if defined(_WIN32) || defined(__CYGWIN__)
  WIN32_FIND_DATAA find_data;
  HANDLE hfile = FindFirstFileA((plugin_dir + sep + "*.dll").c_str(), &find_data);
  if (!hfile) {
    return dso_handles;
  }

  // go through each file in the directory
  bool keep_going = true;
  while (keep_going) {
    const std::string name(find_data.cFileName);
    // load the library and check for plugins
    const std::string dso_path = plugin_dir + sep + name;
    check_and_print_plugins(
        name, unique_dlhandle(LoadLibraryA(dso_path.c_str()), &FreeLibrary));
    keep_going = FindNextFileA(hfile, &find_data);
  }
  FindClose(hfile);
#else
  DIR* dirp = opendir(plugin_dir.c_str());
  if (!dirp) {
    return dso_handles;
  }

  // go through each entry in the directory
  for (struct dirent* dp; (dp = readdir(dirp));) {
    // only look at regular files (i.e. skip symlinks, pipes, directories, etc.)
    if (dp->d_type == DT_REG) {
      const std::string name(dp->d_name);
      if (name.size() > dso_suffix.size() &&
          name.substr(name.size() - dso_suffix.size()) == dso_suffix) {
        // load the library and check for plugins
        const std::string dso_path = plugin_dir + sep + name;
        check_and_print_plugins(
            name, unique_dlhandle(dlopen(dso_path.c_str(), RTLD_NOW | RTLD_LOCAL), &dlclose));
      }
    }
  }
  closedir(dirp);
#endif

  return dso_handles;
}


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
  double syncCPU = 0;
  mjtNum syncSim = 0;

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
      // lock the sim mutex
      const std::lock_guard<std::mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          // record cpu time at start of iteration
          double startCPU = Glfw().glfwGetTime();

          // elapsed CPU and simulation time since last sync
          double elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // inject noise
          if (sim.ctrlnoisestd) {
            // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
            mjtNum rate = mju_exp(-m->opt.timestep / sim.ctrlnoiserate);
            mjtNum scale = sim.ctrlnoisestd * mju_sqrt(1-rate*rate);

            for (int i=0; i<m->nu; i++) {
              // update noise
              ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

              // apply noise
              d->ctrl[i] = ctrlnoise[i];
            }
          }

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.realTimeIndex];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned = mju_abs(elapsedCPU/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU < 0 || syncCPU == 0 || misaligned || sim.speedChanged) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speedChanged = false;

            // clear old perturbations, apply new
            mju_zero(d->xfrc_applied, 6*m->nbody);
            sim.applyposepertubations(0);  // move mocap bodies only
            sim.applyforceperturbations();

            // run single step, let next iteration deal with timing
            mj_step(m, d);
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;
            double refreshTime = simRefreshFraction/sim.refreshRate;

            // step while sim lags behind cpu and within refreshTime
            while ((d->time - syncSim)*slowdown < (Glfw().glfwGetTime()-syncCPU) &&
                   (Glfw().glfwGetTime()-startCPU) < refreshTime) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measuredSlowdown = elapsedCPU / elapsedSim;
                measured = true;
              }

              // clear old perturbations, apply new
              mju_zero(d->xfrc_applied, 6*m->nbody);
              sim.applyposepertubations(0);  // move mocap bodies only
              sim.applyforceperturbations();

              // call mj_step
              mj_step(m, d);

              // break if reset
              if (d->time < prevSim) {
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
    }  // release std::lock_guard<std::mutex>
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

  // scan for libraries in the plugin directory to load additional plugins
  std::vector<unique_dlhandle> dso_handles = scanPluginLibraries();

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
