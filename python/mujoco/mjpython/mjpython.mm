// Copyright 2023 DeepMind Technologies Limited
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

#import <atomic>
#import <cstdlib>
#import <iostream>

#import <dlfcn.h>
#import <pthread.h>
#import <sys/resource.h>

#import <Cocoa/Cocoa.h>
#import <Python.h>

// Wrap Objective-C Cocoa calls into C-style functions with default visibility,
// so that we can dlsym and call them from Python via ctypes.
extern "C" {
__attribute__((used)) void mjpython_hide_dock_icon() {
  [NSApp setActivationPolicy:NSApplicationActivationPolicyAccessory];
}
__attribute__((used)) void mjpython_show_dock_icon() {
  [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
}
}

// TODO(b/273744079): Remove Python 3.7 code after end-of-life (27 Jun 2023).
namespace {
struct {
#define CPYTHON_FN(fname) decltype(&::fname) fname

#if PY_MINOR_VERSION >= 8
  CPYTHON_FN(Py_InitializeFromConfig);
  CPYTHON_FN(Py_RunMain);
  CPYTHON_FN(PyConfig_Clear);
  CPYTHON_FN(PyConfig_InitPythonConfig);
  CPYTHON_FN(PyConfig_SetBytesArgv);
#else
  CPYTHON_FN(Py_DecodeLocale);
  CPYTHON_FN(Py_Initialize);
  CPYTHON_FN(Py_Main);
  CPYTHON_FN(PyMem_RawFree);
  CPYTHON_FN(Py_SetProgramName);
#endif

  // go/keep-sorted start
  CPYTHON_FN(Py_FinalizeEx);
  CPYTHON_FN(PyGILState_Ensure);
  CPYTHON_FN(PyGILState_Release);
  CPYTHON_FN(PyRun_SimpleStringFlags);
  // go/keep-sorted end

#undef CPYTHON_FN
} cpython;

std::atomic_bool py_initialized = false;

struct Args {
  int argc;
  char** argv;
};

// The Python main thread (distinct from the macOS main thread, which executes the main function).
void* mjpython_pymain(void* vargs) {
  Args* args = static_cast<Args*>(vargs);
  PyGILState_STATE gil;

  // Initialize the Python interpreter.
#if PY_MINOR_VERSION >= 8
  PyConfig config;
  cpython.PyConfig_InitPythonConfig(&config);
  cpython.PyConfig_SetBytesArgv(&config, args->argc, args->argv);
  cpython.Py_InitializeFromConfig(&config);
  cpython.PyConfig_Clear(&config);
#else
  // Convert each argv to wchar_t* (needed for Py_Main).
  wchar_t** wargv = static_cast<wchar_t**>(std::calloc(args->argc, sizeof(wchar_t*)));
  for (int i = 0; i < args->argc; ++i) {
    wargv[i] = cpython.Py_DecodeLocale(args->argv[i], nullptr);
  }
  cpython.Py_SetProgramName(wargv[0]);
  cpython.Py_Initialize();
#endif

  // Set up the condition variable to pass control back to the macOS main thread.
  gil = cpython.PyGILState_Ensure();
  cpython.PyRun_SimpleStringFlags("import threading; cond = threading.Condition()", nullptr);
  py_initialized.store(true);

  // Wait until GLFW is initialized on macOS main thread, set up the queue and an atexit hook
  // to enqueue a termination flag upon exit.
  cpython.PyRun_SimpleStringFlags(R"(
import atexit

# The mujoco.viewer module should only be imported here after glfw.init() in the macOS main thread.
with cond:
  cond.wait()
import mujoco.viewer

# Similar to a queue.Queue(maxsize=1), but where only one active task is allowed at a time.
# With queue.Queue(1), another item is allowed to be enqueued before task_done is called.
class _MjPythonImpl(mujoco.viewer._MjPythonBase):

  # Termination statuses
  NOT_TERMINATED = 0
  TERMINATION_REQUESTED = 1
  TERMINATION_ACCEPTED = 2
  TERMINATED = 3

  def __init__(self):
    self._cond = threading.Condition()
    self._model_data = None
    self._termination = self.__class__.NOT_TERMINATED
    self._busy = False

  def launch_on_ui_thread(self, model, data):
    with self._cond:
      if self._busy or self._model_data is not None:
        raise RuntimeError('another MuJoCo viewer is already open')
      else:
        self._model_data = (model, data)
        self._cond.notify()

  def terminate(self):
    with self._cond:
      self._termination = self.__class__.TERMINATION_REQUESTED
      self._cond.notify()
      self._cond.wait_for(
          lambda: self._termination == self.__class__.TERMINATED)

  def get(self):
    with self._cond:
      self._cond.wait_for(
          lambda: self._model_data is not None or self._termination)

      if self._termination:
        if self._termination == self.__class__.TERMINATION_REQUESTED:
          self._termination = self.__class__.TERMINATION_ACCEPTED
        return None

      model_data = self._model_data
      self._busy = True
      self._model_data = None
      return model_data

  def done(self):
    with self._cond:
      self._busy = False
      if self._termination == self.__class__.TERMINATION_ACCEPTED:
        self._termination = self.__class__.TERMINATED
      self._cond.notify()


mujoco.viewer._MJPYTHON = _MjPythonImpl()
atexit.register(mujoco.viewer._MJPYTHON.terminate)
del _MjPythonImpl  # Don't pollute globals for user script.

with cond:
  cond.notify()
del cond  # Don't pollute globals for user script.
)", nullptr);

  // Run the Python interpreter main loop.
#if PY_MINOR_VERSION >= 8
  cpython.Py_RunMain();
#else
  cpython.Py_Main(args->argc, wargv);
#endif

  // Tear down the interpreter.
  cpython.Py_FinalizeEx();
#if PY_MINOR_VERSION < 8
  for (int i = 0; i < args->argc; ++i) {
    cpython.PyMem_RawFree(wargv[i]);
    wargv[i] = nullptr;
  }
  std::free(wargv);
  wargv = nullptr;
#endif
  return nullptr;
}
}  // namespace

int main(int argc, char** argv) {
  const char* libpython_path = getenv("MJPYTHON_LIBPYTHON");
  if (!libpython_path || !libpython_path[0]) {
    std::cerr << "This binary must be launched via the mjpython.py script.\n";
    return 1;
  }

  // Resolve libpython at runtime to prevent linking against the wrong dylib. The correct libpython
  // path is passed from a Python trampoline script, which ran inside the desired interpreter and
  // exec'd this binary.
  void* libpython = dlopen(libpython_path, RTLD_NOW | RTLD_GLOBAL);

  // Look up required CPython API functions from table of symbols already loaded into the process.
#define CPYTHON_INITFN(fname) \
    cpython.fname = reinterpret_cast<decltype(cpython.fname)>(dlsym(libpython, #fname))

#if PY_MINOR_VERSION >= 8
  CPYTHON_INITFN(Py_InitializeFromConfig);
  CPYTHON_INITFN(Py_RunMain);
  CPYTHON_INITFN(PyConfig_Clear);
  CPYTHON_INITFN(PyConfig_InitPythonConfig);
  CPYTHON_INITFN(PyConfig_SetBytesArgv);
#else
  CPYTHON_INITFN(Py_DecodeLocale);
  CPYTHON_INITFN(Py_Initialize);
  CPYTHON_INITFN(Py_Main);
  CPYTHON_INITFN(PyMem_RawFree);
  CPYTHON_INITFN(Py_SetProgramName);
#endif

  // go/keep-sorted start
  CPYTHON_INITFN(Py_FinalizeEx);
  CPYTHON_INITFN(PyGILState_Ensure);
  CPYTHON_INITFN(PyGILState_Release);
  CPYTHON_INITFN(PyRun_SimpleStringFlags);
  // go/keep-sorted end

#undef CPYTHON_INITFN

  // Package up argc and argv together to pass to pthread_create.
  Args args{argc, argv};

  // Create a thread to be used as the "Python main thread".
  pthread_t pymain_thread = [&args]() {
    // Set the stack size of the Python main thread to be the same as the OS main thread.
    // (e.g. the default pthread stack size is too small to import NumPy)
    rlimit limit;
    getrlimit(RLIMIT_STACK, &limit);

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, limit.rlim_cur);

    pthread_t thread;
    pthread_create(&thread, &attr, &mjpython_pymain, &args);
    return thread;
  }();

  // Busy-wait until Python interpreter is initialized.
  while (!py_initialized.load()) {}

  // Initialize GLFW on the macOS main thread, yield control to Python main thread and wait for it
  // to finish setting up _MJPYTHON, then serve incoming viewer launch requests.
  PyGILState_STATE gil = cpython.PyGILState_Ensure();
  cpython.PyRun_SimpleStringFlags(R"(
import ctypes

# GLFW must be initialized on the OS main thread (i.e. here).
import glfw
import mujoco.viewer

glfw.init()
glfw.poll_events()
ctypes.CDLL(None).mjpython_hide_dock_icon()

# Wait for Python main thread to finish setting up _MJPYTHON
with cond:
  cond.notify()
  cond.wait()

while True:
  try:
    # Wait for an incoming payload.
    payload = mujoco.viewer._MJPYTHON.get()

    # None means that we are exiting.
    if payload is None:
      glfw.terminate()
      break

    # Otherwise, launch the viewer.
    model, data = payload
    ctypes.CDLL(None).mjpython_show_dock_icon()
    mujoco.viewer._launch_internal(model, data, run_physics_thread=False)
    ctypes.CDLL(None).mjpython_hide_dock_icon()

  finally:
    mujoco.viewer._MJPYTHON.done()
)", nullptr);
  cpython.PyGILState_Release(gil);

  // Tear everything down.
  pthread_join(pymain_thread, nullptr);
  dlclose(libpython);
}
