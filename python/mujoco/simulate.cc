// Copyright 2022 DeepMind Technologies Limited
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

#include <atomic>
#include <chrono>  // NOLINT(build/c++11)
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>  // NOLINT(build/c++11)
#include <tuple>
#include <utility>
#include <vector>

#include <glfw_adapter.h>
#include <glfw_dispatch.h>
#include <simulate.h>
#include <Python.h>
#include "errors.h"
#include "indexers.h"
#include "structs.h"
#include <pybind11/gil.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace mujoco::python {
namespace {
using UIAdapter = mujoco::GlfwAdapter;
namespace py = ::pybind11;

template <typename T, int N>
constexpr inline std::size_t sizeof_arr(const T (&arr)[N]) {
  return sizeof(arr);
}

template <typename Adapter>
class UIAdapterWithPyCallback : public Adapter {
 public:
  template <typename... Args>
  UIAdapterWithPyCallback(py::handle key_callback, Args&&... args)
      : Adapter(std::forward<Args>(args)...) {
    if (!key_callback.is_none()) {
      Py_XINCREF(key_callback.ptr());
      key_callback_ = key_callback.ptr();
    }
  }

  ~UIAdapterWithPyCallback() override { Py_XDECREF(key_callback_); }

 protected:
  void OnKey(int key, int scancode, int act) override {
    Adapter::OnKey(key, scancode, act);
    if (this->IsKeyDownEvent(act) && key_callback_) {
      py::gil_scoped_acquire gil;
      (py::handle(key_callback_))(this->last_key_);
    }
  }

 private:
  PyObject* key_callback_ = nullptr;
};

class SimulateWrapper {
 public:
  SimulateWrapper(std::unique_ptr<PlatformUIAdapter> platform_ui_adapter,
                  py::object cam, py::object opt, py::object pert,
                  py::object user_scn, bool is_passive)
      : simulate_(new mujoco::Simulate(
            std::move(platform_ui_adapter), cam.cast<MjvCameraWrapper&>().get(),
            opt.cast<MjvOptionWrapper&>().get(),
            pert.cast<MjvPerturbWrapper&>().get(), is_passive)),
        m_(py::none()),
        d_(py::none()),
        cam_(cam),
        opt_(opt),
        pert_(pert),
        user_scn_(user_scn) {
    if (!user_scn.is_none()) {
      simulate_->user_scn = user_scn_.cast<MjvSceneWrapper&>().get();
    }
  }

  ~SimulateWrapper() { Destroy(); }

  void Destroy() {
    if (simulate_) {
      delete simulate_;
      simulate_ = nullptr;
      destroyed_.store(1);
    }
  }

  void WaitUntilExit() {
    // TODO: replace with atomic wait when we migrate to C++20
    while (simulate_ && simulate_->exitrequest.load() != 2) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void Load(py::object m, py::object d, const std::string& path) {
    if (!simulate_) {
      return;
    }

    mjModel* m_raw = m.cast<MjModelWrapper&>().get();
    mjData* d_raw = d.cast<MjDataWrapper&>().get();
    {
      py::gil_scoped_release no_gil;
      simulate_->Load(m_raw, d_raw, path.c_str());
    }
    m_ = m;
    d_ = d;
    m_raw_ = m_raw;
    d_raw_ = d_raw;
  }

  mujoco::Simulate* simulate() { return simulate_; }

  py::object GetModel() const { return m_; }
  py::object GetData() const { return d_; }

  mjrRect GetViewport() const {
    // Return the viewport corresponding to the 3D view, i.e. the viewer window
    // without the UI elements.
    return simulate_->uistate.rect[3];
  }

  void SetFigures(
      const std::vector<std::pair<mjrRect, py::object>>& viewports_figures) {

    // TODO: replace with atomic wait when we migrate to C++20
    while (simulate_ && simulate_->newfigurerequest.load() != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Pairs of [viewport, figure], where viewport corresponds to the location
    // of the figure on the viewer window.
    for (const auto& [viewport, figure] : viewports_figures) {
      mjvFigure casted_figure = *figure.cast<MjvFigureWrapper&>().get();
      simulate_->user_figures_new_.push_back(std::make_pair(viewport, casted_figure));
    }

    int value = 0;
    simulate_->newfigurerequest.compare_exchange_strong(value, 1);
  }

  void ClearFigures() {
    // TODO: replace with atomic wait when we migrate to C++20
    while (simulate_ && simulate_->newfigurerequest.load() != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    simulate_->user_figures_new_.clear();

    int value = 0;
    simulate_->newfigurerequest.compare_exchange_strong(value, 1);
  }

  void SetTexts(
      const std::vector<std::tuple<int, int, std::string, std::string>>&
          texts) {
    // TODO: replace with atomic wait when we migrate to C++20
    while (simulate_ && simulate_->newtextrequest.load() != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Collection of [font, gridpos, text1, text2] tuples for overlay text
    for (const auto& [font, gridpos, text1, text2] : texts) {
      simulate_->user_texts_new_.push_back(std::make_tuple(font, gridpos, text1, text2));
    }

    int value = 0;
    simulate_->newtextrequest.compare_exchange_strong(value, 1);
  }

  void ClearTexts() {
    // TODO: replace with atomic wait when we migrate to C++20
    while (simulate_ && simulate_->newtextrequest.load() != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    simulate_->user_texts_new_.clear();

    int value = 0;
    simulate_->newtextrequest.compare_exchange_strong(value, 1);
  }

  void SetImages(
    const std::vector<std::tuple<mjrRect, pybind11::array&>> viewports_images
  ) {
    // TODO: replace with atomic wait when we migrate to C++20
    while (simulate_ && simulate_->newimagerequest.load() != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    for (const auto& [viewport, image] : viewports_images) {
      auto buf = image.request();
      if (buf.ndim != 3) {
        throw std::invalid_argument("image must have 3 dimensions (H, W, C)");
      }
      if (static_cast<int>(buf.shape[2]) != 3) {
        throw std::invalid_argument("image must have 3 channels");
      }
      if (buf.itemsize != sizeof(unsigned char)) {
        throw std::invalid_argument("image must be uint8 format");
      }

      // Calculate size of the image data
      size_t height = buf.shape[0];
      size_t width = buf.shape[1];
      size_t size = height * width * 3;

      // Make a copy of the image data since Python is
      // not required to keep it
      std::unique_ptr<unsigned char[]> image_copy(new unsigned char[size]());
      std::memcpy(image_copy.get(), buf.ptr, size);

      simulate_->user_images_new_.push_back(std::make_tuple(viewport, std::move(image_copy)));
    }

    int value = 0;
    simulate_->newimagerequest.compare_exchange_strong(value, 1);
  }

  void ClearImages() {
    // TODO: replace with atomic wait when we migrate to C++20
    while (simulate_ && simulate_->newimagerequest.load() != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    simulate_->user_images_new_.clear();

    int value = 0;
    simulate_->newimagerequest.compare_exchange_strong(value, 1);
  }

 private:
  mujoco::Simulate* simulate_;
  std::atomic_int destroyed_ = 0;

  // Hold references to keep these Python objects alive for as long as the
  // simulate object.
  py::object m_;
  py::object d_;
  py::object cam_;
  py::object opt_;
  py::object pert_;
  py::object user_scn_;

  mjModel* m_raw_ = nullptr;
  mjData* d_raw_ = nullptr;
};

inline mujoco::Simulate& SimulateRefOrThrow(SimulateWrapper& wrapper) {
  auto* sim = wrapper.simulate();
  if (!sim) {
    throw UnexpectedError("simulate object is already deleted");
  }
  return *sim;
}

template <typename T, typename... Args>
inline auto CallIfNotNull(T (*func)(mujoco::Simulate&, Args...)) {
  return [func](SimulateWrapper& wrapper, Args&&... args) {
    return func(SimulateRefOrThrow(wrapper), std::forward<Args>(args)...);
  };
}

template <typename... Args>
inline auto CallIfNotNull(void (*func)(mujoco::Simulate&, Args...)) {
  return [func](SimulateWrapper& wrapper, Args&&... args) -> void {
    func(SimulateRefOrThrow(wrapper), std::forward<Args>(args)...);
  };
}

template <typename... Args>
inline auto CallIfNotNull(void (mujoco::Simulate::*func)(Args...)) {
  return [func](SimulateWrapper& wrapper, Args&&... args) -> void {
    (SimulateRefOrThrow(wrapper).*func)(std::forward<Args>(args)...);
  };
}

template <typename T>
inline auto GetIfNotNull(T mujoco::Simulate::* member) {
  return [member](SimulateWrapper& wrapper) -> T& {
    return SimulateRefOrThrow(wrapper).*member;
  };
}

template <typename T, typename... Args>
inline auto SetIfNotNull(T mujoco::Simulate::* member) {
  return [member](SimulateWrapper& wrapper, const T& value) -> void {
    SimulateRefOrThrow(wrapper).*member = value;
  };
}

PYBIND11_MODULE(_simulate, pymodule) {
  py::class_<SimulateMutex>(pymodule, "Mutex")
      .def(
          "__enter__", [](SimulateMutex& mtx) { mtx.lock(); },
          py::call_guard<py::gil_scoped_release>())
      .def(
          "__exit__",
          [](SimulateMutex& mtx, py::handle, py::handle, py::handle) {
            mtx.unlock();
          },
          py::call_guard<py::gil_scoped_release>());

  py::class_<SimulateWrapper>(pymodule, "Simulate")
      .def_readonly_static("MAX_GEOM", &mujoco::Simulate::kMaxGeom)
      .def(py::init([](py::object scn, py::object cam, py::object opt,
                       py::object pert, bool run_physics_thread,
                       py::object key_callback) {
        bool is_passive = !run_physics_thread;
        return std::make_unique<SimulateWrapper>(
            std::make_unique<UIAdapterWithPyCallback<UIAdapter>>(key_callback),
            scn, cam, opt, pert, is_passive);
      }))
      .def("destroy", &SimulateWrapper::Destroy)
      .def("load_message", CallIfNotNull(&mujoco::Simulate::LoadMessage),
           py::call_guard<py::gil_scoped_release>())
      .def("load", &SimulateWrapper::Load)
      .def("load_message_clear",
           CallIfNotNull(&mujoco::Simulate::LoadMessageClear),
           py::call_guard<py::gil_scoped_release>())
      .def("sync", CallIfNotNull(&mujoco::Simulate::Sync),
           py::call_guard<py::gil_scoped_release>())
      .def("add_to_history", CallIfNotNull(&mujoco::Simulate::AddToHistory),
           py::call_guard<py::gil_scoped_release>())
      .def("render_loop", CallIfNotNull(&mujoco::Simulate::RenderLoop),
           py::call_guard<py::gil_scoped_release>())
      .def("lock", GetIfNotNull(&mujoco::Simulate::mtx),
           py::call_guard<py::gil_scoped_release>(),
           py::return_value_policy::reference_internal)
      .def("set_figures", &SimulateWrapper::SetFigures,
           py::arg("viewports_figures"))
      .def("clear_figures", &SimulateWrapper::ClearFigures)
      .def("set_texts", &SimulateWrapper::SetTexts, py::arg("overlay_texts"))
      .def("clear_texts", &SimulateWrapper::ClearTexts)
      .def("set_images", &SimulateWrapper::SetImages,
           py::arg("viewports_images"))
      .def("clear_images", &SimulateWrapper::ClearImages)
      .def_property_readonly("m", &SimulateWrapper::GetModel)
      .def_property_readonly("d", &SimulateWrapper::GetData)
      .def_property_readonly("viewport", &SimulateWrapper::GetViewport)
      .def_property_readonly("ctrl_noise_std",
                             GetIfNotNull(&mujoco::Simulate::ctrl_noise_std),
                             py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("ctrl_noise_rate",
                             GetIfNotNull(&mujoco::Simulate::ctrl_noise_rate),
                             py::call_guard<py::gil_scoped_release>())

      .def_property_readonly("real_time_index",
                             GetIfNotNull(&mujoco::Simulate::real_time_index),
                             py::call_guard<py::gil_scoped_release>())
      .def_property("speed_changed",
                    GetIfNotNull(&mujoco::Simulate::speed_changed),
                    SetIfNotNull(&mujoco::Simulate::speed_changed),
                    py::call_guard<py::gil_scoped_release>())
      .def_property("measured_slowdown",
                    GetIfNotNull(&mujoco::Simulate::measured_slowdown),
                    SetIfNotNull(&mujoco::Simulate::measured_slowdown),
                    py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("refresh_rate",
                             GetIfNotNull(&mujoco::Simulate::refresh_rate),
                             py::call_guard<py::gil_scoped_release>())

      .def_property_readonly("busywait",
                             GetIfNotNull(&mujoco::Simulate::busywait),
                             py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("run", GetIfNotNull(&mujoco::Simulate::run),
                             py::call_guard<py::gil_scoped_release>())

      .def_property_readonly("exitrequest",
                             CallIfNotNull(+[](mujoco::Simulate& sim) {
                               return sim.exitrequest.load();
                             }),
                             py::call_guard<py::gil_scoped_release>())
      .def("exit",
           [](SimulateWrapper& wrapper) {
             mujoco::Simulate* sim = wrapper.simulate();
             if (!sim) {
               return;
             }

             int value = 0;
             sim->exitrequest.compare_exchange_strong(value, 1);
             wrapper.WaitUntilExit();
           })

      .def_property_readonly("uiloadrequest",
                             CallIfNotNull(+[](mujoco::Simulate& sim) {
                               return sim.uiloadrequest.load();
                             }),
                             py::call_guard<py::gil_scoped_release>())
      .def("uiloadrequest_decrement", CallIfNotNull(+[](mujoco::Simulate& sim) {
             sim.uiloadrequest.fetch_sub(1);
           }),
           py::call_guard<py::gil_scoped_release>())
      .def("update_hfield",
           CallIfNotNull(+[](mujoco::Simulate& sim, int hfieldid) {
             sim.UpdateHField(hfieldid);
           }),
           py::call_guard<py::gil_scoped_release>())
      .def("update_mesh", CallIfNotNull(+[](mujoco::Simulate& sim, int meshid) {
             sim.UpdateMesh(meshid);
           }),
           py::call_guard<py::gil_scoped_release>())
      .def("update_texture",
           CallIfNotNull(+[](mujoco::Simulate& sim, int texid) {
             sim.UpdateTexture(texid);
           }),
           py::call_guard<py::gil_scoped_release>())

      .def_property(
          "droploadrequest", CallIfNotNull(+[](mujoco::Simulate& sim) {
            return sim.droploadrequest.load();
          }),
          CallIfNotNull(+[](mujoco::Simulate& sim, bool droploadrequest) {
            sim.droploadrequest.store(droploadrequest);
          }),
          py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("dropfilename",
                             GetIfNotNull(&mujoco::Simulate::dropfilename),
                             py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("filename",
                             GetIfNotNull(&mujoco::Simulate::filename),
                             py::call_guard<py::gil_scoped_release>())
      .def_property(
          "load_error", GetIfNotNull(&mujoco::Simulate::load_error),
          CallIfNotNull(+[](mujoco::Simulate& sim, const std::string& error) {
            const auto max_length = sizeof_arr(sim.load_error);
            std::strncpy(sim.load_error, error.c_str(), max_length - 1);
            sim.load_error[max_length - 1] = '\0';
          }))
      .def_property("ui0_enable", GetIfNotNull(&mujoco::Simulate::ui0_enable),
                    CallIfNotNull(+[](mujoco::Simulate& sim, int enabled) {
                      sim.ui0_enable = enabled;
                    }),
                    py::call_guard<py::gil_scoped_release>())
      .def_property("ui1_enable", GetIfNotNull(&mujoco::Simulate::ui1_enable),
                    CallIfNotNull(+[](mujoco::Simulate& sim, int enabled) {
                      sim.ui1_enable = enabled;
                    }),
                    py::call_guard<py::gil_scoped_release>());

  pymodule.def("set_glfw_dlhandle", [](std::uintptr_t dlhandle) {
    mujoco::Glfw(reinterpret_cast<void*>(dlhandle));
  });
}

}  // namespace
}  // namespace mujoco::python
