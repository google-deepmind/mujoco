// Copyright 2026 DeepMind Technologies Limited
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

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <mujoco/mjbatch.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "errors.h"
#include "raw.h"
#include "structs.h"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace mujoco::python {

namespace {

namespace py = ::pybind11;

// int32, C-contiguous, as given: no cast that could truncate or wrap an id
using Ids = py::array_t<int, py::array::c_style>;
using NumArray = py::array_t<mjtNum, py::array::c_style>;  // C-contiguous mjtNum
using IntArray = py::array_t<int, py::array::c_style>;     // C-contiguous int

// physics callbacks, which a batch calls on its own models and data
constexpr const char* kCallbacks[] = {"mjcb_passive",  "mjcb_control", "mjcb_contactfilter",
                                      "mjcb_sensor",   "mjcb_time",    "mjcb_act_dyn",
                                      "mjcb_act_gain", "mjcb_act_bias"};

// the installed physics callbacks, in the order of kCallbacks
std::array<std::uintptr_t, 8> InstalledCallbacks() {
  return {reinterpret_cast<std::uintptr_t>(mjcb_passive),
          reinterpret_cast<std::uintptr_t>(mjcb_control),
          reinterpret_cast<std::uintptr_t>(mjcb_contactfilter),
          reinterpret_cast<std::uintptr_t>(mjcb_sensor),
          reinterpret_cast<std::uintptr_t>(mjcb_time),
          reinterpret_cast<std::uintptr_t>(mjcb_act_dyn),
          reinterpret_cast<std::uintptr_t>(mjcb_act_gain),
          reinterpret_cast<std::uintptr_t>(mjcb_act_bias)};
}

// bytes per simulation and element type of mjData array field name, as the batch records it
// (fixed-size numeric arrays included); nullopt if unknown
std::optional<std::pair<size_t, py::dtype>> DataField(const mjModel* m, const std::string& name) {
#define X(type, field, nr, nc) \
  if (name == #field) return std::make_pair(sizeof(type) * m->nr * (nc), py::dtype::of<type>());
  MJDATA_POINTERS
#undef X
#define X(type, field, nr, nc)                                                       \
  if constexpr (std::is_arithmetic_v<type>) {                                        \
    if (name == #field) {                                                            \
      return std::make_pair(sizeof(type) * (nr) * (nc), py::dtype::of<type>());      \
    }                                                                                \
  }
  MJDATA_VECTOR
#undef X
  return std::nullopt;
}

// position of each simulation in a call: (nsim) for a selection, empty for all simulations;
// raise ValueError for an id out of range
std::vector<int> Positions(const std::optional<Ids>& ids, int nsim) {
  std::vector<int> pos;
  if (ids) {
    pos.assign(nsim, -1);
    for (py::ssize_t j = 0; j < ids->size(); ++j) {
      int id = ids->data()[j];
      if (id < 0 || id >= nsim) throw py::value_error("simulation id out of range");
      pos[id] = static_cast<int>(j);
    }
  }
  return pos;
}

// arguments of Jac, arrays indexed by position in the call
struct JacArgs {
  const int* pos;       // position of each simulation, NULL: its index
  const mjtNum* point;  // (n, 3)
  mjtNum* jacp;         // (n, 3, nv) or NULL
  mjtNum* jacr;         // (n, 3, nv) or NULL
  const int* body;      // (n) body ids
};

// mjb_apply function: Jacobians of a point on a body
void Jac(const mjModel* m, mjData* d, int sim, void* arg) {
  auto* a = static_cast<JacArgs*>(arg);
  int j = a->pos ? a->pos[sim] : sim;
  mj_kinematics(m, d);
  mj_comPos(m, d);
  size_t jac = static_cast<size_t>(j) * 3 * m->nv;
  mj_jac(m, d, a->jacp ? a->jacp + jac : nullptr, a->jacr ? a->jacr + jac : nullptr,
         a->point + 3 * j, a->body[j]);
}

// arguments of Ray, arrays indexed by position in the call
struct RayArgs {
  const int* pos;            // position of each simulation, NULL: its index
  const mjtNum* pnt;         // (n, nray, 3)
  const mjtNum* vec;         // (n, nray, 3)
  int nray;                  // rays per simulation
  const mjtByte* geomgroup;  // mjNGROUP or NULL
  mjtByte flg_static;        // include static geoms
  const int* bodyexclude;    // (n, nray) body to exclude, or -1
  mjtNum* dist;              // (n, nray)
  int* geomid;               // (n, nray)
  mjtNum* normal;            // (n, nray, 3) or NULL
};

// mjb_apply function: intersect rays with the geoms
void Ray(const mjModel* m, mjData* d, int sim, void* arg) {
  auto* a = static_cast<RayArgs*>(arg);
  int j = a->pos ? a->pos[sim] : sim;
  mj_kinematics(m, d);
  if (m->nflex) mj_flex(m, d);
  for (int k = 0; k < a->nray; ++k) {
    size_t r = static_cast<size_t>(j) * a->nray + k;
    a->dist[r] = mj_ray(m, d, a->pnt + 3 * r, a->vec + 3 * r, a->geomgroup, a->flg_static,
                        a->bodyexclude[r], a->geomid + r, a->normal ? a->normal + 3 * r : nullptr);
  }
}

// owner of an mjBatch, kept alive by the arrays it hands out through their base object
class Batch {
 public:
  // allocate the batch, raise ValueError on failure; mj_makeData runs mj_forward, and so the
  // callbacks, when trees start asleep
  Batch(const MjModelWrapper& model, int nsim, int nthread, bool persistent) {
    if (StartsAsleep(model.get())) CheckCallbacks();
    mjBatchConfig config;
    mjb_defaultConfig(&config);
    config.nthread = nthread;
    config.persistent = persistent;
    char error[512];
    {
      py::gil_scoped_release release;  // a ctypes callback on a worker takes the GIL
      b_ = InterceptMjErrors(mjb_makeBatch)(model.get(), nsim, &config, error, sizeof(error));
    }
    if (!b_) throw py::value_error(error);
  }
  // free the batch
  ~Batch() { mjb_deleteBatch(b_); }
  Batch(const Batch&) = delete;
  Batch& operator=(const Batch&) = delete;

  // the batch
  mjBatch* get() { return b_; }

  // (nsim, nstate) view of the state rows
  static py::array State(py::object self) {
    mjBatch* b = self.cast<Batch&>().b_;
    return py::array_t<mjtNum>({mjb_nsim(b), mjb_nstate(b)}, mjb_state(b), self);
  }

  // (nsim, mjNWARNING, 2) view of the warning counters
  static py::array Warning(py::object self) {
    mjBatch* b = self.cast<Batch&>().b_;
    return py::array_t<int>({mjb_nsim(b), static_cast<int>(mjNWARNING), 2},
                            reinterpret_cast<int*>(mjb_warning(b)), self);
  }

  // (nsim,) view of the per-simulation status
  static py::array Status(py::object self) {
    mjBatch* b = self.cast<Batch&>().b_;
    return py::array_t<int>(std::vector<py::ssize_t>{mjb_nsim(b)}, mjb_status(b), self);
  }

  // (nsim, bytes) uint8 view of a per-simulation buffer; the caller reinterprets it
  static py::object Bytes(py::object self, void* buf, int size, int elemsize) {
    if (!buf) return py::none();
    mjBatch* b = self.cast<Batch&>().b_;
    return py::array_t<uint8_t>({mjb_nsim(b), size * elemsize},
                                static_cast<uint8_t*>(buf), self);
  }

  // run a call; return the number of simulations that raised
  template <typename Func, typename... Args>
  int Call(Func func, const std::optional<Ids>& ids, Args... args) {
    CheckCallbacks();
    const int* p = ids ? ids->data() : nullptr;
    int n = ids ? static_cast<int>(ids->size()) : 0;
    py::gil_scoped_release release;
    return InterceptMjErrors(*func)(b_, p, n, args...);
  }

 private:
  // whether model m has trees that start asleep
  static bool StartsAsleep(const mjModel* m) {
    if (!(m->opt.enableflags & mjENBL_SLEEP)) return false;
    for (int i = 0; i < m->ntree; ++i) {
      if (m->tree_sleep_policy[i] == mjSLEEP_INIT) return true;
    }
    return false;
  }

  // raise if a Python function is installed as a physics callback: it would need MjModel
  // and MjData wrappers, which the batch's models and data do not have; C functions
  // installed through ctypes are fine. Python is consulted only when the callbacks change.
  void CheckCallbacks() {
    std::array<std::uintptr_t, 8> installed = InstalledCallbacks();
    {
      std::lock_guard<std::mutex> lock(checked_mu_);
      if (installed == checked_) return;
    }
    py::module_ callbacks = py::module_::import("mujoco._callbacks");
    py::object cfuncptr = py::module_::import("ctypes").attr("_CFuncPtr");
    for (const char* name : kCallbacks) {
      py::object cb = callbacks.attr(("get_" + std::string(name)).c_str())();
      if (!cb.is_none() && !py::isinstance(cb, cfuncptr)) {
        throw py::value_error(std::string(name) +
                              " is a Python function; a batch accepts only C callbacks");
      }
    }
    std::lock_guard<std::mutex> lock(checked_mu_);
    checked_ = installed;
  }

  mjBatch* b_ = nullptr;                    // the batch
  std::array<std::uintptr_t, 8> checked_{};  // callbacks last found to be C functions
  std::mutex checked_mu_;                   // guards checked_
};

// raise ValueError if array a does not have the given shape
void CheckShape(const py::array& a, std::initializer_list<py::ssize_t> shape,
                const char* name) {
  if (a.ndim() != static_cast<py::ssize_t>(shape.size()) ||
      !std::equal(shape.begin(), shape.end(), a.shape())) {
    throw py::value_error(std::string(name) + " has the wrong shape");
  }
}

}  // namespace

PYBIND11_MODULE(_batch, pymodule, pybind11::mod_gil_not_used()) {
  py::class_<Batch>(pymodule, "Batch")
      .def(py::init([](const MjModelWrapper& model, int nsim, int nthread, bool persistent) {
             return std::make_unique<Batch>(model, nsim, nthread, persistent);
           }),
           py::arg("model"), py::arg("nsim"), py::arg("nthread") = 0,
           py::arg("persistent") = false)
      .def_property_readonly("nsim", [](Batch& b) { return mjb_nsim(b.get()); })
      .def_property_readonly("nthread", [](Batch& b) { return mjb_nthread(b.get()); })
      .def_property_readonly("nstate", [](Batch& b) { return mjb_nstate(b.get()); })
      .def_property_readonly("persistent",
                             [](Batch& b) { return mjb_persistent(b.get()) != 0; })
      .def("state", &Batch::State)
      .def("state_component",
           [](Batch& b, int component) {
             int size = 0;
             mjtNum* adr = InterceptMjErrors(mjb_stateComponent)(b.get(), component, &size);
             return py::make_tuple(adr - mjb_state(b.get()), size);
           })
      .def("warning", &Batch::Warning)
      .def("status", &Batch::Status)
      .def("error",
           [](Batch& b, int sim) {
             if (sim < 0 || sim >= mjb_nsim(b.get())) throw py::index_error("sim");
             return std::string(mjb_error(b.get(), sim));
           })
      .def("output",
           [](py::object self, const std::string& name) {
             int size = 0, elemsize = 0;
             mjBatch* b = self.cast<Batch&>().get();
             void* buf;
             {
               py::gil_scoped_release release;  // waits for a running call
               buf = InterceptMjErrors(mjb_output)(b, name.c_str(), &size, &elemsize);
             }
             return Batch::Bytes(self, buf, size, elemsize);
           })
      .def("expand",
           [](py::object self, const std::string& name) {
             int size = 0, elemsize = 0;
             mjBatch* b = self.cast<Batch&>().get();
             void* buf;
             {
               py::gil_scoped_release release;  // waits for a running call
               buf = InterceptMjErrors(mjb_expand)(b, name.c_str(), &size, &elemsize);
             }
             return Batch::Bytes(self, buf, size, elemsize);
           })
      .def("step",
           [](Batch& b, std::optional<Ids> ids, int nstep, int stage) {
             return b.Call(mjb_step, ids, nstep, stage);
           },
           py::arg("ids") = py::none(), py::arg("nstep") = 1, py::arg("stage") = 0)
      .def("forward",
           [](Batch& b, std::optional<Ids> ids, int stage) {
             return b.Call(mjb_forward, ids, stage);
           },
           py::arg("ids") = py::none(), py::arg("stage") = static_cast<int>(mjSTAGE_ACC))
      .def("reset",
           // key: one keyframe per simulation of the call (-1: defaults), or None for defaults
           [](Batch& b, std::optional<Ids> ids, std::optional<IntArray> key, int stage) {
             py::ssize_t n = ids ? ids->size() : mjb_nsim(b.get());
             if (key) CheckShape(*key, {n}, "keyframe");
             return b.Call(mjb_reset, ids, key ? key->data() : nullptr, stage);
           },
           py::arg("ids") = py::none(), py::arg("keyframe") = py::none(),
           py::arg("stage") = static_cast<int>(mjSTAGE_ACC))
      .def("get_model",
           [](Batch& b, int sim, MjModelWrapper& model) {
             if (sim < 0 || sim >= mjb_nsim(b.get())) throw py::index_error("sim");
             py::gil_scoped_release release;  // waits for a running call
             InterceptMjErrors(mjb_getModel)(b.get(), sim, model.get());
           })
      .def("get_data",
           [](Batch& b, int sim, MjDataWrapper& data) {
             if (sim < 0 || sim >= mjb_nsim(b.get())) throw py::index_error("sim");
             py::gil_scoped_release release;  // waits for a running call
             InterceptMjErrors(mjb_getData)(b.get(), sim, data.get());
           })
      .def("set_const",
           [](Batch& b, std::optional<Ids> ids) { return b.Call(mjb_setConst, ids); },
           py::arg("ids") = py::none())
      .def(
          "rollout",
          // records: (field or None, spec, buffer), buffer (n, nstep, size) of the field's dtype
          [](Batch& b, std::optional<Ids> ids, int nstep, std::optional<NumArray> control,
             int control_spec, const std::vector<py::tuple>& records) {
            const mjModel* m = mjb_model(b.get());
            py::ssize_t n = ids ? ids->size() : mjb_nsim(b.get());
            if (nstep < 1) throw py::value_error("nstep must be positive");
            if (control) {
              if (control_spec & ~mjSTATE_USER) {
                throw py::value_error("control_spec must be a subset of mjSTATE_USER");
              }
              CheckShape(*control, {n, nstep, mj_stateSize(m, control_spec)}, "control");
            }
            std::vector<std::string> names;
            std::vector<mjBatchRecord> recs;
            names.reserve(records.size());
            for (const py::tuple& r : records) {
              if (r.size() != 3) throw py::value_error("a record is (field, spec, buffer)");
              py::array buf = r[2].cast<py::array>();
              if (!(buf.flags() & py::array::c_style) || !buf.writeable()) {
                throw py::value_error("record buffers must be writable and C-contiguous");
              }
              const char* field = nullptr;
              int spec = r[1].cast<int>();
              size_t bytes;
              py::dtype dtype = py::dtype::of<mjtNum>();
              if (!r[0].is_none()) {
                names.push_back(r[0].cast<std::string>());
                field = names.back().c_str();
                auto info = DataField(m, names.back());
                if (!info) throw py::value_error("unknown field " + names.back());
                std::tie(bytes, dtype) = *info;
              } else {
                if (spec < 0 || spec >= (1 << mjNSTATE)) {
                  throw py::value_error("invalid state spec " + std::to_string(spec));
                }
                bytes = sizeof(mjtNum) * mj_stateSize(m, spec);
              }
              if (!buf.dtype().equal(dtype) ||
                  static_cast<size_t>(buf.nbytes()) != static_cast<size_t>(n) * nstep * bytes) {
                throw py::value_error("a record buffer must be (n, nstep, size) of its type");
              }
              recs.push_back({field, spec, buf.mutable_data()});
            }
            return b.Call(mjb_rollout, ids, nstep, control ? control->data() : nullptr,
                          control_spec, static_cast<const mjBatchRecord*>(recs.data()),
                          static_cast<int>(recs.size()));
          },
          py::arg("ids"), py::arg("nstep"), py::arg("control"), py::arg("control_spec"),
          py::arg("records"))
      .def(
          "jac",
          // for the n simulations of the call: body (n); point (n, 3); jacp, jacr (n, 3, nv) or
          // None
          [](Batch& b, std::optional<Ids> ids, IntArray body, NumArray point,
             std::optional<NumArray> jacp, std::optional<NumArray> jacr) {
            const mjModel* m = mjb_model(b.get());
            int nsim = mjb_nsim(b.get());
            py::ssize_t n = ids ? ids->size() : nsim;
            CheckShape(body, {n}, "body");
            for (py::ssize_t j = 0; j < n; ++j) {
              if (body.data()[j] < 0 || body.data()[j] >= m->nbody) {
                throw py::value_error("body out of range");
              }
            }
            CheckShape(point, {n, 3}, "point");
            if (jacp) CheckShape(*jacp, {n, 3, m->nv}, "jacp");
            if (jacr) CheckShape(*jacr, {n, 3, m->nv}, "jacr");
            std::vector<int> pos = Positions(ids, nsim);
            JacArgs args{pos.empty() ? nullptr : pos.data(), point.data(),
                         jacp ? jacp->mutable_data() : nullptr,
                         jacr ? jacr->mutable_data() : nullptr, body.data()};
            return b.Call(mjb_apply, ids, &Jac, static_cast<void*>(&args), 0);
          },
          py::arg("ids"), py::arg("body"), py::arg("point"), py::arg("jacp"), py::arg("jacr"))
      .def(
          "ray",
          // for the n simulations of the call: pnt, vec (n, nray, 3); bodyexclude, dist, geomid
          // (n, nray); normal (n, nray, 3) or None
          [](Batch& b, std::optional<Ids> ids, NumArray pnt, NumArray vec,
             std::optional<py::array_t<mjtByte, py::array::c_style>> geomgroup, bool flg_static,
             IntArray bodyexclude, NumArray dist, IntArray geomid,
             std::optional<NumArray> normal) {
            int nsim = mjb_nsim(b.get());
            py::ssize_t n = ids ? ids->size() : nsim;
            if (pnt.ndim() != 3) throw py::value_error("pnt has the wrong shape");
            py::ssize_t nray = pnt.shape(1);
            CheckShape(pnt, {n, nray, 3}, "pnt");
            CheckShape(vec, {n, nray, 3}, "vec");
            CheckShape(dist, {n, nray}, "dist");
            CheckShape(geomid, {n, nray}, "geomid");
            CheckShape(bodyexclude, {n, nray}, "bodyexclude");
            if (normal) CheckShape(*normal, {n, nray, 3}, "normal");
            if (geomgroup) CheckShape(*geomgroup, {mjNGROUP}, "geomgroup");
            std::vector<int> pos = Positions(ids, nsim);
            RayArgs args{pos.empty() ? nullptr : pos.data(), pnt.data(), vec.data(),
                         static_cast<int>(nray),
                         geomgroup ? geomgroup->data() : nullptr,
                         static_cast<mjtByte>(flg_static), bodyexclude.data(),
                         dist.mutable_data(), geomid.mutable_data(),
                         normal ? normal->mutable_data() : nullptr};
            return b.Call(mjb_apply, ids, &Ray, static_cast<void*>(&args), 0);
          },
          py::arg("ids"), py::arg("pnt"), py::arg("vec"), py::arg("geomgroup"),
          py::arg("flg_static"), py::arg("bodyexclude"), py::arg("dist"), py::arg("geomid"),
          py::arg("normal"));
}

}  // namespace mujoco::python
