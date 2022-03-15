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

#include <array>
#include <cstdio>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include "functions.h"
#include "raw.h"
#include <pybind11/buffer_info.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace mujoco::python {

namespace {

namespace py = ::pybind11;

const auto rollout_doc = R"(
Roll out open-loop trajectories from initial states, get subsequent states and sensor values.

  input arguments (required):
    model              an instance of MjModel
    data               an associated instance of MjData
    nstate             an integer, number of initial states from which to roll out trajectories
    nstep              an integer, number of steps to be taken for each trajectory
  input arguments (optional):
    initial_state      (nstate x nqva)                 nstate initial state vectors, nqva=nq+nv+na
    initial_time       (nstate x 1)                    nstate initial times
    initial_warmstart  (nstate x nv)                   nstate qacc_warmstart vectors
    ctrl               (nstate x nstep x nu)           nstate length-nstep controls
    qfrc_applied       (nstate x nstep x nv)           nstate length-nstep generalized forces
    xfrc_applied       (nstate x nstep x nbody*6)      nstate length-nstep Cartesian wrenches
    mocap              (nstate x nstep x nmocap*7)     nstate length-nstep mocap body poses
  output arguments (optional):
    state              (nstate x nstep x nqva)         nstate length-nstep states
    sensordata         (nstate x nstep x nsendordata)  nstate length-nstep sensordatas
)";

// C-style rollout function, assumes all arguments are valid
// all input fields of d are initialised, contents at call time do not matter
// after returning, d will contain the last step of the last rollout
void _unsafe_rollout(const mjModel* m, mjData* d, int nstate, int nstep,
                     const mjtNum* state0, const mjtNum* ctrl,
                     const mjtNum* qfrc, const mjtNum* xfrc,
                     const mjtNum* mocap, const mjtNum* time0,
                     const mjtNum* warmstart0,
                     mjtNum* state, mjtNum* sensordata) {
  // model sizes
  int nq = m->nq;
  int nv = m->nv;
  int na = m->na;
  int nqva = nq + nv + na;
  int nu = m->nu;
  int nbody = m->nbody;
  int nmocap = m->nmocap;
  int nsensordata = m->nsensordata;

  // loop over initial states
  for (int s=0; s < nstate; s++) {

    // set initial state
    if (state0) {
      mju_copy(d->qpos, state0 + s*nqva, nq);
      mju_copy(d->qvel, state0 + s*nqva + nq, nv);
      mju_copy(d->act,  state0 + s*nqva + nq + nv, na);
    } else {
      mju_copy(d->qpos, m->qpos0, nq);
      mju_zero(d->qvel, nv);
      mju_zero(d->act, na);
    }

    // set initial time
    d->time = time0 ? time0[s] : 0;

    // set warmstart accelerations
    if (warmstart0) {
      mju_copy(d->qacc_warmstart, warmstart0 + s*nv, nv);
    } else {
      mju_zero(d->qacc_warmstart, nv);
    }

    // clear control inputs if unspecified
    if (s == 0) {
      if (!ctrl) {
        mju_zero(d->ctrl, nu);
      }
      if (!qfrc) {
        mju_zero(d->qfrc_applied, nv);
      }
      if (!xfrc) {
        mju_zero(d->xfrc_applied, 6*nbody);
      }
      if (!mocap) {
        for (int j=0; j<nbody; j++) {
          int id = m->body_mocapid[j];
          if (id>=0) {
            mju_copy3(d->mocap_pos+3*id, m->body_pos+3*j);
            mju_copy4(d->mocap_quat+4*id, m->body_quat+4*j);
          }
        }
      }
    }

    // roll out trajectories
    for (int t = 0; t < nstep; t++) {
      // controls
      if (ctrl) {
        mju_copy(d->ctrl, ctrl + s*nstep*nu + t*nu, nu);
      }
      // generalized forces
      if (qfrc) {
        mju_copy(d->qfrc_applied, qfrc + s*nstep*nv + t*nv, nv);
      }
      // Cartesian wrenches
      if (xfrc) {
        mju_copy(d->xfrc_applied, xfrc + s*nstep*6*nbody + t*6*nbody, 6*nbody);
      }
      // mocap bodies
      if (mocap) {
        mju_copy(d->mocap_pos,
                 mocap + s*nstep*7*nmocap + t*7*nmocap, 3*nmocap);
        mju_copy(d->mocap_quat,
                 mocap + s*nstep*7*nmocap + t*7*nmocap + 3*nmocap, 4*nmocap);
      }

      // step
      mj_step(m, d);

      // copy out new state
      if (state) {
        mju_copy(state + s*nstep*nqva + t*nqva,           d->qpos, nq);
        mju_copy(state + s*nstep*nqva + t*nqva + nq,      d->qvel, nv);
        mju_copy(state + s*nstep*nqva + t*nqva + nq + nv, d->act,  na);
      }
      // copy out sensor values
      if (sensordata) {
        mju_copy(sensordata + s*nstep*nsensordata + t*nsensordata,
                 d->sensordata, nsensordata);
      }
    }
  }
}


// check size of optional argument to rollout(), return raw pointer
mjtNum* get_array_ptr(std::optional<const py::array_t<mjtNum>> arg,
                      const char* name, int nstate, int nstep, int dim) {
  // if empty return nullptr
  if (!arg.has_value()) {
    return nullptr;
  }

  // get info
  py::buffer_info info = arg->request();

  // check size
  int expected_size = nstate * nstep * dim;
  if (info.size != expected_size) {
    std::ostringstream msg;
    msg << name << ".size should be " << expected_size <<  ", got " << info.size;
    throw py::value_error(msg.str());
  }
  return static_cast<mjtNum*>(info.ptr);
}


PYBIND11_MODULE(_rollout, pymodule) {
  namespace py = ::pybind11;
  using PyCArray = py::array_t<mjtNum, py::array::c_style>;

  // roll out open loop trajectories from multiple initial states
  // get subsequent states and corresponding sensor values
  pymodule.def(
      "rollout",
      [](const MjModelWrapper& m, MjDataWrapper& d, int nstate, int nstep,
         std::optional<const PyCArray> init_state,
         std::optional<const PyCArray> init_time,
         std::optional<const PyCArray> init_warmstart,
         std::optional<const PyCArray> ctrl,
         std::optional<const PyCArray> qfrc,
         std::optional<const PyCArray> xfrc,
         std::optional<const PyCArray> mocap,
         std::optional<const PyCArray> state,
         std::optional<const PyCArray> sensordata
         ) {

        const raw::MjModel* model = m.get();
        raw::MjData* data = d.get();

        // check that some steps need to be taken, return if not
        if (nstate < 1 || nstep < 1) {
          return;
        }

        // get raw pointers
        int nqva = model->nq + model->nv + model->na;
        mjtNum* init_state_ptr =
            get_array_ptr(init_state, "initial_state", nstate, 1, nqva);
        mjtNum* ctrl_ptr = get_array_ptr(ctrl, "ctrl", nstate, nstep, model->nu);
        mjtNum* qfrc_ptr =
            get_array_ptr(qfrc, "qfrc_applied", nstate, nstep, model->nv);
        mjtNum* xfrc_ptr =
            get_array_ptr(xfrc, "xfrc_applied", nstate, nstep, 6*model->nbody);
        mjtNum* mocap_ptr =
            get_array_ptr(mocap, "mocap", nstate, nstep, 7*model->nmocap);
        mjtNum* init_time_ptr =
            get_array_ptr(init_time, "init_time", nstate, 1, 1);
        mjtNum* init_warmstart_ptr =
            get_array_ptr(init_warmstart, "init_warmstart", nstate, 1, model->nv);
        mjtNum* state_ptr = get_array_ptr(state, "state", nstate, nstep, nqva);
        mjtNum* sensordata_ptr =
            get_array_ptr(sensordata, "sensordata", nstate, nstep, model->nsensordata);

        // perform rollouts
        {
          // release the GIL
          py::gil_scoped_release no_gil;

          // call unsafe rollout function
          InterceptMjErrors(_unsafe_rollout)(
              model, data, nstate, nstep, init_state_ptr, ctrl_ptr, qfrc_ptr,
              xfrc_ptr, mocap_ptr, init_time_ptr, init_warmstart_ptr, state_ptr,
              sensordata_ptr);
        }
      },
      py::arg("model"),
      py::arg("data"),
      py::arg("nstate"),
      py::arg("nstep"),
      py::arg("initial_state")     = py::none(),
      py::arg("initial_time")      = py::none(),
      py::arg("initial_warmstart") = py::none(),
      py::arg("ctrl")              = py::none(),
      py::arg("qfrc_applied")      = py::none(),
      py::arg("xfrc_applied")      = py::none(),
      py::arg("mocap")             = py::none(),
      py::arg("state")             = py::none(),
      py::arg("sensordata")        = py::none(),
      py::doc(rollout_doc)
  );

}  // namespace

}

}
