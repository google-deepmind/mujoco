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
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <optional>

#include <Eigen/Core>
#include "function_traits.h"
#include "functions.h"
#include "raw.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace mujoco::python {
namespace {
PYBIND11_MODULE(_functions, pymodule) {
  namespace py = ::pybind11;
  namespace traits = python_traits;

  using EigenVectorX = Eigen::Vector<mjtNum, Eigen::Dynamic>;
  using EigenArrayXX = Eigen::Array<
      mjtNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;


  // Import the _structs module so that pybind11 knows about Python bindings
  // for MjWrapper types and therefore generates prettier docstrings.
  py::module::import("mujoco._structs");

  // Virtual file system
  // Skipped entire section

  // Parse and compile
  // Skipped: mj_loadXML (have MjModel.from_xml_string)
  DEF_WITH_OMITTED_PY_ARGS(traits::mj_saveLastXML, "error", "error_sz")(
      pymodule,
      [](const char* filename, const mjModel* m) {
        std::array<char, 1024> error;
        int success = InterceptMjErrors(::mj_saveLastXML)(
            filename, m, error.data(), error.size());
        if (!success) {
          throw FatalError(std::string(error.data()));
        }
      });
  // Skipped: mj_freeLastXML
  DEF_WITH_OMITTED_PY_ARGS(traits::mj_printSchema,
                           "filename", "buffer", "buffer_sz")(
      pymodule, [](bool flg_html, bool flg_pad) {
        constexpr int kBufferSize = 40000;
        auto buffer = std::unique_ptr<char[]>(new char[kBufferSize]);
        const int out_length = InterceptMjErrors(::mj_printSchema)(
            nullptr, buffer.get(), kBufferSize, flg_html, flg_pad);
        if (out_length >= kBufferSize) {
          throw UnexpectedError("output buffer too small");
        }
        return std::string(buffer.get(), out_length);
      });

  // Main simulation
  pymodule.def(
      "mj_step",
      InterceptMjErrors(
          [](const MjModelWrapper& m, MjDataWrapper& d, int nstep) {
            const raw::MjModel* const m_ptr = m.get();
            raw::MjData* const d_ptr = d.get();
            for (int i = 0; i < nstep; ++i) {
              ::mj_step(m_ptr, d_ptr);
            }
          }),
      py::arg("m"), py::arg("d"), py::arg_v("nstep", 1),
      py::doc((std::string(traits::mj_step::doc) +
               std::string(" Optionally, repeat nstep times.")).c_str()),
      py::call_guard<py::gil_scoped_release>());
  Def<traits::mj_step1>(pymodule);
  Def<traits::mj_step2>(pymodule);
  Def<traits::mj_forward>(pymodule);
  Def<traits::mj_inverse>(pymodule);
  Def<traits::mj_forwardSkip>(pymodule);
  Def<traits::mj_inverseSkip>(pymodule);

  // Initialization
  Def<traits::mj_defaultLROpt>(pymodule);
  Def<traits::mj_defaultSolRefImp>(pymodule);
  Def<traits::mj_defaultOption>(pymodule);
  Def<traits::mj_defaultVisual>(pymodule);
  // Skipped: mj_copyModel (have MjModel.__copy__, memory managed by MjModel)
  DEF_WITH_OMITTED_PY_ARGS(traits::mj_saveModel, "buffer_sz")(
      pymodule,
      [](const raw::MjModel* m, const std::optional<std::string>& filename,
         std::optional<
             Eigen::Ref<Eigen::Vector<std::uint8_t, Eigen::Dynamic>>> buffer) {
        void* buffer_ptr = nullptr;
        int buffer_sz = 0;
        if (buffer.has_value()) {
          buffer_ptr = buffer->data();
          buffer_sz = buffer->size();
        }
        return InterceptMjErrors(::mj_saveModel)(
            m, filename.has_value() ? filename->c_str() : nullptr,
            buffer_ptr, buffer_sz);
      });
  // Skipped: mj_loadModel (have MjModel.from_binary_path)
  // Skipped: mj_deleteModel (have MjModel.__del__)
  Def<traits::mj_sizeModel>(pymodule);
  // Skipped: mj_makeData (have MjData.__init__)
  // Skipped: mj_copyData (have MjData.__copy__, memory managed by MjData)
  Def<traits::mj_resetData>(pymodule);
  Def<traits::mj_resetDataDebug>(pymodule);
  Def<traits::mj_resetDataKeyframe>(pymodule);
  // Skipped: mj_stackAlloc (doesn't make sense in Python)
  // Skipped: mj_deleteData (have MjData.__del__)
  Def<traits::mj_resetCallbacks>(pymodule);
  Def<traits::mj_setConst>(pymodule);
  DEF_WITH_OMITTED_PY_ARGS(traits::mj_setLengthRange, "error", "error_sz")(
      pymodule,
      [](raw::MjModel* m, raw::MjData* d, int index, const raw::MjLROpt* opt) {
        std::array<char, 1024> error;
        int success = InterceptMjErrors(::mj_setLengthRange)(
            m, d, index, opt, error.data(), error.size());
        if (!success) {
          throw FatalError(std::string(error.data()));
        }
      });

  // Printing
  Def<traits::mj_printFormattedModel>(pymodule);
  Def<traits::mj_printModel>(pymodule);
  Def<traits::mj_printFormattedData>(pymodule);
  Def<traits::mj_printData>(pymodule);
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_printMat, "nr", "nc")(
      pymodule,
      [](Eigen::Ref<const EigenArrayXX> mat) {
        return ::mju_printMat(mat.data(), mat.rows(), mat.cols());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_printMatSparse, "nr")(
      pymodule,
      [](Eigen::Ref<const EigenVectorX> mat,
         Eigen::Ref<const Eigen::Vector<int, Eigen::Dynamic>> rownnz,
         Eigen::Ref<const Eigen::Vector<int, Eigen::Dynamic>> rowadr,
         Eigen::Ref<const Eigen::Vector<int, Eigen::Dynamic>> colind) {
        if (rownnz.size() != rowadr.size()) {
          throw py::type_error("size of rownnz should equal size of rowadr");
        }
        const int nnz = rowadr[rowadr.size() - 1] + rownnz[rownnz.size() - 1];
        if (mat.size() != nnz) {
          throw py::type_error(
              "size of mat should equal rownnz[-1] + rowadr[-1]");
        }
        if (colind.size() != nnz) {
          throw py::type_error(
              "size of colind should equal rownnz[-1] + rowadr[-1]");
        }
        return InterceptMjErrors(::mju_printMatSparse)(
            mat.data(), rowadr.size(), rownnz.data(),
            rowadr.data(), colind.data());
      });

  // Components
  Def<traits::mj_fwdPosition>(pymodule);
  Def<traits::mj_fwdVelocity>(pymodule);
  Def<traits::mj_fwdActuation>(pymodule);
  Def<traits::mj_fwdAcceleration>(pymodule);
  Def<traits::mj_fwdConstraint>(pymodule);
  Def<traits::mj_Euler>(pymodule);
  Def<traits::mj_RungeKutta>(pymodule);
  Def<traits::mj_invPosition>(pymodule);
  Def<traits::mj_invVelocity>(pymodule);
  Def<traits::mj_invConstraint>(pymodule);
  Def<traits::mj_compareFwdInv>(pymodule);

  // Sub components
  Def<traits::mj_sensorPos>(pymodule);
  Def<traits::mj_sensorVel>(pymodule);
  Def<traits::mj_sensorAcc>(pymodule);
  Def<traits::mj_energyPos>(pymodule);
  Def<traits::mj_energyVel>(pymodule);
  Def<traits::mj_checkPos>(pymodule);
  Def<traits::mj_checkVel>(pymodule);
  Def<traits::mj_checkAcc>(pymodule);
  Def<traits::mj_kinematics>(pymodule);
  Def<traits::mj_comPos>(pymodule);
  Def<traits::mj_camlight>(pymodule);
  Def<traits::mj_tendon>(pymodule);
  Def<traits::mj_transmission>(pymodule);
  Def<traits::mj_crb>(pymodule);
  Def<traits::mj_factorM>(pymodule);
  DEF_WITH_OMITTED_PY_ARGS(traits::mj_solveM, "n")(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d, Eigen::Ref<EigenArrayXX> x,
         Eigen::Ref<const EigenArrayXX> y) {
        if (x.rows() != y.rows()) {
          throw py::type_error(
              "the first dimension of x and y should be of the same size");
        }
        if (x.cols() != m->nv) {
          throw py::type_error(
              "the last dimension of x should be of size nv");
        }
        if (y.cols() != m->nv) {
          throw py::type_error(
              "the last dimension of y should be of size nv");
        }
        return InterceptMjErrors(::mj_solveM)(
            m, d, x.data(), y.data(), y.rows());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mj_solveM2, "n")(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d, Eigen::Ref<EigenArrayXX> x,
         Eigen::Ref<const EigenArrayXX> y) {
        if (x.rows() != y.rows()) {
          throw py::type_error(
              "the first dimension of x and y should be of the same size");
        }
        if (x.cols() != m->nv) {
          throw py::type_error(
              "the last dimension of x should be of size nv");
        }
        if (y.cols() != m->nv) {
          throw py::type_error(
              "the last dimension of y should be of size nv");
        }
        return InterceptMjErrors(::mj_solveM2)(
            m, d, x.data(), y.data(), y.rows());
      });
  Def<traits::mj_comVel>(pymodule);
  Def<traits::mj_passive>(pymodule);
  Def<traits::mj_subtreeVel>(pymodule);
  Def<traits::mj_rne>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d, int flg_acc,
         Eigen::Ref<EigenVectorX> result) {
        if (result.size() != m->nv) {
          throw py::type_error("result should have length nv");
        }
        return InterceptMjErrors(::mj_rne)(
            m, d, flg_acc, result.data());
      });
  Def<traits::mj_rnePostConstraint>(pymodule);
  Def<traits::mj_collision>(pymodule);
  Def<traits::mj_makeConstraint>(pymodule);
  Def<traits::mj_projectConstraint>(pymodule);
  Def<traits::mj_referenceConstraint>(pymodule);
  Def<traits::mj_constraintUpdate>(
      pymodule, [](const raw::MjModel* m, raw::MjData* d,
                   Eigen::Ref<const EigenVectorX> jar,
                   std::optional<Eigen::Ref<Eigen::Vector<mjtNum, 1>>> cost,
                   int flg_coneHessian) {
        if (jar.size() != d->nefc) {
          throw py::type_error("size of jar should equal nefc");
        }
        return InterceptMjErrors(::mj_constraintUpdate)(
            m, d, jar.data(), cost.has_value() ? cost->data() : nullptr,
            flg_coneHessian);
      });

  // Support
  Def<traits::mj_stateSize>(pymodule);
  Def<traits::mj_getState>(
      pymodule,
      [](const raw::MjModel* m, const raw::MjData* d,
         Eigen::Ref<EigenVectorX> state, unsigned int spec) {
        if (state.size() != mj_stateSize(m, spec)) {
          throw py::type_error("state size should equal mj_stateSize(m, spec)");
        }
        return InterceptMjErrors(::mj_getState)(m, d, state.data(), spec);
      });
  Def<traits::mj_setState>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         const Eigen::Ref<EigenVectorX> state, unsigned int spec) {
        if (state.size() != mj_stateSize(m, spec)) {
          throw py::type_error("state size should equal mj_stateSize(m, spec)");
        }
        return InterceptMjErrors(::mj_setState)(m, d, state.data(), spec);
      });
  Def<traits::mj_addContact>(pymodule);
  Def<traits::mj_isPyramidal>(pymodule);
  Def<traits::mj_isSparse>(pymodule);
  Def<traits::mj_isDual>(pymodule);
  Def<traits::mj_mulJacVec>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d, Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != d->nefc) {
          throw py::type_error("res should be of length nefc");
        }
        if (vec.size() != m->nv) {
          throw py::type_error("vec should be of length nv");
        }
        return InterceptMjErrors(::mj_mulJacVec)(m, d, res.data(), vec.data());
      });
  Def<traits::mj_mulJacTVec>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d, Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != m->nv) {
          throw py::type_error("res should be of length nv");
        }
        if (vec.size() != d->nefc) {
          throw py::type_error("vec should be of length nefc");
        }
        return InterceptMjErrors(::mj_mulJacTVec)(m, d, res.data(), vec.data());
      });
  Def<traits::mj_jac>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         std::optional<Eigen::Ref<EigenArrayXX>> jacp,
         std::optional<Eigen::Ref<EigenArrayXX>> jacr,
         const mjtNum (*point)[3], int body) {
        if (jacp.has_value() &&
            (jacp->rows() != 3 || jacp->cols() != m->nv)) {
          throw py::type_error("jacp should be of shape (3, nv)");
        }
        if (jacr.has_value() &&
            (jacr->rows() != 3 || jacr->cols() != m->nv)) {
          throw py::type_error("jacr should be of shape (3, nv)");
        }
        return InterceptMjErrors(::mj_jac)(
            m, d,
            jacp.has_value() ? jacp->data() : nullptr,
            jacr.has_value() ? jacr->data() : nullptr,
            &(*point)[0], body);
      });
  Def<traits::mj_jacBody>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         std::optional<Eigen::Ref<EigenArrayXX>> jacp,
         std::optional<Eigen::Ref<EigenArrayXX>> jacr, int body) {
        if (jacp.has_value() &&
            (jacp->rows() != 3 || jacp->cols() != m->nv)) {
          throw py::type_error("jacp should be of shape (3, nv)");
        }
        if (jacr.has_value() &&
            (jacr->rows() != 3 || jacr->cols() != m->nv)) {
          throw py::type_error("jacr should be of shape (3, nv)");
        }
        return InterceptMjErrors(::mj_jacBody)(
            m, d, jacp.has_value() ? jacp->data() : nullptr,
            jacr.has_value() ? jacr->data() : nullptr, body);
      });
  Def<traits::mj_jacBodyCom>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         std::optional<Eigen::Ref<EigenArrayXX>> jacp,
         std::optional<Eigen::Ref<EigenArrayXX>> jacr, int body) {
        if (jacp.has_value() &&
            (jacp->rows() != 3 || jacp->cols() != m->nv)) {
          throw py::type_error("jacp should be of shape (3, nv)");
        }
        if (jacr.has_value() &&
            (jacr->rows() != 3 || jacr->cols() != m->nv)) {
          throw py::type_error("jacr should be of shape (3, nv)");
        }
        return InterceptMjErrors(::mj_jacBodyCom)(
            m, d, jacp.has_value() ? jacp->data() : nullptr,
            jacr.has_value() ? jacr->data() : nullptr, body);
      });
  Def<traits::mj_jacSubtreeCom>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         std::optional<Eigen::Ref<EigenArrayXX>> jacp, int body) {
        if (jacp.has_value() &&
            (jacp->rows() != 3 || jacp->cols() != m->nv)) {
          throw py::type_error("jacp should be of shape (3, nv)");
        }
        return InterceptMjErrors(::mj_jacSubtreeCom)(
            m, d, jacp.has_value() ? jacp->data() : nullptr, body);
      });
  Def<traits::mj_jacGeom>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         std::optional<Eigen::Ref<EigenArrayXX>> jacp,
         std::optional<Eigen::Ref<EigenArrayXX>> jacr, int geom) {
        if (jacp.has_value() &&
            (jacp->rows() != 3 || jacp->cols() != m->nv)) {
          throw py::type_error("jacp should be of shape (3, nv)");
        }
        if (jacr.has_value() &&
            (jacr->rows() != 3 || jacr->cols() != m->nv)) {
          throw py::type_error("jacr should be of shape (3, nv)");
        }
        return InterceptMjErrors(::mj_jacGeom)(
            m, d, jacp.has_value() ? jacp->data() : nullptr,
            jacr.has_value() ? jacr->data() : nullptr, geom);
      });
  Def<traits::mj_jacSite>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         std::optional<Eigen::Ref<EigenArrayXX>> jacp,
         std::optional<Eigen::Ref<EigenArrayXX>> jacr, int site) {
        if (jacp.has_value() &&
            (jacp->rows() != 3 || jacp->cols() != m->nv)) {
          throw py::type_error("jacp should be of shape (3, nv)");
        }
        if (jacr.has_value() &&
            (jacr->rows() != 3 || jacr->cols() != m->nv)) {
          throw py::type_error("jacr should be of shape (3, nv)");
        }
        return InterceptMjErrors(::mj_jacSite)(
            m, d, jacp.has_value() ? jacp->data() : nullptr,
            jacr.has_value() ? jacr->data() : nullptr, site);
      });
  Def<traits::mj_jacPointAxis>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         std::optional<Eigen::Ref<EigenArrayXX>> jacp,
         std::optional<Eigen::Ref<EigenArrayXX>> jacr,
         const mjtNum (*point)[3], const mjtNum (*axis)[3], int body) {
        if (jacp.has_value() &&
            (jacp->rows() != 3 || jacp->cols() != m->nv)) {
          throw py::type_error("jacp should be of shape (3, nv)");
        }
        if (jacr.has_value() &&
            (jacr->rows() != 3 || jacr->cols() != m->nv)) {
          throw py::type_error("jacr should be of shape (3, nv)");
        }
        return InterceptMjErrors(::mj_jacPointAxis)(
            m, d, jacp.has_value() ? jacp->data() : nullptr,
            jacr.has_value() ? jacr->data() : nullptr,
            &(*point)[0], &(*axis)[0], body);
      });
  Def<traits::mj_name2id>(pymodule);
  Def<traits::mj_id2name>(pymodule);
  Def<traits::mj_fullM>(
      pymodule,
      [](const raw::MjModel* m, Eigen::Ref<EigenArrayXX> dst,
         Eigen::Ref<const EigenVectorX> M) {
        if (M.size() != m->nM) {
          throw py::type_error("M should be of size nM");
        }
        if (dst.cols() != m->nv || dst.rows() != m->nv) {
          throw py::type_error("dst should be of shape (nv, nv)");
        }
        return ::mj_fullM(m, dst.data(), M.data());
      });
  Def<traits::mj_mulM>(
      pymodule,
      [](const raw::MjModel* m, const raw::MjData* d,
         Eigen::Ref<EigenVectorX> res, Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != m->nv) {
          throw py::type_error("res should be of size nv");
        }
        if (vec.size() != m->nv) {
          throw py::type_error("vec should be of size nv");
        }
        return InterceptMjErrors(::mj_mulM)(m, d, res.data(), vec.data());
      });
  Def<traits::mj_mulM2>(
      pymodule,
      [](const raw::MjModel* m, const raw::MjData* d,
         Eigen::Ref<EigenVectorX> res, Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != m->nv) {
          throw py::type_error("res should be of size nv");
        }
        if (vec.size() != m->nv) {
          throw py::type_error("vec should be of size nv");
        }
        return InterceptMjErrors(::mj_mulM2)(m, d, res.data(), vec.data());
      });
  Def<traits::mj_addM>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d, Eigen::Ref<EigenVectorX> dst,
         Eigen::Ref<Eigen::Vector<int, Eigen::Dynamic>> rownnz,
         Eigen::Ref<Eigen::Vector<int, Eigen::Dynamic>> rowadr,
         Eigen::Ref<Eigen::Vector<int, Eigen::Dynamic>> colind) {
        if (dst.size() != m->nM) {
          throw py::type_error("dst should be of size nM");
        }
        if (rownnz.size() != m->nv) {
          throw py::type_error("rownnz should be of size nv");
        }
        if (rowadr.size() != m->nv) {
          throw py::type_error("rowadr should be of size nv");
        }
        if (colind.size() != m->nM) {
          throw py::type_error("colind should be of size nM");
        }
        return InterceptMjErrors(::mj_addM)(
            m, d, dst.data(), rownnz.data(), rowadr.data(), colind.data());
      });
  Def<traits::mj_applyFT>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         const mjtNum (*force)[3], const mjtNum (*torque)[3],
         const mjtNum (*point)[3], int body,
         Eigen::Ref<EigenVectorX> qfrc_target) {
        if (qfrc_target.size() != m->nv) {
          throw py::type_error("qfrc_target should be of size nv");
        }
        return InterceptMjErrors(::mj_applyFT)(
            m, d, &(*force)[0], &(*torque)[0], &(*point)[0],
            body, qfrc_target.data());
      });
  Def<traits::mj_objectVelocity>(pymodule);
  Def<traits::mj_objectAcceleration>(pymodule);
  Def<traits::mj_contactForce>(pymodule);
  Def<traits::mj_differentiatePos>(
      pymodule,
      [](const raw::MjModel* m, Eigen::Ref<EigenVectorX> qvel,
         mjtNum dt, Eigen::Ref<const EigenVectorX> qpos1,
         Eigen::Ref<const EigenVectorX> qpos2) {
        if (qvel.size() != m->nv) {
          throw py::type_error("qvel should be of size nv");
        }
        if (qpos1.size() != m->nq) {
          throw py::type_error("qpos1 should be of size nq");
        }
        if (qpos2.size() != m->nq) {
          throw py::type_error("qpos2 should be of size nq");
        }
        return InterceptMjErrors(::mj_differentiatePos)(
            m, qvel.data(), dt, qpos1.data(), qpos2.data());
      });
  Def<traits::mj_integratePos>(
      pymodule,
      [](const raw::MjModel* m, Eigen::Ref<EigenVectorX> qpos,
         Eigen::Ref<const EigenVectorX> qvel, mjtNum dt) {
        if (qpos.size() != m->nq) {
          throw py::type_error("qpos should be of size nq");
        }
        if (qvel.size() != m->nv) {
          throw py::type_error("qvel should be of size nv");
        }
        return InterceptMjErrors(::mj_integratePos)(
            m, qpos.data(), qvel.data(), dt);
      });
  Def<traits::mj_normalizeQuat>(
      pymodule,
      [](const raw::MjModel* m, Eigen::Ref<EigenVectorX> qpos) {
        if (qpos.size() != m->nq) {
          throw py::type_error("qpos should be of size nq");
        }
        return InterceptMjErrors(::mj_normalizeQuat)(m, qpos.data());
      });
  Def<traits::mj_local2Global>(pymodule);
  Def<traits::mj_getTotalmass>(pymodule);
  Def<traits::mj_setTotalmass>(pymodule);
  Def<traits::mj_loadPluginLibrary>(pymodule);
  DEF_WITH_OMITTED_PY_ARGS(traits::mj_loadAllPluginLibraries, "callback")(
      pymodule,
      [](const std::string& directory) {
        InterceptMjErrors(::mj_loadAllPluginLibraries)(
            directory.c_str(), nullptr);
      });
  Def<traits::mj_version>(pymodule);
  Def<traits::mj_versionString>(pymodule);

  // Ray collision
  Def<traits::mj_multiRay>(pymodule);
  Def<traits::mj_ray>(
      pymodule,
      [](const raw::MjModel* m, const raw::MjData* d, const mjtNum(*pnt)[3],
         const mjtNum(*vec)[3],
         std::optional<Eigen::Ref<const Eigen::Vector<mjtByte, mjNGROUP>>>
             geomgroup,
         mjtByte flg_static, int bodyexclude, int(*geomid)[1]) {
        return mj_ray(m, d, &(*pnt)[0], &(*vec)[0],
                      geomgroup.has_value() ? geomgroup->data() : nullptr,
                      flg_static, bodyexclude, &(*geomid)[0]);
      });
  Def<traits::mj_rayHfield>(pymodule);
  Def<traits::mj_rayMesh>(pymodule);
  Def<traits::mju_rayGeom>(pymodule);
  Def<traits::mju_raySkin>(pymodule);

  // Interaction
  Def<traits::mjv_defaultCamera>(pymodule);
  Def<traits::mjv_defaultFreeCamera>(pymodule);
  Def<traits::mjv_defaultPerturb>(pymodule);
  Def<traits::mjv_room2model>(pymodule);
  Def<traits::mjv_model2room>(pymodule);
  Def<traits::mjv_cameraInModel>(pymodule);
  Def<traits::mjv_cameraInRoom>(pymodule);
  Def<traits::mjv_frustumHeight>(pymodule);
  Def<traits::mjv_alignToCamera>(pymodule);
  Def<traits::mjv_moveCamera>(pymodule);
  Def<traits::mjv_movePerturb>(pymodule);
  Def<traits::mjv_moveModel>(pymodule);
  Def<traits::mjv_initPerturb>(pymodule);
  Def<traits::mjv_applyPerturbPose>(pymodule);
  Def<traits::mjv_applyPerturbForce>(pymodule);
  // Skipped: mjv_averageCamera (defined in structs.cc due to the return type)
  Def<traits::mjv_select>(pymodule);

  // Visualization
  Def<traits::mjv_defaultOption>(pymodule);
  Def<traits::mjv_defaultFigure>(pymodule);
  Def<traits::mjv_initGeom>(pymodule);
  Def<traits::mjv_makeConnector>(pymodule);
  Def<traits::mjv_connector>(pymodule);
  // Skipped: mjv_defaultScene (have MjvScene.__init__, memory managed by
  // MjvScene).
  // Skipped: mjv_makeScene (have MjvScene.__init__)
  // Skipped: mjv_freeScene (have MjvScene.__del__)
  Def<traits::mjv_updateScene>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d, const raw::MjvOption* opt,
         const std::optional<raw::MjvPerturb*> pert, raw::MjvCamera* cam,
         int catmask, raw::MjvScene* scn) {
        const raw::MjvPerturb* pert_ptr = pert.has_value() ? *pert : nullptr;
        return mjv_updateScene(m, d, opt, pert_ptr, cam, catmask, scn);
      });
  Def<traits::mjv_addGeoms>(pymodule);
  Def<traits::mjv_makeLights>(pymodule);
  Def<traits::mjv_updateCamera>(pymodule);
  Def<traits::mjv_updateSkin>(pymodule);

  // UI framework
  // Skipped: entire section (can add this if there's demand)

  // Error and memory
  // Skipped: everything other than the function below (Python has exceptions)
  Def<traits::mju_writeLog>(pymodule);

  // Standard math
  // This section consists only of preprocessor macros.

  // Vector math
  Def<traits::mju_zero3>(pymodule);
  Def<traits::mju_copy3>(pymodule);
  Def<traits::mju_scl3>(pymodule);
  Def<traits::mju_add3>(pymodule);
  Def<traits::mju_sub3>(pymodule);
  Def<traits::mju_addTo3>(pymodule);
  Def<traits::mju_subFrom3>(pymodule);
  Def<traits::mju_addToScl3>(pymodule);
  Def<traits::mju_addScl3>(pymodule);
  Def<traits::mju_normalize3>(pymodule);
  Def<traits::mju_norm3>(pymodule);
  Def<traits::mju_dot3>(pymodule);
  Def<traits::mju_dist3>(pymodule);
  Def<traits::mju_rotVecMat>(pymodule);
  Def<traits::mju_rotVecMatT>(pymodule);
  Def<traits::mju_cross>(pymodule);
  Def<traits::mju_zero4>(pymodule);
  Def<traits::mju_unit4>(pymodule);
  Def<traits::mju_copy4>(pymodule);
  Def<traits::mju_normalize4>(pymodule);
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_zero, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res) {
        return InterceptMjErrors(::mju_zero)(res.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_fill, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res, mjtNum val) {
        return InterceptMjErrors(::mju_fill)(res.data(), val, res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_copy, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> data) {
        if (res.size() != data.size()) {
          throw py::type_error("res and data should have the same size");
        }
        return InterceptMjErrors(::mju_copy)(
            res.data(), data.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_sum, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> vec) {
        return ::mju_sum(vec.data(), vec.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_L1, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> vec) {
        return InterceptMjErrors(::mju_L1)(vec.data(), vec.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_scl, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> vec, mjtNum scl) {
        if (res.size() != vec.size()) {
          throw py::type_error("res and vec should have the same size");
        }
        return InterceptMjErrors(::mju_scl)(
            res.data(), vec.data(), scl, res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_add, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> vec1,
         Eigen::Ref<const EigenVectorX> vec2) {
        if (res.size() != vec1.size()) {
          throw py::type_error("res and vec1 should have the same size");
        }
        if (res.size() != vec2.size()) {
          throw py::type_error("res and vec2 should have the same size");
        }
        return ::mju_add(res.data(), vec1.data(), vec2.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_sub, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> vec1,
         Eigen::Ref<const EigenVectorX> vec2) {
        if (res.size() != vec1.size()) {
          throw py::type_error("res and vec1 should have the same size");
        }
        if (res.size() != vec2.size()) {
          throw py::type_error("res and vec2 should have the same size");
        }
        return InterceptMjErrors(::mju_sub)(
            res.data(), vec1.data(), vec2.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_addTo, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != vec.size()) {
          throw py::type_error("res and vec should have the same size");
        }
        return InterceptMjErrors(::mju_addTo)(
            res.data(), vec.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_subFrom, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != vec.size()) {
          throw py::type_error("res and vec should have the same size");
        }
        return InterceptMjErrors(::mju_subFrom)(
            res.data(), vec.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_addToScl, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> vec, mjtNum scl) {
        if (res.size() != vec.size()) {
          throw py::type_error("res and vec should have the same size");
        }
        return ::mju_addToScl(res.data(), vec.data(), scl, res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_addScl, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenVectorX> vec1,
         Eigen::Ref<const EigenVectorX> vec2, mjtNum scl) {
        if (res.size() != vec1.size()) {
          throw py::type_error("res and vec1 should have the same size");
        }
        if (res.size() != vec2.size()) {
          throw py::type_error("res and vec2 should have the same size");
        }
        return InterceptMjErrors(::mju_addScl)(
            res.data(), vec1.data(), vec2.data(), scl, res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_normalize, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> vec) {
        return InterceptMjErrors(::mju_normalize)(vec.data(), vec.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_norm, "n")(
      pymodule,
      [](Eigen::Ref<const EigenVectorX> vec) {
        return InterceptMjErrors(::mju_norm)(vec.data(), vec.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_dot, "n")(
      pymodule,
      [](Eigen::Ref<const EigenVectorX> vec1,
         Eigen::Ref<const EigenVectorX> vec2) {
        if (vec1.size() != vec2.size()) {
          throw py::type_error("vec1 and vec2 should have the same size");
        }
        return InterceptMjErrors(::mju_dot)(
            vec1.data(), vec2.data(), vec1.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_mulMatVec, "nr", "nc")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenArrayXX> mat,
         Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != mat.rows()) {
          throw py::type_error(
              "size of res should equal the number of rows in mat");
        }
        if (vec.size() != mat.cols()) {
          throw py::type_error(
              "size of vec should equal the number of columns in mat");
        }
        return InterceptMjErrors(::mju_mulMatVec)(
            res.data(), mat.data(), vec.data(), mat.rows(), mat.cols());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_mulMatTVec, "nr", "nc")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenArrayXX> mat,
         Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != mat.cols()) {
          throw py::type_error(
              "size of res should equal the number of columns in mat");
        }
        if (vec.size() != mat.rows()) {
          throw py::type_error(
              "size of vec should equal the number of rows in mat");
        }
        return InterceptMjErrors(::mju_mulMatTVec)(
            res.data(), mat.data(), vec.data(), mat.rows(), mat.cols());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_mulVecMatVec, "n")(
      pymodule,
      [](Eigen::Ref<const EigenVectorX> vec1,
         Eigen::Ref<const EigenArrayXX> mat,
         Eigen::Ref<const EigenVectorX> vec2) {
        if (vec1.size() != vec2.size()) {
          throw py::type_error(
              "size of vec1 should equal the size of vec2");
        }
        if (vec1.size() != mat.cols()) {
          throw py::type_error(
              "size of vectors should equal the number of columns in mat");
        }
        if (vec1.size() != mat.rows()) {
          throw py::type_error(
              "size of vectors should equal the number of rows in mat");
        }
        return InterceptMjErrors(::mju_mulVecMatVec)(
            vec1.data(), mat.data(), vec2.data(), vec1.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_transpose, "nr", "nc")(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> res,
         Eigen::Ref<const EigenArrayXX> mat) {
        if (res.cols() != mat.rows()) {
          throw py::type_error("#columns in res should equal #rows in mat");
        }
        if (res.rows() != mat.cols()) {
          throw py::type_error("#rows in res should equal #columns in mat");
        }
        return InterceptMjErrors(::mju_transpose)(
            res.data(), mat.data(), mat.rows(), mat.cols());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_symmetrize, "n")(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> res,
         Eigen::Ref<const EigenArrayXX> mat) {
        if (mat.cols() != mat.rows()) {
          throw py::type_error("mat should be square");
        }
        if (res.cols() != mat.cols() || res.rows() != mat.rows()) {
          throw py::type_error("res and mat should have the same shape");
        }
        return InterceptMjErrors(::mju_symmetrize)(
            res.data(), mat.data(), mat.rows());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_eye, "n")(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> mat) {
        if (mat.cols() != mat.rows()) {
          throw py::type_error("mat should be square");
        }
        return InterceptMjErrors(::mju_eye)(mat.data(), mat.rows());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_mulMatMat, "r1", "c1", "c2")(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> res,
         Eigen::Ref<const EigenArrayXX> mat1,
         Eigen::Ref<const EigenArrayXX> mat2) {
        if (res.rows() != mat1.rows()) {
          throw py::type_error("#rows in res should equal #rows in mat1");
        }
        if (res.cols() != mat2.cols()) {
          throw py::type_error(
              "#columns in res should equal #columns in mat2");
        }
        if (mat1.cols() != mat2.rows()) {
          throw py::type_error("#columns in mat1 should equal #rows in mat2");
        }
        return InterceptMjErrors(::mju_mulMatMat)(
            res.data(), mat1.data(), mat2.data(),
            mat1.rows(), mat1.cols(), mat2.cols());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_mulMatMatT, "r1", "c1", "r2")(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> res,
         Eigen::Ref<const EigenArrayXX> mat1,
         Eigen::Ref<const EigenArrayXX> mat2) {
        if (res.rows() != mat1.rows()) {
          throw py::type_error("#rows in res should equal #rows in mat1");
        }
        if (res.cols() != mat2.rows()) {
          throw py::type_error("#columns in res should equal #rows in mat2");
        }
        if (mat1.cols() != mat2.cols()) {
          throw py::type_error(
              "#columns in mat1 should equal #columns in mat2");
        }
        return InterceptMjErrors(::mju_mulMatMatT)(
            res.data(), mat1.data(), mat2.data(),
            mat1.rows(), mat1.cols(), mat2.rows());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_mulMatTMat, "r1", "c1", "c2")(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> res,
         Eigen::Ref<const EigenArrayXX> mat1,
         Eigen::Ref<const EigenArrayXX> mat2) {
        if (res.rows() != mat1.cols()) {
          throw py::type_error("#rows in res should equal #columns in mat1");
        }
        if (res.cols() != mat2.cols()) {
          throw py::type_error(
              "#columns in res should equal #columns in mat2");
        }
        if (mat1.rows() != mat2.rows()) {
          throw py::type_error("#rows in mat1 should equal #rows in mat2");
        }
        return ::mju_mulMatTMat(res.data(), mat1.data(), mat2.data(),
                                mat1.rows(), mat1.cols(), mat2.cols());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_sqrMatTD, "nr", "nc")(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> res,
         Eigen::Ref<const EigenArrayXX> mat,
         std::optional<Eigen::Ref<EigenVectorX>> diag) {
        if (res.rows() != mat.cols()) {
          throw py::type_error("#rows in res should equal #columns in mat");
        }
        if (res.cols() != mat.cols()) {
          throw py::type_error("#rows in res should equal #columns in mat");
        }
        if (diag.has_value() && diag->size() != mat.rows()) {
          throw py::type_error(
              "size of diag should equal the number of rows in mat");
        }
        return InterceptMjErrors(::mju_sqrMatTD)(
            res.data(), mat.data(),
            diag.has_value() ? diag->data() : nullptr,
            mat.rows(), mat.cols());
      });
  Def<traits::mju_transformSpatial>(pymodule);

  // Quaternions
  Def<traits::mju_rotVecQuat>(pymodule);
  Def<traits::mju_negQuat>(pymodule);
  Def<traits::mju_mulQuat>(pymodule);
  Def<traits::mju_mulQuatAxis>(pymodule);
  Def<traits::mju_axisAngle2Quat>(pymodule);
  Def<traits::mju_quat2Vel>(pymodule);
  Def<traits::mju_subQuat>(pymodule);
  Def<traits::mju_quat2Mat>(pymodule);
  Def<traits::mju_mat2Quat>(pymodule);
  Def<traits::mju_derivQuat>(pymodule);
  Def<traits::mju_quatIntegrate>(pymodule);
  Def<traits::mju_quatZ2Vec>(pymodule);

  // Poses
  Def<traits::mju_mulPose>(pymodule);
  Def<traits::mju_negPose>(pymodule);
  Def<traits::mju_trnVecPose>(pymodule);

  // Decompositions
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_cholFactor, "n")(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> mat, mjtNum mindiag) {
        if (mat.rows() != mat.cols()) {
          throw py::type_error("mat should be a square matrix");
        }
        return InterceptMjErrors(::mju_cholFactor)(
            mat.data(), mat.rows(), mindiag);
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_cholSolve, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const EigenArrayXX> mat,
         Eigen::Ref<const EigenVectorX> vec) {
        if (mat.rows() != mat.cols()) {
          throw py::type_error("mat should be a square matrix");
        }
        if (res.size() != mat.rows()) {
          throw py::type_error(
              "size of res should equal the number of rows in mat");
        }
        if (vec.size() != mat.cols()) {
          throw py::type_error(
              "size of vec should equal the number of rows in mat");
        }
        return InterceptMjErrors(::mju_cholSolve)(
            res.data(), mat.data(), vec.data(), mat.rows());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_cholUpdate, "n")(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> mat, Eigen::Ref<EigenVectorX> x,
         int flg_plus) {
        if (mat.rows() != mat.cols()) {
          throw py::type_error("mat should be a square matrix");
        }
        if (x.size() != mat.rows()) {
          throw py::type_error(
              "size of x should equal the number of rows in mat");
        }
        return InterceptMjErrors(::mju_cholUpdate)(
            mat.data(), x.data(), mat.rows(), flg_plus);
      });
  Def<traits::mju_cholFactorBand>(
      pymodule, [](Eigen::Ref<EigenVectorX> mat, int ntotal, int nband,
                   int ndense, mjtNum diagadd, mjtNum diagmul) {
        int nMat = (ntotal - ndense) * nband + ndense * ntotal;
        if (mat.size() != nMat) {
          throw py::type_error(
              "mat must have size (ntotal-ndense)*nband + ndense*ntotal");
        }
        return InterceptMjErrors(::mju_cholFactorBand)(
            mat.data(), ntotal, nband, ndense, diagadd, diagmul);
      });
  Def<traits::mju_cholSolveBand>(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res, Eigen::Ref<const EigenVectorX> mat,
         Eigen::Ref<const EigenVectorX> vec, int ntotal, int nband,
         int ndense) {
        int nMat = (ntotal - ndense) * nband + ndense * ntotal;
        if (mat.size() != nMat) {
          throw py::type_error(
              "mat must have (ntotal-ndense)*nband + "
              "ndense*ntotal elements");
        }
        if (res.size() != ntotal) {
          throw py::type_error("size of res should equal ntotal");
        }
        if (vec.size() != ntotal) {
          throw py::type_error("size of vec should equal ntotal");
        }
        return InterceptMjErrors(::mju_cholSolveBand)(
            res.data(), mat.data(), vec.data(), ntotal, nband, ndense);
      });
  Def<traits::mju_band2Dense>(
      pymodule,
      [](Eigen::Ref<EigenArrayXX> res, Eigen::Ref<const EigenVectorX> mat,
         int ntotal, int nband, int ndense, mjtByte flg_sym) {
        int nMat = (ntotal - ndense) * nband + ndense * ntotal;
        if (mat.size() != nMat) {
          throw py::type_error(
              "mat must have size (ntotal-ndense)*nband + ndense*ntotal");
        }
        if (res.rows() != ntotal) {
          throw py::type_error("res should have ntotal rows");
        }
        if (res.cols() != ntotal) {
          throw py::type_error("res should have ntotal columns");
        }
        return InterceptMjErrors(::mju_band2Dense)(
            res.data(), mat.data(), ntotal, nband, ndense, flg_sym);
      });
  Def<traits::mju_dense2Band>(pymodule, [](Eigen::Ref<EigenVectorX> res,
                                           Eigen::Ref<const EigenArrayXX> mat,
                                           int ntotal, int nband, int ndense) {
    int nRes = (ntotal - ndense) * nband + ndense * ntotal;
    if (res.size() != nRes) {
      throw py::type_error(
          "res must have size (ntotal-ndense)*nband + ndense*ntotal");
    }
    if (mat.rows() != ntotal) {
      throw py::type_error("mat should have ntotal rows");
    }
    if (mat.cols() != ntotal) {
      throw py::type_error("mat should have ntotal columns");
    }
    return InterceptMjErrors(::mju_dense2Band)(res.data(), mat.data(), ntotal,
                                               nband, ndense);
  });
  Def<traits::mju_bandMulMatVec>(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res, Eigen::Ref<const EigenArrayXX> mat,
         Eigen::Ref<const EigenArrayXX> vec, int ntotal, int nband, int ndense,
         int nVec, mjtByte flg_sym) {
        int nMat = (ntotal - ndense) * nband + ndense * ntotal;
        if (mat.size() != nMat) {
          throw py::type_error(
              "mat must have size (ntotal-ndense)*nband + ndense*ntotal");
        }
        if (res.rows() != ntotal) {
          throw py::type_error("res should have ntotal rows");
        }
        if (res.cols() != nVec) {
          throw py::type_error("res should have nVec columns");
        }
        if (vec.rows() != ntotal) {
          throw py::type_error("vec should have ntotal rows");
        }
        if (vec.cols() != nVec) {
          throw py::type_error("vec should have nVec columns");
        }
        return InterceptMjErrors(::mju_bandMulMatVec)(res.data(), mat.data(),
                                                      vec.data(), ntotal, nband,
                                                      ndense, nVec, flg_sym);
      });
  Def<traits::mju_bandDiag>(pymodule);
  Def<traits::mju_eig3>(pymodule);
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_boxQP, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<EigenArrayXX> R,
         std::optional<Eigen::Ref<Eigen::Vector<int, Eigen::Dynamic>>> index,
         Eigen::Ref<const EigenArrayXX> H,
         Eigen::Ref<const EigenVectorX> g,
         std::optional<Eigen::Ref<const EigenVectorX>> lower,
         std::optional<Eigen::Ref<const EigenVectorX>> upper) {
        int n = res.size();
        if (R.size() != n*(n+7)) {
          throw py::type_error("size of R should be n*(n+7)");
        }
        if (index.has_value() && (index->size() != n)) {
          throw py::type_error("size of index should equal n");
        }
        if (H.rows() != n || H.cols() != n) {
          throw py::type_error("H should be of shape (n, n)");
        }
        if (g.size() != n) {
          throw py::type_error("size of g should equal n");
        }
        if (lower.has_value() && (lower->size() != n)) {
          throw py::type_error("size of lower should equal n");
        }
        if (upper.has_value() && (upper->size() != n)) {
          throw py::type_error("size of upper should equal n");
        }
        return InterceptMjErrors(::mju_boxQP)(
            res.data(), R.data(),
            index.has_value() ? index->data() : nullptr,
            H.data(), g.data(), n,
            lower.has_value() ? lower->data() : nullptr,
            upper.has_value() ? upper->data() : nullptr);
      });
  // Miscellaneous
  Def<traits::mju_muscleGain>(pymodule);
  Def<traits::mju_muscleBias>(pymodule);
  Def<traits::mju_muscleDynamics>(pymodule);
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_encodePyramid, "dim")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> pyramid, Eigen::Ref<const EigenVectorX> force,
         Eigen::Ref<const EigenVectorX> mu) {
        if (pyramid.size() != 2*mu.size()) {
          throw py::type_error(
              "size of pyramid should be twice as large as size of mu");
        }
        if (force.size() != mu.size() + 1) {
          throw py::type_error(
              "size of force should be exactly one larger than size of mu");
        }
        return InterceptMjErrors(::mju_encodePyramid)(
            pyramid.data(), force.data(), mu.data(), mu.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_decodePyramid, "dim")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> force, Eigen::Ref<const EigenVectorX> pyramid,
         Eigen::Ref<const EigenVectorX> mu) {
        if (pyramid.size() != 2*mu.size()) {
          throw py::type_error(
              "size of pyramid should be twice as large as size of mu");
        }
        if (force.size() != mu.size() + 1) {
          throw py::type_error(
              "size of force should be exactly one larger than size of mu");
        }
        return InterceptMjErrors(::mju_decodePyramid)(
            force.data(), pyramid.data(), mu.data(), mu.size());
      });
  Def<traits::mju_springDamper>(pymodule);
  Def<traits::mju_min>(pymodule);
  Def<traits::mju_max>(pymodule);
  Def<traits::mju_clip>(pymodule);
  Def<traits::mju_sign>(pymodule);
  Def<traits::mju_round>(pymodule);
  Def<traits::mju_type2Str>(pymodule);
  Def<traits::mju_str2Type>(pymodule);
  Def<traits::mju_writeNumBytes>(pymodule);
  Def<traits::mju_warningText>(pymodule);
  Def<traits::mju_isBad>(pymodule);
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_isZero, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> vec) {
        return InterceptMjErrors(::mju_isZero)(vec.data(), vec.size());
      });
  Def<traits::mju_standardNormal>(
      pymodule,
      [](std::optional<mjtNum> num2) {
        return InterceptMjErrors(::mju_standardNormal)(
            num2.has_value() ? &*num2 : nullptr);
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_f2n, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const Eigen::Vector<float, Eigen::Dynamic>> vec) {
        if (res.size() != vec.size()) {
          throw py::type_error("res and vec should have the same size");
        }
        return InterceptMjErrors(::mju_f2n)(res.data(), vec.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_n2f, "n")(
      pymodule,
      [](Eigen::Ref<Eigen::Vector<float, Eigen::Dynamic>> res,
         Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != vec.size()) {
          throw py::type_error("res and vec should have the same size");
        }
        return InterceptMjErrors(::mju_n2f)(res.data(), vec.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_d2n, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res,
         Eigen::Ref<const Eigen::Vector<double, Eigen::Dynamic>> vec) {
        if (res.size() != vec.size()) {
          throw py::type_error("res and vec should have the same size");
        }
        return InterceptMjErrors(::mju_d2n)(res.data(), vec.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_n2d, "n")(
      pymodule,
      [](Eigen::Ref<Eigen::Vector<double, Eigen::Dynamic>> res,
         Eigen::Ref<const EigenVectorX> vec) {
        if (res.size() != vec.size()) {
          throw py::type_error("res and vec should have the same size");
        }
        return InterceptMjErrors(::mju_n2d)(res.data(), vec.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_insertionSort, "n")(
      pymodule,
      [](Eigen::Ref<EigenVectorX> res) {
        return InterceptMjErrors(::mju_insertionSort)(res.data(), res.size());
      });
  DEF_WITH_OMITTED_PY_ARGS(traits::mju_insertionSortInt, "n")(
      pymodule,
      [](Eigen::Ref<Eigen::Vector<int, Eigen::Dynamic>> res) {
        return InterceptMjErrors(::mju_insertionSortInt)(
            res.data(), res.size());
      });
  Def<traits::mju_Halton>(pymodule);
  // Skipped: mju_strncpy (doesn't make sense in Python)
  Def<traits::mju_sigmoid>(pymodule);

  // Derivatives
  Def<traits::mjd_transitionFD>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         mjtNum eps, mjtByte flg_centered,
         std::optional<Eigen::Ref<EigenArrayXX>> A,
         std::optional<Eigen::Ref<EigenArrayXX>> B,
         std::optional<Eigen::Ref<EigenArrayXX>> C,
         std::optional<Eigen::Ref<EigenArrayXX>> D) {
        if (A.has_value() &&
            (A->rows() != 2*m->nv+m->na || A->cols() != 2*m->nv+m->na)) {
          throw py::type_error("A should be of shape (2*nv+na, 2*nv+na)");
        }
        if (B.has_value() &&
            (B->rows() != 2*m->nv+m->na || B->cols() != m->nu)) {
          throw py::type_error("B should be of shape (2*nv+na, nu)");
        }
        if (C.has_value() &&
            (C->rows() != m->nsensordata || C->cols() != 2*m->nv+m->na)) {
          throw py::type_error("C should be of shape (nsensordata, 2*nv+na)");
        }
        if (D.has_value() &&
            (D->rows() != m->nsensordata || D->cols() != m->nu)) {
          throw py::type_error("D should be of shape (nsensordata, nu)");
        }
        return InterceptMjErrors(::mjd_transitionFD)(
            m, d, eps, flg_centered,
            A.has_value() ? A->data() : nullptr,
            B.has_value() ? B->data() : nullptr,
            C.has_value() ? C->data() : nullptr,
            D.has_value() ? D->data() : nullptr);
      });
  Def<traits::mjd_inverseFD>(
      pymodule,
      [](const raw::MjModel* m, raw::MjData* d,
         mjtNum eps, mjtByte flg_actuation,
         std::optional<Eigen::Ref<EigenArrayXX>> DfDq,
         std::optional<Eigen::Ref<EigenArrayXX>> DfDv,
         std::optional<Eigen::Ref<EigenArrayXX>> DfDa,
         std::optional<Eigen::Ref<EigenArrayXX>> DsDq,
         std::optional<Eigen::Ref<EigenArrayXX>> DsDv,
         std::optional<Eigen::Ref<EigenArrayXX>> DsDa,
         std::optional<Eigen::Ref<EigenArrayXX>> DmDq) {
        if (DfDq.has_value() &&
            (DfDq->rows() != m->nv || DfDq->cols() != m->nv)) {
          throw py::type_error("DfDq should be of shape (nv, nv)");
        }
        if (DfDv.has_value() &&
            (DfDv->rows() != m->nv || DfDv->cols() != m->nv)) {
          throw py::type_error("DfDv should be of shape (nv, nv)");
        }
        if (DfDa.has_value() &&
            (DfDa->rows() != m->nv || DfDa->cols() != m->nv)) {
          throw py::type_error("DfDa should be of shape (nv, nv)");
        }
        if (DsDq.has_value() &&
            (DsDq->rows() != m->nv || DsDq->cols() != m->nsensordata)) {
          throw py::type_error("DsDq should be of shape (nv, nsensordata)");
        }
        if (DsDv.has_value() &&
            (DsDv->rows() != m->nv || DsDv->cols() != m->nsensordata)) {
          throw py::type_error("DsDv should be of shape (nv, nsensordata)");
        }
        if (DsDa.has_value() &&
            (DsDa->rows() != m->nv || DsDa->cols() != m->nsensordata)) {
          throw py::type_error("DsDa should be of shape (nv, nsensordata)");
        }
        if (DmDq.has_value() &&
            (DmDq->rows() != m->nv || DmDq->cols() != m->nM)) {
          throw py::type_error("DmDq should be of shape (nv, nM)");
        }
        return InterceptMjErrors(::mjd_inverseFD)(
            m, d, eps, flg_actuation,
            DfDq.has_value() ? DfDq->data() : nullptr,
            DfDv.has_value() ? DfDv->data() : nullptr,
            DfDa.has_value() ? DfDa->data() : nullptr,
            DsDq.has_value() ? DsDq->data() : nullptr,
            DsDv.has_value() ? DsDv->data() : nullptr,
            DsDa.has_value() ? DsDa->data() : nullptr,
            DmDq.has_value() ? DmDq->data() : nullptr);
      });
  Def<traits::mjd_subQuat>(pymodule);
  Def<traits::mjd_quatIntegrate>(pymodule);
}  // PYBIND11_MODULE NOLINT(readability/fn_size)
}  // namespace
}  // namespace mujoco::python
