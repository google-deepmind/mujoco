// Copyright 2025 DeepMind Technologies Limited
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

// NOLINTBEGIN(whitespace/line_length)
// NOLINTBEGIN(whitespace/semicolon)

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>  // NOLINT
#include <memory>
#include <optional>  // NOLINT
#include <string>    // NOLINT
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "engine/engine_util_errmem.h"
#include "wasm/unpack.h"

namespace mujoco::wasm {

using emscripten::enum_;
using emscripten::function;
using emscripten::val;
using emscripten::return_value_policy::reference;
using emscripten::return_value_policy::take_ownership;

EMSCRIPTEN_DECLARE_VAL_TYPE(NumberArray);
EMSCRIPTEN_DECLARE_VAL_TYPE(String);

// Raises an error if the given val is null or undefined.
// A macro is used so that the error contains the name of the variable.
// TODO(matijak): Remove this when we can handle strings using UNPACK_STRING?
#define CHECK_VAL(val)                                    \
  if (val.isNull()) {                                     \
    mju_error("Invalid argument: %s is null", #val);      \
  } else if (val.isUndefined()) {                         \
    mju_error("Invalid argument: %s is undefined", #val); \
  }

void ThrowMujocoErrorToJS(const char* msg) {
  // Get a handle to the JS global Error constructor function, create a new
  // object instance and then throw the object as an exception using the
  // val::throw_() helper function.
  val(val::global("Error").new_(val("MuJoCo Error: " + std::string(msg))))
      .throw_();
}
__attribute__((constructor)) void InitMuJoCoErrorHandler() {
  mju_user_error = ThrowMujocoErrorToJS;
}

template <size_t N>
val MakeValArray(const char* (&strings)[N]) {
  val result = val::array();
  for (int i = 0; i < N; i++) {
    result.call<void>("push", val(strings[i]));
  }
  return result;
}

template <size_t N, size_t M>
val MakeValArray3(const char* (&strings)[N][M]) {
  val result = val::array();
  for (int i = 0; i < N; i++) {
    val inner = val::array();
    for (int j = 0; j < M; j++) {
      inner.call<void>("push", val(strings[i][j]));
    }
    result.call<void>("push", inner);
  }
  return result;
}

template <typename WrapperType, typename ArrayType, typename SizeType>
std::vector<WrapperType> InitWrapperArray(ArrayType* array, SizeType size) {
  std::vector<WrapperType> result;
  result.reserve(size);
  for (int i = 0; i < size; ++i) {
    result.emplace_back(&array[i]);
  }
  return result;
}

val get_mjDISABLESTRING() { return MakeValArray(mjDISABLESTRING); }
val get_mjENABLESTRING() { return MakeValArray(mjENABLESTRING); }
val get_mjTIMERSTRING() { return MakeValArray(mjTIMERSTRING); }
val get_mjLABELSTRING() { return MakeValArray(mjLABELSTRING); }
val get_mjFRAMESTRING() { return MakeValArray(mjFRAMESTRING); }
val get_mjVISSTRING() { return MakeValArray3(mjVISSTRING); }
val get_mjRNDSTRING() { return MakeValArray3(mjRNDSTRING); }


using mjVisualGlobal = decltype(::mjVisual::global);
using mjVisualHeadlight = decltype(::mjVisual::headlight);
using mjVisualMap = decltype(::mjVisual::map);
using mjVisualQuality = decltype(::mjVisual::quality);
using mjVisualRgba = decltype(::mjVisual::rgba);
using mjVisualScale = decltype(::mjVisual::scale);

struct MjContact {
  ~MjContact();
  MjContact();
  explicit MjContact(mjContact *ptr);
  MjContact(const MjContact &);
  MjContact &operator=(const MjContact &);
  std::unique_ptr<MjContact> copy();
  mjContact* get() const;
  void set(mjContact* ptr);
  mjtNum dist() const {
    return ptr_->dist;
  }
  void set_dist(mjtNum value) {
    ptr_->dist = value;
  }
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val frame() const {
    return emscripten::val(emscripten::typed_memory_view(9, ptr_->frame));
  }
  mjtNum includemargin() const {
    return ptr_->includemargin;
  }
  void set_includemargin(mjtNum value) {
    ptr_->includemargin = value;
  }
  emscripten::val friction() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->friction));
  }
  emscripten::val solref() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solref));
  }
  emscripten::val solreffriction() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solreffriction));
  }
  emscripten::val solimp() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->solimp));
  }
  mjtNum mu() const {
    return ptr_->mu;
  }
  void set_mu(mjtNum value) {
    ptr_->mu = value;
  }
  emscripten::val H() const {
    return emscripten::val(emscripten::typed_memory_view(36, ptr_->H));
  }
  int dim() const {
    return ptr_->dim;
  }
  void set_dim(int value) {
    ptr_->dim = value;
  }
  int geom1() const {
    return ptr_->geom1;
  }
  void set_geom1(int value) {
    ptr_->geom1 = value;
  }
  int geom2() const {
    return ptr_->geom2;
  }
  void set_geom2(int value) {
    ptr_->geom2 = value;
  }
  emscripten::val geom() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->geom));
  }
  emscripten::val flex() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->flex));
  }
  emscripten::val elem() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->elem));
  }
  emscripten::val vert() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->vert));
  }
  int exclude() const {
    return ptr_->exclude;
  }
  void set_exclude(int value) {
    ptr_->exclude = value;
  }
  int efc_address() const {
    return ptr_->efc_address;
  }
  void set_efc_address(int value) {
    ptr_->efc_address = value;
  }

 private:
  mjContact* ptr_;
  bool owned_ = false;
};

struct MjLROpt {
  ~MjLROpt();
  MjLROpt();
  explicit MjLROpt(mjLROpt *ptr);
  MjLROpt(const MjLROpt &);
  MjLROpt &operator=(const MjLROpt &);
  std::unique_ptr<MjLROpt> copy();
  mjLROpt* get() const;
  void set(mjLROpt* ptr);
  int mode() const {
    return ptr_->mode;
  }
  void set_mode(int value) {
    ptr_->mode = value;
  }
  int useexisting() const {
    return ptr_->useexisting;
  }
  void set_useexisting(int value) {
    ptr_->useexisting = value;
  }
  int uselimit() const {
    return ptr_->uselimit;
  }
  void set_uselimit(int value) {
    ptr_->uselimit = value;
  }
  mjtNum accel() const {
    return ptr_->accel;
  }
  void set_accel(mjtNum value) {
    ptr_->accel = value;
  }
  mjtNum maxforce() const {
    return ptr_->maxforce;
  }
  void set_maxforce(mjtNum value) {
    ptr_->maxforce = value;
  }
  mjtNum timeconst() const {
    return ptr_->timeconst;
  }
  void set_timeconst(mjtNum value) {
    ptr_->timeconst = value;
  }
  mjtNum timestep() const {
    return ptr_->timestep;
  }
  void set_timestep(mjtNum value) {
    ptr_->timestep = value;
  }
  mjtNum inttotal() const {
    return ptr_->inttotal;
  }
  void set_inttotal(mjtNum value) {
    ptr_->inttotal = value;
  }
  mjtNum interval() const {
    return ptr_->interval;
  }
  void set_interval(mjtNum value) {
    ptr_->interval = value;
  }
  mjtNum tolrange() const {
    return ptr_->tolrange;
  }
  void set_tolrange(mjtNum value) {
    ptr_->tolrange = value;
  }

 private:
  mjLROpt* ptr_;
  bool owned_ = false;
};

struct MjOption {
  ~MjOption();
  MjOption();
  explicit MjOption(mjOption *ptr);
  MjOption(const MjOption &);
  MjOption &operator=(const MjOption &);
  std::unique_ptr<MjOption> copy();
  mjOption* get() const;
  void set(mjOption* ptr);
  mjtNum timestep() const {
    return ptr_->timestep;
  }
  void set_timestep(mjtNum value) {
    ptr_->timestep = value;
  }
  mjtNum impratio() const {
    return ptr_->impratio;
  }
  void set_impratio(mjtNum value) {
    ptr_->impratio = value;
  }
  mjtNum tolerance() const {
    return ptr_->tolerance;
  }
  void set_tolerance(mjtNum value) {
    ptr_->tolerance = value;
  }
  mjtNum ls_tolerance() const {
    return ptr_->ls_tolerance;
  }
  void set_ls_tolerance(mjtNum value) {
    ptr_->ls_tolerance = value;
  }
  mjtNum noslip_tolerance() const {
    return ptr_->noslip_tolerance;
  }
  void set_noslip_tolerance(mjtNum value) {
    ptr_->noslip_tolerance = value;
  }
  mjtNum ccd_tolerance() const {
    return ptr_->ccd_tolerance;
  }
  void set_ccd_tolerance(mjtNum value) {
    ptr_->ccd_tolerance = value;
  }
  mjtNum sleep_tolerance() const {
    return ptr_->sleep_tolerance;
  }
  void set_sleep_tolerance(mjtNum value) {
    ptr_->sleep_tolerance = value;
  }
  emscripten::val gravity() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->gravity));
  }
  emscripten::val wind() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->wind));
  }
  emscripten::val magnetic() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->magnetic));
  }
  mjtNum density() const {
    return ptr_->density;
  }
  void set_density(mjtNum value) {
    ptr_->density = value;
  }
  mjtNum viscosity() const {
    return ptr_->viscosity;
  }
  void set_viscosity(mjtNum value) {
    ptr_->viscosity = value;
  }
  mjtNum o_margin() const {
    return ptr_->o_margin;
  }
  void set_o_margin(mjtNum value) {
    ptr_->o_margin = value;
  }
  emscripten::val o_solref() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->o_solref));
  }
  emscripten::val o_solimp() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->o_solimp));
  }
  emscripten::val o_friction() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->o_friction));
  }
  int integrator() const {
    return ptr_->integrator;
  }
  void set_integrator(int value) {
    ptr_->integrator = value;
  }
  int cone() const {
    return ptr_->cone;
  }
  void set_cone(int value) {
    ptr_->cone = value;
  }
  int jacobian() const {
    return ptr_->jacobian;
  }
  void set_jacobian(int value) {
    ptr_->jacobian = value;
  }
  int solver() const {
    return ptr_->solver;
  }
  void set_solver(int value) {
    ptr_->solver = value;
  }
  int iterations() const {
    return ptr_->iterations;
  }
  void set_iterations(int value) {
    ptr_->iterations = value;
  }
  int ls_iterations() const {
    return ptr_->ls_iterations;
  }
  void set_ls_iterations(int value) {
    ptr_->ls_iterations = value;
  }
  int noslip_iterations() const {
    return ptr_->noslip_iterations;
  }
  void set_noslip_iterations(int value) {
    ptr_->noslip_iterations = value;
  }
  int ccd_iterations() const {
    return ptr_->ccd_iterations;
  }
  void set_ccd_iterations(int value) {
    ptr_->ccd_iterations = value;
  }
  int disableflags() const {
    return ptr_->disableflags;
  }
  void set_disableflags(int value) {
    ptr_->disableflags = value;
  }
  int enableflags() const {
    return ptr_->enableflags;
  }
  void set_enableflags(int value) {
    ptr_->enableflags = value;
  }
  int disableactuator() const {
    return ptr_->disableactuator;
  }
  void set_disableactuator(int value) {
    ptr_->disableactuator = value;
  }
  int sdf_initpoints() const {
    return ptr_->sdf_initpoints;
  }
  void set_sdf_initpoints(int value) {
    ptr_->sdf_initpoints = value;
  }
  int sdf_iterations() const {
    return ptr_->sdf_iterations;
  }
  void set_sdf_iterations(int value) {
    ptr_->sdf_iterations = value;
  }

 private:
  mjOption* ptr_;
  bool owned_ = false;
};

struct MjSolverStat {
  ~MjSolverStat();
  MjSolverStat();
  explicit MjSolverStat(mjSolverStat *ptr);
  MjSolverStat(const MjSolverStat &);
  MjSolverStat &operator=(const MjSolverStat &);
  std::unique_ptr<MjSolverStat> copy();
  mjSolverStat* get() const;
  void set(mjSolverStat* ptr);
  mjtNum improvement() const {
    return ptr_->improvement;
  }
  void set_improvement(mjtNum value) {
    ptr_->improvement = value;
  }
  mjtNum gradient() const {
    return ptr_->gradient;
  }
  void set_gradient(mjtNum value) {
    ptr_->gradient = value;
  }
  mjtNum lineslope() const {
    return ptr_->lineslope;
  }
  void set_lineslope(mjtNum value) {
    ptr_->lineslope = value;
  }
  int nactive() const {
    return ptr_->nactive;
  }
  void set_nactive(int value) {
    ptr_->nactive = value;
  }
  int nchange() const {
    return ptr_->nchange;
  }
  void set_nchange(int value) {
    ptr_->nchange = value;
  }
  int neval() const {
    return ptr_->neval;
  }
  void set_neval(int value) {
    ptr_->neval = value;
  }
  int nupdate() const {
    return ptr_->nupdate;
  }
  void set_nupdate(int value) {
    ptr_->nupdate = value;
  }

 private:
  mjSolverStat* ptr_;
  bool owned_ = false;
};

struct MjStatistic {
  ~MjStatistic();
  MjStatistic();
  explicit MjStatistic(mjStatistic *ptr);
  MjStatistic(const MjStatistic &);
  MjStatistic &operator=(const MjStatistic &);
  std::unique_ptr<MjStatistic> copy();
  mjStatistic* get() const;
  void set(mjStatistic* ptr);
  mjtNum meaninertia() const {
    return ptr_->meaninertia;
  }
  void set_meaninertia(mjtNum value) {
    ptr_->meaninertia = value;
  }
  mjtNum meanmass() const {
    return ptr_->meanmass;
  }
  void set_meanmass(mjtNum value) {
    ptr_->meanmass = value;
  }
  mjtNum meansize() const {
    return ptr_->meansize;
  }
  void set_meansize(mjtNum value) {
    ptr_->meansize = value;
  }
  mjtNum extent() const {
    return ptr_->extent;
  }
  void set_extent(mjtNum value) {
    ptr_->extent = value;
  }
  emscripten::val center() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->center));
  }

 private:
  mjStatistic* ptr_;
  bool owned_ = false;
};

struct MjTimerStat {
  ~MjTimerStat();
  MjTimerStat();
  explicit MjTimerStat(mjTimerStat *ptr);
  MjTimerStat(const MjTimerStat &);
  MjTimerStat &operator=(const MjTimerStat &);
  std::unique_ptr<MjTimerStat> copy();
  mjTimerStat* get() const;
  void set(mjTimerStat* ptr);
  mjtNum duration() const {
    return ptr_->duration;
  }
  void set_duration(mjtNum value) {
    ptr_->duration = value;
  }
  int number() const {
    return ptr_->number;
  }
  void set_number(int value) {
    ptr_->number = value;
  }

 private:
  mjTimerStat* ptr_;
  bool owned_ = false;
};

struct MjVisualGlobal {
  ~MjVisualGlobal();
  MjVisualGlobal();
  explicit MjVisualGlobal(mjVisualGlobal *ptr);
  MjVisualGlobal(const MjVisualGlobal &);
  MjVisualGlobal &operator=(const MjVisualGlobal &);
  std::unique_ptr<MjVisualGlobal> copy();
  mjVisualGlobal* get() const;
  void set(mjVisualGlobal* ptr);
  int cameraid() const {
    return ptr_->cameraid;
  }
  void set_cameraid(int value) {
    ptr_->cameraid = value;
  }
  int orthographic() const {
    return ptr_->orthographic;
  }
  void set_orthographic(int value) {
    ptr_->orthographic = value;
  }
  float fovy() const {
    return ptr_->fovy;
  }
  void set_fovy(float value) {
    ptr_->fovy = value;
  }
  float ipd() const {
    return ptr_->ipd;
  }
  void set_ipd(float value) {
    ptr_->ipd = value;
  }
  float azimuth() const {
    return ptr_->azimuth;
  }
  void set_azimuth(float value) {
    ptr_->azimuth = value;
  }
  float elevation() const {
    return ptr_->elevation;
  }
  void set_elevation(float value) {
    ptr_->elevation = value;
  }
  float linewidth() const {
    return ptr_->linewidth;
  }
  void set_linewidth(float value) {
    ptr_->linewidth = value;
  }
  float glow() const {
    return ptr_->glow;
  }
  void set_glow(float value) {
    ptr_->glow = value;
  }
  float realtime() const {
    return ptr_->realtime;
  }
  void set_realtime(float value) {
    ptr_->realtime = value;
  }
  int offwidth() const {
    return ptr_->offwidth;
  }
  void set_offwidth(int value) {
    ptr_->offwidth = value;
  }
  int offheight() const {
    return ptr_->offheight;
  }
  void set_offheight(int value) {
    ptr_->offheight = value;
  }
  int ellipsoidinertia() const {
    return ptr_->ellipsoidinertia;
  }
  void set_ellipsoidinertia(int value) {
    ptr_->ellipsoidinertia = value;
  }
  int bvactive() const {
    return ptr_->bvactive;
  }
  void set_bvactive(int value) {
    ptr_->bvactive = value;
  }

 private:
  mjVisualGlobal* ptr_;
  bool owned_ = false;
};

struct MjVisualHeadlight {
  ~MjVisualHeadlight();
  MjVisualHeadlight();
  explicit MjVisualHeadlight(mjVisualHeadlight *ptr);
  MjVisualHeadlight(const MjVisualHeadlight &);
  MjVisualHeadlight &operator=(const MjVisualHeadlight &);
  std::unique_ptr<MjVisualHeadlight> copy();
  mjVisualHeadlight* get() const;
  void set(mjVisualHeadlight* ptr);
  emscripten::val ambient() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->ambient));
  }
  emscripten::val diffuse() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->diffuse));
  }
  emscripten::val specular() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->specular));
  }
  int active() const {
    return ptr_->active;
  }
  void set_active(int value) {
    ptr_->active = value;
  }

 private:
  mjVisualHeadlight* ptr_;
  bool owned_ = false;
};

struct MjVisualMap {
  ~MjVisualMap();
  MjVisualMap();
  explicit MjVisualMap(mjVisualMap *ptr);
  MjVisualMap(const MjVisualMap &);
  MjVisualMap &operator=(const MjVisualMap &);
  std::unique_ptr<MjVisualMap> copy();
  mjVisualMap* get() const;
  void set(mjVisualMap* ptr);
  float stiffness() const {
    return ptr_->stiffness;
  }
  void set_stiffness(float value) {
    ptr_->stiffness = value;
  }
  float stiffnessrot() const {
    return ptr_->stiffnessrot;
  }
  void set_stiffnessrot(float value) {
    ptr_->stiffnessrot = value;
  }
  float force() const {
    return ptr_->force;
  }
  void set_force(float value) {
    ptr_->force = value;
  }
  float torque() const {
    return ptr_->torque;
  }
  void set_torque(float value) {
    ptr_->torque = value;
  }
  float alpha() const {
    return ptr_->alpha;
  }
  void set_alpha(float value) {
    ptr_->alpha = value;
  }
  float fogstart() const {
    return ptr_->fogstart;
  }
  void set_fogstart(float value) {
    ptr_->fogstart = value;
  }
  float fogend() const {
    return ptr_->fogend;
  }
  void set_fogend(float value) {
    ptr_->fogend = value;
  }
  float znear() const {
    return ptr_->znear;
  }
  void set_znear(float value) {
    ptr_->znear = value;
  }
  float zfar() const {
    return ptr_->zfar;
  }
  void set_zfar(float value) {
    ptr_->zfar = value;
  }
  float haze() const {
    return ptr_->haze;
  }
  void set_haze(float value) {
    ptr_->haze = value;
  }
  float shadowclip() const {
    return ptr_->shadowclip;
  }
  void set_shadowclip(float value) {
    ptr_->shadowclip = value;
  }
  float shadowscale() const {
    return ptr_->shadowscale;
  }
  void set_shadowscale(float value) {
    ptr_->shadowscale = value;
  }
  float actuatortendon() const {
    return ptr_->actuatortendon;
  }
  void set_actuatortendon(float value) {
    ptr_->actuatortendon = value;
  }

 private:
  mjVisualMap* ptr_;
  bool owned_ = false;
};

struct MjVisualQuality {
  ~MjVisualQuality();
  MjVisualQuality();
  explicit MjVisualQuality(mjVisualQuality *ptr);
  MjVisualQuality(const MjVisualQuality &);
  MjVisualQuality &operator=(const MjVisualQuality &);
  std::unique_ptr<MjVisualQuality> copy();
  mjVisualQuality* get() const;
  void set(mjVisualQuality* ptr);
  int shadowsize() const {
    return ptr_->shadowsize;
  }
  void set_shadowsize(int value) {
    ptr_->shadowsize = value;
  }
  int offsamples() const {
    return ptr_->offsamples;
  }
  void set_offsamples(int value) {
    ptr_->offsamples = value;
  }
  int numslices() const {
    return ptr_->numslices;
  }
  void set_numslices(int value) {
    ptr_->numslices = value;
  }
  int numstacks() const {
    return ptr_->numstacks;
  }
  void set_numstacks(int value) {
    ptr_->numstacks = value;
  }
  int numquads() const {
    return ptr_->numquads;
  }
  void set_numquads(int value) {
    ptr_->numquads = value;
  }

 private:
  mjVisualQuality* ptr_;
  bool owned_ = false;
};

struct MjVisualRgba {
  ~MjVisualRgba();
  MjVisualRgba();
  explicit MjVisualRgba(mjVisualRgba *ptr);
  MjVisualRgba(const MjVisualRgba &);
  MjVisualRgba &operator=(const MjVisualRgba &);
  std::unique_ptr<MjVisualRgba> copy();
  mjVisualRgba* get() const;
  void set(mjVisualRgba* ptr);
  emscripten::val fog() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->fog));
  }
  emscripten::val haze() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->haze));
  }
  emscripten::val force() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->force));
  }
  emscripten::val inertia() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->inertia));
  }
  emscripten::val joint() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->joint));
  }
  emscripten::val actuator() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->actuator));
  }
  emscripten::val actuatornegative() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->actuatornegative));
  }
  emscripten::val actuatorpositive() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->actuatorpositive));
  }
  emscripten::val com() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->com));
  }
  emscripten::val camera() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->camera));
  }
  emscripten::val light() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->light));
  }
  emscripten::val selectpoint() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->selectpoint));
  }
  emscripten::val connect() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->connect));
  }
  emscripten::val contactpoint() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->contactpoint));
  }
  emscripten::val contactforce() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->contactforce));
  }
  emscripten::val contactfriction() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->contactfriction));
  }
  emscripten::val contacttorque() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->contacttorque));
  }
  emscripten::val contactgap() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->contactgap));
  }
  emscripten::val rangefinder() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rangefinder));
  }
  emscripten::val constraint() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->constraint));
  }
  emscripten::val slidercrank() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->slidercrank));
  }
  emscripten::val crankbroken() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->crankbroken));
  }
  emscripten::val frustum() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->frustum));
  }
  emscripten::val bv() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->bv));
  }
  emscripten::val bvactive() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->bvactive));
  }

 private:
  mjVisualRgba* ptr_;
  bool owned_ = false;
};

struct MjVisualScale {
  ~MjVisualScale();
  MjVisualScale();
  explicit MjVisualScale(mjVisualScale *ptr);
  MjVisualScale(const MjVisualScale &);
  MjVisualScale &operator=(const MjVisualScale &);
  std::unique_ptr<MjVisualScale> copy();
  mjVisualScale* get() const;
  void set(mjVisualScale* ptr);
  float forcewidth() const {
    return ptr_->forcewidth;
  }
  void set_forcewidth(float value) {
    ptr_->forcewidth = value;
  }
  float contactwidth() const {
    return ptr_->contactwidth;
  }
  void set_contactwidth(float value) {
    ptr_->contactwidth = value;
  }
  float contactheight() const {
    return ptr_->contactheight;
  }
  void set_contactheight(float value) {
    ptr_->contactheight = value;
  }
  float connect() const {
    return ptr_->connect;
  }
  void set_connect(float value) {
    ptr_->connect = value;
  }
  float com() const {
    return ptr_->com;
  }
  void set_com(float value) {
    ptr_->com = value;
  }
  float camera() const {
    return ptr_->camera;
  }
  void set_camera(float value) {
    ptr_->camera = value;
  }
  float light() const {
    return ptr_->light;
  }
  void set_light(float value) {
    ptr_->light = value;
  }
  float selectpoint() const {
    return ptr_->selectpoint;
  }
  void set_selectpoint(float value) {
    ptr_->selectpoint = value;
  }
  float jointlength() const {
    return ptr_->jointlength;
  }
  void set_jointlength(float value) {
    ptr_->jointlength = value;
  }
  float jointwidth() const {
    return ptr_->jointwidth;
  }
  void set_jointwidth(float value) {
    ptr_->jointwidth = value;
  }
  float actuatorlength() const {
    return ptr_->actuatorlength;
  }
  void set_actuatorlength(float value) {
    ptr_->actuatorlength = value;
  }
  float actuatorwidth() const {
    return ptr_->actuatorwidth;
  }
  void set_actuatorwidth(float value) {
    ptr_->actuatorwidth = value;
  }
  float framelength() const {
    return ptr_->framelength;
  }
  void set_framelength(float value) {
    ptr_->framelength = value;
  }
  float framewidth() const {
    return ptr_->framewidth;
  }
  void set_framewidth(float value) {
    ptr_->framewidth = value;
  }
  float constraint() const {
    return ptr_->constraint;
  }
  void set_constraint(float value) {
    ptr_->constraint = value;
  }
  float slidercrank() const {
    return ptr_->slidercrank;
  }
  void set_slidercrank(float value) {
    ptr_->slidercrank = value;
  }
  float frustum() const {
    return ptr_->frustum;
  }
  void set_frustum(float value) {
    ptr_->frustum = value;
  }

 private:
  mjVisualScale* ptr_;
  bool owned_ = false;
};

struct MjWarningStat {
  ~MjWarningStat();
  MjWarningStat();
  explicit MjWarningStat(mjWarningStat *ptr);
  MjWarningStat(const MjWarningStat &);
  MjWarningStat &operator=(const MjWarningStat &);
  std::unique_ptr<MjWarningStat> copy();
  mjWarningStat* get() const;
  void set(mjWarningStat* ptr);
  int lastinfo() const {
    return ptr_->lastinfo;
  }
  void set_lastinfo(int value) {
    ptr_->lastinfo = value;
  }
  int number() const {
    return ptr_->number;
  }
  void set_number(int value) {
    ptr_->number = value;
  }

 private:
  mjWarningStat* ptr_;
  bool owned_ = false;
};

struct MjsElement {
  explicit MjsElement(mjsElement *ptr);
  mjsElement* get() const;
  void set(mjsElement* ptr);
  mjtObj elemtype() const {
    return ptr_->elemtype;
  }
  void set_elemtype(mjtObj value) {
    ptr_->elemtype = value;
  }
  uint64_t signature() const {
    return ptr_->signature;
  }
  void set_signature(uint64_t value) {
    ptr_->signature = value;
  }

 private:
  mjsElement* ptr_;
};

struct MjsOrientation {
  explicit MjsOrientation(mjsOrientation *ptr);
  mjsOrientation* get() const;
  void set(mjsOrientation* ptr);
  mjtOrientation type() const {
    return ptr_->type;
  }
  void set_type(mjtOrientation value) {
    ptr_->type = value;
  }
  emscripten::val axisangle() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->axisangle));
  }
  emscripten::val xyaxes() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->xyaxes));
  }
  emscripten::val zaxis() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->zaxis));
  }
  emscripten::val euler() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->euler));
  }

 private:
  mjsOrientation* ptr_;
};

struct MjvCamera {
  ~MjvCamera();
  MjvCamera();
  explicit MjvCamera(mjvCamera *ptr);
  MjvCamera(const MjvCamera &);
  MjvCamera &operator=(const MjvCamera &);
  std::unique_ptr<MjvCamera> copy();
  mjvCamera* get() const;
  void set(mjvCamera* ptr);
  int type() const {
    return ptr_->type;
  }
  void set_type(int value) {
    ptr_->type = value;
  }
  int fixedcamid() const {
    return ptr_->fixedcamid;
  }
  void set_fixedcamid(int value) {
    ptr_->fixedcamid = value;
  }
  int trackbodyid() const {
    return ptr_->trackbodyid;
  }
  void set_trackbodyid(int value) {
    ptr_->trackbodyid = value;
  }
  emscripten::val lookat() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->lookat));
  }
  mjtNum distance() const {
    return ptr_->distance;
  }
  void set_distance(mjtNum value) {
    ptr_->distance = value;
  }
  mjtNum azimuth() const {
    return ptr_->azimuth;
  }
  void set_azimuth(mjtNum value) {
    ptr_->azimuth = value;
  }
  mjtNum elevation() const {
    return ptr_->elevation;
  }
  void set_elevation(mjtNum value) {
    ptr_->elevation = value;
  }
  int orthographic() const {
    return ptr_->orthographic;
  }
  void set_orthographic(int value) {
    ptr_->orthographic = value;
  }

 private:
  mjvCamera* ptr_;
  bool owned_ = false;
};

struct MjvFigure {
  ~MjvFigure();
  MjvFigure();
  explicit MjvFigure(mjvFigure *ptr);
  MjvFigure(const MjvFigure &);
  MjvFigure &operator=(const MjvFigure &);
  std::unique_ptr<MjvFigure> copy();
  mjvFigure* get() const;
  void set(mjvFigure* ptr);
  int flg_legend() const {
    return ptr_->flg_legend;
  }
  void set_flg_legend(int value) {
    ptr_->flg_legend = value;
  }
  emscripten::val flg_ticklabel() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->flg_ticklabel));
  }
  int flg_extend() const {
    return ptr_->flg_extend;
  }
  void set_flg_extend(int value) {
    ptr_->flg_extend = value;
  }
  int flg_barplot() const {
    return ptr_->flg_barplot;
  }
  void set_flg_barplot(int value) {
    ptr_->flg_barplot = value;
  }
  int flg_selection() const {
    return ptr_->flg_selection;
  }
  void set_flg_selection(int value) {
    ptr_->flg_selection = value;
  }
  int flg_symmetric() const {
    return ptr_->flg_symmetric;
  }
  void set_flg_symmetric(int value) {
    ptr_->flg_symmetric = value;
  }
  float linewidth() const {
    return ptr_->linewidth;
  }
  void set_linewidth(float value) {
    ptr_->linewidth = value;
  }
  float gridwidth() const {
    return ptr_->gridwidth;
  }
  void set_gridwidth(float value) {
    ptr_->gridwidth = value;
  }
  emscripten::val gridsize() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->gridsize));
  }
  emscripten::val gridrgb() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->gridrgb));
  }
  emscripten::val figurergba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->figurergba));
  }
  emscripten::val panergba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->panergba));
  }
  emscripten::val legendrgba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->legendrgba));
  }
  emscripten::val textrgb() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->textrgb));
  }
  emscripten::val linergb() const {
    return emscripten::val(emscripten::typed_memory_view(300, reinterpret_cast<float*>(ptr_->linergb)));
  }
  emscripten::val range() const {
    return emscripten::val(emscripten::typed_memory_view(4, reinterpret_cast<float*>(ptr_->range)));
  }
  emscripten::val xformat() const {
    return emscripten::val(emscripten::typed_memory_view(20, ptr_->xformat));
  }
  emscripten::val yformat() const {
    return emscripten::val(emscripten::typed_memory_view(20, ptr_->yformat));
  }
  emscripten::val minwidth() const {
    return emscripten::val(emscripten::typed_memory_view(20, ptr_->minwidth));
  }
  emscripten::val title() const {
    return emscripten::val(emscripten::typed_memory_view(1000, ptr_->title));
  }
  emscripten::val xlabel() const {
    return emscripten::val(emscripten::typed_memory_view(100, ptr_->xlabel));
  }
  emscripten::val linename() const {
    return emscripten::val(emscripten::typed_memory_view(10000, reinterpret_cast<char*>(ptr_->linename)));
  }
  int legendoffset() const {
    return ptr_->legendoffset;
  }
  void set_legendoffset(int value) {
    ptr_->legendoffset = value;
  }
  int subplot() const {
    return ptr_->subplot;
  }
  void set_subplot(int value) {
    ptr_->subplot = value;
  }
  emscripten::val highlight() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->highlight));
  }
  int highlightid() const {
    return ptr_->highlightid;
  }
  void set_highlightid(int value) {
    ptr_->highlightid = value;
  }
  float selection() const {
    return ptr_->selection;
  }
  void set_selection(float value) {
    ptr_->selection = value;
  }
  emscripten::val linepnt() const {
    return emscripten::val(emscripten::typed_memory_view(100, ptr_->linepnt));
  }
  emscripten::val linedata() const {
    return emscripten::val(emscripten::typed_memory_view(200200, reinterpret_cast<float*>(ptr_->linedata)));
  }
  emscripten::val xaxispixel() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->xaxispixel));
  }
  emscripten::val yaxispixel() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->yaxispixel));
  }
  emscripten::val xaxisdata() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->xaxisdata));
  }
  emscripten::val yaxisdata() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->yaxisdata));
  }

 private:
  mjvFigure* ptr_;
  bool owned_ = false;
};

struct MjvGLCamera {
  ~MjvGLCamera();
  MjvGLCamera();
  explicit MjvGLCamera(mjvGLCamera *ptr);
  MjvGLCamera(const MjvGLCamera &);
  MjvGLCamera &operator=(const MjvGLCamera &);
  std::unique_ptr<MjvGLCamera> copy();
  mjvGLCamera* get() const;
  void set(mjvGLCamera* ptr);
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val forward() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->forward));
  }
  emscripten::val up() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->up));
  }
  float frustum_center() const {
    return ptr_->frustum_center;
  }
  void set_frustum_center(float value) {
    ptr_->frustum_center = value;
  }
  float frustum_width() const {
    return ptr_->frustum_width;
  }
  void set_frustum_width(float value) {
    ptr_->frustum_width = value;
  }
  float frustum_bottom() const {
    return ptr_->frustum_bottom;
  }
  void set_frustum_bottom(float value) {
    ptr_->frustum_bottom = value;
  }
  float frustum_top() const {
    return ptr_->frustum_top;
  }
  void set_frustum_top(float value) {
    ptr_->frustum_top = value;
  }
  float frustum_near() const {
    return ptr_->frustum_near;
  }
  void set_frustum_near(float value) {
    ptr_->frustum_near = value;
  }
  float frustum_far() const {
    return ptr_->frustum_far;
  }
  void set_frustum_far(float value) {
    ptr_->frustum_far = value;
  }
  int orthographic() const {
    return ptr_->orthographic;
  }
  void set_orthographic(int value) {
    ptr_->orthographic = value;
  }

 private:
  mjvGLCamera* ptr_;
  bool owned_ = false;
};

struct MjvGeom {
  ~MjvGeom();
  MjvGeom();
  explicit MjvGeom(mjvGeom *ptr);
  MjvGeom(const MjvGeom &);
  MjvGeom &operator=(const MjvGeom &);
  std::unique_ptr<MjvGeom> copy();
  mjvGeom* get() const;
  void set(mjvGeom* ptr);
  int type() const {
    return ptr_->type;
  }
  void set_type(int value) {
    ptr_->type = value;
  }
  int dataid() const {
    return ptr_->dataid;
  }
  void set_dataid(int value) {
    ptr_->dataid = value;
  }
  int objtype() const {
    return ptr_->objtype;
  }
  void set_objtype(int value) {
    ptr_->objtype = value;
  }
  int objid() const {
    return ptr_->objid;
  }
  void set_objid(int value) {
    ptr_->objid = value;
  }
  int category() const {
    return ptr_->category;
  }
  void set_category(int value) {
    ptr_->category = value;
  }
  int matid() const {
    return ptr_->matid;
  }
  void set_matid(int value) {
    ptr_->matid = value;
  }
  int texcoord() const {
    return ptr_->texcoord;
  }
  void set_texcoord(int value) {
    ptr_->texcoord = value;
  }
  int segid() const {
    return ptr_->segid;
  }
  void set_segid(int value) {
    ptr_->segid = value;
  }
  emscripten::val size() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->size));
  }
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val mat() const {
    return emscripten::val(emscripten::typed_memory_view(9, ptr_->mat));
  }
  emscripten::val rgba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rgba));
  }
  float emission() const {
    return ptr_->emission;
  }
  void set_emission(float value) {
    ptr_->emission = value;
  }
  float specular() const {
    return ptr_->specular;
  }
  void set_specular(float value) {
    ptr_->specular = value;
  }
  float shininess() const {
    return ptr_->shininess;
  }
  void set_shininess(float value) {
    ptr_->shininess = value;
  }
  float reflectance() const {
    return ptr_->reflectance;
  }
  void set_reflectance(float value) {
    ptr_->reflectance = value;
  }
  emscripten::val label() const {
    return emscripten::val(emscripten::typed_memory_view(100, ptr_->label));
  }
  float camdist() const {
    return ptr_->camdist;
  }
  void set_camdist(float value) {
    ptr_->camdist = value;
  }
  float modelrbound() const {
    return ptr_->modelrbound;
  }
  void set_modelrbound(float value) {
    ptr_->modelrbound = value;
  }
  mjtByte transparent() const {
    return ptr_->transparent;
  }
  void set_transparent(mjtByte value) {
    ptr_->transparent = value;
  }

 private:
  mjvGeom* ptr_;
  bool owned_ = false;
};

struct MjvLight {
  ~MjvLight();
  MjvLight();
  explicit MjvLight(mjvLight *ptr);
  MjvLight(const MjvLight &);
  MjvLight &operator=(const MjvLight &);
  std::unique_ptr<MjvLight> copy();
  mjvLight* get() const;
  void set(mjvLight* ptr);
  int id() const {
    return ptr_->id;
  }
  void set_id(int value) {
    ptr_->id = value;
  }
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val dir() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->dir));
  }
  int type() const {
    return ptr_->type;
  }
  void set_type(int value) {
    ptr_->type = value;
  }
  int texid() const {
    return ptr_->texid;
  }
  void set_texid(int value) {
    ptr_->texid = value;
  }
  emscripten::val attenuation() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->attenuation));
  }
  float cutoff() const {
    return ptr_->cutoff;
  }
  void set_cutoff(float value) {
    ptr_->cutoff = value;
  }
  float exponent() const {
    return ptr_->exponent;
  }
  void set_exponent(float value) {
    ptr_->exponent = value;
  }
  emscripten::val ambient() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->ambient));
  }
  emscripten::val diffuse() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->diffuse));
  }
  emscripten::val specular() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->specular));
  }
  mjtByte headlight() const {
    return ptr_->headlight;
  }
  void set_headlight(mjtByte value) {
    ptr_->headlight = value;
  }
  mjtByte castshadow() const {
    return ptr_->castshadow;
  }
  void set_castshadow(mjtByte value) {
    ptr_->castshadow = value;
  }
  float bulbradius() const {
    return ptr_->bulbradius;
  }
  void set_bulbradius(float value) {
    ptr_->bulbradius = value;
  }
  float intensity() const {
    return ptr_->intensity;
  }
  void set_intensity(float value) {
    ptr_->intensity = value;
  }
  float range() const {
    return ptr_->range;
  }
  void set_range(float value) {
    ptr_->range = value;
  }

 private:
  mjvLight* ptr_;
  bool owned_ = false;
};

struct MjvOption {
  ~MjvOption();
  MjvOption();
  explicit MjvOption(mjvOption *ptr);
  MjvOption(const MjvOption &);
  MjvOption &operator=(const MjvOption &);
  std::unique_ptr<MjvOption> copy();
  mjvOption* get() const;
  void set(mjvOption* ptr);
  int label() const {
    return ptr_->label;
  }
  void set_label(int value) {
    ptr_->label = value;
  }
  int frame() const {
    return ptr_->frame;
  }
  void set_frame(int value) {
    ptr_->frame = value;
  }
  emscripten::val geomgroup() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->geomgroup));
  }
  emscripten::val sitegroup() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->sitegroup));
  }
  emscripten::val jointgroup() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->jointgroup));
  }
  emscripten::val tendongroup() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->tendongroup));
  }
  emscripten::val actuatorgroup() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->actuatorgroup));
  }
  emscripten::val flexgroup() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->flexgroup));
  }
  emscripten::val skingroup() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->skingroup));
  }
  emscripten::val flags() const {
    return emscripten::val(emscripten::typed_memory_view(31, ptr_->flags));
  }
  int bvh_depth() const {
    return ptr_->bvh_depth;
  }
  void set_bvh_depth(int value) {
    ptr_->bvh_depth = value;
  }
  int flex_layer() const {
    return ptr_->flex_layer;
  }
  void set_flex_layer(int value) {
    ptr_->flex_layer = value;
  }

 private:
  mjvOption* ptr_;
  bool owned_ = false;
};

struct MjvPerturb {
  ~MjvPerturb();
  MjvPerturb();
  explicit MjvPerturb(mjvPerturb *ptr);
  MjvPerturb(const MjvPerturb &);
  MjvPerturb &operator=(const MjvPerturb &);
  std::unique_ptr<MjvPerturb> copy();
  mjvPerturb* get() const;
  void set(mjvPerturb* ptr);
  int select() const {
    return ptr_->select;
  }
  void set_select(int value) {
    ptr_->select = value;
  }
  int flexselect() const {
    return ptr_->flexselect;
  }
  void set_flexselect(int value) {
    ptr_->flexselect = value;
  }
  int skinselect() const {
    return ptr_->skinselect;
  }
  void set_skinselect(int value) {
    ptr_->skinselect = value;
  }
  int active() const {
    return ptr_->active;
  }
  void set_active(int value) {
    ptr_->active = value;
  }
  int active2() const {
    return ptr_->active2;
  }
  void set_active2(int value) {
    ptr_->active2 = value;
  }
  emscripten::val refpos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->refpos));
  }
  emscripten::val refquat() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->refquat));
  }
  emscripten::val refselpos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->refselpos));
  }
  emscripten::val localpos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->localpos));
  }
  mjtNum localmass() const {
    return ptr_->localmass;
  }
  void set_localmass(mjtNum value) {
    ptr_->localmass = value;
  }
  mjtNum scale() const {
    return ptr_->scale;
  }
  void set_scale(mjtNum value) {
    ptr_->scale = value;
  }

 private:
  mjvPerturb* ptr_;
  bool owned_ = false;
};

struct MjsCompiler {
  explicit MjsCompiler(mjsCompiler *ptr);
  mjsCompiler* get() const;
  void set(mjsCompiler* ptr);
  mjtByte autolimits() const {
    return ptr_->autolimits;
  }
  void set_autolimits(mjtByte value) {
    ptr_->autolimits = value;
  }
  double boundmass() const {
    return ptr_->boundmass;
  }
  void set_boundmass(double value) {
    ptr_->boundmass = value;
  }
  double boundinertia() const {
    return ptr_->boundinertia;
  }
  void set_boundinertia(double value) {
    ptr_->boundinertia = value;
  }
  double settotalmass() const {
    return ptr_->settotalmass;
  }
  void set_settotalmass(double value) {
    ptr_->settotalmass = value;
  }
  mjtByte balanceinertia() const {
    return ptr_->balanceinertia;
  }
  void set_balanceinertia(mjtByte value) {
    ptr_->balanceinertia = value;
  }
  mjtByte fitaabb() const {
    return ptr_->fitaabb;
  }
  void set_fitaabb(mjtByte value) {
    ptr_->fitaabb = value;
  }
  mjtByte degree() const {
    return ptr_->degree;
  }
  void set_degree(mjtByte value) {
    ptr_->degree = value;
  }
  emscripten::val eulerseq() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->eulerseq));
  }
  mjtByte discardvisual() const {
    return ptr_->discardvisual;
  }
  void set_discardvisual(mjtByte value) {
    ptr_->discardvisual = value;
  }
  mjtByte usethread() const {
    return ptr_->usethread;
  }
  void set_usethread(mjtByte value) {
    ptr_->usethread = value;
  }
  mjtByte fusestatic() const {
    return ptr_->fusestatic;
  }
  void set_fusestatic(mjtByte value) {
    ptr_->fusestatic = value;
  }
  int inertiafromgeom() const {
    return ptr_->inertiafromgeom;
  }
  void set_inertiafromgeom(int value) {
    ptr_->inertiafromgeom = value;
  }
  emscripten::val inertiagrouprange() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->inertiagrouprange));
  }
  mjtByte saveinertial() const {
    return ptr_->saveinertial;
  }
  void set_saveinertial(mjtByte value) {
    ptr_->saveinertial = value;
  }
  int alignfree() const {
    return ptr_->alignfree;
  }
  void set_alignfree(int value) {
    ptr_->alignfree = value;
  }
  mjString meshdir() const {
    return (ptr_ && ptr_->meshdir) ? *(ptr_->meshdir) : "";
  }
  void set_meshdir(const mjString& value) {
    if (ptr_ && ptr_->meshdir) {
      *(ptr_->meshdir) = value;
    }
  }
  mjString texturedir() const {
    return (ptr_ && ptr_->texturedir) ? *(ptr_->texturedir) : "";
  }
  void set_texturedir(const mjString& value) {
    if (ptr_ && ptr_->texturedir) {
      *(ptr_->texturedir) = value;
    }
  }

 private:
  mjsCompiler* ptr_;

 public:
  MjLROpt LRopt;
};

struct MjVisual {
  ~MjVisual();
  MjVisual();
  explicit MjVisual(mjVisual *ptr);
  MjVisual(const MjVisual &);
  MjVisual &operator=(const MjVisual &);
  std::unique_ptr<MjVisual> copy();
  mjVisual* get() const;
  void set(mjVisual* ptr);

 private:
  mjVisual* ptr_;
  bool owned_ = false;

 public:
  MjVisualGlobal global;
  MjVisualQuality quality;
  MjVisualHeadlight headlight;
  MjVisualMap map;
  MjVisualScale scale;
  MjVisualRgba rgba;
};

struct MjsEquality {
  explicit MjsEquality(mjsEquality *ptr);
  mjsEquality* get() const;
  void set(mjsEquality* ptr);
  mjtEq type() const {
    return ptr_->type;
  }
  void set_type(mjtEq value) {
    ptr_->type = value;
  }
  emscripten::val data() const {
    return emscripten::val(emscripten::typed_memory_view(11, ptr_->data));
  }
  mjtByte active() const {
    return ptr_->active;
  }
  void set_active(mjtByte value) {
    ptr_->active = value;
  }
  mjString name1() const {
    return (ptr_ && ptr_->name1) ? *(ptr_->name1) : "";
  }
  void set_name1(const mjString& value) {
    if (ptr_ && ptr_->name1) {
      *(ptr_->name1) = value;
    }
  }
  mjString name2() const {
    return (ptr_ && ptr_->name2) ? *(ptr_->name2) : "";
  }
  void set_name2(const mjString& value) {
    if (ptr_ && ptr_->name2) {
      *(ptr_->name2) = value;
    }
  }
  mjtObj objtype() const {
    return ptr_->objtype;
  }
  void set_objtype(mjtObj value) {
    ptr_->objtype = value;
  }
  emscripten::val solref() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solref));
  }
  emscripten::val solimp() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->solimp));
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsEquality* ptr_;

 public:
  MjsElement element;
};

struct MjsExclude {
  explicit MjsExclude(mjsExclude *ptr);
  mjsExclude* get() const;
  void set(mjsExclude* ptr);
  mjString bodyname1() const {
    return (ptr_ && ptr_->bodyname1) ? *(ptr_->bodyname1) : "";
  }
  void set_bodyname1(const mjString& value) {
    if (ptr_ && ptr_->bodyname1) {
      *(ptr_->bodyname1) = value;
    }
  }
  mjString bodyname2() const {
    return (ptr_ && ptr_->bodyname2) ? *(ptr_->bodyname2) : "";
  }
  void set_bodyname2(const mjString& value) {
    if (ptr_ && ptr_->bodyname2) {
      *(ptr_->bodyname2) = value;
    }
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsExclude* ptr_;

 public:
  MjsElement element;
};

struct MjsFlex {
  explicit MjsFlex(mjsFlex *ptr);
  mjsFlex* get() const;
  void set(mjsFlex* ptr);
  int contype() const {
    return ptr_->contype;
  }
  void set_contype(int value) {
    ptr_->contype = value;
  }
  int conaffinity() const {
    return ptr_->conaffinity;
  }
  void set_conaffinity(int value) {
    ptr_->conaffinity = value;
  }
  int condim() const {
    return ptr_->condim;
  }
  void set_condim(int value) {
    ptr_->condim = value;
  }
  int priority() const {
    return ptr_->priority;
  }
  void set_priority(int value) {
    ptr_->priority = value;
  }
  emscripten::val friction() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->friction));
  }
  double solmix() const {
    return ptr_->solmix;
  }
  void set_solmix(double value) {
    ptr_->solmix = value;
  }
  emscripten::val solref() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solref));
  }
  emscripten::val solimp() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->solimp));
  }
  double margin() const {
    return ptr_->margin;
  }
  void set_margin(double value) {
    ptr_->margin = value;
  }
  double gap() const {
    return ptr_->gap;
  }
  void set_gap(double value) {
    ptr_->gap = value;
  }
  int dim() const {
    return ptr_->dim;
  }
  void set_dim(int value) {
    ptr_->dim = value;
  }
  double radius() const {
    return ptr_->radius;
  }
  void set_radius(double value) {
    ptr_->radius = value;
  }
  mjtByte internal() const {
    return ptr_->internal;
  }
  void set_internal(mjtByte value) {
    ptr_->internal = value;
  }
  mjtByte flatskin() const {
    return ptr_->flatskin;
  }
  void set_flatskin(mjtByte value) {
    ptr_->flatskin = value;
  }
  int selfcollide() const {
    return ptr_->selfcollide;
  }
  void set_selfcollide(int value) {
    ptr_->selfcollide = value;
  }
  int vertcollide() const {
    return ptr_->vertcollide;
  }
  void set_vertcollide(int value) {
    ptr_->vertcollide = value;
  }
  int passive() const {
    return ptr_->passive;
  }
  void set_passive(int value) {
    ptr_->passive = value;
  }
  int activelayers() const {
    return ptr_->activelayers;
  }
  void set_activelayers(int value) {
    ptr_->activelayers = value;
  }
  int group() const {
    return ptr_->group;
  }
  void set_group(int value) {
    ptr_->group = value;
  }
  double edgestiffness() const {
    return ptr_->edgestiffness;
  }
  void set_edgestiffness(double value) {
    ptr_->edgestiffness = value;
  }
  double edgedamping() const {
    return ptr_->edgedamping;
  }
  void set_edgedamping(double value) {
    ptr_->edgedamping = value;
  }
  emscripten::val rgba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rgba));
  }
  mjString material() const {
    return (ptr_ && ptr_->material) ? *(ptr_->material) : "";
  }
  void set_material(const mjString& value) {
    if (ptr_ && ptr_->material) {
      *(ptr_->material) = value;
    }
  }
  double young() const {
    return ptr_->young;
  }
  void set_young(double value) {
    ptr_->young = value;
  }
  double poisson() const {
    return ptr_->poisson;
  }
  void set_poisson(double value) {
    ptr_->poisson = value;
  }
  double damping() const {
    return ptr_->damping;
  }
  void set_damping(double value) {
    ptr_->damping = value;
  }
  double thickness() const {
    return ptr_->thickness;
  }
  void set_thickness(double value) {
    ptr_->thickness = value;
  }
  int elastic2d() const {
    return ptr_->elastic2d;
  }
  void set_elastic2d(int value) {
    ptr_->elastic2d = value;
  }
  mjStringVec &nodebody() const {
    return *(ptr_->nodebody);
  }
  mjStringVec &vertbody() const {
    return *(ptr_->vertbody);
  }
  mjDoubleVec &node() const {
    return *(ptr_->node);
  }
  mjDoubleVec &vert() const {
    return *(ptr_->vert);
  }
  mjIntVec &elem() const {
    return *(ptr_->elem);
  }
  mjFloatVec &texcoord() const {
    return *(ptr_->texcoord);
  }
  mjIntVec &elemtexcoord() const {
    return *(ptr_->elemtexcoord);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsFlex* ptr_;

 public:
  MjsElement element;
};

struct MjsHField {
  explicit MjsHField(mjsHField *ptr);
  mjsHField* get() const;
  void set(mjsHField* ptr);
  mjString content_type() const {
    return (ptr_ && ptr_->content_type) ? *(ptr_->content_type) : "";
  }
  void set_content_type(const mjString& value) {
    if (ptr_ && ptr_->content_type) {
      *(ptr_->content_type) = value;
    }
  }
  mjString file() const {
    return (ptr_ && ptr_->file) ? *(ptr_->file) : "";
  }
  void set_file(const mjString& value) {
    if (ptr_ && ptr_->file) {
      *(ptr_->file) = value;
    }
  }
  emscripten::val size() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->size));
  }
  int nrow() const {
    return ptr_->nrow;
  }
  void set_nrow(int value) {
    ptr_->nrow = value;
  }
  int ncol() const {
    return ptr_->ncol;
  }
  void set_ncol(int value) {
    ptr_->ncol = value;
  }
  mjFloatVec &userdata() const {
    return *(ptr_->userdata);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsHField* ptr_;

 public:
  MjsElement element;
};

struct MjsJoint {
  explicit MjsJoint(mjsJoint *ptr);
  mjsJoint* get() const;
  void set(mjsJoint* ptr);
  mjtJoint type() const {
    return ptr_->type;
  }
  void set_type(mjtJoint value) {
    ptr_->type = value;
  }
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val axis() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->axis));
  }
  double ref() const {
    return ptr_->ref;
  }
  void set_ref(double value) {
    ptr_->ref = value;
  }
  int align() const {
    return ptr_->align;
  }
  void set_align(int value) {
    ptr_->align = value;
  }
  double stiffness() const {
    return ptr_->stiffness;
  }
  void set_stiffness(double value) {
    ptr_->stiffness = value;
  }
  double springref() const {
    return ptr_->springref;
  }
  void set_springref(double value) {
    ptr_->springref = value;
  }
  emscripten::val springdamper() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->springdamper));
  }
  int limited() const {
    return ptr_->limited;
  }
  void set_limited(int value) {
    ptr_->limited = value;
  }
  emscripten::val range() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->range));
  }
  double margin() const {
    return ptr_->margin;
  }
  void set_margin(double value) {
    ptr_->margin = value;
  }
  emscripten::val solref_limit() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solref_limit));
  }
  emscripten::val solimp_limit() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->solimp_limit));
  }
  int actfrclimited() const {
    return ptr_->actfrclimited;
  }
  void set_actfrclimited(int value) {
    ptr_->actfrclimited = value;
  }
  emscripten::val actfrcrange() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->actfrcrange));
  }
  double armature() const {
    return ptr_->armature;
  }
  void set_armature(double value) {
    ptr_->armature = value;
  }
  double damping() const {
    return ptr_->damping;
  }
  void set_damping(double value) {
    ptr_->damping = value;
  }
  double frictionloss() const {
    return ptr_->frictionloss;
  }
  void set_frictionloss(double value) {
    ptr_->frictionloss = value;
  }
  emscripten::val solref_friction() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solref_friction));
  }
  emscripten::val solimp_friction() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->solimp_friction));
  }
  int group() const {
    return ptr_->group;
  }
  void set_group(int value) {
    ptr_->group = value;
  }
  mjtByte actgravcomp() const {
    return ptr_->actgravcomp;
  }
  void set_actgravcomp(mjtByte value) {
    ptr_->actgravcomp = value;
  }
  mjDoubleVec &userdata() const {
    return *(ptr_->userdata);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsJoint* ptr_;

 public:
  MjsElement element;
};

struct MjsKey {
  explicit MjsKey(mjsKey *ptr);
  mjsKey* get() const;
  void set(mjsKey* ptr);
  double time() const {
    return ptr_->time;
  }
  void set_time(double value) {
    ptr_->time = value;
  }
  mjDoubleVec &qpos() const {
    return *(ptr_->qpos);
  }
  mjDoubleVec &qvel() const {
    return *(ptr_->qvel);
  }
  mjDoubleVec &act() const {
    return *(ptr_->act);
  }
  mjDoubleVec &mpos() const {
    return *(ptr_->mpos);
  }
  mjDoubleVec &mquat() const {
    return *(ptr_->mquat);
  }
  mjDoubleVec &ctrl() const {
    return *(ptr_->ctrl);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsKey* ptr_;

 public:
  MjsElement element;
};

struct MjsLight {
  explicit MjsLight(mjsLight *ptr);
  mjsLight* get() const;
  void set(mjsLight* ptr);
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val dir() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->dir));
  }
  mjtCamLight mode() const {
    return ptr_->mode;
  }
  void set_mode(mjtCamLight value) {
    ptr_->mode = value;
  }
  mjString targetbody() const {
    return (ptr_ && ptr_->targetbody) ? *(ptr_->targetbody) : "";
  }
  void set_targetbody(const mjString& value) {
    if (ptr_ && ptr_->targetbody) {
      *(ptr_->targetbody) = value;
    }
  }
  mjtByte active() const {
    return ptr_->active;
  }
  void set_active(mjtByte value) {
    ptr_->active = value;
  }
  mjtLightType type() const {
    return ptr_->type;
  }
  void set_type(mjtLightType value) {
    ptr_->type = value;
  }
  mjString texture() const {
    return (ptr_ && ptr_->texture) ? *(ptr_->texture) : "";
  }
  void set_texture(const mjString& value) {
    if (ptr_ && ptr_->texture) {
      *(ptr_->texture) = value;
    }
  }
  mjtByte castshadow() const {
    return ptr_->castshadow;
  }
  void set_castshadow(mjtByte value) {
    ptr_->castshadow = value;
  }
  float bulbradius() const {
    return ptr_->bulbradius;
  }
  void set_bulbradius(float value) {
    ptr_->bulbradius = value;
  }
  float intensity() const {
    return ptr_->intensity;
  }
  void set_intensity(float value) {
    ptr_->intensity = value;
  }
  float range() const {
    return ptr_->range;
  }
  void set_range(float value) {
    ptr_->range = value;
  }
  emscripten::val attenuation() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->attenuation));
  }
  float cutoff() const {
    return ptr_->cutoff;
  }
  void set_cutoff(float value) {
    ptr_->cutoff = value;
  }
  float exponent() const {
    return ptr_->exponent;
  }
  void set_exponent(float value) {
    ptr_->exponent = value;
  }
  emscripten::val ambient() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->ambient));
  }
  emscripten::val diffuse() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->diffuse));
  }
  emscripten::val specular() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->specular));
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsLight* ptr_;

 public:
  MjsElement element;
};

struct MjsMaterial {
  explicit MjsMaterial(mjsMaterial *ptr);
  mjsMaterial* get() const;
  void set(mjsMaterial* ptr);
  mjStringVec &textures() const {
    return *(ptr_->textures);
  }
  mjtByte texuniform() const {
    return ptr_->texuniform;
  }
  void set_texuniform(mjtByte value) {
    ptr_->texuniform = value;
  }
  emscripten::val texrepeat() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->texrepeat));
  }
  float emission() const {
    return ptr_->emission;
  }
  void set_emission(float value) {
    ptr_->emission = value;
  }
  float specular() const {
    return ptr_->specular;
  }
  void set_specular(float value) {
    ptr_->specular = value;
  }
  float shininess() const {
    return ptr_->shininess;
  }
  void set_shininess(float value) {
    ptr_->shininess = value;
  }
  float reflectance() const {
    return ptr_->reflectance;
  }
  void set_reflectance(float value) {
    ptr_->reflectance = value;
  }
  float metallic() const {
    return ptr_->metallic;
  }
  void set_metallic(float value) {
    ptr_->metallic = value;
  }
  float roughness() const {
    return ptr_->roughness;
  }
  void set_roughness(float value) {
    ptr_->roughness = value;
  }
  emscripten::val rgba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rgba));
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsMaterial* ptr_;

 public:
  MjsElement element;
};

struct MjsNumeric {
  explicit MjsNumeric(mjsNumeric *ptr);
  mjsNumeric* get() const;
  void set(mjsNumeric* ptr);
  mjDoubleVec &data() const {
    return *(ptr_->data);
  }
  int size() const {
    return ptr_->size;
  }
  void set_size(int value) {
    ptr_->size = value;
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsNumeric* ptr_;

 public:
  MjsElement element;
};

struct MjsPair {
  explicit MjsPair(mjsPair *ptr);
  mjsPair* get() const;
  void set(mjsPair* ptr);
  mjString geomname1() const {
    return (ptr_ && ptr_->geomname1) ? *(ptr_->geomname1) : "";
  }
  void set_geomname1(const mjString& value) {
    if (ptr_ && ptr_->geomname1) {
      *(ptr_->geomname1) = value;
    }
  }
  mjString geomname2() const {
    return (ptr_ && ptr_->geomname2) ? *(ptr_->geomname2) : "";
  }
  void set_geomname2(const mjString& value) {
    if (ptr_ && ptr_->geomname2) {
      *(ptr_->geomname2) = value;
    }
  }
  int condim() const {
    return ptr_->condim;
  }
  void set_condim(int value) {
    ptr_->condim = value;
  }
  emscripten::val solref() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solref));
  }
  emscripten::val solreffriction() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solreffriction));
  }
  emscripten::val solimp() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->solimp));
  }
  double margin() const {
    return ptr_->margin;
  }
  void set_margin(double value) {
    ptr_->margin = value;
  }
  double gap() const {
    return ptr_->gap;
  }
  void set_gap(double value) {
    ptr_->gap = value;
  }
  emscripten::val friction() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->friction));
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsPair* ptr_;

 public:
  MjsElement element;
};

struct MjsPlugin {
  explicit MjsPlugin(mjsPlugin *ptr);
  mjsPlugin* get() const;
  void set(mjsPlugin* ptr);
  mjString name() const {
    return (ptr_ && ptr_->name) ? *(ptr_->name) : "";
  }
  void set_name(const mjString& value) {
    if (ptr_ && ptr_->name) {
      *(ptr_->name) = value;
    }
  }
  mjString plugin_name() const {
    return (ptr_ && ptr_->plugin_name) ? *(ptr_->plugin_name) : "";
  }
  void set_plugin_name(const mjString& value) {
    if (ptr_ && ptr_->plugin_name) {
      *(ptr_->plugin_name) = value;
    }
  }
  mjtByte active() const {
    return ptr_->active;
  }
  void set_active(mjtByte value) {
    ptr_->active = value;
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsPlugin* ptr_;

 public:
  MjsElement element;
};

struct MjsSkin {
  explicit MjsSkin(mjsSkin *ptr);
  mjsSkin* get() const;
  void set(mjsSkin* ptr);
  mjString file() const {
    return (ptr_ && ptr_->file) ? *(ptr_->file) : "";
  }
  void set_file(const mjString& value) {
    if (ptr_ && ptr_->file) {
      *(ptr_->file) = value;
    }
  }
  mjString material() const {
    return (ptr_ && ptr_->material) ? *(ptr_->material) : "";
  }
  void set_material(const mjString& value) {
    if (ptr_ && ptr_->material) {
      *(ptr_->material) = value;
    }
  }
  emscripten::val rgba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rgba));
  }
  float inflate() const {
    return ptr_->inflate;
  }
  void set_inflate(float value) {
    ptr_->inflate = value;
  }
  int group() const {
    return ptr_->group;
  }
  void set_group(int value) {
    ptr_->group = value;
  }
  mjFloatVec &vert() const {
    return *(ptr_->vert);
  }
  mjFloatVec &texcoord() const {
    return *(ptr_->texcoord);
  }
  mjIntVec &face() const {
    return *(ptr_->face);
  }
  mjStringVec &bodyname() const {
    return *(ptr_->bodyname);
  }
  mjFloatVec &bindpos() const {
    return *(ptr_->bindpos);
  }
  mjFloatVec &bindquat() const {
    return *(ptr_->bindquat);
  }
  mjIntVecVec &vertid() const {
    return *(ptr_->vertid);
  }
  mjFloatVecVec &vertweight() const {
    return *(ptr_->vertweight);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsSkin* ptr_;

 public:
  MjsElement element;
};

struct MjsTendon {
  explicit MjsTendon(mjsTendon *ptr);
  mjsTendon* get() const;
  void set(mjsTendon* ptr);
  double stiffness() const {
    return ptr_->stiffness;
  }
  void set_stiffness(double value) {
    ptr_->stiffness = value;
  }
  emscripten::val springlength() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->springlength));
  }
  double damping() const {
    return ptr_->damping;
  }
  void set_damping(double value) {
    ptr_->damping = value;
  }
  double frictionloss() const {
    return ptr_->frictionloss;
  }
  void set_frictionloss(double value) {
    ptr_->frictionloss = value;
  }
  emscripten::val solref_friction() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solref_friction));
  }
  emscripten::val solimp_friction() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->solimp_friction));
  }
  double armature() const {
    return ptr_->armature;
  }
  void set_armature(double value) {
    ptr_->armature = value;
  }
  int limited() const {
    return ptr_->limited;
  }
  void set_limited(int value) {
    ptr_->limited = value;
  }
  int actfrclimited() const {
    return ptr_->actfrclimited;
  }
  void set_actfrclimited(int value) {
    ptr_->actfrclimited = value;
  }
  emscripten::val range() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->range));
  }
  emscripten::val actfrcrange() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->actfrcrange));
  }
  double margin() const {
    return ptr_->margin;
  }
  void set_margin(double value) {
    ptr_->margin = value;
  }
  emscripten::val solref_limit() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solref_limit));
  }
  emscripten::val solimp_limit() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->solimp_limit));
  }
  mjString material() const {
    return (ptr_ && ptr_->material) ? *(ptr_->material) : "";
  }
  void set_material(const mjString& value) {
    if (ptr_ && ptr_->material) {
      *(ptr_->material) = value;
    }
  }
  double width() const {
    return ptr_->width;
  }
  void set_width(double value) {
    ptr_->width = value;
  }
  emscripten::val rgba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rgba));
  }
  int group() const {
    return ptr_->group;
  }
  void set_group(int value) {
    ptr_->group = value;
  }
  mjDoubleVec &userdata() const {
    return *(ptr_->userdata);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsTendon* ptr_;

 public:
  MjsElement element;
};

struct MjsText {
  explicit MjsText(mjsText *ptr);
  mjsText* get() const;
  void set(mjsText* ptr);
  mjString data() const {
    return (ptr_ && ptr_->data) ? *(ptr_->data) : "";
  }
  void set_data(const mjString& value) {
    if (ptr_ && ptr_->data) {
      *(ptr_->data) = value;
    }
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsText* ptr_;

 public:
  MjsElement element;
};

struct MjsTexture {
  explicit MjsTexture(mjsTexture *ptr);
  mjsTexture* get() const;
  void set(mjsTexture* ptr);
  mjtTexture type() const {
    return ptr_->type;
  }
  void set_type(mjtTexture value) {
    ptr_->type = value;
  }
  mjtColorSpace colorspace() const {
    return ptr_->colorspace;
  }
  void set_colorspace(mjtColorSpace value) {
    ptr_->colorspace = value;
  }
  int builtin() const {
    return ptr_->builtin;
  }
  void set_builtin(int value) {
    ptr_->builtin = value;
  }
  int mark() const {
    return ptr_->mark;
  }
  void set_mark(int value) {
    ptr_->mark = value;
  }
  emscripten::val rgb1() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->rgb1));
  }
  emscripten::val rgb2() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->rgb2));
  }
  emscripten::val markrgb() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->markrgb));
  }
  double random() const {
    return ptr_->random;
  }
  void set_random(double value) {
    ptr_->random = value;
  }
  int height() const {
    return ptr_->height;
  }
  void set_height(int value) {
    ptr_->height = value;
  }
  int width() const {
    return ptr_->width;
  }
  void set_width(int value) {
    ptr_->width = value;
  }
  int nchannel() const {
    return ptr_->nchannel;
  }
  void set_nchannel(int value) {
    ptr_->nchannel = value;
  }
  mjString content_type() const {
    return (ptr_ && ptr_->content_type) ? *(ptr_->content_type) : "";
  }
  void set_content_type(const mjString& value) {
    if (ptr_ && ptr_->content_type) {
      *(ptr_->content_type) = value;
    }
  }
  mjString file() const {
    return (ptr_ && ptr_->file) ? *(ptr_->file) : "";
  }
  void set_file(const mjString& value) {
    if (ptr_ && ptr_->file) {
      *(ptr_->file) = value;
    }
  }
  emscripten::val gridsize() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->gridsize));
  }
  emscripten::val gridlayout() const {
    return emscripten::val(emscripten::typed_memory_view(12, ptr_->gridlayout));
  }
  mjStringVec &cubefiles() const {
    return *(ptr_->cubefiles);
  }
  std::vector<uint8_t> &data() const {
    return *(reinterpret_cast<std::vector<uint8_t>*>(ptr_->data));
  }
  mjtByte hflip() const {
    return ptr_->hflip;
  }
  void set_hflip(mjtByte value) {
    ptr_->hflip = value;
  }
  mjtByte vflip() const {
    return ptr_->vflip;
  }
  void set_vflip(mjtByte value) {
    ptr_->vflip = value;
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsTexture* ptr_;

 public:
  MjsElement element;
};

struct MjsTuple {
  explicit MjsTuple(mjsTuple *ptr);
  mjsTuple* get() const;
  void set(mjsTuple* ptr);
  mjIntVec &objtype() const {
    return *(ptr_->objtype);
  }
  mjStringVec &objname() const {
    return *(ptr_->objname);
  }
  mjDoubleVec &objprm() const {
    return *(ptr_->objprm);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsTuple* ptr_;

 public:
  MjsElement element;
};

struct MjsWrap {
  explicit MjsWrap(mjsWrap *ptr);
  mjsWrap* get() const;
  void set(mjsWrap* ptr);
  mjtWrap type() const {
    return ptr_->type;
  }
  void set_type(mjtWrap value) {
    ptr_->type = value;
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsWrap* ptr_;

 public:
  MjsElement element;
};

struct MjsCamera {
  explicit MjsCamera(mjsCamera *ptr);
  mjsCamera* get() const;
  void set(mjsCamera* ptr);
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val quat() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->quat));
  }
  mjtCamLight mode() const {
    return ptr_->mode;
  }
  void set_mode(mjtCamLight value) {
    ptr_->mode = value;
  }
  mjString targetbody() const {
    return (ptr_ && ptr_->targetbody) ? *(ptr_->targetbody) : "";
  }
  void set_targetbody(const mjString& value) {
    if (ptr_ && ptr_->targetbody) {
      *(ptr_->targetbody) = value;
    }
  }
  int orthographic() const {
    return ptr_->orthographic;
  }
  void set_orthographic(int value) {
    ptr_->orthographic = value;
  }
  double fovy() const {
    return ptr_->fovy;
  }
  void set_fovy(double value) {
    ptr_->fovy = value;
  }
  double ipd() const {
    return ptr_->ipd;
  }
  void set_ipd(double value) {
    ptr_->ipd = value;
  }
  emscripten::val intrinsic() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->intrinsic));
  }
  emscripten::val sensor_size() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->sensor_size));
  }
  emscripten::val resolution() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->resolution));
  }
  emscripten::val focal_length() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->focal_length));
  }
  emscripten::val focal_pixel() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->focal_pixel));
  }
  emscripten::val principal_length() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->principal_length));
  }
  emscripten::val principal_pixel() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->principal_pixel));
  }
  mjDoubleVec &userdata() const {
    return *(ptr_->userdata);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsCamera* ptr_;

 public:
  MjsElement element;
  MjsOrientation alt;
};

struct MjsFrame {
  explicit MjsFrame(mjsFrame *ptr);
  mjsFrame* get() const;
  void set(mjsFrame* ptr);
  mjString childclass() const {
    return (ptr_ && ptr_->childclass) ? *(ptr_->childclass) : "";
  }
  void set_childclass(const mjString& value) {
    if (ptr_ && ptr_->childclass) {
      *(ptr_->childclass) = value;
    }
  }
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val quat() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->quat));
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsFrame* ptr_;

 public:
  MjsElement element;
  MjsOrientation alt;
};

struct MjsSite {
  explicit MjsSite(mjsSite *ptr);
  mjsSite* get() const;
  void set(mjsSite* ptr);
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val quat() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->quat));
  }
  emscripten::val fromto() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->fromto));
  }
  emscripten::val size() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->size));
  }
  mjtGeom type() const {
    return ptr_->type;
  }
  void set_type(mjtGeom value) {
    ptr_->type = value;
  }
  mjString material() const {
    return (ptr_ && ptr_->material) ? *(ptr_->material) : "";
  }
  void set_material(const mjString& value) {
    if (ptr_ && ptr_->material) {
      *(ptr_->material) = value;
    }
  }
  int group() const {
    return ptr_->group;
  }
  void set_group(int value) {
    ptr_->group = value;
  }
  emscripten::val rgba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rgba));
  }
  mjDoubleVec &userdata() const {
    return *(ptr_->userdata);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsSite* ptr_;

 public:
  MjsElement element;
  MjsOrientation alt;
};

struct MjModel {
  ~MjModel();
  explicit MjModel(mjModel *ptr);
  MjModel(const MjModel &);
  std::unique_ptr<MjModel> copy();
  mjModel* get() const;
  void set(mjModel* ptr);
  int nq() const {
    return ptr_->nq;
  }
  void set_nq(int value) {
    ptr_->nq = value;
  }
  int nv() const {
    return ptr_->nv;
  }
  void set_nv(int value) {
    ptr_->nv = value;
  }
  int nu() const {
    return ptr_->nu;
  }
  void set_nu(int value) {
    ptr_->nu = value;
  }
  int na() const {
    return ptr_->na;
  }
  void set_na(int value) {
    ptr_->na = value;
  }
  int nbody() const {
    return ptr_->nbody;
  }
  void set_nbody(int value) {
    ptr_->nbody = value;
  }
  int nbvh() const {
    return ptr_->nbvh;
  }
  void set_nbvh(int value) {
    ptr_->nbvh = value;
  }
  int nbvhstatic() const {
    return ptr_->nbvhstatic;
  }
  void set_nbvhstatic(int value) {
    ptr_->nbvhstatic = value;
  }
  int nbvhdynamic() const {
    return ptr_->nbvhdynamic;
  }
  void set_nbvhdynamic(int value) {
    ptr_->nbvhdynamic = value;
  }
  int noct() const {
    return ptr_->noct;
  }
  void set_noct(int value) {
    ptr_->noct = value;
  }
  int njnt() const {
    return ptr_->njnt;
  }
  void set_njnt(int value) {
    ptr_->njnt = value;
  }
  int ntree() const {
    return ptr_->ntree;
  }
  void set_ntree(int value) {
    ptr_->ntree = value;
  }
  int nM() const {
    return ptr_->nM;
  }
  void set_nM(int value) {
    ptr_->nM = value;
  }
  int nB() const {
    return ptr_->nB;
  }
  void set_nB(int value) {
    ptr_->nB = value;
  }
  int nC() const {
    return ptr_->nC;
  }
  void set_nC(int value) {
    ptr_->nC = value;
  }
  int nD() const {
    return ptr_->nD;
  }
  void set_nD(int value) {
    ptr_->nD = value;
  }
  int ngeom() const {
    return ptr_->ngeom;
  }
  void set_ngeom(int value) {
    ptr_->ngeom = value;
  }
  int nsite() const {
    return ptr_->nsite;
  }
  void set_nsite(int value) {
    ptr_->nsite = value;
  }
  int ncam() const {
    return ptr_->ncam;
  }
  void set_ncam(int value) {
    ptr_->ncam = value;
  }
  int nlight() const {
    return ptr_->nlight;
  }
  void set_nlight(int value) {
    ptr_->nlight = value;
  }
  int nflex() const {
    return ptr_->nflex;
  }
  void set_nflex(int value) {
    ptr_->nflex = value;
  }
  int nflexnode() const {
    return ptr_->nflexnode;
  }
  void set_nflexnode(int value) {
    ptr_->nflexnode = value;
  }
  int nflexvert() const {
    return ptr_->nflexvert;
  }
  void set_nflexvert(int value) {
    ptr_->nflexvert = value;
  }
  int nflexedge() const {
    return ptr_->nflexedge;
  }
  void set_nflexedge(int value) {
    ptr_->nflexedge = value;
  }
  int nflexelem() const {
    return ptr_->nflexelem;
  }
  void set_nflexelem(int value) {
    ptr_->nflexelem = value;
  }
  int nflexelemdata() const {
    return ptr_->nflexelemdata;
  }
  void set_nflexelemdata(int value) {
    ptr_->nflexelemdata = value;
  }
  int nflexelemedge() const {
    return ptr_->nflexelemedge;
  }
  void set_nflexelemedge(int value) {
    ptr_->nflexelemedge = value;
  }
  int nflexshelldata() const {
    return ptr_->nflexshelldata;
  }
  void set_nflexshelldata(int value) {
    ptr_->nflexshelldata = value;
  }
  int nflexevpair() const {
    return ptr_->nflexevpair;
  }
  void set_nflexevpair(int value) {
    ptr_->nflexevpair = value;
  }
  int nflextexcoord() const {
    return ptr_->nflextexcoord;
  }
  void set_nflextexcoord(int value) {
    ptr_->nflextexcoord = value;
  }
  int nmesh() const {
    return ptr_->nmesh;
  }
  void set_nmesh(int value) {
    ptr_->nmesh = value;
  }
  int nmeshvert() const {
    return ptr_->nmeshvert;
  }
  void set_nmeshvert(int value) {
    ptr_->nmeshvert = value;
  }
  int nmeshnormal() const {
    return ptr_->nmeshnormal;
  }
  void set_nmeshnormal(int value) {
    ptr_->nmeshnormal = value;
  }
  int nmeshtexcoord() const {
    return ptr_->nmeshtexcoord;
  }
  void set_nmeshtexcoord(int value) {
    ptr_->nmeshtexcoord = value;
  }
  int nmeshface() const {
    return ptr_->nmeshface;
  }
  void set_nmeshface(int value) {
    ptr_->nmeshface = value;
  }
  int nmeshgraph() const {
    return ptr_->nmeshgraph;
  }
  void set_nmeshgraph(int value) {
    ptr_->nmeshgraph = value;
  }
  int nmeshpoly() const {
    return ptr_->nmeshpoly;
  }
  void set_nmeshpoly(int value) {
    ptr_->nmeshpoly = value;
  }
  int nmeshpolyvert() const {
    return ptr_->nmeshpolyvert;
  }
  void set_nmeshpolyvert(int value) {
    ptr_->nmeshpolyvert = value;
  }
  int nmeshpolymap() const {
    return ptr_->nmeshpolymap;
  }
  void set_nmeshpolymap(int value) {
    ptr_->nmeshpolymap = value;
  }
  int nskin() const {
    return ptr_->nskin;
  }
  void set_nskin(int value) {
    ptr_->nskin = value;
  }
  int nskinvert() const {
    return ptr_->nskinvert;
  }
  void set_nskinvert(int value) {
    ptr_->nskinvert = value;
  }
  int nskintexvert() const {
    return ptr_->nskintexvert;
  }
  void set_nskintexvert(int value) {
    ptr_->nskintexvert = value;
  }
  int nskinface() const {
    return ptr_->nskinface;
  }
  void set_nskinface(int value) {
    ptr_->nskinface = value;
  }
  int nskinbone() const {
    return ptr_->nskinbone;
  }
  void set_nskinbone(int value) {
    ptr_->nskinbone = value;
  }
  int nskinbonevert() const {
    return ptr_->nskinbonevert;
  }
  void set_nskinbonevert(int value) {
    ptr_->nskinbonevert = value;
  }
  int nhfield() const {
    return ptr_->nhfield;
  }
  void set_nhfield(int value) {
    ptr_->nhfield = value;
  }
  int nhfielddata() const {
    return ptr_->nhfielddata;
  }
  void set_nhfielddata(int value) {
    ptr_->nhfielddata = value;
  }
  int ntex() const {
    return ptr_->ntex;
  }
  void set_ntex(int value) {
    ptr_->ntex = value;
  }
  int ntexdata() const {
    return ptr_->ntexdata;
  }
  void set_ntexdata(int value) {
    ptr_->ntexdata = value;
  }
  int nmat() const {
    return ptr_->nmat;
  }
  void set_nmat(int value) {
    ptr_->nmat = value;
  }
  int npair() const {
    return ptr_->npair;
  }
  void set_npair(int value) {
    ptr_->npair = value;
  }
  int nexclude() const {
    return ptr_->nexclude;
  }
  void set_nexclude(int value) {
    ptr_->nexclude = value;
  }
  int neq() const {
    return ptr_->neq;
  }
  void set_neq(int value) {
    ptr_->neq = value;
  }
  int ntendon() const {
    return ptr_->ntendon;
  }
  void set_ntendon(int value) {
    ptr_->ntendon = value;
  }
  int nwrap() const {
    return ptr_->nwrap;
  }
  void set_nwrap(int value) {
    ptr_->nwrap = value;
  }
  int nsensor() const {
    return ptr_->nsensor;
  }
  void set_nsensor(int value) {
    ptr_->nsensor = value;
  }
  int nnumeric() const {
    return ptr_->nnumeric;
  }
  void set_nnumeric(int value) {
    ptr_->nnumeric = value;
  }
  int nnumericdata() const {
    return ptr_->nnumericdata;
  }
  void set_nnumericdata(int value) {
    ptr_->nnumericdata = value;
  }
  int ntext() const {
    return ptr_->ntext;
  }
  void set_ntext(int value) {
    ptr_->ntext = value;
  }
  int ntextdata() const {
    return ptr_->ntextdata;
  }
  void set_ntextdata(int value) {
    ptr_->ntextdata = value;
  }
  int ntuple() const {
    return ptr_->ntuple;
  }
  void set_ntuple(int value) {
    ptr_->ntuple = value;
  }
  int ntupledata() const {
    return ptr_->ntupledata;
  }
  void set_ntupledata(int value) {
    ptr_->ntupledata = value;
  }
  int nkey() const {
    return ptr_->nkey;
  }
  void set_nkey(int value) {
    ptr_->nkey = value;
  }
  int nmocap() const {
    return ptr_->nmocap;
  }
  void set_nmocap(int value) {
    ptr_->nmocap = value;
  }
  int nplugin() const {
    return ptr_->nplugin;
  }
  void set_nplugin(int value) {
    ptr_->nplugin = value;
  }
  int npluginattr() const {
    return ptr_->npluginattr;
  }
  void set_npluginattr(int value) {
    ptr_->npluginattr = value;
  }
  int nuser_body() const {
    return ptr_->nuser_body;
  }
  void set_nuser_body(int value) {
    ptr_->nuser_body = value;
  }
  int nuser_jnt() const {
    return ptr_->nuser_jnt;
  }
  void set_nuser_jnt(int value) {
    ptr_->nuser_jnt = value;
  }
  int nuser_geom() const {
    return ptr_->nuser_geom;
  }
  void set_nuser_geom(int value) {
    ptr_->nuser_geom = value;
  }
  int nuser_site() const {
    return ptr_->nuser_site;
  }
  void set_nuser_site(int value) {
    ptr_->nuser_site = value;
  }
  int nuser_cam() const {
    return ptr_->nuser_cam;
  }
  void set_nuser_cam(int value) {
    ptr_->nuser_cam = value;
  }
  int nuser_tendon() const {
    return ptr_->nuser_tendon;
  }
  void set_nuser_tendon(int value) {
    ptr_->nuser_tendon = value;
  }
  int nuser_actuator() const {
    return ptr_->nuser_actuator;
  }
  void set_nuser_actuator(int value) {
    ptr_->nuser_actuator = value;
  }
  int nuser_sensor() const {
    return ptr_->nuser_sensor;
  }
  void set_nuser_sensor(int value) {
    ptr_->nuser_sensor = value;
  }
  int nnames() const {
    return ptr_->nnames;
  }
  void set_nnames(int value) {
    ptr_->nnames = value;
  }
  int npaths() const {
    return ptr_->npaths;
  }
  void set_npaths(int value) {
    ptr_->npaths = value;
  }
  int nnames_map() const {
    return ptr_->nnames_map;
  }
  void set_nnames_map(int value) {
    ptr_->nnames_map = value;
  }
  int nJmom() const {
    return ptr_->nJmom;
  }
  void set_nJmom(int value) {
    ptr_->nJmom = value;
  }
  int ngravcomp() const {
    return ptr_->ngravcomp;
  }
  void set_ngravcomp(int value) {
    ptr_->ngravcomp = value;
  }
  int nemax() const {
    return ptr_->nemax;
  }
  void set_nemax(int value) {
    ptr_->nemax = value;
  }
  int njmax() const {
    return ptr_->njmax;
  }
  void set_njmax(int value) {
    ptr_->njmax = value;
  }
  int nconmax() const {
    return ptr_->nconmax;
  }
  void set_nconmax(int value) {
    ptr_->nconmax = value;
  }
  int nuserdata() const {
    return ptr_->nuserdata;
  }
  void set_nuserdata(int value) {
    ptr_->nuserdata = value;
  }
  int nsensordata() const {
    return ptr_->nsensordata;
  }
  void set_nsensordata(int value) {
    ptr_->nsensordata = value;
  }
  int npluginstate() const {
    return ptr_->npluginstate;
  }
  void set_npluginstate(int value) {
    ptr_->npluginstate = value;
  }
  mjtSize narena() const {
    return ptr_->narena;
  }
  void set_narena(mjtSize value) {
    ptr_->narena = value;
  }
  mjtSize nbuffer() const {
    return ptr_->nbuffer;
  }
  void set_nbuffer(mjtSize value) {
    ptr_->nbuffer = value;
  }
  emscripten::val buffer() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbuffer, static_cast<uint8_t*>(ptr_->buffer)));
  }
  emscripten::val qpos0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nq, ptr_->qpos0));
  }
  emscripten::val qpos_spring() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nq, ptr_->qpos_spring));
  }
  emscripten::val body_parentid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_parentid));
  }
  emscripten::val body_rootid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_rootid));
  }
  emscripten::val body_weldid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_weldid));
  }
  emscripten::val body_mocapid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_mocapid));
  }
  emscripten::val body_jntnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_jntnum));
  }
  emscripten::val body_jntadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_jntadr));
  }
  emscripten::val body_dofnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_dofnum));
  }
  emscripten::val body_dofadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_dofadr));
  }
  emscripten::val body_treeid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_treeid));
  }
  emscripten::val body_geomnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_geomnum));
  }
  emscripten::val body_geomadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_geomadr));
  }
  emscripten::val body_simple() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_simple));
  }
  emscripten::val body_sameframe() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_sameframe));
  }
  emscripten::val body_pos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody * 3, ptr_->body_pos));
  }
  emscripten::val body_quat() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody * 4, ptr_->body_quat));
  }
  emscripten::val body_ipos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody * 3, ptr_->body_ipos));
  }
  emscripten::val body_iquat() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody * 4, ptr_->body_iquat));
  }
  emscripten::val body_mass() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_mass));
  }
  emscripten::val body_subtreemass() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_subtreemass));
  }
  emscripten::val body_inertia() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody * 3, ptr_->body_inertia));
  }
  emscripten::val body_invweight0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody * 2, ptr_->body_invweight0));
  }
  emscripten::val body_gravcomp() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_gravcomp));
  }
  emscripten::val body_margin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_margin));
  }
  emscripten::val body_user() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody * ptr_->nuser_body, ptr_->body_user));
  }
  emscripten::val body_plugin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_plugin));
  }
  emscripten::val body_contype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_contype));
  }
  emscripten::val body_conaffinity() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_conaffinity));
  }
  emscripten::val body_bvhadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_bvhadr));
  }
  emscripten::val body_bvhnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->body_bvhnum));
  }
  emscripten::val bvh_depth() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbvh, ptr_->bvh_depth));
  }
  emscripten::val bvh_child() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbvh * 2, ptr_->bvh_child));
  }
  emscripten::val bvh_nodeid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbvh, ptr_->bvh_nodeid));
  }
  emscripten::val bvh_aabb() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbvhstatic * 6, ptr_->bvh_aabb));
  }
  emscripten::val oct_depth() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->noct, ptr_->oct_depth));
  }
  emscripten::val oct_child() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->noct * 8, ptr_->oct_child));
  }
  emscripten::val oct_aabb() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->noct * 6, ptr_->oct_aabb));
  }
  emscripten::val oct_coeff() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->noct * 8, ptr_->oct_coeff));
  }
  emscripten::val jnt_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_type));
  }
  emscripten::val jnt_qposadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_qposadr));
  }
  emscripten::val jnt_dofadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_dofadr));
  }
  emscripten::val jnt_bodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_bodyid));
  }
  emscripten::val jnt_group() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_group));
  }
  emscripten::val jnt_limited() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_limited));
  }
  emscripten::val jnt_actfrclimited() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_actfrclimited));
  }
  emscripten::val jnt_actgravcomp() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_actgravcomp));
  }
  emscripten::val jnt_solref() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt * mjNREF, ptr_->jnt_solref));
  }
  emscripten::val jnt_solimp() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt * mjNIMP, ptr_->jnt_solimp));
  }
  emscripten::val jnt_pos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt * 3, ptr_->jnt_pos));
  }
  emscripten::val jnt_axis() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt * 3, ptr_->jnt_axis));
  }
  emscripten::val jnt_stiffness() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_stiffness));
  }
  emscripten::val jnt_range() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt * 2, ptr_->jnt_range));
  }
  emscripten::val jnt_actfrcrange() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt * 2, ptr_->jnt_actfrcrange));
  }
  emscripten::val jnt_margin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->jnt_margin));
  }
  emscripten::val jnt_user() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt * ptr_->nuser_jnt, ptr_->jnt_user));
  }
  emscripten::val dof_bodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_bodyid));
  }
  emscripten::val dof_jntid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_jntid));
  }
  emscripten::val dof_parentid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_parentid));
  }
  emscripten::val dof_treeid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_treeid));
  }
  emscripten::val dof_Madr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_Madr));
  }
  emscripten::val dof_simplenum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_simplenum));
  }
  emscripten::val dof_solref() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv * mjNREF, ptr_->dof_solref));
  }
  emscripten::val dof_solimp() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv * mjNIMP, ptr_->dof_solimp));
  }
  emscripten::val dof_frictionloss() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_frictionloss));
  }
  emscripten::val dof_armature() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_armature));
  }
  emscripten::val dof_damping() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_damping));
  }
  emscripten::val dof_invweight0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_invweight0));
  }
  emscripten::val dof_M0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_M0));
  }
  emscripten::val dof_length() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->dof_length));
  }
  emscripten::val tree_bodyadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntree, ptr_->tree_bodyadr));
  }
  emscripten::val tree_bodynum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntree, ptr_->tree_bodynum));
  }
  emscripten::val tree_dofadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntree, ptr_->tree_dofadr));
  }
  emscripten::val tree_dofnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntree, ptr_->tree_dofnum));
  }
  emscripten::val tree_sleep_policy() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntree, ptr_->tree_sleep_policy));
  }
  emscripten::val geom_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_type));
  }
  emscripten::val geom_contype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_contype));
  }
  emscripten::val geom_conaffinity() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_conaffinity));
  }
  emscripten::val geom_condim() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_condim));
  }
  emscripten::val geom_bodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_bodyid));
  }
  emscripten::val geom_dataid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_dataid));
  }
  emscripten::val geom_matid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_matid));
  }
  emscripten::val geom_group() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_group));
  }
  emscripten::val geom_priority() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_priority));
  }
  emscripten::val geom_plugin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_plugin));
  }
  emscripten::val geom_sameframe() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_sameframe));
  }
  emscripten::val geom_solmix() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_solmix));
  }
  emscripten::val geom_solref() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * mjNREF, ptr_->geom_solref));
  }
  emscripten::val geom_solimp() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * mjNIMP, ptr_->geom_solimp));
  }
  emscripten::val geom_size() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 3, ptr_->geom_size));
  }
  emscripten::val geom_aabb() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 6, ptr_->geom_aabb));
  }
  emscripten::val geom_rbound() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_rbound));
  }
  emscripten::val geom_pos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 3, ptr_->geom_pos));
  }
  emscripten::val geom_quat() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 4, ptr_->geom_quat));
  }
  emscripten::val geom_friction() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 3, ptr_->geom_friction));
  }
  emscripten::val geom_margin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_margin));
  }
  emscripten::val geom_gap() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->geom_gap));
  }
  emscripten::val geom_fluid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * mjNFLUID, ptr_->geom_fluid));
  }
  emscripten::val geom_user() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * ptr_->nuser_geom, ptr_->geom_user));
  }
  emscripten::val geom_rgba() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 4, ptr_->geom_rgba));
  }
  emscripten::val site_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite, ptr_->site_type));
  }
  emscripten::val site_bodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite, ptr_->site_bodyid));
  }
  emscripten::val site_matid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite, ptr_->site_matid));
  }
  emscripten::val site_group() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite, ptr_->site_group));
  }
  emscripten::val site_sameframe() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite, ptr_->site_sameframe));
  }
  emscripten::val site_size() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite * 3, ptr_->site_size));
  }
  emscripten::val site_pos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite * 3, ptr_->site_pos));
  }
  emscripten::val site_quat() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite * 4, ptr_->site_quat));
  }
  emscripten::val site_user() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite * ptr_->nuser_site, ptr_->site_user));
  }
  emscripten::val site_rgba() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite * 4, ptr_->site_rgba));
  }
  emscripten::val cam_mode() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam, ptr_->cam_mode));
  }
  emscripten::val cam_bodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam, ptr_->cam_bodyid));
  }
  emscripten::val cam_targetbodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam, ptr_->cam_targetbodyid));
  }
  emscripten::val cam_pos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam * 3, ptr_->cam_pos));
  }
  emscripten::val cam_quat() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam * 4, ptr_->cam_quat));
  }
  emscripten::val cam_poscom0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam * 3, ptr_->cam_poscom0));
  }
  emscripten::val cam_pos0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam * 3, ptr_->cam_pos0));
  }
  emscripten::val cam_mat0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam * 9, ptr_->cam_mat0));
  }
  emscripten::val cam_orthographic() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam, ptr_->cam_orthographic));
  }
  emscripten::val cam_fovy() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam, ptr_->cam_fovy));
  }
  emscripten::val cam_ipd() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam, ptr_->cam_ipd));
  }
  emscripten::val cam_resolution() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam * 2, ptr_->cam_resolution));
  }
  emscripten::val cam_sensorsize() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam * 2, ptr_->cam_sensorsize));
  }
  emscripten::val cam_intrinsic() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam * 4, ptr_->cam_intrinsic));
  }
  emscripten::val cam_user() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam * ptr_->nuser_cam, ptr_->cam_user));
  }
  emscripten::val light_mode() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_mode));
  }
  emscripten::val light_bodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_bodyid));
  }
  emscripten::val light_targetbodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_targetbodyid));
  }
  emscripten::val light_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_type));
  }
  emscripten::val light_texid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_texid));
  }
  emscripten::val light_castshadow() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_castshadow));
  }
  emscripten::val light_bulbradius() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_bulbradius));
  }
  emscripten::val light_intensity() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_intensity));
  }
  emscripten::val light_range() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_range));
  }
  emscripten::val light_active() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_active));
  }
  emscripten::val light_pos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight * 3, ptr_->light_pos));
  }
  emscripten::val light_dir() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight * 3, ptr_->light_dir));
  }
  emscripten::val light_poscom0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight * 3, ptr_->light_poscom0));
  }
  emscripten::val light_pos0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight * 3, ptr_->light_pos0));
  }
  emscripten::val light_dir0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight * 3, ptr_->light_dir0));
  }
  emscripten::val light_attenuation() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight * 3, ptr_->light_attenuation));
  }
  emscripten::val light_cutoff() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_cutoff));
  }
  emscripten::val light_exponent() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->light_exponent));
  }
  emscripten::val light_ambient() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight * 3, ptr_->light_ambient));
  }
  emscripten::val light_diffuse() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight * 3, ptr_->light_diffuse));
  }
  emscripten::val light_specular() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight * 3, ptr_->light_specular));
  }
  emscripten::val flex_contype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_contype));
  }
  emscripten::val flex_conaffinity() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_conaffinity));
  }
  emscripten::val flex_condim() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_condim));
  }
  emscripten::val flex_priority() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_priority));
  }
  emscripten::val flex_solmix() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_solmix));
  }
  emscripten::val flex_solref() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex * mjNREF, ptr_->flex_solref));
  }
  emscripten::val flex_solimp() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex * mjNIMP, ptr_->flex_solimp));
  }
  emscripten::val flex_friction() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex * 3, ptr_->flex_friction));
  }
  emscripten::val flex_margin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_margin));
  }
  emscripten::val flex_gap() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_gap));
  }
  emscripten::val flex_internal() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_internal));
  }
  emscripten::val flex_selfcollide() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_selfcollide));
  }
  emscripten::val flex_activelayers() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_activelayers));
  }
  emscripten::val flex_passive() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_passive));
  }
  emscripten::val flex_dim() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_dim));
  }
  emscripten::val flex_matid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_matid));
  }
  emscripten::val flex_group() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_group));
  }
  emscripten::val flex_interp() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_interp));
  }
  emscripten::val flex_nodeadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_nodeadr));
  }
  emscripten::val flex_nodenum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_nodenum));
  }
  emscripten::val flex_vertadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_vertadr));
  }
  emscripten::val flex_vertnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_vertnum));
  }
  emscripten::val flex_edgeadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_edgeadr));
  }
  emscripten::val flex_edgenum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_edgenum));
  }
  emscripten::val flex_elemadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_elemadr));
  }
  emscripten::val flex_elemnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_elemnum));
  }
  emscripten::val flex_elemdataadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_elemdataadr));
  }
  emscripten::val flex_elemedgeadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_elemedgeadr));
  }
  emscripten::val flex_shellnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_shellnum));
  }
  emscripten::val flex_shelldataadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_shelldataadr));
  }
  emscripten::val flex_evpairadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_evpairadr));
  }
  emscripten::val flex_evpairnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_evpairnum));
  }
  emscripten::val flex_texcoordadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_texcoordadr));
  }
  emscripten::val flex_nodebodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexnode, ptr_->flex_nodebodyid));
  }
  emscripten::val flex_vertbodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexvert, ptr_->flex_vertbodyid));
  }
  emscripten::val flex_edge() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexedge * 2, ptr_->flex_edge));
  }
  emscripten::val flex_edgeflap() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexedge * 2, ptr_->flex_edgeflap));
  }
  emscripten::val flex_elem() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexelemdata, ptr_->flex_elem));
  }
  emscripten::val flex_elemtexcoord() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexelemdata, ptr_->flex_elemtexcoord));
  }
  emscripten::val flex_elemedge() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexelemedge, ptr_->flex_elemedge));
  }
  emscripten::val flex_elemlayer() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexelem, ptr_->flex_elemlayer));
  }
  emscripten::val flex_shell() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexshelldata, ptr_->flex_shell));
  }
  emscripten::val flex_evpair() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexevpair * 2, ptr_->flex_evpair));
  }
  emscripten::val flex_vert() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexvert * 3, ptr_->flex_vert));
  }
  emscripten::val flex_vert0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexvert * 3, ptr_->flex_vert0));
  }
  emscripten::val flex_node() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexnode * 3, ptr_->flex_node));
  }
  emscripten::val flex_node0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexnode * 3, ptr_->flex_node0));
  }
  emscripten::val flexedge_length0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexedge, ptr_->flexedge_length0));
  }
  emscripten::val flexedge_invweight0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexedge, ptr_->flexedge_invweight0));
  }
  emscripten::val flex_radius() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_radius));
  }
  emscripten::val flex_stiffness() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexelem * 21, ptr_->flex_stiffness));
  }
  emscripten::val flex_bending() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexedge * 17, ptr_->flex_bending));
  }
  emscripten::val flex_damping() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_damping));
  }
  emscripten::val flex_edgestiffness() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_edgestiffness));
  }
  emscripten::val flex_edgedamping() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_edgedamping));
  }
  emscripten::val flex_edgeequality() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_edgeequality));
  }
  emscripten::val flex_rigid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_rigid));
  }
  emscripten::val flexedge_rigid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflexedge, ptr_->flexedge_rigid));
  }
  emscripten::val flex_centered() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_centered));
  }
  emscripten::val flex_flatskin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_flatskin));
  }
  emscripten::val flex_bvhadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_bvhadr));
  }
  emscripten::val flex_bvhnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->flex_bvhnum));
  }
  emscripten::val flex_rgba() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex * 4, ptr_->flex_rgba));
  }
  emscripten::val flex_texcoord() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflextexcoord * 2, ptr_->flex_texcoord));
  }
  emscripten::val mesh_vertadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_vertadr));
  }
  emscripten::val mesh_vertnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_vertnum));
  }
  emscripten::val mesh_faceadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_faceadr));
  }
  emscripten::val mesh_facenum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_facenum));
  }
  emscripten::val mesh_bvhadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_bvhadr));
  }
  emscripten::val mesh_bvhnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_bvhnum));
  }
  emscripten::val mesh_octadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_octadr));
  }
  emscripten::val mesh_octnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_octnum));
  }
  emscripten::val mesh_normaladr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_normaladr));
  }
  emscripten::val mesh_normalnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_normalnum));
  }
  emscripten::val mesh_texcoordadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_texcoordadr));
  }
  emscripten::val mesh_texcoordnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_texcoordnum));
  }
  emscripten::val mesh_graphadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_graphadr));
  }
  emscripten::val mesh_vert() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshvert * 3, ptr_->mesh_vert));
  }
  emscripten::val mesh_normal() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshnormal * 3, ptr_->mesh_normal));
  }
  emscripten::val mesh_texcoord() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshtexcoord * 2, ptr_->mesh_texcoord));
  }
  emscripten::val mesh_face() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshface * 3, ptr_->mesh_face));
  }
  emscripten::val mesh_facenormal() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshface * 3, ptr_->mesh_facenormal));
  }
  emscripten::val mesh_facetexcoord() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshface * 3, ptr_->mesh_facetexcoord));
  }
  emscripten::val mesh_graph() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshgraph, ptr_->mesh_graph));
  }
  emscripten::val mesh_scale() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh * 3, ptr_->mesh_scale));
  }
  emscripten::val mesh_pos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh * 3, ptr_->mesh_pos));
  }
  emscripten::val mesh_quat() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh * 4, ptr_->mesh_quat));
  }
  emscripten::val mesh_pathadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_pathadr));
  }
  emscripten::val mesh_polynum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_polynum));
  }
  emscripten::val mesh_polyadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->mesh_polyadr));
  }
  emscripten::val mesh_polynormal() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshpoly * 3, ptr_->mesh_polynormal));
  }
  emscripten::val mesh_polyvertadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshpoly, ptr_->mesh_polyvertadr));
  }
  emscripten::val mesh_polyvertnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshpoly, ptr_->mesh_polyvertnum));
  }
  emscripten::val mesh_polyvert() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshpolyvert, ptr_->mesh_polyvert));
  }
  emscripten::val mesh_polymapadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshvert, ptr_->mesh_polymapadr));
  }
  emscripten::val mesh_polymapnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshvert, ptr_->mesh_polymapnum));
  }
  emscripten::val mesh_polymap() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmeshpolymap, ptr_->mesh_polymap));
  }
  emscripten::val skin_matid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_matid));
  }
  emscripten::val skin_group() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_group));
  }
  emscripten::val skin_rgba() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin * 4, ptr_->skin_rgba));
  }
  emscripten::val skin_inflate() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_inflate));
  }
  emscripten::val skin_vertadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_vertadr));
  }
  emscripten::val skin_vertnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_vertnum));
  }
  emscripten::val skin_texcoordadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_texcoordadr));
  }
  emscripten::val skin_faceadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_faceadr));
  }
  emscripten::val skin_facenum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_facenum));
  }
  emscripten::val skin_boneadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_boneadr));
  }
  emscripten::val skin_bonenum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_bonenum));
  }
  emscripten::val skin_vert() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskinvert * 3, ptr_->skin_vert));
  }
  emscripten::val skin_texcoord() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskintexvert * 2, ptr_->skin_texcoord));
  }
  emscripten::val skin_face() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskinface * 3, ptr_->skin_face));
  }
  emscripten::val skin_bonevertadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskinbone, ptr_->skin_bonevertadr));
  }
  emscripten::val skin_bonevertnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskinbone, ptr_->skin_bonevertnum));
  }
  emscripten::val skin_bonebindpos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskinbone * 3, ptr_->skin_bonebindpos));
  }
  emscripten::val skin_bonebindquat() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskinbone * 4, ptr_->skin_bonebindquat));
  }
  emscripten::val skin_bonebodyid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskinbone, ptr_->skin_bonebodyid));
  }
  emscripten::val skin_bonevertid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskinbonevert, ptr_->skin_bonevertid));
  }
  emscripten::val skin_bonevertweight() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskinbonevert, ptr_->skin_bonevertweight));
  }
  emscripten::val skin_pathadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->skin_pathadr));
  }
  emscripten::val hfield_size() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nhfield * 4, ptr_->hfield_size));
  }
  emscripten::val hfield_nrow() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nhfield, ptr_->hfield_nrow));
  }
  emscripten::val hfield_ncol() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nhfield, ptr_->hfield_ncol));
  }
  emscripten::val hfield_adr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nhfield, ptr_->hfield_adr));
  }
  emscripten::val hfield_data() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nhfielddata, ptr_->hfield_data));
  }
  emscripten::val hfield_pathadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nhfield, ptr_->hfield_pathadr));
  }
  emscripten::val tex_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntex, ptr_->tex_type));
  }
  emscripten::val tex_colorspace() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntex, ptr_->tex_colorspace));
  }
  emscripten::val tex_height() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntex, ptr_->tex_height));
  }
  emscripten::val tex_width() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntex, ptr_->tex_width));
  }
  emscripten::val tex_nchannel() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntex, ptr_->tex_nchannel));
  }
  emscripten::val tex_adr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntex, ptr_->tex_adr));
  }
  emscripten::val tex_data() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntexdata, ptr_->tex_data));
  }
  emscripten::val tex_pathadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntex, ptr_->tex_pathadr));
  }
  emscripten::val mat_texid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat * mjNTEXROLE, ptr_->mat_texid));
  }
  emscripten::val mat_texuniform() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat, ptr_->mat_texuniform));
  }
  emscripten::val mat_texrepeat() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat * 2, ptr_->mat_texrepeat));
  }
  emscripten::val mat_emission() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat, ptr_->mat_emission));
  }
  emscripten::val mat_specular() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat, ptr_->mat_specular));
  }
  emscripten::val mat_shininess() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat, ptr_->mat_shininess));
  }
  emscripten::val mat_reflectance() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat, ptr_->mat_reflectance));
  }
  emscripten::val mat_metallic() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat, ptr_->mat_metallic));
  }
  emscripten::val mat_roughness() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat, ptr_->mat_roughness));
  }
  emscripten::val mat_rgba() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat * 4, ptr_->mat_rgba));
  }
  emscripten::val pair_dim() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair, ptr_->pair_dim));
  }
  emscripten::val pair_geom1() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair, ptr_->pair_geom1));
  }
  emscripten::val pair_geom2() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair, ptr_->pair_geom2));
  }
  emscripten::val pair_signature() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair, ptr_->pair_signature));
  }
  emscripten::val pair_solref() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair * mjNREF, ptr_->pair_solref));
  }
  emscripten::val pair_solreffriction() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair * mjNREF, ptr_->pair_solreffriction));
  }
  emscripten::val pair_solimp() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair * mjNIMP, ptr_->pair_solimp));
  }
  emscripten::val pair_margin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair, ptr_->pair_margin));
  }
  emscripten::val pair_gap() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair, ptr_->pair_gap));
  }
  emscripten::val pair_friction() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair * 5, ptr_->pair_friction));
  }
  emscripten::val exclude_signature() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nexclude, ptr_->exclude_signature));
  }
  emscripten::val eq_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->neq, ptr_->eq_type));
  }
  emscripten::val eq_obj1id() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->neq, ptr_->eq_obj1id));
  }
  emscripten::val eq_obj2id() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->neq, ptr_->eq_obj2id));
  }
  emscripten::val eq_objtype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->neq, ptr_->eq_objtype));
  }
  emscripten::val eq_active0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->neq, ptr_->eq_active0));
  }
  emscripten::val eq_solref() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->neq * mjNREF, ptr_->eq_solref));
  }
  emscripten::val eq_solimp() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->neq * mjNIMP, ptr_->eq_solimp));
  }
  emscripten::val eq_data() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->neq * mjNEQDATA, ptr_->eq_data));
  }
  emscripten::val tendon_adr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_adr));
  }
  emscripten::val tendon_num() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_num));
  }
  emscripten::val tendon_matid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_matid));
  }
  emscripten::val tendon_group() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_group));
  }
  emscripten::val tendon_treenum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_treenum));
  }
  emscripten::val tendon_treeid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * 2, ptr_->tendon_treeid));
  }
  emscripten::val tendon_limited() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_limited));
  }
  emscripten::val tendon_actfrclimited() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_actfrclimited));
  }
  emscripten::val tendon_width() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_width));
  }
  emscripten::val tendon_solref_lim() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * mjNREF, ptr_->tendon_solref_lim));
  }
  emscripten::val tendon_solimp_lim() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * mjNIMP, ptr_->tendon_solimp_lim));
  }
  emscripten::val tendon_solref_fri() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * mjNREF, ptr_->tendon_solref_fri));
  }
  emscripten::val tendon_solimp_fri() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * mjNIMP, ptr_->tendon_solimp_fri));
  }
  emscripten::val tendon_range() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * 2, ptr_->tendon_range));
  }
  emscripten::val tendon_actfrcrange() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * 2, ptr_->tendon_actfrcrange));
  }
  emscripten::val tendon_margin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_margin));
  }
  emscripten::val tendon_stiffness() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_stiffness));
  }
  emscripten::val tendon_damping() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_damping));
  }
  emscripten::val tendon_armature() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_armature));
  }
  emscripten::val tendon_frictionloss() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_frictionloss));
  }
  emscripten::val tendon_lengthspring() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * 2, ptr_->tendon_lengthspring));
  }
  emscripten::val tendon_length0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_length0));
  }
  emscripten::val tendon_invweight0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->tendon_invweight0));
  }
  emscripten::val tendon_user() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * ptr_->nuser_tendon, ptr_->tendon_user));
  }
  emscripten::val tendon_rgba() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon * 4, ptr_->tendon_rgba));
  }
  emscripten::val wrap_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nwrap, ptr_->wrap_type));
  }
  emscripten::val wrap_objid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nwrap, ptr_->wrap_objid));
  }
  emscripten::val wrap_prm() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nwrap, ptr_->wrap_prm));
  }
  emscripten::val actuator_trntype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_trntype));
  }
  emscripten::val actuator_dyntype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_dyntype));
  }
  emscripten::val actuator_gaintype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_gaintype));
  }
  emscripten::val actuator_biastype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_biastype));
  }
  emscripten::val actuator_trnid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * 2, ptr_->actuator_trnid));
  }
  emscripten::val actuator_actadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_actadr));
  }
  emscripten::val actuator_actnum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_actnum));
  }
  emscripten::val actuator_group() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_group));
  }
  emscripten::val actuator_ctrllimited() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_ctrllimited));
  }
  emscripten::val actuator_forcelimited() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_forcelimited));
  }
  emscripten::val actuator_actlimited() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_actlimited));
  }
  emscripten::val actuator_dynprm() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * mjNDYN, ptr_->actuator_dynprm));
  }
  emscripten::val actuator_gainprm() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * mjNGAIN, ptr_->actuator_gainprm));
  }
  emscripten::val actuator_biasprm() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * mjNBIAS, ptr_->actuator_biasprm));
  }
  emscripten::val actuator_actearly() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_actearly));
  }
  emscripten::val actuator_ctrlrange() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * 2, ptr_->actuator_ctrlrange));
  }
  emscripten::val actuator_forcerange() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * 2, ptr_->actuator_forcerange));
  }
  emscripten::val actuator_actrange() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * 2, ptr_->actuator_actrange));
  }
  emscripten::val actuator_gear() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * 6, ptr_->actuator_gear));
  }
  emscripten::val actuator_cranklength() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_cranklength));
  }
  emscripten::val actuator_acc0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_acc0));
  }
  emscripten::val actuator_length0() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_length0));
  }
  emscripten::val actuator_lengthrange() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * 2, ptr_->actuator_lengthrange));
  }
  emscripten::val actuator_user() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu * ptr_->nuser_actuator, ptr_->actuator_user));
  }
  emscripten::val actuator_plugin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->actuator_plugin));
  }
  emscripten::val sensor_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_type));
  }
  emscripten::val sensor_datatype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_datatype));
  }
  emscripten::val sensor_needstage() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_needstage));
  }
  emscripten::val sensor_objtype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_objtype));
  }
  emscripten::val sensor_objid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_objid));
  }
  emscripten::val sensor_reftype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_reftype));
  }
  emscripten::val sensor_refid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_refid));
  }
  emscripten::val sensor_intprm() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor * mjNSENS, ptr_->sensor_intprm));
  }
  emscripten::val sensor_dim() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_dim));
  }
  emscripten::val sensor_adr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_adr));
  }
  emscripten::val sensor_cutoff() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_cutoff));
  }
  emscripten::val sensor_noise() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_noise));
  }
  emscripten::val sensor_user() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor * ptr_->nuser_sensor, ptr_->sensor_user));
  }
  emscripten::val sensor_plugin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->sensor_plugin));
  }
  emscripten::val plugin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nplugin, ptr_->plugin));
  }
  emscripten::val plugin_stateadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nplugin, ptr_->plugin_stateadr));
  }
  emscripten::val plugin_statenum() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nplugin, ptr_->plugin_statenum));
  }
  emscripten::val plugin_attr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npluginattr, ptr_->plugin_attr));
  }
  emscripten::val plugin_attradr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nplugin, ptr_->plugin_attradr));
  }
  emscripten::val numeric_adr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nnumeric, ptr_->numeric_adr));
  }
  emscripten::val numeric_size() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nnumeric, ptr_->numeric_size));
  }
  emscripten::val numeric_data() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nnumericdata, ptr_->numeric_data));
  }
  emscripten::val text_adr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntext, ptr_->text_adr));
  }
  emscripten::val text_size() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntext, ptr_->text_size));
  }
  emscripten::val text_data() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntextdata, ptr_->text_data));
  }
  emscripten::val tuple_adr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntuple, ptr_->tuple_adr));
  }
  emscripten::val tuple_size() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntuple, ptr_->tuple_size));
  }
  emscripten::val tuple_objtype() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntupledata, ptr_->tuple_objtype));
  }
  emscripten::val tuple_objid() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntupledata, ptr_->tuple_objid));
  }
  emscripten::val tuple_objprm() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntupledata, ptr_->tuple_objprm));
  }
  emscripten::val key_time() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nkey, ptr_->key_time));
  }
  emscripten::val key_qpos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nkey * ptr_->nq, ptr_->key_qpos));
  }
  emscripten::val key_qvel() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nkey * ptr_->nv, ptr_->key_qvel));
  }
  emscripten::val key_act() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nkey * ptr_->na, ptr_->key_act));
  }
  emscripten::val key_mpos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nkey * ptr_->nmocap*3, ptr_->key_mpos));
  }
  emscripten::val key_mquat() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nkey * ptr_->nmocap*4, ptr_->key_mquat));
  }
  emscripten::val key_ctrl() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nkey * ptr_->nu, ptr_->key_ctrl));
  }
  emscripten::val name_bodyadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->name_bodyadr));
  }
  emscripten::val name_jntadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->njnt, ptr_->name_jntadr));
  }
  emscripten::val name_geomadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom, ptr_->name_geomadr));
  }
  emscripten::val name_siteadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsite, ptr_->name_siteadr));
  }
  emscripten::val name_camadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ncam, ptr_->name_camadr));
  }
  emscripten::val name_lightadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nlight, ptr_->name_lightadr));
  }
  emscripten::val name_flexadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nflex, ptr_->name_flexadr));
  }
  emscripten::val name_meshadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmesh, ptr_->name_meshadr));
  }
  emscripten::val name_skinadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nskin, ptr_->name_skinadr));
  }
  emscripten::val name_hfieldadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nhfield, ptr_->name_hfieldadr));
  }
  emscripten::val name_texadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntex, ptr_->name_texadr));
  }
  emscripten::val name_matadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nmat, ptr_->name_matadr));
  }
  emscripten::val name_pairadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npair, ptr_->name_pairadr));
  }
  emscripten::val name_excludeadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nexclude, ptr_->name_excludeadr));
  }
  emscripten::val name_eqadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->neq, ptr_->name_eqadr));
  }
  emscripten::val name_tendonadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntendon, ptr_->name_tendonadr));
  }
  emscripten::val name_actuatoradr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nu, ptr_->name_actuatoradr));
  }
  emscripten::val name_sensoradr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nsensor, ptr_->name_sensoradr));
  }
  emscripten::val name_numericadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nnumeric, ptr_->name_numericadr));
  }
  emscripten::val name_textadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntext, ptr_->name_textadr));
  }
  emscripten::val name_tupleadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->ntuple, ptr_->name_tupleadr));
  }
  emscripten::val name_keyadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nkey, ptr_->name_keyadr));
  }
  emscripten::val name_pluginadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nplugin, ptr_->name_pluginadr));
  }
  emscripten::val names() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nnames, ptr_->names));
  }
  emscripten::val names_map() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nnames_map, ptr_->names_map));
  }
  emscripten::val paths() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->npaths, ptr_->paths));
  }
  emscripten::val B_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->B_rownnz));
  }
  emscripten::val B_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nbody, ptr_->B_rowadr));
  }
  emscripten::val B_colind() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nB, ptr_->B_colind));
  }
  emscripten::val M_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->M_rownnz));
  }
  emscripten::val M_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->M_rowadr));
  }
  emscripten::val M_colind() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nC, ptr_->M_colind));
  }
  emscripten::val mapM2M() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nC, ptr_->mapM2M));
  }
  emscripten::val D_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->D_rownnz));
  }
  emscripten::val D_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->D_rowadr));
  }
  emscripten::val D_diag() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nv, ptr_->D_diag));
  }
  emscripten::val D_colind() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nD, ptr_->D_colind));
  }
  emscripten::val mapM2D() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nD, ptr_->mapM2D));
  }
  emscripten::val mapD2M() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nC, ptr_->mapD2M));
  }
  uint64_t signature() const {
    return ptr_->signature;
  }
  void set_signature(uint64_t value) {
    ptr_->signature = value;
  }

 private:
  mjModel* ptr_;

 public:
  MjOption opt;
  MjVisual vis;
  MjStatistic stat;
};

struct MjSpec {
  ~MjSpec();
  MjSpec();
  explicit MjSpec(mjSpec *ptr);
  MjSpec(const MjSpec &);
  MjSpec &operator=(const MjSpec &);
  std::unique_ptr<MjSpec> copy();
  mjSpec* get() const;
  void set(mjSpec* ptr);
  mjString modelname() const {
    return (ptr_ && ptr_->modelname) ? *(ptr_->modelname) : "";
  }
  void set_modelname(const mjString& value) {
    if (ptr_ && ptr_->modelname) {
      *(ptr_->modelname) = value;
    }
  }
  mjtByte strippath() const {
    return ptr_->strippath;
  }
  void set_strippath(mjtByte value) {
    ptr_->strippath = value;
  }
  mjtSize memory() const {
    return ptr_->memory;
  }
  void set_memory(mjtSize value) {
    ptr_->memory = value;
  }
  int nemax() const {
    return ptr_->nemax;
  }
  void set_nemax(int value) {
    ptr_->nemax = value;
  }
  int nuserdata() const {
    return ptr_->nuserdata;
  }
  void set_nuserdata(int value) {
    ptr_->nuserdata = value;
  }
  int nuser_body() const {
    return ptr_->nuser_body;
  }
  void set_nuser_body(int value) {
    ptr_->nuser_body = value;
  }
  int nuser_jnt() const {
    return ptr_->nuser_jnt;
  }
  void set_nuser_jnt(int value) {
    ptr_->nuser_jnt = value;
  }
  int nuser_geom() const {
    return ptr_->nuser_geom;
  }
  void set_nuser_geom(int value) {
    ptr_->nuser_geom = value;
  }
  int nuser_site() const {
    return ptr_->nuser_site;
  }
  void set_nuser_site(int value) {
    ptr_->nuser_site = value;
  }
  int nuser_cam() const {
    return ptr_->nuser_cam;
  }
  void set_nuser_cam(int value) {
    ptr_->nuser_cam = value;
  }
  int nuser_tendon() const {
    return ptr_->nuser_tendon;
  }
  void set_nuser_tendon(int value) {
    ptr_->nuser_tendon = value;
  }
  int nuser_actuator() const {
    return ptr_->nuser_actuator;
  }
  void set_nuser_actuator(int value) {
    ptr_->nuser_actuator = value;
  }
  int nuser_sensor() const {
    return ptr_->nuser_sensor;
  }
  void set_nuser_sensor(int value) {
    ptr_->nuser_sensor = value;
  }
  int nkey() const {
    return ptr_->nkey;
  }
  void set_nkey(int value) {
    ptr_->nkey = value;
  }
  int njmax() const {
    return ptr_->njmax;
  }
  void set_njmax(int value) {
    ptr_->njmax = value;
  }
  int nconmax() const {
    return ptr_->nconmax;
  }
  void set_nconmax(int value) {
    ptr_->nconmax = value;
  }
  mjtSize nstack() const {
    return ptr_->nstack;
  }
  void set_nstack(mjtSize value) {
    ptr_->nstack = value;
  }
  mjString comment() const {
    return (ptr_ && ptr_->comment) ? *(ptr_->comment) : "";
  }
  void set_comment(const mjString& value) {
    if (ptr_ && ptr_->comment) {
      *(ptr_->comment) = value;
    }
  }
  mjString modelfiledir() const {
    return (ptr_ && ptr_->modelfiledir) ? *(ptr_->modelfiledir) : "";
  }
  void set_modelfiledir(const mjString& value) {
    if (ptr_ && ptr_->modelfiledir) {
      *(ptr_->modelfiledir) = value;
    }
  }
  mjtByte hasImplicitPluginElem() const {
    return ptr_->hasImplicitPluginElem;
  }
  void set_hasImplicitPluginElem(mjtByte value) {
    ptr_->hasImplicitPluginElem = value;
  }

 private:
  mjSpec* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
  MjsCompiler compiler;
  MjOption option;
  MjVisual visual;
  MjStatistic stat;
};

struct MjsActuator {
  explicit MjsActuator(mjsActuator *ptr);
  mjsActuator* get() const;
  void set(mjsActuator* ptr);
  mjtGain gaintype() const {
    return ptr_->gaintype;
  }
  void set_gaintype(mjtGain value) {
    ptr_->gaintype = value;
  }
  emscripten::val gainprm() const {
    return emscripten::val(emscripten::typed_memory_view(10, ptr_->gainprm));
  }
  mjtBias biastype() const {
    return ptr_->biastype;
  }
  void set_biastype(mjtBias value) {
    ptr_->biastype = value;
  }
  emscripten::val biasprm() const {
    return emscripten::val(emscripten::typed_memory_view(10, ptr_->biasprm));
  }
  mjtDyn dyntype() const {
    return ptr_->dyntype;
  }
  void set_dyntype(mjtDyn value) {
    ptr_->dyntype = value;
  }
  emscripten::val dynprm() const {
    return emscripten::val(emscripten::typed_memory_view(10, ptr_->dynprm));
  }
  int actdim() const {
    return ptr_->actdim;
  }
  void set_actdim(int value) {
    ptr_->actdim = value;
  }
  mjtByte actearly() const {
    return ptr_->actearly;
  }
  void set_actearly(mjtByte value) {
    ptr_->actearly = value;
  }
  mjtTrn trntype() const {
    return ptr_->trntype;
  }
  void set_trntype(mjtTrn value) {
    ptr_->trntype = value;
  }
  emscripten::val gear() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->gear));
  }
  mjString target() const {
    return (ptr_ && ptr_->target) ? *(ptr_->target) : "";
  }
  void set_target(const mjString& value) {
    if (ptr_ && ptr_->target) {
      *(ptr_->target) = value;
    }
  }
  mjString refsite() const {
    return (ptr_ && ptr_->refsite) ? *(ptr_->refsite) : "";
  }
  void set_refsite(const mjString& value) {
    if (ptr_ && ptr_->refsite) {
      *(ptr_->refsite) = value;
    }
  }
  mjString slidersite() const {
    return (ptr_ && ptr_->slidersite) ? *(ptr_->slidersite) : "";
  }
  void set_slidersite(const mjString& value) {
    if (ptr_ && ptr_->slidersite) {
      *(ptr_->slidersite) = value;
    }
  }
  double cranklength() const {
    return ptr_->cranklength;
  }
  void set_cranklength(double value) {
    ptr_->cranklength = value;
  }
  emscripten::val lengthrange() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->lengthrange));
  }
  double inheritrange() const {
    return ptr_->inheritrange;
  }
  void set_inheritrange(double value) {
    ptr_->inheritrange = value;
  }
  int ctrllimited() const {
    return ptr_->ctrllimited;
  }
  void set_ctrllimited(int value) {
    ptr_->ctrllimited = value;
  }
  emscripten::val ctrlrange() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->ctrlrange));
  }
  int forcelimited() const {
    return ptr_->forcelimited;
  }
  void set_forcelimited(int value) {
    ptr_->forcelimited = value;
  }
  emscripten::val forcerange() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->forcerange));
  }
  int actlimited() const {
    return ptr_->actlimited;
  }
  void set_actlimited(int value) {
    ptr_->actlimited = value;
  }
  emscripten::val actrange() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->actrange));
  }
  int group() const {
    return ptr_->group;
  }
  void set_group(int value) {
    ptr_->group = value;
  }
  mjDoubleVec &userdata() const {
    return *(ptr_->userdata);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsActuator* ptr_;

 public:
  MjsElement element;
  MjsPlugin plugin;
};

struct MjsBody {
  explicit MjsBody(mjsBody *ptr);
  mjsBody* get() const;
  void set(mjsBody* ptr);
  mjString childclass() const {
    return (ptr_ && ptr_->childclass) ? *(ptr_->childclass) : "";
  }
  void set_childclass(const mjString& value) {
    if (ptr_ && ptr_->childclass) {
      *(ptr_->childclass) = value;
    }
  }
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val quat() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->quat));
  }
  double mass() const {
    return ptr_->mass;
  }
  void set_mass(double value) {
    ptr_->mass = value;
  }
  emscripten::val ipos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->ipos));
  }
  emscripten::val iquat() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->iquat));
  }
  emscripten::val inertia() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->inertia));
  }
  emscripten::val fullinertia() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->fullinertia));
  }
  mjtByte mocap() const {
    return ptr_->mocap;
  }
  void set_mocap(mjtByte value) {
    ptr_->mocap = value;
  }
  double gravcomp() const {
    return ptr_->gravcomp;
  }
  void set_gravcomp(double value) {
    ptr_->gravcomp = value;
  }
  mjtSleepPolicy sleep() const {
    return ptr_->sleep;
  }
  void set_sleep(mjtSleepPolicy value) {
    ptr_->sleep = value;
  }
  mjDoubleVec &userdata() const {
    return *(ptr_->userdata);
  }
  mjtByte explicitinertial() const {
    return ptr_->explicitinertial;
  }
  void set_explicitinertial(mjtByte value) {
    ptr_->explicitinertial = value;
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsBody* ptr_;

 public:
  MjsElement element;
  MjsOrientation alt;
  MjsOrientation ialt;
  MjsPlugin plugin;
};

struct MjsGeom {
  explicit MjsGeom(mjsGeom *ptr);
  mjsGeom* get() const;
  void set(mjsGeom* ptr);
  mjtGeom type() const {
    return ptr_->type;
  }
  void set_type(mjtGeom value) {
    ptr_->type = value;
  }
  emscripten::val pos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->pos));
  }
  emscripten::val quat() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->quat));
  }
  emscripten::val fromto() const {
    return emscripten::val(emscripten::typed_memory_view(6, ptr_->fromto));
  }
  emscripten::val size() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->size));
  }
  int contype() const {
    return ptr_->contype;
  }
  void set_contype(int value) {
    ptr_->contype = value;
  }
  int conaffinity() const {
    return ptr_->conaffinity;
  }
  void set_conaffinity(int value) {
    ptr_->conaffinity = value;
  }
  int condim() const {
    return ptr_->condim;
  }
  void set_condim(int value) {
    ptr_->condim = value;
  }
  int priority() const {
    return ptr_->priority;
  }
  void set_priority(int value) {
    ptr_->priority = value;
  }
  emscripten::val friction() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->friction));
  }
  double solmix() const {
    return ptr_->solmix;
  }
  void set_solmix(double value) {
    ptr_->solmix = value;
  }
  emscripten::val solref() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solref));
  }
  emscripten::val solimp() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->solimp));
  }
  double margin() const {
    return ptr_->margin;
  }
  void set_margin(double value) {
    ptr_->margin = value;
  }
  double gap() const {
    return ptr_->gap;
  }
  void set_gap(double value) {
    ptr_->gap = value;
  }
  double mass() const {
    return ptr_->mass;
  }
  void set_mass(double value) {
    ptr_->mass = value;
  }
  double density() const {
    return ptr_->density;
  }
  void set_density(double value) {
    ptr_->density = value;
  }
  mjtGeomInertia typeinertia() const {
    return ptr_->typeinertia;
  }
  void set_typeinertia(mjtGeomInertia value) {
    ptr_->typeinertia = value;
  }
  mjtNum fluid_ellipsoid() const {
    return ptr_->fluid_ellipsoid;
  }
  void set_fluid_ellipsoid(mjtNum value) {
    ptr_->fluid_ellipsoid = value;
  }
  emscripten::val fluid_coefs() const {
    return emscripten::val(emscripten::typed_memory_view(5, ptr_->fluid_coefs));
  }
  mjString material() const {
    return (ptr_ && ptr_->material) ? *(ptr_->material) : "";
  }
  void set_material(const mjString& value) {
    if (ptr_ && ptr_->material) {
      *(ptr_->material) = value;
    }
  }
  emscripten::val rgba() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rgba));
  }
  int group() const {
    return ptr_->group;
  }
  void set_group(int value) {
    ptr_->group = value;
  }
  mjString hfieldname() const {
    return (ptr_ && ptr_->hfieldname) ? *(ptr_->hfieldname) : "";
  }
  void set_hfieldname(const mjString& value) {
    if (ptr_ && ptr_->hfieldname) {
      *(ptr_->hfieldname) = value;
    }
  }
  mjString meshname() const {
    return (ptr_ && ptr_->meshname) ? *(ptr_->meshname) : "";
  }
  void set_meshname(const mjString& value) {
    if (ptr_ && ptr_->meshname) {
      *(ptr_->meshname) = value;
    }
  }
  double fitscale() const {
    return ptr_->fitscale;
  }
  void set_fitscale(double value) {
    ptr_->fitscale = value;
  }
  mjDoubleVec &userdata() const {
    return *(ptr_->userdata);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsGeom* ptr_;

 public:
  MjsElement element;
  MjsOrientation alt;
  MjsPlugin plugin;
};

struct MjsMesh {
  explicit MjsMesh(mjsMesh *ptr);
  mjsMesh* get() const;
  void set(mjsMesh* ptr);
  mjString content_type() const {
    return (ptr_ && ptr_->content_type) ? *(ptr_->content_type) : "";
  }
  void set_content_type(const mjString& value) {
    if (ptr_ && ptr_->content_type) {
      *(ptr_->content_type) = value;
    }
  }
  mjString file() const {
    return (ptr_ && ptr_->file) ? *(ptr_->file) : "";
  }
  void set_file(const mjString& value) {
    if (ptr_ && ptr_->file) {
      *(ptr_->file) = value;
    }
  }
  emscripten::val refpos() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->refpos));
  }
  emscripten::val refquat() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->refquat));
  }
  emscripten::val scale() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->scale));
  }
  mjtMeshInertia inertia() const {
    return ptr_->inertia;
  }
  void set_inertia(mjtMeshInertia value) {
    ptr_->inertia = value;
  }
  mjtByte smoothnormal() const {
    return ptr_->smoothnormal;
  }
  void set_smoothnormal(mjtByte value) {
    ptr_->smoothnormal = value;
  }
  mjtByte needsdf() const {
    return ptr_->needsdf;
  }
  void set_needsdf(mjtByte value) {
    ptr_->needsdf = value;
  }
  int maxhullvert() const {
    return ptr_->maxhullvert;
  }
  void set_maxhullvert(int value) {
    ptr_->maxhullvert = value;
  }
  mjFloatVec &uservert() const {
    return *(ptr_->uservert);
  }
  mjFloatVec &usernormal() const {
    return *(ptr_->usernormal);
  }
  mjFloatVec &usertexcoord() const {
    return *(ptr_->usertexcoord);
  }
  mjIntVec &userface() const {
    return *(ptr_->userface);
  }
  mjIntVec &userfacenormal() const {
    return *(ptr_->userfacenormal);
  }
  mjIntVec &userfacetexcoord() const {
    return *(ptr_->userfacetexcoord);
  }
  mjString material() const {
    return (ptr_ && ptr_->material) ? *(ptr_->material) : "";
  }
  void set_material(const mjString& value) {
    if (ptr_ && ptr_->material) {
      *(ptr_->material) = value;
    }
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsMesh* ptr_;

 public:
  MjsElement element;
  MjsPlugin plugin;
};

struct MjsSensor {
  explicit MjsSensor(mjsSensor *ptr);
  mjsSensor* get() const;
  void set(mjsSensor* ptr);
  mjtSensor type() const {
    return ptr_->type;
  }
  void set_type(mjtSensor value) {
    ptr_->type = value;
  }
  mjtObj objtype() const {
    return ptr_->objtype;
  }
  void set_objtype(mjtObj value) {
    ptr_->objtype = value;
  }
  mjString objname() const {
    return (ptr_ && ptr_->objname) ? *(ptr_->objname) : "";
  }
  void set_objname(const mjString& value) {
    if (ptr_ && ptr_->objname) {
      *(ptr_->objname) = value;
    }
  }
  mjtObj reftype() const {
    return ptr_->reftype;
  }
  void set_reftype(mjtObj value) {
    ptr_->reftype = value;
  }
  mjString refname() const {
    return (ptr_ && ptr_->refname) ? *(ptr_->refname) : "";
  }
  void set_refname(const mjString& value) {
    if (ptr_ && ptr_->refname) {
      *(ptr_->refname) = value;
    }
  }
  emscripten::val intprm() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->intprm));
  }
  mjtDataType datatype() const {
    return ptr_->datatype;
  }
  void set_datatype(mjtDataType value) {
    ptr_->datatype = value;
  }
  mjtStage needstage() const {
    return ptr_->needstage;
  }
  void set_needstage(mjtStage value) {
    ptr_->needstage = value;
  }
  int dim() const {
    return ptr_->dim;
  }
  void set_dim(int value) {
    ptr_->dim = value;
  }
  double cutoff() const {
    return ptr_->cutoff;
  }
  void set_cutoff(double value) {
    ptr_->cutoff = value;
  }
  double noise() const {
    return ptr_->noise;
  }
  void set_noise(double value) {
    ptr_->noise = value;
  }
  mjDoubleVec &userdata() const {
    return *(ptr_->userdata);
  }
  mjString info() const {
    return (ptr_ && ptr_->info) ? *(ptr_->info) : "";
  }
  void set_info(const mjString& value) {
    if (ptr_ && ptr_->info) {
      *(ptr_->info) = value;
    }
  }

 private:
  mjsSensor* ptr_;

 public:
  MjsElement element;
  MjsPlugin plugin;
};

struct MjData {
  ~MjData();
  MjData(MjModel *m);
  explicit MjData(const MjModel &, const MjData &);
  std::vector<MjContact> contact() const;
  std::unique_ptr<MjData> copy();
  mjData* get() const;
  void set(mjData* ptr);
  mjtSize narena() const {
    return ptr_->narena;
  }
  void set_narena(mjtSize value) {
    ptr_->narena = value;
  }
  mjtSize nbuffer() const {
    return ptr_->nbuffer;
  }
  void set_nbuffer(mjtSize value) {
    ptr_->nbuffer = value;
  }
  int nplugin() const {
    return ptr_->nplugin;
  }
  void set_nplugin(int value) {
    ptr_->nplugin = value;
  }
  size_t pstack() const {
    return ptr_->pstack;
  }
  void set_pstack(size_t value) {
    ptr_->pstack = value;
  }
  size_t pbase() const {
    return ptr_->pbase;
  }
  void set_pbase(size_t value) {
    ptr_->pbase = value;
  }
  size_t parena() const {
    return ptr_->parena;
  }
  void set_parena(size_t value) {
    ptr_->parena = value;
  }
  mjtSize maxuse_stack() const {
    return ptr_->maxuse_stack;
  }
  void set_maxuse_stack(mjtSize value) {
    ptr_->maxuse_stack = value;
  }
  emscripten::val maxuse_threadstack() const {
    return emscripten::val(emscripten::typed_memory_view(128, ptr_->maxuse_threadstack));
  }
  mjtSize maxuse_arena() const {
    return ptr_->maxuse_arena;
  }
  void set_maxuse_arena(mjtSize value) {
    ptr_->maxuse_arena = value;
  }
  int maxuse_con() const {
    return ptr_->maxuse_con;
  }
  void set_maxuse_con(int value) {
    ptr_->maxuse_con = value;
  }
  int maxuse_efc() const {
    return ptr_->maxuse_efc;
  }
  void set_maxuse_efc(int value) {
    ptr_->maxuse_efc = value;
  }
  emscripten::val solver_niter() const {
    return emscripten::val(emscripten::typed_memory_view(20, ptr_->solver_niter));
  }
  emscripten::val solver_nnz() const {
    return emscripten::val(emscripten::typed_memory_view(20, ptr_->solver_nnz));
  }
  emscripten::val solver_fwdinv() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solver_fwdinv));
  }
  int ncon() const {
    return ptr_->ncon;
  }
  void set_ncon(int value) {
    ptr_->ncon = value;
  }
  int ne() const {
    return ptr_->ne;
  }
  void set_ne(int value) {
    ptr_->ne = value;
  }
  int nf() const {
    return ptr_->nf;
  }
  void set_nf(int value) {
    ptr_->nf = value;
  }
  int nl() const {
    return ptr_->nl;
  }
  void set_nl(int value) {
    ptr_->nl = value;
  }
  int nefc() const {
    return ptr_->nefc;
  }
  void set_nefc(int value) {
    ptr_->nefc = value;
  }
  int nJ() const {
    return ptr_->nJ;
  }
  void set_nJ(int value) {
    ptr_->nJ = value;
  }
  int nA() const {
    return ptr_->nA;
  }
  void set_nA(int value) {
    ptr_->nA = value;
  }
  int nisland() const {
    return ptr_->nisland;
  }
  void set_nisland(int value) {
    ptr_->nisland = value;
  }
  int nidof() const {
    return ptr_->nidof;
  }
  void set_nidof(int value) {
    ptr_->nidof = value;
  }
  int ntree_awake() const {
    return ptr_->ntree_awake;
  }
  void set_ntree_awake(int value) {
    ptr_->ntree_awake = value;
  }
  int nbody_awake() const {
    return ptr_->nbody_awake;
  }
  void set_nbody_awake(int value) {
    ptr_->nbody_awake = value;
  }
  int nparent_awake() const {
    return ptr_->nparent_awake;
  }
  void set_nparent_awake(int value) {
    ptr_->nparent_awake = value;
  }
  int nv_awake() const {
    return ptr_->nv_awake;
  }
  void set_nv_awake(int value) {
    ptr_->nv_awake = value;
  }
  mjtNum time() const {
    return ptr_->time;
  }
  void set_time(mjtNum value) {
    ptr_->time = value;
  }
  emscripten::val energy() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->energy));
  }
  emscripten::val buffer() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbuffer, static_cast<uint8_t*>(ptr_->buffer)));
  }
  emscripten::val arena() const {
    return emscripten::val(emscripten::typed_memory_view(model->narena, static_cast<uint8_t*>(ptr_->arena)));
  }
  emscripten::val qpos() const {
    return emscripten::val(emscripten::typed_memory_view(model->nq, ptr_->qpos));
  }
  emscripten::val qvel() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qvel));
  }
  emscripten::val act() const {
    return emscripten::val(emscripten::typed_memory_view(model->na, ptr_->act));
  }
  emscripten::val qacc_warmstart() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qacc_warmstart));
  }
  emscripten::val plugin_state() const {
    return emscripten::val(emscripten::typed_memory_view(model->npluginstate, ptr_->plugin_state));
  }
  emscripten::val ctrl() const {
    return emscripten::val(emscripten::typed_memory_view(model->nu, ptr_->ctrl));
  }
  emscripten::val qfrc_applied() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_applied));
  }
  emscripten::val xfrc_applied() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 6, ptr_->xfrc_applied));
  }
  emscripten::val eq_active() const {
    return emscripten::val(emscripten::typed_memory_view(model->neq, ptr_->eq_active));
  }
  emscripten::val mocap_pos() const {
    return emscripten::val(emscripten::typed_memory_view(model->nmocap * 3, ptr_->mocap_pos));
  }
  emscripten::val mocap_quat() const {
    return emscripten::val(emscripten::typed_memory_view(model->nmocap * 4, ptr_->mocap_quat));
  }
  emscripten::val qacc() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qacc));
  }
  emscripten::val act_dot() const {
    return emscripten::val(emscripten::typed_memory_view(model->na, ptr_->act_dot));
  }
  emscripten::val userdata() const {
    return emscripten::val(emscripten::typed_memory_view(model->nuserdata, ptr_->userdata));
  }
  emscripten::val sensordata() const {
    return emscripten::val(emscripten::typed_memory_view(model->nsensordata, ptr_->sensordata));
  }
  emscripten::val tree_asleep() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntree, ptr_->tree_asleep));
  }
  emscripten::val plugin() const {
    return emscripten::val(emscripten::typed_memory_view(model->nplugin, ptr_->plugin));
  }
  emscripten::val plugin_data() const {
    return emscripten::val(emscripten::typed_memory_view(model->nplugin, ptr_->plugin_data));
  }
  emscripten::val xpos() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 3, ptr_->xpos));
  }
  emscripten::val xquat() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 4, ptr_->xquat));
  }
  emscripten::val xmat() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 9, ptr_->xmat));
  }
  emscripten::val xipos() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 3, ptr_->xipos));
  }
  emscripten::val ximat() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 9, ptr_->ximat));
  }
  emscripten::val xanchor() const {
    return emscripten::val(emscripten::typed_memory_view(model->njnt * 3, ptr_->xanchor));
  }
  emscripten::val xaxis() const {
    return emscripten::val(emscripten::typed_memory_view(model->njnt * 3, ptr_->xaxis));
  }
  emscripten::val geom_xpos() const {
    return emscripten::val(emscripten::typed_memory_view(model->ngeom * 3, ptr_->geom_xpos));
  }
  emscripten::val geom_xmat() const {
    return emscripten::val(emscripten::typed_memory_view(model->ngeom * 9, ptr_->geom_xmat));
  }
  emscripten::val site_xpos() const {
    return emscripten::val(emscripten::typed_memory_view(model->nsite * 3, ptr_->site_xpos));
  }
  emscripten::val site_xmat() const {
    return emscripten::val(emscripten::typed_memory_view(model->nsite * 9, ptr_->site_xmat));
  }
  emscripten::val cam_xpos() const {
    return emscripten::val(emscripten::typed_memory_view(model->ncam * 3, ptr_->cam_xpos));
  }
  emscripten::val cam_xmat() const {
    return emscripten::val(emscripten::typed_memory_view(model->ncam * 9, ptr_->cam_xmat));
  }
  emscripten::val light_xpos() const {
    return emscripten::val(emscripten::typed_memory_view(model->nlight * 3, ptr_->light_xpos));
  }
  emscripten::val light_xdir() const {
    return emscripten::val(emscripten::typed_memory_view(model->nlight * 3, ptr_->light_xdir));
  }
  emscripten::val subtree_com() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 3, ptr_->subtree_com));
  }
  emscripten::val cdof() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv * 6, ptr_->cdof));
  }
  emscripten::val cinert() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 10, ptr_->cinert));
  }
  emscripten::val flexvert_xpos() const {
    return emscripten::val(emscripten::typed_memory_view(model->nflexvert * 3, ptr_->flexvert_xpos));
  }
  emscripten::val flexelem_aabb() const {
    return emscripten::val(emscripten::typed_memory_view(model->nflexelem * 6, ptr_->flexelem_aabb));
  }
  emscripten::val flexedge_J_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(model->nflexedge, ptr_->flexedge_J_rownnz));
  }
  emscripten::val flexedge_J_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(model->nflexedge, ptr_->flexedge_J_rowadr));
  }
  emscripten::val flexedge_J_colind() const {
    return emscripten::val(emscripten::typed_memory_view(model->nflexedge * model->nv, ptr_->flexedge_J_colind));
  }
  emscripten::val flexedge_J() const {
    return emscripten::val(emscripten::typed_memory_view(model->nflexedge * model->nv, ptr_->flexedge_J));
  }
  emscripten::val flexedge_length() const {
    return emscripten::val(emscripten::typed_memory_view(model->nflexedge, ptr_->flexedge_length));
  }
  emscripten::val bvh_aabb_dyn() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbvhdynamic * 6, ptr_->bvh_aabb_dyn));
  }
  emscripten::val ten_wrapadr() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntendon, ptr_->ten_wrapadr));
  }
  emscripten::val ten_wrapnum() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntendon, ptr_->ten_wrapnum));
  }
  emscripten::val ten_J_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntendon, ptr_->ten_J_rownnz));
  }
  emscripten::val ten_J_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntendon, ptr_->ten_J_rowadr));
  }
  emscripten::val ten_J_colind() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntendon * model->nv, ptr_->ten_J_colind));
  }
  emscripten::val ten_J() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntendon * model->nv, ptr_->ten_J));
  }
  emscripten::val ten_length() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntendon, ptr_->ten_length));
  }
  emscripten::val wrap_obj() const {
    return emscripten::val(emscripten::typed_memory_view(model->nwrap * 2, ptr_->wrap_obj));
  }
  emscripten::val wrap_xpos() const {
    return emscripten::val(emscripten::typed_memory_view(model->nwrap * 6, ptr_->wrap_xpos));
  }
  emscripten::val actuator_length() const {
    return emscripten::val(emscripten::typed_memory_view(model->nu, ptr_->actuator_length));
  }
  emscripten::val moment_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(model->nu, ptr_->moment_rownnz));
  }
  emscripten::val moment_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(model->nu, ptr_->moment_rowadr));
  }
  emscripten::val moment_colind() const {
    return emscripten::val(emscripten::typed_memory_view(model->nJmom, ptr_->moment_colind));
  }
  emscripten::val actuator_moment() const {
    return emscripten::val(emscripten::typed_memory_view(model->nJmom, ptr_->actuator_moment));
  }
  emscripten::val crb() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 10, ptr_->crb));
  }
  emscripten::val qM() const {
    return emscripten::val(emscripten::typed_memory_view(model->nM, ptr_->qM));
  }
  emscripten::val M() const {
    return emscripten::val(emscripten::typed_memory_view(model->nC, ptr_->M));
  }
  emscripten::val qLD() const {
    return emscripten::val(emscripten::typed_memory_view(model->nC, ptr_->qLD));
  }
  emscripten::val qLDiagInv() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qLDiagInv));
  }
  emscripten::val bvh_active() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbvh, ptr_->bvh_active));
  }
  emscripten::val tree_awake() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntree, ptr_->tree_awake));
  }
  emscripten::val body_awake() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody, ptr_->body_awake));
  }
  emscripten::val body_awake_ind() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody, ptr_->body_awake_ind));
  }
  emscripten::val parent_awake_ind() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody, ptr_->parent_awake_ind));
  }
  emscripten::val dof_awake_ind() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->dof_awake_ind));
  }
  emscripten::val flexedge_velocity() const {
    return emscripten::val(emscripten::typed_memory_view(model->nflexedge, ptr_->flexedge_velocity));
  }
  emscripten::val ten_velocity() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntendon, ptr_->ten_velocity));
  }
  emscripten::val actuator_velocity() const {
    return emscripten::val(emscripten::typed_memory_view(model->nu, ptr_->actuator_velocity));
  }
  emscripten::val cvel() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 6, ptr_->cvel));
  }
  emscripten::val cdof_dot() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv * 6, ptr_->cdof_dot));
  }
  emscripten::val qfrc_bias() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_bias));
  }
  emscripten::val qfrc_spring() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_spring));
  }
  emscripten::val qfrc_damper() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_damper));
  }
  emscripten::val qfrc_gravcomp() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_gravcomp));
  }
  emscripten::val qfrc_fluid() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_fluid));
  }
  emscripten::val qfrc_passive() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_passive));
  }
  emscripten::val subtree_linvel() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 3, ptr_->subtree_linvel));
  }
  emscripten::val subtree_angmom() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 3, ptr_->subtree_angmom));
  }
  emscripten::val qH() const {
    return emscripten::val(emscripten::typed_memory_view(model->nC, ptr_->qH));
  }
  emscripten::val qHDiagInv() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qHDiagInv));
  }
  emscripten::val qDeriv() const {
    return emscripten::val(emscripten::typed_memory_view(model->nD, ptr_->qDeriv));
  }
  emscripten::val qLU() const {
    return emscripten::val(emscripten::typed_memory_view(model->nD, ptr_->qLU));
  }
  emscripten::val actuator_force() const {
    return emscripten::val(emscripten::typed_memory_view(model->nu, ptr_->actuator_force));
  }
  emscripten::val qfrc_actuator() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_actuator));
  }
  emscripten::val qfrc_smooth() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_smooth));
  }
  emscripten::val qacc_smooth() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qacc_smooth));
  }
  emscripten::val qfrc_constraint() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_constraint));
  }
  emscripten::val qfrc_inverse() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->qfrc_inverse));
  }
  emscripten::val cacc() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 6, ptr_->cacc));
  }
  emscripten::val cfrc_int() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 6, ptr_->cfrc_int));
  }
  emscripten::val cfrc_ext() const {
    return emscripten::val(emscripten::typed_memory_view(model->nbody * 6, ptr_->cfrc_ext));
  }
  // contact field is handled manually in template file struct declaration
  emscripten::val efc_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_type));
  }
  emscripten::val efc_id() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_id));
  }
  emscripten::val efc_J_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_J_rownnz));
  }
  emscripten::val efc_J_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_J_rowadr));
  }
  emscripten::val efc_J_rowsuper() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_J_rowsuper));
  }
  emscripten::val efc_J_colind() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nJ, ptr_->efc_J_colind));
  }
  emscripten::val efc_J() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nJ, ptr_->efc_J));
  }
  emscripten::val efc_pos() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_pos));
  }
  emscripten::val efc_margin() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_margin));
  }
  emscripten::val efc_frictionloss() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_frictionloss));
  }
  emscripten::val efc_diagApprox() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_diagApprox));
  }
  emscripten::val efc_KBIP() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc * 4, ptr_->efc_KBIP));
  }
  emscripten::val efc_D() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_D));
  }
  emscripten::val efc_R() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_R));
  }
  emscripten::val tendon_efcadr() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntendon, ptr_->tendon_efcadr));
  }
  emscripten::val tree_island() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntree, ptr_->tree_island));
  }
  emscripten::val island_ntree() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nisland, ptr_->island_ntree));
  }
  emscripten::val island_itreeadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nisland, ptr_->island_itreeadr));
  }
  emscripten::val map_itree2tree() const {
    return emscripten::val(emscripten::typed_memory_view(model->ntree, ptr_->map_itree2tree));
  }
  emscripten::val dof_island() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->dof_island));
  }
  emscripten::val island_nv() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nisland, ptr_->island_nv));
  }
  emscripten::val island_idofadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nisland, ptr_->island_idofadr));
  }
  emscripten::val island_dofadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nisland, ptr_->island_dofadr));
  }
  emscripten::val map_dof2idof() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->map_dof2idof));
  }
  emscripten::val map_idof2dof() const {
    return emscripten::val(emscripten::typed_memory_view(model->nv, ptr_->map_idof2dof));
  }
  emscripten::val ifrc_smooth() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nidof, ptr_->ifrc_smooth));
  }
  emscripten::val iacc_smooth() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nidof, ptr_->iacc_smooth));
  }
  emscripten::val iM_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nidof, ptr_->iM_rownnz));
  }
  emscripten::val iM_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nidof, ptr_->iM_rowadr));
  }
  emscripten::val iM_colind() const {
    return emscripten::val(emscripten::typed_memory_view(model->nC, ptr_->iM_colind));
  }
  emscripten::val iM() const {
    return emscripten::val(emscripten::typed_memory_view(model->nC, ptr_->iM));
  }
  emscripten::val iLD() const {
    return emscripten::val(emscripten::typed_memory_view(model->nC, ptr_->iLD));
  }
  emscripten::val iLDiagInv() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nidof, ptr_->iLDiagInv));
  }
  emscripten::val iacc() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nidof, ptr_->iacc));
  }
  emscripten::val efc_island() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_island));
  }
  emscripten::val island_ne() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nisland, ptr_->island_ne));
  }
  emscripten::val island_nf() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nisland, ptr_->island_nf));
  }
  emscripten::val island_nefc() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nisland, ptr_->island_nefc));
  }
  emscripten::val island_iefcadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nisland, ptr_->island_iefcadr));
  }
  emscripten::val map_efc2iefc() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->map_efc2iefc));
  }
  emscripten::val map_iefc2efc() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->map_iefc2efc));
  }
  emscripten::val iefc_type() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_type));
  }
  emscripten::val iefc_id() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_id));
  }
  emscripten::val iefc_J_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_J_rownnz));
  }
  emscripten::val iefc_J_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_J_rowadr));
  }
  emscripten::val iefc_J_rowsuper() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_J_rowsuper));
  }
  emscripten::val iefc_J_colind() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nJ, ptr_->iefc_J_colind));
  }
  emscripten::val iefc_J() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nJ, ptr_->iefc_J));
  }
  emscripten::val iefc_frictionloss() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_frictionloss));
  }
  emscripten::val iefc_D() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_D));
  }
  emscripten::val iefc_R() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_R));
  }
  emscripten::val efc_AR_rownnz() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_AR_rownnz));
  }
  emscripten::val efc_AR_rowadr() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_AR_rowadr));
  }
  emscripten::val efc_AR_colind() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nA, ptr_->efc_AR_colind));
  }
  emscripten::val efc_AR() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nA, ptr_->efc_AR));
  }
  emscripten::val efc_vel() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_vel));
  }
  emscripten::val efc_aref() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_aref));
  }
  emscripten::val efc_b() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_b));
  }
  emscripten::val iefc_aref() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_aref));
  }
  emscripten::val iefc_state() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_state));
  }
  emscripten::val iefc_force() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->iefc_force));
  }
  emscripten::val efc_state() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_state));
  }
  emscripten::val efc_force() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nefc, ptr_->efc_force));
  }
  emscripten::val ifrc_constraint() const {
    return emscripten::val(emscripten::typed_memory_view(ptr_->nidof, ptr_->ifrc_constraint));
  }
  uintptr_t threadpool() const {
    return ptr_->threadpool;
  }
  void set_threadpool(uintptr_t value) {
    ptr_->threadpool = value;
  }
  uint64_t signature() const {
    return ptr_->signature;
  }
  void set_signature(uint64_t value) {
    ptr_->signature = value;
  }

 private:
  mjData* ptr_;

 public:
  std::vector<MjSolverStat> solver;
  std::vector<MjWarningStat> warning;
  std::vector<MjTimerStat> timer;
  mjModel *model;
};

struct MjsDefault {
  explicit MjsDefault(mjsDefault *ptr);
  mjsDefault* get() const;
  void set(mjsDefault* ptr);

 private:
  mjsDefault* ptr_;

 public:
  MjsElement element;
  MjsJoint joint;
  MjsGeom geom;
  MjsSite site;
  MjsCamera camera;
  MjsLight light;
  MjsFlex flex;
  MjsMesh mesh;
  MjsMaterial material;
  MjsPair pair;
  MjsEquality equality;
  MjsTendon tendon;
  MjsActuator actuator;
};


MjContact::MjContact(mjContact *ptr) : ptr_(ptr) {}
MjContact::~MjContact() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjContact::MjContact() : ptr_(new mjContact) {
  owned_ = true;
}
MjContact::MjContact(const MjContact &other) : MjContact() {
  *ptr_ = *other.get();
}
MjContact& MjContact::operator=(const MjContact &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjContact> MjContact::copy() {
  return std::make_unique<MjContact>(*this);
}
mjContact* MjContact::get() const {
  return ptr_;
}
void MjContact::set(mjContact* ptr) {
  ptr_ = ptr;
}

MjLROpt::MjLROpt(mjLROpt *ptr) : ptr_(ptr) {}
MjLROpt::~MjLROpt() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjLROpt::MjLROpt() : ptr_(new mjLROpt) {
  owned_ = true;
  mj_defaultLROpt(ptr_);
}
MjLROpt::MjLROpt(const MjLROpt &other) : MjLROpt() {
  *ptr_ = *other.get();
}
MjLROpt& MjLROpt::operator=(const MjLROpt &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjLROpt> MjLROpt::copy() {
  return std::make_unique<MjLROpt>(*this);
}
mjLROpt* MjLROpt::get() const {
  return ptr_;
}
void MjLROpt::set(mjLROpt* ptr) {
  ptr_ = ptr;
}

MjOption::MjOption(mjOption *ptr) : ptr_(ptr) {}
MjOption::~MjOption() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjOption::MjOption() : ptr_(new mjOption) {
  owned_ = true;
  mj_defaultOption(ptr_);
}
MjOption::MjOption(const MjOption &other) : MjOption() {
  *ptr_ = *other.get();
}
MjOption& MjOption::operator=(const MjOption &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjOption> MjOption::copy() {
  return std::make_unique<MjOption>(*this);
}
mjOption* MjOption::get() const {
  return ptr_;
}
void MjOption::set(mjOption* ptr) {
  ptr_ = ptr;
}

MjSolverStat::MjSolverStat(mjSolverStat *ptr) : ptr_(ptr) {}
MjSolverStat::~MjSolverStat() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjSolverStat::MjSolverStat() : ptr_(new mjSolverStat) {
  owned_ = true;
}
MjSolverStat::MjSolverStat(const MjSolverStat &other) : MjSolverStat() {
  *ptr_ = *other.get();
}
MjSolverStat& MjSolverStat::operator=(const MjSolverStat &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjSolverStat> MjSolverStat::copy() {
  return std::make_unique<MjSolverStat>(*this);
}
mjSolverStat* MjSolverStat::get() const {
  return ptr_;
}
void MjSolverStat::set(mjSolverStat* ptr) {
  ptr_ = ptr;
}

MjStatistic::MjStatistic(mjStatistic *ptr) : ptr_(ptr) {}
MjStatistic::~MjStatistic() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjStatistic::MjStatistic() : ptr_(new mjStatistic) {
  owned_ = true;
}
MjStatistic::MjStatistic(const MjStatistic &other) : MjStatistic() {
  *ptr_ = *other.get();
}
MjStatistic& MjStatistic::operator=(const MjStatistic &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjStatistic> MjStatistic::copy() {
  return std::make_unique<MjStatistic>(*this);
}
mjStatistic* MjStatistic::get() const {
  return ptr_;
}
void MjStatistic::set(mjStatistic* ptr) {
  ptr_ = ptr;
}

MjTimerStat::MjTimerStat(mjTimerStat *ptr) : ptr_(ptr) {}
MjTimerStat::~MjTimerStat() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjTimerStat::MjTimerStat() : ptr_(new mjTimerStat) {
  owned_ = true;
}
MjTimerStat::MjTimerStat(const MjTimerStat &other) : MjTimerStat() {
  *ptr_ = *other.get();
}
MjTimerStat& MjTimerStat::operator=(const MjTimerStat &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjTimerStat> MjTimerStat::copy() {
  return std::make_unique<MjTimerStat>(*this);
}
mjTimerStat* MjTimerStat::get() const {
  return ptr_;
}
void MjTimerStat::set(mjTimerStat* ptr) {
  ptr_ = ptr;
}

MjVisualGlobal::MjVisualGlobal(mjVisualGlobal *ptr) : ptr_(ptr) {}
MjVisualGlobal::~MjVisualGlobal() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjVisualGlobal::MjVisualGlobal() : ptr_(new mjVisualGlobal) {
  owned_ = true;
}
MjVisualGlobal::MjVisualGlobal(const MjVisualGlobal &other) : MjVisualGlobal() {
  *ptr_ = *other.get();
}
MjVisualGlobal& MjVisualGlobal::operator=(const MjVisualGlobal &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjVisualGlobal> MjVisualGlobal::copy() {
  return std::make_unique<MjVisualGlobal>(*this);
}
mjVisualGlobal* MjVisualGlobal::get() const {
  return ptr_;
}
void MjVisualGlobal::set(mjVisualGlobal* ptr) {
  ptr_ = ptr;
}

MjVisualHeadlight::MjVisualHeadlight(mjVisualHeadlight *ptr) : ptr_(ptr) {}
MjVisualHeadlight::~MjVisualHeadlight() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjVisualHeadlight::MjVisualHeadlight() : ptr_(new mjVisualHeadlight) {
  owned_ = true;
}
MjVisualHeadlight::MjVisualHeadlight(const MjVisualHeadlight &other) : MjVisualHeadlight() {
  *ptr_ = *other.get();
}
MjVisualHeadlight& MjVisualHeadlight::operator=(const MjVisualHeadlight &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjVisualHeadlight> MjVisualHeadlight::copy() {
  return std::make_unique<MjVisualHeadlight>(*this);
}
mjVisualHeadlight* MjVisualHeadlight::get() const {
  return ptr_;
}
void MjVisualHeadlight::set(mjVisualHeadlight* ptr) {
  ptr_ = ptr;
}

MjVisualMap::MjVisualMap(mjVisualMap *ptr) : ptr_(ptr) {}
MjVisualMap::~MjVisualMap() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjVisualMap::MjVisualMap() : ptr_(new mjVisualMap) {
  owned_ = true;
}
MjVisualMap::MjVisualMap(const MjVisualMap &other) : MjVisualMap() {
  *ptr_ = *other.get();
}
MjVisualMap& MjVisualMap::operator=(const MjVisualMap &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjVisualMap> MjVisualMap::copy() {
  return std::make_unique<MjVisualMap>(*this);
}
mjVisualMap* MjVisualMap::get() const {
  return ptr_;
}
void MjVisualMap::set(mjVisualMap* ptr) {
  ptr_ = ptr;
}

MjVisualQuality::MjVisualQuality(mjVisualQuality *ptr) : ptr_(ptr) {}
MjVisualQuality::~MjVisualQuality() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjVisualQuality::MjVisualQuality() : ptr_(new mjVisualQuality) {
  owned_ = true;
}
MjVisualQuality::MjVisualQuality(const MjVisualQuality &other) : MjVisualQuality() {
  *ptr_ = *other.get();
}
MjVisualQuality& MjVisualQuality::operator=(const MjVisualQuality &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjVisualQuality> MjVisualQuality::copy() {
  return std::make_unique<MjVisualQuality>(*this);
}
mjVisualQuality* MjVisualQuality::get() const {
  return ptr_;
}
void MjVisualQuality::set(mjVisualQuality* ptr) {
  ptr_ = ptr;
}

MjVisualRgba::MjVisualRgba(mjVisualRgba *ptr) : ptr_(ptr) {}
MjVisualRgba::~MjVisualRgba() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjVisualRgba::MjVisualRgba() : ptr_(new mjVisualRgba) {
  owned_ = true;
}
MjVisualRgba::MjVisualRgba(const MjVisualRgba &other) : MjVisualRgba() {
  *ptr_ = *other.get();
}
MjVisualRgba& MjVisualRgba::operator=(const MjVisualRgba &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjVisualRgba> MjVisualRgba::copy() {
  return std::make_unique<MjVisualRgba>(*this);
}
mjVisualRgba* MjVisualRgba::get() const {
  return ptr_;
}
void MjVisualRgba::set(mjVisualRgba* ptr) {
  ptr_ = ptr;
}

MjVisualScale::MjVisualScale(mjVisualScale *ptr) : ptr_(ptr) {}
MjVisualScale::~MjVisualScale() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjVisualScale::MjVisualScale() : ptr_(new mjVisualScale) {
  owned_ = true;
}
MjVisualScale::MjVisualScale(const MjVisualScale &other) : MjVisualScale() {
  *ptr_ = *other.get();
}
MjVisualScale& MjVisualScale::operator=(const MjVisualScale &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjVisualScale> MjVisualScale::copy() {
  return std::make_unique<MjVisualScale>(*this);
}
mjVisualScale* MjVisualScale::get() const {
  return ptr_;
}
void MjVisualScale::set(mjVisualScale* ptr) {
  ptr_ = ptr;
}

MjWarningStat::MjWarningStat(mjWarningStat *ptr) : ptr_(ptr) {}
MjWarningStat::~MjWarningStat() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjWarningStat::MjWarningStat() : ptr_(new mjWarningStat) {
  owned_ = true;
}
MjWarningStat::MjWarningStat(const MjWarningStat &other) : MjWarningStat() {
  *ptr_ = *other.get();
}
MjWarningStat& MjWarningStat::operator=(const MjWarningStat &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjWarningStat> MjWarningStat::copy() {
  return std::make_unique<MjWarningStat>(*this);
}
mjWarningStat* MjWarningStat::get() const {
  return ptr_;
}
void MjWarningStat::set(mjWarningStat* ptr) {
  ptr_ = ptr;
}

MjsElement::MjsElement(mjsElement *ptr) : ptr_(ptr) {}
mjsElement* MjsElement::get() const {
  return ptr_;
}
void MjsElement::set(mjsElement* ptr) {
  ptr_ = ptr;
}

MjsOrientation::MjsOrientation(mjsOrientation *ptr) : ptr_(ptr) {}
mjsOrientation* MjsOrientation::get() const {
  return ptr_;
}
void MjsOrientation::set(mjsOrientation* ptr) {
  ptr_ = ptr;
}

MjvCamera::MjvCamera(mjvCamera *ptr) : ptr_(ptr) {}
MjvCamera::~MjvCamera() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjvCamera::MjvCamera() : ptr_(new mjvCamera) {
  owned_ = true;
  mjv_defaultCamera(ptr_);
}
MjvCamera::MjvCamera(const MjvCamera &other) : MjvCamera() {
  *ptr_ = *other.get();
}
MjvCamera& MjvCamera::operator=(const MjvCamera &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjvCamera> MjvCamera::copy() {
  return std::make_unique<MjvCamera>(*this);
}
mjvCamera* MjvCamera::get() const {
  return ptr_;
}
void MjvCamera::set(mjvCamera* ptr) {
  ptr_ = ptr;
}

MjvFigure::MjvFigure(mjvFigure *ptr) : ptr_(ptr) {}
MjvFigure::~MjvFigure() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjvFigure::MjvFigure() : ptr_(new mjvFigure) {
  owned_ = true;
  mjv_defaultFigure(ptr_);
}
MjvFigure::MjvFigure(const MjvFigure &other) : MjvFigure() {
  *ptr_ = *other.get();
}
MjvFigure& MjvFigure::operator=(const MjvFigure &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjvFigure> MjvFigure::copy() {
  return std::make_unique<MjvFigure>(*this);
}
mjvFigure* MjvFigure::get() const {
  return ptr_;
}
void MjvFigure::set(mjvFigure* ptr) {
  ptr_ = ptr;
}

MjvGLCamera::MjvGLCamera(mjvGLCamera *ptr) : ptr_(ptr) {}
MjvGLCamera::~MjvGLCamera() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjvGLCamera::MjvGLCamera() : ptr_(new mjvGLCamera) {
  owned_ = true;
}
MjvGLCamera::MjvGLCamera(const MjvGLCamera &other) : MjvGLCamera() {
  *ptr_ = *other.get();
}
MjvGLCamera& MjvGLCamera::operator=(const MjvGLCamera &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjvGLCamera> MjvGLCamera::copy() {
  return std::make_unique<MjvGLCamera>(*this);
}
mjvGLCamera* MjvGLCamera::get() const {
  return ptr_;
}
void MjvGLCamera::set(mjvGLCamera* ptr) {
  ptr_ = ptr;
}

MjvGeom::MjvGeom(mjvGeom *ptr) : ptr_(ptr) {}
MjvGeom::~MjvGeom() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjvGeom::MjvGeom() : ptr_(new mjvGeom) {
  owned_ = true;
  mjv_initGeom(ptr_, mjGEOM_NONE, nullptr, nullptr, nullptr, nullptr);
}
MjvGeom::MjvGeom(const MjvGeom &other) : MjvGeom() {
  *ptr_ = *other.get();
}
MjvGeom& MjvGeom::operator=(const MjvGeom &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjvGeom> MjvGeom::copy() {
  return std::make_unique<MjvGeom>(*this);
}
mjvGeom* MjvGeom::get() const {
  return ptr_;
}
void MjvGeom::set(mjvGeom* ptr) {
  ptr_ = ptr;
}

MjvLight::MjvLight(mjvLight *ptr) : ptr_(ptr) {}
MjvLight::~MjvLight() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjvLight::MjvLight() : ptr_(new mjvLight) {
  owned_ = true;
}
MjvLight::MjvLight(const MjvLight &other) : MjvLight() {
  *ptr_ = *other.get();
}
MjvLight& MjvLight::operator=(const MjvLight &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjvLight> MjvLight::copy() {
  return std::make_unique<MjvLight>(*this);
}
mjvLight* MjvLight::get() const {
  return ptr_;
}
void MjvLight::set(mjvLight* ptr) {
  ptr_ = ptr;
}

MjvOption::MjvOption(mjvOption *ptr) : ptr_(ptr) {}
MjvOption::~MjvOption() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjvOption::MjvOption() : ptr_(new mjvOption) {
  owned_ = true;
  mjv_defaultOption(ptr_);
}
MjvOption::MjvOption(const MjvOption &other) : MjvOption() {
  *ptr_ = *other.get();
}
MjvOption& MjvOption::operator=(const MjvOption &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjvOption> MjvOption::copy() {
  return std::make_unique<MjvOption>(*this);
}
mjvOption* MjvOption::get() const {
  return ptr_;
}
void MjvOption::set(mjvOption* ptr) {
  ptr_ = ptr;
}

MjvPerturb::MjvPerturb(mjvPerturb *ptr) : ptr_(ptr) {}
MjvPerturb::~MjvPerturb() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjvPerturb::MjvPerturb() : ptr_(new mjvPerturb) {
  owned_ = true;
  mjv_defaultPerturb(ptr_);
}
MjvPerturb::MjvPerturb(const MjvPerturb &other) : MjvPerturb() {
  *ptr_ = *other.get();
}
MjvPerturb& MjvPerturb::operator=(const MjvPerturb &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjvPerturb> MjvPerturb::copy() {
  return std::make_unique<MjvPerturb>(*this);
}
mjvPerturb* MjvPerturb::get() const {
  return ptr_;
}
void MjvPerturb::set(mjvPerturb* ptr) {
  ptr_ = ptr;
}

MjsCompiler::MjsCompiler(mjsCompiler *ptr) : ptr_(ptr), LRopt(&ptr_->LRopt) {}
mjsCompiler* MjsCompiler::get() const {
  return ptr_;
}
void MjsCompiler::set(mjsCompiler* ptr) {
  ptr_ = ptr;
}

MjVisual::MjVisual(mjVisual *ptr) : ptr_(ptr), global(&ptr_->global), quality(&ptr_->quality), headlight(&ptr_->headlight), map(&ptr_->map), scale(&ptr_->scale), rgba(&ptr_->rgba) {}
MjVisual::~MjVisual() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjVisual::MjVisual() : ptr_(new mjVisual), global(&ptr_->global), quality(&ptr_->quality), headlight(&ptr_->headlight), map(&ptr_->map), scale(&ptr_->scale), rgba(&ptr_->rgba) {
  owned_ = true;
  mj_defaultVisual(ptr_);
}
MjVisual::MjVisual(const MjVisual &other) : MjVisual() {
  *ptr_ = *other.get();
  global.set(&ptr_->global);
  quality.set(&ptr_->quality);
  headlight.set(&ptr_->headlight);
  map.set(&ptr_->map);
  scale.set(&ptr_->scale);
  rgba.set(&ptr_->rgba);
}
MjVisual& MjVisual::operator=(const MjVisual &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  global.set(&ptr_->global);
  quality.set(&ptr_->quality);
  headlight.set(&ptr_->headlight);
  map.set(&ptr_->map);
  scale.set(&ptr_->scale);
  rgba.set(&ptr_->rgba);
  return *this;
}
std::unique_ptr<MjVisual> MjVisual::copy() {
  return std::make_unique<MjVisual>(*this);
}
mjVisual* MjVisual::get() const {
  return ptr_;
}
void MjVisual::set(mjVisual* ptr) {
  ptr_ = ptr;
}

MjsEquality::MjsEquality(mjsEquality *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsEquality* MjsEquality::get() const {
  return ptr_;
}
void MjsEquality::set(mjsEquality* ptr) {
  ptr_ = ptr;
}

MjsExclude::MjsExclude(mjsExclude *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsExclude* MjsExclude::get() const {
  return ptr_;
}
void MjsExclude::set(mjsExclude* ptr) {
  ptr_ = ptr;
}

MjsFlex::MjsFlex(mjsFlex *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsFlex* MjsFlex::get() const {
  return ptr_;
}
void MjsFlex::set(mjsFlex* ptr) {
  ptr_ = ptr;
}

MjsHField::MjsHField(mjsHField *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsHField* MjsHField::get() const {
  return ptr_;
}
void MjsHField::set(mjsHField* ptr) {
  ptr_ = ptr;
}

MjsJoint::MjsJoint(mjsJoint *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsJoint* MjsJoint::get() const {
  return ptr_;
}
void MjsJoint::set(mjsJoint* ptr) {
  ptr_ = ptr;
}

MjsKey::MjsKey(mjsKey *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsKey* MjsKey::get() const {
  return ptr_;
}
void MjsKey::set(mjsKey* ptr) {
  ptr_ = ptr;
}

MjsLight::MjsLight(mjsLight *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsLight* MjsLight::get() const {
  return ptr_;
}
void MjsLight::set(mjsLight* ptr) {
  ptr_ = ptr;
}

MjsMaterial::MjsMaterial(mjsMaterial *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsMaterial* MjsMaterial::get() const {
  return ptr_;
}
void MjsMaterial::set(mjsMaterial* ptr) {
  ptr_ = ptr;
}

MjsNumeric::MjsNumeric(mjsNumeric *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsNumeric* MjsNumeric::get() const {
  return ptr_;
}
void MjsNumeric::set(mjsNumeric* ptr) {
  ptr_ = ptr;
}

MjsPair::MjsPair(mjsPair *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsPair* MjsPair::get() const {
  return ptr_;
}
void MjsPair::set(mjsPair* ptr) {
  ptr_ = ptr;
}

MjsPlugin::MjsPlugin(mjsPlugin *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsPlugin* MjsPlugin::get() const {
  return ptr_;
}
void MjsPlugin::set(mjsPlugin* ptr) {
  ptr_ = ptr;
}

MjsSkin::MjsSkin(mjsSkin *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsSkin* MjsSkin::get() const {
  return ptr_;
}
void MjsSkin::set(mjsSkin* ptr) {
  ptr_ = ptr;
}

MjsTendon::MjsTendon(mjsTendon *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsTendon* MjsTendon::get() const {
  return ptr_;
}
void MjsTendon::set(mjsTendon* ptr) {
  ptr_ = ptr;
}

MjsText::MjsText(mjsText *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsText* MjsText::get() const {
  return ptr_;
}
void MjsText::set(mjsText* ptr) {
  ptr_ = ptr;
}

MjsTexture::MjsTexture(mjsTexture *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsTexture* MjsTexture::get() const {
  return ptr_;
}
void MjsTexture::set(mjsTexture* ptr) {
  ptr_ = ptr;
}

MjsTuple::MjsTuple(mjsTuple *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsTuple* MjsTuple::get() const {
  return ptr_;
}
void MjsTuple::set(mjsTuple* ptr) {
  ptr_ = ptr;
}

MjsWrap::MjsWrap(mjsWrap *ptr) : ptr_(ptr), element(ptr_->element) {}
mjsWrap* MjsWrap::get() const {
  return ptr_;
}
void MjsWrap::set(mjsWrap* ptr) {
  ptr_ = ptr;
}

MjsCamera::MjsCamera(mjsCamera *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt) {}
mjsCamera* MjsCamera::get() const {
  return ptr_;
}
void MjsCamera::set(mjsCamera* ptr) {
  ptr_ = ptr;
}

MjsFrame::MjsFrame(mjsFrame *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt) {}
mjsFrame* MjsFrame::get() const {
  return ptr_;
}
void MjsFrame::set(mjsFrame* ptr) {
  ptr_ = ptr;
}

MjsSite::MjsSite(mjsSite *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt) {}
mjsSite* MjsSite::get() const {
  return ptr_;
}
void MjsSite::set(mjsSite* ptr) {
  ptr_ = ptr;
}

MjsActuator::MjsActuator(mjsActuator *ptr) : ptr_(ptr), element(ptr_->element), plugin(&ptr_->plugin) {}
mjsActuator* MjsActuator::get() const {
  return ptr_;
}
void MjsActuator::set(mjsActuator* ptr) {
  ptr_ = ptr;
}

MjsBody::MjsBody(mjsBody *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt), ialt(&ptr_->ialt), plugin(&ptr_->plugin) {}
mjsBody* MjsBody::get() const {
  return ptr_;
}
void MjsBody::set(mjsBody* ptr) {
  ptr_ = ptr;
}

MjsGeom::MjsGeom(mjsGeom *ptr) : ptr_(ptr), element(ptr_->element), alt(&ptr_->alt), plugin(&ptr_->plugin) {}
mjsGeom* MjsGeom::get() const {
  return ptr_;
}
void MjsGeom::set(mjsGeom* ptr) {
  ptr_ = ptr;
}

MjsMesh::MjsMesh(mjsMesh *ptr) : ptr_(ptr), element(ptr_->element), plugin(&ptr_->plugin) {}
mjsMesh* MjsMesh::get() const {
  return ptr_;
}
void MjsMesh::set(mjsMesh* ptr) {
  ptr_ = ptr;
}

MjsSensor::MjsSensor(mjsSensor *ptr) : ptr_(ptr), element(ptr_->element), plugin(&ptr_->plugin) {}
mjsSensor* MjsSensor::get() const {
  return ptr_;
}
void MjsSensor::set(mjsSensor* ptr) {
  ptr_ = ptr;
}

MjsDefault::MjsDefault(mjsDefault *ptr) : ptr_(ptr), element(ptr_->element), joint(ptr_->joint), geom(ptr_->geom), site(ptr_->site), camera(ptr_->camera), light(ptr_->light), flex(ptr_->flex), mesh(ptr_->mesh), material(ptr_->material), pair(ptr_->pair), equality(ptr_->equality), tendon(ptr_->tendon), actuator(ptr_->actuator) {}
mjsDefault* MjsDefault::get() const {
  return ptr_;
}
void MjsDefault::set(mjsDefault* ptr) {
  ptr_ = ptr;
}


struct MjvScene {
  MjvScene();
  MjvScene(MjModel *m, int maxgeom);
  ~MjvScene();
  std::unique_ptr<MjvScene> copy();
  int GetSumFlexFaces() const;

  mjvScene* get() const;
  void set(mjvScene* ptr);

  std::vector<MjvGeom> geoms() const;

  emscripten::val geomorder() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->ngeom, ptr_->geomorder));
  }
  emscripten::val flexedgeadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexedgeadr));
  }
  emscripten::val flexedgenum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexedgenum));
  }
  emscripten::val flexvertadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexvertadr));
  }
  emscripten::val flexvertnum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexvertnum));
  }
  emscripten::val flexfaceadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexfaceadr));
  }
  emscripten::val flexfacenum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexfacenum));
  }
  emscripten::val flexfaceused() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexfaceused));
  }
  emscripten::val flexedge() const {
    return emscripten::val(
        emscripten::typed_memory_view(2 * model->nflexedge, ptr_->flexedge));
  }
  emscripten::val flexvert() const {
    return emscripten::val(
        emscripten::typed_memory_view(3 * model->nflexvert, ptr_->flexvert));
  }
  emscripten::val skinfacenum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nskin, ptr_->skinfacenum));
  }
  emscripten::val skinvertadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nskin, ptr_->skinvertadr));
  }
  emscripten::val skinvertnum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nskin, ptr_->skinvertnum));
  }
  emscripten::val skinvert() const {
    return emscripten::val(
        emscripten::typed_memory_view(3 * model->nskinvert, ptr_->skinvert));
  }
  emscripten::val skinnormal() const {
    return emscripten::val(
        emscripten::typed_memory_view(3 * model->nskinvert, ptr_->skinnormal));
  }
  emscripten::val flexface() const {
    return emscripten::val(emscripten::typed_memory_view(
        9 * MjvScene::GetSumFlexFaces(), ptr_->flexface));
  }
  emscripten::val flexnormal() const {
    return emscripten::val(emscripten::typed_memory_view(
        9 * MjvScene::GetSumFlexFaces(), ptr_->flexnormal));
  }
  emscripten::val flextexcoord() const {
    return emscripten::val(emscripten::typed_memory_view(
        6 * MjvScene::GetSumFlexFaces(), ptr_->flextexcoord));
  }
  // camera field is handled manually in template file struct declaration
  // flexedge field is handled manually in template file struct declaration
  // flexedgeadr field is handled manually in template file struct declaration
  // flexedgenum field is handled manually in template file struct declaration
  // flexface field is handled manually in template file struct declaration
  // flexfaceadr field is handled manually in template file struct declaration
  // flexfacenum field is handled manually in template file struct declaration
  // flexfaceused field is handled manually in template file struct declaration
  // flexnormal field is handled manually in template file struct declaration
  // flextexcoord field is handled manually in template file struct declaration
  // flexvert field is handled manually in template file struct declaration
  // flexvertadr field is handled manually in template file struct declaration
  // flexvertnum field is handled manually in template file struct declaration
  // geomorder field is handled manually in template file struct declaration
  // geoms field is handled manually in template file struct declaration
  // lights field is handled manually in template file struct declaration
  // skinfacenum field is handled manually in template file struct declaration
  // skinnormal field is handled manually in template file struct declaration
  // skinvert field is handled manually in template file struct declaration
  // skinvertadr field is handled manually in template file struct declaration
  // skinvertnum field is handled manually in template file struct declaration
  emscripten::val flags() const {
    return emscripten::val(emscripten::typed_memory_view(10, ptr_->flags));
  }
  emscripten::val framergb() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->framergb));
  }
  emscripten::val rotate() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rotate));
  }
  emscripten::val translate() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->translate));
  }
  float scale() const {
    return ptr_->scale;
  }
  void set_scale(float value) {
    ptr_->scale = value;
  }
  int framewidth() const {
    return ptr_->framewidth;
  }
  void set_framewidth(int value) {
    ptr_->framewidth = value;
  }
  int maxgeom() const {
    return ptr_->maxgeom;
  }
  void set_maxgeom(int value) {
    ptr_->maxgeom = value;
  }
  int nflex() const {
    return ptr_->nflex;
  }
  void set_nflex(int value) {
    ptr_->nflex = value;
  }
  int ngeom() const {
    return ptr_->ngeom;
  }
  void set_ngeom(int value) {
    ptr_->ngeom = value;
  }
  int nlight() const {
    return ptr_->nlight;
  }
  void set_nlight(int value) {
    ptr_->nlight = value;
  }
  int nskin() const {
    return ptr_->nskin;
  }
  void set_nskin(int value) {
    ptr_->nskin = value;
  }
  int status() const {
    return ptr_->status;
  }
  void set_status(int value) {
    ptr_->status = value;
  }
  int stereo() const {
    return ptr_->stereo;
  }
  void set_stereo(int value) {
    ptr_->stereo = value;
  }
  mjtByte enabletransform() const {
    return ptr_->enabletransform;
  }
  void set_enabletransform(mjtByte value) {
    ptr_->enabletransform = value;
  }
  mjtByte flexedgeopt() const {
    return ptr_->flexedgeopt;
  }
  void set_flexedgeopt(mjtByte value) {
    ptr_->flexedgeopt = value;
  }
  mjtByte flexfaceopt() const {
    return ptr_->flexfaceopt;
  }
  void set_flexfaceopt(mjtByte value) {
    ptr_->flexfaceopt = value;
  }
  mjtByte flexskinopt() const {
    return ptr_->flexskinopt;
  }
  void set_flexskinopt(mjtByte value) {
    ptr_->flexskinopt = value;
  }
  mjtByte flexvertopt() const {
    return ptr_->flexvertopt;
  }
  void set_flexvertopt(mjtByte value) {
    ptr_->flexvertopt = value;
  }

 private:
  mjvScene* ptr_;
  bool owned_ = false;

 public:
  mjModel *model;
  std::vector<MjvLight> lights;
  std::vector<MjvGLCamera> camera;
};

MjModel::MjModel(mjModel* ptr)
    : ptr_(ptr), opt(&ptr->opt), vis(&ptr->vis), stat(&ptr->stat) {}

MjModel::MjModel(const MjModel& other)
    : ptr_(mj_copyModel(nullptr, other.get())),
      opt(&ptr_->opt),
      vis(&ptr_->vis),
      stat(&ptr_->stat) {}

MjModel::~MjModel() {
  if (ptr_) {
    mj_deleteModel(ptr_);
  }
}

mjModel* MjModel::get() const { return ptr_; }

void MjModel::set(mjModel* ptr) { ptr_ = ptr; }

MjData::MjData(MjModel* m) {
  model = m->get();
  ptr_ = mj_makeData(model);
  if (ptr_) {
    solver =
        InitWrapperArray<MjSolverStat>(get()->solver, mjNSOLVER * mjNISLAND);
    timer = InitWrapperArray<MjTimerStat>(get()->timer, mjNTIMER);
    warning = InitWrapperArray<MjWarningStat>(get()->warning, mjNWARNING);
  }
}

MjData::MjData(const MjModel& model, const MjData& other)
    : ptr_(mj_copyData(nullptr, model.get(), other.get())), model(model.get()) {
  if (ptr_) {
    solver =
        InitWrapperArray<MjSolverStat>(get()->solver, mjNSOLVER * mjNISLAND);
    timer = InitWrapperArray<MjTimerStat>(get()->timer, mjNTIMER);
    warning = InitWrapperArray<MjWarningStat>(get()->warning, mjNWARNING);
  }
}

MjData::~MjData() {
  if (ptr_) {
    mj_deleteData(ptr_);
  }
}
mjData* MjData::get() const { return ptr_; }

void MjData::set(mjData* ptr) { ptr_ = ptr; }

std::vector<MjContact> MjData::contact() const {
  return InitWrapperArray<MjContact>(get()->contact, get()->ncon);
}

MjvScene::MjvScene() {
  owned_ = true;
  ptr_ = new mjvScene;
  mjv_defaultScene(ptr_);
  mjv_makeScene(nullptr, ptr_, 0);
  lights = InitWrapperArray<MjvLight>(ptr_->lights, mjMAXLIGHT);
  camera = InitWrapperArray<MjvGLCamera>(ptr_->camera, 2);
};

MjvScene::MjvScene(MjModel* m, int maxgeom) {
  owned_ = true;
  model = m->get();
  ptr_ = new mjvScene;
  mjv_defaultScene(ptr_);
  mjv_makeScene(model, ptr_, maxgeom);
  lights = InitWrapperArray<MjvLight>(ptr_->lights, mjMAXLIGHT);
  camera = InitWrapperArray<MjvGLCamera>(ptr_->camera, 2);
};

MjvScene::~MjvScene() {
  if (owned_ && ptr_) {
    mjv_freeScene(ptr_);
    delete ptr_;
  }
}

mjvScene* MjvScene::get() const { return ptr_; }
void MjvScene::set(mjvScene* ptr) { ptr_ = ptr; }

// Taken from the python mujoco bindings code for MjvScene Wrapper
int MjvScene::GetSumFlexFaces() const {
  int nflexface = 0;
  int flexfacenum = 0;
  for (int f = 0; f < model->nflex; f++) {
    if (model->flex_dim[f] == 0) {
      // 1D : 0
      flexfacenum = 0;
    } else if (model->flex_dim[f] == 2) {
      // 2D: 2*fragments + 2*elements
      flexfacenum = 2 * model->flex_shellnum[f] + 2 * model->flex_elemnum[f];
    } else {
      // 3D: max(fragments, 4*maxlayer)
      // find number of elements in biggest layer
      int maxlayer = 0, layer = 0, nlayer = 1;
      while (nlayer) {
        nlayer = 0;
        for (int e = 0; e < model->flex_elemnum[f]; e++) {
          if (model->flex_elemlayer[model->flex_elemadr[f] + e] == layer) {
            nlayer++;
          }
        }
        maxlayer = mjMAX(maxlayer, nlayer);
        layer++;
      }
      flexfacenum = mjMAX(model->flex_shellnum[f], 4 * maxlayer);
    }

    // accumulate over flexes
    nflexface += flexfacenum;
  }
  return nflexface;
}

std::vector<MjvGeom> MjvScene::geoms() const {
  return InitWrapperArray<MjvGeom>(ptr_->geoms, ptr_->ngeom);
}

MjSpec::MjSpec()
    : ptr_(mj_makeSpec()),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat) {
  owned_ = true;
  mjs_defaultSpec(ptr_);
};

MjSpec::MjSpec(mjSpec *ptr)
    : ptr_(ptr),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat) {}

MjSpec::MjSpec(const MjSpec &other)
    : ptr_(mj_copySpec(other.get())),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat) {
  owned_ = true;
}

MjSpec& MjSpec::operator=(const MjSpec &other) {
  if (this == &other) {
    return *this;
  }
  if (owned_ && ptr_) {
    mj_deleteSpec(ptr_);
  }
  ptr_ = mj_copySpec(other.get());
  owned_ = true;
  option.set(&ptr_->option);
  visual.set(&ptr_->visual);
  stat.set(&ptr_->stat);
  compiler.set(&ptr_->compiler);
  element.set(ptr_->element);
  return *this;
}

MjSpec::~MjSpec() {
  if (ptr_ && owned_) {
    mj_deleteSpec(ptr_);
  }
}

mjSpec *MjSpec::get() const { return ptr_; }
void MjSpec::set(mjSpec *ptr) { ptr_ = ptr; }

std::unique_ptr<MjModel> mj_loadXML_wrapper(std::string filename) {
  char error[1000];
  mjModel *model = mj_loadXML(filename.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    printf("Loading error: %s\n", error);
    return nullptr;
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjSpec> parseXMLString_wrapper(const std::string &xml) {
  char error[1000];
  mjSpec *ptr = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
  if (!ptr) {
    printf("Could not create Spec from XML string: %s\n", error);
    return nullptr;
  }
  return std::unique_ptr<MjSpec>(new MjSpec(ptr));
}

void error_wrapper(const String& msg) { mju_error("%s\n", msg.as<const std::string>().data()); }

int mj_saveLastXML_wrapper(const String& filename, const MjModel& m) {
  CHECK_VAL(filename);
  std::array<char, 1024> error;
  int result = mj_saveLastXML(filename.as<const std::string>().data(), m.get(), error.data(), error.size());
  if (!result) {
    mju_error("%s", error.data());
  }
  return result;
}

int mj_setLengthRange_wrapper(const MjModel& m, const MjData& d, int index, const MjLROpt& opt) {
  std::array<char, 1024> error;
  int result = mj_setLengthRange(m.get(), d.get(), index, opt.get(), error.data(), error.size());
  if (!result) {
    mju_error("%s", error.data());
  }
  return result;
}

void mj_Euler_wrapper(const MjModel& m, MjData& d) {
  mj_Euler(m.get(), d.get());
}

void mj_RungeKutta_wrapper(const MjModel& m, MjData& d, int N) {
  mj_RungeKutta(m.get(), d.get(), N);
}

int mj_addContact_wrapper(const MjModel& m, MjData& d, const MjContact& con) {
  return mj_addContact(m.get(), d.get(), con.get());
}

void mj_addM_wrapper(const MjModel& m, MjData& d, const val& dst, const val& rownnz, const val& rowadr, const val& colind) {
  UNPACK_VALUE(mjtNum, dst);
  UNPACK_NULLABLE_VALUE(int, rownnz);
  UNPACK_NULLABLE_VALUE(int, rowadr);
  UNPACK_NULLABLE_VALUE(int, colind);
  CHECK_SIZE(rownnz, m.nv());
  CHECK_SIZE(rowadr, m.nv());
  CHECK_SIZE(colind, m.nM());
  CHECK_SIZE(dst, m.nM());
  mj_addM(m.get(), d.get(), dst_.data(), rownnz_.data(), rowadr_.data(), colind_.data());
}

void mj_angmomMat_wrapper(const MjModel& m, MjData& d, const val& mat, int body) {
  UNPACK_VALUE(mjtNum, mat);
  CHECK_SIZE(mat, m.nv() * 3);
  mj_angmomMat(m.get(), d.get(), mat_.data(), body);
}

void mj_applyFT_wrapper(const MjModel& m, MjData& d, const NumberArray& force, const NumberArray& torque, const NumberArray& point, int body, const val& qfrc_target) {
  UNPACK_NULLABLE_ARRAY(mjtNum, force);
  UNPACK_NULLABLE_ARRAY(mjtNum, torque);
  UNPACK_ARRAY(mjtNum, point);
  UNPACK_VALUE(mjtNum, qfrc_target);
  CHECK_SIZE(qfrc_target, m.nv());
  CHECK_SIZE(force, 3);
  CHECK_SIZE(torque, 3);
  CHECK_SIZE(point, 3);
  mj_applyFT(m.get(), d.get(), force_.data(), torque_.data(), point_.data(), body, qfrc_target_.data());
}

void mj_camlight_wrapper(const MjModel& m, MjData& d) {
  mj_camlight(m.get(), d.get());
}

void mj_checkAcc_wrapper(const MjModel& m, MjData& d) {
  mj_checkAcc(m.get(), d.get());
}

void mj_checkPos_wrapper(const MjModel& m, MjData& d) {
  mj_checkPos(m.get(), d.get());
}

void mj_checkVel_wrapper(const MjModel& m, MjData& d) {
  mj_checkVel(m.get(), d.get());
}

void mj_collision_wrapper(const MjModel& m, MjData& d) {
  mj_collision(m.get(), d.get());
}

void mj_comPos_wrapper(const MjModel& m, MjData& d) {
  mj_comPos(m.get(), d.get());
}

void mj_comVel_wrapper(const MjModel& m, MjData& d) {
  mj_comVel(m.get(), d.get());
}

void mj_compareFwdInv_wrapper(const MjModel& m, MjData& d) {
  mj_compareFwdInv(m.get(), d.get());
}

void mj_constraintUpdate_wrapper(const MjModel& m, MjData& d, const NumberArray& jar, const val& cost, int flg_coneHessian) {
  UNPACK_ARRAY(mjtNum, jar);
  UNPACK_NULLABLE_VALUE(mjtNum, cost);
  CHECK_SIZE(cost, 1);
  CHECK_SIZE(jar, d.nefc());
  mj_constraintUpdate(m.get(), d.get(), jar_.data(), cost_.data(), flg_coneHessian);
}

void mj_contactForce_wrapper(const MjModel& m, const MjData& d, int id, const val& result) {
  UNPACK_VALUE(mjtNum, result);
  mj_contactForce(m.get(), d.get(), id, result_.data());
}

int mj_copyBack_wrapper(MjSpec& s, const MjModel& m) {
  return mj_copyBack(s.get(), m.get());
}

void mj_copyState_wrapper(const MjModel& m, const MjData& src, MjData& dst, int sig) {
  mj_copyState(m.get(), src.get(), dst.get(), sig);
}

void mj_crb_wrapper(const MjModel& m, MjData& d) {
  mj_crb(m.get(), d.get());
}

void mj_defaultLROpt_wrapper(MjLROpt& opt) {
  mj_defaultLROpt(opt.get());
}

void mj_defaultOption_wrapper(MjOption& opt) {
  mj_defaultOption(opt.get());
}

void mj_defaultSolRefImp_wrapper(const val& solref, const val& solimp) {
  UNPACK_NULLABLE_VALUE(mjtNum, solref);
  UNPACK_NULLABLE_VALUE(mjtNum, solimp);
  mj_defaultSolRefImp(solref_.data(), solimp_.data());
}

void mj_defaultVisual_wrapper(MjVisual& vis) {
  mj_defaultVisual(vis.get());
}

void mj_differentiatePos_wrapper(const MjModel& m, const val& qvel, mjtNum dt, const NumberArray& qpos1, const NumberArray& qpos2) {
  UNPACK_VALUE(mjtNum, qvel);
  UNPACK_ARRAY(mjtNum, qpos1);
  UNPACK_ARRAY(mjtNum, qpos2);
  CHECK_SIZE(qvel, m.nv());
  CHECK_SIZE(qpos1, m.nq());
  CHECK_SIZE(qpos2, m.nq());
  mj_differentiatePos(m.get(), qvel_.data(), dt, qpos1_.data(), qpos2_.data());
}

void mj_energyPos_wrapper(const MjModel& m, MjData& d) {
  mj_energyPos(m.get(), d.get());
}

void mj_energyVel_wrapper(const MjModel& m, MjData& d) {
  mj_energyVel(m.get(), d.get());
}

void mj_extractState_wrapper(const MjModel& m, const NumberArray& src, int srcsig, const val& dst, int dstsig) {
  UNPACK_ARRAY(mjtNum, src);
  UNPACK_VALUE(mjtNum, dst);
  mj_extractState(m.get(), src_.data(), srcsig, dst_.data(), dstsig);
}

void mj_factorM_wrapper(const MjModel& m, MjData& d) {
  mj_factorM(m.get(), d.get());
}

void mj_flex_wrapper(const MjModel& m, MjData& d) {
  mj_flex(m.get(), d.get());
}

void mj_forward_wrapper(const MjModel& m, MjData& d) {
  mj_forward(m.get(), d.get());
}

void mj_forwardSkip_wrapper(const MjModel& m, MjData& d, int skipstage, int skipsensor) {
  mj_forwardSkip(m.get(), d.get(), skipstage, skipsensor);
}

void mj_fullM_wrapper(const MjModel& m, const val& dst, const NumberArray& M) {
  UNPACK_VALUE(mjtNum, dst);
  UNPACK_ARRAY(mjtNum, M);
  CHECK_SIZE(M, m.nM());
  CHECK_SIZE(dst, m.nv() * m.nv());
  mj_fullM(m.get(), dst_.data(), M_.data());
}

void mj_fwdAcceleration_wrapper(const MjModel& m, MjData& d) {
  mj_fwdAcceleration(m.get(), d.get());
}

void mj_fwdActuation_wrapper(const MjModel& m, MjData& d) {
  mj_fwdActuation(m.get(), d.get());
}

void mj_fwdConstraint_wrapper(const MjModel& m, MjData& d) {
  mj_fwdConstraint(m.get(), d.get());
}

void mj_fwdKinematics_wrapper(const MjModel& m, MjData& d) {
  mj_fwdKinematics(m.get(), d.get());
}

void mj_fwdPosition_wrapper(const MjModel& m, MjData& d) {
  mj_fwdPosition(m.get(), d.get());
}

void mj_fwdVelocity_wrapper(const MjModel& m, MjData& d) {
  mj_fwdVelocity(m.get(), d.get());
}

mjtNum mj_geomDistance_wrapper(const MjModel& m, const MjData& d, int geom1, int geom2, mjtNum distmax, const val& fromto) {
  UNPACK_NULLABLE_VALUE(mjtNum, fromto);
  CHECK_SIZE(fromto, 6);
  return mj_geomDistance(m.get(), d.get(), geom1, geom2, distmax, fromto_.data());
}

void mj_getState_wrapper(const MjModel& m, const MjData& d, const val& state, int sig) {
  UNPACK_VALUE(mjtNum, state);
  CHECK_SIZE(state, mj_stateSize(m.get(), sig));
  mj_getState(m.get(), d.get(), state_.data(), sig);
}

mjtNum mj_getTotalmass_wrapper(const MjModel& m) {
  return mj_getTotalmass(m.get());
}

std::string mj_id2name_wrapper(const MjModel& m, int type, int id) {
  return std::string(mj_id2name(m.get(), type, id));
}

void mj_implicit_wrapper(const MjModel& m, MjData& d) {
  mj_implicit(m.get(), d.get());
}

void mj_integratePos_wrapper(const MjModel& m, const val& qpos, const NumberArray& qvel, mjtNum dt) {
  UNPACK_VALUE(mjtNum, qpos);
  UNPACK_ARRAY(mjtNum, qvel);
  CHECK_SIZE(qpos, m.nq());
  CHECK_SIZE(qvel, m.nv());
  mj_integratePos(m.get(), qpos_.data(), qvel_.data(), dt);
}

void mj_invConstraint_wrapper(const MjModel& m, MjData& d) {
  mj_invConstraint(m.get(), d.get());
}

void mj_invPosition_wrapper(const MjModel& m, MjData& d) {
  mj_invPosition(m.get(), d.get());
}

void mj_invVelocity_wrapper(const MjModel& m, MjData& d) {
  mj_invVelocity(m.get(), d.get());
}

void mj_inverse_wrapper(const MjModel& m, MjData& d) {
  mj_inverse(m.get(), d.get());
}

void mj_inverseSkip_wrapper(const MjModel& m, MjData& d, int skipstage, int skipsensor) {
  mj_inverseSkip(m.get(), d.get(), skipstage, skipsensor);
}

int mj_isDual_wrapper(const MjModel& m) {
  return mj_isDual(m.get());
}

int mj_isPyramidal_wrapper(const MjModel& m) {
  return mj_isPyramidal(m.get());
}

int mj_isSparse_wrapper(const MjModel& m) {
  return mj_isSparse(m.get());
}

void mj_island_wrapper(const MjModel& m, MjData& d) {
  mj_island(m.get(), d.get());
}

void mj_jac_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, const NumberArray& point, int body) {
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  UNPACK_ARRAY(mjtNum, point);
  CHECK_SIZE(point, 3);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jac(m.get(), d.get(), jacp_.data(), jacr_.data(), point_.data(), body);
}

void mj_jacBody_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int body) {
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacBody(m.get(), d.get(), jacp_.data(), jacr_.data(), body);
}

void mj_jacBodyCom_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int body) {
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacBodyCom(m.get(), d.get(), jacp_.data(), jacr_.data(), body);
}

void mj_jacDot_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, const NumberArray& point, int body) {
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  UNPACK_ARRAY(mjtNum, point);
  CHECK_SIZE(point, 3);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacDot(m.get(), d.get(), jacp_.data(), jacr_.data(), point_.data(), body);
}

void mj_jacGeom_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int geom) {
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacGeom(m.get(), d.get(), jacp_.data(), jacr_.data(), geom);
}

void mj_jacPointAxis_wrapper(const MjModel& m, MjData& d, const val& jacPoint, const val& jacAxis, const NumberArray& point, const NumberArray& axis, int body) {
  UNPACK_NULLABLE_VALUE(mjtNum, jacPoint);
  UNPACK_NULLABLE_VALUE(mjtNum, jacAxis);
  UNPACK_ARRAY(mjtNum, point);
  UNPACK_ARRAY(mjtNum, axis);
  CHECK_SIZE(point, 3);
  CHECK_SIZE(axis, 3);
  CHECK_SIZE(jacPoint, m.nv() * 3);
  CHECK_SIZE(jacAxis, m.nv() * 3);
  mj_jacPointAxis(m.get(), d.get(), jacPoint_.data(), jacAxis_.data(), point_.data(), axis_.data(), body);
}

void mj_jacSite_wrapper(const MjModel& m, const MjData& d, const val& jacp, const val& jacr, int site) {
  UNPACK_NULLABLE_VALUE(mjtNum, jacp);
  UNPACK_NULLABLE_VALUE(mjtNum, jacr);
  CHECK_SIZE(jacp, m.nv() * 3);
  CHECK_SIZE(jacr, m.nv() * 3);
  mj_jacSite(m.get(), d.get(), jacp_.data(), jacr_.data(), site);
}

void mj_jacSubtreeCom_wrapper(const MjModel& m, MjData& d, const val& jacp, int body) {
  UNPACK_VALUE(mjtNum, jacp);
  CHECK_SIZE(jacp, m.nv() * 3);
  mj_jacSubtreeCom(m.get(), d.get(), jacp_.data(), body);
}

void mj_kinematics_wrapper(const MjModel& m, MjData& d) {
  mj_kinematics(m.get(), d.get());
}

void mj_local2Global_wrapper(MjData& d, const val& xpos, const val& xmat, const NumberArray& pos, const NumberArray& quat, int body, mjtByte sameframe) {
  UNPACK_VALUE(mjtNum, xpos);
  UNPACK_VALUE(mjtNum, xmat);
  UNPACK_ARRAY(mjtNum, pos);
  UNPACK_ARRAY(mjtNum, quat);
  mj_local2Global(d.get(), xpos_.data(), xmat_.data(), pos_.data(), quat_.data(), body, sameframe);
}

void mj_makeConstraint_wrapper(const MjModel& m, MjData& d) {
  mj_makeConstraint(m.get(), d.get());
}

void mj_makeM_wrapper(const MjModel& m, MjData& d) {
  mj_makeM(m.get(), d.get());
}

void mj_mulJacTVec_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, d.nefc());
  mj_mulJacTVec(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_mulJacVec_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, d.nefc());
  CHECK_SIZE(vec, m.nv());
  mj_mulJacVec(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_mulM_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, m.nv());
  mj_mulM(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_mulM2_wrapper(const MjModel& m, const MjData& d, const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(res, m.nv());
  CHECK_SIZE(vec, m.nv());
  mj_mulM2(m.get(), d.get(), res_.data(), vec_.data());
}

void mj_multiRay_wrapper(const MjModel& m, MjData& d, const NumberArray& pnt, const NumberArray& vec, const NumberArray& geomgroup, mjtByte flg_static, int bodyexclude, const val& geomid, const val& dist, int nray, mjtNum cutoff) {
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_NULLABLE_ARRAY(mjtByte, geomgroup);
  UNPACK_VALUE(int, geomid);
  UNPACK_VALUE(mjtNum, dist);
  CHECK_SIZE(dist, nray);
  CHECK_SIZE(geomid, nray);
  CHECK_SIZE(vec, 3 * nray);
  mj_multiRay(m.get(), d.get(), pnt_.data(), vec_.data(), geomgroup_.data(), flg_static, bodyexclude, geomid_.data(), dist_.data(), nray, cutoff);
}

int mj_name2id_wrapper(const MjModel& m, int type, const String& name) {
  CHECK_VAL(name);
  return mj_name2id(m.get(), type, name.as<const std::string>().data());
}

void mj_normalizeQuat_wrapper(const MjModel& m, const val& qpos) {
  UNPACK_VALUE(mjtNum, qpos);
  CHECK_SIZE(qpos, m.nq());
  mj_normalizeQuat(m.get(), qpos_.data());
}

void mj_objectAcceleration_wrapper(const MjModel& m, const MjData& d, int objtype, int objid, const val& res, int flg_local) {
  UNPACK_VALUE(mjtNum, res);
  mj_objectAcceleration(m.get(), d.get(), objtype, objid, res_.data(), flg_local);
}

void mj_objectVelocity_wrapper(const MjModel& m, const MjData& d, int objtype, int objid, const val& res, int flg_local) {
  UNPACK_VALUE(mjtNum, res);
  mj_objectVelocity(m.get(), d.get(), objtype, objid, res_.data(), flg_local);
}

void mj_passive_wrapper(const MjModel& m, MjData& d) {
  mj_passive(m.get(), d.get());
}

void mj_printData_wrapper(const MjModel& m, const MjData& d, const String& filename) {
  CHECK_VAL(filename);
  mj_printData(m.get(), d.get(), filename.as<const std::string>().data());
}

void mj_printFormattedData_wrapper(const MjModel& m, const MjData& d, const String& filename, const String& float_format) {
  CHECK_VAL(filename);
  CHECK_VAL(float_format);
  mj_printFormattedData(m.get(), d.get(), filename.as<const std::string>().data(), float_format.as<const std::string>().data());
}

void mj_printFormattedModel_wrapper(const MjModel& m, const String& filename, const String& float_format) {
  CHECK_VAL(filename);
  CHECK_VAL(float_format);
  mj_printFormattedModel(m.get(), filename.as<const std::string>().data(), float_format.as<const std::string>().data());
}

void mj_printFormattedScene_wrapper(const MjvScene& s, const String& filename, const String& float_format) {
  CHECK_VAL(filename);
  CHECK_VAL(float_format);
  mj_printFormattedScene(s.get(), filename.as<const std::string>().data(), float_format.as<const std::string>().data());
}

void mj_printModel_wrapper(const MjModel& m, const String& filename) {
  CHECK_VAL(filename);
  mj_printModel(m.get(), filename.as<const std::string>().data());
}

void mj_printScene_wrapper(const MjvScene& s, const String& filename) {
  CHECK_VAL(filename);
  mj_printScene(s.get(), filename.as<const std::string>().data());
}

void mj_projectConstraint_wrapper(const MjModel& m, MjData& d) {
  mj_projectConstraint(m.get(), d.get());
}

mjtNum mj_ray_wrapper(const MjModel& m, const MjData& d, const NumberArray& pnt, const NumberArray& vec, const NumberArray& geomgroup, mjtByte flg_static, int bodyexclude, const val& geomid) {
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_NULLABLE_ARRAY(mjtByte, geomgroup);
  UNPACK_NULLABLE_VALUE(int, geomid);
  return mj_ray(m.get(), d.get(), pnt_.data(), vec_.data(), geomgroup_.data(), flg_static, bodyexclude, geomid_.data());
}

mjtNum mj_rayHfield_wrapper(const MjModel& m, const MjData& d, int geomid, const NumberArray& pnt, const NumberArray& vec) {
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  return mj_rayHfield(m.get(), d.get(), geomid, pnt_.data(), vec_.data());
}

mjtNum mj_rayMesh_wrapper(const MjModel& m, const MjData& d, int geomid, const NumberArray& pnt, const NumberArray& vec) {
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  return mj_rayMesh(m.get(), d.get(), geomid, pnt_.data(), vec_.data());
}

void mj_referenceConstraint_wrapper(const MjModel& m, MjData& d) {
  mj_referenceConstraint(m.get(), d.get());
}

void mj_resetData_wrapper(const MjModel& m, MjData& d) {
  mj_resetData(m.get(), d.get());
}

void mj_resetDataDebug_wrapper(const MjModel& m, MjData& d, unsigned char debug_value) {
  mj_resetDataDebug(m.get(), d.get(), debug_value);
}

void mj_resetDataKeyframe_wrapper(const MjModel& m, MjData& d, int key) {
  mj_resetDataKeyframe(m.get(), d.get(), key);
}

void mj_rne_wrapper(const MjModel& m, MjData& d, int flg_acc, const val& result) {
  UNPACK_VALUE(mjtNum, result);
  CHECK_SIZE(result, m.nv());
  mj_rne(m.get(), d.get(), flg_acc, result_.data());
}

void mj_rnePostConstraint_wrapper(const MjModel& m, MjData& d) {
  mj_rnePostConstraint(m.get(), d.get());
}

void mj_sensorAcc_wrapper(const MjModel& m, MjData& d) {
  mj_sensorAcc(m.get(), d.get());
}

void mj_sensorPos_wrapper(const MjModel& m, MjData& d) {
  mj_sensorPos(m.get(), d.get());
}

void mj_sensorVel_wrapper(const MjModel& m, MjData& d) {
  mj_sensorVel(m.get(), d.get());
}

void mj_setConst_wrapper(MjModel& m, MjData& d) {
  mj_setConst(m.get(), d.get());
}

void mj_setKeyframe_wrapper(MjModel& m, const MjData& d, int k) {
  mj_setKeyframe(m.get(), d.get(), k);
}

void mj_setState_wrapper(const MjModel& m, MjData& d, const NumberArray& state, int sig) {
  UNPACK_ARRAY(mjtNum, state);
  CHECK_SIZE(state, mj_stateSize(m.get(), sig));
  mj_setState(m.get(), d.get(), state_.data(), sig);
}

void mj_setTotalmass_wrapper(MjModel& m, mjtNum newmass) {
  mj_setTotalmass(m.get(), newmass);
}

mjtSize mj_sizeModel_wrapper(const MjModel& m) {
  return mj_sizeModel(m.get());
}

void mj_solveM_wrapper(const MjModel& m, MjData& d, const val& x, const NumberArray& y) {
  UNPACK_VALUE(mjtNum, x);
  UNPACK_ARRAY(mjtNum, y);
  CHECK_SIZES(x, y);
  CHECK_DIVISIBLE(x, m.nv());
  int n = x_div.quot;
  mj_solveM(m.get(), d.get(), x_.data(), y_.data(), n);
}

void mj_solveM2_wrapper(const MjModel& m, MjData& d, const val& x, const NumberArray& y, const NumberArray& sqrtInvD) {
  UNPACK_VALUE(mjtNum, x);
  UNPACK_ARRAY(mjtNum, y);
  UNPACK_ARRAY(mjtNum, sqrtInvD);
  CHECK_SIZES(x, y);
  CHECK_SIZE(sqrtInvD, m.nv());
  CHECK_DIVISIBLE(x, m.nv());
  int n = x_div.quot;
  mj_solveM2(m.get(), d.get(), x_.data(), y_.data(), sqrtInvD_.data(), n);
}

int mj_stateSize_wrapper(const MjModel& m, int sig) {
  return mj_stateSize(m.get(), sig);
}

void mj_step_wrapper(const MjModel& m, MjData& d) {
  mj_step(m.get(), d.get());
}

void mj_step1_wrapper(const MjModel& m, MjData& d) {
  mj_step1(m.get(), d.get());
}

void mj_step2_wrapper(const MjModel& m, MjData& d) {
  mj_step2(m.get(), d.get());
}

void mj_subtreeVel_wrapper(const MjModel& m, MjData& d) {
  mj_subtreeVel(m.get(), d.get());
}

void mj_tendon_wrapper(const MjModel& m, MjData& d) {
  mj_tendon(m.get(), d.get());
}

void mj_transmission_wrapper(const MjModel& m, MjData& d) {
  mj_transmission(m.get(), d.get());
}

std::string mj_versionString_wrapper() {
  return std::string(mj_versionString());
}

void mjd_inverseFD_wrapper(const MjModel& m, MjData& d, mjtNum eps, mjtByte flg_actuation, const val& DfDq, const val& DfDv, const val& DfDa, const val& DsDq, const val& DsDv, const val& DsDa, const val& DmDq) {
  UNPACK_NULLABLE_VALUE(mjtNum, DfDq);
  UNPACK_NULLABLE_VALUE(mjtNum, DfDv);
  UNPACK_NULLABLE_VALUE(mjtNum, DfDa);
  UNPACK_NULLABLE_VALUE(mjtNum, DsDq);
  UNPACK_NULLABLE_VALUE(mjtNum, DsDv);
  UNPACK_NULLABLE_VALUE(mjtNum, DsDa);
  UNPACK_NULLABLE_VALUE(mjtNum, DmDq);
  CHECK_SIZE(DfDq, m.nv() * m.nv());
  CHECK_SIZE(DfDv, m.nv() * m.nv());
  CHECK_SIZE(DfDa, m.nv() * m.nv());
  CHECK_SIZE(DsDq, m.nv() * m.nsensordata());
  CHECK_SIZE(DsDv, m.nv() * m.nsensordata());
  CHECK_SIZE(DsDa, m.nv() * m.nsensordata());
  CHECK_SIZE(DmDq, m.nv() * m.nM());
  mjd_inverseFD(m.get(), d.get(), eps, flg_actuation, DfDq_.data(), DfDv_.data(), DfDa_.data(), DsDq_.data(), DsDv_.data(), DsDa_.data(), DmDq_.data());
}

void mjd_quatIntegrate_wrapper(const NumberArray& vel, mjtNum scale, const val& Dquat, const val& Dvel, const val& Dscale) {
  UNPACK_ARRAY(mjtNum, vel);
  UNPACK_NULLABLE_VALUE(mjtNum, Dquat);
  UNPACK_NULLABLE_VALUE(mjtNum, Dvel);
  UNPACK_NULLABLE_VALUE(mjtNum, Dscale);
  mjd_quatIntegrate(vel_.data(), scale, Dquat_.data(), Dvel_.data(), Dscale_.data());
}

void mjd_subQuat_wrapper(const NumberArray& qa, const NumberArray& qb, const val& Da, const val& Db) {
  UNPACK_ARRAY(mjtNum, qa);
  UNPACK_ARRAY(mjtNum, qb);
  UNPACK_NULLABLE_VALUE(mjtNum, Da);
  UNPACK_NULLABLE_VALUE(mjtNum, Db);
  CHECK_SIZE(qa, 4);
  CHECK_SIZE(qb, 4);
  CHECK_SIZE(Da, 9);
  CHECK_SIZE(Db, 9);
  mjd_subQuat(qa_.data(), qb_.data(), Da_.data(), Db_.data());
}

void mjd_transitionFD_wrapper(const MjModel& m, MjData& d, mjtNum eps, mjtByte flg_centered, const val& A, const val& B, const val& C, const val& D) {
  UNPACK_NULLABLE_VALUE(mjtNum, A);
  UNPACK_NULLABLE_VALUE(mjtNum, B);
  UNPACK_NULLABLE_VALUE(mjtNum, C);
  UNPACK_NULLABLE_VALUE(mjtNum, D);
  CHECK_SIZE(A, (2 * m.nv() + m.na()) * (2 * m.nv() + m.na()));
  CHECK_SIZE(B, (2 * m.nv() + m.na()) * m.nu());
  CHECK_SIZE(C, m.nsensordata() * (2 * m.nv() + m.na()));
  CHECK_SIZE(D, m.nsensordata() * m.nu());
  mjd_transitionFD(m.get(), d.get(), eps, flg_centered, A_.data(), B_.data(), C_.data(), D_.data());
}

int mjs_activatePlugin_wrapper(MjSpec& s, const String& name) {
  CHECK_VAL(name);
  return mjs_activatePlugin(s.get(), name.as<const std::string>().data());
}

std::optional<MjsActuator> mjs_addActuator_wrapper(MjSpec& s, const MjsDefault& def) {
  mjsActuator* result = mjs_addActuator(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsActuator(result);
}

std::optional<MjsBody> mjs_addBody_wrapper(MjsBody& body, const MjsDefault& def) {
  mjsBody* result = mjs_addBody(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjsCamera> mjs_addCamera_wrapper(MjsBody& body, const MjsDefault& def) {
  mjsCamera* result = mjs_addCamera(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsCamera(result);
}

std::optional<MjsDefault> mjs_addDefault_wrapper(MjSpec& s, const String& classname, const MjsDefault& parent) {
  CHECK_VAL(classname);
  mjsDefault* result = mjs_addDefault(s.get(), classname.as<const std::string>().data(), parent.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsDefault(result);
}

std::optional<MjsEquality> mjs_addEquality_wrapper(MjSpec& s, const MjsDefault& def) {
  mjsEquality* result = mjs_addEquality(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsEquality(result);
}

std::optional<MjsExclude> mjs_addExclude_wrapper(MjSpec& s) {
  mjsExclude* result = mjs_addExclude(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsExclude(result);
}

std::optional<MjsFlex> mjs_addFlex_wrapper(MjSpec& s) {
  mjsFlex* result = mjs_addFlex(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFlex(result);
}

std::optional<MjsFrame> mjs_addFrame_wrapper(MjsBody& body, MjsFrame& parentframe) {
  mjsFrame* result = mjs_addFrame(body.get(), parentframe.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFrame(result);
}

std::optional<MjsJoint> mjs_addFreeJoint_wrapper(MjsBody& body) {
  mjsJoint* result = mjs_addFreeJoint(body.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsJoint(result);
}

std::optional<MjsGeom> mjs_addGeom_wrapper(MjsBody& body, const MjsDefault& def) {
  mjsGeom* result = mjs_addGeom(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsGeom(result);
}

std::optional<MjsHField> mjs_addHField_wrapper(MjSpec& s) {
  mjsHField* result = mjs_addHField(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsHField(result);
}

std::optional<MjsJoint> mjs_addJoint_wrapper(MjsBody& body, const MjsDefault& def) {
  mjsJoint* result = mjs_addJoint(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsJoint(result);
}

std::optional<MjsKey> mjs_addKey_wrapper(MjSpec& s) {
  mjsKey* result = mjs_addKey(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsKey(result);
}

std::optional<MjsLight> mjs_addLight_wrapper(MjsBody& body, const MjsDefault& def) {
  mjsLight* result = mjs_addLight(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsLight(result);
}

std::optional<MjsMaterial> mjs_addMaterial_wrapper(MjSpec& s, const MjsDefault& def) {
  mjsMaterial* result = mjs_addMaterial(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsMaterial(result);
}

std::optional<MjsMesh> mjs_addMesh_wrapper(MjSpec& s, const MjsDefault& def) {
  mjsMesh* result = mjs_addMesh(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsMesh(result);
}

std::optional<MjsNumeric> mjs_addNumeric_wrapper(MjSpec& s) {
  mjsNumeric* result = mjs_addNumeric(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsNumeric(result);
}

std::optional<MjsPair> mjs_addPair_wrapper(MjSpec& s, const MjsDefault& def) {
  mjsPair* result = mjs_addPair(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsPair(result);
}

std::optional<MjsPlugin> mjs_addPlugin_wrapper(MjSpec& s) {
  mjsPlugin* result = mjs_addPlugin(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsPlugin(result);
}

std::optional<MjsSensor> mjs_addSensor_wrapper(MjSpec& s) {
  mjsSensor* result = mjs_addSensor(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSensor(result);
}

std::optional<MjsSite> mjs_addSite_wrapper(MjsBody& body, const MjsDefault& def) {
  mjsSite* result = mjs_addSite(body.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSite(result);
}

std::optional<MjsSkin> mjs_addSkin_wrapper(MjSpec& s) {
  mjsSkin* result = mjs_addSkin(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSkin(result);
}

std::optional<MjsTendon> mjs_addTendon_wrapper(MjSpec& s, const MjsDefault& def) {
  mjsTendon* result = mjs_addTendon(s.get(), def.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTendon(result);
}

std::optional<MjsText> mjs_addText_wrapper(MjSpec& s) {
  mjsText* result = mjs_addText(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsText(result);
}

std::optional<MjsTexture> mjs_addTexture_wrapper(MjSpec& s) {
  mjsTexture* result = mjs_addTexture(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTexture(result);
}

std::optional<MjsTuple> mjs_addTuple_wrapper(MjSpec& s) {
  mjsTuple* result = mjs_addTuple(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTuple(result);
}

std::optional<MjsActuator> mjs_asActuator_wrapper(MjsElement& element) {
  mjsActuator* result = mjs_asActuator(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsActuator(result);
}

std::optional<MjsBody> mjs_asBody_wrapper(MjsElement& element) {
  mjsBody* result = mjs_asBody(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjsCamera> mjs_asCamera_wrapper(MjsElement& element) {
  mjsCamera* result = mjs_asCamera(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsCamera(result);
}

std::optional<MjsEquality> mjs_asEquality_wrapper(MjsElement& element) {
  mjsEquality* result = mjs_asEquality(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsEquality(result);
}

std::optional<MjsExclude> mjs_asExclude_wrapper(MjsElement& element) {
  mjsExclude* result = mjs_asExclude(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsExclude(result);
}

std::optional<MjsFlex> mjs_asFlex_wrapper(MjsElement& element) {
  mjsFlex* result = mjs_asFlex(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFlex(result);
}

std::optional<MjsFrame> mjs_asFrame_wrapper(MjsElement& element) {
  mjsFrame* result = mjs_asFrame(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFrame(result);
}

std::optional<MjsGeom> mjs_asGeom_wrapper(MjsElement& element) {
  mjsGeom* result = mjs_asGeom(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsGeom(result);
}

std::optional<MjsHField> mjs_asHField_wrapper(MjsElement& element) {
  mjsHField* result = mjs_asHField(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsHField(result);
}

std::optional<MjsJoint> mjs_asJoint_wrapper(MjsElement& element) {
  mjsJoint* result = mjs_asJoint(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsJoint(result);
}

std::optional<MjsKey> mjs_asKey_wrapper(MjsElement& element) {
  mjsKey* result = mjs_asKey(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsKey(result);
}

std::optional<MjsLight> mjs_asLight_wrapper(MjsElement& element) {
  mjsLight* result = mjs_asLight(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsLight(result);
}

std::optional<MjsMaterial> mjs_asMaterial_wrapper(MjsElement& element) {
  mjsMaterial* result = mjs_asMaterial(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsMaterial(result);
}

std::optional<MjsMesh> mjs_asMesh_wrapper(MjsElement& element) {
  mjsMesh* result = mjs_asMesh(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsMesh(result);
}

std::optional<MjsNumeric> mjs_asNumeric_wrapper(MjsElement& element) {
  mjsNumeric* result = mjs_asNumeric(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsNumeric(result);
}

std::optional<MjsPair> mjs_asPair_wrapper(MjsElement& element) {
  mjsPair* result = mjs_asPair(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsPair(result);
}

std::optional<MjsPlugin> mjs_asPlugin_wrapper(MjsElement& element) {
  mjsPlugin* result = mjs_asPlugin(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsPlugin(result);
}

std::optional<MjsSensor> mjs_asSensor_wrapper(MjsElement& element) {
  mjsSensor* result = mjs_asSensor(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSensor(result);
}

std::optional<MjsSite> mjs_asSite_wrapper(MjsElement& element) {
  mjsSite* result = mjs_asSite(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSite(result);
}

std::optional<MjsSkin> mjs_asSkin_wrapper(MjsElement& element) {
  mjsSkin* result = mjs_asSkin(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSkin(result);
}

std::optional<MjsTendon> mjs_asTendon_wrapper(MjsElement& element) {
  mjsTendon* result = mjs_asTendon(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTendon(result);
}

std::optional<MjsText> mjs_asText_wrapper(MjsElement& element) {
  mjsText* result = mjs_asText(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsText(result);
}

std::optional<MjsTexture> mjs_asTexture_wrapper(MjsElement& element) {
  mjsTexture* result = mjs_asTexture(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTexture(result);
}

std::optional<MjsTuple> mjs_asTuple_wrapper(MjsElement& element) {
  mjsTuple* result = mjs_asTuple(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsTuple(result);
}

std::optional<MjsElement> mjs_attach_wrapper(MjsElement& parent, const MjsElement& child, const String& prefix, const String& suffix) {
  CHECK_VAL(prefix);
  CHECK_VAL(suffix);
  mjsElement* result = mjs_attach(parent.get(), child.get(), prefix.as<const std::string>().data(), suffix.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

void mjs_defaultActuator_wrapper(MjsActuator& actuator) {
  mjs_defaultActuator(actuator.get());
}

void mjs_defaultBody_wrapper(MjsBody& body) {
  mjs_defaultBody(body.get());
}

void mjs_defaultCamera_wrapper(MjsCamera& camera) {
  mjs_defaultCamera(camera.get());
}

void mjs_defaultEquality_wrapper(MjsEquality& equality) {
  mjs_defaultEquality(equality.get());
}

void mjs_defaultFlex_wrapper(MjsFlex& flex) {
  mjs_defaultFlex(flex.get());
}

void mjs_defaultFrame_wrapper(MjsFrame& frame) {
  mjs_defaultFrame(frame.get());
}

void mjs_defaultGeom_wrapper(MjsGeom& geom) {
  mjs_defaultGeom(geom.get());
}

void mjs_defaultHField_wrapper(MjsHField& hfield) {
  mjs_defaultHField(hfield.get());
}

void mjs_defaultJoint_wrapper(MjsJoint& joint) {
  mjs_defaultJoint(joint.get());
}

void mjs_defaultKey_wrapper(MjsKey& key) {
  mjs_defaultKey(key.get());
}

void mjs_defaultLight_wrapper(MjsLight& light) {
  mjs_defaultLight(light.get());
}

void mjs_defaultMaterial_wrapper(MjsMaterial& material) {
  mjs_defaultMaterial(material.get());
}

void mjs_defaultMesh_wrapper(MjsMesh& mesh) {
  mjs_defaultMesh(mesh.get());
}

void mjs_defaultNumeric_wrapper(MjsNumeric& numeric) {
  mjs_defaultNumeric(numeric.get());
}

void mjs_defaultOrientation_wrapper(MjsOrientation& orient) {
  mjs_defaultOrientation(orient.get());
}

void mjs_defaultPair_wrapper(MjsPair& pair) {
  mjs_defaultPair(pair.get());
}

void mjs_defaultPlugin_wrapper(MjsPlugin& plugin) {
  mjs_defaultPlugin(plugin.get());
}

void mjs_defaultSensor_wrapper(MjsSensor& sensor) {
  mjs_defaultSensor(sensor.get());
}

void mjs_defaultSite_wrapper(MjsSite& site) {
  mjs_defaultSite(site.get());
}

void mjs_defaultSkin_wrapper(MjsSkin& skin) {
  mjs_defaultSkin(skin.get());
}

void mjs_defaultSpec_wrapper(MjSpec& spec) {
  mjs_defaultSpec(spec.get());
}

void mjs_defaultTendon_wrapper(MjsTendon& tendon) {
  mjs_defaultTendon(tendon.get());
}

void mjs_defaultText_wrapper(MjsText& text) {
  mjs_defaultText(text.get());
}

void mjs_defaultTexture_wrapper(MjsTexture& texture) {
  mjs_defaultTexture(texture.get());
}

void mjs_defaultTuple_wrapper(MjsTuple& tuple) {
  mjs_defaultTuple(tuple.get());
}

int mjs_delete_wrapper(MjSpec& spec, MjsElement& element) {
  return mjs_delete(spec.get(), element.get());
}

void mjs_deleteUserValue_wrapper(MjsElement& element, const String& key) {
  CHECK_VAL(key);
  mjs_deleteUserValue(element.get(), key.as<const std::string>().data());
}

std::optional<MjsBody> mjs_findBody_wrapper(MjSpec& s, const String& name) {
  CHECK_VAL(name);
  mjsBody* result = mjs_findBody(s.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjsBody> mjs_findChild_wrapper(MjsBody& body, const String& name) {
  CHECK_VAL(name);
  mjsBody* result = mjs_findChild(body.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjsDefault> mjs_findDefault_wrapper(MjSpec& s, const String& classname) {
  CHECK_VAL(classname);
  mjsDefault* result = mjs_findDefault(s.get(), classname.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsDefault(result);
}

std::optional<MjsElement> mjs_findElement_wrapper(MjSpec& s, mjtObj type, const String& name) {
  CHECK_VAL(name);
  mjsElement* result = mjs_findElement(s.get(), type, name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsFrame> mjs_findFrame_wrapper(MjSpec& s, const String& name) {
  CHECK_VAL(name);
  mjsFrame* result = mjs_findFrame(s.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFrame(result);
}

std::optional<MjSpec> mjs_findSpec_wrapper(MjSpec& spec, const String& name) {
  CHECK_VAL(name);
  mjSpec* result = mjs_findSpec(spec.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjSpec(result);
}

std::optional<MjsElement> mjs_firstChild_wrapper(MjsBody& body, mjtObj type, int recurse) {
  mjsElement* result = mjs_firstChild(body.get(), type, recurse);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsElement> mjs_firstElement_wrapper(MjSpec& s, mjtObj type) {
  mjsElement* result = mjs_firstElement(s.get(), type);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsDefault> mjs_getDefault_wrapper(MjsElement& element) {
  mjsDefault* result = mjs_getDefault(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsDefault(result);
}

std::string mjs_getError_wrapper(MjSpec& s) {
  return std::string(mjs_getError(s.get()));
}

std::optional<MjsFrame> mjs_getFrame_wrapper(MjsElement& element) {
  mjsFrame* result = mjs_getFrame(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsFrame(result);
}

int mjs_getId_wrapper(MjsElement& element) {
  return mjs_getId(element.get());
}

std::string mjs_getName_wrapper(MjsElement& element) {
  return *mjs_getName(element.get());
}

std::optional<MjsBody> mjs_getParent_wrapper(MjsElement& element) {
  mjsBody* result = mjs_getParent(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsBody(result);
}

std::optional<MjSpec> mjs_getSpec_wrapper(MjsElement& element) {
  mjSpec* result = mjs_getSpec(element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjSpec(result);
}

std::optional<MjsDefault> mjs_getSpecDefault_wrapper(MjSpec& s) {
  mjsDefault* result = mjs_getSpecDefault(s.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsDefault(result);
}

std::optional<MjsWrap> mjs_getWrap_wrapper(const MjsTendon& tendonspec, int i) {
  mjsWrap* result = mjs_getWrap(tendonspec.get(), i);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

double mjs_getWrapCoef_wrapper(MjsWrap& wrap) {
  return mjs_getWrapCoef(wrap.get());
}

double mjs_getWrapDivisor_wrapper(MjsWrap& wrap) {
  return mjs_getWrapDivisor(wrap.get());
}

int mjs_getWrapNum_wrapper(const MjsTendon& tendonspec) {
  return mjs_getWrapNum(tendonspec.get());
}

std::optional<MjsSite> mjs_getWrapSideSite_wrapper(MjsWrap& wrap) {
  mjsSite* result = mjs_getWrapSideSite(wrap.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsSite(result);
}

std::optional<MjsElement> mjs_getWrapTarget_wrapper(MjsWrap& wrap) {
  mjsElement* result = mjs_getWrapTarget(wrap.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

int mjs_isWarning_wrapper(MjSpec& s) {
  return mjs_isWarning(s.get());
}

int mjs_makeMesh_wrapper(MjsMesh& mesh, mjtMeshBuiltin builtin, const val& params, int nparams) {
  UNPACK_VALUE(double, params);
  return mjs_makeMesh(mesh.get(), builtin, params_.data(), nparams);
}

std::optional<MjsElement> mjs_nextChild_wrapper(MjsBody& body, MjsElement& child, int recurse) {
  mjsElement* result = mjs_nextChild(body.get(), child.get(), recurse);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::optional<MjsElement> mjs_nextElement_wrapper(MjSpec& s, MjsElement& element) {
  mjsElement* result = mjs_nextElement(s.get(), element.get());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsElement(result);
}

std::string mjs_resolveOrientation_wrapper(const val& quat, mjtByte degree, const String& sequence, const MjsOrientation& orientation) {
  CHECK_VAL(sequence);
  UNPACK_VALUE(double, quat);
  return std::string(mjs_resolveOrientation(quat_.data(), degree, sequence.as<const std::string>().data(), orientation.get()));
}

int mjs_sensorDim_wrapper(const MjsSensor& sensor) {
  return mjs_sensorDim(sensor.get());
}

int mjs_setDeepCopy_wrapper(MjSpec& s, int deepcopy) {
  return mjs_setDeepCopy(s.get(), deepcopy);
}

void mjs_setDefault_wrapper(MjsElement& element, const MjsDefault& def) {
  mjs_setDefault(element.get(), def.get());
}

int mjs_setFrame_wrapper(MjsElement& dest, MjsFrame& frame) {
  return mjs_setFrame(dest.get(), frame.get());
}

int mjs_setName_wrapper(MjsElement& element, const String& name) {
  CHECK_VAL(name);
  return mjs_setName(element.get(), name.as<const std::string>().data());
}

std::string mjs_setToAdhesion_wrapper(MjsActuator& actuator, double gain) {
  return std::string(mjs_setToAdhesion(actuator.get(), gain));
}

std::string mjs_setToCylinder_wrapper(MjsActuator& actuator, double timeconst, double bias, double area, double diameter) {
  return std::string(mjs_setToCylinder(actuator.get(), timeconst, bias, area, diameter));
}

std::string mjs_setToDamper_wrapper(MjsActuator& actuator, double kv) {
  return std::string(mjs_setToDamper(actuator.get(), kv));
}

std::string mjs_setToIntVelocity_wrapper(MjsActuator& actuator, double kp, const val& kv, const val& dampratio, const val& timeconst, double inheritrange) {
  UNPACK_VALUE(double, kv);
  UNPACK_VALUE(double, dampratio);
  UNPACK_VALUE(double, timeconst);
  return std::string(mjs_setToIntVelocity(actuator.get(), kp, kv_.data(), dampratio_.data(), timeconst_.data(), inheritrange));
}

std::string mjs_setToMotor_wrapper(MjsActuator& actuator) {
  return std::string(mjs_setToMotor(actuator.get()));
}

std::string mjs_setToMuscle_wrapper(MjsActuator& actuator, const val& timeconst, double tausmooth, const val& range, double force, double scale, double lmin, double lmax, double vmax, double fpmax, double fvmax) {
  UNPACK_VALUE(double, timeconst);
  UNPACK_VALUE(double, range);
  return std::string(mjs_setToMuscle(actuator.get(), timeconst_.data(), tausmooth, range_.data(), force, scale, lmin, lmax, vmax, fpmax, fvmax));
}

std::string mjs_setToPosition_wrapper(MjsActuator& actuator, double kp, const val& kv, const val& dampratio, const val& timeconst, double inheritrange) {
  UNPACK_VALUE(double, kv);
  UNPACK_VALUE(double, dampratio);
  UNPACK_VALUE(double, timeconst);
  return std::string(mjs_setToPosition(actuator.get(), kp, kv_.data(), dampratio_.data(), timeconst_.data(), inheritrange));
}

std::string mjs_setToVelocity_wrapper(MjsActuator& actuator, double kv) {
  return std::string(mjs_setToVelocity(actuator.get(), kv));
}

std::optional<MjsWrap> mjs_wrapGeom_wrapper(MjsTendon& tendon, const String& name, const String& sidesite) {
  CHECK_VAL(name);
  CHECK_VAL(sidesite);
  mjsWrap* result = mjs_wrapGeom(tendon.get(), name.as<const std::string>().data(), sidesite.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

std::optional<MjsWrap> mjs_wrapJoint_wrapper(MjsTendon& tendon, const String& name, double coef) {
  CHECK_VAL(name);
  mjsWrap* result = mjs_wrapJoint(tendon.get(), name.as<const std::string>().data(), coef);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

std::optional<MjsWrap> mjs_wrapPulley_wrapper(MjsTendon& tendon, double divisor) {
  mjsWrap* result = mjs_wrapPulley(tendon.get(), divisor);
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

std::optional<MjsWrap> mjs_wrapSite_wrapper(MjsTendon& tendon, const String& name) {
  CHECK_VAL(name);
  mjsWrap* result = mjs_wrapSite(tendon.get(), name.as<const std::string>().data());
  if (result == nullptr) {
    return std::nullopt;
  }
  return MjsWrap(result);
}

mjtNum mju_L1_wrapper(const NumberArray& vec, int n) {
  UNPACK_ARRAY(mjtNum, vec);
  return mju_L1(vec_.data(), n);
}

void mju_add_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  int n = res_.size();
  mju_add(res_.data(), vec1_.data(), vec2_.data(), n);
}

void mju_add3_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  mju_add3(res_.data(), vec1_.data(), vec2_.data());
}

void mju_addScl_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2, mjtNum scl) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  int n = res_.size();
  mju_addScl(res_.data(), vec1_.data(), vec2_.data(), scl, n);
}

void mju_addScl3_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2, mjtNum scl) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  mju_addScl3(res_.data(), vec1_.data(), vec2_.data(), scl);
}

void mju_addTo_wrapper(const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  int n = res_.size();
  mju_addTo(res_.data(), vec_.data(), n);
}

void mju_addTo3_wrapper(const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  mju_addTo3(res_.data(), vec_.data());
}

void mju_addToScl_wrapper(const val& res, const NumberArray& vec, mjtNum scl) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  int n = res_.size();
  mju_addToScl(res_.data(), vec_.data(), scl, n);
}

void mju_addToScl3_wrapper(const val& res, const NumberArray& vec, mjtNum scl) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  mju_addToScl3(res_.data(), vec_.data(), scl);
}

void mju_axisAngle2Quat_wrapper(const val& res, const NumberArray& axis, mjtNum angle) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, axis);
  mju_axisAngle2Quat(res_.data(), axis_.data(), angle);
}

void mju_band2Dense_wrapper(const val& res, const NumberArray& mat, int ntotal, int nband, int ndense, mjtByte flg_sym) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal * ntotal);
  mju_band2Dense(res_.data(), mat_.data(), ntotal, nband, ndense, flg_sym);
}

void mju_bandMulMatVec_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec, int ntotal, int nband, int ndense, int nvec, mjtByte flg_sym) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal * nvec);
  CHECK_SIZE(vec, ntotal * nvec);
  mju_bandMulMatVec(res_.data(), mat_.data(), vec_.data(), ntotal, nband, ndense, nvec, flg_sym);
}

int mju_boxQP_wrapper(const val& res, const val& R, const val& index, const NumberArray& H, const NumberArray& g, const NumberArray& lower, const NumberArray& upper) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_VALUE(mjtNum, R);
  UNPACK_NULLABLE_VALUE(int, index);
  UNPACK_ARRAY(mjtNum, H);
  UNPACK_ARRAY(mjtNum, g);
  UNPACK_NULLABLE_ARRAY(mjtNum, lower);
  UNPACK_NULLABLE_ARRAY(mjtNum, upper);
  CHECK_SIZES(lower, res);
  CHECK_SIZES(upper, res);
  CHECK_SIZES(index, res);
  CHECK_SIZE(R, res_.size() * (res_.size() + 7))
  CHECK_PERFECT_SQUARE(H);
  CHECK_SIZES(g, res);
  int n = res_.size();
  return mju_boxQP(res_.data(), R_.data(), index_.data(), H_.data(), g_.data(), n, lower_.data(), upper_.data());
}

int mju_cholFactor_wrapper(const val& mat, mjtNum mindiag) {
  UNPACK_VALUE(mjtNum, mat);
  CHECK_PERFECT_SQUARE(mat);
  int n = mat_sqrt;
  return mju_cholFactor(mat_.data(), n, mindiag);
}

mjtNum mju_cholFactorBand_wrapper(const val& mat, int ntotal, int nband, int ndense, mjtNum diagadd, mjtNum diagmul) {
  UNPACK_VALUE(mjtNum, mat);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  return mju_cholFactorBand(mat_.data(), ntotal, nband, ndense, diagadd, diagmul);
}

void mju_cholSolve_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_PERFECT_SQUARE(mat);
  CHECK_SIZE(res, mat_sqrt);
  CHECK_SIZE(vec, mat_sqrt);
  int n = mat_sqrt;
  mju_cholSolve(res_.data(), mat_.data(), vec_.data(), n);
}

void mju_cholSolveBand_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec, int ntotal, int nband, int ndense) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, (ntotal - ndense) * nband + ndense * ntotal);
  CHECK_SIZE(res, ntotal);
  CHECK_SIZE(vec, ntotal);
  mju_cholSolveBand(res_.data(), mat_.data(), vec_.data(), ntotal, nband, ndense);
}

int mju_cholUpdate_wrapper(const val& mat, const val& x, int flg_plus) {
  UNPACK_VALUE(mjtNum, mat);
  UNPACK_VALUE(mjtNum, x);
  CHECK_PERFECT_SQUARE(mat);
  CHECK_SIZE(x, mat_sqrt);
  int n = mat_sqrt;
  return mju_cholUpdate(mat_.data(), x_.data(), n, flg_plus);
}

void mju_copy_wrapper(const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  int n = res_.size();
  mju_copy(res_.data(), vec_.data(), n);
}

void mju_copy3_wrapper(const val& res, const NumberArray& data) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, data);
  mju_copy3(res_.data(), data_.data());
}

void mju_copy4_wrapper(const val& res, const NumberArray& data) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, data);
  mju_copy4(res_.data(), data_.data());
}

void mju_cross_wrapper(const val& res, const NumberArray& a, const NumberArray& b) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, a);
  UNPACK_ARRAY(mjtNum, b);
  mju_cross(res_.data(), a_.data(), b_.data());
}

void mju_d2n_wrapper(const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(double, vec);
  CHECK_SIZES(res, vec);
  int n = res_.size();
  mju_d2n(res_.data(), vec_.data(), n);
}

void mju_decodePyramid_wrapper(const val& force, const NumberArray& pyramid, const NumberArray& mu) {
  UNPACK_VALUE(mjtNum, force);
  UNPACK_ARRAY(mjtNum, pyramid);
  UNPACK_ARRAY(mjtNum, mu);
  CHECK_SIZE(pyramid, 2 * mu_.size());
  CHECK_SIZE(force, mu_.size() + 1);
  int dim = mu_.size();
  mju_decodePyramid(force_.data(), pyramid_.data(), mu_.data(), dim);
}

void mju_dense2Band_wrapper(const val& res, const NumberArray& mat, int ntotal, int nband, int ndense) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, ntotal * ntotal);
  CHECK_SIZE(res, (ntotal - ndense) * nband + ndense * ntotal);
  mju_dense2Band(res_.data(), mat_.data(), ntotal, nband, ndense);
}

int mju_dense2sparse_wrapper(const val& res, const NumberArray& mat, int nr, int nc, const val& rownnz, const val& rowadr, const val& colind) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_VALUE(int, rownnz);
  UNPACK_VALUE(int, rowadr);
  UNPACK_VALUE(int, colind);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(rownnz, nr);
  CHECK_SIZE(rowadr, nr);
  CHECK_SIZE(colind, res_.size());
  int nnz = res_.size();
  return mju_dense2sparse(res_.data(), mat_.data(), nr, nc, rownnz_.data(), rowadr_.data(), colind_.data(), nnz);
}

void mju_derivQuat_wrapper(const val& res, const NumberArray& quat, const NumberArray& vel) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, vel);
  mju_derivQuat(res_.data(), quat_.data(), vel_.data());
}

mjtNum mju_dist3_wrapper(const NumberArray& pos1, const NumberArray& pos2) {
  UNPACK_ARRAY(mjtNum, pos1);
  UNPACK_ARRAY(mjtNum, pos2);
  return mju_dist3(pos1_.data(), pos2_.data());
}

mjtNum mju_dot_wrapper(const NumberArray& vec1, const NumberArray& vec2) {
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(vec1, vec2);
  int n = vec1_.size();
  return mju_dot(vec1_.data(), vec2_.data(), n);
}

mjtNum mju_dot3_wrapper(const NumberArray& vec1, const NumberArray& vec2) {
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  return mju_dot3(vec1_.data(), vec2_.data());
}

int mju_eig3_wrapper(const val& eigval, const val& eigvec, const val& quat, const NumberArray& mat) {
  UNPACK_VALUE(mjtNum, eigval);
  UNPACK_VALUE(mjtNum, eigvec);
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, mat);
  return mju_eig3(eigval_.data(), eigvec_.data(), quat_.data(), mat_.data());
}

void mju_encodePyramid_wrapper(const val& pyramid, const NumberArray& force, const NumberArray& mu) {
  UNPACK_VALUE(mjtNum, pyramid);
  UNPACK_ARRAY(mjtNum, force);
  UNPACK_ARRAY(mjtNum, mu);
  CHECK_SIZE(pyramid, 2 * mu_.size());
  CHECK_SIZE(force, mu_.size() + 1);
  int dim = mu_.size();
  mju_encodePyramid(pyramid_.data(), force_.data(), mu_.data(), dim);
}

void mju_euler2Quat_wrapper(const val& quat, const NumberArray& euler, const String& seq) {
  CHECK_VAL(seq);
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, euler);
  mju_euler2Quat(quat_.data(), euler_.data(), seq.as<const std::string>().data());
}

void mju_eye_wrapper(const val& mat) {
  UNPACK_VALUE(mjtNum, mat);
  CHECK_PERFECT_SQUARE(mat);
  int n = mat_sqrt;
  mju_eye(mat_.data(), n);
}

void mju_f2n_wrapper(const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(float, vec);
  CHECK_SIZES(res, vec);
  int n = res_.size();
  mju_f2n(res_.data(), vec_.data(), n);
}

void mju_fill_wrapper(const val& res, mjtNum val) {
  UNPACK_VALUE(mjtNum, res);
  int n = res_.size();
  mju_fill(res_.data(), val, n);
}

void mju_insertionSort_wrapper(const val& list) {
  UNPACK_VALUE(mjtNum, list);
  int n = list_.size();
  mju_insertionSort(list_.data(), n);
}

void mju_insertionSortInt_wrapper(const val& list) {
  UNPACK_VALUE(int, list);
  int n = list_.size();
  mju_insertionSortInt(list_.data(), n);
}

int mju_isZero_wrapper(const NumberArray& vec, int n) {
  UNPACK_ARRAY(mjtNum, vec);
  return mju_isZero(vec_.data(), n);
}

void mju_mat2Quat_wrapper(const val& quat, const NumberArray& mat) {
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, mat);
  mju_mat2Quat(quat_.data(), mat_.data());
}

int mju_mat2Rot_wrapper(const val& quat, const NumberArray& mat) {
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, mat);
  return mju_mat2Rot(quat_.data(), mat_.data());
}

void mju_mulMatMat_wrapper(const val& res, const NumberArray& mat1, const NumberArray& mat2, int r1, int c1, int c2) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat1);
  UNPACK_ARRAY(mjtNum, mat2);
  CHECK_SIZE(res, r1 * c2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, c1 * c2);
  mju_mulMatMat(res_.data(), mat1_.data(), mat2_.data(), r1, c1, c2);
}

void mju_mulMatMatT_wrapper(const val& res, const NumberArray& mat1, const NumberArray& mat2, int r1, int c1, int r2) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat1);
  UNPACK_ARRAY(mjtNum, mat2);
  CHECK_SIZE(res, r1 * r2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, r2 * c1);
  mju_mulMatMatT(res_.data(), mat1_.data(), mat2_.data(), r1, c1, r2);
}

void mju_mulMatTMat_wrapper(const val& res, const NumberArray& mat1, const NumberArray& mat2, int r1, int c1, int c2) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat1);
  UNPACK_ARRAY(mjtNum, mat2);
  CHECK_SIZE(res, c1 * c2);
  CHECK_SIZE(mat1, r1 * c1);
  CHECK_SIZE(mat2, r1 * c2);
  mju_mulMatTMat(res_.data(), mat1_.data(), mat2_.data(), r1, c1, c2);
}

void mju_mulMatTVec_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec, int nr, int nc) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nc);
  CHECK_SIZE(vec, nr);
  mju_mulMatTVec(res_.data(), mat_.data(), vec_.data(), nr, nc);
}

void mju_mulMatTVec3_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  mju_mulMatTVec3(res_.data(), mat_.data(), vec_.data());
}

void mju_mulMatVec_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec, int nr, int nc) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nr);
  CHECK_SIZE(vec, nc);
  mju_mulMatVec(res_.data(), mat_.data(), vec_.data(), nr, nc);
}

void mju_mulMatVec3_wrapper(const val& res, const NumberArray& mat, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec);
  mju_mulMatVec3(res_.data(), mat_.data(), vec_.data());
}

void mju_mulPose_wrapper(const val& posres, const val& quatres, const NumberArray& pos1, const NumberArray& quat1, const NumberArray& pos2, const NumberArray& quat2) {
  UNPACK_VALUE(mjtNum, posres);
  UNPACK_VALUE(mjtNum, quatres);
  UNPACK_ARRAY(mjtNum, pos1);
  UNPACK_ARRAY(mjtNum, quat1);
  UNPACK_ARRAY(mjtNum, pos2);
  UNPACK_ARRAY(mjtNum, quat2);
  mju_mulPose(posres_.data(), quatres_.data(), pos1_.data(), quat1_.data(), pos2_.data(), quat2_.data());
}

void mju_mulQuat_wrapper(const val& res, const NumberArray& quat1, const NumberArray& quat2) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat1);
  UNPACK_ARRAY(mjtNum, quat2);
  mju_mulQuat(res_.data(), quat1_.data(), quat2_.data());
}

void mju_mulQuatAxis_wrapper(const val& res, const NumberArray& quat, const NumberArray& axis) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, axis);
  mju_mulQuatAxis(res_.data(), quat_.data(), axis_.data());
}

mjtNum mju_mulVecMatVec_wrapper(const NumberArray& vec1, const NumberArray& mat, const NumberArray& vec2) {
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(vec1, vec2);
  CHECK_SIZE(mat, vec1_.size() * vec2_.size());
  int n = vec1_.size();
  return mju_mulVecMatVec(vec1_.data(), mat_.data(), vec2_.data(), n);
}

mjtNum mju_muscleBias_wrapper(mjtNum len, const NumberArray& lengthrange, mjtNum acc0, const NumberArray& prm) {
  UNPACK_ARRAY(mjtNum, lengthrange);
  UNPACK_ARRAY(mjtNum, prm);
  return mju_muscleBias(len, lengthrange_.data(), acc0, prm_.data());
}

mjtNum mju_muscleDynamics_wrapper(mjtNum ctrl, mjtNum act, const NumberArray& prm) {
  UNPACK_ARRAY(mjtNum, prm);
  return mju_muscleDynamics(ctrl, act, prm_.data());
}

mjtNum mju_muscleGain_wrapper(mjtNum len, mjtNum vel, const NumberArray& lengthrange, mjtNum acc0, const NumberArray& prm) {
  UNPACK_ARRAY(mjtNum, lengthrange);
  UNPACK_ARRAY(mjtNum, prm);
  return mju_muscleGain(len, vel, lengthrange_.data(), acc0, prm_.data());
}

void mju_n2d_wrapper(const val& res, const NumberArray& vec) {
  UNPACK_VALUE(double, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  int n = res_.size();
  mju_n2d(res_.data(), vec_.data(), n);
}

void mju_n2f_wrapper(const val& res, const NumberArray& vec) {
  UNPACK_VALUE(float, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  int n = res_.size();
  mju_n2f(res_.data(), vec_.data(), n);
}

void mju_negPose_wrapper(const val& posres, const val& quatres, const NumberArray& pos, const NumberArray& quat) {
  UNPACK_VALUE(mjtNum, posres);
  UNPACK_VALUE(mjtNum, quatres);
  UNPACK_ARRAY(mjtNum, pos);
  UNPACK_ARRAY(mjtNum, quat);
  mju_negPose(posres_.data(), quatres_.data(), pos_.data(), quat_.data());
}

void mju_negQuat_wrapper(const val& res, const NumberArray& quat) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  mju_negQuat(res_.data(), quat_.data());
}

mjtNum mju_norm_wrapper(const NumberArray& res, int n) {
  UNPACK_ARRAY(mjtNum, res);
  return mju_norm(res_.data(), n);
}

mjtNum mju_norm3_wrapper(const NumberArray& vec) {
  UNPACK_ARRAY(mjtNum, vec);
  return mju_norm3(vec_.data());
}

mjtNum mju_normalize_wrapper(const val& res, int n) {
  UNPACK_VALUE(mjtNum, res);
  return mju_normalize(res_.data(), n);
}

mjtNum mju_normalize3_wrapper(const val& vec) {
  UNPACK_VALUE(mjtNum, vec);
  return mju_normalize3(vec_.data());
}

mjtNum mju_normalize4_wrapper(const val& vec) {
  UNPACK_VALUE(mjtNum, vec);
  return mju_normalize4(vec_.data());
}

void mju_printMat_wrapper(const NumberArray& mat, int nr, int nc) {
  UNPACK_ARRAY(mjtNum, mat);
  mju_printMat(mat_.data(), nr, nc);
}

void mju_printMatSparse_wrapper(const NumberArray& mat, const NumberArray& rownnz, const NumberArray& rowadr, const NumberArray& colind) {
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(int, rownnz);
  UNPACK_ARRAY(int, rowadr);
  UNPACK_ARRAY(int, colind);
  CHECK_SIZES(rownnz, rowadr);
  int nr = rowadr_.size();
  mju_printMatSparse(mat_.data(), nr, rownnz_.data(), rowadr_.data(), colind_.data());
}

void mju_quat2Mat_wrapper(const val& res, const NumberArray& quat) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  mju_quat2Mat(res_.data(), quat_.data());
}

void mju_quat2Vel_wrapper(const val& res, const NumberArray& quat, mjtNum dt) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, quat);
  mju_quat2Vel(res_.data(), quat_.data(), dt);
}

void mju_quatIntegrate_wrapper(const val& quat, const NumberArray& vel, mjtNum scale) {
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, vel);
  mju_quatIntegrate(quat_.data(), vel_.data(), scale);
}

void mju_quatZ2Vec_wrapper(const val& quat, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, vec);
  mju_quatZ2Vec(quat_.data(), vec_.data());
}

mjtNum mju_rayFlex_wrapper(const MjModel& m, const MjData& d, int flex_layer, mjtByte flg_vert, mjtByte flg_edge, mjtByte flg_face, mjtByte flg_skin, int flexid, const NumberArray& pnt, const NumberArray& vec, const val& vertid) {
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_NULLABLE_VALUE(int, vertid);
  return mju_rayFlex(m.get(), d.get(), flex_layer, flg_vert, flg_edge, flg_face, flg_skin, flexid, pnt_.data(), vec_.data(), vertid_.data());
}

mjtNum mju_rayGeom_wrapper(const NumberArray& pos, const NumberArray& mat, const NumberArray& size, const NumberArray& pnt, const NumberArray& vec, int geomtype) {
  UNPACK_ARRAY(mjtNum, pos);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, size);
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  return mju_rayGeom(pos_.data(), mat_.data(), size_.data(), pnt_.data(), vec_.data(), geomtype);
}

mjtNum mju_raySkin_wrapper(int nface, int nvert, const NumberArray& face, const NumberArray& vert, const NumberArray& pnt, const NumberArray& vec, const val& vertid) {
  UNPACK_ARRAY(int, face);
  UNPACK_ARRAY(float, vert);
  UNPACK_ARRAY(mjtNum, pnt);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_NULLABLE_VALUE(int, vertid);
  return mju_raySkin(nface, nvert, face_.data(), vert_.data(), pnt_.data(), vec_.data(), vertid_.data());
}

void mju_rotVecQuat_wrapper(const val& res, const NumberArray& vec, const NumberArray& quat) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_ARRAY(mjtNum, quat);
  mju_rotVecQuat(res_.data(), vec_.data(), quat_.data());
}

void mju_scl_wrapper(const val& res, const NumberArray& vec, mjtNum scl) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  int n = res_.size();
  mju_scl(res_.data(), vec_.data(), scl, n);
}

void mju_scl3_wrapper(const val& res, const NumberArray& vec, mjtNum scl) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  mju_scl3(res_.data(), vec_.data(), scl);
}

void mju_sparse2dense_wrapper(const val& res, const NumberArray& mat, int nr, int nc, const NumberArray& rownnz, const NumberArray& rowadr, const NumberArray& colind) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(int, rownnz);
  UNPACK_ARRAY(int, rowadr);
  UNPACK_ARRAY(int, colind);
  CHECK_SIZE(res, nr * nc);
  CHECK_SIZE(rownnz, nr);
  CHECK_SIZE(rowadr, nr);
  mju_sparse2dense(res_.data(), mat_.data(), nr, nc, rownnz_.data(), rowadr_.data(), colind_.data());
}

void mju_sqrMatTD_wrapper(const val& res, const NumberArray& mat, const NumberArray& diag, int nr, int nc) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  UNPACK_ARRAY(mjtNum, diag);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nc * nc);
  CHECK_SIZE(diag, nr);
  mju_sqrMatTD(res_.data(), mat_.data(), diag_.data(), nr, nc);
}

mjtNum mju_standardNormal_wrapper(const val& num2) {
  UNPACK_VALUE(mjtNum, num2);
  return mju_standardNormal(num2_.data());
}

int mju_str2Type_wrapper(const String& str) {
  CHECK_VAL(str);
  return mju_str2Type(str.as<const std::string>().data());
}

void mju_sub_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  CHECK_SIZES(res, vec1);
  CHECK_SIZES(res, vec2);
  int n = res_.size();
  mju_sub(res_.data(), vec1_.data(), vec2_.data(), n);
}

void mju_sub3_wrapper(const val& res, const NumberArray& vec1, const NumberArray& vec2) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec1);
  UNPACK_ARRAY(mjtNum, vec2);
  mju_sub3(res_.data(), vec1_.data(), vec2_.data());
}

void mju_subFrom_wrapper(const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  CHECK_SIZES(res, vec);
  int n = res_.size();
  mju_subFrom(res_.data(), vec_.data(), n);
}

void mju_subFrom3_wrapper(const val& res, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  mju_subFrom3(res_.data(), vec_.data());
}

void mju_subQuat_wrapper(const val& res, const NumberArray& qa, const NumberArray& qb) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, qa);
  UNPACK_ARRAY(mjtNum, qb);
  mju_subQuat(res_.data(), qa_.data(), qb_.data());
}

mjtNum mju_sum_wrapper(const NumberArray& vec, int n) {
  UNPACK_ARRAY(mjtNum, vec);
  return mju_sum(vec_.data(), n);
}

void mju_symmetrize_wrapper(const val& res, const NumberArray& mat, int n) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, n * n);
  CHECK_SIZE(res, n * n);
  mju_symmetrize(res_.data(), mat_.data(), n);
}

void mju_transformSpatial_wrapper(const val& res, const NumberArray& vec, int flg_force, const NumberArray& newpos, const NumberArray& oldpos, const NumberArray& rotnew2old) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_ARRAY(mjtNum, newpos);
  UNPACK_ARRAY(mjtNum, oldpos);
  UNPACK_NULLABLE_ARRAY(mjtNum, rotnew2old);
  mju_transformSpatial(res_.data(), vec_.data(), flg_force, newpos_.data(), oldpos_.data(), rotnew2old_.data());
}

void mju_transpose_wrapper(const val& res, const NumberArray& mat, int nr, int nc) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, mat);
  CHECK_SIZE(mat, nr * nc);
  CHECK_SIZE(res, nr * nc);
  mju_transpose(res_.data(), mat_.data(), nr, nc);
}

void mju_trnVecPose_wrapper(const val& res, const NumberArray& pos, const NumberArray& quat, const NumberArray& vec) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, pos);
  UNPACK_ARRAY(mjtNum, quat);
  UNPACK_ARRAY(mjtNum, vec);
  mju_trnVecPose(res_.data(), pos_.data(), quat_.data(), vec_.data());
}

std::string mju_type2Str_wrapper(int type) {
  return std::string(mju_type2Str(type));
}

void mju_unit4_wrapper(const val& res) {
  UNPACK_VALUE(mjtNum, res);
  mju_unit4(res_.data());
}

std::string mju_warningText_wrapper(int warning, size_t info) {
  return std::string(mju_warningText(warning, info));
}

void mju_writeLog_wrapper(const String& type, const String& msg) {
  CHECK_VAL(type);
  CHECK_VAL(msg);
  mju_writeLog(type.as<const std::string>().data(), msg.as<const std::string>().data());
}

std::string mju_writeNumBytes_wrapper(size_t nbytes) {
  return std::string(mju_writeNumBytes(nbytes));
}

void mju_zero_wrapper(const val& res, int n) {
  UNPACK_VALUE(mjtNum, res);
  mju_zero(res_.data(), n);
}

void mju_zero3_wrapper(const val& res) {
  UNPACK_VALUE(mjtNum, res);
  mju_zero3(res_.data());
}

void mju_zero4_wrapper(const val& res) {
  UNPACK_VALUE(mjtNum, res);
  mju_zero4(res_.data());
}

void mjv_addGeoms_wrapper(const MjModel& m, MjData& d, const MjvOption& opt, const MjvPerturb& pert, int catmask, MjvScene& scn) {
  mjv_addGeoms(m.get(), d.get(), opt.get(), pert.get(), catmask, scn.get());
}

void mjv_alignToCamera_wrapper(const val& res, const NumberArray& vec, const NumberArray& forward) {
  UNPACK_VALUE(mjtNum, res);
  UNPACK_ARRAY(mjtNum, vec);
  UNPACK_ARRAY(mjtNum, forward);
  mjv_alignToCamera(res_.data(), vec_.data(), forward_.data());
}

void mjv_applyPerturbForce_wrapper(const MjModel& m, MjData& d, const MjvPerturb& pert) {
  mjv_applyPerturbForce(m.get(), d.get(), pert.get());
}

void mjv_applyPerturbPose_wrapper(const MjModel& m, MjData& d, const MjvPerturb& pert, int flg_paused) {
  mjv_applyPerturbPose(m.get(), d.get(), pert.get(), flg_paused);
}

void mjv_cameraFrame_wrapper(const val& headpos, const val& forward, const val& up, const val& right, const MjData& d, const MjvCamera& cam) {
  UNPACK_NULLABLE_VALUE(mjtNum, headpos);
  UNPACK_NULLABLE_VALUE(mjtNum, forward);
  UNPACK_NULLABLE_VALUE(mjtNum, up);
  UNPACK_NULLABLE_VALUE(mjtNum, right);
  mjv_cameraFrame(headpos_.data(), forward_.data(), up_.data(), right_.data(), d.get(), cam.get());
}

void mjv_cameraFrustum_wrapper(const val& zver, const val& zhor, const val& zclip, const MjModel& m, const MjvCamera& cam) {
  UNPACK_NULLABLE_VALUE(float, zver);
  UNPACK_NULLABLE_VALUE(float, zhor);
  UNPACK_NULLABLE_VALUE(float, zclip);
  mjv_cameraFrustum(zver_.data(), zhor_.data(), zclip_.data(), m.get(), cam.get());
}

void mjv_cameraInModel_wrapper(const val& headpos, const val& forward, const val& up, const MjvScene& scn) {
  UNPACK_VALUE(mjtNum, headpos);
  UNPACK_VALUE(mjtNum, forward);
  UNPACK_VALUE(mjtNum, up);
  mjv_cameraInModel(headpos_.data(), forward_.data(), up_.data(), scn.get());
}

void mjv_cameraInRoom_wrapper(const val& headpos, const val& forward, const val& up, const MjvScene& scn) {
  UNPACK_VALUE(mjtNum, headpos);
  UNPACK_VALUE(mjtNum, forward);
  UNPACK_VALUE(mjtNum, up);
  mjv_cameraInRoom(headpos_.data(), forward_.data(), up_.data(), scn.get());
}

void mjv_connector_wrapper(MjvGeom& geom, int type, mjtNum width, const NumberArray& from, const NumberArray& to) {
  UNPACK_ARRAY(mjtNum, from);
  UNPACK_ARRAY(mjtNum, to);
  mjv_connector(geom.get(), type, width, from_.data(), to_.data());
}

void mjv_defaultCamera_wrapper(MjvCamera& cam) {
  mjv_defaultCamera(cam.get());
}

void mjv_defaultFigure_wrapper(MjvFigure& fig) {
  mjv_defaultFigure(fig.get());
}

void mjv_defaultFreeCamera_wrapper(const MjModel& m, MjvCamera& cam) {
  mjv_defaultFreeCamera(m.get(), cam.get());
}

void mjv_defaultOption_wrapper(MjvOption& opt) {
  mjv_defaultOption(opt.get());
}

void mjv_defaultPerturb_wrapper(MjvPerturb& pert) {
  mjv_defaultPerturb(pert.get());
}

mjtNum mjv_frustumHeight_wrapper(const MjvScene& scn) {
  return mjv_frustumHeight(scn.get());
}

void mjv_initGeom_wrapper(MjvGeom& geom, int type, const NumberArray& size, const NumberArray& pos, const NumberArray& mat, const NumberArray& rgba) {
  UNPACK_NULLABLE_ARRAY(mjtNum, size);
  UNPACK_NULLABLE_ARRAY(mjtNum, pos);
  UNPACK_NULLABLE_ARRAY(mjtNum, mat);
  UNPACK_NULLABLE_ARRAY(float, rgba);
  mjv_initGeom(geom.get(), type, size_.data(), pos_.data(), mat_.data(), rgba_.data());
}

void mjv_initPerturb_wrapper(const MjModel& m, MjData& d, const MjvScene& scn, MjvPerturb& pert) {
  mjv_initPerturb(m.get(), d.get(), scn.get(), pert.get());
}

void mjv_makeLights_wrapper(const MjModel& m, const MjData& d, MjvScene& scn) {
  mjv_makeLights(m.get(), d.get(), scn.get());
}

void mjv_model2room_wrapper(const val& roompos, const val& roomquat, const NumberArray& modelpos, const NumberArray& modelquat, const MjvScene& scn) {
  UNPACK_VALUE(mjtNum, roompos);
  UNPACK_VALUE(mjtNum, roomquat);
  UNPACK_ARRAY(mjtNum, modelpos);
  UNPACK_ARRAY(mjtNum, modelquat);
  mjv_model2room(roompos_.data(), roomquat_.data(), modelpos_.data(), modelquat_.data(), scn.get());
}

void mjv_moveCamera_wrapper(const MjModel& m, int action, mjtNum reldx, mjtNum reldy, const MjvScene& scn, MjvCamera& cam) {
  mjv_moveCamera(m.get(), action, reldx, reldy, scn.get(), cam.get());
}

void mjv_moveModel_wrapper(const MjModel& m, int action, mjtNum reldx, mjtNum reldy, const NumberArray& roomup, MjvScene& scn) {
  UNPACK_ARRAY(mjtNum, roomup);
  mjv_moveModel(m.get(), action, reldx, reldy, roomup_.data(), scn.get());
}

void mjv_movePerturb_wrapper(const MjModel& m, const MjData& d, int action, mjtNum reldx, mjtNum reldy, const MjvScene& scn, MjvPerturb& pert) {
  mjv_movePerturb(m.get(), d.get(), action, reldx, reldy, scn.get(), pert.get());
}

void mjv_room2model_wrapper(const val& modelpos, const val& modelquat, const NumberArray& roompos, const NumberArray& roomquat, const MjvScene& scn) {
  UNPACK_VALUE(mjtNum, modelpos);
  UNPACK_VALUE(mjtNum, modelquat);
  UNPACK_ARRAY(mjtNum, roompos);
  UNPACK_ARRAY(mjtNum, roomquat);
  mjv_room2model(modelpos_.data(), modelquat_.data(), roompos_.data(), roomquat_.data(), scn.get());
}

int mjv_select_wrapper(const MjModel& m, const MjData& d, const MjvOption& vopt, mjtNum aspectratio, mjtNum relx, mjtNum rely, const MjvScene& scn, const val& selpnt, const val& geomid, const val& flexid, const val& skinid) {
  UNPACK_VALUE(mjtNum, selpnt);
  UNPACK_NULLABLE_VALUE(int, geomid);
  UNPACK_NULLABLE_VALUE(int, flexid);
  UNPACK_NULLABLE_VALUE(int, skinid);
  return mjv_select(m.get(), d.get(), vopt.get(), aspectratio, relx, rely, scn.get(), selpnt_.data(), geomid_.data(), flexid_.data(), skinid_.data());
}

void mjv_updateCamera_wrapper(const MjModel& m, const MjData& d, MjvCamera& cam, MjvScene& scn) {
  mjv_updateCamera(m.get(), d.get(), cam.get(), scn.get());
}

void mjv_updateScene_wrapper(const MjModel& m, MjData& d, const MjvOption& opt, const MjvPerturb& pert, MjvCamera& cam, int catmask, MjvScene& scn) {
  mjv_updateScene(m.get(), d.get(), opt.get(), pert.get(), cam.get(), catmask, scn.get());
}

void mjv_updateSkin_wrapper(const MjModel& m, const MjData& d, MjvScene& scn) {
  mjv_updateSkin(m.get(), d.get(), scn.get());
}

EMSCRIPTEN_BINDINGS(mujoco_bindings) {
  enum_<mjtAlignFree>("mjtAlignFree")
    .value("mjALIGNFREE_FALSE", mjALIGNFREE_FALSE)
    .value("mjALIGNFREE_TRUE", mjALIGNFREE_TRUE)
    .value("mjALIGNFREE_AUTO", mjALIGNFREE_AUTO);
  enum_<mjtBias>("mjtBias")
    .value("mjBIAS_NONE", mjBIAS_NONE)
    .value("mjBIAS_AFFINE", mjBIAS_AFFINE)
    .value("mjBIAS_MUSCLE", mjBIAS_MUSCLE)
    .value("mjBIAS_USER", mjBIAS_USER);
  enum_<mjtBuiltin>("mjtBuiltin")
    .value("mjBUILTIN_NONE", mjBUILTIN_NONE)
    .value("mjBUILTIN_GRADIENT", mjBUILTIN_GRADIENT)
    .value("mjBUILTIN_CHECKER", mjBUILTIN_CHECKER)
    .value("mjBUILTIN_FLAT", mjBUILTIN_FLAT);
  enum_<mjtButton>("mjtButton")
    .value("mjBUTTON_NONE", mjBUTTON_NONE)
    .value("mjBUTTON_LEFT", mjBUTTON_LEFT)
    .value("mjBUTTON_RIGHT", mjBUTTON_RIGHT)
    .value("mjBUTTON_MIDDLE", mjBUTTON_MIDDLE);
  enum_<mjtCamLight>("mjtCamLight")
    .value("mjCAMLIGHT_FIXED", mjCAMLIGHT_FIXED)
    .value("mjCAMLIGHT_TRACK", mjCAMLIGHT_TRACK)
    .value("mjCAMLIGHT_TRACKCOM", mjCAMLIGHT_TRACKCOM)
    .value("mjCAMLIGHT_TARGETBODY", mjCAMLIGHT_TARGETBODY)
    .value("mjCAMLIGHT_TARGETBODYCOM", mjCAMLIGHT_TARGETBODYCOM);
  enum_<mjtCamera>("mjtCamera")
    .value("mjCAMERA_FREE", mjCAMERA_FREE)
    .value("mjCAMERA_TRACKING", mjCAMERA_TRACKING)
    .value("mjCAMERA_FIXED", mjCAMERA_FIXED)
    .value("mjCAMERA_USER", mjCAMERA_USER);
  enum_<mjtCatBit>("mjtCatBit")
    .value("mjCAT_STATIC", mjCAT_STATIC)
    .value("mjCAT_DYNAMIC", mjCAT_DYNAMIC)
    .value("mjCAT_DECOR", mjCAT_DECOR)
    .value("mjCAT_ALL", mjCAT_ALL);
  enum_<mjtColorSpace>("mjtColorSpace")
    .value("mjCOLORSPACE_AUTO", mjCOLORSPACE_AUTO)
    .value("mjCOLORSPACE_LINEAR", mjCOLORSPACE_LINEAR)
    .value("mjCOLORSPACE_SRGB", mjCOLORSPACE_SRGB);
  enum_<mjtConDataField>("mjtConDataField")
    .value("mjCONDATA_FOUND", mjCONDATA_FOUND)
    .value("mjCONDATA_FORCE", mjCONDATA_FORCE)
    .value("mjCONDATA_TORQUE", mjCONDATA_TORQUE)
    .value("mjCONDATA_DIST", mjCONDATA_DIST)
    .value("mjCONDATA_POS", mjCONDATA_POS)
    .value("mjCONDATA_NORMAL", mjCONDATA_NORMAL)
    .value("mjCONDATA_TANGENT", mjCONDATA_TANGENT)
    .value("mjNCONDATA", mjNCONDATA);
  enum_<mjtCone>("mjtCone")
    .value("mjCONE_PYRAMIDAL", mjCONE_PYRAMIDAL)
    .value("mjCONE_ELLIPTIC", mjCONE_ELLIPTIC);
  enum_<mjtConstraint>("mjtConstraint")
    .value("mjCNSTR_EQUALITY", mjCNSTR_EQUALITY)
    .value("mjCNSTR_FRICTION_DOF", mjCNSTR_FRICTION_DOF)
    .value("mjCNSTR_FRICTION_TENDON", mjCNSTR_FRICTION_TENDON)
    .value("mjCNSTR_LIMIT_JOINT", mjCNSTR_LIMIT_JOINT)
    .value("mjCNSTR_LIMIT_TENDON", mjCNSTR_LIMIT_TENDON)
    .value("mjCNSTR_CONTACT_FRICTIONLESS", mjCNSTR_CONTACT_FRICTIONLESS)
    .value("mjCNSTR_CONTACT_PYRAMIDAL", mjCNSTR_CONTACT_PYRAMIDAL)
    .value("mjCNSTR_CONTACT_ELLIPTIC", mjCNSTR_CONTACT_ELLIPTIC);
  enum_<mjtConstraintState>("mjtConstraintState")
    .value("mjCNSTRSTATE_SATISFIED", mjCNSTRSTATE_SATISFIED)
    .value("mjCNSTRSTATE_QUADRATIC", mjCNSTRSTATE_QUADRATIC)
    .value("mjCNSTRSTATE_LINEARNEG", mjCNSTRSTATE_LINEARNEG)
    .value("mjCNSTRSTATE_LINEARPOS", mjCNSTRSTATE_LINEARPOS)
    .value("mjCNSTRSTATE_CONE", mjCNSTRSTATE_CONE);
  enum_<mjtDataType>("mjtDataType")
    .value("mjDATATYPE_REAL", mjDATATYPE_REAL)
    .value("mjDATATYPE_POSITIVE", mjDATATYPE_POSITIVE)
    .value("mjDATATYPE_AXIS", mjDATATYPE_AXIS)
    .value("mjDATATYPE_QUATERNION", mjDATATYPE_QUATERNION);
  enum_<mjtDepthMap>("mjtDepthMap")
    .value("mjDEPTH_ZERONEAR", mjDEPTH_ZERONEAR)
    .value("mjDEPTH_ZEROFAR", mjDEPTH_ZEROFAR);
  enum_<mjtDisableBit>("mjtDisableBit")
    .value("mjDSBL_CONSTRAINT", mjDSBL_CONSTRAINT)
    .value("mjDSBL_EQUALITY", mjDSBL_EQUALITY)
    .value("mjDSBL_FRICTIONLOSS", mjDSBL_FRICTIONLOSS)
    .value("mjDSBL_LIMIT", mjDSBL_LIMIT)
    .value("mjDSBL_CONTACT", mjDSBL_CONTACT)
    .value("mjDSBL_SPRING", mjDSBL_SPRING)
    .value("mjDSBL_DAMPER", mjDSBL_DAMPER)
    .value("mjDSBL_GRAVITY", mjDSBL_GRAVITY)
    .value("mjDSBL_CLAMPCTRL", mjDSBL_CLAMPCTRL)
    .value("mjDSBL_WARMSTART", mjDSBL_WARMSTART)
    .value("mjDSBL_FILTERPARENT", mjDSBL_FILTERPARENT)
    .value("mjDSBL_ACTUATION", mjDSBL_ACTUATION)
    .value("mjDSBL_REFSAFE", mjDSBL_REFSAFE)
    .value("mjDSBL_SENSOR", mjDSBL_SENSOR)
    .value("mjDSBL_MIDPHASE", mjDSBL_MIDPHASE)
    .value("mjDSBL_EULERDAMP", mjDSBL_EULERDAMP)
    .value("mjDSBL_AUTORESET", mjDSBL_AUTORESET)
    .value("mjDSBL_NATIVECCD", mjDSBL_NATIVECCD)
    .value("mjDSBL_ISLAND", mjDSBL_ISLAND)
    .value("mjNDISABLE", mjNDISABLE);
  enum_<mjtDyn>("mjtDyn")
    .value("mjDYN_NONE", mjDYN_NONE)
    .value("mjDYN_INTEGRATOR", mjDYN_INTEGRATOR)
    .value("mjDYN_FILTER", mjDYN_FILTER)
    .value("mjDYN_FILTEREXACT", mjDYN_FILTEREXACT)
    .value("mjDYN_MUSCLE", mjDYN_MUSCLE)
    .value("mjDYN_USER", mjDYN_USER);
  enum_<mjtEnableBit>("mjtEnableBit")
    .value("mjENBL_OVERRIDE", mjENBL_OVERRIDE)
    .value("mjENBL_ENERGY", mjENBL_ENERGY)
    .value("mjENBL_FWDINV", mjENBL_FWDINV)
    .value("mjENBL_INVDISCRETE", mjENBL_INVDISCRETE)
    .value("mjENBL_MULTICCD", mjENBL_MULTICCD)
    .value("mjENBL_SLEEP", mjENBL_SLEEP)
    .value("mjNENABLE", mjNENABLE);
  enum_<mjtEq>("mjtEq")
    .value("mjEQ_CONNECT", mjEQ_CONNECT)
    .value("mjEQ_WELD", mjEQ_WELD)
    .value("mjEQ_JOINT", mjEQ_JOINT)
    .value("mjEQ_TENDON", mjEQ_TENDON)
    .value("mjEQ_FLEX", mjEQ_FLEX)
    .value("mjEQ_DISTANCE", mjEQ_DISTANCE);
  enum_<mjtEvent>("mjtEvent")
    .value("mjEVENT_NONE", mjEVENT_NONE)
    .value("mjEVENT_MOVE", mjEVENT_MOVE)
    .value("mjEVENT_PRESS", mjEVENT_PRESS)
    .value("mjEVENT_RELEASE", mjEVENT_RELEASE)
    .value("mjEVENT_SCROLL", mjEVENT_SCROLL)
    .value("mjEVENT_KEY", mjEVENT_KEY)
    .value("mjEVENT_RESIZE", mjEVENT_RESIZE)
    .value("mjEVENT_REDRAW", mjEVENT_REDRAW)
    .value("mjEVENT_FILESDROP", mjEVENT_FILESDROP);
  enum_<mjtFlexSelf>("mjtFlexSelf")
    .value("mjFLEXSELF_NONE", mjFLEXSELF_NONE)
    .value("mjFLEXSELF_NARROW", mjFLEXSELF_NARROW)
    .value("mjFLEXSELF_BVH", mjFLEXSELF_BVH)
    .value("mjFLEXSELF_SAP", mjFLEXSELF_SAP)
    .value("mjFLEXSELF_AUTO", mjFLEXSELF_AUTO);
  enum_<mjtFont>("mjtFont")
    .value("mjFONT_NORMAL", mjFONT_NORMAL)
    .value("mjFONT_SHADOW", mjFONT_SHADOW)
    .value("mjFONT_BIG", mjFONT_BIG);
  enum_<mjtFontScale>("mjtFontScale")
    .value("mjFONTSCALE_50", mjFONTSCALE_50)
    .value("mjFONTSCALE_100", mjFONTSCALE_100)
    .value("mjFONTSCALE_150", mjFONTSCALE_150)
    .value("mjFONTSCALE_200", mjFONTSCALE_200)
    .value("mjFONTSCALE_250", mjFONTSCALE_250)
    .value("mjFONTSCALE_300", mjFONTSCALE_300);
  enum_<mjtFrame>("mjtFrame")
    .value("mjFRAME_NONE", mjFRAME_NONE)
    .value("mjFRAME_BODY", mjFRAME_BODY)
    .value("mjFRAME_GEOM", mjFRAME_GEOM)
    .value("mjFRAME_SITE", mjFRAME_SITE)
    .value("mjFRAME_CAMERA", mjFRAME_CAMERA)
    .value("mjFRAME_LIGHT", mjFRAME_LIGHT)
    .value("mjFRAME_CONTACT", mjFRAME_CONTACT)
    .value("mjFRAME_WORLD", mjFRAME_WORLD)
    .value("mjNFRAME", mjNFRAME);
  enum_<mjtFramebuffer>("mjtFramebuffer")
    .value("mjFB_WINDOW", mjFB_WINDOW)
    .value("mjFB_OFFSCREEN", mjFB_OFFSCREEN);
  enum_<mjtGain>("mjtGain")
    .value("mjGAIN_FIXED", mjGAIN_FIXED)
    .value("mjGAIN_AFFINE", mjGAIN_AFFINE)
    .value("mjGAIN_MUSCLE", mjGAIN_MUSCLE)
    .value("mjGAIN_USER", mjGAIN_USER);
  enum_<mjtGeom>("mjtGeom")
    .value("mjGEOM_PLANE", mjGEOM_PLANE)
    .value("mjGEOM_HFIELD", mjGEOM_HFIELD)
    .value("mjGEOM_SPHERE", mjGEOM_SPHERE)
    .value("mjGEOM_CAPSULE", mjGEOM_CAPSULE)
    .value("mjGEOM_ELLIPSOID", mjGEOM_ELLIPSOID)
    .value("mjGEOM_CYLINDER", mjGEOM_CYLINDER)
    .value("mjGEOM_BOX", mjGEOM_BOX)
    .value("mjGEOM_MESH", mjGEOM_MESH)
    .value("mjGEOM_SDF", mjGEOM_SDF)
    .value("mjNGEOMTYPES", mjNGEOMTYPES)
    .value("mjGEOM_ARROW", mjGEOM_ARROW)
    .value("mjGEOM_ARROW1", mjGEOM_ARROW1)
    .value("mjGEOM_ARROW2", mjGEOM_ARROW2)
    .value("mjGEOM_LINE", mjGEOM_LINE)
    .value("mjGEOM_LINEBOX", mjGEOM_LINEBOX)
    .value("mjGEOM_FLEX", mjGEOM_FLEX)
    .value("mjGEOM_SKIN", mjGEOM_SKIN)
    .value("mjGEOM_LABEL", mjGEOM_LABEL)
    .value("mjGEOM_TRIANGLE", mjGEOM_TRIANGLE)
    .value("mjGEOM_NONE", mjGEOM_NONE);
  enum_<mjtGeomInertia>("mjtGeomInertia")
    .value("mjINERTIA_VOLUME", mjINERTIA_VOLUME)
    .value("mjINERTIA_SHELL", mjINERTIA_SHELL);
  enum_<mjtGridPos>("mjtGridPos")
    .value("mjGRID_TOPLEFT", mjGRID_TOPLEFT)
    .value("mjGRID_TOPRIGHT", mjGRID_TOPRIGHT)
    .value("mjGRID_BOTTOMLEFT", mjGRID_BOTTOMLEFT)
    .value("mjGRID_BOTTOMRIGHT", mjGRID_BOTTOMRIGHT)
    .value("mjGRID_TOP", mjGRID_TOP)
    .value("mjGRID_BOTTOM", mjGRID_BOTTOM)
    .value("mjGRID_LEFT", mjGRID_LEFT)
    .value("mjGRID_RIGHT", mjGRID_RIGHT);
  enum_<mjtInertiaFromGeom>("mjtInertiaFromGeom")
    .value("mjINERTIAFROMGEOM_FALSE", mjINERTIAFROMGEOM_FALSE)
    .value("mjINERTIAFROMGEOM_TRUE", mjINERTIAFROMGEOM_TRUE)
    .value("mjINERTIAFROMGEOM_AUTO", mjINERTIAFROMGEOM_AUTO);
  enum_<mjtIntegrator>("mjtIntegrator")
    .value("mjINT_EULER", mjINT_EULER)
    .value("mjINT_RK4", mjINT_RK4)
    .value("mjINT_IMPLICIT", mjINT_IMPLICIT)
    .value("mjINT_IMPLICITFAST", mjINT_IMPLICITFAST);
  enum_<mjtItem>("mjtItem")
    .value("mjITEM_END", mjITEM_END)
    .value("mjITEM_SECTION", mjITEM_SECTION)
    .value("mjITEM_SEPARATOR", mjITEM_SEPARATOR)
    .value("mjITEM_STATIC", mjITEM_STATIC)
    .value("mjITEM_BUTTON", mjITEM_BUTTON)
    .value("mjITEM_CHECKINT", mjITEM_CHECKINT)
    .value("mjITEM_CHECKBYTE", mjITEM_CHECKBYTE)
    .value("mjITEM_RADIO", mjITEM_RADIO)
    .value("mjITEM_RADIOLINE", mjITEM_RADIOLINE)
    .value("mjITEM_SELECT", mjITEM_SELECT)
    .value("mjITEM_SLIDERINT", mjITEM_SLIDERINT)
    .value("mjITEM_SLIDERNUM", mjITEM_SLIDERNUM)
    .value("mjITEM_EDITINT", mjITEM_EDITINT)
    .value("mjITEM_EDITNUM", mjITEM_EDITNUM)
    .value("mjITEM_EDITFLOAT", mjITEM_EDITFLOAT)
    .value("mjITEM_EDITTXT", mjITEM_EDITTXT)
    .value("mjNITEM", mjNITEM);
  enum_<mjtJacobian>("mjtJacobian")
    .value("mjJAC_DENSE", mjJAC_DENSE)
    .value("mjJAC_SPARSE", mjJAC_SPARSE)
    .value("mjJAC_AUTO", mjJAC_AUTO);
  enum_<mjtJoint>("mjtJoint")
    .value("mjJNT_FREE", mjJNT_FREE)
    .value("mjJNT_BALL", mjJNT_BALL)
    .value("mjJNT_SLIDE", mjJNT_SLIDE)
    .value("mjJNT_HINGE", mjJNT_HINGE);
  enum_<mjtLRMode>("mjtLRMode")
    .value("mjLRMODE_NONE", mjLRMODE_NONE)
    .value("mjLRMODE_MUSCLE", mjLRMODE_MUSCLE)
    .value("mjLRMODE_MUSCLEUSER", mjLRMODE_MUSCLEUSER)
    .value("mjLRMODE_ALL", mjLRMODE_ALL);
  enum_<mjtLabel>("mjtLabel")
    .value("mjLABEL_NONE", mjLABEL_NONE)
    .value("mjLABEL_BODY", mjLABEL_BODY)
    .value("mjLABEL_JOINT", mjLABEL_JOINT)
    .value("mjLABEL_GEOM", mjLABEL_GEOM)
    .value("mjLABEL_SITE", mjLABEL_SITE)
    .value("mjLABEL_CAMERA", mjLABEL_CAMERA)
    .value("mjLABEL_LIGHT", mjLABEL_LIGHT)
    .value("mjLABEL_TENDON", mjLABEL_TENDON)
    .value("mjLABEL_ACTUATOR", mjLABEL_ACTUATOR)
    .value("mjLABEL_CONSTRAINT", mjLABEL_CONSTRAINT)
    .value("mjLABEL_FLEX", mjLABEL_FLEX)
    .value("mjLABEL_SKIN", mjLABEL_SKIN)
    .value("mjLABEL_SELECTION", mjLABEL_SELECTION)
    .value("mjLABEL_SELPNT", mjLABEL_SELPNT)
    .value("mjLABEL_CONTACTPOINT", mjLABEL_CONTACTPOINT)
    .value("mjLABEL_CONTACTFORCE", mjLABEL_CONTACTFORCE)
    .value("mjLABEL_ISLAND", mjLABEL_ISLAND)
    .value("mjNLABEL", mjNLABEL);
  enum_<mjtLightType>("mjtLightType")
    .value("mjLIGHT_SPOT", mjLIGHT_SPOT)
    .value("mjLIGHT_DIRECTIONAL", mjLIGHT_DIRECTIONAL)
    .value("mjLIGHT_POINT", mjLIGHT_POINT)
    .value("mjLIGHT_IMAGE", mjLIGHT_IMAGE);
  enum_<mjtLimited>("mjtLimited")
    .value("mjLIMITED_FALSE", mjLIMITED_FALSE)
    .value("mjLIMITED_TRUE", mjLIMITED_TRUE)
    .value("mjLIMITED_AUTO", mjLIMITED_AUTO);
  enum_<mjtMark>("mjtMark")
    .value("mjMARK_NONE", mjMARK_NONE)
    .value("mjMARK_EDGE", mjMARK_EDGE)
    .value("mjMARK_CROSS", mjMARK_CROSS)
    .value("mjMARK_RANDOM", mjMARK_RANDOM);
  enum_<mjtMeshBuiltin>("mjtMeshBuiltin")
    .value("mjMESH_BUILTIN_NONE", mjMESH_BUILTIN_NONE)
    .value("mjMESH_BUILTIN_SPHERE", mjMESH_BUILTIN_SPHERE)
    .value("mjMESH_BUILTIN_HEMISPHERE", mjMESH_BUILTIN_HEMISPHERE)
    .value("mjMESH_BUILTIN_CONE", mjMESH_BUILTIN_CONE)
    .value("mjMESH_BUILTIN_SUPERSPHERE", mjMESH_BUILTIN_SUPERSPHERE)
    .value("mjMESH_BUILTIN_SUPERTORUS", mjMESH_BUILTIN_SUPERTORUS)
    .value("mjMESH_BUILTIN_WEDGE", mjMESH_BUILTIN_WEDGE)
    .value("mjMESH_BUILTIN_PLATE", mjMESH_BUILTIN_PLATE);
  enum_<mjtMeshInertia>("mjtMeshInertia")
    .value("mjMESH_INERTIA_CONVEX", mjMESH_INERTIA_CONVEX)
    .value("mjMESH_INERTIA_EXACT", mjMESH_INERTIA_EXACT)
    .value("mjMESH_INERTIA_LEGACY", mjMESH_INERTIA_LEGACY)
    .value("mjMESH_INERTIA_SHELL", mjMESH_INERTIA_SHELL);
  enum_<mjtMouse>("mjtMouse")
    .value("mjMOUSE_NONE", mjMOUSE_NONE)
    .value("mjMOUSE_ROTATE_V", mjMOUSE_ROTATE_V)
    .value("mjMOUSE_ROTATE_H", mjMOUSE_ROTATE_H)
    .value("mjMOUSE_MOVE_V", mjMOUSE_MOVE_V)
    .value("mjMOUSE_MOVE_H", mjMOUSE_MOVE_H)
    .value("mjMOUSE_ZOOM", mjMOUSE_ZOOM)
    .value("mjMOUSE_MOVE_V_REL", mjMOUSE_MOVE_V_REL)
    .value("mjMOUSE_MOVE_H_REL", mjMOUSE_MOVE_H_REL);
  enum_<mjtObj>("mjtObj")
    .value("mjOBJ_UNKNOWN", mjOBJ_UNKNOWN)
    .value("mjOBJ_BODY", mjOBJ_BODY)
    .value("mjOBJ_XBODY", mjOBJ_XBODY)
    .value("mjOBJ_JOINT", mjOBJ_JOINT)
    .value("mjOBJ_DOF", mjOBJ_DOF)
    .value("mjOBJ_GEOM", mjOBJ_GEOM)
    .value("mjOBJ_SITE", mjOBJ_SITE)
    .value("mjOBJ_CAMERA", mjOBJ_CAMERA)
    .value("mjOBJ_LIGHT", mjOBJ_LIGHT)
    .value("mjOBJ_FLEX", mjOBJ_FLEX)
    .value("mjOBJ_MESH", mjOBJ_MESH)
    .value("mjOBJ_SKIN", mjOBJ_SKIN)
    .value("mjOBJ_HFIELD", mjOBJ_HFIELD)
    .value("mjOBJ_TEXTURE", mjOBJ_TEXTURE)
    .value("mjOBJ_MATERIAL", mjOBJ_MATERIAL)
    .value("mjOBJ_PAIR", mjOBJ_PAIR)
    .value("mjOBJ_EXCLUDE", mjOBJ_EXCLUDE)
    .value("mjOBJ_EQUALITY", mjOBJ_EQUALITY)
    .value("mjOBJ_TENDON", mjOBJ_TENDON)
    .value("mjOBJ_ACTUATOR", mjOBJ_ACTUATOR)
    .value("mjOBJ_SENSOR", mjOBJ_SENSOR)
    .value("mjOBJ_NUMERIC", mjOBJ_NUMERIC)
    .value("mjOBJ_TEXT", mjOBJ_TEXT)
    .value("mjOBJ_TUPLE", mjOBJ_TUPLE)
    .value("mjOBJ_KEY", mjOBJ_KEY)
    .value("mjOBJ_PLUGIN", mjOBJ_PLUGIN)
    .value("mjNOBJECT", mjNOBJECT)
    .value("mjOBJ_FRAME", mjOBJ_FRAME)
    .value("mjOBJ_DEFAULT", mjOBJ_DEFAULT)
    .value("mjOBJ_MODEL", mjOBJ_MODEL);
  enum_<mjtOrientation>("mjtOrientation")
    .value("mjORIENTATION_QUAT", mjORIENTATION_QUAT)
    .value("mjORIENTATION_AXISANGLE", mjORIENTATION_AXISANGLE)
    .value("mjORIENTATION_XYAXES", mjORIENTATION_XYAXES)
    .value("mjORIENTATION_ZAXIS", mjORIENTATION_ZAXIS)
    .value("mjORIENTATION_EULER", mjORIENTATION_EULER);
  enum_<mjtPertBit>("mjtPertBit")
    .value("mjPERT_TRANSLATE", mjPERT_TRANSLATE)
    .value("mjPERT_ROTATE", mjPERT_ROTATE);
  enum_<mjtPluginCapabilityBit>("mjtPluginCapabilityBit")
    .value("mjPLUGIN_ACTUATOR", mjPLUGIN_ACTUATOR)
    .value("mjPLUGIN_SENSOR", mjPLUGIN_SENSOR)
    .value("mjPLUGIN_PASSIVE", mjPLUGIN_PASSIVE)
    .value("mjPLUGIN_SDF", mjPLUGIN_SDF);
  enum_<mjtRndFlag>("mjtRndFlag")
    .value("mjRND_SHADOW", mjRND_SHADOW)
    .value("mjRND_WIREFRAME", mjRND_WIREFRAME)
    .value("mjRND_REFLECTION", mjRND_REFLECTION)
    .value("mjRND_ADDITIVE", mjRND_ADDITIVE)
    .value("mjRND_SKYBOX", mjRND_SKYBOX)
    .value("mjRND_FOG", mjRND_FOG)
    .value("mjRND_HAZE", mjRND_HAZE)
    .value("mjRND_SEGMENT", mjRND_SEGMENT)
    .value("mjRND_IDCOLOR", mjRND_IDCOLOR)
    .value("mjRND_CULL_FACE", mjRND_CULL_FACE)
    .value("mjNRNDFLAG", mjNRNDFLAG);
  enum_<mjtSDFType>("mjtSDFType")
    .value("mjSDFTYPE_SINGLE", mjSDFTYPE_SINGLE)
    .value("mjSDFTYPE_INTERSECTION", mjSDFTYPE_INTERSECTION)
    .value("mjSDFTYPE_MIDSURFACE", mjSDFTYPE_MIDSURFACE)
    .value("mjSDFTYPE_COLLISION", mjSDFTYPE_COLLISION);
  enum_<mjtSameFrame>("mjtSameFrame")
    .value("mjSAMEFRAME_NONE", mjSAMEFRAME_NONE)
    .value("mjSAMEFRAME_BODY", mjSAMEFRAME_BODY)
    .value("mjSAMEFRAME_INERTIA", mjSAMEFRAME_INERTIA)
    .value("mjSAMEFRAME_BODYROT", mjSAMEFRAME_BODYROT)
    .value("mjSAMEFRAME_INERTIAROT", mjSAMEFRAME_INERTIAROT);
  enum_<mjtSection>("mjtSection")
    .value("mjSECT_CLOSED", mjSECT_CLOSED)
    .value("mjSECT_OPEN", mjSECT_OPEN)
    .value("mjSECT_FIXED", mjSECT_FIXED);
  enum_<mjtSensor>("mjtSensor")
    .value("mjSENS_TOUCH", mjSENS_TOUCH)
    .value("mjSENS_ACCELEROMETER", mjSENS_ACCELEROMETER)
    .value("mjSENS_VELOCIMETER", mjSENS_VELOCIMETER)
    .value("mjSENS_GYRO", mjSENS_GYRO)
    .value("mjSENS_FORCE", mjSENS_FORCE)
    .value("mjSENS_TORQUE", mjSENS_TORQUE)
    .value("mjSENS_MAGNETOMETER", mjSENS_MAGNETOMETER)
    .value("mjSENS_RANGEFINDER", mjSENS_RANGEFINDER)
    .value("mjSENS_CAMPROJECTION", mjSENS_CAMPROJECTION)
    .value("mjSENS_JOINTPOS", mjSENS_JOINTPOS)
    .value("mjSENS_JOINTVEL", mjSENS_JOINTVEL)
    .value("mjSENS_TENDONPOS", mjSENS_TENDONPOS)
    .value("mjSENS_TENDONVEL", mjSENS_TENDONVEL)
    .value("mjSENS_ACTUATORPOS", mjSENS_ACTUATORPOS)
    .value("mjSENS_ACTUATORVEL", mjSENS_ACTUATORVEL)
    .value("mjSENS_ACTUATORFRC", mjSENS_ACTUATORFRC)
    .value("mjSENS_JOINTACTFRC", mjSENS_JOINTACTFRC)
    .value("mjSENS_TENDONACTFRC", mjSENS_TENDONACTFRC)
    .value("mjSENS_BALLQUAT", mjSENS_BALLQUAT)
    .value("mjSENS_BALLANGVEL", mjSENS_BALLANGVEL)
    .value("mjSENS_JOINTLIMITPOS", mjSENS_JOINTLIMITPOS)
    .value("mjSENS_JOINTLIMITVEL", mjSENS_JOINTLIMITVEL)
    .value("mjSENS_JOINTLIMITFRC", mjSENS_JOINTLIMITFRC)
    .value("mjSENS_TENDONLIMITPOS", mjSENS_TENDONLIMITPOS)
    .value("mjSENS_TENDONLIMITVEL", mjSENS_TENDONLIMITVEL)
    .value("mjSENS_TENDONLIMITFRC", mjSENS_TENDONLIMITFRC)
    .value("mjSENS_FRAMEPOS", mjSENS_FRAMEPOS)
    .value("mjSENS_FRAMEQUAT", mjSENS_FRAMEQUAT)
    .value("mjSENS_FRAMEXAXIS", mjSENS_FRAMEXAXIS)
    .value("mjSENS_FRAMEYAXIS", mjSENS_FRAMEYAXIS)
    .value("mjSENS_FRAMEZAXIS", mjSENS_FRAMEZAXIS)
    .value("mjSENS_FRAMELINVEL", mjSENS_FRAMELINVEL)
    .value("mjSENS_FRAMEANGVEL", mjSENS_FRAMEANGVEL)
    .value("mjSENS_FRAMELINACC", mjSENS_FRAMELINACC)
    .value("mjSENS_FRAMEANGACC", mjSENS_FRAMEANGACC)
    .value("mjSENS_SUBTREECOM", mjSENS_SUBTREECOM)
    .value("mjSENS_SUBTREELINVEL", mjSENS_SUBTREELINVEL)
    .value("mjSENS_SUBTREEANGMOM", mjSENS_SUBTREEANGMOM)
    .value("mjSENS_INSIDESITE", mjSENS_INSIDESITE)
    .value("mjSENS_GEOMDIST", mjSENS_GEOMDIST)
    .value("mjSENS_GEOMNORMAL", mjSENS_GEOMNORMAL)
    .value("mjSENS_GEOMFROMTO", mjSENS_GEOMFROMTO)
    .value("mjSENS_CONTACT", mjSENS_CONTACT)
    .value("mjSENS_E_POTENTIAL", mjSENS_E_POTENTIAL)
    .value("mjSENS_E_KINETIC", mjSENS_E_KINETIC)
    .value("mjSENS_CLOCK", mjSENS_CLOCK)
    .value("mjSENS_TACTILE", mjSENS_TACTILE)
    .value("mjSENS_PLUGIN", mjSENS_PLUGIN)
    .value("mjSENS_USER", mjSENS_USER);
  enum_<mjtSleepPolicy>("mjtSleepPolicy")
    .value("mjSLEEP_AUTO", mjSLEEP_AUTO)
    .value("mjSLEEP_AUTO_NEVER", mjSLEEP_AUTO_NEVER)
    .value("mjSLEEP_AUTO_ALLOWED", mjSLEEP_AUTO_ALLOWED)
    .value("mjSLEEP_NEVER", mjSLEEP_NEVER)
    .value("mjSLEEP_ALLOWED", mjSLEEP_ALLOWED)
    .value("mjSLEEP_INIT", mjSLEEP_INIT);
  enum_<mjtSleepState>("mjtSleepState")
    .value("mjS_STATIC", mjS_STATIC)
    .value("mjS_ASLEEP", mjS_ASLEEP)
    .value("mjS_AWAKE", mjS_AWAKE);
  enum_<mjtSolver>("mjtSolver")
    .value("mjSOL_PGS", mjSOL_PGS)
    .value("mjSOL_CG", mjSOL_CG)
    .value("mjSOL_NEWTON", mjSOL_NEWTON);
  enum_<mjtStage>("mjtStage")
    .value("mjSTAGE_NONE", mjSTAGE_NONE)
    .value("mjSTAGE_POS", mjSTAGE_POS)
    .value("mjSTAGE_VEL", mjSTAGE_VEL)
    .value("mjSTAGE_ACC", mjSTAGE_ACC);
  enum_<mjtState>("mjtState")
    .value("mjSTATE_TIME", mjSTATE_TIME)
    .value("mjSTATE_QPOS", mjSTATE_QPOS)
    .value("mjSTATE_QVEL", mjSTATE_QVEL)
    .value("mjSTATE_ACT", mjSTATE_ACT)
    .value("mjSTATE_WARMSTART", mjSTATE_WARMSTART)
    .value("mjSTATE_CTRL", mjSTATE_CTRL)
    .value("mjSTATE_QFRC_APPLIED", mjSTATE_QFRC_APPLIED)
    .value("mjSTATE_XFRC_APPLIED", mjSTATE_XFRC_APPLIED)
    .value("mjSTATE_EQ_ACTIVE", mjSTATE_EQ_ACTIVE)
    .value("mjSTATE_MOCAP_POS", mjSTATE_MOCAP_POS)
    .value("mjSTATE_MOCAP_QUAT", mjSTATE_MOCAP_QUAT)
    .value("mjSTATE_USERDATA", mjSTATE_USERDATA)
    .value("mjSTATE_PLUGIN", mjSTATE_PLUGIN)
    .value("mjNSTATE", mjNSTATE)
    .value("mjSTATE_PHYSICS", mjSTATE_PHYSICS)
    .value("mjSTATE_FULLPHYSICS", mjSTATE_FULLPHYSICS)
    .value("mjSTATE_USER", mjSTATE_USER)
    .value("mjSTATE_INTEGRATION", mjSTATE_INTEGRATION);
  enum_<mjtStereo>("mjtStereo")
    .value("mjSTEREO_NONE", mjSTEREO_NONE)
    .value("mjSTEREO_QUADBUFFERED", mjSTEREO_QUADBUFFERED)
    .value("mjSTEREO_SIDEBYSIDE", mjSTEREO_SIDEBYSIDE);
  enum_<mjtTaskStatus>("mjtTaskStatus")
    .value("mjTASK_NEW", mjTASK_NEW)
    .value("mjTASK_QUEUED", mjTASK_QUEUED)
    .value("mjTASK_COMPLETED", mjTASK_COMPLETED);
  enum_<mjtTexture>("mjtTexture")
    .value("mjTEXTURE_2D", mjTEXTURE_2D)
    .value("mjTEXTURE_CUBE", mjTEXTURE_CUBE)
    .value("mjTEXTURE_SKYBOX", mjTEXTURE_SKYBOX);
  enum_<mjtTextureRole>("mjtTextureRole")
    .value("mjTEXROLE_USER", mjTEXROLE_USER)
    .value("mjTEXROLE_RGB", mjTEXROLE_RGB)
    .value("mjTEXROLE_OCCLUSION", mjTEXROLE_OCCLUSION)
    .value("mjTEXROLE_ROUGHNESS", mjTEXROLE_ROUGHNESS)
    .value("mjTEXROLE_METALLIC", mjTEXROLE_METALLIC)
    .value("mjTEXROLE_NORMAL", mjTEXROLE_NORMAL)
    .value("mjTEXROLE_OPACITY", mjTEXROLE_OPACITY)
    .value("mjTEXROLE_EMISSIVE", mjTEXROLE_EMISSIVE)
    .value("mjTEXROLE_RGBA", mjTEXROLE_RGBA)
    .value("mjTEXROLE_ORM", mjTEXROLE_ORM)
    .value("mjNTEXROLE", mjNTEXROLE);
  enum_<mjtTimer>("mjtTimer")
    .value("mjTIMER_STEP", mjTIMER_STEP)
    .value("mjTIMER_FORWARD", mjTIMER_FORWARD)
    .value("mjTIMER_INVERSE", mjTIMER_INVERSE)
    .value("mjTIMER_POSITION", mjTIMER_POSITION)
    .value("mjTIMER_VELOCITY", mjTIMER_VELOCITY)
    .value("mjTIMER_ACTUATION", mjTIMER_ACTUATION)
    .value("mjTIMER_CONSTRAINT", mjTIMER_CONSTRAINT)
    .value("mjTIMER_ADVANCE", mjTIMER_ADVANCE)
    .value("mjTIMER_POS_KINEMATICS", mjTIMER_POS_KINEMATICS)
    .value("mjTIMER_POS_INERTIA", mjTIMER_POS_INERTIA)
    .value("mjTIMER_POS_COLLISION", mjTIMER_POS_COLLISION)
    .value("mjTIMER_POS_MAKE", mjTIMER_POS_MAKE)
    .value("mjTIMER_POS_PROJECT", mjTIMER_POS_PROJECT)
    .value("mjTIMER_COL_BROAD", mjTIMER_COL_BROAD)
    .value("mjTIMER_COL_NARROW", mjTIMER_COL_NARROW)
    .value("mjNTIMER", mjNTIMER);
  enum_<mjtTrn>("mjtTrn")
    .value("mjTRN_JOINT", mjTRN_JOINT)
    .value("mjTRN_JOINTINPARENT", mjTRN_JOINTINPARENT)
    .value("mjTRN_SLIDERCRANK", mjTRN_SLIDERCRANK)
    .value("mjTRN_TENDON", mjTRN_TENDON)
    .value("mjTRN_SITE", mjTRN_SITE)
    .value("mjTRN_BODY", mjTRN_BODY)
    .value("mjTRN_UNDEFINED", mjTRN_UNDEFINED);
  enum_<mjtVisFlag>("mjtVisFlag")
    .value("mjVIS_CONVEXHULL", mjVIS_CONVEXHULL)
    .value("mjVIS_TEXTURE", mjVIS_TEXTURE)
    .value("mjVIS_JOINT", mjVIS_JOINT)
    .value("mjVIS_CAMERA", mjVIS_CAMERA)
    .value("mjVIS_ACTUATOR", mjVIS_ACTUATOR)
    .value("mjVIS_ACTIVATION", mjVIS_ACTIVATION)
    .value("mjVIS_LIGHT", mjVIS_LIGHT)
    .value("mjVIS_TENDON", mjVIS_TENDON)
    .value("mjVIS_RANGEFINDER", mjVIS_RANGEFINDER)
    .value("mjVIS_CONSTRAINT", mjVIS_CONSTRAINT)
    .value("mjVIS_INERTIA", mjVIS_INERTIA)
    .value("mjVIS_SCLINERTIA", mjVIS_SCLINERTIA)
    .value("mjVIS_PERTFORCE", mjVIS_PERTFORCE)
    .value("mjVIS_PERTOBJ", mjVIS_PERTOBJ)
    .value("mjVIS_CONTACTPOINT", mjVIS_CONTACTPOINT)
    .value("mjVIS_ISLAND", mjVIS_ISLAND)
    .value("mjVIS_CONTACTFORCE", mjVIS_CONTACTFORCE)
    .value("mjVIS_CONTACTSPLIT", mjVIS_CONTACTSPLIT)
    .value("mjVIS_TRANSPARENT", mjVIS_TRANSPARENT)
    .value("mjVIS_AUTOCONNECT", mjVIS_AUTOCONNECT)
    .value("mjVIS_COM", mjVIS_COM)
    .value("mjVIS_SELECT", mjVIS_SELECT)
    .value("mjVIS_STATIC", mjVIS_STATIC)
    .value("mjVIS_SKIN", mjVIS_SKIN)
    .value("mjVIS_FLEXVERT", mjVIS_FLEXVERT)
    .value("mjVIS_FLEXEDGE", mjVIS_FLEXEDGE)
    .value("mjVIS_FLEXFACE", mjVIS_FLEXFACE)
    .value("mjVIS_FLEXSKIN", mjVIS_FLEXSKIN)
    .value("mjVIS_BODYBVH", mjVIS_BODYBVH)
    .value("mjVIS_MESHBVH", mjVIS_MESHBVH)
    .value("mjVIS_SDFITER", mjVIS_SDFITER)
    .value("mjNVISFLAG", mjNVISFLAG);
  enum_<mjtWarning>("mjtWarning")
    .value("mjWARN_INERTIA", mjWARN_INERTIA)
    .value("mjWARN_CONTACTFULL", mjWARN_CONTACTFULL)
    .value("mjWARN_CNSTRFULL", mjWARN_CNSTRFULL)
    .value("mjWARN_VGEOMFULL", mjWARN_VGEOMFULL)
    .value("mjWARN_BADQPOS", mjWARN_BADQPOS)
    .value("mjWARN_BADQVEL", mjWARN_BADQVEL)
    .value("mjWARN_BADQACC", mjWARN_BADQACC)
    .value("mjWARN_BADCTRL", mjWARN_BADCTRL)
    .value("mjNWARNING", mjNWARNING);
  enum_<mjtWrap>("mjtWrap")
    .value("mjWRAP_NONE", mjWRAP_NONE)
    .value("mjWRAP_JOINT", mjWRAP_JOINT)
    .value("mjWRAP_PULLEY", mjWRAP_PULLEY)
    .value("mjWRAP_SITE", mjWRAP_SITE)
    .value("mjWRAP_SPHERE", mjWRAP_SPHERE)
    .value("mjWRAP_CYLINDER", mjWRAP_CYLINDER);

  emscripten::class_<MjContact>("MjContact")
    .constructor<>()
    .function("copy", &MjContact::copy, take_ownership())
    .property("H", &MjContact::H)
    .property("dim", &MjContact::dim, &MjContact::set_dim, reference())
    .property("dist", &MjContact::dist, &MjContact::set_dist, reference())
    .property("efc_address", &MjContact::efc_address, &MjContact::set_efc_address, reference())
    .property("elem", &MjContact::elem)
    .property("exclude", &MjContact::exclude, &MjContact::set_exclude, reference())
    .property("flex", &MjContact::flex)
    .property("frame", &MjContact::frame)
    .property("friction", &MjContact::friction)
    .property("geom", &MjContact::geom)
    .property("geom1", &MjContact::geom1, &MjContact::set_geom1, reference())
    .property("geom2", &MjContact::geom2, &MjContact::set_geom2, reference())
    .property("includemargin", &MjContact::includemargin, &MjContact::set_includemargin, reference())
    .property("mu", &MjContact::mu, &MjContact::set_mu, reference())
    .property("pos", &MjContact::pos)
    .property("solimp", &MjContact::solimp)
    .property("solref", &MjContact::solref)
    .property("solreffriction", &MjContact::solreffriction)
    .property("vert", &MjContact::vert);
  emscripten::class_<MjData>("MjData")
    .constructor<MjModel *>()
    .constructor<const MjModel &, const MjData &>()
    .property("M", &MjData::M)
    .property("act", &MjData::act)
    .property("act_dot", &MjData::act_dot)
    .property("actuator_force", &MjData::actuator_force)
    .property("actuator_length", &MjData::actuator_length)
    .property("actuator_moment", &MjData::actuator_moment)
    .property("actuator_velocity", &MjData::actuator_velocity)
    .property("arena", &MjData::arena)
    .property("body_awake", &MjData::body_awake)
    .property("body_awake_ind", &MjData::body_awake_ind)
    .property("buffer", &MjData::buffer)
    .property("bvh_aabb_dyn", &MjData::bvh_aabb_dyn)
    .property("bvh_active", &MjData::bvh_active)
    .property("cacc", &MjData::cacc)
    .property("cam_xmat", &MjData::cam_xmat)
    .property("cam_xpos", &MjData::cam_xpos)
    .property("cdof", &MjData::cdof)
    .property("cdof_dot", &MjData::cdof_dot)
    .property("cfrc_ext", &MjData::cfrc_ext)
    .property("cfrc_int", &MjData::cfrc_int)
    .property("cinert", &MjData::cinert)
    .property("contact", &MjData::contact)
    .property("crb", &MjData::crb)
    .property("ctrl", &MjData::ctrl)
    .property("cvel", &MjData::cvel)
    .property("dof_awake_ind", &MjData::dof_awake_ind)
    .property("dof_island", &MjData::dof_island)
    .property("efc_AR", &MjData::efc_AR)
    .property("efc_AR_colind", &MjData::efc_AR_colind)
    .property("efc_AR_rowadr", &MjData::efc_AR_rowadr)
    .property("efc_AR_rownnz", &MjData::efc_AR_rownnz)
    .property("efc_D", &MjData::efc_D)
    .property("efc_J", &MjData::efc_J)
    .property("efc_J_colind", &MjData::efc_J_colind)
    .property("efc_J_rowadr", &MjData::efc_J_rowadr)
    .property("efc_J_rownnz", &MjData::efc_J_rownnz)
    .property("efc_J_rowsuper", &MjData::efc_J_rowsuper)
    .property("efc_KBIP", &MjData::efc_KBIP)
    .property("efc_R", &MjData::efc_R)
    .property("efc_aref", &MjData::efc_aref)
    .property("efc_b", &MjData::efc_b)
    .property("efc_diagApprox", &MjData::efc_diagApprox)
    .property("efc_force", &MjData::efc_force)
    .property("efc_frictionloss", &MjData::efc_frictionloss)
    .property("efc_id", &MjData::efc_id)
    .property("efc_island", &MjData::efc_island)
    .property("efc_margin", &MjData::efc_margin)
    .property("efc_pos", &MjData::efc_pos)
    .property("efc_state", &MjData::efc_state)
    .property("efc_type", &MjData::efc_type)
    .property("efc_vel", &MjData::efc_vel)
    .property("energy", &MjData::energy)
    .property("eq_active", &MjData::eq_active)
    .property("flexedge_J", &MjData::flexedge_J)
    .property("flexedge_J_colind", &MjData::flexedge_J_colind)
    .property("flexedge_J_rowadr", &MjData::flexedge_J_rowadr)
    .property("flexedge_J_rownnz", &MjData::flexedge_J_rownnz)
    .property("flexedge_length", &MjData::flexedge_length)
    .property("flexedge_velocity", &MjData::flexedge_velocity)
    .property("flexelem_aabb", &MjData::flexelem_aabb)
    .property("flexvert_xpos", &MjData::flexvert_xpos)
    .property("geom_xmat", &MjData::geom_xmat)
    .property("geom_xpos", &MjData::geom_xpos)
    .property("iLD", &MjData::iLD)
    .property("iLDiagInv", &MjData::iLDiagInv)
    .property("iM", &MjData::iM)
    .property("iM_colind", &MjData::iM_colind)
    .property("iM_rowadr", &MjData::iM_rowadr)
    .property("iM_rownnz", &MjData::iM_rownnz)
    .property("iacc", &MjData::iacc)
    .property("iacc_smooth", &MjData::iacc_smooth)
    .property("iefc_D", &MjData::iefc_D)
    .property("iefc_J", &MjData::iefc_J)
    .property("iefc_J_colind", &MjData::iefc_J_colind)
    .property("iefc_J_rowadr", &MjData::iefc_J_rowadr)
    .property("iefc_J_rownnz", &MjData::iefc_J_rownnz)
    .property("iefc_J_rowsuper", &MjData::iefc_J_rowsuper)
    .property("iefc_R", &MjData::iefc_R)
    .property("iefc_aref", &MjData::iefc_aref)
    .property("iefc_force", &MjData::iefc_force)
    .property("iefc_frictionloss", &MjData::iefc_frictionloss)
    .property("iefc_id", &MjData::iefc_id)
    .property("iefc_state", &MjData::iefc_state)
    .property("iefc_type", &MjData::iefc_type)
    .property("ifrc_constraint", &MjData::ifrc_constraint)
    .property("ifrc_smooth", &MjData::ifrc_smooth)
    .property("island_dofadr", &MjData::island_dofadr)
    .property("island_idofadr", &MjData::island_idofadr)
    .property("island_iefcadr", &MjData::island_iefcadr)
    .property("island_itreeadr", &MjData::island_itreeadr)
    .property("island_ne", &MjData::island_ne)
    .property("island_nefc", &MjData::island_nefc)
    .property("island_nf", &MjData::island_nf)
    .property("island_ntree", &MjData::island_ntree)
    .property("island_nv", &MjData::island_nv)
    .property("light_xdir", &MjData::light_xdir)
    .property("light_xpos", &MjData::light_xpos)
    .property("map_dof2idof", &MjData::map_dof2idof)
    .property("map_efc2iefc", &MjData::map_efc2iefc)
    .property("map_idof2dof", &MjData::map_idof2dof)
    .property("map_iefc2efc", &MjData::map_iefc2efc)
    .property("map_itree2tree", &MjData::map_itree2tree)
    .property("maxuse_arena", &MjData::maxuse_arena, &MjData::set_maxuse_arena, reference())
    .property("maxuse_con", &MjData::maxuse_con, &MjData::set_maxuse_con, reference())
    .property("maxuse_efc", &MjData::maxuse_efc, &MjData::set_maxuse_efc, reference())
    .property("maxuse_stack", &MjData::maxuse_stack, &MjData::set_maxuse_stack, reference())
    .property("maxuse_threadstack", &MjData::maxuse_threadstack)
    .property("mocap_pos", &MjData::mocap_pos)
    .property("mocap_quat", &MjData::mocap_quat)
    .property("moment_colind", &MjData::moment_colind)
    .property("moment_rowadr", &MjData::moment_rowadr)
    .property("moment_rownnz", &MjData::moment_rownnz)
    .property("nA", &MjData::nA, &MjData::set_nA, reference())
    .property("nJ", &MjData::nJ, &MjData::set_nJ, reference())
    .property("narena", &MjData::narena, &MjData::set_narena, reference())
    .property("nbody_awake", &MjData::nbody_awake, &MjData::set_nbody_awake, reference())
    .property("nbuffer", &MjData::nbuffer, &MjData::set_nbuffer, reference())
    .property("ncon", &MjData::ncon, &MjData::set_ncon, reference())
    .property("ne", &MjData::ne, &MjData::set_ne, reference())
    .property("nefc", &MjData::nefc, &MjData::set_nefc, reference())
    .property("nf", &MjData::nf, &MjData::set_nf, reference())
    .property("nidof", &MjData::nidof, &MjData::set_nidof, reference())
    .property("nisland", &MjData::nisland, &MjData::set_nisland, reference())
    .property("nl", &MjData::nl, &MjData::set_nl, reference())
    .property("nparent_awake", &MjData::nparent_awake, &MjData::set_nparent_awake, reference())
    .property("nplugin", &MjData::nplugin, &MjData::set_nplugin, reference())
    .property("ntree_awake", &MjData::ntree_awake, &MjData::set_ntree_awake, reference())
    .property("nv_awake", &MjData::nv_awake, &MjData::set_nv_awake, reference())
    .property("parena", &MjData::parena, &MjData::set_parena, reference())
    .property("parent_awake_ind", &MjData::parent_awake_ind)
    .property("pbase", &MjData::pbase, &MjData::set_pbase, reference())
    .property("plugin", &MjData::plugin)
    .property("plugin_data", &MjData::plugin_data)
    .property("plugin_state", &MjData::plugin_state)
    .property("pstack", &MjData::pstack, &MjData::set_pstack, reference())
    .property("qDeriv", &MjData::qDeriv)
    .property("qH", &MjData::qH)
    .property("qHDiagInv", &MjData::qHDiagInv)
    .property("qLD", &MjData::qLD)
    .property("qLDiagInv", &MjData::qLDiagInv)
    .property("qLU", &MjData::qLU)
    .property("qM", &MjData::qM)
    .property("qacc", &MjData::qacc)
    .property("qacc_smooth", &MjData::qacc_smooth)
    .property("qacc_warmstart", &MjData::qacc_warmstart)
    .property("qfrc_actuator", &MjData::qfrc_actuator)
    .property("qfrc_applied", &MjData::qfrc_applied)
    .property("qfrc_bias", &MjData::qfrc_bias)
    .property("qfrc_constraint", &MjData::qfrc_constraint)
    .property("qfrc_damper", &MjData::qfrc_damper)
    .property("qfrc_fluid", &MjData::qfrc_fluid)
    .property("qfrc_gravcomp", &MjData::qfrc_gravcomp)
    .property("qfrc_inverse", &MjData::qfrc_inverse)
    .property("qfrc_passive", &MjData::qfrc_passive)
    .property("qfrc_smooth", &MjData::qfrc_smooth)
    .property("qfrc_spring", &MjData::qfrc_spring)
    .property("qpos", &MjData::qpos)
    .property("qvel", &MjData::qvel)
    .property("sensordata", &MjData::sensordata)
    .property("signature", &MjData::signature, &MjData::set_signature, reference())
    .property("site_xmat", &MjData::site_xmat)
    .property("site_xpos", &MjData::site_xpos)
    .property("solver", &MjData::solver, reference())
    .property("solver_fwdinv", &MjData::solver_fwdinv)
    .property("solver_niter", &MjData::solver_niter)
    .property("solver_nnz", &MjData::solver_nnz)
    .property("subtree_angmom", &MjData::subtree_angmom)
    .property("subtree_com", &MjData::subtree_com)
    .property("subtree_linvel", &MjData::subtree_linvel)
    .property("ten_J", &MjData::ten_J)
    .property("ten_J_colind", &MjData::ten_J_colind)
    .property("ten_J_rowadr", &MjData::ten_J_rowadr)
    .property("ten_J_rownnz", &MjData::ten_J_rownnz)
    .property("ten_length", &MjData::ten_length)
    .property("ten_velocity", &MjData::ten_velocity)
    .property("ten_wrapadr", &MjData::ten_wrapadr)
    .property("ten_wrapnum", &MjData::ten_wrapnum)
    .property("tendon_efcadr", &MjData::tendon_efcadr)
    .property("threadpool", &MjData::threadpool, &MjData::set_threadpool, reference())
    .property("time", &MjData::time, &MjData::set_time, reference())
    .property("timer", &MjData::timer, reference())
    .property("tree_asleep", &MjData::tree_asleep)
    .property("tree_awake", &MjData::tree_awake)
    .property("tree_island", &MjData::tree_island)
    .property("userdata", &MjData::userdata)
    .property("warning", &MjData::warning, reference())
    .property("wrap_obj", &MjData::wrap_obj)
    .property("wrap_xpos", &MjData::wrap_xpos)
    .property("xanchor", &MjData::xanchor)
    .property("xaxis", &MjData::xaxis)
    .property("xfrc_applied", &MjData::xfrc_applied)
    .property("ximat", &MjData::ximat)
    .property("xipos", &MjData::xipos)
    .property("xmat", &MjData::xmat)
    .property("xpos", &MjData::xpos)
    .property("xquat", &MjData::xquat);
  emscripten::class_<MjLROpt>("MjLROpt")
    .constructor<>()
    .function("copy", &MjLROpt::copy, take_ownership())
    .property("accel", &MjLROpt::accel, &MjLROpt::set_accel, reference())
    .property("interval", &MjLROpt::interval, &MjLROpt::set_interval, reference())
    .property("inttotal", &MjLROpt::inttotal, &MjLROpt::set_inttotal, reference())
    .property("maxforce", &MjLROpt::maxforce, &MjLROpt::set_maxforce, reference())
    .property("mode", &MjLROpt::mode, &MjLROpt::set_mode, reference())
    .property("timeconst", &MjLROpt::timeconst, &MjLROpt::set_timeconst, reference())
    .property("timestep", &MjLROpt::timestep, &MjLROpt::set_timestep, reference())
    .property("tolrange", &MjLROpt::tolrange, &MjLROpt::set_tolrange, reference())
    .property("useexisting", &MjLROpt::useexisting, &MjLROpt::set_useexisting, reference())
    .property("uselimit", &MjLROpt::uselimit, &MjLROpt::set_uselimit, reference());
  emscripten::class_<MjModel>("MjModel")
    .class_function("mj_loadXML", &mj_loadXML_wrapper, take_ownership())
    .constructor<const MjModel &>()
    .property("B_colind", &MjModel::B_colind)
    .property("B_rowadr", &MjModel::B_rowadr)
    .property("B_rownnz", &MjModel::B_rownnz)
    .property("D_colind", &MjModel::D_colind)
    .property("D_diag", &MjModel::D_diag)
    .property("D_rowadr", &MjModel::D_rowadr)
    .property("D_rownnz", &MjModel::D_rownnz)
    .property("M_colind", &MjModel::M_colind)
    .property("M_rowadr", &MjModel::M_rowadr)
    .property("M_rownnz", &MjModel::M_rownnz)
    .property("actuator_acc0", &MjModel::actuator_acc0)
    .property("actuator_actadr", &MjModel::actuator_actadr)
    .property("actuator_actearly", &MjModel::actuator_actearly)
    .property("actuator_actlimited", &MjModel::actuator_actlimited)
    .property("actuator_actnum", &MjModel::actuator_actnum)
    .property("actuator_actrange", &MjModel::actuator_actrange)
    .property("actuator_biasprm", &MjModel::actuator_biasprm)
    .property("actuator_biastype", &MjModel::actuator_biastype)
    .property("actuator_cranklength", &MjModel::actuator_cranklength)
    .property("actuator_ctrllimited", &MjModel::actuator_ctrllimited)
    .property("actuator_ctrlrange", &MjModel::actuator_ctrlrange)
    .property("actuator_dynprm", &MjModel::actuator_dynprm)
    .property("actuator_dyntype", &MjModel::actuator_dyntype)
    .property("actuator_forcelimited", &MjModel::actuator_forcelimited)
    .property("actuator_forcerange", &MjModel::actuator_forcerange)
    .property("actuator_gainprm", &MjModel::actuator_gainprm)
    .property("actuator_gaintype", &MjModel::actuator_gaintype)
    .property("actuator_gear", &MjModel::actuator_gear)
    .property("actuator_group", &MjModel::actuator_group)
    .property("actuator_length0", &MjModel::actuator_length0)
    .property("actuator_lengthrange", &MjModel::actuator_lengthrange)
    .property("actuator_plugin", &MjModel::actuator_plugin)
    .property("actuator_trnid", &MjModel::actuator_trnid)
    .property("actuator_trntype", &MjModel::actuator_trntype)
    .property("actuator_user", &MjModel::actuator_user)
    .property("body_bvhadr", &MjModel::body_bvhadr)
    .property("body_bvhnum", &MjModel::body_bvhnum)
    .property("body_conaffinity", &MjModel::body_conaffinity)
    .property("body_contype", &MjModel::body_contype)
    .property("body_dofadr", &MjModel::body_dofadr)
    .property("body_dofnum", &MjModel::body_dofnum)
    .property("body_geomadr", &MjModel::body_geomadr)
    .property("body_geomnum", &MjModel::body_geomnum)
    .property("body_gravcomp", &MjModel::body_gravcomp)
    .property("body_inertia", &MjModel::body_inertia)
    .property("body_invweight0", &MjModel::body_invweight0)
    .property("body_ipos", &MjModel::body_ipos)
    .property("body_iquat", &MjModel::body_iquat)
    .property("body_jntadr", &MjModel::body_jntadr)
    .property("body_jntnum", &MjModel::body_jntnum)
    .property("body_margin", &MjModel::body_margin)
    .property("body_mass", &MjModel::body_mass)
    .property("body_mocapid", &MjModel::body_mocapid)
    .property("body_parentid", &MjModel::body_parentid)
    .property("body_plugin", &MjModel::body_plugin)
    .property("body_pos", &MjModel::body_pos)
    .property("body_quat", &MjModel::body_quat)
    .property("body_rootid", &MjModel::body_rootid)
    .property("body_sameframe", &MjModel::body_sameframe)
    .property("body_simple", &MjModel::body_simple)
    .property("body_subtreemass", &MjModel::body_subtreemass)
    .property("body_treeid", &MjModel::body_treeid)
    .property("body_user", &MjModel::body_user)
    .property("body_weldid", &MjModel::body_weldid)
    .property("buffer", &MjModel::buffer)
    .property("bvh_aabb", &MjModel::bvh_aabb)
    .property("bvh_child", &MjModel::bvh_child)
    .property("bvh_depth", &MjModel::bvh_depth)
    .property("bvh_nodeid", &MjModel::bvh_nodeid)
    .property("cam_bodyid", &MjModel::cam_bodyid)
    .property("cam_fovy", &MjModel::cam_fovy)
    .property("cam_intrinsic", &MjModel::cam_intrinsic)
    .property("cam_ipd", &MjModel::cam_ipd)
    .property("cam_mat0", &MjModel::cam_mat0)
    .property("cam_mode", &MjModel::cam_mode)
    .property("cam_orthographic", &MjModel::cam_orthographic)
    .property("cam_pos", &MjModel::cam_pos)
    .property("cam_pos0", &MjModel::cam_pos0)
    .property("cam_poscom0", &MjModel::cam_poscom0)
    .property("cam_quat", &MjModel::cam_quat)
    .property("cam_resolution", &MjModel::cam_resolution)
    .property("cam_sensorsize", &MjModel::cam_sensorsize)
    .property("cam_targetbodyid", &MjModel::cam_targetbodyid)
    .property("cam_user", &MjModel::cam_user)
    .property("dof_M0", &MjModel::dof_M0)
    .property("dof_Madr", &MjModel::dof_Madr)
    .property("dof_armature", &MjModel::dof_armature)
    .property("dof_bodyid", &MjModel::dof_bodyid)
    .property("dof_damping", &MjModel::dof_damping)
    .property("dof_frictionloss", &MjModel::dof_frictionloss)
    .property("dof_invweight0", &MjModel::dof_invweight0)
    .property("dof_jntid", &MjModel::dof_jntid)
    .property("dof_length", &MjModel::dof_length)
    .property("dof_parentid", &MjModel::dof_parentid)
    .property("dof_simplenum", &MjModel::dof_simplenum)
    .property("dof_solimp", &MjModel::dof_solimp)
    .property("dof_solref", &MjModel::dof_solref)
    .property("dof_treeid", &MjModel::dof_treeid)
    .property("eq_active0", &MjModel::eq_active0)
    .property("eq_data", &MjModel::eq_data)
    .property("eq_obj1id", &MjModel::eq_obj1id)
    .property("eq_obj2id", &MjModel::eq_obj2id)
    .property("eq_objtype", &MjModel::eq_objtype)
    .property("eq_solimp", &MjModel::eq_solimp)
    .property("eq_solref", &MjModel::eq_solref)
    .property("eq_type", &MjModel::eq_type)
    .property("exclude_signature", &MjModel::exclude_signature)
    .property("flex_activelayers", &MjModel::flex_activelayers)
    .property("flex_bending", &MjModel::flex_bending)
    .property("flex_bvhadr", &MjModel::flex_bvhadr)
    .property("flex_bvhnum", &MjModel::flex_bvhnum)
    .property("flex_centered", &MjModel::flex_centered)
    .property("flex_conaffinity", &MjModel::flex_conaffinity)
    .property("flex_condim", &MjModel::flex_condim)
    .property("flex_contype", &MjModel::flex_contype)
    .property("flex_damping", &MjModel::flex_damping)
    .property("flex_dim", &MjModel::flex_dim)
    .property("flex_edge", &MjModel::flex_edge)
    .property("flex_edgeadr", &MjModel::flex_edgeadr)
    .property("flex_edgedamping", &MjModel::flex_edgedamping)
    .property("flex_edgeequality", &MjModel::flex_edgeequality)
    .property("flex_edgeflap", &MjModel::flex_edgeflap)
    .property("flex_edgenum", &MjModel::flex_edgenum)
    .property("flex_edgestiffness", &MjModel::flex_edgestiffness)
    .property("flex_elem", &MjModel::flex_elem)
    .property("flex_elemadr", &MjModel::flex_elemadr)
    .property("flex_elemdataadr", &MjModel::flex_elemdataadr)
    .property("flex_elemedge", &MjModel::flex_elemedge)
    .property("flex_elemedgeadr", &MjModel::flex_elemedgeadr)
    .property("flex_elemlayer", &MjModel::flex_elemlayer)
    .property("flex_elemnum", &MjModel::flex_elemnum)
    .property("flex_elemtexcoord", &MjModel::flex_elemtexcoord)
    .property("flex_evpair", &MjModel::flex_evpair)
    .property("flex_evpairadr", &MjModel::flex_evpairadr)
    .property("flex_evpairnum", &MjModel::flex_evpairnum)
    .property("flex_flatskin", &MjModel::flex_flatskin)
    .property("flex_friction", &MjModel::flex_friction)
    .property("flex_gap", &MjModel::flex_gap)
    .property("flex_group", &MjModel::flex_group)
    .property("flex_internal", &MjModel::flex_internal)
    .property("flex_interp", &MjModel::flex_interp)
    .property("flex_margin", &MjModel::flex_margin)
    .property("flex_matid", &MjModel::flex_matid)
    .property("flex_node", &MjModel::flex_node)
    .property("flex_node0", &MjModel::flex_node0)
    .property("flex_nodeadr", &MjModel::flex_nodeadr)
    .property("flex_nodebodyid", &MjModel::flex_nodebodyid)
    .property("flex_nodenum", &MjModel::flex_nodenum)
    .property("flex_passive", &MjModel::flex_passive)
    .property("flex_priority", &MjModel::flex_priority)
    .property("flex_radius", &MjModel::flex_radius)
    .property("flex_rgba", &MjModel::flex_rgba)
    .property("flex_rigid", &MjModel::flex_rigid)
    .property("flex_selfcollide", &MjModel::flex_selfcollide)
    .property("flex_shell", &MjModel::flex_shell)
    .property("flex_shelldataadr", &MjModel::flex_shelldataadr)
    .property("flex_shellnum", &MjModel::flex_shellnum)
    .property("flex_solimp", &MjModel::flex_solimp)
    .property("flex_solmix", &MjModel::flex_solmix)
    .property("flex_solref", &MjModel::flex_solref)
    .property("flex_stiffness", &MjModel::flex_stiffness)
    .property("flex_texcoord", &MjModel::flex_texcoord)
    .property("flex_texcoordadr", &MjModel::flex_texcoordadr)
    .property("flex_vert", &MjModel::flex_vert)
    .property("flex_vert0", &MjModel::flex_vert0)
    .property("flex_vertadr", &MjModel::flex_vertadr)
    .property("flex_vertbodyid", &MjModel::flex_vertbodyid)
    .property("flex_vertnum", &MjModel::flex_vertnum)
    .property("flexedge_invweight0", &MjModel::flexedge_invweight0)
    .property("flexedge_length0", &MjModel::flexedge_length0)
    .property("flexedge_rigid", &MjModel::flexedge_rigid)
    .property("geom_aabb", &MjModel::geom_aabb)
    .property("geom_bodyid", &MjModel::geom_bodyid)
    .property("geom_conaffinity", &MjModel::geom_conaffinity)
    .property("geom_condim", &MjModel::geom_condim)
    .property("geom_contype", &MjModel::geom_contype)
    .property("geom_dataid", &MjModel::geom_dataid)
    .property("geom_fluid", &MjModel::geom_fluid)
    .property("geom_friction", &MjModel::geom_friction)
    .property("geom_gap", &MjModel::geom_gap)
    .property("geom_group", &MjModel::geom_group)
    .property("geom_margin", &MjModel::geom_margin)
    .property("geom_matid", &MjModel::geom_matid)
    .property("geom_plugin", &MjModel::geom_plugin)
    .property("geom_pos", &MjModel::geom_pos)
    .property("geom_priority", &MjModel::geom_priority)
    .property("geom_quat", &MjModel::geom_quat)
    .property("geom_rbound", &MjModel::geom_rbound)
    .property("geom_rgba", &MjModel::geom_rgba)
    .property("geom_sameframe", &MjModel::geom_sameframe)
    .property("geom_size", &MjModel::geom_size)
    .property("geom_solimp", &MjModel::geom_solimp)
    .property("geom_solmix", &MjModel::geom_solmix)
    .property("geom_solref", &MjModel::geom_solref)
    .property("geom_type", &MjModel::geom_type)
    .property("geom_user", &MjModel::geom_user)
    .property("hfield_adr", &MjModel::hfield_adr)
    .property("hfield_data", &MjModel::hfield_data)
    .property("hfield_ncol", &MjModel::hfield_ncol)
    .property("hfield_nrow", &MjModel::hfield_nrow)
    .property("hfield_pathadr", &MjModel::hfield_pathadr)
    .property("hfield_size", &MjModel::hfield_size)
    .property("jnt_actfrclimited", &MjModel::jnt_actfrclimited)
    .property("jnt_actfrcrange", &MjModel::jnt_actfrcrange)
    .property("jnt_actgravcomp", &MjModel::jnt_actgravcomp)
    .property("jnt_axis", &MjModel::jnt_axis)
    .property("jnt_bodyid", &MjModel::jnt_bodyid)
    .property("jnt_dofadr", &MjModel::jnt_dofadr)
    .property("jnt_group", &MjModel::jnt_group)
    .property("jnt_limited", &MjModel::jnt_limited)
    .property("jnt_margin", &MjModel::jnt_margin)
    .property("jnt_pos", &MjModel::jnt_pos)
    .property("jnt_qposadr", &MjModel::jnt_qposadr)
    .property("jnt_range", &MjModel::jnt_range)
    .property("jnt_solimp", &MjModel::jnt_solimp)
    .property("jnt_solref", &MjModel::jnt_solref)
    .property("jnt_stiffness", &MjModel::jnt_stiffness)
    .property("jnt_type", &MjModel::jnt_type)
    .property("jnt_user", &MjModel::jnt_user)
    .property("key_act", &MjModel::key_act)
    .property("key_ctrl", &MjModel::key_ctrl)
    .property("key_mpos", &MjModel::key_mpos)
    .property("key_mquat", &MjModel::key_mquat)
    .property("key_qpos", &MjModel::key_qpos)
    .property("key_qvel", &MjModel::key_qvel)
    .property("key_time", &MjModel::key_time)
    .property("light_active", &MjModel::light_active)
    .property("light_ambient", &MjModel::light_ambient)
    .property("light_attenuation", &MjModel::light_attenuation)
    .property("light_bodyid", &MjModel::light_bodyid)
    .property("light_bulbradius", &MjModel::light_bulbradius)
    .property("light_castshadow", &MjModel::light_castshadow)
    .property("light_cutoff", &MjModel::light_cutoff)
    .property("light_diffuse", &MjModel::light_diffuse)
    .property("light_dir", &MjModel::light_dir)
    .property("light_dir0", &MjModel::light_dir0)
    .property("light_exponent", &MjModel::light_exponent)
    .property("light_intensity", &MjModel::light_intensity)
    .property("light_mode", &MjModel::light_mode)
    .property("light_pos", &MjModel::light_pos)
    .property("light_pos0", &MjModel::light_pos0)
    .property("light_poscom0", &MjModel::light_poscom0)
    .property("light_range", &MjModel::light_range)
    .property("light_specular", &MjModel::light_specular)
    .property("light_targetbodyid", &MjModel::light_targetbodyid)
    .property("light_texid", &MjModel::light_texid)
    .property("light_type", &MjModel::light_type)
    .property("mapD2M", &MjModel::mapD2M)
    .property("mapM2D", &MjModel::mapM2D)
    .property("mapM2M", &MjModel::mapM2M)
    .property("mat_emission", &MjModel::mat_emission)
    .property("mat_metallic", &MjModel::mat_metallic)
    .property("mat_reflectance", &MjModel::mat_reflectance)
    .property("mat_rgba", &MjModel::mat_rgba)
    .property("mat_roughness", &MjModel::mat_roughness)
    .property("mat_shininess", &MjModel::mat_shininess)
    .property("mat_specular", &MjModel::mat_specular)
    .property("mat_texid", &MjModel::mat_texid)
    .property("mat_texrepeat", &MjModel::mat_texrepeat)
    .property("mat_texuniform", &MjModel::mat_texuniform)
    .property("mesh_bvhadr", &MjModel::mesh_bvhadr)
    .property("mesh_bvhnum", &MjModel::mesh_bvhnum)
    .property("mesh_face", &MjModel::mesh_face)
    .property("mesh_faceadr", &MjModel::mesh_faceadr)
    .property("mesh_facenormal", &MjModel::mesh_facenormal)
    .property("mesh_facenum", &MjModel::mesh_facenum)
    .property("mesh_facetexcoord", &MjModel::mesh_facetexcoord)
    .property("mesh_graph", &MjModel::mesh_graph)
    .property("mesh_graphadr", &MjModel::mesh_graphadr)
    .property("mesh_normal", &MjModel::mesh_normal)
    .property("mesh_normaladr", &MjModel::mesh_normaladr)
    .property("mesh_normalnum", &MjModel::mesh_normalnum)
    .property("mesh_octadr", &MjModel::mesh_octadr)
    .property("mesh_octnum", &MjModel::mesh_octnum)
    .property("mesh_pathadr", &MjModel::mesh_pathadr)
    .property("mesh_polyadr", &MjModel::mesh_polyadr)
    .property("mesh_polymap", &MjModel::mesh_polymap)
    .property("mesh_polymapadr", &MjModel::mesh_polymapadr)
    .property("mesh_polymapnum", &MjModel::mesh_polymapnum)
    .property("mesh_polynormal", &MjModel::mesh_polynormal)
    .property("mesh_polynum", &MjModel::mesh_polynum)
    .property("mesh_polyvert", &MjModel::mesh_polyvert)
    .property("mesh_polyvertadr", &MjModel::mesh_polyvertadr)
    .property("mesh_polyvertnum", &MjModel::mesh_polyvertnum)
    .property("mesh_pos", &MjModel::mesh_pos)
    .property("mesh_quat", &MjModel::mesh_quat)
    .property("mesh_scale", &MjModel::mesh_scale)
    .property("mesh_texcoord", &MjModel::mesh_texcoord)
    .property("mesh_texcoordadr", &MjModel::mesh_texcoordadr)
    .property("mesh_texcoordnum", &MjModel::mesh_texcoordnum)
    .property("mesh_vert", &MjModel::mesh_vert)
    .property("mesh_vertadr", &MjModel::mesh_vertadr)
    .property("mesh_vertnum", &MjModel::mesh_vertnum)
    .property("nB", &MjModel::nB, &MjModel::set_nB, reference())
    .property("nC", &MjModel::nC, &MjModel::set_nC, reference())
    .property("nD", &MjModel::nD, &MjModel::set_nD, reference())
    .property("nJmom", &MjModel::nJmom, &MjModel::set_nJmom, reference())
    .property("nM", &MjModel::nM, &MjModel::set_nM, reference())
    .property("na", &MjModel::na, &MjModel::set_na, reference())
    .property("name_actuatoradr", &MjModel::name_actuatoradr)
    .property("name_bodyadr", &MjModel::name_bodyadr)
    .property("name_camadr", &MjModel::name_camadr)
    .property("name_eqadr", &MjModel::name_eqadr)
    .property("name_excludeadr", &MjModel::name_excludeadr)
    .property("name_flexadr", &MjModel::name_flexadr)
    .property("name_geomadr", &MjModel::name_geomadr)
    .property("name_hfieldadr", &MjModel::name_hfieldadr)
    .property("name_jntadr", &MjModel::name_jntadr)
    .property("name_keyadr", &MjModel::name_keyadr)
    .property("name_lightadr", &MjModel::name_lightadr)
    .property("name_matadr", &MjModel::name_matadr)
    .property("name_meshadr", &MjModel::name_meshadr)
    .property("name_numericadr", &MjModel::name_numericadr)
    .property("name_pairadr", &MjModel::name_pairadr)
    .property("name_pluginadr", &MjModel::name_pluginadr)
    .property("name_sensoradr", &MjModel::name_sensoradr)
    .property("name_siteadr", &MjModel::name_siteadr)
    .property("name_skinadr", &MjModel::name_skinadr)
    .property("name_tendonadr", &MjModel::name_tendonadr)
    .property("name_texadr", &MjModel::name_texadr)
    .property("name_textadr", &MjModel::name_textadr)
    .property("name_tupleadr", &MjModel::name_tupleadr)
    .property("names", &MjModel::names)
    .property("names_map", &MjModel::names_map)
    .property("narena", &MjModel::narena, &MjModel::set_narena, reference())
    .property("nbody", &MjModel::nbody, &MjModel::set_nbody, reference())
    .property("nbuffer", &MjModel::nbuffer, &MjModel::set_nbuffer, reference())
    .property("nbvh", &MjModel::nbvh, &MjModel::set_nbvh, reference())
    .property("nbvhdynamic", &MjModel::nbvhdynamic, &MjModel::set_nbvhdynamic, reference())
    .property("nbvhstatic", &MjModel::nbvhstatic, &MjModel::set_nbvhstatic, reference())
    .property("ncam", &MjModel::ncam, &MjModel::set_ncam, reference())
    .property("nconmax", &MjModel::nconmax, &MjModel::set_nconmax, reference())
    .property("nemax", &MjModel::nemax, &MjModel::set_nemax, reference())
    .property("neq", &MjModel::neq, &MjModel::set_neq, reference())
    .property("nexclude", &MjModel::nexclude, &MjModel::set_nexclude, reference())
    .property("nflex", &MjModel::nflex, &MjModel::set_nflex, reference())
    .property("nflexedge", &MjModel::nflexedge, &MjModel::set_nflexedge, reference())
    .property("nflexelem", &MjModel::nflexelem, &MjModel::set_nflexelem, reference())
    .property("nflexelemdata", &MjModel::nflexelemdata, &MjModel::set_nflexelemdata, reference())
    .property("nflexelemedge", &MjModel::nflexelemedge, &MjModel::set_nflexelemedge, reference())
    .property("nflexevpair", &MjModel::nflexevpair, &MjModel::set_nflexevpair, reference())
    .property("nflexnode", &MjModel::nflexnode, &MjModel::set_nflexnode, reference())
    .property("nflexshelldata", &MjModel::nflexshelldata, &MjModel::set_nflexshelldata, reference())
    .property("nflextexcoord", &MjModel::nflextexcoord, &MjModel::set_nflextexcoord, reference())
    .property("nflexvert", &MjModel::nflexvert, &MjModel::set_nflexvert, reference())
    .property("ngeom", &MjModel::ngeom, &MjModel::set_ngeom, reference())
    .property("ngravcomp", &MjModel::ngravcomp, &MjModel::set_ngravcomp, reference())
    .property("nhfield", &MjModel::nhfield, &MjModel::set_nhfield, reference())
    .property("nhfielddata", &MjModel::nhfielddata, &MjModel::set_nhfielddata, reference())
    .property("njmax", &MjModel::njmax, &MjModel::set_njmax, reference())
    .property("njnt", &MjModel::njnt, &MjModel::set_njnt, reference())
    .property("nkey", &MjModel::nkey, &MjModel::set_nkey, reference())
    .property("nlight", &MjModel::nlight, &MjModel::set_nlight, reference())
    .property("nmat", &MjModel::nmat, &MjModel::set_nmat, reference())
    .property("nmesh", &MjModel::nmesh, &MjModel::set_nmesh, reference())
    .property("nmeshface", &MjModel::nmeshface, &MjModel::set_nmeshface, reference())
    .property("nmeshgraph", &MjModel::nmeshgraph, &MjModel::set_nmeshgraph, reference())
    .property("nmeshnormal", &MjModel::nmeshnormal, &MjModel::set_nmeshnormal, reference())
    .property("nmeshpoly", &MjModel::nmeshpoly, &MjModel::set_nmeshpoly, reference())
    .property("nmeshpolymap", &MjModel::nmeshpolymap, &MjModel::set_nmeshpolymap, reference())
    .property("nmeshpolyvert", &MjModel::nmeshpolyvert, &MjModel::set_nmeshpolyvert, reference())
    .property("nmeshtexcoord", &MjModel::nmeshtexcoord, &MjModel::set_nmeshtexcoord, reference())
    .property("nmeshvert", &MjModel::nmeshvert, &MjModel::set_nmeshvert, reference())
    .property("nmocap", &MjModel::nmocap, &MjModel::set_nmocap, reference())
    .property("nnames", &MjModel::nnames, &MjModel::set_nnames, reference())
    .property("nnames_map", &MjModel::nnames_map, &MjModel::set_nnames_map, reference())
    .property("nnumeric", &MjModel::nnumeric, &MjModel::set_nnumeric, reference())
    .property("nnumericdata", &MjModel::nnumericdata, &MjModel::set_nnumericdata, reference())
    .property("noct", &MjModel::noct, &MjModel::set_noct, reference())
    .property("npair", &MjModel::npair, &MjModel::set_npair, reference())
    .property("npaths", &MjModel::npaths, &MjModel::set_npaths, reference())
    .property("nplugin", &MjModel::nplugin, &MjModel::set_nplugin, reference())
    .property("npluginattr", &MjModel::npluginattr, &MjModel::set_npluginattr, reference())
    .property("npluginstate", &MjModel::npluginstate, &MjModel::set_npluginstate, reference())
    .property("nq", &MjModel::nq, &MjModel::set_nq, reference())
    .property("nsensor", &MjModel::nsensor, &MjModel::set_nsensor, reference())
    .property("nsensordata", &MjModel::nsensordata, &MjModel::set_nsensordata, reference())
    .property("nsite", &MjModel::nsite, &MjModel::set_nsite, reference())
    .property("nskin", &MjModel::nskin, &MjModel::set_nskin, reference())
    .property("nskinbone", &MjModel::nskinbone, &MjModel::set_nskinbone, reference())
    .property("nskinbonevert", &MjModel::nskinbonevert, &MjModel::set_nskinbonevert, reference())
    .property("nskinface", &MjModel::nskinface, &MjModel::set_nskinface, reference())
    .property("nskintexvert", &MjModel::nskintexvert, &MjModel::set_nskintexvert, reference())
    .property("nskinvert", &MjModel::nskinvert, &MjModel::set_nskinvert, reference())
    .property("ntendon", &MjModel::ntendon, &MjModel::set_ntendon, reference())
    .property("ntex", &MjModel::ntex, &MjModel::set_ntex, reference())
    .property("ntexdata", &MjModel::ntexdata, &MjModel::set_ntexdata, reference())
    .property("ntext", &MjModel::ntext, &MjModel::set_ntext, reference())
    .property("ntextdata", &MjModel::ntextdata, &MjModel::set_ntextdata, reference())
    .property("ntree", &MjModel::ntree, &MjModel::set_ntree, reference())
    .property("ntuple", &MjModel::ntuple, &MjModel::set_ntuple, reference())
    .property("ntupledata", &MjModel::ntupledata, &MjModel::set_ntupledata, reference())
    .property("nu", &MjModel::nu, &MjModel::set_nu, reference())
    .property("numeric_adr", &MjModel::numeric_adr)
    .property("numeric_data", &MjModel::numeric_data)
    .property("numeric_size", &MjModel::numeric_size)
    .property("nuser_actuator", &MjModel::nuser_actuator, &MjModel::set_nuser_actuator, reference())
    .property("nuser_body", &MjModel::nuser_body, &MjModel::set_nuser_body, reference())
    .property("nuser_cam", &MjModel::nuser_cam, &MjModel::set_nuser_cam, reference())
    .property("nuser_geom", &MjModel::nuser_geom, &MjModel::set_nuser_geom, reference())
    .property("nuser_jnt", &MjModel::nuser_jnt, &MjModel::set_nuser_jnt, reference())
    .property("nuser_sensor", &MjModel::nuser_sensor, &MjModel::set_nuser_sensor, reference())
    .property("nuser_site", &MjModel::nuser_site, &MjModel::set_nuser_site, reference())
    .property("nuser_tendon", &MjModel::nuser_tendon, &MjModel::set_nuser_tendon, reference())
    .property("nuserdata", &MjModel::nuserdata, &MjModel::set_nuserdata, reference())
    .property("nv", &MjModel::nv, &MjModel::set_nv, reference())
    .property("nwrap", &MjModel::nwrap, &MjModel::set_nwrap, reference())
    .property("oct_aabb", &MjModel::oct_aabb)
    .property("oct_child", &MjModel::oct_child)
    .property("oct_coeff", &MjModel::oct_coeff)
    .property("oct_depth", &MjModel::oct_depth)
    .property("opt", &MjModel::opt, reference())
    .property("pair_dim", &MjModel::pair_dim)
    .property("pair_friction", &MjModel::pair_friction)
    .property("pair_gap", &MjModel::pair_gap)
    .property("pair_geom1", &MjModel::pair_geom1)
    .property("pair_geom2", &MjModel::pair_geom2)
    .property("pair_margin", &MjModel::pair_margin)
    .property("pair_signature", &MjModel::pair_signature)
    .property("pair_solimp", &MjModel::pair_solimp)
    .property("pair_solref", &MjModel::pair_solref)
    .property("pair_solreffriction", &MjModel::pair_solreffriction)
    .property("paths", &MjModel::paths)
    .property("plugin", &MjModel::plugin)
    .property("plugin_attr", &MjModel::plugin_attr)
    .property("plugin_attradr", &MjModel::plugin_attradr)
    .property("plugin_stateadr", &MjModel::plugin_stateadr)
    .property("plugin_statenum", &MjModel::plugin_statenum)
    .property("qpos0", &MjModel::qpos0)
    .property("qpos_spring", &MjModel::qpos_spring)
    .property("sensor_adr", &MjModel::sensor_adr)
    .property("sensor_cutoff", &MjModel::sensor_cutoff)
    .property("sensor_datatype", &MjModel::sensor_datatype)
    .property("sensor_dim", &MjModel::sensor_dim)
    .property("sensor_intprm", &MjModel::sensor_intprm)
    .property("sensor_needstage", &MjModel::sensor_needstage)
    .property("sensor_noise", &MjModel::sensor_noise)
    .property("sensor_objid", &MjModel::sensor_objid)
    .property("sensor_objtype", &MjModel::sensor_objtype)
    .property("sensor_plugin", &MjModel::sensor_plugin)
    .property("sensor_refid", &MjModel::sensor_refid)
    .property("sensor_reftype", &MjModel::sensor_reftype)
    .property("sensor_type", &MjModel::sensor_type)
    .property("sensor_user", &MjModel::sensor_user)
    .property("signature", &MjModel::signature, &MjModel::set_signature, reference())
    .property("site_bodyid", &MjModel::site_bodyid)
    .property("site_group", &MjModel::site_group)
    .property("site_matid", &MjModel::site_matid)
    .property("site_pos", &MjModel::site_pos)
    .property("site_quat", &MjModel::site_quat)
    .property("site_rgba", &MjModel::site_rgba)
    .property("site_sameframe", &MjModel::site_sameframe)
    .property("site_size", &MjModel::site_size)
    .property("site_type", &MjModel::site_type)
    .property("site_user", &MjModel::site_user)
    .property("skin_boneadr", &MjModel::skin_boneadr)
    .property("skin_bonebindpos", &MjModel::skin_bonebindpos)
    .property("skin_bonebindquat", &MjModel::skin_bonebindquat)
    .property("skin_bonebodyid", &MjModel::skin_bonebodyid)
    .property("skin_bonenum", &MjModel::skin_bonenum)
    .property("skin_bonevertadr", &MjModel::skin_bonevertadr)
    .property("skin_bonevertid", &MjModel::skin_bonevertid)
    .property("skin_bonevertnum", &MjModel::skin_bonevertnum)
    .property("skin_bonevertweight", &MjModel::skin_bonevertweight)
    .property("skin_face", &MjModel::skin_face)
    .property("skin_faceadr", &MjModel::skin_faceadr)
    .property("skin_facenum", &MjModel::skin_facenum)
    .property("skin_group", &MjModel::skin_group)
    .property("skin_inflate", &MjModel::skin_inflate)
    .property("skin_matid", &MjModel::skin_matid)
    .property("skin_pathadr", &MjModel::skin_pathadr)
    .property("skin_rgba", &MjModel::skin_rgba)
    .property("skin_texcoord", &MjModel::skin_texcoord)
    .property("skin_texcoordadr", &MjModel::skin_texcoordadr)
    .property("skin_vert", &MjModel::skin_vert)
    .property("skin_vertadr", &MjModel::skin_vertadr)
    .property("skin_vertnum", &MjModel::skin_vertnum)
    .property("stat", &MjModel::stat, reference())
    .property("tendon_actfrclimited", &MjModel::tendon_actfrclimited)
    .property("tendon_actfrcrange", &MjModel::tendon_actfrcrange)
    .property("tendon_adr", &MjModel::tendon_adr)
    .property("tendon_armature", &MjModel::tendon_armature)
    .property("tendon_damping", &MjModel::tendon_damping)
    .property("tendon_frictionloss", &MjModel::tendon_frictionloss)
    .property("tendon_group", &MjModel::tendon_group)
    .property("tendon_invweight0", &MjModel::tendon_invweight0)
    .property("tendon_length0", &MjModel::tendon_length0)
    .property("tendon_lengthspring", &MjModel::tendon_lengthspring)
    .property("tendon_limited", &MjModel::tendon_limited)
    .property("tendon_margin", &MjModel::tendon_margin)
    .property("tendon_matid", &MjModel::tendon_matid)
    .property("tendon_num", &MjModel::tendon_num)
    .property("tendon_range", &MjModel::tendon_range)
    .property("tendon_rgba", &MjModel::tendon_rgba)
    .property("tendon_solimp_fri", &MjModel::tendon_solimp_fri)
    .property("tendon_solimp_lim", &MjModel::tendon_solimp_lim)
    .property("tendon_solref_fri", &MjModel::tendon_solref_fri)
    .property("tendon_solref_lim", &MjModel::tendon_solref_lim)
    .property("tendon_stiffness", &MjModel::tendon_stiffness)
    .property("tendon_treeid", &MjModel::tendon_treeid)
    .property("tendon_treenum", &MjModel::tendon_treenum)
    .property("tendon_user", &MjModel::tendon_user)
    .property("tendon_width", &MjModel::tendon_width)
    .property("tex_adr", &MjModel::tex_adr)
    .property("tex_colorspace", &MjModel::tex_colorspace)
    .property("tex_data", &MjModel::tex_data)
    .property("tex_height", &MjModel::tex_height)
    .property("tex_nchannel", &MjModel::tex_nchannel)
    .property("tex_pathadr", &MjModel::tex_pathadr)
    .property("tex_type", &MjModel::tex_type)
    .property("tex_width", &MjModel::tex_width)
    .property("text_adr", &MjModel::text_adr)
    .property("text_data", &MjModel::text_data)
    .property("text_size", &MjModel::text_size)
    .property("tree_bodyadr", &MjModel::tree_bodyadr)
    .property("tree_bodynum", &MjModel::tree_bodynum)
    .property("tree_dofadr", &MjModel::tree_dofadr)
    .property("tree_dofnum", &MjModel::tree_dofnum)
    .property("tree_sleep_policy", &MjModel::tree_sleep_policy)
    .property("tuple_adr", &MjModel::tuple_adr)
    .property("tuple_objid", &MjModel::tuple_objid)
    .property("tuple_objprm", &MjModel::tuple_objprm)
    .property("tuple_objtype", &MjModel::tuple_objtype)
    .property("tuple_size", &MjModel::tuple_size)
    .property("vis", &MjModel::vis, reference())
    .property("wrap_objid", &MjModel::wrap_objid)
    .property("wrap_prm", &MjModel::wrap_prm)
    .property("wrap_type", &MjModel::wrap_type);
  emscripten::class_<MjOption>("MjOption")
    .constructor<>()
    .function("copy", &MjOption::copy, take_ownership())
    .property("ccd_iterations", &MjOption::ccd_iterations, &MjOption::set_ccd_iterations, reference())
    .property("ccd_tolerance", &MjOption::ccd_tolerance, &MjOption::set_ccd_tolerance, reference())
    .property("cone", &MjOption::cone, &MjOption::set_cone, reference())
    .property("density", &MjOption::density, &MjOption::set_density, reference())
    .property("disableactuator", &MjOption::disableactuator, &MjOption::set_disableactuator, reference())
    .property("disableflags", &MjOption::disableflags, &MjOption::set_disableflags, reference())
    .property("enableflags", &MjOption::enableflags, &MjOption::set_enableflags, reference())
    .property("gravity", &MjOption::gravity)
    .property("impratio", &MjOption::impratio, &MjOption::set_impratio, reference())
    .property("integrator", &MjOption::integrator, &MjOption::set_integrator, reference())
    .property("iterations", &MjOption::iterations, &MjOption::set_iterations, reference())
    .property("jacobian", &MjOption::jacobian, &MjOption::set_jacobian, reference())
    .property("ls_iterations", &MjOption::ls_iterations, &MjOption::set_ls_iterations, reference())
    .property("ls_tolerance", &MjOption::ls_tolerance, &MjOption::set_ls_tolerance, reference())
    .property("magnetic", &MjOption::magnetic)
    .property("noslip_iterations", &MjOption::noslip_iterations, &MjOption::set_noslip_iterations, reference())
    .property("noslip_tolerance", &MjOption::noslip_tolerance, &MjOption::set_noslip_tolerance, reference())
    .property("o_friction", &MjOption::o_friction)
    .property("o_margin", &MjOption::o_margin, &MjOption::set_o_margin, reference())
    .property("o_solimp", &MjOption::o_solimp)
    .property("o_solref", &MjOption::o_solref)
    .property("sdf_initpoints", &MjOption::sdf_initpoints, &MjOption::set_sdf_initpoints, reference())
    .property("sdf_iterations", &MjOption::sdf_iterations, &MjOption::set_sdf_iterations, reference())
    .property("sleep_tolerance", &MjOption::sleep_tolerance, &MjOption::set_sleep_tolerance, reference())
    .property("solver", &MjOption::solver, &MjOption::set_solver, reference())
    .property("timestep", &MjOption::timestep, &MjOption::set_timestep, reference())
    .property("tolerance", &MjOption::tolerance, &MjOption::set_tolerance, reference())
    .property("viscosity", &MjOption::viscosity, &MjOption::set_viscosity, reference())
    .property("wind", &MjOption::wind);
  emscripten::class_<MjSolverStat>("MjSolverStat")
    .constructor<>()
    .function("copy", &MjSolverStat::copy, take_ownership())
    .property("gradient", &MjSolverStat::gradient, &MjSolverStat::set_gradient, reference())
    .property("improvement", &MjSolverStat::improvement, &MjSolverStat::set_improvement, reference())
    .property("lineslope", &MjSolverStat::lineslope, &MjSolverStat::set_lineslope, reference())
    .property("nactive", &MjSolverStat::nactive, &MjSolverStat::set_nactive, reference())
    .property("nchange", &MjSolverStat::nchange, &MjSolverStat::set_nchange, reference())
    .property("neval", &MjSolverStat::neval, &MjSolverStat::set_neval, reference())
    .property("nupdate", &MjSolverStat::nupdate, &MjSolverStat::set_nupdate, reference());
  emscripten::class_<MjSpec>("MjSpec")
    .constructor<const MjSpec &>()
    .property("comment", &MjSpec::comment, &MjSpec::set_comment, reference())
    .property("compiler", &MjSpec::compiler, reference())
    .property("element", &MjSpec::element, reference())
    .property("hasImplicitPluginElem", &MjSpec::hasImplicitPluginElem, &MjSpec::set_hasImplicitPluginElem, reference())
    .property("memory", &MjSpec::memory, &MjSpec::set_memory, reference())
    .property("modelfiledir", &MjSpec::modelfiledir, &MjSpec::set_modelfiledir, reference())
    .property("modelname", &MjSpec::modelname, &MjSpec::set_modelname, reference())
    .property("nconmax", &MjSpec::nconmax, &MjSpec::set_nconmax, reference())
    .property("nemax", &MjSpec::nemax, &MjSpec::set_nemax, reference())
    .property("njmax", &MjSpec::njmax, &MjSpec::set_njmax, reference())
    .property("nkey", &MjSpec::nkey, &MjSpec::set_nkey, reference())
    .property("nstack", &MjSpec::nstack, &MjSpec::set_nstack, reference())
    .property("nuser_actuator", &MjSpec::nuser_actuator, &MjSpec::set_nuser_actuator, reference())
    .property("nuser_body", &MjSpec::nuser_body, &MjSpec::set_nuser_body, reference())
    .property("nuser_cam", &MjSpec::nuser_cam, &MjSpec::set_nuser_cam, reference())
    .property("nuser_geom", &MjSpec::nuser_geom, &MjSpec::set_nuser_geom, reference())
    .property("nuser_jnt", &MjSpec::nuser_jnt, &MjSpec::set_nuser_jnt, reference())
    .property("nuser_sensor", &MjSpec::nuser_sensor, &MjSpec::set_nuser_sensor, reference())
    .property("nuser_site", &MjSpec::nuser_site, &MjSpec::set_nuser_site, reference())
    .property("nuser_tendon", &MjSpec::nuser_tendon, &MjSpec::set_nuser_tendon, reference())
    .property("nuserdata", &MjSpec::nuserdata, &MjSpec::set_nuserdata, reference())
    .property("option", &MjSpec::option, reference())
    .property("stat", &MjSpec::stat, reference())
    .property("strippath", &MjSpec::strippath, &MjSpec::set_strippath, reference())
    .property("visual", &MjSpec::visual, reference());
  emscripten::class_<MjStatistic>("MjStatistic")
    .constructor<>()
    .function("copy", &MjStatistic::copy, take_ownership())
    .property("center", &MjStatistic::center)
    .property("extent", &MjStatistic::extent, &MjStatistic::set_extent, reference())
    .property("meaninertia", &MjStatistic::meaninertia, &MjStatistic::set_meaninertia, reference())
    .property("meanmass", &MjStatistic::meanmass, &MjStatistic::set_meanmass, reference())
    .property("meansize", &MjStatistic::meansize, &MjStatistic::set_meansize, reference());
  emscripten::class_<MjTimerStat>("MjTimerStat")
    .constructor<>()
    .function("copy", &MjTimerStat::copy, take_ownership())
    .property("duration", &MjTimerStat::duration, &MjTimerStat::set_duration, reference())
    .property("number", &MjTimerStat::number, &MjTimerStat::set_number, reference());
  emscripten::class_<MjVisual>("MjVisual")
    .constructor<>()
    .function("copy", &MjVisual::copy, take_ownership())
    .property("global", &MjVisual::global, reference())
    .property("headlight", &MjVisual::headlight, reference())
    .property("map", &MjVisual::map, reference())
    .property("quality", &MjVisual::quality, reference())
    .property("rgba", &MjVisual::rgba, reference())
    .property("scale", &MjVisual::scale, reference());
  emscripten::class_<MjVisualGlobal>("MjVisualGlobal")
    .constructor<>()
    .function("copy", &MjVisualGlobal::copy, take_ownership())
    .property("azimuth", &MjVisualGlobal::azimuth, &MjVisualGlobal::set_azimuth, reference())
    .property("bvactive", &MjVisualGlobal::bvactive, &MjVisualGlobal::set_bvactive, reference())
    .property("cameraid", &MjVisualGlobal::cameraid, &MjVisualGlobal::set_cameraid, reference())
    .property("elevation", &MjVisualGlobal::elevation, &MjVisualGlobal::set_elevation, reference())
    .property("ellipsoidinertia", &MjVisualGlobal::ellipsoidinertia, &MjVisualGlobal::set_ellipsoidinertia, reference())
    .property("fovy", &MjVisualGlobal::fovy, &MjVisualGlobal::set_fovy, reference())
    .property("glow", &MjVisualGlobal::glow, &MjVisualGlobal::set_glow, reference())
    .property("ipd", &MjVisualGlobal::ipd, &MjVisualGlobal::set_ipd, reference())
    .property("linewidth", &MjVisualGlobal::linewidth, &MjVisualGlobal::set_linewidth, reference())
    .property("offheight", &MjVisualGlobal::offheight, &MjVisualGlobal::set_offheight, reference())
    .property("offwidth", &MjVisualGlobal::offwidth, &MjVisualGlobal::set_offwidth, reference())
    .property("orthographic", &MjVisualGlobal::orthographic, &MjVisualGlobal::set_orthographic, reference())
    .property("realtime", &MjVisualGlobal::realtime, &MjVisualGlobal::set_realtime, reference());
  emscripten::class_<MjVisualHeadlight>("MjVisualHeadlight")
    .constructor<>()
    .function("copy", &MjVisualHeadlight::copy, take_ownership())
    .property("active", &MjVisualHeadlight::active, &MjVisualHeadlight::set_active, reference())
    .property("ambient", &MjVisualHeadlight::ambient)
    .property("diffuse", &MjVisualHeadlight::diffuse)
    .property("specular", &MjVisualHeadlight::specular);
  emscripten::class_<MjVisualMap>("MjVisualMap")
    .constructor<>()
    .function("copy", &MjVisualMap::copy, take_ownership())
    .property("actuatortendon", &MjVisualMap::actuatortendon, &MjVisualMap::set_actuatortendon, reference())
    .property("alpha", &MjVisualMap::alpha, &MjVisualMap::set_alpha, reference())
    .property("fogend", &MjVisualMap::fogend, &MjVisualMap::set_fogend, reference())
    .property("fogstart", &MjVisualMap::fogstart, &MjVisualMap::set_fogstart, reference())
    .property("force", &MjVisualMap::force, &MjVisualMap::set_force, reference())
    .property("haze", &MjVisualMap::haze, &MjVisualMap::set_haze, reference())
    .property("shadowclip", &MjVisualMap::shadowclip, &MjVisualMap::set_shadowclip, reference())
    .property("shadowscale", &MjVisualMap::shadowscale, &MjVisualMap::set_shadowscale, reference())
    .property("stiffness", &MjVisualMap::stiffness, &MjVisualMap::set_stiffness, reference())
    .property("stiffnessrot", &MjVisualMap::stiffnessrot, &MjVisualMap::set_stiffnessrot, reference())
    .property("torque", &MjVisualMap::torque, &MjVisualMap::set_torque, reference())
    .property("zfar", &MjVisualMap::zfar, &MjVisualMap::set_zfar, reference())
    .property("znear", &MjVisualMap::znear, &MjVisualMap::set_znear, reference());
  emscripten::class_<MjVisualQuality>("MjVisualQuality")
    .constructor<>()
    .function("copy", &MjVisualQuality::copy, take_ownership())
    .property("numquads", &MjVisualQuality::numquads, &MjVisualQuality::set_numquads, reference())
    .property("numslices", &MjVisualQuality::numslices, &MjVisualQuality::set_numslices, reference())
    .property("numstacks", &MjVisualQuality::numstacks, &MjVisualQuality::set_numstacks, reference())
    .property("offsamples", &MjVisualQuality::offsamples, &MjVisualQuality::set_offsamples, reference())
    .property("shadowsize", &MjVisualQuality::shadowsize, &MjVisualQuality::set_shadowsize, reference());
  emscripten::class_<MjVisualRgba>("MjVisualRgba")
    .constructor<>()
    .function("copy", &MjVisualRgba::copy, take_ownership())
    .property("actuator", &MjVisualRgba::actuator)
    .property("actuatornegative", &MjVisualRgba::actuatornegative)
    .property("actuatorpositive", &MjVisualRgba::actuatorpositive)
    .property("bv", &MjVisualRgba::bv)
    .property("bvactive", &MjVisualRgba::bvactive)
    .property("camera", &MjVisualRgba::camera)
    .property("com", &MjVisualRgba::com)
    .property("connect", &MjVisualRgba::connect)
    .property("constraint", &MjVisualRgba::constraint)
    .property("contactforce", &MjVisualRgba::contactforce)
    .property("contactfriction", &MjVisualRgba::contactfriction)
    .property("contactgap", &MjVisualRgba::contactgap)
    .property("contactpoint", &MjVisualRgba::contactpoint)
    .property("contacttorque", &MjVisualRgba::contacttorque)
    .property("crankbroken", &MjVisualRgba::crankbroken)
    .property("fog", &MjVisualRgba::fog)
    .property("force", &MjVisualRgba::force)
    .property("frustum", &MjVisualRgba::frustum)
    .property("haze", &MjVisualRgba::haze)
    .property("inertia", &MjVisualRgba::inertia)
    .property("joint", &MjVisualRgba::joint)
    .property("light", &MjVisualRgba::light)
    .property("rangefinder", &MjVisualRgba::rangefinder)
    .property("selectpoint", &MjVisualRgba::selectpoint)
    .property("slidercrank", &MjVisualRgba::slidercrank);
  emscripten::class_<MjVisualScale>("MjVisualScale")
    .constructor<>()
    .function("copy", &MjVisualScale::copy, take_ownership())
    .property("actuatorlength", &MjVisualScale::actuatorlength, &MjVisualScale::set_actuatorlength, reference())
    .property("actuatorwidth", &MjVisualScale::actuatorwidth, &MjVisualScale::set_actuatorwidth, reference())
    .property("camera", &MjVisualScale::camera, &MjVisualScale::set_camera, reference())
    .property("com", &MjVisualScale::com, &MjVisualScale::set_com, reference())
    .property("connect", &MjVisualScale::connect, &MjVisualScale::set_connect, reference())
    .property("constraint", &MjVisualScale::constraint, &MjVisualScale::set_constraint, reference())
    .property("contactheight", &MjVisualScale::contactheight, &MjVisualScale::set_contactheight, reference())
    .property("contactwidth", &MjVisualScale::contactwidth, &MjVisualScale::set_contactwidth, reference())
    .property("forcewidth", &MjVisualScale::forcewidth, &MjVisualScale::set_forcewidth, reference())
    .property("framelength", &MjVisualScale::framelength, &MjVisualScale::set_framelength, reference())
    .property("framewidth", &MjVisualScale::framewidth, &MjVisualScale::set_framewidth, reference())
    .property("frustum", &MjVisualScale::frustum, &MjVisualScale::set_frustum, reference())
    .property("jointlength", &MjVisualScale::jointlength, &MjVisualScale::set_jointlength, reference())
    .property("jointwidth", &MjVisualScale::jointwidth, &MjVisualScale::set_jointwidth, reference())
    .property("light", &MjVisualScale::light, &MjVisualScale::set_light, reference())
    .property("selectpoint", &MjVisualScale::selectpoint, &MjVisualScale::set_selectpoint, reference())
    .property("slidercrank", &MjVisualScale::slidercrank, &MjVisualScale::set_slidercrank, reference());
  emscripten::class_<MjWarningStat>("MjWarningStat")
    .constructor<>()
    .function("copy", &MjWarningStat::copy, take_ownership())
    .property("lastinfo", &MjWarningStat::lastinfo, &MjWarningStat::set_lastinfo, reference())
    .property("number", &MjWarningStat::number, &MjWarningStat::set_number, reference());
  emscripten::class_<MjsActuator>("MjsActuator")
    .property("actdim", &MjsActuator::actdim, &MjsActuator::set_actdim, reference())
    .property("actearly", &MjsActuator::actearly, &MjsActuator::set_actearly, reference())
    .property("actlimited", &MjsActuator::actlimited, &MjsActuator::set_actlimited, reference())
    .property("actrange", &MjsActuator::actrange)
    .property("biasprm", &MjsActuator::biasprm)
    .property("biastype", &MjsActuator::biastype, &MjsActuator::set_biastype, reference())
    .property("cranklength", &MjsActuator::cranklength, &MjsActuator::set_cranklength, reference())
    .property("ctrllimited", &MjsActuator::ctrllimited, &MjsActuator::set_ctrllimited, reference())
    .property("ctrlrange", &MjsActuator::ctrlrange)
    .property("dynprm", &MjsActuator::dynprm)
    .property("dyntype", &MjsActuator::dyntype, &MjsActuator::set_dyntype, reference())
    .property("element", &MjsActuator::element, reference())
    .property("forcelimited", &MjsActuator::forcelimited, &MjsActuator::set_forcelimited, reference())
    .property("forcerange", &MjsActuator::forcerange)
    .property("gainprm", &MjsActuator::gainprm)
    .property("gaintype", &MjsActuator::gaintype, &MjsActuator::set_gaintype, reference())
    .property("gear", &MjsActuator::gear)
    .property("group", &MjsActuator::group, &MjsActuator::set_group, reference())
    .property("info", &MjsActuator::info, &MjsActuator::set_info, reference())
    .property("inheritrange", &MjsActuator::inheritrange, &MjsActuator::set_inheritrange, reference())
    .property("lengthrange", &MjsActuator::lengthrange)
    .property("plugin", &MjsActuator::plugin, reference())
    .property("refsite", &MjsActuator::refsite, &MjsActuator::set_refsite, reference())
    .property("slidersite", &MjsActuator::slidersite, &MjsActuator::set_slidersite, reference())
    .property("target", &MjsActuator::target, &MjsActuator::set_target, reference())
    .property("trntype", &MjsActuator::trntype, &MjsActuator::set_trntype, reference())
    .property("userdata", &MjsActuator::userdata, reference());
  emscripten::class_<MjsBody>("MjsBody")
    .property("alt", &MjsBody::alt, reference())
    .property("childclass", &MjsBody::childclass, &MjsBody::set_childclass, reference())
    .property("element", &MjsBody::element, reference())
    .property("explicitinertial", &MjsBody::explicitinertial, &MjsBody::set_explicitinertial, reference())
    .property("fullinertia", &MjsBody::fullinertia)
    .property("gravcomp", &MjsBody::gravcomp, &MjsBody::set_gravcomp, reference())
    .property("ialt", &MjsBody::ialt, reference())
    .property("inertia", &MjsBody::inertia)
    .property("info", &MjsBody::info, &MjsBody::set_info, reference())
    .property("ipos", &MjsBody::ipos)
    .property("iquat", &MjsBody::iquat)
    .property("mass", &MjsBody::mass, &MjsBody::set_mass, reference())
    .property("mocap", &MjsBody::mocap, &MjsBody::set_mocap, reference())
    .property("plugin", &MjsBody::plugin, reference())
    .property("pos", &MjsBody::pos)
    .property("quat", &MjsBody::quat)
    .property("sleep", &MjsBody::sleep, &MjsBody::set_sleep, reference())
    .property("userdata", &MjsBody::userdata, reference());
  emscripten::class_<MjsCamera>("MjsCamera")
    .property("alt", &MjsCamera::alt, reference())
    .property("element", &MjsCamera::element, reference())
    .property("focal_length", &MjsCamera::focal_length)
    .property("focal_pixel", &MjsCamera::focal_pixel)
    .property("fovy", &MjsCamera::fovy, &MjsCamera::set_fovy, reference())
    .property("info", &MjsCamera::info, &MjsCamera::set_info, reference())
    .property("intrinsic", &MjsCamera::intrinsic)
    .property("ipd", &MjsCamera::ipd, &MjsCamera::set_ipd, reference())
    .property("mode", &MjsCamera::mode, &MjsCamera::set_mode, reference())
    .property("orthographic", &MjsCamera::orthographic, &MjsCamera::set_orthographic, reference())
    .property("pos", &MjsCamera::pos)
    .property("principal_length", &MjsCamera::principal_length)
    .property("principal_pixel", &MjsCamera::principal_pixel)
    .property("quat", &MjsCamera::quat)
    .property("resolution", &MjsCamera::resolution)
    .property("sensor_size", &MjsCamera::sensor_size)
    .property("targetbody", &MjsCamera::targetbody, &MjsCamera::set_targetbody, reference())
    .property("userdata", &MjsCamera::userdata, reference());
  emscripten::class_<MjsCompiler>("MjsCompiler")
    .property("LRopt", &MjsCompiler::LRopt, reference())
    .property("alignfree", &MjsCompiler::alignfree, &MjsCompiler::set_alignfree, reference())
    .property("autolimits", &MjsCompiler::autolimits, &MjsCompiler::set_autolimits, reference())
    .property("balanceinertia", &MjsCompiler::balanceinertia, &MjsCompiler::set_balanceinertia, reference())
    .property("boundinertia", &MjsCompiler::boundinertia, &MjsCompiler::set_boundinertia, reference())
    .property("boundmass", &MjsCompiler::boundmass, &MjsCompiler::set_boundmass, reference())
    .property("degree", &MjsCompiler::degree, &MjsCompiler::set_degree, reference())
    .property("discardvisual", &MjsCompiler::discardvisual, &MjsCompiler::set_discardvisual, reference())
    .property("eulerseq", &MjsCompiler::eulerseq)
    .property("fitaabb", &MjsCompiler::fitaabb, &MjsCompiler::set_fitaabb, reference())
    .property("fusestatic", &MjsCompiler::fusestatic, &MjsCompiler::set_fusestatic, reference())
    .property("inertiafromgeom", &MjsCompiler::inertiafromgeom, &MjsCompiler::set_inertiafromgeom, reference())
    .property("inertiagrouprange", &MjsCompiler::inertiagrouprange)
    .property("meshdir", &MjsCompiler::meshdir, &MjsCompiler::set_meshdir, reference())
    .property("saveinertial", &MjsCompiler::saveinertial, &MjsCompiler::set_saveinertial, reference())
    .property("settotalmass", &MjsCompiler::settotalmass, &MjsCompiler::set_settotalmass, reference())
    .property("texturedir", &MjsCompiler::texturedir, &MjsCompiler::set_texturedir, reference())
    .property("usethread", &MjsCompiler::usethread, &MjsCompiler::set_usethread, reference());
  emscripten::class_<MjsDefault>("MjsDefault")
    .property("actuator", &MjsDefault::actuator, reference())
    .property("camera", &MjsDefault::camera, reference())
    .property("element", &MjsDefault::element, reference())
    .property("equality", &MjsDefault::equality, reference())
    .property("flex", &MjsDefault::flex, reference())
    .property("geom", &MjsDefault::geom, reference())
    .property("joint", &MjsDefault::joint, reference())
    .property("light", &MjsDefault::light, reference())
    .property("material", &MjsDefault::material, reference())
    .property("mesh", &MjsDefault::mesh, reference())
    .property("pair", &MjsDefault::pair, reference())
    .property("site", &MjsDefault::site, reference())
    .property("tendon", &MjsDefault::tendon, reference());
  emscripten::class_<MjsElement>("MjsElement")
    .property("elemtype", &MjsElement::elemtype, &MjsElement::set_elemtype, reference())
    .property("signature", &MjsElement::signature, &MjsElement::set_signature, reference());
  emscripten::class_<MjsEquality>("MjsEquality")
    .property("active", &MjsEquality::active, &MjsEquality::set_active, reference())
    .property("data", &MjsEquality::data)
    .property("element", &MjsEquality::element, reference())
    .property("info", &MjsEquality::info, &MjsEquality::set_info, reference())
    .property("name1", &MjsEquality::name1, &MjsEquality::set_name1, reference())
    .property("name2", &MjsEquality::name2, &MjsEquality::set_name2, reference())
    .property("objtype", &MjsEquality::objtype, &MjsEquality::set_objtype, reference())
    .property("solimp", &MjsEquality::solimp)
    .property("solref", &MjsEquality::solref)
    .property("type", &MjsEquality::type, &MjsEquality::set_type, reference());
  emscripten::class_<MjsExclude>("MjsExclude")
    .property("bodyname1", &MjsExclude::bodyname1, &MjsExclude::set_bodyname1, reference())
    .property("bodyname2", &MjsExclude::bodyname2, &MjsExclude::set_bodyname2, reference())
    .property("element", &MjsExclude::element, reference())
    .property("info", &MjsExclude::info, &MjsExclude::set_info, reference());
  emscripten::class_<MjsFlex>("MjsFlex")
    .property("activelayers", &MjsFlex::activelayers, &MjsFlex::set_activelayers, reference())
    .property("conaffinity", &MjsFlex::conaffinity, &MjsFlex::set_conaffinity, reference())
    .property("condim", &MjsFlex::condim, &MjsFlex::set_condim, reference())
    .property("contype", &MjsFlex::contype, &MjsFlex::set_contype, reference())
    .property("damping", &MjsFlex::damping, &MjsFlex::set_damping, reference())
    .property("dim", &MjsFlex::dim, &MjsFlex::set_dim, reference())
    .property("edgedamping", &MjsFlex::edgedamping, &MjsFlex::set_edgedamping, reference())
    .property("edgestiffness", &MjsFlex::edgestiffness, &MjsFlex::set_edgestiffness, reference())
    .property("elastic2d", &MjsFlex::elastic2d, &MjsFlex::set_elastic2d, reference())
    .property("elem", &MjsFlex::elem, reference())
    .property("element", &MjsFlex::element, reference())
    .property("elemtexcoord", &MjsFlex::elemtexcoord, reference())
    .property("flatskin", &MjsFlex::flatskin, &MjsFlex::set_flatskin, reference())
    .property("friction", &MjsFlex::friction)
    .property("gap", &MjsFlex::gap, &MjsFlex::set_gap, reference())
    .property("group", &MjsFlex::group, &MjsFlex::set_group, reference())
    .property("info", &MjsFlex::info, &MjsFlex::set_info, reference())
    .property("internal", &MjsFlex::internal, &MjsFlex::set_internal, reference())
    .property("margin", &MjsFlex::margin, &MjsFlex::set_margin, reference())
    .property("material", &MjsFlex::material, &MjsFlex::set_material, reference())
    .property("node", &MjsFlex::node, reference())
    .property("nodebody", &MjsFlex::nodebody, reference())
    .property("passive", &MjsFlex::passive, &MjsFlex::set_passive, reference())
    .property("poisson", &MjsFlex::poisson, &MjsFlex::set_poisson, reference())
    .property("priority", &MjsFlex::priority, &MjsFlex::set_priority, reference())
    .property("radius", &MjsFlex::radius, &MjsFlex::set_radius, reference())
    .property("rgba", &MjsFlex::rgba)
    .property("selfcollide", &MjsFlex::selfcollide, &MjsFlex::set_selfcollide, reference())
    .property("solimp", &MjsFlex::solimp)
    .property("solmix", &MjsFlex::solmix, &MjsFlex::set_solmix, reference())
    .property("solref", &MjsFlex::solref)
    .property("texcoord", &MjsFlex::texcoord, reference())
    .property("thickness", &MjsFlex::thickness, &MjsFlex::set_thickness, reference())
    .property("vert", &MjsFlex::vert, reference())
    .property("vertbody", &MjsFlex::vertbody, reference())
    .property("vertcollide", &MjsFlex::vertcollide, &MjsFlex::set_vertcollide, reference())
    .property("young", &MjsFlex::young, &MjsFlex::set_young, reference());
  emscripten::class_<MjsFrame>("MjsFrame")
    .property("alt", &MjsFrame::alt, reference())
    .property("childclass", &MjsFrame::childclass, &MjsFrame::set_childclass, reference())
    .property("element", &MjsFrame::element, reference())
    .property("info", &MjsFrame::info, &MjsFrame::set_info, reference())
    .property("pos", &MjsFrame::pos)
    .property("quat", &MjsFrame::quat);
  emscripten::class_<MjsGeom>("MjsGeom")
    .property("alt", &MjsGeom::alt, reference())
    .property("conaffinity", &MjsGeom::conaffinity, &MjsGeom::set_conaffinity, reference())
    .property("condim", &MjsGeom::condim, &MjsGeom::set_condim, reference())
    .property("contype", &MjsGeom::contype, &MjsGeom::set_contype, reference())
    .property("density", &MjsGeom::density, &MjsGeom::set_density, reference())
    .property("element", &MjsGeom::element, reference())
    .property("fitscale", &MjsGeom::fitscale, &MjsGeom::set_fitscale, reference())
    .property("fluid_coefs", &MjsGeom::fluid_coefs)
    .property("fluid_ellipsoid", &MjsGeom::fluid_ellipsoid, &MjsGeom::set_fluid_ellipsoid, reference())
    .property("friction", &MjsGeom::friction)
    .property("fromto", &MjsGeom::fromto)
    .property("gap", &MjsGeom::gap, &MjsGeom::set_gap, reference())
    .property("group", &MjsGeom::group, &MjsGeom::set_group, reference())
    .property("hfieldname", &MjsGeom::hfieldname, &MjsGeom::set_hfieldname, reference())
    .property("info", &MjsGeom::info, &MjsGeom::set_info, reference())
    .property("margin", &MjsGeom::margin, &MjsGeom::set_margin, reference())
    .property("mass", &MjsGeom::mass, &MjsGeom::set_mass, reference())
    .property("material", &MjsGeom::material, &MjsGeom::set_material, reference())
    .property("meshname", &MjsGeom::meshname, &MjsGeom::set_meshname, reference())
    .property("plugin", &MjsGeom::plugin, reference())
    .property("pos", &MjsGeom::pos)
    .property("priority", &MjsGeom::priority, &MjsGeom::set_priority, reference())
    .property("quat", &MjsGeom::quat)
    .property("rgba", &MjsGeom::rgba)
    .property("size", &MjsGeom::size)
    .property("solimp", &MjsGeom::solimp)
    .property("solmix", &MjsGeom::solmix, &MjsGeom::set_solmix, reference())
    .property("solref", &MjsGeom::solref)
    .property("type", &MjsGeom::type, &MjsGeom::set_type, reference())
    .property("typeinertia", &MjsGeom::typeinertia, &MjsGeom::set_typeinertia, reference())
    .property("userdata", &MjsGeom::userdata, reference());
  emscripten::class_<MjsHField>("MjsHField")
    .property("content_type", &MjsHField::content_type, &MjsHField::set_content_type, reference())
    .property("element", &MjsHField::element, reference())
    .property("file", &MjsHField::file, &MjsHField::set_file, reference())
    .property("info", &MjsHField::info, &MjsHField::set_info, reference())
    .property("ncol", &MjsHField::ncol, &MjsHField::set_ncol, reference())
    .property("nrow", &MjsHField::nrow, &MjsHField::set_nrow, reference())
    .property("size", &MjsHField::size)
    .property("userdata", &MjsHField::userdata, reference());
  emscripten::class_<MjsJoint>("MjsJoint")
    .property("actfrclimited", &MjsJoint::actfrclimited, &MjsJoint::set_actfrclimited, reference())
    .property("actfrcrange", &MjsJoint::actfrcrange)
    .property("actgravcomp", &MjsJoint::actgravcomp, &MjsJoint::set_actgravcomp, reference())
    .property("align", &MjsJoint::align, &MjsJoint::set_align, reference())
    .property("armature", &MjsJoint::armature, &MjsJoint::set_armature, reference())
    .property("axis", &MjsJoint::axis)
    .property("damping", &MjsJoint::damping, &MjsJoint::set_damping, reference())
    .property("element", &MjsJoint::element, reference())
    .property("frictionloss", &MjsJoint::frictionloss, &MjsJoint::set_frictionloss, reference())
    .property("group", &MjsJoint::group, &MjsJoint::set_group, reference())
    .property("info", &MjsJoint::info, &MjsJoint::set_info, reference())
    .property("limited", &MjsJoint::limited, &MjsJoint::set_limited, reference())
    .property("margin", &MjsJoint::margin, &MjsJoint::set_margin, reference())
    .property("pos", &MjsJoint::pos)
    .property("range", &MjsJoint::range)
    .property("ref", &MjsJoint::ref, &MjsJoint::set_ref, reference())
    .property("solimp_friction", &MjsJoint::solimp_friction)
    .property("solimp_limit", &MjsJoint::solimp_limit)
    .property("solref_friction", &MjsJoint::solref_friction)
    .property("solref_limit", &MjsJoint::solref_limit)
    .property("springdamper", &MjsJoint::springdamper)
    .property("springref", &MjsJoint::springref, &MjsJoint::set_springref, reference())
    .property("stiffness", &MjsJoint::stiffness, &MjsJoint::set_stiffness, reference())
    .property("type", &MjsJoint::type, &MjsJoint::set_type, reference())
    .property("userdata", &MjsJoint::userdata, reference());
  emscripten::class_<MjsKey>("MjsKey")
    .property("act", &MjsKey::act, reference())
    .property("ctrl", &MjsKey::ctrl, reference())
    .property("element", &MjsKey::element, reference())
    .property("info", &MjsKey::info, &MjsKey::set_info, reference())
    .property("mpos", &MjsKey::mpos, reference())
    .property("mquat", &MjsKey::mquat, reference())
    .property("qpos", &MjsKey::qpos, reference())
    .property("qvel", &MjsKey::qvel, reference())
    .property("time", &MjsKey::time, &MjsKey::set_time, reference());
  emscripten::class_<MjsLight>("MjsLight")
    .property("active", &MjsLight::active, &MjsLight::set_active, reference())
    .property("ambient", &MjsLight::ambient)
    .property("attenuation", &MjsLight::attenuation)
    .property("bulbradius", &MjsLight::bulbradius, &MjsLight::set_bulbradius, reference())
    .property("castshadow", &MjsLight::castshadow, &MjsLight::set_castshadow, reference())
    .property("cutoff", &MjsLight::cutoff, &MjsLight::set_cutoff, reference())
    .property("diffuse", &MjsLight::diffuse)
    .property("dir", &MjsLight::dir)
    .property("element", &MjsLight::element, reference())
    .property("exponent", &MjsLight::exponent, &MjsLight::set_exponent, reference())
    .property("info", &MjsLight::info, &MjsLight::set_info, reference())
    .property("intensity", &MjsLight::intensity, &MjsLight::set_intensity, reference())
    .property("mode", &MjsLight::mode, &MjsLight::set_mode, reference())
    .property("pos", &MjsLight::pos)
    .property("range", &MjsLight::range, &MjsLight::set_range, reference())
    .property("specular", &MjsLight::specular)
    .property("targetbody", &MjsLight::targetbody, &MjsLight::set_targetbody, reference())
    .property("texture", &MjsLight::texture, &MjsLight::set_texture, reference())
    .property("type", &MjsLight::type, &MjsLight::set_type, reference());
  emscripten::class_<MjsMaterial>("MjsMaterial")
    .property("element", &MjsMaterial::element, reference())
    .property("emission", &MjsMaterial::emission, &MjsMaterial::set_emission, reference())
    .property("info", &MjsMaterial::info, &MjsMaterial::set_info, reference())
    .property("metallic", &MjsMaterial::metallic, &MjsMaterial::set_metallic, reference())
    .property("reflectance", &MjsMaterial::reflectance, &MjsMaterial::set_reflectance, reference())
    .property("rgba", &MjsMaterial::rgba)
    .property("roughness", &MjsMaterial::roughness, &MjsMaterial::set_roughness, reference())
    .property("shininess", &MjsMaterial::shininess, &MjsMaterial::set_shininess, reference())
    .property("specular", &MjsMaterial::specular, &MjsMaterial::set_specular, reference())
    .property("texrepeat", &MjsMaterial::texrepeat)
    .property("textures", &MjsMaterial::textures, reference())
    .property("texuniform", &MjsMaterial::texuniform, &MjsMaterial::set_texuniform, reference());
  emscripten::class_<MjsMesh>("MjsMesh")
    .property("content_type", &MjsMesh::content_type, &MjsMesh::set_content_type, reference())
    .property("element", &MjsMesh::element, reference())
    .property("file", &MjsMesh::file, &MjsMesh::set_file, reference())
    .property("inertia", &MjsMesh::inertia, &MjsMesh::set_inertia, reference())
    .property("info", &MjsMesh::info, &MjsMesh::set_info, reference())
    .property("material", &MjsMesh::material, &MjsMesh::set_material, reference())
    .property("maxhullvert", &MjsMesh::maxhullvert, &MjsMesh::set_maxhullvert, reference())
    .property("needsdf", &MjsMesh::needsdf, &MjsMesh::set_needsdf, reference())
    .property("plugin", &MjsMesh::plugin, reference())
    .property("refpos", &MjsMesh::refpos)
    .property("refquat", &MjsMesh::refquat)
    .property("scale", &MjsMesh::scale)
    .property("smoothnormal", &MjsMesh::smoothnormal, &MjsMesh::set_smoothnormal, reference())
    .property("userface", &MjsMesh::userface, reference())
    .property("userfacenormal", &MjsMesh::userfacenormal, reference())
    .property("userfacetexcoord", &MjsMesh::userfacetexcoord, reference())
    .property("usernormal", &MjsMesh::usernormal, reference())
    .property("usertexcoord", &MjsMesh::usertexcoord, reference())
    .property("uservert", &MjsMesh::uservert, reference());
  emscripten::class_<MjsNumeric>("MjsNumeric")
    .property("data", &MjsNumeric::data, reference())
    .property("element", &MjsNumeric::element, reference())
    .property("info", &MjsNumeric::info, &MjsNumeric::set_info, reference())
    .property("size", &MjsNumeric::size, &MjsNumeric::set_size, reference());
  emscripten::class_<MjsOrientation>("MjsOrientation")
    .property("axisangle", &MjsOrientation::axisangle)
    .property("euler", &MjsOrientation::euler)
    .property("type", &MjsOrientation::type, &MjsOrientation::set_type, reference())
    .property("xyaxes", &MjsOrientation::xyaxes)
    .property("zaxis", &MjsOrientation::zaxis);
  emscripten::class_<MjsPair>("MjsPair")
    .property("condim", &MjsPair::condim, &MjsPair::set_condim, reference())
    .property("element", &MjsPair::element, reference())
    .property("friction", &MjsPair::friction)
    .property("gap", &MjsPair::gap, &MjsPair::set_gap, reference())
    .property("geomname1", &MjsPair::geomname1, &MjsPair::set_geomname1, reference())
    .property("geomname2", &MjsPair::geomname2, &MjsPair::set_geomname2, reference())
    .property("info", &MjsPair::info, &MjsPair::set_info, reference())
    .property("margin", &MjsPair::margin, &MjsPair::set_margin, reference())
    .property("solimp", &MjsPair::solimp)
    .property("solref", &MjsPair::solref)
    .property("solreffriction", &MjsPair::solreffriction);
  emscripten::class_<MjsPlugin>("MjsPlugin")
    .property("active", &MjsPlugin::active, &MjsPlugin::set_active, reference())
    .property("element", &MjsPlugin::element, reference())
    .property("info", &MjsPlugin::info, &MjsPlugin::set_info, reference())
    .property("name", &MjsPlugin::name, &MjsPlugin::set_name, reference())
    .property("plugin_name", &MjsPlugin::plugin_name, &MjsPlugin::set_plugin_name, reference());
  emscripten::class_<MjsSensor>("MjsSensor")
    .property("cutoff", &MjsSensor::cutoff, &MjsSensor::set_cutoff, reference())
    .property("datatype", &MjsSensor::datatype, &MjsSensor::set_datatype, reference())
    .property("dim", &MjsSensor::dim, &MjsSensor::set_dim, reference())
    .property("element", &MjsSensor::element, reference())
    .property("info", &MjsSensor::info, &MjsSensor::set_info, reference())
    .property("intprm", &MjsSensor::intprm)
    .property("needstage", &MjsSensor::needstage, &MjsSensor::set_needstage, reference())
    .property("noise", &MjsSensor::noise, &MjsSensor::set_noise, reference())
    .property("objname", &MjsSensor::objname, &MjsSensor::set_objname, reference())
    .property("objtype", &MjsSensor::objtype, &MjsSensor::set_objtype, reference())
    .property("plugin", &MjsSensor::plugin, reference())
    .property("refname", &MjsSensor::refname, &MjsSensor::set_refname, reference())
    .property("reftype", &MjsSensor::reftype, &MjsSensor::set_reftype, reference())
    .property("type", &MjsSensor::type, &MjsSensor::set_type, reference())
    .property("userdata", &MjsSensor::userdata, reference());
  emscripten::class_<MjsSite>("MjsSite")
    .property("alt", &MjsSite::alt, reference())
    .property("element", &MjsSite::element, reference())
    .property("fromto", &MjsSite::fromto)
    .property("group", &MjsSite::group, &MjsSite::set_group, reference())
    .property("info", &MjsSite::info, &MjsSite::set_info, reference())
    .property("material", &MjsSite::material, &MjsSite::set_material, reference())
    .property("pos", &MjsSite::pos)
    .property("quat", &MjsSite::quat)
    .property("rgba", &MjsSite::rgba)
    .property("size", &MjsSite::size)
    .property("type", &MjsSite::type, &MjsSite::set_type, reference())
    .property("userdata", &MjsSite::userdata, reference());
  emscripten::class_<MjsSkin>("MjsSkin")
    .property("bindpos", &MjsSkin::bindpos, reference())
    .property("bindquat", &MjsSkin::bindquat, reference())
    .property("bodyname", &MjsSkin::bodyname, reference())
    .property("element", &MjsSkin::element, reference())
    .property("face", &MjsSkin::face, reference())
    .property("file", &MjsSkin::file, &MjsSkin::set_file, reference())
    .property("group", &MjsSkin::group, &MjsSkin::set_group, reference())
    .property("inflate", &MjsSkin::inflate, &MjsSkin::set_inflate, reference())
    .property("info", &MjsSkin::info, &MjsSkin::set_info, reference())
    .property("material", &MjsSkin::material, &MjsSkin::set_material, reference())
    .property("rgba", &MjsSkin::rgba)
    .property("texcoord", &MjsSkin::texcoord, reference())
    .property("vert", &MjsSkin::vert, reference())
    .property("vertid", &MjsSkin::vertid, reference())
    .property("vertweight", &MjsSkin::vertweight, reference());
  emscripten::class_<MjsTendon>("MjsTendon")
    .property("actfrclimited", &MjsTendon::actfrclimited, &MjsTendon::set_actfrclimited, reference())
    .property("actfrcrange", &MjsTendon::actfrcrange)
    .property("armature", &MjsTendon::armature, &MjsTendon::set_armature, reference())
    .property("damping", &MjsTendon::damping, &MjsTendon::set_damping, reference())
    .property("element", &MjsTendon::element, reference())
    .property("frictionloss", &MjsTendon::frictionloss, &MjsTendon::set_frictionloss, reference())
    .property("group", &MjsTendon::group, &MjsTendon::set_group, reference())
    .property("info", &MjsTendon::info, &MjsTendon::set_info, reference())
    .property("limited", &MjsTendon::limited, &MjsTendon::set_limited, reference())
    .property("margin", &MjsTendon::margin, &MjsTendon::set_margin, reference())
    .property("material", &MjsTendon::material, &MjsTendon::set_material, reference())
    .property("range", &MjsTendon::range)
    .property("rgba", &MjsTendon::rgba)
    .property("solimp_friction", &MjsTendon::solimp_friction)
    .property("solimp_limit", &MjsTendon::solimp_limit)
    .property("solref_friction", &MjsTendon::solref_friction)
    .property("solref_limit", &MjsTendon::solref_limit)
    .property("springlength", &MjsTendon::springlength)
    .property("stiffness", &MjsTendon::stiffness, &MjsTendon::set_stiffness, reference())
    .property("userdata", &MjsTendon::userdata, reference())
    .property("width", &MjsTendon::width, &MjsTendon::set_width, reference());
  emscripten::class_<MjsText>("MjsText")
    .property("data", &MjsText::data, &MjsText::set_data, reference())
    .property("element", &MjsText::element, reference())
    .property("info", &MjsText::info, &MjsText::set_info, reference());
  emscripten::class_<MjsTexture>("MjsTexture")
    .property("builtin", &MjsTexture::builtin, &MjsTexture::set_builtin, reference())
    .property("colorspace", &MjsTexture::colorspace, &MjsTexture::set_colorspace, reference())
    .property("content_type", &MjsTexture::content_type, &MjsTexture::set_content_type, reference())
    .property("cubefiles", &MjsTexture::cubefiles, reference())
    .property("data", &MjsTexture::data, reference())
    .property("element", &MjsTexture::element, reference())
    .property("file", &MjsTexture::file, &MjsTexture::set_file, reference())
    .property("gridlayout", &MjsTexture::gridlayout)
    .property("gridsize", &MjsTexture::gridsize)
    .property("height", &MjsTexture::height, &MjsTexture::set_height, reference())
    .property("hflip", &MjsTexture::hflip, &MjsTexture::set_hflip, reference())
    .property("info", &MjsTexture::info, &MjsTexture::set_info, reference())
    .property("mark", &MjsTexture::mark, &MjsTexture::set_mark, reference())
    .property("markrgb", &MjsTexture::markrgb)
    .property("nchannel", &MjsTexture::nchannel, &MjsTexture::set_nchannel, reference())
    .property("random", &MjsTexture::random, &MjsTexture::set_random, reference())
    .property("rgb1", &MjsTexture::rgb1)
    .property("rgb2", &MjsTexture::rgb2)
    .property("type", &MjsTexture::type, &MjsTexture::set_type, reference())
    .property("vflip", &MjsTexture::vflip, &MjsTexture::set_vflip, reference())
    .property("width", &MjsTexture::width, &MjsTexture::set_width, reference());
  emscripten::class_<MjsTuple>("MjsTuple")
    .property("element", &MjsTuple::element, reference())
    .property("info", &MjsTuple::info, &MjsTuple::set_info, reference())
    .property("objname", &MjsTuple::objname, reference())
    .property("objprm", &MjsTuple::objprm, reference())
    .property("objtype", &MjsTuple::objtype, reference());
  emscripten::class_<MjsWrap>("MjsWrap")
    .property("element", &MjsWrap::element, reference())
    .property("info", &MjsWrap::info, &MjsWrap::set_info, reference())
    .property("type", &MjsWrap::type, &MjsWrap::set_type, reference());
  emscripten::class_<MjvCamera>("MjvCamera")
    .constructor<>()
    .function("copy", &MjvCamera::copy, take_ownership())
    .property("azimuth", &MjvCamera::azimuth, &MjvCamera::set_azimuth, reference())
    .property("distance", &MjvCamera::distance, &MjvCamera::set_distance, reference())
    .property("elevation", &MjvCamera::elevation, &MjvCamera::set_elevation, reference())
    .property("fixedcamid", &MjvCamera::fixedcamid, &MjvCamera::set_fixedcamid, reference())
    .property("lookat", &MjvCamera::lookat)
    .property("orthographic", &MjvCamera::orthographic, &MjvCamera::set_orthographic, reference())
    .property("trackbodyid", &MjvCamera::trackbodyid, &MjvCamera::set_trackbodyid, reference())
    .property("type", &MjvCamera::type, &MjvCamera::set_type, reference());
  emscripten::class_<MjvFigure>("MjvFigure")
    .constructor<>()
    .function("copy", &MjvFigure::copy, take_ownership())
    .property("figurergba", &MjvFigure::figurergba)
    .property("flg_barplot", &MjvFigure::flg_barplot, &MjvFigure::set_flg_barplot, reference())
    .property("flg_extend", &MjvFigure::flg_extend, &MjvFigure::set_flg_extend, reference())
    .property("flg_legend", &MjvFigure::flg_legend, &MjvFigure::set_flg_legend, reference())
    .property("flg_selection", &MjvFigure::flg_selection, &MjvFigure::set_flg_selection, reference())
    .property("flg_symmetric", &MjvFigure::flg_symmetric, &MjvFigure::set_flg_symmetric, reference())
    .property("flg_ticklabel", &MjvFigure::flg_ticklabel)
    .property("gridrgb", &MjvFigure::gridrgb)
    .property("gridsize", &MjvFigure::gridsize)
    .property("gridwidth", &MjvFigure::gridwidth, &MjvFigure::set_gridwidth, reference())
    .property("highlight", &MjvFigure::highlight)
    .property("highlightid", &MjvFigure::highlightid, &MjvFigure::set_highlightid, reference())
    .property("legendoffset", &MjvFigure::legendoffset, &MjvFigure::set_legendoffset, reference())
    .property("legendrgba", &MjvFigure::legendrgba)
    .property("linedata", &MjvFigure::linedata)
    .property("linename", &MjvFigure::linename)
    .property("linepnt", &MjvFigure::linepnt)
    .property("linergb", &MjvFigure::linergb)
    .property("linewidth", &MjvFigure::linewidth, &MjvFigure::set_linewidth, reference())
    .property("minwidth", &MjvFigure::minwidth)
    .property("panergba", &MjvFigure::panergba)
    .property("range", &MjvFigure::range)
    .property("selection", &MjvFigure::selection, &MjvFigure::set_selection, reference())
    .property("subplot", &MjvFigure::subplot, &MjvFigure::set_subplot, reference())
    .property("textrgb", &MjvFigure::textrgb)
    .property("title", &MjvFigure::title)
    .property("xaxisdata", &MjvFigure::xaxisdata)
    .property("xaxispixel", &MjvFigure::xaxispixel)
    .property("xformat", &MjvFigure::xformat)
    .property("xlabel", &MjvFigure::xlabel)
    .property("yaxisdata", &MjvFigure::yaxisdata)
    .property("yaxispixel", &MjvFigure::yaxispixel)
    .property("yformat", &MjvFigure::yformat);
  emscripten::class_<MjvGLCamera>("MjvGLCamera")
    .constructor<>()
    .function("copy", &MjvGLCamera::copy, take_ownership())
    .property("forward", &MjvGLCamera::forward)
    .property("frustum_bottom", &MjvGLCamera::frustum_bottom, &MjvGLCamera::set_frustum_bottom, reference())
    .property("frustum_center", &MjvGLCamera::frustum_center, &MjvGLCamera::set_frustum_center, reference())
    .property("frustum_far", &MjvGLCamera::frustum_far, &MjvGLCamera::set_frustum_far, reference())
    .property("frustum_near", &MjvGLCamera::frustum_near, &MjvGLCamera::set_frustum_near, reference())
    .property("frustum_top", &MjvGLCamera::frustum_top, &MjvGLCamera::set_frustum_top, reference())
    .property("frustum_width", &MjvGLCamera::frustum_width, &MjvGLCamera::set_frustum_width, reference())
    .property("orthographic", &MjvGLCamera::orthographic, &MjvGLCamera::set_orthographic, reference())
    .property("pos", &MjvGLCamera::pos)
    .property("up", &MjvGLCamera::up);
  emscripten::class_<MjvGeom>("MjvGeom")
    .constructor<>()
    .function("copy", &MjvGeom::copy, take_ownership())
    .property("camdist", &MjvGeom::camdist, &MjvGeom::set_camdist, reference())
    .property("category", &MjvGeom::category, &MjvGeom::set_category, reference())
    .property("dataid", &MjvGeom::dataid, &MjvGeom::set_dataid, reference())
    .property("emission", &MjvGeom::emission, &MjvGeom::set_emission, reference())
    .property("label", &MjvGeom::label)
    .property("mat", &MjvGeom::mat)
    .property("matid", &MjvGeom::matid, &MjvGeom::set_matid, reference())
    .property("modelrbound", &MjvGeom::modelrbound, &MjvGeom::set_modelrbound, reference())
    .property("objid", &MjvGeom::objid, &MjvGeom::set_objid, reference())
    .property("objtype", &MjvGeom::objtype, &MjvGeom::set_objtype, reference())
    .property("pos", &MjvGeom::pos)
    .property("reflectance", &MjvGeom::reflectance, &MjvGeom::set_reflectance, reference())
    .property("rgba", &MjvGeom::rgba)
    .property("segid", &MjvGeom::segid, &MjvGeom::set_segid, reference())
    .property("shininess", &MjvGeom::shininess, &MjvGeom::set_shininess, reference())
    .property("size", &MjvGeom::size)
    .property("specular", &MjvGeom::specular, &MjvGeom::set_specular, reference())
    .property("texcoord", &MjvGeom::texcoord, &MjvGeom::set_texcoord, reference())
    .property("transparent", &MjvGeom::transparent, &MjvGeom::set_transparent, reference())
    .property("type", &MjvGeom::type, &MjvGeom::set_type, reference());
  emscripten::class_<MjvLight>("MjvLight")
    .constructor<>()
    .function("copy", &MjvLight::copy, take_ownership())
    .property("ambient", &MjvLight::ambient)
    .property("attenuation", &MjvLight::attenuation)
    .property("bulbradius", &MjvLight::bulbradius, &MjvLight::set_bulbradius, reference())
    .property("castshadow", &MjvLight::castshadow, &MjvLight::set_castshadow, reference())
    .property("cutoff", &MjvLight::cutoff, &MjvLight::set_cutoff, reference())
    .property("diffuse", &MjvLight::diffuse)
    .property("dir", &MjvLight::dir)
    .property("exponent", &MjvLight::exponent, &MjvLight::set_exponent, reference())
    .property("headlight", &MjvLight::headlight, &MjvLight::set_headlight, reference())
    .property("id", &MjvLight::id, &MjvLight::set_id, reference())
    .property("intensity", &MjvLight::intensity, &MjvLight::set_intensity, reference())
    .property("pos", &MjvLight::pos)
    .property("range", &MjvLight::range, &MjvLight::set_range, reference())
    .property("specular", &MjvLight::specular)
    .property("texid", &MjvLight::texid, &MjvLight::set_texid, reference())
    .property("type", &MjvLight::type, &MjvLight::set_type, reference());
  emscripten::class_<MjvOption>("MjvOption")
    .constructor<>()
    .function("copy", &MjvOption::copy, take_ownership())
    .property("actuatorgroup", &MjvOption::actuatorgroup)
    .property("bvh_depth", &MjvOption::bvh_depth, &MjvOption::set_bvh_depth, reference())
    .property("flags", &MjvOption::flags)
    .property("flex_layer", &MjvOption::flex_layer, &MjvOption::set_flex_layer, reference())
    .property("flexgroup", &MjvOption::flexgroup)
    .property("frame", &MjvOption::frame, &MjvOption::set_frame, reference())
    .property("geomgroup", &MjvOption::geomgroup)
    .property("jointgroup", &MjvOption::jointgroup)
    .property("label", &MjvOption::label, &MjvOption::set_label, reference())
    .property("sitegroup", &MjvOption::sitegroup)
    .property("skingroup", &MjvOption::skingroup)
    .property("tendongroup", &MjvOption::tendongroup);
  emscripten::class_<MjvPerturb>("MjvPerturb")
    .constructor<>()
    .function("copy", &MjvPerturb::copy, take_ownership())
    .property("active", &MjvPerturb::active, &MjvPerturb::set_active, reference())
    .property("active2", &MjvPerturb::active2, &MjvPerturb::set_active2, reference())
    .property("flexselect", &MjvPerturb::flexselect, &MjvPerturb::set_flexselect, reference())
    .property("localmass", &MjvPerturb::localmass, &MjvPerturb::set_localmass, reference())
    .property("localpos", &MjvPerturb::localpos)
    .property("refpos", &MjvPerturb::refpos)
    .property("refquat", &MjvPerturb::refquat)
    .property("refselpos", &MjvPerturb::refselpos)
    .property("scale", &MjvPerturb::scale, &MjvPerturb::set_scale, reference())
    .property("select", &MjvPerturb::select, &MjvPerturb::set_select, reference())
    .property("skinselect", &MjvPerturb::skinselect, &MjvPerturb::set_skinselect, reference());
  emscripten::class_<MjvScene>("MjvScene")
    .constructor<MjModel *, int>()
    .constructor<>()
    .property("camera", &MjvScene::camera)
    .property("enabletransform", &MjvScene::enabletransform, &MjvScene::set_enabletransform, reference())
    .property("flags", &MjvScene::flags)
    .property("flexedge", &MjvScene::flexedge)
    .property("flexedgeadr", &MjvScene::flexedgeadr)
    .property("flexedgenum", &MjvScene::flexedgenum)
    .property("flexedgeopt", &MjvScene::flexedgeopt, &MjvScene::set_flexedgeopt, reference())
    .property("flexface", &MjvScene::flexface)
    .property("flexfaceadr", &MjvScene::flexfaceadr)
    .property("flexfacenum", &MjvScene::flexfacenum)
    .property("flexfaceopt", &MjvScene::flexfaceopt, &MjvScene::set_flexfaceopt, reference())
    .property("flexfaceused", &MjvScene::flexfaceused)
    .property("flexnormal", &MjvScene::flexnormal)
    .property("flexskinopt", &MjvScene::flexskinopt, &MjvScene::set_flexskinopt, reference())
    .property("flextexcoord", &MjvScene::flextexcoord)
    .property("flexvert", &MjvScene::flexvert)
    .property("flexvertadr", &MjvScene::flexvertadr)
    .property("flexvertnum", &MjvScene::flexvertnum)
    .property("flexvertopt", &MjvScene::flexvertopt, &MjvScene::set_flexvertopt, reference())
    .property("framergb", &MjvScene::framergb)
    .property("framewidth", &MjvScene::framewidth, &MjvScene::set_framewidth, reference())
    .property("geomorder", &MjvScene::geomorder)
    .property("geoms", &MjvScene::geoms)
    .property("lights", &MjvScene::lights)
    .property("maxgeom", &MjvScene::maxgeom, &MjvScene::set_maxgeom, reference())
    .property("nflex", &MjvScene::nflex, &MjvScene::set_nflex, reference())
    .property("ngeom", &MjvScene::ngeom, &MjvScene::set_ngeom, reference())
    .property("nlight", &MjvScene::nlight, &MjvScene::set_nlight, reference())
    .property("nskin", &MjvScene::nskin, &MjvScene::set_nskin, reference())
    .property("rotate", &MjvScene::rotate)
    .property("scale", &MjvScene::scale, &MjvScene::set_scale, reference())
    .property("skinfacenum", &MjvScene::skinfacenum)
    .property("skinnormal", &MjvScene::skinnormal)
    .property("skinvert", &MjvScene::skinvert)
    .property("skinvertadr", &MjvScene::skinvertadr)
    .property("skinvertnum", &MjvScene::skinvertnum)
    .property("status", &MjvScene::status, &MjvScene::set_status, reference())
    .property("stereo", &MjvScene::stereo, &MjvScene::set_stereo, reference())
    .property("translate", &MjvScene::translate);
  emscripten::register_optional<MjSpec>();
  emscripten::register_optional<MjsActuator>();
  emscripten::register_optional<MjsBody>();
  emscripten::register_optional<MjsCamera>();
  emscripten::register_optional<MjsCompiler>();
  emscripten::register_optional<MjsDefault>();
  emscripten::register_optional<MjsElement>();
  emscripten::register_optional<MjsEquality>();
  emscripten::register_optional<MjsExclude>();
  emscripten::register_optional<MjsFlex>();
  emscripten::register_optional<MjsFrame>();
  emscripten::register_optional<MjsGeom>();
  emscripten::register_optional<MjsHField>();
  emscripten::register_optional<MjsJoint>();
  emscripten::register_optional<MjsKey>();
  emscripten::register_optional<MjsLight>();
  emscripten::register_optional<MjsMaterial>();
  emscripten::register_optional<MjsMesh>();
  emscripten::register_optional<MjsNumeric>();
  emscripten::register_optional<MjsOrientation>();
  emscripten::register_optional<MjsPair>();
  emscripten::register_optional<MjsPlugin>();
  emscripten::register_optional<MjsSensor>();
  emscripten::register_optional<MjsSite>();
  emscripten::register_optional<MjsSkin>();
  emscripten::register_optional<MjsTendon>();
  emscripten::register_optional<MjsText>();
  emscripten::register_optional<MjsTexture>();
  emscripten::register_optional<MjsTuple>();
  emscripten::register_optional<MjsWrap>();

  function("mj_Euler", &mj_Euler_wrapper);
  function("mj_RungeKutta", &mj_RungeKutta_wrapper);
  function("mj_addContact", &mj_addContact_wrapper);
  function("mj_addM", &mj_addM_wrapper);
  function("mj_angmomMat", &mj_angmomMat_wrapper);
  function("mj_applyFT", &mj_applyFT_wrapper);
  function("mj_camlight", &mj_camlight_wrapper);
  function("mj_checkAcc", &mj_checkAcc_wrapper);
  function("mj_checkPos", &mj_checkPos_wrapper);
  function("mj_checkVel", &mj_checkVel_wrapper);
  function("mj_collision", &mj_collision_wrapper);
  function("mj_comPos", &mj_comPos_wrapper);
  function("mj_comVel", &mj_comVel_wrapper);
  function("mj_compareFwdInv", &mj_compareFwdInv_wrapper);
  function("mj_constraintUpdate", &mj_constraintUpdate_wrapper);
  function("mj_contactForce", &mj_contactForce_wrapper);
  function("mj_copyBack", &mj_copyBack_wrapper);
  function("mj_copyState", &mj_copyState_wrapper);
  function("mj_crb", &mj_crb_wrapper);
  function("mj_defaultLROpt", &mj_defaultLROpt_wrapper);
  function("mj_defaultOption", &mj_defaultOption_wrapper);
  function("mj_defaultSolRefImp", &mj_defaultSolRefImp_wrapper);
  function("mj_defaultVisual", &mj_defaultVisual_wrapper);
  function("mj_differentiatePos", &mj_differentiatePos_wrapper);
  function("mj_energyPos", &mj_energyPos_wrapper);
  function("mj_energyVel", &mj_energyVel_wrapper);
  function("mj_extractState", &mj_extractState_wrapper);
  function("mj_factorM", &mj_factorM_wrapper);
  function("mj_flex", &mj_flex_wrapper);
  function("mj_forward", &mj_forward_wrapper);
  function("mj_forwardSkip", &mj_forwardSkip_wrapper);
  function("mj_fullM", &mj_fullM_wrapper);
  function("mj_fwdAcceleration", &mj_fwdAcceleration_wrapper);
  function("mj_fwdActuation", &mj_fwdActuation_wrapper);
  function("mj_fwdConstraint", &mj_fwdConstraint_wrapper);
  function("mj_fwdKinematics", &mj_fwdKinematics_wrapper);
  function("mj_fwdPosition", &mj_fwdPosition_wrapper);
  function("mj_fwdVelocity", &mj_fwdVelocity_wrapper);
  function("mj_geomDistance", &mj_geomDistance_wrapper);
  function("mj_getState", &mj_getState_wrapper);
  function("mj_getTotalmass", &mj_getTotalmass_wrapper);
  function("mj_id2name", &mj_id2name_wrapper);
  function("mj_implicit", &mj_implicit_wrapper);
  function("mj_integratePos", &mj_integratePos_wrapper);
  function("mj_invConstraint", &mj_invConstraint_wrapper);
  function("mj_invPosition", &mj_invPosition_wrapper);
  function("mj_invVelocity", &mj_invVelocity_wrapper);
  function("mj_inverse", &mj_inverse_wrapper);
  function("mj_inverseSkip", &mj_inverseSkip_wrapper);
  function("mj_isDual", &mj_isDual_wrapper);
  function("mj_isPyramidal", &mj_isPyramidal_wrapper);
  function("mj_isSparse", &mj_isSparse_wrapper);
  function("mj_island", &mj_island_wrapper);
  function("mj_jac", &mj_jac_wrapper);
  function("mj_jacBody", &mj_jacBody_wrapper);
  function("mj_jacBodyCom", &mj_jacBodyCom_wrapper);
  function("mj_jacDot", &mj_jacDot_wrapper);
  function("mj_jacGeom", &mj_jacGeom_wrapper);
  function("mj_jacPointAxis", &mj_jacPointAxis_wrapper);
  function("mj_jacSite", &mj_jacSite_wrapper);
  function("mj_jacSubtreeCom", &mj_jacSubtreeCom_wrapper);
  function("mj_kinematics", &mj_kinematics_wrapper);
  function("mj_local2Global", &mj_local2Global_wrapper);
  function("mj_makeConstraint", &mj_makeConstraint_wrapper);
  function("mj_makeM", &mj_makeM_wrapper);
  function("mj_mulJacTVec", &mj_mulJacTVec_wrapper);
  function("mj_mulJacVec", &mj_mulJacVec_wrapper);
  function("mj_mulM", &mj_mulM_wrapper);
  function("mj_mulM2", &mj_mulM2_wrapper);
  function("mj_multiRay", &mj_multiRay_wrapper);
  function("mj_name2id", &mj_name2id_wrapper);
  function("mj_normalizeQuat", &mj_normalizeQuat_wrapper);
  function("mj_objectAcceleration", &mj_objectAcceleration_wrapper);
  function("mj_objectVelocity", &mj_objectVelocity_wrapper);
  function("mj_passive", &mj_passive_wrapper);
  function("mj_printData", &mj_printData_wrapper);
  function("mj_printFormattedData", &mj_printFormattedData_wrapper);
  function("mj_printFormattedModel", &mj_printFormattedModel_wrapper);
  function("mj_printFormattedScene", &mj_printFormattedScene_wrapper);
  function("mj_printModel", &mj_printModel_wrapper);
  function("mj_printScene", &mj_printScene_wrapper);
  function("mj_projectConstraint", &mj_projectConstraint_wrapper);
  function("mj_ray", &mj_ray_wrapper);
  function("mj_rayHfield", &mj_rayHfield_wrapper);
  function("mj_rayMesh", &mj_rayMesh_wrapper);
  function("mj_referenceConstraint", &mj_referenceConstraint_wrapper);
  function("mj_resetCallbacks", &mj_resetCallbacks);
  function("mj_resetData", &mj_resetData_wrapper);
  function("mj_resetDataDebug", &mj_resetDataDebug_wrapper);
  function("mj_resetDataKeyframe", &mj_resetDataKeyframe_wrapper);
  function("mj_rne", &mj_rne_wrapper);
  function("mj_rnePostConstraint", &mj_rnePostConstraint_wrapper);
  function("mj_saveLastXML", &mj_saveLastXML_wrapper);
  function("mj_sensorAcc", &mj_sensorAcc_wrapper);
  function("mj_sensorPos", &mj_sensorPos_wrapper);
  function("mj_sensorVel", &mj_sensorVel_wrapper);
  function("mj_setConst", &mj_setConst_wrapper);
  function("mj_setKeyframe", &mj_setKeyframe_wrapper);
  function("mj_setLengthRange", &mj_setLengthRange_wrapper);
  function("mj_setState", &mj_setState_wrapper);
  function("mj_setTotalmass", &mj_setTotalmass_wrapper);
  function("mj_sizeModel", &mj_sizeModel_wrapper);
  function("mj_solveM", &mj_solveM_wrapper);
  function("mj_solveM2", &mj_solveM2_wrapper);
  function("mj_stateSize", &mj_stateSize_wrapper);
  function("mj_step", &mj_step_wrapper);
  function("mj_step1", &mj_step1_wrapper);
  function("mj_step2", &mj_step2_wrapper);
  function("mj_subtreeVel", &mj_subtreeVel_wrapper);
  function("mj_tendon", &mj_tendon_wrapper);
  function("mj_transmission", &mj_transmission_wrapper);
  function("mj_version", &mj_version);
  function("mj_versionString", &mj_versionString_wrapper);
  function("mjd_inverseFD", &mjd_inverseFD_wrapper);
  function("mjd_quatIntegrate", &mjd_quatIntegrate_wrapper);
  function("mjd_subQuat", &mjd_subQuat_wrapper);
  function("mjd_transitionFD", &mjd_transitionFD_wrapper);
  function("mjs_activatePlugin", &mjs_activatePlugin_wrapper);
  function("mjs_addActuator", &mjs_addActuator_wrapper);
  function("mjs_addBody", &mjs_addBody_wrapper);
  function("mjs_addCamera", &mjs_addCamera_wrapper);
  function("mjs_addDefault", &mjs_addDefault_wrapper);
  function("mjs_addEquality", &mjs_addEquality_wrapper);
  function("mjs_addExclude", &mjs_addExclude_wrapper);
  function("mjs_addFlex", &mjs_addFlex_wrapper);
  function("mjs_addFrame", &mjs_addFrame_wrapper);
  function("mjs_addFreeJoint", &mjs_addFreeJoint_wrapper);
  function("mjs_addGeom", &mjs_addGeom_wrapper);
  function("mjs_addHField", &mjs_addHField_wrapper);
  function("mjs_addJoint", &mjs_addJoint_wrapper);
  function("mjs_addKey", &mjs_addKey_wrapper);
  function("mjs_addLight", &mjs_addLight_wrapper);
  function("mjs_addMaterial", &mjs_addMaterial_wrapper);
  function("mjs_addMesh", &mjs_addMesh_wrapper);
  function("mjs_addNumeric", &mjs_addNumeric_wrapper);
  function("mjs_addPair", &mjs_addPair_wrapper);
  function("mjs_addPlugin", &mjs_addPlugin_wrapper);
  function("mjs_addSensor", &mjs_addSensor_wrapper);
  function("mjs_addSite", &mjs_addSite_wrapper);
  function("mjs_addSkin", &mjs_addSkin_wrapper);
  function("mjs_addTendon", &mjs_addTendon_wrapper);
  function("mjs_addText", &mjs_addText_wrapper);
  function("mjs_addTexture", &mjs_addTexture_wrapper);
  function("mjs_addTuple", &mjs_addTuple_wrapper);
  function("mjs_asActuator", &mjs_asActuator_wrapper);
  function("mjs_asBody", &mjs_asBody_wrapper);
  function("mjs_asCamera", &mjs_asCamera_wrapper);
  function("mjs_asEquality", &mjs_asEquality_wrapper);
  function("mjs_asExclude", &mjs_asExclude_wrapper);
  function("mjs_asFlex", &mjs_asFlex_wrapper);
  function("mjs_asFrame", &mjs_asFrame_wrapper);
  function("mjs_asGeom", &mjs_asGeom_wrapper);
  function("mjs_asHField", &mjs_asHField_wrapper);
  function("mjs_asJoint", &mjs_asJoint_wrapper);
  function("mjs_asKey", &mjs_asKey_wrapper);
  function("mjs_asLight", &mjs_asLight_wrapper);
  function("mjs_asMaterial", &mjs_asMaterial_wrapper);
  function("mjs_asMesh", &mjs_asMesh_wrapper);
  function("mjs_asNumeric", &mjs_asNumeric_wrapper);
  function("mjs_asPair", &mjs_asPair_wrapper);
  function("mjs_asPlugin", &mjs_asPlugin_wrapper);
  function("mjs_asSensor", &mjs_asSensor_wrapper);
  function("mjs_asSite", &mjs_asSite_wrapper);
  function("mjs_asSkin", &mjs_asSkin_wrapper);
  function("mjs_asTendon", &mjs_asTendon_wrapper);
  function("mjs_asText", &mjs_asText_wrapper);
  function("mjs_asTexture", &mjs_asTexture_wrapper);
  function("mjs_asTuple", &mjs_asTuple_wrapper);
  function("mjs_attach", &mjs_attach_wrapper);
  function("mjs_defaultActuator", &mjs_defaultActuator_wrapper);
  function("mjs_defaultBody", &mjs_defaultBody_wrapper);
  function("mjs_defaultCamera", &mjs_defaultCamera_wrapper);
  function("mjs_defaultEquality", &mjs_defaultEquality_wrapper);
  function("mjs_defaultFlex", &mjs_defaultFlex_wrapper);
  function("mjs_defaultFrame", &mjs_defaultFrame_wrapper);
  function("mjs_defaultGeom", &mjs_defaultGeom_wrapper);
  function("mjs_defaultHField", &mjs_defaultHField_wrapper);
  function("mjs_defaultJoint", &mjs_defaultJoint_wrapper);
  function("mjs_defaultKey", &mjs_defaultKey_wrapper);
  function("mjs_defaultLight", &mjs_defaultLight_wrapper);
  function("mjs_defaultMaterial", &mjs_defaultMaterial_wrapper);
  function("mjs_defaultMesh", &mjs_defaultMesh_wrapper);
  function("mjs_defaultNumeric", &mjs_defaultNumeric_wrapper);
  function("mjs_defaultOrientation", &mjs_defaultOrientation_wrapper);
  function("mjs_defaultPair", &mjs_defaultPair_wrapper);
  function("mjs_defaultPlugin", &mjs_defaultPlugin_wrapper);
  function("mjs_defaultSensor", &mjs_defaultSensor_wrapper);
  function("mjs_defaultSite", &mjs_defaultSite_wrapper);
  function("mjs_defaultSkin", &mjs_defaultSkin_wrapper);
  function("mjs_defaultSpec", &mjs_defaultSpec_wrapper);
  function("mjs_defaultTendon", &mjs_defaultTendon_wrapper);
  function("mjs_defaultText", &mjs_defaultText_wrapper);
  function("mjs_defaultTexture", &mjs_defaultTexture_wrapper);
  function("mjs_defaultTuple", &mjs_defaultTuple_wrapper);
  function("mjs_delete", &mjs_delete_wrapper);
  function("mjs_deleteUserValue", &mjs_deleteUserValue_wrapper);
  function("mjs_findBody", &mjs_findBody_wrapper);
  function("mjs_findChild", &mjs_findChild_wrapper);
  function("mjs_findDefault", &mjs_findDefault_wrapper);
  function("mjs_findElement", &mjs_findElement_wrapper);
  function("mjs_findFrame", &mjs_findFrame_wrapper);
  function("mjs_findSpec", &mjs_findSpec_wrapper);
  function("mjs_firstChild", &mjs_firstChild_wrapper);
  function("mjs_firstElement", &mjs_firstElement_wrapper);
  function("mjs_getDefault", &mjs_getDefault_wrapper);
  function("mjs_getError", &mjs_getError_wrapper);
  function("mjs_getFrame", &mjs_getFrame_wrapper);
  function("mjs_getId", &mjs_getId_wrapper);
  function("mjs_getName", &mjs_getName_wrapper);
  function("mjs_getParent", &mjs_getParent_wrapper);
  function("mjs_getSpec", &mjs_getSpec_wrapper);
  function("mjs_getSpecDefault", &mjs_getSpecDefault_wrapper);
  function("mjs_getWrap", &mjs_getWrap_wrapper);
  function("mjs_getWrapCoef", &mjs_getWrapCoef_wrapper);
  function("mjs_getWrapDivisor", &mjs_getWrapDivisor_wrapper);
  function("mjs_getWrapNum", &mjs_getWrapNum_wrapper);
  function("mjs_getWrapSideSite", &mjs_getWrapSideSite_wrapper);
  function("mjs_getWrapTarget", &mjs_getWrapTarget_wrapper);
  function("mjs_isWarning", &mjs_isWarning_wrapper);
  function("mjs_makeMesh", &mjs_makeMesh_wrapper);
  function("mjs_nextChild", &mjs_nextChild_wrapper);
  function("mjs_nextElement", &mjs_nextElement_wrapper);
  function("mjs_resolveOrientation", &mjs_resolveOrientation_wrapper);
  function("mjs_sensorDim", &mjs_sensorDim_wrapper);
  function("mjs_setDeepCopy", &mjs_setDeepCopy_wrapper);
  function("mjs_setDefault", &mjs_setDefault_wrapper);
  function("mjs_setFrame", &mjs_setFrame_wrapper);
  function("mjs_setName", &mjs_setName_wrapper);
  function("mjs_setToAdhesion", &mjs_setToAdhesion_wrapper);
  function("mjs_setToCylinder", &mjs_setToCylinder_wrapper);
  function("mjs_setToDamper", &mjs_setToDamper_wrapper);
  function("mjs_setToIntVelocity", &mjs_setToIntVelocity_wrapper);
  function("mjs_setToMotor", &mjs_setToMotor_wrapper);
  function("mjs_setToMuscle", &mjs_setToMuscle_wrapper);
  function("mjs_setToPosition", &mjs_setToPosition_wrapper);
  function("mjs_setToVelocity", &mjs_setToVelocity_wrapper);
  function("mjs_wrapGeom", &mjs_wrapGeom_wrapper);
  function("mjs_wrapJoint", &mjs_wrapJoint_wrapper);
  function("mjs_wrapPulley", &mjs_wrapPulley_wrapper);
  function("mjs_wrapSite", &mjs_wrapSite_wrapper);
  function("mju_Halton", &mju_Halton);
  function("mju_L1", &mju_L1_wrapper);
  function("mju_add", &mju_add_wrapper);
  function("mju_add3", &mju_add3_wrapper);
  function("mju_addScl", &mju_addScl_wrapper);
  function("mju_addScl3", &mju_addScl3_wrapper);
  function("mju_addTo", &mju_addTo_wrapper);
  function("mju_addTo3", &mju_addTo3_wrapper);
  function("mju_addToScl", &mju_addToScl_wrapper);
  function("mju_addToScl3", &mju_addToScl3_wrapper);
  function("mju_axisAngle2Quat", &mju_axisAngle2Quat_wrapper);
  function("mju_band2Dense", &mju_band2Dense_wrapper);
  function("mju_bandDiag", &mju_bandDiag);
  function("mju_bandMulMatVec", &mju_bandMulMatVec_wrapper);
  function("mju_boxQP", &mju_boxQP_wrapper);
  function("mju_cholFactor", &mju_cholFactor_wrapper);
  function("mju_cholFactorBand", &mju_cholFactorBand_wrapper);
  function("mju_cholSolve", &mju_cholSolve_wrapper);
  function("mju_cholSolveBand", &mju_cholSolveBand_wrapper);
  function("mju_cholUpdate", &mju_cholUpdate_wrapper);
  function("mju_clip", &mju_clip);
  function("mju_copy", &mju_copy_wrapper);
  function("mju_copy3", &mju_copy3_wrapper);
  function("mju_copy4", &mju_copy4_wrapper);
  function("mju_cross", &mju_cross_wrapper);
  function("mju_d2n", &mju_d2n_wrapper);
  function("mju_decodePyramid", &mju_decodePyramid_wrapper);
  function("mju_dense2Band", &mju_dense2Band_wrapper);
  function("mju_dense2sparse", &mju_dense2sparse_wrapper);
  function("mju_derivQuat", &mju_derivQuat_wrapper);
  function("mju_dist3", &mju_dist3_wrapper);
  function("mju_dot", &mju_dot_wrapper);
  function("mju_dot3", &mju_dot3_wrapper);
  function("mju_eig3", &mju_eig3_wrapper);
  function("mju_encodePyramid", &mju_encodePyramid_wrapper);
  function("mju_euler2Quat", &mju_euler2Quat_wrapper);
  function("mju_eye", &mju_eye_wrapper);
  function("mju_f2n", &mju_f2n_wrapper);
  function("mju_fill", &mju_fill_wrapper);
  function("mju_insertionSort", &mju_insertionSort_wrapper);
  function("mju_insertionSortInt", &mju_insertionSortInt_wrapper);
  function("mju_isBad", &mju_isBad);
  function("mju_isZero", &mju_isZero_wrapper);
  function("mju_mat2Quat", &mju_mat2Quat_wrapper);
  function("mju_mat2Rot", &mju_mat2Rot_wrapper);
  function("mju_max", &mju_max);
  function("mju_min", &mju_min);
  function("mju_mulMatMat", &mju_mulMatMat_wrapper);
  function("mju_mulMatMatT", &mju_mulMatMatT_wrapper);
  function("mju_mulMatTMat", &mju_mulMatTMat_wrapper);
  function("mju_mulMatTVec", &mju_mulMatTVec_wrapper);
  function("mju_mulMatTVec3", &mju_mulMatTVec3_wrapper);
  function("mju_mulMatVec", &mju_mulMatVec_wrapper);
  function("mju_mulMatVec3", &mju_mulMatVec3_wrapper);
  function("mju_mulPose", &mju_mulPose_wrapper);
  function("mju_mulQuat", &mju_mulQuat_wrapper);
  function("mju_mulQuatAxis", &mju_mulQuatAxis_wrapper);
  function("mju_mulVecMatVec", &mju_mulVecMatVec_wrapper);
  function("mju_muscleBias", &mju_muscleBias_wrapper);
  function("mju_muscleDynamics", &mju_muscleDynamics_wrapper);
  function("mju_muscleGain", &mju_muscleGain_wrapper);
  function("mju_n2d", &mju_n2d_wrapper);
  function("mju_n2f", &mju_n2f_wrapper);
  function("mju_negPose", &mju_negPose_wrapper);
  function("mju_negQuat", &mju_negQuat_wrapper);
  function("mju_norm", &mju_norm_wrapper);
  function("mju_norm3", &mju_norm3_wrapper);
  function("mju_normalize", &mju_normalize_wrapper);
  function("mju_normalize3", &mju_normalize3_wrapper);
  function("mju_normalize4", &mju_normalize4_wrapper);
  function("mju_printMat", &mju_printMat_wrapper);
  function("mju_printMatSparse", &mju_printMatSparse_wrapper);
  function("mju_quat2Mat", &mju_quat2Mat_wrapper);
  function("mju_quat2Vel", &mju_quat2Vel_wrapper);
  function("mju_quatIntegrate", &mju_quatIntegrate_wrapper);
  function("mju_quatZ2Vec", &mju_quatZ2Vec_wrapper);
  function("mju_rayFlex", &mju_rayFlex_wrapper);
  function("mju_rayGeom", &mju_rayGeom_wrapper);
  function("mju_raySkin", &mju_raySkin_wrapper);
  function("mju_rotVecQuat", &mju_rotVecQuat_wrapper);
  function("mju_round", &mju_round);
  function("mju_scl", &mju_scl_wrapper);
  function("mju_scl3", &mju_scl3_wrapper);
  function("mju_sigmoid", &mju_sigmoid);
  function("mju_sign", &mju_sign);
  function("mju_sparse2dense", &mju_sparse2dense_wrapper);
  function("mju_springDamper", &mju_springDamper);
  function("mju_sqrMatTD", &mju_sqrMatTD_wrapper);
  function("mju_standardNormal", &mju_standardNormal_wrapper);
  function("mju_str2Type", &mju_str2Type_wrapper);
  function("mju_sub", &mju_sub_wrapper);
  function("mju_sub3", &mju_sub3_wrapper);
  function("mju_subFrom", &mju_subFrom_wrapper);
  function("mju_subFrom3", &mju_subFrom3_wrapper);
  function("mju_subQuat", &mju_subQuat_wrapper);
  function("mju_sum", &mju_sum_wrapper);
  function("mju_symmetrize", &mju_symmetrize_wrapper);
  function("mju_transformSpatial", &mju_transformSpatial_wrapper);
  function("mju_transpose", &mju_transpose_wrapper);
  function("mju_trnVecPose", &mju_trnVecPose_wrapper);
  function("mju_type2Str", &mju_type2Str_wrapper);
  function("mju_unit4", &mju_unit4_wrapper);
  function("mju_warningText", &mju_warningText_wrapper);
  function("mju_writeLog", &mju_writeLog_wrapper);
  function("mju_writeNumBytes", &mju_writeNumBytes_wrapper);
  function("mju_zero", &mju_zero_wrapper);
  function("mju_zero3", &mju_zero3_wrapper);
  function("mju_zero4", &mju_zero4_wrapper);
  function("mjv_addGeoms", &mjv_addGeoms_wrapper);
  function("mjv_alignToCamera", &mjv_alignToCamera_wrapper);
  function("mjv_applyPerturbForce", &mjv_applyPerturbForce_wrapper);
  function("mjv_applyPerturbPose", &mjv_applyPerturbPose_wrapper);
  function("mjv_cameraFrame", &mjv_cameraFrame_wrapper);
  function("mjv_cameraFrustum", &mjv_cameraFrustum_wrapper);
  function("mjv_cameraInModel", &mjv_cameraInModel_wrapper);
  function("mjv_cameraInRoom", &mjv_cameraInRoom_wrapper);
  function("mjv_connector", &mjv_connector_wrapper);
  function("mjv_defaultCamera", &mjv_defaultCamera_wrapper);
  function("mjv_defaultFigure", &mjv_defaultFigure_wrapper);
  function("mjv_defaultFreeCamera", &mjv_defaultFreeCamera_wrapper);
  function("mjv_defaultOption", &mjv_defaultOption_wrapper);
  function("mjv_defaultPerturb", &mjv_defaultPerturb_wrapper);
  function("mjv_frustumHeight", &mjv_frustumHeight_wrapper);
  function("mjv_initGeom", &mjv_initGeom_wrapper);
  function("mjv_initPerturb", &mjv_initPerturb_wrapper);
  function("mjv_makeLights", &mjv_makeLights_wrapper);
  function("mjv_model2room", &mjv_model2room_wrapper);
  function("mjv_moveCamera", &mjv_moveCamera_wrapper);
  function("mjv_moveModel", &mjv_moveModel_wrapper);
  function("mjv_movePerturb", &mjv_movePerturb_wrapper);
  function("mjv_room2model", &mjv_room2model_wrapper);
  function("mjv_select", &mjv_select_wrapper);
  function("mjv_updateCamera", &mjv_updateCamera_wrapper);
  function("mjv_updateScene", &mjv_updateScene_wrapper);
  function("mjv_updateSkin", &mjv_updateSkin_wrapper);

  function("parseXMLString", &parseXMLString_wrapper, take_ownership());
  function("error", &error_wrapper);

  emscripten::class_<WasmBuffer<float>>("FloatBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<float>::FromArray)
      .function("GetPointer", &WasmBuffer<float>::GetPointer)
      .function("GetElementCount", &WasmBuffer<float>::GetElementCount)
      .function("GetView", &WasmBuffer<float>::GetView);

  emscripten::class_<WasmBuffer<double>>("DoubleBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<double>::FromArray)
      .function("GetPointer", &WasmBuffer<double>::GetPointer)
      .function("GetElementCount", &WasmBuffer<double>::GetElementCount)
      .function("GetView", &WasmBuffer<double>::GetView);

  emscripten::class_<WasmBuffer<int>>("IntBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<int>::FromArray)
      .function("GetPointer", &WasmBuffer<int>::GetPointer)
      .function("GetElementCount", &WasmBuffer<int>::GetElementCount)
      .function("GetView", &WasmBuffer<int>::GetView);

  emscripten::register_vector<std::string>("mjStringVec");
  emscripten::register_vector<int>("mjIntVec");
  emscripten::register_vector<mjIntVec>("mjIntVecVec");
  emscripten::register_vector<float>("mjFloatVec");
  emscripten::register_vector<mjFloatVec>("mjFloatVecVec");
  emscripten::register_vector<double>("mjDoubleVec");
  emscripten::register_vector<uint8_t>("mjByteVec");
  emscripten::register_vector<MjSolverStat>("MjSolverStatVec");
  emscripten::register_vector<MjTimerStat>("MjTimerStatVec");
  emscripten::register_vector<MjWarningStat>("MjWarningStatVec");
  emscripten::register_vector<MjContact>("MjContactVec");
  emscripten::register_vector<MjvLight>("MjvLightVec");
  emscripten::register_vector<MjvGLCamera>("MjvGLCameraVec");
  emscripten::register_vector<MjvGeom>("MjvGeomVec");

  // register_type() improves type information (val is mapped to any by default)
  emscripten::register_type<NumberArray>("number[]");
  emscripten::register_type<String>("string");

  emscripten::constant("mjMAXCONPAIR", mjMAXCONPAIR);
  emscripten::constant("mjMAXIMP", mjMAXIMP);
  emscripten::constant("mjMAXLIGHT", mjMAXLIGHT);
  emscripten::constant("mjMAXLINE", mjMAXLINE);
  emscripten::constant("mjMAXLINEPNT", mjMAXLINEPNT);
  emscripten::constant("mjMAXOVERLAY", mjMAXOVERLAY);
  emscripten::constant("mjMAXPLANEGRID", mjMAXPLANEGRID);
  emscripten::constant("mjMAXVAL", mjMAXVAL);
  emscripten::constant("mjMINIMP", mjMINIMP);
  emscripten::constant("mjMINMU", mjMINMU);
  emscripten::constant("mjMINVAL", mjMINVAL);
  emscripten::constant("mjNBIAS", mjNBIAS);
  emscripten::constant("mjNDYN", mjNDYN);
  emscripten::constant("mjNEQDATA", mjNEQDATA);
  emscripten::constant("mjNGAIN", mjNGAIN);
  emscripten::constant("mjNGROUP", mjNGROUP);
  emscripten::constant("mjNIMP", mjNIMP);
  emscripten::constant("mjNREF", mjNREF);
  emscripten::constant("mjNSOLVER", mjNSOLVER);
  emscripten::constant("mjPI", mjPI);
  emscripten::constant("mjVERSION_HEADER", mjVERSION_HEADER);

  // These complex constants are bound using function() rather than constant()
  emscripten::function("get_mjDISABLESTRING", &get_mjDISABLESTRING);
  emscripten::function("get_mjENABLESTRING", &get_mjENABLESTRING);
  emscripten::function("get_mjFRAMESTRING", &get_mjFRAMESTRING);
  emscripten::function("get_mjLABELSTRING", &get_mjLABELSTRING);
  emscripten::function("get_mjRNDSTRING", &get_mjRNDSTRING);
  emscripten::function("get_mjTIMERSTRING", &get_mjTIMERSTRING);
  emscripten::function("get_mjVISSTRING", &get_mjVISSTRING);
}

}  // namespace mujoco::wasm
// NOLINTEND(whitespace/semicolon)
// NOLINTEND(whitespace/line_length)
