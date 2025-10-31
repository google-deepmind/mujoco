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
#ifndef MUJOCO_WASM_CODEGEN_GENERATED_BINDINGS_H_
#define MUJOCO_WASM_CODEGEN_GENERATED_BINDINGS_H_
#include <emscripten.h>
#include <emscripten/bind.h>

#include <cstddef>
#include <cstdint>
#include <memory>

#include <mujoco/mujoco.h>

namespace mujoco::wasm {

// Create the types for anonymous structs
using mjVisualGlobal = decltype(::mjVisual::global);
using mjVisualQuality = decltype(::mjVisual::quality);
using mjVisualHeadlight = decltype(::mjVisual::headlight);
using mjVisualMap = decltype(::mjVisual::map);
using mjVisualScale = decltype(::mjVisual::scale);
using mjVisualRgba = decltype(::mjVisual::rgba);

struct MjContact {
  MjContact();
  MjContact(const MjContact &);
  MjContact &operator=(const MjContact &);
  explicit MjContact(mjContact *ptr);
  ~MjContact();
  std::unique_ptr<MjContact> copy();
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
  mjContact* get() const { return ptr_; }
  void set(mjContact* ptr) { ptr_ = ptr; }

 private:
  mjContact* ptr_;
  bool owned_ = false;
};

struct MjLROpt {
  MjLROpt();
  MjLROpt(const MjLROpt &);
  MjLROpt &operator=(const MjLROpt &);
  explicit MjLROpt(mjLROpt *ptr);
  ~MjLROpt();
  std::unique_ptr<MjLROpt> copy();
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
  mjLROpt* get() const { return ptr_; }
  void set(mjLROpt* ptr) { ptr_ = ptr; }

 private:
  mjLROpt* ptr_;
  bool owned_ = false;
};

struct MjOption {
  MjOption();
  MjOption(const MjOption &);
  MjOption &operator=(const MjOption &);
  explicit MjOption(mjOption *ptr);
  ~MjOption();
  std::unique_ptr<MjOption> copy();
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
  mjOption* get() const { return ptr_; }
  void set(mjOption* ptr) { ptr_ = ptr; }

 private:
  mjOption* ptr_;
  bool owned_ = false;
};

struct MjSolverStat {
  MjSolverStat();
  MjSolverStat(const MjSolverStat &);
  MjSolverStat &operator=(const MjSolverStat &);
  explicit MjSolverStat(mjSolverStat *ptr);
  ~MjSolverStat();
  std::unique_ptr<MjSolverStat> copy();
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
  mjSolverStat* get() const { return ptr_; }
  void set(mjSolverStat* ptr) { ptr_ = ptr; }

 private:
  mjSolverStat* ptr_;
  bool owned_ = false;
};

struct MjStatistic {
  MjStatistic();
  MjStatistic(const MjStatistic &);
  MjStatistic &operator=(const MjStatistic &);
  explicit MjStatistic(mjStatistic *ptr);
  ~MjStatistic();
  std::unique_ptr<MjStatistic> copy();
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
  mjStatistic* get() const { return ptr_; }
  void set(mjStatistic* ptr) { ptr_ = ptr; }

 private:
  mjStatistic* ptr_;
  bool owned_ = false;
};

struct MjTimerStat {
  MjTimerStat();
  MjTimerStat(const MjTimerStat &);
  MjTimerStat &operator=(const MjTimerStat &);
  explicit MjTimerStat(mjTimerStat *ptr);
  ~MjTimerStat();
  std::unique_ptr<MjTimerStat> copy();
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
  mjTimerStat* get() const { return ptr_; }
  void set(mjTimerStat* ptr) { ptr_ = ptr; }

 private:
  mjTimerStat* ptr_;
  bool owned_ = false;
};

struct MjVFS {
  MjVFS();
  MjVFS(const MjVFS &);
  MjVFS &operator=(const MjVFS &);
  explicit MjVFS(mjVFS *ptr);
  ~MjVFS();
  // TODO: Define primitive pointer field with complex extents manually for impl_
  mjVFS* get() const { return ptr_; }
  void set(mjVFS* ptr) { ptr_ = ptr; }

 private:
  mjVFS* ptr_;
  bool owned_ = false;
};

struct MjWarningStat {
  MjWarningStat();
  MjWarningStat(const MjWarningStat &);
  MjWarningStat &operator=(const MjWarningStat &);
  explicit MjWarningStat(mjWarningStat *ptr);
  ~MjWarningStat();
  std::unique_ptr<MjWarningStat> copy();
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
  mjWarningStat* get() const { return ptr_; }
  void set(mjWarningStat* ptr) { ptr_ = ptr; }

 private:
  mjWarningStat* ptr_;
  bool owned_ = false;
};

struct MjsElement {
  explicit MjsElement(mjsElement *ptr);
  ~MjsElement();
  std::unique_ptr<MjsElement> copy();
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
  mjsElement* get() const { return ptr_; }
  void set(mjsElement* ptr) { ptr_ = ptr; }

 private:
  mjsElement* ptr_;
  bool owned_ = false;
};

struct MjsOrientation {
  explicit MjsOrientation(mjsOrientation *ptr);
  ~MjsOrientation();
  std::unique_ptr<MjsOrientation> copy();
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
  mjsOrientation* get() const { return ptr_; }
  void set(mjsOrientation* ptr) { ptr_ = ptr; }

 private:
  mjsOrientation* ptr_;
  bool owned_ = false;
};

struct MjvCamera {
  MjvCamera();
  MjvCamera(const MjvCamera &);
  MjvCamera &operator=(const MjvCamera &);
  explicit MjvCamera(mjvCamera *ptr);
  ~MjvCamera();
  std::unique_ptr<MjvCamera> copy();
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
  mjvCamera* get() const { return ptr_; }
  void set(mjvCamera* ptr) { ptr_ = ptr; }

 private:
  mjvCamera* ptr_;
  bool owned_ = false;
};

struct MjvFigure {
  MjvFigure();
  MjvFigure(const MjvFigure &);
  MjvFigure &operator=(const MjvFigure &);
  explicit MjvFigure(mjvFigure *ptr);
  ~MjvFigure();
  std::unique_ptr<MjvFigure> copy();
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
  mjvFigure* get() const { return ptr_; }
  void set(mjvFigure* ptr) { ptr_ = ptr; }

 private:
  mjvFigure* ptr_;
  bool owned_ = false;
};

struct MjvGLCamera {
  MjvGLCamera();
  MjvGLCamera(const MjvGLCamera &);
  MjvGLCamera &operator=(const MjvGLCamera &);
  explicit MjvGLCamera(mjvGLCamera *ptr);
  ~MjvGLCamera();
  std::unique_ptr<MjvGLCamera> copy();
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
  mjvGLCamera* get() const { return ptr_; }
  void set(mjvGLCamera* ptr) { ptr_ = ptr; }

 private:
  mjvGLCamera* ptr_;
  bool owned_ = false;
};

struct MjvGeom {
  MjvGeom();
  MjvGeom(const MjvGeom &);
  MjvGeom &operator=(const MjvGeom &);
  explicit MjvGeom(mjvGeom *ptr);
  ~MjvGeom();
  std::unique_ptr<MjvGeom> copy();
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
  mjvGeom* get() const { return ptr_; }
  void set(mjvGeom* ptr) { ptr_ = ptr; }

 private:
  mjvGeom* ptr_;
  bool owned_ = false;
};

struct MjvLight {
  MjvLight();
  MjvLight(const MjvLight &);
  MjvLight &operator=(const MjvLight &);
  explicit MjvLight(mjvLight *ptr);
  ~MjvLight();
  std::unique_ptr<MjvLight> copy();
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
  mjvLight* get() const { return ptr_; }
  void set(mjvLight* ptr) { ptr_ = ptr; }

 private:
  mjvLight* ptr_;
  bool owned_ = false;
};

struct MjvOption {
  MjvOption();
  MjvOption(const MjvOption &);
  MjvOption &operator=(const MjvOption &);
  explicit MjvOption(mjvOption *ptr);
  ~MjvOption();
  std::unique_ptr<MjvOption> copy();
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
  mjvOption* get() const { return ptr_; }
  void set(mjvOption* ptr) { ptr_ = ptr; }

 private:
  mjvOption* ptr_;
  bool owned_ = false;
};

struct MjvPerturb {
  MjvPerturb();
  MjvPerturb(const MjvPerturb &);
  MjvPerturb &operator=(const MjvPerturb &);
  explicit MjvPerturb(mjvPerturb *ptr);
  ~MjvPerturb();
  std::unique_ptr<MjvPerturb> copy();
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
  mjvPerturb* get() const { return ptr_; }
  void set(mjvPerturb* ptr) { ptr_ = ptr; }

 private:
  mjvPerturb* ptr_;
  bool owned_ = false;
};

struct MjsCompiler {
  explicit MjsCompiler(mjsCompiler *ptr);
  ~MjsCompiler();
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
  mjsCompiler* get() const { return ptr_; }
  void set(mjsCompiler* ptr) { ptr_ = ptr; }

 private:
  mjsCompiler* ptr_;
  bool owned_ = false;

 public:
  MjLROpt LRopt;
};

struct MjsEquality {
  explicit MjsEquality(mjsEquality *ptr);
  ~MjsEquality();
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
  mjsEquality* get() const { return ptr_; }
  void set(mjsEquality* ptr) { ptr_ = ptr; }

 private:
  mjsEquality* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsExclude {
  explicit MjsExclude(mjsExclude *ptr);
  ~MjsExclude();
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
  mjsExclude* get() const { return ptr_; }
  void set(mjsExclude* ptr) { ptr_ = ptr; }

 private:
  mjsExclude* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsFlex {
  explicit MjsFlex(mjsFlex *ptr);
  ~MjsFlex();
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
  mjsFlex* get() const { return ptr_; }
  void set(mjsFlex* ptr) { ptr_ = ptr; }

 private:
  mjsFlex* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsHField {
  explicit MjsHField(mjsHField *ptr);
  ~MjsHField();
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
  mjsHField* get() const { return ptr_; }
  void set(mjsHField* ptr) { ptr_ = ptr; }

 private:
  mjsHField* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsJoint {
  explicit MjsJoint(mjsJoint *ptr);
  ~MjsJoint();
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
  mjsJoint* get() const { return ptr_; }
  void set(mjsJoint* ptr) { ptr_ = ptr; }

 private:
  mjsJoint* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsKey {
  explicit MjsKey(mjsKey *ptr);
  ~MjsKey();
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
  mjsKey* get() const { return ptr_; }
  void set(mjsKey* ptr) { ptr_ = ptr; }

 private:
  mjsKey* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsLight {
  explicit MjsLight(mjsLight *ptr);
  ~MjsLight();
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
  mjsLight* get() const { return ptr_; }
  void set(mjsLight* ptr) { ptr_ = ptr; }

 private:
  mjsLight* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsMaterial {
  explicit MjsMaterial(mjsMaterial *ptr);
  ~MjsMaterial();
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
  mjsMaterial* get() const { return ptr_; }
  void set(mjsMaterial* ptr) { ptr_ = ptr; }

 private:
  mjsMaterial* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsNumeric {
  explicit MjsNumeric(mjsNumeric *ptr);
  ~MjsNumeric();
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
  mjsNumeric* get() const { return ptr_; }
  void set(mjsNumeric* ptr) { ptr_ = ptr; }

 private:
  mjsNumeric* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsPair {
  explicit MjsPair(mjsPair *ptr);
  ~MjsPair();
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
  mjsPair* get() const { return ptr_; }
  void set(mjsPair* ptr) { ptr_ = ptr; }

 private:
  mjsPair* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsPlugin {
  explicit MjsPlugin(mjsPlugin *ptr);
  ~MjsPlugin();
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
  mjsPlugin* get() const { return ptr_; }
  void set(mjsPlugin* ptr) { ptr_ = ptr; }

 private:
  mjsPlugin* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsSkin {
  explicit MjsSkin(mjsSkin *ptr);
  ~MjsSkin();
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
  mjsSkin* get() const { return ptr_; }
  void set(mjsSkin* ptr) { ptr_ = ptr; }

 private:
  mjsSkin* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsTendon {
  explicit MjsTendon(mjsTendon *ptr);
  ~MjsTendon();
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
  mjsTendon* get() const { return ptr_; }
  void set(mjsTendon* ptr) { ptr_ = ptr; }

 private:
  mjsTendon* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsText {
  explicit MjsText(mjsText *ptr);
  ~MjsText();
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
  mjsText* get() const { return ptr_; }
  void set(mjsText* ptr) { ptr_ = ptr; }

 private:
  mjsText* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsTexture {
  explicit MjsTexture(mjsTexture *ptr);
  ~MjsTexture();
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
    return emscripten::val(emscripten::typed_memory_view(13, ptr_->gridlayout));
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
  mjsTexture* get() const { return ptr_; }
  void set(mjsTexture* ptr) { ptr_ = ptr; }

 private:
  mjsTexture* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsTuple {
  explicit MjsTuple(mjsTuple *ptr);
  ~MjsTuple();
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
  mjsTuple* get() const { return ptr_; }
  void set(mjsTuple* ptr) { ptr_ = ptr; }

 private:
  mjsTuple* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsWrap {
  explicit MjsWrap(mjsWrap *ptr);
  ~MjsWrap();
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
  mjsWrap* get() const { return ptr_; }
  void set(mjsWrap* ptr) { ptr_ = ptr; }

 private:
  mjsWrap* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
};

struct MjsCamera {
  explicit MjsCamera(mjsCamera *ptr);
  ~MjsCamera();
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
  mjsCamera* get() const { return ptr_; }
  void set(mjsCamera* ptr) { ptr_ = ptr; }

 private:
  mjsCamera* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
  MjsOrientation alt;
};

struct MjsFrame {
  explicit MjsFrame(mjsFrame *ptr);
  ~MjsFrame();
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
  mjsFrame* get() const { return ptr_; }
  void set(mjsFrame* ptr) { ptr_ = ptr; }

 private:
  mjsFrame* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
  MjsOrientation alt;
};

struct MjsSite {
  explicit MjsSite(mjsSite *ptr);
  ~MjsSite();
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
  mjsSite* get() const { return ptr_; }
  void set(mjsSite* ptr) { ptr_ = ptr; }

 private:
  mjsSite* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
  MjsOrientation alt;
};

struct MjsActuator {
  explicit MjsActuator(mjsActuator *ptr);
  ~MjsActuator();
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
  mjsActuator* get() const { return ptr_; }
  void set(mjsActuator* ptr) { ptr_ = ptr; }

 private:
  mjsActuator* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
  MjsPlugin plugin;
};

struct MjsBody {
  explicit MjsBody(mjsBody *ptr);
  ~MjsBody();
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
  mjsBody* get() const { return ptr_; }
  void set(mjsBody* ptr) { ptr_ = ptr; }

 private:
  mjsBody* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
  MjsOrientation alt;
  MjsOrientation ialt;
  MjsPlugin plugin;
};

struct MjsGeom {
  explicit MjsGeom(mjsGeom *ptr);
  ~MjsGeom();
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
  mjsGeom* get() const { return ptr_; }
  void set(mjsGeom* ptr) { ptr_ = ptr; }

 private:
  mjsGeom* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
  MjsOrientation alt;
  MjsPlugin plugin;
};

struct MjsMesh {
  explicit MjsMesh(mjsMesh *ptr);
  ~MjsMesh();
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
  mjsMesh* get() const { return ptr_; }
  void set(mjsMesh* ptr) { ptr_ = ptr; }

 private:
  mjsMesh* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
  MjsPlugin plugin;
};

struct MjsSensor {
  explicit MjsSensor(mjsSensor *ptr);
  ~MjsSensor();
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
  mjsSensor* get() const { return ptr_; }
  void set(mjsSensor* ptr) { ptr_ = ptr; }

 private:
  mjsSensor* ptr_;
  bool owned_ = false;

 public:
  MjsElement element;
  MjsPlugin plugin;
};

struct MjsDefault {
  explicit MjsDefault(mjsDefault *ptr);
  ~MjsDefault();
  mjsDefault* get() const { return ptr_; }
  void set(mjsDefault* ptr) { ptr_ = ptr; }

 private:
  mjsDefault* ptr_;
  bool owned_ = false;

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

struct MjVisualGlobal {
  MjVisualGlobal();
  explicit MjVisualGlobal(mjVisualGlobal *ptr);
  MjVisualGlobal(const MjVisualGlobal &);
  MjVisualGlobal &operator=(const MjVisualGlobal &);
  ~MjVisualGlobal();
  std::unique_ptr<MjVisualGlobal> copy();
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
  mjVisualGlobal* get() const { return ptr_; }
  void set(mjVisualGlobal* ptr) { ptr_ = ptr; }

 private:
  mjVisualGlobal* ptr_;
  bool owned_ = false;
};

struct MjVisualQuality {
  MjVisualQuality();
  explicit MjVisualQuality(mjVisualQuality *ptr);
  MjVisualQuality(const MjVisualQuality &);
  MjVisualQuality &operator=(const MjVisualQuality &);
  ~MjVisualQuality();
  std::unique_ptr<MjVisualQuality> copy();
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
  mjVisualQuality* get() const { return ptr_; }
  void set(mjVisualQuality* ptr) { ptr_ = ptr; }

 private:
  mjVisualQuality* ptr_;
  bool owned_ = false;
};

struct MjVisualHeadlight {
  MjVisualHeadlight();
  explicit MjVisualHeadlight(mjVisualHeadlight *ptr);
  MjVisualHeadlight(const MjVisualHeadlight &);
  MjVisualHeadlight &operator=(const MjVisualHeadlight &);
  ~MjVisualHeadlight();
  std::unique_ptr<MjVisualHeadlight> copy();
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
  mjVisualHeadlight* get() const { return ptr_; }
  void set(mjVisualHeadlight* ptr) { ptr_ = ptr; }

 private:
  mjVisualHeadlight* ptr_;
  bool owned_ = false;
};

struct MjVisualMap {
  MjVisualMap();
  explicit MjVisualMap(mjVisualMap *ptr);
  MjVisualMap(const MjVisualMap &);
  MjVisualMap &operator=(const MjVisualMap &);
  ~MjVisualMap();
  std::unique_ptr<MjVisualMap> copy();
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
  mjVisualMap* get() const { return ptr_; }
  void set(mjVisualMap* ptr) { ptr_ = ptr; }

 private:
  mjVisualMap* ptr_;
  bool owned_ = false;
};

struct MjVisualScale {
  MjVisualScale();
  explicit MjVisualScale(mjVisualScale *ptr);
  MjVisualScale(const MjVisualScale &);
  MjVisualScale &operator=(const MjVisualScale &);
  ~MjVisualScale();
  std::unique_ptr<MjVisualScale> copy();
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
  mjVisualScale* get() const { return ptr_; }
  void set(mjVisualScale* ptr) { ptr_ = ptr; }

 private:
  mjVisualScale* ptr_;
  bool owned_ = false;
};

struct MjVisualRgba {
  MjVisualRgba();
  explicit MjVisualRgba(mjVisualRgba *ptr);
  MjVisualRgba(const MjVisualRgba &);
  MjVisualRgba &operator=(const MjVisualRgba &);
  ~MjVisualRgba();
  std::unique_ptr<MjVisualRgba> copy();
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
  mjVisualRgba* get() const { return ptr_; }
  void set(mjVisualRgba* ptr) { ptr_ = ptr; }

 private:
  mjVisualRgba* ptr_;
  bool owned_ = false;
};

struct MjVisual {
  MjVisual();
  explicit MjVisual(mjVisual *ptr_);
  MjVisual(const MjVisual &);
  MjVisual &operator=(const MjVisual &);
  ~MjVisual();
  std::unique_ptr<MjVisual> copy();
  mjVisual* get() const { return ptr_; }
  void set(mjVisual* ptr) { ptr_ = ptr; }

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

struct MjModel {
  explicit MjModel(mjModel *m);
  explicit MjModel(const MjModel &other);
  ~MjModel();
  std::unique_ptr<MjModel> copy();
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
  mjModel* get() const { return ptr_; }
  void set(mjModel* ptr) { ptr_ = ptr; }

 private:
  mjModel* ptr_;

 public:
  MjOption opt;
  MjStatistic stat;
  MjVisual vis;
};

struct MjData {
  MjData(MjModel *m);
  explicit MjData(const MjModel &, const MjData &);
  ~MjData();
  std::vector<MjSolverStat> InitSolverArray();
  std::vector<MjTimerStat> InitTimerArray();
  std::vector<MjWarningStat> InitWarningArray();
  std::vector<MjContact> contact() const;
  std::unique_ptr<MjData> copy();
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
  // array field is defined manually. solver
  emscripten::val solver_niter() const {
    return emscripten::val(emscripten::typed_memory_view(20, ptr_->solver_niter));
  }
  emscripten::val solver_nnz() const {
    return emscripten::val(emscripten::typed_memory_view(20, ptr_->solver_nnz));
  }
  emscripten::val solver_fwdinv() const {
    return emscripten::val(emscripten::typed_memory_view(2, ptr_->solver_fwdinv));
  }
  // array field is defined manually. warning
  // array field is defined manually. timer
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
  // complex pointer field is defined manually. contact
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
  mjData* get() const { return ptr_; }
  void set(mjData* ptr) { ptr_ = ptr; }

 private:
  mjData* ptr_;

 public:
  mjModel *model;
  std::vector<MjSolverStat> solver;
  std::vector<MjTimerStat> timer;
  std::vector<MjWarningStat> warning;
};

struct MjvScene {
  MjvScene();
  MjvScene(MjModel *m, int maxgeom);
  // MjvScene(const MjvScene &);
  ~MjvScene();
  std::unique_ptr<MjvScene> copy();
  int GetSumFlexFaces() const;
  std::vector<MjvLight> InitLightsArray();
  std::vector<MjvGLCamera> InitCameraArray();

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
  int maxgeom() const {
    return ptr_->maxgeom;
  }
  void set_maxgeom(int value) {
    ptr_->maxgeom = value;
  }
  int ngeom() const {
    return ptr_->ngeom;
  }
  void set_ngeom(int value) {
    ptr_->ngeom = value;
  }
  // complex pointer field is defined manually. geoms
  // primitive pointer field with complex extents is defined manually. geomorder
  int nflex() const {
    return ptr_->nflex;
  }
  void set_nflex(int value) {
    ptr_->nflex = value;
  }
  // primitive pointer field with complex extents is defined manually. flexedgeadr
  // primitive pointer field with complex extents is defined manually. flexedgenum
  // primitive pointer field with complex extents is defined manually. flexvertadr
  // primitive pointer field with complex extents is defined manually. flexvertnum
  // primitive pointer field with complex extents is defined manually. flexfaceadr
  // primitive pointer field with complex extents is defined manually. flexfacenum
  // primitive pointer field with complex extents is defined manually. flexfaceused
  // primitive pointer field with complex extents is defined manually. flexedge
  // primitive pointer field with complex extents is defined manually. flexvert
  // primitive pointer field with complex extents is defined manually. flexface
  // primitive pointer field with complex extents is defined manually. flexnormal
  // primitive pointer field with complex extents is defined manually. flextexcoord
  mjtByte flexvertopt() const {
    return ptr_->flexvertopt;
  }
  void set_flexvertopt(mjtByte value) {
    ptr_->flexvertopt = value;
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
  int nskin() const {
    return ptr_->nskin;
  }
  void set_nskin(int value) {
    ptr_->nskin = value;
  }
  // primitive pointer field with complex extents is defined manually. skinfacenum
  // primitive pointer field with complex extents is defined manually. skinvertadr
  // primitive pointer field with complex extents is defined manually. skinvertnum
  // primitive pointer field with complex extents is defined manually. skinvert
  // primitive pointer field with complex extents is defined manually. skinnormal
  int nlight() const {
    return ptr_->nlight;
  }
  void set_nlight(int value) {
    ptr_->nlight = value;
  }
  // array field is defined manually. lights
  // array field is defined manually. camera
  mjtByte enabletransform() const {
    return ptr_->enabletransform;
  }
  void set_enabletransform(mjtByte value) {
    ptr_->enabletransform = value;
  }
  emscripten::val translate() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->translate));
  }
  emscripten::val rotate() const {
    return emscripten::val(emscripten::typed_memory_view(4, ptr_->rotate));
  }
  float scale() const {
    return ptr_->scale;
  }
  void set_scale(float value) {
    ptr_->scale = value;
  }
  int stereo() const {
    return ptr_->stereo;
  }
  void set_stereo(int value) {
    ptr_->stereo = value;
  }
  emscripten::val flags() const {
    return emscripten::val(emscripten::typed_memory_view(10, ptr_->flags));
  }
  int framewidth() const {
    return ptr_->framewidth;
  }
  void set_framewidth(int value) {
    ptr_->framewidth = value;
  }
  emscripten::val framergb() const {
    return emscripten::val(emscripten::typed_memory_view(3, ptr_->framergb));
  }
  int status() const {
    return ptr_->status;
  }
  void set_status(int value) {
    ptr_->status = value;
  }
  mjvScene* get() const { return ptr_; }
  void set(mjvScene* ptr) { ptr_ = ptr; }

 private:
  mjvScene* ptr_;
  bool owned_ = false;

 public:
  mjModel *model;
  std::vector<MjvLight> lights;
  std::vector<MjvGLCamera> camera;
};

struct MjSpec {
  MjSpec();
  explicit MjSpec(mjSpec *ptr);
  MjSpec(const MjSpec &);
  MjSpec &operator=(const MjSpec &);
  ~MjSpec();
  std::unique_ptr<MjSpec> copy();
  // complex pointer field is defined manually. element
  mjString modelname() const {
    return (ptr_ && ptr_->modelname) ? *(ptr_->modelname) : "";
  }
  void set_modelname(const mjString& value) {
    if (ptr_ && ptr_->modelname) {
      *(ptr_->modelname) = value;
    }
  }
  // struct field is defined manually. compiler
  mjtByte strippath() const {
    return ptr_->strippath;
  }
  void set_strippath(mjtByte value) {
    ptr_->strippath = value;
  }
  // struct field is defined manually. option
  // struct field is defined manually. visual
  // struct field is defined manually. stat
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
  mjSpec* get() const { return ptr_; }
  void set(mjSpec* ptr) { ptr_ = ptr; }

 private:
  mjSpec* ptr_;
  bool owned_ = false;

 public:
  MjOption option;
  MjVisual visual;
  MjStatistic stat;
  MjsCompiler compiler;
  MjsElement element;
};

// TODO: Refactor, Structs Manually added so functions.cc compile -- //
struct MjpResourceProvider {
  MjpResourceProvider(mjpResourceProvider *ptr_) { ptr = ptr_; };
  ~MjpResourceProvider() {}
  mjpResourceProvider *get() const { return ptr; }
  mjpResourceProvider *ptr;
};

struct MjpPlugin {
  MjpPlugin(mjpPlugin *ptr_) { ptr = ptr_; };
  ~MjpPlugin() {}
  mjpPlugin *get() const { return ptr; }
  mjpPlugin *ptr;
};

// TODO: Factory and debug helper functions, some should be removed when
// functions are generated -- //
std::unique_ptr<MjModel> loadFromXML(std::string filename);
void step(MjModel *model, MjData *data);
void error(const std::string &msg);
void kinematics(MjModel *m, MjData *d);
std::unique_ptr<MjSpec> parseXMLString(const std::string &xml);
std::unique_ptr<MjsBody> findBody(MjSpec *spec, const std::string &name);
std::unique_ptr<MjsGeom> findGeom(MjSpec *spec, const std::string &name);

}  // namespace mujoco::wasm

#endif  // MUJOCO_WASM_CODEGEN_GENERATED_BINDINGS_H_
// NOLINTEND(whitespace/line_length)
