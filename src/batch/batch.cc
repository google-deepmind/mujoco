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

#include <mujoco/experimental/batch.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csetjmp>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "engine/engine_util_errmem.h"

// setjmp on macOS saves the signal mask with a system call; _setjmp does not
#if defined(__APPLE__)
#define mjb_setjmp _setjmp
#define mjb_longjmp _longjmp
#else
#define mjb_setjmp setjmp
#define mjb_longjmp longjmp
#endif

// hint to the CPU that the thread is spin-waiting
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
static inline void CpuPause() { _mm_pause(); }
#elif defined(_MSC_VER) && (defined(_M_ARM) || defined(_M_ARM64))
#include <intrin.h>
static inline void CpuPause() { __yield(); }
#elif defined(__aarch64__) || defined(__arm__)
static inline void CpuPause() { __asm__ __volatile__("yield" ::: "memory"); }
#else
static inline void CpuPause() {}
#endif

namespace {

//------------------------------------------ fields -----------------------------------------------

// one field of mjModel or mjData, sized for a particular model
struct Field {
  void* (*get)(void* obj);            // pointer to the field in an mjModel or mjData
  void (*set)(void* obj, void* ptr);  // mjModel arrays: point the field at ptr
  size_t bytes;                       // bytes per simulation
  int size;                           // elements per simulation
  int elemsize;                       // bytes per element
  bool state;                         // mjData: in the mjSTATE_INTEGRATION row
  bool asset;                         // mjModel: large constant data mj_setConst never writes
};

// fields by name
using FieldTable = std::unordered_map<std::string, Field>;

// whether an mjData field is in the mjSTATE_INTEGRATION row
bool InState(std::string_view name) {
  for (std::string_view s : {"qpos", "qvel", "act", "history", "qacc_warmstart", "ctrl",
                             "qfrc_applied", "xfrc_applied", "eq_active", "mocap_pos",
                             "mocap_quat", "userdata", "plugin_state"}) {
    if (name == s) return true;
  }
  return false;
}

// whether an mjModel field is asset data, shared by the thread models
bool IsAsset(std::string_view name) {
  for (std::string_view p : {"mesh_", "hfield_", "tex_", "skin_", "bvh_", "oct_"}) {
    if (name.substr(0, p.size()) == p) return true;
  }
  return false;
}

// field of nr x nc elements of elemsize bytes
Field MakeField(void* (*get)(void*), void (*set)(void*, void*), size_t nr, size_t nc,
                size_t elemsize, bool state, bool asset) {
  size_t size = nr * nc;
  return {get, set, size * elemsize, static_cast<int>(size), static_cast<int>(elemsize), state,
          asset};
}

// array fields of mjData
FieldTable DataFields(const mjModel* m) {
  FieldTable t;
#define X(type, name, nr, nc)                                                              \
  t[#name] = MakeField([](void* o) -> void* { return static_cast<mjData*>(o)->name; },     \
                       nullptr, m->nr, nc, sizeof(type), InState(#name), false);
  MJDATA_POINTERS
#undef X
  return t;
}

// array fields of mjModel and fields of mjOption
FieldTable ModelFields(const mjModel* m) {
  FieldTable t;
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
  MJMODEL_POINTERS_PREAMBLE(m)
#define X(type, name, nr, nc)                                                              \
  t[#name] = MakeField(                                                                     \
      [](void* o) -> void* { return static_cast<mjModel*>(o)->name; },                     \
      [](void* o, void* p) { static_cast<mjModel*>(o)->name = static_cast<type*>(p); },     \
      m->nr, nc, sizeof(type), false, IsAsset(#name));
  MJMODEL_POINTERS
#undef X
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

  // mjOption fields, named "opt.<name>"
#define X(type, name, n)                                                                   \
  t["opt." #name] = MakeField(                                                             \
      [](void* o) -> void* { return &static_cast<mjModel*>(o)->opt.name; }, nullptr, 1, n, \
      sizeof(type), false, false);
#define XVEC(type, name, n)                                                                \
  t["opt." #name] = MakeField(                                                             \
      [](void* o) -> void* { return static_cast<mjModel*>(o)->opt.name; }, nullptr, 1, n,  \
      sizeof(type), false, false);
  MJOPTION_FIELDS
#undef X
#undef XVEC
  return t;
}

// mjModel scalars that mj_setConst writes; kept per simulation like expanded fields
struct Scalars {
  mjtSize ngravcomp;                                   // number of bodies with gravcomp
  mjtByte flg_gravcomp, flg_surfacevel, flg_adhesion;  // features in use
  mjStatistic stat;                                    // model statistics

  // scalars of model m
  static Scalars Of(const mjModel* m) {
    return {m->ngravcomp, m->flg_gravcomp, m->flg_surfacevel, m->flg_adhesion, m->stat};
  }

  // write the scalars into model m
  void Apply(mjModel* m) const {
    m->ngravcomp = ngravcomp;
    m->flg_gravcomp = flg_gravcomp;
    m->flg_surfacevel = flg_surfacevel;
    m->flg_adhesion = flg_adhesion;
    m->stat = stat;
  }
};

// per-simulation storage for one field
struct Slot {
  const Field* field;              // the field
  std::unique_ptr<uint8_t[]> buf;  // (nsim, bytes)
  bool derived = false;            // expanded because mj_setConst changed it

  // storage of simulation i
  uint8_t* Row(int i) { return buf.get() + static_cast<size_t>(i) * field->bytes; }
};

// a record of mjb_rollout, resolved
struct Record {
  const Field* field;  // mjData field, or NULL for a state
  int spec;            // mjtState bits of the state
  size_t bytes;        // per substep
  uint8_t* buf;        // output, (n, nstep, bytes)
};

//------------------------------------------ errors -----------------------------------------------

thread_local std::jmp_buf* tls_jmp = nullptr;  // setjmp around the running simulation's call
thread_local char tls_error[1024];              // message of the trapped error
thread_local mjfLogHandler tls_prev = nullptr;  // the thread's handler before the batch's
thread_local const void* tls_batch = nullptr;   // batch whose simulation the thread runs

// trap errors by unwinding to tls_jmp, pass other messages to the previous handler
void BatchLogHandler(const mjLogMessage* msg) {
  if (msg->level == mjLOG_ERROR && tls_jmp) {
    std::snprintf(tls_error, sizeof(tls_error), "%s", msg->subject);
    mjb_longjmp(*tls_jmp, 1);
  }
  (tls_prev ? tls_prev : _mjPRIVATE_getGlobalLogHandler())(msg);
}

//------------------------------------------ thread pool ------------------------------------------

// thread pool of nthread - 1 workers and the caller, which is thread nthread - 1
class Pool {
 public:
  // task run on one item
  using Fn = void (*)(void* ctx, int thread, int item);

  // start the workers
  explicit Pool(int nthread) : nthread_(nthread), next_(new std::atomic<int>[nthread]) {
    for (int t = 0; t < nthread - 1; ++t) workers_.emplace_back([this, t] { Worker(t); });
  }

  // stop and join the workers
  ~Pool() {
    {
      std::lock_guard<std::mutex> lock(mu_);
      stop_.store(true, std::memory_order_release);
    }
    wake_.notify_all();
    for (auto& w : workers_) w.join();
  }

  // number of threads, including the caller's
  int size() const { return nthread_; }

  // run fn(ctx, thread, item) for item in [0, n) on all threads, return when all are done
  void Run(int n, Fn fn, void* ctx) {
    if (n == 0) return;
    fn_ = fn;
    ctx_ = ctx;
    n_ = n;
    for (int t = 0; t < nthread_; ++t) next_[t].store(Start(t), std::memory_order_relaxed);
    if (!workers_.empty()) {
      active_.store(static_cast<int>(workers_.size()), std::memory_order_relaxed);
      {
        std::lock_guard<std::mutex> lock(mu_);
        epoch_.fetch_add(1, std::memory_order_release);
      }
      wake_.notify_all();
    }
    Work(nthread_ - 1);
    if (!workers_.empty()) {
      if (!SpinUntil([this] { return active_.load(std::memory_order_acquire) == 0; })) {
        std::unique_lock<std::mutex> lock(mu_);
        done_.wait(lock, [this] { return active_.load(std::memory_order_acquire) == 0; });
      }
    }
  }

 private:
  // first item of thread t's slice: item i starts in the slice of thread i * nthread / n
  int Start(int t) const { return static_cast<int>(static_cast<int64_t>(t) * n_ / nthread_); }

  // spin until pred holds or 50 us pass; return whether it holds
  template <typename Pred>
  static bool SpinUntil(Pred pred) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(50);
    for (int k = 0;; ++k) {
      if (pred()) return true;
      if ((k & 63) == 63 && std::chrono::steady_clock::now() > deadline) return false;
      CpuPause();
    }
  }

  // run the items of this thread's slice, then take from the other slices
  void Work(int thread) {
    for (int k = 0; k < nthread_; ++k) {
      int t = (thread + k) % nthread_;
      int end = Start(t + 1);
      for (int i = next_[t].fetch_add(1, std::memory_order_relaxed); i < end;
           i = next_[t].fetch_add(1, std::memory_order_relaxed)) {
        fn_(ctx_, thread, i);
      }
    }
  }

  // worker loop: spin briefly after a call, so back-to-back calls skip the wake-up, then park
  void Worker(int thread) {
    _mjPRIVATE_setTlsLogHandler(BatchLogHandler);
    uint64_t seen = 0;
    while (true) {
      auto ready = [this, &seen] {
        return stop_.load(std::memory_order_acquire) ||
               epoch_.load(std::memory_order_acquire) != seen;
      };
      if (!SpinUntil(ready)) {
        std::unique_lock<std::mutex> lock(mu_);
        wake_.wait(lock, ready);
      }
      if (stop_.load(std::memory_order_acquire)) return;
      seen = epoch_.load(std::memory_order_acquire);
      Work(thread);
      if (active_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        std::lock_guard<std::mutex> lock(mu_);
        done_.notify_one();
      }
    }
  }

  const int nthread_;                         // threads, including the caller's
  std::vector<std::thread> workers_;          // nthread - 1 workers
  std::unique_ptr<std::atomic<int>[]> next_;  // next item of each slice
  std::mutex mu_;                             // guards parking and waking
  std::condition_variable wake_;              // wakes parked workers for a call
  std::condition_variable done_;              // wakes the caller when workers are done
  std::atomic<uint64_t> epoch_{0};            // number of calls so far
  std::atomic<int> active_{0};                // workers still running the call
  std::atomic<bool> stop_{false};             // workers should exit
  Fn fn_ = nullptr;                           // task of the call
  void* ctx_ = nullptr;                       // context of the task
  int n_ = 0;                                 // number of items of the call
};

}  // namespace

//------------------------------------------ batch ------------------------------------------------

// batch of simulations of one model
struct mjBatch_ {
  // operation of a call
  enum class Op { Step, Forward, Reset, SetConst, Rollout, Apply };

  int nsim;                                     // number of simulations
  int nstate;                                   // size of a state row
  bool persistent;                              // one mjData per simulation
  mjModel* model;                               // the batch's copy
  FieldTable data_fields;                       // mjData fields, by name
  FieldTable model_fields;                      // mjModel and mjOption fields, by name
  std::vector<const Field*> restorable;         // non-asset model fields
  std::vector<mjtNum> state;                    // (nsim, nstate)
  std::vector<mjWarningStat> warning;           // (nsim, mjNWARNING)
  std::vector<Scalars> scalars;                 // per simulation
  std::vector<int> status;                      // per simulation
  std::vector<std::string> errors;              // per simulation
  std::vector<std::unique_ptr<Slot>> outputs;   // registered mjData fields
  std::vector<std::unique_ptr<Slot>> expanded;  // per-simulation mjModel fields
  std::set<const Field*> expanded_set;          // fields of expanded
  std::vector<mjData*> data;                    // per thread, or per simulation if persistent
  std::vector<mjData*> scratch;                 // per thread, for set_const if persistent
  std::vector<mjModel*> models;                 // per thread, once a field is expanded
  std::vector<void*> model_buffers;             // their non-asset arrays
  std::set<const Field*> changed;               // set_const outputs not yet expanded
  std::unique_ptr<Pool> pool;                   // the batch's threads
  std::mutex mu;                                // serializes calls
  std::mutex changed_mu;                        // changed, from threads

  // call arguments
  Op op;                        // operation
  int arg;                      // nstep, or keyframe of reset
  const int* ids;               // simulations, NULL: all
  mjfBatchFunc func;            // apply: function
  void* func_arg;               // apply: its argument
  int save;                     // apply: save the state
  const mjtNum* control;        // rollout: controls, or NULL
  int control_spec;             // rollout: state components of the controls
  int ncontrol;                 // rollout: size of a control
  std::vector<Record> records;  // rollout: records

  // state row of simulation i
  mjtNum* State(int i) { return state.data() + static_cast<size_t>(i) * nstate; }
  // warning counters of simulation i
  mjWarningStat* Warning(int i) { return warning.data() + static_cast<size_t>(i) * mjNWARNING; }

  // restore the non-expanded model fields and the scalars of a thread's model
  void Restore(mjModel* m) {
    for (const Field* f : restorable) std::memcpy(f->get(m), f->get(model), f->bytes);
    Scalars::Of(model).Apply(m);
  }

  // give model field f per-simulation storage, seeded from the model
  Slot& Expand(const Field& f) {
    auto s = std::make_unique<Slot>();
    s->field = &f;
    s->buf = std::make_unique<uint8_t[]>(f.bytes * nsim);
    for (int i = 0; i < nsim; ++i) std::memcpy(s->Row(i), f.get(model), f.bytes);
    expanded.push_back(std::move(s));
    expanded_set.insert(&f);
    if (models.empty()) {
      for (int t = 0; t < pool->size(); ++t) models.push_back(ShallowCopy());
    }
    return *expanded.back();
  }

  // model for one thread: copies of the struct and non-asset arrays, sharing the asset arrays
  mjModel* ShallowCopy() {
    constexpr size_t kAlign = 64;
    size_t total = 0;
    for (auto& [name, f] : model_fields) {
      if (f.set && !f.asset) total += (f.bytes + kAlign - 1) / kAlign * kAlign;
    }
    auto* buf = static_cast<uint8_t*>(::operator new(total ? total : kAlign,
                                                      std::align_val_t(kAlign)));
    model_buffers.push_back(buf);
    auto* m = new mjModel(*model);
    size_t offset = 0;
    for (auto& [name, f] : model_fields) {
      if (!f.set || f.asset) continue;
      std::memcpy(buf + offset, f.get(model), f.bytes);
      f.set(m, buf + offset);
      offset += (f.bytes + kAlign - 1) / kAlign * kAlign;
    }
    m->buffer = nullptr;  // not an mjModel to free with mj_deleteModel
    m->nbuffer = 0;
    return m;
  }

  // number of warnings raised in d
  int WarningCount(const mjData* d) {
    int n = 0;
    for (int w = 0; w < mjNWARNING; ++w) n += d->warning[w].number;
    return n;
  }

  // copy record r of substep k for call position j out of d, or repeat substep k - 1
  void CopyRecord(const Record& r, const mjModel* m, mjData* d, int j, int k, bool repeat) {
    uint8_t* dst = r.buf + (static_cast<size_t>(j) * arg + k) * r.bytes;
    if (repeat) {
      std::memcpy(dst, dst - r.bytes, r.bytes);
    } else if (r.field) {
      std::memcpy(dst, r.field->get(d), r.bytes);
    } else {
      mj_getState(m, d, reinterpret_cast<mjtNum*>(dst), r.spec);
    }
  }

  // run the current op on simulation i, at position j of the call, on thread t; may longjmp
  void RunSim(int t, int i, int j) {
    mjModel* m = models.empty() ? model : models[t];
    mjData* d = Data(t, i);
    if (op == Op::SetConst) Restore(m);
    for (auto& s : expanded) {
      // mj_setConst recomputes its outputs from the template's, as on a fresh copy
      if (op == Op::SetConst && s->derived) continue;
      std::memcpy(s->field->get(m), s->Row(i), s->field->bytes);
    }

    if (op == Op::SetConst) {
      mj_setConst(m, d);
      for (auto& s : expanded) std::memcpy(s->Row(i), s->field->get(m), s->field->bytes);
      scalars[i] = Scalars::Of(m);
      for (const Field* f : restorable) {
        if (!expanded_set.count(f) && std::memcmp(f->get(m), f->get(model), f->bytes)) {
          std::lock_guard<std::mutex> lock(changed_mu);
          changed.insert(f);
        }
      }
      Restore(m);
      return;
    }

    if (!models.empty()) scalars[i].Apply(m);
    if (!persistent && (m->opt.enableflags & mjENBL_SLEEP)) {
      mju_error("sleep requires a persistent batch");
    }
    if (op == Op::Reset) {
      if (arg >= 0) {
        mj_resetDataKeyframe(m, d, arg);
      } else {
        mj_resetData(m, d);
      }
    } else {
      mj_setState(m, d, State(i), mjSTATE_INTEGRATION);
      std::memcpy(d->warning, Warning(i), sizeof(d->warning));
    }

    switch (op) {
      case Op::Step:
        for (int k = 0; k < arg; ++k) mj_step(m, d);
        break;
      case Op::Forward:
      case Op::Reset:
        mj_forward(m, d);
        break;
      case Op::Rollout: {
        int warnings = WarningCount(d);
        bool stopped = false;
        for (int k = 0; k < arg; ++k) {
          if (!stopped) {
            if (control) {
              size_t row = static_cast<size_t>(j) * arg + k;
              mj_setState(m, d, control + row * ncontrol, control_spec);
            }
            mj_step(m, d);
          }
          for (const Record& r : records) CopyRecord(r, m, d, j, k, stopped);
          stopped = stopped || WarningCount(d) > warnings;
        }
        break;
      }
      case Op::Apply:
        func(m, d, i, func_arg);
        break;
      case Op::SetConst:
        break;
    }

    if (op != Op::Apply || save) {
      mj_getState(m, d, State(i), mjSTATE_INTEGRATION);
      std::memcpy(Warning(i), d->warning, sizeof(d->warning));
    }

    // apply's function may compute any subset of the fields, so apply publishes none
    if (op != Op::Apply) {
      for (auto& s : outputs) std::memcpy(s->Row(i), s->field->get(d), s->field->bytes);
    }
  }

  // mjData for simulation i on thread t: set_const in a persistent batch uses scratch
  // data, since mj_setConst would clobber the simulation's sleep state and be misled by it
  mjData* Data(int t, int i) {
    if (!persistent) return data[t];
    return op == Op::SetConst ? scratch[t] : data[i];
  }

  // run simulation i, trapping a MuJoCo error into its status
  void Guarded(int t, int i, int j) {
    std::jmp_buf jb;
    std::jmp_buf* outer_jmp = tls_jmp;  // an enclosing batch call's, when nested
    const void* outer_batch = tls_batch;
    tls_jmp = &jb;
    tls_batch = this;
    status[i] = 0;
    if (mjb_setjmp(jb) == 0) {
      RunSim(t, i, j);
      errors[i].clear();
    } else {
      // the simulation's state was not saved; its mjData is reset for the next call
      tls_jmp = outer_jmp;
      if (op == Op::SetConst) Restore(models[t]);
      mj_resetData(model, Data(t, i));
      status[i] = 1;
      errors[i] = tls_error;
    }
    tls_jmp = outer_jmp;
    tls_batch = outer_batch;
  }

  // pool task: run the simulation at position j of the call
  static void Task(void* ctx, int thread, int j) {
    auto* b = static_cast<mjBatch_*>(ctx);
    b->Guarded(thread, b->ids ? b->ids[j] : j, j);
  }

  // run on the selection, with the caller's errors trapped like the workers'
  void RunLocked(const int* sel, int n) {
    ids = sel;
    mjfLogHandler handler = _mjPRIVATE_setTlsLogHandler(BatchLogHandler);
    mjfLogHandler prev = tls_prev;
    if (handler != BatchLogHandler) tls_prev = handler;  // not nested in a batch call
    pool->Run(n, &Task, this);
    _mjPRIVATE_setTlsLogHandler(handler);
    tls_prev = prev;
  }

  // number of failed simulations in the selection
  int Failures(const int* sel, int n) {
    int nfail = 0;
    for (int j = 0; j < n; ++j) nfail += status[sel ? sel[j] : j] != 0;
    return nfail;
  }

  // run op o with argument a on the selection; return the number of failed simulations
  int Run(Op o, const int* sel, int nid, int a) {
    // validate before locking: mju_error may not return, and must not leave mu held
    CheckIds(sel, nid);
    int n = sel ? nid : nsim;
    std::lock_guard<std::mutex> lock(mu);
    op = o;
    arg = a;

    if (o != Op::SetConst) {
      RunLocked(sel, n);
      return Failures(sel, n);
    }

    // Expand the fields mj_setConst changed and recompute every simulation, until no field
    // changes outside the expanded ones. A simulation that raises keeps its constants, since
    // nothing is written back for it, and raises again in each round.
    if (models.empty()) return 0;
    while (true) {
      RunLocked(sel, n);
      if (changed.empty()) break;
      for (const Field* f : changed) Expand(*f).derived = true;
      changed.clear();
      sel = nullptr;
      n = nsim;
    }
    return Failures(sel, n);
  }

  // check that the call is not made from this batch's own simulation (it would deadlock),
  // and that the selection is sorted, unique and in range
  void CheckIds(const int* sel, int nid) {
    CheckNotNested();
    if (!sel) return;
    for (int j = 0; j < nid; ++j) {
      if (sel[j] < 0 || sel[j] >= nsim || (j > 0 && sel[j] <= sel[j - 1])) {
        mju_error("mjb: simulation ids must be sorted, unique and in [0, %d)", nsim);
      }
    }
  }

  // raise if called from a function running on one of this batch's simulations
  void CheckNotNested() {
    if (tls_batch == this) mju_error("mjb: a function run by mjb_apply called its own batch");
  }
};

//------------------------------------------ C API ------------------------------------------------

// allocate a batch of nsim simulations of a copy of m
mjBatch* mjb_makeBatch(const mjModel* m, int nsim, int nthread, int persistent, char* error,
                       int error_sz) {
  auto fail = [&](const char* msg) -> mjBatch* {
    if (error && error_sz > 0) std::snprintf(error, error_sz, "%s", msg);
    return nullptr;
  };
  if (nsim < 1) return fail("nsim must be positive");
  if (!persistent && (m->opt.enableflags & mjENBL_SLEEP)) {
    return fail("sleep requires a persistent batch: its bookkeeping is not part of the state");
  }
  if (error && error_sz > 0) error[0] = '\0';

  auto* b = new mjBatch_;
  b->nsim = nsim;
  b->persistent = persistent != 0;
  b->model = mj_copyModel(nullptr, m);
  b->nstate = mj_stateSize(b->model, mjSTATE_INTEGRATION);
  b->data_fields = DataFields(b->model);
  b->model_fields = ModelFields(b->model);
  for (auto& [name, f] : b->model_fields) {
    if (!f.asset) b->restorable.push_back(&f);
  }
  b->scalars.assign(nsim, Scalars::Of(b->model));
  b->status.assign(nsim, 0);
  b->errors.resize(nsim);

  if (nthread <= 0) nthread = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
  b->pool = std::make_unique<Pool>(std::min(nthread, nsim));
  int ndata = b->persistent ? nsim : b->pool->size();
  for (int k = 0; k < ndata; ++k) b->data.push_back(mj_makeData(b->model));
  if (b->persistent) {
    for (int t = 0; t < b->pool->size(); ++t) b->scratch.push_back(mj_makeData(b->model));
  }

  b->state.resize(static_cast<size_t>(nsim) * b->nstate);
  b->warning.resize(static_cast<size_t>(nsim) * mjNWARNING);
  for (int i = 0; i < nsim; ++i) {
    mj_getState(b->model, b->data[0], b->State(i), mjSTATE_INTEGRATION);
  }
  return b;
}

// free a batch
void mjb_deleteBatch(mjBatch* b) {
  if (!b) return;
  b->pool.reset();
  for (mjData* d : b->data) mj_deleteData(d);
  for (mjData* d : b->scratch) mj_deleteData(d);
  for (mjModel* m : b->models) delete m;
  for (void* buf : b->model_buffers) ::operator delete(buf, std::align_val_t(64));
  mj_deleteModel(b->model);
  delete b;
}

// number of simulations
int mjb_nsim(const mjBatch* b) { return b->nsim; }

// number of threads, including the caller's
int mjb_nthread(const mjBatch* b) { return b->pool->size(); }

// whether the batch keeps one mjData per simulation
int mjb_persistent(const mjBatch* b) { return b->persistent; }

// the batch's copy of the model
const mjModel* mjb_model(const mjBatch* b) { return b->model; }

// size of a state row
int mjb_nstate(const mjBatch* b) { return b->nstate; }

// state rows
mjtNum* mjb_state(mjBatch* b) { return b->state.data(); }

// warning counters
mjWarningStat* mjb_warning(mjBatch* b) { return b->warning.data(); }

// register an mjData field to copy out after every call, return its storage
void* mjb_output(mjBatch* b, const char* name, int* size, int* elemsize) {
  b->CheckNotNested();
  std::lock_guard<std::mutex> lock(b->mu);
  auto it = b->data_fields.find(name);
  if (it == b->data_fields.end() || it->second.state) return nullptr;
  const Field& f = it->second;
  if (size) *size = f.size;
  if (elemsize) *elemsize = f.elemsize;
  for (auto& s : b->outputs) {
    if (s->field == &f) return s->buf.get();
  }
  auto s = std::make_unique<Slot>();
  s->field = &f;
  s->buf = std::make_unique<uint8_t[]>(f.bytes * b->nsim);  // zero until a call fills it
  b->outputs.push_back(std::move(s));
  return b->outputs.back()->buf.get();
}

// give an mjModel field per-simulation values, return their storage
void* mjb_expand(mjBatch* b, const char* name, int* size, int* elemsize) {
  b->CheckNotNested();
  std::lock_guard<std::mutex> lock(b->mu);
  auto it = b->model_fields.find(name);
  if (it == b->model_fields.end() || it->second.asset) return nullptr;
  const Field& f = it->second;
  if (size) *size = f.size;
  if (elemsize) *elemsize = f.elemsize;
  for (auto& s : b->expanded) {
    if (s->field == &f) return s->buf.get();
  }
  return b->Expand(f).buf.get();
}

// advance simulations by nstep calls to mj_step
int mjb_step(mjBatch* b, const int* ids, int nid, int nstep) {
  if (nstep < 1) mju_error("mjb_step: nstep must be positive");
  return b->Run(mjBatch_::Op::Step, ids, nid, nstep);
}

// run mj_forward on simulations
int mjb_forward(mjBatch* b, const int* ids, int nid) {
  return b->Run(mjBatch_::Op::Forward, ids, nid, 0);
}

// reset simulations to defaults or a keyframe, then run mj_forward
int mjb_reset(mjBatch* b, const int* ids, int nid, int key) {
  if (key >= b->model->nkey) mju_error("mjb_reset: keyframe %d out of range", key);
  return b->Run(mjBatch_::Op::Reset, ids, nid, key < 0 ? -1 : key);
}

// run mj_setConst on each simulation's model
int mjb_setConst(mjBatch* b, const int* ids, int nid) {
  return b->Run(mjBatch_::Op::SetConst, ids, nid, 0);
}

// advance simulations with per-substep controls and records
int mjb_rollout(mjBatch* b, const int* ids, int nid, int nstep, const mjtNum* control,
                int control_spec, const mjBatchRecord* record, int nrecord) {
  if (nstep < 1) mju_error("mjb_rollout: nstep must be positive");
  if (control && (control_spec & ~mjSTATE_USER)) {
    mju_error("mjb_rollout: control_spec must be a subset of mjSTATE_USER");
  }
  // validate everything before allocating: mju_error may not return
  for (int r = 0; r < nrecord; ++r) {
    const mjBatchRecord& rec = record[r];
    if (!rec.buf) mju_error("mjb_rollout: record %d has no buffer", r);
    if (rec.field && !b->data_fields.count(rec.field)) {
      mju_error("mjb_rollout: unknown field '%s'", rec.field);
    }
    if (!rec.field && (rec.spec < 0 || rec.spec >= (1 << mjNSTATE))) {
      mju_error("mjb_rollout: invalid state spec %d in record %d", rec.spec, r);
    }
  }
  b->CheckIds(ids, nid);

  std::lock_guard<std::mutex> lock(b->mu);
  b->control = control;
  b->control_spec = control_spec;
  b->ncontrol = control ? mj_stateSize(b->model, control_spec) : 0;
  b->records.clear();
  for (int r = 0; r < nrecord; ++r) {
    const mjBatchRecord& rec = record[r];
    auto* buf = static_cast<uint8_t*>(rec.buf);
    if (rec.field) {
      const Field& f = b->data_fields.at(rec.field);
      b->records.push_back({&f, 0, f.bytes, buf});
    } else {
      size_t bytes = sizeof(mjtNum) * mj_stateSize(b->model, rec.spec);
      b->records.push_back({nullptr, rec.spec, bytes, buf});
    }
  }
  b->op = mjBatch_::Op::Rollout;
  b->arg = nstep;
  int n = ids ? nid : b->nsim;
  b->RunLocked(ids, n);
  b->records.clear();
  return b->Failures(ids, n);
}

// run func on each simulation
int mjb_apply(mjBatch* b, const int* ids, int nid, mjfBatchFunc func, void* arg, int save) {
  if (!func) mju_error("mjb_apply: func is NULL");
  b->CheckIds(ids, nid);
  std::lock_guard<std::mutex> lock(b->mu);
  b->func = func;
  b->func_arg = arg;
  b->save = save;
  b->op = mjBatch_::Op::Apply;
  b->arg = 0;
  int n = ids ? nid : b->nsim;
  b->RunLocked(ids, n);
  return b->Failures(ids, n);
}

// status of each simulation's last call
const int* mjb_status(const mjBatch* b) { return b->status.data(); }

// error message of simulation sim's last call
const char* mjb_error(const mjBatch* b, int sim) {
  if (sim < 0 || sim >= b->nsim) return "";
  return b->errors[sim].c_str();
}
