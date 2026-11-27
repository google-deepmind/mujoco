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

// Benchmarks for procedural model editing with the mjSpec API.

#include <cmath>
#include <string>
#include <vector>

#include <benchmark/benchmark.h>
#include "absl/base/attributes.h"
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// add n geoms to the world body
static void add_geoms(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  for (auto s : state) {
    mjSpec* spec = mj_makeSpec();
    mjsBody* world = mjs_findBody(spec, "world");
    for (int i = 0; i < n; i++) {
      mjs_addGeom(world, nullptr);
    }
    state.PauseTiming();
    mj_deleteSpec(spec);
    state.ResumeTiming();
  }
  state.SetItemsProcessed(state.iterations() * n);
}

// add n bodies to the world body
static void add_bodies(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  for (auto s : state) {
    mjSpec* spec = mj_makeSpec();
    mjsBody* world = mjs_findBody(spec, "world");
    for (int i = 0; i < n; i++) {
      mjs_addBody(world, nullptr);
    }
    state.PauseTiming();
    mj_deleteSpec(spec);
    state.ResumeTiming();
  }
  state.SetItemsProcessed(state.iterations() * n);
}

// add a chain of n nested bodies
static void add_nested_bodies(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  for (auto s : state) {
    mjSpec* spec = mj_makeSpec();
    mjsBody* body = mjs_findBody(spec, "world");
    for (int i = 0; i < n; i++) {
      body = mjs_addBody(body, nullptr);
    }
    state.PauseTiming();
    mj_deleteSpec(spec);
    state.ResumeTiming();
  }
  state.SetItemsProcessed(state.iterations() * n);
}

// add n geoms to the world body, naming each one right after adding it
static void add_named_geoms(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  for (auto s : state) {
    mjSpec* spec = mj_makeSpec();
    mjsBody* world = mjs_findBody(spec, "world");
    for (int i = 0; i < n; i++) {
      mjsGeom* geom = mjs_addGeom(world, nullptr);
      mjs_setName(geom->element, ("geom_" + std::to_string(i)).c_str());
    }
    state.PauseTiming();
    mj_deleteSpec(spec);
    state.ResumeTiming();
  }
  state.SetItemsProcessed(state.iterations() * n);
}

// Use ABSL_ATTRIBUTE_NO_TAIL_CALL to make sure the benchmark functions appear
// separately in CPU profiles (and don't get replaced with raw calls to
// run_*_benchmark).

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_SpecAddGeoms(benchmark::State& state) {
  add_geoms(state);
}
BENCHMARK(BM_SpecAddGeoms)->Arg(1000)->Arg(8000);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_SpecAddBodies(benchmark::State& state) {
  add_bodies(state);
}
BENCHMARK(BM_SpecAddBodies)->Arg(1000)->Arg(8000);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_SpecAddNestedBodies(benchmark::State& state) {
  add_nested_bodies(state);
}
BENCHMARK(BM_SpecAddNestedBodies)->Arg(1000);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_SpecAddNamedGeoms(benchmark::State& state) {
  add_named_geoms(state);
}
BENCHMARK(BM_SpecAddNamedGeoms)->Arg(1000)->Arg(8000);

// add a subdivided sphere mesh asset and referencing geom to worldbody
static void AddSphereMesh(mjSpec* spec, mjsBody* world, int index) {
  constexpr int kGrid = 10;
  std::vector<float> vert;
  std::vector<int> face;
  vert.reserve(6 * (kGrid + 1) * (kGrid + 1) * 3);
  face.reserve(6 * 2 * kGrid * kGrid * 3);

  for (int f = 0; f < 6; f++) {
    int axis = f / 2;
    float sign = (f % 2 == 0) ? 1.0f : -1.0f;
    int u_axis = (axis + 1) % 3;
    int v_axis = (axis + 2) % 3;
    int base = static_cast<int>(vert.size() / 3);
    for (int iu = 0; iu <= kGrid; iu++) {
      for (int iv = 0; iv <= kGrid; iv++) {
        float p[3] = {0.0f, 0.0f, 0.0f};
        p[axis] = sign;
        p[u_axis] = -1.0f + 2.0f * static_cast<float>(iu) / kGrid;
        p[v_axis] = -1.0f + 2.0f * static_cast<float>(iv) / kGrid;
        float inv_norm =
            0.1f / std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        vert.push_back(p[0] * inv_norm);
        vert.push_back(p[1] * inv_norm);
        vert.push_back(p[2] * inv_norm);
      }
    }
    for (int iu = 0; iu < kGrid; iu++) {
      for (int iv = 0; iv < kGrid; iv++) {
        int i00 = base + iu * (kGrid + 1) + iv;
        int i10 = base + (iu + 1) * (kGrid + 1) + iv;
        int i01 = base + iu * (kGrid + 1) + (iv + 1);
        int i11 = base + (iu + 1) * (kGrid + 1) + (iv + 1);
        if (sign > 0.0f) {
          face.insert(face.end(), {i00, i10, i11, i00, i11, i01});
        } else {
          face.insert(face.end(), {i00, i11, i10, i00, i01, i11});
        }
      }
    }
  }

  std::string name = "mesh_" + std::to_string(index);
  mjsMesh* mesh = mjs_addMesh(spec, nullptr);
  mjs_setName(mesh->element, name.c_str());
  mjs_setFloat(mesh->uservert, vert.data(), static_cast<int>(vert.size()));
  mjs_setInt(mesh->userface, face.data(), static_cast<int>(face.size()));

  mjsGeom* geom = mjs_addGeom(world, nullptr);
  geom->type = mjGEOM_MESH;
  mjs_setString(geom->meshname, name.c_str());
}

// delete n geoms from the world body
static void delete_geoms(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  bool compiled = state.range(1);

  mjSpec* base = mj_makeSpec();
  mjsBody* base_world = mjs_findBody(base, "world");
  for (int i = 0; i < n; i++) {
    mjsGeom* geom = mjs_addGeom(base_world, nullptr);
    geom->size[0] = 0.01;
  }
  if (compiled) {
    mjModel* model = mj_compile(base, nullptr);
    if (!model) {
      state.SkipWithError(mjs_getError(base));
      mj_deleteSpec(base);
      return;
    }
    mj_deleteModel(model);
  }

  auto prepare = [](mjSpec* src, std::vector<mjsElement*>& elems) {
    mjSpec* copy = mj_copySpec(src);
    elems.clear();
    mjsElement* el = mjs_firstElement(copy, mjOBJ_GEOM);
    while (el) {
      elems.push_back(el);
      el = mjs_nextElement(copy, el);
    }
    return copy;
  };

  std::vector<mjsElement*> elems;
  elems.reserve(n);
  mjSpec* spec = prepare(base, elems);
  for (auto s : state) {
    for (mjsElement* el : elems) {
      mjs_delete(spec, el);
    }
    state.PauseTiming();
    mj_deleteSpec(spec);
    spec = prepare(base, elems);
    state.ResumeTiming();
  }
  mj_deleteSpec(spec);
  mj_deleteSpec(base);
  state.SetItemsProcessed(state.iterations() * n);
}

// delete n bodies from the world body
static void delete_bodies(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  bool compiled = state.range(1);

  mjSpec* base = mj_makeSpec();
  mjsBody* base_world = mjs_findBody(base, "world");
  for (int i = 0; i < n; i++) {
    mjsBody* body = mjs_addBody(base_world, nullptr);
    mjs_addJoint(body, nullptr);
    mjsGeom* geom = mjs_addGeom(body, nullptr);
    geom->size[0] = 0.01;
  }
  if (compiled) {
    mjModel* model = mj_compile(base, nullptr);
    if (!model) {
      state.SkipWithError(mjs_getError(base));
      mj_deleteSpec(base);
      return;
    }
    mj_deleteModel(model);
  }

  auto prepare = [](mjSpec* src, std::vector<mjsElement*>& elems) {
    mjSpec* copy = mj_copySpec(src);
    elems.clear();
    mjsBody* world = mjs_findBody(copy, "world");
    mjsElement* el = mjs_firstChild(world, mjOBJ_BODY, /*recurse=*/0);
    while (el) {
      elems.push_back(el);
      el = mjs_nextChild(world, el, /*recurse=*/0);
    }
    return copy;
  };

  std::vector<mjsElement*> elems;
  elems.reserve(n);
  mjSpec* spec = prepare(base, elems);
  for (auto s : state) {
    for (mjsElement* el : elems) {
      mjs_delete(spec, el);
    }
    state.PauseTiming();
    mj_deleteSpec(spec);
    spec = prepare(base, elems);
    state.ResumeTiming();
  }
  mj_deleteSpec(spec);
  mj_deleteSpec(base);
  state.SetItemsProcessed(state.iterations() * n);
}

// delete n sites from a mesh-bearing model (uncompiled vs compiled)
static void delete_from_mesh_model(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  bool compiled = state.range(1);

  mjSpec* base = mj_makeSpec();
  mjsBody* base_world = mjs_findBody(base, "world");
  for (int m = 0; m < 16; m++) {
    AddSphereMesh(base, base_world, m);
  }
  for (int i = 0; i < n; i++) {
    mjs_addSite(base_world, nullptr);
  }
  if (compiled) {
    mjModel* model = mj_compile(base, nullptr);
    if (!model) {
      state.SkipWithError(mjs_getError(base));
      mj_deleteSpec(base);
      return;
    }
    mj_deleteModel(model);
  }

  auto prepare = [](mjSpec* src, std::vector<mjsElement*>& elems) {
    mjSpec* copy = mj_copySpec(src);
    elems.clear();
    mjsElement* el = mjs_firstElement(copy, mjOBJ_SITE);
    while (el) {
      elems.push_back(el);
      el = mjs_nextElement(copy, el);
    }
    return copy;
  };

  std::vector<mjsElement*> elems;
  elems.reserve(n);
  mjSpec* spec = prepare(base, elems);
  for (auto s : state) {
    for (mjsElement* el : elems) {
      mjs_delete(spec, el);
    }
    state.PauseTiming();
    mj_deleteSpec(spec);
    spec = prepare(base, elems);
    state.ResumeTiming();
  }
  mj_deleteSpec(spec);
  mj_deleteSpec(base);
  state.SetItemsProcessed(state.iterations() * n);
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_SpecDeleteGeoms(benchmark::State& state) {
  delete_geoms(state);
}
BENCHMARK(BM_SpecDeleteGeoms)
    ->ArgsProduct({{200, 1000}, {0, 1}})
    ->ArgNames({"geoms", "compiled"});

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_SpecDeleteBodies(benchmark::State& state) {
  delete_bodies(state);
}
BENCHMARK(BM_SpecDeleteBodies)
    ->ArgsProduct({{200, 1000}, {0, 1}})
    ->ArgNames({"bodies", "compiled"});

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_SpecDeleteFromMeshModel(benchmark::State& state) {
  delete_from_mesh_model(state);
}
BENCHMARK(BM_SpecDeleteFromMeshModel)
    ->ArgsProduct({{50, 200}, {0, 1}})
    ->ArgNames({"sites", "compiled"});

}  // namespace
}  // namespace mujoco
