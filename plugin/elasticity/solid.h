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

#ifndef MUJOCO_PLUGIN_ELASTICITY_SOLID_H_
#define MUJOCO_PLUGIN_ELASTICITY_SOLID_H_

#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>


namespace mujoco::plugin::elasticity {

struct Stencil3D {
  static constexpr int kNumEdges = 6;
  static constexpr int kNumVerts = 4;
  int vertices[kNumVerts];
  int edges[kNumEdges];
};

class Solid {
 public:
  // Returns a new Solid instance or nullopt on failure.
  static std::optional<Solid> Create(const mjModel* m, mjData* d, int instance);
  Solid(Solid&&) = default;

  Solid& operator=(Solid&& other) = default;

  void Compute(const mjModel* m, mjData* d, int instance);

  static void RegisterPlugin();

  int i0;  // index of first body
  int nc;  // number of cubes in the grid
  int nv;  // number of vertices (bodies) in the solid
  int nt;  // number of volumetric elements (tetrahedra)
  int ne;  // number of edges in the solid

  // connectivity info for mapping tetrahedra to edges and vertices
  std::vector<Stencil3D> elements;          // 4 vertices and 6 edges (nt x 10)
  std::vector<std::pair<int, int> > edges;  // edge to vertex map     (ne x 2)

  // precomputed quantities
  std::vector<mjtNum> metric;               // geom-induced metric    (nt x 36)
  std::vector<mjtNum> reference;            // reference lengths      (ne x 1)
  std::vector<mjtNum> deformed;             // deformed lengths       (ne x 1)
  std::vector<mjtNum> previous;             // previous-step lengths  (ne x 1)

  mjtNum damping;

 private:
  Solid(const mjModel* m, mjData* d, int instance, int nx, int ny, int nz,
        mjtNum nu, mjtNum E, mjtNum damp);

  void CreateStencils(int nx, int ny, int nz);
};

}  // namespace mujoco::plugin::elasticity

#endif  // MUJOCO_PLUGIN_ELASTICITY_SOLID_H_
