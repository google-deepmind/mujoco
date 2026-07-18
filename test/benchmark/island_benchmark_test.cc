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

// Benchmarks island discovery on deterministic geodesic Rips graphs.  The
// corpus spans the connectivity transition of points sampled on S^2 and adds
// MuJoCo-relevant static and repeated incidences.  Corpus construction and
// validation are deliberately outside the timed region.

#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "src/engine/engine_island.h"

namespace mujoco {
namespace {

struct Edge {
  int first;
  int second;
};

struct GraphCase {
  std::string name;
  int node_count;
  std::vector<Edge> incidences;
  std::vector<int> expected_partition;
  int active_nodes;
  int expected_components;
  int pre_bridge_components;
  bool bridge_added;
  std::uint64_t expected_checksum;
};

struct Point {
  double x;
  double y;
  double z;
};

std::uint64_t SplitMix64(std::uint64_t& state) {
  state += 0x9e3779b97f4a7c15ULL;
  std::uint64_t value = state;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

double Uniform01(std::uint64_t& state) {
  return static_cast<double>(SplitMix64(state) >> 11) * 0x1.0p-53;
}

std::vector<Point> SampleSphere(int count, std::uint64_t seed) {
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  std::vector<Point> points;
  points.reserve(count);
  for (int i = 0; i < count; ++i) {
    const double z = 2.0 * Uniform01(seed) - 1.0;
    const double angle = kTwoPi * Uniform01(seed);
    const double radial = std::sqrt(std::max(0.0, 1.0 - z*z));
    points.push_back({radial * std::cos(angle), radial * std::sin(angle), z});
  }
  return points;
}

double Dot(const Point& a, const Point& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

std::vector<Edge> RipsEdges(const std::vector<Point>& points, double target_degree) {
  const double probability = target_degree / (points.size() - 1);
  const double radius = 2.0 * std::asin(std::sqrt(probability));
  const double minimum_dot = std::cos(radius);
  std::vector<Edge> edges;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(points.size()); ++j) {
      if (Dot(points[i], points[j]) >= minimum_dot) {
        edges.push_back({i, j});
      }
    }
  }
  return edges;
}

std::vector<int> CanonicalPartition(int node_count, const std::vector<Edge>& edges) {
  std::vector<std::vector<int>> adjacency(node_count);
  std::vector<bool> active(node_count, false);
  for (const Edge& edge : edges) {
    active[edge.first] = true;
    active[edge.second] = true;
    if (edge.first != edge.second) {
      adjacency[edge.first].push_back(edge.second);
      adjacency[edge.second].push_back(edge.first);
    }
  }

  std::vector<int> partition(node_count, -1);
  std::queue<int> pending;
  for (int start = 0; start < node_count; ++start) {
    if (!active[start] || partition[start] != -1) {
      continue;
    }
    partition[start] = start;
    pending.push(start);
    while (!pending.empty()) {
      const int node = pending.front();
      pending.pop();
      for (int neighbor : adjacency[node]) {
        if (partition[neighbor] == -1) {
          partition[neighbor] = start;
          pending.push(neighbor);
        }
      }
    }
  }
  return partition;
}

int CountComponents(const std::vector<int>& partition) {
  int count = 0;
  for (int node = 0; node < static_cast<int>(partition.size()); ++node) {
    count += partition[node] == node;
  }
  return count;
}

std::uint64_t PartitionChecksum(const std::vector<int>& partition) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (int value : partition) {
    hash ^= static_cast<std::uint32_t>(value);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::vector<int> CanonicalizeLabels(const std::vector<int>& labels) {
  std::vector<int> minimum(labels.size(), std::numeric_limits<int>::max());
  for (int node = 0; node < static_cast<int>(labels.size()); ++node) {
    if (labels[node] >= 0) {
      minimum[labels[node]] = std::min(minimum[labels[node]], node);
    }
  }
  std::vector<int> canonical(labels.size(), -1);
  for (int node = 0; node < static_cast<int>(labels.size()); ++node) {
    if (labels[node] >= 0) {
      canonical[node] = minimum[labels[node]];
    }
  }
  return canonical;
}

void DeterministicShuffle(std::vector<Edge>& edges, std::uint64_t seed) {
  for (std::size_t i = edges.size(); i > 1; --i) {
    const std::size_t j = SplitMix64(seed) % i;
    std::swap(edges[i - 1], edges[j]);
  }
}

bool AddCriticalBridge(const std::vector<Point>& points, std::vector<Edge>& edges) {
  const std::vector<int> partition = CanonicalPartition(points.size(), edges);
  if (CountComponents(partition) < 2) {
    return false;
  }

  double best_dot = -2.0;
  Edge bridge{-1, -1};
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(points.size()); ++j) {
      if (partition[i] >= 0 && partition[j] >= 0 && partition[i] != partition[j] &&
          Dot(points[i], points[j]) > best_dot) {
        best_dot = Dot(points[i], points[j]);
        bridge = {i, j};
      }
    }
  }
  if (bridge.first >= 0) {
    edges.push_back(bridge);
    return true;
  }
  return false;
}

GraphCase MakeCase(std::string name, int node_count, double target_degree,
                   std::uint64_t seed, bool critical_bridge, bool static_rows,
                   bool repeated_rows) {
  const std::vector<Point> points = SampleSphere(node_count, seed);
  std::vector<Edge> edges = RipsEdges(points, target_degree);
  const int pre_bridge_components =
      critical_bridge ? CountComponents(CanonicalPartition(node_count, edges)) : -1;
  bool bridge_added = false;
  if (critical_bridge) {
    bridge_added = AddCriticalBridge(points, edges);
  }

  const std::vector<Edge> unique_edges = edges;
  if (static_rows) {
    for (int node = 0; node < node_count; node += 17) {
      edges.push_back({node, node});
    }
  }
  if (repeated_rows) {
    for (std::size_t i = 0; i < unique_edges.size(); i += 11) {
      edges.push_back(unique_edges[i]);
      edges.push_back({unique_edges[i].second, unique_edges[i].first});
    }
  }
  DeterministicShuffle(edges, seed ^ 0xd1b54a32d192ed03ULL);

  std::vector<int> expected = CanonicalPartition(node_count, edges);
  const int active_nodes = std::count_if(expected.begin(), expected.end(),
                                         [](int component) { return component >= 0; });
  const int components = CountComponents(expected);
  const std::uint64_t checksum = PartitionChecksum(expected);
  return {std::move(name), node_count, std::move(edges), std::move(expected), active_nodes,
          components, pre_bridge_components, bridge_added, checksum};
}

const std::vector<GraphCase>& Corpus() {
  static const std::vector<GraphCase> corpus = {
      MakeCase("StableSparse_S2Rips_64", 64, 2.0, 0x33960001ULL, false, false, false),
      MakeCase("CriticalBridge_S2Rips_256", 256, 0.75 * std::log(256.0),
               0x33960002ULL, true, false, false),
      MakeCase("SupercriticalDense_S2Rips_256", 256, 2.0 * std::ceil(std::log(256.0)),
               0x33960003ULL, false, false, false),
      MakeCase("GroundedStaticRepeated_S2Rips_256", 256,
               2.0 * std::ceil(std::log(256.0)), 0x33960004ULL, false, true, true),
      MakeCase("StableRepeated_S2Rips_1024", 1024, 2.0, 0x33960005ULL,
               false, false, true),
      MakeCase("CriticalLarge_S2Rips_1024", 1024, std::ceil(std::log(1024.0)),
               0x33960006ULL, true, false, false),
  };
  return corpus;
}

struct FloodFillWorkspace {
  explicit FloodFillWorkspace(int node_count)
      : adjacency(node_count * node_count), rownnz(node_count), rowadr(node_count),
        colind(node_count * node_count), stack(node_count * node_count + node_count),
        island(node_count) {}

  std::vector<unsigned char> adjacency;
  std::vector<int> rownnz;
  std::vector<int> rowadr;
  std::vector<int> colind;
  std::vector<int> stack;
  std::vector<int> island;
};

int RunFloodFill(const GraphCase& graph, FloodFillWorkspace& work) {
  const int n = graph.node_count;
  std::fill(work.adjacency.begin(), work.adjacency.end(), 0);
  std::fill(work.rownnz.begin(), work.rownnz.end(), 0);
  for (const Edge& edge : graph.incidences) {
    work.adjacency[edge.first*n + edge.second] = 1;
    work.adjacency[edge.second*n + edge.first] = 1;
  }

  int address = 0;
  for (int row = 0; row < n; ++row) {
    work.rowadr[row] = address;
    for (int column = 0; column < n; ++column) {
      if (work.adjacency[row*n + column]) {
        work.colind[address++] = column;
        ++work.rownnz[row];
      }
    }
  }
  return mj_floodFill(work.island.data(), n, work.rownnz.data(), work.rowadr.data(),
                      work.colind.data(), work.stack.data());
}

struct DsuWorkspace {
  explicit DsuWorkspace(int node_count)
      : parent(node_count), island(node_count), dof_count(node_count, 1) {}

  std::vector<int> parent;
  std::vector<int> island;
  std::vector<int> dof_count;
};

int RunDsu(const GraphCase& graph, DsuWorkspace& work) {
  _mjPRIVATE_dsuInit(work.parent.data(), graph.node_count);
  for (const Edge& edge : graph.incidences) {
    _mjPRIVATE_dsuUnion(work.parent.data(), edge.first, edge.second);
  }
  int dof_count = 0;
  return _mjPRIVATE_dsuAssign(work.island.data(), work.parent.data(), work.dof_count.data(),
                              graph.node_count, &dof_count);
}

bool Validate(const GraphCase& graph) {
  FloodFillWorkspace flood(graph.node_count);
  DsuWorkspace dsu(graph.node_count);
  const int flood_components = RunFloodFill(graph, flood);
  const int dsu_components = RunDsu(graph, dsu);
  const bool bridge_valid = graph.pre_bridge_components < 0 ||
                            (graph.bridge_added &&
                             graph.pre_bridge_components == graph.expected_components + 1);
  return bridge_valid && flood_components == graph.expected_components &&
         dsu_components == graph.expected_components &&
         CanonicalizeLabels(flood.island) == graph.expected_partition &&
         CanonicalizeLabels(dsu.island) == graph.expected_partition &&
         PartitionChecksum(graph.expected_partition) == graph.expected_checksum;
}

void BM_FloodFill(benchmark::State& state, const GraphCase* graph) {
  if (!Validate(*graph)) {
    state.SkipWithError("invalid S2-Rips graph fixture");
    return;
  }
  FloodFillWorkspace work(graph->node_count);
  state.SetLabel("edges=" + std::to_string(graph->incidences.size()) +
                 " active=" + std::to_string(graph->active_nodes) +
                 " components=" + std::to_string(graph->expected_components) +
                 " pre_bridge=" + std::to_string(graph->pre_bridge_components) +
                 " bridge_added=" + std::to_string(graph->bridge_added) +
                 " checksum=" + std::to_string(graph->expected_checksum));
  for (auto _ : state) {
    int components = RunFloodFill(*graph, work);
    benchmark::DoNotOptimize(components);
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * graph->incidences.size());
}

void BM_Dsu(benchmark::State& state, const GraphCase* graph) {
  if (!Validate(*graph)) {
    state.SkipWithError("invalid S2-Rips graph fixture");
    return;
  }
  DsuWorkspace work(graph->node_count);
  state.SetLabel("edges=" + std::to_string(graph->incidences.size()) +
                 " active=" + std::to_string(graph->active_nodes) +
                 " components=" + std::to_string(graph->expected_components) +
                 " pre_bridge=" + std::to_string(graph->pre_bridge_components) +
                 " bridge_added=" + std::to_string(graph->bridge_added) +
                 " checksum=" + std::to_string(graph->expected_checksum));
  for (auto _ : state) {
    int components = RunDsu(*graph, work);
    benchmark::DoNotOptimize(components);
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * graph->incidences.size());
}

const bool kRegistered = [] {
  for (const GraphCase& graph : Corpus()) {
    benchmark::RegisterBenchmark(("Island/FloodFill/" + graph.name).c_str(), BM_FloodFill, &graph);
    benchmark::RegisterBenchmark(("Island/DSU/" + graph.name).c_str(), BM_Dsu, &graph);
  }
  return true;
}();

}  // namespace
}  // namespace mujoco
