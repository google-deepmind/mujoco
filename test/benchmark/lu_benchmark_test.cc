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

// A benchmark comparing generic (runtime-sized) and fixed-size dense LU
// factorization and solve at n = 6, with dense Cholesky as a reference.
// The 6x6 case is the free-body block in the implicitfast integrator.

#include <random>
#include <vector>

#include "benchmark/benchmark.h"
#include <absl/base/attributes.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_solve.h"

namespace mujoco {
namespace {

constexpr int kN = 6;
constexpr int kNumMat = 256;  // cycle through matrices to avoid cache pinning

// generate matrices with the structure of the free-body block M - h*D + h*G:
// symmetric positive definite plus a scaled antisymmetric part
struct LUData {
  std::vector<mjtNum> mats;  // kNumMat x 36
  std::vector<mjtNum> rhs;   // kNumMat x 6

  LUData() : mats(kNumMat * kN * kN), rhs(kNumMat * kN) {
    std::mt19937_64 rng(42);
    std::normal_distribution<double> dist(0, 1);

    for (int m = 0; m < kNumMat; m++) {
      mjtNum* A = mats.data() + m * kN * kN;
      mjtNum sqrtH[kN * kN];
      for (int i = 0; i < kN * kN; i++) {
        sqrtH[i] = dist(rng);
      }

      // A = sqrtH' * sqrtH + n*I: SPD
      mju_mulMatTMat(A, sqrtH, sqrtH, kN, kN, kN);
      for (int i = 0; i < kN; i++) {
        A[i * kN + i] += kN;
      }

      // add antisymmetric part (gyroscopic-like), scaled to ~10% of diagonal
      for (int r = 0; r < kN; r++) {
        for (int c = r + 1; c < kN; c++) {
          mjtNum g = 0.6 * dist(rng);
          A[r * kN + c] += g;
          A[c * kN + r] -= g;
        }
      }

      for (int i = 0; i < kN; i++) {
        rhs[m * kN + i] = dist(rng);
      }
    }
  }
};

LUData& GetData() {
  static LUData* data = new LUData();
  return *data;
}

enum class Variant {
  kGeneric,  // mju_factorLU / mju_solveLU with n = 6
  kFixed,    // mju_factorLU6 / mju_solveLU6
  kCholesky  // mju_cholFactor / mju_cholSolve (symmetric part; reference)
};

// factor only (includes the copy, identical across variants)
ABSL_ATTRIBUTE_NO_TAIL_CALL static void FactorBenchmark(benchmark::State& state,
                                                        Variant variant) {
  LUData& data = GetData();
  mjtNum A[kN * kN];
  int pivot[kN];
  int m = 0;

  for (auto s : state) {
    mju_copy(A, data.mats.data() + m * kN * kN, kN * kN);
    m = (m + 1) % kNumMat;

    int rank;
    switch (variant) {
      case Variant::kGeneric:
        rank = mju_factorLU(A, kN, pivot);
        break;
      case Variant::kFixed:
        rank = mju_factorLU6(A, pivot);
        break;
      case Variant::kCholesky:
        rank = mju_cholFactor(A, kN, 0);
        break;
    }
    benchmark::DoNotOptimize(rank);
    benchmark::DoNotOptimize(A);
  }

  state.counters["factor/s"] =
      benchmark::Counter(state.iterations(), benchmark::Counter::kIsRate);
}

// factor and solve
ABSL_ATTRIBUTE_NO_TAIL_CALL static void FactorSolveBenchmark(
    benchmark::State& state, Variant variant) {
  LUData& data = GetData();
  mjtNum A[kN * kN];
  mjtNum x[kN];
  int pivot[kN];
  int m = 0;

  for (auto s : state) {
    mju_copy(A, data.mats.data() + m * kN * kN, kN * kN);
    const mjtNum* b = data.rhs.data() + m * kN;
    m = (m + 1) % kNumMat;

    switch (variant) {
      case Variant::kGeneric:
        mju_factorLU(A, kN, pivot);
        mju_solveLU(x, A, b, pivot, kN);
        break;
      case Variant::kFixed:
        mju_factorLU6(A, pivot);
        mju_solveLU6(x, A, b, pivot);
        break;
      case Variant::kCholesky:
        mju_cholFactor(A, kN, 0);
        mju_cholSolve(x, A, b, kN);
        break;
    }
    benchmark::DoNotOptimize(x);
  }

  state.counters["solve/s"] =
      benchmark::Counter(state.iterations(), benchmark::Counter::kIsRate);
}

void BM_factorLU_generic6(benchmark::State& state) {
  FactorBenchmark(state, Variant::kGeneric);
}
BENCHMARK(BM_factorLU_generic6);

void BM_factorLU_fixed6(benchmark::State& state) {
  FactorBenchmark(state, Variant::kFixed);
}
BENCHMARK(BM_factorLU_fixed6);

void BM_factorChol6(benchmark::State& state) {
  FactorBenchmark(state, Variant::kCholesky);
}
BENCHMARK(BM_factorChol6);

void BM_factorSolveLU_generic6(benchmark::State& state) {
  FactorSolveBenchmark(state, Variant::kGeneric);
}
BENCHMARK(BM_factorSolveLU_generic6);

void BM_factorSolveLU_fixed6(benchmark::State& state) {
  FactorSolveBenchmark(state, Variant::kFixed);
}
BENCHMARK(BM_factorSolveLU_fixed6);

void BM_factorSolveChol6(benchmark::State& state) {
  FactorSolveBenchmark(state, Variant::kCholesky);
}
BENCHMARK(BM_factorSolveChol6);

}  // namespace
}  // namespace mujoco

int main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  return 0;
}
