// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_SOLVE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_SOLVE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// Cholesky decomposition: mat = L*L'; return rank
MJAPI int mju_cholFactor(mjtNum* mat, int n, mjtNum mindiag);

// Cholesky solve
MJAPI void mju_cholSolve(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int n);

// Cholesky rank-one update: L*L' +/- x*x'; return rank
MJAPI int mju_cholUpdate(mjtNum* mat, mjtNum* x, int n, int flg_plus);

// sparse reverse-order Cholesky decomposition: mat = L'*L; return 'rank'
//  mat must have uncompressed layout; rownnz is modified to end at diagonal
int mju_cholFactorSparse(mjtNum* mat, int n, mjtNum mindiag,
                         int* rownnz, int* rowadr, int* colind,
                         mjData* d);

// sparse reverse-order Cholesky solve
void mju_cholSolveSparse(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int n,
                         const int* rownnz, const int* rowadr, const int* colind);

// sparse reverse-order Cholesky rank-one update: L'*L +/i x*x'; return rank
//  x is sparse, change in sparsity pattern of mat is not allowed
int mju_cholUpdateSparse(mjtNum* mat, mjtNum* x, int n, int flg_plus,
                         int* rownnz, int* rowadr, int* colind, int x_nnz, int* x_ind,
                         mjData* d);

// sparse reverse-order LU factorization, no fill-in (assuming tree topology)
//  LU = L + U; original = (U+I) * L; scratch is size n
void mju_factorLUSparse(mjtNum *LU, int n, int* scratch,
                        const int *rownnz, const int *rowadr, const int *colind);

// solve mat*res=vec given LU factorization of mat
void mju_solveLUSparse(mjtNum *res, const mjtNum *LU, const mjtNum* vec, int n,
                       const int *rownnz, const int *rowadr, const int *colind);

// eigenvalue decomposition of symmetric 3x3 matrix
MJAPI int mju_eig3(mjtNum* eigval, mjtNum* eigvec, mjtNum quat[4], const mjtNum mat[9]);

// solve QCQP in 2 dimensions:
//  min  0.5*x'*A*x + x'*b  s.t.  sum (xi/di)^2 <= r^2
// return 0 if unconstrained, 1 if constrained
MJAPI int mju_QCQP2(mjtNum* res, const mjtNum* Ain, const mjtNum* bin, const mjtNum* d, mjtNum r);

// solve QCQP in 3 dimensions:
//  min  0.5*x'*A*x + x'*b  s.t.  sum (xi/di)^2 <= r^2
// return 0 if unconstrained, 1 if constrained
MJAPI int mju_QCQP3(mjtNum* res, const mjtNum* Ain, const mjtNum* bin, const mjtNum* d, mjtNum r);

// solve QCQP in n<=5 dimensions:
//  min  0.5*x'*A*x + x'*b  s.t.  sum (xi/di)^2 <= r^2
// return 0 if unconstrained, 1 if constrained
int mju_QCQP(mjtNum* res, const mjtNum* Ain, const mjtNum* bin, const mjtNum* d, mjtNum r, int n);

// solve box-constrained Quadratic Program
//  min 0.5*x'*H*x + x'*g  s.t. lower <= x <=upper
// return rank of unconstrained subspace or -1 on failure
MJAPI int mju_boxQP(mjtNum* res, mjtNum* R, int* index,
                    const mjtNum* H, const mjtNum* g, int n,
                    const mjtNum* lower, const mjtNum* upper);

// allocate memory for box-constrained Quadratic Program
MJAPI void mju_boxQPmalloc(mjtNum** res, mjtNum** R, int** index,
                           mjtNum** H, mjtNum** g, int n,
                           mjtNum** lower, mjtNum** upper);

// minimize 0.5*x'*H*x + x'*g  s.t. lower <= x <=upper, explicit options (see implemetation)
MJAPI int mju_boxQPoption(mjtNum* res, mjtNum* R, int* index,
                          const mjtNum* H, const mjtNum* g, int n,
                          const mjtNum* lower, const mjtNum* upper,
                          int maxiter, mjtNum mingrad, mjtNum backtrack,
                          mjtNum minstep, mjtNum armijo,
                          char* log, int logsz);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_SOLVE_H_
