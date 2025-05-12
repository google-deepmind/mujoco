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

#include "engine/engine_util_solve.h"

#include <stdio.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"

//---------------------------- dense Cholesky ------------------------------------------------------

// Cholesky decomposition: mat = L*L'; return 'rank'
int mju_cholFactor(mjtNum* mat, int n, mjtNum mindiag) {
  int rank = n;
  mjtNum tmp;

  // in-place Cholesky factorization
  for (int j=0; j < n; j++) {
    // compute new diagonal
    tmp = mat[j*(n+1)];
    if (j) {
      tmp -= mju_dot(mat+j*n, mat+j*n, j);
    }

    // correct diagonal values below threshold
    if (tmp < mindiag) {
      tmp = mindiag;
      rank--;
    }

    // save diagonal
    mat[j*(n+1)] = mju_sqrt(tmp);

    // process off-diagonal entries
    tmp = 1/mat[j*(n+1)];
    for (int i=j+1; i < n; i++) {
      mat[i*n+j] = (mat[i*n+j] - mju_dot(mat+i*n, mat+j*n, j)) * tmp;
    }
  }

  return rank;
}



// Cholesky solve
void mju_cholSolve(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int n) {
  // copy if source and destination are different
  if (res != vec) {
    mju_copy(res, vec, n);
  }

  // forward substitution: solve L*res = vec
  for (int i=0; i < n; i++) {
    if (i) {
      res[i] -= mju_dot(mat+i*n, res, i);
    }

    // diagonal
    res[i] /= mat[i*(n+1)];
  }

  // backward substitution: solve L'*res = res
  for (int i=n-1; i >= 0; i--) {
    if (i < n-1) {
      for (int j=i+1; j < n; j++) {
        res[i] -= mat[j*n+i] * res[j];
      }
    }
    // diagonal
    res[i] /= mat[i*(n+1)];
  }
}



// Cholesky rank-one update: L*L' +/- x*x'; return rank
int mju_cholUpdate(mjtNum* mat, mjtNum* x, int n, int flg_plus) {
  int rank = n;
  mjtNum r, c, cinv, s, Lkk, tmp;

  for (int k=0; k < n; k++) {
    if (x[k]) {
      // prepare constants
      Lkk = mat[k*(n+1)];
      tmp = Lkk*Lkk + (flg_plus ? x[k]*x[k] : -x[k]*x[k]);
      if (tmp < mjMINVAL) {
        tmp = mjMINVAL;
        rank--;
      }
      r = mju_sqrt(tmp);
      c = r / Lkk;
      cinv = 1 / c;
      s = x[k] / Lkk;

      // update diagonal
      mat[k*(n+1)] = r;

      // update mat
      if (flg_plus) {
        for (int i=k+1; i < n; i++) {
          mat[i*n+k] = (mat[i*n+k] + s*x[i])*cinv;
        }
      } else {
        for (int i=k+1; i < n; i++) {
          mat[i*n+k] = (mat[i*n+k] - s*x[i])*cinv;
        }
      }

      // update x
      for (int i=k+1; i < n; i++) {
        x[i] = c*x[i] - s*mat[i*n+k];
      }
    }
  }

  return rank;
}



//---------------------------- sparse Cholesky -----------------------------------------------------

// sparse reverse-order Cholesky decomposition: mat = L'*L; return 'rank'
//  mat must be lower-triangular, have preallocated space for fill-in
int mju_cholFactorSparse(mjtNum* mat, int n, mjtNum mindiag,
                         int* rownnz, const int* rowadr, int* colind,
                         mjData* d) {
  int rank = n;

  mj_markStack(d);
  mjtNum* buf = mjSTACKALLOC(d, n, mjtNum);
  int* buf_ind = mjSTACKALLOC(d, n, int);

  // backpass over rows
  for (int r=n-1; r >= 0; r--) {
    // get rownnz and rowadr for row r
    int nnz = rownnz[r], adr = rowadr[r];

    // update row r diagonal
    mjtNum tmp = mat[adr+nnz-1];
    if (tmp < mindiag) {
      tmp = mindiag;
      rank--;
    }
    mat[adr+nnz-1] = mju_sqrt(tmp);
    tmp = 1/mat[adr+nnz-1];

    // update row r before diagonal
    for (int i=0; i < nnz-1; i++) {
      mat[adr+i] *= tmp;
    }

    // update row c<r where mat(r,c)!=0
    for (int i=0; i < nnz-1; i++) {
      // get column index
      int c = colind[adr+i];

      // mat(c,0:c) = mat(c,0:c) - mat(r,c) * mat(r,0:c)
      int nnz_c = mju_combineSparse(mat + rowadr[c], mat+rowadr[r], 1, -mat[adr+i],
                                    rownnz[c], i+1, colind+rowadr[c], colind+rowadr[r],
                                    buf, buf_ind);

      // assign new nnz to row c
      rownnz[c] = nnz_c;
    }
  }

  mj_freeStack(d);
  return rank;
}



// precount row non-zeros of reverse-Cholesky factor L, return total non-zeros
//  based on ldl_symbolic from 'Algorithm 8xx: a concise sparse Cholesky factorization package'
//  reads pattern from upper triangle
int mju_cholFactorCount(int* L_rownnz, const int* rownnz, const int* rowadr, const int* colind,
                        int n, mjData* d) {
  mj_markStack(d);
  int* parent = mjSTACKALLOC(d, n, int);
  int* flag = mjSTACKALLOC(d, n, int);

  // loop over rows in reverse order
  for (int r = n - 1; r >= 0; r--) {
    parent[r] = -1;
    flag[r] = r;
    L_rownnz[r] = 1;  // start with 1 for diagonal

    // loop over non-zero columns of upper triangle
    int start = rowadr[r];
    int end = start + rownnz[r];
    for (int c = start; c < end; c++) {
      int i = colind[c];

      // skip lower triangle
      if (i <= r) {
        continue;
      }

      // traverse from i to ancestor, stop when row is flagged
      while (flag[i] != r) {
        // if not yet set, set parent to current row
        if (parent[i] == -1) {
          parent[i] = r;
        }

        // increment non-zeros, flag row i, advance to parent
        L_rownnz[i]++;
        flag[i] = r;
        i = parent[i];
      }
    }
  }

  mj_freeStack(d);

  // sum up all row non-zeros
  int nnz = 0;
  for (int r = 0; r < n; r++) {
    nnz += L_rownnz[r];
  }

  return nnz;
}



// sparse reverse-order Cholesky solve
void mju_cholSolveSparse(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int n,
                         const int* rownnz, const int* rowadr, const int* colind) {
  // copy input into result
  mju_copy(res, vec, n);

  // vec <- L^-T vec
  for (int i=n-1; i >= 0; i--) {
    if (res[i]) {
      // get rowadr[i], rownnz[i]
      const int adr = rowadr[i], nnz = rownnz[i];

      // x(i) /= L(i,i)
      res[i] /= mat[adr+nnz-1];
      mjtNum tmp = res[i];

      // x(j) -= L(i,j)*x(i), j=0:i-1
      for (int j=0; j < nnz-1; j++) {
        res[colind[adr+j]] -= mat[adr+j]*tmp;
      }
    }
  }

  // vec <- L^-1 vec
  for (int i=0; i < n; i++) {
    // get rowadr[i], rownnz[i]
    const int adr = rowadr[i], nnz = rownnz[i];

    // x(i) -= sum_j L(i,j)*x(j), j=0:i-1
    if (nnz > 1) {
      res[i] -= mju_dotSparse(mat+adr, res, nnz-1, colind+adr);
      // modulo AVX, the above line does
      // for (int j=0; j<nnz-1; j++)
      //   res[i] -= mat[adr+j]*res[colind[adr+j]];
    }


    // x(i) /= L(i,i)
    res[i] /= mat[adr+nnz-1];
  }
}



// sparse reverse-order Cholesky rank-one update: L'*L +/- x*x'; return rank
//  x is sparse, change in sparsity pattern of mat is not allowed
int mju_cholUpdateSparse(mjtNum* mat, mjtNum* x, int n, int flg_plus,
                         const int* rownnz, const int* rowadr, const int* colind,
                         int x_nnz, int* x_ind,
                         mjData* d) {
  mj_markStack(d);
  int* buf_ind = mjSTACKALLOC(d, n, int);
  mjtNum* sparse_buf = mjSTACKALLOC(d, n, mjtNum);

  // backpass over rows corresponding to non-zero x(r)
  int rank = n, i = x_nnz - 1;
  while (i >= 0) {
    // get rownnz and rowadr for this row
    int nnz = rownnz[x_ind[i]], adr = rowadr[x_ind[i]];

    // compute quantities
    mjtNum tmp = mat[adr+nnz-1]*mat[adr+nnz-1] + (flg_plus ? x[i]*x[i] : -x[i]*x[i]);
    if (tmp < mjMINVAL) {
      tmp = mjMINVAL;
      rank--;
    }
    mjtNum r = mju_sqrt(tmp);
    mjtNum c = r / mat[adr+nnz-1];
    mjtNum s = x[i] / mat[adr+nnz-1];

    // update diagonal
    mat[adr+nnz-1] = r;

    // update row:  mat(r,1:r-1) = (mat(r,1:r-1) + s*x(1:r-1)) / c
    mju_combineSparseInc(mat + adr, x, n, 1 / c, (flg_plus ? s / c : -s / c),
                         nnz-1, i, colind + adr, x_ind);

    // update x:  x(1:r-1) = c*x(1:r-1) - s*mat(r,1:r-1)
    int new_x_nnz = mju_combineSparse(x, mat+adr, c, -s, i, nnz-1, x_ind,
                                      colind+adr, sparse_buf, buf_ind);

    // update i, correct for changing x
    i = i - 1 + (new_x_nnz - i);
  }

  mj_freeStack(d);
  return rank;
}

//---------------------------- banded Cholesky -----------------------------------------------------

// band-dense Cholesky decomposition
//  returns minimum value in the factorized diagonal, or 0 if rank-deficient
//  mat has (ntotal-ndense) x nband + ndense x ntotal elements
//  the first (ntotal-ndense) x nband store the band part, left of diagonal, inclusive
//  the second ndense x ntotal store the band part as entire dense rows
//  add diagadd+diagmul*mat_ii to diagonal before factorization
mjtNum mju_cholFactorBand(mjtNum* mat, int ntotal, int nband, int ndense,
                          mjtNum diagadd, mjtNum diagmul) {
  int nsparse = ntotal - ndense;
  mjtNum mindiag = -1;

  // sparse part, including sparse-sparse and sparse-dense
  for (int j=0; j < nsparse; j++) {
    // number of non-zeros left of (j,j)
    int width_jj = mjMIN(j, nband-1);

    // number of non-zeros below (j,j), sparse part
    int height = mjMIN(nsparse-j-1, nband-1);

    // address of (j,j)
    int adr_jj = (j+1)*nband-1;

    // compute L(j,j), before sqrt
    mjtNum left_ij = width_jj > 0 ? mju_dot(mat+adr_jj-width_jj, mat+adr_jj-width_jj, width_jj) : 0;
    mjtNum Ljj = diagadd + diagmul*mat[adr_jj] + mat[adr_jj] - left_ij;

    // update mindiag
    if (Ljj < mindiag || mindiag < 0) {
      mindiag = Ljj;
    }

    // stop if rank-deficient
    if (Ljj < mjMINVAL) {
      return 0;
    }

    // compute Ljj, scale = 1/Ljj
    Ljj = mju_sqrt(Ljj);
    mjtNum scale = 1/Ljj;

    // compute L(i,j) for i>j, sparse part
    for (int i=j+1; i <= j+height; i++)   {
      // number of non-zeros left of (i,j)
      int width_ij = mjMIN(j, nband-1-i+j);

      // address of (i,j)
      int adr_ij = (i+1)*nband-1-i+j;

      // in-place computation of L(i,j)
      left_ij = width_ij > 0 ? mju_dot(mat+adr_jj-width_ij, mat+adr_ij-width_ij, width_ij) : 0;
      mat[adr_ij] = scale * (mat[adr_ij] - left_ij);
    }

    // compute L(i,j) for i>j, dense part
    for (int i=nsparse; i < ntotal; i++)   {
      // address of (i,j)
      int adr_ij = nsparse*nband + (i-nsparse)*ntotal + j;

      // in-place computation of L(i,j)
      //  number of non-zeros left of (i,j) now equals width_jj
      left_ij = width_jj > 0 ? mju_dot(mat+adr_jj-width_jj, mat+adr_ij-width_jj, width_jj) : 0;
      mat[adr_ij] = scale * (mat[adr_ij] - left_ij);
    }

    // save L(j,j)
    mat[adr_jj] = Ljj;
  }

  // dense part
  for (int j=nsparse; j < ntotal; j++) {
    // address of (j,j)
    int adr_jj = nsparse*nband + (j-nsparse)*ntotal + j;

    // compute Ljj
    mjtNum Ljj = diagadd + diagmul*mat[adr_jj] + mat[adr_jj] -
                 mju_dot(mat+adr_jj-j, mat+adr_jj-j, j);

    // update mindiag
    if (Ljj < mindiag || mindiag < 0) {
      mindiag = Ljj;
    }

    // stop if rank-deficient
    if (Ljj < mjMINVAL) {
      return 0;
    }

    // compute Ljj, scale = 1/Ljj
    Ljj = mju_sqrt(Ljj);
    mjtNum scale = 1/Ljj;

    // compute L(i,j) for i>j
    for (int i=j+1; i < ntotal; i++) {
      // address of off-diagonal element
      int adr_ij = adr_jj + ntotal*(i-j);

      // in-place computation of L(i,j)
      mat[adr_ij] = scale * (mat[adr_ij] - mju_dot(mat+adr_jj-j, mat+adr_ij-j, j));
    }

    // save L(j,j)
    mat[adr_jj] = Ljj;
  }

  return mindiag;
}



// solve with band-Cholesky decomposition
void mju_cholSolveBand(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                       int ntotal, int nband, int ndense) {
  int width, height, nsparse = ntotal - ndense;

  // copy into result if different
  if (res != vec) {
    mju_copy(res, vec, ntotal);
  }

  //------- forward substitution: solve L*res = vec

  // sparse part
  for (int i=0; i < nsparse; i++) {
    // number of non-zeros left of (i,i)
    width = mjMIN(i, nband-1);

    if (width) {
      res[i] -= mju_dot(mat+(i+1)*nband-1-width, res+i-width, width);
    }

    // diagonal
    res[i] /= mat[(i+1)*nband-1];
  }

  // dense part
  for (int i=nsparse; i < ntotal; i++) {
    res[i] -= mju_dot(mat+nsparse*nband+(i-nsparse)*ntotal, res, i);

    // diagonal
    res[i] /= mat[nsparse*nband+(i-nsparse)*ntotal+i];
  }

  //------- backward substitution: solve L'*res = res

  // dense part
  for (int i=ntotal-1; i >= nsparse; i--) {
    for (int j=i+1; j < ntotal; j++) {
      res[i] -= mat[nsparse*nband+(j-nsparse)*ntotal+i] * res[j];
    }

    // diagonal
    res[i] /= mat[nsparse*nband+(i-nsparse)*ntotal+i];
  }

  // sparse part
  for (int i=nsparse-1; i >= 0; i--) {
    // number of non-zeros below (i,i), sparse part
    height = mjMIN(nsparse-1-i, nband-1);

    // sparse rows
    for (int j=i+1; j <= i+height; j++)
      res[i] -= mat[(j+1)*nband-1-(j-i)] * res[j];

    // dense rows
    for (int j=nsparse; j < ntotal; j++)
      res[i] -= mat[nsparse*nband+(j-nsparse)*ntotal+i] * res[j];

    // diagonal
    res[i] /= mat[(i+1)*nband-1];
  }
}



// address of diagonal element i in band-dense matrix representation
int mju_bandDiag(int i, int ntotal, int nband, int ndense) {
  int nsparse = ntotal-ndense;

  // sparse part
  if (i < nsparse) {
    return i*nband + nband-1;
  }

  // dense part
  else {
    return nsparse*nband + (i-nsparse)*ntotal + i;
  }
}



// convert band matrix to dense matrix
void mju_band2Dense(mjtNum* res, const mjtNum* mat, int ntotal, int nband, int ndense,
                    mjtByte flg_sym) {
  int nsparse = ntotal-ndense;

  // clear all
  mju_zero(res, ntotal*ntotal);

  // sparse part
  for(int i=0; i < nsparse; i++) {
    // number of non-zeros left of (i,i)
    int width = mjMIN(i, nband-1);

    // copy data
    mju_copy(res + i*ntotal + i-width, mat + (i+1)*nband - (width+1), width+1);
  }

  // dense part
  for(int i=nsparse; i < ntotal; i++) {
    mju_copy(res + i*ntotal, mat + nsparse*nband + (i-nsparse)*ntotal, i+1);
  }

  // make symmetric
  if (flg_sym) {
    for(int i=0; i < ntotal; i++) {
      for (int j=i+1; j < ntotal; j++) {
        res[i*ntotal + j] = res[j*ntotal + i];
      }
    }
  }
}



// convert dense matrix to band matrix
void mju_dense2Band(mjtNum* res, const mjtNum* mat, int ntotal, int nband, int ndense) {
  int nsparse = ntotal-ndense;

  // sparse part
  for(int i=0; i < nsparse; i++) {
    // number of non-zeros left of (i,i)
    int width = mjMIN(i, nband-1);

    // copy data
    mju_copy(res + (i+1)*nband - (width+1), mat + i*ntotal + i-width, width+1);
  }

  // dense part
  for(int i=nsparse; i < ntotal; i++) {
    mju_copy(res + nsparse*nband + (i-nsparse)*ntotal, mat + i*ntotal, i+1);
  }
}



// multiply band-diagonal matrix with vector
void mju_bandMulMatVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                       int ntotal, int nband, int ndense, int nvec, mjtByte flg_sym) {
  int nsparse = ntotal-ndense;

  // handle multiple vectors
  for(int j=0; j < nvec; j++ ) {
    // precompute pointer to corresponding vector in vec and res
    const mjtNum* vec_j = vec + ntotal*j;
    mjtNum* res_j = res + ntotal*j;

    // sparse part
    for(int i=0; i < nsparse; i++) {
      int width = mjMIN(i+1, nband);
      int adr = i*nband + nband - width;
      int offset = mjMAX(0, i-nband+1);
      res_j[i] = mju_dot(mat+adr, vec_j+offset, width);  // lower triangle
      if (flg_sym) {
        // strict upper triangle
        mju_addToScl(res_j+offset, mat+adr, vec_j[i], width-1);
      }
    }

    // dense part
    for(int i=nsparse; i < ntotal; i++) {
      int adr = nsparse*nband + (i-nsparse)*ntotal;
      res_j[i] = mju_dot(mat+adr, vec_j, i+1);
      if (flg_sym) {
        // strict upper triangle
        mju_addToScl(res_j, mat+adr, vec_j[i], i);
      }
    }
  }
}


//------------------------------ LU factorization --------------------------------------------------

// sparse reverse-order LU factorization, no fill-in (assuming tree topology)
//   result: LU = L + U; original = (U+I) * L; scratch size is n
void mju_factorLUSparse(mjtNum* LU, int n, int* scratch,
                        const int* rownnz, const int* rowadr, const int* colind) {
  int* remaining = scratch;

  // set remaining = rownnz
  mju_copyInt(remaining, rownnz, n);

  // diagonal elements (i,i)
  for (int i=n-1; i >= 0; i--) {
    // get address of last remaining element of row i, adjust remaining counter
    int ii = rowadr[i] + remaining[i] - 1;
    remaining[i]--;

    // make sure ii is on diagonal
    if (colind[ii] != i) {
      mjERROR("missing diagonal element");
    }

    // make sure diagonal is not too small
    if (mju_abs(LU[ii]) < mjMINVAL) {
      mjERROR("diagonal element too small");
    }

    // rows j above i
    for (int j=i-1; j >= 0; j--) {
      // get address of last remaining element of row j
      int ji = rowadr[j] + remaining[j] - 1;

      // process row j if (j,i) is non-zero
      if (colind[ji] == i) {
        // adjust remaining counter
        remaining[j]--;

        // (j,i) = (j,i) / (i,i)
        LU[ji] = LU[ji] / LU[ii];
        mjtNum LUji = LU[ji];

        // (j,k) = (j,k) - (i,k) * (j,i) for k<i; handle incompatible sparsity
        int icnt = rowadr[i], jcnt = rowadr[j];
        while (jcnt < rowadr[j]+remaining[j]) {
          // both non-zero
          if (colind[icnt] == colind[jcnt]) {
            // update LU, advance counters
            LU[jcnt++] -= LU[icnt++] * LUji;
          }

          // only (j,k) non-zero
          else if (colind[icnt] > colind[jcnt]) {
            // advance j counter
            jcnt++;
          }

          // only (i,k) non-zero
          else {
            mjERROR("requires fill-in");
          }
        }

        // make sure both rows fully processed
        if (icnt != rowadr[i]+remaining[i] || jcnt != rowadr[j]+remaining[j]) {
          mjERROR("row processing incomplete");
        }
      }
    }
  }

  // make sure remaining points to diagonal
  for (int i=0; i < n; i++) {
    if (remaining[i] < 0 || colind[rowadr[i]+remaining[i]] != i) {
      mjERROR("unexpected sparse matrix structure");
    }
  }
}



// solve mat*res=vec given LU factorization of mat
void mju_solveLUSparse(mjtNum* res, const mjtNum* LU, const mjtNum* vec, int n,
                       const int* rownnz, const int* rowadr, const int* diag, const int* colind) {
  // solve (U+I)*res = vec
  for (int i=n-1; i >= 0; i--) {
    // init: diagonal of (U+I) is 1
    res[i] = vec[i];

    int d1 = diag[i]+1;
    int nnz = rownnz[i] - d1;
    if (nnz > 0) {
      int adr = rowadr[i] + d1;
      res[i] -= mju_dotSparse(LU+adr, res, nnz, colind+adr);
    }
  }

  //------------------ solve L*res(new) = res
  for (int i=0; i < n; i++) {
    // res[i] -= sum_k<i res[k]*LU(i,k)
    int d = diag[i];
    int adr = rowadr[i];
    if (d > 0) {
      res[i] -= mju_dotSparse(LU+adr, res, d, colind+adr);
    }

    // divide by diagonal element of L
    res[i] /= LU[adr + d];
  }
}



//--------------------------- eigen decomposition --------------------------------------------------

// eigenvalue decomposition of symmetric 3x3 matrix
static const mjtNum eigEPS = mjMINVAL * 1000;
int mju_eig3(mjtNum eigval[3], mjtNum eigvec[9], mjtNum quat[4], const mjtNum mat[9]) {
  mjtNum D[9], tmp[9];
  mjtNum tau, t, c;
  int iter, rk, ck, rotk;

  // initialize with unit quaternion
  quat[0] = 1;
  quat[1] = quat[2] = quat[3] = 0;

  // Jacobi iteration
  for (iter=0; iter < 500; iter++) {
    // make quaternion matrix eigvec, compute D = eigvec'*mat*eigvec
    mju_quat2Mat(eigvec, quat);
    mju_mulMatTMat3(tmp, eigvec, mat);
    mju_mulMatMat3(D, tmp, eigvec);

    // assign eigenvalues
    eigval[0] = D[0];
    eigval[1] = D[4];
    eigval[2] = D[8];

    // find max off-diagonal element, set indices
    if (mju_abs(D[1]) > mju_abs(D[2]) && mju_abs(D[1]) > mju_abs(D[5])) {
      rk = 0;     // row
      ck = 1;     // column
      rotk = 2;   // rotation axis
    } else if (mju_abs(D[2]) > mju_abs(D[5])) {
      rk = 0;
      ck = 2;
      rotk = 1;
    } else {
      rk = 1;
      ck = 2;
      rotk = 0;
    }

    // terminate if max off-diagonal element too small
    if (mju_abs(D[3*rk+ck]) < eigEPS) {
      break;
    }

    // 2x2 symmetric Schur decomposition
    tau = (D[4*ck]-D[4*rk])/(2*D[3*rk+ck]);
    if (tau >= 0) {
      t = 1.0/(tau + mju_sqrt(1 + tau*tau));
    } else {
      t = -1.0/(-tau + mju_sqrt(1 + tau*tau));
    }
    c = 1.0/mju_sqrt(1 + t*t);

    // terminate if cosine too close to 1
    if (c > 1.0-eigEPS) {
      break;
    }

    // express rotation as quaternion
    tmp[1] = tmp[2] = tmp[3] = 0;
    tmp[rotk+1] = (tau >= 0 ? -mju_sqrt(0.5-0.5*c) : mju_sqrt(0.5-0.5*c));
    if (rotk == 1) {
      tmp[rotk+1] = -tmp[rotk+1];
    }
    tmp[0] = mju_sqrt(1.0 - tmp[rotk+1]*tmp[rotk+1]);
    mju_normalize4(tmp);

    // accumulate quaternion rotation
    mju_mulQuat(quat, quat, tmp);
    mju_normalize4(quat);
  }

  // sort eigenvalues in decreasing order (bubblesort: 0, 1, 0)
  for (int j=0; j < 3; j++) {
    int j1 = j%2;       // lead index

    // only swap if the eigenvalues are different
    if (eigval[j1]+eigEPS < eigval[j1+1]) {
      // swap eigenvalues
      t = eigval[j1];
      eigval[j1] = eigval[j1+1];
      eigval[j1+1] = t;

      // rotate quaternion
      tmp[0] = 0.707106781186548;  // = cos(pi/4) = sin(pi/4)
      tmp[1] = tmp[2] = tmp[3] = 0;
      tmp[(j1+2)%3+1] = tmp[0];
      mju_mulQuat(quat, quat, tmp);
      mju_normalize4(quat);
    }
  }

  // recompute eigvec
  mju_quat2Mat(eigvec, quat);

  return iter;
}



//---------------------------------- QCQP ----------------------------------------------------------

// solve QCQP in 2 dimensions:
//  min  0.5*x'*A*x + x'*b  s.t.  sum (xi/di)^2 <= r^2
// return 0 if unconstrained, 1 if constrained
int mju_QCQP2(mjtNum* res, const mjtNum* Ain, const mjtNum* bin,
              const mjtNum* d, mjtNum r) {
  mjtNum A11, A22, A12, b1, b2;
  mjtNum P11, P22, P12, det, detinv, v1, v2, la, val, deriv;

  // scale A,b so that constraint becomes x'*x <= r*r
  b1 = bin[0]*d[0];
  b2 = bin[1]*d[1];
  A11 = Ain[0]*d[0]*d[0];
  A22 = Ain[3]*d[1]*d[1];
  A12 = Ain[1]*d[0]*d[1];

  // Newton iteration
  la = 0;
  for (int iter=0; iter < 20; iter++) {
    // det(A+la)
    det = (A11+la)*(A22+la) - A12*A12;

    // check SPD, with 1e-10 threshold
    if (det < 1e-10) {
      res[0] = 0;
      res[1] = 0;
      return 0;
    }

    // P = inv(A+la)
    detinv = 1/det;
    P11 = (A22+la)*detinv;
    P22 = (A11+la)*detinv;
    P12 = -A12*detinv;

    // v = -P*b
    v1 = -P11*b1 - P12*b2;
    v2 = -P12*b1 - P22*b2;

    // val = v'*v - r*r
    val = v1*v1 + v2*v2 - r*r;

    // check for convergence, or initial solution inside constraint set
    if (val < 1e-10) {
      break;
    }

    // deriv = -2 * v' * P * v
    deriv = -2.0*(P11*v1*v1 + 2.0*P12*v1*v2 + P22*v2*v2);

    // compute update, exit if too small
    mjtNum delta = -val/deriv;
    if (delta < 1e-10) {
      break;
    }

    // update
    la += delta;
  }

  // undo scaling
  res[0] = v1*d[0];
  res[1] = v2*d[1];

  return (la != 0);
}



// solve QCQP in 3 dimensions:
//  min  0.5*x'*A*x + x'*b  s.t.  sum (xi/di)^2 <= r^2
// return 0 if unconstrained, 1 if constrained
int mju_QCQP3(mjtNum* res, const mjtNum* Ain, const mjtNum* bin,
              const mjtNum* d, mjtNum r) {
  mjtNum A11, A22, A33, A12, A13, A23, b1, b2, b3;
  mjtNum P11, P22, P33, P12, P13, P23, det, detinv, v1, v2, v3, la, val, deriv;

  // scale A,b so that constraint becomes x'*x <= r*r
  b1 = bin[0]*d[0];
  b2 = bin[1]*d[1];
  b3 = bin[2]*d[2];
  A11 = Ain[0]*d[0]*d[0];
  A22 = Ain[4]*d[1]*d[1];
  A33 = Ain[8]*d[2]*d[2];
  A12 = Ain[1]*d[0]*d[1];
  A13 = Ain[2]*d[0]*d[2];
  A23 = Ain[5]*d[1]*d[2];

  // Newton iteration
  la = 0;
  for (int iter=0; iter < 20; iter++) {
    // unscaled P
    P11 = (A22+la)*(A33+la) - A23*A23;
    P22 = (A11+la)*(A33+la) - A13*A13;
    P33 = (A11+la)*(A22+la) - A12*A12;
    P12 = A13*A23 - A12*(A33+la);
    P13 = A12*A23 - A13*(A22+la);
    P23 = A12*A13 - A23*(A11+la);

    // det(A+la)
    det = (A11+la)*P11 + A12*P12 + A13*P13;

    // check SPD, with 1e-10 threshold
    if (det < 1e-10) {
      res[0] = 0;
      res[1] = 0;
      res[2] = 0;
      return 0;
    }

    // detinv
    detinv = 1/det;

    // final P
    P11 *= detinv;
    P22 *= detinv;
    P33 *= detinv;
    P12 *= detinv;
    P13 *= detinv;
    P23 *= detinv;

    // v = -P*b
    v1 = -P11*b1 - P12*b2 - P13*b3;
    v2 = -P12*b1 - P22*b2 - P23*b3;
    v3 = -P13*b1 - P23*b2 - P33*b3;

    // val = v'*v - r*r
    val = v1*v1 + v2*v2 + v3*v3 - r*r;

    // check for convergence, or initial solution inside constraint set
    if (val < 1e-10) {
      break;
    }

    // deriv = -2 * v' * P * v
    deriv = -2.0*(P11*v1*v1 + P22*v2*v2 + P33*v3*v3)
            -4.0*(P12*v1*v2 + P13*v1*v3 + P23*v2*v3);

    // compute update, exit if too small
    mjtNum delta = -val/deriv;
    if (delta < 1e-10) {
      break;
    }

    // update
    la += delta;
  }

  // undo scaling
  res[0] = v1*d[0];
  res[1] = v2*d[1];
  res[2] = v3*d[2];

  return (la != 0);
}



// solve QCQP in n dimensions:
//  min  0.5*x'*A*x + x'*b  s.t.  sum (xi/di)^2 <= r^2
// return 0 if unconstrained, 1 if constrained
int mju_QCQP(mjtNum* res, const mjtNum* Ain, const mjtNum* bin,
             const mjtNum* d, mjtNum r, int n) {
  mjtNum A[25], Ala[25], b[5];
  mjtNum la, val, deriv, tmp[5];

  // check size
  if (n > 5) {
    mjERROR("n is only supported up to 5");
  }

  // scale A,b so that constraint becomes x'*x <= r*r
  for (int i=0; i < n; i++) {
    b[i] = bin[i] * d[i];

    for (int j=0; j < n; j++) {
      A[j+i*n] = Ain[j+i*n] * d[i] * d[j];
    }
  }

  // Newton iteration
  la = 0;
  for (int iter=0; iter < 20; iter++) {
    // make A+la
    mju_copy(Ala, A, n*n);
    for (int i=0; i < n; i++) {
      Ala[i*(n+1)] += la;
    }

    // factorize, check rank with 1e-10 threshold
    if (mju_cholFactor(Ala, n, 1e-10) < n) {
      mju_zero(res, n);
      return 0;
    }

    // set res = -Ala \ b
    mju_cholSolve(res, Ala, b, n);
    mju_scl(res, res, -1, n);

    // val = b' * Ala^-2 * b - r*r
    val = mju_dot(res, res, n) - r*r;

    // check for convergence, or initial solution inside constraint set
    if (val < 1e-10) {
      break;
    }

    // deriv = -2 * b' * Ala^-3 * b
    mju_cholSolve(tmp, Ala, res, n);
    deriv = -2.0 * mju_dot(res, tmp, n);

    // compute update, exit if too small
    mjtNum delta = -val/deriv;
    if (delta < 1e-10) {
      break;
    }

    // update
    la += delta;
  }

  // undo scaling
  for (int i=0; i < n; i++) {
    res[i] = res[i] * d[i];
  }

  return (la != 0);
}



//--------------------------- box-constrained quadratic program ------------------------------------

// minimize 0.5*x'*H*x + x'*g  s.t. lower <= x <= upper, return rank or -1 if failed
//   inputs:
//     n           - problem dimension
//     H           - SPD matrix                n*n
//     g           - bias vector               n
//     lower       - lower bounds              n
//     upper       - upper bounds              n
//     res         - solution warmstart        n
//   return value:
//     nfree <= n  - rank of unconstrained subspace, -1 if failure
//   outputs (required):
//     res         - solution                  n
//     R           - subspace Cholesky factor  nfree*nfree    allocated: n*(n+7)
//   outputs (optional):
//     index       - set of free dimensions    nfree          allocated: n
//   notes:
//     the initial value of res is used to warmstart the solver
//     R must have allocatd size n*(n+7), but only nfree*nfree values are used in output
//     index (if given) must have allocated size n, but only nfree values are used in output
//     only lower triangles of H and R and read from and written to, respectively
int mju_boxQP(mjtNum* res, mjtNum* R, int* index,
              const mjtNum* H, const mjtNum* g, int n,
              const mjtNum* lower, const mjtNum* upper) {
  // algorithm options
  int    maxiter    = 100;    // maximum number of iterations
  mjtNum mingrad    = 1E-16;  // minimum squared norm of (unclamped) gradient
  mjtNum backtrack  = 0.5;    // backtrack factor for decreasing stepsize
  mjtNum minstep    = 1E-22;  // minimum stepsize for linesearch
  mjtNum armijo     = 0.1;    // Armijo parameter (fraction of expected linear improvement)

  // logging (disabled)
  char*  log        = NULL;   // buffer to write log messages into
  int    logsz      = 0;      // size of log buffer

  return mju_boxQPoption(res, R, index, H, g, n, lower, upper,
                         maxiter, mingrad, backtrack, minstep, armijo, log, logsz);
}



// allocate heap memory for box-constrained Quadratic Program
//   as in mju_boxQP, index, lower and upper are optional
//   free all pointers with mju_free()
void mju_boxQPmalloc(mjtNum** res, mjtNum** R, int** index,
                     mjtNum** H, mjtNum** g, int n,
                     mjtNum** lower, mjtNum** upper) {
  // required arrays
  *res = (mjtNum*) mju_malloc(sizeof(mjtNum)*n);
  *R   = (mjtNum*) mju_malloc(sizeof(mjtNum)*n*(n+7));
  *H   = (mjtNum*) mju_malloc(sizeof(mjtNum)*n*n);
  *g   = (mjtNum*) mju_malloc(sizeof(mjtNum)*n);

  // optional arrays
  if (lower) *lower = (mjtNum*) mju_malloc(sizeof(mjtNum)*n);
  if (upper) *upper = (mjtNum*) mju_malloc(sizeof(mjtNum)*n);
  if (index) *index = (int*) mju_malloc(sizeof(int)*n);
}



// local enum encoding mju_boxQP solver status (purely for readability)
enum mjtStatusBoxQP {
  mjBOXQP_NOT_SPD     = -1,        //  Hessian is not positive definite
  mjBOXQP_NO_DESCENT  = 0,         //  no descent direction found
  mjBOXQP_MAX_ITER    = 1,         //  maximum main iterations exceeded
  mjBOXQP_MAX_LS_ITER = 2,         //  maximum line-search iterations exceeded
  mjBOXQP_TOL_GRAD    = 3,         //  gradient norm smaller than tolerance
  mjBOXQP_UNBOUNDED   = 4,         //  no dimensions clamped, returning Newton point
  mjBOXQP_ALL_CLAMPED = 5,         //  all dimensions clamped

  mjNBOXQP            = 7          //  number of boxQP status values
};


// multiply symmetric matrix with vector on both sides: return vec'*mat*vec
//   assumes symmetry of mat, ignores upper triangle
static mjtNum mulVecMatVecSym(const mjtNum* vec, const mjtNum* mat, int n) {
  mjtNum res = 0;
  for (int i=0; i < n; i++) {
    res += vec[i] * mat[n*i+i] * vec[i];           // diagonal
    res += 2 * vec[i] * mju_dot(mat+n*i, vec, i);  // off-diagonal
  }
  return res;
}


// minimize 0.5*x'*H*x + x'*g  s.t. lower <= x <=upper, explicit options
//  additional arguments to mju_boxQP (see mju_boxQP documentation):
//   maxiter     maximum number of iterations
//   mingrad     minimum squared norm of (unclamped) gradient
//   backtrack   backtrack factor for decreasing stepsize
//   minstep     minimum stepsize for linesearch
//   armijo      Armijo parameter (fraction of expected linear improvement)
//   log         buffer to write log messages into
//   logsz       size of log buffer
int mju_boxQPoption(mjtNum* res, mjtNum* R, int* index,               // outputs
                    const mjtNum* H, const mjtNum* g, int n,          // QP definition
                    const mjtNum* lower, const mjtNum* upper,         // bounds
                    int maxiter, mjtNum mingrad, mjtNum backtrack,    // options
                    mjtNum minstep, mjtNum armijo,                    // options
                    char* log, int logsz)                             // logging
{
  int status = mjBOXQP_NO_DESCENT;  // initial status: no descent direction found
  int factorize = 1;                // always factorize on the first iteration
  int nfree = n;                    // initialise nfree with n
  int nfactor = 0;
  mjtNum sdotg, improvement=0, value=0, norm2=0;

  // basic checks
  if (n <= 0) {
    mjERROR("problem size n must be positive");
  }
  if (upper && lower) {
    for (int i=0; i < n; i++) {
      if (lower[i] >= upper[i]) {
        mjERROR("upper bounds must be stricly larger than lower bounds");
      }
    }
  }

  // local scratch vectors, allocate in R
  mjtNum* scratch   = R + n*n;
  mjtNum* grad      = scratch + 0*n;
  mjtNum* search    = scratch + 1*n;
  mjtNum* candidate = scratch + 2*n;
  mjtNum* temp      = scratch + 3*n;
  int* clamped      = (int*) (scratch + 4*n);
  int* oldclamped   = (int*) (scratch + 5*n);

  // if index vector not given, use scratch space
  if (!index) {
    index = (int*) (scratch + 6*n);
  }

  static const char status_string[mjNBOXQP][50]= {
    "Hessian is not positive definite",
    "No descent direction found",
    "Maximum main iterations exceeded",
    "Maximum line-search iterations exceeded",
    "Gradient norm smaller than tolerance",
    "No dimensions clamped, returning Newton point",
    "All dimensions clamped"
  };

  // no bounds: return Newton point
  if (!lower && !upper) {
    // try to factorize
    mju_copy(R, H, n*n);
    int rank = mju_cholFactor(R, n, mjMINVAL);
    if (rank == n) {
      mju_cholSolve(res, R, g, n);
      mju_scl(res, res, -1, n);
      nfactor = 1;
      status = mjBOXQP_UNBOUNDED;
    } else {
      status = mjBOXQP_NOT_SPD;
    }

    // full index set (no clamping)
    for (int i=0; i < n; i++) {
      index[i] = i;
    }
  }

  // have bounds: clamp res
  else {
    for (int i=0; i < n; i++) {
      if (lower) {
        res[i] = mju_max(res[i], lower[i]);
      }
      if (upper) {
        res[i] = mju_min(res[i], upper[i]);
      }
    }
  }

  // ------ main loop
  int iter, logptr = 0;
  mjtNum oldvalue;
  for (iter=0; iter < maxiter; iter++) {
    if (status != mjBOXQP_NO_DESCENT) {
      break;
    }

    // compute objective: value = 0.5*res'*H*res + res'*g
    value = 0.5 * mulVecMatVecSym(res, H, n) + mju_dot(res, g, n);

    // save last value
    oldvalue = value;

    // compute gradient
    mju_mulMatVec(grad, H, res, n, n);
    mju_addTo(grad, g, n);

    // find clamped dimensions
    for (int i=0; i < n; i++) {
      clamped[i] = ( lower && res[i] == lower[i] && grad[i] > 0 ) ||
                   ( upper && res[i] == upper[i] && grad[i] < 0 );
    }

    // build index of free dimensions, count them
    nfree = 0;
    for (int i=0; i < n; i++) {
      if (!clamped[i]) {
        index[nfree++] = i;
      }
    }

    // all dimensions are clamped: minimum found
    if (!nfree) {
      status = mjBOXQP_ALL_CLAMPED;
      break;
    }

    // re-factorize if clamped dimensions have changed
    if (iter) {
      factorize = 0;
      for (int i=0; i < n; i++) {
        if (clamped[i] != oldclamped[i]) {
          factorize = 1;
          break;
        }
      }
    }

    // save last clamped
    for (int i=0; i < n; i++) {
      oldclamped[i] = clamped[i];
    }

    // get search direction: search = g + H_all,clamped * res_clamped
    for (int i=0; i < n; i++) {
      temp[i] = clamped[i] ? res[i] : 0;
    }
    mju_mulMatVec(search, H, temp, n, n);
    mju_addTo(search, g, n);

    // search = compress_free(search)
    for (int i=0; i < nfree; i++) {
      search[i] = search[index[i]];
    }

    // R = compress_free(H)
    if (factorize) {
      for (int i=0; i < nfree; i++) {
        for (int j=0; j < i+1; j++) {
          R[i*nfree+j] = H[index[i]*n+index[j]];
        }
      }
    }

    // re-factorize and increment counter, if required
    int rank = factorize ? mju_cholFactor(R, nfree, mjMINVAL) : nfree;
    nfactor += factorize;

    // abort if factorization failed
    if (rank != nfree) {
      status = mjBOXQP_NOT_SPD;
      break;
    }

    // temp = H_free,free \ search_free
    mju_cholSolve(temp, R, search, nfree);

    // search_free = expand_free(-temp) - x_free
    mju_zero(search, n);
    for (int i=0; i < nfree; i++) {
      search[index[i]] = -temp[i] -res[index[i]];
    }

    // ------ check gradient

    // squared norm of free gradient
    norm2 = 0;
    for (int i=0; i < nfree; i++) {
      mjtNum grad_i = grad[index[i]];
      norm2 += grad_i*grad_i;
    }

    // small gradient: minimum found
    if (norm2 < mingrad) {
      status = nfree == n ? mjBOXQP_UNBOUNDED : mjBOXQP_TOL_GRAD;
      break;
    }

    // sanity check: make sure we have a descent direction
    if ((sdotg = mju_dot(search, grad, n)) >= 0) {
      break;  // SHOULD NOT OCCUR
    }

    // ------ projected Armijo line search
    mjtNum step = 1;
    int nstep = 0;
    do {
      // candidate = clamp(x + step*search)
      mju_scl(candidate, search, step, n);
      mju_addTo(candidate, res, n);
      for (int i=0; i < n; i++) {
        if (lower && candidate[i] < lower[i]) {
          candidate[i] = lower[i];
        } else if (upper && candidate[i] > upper[i]) {
          candidate[i] = upper[i];
        }
      }

      // new objective value
      value = 0.5 * mulVecMatVecSym(candidate, H, n) + mju_dot(candidate, g, n);

      // increment and break if step is too small
      nstep++;
      step = step*backtrack;
      if (step < minstep) {
        status = mjBOXQP_MAX_LS_ITER;
        break;
      }

      // repeat until relative improvement >= Armijo
      improvement = (value - oldvalue) / (step*sdotg);
    } while (improvement < armijo);


    // print iteration info
    if (log) {
      logptr += snprintf(log+logptr, logsz-logptr,
                         "iter %-3d:  |grad|: %-8.2g  reduction: %-8.2g  improvement: %-8.4g  "
                         "linesearch: %g^%-2d  factorized: %d  nfree: %d\n",
                         iter+1, mju_sqrt(norm2), oldvalue-value, improvement,
                         backtrack, nstep-1, factorize, nfree);
    }

    // accept candidate
    mju_copy(res, candidate, n);
  }

  // max iterations exceeded
  if (iter == maxiter) {
    status = mjBOXQP_MAX_ITER;
  }

  // print final info
  if (log) {
    snprintf(log+logptr, logsz-logptr, "BOXQP: %s.\n"
             "iterations= %d,  factorizations= %d,  |grad|= %-12.6g, final value= %-12.6g\n",
             status_string[status+1], iter, nfactor, mju_sqrt(norm2), value);
  }

  // return nf or -1 if failure
  return (status == mjBOXQP_NO_DESCENT || status == mjBOXQP_NOT_SPD) ? -1 : nfree;
}
