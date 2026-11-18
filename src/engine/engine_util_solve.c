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
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_crossplatform.h"
#include "engine/engine_inline.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_memory.h"
#include "engine/engine_util_spatial.h"

//---------------------------- dense Cholesky ------------------------------------------------------

// Cholesky decomposition: mat = L*L'; return 'rank'
int mju_cholFactor(mjtNum* restrict mat, int n, mjtNum mindiag) {
  int rank = n;
  mjtNum tmp;

  // in-place Cholesky factorization
  for (int j=0; j < n; j++) {
    // compute new diagonal
    tmp = mat[j*(n+1)];
    if (j) {
      tmp -= mju_dot(mat+j*n, mat+j*n, j);
    }

    // correct diagonal values below threshold (negated >= so NaN counts as deficient)
    int deficient = !(tmp >= mindiag);
    if (deficient) {
      tmp = mindiag;
      rank--;
    }

    // save diagonal
    mat[j*(n+1)] = mju_sqrt(tmp);

    // process off-diagonal entries
    if (deficient) {
      // clear off-diagonals if deficient
      for (int i=j+1; i < n; i++) {
        mat[i*n+j] = 0;
      }
    } else {
      tmp = 1/mat[j*(n+1)];
      for (int i=j+1; i < n; i++) {
        mat[i*n+j] = (mat[i*n+j] - mju_dot(mat+i*n, mat+j*n, j)) * tmp;
      }
    }
  }

  return rank;
}


// Cholesky solve
void mju_cholSolve(mjtNum* res, const mjtNum* restrict mat, const mjtNum* vec, int n) {
  // copy if source and destination are different
  if (res != vec) {
    mju_copy(res, vec, n);
  }

  // forward substitution: solve L*res = vec
  for (int i=0; i < n; i++) {
    // off-diagonal
    if (i) {
      res[i] -= mju_dot(mat+i*n, res, i);
    }

    // diagonal
    res[i] /= mat[i*(n+1)];
  }

  // backward substitution: solve L'*res = res
  for (int i=n-1; i >= 0; i--) {
    // diagonal
    res[i] /= mat[i*(n+1)];

    // update res[0..i-1] with row i of L
    if (i && res[i]) {
      mju_addToScl(res, mat+i*n, -res[i], i);
    }
  }
}


// Cholesky rank-one update: L*L' +/- x*x'; return rank
int mju_cholUpdate(mjtNum* restrict mat, mjtNum* restrict x, int n, int flg_plus) {
  int rank = n;
  mjtNum r, c, cinv, s, Lkk, tmp;

  for (int k=0; k < n; k++) {
    if (x[k]) {
      // prepare constants
      Lkk = mat[k*(n+1)];
      tmp = Lkk*Lkk + (flg_plus ? x[k]*x[k] : -x[k]*x[k]);
      if (!(tmp >= mjMINVAL)) {
        tmp = mjMINVAL;
        rank--;
      }
      r = mju_sqrt(tmp);
      c = r / Lkk;
      cinv = 1 / c;
      s = x[k] / Lkk;

      // update diagonal
      mat[k*(n+1)] = r;

      // update mat and x
      mjtNum s_signed = flg_plus ? s : -s;
      for (int i=k+1; i < n; i++) {
        mjtNum m = (mat[i*n+k] + s_signed*x[i])*cinv;
        mat[i*n+k] = m;
        x[i] = c*x[i] - s*m;
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
  (void) d;

  // backpass over rows
  for (int r=n-1; r >= 0; r--) {
    // get rownnz and rowadr for row r
    int nnz = rownnz[r], adr = rowadr[r];

    // update row r diagonal
    mjtNum tmp = mat[adr+nnz-1];
    int deficient = !(tmp >= mindiag);
    if (deficient) {
      tmp = mindiag;
      rank--;
    }
    mat[adr+nnz-1] = mju_sqrt(tmp);

    // clear off-diagonals if deficient
    if (deficient) {
      mju_zero(mat+adr, nnz-1);
    } else {
      tmp = 1/mat[adr+nnz-1];

      // update row r before diagonal
      for (int i=0; i < nnz-1; i++) {
        mat[adr+i] *= tmp;
      }
    }

    // update row c<r where mat(r,c)!=0 (zero coefficients still merge the pattern)
    for (int i=0; i < nnz-1; i++) {
      // get column index
      int c = colind[adr+i];

      // mat(c,0:c) = mat(c,0:c) - mat(r,c) * mat(r,0:c)
      int nnz_c = mju_combineSparse(mat + rowadr[c], mat+rowadr[r], 1, -mat[adr+i],
                                    rownnz[c], i+1, colind+rowadr[c], colind+rowadr[r]);

      // assign new nnz to row c
      rownnz[c] = nnz_c;
    }
  }

  return rank;
}


// return index of first upper-triangular column (> r) in sorted row colind[start .. end-1]
static inline int upperStart(const int* colind, int start, int end, int r) {
  if (start < end && colind[start] <= r) {
    if (colind[start] == r) {
      return start + 1;
    }
    int lo = start + 1, hi = end;
    while (lo < hi) {
      int mid = (lo + hi) >> 1;
      if (colind[mid] <= r) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    return lo;
  }
  return start;
}


// symbolic reverse-Cholesky: compute both L (CSR) and LT (CSC) structures
//   if L_colind is NULL, perform counting logic (fill rownnz/rowadr arrays and return total nnz)
//   if L_colind is not NULL, assume rownnz/rowadr are precomputed and fill colind/map arrays
//   reads pattern from upper triangle
//   based on ldl_symbolic from 'Algorithm 8xx: a concise sparse Cholesky factorization package'
int mju_cholFactorSymbolic(int* restrict L_colind, int* restrict L_rownnz, int* restrict L_rowadr,
                           int* restrict LT_colind, int* restrict LT_rownnz,
                           int* restrict LT_rowadr, int* restrict LT_map, int* restrict L_super,
                           const int* restrict rownnz, const int* restrict rowadr,
                           const int* restrict colind, int n, mjData* d) {
  // d supplies stack scratch; if NULL, scratch is heap-allocated
  if (d) {
    mj_markStack(d);
  }
  int* restrict parent = d ? mjSTACKALLOC(d, n, int) : (int*) mju_malloc(sizeof(int)*n);
  int* restrict flag = d ? mjSTACKALLOC(d, n, int) : (int*) mju_malloc(sizeof(int)*n);

  // counting phase: fill rownnz/rowadr and return total non-zeros
  if (!L_colind) {
    // loop over rows in reverse order
    for (int r=n-1; r >= 0; r--) {
      parent[r] = -1;
      flag[r] = r;

      // start with 1 for diagonal
      L_rownnz[r] = 1;
      int lt_nnz = 1;

      // loop over non-zero columns of upper triangle (> r)
      int end = rowadr[r] + rownnz[r];
      for (int c=upperStart(colind, rowadr[r], end, r); c < end; c++) {
        int i = colind[c];

        // traverse elimination tree from i toward root, stop when row is already flagged for r
        while (flag[i] != r) {
          // count non-zero in L[i, r] and LT[r, i], mark visited, attach unparented root to r
          L_rownnz[i]++;
          lt_nnz++;
          flag[i] = r;
          int p = parent[i];
          if (p == -1) {
            parent[i] = r;
            break;
          }
          i = p;
        }
      }
      LT_rownnz[r] = lt_nnz;
    }

    if (d) {
      mj_freeStack(d);
    } else {
      mju_free(parent);
      mju_free(flag);
    }

    // compute row addresses, add up total non-zeros
    int nnz = L_rownnz[0];
    L_rowadr[0] = 0;
    LT_rowadr[0] = 0;
    for (int r=1; r < n; r++) {
      L_rowadr[r] = L_rowadr[r-1] + L_rownnz[r-1];
      LT_rowadr[r] = LT_rowadr[r-1] + LT_rownnz[r-1];
      nnz += L_rownnz[r];
    }
    return nnz;
  }

  // filling phase: populate L_colind, LT_colind, LT_map and L_super
  // cursor[i] tracks next write slot in row i of L (filled right-to-left as r decreases)
  int* restrict cursor = d ? mjSTACKALLOC(d, n, int) : (int*) mju_malloc(sizeof(int)*n);

  // loop over rows in reverse order
  for (int r=n-1; r >= 0; r--) {
    parent[r] = -1;
    flag[r] = r;

    // write L diagonal at end of row r and initialize write cursor just before diagonal
    int diag_idx = L_rowadr[r] + L_rownnz[r] - 1;
    cursor[r] = diag_idx - 1;
    L_colind[diag_idx] = r;

    // write LT diagonal at head of transpose row r
    int lt_write = LT_rowadr[r];
    LT_colind[lt_write] = r;
    LT_map[lt_write] = diag_idx;
    lt_write++;

    // loop over non-zero columns of upper triangle (> r)
    int end = rowadr[r] + rownnz[r];
    int overlap = 0;
    for (int c=upperStart(colind, rowadr[r], end, r); c < end; c++) {
      int i = colind[c];

      // traverse elimination tree from i toward root, stop when row is already flagged for r
      while (flag[i] != r) {
        // count rows of LT[r+1] (still flagged r+1) that are also in LT[r]
        overlap += (flag[i] == r + 1);

        // record column r in L[i] and row i (with its L index) in LT[r]
        int L_idx = cursor[i]--;
        L_colind[L_idx] = r;
        LT_colind[lt_write] = i;
        LT_map[lt_write] = L_idx;
        lt_write++;

        // mark row i visited for r, advance to parent (or attach unparented root to r)
        flag[i] = r;
        int p = parent[i];
        if (p == -1) {
          parent[i] = r;
          break;
        }
        i = p;
      }
    }

    // rows r+1 and r merge iff r+1 is a child of r with the same remaining columns in L, and
    // LT[r] = LT[r+1] + {r}: equal counts alone do not imply equal descendant rows
    if (r < n-1) {
      L_super[r+1] = (parent[r+1] == r &&
                      L_rownnz[r] == L_rownnz[r+1] - 1 &&
                      LT_rownnz[r] == LT_rownnz[r+1] + 1 &&
                      overlap == LT_rownnz[r+1]);
    }
  }

  // supernode sizes: L_super[r] = number of rows r, r-1, ... (up to 3) that merge
  if (n > 0) {
    L_super[0] = 0;
  }
  for (int r=n-1; r >= 0; r--) {
    int merge = L_super[r];
    L_super[r] = 1 + merge + (merge && r > 0 && L_super[r-1]);
  }

  if (d) {
    mj_freeStack(d);
  } else {
    mju_free(parent);
    mju_free(flag);
    mju_free(cursor);
  }

  return 0;
}


// symbolic reverse-Cholesky of a matrix of dense bs x bs blocks, from its block pattern
//   mju_cholFactorSymbolic on the expanded pattern walks the dofs of each block together: the
//   rows of a block share their block columns, and the dof elimination tree is the block
//   elimination tree with each block a chain. So the walks run on the blocks, each visit filling
//   one block slot of the rows of L and one entry of the block transpose
int mju_cholFactorSymbolicBlocked(int* restrict L_colind, int* restrict L_rownnz,
                                  int* restrict L_rowadr, int* restrict BT_adr,
                                  int* restrict BT_row, int* restrict BT_off, const int* rownnz,
                                  const int* rowadr, const int* colind, int nb, int bs,
                                  int* scratch) {
  int* restrict parent = scratch;          // block elimination tree
  int* restrict flag = scratch + nb;       // the last block column whose walk reached the block
  int* restrict count = scratch + 2*nb;    // counting: block columns of the block's rows
                                           // filling: those still to fill, right to left

  // counting phase: the block walks, then the dof row sizes and addresses
  if (!L_colind) {
    for (int q = nb - 1; q >= 0; q--) {
      parent[q] = -1;
      flag[q] = q;
      count[q] = 0;
      int ncol = 0;
      int end = rowadr[q] + rownnz[q];
      for (int j = upperStart(colind, rowadr[q], end, q); j < end; j++) {
        int p = colind[j];
        while (flag[p] != q) {
          flag[p] = q;
          count[p]++;
          ncol++;
          int par = parent[p];
          if (par == -1) {
            parent[p] = q;
            break;
          }
          p = par;
        }
      }
      BT_adr[q] = ncol;   // block rows its column reaches, accumulated below
    }
    int nnz = 0, nnzT = 0;
    for (int p = 0; p < nb; p++) {
      int ncol = BT_adr[p];
      BT_adr[p] = nnzT;
      nnzT += ncol;
      for (int a = 0; a < bs; a++) {
        int r = bs*p + a;
        L_rownnz[r] = bs*count[p] + a + 1;
        L_rowadr[r] = nnz;
        nnz += L_rownnz[r];
      }
    }
    BT_adr[nb] = nnzT;
    return nnz;
  }

  // filling phase: the diagonal block and cursor of q, then the walk from its upper columns
  for (int q = nb - 1; q >= 0; q--) {
    int x = BT_adr[q];
    parent[q] = -1;
    flag[q] = q;
    count[q] = (L_rownnz[bs*q] - 1) / bs;
    int off = bs*count[q];
    for (int c = 0; c < bs; c++) {
      for (int b = 0; b <= c; b++) {
        L_colind[L_rowadr[bs*q + c] + off + b] = bs*q + b;
      }
    }
    int end = rowadr[q] + rownnz[q];
    for (int j = upperStart(colind, rowadr[q], end, q); j < end; j++) {
      int p = colind[j];
      while (flag[p] != q) {
        flag[p] = q;
        int ip = --count[p];
        for (int c = 0; c < bs; c++) {
          int* col = L_colind + L_rowadr[bs*p + c] + bs*ip;
          for (int b = 0; b < bs; b++) {
            col[b] = bs*q + b;
          }
        }
        BT_row[x] = p;
        BT_off[x++] = bs*ip;
        int par = parent[p];
        if (par == -1) {
          parent[p] = q;
          break;
        }
        p = par;
      }
    }
  }
  return 0;
}


// suffix update of ns target rows from batch descendant rows (ns, batch are compile-time constants)
static mjALWAYS_INLINE
void cholSuffixUpdate(int ns, int batch, int nlead,
                      mjtNum* restrict dptr0, mjtNum* restrict dptr1, mjtNum* restrict dptr2,
                      const mjtNum* restrict p0, const mjtNum* restrict p1,
                      const mjtNum* restrict p2, const mjtNum* restrict p3,
                      mjtNum s[3][4]) {
  // leading columns shared by all ns rows
  for (int i=0; i < nlead; i++) {
    mjtNum v[4] = {p0[i], (batch >= 2) ? p1[i] : 0,
                   (batch >= 3) ? p2[i] : 0, (batch >= 4) ? p3[i] : 0};
    for (int b=0; b < batch; b++) dptr0[i] -= s[0][b] * v[b];
    if (ns >= 2) {
      for (int b=0; b < batch; b++) dptr1[i] -= s[1][b] * v[b];
    }
    if (ns >= 3) {
      for (int b=0; b < batch; b++) dptr2[i] -= s[2][b] * v[b];
    }
  }

  // triangular tail: column nlead+u is present in rows t <= ns-2-u
  for (int u=0; u < ns-1; u++) {
    int idx = nlead + u;
    mjtNum v[4] = {p0[idx], (batch >= 2) ? p1[idx] : 0,
                   (batch >= 3) ? p2[idx] : 0, (batch >= 4) ? p3[idx] : 0};
    for (int b=0; b < batch; b++) dptr0[idx] -= s[0][b] * v[b];
    if (u == 0 && ns >= 3) {
      for (int b=0; b < batch; b++) dptr1[idx] -= s[1][b] * v[b];
    }
  }
}


// non-suffix mapped scatter of batch descendant rows directly into supernode L rows
//  (ns, batch are compile-time constants)
static mjALWAYS_INLINE
void cholMappedUpdate(int ns, int batch, int nlead, int L_nnz,
                      const int* restrict pos_map, const int* restrict colptr,
                      mjtNum* restrict dptr0, mjtNum* restrict dptr1, mjtNum* restrict dptr2,
                      const mjtNum* restrict p0, const mjtNum* restrict p1,
                      const mjtNum* restrict p2, const mjtNum* restrict p3,
                      mjtNum s[3][4]) {
  // leading columns shared by all ns rows at identical pos_map[colptr[i]]
  for (int i=0; i < nlead; i++) {
    int pos = pos_map[colptr[i]];
    mjtNum v[4] = {p0[i], (batch >= 2) ? p1[i] : 0,
                   (batch >= 3) ? p2[i] : 0, (batch >= 4) ? p3[i] : 0};
    mjtNum a0 = dptr0[pos];
    for (int b=0; b < batch; b++) a0 -= s[0][b] * v[b];
    dptr0[pos] = a0;
    if (ns >= 2) {
      mjtNum a1 = dptr1[pos];
      for (int b=0; b < batch; b++) a1 -= s[1][b] * v[b];
      dptr1[pos] = a1;
    }
    if (ns >= 3) {
      mjtNum a2 = dptr2[pos];
      for (int b=0; b < batch; b++) a2 -= s[2][b] * v[b];
      dptr2[pos] = a2;
    }
  }

  // triangular tail: column r - (ns - 2 - u) sits at offset L_nnz - ns + 1 + u
  for (int u=0; u < ns-1; u++) {
    int idx = nlead + u;
    int pos = L_nnz - ns + 1 + u;
    mjtNum v[4] = {p0[idx], (batch >= 2) ? p1[idx] : 0,
                   (batch >= 3) ? p2[idx] : 0, (batch >= 4) ? p3[idx] : 0};
    mjtNum a0 = dptr0[pos];
    for (int b=0; b < batch; b++) a0 -= s[0][b] * v[b];
    dptr0[pos] = a0;
    if (u == 0 && ns >= 3) {
      mjtNum a1 = dptr1[pos];
      for (int b=0; b < batch; b++) a1 -= s[1][b] * v[b];
      dptr1[pos] = a1;
    }
  }
}


// initialize supernode L rows from H when fill-in is present (outlined from hot no-fill path)
static mjNOINLINE
void cholInitMappedH(int ns, int r, int s_start, int L_nnz, const int* restrict L_colptr,
                     mjtNum* restrict Lptr0, mjtNum* restrict Lptr1, mjtNum* restrict Lptr2,
                     const mjtNum* restrict H, const int* restrict H_rownnz,
                     const int* restrict H_rowadr, const int* restrict H_colind,
                     int* restrict mapped, int* restrict pos_map) {
  mjtNum* restrict Lptr_s[3] = {Lptr0, Lptr1, Lptr2};
  if (!*mapped) {
    for (int i=0; i < L_nnz; i++) {
      pos_map[L_colptr[i]] = i;
    }
    *mapped = 1;
  }
  for (int s=s_start; s < ns; s++) {
    int row = r - s;
    int nnz_s = L_nnz - s;
    int h_adr = H_rowadr[row];
    int h_nnz = H_rownnz[row];
    mjtNum* restrict row_s = Lptr_s[s];
    if (h_nnz >= nnz_s && H_colind[h_adr+nnz_s-1] == row) {
      mju_copy(row_s, H+h_adr, nnz_s);
    } else {
      mju_zero(row_s, nnz_s);
      for (int i=0; i < h_nnz; i++) {
        int c = H_colind[h_adr+i];
        if (c > row) break;
        row_s[pos_map[c]] = H[h_adr+i];
      }
    }
  }
}


// process consecutive non-suffix descendant rows in LT[r] (ns is a compile-time constant)
static mjALWAYS_INLINE
int cholMappedRunImpl(int ns, int k, int LT_nnz, int LT_adr, int L_nnz,
                      const int* restrict L_colptr,
                      const mjtNum* restrict L, const int* restrict L_rownnz,
                      const int* restrict L_rowadr, const int* restrict L_colind,
                      const int* restrict LT_colind, const int* restrict LT_map,
                      mjtNum* restrict Lptr0, mjtNum* restrict Lptr1, mjtNum* restrict Lptr2,
                      int* restrict mapped, int* restrict pos_map) {
  if (!*mapped) {
    for (int i=0; i < L_nnz; i++) {
      pos_map[L_colptr[i]] = i;
    }
    *mapped = 1;
  }

  const int max_batch = (ns == 3) ? 3 : 4;
  do {
    int c0 = LT_colind[LT_adr+k];
    int idx0 = LT_map[LT_adr+k];
    int c0_adr = L_rowadr[c0];
    int num_cols = idx0 - c0_adr + 1;
    const int* restrict colptr = L_colind + c0_adr;
    if (L_colptr[L_nnz-num_cols] == colptr[0]) {
      break;
    }

    const mjtNum* p[4] = {L + c0_adr, NULL, NULL, NULL};
    int idx[4] = {idx0, 0, 0, 0};
    int batch = 1;
    int prev_c = c0, prev_adr = c0_adr;

    while (k + batch < LT_nnz && batch < max_batch) {
      int next_c = LT_colind[LT_adr+k+batch];
      int next_idx = LT_map[LT_adr+k+batch];
      int next_adr = L_rowadr[next_c];
      if (next_idx - next_adr + 1 != num_cols) break;
      int hi = next_c > prev_c ? next_c : prev_c;
      int lo = next_c > prev_c ? prev_c : next_c;
      int hi_adr = next_c > prev_c ? next_adr : prev_adr;
      int lo_nnz = L_rownnz[lo];
      if (lo_nnz >= L_rownnz[hi] || L_colind[hi_adr+lo_nnz-1] != lo) break;
      prev_c = next_c;
      prev_adr = next_adr;
      p[batch] = L + next_adr;
      idx[batch] = next_idx;
      batch++;
    }

    int nlead = num_cols - (ns - 1);
    mjtNum s[3][4];
    for (int t=0; t < ns; t++) {
      for (int b=0; b < batch; b++) {
        s[t][b] = L[idx[b] - t];
      }
    }
    switch (batch) {
      case 1:
        cholMappedUpdate(ns, 1, nlead, L_nnz, pos_map, colptr, Lptr0, Lptr1, Lptr2,
                         p[0], p[1], p[2], p[3], s);
        break;
      case 2:
        cholMappedUpdate(ns, 2, nlead, L_nnz, pos_map, colptr, Lptr0, Lptr1, Lptr2,
                         p[0], p[1], p[2], p[3], s);
        break;
      case 3:
        cholMappedUpdate(ns, 3, nlead, L_nnz, pos_map, colptr, Lptr0, Lptr1, Lptr2,
                         p[0], p[1], p[2], p[3], s);
        break;
      case 4:
        cholMappedUpdate(ns, 4, nlead, L_nnz, pos_map, colptr, Lptr0, Lptr1, Lptr2,
                         p[0], p[1], p[2], p[3], s);
        break;
    }
    k += batch;
  } while (k < LT_nnz);

  return k;
}


// outlined ns=1 wrapper for cholMappedRunImpl; returns next index in LT[r]
static mjNOINLINE int cholMappedRun1(
    int k, int LT_nnz, int LT_adr, int L_nnz, const int* restrict L_colptr,
    const mjtNum* restrict L, const int* restrict L_rownnz, const int* restrict L_rowadr,
    const int* restrict L_colind, const int* restrict LT_colind, const int* restrict LT_map,
    mjtNum* restrict Lptr0, int* restrict mapped, int* restrict pos_map) {
  return cholMappedRunImpl(1, k, LT_nnz, LT_adr, L_nnz, L_colptr, L, L_rownnz, L_rowadr,
                           L_colind, LT_colind, LT_map, Lptr0, NULL, NULL, mapped, pos_map);
}


// outlined ns=2 wrapper for cholMappedRunImpl; returns next index in LT[r]
static mjNOINLINE int cholMappedRun2(
    int k, int LT_nnz, int LT_adr, int L_nnz, const int* restrict L_colptr,
    const mjtNum* restrict L, const int* restrict L_rownnz, const int* restrict L_rowadr,
    const int* restrict L_colind, const int* restrict LT_colind, const int* restrict LT_map,
    mjtNum* restrict Lptr0, mjtNum* restrict Lptr1, int* restrict mapped, int* restrict pos_map) {
  return cholMappedRunImpl(2, k, LT_nnz, LT_adr, L_nnz, L_colptr, L, L_rownnz, L_rowadr,
                           L_colind, LT_colind, LT_map, Lptr0, Lptr1, NULL, mapped, pos_map);
}


// outlined ns=3 wrapper for cholMappedRunImpl; returns next index in LT[r]
static mjNOINLINE int cholMappedRun3(
    int k, int LT_nnz, int LT_adr, int L_nnz, const int* restrict L_colptr,
    const mjtNum* restrict L, const int* restrict L_rownnz, const int* restrict L_rowadr,
    const int* restrict L_colind, const int* restrict LT_colind, const int* restrict LT_map,
    mjtNum* restrict Lptr0, mjtNum* restrict Lptr1, mjtNum* restrict Lptr2,
    int* restrict mapped, int* restrict pos_map) {
  return cholMappedRunImpl(3, k, LT_nnz, LT_adr, L_nnz, L_colptr, L, L_rownnz, L_rowadr,
                           L_colind, LT_colind, LT_map, Lptr0, Lptr1, Lptr2, mapped, pos_map);
}


// factor supernode of size ns (1, 2, or 3) starting at row r; return rank decrement (force inline)
static mjALWAYS_INLINE
int cholFactorSupernode(
    int r, int ns, mjtNum mindiag, mjtNum* restrict L,
    const int* restrict L_rownnz, const int* restrict L_rowadr, const int* restrict L_colind,
    const int* restrict LT_rownnz, const int* restrict LT_rowadr,
    const int* restrict LT_colind, const int* restrict LT_map,
    const mjtNum* restrict H,
    const int* restrict H_rownnz, const int* restrict H_rowadr, const int* restrict H_colind,
    int* restrict pos_map) {
  int L_adr = L_rowadr[r];
  int L_nnz = L_rownnz[r];
  const int* restrict L_colptr = L_colind + L_adr;
  int LT_adr = LT_rowadr[r];
  int LT_nnz = LT_rownnz[r];

  mjtNum* restrict Lptr0 = L + L_adr;
  mjtNum* restrict Lptr1 = (ns >= 2) ? L + L_rowadr[r-1] : NULL;
  mjtNum* restrict Lptr2 = (ns >= 3) ? L + L_rowadr[r-2] : NULL;
  mjtNum* restrict Lptr_s[3] = {Lptr0, Lptr1, Lptr2};

  // initialize L rows for the supernode from H (copy if no fill-in, else outlined map into L)
  int mapped = 0;
  for (int s=0; s < ns; s++) {
    int row = r - s;
    int nnz_s = L_nnz - s;
    int h_adr = H_rowadr[row];
    int h_nnz = H_rownnz[row];
    if (h_nnz >= nnz_s && H_colind[h_adr+nnz_s-1] == row) {
      mju_copy(Lptr_s[s], H+h_adr, nnz_s);
    } else {
      cholInitMappedH(ns, r, s, L_nnz, L_colptr, Lptr0, Lptr1, Lptr2,
                      H, H_rownnz, H_rowadr, H_colind, &mapped, pos_map);
      break;
    }
  }

  // accumulate updates from descendant rows c > r in LT[r] (skip k=0 diagonal entry);
  // cap batch at 3 when ns == 3 so 3*3 broadcast scalars + row/descendant vectors fit in 16 YMM regs
  const int max_batch = (ns == 3) ? 3 : 4;
  int k = 1;
  while (k < LT_nnz) {
    int c0 = LT_colind[LT_adr+k];
    int idx0 = LT_map[LT_adr+k];
    int c0_adr = L_rowadr[c0];
    int num_cols = idx0 - c0_adr + 1;
    int col0 = L_colind[c0_adr];

    // by Cholesky elimination, cols(L[c, 0:r]) is a subset of cols(L[r, 0:r]) ending at r;
    // it matches the suffix L_colptr[L_nnz-num_cols .. L_nnz-1] iff its first column matches
    if (L_colptr[L_nnz-num_cols] == col0) {
      const mjtNum* p[4] = {L + c0_adr, NULL, NULL, NULL};
      int idx[4] = {idx0, 0, 0, 0};
      int batch = 1;
      while (k + batch < LT_nnz && batch < max_batch) {
        int next_c = LT_colind[LT_adr+k+batch];
        int next_idx = LT_map[LT_adr+k+batch];
        int next_adr = L_rowadr[next_c];
        if (next_idx - next_adr + 1 != num_cols || L_colind[next_adr] != col0) {
          break;
        }
        p[batch] = L + next_adr;
        idx[batch] = next_idx;
        batch++;
      }

      int off = L_nnz - num_cols;
      int nlead = num_cols - (ns - 1);
      mjtNum* restrict dptr0 = Lptr0 + off;
      mjtNum* restrict dptr1 = (ns >= 2) ? Lptr1 + off : NULL;
      mjtNum* restrict dptr2 = (ns >= 3) ? Lptr2 + off : NULL;
      mjtNum s[3][4];
      for (int t=0; t < ns; t++) {
        for (int b=0; b < batch; b++) {
          s[t][b] = L[idx[b] - t];
        }
      }
      switch (batch) {
        case 1:
          cholSuffixUpdate(ns, 1, nlead, dptr0, dptr1, dptr2, p[0], p[1], p[2], p[3], s);
          break;
        case 2:
          cholSuffixUpdate(ns, 2, nlead, dptr0, dptr1, dptr2, p[0], p[1], p[2], p[3], s);
          break;
        case 3:
          cholSuffixUpdate(ns, 3, nlead, dptr0, dptr1, dptr2, p[0], p[1], p[2], p[3], s);
          break;
        case 4:
          cholSuffixUpdate(ns, 4, nlead, dptr0, dptr1, dptr2, p[0], p[1], p[2], p[3], s);
          break;
      }
      k += batch;
    }

    // non-suffix run: outlined per ns so hot suffix loop stays small and spill-free
    else if (ns == 3) {
      k = cholMappedRun3(k, LT_nnz, LT_adr, L_nnz, L_colptr, L, L_rownnz, L_rowadr,
                         L_colind, LT_colind, LT_map, Lptr0, Lptr1, Lptr2, &mapped, pos_map);
    } else if (ns == 2) {
      k = cholMappedRun2(k, LT_nnz, LT_adr, L_nnz, L_colptr, L, L_rownnz, L_rowadr,
                         L_colind, LT_colind, LT_map, Lptr0, Lptr1, &mapped, pos_map);
    } else {
      k = cholMappedRun1(k, LT_nnz, LT_adr, L_nnz, L_colptr, L, L_rownnz, L_rowadr,
                         L_colind, LT_colind, LT_map, Lptr0, &mapped, pos_map);
    }
  }

  // factor supernode rows r, r-1, ... and fuse scaling with intra-supernode elimination
  int rank_dec = 0;
  int n_off = L_nnz - ns;
  for (int s=0; s < ns; s++) {
    mjtNum* restrict row_s = Lptr_s[s];
    int nnz_s = L_nnz - s;
    mjtNum diag = row_s[nnz_s-1];
    int deficient = !(diag >= mindiag);
    if (deficient) {
      diag = mindiag;
      rank_dec++;
    }
    mjtNum L_rr = mju_sqrt(diag);
    if (deficient) {
      mju_zero(row_s, nnz_s-1);
    } else {
      mjtNum L_rr_inv = 1.0 / L_rr;
      mjtNum s_st[3] = {0, 0, 0};
      for (int t=s+1; t < ns; t++) {
        s_st[t] = (row_s[L_nnz-1-t] *= L_rr_inv);
      }
      for (int i=0; i < n_off; i++) {
        mjtNum l_si = (row_s[i] *= L_rr_inv);
        if (s + 1 < ns) Lptr_s[s+1][i] -= s_st[s+1] * l_si;
        if (s + 2 < ns) Lptr_s[s+2][i] -= s_st[s+2] * l_si;
      }
      for (int t=s+1; t < ns; t++) {
        for (int u=t; u < ns; u++) {
          Lptr_s[t][L_nnz-1-u] -= s_st[t] * s_st[u];
        }
      }
    }
    row_s[nnz_s-1] = L_rr;
  }

  return rank_dec;
}


// numeric reverse-Cholesky: compute L values given fixed sparsity pattern, returns rank
//  L_colind must already contain the correct sparsity pattern (from mju_cholFactorSymbolic)
//  LT_map[k] gives index in L for LT_colind[k], L_super gives supernode sizes
int mju_cholFactorNumeric(mjtNum* restrict L, int n, mjtNum mindiag,
                          const int* restrict L_rownnz, const int* restrict L_rowadr,
                          const int* restrict L_colind,
                          const int* restrict LT_rownnz, const int* restrict LT_rowadr,
                          const int* restrict LT_colind, const int* restrict LT_map,
                          const int* restrict L_super, const mjtNum* restrict H,
                          const int* restrict H_rownnz, const int* restrict H_rowadr,
                          const int* restrict H_colind, int* restrict scratch) {
  int rank = n;
  int* restrict pos_map = scratch;

  // backpass over rows
  int ns;
  for (int r=n-1; r >= 0; r -= ns) {
    int L_adr = L_rowadr[r];
    int L_nnz = L_rownnz[r];
    mjtNum* restrict Lptr = L + L_adr;

    int LT_nnz = LT_rownnz[r];

    // lower triangle of H[r] has no fill-in iff its L_nnz-th column is the diagonal r
    int H_adr = H_rowadr[r];
    int H_nnz = H_rownnz[r];
    int H_nofill = (H_nnz >= L_nnz && H_colind[H_adr+L_nnz-1] == r);

    // leaf fast path: no updates from rows c > r and no fill-in on row r
    if (LT_nnz == 1 && H_nofill) {
      ns = 1;
      const mjtNum* restrict Hptr = H + H_adr;
      mjtNum diag = Hptr[L_nnz-1];
      int deficient = !(diag >= mindiag);
      if (deficient) {
        diag = mindiag;
        rank--;
      }

      // scale off-diagonals directly from H into L; clear if rank-deficient
      mjtNum L_rr = mju_sqrt(diag);
      if (deficient) {
        mju_zero(Lptr, L_nnz-1);
      } else {
        mjtNum L_rr_inv = 1.0 / L_rr;
        for (int i=0; i < L_nnz-1; i++) {
          Lptr[i] = Hptr[i] * L_rr_inv;
        }
      }
      Lptr[L_nnz-1] = L_rr;
      continue;
    }

    // supernode size from the symbolic phase
    ns = L_super[r];

    // pass ns as a compile-time literal so cholFactorSupernode specializes per supernode size
    if (ns == 3) {
      rank -= cholFactorSupernode(r, 3, mindiag, L, L_rownnz, L_rowadr, L_colind,
                                  LT_rownnz, LT_rowadr, LT_colind, LT_map,
                                  H, H_rownnz, H_rowadr, H_colind, pos_map);
    } else if (ns == 2) {
      rank -= cholFactorSupernode(r, 2, mindiag, L, L_rownnz, L_rowadr, L_colind,
                                  LT_rownnz, LT_rowadr, LT_colind, LT_map,
                                  H, H_rownnz, H_rowadr, H_colind, pos_map);
    } else {
      rank -= cholFactorSupernode(r, 1, mindiag, L, L_rownnz, L_rowadr, L_colind,
                                  LT_rownnz, LT_rowadr, LT_colind, LT_map,
                                  H, H_rownnz, H_rowadr, H_colind, pos_map);
    }
  }

  return rank;
}


// sparse reverse-order Cholesky solve
//  requires a complete factor: supernode detection assumes that L(r,c) != 0 (r > c) implies
//  struct(L(r,0:c)) is a subset of struct(L(c,0:c)), which incomplete factors may violate
void mju_cholSolveSparse(mjtNum* restrict res, const mjtNum* restrict mat,
                         const mjtNum* vec, int n,
                         const int* restrict rownnz, const int* restrict rowadr,
                         const int* restrict colind) {
  // copy into result if different
  if (res != vec) {
    mju_copy(res, vec, n);
  }

  // backward substitution: res <- L^-T res
  int i = n - 1;
  while (i >= 0) {
    const int adr = rowadr[i], nnz = rownnz[i];

    // fuse adjacent supernode rows (up to 3) when they share the exact same off-diagonal columns
    if (i > 0 && nnz >= 2 && rownnz[i-1] == nnz-1 && colind[adr+nnz-2] == i-1) {
      const mjtNum* restrict row0 = mat + adr;
      const mjtNum* restrict row1 = mat + rowadr[i-1];
      const int* restrict ci = colind + adr;

      // 3-row supernode (i, i-1, i-2)
      if (i > 1 && nnz >= 3 && rownnz[i-2] == nnz-2 && colind[adr+nnz-3] == i-2) {
        const int nob = nnz - 3;
        const mjtNum* restrict row2 = mat + rowadr[i-2];

        mjtNum x0 = (res[i] /= row0[nob+2]);
        mjtNum x1 = (res[i-1] = (res[i-1] - row0[nob+1]*x0) / row1[nob+1]);
        mjtNum x2 = (res[i-2] = (res[i-2] - row0[nob]*x0 - row1[nob]*x1) / row2[nob]);

        if (x0 || x1 || x2) {
          for (int j=0; j < nob; j++) {
            int c = ci[j];
            res[c] = res[c] - row0[j]*x0 - row1[j]*x1 - row2[j]*x2;
          }
        }
        i -= 3;
        continue;
      }

      // 2-row supernode (i, i-1)
      const int nob = nnz - 2;
      mjtNum x0 = (res[i] /= row0[nob+1]);
      mjtNum x1 = (res[i-1] = (res[i-1] - row0[nob]*x0) / row1[nob]);

      if (x0 || x1) {
        for (int j=0; j < nob; j++) {
          int c = ci[j];
          res[c] = res[c] - row0[j]*x0 - row1[j]*x1;
        }
      }
      i -= 2;
      continue;
    }

    if (res[i]) {
      mjtNum tmp = (res[i] /= mat[adr+nnz-1]);
      const mjtNum* restrict row = mat + adr;
      const int* restrict ci = colind + adr;
      for (int j=0; j < nnz-1; j++) {
        res[ci[j]] -= row[j] * tmp;
      }
    }
    i--;
  }

  // forward substitution: res <- L^-1 res
  i = 0;
  while (i < n) {
    const int adr = rowadr[i], nnz = rownnz[i];

    // fuse adjacent supernode rows (up to 3) when they share the exact same off-diagonal columns
    if (i + 1 < n && rownnz[i+1] == nnz+1 && colind[rowadr[i+1]+nnz-1] == i) {
      const int nob = nnz - 1;
      const mjtNum* restrict row0 = mat + adr;
      const mjtNum* restrict row1 = mat + rowadr[i+1];
      const int* restrict ci = colind + adr;

      // 3-row supernode (i, i+1, i+2)
      if (i + 2 < n && rownnz[i+2] == nnz+2 && colind[rowadr[i+2]+nnz-1] == i) {
        const mjtNum* restrict row2 = mat + rowadr[i+2];
        mjtNum s0 = 0, s1 = 0, s2 = 0;
        for (int j=0; j < nob; j++) {
          mjtNum y = res[ci[j]];
          s0 += row0[j] * y;
          s1 += row1[j] * y;
          s2 += row2[j] * y;
        }
        res[i] = (res[i] - s0) / row0[nob];
        res[i+1] = (res[i+1] - s1 - row1[nob]*res[i]) / row1[nob+1];
        res[i+2] = (res[i+2] - s2 - row2[nob]*res[i] - row2[nob+1]*res[i+1]) / row2[nob+2];
        i += 3;
        continue;
      }

      // 2-row supernode (i, i+1)
      mjtNum s0 = 0, s1 = 0;
      for (int j=0; j < nob; j++) {
        mjtNum y = res[ci[j]];
        s0 += row0[j] * y;
        s1 += row1[j] * y;
      }
      res[i] = (res[i] - s0) / row0[nob];
      res[i+1] = (res[i+1] - s1 - row1[nob]*res[i]) / row1[nob+1];
      i += 2;
      continue;
    }

    // x(i) -= sum_j L(i,j)*x(j), j=0:i-1
    if (nnz > 1) {
      res[i] -= mju_dotSparse(mat+adr, res, nnz-1, colind+adr);
      // modulo AVX, the above line does
      // for (int j=0; j<nnz-1; j++)
      //   res[i] -= mat[adr+j]*res[colind[adr+j]];
    }

    // x(i) /= L(i,i)
    res[i] /= mat[adr+nnz-1];
    i++;
  }
}


// sparse reverse-order Cholesky rank-one update: L'*L +/- x*x'; return rank
//  x is sparse (with x_ind sorted ascending), change in sparsity pattern of mat is not allowed
int mju_cholUpdateSparse(mjtNum* restrict mat, const mjtNum* restrict x, int n, int flg_plus,
                         const int* restrict rownnz, const int* restrict rowadr,
                         const int* restrict colind, int x_nnz, const int* restrict x_ind,
                         mjtNum* scratch) {
  // early return if x is empty
  if (x_nnz == 0) {
    return n;
  }

  // get starting row (last non-zero in x) and initial minimum active row
  int start = x_ind[x_nnz-1];
  int min_row = x_ind[0];

  // zero active range of dense accumulator, then scatter x into dense
  mjtNum* restrict dense = scratch;
  mju_zero(dense+min_row, start-min_row+1);
  mju_scatter(dense, x, x_ind, x_nnz);

  // backpass over rows from start down to min_row
  int rank = n;
  for (int row=start; row >= min_row; row--) {
    // skip if zero
    mjtNum x_row = dense[row];
    if (x_row == 0) continue;

    // get rownnz (excluding diagonal), rowadr
    int nnz = rownnz[row] - 1;
    int adr = rowadr[row];

    // update diagonal, handle rank-deficient case
    mjtNum diag = mat[adr+nnz];
    mjtNum tmp = diag*diag + (flg_plus ? x_row*x_row : -x_row*x_row);
    if (!(tmp >= mjMINVAL)) {
      tmp = mjMINVAL;
      rank--;
    }
    mjtNum r = mju_sqrt(tmp);
    mat[adr+nnz] = r;

    if (nnz == 0) {
      continue;
    }

    // compute Givens rotation parameters https://en.wikipedia.org/wiki/Givens_rotation
    mjtNum c = diag / r;
    mjtNum s = -x_row / r;
    mjtNum s_signed = flg_plus ? -s : s;

    // extend zeroed min_row range if this row reaches earlier columns
    const int* restrict ci = colind + adr;
    int col0 = ci[0];
    if (col0 < min_row) {
      mju_zero(dense+col0, min_row-col0);
      min_row = col0;
    }

    // apply Givens rotation to off-diagonals and dense workspace (contiguous fast path if dense)
    mjtNum* restrict mrow = mat + adr;
    if (nnz == ci[nnz-1] - col0 + 1) {
      mjtNum* restrict dptr = dense + col0;
      for (int i=0; i < nnz; i++) {
        mjtNum dense_j = dptr[i];
        mjtNum mat_val = mrow[i];
        mrow[i] = c*mat_val + s_signed*dense_j;
        dptr[i] = s*mat_val + c*dense_j;
      }
    } else {
      for (int i=0; i < nnz; i++) {
        int j = ci[i];
        mjtNum dense_j = dense[j];
        mjtNum mat_val = mrow[i];
        mrow[i]  = c*mat_val + s_signed*dense_j;
        dense[j] = s*mat_val + c*dense_j;
      }
    }
  }

  return rank;
}


//---------------------------- banded Cholesky -----------------------------------------------------

// integer min and max without double evaluation
static inline int minInt(int a, int b) {
  return a < b ? a : b;
}
static inline int maxInt(int a, int b) {
  return a > b ? a : b;
}


// address of diagonal element i in band-dense matrix representation
int mju_bandDiag(int i, int ntotal, int nband, int ndense) {
  int nsparse = ntotal - ndense;

  // sparse part
  if (i < nsparse) {
    return i*nband + nband - 1;
  }

  // dense part
  else {
    return nsparse*nband + (i-nsparse)*ntotal + i;
  }
}


// band-dense Cholesky decomposition
//  returns minimum value in the factorized diagonal, or 0 if rank-deficient
//  mat has (ntotal-ndense) x nband + ndense x ntotal elements
//  the first (ntotal-ndense) x nband store the band part, left of diagonal, inclusive
//  the second ndense x ntotal store the band part as entire dense rows
//  add diagadd+diagmul*mat_ii to diagonal before factorization
mjtNum mju_cholFactorBand(mjtNum* restrict mat, int ntotal, int nband, int ndense,
                          mjtNum diagadd, mjtNum diagmul) {
  int nsparse = ntotal - ndense;
  mjtNum mindiag = -1;

  // sparse part, including sparse-sparse and sparse-dense
  for (int j=0; j < nsparse; j++) {
    // number of non-zeros left of (j,j)
    int width_jj = minInt(j, nband-1);

    // number of non-zeros below (j,j), sparse part
    int height = minInt(nsparse-j-1, nband-1);

    // address of (j,j)
    int adr_jj = (j+1)*nband - 1;

    // compute L(j,j), before sqrt
    mjtNum left_jj = width_jj > 1 ? mju_dot(mat+adr_jj-width_jj, mat+adr_jj-width_jj, width_jj) :
                     (width_jj == 1 ? mat[adr_jj-1]*mat[adr_jj-1] : 0);
    mjtNum Ljj = diagadd + diagmul*mat[adr_jj] + mat[adr_jj] - left_jj;

    // stop if rank-deficient
    if (!(Ljj >= mjMINVAL)) {
      return 0;
    }

    // compute Ljj, scale = 1/Ljj
    Ljj = mju_sqrt(Ljj);
    mjtNum scale = 1 / Ljj;

    // update mindiag
    if (Ljj < mindiag || mindiag < 0) {
      mindiag = Ljj;
    }

    // compute L(i,j) for i > j, sparse part
    int adr_ij = adr_jj + nband - 1;
    for (int i=j+1; i <= j+height; i++, adr_ij += nband-1) {
      // number of non-zeros left of (i,j)
      int width_ij = minInt(j, nband-1-i+j);

      // in-place computation of L(i,j)
      mjtNum left_ij = width_ij > 1 ? mju_dot(mat+adr_jj-width_ij, mat+adr_ij-width_ij, width_ij) :
                       (width_ij == 1 ? mat[adr_jj-1]*mat[adr_ij-1] : 0);
      mat[adr_ij] = scale * (mat[adr_ij] - left_ij);
    }

    // compute L(i,j) for i >= nsparse, dense part
    adr_ij = nsparse*nband + j;
    for (int i=nsparse; i < ntotal; i++, adr_ij += ntotal) {
      // in-place computation of L(i,j)
      //  number of non-zeros left of (i,j) now equals width_jj
      mjtNum left_ij = width_jj > 1 ? mju_dot(mat+adr_jj-width_jj, mat+adr_ij-width_jj, width_jj) :
                       (width_jj == 1 ? mat[adr_jj-1]*mat[adr_ij-1] : 0);
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

    // stop if rank-deficient
    if (!(Ljj >= mjMINVAL)) {
      return 0;
    }

    // compute Ljj, scale = 1/Ljj
    Ljj = mju_sqrt(Ljj);
    mjtNum scale = 1 / Ljj;

    // update mindiag
    if (Ljj < mindiag || mindiag < 0) {
      mindiag = Ljj;
    }

    // compute L(i,j) for i > j
    int adr_ij = adr_jj + ntotal;
    for (int i=j+1; i < ntotal; i++, adr_ij += ntotal) {
      // in-place computation of L(i,j)
      mat[adr_ij] = scale * (mat[adr_ij] - mju_dot(mat+adr_jj-j, mat+adr_ij-j, j));
    }

    // save L(j,j)
    mat[adr_jj] = Ljj;
  }

  return mindiag;
}


// solve with band-Cholesky decomposition
void mju_cholSolveBand(mjtNum* res, const mjtNum* restrict mat, const mjtNum* vec,
                       int ntotal, int nband, int ndense) {
  int nsparse = ntotal - ndense;

  // copy into result if different
  if (res != vec) {
    mju_copy(res, vec, ntotal);
  }

  //------- forward substitution: solve L*res = vec

  // sparse part
  for (int i=0, adr_ii=nband-1; i < nsparse; i++, adr_ii += nband) {
    // number of non-zeros left of (i,i)
    int width = minInt(i, nband-1);

    // off-diagonal
    if (width > 1) {
      res[i] -= mju_dot(mat + adr_ii - width, res + i - width, width);
    } else if (width == 1) {
      res[i] -= mat[adr_ii - 1] * res[i - 1];
    }

    // diagonal
    res[i] /= mat[adr_ii];
  }

  // dense part
  for (int i=nsparse, row_i=nsparse*nband; i < ntotal; i++, row_i += ntotal) {
    // off-diagonal
    if (i) {
      res[i] -= mju_dot(mat + row_i, res, i);
    }

    // diagonal
    res[i] /= mat[row_i + i];
  }

  //------- backward substitution: solve L'*res = res

  // dense part
  for (int i=ntotal-1, row_i=nsparse*nband+(ndense-1)*ntotal; i >= nsparse; i--, row_i -= ntotal) {
    // diagonal
    res[i] /= mat[row_i + i];

    // update res[0..i-1] with row i of L
    if (i && res[i]) {
      mju_addToScl(res, mat + row_i, -res[i], i);
    }
  }

  // sparse part
  for (int i=nsparse-1, adr_ii=nsparse*nband-1; i >= 0; i--, adr_ii -= nband) {
    // number of non-zeros left of (i,i)
    int width = minInt(i, nband-1);

    // diagonal
    res[i] /= mat[adr_ii];

    // update res[i-width..i-1] with row i of L
    if (width > 1 && res[i]) {
      mju_addToScl(res + i - width, mat + adr_ii - width, -res[i], width);
    } else if (width == 1) {
      res[i - 1] -= mat[adr_ii - 1] * res[i];
    }
  }
}


// convert band matrix to dense matrix
void mju_band2Dense(mjtNum* restrict res, const mjtNum* restrict mat,
                    int ntotal, int nband, int ndense, mjtBool flg_sym) {
  int nsparse = ntotal - ndense;

  // clear all
  mju_zero(res, ntotal*ntotal);

  // sparse part
  for (int i=0; i < nsparse; i++) {
    // number of non-zeros left of (i,i)
    int width = minInt(i, nband-1);

    // copy data
    mju_copy(res + i*ntotal + i-width, mat + (i+1)*nband - (width+1), width+1);
  }

  // dense part
  for (int i=nsparse; i < ntotal; i++) {
    mju_copy(res + i*ntotal, mat + nsparse*nband + (i-nsparse)*ntotal, i+1);
  }

  // make symmetric
  if (flg_sym) {
    for (int i=0; i < ntotal; i++) {
      for (int j=i+1; j < ntotal; j++) {
        res[i*ntotal + j] = res[j*ntotal + i];
      }
    }
  }
}


// convert dense matrix to band matrix
void mju_dense2Band(mjtNum* restrict res, const mjtNum* restrict mat,
                    int ntotal, int nband, int ndense) {
  int nsparse = ntotal - ndense;

  // sparse part
  for (int i=0; i < nsparse; i++) {
    // number of non-zeros left of (i,i)
    int width = minInt(i, nband-1);

    // copy data
    mju_copy(res + (i+1)*nband - (width+1), mat + i*ntotal + i-width, width+1);
  }

  // dense part
  for (int i=nsparse; i < ntotal; i++) {
    mju_copy(res + nsparse*nband + (i-nsparse)*ntotal, mat + i*ntotal, i+1);
  }
}


// multiply band-diagonal matrix with vector, res must not alias vec
void mju_bandMulMatVec(mjtNum* restrict res, const mjtNum* restrict mat,
                       const mjtNum* restrict vec, int ntotal, int nband, int ndense,
                       int nvec, mjtBool flg_sym) {
  int nsparse = ntotal - ndense;

  // handle multiple vectors
  for (int j=0; j < nvec; j++) {
    // precompute pointer to corresponding vector in vec and res
    const mjtNum* restrict vec_j = vec + ntotal*j;
    mjtNum* restrict res_j = res + ntotal*j;

    // sparse part
    int row_end = nband;
    for (int i=0; i < nsparse; i++, row_end += nband) {
      int width = minInt(i+1, nband);
      int adr = row_end - width;
      int offset = i + 1 - width;
      res_j[i] = mju_dot(mat+adr, vec_j+offset, width);  // lower triangle
      if (flg_sym && width > 1) {
        // strict upper triangle
        mju_addToScl(res_j+offset, mat+adr, vec_j[i], width-1);
      }
    }

    // dense part
    int adr = nsparse*nband;
    for (int i=nsparse; i < ntotal; i++, adr += ntotal) {
      res_j[i] = mju_dot(mat+adr, vec_j, i+1);          // lower triangle
      if (flg_sym && i > 0) {
        // strict upper triangle
        mju_addToScl(res_j, mat+adr, vec_j[i], i);
      }
    }
  }
}


//------------------------------ dense LU factorization --------------------------------------------

// dense LU factorization with partial pivoting
//   factorizes n x n row-major matrix A in-place into L and U
//   L has unit diagonal (not stored), U has explicit diagonal
//   pivot stores row permutation: row i of original = row pivot[i] of result
//   return 1 if successful, 0 if singular (diagonal element < mjMINVAL)
int mju_factorLU(mjtNum* restrict A, int n, int* pivot) {
  for (int k=0; k < n; k++) {
    // initialize pivot
    pivot[k] = k;

    // find pivot: max absolute value in column k, rows k..n-1
    mjtNum maxval = mju_abs(A[k*n+k]);
    int maxrow = k;
    for (int i=k+1; i < n; i++) {
      mjtNum val = mju_abs(A[i*n+k]);
      if (val > maxval) {
        maxval = val;
        maxrow = i;
      }
    }

    // check singularity
    if (maxval < mjMINVAL) {
      return 0;
    }

    // swap rows k and maxrow
    if (maxrow != k) {
      pivot[k] = maxrow;
      for (int j=0; j < n; j++) {
        mjtNum tmp = A[k*n+j];
        A[k*n+j] = A[maxrow*n+j];
        A[maxrow*n+j] = tmp;
      }
    }

    // compute multipliers and update trailing submatrix
    mjtNum diaginv = 1.0 / A[k*n+k];
    for (int i=k+1; i < n; i++) {
      A[i*n+k] *= diaginv;
      mjtNum Aik = A[i*n+k];
      for (int j=k+1; j < n; j++) {
        A[i*n+j] -= Aik * A[k*n+j];
      }
    }
  }

  return 1;
}


// solve A*x = b given LU factorization of A, LU and pivot are output of mju_factorLU
void mju_solveLU(mjtNum* restrict x, const mjtNum* LU, const mjtNum* b, const int* pivot, int n) {
  // copy b into x
  mju_copy(x, b, n);

  // apply row permutation and forward substitution: solve L*y = P*b
  for (int i=0; i < n; i++) {
    // apply pivot swap
    if (pivot[i] != i) {
      mjtNum tmp = x[i];
      x[i] = x[pivot[i]];
      x[pivot[i]] = tmp;
    }

    // subtract known terms
    for (int j=0; j < i; j++) {
      x[i] -= LU[i*n+j] * x[j];
    }
  }

  // back substitution: solve U*x = y
  for (int i=n-1; i >= 0; i--) {
    for (int j=i+1; j < n; j++) {
      x[i] -= LU[i*n+j] * x[j];
    }
    x[i] /= LU[i*n+i];
  }
}


// single Gaussian-elimination step k in {0..4} of 6x6 dense LU factorization (force inline)
static mjALWAYS_INLINE
int factorLU6_step(mjtNum* restrict A, int* restrict pivot, int k) {
  // initialize pivot
  pivot[k] = k;

  // find pivot: max absolute value in column k, rows k..5
  mjtNum maxval = mju_abs(A[k*6+k]);
  int maxrow = k;
  for (int i=k+1; i < 6; i++) {
    mjtNum val = mju_abs(A[i*6+k]);
    if (val > maxval) {
      maxval = val;
      maxrow = i;
    }
  }

  // check singularity
  if (maxval < mjMINVAL) {
    return 0;
  }

  // swap rows k and maxrow
  if (maxrow != k) {
    pivot[k] = maxrow;
    for (int j=0; j < 6; j++) {
      mjtNum tmp = A[k*6+j];
      A[k*6+j] = A[maxrow*6+j];
      A[maxrow*6+j] = tmp;
    }
  }

  // compute multipliers and update trailing submatrix
  mjtNum diaginv = 1.0 / A[k*6+k];
  for (int i=k+1; i < 6; i++) {
    A[i*6+k] *= diaginv;
    mjtNum Aik = A[i*6+k];
    for (int j=k+1; j < 6; j++) {
      A[i*6+j] -= Aik * A[k*6+j];
    }
  }

  return 1;
}


// 6x6 specialization of mju_factorLU: same algorithm with compile-time size,
// allowing full unrolling; produces identical results
int mju_factorLU6(mjtNum A[36], int pivot[6]) {
  if (!factorLU6_step(A, pivot, 0)) return 0;
  if (!factorLU6_step(A, pivot, 1)) return 0;
  if (!factorLU6_step(A, pivot, 2)) return 0;
  if (!factorLU6_step(A, pivot, 3)) return 0;
  if (!factorLU6_step(A, pivot, 4)) return 0;

  pivot[5] = 5;
  if (mju_abs(A[35]) < mjMINVAL) {
    return 0;
  }

  return 1;
}


// solve A*x = b given 6x6 LU factorization from mju_factorLU6
void mju_solveLU6(mjtNum x[6], const mjtNum LU[36], const mjtNum b[6], const int pivot[6]) {
  // apply row permutation
  mjtNum t[6] = {b[0], b[1], b[2], b[3], b[4], b[5]};
  for (int i=0; i < 5; i++) {
    int p = pivot[i];
    if (p != i) {
      mjtNum tmp = t[i];
      t[i] = t[p];
      t[p] = tmp;
    }
  }

  // forward substitution: solve L*y = P*b
  mjtNum y0 = t[0];
  mjtNum y1 = t[1] - LU[1*6+0]*y0;
  mjtNum y2 = t[2] - LU[2*6+0]*y0 - LU[2*6+1]*y1;
  mjtNum y3 = t[3] - LU[3*6+0]*y0 - LU[3*6+1]*y1 - LU[3*6+2]*y2;
  mjtNum y4 = t[4] - LU[4*6+0]*y0 - LU[4*6+1]*y1 - LU[4*6+2]*y2 - LU[4*6+3]*y3;
  mjtNum y5 = t[5] - LU[5*6+0]*y0 - LU[5*6+1]*y1 - LU[5*6+2]*y2 - LU[5*6+3]*y3 - LU[5*6+4]*y4;

  // back substitution: solve U*x = y
  mjtNum x5 = y5 / LU[35];
  mjtNum x4 = (y4 - LU[4*6+5]*x5) / LU[28];
  mjtNum x3 = (y3 - LU[3*6+4]*x4 - LU[3*6+5]*x5) / LU[21];
  mjtNum x2 = (y2 - LU[2*6+3]*x3 - LU[2*6+4]*x4 - LU[2*6+5]*x5) / LU[14];
  mjtNum x1 = (y1 - LU[1*6+2]*x2 - LU[1*6+3]*x3 - LU[1*6+4]*x4 - LU[1*6+5]*x5) / LU[7];
  mjtNum x0 = (y0 - LU[1]*x1 - LU[2]*x2 - LU[3]*x3 - LU[4]*x4 - LU[5]*x5) / LU[0];

  x[0] = x0;
  x[1] = x1;
  x[2] = x2;
  x[3] = x3;
  x[4] = x4;
  x[5] = x5;
}


//------------------------------ sparse LU factorization -------------------------------------------

// sparse reverse-order LU factorization, no fill-in (assuming tree topology)
//   result: LU = L + U; original = (U+I) * L; scratch size is n
//   clamp pivots with magnitude below mjMINVAL, return first clamped dof index or -1 if none
int mju_factorLUSparse(mjtNum* LU, int n, int* scratch,
                       const int* rownnz, const int* rowadr, const int* colind,
                       const int* index) {
  int* remaining = scratch;
  int clamped = -1;

  // set remaining = rownnz
  if (index) {
    for (int i=0; i < n; i++) {
      remaining[i] = rownnz[index[i]];
    }
  } else {
    mju_copyInt(remaining, rownnz, n);
  }

  // diagonal elements (i,i)
  for (int r=n-1; r >= 0; r--) {
    int i = index ? index[r] : r;

    // get address of last remaining element of row i, adjust remaining counter
    int ii = rowadr[i] + remaining[r] - 1;
    remaining[r]--;

    // make sure ii is on diagonal
    if (colind[ii] != i) {
      mjERROR("missing diagonal element");
    }

    // near-singular pivot: clamp, preserving the sign
    if (mju_abs(LU[ii]) < mjMINVAL) {
      LU[ii] = LU[ii] < 0 ? -mjMINVAL : mjMINVAL;
      if (clamped < 0) {
        clamped = i;
      }
    }

    // rows j above i
    for (int c=r-1; c >= 0; c--) {
      int j = index ? index[c] : c;

      // get address of last remaining element of row j
      int ji = rowadr[j] + remaining[c] - 1;

      // process row j if (j,i) is non-zero
      if (colind[ji] == i) {
        // adjust remaining counter
        remaining[c]--;

        // (j,i) = (j,i) / (i,i)
        LU[ji] = LU[ji] / LU[ii];
        mjtNum LUji = LU[ji];

        // (j,k) = (j,k) - (i,k) * (j,i) for k<i; handle incompatible sparsity
        int icnt = rowadr[i], jcnt = rowadr[j];
        while (jcnt < rowadr[j]+remaining[c]) {
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
        if (icnt != rowadr[i]+remaining[r] || jcnt != rowadr[j]+remaining[c]) {
          mjERROR("row processing incomplete");
        }
      }
    }
  }

  // make sure remaining points to diagonal
  for (int r=0; r < n; r++) {
    int i = index ? index[r] : r;
    if (remaining[r] < 0 || colind[rowadr[i]+remaining[r]] != i) {
      mjERROR("unexpected sparse matrix structure");
    }
  }

  return clamped;
}


// solve mat*res=vec given LU factorization of mat
void mju_solveLUSparse(mjtNum* res, const mjtNum* LU, const mjtNum* vec, int n,
                       const int* rownnz, const int* rowadr, const int* diag, const int* colind,
                       const int* index) {
  // solve (U+I)*res = vec
  for (int k=n-1; k >= 0; k--) {
    int i = index ? index[k] : k;

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
  for (int k=0; k < n; k++) {
    int i = index ? index[k] : k;

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


//--------------------------- 3x3 linear solve -----------------------------------------------------

// solve 3x3 linear system A*x = b using Gaussian elimination
void mju_solve3(mjtNum x[3], const mjtNum A[9], const mjtNum b[3]) {
  mjtNum M[3][4] = {
    {A[0], A[1], A[2], b[0]},
    {A[3], A[4], A[5], b[1]},
    {A[6], A[7], A[8], b[2]}
  };

  for (int i=0; i<3; i++) {
    mjtNum pivot = M[i][i];
    for (int j=i; j<4; j++) {
      M[i][j] /= pivot;
    }

    for (int k=0; k<3; k++) {
      if (k != i) {
        mjtNum factor = M[k][i];
        for (int j=i; j<4; j++) {
          M[k][j] -= factor * M[i][j];
        }
      }
    }
  }
  x[0] = M[0][3];
  x[1] = M[1][3];
  x[2] = M[2][3];
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
//     R must have allocated size n*(n+7), but only nfree*nfree values are used in output
//     index (if given) must have allocated size n, but only nfree values are used in output
//     only lower triangles of H and R are read from and written to, respectively
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


// multiply symmetric matrix with vector: res = mat*vec
//   assumes symmetry of mat, ignores upper triangle, res must not alias vec
static void mulSymVec(mjtNum* restrict res, const mjtNum* mat, const mjtNum* vec, int n) {
  for (int i=0; i < n; i++) {
    // diagonal + strict lower triangle: res[i] = sum_{j<=i} mat[i,j] * vec[j]
    res[i] = mat[n*i+i]*vec[i] + mju_dot(mat+n*i, vec, i);

    // strict upper mirror contribution: res[k] += mat[i,k] * vec[i] for k < i
    if (vec[i]) {
      mju_addToScl(res, mat+n*i, vec[i], i);
    }
  }
}


// delete row and column p from m-by-m Cholesky factor R, return rank of (m-1)-by-(m-1) factor
static int cholDelete(mjtNum* restrict R, mjtNum* restrict temp, int m, int p) {
  // shift rows 1..p-1 to stride m-1
  for (int i=1; i < p; i++) {
    for (int j=0; j <= i; j++) {
      R[i*(m-1)+j] = R[i*m+j];
    }
  }

  // zero leading entries of update vector
  mju_zero(temp, p);

  // shift rows p+1..m-1 to rows p..m-2 (stride m-1) and extract column p into temp
  for (int i=p+1; i < m; i++) {
    for (int j=0; j < p; j++) {
      R[(i-1)*(m-1)+j] = R[i*m+j];
    }
    temp[i-1] = R[i*m+p];
    for (int j=p+1; j <= i; j++) {
      R[(i-1)*(m-1)+(j-1)] = R[i*m+j];
    }
  }

  // rank-1 update on trailing (m-1-p) block
  return mju_cholUpdate(R, temp, m-1, 1);
}


// insert variable idx at position p into m-by-m Cholesky factor R, return rank of (m+1)-by-(m+1)
static int cholInsert(mjtNum* restrict R, mjtNum* restrict temp, mjtNum* restrict rowp,
                      const mjtNum* H, int n, const int* index, int m, int p, int idx) {
  // solve L_11 * l_21 = h_21 for new row p (entries 0..p-1)
  for (int j=0; j < p; j++) {
    mjtNum s = H[idx*n+index[j]];
    if (j) {
      s -= mju_dot(R+j*m, rowp, j);
    }
    rowp[j] = s / R[j*(m+1)];
  }

  // diagonal entry l_22
  mjtNum diag = H[idx*(n+1)] - mju_dot(rowp, rowp, p);
  if (!(diag >= mjMINVAL)) {
    return -1;
  }
  mjtNum l22 = mju_sqrt(diag);
  rowp[p] = l22;

  // compute new column p below diagonal: l_32 = (h_32 - L_31 * l_21) / l_22
  mju_zero(temp, p+1);
  mjtNum inv_l22 = 1 / l22;
  for (int i=p; i < m; i++) {
    temp[i+1] = (H[index[i]*n+idx] - mju_dot(R+i*m, rowp, p)) * inv_l22;
  }

  // expand R from stride m to stride m+1 (in-place backward pass)
  for (int i=m-1; i >= p; i--) {
    for (int j=i; j >= p; j--) {
      R[(i+1)*(m+1)+(j+1)] = R[i*m+j];
    }
    R[(i+1)*(m+1)+p] = temp[i+1];
    for (int j=p-1; j >= 0; j--) {
      R[(i+1)*(m+1)+j] = R[i*m+j];
    }
  }

  // write new row p
  for (int j=0; j <= p; j++) {
    R[p*(m+1)+j] = rowp[j];
  }

  // shift rows p-1..1 to stride m+1
  for (int i=p-1; i >= 1; i--) {
    for (int j=i; j >= 0; j--) {
      R[i*(m+1)+j] = R[i*m+j];
    }
  }

  // rank-1 downdate on trailing (m-p) block
  return mju_cholUpdate(R, temp, m+1, 0);
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
  int nfree = n;                    // initialize nfree with n
  int nfactor = 0;
  mjtNum value = 0, norm2 = 0;

  // basic checks
  if (n <= 0) {
    mjERROR("problem size n must be positive");
  }
  if (upper && lower) {
    for (int i=0; i < n; i++) {
      if (lower[i] >= upper[i]) {
        mjERROR("upper bounds must be strictly larger than lower bounds");
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
    // try to factorize (only lower triangle of H is read and written to R)
    for (int i=0; i < n; i++) {
      for (int j=0; j <= i; j++) {
        R[i*n+j] = H[i*n+j];
      }
    }
    int rank = mju_cholFactor(R, n, mjMINVAL);
    if (rank == n) {
      mju_cholSolve(res, R, g, n);
      mju_scl(res, res, -1, n);
      value = 0.5 * mju_dot(res, g, n);
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
  for (iter=0; iter < maxiter; iter++) {
    // stop if already resolved (e.g. unbounded fast path or line-search failure)
    if (status != mjBOXQP_NO_DESCENT) {
      break;
    }

    // compute gradient and objective: value = 0.5*res'*H*res + res'*g
    mulSymVec(grad, H, res, n);
    if (iter == 0) {
      value = 0.5 * mju_dot(res, grad, n) + mju_dot(res, g, n);
    }
    mju_addTo(grad, g, n);

    // save last value
    mjtNum oldvalue = value;

    // find clamped dimensions, count free dimensions
    int oldnfree = nfree;
    nfree = 0;
    for (int i=0; i < n; i++) {
      clamped[i] = ( lower && res[i] == lower[i] && grad[i] > 0 ) ||
                   ( upper && res[i] == upper[i] && grad[i] < 0 );
      if (!clamped[i]) {
        nfree++;
      }
    }

    // all dimensions are clamped: minimum found
    if (!nfree) {
      status = mjBOXQP_ALL_CLAMPED;
      break;
    }

    // update or re-factorize if clamped dimensions have changed
    int rank = nfree;
    int factorize = (iter == 0);
    if (iter) {
      // count newly clamped and unclamped dimensions
      int nclamp = 0, nunclamp = 0;
      for (int i=0; i < n; i++) {
        if (clamped[i] && !oldclamped[i]) {
          nclamp++;
        } else if (!clamped[i] && oldclamped[i]) {
          nunclamp++;
        }
      }

      // maybe update Cholesky factor with rank-1 operations
      if (nclamp + nunclamp > 0) {
        // use rank-1 updates rather than full re-factorization when:
        //  - at least one previously free variable remains free (nclamp < oldnfree)
        //  - insertions are weighted 2x deletions because cholInsert adds a triangular solve
        //  - weighted count <= max(2, nfree/8): flop crossover is nfree/3, but cholFactor uses
        //    row-wise SIMD mju_dot while updates shift R and do column-strided Givens rotations
        if (nclamp < oldnfree && nclamp + 2*nunclamp <= maxInt(2, nfree/8)) {
          int m = oldnfree;
          int rank_ok = 1;

          // delete newly clamped dimensions in reverse order
          for (int p=m-1; p >= 0; p--) {
            if (clamped[index[p]]) {
              if (cholDelete(R, temp, m, p) != m-1) {
                rank_ok = 0;
                break;
              }
              for (int j=p; j < m-1; j++) {
                index[j] = index[j+1];
              }
              m--;
            }
          }

          // insert newly unclamped dimensions
          if (nunclamp && rank_ok) {
            int p = 0;
            for (int i=0; i < n; i++) {
              if (!clamped[i]) {
                if (oldclamped[i]) {
                  if (cholInsert(R, temp, search, H, n, index, m, p, i) != m+1) {
                    rank_ok = 0;
                    break;
                  }
                  for (int j=m; j > p; j--) {
                    index[j] = index[j-1];
                  }
                  index[p] = i;
                  m++;
                }
                p++;
              }
            }
          }

          if (!rank_ok) {
            factorize = 1;
          }
        } else {
          factorize = 1;
        }
      }
    }

    // full factorization if required
    if (factorize) {
      int j = 0;
      for (int i=0; i < n; i++) {
        if (!clamped[i]) {
          index[j++] = i;
        }
      }
      for (int i=0; i < nfree; i++) {
        for (int k=0; k < i+1; k++) {
          R[i*nfree+k] = H[index[i]*n+index[k]];
        }
      }
      rank = mju_cholFactor(R, nfree, mjMINVAL);
      nfactor++;
    }

    // save last clamped
    for (int i=0; i < n; i++) {
      oldclamped[i] = clamped[i];
    }

    // abort if factorization failed
    if (rank != nfree) {
      status = mjBOXQP_NOT_SPD;
      break;
    }

    // ------ check gradient

    // squared norm of free gradient, compress grad_free into temp
    norm2 = 0;
    for (int i=0; i < nfree; i++) {
      mjtNum grad_i = grad[index[i]];
      temp[i] = grad_i;
      norm2 += grad_i*grad_i;
    }

    // small gradient: minimum found
    if (norm2 < mingrad) {
      status = nfree == n ? mjBOXQP_UNBOUNDED : mjBOXQP_TOL_GRAD;
      break;
    }

    // get search direction: solve H_free,free * temp = grad_free
    mju_cholSolve(temp, R, temp, nfree);

    // search = -temp, compute directional derivative sdotg
    mjtNum sdotg = 0;
    for (int i=0; i < nfree; i++) {
      search[i] = -temp[i];
      sdotg -= temp[i] * grad[index[i]];
    }

    // sanity check: make sure we have a descent direction
    if (sdotg >= 0) {
      break;  // SHOULD NOT OCCUR
    }

    // ------ projected Armijo line search in free subspace
    mjtNum step = 1, improvement = 0, reduction = 0;
    int nstep = 0, clipped = 0;
    do {
      // candidate = clamp(res_free + step*search)
      clipped = 0;
      for (int i=0; i < nfree; i++) {
        int idx = index[i];
        mjtNum c = res[idx] + step*search[i];
        if (lower && c < lower[idx]) {
          c = lower[idx];
          clipped = 1;
        } else if (upper && c > upper[idx]) {
          c = upper[idx];
          clipped = 1;
        }
        candidate[i] = c;
      }

      // objective reduction: -grad_free'*delta - 0.5*||R'*delta||^2
      if (!clipped) {
        reduction = -step*(1 - 0.5*step)*sdotg;
      } else {
        mju_zero(temp, nfree);
        mjtNum ddotg = 0;
        for (int i=0; i < nfree; i++) {
          int idx = index[i];
          mjtNum di = candidate[i] - res[idx];
          if (di) {
            ddotg += di * grad[idx];
            mju_addToScl(temp, R+i*nfree, di, i+1);
          }
        }
        reduction = -ddotg - 0.5*mju_dot(temp, temp, nfree);
      }
      value = oldvalue - reduction;

      // repeat until relative improvement >= Armijo, or break if step is too small
      nstep++;
      improvement = -reduction / (step*sdotg);
      if (improvement < armijo) {
        step = step*backtrack;
        if (step < minstep) {
          status = mjBOXQP_MAX_LS_ITER;
          break;
        }
      }
    } while (improvement < armijo);


    // print iteration info
    if (log && logptr < logsz) {
      int written = snprintf(log+logptr, logsz-logptr,
                             "iter %-3d:  |grad|: %-8.2g  reduction: %-8.2g  improvement: %-8.4g  "
                             "linesearch: %g^%-2d  factorized: %d  nfree: %d\n",
                             iter+1, mju_sqrt(norm2), reduction, improvement,
                             backtrack, nstep-1, factorize, nfree);
      if (written > 0) {
        logptr = minInt(logptr + written, logsz);
      }
    }

    // stop if line search failed without accepting rejected candidate
    if (status == mjBOXQP_MAX_LS_ITER) {
      value = oldvalue;
      break;
    }

    // accept candidate
    for (int i=0; i < nfree; i++) {
      res[index[i]] = candidate[i];
    }

    // stop if unclipped step produced no floating-point decrease in objective
    if (!clipped && value >= oldvalue) {
      status = nfree == n ? mjBOXQP_UNBOUNDED : mjBOXQP_TOL_GRAD;
      break;
    }
  }

  // max iterations exceeded
  if (iter == maxiter) {
    status = mjBOXQP_MAX_ITER;
  }

  // print final info
  if (log && logptr < logsz) {
    snprintf(log+logptr, logsz-logptr, "BOXQP: %s.\n"
             "iterations= %d,  factorizations= %d,  |grad|= %-12.6g, final value= %-12.6g\n",
             status_string[status+1], iter, nfactor, mju_sqrt(norm2), value);
  }

  // return nf or -1 if failure
  return (status == mjBOXQP_NO_DESCENT || status == mjBOXQP_NOT_SPD) ? -1 : nfree;
}



//--------------------------- general quadratic program --------------------------------------------

// mju_QP is a dual active-set method. With H = L*L' and rows m_i = inv(L)*a_i, the dual problem is
//   min 0.5*lam'*K*lam + lin'*lam,  K = M*M',  lin_i = bound_i - a_i'*x0,  x0 = -inv(H)*g,
// over multipliers lam that are non-negative at upper bounds and non-positive at lower bounds.
// The primal solution is x = x0 - inv(L')*M'*lam. A working set of constraints holds with
// multipliers that minimize the dual, and K restricted to the working set is kept in Cholesky
// form. Iterations either add a violated constraint or, if a multiplier would leave its range,
// step to the end of the range and change the state of that constraint.
//
// The penalty w*s + s^2/(2*c) on the violation s of a soft constraint, with threshold w and
// compliance c, adds c/2*(|lam|-w)^2 to the dual where |lam| > w. Such a constraint is rigid
// while |lam| <= w and yields beyond, where its row of K gains c on the diagonal and its
// multiplier is represented by the excess |lam|-w.
//
// A rigid constraint that is linearly dependent on the working set cannot be added to it.
// Multipliers then move along the direction that keeps the primal solution fixed, until one
// of them reaches the end of its range. If none does, the constraints are infeasible. A soft
// constraint that is dependent while its multiplier is at the threshold is held there, outside
// the working set.
//
// If H is singular the method is applied to a sequence of problems with Hessian H + prox*I and
// bias g - prox*center, where the center is the previous solution. The factorization of the
// Hessian and the working set carry over, only x0 changes. The sequence ends when the gradient
// prox*(x - center) of the proximal term is negligible. If the step x - center has no curvature
// and decreases the objective, the center moves on along it to the nearest bound. If there is no
// such bound the problem is unbounded.
//
// A linear program, H = 0, has no scale for the weight of the proximal term. The weight is the
// size of g divided by the size of the center, so that a step is as long as the center is large.
// It changes when the center does, and the factor of the working set is rebuilt.
//
// The dual problem squares the condition number of H, so H is treated as singular if a pivot
// of its factorization is below the weight of the proximal term, relative to the largest
// diagonal element. The condition number of H + prox*I is then bounded by the inverse of that
// relative weight.

// status values returned by mju_QP
enum mjtStatusQP {
  mjQP_NOT_PSD     = -1,           // Hessian is not positive semidefinite
  mjQP_INFEASIBLE  = -2,           // constraints are infeasible
  mjQP_UNBOUNDED   = -3,           // objective is unbounded on the feasible set
  mjQP_MAX_ITER    = -4            // maximum iterations exceeded
};


// state of a constraint in mju_QP, a negative sign selects the lower bound
enum mjtStateQP {
  mjQP_INACTIVE    = 0,            // multiplier is zero
  mjQP_RIGID       = 1,            // constraint holds, multiplier is within the threshold
  mjQP_YIELD       = 2,            // constraint is violated, multiplier is threshold + excess
  mjQP_EQUALITY    = 3,            // equality holds, multiplier can have either sign
  mjQP_HELD        = 4             // constraint holds, multiplier is fixed at the threshold
};


// tolerances of mju_QP
#ifndef mjUSESINGLE
  #define mjQP_FEASTOL   1e-9      // relative constraint violation
  #define mjQP_DEPTOL    1e-6      // relative violation of a dependent constraint
  #define mjQP_PIVOTTOL  1e-10     // relative squared pivot of a dependent constraint
  #define mjQP_MINPIVOT  1e-14     // smallest relative squared pivot
  #define mjQP_DIRTOL    1e-10     // relative component of a direction that is rounding error
  #define mjQP_SINGTOL   1e-6      // relative squared pivot of a singular Hessian, equals PROX
  #define mjQP_PROX      1e-6      // relative weight of the proximal term
  #define mjQP_PROXTOL   1e-11     // relative gradient of the proximal term at convergence
  #define mjQP_PROXSLOW  1e-6      // relative gradient accepted at the iteration limit
  #define mjQP_XSCALE    2.2e-7    // fraction of the size of x in the scale of a constraint
  #define mjQP_ROUND     2e-15     // relative rounding error of the difference of two points
#else
  #define mjQP_FEASTOL   1e-4f
  #define mjQP_DEPTOL    1e-3f
  #define mjQP_PIVOTTOL  1e-5f
  #define mjQP_MINPIVOT  1e-6f
  #define mjQP_DIRTOL    1e-5f
  #define mjQP_SINGTOL   1e-3f
  #define mjQP_PROX      1e-3f
  #define mjQP_PROXTOL   1e-5f
  #define mjQP_PROXSLOW  1e-3f
  #define mjQP_XSCALE    1.2e-3f
  #define mjQP_ROUND     1e-6f
#endif
#define mjQP_INF        ((mjtNum)1e30)   // unbounded multiplier
#define mjQP_REFINE     0.01             // relative squared pivot below which it is recomputed
#define mjQP_ITERBASE   100              // iteration limit is ITERBASE + ITERSCALE*(n+m)
#define mjQP_ITERSCALE  20
#define mjQP_PROXITER   50               // maximum number of proximal iterations


// problem definition and workspace of mju_QP, constraints are bounds followed by rows of A
typedef struct {
  int n;                           // number of variables
  int m;                           // number of rows of A
  int nc;                          // number of constraints (n + m)
  int kmax;                        // maximum size of the working set
  const mjtNum* lower;             // lower bounds on variables                   (n)
  const mjtNum* upper;             // upper bounds on variables                   (n)
  const mjtNum* A;                 // constraint matrix                           (m x n)
  const mjtNum* alower;            // lower bounds on rows                        (m)
  const mjtNum* aupper;            // upper bounds on rows                        (m)
  const mjtNum* threshold;         // multiplier thresholds                       (nc)
  const mjtNum* compliance;        // constraint compliances                      (nc)
  const mjtNum* g;                 // bias vector                                 (n)
  mjtNum prox;                     // weight of the proximal term, 0: H is not singular
  mjtNum cond;                     // condition number of the factorized Hessian
  mjtNum* center;                  // center of the proximal term                 (n)
  mjtNum* L;                       // Cholesky factor of H + prox*I               (n x n)
  mjtNum* x0;                      // unconstrained minimizer                     (n)
  mjtNum* cx;                      // constraint values at x0                     (nc)
  mjtNum* norm;                    // norms of constraint rows, negative: ignored (nc)
  mjtNum* lam;                     // multipliers, excess if yielding             (nc)
  int* state;                      // constraint states                           (nc)
  int nact;                        // size of the working set
  int nrigid;                      // number of constraints in the working set that hold
  int* ws;                         // constraints in the working set              (kmax)
  mjtNum* M;                       // rows inv(L)*a of the working set            (kmax x n)
  mjtNum* C;                       // Cholesky factor of K = M*M' + compliance    (kmax x kmax)
  int nfixed;                      // number of held or yielding constraints with thresholds
  mjtNum* fixed;                   // sum of threshold * inv(L)*a over those      (n)
  mjtNum pivot2;                   // squared pivot of the last qpAppend
  mjtNum* y;                       // scratch                                     (kmax)
  mjtNum* p;                       // scratch                                     (kmax)
  mjtNum* d;                       // scratch                                     (kmax)
  mjtNum* row;                     // scratch                                     (n)
  mjtNum* r;                       // scratch                                     (n)
  mjtNum* z;                       // scratch                                     (n)
} mjQP;


// return 1 if constraint c is soft
static int qpSoft(const mjQP* q, int c) {
  return q->compliance && q->compliance[c] > 0;
}


// return the multiplier threshold of constraint c
static mjtNum qpThreshold(const mjQP* q, int c) {
  return q->threshold ? q->threshold[c] : 0;
}


// return 1 if state is yielding
static int qpYield(int state) {
  return state == mjQP_YIELD || state == -mjQP_YIELD;
}


// get lower and upper bounds of constraint c, absent bounds are -/+ mjMAXVAL
static void qpBounds(const mjQP* q, int c, mjtNum* lo, mjtNum* hi) {
  const mjtNum* lower = c < q->n ? q->lower : q->alower;
  const mjtNum* upper = c < q->n ? q->upper : q->aupper;
  int i = c < q->n ? c : c - q->n;
  *lo = lower ? mju_max(lower[i], -mjMAXVAL) : -mjMAXVAL;
  *hi = upper ? mju_min(upper[i], mjMAXVAL) : mjMAXVAL;
}


// get the range of the multiplier of constraint c in a given state
static void qpRange(const mjQP* q, int c, int state, mjtNum* lo, mjtNum* hi) {
  mjtNum w = qpThreshold(q, c);
  int soft = qpSoft(q, c);
  switch (state) {
  case mjQP_EQUALITY:
    *lo = soft ? -w : -mjQP_INF;
    *hi = soft ? w : mjQP_INF;
    break;
  case mjQP_RIGID:
    *lo = 0;
    *hi = soft ? w : mjQP_INF;
    break;
  case -mjQP_RIGID:
    *lo = soft ? -w : -mjQP_INF;
    *hi = 0;
    break;
  case mjQP_YIELD:
    *lo = 0;
    *hi = mjQP_INF;
    break;
  default:  // -mjQP_YIELD
    *lo = -mjQP_INF;
    *hi = 0;
  }
}


// linear term of the dual objective for position i of the working set
static mjtNum qpLinear(const mjQP* q, int i) {
  int c = q->ws[i];
  int state = q->state[c];
  mjtNum lo, hi;
  qpBounds(q, c, &lo, &hi);

  // distance of the unconstrained minimizer from the bound
  mjtNum lin = (state > 0 ? hi : lo) - q->cx[c];

  // fixed multipliers
  if (q->nfixed) {
    lin += mju_dot(q->M+i*q->n, q->fixed, q->n);
  }

  return lin;
}


// row = inv(L) * a, where a is the row of constraint c
static void qpRow(const mjQP* q, int c, mjtNum* row) {
  int n = q->n;
  const mjtNum* L = q->L;

  // bound: unit vector, leading zeros are preserved
  int start = 0;
  if (c < n) {
    mju_zero(row, n);
    row[c] = 1;
    start = c;
  } else {
    mju_copy(row, q->A + (c-n)*n, n);
  }

  // forward substitution
  for (int i=start; i < n; i++) {
    if (i > start) {
      row[i] -= mju_dot(L+i*n+start, row+start, i-start);
    }
    row[i] /= L[i*(n+1)];
  }
}


// res = inv(K) * res, where K = C*C' is restricted to the working set
static void qpSolve(const mjQP* q, mjtNum* res, int transpose_only) {
  int k = q->nact, kmax = q->kmax;
  const mjtNum* C = q->C;

  // forward substitution: solve C*res = res
  if (!transpose_only) {
    for (int i=0; i < k; i++) {
      if (i) {
        res[i] -= mju_dot(C+i*kmax, res, i);
      }
      res[i] /= C[i*(kmax+1)];
    }
  }

  // backward substitution: solve C'*res = res
  for (int i=k-1; i >= 0; i--) {
    res[i] /= C[i*(kmax+1)];
    if (i && res[i]) {
      mju_addToScl(res, C+i*kmax, -res[i], i);
    }
  }
}


// add constraint c to the working set in a given state, return 0 if it is dependent
//  if dependent and not forced: nothing is added, y is the projection inv(C)*M*row
static int qpAppend(mjQP* q, int c, int state, const mjtNum* row, int force) {
  int n = q->n, k = q->nact, kmax = q->kmax;
  mjtNum* C = q->C;
  mjtNum* y = q->y;

  // diagonal of K
  mjtNum diag = mju_dot(row, row, n);
  int yield = qpYield(state);
  if (yield) {
    diag += q->compliance[c];
  }

  // y = inv(C) * M * row
  for (int i=0; i < k; i++) {
    y[i] = mju_dot(q->M+i*n, row, n);
    if (i) {
      y[i] -= mju_dot(C+i*kmax, y, i);
    }
    y[i] /= C[i*(kmax+1)];
  }

  // squared pivot
  mjtNum pivot2 = diag - mju_dot(y, y, k);

  // small pivot: recompute without cancellation
  //  pivot2 = min_p |row - M'*p|^2 + p'*D*p, where D are compliances
  //  the minimizer is p = inv(K)*M*row, computed with one step of iterative refinement
  if (k && pivot2 < mjQP_REFINE*diag) {
    mju_copy(q->p, y, k);
    qpSolve(q, q->p, /*transpose_only=*/1);
    for (int pass=0; pass < 2; pass++) {
      // r = row - M'*p
      mju_copy(q->r, row, n);
      pivot2 = yield ? q->compliance[c] : 0;
      for (int i=0; i < k; i++) {
        mju_addToScl(q->r, q->M+i*n, -q->p[i], n);
        if (qpYield(q->state[q->ws[i]])) {
          pivot2 += q->compliance[q->ws[i]]*q->p[i]*q->p[i];
        }
      }
      pivot2 += mju_dot(q->r, q->r, n);

      // refine: p += inv(K) * (M*r - D*p)
      if (pass == 0) {
        mjtNum* d = q->d;
        for (int i=0; i < k; i++) {
          d[i] = mju_dot(q->M+i*n, q->r, n);
          if (qpYield(q->state[q->ws[i]])) {
            d[i] -= q->compliance[q->ws[i]]*q->p[i];
          }
        }
        qpSolve(q, d, /*transpose_only=*/0);
        mju_addTo(q->p, d, k);
      }
    }
  }
  q->pivot2 = pivot2;

  // dependent if the pivot is relatively small or n constraints already hold, unless yielding
  if (!yield && !force && (pivot2 <= mjQP_PIVOTTOL*diag || q->nrigid >= n)) {
    return 0;
  }
  pivot2 = mju_max(pivot2, mjQP_MINPIVOT*diag);
  if (pivot2 <= 0) {
    pivot2 = mjMINVAL;
  }

  // append row to M and C, update working set
  mju_copy(q->M+k*n, row, n);
  mju_copy(C+k*kmax, y, k);
  C[k*(kmax+1)] = mju_sqrt(pivot2);
  q->ws[k] = c;
  q->state[c] = state;
  q->nact++;
  q->nrigid += !yield;

  return 1;
}


// remove position p from the working set
static void qpRemove(mjQP* q, int p) {
  int n = q->n, k = q->nact, kmax = q->kmax;
  mjtNum* C = q->C;
  q->nrigid -= !qpYield(q->state[q->ws[p]]);
  q->state[q->ws[p]] = mjQP_INACTIVE;

  // shift up rows below p, row i gains a superdiagonal element
  for (int i=p; i < k-1; i++) {
    q->ws[i] = q->ws[i+1];
    mju_copy(q->M+i*n, q->M+(i+1)*n, n);
    mju_copy(C+i*kmax, C+(i+1)*kmax, i+2);
  }

  // restore lower-triangular form with Givens rotations of columns (i, i+1)
  for (int i=p; i < k-1; i++) {
    mjtNum a = C[i*kmax+i];
    mjtNum b = C[i*kmax+i+1];
    mjtNum r = mju_sqrt(a*a + b*b);
    if (r < mjMINVAL) {
      C[i*kmax+i] = mjMINVAL;
      continue;
    }
    mjtNum cs = a/r;
    mjtNum sn = b/r;
    C[i*kmax+i] = r;
    for (int j=i+1; j < k-1; j++) {
      mjtNum u = C[j*kmax+i];
      mjtNum v = C[j*kmax+i+1];
      C[j*kmax+i] = cs*u + sn*v;
      C[j*kmax+i+1] = cs*v - sn*u;
    }
  }

  q->nact--;
}


// add a fixed multiplier to (count = 1) or remove it from (count = -1) the fixed sum
static void qpFix(mjQP* q, const mjtNum* row, mjtNum lam, int count) {
  q->nfixed += count;

  // nothing fixed: clear accumulated rounding errors
  if (q->nfixed) {
    mju_addToScl(q->fixed, row, count*lam, q->n);
  } else {
    mju_zero(q->fixed, q->n);
  }
}


// the multiplier at position p reached the lower or upper end of its range: change state
static void qpTransition(mjQP* q, int p, int upper) {
  int c = q->ws[p];
  int state = q->state[c];
  mjtNum w = qpThreshold(q, c);
  mjtNum* row = q->z;

  // get bounds, detect equality
  mjtNum lo, hi;
  qpBounds(q, c, &lo, &hi);
  int equality = (lo == hi);

  // remove from the working set
  mju_copy(row, q->M+p*q->n, q->n);
  qpRemove(q, p);
  q->lam[c] = 0;

  // yielding, excess is zero
  if (qpYield(state)) {
    int sign = state > 0 ? 1 : -1;

    // no threshold: equality yields in the other direction, inequality becomes inactive
    if (w <= 0) {
      if (equality) {
        qpAppend(q, c, -state, row, /*force=*/0);
      }
      return;
    }

    // multiplier is at the threshold: becomes rigid, or held if dependent
    q->lam[c] = sign*w;
    qpFix(q, row, sign*w, -1);
    if (!qpAppend(q, c, equality ? mjQP_EQUALITY : sign*mjQP_RIGID, row, /*force=*/0)) {
      qpFix(q, row, sign*w, 1);
      q->state[c] = sign*mjQP_HELD;
    }
  }

  // rigid, multiplier is zero: becomes inactive
  else if (!equality && (state > 0) != upper) {
    return;
  }

  // rigid, multiplier is at the threshold: yields
  else {
    int sign = upper ? 1 : -1;
    qpFix(q, row, sign*w, 1);
    qpAppend(q, c, sign*mjQP_YIELD, row, /*force=*/0);
  }
}


// primal solution given multipliers: x = x0 - inv(L') * (M' * lam + fixed)
static void qpPrimal(const mjQP* q, mjtNum* x) {
  int n = q->n;
  const mjtNum* L = q->L;
  mjtNum* z = q->z;

  // z = M' * lam + fixed
  mju_copy(z, q->fixed, n);
  for (int i=0; i < q->nact; i++) {
    mju_addToScl(z, q->M+i*n, q->lam[q->ws[i]], n);
  }

  // backward substitution: solve L'*z = z
  for (int i=n-1; i >= 0; i--) {
    z[i] /= L[i*(n+1)];
    if (i && z[i]) {
      mju_addToScl(z, L+i*n, -z[i], i);
    }
  }

  mju_sub(x, q->x0, z, n);
}


// unconstrained minimizer of the objective plus the proximal term, constraint values there
static void qpCenter(mjQP* q) {
  int n = q->n;

  // x0 = -inv(H + prox*I) * (g - prox*center)
  mju_copy(q->x0, q->g, n);
  if (q->prox) {
    mju_addToScl(q->x0, q->center, -q->prox, n);
  }
  mju_cholSolve(q->x0, q->L, q->x0, n);
  mju_scl(q->x0, q->x0, -1, n);

  // constraint values, skipping ignored rows
  mju_copy(q->cx, q->x0, n);
  for (int c=n; c < q->nc; c++) {
    if (q->norm[c] || q->state[c] != mjQP_INACTIVE) {
      q->cx[c] = mju_dot(q->A+(c-n)*n, q->x0, n);
    }
  }
}


// initial state of constraint c when activated at its upper or lower bound
static int qpInitState(const mjQP* q, int c, int upper) {
  mjtNum lo, hi;
  qpBounds(q, c, &lo, &hi);

  // soft constraint with zero threshold: yield
  if (qpSoft(q, c) && qpThreshold(q, c) <= 0) {
    return upper ? mjQP_YIELD : -mjQP_YIELD;
  }

  // equality
  if (lo == hi) {
    return mjQP_EQUALITY;
  }

  return upper ? mjQP_RIGID : -mjQP_RIGID;
}


// add constraint c to the working set in a given state, return 0 if infeasible
//  the multiplier moves in the direction of sign, viol is the rate of decrease of the dual
//  while the constraint is dependent on the working set, move along the direction that keeps
//  the working set stationary, until a multiplier reaches the end of its range
static int qpAdd(mjQP* q, int c, int state, mjtNum sign, mjtNum viol) {
  qpRow(q, c, q->row);
  while (!qpAppend(q, c, state, q->row, /*force=*/0)) {
    mjtNum pivot2 = q->pivot2;
    mjtNum diag = mju_dot(q->row, q->row, q->n);

    // direction of working set multipliers: p = -sign * inv(K) * M * row
    mju_scl(q->p, q->y, -sign, q->nact);
    qpSolve(q, q->p, /*transpose_only=*/1);
    mjtNum pmax = 1;
    for (int i=0; i < q->nact; i++) {
      pmax = mju_max(pmax, mju_abs(q->p[i]));
    }

    // step to the minimum along the direction, if curvature is not negligible
    mjtNum step = mjQP_INF;
    mjtNum bound = 0;
    int limit = -3;
    if (pivot2 > mjQP_MINPIVOT*diag && q->nrigid < q->n) {
      step = viol / pivot2;
      limit = -2;
    }

    // step to the end of the range of the added multiplier
    mjtNum lo, hi;
    qpRange(q, c, state, &lo, &hi);
    mjtNum end = sign > 0 ? hi : lo;
    if (end < mjQP_INF && end > -mjQP_INF && mju_abs(end - q->lam[c]) < step) {
      step = mju_abs(end - q->lam[c]);
      bound = end;
      limit = -1;
    }

    // step to the end of the range of a multiplier in the working set
    int limitupper = 0;
    for (int i=0; i < q->nact; i++) {
      if (mju_abs(q->p[i]) <= mjQP_DIRTOL*pmax) {
        continue;
      }
      qpRange(q, q->ws[i], q->state[q->ws[i]], &lo, &hi);
      end = q->p[i] > 0 ? hi : lo;
      if (end >= mjQP_INF || end <= -mjQP_INF) {
        continue;
      }
      mjtNum t = mju_max(0, (end - q->lam[q->ws[i]]) / q->p[i]);
      if (t < step) {
        step = t;
        bound = end;
        limit = i;
        limitupper = q->p[i] > 0;
      }
    }

    // unbounded dual: infeasible
    if (limit == -3) {
      return 0;
    }

    // take step, keep multipliers within their ranges
    for (int i=0; i < q->nact; i++) {
      qpRange(q, q->ws[i], q->state[q->ws[i]], &lo, &hi);
      q->lam[q->ws[i]] = mju_clip(q->lam[q->ws[i]] + step * q->p[i], lo, hi);
    }
    q->lam[c] += sign * step;
    viol -= pivot2 * step;

    // minimum along the direction: add with a small pivot
    if (limit == -2) {
      qpAppend(q, c, state, q->row, /*force=*/1);
      break;
    }

    // added multiplier at the end of its range: becomes inactive or yields
    else if (limit == -1) {
      q->lam[c] = 0;
      if (bound == 0) {
        break;
      }
      qpFix(q, q->row, bound, 1);
      state = bound > 0 ? mjQP_YIELD : -mjQP_YIELD;
    }

    // multiplier in the working set at the end of its range: change its state
    else {
      q->lam[q->ws[limit]] = bound;
      qpTransition(q, limit, limitupper);
    }
  }

  return 1;
}


// value of constraint c at x, mag is the sum of the absolute values of its terms
static mjtNum qpValue(const mjQP* q, int c, const mjtNum* x, mjtNum* mag) {
  int n = q->n;
  if (c < n) {
    *mag = mju_abs(x[c]);
    return x[c];
  }
  const mjtNum* row = q->A + (c-n)*n;
  mjtNum val = 0;
  *mag = 0;
  for (int i=0; i < n; i++) {
    mjtNum term = row[i]*x[i];
    val += term;
    *mag += mju_abs(term);
  }
  return val;
}


// scale of the tolerances of constraint c: the larger magnitude of its value and of its bound
//  a fraction of the size of x keeps them above the rounding error of the solution: xsize is
//  the size of x0 plus the size of x times the condition number of the factorized Hessian
static mjtNum qpScale(const mjQP* q, int c, mjtNum mag, mjtNum bound, mjtNum xsize) {
  return mju_max(mag, mju_abs(bound)) + mjQP_XSCALE*mju_abs(q->norm[c])*xsize;
}


// size of x for the scale of tolerances: the largest element of the unconstrained minimizer,
// plus the largest element of x times the condition number of the factorized Hessian
static mjtNum qpSizeX(const mjQP* q, const mjtNum* x) {
  mjtNum x0max = 0;
  mjtNum xmax = 0;
  for (int i=0; i < q->n; i++) {
    x0max = mju_max(x0max, mju_abs(q->x0[i]));
    xmax = mju_max(xmax, mju_abs(x[i]));
  }
  return x0max + q->cond*xmax;
}


// step along d from the feasible point x, if d has no curvature and decreases the objective
//  return 0: d has curvature or does not decrease the objective
//         1: step is the distance to the nearest bound that d approaches, of constraint
//            block at its upper or lower bound
//         2: d approaches no bound, the objective is unbounded
//  bounds of soft constraints count, their penalty is quadratic
//  d is the difference of x and another point, its rounding error scales with their size
//  ignored constraints depend on the working set and do not change along d
static int qpRay(const mjQP* q, const mjtNum* H, mjtNum hmax, const mjtNum* x, const mjtNum* d,
                 mjtNum* step, int* block, int* upperblock) {
  int n = q->n;
  mjtNum dnorm = mju_norm(d, n);
  if (dnorm == 0) {
    return 0;
  }

  // no curvature: H*d = 0
  mulSymVec(q->r, H, d, n);
  for (int i=0; i < n; i++) {
    if (mju_abs(q->r[i]) > mjQP_DIRTOL*hmax*dnorm) {
      return 0;
    }
  }

  // decrease: g'*d < 0
  if (mju_dot(q->g, d, n) >= -mjQP_DIRTOL*mju_norm(q->g, n)*dnorm) {
    return 0;
  }

  // size of the points that d is the difference of
  mjtNum xmax = 0;
  for (int i=0; i < n; i++) {
    xmax = mju_max(xmax, mju_max(mju_abs(x[i]), mju_abs(x[i] - d[i])));
  }

  // nearest bound that is approached
  int blocked = 0;
  for (int c=0; c < q->nc; c++) {
    if (q->norm[c] < 0) {
      continue;
    }
    mjtNum lo, hi;
    qpBounds(q, c, &lo, &hi);
    const mjtNum* row = c < n ? NULL : q->A + (c-n)*n;
    mjtNum rate = row ? mju_dot(row, d, n) : d[c];
    mjtNum tol = (mjQP_DIRTOL*dnorm + mjQP_ROUND*xmax)*(row ? mju_norm(row, n) : 1);
    int upper = hi < mjMAXVAL && rate > tol;
    int lower = lo > -mjMAXVAL && rate < -tol;
    if (!upper && !lower) {
      continue;
    }
    mjtNum mag;
    mjtNum val = qpValue(q, c, x, &mag);
    mjtNum dist = mju_max(0, (upper ? hi - val : val - lo) / mju_abs(rate));
    if (!blocked || dist < *step) {
      *step = dist;
      *block = c;
      *upperblock = upper;
    }
    blocked = 1;
  }

  return blocked ? 1 : 2;
}


// proximal weight of a linear program for which a step g/prox has the given size
static mjtNum qpWeight(const mjQP* q, mjtNum size) {
  mjtNum gmax = 0;
  for (int i=0; i < q->n; i++) {
    gmax = mju_max(gmax, mju_abs(q->g[i]));
  }
  return (gmax > 0 && size > 0) ? gmax/size : 1;
}


// largest distance of a bound from the origin, 0 if there are no bounds
static mjtNum qpSize(const mjQP* q) {
  int n = q->n;
  mjtNum size = 0;
  for (int c=0; c < q->nc; c++) {
    mjtNum lo, hi;
    qpBounds(q, c, &lo, &hi);
    mjtNum norm = c < n ? 1 : mju_norm(q->A + (c-n)*n, n);
    if (norm == 0) {
      continue;
    }
    if (hi < mjMAXVAL) {
      size = mju_max(size, mju_abs(hi)/norm);
    }
    if (lo > -mjMAXVAL) {
      size = mju_max(size, mju_abs(lo)/norm);
    }
  }
  return size;
}


// change the proximal weight of a linear program, where L = sqrt(prox)*I
//  rows of the working set and fixed multipliers scale, the factor of the working set is rebuilt
static void qpReweight(mjQP* q, mjtNum prox) {
  int n = q->n, k = q->nact;
  mjtNum ratio = mju_sqrt(q->prox/prox);
  q->prox = prox;
  for (int i=0; i < n; i++) {
    q->L[i*(n+1)] = mju_sqrt(prox);
  }
  mju_scl(q->fixed, q->fixed, ratio, n);

  q->nact = 0;
  q->nrigid = 0;
  for (int i=0; i < k; i++) {
    int c = q->ws[i];
    mju_scl(q->row, q->M+i*n, ratio, n);
    qpAppend(q, c, q->state[c], q->row, /*force=*/1);
  }
}


// solve the quadratic program
//   min 0.5*x'*H*x + x'*g  s.t.  lower <= x <= upper,  alower <= A*x <= aupper
// with a dual active-set method, return number of active constraints or negative status
int mju_QP(mjtNum* res, mjtNum* dual, int* active, mjtNum* work,
           const mjtNum* H, const mjtNum* g, int n,
           const mjtNum* lower, const mjtNum* upper,
           const mjtNum* A, const mjtNum* alower, const mjtNum* aupper, int m,
           const mjtNum* threshold, const mjtNum* compliance) {
  // basic checks
  if (n <= 0) {
    mjERROR("problem size n must be positive");
  }
  if (m < 0 || (m > 0 && !A)) {
    mjERROR("m must be non-negative and A must be given if m is positive");
  }
  if (!res || !H || !g) {
    mjERROR("res, H and g must be given");
  }
  int nc = n + m;

  // maximum size of the working set: independent hard constraints and all soft constraints
  int kmax = nc;
  if (!work) {
    int nsoft = 0;
    for (int c=0; c < nc && compliance; c++) {
      nsoft += compliance[c] > 0;
    }
    kmax = mjMIN(nc, n + nsoft);
  }

  // allocate workspace if not given
  mjtNum* alloc = NULL;
  if (!work) {
    size_t size = (size_t)n*n + (size_t)kmax*(n + kmax + 4) + 4*(size_t)nc + 5*(size_t)n;
    alloc = (mjtNum*) mju_malloc(size*sizeof(mjtNum));
    if (!alloc) {
      mjERROR("could not allocate memory");
    }
    work = alloc;
  }

  // assign problem definition and carve workspace
  mjQP q;
  q.n = n;
  q.m = m;
  q.nc = nc;
  q.kmax = kmax;
  q.lower = lower;
  q.upper = upper;
  q.A = A;
  q.alower = alower;
  q.aupper = aupper;
  q.threshold = threshold;
  q.compliance = compliance;
  q.nact = 0;
  q.nrigid = 0;
  q.nfixed = 0;
  q.pivot2 = 0;
  q.g = g;
  q.prox = 0;
  q.z = res;
  q.L = work;             work += (size_t)n*n;
  q.M = work;             work += (size_t)kmax*n;
  q.C = work;             work += (size_t)kmax*kmax;
  q.y = work;             work += kmax;
  q.p = work;             work += kmax;
  q.d = work;             work += kmax;
  q.x0 = work;            work += n;
  q.row = work;           work += n;
  q.r = work;             work += n;
  q.fixed = work;         work += n;
  q.center = work;        work += n;
  q.cx = work;            work += nc;
  q.norm = work;          work += nc;
  q.lam = work;           work += nc;
  q.state = (int*) work;  work += nc;
  q.ws = (int*) work;

  // factorize H, only the lower triangle is read
  mjtNum hmax = 0;
  int linear = 1;
  for (int i=0; i < n; i++) {
    mju_copy(q.L+i*n, H+i*n, i+1);
    hmax = mju_max(hmax, H[i*(n+1)]);
    for (int j=0; j <= i; j++) {
      linear = linear && H[i*n+j] == 0;
    }
  }
  mjtNum mindiag = mjQP_SINGTOL*hmax;
  int rank = hmax > 0 ? mju_cholFactor(q.L, n, mindiag) : 0;

  // singular H, or with a pivot below the proximal weight: add a proximal term, to be iterated
  // to convergence
  if (rank < n && hmax > 0) {
    q.prox = mjQP_PROX*hmax;
    for (int i=0; i < n; i++) {
      mju_copy(q.L+i*n, H+i*n, i+1);
      q.L[i*(n+1)] += q.prox;
    }

    // pivots of a positive semidefinite H are now at least prox, up to rounding error
    rank = mju_cholFactor(q.L, n, q.prox/2);
  }

  // linear program, H = 0: the first step is as long as the largest distance of a bound
  if (linear) {
    q.prox = qpWeight(&q, qpSize(&q));
    mju_zero(q.L, n*n);
    for (int i=0; i < n; i++) {
      q.L[i*(n+1)] = mju_sqrt(q.prox);
    }
    rank = n;
  }

  // condition number of the factorized Hessian, from the squared pivots
  mjtNum pivotmin = q.L[0]*q.L[0];
  mjtNum pivotmax = pivotmin;
  for (int i=1; i < n; i++) {
    mjtNum pivot = q.L[i*(n+1)]*q.L[i*(n+1)];
    pivotmin = mju_min(pivotmin, pivot);
    pivotmax = mju_max(pivotmax, pivot);
  }
  q.cond = pivotmin > 0 ? pivotmax/pivotmin : 1;

  // indefinite H
  if (rank < n) {
    mju_zero(res, n);
    if (dual) mju_zero(dual, nc);
    if (active) mju_zeroInt(active, nc);
    mju_free(alloc);
    return mjQP_NOT_PSD;
  }

  // row norms, initial state
  int feasible = 1;
  for (int c=0; c < nc; c++) {
    q.norm[c] = c < n ? 1 : mju_norm(A+(c-n)*n, n);
    q.lam[c] = 0;
    q.state[c] = mjQP_INACTIVE;

    // crossed bounds or hard zero row with bounds that exclude zero: infeasible
    mjtNum lo, hi;
    qpBounds(&q, c, &lo, &hi);
    if (lo > hi || (q.norm[c] == 0 && !qpSoft(&q, c) && (lo > 0 || hi < 0))) {
      feasible = 0;
    }
  }

  // unconstrained minimizer, constraint values
  mju_zero(q.center, n);
  mju_zero(q.fixed, n);
  qpCenter(&q);

  // warmstart: add independent constraints to the working set with zero multipliers
  for (int c=0; c < nc && active && feasible; c++) {
    mjtNum lo, hi;
    qpBounds(&q, c, &lo, &hi);
    int isupper = active[c] > 0;
    if (!active[c] || q.norm[c] == 0 || (isupper ? hi >= mjMAXVAL : lo <= -mjMAXVAL)) {
      continue;
    }
    qpRow(&q, c, q.row);
    qpAppend(&q, c, qpInitState(&q, c, isupper), q.row, /*force=*/0);
  }

  // main loop
  int status = mjQP_MAX_ITER;
  int maxiter = mjQP_ITERBASE + mjQP_ITERSCALE*nc;
  int last = -1;
  int laststate = mjQP_INACTIVE;
  int proxiter = 0;
  for (int iter=0; iter < maxiter && feasible; iter++) {
    // stationary multipliers of the working set: solve K*y = -linear
    for (int i=0; i < q.nact; i++) {
      q.y[i] = -qpLinear(&q, i);
    }
    qpSolve(&q, q.y, /*transpose_only=*/0);

    // largest step towards y that keeps multipliers within their ranges
    mjtNum alpha = 1;
    mjtNum bound = 0;
    int block = -1;
    int blockupper = 0;
    for (int i=0; i < q.nact; i++) {
      mjtNum lo, hi;
      qpRange(&q, q.ws[i], q.state[q.ws[i]], &lo, &hi);
      if (q.y[i] >= lo && q.y[i] <= hi) {
        continue;
      }
      mjtNum lam = q.lam[q.ws[i]];
      mjtNum end = q.y[i] < lo ? lo : hi;
      mjtNum a = mju_clip((end - lam) / (q.y[i] - lam), 0, 1);
      if (block < 0 || a < alpha) {
        alpha = a;
        bound = end;
        block = i;
        blockupper = q.y[i] > hi;
      }
    }

    // blocked: take partial step, change state of blocking constraint
    if (block >= 0) {
      for (int i=0; i < q.nact; i++) {
        q.lam[q.ws[i]] += alpha * (q.y[i] - q.lam[q.ws[i]]);
      }
      int c = q.ws[block];
      q.lam[c] = bound;
      qpTransition(&q, block, blockupper);

      // the last change of state was reverted without progress: rounding error, ignore constraint
      if (alpha == 0 && c == last && q.state[c] == laststate) {
        q.norm[c] = -mju_abs(q.norm[c]);
      }
      last = -1;
      continue;
    }

    // not blocked: take full step, update primal solution
    for (int i=0; i < q.nact; i++) {
      q.lam[q.ws[i]] = q.y[i];
    }
    qpPrimal(&q, res);

    // find the largest violation of the optimality conditions
    //  inactive constraint: violated bound
    //  held constraint: violated bound or multiplier that holds a satisfied bound
    int add = -1;
    int addupper = 0;
    int addsmall = 0;
    mjtNum addviol = 0;
    mjtNum maxviol = 0;
    mjtNum xsize = qpSizeX(&q, res);
    for (int c=0; c < nc; c++) {
      int state = q.state[c];
      int held = (state == mjQP_HELD || state == -mjQP_HELD);
      if ((state != mjQP_INACTIVE && !held) || q.norm[c] == 0) {
        continue;
      }
      mjtNum lo, hi;
      qpBounds(&q, c, &lo, &hi);

      // constraint value and its magnitude, the sum of absolute values of its terms
      mjtNum mag;
      mjtNum val = qpValue(&q, c, res, &mag);
      mjtNum tol = held ? mjQP_DEPTOL : mjQP_FEASTOL;

      // signed distance beyond the upper bound, or beyond the lower bound
      mjtNum viol = 0;
      mjtNum edge = 0;
      int isupper = 0;
      if (hi < mjMAXVAL && (state > 0 || (!held && val > hi))) {
        viol = val - hi;
        edge = hi;
        isupper = 1;
      } else if (lo > -mjMAXVAL && (state < 0 || (!held && val < lo))) {
        viol = lo - val;
        edge = lo;
      }

      // tolerances are relative to the larger magnitude of the value and of the bound
      //  a fraction of the size of x keeps them above the rounding error of the solution
      mjtNum scale = qpScale(&q, c, mag, edge, xsize);

      // ignored: restore if the violation is no longer small
      if (q.norm[c] < 0) {
        if (mju_abs(viol) <= mjQP_DEPTOL*scale) {
          continue;
        }
        q.norm[c] = -q.norm[c];
      }

      // within tolerance, or inactive and satisfied
      if (mju_abs(viol) <= tol*scale || (!held && viol < 0)) {
        continue;
      }

      if (mju_abs(viol) > maxviol*q.norm[c]) {
        maxviol = mju_abs(viol) / q.norm[c];
        addviol = viol;
        addsmall = mju_abs(viol) <= mjQP_DEPTOL*scale;
        add = c;
        addupper = isupper;
      }
    }

    // optimal with a proximal term: done if stationary without it, otherwise move the center
    //  the proximal term adds prox*(x - center) to the gradient
    if (add < 0 && q.prox) {
      mjtNum change = 0;
      mjtNum scale = 0;
      for (int i=0; i < n; i++) {
        change = mju_max(change, mju_abs(res[i] - q.center[i]));
        scale = mju_max(scale, mju_abs(g[i]) + q.prox*mju_abs(res[i]));
      }
      if (q.prox*change > mjQP_PROXTOL*scale) {
        // the step from the center decreases the objective without bound: unbounded
        mjtNum step = 0;
        int rayblock = -1;
        int rayupper = 0;
        mju_sub(q.row, res, q.center, n);
        int ray = qpRay(&q, H, hmax, res, q.row, &step, &rayblock, &rayupper);
        if (ray == 2) {
          status = mjQP_UNBOUNDED;
          break;
        }

        // the step is blocked by an inactive constraint at its bound: activate it
        if (ray == 1 && step == 0 && q.state[rayblock] == mjQP_INACTIVE) {
          add = rayblock;
          addupper = rayupper;
          addsmall = 1;
          addviol = 0;
        }

        // iteration limit: accept a slowly converging sequence, fail if unbounded
        else if (++proxiter >= mjQP_PROXITER) {
          if (q.prox*change > mjQP_PROXSLOW*scale) {
            break;
          }
        } else {
          // move the center to the solution, and on to the nearest bound along the step
          mju_copy(q.center, res, n);
          if (ray == 1) {
            mju_addToScl(q.center, q.row, step, n);
          }

          // linear program: keep a step as long as the center is large
          if (linear) {
            mjtNum size = 0;
            for (int i=0; i < n; i++) {
              size = mju_max(size, mju_abs(q.center[i]));
            }
            mjtNum prox = qpWeight(&q, size);
            if (size > 0 && (prox > 10*q.prox || prox < q.prox/10)) {
              qpReweight(&q, prox);
            }
          }

          qpCenter(&q);
          last = -1;
          iter = -1;
          continue;
        }
      }
    }

    // optimal: done
    if (add < 0) {
      status = 0;
      for (int c=0; c < nc; c++) {
        status += q.state[c] != mjQP_INACTIVE;
      }
      break;
    }

    // inactive: activate at the violated bound
    mjtNum sign = addupper ? 1 : -1;
    last = add;
    laststate = q.state[add];
    if (q.state[add] == mjQP_INACTIVE) {
      // small violation of a dependent constraint: rounding error, ignore this constraint
      qpRow(&q, add, q.row);
      int state = qpInitState(&q, add, addupper);
      if (addsmall && !qpYield(state)) {
        if (!qpAppend(&q, add, state, q.row, /*force=*/0)) {
          q.norm[add] = -q.norm[add];
        }
        continue;
      }
      feasible = qpAdd(&q, add, state, sign, addviol);
    }

    // held and violated: yield, multiplier remains fixed at the threshold
    else if (addviol > 0) {
      qpRow(&q, add, q.row);
      q.lam[add] = 0;
      qpAppend(&q, add, addupper ? mjQP_YIELD : -mjQP_YIELD, q.row, /*force=*/0);
    }

    // held and satisfied: reduce multiplier
    else {
      qpRow(&q, add, q.row);
      qpFix(&q, q.row, q.lam[add], -1);
      mjtNum lo, hi;
      qpBounds(&q, add, &lo, &hi);
      int state = lo == hi ? mjQP_EQUALITY : (addupper ? mjQP_RIGID : -mjQP_RIGID);
      q.state[add] = mjQP_INACTIVE;
      feasible = qpAdd(&q, add, state, -sign, -addviol);
    }
  }

  // failure: report primal solution of the current multipliers
  if (!feasible) {
    status = mjQP_INFEASIBLE;
  }
  if (status < 0) {
    qpPrimal(&q, res);
  }

  // copy outputs, equality and held states are reported as rigid with the sign of the multiplier
  for (int c=0; c < nc; c++) {
    int state = q.state[c];
    if (dual) {
      dual[c] = q.lam[c];
      if (qpYield(state)) {
        dual[c] += state > 0 ? qpThreshold(&q, c) : -qpThreshold(&q, c);
      }
    }
    if (active) {
      if (state == mjQP_EQUALITY || state == mjQP_HELD || state == -mjQP_HELD) {
        state = q.lam[c] < 0 ? -mjQP_RIGID : mjQP_RIGID;
      }
      active[c] = state;
    }
  }

  mju_free(alloc);
  return status;
}
