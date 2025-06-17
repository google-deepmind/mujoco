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

#include "engine/engine_util_sparse.h"
#include "engine/engine_util_sparse_avx.h"  // IWYU pragma: keep

#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjtnum.h>
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_misc.h"


//------------------------------ sparse operations -------------------------------------------------


// dot-productX3, first vector is sparse; supernode of size 3
void mju_dotSparseX3(mjtNum* res0, mjtNum* res1, mjtNum* res2,
                     const mjtNum* vec10, const mjtNum* vec11, const mjtNum* vec12,
                     const mjtNum* vec2, int nnz1, const int* ind1) {
#ifdef mjUSEAVX
  mju_dotSparseX3_avx(res0, res1, res2, vec10, vec11, vec12, vec2, nnz1, ind1);
#else
  int i = 0;

  // clear result
  mjtNum RES0 = 0;
  mjtNum RES1 = 0;
  mjtNum RES2 = 0;

  for (; i < nnz1; i++) {
    mjtNum v2 = vec2[ind1[i]];

    RES0 += vec10[i] * v2;
    RES1 += vec11[i] * v2;
    RES2 += vec12[i] * v2;
  }

  // copy result
  *res0 = RES0;
  *res1 = RES1;
  *res2 = RES2;
#endif  // mjUSEAVX
}



// dot-product, both vectors are sparse
mjtNum mju_dotSparse2(const mjtNum* vec1, const int* ind1, int nnz1,
                      const mjtNum* vec2, const int* ind2, int nnz2) {
  int i1 = 0, i2 = 0;
  mjtNum res = 0;

  // check for empty array
  if (!nnz1 || !nnz2) {
    return 0;
  }

  while (i1 < nnz1 && i2 < nnz2) {
    // get current indices
    int adr1 = ind1[i1], adr2 = ind2[i2];

    // match: accumulate result, advance both
    if (adr1 == adr2) {
      res += vec1[i1++] * vec2[i2++];
    }

    // otherwise advance smaller
    else if (adr1 < adr2) {
      i1++;
    } else {
      i2++;
    }
  }

  return res;
}



// convert matrix from dense to sparse
//  nnz is size of res and colind, return 1 if too small, 0 otherwise
int mju_dense2sparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                     int* rownnz, int* rowadr, int* colind, int nnz) {
  if (nnz <= 0) {
    return 1;
  }

  int adr = 0;

  // find non-zeros and construct sparse
  for (int r=0; r < nr; r++) {
    // init row
    rownnz[r] = 0;
    rowadr[r] = adr;

    // find non-zeros
    for (int c=0; c < nc; c++) {
      if (mat[r*nc+c]) {
        // check for out of bounds
        if (adr >= nnz) {
          return 1;
        }

        // record index and count
        colind[adr] = c;
        rownnz[r]++;

        // copy element
        res[adr++] = mat[r*nc+c];
      }
    }
  }
  return 0;
}



// convert matrix from sparse to dense
void mju_sparse2dense(mjtNum* res, const mjtNum* mat, int nr, int nc,
                      const int* rownnz, const int* rowadr, const int* colind) {
  // clear
  mju_zero(res, nr*nc);

  // copy non-zeros
  for (int r=0; r < nr; r++) {
    for (int i=0; i < rownnz[r]; i++) {
      res[r*nc + colind[rowadr[r]+i]] = mat[rowadr[r]+i];
    }
  }
}



// multiply sparse matrix and dense vector:  res = mat * vec.
void mju_mulMatVecSparse(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                         int nr, const int* rownnz, const int* rowadr,
                         const int* colind, const int* rowsuper) {
#ifdef mjUSEAVX
  mju_mulMatVecSparse_avx(res, mat, vec, nr, rownnz, rowadr, colind, rowsuper);
#else
  // regular sparse dot-product
  for (int r=0; r < nr; r++) {
    res[r] = mju_dotSparse(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
  }
#endif  // mjUSEAVX
}



// multiply transposed sparse matrix and dense vector:  res = mat' * vec.
void mju_mulMatTVecSparse(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int nr, int nc,
                          const int* rownnz, const int* rowadr, const int* colind) {
  // clear res
  mju_zero(res, nc);

  for (int i=0; i < nr; i++) {
    mjtNum scl = vec[i];

    // skip if 0
    if (!scl) continue;

    // add row scaled by the corresponding vector element
    int nnz = rownnz[i];
    int adr = rowadr[i];
    const int* ind = colind + adr;
    const mjtNum* row = mat + adr;
    for (int j=0; j < nnz; j++) {
      res[ind[j]] += row[j] * scl;
    }
  }
}


// add sparse matrix M to sparse destination matrix, requires pre-allocated buffers
void mju_addToMatSparse(mjtNum* dst, int* rownnz, int* rowadr, int* colind, int nr,
                        const mjtNum* M, const int* M_rownnz, const int* M_rowadr,
                        const int* M_colind,
                        mjtNum* buf_val, int* buf_ind) {
  for (int i=0; i < nr; i++) {
    rownnz[i] = mju_combineSparse(dst + rowadr[i], M + M_rowadr[i], 1, 1,
                                  rownnz[i], M_rownnz[i], colind + rowadr[i],
                                  M_colind + M_rowadr[i], buf_val, buf_ind);
  }
}


// add symmetric matrix (lower triangle) to dense matrix, upper triangle optional
void mju_addToSymSparse(mjtNum* res, const mjtNum* mat, int n,
                        const int* rownnz, const int* rowadr, const int* colind, int flg_upper) {
  for (int i=0; i < n; i++) {
    int start = rowadr[i];
    int end = start + rownnz[i];
    for (int adr=start; adr < end; adr++) {
      mjtNum val = mat[adr];
      int j = colind[adr];

      // lower + diagonal
      res[i*n + j] += val;

      // strict upper
      if (flg_upper && j < i) {
        res[j*n + i] += val;
      }
    }
  }
}



// multiply symmetric matrix (only lower triangle represented) by vector:
//  res = (mat + strict_upper(mat')) * vec
void mju_mulSymVecSparse(mjtNum* restrict res, const mjtNum* restrict mat,
                         const mjtNum* restrict vec, int n,
                         const int* restrict rownnz, const int* restrict rowadr,
                         const int* restrict colind) {
  // clear res
  mju_zero(res, n);

  // multiply
  for (int i=0; i < n; i++) {
    int adr = rowadr[i];
    int diag = rownnz[i] - 1;
    const mjtNum* row = mat + adr;

    // diagonal
    res[i] = row[diag] * vec[i];

    // off-diagonals
    const int* ind = colind + adr;
    for (int k=diag-1; k >= 0; k--) {
      int j = ind[k];
      mjtNum val = row[k];
      res[i] += val * vec[j]; // strict lower
      res[j] += val * vec[i]; // strict upper
    }
  }
}



// count the number of non-zeros in the sum of two sparse vectors
int mju_combineSparseCount(int a_nnz, int b_nnz, const int* a_ind, const int* b_ind) {
  int a = 0, b = 0, c_nnz = 0;

  // count c_nnz: nonzero indices common to both a and b
  while (a < a_nnz && b < b_nnz) {
    // common index, increment everything
    if (a_ind[a] == b_ind[b]) {
      c_nnz++;
      a++;
      b++;
    }

    // update smallest index
    else if (a_ind[a] < b_ind[b]) {
      a++;
    } else {
      b++;
    }
  }

  // union minus the intersection
  return a_nnz + b_nnz - c_nnz;
}



// incomplete combine sparse: dst = a*dst + b*src at common indices
void mju_combineSparseInc(mjtNum* dst, const mjtNum* src, int n, mjtNum a, mjtNum b,
                          int dst_nnz, int src_nnz, const int* dst_ind, const int* src_ind) {
  // check for identical pattern
  if (dst_nnz == src_nnz) {
    if (mju_compare(dst_ind, src_ind, dst_nnz)) {
      // combine mjtNum data directly
      mju_addToSclScl(dst, src, a, b, dst_nnz);
      return;
    }
  }

  // scale dst by a
  if (a != 1) {
    mju_scl(dst, dst, a, dst_nnz);
  }

  // prepare to merge
  int di = 0, si = 0;
  int dadr = di < dst_nnz ? dst_ind[di] : n+1;
  int sadr = si < src_nnz ? src_ind[si] : n+1;

  // add src*b at common indices
  while (di < dst_nnz) {
    // both
    if (dadr == sadr) {
      dst[di++] += b*src[si++];

      dadr = di < dst_nnz ? dst_ind[di] : n+1;
      sadr = si < src_nnz ? src_ind[si] : n+1;
    }

    // dst only
    else if (dadr < sadr) {
      di++;
      dadr = di < dst_nnz ? dst_ind[di] : n+1;
    }

    // src only
    else {
      si++;
      sadr = si < src_nnz ? src_ind[si] : n+1;
    }
  }
}



// dst += scl*src, only at common non-zero indices
void mju_addToSclSparseInc(mjtNum* dst, const mjtNum* src,
                           int nnzdst, const int* inddst,
                           int nnzsrc, const int* indsrc, mjtNum scl) {
  if (!nnzdst || !nnzsrc) {
    return;
  }

  int adrs = 0, adrd = 0, inds = indsrc[0], indd = inddst[0];
  while (1) {
    // common non-zero index
    if (inds == indd) {
      // add
      dst[adrd] += scl * src[adrs];

      // advance src
      if (++adrs < nnzsrc) {
        inds = indsrc[adrs];
      } else {
        return;
      }

      // advance dst
      if (++adrd < nnzdst) {
        indd = inddst[adrd];
      } else {
        return;
      }
    }

    // src non-zero index smaller: advance src
    else if (inds < indd) {
      if (++adrs < nnzsrc) {
        inds = indsrc[adrs];
      } else {
        return;
      }
    }

    // dst non-zero index smaller: advance dst
    else {
      if (++adrd < nnzdst) {
        indd = inddst[adrd];
      } else {
        return;
      }
    }
  }
}



// add to sparse matrix: dst = dst + scl*src, return nnz of result
int mju_addToSparseMat(mjtNum* dst, const mjtNum* src, int n, int nrow, mjtNum scl,
                       int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind,
                       mjtNum* buf, int* buf_ind) {
  // check for identical pattern
  if (dst_nnz == src_nnz) {
    if (dst_nnz == 0) {
      return 0;
    }
    if (mju_compare(dst_ind, src_ind, dst_nnz)) {
      // combine mjtNum data directly
      mju_addToScl(dst, src, scl, nrow*dst_nnz);
      return dst_nnz;
    }
  }

  // prepare to merge scr and dst into buf^T
  int si = 0, di = 0, nnz = 0;
  int sadr = src_nnz ? src_ind[0] : n+1;
  int dadr = dst_nnz ? dst_ind[0] : n+1;

  // merge matrices
  while (si < src_nnz || di < dst_nnz) {
    // both
    if (sadr == dadr) {
      for (int k=0; k < nrow; k++) {
        buf[nrow*nnz + k] = dst[di + k*dst_nnz] + scl*src[si + k*src_nnz];
      }

      buf_ind[nnz++] = sadr;
      si++;
      di++;
      sadr = si < src_nnz ? src_ind[si] : n+1;
      dadr = di < dst_nnz ? dst_ind[di] : n+1;
    }

    // dst only
    else if (dadr < sadr) {
      for (int k=0; k < nrow; k++) {
        buf[nrow*nnz + k] = dst[di + k*dst_nnz];
      }

      buf_ind[nnz++] = dadr;
      di++;
      dadr = di < dst_nnz ? dst_ind[di] : n+1;
    }

    // src only
    else {
      for (int k=0; k < nrow; k++) {
        buf[nrow*nnz + k] = scl*src[si + k*src_nnz];
      }

      buf_ind[nnz++] = sadr;
      si++;
      sadr = si < src_nnz ? src_ind[si] : n+1;
    }
  }

  // copy transposed buf into dst
  mju_transpose(dst, buf, nnz, nrow);
  mju_copyInt(dst_ind, buf_ind, nnz);

  return nnz;
}



// add(merge) two chains
int mju_addChains(int* res, int n, int NV1, int NV2,
                  const int* chain1, const int* chain2) {
  // check for identical pattern
  if (NV1 == NV2) {
    if (NV1 == 0) {
      return 0;
    }
    if (mju_compare(chain1, chain2, NV1)) {
      mju_copyInt(res, chain1, NV1);
      return NV1;
    }
  }

  // prepare to merge
  int i1 = 0, i2 = 0, NV = 0;
  int adr1 = NV1 ? chain1[0] : n+1;
  int adr2 = NV2 ? chain2[0] : n+1;

  // merge chains
  while (i1 < NV1 || i2 < NV2) {
    // both
    if (adr1 == adr2) {
      res[NV++] = adr1;
      i1++;
      i2++;
      adr1 = i1 < NV1 ? chain1[i1] : n+1;
      adr2 = i2 < NV2 ? chain2[i2] : n+1;
    }

    // chain1 only
    else if (adr1 < adr2) {
      res[NV++] = adr1;
      i1++;
      adr1 = i1 < NV1 ? chain1[i1] : n+1;
    }

    // chain2 only
    else {
      res[NV++] = adr2;
      i2++;
      adr2 = i2 < NV2 ? chain2[i2] : n+1;
    }
  }

  return NV;
}



// compress sparse matrix, remove elements with abs(value) <= minval, return total non-zeros
int mju_compressSparse(mjtNum* mat, int nr, int nc, int* rownnz, int* rowadr, int* colind,
                       mjtNum minval) {
  int remove_small = minval >= 0;
  int adr = 0;
  for (int r=0; r < nr; r++) {
    // save old rowadr, record new
    int rowadr_old = rowadr[r];
    rowadr[r] = adr;

    // shift mat and colind
    int nnz = 0;
    int end = rowadr_old + rownnz[r];
    for (int adr_old=rowadr_old; adr_old < end; adr_old++) {
      if (remove_small && mju_abs(mat[adr_old]) <= minval) {
        continue;
      }
      if (adr != adr_old) {
        mat[adr] = mat[adr_old];
        colind[adr] = colind[adr_old];
      }
      adr++;
      if (remove_small) nnz++;
    }
    if (remove_small) rownnz[r] = nnz;
  }

  return rowadr[nr-1] + rownnz[nr-1];
}



// transpose sparse matrix, optionally compute row supernodes
void mju_transposeSparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                         int* res_rownnz, int* res_rowadr, int* res_colind, int* res_rowsuper,
                         const int* rownnz, const int* rowadr, const int* colind) {
  // clear number of non-zeros for each row of transposed
  mju_zeroInt(res_rownnz, nc);

  // count the number of non-zeros for each row of the transposed matrix
  for (int r = 0; r < nr; r++) {
    int start = rowadr[r];
    int end = start + rownnz[r];
    for (int j = start; j < end; j++) {
      res_rownnz[colind[j]]++;
    }
  }

  // init res_rowsuper
  if (res_rowsuper) {
    for (int i = 0; i < nc - 1; i++) {
      res_rowsuper[i] = (res_rownnz[i] == res_rownnz[i + 1]);
    }
    res_rowsuper[nc - 1] = 0;
  }

  // compute the row addresses for the transposed matrix
  res_rowadr[0] = 0;
  for (int i = 1; i < nc; i++) {
    res_rowadr[i] = res_rowadr[i-1] + res_rownnz[i-1];
  }

  // iterate through each row (column) of mat (res)
  for (int r = 0; r < nr; r++) {
    int c_prev = -1;
    int start = rowadr[r];
    int end = start + rownnz[r];
    for (int i = start; i < end; i++) {
      // swap rows with columns and increment res_rowadr
      int c = colind[i];
      int adr = res_rowadr[c]++;
      res_colind[adr] = r;
      if (res) {
        res[adr] = mat[i];
      }

      // mark non-supernodes
      if (res_rowsuper) {
        if (c > 0 && c != c_prev + 1 && res_rowsuper[c - 1]) {
          res_rowsuper[c - 1] = 0;
        }
        c_prev = c;
      }
    }
  }

  // shift back row addresses
  for (int i = nc-1; i > 0; i--) {
    res_rowadr[i] = res_rowadr[i-1];
  }
  res_rowadr[0] = 0;

  // accumulate supernodes
  if (res_rowsuper) {
    for (int i = nc - 2; i >= 0; i--) {
      if (res_rowsuper[i]) {
        res_rowsuper[i] += res_rowsuper[i + 1];
      }
    }
  }
}



// construct row supernodes
void mju_superSparse(int nr, int* rowsuper,
                     const int* rownnz, const int* rowadr, const int* colind) {
  // no rows: nothing to do
  if (!nr) {
    return;
  }

  // find match to child
  for (int r=0; r < nr-1; r++) {
    // different number of nonzeros: cannot be a match
    if (rownnz[r] != rownnz[r+1]) {
      rowsuper[r] = 0;
    }

    // same number of nonzeros: compare colind vectors
    else {
      rowsuper[r] = mju_compare(colind+rowadr[r], colind+rowadr[r+1], rownnz[r]);
    }
  }

  // clear last (by definition)
  rowsuper[nr-1] = 0;

  // accumulate in reverse
  for (int r=nr-2; r >= 0; r--) {
    if (rowsuper[r]) {
      rowsuper[r] += rowsuper[r+1];
    }
  }
}


// precount res_rownnz and precompute res_rowadr for mju_sqrMatTDSparse, return total non-zeros
int mju_sqrMatTDSparseCount(int* res_rownnz, int* res_rowadr, int nr,
                            const int* rownnz, const int* rowadr, const int* colind,
                            const int* rownnzT, const int* rowadrT, const int* colindT,
                            const int* rowsuperT, mjData* d, int flg_upper) {
  mj_markStack(d);
  int* chain = mjSTACKALLOC(d, 2*nr, int);
  int nchain = 0;
  int* res_colind = NULL;

  for (int r=0; r < nr; r++) {
    // supernode; copy everything to next row
    if (rowsuperT && r > 0 && rowsuperT[r-1] > 0) {
      res_rownnz[r] = res_rownnz[r - 1];

      // fill in upper triangle
      if (flg_upper) {
        for (int j=0; j < nchain; j++) {
          res_rownnz[res_colind[j]]++;
        }
      }

      // update chain with diagonal
      if (rownnzT[r]) {
        res_colind[nchain++] = r;
        res_rownnz[r]++;
      }
    } else {
      int inew = 0, iold = nr;
      nchain = 0;

      for (int i=0; i < rownnzT[r]; i++) {
        int c = colindT[rowadrT[r] + i];

        int adr = inew;
        inew = iold;
        iold = adr;

        int nnewchain = 0;
        adr = 0;
        int end = rowadr[c] + rownnz[c];
        for (int adr1=rowadr[c]; adr1 < end; adr1++) {
          int col_mat = colind[adr1];
          while (adr < nchain && chain[iold + adr] < col_mat && chain[iold + adr] <= r) {
            chain[inew + nnewchain++] = chain[iold + adr++];
          }

          // skip upper triangle
          if (col_mat > r) {
            break;
          }

          if (adr < nchain && chain[iold + adr] == col_mat) {
            adr++;
          }
          chain[inew + nnewchain++] = col_mat;
        }

        while (adr < nchain && chain[iold + adr] <= r) {
          chain[inew + nnewchain++] = chain[iold + adr++];
        }
        nchain = nnewchain;
      }

      // only computed for lower triangle
      res_rownnz[r] = nchain;
      res_colind = chain + inew;

      // update upper triangle
      if (flg_upper) {
        int nchain_end = nchain;

        // avoid double counting
        if (nchain > 0 && res_colind[nchain-1] == r) {
          nchain_end = nchain - 1;
        }

        for (int j=0; j < nchain_end; j++) {
          res_rownnz[res_colind[j]]++;
        }
      }
    }
  }

  res_rowadr[0] = 0;
  for (int r = 1; r < nr; r++) {
    res_rowadr[r] = res_rowadr[r-1] + res_rownnz[r-1];
  }

  mj_freeStack(d);

  return res_rowadr[nr-1] + res_rownnz[nr-1];
}


// precompute res_rowadr for mju_sqrMatTDSparse using uncompressed memory
void mju_sqrMatTDUncompressedInit(int* res_rowadr, int nc) {
  for (int r=0; r < nc; r++) {
    res_rowadr[r] = r*nc;
  }
}



// max number of supernodes handled
#define mjMAXSUPER 8

// compute sparse M'*diag*M (diag=NULL: compute M'*M), res_rowadr must be precomputed
void mju_sqrMatTDSparse(mjtNum* res, const mjtNum* mat, const mjtNum* matT,
                        const mjtNum* diag, int nr, int nc,
                        int* res_rownnz, const int* res_rowadr, int* res_colind,
                        const int* rownnz, const int* rowadr,
                        const int* colind, const int* rowsuper,
                        const int* rownnzT, const int* rowadrT,
                        const int* colindT, const int* rowsuperT,
                        mjData* d, int* diagind) {
  mj_markStack(d);

  // reinterpret transposed matrices as compressed sparse column
  const mjtNum* mat_csc  = matT;
  const int* colnnz      = rownnzT;
  const int* coladr      = rowadrT;
  const int* rowind      = colindT;
  const int* colsuper    = rowsuperT;
  const mjtNum* matT_csc = mat;
  const int* colnnzT     = rownnz;
  const int* coladrT     = rowadr;
  const int* rowindT     = colind;
  // rowsuper is unused

  // marker[i] = 1 if row i is set in current column
  int* marker = mjSTACKALLOC(d, nc, int);
  mju_zeroInt(marker, nc);

  // dense buffer (considered column-major) containing up to mjMAXSUPER columns
  mjtNum* buffer = mjSTACKALLOC(d, nc*mjMAXSUPER, mjtNum);

  // dense index vector of the current column (unsorted)
  int* buffer_idx = mjSTACKALLOC(d, nc, int);

  // rowstart[i]: address of first row in column mat'[:, i] with index > current column
  int* rowstart = mjSTACKALLOC(d, nr, int);
  mju_zeroInt(rowstart, nr);

  // clear res_rownnz
  mju_zeroInt(res_rownnz, nc);

  // construct res[lower+diagonal], by column
  for (int c=0; c < nc; c++) {
    int buffer_nnz = 0;

    // prepare column c of mat
    int nnz = colnnz[c];
    int adr = coladr[c];
    const int* ind = rowind + adr;

    // val: array of ns > 0 column pointers with identical pattern to c
    const mjtNum* val[mjMAXSUPER];

    // first column is c
    int ns = 1;
    val[0] = mat_csc + adr;

    // add c's supernodes, if any
    int cs;
    if (colsuper && (cs = colsuper[c])) {
      ns += mjMIN(cs, mjMAXSUPER - 1);
      for (int s=1; s < ns; s++) {
        val[s] = mat_csc + coladr[c + s];
      }
    }

    // diagonal special-case: dense dot product of column c, with/out diag
    mjtNum diag_c[mjMAXSUPER];
    if (diag) {
      for (int s=0; s < ns; s++) {
        mjtNum ds = 0;
        for (int k=0; k < nnz; k++) {
          ds += (val[s][k] * val[s][k]) * diag[ind[k]];
        }
        diag_c[s] = ds;
      }
    } else {
      for (int s=0; s < ns; s++) {
        diag_c[s] = mju_dot(val[s], val[s], nnz);
      }
    }

    // in the strict lower triangle, compute
    // res[:, c] = mat' * mat[:, c] = sum_r(diag[r] * mat'[:, r] * mat[:, c])
    for (int i=0; i < nnz; i++) {
      // prepare column r of mat'
      int r = ind[i];
      int adrT = coladrT[r];
      int nnzT = colnnzT[r];
      const int* indT = rowindT + adrT;
      const mjtNum* valT = matT_csc + adrT;

      // get v[s] = diag[r] * mat[r, c + s] for s in [0, ns)
      mjtNum v[mjMAXSUPER];
      if (diag) {
        mjtNum diag_r = diag[r];
        for (int s=0; s < ns; s++) {
          v[s] = diag_r * val[s][i];
        }
      } else {
        for (int s=0; s < ns; s++) {
          v[s] = val[s][i];
        }
      }

      // gather to dense buffer columns:  buffer[:, s] += mat'[:, r] * v[s]
      for (int k=rowstart[r]; k < nnzT; k++) {
        int j = indT[k];

        // if j is not in the strict lower triangle, increment rowstart and continue
        if (j <= c) {
          rowstart[r]++;
          continue;
        }

        // first nonzero in row j: mark and set value
        if (!marker[j]) {
          // mark j and save it
          marker[j] = 1;
          buffer_idx[buffer_nnz++] = j;

          // set value
          mjtNum vk = valT[k];
          for (int s=0; s < ns; s++) {
            buffer[s*nc + j] = vk * v[s];
          }
        }

        // otherwise existing nonzero in row j: add to value
        else {
          mjtNum vk = valT[k];
          for (int s=0; s < ns; s++) {
            buffer[s*nc + j] += vk * v[s];
          }
        }
      }
    }

    // scatter to res from dense buffer:  res[:, c + s] = buffer[:, s] for s in [0, ns)

    // write values under diagonal
    for (int i=0; i < buffer_nnz; i++) {
      int j = buffer_idx[i];
      marker[j] = 0;
      int adr_j = res_rowadr[j] + res_rownnz[j];

      // truncate row to strict lower triangle
      int lower = j - c;
      int nm = mjMIN(ns, lower);

      // increment nonzeros
      res_rownnz[j] += nm;

      // write value
      for (int s=0; s < nm; s++) {
        res[adr_j + s] = buffer[s*nc + j];
      }

      // write index
      for (int s=0; s < nm; s++) {
        res_colind[adr_j + s] = c + s;
      }
    }

    // write diagonal value
    for (int s=0; s < ns; s++) {
      int adr_s = res_rowadr[c + s] + res_rownnz[c + s]++;
      res_colind[adr_s] = c + s;
      res[adr_s] = diag_c[s];
    }

    // supernode: skip ahead if ns > 1
    c += ns - 1;
  }

  // upper triangle requested: save diagonal indices and fill
  if (diagind) {
    // save diagonal indices
    for (int i=0; i < nc; i++) {
      diagind[i] = res_rowadr[i] + res_rownnz[i] - 1;
    }

    // fill upper triangle
    for (int i=0; i < nc; i++) {
      int start = res_rowadr[i];
      int end = start + res_rownnz[i] - 1;
      for (int j=start; j < end; j++) {
        int adr = res_rowadr[res_colind[j]] + res_rownnz[res_colind[j]]++;
        res[adr] = res[j];
        res_colind[adr] = i;
      }
    }
  }

  mj_freeStack(d);
}

#undef mjMAXSUPER



// legacy row-based implementation (reference)
void mju_sqrMatTDSparse_row(mjtNum* res, const mjtNum* mat, const mjtNum* matT,
                            const mjtNum* diag, int nr, int nc,
                            int* res_rownnz, const int* res_rowadr, int* res_colind,
                            const int* rownnz, const int* rowadr,
                            const int* colind, const int* rowsuper,
                            const int* rownnzT, const int* rowadrT,
                            const int* colindT, const int* rowsuperT,
                            mjData* d, int* diagind) {
  // allocate space for accumulation buffer and matT
  mj_markStack(d);

  // a dense row buffer that stores the current row in the resulting matrix
  mjtNum* buffer = mjSTACKALLOC(d, nc, mjtNum);

  // these mark the currently set columns in the dense row buffer,
  // used for when creating the resulting sparse row
  int* markers = mjSTACKALLOC(d, nc, int);

  for (int i=0; i < nc; i++) {
    int rowadr_i = res_rowadr[i];
    int* cols = res_colind + rowadr_i;

    res_rownnz[i] = 0;
    buffer[i] = 0;
    markers[i] = 0;

    // if rowsuper, use the previous row sparsity structure
    if (rowsuperT && i > 0 && rowsuperT[i-1]) {
      res_rownnz[i] = res_rownnz[i-1];
      mju_copyInt(cols, res_colind+res_rowadr[i-1], res_rownnz[i]);
    }

    // iterate through each row of M'
    int adrT = rowadrT[i];
    int end_r = adrT + rownnzT[i];
    for (int r = adrT; r < end_r; r++) {
      int t = colindT[r];
      int adr = rowadr[t];
      int end_c = adr + rownnz[t];
      for (int c=adr; c < end_c; c++) {
        int cc = colind[c];

        // ignore upper triangle
        if (cc > i) {
          break;
        }

        // add value to buffer
        if (diag) {
          buffer[cc] += matT[r] * diag[t] * mat[c];
        } else {
          buffer[cc] += matT[r] * mat[c];
        }

        // only need to insert nnz if not marked
        if (!markers[cc]) {
          markers[cc] = 1;

          // since i is the rightmost column, it can be inserted at the end
          if (cc == i) {
            cols[res_rownnz[i]++] = cc;
            continue;
          }

          // insert col in order via binary search
          int l = 0, h = res_rownnz[i];
          while (l < h) {
            int m = (l + h) >> 1;
            if (cols[m] < cc) {
              l = m + 1;
            } else {
              h = m;
            }
          }

          // cc is the rightmost column so far, it can be inserted at the end
          if (l == res_rownnz[i]) {
            cols[l] = cc;
            res_rownnz[i]++;
            continue;
          }

          // move the cols to the right
          h = res_rownnz[i];
          while (l < h) {
            cols[h] = cols[h-1];
            h--;
          }

          // insert
          cols[l] = cc;
          res_rownnz[i]++;
        }
      }
    }

    end_r = res_rownnz[i];

    // rowsuperT: reuse sparsity, copy into res
    if (rowsuperT && rowsuperT[i]) {
      for (int r=0; r < end_r; r++) {
        int c = cols[r];
        res[rowadr_i + r] = buffer[c];
        buffer[c] = 0;
      }
    }

    // clear out buffers, sparsity cannot be reused
    else {
      for (int r=0; r < end_r; r++) {
        int c = cols[r];
        int adr = rowadr_i + r;
        res[adr] = buffer[c];
        res_colind[adr] = c;
        buffer[c] = 0;
        markers[c] = 0;
      }
    }
  }


  // diagonal indices requested: fill upper triangle
  if (diagind) {
    // save diagonal indices
    for (int i=0; i < nc; i++) {
      diagind[i] = res_rowadr[i] + res_rownnz[i] - 1;
    }

    // fill upper triangle
    for (int i=0; i < nc; i++) {
      int start = res_rowadr[i];
      int end = start + res_rownnz[i] - 1;
      for (int j=start; j < end; j++) {
        int adr = res_rowadr[res_colind[j]] + res_rownnz[res_colind[j]]++;
        res[adr] = res[j];
        res_colind[adr] = i;
      }
    }
  }

  mj_freeStack(d);
}



// block-diagonalize a dense matrix
//   res         output matrix
//   mat         input matrix
//   nc_mat      number of columns in mat
//   nc_res      number of columns in res
//   nb          number of blocks
//   perm_r      reverse permutation of rows (res -> mat)
//   perm_c      reverse permutation of columns (res -> mat)
//   block_nr    number of rows in each block
//   block_nc    number of columns in each block
//   block_r     first row of each block
//   block_c     first column of each block
void mju_blockDiag(mjtNum* restrict res, const mjtNum* restrict mat,
                   int nc_mat, int nc_res, int nb,
                   const int* restrict perm_r, const int* restrict perm_c,
                   const int* restrict block_nr, const int* restrict block_nc,
                   const int* restrict block_r, const int* restrict block_c) {
  for (int b=0; b < nb; b++) {
    int bnr = block_nr[b];
    int bnc = block_nc[b];
    const int* adr_r = perm_r + block_r[b];
    const int* adr_c = perm_c + block_c[b];
    int adr = nc_res * block_r[b];
    for (int r = 0; r < bnr; r++) {
      for (int c = 0; c < bnc; c++) {
        res[adr++] = mat[nc_mat * adr_r[r] + adr_c[c]];
      }
    }
  }
}


// block-diagonalize a sparse matrix
//   res         values of the target matrix res
//   res_rownnz  number of non-zeros in each row of res
//   res_rowadr  row address of each non-zero in res
//   res_colind  column index of each non-zero in res
//   mat         values of the source matrix mat
//   mat_rownnz  number of non-zeros in each row of mat
//   mat_rowadr  row address of each non-zero in mat
//   mat_colind  column index of each non-zero in mat
//   nr          number of rows in mat/res
//   nb          number of blocks
//   perm_r      reverse permutation of rows (res -> mat)
//   perm_c      forward permutation of columns (mat -> res)
//   block_r     first row of each block in res
//   block_c     first column of each block in res
//   mat2        optional additional source matrix (same structure as mat)
//   res2        optional additional target matrix (same structure as res)
void mju_blockDiagSparse(mjtNum* restrict res, int* restrict res_rownnz,
                         int* restrict res_rowadr, int* restrict res_colind,
                         const mjtNum* restrict mat, const int* restrict rownnz,
                         const int* restrict rowadr, const int* restrict colind,
                         int nr, int nb,
                         const int* restrict perm_r, const int* restrict perm_c,
                         const int* restrict block_r, const int* restrict block_c,
                         mjtNum* restrict res2, const mjtNum* restrict mat2) {
  int block = 0;
  int col_offset = block_c[block];
  int row_next = block + 1 < nb ? block_r[block + 1] : nr;
  for (int r=0; r < nr; r++) {
    // row k in mat goes to row r in res
    int k = perm_r[r];

    // rownnz
    int nnz = rownnz[k];
    res_rownnz[r] = nnz;

    // rowadr
    int res_adr = (r == 0) ? 0 : res_rowadr[r-1] + res_rownnz[r-1];
    res_rowadr[r] = res_adr;

    // colind
    int* res_colind_r = res_colind + res_adr;
    mjtNum* res_r = res + res_adr;
    int mat_adr = rowadr[k];
    const int* colind_k = colind + mat_adr;
    const mjtNum* mat_k = mat + mat_adr;
    for (int j=0; j < nnz; j++) {
      res_colind_r[j] = perm_c[colind_k[j]] - col_offset;
    }

    // values (dense copy: partial order within block is guaranteed)
    mju_copy(res_r, mat_k, nnz);
    if (mat2 && res2) {
      mju_copy(res2 + res_adr, mat2 + mat_adr, nnz);
    }

    // end of block reached: update block counter, column offset, next row
    if (r + 1 >= row_next && block + 1 < nb ) {
      block++;
      col_offset = block_c[block];
      row_next = block + 1 < nb ? block_r[block + 1] : nr;
    }
  }
}

