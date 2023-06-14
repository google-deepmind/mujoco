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
#include "engine/engine_util_sparse_avx.h"

#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjtnum.h>
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"


//------------------------------ sparse operations -------------------------------------------------

// dot-product, first vector is sparse
mjtNum mju_dotSparse(const mjtNum* vec1, const mjtNum* vec2,
                     const int nnz1, const int* ind1) {
#ifdef mjUSEAVX
  return mju_dotSparse_avx(vec1, vec2, nnz1, ind1);
#else
  int i = 0;
  mjtNum res = 0;
  int n_4 = nnz1 - 4;
  mjtNum res0 = 0;
  mjtNum res1 = 0;
  mjtNum res2 = 0;
  mjtNum res3 = 0;

  for (; i <= n_4; i+=4) {
    res0 += vec1[i+0] * vec2[ind1[i+0]];
    res1 += vec1[i+1] * vec2[ind1[i+1]];
    res2 += vec1[i+2] * vec2[ind1[i+2]];
    res3 += vec1[i+3] * vec2[ind1[i+3]];
  }
  res = (res0 + res2) + (res1 + res3);

  // scalar part
  for (; i < nnz1; i++) {
    res += vec1[i] * vec2[ind1[i]];
  }

  return res;
#endif  // mjUSEAVX
}



// dot-productX3, first vector is sparse; supernode of size 3
void mju_dotSparseX3(mjtNum* res0, mjtNum* res1, mjtNum* res2,
                     const mjtNum* vec10, const mjtNum* vec11, const mjtNum* vec12,
                     const mjtNum* vec2, const int nnz1, const int* ind1) {
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
mjtNum mju_dotSparse2(const mjtNum* vec1, const mjtNum* vec2,
                      const int nnz1, const int* ind1,
                      const int nnz2, const int* ind2) {
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
void mju_dense2sparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                      int* rownnz, int* rowadr, int* colind) {
  int adr = 0;

  // find non-zeros and construct sparse
  for (int r=0; r < nr; r++) {
    // init row
    rownnz[r] = 0;
    rowadr[r] = adr;

    // find non-zeros
    for (int c=0; c < nc; c++) {
      if (mat[r*nc+c]) {
        // record index and count
        colind[adr] = c;
        rownnz[r]++;

        // copy element
        res[adr++] = mat[r*nc+c];
      }
    }
  }
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



// res = res*scl1 + vec*scl2
static void mju_addToSclScl(mjtNum* res, const mjtNum* vec, mjtNum scl1, mjtNum scl2, int n) {
#ifdef mjUSEAVX
  mju_addToSclScl_avx(res, vec, scl1, scl2, n);
#else
  for (int i=0; i < n; i++) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
  }
#endif  // mjUSEAVX
}



// return 1 if vec1==vec2, 0 otherwise
static int mju_compare(const int* vec1, const int* vec2, int n) {
#ifdef mjUSEAVX
  return mju_compare_avx(vec1, vec2, n);
#else
  return !memcmp(vec1, vec2, n*sizeof(int));
#endif  // mjUSEAVX
}



// combine two sparse vectors: dst = a*dst + b*src, return nnz of result
int mju_combineSparse(mjtNum* dst, const mjtNum* src, int n, mjtNum a, mjtNum b,
                      int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind,
                      mjtNum* buf, int* buf_ind) {
  // check for identical pattern
  if (dst_nnz == src_nnz) {
    if (mju_compare(dst_ind, src_ind, dst_nnz)) {
      // combine mjtNum data directly
      mju_addToSclScl(dst, src, a, b, dst_nnz);
      return dst_nnz;
    }
  }

  // copy dst into buf
  if (dst_nnz) {
    memcpy(buf, dst, dst_nnz*sizeof(mjtNum));
    memcpy(buf_ind, dst_ind, dst_nnz*sizeof(int));
  }

  // prepare to merge buf and scr into dst
  int bi = 0, si = 0, nnz = 0;
  int buf_nnz = dst_nnz;
  int badr = bi < buf_nnz ? buf_ind[bi] : n+1;
  int sadr = si < src_nnz ? src_ind[si] : n+1;

  // merge vectors
  while (bi < buf_nnz || si < src_nnz) {
    // both
    if (badr == sadr) {
      dst[nnz] = a*buf[bi++] + b*src[si++];
      dst_ind[nnz++] = badr;

      badr = bi < buf_nnz ? buf_ind[bi] : n+1;
      sadr = si < src_nnz ? src_ind[si] : n+1;
    }

    // dst only
    else if (badr < sadr) {
      dst[nnz] = a*buf[bi++];
      dst_ind[nnz++] = badr;

      badr = bi < buf_nnz ? buf_ind[bi] : n+1;
    }

    // src only
    else {
      dst[nnz] = b*src[si++];
      dst_ind[nnz++] = sadr;

      sadr = si < src_nnz ? src_ind[si] : n+1;
    }
  }

  return nnz;
}



// incomplete combine sparse: dst = a*dst + b*src at common indices
void mju_combineSparseInc(mjtNum* dst, const mjtNum* src, int n, mjtNum a, mjtNum b,
                          int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind) {
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



// compress layout of sparse matrix
void mju_compressSparse(mjtNum* mat, int nr, int nc, int* rownnz, int* rowadr, int* colind) {
  rowadr[0] = 0;
  int adr = rownnz[0];
  for (int r=1; r < nr; r++) {
    // save old rowadr, record new
    int rowadr1 = rowadr[r];
    rowadr[r] = adr;

    // shift mat and mat_colind
    for (int adr1=rowadr1; adr1 < rowadr1+rownnz[r]; adr1++) {
      mat[adr] = mat[adr1];
      colind[adr] = colind[adr1];
      adr++;
    }
  }
}



// transpose sparse matrix
void mju_transposeSparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                         int* res_rownnz, int* res_rowadr, int* res_colind,
                         const int* rownnz, const int* rowadr, const int* colind) {
  // clear number of non-zeros for each row of transposed
  memset(res_rownnz, 0, nc*sizeof(int));

  // total number of non-zeros of mat
  int nnz = rowadr[nr-1] + rownnz[nr-1];

  // count the number of non-zeros for each row of the transposed matrix
  for (int i = 0; i < nnz; i++) {
    res_rownnz[colind[i]]++;
  }

  // compute the row addresses for the transposed matrix
  res_rowadr[0] = 0;
  for (int i = 1; i < nc; i++) {
    res_rowadr[i] = res_rowadr[i-1] + res_rownnz[i-1];
  }

  // r holds the current row in mat
  int r = 0;

  // iterate through each non-zero entry of mat
  for (int i = 0; i < nnz; i++) {
    // iterate to get to the current row (skipping rows with all zeros)
    while ((i-rowadr[r]) >= rownnz[r]) r++;

    // swap rows with columns and increment res_rowadr
    int c = res_rowadr[colind[i]]++;
    res[c] = mat[i];
    res_colind[c] = r;
  }

  // shift back row addresses
  for (int i = nc-1; i > 0; i--) {
    res_rowadr[i] = res_rowadr[i-1];
  }

  res_rowadr[0] = 0;
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


// precount res_rownnz and precompute res_rowadr for mju_sqrMatTDSparse
void mju_sqrMatTDSparseInit(int* res_rownnz, int* res_rowadr,
                            const mjtNum* mat, const mjtNum* matT,
                            int nr, int nc,  const int* rownnz,
                            const int* rowadr, const int* colind,
                            const int* rownnzT, const int* rowadrT,
                            const int* colindT, const int* rowsuperT,
                            mjData* d) {
  mjMARKSTACK;
  int* chain = mj_stackAllocInt(d, 2*nc);
  int nchain = 0;
  int* res_colind = NULL;

  for (int r=0; r < nc; r++) {

    // supernode; copy everything to next row
    if (rowsuperT && r > 0 && rowsuperT[r-1] > 0) {
      res_rownnz[r] = res_rownnz[r - 1];

      // fill in upper triangle
      for (int j=0; j < nchain; j++) {
        res_rownnz[res_colind[j]]++;
      }

      // update chain with diagonal
      if (rownnzT[r]) {
        res_colind[nchain++] = r;
        res_rownnz[r]++;
      }
    } else {
      int inew = 0, iold = nc;
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
          while (adr < nchain && chain[iold + adr] < col_mat &&
                 chain[iold + adr] <= r) {
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
      int nchain_end = nchain;

      // avoid double counting.
      if (nchain > 0 && res_colind[nchain-1] == r) {
        nchain_end = nchain - 1;
      }

      for (int j=0; j < nchain_end; j++) {
        res_rownnz[res_colind[j]]++;
      }
    }
  }

  res_rowadr[0] = 0;
  for (int r = 1; r < nc; r++) {
    res_rowadr[r] = res_rowadr[r-1] + res_rownnz[r-1];
  }

  mjFREESTACK;
}


// precompute res_rowadr for mju_sqrMatTDSparse using uncompressed memory
void mju_sqrMatTDUncompressedInit(int* res_rowadr, int nc) {
  for (int r=0; r < nc; r++) {
    res_rowadr[r] = r*nc;
  }
}



// compute sparse M'*diag*M (diag=NULL: compute M'*M), res has uncompressed layout
// res_rowadr is required to be precomputed
void mju_sqrMatTDSparse(mjtNum* res, const mjtNum* mat, const mjtNum* matT,
                        const mjtNum* diag, int nr, int nc,
                        int* res_rownnz, int* res_rowadr, int* res_colind,
                        const int* rownnz, const int* rowadr,
                        const int* colind, const int* rowsuper,
                        const int* rownnzT, const int* rowadrT,
                        const int* colindT, const int* rowsuperT,
                        mjData* d) {
  // allocate space for accumulation buffer and matT
  mjMARKSTACK;

  // a dense row buffer that stores the current row in the resulting matrix
  mjtNum* buffer = mj_stackAlloc(d, nc);

  // these mark the currently set columns in the dense row buffer,
  // used for when creating the resulting sparse row
  int* markers = mj_stackAllocInt(d, nc);

  for (int i=0; i < nc; i++) {
    int* cols = res_colind+res_rowadr[i];

    res_rownnz[i] = 0;
    buffer[i] = 0;
    markers[i] = 0;

    // if rowsuper, use the previous row sparsity structure
    if (rowsuperT && i > 0 && rowsuperT[i-1]) {
      res_rownnz[i] = res_rownnz[i-1];
      memcpy(cols, res_colind+res_rowadr[i-1], res_rownnz[i]*sizeof(int));
    }

    // iterate through each row of M'
    int end = rowadrT[i] + rownnzT[i];
    for (int r = rowadrT[i]; r < end; r++) {
      int t = colindT[r];
      mjtNum v = diag ? matT[r] * diag[t] : matT[r];
      for (int c=rowadr[t]; c < rowadr[t]+rownnz[t]; c++) {
        int cc = colind[c];
        // ignore upper triangle
        if (cc > i) {
          break;
        }

        buffer[cc] += v*mat[c];

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

    end = res_rownnz[i];

    // rowsuperT: reuse sparsity, copy into res
    if (rowsuperT && rowsuperT[i]) {
      for (int r=0; r < end; r++) {
        res[res_rowadr[i] + r] = buffer[cols[r]];
        buffer[cols[r]] = 0;
      }
    } else {
      // clear out buffers since sparsity cannot be reused
      for (int r=0; r < end; r++) {
        int cc = cols[r];
        res[res_rowadr[i] + r] = buffer[cc];
        res_colind[res_rowadr[i] + r] = cc;
        buffer[cc] = 0;
        markers[cc] = 0;
      }
    }
  }


  // fill upper triangle
  for (int i=0; i < nc; i++) {
    int end = res_rowadr[i] + res_rownnz[i] - 1;
    for (int j=res_rowadr[i]; j < end; j++) {
      int adr = res_rowadr[res_colind[j]] + res_rownnz[res_colind[j]]++;
      res[adr] = res[j];
      res_colind[adr] = i;
    }
  }

  mjFREESTACK;
}
