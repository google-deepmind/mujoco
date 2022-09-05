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

#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjtnum.h>
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_util_blas.h"

#ifdef mjUSEPLATFORMSIMD
  #if defined(__AVX__) && defined(mjUSEDOUBLE)
    #define mjUSEAVX
    #include "immintrin.h"
  #endif
#endif

//------------------------------ sparse operations -------------------------------------------------

// dot-product, first vector is sparse
mjtNum mju_dotSparse(const mjtNum* vec1, const mjtNum* vec2,
                     const int nnz1, const int* ind1) {
  int i = 0;
  mjtNum res = 0;

#ifdef mjUSEAVX
  int nnz1_4 = nnz1 - 4;

  // vector part
  if (nnz1_4>=0) {
    __m256d sum, prod, val1, val2;
    __m128d vlow, vhigh, high64;

    // init
    val2 = _mm256_set_pd(vec2[ind1[3]],
                         vec2[ind1[2]],
                         vec2[ind1[1]],
                         vec2[ind1[0]]);
    val1 = _mm256_loadu_pd(vec1);
    sum = _mm256_mul_pd(val1, val2);
    i = 4;

    // parallel computation
    while (i<=nnz1_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_set_pd(vec2[ind1[i+3]],
                           vec2[ind1[i+2]],
                           vec2[ind1[i+1]],
                           vec2[ind1[i+0]]);
      prod = _mm256_mul_pd(val1, val2);
      sum = _mm256_add_pd(sum, prod);
      i += 4;
    }

    // reduce
    vlow = _mm256_castpd256_pd128(sum);
    vhigh = _mm256_extractf128_pd(sum, 1);
    vlow = _mm_add_pd(vlow, vhigh);
    high64 = _mm_unpackhi_pd(vlow, vlow);
    res = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));
  }
#endif

  // scalar part
  for (; i<nnz1; i++) {
    res += vec1[i] * vec2[ind1[i]];
  }

  return res;
}



// dot-productX3, first vector is sparse; supernode of size 3
void mju_dotSparseX3(mjtNum* res0, mjtNum* res1, mjtNum* res2,
                     const mjtNum* vec10, const mjtNum* vec11, const mjtNum* vec12,
                     const mjtNum* vec2, const int nnz1, const int* ind1) {
  int i = 0;

  // clear result
  mjtNum RES0 = 0;
  mjtNum RES1 = 0;
  mjtNum RES2 = 0;

#ifdef mjUSEAVX
  int nnz1_4 = nnz1 - 4;

  // vector part
  if (nnz1_4>=0) {
    __m256d sum0, sum1, sum2, prod, val1, val2;
    __m128d vlow, vhigh, high64;

    // init
    val2 = _mm256_set_pd(vec2[ind1[3]],
                         vec2[ind1[2]],
                         vec2[ind1[1]],
                         vec2[ind1[0]]);
    val1 = _mm256_loadu_pd(vec10);
    sum0 = _mm256_mul_pd(val1, val2);
    val1 = _mm256_loadu_pd(vec11);
    sum1 = _mm256_mul_pd(val1, val2);
    val1 = _mm256_loadu_pd(vec12);
    sum2 = _mm256_mul_pd(val1, val2);
    i = 4;

    // parallel computation
    while (i<=nnz1_4) {
      // get val2 only once
      val2 = _mm256_set_pd(vec2[ind1[i+3]],
                           vec2[ind1[i+2]],
                           vec2[ind1[i+1]],
                           vec2[ind1[i+0]]);

      // process each val1
      val1 = _mm256_loadu_pd(vec10+i);
      prod = _mm256_mul_pd(val1, val2);
      sum0 = _mm256_add_pd(sum0, prod);

      val1 = _mm256_loadu_pd(vec11+i);
      prod = _mm256_mul_pd(val1, val2);
      sum1 = _mm256_add_pd(sum1, prod);

      val1 = _mm256_loadu_pd(vec12+i);
      prod = _mm256_mul_pd(val1, val2);
      sum2 = _mm256_add_pd(sum2, prod);

      i += 4;
    }

    // reduce
    vlow   = _mm256_castpd256_pd128(sum0);
    vhigh  = _mm256_extractf128_pd(sum0, 1);
    vlow   = _mm_add_pd(vlow, vhigh);
    high64 = _mm_unpackhi_pd(vlow, vlow);
    RES0   = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));

    vlow   = _mm256_castpd256_pd128(sum1);
    vhigh  = _mm256_extractf128_pd(sum1, 1);
    vlow   = _mm_add_pd(vlow, vhigh);
    high64 = _mm_unpackhi_pd(vlow, vlow);
    RES1   = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));

    vlow   = _mm256_castpd256_pd128(sum2);
    vhigh  = _mm256_extractf128_pd(sum2, 1);
    vlow   = _mm_add_pd(vlow, vhigh);
    high64 = _mm_unpackhi_pd(vlow, vlow);
    RES2   = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));
  }
#endif

  // scalar part
  for (; i<nnz1; i++) {
    mjtNum v2 = vec2[ind1[i]];

    RES0 += vec10[i] * v2;
    RES1 += vec11[i] * v2;
    RES2 += vec12[i] * v2;
  }

  // copy result
  *res0 = RES0;
  *res1 = RES1;
  *res2 = RES2;
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

  while (i1<nnz1 && i2<nnz2) {
    // get current indices
    int adr1 = ind1[i1], adr2 = ind2[i2];

    // match: accumulate result, advance both
    if (adr1==adr2) {
      res += vec1[i1++] * vec2[i2++];
    }

    // otherwise advance smaller
    else if (adr1<adr2) {
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
  for (int r=0; r<nr; r++) {
    // init row
    rownnz[r] = 0;
    rowadr[r] = adr;

    // find non-zeros
    for (int c=0; c<nc; c++) {
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
  for (int r=0; r<nr; r++) {
    for (int i=0; i<rownnz[r]; i++) {
      res[r*nc + colind[rowadr[r]+i]] = mat[rowadr[r]+i];
    }
  }
}



// multiply sparse matrix and dense vector:  res = mat * vec.
void mju_mulMatVecSparse(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                         int nr, const int* rownnz, const int* rowadr,
                         const int* colind, const int* rowsuper) {
  // no supernodes, or no AVX
#ifdef mjUSEAVX
  if (!rowsuper) {
#endif

    // regular sparse dot-product
    for (int r=0; r<nr; r++) {
      res[r] = mju_dotSparse(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
    }

    return;

#ifdef mjUSEAVX
  }
#endif

  // regular or supernode
  for (int r=0; r<nr; r++) {
    if (rowsuper[r]) {
      int rs = rowsuper[r]+1;

      // handle rows in blocks of 3
      while (rs>=3) {
        mju_dotSparseX3(res+r, res+r+1, res+r+2,
                        mat+rowadr[r], mat+rowadr[r+1], mat+rowadr[r+2],
                        vec, rownnz[r], colind+rowadr[r]);

        r += 3;
        rs -= 3;
      }

      // handle remaining rows
      while (rs>0) {
        res[r] = mju_dotSparse(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);

        r++;
        rs--;
      }

      // go back one, because of outer for loop
      r--;
    }

    else {
      res[r] = mju_dotSparse(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
    }
  }
}

// res = res*scl1 + vec*scl2
static void mju_addToSclScl(mjtNum* res, const mjtNum* vec, mjtNum scl1, mjtNum scl2, int n) {
  int i = 0;

#ifdef mjUSEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4>=0) {
    __m256d sclpar1, sclpar2, sum, val1, val2;

    // init
    sclpar1 = _mm256_set1_pd(scl1);
    sclpar2 = _mm256_set1_pd(scl2);

    // parallel computation
    while (i<=n_4) {
      val1 = _mm256_loadu_pd(res+i);
      val2 = _mm256_loadu_pd(vec+i);
      val1 = _mm256_mul_pd(val1, sclpar1);
      val2 = _mm256_mul_pd(val2, sclpar2);
      sum = _mm256_add_pd(val1, val2);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i==3) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
    res[i+1] = res[i+1]*scl1 + vec[i+1]*scl2;
    res[i+2] = res[i+2]*scl1 + vec[i+2]*scl2;
  } else if (n_i==2) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
    res[i+1] = res[i+1]*scl1 + vec[i+1]*scl2;
  } else if (n_i==1) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
  }

#else
  for (; i<n; i++) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
  }
#endif
}

// return 1 if vec1==vec2, 0 otherwise
static int mju_compare(const int* vec1, const int* vec2, int n) {
  int i = 0;

#ifdef mjUSEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4>=0) {
    __m128i val1, val2, cmp;

    // parallel computation
    while (i<=n_4) {
      val1 = _mm_loadu_si128((const __m128i*)(vec1+i));
      val2 = _mm_loadu_si128((const __m128i*)(vec2+i));
      cmp = _mm_cmpeq_epi32(val1, val2);
      if (_mm_movemask_epi8(cmp)!= 0xFFFF) {
        return 0;
      }
      i += 4;
    }
  }
#endif

  // scalar part
  for (; i<n; i++) {
    if (vec1[i]!=vec2[i]) {
      return 0;
    }
  }

  return 1;
}

// combine two sparse vectors: dst = a*dst + b*src, return nnz of result
int mju_combineSparse(mjtNum* dst, const mjtNum* src, int n, mjtNum a, mjtNum b,
                      int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind,
                      mjtNum* buf, int* buf_ind) {
  // check for identical pattern
  if (dst_nnz==src_nnz) {
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
  int badr = bi<buf_nnz ? buf_ind[bi] : n+1;
  int sadr = si<src_nnz ? src_ind[si] : n+1;

  // merge vectors
  while (bi<buf_nnz || si<src_nnz) {
    // both
    if (badr==sadr) {
      dst[nnz] = a*buf[bi++] + b*src[si++];
      dst_ind[nnz++] = badr;

      badr = bi<buf_nnz ? buf_ind[bi] : n+1;
      sadr = si<src_nnz ? src_ind[si] : n+1;
    }

    // dst only
    else if (badr<sadr) {
      dst[nnz] = a*buf[bi++];
      dst_ind[nnz++] = badr;

      badr = bi<buf_nnz ? buf_ind[bi] : n+1;
    }

    // src only
    else {
      dst[nnz] = b*src[si++];
      dst_ind[nnz++] = sadr;

      sadr = si<src_nnz ? src_ind[si] : n+1;
    }
  }

  return nnz;
}



// incomplete combine sparse: dst = a*dst + b*src at common indices
void mju_combineSparseInc(mjtNum* dst, const mjtNum* src, int n, mjtNum a, mjtNum b,
                          int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind) {
  // check for identical pattern
  if (dst_nnz==src_nnz) {
    if (mju_compare(dst_ind, src_ind, dst_nnz)) {
      // combine mjtNum data directly
      mju_addToSclScl(dst, src, a, b, dst_nnz);
      return;
    }
  }

  // scale dst by a
  if (a!=1) {
    mju_scl(dst, dst, a, dst_nnz);
  }

  // prepare to merge
  int di = 0, si = 0;
  int dadr = di<dst_nnz ? dst_ind[di] : n+1;
  int sadr = si<src_nnz ? src_ind[si] : n+1;

  // add src*b at common indices
  while (di<dst_nnz) {
    // both
    if (dadr==sadr) {
      dst[di++] += b*src[si++];

      dadr = di<dst_nnz ? dst_ind[di] : n+1;
      sadr = si<src_nnz ? src_ind[si] : n+1;
    }

    // dst only
    else if (dadr<sadr) {
      di++;
      dadr = di<dst_nnz ? dst_ind[di] : n+1;
    }

    // src only
    else {
      si++;
      sadr = si<src_nnz ? src_ind[si] : n+1;
    }
  }
}



// compress layout of sparse matrix
void mju_compressSparse(mjtNum* mat, int nr, int nc, int* rownnz, int* rowadr, int* colind) {
  rowadr[0] = 0;
  int adr = rownnz[0];
  for (int r=1; r<nr; r++) {
    // save old rowadr, record new
    int rowadr1 = rowadr[r];
    rowadr[r] = adr;

    // shift mat and mat_colind
    for (int adr1=rowadr1; adr1<rowadr1+rownnz[r]; adr1++) {
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
  // clear counters for transposed
  memset(res_rownnz, 0, nc*sizeof(int));

  // set uncompressed layout
  for (int rt=0; rt<nc; rt++) {
    res_rowadr[rt] = rt*nr;
  }

  // scan original, compute uncompressed
  for (int r=0; r<nr; r++) {
    for (int ci=0; ci<rownnz[r]; ci++) {
      // get rt=c
      int rt = colind[rowadr[r]+ci];

      // record index ct=r, assuming uncompressed res_rowadr[rt]=rt*nr
      res_colind[rt*nr + res_rownnz[rt]] = r;

      // copy data
      res[rt*nr + res_rownnz[rt]] = mat[rowadr[r]+ci];

      // increase counter for rt
      res_rownnz[rt]++;
    }
  }

  // compress
  mju_compressSparse(res, nc, nr, res_rownnz, res_rowadr, res_colind);
}



// construct row supernodes
void mju_superSparse(int nr, int* rowsuper,
                     const int* rownnz, const int* rowadr, const int* colind) {
  // no rows: nothing to do
  if (!nr) {
    return;
  }

  // find match to child
  for (int r=0; r<nr-1; r++) {
    // different number of nonzeros: cannot be a match
    if (rownnz[r]!=rownnz[r+1]) {
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
  for (int r=nr-2; r>=0; r--) {
    if (rowsuper[r]) {
      rowsuper[r] += rowsuper[r+1];
    }
  }
}

// compute sparse M'*diag*M (diag=NULL: compute M'*M), res has uncompressed layout
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
  int* chain = (int*) mj_stackAlloc(d, 2*nc);
  mjtNum* buffer = mj_stackAlloc(d, nc);

  // set uncompressed layout
  for (int r=0; r<nc; r++) {
    res_rowadr[r] = r*nc;
  }

  // compute lower-triangular uncompressed layout (nc per row)
  for (int r=0; r<nc; r++) {
    // copy chain from parent
    if (rowsuperT && r>0 && rowsuperT[r-1]>0) {
      // copy parent chain
      res_rownnz[r] = res_rownnz[r-1];
      memcpy(res_colind+res_rowadr[r], res_colind+res_rowadr[r-1],
             res_rownnz[r]*sizeof(int));

      // add diagonal if rowT is not empty
      if (rownnzT[r]) {
        res_colind[res_rowadr[r]+res_rownnz[r]] = r;
        res_rownnz[r]++;
      }
    }

    // construct chain
    else {
      // clear chain accumulation buffers
      int nchain = 0;
      int inew = 0, iold = nc;
      int lastadded = -1;

      // for each nonzero c in matT_row(r), add nonzeros of mat_row(c) to chain(r)
      for (int i=0; i<rownnzT[r]; i++) {
        // save c
        int c = colindT[rowadrT[r]+i];

        // skip if a chain from same supernode was already added
        if (rowsuper && lastadded>=0 && (c-lastadded)<=rowsuper[lastadded]) {
          continue;
        } else {
          lastadded = c;
        }

        // swap chains
        int adr = inew;
        inew = iold;
        iold = adr;

        // merge chains
        int nnewchain = 0;
        adr = 0;
        int end = rowadr[c]+rownnz[c];
        for (int adr1=rowadr[c]; adr1<end; adr1++) {
          // save column index from mat
          int col_mat = colind[adr1];

          // skip column indices in chain smaller than col_mat
          while (adr<nchain && chain[iold + adr]<col_mat && chain[iold + adr]<=r) {
            chain[inew + nnewchain++] = chain[iold + adr++];
          }

          // only lower-triangular
          if (col_mat>r) {
            break;
          }

          // existing element: advance chain
          if (adr<nchain && chain[iold + adr]==col_mat) {
            adr++;
          }

          // add column index from matT
          chain[inew + nnewchain++] = col_mat;
        }

        // append the rest of the master chain
        while (adr<nchain && chain[iold + adr]<=r) {
          chain[inew + nnewchain++] = chain[iold + adr++];
        }

        // assign newchain
        nchain = nnewchain;
      }

      // copy chain
      res_rownnz[r] = nchain;
      if (nchain) {
        memcpy(res_colind+res_rowadr[r], chain+inew, nchain*sizeof(int));
      }
    }
  }

  // compute matrix data given uncompressed layout
  for (int r=0; r<nc; r++) {
    // clear buffer[colind] for this chain
    int adr = res_rowadr[r];
    for (int i=0; i<res_rownnz[r]; i++) {
      buffer[res_colind[adr+i]] = 0;
    }

    // res_row(r) = sum_c ( matT(r,c) * diag(c) * mat_row(c) )
    for (int i=0; i<rownnzT[r]; i++) {
      // save c and matT(r,c)*diag(c)
      int c = colindT[rowadrT[r]+i];
      mjtNum matTrc = matT[rowadrT[r]+i];
      if (diag) {
        matTrc *= diag[c];
      }

      // process row
      int end = rowadr[c]+rownnz[c];
      for (int adr=rowadr[c]; adr<end; adr++) {
        // get column index from mat, only lower-triangular
        int adr1;
        if ((adr1=colind[adr])>r) {
          break;
        }

        // add to buffer
        buffer[adr1] += matTrc*mat[adr];
      }
    }

    // copy buffer
    adr = res_rowadr[r];
    for (int i=0; i<res_rownnz[r]; i++) {
      res[adr+i] = buffer[res_colind[adr+i]];
    }
  }

  // make symmetric; uncompressed layout
  for (int r=1; r<nc; r++) {
    int end = nc*r+res_rownnz[r]-1;
    for (int adr=nc*r; adr<end; adr++) {
      // add to row given by column index
      int adr1 = nc*res_colind[adr] + res_rownnz[res_colind[adr]]++;
      res[adr1] = res[adr];
      res_colind[adr1] = r;
    }
  }

  mjFREESTACK;
}
