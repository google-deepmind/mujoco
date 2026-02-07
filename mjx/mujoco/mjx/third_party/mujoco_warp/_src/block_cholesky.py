# Copyright 2025 The Newton Developers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

from functools import lru_cache

import warp as wp


@lru_cache(maxsize=None)
def create_blocked_cholesky_func(block_size: int):
  @wp.func
  def blocked_cholesky_func(
    # In:
    A: wp.array(dtype=float, ndim=2),
    matrix_size: int,
    # Out:
    L: wp.array(dtype=float, ndim=2),
  ):
    """Computes the Cholesky factorization of a symmetric positive definite matrix A in blocks.

    It returns a lower-triangular matrix L such that A = L L^T.
    """
    # TODO(team): remove conditional after mjwarp relies on >= 1.11
    bleeding_edge_warp = wp.static(wp.__version__ >= "1.11")

    # Process the matrix in blocks along its leading dimension.
    for k in range(0, matrix_size, block_size):
      end = k + block_size

      # Load current diagonal block A[k:end, k:end]
      # and update with contributions from previously computed blocks.
      A_kk_tile = wp.tile_load(A, shape=(block_size, block_size), offset=(k, k), storage="shared")

      for j in range(0, k, block_size):
        L_block = wp.tile_load(L, shape=(block_size, block_size), offset=(k, j), storage="shared")
        if bleeding_edge_warp:
          wp.tile_matmul(L_block, wp.tile_transpose(L_block), A_kk_tile, alpha=-1.0)
        else:
          A_kk_tile -= wp.tile_matmul(L_block, wp.tile_transpose(L_block))

      # Compute the Cholesky factorization for the block
      L_kk_tile = wp.tile_cholesky(A_kk_tile)
      wp.tile_store(L, L_kk_tile, offset=(k, k))

      # Process the blocks below the current block
      for i in range(end, matrix_size, block_size):
        A_ik_tile = wp.tile_load(A, shape=(block_size, block_size), offset=(i, k), storage="shared")

        for j in range(0, k, block_size):
          L_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(i, j), storage="shared")
          L_2_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(k, j), storage="shared")
          if bleeding_edge_warp:
            wp.tile_matmul(L_tile, wp.tile_transpose(L_2_tile), A_ik_tile, alpha=-1.0)
          else:
            A_ik_tile -= wp.tile_matmul(L_tile, wp.tile_transpose(L_2_tile))

        if bleeding_edge_warp:
          wp.tile_lower_solve_inplace(L_kk_tile, wp.tile_transpose(A_ik_tile))
        else:
          A_ik_tile = wp.tile_transpose(wp.tile_lower_solve(L_kk_tile, wp.tile_transpose(A_ik_tile)))
        wp.tile_store(L, A_ik_tile, offset=(i, k))

  return blocked_cholesky_func


@lru_cache(maxsize=None)
def create_blocked_cholesky_solve_func(block_size: int, matrix_size_static: int):
  @wp.func
  def blocked_cholesky_solve_func(
    # In:
    L: wp.array(dtype=float, ndim=2),
    b: wp.array(dtype=float, ndim=2),
    matrix_size: int,
    # Out:
    x: wp.array(dtype=float, ndim=2),
  ):
    """Block Cholesky factorization and solve.

    Solves A x = b given the Cholesky factor L (A = L L^T) using blocked forward and backward
    substitution.
    """
    # TODO(team): remove conditional after mjwarp relies on >= 1.11
    bleeding_edge_warp = wp.static(wp.__version__ >= "1.11")
    rhs_tile = wp.tile_load(b, shape=(matrix_size_static, 1), offset=(0, 0), storage="shared", bounds_check=False)

    # Forward substitution: solve L y = b
    for i in range(0, matrix_size, block_size):
      rhs_view = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(i, 0))
      for j in range(0, i, block_size):
        L_block = wp.tile_load(L, shape=(block_size, block_size), offset=(i, j), storage="shared")
        y_block = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(j, 0))
        if bleeding_edge_warp:
          wp.tile_matmul(L_block, y_block, rhs_view, alpha=-1.0)
        else:
          rhs_view -= wp.tile_matmul(L_block, y_block)

      L_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(i, i), storage="shared")
      if bleeding_edge_warp:
        wp.tile_lower_solve_inplace(L_tile, rhs_view)
      else:
        rhs_tmp = wp.tile_lower_solve(L_tile, rhs_view)
        wp.tile_assign(rhs_tile, rhs_tmp, offset=(i, 0))

    # Backward substitution: solve L^T x = y
    for i in range(matrix_size - block_size, -1, -block_size):
      i_end = i + block_size
      tmp_tile = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(i, 0))
      for j in range(i_end, matrix_size, block_size):
        L_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(j, i), storage="shared")
        x_tile = wp.tile_load(x, shape=(block_size, 1), offset=(j, 0), storage="shared", bounds_check=False)
        if bleeding_edge_warp:
          wp.tile_matmul(wp.tile_transpose(L_tile), x_tile, tmp_tile, alpha=-1.0)
        else:
          tmp_tile -= wp.tile_matmul(wp.tile_transpose(L_tile), x_tile)
      L_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(i, i), storage="shared")

      if bleeding_edge_warp:
        wp.tile_upper_solve_inplace(wp.tile_transpose(L_tile), tmp_tile)
      else:
        tmp_tile = wp.tile_upper_solve(wp.tile_transpose(L_tile), tmp_tile)
      wp.tile_store(x, tmp_tile, offset=(i, 0), bounds_check=False)

  return blocked_cholesky_solve_func
