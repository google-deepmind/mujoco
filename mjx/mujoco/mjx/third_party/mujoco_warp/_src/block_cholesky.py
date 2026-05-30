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
def create_blocked_cholesky_factorize_solve_func(block_size: int, matrix_size_static: int):
  @wp.func
  def blocked_cholesky_factorize_solve_func(
    # In:
    A: wp.array2d[float],
    b: wp.array2d[float],
    matrix_size: int,
    # Out:
    U: wp.array2d[float],
    x: wp.array2d[float],
  ):
    """Block Cholesky factorization and solve while keeping the forward RHS live."""
    rhs_tile = wp.tile_load(b, shape=(matrix_size_static, 1), offset=(0, 0), storage="shared", bounds_check=False)

    for k in range(0, matrix_size, block_size):
      end = k + block_size
      rhs_view = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(k, 0))

      A_kk_tile = wp.tile_load(
        A, shape=(block_size, block_size), offset=(k, k), storage="shared", bounds_check=False, aligned=True
      )

      for j in range(0, k, block_size):
        U_block = wp.tile_load(
          U, shape=(block_size, block_size), offset=(j, k), storage="shared", bounds_check=False, aligned=True
        )
        wp.tile_matmul(wp.tile_transpose(U_block), U_block, A_kk_tile, alpha=-1.0)

        y_block = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(j, 0))
        wp.tile_matmul(wp.tile_transpose(U_block), y_block, rhs_view, alpha=-1.0)

      wp.tile_cholesky_inplace(A_kk_tile, fill_mode="upper")
      wp.tile_store(U, A_kk_tile, offset=(k, k), bounds_check=False, aligned=True)

      wp.tile_lower_solve_inplace(wp.tile_transpose(A_kk_tile), rhs_view)

      for i in range(end, matrix_size, block_size):
        A_ki_tile = wp.tile_load(
          A, shape=(block_size, block_size), offset=(k, i), storage="shared", bounds_check=False, aligned=True
        )

        for j in range(0, k, block_size):
          U_jk_tile = wp.tile_load(
            U, shape=(block_size, block_size), offset=(j, k), storage="shared", bounds_check=False, aligned=True
          )
          U_ji_tile = wp.tile_load(
            U, shape=(block_size, block_size), offset=(j, i), storage="shared", bounds_check=False, aligned=True
          )
          wp.tile_matmul(wp.tile_transpose(U_jk_tile), U_ji_tile, A_ki_tile, alpha=-1.0)

        wp.tile_lower_solve_inplace(wp.tile_transpose(A_kk_tile), A_ki_tile)
        wp.tile_store(U, A_ki_tile, offset=(k, i), bounds_check=False, aligned=True)

    for i in range(matrix_size - block_size, -1, -block_size):
      i_end = i + block_size
      tmp_tile = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(i, 0))
      for j in range(i_end, matrix_size, block_size):
        U_tile = wp.tile_load(
          U, shape=(block_size, block_size), offset=(i, j), storage="shared", bounds_check=False, aligned=True
        )
        x_tile = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(j, 0))
        wp.tile_matmul(U_tile, x_tile, tmp_tile, alpha=-1.0)

      U_tile = wp.tile_load(
        U, shape=(block_size, block_size), offset=(i, i), storage="shared", bounds_check=False, aligned=True
      )
      wp.tile_upper_solve_inplace(U_tile, tmp_tile)

    wp.tile_store(x, rhs_tile, offset=(0, 0), bounds_check=False)

  return blocked_cholesky_factorize_solve_func


@lru_cache(maxsize=None)
def create_blocked_cholesky_solve_func(block_size: int, matrix_size_static: int):
  @wp.func
  def blocked_cholesky_solve_func(
    # In:
    U: wp.array2d[float],
    b: wp.array2d[float],
    matrix_size: int,
    # Out:
    x: wp.array2d[float],
  ):
    """Block Cholesky solve.

    Solves A x = b given the Cholesky factor U (A = U^T U) using blocked forward and backward
    substitution.
    """
    rhs_tile = wp.tile_load(b, shape=(matrix_size_static, 1), offset=(0, 0), storage="shared", bounds_check=False)

    # Forward substitution: solve U^T y = b
    for i in range(0, matrix_size, block_size):
      rhs_view = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(i, 0))
      for j in range(0, i, block_size):
        U_block = wp.tile_load(
          U, shape=(block_size, block_size), offset=(j, i), storage="shared", bounds_check=False, aligned=True
        )
        y_block = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(j, 0))
        wp.tile_matmul(wp.tile_transpose(U_block), y_block, rhs_view, alpha=-1.0)

      U_tile = wp.tile_load(
        U, shape=(block_size, block_size), offset=(i, i), storage="shared", bounds_check=False, aligned=True
      )
      wp.tile_lower_solve_inplace(wp.tile_transpose(U_tile), rhs_view)

    # Backward substitution: solve U x = y
    for i in range(matrix_size - block_size, -1, -block_size):
      i_end = i + block_size
      tmp_tile = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(i, 0))
      for j in range(i_end, matrix_size, block_size):
        U_tile = wp.tile_load(
          U, shape=(block_size, block_size), offset=(i, j), storage="shared", bounds_check=False, aligned=True
        )
        x_tile = wp.tile_view(rhs_tile, shape=(block_size, 1), offset=(j, 0))
        wp.tile_matmul(U_tile, x_tile, tmp_tile, alpha=-1.0)
      U_tile = wp.tile_load(
        U, shape=(block_size, block_size), offset=(i, i), storage="shared", bounds_check=False, aligned=True
      )

      wp.tile_upper_solve_inplace(U_tile, tmp_tile)

    wp.tile_store(x, rhs_tile, offset=(0, 0), bounds_check=False)

  return blocked_cholesky_solve_func
