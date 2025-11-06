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
    tid_block: int,
    A: wp.array(dtype=float, ndim=2),
    active_matrix_size: int,
    # Out:
    L: wp.array(dtype=float, ndim=2),
  ):
    """Computes the Cholesky factorization of a symmetric positive definite matrix A in blocks.

    It returns a lower-triangular matrix L such that A = L L^T.
    """
    num_threads_per_block = wp.block_dim()

    # Round up active_matrix_size to next multiple of block_size
    n = ((active_matrix_size + block_size - 1) // block_size) * block_size

    # Process the matrix in blocks along its leading dimension.
    for k in range(0, n, block_size):
      end = k + block_size

      # Load current diagonal block A[k:end, k:end]
      # and update with contributions from previously computed blocks.
      A_kk_tile = wp.tile_load(A, shape=(block_size, block_size), offset=(k, k), storage="shared")
      # The following if pads the matrix if it is not divisible by block_size
      if k + block_size > active_matrix_size or k + block_size > active_matrix_size:
        num_tile_elements = block_size * block_size
        num_iterations = (num_tile_elements + num_threads_per_block - 1) // num_threads_per_block

        for i in range(num_iterations):
          linear_index = tid_block + i * num_threads_per_block
          linear_index = linear_index % num_tile_elements
          row = linear_index // block_size
          col = linear_index % block_size
          value = A_kk_tile[row, col]
          if k + row >= active_matrix_size or k + col >= active_matrix_size:
            value = wp.where(row == col, float(1), float(0))
          A_kk_tile[row, col] = value

      if k > 0:
        for j in range(0, k, block_size):
          L_block = wp.tile_load(L, shape=(block_size, block_size), offset=(k, j))
          L_block_T = wp.tile_transpose(L_block)
          L_L_T_block = wp.tile_matmul(L_block, L_block_T)
          A_kk_tile -= L_L_T_block

      # Compute the Cholesky factorization for the block
      L_kk_tile = wp.tile_cholesky(A_kk_tile)
      wp.tile_store(L, L_kk_tile, offset=(k, k))

      # Process the blocks below the current block
      for i in range(end, n, block_size):
        A_ik_tile = wp.tile_load(A, shape=(block_size, block_size), offset=(i, k), storage="shared")
        # The following if pads the matrix if it is not divisible by block_size
        if i + block_size > active_matrix_size or k + block_size > active_matrix_size:
          num_tile_elements = block_size * block_size
          num_iterations = (num_tile_elements + num_threads_per_block - 1) // num_threads_per_block

          for ii in range(num_iterations):
            linear_index = tid_block + ii * num_threads_per_block
            linear_index = linear_index % num_tile_elements
            row = linear_index // block_size
            col = linear_index % block_size
            value = A_ik_tile[row, col]
            if i + row >= active_matrix_size or k + col >= active_matrix_size:
              value = wp.where(i + row == k + col, float(1), float(0))
            A_ik_tile[row, col] = value

        if k > 0:
          for j in range(0, k, block_size):
            L_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(i, j))
            L_2_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(k, j))
            L_T_tile = wp.tile_transpose(L_2_tile)
            L_L_T_tile = wp.tile_matmul(L_tile, L_T_tile)
            A_ik_tile -= L_L_T_tile

        t = wp.tile_transpose(A_ik_tile)
        tmp = wp.tile_lower_solve(L_kk_tile, t)
        sol_tile = wp.tile_transpose(tmp)

        wp.tile_store(L, sol_tile, offset=(i, k))

  return blocked_cholesky_func


@lru_cache(maxsize=None)
def create_blocked_cholesky_solve_func(block_size: int):
  @wp.func
  def blocked_cholesky_solve_func(
    # In:
    tid_block: int,
    L: wp.array(dtype=float, ndim=2),
    b: wp.array(dtype=float, ndim=2),
    tmp: wp.array(dtype=float, ndim=2),
    active_matrix_size: int,
    # Out:
    x: wp.array(dtype=float, ndim=2),
  ):
    """Block Cholesky factorization and solve.

    Solves A x = b given the Cholesky factor L (A = L L^T) using blocked forward and backward
    substitution.
    """
    num_threads_per_block = wp.block_dim()

    # Round up active_matrix_size to next multiple of block_size
    n = ((active_matrix_size + block_size - 1) // block_size) * block_size

    # Forward substitution: solve L y = b
    for i in range(0, n, block_size):
      i_end = i + block_size
      rhs_tile = wp.tile_load(b, shape=(block_size, 1), offset=(i, 0))
      if i > 0:
        for j in range(0, i, block_size):
          L_block = wp.tile_load(L, shape=(block_size, block_size), offset=(i, j))
          y_block = wp.tile_load(tmp, shape=(block_size, 1), offset=(j, 0))
          Ly_block = wp.tile_matmul(L_block, y_block)
          rhs_tile -= Ly_block
      L_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(i, i))

      # The following if pads the matrix if it is not divisible by block_size
      if i + block_size > active_matrix_size:
        num_tile_elements = block_size * block_size
        num_iterations = (num_tile_elements + num_threads_per_block - 1) // num_threads_per_block

        for ii in range(num_iterations):
          linear_index = tid_block + ii * num_threads_per_block
          linear_index = linear_index % num_tile_elements
          row = linear_index // block_size
          col = linear_index % block_size
          value = L_tile[row, col]
          if i + row >= active_matrix_size or i + col >= active_matrix_size:
            value = wp.where(row == col, float(1), float(0))
          L_tile[row, col] = value

        # Handle rhs
        num_tile_elements = block_size
        num_iterations = (num_tile_elements + num_threads_per_block - 1) // num_threads_per_block

        for ii in range(num_iterations):
          linear_index = tid_block + ii * num_threads_per_block
          linear_index = linear_index % num_tile_elements
          value = rhs_tile[linear_index, 0]
          if i + linear_index >= active_matrix_size:
            value = float(0)
          rhs_tile[linear_index, 0] = value

      y_tile = wp.tile_lower_solve(L_tile, rhs_tile)
      wp.tile_store(tmp, y_tile, offset=(i, 0))

    # Backward substitution: solve L^T x = y
    for i in range(n - block_size, -1, -block_size):
      i_end = i + block_size
      rhs_tile = wp.tile_load(tmp, shape=(block_size, 1), offset=(i, 0))
      if i_end < n:
        for j in range(i_end, n, block_size):
          L_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(j, i))
          L_T_tile = wp.tile_transpose(L_tile)
          x_tile = wp.tile_load(x, shape=(block_size, 1), offset=(j, 0))
          L_T_x_tile = wp.tile_matmul(L_T_tile, x_tile)
          rhs_tile -= L_T_x_tile
      L_tile = wp.tile_load(L, shape=(block_size, block_size), offset=(i, i))

      # The following if pads the matrix if it is not divisible by block_size
      if i + block_size > active_matrix_size:
        num_tile_elements = block_size * block_size
        num_iterations = (num_tile_elements + num_threads_per_block - 1) // num_threads_per_block

        for ii in range(num_iterations):
          linear_index = tid_block + ii * num_threads_per_block
          linear_index = linear_index % num_tile_elements
          row = linear_index // block_size
          col = linear_index % block_size
          value = L_tile[row, col]
          if i + row >= active_matrix_size or i + col >= active_matrix_size:
            value = wp.where(row == col, float(1), float(0))
          L_tile[row, col] = value

        # Handle rhs
        num_tile_elements = block_size
        num_iterations = (num_tile_elements + num_threads_per_block - 1) // num_threads_per_block

        for ii in range(num_iterations):
          linear_index = tid_block + ii * num_threads_per_block
          linear_index = linear_index % num_tile_elements
          value = rhs_tile[linear_index, 0]
          if i + linear_index >= active_matrix_size:
            value = float(0)
          rhs_tile[linear_index, 0] = value

      x_tile = wp.tile_upper_solve(wp.tile_transpose(L_tile), rhs_tile)
      wp.tile_store(x, x_tile, offset=(i, 0))

  return blocked_cholesky_solve_func
