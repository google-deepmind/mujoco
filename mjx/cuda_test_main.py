"""Requires CUDA before running an accelerator test suite."""

import os
from pathlib import Path
import runpy
import sys
import tempfile

# Bazel keeps CUDA wheels in separate repositories, outside JAX's ELF rpaths.
if not os.environ.get('MUJOCO_BAZEL_CUDA_INITIALIZED'):
  libraries = []
  binaries = []
  for entry in sys.path:
    root = Path(entry) / 'nvidia'
    libraries.extend(str(path) for path in root.glob('*/lib'))
    binaries.extend(str(path) for path in root.glob('*/bin'))
    nvcc = root / 'cuda_nvcc'
    if nvcc.is_dir():
      os.environ['XLA_FLAGS'] = (
          os.environ.get('XLA_FLAGS', '') + f' --xla_gpu_cuda_data_dir={nvcc}'
      )
  os.environ['LD_LIBRARY_PATH'] = os.pathsep.join(
      [*libraries, os.environ.get('LD_LIBRARY_PATH', '')]
  )
  os.environ['PATH'] = os.pathsep.join([*binaries, os.environ['PATH']])
  os.environ['PYTHONPATH'] = os.pathsep.join(sys.path)
  os.environ['MUJOCO_BAZEL_CUDA_INITIALIZED'] = '1'
  os.execve(sys.executable, [sys.executable, *sys.argv], os.environ)

import jax
import jax.numpy as jp
from python.runfiles import runfiles
import warp


def main():
  tempfile.tempdir = os.environ['TEST_TMPDIR']
  warp.config.kernel_cache_dir = os.path.join(tempfile.tempdir, 'warp')
  if not jax.devices('gpu'):
    raise RuntimeError('JAX did not find a CUDA device.')
  if not warp.get_cuda_devices():
    raise RuntimeError('Warp did not find a CUDA device.')
  if len(sys.argv) == 1:
    result = jax.jit(lambda x: x + 1)(jp.array([1]))
    if int(result.block_until_ready()[0]) != 2:
      raise RuntimeError('The CUDA kernel returned an unexpected result.')
    return
  test = runfiles.Create().Rlocation(sys.argv[1])
  sys.argv = [test, *sys.argv[2:]]
  runpy.run_path(test, run_name='__main__')


if __name__ == '__main__':
  main()
