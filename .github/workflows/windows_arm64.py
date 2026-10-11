"""Build and test CPU-only Windows ARM64 wheels using the configured MSVC SDK.

Studio/Filament, USD, rendering, and GPU backends are not covered by this lane.
"""

import hashlib
import os
import pathlib
import shutil
import subprocess
import sys
import tarfile
import urllib.request

import zstandard


SOURCE = pathlib.Path(__file__).resolve().parents[2]
WORK = pathlib.Path(os.environ['MUJOCO_ARM64_WORK']).resolve()
PYTHON = sys.executable


def run(*args, cwd=WORK, env=None):
  subprocess.run([str(arg) for arg in args], cwd=cwd, env=env, check=True)


def main():
  WORK.mkdir(parents=True, exist_ok=True)
  archive = WORK / 'llvm.tar.zst'
  url = (
      'https://github.com/llvm/llvm-project/releases/download/llvmorg-23.1.3/'
      'clang%2Bllvm-23.1.3-aarch64-pc-windows-msvc.tar.zst'
  )
  if not archive.exists():
    urllib.request.urlretrieve(url, archive)
  assert hashlib.sha256(archive.read_bytes()).hexdigest() == (
      '667b881302a4e041d3034601a9055202edd8a866afaaaac6017682fba587f5f4'
  )
  llvm = WORK / 'llvm' / 'clang+llvm-23.1.3-aarch64-pc-windows-msvc'
  if not (llvm / 'bin' / 'clang-cl.exe').exists():
    with archive.open('rb') as compressed:
      with zstandard.ZstdDecompressor(max_window_size=2147483648).stream_reader(
          compressed
      ) as stream:
        with tarfile.open(fileobj=stream, mode='r|') as files:
          for member in files:
            parts = pathlib.PurePosixPath(member.name).parts
            if len(parts) > 1 and (
                parts[1] == 'bin' or parts[1:3] == ('lib', 'clang')
            ):
              files.extract(member, WORK / 'llvm', filter='data')

  # Match the existing Windows core toolchain; use Clang for Python bindings.
  # MSVC gives unnamed nested-type wrapper specializations internal linkage.
  install = WORK / 'install'
  run(
      'cmake',
      '-S',
      SOURCE,
      '-B',
      WORK / 'core',
      '-G',
      'Ninja',
      '-DCMAKE_C_COMPILER=cl',
      '-DCMAKE_CXX_COMPILER=cl',
      '-DCMAKE_SYSTEM_PROCESSOR=ARM64',
      '-DCMAKE_BUILD_TYPE=Release',
      '-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON',
      '-DMUJOCO_BUILD_TESTS=OFF',
      '-DMUJOCO_BUILD_EXAMPLES=OFF',
      '-DMUJOCO_BUILD_SIMULATE=OFF',
      '-DMUJOCO_BUILD_STUDIO=OFF',
      f'-DCMAKE_INSTALL_PREFIX={install.as_posix()}',
  )
  run('cmake', '--build', WORK / 'core', '--parallel', '3')
  run('cmake', '--install', WORK / 'core')
  plugins = WORK / 'plugins'
  plugins.mkdir(exist_ok=True)
  for dll in (WORK / 'core' / 'bin').glob('*.dll'):
    if dll.name != 'mujoco.dll':
      shutil.copy2(dll, plugins)

  package = WORK / 'package'
  shutil.copytree(SOURCE / 'python', package, dirs_exist_ok=True)
  shutil.copy2(SOURCE / 'LICENSE', package / 'LICENSE')
  shutil.copytree(
      SOURCE / 'cmake', package / 'mujoco' / 'cmake', dirs_exist_ok=True
  )
  shutil.copytree(
      SOURCE / 'simulate', package / 'mujoco' / 'simulate', dirs_exist_ok=True
  )
  env = dict(os.environ, PYTHONPATH=str(SOURCE / 'python' / 'mujoco'))
  for generator, header in (
      ('generate_enum_traits.py', 'enum_traits.h'),
      ('generate_function_traits.py', 'function_traits.h'),
      ('generate_spec_bindings.py', 'specs.cc.inc'),
  ):
    with (package / 'mujoco' / header).open('wb') as output:
      subprocess.run(
          [PYTHON, str(package / 'mujoco' / 'codegen' / generator)],
          env=env,
          stdout=output,
          check=True,
      )
  env = dict(os.environ)
  env.update(
      PATH=str(llvm / 'bin') + os.pathsep + env['PATH'],
      CC='clang-cl',
      CXX='clang-cl',
      MUJOCO_PATH=str(install),
      MUJOCO_PLUGIN_PATH=str(plugins),
      MUJOCO_CMAKE_ARGS=(
          '-G Ninja -DCMAKE_SYSTEM_PROCESSOR=ARM64 '
          f'-DCMAKE_LINKER="{(llvm / "bin" / "lld-link.exe").as_posix()}" '
          '-DCMAKE_JOB_POOLS=native=3 -DCMAKE_JOB_POOL_COMPILE=native '
          '-DCMAKE_JOB_POOL_LINK=native'
      ),
  )
  run(PYTHON, 'setup_test.py', cwd=package)
  run(
      PYTHON,
      'setup.py',
      'build_ext',
      '--build-temp',
      WORK / 'bindings',
      cwd=package,
      env=env,
  )
  run(
      PYTHON, 'setup.py', 'build_py', 'bdist_wheel', '--skip-build', cwd=package
  )
  wheels = list((package / 'dist').glob('*-win_arm64.whl'))
  assert len(wheels) == 1, wheels
  run(
      PYTHON,
      SOURCE / '.github/workflows/audit_windows_arm64_wheel.py',
      wheels[0],
  )
  run(PYTHON, '-m', 'venv', WORK / 'test-venv')
  test_python = WORK / 'test-venv/Scripts/python.exe'
  run(
      test_python,
      '-m',
      'pip',
      'install',
      wheels[0],
      'pytest',
      'pytest-subtests',
      'typing_extensions',
  )
  env = dict(os.environ, MUJOCO_GL='disable')
  run(
      test_python,
      '-m',
      'pytest',
      '--pyargs',
      'mujoco.bindings_test',
      'mujoco.rollout_test',
      'mujoco.specs_test',
      'mujoco.vfs_test',
      '-q',
      '-rs',
      f'--junitxml={WORK / "tests.xml"}',
      env=env,
  )


if __name__ == '__main__':
  main()
