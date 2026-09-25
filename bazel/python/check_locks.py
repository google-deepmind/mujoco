"""Checks Bazel dependency locks against the package build locks."""

from pathlib import Path
import re
import sys

from python.runfiles import runfiles


_BASE = {
    'absl-py', 'etils', 'glfw', 'numpy', 'pyopengl', 'websockets', 'fsspec',
    'importlib-resources', 'typing-extensions', 'zipp',
}
_MJX = _BASE | {'jax', 'jaxlib', 'scipy', 'trimesh', 'ml-dtypes', 'opt-einsum'}
_CUDA = {'jax-cuda12-plugin', 'jax-cuda12-pjrt', 'warp-lang'}
_SYSID = _BASE | {
    'pytest', 'pygments', 'packaging', 'iniconfig', 'pluggy', 'colorama',
    'exceptiongroup', 'tomli', 'imageio', 'imageio-ffmpeg', 'jinja2', 'matplotlib',
    'plotly', 'pyyaml', 'scipy', 'tabulate', 'contourpy', 'cycler', 'fonttools',
    'kiwisolver', 'markupsafe', 'narwhals', 'pillow', 'psutil', 'pyparsing',
    'python-dateutil', 'six',
}


def requirements(path, names=None):
  result = set()
  for line in path.read_text().replace('\\\n', '').splitlines():
    line = ' '.join(line.split())
    if not line or line.startswith('#'):
      continue
    requirement, *hashes = line.split(' --hash=')
    name = re.match(r'[A-Za-z0-9_-]+', requirement)[0].lower().replace('_', '-')
    if names is not None and name not in names:
      continue
    if names is not None and name == 'colorama' and ';' in requirement:
      continue
    result.add((requirement, tuple(sorted(hashes))))
  return result


def main():
  resolver = runfiles.Create()
  paths = [Path(resolver.Rlocation(argument)) for argument in sys.argv[1:]]
  python_source, mjx_source, cuda_source, cuda_runtime, *locks = paths
  expected = [
      requirements(python_source, _BASE | {'pip'}),
      requirements(mjx_source, _MJX),
      requirements(mjx_source, _MJX) | requirements(cuda_source, _CUDA)
      | requirements(cuda_runtime),
      requirements(python_source, _SYSID),
  ]
  differences = []
  for lock, source_requirements in zip(locks, expected, strict=True):
    locked = requirements(lock)
    for requirement, _ in sorted(source_requirements - locked):
      differences.append(f'{lock.name}: missing or stale {requirement}')
    for requirement, _ in sorted(locked - source_requirements):
      differences.append(f'{lock.name}: unexpected {requirement}')
  if differences:
    raise SystemExit('\n'.join(differences))


if __name__ == '__main__':
  main()
