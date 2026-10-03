"""Checks native wheel imports outside the source checkout."""

import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import zipfile

from python.runfiles import runfiles


def main():
  destination = Path(os.environ['TEST_TMPDIR']) / 'installed'
  destination.mkdir()
  resolver = runfiles.Create()
  if os.environ.get('MUJOCO_EXPECT_STUDIO_WEB') == '1':
    prefix = 'mujoco/experimental/studio/web/dist/'
    with zipfile.ZipFile(resolver.Rlocation(sys.argv[1])) as archive:
      for entry in archive.infolist():
        if entry.filename.startswith(prefix) and not entry.is_dir():
          assert not stat.S_ISLNK(entry.external_attr >> 16)
          assert entry.file_size > 0
      for name in ('index.html', 'web_client.js', 'web_client.wasm'):
        assert archive.read(prefix + name)
      assert archive.read(prefix + 'web_client.wasm').startswith(b'\x00asm')
      assert any(name.startswith(prefix + 'assets/') for name in archive.namelist())
  environment = dict(os.environ)
  environment['PYTHONPATH'] = os.pathsep.join(sys.path)
  subprocess.run(
      [sys.executable, '-m', 'pip', 'install', '--no-index', '--no-deps',
       '--no-cache-dir', '--disable-pip-version-check',
       '--no-compile', '--target', str(destination),
       *[resolver.Rlocation(wheel) for wheel in sys.argv[1:]]],
      check=True,
      env=environment,
  )
  if sys.platform == 'darwin':
    for binary in destination.rglob('*'):
      if not binary.is_file():
        continue
      with binary.open('rb') as stream:
        magic = stream.read(4)
      if magic not in (b'\xcf\xfa\xed\xfe', b'\xca\xfe\xba\xbe'):
        continue
      commands = subprocess.check_output(['otool', '-l', str(binary)], text=True)
      minimum_versions = re.findall(
          r'^\s+cmd LC_(?:BUILD_VERSION|VERSION_MIN_MACOSX)\n'
          r'(?:(?!Load command ).*\n)*?\s+(?:minos|version) ([0-9.]+)\n',
          commands,
          re.MULTILINE,
      )
      assert minimum_versions, f'No deployment target in {binary}'
      for version in minimum_versions:
        assert tuple(map(int, version.split('.')[:2])) <= (11, 0), (
            f'{binary} requires macOS {version}, beyond its macosx_11_0 tag'
        )
  dependency_paths = [
      path for path in sys.path if path and not (Path(path) / 'mujoco').exists()
  ]
  environment = dict(os.environ)
  environment['PYTHONPATH'] = os.pathsep.join(
      [str(destination), *dependency_paths]
  )
  program = '''
from pathlib import Path
import mujoco
import mujoco.rollout
import mujoco.viewer
import sys
import subprocess

assert Path(mujoco.__file__).is_relative_to(Path(sys.argv[1]))
assert Path(mujoco.HEADERS_DIR, 'mujoco.h').is_file()
model = mujoco.MjModel.from_xml_string(''' + repr(
          '<mujoco><worldbody><body><freejoint/><geom type="sphere" '
          'size="0.1"/></body></worldbody></mujoco>'
      ) + ''')
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
assert data.time == model.opt.timestep
assert data.qvel[2] < 0
'''
  if os.environ.get('MUJOCO_EXPECT_STUDIO') == '1':
    program += '''
import importlib
modules = [
    'mujoco._render_filament',
    'mujoco.experimental.dear_imgui.dear_imgui',
    'mujoco.experimental.implot.implot',
    *['mujoco.experimental.studio.' + name for name in
      ('native_viewer_cc', 'renderer', 'sim', 'ux', 'window')],
]
if sys.platform != 'win32':
  modules += ['mujoco.experimental.studio.web.' + name for name in
              ('headless_ui', 'state_payload')]
for name in modules:
  module = importlib.import_module(name)
  assert Path(module.__file__).is_relative_to(Path(sys.argv[1]))
'''
  if len(sys.argv) > 2:
    program += '''
from importlib import metadata
import jax
from mujoco import mjx
import numpy as np

assert Path(mjx.__file__).is_relative_to(Path(sys.argv[1]))
entries = metadata.distribution('mujoco-mjx').entry_points
assert {entry.name for entry in entries} == {'mjx-testspeed', 'mjx-viewer'}
for entry in entries:
  subprocess.run([sys.executable, '-c',
                  'from importlib import metadata; '
                  'entries = metadata.distribution(\"mujoco-mjx\").entry_points; '
                  'entry = next(e for e in entries '
                  f'if e.name == {entry.name!r}); assert callable(entry.load())'],
                 check=True)
mjx_model = mjx.put_model(model, impl='jax')
mjx_data = mjx.make_data(model, impl='jax')
mjx_data = jax.jit(mjx.step)(mjx_model, mjx_data)
np.testing.assert_allclose(mjx_data.qpos, data.qpos, atol=1e-6)
np.testing.assert_allclose(mjx_data.qvel, data.qvel, atol=1e-6)
'''
  subprocess.run(
      [sys.executable, '-c', program, str(destination)],
      check=True,
      cwd=destination,
      env=environment,
  )


if __name__ == '__main__':
  main()
