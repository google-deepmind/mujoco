"""Audit the native Windows ARM64 wheel before running its installed tests."""

import base64
import csv
import hashlib
import io
import pathlib
import struct
import sys
import sysconfig
import zipfile


def assert_arm64(data, name):
  assert data[:2] == b'MZ', name
  pe = struct.unpack_from('<I', data, 0x3C)[0]
  assert data[pe : pe + 4] == b'PE\0\0', name
  machine = struct.unpack_from('<H', data, pe + 4)[0]
  assert machine == 0xAA64, f'{name}: expected ARM64, got {machine:#x}'


def main():
  assert sysconfig.get_platform() == 'win-arm64', sysconfig.get_platform()
  assert_arm64(pathlib.Path(sys._base_executable).read_bytes(), 'Python')
  wheel = pathlib.Path(sys.argv[1])
  assert wheel.name.endswith('-win_arm64.whl'), wheel.name
  with zipfile.ZipFile(wheel) as archive:
    names = archive.namelist()
    assert len(names) == len(set(names)), 'Duplicate wheel entries'
    assert 'mujoco/include/mujoco/mujoco.h' in names
    assert not any('/mujoco/mujoco/' in name for name in names)
    extensions = {
        pathlib.PurePosixPath(name).name.split('.')[0]
        for name in names
        if name.endswith('.pyd')
    }
    required = {
        '_callbacks',
        '_constants',
        '_enums',
        '_errors',
        '_functions',
        '_render',
        '_rollout',
        '_simulate',
        '_specs',
        '_structs',
    }
    assert required <= extensions, required - extensions
    assert 'mujoco/mujoco.dll' in names
    native = [name for name in names if name.lower().endswith(('.dll', '.pyd'))]
    assert any(name.startswith('mujoco/plugin/') for name in native)
    for name in native:
      assert_arm64(archive.read(name), name)
    record = next(name for name in names if name.endswith('.dist-info/RECORD'))
    for name, digest, size in csv.reader(
        io.StringIO(archive.read(record).decode())
    ):
      if digest:
        algorithm, expected = digest.split('=', 1)
        data = archive.read(name)
        actual = base64.urlsafe_b64encode(hashlib.new(algorithm, data).digest())
        assert actual.rstrip(b'=').decode() == expected, name
        assert len(data) == int(size), name
  print(
      f'ARM64 wheel audit PASS: {len(native)} native binaries; '
      f'SHA256={hashlib.sha256(wheel.read_bytes()).hexdigest()}'
  )


if __name__ == '__main__':
  main()
