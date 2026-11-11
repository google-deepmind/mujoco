// Copyright 2025 DeepMind Technologies Limited
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

import 'jasmine';

import loadMujoco, {MainModule} from '../dist/mujoco.js';

describe('MuJoCo WASM logging', () => {
  let mujoco: MainModule;
  let NumBuffer: MainModule['DoubleBuffer']|MainModule['FloatBuffer'];
  const stdout: string[] = [];
  const stderr: string[] = [];

  beforeAll(async () => {
    mujoco = await loadMujoco({
      print: (message: string) => stdout.push(message),
      printErr: (message: string) => stderr.push(message),
    });
    const opt = new mujoco.MjOption();
    try {
      NumBuffer = opt.gravity.BYTES_PER_ELEMENT === 4 ?
          mujoco.FloatBuffer : mujoco.DoubleBuffer;
    } finally {
      opt.delete();
    }
  }, 60_000);

  beforeEach(() => {
    stdout.length = 0;
    stderr.length = 0;
    const config = mujoco.mju_getLogConfig();
    try {
      config.logto_console = true;
      config.logto_file = false;
      config.topics = 0;
      mujoco.mju_setLogConfig(config);
    } finally {
      config.delete();
    }
  });

  it('reports simulation warnings on stderr', () => {
    const model = mujoco.MjModel.from_xml_string(`
      <mujoco><worldbody><body>
        <joint/><geom size=".1"/>
      </body></worldbody></mujoco>`);
    const data = new mujoco.MjData(model);
    try {
      data.qpos[0] = NaN;
      mujoco.mj_checkPos(model, data);
      expect(data.qpos[0]).toBe(0);
      expect(stderr.join('\n')).toContain('QPOS');
      expect(stderr.join('\n')).toContain('unstable');
      expect(stdout).toEqual([]);
    } finally {
      data.delete();
      model.delete();
    }
  });

  it('reports informational messages and respects topic filtering', () => {
    mujoco.mju_info(0, 'unfiltered message');
    expect(stdout.join('\n')).toContain('unfiltered message');
    const topic = mujoco.mjtLogTopic.mjTOPIC_TIME_STP.value;
    stdout.length = 0;
    mujoco.mju_info(topic, 'disabled topic');
    expect(stdout).toEqual([]);

    const config = mujoco.mju_getLogConfig();
    try {
      config.topics = 1 << (topic - 1);
      mujoco.mju_setLogConfig(config);
    } finally {
      config.delete();
    }
    mujoco.mju_info(topic, 'enabled topic');
    expect(stdout.join('\n')).toContain('enabled topic');
    expect(stderr).toEqual([]);
  });

  it('respects console and file output settings', () => {
    const config = mujoco.mju_getLogConfig();
    try {
      config.logto_console = false;
      config.logto_file = true;
      mujoco.mju_setLogConfig(config);
      mujoco.mju_info(0, 'file only message');
      const fs = (mujoco as any).FS;
      const contents = fs.readFile('MUJOCO_LOG.TXT', {encoding: 'utf8'});
      expect(contents).toContain('file only message');
      expect(stdout).toEqual([]);
      expect(stderr).toEqual([]);

      config.logto_file = false;
      mujoco.mju_setLogConfig(config);
      mujoco.mju_info(0, 'disabled output');
      expect(fs.readFile('MUJOCO_LOG.TXT', {encoding: 'utf8'})).toBe(contents);
      expect(stdout).toEqual([]);
      fs.unlink('MUJOCO_LOG.TXT');
    } finally {
      config.delete();
    }
  });

  it('still throws JavaScript errors and logs after an error is caught', () => {
    const quat = new NumBuffer(4);
    try {
      expect(() => mujoco.mju_euler2Quat(quat, [0, 0, 0], 'xy'))
          .toThrowError(
              'MuJoCo Error: mju_euler2Quat: seq must contain exactly 3 characters');
      expect(stderr).toEqual([]);
      mujoco.mju_info(0, 'after error');
      expect(stdout.join('\n')).toContain('after error');
    } finally {
      quat.delete();
    }
  });
});
