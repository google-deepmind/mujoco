// Copyright 2019 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Runtime.InteropServices;
using NUnit.Framework;

namespace Mujoco {

  using static MujocoLib;

  [TestFixture]
  public class MjVfsTests {
    private MjVfs _vfs;

    [SetUp]
    public void SetUp() {
      _vfs = new MjVfs();
    }

    [TearDown]
    public void TearDown() {
      _vfs.Dispose();
      _vfs = null;
    }

    [Test]
    public unsafe void AddingNewFile() {
      var filename = "filename";
      var contents = "contents";
      _vfs.AddFile(filename, contents);
      Assert.That(() => { _vfs.AddFile(filename, contents); }, Throws.Exception);  // duplicate file
    }
  }
}
