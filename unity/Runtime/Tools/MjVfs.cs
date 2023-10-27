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
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

namespace Mujoco {

using static MujocoLib;

// A wrapper around Mujoco's native _mjVFS that simplifies the process of creating memory files.
public sealed class MjVfs : IDisposable {

  private _mjVFS _managedVfs;
  private IntPtr _unmanagedVfs;

  // Managed version of the underlying _mjVFS structure.
  public _mjVFS Data {
    get {
      _managedVfs = (_mjVFS)Marshal.PtrToStructure(_unmanagedVfs, typeof(_mjVFS));
      return _managedVfs;
    }
    set {
      _managedVfs = value;
      Marshal.StructureToPtr(_managedVfs, _unmanagedVfs, fDeleteOld: false);
    }
  }

  // Number of files added to the filesystem.
  public int FilesCount {
    get { return Data.nfile; }
  }

  // Adds a new file to the virtual filesystem.
  public unsafe void AddFile(string filename, string contents) {
    var result = mj_makeEmptyFileVFS(_unmanagedVfs.ToPointer(), filename, contents.Length);
    if (result != 0) {
      throw new Exception(
          "VFS error (" + result + ") encountered while creating an empty file");
    }
    var fileIndex = mj_findFileVFS(_unmanagedVfs.ToPointer(), filename);
    if (fileIndex < 0) {
      throw new IndexOutOfRangeException("VFS didn't properly create the empty file.");
    }
    var contents_bytes = Encoding.UTF8.GetBytes(contents);
    Marshal.Copy(contents_bytes, 0, Data.filedata[fileIndex], contents_bytes.Length);
  }

  // Searches the VFS for the specified file and returns its index.
  // The index then can be used to retrieve the file contents from Data.filedata array.
  public unsafe int FindFile(string filename) {
    return mj_findFileVFS(_unmanagedVfs.ToPointer(), filename);
  }

  // Loads a model from the specified file.
  // The file is assumed to be located in the filesystem. If it's not found, the method will throw
  // an ArgumentException.
  public unsafe MujocoLib.mjModel_* LoadXML(string filename) {
    if (FindFile(filename) < 0) {
      throw new ArgumentException($"File {filename} was not added to the VFS.");
    }

    var errorBuf = new StringBuilder(1024);
    MujocoLib.mjModel_* model = MujocoLib.mj_loadXML(
      filename, _unmanagedVfs.ToPointer(), errorBuf, errorBuf.Capacity);
    if (model == null || errorBuf.Length > 0) {
      throw new IOException($"Error loading the model: {errorBuf}");
    }
    return model;
  }

  public unsafe MjVfs() {
    _unmanagedVfs = Marshal.AllocHGlobal(Marshal.SizeOf(_managedVfs));
    mj_defaultVFS(_unmanagedVfs.ToPointer());
    _managedVfs = (_mjVFS)Marshal.PtrToStructure(_unmanagedVfs, typeof(_mjVFS));
  }

  ~MjVfs() {
    ReleaseUnmanagedMemory();
  }

  public void Dispose() {
    GC.SuppressFinalize(this);
    ReleaseUnmanagedMemory();
  }

  private unsafe void ReleaseUnmanagedMemory() {
    if (_unmanagedVfs != IntPtr.Zero) {
      mj_deleteVFS(_unmanagedVfs.ToPointer());
      Marshal.FreeHGlobal(_unmanagedVfs);
      _unmanagedVfs = IntPtr.Zero;
    }
  }
}
}
