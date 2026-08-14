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
using System.Linq;
using UnityEditor.AssetImporters;
using UnityEngine;

namespace Mujoco {
// Importer for STL mesh files.
[ScriptedImporter(version: 1, ext: "stl")]
public class StlMeshImporter : ScriptedImporter {

  public override void OnImportAsset(AssetImportContext ctx) {
    var assetName = Path.GetFileNameWithoutExtension(ctx.assetPath);
    var modelContents = File.ReadAllBytes(ctx.assetPath);

    var mesh = StlMeshParser.ParseBinary(modelContents, Vector3.one);

    ctx.AddObjectToAsset(assetName, mesh);
    ctx.SetMainObject(mesh);
  }
}
}
