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
using System.Linq;
using System.Xml;
using UnityEngine;

namespace Mujoco {

[Serializable]
public class MjMeshShape : IMjShape {

  public Mesh Mesh;

  public void ToMjcf(XmlElement mjcf, Transform transform) {
    var scene = MjScene.Instance;
    var assetName = scene.GenerationContext.AddMeshAsset(Mesh);
    mjcf.SetAttribute("mesh", assetName);
  }

  public void FromMjcf(XmlElement mjcf) {
    // When the asset was parsed and stored its name was sanitized, so we should load it using a
    // sanitized name:
    var assetName = MjEngineTool.Sanitize(
        mjcf.GetStringAttribute("mesh", defaultValue: string.Empty));
    if (!string.IsNullOrEmpty(assetName)) {
      Mesh = Resources.Load<Mesh>(assetName);
    }
  }

  public Tuple<Vector3[], int[]> BuildMesh() {
    return Tuple.Create(Mesh.vertices, Mesh.triangles);
  }

  public Vector4 GetChangeStamp() {
    return Vector4.one;
  }

  public void DebugDraw(Transform transform) {
    Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale);
    Gizmos.DrawWireMesh(Mesh);
  }
}
}
