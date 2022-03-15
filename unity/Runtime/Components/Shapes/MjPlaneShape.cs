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
public class MjPlaneShape : IMjShape {
  public Vector2 Extents = Vector3.one;

  public void ToMjcf(XmlElement mjcf, Transform transform) {
    var extents = new Vector3(Extents.x, Extents.y, 1.0f);
    var scaledExtents = Vector3.Scale(extents, transform.lossyScale);
    mjcf.SetAttribute("size", MjEngineTool.MakeLocaleInvariant($"{scaledExtents.x} {scaledExtents.y} {scaledExtents.z}"));
  }

  public void FromMjcf(XmlElement mjcf) {
    var components = mjcf.GetFloatArrayAttribute("size", defaultValue: new float[] { 1, 1, 1 });
    Extents = new Vector2(components[0], components[1]);
  }

  public Tuple<Vector3[], int[]> BuildMesh() {
    return MeshGenerators.BuildPlane(Extents.x * 2.0f, Extents.y * 2.0f);
  }

  public Vector4 GetChangeStamp() {
    return new Vector4(Extents.x, Extents.y, 1, 0);
  }

  public void DebugDraw(Transform transform) {
    Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
    MjGizmos.DrawWirePlane(Vector3.zero, Extents.x * 2.0f, Extents.y * 2.0f);
  }
}
}
