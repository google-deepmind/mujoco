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
public class MjBoxShape : IMjShape {
  public Vector3 Extents = Vector3.one * 0.5f;

  public void ToMjcf(XmlElement mjcf, Transform transform) {
    var scaledExtents = MjEngineTool.MjExtents(Vector3.Scale(Extents, transform.lossyScale));
    mjcf.SetAttribute("size", MjEngineTool.Vector3ToMjcf(scaledExtents));
  }

  public void FromMjcf(XmlElement mjcf) {
    Extents = MjEngineTool.UnityExtents(
        mjcf.GetVector3Attribute("size", defaultValue: Vector3.one * 0.5f));

    Vector3 fromPoint, toPoint;
    if (MjEngineTool.ParseFromToMjcf(mjcf, out fromPoint, out toPoint)) {
      var extent = (toPoint - fromPoint).magnitude * 0.5f;
      Extents = Vector3.one * extent;
    }
  }

  public Tuple<Vector3[], int[]> BuildMesh() {
    return MeshGenerators.BuildBox(extents: Extents);
  }

  public Vector4 GetChangeStamp() {
    return Extents;
  }

  public void DebugDraw(Transform transform) {
    Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale);
    Gizmos.DrawWireCube(Vector3.zero, Extents * 2.0f);
  }
}
}
