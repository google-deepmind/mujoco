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
public class MjEllipsoidShape : IMjShape {
  public Vector3 Radiuses = Vector3.one * 0.5f;

  public void ToMjcf(XmlElement mjcf, Transform transform) {
    var scaledRadiuses = MjEngineTool.MjExtents(Vector3.Scale(Radiuses, transform.lossyScale));
    mjcf.SetAttribute("size", MjEngineTool.Vector3ToMjcf(scaledRadiuses));
  }

  public void FromMjcf(XmlElement mjcf) {
    Radiuses = MjEngineTool.UnityExtents(
        mjcf.GetVector3Attribute("size", defaultValue: Vector3.one * 0.5f));

    Vector3 fromPoint, toPoint;
    if (MjEngineTool.ParseFromToMjcf(mjcf, out fromPoint, out toPoint)) {
      var radius = (toPoint - fromPoint).magnitude * 0.5f;
      Radiuses = Vector3.one * radius;
    }
  }

  public Tuple<Vector3[], int[]> BuildMesh() {
    return MeshGenerators.BuildSphere(scale: Radiuses);
  }

  public Vector4 GetChangeStamp() {
    return Radiuses;
  }

  public void DebugDraw(Transform transform) {
    Gizmos.matrix = Matrix4x4.TRS(
        transform.position,
        transform.rotation,
        Vector3.Scale(Radiuses, transform.lossyScale));
    Gizmos.DrawWireSphere(Vector3.zero, 1.0f);
  }
}
}
