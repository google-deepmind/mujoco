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
public class MjCapsuleShape : IMjShape {
  public float Radius = 0.5f;
  public float HalfHeight = 0.5f;
  private const float _scaleTolerance = 1e-5f;

  public void ToMjcf(XmlElement mjcf, Transform transform) {
    if (Math.Abs(transform.lossyScale.x - transform.lossyScale.z) > _scaleTolerance) {
      Debug.LogWarning(
          $"{transform.name}: Capsule shapes require uniform scaling of XZ plane. Using the" +
          " value of X and Y components.", transform);
    }
    mjcf.SetAttribute(
        "size", MjEngineTool.MakeLocaleInvariant($"{Radius * transform.lossyScale.x} {HalfHeight * transform.lossyScale.y}"));
  }

  public void FromMjcf(XmlElement mjcf) {
    var components = mjcf.GetFloatArrayAttribute("size", defaultValue: new float[] { 0.5f, 0.5f });
    Radius = components[0];
    HalfHeight = components[1];

    Vector3 fromPoint, toPoint;
    if (MjEngineTool.ParseFromToMjcf(mjcf, out fromPoint, out toPoint)) {
      HalfHeight = 0.5f * (toPoint - fromPoint).magnitude;
    }
  }

  public Tuple<Vector3[], int[]> BuildMesh() {
    return MeshGenerators.BuildCapsule(radius: Radius, height: (HalfHeight + Radius) * 2.0f);
  }

  public Vector4 GetChangeStamp() {
    return new Vector4(Radius, HalfHeight, 1, 0);
  }

  public void DebugDraw(Transform transform) {
    Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
    MjGizmos.DrawWireCapsule(
        Vector3.zero, Radius * transform.lossyScale.x, 2 * HalfHeight * transform.lossyScale.y);
  }
}
}
