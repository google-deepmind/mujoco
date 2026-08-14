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
public class MjSphereShape : IMjShape {
  public float Radius = 0.5f;
  private const float _scaleTolerance = 1e-6f;

  public void ToMjcf(XmlElement mjcf, Transform transform) {
    if (Math.Abs(transform.lossyScale.x - transform.lossyScale.y) > _scaleTolerance ||
        Math.Abs(transform.lossyScale.y - transform.lossyScale.z) > _scaleTolerance) {
      Debug.LogWarning(
          $"{transform.name}: Sphere shapes work only with uniform scaling. Using the value of X" +
          " component.\n Consider using Ellipsoid shape if you want to use non-uniform scaling.",
          transform);
    }
    mjcf.SetAttribute("size", MjEngineTool.MakeLocaleInvariant($"{Radius * transform.lossyScale.x}"));
  }

  public void FromMjcf(XmlElement mjcf) {
    var values = mjcf.GetFloatArrayAttribute("size", defaultValue: new float[] { 0.5f });
    Radius = values[0];
  }

  public Tuple<Vector3[], int[]> BuildMesh() {
    return MeshGenerators.BuildSphere(scale: Vector3.one * Radius);
  }

  public Vector4 GetChangeStamp() {
    return new Vector4(Radius, 1, 1, 0);
  }

  public void DebugDraw(Transform transform) {
    Gizmos.DrawWireSphere(transform.position, Radius * transform.lossyScale.x);
  }
}
}
