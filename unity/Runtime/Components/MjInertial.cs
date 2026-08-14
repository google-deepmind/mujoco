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

// Defines an inertial frame of a body.
public class MjInertial : MjComponent {

  [Tooltip("Mass assigned to the parent body.")]
  public float Mass = 1.0f;

  [Tooltip("Diagonal inertia matrix, expressing the body inertia relative to the inertial frame.")]
  public Vector3 DiagInertia = Vector3.one;

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_BODY;
  protected override bool _suppressNameAttribute => true;

  // Parse the component settings from an external Mjcf.
  protected override void OnParseMjcf(XmlElement mjcf) {
    MjEngineTool.ParseTransformMjcf(mjcf, transform);
    Mass = mjcf.GetFloatAttribute("mass", defaultValue: 1.0f);
    DiagInertia = MjEngineTool.UnityExtents(
        mjcf.GetVector3Attribute("diaginertia", defaultValue: Vector3.one));
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    var mjcf = (XmlElement)doc.CreateElement("inertial");
    MjEngineTool.PositionRotationToMjcf(mjcf, this);
    mjcf.SetAttribute("mass", MjEngineTool.MakeLocaleInvariant($"{Mass}"));
    mjcf.SetAttribute("diaginertia",
                      MjEngineTool.Vector3ToMjcf(MjEngineTool.MjExtents(DiagInertia)));
    return mjcf;
  }

  protected void OnDrawGizmosSelected() {
    Gizmos.color = Color.yellow;
    var oldMatrix = Gizmos.matrix;
    Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.lossyScale);
    var box = new Vector3(
        Mathf.Sqrt((DiagInertia[1] + DiagInertia[2] - DiagInertia[0]) / Mass * 6.0f),
        Mathf.Sqrt((DiagInertia[0] + DiagInertia[2] - DiagInertia[1]) / Mass * 6.0f),
        Mathf.Sqrt((DiagInertia[0] + DiagInertia[1] - DiagInertia[2]) / Mass * 6.0f));
    Gizmos.DrawWireCube(Vector3.zero, box);
    Gizmos.matrix = oldMatrix;
  }
}
}
