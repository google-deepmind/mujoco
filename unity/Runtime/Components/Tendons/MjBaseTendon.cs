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
using System.Xml;
using UnityEngine;

namespace Mujoco {

public abstract class MjBaseTendon : MjComponent {

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_TENDON;

  public SolverSettings Solver = SolverSettings.Default;

  [Tooltip("Lower bound of deadband resting length at zero spring force. If negative, computed at qpos0.")]
  public float SpringLengthLower = -1.0f;
  [Tooltip("Upper bound of deadband resting length at zero spring force. If negative, computed at qpos0.")]
  public float SpringLengthUpper = -1.0f;
  public float Stiffness = 0.0f;
  public float Damping = 0.0f;

  // Tendon length.
  public float Length { get; private set; }

  // Create the implementation dependent Mjcf node.
  protected abstract XmlElement ToMjcf(XmlDocument doc);

  // Parse the implementation dependent details from the provided Mjcf node.
  protected abstract void FromMjcf(XmlElement mjcf);

  // Parse the component settings from an external Mjcf.
  protected override void OnParseMjcf(XmlElement mjcf) {
    Solver.FromMjcf(mjcf);
    var springLengthValues = mjcf.GetFloatArrayAttribute(
        "springlength", defaultValue: new float[] {-1.0f, -1.0f}, fillMissingValues: false);
    if (springLengthValues.Length == 1) {
      SpringLengthLower = springLengthValues[0];
      SpringLengthUpper = springLengthValues[0];
    } else if (springLengthValues.Length == 2) {
      SpringLengthLower = springLengthValues[0];
      SpringLengthUpper = springLengthValues[1];
    } else {
      throw new ArgumentException("Invalid springlength string representation.");
    }
    Stiffness = mjcf.GetFloatAttribute("stiffness");
    Damping = mjcf.GetFloatAttribute("damping");
    FromMjcf(mjcf);
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    var mjcf = ToMjcf(doc);
    Solver.ToMjcf(mjcf);
    if (SpringLengthLower > SpringLengthUpper) {
      throw new ArgumentException("Lower spring length value can't be bigger than Upper");
    }
    mjcf.SetAttribute(
        "springlength",
        MjEngineTool.MakeLocaleInvariant($"{SpringLengthLower} {SpringLengthUpper}"));
    mjcf.SetAttribute("damping", MjEngineTool.MakeLocaleInvariant($"{Damping}"));
    mjcf.SetAttribute("stiffness", MjEngineTool.MakeLocaleInvariant($"{Stiffness}"));
    return mjcf;
  }

  // Synchronize the state of the component.
  public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
    Length = (float)data->ten_length[MujocoId];
  }
}
}
