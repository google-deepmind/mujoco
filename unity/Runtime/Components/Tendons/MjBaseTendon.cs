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

  [Tooltip("Length at zero spring force. If negative, this resting length is computed at qpos0.")]
  public float SpringLength = -1.0f;
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
    SpringLength = mjcf.GetFloatAttribute("springlength", defaultValue: -1.0f);
    Stiffness = mjcf.GetFloatAttribute("stiffness");
    Damping = mjcf.GetFloatAttribute("damping");
    FromMjcf(mjcf);
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    var mjcf = ToMjcf(doc);
    Solver.ToMjcf(mjcf);
    mjcf.SetAttribute("springlength", MjEngineTool.MakeLocaleInvariant($"{SpringLength}"));
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
