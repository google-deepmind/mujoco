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

public class MjExclude : MjComponent {

  [Tooltip("A body whose contacts with other body are ignored")]
  public MjBody Body1;
  [Tooltip("Other body whose contacts with first body are ignored")]
  public MjBody Body2;

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_EXCLUDE;

  protected override void OnParseMjcf(XmlElement mjcf) {
    var body1Name = mjcf.GetStringAttribute("body1", defaultValue: string.Empty);
    if (!string.IsNullOrEmpty(body1Name)) {
      Body1 = MjHierarchyTool.FindComponentOfTypeAndName<MjBody>(body1Name);
    }
    var body2Name = mjcf.GetStringAttribute("body2", defaultValue: string.Empty);
    if (!string.IsNullOrEmpty(body2Name)) {
      Body2 = MjHierarchyTool.FindComponentOfTypeAndName<MjBody>(body2Name);
    }
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    if (Body1 == null || Body2 == null) {
      throw new NullReferenceException($"Both bodies in {name} must be assigned.");
    }

    var mjcf = (XmlElement)doc.CreateElement("exclude");
    mjcf.SetAttribute("body1", Body1.MujocoName);
    mjcf.SetAttribute("body2", Body2.MujocoName);
    return mjcf;
  }
}
}
