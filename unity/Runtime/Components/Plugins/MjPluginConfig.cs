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
using System.Collections.Generic;
using System.Linq;
using System.Xml;
using UnityEngine;

namespace Mujoco {
public class MjPluginConfig : MjComponent {
  public string Key = "";
  public string Value = "";

  protected override bool _suppressNameAttribute => true;

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_PLUGIN;

  // Parse the component settings from an external Mjcf.
  protected override void OnParseMjcf(XmlElement mjcf) {
    Key = mjcf.GetStringAttribute("key", "");
    Value = mjcf.GetStringAttribute("value", "");
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {

    var mjcf = (XmlElement)doc.CreateElement("plugin");
    if (Key.Length > 0)
      mjcf.SetAttribute("key", Key);
    if (Value.Length > 0)
      mjcf.SetAttribute("value", Value);
    return mjcf;
  }
}
}
