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

public class MjPluginTag : MjComponent {

  //Plugin identifier, used for implicit plugin instantiation.
  public string Plugin = "";

  //Instance name, used for explicit plugin instantiation.
  public string Instance = "";

  protected override bool _suppressNameAttribute => true;

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_PLUGIN;

  // Parse the component settings from an external Mjcf.
  protected override void OnParseMjcf(XmlElement mjcf) {
    Plugin = mjcf.GetStringAttribute("plugin", "");
    Instance = mjcf.GetStringAttribute("instance", "");
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {

    var mjcf = (XmlElement)doc.CreateElement("plugin");
    if (Plugin.Length > 0)
      mjcf.SetAttribute("plugin", Plugin);
    if (Instance.Length > 0)
      mjcf.SetAttribute("instance", Instance);
    return mjcf;
  }
}
}
