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

public class MjUserSensor : MjBaseSensor {

  public String Name;
  public int Dimension = 0;
  public String UserData;

  protected override XmlElement ToMjcf(XmlDocument doc) {
    if (Dimension == 0) {
      throw new MissingFieldException("Dimension should be larger than 0.");
    }
    var mjcf = doc.CreateElement("user");
    mjcf.SetAttribute("name", Name);
    mjcf.SetAttribute("dim", $"{Dimension}");
    if (!String.IsNullOrEmpty(UserData)) {
      // TODO: add validation that UserData is a space-separated list of floating numbers?
      mjcf.SetAttribute("user", UserData);
    }
    return mjcf;
  }

  protected override void FromMjcf(XmlElement mjcf) {
    Name = mjcf.GetAttribute("name");
    int.TryParse(mjcf.GetAttribute("dim"), out Dimension);
    UserData = mjcf.GetAttribute("user");
  }
}
}
