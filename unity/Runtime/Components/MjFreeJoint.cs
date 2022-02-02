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

  // The component represents a joint with 6 degrees of freedom.
  public class MjFreeJoint : MjBaseJoint {
    protected override void OnParseMjcf(XmlElement mjcf) {}

    protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
      return (XmlElement)doc.CreateElement("freejoint");
    }
  }
}
