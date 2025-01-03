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

  public class MjFlexConstraint : MjBaseConstraint {
    public string Flex;
    protected override string _constraintName => "flex";

    protected override void FromMjcf(XmlElement mjcf) {
      Flex = mjcf.GetStringAttribute("flex");
    }

    protected override void ToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("flex", Flex);
    }

  }
}
