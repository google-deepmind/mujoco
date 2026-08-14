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

  public class MjConnect : MjBaseConstraint {
    public MjBaseBody Body1;
    public MjBaseBody Body2;
    public Transform Anchor;
    protected override string _constraintName => "connect";

    protected override void FromMjcf(XmlElement mjcf) {
      Body1 = mjcf.GetObjectReferenceAttribute<MjBaseBody>("body1");
      Body2 = mjcf.GetObjectReferenceAttribute<MjBaseBody>("body2");
      var anchorPos = MjEngineTool.UnityVector3(
          mjcf.GetVector3Attribute("anchor", defaultValue: Vector3.zero));
      Anchor = new GameObject("connect_anchor").transform;
      Anchor.parent = Body1.transform;
      Anchor.localPosition = anchorPos;
    }

    // Generate implementation specific XML element.
    protected override void ToMjcf(XmlElement mjcf) {
      if (Body1 == null || Body2 == null) {
        throw new NullReferenceException($"Both bodies in connect {name} are required.");
      }
      if (Anchor == null) {
        throw new NullReferenceException($"Anchor in connect {name} is required.");
      }
      mjcf.SetAttribute("body1", Body1.MujocoName);
      mjcf.SetAttribute("body2", Body2.MujocoName);
      mjcf.SetAttribute("anchor",
                        MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(Anchor.localPosition)));
    }

    public void OnValidate() {
      if (Body1 != null && Body1 == Body2) {
        Debug.LogError("Body1 and Body2 can't be the same - resetting Body2.", this);
        Body2 = null;
      }
    }
  }
}
