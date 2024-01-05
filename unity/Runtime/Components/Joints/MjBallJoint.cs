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

  // The component represents a joint with 1 degree of translational freedom.
  public class MjBallJoint : MjBaseJoint {
    [Tooltip("In degrees.")]
    public float RangeUpper;

    [Tooltip("Joint settings.")]
    public MjJointSettings Settings = MjJointSettings.Default;

    protected override void OnParseMjcf(XmlElement mjcf) {
      // Transform.
      transform.localPosition =
          MjEngineTool.UnityVector3(mjcf.GetVector3Attribute("pos", defaultValue: Vector3.zero));

      Settings.FromMjcf(mjcf);
      var rangeValues = mjcf.GetFloatArrayAttribute("range", defaultValue: new float[] { 0, 0 });
      // rangeValues[0] is always 0 for ball joints.
      RangeUpper = rangeValues[1];
    }

    protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
      var mjcf = (XmlElement)doc.CreateElement("joint");
      mjcf.SetAttribute("type", "ball");

      MjEngineTool.PositionToMjcf(mjcf, this);

      Settings.ToMjcf(mjcf);
      mjcf.SetAttribute("range", MjEngineTool.MakeLocaleInvariant($"0 {RangeUpper}"));

      return mjcf;
    }
  }
}
