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
[Serializable]
public class FixedTendonEntry {
  [Tooltip("Only scalar joints can be used - hinge and slide.")]
  public MjBaseJoint Joint;

  [Tooltip("Multiplier for the value of the joint.")]
  public float Coefficient = 1.0f;
}

public class MjFixedTendon : MjBaseTendon {

  [Tooltip("In (scaled) radians/meters.")]
  public float RangeLower;
  [Tooltip("In (scaled) radians/meters.")]
  public float RangeUpper;

  public List<FixedTendonEntry> JointList = new List<FixedTendonEntry>() {};

  protected override void FromMjcf(XmlElement mjcf) {
    foreach (var child in mjcf.Cast<XmlNode>().OfType<XmlElement>()) {
      var fixedTendonEntry = new FixedTendonEntry();
      fixedTendonEntry.Joint = child.GetObjectReferenceAttribute<MjBaseJoint>("joint");
      fixedTendonEntry.Coefficient = child.GetFloatAttribute("coef", defaultValue: 1.0f);
      JointList.Add(fixedTendonEntry);
    }
    var rangeValues = mjcf.GetFloatArrayAttribute("range", defaultValue: new float[] { 0, 0 });
    RangeLower = rangeValues[0];
    RangeUpper = rangeValues[1];
  }

  protected override XmlElement ToMjcf(XmlDocument doc) {
    if (JointList.Count < 1) {
      throw new ArgumentOutOfRangeException($"Fixed tendon {name} needs at least one joint.");
    }
    var mjcf = doc.CreateElement("fixed");
    foreach (FixedTendonEntry fixedTendonEntry in JointList) {
      var jointMjcf = doc.CreateElement("joint");
      jointMjcf.SetAttribute("joint", fixedTendonEntry.Joint.MujocoName);
      jointMjcf.SetAttribute("coef", MjEngineTool.MakeLocaleInvariant($"{fixedTendonEntry.Coefficient}"));
      mjcf.AppendChild(jointMjcf);
    }
    if (RangeLower > RangeUpper) {
      throw new ArgumentException("Lower range value can't be bigger than Higher");
    }
    mjcf.SetAttribute("range", MjEngineTool.MakeLocaleInvariant($"{RangeLower} {RangeUpper}"));

    return mjcf;
  }

  public void OnValidate() {
    foreach (FixedTendonEntry fixedTendonEntry in JointList) {
      if (!(fixedTendonEntry.Joint is MjHingeJoint) &&
          !(fixedTendonEntry.Joint is MjSlideJoint)) {
        Debug.LogError("Only scalar joints (hinge or slide) are allowed.", this);
        fixedTendonEntry.Joint = null;
      }
    }
  }
}
}
