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

  public class MjJointConstraint : MjBaseConstraint {
    [Tooltip("Mandatory.")]
    public MjBaseJoint Joint1;

    [Tooltip("Optional - if empty, constrain the value of Joint1")]
    public MjBaseJoint Joint2;

    public float[] PolynomialCoefficients = new float[5] { 0, 1, 0, 0, 0 };
    protected override string _constraintName => "joint";

    protected override void FromMjcf(XmlElement mjcf) {
      Joint1 = mjcf.GetObjectReferenceAttribute<MjBaseJoint>("joint1");
      Joint2 = mjcf.GetObjectReferenceAttribute<MjBaseJoint>("joint2");
      PolynomialCoefficients =
          mjcf.GetFloatArrayAttribute("polycoef", defaultValue: new float[] { 0, 1, 0, 0, 0 });
    }

    // Generate implementation specific XML element.
    protected override void ToMjcf(XmlElement mjcf) {
      if (Joint1 == null) {
        throw new NullReferenceException($"At least Joint1 in constraint {name} must be assigned.");
      }

      mjcf.SetAttribute("joint1", Joint1.MujocoName);
      if (Joint2 != null) {
        mjcf.SetAttribute("joint2", Joint2.MujocoName);
      }
      mjcf.SetAttribute("polycoef", MjEngineTool.ArrayToMjcf(PolynomialCoefficients));
    }

    public void OnValidate() {
      if (Joint1 != null && Joint1 == Joint2) {
        Debug.LogError("Joint1 and Joint2 can't be the same - resetting Joint2.", this);
        Joint2 = null;
      }
    }
  }
}
