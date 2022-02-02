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

  public class MjTendonConstraint : MjBaseConstraint {
    public MjBaseTendon Tendon1;
    public MjBaseTendon Tendon2;
    public float[] PolynomialCoefficients = new float[5] { 0, 1, 0, 0, 0 };
    protected override string _constraintName => "tendon";

    protected override void FromMjcf(XmlElement mjcf) {
      Tendon1 = mjcf.GetObjectReferenceAttribute<MjBaseTendon>("tendon1");
      Tendon2 = mjcf.GetObjectReferenceAttribute<MjBaseTendon>("tendon2");
      PolynomialCoefficients =
          mjcf.GetFloatArrayAttribute("polycoef", new Single[] { 0, 1, 0, 0, 0 });
    }

    // Generate implementation specific XML element.
    protected override void ToMjcf(XmlElement mjcf) {
      if (Tendon1 == null) {
        throw new NullReferenceException(
            $"At least Tendon1 in constraint {name} must be assigned.");
      }

      mjcf.SetAttribute("tendon1", Tendon1.MujocoName);
      if (Tendon2 != null) {
        mjcf.SetAttribute("tendon2", Tendon2.MujocoName);
      }
      mjcf.SetAttribute("polycoef", MjEngineTool.ArrayToMjcf(PolynomialCoefficients));
    }

    public void OnValidate() {
      if (Tendon1 != null && Tendon1 == Tendon2) {
        Debug.LogWarning(
            "Tendon1 and Tendon2 can't be the same - resetting Tendon2.", this);
        Tendon2 = null;
      }
    }
  }
}
