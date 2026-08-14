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

  public class MjWeld : MjBaseConstraint {
    public MjBaseBody Body1;
    public MjBaseBody Body2;
    public Transform WeldOffset;
    protected override string _constraintName => "weld";

    protected override unsafe void OnBindToRuntime(MujocoLib.mjModel_* model, MujocoLib.mjData_* data) {
      if (WeldOffset) {
        MjEngineTool.SetMjTransform(
            MjEngineTool.MjEqualityAtEntry(model->eq_data, MujocoId)+3,
            WeldOffset.localPosition, WeldOffset.localRotation);
      }
    }

    protected override void FromMjcf(XmlElement mjcf) {
      Body1 = mjcf.GetObjectReferenceAttribute<MjBaseBody>("body1");
      Body2 = mjcf.GetObjectReferenceAttribute<MjBaseBody>("body2");
      var relpose = mjcf.GetStringAttribute("relpose");
      if (relpose != null) {
        Debug.Log(
            $"relpose {relpose} in weld {name} ignored. Set WeldOffset in the editor.");
      }
    }

    protected override void ToMjcf(XmlElement mjcf) {
      if (Body1 == null || Body2 == null) {
        throw new NullReferenceException($"Both bodies in weld {name} must be assigned.");
      }

      mjcf.SetAttribute("body1", Body1.MujocoName);
      mjcf.SetAttribute("body2", Body2.MujocoName);
    }

    public void OnValidate() {
      if (Body1 != null && Body1 == Body2) {
        Debug.LogWarning("Body1 and Body2 can't be the same - resetting Body2.", this);
        Body2 = null;
      }
    }
  }
}
