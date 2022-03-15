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

  public class MjDistance : MjBaseConstraint {
    public MjGeom Geom1;
    public MjGeom Geom2;
    protected override string _constraintName => "distance";

    protected override void FromMjcf(XmlElement mjcf) {
      Geom1 = mjcf.GetObjectReferenceAttribute<MjGeom>("geom1");
      Geom2 = mjcf.GetObjectReferenceAttribute<MjGeom>("geom2");
    }

    protected override void ToMjcf(XmlElement mjcf) {
      if (Geom1 == null || Geom2 == null) {
        throw new NullReferenceException($"Both geoms in distance {name} must be assigned.");
      }

      mjcf.SetAttribute("geom1", Geom1.MujocoName);
      mjcf.SetAttribute("geom2", Geom2.MujocoName);
    }

    public void OnValidate() {
      if (Geom1 != null && Geom1 == Geom2) {
        Debug.LogError("Geom1 and Geom2 can't be the same - resetting Geom2.", this);
        Geom2 = null;
      }
    }
  }
}
