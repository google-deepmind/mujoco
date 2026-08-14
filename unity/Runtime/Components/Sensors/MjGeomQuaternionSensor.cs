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
public class MjGeomQuaternionSensor : MjBaseSensor {
  public MjGeom Geom;

  public Quaternion SensorReading { get; private set; }

  protected override XmlElement ToMjcf(XmlDocument doc) {
    if (Geom == null) {
      throw new NullReferenceException("Missing a reference to a MjGeom.");
    }
    var mjcf = doc.CreateElement("framequat");
    mjcf.SetAttribute("objtype", "geom");
    mjcf.SetAttribute("objname", Geom.MujocoName);
    return mjcf;
  }

  protected override void FromMjcf(XmlElement mjcf) {
    Geom = mjcf.GetObjectReferenceAttribute<MjGeom>("objname");
    if (Geom == null) {
      throw new NullReferenceException("Missing a reference to a MjGeom.");
    }
  }

  public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
    SensorReading = MjEngineTool.UnityQuaternion(data->sensordata + _sensorAddress);
  }
}
}
