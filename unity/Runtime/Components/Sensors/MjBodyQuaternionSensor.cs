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

public class MjBodyQuaternionSensor : MjBaseSensor {
  public MjBaseBody Body;

  [Tooltip("Should the Frame sensors use the inertial or the regular frame of reference.")]
  public bool UseInertialFrame;

  public Quaternion SensorReading { get; private set; }

  protected override XmlElement ToMjcf(XmlDocument doc) {
    if (Body == null) {
      throw new NullReferenceException("Missing a reference to a MjBody.");
    }
    var mjcf = doc.CreateElement("framequat");
    mjcf.SetAttribute("objtype", UseInertialFrame ? "body" : "xbody");
    mjcf.SetAttribute("objname", Body.MujocoName);
    return mjcf;
  }

  protected override void FromMjcf(XmlElement mjcf) {
    UseInertialFrame = mjcf.HasAttribute("body");
    Body = mjcf.GetObjectReferenceAttribute<MjBaseBody>("objname");
    if (Body == null) {
      throw new NullReferenceException("Missing a reference to a MjBody.");
    }
  }

  public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
    SensorReading = MjEngineTool.UnityQuaternion(data->sensordata + _sensorAddress);
  }
}
}
