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
public class MjJointScalarSensor : MjBaseSensor {
  // NOTE: These names must match sensor names listed on mujoco.org.
  // Changing them is justified only when Mujoco is upgraded to a new version.
  public enum AvailableSensors {
    JointPos,
    JointVel,
    JointLimitPos,
    JointLimitVel,
    JointLimitFrc,
  }
  public AvailableSensors SensorType;
  public MjBaseJoint Joint;

  public double SensorReading { get; private set; }

  protected override XmlElement ToMjcf(XmlDocument doc) {
    if (Joint == null) {
      throw new NullReferenceException("Missing a reference to a Joint.");
    }
    var mjcf = doc.CreateElement(SensorType.ToString().ToLower());
    mjcf.SetAttribute("joint", Joint.MujocoName);
    return mjcf;
  }

  protected override void FromMjcf(XmlElement mjcf) {
    if (!Enum.TryParse(mjcf.Name, ignoreCase: true, result: out SensorType)) {
      throw new ArgumentException($"Unknown sensor type {mjcf.Name}.");
    }
    Joint = mjcf.GetObjectReferenceAttribute<MjBaseJoint>("joint");
  }

  public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
     SensorReading = data->sensordata[_sensorAddress];
  }
}
}
