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

public class MjBodyVectorSensor : MjBaseSensor {
  // NOTE: These names must match sensor names listed on mujoco.org.
  // Changing them is justified only when Mujoco is upgraded to a new version.
  public enum AvailableSensors {
    SubtreeCom,
    SubtreeLinVel,
    SubtreeAngMom,
    FramePos,
    FrameXAxis,
    FrameYAxis,
    FrameZAxis,
    FrameLinVel,
    FrameAngVel,
    FrameLinAcc,
    FrameAngAcc,
  }
  public AvailableSensors SensorType;
  public MjBaseBody Body;

  [Tooltip("Should the Frame sensors use the inertial or the regular frame of reference.")]
  public bool UseInertialFrame;

  public Vector3 SensorReading { get; private set; }

  protected override XmlElement ToMjcf(XmlDocument doc) {
    if (Body == null) {
      throw new NullReferenceException("Missing a reference to a MjBody.");
    }
    var tag = SensorType.ToString().ToLower();
    var mjcf = doc.CreateElement(tag);
    if (tag.Contains("frame")) {
      mjcf.SetAttribute("objtype", UseInertialFrame ? "body" : "xbody");
      mjcf.SetAttribute("objname", Body.MujocoName);
    } else {
      mjcf.SetAttribute("body", Body.MujocoName);
    }
    return mjcf;
  }

  protected override void FromMjcf(XmlElement mjcf) {
    if (!Enum.TryParse(mjcf.Name, ignoreCase: true, result: out SensorType)) {
      throw new ArgumentException($"Unknown sensor type {mjcf.Name}.");
    }
    if (mjcf.Name.Contains("frame")) {
      UseInertialFrame = mjcf.HasAttribute("body"); // as opposed to xbody
      Body = mjcf.GetObjectReferenceAttribute<MjBaseBody>("objname");
    } else {
      Body = mjcf.GetObjectReferenceAttribute<MjBaseBody>("body");
    }
  }

  public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
    SensorReading = MjEngineTool.UnityVector3(data->sensordata + _sensorAddress);
  }
}
}
