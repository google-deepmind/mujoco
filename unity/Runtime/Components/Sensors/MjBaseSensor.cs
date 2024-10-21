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

// The base class for Mujoco sensors.
//
// Mujoco offers a very wide range of sensors, each contingent on the following three aspects:
//  1. the Mujoco Component it observes: Body, Joint, etc.
//  2. the type of data it returns: scalar, Vector3, quaternion, etc.
//  3. the field it reads to retreive the data: xpos, xquat, cfrc, etc.
//
// In Mujoco proper, names of some of the sensors repeat for many data types. In order to get rid
// of this ambiguity, we chose to split the sensors into groups.
// Each sensor class represents a group of aspects (1) and (2), which is reflected in its name.
// For example, the class of sensors that listen to Bodies and produce Vector3 observations is
// called MjBodyVectorSensor.
//
// That very class contains a wide range of sensing capabilities - as in fact most of the classes
// do. What each sensor can sense can be controlled by the sensor's Type field. Using the
// MjBodyVectorSensor as an example, one can set it to SubtreeCom, FramePos or any of the other
// supported observation types.
public abstract class MjBaseSensor : MjComponent {

  [Tooltip("When this value is positive, it limits the absolute value of the sensor output.")]
  [AbsoluteValue]
  public float Cutoff = 0.0f;

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_SENSOR;

  // Address of the sensor, that can be used to index into MujocoLib.mjData_.sensordata.
  protected int _sensorAddress;

  // Parse the component settings from an external Mjcf.
  protected override void OnParseMjcf(XmlElement mjcf) {
    Cutoff = mjcf.GetFloatAttribute("cutoff", defaultValue: 0.0f);
    FromMjcf(mjcf);
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    var mjcf = ToMjcf(doc);
    mjcf.SetAttribute("cutoff", Cutoff.ToString());
    return mjcf;
  }

  // Perform bind time initialization of the component.
  protected override unsafe void OnBindToRuntime(MujocoLib.mjModel_* model, MujocoLib.mjData_* data) {
    _sensorAddress = model->sensor_adr[MujocoId];
  }

  // Create the implementation dependent Mjcf node.
  protected abstract XmlElement ToMjcf(XmlDocument doc);

  // Parse the implementation dependent details from the provided Mjcf node.
  protected abstract void FromMjcf(XmlElement mjcf);
}
}
