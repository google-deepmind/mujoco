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
using System.Xml;
using UnityEngine;

namespace Mujoco {

  // The component represents the apex of hierarchy that defines a single rigid body.
  public class MjBody : MjBaseBody {

    [Tooltip("Gravity compensation force, specified as fraction of body weight.")]
    public float GravityCompensation;

    protected override void OnParseMjcf(XmlElement mjcf) {
      // Transform
      transform.localPosition =
          MjEngineTool.UnityVector3(mjcf.GetVector3Attribute("pos", defaultValue: Vector3.zero));
      transform.localRotation = MjEngineTool.UnityQuaternion(
          mjcf.GetQuaternionAttribute("quat", defaultValue: MjEngineTool.MjQuaternionIdentity));
      GravityCompensation = mjcf.GetFloatAttribute("gravcomp", defaultValue: 0.0f);
    }

    protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
      var mjcf = (XmlElement)doc.CreateElement("body");
      MjEngineTool.PositionRotationToMjcf(mjcf, this);
      mjcf.SetAttribute("gravcomp", MjEngineTool.MakeLocaleInvariant($"{GravityCompensation}"));
      return mjcf;
    }

    public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
      transform.position = MjEngineTool.UnityVector3(
          MjEngineTool.MjVector3AtEntry(data->xpos, MujocoId));
      transform.rotation = MjEngineTool.UnityQuaternion(
          MjEngineTool.MjQuaternionAtEntry(data->xquat, MujocoId));
    }
  }
}
