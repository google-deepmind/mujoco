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
  public class MjMocapBody : MjBaseBody {
    protected override void OnParseMjcf(XmlElement mjcf) {
      // Transform
      transform.localPosition =
          MjEngineTool.UnityVector3(mjcf.GetVector3Attribute("pos", defaultValue: Vector3.zero));
      transform.localRotation = MjEngineTool.UnityQuaternion(
          mjcf.GetQuaternionAttribute("quat", defaultValue: MjEngineTool.MjQuaternionIdentity));
    }

    protected override unsafe void OnBindToRuntime(MujocoLib.mjModel_* model, MujocoLib.mjData_* data) {
      var bodyId = MujocoLib.mj_name2id(model, (int)ObjectType, MujocoName);
      MujocoId = model->body_mocapid[bodyId];
    }

    protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
      var mjcf = doc.CreateElement("body");
      MjEngineTool.PositionRotationToMjcf(mjcf, this);
      mjcf.SetAttribute("mocap", "true");
      return mjcf;
    }

    public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
      MjEngineTool.SetMjVector3(
          MjEngineTool.MjVector3AtEntry(data->mocap_pos, MujocoId), transform.position);
      MjEngineTool.SetMjQuaternion(
          MjEngineTool.MjQuaternionAtEntry(data->mocap_quat, MujocoId), transform.rotation);
    }
  }
}
