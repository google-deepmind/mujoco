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

  // The component represents a joint with 1 degree of translational freedom.
  public class MjSlideJoint : MjBaseJoint {
    [Tooltip("In meters.")]
    public float RangeLower;
    [Tooltip("In meters.")]
    public float RangeUpper;

    [Tooltip("The current joint angle, in degrees. Read-only at play time.")]
    public float Configuration;

    [Tooltip("Joint movement, in rad/sec. Read-only at play time.")]
    public float Velocity;

    [Tooltip("Joint settings.")]
    public MjJointSettings Settings = MjJointSettings.Default;

    // World space slide axis.
    public unsafe Vector3 SlideAxis {
      get {
        if (MjScene.InstanceExists) {
          return MjEngineTool.UnityVector3(MjScene.Instance.Data->xaxis + 3 * MujocoId);
        }
        return transform.rotation * Vector3.right;
      }
    }

    protected override unsafe void OnBindToRuntime(MujocoLib.mjModel_* model, MujocoLib.mjData_* data) {
      base.OnBindToRuntime(model, data);
      data->qvel[DofAddress] = Velocity;
    }

    public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
      Configuration = (float)data->qpos[QposAddress];
      Velocity = (float)data->qvel[DofAddress];
    }

    protected override void OnParseMjcf(XmlElement mjcf) {
      // Transform.
      transform.localPosition =
          MjEngineTool.UnityVector3(mjcf.GetVector3Attribute("pos", defaultValue: Vector3.zero));

      var rotationAxis =
          MjEngineTool.UnityVector3(mjcf.GetVector3Attribute("axis", defaultValue: Vector3.right));
      transform.localRotation = Quaternion.FromToRotation(Vector3.right, rotationAxis);

      var rangeValues = mjcf.GetFloatArrayAttribute("range", defaultValue: new float[] { 0, 0 });
      RangeLower = rangeValues[0];
      RangeUpper = rangeValues[1];

      Configuration = mjcf.GetFloatAttribute("ref", 0.0f);

      Settings.FromMjcf(mjcf);
    }

    protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
      var mjcf = (XmlElement)doc.CreateElement("joint");
      mjcf.SetAttribute("type", "slide");

      MjEngineTool.PositionAxisToMjcf(mjcf, this);

      Settings.ToMjcf(mjcf);
      if (RangeLower > RangeUpper) {
        throw new ArgumentException("Lower range value can't be bigger than Higher");
      }
      mjcf.SetAttribute("range", MjEngineTool.MakeLocaleInvariant($"{RangeLower} {RangeUpper}"));
      mjcf.SetAttribute("ref", MjEngineTool.MakeLocaleInvariant($"{Configuration}"));

      return mjcf;
    }
  }
}
