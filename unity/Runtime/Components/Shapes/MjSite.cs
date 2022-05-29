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

// Defines a reference frame and an anchoring point for other elements of the rig.
public class MjSite : MjShapeComponent {

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_SITE;

  // Parse the component settings from an external Mjcf.
  protected override void OnParseMjcf(XmlElement mjcf) {
    ShapeFromMjcf(mjcf);
    MjEngineTool.ParseTransformMjcf(mjcf, transform);
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    var mjcf = (XmlElement)doc.CreateElement("site");
    ShapeToMjcf(mjcf, transform);
    MjEngineTool.PositionRotationToMjcf(mjcf, this);
    return mjcf;
  }

  // Synchronize the state of the component.
  public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
    transform.position = MjEngineTool.UnityVector3(
        MjEngineTool.MjVector3AtEntry(data->site_xpos, MujocoId));
    transform.rotation = MjEngineTool.UnityQuaternionFromMatrix(
        MjEngineTool.MjMatrixAtEntry(data->site_xmat, MujocoId));
  }

  public void OnDrawGizmosSelected() {
    Gizmos.color = Color.magenta;
    DrawGizmos(transform);
  }
}
}
