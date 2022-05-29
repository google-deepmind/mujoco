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

// The component represents a physical shape and models its inertia and material properties.
public class MjGeom : MjShapeComponent {
  [Tooltip("If larger than zero, Density has no effect.")]
  public float Mass = 0.0f;

  [Tooltip("Material density. Set to 0 for a zero-mass geom.")]
  public float Density = 1000.0f;

  [Tooltip("Advanced settings.")]
  public MjGeomSettings Settings = MjGeomSettings.Default;

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_GEOM;
  private MjTransformation _geomInGlobalFrame = new MjTransformation();
  private MjTransformation _comTransform;

  protected override void OnParseMjcf(XmlElement mjcf) {
    ShapeFromMjcf(mjcf);
    Mass = mjcf.GetFloatAttribute("mass", defaultValue: 0.0f);
    Density = mjcf.GetFloatAttribute("density", defaultValue: 1000.0f);
    MjEngineTool.ParseTransformMjcf(mjcf, transform);
    Settings.FromMjcf(mjcf);
  }

  // The MuJoCo compiler shifts meshes' frames, so we need to cache this transformation at init and
  // apply it at runtime.
  protected override unsafe void OnBindToRuntime(MujocoLib.mjModel_* model, MujocoLib.mjData_* data) {
    var MjParent = MjHierarchyTool.FindParentComponent<MjBaseBody>(this);
    if (MjParent != null) {
      var comInParentFrame = new MjTransformation(
          translation: MjEngineTool.UnityVector3(
              MjEngineTool.MjVector3AtEntry(model->geom_pos, MujocoId)),
          rotation: MjEngineTool.UnityQuaternion(
              MjEngineTool.MjQuaternionAtEntry(model->geom_quat, MujocoId)));

      // We don't want to bother calculating global transform in mujoco from mjModel,
      // so we'll assume it's the same as the Unity transfor (it's the beginning of simulation after
      // all).
      var globalParentFrame = MjTransformation.LoadGlobal(transform.parent);
      var comInGlobalFrame = globalParentFrame * comInParentFrame;
      var globalFrame = MjTransformation.LoadGlobal(transform);
      _comTransform = comInGlobalFrame.Inverse() * globalFrame;
    }
  }

  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    var mjcf = (XmlElement)doc.CreateElement("geom");
    if (Mass > 0) {
      mjcf.SetAttribute("mass", MjEngineTool.MakeLocaleInvariant($"{Mass}"));
    } else {
      mjcf.SetAttribute("density", MjEngineTool.MakeLocaleInvariant($"{Density}"));
    }
    ShapeToMjcf(mjcf, transform);
    MjEngineTool.PositionRotationToMjcf(mjcf, this);
    Settings.ToMjcf(mjcf);

    return mjcf;
  }

  public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
    if (ShapeType == ShapeTypes.Mesh) {
      _geomInGlobalFrame.Set(
          translation: MjEngineTool.UnityVector3(
              MjEngineTool.MjVector3AtEntry(data->geom_xpos, MujocoId)),
          rotation: MjEngineTool.UnityQuaternionFromMatrix(
              MjEngineTool.MjMatrixAtEntry(data->geom_xmat, MujocoId)));
      var comInGlobalFrame = _geomInGlobalFrame * _comTransform;
      comInGlobalFrame.StoreGlobal(transform);
    } else {
      transform.position = MjEngineTool.UnityVector3(
          MjEngineTool.MjVector3AtEntry(data->geom_xpos, MujocoId));
      transform.rotation = MjEngineTool.UnityQuaternionFromMatrix(
          MjEngineTool.MjMatrixAtEntry(data->geom_xmat, MujocoId));
    }
  }

  public void OnDrawGizmosSelected() {
    Gizmos.color = Color.blue;
    DrawGizmos(transform);
  }
}
}
