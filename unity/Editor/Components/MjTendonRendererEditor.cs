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

using UnityEditor;
using UnityEngine;

namespace Mujoco {

[CustomEditor(typeof(MjTendonRenderer))]
public class MjTendonRendererEditor : Editor {

  public override void OnInspectorGUI() {
    DrawDefaultInspector();
    var mjTendonRenderer = (MjTendonRenderer)target;
    EditorGUILayout.Space(10);

    if (GUILayout.Button("Add Line Renderers")) {
      ConfigureLineRenderers(mjTendonRenderer);
    }
    EditorGUILayout.Space(5);
    if (GUILayout.Button("Update Rendered Line Positions")) {
      UpdateRenderedTendons(mjTendonRenderer);
    }
    EditorGUILayout.Space(10);
  }

  private unsafe void ConfigureLineRenderers(MjTendonRenderer mjTendonRenderer) {
    if (mjTendonRenderer.SpatialTendons == null || mjTendonRenderer.SpatialTendons.Length == 0) {
      Debug.LogWarning("LineRendererSetup: No target GameObjects assigned in the list.", mjTendonRenderer);
      return;
    }

    Undo.SetCurrentGroupName("Configure Line Renderers");
    var group = Undo.GetCurrentGroup();

    foreach (var tendon in mjTendonRenderer.SpatialTendons) {
      var lr = tendon.GetComponent<LineRenderer>();

      if (lr != null) {
        if (!mjTendonRenderer.LineRendererDefaults.OverwriteExisting) {
          continue;
        }
        Undo.RecordObject(lr, "Configure Existing Line Renderer");
        ApplySettings(lr, tendon, mjTendonRenderer.LineRendererDefaults);
      } else {
        lr = Undo.AddComponent<LineRenderer>(tendon.gameObject);
        ApplySettings(lr, tendon, mjTendonRenderer.LineRendererDefaults);
      }
    }
    UpdateRenderedTendons(mjTendonRenderer);
    Undo.CollapseUndoOperations(group);
  }

  private unsafe void UpdateRenderedTendons(MjTendonRenderer mjTendonRenderer) {
    Undo.IncrementCurrentGroup();
    Undo.SetCurrentGroupName("Update Rendered Tendons");
    var group = Undo.GetCurrentGroup();
    MjScene.Instance.CreateScene();
    var model = MjScene.Instance.Model;
    var data = MjScene.Instance.Data;
    MujocoLib.mj_forward(model, data);
    var renderedTendons = mjTendonRenderer.RenderedTendons;
    foreach ((var _, LineRenderer lr) in renderedTendons) {
      Undo.RecordObject(lr, "Update Tendon's Line Renderer");
    }
    mjTendonRenderer.UpdateTendons(this, new MjStepArgs(model, data), renderedTendons);
    DestroyImmediate(MjScene.Instance.gameObject);
    Undo.CollapseUndoOperations(group);
  }

  private void ApplySettings(LineRenderer lineRenderer, MjSpatialTendon tendon,
      MjTendonRenderer.TendonRenderingDefaults settings) {
    lineRenderer.material = settings.DefaultMaterial;
    lineRenderer.startColor = settings.DefaultColor;
    lineRenderer.endColor = settings.DefaultColor;
    lineRenderer.useWorldSpace = true;
    lineRenderer.numCapVertices = settings.NumVertices;
    lineRenderer.numCornerVertices = settings.NumVertices;
    lineRenderer.generateLightingData = settings.GenerateLightingData;
    if (settings.ScaleByWidth) {
      lineRenderer.widthMultiplier = tendon.Width * settings.WidthMultiplier;
    }
  }
}
}