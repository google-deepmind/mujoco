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
using System.IO;
using System.Linq;
using System.Xml;
using UnityEngine;
// internal imports
namespace Mujoco {

[Serializable]
public class MjHeightFieldShape : IMjShape {
  [Tooltip("Terrain's heightmap should have a minimum value of zero (fully black).")]
  public Terrain Terrain;

  [Tooltip("The path, relative to Application.dataPath, where the heightmap " +
           "data will be saved/exported in PNG format. Leave blank if hfield data should " +
           "be set instead programmatically (faster).")]
  public string HeightMapExportPath;

  public bool ExportImage => !string.IsNullOrEmpty(HeightMapExportPath);

  public string FullHeightMapPath => Path.GetFullPath(Path.Combine(Application.dataPath,
      HeightMapExportPath));

  public int HeightMapWidth => Terrain.terrainData.heightmapTexture.width;
  public int HeightMapLength => Terrain.terrainData.heightmapTexture.height;
  public Vector3 HeightMapScale => Terrain.terrainData.heightmapScale;

  [Tooltip("At least this many frames will have to pass before the scene is rebuilt with an " +
           "updated heightmap. Leave as 0 to not update the hfield during simulation. " +
           "If nonzero, increasing this can improve performance.")]
  public int UpdateLimit;

  private int _updateCountdown;

  [HideInInspector]
  public float MinimumHeight { get; private set; }

  [HideInInspector]
  public float MaximumHeight { get; private set; }

  public int HeightFieldId { get; private set; }

  public unsafe void ToMjcf(XmlElement mjcf, Transform transform) {
    if (Terrain.transform.parent != transform)
      Debug.LogWarning(
          $"The terrain of heightfield {transform.name} needs to be parented to the Geom " +
          "for proper rendering.");
    else {
      if ((Terrain.transform.localPosition - new Vector3(
              -(HeightMapLength - 1) * HeightMapScale.x / 2f,
              Terrain.transform.localPosition.y,
              -(HeightMapWidth - 1) * HeightMapScale.z / 2)).magnitude > 0.001) {
        Debug.LogWarning($"Terrain of heightfield {transform.name} not aligned with geom. The " +
                         " terrain will be moved to accurately represent the simulated position.");
      }
      Terrain.transform.localPosition = new Vector3(-(HeightMapLength - 1) * HeightMapScale.x / 2,
          Terrain.transform.localPosition.y,
          -(HeightMapWidth - 1) * HeightMapScale.z / 2);
    }
    var scene = MjScene.Instance;
    var assetName = scene.GenerationContext.AddHeightFieldAsset(this);

    if (Application.isPlaying) {
      scene.postInitEvent += (unused_first, unused_second) =>
          HeightFieldId =
              MujocoLib.mj_name2id(scene.Model, (int)MujocoLib.mjtObj.mjOBJ_HFIELD, assetName);
    }

    if (UpdateLimit > 0) {
      _updateCountdown = UpdateLimit;
      if (UpdateLimit > 1) {
        scene.preUpdateEvent += (unused_first, unused_second) => CountdownUpdateCondition();
      }
      TerrainCallbacks.heightmapChanged += (unused_first, unused_second, unused_third) =>
          RebuildHeightField();
    }

    mjcf.SetAttribute("hfield", assetName);
    PrepareHeightMap();
  }

  public void FromMjcf(XmlElement mjcf) {
  }

  public void PrepareHeightMap() {
    RenderTexture.active = Terrain.terrainData.heightmapTexture;
    Texture2D texture = new Texture2D(RenderTexture.active.width, RenderTexture.active.height);
    texture.ReadPixels(new Rect(0, 0, RenderTexture.active.width, RenderTexture.active.height),
        0,
        0);
    MaximumHeight = texture.GetPixels().Select(c => c.r).Max() * HeightMapScale.y * 2;
    var minimumHeight = texture.GetPixels().Select(c => c.r).Min() * HeightMapScale.y * 2;

    RenderTexture.active = null;
    if (ExportImage) {
      if (minimumHeight > 0.0001)
        Debug.LogWarning("Due to assumptions in MuJoCo heightfields, terrains should have a " +
                         "minimum heightmap value of 0.");
      File.WriteAllBytes(FullHeightMapPath, texture.EncodeToPNG());
    } else if (Application.isPlaying) {
      MjScene.Instance.postInitEvent += (unused_first, unused_second) => UpdateHeightFieldData();
    }
  }

  public unsafe void UpdateHeightFieldData() {
    RenderTexture.active = Terrain.terrainData.heightmapTexture;
    Texture2D texture = new Texture2D(RenderTexture.active.width, RenderTexture.active.height);
    texture.ReadPixels(new Rect(0, 0, RenderTexture.active.width, RenderTexture.active.height),
        0,
        0);
    RenderTexture.active = null;

    float[] curData = texture.GetPixels(0, 0, texture.width, texture.height)
        .Select(c => c.r * 2).ToArray();
    int adr = MjScene.Instance.Model->hfield_adr[HeightFieldId];
    for (int i = 0; i < curData.Length; i++) {
      MjScene.Instance.Model->hfield_data[adr + i] = curData[i];
    }
  }

  public void CountdownUpdateCondition() {
    if (_updateCountdown < 1) return;
    _updateCountdown -= 1;
  }

  public void RebuildHeightField() {
    // The update rate limiting countdown goes from UpdateLimit + 1 to 1, since if the Update
    // limit is at 1, we can update on every frame and don't need a countdown.
    if (_updateCountdown > 1) return;
    if (!Application.isPlaying || !MjScene.InstanceExists) return;
    if (ExportImage) {
      // If we export an image, it needs to be read by the compiler so we might as well rebuild the scene.
      MjScene.Instance.SceneRecreationAtLateUpdateRequested = true;
    } else {
      UpdateHeightFieldData();
    }
    _updateCountdown = UpdateLimit + 1;
  }

  public Vector4 GetChangeStamp() {
    return Vector4.one;
  }

  public Tuple<Vector3[], int[]> BuildMesh() {
    return null;
  }

  public void DebugDraw(Transform transform) {}
}
}
