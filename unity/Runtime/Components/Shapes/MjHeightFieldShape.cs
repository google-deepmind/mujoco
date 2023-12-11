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

namespace Mujoco
{

[Serializable]
public class MjHeightFieldShape : IMjShape
{
  [Tooltip("Terrain's heightmap should have a minimum value of zero (fully black).")]
  public Terrain terrain;

  [Tooltip("The path, relative to Application.dataPath, where the heightmap data will be save in PNG format.")]
  public string heightMapExportPath;

  public string FullHeightMapPath => Path.GetFullPath(Path.Combine(Application.dataPath, heightMapExportPath));
  public int HeightMapWidth => terrain.terrainData.heightmapTexture.width;
  public int HeightMapLength => terrain.terrainData.heightmapTexture.height;
  public Vector3 HeightMapScale => terrain.terrainData.heightmapScale;

  [Tooltip("At least this many frames will have to pass before the scene is rebuilt with an updated heightmap. Leave as 0 to not update the hfield during simulation. Increasing this can improve performance.")]
  public int UpdateLimit;
  private int updateCountdown;

  [HideInInspector]
  public float MinimumHeight { get; private set; }
  [HideInInspector]
  public float MaximumHeight { get; private set; }

  public int HeightFieldId { get; private set; }

  public unsafe void ToMjcf(XmlElement mjcf, Transform transform) {
    ExportHeightMap();
    if(terrain.transform.parent != transform) Debug.LogWarning($"The terrain of heightfield {transform.name} needs to be parented to the Geom for proper rendering.");
    else {
      if((terrain.transform.localPosition - new Vector3(-HeightMapLength * HeightMapScale.x / 2, terrain.transform.localPosition.y, -HeightMapWidth * HeightMapScale.z / 2)).magnitude > 0.001) {
                    Debug.LogWarning($"Terrain of heightfield {transform.name} not aligned with geom. The terrain will be moved to accurately represent the simulated position.");
      }
      terrain.transform.localPosition = new Vector3(-HeightMapLength*HeightMapScale.x / 2, terrain.transform.localPosition.y, -HeightMapWidth*HeightMapScale.z / 2);
    }
    var scene = MjScene.Instance;
    var assetName = scene.GenerationContext.AddHeightFieldAsset(this);

    scene.postInitEvent += (_,_) => HeightFieldId = MujocoLib.mj_name2id(scene.Model, (int)MujocoLib.mjtObj.mjOBJ_HFIELD, assetName);


    if(UpdateLimit>0){
      updateCountdown = UpdateLimit;
      scene.preUpdateEvent += (_, _) => CountdownUpdateCondition();
      TerrainCallbacks.heightmapChanged += RebuildScene;
    }

    mjcf.SetAttribute("hfield", assetName);
  }

  public void FromMjcf(XmlElement mjcf) {

  }

  public void ExportHeightMap() {
    RenderTexture.active = terrain.terrainData.heightmapTexture;
    Texture2D texture = new Texture2D(RenderTexture.active.width, RenderTexture.active.height);
    texture.ReadPixels(new Rect(0, 0, RenderTexture.active.width, RenderTexture.active.height), 0, 0);
    MaximumHeight = texture.GetPixels().Select(c => c.r).Max()*HeightMapScale.y*2;
    var minimumHeight = texture.GetPixels().Select(c => c.r).Min()*HeightMapScale.y*2;
    if (minimumHeight > 0.0001) Debug.LogWarning("Due to assumptions in MuJoCo heightfields, terrains should have a minimum heightmap value of 0.");
    RenderTexture.active = null;
    File.WriteAllBytes(FullHeightMapPath, texture.EncodeToPNG());
  }

  public void CountdownUpdateCondition() {
    if(updateCountdown < 1) return;
    updateCountdown -= 1;
  }

  public void RebuildScene(Terrain terrain, RectInt heightRegion, bool synched) {
    if(updateCountdown > 0) return;
    if(!Application.isPlaying || !MjScene.InstanceExists) return;
    MjScene.Instance.SceneRecreationAtLateUpdateRequested = true;
    updateCountdown = UpdateLimit;
  }

  public Vector4 GetChangeStamp() {
    return Vector4.one;
  }

  public Tuple<Vector3[], int[]> BuildMesh() {
    return null;
  }

  public void DebugDraw(Transform transform) {
  }
}
}
