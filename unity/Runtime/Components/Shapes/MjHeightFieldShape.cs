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
  public Terrain terrain;

  [Tooltip("The path, relative to Application.dataPath, where the heightmap data will be save in PNG format.")]
  public string heightMapExportPath;

  public string FullHeightMapPath => Path.GetFullPath(Path.Combine(Application.dataPath, heightMapExportPath));
  public int HeightMapWidth => terrain.terrainData.heightmapTexture.width;
  public int HeightMapHeight => terrain.terrainData.heightmapTexture.height;
  public Vector3 HeightMapScale => terrain.terrainData.heightmapScale;

  public void ToMjcf(XmlElement mjcf, Transform transform){
    ExportHeightMap();
    var scene = MjScene.Instance;
    var assetName = scene.GenerationContext.AddHeightFieldAsset(this);
    mjcf.SetAttribute("hfield", assetName);
  }

  public void FromMjcf(XmlElement mjcf){

  }

  public void ExportHeightMap(){
    RenderTexture.active = terrain.terrainData.heightmapTexture;
    Texture2D texture = new Texture2D(RenderTexture.active.width, RenderTexture.active.height);
    texture.ReadPixels(new Rect(0, 0, RenderTexture.active.width, RenderTexture.active.height), 0, 0);
    RenderTexture.active = null;
    File.WriteAllBytes(FullHeightMapPath, texture.EncodeToPNG());
  }


  public Vector4 GetChangeStamp(){
    return Vector4.one;
  }

  public Tuple<Vector3[], int[]> BuildMesh(){
    return Tuple.Create(new Vector3[]{}, new int[]{});
  }

  public void DebugDraw(Transform transform){
  }
}
}
