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
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Xml;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

namespace Mujoco {

[TestFixture]
public class MjMaterialImportTests {

  private GameObject _sceneRoot;
  private bool _removeLocal = false;
  private bool _removeLocalMujoco = false;
  private MjImporterWithAssets _importer;

  [SetUp]
  public void SetUp() {
    _importer = new MjImporterWithAssets();
    LogAssert.ignoreFailingMessages = true;
    // these two directories may exist from previous imports
    string[] assetsRoot = {"Assets"};
    var guids = AssetDatabase.FindAssets("Local", assetsRoot);
    if (guids.Length == 0) {
      _removeLocal = true;
      AssetDatabase.CreateFolder("Assets", "Local");
    }
    string[] assetsLocal = {"Assets/Local"};
    guids = AssetDatabase.FindAssets("MjImports", assetsLocal);
    if (guids.Length == 0) {
      _removeLocalMujoco = true;
    }
  }

  [TearDown]
  public void TearDown() {
    LogAssert.ignoreFailingMessages = false;
    if (_sceneRoot != null) {
      UnityEngine.Object.DestroyImmediate(_sceneRoot);
    }
    // remove main resources folder:
    // string[] mainTestResources = {"Assets/Local/Mujoco/MjMaterialImportTests/Resources"};
    // foreach (var asset in AssetDatabase.FindAssets("", mainTestResources)) {
    //   AssetDatabase.DeleteAsset(AssetDatabase.GUIDToAssetPath(asset));
    // }
    // // remove main resources folder:
    // AssetDatabase.DeleteAsset(mainTestResources[0]);
    // remove all test folders:
    string[] assetLocalMujoco = {"Assets/Local/MjImports"};
    foreach (var asset in AssetDatabase.FindAssets("MjMaterialImportTests", assetLocalMujoco)) {
      AssetDatabase.DeleteAsset(AssetDatabase.GUIDToAssetPath(asset));
    }
    if (_removeLocalMujoco) {
      AssetDatabase.DeleteAsset("Assets/Local/MjImports");
    }
    if (_removeLocal) {
      AssetDatabase.DeleteAsset("Assets/Local");
    }
  }

  [Test]
  public void ParsingMaterialProperties() {
    var mjcfString = @"<mujoco>
      <asset>
        <material name='test_material'
                  rgba='.7 .5 .3 .1' specular='.5' shininess='.7' reflectance='.2'/>
      </asset>
      <worldbody>
        <body>
          <geom name='test_geom' material='test_material' size='1 1 1'/>
        </body>
      </worldbody>
    </mujoco>";
    _sceneRoot = _importer.ImportString(
        name: "MjMaterialImportTests", mjcfString: mjcfString);
    var geom = _sceneRoot.GetComponentInChildren<MjGeom>();
    Assert.That(geom, Is.Not.Null);
    var meshRenderer = geom.GetComponent<MeshRenderer>();
    Assert.That(meshRenderer, Is.Not.Null);
    Assert.That(meshRenderer.sharedMaterial, Is.Not.Null);
    Assert.That(meshRenderer.sharedMaterial.color, Is.EqualTo(new Color(0.7f, 0.5f, 0.3f, 0.1f)));
    Assert.That(meshRenderer.sharedMaterial.GetFloat("_Metallic"), Is.EqualTo(0.2f));
    Assert.That(meshRenderer.sharedMaterial.GetFloat("_Glossiness"), Is.GreaterThan(0.0f));
  }

  [Test]
  public void ParsingMaterialWithoutEmissivity() {
    var mjcfString = @"<mujoco>
      <asset>
        <material name='test_material' rgba='.7 .5 .3 .1'/>
      </asset>
      <worldbody>
        <body>
          <geom name='test_geom' material='test_material' size='1 1 1'/>
        </body>
      </worldbody>
    </mujoco>";
    _sceneRoot = _importer.ImportString(
        name: "MjMaterialImportTests", mjcfString: mjcfString);
    var geom = _sceneRoot.GetComponentInChildren<MjGeom>();
    Assert.That(geom, Is.Not.Null);
    var meshRenderer = geom.GetComponent<MeshRenderer>();
    Assert.That(meshRenderer, Is.Not.Null);
    Assert.That(meshRenderer.sharedMaterial, Is.Not.Null);
    Assert.That(meshRenderer.sharedMaterial.shaderKeywords, Has.No.Member("_EMISSION"));
  }

  [Test]
  public void ParsingMaterialWithEmissivity() {
    var mjcfString = @"<mujoco>
      <asset>
        <material name='test_material' rgba='.7 .5 .3 .1' emission='1'/>
      </asset>
      <worldbody>
        <body>
          <geom name='test_geom' material='test_material' size='1 1 1'/>
        </body>
      </worldbody>
    </mujoco>";
    _sceneRoot = _importer.ImportString(
        name: "MjMaterialImportTests", mjcfString: mjcfString);
    var geom = _sceneRoot.GetComponentInChildren<MjGeom>();
    Assert.That(geom, Is.Not.Null);
    var meshRenderer = geom.GetComponent<MeshRenderer>();
    Assert.That(meshRenderer, Is.Not.Null);
    Assert.That(meshRenderer.sharedMaterial, Is.Not.Null);
    Assert.That(meshRenderer.sharedMaterial.shaderKeywords, Has.Member("_EMISSION"));
    Assert.That(
        meshRenderer.sharedMaterial.GetColor("_EmissionColor"),
        Is.EqualTo(new Color(0.7f, 0.5f, 0.3f, 0.1f)));
  }

  [Test]
  public void ResettingColorDatabaseForEachImport() {
    var mjcfString1 = @"<mujoco>
      <asset>
        <material name='test_material' rgba='.2 .2 .2 .2'/>
      </asset>
      <worldbody>
        <body>
          <geom name='test_geom' material='test_material' size='1 1 1'/>
        </body>
      </worldbody>
    </mujoco>";
    var mjcfString2 = @"<mujoco>
      <asset>
        <material name='test_material' rgba='.5 .5 .5 .5'/>
      </asset>
      <worldbody>
        <body>
          <geom name='test_geom' material='test_material' size='1 1 1'/>
        </body>
      </worldbody>
    </mujoco>";
    _sceneRoot = new GameObject();
    var importedMesh1 = _importer.ImportString(
        name: "MjMaterialImportTests"+$"{UnityEngine.Random.Range(0,999)}", mjcfString: mjcfString1);
    importedMesh1.transform.parent = _sceneRoot.transform;
    var importedMesh2 = _importer.ImportString(
        name: "MjMaterialImportTests"+$"{UnityEngine.Random.Range(0,999)}", mjcfString: mjcfString2);
    importedMesh2.transform.parent = _sceneRoot.transform;
    var geom = importedMesh2.GetComponentInChildren<MjGeom>();
    var meshRenderer = geom.GetComponent<MeshRenderer>();
    Assert.That(meshRenderer.sharedMaterial.color, Is.EqualTo(new Color(0.5f, 0.5f, 0.5f, 0.5f)));
  }
}
}
