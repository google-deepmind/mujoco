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
using System.Xml;
using NUnit.Framework;
using UnityEngine;

namespace Mujoco {

[TestFixture]
public class MjcfGenerationContextTests {

  [Test]
  public void ComponentAssignedMujocoName() {
    var mjcf = _scene.CreateScene(skipCompile:true);
    Assert.That(mjcf.OuterXml, Does.Contain(
        @"name=""component_0"""));
  }

  [Test]
  public void TwoComponentsWithSameNameReceiveDistinctMujocoNames() {
    var mjcf = _scene.CreateScene(skipCompile:true);
    Assert.That(mjcf.OuterXml, Does.Contain(
        @"name=""component_0"""));
    Assert.That(mjcf.OuterXml, Does.Contain(
        @"name=""component_1"""));
  }

  [Test]
  public void GravityVectorIsConvertedToMjWorldReferenceFrame() {
    Physics.gravity = new Vector3(1, 2, 3);
    var mjcf = _scene.CreateScene(skipCompile:true);
    Assert.That(mjcf.OuterXml, Does.Contain("gravity=\"1 3 2\""));
  }

  [Test]
  public void ConfigurationMjcfSpecifiesTimeout() {
    var mjcf = _scene.CreateScene(skipCompile:true);
    Assert.That(mjcf.OuterXml, Does.Contain($"timestep=\"{Time.fixedDeltaTime}\""));
  }

  [Test]
  public void ConfigurationMjcfSpecifiesCoordinateSystemAsLocal() {
    var mjcf = _scene.CreateScene(skipCompile:true);
    Assert.That(mjcf.OuterXml, Does.Contain("coordinate=\"local\""));
  }

  [Test]
  public void ConfigurationMjcIncludesOptions() {
    var mjcf = _scene.CreateScene(skipCompile:true);
    Assert.That(mjcf.OuterXml, Does.Contain("impratio"));
    Assert.That(mjcf.OuterXml, Does.Contain("magnetic"));
    Assert.That(mjcf.OuterXml, Does.Contain("wind"));
    Assert.That(mjcf.OuterXml, Does.Contain("density"));
    Assert.That(mjcf.OuterXml, Does.Contain("viscosity"));
    Assert.That(mjcf.OuterXml, Does.Contain("o_margin"));
    Assert.That(mjcf.OuterXml, Does.Contain("o_solref"));
    Assert.That(mjcf.OuterXml, Does.Contain("o_solimp"));
    Assert.That(mjcf.OuterXml, Does.Contain("iterations"));
    Assert.That(mjcf.OuterXml, Does.Contain("tolerance"));
    Assert.That(mjcf.OuterXml, Does.Contain("noslip_iterations"));
    Assert.That(mjcf.OuterXml, Does.Contain("noslip_tolerance"));
    Assert.That(mjcf.OuterXml, Does.Contain("ccd_iterations"));
    Assert.That(mjcf.OuterXml, Does.Contain("ccd_tolerance"));
  }

  [Test]
  public void AssigningGeneratedMeshAssetsUniqueNames() {
    var context = new MjcfGenerationContext();
    var uniqueAssetName = context.AddMeshAsset(mesh: new Mesh());
    var element = new XmlDocument().CreateElement("test");
    context.GenerateMjcf(element);
    var assetNodes = element.SelectNodes($"asset/mesh[@name='{uniqueAssetName}']");
    Assert.That(assetNodes, Has.Count.EqualTo(1));
  }

  [Test]
  public void MeshMjcfIgnoresTriangleConnectivityAndStoredOnlyTheVertexCloud() {
    var mesh = new Mesh();
    mesh.vertices = new Vector3[] {
        new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1) };
    mesh.triangles = new int[] { 0, 1, 2, 2, 1, 0, 1, 0, 2 };
    var context = new MjcfGenerationContext();
    context.AddMeshAsset(mesh);
    var element = new XmlDocument().CreateElement("test");
    context.GenerateMjcf(element);
    Assert.That(element.OuterXml, Does.Contain("vertex=\"1 0 0 0 0 1 0 1 0 \""));
  }

  #region Test setup.

  private MjBody _componentA;
  private MjBody _componentB;
  private Vector3 _originalGravity;
  private MjScene _scene;
  private MjGlobalSettings _settings;

  [SetUp]
  public void SetUp() {
    _componentA = new GameObject("component").AddComponent<MjBody>();
    _componentB = new GameObject("component").AddComponent<MjBody>();
    _scene = MjScene.Instance;
    _originalGravity = Physics.gravity;
    _settings = new GameObject("settings").AddComponent<MjGlobalSettings>();
  }

  [TearDown]
  public void TearDown() {
    Physics.gravity = _originalGravity;
    UnityEngine.Object.DestroyImmediate(_componentA.gameObject);
    UnityEngine.Object.DestroyImmediate(_componentB.gameObject);
    UnityEngine.Object.DestroyImmediate(_settings.gameObject);
    UnityEngine.Object.DestroyImmediate(_scene.gameObject);
  }

  #endregion
}
}
