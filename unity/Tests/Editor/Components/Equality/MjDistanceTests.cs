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
  public class MjDistanceTests {
    private MjGeom _geom1;
    private MjGeom _geom2;
    private MjDistance _distance;
    private XmlDocument _doc;

    [SetUp]
    public void SetUp() {
      _geom1 = new GameObject("geom1").AddComponent<MjGeom>();
      _geom2 = new GameObject("geom2").AddComponent<MjGeom>();
      _distance = new GameObject("distance").AddComponent<MjDistance>();
      _doc = new XmlDocument();
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_distance.gameObject);
      UnityEngine.Object.DestroyImmediate(_geom1.gameObject);
      UnityEngine.Object.DestroyImmediate(_geom2.gameObject);
    }

    [Test]
    public void ErrorThrownWhenGeom1Empty() {
      // This is an illegal MJCF, but the purpose of this test is to verify that if
      // the user didn't assign the geom in the editor, an error will be thrown when play is hit.
      _doc.LoadXml("<geom/>");
      _distance.ParseMjcf(_doc.GetElementsByTagName("geom")[0] as XmlElement);
      Assert.That(() => { _distance.GenerateMjcf("name", _doc); }, Throws.Exception);
    }

    // In this constraint both elements are required.
    [Test]
    public void ErrorThrownWhenGeom2Empty() {
      _doc.LoadXml("<geom geom1='geom1'/>");
      _distance.ParseMjcf(_doc.GetElementsByTagName("geom")[0] as XmlElement);
      Assert.That(() => { _distance.GenerateMjcf("name", _doc); }, Throws.Exception);
    }

    [Test]
    public void ParseAndGenerate() {
      _doc.LoadXml("<distance geom1='geom1' geom2='geom2'/>");
      _distance.ParseMjcf(_doc.GetElementsByTagName("distance")[0] as XmlElement);
      Assert.That(_distance.Geom1, Is.EqualTo(_geom1));
      Assert.That(_distance.Geom2, Is.EqualTo(_geom2));
      var mjcf = _distance.GenerateMjcf("name", _doc);
      Assert.That(mjcf.OuterXml, Does.Contain("<distance"));
      Assert.That(mjcf.OuterXml, Does.Contain("geom1=\""));
      Assert.That(mjcf.OuterXml, Does.Contain("geom2=\""));
    }
  }
}
