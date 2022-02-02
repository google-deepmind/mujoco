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
  public class MjPlaneShapeTests {
    private GameObject _owner;
    private MjShapeComponent _shape;
    private XmlDocument _doc;

    [SetUp]
    public void SetUp() {
      _owner = new GameObject("owner");
      _shape = _owner.AddComponent<MjSite>();
      _shape.ShapeType = MjShapeComponent.ShapeTypes.Plane;
      _doc = new XmlDocument();
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_owner);
    }

    [TestCase(1, 2, "1 2 1")]
    public void PlaneSize(float x, float y, string expectedResult) {
      var mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("test"));
      _shape.Plane.Extents = new Vector2(x, y);
      _shape.ShapeToMjcf(mjcf, _owner.transform);
      Assert.That(_doc.OuterXml, Does.Contain("size=\"" + expectedResult + "\""));
    }

    [TestCase(0.5f, "0.5 0.5 0.5")]
    [TestCase(2.0f, "2 2 2")]
    public void EffectOfUniformScaling(float uniformScale, string expectedResult) {
      var mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("test"));
      _owner.transform.localScale = Vector3.one * uniformScale;
      _shape.ShapeToMjcf(mjcf, _owner.transform);
      Assert.That(_doc.OuterXml, Does.Contain("size=\"" + expectedResult + "\""));
    }

    [TestCase(2, 3, 4, "2 3 4")]
    public void EffectOfNonUniformScaling(float x, float y, float z, string expectedResult) {
      var mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("test"));
      _owner.transform.localScale = new Vector3(x, y, z);
      _shape.ShapeToMjcf(mjcf, _owner.transform);
      Assert.That(_doc.OuterXml, Does.Contain("size=\"" + expectedResult + "\""));
    }

    [Test]
    public void ParsingSettings() {
      var mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("test"));
      mjcf.SetAttribute("type", "plane");
      mjcf.SetAttribute("size", "0.1 0.2 0.3");
      _shape.ShapeFromMjcf(mjcf);
      Assert.That(_shape.ShapeType, Is.EqualTo(MjShapeComponent.ShapeTypes.Plane));
      Assert.That(_shape.Plane.Extents, Is.EqualTo(new Vector2(0.1f, 0.2f)));
    }
  }
}
