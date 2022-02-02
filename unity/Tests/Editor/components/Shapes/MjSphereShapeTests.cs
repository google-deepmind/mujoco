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
  public class MjSphereShapeTests {
    private GameObject _owner;
    private MjShapeComponent _shape;
    private XmlDocument _doc;

    [SetUp]
    public void SetUp() {
      _owner = new GameObject("owner");
      _shape = _owner.AddComponent<MjSite>();
      _shape.ShapeType = MjShapeComponent.ShapeTypes.Sphere;
      _shape.Sphere.Radius = 1.0f;
      _doc = new XmlDocument();
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_owner);
    }

    [TestCase(2.0f, "2")]
    [TestCase(1.5f, "1.5")]
    public void XmlSizeForUnscaledSphere(float radius, string expectedResult) {
      var mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("test"));
      _shape.Sphere.Radius = radius;
      _shape.ShapeToMjcf(mjcf, _owner.transform);
      Assert.That(_doc.OuterXml, Does.Contain("size=\"" + expectedResult + "\""));
    }

    [TestCase(0.5f, "0.5")]
    [TestCase(2.0f, "2")]
    public void EffectOfUniformScaling(float uniformScale, string expectedResult) {
      var mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("test"));
      _owner.transform.localScale = Vector3.one * uniformScale;
      _shape.ShapeToMjcf(mjcf, _owner.transform);
      Assert.That(_doc.OuterXml, Does.Contain("size=\"" + expectedResult + "\""));
    }

    [Test]
    public void ParsingSettings() {
      var mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("test"));
      mjcf.SetAttribute("type", "sphere");
      mjcf.SetAttribute("size", "0.75");
      _shape.ShapeFromMjcf(mjcf);
      Assert.That(_shape.ShapeType, Is.EqualTo(MjShapeComponent.ShapeTypes.Sphere));
      Assert.That(_shape.Sphere.Radius, Is.EqualTo(0.75f));
    }
  }
}
