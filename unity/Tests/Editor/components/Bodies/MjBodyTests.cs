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
using UnityEngine.TestTools.Utils;

namespace Mujoco {

[TestFixture]
public class MjBodyTransformTests {

  private GameObject _parent;
  private MjBody _body;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _parent = new GameObject("parent");
    _body = new GameObject("body", typeof(MjBody)).GetComponent<MjBody>();
    _body.transform.parent = _parent.transform;
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    GameObject.DestroyImmediate(_body.gameObject);
    GameObject.DestroyImmediate(_parent);
  }

  [Test]
  public void RelativePositionUsedInMjcf() {
    _parent.transform.position = new Vector3(1, 2, 3);
    _parent.transform.rotation = Quaternion.AngleAxis(45.0f, (new Vector3(1, 2, 3)).normalized);
    var element = _doc.AppendChild(_body.GenerateMjcf("name", _doc)) as XmlElement;
    Assert.That(_doc.OuterXml, Does.Contain(@"pos=""1 3 2"""));
    Assert.That(
        element.GetQuaternionAttribute("quat", Quaternion.identity),
        Is.EqualTo(new Quaternion(w:-0.9238795f, x:0.1022765f, y:0.3068294f, z:0.2045529f))
        .Using(new Vector4EqualityComparer(1e-4f)));
  }

  [Test]
  public void LocalPositionParsing() {
    var bodyElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("body"));
    bodyElement.SetAttribute("pos", "1 2 3");
    _body.ParseMjcf(bodyElement);
    Assert.That(_body.transform.localPosition, Is.EqualTo(new Vector3(1, 3, 2)));
  }

  [TestCase(0.866f, 0.5f, 0, 0, 0.5f, 0, 0, -0.866f)]
  [TestCase(0.866f, 0, 0.5f, 0, 0, 0, 0.5f, -0.866f)]
  [TestCase(0.866f, 0, 0, 0.5f, 0, 0.5f, 0, -0.866f)]
  [TestCase(0, 0.866f, 0.5f, 0, 0.866f, 0, 0.5f, 0)]
  [TestCase(0, 0.866f, 0, 0.5f, 0.866f, 0.5f, 0, 0)]
  [TestCase(0, 0, 0.866f, 0.5f, 0, 0.5f, 0.866f, 0)]
  public void BodyLocalRotationParsing(
      float ix, float iy, float iz, float iw, float ex, float ey, float ez, float ew) {
    var bodyElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("body"));
    bodyElement.SetAttribute("quat", $"{ix} {iy} {iz} {iw}");
    _body.ParseMjcf(bodyElement);
    var angle = Quaternion.Angle(_body.transform.localRotation, new Quaternion(ex, ey, ez, ew));
    Assert.That(angle, Is.EqualTo(0).Within(3));
  }

  [Test]
  public void ParsingDefaultRotation() {
    var bodyElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("body"));
    _body.ParseMjcf(bodyElement);
    Assert.That(_body.transform.localRotation, Is.EqualTo(Quaternion.identity));
  }
}
}
