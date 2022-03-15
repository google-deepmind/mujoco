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
public class MjSlideJointTests {

  private GameObject _parent;
  private MjSlideJoint _joint;
  private XmlDocument _doc;
  private Vector3EqualityComparer _comparer;

  [SetUp]
  public void SetUp() {
    _parent = new GameObject("parent");
    _joint = new GameObject("joint", typeof(MjSlideJoint)).GetComponent<MjSlideJoint>();
    _joint.transform.parent = _parent.transform;
    _doc = new XmlDocument();
    _comparer = new Vector3EqualityComparer(1e-3f);
  }

  [TearDown]
  public void TearDown() {
    GameObject.DestroyImmediate(_parent);
  }

  [Test]
  public void ComponentWorldPositionConvertedToMjCoordinates() {
    _joint.transform.position = new Vector3(1, 2, 3);
    _doc.AppendChild(_joint.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain(@"pos=""1 3 2"""));
  }

  [Test]
  public void GenerateLimits() {
    _joint.RangeLower = -20;
    _joint.RangeUpper = 10;
    _doc.AppendChild(_joint.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain(@"range=""-20 10"""));
  }

  [Test]
  public void TestingAnOffsetAxis() {
    _joint.transform.rotation =
        Quaternion.AngleAxis(45.0f, Vector3.up);
    var element = _joint.GenerateMjcf("name", _doc);
    Assert.That(element.GetVector3Attribute("axis", Vector3.zero),
                Is.EqualTo(new Vector3(0.7071068f, -0.7071069f, 0)).Using(_comparer));
  }

  [Test]
  public void ParsingPosition() {
    var jointElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("joint"));
    jointElement.SetAttribute("pos", "1 2 3");
    _joint.ParseMjcf(jointElement);
    Assert.That(_joint.transform.localPosition, Is.EqualTo(new Vector3(1, 3, 2)));
  }

  [TestCase("1 0 0", 1, 0, 0)]
  [TestCase("0 1 0", 0, 0, 1)]
  [TestCase("0 0 1", 0, 1, 0)]
  public void ParsingTranslatesSlidexisIntoJointRotation(
      string setting, float rx, float ry, float rz) {
    var jointElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("joint"));
    jointElement.SetAttribute("axis", setting);
    _joint.ParseMjcf(jointElement);
    Assert.That(_joint.SlideAxis, Is.EqualTo(new Vector3(rx, ry, rz)).Using(_comparer));
  }

  [TestCase("0 10", 0, 10)]
  [TestCase("-10 0", -10, 0)]
  [TestCase("-10 10", -10, 10)]
  public void ParsingJointLimitRangeSettings(
      string setting, float expectedLow, float expectedHigh) {
    var jointElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("joint"));
    jointElement.SetAttribute("range", setting);
    _joint.ParseMjcf(jointElement);
    Assert.That(_joint.RangeLower, Is.EqualTo(expectedLow));
    Assert.That(_joint.RangeUpper, Is.EqualTo(expectedHigh));
  }
}
}
