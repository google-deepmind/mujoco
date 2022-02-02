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
public class MjBallJointTests {

  private GameObject _parent;
  private MjBallJoint _joint;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _parent = new GameObject("parent");
    _joint = new GameObject("joint").AddComponent<MjBallJoint>();
    _joint.transform.parent = _parent.transform;
    _doc = new XmlDocument();
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
    _joint.RangeUpper = 10;
    _doc.AppendChild(_joint.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain(@"range=""0 10"""));
  }

  [Test]
  public void ParsingPosition() {
    var jointElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("joint"));
    jointElement.SetAttribute("pos", "1 2 3");
    _joint.ParseMjcf(jointElement);
    Assert.That(_joint.transform.localPosition, Is.EqualTo(new Vector3(1, 3, 2)));
  }

  [Test]
  public void ParsingJointLimitRangeSettings() {
    var jointElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("joint"));
    jointElement.SetAttribute("range", "0 10");
    _joint.ParseMjcf(jointElement);
    Assert.That(_joint.RangeUpper, Is.EqualTo(10));
  }
}
}
