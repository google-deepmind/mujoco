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
public class MjInertialTests {
  private MjInertial _inertial;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _inertial = new GameObject("inertial").AddComponent<MjInertial>();
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_inertial.gameObject);
  }

  [Test]
  public void ParseAllSettings() {
    _doc.LoadXml("<inertial mass='5' diaginertia='1 2 3'/>");
    _inertial.ParseMjcf(_doc.GetElementsByTagName("inertial")[0] as XmlElement);
    Assert.That(_inertial.Mass, Is.EqualTo(5.0f));
    Assert.That(_inertial.DiagInertia, Is.EqualTo(new Vector3(1, 3, 2)));
  }

  [Test]
  public void GenerateMjcf() {
    _inertial.Mass = 6.0f;
    _inertial.DiagInertia = new Vector3(7, 8, 9);
    var mjcf = _inertial.GenerateMjcf("inertial", _doc);
    Assert.That(mjcf.OuterXml, Does.Contain("mass=\"6\""));
    Assert.That(mjcf.OuterXml, Does.Contain("diaginertia=\"7 9 8\""));
  }
}
}
