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
public class MjSiteQuaternionSensorTests {

  private MjSiteQuaternionSensor _sensor;
  private MjSite _site;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("sensor", typeof(MjSiteQuaternionSensor))
        .GetComponent<MjSiteQuaternionSensor>();
    _site = new GameObject("site", typeof(MjSite)).GetComponent<MjSite>();
    _sensor.Site = _site;

    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_sensor.gameObject);
    UnityEngine.Object.DestroyImmediate(_site.gameObject);
  }

  private XmlElement Parse(string mjcfString, string tag) {
    _doc.LoadXml(mjcfString);
    var elementList = _doc.GetElementsByTagName(tag);
    return elementList[0] as XmlElement;
  }

  [Test]
  public void GenerateSensorName() {
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain("<framequat"));
  }

  [Test]
  public void GenerateSiteReference() {
    var parentNode = _doc.CreateElement("parent");
    _doc.AppendChild(parentNode);
    parentNode.AppendChild(_site.GenerateMjcf("test_site", _doc));
    parentNode.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain("objname=\"test_site\""));
  }

  [Test]
  public void FindingRenferencedSite() {
    var mjcfString = "<sensors><framequat objname=\"site\"/></sensors>";
    var mjcfElement = Parse(mjcfString, "framequat");
    _sensor.Site = null;
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Site, Is.EqualTo(_site));
  }

  [Test]
  public void ExceptionThrownWhenReferencedBodyNotFoundInScene() {
    var mjcfString = "<sensors><framequat objname=\"missing_site\"/></sensors>";
    var mjcfElement = Parse(mjcfString, "framequat");
    Assert.That(() => { _sensor.ParseMjcf(mjcfElement); }, Throws.TypeOf<ArgumentException>());
  }
}
}
