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
public class MjSiteScalarSensorTests {

  private MjSiteScalarSensor _sensor;
  private MjSite _site;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("sensor", typeof(MjSiteScalarSensor))
        .GetComponent<MjSiteScalarSensor>();
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

  [TestCase(MjSiteScalarSensor.AvailableSensors.Touch, "touch")]
  [TestCase(MjSiteScalarSensor.AvailableSensors.RangeFinder, "rangefinder")]
  public void GenerateSensorName(
      MjSiteScalarSensor.AvailableSensors type, string expectedTag) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<{expectedTag}"));
  }

  [TestCase(MjSiteScalarSensor.AvailableSensors.Touch)]
  [TestCase(MjSiteScalarSensor.AvailableSensors.RangeFinder)]
  public void GenerateSiteReference(MjSiteScalarSensor.AvailableSensors type) {
    var parentNode = (XmlElement)_doc.CreateElement("parent");
    _doc.AppendChild(parentNode);
    _sensor.SensorType = type;
    parentNode.AppendChild(_site.GenerateMjcf("test_site", _doc));
    parentNode.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"site=\"test_site\""));
  }

  [TestCase(MjSiteScalarSensor.AvailableSensors.Touch)]
  [TestCase(MjSiteScalarSensor.AvailableSensors.RangeFinder)]
  public void SensorsWithoutExplicitBindingTypeSpecification(
      MjSiteScalarSensor.AvailableSensors type) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Not.Contain($"objname"));
  }

  [TestCase("touch", MjSiteScalarSensor.AvailableSensors.Touch)]
  [TestCase("rangefinder", MjSiteScalarSensor.AvailableSensors.RangeFinder)]
  public void ParsingSensorType(
      string tag, MjSiteScalarSensor.AvailableSensors expectedType) {
    var mjcfString = $"<sensors><{tag} site=\"site\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.SensorType, Is.EqualTo(expectedType));
  }

  [TestCase("touch", "site")]
  [TestCase("rangefinder", "site")]
  public void ParsingSubtreeSiteName(string tag, string siteAttribute) {
    var mjcfString = $"<sensors><{tag} {siteAttribute}=\"site\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Site, Is.EqualTo(_site));
  }
}
}
