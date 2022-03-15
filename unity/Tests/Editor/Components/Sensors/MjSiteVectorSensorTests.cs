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
public class MjSiteVectorSensorTests {

  private MjSiteVectorSensor _sensor;
  private MjSite _site;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("sensor").AddComponent<MjSiteVectorSensor>();
    _site = new GameObject("site").AddComponent<MjSite>();
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

  [TestCase(MjSiteVectorSensor.AvailableSensors.Accelerometer, "accelerometer")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Velocimeter, "velocimeter")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Gyro, "gyro")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Force, "force")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Torque, "torque")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Magnetometer, "magnetometer")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FramePos, "framepos")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameXAxis, "framexaxis")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameYAxis, "frameyaxis")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameZAxis, "framezaxis")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameLinVel, "framelinvel")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameAngVel, "frameangvel")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameLinAcc, "framelinacc")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameAngAcc, "frameangacc")]
  public void GenerateSensorName(
      MjSiteVectorSensor.AvailableSensors type, string expectedTag) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<{expectedTag}"));
  }

  [TestCase(MjSiteVectorSensor.AvailableSensors.Accelerometer, "site")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Velocimeter, "site")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Gyro, "site")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Force, "site")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Torque, "site")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Magnetometer, "site")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FramePos, "objname")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameXAxis, "objname")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameYAxis, "objname")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameZAxis, "objname")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameLinVel, "objname")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameAngVel, "objname")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameLinAcc, "objname")]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameAngAcc, "objname")]
  public void GenerateSiteReference(
      MjSiteVectorSensor.AvailableSensors type, string expectedAttribute) {
    var parentNode = (XmlElement)_doc.CreateElement("parent");
    _doc.AppendChild(parentNode);
    _sensor.SensorType = type;
    parentNode.AppendChild(_site.GenerateMjcf("test_site", _doc));
    parentNode.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"{expectedAttribute}=\"test_site\""));
  }

  [TestCase(MjSiteVectorSensor.AvailableSensors.Accelerometer)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Velocimeter)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Gyro)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Force)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Torque)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.Magnetometer)]
  public void SensorsWithoutExplicitBindingTypeSpecification(
      MjSiteVectorSensor.AvailableSensors type) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Not.Contain("objname"));
  }

  [TestCase(MjSiteVectorSensor.AvailableSensors.FramePos)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameXAxis)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameYAxis)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameZAxis)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameLinVel)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameAngVel)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameLinAcc)]
  [TestCase(MjSiteVectorSensor.AvailableSensors.FrameAngAcc)]
  public void FrameTypeSpecification(MjSiteVectorSensor.AvailableSensors type) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Not.Contain("objname=\"site\""));
  }

  [TestCase("accelerometer", MjSiteVectorSensor.AvailableSensors.Accelerometer)]
  [TestCase("velocimeter", MjSiteVectorSensor.AvailableSensors.Velocimeter)]
  [TestCase("gyro", MjSiteVectorSensor.AvailableSensors.Gyro)]
  [TestCase("force", MjSiteVectorSensor.AvailableSensors.Force)]
  [TestCase("torque", MjSiteVectorSensor.AvailableSensors.Torque)]
  [TestCase("magnetometer", MjSiteVectorSensor.AvailableSensors.Magnetometer)]
  [TestCase("framepos", MjSiteVectorSensor.AvailableSensors.FramePos)]
  [TestCase("framexaxis", MjSiteVectorSensor.AvailableSensors.FrameXAxis)]
  [TestCase("frameyaxis", MjSiteVectorSensor.AvailableSensors.FrameYAxis)]
  [TestCase("framezaxis", MjSiteVectorSensor.AvailableSensors.FrameZAxis)]
  [TestCase("framelinvel", MjSiteVectorSensor.AvailableSensors.FrameLinVel)]
  [TestCase("frameangvel", MjSiteVectorSensor.AvailableSensors.FrameAngVel)]
  [TestCase("framelinacc", MjSiteVectorSensor.AvailableSensors.FrameLinAcc)]
  [TestCase("frameangacc", MjSiteVectorSensor.AvailableSensors.FrameAngAcc)]
  public void ParsingSensorType(
      string tag, MjSiteVectorSensor.AvailableSensors expectedType) {
    var mjcfString = $"<sensors><{tag} site=\"site\" objname=\"site\"/></sensors>";
    var mjcfElement = Parse(mjcfString, tag);
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.SensorType, Is.EqualTo(expectedType));
  }

  [TestCase("accelerometer", "site")]
  [TestCase("velocimeter", "site")]
  [TestCase("gyro", "site")]
  [TestCase("force", "site")]
  [TestCase("torque", "site")]
  [TestCase("magnetometer", "site")]
  public void ParsingSubtreeSiteName(string tag, string siteAttribute) {
    var mjcfString = $"<sensors><{tag} {siteAttribute}=\"site\"/></sensors>";
    var mjcfElement = Parse(mjcfString, tag);
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Site, Is.EqualTo(_site));
  }

  [TestCase("framepos", "objname")]
  [TestCase("framexaxis", "objname")]
  [TestCase("frameyaxis", "objname")]
  [TestCase("framezaxis", "objname")]
  [TestCase("framelinvel", "objname")]
  [TestCase("frameangvel", "objname")]
  [TestCase("framelinacc", "objname")]
  [TestCase("frameangacc", "objname")]
  public void ParsingFrameSiteName(string tag, string siteAttribute) {
    var mjcfString = $"<sensors><{tag} {siteAttribute}=\"site\"/></sensors>";
    var mjcfElement = Parse(mjcfString, tag);
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Site, Is.EqualTo(_site));
  }
}
}
