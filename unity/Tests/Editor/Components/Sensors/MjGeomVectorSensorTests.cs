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
public class MjGeomVectorSensorTests {

  private MjGeomVectorSensor _sensor;
  private MjGeom _geom;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("MjGeomVectorSensorTests_sensor")
        .AddComponent<MjGeomVectorSensor>();
    _geom = new GameObject("MjGeomVectorSensorTests_geom").AddComponent<MjGeom>();
    _sensor.Geom = _geom;

    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_sensor.gameObject);
    UnityEngine.Object.DestroyImmediate(_geom.gameObject);
  }

  private XmlElement Parse(string mjcfString, string tag) {
    _doc.LoadXml(mjcfString);
    var elementList = _doc.GetElementsByTagName(tag);
    return elementList[0] as XmlElement;
  }

  [TestCase(MjGeomVectorSensor.AvailableSensors.FramePos, "framepos")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameXAxis, "framexaxis")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameYAxis, "frameyaxis")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameZAxis, "framezaxis")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameLinVel, "framelinvel")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameAngVel, "frameangvel")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameLinAcc, "framelinacc")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameAngAcc, "frameangacc")]
  public void GenerateSensorName(
      MjGeomVectorSensor.AvailableSensors type, string expectedTag) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<{expectedTag}"));
  }

  [TestCase(MjGeomVectorSensor.AvailableSensors.FramePos, "objname")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameXAxis, "objname")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameYAxis, "objname")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameZAxis, "objname")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameLinVel, "objname")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameAngVel, "objname")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameLinAcc, "objname")]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameAngAcc, "objname")]
  public void GenerateGeomReference(
      MjGeomVectorSensor.AvailableSensors type, string expectedAttribute) {
    var parentNode = (XmlElement)_doc.CreateElement("parent");
    _doc.AppendChild(parentNode);
    _sensor.SensorType = type;
    parentNode.AppendChild(_geom.GenerateMjcf("test_geom", _doc));
    parentNode.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"{expectedAttribute}=\"test_geom\""));
  }

  [TestCase(MjGeomVectorSensor.AvailableSensors.FramePos)]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameXAxis)]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameYAxis)]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameZAxis)]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameLinVel)]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameAngVel)]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameLinAcc)]
  [TestCase(MjGeomVectorSensor.AvailableSensors.FrameAngAcc)]
  public void FrameTypeSpecification(MjGeomVectorSensor.AvailableSensors type) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Not.Contain($"objname=\"MjGeomVectorSensorTests_geom\""));
  }

  [TestCase("framepos", MjGeomVectorSensor.AvailableSensors.FramePos)]
  [TestCase("framexaxis", MjGeomVectorSensor.AvailableSensors.FrameXAxis)]
  [TestCase("frameyaxis", MjGeomVectorSensor.AvailableSensors.FrameYAxis)]
  [TestCase("framezaxis", MjGeomVectorSensor.AvailableSensors.FrameZAxis)]
  [TestCase("framelinvel", MjGeomVectorSensor.AvailableSensors.FrameLinVel)]
  [TestCase("frameangvel", MjGeomVectorSensor.AvailableSensors.FrameAngVel)]
  [TestCase("framelinacc", MjGeomVectorSensor.AvailableSensors.FrameLinAcc)]
  [TestCase("frameangacc", MjGeomVectorSensor.AvailableSensors.FrameAngAcc)]
  public void ParsingSensorType(
      string tag, MjGeomVectorSensor.AvailableSensors expectedType) {
    var mjcfString =
      $"<sensors><{tag} geom=\"MjGeomVectorSensorTests_geom\" " +
      "objname=\"MjGeomVectorSensorTests_geom\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.SensorType, Is.EqualTo(expectedType));
  }

  [TestCase("framepos", "objname")]
  [TestCase("framexaxis", "objname")]
  [TestCase("frameyaxis", "objname")]
  [TestCase("framezaxis", "objname")]
  [TestCase("framelinvel", "objname")]
  [TestCase("frameangvel", "objname")]
  [TestCase("framelinacc", "objname")]
  [TestCase("frameangacc", "objname")]
  public void ParsingFrameGeomName(string tag, string siteAttribute) {
    var mjcfString =
        $"<sensors><{tag} {siteAttribute}=\"MjGeomVectorSensorTests_geom\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Geom, Is.EqualTo(_geom));
  }
}
}
