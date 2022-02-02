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
public class MjBodyVectorSensorTests {

  private MjBodyVectorSensor _sensor;
  private MjBody _body;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("sensor", typeof(MjBodyVectorSensor))
        .GetComponent<MjBodyVectorSensor>();
    _body = new GameObject("body", typeof(MjBody)).GetComponent<MjBody>();
    _sensor.Body = _body;

    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_sensor.gameObject);
    UnityEngine.Object.DestroyImmediate(_body.gameObject);
  }

  private XmlElement Parse(string mjcfString, string tag) {
    _doc.LoadXml(mjcfString);
    var elementList = _doc.GetElementsByTagName(tag);
    return elementList[0] as XmlElement;
  }

  [TestCase(MjBodyVectorSensor.AvailableSensors.SubtreeCom, "subtreecom")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.SubtreeLinVel, "subtreelinvel")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.SubtreeAngMom, "subtreeangmom")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FramePos, "framepos")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameXAxis, "framexaxis")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameYAxis, "frameyaxis")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameZAxis, "framezaxis")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameLinVel, "framelinvel")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameAngVel, "frameangvel")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameLinAcc, "framelinacc")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameAngAcc, "frameangacc")]
  public void GenerateSensorName(
      MjBodyVectorSensor.AvailableSensors type, string expectedTag) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<{expectedTag}"));
  }

  [TestCase(MjBodyVectorSensor.AvailableSensors.SubtreeCom, "body")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.SubtreeLinVel, "body")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.SubtreeAngMom, "body")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FramePos, "objname")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameXAxis, "objname")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameYAxis, "objname")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameZAxis, "objname")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameLinVel, "objname")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameAngVel, "objname")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameLinAcc, "objname")]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameAngAcc, "objname")]
  public void GenerateBodyReference(
      MjBodyVectorSensor.AvailableSensors type, string expectedAttribute) {
    var parentNode = (XmlElement)_doc.CreateElement("parent");
    _doc.AppendChild(parentNode);
    _sensor.SensorType = type;
    parentNode.AppendChild(_body.GenerateMjcf("test_body", _doc));
    parentNode.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"{expectedAttribute}=\"test_body\""));
  }

  [TestCase(MjBodyVectorSensor.AvailableSensors.SubtreeCom)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.SubtreeLinVel)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.SubtreeAngMom)]
  public void SensorsWithoutExplicitBindingTypeSpecification(
      MjBodyVectorSensor.AvailableSensors type) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Not.Contain($"objname"));
  }

  [TestCase(MjBodyVectorSensor.AvailableSensors.FramePos)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameXAxis)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameYAxis)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameZAxis)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameLinVel)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameAngVel)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameLinAcc)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameAngAcc)]
  public void InertialFrameSpecification(MjBodyVectorSensor.AvailableSensors type) {
    _sensor.SensorType = type;
    _sensor.UseInertialFrame = true;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Not.Contain($"objname=\"xbody\""));
  }

  [TestCase(MjBodyVectorSensor.AvailableSensors.FramePos)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameXAxis)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameYAxis)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameZAxis)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameLinVel)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameAngVel)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameLinAcc)]
  [TestCase(MjBodyVectorSensor.AvailableSensors.FrameAngAcc)]
  public void RegularFrameSpecification(MjBodyVectorSensor.AvailableSensors type) {
    _sensor.SensorType = type;
    _sensor.UseInertialFrame = false;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Not.Contain($"objname=\"body\""));
  }

  [TestCase("subtreecom", MjBodyVectorSensor.AvailableSensors.SubtreeCom)]
  [TestCase("subtreelinvel", MjBodyVectorSensor.AvailableSensors.SubtreeLinVel)]
  [TestCase("subtreeangmom", MjBodyVectorSensor.AvailableSensors.SubtreeAngMom)]
  [TestCase("framepos", MjBodyVectorSensor.AvailableSensors.FramePos)]
  [TestCase("framexaxis", MjBodyVectorSensor.AvailableSensors.FrameXAxis)]
  [TestCase("frameyaxis", MjBodyVectorSensor.AvailableSensors.FrameYAxis)]
  [TestCase("framezaxis", MjBodyVectorSensor.AvailableSensors.FrameZAxis)]
  [TestCase("framelinvel", MjBodyVectorSensor.AvailableSensors.FrameLinVel)]
  [TestCase("frameangvel", MjBodyVectorSensor.AvailableSensors.FrameAngVel)]
  [TestCase("framelinacc", MjBodyVectorSensor.AvailableSensors.FrameLinAcc)]
  [TestCase("frameangacc", MjBodyVectorSensor.AvailableSensors.FrameAngAcc)]
  public void ParsingSensorType(
      string tag, MjBodyVectorSensor.AvailableSensors expectedType) {
    var mjcfString = $"<sensors><{tag} body=\"body\" objname=\"body\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.SensorType, Is.EqualTo(expectedType));
  }

  [TestCase("subtreecom", "body")]
  [TestCase("subtreelinvel", "body")]
  [TestCase("subtreeangmom", "body")]
  public void ParsingSubtreeBodyName(string tag, string bodyAttribute) {
    var mjcfString = $"<sensors><{tag} {bodyAttribute}=\"body\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Body, Is.EqualTo(_body));
  }

  [TestCase("framepos", "objname")]
  [TestCase("framexaxis", "objname")]
  [TestCase("frameyaxis", "objname")]
  [TestCase("framezaxis", "objname")]
  [TestCase("framelinvel", "objname")]
  [TestCase("frameangvel", "objname")]
  [TestCase("framelinacc", "objname")]
  [TestCase("frameangacc", "objname")]
  public void ParsingFrameBodyName(string tag, string bodyAttribute) {
    var mjcfString = $"<sensors><{tag} {bodyAttribute}=\"body\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Body, Is.EqualTo(_body));
  }
}
}
