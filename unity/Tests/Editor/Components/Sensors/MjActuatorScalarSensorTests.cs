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
public class MjActuatorScalarSensorTests {

  private MjActuatorScalarSensor _sensor;
  private MjActuator _actuator;
  private MjHingeJoint _joint;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("sensor", typeof(MjActuatorScalarSensor))
        .GetComponent<MjActuatorScalarSensor>();
    _joint = new GameObject("joint", typeof(MjHingeJoint)).GetComponent<MjHingeJoint>();
    _actuator = new GameObject("actuator", typeof(MjActuator)).GetComponent<MjActuator>();
    _actuator.Joint = _joint;
    _sensor.Actuator = _actuator;

    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_sensor.gameObject);
    UnityEngine.Object.DestroyImmediate(_joint.gameObject);
    UnityEngine.Object.DestroyImmediate(_actuator.gameObject);
  }

  private XmlElement Parse(string mjcfString, string tag) {
    _doc.LoadXml(mjcfString);
    var elementList = _doc.GetElementsByTagName(tag);
    return elementList[0] as XmlElement;
  }

  [TestCase(MjActuatorScalarSensor.AvailableSensors.ActuatorPos, "actuatorpos")]
  [TestCase(MjActuatorScalarSensor.AvailableSensors.ActuatorVel, "actuatorvel")]
  [TestCase(MjActuatorScalarSensor.AvailableSensors.ActuatorFrc, "actuatorfrc")]
  public void GenerateSensorName(
    MjActuatorScalarSensor.AvailableSensors type, string expectedTag) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<{expectedTag}"));
  }

  [TestCase(MjActuatorScalarSensor.AvailableSensors.ActuatorPos)]
  [TestCase(MjActuatorScalarSensor.AvailableSensors.ActuatorVel)]
  [TestCase(MjActuatorScalarSensor.AvailableSensors.ActuatorFrc)]
  public void GenerateJointReference(MjActuatorScalarSensor.AvailableSensors type) {
    var parentNode = _doc.CreateElement("parent");
    _doc.AppendChild(parentNode);
    _sensor.SensorType = type;
    parentNode.AppendChild(_actuator.GenerateMjcf("test_actuator", _doc));
    parentNode.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain("actuator=\"test_actuator\""));
  }

  [TestCase("actuatorpos", MjActuatorScalarSensor.AvailableSensors.ActuatorPos)]
  [TestCase("actuatorvel", MjActuatorScalarSensor.AvailableSensors.ActuatorVel)]
  [TestCase("actuatorfrc", MjActuatorScalarSensor.AvailableSensors.ActuatorFrc)]
  public void ParsingSensorType(
      string tag, MjActuatorScalarSensor.AvailableSensors expectedType) {
    var mjcfString = $"<sensors><{tag} actuator=\"actuator\"/></sensors>";
    var mjcfElement = Parse(mjcfString, tag);
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.SensorType, Is.EqualTo(expectedType));
  }

  [TestCase("actuatorpos")]
  [TestCase("actuatorvel")]
  [TestCase("actuatorfrc")]
  public void ParsingReferenceToExistingActuator(string tag) {
    var mjcfString = $"<sensors><{tag} actuator=\"actuator\"/></sensors>";
    var mjcfElement = Parse(mjcfString, tag);
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Actuator, Is.EqualTo(_actuator));
  }

  [TestCase("actuatorpos")]
  [TestCase("actuatorvel")]
  [TestCase("actuatorfrc")]
  public void ParsingReferenceToMissingAcgtuatorThrowsException(string tag) {
    var mjcfString = $"<sensors><{tag} actuator=\"missing_actuator\"/></sensors>";
    var mjcfElement = Parse(mjcfString, tag);
    Assert.That(() => { _sensor.ParseMjcf(mjcfElement); }, Throws.TypeOf<ArgumentException>());
  }
}
}
