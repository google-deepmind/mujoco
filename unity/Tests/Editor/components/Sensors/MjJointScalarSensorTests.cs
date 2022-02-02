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
public class MjJointScalarSensorTests {

  private MjJointScalarSensor _sensor;
  private MjHingeJoint _joint;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("sensor").AddComponent<MjJointScalarSensor>();
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _sensor.Joint = _joint;

    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_sensor.gameObject);
    UnityEngine.Object.DestroyImmediate(_joint.gameObject);
  }

  private XmlElement Parse(string mjcfString, string tag) {
    _doc.LoadXml(mjcfString);
    var elementList = _doc.GetElementsByTagName(tag);
    return elementList[0] as XmlElement;
  }

  [TestCase(MjJointScalarSensor.AvailableSensors.JointPos, "jointpos")]
  [TestCase(MjJointScalarSensor.AvailableSensors.JointVel, "jointvel")]
  [TestCase(MjJointScalarSensor.AvailableSensors.JointLimitPos, "jointlimitpos")]
  [TestCase(MjJointScalarSensor.AvailableSensors.JointLimitVel, "jointlimitvel")]
  [TestCase(MjJointScalarSensor.AvailableSensors.JointLimitFrc, "jointlimitfrc")]
  public void GenerateSensorName(
      MjJointScalarSensor.AvailableSensors type, string expectedTag) {
    _sensor.SensorType = type;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<{expectedTag}"));
  }

  [TestCase(MjJointScalarSensor.AvailableSensors.JointPos)]
  [TestCase(MjJointScalarSensor.AvailableSensors.JointVel)]
  [TestCase(MjJointScalarSensor.AvailableSensors.JointLimitPos)]
  [TestCase(MjJointScalarSensor.AvailableSensors.JointLimitVel)]
  [TestCase(MjJointScalarSensor.AvailableSensors.JointLimitFrc)]
  public void GenerateJointReference(MjJointScalarSensor.AvailableSensors type) {
    var parentNode = _doc.CreateElement("parent");
    _doc.AppendChild(parentNode);
    _sensor.SensorType = type;
    parentNode.AppendChild(_joint.GenerateMjcf("test_joint", _doc));
    parentNode.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain("joint=\"test_joint\""));
  }

  [TestCase("jointpos", MjJointScalarSensor.AvailableSensors.JointPos)]
  [TestCase("jointvel", MjJointScalarSensor.AvailableSensors.JointVel)]
  [TestCase("jointlimitpos", MjJointScalarSensor.AvailableSensors.JointLimitPos)]
  [TestCase("jointlimitvel", MjJointScalarSensor.AvailableSensors.JointLimitVel)]
  [TestCase("jointlimitfrc", MjJointScalarSensor.AvailableSensors.JointLimitFrc)]
  public void ParsingSensorType(
      string tag, MjJointScalarSensor.AvailableSensors expectedType) {
    var mjcfString = $"<sensors><{tag} joint=\"joint\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.SensorType, Is.EqualTo(expectedType));
  }

  [TestCase("jointpos")]
  [TestCase("jointvel")]
  [TestCase("jointlimitpos")]
  [TestCase("jointlimitvel")]
  [TestCase("jointlimitfrc")]
  public void ParsingReferenceToExistingJoint(string tag) {
    var mjcfString = $"<sensors><{tag} joint=\"joint\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Joint, Is.EqualTo(_joint));
  }

  [TestCase("jointpos")]
  [TestCase("jointvel")]
  [TestCase("jointlimitpos")]
  [TestCase("jointlimitvel")]
  [TestCase("jointlimitfrc")]
  public void ParsingReferenceToMissingJointThrowsException(string tag) {
    var mjcfString = $"<sensors><{tag} joint=\"missing_joint\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"{tag}");
    Assert.That(() => { _sensor.ParseMjcf(mjcfElement); }, Throws.TypeOf<ArgumentException>());
  }
}
}
