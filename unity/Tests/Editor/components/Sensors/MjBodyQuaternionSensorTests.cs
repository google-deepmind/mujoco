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
public class MjBodyQuaternionSensorTests {

  private MjBodyQuaternionSensor _sensor;
  private MjBody _body;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("sensor", typeof(MjBodyQuaternionSensor))
        .GetComponent<MjBodyQuaternionSensor>();
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

  [Test]
  public void GenerateSensorName() {
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<framequat"));
  }

  [Test]
  public void GenerateBodyReference() {
    var parentNode = (XmlElement)_doc.CreateElement("parent");
    _doc.AppendChild(parentNode);
    parentNode.AppendChild(_body.GenerateMjcf("test_body", _doc));
    parentNode.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"objname=\"test_body\""));
  }

  [TestCase(true, "xbody")]
  [TestCase(false, "body")]
  public void FrameTypeSpecification(bool useInertial, string expectedValue) {
    _sensor.UseInertialFrame = useInertial;
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Not.Contain($"objname=\"{expectedValue}\""));
  }

  [Test]
  public void ParsingFrameBodyName() {
    var mjcfString = $"<sensors><framequat objname=\"body\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"framequat");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Body, Is.EqualTo(_body));
  }

  [Test]
  public void ExceptionThrownWhenReferencedBodyNotFoundInScene() {
    var mjcfString = $"<sensors><framequat objname=\"missing_body\"/></sensors>";
    var mjcfElement = Parse(mjcfString, $"framequat");
    Assert.That(() => { _sensor.ParseMjcf(mjcfElement); }, Throws.TypeOf<ArgumentException>());
  }
}
}
