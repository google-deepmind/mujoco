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
public class MjGeomQuaternionSensorTests {

  private MjGeomQuaternionSensor _sensor;
  private MjGeom _geom;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("MjGeomQuaternionSensorTests_sensor")
        .AddComponent<MjGeomQuaternionSensor>();
    _geom = new GameObject("MjGeomQuaternionSensorTests_geom").AddComponent<MjGeom>();
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

  public void GenerateSensorName() {
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain("<framequat"));
  }

  public void GenerateGeomReference() {
    var parentNode = (XmlElement)_doc.CreateElement("parent");
    _doc.AppendChild(parentNode);
    parentNode.AppendChild(_geom.GenerateMjcf("test_geom", _doc));
    parentNode.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(_doc.OuterXml, Does.Contain("objname=\"test_geom\""));
  }

  public void FrameTypeSpecification() {
    _doc.AppendChild(_sensor.GenerateMjcf("test_sensor", _doc));
    Assert.That(
        _doc.OuterXml, Does.Not.Contain("objname=\"MjGeomQuaternionSensorTests_geom\""));
  }

  [Test]
  public void FindTheReferencedGeom() {
    var mjcfString =
        "<sensors><framequat objname=\"MjGeomQuaternionSensorTests_geom\"/></sensors>";
    var mjcfElement = Parse(mjcfString, "framequat");
    _sensor.Geom = null;
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Geom, Is.EqualTo(_geom));
  }

  [Test]
  public void ExceptionThrownWhenReferencedBodyNotFoundInScene() {
    var mjcfString = "<sensors><framequat objname=\"missing_geom\"/></sensors>";
    var mjcfElement = Parse(mjcfString, "framequat");
    Assert.That(() => { _sensor.ParseMjcf(mjcfElement); }, Throws.TypeOf<ArgumentException>());
  }
}
}
