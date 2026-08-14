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
public class MjBaseSensorTests {

  public class MjMockSensor : MjBaseSensor {
    protected override XmlElement ToMjcf(XmlDocument doc) {
      return doc.CreateElement("sensor");
    }
    protected override void FromMjcf(XmlElement mjcf) {}
  }

  private MjMockSensor _sensor;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _sensor = new GameObject("sensor", typeof(MjMockSensor)).GetComponent<MjMockSensor>();
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_sensor.gameObject);
  }

  private XmlElement Parse(string mjcfString, string tag) {
    _doc.LoadXml(mjcfString);
    var elementList = _doc.GetElementsByTagName(tag);
    return elementList[0] as XmlElement;
  }

  [Test]
  public void GeneratingNoiseAndCutoffMjcf() {
    _sensor.Cutoff = 3.0f;
    _doc.AppendChild(_sensor.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain("cutoff=\"3\""));
  }

  [Test]
  public void ParsingShapePropertiesMjcf() {
    var mjcfString = "<sensor cutoff=\"4\" actuator=\"actuator\"/>";
    var mjcfElement = Parse(mjcfString, "sensor");
    _sensor.ParseMjcf(mjcfElement);
    Assert.That(_sensor.Cutoff, Is.EqualTo(4.0f));
  }
}
}
