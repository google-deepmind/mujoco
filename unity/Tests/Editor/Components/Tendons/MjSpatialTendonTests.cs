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
  public class MjSpatialTendonTests {
    private MjSite _site1;
    private MjSite _site2;
    private MjSpatialTendon _tendon;
    private XmlDocument _doc;

    [SetUp]
    public void SetUp() {
      _site1 = new GameObject("site1").AddComponent<MjSite>();
      _site2 = new GameObject("site2").AddComponent<MjSite>();
      _tendon = new GameObject("tendon").AddComponent<MjSpatialTendon>();
      _doc = new XmlDocument();
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_tendon.gameObject);
      UnityEngine.Object.DestroyImmediate(_site1.gameObject);
      UnityEngine.Object.DestroyImmediate(_site2.gameObject);
      UnityEngine.Object.DestroyImmediate(MjScene.Instance);
    }

    [Test]
    public void ParseAllSettings() {
      _doc.LoadXml("<spatial>" + "<site site='site1'/>" +
                   "<site site='site2'/>" + "</spatial>");
      _tendon.ParseMjcf(_doc.GetElementsByTagName("spatial")[0] as XmlElement);
      Assert.That(_tendon.SpringLengthLower, Is.EqualTo(-1.0f));
      Assert.That(_tendon.SpringLengthUpper, Is.EqualTo(-1.0f));
      Assert.That(_tendon.ViapointsList.Count, Is.EqualTo(2));
      Assert.That(_tendon.ViapointsList[0].Site, Is.EqualTo(_site1));
      Assert.That(_tendon.ViapointsList[1].Site, Is.EqualTo(_site2));
    }

    [Test]
    public void ParseSingleElementSpringLength() {
      _doc.LoadXml("<spatial springlength='0.5'>" + "<site site='site1'/>" +
                   "<site site='site2'/>" + "</spatial>");
      _tendon.ParseMjcf(_doc.GetElementsByTagName("spatial")[0] as XmlElement);
      Assert.That(_tendon.SpringLengthLower, Is.EqualTo(0.5f));
      Assert.That(_tendon.SpringLengthUpper, Is.EqualTo(0.5f));
    }

    [Test]
    public void ParseTwoElementSpringLength() {
      _doc.LoadXml("<spatial springlength='0.1 0.5'>" + "<site site='site1'/>" +
                   "<site site='site2'/>" + "</spatial>");
      _tendon.ParseMjcf(_doc.GetElementsByTagName("spatial")[0] as XmlElement);
      Assert.That(_tendon.SpringLengthLower, Is.EqualTo(0.1f));
      Assert.That(_tendon.SpringLengthUpper, Is.EqualTo(0.5f));
    }

    [Test]
    public void ParseInvalidSpringLengthThrows() {
      _doc.LoadXml("<spatial springlength='0.1 0.5 1.0'>" + "<site site='site1'/>" +
                   "<site site='site2'/>" + "</spatial>");
      var element = _doc.GetElementsByTagName("spatial")[0] as XmlElement;
      Assert.That(() => { _tendon.ParseMjcf(element); }, Throws.TypeOf<ArgumentException>());
    }

    [Test]
    public void GenerateMjcfWithSpringLength() {
      _tendon.ViapointsList.Add(new SpatialTendonEntry { Site = _site1 });
      _tendon.ViapointsList.Add(new SpatialTendonEntry { Site = _site2 });
      _tendon.SpringLengthLower = 0.1f;
      _tendon.SpringLengthUpper = 0.5f;
      var element = _tendon.GenerateMjcf("test_tendon", _doc);
      Assert.That(element.GetAttribute("springlength"), Is.EqualTo("0.1 0.5"));
    }

    [Test]
    public void GenerateMjcfWithInvalidSpringLengthThrows() {
      _tendon.ViapointsList.Add(new SpatialTendonEntry { Site = _site1 });
      _tendon.ViapointsList.Add(new SpatialTendonEntry { Site = _site2 });
      _tendon.SpringLengthLower = 0.5f;
      _tendon.SpringLengthUpper = 0.1f;
      Assert.That(() => { _tendon.GenerateMjcf("test_tendon", _doc); }, Throws.TypeOf<ArgumentException>());
    }
  }
}
