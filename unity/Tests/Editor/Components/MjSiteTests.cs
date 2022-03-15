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
public class MjSiteEditorTests {

  private MjSite _site;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _site = new GameObject("site", typeof(MjSite)).GetComponent<MjSite>();
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    GameObject.DestroyImmediate(_site.gameObject);
  }

  private XmlElement Parse(string mjcfString, string tag) {
    _doc.LoadXml(mjcfString);
    var elementList = _doc.GetElementsByTagName(tag);
    return elementList[0] as XmlElement;
  }

  [Test]
  public void GeneratingShapePropertiesMjcf() {
    _site.ShapeType = MjShapeComponent.ShapeTypes.Box;
    _site.Box.Extents = new Vector3(1, 2, 3);
    _doc.AppendChild(_site.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain("type=\"box\""));
    Assert.That(_doc.OuterXml, Does.Contain("size=\"1 3 2\""));
  }

  [Test]
  public void ParsingShapePropertiesMjcf() {
    var mjcfString = "<site type=\"box\" size=\"1 3 2\"/>";
    var mjcfElement = Parse(mjcfString, "site");
    _site.ParseMjcf(mjcfElement);
    Assert.That(_site.ShapeType, Is.EqualTo(MjShapeComponent.ShapeTypes.Box));
    Assert.That(_site.Box.Extents, Is.EqualTo(new Vector3(1, 2, 3)));
  }
}
}
