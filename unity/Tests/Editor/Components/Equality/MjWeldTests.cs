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
  public class MjWeldTests {
    private MjBody _body1;
    private MjBody _body2;
    private MjWeld _constraint;
    private XmlDocument _doc;

    [SetUp]
    public void SetUp() {
      _body1 = new GameObject("body1").AddComponent<MjBody>();
      _body2 = new GameObject("body2").AddComponent<MjBody>();
      _constraint = new GameObject("weld").AddComponent<MjWeld>();
      _doc = new XmlDocument();
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_constraint.gameObject);
      UnityEngine.Object.DestroyImmediate(_body1.gameObject);
      UnityEngine.Object.DestroyImmediate(_body2.gameObject);
    }

    [Test]
    public void ParseXML() {
      _doc.LoadXml("<weld body1='body1' body2='body2'/>");
      _constraint.ParseMjcf(_doc.GetElementsByTagName("weld")[0] as XmlElement);
      Assert.That(_constraint.Body1, Is.EqualTo(_body1));
      Assert.That(_constraint.Body2, Is.EqualTo(_body2));
    }

    [Test]
    public void GenerateXML() {
      _constraint.Body1 = _body1;
      _constraint.Body2 = _body2;
      var mjcf = _constraint.GenerateMjcf("name", _doc);
      Assert.That(mjcf.OuterXml, Does.Contain("<weld"));
      Assert.That(mjcf.OuterXml, Does.Contain("body1=\""));
      Assert.That(mjcf.OuterXml, Does.Contain("body2=\""));
    }

    [Test]
    public void ErrorThrownWhenBody1Empty() {
      // This is an illegal MJCF, but the purpose of this test is to verify that if
      // the user didn't assign the body in the editor, an error will be thrown when play is hit.
      _doc.LoadXml("<weld/>");
      _constraint.ParseMjcf(_doc.GetElementsByTagName("weld")[0] as XmlElement);
      Assert.That(() => { _constraint.GenerateMjcf("name", _doc); }, Throws.Exception);
    }

    // In this constraint both elements are required.
    [Test]
    public void ErrorThrownWhenBody2Empty() {
      _doc.LoadXml("<weld body1='body1'/>");
      _constraint.ParseMjcf(_doc.GetElementsByTagName("weld")[0] as XmlElement);
      Assert.That(() => { _constraint.GenerateMjcf("name", _doc); }, Throws.Exception);
    }
  }
}
