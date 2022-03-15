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
  public class MjTendonConstraintTests {
    private MjFixedTendon _tendon1;
    private MjFixedTendon _tendon2;
    private MjTendonConstraint _constraint;
    private XmlDocument _doc;

    [SetUp]
    public void SetUp() {
      _tendon1 = new GameObject("tendon1").AddComponent<MjFixedTendon>();
      _tendon2 = new GameObject("tendon2").AddComponent<MjFixedTendon>();
      _constraint = new GameObject("tendon").AddComponent<MjTendonConstraint>();
      _doc = new XmlDocument();
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_constraint.gameObject);
      UnityEngine.Object.DestroyImmediate(_tendon1.gameObject);
      UnityEngine.Object.DestroyImmediate(_tendon2.gameObject);
    }

    [Test]
    public void ErrorThrownWhenTendon1Empty() {
      // This is an illegal MJCF, but the purpose of this test is to verify that if
      // the user didn't assign the tendon in the editor, an error will be thrown when play is hit.
      _doc.LoadXml("<tendon/>");
      _constraint.ParseMjcf(_doc.GetElementsByTagName("tendon")[0] as XmlElement);
      Assert.That(() => { _constraint.GenerateMjcf("name", _doc); }, Throws.Exception);
    }

    [Test]
    public void Tendon2Empty() {
      _doc.LoadXml("<tendon tendon1='tendon1'/>");
      _constraint.ParseMjcf(_doc.GetElementsByTagName("tendon")[0] as XmlElement);
      _constraint.GenerateMjcf("name", _doc);
    }

    [Test]
    public void ParseAllSettings() {
      _doc.LoadXml("<tendon tendon1='tendon1' tendon2='tendon2'/>");
      _constraint.ParseMjcf(_doc.GetElementsByTagName("tendon")[0] as XmlElement);
      Assert.That(_constraint.Tendon1, Is.EqualTo(_tendon1));
      Assert.That(_constraint.Tendon2, Is.EqualTo(_tendon2));
    }
  }
}
