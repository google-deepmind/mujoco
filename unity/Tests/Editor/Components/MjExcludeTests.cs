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
public class MjExcludeTests {
  private MjBody _body1;
  private MjBody _body2;
  private MjExclude _exclude;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _body1 = new GameObject("body1").AddComponent<MjBody>();
    _body2 = new GameObject("body2").AddComponent<MjBody>();
    _exclude = new GameObject("exclude").AddComponent<MjExclude>();
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_exclude.gameObject);
    UnityEngine.Object.DestroyImmediate(_body1.gameObject);
    UnityEngine.Object.DestroyImmediate(_body2.gameObject);
  }

  [Test]
  public void ParseAllSettings() {
    _doc.LoadXml(
        "<exclude body1='body1' body2='body2'/>");
    _exclude.ParseMjcf(_doc.GetElementsByTagName("exclude")[0] as XmlElement);
    Assert.That(_exclude.Body1, Is.EqualTo(_body1));
    Assert.That(_exclude.Body2, Is.EqualTo(_body2));
  }
}
}
