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
  public class MjFixedTendonTests {
    private MjHingeJoint _joint1;
    private MjHingeJoint _joint2;
    private MjFixedTendon _tendon;
    private XmlDocument _doc;

    [SetUp]
    public void SetUp() {
      _joint1 = new GameObject("joint1").AddComponent<MjHingeJoint>();
      _joint2 = new GameObject("joint2").AddComponent<MjHingeJoint>();
      _tendon = new GameObject("tendon").AddComponent<MjFixedTendon>();
      _doc = new XmlDocument();
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_tendon.gameObject);
      UnityEngine.Object.DestroyImmediate(_joint1.gameObject);
      UnityEngine.Object.DestroyImmediate(_joint2.gameObject);
      UnityEngine.Object.DestroyImmediate(MjScene.Instance);
    }

    [Test]
    public void ParseAllSettings() {
      _doc.LoadXml("<fixed range='3 4'>" + "<joint joint='joint1' coef='1'/>" +
                   "<joint joint='joint2' coef='2'/>" + "</fixed>");
      _tendon.ParseMjcf(_doc.GetElementsByTagName("fixed")[0] as XmlElement);
      Assert.That(_tendon.RangeLower, Is.EqualTo(3));
      Assert.That(_tendon.RangeUpper, Is.EqualTo(4));
      Assert.That(_tendon.JointList.Count, Is.EqualTo(2));
      Assert.That(_tendon.JointList[0].Joint, Is.EqualTo(_joint1));
      Assert.That(_tendon.JointList[0].Coefficient, Is.EqualTo(1.0f));
      Assert.That(_tendon.JointList[1].Joint, Is.EqualTo(_joint2));
      Assert.That(_tendon.JointList[1].Coefficient, Is.EqualTo(2.0f));
    }
  }
}
