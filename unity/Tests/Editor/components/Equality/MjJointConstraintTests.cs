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
  public class MjJointConstraintTests {
    private MjHingeJoint _joint1;
    private MjHingeJoint _joint2;
    private MjJointConstraint _constraint;
    private XmlDocument _doc;

    [SetUp]
    public void SetUp() {
      _joint1 = new GameObject("joint1").AddComponent<MjHingeJoint>();
      _joint2 = new GameObject("joint2").AddComponent<MjHingeJoint>();
      _constraint = new GameObject("joint").AddComponent<MjJointConstraint>();
      _doc = new XmlDocument();
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_constraint.gameObject);
      UnityEngine.Object.DestroyImmediate(_joint1.gameObject);
      UnityEngine.Object.DestroyImmediate(_joint2.gameObject);
    }

    [Test]
    public void ErrorThrownWhenJoint1Empty() {
      // This is an illegal MJCF, but the purpose of this test is to verify that if
      // the user didn't assign the joint in the editor, an error will be thrown when play is hit.
      _doc.LoadXml("<joint/>");
      _constraint.ParseMjcf(_doc.GetElementsByTagName("joint")[0] as XmlElement);
      Assert.That(() => { _constraint.GenerateMjcf("name", _doc); }, Throws.Exception);
    }

    [Test]
    public void Joint2Empty() {
      _doc.LoadXml("<joint joint1='joint1'/>");
      _constraint.ParseMjcf(_doc.GetElementsByTagName("joint")[0] as XmlElement);
      _constraint.GenerateMjcf("name", _doc);
    }

    [Test]
    public void ParseAllSettings() {
      _doc.LoadXml("<joint joint1='joint1' joint2='joint2'/>");
      _constraint.ParseMjcf(_doc.GetElementsByTagName("joint")[0] as XmlElement);
      Assert.That(_constraint.Joint1, Is.EqualTo(_joint1));
      Assert.That(_constraint.Joint2, Is.EqualTo(_joint2));
    }
  }
}
