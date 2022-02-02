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
using UnityEngine.TestTools;

namespace Mujoco {

[TestFixture]
public class MjBaseJointTests {

  public class FakeJoint : MjBaseJoint {
    protected override void OnParseMjcf(XmlElement mjcf) {}

    protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
      return (XmlElement)doc.CreateElement("joint");
    }
  }

  private FakeJoint _joint;
  private MjBaseBody _parent;
  private MjBaseBody _grandParent;

  [Test]
  public void RetrievingBodiesConnectedByTheJoint() {
    _joint = new GameObject("joint").AddComponent<FakeJoint>();
    _parent = new GameObject("parent").AddComponent<MjBody>();
    _grandParent = new GameObject("grandParent").AddComponent<MjBody>();

    _joint.transform.parent = _parent.transform;
    _parent.transform.parent = _grandParent.transform;

    MjBaseBody grandParent, parent;
    _joint.GetConnectedBodies(out grandParent, out parent);
    Assert.That(grandParent, Is.EqualTo(_grandParent));
    Assert.That(parent, Is.EqualTo(_parent));

    GameObject.DestroyImmediate(_joint.gameObject);
    GameObject.DestroyImmediate(_parent.gameObject);
    GameObject.DestroyImmediate(_grandParent.gameObject);
    GameObject.DestroyImmediate(MjScene.Instance);
  }

  [Test]
  public void RetrievingBodiesConnectedToTheWorld() {
    _joint = new GameObject("joint").AddComponent<FakeJoint>();
    _parent = new GameObject("parent").AddComponent<MjBody>();

    _joint.transform.parent = _parent.transform;

    MjBaseBody grandParent, parent;
    _joint.GetConnectedBodies(out grandParent, out parent);
    Assert.That(grandParent, Is.Null);
    Assert.That(parent, Is.EqualTo(_parent));

    GameObject.DestroyImmediate(_joint.gameObject);
    GameObject.DestroyImmediate(_parent.gameObject);
    GameObject.DestroyImmediate(MjScene.Instance);
  }
}
}
