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

#if UNITY_EDITOR

using System;
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace Mujoco {

[TestFixture]
public class MjJointStateTests {
  private MjBody _body;
  private MjHingeJoint _joint;
  private MjInertial _inertial;
  private MjScene _scene;

  [SetUp]
  public void SetUp() {
    _scene = MjScene.Instance;
    _body = new GameObject("body").AddComponent<MjBody>();
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _joint.transform.parent = _body.transform;
    _inertial = new GameObject("inertial").AddComponent<MjInertial>();
    _inertial.transform.parent = _body.transform;
  }

  [TearDown]
  public void TearDown() {
    GameObject.DestroyImmediate(_joint.gameObject);
    GameObject.DestroyImmediate(_inertial.gameObject);
    GameObject.DestroyImmediate(_body.gameObject);
    GameObject.DestroyImmediate(MjScene.Instance);
  }

  [UnityTest]
  public IEnumerator JointVelocityChangesTheConfiguration() {
    _joint.Velocity = 10;
    _scene.CreateScene();
    yield return new WaitForFixedUpdate();
    Assert.That(_joint.Configuration, Is.Not.EqualTo(0));
  }

  [UnityTest]
  public IEnumerator JointConfigurationIsUsed() {
    _joint.Configuration = 10;
    _scene.CreateScene();
    yield return new WaitForFixedUpdate();
    Assert.That(_joint.Configuration, Is.Not.EqualTo(0));
  }

  [UnityTest]
  public IEnumerator JointConfigurationIsReadOnly() {
    _scene.CreateScene();
    _joint.Configuration = 10;
    yield return new WaitForFixedUpdate();
    Assert.That(_joint.Configuration, Is.EqualTo(0));
  }
}
}

#endif
