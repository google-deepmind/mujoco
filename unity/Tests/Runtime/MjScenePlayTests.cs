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
public class MjScenePlayTests {
  private MjScene _scene;
  private MjBody _body;
  private MjInertial _inertia;
  private MjHingeJoint _joint;

  [SetUp]
  public void SetUp() {
    _scene = MjScene.Instance;
    _body = new GameObject("body").AddComponent<MjBody>();
    _inertia = new GameObject("inertia").AddComponent<MjInertial>();
    _inertia.transform.parent = _body.transform;
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _joint.transform.parent = _body.transform;
  }

  [TearDown]
  public void TearDown() {
    GameObject.DestroyImmediate(_joint.gameObject);
    GameObject.DestroyImmediate(_inertia.gameObject);
    GameObject.DestroyImmediate(_body.gameObject);
    GameObject.DestroyImmediate(_scene.gameObject);
  }

  [UnityTest]
  public IEnumerator RecreatedScenePreservesPosition() {
    _joint.Velocity = .1f;
    _scene.CreateScene();
    yield return null; // qpos moves, xpos stays at zero
    yield return null; // now xpos changed
    var tickedRotation = _body.transform.rotation;
    _scene.RecreateScene();
    // xpos is one step behind qpos, so it's not a perfect no-op, hence the high tolerance
    Assert.That(tickedRotation.x, Is.EqualTo(_body.transform.rotation.x).Within(1e-3f));
    Assert.That(tickedRotation.y, Is.EqualTo(_body.transform.rotation.y).Within(1e-3f));
    Assert.That(tickedRotation.z, Is.EqualTo(_body.transform.rotation.z).Within(1e-3f));
    Assert.That(tickedRotation.w, Is.EqualTo(_body.transform.rotation.w).Within(1e-3f));
    // Assert.That(tickedRotation, Is.EqualTo(_body.transform.rotation).Using(_quaternionComparer));
  }

  [UnityTest]
  public IEnumerator SceneRecreatedWithAddition() {
    _joint.Velocity = 1;
    _scene.CreateScene();
    Assert.That(_scene.SceneRecreationAtLateUpdateRequested, Is.False);
    var body = new GameObject("body").AddComponent<MjBody>();
    var inertia = new GameObject("inertia").AddComponent<MjInertial>();
    inertia.transform.parent = body.transform;
    var joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    joint.transform.parent = body.transform;
    yield return null; // call "start"
    Assert.That(_scene.SceneRecreationAtLateUpdateRequested, Is.True);
    yield return null; // call "lateUpdate"
    Assert.That(_scene.SceneRecreationAtLateUpdateRequested, Is.False);
    UnityEngine.Object.DestroyImmediate(inertia.gameObject);
    UnityEngine.Object.DestroyImmediate(joint.gameObject);
    UnityEngine.Object.DestroyImmediate(body.gameObject);
  }

  [UnityTest]
  public IEnumerator SceneRecreatedAfterDeletion() {
    var body = new GameObject("body").AddComponent<MjBody>();
    var inertia = new GameObject("inertia").AddComponent<MjInertial>();
    inertia.transform.parent = body.transform;
    var joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    joint.transform.parent = body.transform;
    yield return null;
    yield return null;
    Assert.That(_scene.SceneRecreationAtLateUpdateRequested, Is.False);
    UnityEngine.Object.DestroyImmediate(inertia.gameObject);
    UnityEngine.Object.DestroyImmediate(joint.gameObject);
    UnityEngine.Object.DestroyImmediate(body.gameObject);
    Assert.That(_scene.SceneRecreationAtLateUpdateRequested, Is.True);
    yield return null; // should result in a new scene
    Assert.That(_scene.SceneRecreationAtLateUpdateRequested, Is.False);
  }
}
}
#endif
