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
public class MjSiteStateSyncTests {
  private MjBody _body;
  private MjSite _site;
  private MjScene _scene;

  [SetUp]
  public void SetUp() {
    _scene = MjScene.Instance;
    _body = new GameObject("body").AddComponent<MjBody>();
    _site = new GameObject("site").AddComponent<MjSite>();
    _site.transform.parent = _body.transform;
  }

  [TearDown]
  public void TearDown() {
    GameObject.DestroyImmediate(_site.gameObject);
    GameObject.DestroyImmediate(_body.gameObject);
    GameObject.DestroyImmediate(_scene.gameObject);
    GameObject.DestroyImmediate(MjScene.Instance);
  }

  [UnityTest]
  public IEnumerator InitialTransform() {
    _site.transform.position = new Vector3(1, 2, 3);
    _scene.CreateScene(); // compilation happens here
    yield return new WaitForFixedUpdate(); // updating the transform
    Assert.That(_site.transform.position, Is.EqualTo(new Vector3(1, 2, 3)));
  }

  [UnityTest]
  public IEnumerator ChangedTransformIsOverwrittenByItsMjCounterpart() {
    _scene.CreateScene(); // compilation happens here
    _site.transform.position = new Vector3(1, 2, 3);
    yield return new WaitForFixedUpdate(); // updating the transform
    Assert.That(_site.transform.position, Is.EqualTo(Vector3.zero));
  }
}
}

#endif
