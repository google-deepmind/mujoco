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
using NUnit.Framework;
using UnityEngine;

namespace Mujoco {

  [TestFixture]
  public class MjTransformationTests {
    [Test]
    public void ConcatenateTranslation() {
      var t1 = new MjTransformation(new Vector3(10, 0, 0), Quaternion.identity);
      var t2 = new MjTransformation(new Vector3(0, 20, 0), Quaternion.identity);
      var result = t1 * t2;
      var expectedResult = new MjTransformation(new Vector3(10, 20, 0), Quaternion.identity);
      CompareTransforms(result, expectedResult);
    }

    [Test]
    public void ConcatenateRotation() {
      var t1 = new MjTransformation(Vector3.zero, Quaternion.Euler(0, 45, 0));
      var t2 = new MjTransformation(Vector3.zero, Quaternion.Euler(0, 15, 0));
      var result = t1 * t2;
      var expectedResult = new MjTransformation(Vector3.zero, Quaternion.Euler(0, 60, 0));
      CompareTransforms(result, expectedResult);
    }

    [Test]
    public void ConcatenateTranslationAndRotation() {
      var t1 = new MjTransformation(new Vector3(10, 0, 0), Quaternion.Euler(0, 45, 0));
      var t2 = new MjTransformation(new Vector3(20, 0, 0), Quaternion.Euler(0, 15, 0));
      var result = t1 * t2;
      // T2 is some translation and rotation in T1's reference frame. We want to learn what that
      // transform would look like in global space - as if there was no T1.
      // T1 is located at (10, 0, 0), and rotates its children by 45 deg around the Y axis.
      // That means it will rotate T2 translation (20, 0, 0) by 45 degrees. Since the rotation
      // is performed around the Y axis, the X coordinate 20 will be projected onto X and Z axis
      // in amount [cos(45deg) * 20, -sin(45deg) * 20] = [14.14213, -14.14213].
      // Next, the rotation will be added to that of the parent: 45 + 15 = 60 deg.
      var expectedResult = new MjTransformation(new Vector3(24.14213f, 0, -14.14213f),
                                                    Quaternion.Euler(0, 60, 0));
      CompareTransforms(result, expectedResult);
    }

    [Test]
    public void ConcatenatingRightInverse() {
      var t1 = new MjTransformation(new Vector3(10, 0, 0), Quaternion.Euler(0, 45, 0));
      var result = t1 * t1.Inverse();
      var expectedResult = new MjTransformation(Vector3.zero, Quaternion.identity);
      CompareTransforms(result, expectedResult);
    }

    [Test]
    public void ConcatenatingLeftInverse() {
      var t1 = new MjTransformation(new Vector3(10, 0, 0), Quaternion.Euler(0, 45, 0));
      var result = t1.Inverse() * t1;
      var expectedResult = new MjTransformation(Vector3.zero, Quaternion.identity);
      CompareTransforms(result, expectedResult);
    }

    [Test]
    public void MjTransformationsConcatenationIsTheSameAsUnitys() {
      _parent.transform.position = new Vector3(10, 0, 0);
      _parent.transform.rotation = Quaternion.Euler(0, 45, 0);
      _child.transform.localPosition = new Vector3(20, 0, 0);
      _child.transform.localRotation = Quaternion.Euler(0, 15, 0);
      var t1 = MjTransformation.LoadGlobal(_parent.transform);
      var t2 = MjTransformation.LoadLocal(_child.transform);
      var result = t1 * t2;
      var expectedResult = MjTransformation.LoadGlobal(_child.transform);
      var expectedExplicitResult = new MjTransformation(new Vector3(24.14213f, 0, -14.14213f),
                                                            Quaternion.Euler(0, 60, 0));
      CompareTransforms(result, expectedResult);
      CompareTransforms(result, expectedExplicitResult);
    }

    [Test]
    public void LoadingGlobalTransform() {
      var result = MjTransformation.LoadGlobal(_parent.transform);
      CompareTransforms(result, _parent.transform.position, _parent.transform.rotation);
    }

    [Test]
    public void LoadingLocalTransform() {
      var result = MjTransformation.LoadLocal(_parent.transform);
      CompareTransforms(result, _parent.transform.localPosition, _parent.transform.localRotation);
    }

    [Test]
    public void StoringGlobalTransform() {
      var transform = new MjTransformation(new Vector3(1, 2, 3), Quaternion.Euler(0, 45, 0));
      transform.StoreGlobal(_parent.transform);
      CompareTransforms(transform, _parent.transform.position, _parent.transform.rotation);
    }

    [Test]
    public void StoringLocalTransform() {
      var transform = new MjTransformation(new Vector3(1, 2, 3), Quaternion.Euler(0, 45, 0));
      transform.StoreGlobal(_parent.transform);
      CompareTransforms(transform, _parent.transform.localPosition,
                        _parent.transform.localRotation);
    }

#region Setup

    private GameObject _parent;
    private GameObject _child;

    [SetUp]
    public void SetUp() {
      _parent = new GameObject("parent");
      _child = new GameObject("child");
      _child.transform.parent = _parent.transform;
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_parent);
    }

    private void CompareTransforms(MjTransformation lhs, MjTransformation rhs) {
      Assert.That(lhs.Translation.x, Is.EqualTo(rhs.Translation.x).Within(1e-5f));
      Assert.That(lhs.Translation.y, Is.EqualTo(rhs.Translation.y).Within(1e-5f));
      Assert.That(lhs.Translation.z, Is.EqualTo(rhs.Translation.z).Within(1e-5f));
      Assert.That(lhs.Rotation.x, Is.EqualTo(rhs.Rotation.x).Within(1e-5f));
      Assert.That(lhs.Rotation.y, Is.EqualTo(rhs.Rotation.y).Within(1e-5f));
      Assert.That(lhs.Rotation.z, Is.EqualTo(rhs.Rotation.z).Within(1e-5f));
      Assert.That(lhs.Rotation.w, Is.EqualTo(rhs.Rotation.w).Within(1e-5f));
    }

    private void CompareTransforms(MjTransformation lhs, Vector3 position,
                                   Quaternion rotation) {
      Assert.That(lhs.Translation.x, Is.EqualTo(position.x).Within(1e-5f));
      Assert.That(lhs.Translation.y, Is.EqualTo(position.y).Within(1e-5f));
      Assert.That(lhs.Translation.z, Is.EqualTo(position.z).Within(1e-5f));
      Assert.That(lhs.Rotation.x, Is.EqualTo(rotation.x).Within(1e-5f));
      Assert.That(lhs.Rotation.y, Is.EqualTo(rotation.y).Within(1e-5f));
      Assert.That(lhs.Rotation.z, Is.EqualTo(rotation.z).Within(1e-5f));
      Assert.That(lhs.Rotation.w, Is.EqualTo(rotation.w).Within(1e-5f));
    }

#endregion
  }
}
