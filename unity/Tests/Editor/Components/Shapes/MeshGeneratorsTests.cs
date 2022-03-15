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
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

namespace Mujoco {

  [TestFixture]
  public class MeshGeneratorsEditorTests {
    [Test]
    public void FirstAndLastVerticalSlicesContainVerticesLocatedInTheSamePlace() {
      var meshData = MeshGenerators.BuildSphere(scale: Vector3.one, numVerticalSlices: 4,
                                                numHorizontalSlices: 3);
      var vertices = meshData.Item1;
      Assert.That(vertices[0], Is.EqualTo(vertices[1]));
      Assert.That(vertices[1], Is.EqualTo(vertices[2]));
      Assert.That(vertices[2], Is.EqualTo(vertices[3]));
      Assert.That(vertices[8], Is.EqualTo(vertices[9]));
      Assert.That(vertices[9], Is.EqualTo(vertices[10]));
      Assert.That(vertices[10], Is.EqualTo(vertices[11]));
    }

    [Test]
    public void GeneratedSphereWithProperScaling() {
      var scale = new Vector3(0.5f, 2, 3);
      var meshData =
          MeshGenerators.BuildSphere(scale: scale, numVerticalSlices: 4, numHorizontalSlices: 3);
      var vertices = meshData.Item1;
      // One of the vertices of the bottom cap.
      Assert.That(vertices[0], Is.EqualTo(Vector3.down * scale.y).Using(Vector3ComparerWithEqualsOperator.Instance));
      // One of the vertices of the top cap.
      Assert.That(vertices[8], Is.EqualTo(Vector3.up * scale.y).Using(Vector3ComparerWithEqualsOperator.Instance));
      // The sides.
      Assert.That(vertices[4], Is.EqualTo(Vector3.right * scale.x).Using(Vector3ComparerWithEqualsOperator.Instance));
      Assert.That(vertices[5], Is.EqualTo(Vector3.back * scale.z).Using(Vector3ComparerWithEqualsOperator.Instance));
      Assert.That(vertices[6], Is.EqualTo(Vector3.left * scale.x).Using(Vector3ComparerWithEqualsOperator.Instance));
      Assert.That(vertices[7],
                  Is.EqualTo(Vector3.forward * scale.z).Using(Vector3ComparerWithEqualsOperator.Instance));
    }

    [Test]
    public void GeneratedCylinderHasCorrectHeight() {
      var meshData = MeshGenerators.BuildCylinder(radius: 1.0f, height: 5.0f, numVerticalSlices: 4);
      var bottomVertex =
          meshData.Item1.Aggregate((result, item) => result.y < item.y ? result : item);
      var topVertex = meshData.Item1.Aggregate((result, item) => result.y > item.y ? result : item);
      Assert.That(topVertex.y - bottomVertex.y, Is.EqualTo(5.0f).Within(1e-3f));
    }

    [Test]
    public void GeneratedCylinderHasCorrectRadius() {
      var meshData = MeshGenerators.BuildCylinder(radius: 2.0f, height: 1.0f, numVerticalSlices: 4);
      var averageRadius =
          meshData.Item1.Select(v => new Vector3(v.x, 0, v.z).magnitude).Average(radius => radius);
      Assert.That(averageRadius, Is.EqualTo(2.0f).Within(1e-3f));
    }

    [Test]
    public void GeneratedCapsuleHasCorrectHeight() {
      var meshData = MeshGenerators.BuildCapsule(radius: 1.0f, height: 5.0f, numVerticalSlices: 4,
                                                 numHorizontalSlices: 3);
      var bottomVertex =
          meshData.Item1.Aggregate((result, item) => result.y < item.y ? result : item);
      var topVertex = meshData.Item1.Aggregate((result, item) => result.y > item.y ? result : item);
      // The height of the capsule is the height of the body plus the diameter of the sphere that
      // forms the bases.
      Assert.That(topVertex.y - bottomVertex.y, Is.EqualTo(5.0f).Within(1e-3f));
    }

    [Test]
    public void GeneratedCapsuleHasCorrectRadius() {
      var meshData = MeshGenerators.BuildCapsule(radius: 2.0f, height: 3.0f, numVerticalSlices: 4,
                                                 numHorizontalSlices: 3);
      var averageRadius = meshData.Item1.Where(v => Math.Abs(v.y) <= 0.5f)
                              .Select(v => new Vector3(v.x, 0, v.z).magnitude)
                              .Average(radius => radius);
      Assert.That(averageRadius, Is.EqualTo(2.0f).Within(1e-3f));
    }

    [Test]
    public void GeneratedBoxHasCorrectExtents() {
      var extents = new Vector3(0.5f, 2, 3);
      var meshData = MeshGenerators.BuildBox(extents: extents);
      var minVertex = meshData.Item1.Aggregate(
          (result, item) =>
              (result.x + result.y + result.z) < (item.x + item.y + item.z) ? result : item);
      var maxVertex = meshData.Item1.Aggregate(
          (result, item) =>
              (result.x + result.y + result.z) > (item.x + item.y + item.z) ? result : item);
      Assert.That(minVertex, Is.EqualTo(extents * -1.0f).Using(Vector3ComparerWithEqualsOperator.Instance));
      Assert.That(maxVertex, Is.EqualTo(extents).Using(Vector3ComparerWithEqualsOperator.Instance));
    }

    [Test]
    public void GeneratedBoxTriangleMapIsAListOfConsecutiveIntegers() {
      var meshData = MeshGenerators.BuildBox(extents: Vector3.one);
      Assert.That(meshData.Item2, Has.Length.EqualTo(36));
      for (var i = 0; i < 36; ++i) {
        Assert.That(meshData.Item2[i], Is.EqualTo(i));
      }
    }

    [Test]
    public void GeneratedPlaneHasCorrectExtents() {
      var meshData = MeshGenerators.BuildPlane(width: 2, height: 3);
      var minVertex = meshData.Item1.Aggregate(
          (result, item) => (result.x + result.z) < (item.x + item.z) ? result : item);
      var maxVertex = meshData.Item1.Aggregate(
          (result, item) => (result.x + result.z) > (item.x + item.z) ? result : item);
      Assert.That(minVertex, Is.EqualTo(new Vector3(-1, 0, -1.5f)).Using(Vector3ComparerWithEqualsOperator.Instance));
      Assert.That(maxVertex, Is.EqualTo(new Vector3(1, 0, 1.5f)).Using(Vector3ComparerWithEqualsOperator.Instance));
    }
  }

  [TestFixture]
  public class MeshMergerEditorTests {
    private MeshMerger _meshMerger;

    [SetUp]
    public void SetUp() {
      _meshMerger = new MeshMerger();
    }

    [Test]
    public void AddingMeshVerticesAggregatesThem() {
      _meshMerger.Add(vertices: new Vector3[] { Vector3.one }, triangles: new int[] {});
      Assert.That(_meshMerger.Vertices, Has.Length.EqualTo(1));
      _meshMerger.Add(vertices: new Vector3[] { Vector3.forward }, triangles: new int[] {});
      Assert.That(_meshMerger.Vertices, Has.Length.EqualTo(2));
      Assert.That(_meshMerger.Vertices[0], Is.EqualTo(Vector3.one));
      Assert.That(_meshMerger.Vertices[1], Is.EqualTo(Vector3.forward));
    }

    [Test]
    public void TriangleIndicesAreOffsetToPointAtTheCorrespondingMeshes() {
      _meshMerger.Add(vertices: new Vector3[] { Vector3.right, Vector3.up },
                      triangles: new int[] { 0, 1 });
      _meshMerger.Add(vertices: new Vector3[] { Vector3.forward, Vector3.zero },
                      triangles: new int[] { 0, 1 });
      Assert.That(_meshMerger.Triangles, Has.Length.EqualTo(4));
      Assert.That(_meshMerger.Vertices[_meshMerger.Triangles[0]], Is.EqualTo(Vector3.right));
      Assert.That(_meshMerger.Vertices[_meshMerger.Triangles[1]], Is.EqualTo(Vector3.up));
      Assert.That(_meshMerger.Vertices[_meshMerger.Triangles[2]], Is.EqualTo(Vector3.forward));
      Assert.That(_meshMerger.Vertices[_meshMerger.Triangles[3]], Is.EqualTo(Vector3.zero));
    }

    [Test]
    public void TranslatingAddedVertices() {
      _meshMerger.AddAndTranslate(vertices: new Vector3[] { Vector3.zero }, triangles: new int[] {},
                                  translation: Vector3.right);
      _meshMerger.AddAndTranslate(vertices: new Vector3[] { Vector3.zero }, triangles: new int[] {},
                                  translation: Vector3.up);
      Assert.That(_meshMerger.Vertices[0], Is.EqualTo(Vector3.right));
      Assert.That(_meshMerger.Vertices[1], Is.EqualTo(Vector3.up));
    }
  }
}
