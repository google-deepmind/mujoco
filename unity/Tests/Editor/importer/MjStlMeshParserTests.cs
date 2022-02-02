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
using UnityEngine.TestTools;

namespace Mujoco {

[TestFixture]
public class MjStlMeshParserTests {

  private Mesh _originalMesh;

  [SetUp]
  public void SetUp() {
    _originalMesh = new Mesh();
    _originalMesh.vertices = new Vector3[] {
        new Vector3(0, 0, 2), new Vector3(1, 0, 2), new Vector3(0, 1, 2)
    };
    _originalMesh.triangles = new int[] {0, 1, 2};
    _originalMesh.normals = new Vector3[] {
        new Vector3(0, 0, 1), new Vector3(0, 0, 1), new Vector3(0, 0, 1)
    };
  }

  [Test]
  public void RoundrobinMeshParsing() {
    var serializedMesh = StlMeshParser.SerializeBinary(_originalMesh);
    var restoredMesh = StlMeshParser.ParseBinary(serializedMesh, Vector3.one);

    Assert.That(restoredMesh.vertices, Has.Length.EqualTo(_originalMesh.vertices.Length));
    Assert.That(restoredMesh.normals, Has.Length.EqualTo(_originalMesh.normals.Length));
    Assert.That(restoredMesh.triangles, Has.Length.EqualTo(_originalMesh.triangles.Length));
  }

  [Test]
  public void VertexPositionsSerialization() {
    var serializedMesh = StlMeshParser.SerializeBinary(_originalMesh);
    var restoredMesh = StlMeshParser.ParseBinary(serializedMesh, Vector3.one);

    Assert.That(
        restoredMesh.vertices[restoredMesh.triangles[0]],
        Is.EqualTo(_originalMesh.vertices[_originalMesh.triangles[0]]));
    Assert.That(
        restoredMesh.vertices[restoredMesh.triangles[1]],
        Is.EqualTo(_originalMesh.vertices[_originalMesh.triangles[1]]));
    Assert.That(
        restoredMesh.vertices[restoredMesh.triangles[2]],
        Is.EqualTo(_originalMesh.vertices[_originalMesh.triangles[2]]));
  }

  [Test]
  public void ProcessDuplicatesSharedVertices() {
    _originalMesh.triangles = new int[] {0, 0, 2};
    var serializedMesh = StlMeshParser.SerializeBinary(_originalMesh);
    var restoredMesh = StlMeshParser.ParseBinary(serializedMesh, Vector3.one);

    Assert.That(
        restoredMesh.vertices[restoredMesh.triangles[0]],
        Is.EqualTo(_originalMesh.vertices[_originalMesh.triangles[0]]));
    Assert.That(
        restoredMesh.vertices[restoredMesh.triangles[1]],
        Is.EqualTo(_originalMesh.vertices[_originalMesh.triangles[1]]));
    Assert.That(
        restoredMesh.vertices[restoredMesh.triangles[2]],
        Is.EqualTo(_originalMesh.vertices[_originalMesh.triangles[2]]));
  }

  [Test]
  public void VertexNormalsAreRcalculated() {
    _originalMesh.normals = new Vector3[] {
        new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1)
    };
    var serializedMesh = StlMeshParser.SerializeBinary(_originalMesh);
    var restoredMesh = StlMeshParser.ParseBinary(serializedMesh, Vector3.one);

    Assert.That(restoredMesh.normals[0], Is.EqualTo(Vector3.forward));
    Assert.That(restoredMesh.normals[1], Is.EqualTo(Vector3.forward));
    Assert.That(restoredMesh.normals[2], Is.EqualTo(Vector3.forward));
  }
}
}
