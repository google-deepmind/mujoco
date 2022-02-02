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
using System.IO;
using System.Linq;
using UnityEngine;

namespace Mujoco {

public static class MeshGenerators {

  // Creates a sphere mesh geometry.
  //
  // Args:
  //  scale: Non-uniform sphere scale. Allows to create ellipsoid shapes.
  //  numVerticalSlices: How many vertices should there be in a single horizontal slice.
  //  numHorizontalSlices: How many horizontal slices should the sphere consist of.
  public static Tuple<Vector3[], int[]> BuildSphere(
      Vector3 scale, int numVerticalSlices = 16, int numHorizontalSlices = 16) {
    // Generate the vertices.
    Vector3[] vertices;
    int[] triangles;
    GenerateSphereMeshSlice(
        scale: scale,
        numVerticalSlices: numVerticalSlices,
        numHorizontalSlices: numHorizontalSlices,
        firstSliceY: -1.0f,
        lastSliceY: 1.0f,
        vertices: out vertices,
        triangles: out triangles);
    return Tuple.Create(vertices, triangles);
  }

  // Creates a cylinder mesh geometry.
  //
  // Args:
  //  radius: Radius of the cylider's body.
  //  height: Height of the cylinder, from its bottom to its top base.
  //  numVerticalSlices: How many vertices should there be around the base's circumference.
  public static Tuple<Vector3[], int[]> BuildCylinder(
      float radius, float height, int numVerticalSlices = 16) {
    Vector3[] bodyVertices;
    int[] bodyTriangles;
    GenerateCylinderBody(radius, height, numVerticalSlices, out bodyVertices, out bodyTriangles);

    Vector3[] baseCapVertices;
    int[] baseCapTriangles;
    GenerateCylinderBaseCaps(
        radius, height, numVerticalSlices, out baseCapVertices, out baseCapTriangles);

    var merger = new MeshMerger();
    merger.Add(bodyVertices, bodyTriangles);
    merger.Add(baseCapVertices, baseCapTriangles);
    return Tuple.Create(merger.Vertices, merger.Triangles);
  }

  // Creates a capsule mesh geometry.
  //
  // Args:
  //  radius: Radius of the capsule's body.
  //  height: Height of the capsule, from its bottom to its top base.
  //  numVerticalSlices: How many vertices should there be in a single horizontal slice.
  //  numHorizontalSlices: How many vertical slices should the sphere consist of.
  public static Tuple<Vector3[], int[]> BuildCapsule(
      float radius, float height, int numVerticalSlices = 16, int numHorizontalSlices = 16) {
    var baseHalfHeight = Math.Max(0.0f, height * 0.5f - radius);
    Vector3[] topCapVertices;
    int[] topCapTriangles;
    GenerateSphereMeshSlice(
        scale: Vector3.one * radius,
        numVerticalSlices: numVerticalSlices,
        numHorizontalSlices: numHorizontalSlices / 2,
        firstSliceY: -1.0f,
        lastSliceY: 0.0f,
        vertices: out topCapVertices,
        triangles: out topCapTriangles);

    Vector3[] bottomCapVertices;
    int[] bottomCapTriangles;
    GenerateSphereMeshSlice(
        scale: Vector3.one * radius,
        numVerticalSlices: numVerticalSlices,
        numHorizontalSlices: numHorizontalSlices / 2,
        firstSliceY: 0.0f,
        lastSliceY: 1.0f,
        vertices: out bottomCapVertices,
        triangles: out bottomCapTriangles);

    Vector3[] bodyVertices;
    int[] bodyTriangles;
    GenerateCylinderBody(
        radius: radius,
        height: baseHalfHeight * 2.0f,
        numVerticalSlices: numVerticalSlices,
        vertices: out bodyVertices,
        triangles: out bodyTriangles);

    var merger = new MeshMerger();
    merger.AddAndTranslate(topCapVertices, topCapTriangles, Vector3.up * baseHalfHeight * -1.0f);
    merger.AddAndTranslate(
        bottomCapVertices, bottomCapTriangles, Vector3.up * baseHalfHeight * 1.0f);
    merger.Add(bodyVertices, bodyTriangles);
    return Tuple.Create(merger.Vertices, merger.Triangles);
  }

  // Creates a box mesh geometry.
  //
  // Args:
  //  extents: Extents of the box, along each major axis.
  public static Tuple<Vector3[], int[]> BuildBox(Vector3 extents) {
    // In order to ensure the box renders with flat faces, we need to make sure the triangles
    // do not share vertices. We'll accomplish that by assigning a unique vertex to every triangle
    // apex.
    // We define a set of 8 vertex positions that form the box, and then sample from that set using
    // triangle indices.
    var vertexPositions = new Vector3[] {
      Vector3.Scale(new Vector3(-1, -1, -1), extents),
      Vector3.Scale(new Vector3(1, -1, -1), extents),
      Vector3.Scale(new Vector3(-1, -1, 1), extents),
      Vector3.Scale(new Vector3(1, -1, 1), extents),
      Vector3.Scale(new Vector3(-1, 1, -1), extents),
      Vector3.Scale(new Vector3(1, 1, -1), extents),
      Vector3.Scale(new Vector3(-1, 1, 1), extents),
      Vector3.Scale(new Vector3(1, 1, 1), extents),
    };
    var vertexSamplingPattern = new int[] {
      0, 1, 3, 0, 3, 2,
      4, 7, 5, 4, 6, 7,
      0, 5, 1, 0, 4, 5,
      1, 7, 3, 1, 5, 7,
      3, 6, 2, 3, 7, 6,
      2, 4, 0, 2, 6, 4,
    };

    var vertices = vertexSamplingPattern.Select(index => vertexPositions[index]).ToArray();
    var triangles = Enumerable.Range(0, vertexSamplingPattern.Length).ToArray();
    return Tuple.Create(vertices, triangles);
  }

  // Creates a plane mesh geometry.
  //
  // Args:
  //  width: Width of the plane, along the OX axis.
  //  height: Height of the plane, along the OZ axis.
  public static Tuple<Vector3[], int[]> BuildPlane(float width, float height) {
    var vertices = new Vector3[] {
      new Vector3(-0.5f * width, 0, -0.5f * height),
      new Vector3(0.5f * width, 0, -0.5f * height),
      new Vector3(-0.5f * width, 0, 0.5f * height),
      new Vector3(0.5f * width, 0, 0.5f * height),
    };
    var triangles = new int[] {
      0, 3, 1,
      0, 2, 3,
    };
    return Tuple.Create(vertices, triangles);
  }

  // Generates a slice of a sphere mesh.
  // Conceptually, the algorithm generates a sphere with a unit radius (spanning from -1 to 1 along
  // each of the major axes). Then, it cuts it using 2 planes parallel to the XZ plane, located at
  // distances defined by 'firstSliceY' and 'lastSliceY' parameters respectively. It then returns
  // the section of the mesh contained between those planes.
  //
  // This allows to use the method to generate variants of the sphere - full sphere, hemispheres.
  //
  // Args:
  //  scale: Non-uniform sphere scale. Allows to create ellipsoid shapes.
  //  numVerticalSlices: How many vertices should there be in a single horizontal slice.
  //  numHorizontalSlices: How many horizontal slices should the sphere consist of.
  //  firstSliceY: Vertical position of the first slice. Must be a value in range <-1, 1>.
  //  lastSliceY: Vertical position of the last slice. Must a value in range <-1, 1>.
  //  vertices: (Out) Array of sphere vertex positions.
  //  triangles: (Out) Array with the sphere triangle connectivity.
  private static void GenerateSphereMeshSlice(
      Vector3 scale, int numVerticalSlices, int numHorizontalSlices, float firstSliceY,
      float lastSliceY, out Vector3[] vertices, out int[] triangles) {
    numVerticalSlices = Math.Max(3, numVerticalSlices);
    numHorizontalSlices = Math.Max(3, numHorizontalSlices);
    triangles = new int[(numHorizontalSlices - 1) * numVerticalSlices * 6];
    vertices = new Vector3[numVerticalSlices * numHorizontalSlices];
    if (firstSliceY < -1.0f || firstSliceY > 1.0f) {
      throw new IOException("firstSliceY should be a value in range <-1, 1>");
    }
    if (lastSliceY < -1.0f || lastSliceY > 1.0f) {
      throw new IOException("lastSliceY should be a value in range <-1, 1>");
    }
    if (firstSliceY > lastSliceY) {
      throw new IOException("Value of firstSliceY should be lower than the value of lastSliceY.");
    }
    // Generate the vertices.
    var deltaY = (lastSliceY - firstSliceY) / (numHorizontalSlices - 1);
    var deltaYaw = 360.0f / numVerticalSlices;
    for (var slice = 0; slice < numHorizontalSlices; ++slice) {
      var y = firstSliceY + deltaY * slice;
      var radius = (float)Math.Sqrt(1 - Math.Min(1.0f, y * y));
      for (var vertex = 0; vertex < numVerticalSlices; ++vertex) {
        var position = Quaternion.AngleAxis(deltaYaw * vertex, Vector3.up) * Vector3.right * radius;
        position.y = y;
        vertices[slice * numVerticalSlices + vertex] = Vector3.Scale(position, scale);
      }
    }
    // Build the triangles.
    for (var slice = 0; slice < (numHorizontalSlices - 1); ++slice) {
      var firstVertexInSlice = slice * numVerticalSlices;
      for (var vertex = 0; vertex < numVerticalSlices; ++vertex) {
        var index1 = firstVertexInSlice + vertex;
        var index2 = (vertex + 1 == numVerticalSlices) ? firstVertexInSlice : index1 + 1;
        var index3 = index1 + numVerticalSlices;
        var index4 = index2 + numVerticalSlices;
        var quadBaseAddress = (slice * numVerticalSlices + vertex) * 6;
        triangles[quadBaseAddress] = index1;
        triangles[quadBaseAddress + 1] = index2;
        triangles[quadBaseAddress + 2] = index4;
        triangles[quadBaseAddress + 3] = index1;
        triangles[quadBaseAddress + 4] = index4;
        triangles[quadBaseAddress + 5] = index3;
      }
    }
  }

  // Generates the cylinder mesh vertices.
  // This functionality will be shared between the cylinder body and cylinder base caps generators.
  //
  // Args:
  //  radius: Cylinder radius.
  //  height: Cylinder height.
  //  numVerticalSlices: How many vertices should there be around the base's circumference.
  //  vertices: (Out) Array of sphere vertex positions.
  private static void GenerateCylinderVertices(
      float radius, float height, int numVerticalSlices, out Vector3[] vertices) {
    vertices = new Vector3[numVerticalSlices * 2];
    var dYaw = 360.0f / numVerticalSlices;
    var yaw = 0.0f;
    var vertexIndex = 0;
    for (var y = 0; y <= 1; ++y) {
      var yPos = (y - 0.5f) * height;
      for (var i = 0; i < numVerticalSlices; ++i, yaw += dYaw) {
        var vertexPosition = Quaternion.AngleAxis(yaw, Vector3.up) * Vector3.right * radius;
        vertexPosition.y = yPos;
        vertices[vertexIndex++] = vertexPosition;
      }
    }
  }

  // Generates the meshes for the cylinder body, excluding its bases.
  //
  // Args:
  //  radius: Cylinder radius.
  //  height: Cylinder height.
  //  numVerticalSlices: How many vertices should there be around the base's circumference.
  //  vertices: (Out) Array of sphere vertex positions.
  //  triangles: (Out) Array with the sphere triangle connectivity.
  private static void GenerateCylinderBody(
      float radius, float height, int numVerticalSlices, out Vector3[] vertices,
      out int[] triangles) {
    GenerateCylinderVertices(radius, height, numVerticalSlices, out vertices);
    triangles = new int[numVerticalSlices * 6];
    var apexIndex = 0;
    for (var i = 0; i < numVerticalSlices; ++i) {
      var v1 = i;
      var v2 = (i + 1 == numVerticalSlices) ? 0 : v1 + 1;
      var v3 = v1 + numVerticalSlices;
      var v4 = v2 + numVerticalSlices;
      triangles[apexIndex++] = v1;
      triangles[apexIndex++] = v2;
      triangles[apexIndex++] = v4;
      triangles[apexIndex++] = v1;
      triangles[apexIndex++] = v4;
      triangles[apexIndex++] = v3;
    }
  }

  // Generates the meshes for the cylinder base circles.
  //
  // Args:
  //  radius: Cylinder radius.
  //  height: Cylinder height.
  //  numVerticalSlices: How many vertices should there be around the base's circumference.
  //  vertices: (Out) Array of sphere vertex positions.
  //  triangles: (Out) Array with the sphere triangle connectivity.
  private static void GenerateCylinderBaseCaps(
      float radius, float height, int numVerticalSlices, out Vector3[] vertices,
      out int[] triangles) {
    GenerateCylinderVertices(radius, height, numVerticalSlices, out vertices);
    triangles = new int[(numVerticalSlices - 1) * 6];
    var apexIndex = 0;
    for (var baseIdx = 0; baseIdx <= 1; ++baseIdx) {
      var v1 = baseIdx * numVerticalSlices;
      for (var i = 1; i < numVerticalSlices; ++i) {
        var v2 = v1 + (i % numVerticalSlices);
        var v3 = v1 + ((i + 1) % numVerticalSlices);
        triangles[apexIndex++] = v1;
        triangles[apexIndex++] = baseIdx == 0 ? v3 : v2;
        triangles[apexIndex++] = baseIdx == 0 ? v2 : v3;
      }
    }
  }
}

// A tool used to merge the geometry of separate meshes.
public class MeshMerger {
  private List<Vector3> _vertices = new List<Vector3>();
  private List<int> _triangles = new List<int>();

  // The vertices of the merged mesh.
  public Vector3[] Vertices => _vertices.ToArray();

  // The connectivity array of the merged mesh.
  public int[] Triangles => _triangles.ToArray();

  // Adds a new submesh.
  //
  // Args:
  //  vertices: Array of submesh vertices.
  //  vertices: Triangles connectivity array of the submesh.
  public void Add(Vector3[] vertices, int[] triangles) {
    var triangleOffset = _triangles.Count;
    var vertexOffset = _vertices.Count;
    _vertices.AddRange(vertices);
    _triangles.AddRange(triangles);
    for (var i = triangleOffset; i < _triangles.Count; ++i) {
      _triangles[i] += vertexOffset;
    }
  }

  // Adds a new submesh, translating its vertices by a specified amount.
  //
  // Args:
  //  vertices: Array of submesh vertices.
  //  vertices: Triangles connectivity array of the submesh.
  //  translation: Additional translation to be applied to the submesh.
  public void AddAndTranslate(Vector3[] vertices, int[] triangles, Vector3 translation) {
    var triangleOffset = _triangles.Count;
    var vertexOffset = _vertices.Count;
    _vertices.AddRange(vertices);
    for (var i = vertexOffset; i < _vertices.Count; ++i) {
      _vertices[i] += translation;
    }
    _triangles.AddRange(triangles);
    for (var i = triangleOffset; i < _triangles.Count; ++i) {
      _triangles[i] += vertexOffset;
    }
  }
}
}
