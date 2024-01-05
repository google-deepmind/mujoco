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
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

namespace Mujoco {

public static class BinaryReaderExtensions {
  public static Vector3 ReadVector3(this BinaryReader reader) {
    var x = reader.ReadSingle();
    var y = reader.ReadSingle();
    var z = reader.ReadSingle();
    return new Vector3(x, y, z);
  }
}

public static class BinaryWriterExtensions {
  public static void Write(this BinaryWriter writer, Vector3 val) {
    writer.Write(val.x);
    writer.Write(val.y);
    writer.Write(val.z);
  }
}

public class StlMeshParser {

  private const int _headerLength = 80;
  private const int _attributesSizeLength = 2;
  private const int _verticesPerTriangle = 3;
  private const int _unityLimitNumVerticesPerMesh = 65535;
  private const string _asciiFileTypeId = "solid";

  private static Vector3 ToXZY(Vector3 v) => new Vector3(v.x, v.z, v.y);

  // The binary STL format is described here: https://en.wikipedia.org/wiki/STL_(file_format)
  public static Mesh ParseBinary(byte[] stlFileContents, Vector3 scale) {
    var fileTypeId = System.Text.Encoding.UTF8.GetString(
      stlFileContents.Take(_asciiFileTypeId.Length).ToArray());
    if (fileTypeId == _asciiFileTypeId) {
      throw new IOException("Ascii STL file format is not supported.");
    }

    using (var stream = new MemoryStream(stlFileContents)) {
      using (var reader = new BinaryReader(stream)) {
        reader.ReadBytes(_headerLength);
        var numTriangles = reader.ReadUInt32();
        var numVertices = numTriangles * _verticesPerTriangle;
        if (numVertices > _unityLimitNumVerticesPerMesh) {
          throw new IndexOutOfRangeException(
              "The mesh exceeds the number of vertices per mesh allowed by Unity. " +
              $"({numVertices} > {_unityLimitNumVerticesPerMesh})");
        }
        var triangleIndices = new List<int>(capacity: (int)numVertices);
        var vertices = new List<Vector3>(capacity: (int)numVertices);
        var normals = new List<Vector3>(capacity: (int)numVertices);
        for (var i = 0; i < numVertices; i += _verticesPerTriangle) {
          var triangleNormal = ToXZY(reader.ReadVector3());
          normals.AddRange(new[] { triangleNormal, triangleNormal, triangleNormal });
          vertices.AddRange(new[] {
            ToXZY(reader.ReadVector3()),
            ToXZY(reader.ReadVector3()),
            ToXZY(reader.ReadVector3()) });
          triangleIndices.AddRange(new[] {i, i + 2, i + 1});
          reader.ReadInt16();  // Read the unused attribute indices field.
        }

        var mesh = new Mesh();
        mesh.vertices = vertices.ToArray();
        mesh.normals = normals.ToArray();
        mesh.triangles = triangleIndices.ToArray();
        mesh.vertices = mesh.vertices.Select(
            vertexPosition => Vector3.Scale(vertexPosition, scale)).ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateTangents();
        mesh.RecalculateBounds();
        return mesh;
      }
    }
  }

  public static byte[] SerializeBinary(Mesh mesh) {
    using (var stream = new MemoryStream()) {
      using (var writer = new BinaryWriter(stream)) {
        // Write the header. We only want to write the id and then pad the rest with zeros, up to 80
        // bytes.
        writer.Write(_asciiFileTypeId);
        writer.Write(new byte[_headerLength - _asciiFileTypeId.Length - 1]);

        // Reading mesh.triangles etc causes a c# array to be instantiated each time to store a copy
        // of the data that is owned by the native runtime. For this reason, it's important we do
        // this once per mesh, and definitely not per triangle.
        var triangles = mesh.triangles;
        var normals = mesh.normals;
        var vertices = mesh.vertices;

        var numTriangles = triangles.Length / 3;
        writer.Write((int)numTriangles);

        for (var i = 0; i < triangles.Length; i += _verticesPerTriangle) {
          // STL format uses face normals, while Unity Meshes use vertex normals. We need to convert
          // one into another by calculating a mean of vertex normals.
          var i1 = triangles[i];
          var i2 = triangles[i + 1];
          var i3 = triangles[i + 2];
          var faceNormal = (normals[i1] + normals[i2] + normals[i3]).normalized;
          writer.Write(ToXZY(faceNormal));

          writer.Write(ToXZY(vertices[i1]));
          writer.Write(ToXZY(vertices[i3]));
          writer.Write(ToXZY(vertices[i2]));

          writer.Write((short)0);
        }
        return stream.GetBuffer();
      }
    }
  }
}
}
