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

using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Mujoco {

/// <summary>
/// Scale vertex data manually line by line. We skip normals. Warning: Parameter vertex points
/// (`vp`) were unclear to me how to handle with scaling, if you use them and notice an issue
/// please report it.
/// </summary>
public static class ObjMeshImportUtility {
  private static Vector3 ToXZY(float x, float y, float z) => new Vector3(x, z, y);

  public static void CopyAndScaleOBJFile(string sourceFilePath, string targetFilePath,
      Vector3 scale) {
    // OBJ files are human-readable
    string[] lines = File.ReadAllLines(sourceFilePath);
    StringBuilder outputBuilder = new StringBuilder();
    // Culture info for consistent decimal point handling
    CultureInfo invariantCulture = CultureInfo.InvariantCulture;
    scale = ToXZY(scale.x, scale.y, scale.z);
    foreach (string line in lines) {
      if (line.StartsWith("v ")) // Vertex line
      {
        // Split the line into components
        string[] parts = line.Split(' ');
        if (parts.Length >= 4) {
          // Scale the vertex. It is unclear to me why flipping along x axis was necessary,
          // but without it meshes were incorrectly oriented.
          float x = -float.Parse(parts[1], invariantCulture) * scale.x;
          float y = float.Parse(parts[2], invariantCulture) * scale.y;
          float z = float.Parse(parts[3], invariantCulture) * scale.z;

          var swizzled = ToXZY(x, y, z);
          outputBuilder.AppendLine(
              $"v {swizzled.x.ToString(invariantCulture)} "+
              $"{swizzled.y.ToString(invariantCulture)} "+
              $"{swizzled.z.ToString(invariantCulture)}");
        }
      } else if (line.StartsWith("vn ")) {
        // We swizzle the normals too
        string[] parts = line.Split(' ');
        if (parts.Length >= 4) {
          float x = -float.Parse(parts[1], invariantCulture);
          float y = float.Parse(parts[2], invariantCulture);
          float z = float.Parse(parts[3], invariantCulture);

          var swizzled = ToXZY(x, y, z);
          outputBuilder.AppendLine(
              $"vn {swizzled.x.ToString(invariantCulture)} "+
              $"{swizzled.y.ToString(invariantCulture)} "+
              $"{swizzled.z.ToString(invariantCulture)}");
        }
      } else if (line.StartsWith("f ") && scale.x*scale.y*scale.z < 0) {
        // Faces definition, flip face by reordering vertices
        string[] parts = line.Split(' ');
        if (parts.Length >= 4) {
          outputBuilder.Append(parts[0]+" ");
          var face = parts.Skip(1).ToArray();
          if (face.Length >= 3) {
            outputBuilder.Append(face[0]+" ");
            outputBuilder.Append(face[2]+" ");
            outputBuilder.Append(face[1]);
          }
          outputBuilder.AppendLine();
        }
      } else {
        // Copy non-vertex lines as-is
        outputBuilder.AppendLine(line);
      }
    }
    // Write the scaled OBJ to the target file
    File.WriteAllText(targetFilePath, outputBuilder.ToString());
  }
}
}
