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
using UnityEngine;

namespace Mujoco {

// Additional Debug Gizmos.
public static class MjGizmos {

  private const float _vectorPerpendicularityMagnitudeThreshold = 1e-6f;

  public static int NumCircleSegments = 32;

  private static Vector3[] _circumferenceVertices = new Vector3[] {
    Vector3.right, -Vector3.right, Vector3.forward, -Vector3.forward,
  };

  public static Action<Vector3 /*from*/, Vector3 /*to*/> DrawLine { get; set; } = Gizmos.DrawLine;

  public static Action<Vector3 /*position*/, float /*radius*/, Vector3 /*axis*/, float /*length*/,
                       float /*startAngle*/> DrawArc { get; set; } = MjGizmos.DrawArcInternal;

  // Draws a wireframe capsule of the specified radius and height.
  // The capsule's height will be aligned with the global Y axis.
  // The specified height represents the height of the cylinder between two hemisphere bases.
  public static void DrawWireCapsule(Vector3 position, float radius, float height) {
    var offset = DrawHollowWireCylinder(position, radius, height);
    // Arches marking the bases of the capsule.
    DrawArc(position - offset, radius, Vector3.right, 0.5f, 0.0f);
    DrawArc(position + offset, radius, Vector3.right, 0.5f, 180.0f);
    DrawArc(position + offset, radius, Vector3.forward, 0.5f, -90.0f);
    DrawArc(position - offset, radius, Vector3.forward, 0.5f, 90.0f);
  }

  // Draws a wireframe cylinder of the specified radius and height.
  // The cylinder's height will be aligned with the global Y axis.
  public static void DrawWireCylinder(Vector3 position, float radius, float height) {
    var offset = DrawHollowWireCylinder(position, radius, height);
    // Lines marking the bases of the cylinder.
    for (var i = 0; i < _circumferenceVertices.Length / 2; ++i) {
      var vertex = _circumferenceVertices[i * 2] * radius;
      var nextVertex = _circumferenceVertices[i * 2 + 1] * radius;
      DrawLine(position + offset + vertex, position + offset + nextVertex);
      DrawLine(position - offset + vertex, position - offset + nextVertex);
    }
  }

  private static Vector3 DrawHollowWireCylinder(Vector3 position, float radius, float height) {
    var offset = Vector3.up * (height * 0.5f);
    // Cylinder bases.
    DrawArc(position + offset, radius, Vector3.up, 1.0f, 0.0f);
    DrawArc(position - offset, radius, Vector3.up, 1.0f, 0.0f);
    // Lines marking the sides of the cylinder.
    for (var i = 0; i < _circumferenceVertices.Length; ++i) {
      // Lines marking the sides of the cylinder.
      var vertex = _circumferenceVertices[i] * radius;
      DrawLine(position + offset + vertex, position - offset + vertex);
    }
    return offset;
  }

  // Draws an arc in the XZ plane.
  // 'length' parameter, in range (0, 1], allows to define how much of the circumference should be
  // drawn. In other words, it allows to draw arbitrary arcs.
  // Two additional parameters, 'axis' and 'startAngle', allow to orient the figure without having
  // to manipulate the value of Gizmos.matrix.
  public static void DrawArcInternal(
      Vector3 position, float radius, Vector3 axis, float length, float startAngle) {
    var deltaAngle = 360.0f / NumCircleSegments;
    var numSegments = Math.Ceiling(length * NumCircleSegments);

    var perpendicular = Vector3.Cross(axis, Vector3.right);
    if (perpendicular.magnitude <= _vectorPerpendicularityMagnitudeThreshold) {
      perpendicular = Vector3.Cross(axis, Vector3.up);
    }
    perpendicular.Normalize();

    var offset = perpendicular * radius;
    var start = Quaternion.AngleAxis(startAngle, axis) * offset;
    var angle = startAngle + deltaAngle;

    for (var segment = 0; segment < numSegments; ++segment, angle += deltaAngle) {
      var end = Quaternion.AngleAxis(angle, axis) * offset;
      DrawLine(position + start, position + end);
      start = end;
    }
  }

  public static void DrawWirePlane(Vector3 position, float width, float height) {
    var quadPoints = new Vector3[] {
      new Vector3(-0.5f * width, 0, -0.5f * height),
      new Vector3(0.5f * width, 0, -0.5f * height),
      new Vector3(0.5f * width, 0, 0.5f * height),
      new Vector3(-0.5f * width, 0, 0.5f * height),
      new Vector3(-0.5f * width, 0, -0.5f * height),
      new Vector3(0.5f * width, 0, 0.5f * height),
    };
    for (var i = 0; i < quadPoints.Length - 1; ++i) {
      DrawLine(position + quadPoints[i], position + quadPoints[i + 1]);
    }
  }
}
}
