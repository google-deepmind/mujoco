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
  public class MjGizmosDrawArcTests {
    [SetUp]
    public void SetUp() {
      MjGizmos.DrawLine = MockDrawLine;
      MjGizmos.NumCircleSegments = 4;
      _lineFrom = new List<Vector3>();
      _lineTo = new List<Vector3>();
    }

    [TearDown]
    public void TearDown() {
      MjGizmos.DrawLine = Gizmos.DrawLine;
      MjGizmos.NumCircleSegments = 32;
    }

    private List<Vector3> _lineFrom;
    private List<Vector3> _lineTo;

    private void MockDrawLine(Vector3 from, Vector3 to) {
      _lineFrom.Add(from);
      _lineTo.Add(to);
    }

    [Test]
    public void LineCountOfDrawnCircle() {
      MjGizmos.DrawArc(Vector3.zero, 1.0f, Vector3.up, 1.0f, 0.0f);
      Assert.That(_lineFrom, Has.Count.EqualTo(MjGizmos.NumCircleSegments));
      Assert.That(_lineTo, Has.Count.EqualTo(MjGizmos.NumCircleSegments));
    }

    [Test]
    public void LineAdjacencyOfDrawnCircle() {
      MjGizmos.DrawArc(Vector3.zero, 1.0f, Vector3.up, 1.0f, 0.0f);
      Assert.That((_lineFrom[0] - _lineTo[3]).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[1] - _lineTo[0]).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[2] - _lineTo[1]).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[3] - _lineTo[2]).magnitude, Is.LessThan(1e-3f));
    }

    [Test]
    public void CircleVertexPositions() {
      MjGizmos.DrawArc(Vector3.zero, 1.0f, Vector3.up, 1.0f, 0.0f);
      Assert.That((_lineFrom[0] - new Vector3(0, 0, -1)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[1] - new Vector3(-1, 0, 0)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[2] - new Vector3(0, 0, 1)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[3] - new Vector3(1, 0, 0)).magnitude, Is.LessThan(1e-3f));
    }

    [Test]
    public void VerticesOfOffsetCircle() {
      MjGizmos.DrawArc(Vector3.one, 1.0f, Vector3.up, 1.0f, 0.0f);
      Assert.That((_lineFrom[0] - new Vector3(1, 1, 0)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[1] - new Vector3(0, 1, 1)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[2] - new Vector3(1, 1, 2)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[3] - new Vector3(2, 1, 1)).magnitude, Is.LessThan(1e-3f));
    }

    [Test]
    public void DrawingRotatedCircle() {
      MjGizmos.DrawArc(Vector3.zero, 1.0f, Vector3.right, 1.0f, 0.0f);
      Assert.That((_lineFrom[0] - new Vector3(0, 0, 1)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[1] - new Vector3(0, -1, 0)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[2] - new Vector3(0, 0, -1)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineFrom[3] - new Vector3(0, 1, 0)).magnitude, Is.LessThan(1e-3f));
    }

    [Test]
    public void DrawingHalfCircle() {
      MjGizmos.DrawArc(Vector3.zero, 1.0f, Vector3.up, 0.25f, 0.0f);
      Assert.That(_lineFrom, Has.Count.EqualTo(1));
      Assert.That(_lineTo, Has.Count.EqualTo(1));
      Assert.That((_lineFrom[0] - new Vector3(0, 0, -1)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineTo[0] - new Vector3(-1, 0, 0)).magnitude, Is.LessThan(1e-3f));
    }

    [Test]
    public void DrawingHalfCircleWithOffsetCircumference() {
      MjGizmos.DrawArc(Vector3.zero, 1.0f, Vector3.up, 0.25f, 90);
      Assert.That((_lineFrom[0] - new Vector3(-1, 0, 0)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_lineTo[0] - new Vector3(0, 0, 1)).magnitude, Is.LessThan(1e-3f));
    }
  }

  [TestFixture]
  public class MjGizmosCapsuleTests {
    [SetUp]
    public void SetUp() {
      MjGizmos.DrawLine = MockDrawLine;
      MjGizmos.DrawArc = MockDrawArc;
      MjGizmos.NumCircleSegments = 4;
      _arc = new List<Tuple<Vector3, float, Vector3, float, float>>();
    }

    [TearDown]
    public void TearDown() {
      MjGizmos.DrawLine = Gizmos.DrawLine;
      MjGizmos.DrawArc = MjGizmos.DrawArcInternal;
      MjGizmos.NumCircleSegments = 32;
    }

    private List<Tuple<Vector3, float, Vector3, float, float>> _arc;

    private void MockDrawLine(Vector3 from, Vector3 to) {}

    private void MockDrawArc(Vector3 position, float radius, Vector3 axis, float length,
                             float startAngle) {
      _arc.Add(Tuple.Create(position, radius, axis, length, startAngle));
    }

    [Test]
    public void NumberOfArcsUsedToDrawCapsule() {
      MjGizmos.DrawWireCapsule(Vector3.zero, 0.5f, 1.0f);
      Assert.That(_arc, Has.Count.EqualTo(6));
    }

    [Test]
    public void CapsuleBaseCirclesOrigins() {
      MjGizmos.DrawWireCapsule(Vector3.zero, 0.5f, 1.0f);
      Assert.That((_arc[0].Item1 - new Vector3(0, 0.5f, 0)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_arc[1].Item1 - new Vector3(0, -0.5f, 0)).magnitude, Is.LessThan(1e-3f));
    }

    [Test]
    public void CapsuleBaseCirclesRadiuses() {
      MjGizmos.DrawWireCapsule(Vector3.zero, 0.5f, 1.0f);
      Assert.That(_arc[0].Item2, Is.EqualTo(0.5f));
      Assert.That(_arc[1].Item2, Is.EqualTo(0.5f));
    }

    [Test]
    public void CapsuleBaseArcsOrigins() {
      MjGizmos.DrawWireCapsule(Vector3.zero, 0.5f, 1.0f);
      Assert.That((_arc[2].Item1 - new Vector3(0, -0.5f, 0)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_arc[3].Item1 - new Vector3(0, 0.5f, 0)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_arc[4].Item1 - new Vector3(0, 0.5f, 0)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_arc[5].Item1 - new Vector3(0, -0.5f, 0)).magnitude, Is.LessThan(1e-3f));
    }

    [Test]
    public void CapsuleBaseArcsRadiuses() {
      MjGizmos.DrawWireCapsule(Vector3.zero, 0.5f, 1.0f);
      Assert.That(_arc[2].Item2, Is.EqualTo(0.5f));
      Assert.That(_arc[3].Item2, Is.EqualTo(0.5f));
      Assert.That(_arc[4].Item2, Is.EqualTo(0.5f));
      Assert.That(_arc[5].Item2, Is.EqualTo(0.5f));
    }
  }

  [TestFixture]
  public class MjGizmosCylinderTests {
    [SetUp]
    public void SetUp() {
      MjGizmos.DrawLine = MockDrawLine;
      MjGizmos.DrawArc = MockDrawArc;
      MjGizmos.NumCircleSegments = 4;
      _arc = new List<Tuple<Vector3, float, Vector3, float, float>>();
    }

    [TearDown]
    public void TearDown() {
      MjGizmos.DrawLine = Gizmos.DrawLine;
      MjGizmos.DrawArc = MjGizmos.DrawArcInternal;
      MjGizmos.NumCircleSegments = 32;
    }

    private List<Tuple<Vector3, float, Vector3, float, float>> _arc;

    private void MockDrawLine(Vector3 from, Vector3 to) {}

    private void MockDrawArc(Vector3 position, float radius, Vector3 axis, float length,
                             float startAngle) {
      _arc.Add(Tuple.Create(position, radius, axis, length, startAngle));
    }

    [Test]
    public void NumberOfArcsUsedToDrawCylinder() {
      MjGizmos.DrawWireCylinder(Vector3.zero, 0.5f, 1.0f);
      Assert.That(_arc, Has.Count.EqualTo(2));
    }

    [Test]
    public void CylinderBaseCirclesOrigins() {
      MjGizmos.DrawWireCylinder(Vector3.zero, 0.5f, 1.0f);
      Assert.That((_arc[0].Item1 - new Vector3(0, 0.5f, 0)).magnitude, Is.LessThan(1e-3f));
      Assert.That((_arc[1].Item1 - new Vector3(0, -0.5f, 0)).magnitude, Is.LessThan(1e-3f));
    }

    [Test]
    public void CylinderBaseCirclesRadiuses() {
      MjGizmos.DrawWireCylinder(Vector3.zero, 0.5f, 1.0f);
      Assert.That(_arc[0].Item2, Is.EqualTo(0.5f));
      Assert.That(_arc[1].Item2, Is.EqualTo(0.5f));
    }
  }

  [TestFixture]
  public class MjGizmosPlaneTests {
    [SetUp]
    public void SetUp() {
      MjGizmos.DrawLine = MockDrawLine;
      _lines = new List<Tuple<Vector3, Vector3>>();
    }

    [TearDown]
    public void TearDown() {
      MjGizmos.DrawLine = Gizmos.DrawLine;
      _lines.Clear();
    }

    private List<Tuple<Vector3, Vector3>> _lines;

    private void MockDrawLine(Vector3 start, Vector3 end) {
      _lines.Add(Tuple.Create(start, end));
    }

    [Test]
    public void DrawingPlaneAroundArbitraryCenter() {
      MjGizmos.DrawWirePlane(new Vector3(1, 2, 3), 1, 1);
      Assert.That(_lines, Has.Count.EqualTo(5));
      Assert.That(_lines[0].Item1, Is.EqualTo(new Vector3(0.5f, 2, 2.5f)));
      Assert.That(_lines[0].Item2, Is.EqualTo(new Vector3(1.5f, 2, 2.5f)));
      Assert.That(_lines[1].Item1, Is.EqualTo(new Vector3(1.5f, 2, 2.5f)));
      Assert.That(_lines[1].Item2, Is.EqualTo(new Vector3(1.5f, 2, 3.5f)));
      Assert.That(_lines[2].Item1, Is.EqualTo(new Vector3(1.5f, 2, 3.5f)));
      Assert.That(_lines[2].Item2, Is.EqualTo(new Vector3(0.5f, 2, 3.5f)));
      Assert.That(_lines[3].Item1, Is.EqualTo(new Vector3(0.5f, 2, 3.5f)));
      Assert.That(_lines[3].Item2, Is.EqualTo(new Vector3(0.5f, 2, 2.5f)));
      Assert.That(_lines[4].Item1, Is.EqualTo(new Vector3(0.5f, 2, 2.5f)));
      Assert.That(_lines[4].Item2, Is.EqualTo(new Vector3(1.5f, 2, 3.5f)));
    }

    [Test]
    public void DrawingPlaneWithCorrectDimensions() {
      MjGizmos.DrawWirePlane(Vector3.zero, width: 2, height: 4);
      Assert.That(_lines[0].Item1, Is.EqualTo(new Vector3(-1, 0, -2)));
      Assert.That(_lines[0].Item2, Is.EqualTo(new Vector3(1, 0, -2)));
      Assert.That(_lines[1].Item1, Is.EqualTo(new Vector3(1, 0, -2)));
      Assert.That(_lines[1].Item2, Is.EqualTo(new Vector3(1, 0, 2)));
      Assert.That(_lines[2].Item1, Is.EqualTo(new Vector3(1, 0, 2)));
      Assert.That(_lines[2].Item2, Is.EqualTo(new Vector3(-1, 0, 2)));
      Assert.That(_lines[3].Item1, Is.EqualTo(new Vector3(-1, 0, 2)));
      Assert.That(_lines[3].Item2, Is.EqualTo(new Vector3(-1, 0, -2)));
      Assert.That(_lines[4].Item1, Is.EqualTo(new Vector3(-1, 0, -2)));
      Assert.That(_lines[4].Item2, Is.EqualTo(new Vector3(1, 0, 2)));
    }
  }
}
