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
using System.Linq;
using UnityEditor;
using UnityEditor.AnimatedValues;
using UnityEngine;

namespace Mujoco {

  public static class MjHandles {

    // Renders a visualization of an axis, anchored at the specified world space origin.
    //
    // Args:
    //   origin: World space coordinates of the anchoring point for the visualization.
    //   direction: Axis direction.
    public static void Axis(Vector3 origin, Vector3 direction) {
      var size = HandleUtility.GetHandleSize(origin) * 0.3f;
      var axisEnd = origin + direction * size;
      var capRotation = Quaternion.LookRotation(direction);
      Handles.DrawLine(origin, axisEnd);
      Handles.ConeHandleCap(0, axisEnd, capRotation, size, EventType.Repaint);
    }

    // Renders a visualization of linear (translational) limits, anchored at the specified world
    // space origin.
    //
    // Args:
    //   origin: World space coordinates of the anchoring point for the visualization.
    //   lower: Distance from the origin below, where the limit expires.
    //   upper: Distance from the origin above, where the limit expires.
    //   axis: Axis along which the limit works.
    public static void LinearLimits(Vector3 origin, float lower, float upper, Vector3 axis) {
      if (upper > lower) {
        var discRadius = HandleUtility.GetHandleSize(origin) * 0.7f;
        var startPosition = origin + axis * lower;
        var endPosition = origin + axis * upper;
        Handles.DrawSolidDisc(startPosition, axis, discRadius);
        Handles.DrawSolidDisc(endPosition, axis, discRadius);
        Handles.DrawLine(origin, endPosition);
      }
    }
  }
}
