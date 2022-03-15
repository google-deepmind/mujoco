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
using System.Xml;
using UnityEngine;

namespace Mujoco {

// Functionality that needs to be implemented for each shape type.
public interface IMjShape {

  // Generate the Mjcf representation of the shape. Infer the scale of the shape from the transform.
  void ToMjcf(XmlElement mjcf, Transform transform);

  // Parse the shape settings from the specified Mjcf node.
  void FromMjcf(XmlElement mjcf);

  // Build a parametric mesh (vertices, triangles) of this shape.
  Tuple<Vector3[], int[]> BuildMesh();

  // Generate a timestamp that can be used to quickly compare if the settings of a shape
  // have changed.
  // The timestamp should also allow to distinguish between different types of shapes.
  //
  // Because most of shapes use at most 3 floating point values as parameters, we will
  // use the first 3 components of the returned Vector4 to store those values. We will store
  // the id of the shape in the 4th component. That id will be derrived from MjShapeComponent.ShapeTypes
  // enum.
  Vector4 GetChangeStamp();

  // Draw a debug gizmo for the shape.
  void DebugDraw(Transform transform);
}
}
