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

// Utility class that represents a translation and rotation.
// Used for converting between MuJoCo and Unity spaces.
public struct MjTransformation {
  public Vector3 Translation;
  public Quaternion Rotation;

  public MjTransformation(Vector3 translation, Quaternion rotation) {
    Translation = translation;
    Rotation = rotation;
  }

  public void Set(Vector3 translation, Quaternion rotation) {
    Translation = translation;
    Rotation = rotation;
  }

  // Initializes MjTransformation from the specified Transform's local coordinates.
  public static MjTransformation LoadGlobal(Transform transform) {
    if (transform != null) {
      return new MjTransformation(transform.position, transform.rotation);
    } else {
      return new MjTransformation(Vector3.zero, Quaternion.identity);
    }
  }

  // Initializes MjTransformation from the specified Transform's global coordinates.
  public static MjTransformation LoadLocal(Transform transform) {
    if (transform != null) {
      return new MjTransformation(transform.localPosition, transform.localRotation);
    } else {
      return new MjTransformation(Vector3.zero, Quaternion.identity);
    }
  }

  // Stores the transform coordinates in the specified Transform's global coordinates.
  public void StoreGlobal(Transform transform) {
    transform.position = Translation;
    transform.rotation = Rotation;
  }

  // Stores the transform coordinates in the specified Transform's local coordinates.
  public void StoreLocal(Transform transform) {
    transform.localPosition = Translation;
    transform.localRotation = Rotation;
  }

  // Concatenates two transforms.
  // Equivalent of a matrix multiplication. This operation is not transitive.
  public static MjTransformation operator *(
      MjTransformation lhs, MjTransformation rhs) {
    return new MjTransformation(
        lhs.Translation + lhs.Rotation * rhs.Translation, lhs.Rotation * rhs.Rotation);
  }

  // Returns an inverse of this transform.
  public MjTransformation Inverse() {
    var inverseRotation = Quaternion.Inverse(Rotation);
    return new MjTransformation(inverseRotation * Translation * -1.0f, inverseRotation);
  }

  public override string ToString() {
    return $"[{Translation}, {Rotation}]";
  }
}
}
