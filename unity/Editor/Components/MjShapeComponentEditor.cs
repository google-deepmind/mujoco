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
using UnityEditor;
using UnityEngine;

namespace Mujoco {

[CustomEditor(typeof(MjShapeComponent), true)]
[CanEditMultipleObjects]
public class MjShapeComponentEditor : MjMouseSpring {

  public override void OnInspectorGUI() {

    serializedObject.Update();
    var shapeType = serializedObject.FindProperty("ShapeType");
    EditorGUILayout.PropertyField(shapeType);
    EditorGUILayout.PropertyField(
        serializedObject.FindProperty($"{shapeType.enumNames[shapeType.enumValueIndex]}"),
        includeChildren: true);
    serializedObject.ApplyModifiedProperties();
  }
}
}
