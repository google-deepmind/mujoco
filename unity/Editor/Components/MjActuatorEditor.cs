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

using UnityEditor;
using UnityEngine;

namespace Mujoco {

  [CustomEditor(typeof(MjActuator))]
  [CanEditMultipleObjects]
  public class MjActuatorEditor : Editor {
    private bool _showCustomParams = false;
    private bool _showDebugValues = false;
    private bool _showDynPrm = false;
    private bool _showGainPrm = false;
    private bool _showBiasValues = false;

    // General actuator properties.
    private SerializedProperty _dynType;
    private SerializedProperty _gainType;
    private SerializedProperty _biasType;
    private SerializedProperty _dynPrm;
    private SerializedProperty _gainPrm;
    private SerializedProperty _biasPrm;
    // Position actuator properties.
    private SerializedProperty _kp;
    private SerializedProperty _kvp;
    // Velocity actuator properties.
    private SerializedProperty _kv;
    // Cylinder actuator properties.
    private SerializedProperty _cylinderTimeConst;
    private SerializedProperty _area;
    private SerializedProperty _diameter;
    private SerializedProperty _bias;
    // Cylinder actuator properties.
    private SerializedProperty _muscleTimeConst;
    private SerializedProperty _range;
    private SerializedProperty _force;
    private SerializedProperty _scale;
    private SerializedProperty _lmin;
    private SerializedProperty _lmax;
    private SerializedProperty _vmax;
    private SerializedProperty _fpmax;
    private SerializedProperty _fvmax;

    public void OnEnable() {
      _showCustomParams = false;
      _showDebugValues = false;
      _showDynPrm = false;
      _showGainPrm = false;
      _showBiasValues = false;

      var customParams = serializedObject.FindProperty("CustomParams");
      // General actuator properties.
      _dynType = customParams.FindPropertyRelative("DynType");
      _gainType = customParams.FindPropertyRelative("GainType");
      _biasType = customParams.FindPropertyRelative("BiasType");
      _dynPrm = customParams.FindPropertyRelative("DynPrm");
      _gainPrm = customParams.FindPropertyRelative("GainPrm");
      _biasPrm = customParams.FindPropertyRelative("BiasPrm");
      // Position actuator properties.
      _kp = customParams.FindPropertyRelative("Kp");
      _kvp = customParams.FindPropertyRelative("Kvp");
      // Velocity actuator properties.
      _kv = customParams.FindPropertyRelative("Kv");
      // Cylinder actuator properties.
      _cylinderTimeConst = customParams.FindPropertyRelative("CylinderTimeConst");
      _area = customParams.FindPropertyRelative("Area");
      _diameter = customParams.FindPropertyRelative("Diameter");
      _bias = customParams.FindPropertyRelative("Bias");
      // Cylinder actuator properties.
      _muscleTimeConst = customParams.FindPropertyRelative("MuscleTimeConst");
      _range = customParams.FindPropertyRelative("Range");
      _force = customParams.FindPropertyRelative("Force");
      _scale = customParams.FindPropertyRelative("Scale");
      _lmin = customParams.FindPropertyRelative("LMin");
      _lmax = customParams.FindPropertyRelative("LMax");
      _vmax = customParams.FindPropertyRelative("VMax");
      _fpmax = customParams.FindPropertyRelative("FpMax");
      _fvmax = customParams.FindPropertyRelative("FvMax");
    }

    public override void OnInspectorGUI() {
      serializedObject.Update();
      foreach (var target in serializedObject.targetObjects) {
        EditActuator(target as MjActuator);
      }
      serializedObject.ApplyModifiedProperties();
    }

    private void EditActuator(MjActuator actuator) {
      DrawDefaultInspector();
      _showCustomParams = EditorGUILayout.Foldout(_showCustomParams, "Custom Params");
      if (_showCustomParams) {
        EditCustomParams(actuator.Type, actuator.CustomParams);
      }
      _showDebugValues = EditorGUILayout.Foldout(_showDebugValues, "Debug");
      if (_showDebugValues) {
        using (new EditorGUI.DisabledScope(true)) {
          EditorGUILayout.FloatField("Length", actuator.Length);
          EditorGUILayout.FloatField("Velocity", actuator.Velocity);
          EditorGUILayout.FloatField("Force", actuator.Force);
        }
      }
    }

    private void EditCustomParams(MjActuator.ActuatorType type,
                                  MjActuator.CustomParameters parameters) {
      switch (type) {
        case MjActuator.ActuatorType.General: {
          EditGeneralParams(parameters);
          break;
        }
        case MjActuator.ActuatorType.Position: {
          EditPositionParams(parameters);
          break;
        }
        case MjActuator.ActuatorType.Velocity: {
          EditVelocityParams(parameters);
          break;
        }
        case MjActuator.ActuatorType.Cylinder: {
          EditCylinderParams(parameters);
          break;
        }
        case MjActuator.ActuatorType.Muscle: {
          EditMuscleParams(parameters);
          break;
        }
      }
    }

    private void EditGeneralParams(MjActuator.CustomParameters parameters) {
      EditorGUILayout.PropertyField(_dynType);
      EditorGUILayout.PropertyField(_gainType);
      EditorGUILayout.PropertyField(_biasType);

      _showDynPrm = EditorGUILayout.Foldout(_showDynPrm, "DynPrm");
      if (_showDynPrm) {
        for (var i = 0; i < parameters.DynPrm.Count; ++i) {
          EditorGUILayout.PropertyField(_dynPrm.GetArrayElementAtIndex(i));
        }
      }
      _showGainPrm = EditorGUILayout.Foldout(_showGainPrm, "GainPrm");
      if (_showGainPrm) {
        for (var i = 0; i < parameters.GainPrm.Count; ++i) {
          EditorGUILayout.PropertyField(_gainPrm.GetArrayElementAtIndex(i));
        }
      }
      _showBiasValues = EditorGUILayout.Foldout(_showBiasValues, "BiasPrm");
      if (_showBiasValues) {
        for (var i = 0; i < parameters.BiasPrm.Count; ++i) {
          EditorGUILayout.PropertyField(_biasPrm.GetArrayElementAtIndex(i));
        }
      }
    }

    private void EditPositionParams(MjActuator.CustomParameters parameters) {
      EditorGUILayout.PropertyField(_kp);
      EditorGUILayout.PropertyField(_kvp, new GUIContent("Kv"));
    }

    private void EditVelocityParams(MjActuator.CustomParameters parameters) {
      EditorGUILayout.PropertyField(_kv);
    }

    private void EditCylinderParams(MjActuator.CustomParameters parameters) {
      EditorGUILayout.PropertyField(_cylinderTimeConst);
      EditorGUILayout.PropertyField(_area);
      EditorGUILayout.PropertyField(_diameter);
      _showBiasValues = EditorGUILayout.Foldout(_showBiasValues, "Bias");
      if (_showBiasValues) {
        for (var i = 0; i < parameters.Bias.Length; ++i) {
          EditorGUILayout.PropertyField(_bias.GetArrayElementAtIndex(i));
        }
      }
    }

    private void EditMuscleParams(MjActuator.CustomParameters parameters) {
      EditorGUILayout.PropertyField(_muscleTimeConst, new GUIContent("TimeConst"));
      EditorGUILayout.PropertyField(_range);
      EditorGUILayout.PropertyField(_force);
      EditorGUILayout.PropertyField(_scale);
      EditorGUILayout.PropertyField(_lmin);
      EditorGUILayout.PropertyField(_lmax);
      EditorGUILayout.PropertyField(_vmax);
      EditorGUILayout.PropertyField(_fpmax);
      EditorGUILayout.PropertyField(_fvmax);
    }
  }
}
