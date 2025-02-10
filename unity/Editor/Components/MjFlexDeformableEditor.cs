using UnityEngine;
using UnityEditor;

namespace Mujoco {
  [CustomEditor(typeof(MjFlexDeformable))]
  public class MjFlexDeformableEditor : Editor {
    public override void OnInspectorGUI() {
      serializedObject.Update();

      // To have properly rendered, dynamic edge/elasticity/contact properties with dropdowns
      // without manually creating the dropdown, HideInInspector in the main class doesn't work,
      // would end up needing to manually create the dropdown views. That would let us simply call
      // the default inspector for the other fields, but instead let's just add the boilerplate
      // them.
      
      EditorGUILayout.PropertyField(serializedObject.FindProperty("FlexName"), true);
      EditorGUILayout.PropertyField(serializedObject.FindProperty("Dim"), true);
      EditorGUILayout.PropertyField(serializedObject.FindProperty("Radius"), true);
      EditorGUILayout.PropertyField(serializedObject.FindProperty("Body"), true);
      EditorGUILayout.PropertyField(serializedObject.FindProperty("Vertex"), true);
      EditorGUILayout.PropertyField(serializedObject.FindProperty("Texcoord"), true);
      EditorGUILayout.PropertyField(serializedObject.FindProperty("Element"), true);
      EditorGUILayout.PropertyField(serializedObject.FindProperty("Flatskin"), true);
      EditorGUILayout.PropertyField(serializedObject.FindProperty("Group"), true);
      var component = (MjFlexDeformable)target;

      EditorGUILayout.Space(10);
      EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

      // Draw our hidden configuration sections
      SerializedProperty configureEdge = serializedObject.FindProperty("ConfigureEdge");
      SerializedProperty configureContact = serializedObject.FindProperty("ConfigureContact");
      SerializedProperty configureElasticity = serializedObject.FindProperty("ConfigureElasticity");

      // Edge Configuration
      EditorGUILayout.PropertyField(configureEdge);
      if (configureEdge.boolValue) {
        EditorGUI.indentLevel++;
        var edge = serializedObject.FindProperty("Edge");
        EditorGUILayout.PropertyField(edge, true);
        EditorGUI.indentLevel--;
        EditorGUILayout.Space(5);
      }

      // Contact Configuration
      EditorGUILayout.PropertyField(configureContact);
      if (configureContact.boolValue) {
        EditorGUI.indentLevel++;
        var contact = serializedObject.FindProperty("Contact");
        EditorGUILayout.PropertyField(contact, true);
        EditorGUI.indentLevel--;
        EditorGUILayout.Space(5);
      }

      // Elasticity Configuration
      EditorGUILayout.PropertyField(configureElasticity);
      if (configureElasticity.boolValue) {
        EditorGUI.indentLevel++;
        var elasticity = serializedObject.FindProperty("Elasticity");
        EditorGUILayout.PropertyField(elasticity, true);
        EditorGUI.indentLevel--;
      }

      serializedObject.ApplyModifiedProperties();
    }
  }
}