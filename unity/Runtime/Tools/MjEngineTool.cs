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
using System.IO;
using System.Text;
using System.Xml;
using UnityEngine;

namespace Mujoco {

// Mujoco engine helper methods.
public static class MjEngineTool {
  private const int _elementsPerPosition = 3;
  private const int _elementsPerRotation = 4;
  private const int _elementsPerTransform = 7;
  private const int _elementsPerEquality = 11;
  private static double[] _mjQuat = new double[4];
  private static double[] _mjMat = new double[9];

  public static string Sanitize(string name) {
    return name.Replace('/', '_');
  }

  public static string MakeLocaleInvariant(FormattableString interpolated) {
    return FormattableString.Invariant(interpolated);
  }

  // Loads a model from the specified file.
  // `filename` should contain an absolute path to the file.
  public static unsafe MujocoLib.mjModel_* LoadModelFromFile(string fileName) {
    var errorBuf = new StringBuilder(1024);
    MujocoLib.mjModel_* model = MujocoLib.mj_loadXML(fileName, null, errorBuf, 1024);
    if (model == null || errorBuf.Length > 0) {
      throw new IOException(string.Format("Error loading the model: {0}", errorBuf));
    }
    return model;
  }

  // Saves a model to the specified file.
  // `filename` should contain an absolute path to the file.
  public static unsafe void SaveModelToFile(string fileName, MujocoLib.mjModel_* model) {
    var errorBuf = new StringBuilder(1024);
    MujocoLib.mj_saveLastXML(fileName, model, errorBuf, 1024);
    if (errorBuf.Length > 0) {
      throw new IOException(string.Format("Error saving the model: {0}", errorBuf));
    }
  }

  // Loads a model from a string.
  // It allows to perform diskless instantiation of Mujoco models.
  public static unsafe MujocoLib.mjModel_* LoadModelFromString(string contents) {
    using (var vfs = new MjVfs()) {
      var filename = "filename";
      vfs.AddFile(filename, contents);
      return vfs.LoadXML(filename);
    }
  }

  // Stores a unity vector in a Mujoco target buffer.
  public static unsafe void SetMjVector3(double* mjTarget, Vector3 unityVec) {
    var mjVec = MjVector3(unityVec);
    mjTarget[0] = mjVec[0];
    mjTarget[1] = mjVec[1];
    mjTarget[2] = mjVec[2];
  }

  // Stores a unity quaternion in a Mujoco target buffer.
  public static unsafe void SetMjQuaternion(double* mjTarget, Quaternion unityQuat) {
    var mjQuat = MjQuaternion(unityQuat);
    mjTarget[0] = mjQuat.w;
    mjTarget[1] = mjQuat.x;
    mjTarget[2] = mjQuat.y;
    mjTarget[3] = mjQuat.z;
  }

  // Returns a pointer to an entry in the MuJoCo field that's 3*offsetEntry down the buffer.
  public static unsafe double* MjVector3AtEntry(double* mjTarget, int offsetEntry) {
    return mjTarget + offsetEntry * _elementsPerPosition;
  }

  // Returns a pointer to an entry in the MuJoCo field that's 4*offsetEntry down the buffer.
  public static unsafe double* MjQuaternionAtEntry(double* mjTarget, int offsetEntry) {
    return mjTarget + offsetEntry * _elementsPerRotation;
  }

  // Returns a pointer to an entry in the MuJoCo field that's 7*offsetEntry down the buffer.
  public static unsafe double* MjTransformAtEntry(double* mjTarget, int offsetEntry) {
    return mjTarget + offsetEntry * _elementsPerTransform;
  }

  // Returns a pointer to an entry in the MuJoCo field that's 9*offsetEntry down the buffer.
  public static unsafe double* MjMatrixAtEntry(double* mjTarget, int offsetEntry) {
    return mjTarget + offsetEntry * _mjMat.Length;
  }

  // Returns a pointer to an entry in the MuJoCo field that's 11*offsetEntry down the buffer.
  public static unsafe double* MjEqualityAtEntry(double* mjTarget, int offsetEntry) {
    return mjTarget + offsetEntry * _elementsPerEquality;
  }

  // Stores a unity transform (position+rotation) in a Mujoco target buffer.
  public static unsafe void SetMjTransform(
      double* mjTarget, Vector3 unityVec, Quaternion unityQuat) {
    var mjVec = MjVector3(unityVec);
    var mjQuat = MjQuaternion(unityQuat);
    mjTarget[0] = mjVec[0];
    mjTarget[1] = mjVec[1];
    mjTarget[2] = mjVec[2];
    mjTarget[3] = mjQuat.w;
    mjTarget[4] = mjQuat.x;
    mjTarget[5] = mjQuat.y;
    mjTarget[6] = mjQuat.z;
  }

  // Converts a Unity Vector3 to a Mujoco vector.
  public static Vector3 MjVector3(Vector3 unityVec) {
    return new Vector3(unityVec.x, unityVec.z, unityVec.y);
  }

  // Converts a Mujoco vector to a Unity vector.
  public static unsafe Vector3 UnityVector3(double* mjVector) {
    return new Vector3((float)mjVector[0], (float)mjVector[2], (float)mjVector[1]);
  }

  // Converts a Mujoco vector to a Unity vector.
  // The vector coordinates are expected to be stored at the 'entryIndex * 3' of the 'coords'
  // array.
  public static unsafe Vector3 UnityVector3(float[] coords, int entryIndex) {
    var startOffset = entryIndex * _elementsPerPosition;
    return new Vector3(coords[startOffset], coords[startOffset + 2], coords[startOffset + 1]);
  }

  // Converts a Mujoco vector to a Unity vector.
  public static Vector3 UnityVector3(Vector3 mjVector) {
    return new Vector3(mjVector.x, mjVector.z, mjVector.y);
  }

  // Converts a Unity Quaternion to a Mujoco Quaternion
  public static Quaternion MjQuaternion(Quaternion unityQuat) {
    return new Quaternion(w:-unityQuat.w, x:unityQuat.x, y:unityQuat.z, z:unityQuat.y);
  }

  // Converts a Mujoco quaternion to a Unity quaternion.
  public static unsafe Quaternion UnityQuaternion(double* mjQuat) {
    return new Quaternion(
        x:(float)mjQuat[1], y:(float)mjQuat[3], z:(float)mjQuat[2], w:(float)-mjQuat[0]);
  }

  // Converts a Mujoco quaternion to a Unity quaternion.
  public static Quaternion UnityQuaternion(Quaternion mjQuat) {
    // This is simply the inverse permutation of the one in MjQuaternion
    return new Quaternion(x:mjQuat.x, y:mjQuat.z, z:mjQuat.y, w:-mjQuat.w);
  }

  // Converts a Mujoco matrix to a Unity quaternion.
  // The matrix coordinates are at the 'entryIndex * 9' of the 'mjMat' array.
  public static unsafe Quaternion UnityQuaternionFromMatrix(double* mjMat) {
    for (var j = 0; j < _mjMat.Length; ++j) {
      _mjMat[j] = mjMat[j];
    }
    fixed (double* q_out = _mjQuat)
    fixed (double* m_in = _mjMat) {
      MujocoLib.mju_mat2Quat(q_out, m_in);
      return UnityQuaternion(q_out);
    }
  }

  // Converts a Unity extents Vector3 to a Mujoco extents vector.
  // Use to convert size vectors that should contain only absolute values.
  public static Vector3 MjExtents(Vector3 unityExtents) {
    return new Vector3(unityExtents.x, unityExtents.z, unityExtents.y);
  }

  // Converts a Mujoco extents vector to a Unity extents Vector3.
  // Use to convert size vectors that should contain only absolute values.
  public static Vector3 UnityExtents(Vector3 mjExtents) {
    return new Vector3(mjExtents.x, mjExtents.z, mjExtents.y);
  }

  // Converts a vector containing Mujoco Euler angles to a Unity representation.
  public static Vector3 UnityEuler(Vector3 mjEuler) {
    return new Vector3(mjEuler.x, mjEuler.z, -mjEuler.y);
  }

  public static string Vector3ToMjcf(Vector3 vec) {
    return MakeLocaleInvariant($"{vec.x} {vec.y} {vec.z}");
  }

  public static string QuaternionToMjcf(Quaternion quat) {
    return MakeLocaleInvariant($"{quat.w} {quat.x} {quat.y} {quat.z}");
  }

  // We can't use transform.localPosition because we allow arbitrary deep gameObject hierarchies
  // between MuJoCo parent and child entities.  Therefore, the method takes into account extra
  // transforms that may be present in the chain separating the component from its parent body.
  public static MjTransformation LocalTransformInParentBody(MjComponent component) {
    var MjParent = MjHierarchyTool.FindParentComponent<MjBaseBody>(component);
    var parentGlobalPosition = Vector3.zero;
    var parentGlobalRotation = Quaternion.identity;
    if (MjParent != null) {
      parentGlobalPosition = MjParent.transform.position;
      parentGlobalRotation = MjParent.transform.rotation;
    }

    var invParentGlobalRotation = Quaternion.Inverse(parentGlobalRotation);

    var localPosition =
        invParentGlobalRotation * (component.transform.position - parentGlobalPosition);
    var localRotation = invParentGlobalRotation * component.transform.rotation;
    return new MjTransformation(localPosition, localRotation);
  }

  // Identity quaternion in MujocoLib.
  public static readonly Quaternion MjQuaternionIdentity = new Quaternion(w:-1, x:0, y:0, z:0);

  // Mujoco's Z-axis corresponds to Unity's up (Y) axis.
  public static readonly Vector3 MjVector3Up = new Vector3(0, 0, 1);

  // Sorts the components of a Vector2. Doesn't modify the original vector.
  public static Vector2 GetSorted(Vector2 vec) {
    return new Vector2(Math.Min(vec.x, vec.y), Math.Max(vec.x, vec.y));
  }

  // Converts an array of floats to an Mjcf.
  public static string ArrayToMjcf(float[] array) {
    String ret = "";
    foreach (float entry in array) {
      ret += MakeLocaleInvariant($"{entry} ");
    }
    return ret.Substring(startIndex:0, length:ret.Length - 1);
  }

  // Converts a list of floats to an Mjcf.
  public static string ListToMjcf(List<float> list) {
    String ret = "";
    foreach (float entry in list) {
      ret += MakeLocaleInvariant($"{entry} ");
    }
    return ret.Substring(startIndex:0, length:ret.Length - 1);
  }

  // Generates an Mjcf of the specified component's transform.
  //
  // It will use the component's position (Vector3) and rotation (Quaternion), relative to its
  // MjComponent parent, and save them in "pos" and "quat" attributes.
  //
  // The method takes into account extra transforms that may be present in the chain separating
  // the component from its MjComponent parent.
  public static void PositionRotationToMjcf(XmlElement mjcf, MjComponent component) {
    var localTransform = LocalTransformInParentBody(component);
    mjcf.SetAttribute(
        "pos",
        MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(localTransform.Translation)));
    mjcf.SetAttribute(
        "quat",
        MjEngineTool.QuaternionToMjcf(MjEngineTool.MjQuaternion(localTransform.Rotation)));
  }

  // Generates an Mjcf of the specified component's transform.
  //
  // It will use the component's position (Vector3) and forward direction (Vector3), relative to
  // its MjComponent parent, and save them in "pos" and "axis" attributes. Designed to work
  // with Joints, it will set the "ref" attribute to 0, indicating that the joint is in its
  // reference pose frame.
  //
  // The method takes into account extra transforms that may be present in the chain separating
  // the component from its MjComponent parent.
  public static void PositionAxisToMjcf(XmlElement mjcf, MjComponent component) {
    var localTransform = LocalTransformInParentBody(component);
    var axis = (localTransform.Rotation * Vector3.right).normalized;

    mjcf.SetAttribute(
        "pos",
        MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(localTransform.Translation)));
    mjcf.SetAttribute("axis", MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(axis)));
    mjcf.SetAttribute("ref", "0");
  }

  public static void PositionToMjcf(XmlElement mjcf, MjComponent component) {
    var localTransform = LocalTransformInParentBody(component);
    mjcf.SetAttribute(
        "pos",
        MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(localTransform.Translation)));
  }

  private const int _fromOffset = 0;
  private const int _toOffset = 1;
  private const float _fromToValidityTolerance = 1e-3f;

  public static bool ParseFromToMjcf(XmlElement mjcf, out Vector3 fromPoint, out Vector3 toPoint) {
    var fromto = mjcf.GetFloatArrayAttribute("fromto", defaultValue: null);
    if (fromto != null) {
      fromPoint = MjEngineTool.UnityVector3(fromto, _fromOffset);
      toPoint = MjEngineTool.UnityVector3(fromto, _toOffset);
      var nodeName = mjcf.GetStringAttribute("name", mjcf.Name);
      if ((toPoint - fromPoint).magnitude < _fromToValidityTolerance) {
        throw new ArgumentException(
            $"{nodeName}: 'fromto' produces a vector that's too short. {fromto} has magnitude " +
            "{fromto.magnitude} lower than the tolerance {_fromToValidityTolerance}");
      }
      return true;
    } else {
      fromPoint = Vector3.zero;
      toPoint = Vector3.zero;
      return false;
    }
  }

  // Parses the transform-shape specifications, shared by the Geom and Site Mjcfs.
  public static void ParseTransformMjcf(XmlElement mjcf, Transform transform) {
    Vector3 fromPoint, toPoint;
    if (ParseFromToMjcf(mjcf, out fromPoint, out toPoint)) {
      transform.localPosition = Vector3.Lerp(toPoint, fromPoint, 0.5f);
      transform.localRotation = Quaternion.FromToRotation(
          Vector3.up, (toPoint - fromPoint).normalized);
    } else {
      // Parse the regular transform.
      transform.localPosition = MjEngineTool.UnityVector3(
          mjcf.GetVector3Attribute("pos", defaultValue: Vector3.zero));
      transform.localRotation = Quaternion.identity;

      if (mjcf.HasAttribute("quat")) {
        transform.localRotation = MjEngineTool.UnityQuaternion(
            mjcf.GetQuaternionAttribute("quat",
                                        defaultValue: MjEngineTool.MjQuaternionIdentity));
      } else if (mjcf.HasAttribute("zaxis")) {
        var upAxis = MjEngineTool.UnityVector3(
            mjcf.GetVector3Attribute("zaxis", defaultValue: MjEngineTool.MjVector3Up));
        transform.localRotation = Quaternion.FromToRotation(
            Vector3.up, upAxis);
      } else if (mjcf.HasAttribute("xyaxes")) {
        var xyaxes = mjcf.GetFloatArrayAttribute(
            "xyaxes", defaultValue: new float[] { 1, 0, 0, 0, 1, 0 });
        var xAxis = MjEngineTool.UnityVector3(xyaxes, 0);
        var yAxis = MjEngineTool.UnityVector3(xyaxes, 1);
        var zAxis = Vector3.Cross(xAxis, yAxis);
        transform.localRotation = Quaternion.LookRotation(zAxis, yAxis);
      } else if (mjcf.HasAttribute("axisangle")) {
        var axisAngle = mjcf.GetFloatArrayAttribute(
            "axisangle", defaultValue: new float[] { 0, 0, 1, 0 });
        var axis = MjEngineTool.UnityVector3(axisAngle, 0);
        var angle = axisAngle[3];
        if (MjSceneImportSettings.AnglesInDegrees == false) {
          angle *= Mathf.Rad2Deg;
        }
        transform.localRotation = Quaternion.AngleAxis(angle, axis);
      } else if (mjcf.HasAttribute("euler")) {
        var euler = MjEngineTool.UnityEuler(
            mjcf.GetVector3Attribute("euler", defaultValue: Vector3.zero));
        if (MjSceneImportSettings.AnglesInDegrees == false) {
          euler = euler * Mathf.Rad2Deg;
        }
        transform.localRotation = Quaternion.Euler(euler);
      }
    }
  }

  public static void LoadPlugins() {
    if(!Directory.Exists("Packages/org.mujoco/Runtime/Plugins/")) return;

    foreach (string pluginPath in Directory.GetFiles("Packages/org.mujoco/Runtime/Plugins/")) {
      MujocoLib.mj_loadPluginLibrary(pluginPath);
    }

  }

}

public static class MjSceneImportSettings {
  public static bool AnglesInDegrees = true;
}
}
