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
using System.IO;
using System.Runtime.InteropServices;
using System.Xml;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

namespace Mujoco {

  [TestFixture]
  public class MjEngineToolXmlLoadingTests {

    [Test]
    public unsafe void LoadingSceneFromAProvidedAsset() {
      var modelFile = Resources.Load<TextAsset>("ValidModel");
      var modelPtr = MjEngineTool.LoadModelFromString(modelFile.text);
      var model =
          (MujocoLib.mjModel_)Marshal.PtrToStructure(new IntPtr(modelPtr), typeof(MujocoLib.mjModel_));
      Assert.That(model.nbody, Is.EqualTo(3));
    }
  }

  [TestFixture]
  public class MjEngineToolTransformHelpersTests {
    [TestCase(1, 2, 3)]
    [TestCase(-1, -2, -3)]
    [TestCase(-1, 2, 3)]
    [TestCase(1, -2, 3)]
    [TestCase(1, 2, -3)]
    public unsafe void RoundrobinConversionOfVector3(float x, float y, float z) {
      var vec = new Vector3(x, y, z);
      var MjVec = MjEngineTool.MjVector3(vec);
      var MjVecAsArray = new double[] { 10, 11, 12, MjVec.x, MjVec.y, MjVec.z };
      fixed (double* MjArrPtr = MjVecAsArray) {
        var recreatedVec = MjEngineTool.UnityVector3(
            MjEngineTool.MjVector3AtEntry(MjArrPtr, 1));
        Assert.That(recreatedVec, Is.EqualTo(vec));
      }
    }

    [TestCase(1, 2, 3)]
    [TestCase(-1, -2, -3)]
    [TestCase(-1, 2, 3)]
    [TestCase(1, -2, 3)]
    [TestCase(1, 2, -3)]
    public void RoundrobinConversionOfVector3UsingCoreType(float x, float y, float z) {
      var vec = new Vector3(x, y, z);
      var MjVec = MjEngineTool.MjVector3(vec);
      var unityVec = MjEngineTool.UnityVector3(MjVec);
      Assert.That(unityVec, Is.EqualTo(vec));
    }

    [TestCase(0.1f, 0.2f, 0.3f, 0.4f)]
    [TestCase(-0.1f, -0.2f, -0.3f, -0.4f)]
    [TestCase(-0.1f, 0.2f, 0.3f, 0.4f)]
    [TestCase(0.1f, -0.2f, 0.3f, 0.4f)]
    [TestCase(0.1f, 0.2f, -0.3f, 0.4f)]
    [TestCase(0.1f, 0.2f, 0.3f, -0.4f)]
    public unsafe void RoundrobinConversionOfQuaternionUsingUnsafeArrays(float x, float y, float z,
                                                                         float w) {
      var quat = new Quaternion(x, y, z, w);
      var MjQuat = MjEngineTool.MjQuaternion(quat);
      var MjQuatAsArray =
          new double[] { 10, 20, 30, 40, MjQuat.w, MjQuat.x, MjQuat.y, MjQuat.z };
      fixed (double* MjArrPtr = MjQuatAsArray) {
        var recreatedQuat = MjEngineTool.UnityQuaternion(
            MjEngineTool.MjQuaternionAtEntry(MjArrPtr, 1));
        var q1 = new Vector4(quat.x, quat.y, quat.z, quat.w);
        var q2 = new Vector4(recreatedQuat.x, recreatedQuat.y, recreatedQuat.z, recreatedQuat.w);
        Assert.That(q1, Is.EqualTo(q2));
      }
    }

    [TestCase(0.1f, 0.2f, 0.3f, 0.4f)]
    [TestCase(-0.1f, -0.2f, -0.3f, -0.4f)]
    [TestCase(-0.1f, 0.2f, 0.3f, 0.4f)]
    [TestCase(0.1f, -0.2f, 0.3f, 0.4f)]
    [TestCase(0.1f, 0.2f, -0.3f, 0.4f)]
    [TestCase(0.1f, 0.2f, 0.3f, -0.4f)]
    public void RoundrobinConversionOfQuaternionUsingCoreType(float x, float y, float z, float w) {
      var quat = new Quaternion(x, y, z, w);
      var MjQuat = MjEngineTool.MjQuaternion(quat);
      var unityQuat = MjEngineTool.UnityQuaternion(MjQuat);
      Assert.That(unityQuat, Is.EqualTo(quat));
    }

    [TestCase(1, 2, 3)]
    [TestCase(-1, -2, -3)]
    [TestCase(-1, 2, 3)]
    [TestCase(1, -2, 3)]
    [TestCase(1, 2, -3)]
    public void RoundrobinConversionOfExtents(float x, float y, float z) {
      var extents = new Vector3(x, y, z);
      var MjExtents = MjEngineTool.MjExtents(extents);
      var unityExtents = MjEngineTool.UnityExtents(MjExtents);
      Assert.That(MjExtents, Is.Not.EqualTo(extents));
      Assert.That(unityExtents, Is.EqualTo(extents));
    }

    [TestCase(0.1f, 0.2f, 0.3f, 0.4f)]
    [TestCase(-0.1f, -0.2f, -0.3f, -0.4f)]
    [TestCase(-0.1f, 0.2f, 0.3f, 0.4f)]
    [TestCase(0.1f, -0.2f, 0.3f, 0.4f)]
    [TestCase(0.1f, 0.2f, -0.3f, 0.4f)]
    [TestCase(0.1f, 0.2f, 0.3f, -0.4f)]
    public unsafe void SettingAndRetrievingQuaternions(float x, float y, float z, float w) {
      var quat = new Quaternion(x, y, z, w);
      var result = Quaternion.identity;
      var buffer = new double[4];
      fixed (double* unsafeBuffer = buffer) {
        MjEngineTool.SetMjQuaternion(unsafeBuffer, quat);
        result = MjEngineTool.UnityQuaternion(unsafeBuffer);
      }
      Assert.That(quat, Is.EqualTo(result));
    }

    [TestCase(1, 2, 3)]
    [TestCase(-1, -2, -3)]
    [TestCase(-1, 2, 3)]
    [TestCase(1, -2, 3)]
    [TestCase(1, 2, -3)]
    public unsafe void SettingAndRetrievingVectors(float x, float y, float z) {
      var vec = new Vector3(x, y, z);
      var result = Vector3.zero;
      double[] buffer = new double[3];
      fixed (double* unsafeBuffer = buffer) {
        MjEngineTool.SetMjVector3(unsafeBuffer, vec);
        result = MjEngineTool.UnityVector3(unsafeBuffer);
      }
      Assert.That(vec, Is.EqualTo(result));
    }
  }

  [TestFixture]
  public class MjEngineToolTransformSerializationTests {
    public class FakeMjComponent : MjComponent {
      public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_GEOM;

      protected override void OnParseMjcf(XmlElement mjcf) {}

      protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
        return doc.CreateElement("geom");
      }
    }

    private GameObject _rootObject;
    private GameObject _intermediateObject;
    private FakeMjComponent _component;
    private Vector3EqualityComparer _vectorComparer;
    private Vector4EqualityComparer _quaternionComparer;

    [SetUp]
    public void SetUp() {
      _rootObject = new GameObject("rootObject");
      _intermediateObject = new GameObject("intermediateObject");
      _component = new GameObject("component").AddComponent<FakeMjComponent>();
      var epsilon = 1e-3f;
      _vectorComparer = new Vector3EqualityComparer(epsilon);
      _quaternionComparer = new Vector4EqualityComparer(epsilon);
    }

    [TearDown]
    public void TearDown() {
      UnityEngine.Object.DestroyImmediate(_component.gameObject);
      UnityEngine.Object.DestroyImmediate(_intermediateObject);
      UnityEngine.Object.DestroyImmediate(_rootObject);
      MjSceneImportSettings.AnglesInDegrees = true;
    }

    [Test]
    public void SerializingPositionRotationOfRootComponent() {
      _component.transform.position = new Vector3(1, 2, 3);
      _component.transform.rotation = Quaternion.AngleAxis(45, Vector3.up);
      var doc = new XmlDocument();
      var mjcf = doc.CreateElement("element");
      MjEngineTool.PositionRotationToMjcf(mjcf, _component);
      Assert.That(mjcf.GetAttribute("pos"), Is.EqualTo("1 3 2"));
      Assert.That(mjcf.GetQuaternionAttribute("quat", Quaternion.identity),
                  Is.EqualTo(new Quaternion(w:-0.9238795f, x:0, y:0, z:0.3826835f))
                  .Using(_quaternionComparer));
    }

    [Test]
    public void SerializingPositionRotationOfRootComponentParentedToGameObject() {
      _component.transform.parent = _rootObject.transform;
      _rootObject.transform.position = new Vector3(1, 2, 3);
      _rootObject.transform.rotation = Quaternion.AngleAxis(45, Vector3.up);
      var doc = new XmlDocument();
      var mjcf = doc.CreateElement("element");
      MjEngineTool.PositionRotationToMjcf(mjcf, _component);
      Assert.That(mjcf.GetAttribute("pos"), Is.EqualTo("1 3 2"));
      Assert.That(mjcf.GetQuaternionAttribute("quat", Quaternion.identity),
                  Is.EqualTo(new Quaternion(w:-0.9238795f, x:0, y:0, z:0.3826835f))
                  .Using(_quaternionComparer));
    }

    [Test]
    public void SerializingPositionRotationOfChildComponentParentedThroughGameObject() {
      _component.transform.parent = _intermediateObject.transform;
      _intermediateObject.transform.parent = _rootObject.transform;
      _intermediateObject.transform.position = new Vector3(1, 2, 3);
      _intermediateObject.transform.rotation = Quaternion.AngleAxis(45, Vector3.up);
      var doc = new XmlDocument();
      var mjcf = doc.CreateElement("element");
      MjEngineTool.PositionRotationToMjcf(mjcf, _component);
      Assert.That(mjcf.GetAttribute("pos"), Is.EqualTo("1 3 2"));
      Assert.That(mjcf.GetQuaternionAttribute("quat", Quaternion.identity),
                  Is.EqualTo(new Quaternion(w:-0.9238795f, x:0, y:0, z:0.3826835f))
                  .Using(_quaternionComparer));
    }

    [Test]
    public void SerializingPositionAxisOfRootComponent() {
      _component.transform.position = new Vector3(1, 2, 3);
      _component.transform.rotation = Quaternion.AngleAxis(45, Vector3.up);
      var doc = new XmlDocument();
      var mjcf = doc.CreateElement("element");
      MjEngineTool.PositionAxisToMjcf(mjcf, _component);
      Assert.That(mjcf.GetAttribute("pos"), Is.EqualTo("1 3 2"));
      Assert.That(mjcf.GetVector3Attribute("axis", Vector3.zero),
                  Is.EqualTo(new Vector3(0.7071068f, -0.7071069f, 0)).Using(_vectorComparer));
      Assert.That(mjcf.GetAttribute("ref"), Is.EqualTo("0"));
    }

    [Test]
    public void SerializingPositionAxisOfRootComponentParentedToGameObject() {
      _component.transform.parent = _rootObject.transform;
      _rootObject.transform.position = new Vector3(1, 2, 3);
      _rootObject.transform.rotation = Quaternion.AngleAxis(45, Vector3.up);
      var doc = new XmlDocument();
      var mjcf = doc.CreateElement("element");
      MjEngineTool.PositionAxisToMjcf(mjcf, _component);
      Assert.That(mjcf.GetAttribute("pos"), Is.EqualTo("1 3 2"));
      Assert.That(mjcf.GetVector3Attribute("axis", Vector3.zero),
                  Is.EqualTo(new Vector3(0.7071068f, -0.7071069f, 0)).Using(_vectorComparer));
      Assert.That(mjcf.GetAttribute("ref"), Is.EqualTo("0"));
    }

    [Test]
    public void SerializingPositionAxisOfChildComponentParentedThroughGameObject() {
      _component.transform.parent = _intermediateObject.transform;
      _intermediateObject.transform.parent = _rootObject.transform;
      _intermediateObject.transform.position = new Vector3(1, 2, 3);
      _intermediateObject.transform.rotation = Quaternion.AngleAxis(45, Vector3.up);
      var doc = new XmlDocument();
      var mjcf = doc.CreateElement("element");
      MjEngineTool.PositionAxisToMjcf(mjcf, _component);
      Assert.That(mjcf.GetAttribute("pos"), Is.EqualTo("1 3 2"));
      Assert.That(mjcf.GetVector3Attribute("axis", Vector3.zero),
                  Is.EqualTo(new Vector3(0.7071068f, -0.7071069f, 0)).Using(_vectorComparer));
      Assert.That(mjcf.GetAttribute("ref"), Is.EqualTo("0"));
    }

    [Test]
    public void ParsePosition() {
      var mjcf = new XmlDocument().CreateElement("element");
      mjcf.SetAttribute("pos", "1 3 2");
      MjEngineTool.ParseTransformMjcf(mjcf, _rootObject.transform);
      Assert.That(_rootObject.transform.position, Is.EqualTo(new Vector3(1, 2, 3)));
    }

    [Test]
    public void ParseRotationFromQuat() {
      var mjcf = new XmlDocument().CreateElement("element");
      mjcf.SetAttribute("quat", "-0.9238795 0 0 0.3826835");
      MjEngineTool.ParseTransformMjcf(mjcf, _rootObject.transform);
      var expectedRotation = Quaternion.AngleAxis(45, Vector3.up);
      Assert.That(_rootObject.transform.rotation.x, Is.EqualTo(expectedRotation.x).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.y, Is.EqualTo(expectedRotation.y).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.z, Is.EqualTo(expectedRotation.z).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.w, Is.EqualTo(expectedRotation.w).Within(1e-5f));
      // Assert.That(
      //     _rootObject.transform.rotation,
      //     Is.EqualTo(Quaternion.AngleAxis(45, Vector3.up))
      //     .Using(_quaternionComparer));
    }

    [Test]
    public void ParseRotationFromZAxis() {
      var mjcf = new XmlDocument().CreateElement("element");
      mjcf.SetAttribute("zaxis", "0 1 0");
      MjEngineTool.ParseTransformMjcf(mjcf, _rootObject.transform);
      var expectedRotation = Quaternion.AngleAxis(90, Vector3.right);
      Assert.That(_rootObject.transform.rotation.x, Is.EqualTo(expectedRotation.x).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.y, Is.EqualTo(expectedRotation.y).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.z, Is.EqualTo(expectedRotation.z).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.w, Is.EqualTo(expectedRotation.w).Within(1e-5f));
      // Assert.That(
      //     _rootObject.transform.rotation,
      //     Is.EqualTo(Quaternion.AngleAxis(90, Vector3.right))
      //     .Using(_quaternionComparer));
    }

    [TestCase(true, 45, 45)]
    [TestCase(false, 0.785398f, 45)]
    [TestCase(true, 90, 90)]
    [TestCase(false, 1.570796f, 90)]
    public void ParseRotationFromAxisAngle(bool useDegrees, float MjAngle,
                                           float expectedAngle) {
      MjSceneImportSettings.AnglesInDegrees = useDegrees;
      var mjcf = new XmlDocument().CreateElement("element");
      mjcf.SetAttribute("axisangle", $"1 0 0 {MjAngle}");
      MjEngineTool.ParseTransformMjcf(mjcf, _rootObject.transform);
      var expectedRotation = Quaternion.AngleAxis(expectedAngle, Vector3.right);
      Assert.That(_rootObject.transform.rotation.x, Is.EqualTo(expectedRotation.x).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.y, Is.EqualTo(expectedRotation.y).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.z, Is.EqualTo(expectedRotation.z).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.w, Is.EqualTo(expectedRotation.w).Within(1e-5f));
      // Assert.That(
      //     _rootObject.transform.rotation,
      //     Is.EqualTo(Quaternion.AngleAxis(expectedAngle, Vector3.right))
      //     .Using(_quaternionComparer));
    }

    [TestCase(true, 0, 45, 0, 0, 0, -45)]
    [TestCase(false, 0, 0.785398f, 0, 0, 0, -45)]
    [TestCase(true, 0, 0, 45, 0, 45, 0)]
    [TestCase(false, 0, 0, 0.785398f, 0, 45, 0)]
    [TestCase(true, 45, 0, 0, 45, 0, 0)]
    [TestCase(false, 0.785398f, 0, 0, 45, 0, 0)]
    public void ParseRotationFromEuler(bool useDegrees, float mx, float my, float mz, float ex,
                                       float ey, float ez) {
      MjSceneImportSettings.AnglesInDegrees = useDegrees;
      var mjcf = new XmlDocument().CreateElement("element");
      mjcf.SetAttribute("euler", $"{mx} {my} {mz}");
      MjEngineTool.ParseTransformMjcf(mjcf, _rootObject.transform);
      var expectedRotation = Quaternion.Euler(ex, ey, ez);
      Assert.That(_rootObject.transform.rotation.x, Is.EqualTo(expectedRotation.x).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.y, Is.EqualTo(expectedRotation.y).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.z, Is.EqualTo(expectedRotation.z).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.w, Is.EqualTo(expectedRotation.w).Within(1e-5f));
      // Assert.That(
      //     _rootObject.transform.rotation,
      //     Is.EqualTo(Quaternion.Euler(ex, ey, ez))
      //     .Using(_quaternionComparer));
    }

    [Test]
    public void ParseRotationFromDirectionVector() {
      var mjcf = new XmlDocument().CreateElement("element");
      mjcf.SetAttribute("fromto", $"0 0 0 1 1 0");
      MjEngineTool.ParseTransformMjcf(mjcf, _rootObject.transform);
      var expectedRotation = new Quaternion(0.5f, 0, -0.5f, 0.7071068f);
      Assert.That(_rootObject.transform.rotation.x, Is.EqualTo(expectedRotation.x).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.y, Is.EqualTo(expectedRotation.y).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.z, Is.EqualTo(expectedRotation.z).Within(1e-5f));
      Assert.That(_rootObject.transform.rotation.w, Is.EqualTo(expectedRotation.w).Within(1e-5f));
      // Assert.That(
      //     _rootObject.transform.rotation,
      //     Is.EqualTo(new Quaternion(0.5f, 0, -0.5f, 0.7071068f))
      //     .Using(_quaternionComparer));
    }

    [Test]
    public void ParsePositionFromDirectionVector() {
      var mjcf = new XmlDocument().CreateElement("element");
      mjcf.SetAttribute("fromto", $"1 2 3 5 7 9");
      MjEngineTool.ParseTransformMjcf(mjcf, _rootObject.transform);
      Assert.That(_rootObject.transform.position, Is.EqualTo(new Vector3(3, 6, 4.5f)));
    }
  }
}
