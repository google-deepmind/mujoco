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
using System.Xml;
using NUnit.Framework;
using UnityEngine;

namespace Mujoco {
[TestFixture]
public class MjActuatorTests {
  private MjHingeJoint _joint;
  private MjActuator _actuator;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _joint.name = "my_joint";
    _actuator = new GameObject("actuator").AddComponent<MjActuator>();
    _actuator.Joint = _joint;
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_actuator.gameObject);
    UnityEngine.Object.DestroyImmediate(_joint.gameObject);
  }

  [TestCase(true, "true")]
  [TestCase(false, "false")]
  public void ControlLimitedFlagMjcf(bool value, string expected) {
    _actuator.CommonParams.CtrlLimited = value;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"ctrllimited=\"{expected}\""));
  }

  [TestCase(true, "true")]
  [TestCase(false, "false")]
  public void ForceLimitedFlagMjcf(bool value, string expected) {
    _actuator.CommonParams.ForceLimited = value;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"forcelimited=\"{expected}\""));
  }

  [TestCase(0, 1, "0 1")]
  [TestCase(1, 0, "0 1")]
  public void ControlRangeSortingForMjcf(float min, float max, string expected) {
    _actuator.CommonParams.CtrlRange = new Vector2(min, max);
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"ctrlrange=\"{expected}\""));
  }

  [TestCase(0, 1, "0 1")]
  [TestCase(1, 0, "0 1")]
  public void ForceRangeSortingForMjcf(float min, float max, string expected) {
    _actuator.CommonParams.ForceRange = new Vector2(min, max);
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"forcerange=\"{expected}\""));
  }

  [TestCase(0, 1, "0 1")]
  [TestCase(1, 0, "0 1")]
  public void LengthRangeSortingForMjcf(float min, float max, string expected) {
    _actuator.CommonParams.LengthRange = new Vector2(min, max);
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"lengthrange=\"{expected}\""));
  }

  [Test]
  public void GearMjcf() {
    _actuator.CommonParams.Gear = new List<float>() { 1, 2, 3, 4, 5, 6 };
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"gear=\"1 2 3 4 5 6\""));
  }

  [Test]
  public void JointNameMjcf() {
    _joint.GenerateMjcf("joint_name", _doc);
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"joint=\"joint_name\""));
  }

  [Test]
  public void ParseAllSettings() {
    _doc.LoadXml(
        "<general joint=\"my_joint\" ctrllimited=\"true\" forcelimited=\"true\" ctrlrange=\"2 3\" "
        + "forcerange=\"4 5\" lengthrange=\"6 7\" gear=\"8 9\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("general")[0] as XmlElement);
    Assert.That(_actuator.CommonParams.CtrlLimited, Is.True);
    Assert.That(_actuator.CommonParams.ForceLimited, Is.True);
    Assert.That(_actuator.CommonParams.CtrlRange, Is.EqualTo(new Vector2(2, 3)));
    Assert.That(_actuator.CommonParams.ForceRange, Is.EqualTo(new Vector2(4, 5)));
    Assert.That(_actuator.CommonParams.LengthRange, Is.EqualTo(new Vector2(6, 7)));
    Assert.That(_actuator.CommonParams.Gear, Has.Count.EqualTo(2));
    Assert.That(_actuator.CommonParams.Gear[0], Is.EqualTo(8));
    Assert.That(_actuator.CommonParams.Gear[1], Is.EqualTo(9));
  }

  [Test]
  public void ParseDefaultSettings() {
    _doc.LoadXml("<general joint=\"my_joint\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("general")[0] as XmlElement);
    Assert.That(_actuator.CommonParams.CtrlLimited, Is.False);
    Assert.That(_actuator.CommonParams.ForceLimited, Is.False);
    Assert.That(_actuator.CommonParams.CtrlRange, Is.EqualTo(Vector2.zero));
    Assert.That(_actuator.CommonParams.ForceRange, Is.EqualTo(Vector2.zero));
    Assert.That(_actuator.CommonParams.LengthRange, Is.EqualTo(Vector2.zero));
    Assert.That(_actuator.CommonParams.Gear, Has.Count.EqualTo(1));
    Assert.That(_actuator.CommonParams.Gear[0], Is.EqualTo(1));
  }
}

[TestFixture]
public class MjGeneralActuatorTests {
  private MjHingeJoint _joint;
  private MjActuator _actuator;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _joint.name = "my_joint";
    _actuator = new GameObject("actuator").AddComponent<MjActuator>();
    _actuator.Type = MjActuator.ActuatorType.General;
    _actuator.Joint = _joint;
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_actuator.gameObject);
    UnityEngine.Object.DestroyImmediate(_joint.gameObject);
  }

  [Test]
  public void ProperTagNameUsed() {
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<general"));
  }

  [TestCase(MujocoLib.mjtDyn.mjDYN_NONE, "none")]
  [TestCase(MujocoLib.mjtDyn.mjDYN_INTEGRATOR, "integrator")]
  [TestCase(MujocoLib.mjtDyn.mjDYN_FILTER, "filter")]
  [TestCase(MujocoLib.mjtDyn.mjDYN_MUSCLE, "muscle")]
  [TestCase(MujocoLib.mjtDyn.mjDYN_USER, "user")]
  public void DynamicsTypeMjcf(MujocoLib.mjtDyn value, string expected) {
    _actuator.CustomParams.DynType = value;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"dyntype=\"{expected}\""));
  }

  [TestCase(MujocoLib.mjtGain.mjGAIN_FIXED, "fixed")]
  [TestCase(MujocoLib.mjtGain.mjGAIN_AFFINE, "affine")]
  [TestCase(MujocoLib.mjtGain.mjGAIN_MUSCLE, "muscle")]
  [TestCase(MujocoLib.mjtGain.mjGAIN_USER, "user")]
  public void GainTypeMjcf(MujocoLib.mjtGain value, string expected) {
    _actuator.CustomParams.GainType = value;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"gaintype=\"{expected}\""));
  }

  [TestCase(MujocoLib.mjtBias.mjBIAS_NONE, "none")]
  [TestCase(MujocoLib.mjtBias.mjBIAS_AFFINE, "affine")]
  [TestCase(MujocoLib.mjtBias.mjBIAS_MUSCLE, "muscle")]
  [TestCase(MujocoLib.mjtBias.mjBIAS_USER, "user")]
  public void BiasTypeMjcf(MujocoLib.mjtBias value, string expected) {
    _actuator.CustomParams.BiasType = value;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"biastype=\"{expected}\""));
  }

  [Test]
  public void DynamicsParamsMjcf() {
    _actuator.CustomParams.DynPrm = new List<float>() { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"dynprm=\"1 2 3 4 5 6 7 8 9 10\""));
  }

  [Test]
  public void GainParamsMjcf() {
    _actuator.CustomParams.GainPrm = new List<float>() { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"gainprm=\"1 2 3 4 5 6 7 8 9 10\""));
  }

  [Test]
  public void BiasParamsMjcf() {
    _actuator.CustomParams.BiasPrm = new List<float>() { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"biasprm=\"1 2 3 4 5 6 7 8 9 10\""));
  }

  [Test]
  public void ParseAllSettings() {
    _doc.LoadXml(
        "<general joint=\"my_joint\" dyntype=\"integrator\" gaintype=\"muscle\" "
        + "biastype=\"affine\" dynprm=\"6\" gainprm=\"7 8\" biasprm=\"9 10 11 12\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("general")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.DynType, Is.EqualTo(MujocoLib.mjtDyn.mjDYN_INTEGRATOR));
    Assert.That(_actuator.CustomParams.GainType, Is.EqualTo(MujocoLib.mjtGain.mjGAIN_MUSCLE));
    Assert.That(_actuator.CustomParams.BiasType, Is.EqualTo(MujocoLib.mjtBias.mjBIAS_AFFINE));
    Assert.That(_actuator.CustomParams.DynPrm, Has.Count.EqualTo(3));
    Assert.That(_actuator.CustomParams.GainPrm, Has.Count.EqualTo(3));
    Assert.That(_actuator.CustomParams.BiasPrm, Has.Count.EqualTo(4));
    Assert.That(_actuator.CustomParams.DynPrm[0], Is.EqualTo(6));
    Assert.That(_actuator.CustomParams.DynPrm[1], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.DynPrm[2], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.GainPrm[0], Is.EqualTo(7));
    Assert.That(_actuator.CustomParams.GainPrm[1], Is.EqualTo(8));
    Assert.That(_actuator.CustomParams.GainPrm[2], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.BiasPrm[0], Is.EqualTo(9));
    Assert.That(_actuator.CustomParams.BiasPrm[1], Is.EqualTo(10));
    Assert.That(_actuator.CustomParams.BiasPrm[2], Is.EqualTo(11));
    Assert.That(_actuator.CustomParams.BiasPrm[3], Is.EqualTo(12));
  }

  [Test]
  public void ParseDefaultSettings() {
    _doc.LoadXml("<general joint=\"my_joint\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("general")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.DynType, Is.EqualTo(MujocoLib.mjtDyn.mjDYN_NONE));
    Assert.That(_actuator.CustomParams.GainType, Is.EqualTo(MujocoLib.mjtGain.mjGAIN_FIXED));
    Assert.That(_actuator.CustomParams.BiasType, Is.EqualTo(MujocoLib.mjtBias.mjBIAS_NONE));
    Assert.That(_actuator.CustomParams.DynPrm, Has.Count.EqualTo(3));
    Assert.That(_actuator.CustomParams.GainPrm, Has.Count.EqualTo(3));
    Assert.That(_actuator.CustomParams.BiasPrm, Has.Count.EqualTo(3));
    Assert.That(_actuator.CustomParams.DynPrm[0], Is.EqualTo(1));
    Assert.That(_actuator.CustomParams.DynPrm[1], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.DynPrm[2], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.GainPrm[0], Is.EqualTo(1));
    Assert.That(_actuator.CustomParams.GainPrm[1], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.GainPrm[2], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.BiasPrm[0], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.BiasPrm[1], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.BiasPrm[2], Is.EqualTo(0));
  }
}

[TestFixture]
public class MjPositionActuatorTests {
  private MjHingeJoint _joint;
  private MjActuator _actuator;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _joint.name = "my_joint";
    _actuator = new GameObject("actuator").AddComponent<MjActuator>();
    _actuator.Type = MjActuator.ActuatorType.Position;
    _actuator.Joint = _joint;
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_actuator.gameObject);
    UnityEngine.Object.DestroyImmediate(_joint.gameObject);
  }

  [Test]
  public void ProperTagNameUsed() {
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<position"));
  }

  [TestCase(3, 2, "3", "2")]
  [TestCase(-2, -1, "2", "1")]
  public void BiasParamsMjcf(float valueP, float valueV, string expectedP, string expectedV) {
    _actuator.CustomParams.Kp = valueP;
    _actuator.CustomParams.Kvp = valueV;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"kp=\"{expectedP}\""));
    Assert.That(_doc.OuterXml, Does.Contain($"kv=\"{expectedV}\""));
  }

  [Test]
  public void ParseAllSettings() {
    _doc.LoadXml("<position joint=\"my_joint\" kp=\"2\" kv=\"1\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("position")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.Kp, Is.EqualTo(2));
    Assert.That(_actuator.CustomParams.Kvp, Is.EqualTo(1));
  }

  [Test]
  public void ParseDefaultSettings() {
    _doc.LoadXml("<position joint=\"my_joint\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("position")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.Kp, Is.EqualTo(1));
    Assert.That(_actuator.CustomParams.Kvp, Is.EqualTo(0));
  }
}

[TestFixture]
public class MjVelocityActuatorTests {
  private MjHingeJoint _joint;
  private MjActuator _actuator;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _joint.name = "my_joint";
    _actuator = new GameObject("actuator").AddComponent<MjActuator>();
    _actuator.Type = MjActuator.ActuatorType.Velocity;
    _actuator.Joint = _joint;
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_actuator.gameObject);
    UnityEngine.Object.DestroyImmediate(_joint.gameObject);
  }

  [Test]
  public void ProperTagNameUsed() {
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<velocity"));
  }

  [TestCase(3, "3")]
  [TestCase(-2, "2")]
  public void BiasParamsMjcf(float value, string expected) {
    _actuator.CustomParams.Kv = value;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"kv=\"{expected}\""));
  }

  [Test]
  public void ParseAllSettings() {
    _doc.LoadXml("<velocity joint=\"my_joint\" kv=\"2\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("velocity")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.Kv, Is.EqualTo(2));
  }

  [Test]
  public void ParseDefaultSettings() {
    _doc.LoadXml("<velocity joint=\"my_joint\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("velocity")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.Kv, Is.EqualTo(1));
  }
}

[TestFixture]
public class MjCylinderActuatorTests {
  private MjHingeJoint _joint;
  private MjActuator _actuator;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _joint.name = "my_joint";
    _actuator = new GameObject("actuator").AddComponent<MjActuator>();
    _actuator.Type = MjActuator.ActuatorType.Cylinder;
    _actuator.Joint = _joint;
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_actuator.gameObject);
    UnityEngine.Object.DestroyImmediate(_joint.gameObject);
  }

  [Test]
  public void ProperTagNameUsed() {
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<cylinder"));
  }

  [Test]
  public void TimeConstantMjcf() {
    _actuator.CustomParams.CylinderTimeConst = 2.0f;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"timeconst=\"2\""));
  }

  [TestCase(3, "3")]
  [TestCase(-2, "2")]
  public void AreaMjcf(float value, string expected) {
    _actuator.CustomParams.Area = value;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"area=\"{expected}\""));
  }

  [TestCase(3, "3")]
  [TestCase(-2, "2")]
  public void DiameterMjcf(float value, string expected) {
    _actuator.CustomParams.Diameter = value;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"diameter=\"{expected}\""));
  }

  [Test]
  public void BiasMjcf() {
    _actuator.CustomParams.Bias = new float[] { 1, 2, 3 };
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"bias=\"1 2 3\""));
  }

  [Test]
  public void ParseAllSettings() {
    _doc.LoadXml("<cylinder joint=\"my_joint\" timeconst=\"1\" area =\"2\" diameter=\"3\" "
                 + "bias=\"4 5\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("cylinder")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.CylinderTimeConst, Is.EqualTo(1));
    Assert.That(_actuator.CustomParams.Area, Is.EqualTo(2));
    Assert.That(_actuator.CustomParams.Diameter, Is.EqualTo(3));
    Assert.That(_actuator.CustomParams.Bias, Has.Length.EqualTo(3));
    Assert.That(_actuator.CustomParams.Bias[0], Is.EqualTo(4));
    Assert.That(_actuator.CustomParams.Bias[1], Is.EqualTo(5));
    Assert.That(_actuator.CustomParams.Bias[2], Is.EqualTo(0));
  }

  [Test]
  public void ParseDefaultSettings() {
    _doc.LoadXml("<cylinder joint=\"my_joint\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("cylinder")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.CylinderTimeConst, Is.EqualTo(1));
    Assert.That(_actuator.CustomParams.Area, Is.EqualTo(1));
    Assert.That(_actuator.CustomParams.Diameter, Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.Bias, Has.Length.EqualTo(3));
    Assert.That(_actuator.CustomParams.Bias[0], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.Bias[1], Is.EqualTo(0));
    Assert.That(_actuator.CustomParams.Bias[2], Is.EqualTo(0));
  }
}

[TestFixture]
public class MjMuscleActuatorTests {
  private MjHingeJoint _joint;
  private MjActuator _actuator;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _joint.name = "my_joint";
    _actuator = new GameObject("actuator").AddComponent<MjActuator>();
    _actuator.Type = MjActuator.ActuatorType.Muscle;
    _actuator.Joint = _joint;
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_actuator.gameObject);
    UnityEngine.Object.DestroyImmediate(_joint.gameObject);
  }

  [Test]
  public void ProperTagNameUsed() {
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"<muscle"));
  }

  [Test]
  public void TimeConstantMjcf() {
    _actuator.CustomParams.MuscleTimeConst = new Vector2(1, 2);
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"timeconst=\"1 2\""));
  }

  [TestCase(1, 2, "1 2")]
  [TestCase(2, 1, "1 2")]
  public void RangeMjcf(float x, float y, string expected) {
    _actuator.CustomParams.Range = new Vector2(x, y);
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"range=\"{expected}\""));
  }

  [TestCase(-2, "-2")]
  [TestCase(2, "2")]
  public void ForceMjcf(float value, string expected) {
    _actuator.CustomParams.Force = value;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"force=\"{expected}\""));
  }

  [Test]
  public void LMinMjcf() {
    _actuator.CustomParams.LMin = 2.0f;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"lmin=\"2\""));
  }

  [Test]
  public void LMaxMjcf() {
    _actuator.CustomParams.LMin = 3.0f;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"lmin=\"3\""));
  }

  [Test]
  public void VMaxMjcf() {
    _actuator.CustomParams.VMax = 4.0f;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"vmax=\"4\""));
  }

  [Test]
  public void FpMaxMjcf() {
    _actuator.CustomParams.FpMax = 5.0f;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"fpmax=\"5\""));
  }

  [Test]
  public void FvMaxMjcf() {
    _actuator.CustomParams.FvMax = 6.0f;
    _doc.AppendChild(_actuator.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain($"fvmax=\"6\""));
  }

  [Test]
  public void ParseAllSettings() {
    _doc.LoadXml(
        "<muscle joint=\"my_joint\" timeconst=\"1 2\" range =\"3 4\" force=\"5\" scale=\"6\" "
        + "lmin=\"7\" lmax=\"8\" vmax=\"9\" fpmax=\"10\" fvmax=\"11\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("muscle")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.MuscleTimeConst, Is.EqualTo(new Vector2(1, 2)));
    Assert.That(_actuator.CustomParams.Range, Is.EqualTo(new Vector2(3, 4)));
    Assert.That(_actuator.CustomParams.Force, Is.EqualTo(5));
    Assert.That(_actuator.CustomParams.Scale, Is.EqualTo(6));
    Assert.That(_actuator.CustomParams.LMin, Is.EqualTo(7));
    Assert.That(_actuator.CustomParams.LMax, Is.EqualTo(8));
    Assert.That(_actuator.CustomParams.VMax, Is.EqualTo(9));
    Assert.That(_actuator.CustomParams.FpMax, Is.EqualTo(10));
    Assert.That(_actuator.CustomParams.FvMax, Is.EqualTo(11));
  }

  [Test]
  public void ParseDefaultSettings() {
    _doc.LoadXml("<muscle joint=\"my_joint\"/>");
    _actuator.ParseMjcf(_doc.GetElementsByTagName("muscle")[0] as XmlElement);
    Assert.That(_actuator.CustomParams.MuscleTimeConst, Is.EqualTo(new Vector2(0.01f, 0.04f)));
    Assert.That(_actuator.CustomParams.Range, Is.EqualTo(new Vector2(0.75f, 1.05f)));
    Assert.That(_actuator.CustomParams.Force, Is.EqualTo(-1));
    Assert.That(_actuator.CustomParams.Scale, Is.EqualTo(200));
    Assert.That(_actuator.CustomParams.LMin, Is.EqualTo(0.5f));
    Assert.That(_actuator.CustomParams.LMax, Is.EqualTo(1.6f));
    Assert.That(_actuator.CustomParams.VMax, Is.EqualTo(1.5f));
    Assert.That(_actuator.CustomParams.FpMax, Is.EqualTo(1.3f));
    Assert.That(_actuator.CustomParams.FvMax, Is.EqualTo(1.2f));
  }
}
}
