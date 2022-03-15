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
public class MjJointSettingsGenerationTests {

  private MjJointSettings _settings;
  private XmlDocument _doc;
  private XmlElement _mjcf;

  [SetUp]
  public void SetUp() {
    _settings = new MjJointSettings();
    _doc = new XmlDocument();
    _mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("joint"));
  }

  [Test]
  public void DampingMjcf() {
    _settings.Spring.Damping = 5;
    _settings.Spring.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"damping=""5"""));
  }

  [Test]
  public void FrictionLossMjcf() {
    _settings.Solver.FrictionLoss = 5;
    _settings.Solver.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"frictionloss=""5"""));
  }

  [Test]
  public void StiffnessMjcf() {
    _settings.Spring.Stiffness = 5;
    _settings.Spring.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"stiffness=""5"""));
  }

  [Test]
  public void SpringDamperSettingsMjcf() {
    _settings.Spring.TimeConstant = 5;
    _settings.Spring.DampingRatio = 6;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"springdamper=""5 6"""));
  }

  [Test]
  public void SpringEquilibriumPoseMjcf() {
    _settings.Spring.EquilibriumPose = 5;
    _settings.Spring.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"springref=""5"""));
  }

  [Test]
  public void ArmatureMjcf() {
    _settings.Armature = 3;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"armature=""3"""));
  }

  [Test]
  public void SolverRefLimitMjcf() {
    _settings.Solver.RefLimit = new SolverReference() { TimeConst = 5, DampRatio = 6 };
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"solreflimit=""5 6"""));
  }

  [Test]
  public void SolverImpLimitMjcf() {
    _settings.Solver.ImpLimit = new SolverImpedance() {
        DMin = 5, DMax = 6, Width = 7, Midpoint = 8, Power = 9 };
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"solimplimit=""5 6 7 8 9"""));
  }

  [Test]
  public void SolverRefFrictionMjcf() {
    _settings.Solver.RefFriction = new SolverReference() { TimeConst = 5, DampRatio = 6 };
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"solreffriction=""5 6"""));
  }

  [Test]
  public void SolverImpFrictionMjcf() {
    _settings.Solver.ImpFriction = new SolverImpedance() {
        DMin = 5, DMax = 6, Width = 7, Midpoint = 8, Power = 9 };
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"solimpfriction=""5 6 7 8 9"""));
  }
}

[TestFixture]
public class MjJointSettingsParsingTests {

  private MjJointSettings _settings;
  private XmlDocument _doc;
  private XmlElement _mjcf;

  [SetUp]
  public void SetUp() {
    _doc = new XmlDocument();
    _mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("joint"));
    _settings = new MjJointSettings();
  }

  [Test]
  public void DampingMjcf() {
    _mjcf.SetAttribute("damping", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Spring.Damping, Is.EqualTo(5));
  }

  [Test]
  public void ArmatureMjcf() {
    _mjcf.SetAttribute("armature", "3");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Armature, Is.EqualTo(3));
  }

  [Test]
  public void FrictionLossMjcf() {
    _mjcf.SetAttribute("frictionloss", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.FrictionLoss, Is.EqualTo(5));
  }

  [Test]
  public void StiffnessMjcf() {
    _mjcf.SetAttribute("stiffness", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Spring.Stiffness, Is.EqualTo(5));
  }

  [Test]
  public void SpringDamperSettingsMjcf() {
    _mjcf.SetAttribute("springdamper", "5 6");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Spring.TimeConstant, Is.EqualTo(5));
    Assert.That(_settings.Spring.DampingRatio, Is.EqualTo(6));
  }

  [Test]
  public void SpringEquilibriumPoseMjcf() {
    _mjcf.SetAttribute("springref", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Spring.EquilibriumPose, Is.EqualTo(5));
  }

  [TestCase("true", true)]
  [TestCase("false", false)]
  public void ParsingJointLimitSettings(string setting, bool expected) {
    _mjcf.SetAttribute("limited", setting);
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.Limited, Is.EqualTo(expected));
  }

  [TestCase("1", 1)]
  [TestCase("-5", -5)]
  public void ParsingJointLimitMarginSettings(string setting, float expected) {
    _mjcf.SetAttribute("margin", setting);
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.Margin, Is.EqualTo(expected));
  }

  [Test]
  public void SolverRefLimitMjcf() {
    _mjcf.SetAttribute("solreflimit", "5 6");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.RefLimit.TimeConst, Is.EqualTo(5));
    Assert.That(_settings.Solver.RefLimit.DampRatio, Is.EqualTo(6));
  }

  [Test]
  public void SolverImpLimitMjcf() {
    _mjcf.SetAttribute("solimplimit", "5 6 7 8 9");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.ImpLimit.DMin, Is.EqualTo(5));
    Assert.That(_settings.Solver.ImpLimit.DMax, Is.EqualTo(6));
    Assert.That(_settings.Solver.ImpLimit.Width, Is.EqualTo(7));
    Assert.That(_settings.Solver.ImpLimit.Midpoint, Is.EqualTo(8));
    Assert.That(_settings.Solver.ImpLimit.Power, Is.EqualTo(9));
  }

  [Test]
  public void SolverRefFrictionMjcf() {
    _mjcf.SetAttribute("solreffriction", "5 6");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.RefFriction.TimeConst, Is.EqualTo(5));
    Assert.That(_settings.Solver.RefFriction.DampRatio, Is.EqualTo(6));
  }

  [Test]
  public void SolverImpFrictionMjcf() {
    _mjcf.SetAttribute("solimpfriction", "5 6 7 8 9");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.ImpFriction.DMin, Is.EqualTo(5));
    Assert.That(_settings.Solver.ImpFriction.DMax, Is.EqualTo(6));
    Assert.That(_settings.Solver.ImpFriction.Width, Is.EqualTo(7));
    Assert.That(_settings.Solver.ImpFriction.Midpoint, Is.EqualTo(8));
    Assert.That(_settings.Solver.ImpFriction.Power, Is.EqualTo(9));
  }
}
}
