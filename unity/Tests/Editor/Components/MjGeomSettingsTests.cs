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
public class MjGeomSettingsGenerationTests {

  private MjGeomSettings _settings;
  private XmlDocument _doc;
  private XmlElement _mjcf;

  [SetUp]
  public void SetUp() {
    _settings = new MjGeomSettings();
    _doc = new XmlDocument();
    _mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
  }

  [Test]
  public void ContactFilteringTypeMjcf() {
    _settings.Filtering.Contype = 5;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"contype=""5"""));
  }

  [Test]
  public void ContactFilteringAffinityMjcf() {
    _settings.Filtering.Conaffinity = 5;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"conaffinity=""5"""));
  }

  [Test]
  public void ContactFilteringGroupMjcf() {
    _settings.Filtering.Group = 5;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"group=""5"""));
  }

  [Test]
  public void SolverContactDimensionalityMjcf() {
    _settings.Solver.ConDim = 5;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"condim=""5"""));
  }

  [Test]
  public void SolverMixMjcf() {
    _settings.Solver.SolMix = 5;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"solmix=""5"""));
  }

  [Test]
  public void SolverReferenceMjcf() {
    _settings.Solver.SolRef.TimeConst = 5;
    _settings.Solver.SolRef.DampRatio = 6;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"solref=""5 6"""));
  }

  [Test]
  public void SolverImpendanceMjcf() {
    _settings.Solver.SolImp.DMin = 5;
    _settings.Solver.SolImp.DMax = 6;
    _settings.Solver.SolImp.Width = 7;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"solimp=""5 6 7"""));
  }

  [Test]
  public void SolverMarginMjcf() {
    _settings.Solver.Margin = 5;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"margin=""5"""));
  }

  [Test]
  public void SolverGapMjcf() {
    _settings.Solver.Gap = 5;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"gap=""5"""));
  }

  [Test]
  public void MaterialFrictionMjcf() {
    _settings.Friction.Sliding = 5;
    _settings.Friction.Torsional = 6;
    _settings.Friction.Rolling = 7;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"friction=""5 6 7"""));
  }

  [Test]
  public void FluidShapeTypeMjcf() {
    _settings.FluidShapeType = MjGeomSettings.FluidShapeTypes.Ellipsoid;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"fluidshape=""ellipsoid"""));
  }

  [Test]
  public void FluidCoefficientsMjcf() {
    _settings.FluidCoefficients.BluntDrag = 2f;
    _settings.FluidCoefficients.SlenderDrag = 3f;
    _settings.FluidCoefficients.AngularDrag = 4f;
    _settings.FluidCoefficients.KuttaLift = 5f;
    _settings.FluidCoefficients.MagnusLift = 6f;
    _settings.ToMjcf(_mjcf);
    Assert.That(_doc.OuterXml, Does.Contain(@"fluidcoef=""2 3 4 5 6"""));
  }
}

[TestFixture]
public class MjGeomSettingsParsingTests {

  private MjGeomSettings _settings;
  private XmlDocument _doc;
  private XmlElement _mjcf;

  [SetUp]
  public void SetUp() {
    _doc = new XmlDocument();
    _mjcf = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    _settings = new MjGeomSettings();
  }

  [Test]
  public void ContactFilteringTypeMjcf() {
    _mjcf.SetAttribute("contype", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Filtering.Contype, Is.EqualTo(5));
  }

  [Test]
  public void ContactFilteringAffinityMjcf() {
    _mjcf.SetAttribute("conaffinity", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Filtering.Conaffinity, Is.EqualTo(5));
  }

  [Test]
  public void ContactFilteringGroupMjcf() {
    _mjcf.SetAttribute("group", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Filtering.Group, Is.EqualTo(5));
  }

  [Test]
  public void SolverContactDimensionalityMjcf() {
    _mjcf.SetAttribute("condim", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.ConDim, Is.EqualTo(5));
  }

  [Test]
  public void SolverMixMjcf() {
    _mjcf.SetAttribute("solmix", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.SolMix, Is.EqualTo(5));
  }

  [Test]
  public void SolverReferenceMjcf() {
    _mjcf.SetAttribute("solref", "5 6");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.SolRef.TimeConst, Is.EqualTo(5));
    Assert.That(_settings.Solver.SolRef.DampRatio, Is.EqualTo(6));
  }

  [Test]
  public void SolverImpendanceMjcf() {
    _mjcf.SetAttribute("solimp", "5 6 7");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.SolImp.DMin, Is.EqualTo(5));
    Assert.That(_settings.Solver.SolImp.DMax, Is.EqualTo(6));
    Assert.That(_settings.Solver.SolImp.Width, Is.EqualTo(7));
  }

  [Test]
  public void SolverMarginMjcf() {
    _mjcf.SetAttribute("margin", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.Margin, Is.EqualTo(5));
  }

  [Test]
  public void SolverGapMjcf() {
    _mjcf.SetAttribute("gap", "5");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Solver.Gap, Is.EqualTo(5));
  }

  [Test]
  public void MaterialFrictionMjcf() {
    _mjcf.SetAttribute("friction", "5 6 7");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.Friction.Sliding, Is.EqualTo(5));
    Assert.That(_settings.Friction.Torsional, Is.EqualTo(6));
    Assert.That(_settings.Friction.Rolling, Is.EqualTo(7));
  }

  [Test]
  public void FluidShapeTypeMjcf() {
    _mjcf.SetAttribute("fluidshape", "ellipsoid");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.FluidShapeType, Is.EqualTo(
        MjGeomSettings.FluidShapeTypes.Ellipsoid
    ));
  }

  [Test]
  public void FluidCoefficientsMjcf() {
    _mjcf.SetAttribute("fluidcoef", "2 3 4 5 6");
    _settings.FromMjcf(_mjcf);
    Assert.That(_settings.FluidCoefficients.BluntDrag, Is.EqualTo(2));
    Assert.That(_settings.FluidCoefficients.SlenderDrag, Is.EqualTo(3));
    Assert.That(_settings.FluidCoefficients.AngularDrag, Is.EqualTo(4));
    Assert.That(_settings.FluidCoefficients.KuttaLift, Is.EqualTo(5));
    Assert.That(_settings.FluidCoefficients.MagnusLift, Is.EqualTo(6));
  }
}
}
