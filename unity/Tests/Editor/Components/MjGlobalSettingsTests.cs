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
public class MjGlobalSettingsGenerationTests {

  private MjGlobalSettings _settings;
  private XmlDocument _doc;
  private XmlElement _root;

  [SetUp]
  public void SetUp() {
    _settings = new GameObject("settings").AddComponent<MjGlobalSettings>();
    _doc = new XmlDocument();
    _root = (XmlElement)_doc.AppendChild(_doc.CreateElement("root"));
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_settings.gameObject);
  }

  [Test]
  public void GenerateOptionSizeFlagCustomNumeric() {
    _settings.GlobalOptions.ImpRatio = 1.2f;
    _settings.GlobalOptions.Magnetic = new Vector3(3.4f, 4.5f, 5.6f);
    _settings.GlobalOptions.Wind = new Vector3(6.7f, 7.8f, 8.9f);
    _settings.GlobalOptions.Density = 0.1f;
    _settings.GlobalOptions.Viscosity = 2.3f;
    _settings.GlobalOptions.OverrideMargin = 4.5f;
    _settings.GlobalOptions.Integrator = IntegratorType.RK4;
    _settings.GlobalOptions.Cone = FrictionConeType.elliptic;
    _settings.GlobalOptions.Jacobian = JacobianType.dense;
    _settings.GlobalOptions.Solver = ConstraintSolverType.PGS;
    _settings.GlobalOptions.Iterations = 12;
    _settings.GlobalOptions.Tolerance = 3.4f;
    _settings.GlobalOptions.NoSlipIterations = 5;
    _settings.GlobalOptions.NoSlipTolerance = 6.7f;
    _settings.GlobalOptions.CcdIterations = 8;
    _settings.GlobalOptions.CcdTolerance = 0.9f;
    _settings.GlobalSizes.Memory = "1M";

    _settings.GlobalsToMjcf(_root);
    Assert.That(_doc.OuterXml, Does.Contain(@"impratio=""1.2"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"magnetic=""3.4 4.5 5.6"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"wind=""6.7 7.8 8.9"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"density=""0.1"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"viscosity=""2.3"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"o_margin=""4.5"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"integrator=""RK4"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"cone=""elliptic"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"jacobian=""dense"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"o_solref="));
    Assert.That(_doc.OuterXml, Does.Contain(@"o_solimp="));
    Assert.That(_doc.OuterXml, Does.Contain(@"solver=""PGS"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"iterations=""12"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"tolerance=""3.4"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"noslip_iterations=""5"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"noslip_tolerance=""6.7"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"ccd_iterations=""8"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"ccd_tolerance=""0.9"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"memory=""1M"""));
  }

  [Test]
  public void GenerateFlag() {
    _settings.GlobalOptions.Flag.Gravity = EnableDisableFlag.disable;
    _settings.GlobalsToMjcf(_root);
    Assert.That(_doc.OuterXml, Does.Contain(@"gravity=""disable"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"constraint="));
    Assert.That(_doc.OuterXml, Does.Contain(@"equality="));
    Assert.That(_doc.OuterXml, Does.Contain(@"frictionloss="));
    Assert.That(_doc.OuterXml, Does.Contain(@"limit="));
    Assert.That(_doc.OuterXml, Does.Contain(@"contact="));
    Assert.That(_doc.OuterXml, Does.Contain(@"passive="));
    Assert.That(_doc.OuterXml, Does.Contain(@"clampctrl="));
    Assert.That(_doc.OuterXml, Does.Contain(@"warmstart="));
    Assert.That(_doc.OuterXml, Does.Contain(@"filterparent="));
    Assert.That(_doc.OuterXml, Does.Contain(@"actuation="));
    Assert.That(_doc.OuterXml, Does.Contain(@"refsafe="));
    Assert.That(_doc.OuterXml, Does.Contain(@"override="));
    Assert.That(_doc.OuterXml, Does.Contain(@"energy="));
    Assert.That(_doc.OuterXml, Does.Contain(@"fwdinv="));
  }
}

[TestFixture]
public class MjGlobalSettingsParsingTests {

  private MjGlobalSettings _settings;
  private XmlElement _root;
  private XmlElement _option;
  private XmlElement _size;
  private XmlElement _flag;
  private XmlElement _numeric;

  [SetUp]
  public void SetUp() {
    _settings = new GameObject("settings").AddComponent<MjGlobalSettings>();
    var doc = new XmlDocument();
    _root = (XmlElement)doc.AppendChild(doc.CreateElement("root"));
    _option = (XmlElement)_root.AppendChild(doc.CreateElement("option"));
    _size = (XmlElement)_root.AppendChild(doc.CreateElement("size"));
    _flag = (XmlElement)_option.AppendChild(doc.CreateElement("flag"));
    var custom = (XmlElement)_root.AppendChild(doc.CreateElement("custom"));
    _numeric = (XmlElement)custom.AppendChild(doc.CreateElement("numeric"));
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_settings.gameObject);
  }

  [Test]
  public void ParseSizeOptionFlagCustomNumeric() {
    _option.SetAttribute("impratio", "1.2");
    _option.SetAttribute("magnetic", "3.4 4.5 5.6");
    _option.SetAttribute("wind", "6.7 7.8 8.9");
    _option.SetAttribute("density", "0.1");
    _option.SetAttribute("viscosity", "2.3");
    _option.SetAttribute("o_margin", "4.5");
    _option.SetAttribute("integrator", "RK4");
    _option.SetAttribute("cone", "elliptic");
    _option.SetAttribute("jacobian", "dense");
    _option.SetAttribute("solver", "PGS");
    _option.SetAttribute("iterations", "12");
    _option.SetAttribute("tolerance", "3.4");
    _option.SetAttribute("noslip_iterations", "5");
    _option.SetAttribute("noslip_tolerance", "6.7");
    _option.SetAttribute("ccd_iterations", "8");
    _option.SetAttribute("ccd_tolerance", "0.9");

    _flag.SetAttribute("gravity", "disable");

    _size.SetAttribute("memory", "1M");

    _numeric.SetAttribute("name", "numeric_name");
    _numeric.SetAttribute("data", "1 2 3");

    _settings.ParseGlobalMjcfSections(_root);
    Assert.That(_settings.GlobalOptions.ImpRatio, Is.EqualTo(1.2f));
    Assert.That(_settings.GlobalOptions.Magnetic, Is.EqualTo(new Vector3(3.4f, 4.5f, 5.6f)));
    Assert.That(_settings.GlobalOptions.Wind, Is.EqualTo(new Vector3(6.7f, 7.8f, 8.9f)));
    Assert.That(_settings.GlobalOptions.Density, Is.EqualTo(0.1f));
    Assert.That(_settings.GlobalOptions.Viscosity, Is.EqualTo(2.3f));
    Assert.That(_settings.GlobalOptions.OverrideMargin, Is.EqualTo(4.5f));
    Assert.That(_settings.GlobalOptions.Integrator, Is.EqualTo(IntegratorType.RK4));
    Assert.That(_settings.GlobalOptions.Cone, Is.EqualTo(FrictionConeType.elliptic));
    Assert.That(_settings.GlobalOptions.Jacobian, Is.EqualTo(JacobianType.dense));
    Assert.That(_settings.GlobalOptions.Solver, Is.EqualTo(ConstraintSolverType.PGS));
    Assert.That(_settings.GlobalOptions.Iterations, Is.EqualTo(12));
    Assert.That(_settings.GlobalOptions.Tolerance, Is.EqualTo(3.4f));
    Assert.That(_settings.GlobalOptions.NoSlipIterations, Is.EqualTo(5));
    Assert.That(_settings.GlobalOptions.NoSlipTolerance, Is.EqualTo(6.7f));
    Assert.That(_settings.GlobalOptions.CcdIterations, Is.EqualTo(8));
    Assert.That(_settings.GlobalOptions.CcdTolerance, Is.EqualTo(0.9f));

    Assert.That(_settings.GlobalOptions.Flag.Gravity, Is.EqualTo(EnableDisableFlag.disable));

    Assert.That(_settings.GlobalSizes.Memory, Is.EqualTo("1M"));

    Assert.That(_settings.CustomNumeric.Count, Is.EqualTo(1));
    Assert.That(_settings.CustomNumeric[0].Name, Is.EqualTo("numeric_name"));
    Assert.That(_settings.CustomNumeric[0].Data, Is.EqualTo("1 2 3"));
  }
}
}
