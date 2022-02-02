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
public class MjGeomTransformTests {

  private GameObject _parent;
  private MjGeom _geom;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _parent = new GameObject("parent");
    _geom = new GameObject("geom", typeof(MjGeom)).GetComponent<MjGeom>();
    _geom.transform.parent = _parent.transform;
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_geom.gameObject);
    UnityEngine.Object.DestroyImmediate(_parent);
  }

  [Test]
  public void WorldPositionToMjCoordinates() {
    _geom.transform.position = new Vector3(1, 2, 3);
    _doc.AppendChild(_geom.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain(@"pos=""1 3 2"""));
  }

  [Test]
  public void WorldRotationToMjCoordinates() {
    _geom.transform.rotation = Quaternion.AngleAxis(45.0f, Vector3.up);
    _doc.AppendChild(_geom.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain(@"quat=""-0.9238795 0 0 0.3826835"""));
  }

  [Test]
  public void BodyRelativePositionUsedInMjcf() {
    _parent.transform.position = new Vector3(1, 2, 3);
    _parent.transform.rotation = Quaternion.AngleAxis(45.0f, (new Vector3(1, 2, 3)).normalized);
    _doc.AppendChild(_geom.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain(@"pos=""1 3 2"""));
    Assert.That(_doc.OuterXml, Does.Contain(@"quat=""-0.9238795 0.1022765 0.3068294 0.2045529"""));
  }

  [Test]
  public void LocalPositionParsing() {
    var geomElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    geomElement.SetAttribute("pos", "1 2 3");
    _geom.ParseMjcf(geomElement);
    Assert.That(_geom.transform.localPosition, Is.EqualTo(new Vector3(1, 3, 2)));
  }

  [TestCase(0.866f, 0.5f, 0, 0, 0.5f, 0, 0, -0.866f)]
  [TestCase(0.866f, 0, 0.5f, 0, 0, 0, 0.5f, -0.866f)]
  [TestCase(0.866f, 0, 0, 0.5f, 0, 0.5f, 0, -0.866f)]
  [TestCase(0, 0.866f, 0.5f, 0, 0.866f, 0, 0.5f, 0)]
  [TestCase(0, 0.866f, 0, 0.5f, 0.866f, 0.5f, 0, 0)]
  [TestCase(0, 0, 0.866f, 0.5f, 0, 0.5f, 0.866f, 0)]
  public void BodyLocalRotationParsing(
      float ix, float iy, float iz, float iw, float ex, float ey, float ez, float ew) {
    var geomElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    geomElement.SetAttribute("quat", $"{ix} {iy} {iz} {iw}");
    _geom.ParseMjcf(geomElement);
    var angle = Quaternion.Angle(_geom.transform.localRotation, new Quaternion(ex, ey, ez, ew));
    Assert.That(angle, Is.EqualTo(0).Within(3));
  }

  [TestCase("0 0 0 0 0 1", 0, 0, 0, 1)]
  [TestCase("0 0 0 0 1 0", 0.707f, 0, 0, 0.707f)]
  [TestCase("0 0 0 1 0 0", 0, 0, -0.707f, 0.707f)]
  [TestCase("1 0 0 0 0 0", 0, 0, 0.707f, 0.707f)]
  [TestCase("0 1 0 0 0 0", -0.707f, 0, 0, 0.707f)]
  [TestCase("0 0 1 0 0 0", 1, 0, 0, 0)]
  [TestCase("1 0 0 0 0 1", 0, 0, 0.382f, 0.924f)]
  [TestCase("1 0 0 0 1 0", 0.5f, 0, 0.5f, 0.707f)]
  public void OrientationSpecifiedUsingFromToParameterIsTheDirectionOfFromToVector(
      string input, float ex, float ey, float ez, float ew) {
    var geomElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    geomElement.SetAttribute("type", "box");
    geomElement.SetAttribute("fromto", input);
    _geom.ParseMjcf(geomElement);
    var angle = Quaternion.Angle(_geom.transform.localRotation, new Quaternion(ex, ey, ez, ew));
    Assert.That(angle, Is.EqualTo(0).Within(3));
  }

  [TestCase("0 0 0 0 0 1", 0)]
  [TestCase("0 0 0 0 0.01 1", 0.573f)]
  [TestCase("0 0 0 0 -0.01 1", 0.573f)]
  [TestCase("0 0 0 0 0 -1", 180)]
  [TestCase("0 0 0 0 0.01 -1", 179.427f)]
  [TestCase("0 0 0 0 -0.01 -1", 179.427f)]
  public void TestingFromToRotationsAroundThePoles(string input, float expectedAngle) {
    var geomElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    geomElement.SetAttribute("type", "box");
    geomElement.SetAttribute("fromto", input);
    _geom.ParseMjcf(geomElement);
    var transformedForward = _geom.transform.localRotation * Vector3.forward;
    var angle = Vector3.Angle(transformedForward, Vector3.forward);
    Assert.That(angle, Is.EqualTo(expectedAngle).Within(1e-3f));
  }

  [TestCase("0 0 0 0 0 1", 0, 0.5f, 0)]
  [TestCase("0 0 0 0 1 0", 0, 0, 0.5f)]
  [TestCase("0 0 0 1 0 0", 0.5f, 0, 0)]
  [TestCase("1 0 0 0 0 1", 0.5f, 0.5f, 0)]
  [TestCase("0 1 0 0 0 1", 0, 0.5f, 0.5f)]
  [TestCase("0 1 0 0 0 1", 0, 0.5f, 0.5f)]
  public void PositionSpecifiedUsingFromToParameterIsMiddleOfFromToSegment(
      string input, float x, float y, float z) {
    var geomElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    geomElement.SetAttribute("type", "box");
    geomElement.SetAttribute("fromto", input);
    _geom.ParseMjcf(geomElement);
    Assert.That(_geom.transform.localPosition, Is.EqualTo(new Vector3(x, y, z)));
  }

  [TestCase("0 0 1", 0, 0, 0, 1)]
  [TestCase("0 1 0", 0.707f, 0, 0, 0.707f)]
  [TestCase("1 0 0", 0, 0, -0.707f, 0.707f)]
  public void ZAxisParameterSpecifiesForwardVectorOfGeom(
      string input, float ex, float ey, float ez, float ew) {
    var geomElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    geomElement.SetAttribute("zaxis", input);
    _geom.ParseMjcf(geomElement);
    var angle = Quaternion.Angle(_geom.transform.localRotation, new Quaternion(ex, ey, ez, ew));
    Assert.That(angle, Is.EqualTo(0).Within(3));
  }

  [TestCase("0 0 90", 0, 0.707f, 0, 0.707f)]
  [TestCase("0 90 0", 0, 0, -0.707f, 0.707f)]
  [TestCase("90 0 0", 0.707f, 0, 0, 0.707f)]
  [TestCase("0 0 45", 0, 0.382f, 0, 0.924f)]
  [TestCase("0 45 0", 0, 0, -0.382f, 0.924f)]
  [TestCase("45 0 0", 0.382f, 0, 0, 0.924f)]
  public void UsingEulerAnglesToSpecifyRotation(
      string input, float ex, float ey, float ez, float ew) {
    var geomElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    geomElement.SetAttribute("euler", input);
    _geom.ParseMjcf(geomElement);
    var angle = Quaternion.Angle(_geom.transform.localRotation, new Quaternion(ex, ey, ez, ew));
    Assert.That(angle, Is.EqualTo(0).Within(3));
  }

  [Test]
  public void NoRotationSpecificationSetsIdentityRotation() {
    var geomElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    _geom.ParseMjcf(geomElement);
    Assert.That(_geom.transform.localRotation, Is.EqualTo(Quaternion.identity));
  }
}

[TestFixture]
public class MjGeomPropertiesSerializingTests {

  private MjGeom _geom;
  private XmlDocument _doc;

  [SetUp]
  public void SetUp() {
    _geom = new GameObject("geom", typeof(MjGeom)).GetComponent<MjGeom>();
    _doc = new XmlDocument();
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_geom.gameObject);
  }

  [Test]
  public void MaterialDensityMjcf() {
    _geom.Density = 5;
    _doc.AppendChild(_geom.GenerateMjcf("name", _doc));
    Assert.That(_doc.OuterXml, Does.Contain(@"density=""5"""));
  }

  [Test]
  public void MaterialDensityParsing() {
    var geomElement = (XmlElement)_doc.AppendChild(_doc.CreateElement("geom"));
    geomElement.SetAttribute("density", "5.2");
    _geom.ParseMjcf(geomElement);
    Assert.That(_geom.Density, Is.EqualTo(5.2f));
  }
}
}
