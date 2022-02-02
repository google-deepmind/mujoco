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
  public class XmlElementExtensionsEditorTests {
    public enum TestEnum { ValueDefault, ValueA }

    private XmlElement _element;

    [SetUp]
    public void SetUp() {
      var doc = new XmlDocument();
      _element = doc.CreateElement("element");
    }

    [Test]
    public void ParsingMissingFloatAttribute() {
      Assert.That(_element.GetFloatAttribute("attrib", 7.0f), Is.EqualTo(7.0f));
    }

    [TestCase("5.1", 5.1f)]
    [TestCase("-4.83", -4.83f)]
    public void ParsingFloatAttribute(string input, float expected) {
      _element.SetAttribute("attrib", input);
      Assert.That(_element.GetFloatAttribute("attrib", 7.0f), Is.EqualTo(expected));
    }

    [TestCase("")]
    [TestCase("string value")]
    public void InvalidFloatValueTriggersAnException(string input) {
      _element.SetAttribute("attrib", input);
      Assert.That(() => { _element.GetFloatAttribute("attrib", 7.0f); }, Throws.Exception);
    }

    [Test]
    public void ParsingMissingEnumAttribute() {
      Assert.That(_element.GetEnumAttribute<TestEnum>("attrib", TestEnum.ValueDefault),
                  Is.EqualTo(TestEnum.ValueDefault));
    }

    [TestCase("ValueA", TestEnum.ValueA)]
    [TestCase("valuea", TestEnum.ValueA)]
    public void ParsingEnumAttributeIsCaseInsensitive(string input, TestEnum expected) {
      _element.SetAttribute("attrib", input);
      Assert.That(_element.GetEnumAttribute<TestEnum>("attrib", TestEnum.ValueDefault, true),
                  Is.EqualTo(expected));
    }

    [TestCase("")]
    [TestCase("wrong value")]
    public void InvalidEnumValueTriggersAnException(string input) {
      _element.SetAttribute("attrib", input);
      Assert.That(() => { _element.GetEnumAttribute<TestEnum>("attrib", TestEnum.ValueDefault); },
                  Throws.Exception);
    }

    [Test]
    public void ParsingMissingStringAttribute() {
      Assert.That(_element.GetStringAttribute("attrib", "missing"), Is.EqualTo("missing"));
    }

    [TestCase("value", "value")]
    [TestCase("", "")]
    public void ParsingStringAttributes(string input, string expected) {
      _element.SetAttribute("attrib", input);
      Assert.That(_element.GetStringAttribute("attrib", "missing"), Is.EqualTo(expected));
    }

    [Test]
    public void ParsingMissingVector3Attribute() {
      Assert.That(_element.GetVector3Attribute("attrib", Vector3.one), Is.EqualTo(Vector3.one));
    }

    public void ParsingVector3Attributes() {
      _element.SetAttribute("attrib", "-1 2 3.5");
      var expected = new Vector3(-1, 2, 3.5f);
      Assert.That(_element.GetVector3Attribute("attrib", Vector3.one), Is.EqualTo(expected));
    }

    [TestCase("1 2")]
    [TestCase("1")]
    [TestCase("")]
    public void Vector3WithInsufficientNumberOfComponentsTriggersAnException(string input) {
      _element.SetAttribute("attrib", input);
      Assert.That(() => { _element.GetVector3Attribute("attrib", Vector3.one); }, Throws.Exception);
    }

    [TestCase("a 2 3")]
    [TestCase("1 b 3")]
    [TestCase("1 2 c")]
    public void InvalidVector3ComponentValueTriggersException(string input) {
      _element.SetAttribute("attrib", input);
      Assert.That(() => { _element.GetVector3Attribute("attrib", Vector3.one); }, Throws.Exception);
    }

    public void ParsingVector2Attributes() {
      _element.SetAttribute("attrib", "-1 2.5");
      var expected = new Vector3(-1, 2.5f);
      Assert.That(_element.GetVector2Attribute("attrib", Vector2.one), Is.EqualTo(expected));
    }

    [TestCase("1")]
    [TestCase("")]
    public void Vector2WithInsufficientNumberOfComponentsTriggersAnException(string input) {
      _element.SetAttribute("attrib", input);
      Assert.That(() => { _element.GetVector2Attribute("attrib", Vector2.one); }, Throws.Exception);
    }

    [TestCase("a 2")]
    [TestCase("1 b")]
    public void InvalidVector2ComponentValueTriggersException(string input) {
      _element.SetAttribute("attrib", input);
      Assert.That(() => { _element.GetVector2Attribute("attrib", Vector2.one); }, Throws.Exception);
    }

    [Test]
    public void ParsingMissingQuaternionAttribute() {
      Assert.That(_element.GetQuaternionAttribute("attrib", Quaternion.identity),
                  Is.EqualTo(Quaternion.identity));
    }

    public void ParsingQuaternionAttributes() {
      _element.SetAttribute("attrib", "0.1 0.2 0.3 0.4");
      var expected = new Quaternion(0.1f, 0.2f, 0.3f, 0.4f);
      Assert.That(_element.GetQuaternionAttribute("attrib", Quaternion.identity),
                  Is.EqualTo(expected));
    }

    [TestCase("1 2 3")]
    [TestCase("1 2")]
    [TestCase("1")]
    [TestCase("")]
    public void QuaternionWithInsufficientNumberOfComponentsTriggersAnException(string input) {
      _element.SetAttribute("attrib", input);
      Assert.That(() => { _element.GetQuaternionAttribute("attrib", Quaternion.identity); },
                  Throws.Exception);
    }

    [TestCase("a 2 3 4")]
    [TestCase("1 b 3 4")]
    [TestCase("1 2 c 4")]
    [TestCase("1 2 3 d")]
    public void InvalidQuaternionComponentValueTriggersException(string input) {
      _element.SetAttribute("attrib", input);
      Assert.That(() => { _element.GetQuaternionAttribute("attrib", Quaternion.identity); },
                  Throws.Exception);
    }

    [TestCase("1  2")]
    [TestCase("1  2  ")]
    [TestCase("  1 2")]
    public void FloatArraysCanContainArbitraryNumberOfWhitespaces(string input) {
      _element.SetAttribute("attrib", input);
      Assert.That(_element.GetFloatArrayAttribute("attrib", defaultValue: null),
                  Is.EqualTo(new float[] { 1, 2 }));
    }

    public void FloatArraysCanBeFilledUpWithDefaultValues() {
      _element.SetAttribute("attrib", "1 2");
      var defaultValue = new float[] { 5, 6, 7, 8 };
      Assert.That(_element.GetFloatArrayAttribute("attrib", defaultValue: defaultValue),
                  Is.EqualTo(new float[] { 1, 2, 7, 8 }));
    }

    public void FloatArrayDefaultFillingCanBeSwitchedOff() {
      _element.SetAttribute("attrib", "1 2");
      var defaultValue = new float[] { 5, 6, 7, 8 };
      var result = _element.GetFloatArrayAttribute("attrib", defaultValue: defaultValue,
                                                   fillMissingValues: false);
      Assert.That(result, Is.EqualTo(new float[] { 1, 2 }));
    }
  }
}
