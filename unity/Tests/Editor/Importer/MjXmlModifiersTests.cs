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
using System.Linq;
using System.Xml;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace Mujoco {

[TestFixture]
public class MjDefaultSelectorsEditorTests {

  private XmlElement SetUp(string mjcfString, string selector) {
    var mjcf = new XmlDocument();
    mjcf.LoadXml(mjcfString);
    return mjcf.SelectSingleNode(selector) as XmlElement;
  }

  [TestCase("class")]
  [TestCase("childclass")]
  public void SelectingTopLevelClassDefault(string qualifier) {
    var mjcfString = @"<mujoco>
      <worldbody>
        <modified QUALIFIER=""class""/>
      </worldbody>
    </mujoco>".Replace("QUALIFIER", qualifier);
    var modifiedNode = SetUp(mjcfString, "/mujoco/worldbody/modified");
    var classes = MjXmlModifiers.GetApplicableDefaultClasses(modifiedNode).ToArray();
    Assert.That(classes.Length, Is.EqualTo(1));
    Assert.That(
        classes[0], Is.EqualTo("class"));
  }

  [TestCase("class")]
  [TestCase("childclass")]
  public void SelectingNestedClassDefaultWithInheritance(string qualifier) {
    var mjcfString = @"<mujoco>
      <worldbody>
        <some_node childclass=""parent"">
          <modified QUALIFIER=""child""/>
        </some_node>
      </worldbody>
    </mujoco>".Replace("QUALIFIER", qualifier);
    var modifiedNode = SetUp(mjcfString, "/mujoco/worldbody/some_node/modified");
    var classes = MjXmlModifiers.GetApplicableDefaultClasses(modifiedNode).ToArray();

    Assert.That(classes.Length, Is.EqualTo(2));
    Assert.That(classes[0], Is.EqualTo("child"));
    Assert.That(
        classes[1], Is.EqualTo("parent"));
  }

  [TestCase("class")]
  [TestCase("childclass")]
  public void SelectingNestedClassDefaultWithoutInheritance(string qualifier) {
    var mjcfString = @"<mujoco>
      <worldbody>
        <some_node class=""parent"">
          <modified QUALIFIER=""child""/>
        </some_node>
      </worldbody>
    </mujoco>".Replace("QUALIFIER", qualifier);
    var modifiedNode = SetUp(mjcfString, "/mujoco/worldbody/some_node/modified");
    var classes = MjXmlModifiers.GetApplicableDefaultClasses(modifiedNode).ToArray();

    Assert.That(classes.Length, Is.EqualTo(1));
    Assert.That(classes[0], Is.EqualTo("child"));
  }
}

[TestFixture]
public class MjAttributeCopierEditorTests {

  [Test]
  public void CopyingXmlElementAttributes() {
    var mjcfString = @"<root>
      <tag_1 attr_1=""a"" attr_2=""b""/>
      <tag_2/>
    </root>";
    var mjcf = new XmlDocument();
    mjcf.LoadXml(mjcfString);
    var tags = new XmlElement[] {
        mjcf.SelectSingleNode("/root/tag_1") as XmlElement,
        mjcf.SelectSingleNode("/root/tag_2") as XmlElement
    };
    MjXmlModifiers.CopyAttributes(tags[0], tags[1]);
    // Reselect the element to make sure we're testing the actual part of the DOM, and not its copy.
    var testedElement = mjcf.SelectSingleNode("/root/tag_2") as XmlElement;
    Assert.That(testedElement.GetStringAttribute("attr_1", "unknown"), Is.EqualTo("a"));
    Assert.That(testedElement.GetStringAttribute("attr_2", "unknown"), Is.EqualTo("b"));
  }

  [Test]
  public void CopyingXmlElementAttributesWithoutOverwritingTheExistingOnes() {
    var mjcfString = @"<root>
      <tag_1 attr_1=""a"" attr_2=""b""/>
      <tag_2 attr_1=""c""/>
    </root>";
    var mjcf = new XmlDocument();
    mjcf.LoadXml(mjcfString);
    var tags = new XmlElement[] {
        mjcf.SelectSingleNode("/root/tag_1") as XmlElement,
        mjcf.SelectSingleNode("/root/tag_2") as XmlElement
    };
    MjXmlModifiers.CopyAttributes(tags[0], tags[1]);
    // Reselect the element to make sure we're testing the actual part of the DOM, and not its copy.
    var testedElement = mjcf.SelectSingleNode("/root/tag_2") as XmlElement;
    Assert.That(testedElement.GetStringAttribute("attr_1", "unknown"), Is.EqualTo("c"));
    Assert.That(testedElement.GetStringAttribute("attr_2", "unknown"), Is.EqualTo("b"));
  }

  [Test]
  public void CopyingXmlElementAttributesWithOverwritingTheExistingOnes() {
    var mjcfString = @"<root>
      <tag_1 attr_1=""a"" attr_2=""b""/>
      <tag_2 attr_1=""c""/>
    </root>";
    var mjcf = new XmlDocument();
    mjcf.LoadXml(mjcfString);
    var tags = new XmlElement[] {
        mjcf.SelectSingleNode("/root/tag_1") as XmlElement,
        mjcf.SelectSingleNode("/root/tag_2") as XmlElement
    };
    MjXmlModifiers.CopyAttributesOverwriteExisting(tags[0], tags[1]);
    // Reselect the element to make sure we're testing the actual part of the DOM, and not its copy.
    var testedElement = mjcf.SelectSingleNode("/root/tag_2") as XmlElement;
    Assert.That(testedElement.GetStringAttribute("attr_1", "unknown"), Is.EqualTo("a"));
    Assert.That(testedElement.GetStringAttribute("attr_2", "unknown"), Is.EqualTo("b"));
  }
}

[TestFixture]
public class MjXmlModifiersIntegrationEditorTests {

  [Test]
  public void ApplyingDefaultsAggregatesAttributesFromMultipleDefaults() {
    var mjcfString = @"<mujoco>
      <default>
        <modified attrib=""a""/>
        <default class=""class"">
          <modified attrib=""b""/>
        </default>
      </default>
      <worldbody>
        <modified class=""class""/>
      </worldbody>
    </mujoco>";

    var mjcf = new XmlDocument();
    mjcf.LoadXml(mjcfString);
    var modifiedElement = mjcf.SelectSingleNode("/mujoco/worldbody/modified") as XmlElement;
    var modifiers = new MjXmlModifiers(mjcf);
    modifiers.ApplyModifiersToElement(modifiedElement);

    modifiedElement = mjcf.SelectSingleNode("/mujoco/worldbody/modified") as XmlElement;
    Assert.That(modifiedElement.GetStringAttribute("attrib", string.Empty), Is.EqualTo("b"));
  }

  [Test]
  public void ApplyingDefaultsDoesntOverwriteTheExistingNodeAttribute() {
    var mjcfString = @"<mujoco>
      <default>
        <modified attrib=""b""/>
      </default>
      <worldbody>
        <modified attrib=""a""/>
      </worldbody>
    </mujoco>";

    var mjcf = new XmlDocument();
    mjcf.LoadXml(mjcfString);
    var modifiedElement = mjcf.SelectSingleNode("/mujoco/worldbody/modified") as XmlElement;
    var modifiers = new MjXmlModifiers(mjcf);
    modifiers.ApplyModifiersToElement(modifiedElement);

    modifiedElement = mjcf.SelectSingleNode("/mujoco/worldbody/modified") as XmlElement;
    Assert.That(modifiedElement.GetStringAttribute("attrib", string.Empty), Is.EqualTo("a"));
  }

  [Test]
  public void MissingDefaultDoesntBreakInheritanceHierarchy() {
    var mjcfString = @"<mujoco>
      <default>
          <default class=""a"">
              <default class=""b"">
                  <geom type=""sphere""/>
              </default>
          </default>
      </default>
      <worldbody>
          <body childclass=""a"">
              <body childclass=""b"">
                  <geom name=""thingy""/>
              </body>
          </body>
      </worldbody>
    </mujoco>";

    var mjcf = new XmlDocument();
    mjcf.LoadXml(mjcfString);
    var modifiedElement = mjcf.SelectSingleNode("descendant::body/geom") as XmlElement;
    var modifiers = new MjXmlModifiers(mjcf);
    modifiers.ApplyModifiersToElement(modifiedElement);
    Assert.That(modifiedElement.GetStringAttribute("name", string.Empty), Is.EqualTo("thingy"));
    Assert.That(modifiedElement.GetStringAttribute("type", string.Empty), Is.EqualTo("sphere"));
  }

  [Test]
  public void PropagatePropertiesAcrossDefaultHierarchy() {
    var mjcfString = @"<mujoco>
      <default>
          <default class=""a"">
              <geom attrib1=""from_a""/>
              <default class=""b"">
                  <geom attrib2=""from_b""/>
              </default>
          </default>
      </default>
      <worldbody>
          <body>
              <body>
                  <body>
                      <geom class=""b"" name=""thingy""/>
                  </body>
              </body>
          </body>
      </worldbody>
    </mujoco>";

    var mjcf = new XmlDocument();
    mjcf.LoadXml(mjcfString);
    var modifiedElement = mjcf.SelectSingleNode("descendant::body/geom") as XmlElement;
    var modifiers = new MjXmlModifiers(mjcf);
    modifiers.ApplyModifiersToElement(modifiedElement);
    Assert.That(modifiedElement.GetStringAttribute("name", string.Empty), Is.EqualTo("thingy"));
    Assert.That(modifiedElement.GetStringAttribute("attrib1", string.Empty), Is.EqualTo("from_a"));
    Assert.That(modifiedElement.GetStringAttribute("attrib2", string.Empty), Is.EqualTo("from_b"));
  }
}
}
