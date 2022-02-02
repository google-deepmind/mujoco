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

namespace Mujoco {

  [TestFixture]
  public class FindParentComponentTests {
    public class FakeMjComponent : MjComponent {
      public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_GEOM;
      protected override void OnParseMjcf(XmlElement mjcf) {}
      protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
        return null;
      }
    }

    public class FakeGenericComponent : MonoBehaviour {}

    private FakeMjComponent _parent;
    private FakeMjComponent _child;
    private FakeMjComponent _secondChild;
    private FakeGenericComponent _hierarchyBreaker;

    [SetUp]
    public void SetUp() {
      _parent =
          new GameObject("parent", typeof(FakeMjComponent)).GetComponent<FakeMjComponent>();
      _child =
          new GameObject("child", typeof(FakeMjComponent)).GetComponent<FakeMjComponent>();
      _secondChild = new GameObject("secondChild", typeof(FakeMjComponent))
                         .GetComponent<FakeMjComponent>();
      _hierarchyBreaker = new GameObject("hierarchyBreaker", typeof(FakeGenericComponent))
                              .GetComponent<FakeGenericComponent>();
    }

    [TearDown]
    public void TearDown() {
      GameObject.DestroyImmediate(_secondChild.gameObject);
      GameObject.DestroyImmediate(_child.gameObject);
      GameObject.DestroyImmediate(_hierarchyBreaker.gameObject);
      GameObject.DestroyImmediate(_parent.gameObject);
    }

    [Test]
    public void FindParentWithMjComponentAsImmediateParent() {
      _child.transform.parent = _parent.transform;
      Assert.That(MjHierarchyTool.FindParentComponent(_child), Is.EqualTo(_parent));
    }

    [Test]
    public void FindParentWithHavingToSkipAParent() {
      _hierarchyBreaker.transform.parent = _parent.transform;
      _child.transform.parent = _hierarchyBreaker.transform;
      Assert.That(MjHierarchyTool.FindParentComponent(_child), Is.EqualTo(_parent));
    }

    [Test]
    public void FindParentWitNoMjComponentParent() {
      _child.transform.parent = _hierarchyBreaker.transform;
      Assert.That(MjHierarchyTool.FindParentComponent(_child), Is.Null);
    }

    [Test]
    public void InChainOfThreeTheCorrectImmediateParentIsPicked() {
      _child.transform.parent = _parent.transform;
      _secondChild.transform.parent = _child.transform;
      Assert.That(MjHierarchyTool.FindParentComponent(_secondChild), Is.EqualTo(_child));
    }
  }

  [TestFixture]
  public class HierachyLinearizationTests {
    public class FakeMjComponent : MjComponent {
      public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_GEOM;
      protected override void OnParseMjcf(XmlElement mjcf) {}
      protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
        return null;
      }
    }

    public class FakeGenericComponent : MonoBehaviour {}

    private FakeMjComponent _root;
    private FakeMjComponent _node01;
    private FakeMjComponent _node02;
    private FakeMjComponent _node11;
    private FakeMjComponent _node21;
    private FakeGenericComponent _hierarchyBreaker;

    [SetUp]
    public void SetUp() {
      _root =
          new GameObject("root", typeof(FakeMjComponent)).GetComponent<FakeMjComponent>();
      _node01 =
          new GameObject("node01", typeof(FakeMjComponent)).GetComponent<FakeMjComponent>();
      _node02 =
          new GameObject("node02", typeof(FakeMjComponent)).GetComponent<FakeMjComponent>();
      _node11 =
          new GameObject("node11", typeof(FakeMjComponent)).GetComponent<FakeMjComponent>();
      _node21 =
          new GameObject("node21", typeof(FakeMjComponent)).GetComponent<FakeMjComponent>();
      _hierarchyBreaker = new GameObject("hierarchyBreaker", typeof(FakeGenericComponent))
                              .GetComponent<FakeGenericComponent>();
    }

    [TearDown]
    public void TearDown() {
      GameObject.DestroyImmediate(_node21.gameObject);
      GameObject.DestroyImmediate(_node11.gameObject);
      GameObject.DestroyImmediate(_node02.gameObject);
      GameObject.DestroyImmediate(_node01.gameObject);
      GameObject.DestroyImmediate(_hierarchyBreaker.gameObject);
      GameObject.DestroyImmediate(_root.gameObject);
    }

    [Test]
    public void OneNodeHierarchy() {
      var hierarchy = MjHierarchyTool.LinearizeHierarchyBFS(_root.transform).ToArray();
      Assert.That(hierarchy, Has.Length.EqualTo(1));
      Assert.That(hierarchy[0], Is.EqualTo(_root));
    }

    [Test]
    public void OneLevelDeepTreeExclusivelyWithMjComponents() {
      _node01.transform.parent = _root.transform;
      _node02.transform.parent = _root.transform;
      var hierarchy = MjHierarchyTool.LinearizeHierarchyBFS(_root.transform).ToArray();
      Assert.That(hierarchy, Has.Length.EqualTo(3));
      Assert.That(hierarchy[0], Is.EqualTo(_root));
      Assert.That(hierarchy[1], Is.EqualTo(_node01));
      Assert.That(hierarchy[2], Is.EqualTo(_node02));
    }

    [Test]
    public void DeepTreeExclusivelyWithMjComponents() {
      _node01.transform.parent = _root.transform;
      _node02.transform.parent = _root.transform;
      _node11.transform.parent = _node01.transform;
      _node21.transform.parent = _node02.transform;
      var hierarchy = MjHierarchyTool.LinearizeHierarchyBFS(_root.transform).ToArray();
      Assert.That(hierarchy, Has.Length.EqualTo(5));
      Assert.That(hierarchy[0], Is.EqualTo(_root));
      Assert.That(hierarchy[1], Is.EqualTo(_node01));
      Assert.That(hierarchy[2], Is.EqualTo(_node02));
      Assert.That(hierarchy[3], Is.EqualTo(_node11));
      Assert.That(hierarchy[4], Is.EqualTo(_node21));
    }

    [Test]
    public void TreeWithAGenericComponent() {
      _node01.transform.parent = _root.transform;
      _hierarchyBreaker.transform.parent = _root.transform;
      _node02.transform.parent = _hierarchyBreaker.transform;
      var hierarchy = MjHierarchyTool.LinearizeHierarchyBFS(_root.transform).ToArray();
      Assert.That(hierarchy, Has.Length.EqualTo(3));
      Assert.That(hierarchy[0], Is.EqualTo(_root));
      Assert.That(hierarchy[1], Is.EqualTo(_node01));
      Assert.That(hierarchy[2], Is.EqualTo(_node02));
    }
  }

  [TestFixture]
  public class GetComponentsInImmediateChildrenTests {
    public class FakeComponentA : MonoBehaviour {}
    public class FakeComponentB : MonoBehaviour {}

    private FakeComponentA _root;
    private FakeComponentA _validChild;
    private FakeComponentB _invalidChild;
    private FakeComponentA _validGrandchild;

    [SetUp]
    public void SetUp() {
      _root = new GameObject("root", typeof(FakeComponentA)).GetComponent<FakeComponentA>();
      _validChild =
          new GameObject("validChild", typeof(FakeComponentA)).GetComponent<FakeComponentA>();
      _invalidChild =
          new GameObject("invalidChild", typeof(FakeComponentB)).GetComponent<FakeComponentB>();
      _validGrandchild =
          new GameObject("validGrandchild", typeof(FakeComponentA)).GetComponent<FakeComponentA>();
      _validGrandchild.transform.parent = _validChild.transform;
      _validChild.transform.parent = _root.transform;
    }

    [TearDown]
    public void TearDown() {
      GameObject.DestroyImmediate(_validGrandchild.gameObject);
      GameObject.DestroyImmediate(_invalidChild.gameObject);
      GameObject.DestroyImmediate(_validChild.gameObject);
      GameObject.DestroyImmediate(_root.gameObject);
    }

    [Test]
    public void RootNodeIsIgnoredInTheSearch() {
      var components =
          MjHierarchyTool.GetComponentsInImmediateChildren<FakeComponentA>(_root.transform);
      Assert.That(components, Has.No.Member(_root));
    }

    [Test]
    public void ImmediateValidChildIsReturned() {
      var components =
          MjHierarchyTool.GetComponentsInImmediateChildren<FakeComponentA>(_root.transform);
      Assert.That(components, Has.Member(_validChild));
    }

    [Test]
    public void ChildWithAnInvalidTypeIsIgnored() {
      var components =
          MjHierarchyTool.GetComponentsInImmediateChildren<FakeComponentA>(_root.transform);
      Assert.That(components, Has.No.Member(_invalidChild));
    }

    [Test]
    public void GrandchildrenAreIgnoredDuringSearch() {
      var components =
          MjHierarchyTool.GetComponentsInImmediateChildren<FakeComponentA>(_root.transform);
      Assert.That(components, Has.No.Member(_validGrandchild));
    }
  }
}
