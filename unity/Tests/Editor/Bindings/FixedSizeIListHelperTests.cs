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

﻿using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Xml;
using NUnit.Framework;
using UnityEngine;

namespace Mujoco {

[TestFixture]
public class FixedSizeIListHelperTests {
  public class Example : FixedSizeIListHelper<string> {
    public override string this[int index] {
      get {
        switch (index) {
          case 0: return _element0;
          case 1: return _element1;
        }
        return "";
      }
      set {
        switch (index) {
          case 0: _element0 = value; break;
          case 1: _element1 = value; break;
        }
      }
    }

    public override int Count => 2;

    private string _element0;
    private string _element1;
  }

  [Test]
  public void CanReadThroughIListInterface() {
    var example = new Example();
    IList<string> ilist = example;
    example[0] = "a";

    Assert.That(ilist[0], Is.EqualTo("a"));
  }

  [Test]
  public void CanWriteThroughIListInterface() {
    var example = new Example();
    IList<string> ilist = example;
    ilist[1] = "b";

    Assert.That(example[1], Is.EqualTo("b"));
  }

  [Test]
  public void EnumeratorWorks() {
    var example = new Example();
    example[0] = "a";
    example[1] = "b";

    var actual = new List<string>();
    foreach (var element in example) {
      actual.Add(element);
    }
    Assert.That(actual.Count, Is.EqualTo(2));
    Assert.That(actual[0], Is.EqualTo("a"));
    Assert.That(actual[1], Is.EqualTo("b"));
  }

  [Test]
  public void AddNotSupported() {
    var example = new Example();
    Assert.Throws<NotSupportedException>(() => example.Add(""));
  }

  [Test]
  public void ClearNotSupported() {
    var example = new Example();
    Assert.Throws<NotSupportedException>(() => example.Clear());
  }

  [Test]
  public void RemoveNotSupported() {
    var example = new Example();
    Assert.Throws<NotSupportedException>(() => example.Remove(""));
  }

  [Test]
  public void InsertNotSupported() {
    var example = new Example();
    Assert.Throws<NotSupportedException>(() => example.Insert(0, ""));
  }

  [Test]
  public void RemoveAtNotSupported() {
    var example = new Example();
    Assert.Throws<NotSupportedException>(() => example.RemoveAt(0));
  }

  [Test]
  public void ContainsFindsItems() {
    var example = new Example();
    example[0] = "a";
    example[1] = "b";
    Assert.That(example.Contains("a"));
    Assert.That(example.Contains("b"));
  }

  [Test]
  public void ContainsDoesntFindAbsentItems() {
    var example = new Example();
    example[0] = "a";
    example[1] = "b";
    Assert.That(!example.Contains("c"));
  }

  [Test]
  public void CopyTo() {
    var example = new Example();
    example[0] = "a";
    example[1] = "b";

    var buffer = new string[7];
    example.CopyTo(buffer, 3);

    var expected = new string[] {
      null,
      null,
      null,
      "a",
      "b",
      null,
      null,
    };

    Assert.That(buffer, Is.EqualTo(expected));
  }

  [Test]
  public void IndexOfFindsItems() {
    var example = new Example();
    example[0] = "a";
    example[1] = "b";
    Assert.That(0, Is.EqualTo(example.IndexOf("a")));
    Assert.That(1, Is.EqualTo(example.IndexOf("b")));
  }

  [Test]
  public void IndexOfDoesntFindAbsentItems() {
    var example = new Example();
    example[0] = "a";
    example[1] = "b";
    Assert.That(-1, Is.EqualTo(example.IndexOf("c")));
  }
}
}
