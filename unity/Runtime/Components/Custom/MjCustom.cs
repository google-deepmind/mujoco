using System;
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using UnityEngine;

namespace Mujoco {
public class MjCustom : MonoBehaviour {
  public void ParseCustom(GameObject parentObject, XmlElement parentNode) {
    foreach (XmlElement child in parentNode.ChildNodes) {
      switch (child.Name) {
        case "numeric":
          var numeric = new MjNumeric();
          numeric.Parse(child);
          break;

        case "text":
          var text = new MjText();
          text.Parse(child);
          break;

        case "tuple":
          var tuple = new MjTuple();
          tuple.Parse(child);
          break;

        default:
          Debug.LogWarning($"Unknown custom element: {child.Name}");
          break;
      }
    }
  }

  private class MjText {
    public string Name { get; private set; }

    public string Data { get; private set; }

    public void Parse(XmlElement mjcf) {
      Name = mjcf.GetStringAttribute("name");
      Data = mjcf.GetStringAttribute("data");
    }
  }

  private class MjNumeric {
    public string Name { get; private set; }

    public int Size { get; private set; }

    public float[] Data { get; private set; }

    public void Parse(XmlElement mjcf) {
      Name = mjcf.GetStringAttribute("name");

      if (int.TryParse(mjcf.GetStringAttribute("size", "-1"), out int size)) {
        Size = size;
      }
      var data = mjcf.GetFloatArrayAttribute("data", defaultValue: new float[]{});

      if (Size>-1 && data.Length != Size) {
        Array.Resize(ref data, Size);
      }
      Data = data;
      Size = Data.Length;
    }
  }

  private class MjTuple {
    public string Name { get; private set; }

    public List<TupleElement> Elements { get; private set; } = new List<TupleElement>();

    public void Parse(XmlElement element) {
      Name = element.GetAttribute("name");
      if (string.IsNullOrEmpty(Name)) {
        throw new ArgumentException("MjTuple element must have a 'name' attribute.");
      }

      foreach (XmlElement child in element.ChildNodes) {
        if (child.Name == "element") {
          var tupleElement = new TupleElement();
          tupleElement.Parse(child);
          Elements.Add(tupleElement);
        } else {
          Debug.LogWarning($"Unknown child element in MjTuple: {child.Name}");
        }
      }
    }

    public class TupleElement {
      public string ObjType { get; private set; }

      public string ObjName { get; private set; }

      public double Prm { get; private set; }

      public void Parse(XmlElement element) {
        ObjType = element.GetAttribute("objtype");
        if (string.IsNullOrEmpty(ObjType)) {
          throw new ArgumentException("Tuple element must have an 'objtype' attribute.");
        }

        ObjName = element.GetAttribute("objname");
        if (string.IsNullOrEmpty(ObjName)) {
          throw new ArgumentException("Tuple element must have an 'objname' attribute.");
        }

        if (double.TryParse(element.GetAttribute("prm"), out double prm)) {
          Prm = prm;
        }
      }
    }
  }
}
}