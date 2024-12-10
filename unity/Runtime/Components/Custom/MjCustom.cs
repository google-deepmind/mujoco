using System;
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using UnityEngine;
using UnityEngine.Serialization;

namespace Mujoco {
public class MjCustom : MonoBehaviour {

  [SerializeField]
  private List<MjText> texts;

  [SerializeField]
  private List<MjNumeric> numerics;

  [SerializeField]
  private List<MjTuple> tuples;

  public static MjCustom Instance {
    get
    {
      if (_instance == null) {
        var instances = FindObjectsOfType<MjCustom>();
        if (instances.Length > 1) {
          throw new InvalidOperationException(
              "Only one MjCustom instance is allowed - please resolve manually.");
        } else if (instances.Length == 1) {
          _instance = instances[0];
        }
      }

      return _instance;
    }
  }

  private static MjCustom _instance = null;

  public static bool InstanceExists { get => _instance != null; }

  public void Awake() {
    if (_instance == null) {
      _instance = this;
    } else if (_instance != this) {
      throw new InvalidOperationException(
          "At most one MjCustom should be present.");
    }
  }


  public void ParseCustom(XmlElement child) {
      switch (child.Name) {
        case "numeric":
          numerics ??= new List<MjNumeric>();
          var numeric = new MjNumeric();
          numeric.Parse(child);
          numerics.Add(numeric);
          break;

        case "text":
          texts ??= new List<MjText>();
          var text = new MjText();
          text.Parse(child);
          texts.Add(text);
          break;

        case "tuple":
          tuples ??= new List<MjTuple>();
          var tuple = new MjTuple();
          tuple.Parse(child);
          tuples.Add(tuple);
          break;

        default:
          Debug.LogWarning($"Unknown custom element: {child.Name}");
          break;
      }
  }

  public void GenerateCustomMjcf(XmlDocument doc) {
    var mjcf = (XmlElement)doc.CreateElement("custom");
    foreach (var text in texts) {
      var textMjcf = text.ToMjcf(doc);
      mjcf.AppendChild(textMjcf);
    }
    foreach (var numeric in numerics) {
      var textMjcf = numeric.ToMjcf(doc);
      mjcf.AppendChild(textMjcf);
    }
    foreach (var tuple in tuples) {
      var textMjcf = tuple.ToMjcf(doc);
      mjcf.AppendChild(textMjcf);
    }
  }


  [Serializable]
  private abstract class MjCustomElement {

    [SerializeField]
    public string name;

    public void Parse(XmlElement mjcf) {
      name = mjcf.GetStringAttribute("name");
      ParseInner(mjcf);
    }

    protected abstract void ParseInner(XmlElement mjcf);

    public XmlElement ToMjcf(XmlDocument doc) {
      var mjcf = ToMjcfInner(doc);
      if (!string.IsNullOrEmpty(name)) {
        mjcf.SetAttribute("name", name);
      }

      return mjcf;
    }

    protected abstract XmlElement ToMjcfInner(XmlDocument doc);


  }

  [Serializable]
  private class MjText : MjCustomElement {

    [SerializeField]
    public string data;

    protected override void ParseInner(XmlElement mjcf) {
      data = mjcf.GetStringAttribute("data");
    }

    protected override XmlElement ToMjcfInner(XmlDocument doc) {
      var mjcf = (XmlElement)doc.CreateElement("numeric");

      mjcf.SetAttribute("data", data);

      return mjcf;
    }
  }


  [Serializable]
  private class MjNumeric : MjCustomElement {

    [SerializeField]
    public int size;

    [SerializeField]
    public float[] data;

    protected override void ParseInner(XmlElement mjcf) {
      if (int.TryParse(mjcf.GetStringAttribute("size", "-1"), out var size)) {
        this.size = size;
      }
      var data = mjcf.GetFloatArrayAttribute("data", new float[] { });

      if (this.size > -1 && data.Length != this.size) {
        Array.Resize(ref data, this.size);
      }
      this.data = data;
      this.size = this.data.Length;
    }

    protected override XmlElement ToMjcfInner(XmlDocument doc) {
      var mjcf = (XmlElement)doc.CreateElement("numeric");
      if (size >= 0) {
        mjcf.SetAttribute("size", MjEngineTool.MakeLocaleInvariant($"{size}"));
      }

      mjcf.SetAttribute("data", MjEngineTool.ArrayToMjcf(data));

      return mjcf;
    }
  }

  [Serializable]
  private class MjTuple : MjCustomElement {

    [SerializeField]
    protected List<TupleElement> tuples = new List<TupleElement>();

    protected override void ParseInner(XmlElement mjcf) {
      foreach (XmlElement child in mjcf.ChildNodes)
        if (child.Name == "element") {
          var tupleElement = new TupleElement();
          tupleElement.Parse(child);
          tuples.Add(tupleElement);
        } else {
          Debug.LogWarning($"Unknown child element in MjTuple: {child.Name}");
        }
    }

    protected override XmlElement ToMjcfInner(XmlDocument doc) {
      var mjcf = (XmlElement)doc.CreateElement("numeric");
      foreach (var tuple in tuples) {
        var tupleMjcf = tuple.ToMjcf(doc);
        mjcf.AppendChild(tupleMjcf);
      }

      return mjcf;
    }

    [Serializable]
    protected class TupleElement {

      [SerializeField]
      public string objType;

      [SerializeField]
      public string objName;

      [SerializeField]
      public float prm;

      public void Parse(XmlElement element) {
        objType = element.GetAttribute("objtype");
        objName = element.GetAttribute("objname");

        element.GetFloatAttribute("prm", float.NaN);
      }

      public XmlElement ToMjcf(XmlDocument doc) {
        var mjcf = (XmlElement)doc.CreateElement("element");

        mjcf.SetAttribute("objType", objType);
        mjcf.SetAttribute("objName", objName);
        if (!float.IsNaN(prm)) {
          mjcf.SetAttribute("prm", $"{prm}");
        }

        return mjcf;
      }

    }
  }
}
}