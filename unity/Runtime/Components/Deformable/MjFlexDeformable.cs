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
using System.Linq;
using System.Xml;
using UnityEditor;
using UnityEngine;

namespace Mujoco {

public class MjFlexDeformable : MjComponent {

  public string FlexName;

  public int Dim = 2;

  public float Radius = 0.005f;

  public MjBaseBody[] Body;

  public float[] Vertex;

  public float[] Texcoord;

  public int[] Element;

  public bool Flatskin;

  public int Group;

  // Edge, elasticity and contact get special treatment. As they are separate elements in
  // MJCF, and are unique to one instance each, they get toggles whether they are included
  // or not. If the toggle is false, they are not rendered via MjFlexDeformableEditor,
  // and are not added to the generated MJCF. This lets MuJoCo "do its thing" implicitly
  // when parsing the scene, which is more robust if, e.g., the defaults change (where 
  // forced explicit definitions silently alter the scene).

  // Due to their low complexity, no further hierarchy and uniqueness I opted not to have
  // separate Unity MonoBehaviours and GameObjects in Unity for the child elements of flex.
  // H
  public bool ConfigureEdge;

  [SerializeField]
  private MjFlexEdge Edge;

  public bool ConfigureContact;

  [SerializeField]
  public MjFlexContact Contact;

  public bool ConfigureElasticity;

  [SerializeField]
  public MjFlexElasticity Elasticity;

  protected override bool _suppressNameAttribute => true;

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_PLUGIN;

  // Parse the component settings from an external Mjcf.
  protected override void OnParseMjcf(XmlElement mjcf) {
    FlexName = mjcf.GetStringAttribute("name", "");
    Dim = mjcf.GetIntAttribute("dim", 2);
    Radius = mjcf.GetFloatAttribute("radius", 0.005f);
    Body = mjcf.GetStringAttribute("body").Split(" ", StringSplitOptions.RemoveEmptyEntries)
        .Select(MjHierarchyTool.FindComponentOfTypeAndName<MjBaseBody>).ToArray();
    Vertex = mjcf.GetFloatArrayAttribute("vertex", Array.Empty<float>());
    Texcoord = mjcf.GetFloatArrayAttribute("texcoord", Array.Empty<float>());
    Element = mjcf.GetIntArrayAttribute("element", Array.Empty<int>());
    Flatskin = mjcf.GetBoolAttribute("flatskin");
    Group = mjcf.GetIntAttribute("group");

    // Parse child elements
    var edgeElement = mjcf.SelectSingleNode("edge") as XmlElement;
    if (edgeElement != null) {
      ConfigureEdge = true;
      Edge = new MjFlexEdge();
      Edge.FromMjcf(edgeElement);
    } else {
      Edge = null;
    }

    var elasticityElement = mjcf.SelectSingleNode("elasticity") as XmlElement;
    if (elasticityElement != null) {
      ConfigureElasticity = true;
      Elasticity = new MjFlexElasticity();
      Elasticity.FromMjcf(elasticityElement);
    } else {
      Elasticity = null;
    }

    var contactElement = mjcf.SelectSingleNode("contact") as XmlElement;
    if (contactElement != null) {
      ConfigureContact = true;
      Contact = new MjFlexContact();
      Contact.FromMjcf(contactElement);
    } else {
      Contact = null;
    }
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    var mjcf = (XmlElement)doc.CreateElement("flex");
    mjcf.SetAttribute("name", MjEngineTool.MakeLocaleInvariant($"{FlexName}"));
    mjcf.SetAttribute("dim", MjEngineTool.MakeLocaleInvariant($"{Dim}"));
    mjcf.SetAttribute("radius", MjEngineTool.MakeLocaleInvariant($"{Radius}"));
    if (Body.Length > 0)
      mjcf.SetAttribute("body", MjEngineTool.ArrayToMjcf(Body.Select(b => b.MujocoName).ToArray()));
    if (Vertex.Length > 0)
      mjcf.SetAttribute("vertex", MjEngineTool.ArrayToMjcf(Vertex));
    if (Texcoord.Length > 0)
      mjcf.SetAttribute("texcoord", MjEngineTool.ArrayToMjcf(Texcoord));
    if (Element.Length > 0)
      mjcf.SetAttribute("element", MjEngineTool.ArrayToMjcf(Element));
    mjcf.SetAttribute("flatskin",
        MjEngineTool.MakeLocaleInvariant($"{(Flatskin ? "true" : "false")}"));
    mjcf.SetAttribute("group", MjEngineTool.MakeLocaleInvariant($"{Group}"));

    // Add child elements if configured
    if (ConfigureEdge && Edge != null) {
      var edgeElement = doc.CreateElement("edge");
      Edge.ToMjcf(edgeElement);
      mjcf.AppendChild(edgeElement);
    }

    if (ConfigureElasticity && Elasticity != null) {
      var elasticityElement = doc.CreateElement("elasticity");
      Elasticity.ToMjcf(elasticityElement);
      mjcf.AppendChild(elasticityElement);
    }

    if (ConfigureContact && Contact != null) {
      var contactElement = doc.CreateElement("contact");
      Contact.ToMjcf(contactElement);
      mjcf.AppendChild(contactElement);
    }

    return mjcf;
  }

  [Serializable]
  public class MjFlexEdge {
    [SerializeField]
    public float Stiffness = 0;

    [SerializeField]
    public float Damping = 0;

    public void ToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("stiffness", MjEngineTool.MakeLocaleInvariant($"{Stiffness}"));
      mjcf.SetAttribute("damping", MjEngineTool.MakeLocaleInvariant($"{Damping}"));
    }

    // We check the attribute and only overwrite the field instead of giving a default to
    // the attribute getter. This way the default only lives in one place (the initializer)
    // so it may only need to be changed there.
    public void FromMjcf(XmlElement mjcf) {
      if (mjcf.HasAttribute("stiffness"))
        Stiffness = mjcf.GetFloatAttribute("stiffness");
      if (mjcf.HasAttribute("damping"))
        Damping = mjcf.GetFloatAttribute("damping");
    }
  }

  [Serializable]
  public class MjFlexElasticity {
    [SerializeField]
    public float Young = 0;

    [SerializeField]
    public float Poisson = 0;

    [SerializeField]
    public float Damping = 0;

    [SerializeField]
    public float Thickness = -1;

    public void ToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("young", MjEngineTool.MakeLocaleInvariant($"{Young}"));
      mjcf.SetAttribute("poisson", MjEngineTool.MakeLocaleInvariant($"{Poisson}"));
      mjcf.SetAttribute("damping", MjEngineTool.MakeLocaleInvariant($"{Damping}"));
      mjcf.SetAttribute("thickness", MjEngineTool.MakeLocaleInvariant($"{Thickness}"));
    }

    public void FromMjcf(XmlElement mjcf) {
      if (mjcf.HasAttribute("young"))
        Young = mjcf.GetFloatAttribute("young");
      if (mjcf.HasAttribute("poisson"))
        Poisson = mjcf.GetFloatAttribute("poisson");
      if (mjcf.HasAttribute("damping"))
        Damping = mjcf.GetFloatAttribute("damping");
      if (mjcf.HasAttribute("thickness"))
        Thickness = mjcf.GetFloatAttribute("thickness");
    }
  }

  [Serializable]
  public class MjFlexContact {
    [SerializeField]
    public bool Internal = true;

    [SerializeField]
    public string Selfcollide = "auto"; // Would an enum be better?

    [SerializeField]
    public int Activelayers = 1;

    public void ToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("internal",
          MjEngineTool.MakeLocaleInvariant($"{(Internal ? "true" : "false")}"));
      mjcf.SetAttribute("selfcollide", Selfcollide);
      mjcf.SetAttribute("activelayers", MjEngineTool.MakeLocaleInvariant($"{Activelayers}"));
    }

    public void FromMjcf(XmlElement mjcf) {
      if (mjcf.HasAttribute("internal"))
        Internal = mjcf.GetBoolAttribute("internal");
      if (mjcf.HasAttribute("selfcollide"))
        Selfcollide = mjcf.GetStringAttribute("selfcollide");
      if (mjcf.HasAttribute("activelayers"))
        Activelayers = mjcf.GetIntAttribute("activelayers");
    }
  }


}
}