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
using System.Xml;
using UnityEngine;

namespace Mujoco {

  [Serializable]
  public struct MjJointSettings {
    [Tooltip("Added inertia of all DoFs created by this joint.")]
    public float Armature;

    [Tooltip("Joint spring settings.")]
    public SpringSettings Spring;

    [Tooltip("Joint solver settings.")]
    public SolverSettings Solver;

    // Default joint settings.
    public static MjJointSettings Default = new MjJointSettings() {
      Solver = SolverSettings.Default,
    };

    public void FromMjcf(XmlElement mjcf) {
      Armature = mjcf.GetFloatAttribute("armature", 0.0f);
      Spring.FromMjcf(mjcf);
      Solver.FromMjcf(mjcf);
    }

    public void ToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("armature", MjEngineTool.MakeLocaleInvariant($"{Armature}"));
      Spring.ToMjcf(mjcf);
      Solver.ToMjcf(mjcf);
    }
  }

  [Serializable]
  public struct SpringSettings {
    public float TimeConstant;
    public float DampingRatio;

    [Tooltip("Joint stiffness. If this value is positive, a spring will be created.")]
    public float Stiffness;
    [Tooltip("Damping applied to all degrees of freedom created by this joint.")]
    public float Damping;

    [Tooltip("Spring equilibrium pose.")]
    public float EquilibriumPose;

    public void ToMjcf(XmlElement mjcf) {
      // Spring settings.
      mjcf.SetAttribute("springref", MjEngineTool.MakeLocaleInvariant($"{EquilibriumPose}"));
      mjcf.SetAttribute("springdamper", MjEngineTool.MakeLocaleInvariant($"{TimeConstant} {DampingRatio}"));
      mjcf.SetAttribute("damping", MjEngineTool.MakeLocaleInvariant($"{Damping}"));
      mjcf.SetAttribute("stiffness", MjEngineTool.MakeLocaleInvariant($"{Stiffness}"));
    }

    public void FromMjcf(XmlElement mjcf) {
      var springDamper = mjcf.GetVector2Attribute("springdamper", Vector2.zero);
      TimeConstant = springDamper.x;
      DampingRatio = springDamper.y;
      EquilibriumPose = MjSceneImportSettings.AnglesInDegrees? mjcf.GetFloatAttribute("springref", 0.0f) : mjcf.GetFloatAttribute("springref", 0.0f) * Mathf.Rad2Deg;
      Damping = mjcf.GetFloatAttribute("damping", 0.0f);
      Stiffness = mjcf.GetFloatAttribute("stiffness", 0.0f);
    }
  }

  [Serializable]
  public struct SolverImpedance {
    public float DMin;
    public float DMax;
    public float Width;
    public float Midpoint;
    public float Power;

    public static SolverImpedance Default =
        new SolverImpedance() { DMin = 0.9f, DMax = 0.95f, Width = 0.001f, Midpoint = 0.5f,
                                Power = 2.0f };

    public void FromMjcf(XmlElement element, string attributeName) {
      var values = element.GetFloatArrayAttribute(
          attributeName, new float[] { Default.DMin, Default.DMax, Default.Width, Default.Midpoint,
                                       Default.Power });
      DMin = values[0];
      DMax = values[1];
      Width = values[2];
      Midpoint = values[3];
      Power = values[4];
    }

    public void ToMjcf(XmlElement element, string attributeName) {
      element.SetAttribute(attributeName, MjEngineTool.MakeLocaleInvariant($"{DMin} {DMax} {Width} {Midpoint} {Power}"));
    }
  }

  [Serializable]
  public struct SolverReference {
    public float TimeConst;
    public float DampRatio;

    public static SolverReference Default =
        new SolverReference() { TimeConst = 0.02f, DampRatio = 1.0f };

    public void FromMjcf(XmlElement element, string attributeName) {
      var values = element.GetFloatArrayAttribute(
          attributeName, new float[] { Default.TimeConst, Default.DampRatio });
      TimeConst = values[0];
      DampRatio = values[1];
    }

    public void ToMjcf(XmlElement element, string attributeName) {
      element.SetAttribute(attributeName, MjEngineTool.MakeLocaleInvariant($"{TimeConst} {DampRatio}"));
    }
  }

  [Serializable]
  public struct SolverSettings {
    [Tooltip("Does the joint have limits.")]
    public bool Limited;

    [Tooltip("Limit margin - always in radians/meters.")]
    public float Margin;
    public SolverReference RefLimit;
    public SolverImpedance ImpLimit;

    [Tooltip("Friction loss due to dry friction.")]
    public float FrictionLoss;
    public SolverImpedance ImpFriction;
    public SolverReference RefFriction;

    public static SolverSettings Default = new SolverSettings() {
      ImpFriction = SolverImpedance.Default,
      ImpLimit = SolverImpedance.Default,
      RefFriction = SolverReference.Default,
      RefLimit = SolverReference.Default,
    };

    public void ToMjcf(XmlElement mjcf) {
      RefLimit.ToMjcf(mjcf, "solreflimit");
      ImpLimit.ToMjcf(mjcf, "solimplimit");
      RefFriction.ToMjcf(mjcf, "solreffriction");
      ImpFriction.ToMjcf(mjcf, "solimpfriction");
      mjcf.SetAttribute("frictionloss", MjEngineTool.MakeLocaleInvariant($"{FrictionLoss}"));
      mjcf.SetAttribute("limited", $"{Limited}".ToLower());
      mjcf.SetAttribute("margin", MjEngineTool.MakeLocaleInvariant($"{Margin}"));
    }

    public void FromMjcf(XmlElement mjcf) {
      RefLimit.FromMjcf(mjcf, "solreflimit");
      ImpLimit.FromMjcf(mjcf, "solimplimit");
      RefFriction.FromMjcf(mjcf, "solreffriction");
      ImpFriction.FromMjcf(mjcf, "solimpfriction");
      FrictionLoss = mjcf.GetFloatAttribute("frictionloss", 0.0f);

      Limited = mjcf.GetLimitedAttribute("limited", mjcf.HasAttribute("range"));
      Margin = mjcf.GetFloatAttribute("margin", defaultValue: 0.0f);
    }
  }
}
