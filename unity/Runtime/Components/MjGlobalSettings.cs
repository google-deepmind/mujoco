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
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Xml;
using UnityEngine;
using UnityEngine.Profiling;

namespace Mujoco {

// The MuJoCo parser is case-sensitive.  The case format of the entries in the enums below is
// exactly what the documentation specifies: http://mujoco.org/book/XMLreference.html#option .
public enum IntegratorType {
  Euler,
  RK4
}

public enum CollisionCheckType {
  all
}

public enum FrictionConeType {
  pyramidal,
  elliptic
}

public enum JacobianType {
  dense,
  sparse,
  auto
}

public enum ConstraintSolverType {
  PGS,
  CG,
  Newton
}

public enum EnableDisableFlag {
  enable,
  disable
}

[Serializable]
public struct MjcfOptionFlag {
  public EnableDisableFlag Constraint;
  public EnableDisableFlag Equality;
  public EnableDisableFlag FrictionLoss;
  public EnableDisableFlag Limit;
  public EnableDisableFlag Contact;
  public EnableDisableFlag Passive;
  public EnableDisableFlag Gravity;
  public EnableDisableFlag ClampCtrl;
  public EnableDisableFlag WarmStart;
  public EnableDisableFlag FilterParent;
  public EnableDisableFlag Actuation;
  public EnableDisableFlag RefSafe;
  public EnableDisableFlag Override;
  public EnableDisableFlag Energy;
  public EnableDisableFlag FwdInv;
  public EnableDisableFlag SensorNoise;
  public EnableDisableFlag MultiCCD;
  public static MjcfOptionFlag Default = new MjcfOptionFlag() {
    Constraint = EnableDisableFlag.enable,
    Equality = EnableDisableFlag.enable,
    FrictionLoss = EnableDisableFlag.enable,
    Limit = EnableDisableFlag.enable,
    Contact = EnableDisableFlag.enable,
    Passive = EnableDisableFlag.enable,
    Gravity = EnableDisableFlag.enable,
    ClampCtrl = EnableDisableFlag.enable,
    WarmStart = EnableDisableFlag.enable,
    FilterParent = EnableDisableFlag.enable,
    Actuation = EnableDisableFlag.enable,
    RefSafe = EnableDisableFlag.enable,
    Override = EnableDisableFlag.disable,
    Energy = EnableDisableFlag.disable,
    FwdInv = EnableDisableFlag.disable,
    SensorNoise = EnableDisableFlag.disable,
    MultiCCD = EnableDisableFlag.disable
  };

  public void FromMjcf(XmlElement mjcf) {
    var localDefault = MjcfOptionFlag.Default;
    Constraint = mjcf.GetEnumAttribute<EnableDisableFlag>("constraint", localDefault.Constraint);
    Equality = mjcf.GetEnumAttribute<EnableDisableFlag>("equality", localDefault.Equality);
    FrictionLoss = mjcf.GetEnumAttribute<EnableDisableFlag>("frictionloss",
                                                            localDefault.FrictionLoss);
    Limit = mjcf.GetEnumAttribute<EnableDisableFlag>("limit", localDefault.Limit);
    Contact = mjcf.GetEnumAttribute<EnableDisableFlag>("contact", localDefault.Contact);
    Passive = mjcf.GetEnumAttribute<EnableDisableFlag>("passive", localDefault.Passive);
    Gravity = mjcf.GetEnumAttribute<EnableDisableFlag>("gravity", localDefault.Gravity);
    ClampCtrl = mjcf.GetEnumAttribute<EnableDisableFlag>("clampctrl", localDefault.ClampCtrl);
    WarmStart = mjcf.GetEnumAttribute<EnableDisableFlag>("warmstart", localDefault.WarmStart);
    FilterParent = mjcf.GetEnumAttribute<EnableDisableFlag>("filterparent",
                                                            localDefault.FilterParent);
    Actuation = mjcf.GetEnumAttribute<EnableDisableFlag>("actuation", localDefault.Actuation);
    RefSafe = mjcf.GetEnumAttribute<EnableDisableFlag>("refsafe", localDefault.RefSafe);
    Override = mjcf.GetEnumAttribute<EnableDisableFlag>("override", localDefault.Override);
    Energy = mjcf.GetEnumAttribute<EnableDisableFlag>("energy", localDefault.Energy);
    FwdInv = mjcf.GetEnumAttribute<EnableDisableFlag>("fwdinv", localDefault.FwdInv);
    SensorNoise = mjcf.GetEnumAttribute<EnableDisableFlag>("sensornoise", localDefault.SensorNoise);
    MultiCCD = mjcf.GetEnumAttribute<EnableDisableFlag>("multiccd", localDefault.MultiCCD);
  }

  public void ToMjcf(XmlElement mjcf) {
    mjcf.SetAttribute("constraint", Constraint.ToString());
    mjcf.SetAttribute("equality", Equality.ToString());
    mjcf.SetAttribute("frictionloss", FrictionLoss.ToString());
    mjcf.SetAttribute("limit", Limit.ToString());
    mjcf.SetAttribute("contact", Contact.ToString());
    mjcf.SetAttribute("passive", Passive.ToString());
    mjcf.SetAttribute("gravity", Gravity.ToString());
    mjcf.SetAttribute("clampctrl", ClampCtrl.ToString());
    mjcf.SetAttribute("warmstart", WarmStart.ToString());
    mjcf.SetAttribute("filterparent", FilterParent.ToString());
    mjcf.SetAttribute("actuation", Actuation.ToString());
    mjcf.SetAttribute("refsafe", RefSafe.ToString());
    mjcf.SetAttribute("override", Override.ToString());
    mjcf.SetAttribute("energy", Energy.ToString());
    mjcf.SetAttribute("fwdinv", FwdInv.ToString());
    mjcf.SetAttribute("sensornoise", SensorNoise.ToString());
    mjcf.SetAttribute("multiccd", MultiCCD.ToString());
  }
}

[Serializable]
public struct MjSizeStruct {
  public int Njmax;
  public int Nconmax;
  public int Nstack;
  public static MjSizeStruct Default = new MjSizeStruct() {
    Njmax = 0,
    Nconmax = 0,
    Nstack = 0
  };

  public void ParseMjcf(XmlElement mjcf) {
    var localDefault = MjSizeStruct.Default;
    Njmax = (int)mjcf.GetFloatAttribute("njmax", localDefault.Njmax);
    Nconmax = (int)mjcf.GetFloatAttribute("nconmax", localDefault.Nconmax);
    Nstack = (int)mjcf.GetFloatAttribute("nstack", localDefault.Nstack);
  }

  public XmlElement ToMjcf(XmlElement mjcf) {
    if (Njmax > 0) {
      mjcf.SetAttribute("njmax", $"{Njmax}");
    }
    if (Nconmax > 0) {
      mjcf.SetAttribute("nconmax", $"{Nconmax}");
    }
    if (Nstack > 0) {
      mjcf.SetAttribute("nstack", $"{Nstack}");
    }
    return mjcf;
  }
}

[Serializable]
public struct MjOptionStruct {

  // "timestep" and "gravity" come from global settings.
  // "apirate" is only relevant to HAPTIX.
  [Tooltip("Ratio of frictional-to-normal constraint impedance.")]
  public float ImpRatio;
  [Tooltip("Global magnetic flux used by magnetometer sensors.")]
  public Vector3 Magnetic;
  [Tooltip("Velocity vector of the medium, scales with the values of Density and Viscosity below.")]
  public Vector3 Wind;
  [Tooltip("Density of the medium for lift and drag forces. 0 disables lift and drag forces.")]
  public float Density;
  [Tooltip("Viscosity of the medium for viscous forces. 0 disables viscous forces.")]
  public float Viscosity;
  [Tooltip("Contact pair margin used when ContactOverride flag is set.")]
  public float OverrideMargin;
  [Tooltip("Contact pair solref used when ContactOverride flag is set.")]
  public SolverReference OverrideSolRef;
  [Tooltip("Contact pair solimp used when ContactOverride flag is set.")]
  public SolverImpedance OverrideSolImp;
  [Tooltip("Numerical integrator.")]
  public IntegratorType Integrator;
  [Tooltip("What collision types should be checked.")]
  public CollisionCheckType Collision;
  [Tooltip("How to model the friction cone.")]
  public FrictionConeType Cone;
  [Tooltip("How to represent the constraint Jacobian.")]
  public JacobianType Jacobian;
  [Tooltip("Constraint solver algorithm.")]
  public ConstraintSolverType Solver;
  [Tooltip("Maximum iterations for the constraint solver.")]
  public int Iterations;
  [Tooltip("Threshold used for early termination of the constraint solver.")]
  public float Tolerance;
  [Tooltip("Maximum iterations for the no-slip phase of the constraint solver.")]
  public int NoSlipIterations;
  [Tooltip("Threshold used for early termination of the Noslip solver.")]
  public float NoSlipTolerance;
  [Tooltip("Maximum iterations for convex mesh collisions.")]
  public int MprIterations;
  [Tooltip("Threshold used for early termination of the MPR algorithm.")]
  public float MprTolerance;

  public MjcfOptionFlag Flag;

  // Default values copied from the "XML reference" page in the MuJoCo documentation.
  public static MjOptionStruct Default = new MjOptionStruct() {
    ImpRatio = 1.0f,
    Magnetic = Vector3.zero,
    Wind = new Vector3(0.0f, -0.5f, 0.0f),
    Density = 0.0f,
    Viscosity = 0.0f,
    OverrideMargin = 0.0f,
    OverrideSolRef = SolverReference.Default,
    OverrideSolImp = SolverImpedance.Default,
    Integrator = IntegratorType.Euler,
    Collision = CollisionCheckType.all,
    Cone = FrictionConeType.pyramidal,
    Jacobian = JacobianType.auto,
    Solver = ConstraintSolverType.Newton,
    Iterations = 100,
    Tolerance = 1e-8f,
    NoSlipIterations = 0,
    NoSlipTolerance = 1e-6f,
    MprIterations = 50,
    MprTolerance = 1e-6f,
    Flag = MjcfOptionFlag.Default
  };

  public void ParseMjcf(XmlElement mjcf) {
    if (mjcf.HasAttribute("timestep")) {
      var mjTimestep = mjcf.GetFloatAttribute("timestep", Time.fixedDeltaTime);
      if (mjTimestep != Time.fixedDeltaTime) {
        Debug.unityLogger.LogWarning(
            "MuJoCo",
            $"MuJoCo's timestep ({mjTimestep}s) ignored, please set fixedDeltaTime manually.",
            null);
      }
    }
    if (mjcf.HasAttribute("gravity")) {
      var mjGravity = mjcf.GetVector3Attribute("gravity", Vector3.zero);
      Debug.unityLogger.LogWarning(
          "MuJoCo",
          $"MuJoCo's gravity ({mjGravity.x} {mjGravity.z} {mjGravity.y}) ignored, " +
          "please set gravity in the Physics Manager.",
          null);
    }
    var localDefault = MjOptionStruct.Default;
    ImpRatio = mjcf.GetFloatAttribute("impratio", localDefault.ImpRatio);
    Magnetic = mjcf.GetVector3Attribute("magnetic", localDefault.Magnetic);
    Wind = mjcf.GetVector3Attribute("wind", localDefault.Wind);
    Density = mjcf.GetFloatAttribute("density", localDefault.Density);
    Viscosity = mjcf.GetFloatAttribute("viscosity", localDefault.Viscosity);
    OverrideMargin = mjcf.GetFloatAttribute("o_margin", localDefault.OverrideMargin);

    OverrideSolRef.FromMjcf(mjcf, "o_solref");
    OverrideSolImp.FromMjcf(mjcf, "o_solimp");

    Integrator = mjcf.GetEnumAttribute<IntegratorType>("integrator", localDefault.Integrator);
    Collision = mjcf.GetEnumAttribute<CollisionCheckType>("collision", localDefault.Collision);
    Cone = mjcf.GetEnumAttribute<FrictionConeType>("cone", localDefault.Cone);
    Jacobian = mjcf.GetEnumAttribute<JacobianType>("jacobian", localDefault.Jacobian);
    Solver = mjcf.GetEnumAttribute<ConstraintSolverType>("solver", localDefault.Solver);

    Iterations = (int)mjcf.GetFloatAttribute("iterations", localDefault.Iterations);
    Tolerance = mjcf.GetFloatAttribute("tolerance", localDefault.Tolerance);
    NoSlipIterations = (int)mjcf.GetFloatAttribute(
        "noslip_iterations", localDefault.NoSlipIterations);
    NoSlipTolerance = mjcf.GetFloatAttribute("noslip_tolerance", localDefault.NoSlipTolerance);
    MprIterations = (int)mjcf.GetFloatAttribute("mpr_iterations", localDefault.MprIterations);
    MprTolerance = mjcf.GetFloatAttribute("mpr_tolerance", localDefault.MprTolerance);

    var flagElements = mjcf.GetElementsByTagName("flag");
    if (flagElements.Count == 1) {
      Flag.FromMjcf(flagElements[0] as XmlElement);
    } else if (flagElements.Count > 1) {
      throw new ArgumentException("More than one <flag> node is not supported.");
    }
  }

  public XmlElement ToMjcf(XmlElement mjcf) {
    mjcf.SetAttribute("impratio", MjEngineTool.MakeLocaleInvariant($"{ImpRatio}"));

    mjcf.SetAttribute("magnetic", MjEngineTool.MakeLocaleInvariant($"{Magnetic.x} {Magnetic.y} {Magnetic.z}"));
    mjcf.SetAttribute("wind", MjEngineTool.MakeLocaleInvariant($"{Wind.x} {Wind.y} {Wind.z}"));

    mjcf.SetAttribute("density", MjEngineTool.MakeLocaleInvariant($"{Density}"));
    mjcf.SetAttribute("viscosity", MjEngineTool.MakeLocaleInvariant($"{Viscosity}"));
    mjcf.SetAttribute("o_margin", MjEngineTool.MakeLocaleInvariant($"{OverrideMargin}"));

    OverrideSolRef.ToMjcf(mjcf, "o_solref");
    OverrideSolImp.ToMjcf(mjcf, "o_solimp");

    mjcf.SetAttribute("integrator", Integrator.ToString());
    mjcf.SetAttribute("collision", Collision.ToString());
    mjcf.SetAttribute("cone", Cone.ToString());
    mjcf.SetAttribute("jacobian", Jacobian.ToString());
    mjcf.SetAttribute("solver", Solver.ToString());

    mjcf.SetAttribute("iterations", MjEngineTool.MakeLocaleInvariant($"{Iterations}"));
    mjcf.SetAttribute("tolerance", MjEngineTool.MakeLocaleInvariant($"{Tolerance}"));
    mjcf.SetAttribute("noslip_iterations", MjEngineTool.MakeLocaleInvariant($"{NoSlipIterations}"));
    mjcf.SetAttribute("noslip_tolerance", MjEngineTool.MakeLocaleInvariant($"{NoSlipTolerance}"));
    mjcf.SetAttribute("mpr_iterations", MjEngineTool.MakeLocaleInvariant($"{MprIterations}"));
    mjcf.SetAttribute("mpr_tolerance", MjEngineTool.MakeLocaleInvariant($"{MprTolerance}"));

    var flags = (XmlElement)mjcf.AppendChild(
        mjcf.OwnerDocument.CreateElement("flag"));
    Flag.ToMjcf(flags);
    return mjcf;
  }
}

public class MjGlobalSettings : MonoBehaviour {

  [Tooltip("Filename for the generated scene XML.")]
  public string DebugFileName;

  [Tooltip("Scales the force applied by the mouse spring.")]
  public float MouseSpringStiffness = 100;

  public MjOptionStruct GlobalOptions = MjOptionStruct.Default;

  public MjSizeStruct GlobalSizes = MjSizeStruct.Default;

  public static MjGlobalSettings Instance {
    get {
     if (_instance == null) {
        var instances = FindObjectsOfType<MjGlobalSettings>();
        if (instances.Length > 1) {
          throw new InvalidOperationException(
              "Only one MjGlobalSettings instance is allowed - please resolve manually.");
        } else if (instances.Length == 1) {
          _instance = instances[0];
        }
      }
      return _instance;
    }
  }

  private static MjGlobalSettings _instance = null;

  public void Awake() {
    if (_instance == null) {
      _instance = this;
    } else if (_instance != this) {
      throw new InvalidOperationException(
          "At most one MjGlobalSettings should be present.");
    }
  }

  public void ParseOptionSizeMjcf(XmlElement option, XmlElement size) {
    if (option != null) {
      GlobalOptions.ParseMjcf(option);
    }
    if (size != null) {
      GlobalSizes.ParseMjcf(size);
    }
  }

  public void OptionSizeToMjcf(XmlElement option, XmlElement size) {
    GlobalOptions.ToMjcf(option);
    GlobalSizes.ToMjcf(size);
  }
}
}
