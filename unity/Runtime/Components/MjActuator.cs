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
using System.Collections.Generic;
using System.Linq;
using System.Xml;
using UnityEngine;

namespace Mujoco {
// Actuators provide means to set joints in motion.
public class MjActuator : MjComponent {

  public enum ActuatorType {
    General,
    Motor,
    Position,
    Velocity,
    Cylinder,
    Muscle
  }

  // This structure holds the parameters shared by all types of actuators.
  //
  // All constants found in this class are copied from the official documentation, and can be found
  // here: http://mujoco.org/book/XMLreference.html#actuator
  [Serializable]
  public class CommonParameters {

    // If true, the control input to this actuator is automatically clamped to ctrlrange at runtime.
    // If false, control input clamping is disabled.
    public bool CtrlLimited;

    // If true, the force output of this actuator is automatically clamped to forcerange at runtime.
    // If false, force output clamping is disabled.
    public bool ForceLimited;

    // Range for clamping the control input.
    public Vector2 CtrlRange;

    // Range for clamping the force output.
    public Vector2 ForceRange;

    // Range of feasible lengths of the actuator's transmission.
    public Vector2 LengthRange;

    // This attribute scales the length (and consequently moment arms, velocity and force) of the
    // actuator, for all transmission types. It is different from the gain in the force generation
    // mechanism, because the gain only scales the force output and does not affect the length,
    // moment arms and velocity.
    public List<float> Gear = new List<float>() { 1.0f };

    public void ToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("ctrllimited", $"{CtrlLimited}".ToLowerInvariant());
      mjcf.SetAttribute("forcelimited", $"{ForceLimited}".ToLowerInvariant());
      mjcf.SetAttribute(
          "ctrlrange",
          MjEngineTool.MakeLocaleInvariant($"{MjEngineTool.GetSorted(CtrlRange).x} {MjEngineTool.GetSorted(CtrlRange).y}"));
      mjcf.SetAttribute(
          "forcerange",
          MjEngineTool.MakeLocaleInvariant($"{MjEngineTool.GetSorted(ForceRange).x} {MjEngineTool.GetSorted(ForceRange).y}"));
      mjcf.SetAttribute(
          "lengthrange",
          MjEngineTool.MakeLocaleInvariant($"{MjEngineTool.GetSorted(LengthRange).x} {MjEngineTool.GetSorted(LengthRange).y}"));
      mjcf.SetAttribute("gear", MjEngineTool.ListToMjcf(Gear));
    }

    public void FromMjcf(XmlElement mjcf) {
      CtrlRange = mjcf.GetVector2Attribute("ctrlrange", defaultValue: Vector2.zero);
      ForceRange = mjcf.GetVector2Attribute("forcerange", defaultValue: Vector2.zero);
      LengthRange = mjcf.GetVector2Attribute("lengthrange", defaultValue: Vector2.zero);
      Gear = mjcf.GetFloatArrayAttribute("gear", defaultValue: new float[] { 1.0f }).ToList();

      CtrlLimited = mjcf.GetLimitedAttribute("ctrllimited",
          rangeDefined: mjcf.HasAttribute("ctrlrange"));
      ForceLimited = mjcf.GetLimitedAttribute("forcelimited",
          rangeDefined: mjcf.HasAttribute("forcerange"));
    }
  }

  // This structure holds all parameters unique to each type of the actuator.
  //
  // Because UnityEditor doesn't handle polymorphism well, I decided to put all parameters here
  // and then create a custom editor (MjActuatorEditor), that will display only the values
  // relevant to the selected articulation type. The choice will be made based on the value of
  // the 'Type' field.
  //
  // All constants found in this class are copied from the official documentation, and can be found
  // here: http://mujoco.org/book/XMLreference.html#actuator
  [Serializable]
  public class CustomParameters {

    //// General actuator parameters.

    // Activation dynamics type for the actuator.
    public MujocoLib.mjtDyn DynType;

    // The gain and bias together determine the output of the force generation mechanism, which is
    // currently assumed to be affine. As already explained in Actuation model, the general formula
    // is:
    //   scalar_force = gain_term * (act or ctrl) + bias_term.
    // The formula uses the activation state when present, and the control otherwise.
    public MujocoLib.mjtGain GainType;

    // Bias type.
    public MujocoLib.mjtBias BiasType;

    // Activation dynamics parameters. The built-in activation types (except for muscle) use only
    // the first parameter, but we provide additional parameters in case user callbacks implement a
    // more elaborate model. The length of this array is not enforced by the parser, so the user can
    // enter as many parameters as needed.
    public List<float> DynPrm = new List<float>() { 1.0f, 0.0f, 0.0f };

    // Gain parameters. The built-in gain types (except for muscle) use only the first parameter,
    // but we provide additional parameters in case user callbacks implement a more elaborate model.
    // The length of this array is not enforced by the parser, so the user can enter as many
    // parameters as needed.
    public List<float> GainPrm = new List<float>() { 1.0f, 0.0f, 0.0f };

    // Bias parameters. The affine bias type uses three parameters. The length of this array is not
    // enforced by the parser, so the user can enter as many parameters as needed.
    public List<float> BiasPrm = new List<float>() { 0.0f, 0.0f, 0.0f };

    public void GeneralToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("dyntype", $"{DynType}".Substring(6).ToLowerInvariant());
      mjcf.SetAttribute("gaintype", $"{GainType}".Substring(7).ToLowerInvariant());
      mjcf.SetAttribute("biastype", $"{BiasType}".Substring(7).ToLowerInvariant());
      mjcf.SetAttribute("dynprm", MjEngineTool.ListToMjcf(DynPrm));
      mjcf.SetAttribute("gainprm", MjEngineTool.ListToMjcf(GainPrm));
      mjcf.SetAttribute("biasprm", MjEngineTool.ListToMjcf(BiasPrm));
    }

    public void GeneralFromMjcf(XmlElement mjcf) {
      var dynTypeStr = mjcf.GetStringAttribute("dyntype", defaultValue: "none");
      var gainTypeStr = mjcf.GetStringAttribute("gaintype", defaultValue: "fixed");
      var biasTypeStr = mjcf.GetStringAttribute("biastype", defaultValue: "none");
      var ignoreCase = true;
      Enum.TryParse<MujocoLib.mjtDyn>($"mjdyn_{dynTypeStr}", ignoreCase, out DynType);
      Enum.TryParse<MujocoLib.mjtGain>($"mjgain_{gainTypeStr}", ignoreCase, out GainType);
      Enum.TryParse<MujocoLib.mjtBias>($"mjbias_{biasTypeStr}", ignoreCase, out BiasType);

      DynPrm = mjcf.GetFloatArrayAttribute(
        "dynprm", defaultValue: new float[] { 1.0f, 0.0f, 0.0f }).ToList();
      GainPrm = mjcf.GetFloatArrayAttribute(
        "gainprm", defaultValue: new float[] { 1.0f, 0.0f, 0.0f }).ToList();
      BiasPrm = mjcf.GetFloatArrayAttribute(
        "biasprm", defaultValue: new float[] { 0.0f, 0.0f, 0.0f }).ToList();
    }

    //// Position actuator parameters.

    // Position feedback gain.
    [AbsoluteValue]
    public float Kp = 1.0f;

    [AbsoluteValue]
    public float Kvp;

    public void PositionToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("kp", MjEngineTool.MakeLocaleInvariant($"{Math.Abs(Kp)}"));
      mjcf.SetAttribute("kv", MjEngineTool.MakeLocaleInvariant($"{Math.Abs(Kvp)}"));
    }
    public void PositionFromMjcf(XmlElement mjcf) {
      Kp = mjcf.GetFloatAttribute("kp", defaultValue: 1.0f);
      Kvp = mjcf.GetFloatAttribute("kv", defaultValue: 0f);
    }

    //// Velocity actuator parameters.

    // Velocity feedback gain.
    [AbsoluteValue]
    public float Kv = 1.0f;

    public void VelocityToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("kv", MjEngineTool.MakeLocaleInvariant($"{Math.Abs(Kv)}"));
    }
    public void VelocityFromMjcf(XmlElement mjcf) {
      Kv = mjcf.GetFloatAttribute("kv", defaultValue: 1.0f);
    }

    //// Cylinder actuator parameters.

    // Time constant of the activation dynamics.
    public float CylinderTimeConst = 1.0f;

    // Area of the cylinder. This is used internally as actuator gain.
    [AbsoluteValue]
    public float Area = 1.0f;

    // Instead of area the user can specify diameter. If both are specified, diameter has
    // precedence.
    [AbsoluteValue]
    public float Diameter = 0.0f;

    // Bias parameters, copied internally into biasprm.
    public float[] Bias = new float[] { 0.0f, 0.0f, 0.0f };

    public void CylinderToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("timeconst", MjEngineTool.MakeLocaleInvariant($"{CylinderTimeConst}"));
      mjcf.SetAttribute("area", MjEngineTool.MakeLocaleInvariant($"{Math.Abs(Area)}"));
      mjcf.SetAttribute("diameter", MjEngineTool.MakeLocaleInvariant($"{Math.Abs(Diameter)}"));
      mjcf.SetAttribute("bias", MjEngineTool.ArrayToMjcf(Bias));
    }

    public void CylinderFromMjcf(XmlElement mjcf) {
      CylinderTimeConst = mjcf.GetFloatAttribute("timeconst", defaultValue: 1.0f);
      Area = mjcf.GetFloatAttribute("area", defaultValue: 1.0f);
      Diameter = mjcf.GetFloatAttribute("diameter", defaultValue: 0.0f);
      Bias = mjcf.GetFloatArrayAttribute("bias", defaultValue: new float[] { 0.0f, 0.0f, 0.0f });
    }

    //// Muscle actuator parameters.

    // Time constants for activation and de-activation dynamics.
    public Vector2 MuscleTimeConst = new Vector2(0.01f, 0.04f);

    // Operating length range of the muscle, in units of L0.
    public Vector2 Range = new Vector2(0.75f, 1.05f);

    // Peak active force at rest. If this value is negative, the peak force is determined
    // automatically using the scale attribute below.
    public float Force = -1.0f;

    // If the force attribute is negative, the peak active force for the muscle is set to this value
    // divided by mjModel.actuator_acc0. The latter is the norm of the joint-space acceleration
    // vector caused by unit force on the actuator's transmission in qpos0. In other words, scaling
    // produces higher peak forces for muscles that pull more weight.
    public float Scale = 200.0f;

    // Lower position range of the normalized FLV curve, in units of L0.
    public float LMin = 0.5f;

    // Upper position range of the normalized FLV curve, in units of L0.
    public float LMax = 1.6f;

    // Shortening velocity at which muscle force drops to zero, in units of L0 per second.
    public float VMax = 1.5f;

    // Passive force generated at lmax, relative to the peak rest force.
    public float FpMax = 1.3f;

    // Active force generated at saturating lengthening velocity, relative to the peak rest force.
    public float FvMax = 1.2f;

    public void MuscleToMjcf(XmlElement mjcf) {
      mjcf.SetAttribute("timeconst", MjEngineTool.MakeLocaleInvariant($"{MuscleTimeConst[0]} {MuscleTimeConst[1]}"));
      mjcf.SetAttribute(
        "range", MjEngineTool.MakeLocaleInvariant($"{MjEngineTool.GetSorted(Range).x} {MjEngineTool.GetSorted(Range).y}"));
      mjcf.SetAttribute("force", MjEngineTool.MakeLocaleInvariant($"{Force}"));
      mjcf.SetAttribute("scale", MjEngineTool.MakeLocaleInvariant($"{Scale}"));
      mjcf.SetAttribute("lmin", MjEngineTool.MakeLocaleInvariant($"{LMin}"));
      mjcf.SetAttribute("lmax", MjEngineTool.MakeLocaleInvariant($"{LMax}"));
      mjcf.SetAttribute("vmax", MjEngineTool.MakeLocaleInvariant($"{VMax}"));
      mjcf.SetAttribute("fpmax", MjEngineTool.MakeLocaleInvariant($"{FpMax}"));
      mjcf.SetAttribute("fvmax", MjEngineTool.MakeLocaleInvariant($"{FvMax}"));
    }

    public void MuscleFromMjcf(XmlElement mjcf) {
      MuscleTimeConst = mjcf.GetVector2Attribute(
          "timeconst", defaultValue: new Vector2(0.01f, 0.04f));
      Range = mjcf.GetVector2Attribute("range", defaultValue: new Vector2(0.75f, 1.05f));
      Force = mjcf.GetFloatAttribute("force", defaultValue: -1.0f);
      Scale = mjcf.GetFloatAttribute("scale", defaultValue: 200.0f);
      LMin = mjcf.GetFloatAttribute("lmin", defaultValue: 0.5f);
      LMax = mjcf.GetFloatAttribute("lmax", defaultValue: 1.6f);
      VMax = mjcf.GetFloatAttribute("vmax", defaultValue: 1.5f);
      FpMax = mjcf.GetFloatAttribute("fpmax", defaultValue: 1.3f);
      FvMax = mjcf.GetFloatAttribute("fvmax", defaultValue: 1.2f);
    }
  }

  public ActuatorType Type;

  [Tooltip("Joint actuation target. Mutually exclusive with tendon target.")]
  public MjBaseJoint Joint;

  [Tooltip("Tendon actuation target. Mutually exclusive with joint target.")]
  public MjBaseTendon Tendon;

  [Tooltip("Parameters specific to each actuator type.")]
  [HideInInspector]
  public CustomParameters CustomParams = new CustomParameters();

  [Tooltip("Parameters shared by all types of actuators.")]
  public CommonParameters CommonParams = new CommonParameters();

  [Tooltip("Actuator control.")]
  public float Control;

  // Actuator length.
  public float Length { get; private set; }

  // Actuator velocity.
  public float Velocity { get; private set; }

  // Actuator force.
  public float Force { get; private set; }

  public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_ACTUATOR;

  // Parse the component settings from an external Mjcf.
  protected override void OnParseMjcf(XmlElement mjcf) {
    if (!Enum.TryParse(mjcf.Name, ignoreCase: true, result: out Type)) {
      throw new ArgumentException($"Unknown actuator type {mjcf.Name}.");
    }
    CommonParams.FromMjcf(mjcf);
    switch (Type) {
      case MjActuator.ActuatorType.General: {
        CustomParams.GeneralFromMjcf(mjcf);
        break;
      }
      case MjActuator.ActuatorType.Position: {
        CustomParams.PositionFromMjcf(mjcf);
        break;
      }
      case MjActuator.ActuatorType.Velocity: {
        CustomParams.VelocityFromMjcf(mjcf);
        break;
      }
      case MjActuator.ActuatorType.Cylinder: {
        CustomParams.CylinderFromMjcf(mjcf);
        break;
      }
      case MjActuator.ActuatorType.Muscle: {
        CustomParams.MuscleFromMjcf(mjcf);
        break;
      }
    }
    Joint = mjcf.GetObjectReferenceAttribute<MjBaseJoint>("joint");
    Tendon = mjcf.GetObjectReferenceAttribute<MjBaseTendon>("tendon");
  }

  // Generate implementation specific XML element.
  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    if (Joint == null && Tendon == null) {
      throw new InvalidOperationException($"Actuator {name} is not assigned a joint nor tendon.");
    }
    if (Joint != null && Tendon != null) {
      throw new InvalidOperationException(
        $"Actuator {name} can't have both a tendon and a joint target.");
    }

    var mjcf = doc.CreateElement(Type.ToString().ToLowerInvariant());
    if (Joint != null) {
      mjcf.SetAttribute("joint", Joint.MujocoName);
    } else {
      mjcf.SetAttribute("tendon", Tendon.MujocoName);
    }
    CommonParams.ToMjcf(mjcf);

    switch (Type) {
      case MjActuator.ActuatorType.General: {
        CustomParams.GeneralToMjcf(mjcf);
        break;
      }
      case MjActuator.ActuatorType.Position: {
        CustomParams.PositionToMjcf(mjcf);
        break;
      }
      case MjActuator.ActuatorType.Velocity: {
        CustomParams.VelocityToMjcf(mjcf);
        break;
      }
      case MjActuator.ActuatorType.Cylinder: {
        CustomParams.CylinderToMjcf(mjcf);
        break;
      }
      case MjActuator.ActuatorType.Muscle: {
        CustomParams.MuscleToMjcf(mjcf);
        break;
      }
    }
    return mjcf;
  }

  // Synchronize the state of the component.
  public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
    data->ctrl[MujocoId] = Control;
    Length = (float)data->actuator_length[MujocoId];
    Velocity = (float)data->actuator_velocity[MujocoId];
    Force = (float)data->actuator_force[MujocoId];
  }

  public void OnValidate() {
    if (Joint != null && Tendon != null) {
      Debug.LogError(
          $"Actuator {name} can't have both a tendon and a joint target.", this);
    }
  }
}
}
