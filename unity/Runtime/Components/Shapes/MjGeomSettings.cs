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

// Advanced settings of MjGeom component.
[Serializable]
public struct MjGeomSettings {

  public enum FluidShapeTypes {
    None,
    Ellipsoid,
  }

  [Tooltip("Priority")]
  public int Priority;

  [Tooltip("Solver accuracy settings.")]
  public GeomSolver Solver;

  [Tooltip("Collision filtering settings.")]
  public CollisionFiltering Filtering;

  [Tooltip("Contact friction parameters for dynamically generated contact pairs.")]
  public GeomFriction Friction;

  [Tooltip("Shape used for fluid simulation.")]
  public FluidShapeTypes FluidShapeType;

  [Tooltip("Dimensionless coefficients of fluid interaction model.")]
  public GeomFluidCoefficients FluidCoefficients;


  // Default geom settings.
  public static MjGeomSettings Default = new MjGeomSettings() {
    Solver = GeomSolver.Default,
    Filtering = CollisionFiltering.Default,
    Friction = GeomFriction.Default,
    FluidShapeType = FluidShapeTypes.None,
    FluidCoefficients = GeomFluidCoefficients.Default
  };

  public void FromMjcf(XmlElement mjcf) {

    Priority = (int) mjcf.GetFloatAttribute("priority", 0);

    // Contact filtering settings.
    Filtering.Contype = (int)mjcf.GetFloatAttribute("contype", CollisionFiltering.Default.Contype);
    Filtering.Conaffinity = (int)mjcf.GetFloatAttribute(
        "conaffinity", CollisionFiltering.Default.Conaffinity);
    Filtering.Group = (int)mjcf.GetFloatAttribute("group", CollisionFiltering.Default.Group);

    // Solver settings.
    Solver.ConDim = (int)mjcf.GetFloatAttribute("condim", GeomSolver.Default.ConDim);
    Solver.SolMix = mjcf.GetFloatAttribute("solmix", GeomSolver.Default.SolMix);
    var solref = mjcf.GetFloatArrayAttribute(
        "solref", new float[] { GeomSolver.Default.SolRef.TimeConst,
        GeomSolver.Default.SolRef.DampRatio });
    Solver.SolRef.TimeConst = solref[0];
    Solver.SolRef.DampRatio = solref[1];
    var solimp = mjcf.GetFloatArrayAttribute(
      "solimp", new float[] { GeomSolver.Default.SolImp.DMin, GeomSolver.Default.SolImp.DMax,
      GeomSolver.Default.SolImp.Width });
    Solver.SolImp.DMin = solimp[0];
    Solver.SolImp.DMax = solimp[1];
    Solver.SolImp.Width = solimp[2];
    Solver.Margin = mjcf.GetFloatAttribute("margin", GeomSolver.Default.Margin);
    Solver.Gap = mjcf.GetFloatAttribute("gap", GeomSolver.Default.Gap);

    // Inertia and friction settings.
    var friction = mjcf.GetFloatArrayAttribute(
        "friction", new float[] { GeomFriction.Default.Sliding, GeomFriction.Default.Torsional,
        GeomFriction.Default.Rolling });
    Friction.Sliding = friction[0];
    Friction.Torsional = friction[1];
    Friction.Rolling = friction[2];

    // Fluid settings.
    FluidShapeType = mjcf.GetEnumAttribute<FluidShapeTypes>(
        "fluidshape", FluidShapeTypes.None, ignoreCase: true);
    var fluidcoef = mjcf.GetFloatArrayAttribute("fluidcoef", new float[] {
        GeomFluidCoefficients.Default.BluntDrag, GeomFluidCoefficients.Default.SlenderDrag,
        GeomFluidCoefficients.Default.AngularDrag, GeomFluidCoefficients.Default.KuttaLift,
        GeomFluidCoefficients.Default.MagnusLift
    });
    FluidCoefficients.BluntDrag = fluidcoef[0];
    FluidCoefficients.SlenderDrag = fluidcoef[1];
    FluidCoefficients.AngularDrag = fluidcoef[2];
    FluidCoefficients.KuttaLift = fluidcoef[3];
    FluidCoefficients.MagnusLift = fluidcoef[4];
  }

  public void ToMjcf(XmlElement mjcf) {
    mjcf.SetAttribute("priority", $"{Priority}");

    // Contact filtering settings.
    mjcf.SetAttribute("contype", $"{Filtering.Contype}");
    mjcf.SetAttribute("conaffinity", $"{Filtering.Conaffinity}");
    mjcf.SetAttribute("group", $"{Filtering.Group}");

    // Solver settings.
    mjcf.SetAttribute("condim", $"{Solver.ConDim}");
    mjcf.SetAttribute("solmix", MjEngineTool.MakeLocaleInvariant($"{Solver.SolMix}"));
    mjcf.SetAttribute("solref", MjEngineTool.MakeLocaleInvariant($"{Solver.SolRef.TimeConst} {Solver.SolRef.DampRatio}"));
    mjcf.SetAttribute(
      "solimp", MjEngineTool.MakeLocaleInvariant($"{Solver.SolImp.DMin} {Solver.SolImp.DMax} {Solver.SolImp.Width}"));
    mjcf.SetAttribute("margin", MjEngineTool.MakeLocaleInvariant($"{Solver.Margin}"));
    mjcf.SetAttribute("gap", MjEngineTool.MakeLocaleInvariant($"{Solver.Gap}"));

    // Inertia and friction settings.
    mjcf.SetAttribute("friction", MjEngineTool.MakeLocaleInvariant($"{Friction.Sliding} {Friction.Torsional} {Friction.Rolling}"));

    // Fluid settings.
    mjcf.SetAttribute("fluidshape", FluidShapeType.ToString().ToLower());
    mjcf.SetAttribute("fluidcoef", MjEngineTool.MakeLocaleInvariant(
      $"{FluidCoefficients.BluntDrag} {FluidCoefficients.SlenderDrag} {FluidCoefficients.AngularDrag} {FluidCoefficients.KuttaLift} {FluidCoefficients.MagnusLift}"));
  }
}

[Serializable]
public struct GeomFriction {
  public float Sliding;
  public float Torsional;
  public float Rolling;

  public static GeomFriction Default = new GeomFriction {
      Sliding = 1.0f, Torsional = 0.005f, Rolling = 0.0001f
  };
}

[Serializable]
public struct CollisionFiltering {

  [Tooltip("Bitmasks used for contact filtering of dynamically generated contact pairs.")]
  public int Contype;

  [Tooltip("Bitmask for contact filtering")]
  public int Conaffinity;

  // The only effect on the physics is at compile time, when body masses and inertias are inferred
  // from geoms selected based on their group.
  [Tooltip("Group to which the geom belongs.")]
  public int Group;

  public static CollisionFiltering Default = new CollisionFiltering() {
    Contype = 1, Conaffinity = 1, Group = 0
  };
}

[Serializable]
public struct GeomSolver {
  [Tooltip("The dimensionality of the contact space.")]
  public int ConDim;

  [Tooltip("Weight used for averaging of constraint solver parameters.")]
  public float SolMix;

  [Tooltip("Solver function d(r) reference parameters.")]
  public SolverReference SolRef;

  [Tooltip("Solver function d(r) impedance parameters.")]
  public SolverImpedance SolImp;

  [Tooltip("Distance threshold below which contacts are detected.")]
  public float Margin;

  [Tooltip("Positive value enables generation of inactive contacts.")]
  public float Gap;

  public static GeomSolver Default = new GeomSolver() {
    ConDim = 3, SolMix = 1.0f, SolRef = SolverReference.Default, SolImp = SolverImpedance.Default,
    Margin = 0.0f, Gap = 0.0f
  };
}

[Serializable]
public struct GeomFluidCoefficients {
  public float BluntDrag;
  public float SlenderDrag;
  public float AngularDrag;
  public float KuttaLift;
  public float MagnusLift;

  public static GeomFluidCoefficients Default = new GeomFluidCoefficients() {
    BluntDrag = 5f, SlenderDrag = .25f, AngularDrag = 1.5f, KuttaLift = 1f, MagnusLift = 1f
  };
}
}
